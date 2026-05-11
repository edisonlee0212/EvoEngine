#pragma once
#include "RigidBody2D.hpp"

namespace eco_sys_lab_package {
using namespace evo_engine;

/**
 * @brief Handle type for referencing RigidBody2D instances.
 */
typedef int RigidBodyHandle;

/**
 * @brief A 2D physics simulation system for rigid bodies.
 * @tparam T The data type used within the RigidBody2D class for physical calculations.
 */
template <typename T>
class Physics2D {
  /**
   * @brief A list of rigid bodies managed by the physics system.
   */
  std::vector<RigidBody2D<T>> rigid_bodies_2d_{};

  /**
   * @brief Resolves contact between two rigid bodies by adjusting their positions.
   * @param p1_handle Handle to the first rigid body.
   * @param p2_handle Handle to the second rigid body.
   */
  void SolveContact(RigidBodyHandle p1_handle, RigidBodyHandle p2_handle);

  /**
   * @brief The time step used for physics simulations.
   */
  float delta_time_ = 0.002f;

  /**
   * @brief Updates the states of all rigid bodies with an optional modification function.
   * @param modify_rigid_body_func Function used to modify each rigid body before physics updates.
   */
  void Update(const std::function<void(RigidBody2D<T>& rigid_body)>& modify_rigid_body_func);

 public:
  /**
   * @brief Allocates a new rigid body and returns its handle.
   * @return Handle to the newly created rigid body.
   */
  [[nodiscard]] RigidBodyHandle AllocateRigidBody();

  /**
   * @brief Retrieves a reference to a rigid body using its handle.
   * @param handle The handle of the rigid body.
   * @return Reference to the rigid body.
   */
  [[nodiscard]] RigidBody2D<T>& RefRigidBody(RigidBodyHandle handle);

  /**
   * @brief Removes a rigid body from the system.
   * @param handle The handle of the rigid body to remove.
   */
  void RemoveRigidBody(RigidBodyHandle handle);

  /**
   * @brief Shifts all rigid bodies by a specified offset.
   * @param offset The amount to move all rigid bodies.
   */
  void Shift(const glm::vec2& offset);

  /**
   * @brief Provides a read-only view of all rigid bodies managed by the system.
   * @return A constant reference to the list of rigid bodies.
   */
  [[nodiscard]] const std::vector<RigidBody2D<T>>& PeekRigidBodies() const;

  /**
   * @brief Provides a modifiable reference to all rigid bodies managed by the system.
   * @return A reference to the list of rigid bodies.
   */
  [[nodiscard]] std::vector<RigidBody2D<T>>& RefRigidBodies();

  /**
   * @brief Simulates the physics system over a duration.
   * @param time The time duration to simulate.
   * @param modify_rigid_body_func Function called to modify each rigid body before updates.
   */
  void Simulate(float time, const std::function<void(RigidBody2D<T>& rigid_body)>& modify_rigid_body_func);

  /**
   * @brief Handles inspection and visualization of rigid bodies in the editor.
   * @param func Function to call on user interaction with the editor UI.
   * @param draw_func Function to handle drawing operations in the editor.
   */
  void OnInspect(const std::function<void(glm::vec2 position)>& func,
                 const std::function<void(ImVec2 origin, float zoom_factor, ImDrawList*)>& draw_func);
};

template <typename T>
void Physics2D<T>::SolveContact(RigidBodyHandle p1_handle, RigidBodyHandle p2_handle) {
  if (p1_handle == p2_handle)
    return;
  auto& p1 = rigid_bodies_2d_.at(p1_handle);
  auto& p2 = rigid_bodies_2d_.at(p2_handle);
  const auto difference = p1.position_ - p2.position_;
  const auto distance = glm::length(difference);
  const auto min_distance = p1.thickness_ + p2.thickness_;
  if (distance < min_distance) {
    const auto axis = distance < glm::epsilon<float>() ? glm::vec2(1, 0) : difference / distance;
    const auto delta = min_distance - distance;
    p1.position_ += 0.5f * delta * axis;
    p2.position_ -= 0.5f * delta * axis;
  }
}

template <typename T>
void Physics2D<T>::Update(const std::function<void(RigidBody2D<T>& collision_rigid_body)>& modify_rigid_body_func) {
  Jobs::RunParallelFor(rigid_bodies_2d_.size(), [&](size_t i) {
    modify_rigid_body_func(rigid_bodies_2d_[i]);
  });
  for (size_t i = 0; i < rigid_bodies_2d_.size(); i++) {
    for (size_t j = 0; j < rigid_bodies_2d_.size(); j++) {
      SolveContact(i, j);
    }
  }
  Jobs::RunParallelFor(rigid_bodies_2d_.size(), [&](size_t i) {
    rigid_bodies_2d_[i].Update(delta_time_);
  });
}

template <typename T>
RigidBodyHandle Physics2D<T>::AllocateRigidBody() {
  rigid_bodies_2d_.emplace_back();
  return rigid_bodies_2d_.size() - 1;
}

template <typename T>
RigidBody2D<T>& Physics2D<T>::RefRigidBody(RigidBodyHandle handle) {
  return rigid_bodies_2d_[handle];
}

template <typename T>
void Physics2D<T>::RemoveRigidBody(RigidBodyHandle handle) {
  rigid_bodies_2d_[handle] = rigid_bodies_2d_.back();
  rigid_bodies_2d_.pop_back();
}

template <typename T>
void Physics2D<T>::Shift(const glm::vec2& offset) {
  Jobs::RunParallelFor(rigid_bodies_2d_.size(), [&](size_t i) {
    auto& particle = rigid_bodies_2d_[i];
    particle.SetPosition(particle.position_ + offset);
  });
}

template <typename T>
const std::vector<RigidBody2D<T>>& Physics2D<T>::PeekRigidBodies() const {
  return rigid_bodies_2d_;
}

template <typename T>
std::vector<RigidBody2D<T>>& Physics2D<T>::RefRigidBodies() {
  return rigid_bodies_2d_;
}

template <typename T>
void Physics2D<T>::Simulate(const float time,
                            const std::function<void(RigidBody2D<T>& collision_rigid_body)>& modify_rigid_body_func) {
  const auto count = static_cast<size_t>(glm::round(time / delta_time_));
  for (size_t i{count}; i--;) {
    Update(modify_rigid_body_func);
  }
}

template <typename T>
void Physics2D<T>::OnInspect(const std::function<void(glm::vec2 position)>& func,
                             const std::function<void(ImVec2 origin, float zoom_factor, ImDrawList*)>& draw_func) {
  static auto scrolling = glm::vec2(0.0f);
  static float zoom_factor = 1.f;
  if (ImGui::Button("Recenter")) {
    scrolling = glm::vec2(0.0f);
  }
  ImGui::DragFloat("Zoom", &zoom_factor, zoom_factor / 100.0f, 0.01f, 50.0f);
  zoom_factor = glm::clamp(zoom_factor, 0.01f, 50.0f);
  const ImGuiIO& io = ImGui::GetIO();
  ImDrawList* draw_list = ImGui::GetWindowDrawList();

  const ImVec2 canvas_p0 = ImGui::GetCursorScreenPos();  // ImDrawList API uses screen coordinates!
  ImVec2 canvas_sz = ImGui::GetContentRegionAvail();     // Resize canvas to what's available
  if (canvas_sz.x < 50.0f)
    canvas_sz.x = 50.0f;
  if (canvas_sz.y < 50.0f)
    canvas_sz.y = 50.0f;
  const ImVec2 canvas_p1 = ImVec2(canvas_p0.x + canvas_sz.x, canvas_p0.y + canvas_sz.y);
  const ImVec2 origin(canvas_p0.x + canvas_sz.x / 2.0f + scrolling.x,
                      canvas_p0.y + canvas_sz.y / 2.0f + scrolling.y);  // Lock scrolled origin
  const ImVec2 mouse_pos_in_canvas((io.MousePos.x - origin.x) / zoom_factor, (io.MousePos.y - origin.y) / zoom_factor);

  // Draw border and background color
  draw_list->AddRectFilled(canvas_p0, canvas_p1, IM_COL32(50, 50, 50, 255));
  draw_list->AddRect(canvas_p0, canvas_p1, IM_COL32(255, 255, 255, 255));

  // This will catch our interactions
  ImGui::InvisibleButton("canvas", canvas_sz, ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight);
  const bool is_mouse_hovered = ImGui::IsItemHovered();  // Hovered
  const bool is_mouse_active = ImGui::IsItemActive();    // Held

  // Pan (we use a zero mouse threshold when there's no context menu)
  // You may decide to make that threshold dynamic based on whether the mouse is hovering something etc.
  if (constexpr float mouse_threshold_for_pan = -1.0f;
      is_mouse_active && ImGui::IsMouseDragging(ImGuiMouseButton_Right, mouse_threshold_for_pan)) {
    scrolling.x += io.MouseDelta.x;
    scrolling.y += io.MouseDelta.y;
  }
  // Context menu (under default mouse threshold)
  if (const ImVec2 drag_delta = ImGui::GetMouseDragDelta(ImGuiMouseButton_Right);
      drag_delta.x == 0.0f && drag_delta.y == 0.0f)
    ImGui::OpenPopupOnItemClick("context", ImGuiPopupFlags_MouseButtonRight);
  if (ImGui::BeginPopup("context")) {
    ImGui::EndPopup();
  }

  // Draw profile + all lines in the canvas
  draw_list->PushClipRect(canvas_p0, canvas_p1, true);
  if (is_mouse_hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
    func(glm::vec2(mouse_pos_in_canvas.x, mouse_pos_in_canvas.y));
  }
  for (const auto& particle : rigid_bodies_2d_) {
    const auto& point_position = particle.position_;
    const auto& point_radius = particle.thickness_;
    const auto& point_color = particle.color_;
    const auto canvas_position =
        ImVec2(origin.x + point_position.x * zoom_factor, origin.y + point_position.y * zoom_factor);

    draw_list->AddCircleFilled(
        canvas_position, glm::clamp(zoom_factor * point_radius, 1.0f, 100.0f),
        IM_COL32(255.0f * point_color.x, 255.0f * point_color.y, 255.0f * point_color.z, 255.0f * point_color.w));
  }

  draw_list->AddCircle(origin, glm::clamp(0.5f * zoom_factor, 1.0f, 100.0f), IM_COL32(255, 0, 0, 255));

  draw_func(origin, zoom_factor, draw_list);
  draw_list->PopClipRect();
}
}  // namespace eco_sys_lab_package