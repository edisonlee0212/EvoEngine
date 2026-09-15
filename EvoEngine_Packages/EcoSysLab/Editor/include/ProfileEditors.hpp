#pragma once
#include "Physics2D.hpp"
#include "StrandModelProfile.hpp"
#include "imgui.h"
namespace eco_sys_lab_package {
struct ProfileCanvasState {
  ImGuiStorage* storage;
  ImGuiID x_key, y_key, zoom_key;
  glm::vec2 scrolling;
  float zoom_factor;
  ProfileCanvasState(const void* target, float initial_zoom) {
    ImGui::PushID(target);
    storage = ImGui::GetStateStorage();
    x_key = ImGui::GetID("scroll-x");
    y_key = ImGui::GetID("scroll-y");
    zoom_key = ImGui::GetID("zoom");
    scrolling = {storage->GetFloat(x_key), storage->GetFloat(y_key)};
    zoom_factor = storage->GetFloat(zoom_key, initial_zoom);
  }
  ~ProfileCanvasState() {
    storage->SetFloat(x_key, scrolling.x);
    storage->SetFloat(y_key, scrolling.y);
    storage->SetFloat(zoom_key, zoom_factor);
    ImGui::PopID();
  }
};

template <typename T>
void DrawPhysicsCanvas(Physics2D<T>& target, const std::function<void(glm::vec2 position)>& func,
                       const std::function<void(ImVec2 origin, float zoom_factor, ImDrawList*)>& draw_func) {
  ProfileCanvasState state(&target, 1.f);
  auto& scrolling = state.scrolling;
  auto& zoom_factor = state.zoom_factor;
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
  for (const auto& particle : target.PeekRigidBodies()) {
    const auto& point_position = particle.GetPosition();
    const auto& point_radius = particle.GetRadius();
    const auto& point_color = particle.GetColor();
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

template <typename T>
void DrawProfileCanvas(StrandModelProfile<T>& target, const std::function<void(glm::vec2 position)>& func,
                       const std::function<void(ImVec2 origin, float zoom_factor, ImDrawList*)>& draw_func,
                       bool show_grid = false) {
  ProfileCanvasState state(&target, 5.f);
  auto& scrolling = state.scrolling;
  auto& zoom_factor = state.zoom_factor;
  ImGui::Text(("Particle count: " + std::to_string(target.PeekParticles().size()) +
               " | Simulation time: " + std::to_string(target.GetLastSimulationTime()))
                  .c_str());

  if (ImGui::Button("Recenter")) {
    scrolling = glm::vec2(0.0f);
  }

  ImGui::SameLine();
  ImGui::DragFloat("Zoom", &zoom_factor, zoom_factor / 100.0f, 0.1f, 1000.0f);
  zoom_factor = glm::clamp(zoom_factor, 0.01f, 1000.0f);
  const ImGuiIO& io = ImGui::GetIO();
  ImDrawList* draw_list = ImGui::GetWindowDrawList();

  const ImVec2 canvas_p0 = ImGui::GetCursorScreenPos();
  ImVec2 canvas_sz = ImGui::GetContentRegionAvail();

  if (canvas_sz.x < 300.0f)
    canvas_sz.x = 300.0f;
  if (canvas_sz.y < 300.0f)
    canvas_sz.y = 300.0f;

  const ImVec2 canvas_p1 = ImVec2(canvas_p0.x + canvas_sz.x, canvas_p0.y + canvas_sz.y);
  const ImVec2 origin(canvas_p0.x + canvas_sz.x / 2.0f + scrolling.x, canvas_p0.y + canvas_sz.y / 2.0f + scrolling.y);

  const ImVec2 mouse_pos_in_canvas((io.MousePos.x - origin.x) / zoom_factor, (io.MousePos.y - origin.y) / zoom_factor);

  draw_list->AddRectFilled(canvas_p0, canvas_p1, IM_COL32(50, 50, 50, 255));
  draw_list->AddRect(canvas_p0, canvas_p1, IM_COL32(255, 255, 255, 255));

  ImGui::InvisibleButton("canvas", canvas_sz, ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight);
  const bool is_mouse_hovered = ImGui::IsItemHovered();
  const bool is_mouse_active = ImGui::IsItemActive();

  if (is_mouse_active && ImGui::IsMouseDragging(ImGuiMouseButton_Right, -1.0f)) {
    scrolling.x += io.MouseDelta.x;
    scrolling.y += io.MouseDelta.y;
  }

  if (const ImVec2 drag_delta = ImGui::GetMouseDragDelta(ImGuiMouseButton_Right);
      drag_delta.x == 0.0f && drag_delta.y == 0.0f)
    ImGui::OpenPopupOnItemClick("context", ImGuiPopupFlags_MouseButtonRight);

  if (ImGui::BeginPopup("context")) {
    ImGui::EndPopup();
  }

  draw_list->PushClipRect(canvas_p0, canvas_p1, true);

  if (is_mouse_hovered && ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
    func(glm::vec2(mouse_pos_in_canvas.x, mouse_pos_in_canvas.y));
  }

  const size_t mod = target.PeekParticles().size() / 15000;
  int index = 0;

  for (const auto& particle : target.PeekParticles()) {
    index++;
    if (mod > 1 && index % mod != 0)
      continue;

    const auto& point_position = particle.GetPosition();
    const auto& point_color = particle.GetColor();
    const auto canvas_position =
        ImVec2(origin.x + point_position.x * zoom_factor, origin.y + point_position.y * zoom_factor);

    draw_list->AddCircleFilled(canvas_position, glm::clamp(zoom_factor, 1.0f, 100.0f),
                               IM_COL32(255.0f * point_color.x, 255.0f * point_color.y, 255.0f * point_color.z,
                                        particle.IsBoundary() ? 255.0f : 128.0f));
  }

  draw_list->AddCircle(origin, glm::clamp(zoom_factor, 1.0f, 100.0f), IM_COL32(255, 0, 0, 255));

  if (show_grid) {
    for (int i = 0; i < target.particle_grid_2d.GetResolution().x; i++) {
      for (int j = 0; j < target.particle_grid_2d.GetResolution().y; j++) {
        const auto& cell = target.particle_grid_2d.RefCell(glm::ivec2(i, j));
        const auto cell_center = target.particle_grid_2d.GetPosition(glm::ivec2(i, j));
        const auto min = ImVec2(cell_center.x - target.particle_grid_2d.GetCellSize() * 0.5f,
                                cell_center.y - target.particle_grid_2d.GetCellSize() * 0.5f);

        draw_list->AddQuad(
            min * zoom_factor + origin,
            ImVec2(min.x + target.particle_grid_2d.GetCellSize(), min.y) * zoom_factor + origin,
            ImVec2(min.x + target.particle_grid_2d.GetCellSize(), min.y + target.particle_grid_2d.GetCellSize()) *
                    zoom_factor +
                origin,
            ImVec2(min.x, min.y + target.particle_grid_2d.GetCellSize()) * zoom_factor + origin,
            IM_COL32(0, 0, 255, 128));

        const auto cell_target = cell_center + cell.target;
        draw_list->AddLine(ImVec2(cell_center.x, cell_center.y) * zoom_factor + origin,
                           ImVec2(cell_target.x, cell_target.y) * zoom_factor + origin, IM_COL32(255, 0, 0, 128));
      }
    }
  }

  draw_func(origin, zoom_factor, draw_list);
  draw_list->PopClipRect();
}

template <typename ParticleData>
void DrawProfileEdges(StrandModelProfile<ParticleData>& target, ImVec2 origin, float zoom_factor, ImDrawList* draw_list,
                      ImU32 color, float thickness) {
  if (target.PeekEdges().empty())
    return;

  for (const auto& edge : target.PeekEdges()) {
    const auto& p1 = target.PeekParticles()[edge.first].GetPosition();
    const auto& p2 = target.PeekParticles()[edge.second].GetPosition();

    draw_list->AddLine(ImVec2(origin.x + p1.x * zoom_factor, origin.y + p1.y * zoom_factor),
                       ImVec2(origin.x + p2.x * zoom_factor, origin.y + p2.y * zoom_factor), color, thickness);
  }
}

template <typename ParticleData>
void DrawProfileBoundary(StrandModelProfile<ParticleData>& target, ImVec2 origin, float zoom_factor,
                         ImDrawList* draw_list, ImU32 color, float thickness) {
  if (target.PeekBoundaryEdges().empty())
    return;

  for (const auto& edge : target.PeekBoundaryEdges()) {
    const auto& p1 = target.PeekParticles()[edge.first].GetPosition();
    const auto& p2 = target.PeekParticles()[edge.second].GetPosition();

    draw_list->AddLine(ImVec2(origin.x + p1.x * zoom_factor, origin.y + p1.y * zoom_factor),
                       ImVec2(origin.x + p2.x * zoom_factor, origin.y + p2.y * zoom_factor), color, thickness);
  }
}
void DrawProfileBoundary(const ProfileBoundary& target, ImVec2 origin, float zoom_factor, ImDrawList* draw_list,
                         ImU32 color, float thickness);
void DrawProfileAttractor(const ProfileAttractor& target, ImVec2 origin, float zoom_factor, ImDrawList* draw_list,
                          ImU32 color, float thickness);
}  // namespace eco_sys_lab_package
