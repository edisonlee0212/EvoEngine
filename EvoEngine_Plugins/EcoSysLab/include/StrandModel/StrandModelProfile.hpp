#pragma once
#include "Delaunator2D.hpp"
#include "Particle2D.hpp"
#include "ParticleGrid2D.hpp"
#include "Times.hpp"
using namespace evo_engine;
namespace eco_sys_lab {
struct ParticlePhysicsSettings {
  float particle_softness = 0.1f;
  float damping = 0.02f;
  float max_speed = 60.0f;
};

template <typename ParticleData>
class StrandModelProfile {
  template <typename Pd>
  friend class StrandModelProfileSerializer;

  std::vector<Particle2D<ParticleData>> particles_2d_{};
  void SolveCollision(ParticleHandle p1_handle, ParticleHandle p2_handle);
  float delta_time_ = 0.001f;
  void Update(const std::function<void(ParticleGrid2D& grid, bool grid_resized)>& modify_grid_func,
              const std::function<void(Particle2D<ParticleData>& particle)>& modify_particle_func);
  void CheckCollisions(const std::function<void(ParticleGrid2D& grid, bool grid_resized)>& modify_grid_func);
  glm::vec2 min_ = glm::vec2(FLT_MAX);
  glm::vec2 max_ = glm::vec2(FLT_MIN);
  float max_distance_to_center_ = 0.0f;
  glm::vec2 mass_center_ = glm::vec2(0.0f);
  double simulation_time_ = 0.0f;

  std::vector<std::pair<int, int>> edges_{};
  std::vector<std::pair<int, int>> boundary_edges_{};
  std::vector<glm::ivec3> triangles_{};

 public:
  [[nodiscard]] const std::vector<std::pair<int, int>>& PeekBoundaryEdges() const;
  [[nodiscard]] const std::vector<glm::ivec3>& PeekTriangles() const;
  [[nodiscard]] const std::vector<std::pair<int, int>>& PeekEdges() const;
  void RenderEdges(ImVec2 origin, float zoom_factor, ImDrawList* draw_list, ImU32 color, float thickness);
  void RenderBoundary(ImVec2 origin, float zoom_factor, ImDrawList* draw_list, ImU32 color, float thickness);
  void CalculateBoundaries(bool calculate_boundary_distance, float removal_length = 8);
  ParticleGrid2D particle_grid_2d{};
  bool force_reset_grid = false;
  [[nodiscard]] float GetDistanceToOrigin(const glm::vec2& direction, const glm::vec2& origin) const;
  [[nodiscard]] float GetDeltaTime() const;
  void SetEnableAllParticles(bool value);
  void Reset(float delta_time = 0.002f);
  void CalculateMinMax();
  ParticlePhysicsSettings particle_physics_settings{};
  [[nodiscard]] ParticleHandle AllocateParticle();
  [[nodiscard]] Particle2D<ParticleData>& RefParticle(ParticleHandle handle);
  [[nodiscard]] const Particle2D<ParticleData>& PeekParticle(ParticleHandle handle) const;
  void RemoveParticle(ParticleHandle handle);
  void Shift(const glm::vec2& offset);
  [[nodiscard]] const std::vector<Particle2D<ParticleData>>& PeekParticles() const;
  [[nodiscard]] std::vector<Particle2D<ParticleData>>& RefParticles();
  void SimulateByTime(float time, const std::function<void(ParticleGrid2D& grid, bool grid_resized)>& modify_grid_func,
                      const std::function<void(Particle2D<ParticleData>& particle)>& modify_particle_func);
  void Simulate(size_t iterations, const std::function<void(ParticleGrid2D& grid, bool grid_resized)>& modify_grid_func,
                const std::function<void(Particle2D<ParticleData>& particle)>& modify_particle_func);
  [[nodiscard]] glm::vec2 GetMassCenter() const;
  [[nodiscard]] float GetMaxDistanceToCenter() const;
  [[nodiscard]] glm::vec2 FindAvailablePosition(const glm::vec2& direction);
  [[nodiscard]] glm::vec2 CircularFindPosition(int index) const;
  [[nodiscard]] double GetLastSimulationTime() const;
  void OnInspect(const std::function<void(glm::vec2 position)>& func,
                 const std::function<void(ImVec2 origin, float zoom_factor, ImDrawList*)>& draw_func,
                 bool show_grid = false);
};

template <typename T>
void StrandModelProfile<T>::SolveCollision(ParticleHandle p1_handle, ParticleHandle p2_handle) {
  auto& p1 = particles_2d_.at(p1_handle);
  const auto& p2 = particles_2d_.at(p2_handle);
  if (!p1.enable)
    return;
  if (!p2.enable)
    return;

  const auto difference = p1.position_ - p2.position_;
  const auto distance = glm::length(difference);
  if (distance < 2.0f) {
    glm::vec2 axis;
    if (distance < glm::epsilon<float>()) {
      const auto dir = glm::circularRand(1.0f);
      if (p1_handle >= p2_handle) {
        axis = dir;
      } else {
        axis = -dir;
      }
    } else {
      axis = difference / distance;
    }
    const auto delta = 2.0f - distance;
    p1.delta_position_ += (1.0f - particle_physics_settings.particle_softness) * 0.5f * delta * axis;
  }
}

template <typename T>
void StrandModelProfile<T>::Update(const std::function<void(ParticleGrid2D& grid, bool grid_resized)>& modify_grid_func,
                                   const std::function<void(Particle2D<T>& collision_particle)>& modify_particle_func) {
  if (particles_2d_.empty())
    return;
  const auto start_time = Times::Now();

  for (auto& particle : particles_2d_) {
    if (particle.enable)
      modify_particle_func(particle);
    particle.delta_position_ = glm::vec2(0);
  }

  CheckCollisions(modify_grid_func);

  for (auto& particle : particles_2d_) {
    if (particle.enable)
      particle.position_ += particle.delta_position_;
    UpdateSettings update_settings{};
    update_settings.dt = delta_time_;
    update_settings.max_velocity = particle_physics_settings.max_speed;
    update_settings.damping = particle_physics_settings.damping;
    particle.Update(update_settings);
  }

  simulation_time_ = Times::Now() - start_time;
}

template <typename T>
void StrandModelProfile<T>::CalculateMinMax() {
  min_ = glm::vec2(FLT_MAX);
  max_ = glm::vec2(FLT_MIN);
  mass_center_ = glm::vec2(0.0f);
  max_distance_to_center_ = 0.0f;

  int enabled_particle_size = 0;
  for (const auto& particle : particles_2d_) {
    min_ = glm::min(particle.position_, min_);
    max_ = glm::max(particle.position_, max_);
    if (particle.enable) {
      enabled_particle_size++;
      mass_center_ += particle.position_;
      max_distance_to_center_ = glm::max(max_distance_to_center_, glm::length(particle.position_));
    }
  }
  mass_center_ /= enabled_particle_size;
}

template <typename T>
void StrandModelProfile<T>::CheckCollisions(
    const std::function<void(ParticleGrid2D& grid, bool grid_resized)>& modify_grid_func) {
  CalculateMinMax();

  if (min_.x < particle_grid_2d.min_bound_.x || min_.y < particle_grid_2d.min_bound_.y ||
      max_.x > particle_grid_2d.max_bound_.x || max_.y > particle_grid_2d.max_bound_.y || force_reset_grid) {
    particle_grid_2d.Reset(2.0f, min_ - glm::vec2(2.0f), max_ + glm::vec2(2.0f));
    modify_grid_func(particle_grid_2d, true);
  } else {
    particle_grid_2d.Clear();
    modify_grid_func(particle_grid_2d, false);
  }
  for (ParticleHandle i = 0; i < particles_2d_.size(); i++) {
    const auto& particle = particles_2d_[i];
    if (particle.enable)
      particle_grid_2d.RegisterParticle(particle.position_, i);
  }

  for (ParticleHandle particle_handle = 0; particle_handle < particles_2d_.size(); particle_handle++) {
    const auto& particle = particles_2d_[particle_handle];
    if (!particle.enable)
      continue;
    const auto& coordinate = particle_grid_2d.GetCoordinate(particle.position_);
    for (int dx = -1; dx <= 1; dx++) {
      for (int dy = -1; dy <= 1; dy++) {
        const auto x = coordinate.x + dx;
        const auto y = coordinate.y + dy;
        if (x < 0)
          continue;
        if (y < 0)
          continue;
        if (x >= particle_grid_2d.resolution_.x)
          continue;
        if (y >= particle_grid_2d.resolution_.y)
          continue;
        const auto& cell = particle_grid_2d.RefCell(glm::ivec2(x, y));
        for (int i = 0; i < cell.atom_count_; i++) {
          if (const auto particle_handle2 = cell.atom_handles_[i]; particle_handle != particle_handle2) {
            SolveCollision(particle_handle, particle_handle2);
          }
        }
      }
    }
  }
}

template <typename ParticleData>
const std::vector<std::pair<int, int>>& StrandModelProfile<ParticleData>::PeekBoundaryEdges() const {
  return boundary_edges_;
}

template <typename ParticleData>
const std::vector<glm::ivec3>& StrandModelProfile<ParticleData>::PeekTriangles() const {
  return triangles_;
}

template <typename ParticleData>
const std::vector<std::pair<int, int>>& StrandModelProfile<ParticleData>::PeekEdges() const {
  return edges_;
}

template <typename ParticleData>
void StrandModelProfile<ParticleData>::RenderEdges(ImVec2 origin, float zoom_factor, ImDrawList* draw_list, ImU32 color,
                                                   float thickness) {
  if (edges_.empty())
    return;
  for (const auto& edge : edges_) {
    const auto& p1 = particles_2d_[edge.first].position_;
    const auto& p2 = particles_2d_[edge.second].position_;
    draw_list->AddLine(ImVec2(origin.x + p1.x * zoom_factor, origin.y + p1.y * zoom_factor),
                      ImVec2(origin.x + p2.x * zoom_factor, origin.y + p2.y * zoom_factor), color, thickness);
  }
}

template <typename ParticleData>
void StrandModelProfile<ParticleData>::RenderBoundary(ImVec2 origin, float zoom_factor, ImDrawList* draw_list,
                                                      ImU32 color, float thickness) {
  if (boundary_edges_.empty())
    return;
  for (const auto& edge : boundary_edges_) {
    const auto& p1 = particles_2d_[edge.first].position_;
    const auto& p2 = particles_2d_[edge.second].position_;
    draw_list->AddLine(ImVec2(origin.x + p1.x * zoom_factor, origin.y + p1.y * zoom_factor),
                      ImVec2(origin.x + p2.x * zoom_factor, origin.y + p2.y * zoom_factor), color, thickness);
  }
}

template <typename ParticleData>
void StrandModelProfile<ParticleData>::CalculateBoundaries(const bool calculate_boundary_distance,
                                                           const float removal_length) {
  edges_.clear();
  boundary_edges_.clear();
  triangles_.clear();

  if (particles_2d_.size() < 3) {
    for (int i = 0; i < particles_2d_.size(); i++) {
      particles_2d_[i].boundary_ = true;
      particles_2d_[i].distance_to_boundary_ = 0;
    }
    return;
  }
  std::vector<float> positions(particles_2d_.size() * 2);
  for (int i = 0; i < particles_2d_.size(); i++) {
    positions[2 * i] = particles_2d_[i].position_.x;
    positions[2 * i + 1] = particles_2d_[i].position_.y;
    particles_2d_[i].boundary_ = false;
    particles_2d_[i].distance_to_boundary_ = 0;
  }
  const Delaunator::Delaunator2D d(positions);
  std::map<std::pair<int, int>, int> edges;
  for (std::size_t i = 0; i < d.triangles.size(); i += 3) {
    const auto& v0 = d.triangles[i];
    const auto& v1 = d.triangles[i + 1];
    const auto& v2 = d.triangles[i + 2];
    if (glm::distance(particles_2d_[v0].position_, particles_2d_[v1].position_) > removal_length ||
        glm::distance(particles_2d_[v1].position_, particles_2d_[v2].position_) > removal_length ||
        glm::distance(particles_2d_[v0].position_, particles_2d_[v2].position_) > removal_length)
      continue;
    ++edges[std::make_pair(glm::min(v0, v1), glm::max(v0, v1))];
    ++edges[std::make_pair(glm::min(v1, v2), glm::max(v1, v2))];
    ++edges[std::make_pair(glm::min(v0, v2), glm::max(v0, v2))];
    triangles_.emplace_back(glm::ivec3(v0, v1, v2));
  }
  std::set<int> boundary_vertices;
  for (const auto& edge : edges) {
    if (edge.second == 1) {
      boundary_edges_.emplace_back(edge.first);
      particles_2d_[edge.first.first].boundary_ = true;
      particles_2d_[edge.first.second].boundary_ = true;
      boundary_vertices.emplace(edge.first.first);
      boundary_vertices.emplace(edge.first.second);
    }
    edges_.emplace_back(edge.first);
  }

  if (calculate_boundary_distance) {
    for (auto& particle : particles_2d_) {
      if (particle.boundary_ || boundary_vertices.empty()) {
        particle.distance_to_boundary_ = 0.0f;
        continue;
      }
      particle.distance_to_boundary_ = FLT_MAX;
      const auto position = particle.GetPosition();
      for (const auto& boundary_particle_handle : boundary_vertices) {
        const auto& boundary_particle = particles_2d_[boundary_particle_handle];
        const auto current_distance = glm::distance(position, boundary_particle.GetPosition());
        if (particle.distance_to_boundary_ > current_distance) {
          particle.distance_to_boundary_ = current_distance;
        }
      }
    }
  }
}

template <typename T>
float StrandModelProfile<T>::GetDistanceToOrigin(const glm::vec2& direction, const glm::vec2& origin) const {
  float max_distance = FLT_MIN;

  for (const auto& particle : particles_2d_) {
    const auto distance =
        glm::length(glm::closestPointOnLine(particle.position_, glm::vec2(origin), origin + direction * 1000.0f));
    max_distance = glm::max(max_distance, distance);
  }
  return max_distance;
}

template <typename T>
float StrandModelProfile<T>::GetDeltaTime() const {
  return delta_time_;
}

template <typename ParticleData>
void StrandModelProfile<ParticleData>::SetEnableAllParticles(const bool value) {
  for (auto& particle : particles_2d_) {
    particle.enable = value;
  }
}

template <typename T>
void StrandModelProfile<T>::Reset(const float delta_time) {
  delta_time_ = delta_time;
  particles_2d_.clear();
  particle_grid_2d = {};
  mass_center_ = glm::vec2(0.0f);
  min_ = glm::vec2(FLT_MAX);
  max_ = glm::vec2(FLT_MIN);
  max_distance_to_center_ = 0.0f;
  mass_center_ = glm::vec2(0.0f);
  simulation_time_ = 0.0f;
}

template <typename T>
ParticleHandle StrandModelProfile<T>::AllocateParticle() {
  particles_2d_.emplace_back();
  auto& new_particle = particles_2d_.back();
  new_particle = {};
  new_particle.handle_ = particles_2d_.size() - 1;
  return new_particle.handle_;
}

template <typename T>
Particle2D<T>& StrandModelProfile<T>::RefParticle(ParticleHandle handle) {
  return particles_2d_[handle];
}

template <typename T>
const Particle2D<T>& StrandModelProfile<T>::PeekParticle(ParticleHandle handle) const {
  return particles_2d_[handle];
}

template <typename T>
void StrandModelProfile<T>::RemoveParticle(ParticleHandle handle) {
  particles_2d_[handle] = particles_2d_.back();
  particles_2d_.pop_back();
}

template <typename T>
void StrandModelProfile<T>::Shift(const glm::vec2& offset) {
  Jobs::RunParallelFor(particles_2d_.size(), [&](unsigned i) {
    auto& particle = particles_2d_[i];
    particle.SetPosition(particle.position_ + offset);
  });
}

template <typename T>
const std::vector<Particle2D<T>>& StrandModelProfile<T>::PeekParticles() const {
  return particles_2d_;
}

template <typename T>
std::vector<Particle2D<T>>& StrandModelProfile<T>::RefParticles() {
  return particles_2d_;
}

template <typename T>
void StrandModelProfile<T>::SimulateByTime(
    float time, const std::function<void(ParticleGrid2D& grid, bool grid_resized)>& modify_grid_func,
    const std::function<void(Particle2D<T>& collision_particle)>& modify_particle_func) {
  const auto count = static_cast<size_t>(glm::round(time / delta_time_));
  for (int i = 0; i < count; i++) {
    Update(modify_grid_func, modify_particle_func);
  }
}

template <typename T>
void StrandModelProfile<T>::Simulate(const size_t iterations,
                                     const std::function<void(ParticleGrid2D& grid, bool grid_resized)>& modify_grid_func,
                                     const std::function<void(Particle2D<T>& particle)>& modify_particle_func) {
  for (int i = 0; i < iterations; i++) {
    Update(modify_grid_func, modify_particle_func);
  }
}

template <typename T>
glm::vec2 StrandModelProfile<T>::GetMassCenter() const {
  return mass_center_;
}

template <typename ParticleData>
float StrandModelProfile<ParticleData>::GetMaxDistanceToCenter() const {
  return max_distance_to_center_;
}

template <typename ParticleData>
glm::vec2 StrandModelProfile<ParticleData>::FindAvailablePosition(const glm::vec2& direction) {
  auto ret_val = glm::vec2(0, 0);
  bool found = false;
  while (!found) {
    found = true;
    for (const auto& i : particles_2d_) {
      if (glm::distance(i.GetPosition(), ret_val) < 2.05f) {
        found = false;
        break;
      }
    }
    if (!found)
      ret_val += direction * 0.41f;
  }
  return ret_val;
}

template <typename ParticleData>
glm::vec2 StrandModelProfile<ParticleData>::CircularFindPosition(int index) const {
  if (index == 0)
    return glm::vec2(0.0f);
  int layer = 0;
  while (3 * layer * (layer + 1) <= index) {
    layer++;
  }
  const int edge_size = layer;
  const int layer_index = (index - 1) - 3 * (layer - 1) * layer;
  const int edge_index = layer_index / edge_size;
  const int index_in_edge = layer_index % edge_size;
  const glm::vec2 edge_direction =
      glm::vec2(glm::cos(glm::radians(edge_index * 60.0f)), glm::sin(glm::radians(edge_index * 60.0f)));
  const glm::vec2 walker_direction =
      glm::vec2(glm::cos(glm::radians((edge_index + 2) * 60.0f)), glm::sin(glm::radians((edge_index + 2) * 60.0f)));

  return edge_direction * static_cast<float>(layer) * 2.0f + walker_direction * static_cast<float>(index_in_edge) * 2.0f;
}

template <typename ParticleData>
double StrandModelProfile<ParticleData>::GetLastSimulationTime() const {
  return simulation_time_;
}

template <typename T>
void StrandModelProfile<T>::OnInspect(const std::function<void(glm::vec2 position)>& func,
                                      const std::function<void(ImVec2 origin, float zoom_factor, ImDrawList*)>& draw_func,
                                      bool show_grid) {
  static auto scrolling = glm::vec2(0.0f);
  static float zoom_factor = 5.f;
  ImGui::Text(("Particle count: " + std::to_string(particles_2d_.size()) +
               " | Simulation time: " + std::to_string(simulation_time_))
                  .c_str());
  if (ImGui::Button("Recenter")) {
    scrolling = glm::vec2(0.0f);
  }
  ImGui::SameLine();
  ImGui::DragFloat("Zoom", &zoom_factor, zoom_factor / 100.0f, 0.1f, 1000.0f);
  zoom_factor = glm::clamp(zoom_factor, 0.01f, 1000.0f);
  const ImGuiIO& io = ImGui::GetIO();
  ImDrawList* draw_list = ImGui::GetWindowDrawList();

  const ImVec2 canvas_p0 = ImGui::GetCursorScreenPos();  // ImDrawList API uses screen coordinates!
  ImVec2 canvas_sz = ImGui::GetContentRegionAvail();     // Resize canvas to what's available
  if (canvas_sz.x < 300.0f)
    canvas_sz.x = 300.0f;
  if (canvas_sz.y < 300.0f)
    canvas_sz.y = 300.0f;
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
  if (constexpr float mouse_threshold_for_pan = -1.0f; is_mouse_active && ImGui::IsMouseDragging(ImGuiMouseButton_Right, mouse_threshold_for_pan)) {
    scrolling.x += io.MouseDelta.x;
    scrolling.y += io.MouseDelta.y;
  }
  // Context menu (under default mouse threshold)
  if (const ImVec2 drag_delta = ImGui::GetMouseDragDelta(ImGuiMouseButton_Right); drag_delta.x == 0.0f && drag_delta.y == 0.0f)
    ImGui::OpenPopupOnItemClick("context", ImGuiPopupFlags_MouseButtonRight);
  if (ImGui::BeginPopup("context")) {
    ImGui::EndPopup();
  }

  // Draw profile + all lines in the canvas
  draw_list->PushClipRect(canvas_p0, canvas_p1, true);
  if (is_mouse_hovered && ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
    func(glm::vec2(mouse_pos_in_canvas.x, mouse_pos_in_canvas.y));
  }
  const size_t mod = particles_2d_.size() / 15000;
  int index = 0;
  for (const auto& particle : particles_2d_) {
    index++;
    if (mod > 1 && index % mod != 0)
      continue;
    const auto& point_position = particle.position_;
    const auto& point_color = particle.color_;
    const auto canvas_position =
        ImVec2(origin.x + point_position.x * zoom_factor, origin.y + point_position.y * zoom_factor);

    draw_list->AddCircleFilled(canvas_position, glm::clamp(zoom_factor, 1.0f, 100.0f),
                              IM_COL32(255.0f * point_color.x, 255.0f * point_color.y, 255.0f * point_color.z,
                                       particle.IsBoundary() ? 255.0f : 128.0f));
  }
  draw_list->AddCircle(origin, glm::clamp(zoom_factor, 1.0f, 100.0f), IM_COL32(255, 0, 0, 255));
  if (show_grid) {
    for (int i = 0; i < particle_grid_2d.resolution_.x; i++) {
      for (int j = 0; j < particle_grid_2d.resolution_.y; j++) {
        const auto& cell = particle_grid_2d.RefCell(glm::ivec2(i, j));
        const auto cell_center = particle_grid_2d.GetPosition(glm::ivec2(i, j));
        const auto min = ImVec2(cell_center.x - particle_grid_2d.cell_size_ * 0.5f,
                                cell_center.y - particle_grid_2d.cell_size_ * 0.5f);

        draw_list->AddQuad(
            min * zoom_factor + origin, ImVec2(min.x + particle_grid_2d.cell_size_, min.y) * zoom_factor + origin,
            ImVec2(min.x + particle_grid_2d.cell_size_, min.y + particle_grid_2d.cell_size_) * zoom_factor + origin,
            ImVec2(min.x, min.y + particle_grid_2d.cell_size_) * zoom_factor + origin, IM_COL32(0, 0, 255, 128));
        const auto cell_target = cell_center + cell.target;
        draw_list->AddLine(ImVec2(cell_center.x, cell_center.y) * zoom_factor + origin,
                          ImVec2(cell_target.x, cell_target.y) * zoom_factor + origin, IM_COL32(255, 0, 0, 128));
      }
    }
  }
  draw_func(origin, zoom_factor, draw_list);
  draw_list->PopClipRect();
}
}  // namespace eco_sys_lab
