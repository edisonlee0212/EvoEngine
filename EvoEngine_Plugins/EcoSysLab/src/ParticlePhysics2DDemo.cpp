#include "ParticlePhysics2DDemo.hpp"

#include <Times.hpp>

#include "TreeVisualizer.hpp"
using namespace eco_sys_lab;

bool ParticlePhysics2DDemo::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  static bool enable_render = true;
  static float delta_time = 0.002f;
  ImGui::DragFloat("Simulation Delta time", &delta_time, 0.001f, 0.001f, 1.0f);
  if (ImGui::Button("Reset")) {
    particle_physics_2d_.Reset(delta_time);
  }
  ImGui::DragFloat("Particle Softness", &particle_physics_2d_.particle_physics_settings.particle_softness, 0.001f, 0.001f, 1.0f);
  ImGui::Checkbox("Enable render", &enable_render);
  ImGui::DragFloat2("World center", &world_center.x, 0.001f);
  ImGui::DragFloat("World radius", &world_radius, 1.0f, 1.0f, 1000.0f);
  ImGui::DragFloat("Gravity strength", &gravity_strength, 0.01f);
  ImGui::DragInt("Particle Adding speed", &particle_add_count, 1, 1, 1000);
  ImGui::DragFloat("Target damping", &particle_physics_2d_.particle_physics_settings.damping, 0.01f, 0.0f, 1.0f);
  ImGui::DragFloat("Max Velocity", &particle_physics_2d_.particle_physics_settings.max_speed, 0.01f, 0.0f, 1.0f);
  static bool show_grid = false;
  ImGui::Checkbox("Show Grid", &show_grid);
  static float particle_initial_speed = 1.0f;
  ImGui::DragFloat("Particle Initial speed", &particle_initial_speed, 0.1f, 0.0f, 3.0f);
  if (enable_render) {
    const std::string tag = "ParticlePhysics2D Scene [" + std::to_string(GetOwner().GetIndex()) + "]";
    ImGui::SetNextWindowSize(ImVec2(300, 300), ImGuiCond_Appearing);
    if (ImGui::Begin(tag.c_str())) {
      glm::vec2 mouse_position{};
      static bool last_frame_clicked = false;
      bool mouse_down = false;
      static bool add_attractor = false;
      ImGui::Checkbox("Force resize grid", &particle_physics_2d_.force_reset_grid);
      ImGui::Checkbox("Attractor", &add_attractor);
      ImGui::SameLine();
      if (ImGui::Button("Clear boundaries")) {
        profile_boundaries_.boundaries.clear();
        boundaries_updated_ = true;
      }
      ImGui::SameLine();
      if (ImGui::Button("Clear attractors")) {
        profile_boundaries_.attractors.clear();
        boundaries_updated_ = true;
      }
      ImGui::SameLine();
      static float edge_length_limit = 8;
      static bool calculate_edges = false;
      ImGui::Checkbox("Calculate edges", &calculate_edges);
      if (calculate_edges) {
        particle_physics_2d_.CalculateBoundaries(edge_length_limit);
      }
      ImGui::DragFloat("Edge length limit", &edge_length_limit);
      static float elapsed_time = 0.0f;
      elapsed_time += Times::DeltaTime();
      particle_physics_2d_.OnInspect(
          [&](const glm::vec2 position) {
            if (editor_layer->GetKey(GLFW_KEY_LEFT_CONTROL) == KeyActionType::Press ||
                editor_layer->GetKey(GLFW_KEY_LEFT_CONTROL) == KeyActionType::Hold) {
              if (elapsed_time > Times::TimeStep()) {
                elapsed_time = 0.0f;
                for (int i = 0; i < particle_add_count; i++) {
                  const auto particle_handle = particle_physics_2d_.AllocateParticle();
                  auto& particle = particle_physics_2d_.RefParticle(particle_handle);
                  particle.SetColor(glm::vec4(glm::ballRand(1.0f), 1.0f));
                  particle.SetPosition(position + glm::circularRand(4.0f));
                  particle.SetVelocity(glm::vec2(particle_initial_speed, 0.0f) / static_cast<float>(Times::TimeStep()),
                                       particle_physics_2d_.GetDeltaTime());
                }
              }
            } else {
              mouse_down = true;
              mouse_position = position;
            }
          },
          [&](const ImVec2 origin, const float zoom_factor, ImDrawList* draw_list) {
            const auto wc = world_center * zoom_factor;
            draw_list->AddCircle(origin + ImVec2(wc.x, wc.y), world_radius * zoom_factor,
                                IM_COL32(255, 0, 0, 255));
            particle_physics_2d_.RenderEdges(origin, zoom_factor, draw_list, IM_COL32(0.0f, 0.0f, 128.0f, 128.0f), 1.0f);
            particle_physics_2d_.RenderBoundary(origin, zoom_factor, draw_list, IM_COL32(255.f, 255.f, 255.0f, 255.0f),
                                               4.0f);
            for (const auto& boundary : profile_boundaries_.boundaries) {
              boundary.RenderBoundary(origin, zoom_factor, draw_list, IM_COL32(255.0f, 0.0f, 0.0f, 255.0f), 2.0f);
            }
            for (const auto& attractor : profile_boundaries_.attractors) {
              attractor.RenderAttractor(origin, zoom_factor, draw_list, IM_COL32(0.0f, 255.0f, 0.0f, 255.0f), 2.0f);
            }
          },
          show_grid);
      static glm::vec2 attractor_start_mouse_position;
      if (last_frame_clicked) {
        if (mouse_down) {
          if (!add_attractor) {
            // Continue recording.
            if (glm::distance(mouse_position, profile_boundaries_.boundaries.back().points.back()) > 1.0f)
              profile_boundaries_.boundaries.back().points.emplace_back(mouse_position);
          } else {
            if (auto& attractor_points = profile_boundaries_.attractors.back().attractor_points; attractor_points.empty()) {
              if (glm::distance(attractor_start_mouse_position, mouse_position) > 1.0f) {
                attractor_points.emplace_back(attractor_start_mouse_position, mouse_position);
              }
            } else if (glm::distance(mouse_position, attractor_points.back().second) > 1.0f) {
              attractor_points.emplace_back(attractor_points.back().second, mouse_position);
            }
          }
        } else if (!profile_boundaries_.boundaries.empty()) {
          if (!add_attractor) {
            // Stop and check boundary.
            if (!profile_boundaries_.Valid(profile_boundaries_.boundaries.size() - 1)) {
              profile_boundaries_.boundaries.pop_back();
            } else {
              profile_boundaries_.boundaries.back().CalculateCenter();
              boundaries_updated_ = true;
            }
          } else {
            // Stop and check attractors.
            boundaries_updated_ = true;
          }
        }
      } else if (mouse_down) {
        // Start recording.
        if (!add_attractor) {
          profile_boundaries_.boundaries.emplace_back();
          profile_boundaries_.boundaries.back().points.push_back(mouse_position);
        } else {
          profile_boundaries_.attractors.emplace_back();
          attractor_start_mouse_position = mouse_position;
        }
      }
      last_frame_clicked = mouse_down;
    }
    ImGui::End();
  }
  return changed;
}

void ParticlePhysics2DDemo::FixedUpdate() {
  particle_physics_2d_.Simulate(
      Times::TimeStep() / particle_physics_2d_.GetDeltaTime(),
      [&](auto& grid, const bool grid_resized) {
        if (grid_resized || boundaries_updated_)
          grid.ApplyBoundaries(profile_boundaries_);
        boundaries_updated_ = false;
      },
      [&](auto& particle) {
        // Apply constraints
        auto acceleration = glm::vec2(0.f);
        if (!particle_physics_2d_.particle_grid_2d.PeekCells().empty()) {
          const auto& cell = particle_physics_2d_.particle_grid_2d.RefCell(particle.GetPosition());
          if (glm::length(cell.target) > glm::epsilon<float>()) {
            acceleration += gravity_strength * 10.0f * glm::normalize(cell.target);
          }
        }
        particle.SetAcceleration(acceleration);
      });
}
