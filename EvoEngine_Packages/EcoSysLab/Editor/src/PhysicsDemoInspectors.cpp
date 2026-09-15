#include "PhysicsDemoInspectors.hpp"
#include "EditorLayer.hpp"
#include "ProfileEditors.hpp"
using namespace evo_engine;
using namespace eco_sys_lab_package;

bool Physics2DDemoInspector::Inspect(InspectorContext& context, Physics2DDemo& target) {
  bool changed = false;

  if (ImGui::Button("Reset")) {
    target.physics_2d_ = {};
  }
  ImGui::Checkbox("Enable render", &enable_render);
  ImGui::DragFloat2("World center", &target.world_center.x, 0.01f);
  ImGui::DragFloat("World radius", &target.world_radius, 0.01f);
  ImGui::DragFloat("Gravity strength", &target.gravity_strength, 0.01f);
  ImGui::DragFloat("Friction", &target.friction, 0.1f);

  ImGui::DragFloat("Target damping", &target_damping, 0.01f);
  if (ImGui::Button("Apply damping")) {
    for (auto& particle : target.physics_2d_.RefRigidBodies()) {
      particle.SetDamping(target_damping);
    }
  }
  if (enable_render) {
    const std::string tag = "Physics2D Scene [" + std::to_string(target.GetOwner().GetIndex()) + "]";
    if (ImGui::Begin(tag.c_str())) {
      DrawPhysicsCanvas(
          target.physics_2d_,
          [&](glm::vec2 position) {
            const auto rigid_body_handle = target.physics_2d_.AllocateRigidBody();
            auto& particle = target.physics_2d_.RefRigidBody(rigid_body_handle);
            particle.SetColor(glm::vec4(glm::abs(glm::ballRand(1.0f)), 1.0f));
            particle.SetRadius(glm::linearRand(0.1f, 3.0f));
            particle.SetPosition(position);
          },
          [&](const ImVec2 origin, const float zoom_factor, ImDrawList* draw_list) {
            const auto wc = target.world_center * zoom_factor;
            draw_list->AddCircle(origin + ImVec2(wc.x, wc.y), target.world_radius * zoom_factor,
                                 IM_COL32(255, 0, 0, 255));
          });
    }
    ImGui::End();
  }
  return changed;
}

bool ParticlePhysics2DDemoInspector::Inspect(InspectorContext& context, ParticlePhysics2DDemo& target) {
  bool changed = false;

  ImGui::DragFloat("Simulation Delta time", &delta_time, 0.001f, 0.001f, 1.0f);
  if (ImGui::Button("Reset")) {
    target.particle_physics_2d_.Reset(delta_time);
  }
  ImGui::DragFloat("Particle Softness", &target.particle_physics_2d_.particle_physics_settings.particle_softness,
                   0.001f, 0.001f, 1.0f);
  ImGui::Checkbox("Enable render", &enable_render);
  ImGui::DragFloat2("World center", &target.world_center.x, 0.001f);
  ImGui::DragFloat("World radius", &target.world_radius, 1.0f, 1.0f, 1000.0f);
  ImGui::DragFloat("Gravity strength", &target.gravity_strength, 0.01f);
  ImGui::DragInt("Particle Adding speed", &target.particle_add_count, 1, 1, 1000);
  ImGui::DragFloat("Target damping", &target.particle_physics_2d_.particle_physics_settings.damping, 0.01f, 0.0f, 1.0f);
  ImGui::DragFloat("Max Velocity", &target.particle_physics_2d_.particle_physics_settings.max_speed, 0.01f, 0.0f, 1.0f);

  ImGui::Checkbox("Show Grid", &show_grid);

  ImGui::DragFloat("Particle Initial speed", &particle_initial_speed, 0.1f, 0.0f, 3.0f);
  if (enable_render) {
    const std::string tag = "ParticlePhysics2D Scene [" + std::to_string(target.GetOwner().GetIndex()) + "]";
    ImGui::SetNextWindowSize(ImVec2(300, 300), ImGuiCond_Appearing);
    if (ImGui::Begin(tag.c_str())) {
      glm::vec2 mouse_position{};

      bool mouse_down = false;

      ImGui::Checkbox("Force resize grid", &target.particle_physics_2d_.force_reset_grid);
      ImGui::Checkbox("Attractor", &add_attractor);
      ImGui::SameLine();
      if (ImGui::Button("Clear boundaries")) {
        target.profile_boundaries_.boundaries.clear();
        target.boundaries_updated_ = true;
      }
      ImGui::SameLine();
      if (ImGui::Button("Clear attractors")) {
        target.profile_boundaries_.attractors.clear();
        target.boundaries_updated_ = true;
      }
      ImGui::SameLine();

      ImGui::Checkbox("Calculate edges", &calculate_edges);
      if (calculate_edges) {
        target.particle_physics_2d_.CalculateBoundaries(edge_length_limit);
      }
      ImGui::DragFloat("Edge length limit", &edge_length_limit);

      elapsed_time += ApplicationContext::Get().GetTimes().DeltaTime();
      DrawProfileCanvas(
          target.particle_physics_2d_,
          [&](const glm::vec2 position) {
            if (context.editor_layer->GetKey(GLFW_KEY_LEFT_CONTROL) == Input::KeyActionType::Press ||
                context.editor_layer->GetKey(GLFW_KEY_LEFT_CONTROL) == Input::KeyActionType::Hold) {
              if (elapsed_time > ApplicationContext::Get().GetTimes().TimeStep()) {
                elapsed_time = 0.0f;
                for (int i = 0; i < target.particle_add_count; i++) {
                  const auto particle_handle = target.particle_physics_2d_.AllocateParticle();
                  auto& particle = target.particle_physics_2d_.RefParticle(particle_handle);
                  particle.SetColor(glm::vec4(glm::ballRand(1.0f), 1.0f));
                  particle.SetPosition(position + glm::circularRand(4.0f));
                  particle.SetVelocity(glm::vec2(particle_initial_speed, 0.0f) /
                                           static_cast<float>(ApplicationContext::Get().GetTimes().TimeStep()),
                                       target.particle_physics_2d_.GetDeltaTime());
                }
              }
            } else {
              mouse_down = true;
              mouse_position = position;
            }
          },
          [&](const ImVec2 origin, const float zoom_factor, ImDrawList* draw_list) {
            const auto wc = target.world_center * zoom_factor;
            draw_list->AddCircle(origin + ImVec2(wc.x, wc.y), target.world_radius * zoom_factor,
                                 IM_COL32(255, 0, 0, 255));
            DrawProfileEdges(target.particle_physics_2d_, origin, zoom_factor, draw_list,
                             IM_COL32(0.0f, 0.0f, 128.0f, 128.0f), 1.0f);
            DrawProfileBoundary(target.particle_physics_2d_, origin, zoom_factor, draw_list,
                                IM_COL32(255.f, 255.f, 255.0f, 255.0f), 4.0f);
            for (const auto& boundary : target.profile_boundaries_.boundaries) {
              DrawProfileBoundary(boundary, origin, zoom_factor, draw_list, IM_COL32(255.0f, 0.0f, 0.0f, 255.0f), 2.0f);
            }
            for (const auto& attractor : target.profile_boundaries_.attractors) {
              DrawProfileAttractor(attractor, origin, zoom_factor, draw_list, IM_COL32(0.0f, 255.0f, 0.0f, 255.0f),
                                   2.0f);
            }
          },
          show_grid);

      if (last_frame_clicked) {
        if (mouse_down) {
          if (!add_attractor) {
            // Continue recording.
            if (glm::distance(mouse_position, target.profile_boundaries_.boundaries.back().points.back()) > 1.0f)
              target.profile_boundaries_.boundaries.back().points.emplace_back(mouse_position);
          } else {
            if (auto& attractor_points = target.profile_boundaries_.attractors.back().attractor_points;
                attractor_points.empty()) {
              if (glm::distance(attractor_start_mouse_position, mouse_position) > 1.0f) {
                attractor_points.emplace_back(attractor_start_mouse_position, mouse_position);
              }
            } else if (glm::distance(mouse_position, attractor_points.back().second) > 1.0f) {
              attractor_points.emplace_back(attractor_points.back().second, mouse_position);
            }
          }
        } else if (!target.profile_boundaries_.boundaries.empty()) {
          if (!add_attractor) {
            // Stop and check boundary.
            if (!target.profile_boundaries_.Valid(target.profile_boundaries_.boundaries.size() - 1)) {
              target.profile_boundaries_.boundaries.pop_back();
            } else {
              target.profile_boundaries_.boundaries.back().CalculateCenter();
              target.boundaries_updated_ = true;
            }
          } else {
            // Stop and check attractors.
            target.boundaries_updated_ = true;
          }
        }
      } else if (mouse_down) {
        // Start recording.
        if (!add_attractor) {
          target.profile_boundaries_.boundaries.emplace_back();
          target.profile_boundaries_.boundaries.back().points.push_back(mouse_position);
        } else {
          target.profile_boundaries_.attractors.emplace_back();
          attractor_start_mouse_position = mouse_position;
        }
      }
      last_frame_clicked = mouse_down;
    }
    ImGui::End();
  }
  return changed;
}
