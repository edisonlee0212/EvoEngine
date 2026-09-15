#pragma once
#include "InspectorRegistry.hpp"
#include "ParticlePhysics2DDemo.hpp"
#include "Physics2DDemo.hpp"
namespace eco_sys_lab_package {
struct Physics2DDemoInspector {
  bool enable_render = true;
  float target_damping = 0.1f;
  bool Inspect(evo_engine::InspectorContext& context, Physics2DDemo& target);
};
struct ParticlePhysics2DDemoInspector {
  bool enable_render = true;
  float delta_time = 0.002f;
  bool show_grid = false;
  float particle_initial_speed = 1.0f;
  bool last_frame_clicked = false;
  bool add_attractor = false;
  float edge_length_limit = 8;
  bool calculate_edges = false;
  float elapsed_time = 0.0f;
  glm::vec2 attractor_start_mouse_position;
  bool Inspect(evo_engine::InspectorContext& context, ParticlePhysics2DDemo& target);
};
}  // namespace eco_sys_lab_package
