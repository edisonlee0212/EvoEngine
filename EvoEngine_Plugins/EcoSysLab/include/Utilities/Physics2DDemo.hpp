#pragma once
#include "Physics2D.hpp"

using namespace evo_engine;
namespace eco_sys_lab {
struct Physics2DDemoData {
  glm::vec4 color = glm::vec4(1.0f);
};
class Physics2DDemo : public IPrivateComponent {
  Physics2D<Physics2DDemoData> physics_2d_;

 public:
  glm::vec2 world_center = glm::vec2(0.0f);
  float world_radius = 10.0f;
  glm::vec2 gravity_direction = glm::vec2(0, 1);
  float gravity_strength = 9.7f;
  float friction = 1.0f;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void FixedUpdate() override;
};
}  // namespace eco_sys_lab