#pragma once
#include "ProfileConstraints.hpp"
#include "StrandModelProfile.hpp"
using namespace evo_engine;
namespace eco_sys_lab {
struct ParticlePhysicsDemoData {
  glm::vec4 m_color = glm::vec4(1.0f);
};
class ParticlePhysics2DDemo : public IPrivateComponent {
  StrandModelProfile<ParticlePhysicsDemoData> particle_physics_2d_;
  ProfileConstraints profile_boundaries_;
  bool boundaries_updated_ = false;

 public:
  glm::vec2 world_center = glm::vec2(0.0f);
  float world_radius = 100.0f;
  float gravity_strength = 10.0f;
  int particle_add_count = 10;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void FixedUpdate() override;
};
}  // namespace eco_sys_lab