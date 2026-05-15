#pragma once
#include "ProfileConstraints.hpp"
#include "StrandModelProfile.hpp"

namespace eco_sys_lab_package {
using namespace evo_engine;

/**
 * @brief Structure to hold demo data for particle physics.
 */
struct ParticlePhysicsDemoData {
  glm::vec4 m_color = glm::vec4(1.0f);  ///< Color of the particle.
};

/**
 * @brief Class representing a 2D particle physics simulation demo.
 */
class ParticlePhysics2DDemo : public IPrivateComponent {
  /// Profile for managing particles in the physics demo.
  StrandModelProfile<ParticlePhysicsDemoData> particle_physics_2d_;

  /// Constraints defining the boundaries for the simulation.
  ProfileConstraints profile_boundaries_;

  /// Flag to indicate if boundaries have been updated.
  bool boundaries_updated_ = false;

 public:
  glm::vec2 world_center = glm::vec2(0.0f);  ///< Center position of the world.
  float world_radius = 100.0f;               ///< Radius defining the boundary of the world.
  float gravity_strength = 10.0f;            ///< Strength of the gravitational force.
  int particle_add_count = 10;               ///< Number of particles to add in one step.

  /**
   * @brief Inspects and modifies the object's properties if needed.
   * @param editor_layer The editor layer handling the inspection.
   * @return True if the asset's content remains unchanged, otherwise false.
   */
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;

  /**
   * @brief Updates the particle physics in fixed time steps.
   */
  void FixedUpdate() override;
};
}  // namespace eco_sys_lab_package