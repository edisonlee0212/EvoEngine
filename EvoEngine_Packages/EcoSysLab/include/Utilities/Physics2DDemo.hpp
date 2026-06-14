#pragma once
#include "Physics2D.hpp"

namespace eco_sys_lab_package {
using namespace evo_engine;

/**
 * @brief Struct representing the data associated with a Physics2D simulation demo.
 */
struct Physics2DDemoData {
  /// The color used in the simulation.
  glm::vec4 color = glm::vec4(1.0f);
};

/**
 * @brief A class that demonstrates the usage of the Physics2D simulation.
 */
class Physics2DDemo : public IPrivateComponent {
  /// The 2D physics engine handling the simulation.
  Physics2D<Physics2DDemoData> physics_2d_;

 public:
  /// The center of the simulated world.
  glm::vec2 world_center = glm::vec2(0.0f);

  /// The radius of the simulated world.
  float world_radius = 10.0f;

  /// The normalized direction of gravity.
  glm::vec2 gravity_direction = glm::vec2(0, 1);

  /// The magnitude of gravitational acceleration.
  float gravity_strength = 9.7f;

  /// The friction coefficient applied in the simulation.
  float friction = 1.0f;

  /**
   * @brief Inspects the component properties in the editor.
   * @param editor_layer The editor layer handling the inspection.
   * @return True if the asset's content is unmodified during inspection; otherwise, false.
   */
  bool DrawGui(const std::shared_ptr<EditorLayer>& editor_layer);

  /**
   * @brief Updates the physics simulation at a fixed time step.
   */
  void FixedUpdate() override;
};
}  // namespace eco_sys_lab_package