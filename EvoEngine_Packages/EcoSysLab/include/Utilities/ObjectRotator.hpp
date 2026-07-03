#pragma once

#include "EditorLayer.hpp"
#include "IPrivateComponent.hpp"

namespace eco_sys_lab_package {
using namespace evo_engine;

/**
 * @class ObjectRotator
 * @brief A component that handles object rotation within the EcoSysLab package.
 *
 * This class allows objects to rotate at a specified speed and maintains
 * the current rotation as a 3D vector.
 */
class ObjectRotator : public IPrivateComponent {
 public:
  /**
   * @brief The speed at which the object rotates.
   */
  float rotate_speed;

  /**
   * @brief The current rotation of the object in 3D space.
   */
  glm::vec3 rotation = glm::vec3(0, 0, 0);

  /**
   * @brief Displays the inspection UI for the component.
   *
   * @param editor_layer A shared pointer to the editor layer.
   * @return true if the asset's content is not modified during inspection.
   */
  bool DrawGui(const std::shared_ptr<EditorLayer>& editor_layer);

  /**
   * @brief Called every fixed update step to handle physics-based updates.
   */
  void FixedUpdate() override;
};
}  // namespace eco_sys_lab_package
