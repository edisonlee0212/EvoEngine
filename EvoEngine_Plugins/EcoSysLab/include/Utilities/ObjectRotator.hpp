
#pragma once

namespace eco_sys_lab_plugin {
using namespace evo_engine;

/**
 * @class ObjectRotator
 * @brief A component that handles object rotation within the EcoSysLab plugin.
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
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;

  /**
   * @brief Called every fixed update step to handle physics-based updates.
   */
  void FixedUpdate() override;

  /**
   * @brief Serializes the component data to a YAML emitter.
   *
   * @param out The YAML emitter used for output.
   */
  void Serialize(YAML::Emitter& out) const override;

  /**
   * @brief Deserializes component data from a YAML node.
   *
   * @param in The YAML node containing serialized data.
   */
  void Deserialize(const YAML::Node& in) override;
};

}  // namespace eco_sys_lab_plugin
