#pragma once
#include "TreeDescriptor.hpp"

using namespace evo_engine;

namespace eco_sys_lab_plugin {

/**
 * @class BasicFruitDescriptor
 * @brief Represents a descriptor for a fruit asset within the EcoSysLab plugin.
 *
 * This class provides functionality to generate a thumbnail texture
 * for visual representation of the fruit asset.
 */
class BasicFruitDescriptor : public IFruitDescriptor {
 public:
  /**
   * \brief The minimum lighting required for fruit flushing.
   */
  float fruit_flushing_lighting_requirement = 0.1f;

  /**
   * \brief Probability of fruit fall.
   */
  float fruit_fall_probability;

  /**
   * \brief Maximum allowed distance between a fruit and the nearest branch end.
   */
  float fruit_distance_to_branch_end_limit;
  /**
   * \brief Prepares a ShootGrowthController using current growth parameters.
   * \param shoot_growth_controller The controller to configure.
   */
  void PrepareGrowthController(ShootGrowthController& shoot_growth_controller) const;

  /**
   * @brief Generates fruit transformation matrices based on internode information.
   * @param[out] matrices Vector to store the transformation matrices.
   * @param[in] internode_info Information about the skeleton node internode.
   * @param[in] tree_size The overall tree size.
   */
  void GenerateFruitMatrices(std::vector<glm::mat4>& matrices, const SkeletonNodeInfo& internode_info,
                             float tree_size) const override;

  /**
   * @brief Serializes the foliage descriptor to a YAML emitter.
   * @param[out] out YAML emitter to store the serialized data.
   */
  void Serialize(YAML::Emitter& out) const override;

  /**
   * @brief Deserializes the foliage descriptor from a YAML node.
   * @param[in] in YAML node containing the serialized data.
   */
  void Deserialize(const YAML::Node& in) override;

  /**
   * @brief Inspects the foliage descriptor in the editor.
   * @param[in] editor_layer Shared pointer to the editor layer.
   * @return True if the asset's content is not modified during inspection.
   */
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
};

}  // namespace eco_sys_lab_plugin
