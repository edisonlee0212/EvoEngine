#pragma once
#include "TreeDescriptor.hpp"

namespace eco_sys_lab_package {
using namespace evo_engine;

/**
 * @class BasicReproductionModuleDescriptor
 * @brief Represents a descriptor for a fruit asset within the EcoSysLab package.
 *
 * This class provides functionality to generate a thumbnail texture
 * for visual representation of the fruit asset.
 */
class BasicReproductionModuleDescriptor : public IReproductionModuleDescriptor {
 public:
  float flower_size = 0.05f;
  float fruit_size = 0.1f;

  /// Number of flowers/fruit per internode.
  int count_per_internode = 1;

  /// Variance in leaf positioning.
  SingleDistribution<float> stem_length = {0.01f, 0.0f};

  /// Variance in leaf rotation.
  float rotation_variance = 10.f;

  /// Default branching angle in degrees.
  SingleDistribution<float> branching_angle = {30.f, 0.0f};

  /// Maximum thickness at a node.
  float max_node_thickness = 1.0f;

  /// Minimum distance for the root.
  float min_root_distance = 0.0f;

  /// Maximum distance to the end node.
  float max_end_distance = 0.2f;

  /// Phototropism effect for flower.
  float phototropism = 0.9f;

  /// Gravitropism effect for fruit.
  float gravitropism = 0.9f;

  SingleDistribution<float> flower_activation_temperature = {17.5f, 1.f};
  SingleDistribution<float> flower_growth_rate = {0.15f, 0.01f};
  SingleDistribution<float> flower_hang_time = {10.f, .5f};
  SingleDistribution<float> pollination_time = {5.f, 1.f};
  SingleDistribution<float> fruit_activation_temperature = {20.f, 1.f};
  SingleDistribution<float> fruit_hang_time = {10.f, 1.f};
  SingleDistribution<float> fruit_growth_rate = {0.01f, 0.01f};

  /**
   * \brief Prepares a ShootGrowthController using current growth parameters.
   * \param reproduction_controller The controller to configure.
   */
  void PrepareController(ShootReproductionController& reproduction_controller) const override;

  /**
   * @brief Generates fruit transformation matrices based on internode information.
   * @param[out] matrices Vector to store the transformation matrices.
   * @param[in] internode_info Information about the skeleton node internode.
   * @param[in] tree_size The overall tree size.
   */
  void GenerateFruitMatrices(std::vector<glm::mat4>& matrices, const SkeletonNodeInfo& internode_info,
                             float tree_size) const override;

  /**
   * @brief Inspects the foliage descriptor in the editor.
   * @param[in] editor_layer Shared pointer to the editor layer.
   * @return True if the asset's content is not modified during inspection.
   */
  bool DrawGui(const std::shared_ptr<EditorLayer>& editor_layer);
};
}  // namespace eco_sys_lab_package
