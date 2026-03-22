#pragma once
#include "Skeleton.hpp"
#include "TreeDescriptor.hpp"

namespace eco_sys_lab_plugin {
using namespace evo_engine;

/**
 * @class BasicFoliageDescriptor
 * @brief Represents a foliage descriptor for procedural tree generation.
 *
 * This class defines parameters for generating foliage on a procedural tree model.
 * It includes attributes for leaf size, count, branching angles, and material references.
 */
class BasicFoliageDescriptor : public IFoliageDescriptor {
 public:
  /// Leaf size represented as width and height.
  glm::vec2 leaf_size = glm::vec2(0.04f, 0.08f);

  /// Number of leaves per internode.
  int leaf_count = 2;

  /// Probability [0..1] that an internode spawns leaves.
  float leaf_spawn_chance = 0.05f;

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

  /// Horizontal tropism effect.
  float horizontal_tropism = 0.f;

  /// Gravitropism effect.
  float gravitropism = 0.1f;

  SingleDistribution<float> activation_temperature = {7.5f, .5f};
  SingleDistribution<float> activation_light_intensity = {0.0f, 0.0f};
  SingleDistribution<float> growth_rate = {0.07f, 0.01f};
  SingleDistribution<float> damage_temperature = {10.f, 0.5f};
  SingleDistribution<float> damage_rate = {0.07f, 0.01f};
  SingleDistribution<float> hang_time = {10.f, 1.f};

  /// Reference to the leaf material asset (fallback when leaf_material_variants is empty).
  AssetRef leaf_material_ref;

  /// Material variants for leaf quads. Each leaf randomly selects one entry.
  /// If empty, falls back to leaf_material_ref.
  std::vector<AssetRef> leaf_material_variants;

    float leaf_source_strength = 1.0f;

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

  /**
   * @brief Collects asset references from this foliage descriptor.
   * @param[out] list Vector to collect asset references.
   */
  void CollectAssetRef(std::vector<AssetRef>& list) override;

  /**
   * \brief Prepares a FoliageController using current growth parameters.
   * \param foliage_controller The controller to configure.
   */
  void PrepareController(FoliageController& foliage_controller) const override;
  /**
   * @brief Generates foliage transformation matrices based on internode information.
   * @param[out] matrices Vector to store the transformation matrices.
   * @param[in] internode_info Information about the skeleton node internode.
   * @param[in] tree_size The overall tree size.
   */
  void GenerateFoliageMatrices(std::vector<glm::mat4>& matrices, const SkeletonNodeInfo& internode_info,
                               float tree_size) const override;
};
}  // namespace eco_sys_lab_plugin