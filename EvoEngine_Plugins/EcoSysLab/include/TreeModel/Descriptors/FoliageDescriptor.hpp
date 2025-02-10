#pragma once
#include "Skeleton.hpp"

using namespace evo_engine;

namespace eco_sys_lab_plugin {

/**
 * @class FoliageDescriptor
 * @brief Represents a foliage descriptor for procedural tree generation.
 *
 * This class defines parameters for generating foliage on a procedural tree model.
 * It includes attributes for leaf size, count, branching angles, and material references.
 */
class FoliageDescriptor : public IAsset {
 public:
  /// Leaf size represented as width and height.
  glm::vec2 leaf_size = glm::vec2(0.04f, 0.08f);

  /// Number of leaves per internode.
  int leaf_count_per_internode = 5;

  /// Variance in leaf positioning.
  float position_variance = 0.175f;

  /// Variance in leaf rotation.
  float rotation_variance = 10.f;

  /// Default branching angle in degrees.
  float branching_angle = 30.f;

  /// Maximum thickness at a node.
  float max_node_thickness = 1.0f;

  /// Minimum distance for the root.
  float min_root_distance = 0.0f;

  /// Maximum distance to the end node.
  float max_end_distance = 0.2f;

  /// Horizontal tropism effect.
  float horizontal_tropism = 0.f;

  /// Gravitropism effect.
  float gravitropism = 0.f;

  /// Reference to the leaf material asset.
  AssetRef leaf_material_ref;

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
   * @brief Generates a thumbnail texture for this foliage descriptor.
   * @return A shared pointer to the generated texture.
   */
  [[nodiscard]] std::shared_ptr<Texture2D> GenerateThumbnailTexture() override;

  /**
   * @brief Generates foliage transformation matrices based on internode information.
   * @param[out] matrices Vector to store the transformation matrices.
   * @param[in] internode_info Information about the skeleton node internode.
   * @param[in] tree_size The overall tree size.
   */
  void GenerateFoliageMatrices(std::vector<glm::mat4>& matrices, const SkeletonNodeInfo& internode_info,
                               float tree_size) const;
};

}  // namespace eco_sys_lab_plugin
