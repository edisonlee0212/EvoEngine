
#pragma once
using namespace evo_engine;

namespace eco_sys_lab_plugin {

/**
 * @brief Represents a tree descriptor asset in the EcoSysLab plugin.
 *
 * This class contains references to various assets required for tree generation,
 * including shoot, foliage, fruit, flower, and bark descriptors. It provides
 * serialization, deserialization, and instantiation functionalities.
 */
class TreeDescriptor : public IAsset {
 public:
  /**
   * @brief Reference to the shoot descriptor asset.
   */
  AssetRef shoot_descriptor;

  /**
   * @brief Reference to the foliage descriptor asset.
   */
  AssetRef foliage_descriptor;

  /**
   * @brief Reference to the fruit descriptor asset.
   */
  AssetRef fruit_descriptor;

  /**
   * @brief Reference to the flower descriptor asset.
   */
  AssetRef flower_descriptor;

  /**
   * @brief Reference to the bark descriptor asset.
   */
  AssetRef bark_descriptor;

  /**
   * @brief Called when the asset is created.
   */
  void OnCreate() override;

  /**
   * @brief Inspects the asset in the editor.
   *
   * This function will be called by the editor layer to inspect the asset's properties.
   *
   * @param editor_layer The shared pointer to the editor layer.
   * @return Returns true if the asset's content is not modified during inspection.
   */
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;

  /**
   * @brief Collects all asset references contained within this asset.
   *
   * This function populates the provided list with asset references used in this descriptor.
   *
   * @param list A vector to store collected asset references.
   */
  void CollectAssetRef(std::vector<AssetRef>& list) override;

  /**
   * @brief Instantiates the tree entity from this descriptor.
   *
   * @return The instantiated tree entity.
   */
  Entity Instantiate() const;

  /**
   * @brief Serializes the tree descriptor to a YAML emitter.
   *
   * @param out The YAML emitter to write serialized data.
   */
  void Serialize(YAML::Emitter& out) const override;

  /**
   * @brief Generates a thumbnail texture for the asset.
   *
   * @return A shared pointer to a Texture2D representing the thumbnail.
   */
  [[nodiscard]] std::shared_ptr<Texture2D> GenerateThumbnailTexture() override;

  /**
   * @brief Deserializes the tree descriptor from a YAML node.
   *
   * @param in The YAML node containing serialized data.
   */
  void Deserialize(const YAML::Node& in) override;
};

}  // namespace eco_sys_lab_plugin
