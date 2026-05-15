
#pragma once
#ifdef CUDA_MODULE_SERVICE
#  include "BtfMaterial.hpp"
#endif

namespace digital_agriculture_package {
using namespace evo_engine;
/**
 * @class CBTFGroup
 * @brief Represents a collection of Compressed Bidirectional Texture Function (CBTF) assets.
 *
 * This class provides functionality for managing and manipulating a group of CBTF assets,
 * including inspection, serialization, and deserialization. It also supports CUDA-specific
 * operations when compiled with the CUDA module.
 */
class CBTFGroup : public IAsset {
 public:
  /**
   * @brief List of references to associated CBTF assets.
   */
  std::vector<AssetRef> btfs;

  /**
   * @brief Handles the inspection of the asset in the editor.
   *
   * This function is called by the editor layer and allows the user to inspect
   * the properties of the CBTFGroup. It returns `true` if the asset's content
   * remains unmodified during the inspection.
   *
   * @param editor_layer Shared pointer to the editor layer.
   * @return True if the asset's content is not modified, false otherwise.
   */
  bool OnInspect(const std::shared_ptr<EditorLayer> &editor_layer) override;

  /**
   * @brief Collects asset references within this CBTFGroup.
   *
   * This function adds all asset references in the group to the provided list.
   *
   * @param list The list to which asset references will be added.
   */
  void CollectAssetRef(std::vector<AssetRef> &list) override;

  /**
   * @brief Serializes the CBTFGroup asset to YAML format.
   *
   * This function outputs the necessary data to a YAML emitter, allowing the
   * asset to be saved in a structured format.
   *
   * @param out The YAML emitter to which data is serialized.
   */
  void Serialize(YAML::Emitter &out) const override;

  /**
   * @brief Deserializes the CBTFGroup asset from YAML data.
   *
   * This function reads from a YAML node and initializes the asset's members
   * with the corresponding data.
   *
   * @param in The YAML node containing serialized asset data.
   */
  void Deserialize(const YAML::Node &in) override;

#ifdef CUDA_MODULE_SERVICE
  /**
   * @brief Retrieves a randomly selected CBTF asset.
   *
   * This function returns a shared pointer to a randomly chosen BtfMaterial
   * asset from the group. Available only when compiled with CUDA support.
   *
   * @return A shared pointer to a randomly selected CompressedBTF asset.
   */
  std::shared_ptr<BtfMaterial> GetRandom();
#endif
};
}  // namespace digital_agriculture_package
