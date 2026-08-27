
#pragma once
#include "BtfMaterial.hpp"

namespace digital_agriculture_package {
using namespace evo_engine;
/**
 * @class CBTFGroup
 * @brief Represents a collection of Compressed Bidirectional Texture Function (CBTF) assets.
 *
 * This class provides functionality for managing and manipulating a group of CBTF assets,
 * including inspection, serialization, and deserialization.
 */
class CBTFGroup : public IAsset {
 public:
  /**
   * @brief List of references to associated CBTF assets.
   */
  std::vector<AssetRef> btfs;

  /**
   * @brief Collects asset references within this CBTFGroup.
   *
   * This function adds all asset references in the group to the provided list.
   *
   * @param list The list to which asset references will be added.
   */
  void CollectAssetRef(std::vector<AssetRef> &list);

  /**
   * @brief Retrieves a randomly selected CBTF asset.
   *
   * This function returns a shared pointer to a randomly chosen BtfMaterial
   * asset from the group.
   *
   * @return A shared pointer to a randomly selected CompressedBTF asset.
   */
  std::shared_ptr<BtfMaterial> GetRandom();
};
}  // namespace digital_agriculture_package
