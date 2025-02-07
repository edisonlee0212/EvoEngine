
#pragma once

using namespace evo_engine;

namespace eco_sys_lab_plugin {

/**
 * @class FlowerDescriptor
 * @brief Represents a descriptor for generating flower assets.
 *
 * This class is part of the EcoSysLab plugin and extends the IAsset interface.
 * It provides functionality to generate a thumbnail texture for flower assets.
 */
class FlowerDescriptor : public IAsset {
 public:
  /**
   * @brief Generates a thumbnail texture for visualization.
   *
   * This function overrides the base class method to provide a 2D texture
   * representation of the flower asset.
   *
   * @return A shared pointer to the generated Texture2D object.
   */
  [[nodiscard]] std::shared_ptr<Texture2D> GenerateThumbnailTexture() override;
};

}  // namespace eco_sys_lab_plugin
