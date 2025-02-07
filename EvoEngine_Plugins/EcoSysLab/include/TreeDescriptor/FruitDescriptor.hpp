
#pragma once

using namespace evo_engine;

namespace eco_sys_lab_plugin {

/**
 * @class FruitDescriptor
 * @brief Represents a descriptor for a fruit asset within the EcoSysLab plugin.
 *
 * This class provides functionality to generate a thumbnail texture
 * for visual representation of the fruit asset.
 */
class FruitDescriptor : public IAsset {
 public:
  /**
   * @brief Generates a thumbnail texture for the fruit asset.
   *
   * This function overrides the base implementation to provide a
   * texture representation of the fruit.
   *
   * @return A shared pointer to the generated Texture2D object.
   */
  [[nodiscard]] std::shared_ptr<Texture2D> GenerateThumbnailTexture() override;
};

}  // namespace eco_sys_lab_plugin
