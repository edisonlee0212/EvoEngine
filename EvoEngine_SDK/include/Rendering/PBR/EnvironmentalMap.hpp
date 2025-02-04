
#pragma once
#include "AssetRef.hpp"
#include "LightProbe.hpp"
#include "ReflectionProbe.hpp"
#include "RenderTexture.hpp"

namespace evo_engine {

/**
 * @class EnvironmentalMap
 * @brief A final class representing an environmental map asset.
 *
 * This class provides functionalities for constructing and managing environmental maps,
 * including sky illumination, cubemaps, and render texture-based maps.
 */
class EnvironmentalMap final : public IAsset {
  // Declare friend classes to allow access to private and protected members
  friend class Platform;
  friend class Camera;
  friend class Environment;
  friend class RenderLayer;
  friend class Resources;

 public:
  /**
   * @brief Reference to the light probe used in the environment.
   */
  AssetRef light_probe;

  /**
   * @brief Reference to the reflection probe used in the environment.
   */
  AssetRef reflection_probe;

  /**
   * @brief Builds sky illumination information based on the provided sky illumination data.
   *
   * @param sky_illumination The sky illumination data to be used for building.
   * @param resolution The resolution of the generated illumination (default is 1024).
   */
  void BuildSkyIllumination(const SkyIllumination& sky_illumination, uint32_t resolution = 1024);

  /**
   * @brief Constructs the environmental map from a specified cubemap.
   *
   * @param target_cubemap A shared pointer to the target cubemap.
   */
  void ConstructFromCubemap(const std::shared_ptr<Cubemap>& target_cubemap);

  /**
   * @brief Constructs the environmental map from a specified 2D texture.
   *
   * @param target_texture_2d A shared pointer to the target 2D texture.
   */
  void ConstructFromTexture2D(const std::shared_ptr<Texture2D>& target_texture_2d);

  /**
   * @brief Constructs the environmental map from a render texture.
   *
   * @param target_render_texture A shared pointer to the target render texture.
   */
  void ConstructFromRenderTexture(const std::shared_ptr<RenderTexture>& target_render_texture);

  /**
   * @brief Displays and handles inspection of the environmental map in the editor.
   *
   * @param editor_layer A shared pointer to the editor layer.
   * @return True if the inspection is successful; otherwise, false.
   */
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
};

}  // namespace evo_engine
