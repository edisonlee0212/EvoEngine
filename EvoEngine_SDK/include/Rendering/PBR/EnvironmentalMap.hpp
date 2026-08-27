
#pragma once
#include "AssetRef.hpp"
#include "LightProbe.hpp"
#include "RenderTexture.hpp"
#include "SkyIllumination.hpp"

#include <vector>

namespace evo_engine {

/**
 * @class EnvironmentalMap
 * @brief A final class representing an environmental map asset.
 *
 * This class provides functionalities for constructing and managing environmental maps,
 * including sky illumination, cubemaps, and render texture-based maps.
 */
class EVOENGINE_API EnvironmentalMap final : public IAsset {
  // Declare friend classes to allow access to private and protected members
  friend class Platform;
  friend class Camera;
  friend class Environment;
  friend class RenderLayer;
  friend class Resources;

 public:
  enum class SourceType : uint32_t {
    None,
    Texture2D,
    Cubemap,
    SkyIllumination,
  };

  [[nodiscard]] bool SupportsStagedLoading() const {
    return true;
  }

  /**
   * @brief Reference to the light probe used in the environment.
   */
  AssetRef light_probe;

  /**
   * @brief Reference to the HDR/environment-map sampling CDF used by ray tracing.
   */
  AssetRef environment_pdf_texture;

  /**
   * @brief Reference to the unfiltered cubemap evaluated by ray-traced environment lighting.
   */
  AssetRef environment_cubemap;

  /** @brief Rebuildable source used to restore generated runtime textures after asset reload. */
  AssetRef environment_source;

  SourceType environment_source_type = SourceType::None;
  bool environment_source_pdf_expected = false;
  SkyIllumination sky_illumination_source{};
  uint32_t sky_illumination_resolution = 1024;

  [[nodiscard]] static std::vector<glm::vec4> BuildEnvironmentPdfData(const std::vector<glm::vec4>& pixels,
                                                                      const glm::uvec2& resolution);

  void CollectAssetRef(std::vector<AssetRef>& list);
  void EnsureEnvironmentSource();

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
};

}  // namespace evo_engine
