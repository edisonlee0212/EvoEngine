
#pragma once
#include "Cubemap.hpp"

namespace evo_engine {

/**
 * @class LightProbe
 * @brief A class representing a light probe asset.
 *
 * This class extends the IAsset interface and is used to manage light probes,
 * which include functionality for initializing, constructing from a cubemap,
 * and interacting with the editor layer.
 */
class EVOENGINE_API LightProbe final : public IAsset {
  /**
   * @brief The cubemap associated with this light probe.
   */
  std::shared_ptr<Cubemap> cubemap_;
  std::shared_ptr<GraphicsPipeline> irradiance_construct_pipeline_;

  /**
   * @brief Grants RenderLayer access to private members of LightProbe.
   */
  friend class RenderLayer;

  /**
   * @brief Grants Camera access to private members of LightProbe.
   */
  friend class Camera;

 public:
  [[nodiscard]] bool SupportsStagedLoading() const {
    return true;
  }

  /**
   * @brief Initializes the light probe with a specified resolution.
   *
   * @param resolution The resolution of the light probe. Default is 32.
   */
  void Initialize(uint32_t resolution = 32);

  /**
   * @brief Constructs the light probe from an existing cubemap.
   *
   * @param target_cubemap A shared pointer to the target cubemap.
   */
  void ConstructFromCubemap(const std::shared_ptr<Cubemap>& target_cubemap);

  /**
   * @brief Gets the cubemap associated with this light probe.
   *
   * @return A shared pointer to the cubemap.
   */
  [[nodiscard]] std::shared_ptr<Cubemap> GetCubemap() const;
};
}  // namespace evo_engine
