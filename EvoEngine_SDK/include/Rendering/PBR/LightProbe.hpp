
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
class LightProbe final : public IAsset {
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

  /**
   * @brief Inspects the light probe in the editor layer.
   *
   * This function allows for interaction with the light probe in the editor.
   *
   * @param editor_layer A shared pointer to the editor layer.
   * @return True if inspection was successful, false otherwise.
   */
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
};
}  // namespace evo_engine
