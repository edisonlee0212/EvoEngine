
#pragma once
#include "Cubemap.hpp"

namespace evo_engine {

/**
 * @class ReflectionProbe
 * @brief Represents a reflection probe used in rendering for capturing reflections and environment data.
 */
class ReflectionProbe : public IAsset {
  std::shared_ptr<Cubemap> cubemap_;  ///< The cubemap associated with the reflection probe.

  friend class RenderLayer;  ///< Grants RenderLayer access to private members of ReflectionProbe.
  friend class Camera;       ///< Grants Camera access to private members of ReflectionProbe.

  std::vector<std::vector<std::shared_ptr<ImageView>>>
      mip_map_views_;  ///< A collection of mip map views for the cubemap.

 public:
  /**
   * @brief Initializes the reflection probe with a specified resolution.
   * @param resolution The resolution of the cubemap. Default is 512.
   */
  void Initialize(uint32_t resolution = 512);

  /**
   * @brief Retrieves the cubemap associated with the reflection probe.
   * @return A shared pointer to the cubemap.
   */
  [[nodiscard]] std::shared_ptr<Cubemap> GetCubemap() const;

  /**
   * @brief Constructs the reflection probe from an existing cubemap.
   * @param target_cubemap A shared pointer to the target cubemap.
   */
  void ConstructFromCubemap(const std::shared_ptr<Cubemap>& target_cubemap);

  /**
   * @brief Handles the inspection of the reflection probe in an editor layer.
   * @param editor_layer A shared pointer to the editor layer.
   * @return True if the inspection succeeds, false otherwise.
   */
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
};

}  // namespace evo_engine
