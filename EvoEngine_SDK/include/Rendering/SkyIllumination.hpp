
#pragma once
#include "Atmosphere.hpp"

namespace evo_engine {
class EditorLayer;
/**
 * @class SkyIllumination
 * @brief Represents the atmospheric and lighting properties of the sky.
 */
struct SkyIllumination {
  /**
   * @brief The atmosphere settings.
   */
  Atmosphere atmosphere{};

  /**
   * @brief The direction of the sun in the sky. Default is pointing up.
   */
  glm::vec3 sun_direction = glm::vec3(0, 1, 0);

  /**
   * @brief The gamma correction value. Default is 2.2.
   */
  float gamma = 2.2f;

  /**
   * @brief The color of the ground. Default is a shade of grey.
   */
  glm::vec3 ground_color = glm::vec3(0.75f);

  /**
   * @brief The transmittance of the ground layer, representing its transparency. Default is 0.5.
   */
  float ground_transmittance = 0.5f;

  /**
   * @brief A function to inspect and modify the SkyIllumination properties.
   *
   * @param editor_layer A shared pointer to the EditorLayer responsible for managing editing operations.
   * @return true if the properties were successfully updated, false otherwise.
   */
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);
};

}  // namespace evo_engine
