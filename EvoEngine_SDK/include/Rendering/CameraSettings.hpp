
#pragma once

namespace evo_engine {

/**
 * @struct CameraSettings
 * @brief A structure to define the camera settings used in the engine.
 */
struct CameraSettings {
  /** @brief The near clipping distance for the camera. */
  float near_distance = 0.1f;

  /** @brief The far clipping distance for the camera. */
  float far_distance = 200.0f;

  /** @brief The ratio of fade effect applied to the camera view. */
  float fade_ratio = 0.8f;

  /** @brief The factor determining the intensity of the fade effect. */
  float fade_factor = 1.f;

  /** @brief The field of view (FOV) of the camera in degrees. */
  float fov = 120;

  /** @brief Flag to determine if the camera uses a clear color or not. */
  bool use_clear_color = false;

  /** @brief The clear color used when rendering if use_clear_color is true. */
  glm::vec3 clear_color = glm::vec3(0.0f);

  /** @brief The background intensity applied to the camera's view. */
  float background_intensity = 1.0f;
};

}  // namespace evo_engine
