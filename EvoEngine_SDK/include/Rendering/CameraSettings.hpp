
#pragma once

#include <cstdint>

namespace evo_engine {

/**
 * @struct CameraSettings
 * @brief A structure to define the camera settings used in the engine.
 */
struct CameraSettings {
  enum class BackgroundSource : uint32_t {
    ClearColor,
    Cubemap,
    EnvironmentalMap,
    InheritEnvironmentalLighting,
    EngineDefaultSkybox,
  };

  enum class RayDebugView {
    Beauty,
    MaterialId,
    BaseColor,
    GeometricNormal,
    ShadingNormal,
    Roughness,
    Metallic,
    SpecularF0,
    AlphaCoverage,
    Transmission,
    Iridescence,
    Emission,
    DirectPunctual,
    DirectEnvironment,
    DirectEmissive,
    IndirectRadiance,
    PathDepth,
    BsdfPdf,
    LightPdf,
    EmissivePdf,
  };

  enum class ShaderExecutionReorderingMode {
    Disabled,
    Automatic,
    Enabled,
  };

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

  /** @brief Visible background source used by primary camera misses. */
  BackgroundSource background_source = BackgroundSource::Cubemap;

  /** @brief The clear color used when the background source is ClearColor. */
  glm::vec4 clear_color = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);

  /** @brief The background intensity applied to the camera's view. */
  float background_intensity = 1.0f;

  /**
   * \brief Ray tracing sample per pixel
   */
  int sample_size = 4;
  /**
   * \brief Ray tracing bounces
   */
  int bounce = 4;
  /**
   * \brief Ray tracing camera gamma
   */
  float gamma = 2.2f;

  /**
   * \brief Clamp high-energy ray tracing samples before accumulation to reduce fireflies.
   */
  bool firefly_clamp_enabled = true;
  /**
   * \brief Luminance threshold used when firefly_clamp_enabled is true.
   */
  float firefly_clamp_threshold = 10.0f;

  /**
   * \brief Enables emissive-triangle next-event sampling for ray cameras.
   */
  bool emissive_triangle_nee_enabled = true;

  /** @brief Selects a shared RTX/RayQuery diagnostic output. */
  RayDebugView ray_debug_view = RayDebugView::Beauty;

  /**
   * \brief Enables per-pixel adaptive ray tracing accumulation.
   */
  bool auto_spp_enabled = false;
  /**
   * \brief Minimum accumulated samples per pixel before convergence can stop a pixel.
   */
  int auto_spp_min_samples = 16;
  /**
   * \brief Maximum accumulated samples per pixel for adaptive accumulation.
   */
  int auto_spp_max_samples = 256;
  /**
   * \brief Relative luminance delta threshold used to mark an adaptive pixel converged.
   */
  float auto_spp_convergence_threshold = 0.01f;

  ShaderExecutionReorderingMode shader_execution_reordering_mode = ShaderExecutionReorderingMode::Automatic;
};

}  // namespace evo_engine
