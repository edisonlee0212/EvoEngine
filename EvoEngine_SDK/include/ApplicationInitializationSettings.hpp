#pragma once
namespace evo_engine {

/**
 * @brief Utility class for graphics settings.
 */
class GraphicsInitializationSettings {
 public:
  /// Flag to indicate the use of mesh shaders.
  bool use_mesh_shader = true;

  /// Flag to indicate the use of ray tracing.
  bool use_ray_tracing = true;

  /// Resolution for directional light shadow maps.
  uint32_t directional_light_shadow_map_resolution = 2048;

  /// Resolution for point light shadow maps.
  uint32_t point_light_shadow_map_resolution = 2048;

  /// Resolution for spot light shadow maps.
  uint32_t spot_light_shadow_map_resolution = 2048;

  /// Maximum 2D texture resource size.
  uint32_t max_texture_2d_resource_size = 2048;

  /// Maximum cubemap texture resource size.
  uint32_t max_cubemap_resource_size = 256;

  /// Maximum number of directional lights supported.
  uint32_t max_directional_light_size = 4;

  /// Maximum number of point lights supported.
  uint32_t max_point_light_size = 16;

  /// Maximum number of spotlights supported.
  uint32_t max_spot_light_size = 16;
};

/**
 * @brief Structure for storing information about the application.
 */
struct ApplicationInitializationSettings {
  std::filesystem::path project_path;            /**< The path to the application's project. */
  std::string application_name = "Evo Engine";   /**< The name of the application. */
  std::vector<std::filesystem::path> icon_paths; /**< Paths to application icons. */
  glm::ivec2 default_window_size = {1280, 720};  /**< The default size of the application window. */
  bool enable_docking = true;                    /**< Whether to enable docking in the application. */
  bool enable_viewport = true;                   /**< Whether to enable the viewport feature. */
  bool full_screen = false;                      /**< Whether the application starts in full-screen mode. */

  GraphicsInitializationSettings graphics_settings{};
};
}  // namespace evo_engine