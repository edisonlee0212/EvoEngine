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
  glm::ivec2 default_window_size = {1920, 1080}; /**< The default size of the application window. */
  bool allow_empty_project = false;              /**< Whether initialization may proceed without a project path. */
  bool enable_docking = true;                    /**< Whether to enable docking in the application. */
  bool enable_viewport = true;                   /**< Whether to enable the viewport feature. */
  bool full_screen = false;                      /**< Whether the application starts in full-screen mode. */
  bool use_custom_title_bar = false;             /**< Whether supported platforms should use app-rendered chrome. */
  bool load_default_resources = true;            /**< Whether to load built-in default rendering resources. */
  bool load_project_assets = true;               /**< Whether project open should load all discovered project assets. */
  bool load_project_start_scene = true;          /**< Whether project open should load/create and attach a scene. */
  bool enable_runtime_packages = false;          /**< Whether to load runtime packages during initialization. */
  std::vector<std::filesystem::path> package_search_paths; /**< Additional runtime package search paths. */
  std::vector<std::string> startup_runtime_packages;       /**< Runtime packages to load during initialization. */

  GraphicsInitializationSettings graphics_settings{};
};
}  // namespace evo_engine
