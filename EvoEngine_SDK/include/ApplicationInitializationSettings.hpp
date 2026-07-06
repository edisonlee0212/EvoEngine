#pragma once
namespace evo_engine {

enum class ApplicationMode {
  Editor,  /**< Full editor/tooling mode. */
  Player,  /**< Runtime player mode without editor UI. */
  Headless /**< Non-windowed runtime mode reserved for tools and automation. */
};

/**
 * @brief Utility class for graphics settings.
 */
class GraphicsInitializationSettings {
 public:
  /**
   * @brief Named quality levels for shadow map resolution.
   */
  enum class ShadowMapResolutionQuality {
    Low,      /**< 1024x1024 shadow maps. */
    Medium,   /**< 2048x2048 shadow maps. */
    High,     /**< 4096x4096 shadow maps. */
    VeryHigh, /**< 8192x8192 shadow maps. */
  };

  /**
   * @brief Returns the square shadow map resolution for a quality level.
   */
  [[nodiscard]] static constexpr uint32_t ShadowMapResolutionFromQuality(const ShadowMapResolutionQuality quality) {
    switch (quality) {
      case ShadowMapResolutionQuality::Low:
        return 1024;
      case ShadowMapResolutionQuality::Medium:
        return 2048;
      case ShadowMapResolutionQuality::High:
        return 4096;
      case ShadowMapResolutionQuality::VeryHigh:
        return 8192;
    }
    return 4096;
  }

  /**
   * @brief Returns a display name for a shadow map resolution quality level.
   */
  [[nodiscard]] static constexpr const char* ShadowMapResolutionQualityName(const ShadowMapResolutionQuality quality) {
    switch (quality) {
      case ShadowMapResolutionQuality::Low:
        return "Low";
      case ShadowMapResolutionQuality::Medium:
        return "Medium";
      case ShadowMapResolutionQuality::High:
        return "High";
      case ShadowMapResolutionQuality::VeryHigh:
        return "Very High";
    }
    return "High";
  }

  /// Flag to indicate the use of mesh shaders.
  bool use_mesh_shader = true;

  /// Flag to indicate the use of ray tracing.
  bool use_ray_tracing = true;

  /// Quality level used to initialize shadow map resolution fields.
  ShadowMapResolutionQuality shadow_map_resolution_quality = ShadowMapResolutionQuality::High;

  /// Resolution for directional light shadow maps.
  uint32_t directional_light_shadow_map_resolution = ShadowMapResolutionFromQuality(ShadowMapResolutionQuality::High);

  /// Resolution for point light shadow maps.
  uint32_t point_light_shadow_map_resolution = ShadowMapResolutionFromQuality(ShadowMapResolutionQuality::High);

  /// Resolution for spot light shadow maps.
  uint32_t spot_light_shadow_map_resolution = ShadowMapResolutionFromQuality(ShadowMapResolutionQuality::High);

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

  /**
   * @brief Applies a shadow map resolution quality to all shadow-map resource settings.
   */
  void SetShadowMapResolutionQuality(const ShadowMapResolutionQuality quality) {
    shadow_map_resolution_quality = quality;
    const auto resolution = ShadowMapResolutionFromQuality(quality);
    directional_light_shadow_map_resolution = resolution;
    point_light_shadow_map_resolution = resolution;
    spot_light_shadow_map_resolution = resolution;
  }
};

/**
 * @brief Structure for storing information about the application.
 */
struct ApplicationInitializationSettings {
  ApplicationMode application_mode = ApplicationMode::Editor; /**< The runtime mode for layer setup and startup. */
  std::filesystem::path project_path;                         /**< The path to the application's project. */
  std::string application_name = "Evo Engine";                /**< The name of the application. */
  std::vector<std::filesystem::path> icon_paths;              /**< Paths to application icons. */
  glm::ivec2 default_window_size = {1920, 1080};              /**< The default size of the application window. */
  bool allow_empty_project = false;  /**< Whether initialization may proceed without a project path. */
  bool enable_docking = true;        /**< Whether to enable docking in the application. */
  bool enable_viewport = true;       /**< Whether to enable the viewport feature. */
  bool full_screen = false;          /**< Whether the application starts in full-screen mode. */
  bool use_custom_title_bar = false; /**< Whether supported platforms should use app-rendered chrome. */
  bool window_resizable = true;      /**< Whether the application window can be resized by the user. */
  bool hide_console_window = true;   /**< Whether to hide an owned OS console window on supported platforms. */
  bool redirect_standard_streams_to_console =
      true;                             /**< Whether C++ stdout/stderr should be mirrored to Evo's console. */
  bool load_default_resources = true;   /**< Whether to load built-in default rendering resources. */
  bool load_project_assets = true;      /**< Whether project open should load all discovered project assets. */
  bool load_project_start_scene = true; /**< Whether project open should load/create and attach a scene. */
  bool enable_runtime_packages = false; /**< Whether to load runtime packages during initialization. */
  std::vector<std::filesystem::path> package_search_paths; /**< Additional runtime package search paths. */
  std::vector<std::string> startup_runtime_packages;       /**< Runtime packages to load during initialization. */

  GraphicsInitializationSettings graphics_settings{};
};
}  // namespace evo_engine
