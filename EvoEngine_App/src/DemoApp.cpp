#include "AppBootstrap.hpp"
#include "Application.hpp"
#include "Camera.hpp"
#include "DemoProfiles.hpp"
#include "DemoScene.hpp"
#include "EditorLayer.hpp"
#include "EnvironmentalLighting.hpp"
#include "EnvironmentalLightingResolver.hpp"
#include "ImGuiLayer.hpp"
#include "Lights.hpp"
#include "Material.hpp"
#include "MeshRenderer.hpp"
#include "Platform.hpp"
#include "ProjectManager.hpp"
#include "RenderLayer.hpp"
#include "RenderTexture.hpp"
#include "WindowLayer.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <optional>
#include <utility>

#ifdef PHYSX_PHYSICS_SERVICE
#  include "PhysicsLayer.hpp"
#endif

using namespace evo_engine;

namespace {
enum class DemoAppRunMode { Normal, SmokeTest, EditorScreenshot };

template <typename Function>
class ScopeExit {
 public:
  explicit ScopeExit(Function function) : function_(std::move(function)) {
  }

  ScopeExit(const ScopeExit&) = delete;
  ScopeExit& operator=(const ScopeExit&) = delete;

  ~ScopeExit() {
    function_();
  }

  void Run() {
    function_();
  }

 private:
  Function function_;
};

template <typename Function>
ScopeExit<Function> MakeScopeExit(Function function) {
  return ScopeExit<Function>(std::move(function));
}

struct DemoAppRuntimeConfig {
  DemoAppRunMode mode = DemoAppRunMode::Normal;
  ApplicationMode application_mode = ApplicationMode::Editor;
  DemoSetup demo_setup = DemoSetup::Rendering;
  size_t frames_after_play = 100;
  size_t warmup_frames = 30;
  size_t max_load_frames = 30000;
  size_t max_play_frames = 1000;
  int screenshot_width = 1920;
  int screenshot_height = 1080;
  bool exit_on_complete = true;
  bool inspect_render_layer = false;
  bool ddgi_atlas_preview = false;
  bool ddgi_ray_overlay = false;
  std::optional<GraphicsInitializationSettings::ShadowMapResolutionQuality> shadow_map_resolution_quality;
  std::filesystem::path ready_file;
  std::filesystem::path done_file;
  std::filesystem::path screenshot_file;
};

EnvironmentalLighting::DdgiVolume* FindEnvironmentalLightingDdgiVolume(const std::shared_ptr<Scene>& scene,
                                                                       const std::string& name);

struct DemoAppCommandLine {
  std::optional<std::filesystem::path> run_config_path;
  std::optional<ApplicationMode> application_mode;
  std::optional<GraphicsInitializationSettings::ShadowMapResolutionQuality> shadow_map_resolution_quality;
};

DemoAppCommandLine ParseCommandLine(const int argc, char** argv) {
  DemoAppCommandLine command_line;
  for (int arg_index = 1; arg_index < argc; ++arg_index) {
    const std::string argument = argv[arg_index] ? argv[arg_index] : "";
    if (argument == "--run-config") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument("--run-config requires a path.");
      }
      command_line.run_config_path = std::filesystem::absolute(argv[++arg_index]);
    } else if (argument == "--shadow-map-resolution" || argument == "--shadow-resolution") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument(argument + " requires low, medium, high, or very-high.");
      }
      command_line.shadow_map_resolution_quality =
          ParseShadowMapResolutionQualityName(argv[++arg_index] ? argv[arg_index] : "");
    } else {
      auto mode = command_line.application_mode.value_or(ApplicationMode::Editor);
      if (!ConsumeApplicationModeArgument(argc, argv, arg_index, mode)) {
        throw std::invalid_argument("Unknown DemoApp argument: " + argument);
      }
      command_line.application_mode = mode;
    }
  }
  return command_line;
}

std::optional<std::filesystem::path> FindRunConfigPath(const DemoAppCommandLine& command_line, const int argc,
                                                       char** argv) {
  if (command_line.run_config_path) {
    return command_line.run_config_path;
  }
  std::vector<std::filesystem::path> candidates;
  if (argc > 0 && argv[0]) {
    candidates.emplace_back(std::filesystem::absolute(argv[0]).parent_path() / "DemoApp.run.yaml");
  }
  candidates.emplace_back(std::filesystem::current_path() / "DemoApp.run.yaml");
  for (const auto& candidate : candidates) {
    if (std::filesystem::exists(candidate)) {
      return candidate;
    }
  }
  return {};
}

DemoSetup ParseDemoSetup(const std::string& value) {
  if (value == "Empty") {
    return DemoSetup::Empty;
  }
  if (value == "Rendering") {
    return DemoSetup::Rendering;
  }
  if (value == "CornellBox" || value == "Cornell") {
    return DemoSetup::CornellBox;
  }
  if (value == "ThinWall" || value == "ThinWallDdgi") {
    return DemoSetup::ThinWall;
  }
  if (value == "Universe") {
    return DemoSetup::Universe;
  }
  if (value == "ProceduralGalaxy") {
    return DemoSetup::ProceduralGalaxy;
  }
  if (value == "GaussianSplat" || value == "3DGS") {
    return DemoSetup::GaussianSplat;
  }
  if (value == "Bicycle") {
    return DemoSetup::Bicycle;
  }
  throw std::invalid_argument("Unknown demo_setup value: " + value);
}

DemoAppRuntimeConfig LoadRunConfig(const std::filesystem::path& path) {
  const YAML::Node root = YAML::LoadFile(path.string());
  if (!root || !root.IsMap()) {
    throw std::runtime_error("DemoApp run config must be a YAML map.");
  }

  DemoAppRuntimeConfig config;
  if (const auto mode = root["mode"]) {
    const auto mode_name = mode.as<std::string>();
    if (mode_name == "normal") {
      config.mode = DemoAppRunMode::Normal;
    } else if (mode_name == "smoke_test") {
      config.mode = DemoAppRunMode::SmokeTest;
    } else if (mode_name == "editor_screenshot") {
      config.mode = DemoAppRunMode::EditorScreenshot;
    } else {
      throw std::invalid_argument("Unknown mode value: " + mode_name);
    }
  }
  if (const auto application_mode = root["application_mode"]) {
    config.application_mode = ParseApplicationModeName(application_mode.as<std::string>());
  }
  if (const auto shadow_map_resolution = root["shadow_map_resolution"]) {
    config.shadow_map_resolution_quality = ParseShadowMapResolutionQualityName(shadow_map_resolution.as<std::string>());
  }
  if (const auto demo_setup = root["demo_setup"]) {
    config.demo_setup = ParseDemoSetup(demo_setup.as<std::string>());
  }
  if (const auto frames_after_play = root["frames_after_play"]) {
    config.frames_after_play = frames_after_play.as<size_t>();
  }
  if (const auto warmup_frames = root["warmup_frames"]) {
    config.warmup_frames = warmup_frames.as<size_t>();
  }
  if (const auto max_load_frames = root["max_load_frames"]) {
    config.max_load_frames = max_load_frames.as<size_t>();
  }
  if (const auto max_play_frames = root["max_play_frames"]) {
    config.max_play_frames = max_play_frames.as<size_t>();
  }
  if (const auto screenshot_width = root["screenshot_width"]) {
    config.screenshot_width = screenshot_width.as<int>();
  }
  if (const auto screenshot_height = root["screenshot_height"]) {
    config.screenshot_height = screenshot_height.as<int>();
  }
  if (const auto exit_on_complete = root["exit_on_complete"]) {
    config.exit_on_complete = exit_on_complete.as<bool>();
  }
  if (const auto inspect_render_layer = root["inspect_render_layer"]) {
    config.inspect_render_layer = inspect_render_layer.as<bool>();
  }
  if (const auto ddgi_atlas_preview = root["ddgi_atlas_preview"]) {
    config.ddgi_atlas_preview = ddgi_atlas_preview.as<bool>();
  }
  if (const auto ddgi_ray_overlay = root["ddgi_ray_overlay"]) {
    config.ddgi_ray_overlay = ddgi_ray_overlay.as<bool>();
  }
  if (const auto ready_file = root["ready_file"]) {
    config.ready_file = std::filesystem::absolute(ready_file.as<std::string>());
  }
  if (const auto done_file = root["done_file"]) {
    config.done_file = std::filesystem::absolute(done_file.as<std::string>());
  }
  if (const auto screenshot_file = root["screenshot_file"]) {
    config.screenshot_file = std::filesystem::absolute(screenshot_file.as<std::string>());
  }
  return config;
}

int FailSmokeTest(Application& application, const std::string& reason) {
  if (application.IsPlaying()) {
    application.Stop();
  }
  application.End();
  std::cerr << "EVOENGINE_APP_TEST_RESULT failed reason=\"" << reason << "\"" << std::endl;
  return 1;
}

std::optional<Entity> FindEntityByName(const std::shared_ptr<Scene>& scene, const std::string& name);

int ValidateMainCameraRayTracingSetup(Application& application, const DemoAppRuntimeConfig& config,
                                      const std::shared_ptr<Camera>& main_camera, const std::string& scene_label) {
  if (!main_camera || main_camera->camera_render_mode != Camera::CameraRenderMode::RayTracing) {
    return FailSmokeTest(application, scene_label + " main camera is not configured for ray tracing");
  }
  const auto main_camera_size = main_camera->GetSize();
  if (config.application_mode == ApplicationMode::Editor) {
    const auto editor_layer = application.GetLayer<EditorLayer>();
    if (!editor_layer || !editor_layer->main_camera_allow_auto_resize) {
      return FailSmokeTest(application,
                           scene_label + " main camera is not configured to auto-fit the editor camera window");
    }
    if (main_camera_size.x == 0u || main_camera_size.y == 0u) {
      return FailSmokeTest(application, scene_label + " main camera render target size is invalid");
    }
  } else if (main_camera_size.x != 1920u || main_camera_size.y != 1080u) {
    return FailSmokeTest(application, scene_label + " main camera is not 1080p");
  }
  return 0;
}

int ValidateRenderingDemoDdgiState(Application& application, const DemoAppRuntimeConfig& config) {
  if (config.demo_setup != DemoSetup::Rendering) {
    return 0;
  }
  const auto scene = application.GetActiveScene();
  if (!scene) {
    return FailSmokeTest(application, "active scene is missing for DDGI validation");
  }
  const auto render_layer = application.GetLayer<RenderLayer>();
  if (!render_layer) {
    return FailSmokeTest(application, "render layer is missing for DDGI validation");
  }
  const auto resolved_lighting = ResolveEnvironmentalLighting(scene);
  if (!resolved_lighting.environmental_lighting_asset_assigned ||
      resolved_lighting.environmental_lighting_asset_missing) {
    return FailSmokeTest(application, "Rendering demo environmental lighting asset is missing for DDGI validation");
  }
  if (resolved_lighting.environment_lighting_intensity != 1.0f ||
      resolved_lighting.diffuse_fallback_intensity != 1.0f) {
    return FailSmokeTest(application, "Rendering environmental lighting asset is not using neutral diffuse controls");
  }
  const auto main_camera = scene->main_camera.Get<Camera>();
  if (const auto camera_result = ValidateMainCameraRayTracingSetup(application, config, main_camera, "Rendering demo");
      camera_result != 0) {
    return camera_result;
  }
  const auto& ddgi_settings = resolved_lighting.ddgi_settings;
  if (!ddgi_settings.runtime.enabled || !ddgi_settings.debug.enabled ||
      !ddgi_settings.debug.visualize_probe_positions) {
    return FailSmokeTest(application, "DDGI probe visualization is not enabled");
  }
  if (ddgi_settings.runtime.ray_count != 256 || ddgi_settings.runtime.normal_bias != 0.02f ||
      ddgi_settings.debug.visualization_scale != 2.0f || ddgi_settings.storage.max_probe_count < 960) {
    return FailSmokeTest(application, "DDGI runtime defaults are not configured for the Rendering demo");
  }
  const auto source_volume = FindEnvironmentalLightingDdgiVolume(scene, "DDGI Probe Volume");
  if (!source_volume) {
    return FailSmokeTest(application, "DDGI probe volume is missing");
  }
  bool found_ddgi_volume = false;
  for (const auto& volume : resolved_lighting.ddgi_volumes) {
    if (volume.stable_id == source_volume->stable_id) {
      if (volume.probe_counts != glm::ivec3(10, 6, 16) || volume.probe_spacing != glm::vec3(1.5f) ||
          volume.volume_origin != glm::vec3(0.0f, 3.0f, 3.0f) || volume.relocation_distance != 0.25f ||
          !volume.enable_probe_relocation || volume.enable_probe_classification) {
        return FailSmokeTest(application, "DDGI probe volume defaults are not configured for the Rendering demo");
      }
      if (glm::vec3(volume.transform[3]) != glm::vec3(0.0f, 0.0f, -6.0f)) {
        return FailSmokeTest(application, "DDGI probe volume transform is not configured for the Rendering demo");
      }
      found_ddgi_volume = true;
      break;
    }
  }
  if (!found_ddgi_volume) {
    return FailSmokeTest(application, "DDGI probe volume is missing");
  }

  const auto* directional_light_owners = scene->UnsafeGetPrivateComponentOwnersList<DirectionalLight>();
  if (!directional_light_owners) {
    return FailSmokeTest(application, "Rendering demo directional light owner list is missing");
  }
  bool found_directional_light = false;
  for (const auto& owner : *directional_light_owners) {
    const auto directional_light = scene->GetOrSetPrivateComponent<DirectionalLight>(owner).lock();
    if (directional_light && directional_light->IsEnabled() && scene->IsEntityEnabled(owner) &&
        scene->GetEntityName(owner) == "Top Down Directional Light") {
      if (directional_light->diffuse_brightness != 5.0f || directional_light->diffuse != glm::vec3(1.0f) ||
          !directional_light->cast_shadow) {
        return FailSmokeTest(application, "Rendering demo directional light is not configured");
      }
      found_directional_light = true;
      break;
    }
  }
  if (!found_directional_light) {
    return FailSmokeTest(application, "Rendering demo top-down directional light is missing");
  }

  const auto* point_light_owners = scene->UnsafeGetPrivateComponentOwnersList<PointLight>();
  if (!point_light_owners) {
    return FailSmokeTest(application, "Rendering demo point light owner list is missing");
  }
  for (const auto& owner : *point_light_owners) {
    const auto point_light = scene->GetOrSetPrivateComponent<PointLight>(owner).lock();
    if (point_light && point_light->IsEnabled() && scene->IsEntityEnabled(owner) &&
        scene->GetEntityName(owner) == "Left Point Light") {
      if (point_light->diffuse_brightness != 24.0f || point_light->light_size != 0.005f ||
          point_light->constant != 2.5f || point_light->linear != 0.5f || point_light->quadratic != 0.1f ||
          point_light->diffuse != glm::vec3(1.0f, 0.8f, 0.0f) || !point_light->cast_shadow) {
        return FailSmokeTest(application, "Rendering demo point light is not configured for DDGI validation");
      }
      const auto point_light_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(owner).lock();
      if (!point_light_renderer || point_light_renderer->cast_shadow) {
        return FailSmokeTest(application, "Rendering demo point light visualizer should not cast DDGI shadows");
      }
      if (scene->GetDataComponent<Transform>(owner).GetPosition() != glm::vec3(3.0f, 0.0f, -2.5f)) {
        return FailSmokeTest(application, "Rendering demo point light transform is not configured");
      }
      return 0;
    }
  }
  return FailSmokeTest(application, "Rendering demo point light is missing for DDGI validation");
}

int ValidateCornellBoxDdgiState(Application& application, const DemoAppRuntimeConfig& config) {
  if (config.demo_setup != DemoSetup::CornellBox) {
    return 0;
  }
  const auto scene = application.GetActiveScene();
  if (!scene) {
    return FailSmokeTest(application, "active scene is missing for Cornell DDGI validation");
  }
  const auto resolved_lighting = ResolveEnvironmentalLighting(scene);
  if (!resolved_lighting.environmental_lighting_asset_assigned ||
      resolved_lighting.environmental_lighting_asset_missing) {
    return FailSmokeTest(application, "Cornell environmental lighting asset is missing for DDGI validation");
  }
  if (resolved_lighting.environment_lighting_intensity != 0.0f ||
      resolved_lighting.diffuse_fallback_intensity != 1.0f) {
    return FailSmokeTest(application, "Cornell environmental lighting intensity is not disabled");
  }
  const auto main_camera = scene->main_camera.Get<Camera>();
  if (const auto camera_result = ValidateMainCameraRayTracingSetup(application, config, main_camera, "Cornell");
      camera_result != 0) {
    return camera_result;
  }

  const auto& ddgi_settings = resolved_lighting.ddgi_settings;
  if (!ddgi_settings.runtime.enabled || !ddgi_settings.debug.enabled ||
      !ddgi_settings.debug.visualize_probe_positions) {
    return FailSmokeTest(application, "Cornell DDGI probe visualization is not enabled");
  }
  if (ddgi_settings.runtime.ray_count != 256 || ddgi_settings.runtime.normal_bias != 0.02f ||
      ddgi_settings.storage.max_probe_count < 512) {
    return FailSmokeTest(application, "Cornell DDGI runtime defaults are not configured");
  }
  const auto source_volume = FindEnvironmentalLightingDdgiVolume(scene, "DDGI Probe Volume");
  if (!source_volume) {
    return FailSmokeTest(application, "Cornell DDGI probe volume is missing");
  }
  bool found_ddgi_volume = false;
  for (const auto& volume : resolved_lighting.ddgi_volumes) {
    if (volume.stable_id == source_volume->stable_id) {
      if (volume.probe_counts != glm::ivec3(9, 9, 9) || volume.probe_spacing != glm::vec3(0.3f) ||
          volume.volume_origin != glm::vec3(0.0f) || volume.relocation_distance != 0.1f ||
          !volume.enable_probe_relocation || volume.enable_probe_classification) {
        return FailSmokeTest(application, "Cornell DDGI probe volume defaults are not configured");
      }
      if (glm::vec3(volume.transform[3]) != glm::vec3(0.0f, 0.0f, -3.0f)) {
        return FailSmokeTest(application, "Cornell DDGI probe volume transform is not configured");
      }
      found_ddgi_volume = true;
      break;
    }
  }
  if (!found_ddgi_volume) {
    return FailSmokeTest(application, "Cornell DDGI probe volume is missing");
  }

  if (const auto* directional_light_owners = scene->UnsafeGetPrivateComponentOwnersList<DirectionalLight>()) {
    for (const auto& owner : *directional_light_owners) {
      const auto directional_light = scene->GetOrSetPrivateComponent<DirectionalLight>(owner).lock();
      if (directional_light && directional_light->IsEnabled() && scene->IsEntityEnabled(owner)) {
        return FailSmokeTest(application, "Cornell default directional light is still enabled");
      }
    }
  }
  const auto ceiling_light_entity = FindEntityByName(scene, "Cornell Ceiling Light");
  if (!ceiling_light_entity) {
    return FailSmokeTest(application, "Cornell ceiling point light is missing");
  }
  const auto ceiling_light = scene->GetOrSetPrivateComponent<PointLight>(*ceiling_light_entity).lock();
  if (!ceiling_light || !ceiling_light->IsEnabled() || !scene->IsEntityEnabled(*ceiling_light_entity) ||
      !ceiling_light->cast_shadow || ceiling_light->diffuse != glm::vec3(1.0f) ||
      ceiling_light->diffuse_brightness != 45.0f || ceiling_light->light_size != 0.08f ||
      ceiling_light->constant != 1.0f || ceiling_light->linear != 0.08f || ceiling_light->quadratic != 0.02f) {
    return FailSmokeTest(application, "Cornell ceiling point light is not configured for DDGI validation");
  }
  if (scene->GetDataComponent<Transform>(*ceiling_light_entity).GetPosition() != glm::vec3(0.0f, 0.82f, -3.0f)) {
    return FailSmokeTest(application, "Cornell ceiling point light transform is not configured");
  }
  return 0;
}

int ValidateThinWallDdgiState(Application& application, const DemoAppRuntimeConfig& config) {
  if (config.demo_setup != DemoSetup::ThinWall) {
    return 0;
  }
  const auto scene = application.GetActiveScene();
  if (!scene) {
    return FailSmokeTest(application, "active scene is missing for thin-wall DDGI validation");
  }
  const auto resolved_lighting = ResolveEnvironmentalLighting(scene);
  if (!resolved_lighting.environmental_lighting_asset_assigned ||
      resolved_lighting.environmental_lighting_asset_missing) {
    return FailSmokeTest(application, "thin-wall environmental lighting asset is missing for DDGI validation");
  }
  if (resolved_lighting.environment_lighting_intensity != 0.0f ||
      resolved_lighting.diffuse_fallback_intensity != 1.0f) {
    return FailSmokeTest(application, "thin-wall environmental lighting intensity is not disabled");
  }
  const auto main_camera = scene->main_camera.Get<Camera>();
  if (const auto camera_result = ValidateMainCameraRayTracingSetup(application, config, main_camera, "thin-wall");
      camera_result != 0) {
    return camera_result;
  }

  const auto& ddgi_settings = resolved_lighting.ddgi_settings;
  if (!ddgi_settings.runtime.enabled || !ddgi_settings.debug.enabled ||
      !ddgi_settings.debug.visualize_probe_positions) {
    return FailSmokeTest(application, "thin-wall DDGI probe visualization is not enabled");
  }
  if (ddgi_settings.runtime.ray_count != 256 || ddgi_settings.runtime.normal_bias != 0.015f ||
      ddgi_settings.storage.max_probe_count < 512) {
    return FailSmokeTest(application, "thin-wall DDGI runtime defaults are not configured");
  }
  const auto source_volume = FindEnvironmentalLightingDdgiVolume(scene, "DDGI Probe Volume");
  if (!source_volume) {
    return FailSmokeTest(application, "thin-wall DDGI probe volume is missing");
  }
  bool found_ddgi_volume = false;
  for (const auto& volume : resolved_lighting.ddgi_volumes) {
    if (volume.stable_id == source_volume->stable_id) {
      if (volume.probe_counts != glm::ivec3(8, 6, 8) || volume.probe_spacing != glm::vec3(0.35f) ||
          volume.volume_origin != glm::vec3(0.0f) || volume.relocation_distance != 0.25f ||
          !volume.enable_probe_relocation || volume.enable_probe_classification) {
        return FailSmokeTest(application, "thin-wall DDGI probe volume defaults are not configured");
      }
      if (glm::vec3(volume.transform[3]) != glm::vec3(0.0f, 0.0f, -3.0f)) {
        return FailSmokeTest(application, "thin-wall DDGI probe volume transform is not configured");
      }
      found_ddgi_volume = true;
      break;
    }
  }
  if (!found_ddgi_volume) {
    return FailSmokeTest(application, "thin-wall DDGI probe volume is missing");
  }

  if (!FindEntityByName(scene, "Thin Wall Blocker")) {
    return FailSmokeTest(application, "thin-wall blocker is missing");
  }
  if (const auto* directional_light_owners = scene->UnsafeGetPrivateComponentOwnersList<DirectionalLight>()) {
    for (const auto& owner : *directional_light_owners) {
      const auto directional_light = scene->GetOrSetPrivateComponent<DirectionalLight>(owner).lock();
      if (directional_light && directional_light->IsEnabled() && scene->IsEntityEnabled(owner)) {
        return FailSmokeTest(application, "thin-wall default directional light is still enabled");
      }
    }
  }
  const auto left_light_entity = FindEntityByName(scene, "Thin Wall Left Light");
  if (!left_light_entity) {
    return FailSmokeTest(application, "thin-wall point light is missing");
  }
  const auto point_light = scene->GetOrSetPrivateComponent<PointLight>(*left_light_entity).lock();
  if (!point_light || !point_light->IsEnabled() || !scene->IsEntityEnabled(*left_light_entity) ||
      !point_light->cast_shadow || point_light->diffuse != glm::vec3(1.0f, 0.82f, 0.25f) ||
      point_light->diffuse_brightness != 80.0f || point_light->light_size != 0.04f || point_light->constant != 1.0f ||
      point_light->linear != 0.08f || point_light->quadratic != 0.02f) {
    return FailSmokeTest(application, "thin-wall point light is not configured for DDGI validation");
  }
  if (scene->GetDataComponent<Transform>(*left_light_entity).GetPosition() != glm::vec3(-0.72f, 0.72f, -3.0f)) {
    return FailSmokeTest(application, "thin-wall point light transform is not configured");
  }
  return 0;
}

std::optional<Entity> FindRenderingDemoPointLightEntity(const std::shared_ptr<Scene>& scene) {
  const auto* point_light_owners = scene->UnsafeGetPrivateComponentOwnersList<PointLight>();
  if (!point_light_owners) {
    return {};
  }
  for (const auto& owner : *point_light_owners) {
    const auto point_light = scene->GetOrSetPrivateComponent<PointLight>(owner).lock();
    if (point_light && scene->IsEntityEnabled(owner) && scene->GetEntityName(owner) == "Left Point Light") {
      return owner;
    }
  }
  return {};
}

EnvironmentalLighting::DdgiVolume* FindEnvironmentalLightingDdgiVolume(const std::shared_ptr<Scene>& scene,
                                                                       const std::string& name) {
  if (!scene) {
    return nullptr;
  }
  const auto lighting = scene->environmental_lighting.Get<EnvironmentalLighting>();
  if (!lighting) {
    return nullptr;
  }
  for (auto& volume : lighting->ddgi_volumes) {
    if (volume.enabled && volume.name == name) {
      return &volume;
    }
  }
  return nullptr;
}

template <typename LightComponent>
void AppendLightComponentStates(
    const std::shared_ptr<Scene>& scene,
    std::vector<std::pair<std::shared_ptr<IPrivateComponent>, bool>>& light_component_states) {
  const auto* light_owners = scene->UnsafeGetPrivateComponentOwnersList<LightComponent>();
  if (!light_owners) {
    return;
  }
  for (const auto& owner : *light_owners) {
    const auto light = scene->GetOrSetPrivateComponent<LightComponent>(owner).lock();
    if (light) {
      light_component_states.emplace_back(light, light->IsEnabled());
    }
  }
}

struct DdgiProbeDebugReadbackSummary {
  uint32_t active_probe_count = 0;
  uint32_t inactive_probe_count = 0;
  uint32_t relocated_active_probe_count = 0;
  float average_active_irradiance = 0.0f;
  float max_active_irradiance = 0.0f;
  bool metadata_is_finite = true;
  bool has_hit_probe = false;
  bool has_miss_probe = false;
  bool has_backface_probe = false;
};

bool IsFiniteDdgiMetadataSample(const glm::vec4& sample) {
  return std::isfinite(sample.x) && std::isfinite(sample.y) && std::isfinite(sample.z) && std::isfinite(sample.w);
}

glm::ivec3 WrapDdgiProbeGrid(const glm::ivec3& probe_grid, const glm::ivec3& probe_counts) {
  const auto safe_counts = glm::max(probe_counts, glm::ivec3(1));
  return (probe_grid % safe_counts + safe_counts) % safe_counts;
}

uint32_t GetDdgiProbeIndexFromGrid(const glm::ivec3& probe_grid, const glm::ivec3& probe_counts) {
  const auto safe_counts = glm::max(probe_counts, glm::ivec3(1));
  const auto wrapped_grid = WrapDdgiProbeGrid(probe_grid, safe_counts);
  return static_cast<uint32_t>(wrapped_grid.x + wrapped_grid.y * safe_counts.x +
                               wrapped_grid.z * safe_counts.x * safe_counts.y);
}

uint32_t GetScrolledDdgiProbeIndex(const glm::ivec3& logical_probe_grid, const glm::ivec3& probe_scroll_offset,
                                   const glm::ivec3& probe_counts) {
  return GetDdgiProbeIndexFromGrid(logical_probe_grid + probe_scroll_offset, probe_counts);
}

struct DdgiProbeMetadataBlock {
  glm::vec4 irradiance = glm::vec4(0.0f);
  glm::vec4 visibility = glm::vec4(0.0f);
  glm::vec4 state = glm::vec4(0.0f);
};

bool ReadDdgiProbeMetadataBlock(const RenderLayer::DdgiProbeDebugDataView& debug_data,
                                const uint32_t physical_probe_index, DdgiProbeMetadataBlock& metadata_block) {
  if (!debug_data.metadata || physical_probe_index >= debug_data.probe_count) {
    return false;
  }
  const auto metadata_offset = static_cast<size_t>(physical_probe_index) * 3ull;
  if (metadata_offset + 2ull >= debug_data.metadata->size()) {
    return false;
  }
  metadata_block.irradiance = (*debug_data.metadata)[metadata_offset];
  metadata_block.visibility = (*debug_data.metadata)[metadata_offset + 1ull];
  metadata_block.state = (*debug_data.metadata)[metadata_offset + 2ull];
  return true;
}

float MaxDdgiMetadataDelta(const DdgiProbeMetadataBlock& lhs, const DdgiProbeMetadataBlock& rhs) {
  const auto irradiance_delta = glm::abs(lhs.irradiance - rhs.irradiance);
  const auto visibility_delta = glm::abs(lhs.visibility - rhs.visibility);
  const auto state_delta = glm::abs(lhs.state - rhs.state);
  const auto max_component = [](const glm::vec4& value) {
    return glm::max(glm::max(value.x, value.y), glm::max(value.z, value.w));
  };
  return glm::max(max_component(irradiance_delta),
                  glm::max(max_component(visibility_delta), max_component(state_delta)));
}

DdgiProbeDebugReadbackSummary SummarizeDdgiProbeDebugReadback(const RenderLayer::DdgiProbeDebugDataView& debug_data) {
  DdgiProbeDebugReadbackSummary summary;
  if (!debug_data.metadata) {
    return summary;
  }
  const auto available_probe_count = static_cast<uint32_t>(debug_data.metadata->size() / 3ull);
  const auto probe_count = glm::min(debug_data.probe_count, available_probe_count);
  for (uint32_t probe_index = 0; probe_index < probe_count; ++probe_index) {
    const auto metadata_offset = static_cast<size_t>(probe_index) * 3ull;
    const auto irradiance_sample = (*debug_data.metadata)[metadata_offset];
    const auto visibility_sample = (*debug_data.metadata)[metadata_offset + 1ull];
    const auto state_sample = (*debug_data.metadata)[metadata_offset + 2ull];
    summary.metadata_is_finite = summary.metadata_is_finite && IsFiniteDdgiMetadataSample(irradiance_sample) &&
                                 IsFiniteDdgiMetadataSample(visibility_sample) &&
                                 IsFiniteDdgiMetadataSample(state_sample);
    if (state_sample.w < 0.5f) {
      ++summary.inactive_probe_count;
      continue;
    }
    const auto hit_ratio = glm::clamp(irradiance_sample.w, 0.0f, 1.0f);
    const auto backface_ratio = glm::clamp(visibility_sample.y, 0.0f, 1.0f);
    const auto irradiance = glm::max(glm::vec3(irradiance_sample), glm::vec3(0.0f));
    const auto irradiance_strength = glm::max(irradiance.x, glm::max(irradiance.y, irradiance.z));
    ++summary.active_probe_count;
    summary.average_active_irradiance += irradiance_strength;
    summary.max_active_irradiance = glm::max(summary.max_active_irradiance, irradiance_strength);
    if (glm::length(glm::vec3(state_sample)) > 0.001f) {
      ++summary.relocated_active_probe_count;
    }
    summary.has_hit_probe = summary.has_hit_probe || hit_ratio > 0.001f;
    summary.has_miss_probe = summary.has_miss_probe || hit_ratio < 0.999f;
    summary.has_backface_probe = summary.has_backface_probe || backface_ratio > 0.001f;
  }
  if (summary.active_probe_count != 0u) {
    summary.average_active_irradiance /= static_cast<float>(summary.active_probe_count);
  }
  return summary;
}

struct DdgiProbeRegionSummary {
  uint32_t active_probe_count = 0;
  float average_irradiance = 0.0f;
  bool metadata_is_finite = true;
};

struct RenderTextureRegionSummary {
  uint32_t sample_count = 0;
  glm::vec3 average_color = glm::vec3(0.0f);
  float average_luminance = 0.0f;
  float luminance_second_moment = 0.0f;
  float luminance_standard_deviation = 0.0f;
  float minimum_luminance = std::numeric_limits<float>::max();
  float maximum_luminance = 0.0f;
  bool finite = true;
};

DdgiProbeRegionSummary SummarizeDdgiProbeRegion(const RenderLayer::DdgiProbeDebugDataView& debug_data,
                                                const glm::ivec3& probe_counts, const glm::ivec3& probe_begin,
                                                const glm::ivec3& probe_end) {
  DdgiProbeRegionSummary summary;
  const auto safe_begin = glm::clamp(probe_begin, glm::ivec3(0), probe_counts);
  const auto safe_end = glm::clamp(probe_end, safe_begin, probe_counts);
  for (int z = safe_begin.z; z < safe_end.z; ++z) {
    for (int y = safe_begin.y; y < safe_end.y; ++y) {
      for (int x = safe_begin.x; x < safe_end.x; ++x) {
        DdgiProbeMetadataBlock metadata_block;
        const auto probe_index = GetDdgiProbeIndexFromGrid({x, y, z}, probe_counts);
        if (!ReadDdgiProbeMetadataBlock(debug_data, probe_index, metadata_block)) {
          summary.metadata_is_finite = false;
          continue;
        }
        summary.metadata_is_finite =
            summary.metadata_is_finite && IsFiniteDdgiMetadataSample(metadata_block.irradiance) &&
            IsFiniteDdgiMetadataSample(metadata_block.visibility) && IsFiniteDdgiMetadataSample(metadata_block.state);
        if (metadata_block.state.w < 0.5f) {
          continue;
        }
        const auto irradiance = glm::max(glm::vec3(metadata_block.irradiance), glm::vec3(0.0f));
        summary.average_irradiance += glm::max(irradiance.x, glm::max(irradiance.y, irradiance.z));
        ++summary.active_probe_count;
      }
    }
  }
  if (summary.active_probe_count != 0u) {
    summary.average_irradiance /= static_cast<float>(summary.active_probe_count);
  }
  return summary;
}

RenderTextureRegionSummary SummarizeRenderTextureRegion(const std::vector<glm::vec4>& pixels,
                                                        const glm::uvec2& resolution, const glm::vec2& uv_begin,
                                                        const glm::vec2& uv_end) {
  RenderTextureRegionSummary summary;
  if (pixels.size() < static_cast<size_t>(resolution.x) * resolution.y || resolution.x == 0u || resolution.y == 0u) {
    summary.finite = false;
    return summary;
  }
  const auto safe_begin = glm::clamp(glm::min(uv_begin, uv_end), glm::vec2(0.0f), glm::vec2(1.0f));
  const auto safe_end = glm::clamp(glm::max(uv_begin, uv_end), glm::vec2(0.0f), glm::vec2(1.0f));
  const auto begin = glm::clamp(glm::uvec2(glm::floor(safe_begin * glm::vec2(resolution))), glm::uvec2(0), resolution);
  const auto end = glm::clamp(glm::uvec2(glm::ceil(safe_end * glm::vec2(resolution))), begin, resolution);
  for (uint32_t y = begin.y; y < end.y; ++y) {
    for (uint32_t x = begin.x; x < end.x; ++x) {
      const auto color = glm::max(glm::vec3(pixels[static_cast<size_t>(y) * resolution.x + x]), glm::vec3(0.0f));
      const auto luminance = glm::dot(color, glm::vec3(0.2126f, 0.7152f, 0.0722f));
      summary.finite = summary.finite && std::isfinite(color.x) && std::isfinite(color.y) && std::isfinite(color.z);
      summary.average_color += color;
      summary.average_luminance += luminance;
      summary.luminance_second_moment += luminance * luminance;
      summary.minimum_luminance = glm::min(summary.minimum_luminance, luminance);
      summary.maximum_luminance = glm::max(summary.maximum_luminance, luminance);
      ++summary.sample_count;
    }
  }
  if (summary.sample_count != 0u) {
    summary.average_color /= static_cast<float>(summary.sample_count);
    summary.average_luminance /= static_cast<float>(summary.sample_count);
    summary.luminance_second_moment /= static_cast<float>(summary.sample_count);
    summary.luminance_standard_deviation = std::sqrt(
        glm::max(summary.luminance_second_moment - summary.average_luminance * summary.average_luminance, 0.0f));
  }
  return summary;
}

std::optional<RenderTextureRegionSummary> CaptureMainCameraRegion(Application& application,
                                                                  const std::shared_ptr<Scene>& scene,
                                                                  const glm::vec2& uv_begin, const glm::vec2& uv_end,
                                                                  const char* failure_reason) {
  if (!application.Loop()) {
    FailSmokeTest(application, failure_reason);
    return {};
  }
  const auto main_camera = scene->main_camera.Get<Camera>();
  if (!main_camera || !main_camera->GetRenderTexture()) {
    FailSmokeTest(application, "main camera render texture is missing for render-texture validation");
    return {};
  }
  std::vector<glm::vec4> pixels;
  main_camera->GetRenderTexture()->GetRgbaChannelData(pixels);
  return SummarizeRenderTextureRegion(pixels, main_camera->GetSize(), uv_begin, uv_end);
}

int ValidateCornellBoxDdgiProbeReadback(Application& application, const DemoAppRuntimeConfig& config) {
  if (config.demo_setup != DemoSetup::CornellBox) {
    return 0;
  }
  const auto render_layer = application.GetLayer<RenderLayer>();
  if (!render_layer) {
    return FailSmokeTest(application, "render layer is missing for Cornell DDGI probe readback validation");
  }
  const auto scene = application.GetActiveScene();
  if (!scene) {
    return FailSmokeTest(application, "active scene is missing for Cornell DDGI probe readback validation");
  }
  const auto volume = FindEnvironmentalLightingDdgiVolume(scene, "DDGI Probe Volume");
  if (!volume) {
    return FailSmokeTest(application, "Cornell DDGI probe volume is missing for classification validation");
  }

  auto& ddgi_settings = render_layer->GetDdgiSettings();
  const auto original_reset_probe_history = ddgi_settings.runtime.reset_probe_history;
  const auto original_visualize_probe_state = ddgi_settings.debug.visualize_probe_state;
  const auto original_hysteresis = ddgi_settings.runtime.hysteresis;
  const auto original_classification_enabled = volume->enable_probe_classification;
  auto restore_ddgi_settings = MakeScopeExit([&]() {
    ddgi_settings.runtime.reset_probe_history = original_reset_probe_history;
    ddgi_settings.runtime.hysteresis = original_hysteresis;
    ddgi_settings.debug.visualize_probe_state = original_visualize_probe_state;
    volume->enable_probe_classification = original_classification_enabled;
  });
  const auto capture_summary = [&](const bool reset_history,
                                   const char* phase) -> std::optional<DdgiProbeDebugReadbackSummary> {
    ddgi_settings.runtime.reset_probe_history = reset_history;
    ddgi_settings.debug.visualize_probe_state = true;
    if (!application.Loop()) {
      FailSmokeTest(application,
                    std::string("application ended before Cornell DDGI ") + phase + " validation completed");
      return {};
    }
    return SummarizeDdgiProbeDebugReadback(render_layer->GetDdgiProbeDebugData(true));
  };

  ddgi_settings.runtime.hysteresis = 0.0f;
  const auto direct_only_summary = capture_summary(true, "initial probe readback");
  if (!direct_only_summary) {
    return 1;
  }

  if (direct_only_summary->active_probe_count == 0u) {
    return FailSmokeTest(application, "Cornell DDGI probe readback has no active probes");
  }
  if (!direct_only_summary->metadata_is_finite) {
    return FailSmokeTest(application, "Cornell DDGI probe readback metadata is not finite");
  }
  if (!direct_only_summary->has_hit_probe) {
    return FailSmokeTest(application, "Cornell DDGI probe readback has no hit-ratio probe");
  }
  if (direct_only_summary->max_active_irradiance <= 0.01f || direct_only_summary->average_active_irradiance <= 0.001f) {
    return FailSmokeTest(application, "Cornell DDGI probe readback has no positive indirect irradiance");
  }
  auto recursive_summary = *direct_only_summary;
  for (size_t frame_index = 0; frame_index < 3; ++frame_index) {
    const auto frame_summary = capture_summary(false, "recursive probe readback");
    if (!frame_summary) {
      return 1;
    }
    if (frame_summary->average_active_irradiance > recursive_summary.average_active_irradiance) {
      recursive_summary = *frame_summary;
    }
  }
  restore_ddgi_settings.Run();
  const auto minimum_bounce_delta = glm::max(direct_only_summary->average_active_irradiance * 0.001f, 0.0005f);
  if (recursive_summary.average_active_irradiance <=
      direct_only_summary->average_active_irradiance + minimum_bounce_delta) {
    return FailSmokeTest(application, "Cornell recursive DDGI did not increase probe irradiance over direct lighting");
  }
  volume->enable_probe_classification = false;
  const auto unclassified_summary = capture_summary(true, "classification-disabled probe readback");
  if (!unclassified_summary) {
    return 1;
  }
  restore_ddgi_settings.Run();
  if (!unclassified_summary->metadata_is_finite) {
    return FailSmokeTest(application, "Cornell classification-disabled DDGI metadata is not finite");
  }
  if (unclassified_summary->active_probe_count == 0u || unclassified_summary->max_active_irradiance <= 0.01f) {
    return FailSmokeTest(application, "Cornell classification-disabled baseline did not light active probes");
  }
  const auto minimum_classified_active_count =
      glm::max(1u, static_cast<uint32_t>(unclassified_summary->active_probe_count / 4u));
  if (direct_only_summary->active_probe_count < minimum_classified_active_count) {
    return FailSmokeTest(application, "Cornell classification pruned too many active probes active=" +
                                          std::to_string(direct_only_summary->active_probe_count) +
                                          " baseline=" + std::to_string(unclassified_summary->active_probe_count));
  }
  const auto minimum_classified_max_irradiance = unclassified_summary->max_active_irradiance * 0.25f;
  if (direct_only_summary->max_active_irradiance < minimum_classified_max_irradiance) {
    return FailSmokeTest(application, "Cornell classification reduced peak probe irradiance too much classified=" +
                                          std::to_string(direct_only_summary->max_active_irradiance) +
                                          " baseline=" + std::to_string(unclassified_summary->max_active_irradiance));
  }
  return ValidateCornellBoxDdgiState(application, config);
}

int ValidateCornellBoxDdgiClassificationSurfaceReadback(Application& application, const DemoAppRuntimeConfig& config) {
  if (config.demo_setup != DemoSetup::CornellBox) {
    return 0;
  }
  const auto scene = application.GetActiveScene();
  if (!scene) {
    return FailSmokeTest(application, "active scene is missing for Cornell DDGI classification surface validation");
  }
  const auto render_layer = application.GetLayer<RenderLayer>();
  if (!render_layer) {
    return FailSmokeTest(application, "render layer is missing for Cornell DDGI classification surface validation");
  }
  const auto main_camera = scene->main_camera.Get<Camera>();
  if (!main_camera || !main_camera->GetRenderTexture()) {
    return FailSmokeTest(application,
                         "main camera render texture is missing for Cornell DDGI classification validation");
  }
  const auto volume = FindEnvironmentalLightingDdgiVolume(scene, "DDGI Probe Volume");
  if (!volume) {
    return FailSmokeTest(application, "Cornell DDGI probe volume is missing for classification surface validation");
  }

  auto& ddgi_settings = render_layer->GetDdgiSettings();
  const auto original_reset_probe_history = ddgi_settings.runtime.reset_probe_history;
  const auto original_visualize_probe_state = ddgi_settings.debug.visualize_probe_state;
  const auto original_hysteresis = ddgi_settings.runtime.hysteresis;
  const auto original_classification_enabled = volume->enable_probe_classification;
  auto restore_ddgi_settings = MakeScopeExit([&]() {
    ddgi_settings.runtime.reset_probe_history = original_reset_probe_history;
    ddgi_settings.runtime.hysteresis = original_hysteresis;
    ddgi_settings.debug.visualize_probe_state = original_visualize_probe_state;
    volume->enable_probe_classification = original_classification_enabled;
  });
  const auto capture_surface_summary = [&](const bool classification_enabled,
                                           const char* phase) -> std::optional<RenderTextureRegionSummary> {
    volume->enable_probe_classification = classification_enabled;
    ddgi_settings.runtime.hysteresis = 0.0f;
    ddgi_settings.runtime.reset_probe_history = true;
    ddgi_settings.debug.visualize_probe_state = false;
    for (size_t frame_index = 0; frame_index < 3; ++frame_index) {
      if (!application.Loop()) {
        FailSmokeTest(application,
                      std::string("application ended before Cornell DDGI ") + phase + " atlas seed completed");
        return {};
      }
    }
    ddgi_settings.runtime.reset_probe_history = false;
    for (size_t frame_index = 0; frame_index < 2; ++frame_index) {
      if (!application.Loop()) {
        FailSmokeTest(application,
                      std::string("application ended before Cornell DDGI ") + phase + " surface validation completed");
        return {};
      }
    }

    std::vector<glm::vec4> pixels;
    main_camera->GetRenderTexture()->GetRgbaChannelData(pixels);
    return SummarizeRenderTextureRegion(pixels, main_camera->GetSize(), {0.43f, 0.36f}, {0.57f, 0.63f});
  };

  const auto unclassified_surface = capture_surface_summary(false, "classification-disabled");
  if (!unclassified_surface) {
    return 1;
  }
  const auto classified_surface = capture_surface_summary(true, "classification-enabled");
  restore_ddgi_settings.Run();
  if (!classified_surface) {
    return 1;
  }
  if (unclassified_surface->sample_count == 0u || classified_surface->sample_count == 0u ||
      !unclassified_surface->finite || !classified_surface->finite) {
    return FailSmokeTest(application, "Cornell DDGI classification surface validation read invalid main-camera pixels");
  }
  if (unclassified_surface->average_luminance <= 0.01f) {
    return FailSmokeTest(application, "Cornell DDGI classification-disabled surface baseline is too dark baseline=" +
                                          std::to_string(unclassified_surface->average_luminance) +
                                          " classified=" + std::to_string(classified_surface->average_luminance));
  }
  const auto minimum_classified_luminance = glm::max(unclassified_surface->average_luminance * 0.4f, 0.01f);
  if (classified_surface->average_luminance < minimum_classified_luminance) {
    return FailSmokeTest(application, "Cornell classification created a dark final-surface region classified=" +
                                          std::to_string(classified_surface->average_luminance) +
                                          " baseline=" + std::to_string(unclassified_surface->average_luminance) +
                                          " minimum=" + std::to_string(minimum_classified_luminance));
  }
  return ValidateCornellBoxDdgiState(application, config);
}

int ValidateThinWallDdgiProbeLeakReadback(Application& application, const DemoAppRuntimeConfig& config) {
  if (config.demo_setup != DemoSetup::ThinWall) {
    return 0;
  }
  const auto render_layer = application.GetLayer<RenderLayer>();
  if (!render_layer) {
    return FailSmokeTest(application, "render layer is missing for thin-wall DDGI leak validation");
  }

  auto& ddgi_settings = render_layer->GetDdgiSettings();
  const auto original_reset_probe_history = ddgi_settings.runtime.reset_probe_history;
  const auto original_visualize_probe_state = ddgi_settings.debug.visualize_probe_state;
  const auto original_hysteresis = ddgi_settings.runtime.hysteresis;
  auto restore_ddgi_settings = MakeScopeExit([&]() {
    ddgi_settings.runtime.reset_probe_history = original_reset_probe_history;
    ddgi_settings.runtime.hysteresis = original_hysteresis;
    ddgi_settings.debug.visualize_probe_state = original_visualize_probe_state;
  });

  ddgi_settings.runtime.hysteresis = 0.0f;
  ddgi_settings.runtime.reset_probe_history = true;
  ddgi_settings.debug.visualize_probe_state = true;
  constexpr glm::ivec3 probe_counts(8, 6, 8);
  DdgiProbeRegionSummary lit_side;
  DdgiProbeRegionSummary shadow_side;
  for (size_t frame_index = 0; frame_index < 3; ++frame_index) {
    if (!application.Loop()) {
      return FailSmokeTest(application, "application ended before thin-wall DDGI leak validation completed");
    }
    const auto debug_data = render_layer->GetDdgiProbeDebugData(true);
    const auto frame_lit_side = SummarizeDdgiProbeRegion(debug_data, probe_counts, {1, 3, 2}, {3, 5, 6});
    const auto frame_shadow_side = SummarizeDdgiProbeRegion(debug_data, probe_counts, {5, 3, 2}, {7, 5, 6});
    if (frame_index == 0 || frame_lit_side.average_irradiance > lit_side.average_irradiance) {
      lit_side = frame_lit_side;
      shadow_side = frame_shadow_side;
    }
  }
  restore_ddgi_settings.Run();

  if (lit_side.active_probe_count == 0u || shadow_side.active_probe_count == 0u) {
    return FailSmokeTest(application, "thin-wall DDGI leak validation has no active probes in a comparison region");
  }
  if (!lit_side.metadata_is_finite || !shadow_side.metadata_is_finite) {
    return FailSmokeTest(application, "thin-wall DDGI leak validation metadata is not finite");
  }
  if (lit_side.average_irradiance <= 0.001f) {
    return FailSmokeTest(application, "thin-wall DDGI leak validation did not light the source-side probes lit=" +
                                          std::to_string(lit_side.average_irradiance) +
                                          " shadow=" + std::to_string(shadow_side.average_irradiance));
  }
  const auto shadow_limit = glm::max(lit_side.average_irradiance * 0.35f, 0.002f);
  if (shadow_side.average_irradiance >= shadow_limit) {
    return FailSmokeTest(application, "thin-wall DDGI leak validation leaked too much light through the blocker lit=" +
                                          std::to_string(lit_side.average_irradiance) +
                                          " shadow=" + std::to_string(shadow_side.average_irradiance) +
                                          " limit=" + std::to_string(shadow_limit));
  }
  return ValidateThinWallDdgiState(application, config);
}

int ValidateThinWallDdgiSurfaceLeakReadback(Application& application, const DemoAppRuntimeConfig& config) {
  if (config.demo_setup != DemoSetup::ThinWall) {
    return 0;
  }
  const auto scene = application.GetActiveScene();
  if (!scene) {
    return FailSmokeTest(application, "active scene is missing for thin-wall DDGI surface validation");
  }
  const auto render_layer = application.GetLayer<RenderLayer>();
  if (!render_layer) {
    return FailSmokeTest(application, "render layer is missing for thin-wall DDGI surface validation");
  }
  const auto main_camera = scene->main_camera.Get<Camera>();
  if (!main_camera || !main_camera->GetRenderTexture()) {
    return FailSmokeTest(application, "main camera render texture is missing for thin-wall DDGI surface validation");
  }

  auto& ddgi_settings = render_layer->GetDdgiSettings();
  const auto original_reset_probe_history = ddgi_settings.runtime.reset_probe_history;
  const auto original_visualize_probe_state = ddgi_settings.debug.visualize_probe_state;
  const auto original_hysteresis = ddgi_settings.runtime.hysteresis;
  auto restore_ddgi_settings = MakeScopeExit([&]() {
    ddgi_settings.runtime.reset_probe_history = original_reset_probe_history;
    ddgi_settings.runtime.hysteresis = original_hysteresis;
    ddgi_settings.debug.visualize_probe_state = original_visualize_probe_state;
  });

  ddgi_settings.runtime.hysteresis = 0.0f;
  ddgi_settings.runtime.reset_probe_history = true;
  ddgi_settings.debug.visualize_probe_state = false;
  for (size_t frame_index = 0; frame_index < 3; ++frame_index) {
    if (!application.Loop()) {
      return FailSmokeTest(application, "application ended before thin-wall DDGI surface atlas seed completed");
    }
  }
  ddgi_settings.runtime.reset_probe_history = false;
  for (size_t frame_index = 0; frame_index < 2; ++frame_index) {
    if (!application.Loop()) {
      return FailSmokeTest(application, "application ended before thin-wall DDGI surface validation completed");
    }
  }

  std::vector<glm::vec4> pixels;
  main_camera->GetRenderTexture()->GetRgbaChannelData(pixels);
  restore_ddgi_settings.Run();

  const auto resolution = main_camera->GetSize();
  const auto lit_side = SummarizeRenderTextureRegion(pixels, resolution, {0.425f, 0.37f}, {0.492f, 0.56f});
  const auto shadow_side = SummarizeRenderTextureRegion(pixels, resolution, {0.508f, 0.37f}, {0.575f, 0.56f});
  if (lit_side.sample_count == 0u || shadow_side.sample_count == 0u || !lit_side.finite || !shadow_side.finite) {
    return FailSmokeTest(application, "thin-wall DDGI surface validation read invalid main-camera pixels");
  }
  if (lit_side.average_luminance <= 0.01f) {
    return FailSmokeTest(application, "thin-wall DDGI surface validation did not light the source side lit=" +
                                          std::to_string(lit_side.average_luminance) +
                                          " shadow=" + std::to_string(shadow_side.average_luminance));
  }
  const auto shadow_limit = glm::max(lit_side.average_luminance * 0.45f, 0.01f);
  if (shadow_side.average_luminance >= shadow_limit) {
    return FailSmokeTest(application, "thin-wall DDGI surface validation leaked too much visible light lit=" +
                                          std::to_string(lit_side.average_luminance) +
                                          " shadow=" + std::to_string(shadow_side.average_luminance) +
                                          " limit=" + std::to_string(shadow_limit));
  }
  return ValidateThinWallDdgiState(application, config);
}

int ValidateRenderingDemoDdgiProbeDebugReadback(Application& application, const DemoAppRuntimeConfig& config,
                                                const char* phase) {
  if (config.demo_setup != DemoSetup::Rendering) {
    return 0;
  }
  const auto render_layer = application.GetLayer<RenderLayer>();
  if (!render_layer) {
    return FailSmokeTest(application, "render layer is missing for DDGI probe debug readback validation");
  }

  auto& ddgi_settings = render_layer->GetDdgiSettings();
  const auto original_visualize_probe_state = ddgi_settings.debug.visualize_probe_state;
  auto restore_ddgi_settings = MakeScopeExit([&]() {
    ddgi_settings.debug.visualize_probe_state = original_visualize_probe_state;
  });
  ddgi_settings.debug.visualize_probe_state = true;
  if (!application.Loop()) {
    return FailSmokeTest(application, "application ended before DDGI probe debug readback validation completed");
  }
  const auto summary = SummarizeDdgiProbeDebugReadback(render_layer->GetDdgiProbeDebugData(true));
  restore_ddgi_settings.Run();

  const auto phase_suffix = std::string(" during ") + phase;
  if (summary.active_probe_count == 0) {
    return FailSmokeTest(application, "DDGI probe debug readback has no active probes" + phase_suffix);
  }
  if (!summary.has_hit_probe) {
    return FailSmokeTest(application, "DDGI probe debug readback has no hit-ratio probe" + phase_suffix);
  }
  if (!summary.has_miss_probe) {
    return FailSmokeTest(application, "DDGI probe debug readback has no miss-ratio probe" + phase_suffix);
  }
  if (!summary.has_backface_probe) {
    return FailSmokeTest(application, "DDGI probe debug readback has no backface-ratio probe" + phase_suffix);
  }
  return ValidateRenderingDemoDdgiState(application, config);
}

int ValidateRenderingDemoDdgiDisabledLightingReadback(Application& application, const DemoAppRuntimeConfig& config) {
  if (config.demo_setup != DemoSetup::Rendering) {
    return 0;
  }
  const auto scene = application.GetActiveScene();
  if (!scene) {
    return FailSmokeTest(application, "active scene is missing for DDGI disabled-light validation");
  }
  const auto render_layer = application.GetLayer<RenderLayer>();
  if (!render_layer) {
    return FailSmokeTest(application, "render layer is missing for DDGI disabled-light validation");
  }
  const auto main_camera = scene->main_camera.Get<Camera>();
  if (!main_camera || !main_camera->GetRenderTexture()) {
    return FailSmokeTest(application, "main camera render texture is missing for DDGI disabled-light validation");
  }

  std::vector<std::pair<std::shared_ptr<IPrivateComponent>, bool>> light_component_states;
  AppendLightComponentStates<DirectionalLight>(scene, light_component_states);
  AppendLightComponentStates<PointLight>(scene, light_component_states);
  AppendLightComponentStates<SpotLight>(scene, light_component_states);
  if (light_component_states.empty()) {
    return FailSmokeTest(application, "Rendering demo has no lights for DDGI disabled-light validation");
  }

  auto& ddgi_settings = render_layer->GetDdgiSettings();
  const auto original_reset_probe_history = ddgi_settings.runtime.reset_probe_history;
  const auto original_visualize_probe_state = ddgi_settings.debug.visualize_probe_state;
  auto restore_disabled_light_state = MakeScopeExit([&]() {
    for (const auto& [light, enabled] : light_component_states) {
      light->SetEnabled(enabled);
    }
    ddgi_settings.runtime.reset_probe_history = original_reset_probe_history;
    ddgi_settings.debug.visualize_probe_state = original_visualize_probe_state;
  });
  for (const auto& [light, enabled] : light_component_states) {
    light->SetEnabled(false);
  }
  ddgi_settings.runtime.reset_probe_history = true;
  ddgi_settings.debug.visualize_probe_state = true;
  main_camera->ResetFrameCount();
  if (!application.Loop()) {
    return FailSmokeTest(application, "application ended before DDGI disabled-light validation completed");
  }
  if ((render_layer->GetDdgiLastProbeUpdateReasons() & RenderLayer::DdgiUpdateReasonManualReset) == 0u) {
    return FailSmokeTest(application, "DDGI disabled-light validation did not refresh history");
  }

  const auto summary = SummarizeDdgiProbeDebugReadback(render_layer->GetDdgiProbeDebugData(true));
  if (summary.active_probe_count == 0u) {
    return FailSmokeTest(application, "DDGI disabled-light validation has no active probes");
  }
  if (summary.max_active_irradiance > 0.01f || summary.average_active_irradiance > 0.001f) {
    return FailSmokeTest(application, "DDGI disabled-light validation kept stale probe irradiance");
  }
  for (size_t frame_index = 0; frame_index < 2; ++frame_index) {
    main_camera->ResetFrameCount();
    if (!application.Loop()) {
      return FailSmokeTest(application, "application ended before DDGI disabled-light surface validation completed");
    }
  }
  std::vector<glm::vec4> pixels;
  main_camera->GetRenderTexture()->GetRgbaChannelData(pixels);
  const auto center_surface =
      SummarizeRenderTextureRegion(pixels, main_camera->GetSize(), {0.42f, 0.34f}, {0.58f, 0.66f});
  if (center_surface.sample_count == 0u || !center_surface.finite) {
    return FailSmokeTest(application, "DDGI disabled-light surface validation read invalid main-camera pixels");
  }
  if (center_surface.average_luminance > 0.035f) {
    return FailSmokeTest(application, "DDGI disabled-light surface validation kept visible stale lighting luminance=" +
                                          std::to_string(center_surface.average_luminance));
  }

  restore_disabled_light_state.Run();
  if (!application.Loop()) {
    return FailSmokeTest(application, "application ended before DDGI disabled-light restore completed");
  }
  return ValidateRenderingDemoDdgiState(application, config);
}

int ValidateRenderingDemoDdgiSponzaHallwaySurfaceReadback(Application& application,
                                                          const DemoAppRuntimeConfig& config) {
  if (config.demo_setup != DemoSetup::Rendering) {
    return 0;
  }
  const auto scene = application.GetActiveScene();
  if (!scene) {
    return FailSmokeTest(application, "active scene is missing for Sponza hallway DDGI validation");
  }
  const auto render_layer = application.GetLayer<RenderLayer>();
  if (!render_layer) {
    return FailSmokeTest(application, "render layer is missing for Sponza hallway DDGI validation");
  }
  const auto main_camera = scene->main_camera.Get<Camera>();
  if (!main_camera || !main_camera->GetRenderTexture()) {
    return FailSmokeTest(application, "main camera render texture is missing for Sponza hallway DDGI validation");
  }

  auto& ddgi_settings = render_layer->GetDdgiSettings();
  const auto original_reset_probe_history = ddgi_settings.runtime.reset_probe_history;
  const auto original_visualize_probe_state = ddgi_settings.debug.visualize_probe_state;
  auto restore_ddgi_settings = MakeScopeExit([&]() {
    ddgi_settings.runtime.reset_probe_history = original_reset_probe_history;
    ddgi_settings.debug.visualize_probe_state = original_visualize_probe_state;
  });
  ddgi_settings.runtime.reset_probe_history = true;
  ddgi_settings.debug.visualize_probe_state = false;
  for (size_t frame_index = 0; frame_index < 4; ++frame_index) {
    main_camera->ResetFrameCount();
    if (!application.Loop()) {
      return FailSmokeTest(application, "application ended before Sponza hallway DDGI validation completed");
    }
  }

  std::vector<glm::vec4> pixels;
  main_camera->GetRenderTexture()->GetRgbaChannelData(pixels);
  const auto resolution = main_camera->GetSize();
  const auto left_hall = SummarizeRenderTextureRegion(pixels, resolution, {0.18f, 0.36f}, {0.36f, 0.64f});
  const auto center_hall = SummarizeRenderTextureRegion(pixels, resolution, {0.41f, 0.34f}, {0.59f, 0.66f});
  const auto right_hall = SummarizeRenderTextureRegion(pixels, resolution, {0.64f, 0.36f}, {0.82f, 0.64f});
  if (left_hall.sample_count == 0u || center_hall.sample_count == 0u || right_hall.sample_count == 0u ||
      !left_hall.finite || !center_hall.finite || !right_hall.finite) {
    return FailSmokeTest(application, "Sponza hallway DDGI validation read invalid main-camera pixels");
  }
  const auto lit_luminance =
      glm::max(left_hall.average_luminance, glm::max(center_hall.average_luminance, right_hall.average_luminance));
  const auto shadow_luminance =
      glm::min(left_hall.average_luminance, glm::min(center_hall.average_luminance, right_hall.average_luminance));
  if (lit_luminance < 0.04f) {
    return FailSmokeTest(application, "Sponza hallway DDGI validation did not light any sampled hallway region lit=" +
                                          std::to_string(lit_luminance));
  }
  if (shadow_luminance > lit_luminance * 0.85f && shadow_luminance > 0.04f) {
    return FailSmokeTest(application, "Sponza hallway DDGI validation has weak lit/shadow contrast lit=" +
                                          std::to_string(lit_luminance) +
                                          " shadow=" + std::to_string(shadow_luminance));
  }

  restore_ddgi_settings.Run();
  return ValidateRenderingDemoDdgiState(application, config);
}

int ValidateRenderingDemoDdgiRelocationReadback(Application& application, const DemoAppRuntimeConfig& config) {
  if (config.demo_setup != DemoSetup::Rendering) {
    return 0;
  }
  const auto scene = application.GetActiveScene();
  if (!scene) {
    return FailSmokeTest(application, "active scene is missing for DDGI relocation validation");
  }
  const auto render_layer = application.GetLayer<RenderLayer>();
  if (!render_layer) {
    return FailSmokeTest(application, "render layer is missing for DDGI relocation validation");
  }
  const auto volume = FindEnvironmentalLightingDdgiVolume(scene, "DDGI Probe Volume");
  if (!volume) {
    return FailSmokeTest(application, "DDGI probe volume is missing for DDGI relocation validation");
  }

  auto& ddgi_settings = render_layer->GetDdgiSettings();
  const auto original_relocation_enabled = volume->enable_probe_relocation;
  const auto original_relocation_distance = volume->relocation_distance;
  const auto original_reset_probe_history = ddgi_settings.runtime.reset_probe_history;
  const auto original_visualize_probe_state = ddgi_settings.debug.visualize_probe_state;
  auto restore_relocation_state = MakeScopeExit([&]() {
    volume->enable_probe_relocation = original_relocation_enabled;
    volume->relocation_distance = original_relocation_distance;
    ddgi_settings.runtime.reset_probe_history = original_reset_probe_history;
    ddgi_settings.debug.visualize_probe_state = original_visualize_probe_state;
  });
  volume->enable_probe_relocation = true;
  volume->relocation_distance = glm::max(original_relocation_distance, 0.5f);
  ddgi_settings.runtime.reset_probe_history = true;
  ddgi_settings.debug.visualize_probe_state = true;
  if (!application.Loop()) {
    return FailSmokeTest(application, "application ended before DDGI relocation validation completed");
  }
  if ((render_layer->GetDdgiLastProbeUpdateReasons() & RenderLayer::DdgiUpdateReasonManualReset) == 0u) {
    return FailSmokeTest(application, "DDGI relocation validation did not refresh probe history");
  }

  const auto summary = SummarizeDdgiProbeDebugReadback(render_layer->GetDdgiProbeDebugData(true));
  if (summary.active_probe_count == 0u) {
    return FailSmokeTest(application, "DDGI relocation validation has no active probes");
  }
  if (summary.relocated_active_probe_count == 0u) {
    return FailSmokeTest(application, "DDGI relocation validation did not report any relocated active probes");
  }

  restore_relocation_state.Run();
  if (!application.Loop()) {
    return FailSmokeTest(application, "application ended before DDGI relocation restore completed");
  }
  return ValidateRenderingDemoDdgiState(application, config);
}

int ValidateRenderingDemoDdgiClassificationReadback(Application& application, const DemoAppRuntimeConfig& config) {
  if (config.demo_setup != DemoSetup::Rendering) {
    return 0;
  }
  const auto scene = application.GetActiveScene();
  if (!scene) {
    return FailSmokeTest(application, "active scene is missing for DDGI classification validation");
  }
  const auto render_layer = application.GetLayer<RenderLayer>();
  if (!render_layer) {
    return FailSmokeTest(application, "render layer is missing for DDGI classification validation");
  }
  const auto volume = FindEnvironmentalLightingDdgiVolume(scene, "DDGI Probe Volume");
  if (!volume) {
    return FailSmokeTest(application, "DDGI probe volume is missing for DDGI classification validation");
  }

  auto& ddgi_settings = render_layer->GetDdgiSettings();
  const auto original_classification_enabled = volume->enable_probe_classification;
  const auto original_reset_probe_history = ddgi_settings.runtime.reset_probe_history;
  const auto original_visualize_probe_state = ddgi_settings.debug.visualize_probe_state;
  auto restore_classification_state = MakeScopeExit([&]() {
    volume->enable_probe_classification = original_classification_enabled;
    ddgi_settings.runtime.reset_probe_history = original_reset_probe_history;
    ddgi_settings.debug.visualize_probe_state = original_visualize_probe_state;
  });
  volume->enable_probe_classification = true;
  ddgi_settings.runtime.reset_probe_history = true;
  ddgi_settings.debug.visualize_probe_state = true;
  if (!application.Loop()) {
    return FailSmokeTest(application, "application ended before DDGI classification validation completed");
  }
  if ((render_layer->GetDdgiLastProbeUpdateReasons() & RenderLayer::DdgiUpdateReasonSource) == 0u ||
      (render_layer->GetDdgiLastProbeUpdateReasons() & RenderLayer::DdgiUpdateReasonManualReset) == 0u) {
    return FailSmokeTest(application, "DDGI classification validation did not refresh probe source and history");
  }

  const auto summary = SummarizeDdgiProbeDebugReadback(render_layer->GetDdgiProbeDebugData(true));
  if (summary.active_probe_count == 0u) {
    return FailSmokeTest(application, "DDGI classification marked every probe inactive");
  }
  if (summary.inactive_probe_count == 0u) {
    return FailSmokeTest(application, "DDGI classification did not prune any inactive probes");
  }

  restore_classification_state.Run();
  if (!application.Loop()) {
    return FailSmokeTest(application, "application ended before DDGI classification restore completed");
  }
  if ((render_layer->GetDdgiLastProbeUpdateReasons() & RenderLayer::DdgiUpdateReasonSource) == 0u) {
    return FailSmokeTest(application, "DDGI classification validation did not restore the authored volume source");
  }
  return ValidateRenderingDemoDdgiState(application, config);
}

int ValidateRenderingDemoDdgiIdleReadbackStability(Application& application, const DemoAppRuntimeConfig& config) {
  if (config.demo_setup != DemoSetup::Rendering) {
    return 0;
  }
  const auto render_layer = application.GetLayer<RenderLayer>();
  if (!render_layer) {
    return FailSmokeTest(application, "render layer is missing for DDGI idle readback validation");
  }

  auto& ddgi_settings = render_layer->GetDdgiSettings();
  const auto original_visualize_probe_state = ddgi_settings.debug.visualize_probe_state;
  auto restore_ddgi_settings = MakeScopeExit([&]() {
    ddgi_settings.debug.visualize_probe_state = original_visualize_probe_state;
  });
  ddgi_settings.debug.visualize_probe_state = true;

  DdgiProbeDebugReadbackSummary baseline_summary;
  constexpr size_t idle_readback_frame_count = 3;
  constexpr float max_expected_probe_irradiance = 4096.0f;
  for (size_t frame_index = 0; frame_index < idle_readback_frame_count; ++frame_index) {
    if (!application.Loop()) {
      return FailSmokeTest(application, "application ended before DDGI idle readback validation completed");
    }
    if (render_layer->GetDdgiLastProbeUpdateReasons() != RenderLayer::DdgiUpdateReasonSteadyState) {
      return FailSmokeTest(application, "DDGI reported a scene refresh during idle readback validation");
    }

    const auto summary = SummarizeDdgiProbeDebugReadback(render_layer->GetDdgiProbeDebugData(true));
    if (summary.active_probe_count == 0u) {
      return FailSmokeTest(application, "DDGI idle readback validation has no active probes");
    }
    if (!summary.metadata_is_finite || !std::isfinite(summary.average_active_irradiance) ||
        !std::isfinite(summary.max_active_irradiance)) {
      return FailSmokeTest(application, "DDGI idle readback metadata is not finite");
    }
    if (summary.max_active_irradiance > max_expected_probe_irradiance) {
      return FailSmokeTest(application, "DDGI idle readback irradiance exceeded the stability bound");
    }
    if (frame_index == 0u) {
      baseline_summary = summary;
      continue;
    }
    if (summary.active_probe_count != baseline_summary.active_probe_count ||
        summary.inactive_probe_count != baseline_summary.inactive_probe_count) {
      return FailSmokeTest(application, "DDGI idle readback changed probe active-state counts");
    }
  }

  restore_ddgi_settings.Run();
  return ValidateRenderingDemoDdgiState(application, config);
}

int ValidateRenderingDemoDdgiLightRefresh(Application& application, const DemoAppRuntimeConfig& config) {
  if (config.demo_setup != DemoSetup::Rendering) {
    return 0;
  }
  const auto scene = application.GetActiveScene();
  if (!scene) {
    return FailSmokeTest(application, "active scene is missing for DDGI light-refresh validation");
  }
  const auto render_layer = application.GetLayer<RenderLayer>();
  if (!render_layer) {
    return FailSmokeTest(application, "render layer is missing for DDGI light-refresh validation");
  }
  const auto point_light_entity = FindRenderingDemoPointLightEntity(scene);
  if (!point_light_entity) {
    return FailSmokeTest(application, "Rendering demo point light is missing for DDGI light-refresh validation");
  }
  const auto point_light = scene->GetOrSetPrivateComponent<PointLight>(*point_light_entity).lock();
  if (!point_light) {
    return FailSmokeTest(application,
                         "Rendering demo point light component is missing for DDGI light-refresh validation");
  }

  const auto loop_after_light_mutation = [&](const char* failure_reason) {
    if (!application.Loop()) {
      return FailSmokeTest(application, failure_reason);
    }
    return 0;
  };

  const auto original_transform = scene->GetDataComponent<Transform>(*point_light_entity);
  const auto original_light_enabled = point_light->IsEnabled();
  auto restore_light_state = MakeScopeExit([&]() {
    point_light->SetEnabled(original_light_enabled);
    scene->SetDataComponent(*point_light_entity, original_transform);
  });
  auto moved_transform = original_transform;
  moved_transform.SetPosition(original_transform.GetPosition() + glm::vec3(0.0f, 0.0f, 0.25f));
  scene->SetDataComponent(*point_light_entity, moved_transform);
  if (const auto result =
          loop_after_light_mutation("application ended before DDGI point-light move validation completed");
      result != 0) {
    return result;
  }

  point_light->SetEnabled(false);
  if (const auto result =
          loop_after_light_mutation("application ended before DDGI point-light disable validation completed");
      result != 0) {
    return result;
  }

  point_light->SetEnabled(true);
  restore_light_state.Run();
  if (const auto result =
          loop_after_light_mutation("application ended before DDGI point-light restore validation completed");
      result != 0) {
    return result;
  }
  return ValidateRenderingDemoDdgiState(application, config);
}

int ValidateRenderingDemoDdgiManualReset(Application& application, const DemoAppRuntimeConfig& config) {
  if (config.demo_setup != DemoSetup::Rendering) {
    return 0;
  }
  const auto render_layer = application.GetLayer<RenderLayer>();
  if (!render_layer) {
    return FailSmokeTest(application, "render layer is missing for DDGI manual-reset validation");
  }
  render_layer->GetDdgiSettings().runtime.reset_probe_history = true;
  if (!application.Loop()) {
    return FailSmokeTest(application, "application ended before DDGI manual-reset validation completed");
  }
  if ((render_layer->GetDdgiLastProbeUpdateReasons() & RenderLayer::DdgiUpdateReasonManualReset) == 0u) {
    return FailSmokeTest(application, "DDGI did not report a manual reset refresh");
  }
  if (render_layer->GetDdgiSettings().runtime.reset_probe_history) {
    return FailSmokeTest(application, "DDGI manual reset request was not consumed");
  }
  return ValidateRenderingDemoDdgiState(application, config);
}

int ValidateRenderingDemoDdgiMaterialRefresh(Application& application, const DemoAppRuntimeConfig& config) {
  if (config.demo_setup != DemoSetup::Rendering) {
    return 0;
  }
  const auto scene = application.GetActiveScene();
  if (!scene) {
    return FailSmokeTest(application, "active scene is missing for DDGI material-refresh validation");
  }
  const auto render_layer = application.GetLayer<RenderLayer>();
  if (!render_layer) {
    return FailSmokeTest(application, "render layer is missing for DDGI material-refresh validation");
  }
  const auto point_light_entity = FindRenderingDemoPointLightEntity(scene);
  if (!point_light_entity) {
    return FailSmokeTest(application, "Rendering demo point light is missing for DDGI material-refresh validation");
  }
  const auto mesh_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(*point_light_entity).lock();
  const auto material = mesh_renderer ? mesh_renderer->material.Get<Material>() : nullptr;
  if (!material) {
    return FailSmokeTest(application, "Rendering demo point-light material is missing for DDGI material validation");
  }

  const auto original_material_data = material->material_data;
  auto restore_material_state = MakeScopeExit([&]() {
    material->SetGltfMaterialData(original_material_data);
  });
  material->material_data.shade_material.pbr_base_color_factor = glm::vec4(1.0f, 0.6f, 0.15f, 1.0f);
  material->MarkDirty();
  if (!application.Loop()) {
    return FailSmokeTest(application, "application ended before DDGI material-change validation completed");
  }

  restore_material_state.Run();
  if (!application.Loop()) {
    return FailSmokeTest(application, "application ended before DDGI material-restore validation completed");
  }
  return ValidateRenderingDemoDdgiState(application, config);
}

int ValidateRenderingDemoDdgiSourceRefresh(Application& application, const DemoAppRuntimeConfig& config) {
  if (config.demo_setup != DemoSetup::Rendering) {
    return 0;
  }
  const auto scene = application.GetActiveScene();
  if (!scene) {
    return FailSmokeTest(application, "active scene is missing for DDGI source-refresh validation");
  }
  const auto render_layer = application.GetLayer<RenderLayer>();
  if (!render_layer) {
    return FailSmokeTest(application, "render layer is missing for DDGI source-refresh validation");
  }
  const auto volume = FindEnvironmentalLightingDdgiVolume(scene, "DDGI Probe Volume");
  if (!volume) {
    return FailSmokeTest(application, "DDGI probe volume is missing for DDGI source-refresh validation");
  }

  const auto original_volume_origin = volume->volume_origin;
  auto restore_source_state = MakeScopeExit([&]() {
    volume->volume_origin = original_volume_origin;
  });
  volume->volume_origin = original_volume_origin + glm::vec3(0.25f, 0.0f, 0.0f);
  if (!application.Loop()) {
    return FailSmokeTest(application, "application ended before DDGI source-change validation completed");
  }
  if ((render_layer->GetDdgiLastProbeUpdateReasons() & RenderLayer::DdgiUpdateReasonSource) == 0u) {
    return FailSmokeTest(application, "DDGI did not report a source refresh after DDGI volume mutation");
  }

  restore_source_state.Run();
  if (!application.Loop()) {
    return FailSmokeTest(application, "application ended before DDGI source-restore validation completed");
  }
  if ((render_layer->GetDdgiLastProbeUpdateReasons() & RenderLayer::DdgiUpdateReasonSource) == 0u) {
    return FailSmokeTest(application, "DDGI did not report a source refresh after DDGI volume restore");
  }
  return ValidateRenderingDemoDdgiState(application, config);
}

int ValidateRenderingDemoDdgiIdleSteadyState(Application& application, const DemoAppRuntimeConfig& config) {
  if (config.demo_setup != DemoSetup::Rendering) {
    return 0;
  }
  const auto render_layer = application.GetLayer<RenderLayer>();
  if (!render_layer) {
    return FailSmokeTest(application, "render layer is missing for DDGI idle validation");
  }
  if (!application.Loop()) {
    return FailSmokeTest(application, "application ended before DDGI idle validation completed");
  }
  if (render_layer->GetDdgiLastProbeUpdateReasons() != RenderLayer::DdgiUpdateReasonSteadyState) {
    return FailSmokeTest(application, "DDGI reported a scene refresh during idle editor validation");
  }
  return ValidateRenderingDemoDdgiState(application, config);
}

void ApplyReadmeScreenshotEditorSetup(const DemoAppRuntimeConfig* config = nullptr);

int RunSmokeTest(const DemoAppRuntimeConfig& config) {
  auto& application = ApplicationContext::Get();
  const bool expect_player_autoplay = config.application_mode == ApplicationMode::Player;
  application.Start(expect_player_autoplay);

  size_t load_frame_count = 0;
  while (!ProjectManager::IsProjectIdle()) {
    if (!application.Loop()) {
      return FailSmokeTest(application, "application ended before project load completed");
    }
    ++load_frame_count;
    if (load_frame_count >= config.max_load_frames) {
      return FailSmokeTest(application, "project load timed out");
    }
  }

  if (!ProjectManager::GetStartScene().lock()) {
    return FailSmokeTest(application, "start scene is missing after project load");
  }
  if (!application.GetActiveScene()) {
    return FailSmokeTest(application, "active scene is missing after project load");
  }
  ApplyReadmeScreenshotEditorSetup();
  if (const auto validation_result = ValidateRenderingDemoDdgiState(application, config); validation_result != 0) {
    return validation_result;
  }
  if (const auto validation_result = ValidateCornellBoxDdgiState(application, config); validation_result != 0) {
    return validation_result;
  }
  if (const auto validation_result = ValidateThinWallDdgiState(application, config); validation_result != 0) {
    return validation_result;
  }

  const auto loaded_frame = Platform::GetFrameCount();
  if (expect_player_autoplay) {
    if (!application.IsPlaying()) {
      return FailSmokeTest(application, "player mode did not enter play mode automatically");
    }
  } else {
    for (size_t frame_index = 0; frame_index < config.warmup_frames; ++frame_index) {
      if (!application.Loop()) {
        return FailSmokeTest(application, "application ended before smoke warmup completed");
      }
    }
    if (const auto validation_result = ValidateRenderingDemoDdgiState(application, config); validation_result != 0) {
      return validation_result;
    }
    if (const auto validation_result = ValidateCornellBoxDdgiState(application, config); validation_result != 0) {
      return validation_result;
    }
    if (const auto validation_result = ValidateThinWallDdgiState(application, config); validation_result != 0) {
      return validation_result;
    }
    if (const auto validation_result = ValidateCornellBoxDdgiProbeReadback(application, config);
        validation_result != 0) {
      return validation_result;
    }
    if (const auto validation_result = ValidateCornellBoxDdgiClassificationSurfaceReadback(application, config);
        validation_result != 0) {
      return validation_result;
    }
    if (const auto validation_result = ValidateThinWallDdgiProbeLeakReadback(application, config);
        validation_result != 0) {
      return validation_result;
    }
    if (const auto validation_result = ValidateThinWallDdgiSurfaceLeakReadback(application, config);
        validation_result != 0) {
      return validation_result;
    }
    if (const auto validation_result =
            ValidateRenderingDemoDdgiProbeDebugReadback(application, config, "warmup validation");
        validation_result != 0) {
      return validation_result;
    }
    if (const auto validation_result = ValidateRenderingDemoDdgiSponzaHallwaySurfaceReadback(application, config);
        validation_result != 0) {
      return validation_result;
    }
    if (const auto validation_result = ValidateRenderingDemoDdgiLightRefresh(application, config);
        validation_result != 0) {
      return validation_result;
    }
    if (const auto validation_result =
            ValidateRenderingDemoDdgiProbeDebugReadback(application, config, "light-refresh validation");
        validation_result != 0) {
      return validation_result;
    }
    if (const auto validation_result = ValidateRenderingDemoDdgiDisabledLightingReadback(application, config);
        validation_result != 0) {
      return validation_result;
    }
    if (const auto validation_result = ValidateRenderingDemoDdgiRelocationReadback(application, config);
        validation_result != 0) {
      return validation_result;
    }
    if (const auto validation_result = ValidateRenderingDemoDdgiManualReset(application, config);
        validation_result != 0) {
      return validation_result;
    }
    if (const auto validation_result = ValidateRenderingDemoDdgiMaterialRefresh(application, config);
        validation_result != 0) {
      return validation_result;
    }
    if (const auto validation_result = ValidateRenderingDemoDdgiSourceRefresh(application, config);
        validation_result != 0) {
      return validation_result;
    }
    if (const auto validation_result = ValidateRenderingDemoDdgiClassificationReadback(application, config);
        validation_result != 0) {
      return validation_result;
    }
    if (const auto validation_result = ValidateRenderingDemoDdgiIdleReadbackStability(application, config);
        validation_result != 0) {
      return validation_result;
    }
    if (const auto validation_result = ValidateRenderingDemoDdgiIdleSteadyState(application, config);
        validation_result != 0) {
      return validation_result;
    }
    application.Play();
    if (!application.IsPlaying()) {
      return FailSmokeTest(application, "application did not enter play mode");
    }
  }

  const auto play_start_frame = Platform::GetFrameCount();
  size_t play_loop_count = 0;
  while (Platform::GetFrameCount() - play_start_frame < config.frames_after_play) {
    if (!application.Loop()) {
      return FailSmokeTest(application, "application ended before requested play frames completed");
    }
    ++play_loop_count;
    if (play_loop_count >= config.max_play_frames) {
      return FailSmokeTest(application, "play frame wait timed out");
    }
  }

  std::cout << "EVOENGINE_APP_TEST_RESULT passed"
            << " loaded_frame=" << loaded_frame << " play_start_frame=" << play_start_frame
            << " final_frame=" << Platform::GetFrameCount() << std::endl;
  if (!config.exit_on_complete) {
    application.Run();
  } else {
    application.Stop();
    application.End();
  }
  return 0;
}

void WriteMarkerFile(const std::filesystem::path& path) {
  if (path.empty()) {
    throw std::invalid_argument("Marker file path is empty.");
  }
  if (const auto parent = path.parent_path(); !parent.empty()) {
    std::filesystem::create_directories(parent);
  }
  std::ofstream marker(path);
  if (!marker) {
    throw std::runtime_error("Failed to write marker file: " + path.string());
  }
  marker << "ready" << std::endl;
}

std::optional<Entity> FindEntityByName(const std::shared_ptr<Scene>& scene, const std::string& name) {
  if (!scene) {
    return {};
  }
  for (const auto& entity : scene->UnsafeGetAllEntities()) {
    if (scene->IsEntityValid(entity) && scene->GetEntityName(entity) == name) {
      return entity;
    }
  }
  return {};
}

void ApplyReadmeScreenshotDdgiInspectionOptions(const DemoAppRuntimeConfig* config) {
  if (!config) {
    return;
  }
  const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>();
  if (!render_layer) {
    return;
  }
  const auto force_ddgi_layout = config->ddgi_atlas_preview || config->ddgi_ray_overlay;
  if (config->inspect_render_layer || force_ddgi_layout) {
    render_layer->enable_inspection = true;
  }
  render_layer->force_ddgi_inspection_layout = force_ddgi_layout;
  if (!force_ddgi_layout) {
    return;
  }
  auto& settings = render_layer->GetDdgiSettings();
  settings.debug.enabled = true;
  if (config->ddgi_atlas_preview) {
    settings.debug.visualize_probe_state = true;
  }
  if (config->ddgi_ray_overlay) {
    settings.debug.show_rays = true;
    settings.debug.visualize_probe_state = true;
    settings.debug.visualize_selected_probe = true;
  }
}

void ApplyReadmeScreenshotEditorSetup(const DemoAppRuntimeConfig* config) {
  ApplyRenderingDemoEditorSetup();
  ApplyReadmeScreenshotDdgiInspectionOptions(config);
}

int FailEditorScreenshot(Application& application, const std::string& reason) {
  application.End();
  std::cerr << "EVOENGINE_EDITOR_SCREENSHOT_RESULT failed reason=\"" << reason << "\"" << std::endl;
  return 1;
}

int StoreEditorWindowScreenshot(Application& application, const DemoAppRuntimeConfig& config) {
  if (config.screenshot_file.empty()) {
    return 0;
  }
  const auto window_layer = application.GetLayer<WindowLayer>();
  if (!window_layer) {
    return FailEditorScreenshot(application, "window layer is missing");
  }
  window_layer->RequestScreenshot(config.screenshot_file);
  if (!application.Loop()) {
    return FailEditorScreenshot(application, "application ended before editor screenshot could be captured");
  }
  std::string error;
  if (!window_layer->StoreCompletedScreenshot(error)) {
    return FailEditorScreenshot(application, error);
  }
  std::cout << "EVOENGINE_EDITOR_SCREENSHOT_CAPTURED screenshot_file=\"" << config.screenshot_file.string() << "\""
            << std::endl;
  return 0;
}

int RunInteractiveDemo() {
  auto& application = ApplicationContext::Get();
  application.Start();
  while (!ProjectManager::IsProjectIdle()) {
    if (!application.Loop()) {
      return 0;
    }
  }
  ApplyReadmeScreenshotEditorSetup();
  application.Run();
  return 0;
}

int RunEditorScreenshot(const DemoAppRuntimeConfig& config) {
  if (config.ready_file.empty() && config.screenshot_file.empty()) {
    throw std::invalid_argument("editor_screenshot mode requires ready_file.");
  }
  if (config.done_file.empty() && config.screenshot_file.empty()) {
    throw std::invalid_argument("editor_screenshot mode requires done_file.");
  }

  auto& application = ApplicationContext::Get();
  application.Start(false);

  size_t load_frame_count = 0;
  while (!ProjectManager::IsProjectIdle()) {
    if (!application.Loop()) {
      return FailEditorScreenshot(application, "application ended before project load completed");
    }
    ++load_frame_count;
    if (load_frame_count >= config.max_load_frames) {
      return FailEditorScreenshot(application, "project load timed out");
    }
  }

  const auto editor_layer = application.GetLayer<EditorLayer>();
  if (!editor_layer) {
    return FailEditorScreenshot(application, "editor layer is missing");
  }
  ApplyReadmeScreenshotEditorSetup(&config);

  for (size_t frame_index = 0; frame_index < config.warmup_frames; ++frame_index) {
    if (!application.Loop()) {
      return FailEditorScreenshot(application, "application ended before screenshot warmup completed");
    }
  }

  if (const auto screenshot_result = StoreEditorWindowScreenshot(application, config); screenshot_result != 0) {
    return screenshot_result;
  }
  if (!config.ready_file.empty()) {
    WriteMarkerFile(config.ready_file);
    std::cout << "EVOENGINE_EDITOR_SCREENSHOT_READY ready_file=\"" << config.ready_file.string() << "\"" << std::endl;
  }
  if (config.exit_on_complete || config.done_file.empty()) {
    application.End();
    std::cout << "EVOENGINE_EDITOR_SCREENSHOT_RESULT passed" << std::endl;
    return 0;
  }

  while (!std::filesystem::exists(config.done_file)) {
    if (!application.Loop()) {
      return FailEditorScreenshot(application, "application ended before screenshot capture completed");
    }
  }

  application.End();
  std::cout << "EVOENGINE_EDITOR_SCREENSHOT_RESULT passed" << std::endl;
  return 0;
}
}  // namespace

int main(const int argc, char** argv) {
  Application application;
  bool initialized = false;
  bool automated_run = false;
  try {
    const auto command_line = ParseCommandLine(argc, argv);
    const auto run_config_path = FindRunConfigPath(command_line, argc, argv);
    std::optional<DemoAppRuntimeConfig> runtime_config;
    if (run_config_path) {
      runtime_config = LoadRunConfig(*run_config_path);
    }
    if (runtime_config && command_line.application_mode) {
      runtime_config->application_mode = *command_line.application_mode;
    }
    const auto demo_setup = runtime_config ? runtime_config->demo_setup : DemoSetup::Rendering;
    auto application_mode = command_line.application_mode.value_or(ApplicationMode::Editor);
    if (runtime_config) {
      application_mode = runtime_config->mode == DemoAppRunMode::EditorScreenshot ? ApplicationMode::Editor
                                                                                  : runtime_config->application_mode;
    }

    PushStandardApplicationLayers(application_mode);
#ifdef PHYSX_PHYSICS_SERVICE
    ApplicationContext::Get().PushLayer<PhysicsLayer>();
#endif

    ApplicationInitializationSettings application_info;
    SetupDemoScene(demo_setup, application_info);
    application_info.application_mode = application_mode;
    if (runtime_config && runtime_config->mode == DemoAppRunMode::EditorScreenshot) {
      application_info.default_window_size = {runtime_config->screenshot_width, runtime_config->screenshot_height};
    }
    application_info.use_custom_title_bar = true;
    ApplyApplicationModeDefaults(application_info);
    if (runtime_config && runtime_config->shadow_map_resolution_quality) {
      application_info.graphics_settings.SetShadowMapResolutionQuality(*runtime_config->shadow_map_resolution_quality);
    }
    if (command_line.shadow_map_resolution_quality) {
      application_info.graphics_settings.SetShadowMapResolutionQuality(*command_line.shadow_map_resolution_quality);
    }

    ApplicationContext::Get().Initialize(application_info);
    initialized = true;

    int exit_code = 0;
    automated_run = runtime_config && (runtime_config->mode == DemoAppRunMode::SmokeTest ||
                                       runtime_config->mode == DemoAppRunMode::EditorScreenshot);
    if (runtime_config && runtime_config->mode == DemoAppRunMode::SmokeTest) {
      exit_code = RunSmokeTest(*runtime_config);
    } else if (runtime_config && runtime_config->mode == DemoAppRunMode::EditorScreenshot) {
      exit_code = RunEditorScreenshot(*runtime_config);
    } else {
      exit_code = RunInteractiveDemo();
    }
    ApplicationContext::Get().Terminate();
    if (automated_run) {
      std::cout.flush();
      std::cerr.flush();
      std::_Exit(exit_code);
    }
    return exit_code;
  } catch (const std::exception& e) {
    std::cerr << "EVOENGINE_APP_TEST_RESULT failed reason=\"" << e.what() << "\"" << std::endl;
    if (initialized) {
      ApplicationContext::Get().Terminate();
    }
    if (automated_run) {
      std::cout.flush();
      std::cerr.flush();
      std::_Exit(1);
    }
    return 1;
  }
}
