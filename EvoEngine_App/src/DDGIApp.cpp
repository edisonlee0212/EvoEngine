#include "AppBootstrap.hpp"
#include "Application.hpp"
#include "Camera.hpp"
#include "DdgiVolume.hpp"
#include "DemoScene.hpp"
#include "Lights.hpp"
#include "Material.hpp"
#include "MeshRenderer.hpp"
#include "ProjectManager.hpp"
#include "WindowLayer.hpp"

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

#ifdef PHYSX_PHYSICS_SERVICE
#  include "PhysicsLayer.hpp"
#endif

using namespace evo_engine;

namespace {
const glm::ivec2 kComparisonExtent = {1024, 1024};
const glm::vec3 kCornellCameraPosition = {0.0f, 0.0f, 0.8f};
const glm::ivec3 kComparisonProbeCounts = {13, 13, 14};
const glm::vec3 kComparisonVolumeOrigin = {0.0f, 0.0f, 0.0f};
constexpr float kComparisonProbeSpacing = 0.14333334f;
constexpr float kComparisonPointLightBrightness = 2.0f;
constexpr float kComparisonDdgiIndirectIntensity = 1.0f;
constexpr float kComparisonCeilingLightEmission = 2.0f;
constexpr float kComparisonDdgiNormalBias = 0.02f;
constexpr float kComparisonDdgiViewBias = 0.05f;
constexpr bool kComparisonEnableProbeRelocation = true;
constexpr bool kComparisonEnableProbeClassification = true;

struct DdgiAppCommandLine {
  ApplicationMode application_mode = ApplicationMode::Player;
  size_t exit_after_frames = 0;
  size_t max_load_frames = 600;
  size_t screenshot_warmup_frames = 360;
  float point_light_brightness = kComparisonPointLightBrightness;
  float ddgi_indirect_intensity = kComparisonDdgiIndirectIntensity;
  bool enable_probe_relocation = kComparisonEnableProbeRelocation;
  bool enable_probe_classification = kComparisonEnableProbeClassification;
  std::filesystem::path screenshot_path;
};

[[nodiscard]] float ParseFloatArgument(const int argc, char** argv, int& arg_index, const std::string& argument) {
  if (arg_index + 1 >= argc) {
    throw std::invalid_argument(argument + " requires a value.");
  }
  return std::stof(argv[++arg_index]);
}

[[nodiscard]] size_t ParseSizeArgument(const int argc, char** argv, int& arg_index, const std::string& argument) {
  if (arg_index + 1 >= argc) {
    throw std::invalid_argument(argument + " requires a value.");
  }
  const auto value = std::stoull(argv[++arg_index]);
  return static_cast<size_t>(value);
}

[[nodiscard]] DdgiAppCommandLine ParseCommandLine(const int argc, char** argv) {
  DdgiAppCommandLine command_line;
  for (int arg_index = 1; arg_index < argc; ++arg_index) {
    if (ConsumeApplicationModeArgument(argc, argv, arg_index, command_line.application_mode)) {
      continue;
    }
    const std::string argument = argv[arg_index] ? argv[arg_index] : "";
    if (argument == "--exit-after-frames") {
      command_line.exit_after_frames = ParseSizeArgument(argc, argv, arg_index, argument);
    } else if (argument == "--max-load-frames") {
      command_line.max_load_frames = ParseSizeArgument(argc, argv, arg_index, argument);
    } else if (argument == "--screenshot-warmup-frames") {
      command_line.screenshot_warmup_frames = ParseSizeArgument(argc, argv, arg_index, argument);
    } else if (argument == "--point-light-brightness") {
      command_line.point_light_brightness = ParseFloatArgument(argc, argv, arg_index, argument);
    } else if (argument == "--ddgi-indirect-intensity") {
      command_line.ddgi_indirect_intensity = ParseFloatArgument(argc, argv, arg_index, argument);
    } else if (argument == "--enable-probe-relocation") {
      command_line.enable_probe_relocation = true;
    } else if (argument == "--disable-probe-relocation") {
      command_line.enable_probe_relocation = false;
    } else if (argument == "--enable-probe-classification") {
      command_line.enable_probe_classification = true;
    } else if (argument == "--disable-probe-classification") {
      command_line.enable_probe_classification = false;
    } else if (argument == "--screenshot") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument(argument + " requires a path.");
      }
      command_line.screenshot_path = std::filesystem::absolute(argv[++arg_index]);
    } else {
      throw std::invalid_argument("Unknown DDGIApp argument: " + argument);
    }
  }
  if (command_line.application_mode != ApplicationMode::Player) {
    throw std::invalid_argument("DDGIApp only supports player mode.");
  }
  return command_line;
}

int FailDdgiApp(const std::string& reason) {
  std::cerr << "DDGI_APP_RESULT failed reason=\"" << reason << "\"" << std::endl;
  return 1;
}

void DisablePostProcessing(const std::shared_ptr<Scene>& scene) {
  if (const auto main_camera = scene->main_camera.Get<Camera>()) {
    main_camera->post_processing_stack_ref.Clear();
  }
  if (const auto* camera_owners = scene->UnsafeGetPrivateComponentOwnersList<Camera>()) {
    for (const auto& owner : *camera_owners) {
      if (const auto camera = scene->GetOrSetPrivateComponent<Camera>(owner).lock()) {
        camera->post_processing_stack_ref.Clear();
      }
    }
  }
}

void ConfigureDdgiAppScene(const std::shared_ptr<Scene>& scene, const DdgiAppCommandLine& command_line) {
  if (!scene) {
    return;
  }
  DisablePostProcessing(scene);

  scene->environment.environment_type = Scene::EnvironmentType::Color;
  scene->environment.background_color = glm::vec3(0.0f);
  scene->environment.background_intensity = 0.0f;
  scene->environment.ambient_light_intensity = 0.0f;

  auto& ddgi_settings = scene->environment.ddgi_settings;
  ddgi_settings.runtime.enabled = true;
  ddgi_settings.runtime.pause_updates = false;
  ddgi_settings.runtime.ray_count = 256;
  ddgi_settings.runtime.normal_bias = kComparisonDdgiNormalBias;
  ddgi_settings.runtime.view_bias = kComparisonDdgiViewBias;
  ddgi_settings.runtime.reset_probe_history = true;
  ddgi_settings.runtime.indirect_intensity = glm::max(command_line.ddgi_indirect_intensity, 0.0f);
  ddgi_settings.storage.max_probe_count =
      kComparisonProbeCounts.x * kComparisonProbeCounts.y * kComparisonProbeCounts.z;
  ddgi_settings.debug.enabled = false;
  ddgi_settings.debug.visualize_volume_bounds = false;
  ddgi_settings.debug.visualize_probe_positions = false;
  ddgi_settings.debug.visualize_selected_probe = false;
  ddgi_settings.debug.visualize_probe_state = false;
  ddgi_settings.debug.visualize_probe_illumination = false;
  ddgi_settings.debug.show_atlas_preview = false;
  ddgi_settings.debug.show_update_age = false;
  ddgi_settings.debug.show_rays = false;
  ddgi_settings.debug.show_irradiance = false;
  ddgi_settings.debug.show_visibility = false;
  ddgi_settings.debug.show_sampling_weights = false;

  if (const auto main_camera = scene->main_camera.Get<Camera>()) {
    main_camera->Resize(kComparisonExtent);
    main_camera->skybox.Clear();
    main_camera->camera_settings.use_clear_color = true;
    main_camera->camera_settings.clear_color = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
    main_camera->camera_settings.background_intensity = 0.0f;

    auto camera_transform = scene->GetDataComponent<Transform>(main_camera->GetOwner());
    camera_transform.SetPosition(kCornellCameraPosition);
    scene->SetDataComponent(main_camera->GetOwner(), camera_transform);
  }

  if (const auto* volume_owners = scene->UnsafeGetPrivateComponentOwnersList<DdgiVolume>()) {
    for (const auto& owner : *volume_owners) {
      if (const auto volume = scene->GetOrSetPrivateComponent<DdgiVolume>(owner).lock()) {
        volume->visualize_bounds = false;
        volume->visualize_probe_positions = false;
        volume->probe_counts = kComparisonProbeCounts;
        volume->probe_spacing = glm::vec3(kComparisonProbeSpacing);
        volume->volume_origin = kComparisonVolumeOrigin;
        volume->relocation_distance = kComparisonProbeSpacing * 0.5f;
        volume->enable_probe_relocation = command_line.enable_probe_relocation;
        volume->enable_probe_classification = command_line.enable_probe_classification;
        volume->max_visualized_probes = 0;
        volume->ClampSettings();
      }
    }
  }

  if (const auto* point_light_owners = scene->UnsafeGetPrivateComponentOwnersList<PointLight>()) {
    for (const auto& owner : *point_light_owners) {
      if (const auto point_light = scene->GetOrSetPrivateComponent<PointLight>(owner).lock()) {
        point_light->diffuse_brightness = glm::max(command_line.point_light_brightness, 0.0f);
      }
    }
  }

  if (const auto* mesh_renderer_owners = scene->UnsafeGetPrivateComponentOwnersList<MeshRenderer>()) {
    for (const auto& owner : *mesh_renderer_owners) {
      if (scene->GetEntityName(owner) != "Ceiling Light Mesh") {
        continue;
      }
      const auto mesh_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(owner).lock();
      const auto material = mesh_renderer ? mesh_renderer->material.Get<Material>() : nullptr;
      if (material) {
        material->material_properties.emission = kComparisonCeilingLightEmission;
        material->MarkDirty();
      }
    }
  }
}

[[nodiscard]] int WaitForProjectIdle(Application& application, const size_t max_load_frames) {
  size_t load_frame_count = 0;
  while (!ProjectManager::IsProjectIdle()) {
    if (!application.Loop()) {
      return FailDdgiApp("application ended before Cornell box project load completed");
    }
    ++load_frame_count;
    if (load_frame_count >= max_load_frames) {
      return FailDdgiApp("Cornell box project load timed out");
    }
  }
  return 0;
}

[[nodiscard]] int LoopFrames(Application& application, const size_t frame_count, const std::string& failure_reason) {
  for (size_t frame_index = 0; frame_index < frame_count; ++frame_index) {
    if (!application.Loop()) {
      return FailDdgiApp(failure_reason);
    }
  }
  return 0;
}

[[nodiscard]] int CaptureWindowScreenshot(Application& application, const DdgiAppCommandLine& command_line) {
  if (const auto warmup_result = LoopFrames(application, command_line.screenshot_warmup_frames,
                                            "application ended before DDGIApp screenshot warmup completed");
      warmup_result != 0) {
    return warmup_result;
  }

  const auto window_layer = application.GetLayer<WindowLayer>();
  if (!window_layer) {
    return FailDdgiApp("window layer is missing for DDGIApp screenshot capture");
  }
  window_layer->RequestScreenshot(command_line.screenshot_path);
  if (!application.Loop()) {
    return FailDdgiApp("application ended before DDGIApp screenshot copy completed");
  }
  std::string screenshot_error;
  if (!window_layer->StoreCompletedScreenshot(screenshot_error)) {
    return FailDdgiApp(screenshot_error);
  }
  std::cout << "DDGI_APP_SCREENSHOT_CAPTURED screenshot_file=\"" << command_line.screenshot_path.string() << "\""
            << std::endl;
  return 0;
}
}  // namespace

int main(const int argc, char** argv) {
  Application application;
  bool initialized = false;
  try {
    const auto command_line = ParseCommandLine(argc, argv);

    PushStandardApplicationLayers(command_line.application_mode);
#ifdef PHYSX_PHYSICS_SERVICE
    ApplicationContext::Get().PushLayer<PhysicsLayer>();
#endif

    ApplicationInitializationSettings application_info;
    SetupDemoScene(DemoSetup::CornellBox, application_info);
    application_info.application_mode = command_line.application_mode;
    application_info.application_name = "DDGI Cornell Box";
    application_info.default_window_size = kComparisonExtent;
    application_info.use_custom_title_bar = false;
    ApplyApplicationModeDefaults(application_info);

    ApplicationContext::Get().Initialize(application_info);
    initialized = true;
    ApplicationContext::Get().Start(false);
    if (const auto load_result = WaitForProjectIdle(ApplicationContext::Get(), command_line.max_load_frames);
        load_result != 0) {
      ApplicationContext::Get().Terminate();
      return load_result;
    }

    ConfigureDdgiAppScene(ApplicationContext::Get().GetActiveScene(), command_line);
    ApplicationContext::Get().Play();

    if (!command_line.screenshot_path.empty()) {
      const auto screenshot_result = CaptureWindowScreenshot(ApplicationContext::Get(), command_line);
      ApplicationContext::Get().End();
      ApplicationContext::Get().Terminate();
      if (screenshot_result == 0) {
        std::cout << "DDGI_APP_RESULT passed" << std::endl;
      }
      return screenshot_result;
    }

    if (command_line.exit_after_frames != 0) {
      if (const auto smoke_result = LoopFrames(ApplicationContext::Get(), command_line.exit_after_frames,
                                               "application ended before DDGIApp smoke frames completed");
          smoke_result != 0) {
        ApplicationContext::Get().Terminate();
        return smoke_result;
      }
      ApplicationContext::Get().End();
      ApplicationContext::Get().Terminate();
      std::cout << "DDGI_APP_RESULT passed" << std::endl;
      return 0;
    }

    ApplicationContext::Get().Run();
    ApplicationContext::Get().Terminate();
    return 0;
  } catch (const std::exception& e) {
    if (initialized) {
      ApplicationContext::Get().Terminate();
    }
    return FailDdgiApp(e.what());
  }
}
