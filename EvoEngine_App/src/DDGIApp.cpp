#include "AppBootstrap.hpp"
#include "Application.hpp"
#include "Camera.hpp"
#include "DemoProfiles.hpp"
#include "DemoScene.hpp"
#include "ProjectManager.hpp"
#include "RenderLayer.hpp"
#include "Scene.hpp"
#include "WindowLayer.hpp"

#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>

#ifdef PHYSX_PHYSICS_SERVICE
#  include "PhysicsLayer.hpp"
#endif

using namespace evo_engine;

namespace {
struct DdgiAppCommandLine {
  ApplicationMode application_mode = ApplicationMode::Editor;
  size_t exit_after_frames = 0;
  size_t max_load_frames = 600;
  size_t screenshot_warmup_frames = 360;
  uint32_t width = 1024;
  uint32_t height = 1024;
  std::optional<GraphicsInitializationSettings::ShadowMapResolutionQuality> shadow_map_resolution_quality;
  DdgiCornellBoxDemoSettings scene_settings;
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

[[nodiscard]] uint32_t ParseDimensionArgument(const int argc, char** argv, int& arg_index,
                                              const std::string& argument) {
  const auto value = ParseSizeArgument(argc, argv, arg_index, argument);
  if (value == 0 || value > static_cast<size_t>(std::numeric_limits<int>::max())) {
    throw std::invalid_argument(argument + " must be a positive 32-bit signed integer.");
  }
  return static_cast<uint32_t>(value);
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
    } else if (argument == "--width") {
      command_line.width = ParseDimensionArgument(argc, argv, arg_index, argument);
    } else if (argument == "--height") {
      command_line.height = ParseDimensionArgument(argc, argv, arg_index, argument);
    } else if (argument == "--shadow-map-resolution" || argument == "--shadow-resolution") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument(argument + " requires low, medium, high, or very-high.");
      }
      command_line.shadow_map_resolution_quality =
          ParseShadowMapResolutionQualityName(argv[++arg_index] ? argv[arg_index] : "");
    } else if (argument == "--point-light-brightness") {
      command_line.scene_settings.point_light_brightness = ParseFloatArgument(argc, argv, arg_index, argument);
    } else if (argument == "--indirect-lighting-intensity") {
      command_line.scene_settings.indirect_lighting_intensity = ParseFloatArgument(argc, argv, arg_index, argument);
    } else if (argument == "--enable-probe-relocation") {
      command_line.scene_settings.enable_probe_relocation = true;
    } else if (argument == "--disable-probe-relocation") {
      command_line.scene_settings.enable_probe_relocation = false;
    } else if (argument == "--enable-probe-classification") {
      command_line.scene_settings.enable_probe_classification = true;
    } else if (argument == "--disable-probe-classification") {
      command_line.scene_settings.enable_probe_classification = false;
    } else if (argument == "--screenshot") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument(argument + " requires a path.");
      }
      command_line.screenshot_path = std::filesystem::absolute(argv[++arg_index]);
    } else {
      throw std::invalid_argument("Unknown DDGIApp argument: " + argument);
    }
  }
  if (command_line.application_mode == ApplicationMode::Headless) {
    throw std::invalid_argument("DDGIApp does not support headless mode.");
  }
  return command_line;
}

int FailDdgiApp(const std::string& reason) {
  std::cerr << "DDGI_APP_RESULT failed reason=\"" << reason << "\"" << std::endl;
  return 1;
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
  uint32_t observed_probe_update_count = 0;
  uint32_t observed_ray_sample_count = 0;
  for (size_t frame_index = 0; frame_index < command_line.screenshot_warmup_frames; ++frame_index) {
    if (!application.Loop()) {
      return FailDdgiApp("application ended before DDGIApp screenshot warmup completed");
    }
    if (const auto render_layer = application.GetLayer<RenderLayer>()) {
      const auto performance = render_layer->GetDdgiInspectorSnapshot().aggregate;
      observed_probe_update_count = glm::max(observed_probe_update_count, performance.recorded_probe_update_count);
      observed_ray_sample_count = glm::max(observed_ray_sample_count, performance.recorded_ray_sample_count);
    }
  }

  const auto render_layer = application.GetLayer<RenderLayer>();
  const auto scene = ApplicationContext::Get().GetActiveScene();
  const auto main_camera = scene ? scene->main_camera.Get<Camera>() : nullptr;
  if (!render_layer || !main_camera) {
    return FailDdgiApp("DDGI renderer or main camera is missing after screenshot warmup");
  }
  const auto resolution = main_camera->GetSize();
  if (resolution.x != command_line.width || resolution.y != command_line.height || !main_camera->Rendered()) {
    return FailDdgiApp("main camera did not render at the requested screenshot resolution");
  }
  const auto performance = render_layer->GetDdgiInspectorSnapshot().aggregate;
  if (performance.active_probe_count == 0 || performance.storage_probe_count < performance.active_probe_count ||
      observed_probe_update_count == 0 || observed_ray_sample_count == 0 || !performance.lighting_descriptors_bound) {
    return FailDdgiApp("DDGI did not reach a render-ready state before screenshot capture");
  }
  std::cout << "DDGI_APP_DDGI_READY resolution=" << resolution.x << "x" << resolution.y
            << " active_probes=" << performance.active_probe_count
            << " storage_probes=" << performance.storage_probe_count
            << " recorded_updated_probes=" << observed_probe_update_count
            << " recorded_ray_samples=" << observed_ray_sample_count
            << " lighting_descriptors_bound=" << performance.lighting_descriptors_bound << std::endl;

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
    ConfigureDdgiCornellBoxApplication(application_info, command_line.application_mode);
    application_info.default_window_size = {static_cast<int>(command_line.width),
                                            static_cast<int>(command_line.height)};
    ApplyApplicationModeDefaults(application_info);
    if (command_line.shadow_map_resolution_quality) {
      application_info.graphics_settings.SetShadowMapResolutionQuality(*command_line.shadow_map_resolution_quality);
    }

    ApplicationContext::Get().Initialize(application_info);
    initialized = true;
    ApplicationContext::Get().Start(false);
    if (const auto load_result = WaitForProjectIdle(ApplicationContext::Get(), command_line.max_load_frames);
        load_result != 0) {
      ApplicationContext::Get().Terminate();
      return load_result;
    }

    {
      const auto scene = ApplicationContext::Get().GetActiveScene();
      ConfigureDdgiCornellBoxScene(scene, command_line.scene_settings);
      if (const auto main_camera = scene ? scene->main_camera.Get<Camera>() : nullptr) {
        main_camera->Resize({command_line.width, command_line.height});
      }
    }
    if (command_line.application_mode == ApplicationMode::Player) {
      ApplicationContext::Get().Play();
    }

    if (!command_line.screenshot_path.empty()) {
      const auto screenshot_result = CaptureWindowScreenshot(ApplicationContext::Get(), command_line);
      ApplicationContext::Get().End();
      ApplicationContext::Get().Terminate();
      std::cout << "DDGI_APP_SHUTDOWN_COMPLETE" << std::endl;
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
      std::cout << "DDGI_APP_SHUTDOWN_COMPLETE" << std::endl;
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
