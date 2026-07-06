#include "AppBootstrap.hpp"
#include "Application.hpp"
#include "DemoProfiles.hpp"
#include "DemoScene.hpp"
#include "ProjectManager.hpp"
#include "WindowLayer.hpp"

#include <cstdlib>
#include <filesystem>
#include <iostream>
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
    } else if (argument == "--shadow-map-resolution" || argument == "--shadow-resolution") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument(argument + " requires low, medium, high, or very-high.");
      }
      command_line.shadow_map_resolution_quality =
          ParseShadowMapResolutionQualityName(argv[++arg_index] ? argv[arg_index] : "");
    } else if (argument == "--point-light-brightness") {
      command_line.scene_settings.point_light_brightness = ParseFloatArgument(argc, argv, arg_index, argument);
    } else if (argument == "--ddgi-indirect-intensity") {
      command_line.scene_settings.ddgi_indirect_intensity = ParseFloatArgument(argc, argv, arg_index, argument);
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
    ConfigureDdgiCornellBoxApplication(application_info, command_line.application_mode);
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

    ConfigureDdgiCornellBoxScene(ApplicationContext::Get().GetActiveScene(), command_line.scene_settings);
    if (command_line.application_mode == ApplicationMode::Player) {
      ApplicationContext::Get().Play();
    }

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
