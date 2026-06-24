#include "AppBootstrap.hpp"
#include "Application.hpp"
#include "PathUtils.hpp"

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <optional>
#include <stdexcept>

using namespace evo_engine;

namespace {
struct RealtimePlantReconstructorCommandLine {
  std::optional<std::filesystem::path> run_config_path;
  std::optional<ApplicationMode> application_mode;
};

std::filesystem::path FindResourceFolder() {
  if (const auto resource_folder_path =
          path_utils::FindAncestorChildPath("Resources", std::filesystem::current_path(), 8);
      !resource_folder_path.empty()) {
    return resource_folder_path;
  }
  return path_utils::NormalizeAbsolutePath("Resources");
}

EditorLayoutSettings CreatePlantReconstructionEditorLayout() {
  EditorLayoutSettings settings;
  settings.panels.scene = true;
  settings.panels.camera = false;
  settings.panels.scene_camera_debug = false;
  settings.panels.scene_info = true;
  settings.panels.camera_info = false;
  settings.panels.entity_explorer = true;
  settings.panels.entity_inspector = true;
  settings.panels.console = true;
  settings.panels.project = true;
  settings.panels.resources = false;
  settings.panels.profiler = false;
  settings.panels.runtime_package_manager = false;

  EditorDockLayoutSettings dock_layout;
  dock_layout.left_fraction = 0.18f;
  dock_layout.right_fraction = 0.28f;
  dock_layout.bottom_fraction = 0.24f;
  settings.dock_layout = dock_layout;
  return settings;
}

RealtimePlantReconstructorCommandLine ParseCommandLine(const int argc, char** argv) {
  RealtimePlantReconstructorCommandLine command_line;
  for (int arg_index = 1; arg_index < argc; ++arg_index) {
    const std::string argument = argv[arg_index] ? argv[arg_index] : "";
    if (argument == "--run-config") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument("--run-config requires a path.");
      }
      command_line.run_config_path = std::filesystem::absolute(argv[++arg_index]);
    } else {
      auto mode = command_line.application_mode.value_or(ApplicationMode::Editor);
      if (!ConsumeApplicationModeArgument(argc, argv, arg_index, mode)) {
        throw std::invalid_argument("Unknown RealtimePlantReconstructorApp argument: " + argument);
      }
      command_line.application_mode = mode;
    }
  }
  return command_line;
}

glm::ivec2 ReadRunConfigWindowSize(const YAML::Node& run_config) {
  glm::ivec2 size(1920, 1080);
  if (run_config["width"] && run_config["width"].IsScalar()) {
    size.x = std::max(1, run_config["width"].as<int>());
  }
  if (run_config["height"] && run_config["height"].IsScalar()) {
    size.y = std::max(1, run_config["height"].as<int>());
  }
  return size;
}

int RunLayerAutomation(const YAML::Node& run_config) {
  if (!run_config || !run_config.IsMap() || !run_config["mode"] || !run_config["mode"].IsScalar()) {
    std::cerr << "RPR_VIDEO_RESULT failed reason=\"run config requires scalar mode\"" << std::endl;
    return 1;
  }
  const auto mode = run_config["mode"].as<std::string>();
  for (const auto& layer : ApplicationContext::Get().GetLayers()) {
    if (layer && layer->SupportsLayerAutomationMode(mode)) {
      return layer->RunLayerAutomation(run_config);
    }
  }
  std::cerr << "RPR_VIDEO_RESULT failed reason=\"no layer supports mode: " << mode << "\"" << std::endl;
  return 1;
}
}  // namespace

int main(const int argc, char** argv) {
  Application application;
  bool initialized = false;
  bool automated_run = false;
  try {
    const auto command_line = ParseCommandLine(argc, argv);
    automated_run = command_line.run_config_path.has_value();
    const YAML::Node run_config = automated_run ? YAML::LoadFile(command_line.run_config_path->string()) : YAML::Node();
    const auto application_mode =
        automated_run ? ApplicationMode::Editor : command_line.application_mode.value_or(ApplicationMode::Editor);
    const auto resource_folder_path = FindResourceFolder();
    const auto project_path = std::filesystem::absolute(resource_folder_path / "RealtimePlantReconstructorProject" /
                                                        "RealtimePlantReconstructor.eveproj");

    PushStandardApplicationLayers(application_mode);

    ApplicationInitializationSettings application_configs;
    application_configs.application_mode = application_mode;
    application_configs.application_name = "RealtimePlantReconstructor";
    application_configs.project_path = project_path;
    application_configs.default_window_size =
        automated_run ? ReadRunConfigWindowSize(run_config) : glm::ivec2(1920, 1080);
    application_configs.enable_runtime_packages = true;
    application_configs.use_custom_title_bar = true;
    application_configs.load_project_assets = false;
    application_configs.startup_runtime_packages = {"RealtimePlantReconstructor"};
    ApplyApplicationModeDefaults(application_configs);
    ApplicationContext::Get().Initialize(application_configs);
    initialized = true;

    if (const auto editor_layer = ApplicationContext::Get().GetLayer<EditorLayer>()) {
      editor_layer->velocity = 2.0f;
      editor_layer->default_scene_camera_position = glm::vec3(0.0f, 1.0f, 5.0f);
      editor_layer->RequestEditorLayout(CreatePlantReconstructionEditorLayout());
    }

    int exit_code = 0;
    if (automated_run) {
      exit_code = RunLayerAutomation(run_config);
    } else {
      ApplicationContext::Get().Start();
      ApplicationContext::Get().Run();
    }
    ApplicationContext::Get().Terminate();
    if (automated_run) {
      std::cout.flush();
      std::cerr.flush();
      std::_Exit(exit_code);
    }
    return exit_code;
  } catch (const std::exception& error) {
    std::cerr << "RPR_VIDEO_RESULT failed reason=\"" << error.what() << "\"" << std::endl;
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
