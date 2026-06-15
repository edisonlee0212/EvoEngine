#include "Application.hpp"
#include "DemoScene.hpp"
#include "EditorLayer.hpp"
#include "ImGuiLayer.hpp"
#include "Platform.hpp"
#include "ProjectManager.hpp"
#include "RenderLayer.hpp"
#include "WindowLayer.hpp"

#include <cstdlib>

#ifdef PHYSX_PHYSICS_SERVICE
#  include "PhysicsLayer.hpp"
#endif

using namespace evo_engine;

namespace {
enum class DemoAppRunMode { Normal, SmokeTest };

struct DemoAppRuntimeConfig {
  DemoAppRunMode mode = DemoAppRunMode::Normal;
  DemoSetup demo_setup = DemoSetup::Rendering;
  size_t frames_after_play = 100;
  size_t max_load_frames = 30000;
  size_t max_play_frames = 1000;
  bool exit_on_complete = true;
};

std::optional<std::filesystem::path> FindRunConfigPath(const int argc, char** argv) {
  for (int arg_index = 1; arg_index < argc; ++arg_index) {
    const std::string argument = argv[arg_index] ? argv[arg_index] : "";
    if (argument == "--run-config") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument("--run-config requires a path.");
      }
      return std::filesystem::absolute(argv[arg_index + 1]);
    }
    throw std::invalid_argument("Unknown DemoApp argument: " + argument);
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
  if (value == "Universe") {
    return DemoSetup::Universe;
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
    } else {
      throw std::invalid_argument("Unknown mode value: " + mode_name);
    }
  }
  if (const auto demo_setup = root["demo_setup"]) {
    config.demo_setup = ParseDemoSetup(demo_setup.as<std::string>());
  }
  if (const auto frames_after_play = root["frames_after_play"]) {
    config.frames_after_play = frames_after_play.as<size_t>();
  }
  if (const auto max_load_frames = root["max_load_frames"]) {
    config.max_load_frames = max_load_frames.as<size_t>();
  }
  if (const auto max_play_frames = root["max_play_frames"]) {
    config.max_play_frames = max_play_frames.as<size_t>();
  }
  if (const auto exit_on_complete = root["exit_on_complete"]) {
    config.exit_on_complete = exit_on_complete.as<bool>();
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

int RunSmokeTest(const DemoAppRuntimeConfig& config) {
  auto& application = ApplicationContext::Get();
  application.Start(false);

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

  const auto loaded_frame = Platform::GetFrameCount();
  application.Play();
  if (!application.IsPlaying()) {
    return FailSmokeTest(application, "application did not enter play mode");
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
}  // namespace

int main(const int argc, char** argv) {
  Application application;
  bool initialized = false;
  bool smoke_test = false;
  try {
    const auto run_config_path = FindRunConfigPath(argc, argv);
    std::optional<DemoAppRuntimeConfig> runtime_config;
    if (run_config_path) {
      runtime_config = LoadRunConfig(*run_config_path);
    }
    const auto demo_setup = runtime_config ? runtime_config->demo_setup : DemoSetup::Rendering;

    ApplicationContext::Get().PushLayer<RenderLayer>("Render Layer");
    ApplicationContext::Get().PushLayer<WindowLayer>("Window Layer");
    ApplicationContext::Get().PushLayer<ImGuiLayer>("ImGui Layer");
    ApplicationContext::Get().PushLayer<EditorLayer>("Editor Layer");
#ifdef PHYSX_PHYSICS_SERVICE
    ApplicationContext::Get().PushLayer<PhysicsLayer>();
#endif

    ApplicationInitializationSettings application_info;
    SetupDemoScene(demo_setup, application_info);
    application_info.use_custom_title_bar = true;

    ApplicationContext::Get().Initialize(application_info);
    initialized = true;

    int exit_code = 0;
    smoke_test = runtime_config && runtime_config->mode == DemoAppRunMode::SmokeTest;
    if (smoke_test) {
      exit_code = RunSmokeTest(*runtime_config);
    } else {
      ApplicationContext::Get().Start();
      ApplicationContext::Get().Run();
    }
    ApplicationContext::Get().Terminate();
    if (smoke_test) {
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
    if (smoke_test) {
      std::cout.flush();
      std::cerr.flush();
      std::_Exit(1);
    }
    return 1;
  }
}
