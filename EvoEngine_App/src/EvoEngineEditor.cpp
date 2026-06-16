#include "Application.hpp"
#include "EditorLayer.hpp"
#include "ImGuiLayer.hpp"
#include "PathUtils.hpp"
#include "ProjectManager.hpp"
#include "RenderLayer.hpp"
#include "WindowLayer.hpp"

#include <optional>
#include <stdexcept>

#ifdef EVOENGINE_WINDOWS
#  ifndef NOMINMAX
#    define NOMINMAX
#  endif
#  include <Windows.h>
#endif

using namespace evo_engine;

namespace {
std::optional<std::filesystem::path> ParseProjectPath(const int argc, char** argv) {
  std::optional<std::filesystem::path> project_path;
  for (int arg_index = 1; arg_index < argc; ++arg_index) {
    const std::string argument = argv[arg_index] ? argv[arg_index] : "";
    if (argument == "--project" || argument == "-p") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument(argument + " requires a project path.");
      }
      project_path = std::filesystem::absolute(argv[++arg_index]);
    } else if (!project_path) {
      project_path = std::filesystem::absolute(argument);
    } else {
      throw std::invalid_argument("Unknown EvoEngineEditor argument: " + argument);
    }
  }
  return project_path;
}

std::filesystem::path CurrentExecutablePath() {
#ifdef EVOENGINE_WINDOWS
  return path_utils::CurrentExecutablePath("EvoEngineEditor.exe");
#else
  return path_utils::CurrentExecutablePath("EvoEngineEditor");
#endif
}

std::filesystem::path LauncherExecutablePath() {
#ifdef EVOENGINE_WINDOWS
  return CurrentExecutablePath().parent_path() / "EvoEngineLauncher.exe";
#else
  return CurrentExecutablePath().parent_path() / "EvoEngineLauncher";
#endif
}

bool LaunchLauncherProcess(std::string& error) {
  const auto launcher_path = LauncherExecutablePath();
  if (!std::filesystem::exists(launcher_path)) {
    error = "Could not find EvoEngineLauncher next to EvoEngineEditor.";
    return false;
  }

#ifdef EVOENGINE_WINDOWS
  std::wstring command_line = L"\"" + launcher_path.wstring() + L"\"";
  STARTUPINFOW startup_info{};
  startup_info.cb = sizeof(startup_info);
  PROCESS_INFORMATION process_info{};
  const auto working_directory = launcher_path.parent_path().wstring();
  if (!CreateProcessW(nullptr, command_line.data(), nullptr, nullptr, FALSE, 0, nullptr, working_directory.c_str(),
                      &startup_info, &process_info)) {
    error = "Failed to launch EvoEngineLauncher.";
    return false;
  }
  CloseHandle(process_info.hProcess);
  CloseHandle(process_info.hThread);
  return true;
#else
  const auto command = "\"" + launcher_path.string() + "\" &";
  if (std::system(command.c_str()) != 0) {
    error = "Failed to launch EvoEngineLauncher.";
    return false;
  }
  return true;
#endif
}
}  // namespace

int main(const int argc, char** argv) {
  Application application;
  bool initialized = false;
  try {
    const auto project_path = ParseProjectPath(argc, argv);
    if (!project_path) {
      std::string error;
      if (!LaunchLauncherProcess(error)) {
        EVOENGINE_ERROR(error)
        return 1;
      }
      return 0;
    }
    if (project_path->extension() != ".eveproj") {
      EVOENGINE_ERROR("EvoEngineEditor requires --project <path-to-.eveproj>.")
      return 1;
    }

    ApplicationContext::Get().PushLayer<RenderLayer>("Render Layer");
    ApplicationContext::Get().PushLayer<WindowLayer>("Window Layer");
    ApplicationContext::Get().PushLayer<ImGuiLayer>("ImGui Layer");
    ApplicationContext::Get().PushLayer<EditorLayer>("Editor Layer");

    ApplicationInitializationSettings application_info{};
    const auto launch_metadata = ProjectManager::LoadProjectLaunchMetadata(*project_path);
    application_info.application_name = launch_metadata.application_name;
    application_info.project_path = *project_path;
    application_info.use_custom_title_bar = true;
    application_info.startup_runtime_packages = launch_metadata.startup_runtime_packages;
    application_info.enable_runtime_packages = !application_info.startup_runtime_packages.empty();
    ApplicationContext::Get().Initialize(application_info);
    initialized = true;

    ApplicationContext::Get().Start();
    ApplicationContext::Get().Run();
    ApplicationContext::Get().Terminate();
    return 0;
  } catch (const std::exception& error) {
    EVOENGINE_ERROR(error.what())
    if (initialized) {
      ApplicationContext::Get().Terminate();
    }
    return 1;
  }
}
