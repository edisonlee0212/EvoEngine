#include "Application.hpp"
#include "EditorLayer.hpp"
#include "ImGuiLayer.hpp"
#include "PackageManager.hpp"
#include "ProjectManager.hpp"
#include "RenderLayer.hpp"
#include "WindowLayer.hpp"

#include <cctype>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>

#if defined(_WIN32)
#  include <Windows.h>
#else
#  include <dlfcn.h>
#endif

using namespace evo_engine;

namespace {

void PrintUsage() {
  std::cout << "Usage: ScotsPineDataGeneratorApp [options]\n"
            << "\n"
            << "Options:\n"
            << "  --interactive           Launch the editor UI for Scots pine workflows.\n"
            << "  --synthetic-config <path>\n"
            << "                          Run the Scots pine synthetic renderer from a key=value config.\n"
            << "  --project <path>        Project file (.eveproj). Defaults to "
               "Resources/DigitalAgricultureProject/test.eveproj.\n"
            << "  --help                  Show this help.\n"
            << "\n"
            << "No args defaults to --interactive.\n";
}

std::string Trim(std::string value) {
  auto is_space = [](const unsigned char c) {
    return std::isspace(c) != 0;
  };
  while (!value.empty() && is_space(static_cast<unsigned char>(value.front()))) {
    value.erase(value.begin());
  }
  while (!value.empty() && is_space(static_cast<unsigned char>(value.back()))) {
    value.pop_back();
  }
  return value;
}

std::filesystem::path ReadProjectPathFromConfig(const std::filesystem::path& config_path) {
  std::ifstream in(config_path, std::ios::in);
  if (!in.is_open()) {
    return {};
  }

  std::string line;
  while (std::getline(in, line)) {
    line = Trim(line);
    if (line.empty() || line[0] == '#') {
      continue;
    }
    const auto separator_index = line.find('=');
    if (separator_index == std::string::npos) {
      continue;
    }
    const auto key = Trim(line.substr(0, separator_index));
    if (key != "project_path") {
      continue;
    }
    const auto value = Trim(line.substr(separator_index + 1));
    if (!value.empty()) {
      return std::filesystem::path(value);
    }
  }
  return {};
}

bool ParseArgs(const int argc, char** argv, bool& show_help, bool& interactive_mode,
               std::filesystem::path& project_path, std::filesystem::path& synthetic_config_path, std::string& error) {
  show_help = false;
  interactive_mode = argc <= 1;
  project_path.clear();
  synthetic_config_path.clear();

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];

    if (arg == "--help" || arg == "-h") {
      show_help = true;
      return true;
    }
    if (arg == "--interactive") {
      interactive_mode = true;
      continue;
    }

    const auto require_value = [&](const std::string& flag) -> std::optional<std::string> {
      if (i + 1 >= argc) {
        error = "Missing value for " + flag;
        return std::nullopt;
      }
      ++i;
      return std::string(argv[i]);
    };

    if (arg == "--synthetic-config") {
      const auto value = require_value(arg);
      if (!value) {
        return false;
      }
      synthetic_config_path = *value;
      continue;
    }
    if (arg == "--project") {
      const auto value = require_value(arg);
      if (!value) {
        return false;
      }
      project_path = *value;
      continue;
    }

    error = "Unknown argument: " + arg;
    return false;
  }

  if (!synthetic_config_path.empty() && interactive_mode) {
    error = "--interactive and --synthetic-config are mutually exclusive.";
    return false;
  }

  return true;
}

std::filesystem::path FindResourceFolder() {
  std::filesystem::path resource_folder_path("../../../../../Resources");
  if (!std::filesystem::exists(resource_folder_path)) {
    resource_folder_path = "../../../../Resources";
  }
  if (!std::filesystem::exists(resource_folder_path)) {
    resource_folder_path = "../../../Resources";
  }
  if (!std::filesystem::exists(resource_folder_path)) {
    resource_folder_path = "../../Resources";
  }
  if (!std::filesystem::exists(resource_folder_path)) {
    resource_folder_path = "../Resources";
  }
  return resource_folder_path;
}

std::filesystem::path ResolveDefaultScotsPineProjectPath(const std::filesystem::path& resource_folder_path) {
  return std::filesystem::absolute(resource_folder_path / "DigitalAgricultureProject" / "test.eveproj");
}

void* ResolvePackageSymbol(const std::string& package_name, const char* symbol_name) {
  if (package_name.empty() || !symbol_name || symbol_name[0] == '\0') {
    return nullptr;
  }

  if (!PackageManager::Load(package_name)) {
    return nullptr;
  }

  for (const auto& loaded_package : PackageManager::GetLoadedPackages()) {
    if (loaded_package.name != package_name) {
      continue;
    }

#if defined(_WIN32)
    HMODULE library_handle = GetModuleHandleW(loaded_package.loaded_path.wstring().c_str());
    if (!library_handle) {
      library_handle = LoadLibraryW(loaded_package.loaded_path.wstring().c_str());
      if (!library_handle) {
        return nullptr;
      }
    }
    return reinterpret_cast<void*>(GetProcAddress(library_handle, symbol_name));
#else
    void* library_handle = nullptr;
#  if defined(RTLD_NOLOAD)
    library_handle = dlopen(loaded_package.loaded_path.string().c_str(), RTLD_NOW | RTLD_NOLOAD);
#  endif
    if (!library_handle) {
      library_handle = dlopen(loaded_package.loaded_path.string().c_str(), RTLD_NOW);
      if (!library_handle) {
        return nullptr;
      }
    }
    return dlsym(library_handle, symbol_name);
#endif
  }

  return nullptr;
}

int RunInteractive(const std::filesystem::path& project_path) {
  Application application;
  bool initialized = false;
  try {
    ApplicationContext::Get().PushLayer<RenderLayer>("Render Layer");
    ApplicationContext::Get().PushLayer<WindowLayer>("Window Layer");
    ApplicationContext::Get().PushLayer<ImGuiLayer>("ImGui Layer");
    ApplicationContext::Get().PushLayer<EditorLayer>("Editor Layer");

    ApplicationInitializationSettings application_info{};
    application_info.application_name = "Scots Pine Data Generator";
    application_info.project_path = project_path;
    application_info.enable_runtime_packages = true;
    application_info.startup_runtime_packages = {"LSystem"};

    ApplicationContext::Get().Initialize(application_info);
    initialized = true;
    ApplicationContext::Get().Start();
    ApplicationContext::Get().Run();
    ApplicationContext::Get().Terminate();
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "ScotsPineDataGeneratorApp interactive mode failed: " << e.what() << "\n";
    if (initialized) {
      ApplicationContext::Get().Terminate();
    }
    return 1;
  }
}

int RunSynthetic(const std::filesystem::path& project_path, const std::filesystem::path& synthetic_config_path) {
  std::cout << "ScotsPineDataGeneratorApp synthetic configuration:\n"
            << "  project: " << project_path << "\n"
            << "  config: " << synthetic_config_path << "\n";

  Application application;
  bool initialized = false;
  try {
    ApplicationContext::Get().PushLayer<RenderLayer>("Render Layer");

    ApplicationInitializationSettings application_info{};
    application_info.application_name = "Scots Pine Synthetic Renderer";
    application_info.project_path = project_path;
    application_info.load_project_assets = false;
    application_info.load_project_start_scene = false;
    application_info.enable_runtime_packages = true;
    application_info.startup_runtime_packages = {"LSystem"};

    ApplicationContext::Get().Initialize(application_info);
    initialized = true;

    auto& app = ApplicationContext::Get();
    app.Start(false);

    using RunSyntheticFn = int (*)(const char*);
    const auto symbol = ResolvePackageSymbol("LSystem", "EvoEngineLSystemRunScotsPineSyntheticRender");
    if (!symbol) {
      throw std::runtime_error("Failed to locate EvoEngineLSystemRunScotsPineSyntheticRender in LSystem runtime package.");
    }

    const auto run_synthetic = reinterpret_cast<RunSyntheticFn>(symbol);
    const int exit_code = run_synthetic(synthetic_config_path.string().c_str());

    app.End();
    app.Terminate();
    return exit_code;
  } catch (const std::exception& e) {
    std::cerr << "ScotsPineDataGeneratorApp synthetic render failed: " << e.what() << "\n";
    if (initialized) {
      ApplicationContext::Get().Terminate();
    }
    return 1;
  }
}

}  // namespace

int main(const int argc, char** argv) {
  bool show_help = false;
  bool interactive_mode = false;
  std::filesystem::path project_path;
  std::filesystem::path synthetic_config_path;
  std::string parse_error;
  if (!ParseArgs(argc, argv, show_help, interactive_mode, project_path, synthetic_config_path, parse_error)) {
    std::cerr << parse_error << "\n\n";
    PrintUsage();
    return 1;
  }

  if (show_help) {
    PrintUsage();
    return 0;
  }

  if (!synthetic_config_path.empty()) {
    synthetic_config_path = std::filesystem::absolute(synthetic_config_path);
    if (!std::filesystem::exists(synthetic_config_path)) {
      std::cerr << "Synthetic config file not found: " << synthetic_config_path << "\n";
      return 1;
    }
    if (project_path.empty()) {
      project_path = ReadProjectPathFromConfig(synthetic_config_path);
    }
  }

  if (project_path.empty()) {
    project_path = ResolveDefaultScotsPineProjectPath(FindResourceFolder());
  } else {
    project_path = std::filesystem::absolute(project_path);
  }

  if (!std::filesystem::exists(project_path)) {
    std::cerr << "Project file not found: " << project_path << "\n";
    return 1;
  }

  if (!synthetic_config_path.empty()) {
    return RunSynthetic(project_path, synthetic_config_path);
  }
  return RunInteractive(project_path);
}
