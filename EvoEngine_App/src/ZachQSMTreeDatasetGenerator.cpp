#include <filesystem>
#include <iostream>
#include "Application.hpp"
#include "PackageManager.hpp"
#ifdef _WIN32
#  include <Windows.h>
#else
#  include <dlfcn.h>
#endif

using namespace evo_engine;

int main(int argc, char** argv) {
  if (argc != 3 || std::string(argv[1]) != "--config") {
    std::cout << "ZachQSMTreeDatasetGenerator --config <config.yaml>\n"
                 "Headless EcoSysLab growth and skeleton export.\n"
                 "For meshes, validation and figures: python ZachQSM/generate.py --help\n";
    return argc == 2 && std::string(argv[1]) == "--help" ? 0 : 2;
  }
  const auto config = std::filesystem::absolute(argv[2]).string();
  Application application;
  ApplicationContextScope context(application);
  try {
    ApplicationInitializationSettings settings;
    settings.application_name = "ZachQSMTreeDatasetGenerator";
    settings.application_mode = ApplicationMode::Headless;
    settings.allow_empty_project = true;
    settings.load_default_resources = false;
    settings.load_project_assets = false;
    settings.load_project_start_scene = false;
    settings.hide_console_window = true;
    settings.redirect_standard_streams_to_console = false;
    settings.enable_runtime_packages = true;
    settings.startup_runtime_packages = {"EcoSysLab"};
    settings.package_search_paths = {std::filesystem::absolute(argv[0]).parent_path() / "Packages"};
    application.Initialize(settings);
    using Generate = int (*)(const char*);
    Generate generate = nullptr;
    for (const auto& package : PackageManager::GetLoadedPackages()) {
      if (package.name != "EcoSysLab")
        continue;
#ifdef _WIN32
      auto module = GetModuleHandleW(package.loaded_path.c_str());
      generate = reinterpret_cast<Generate>(GetProcAddress(module, "EvoEngineGenerateZachQSMTreeDataset"));
#else
      auto module = dlopen(package.loaded_path.c_str(), RTLD_NOW | RTLD_NOLOAD);
      if (module) {
        generate = reinterpret_cast<Generate>(dlsym(module, "EvoEngineGenerateZachQSMTreeDataset"));
        dlclose(module);
      }
#endif
    }
    if (!generate)
      throw std::runtime_error("EcoSysLab dataset entry point was not loaded.");
    const int result = generate(config.c_str());
    application.Terminate();
    return result;
  } catch (const std::exception& error) {
    std::cerr << "Dataset generation failed: " << error.what() << std::endl;
    application.Terminate();
    return 1;
  }
}
