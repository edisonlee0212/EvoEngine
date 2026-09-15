#pragma once

#include <filesystem>
#include <glm/glm.hpp>
#include <string>
#include <vector>
#include "ApplicationInitializationSettings.hpp"
#include "EvoEngineAPI.hpp"

namespace evo_engine {
struct RuntimePackageRequirement {
  std::string name;
  std::string source_id;
};

struct EVOENGINE_API RuntimeConfiguration {
  std::string application_name;
  std::filesystem::path project;
  glm::ivec2 window_size{1280, 720};
  WindowDisplayMode window_mode = WindowDisplayMode::Windowed;
  bool allow_window_resize = true;
  bool allow_resolution_change = true;
  bool show_console = false;
  GraphicsInitializationSettings graphics;
  std::vector<RuntimePackageRequirement> packages;

  static RuntimeConfiguration Load(const std::filesystem::path& path);
  ApplicationInitializationSettings ApplicationSettings() const;
  void ValidateLoadedPackages() const;
};
}  // namespace evo_engine
