#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>
#include "EvoEngineAPI.hpp"
#include "EvoEngineEditorAPI.hpp"
#include "ProjectBuildSettings.hpp"

namespace evo_engine {
struct EVOENGINE_EDITOR_API BuildPreflightSnapshot {
  bool project_loaded = false;
  bool project_idle = false;
  bool editor_stopped = false;
  bool build_settings_saved = false;
  bool startup_scene_saved = false;
  bool startup_scene_loadable = false;
  bool main_camera_present = false;
  bool main_camera_enabled = false;
  bool main_camera_owner_enabled = false;
  bool template_compatible = false;
  bool destination_available = false;
  std::vector<std::string> unsaved_project_assets;
  std::vector<std::string> content_errors;
};
struct EVOENGINE_EDITOR_API BuildPreflightResult {
  std::vector<std::string> errors;
  [[nodiscard]] bool Passed() const {
    return errors.empty();
  }
};
[[nodiscard]] EVOENGINE_EDITOR_API BuildPreflightResult ValidateBuildPreflight(const ProjectBuildSettings& settings,
                                                                               const BuildPreflightSnapshot& snapshot);
}  // namespace evo_engine
