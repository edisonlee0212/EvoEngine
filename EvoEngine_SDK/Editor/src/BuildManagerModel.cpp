#include "BuildManagerModel.hpp"
#include <algorithm>
namespace evo_engine {
BuildPreflightResult ValidateBuildPreflight(const ProjectBuildSettings& settings,
                                            const BuildPreflightSnapshot& snapshot) {
  BuildPreflightResult result;
  auto fail = [&](std::string message) {
    result.errors.emplace_back(std::move(message));
  };
  if (!snapshot.project_loaded)
    fail("Open a fully loaded project before building.");
  if (!snapshot.project_idle)
    fail("Wait for project scanning and asset loading to finish.");
  if (!snapshot.editor_stopped)
    fail("Stop scene playback; paused and playing states cannot be exported.");
  if (!snapshot.build_settings_saved)
    fail("Could not save project and build settings. Check that the project file is writable.");
  if (settings.application_name.empty())
    fail("Enter an application name.");
  else if (settings.application_name.find_first_of("<>:\"/\\|?*") != std::string::npos ||
           settings.application_name.back() == ' ' || settings.application_name.back() == '.')
    fail("Application name is not a valid Windows filename.");
  if (settings.output_directory.empty() || !settings.output_directory.is_absolute())
    fail("Choose an absolute output directory.");
  if (settings.startup_scene_handle == 0)
    fail("Choose a saved startup scene for the build.");
  if (settings.window_width <= 0 || settings.window_height <= 0)
    fail("Window dimensions must be positive.");
  if (!snapshot.startup_scene_saved)
    fail("The selected startup scene must be saved in the project.");
  if (!snapshot.startup_scene_loadable)
    fail("The selected startup scene could not be loaded.");
  if (!snapshot.main_camera_present)
    fail("The selected startup scene has no main camera.");
  else if (!snapshot.main_camera_enabled)
    fail("The selected startup scene main camera is disabled.");
  else if (!snapshot.main_camera_owner_enabled)
    fail("The selected startup scene main camera owner is disabled.");
  if (!snapshot.template_compatible)
    fail("Install a runtime template matching this editor build and configuration.");
  if (!snapshot.destination_available)
    fail("The output must be absent or an empty directory outside the project.");
  for (const auto& asset : snapshot.unsaved_project_assets)
    fail("Save project asset: " + asset);
  result.errors.insert(result.errors.end(), snapshot.content_errors.begin(), snapshot.content_errors.end());
  return result;
}
}  // namespace evo_engine
