#include <gtest/gtest.h>
#include "BuildManagerModel.hpp"
using namespace evo_engine;
namespace {
BuildPreflightSnapshot Valid() {
  BuildPreflightSnapshot s;
  s.project_loaded = s.project_idle = s.editor_stopped = s.build_settings_saved = true;
  s.startup_scene_saved = s.startup_scene_loadable = true;
  s.main_camera_present = s.main_camera_enabled = s.main_camera_owner_enabled = true;
  s.template_compatible = s.destination_available = true;
  return s;
}
ProjectBuildSettings Settings() {
  ProjectBuildSettings s;
  s.application_name = "App";
  s.output_directory = "C:/Output";
  s.startup_scene_handle = 42;
  return s;
}
}  // namespace
TEST(BuildManagerPreflight, AcceptsIndependentSavedScene) {
  EXPECT_TRUE(ValidateBuildPreflight(Settings(), Valid()).Passed());
}
TEST(BuildManagerPreflight, AggregatesDirtyPlaybackAndCameraFailures) {
  auto snapshot = Valid();
  snapshot.editor_stopped = false;
  snapshot.build_settings_saved = false;
  snapshot.startup_scene_saved = false;
  snapshot.main_camera_present = false;
  snapshot.unsaved_project_assets = {"Assets/A.evescene", "Assets/B.eveasset"};
  const auto result = ValidateBuildPreflight(Settings(), snapshot);
  EXPECT_EQ(result.errors.size(), 6);
}
TEST(BuildManagerPreflight, ResizePermissionsDoNotAffectModeValidity) {
  auto settings = Settings();
  settings.window_mode = WindowDisplayMode::ExclusiveFullscreen;
  settings.allow_window_resize = false;
  settings.allow_resolution_change = true;
  EXPECT_TRUE(ValidateBuildPreflight(settings, Valid()).Passed());
  settings.allow_window_resize = true;
  settings.allow_resolution_change = false;
  EXPECT_TRUE(ValidateBuildPreflight(settings, Valid()).Passed());
}
TEST(BuildManagerPreflight, RejectsPausedPlayingAndNeverSavedScene) {
  auto snapshot = Valid();
  snapshot.editor_stopped = false;
  snapshot.startup_scene_saved = false;
  const auto result = ValidateBuildPreflight(Settings(), snapshot);
  EXPECT_EQ(result.errors.size(), 2);
}
