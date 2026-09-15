#include <gtest/gtest.h>
#include "ProjectBuildSettingsSerialization.hpp"
using namespace evo_engine;
TEST(ProjectBuildSettings, RoundTripsWithoutChangingSourceStartup) {
  ProjectBuildSettings source;
  source.application_name = "Runtime App";
  source.output_directory = L"C:/Exports/Runtime App";
  source.startup_scene_handle = 222;
  source.window_mode = WindowDisplayMode::BorderlessFullscreen;
  source.window_width = 1600;
  source.window_height = 900;
  source.allow_window_resize = false;
  source.show_console = true;
  YAML::Emitter out;
  SerializeProjectBuildSettings(source, out);
  const auto root = YAML::Load(out.c_str());
  EXPECT_EQ(DeserializeProjectBuildSettings(root), source);
  EXPECT_FALSE(root["start_scene_handle"]);
}

TEST(ProjectBuildSettings, ExistingSettingsDefaultToHiddenConsole) {
  YAML::Emitter out;
  SerializeProjectBuildSettings(ProjectBuildSettings{}, out);
  auto root = YAML::Load(out.c_str());
  root.remove("show_console");
  EXPECT_FALSE(DeserializeProjectBuildSettings(root).show_console);
}
