#include "EvoEngine_SDK_PCH.hpp"

#include "DdgiSettings.hpp"
#include "DdgiVolume.hpp"
#include "Scene.hpp"
#include "VolumetricCloudSettings.hpp"

#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>

#include <gtest/gtest.h>

using namespace evo_engine;

namespace {
std::string ReadTextFile(const std::filesystem::path& path) {
  std::ifstream file(path);
  EXPECT_TRUE(file.good()) << path.string();
  return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}
}  // namespace

TEST(VolumetricCloudSettings, DefaultsAreDisabledAndAuthoringFriendly) {
  const VolumetricCloudSettings settings;

  EXPECT_FALSE(settings.enabled);
  EXPECT_FLOAT_EQ(settings.coverage, 0.45f);
  EXPECT_FLOAT_EQ(settings.density, 0.35f);
  EXPECT_FLOAT_EQ(settings.bottom_altitude, 1500.0f);
  EXPECT_FLOAT_EQ(settings.top_altitude, 4500.0f);
  EXPECT_EQ(settings.wind_direction, glm::vec2(1.0f, 0.0f));
  EXPECT_FLOAT_EQ(settings.wind_speed, 25.0f);
  EXPECT_EQ(settings.primary_step_count, 64);
  EXPECT_EQ(settings.light_step_count, 8);
  EXPECT_FLOAT_EQ(settings.lighting_intensity, 1.0f);
  EXPECT_FLOAT_EQ(settings.ambient_lighting_strength, 0.2f);
  EXPECT_FLOAT_EQ(settings.phase_anisotropy, 0.65f);
  EXPECT_FALSE(settings.debug_visualization);
  EXPECT_EQ(settings.debug_mode, 0);
}

TEST(VolumetricCloudSettings, ClampSettingsKeepsValuesSupported) {
  VolumetricCloudSettings settings;
  settings.coverage = -1.0f;
  settings.density = 20.0f;
  settings.bottom_altitude = -10.0f;
  settings.top_altitude = -5.0f;
  settings.wind_direction = {0.0f, 0.0f};
  settings.wind_speed = -1.0f;
  settings.primary_step_count = 0;
  settings.light_step_count = 4096;
  settings.lighting_intensity = 200.0f;
  settings.ambient_lighting_strength = -1.0f;
  settings.phase_anisotropy = 2.0f;
  settings.debug_mode = 99;

  settings.ClampSettings();

  EXPECT_FLOAT_EQ(settings.coverage, 0.0f);
  EXPECT_FLOAT_EQ(settings.density, 10.0f);
  EXPECT_FLOAT_EQ(settings.bottom_altitude, 0.0f);
  EXPECT_FLOAT_EQ(settings.top_altitude, 1.0f);
  EXPECT_EQ(settings.wind_direction, glm::vec2(1.0f, 0.0f));
  EXPECT_FLOAT_EQ(settings.wind_speed, 0.0f);
  EXPECT_EQ(settings.primary_step_count, 1);
  EXPECT_EQ(settings.light_step_count, 128);
  EXPECT_FLOAT_EQ(settings.lighting_intensity, 100.0f);
  EXPECT_FLOAT_EQ(settings.ambient_lighting_strength, 0.0f);
  EXPECT_FLOAT_EQ(settings.phase_anisotropy, 0.99f);
  EXPECT_EQ(settings.debug_mode, 3);
}

TEST(VolumetricCloudSettings, SceneEnvironmentDeserializesCloudSettings) {
  Scene::Environment restored;
  restored.Deserialize(YAML::Load(R"(
volumetric_cloud_settings:
  enabled: true
  coverage: 0.7
  density: 0.9
  bottom_altitude: 1200.0
  top_altitude: 6200.0
  wind_direction: [0.0, 1.0]
  wind_speed: 45.0
  primary_step_count: 96
  light_step_count: 12
  lighting_intensity: 1.4
  ambient_lighting_strength: 0.35
  phase_anisotropy: 0.4
  debug_visualization: true
  debug_mode: 2
)"));

  const auto& settings = restored.volumetric_cloud_settings;
  EXPECT_TRUE(settings.enabled);
  EXPECT_FLOAT_EQ(settings.coverage, 0.7f);
  EXPECT_FLOAT_EQ(settings.density, 0.9f);
  EXPECT_FLOAT_EQ(settings.bottom_altitude, 1200.0f);
  EXPECT_FLOAT_EQ(settings.top_altitude, 6200.0f);
  EXPECT_EQ(settings.wind_direction, glm::vec2(0.0f, 1.0f));
  EXPECT_FLOAT_EQ(settings.wind_speed, 45.0f);
  EXPECT_EQ(settings.primary_step_count, 96);
  EXPECT_EQ(settings.light_step_count, 12);
  EXPECT_FLOAT_EQ(settings.lighting_intensity, 1.4f);
  EXPECT_FLOAT_EQ(settings.ambient_lighting_strength, 0.35f);
  EXPECT_FLOAT_EQ(settings.phase_anisotropy, 0.4f);
  EXPECT_TRUE(settings.debug_visualization);
  EXPECT_EQ(settings.debug_mode, 2);
}

TEST(VolumetricCloudSettings, SceneEnvironmentKeepsCloudDefaultsForLegacyYaml) {
  Scene::Environment restored;
  restored.Deserialize(YAML::Load(R"(
background_color: [0.1, 0.2, 0.3]
environment_gamma: 2.0
)"));

  EXPECT_FALSE(restored.volumetric_cloud_settings.enabled);
  EXPECT_FLOAT_EQ(restored.volumetric_cloud_settings.coverage, 0.45f);
  EXPECT_EQ(restored.volumetric_cloud_settings.primary_step_count, 64);
}

TEST(VolumetricCloudSettings, CloudSettingsStaySeparateFromDdgiSettings) {
  const auto scene_source =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "src" / "Scene.cpp");
  const auto ddgi_settings_source = ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" /
                                                 "include" / "Rendering" / "PBR" / "DdgiSettings.hpp");
  const auto ddgi_volume_source = ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" /
                                               "include" / "Rendering" / "PBR" / "DdgiVolume.hpp");
  ASSERT_FALSE(scene_source.empty());
  ASSERT_FALSE(ddgi_settings_source.empty());
  ASSERT_FALSE(ddgi_volume_source.empty());

  EXPECT_NE(scene_source.find("\"volumetric_cloud_settings\""), std::string::npos);
  EXPECT_NE(scene_source.find("SerializeVolumetricCloudSettings"), std::string::npos);
  EXPECT_NE(scene_source.find("DeserializeVolumetricCloudSettings"), std::string::npos);
  EXPECT_NE(scene_source.find("\"primary_step_count\" << YAML::Value << settings.primary_step_count"),
            std::string::npos);
  EXPECT_NE(scene_source.find("\"lighting_intensity\" << YAML::Value << settings.lighting_intensity"),
            std::string::npos);
  EXPECT_NE(scene_source.find("settings.ClampSettings();"), std::string::npos);
  EXPECT_EQ(ddgi_settings_source.find("cloud"), std::string::npos);
  EXPECT_EQ(ddgi_volume_source.find("cloud"), std::string::npos);
}
