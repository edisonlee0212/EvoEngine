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

std::string ExtractBetween(const std::string& source, const std::string& begin, const std::string& end) {
  const auto begin_index = source.find(begin);
  EXPECT_NE(begin_index, std::string::npos);
  if (begin_index == std::string::npos) {
    return {};
  }
  const auto end_index = source.find(end, begin_index);
  EXPECT_NE(end_index, std::string::npos);
  if (end_index == std::string::npos) {
    return {};
  }
  return source.substr(begin_index, end_index - begin_index);
}
}  // namespace

TEST(VolumetricCloudSettings, DefaultsAreDisabledAndAuthoringFriendly) {
  const VolumetricCloudSettings settings;

  EXPECT_FALSE(settings.enabled);
  EXPECT_FLOAT_EQ(settings.coverage, 0.65f);
  EXPECT_FLOAT_EQ(settings.density, 1.0f);
  EXPECT_FLOAT_EQ(settings.bottom_altitude, 80.0f);
  EXPECT_FLOAT_EQ(settings.top_altitude, 550.0f);
  EXPECT_FLOAT_EQ(settings.max_march_distance, 5000.0f);
  EXPECT_EQ(settings.wind_direction, glm::vec2(1.0f, 0.0f));
  EXPECT_FLOAT_EQ(settings.wind_speed, 25.0f);
  EXPECT_EQ(settings.primary_step_count, 64);
  EXPECT_EQ(settings.light_step_count, 8);
  EXPECT_EQ(settings.resolution_divisor, 1);
  EXPECT_FLOAT_EQ(settings.lighting_intensity, 1.0f);
  EXPECT_FLOAT_EQ(settings.ambient_lighting_strength, 0.2f);
  EXPECT_FLOAT_EQ(settings.phase_anisotropy, 0.65f);
  EXPECT_FLOAT_EQ(settings.base_noise_scale, 0.012f);
  EXPECT_FLOAT_EQ(settings.detail_noise_scale, 0.05f);
  EXPECT_FLOAT_EQ(settings.extinction_scale, 0.01f);
  EXPECT_FALSE(settings.debug_visualization);
  EXPECT_EQ(settings.debug_mode, 0);
}

TEST(VolumetricCloudSettings, ClampSettingsKeepsValuesSupported) {
  VolumetricCloudSettings settings;
  settings.coverage = -1.0f;
  settings.density = 20.0f;
  settings.bottom_altitude = -10.0f;
  settings.top_altitude = -5.0f;
  settings.max_march_distance = -20.0f;
  settings.wind_direction = {0.0f, 0.0f};
  settings.wind_speed = -1.0f;
  settings.primary_step_count = 0;
  settings.light_step_count = 4096;
  settings.resolution_divisor = 3;
  settings.lighting_intensity = 200.0f;
  settings.ambient_lighting_strength = -1.0f;
  settings.phase_anisotropy = 2.0f;
  settings.base_noise_scale = -1.0f;
  settings.detail_noise_scale = 20.0f;
  settings.extinction_scale = 2.0f;
  settings.debug_mode = 99;

  settings.ClampSettings();

  EXPECT_FLOAT_EQ(settings.coverage, 0.0f);
  EXPECT_FLOAT_EQ(settings.density, 10.0f);
  EXPECT_FLOAT_EQ(settings.bottom_altitude, 0.0f);
  EXPECT_FLOAT_EQ(settings.top_altitude, 1.0f);
  EXPECT_FLOAT_EQ(settings.max_march_distance, 1.0f);
  EXPECT_EQ(settings.wind_direction, glm::vec2(1.0f, 0.0f));
  EXPECT_FLOAT_EQ(settings.wind_speed, 0.0f);
  EXPECT_EQ(settings.primary_step_count, 1);
  EXPECT_EQ(settings.light_step_count, 128);
  EXPECT_EQ(settings.resolution_divisor, 4);
  EXPECT_FLOAT_EQ(settings.lighting_intensity, 100.0f);
  EXPECT_FLOAT_EQ(settings.ambient_lighting_strength, 0.0f);
  EXPECT_FLOAT_EQ(settings.phase_anisotropy, 0.99f);
  EXPECT_FLOAT_EQ(settings.base_noise_scale, 0.00001f);
  EXPECT_FLOAT_EQ(settings.detail_noise_scale, 10.0f);
  EXPECT_FLOAT_EQ(settings.extinction_scale, 1.0f);
  EXPECT_EQ(settings.debug_mode, 6);
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
  max_march_distance: 9000.0
  wind_direction: [0.0, 1.0]
  wind_speed: 45.0
  primary_step_count: 96
  light_step_count: 12
  resolution_divisor: 4
  lighting_intensity: 1.4
  ambient_lighting_strength: 0.35
  phase_anisotropy: 0.4
  base_noise_scale: 0.002
  detail_noise_scale: 0.02
  extinction_scale: 0.004
  debug_visualization: true
  debug_mode: 2
)"));

  const auto& settings = restored.volumetric_cloud_settings;
  EXPECT_TRUE(settings.enabled);
  EXPECT_FLOAT_EQ(settings.coverage, 0.7f);
  EXPECT_FLOAT_EQ(settings.density, 0.9f);
  EXPECT_FLOAT_EQ(settings.bottom_altitude, 1200.0f);
  EXPECT_FLOAT_EQ(settings.top_altitude, 6200.0f);
  EXPECT_FLOAT_EQ(settings.max_march_distance, 9000.0f);
  EXPECT_EQ(settings.wind_direction, glm::vec2(0.0f, 1.0f));
  EXPECT_FLOAT_EQ(settings.wind_speed, 45.0f);
  EXPECT_EQ(settings.primary_step_count, 96);
  EXPECT_EQ(settings.light_step_count, 12);
  EXPECT_EQ(settings.resolution_divisor, 4);
  EXPECT_FLOAT_EQ(settings.lighting_intensity, 1.4f);
  EXPECT_FLOAT_EQ(settings.ambient_lighting_strength, 0.35f);
  EXPECT_FLOAT_EQ(settings.phase_anisotropy, 0.4f);
  EXPECT_FLOAT_EQ(settings.base_noise_scale, 0.002f);
  EXPECT_FLOAT_EQ(settings.detail_noise_scale, 0.02f);
  EXPECT_FLOAT_EQ(settings.extinction_scale, 0.004f);
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
  EXPECT_FLOAT_EQ(restored.volumetric_cloud_settings.coverage, 0.65f);
  EXPECT_FLOAT_EQ(restored.volumetric_cloud_settings.max_march_distance, 5000.0f);
  EXPECT_EQ(restored.volumetric_cloud_settings.primary_step_count, 64);
  EXPECT_EQ(restored.volumetric_cloud_settings.resolution_divisor, 1);
  EXPECT_FLOAT_EQ(restored.volumetric_cloud_settings.base_noise_scale, 0.012f);
  EXPECT_FLOAT_EQ(restored.volumetric_cloud_settings.extinction_scale, 0.01f);
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
  EXPECT_NE(scene_source.find("\"resolution_divisor\" << YAML::Value << settings.resolution_divisor"),
            std::string::npos);
  EXPECT_NE(scene_source.find("\"lighting_intensity\" << YAML::Value << settings.lighting_intensity"),
            std::string::npos);
  EXPECT_NE(scene_source.find("\"max_march_distance\" << YAML::Value << settings.max_march_distance"),
            std::string::npos);
  EXPECT_NE(scene_source.find("\"base_noise_scale\" << YAML::Value << settings.base_noise_scale"), std::string::npos);
  EXPECT_NE(scene_source.find("\"extinction_scale\" << YAML::Value << settings.extinction_scale"), std::string::npos);
  EXPECT_NE(scene_source.find("settings.ClampSettings();"), std::string::npos);
  EXPECT_EQ(ddgi_settings_source.find("cloud"), std::string::npos);
  EXPECT_EQ(ddgi_volume_source.find("cloud"), std::string::npos);
}

TEST(VolumetricCloudSettings, CloudSettingsDoNotFeedDdgiSceneChangeTriggers) {
  const auto render_layer_source =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "src" / "RenderLayer.cpp");
  ASSERT_FALSE(render_layer_source.empty());

  const auto trigger_block =
      ExtractBetween(render_layer_source, "ddgi_scene_change_triggers_ =", "const bool render_instance_updated");
  ASSERT_FALSE(trigger_block.empty());

  EXPECT_NE(trigger_block.find("active_light_keys != ddgi_previous_active_light_keys_"), std::string::npos);
  EXPECT_NE(trigger_block.find("current_render_instances->environment_info_block != "
                               "ddgi_previous_environment_info_block_"),
            std::string::npos);
  EXPECT_NE(trigger_block.find("light_signatures != ddgi_previous_light_signatures_"), std::string::npos);
  EXPECT_NE(trigger_block.find("geometry_signatures != ddgi_previous_geometry_signatures_"), std::string::npos);
  EXPECT_NE(trigger_block.find("geometry_storage_version"), std::string::npos);
  EXPECT_NE(trigger_block.find("texture_storage_version"), std::string::npos);
  EXPECT_EQ(trigger_block.find("volumetric_cloud_settings"), std::string::npos);
  EXPECT_EQ(trigger_block.find("VolumetricCloudSettings"), std::string::npos);
  EXPECT_EQ(trigger_block.find("cloud"), std::string::npos);
}

TEST(VolumetricCloudSettings, DemoAppSmokeChecksCloudDensityVariation) {
  const auto demo_app_source =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_App" / "src" / "DemoApp.cpp");
  ASSERT_FALSE(demo_app_source.empty());

  EXPECT_NE(demo_app_source.find("luminance_standard_deviation"), std::string::npos);
  EXPECT_NE(demo_app_source.find("density_luminance_range"), std::string::npos);
  EXPECT_NE(demo_app_source.find("visible_cloud_settings.debug_visualization = true"), std::string::npos);
  EXPECT_NE(demo_app_source.find("visible_cloud_settings.debug_mode = 1"), std::string::npos);
  EXPECT_NE(demo_app_source.find("volumetric cloud density debug output is spatially flat"), std::string::npos);
}

TEST(VolumetricCloudSettings, DemoAppSmokeKeepsCloudMutationsDdgiSteady) {
  const auto demo_app_source =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_App" / "src" / "DemoApp.cpp");
  ASSERT_FALSE(demo_app_source.empty());

  EXPECT_NE(demo_app_source.find("ValidateRenderingDemoCloudSettingsDoNotRefreshDdgi"), std::string::npos);
  EXPECT_NE(demo_app_source.find("DDGI reported a scene refresh after cloud settings changed"), std::string::npos);
  EXPECT_NE(demo_app_source.find("cloud_settings.enabled = true"), std::string::npos);
  EXPECT_NE(demo_app_source.find("scene->environment.volumetric_cloud_settings.wind_speed = 80.0f"), std::string::npos);
  EXPECT_NE(demo_app_source.find("RenderLayer::DdgiUpdateReasonSteadyState"), std::string::npos);
}
