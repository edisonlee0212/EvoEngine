#include "EvoEngine_SDK_PCH.hpp"

#include "DdgiSettings.hpp"
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

void ExpectDefaultVolumetricCloudSettings(const VolumetricCloudSettings& settings) {
  EXPECT_FALSE(settings.enabled);
  EXPECT_FLOAT_EQ(settings.coverage, 0.62f);
  EXPECT_FLOAT_EQ(settings.density, 1.15f);
  EXPECT_FLOAT_EQ(settings.bottom_altitude, 25.0f);
  EXPECT_FLOAT_EQ(settings.top_altitude, 180.0f);
  EXPECT_FLOAT_EQ(settings.max_march_distance, 900.0f);
  EXPECT_EQ(settings.wind_direction, glm::vec2(1.0f, 0.0f));
  EXPECT_FLOAT_EQ(settings.wind_speed, 8.0f);
  EXPECT_EQ(settings.primary_step_count, 96);
  EXPECT_EQ(settings.light_step_count, 12);
  EXPECT_EQ(settings.resolution_divisor, 2);
  EXPECT_FLOAT_EQ(settings.lighting_intensity, 2.2f);
  EXPECT_FLOAT_EQ(settings.ambient_lighting_strength, 0.18f);
  EXPECT_FLOAT_EQ(settings.phase_anisotropy, 0.72f);
  EXPECT_FLOAT_EQ(settings.base_noise_scale, 0.018f);
  EXPECT_FLOAT_EQ(settings.detail_noise_scale, 0.075f);
  EXPECT_FLOAT_EQ(settings.extinction_scale, 0.018f);
  EXPECT_TRUE(settings.use_spherical_atmosphere);
  EXPECT_FLOAT_EQ(settings.atmosphere_radius, 10000.0f);
  EXPECT_FLOAT_EQ(settings.cloud_type, 0.55f);
  EXPECT_FLOAT_EQ(settings.curl_strength, 1.9f);
  EXPECT_FLOAT_EQ(settings.coarse_step_fraction, 0.05f);
  EXPECT_FLOAT_EQ(settings.fine_step_scale, 0.3f);
  EXPECT_EQ(settings.empty_step_fallback_count, 10);
  EXPECT_TRUE(settings.enable_temporal_reprojection);
  EXPECT_FLOAT_EQ(settings.temporal_blend_factor, 0.9f);
  EXPECT_TRUE(settings.enable_cloud_shadows);
  EXPECT_FLOAT_EQ(settings.cloud_shadow_strength, 0.35f);
  EXPECT_EQ(settings.cloud_shadow_step_count, 6);
  EXPECT_FALSE(settings.debug_visualization);
  EXPECT_EQ(settings.debug_mode, 0);
}
}  // namespace

TEST(VolumetricCloudSettings, DefaultsAreDisabledWhileUnowned) {
  ExpectDefaultVolumetricCloudSettings(VolumetricCloudSettings{});
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
  settings.atmosphere_radius = 0.0f;
  settings.cloud_type = -1.0f;
  settings.curl_strength = -2.0f;
  settings.coarse_step_fraction = 0.0f;
  settings.fine_step_scale = 2.0f;
  settings.empty_step_fallback_count = 0;
  settings.temporal_blend_factor = 2.0f;
  settings.cloud_shadow_strength = -1.0f;
  settings.cloud_shadow_step_count = 0;
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
  EXPECT_FLOAT_EQ(settings.atmosphere_radius, 10.0f);
  EXPECT_FLOAT_EQ(settings.cloud_type, 0.0f);
  EXPECT_FLOAT_EQ(settings.curl_strength, 0.0f);
  EXPECT_FLOAT_EQ(settings.coarse_step_fraction, 0.0001f);
  EXPECT_FLOAT_EQ(settings.fine_step_scale, 1.0f);
  EXPECT_EQ(settings.empty_step_fallback_count, 1);
  EXPECT_FLOAT_EQ(settings.temporal_blend_factor, 0.98f);
  EXPECT_FLOAT_EQ(settings.cloud_shadow_strength, 0.0f);
  EXPECT_EQ(settings.cloud_shadow_step_count, 1);
  EXPECT_EQ(settings.debug_mode, 6);
}

TEST(VolumetricCloudSettings, CloudSettingsRemainImplementedButSceneUnowned) {
  const auto scene_source =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "src" / "Scene.cpp");
  const auto scene_header = ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" /
                                         "include" / "Core" / "ECS" / "Scene.hpp");
  const auto render_layer_source =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "src" / "RenderLayer.cpp");
  const auto cloud_pass_source = ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" /
                                              "src" / "RenderPasses" / "VolumetricCloudsPass.cpp");
  const auto inspector_source = ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" /
                                             "src" / "Editor" / "SDKInspectionAdapters.cpp");
  const auto ddgi_settings_source = ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" /
                                                 "include" / "Rendering" / "PBR" / "DdgiSettings.hpp");
  const auto ddgi_volume_source = ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" /
                                               "include" / "Rendering" / "PBR" / "EnvironmentalLighting.hpp");
  ASSERT_FALSE(scene_source.empty());
  ASSERT_FALSE(scene_header.empty());
  ASSERT_FALSE(render_layer_source.empty());
  ASSERT_FALSE(cloud_pass_source.empty());
  ASSERT_FALSE(inspector_source.empty());
  ASSERT_FALSE(ddgi_settings_source.empty());
  ASSERT_FALSE(ddgi_volume_source.empty());

  EXPECT_EQ(scene_header.find("Scene::Environment"), std::string::npos);
  EXPECT_EQ(scene_header.find("VolumetricCloudSettings"), std::string::npos);
  EXPECT_EQ(scene_source.find("\"environment\""), std::string::npos);
  EXPECT_EQ(scene_source.find("volumetric_cloud_settings"), std::string::npos);
  EXPECT_EQ(render_layer_source.find("scene->environment."), std::string::npos);
  EXPECT_NE(render_layer_source.find("VolumetricCloudSettings volumetric_cloud_settings{}"), std::string::npos);
  EXPECT_NE(cloud_pass_source.find("VolumetricCloudSettings settings = parameters.settings"), std::string::npos);
  EXPECT_EQ(inspector_source.find("Volumetric clouds"), std::string::npos);
  EXPECT_EQ(ddgi_settings_source.find("cloud"), std::string::npos);
  EXPECT_EQ(ddgi_volume_source.find("cloud"), std::string::npos);
}

TEST(VolumetricCloudSettings, CloudSettingsDoNotFeedDdgiSceneChangeTriggers) {
  const auto render_layer_source =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "src" / "RenderLayer.cpp");
  ASSERT_FALSE(render_layer_source.empty());

  const auto storage_update = ExtractBetween(render_layer_source, "bool RenderLayer::UpdateRenderInstanceStorage(",
                                             "void RenderLayer::PrepareEnvironmentalBrdfLut()");
  const auto trigger_block =
      ExtractBetween(storage_update, "if (track_ddgi_scene_inputs) {",
                     "PreserveDdgiRenderInfo(current_render_instances->render_info_block, current_render_info);");
  const auto environment_tracker = ExtractBetween(render_layer_source, "const auto track_ddgi_environment_signature",
                                                  "if (ddgi_settings.runtime.pause_updates)");
  ASSERT_FALSE(storage_update.empty());
  ASSERT_FALSE(trigger_block.empty());
  ASSERT_FALSE(environment_tracker.empty());

  EXPECT_NE(trigger_block.find("active_light_keys != ddgi_previous_active_light_keys_"), std::string::npos);
  EXPECT_NE(environment_tracker.find("runtime_state.has_previous_environment_signature"), std::string::npos);
  EXPECT_NE(environment_tracker.find("signature != runtime_state.previous_environment_signature"), std::string::npos);
  EXPECT_NE(trigger_block.find("light_signatures != ddgi_previous_light_signatures_"), std::string::npos);
  EXPECT_NE(trigger_block.find("geometry_signatures != ddgi_previous_geometry_signatures_"), std::string::npos);
  EXPECT_NE(trigger_block.find("material_inputs.keys != ddgi_previous_material_keys_"), std::string::npos);
  EXPECT_NE(trigger_block.find("material_inputs.materials != ddgi_previous_material_signatures_"), std::string::npos);
  EXPECT_NE(trigger_block.find("material_inputs.textures != ddgi_previous_material_texture_signatures_"),
            std::string::npos);
  EXPECT_EQ(trigger_block.find("volumetric_cloud_settings"), std::string::npos);
  EXPECT_EQ(trigger_block.find("VolumetricCloudSettings"), std::string::npos);
  EXPECT_EQ(trigger_block.find("cloud"), std::string::npos);
  EXPECT_EQ(environment_tracker.find("cloud"), std::string::npos);
}
