#include "EvoEngine_SDK_PCH.hpp"

#include "DdgiVolume.hpp"
#include "PointCloudSample.hpp"
#include "RenderLayer.hpp"
#include "Scene.hpp"

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include <gtest/gtest.h>

using namespace evo_engine;

namespace {
std::string ReadTextFile(const std::filesystem::path& path) {
  std::ifstream file(path);
  EXPECT_TRUE(file.good()) << path.string();
  return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}
}  // namespace

TEST(DdgiVolume, DefaultProbeGridMatchesSceneAuthoringDefaults) {
  DdgiVolume volume;
  Scene::Environment environment;

  EXPECT_EQ(volume.probe_counts, glm::ivec3(10, 6, 16));
  EXPECT_EQ(volume.probe_spacing, glm::vec3(1.5f));
  EXPECT_EQ(volume.volume_origin, glm::vec3(0.0f, 3.0f, 3.0f));
  EXPECT_EQ(volume.movement_type, static_cast<int>(DdgiVolumeMovementType::Default));
  EXPECT_TRUE(volume.enable_probe_relocation);
  EXPECT_TRUE(volume.enable_probe_variability);
  EXPECT_TRUE(volume.enable_probe_variability_gating);
  EXPECT_FLOAT_EQ(volume.relocation_distance, 0.25f);
  EXPECT_FLOAT_EQ(volume.random_ray_backface_threshold, 0.1f);
  EXPECT_FLOAT_EQ(volume.fixed_ray_backface_threshold, 0.25f);
  EXPECT_FLOAT_EQ(volume.probe_variability_threshold, 0.2f);
  EXPECT_EQ(volume.probe_variability_min_samples, 16);
  EXPECT_EQ(volume.warmup_trigger_conditions, DdgiVolumeTriggerConditionLightEnableChanged);
  EXPECT_EQ(volume.variability_reset_trigger_conditions,
            DdgiVolumeTriggerConditionLightingConditionChanged | DdgiVolumeTriggerConditionGeometryChanged);
  EXPECT_EQ(volume.GetProbeAmount(), 960u);
  EXPECT_FLOAT_EQ(volume.GetProbeLocalPosition({0, 0, 0}).x, -6.75f);
  EXPECT_FLOAT_EQ(volume.GetProbeLocalPosition({0, 0, 0}).y, -0.75f);
  EXPECT_FLOAT_EQ(volume.GetProbeLocalPosition({0, 0, 0}).z, -8.25f);
  EXPECT_GE(volume.max_visualized_probes, static_cast<int>(volume.GetProbeAmount()));

  const auto& defaults = environment.ddgi_settings.volume_defaults;
  EXPECT_EQ(defaults.probe_counts, volume.probe_counts);
  EXPECT_EQ(defaults.probe_spacing, volume.probe_spacing);
  EXPECT_EQ(defaults.volume_origin, volume.volume_origin);
  EXPECT_FLOAT_EQ(environment.ddgi_settings.runtime.normal_bias, 0.1f);
  EXPECT_FLOAT_EQ(environment.ddgi_settings.runtime.view_bias, 0.1f);
  EXPECT_FLOAT_EQ(environment.ddgi_settings.runtime.max_ray_distance, 1e27f);
  EXPECT_FLOAT_EQ(environment.ddgi_settings.runtime.visibility_moment_bias, 0.02f);
  EXPECT_FLOAT_EQ(environment.ddgi_settings.runtime.indirect_intensity, 1.0f);
  EXPECT_EQ(environment.ddgi_settings.runtime.warmup_frames, 16);
  EXPECT_FLOAT_EQ(environment.ddgi_settings.runtime.distance_exponent, 50.0f);
  EXPECT_FLOAT_EQ(environment.ddgi_settings.runtime.irradiance_threshold, 0.25f);
  EXPECT_FLOAT_EQ(environment.ddgi_settings.runtime.brightness_threshold, 0.10f);
  EXPECT_EQ(defaults.movement_type, static_cast<int>(DdgiVolumeMovementType::Default));
  EXPECT_TRUE(defaults.enable_probe_relocation);
  EXPECT_TRUE(defaults.enable_probe_variability);
  EXPECT_TRUE(defaults.enable_probe_variability_gating);
  EXPECT_FLOAT_EQ(defaults.relocation_distance, 0.25f);
  EXPECT_FLOAT_EQ(environment.ddgi_settings.debug.visualization_scale, 2.0f);
  EXPECT_FLOAT_EQ(defaults.random_ray_backface_threshold, 0.1f);
  EXPECT_FLOAT_EQ(defaults.fixed_ray_backface_threshold, 0.25f);
  EXPECT_FLOAT_EQ(defaults.probe_variability_threshold, 0.2f);
  EXPECT_EQ(defaults.probe_variability_min_samples, 16);
  EXPECT_EQ(RenderLayer::GetDdgiAllocatedProbeCount(environment.ddgi_settings), 960u);
}

TEST(DdgiVolume, ProbePositionsAreCenteredAroundVolumeOrigin) {
  DdgiVolume volume;
  volume.probe_counts = {3, 2, 1};
  volume.probe_spacing = glm::vec3(2.0f);
  volume.volume_origin = {1.0f, 2.0f, 3.0f};
  volume.ClampSettings();

  EXPECT_EQ(volume.GetProbeAmount(), 6u);
  EXPECT_FLOAT_EQ(volume.GetLocalGridSize().x, 4.0f);
  EXPECT_FLOAT_EQ(volume.GetLocalGridSize().y, 2.0f);
  EXPECT_FLOAT_EQ(volume.GetLocalGridSize().z, 0.0f);

  const auto first_probe = volume.GetProbeLocalPosition({0, 0, 0});
  EXPECT_FLOAT_EQ(first_probe.x, -1.0f);
  EXPECT_FLOAT_EQ(first_probe.y, 1.0f);
  EXPECT_FLOAT_EQ(first_probe.z, 3.0f);

  const auto last_probe = volume.GetProbeLocalPosition({2, 1, 0});
  EXPECT_FLOAT_EQ(last_probe.x, 3.0f);
  EXPECT_FLOAT_EQ(last_probe.y, 3.0f);
  EXPECT_FLOAT_EQ(last_probe.z, 3.0f);

  volume.probe_counts = {4, 2, 1};
  volume.ClampSettings();
  const auto expanded_first_probe = volume.GetProbeLocalPosition({0, 0, 0});
  EXPECT_FLOAT_EQ(expanded_first_probe.x, -2.0f);
  EXPECT_FLOAT_EQ(expanded_first_probe.y, 1.0f);
  EXPECT_FLOAT_EQ(expanded_first_probe.z, 3.0f);
}

TEST(DdgiVolume, DdgiAppCornellComparisonAvoidsWallAnchoredProbes) {
  const auto profile_header =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_App" / "include" / "DemoProfiles.hpp");
  const auto profile_source =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_App" / "src" / "DemoProfiles.cpp");
  const auto app_source =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_App" / "src" / "DDGIApp.cpp");
  ASSERT_FALSE(profile_header.empty());
  ASSERT_FALSE(profile_source.empty());
  ASSERT_FALSE(app_source.empty());

  EXPECT_NE(profile_source.find("const glm::vec3 kDdgiCornellBoxVolumeOrigin = {0.0f, 0.0f, 0.0f};"),
            std::string::npos);
  EXPECT_NE(profile_source.find("const glm::ivec3 kDdgiCornellBoxProbeCounts = {13, 13, 14};"), std::string::npos);
  EXPECT_NE(profile_source.find("constexpr float kDdgiCornellBoxProbeSpacing = 0.14333334f;"), std::string::npos);
  EXPECT_NE(profile_header.find("float point_light_brightness = 2.0f;"), std::string::npos);
  EXPECT_NE(profile_header.find("float ddgi_indirect_intensity = 1.0f;"), std::string::npos);
  EXPECT_EQ(profile_source.find("kComparisonProbeUpdateBudget"), std::string::npos);
  EXPECT_EQ(profile_source.find("ddgi_final_visibility_strength"), std::string::npos);
  EXPECT_EQ(app_source.find("argument == \"--disable-ddgi-final-visibility\""), std::string::npos);
  EXPECT_NE(profile_source.find("volume->probe_spacing = glm::vec3(kDdgiCornellBoxProbeSpacing);"), std::string::npos);
  EXPECT_NE(profile_header.find("bool enable_probe_relocation = true;"), std::string::npos);
  EXPECT_NE(profile_header.find("bool enable_probe_classification = true;"), std::string::npos);
  EXPECT_NE(profile_source.find("void DisablePostProcessing(const std::shared_ptr<Scene>& scene)"), std::string::npos);
  EXPECT_NE(profile_source.find("main_camera->post_processing_stack_ref.Clear();"), std::string::npos);
  EXPECT_NE(profile_source.find("camera->post_processing_stack_ref.Clear();"), std::string::npos);
  EXPECT_NE(profile_source.find("DisablePostProcessing(scene);"), std::string::npos);
  EXPECT_NE(profile_source.find("volume->relocation_distance = kDdgiCornellBoxProbeSpacing * 0.5f;"),
            std::string::npos);
  EXPECT_NE(app_source.find("argument == \"--disable-probe-relocation\""), std::string::npos);
  EXPECT_NE(app_source.find("argument == \"--disable-probe-classification\""), std::string::npos);
  EXPECT_EQ(profile_source.find("kComparisonFirstProbeOffset"), std::string::npos);
}

TEST(DdgiVolume, RenderingDemoOffsetsWallAdjacentProbes) {
  const auto demo_source =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_App" / "src" / "DemoScene.cpp");
  const auto demo_app_source =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_App" / "src" / "DemoApp.cpp");
  ASSERT_FALSE(demo_source.empty());
  ASSERT_FALSE(demo_app_source.empty());

  EXPECT_NE(demo_source.find("ddgi_volume->probe_counts = {10, 6, 16};"), std::string::npos);
  EXPECT_NE(demo_source.find("ddgi_volume->probe_spacing = glm::vec3(1.5f);"), std::string::npos);
  EXPECT_NE(demo_source.find("ddgi_volume->volume_origin = {0.0f, 3.0f, 3.0f};"), std::string::npos);
  EXPECT_EQ(demo_source.find("ddgi_volume->ray_count"), std::string::npos);
  EXPECT_EQ(demo_source.find("ddgi_volume->normal_bias"), std::string::npos);
  EXPECT_NE(demo_source.find("ddgi_volume->relocation_distance = 0.25f;"), std::string::npos);
  EXPECT_EQ(demo_source.find("ddgi_volume->relocation_distance = ddgi_volume->probe_spacing * 0.5f;"),
            std::string::npos);
  EXPECT_EQ(demo_source.find("adaptive_probe_update_budget"), std::string::npos);
  EXPECT_NE(demo_source.find("settings.debug.visualization_scale = 2.0f;"), std::string::npos);
  EXPECT_NE(demo_source.find("ddgi_volume->enable_probe_relocation = true;"), std::string::npos);
  EXPECT_EQ(demo_source.find("ddgi_volume->enable_probe_classification = true;"), std::string::npos);
  EXPECT_NE(demo_source.find("ConfigureMaterial(point_light_right_material, glm::vec3(1.0f, 0.8f, 0.0f), 1.0f, "
                             "1.0f, 2.0f);"),
            std::string::npos);
  EXPECT_NE(demo_source.find("point_light_right->diffuse_brightness = 24.0f;"), std::string::npos);
  EXPECT_NE(demo_source.find("DisableImportedLightsRecursive(scene, sponza_entity);"), std::string::npos);
  EXPECT_NE(demo_source.find("DisableLightIfPresent<DirectionalLight>(scene, entity);"), std::string::npos);
  EXPECT_NE(demo_source.find("DisableLightIfPresent<PointLight>(scene, entity);"), std::string::npos);
  EXPECT_NE(demo_source.find("DisableLightIfPresent<SpotLight>(scene, entity);"), std::string::npos);
  EXPECT_NE(demo_app_source.find("ddgi_settings.runtime.ray_count != 64"), std::string::npos);
  EXPECT_NE(demo_app_source.find("ddgi_settings.debug.visualization_scale != 2.0f"), std::string::npos);
  EXPECT_NE(demo_app_source.find("volume->probe_counts != glm::ivec3(10, 6, 16)"), std::string::npos);
  EXPECT_NE(demo_app_source.find("volume->probe_spacing != glm::vec3(1.5f)"), std::string::npos);
  EXPECT_NE(demo_app_source.find("volume->volume_origin != glm::vec3(0.0f, 3.0f, 3.0f)"), std::string::npos);
  EXPECT_NE(demo_app_source.find("volume->relocation_distance != 0.25f"), std::string::npos);
  EXPECT_NE(demo_app_source.find("!volume->enable_probe_relocation || volume->enable_probe_classification"),
            std::string::npos);
}

TEST(DdgiVolume, ClampSettingsKeepsAuthoringValuesInSupportedRanges) {
  DdgiVolume volume;
  volume.probe_counts = {-4, 0, 512};
  volume.probe_spacing = {-1.0f, 0.01f, 20000.0f};
  volume.movement_type = 4;
  volume.relocation_distance = -1.0f;
  volume.random_ray_backface_threshold = -1.0f;
  volume.fixed_ray_backface_threshold = 2.0f;
  volume.probe_variability_threshold = 20.0f;
  volume.probe_variability_min_samples = -4;
  volume.warmup_trigger_conditions = 0xffff;
  volume.variability_reset_trigger_conditions = 0xffff;
  volume.max_visualized_probes = 0;
  volume.probe_visualization_size = -1.0f;
  volume.ClampSettings();

  EXPECT_EQ(volume.probe_counts, glm::ivec3(1, 1, 256));
  EXPECT_EQ(volume.probe_spacing, glm::vec3(0.05f, 0.05f, 10000.0f));
  EXPECT_EQ(volume.movement_type, static_cast<int>(DdgiVolumeMovementType::Scrolling));
  EXPECT_FLOAT_EQ(volume.relocation_distance, 0.0f);
  EXPECT_FLOAT_EQ(volume.random_ray_backface_threshold, 0.0f);
  EXPECT_FLOAT_EQ(volume.fixed_ray_backface_threshold, 1.0f);
  EXPECT_FLOAT_EQ(volume.probe_variability_threshold, 10.0f);
  EXPECT_EQ(volume.probe_variability_min_samples, 0);
  EXPECT_EQ(volume.warmup_trigger_conditions, DdgiVolumeTriggerConditionAll);
  EXPECT_EQ(volume.variability_reset_trigger_conditions, DdgiVolumeTriggerConditionAll);
  EXPECT_EQ(volume.max_visualized_probes, 1);
  EXPECT_FLOAT_EQ(volume.probe_visualization_size, 0.001f);
}

TEST(DdgiVolume, SceneEnvironmentDeserializesDdgiSettings) {
  Scene::Environment restored;
  restored.Deserialize(YAML::Load(R"(
ddgi_settings:
  runtime:
    enabled: true
    ray_count: 64
    warmup_frames: 12
    reset_conditions: 8
    indirect_intensity: 2.5
    distance_exponent: 42.0
    irradiance_threshold: 0.4
    brightness_threshold: 0.7
  volume_defaults:
    probe_counts: [5, 3, 7]
    probe_spacing: 2.0
    volume_origin: [1.0, 2.0, 3.0]
    movement_type: 1
    random_ray_backface_threshold: 0.2
    fixed_ray_backface_threshold: 0.4
    enable_probe_variability: false
    enable_probe_variability_gating: true
    probe_variability_threshold: 0.75
    probe_variability_min_samples: 32
  storage:
    max_probe_count: 1024
  debug:
    enabled: true
    visualize_probe_illumination: false
    probe_visualization_mode: 2
    probe_visualization_depth_mode: 1
    probe_visualization_radius: 0.25
    probe_visualization_intensity: 3.5
    probe_visualization_alpha: 0.4
    selected_probe_visualization_scale: 5.0
    selected_probe_index: 42
)"));
  const auto& restored_settings = restored.ddgi_settings;
  EXPECT_TRUE(restored_settings.runtime.enabled);
  EXPECT_EQ(restored_settings.runtime.ray_count, 64);
  EXPECT_EQ(restored_settings.runtime.warmup_frames, 12);
  EXPECT_FLOAT_EQ(restored_settings.runtime.indirect_intensity, 2.5f);
  EXPECT_FLOAT_EQ(restored_settings.runtime.distance_exponent, 42.0f);
  EXPECT_FLOAT_EQ(restored_settings.runtime.irradiance_threshold, 0.4f);
  EXPECT_FLOAT_EQ(restored_settings.runtime.brightness_threshold, 0.7f);
  EXPECT_EQ(restored_settings.volume_defaults.probe_counts, glm::ivec3(5, 3, 7));
  EXPECT_EQ(restored_settings.volume_defaults.probe_spacing, glm::vec3(2.0f));
  EXPECT_EQ(restored_settings.volume_defaults.volume_origin, glm::vec3(1.0f, 2.0f, 3.0f));
  EXPECT_EQ(restored_settings.volume_defaults.movement_type, static_cast<int>(DdgiVolumeMovementType::Scrolling));
  EXPECT_FLOAT_EQ(restored_settings.volume_defaults.random_ray_backface_threshold, 0.2f);
  EXPECT_FLOAT_EQ(restored_settings.volume_defaults.fixed_ray_backface_threshold, 0.4f);
  EXPECT_FALSE(restored_settings.volume_defaults.enable_probe_variability);
  EXPECT_TRUE(restored_settings.volume_defaults.enable_probe_variability_gating);
  EXPECT_FLOAT_EQ(restored_settings.volume_defaults.probe_variability_threshold, 0.75f);
  EXPECT_EQ(restored_settings.volume_defaults.probe_variability_min_samples, 32);
  EXPECT_EQ(restored_settings.storage.max_probe_count, 1024);
  EXPECT_TRUE(restored_settings.debug.enabled);
  EXPECT_FALSE(restored_settings.debug.visualize_probe_illumination);
  EXPECT_EQ(restored_settings.debug.probe_visualization_mode, 2);
  EXPECT_EQ(restored_settings.debug.probe_visualization_depth_mode, 1);
  EXPECT_FLOAT_EQ(restored_settings.debug.probe_visualization_radius, 0.25f);
  EXPECT_FLOAT_EQ(restored_settings.debug.probe_visualization_intensity, 3.5f);
  EXPECT_FLOAT_EQ(restored_settings.debug.probe_visualization_alpha, 0.4f);
  EXPECT_FLOAT_EQ(restored_settings.debug.selected_probe_visualization_scale, 5.0f);
  EXPECT_EQ(restored_settings.debug.selected_probe_index, 42);
}

TEST(DdgiVolume, SceneEnvironmentSerializesDdgiWarmupFramesWithoutGlobalResetPolicy) {
  const auto scene_source =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "src" / "Scene.cpp");
  ASSERT_FALSE(scene_source.empty());

  EXPECT_NE(scene_source.find("out << YAML::Key << \"warmup_frames\" << YAML::Value << "
                              "settings.runtime.warmup_frames;"),
            std::string::npos);
  EXPECT_EQ(scene_source.find("\"reset_conditions\""), std::string::npos);
}

TEST(DdgiVolume, SceneEnvironmentKeepsDefaultWarmupForLegacyYaml) {
  Scene::Environment restored;
  restored.Deserialize(YAML::Load(R"(
ddgi_settings:
  runtime:
    enabled: true
    ray_count: 64
)"));

  EXPECT_EQ(restored.ddgi_settings.runtime.warmup_frames, 16);
}

TEST(DdgiVolume, DdgiVolumeSerializesPerVolumeTriggerPolicies) {
  const auto application_source =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "src" / "Application.cpp");
  ASSERT_FALSE(application_source.empty());

  EXPECT_NE(application_source.find("out << YAML::Key << \"warmup_trigger_conditions\" << YAML::Value << "
                                    "volume.warmup_trigger_conditions;"),
            std::string::npos);
  EXPECT_NE(application_source.find("out << YAML::Key << \"variability_reset_trigger_conditions\" << YAML::Value"),
            std::string::npos);
  EXPECT_NE(application_source.find("volume.warmup_trigger_conditions = "
                                    "in[\"warmup_trigger_conditions\"].as<int>();"),
            std::string::npos);
  EXPECT_NE(application_source.find("volume.variability_reset_trigger_conditions = "
                                    "in[\"variability_reset_trigger_conditions\"].as<int>();"),
            std::string::npos);
  EXPECT_NE(application_source.find("volume.ClampSettings();"), std::string::npos);
}

TEST(DdgiVolume, SceneEnvironmentConvertsLegacyFirstProbeOffsetToVolumeOrigin) {
  Scene::Environment restored;
  restored.Deserialize(YAML::Load(R"(
ddgi_settings:
  volume_defaults:
    probe_counts: [3, 2, 1]
    probe_spacing: 2.0
    volume_offset: [1.0, 2.0, 3.0]
)"));

  const auto& defaults = restored.ddgi_settings.volume_defaults;
  EXPECT_EQ(defaults.probe_counts, glm::ivec3(3, 2, 1));
  EXPECT_EQ(defaults.probe_spacing, glm::vec3(2.0f));
  EXPECT_EQ(defaults.volume_origin, glm::vec3(3.0f, 3.0f, 3.0f));
}

TEST(DdgiVolume, SceneEnvironmentReadsLegacyVectorProbeSpacing) {
  Scene::Environment restored;
  restored.Deserialize(YAML::Load(R"(
ddgi_settings:
  volume_defaults:
    probe_spacing: [2.0, 3.0, 4.0]
)"));

  EXPECT_EQ(restored.ddgi_settings.volume_defaults.probe_spacing, glm::vec3(2.0f, 3.0f, 4.0f));
}

TEST(DdgiVolume, RenderLayerProbeDebugCoordinatesUseXFastestProbeOrder) {
  RenderLayer::DdgiSettings settings;
  settings.volume_defaults.probe_counts = {4, 3, 2};
  settings.storage.atlas_probe_columns = 5;
  settings.storage.irradiance_tile_resolution = 8;
  settings.debug.selected_probe_index = 17;

  const auto coordinates = RenderLayer::CalculateDdgiProbeDebugCoordinates(
      settings, static_cast<uint32_t>(settings.storage.irradiance_tile_resolution));

  EXPECT_EQ(RenderLayer::GetDdgiProbeCount(settings.volume_defaults.probe_counts), 24u);
  EXPECT_EQ(coordinates.probe_index, 17u);
  EXPECT_EQ(coordinates.grid_index, glm::uvec3(1, 1, 1));
  EXPECT_EQ(coordinates.atlas_layout.probe_count, 24u);
  EXPECT_EQ(coordinates.atlas_layout.tile_resolution, 8u);
  EXPECT_EQ(coordinates.atlas_layout.columns, 5u);
  EXPECT_EQ(coordinates.atlas_layout.rows, 5u);
  EXPECT_EQ(coordinates.atlas_layout.resolution, glm::uvec2(50, 50));
  EXPECT_EQ(coordinates.atlas_tile_offset, glm::uvec2(20, 30));
}

TEST(DdgiVolume, RenderLayerProbeDebugCoordinatesClampInvalidInputs) {
  RenderLayer::DdgiSettings settings;
  settings.volume_defaults.probe_counts = {0, -2, 2};
  settings.storage.atlas_probe_columns = 0;
  settings.debug.selected_probe_index = 99;

  const auto coordinates = RenderLayer::CalculateDdgiProbeDebugCoordinates(settings, 0);

  EXPECT_EQ(RenderLayer::GetDdgiProbeCount(settings.volume_defaults.probe_counts), 2u);
  EXPECT_EQ(coordinates.probe_index, 1u);
  EXPECT_EQ(coordinates.grid_index, glm::uvec3(0, 0, 1));
  EXPECT_EQ(coordinates.atlas_layout.probe_count, 2u);
  EXPECT_EQ(coordinates.atlas_layout.tile_resolution, 1u);
  EXPECT_EQ(coordinates.atlas_layout.columns, 1u);
  EXPECT_EQ(coordinates.atlas_layout.rows, 2u);
  EXPECT_EQ(coordinates.atlas_layout.resolution, glm::uvec2(3, 6));
  EXPECT_EQ(coordinates.atlas_tile_offset, glm::uvec2(0, 3));
}

TEST(DdgiVolume, RenderLayerFrameResourceLayoutUsesStorageProbeCapacity) {
  RenderLayer::DdgiSettings settings;
  settings.volume_defaults.probe_counts = {8, 8, 8};
  settings.storage.max_probe_count = 32;
  settings.storage.atlas_probe_columns = 6;
  settings.storage.irradiance_tile_resolution = 8;
  settings.storage.visibility_tile_resolution = 10;
  settings.runtime.ray_count = 64;
  settings.debug.selected_probe_index = 50;

  const auto coordinates = RenderLayer::CalculateDdgiProbeDebugCoordinates(
      settings, static_cast<uint32_t>(settings.storage.irradiance_tile_resolution));
  const auto layout = RenderLayer::CalculateDdgiFrameResourceLayout(settings);

  EXPECT_EQ(RenderLayer::GetDdgiProbeCount(settings.volume_defaults.probe_counts), 512u);
  EXPECT_EQ(RenderLayer::GetDdgiAllocatedProbeCount(settings), 32u);
  EXPECT_EQ(coordinates.probe_index, 31u);
  EXPECT_EQ(coordinates.grid_index, glm::uvec3(7, 3, 0));
  EXPECT_EQ(coordinates.atlas_tile_offset, glm::uvec2(10, 50));
  EXPECT_EQ(layout.probe_count, 32u);
  EXPECT_EQ(layout.irradiance_atlas.resolution, glm::uvec2(60, 60));
  EXPECT_EQ(layout.visibility_atlas.resolution, glm::uvec2(72, 72));
  EXPECT_EQ(layout.variability_atlas.resolution, glm::uvec2(48, 48));
  EXPECT_EQ(layout.variability_reduction_extent, glm::uvec2(3, 3));
  EXPECT_EQ(layout.probe_metadata_byte_size, 32ull * sizeof(glm::vec4) * 3ull);
  EXPECT_EQ(layout.probe_state_byte_size, 32ull * sizeof(glm::vec4));
  EXPECT_EQ(layout.probe_update_index_byte_size, 32ull * sizeof(uint32_t));
  EXPECT_EQ(layout.ray_output_byte_size, 32ull * 64ull * sizeof(PointCloudSample));
  EXPECT_EQ(layout.variability_atlas_byte_size, 48ull * 48ull * sizeof(uint16_t));
  EXPECT_EQ(layout.variability_reduction_byte_size, 3ull * 3ull * sizeof(glm::vec2));
  EXPECT_EQ(layout.variability_readback_byte_size, sizeof(glm::vec2));
}

TEST(DdgiVolume, RenderLayerFrameResourceLayoutUsesActiveVolumeProbeCount) {
  RenderLayer::DdgiSettings settings;
  settings.volume_defaults.probe_counts = {4, 4, 4};
  settings.storage.max_probe_count = 4096;
  settings.storage.atlas_probe_columns = 16;
  settings.storage.irradiance_tile_resolution = 8;
  settings.storage.visibility_tile_resolution = 10;
  settings.runtime.ray_count = 128;

  const auto layout = RenderLayer::CalculateDdgiFrameResourceLayout(settings, 252);

  EXPECT_EQ(RenderLayer::GetDdgiAllocatedProbeCount(settings), 64u);
  EXPECT_EQ(RenderLayer::GetDdgiAllocatedProbeCount(settings, 252), 252u);
  EXPECT_EQ(layout.probe_count, 252u);
  EXPECT_EQ(layout.irradiance_atlas.rows, 16u);
  EXPECT_EQ(layout.irradiance_atlas.resolution, glm::uvec2(160, 160));
  EXPECT_EQ(layout.visibility_atlas.resolution, glm::uvec2(192, 192));
  EXPECT_EQ(layout.variability_atlas.resolution, glm::uvec2(128, 128));
  EXPECT_EQ(layout.variability_reduction_extent, glm::uvec2(8, 8));
  EXPECT_EQ(layout.probe_metadata_byte_size, 252ull * sizeof(glm::vec4) * 3ull);
  EXPECT_EQ(layout.probe_state_byte_size, 252ull * sizeof(glm::vec4));
  EXPECT_EQ(layout.probe_update_index_byte_size, 252ull * sizeof(uint32_t));
  EXPECT_EQ(layout.ray_output_byte_size, 252ull * 128ull * sizeof(PointCloudSample));
}

TEST(DdgiVolume, ProbeVisualizationFadesNearCameraWithAlphaBlend) {
  const auto shader_root = std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "Internals" /
                           "DefaultResources" / "Shaders" / "Graphics";
  const auto vertex_source = ReadTextFile(shader_root / "Vertex" / "DDGI" / "DDGIProbeVisualization.vert");
  const auto fragment_source = ReadTextFile(shader_root / "Fragment" / "DDGI" / "DDGIProbeVisualization.frag");
  ASSERT_FALSE(vertex_source.empty());
  ASSERT_FALSE(fragment_source.empty());

  EXPECT_NE(fragment_source.find("const float probe_inactive = clamp(fs_in.State.w, 0.0f, 1.0f);"), std::string::npos);
  EXPECT_NE(fragment_source.find("vec3 EE_DDGI_DECODE_DEBUG_IRRADIANCE"), std::string::npos);
  EXPECT_NE(fragment_source.find("irradiance_gamma * 0.5f"), std::string::npos);
  EXPECT_NE(fragment_source.find("EE_DDGI_TONEMAP_DEBUG_COLOR"), std::string::npos);
  EXPECT_NE(fragment_source.find("EE_DDGI_IRRADIANCE_DEBUG_COLOR(atlas_irradiance"), std::string::npos);
  EXPECT_NE(fragment_source.find("if (probe_inactive > 0.5f)"), std::string::npos);
  EXPECT_NE(vertex_source.find("float EE_DDGI_PROBE_CAMERA_FADE_ALPHA"), std::string::npos);
  EXPECT_NE(vertex_source.find("distance(probe_position, EE_CAMERA_POSITION(int(camera_index)))"), std::string::npos);
  EXPECT_NE(vertex_source.find("return smoothstep(fade_start, fade_end, camera_distance);"), std::string::npos);
  EXPECT_NE(vertex_source.find("vs_out.CameraFade = EE_DDGI_PROBE_CAMERA_FADE_ALPHA"), std::string::npos);
  EXPECT_NE(fragment_source.find("clamp(fs_in.CameraFade, 0.0f, 1.0f)"), std::string::npos);
  EXPECT_NE(fragment_source.find("FragColor = vec4(color, alpha);"), std::string::npos);
  EXPECT_EQ(fragment_source.find("FragColor = vec4(color, 1.0f);"), std::string::npos);
  EXPECT_EQ(fragment_source.find("const float probe_active = fs_in.State.w"), std::string::npos);
  EXPECT_EQ(fragment_source.find("hit_alpha"), std::string::npos);

  const auto pass_path = std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "src" / "RenderPasses" /
                         "DdgiProbeVisualizationPass.cpp";
  const auto pass_source = ReadTextFile(pass_path);
  ASSERT_FALSE(pass_source.empty());

  EXPECT_NE(pass_source.find("parameters.pipeline->states.color_blend_attachment_states[0].blendEnable = VK_TRUE;"),
            std::string::npos);
  EXPECT_NE(pass_source.find("parameters.pipeline->states.depth_write = false;"), std::string::npos);
}

TEST(DdgiVolume, DdgiDiffuseUsesRtxgiStyleEnergyEncoding) {
  const auto shader_root =
      std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "Internals" / "DefaultResources" / "Shaders";
  const auto lighting_source = ReadTextFile(shader_root / "Includes" / "Lighting.glsl");
  const auto probe_update_source = ReadTextFile(shader_root / "Compute" / "DDGIProbeUpdate.comp");
  const auto raygen_source = ReadTextFile(shader_root / "RayTracing" / "RayGen" / "DDGIProbeDiagnostics.rgen");
  const auto closest_hit_source =
      ReadTextFile(shader_root / "RayTracing" / "ClosestHit" / "DDGIProbeDiagnostics.rchit");
  const auto ddgi_helper_source = ReadTextFile(shader_root / "Includes" / "DDGI.glsl");
  const auto render_layer_source =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "src" / "RenderLayer.cpp");
  ASSERT_FALSE(lighting_source.empty());
  ASSERT_FALSE(probe_update_source.empty());
  ASSERT_FALSE(raygen_source.empty());
  ASSERT_FALSE(closest_hit_source.empty());
  ASSERT_FALSE(ddgi_helper_source.empty());
  ASSERT_FALSE(render_layer_source.empty());

  EXPECT_NE(probe_update_source.find("1.0f / (2.0f * max(weight_sum, epsilon))"), std::string::npos);
  EXPECT_NE(probe_update_source.find("EE_DDGI_PROBE_MAX_VISIBILITY_DISTANCE()"), std::string::npos);
  EXPECT_NE(probe_update_source.find("vec2(first_moment, second_moment) * (1.0f / (2.0f * weight_sum))"),
            std::string::npos);
  EXPECT_NE(probe_update_source.find("return vec4(irradiance, 1.0f);"), std::string::npos);
  EXPECT_EQ(probe_update_source.find("classification_enabled && classification_inside_geometry ? 1.0f : 0.0f"),
            std::string::npos);
  EXPECT_NE(probe_update_source.find("const bool fixed_rays_enabled = relocation_enabled || classification_enabled;"),
            std::string::npos);
  EXPECT_EQ(probe_update_source.find("!classification_has_nearby_geometry"), std::string::npos);
  EXPECT_EQ(raygen_source.find("uint EE_DDGI_RANDOM_ROTATION_SEED"), std::string::npos);
  EXPECT_EQ(raygen_source.find("EE_RANGED_RANDOM"), std::string::npos);
  EXPECT_NE(raygen_source.find("vec4 EE_DDGI_PROBE_RAY_ROTATION()"), std::string::npos);
  EXPECT_NE(raygen_source.find("EE_DDGI_ROTATE_BY_CONJUGATE_QUATERNION"), std::string::npos);
  EXPECT_NE(raygen_source.find("origin, 0.0f, direction"), std::string::npos);
  EXPECT_NE(raygen_source.find("hit_value.seed = fixed_ray ? EE_DDGI_FIXED_RAY_PAYLOAD_FLAG : 0u;"), std::string::npos);
  EXPECT_NE(raygen_source.find("if (inactive_probe && !fixed_ray)"), std::string::npos);
  EXPECT_NE(ddgi_helper_source.find("const float golden_ratio_fraction"), std::string::npos);
  EXPECT_NE(ddgi_helper_source.find("fract(float(sample_index) * golden_ratio_fraction)"), std::string::npos);
  EXPECT_NE(ddgi_helper_source.find("const uint EE_DDGI_FIXED_RAY_PAYLOAD_FLAG = 1u;"), std::string::npos);
  EXPECT_NE(ddgi_helper_source.find("return fixed_rays_enabled && ray_count > 1u ? min(32u, ray_count - 1u) : 0u;"),
            std::string::npos);
  EXPECT_NE(ddgi_helper_source.find("EE_DDGI_SIGN_NOT_ZERO(direction.xy)"), std::string::npos);
  EXPECT_NE(lighting_source.find("irradiance_gamma * 0.5f"), std::string::npos);
  EXPECT_NE(lighting_source.find("layout(set = EE_PER_GROUP_SET, binding = 19) readonly buffer "
                                 "EE_DDGI_PROBE_STATE_BLOCK"),
            std::string::npos);
  EXPECT_NE(lighting_source.find("const vec4 probe_state = EE_DDGI_PROBE_STATE[probe_index];"), std::string::npos);
  EXPECT_NE(lighting_source.find("1.0f - clamp(probe_state.w, 0.0f, 1.0f)"), std::string::npos);
  EXPECT_NE(lighting_source.find("probe_state.xyz"), std::string::npos);
  EXPECT_EQ(lighting_source.find("probe_active = clamp(visibility_sample.w"), std::string::npos);
  EXPECT_NE(lighting_source.find("EE_DDGI_CHEBYSHEV_VISIBILITY(visibility_sample.rg, biased_probe_distance"),
            std::string::npos);
  EXPECT_EQ(lighting_source.find("mix(1.0f, visibility_weight, final_visibility_strength)"), std::string::npos);
  EXPECT_NE(
      lighting_source.find("EE_FUNC_CALCULATE_DDGI_DIFFUSE(vec3 albedo, vec3 normal, vec3 viewDir, vec3 fragPos)"),
      std::string::npos);
  EXPECT_NE(lighting_source.find("EE_RENDER_INFO.ddgi_volume_parameters.w"), std::string::npos);
  EXPECT_EQ(lighting_source.find("normalized_probe_distance"), std::string::npos);
  EXPECT_NE(lighting_source.find("EE_DDGI_PROBE_COORDINATE(fragPos, EE_RENDER_INFO.ddgi_first_probe.xyz"),
            std::string::npos);
  EXPECT_NE(lighting_source.find("EE_DDGI_VOLUME_BLEND_WEIGHT(volume_probe_coordinate"), std::string::npos);
  EXPECT_NE(lighting_source.find("EE_DDGI_PROBE_COORDINATE(biased_frag_pos"), std::string::npos);
  EXPECT_NE(lighting_source.find("clamp(floor(biased_probe_coordinate), vec3(0.0f), max_probe_grid)"),
            std::string::npos);
  EXPECT_NE(lighting_source.find("EE_DDGI_AXIS_COORDINATE(base_probe_to_biased_position"), std::string::npos);
  EXPECT_NE(lighting_source.find("max(vec3(0.001f), mix(vec3(1.0f) - probe_fraction"), std::string::npos);
  EXPECT_NE(lighting_source.find("visibility_weight = max(0.000001f, visibility_weight);"), std::string::npos);
  EXPECT_EQ(lighting_source.find("probe_validity"), std::string::npos);
  EXPECT_EQ(lighting_source.find("clamp(irradiance.a"), std::string::npos);
  EXPECT_NE(ddgi_helper_source.find("const vec2 moments = max(2.0f * half_moments"), std::string::npos);
  EXPECT_NE(ddgi_helper_source.find("return max(visibility * visibility * visibility, 0.0f);"), std::string::npos);
  EXPECT_NE(ddgi_helper_source.find("view_direction * max(view_bias, 0.0f)"), std::string::npos);
  EXPECT_NE(ddgi_helper_source.find("const vec3 outside_distance = max(lower_distance, upper_distance);"),
            std::string::npos);
  EXPECT_NE(ddgi_helper_source.find("if (inside_volume)"), std::string::npos);
  EXPECT_NE(lighting_source.find("weight_sum += sample_weight;"), std::string::npos);
  EXPECT_NE(lighting_source.find("diffuse *= diffuse * (2.0f * EE_DDGI_PI);"), std::string::npos);
  EXPECT_NE(lighting_source.find("return albedo / EE_DDGI_PI * diffuse * intensity * volume_blend_weight;"),
            std::string::npos);
  EXPECT_NE(closest_hit_source.find("EE_DDGI_LAMBERT_IRRADIANCE(albedo, light.diffuse.rgb"), std::string::npos);
  EXPECT_NE(closest_hit_source.find("EE_DDGI_PROBE_COORDINATE(position, EE_RENDER_INFO.ddgi_first_probe.xyz"),
            std::string::npos);
  EXPECT_NE(closest_hit_source.find("EE_DDGI_VOLUME_BLEND_WEIGHT(volume_probe_coordinate"), std::string::npos);
  EXPECT_NE(closest_hit_source.find("EE_DDGI_PROBE_COORDINATE(biased_position"), std::string::npos);
  EXPECT_NE(closest_hit_source.find("clamp(floor(biased_probe_coordinate), vec3(0.0f), max_probe_grid)"),
            std::string::npos);
  EXPECT_NE(closest_hit_source.find("EE_DDGI_AXIS_COORDINATE(base_probe_to_biased_position"), std::string::npos);
  EXPECT_NE(closest_hit_source.find("visibility_weight = max(0.000001f, visibility_weight);"), std::string::npos);
  EXPECT_EQ(closest_hit_source.find("probe_validity"), std::string::npos);
  EXPECT_EQ(closest_hit_source.find("clamp(irradiance.a"), std::string::npos);
  EXPECT_NE(closest_hit_source.find("return albedo / EE_DDGI_PI * diffuse * intensity * volume_blend_weight;"),
            std::string::npos);
  EXPECT_NE(closest_hit_source.find("#include \"GltfRasterMaterial.glsl\""), std::string::npos);
  EXPECT_NE(closest_hit_source.find("EE_EVALUATE_GLTF_RASTER_SURFACE(material_index"), std::string::npos);
  EXPECT_NE(closest_hit_source.find("EE_EVALUATE_GLTF_RASTER_NORMAL(material_index"), std::string::npos);
  EXPECT_NE(closest_hit_source.find("const vec3 emissive_radiance = surface.emissive;"), std::string::npos);
  EXPECT_NE(closest_hit_source.find("emissive_radiance + EE_DDGI_DIRECT_IRRADIANCE"), std::string::npos);
  EXPECT_NE(closest_hit_source.find("layout(set = 2, binding = 1) readonly buffer EE_DDGI_PROBE_STATE_BLOCK"),
            std::string::npos);
  EXPECT_NE(closest_hit_source.find("const vec4 probe_state = EE_DDGI_PROBE_STATE[probe_index];"), std::string::npos);
  EXPECT_NE(closest_hit_source.find("1.0f - clamp(probe_state.w, 0.0f, 1.0f)"), std::string::npos);
  EXPECT_NE(closest_hit_source.find("probe_state.xyz"), std::string::npos);
  EXPECT_EQ(closest_hit_source.find("probe_active = clamp(visibility_sample.w"), std::string::npos);
  EXPECT_NE(closest_hit_source.find("material.double_sided"), std::string::npos);
  EXPECT_EQ(closest_hit_source.find(std::string("material") + "_properties.cull_mode"), std::string::npos);
  EXPECT_NE(closest_hit_source.find("const bool backface_hit = ray_backface_hit || hit_face_is_culled;"),
            std::string::npos);
  EXPECT_EQ(closest_hit_source.find("const bool backface_hit = false;"), std::string::npos);
  EXPECT_NE(closest_hit_source.find("const bool fixed_probe_ray = hit_value.seed == EE_DDGI_FIXED_RAY_PAYLOAD_FLAG;"),
            std::string::npos);
  EXPECT_NE(closest_hit_source.find("backface_hit || fixed_probe_ray"), std::string::npos);
  EXPECT_NE(closest_hit_source.find("vec4(max(frontface_radiance, vec3(0.0f))"), std::string::npos);
  EXPECT_EQ(closest_hit_source.find("clamp(frontface_radiance, vec3(0.0f), vec3(1.0f))"), std::string::npos);
  EXPECT_NE(closest_hit_source.find("-normalize(gl_WorldRayDirectionEXT)"), std::string::npos);
  EXPECT_NE(closest_hit_source.find("max(vec3(0.001f), mix(vec3(1.0f) - probe_fraction"), std::string::npos);
  EXPECT_NE(render_layer_source.find("CreateDdgiProbeRayRotationQuaternion"), std::string::npos);
  EXPECT_NE(render_layer_source.find("source.selected_volume_index"), std::string::npos);
  EXPECT_NE(render_layer_source.find("push_constant.first_probe = glm::vec4(source.first_probe, ray_rotation.x);"),
            std::string::npos);
  EXPECT_NE(render_layer_source.find("push_constant.probe_step_z = glm::vec4(source.probe_step_z, ray_rotation.w);"),
            std::string::npos);
  EXPECT_NE(render_layer_source.find("glm::max(settings.runtime.view_bias, 0.0f)"), std::string::npos);
  EXPECT_NE(render_layer_source.find("update_push_constant.update_parameters.w"), std::string::npos);
  EXPECT_NE(render_layer_source.find("lighting_layout_->PushDescriptorBinding(19, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER"),
            std::string::npos);
  EXPECT_NE(render_layer_source.find("UpdateBufferDescriptorBinding(19"), std::string::npos);
  EXPECT_NE(render_layer_source.find("CreateDdgiFallbackProbeStateBuffer"), std::string::npos);
}

TEST(DdgiVolume, DdgiProbeMissRaysSampleSceneEnvironment) {
  const auto shader_root =
      std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "Internals" / "DefaultResources" / "Shaders";
  const auto miss_source = ReadTextFile(shader_root / "RayTracing" / "Miss" / "DDGIProbeDiagnostics.rmiss");
  const auto render_layer_source =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "src" / "RenderLayer.cpp");
  ASSERT_FALSE(miss_source.empty());
  ASSERT_FALSE(render_layer_source.empty());

  EXPECT_NE(miss_source.find("uvec4 probe_offset_and_update_count"), std::string::npos);
  EXPECT_NE(miss_source.find("const uint environment_index = probe_offset_and_update_count.w;"), std::string::npos);
  EXPECT_NE(miss_source.find("textureLod(EE_CUBEMAPS[int(environment_index)]"), std::string::npos);
  EXPECT_NE(miss_source.find("EE_ENVIRONMENT.gamma"), std::string::npos);
  EXPECT_NE(miss_source.find("EE_ENVIRONMENT.light_intensity"), std::string::npos);

  EXPECT_NE(render_layer_source.find("uint32_t GetDdgiEnvironmentCubemapIndex"), std::string::npos);
  EXPECT_NE(render_layer_source.find("scene->environment.GetReflectionProbe(position)"), std::string::npos);
  EXPECT_NE(render_layer_source.find("Resources::GetInstance().GetDefaultEnvironmentalMap()"), std::string::npos);
  EXPECT_NE(render_layer_source.find("environment_cubemap_index"), std::string::npos);
  EXPECT_NE(render_layer_source.find("skip_inactive_probe_trace ? 1u : 0u, environment_cubemap_index"),
            std::string::npos);
}

TEST(DdgiVolume, DdgiProbeUpdateUsesRtxgiBlendWithoutTemporalClamp) {
  const auto shader_root =
      std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "Internals" / "DefaultResources" / "Shaders";
  const auto probe_update_source = ReadTextFile(shader_root / "Compute" / "DDGIProbeUpdate.comp");
  const auto render_layer_source =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "src" / "RenderLayer.cpp");
  ASSERT_FALSE(probe_update_source.empty());
  ASSERT_FALSE(render_layer_source.empty());

  EXPECT_EQ(probe_update_source.find("EE_DDGI_TEMPORAL_CLAMP"), std::string::npos);
  EXPECT_NE(probe_update_source.find("EE_DDGI_BLEND_IRRADIANCE_HISTORY"), std::string::npos);
  EXPECT_NE(probe_update_source.find("layout(set = 1, binding = 6, r16f) uniform image2D "
                                     "EE_DDGI_PROBE_VARIABILITY_ATLAS;"),
            std::string::npos);
  EXPECT_NE(probe_update_source.find("hysteresis = max(0.0f, hysteresis - 0.75f);"), std::string::npos);
  EXPECT_NE(probe_update_source.find("delta *= 0.25f;"), std::string::npos);
  EXPECT_NE(probe_update_source.find("const float min_darkening_step = 1.0f / 1024.0f;"), std::string::npos);
  EXPECT_NE(probe_update_source.find("return vec4(irradiance, 1.0f);"), std::string::npos);
  EXPECT_EQ(probe_update_source.find("weight_sum > epsilon ? 1.0f : 0.0f"), std::string::npos);
  EXPECT_NE(probe_update_source.find("if (directional_irradiance.a <= 0.0f)"), std::string::npos);
  EXPECT_NE(probe_update_source.find("EE_DDGI_BLEND_IRRADIANCE_HISTORY(directional_irradiance, "
                                     "history_irradiance, history_weight);"),
            std::string::npos);
  EXPECT_NE(probe_update_source.find("const vec2 visibility_moments = mix(directional_visibility, "
                                     "history_visibility, history_weight);"),
            std::string::npos);
  EXPECT_NE(probe_update_source.find("const float coefficient_of_variation"), std::string::npos);
  EXPECT_NE(probe_update_source.find("const vec3 irradiance_sample = directional_irradiance.rgb;"), std::string::npos);
  EXPECT_NE(probe_update_source.find("const vec3 irradiance_sigma2"), std::string::npos);
  EXPECT_NE(probe_update_source.find("const float luminance_sigma2 = EE_DDGI_LUMINANCE(irradiance_sigma2);"),
            std::string::npos);
  EXPECT_NE(probe_update_source.find("sqrt(luminance_sigma2) / mean_luminance"), std::string::npos);
  EXPECT_EQ(probe_update_source.find("sqrt(max(EE_DDGI_LUMINANCE(variance), 0.0f))"), std::string::npos);
  EXPECT_NE(probe_update_source.find("imageStore(EE_DDGI_PROBE_VARIABILITY_ATLAS"), std::string::npos);
  const auto previous_state_read =
      probe_update_source.find("const vec4 previous_state = EE_DDGI_PROBE_STATE[physical_probe_index];");
  const auto previous_state_blend_guard =
      probe_update_source.find("const bool blend_previous_probe_state = previous_inactive <= 0.5f;");
  const auto irradiance_blend = probe_update_source.find(
      "EE_DDGI_BLEND_IRRADIANCE_HISTORY(directional_irradiance, history_irradiance, "
      "history_weight);");
  ASSERT_NE(previous_state_read, std::string::npos);
  ASSERT_NE(previous_state_blend_guard, std::string::npos);
  ASSERT_NE(irradiance_blend, std::string::npos);
  EXPECT_LT(previous_state_read, previous_state_blend_guard);
  EXPECT_LT(previous_state_blend_guard, irradiance_blend);
  EXPECT_NE(probe_update_source.find("layout(set = 1, binding = 4) readonly buffer EE_DDGI_PROBE_STATE_BLOCK"),
            std::string::npos);
  EXPECT_NE(probe_update_source.find("vec4(relocation_offset, 1.0f - previous_inactive)"), std::string::npos);
  EXPECT_EQ(probe_update_source.find("EE_DDGI_PROBE_STATE[physical_probe_index] ="), std::string::npos);
  EXPECT_EQ(probe_update_source.find("state_value"), std::string::npos);
  EXPECT_EQ(probe_update_source.find("classification_inside_geometry"), std::string::npos);
  EXPECT_EQ(probe_update_source.find("relocation_candidate"), std::string::npos);
  EXPECT_EQ(probe_update_source.find("directional_irradiance *= 1.0f - inactive;"), std::string::npos);
  EXPECT_EQ(probe_update_source.find("history_visibility = EE_DDGI_TEMPORAL_CLAMP"), std::string::npos);
  EXPECT_NE(render_layer_source.find("settings.runtime.distance_exponent"), std::string::npos);
  EXPECT_NE(render_layer_source.find("settings.runtime.irradiance_threshold"), std::string::npos);
  EXPECT_NE(render_layer_source.find("settings.runtime.brightness_threshold"), std::string::npos);
  EXPECT_NE(render_layer_source.find("CalculateDdgiUpdateBrightnessThreshold"), std::string::npos);
  EXPECT_NE(render_layer_source.find("ddgi_update_brightness_threshold"), std::string::npos);
  EXPECT_EQ(render_layer_source.find("temporal_clamp"), std::string::npos);
}

TEST(DdgiVolume, DdgiProbeRelocationUsesStandaloneRtxgiStylePass) {
  const auto shader_root =
      std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "Internals" / "DefaultResources" / "Shaders";
  const auto relocation_source = ReadTextFile(shader_root / "Compute" / "DDGIProbeRelocation.comp");
  const auto update_source = ReadTextFile(shader_root / "Compute" / "DDGIProbeUpdate.comp");
  const auto render_layer_source =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "src" / "RenderLayer.cpp");
  const auto relocation_pass_source = ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" /
                                                   "src" / "RenderPasses" / "DdgiProbeRelocationPass.cpp");
  ASSERT_FALSE(relocation_source.empty());
  ASSERT_FALSE(update_source.empty());
  ASSERT_FALSE(render_layer_source.empty());
  ASSERT_FALSE(relocation_pass_source.empty());

  EXPECT_NE(relocation_source.find("layout(local_size_x = 32"), std::string::npos);
  EXPECT_NE(relocation_source.find("const bool reset_offsets = probe_count_ray_count_and_flags.w != 0u;"),
            std::string::npos);
  EXPECT_NE(relocation_source.find("EE_DDGI_PROBE_STATE[probe_index].xyz = vec3(0.0f);"), std::string::npos);
  EXPECT_NE(relocation_source.find("EE_DDGI_FIXED_RAY_COUNT(ray_count, true)"), std::string::npos);
  EXPECT_NE(relocation_source.find("fixed_ray_backface_threshold"), std::string::npos);
  EXPECT_NE(relocation_source.find("hit_distance *= 5.0f;"), std::string::npos);
  EXPECT_NE(relocation_source.find("EE_DDGI_RELOCATION_OFFSET_INSIDE_VOXEL"), std::string::npos);
  EXPECT_NE(relocation_source.find("EE_DDGI_PROBE_STATE[physical_probe_index].xyz = relocation_offset"),
            std::string::npos);
  EXPECT_EQ(relocation_source.find("EE_DDGI_PROBE_STATE[physical_probe_index].w"), std::string::npos);
  EXPECT_EQ(update_source.find("fixed_ray_backface_threshold"), std::string::npos);
  EXPECT_NE(render_layer_source.find("DDGIProbeRelocation.comp"), std::string::npos);
  EXPECT_NE(render_layer_source.find("ddgi_frame_probe_relocation_reset_ = reset_probe_state;"), std::string::npos);
  EXPECT_NE(render_layer_source.find("ddgi_frame_probe_relocation_enabled_ = ddgi_ray_source.enable_probe_relocation;"),
            std::string::npos);
  EXPECT_NE(render_layer_source.find("DdgiProbeRelocationPass::CreateDescriptor()"), std::string::npos);
  EXPECT_NE(relocation_pass_source.find("RenderPassNames::ddgi_probe_relocation"), std::string::npos);
  EXPECT_NE(relocation_pass_source.find("descriptor.dependencies = {RenderPassNames::ddgi_probe_update};"),
            std::string::npos);
  EXPECT_NE(relocation_pass_source.find("RenderResourceNames::frame_ddgi_probe_state, RenderResourceUsage::ReadWrite"),
            std::string::npos);
  EXPECT_EQ(relocation_source.find(".hlsl"), std::string::npos);
}

TEST(DdgiVolume, DdgiProbeClassificationUsesStandaloneRtxgiStylePass) {
  const auto shader_root =
      std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "Internals" / "DefaultResources" / "Shaders";
  const auto classification_source = ReadTextFile(shader_root / "Compute" / "DDGIProbeClassification.comp");
  const auto update_source = ReadTextFile(shader_root / "Compute" / "DDGIProbeUpdate.comp");
  const auto render_layer_source =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "src" / "RenderLayer.cpp");
  const auto classification_pass_source =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "src" / "RenderPasses" /
                   "DdgiProbeClassificationPass.cpp");
  ASSERT_FALSE(classification_source.empty());
  ASSERT_FALSE(update_source.empty());
  ASSERT_FALSE(render_layer_source.empty());
  ASSERT_FALSE(classification_pass_source.empty());

  EXPECT_NE(classification_source.find("layout(local_size_x = 32"), std::string::npos);
  EXPECT_NE(classification_source.find("const bool reset_classification = probe_count_ray_count_and_flags.w != 0u;"),
            std::string::npos);
  EXPECT_NE(classification_source.find("EE_DDGI_PROBE_STATE[probe_index].w = 0.0f;"), std::string::npos);
  EXPECT_NE(classification_source.find("EE_DDGI_FIXED_RAY_COUNT(ray_count, true)"), std::string::npos);
  EXPECT_NE(classification_source.find("backface_count / float(fixed_ray_count) > fixed_ray_backface_threshold"),
            std::string::npos);
  EXPECT_NE(classification_source.find("EE_DDGI_VOXEL_PLANE_DISTANCE"), std::string::npos);
  EXPECT_NE(classification_source.find("EE_DDGI_PROBE_STATE[physical_probe_index].w = 1.0f;"), std::string::npos);
  EXPECT_NE(classification_source.find("EE_DDGI_PROBE_STATE[physical_probe_index].w = 0.0f;"), std::string::npos);
  EXPECT_EQ(classification_source.find("EE_DDGI_PROBE_STATE[physical_probe_index].xyz"), std::string::npos);
  EXPECT_EQ(update_source.find("classification_inside_geometry"), std::string::npos);
  EXPECT_EQ(update_source.find("!classification_has_nearby_geometry"), std::string::npos);
  EXPECT_NE(render_layer_source.find("DDGIProbeClassification.comp"), std::string::npos);
  EXPECT_NE(render_layer_source.find("ddgi_frame_probe_classification_reset_ = reset_probe_state;"), std::string::npos);
  EXPECT_NE(render_layer_source.find(
                "ddgi_frame_probe_classification_enabled_ = ddgi_ray_source.enable_probe_classification;"),
            std::string::npos);
  EXPECT_NE(render_layer_source.find("DdgiProbeClassificationPass::CreateDescriptor()"), std::string::npos);
  EXPECT_NE(classification_pass_source.find("RenderPassNames::ddgi_probe_classification"), std::string::npos);
  EXPECT_NE(classification_pass_source.find("descriptor.dependencies = {RenderPassNames::ddgi_probe_update};"),
            std::string::npos);
  EXPECT_NE(
      classification_pass_source.find("RenderResourceNames::frame_ddgi_probe_state, RenderResourceUsage::ReadWrite"),
      std::string::npos);
  EXPECT_EQ(classification_source.find(".hlsl"), std::string::npos);
}

TEST(DdgiVolume, DdgiProbeVariabilityUsesGlslReductionWithoutHlslPath) {
  const auto shader_root =
      std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "Internals" / "DefaultResources" / "Shaders";
  const auto reduce_source = ReadTextFile(shader_root / "Compute" / "DDGIProbeVariabilityReduce.comp");
  const auto extra_reduce_source = ReadTextFile(shader_root / "Compute" / "DDGIProbeVariabilityExtraReduce.comp");
  const auto render_layer_source =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "src" / "RenderLayer.cpp");
  ASSERT_FALSE(reduce_source.empty());
  ASSERT_FALSE(extra_reduce_source.empty());
  ASSERT_FALSE(render_layer_source.empty());

  EXPECT_NE(reduce_source.find("layout(set = 0, binding = 0, r16f) uniform image2D "
                               "EE_DDGI_PROBE_VARIABILITY_ATLAS;"),
            std::string::npos);
  EXPECT_NE(reduce_source.find("EE_DDGI_PROBE_STATE[probe_index].w > 0.5f"), std::string::npos);
  EXPECT_NE(reduce_source.find("const float total_possible_samples = 16.0f * 16.0f;"), std::string::npos);
  EXPECT_NE(reduce_source.find("const float normalized_weight = weight_sum / total_possible_samples;"),
            std::string::npos);
  EXPECT_NE(reduce_source.find("imageStore(EE_DDGI_VARIABILITY_REDUCTION_OUTPUT"), std::string::npos);
  EXPECT_NE(extra_reduce_source.find("layout(set = 0, binding = 0, rg32f) uniform image2D "
                                     "EE_DDGI_VARIABILITY_REDUCTION_INPUT;"),
            std::string::npos);
  EXPECT_NE(extra_reduce_source.find("weighted_sum += sample_value.r * sample_value.g"), std::string::npos);
  EXPECT_NE(extra_reduce_source.find("const float total_possible_weight = 16.0f * 16.0f;"), std::string::npos);
  EXPECT_NE(extra_reduce_source.find("const float normalized_weight = weight_sum / total_possible_weight;"),
            std::string::npos);
  EXPECT_NE(render_layer_source.find("DDGIProbeVariabilityReduce.comp"), std::string::npos);
  EXPECT_NE(render_layer_source.find("DDGIProbeVariabilityExtraReduce.comp"), std::string::npos);
  EXPECT_NE(render_layer_source.find("kDdgiProbeVariabilityStableSampleCount = 1"), std::string::npos);
  EXPECT_NE(render_layer_source.find("probe_variability_min_samples = 16"), std::string::npos);
  EXPECT_NE(render_layer_source.find("probe_variability_threshold = 0.2f"), std::string::npos);
  EXPECT_NE(render_layer_source.find("probe_variability_gating_enabled"), std::string::npos);
  EXPECT_NE(render_layer_source.find("ddgi_ray_source.enable_probe_variability_gating"), std::string::npos);
  EXPECT_NE(render_layer_source.find("ddgi_probe_variability_sample_count_ = 0;"), std::string::npos);
  EXPECT_NE(render_layer_source.find("ddgi_probe_variability_stable_sample_count_ = 0;"), std::string::npos);
  EXPECT_NE(render_layer_source.find("DdgiUpdateReasonConverged"), std::string::npos);
  EXPECT_NE(render_layer_source.find("ddgi_frame_probe_warmup_active_"), std::string::npos);
  EXPECT_NE(render_layer_source.find("!ddgi_frame_probe_warmup_active_"), std::string::npos);
  EXPECT_NE(render_layer_source.find("probe_variability_sample_count_complete && probe_variability_below_threshold"),
            std::string::npos);
  EXPECT_EQ(reduce_source.find(".hlsl"), std::string::npos);
  EXPECT_EQ(extra_reduce_source.find(".hlsl"), std::string::npos);
  EXPECT_EQ(render_layer_source.find("DXC"), std::string::npos);
}

TEST(DdgiVolume, DdgiWarmupFrameCountIsGlobalAndTriggerPolicyIsPerVolume) {
  const auto settings_source = ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" /
                                            "include" / "Rendering" / "PBR" / "DdgiSettings.hpp");
  const auto volume_source = ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" /
                                          "include" / "Rendering" / "PBR" / "DdgiVolume.hpp");
  const auto scene_source =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "src" / "Scene.cpp");
  const auto inspector_source = ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" /
                                             "src" / "Editor" / "SDKInspectionAdapters.cpp");
  ASSERT_FALSE(settings_source.empty());
  ASSERT_FALSE(volume_source.empty());
  ASSERT_FALSE(scene_source.empty());
  ASSERT_FALSE(inspector_source.empty());

  EXPECT_NE(settings_source.find("int warmup_frames = 16;"), std::string::npos);
  EXPECT_EQ(volume_source.find("warmup_frames"), std::string::npos);
  EXPECT_NE(scene_source.find("\"warmup_frames\""), std::string::npos);
  EXPECT_NE(inspector_source.find("runtime.warmup_frames = glm::clamp(runtime.warmup_frames, 0, 4096);"),
            std::string::npos);
  EXPECT_NE(inspector_source.find("ImGui::DragInt(\"Warm up frames\""), std::string::npos);
  EXPECT_EQ(settings_source.find("reset_conditions"), std::string::npos);
  EXPECT_EQ(scene_source.find("\"reset_conditions\""), std::string::npos);
  EXPECT_NE(volume_source.find("int warmup_trigger_conditions = DdgiVolumeTriggerConditionLightEnableChanged;"),
            std::string::npos);
  EXPECT_NE(volume_source.find("int variability_reset_trigger_conditions ="), std::string::npos);
  EXPECT_NE(inspector_source.find("DrawDdgiVolumeTriggerConditionCheckbox(\"Light enable/disable##DdgiVolumeWarmup\""),
            std::string::npos);
  EXPECT_NE(
      inspector_source.find("DrawDdgiVolumeTriggerConditionCheckbox(\"Lighting condition##DdgiVolumeVariability\""),
      std::string::npos);
  EXPECT_EQ(inspector_source.find("Probe volume defaults"), std::string::npos);
}

TEST(DdgiVolume, RenderLayerUsesPerVolumeDdgiTriggerPolicy) {
  const auto render_layer_source =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "src" / "RenderLayer.cpp");
  ASSERT_FALSE(render_layer_source.empty());

  EXPECT_NE(render_layer_source.find("std::vector<uint64_t> CollectDdgiLightSignatures"), std::string::npos);
  EXPECT_NE(render_layer_source.find("std::vector<uint64_t> CollectDdgiActiveLightKeys"), std::string::npos);
  EXPECT_NE(render_layer_source.find("bool DdgiTriggerConditionEnabled"), std::string::npos);
  EXPECT_NE(render_layer_source.find("CollectDdgiLightSignatures<DirectionalLight>"), std::string::npos);
  EXPECT_NE(render_layer_source.find("CollectDdgiLightSignatures<PointLight>"), std::string::npos);
  EXPECT_NE(render_layer_source.find("CollectDdgiLightSignatures<SpotLight>"), std::string::npos);
  EXPECT_NE(render_layer_source.find("CollectDdgiActiveLightKeys<DirectionalLight>"), std::string::npos);
  EXPECT_NE(render_layer_source.find("MakeDdgiLightSignatureBase"), std::string::npos);
  EXPECT_NE(render_layer_source.find("scene->GetDataComponent<GlobalTransform>(owner).value"), std::string::npos);
  EXPECT_NE(render_layer_source.find("current_render_instances->environment_info_block != "
                                     "ddgi_previous_environment_info_block_"),
            std::string::npos);
  EXPECT_NE(render_layer_source.find("blocks_changed(current_render_instances->GetGltfShadeMaterials(), "
                                     "ddgi_previous_gltf_shade_materials_)"),
            std::string::npos);
  EXPECT_NE(render_layer_source.find("blocks_changed(current_render_instances->GetGltfTextureInfos(), "
                                     "ddgi_previous_gltf_texture_infos_)"),
            std::string::npos);
  EXPECT_NE(render_layer_source.find("current_render_instances->texture_storage_version != "
                                     "ddgi_previous_texture_storage_version_"),
            std::string::npos);
  EXPECT_NE(render_layer_source.find("ddgi_scene_material_inputs_changed_"), std::string::npos);
  EXPECT_NE(render_layer_source.find("light_signatures != ddgi_previous_light_signatures_"), std::string::npos);
  EXPECT_NE(render_layer_source.find("collect_ddgi_geometry_signatures"), std::string::npos);
  EXPECT_NE(render_layer_source.find("geometry_signatures != ddgi_previous_geometry_signatures_"), std::string::npos);
  EXPECT_NE(render_layer_source.find("render_instance->geometry_version"), std::string::npos);
  EXPECT_NE(render_layer_source.find("render_instance->model.value"), std::string::npos);
  EXPECT_NE(render_layer_source.find("reset_ddgi_warmup_state"), std::string::npos);
  EXPECT_NE(render_layer_source.find("reset_ddgi_variability_state"), std::string::npos);
  EXPECT_NE(render_layer_source.find("scene_material_refresh"), std::string::npos);
  EXPECT_NE(render_layer_source.find("DdgiUpdateReasonSceneInput"), std::string::npos);
  EXPECT_NE(render_layer_source.find("ddgi_ray_source.warmup_trigger_conditions"), std::string::npos);
  EXPECT_NE(render_layer_source.find("ddgi_ray_source.variability_reset_trigger_conditions"), std::string::npos);
  EXPECT_NE(render_layer_source.find("DdgiVolumeTriggerConditionLightEnableChanged"), std::string::npos);
  EXPECT_NE(render_layer_source.find("DdgiVolumeTriggerConditionLightingConditionChanged"), std::string::npos);
  EXPECT_NE(render_layer_source.find("DdgiVolumeTriggerConditionGeometryChanged"), std::string::npos);
  EXPECT_EQ(render_layer_source.find("reset_ddgi_convergence_state"), std::string::npos);
  EXPECT_EQ(render_layer_source.find("DdgiResetConditionEnabled"), std::string::npos);
  EXPECT_EQ(render_layer_source.find("runtime.reset_conditions"), std::string::npos);
  EXPECT_NE(render_layer_source.find("!ddgi_frame_probe_warmup_active_"), std::string::npos);
  EXPECT_NE(render_layer_source.find("current_render_instances->render_info_block != "
                                     "previous_render_instances->render_info_block"),
            std::string::npos);
  EXPECT_EQ(render_layer_source.find("blocks_changed(current_render_instances->directional_light_info_blocks_"),
            std::string::npos);
  EXPECT_EQ(render_layer_source.find("blocks_changed(current_render_instances->GetInstanceInfoBlocks()"),
            std::string::npos);
}

TEST(DdgiVolume, RenderLayerDefersProbeTracingUntilSceneInputsAreReady) {
  const auto source_root = std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK";
  const auto render_layer_source = ReadTextFile(source_root / "src" / "RenderLayer.cpp");
  const auto render_layer_header = ReadTextFile(source_root / "include" / "Layers" / "RenderLayer.hpp");
  ASSERT_FALSE(render_layer_source.empty());
  ASSERT_FALSE(render_layer_header.empty());

  EXPECT_NE(render_layer_header.find("ddgi_deferred_scene_readiness_refresh_"), std::string::npos);
  EXPECT_NE(render_layer_header.find("ddgi_scene_input_settle_frame_count_"), std::string::npos);
  EXPECT_NE(render_layer_source.find("#include \"ProjectManager.hpp\""), std::string::npos);
  EXPECT_NE(render_layer_source.find("kDdgiSceneInputSettleFrameCount = 1"), std::string::npos);
  const auto texture_pending_gate =
      render_layer_source.find("const bool texture_uploads_pending = TextureStorage::HasPendingUploads();");
  const auto project_pending_gate = render_layer_source.find(
      "const bool project_scene_inputs_pending = ProjectManager::HasProject() && !ProjectManager::IsProjectIdle();");
  const auto scene_pending_gate = render_layer_source.find(
      "const bool scene_inputs_pending = texture_uploads_pending || "
      "project_scene_inputs_pending;");
  const auto ray_count_setup = render_layer_source.find("const auto ray_count");
  ASSERT_NE(texture_pending_gate, std::string::npos);
  ASSERT_NE(project_pending_gate, std::string::npos);
  ASSERT_NE(scene_pending_gate, std::string::npos);
  ASSERT_NE(ray_count_setup, std::string::npos);
  EXPECT_LT(texture_pending_gate, ray_count_setup);
  EXPECT_LT(project_pending_gate, ray_count_setup);
  EXPECT_LT(scene_pending_gate, ray_count_setup);
  EXPECT_NE(render_layer_source.find("if (scene_inputs_pending) {"), std::string::npos);
  EXPECT_NE(render_layer_source.find("ddgi_deferred_scene_readiness_refresh_ = true;"), std::string::npos);
  EXPECT_NE(render_layer_source.find("ddgi_scene_input_settle_frame_count_ = 0;"), std::string::npos);
  EXPECT_NE(render_layer_source.find("ddgi_clear_probe_atlas_this_frame_ = true;"), std::string::npos);
  EXPECT_NE(render_layer_source.find("ddgi_frame_probe_update_indices_.clear();"), std::string::npos);
  const auto settle_gate = render_layer_source.find("ddgi_scene_input_settle_frame_count_ <");
  ASSERT_NE(settle_gate, std::string::npos);
  EXPECT_LT(settle_gate, ray_count_setup);
  EXPECT_NE(render_layer_source.find("++ddgi_scene_input_settle_frame_count_;"), std::string::npos);
  EXPECT_NE(render_layer_source.find("const bool scene_readiness_refresh = ddgi_deferred_scene_readiness_refresh_;"),
            std::string::npos);
  EXPECT_NE(render_layer_source.find("scene_material_refresh || scene_readiness_refresh"), std::string::npos);
  EXPECT_NE(render_layer_source.find("reset_probe_history || scene_material_refresh ||"), std::string::npos);
  EXPECT_NE(render_layer_source.find("scene_readiness_refresh || !ddgi_probe_state_buffer_"), std::string::npos);
  const auto refresh_consumed = render_layer_source.find("ddgi_deferred_scene_readiness_refresh_ = false;");
  const auto tracing_enabled = render_layer_source.find("ddgi_frame_trace_probe_rays_ = true;");
  ASSERT_NE(refresh_consumed, std::string::npos);
  ASSERT_NE(tracing_enabled, std::string::npos);
  EXPECT_LT(refresh_consumed, tracing_enabled);
  EXPECT_NE(render_layer_source.find("ddgi_scene_input_settle_frame_count_ = 0;", refresh_consumed), std::string::npos);
}

TEST(DdgiVolume, OffscreenPreviewRenderingDoesNotTouchSceneDdgiTracking) {
  const auto source_root = std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK";
  const auto render_layer_source = ReadTextFile(source_root / "src" / "RenderLayer.cpp");
  ASSERT_FALSE(render_layer_source.empty());

  const auto immediate_render = render_layer_source.find("void RenderLayer::RenderSceneToCameraImmediately");
  const auto temporary_storage = render_layer_source.find(
      "render_instances_list_[current_frame_index] = std::make_shared<RenderInstanceStorage>();", immediate_render);
  const auto isolated_prepare =
      render_layer_source.find("PrepareSceneForRendering(scene, false, false, false, false);", immediate_render);
  const auto camera_render =
      render_layer_source.find("RenderToCamera(scene, camera_global_transform, camera, true);", immediate_render);
  const auto restore_storage = render_layer_source.find(
      "render_instances_list_[current_frame_index] = previous_render_instances;", immediate_render);
  const auto rebind_previous = render_layer_source.find(
      "BindRenderInstanceStorage(current_frame_index, previous_render_instances);", immediate_render);
  ASSERT_NE(immediate_render, std::string::npos);
  ASSERT_NE(temporary_storage, std::string::npos);
  ASSERT_NE(isolated_prepare, std::string::npos);
  ASSERT_NE(camera_render, std::string::npos);
  ASSERT_NE(restore_storage, std::string::npos);
  ASSERT_NE(rebind_previous, std::string::npos);
  EXPECT_LT(temporary_storage, isolated_prepare);
  EXPECT_LT(isolated_prepare, camera_render);
  EXPECT_LT(camera_render, restore_storage);
  EXPECT_LT(restore_storage, rebind_previous);
}

TEST(DdgiVolume, DdgiProbeHitsExplicitlyEvaluateAnalyticSceneLights) {
  const auto shader_root =
      std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "Internals" / "DefaultResources" / "Shaders";
  const auto closest_hit_source =
      ReadTextFile(shader_root / "RayTracing" / "ClosestHit" / "DDGIProbeDiagnostics.rchit");
  ASSERT_FALSE(closest_hit_source.empty());

  EXPECT_NE(closest_hit_source.find("EE_DDGI_EXPLICIT_DIRECTIONAL_LIGHT_IRRADIANCE"), std::string::npos);
  EXPECT_NE(closest_hit_source.find("EE_DDGI_EXPLICIT_POINT_LIGHT_IRRADIANCE"), std::string::npos);
  EXPECT_NE(closest_hit_source.find("EE_DDGI_EXPLICIT_SPOT_LIGHT_IRRADIANCE"), std::string::npos);
  EXPECT_NE(closest_hit_source.find("EE_DDGI_LAMBERT_IRRADIANCE"), std::string::npos);
  EXPECT_NE(closest_hit_source.find("EE_DDGI_DISTANCE_LIGHT_ATTENUATION"), std::string::npos);
  EXPECT_NE(closest_hit_source.find("light_distance >= constant_linear_quadratic_far.w"), std::string::npos);
  EXPECT_NE(closest_hit_source.find("EE_RENDER_INFO.directional_light_size"), std::string::npos);
  EXPECT_NE(closest_hit_source.find("EE_RENDER_INFO.point_light_size"), std::string::npos);
  EXPECT_NE(closest_hit_source.find("EE_RENDER_INFO.spot_light_size"), std::string::npos);
  EXPECT_NE(closest_hit_source.find("EE_DDGI_EXPLICIT_DIRECTIONAL_LIGHT_IRRADIANCE(EE_DIRECTIONAL_LIGHTS[i]"),
            std::string::npos);
  EXPECT_NE(closest_hit_source.find("EE_DDGI_EXPLICIT_POINT_LIGHT_IRRADIANCE(EE_POINT_LIGHTS[i]"), std::string::npos);
  EXPECT_NE(closest_hit_source.find("EE_DDGI_EXPLICIT_SPOT_LIGHT_IRRADIANCE(EE_SPOT_LIGHTS[i]"), std::string::npos);
  EXPECT_NE(closest_hit_source.find("light.diffuse.w == 1.0f"), std::string::npos);
  EXPECT_NE(closest_hit_source.find("EE_DDGI_SHADOW_VISIBILITY(position + normal * trace_parameters.y"),
            std::string::npos);
}

TEST(DdgiVolume, DdgiAtlasBorderCopyTexelsStayInsideSourceTile) {
  const auto border_copy_texel = [](const glm::uvec2 tile_origin, const uint32_t tile_size,
                                    const glm::uvec2 border_texel) {
    const uint32_t tile_stride = tile_size + 2u;
    glm::uvec2 copy_texel = tile_origin;
    const bool is_corner = (border_texel.x == 0u || border_texel.x == tile_stride - 1u) &&
                           (border_texel.y == 0u || border_texel.y == tile_stride - 1u);
    const bool is_row = border_texel.x > 0u && border_texel.x < tile_stride - 1u;
    if (is_corner) {
      copy_texel += glm::uvec2(border_texel.x > 0u ? 1u : tile_size, border_texel.y > 0u ? 1u : tile_size);
    } else if (is_row) {
      copy_texel += glm::uvec2((tile_stride - 1u) - border_texel.x,
                               border_texel.y > 0u ? border_texel.y - 1u : border_texel.y + 1u);
    } else {
      copy_texel += glm::uvec2(border_texel.x > 0u ? border_texel.x - 1u : border_texel.x + 1u,
                               (tile_stride - 1u) - border_texel.y);
    }
    return copy_texel;
  };

  const glm::uvec2 tile_origin(30, 50);
  for (const uint32_t tile_size : {1u, 2u, 8u, 16u}) {
    const uint32_t tile_stride = tile_size + 2u;
    for (uint32_t y = 0; y < tile_stride; ++y) {
      for (uint32_t x = 0; x < tile_stride; ++x) {
        const bool is_border = x == 0u || y == 0u || x == tile_stride - 1u || y == tile_stride - 1u;
        if (!is_border) {
          continue;
        }
        const glm::uvec2 copy_texel = border_copy_texel(tile_origin, tile_size, {x, y});
        EXPECT_GE(copy_texel.x, tile_origin.x + 1u);
        EXPECT_GE(copy_texel.y, tile_origin.y + 1u);
        EXPECT_LE(copy_texel.x, tile_origin.x + tile_size);
        EXPECT_LE(copy_texel.y, tile_origin.y + tile_size);
      }
    }
  }
}

TEST(DdgiVolume, DdgiProbeUpdateClearsRtxgiStyleScrolledPlanes) {
  const auto shader_root =
      std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "Internals" / "DefaultResources" / "Shaders";
  const auto ddgi_helper_source = ReadTextFile(shader_root / "Includes" / "DDGI.glsl");
  const auto probe_update_source = ReadTextFile(shader_root / "Compute" / "DDGIProbeUpdate.comp");
  const auto render_layer_source =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "src" / "RenderLayer.cpp");
  ASSERT_FALSE(ddgi_helper_source.empty());
  ASSERT_FALSE(probe_update_source.empty());
  ASSERT_FALSE(render_layer_source.empty());

  EXPECT_NE(ddgi_helper_source.find("EE_DDGI_SCROLL_CLEAR_X_BIT"), std::string::npos);
  EXPECT_NE(ddgi_helper_source.find("EE_DDGI_SCROLL_POSITIVE_Z_BIT"), std::string::npos);
  EXPECT_NE(ddgi_helper_source.find("EE_DDGI_CLEAR_SCROLLED_PLANE"), std::string::npos);
  EXPECT_NE(ddgi_helper_source.find("positive_direction ? (probe_count + ((offset - 1) % probe_count)) % probe_count"),
            std::string::npos);
  EXPECT_NE(ddgi_helper_source.find(": (probe_count + (offset % probe_count)) % probe_count"), std::string::npos);
  EXPECT_NE(probe_update_source.find("EE_DDGI_SCROLL_PROBE_GRID(logical_probe_grid, probe_scroll_offset.xyz"),
            std::string::npos);
  EXPECT_NE(probe_update_source.find("EE_DDGI_CLEAR_SCROLLED_PROBE(physical_probe_grid, probe_scroll_offset"),
            std::string::npos);
  EXPECT_NE(probe_update_source.find("EE_DDGI_CLEAR_IRRADIANCE_TILE"), std::string::npos);
  EXPECT_NE(probe_update_source.find("EE_DDGI_CLEAR_VISIBILITY_TILE"), std::string::npos);
  EXPECT_NE(probe_update_source.find("EE_DDGI_CLEAR_VARIABILITY_TILE"), std::string::npos);
  EXPECT_NE(render_layer_source.find("DdgiVolumeMovementType::Scrolling"), std::string::npos);
  EXPECT_NE(render_layer_source.find("CalculateDdgiEffectiveFirstProbe"), std::string::npos);
  EXPECT_NE(render_layer_source.find("ResetDdgiProbeScrollOrigin"), std::string::npos);
  EXPECT_NE(render_layer_source.find("PackDdgiProbeScrollFlags"), std::string::npos);
  EXPECT_NE(render_layer_source.find("CreateDdgiProbeScrollPushConstant"), std::string::npos);
  EXPECT_NE(render_layer_source.find("std::floor(value)"), std::string::npos);
  EXPECT_NE(render_layer_source.find("std::ceil(value)"), std::string::npos);
  EXPECT_NE(render_layer_source.find("ddgi_scroll_clear_this_frame"), std::string::npos);
  EXPECT_EQ(ddgi_helper_source.find(".hlsl"), std::string::npos);
  EXPECT_EQ(probe_update_source.find(".hlsl"), std::string::npos);
}

TEST(DdgiVolume, DdgiShadowRaysIgnoreNonShadowCastingLightVisualizers) {
  const auto shader_root =
      std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "Internals" / "DefaultResources" / "Shaders";
  const auto ddgi_helper_source = ReadTextFile(shader_root / "Includes" / "DDGI.glsl");
  const auto raygen_source = ReadTextFile(shader_root / "RayTracing" / "RayGen" / "DDGIProbeDiagnostics.rgen");
  const auto closest_hit_source =
      ReadTextFile(shader_root / "RayTracing" / "ClosestHit" / "DDGIProbeDiagnostics.rchit");
  const auto graphics_resources_source = ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) /
                                                      "EvoEngine_SDK" / "src" / "GraphicsResources.cpp");
  const auto demo_scene_source =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_App" / "src" / "DemoScene.cpp");

  EXPECT_NE(ddgi_helper_source.find("const uint EE_DDGI_RAY_MASK_GEOMETRY = 0x01u;"), std::string::npos);
  EXPECT_NE(ddgi_helper_source.find("const uint EE_DDGI_RAY_MASK_SHADOW = 0x02u;"), std::string::npos);
  EXPECT_NE(raygen_source.find("EE_DDGI_RAY_MASK_GEOMETRY"), std::string::npos);
  EXPECT_NE(closest_hit_source.find("EE_DDGI_RAY_MASK_SHADOW"), std::string::npos);
  EXPECT_NE(graphics_resources_source.find("kDdgiRayMaskGeometry | (render_instance->cast_shadow ? "
                                           "kDdgiRayMaskShadow : 0u)"),
            std::string::npos);
  EXPECT_NE(demo_scene_source.find("point_light_right_renderer->cast_shadow = false;"), std::string::npos);
}

TEST(DdgiVolume, RenderLayerFrameResourceLayoutRespondsToResourceSizingInputs) {
  RenderLayer::DdgiSettings settings;
  settings.storage.max_probe_count = 1000;
  settings.storage.atlas_probe_columns = 5;
  settings.storage.irradiance_tile_resolution = 6;
  settings.storage.visibility_tile_resolution = 14;
  settings.runtime.ray_count = 11;

  auto layout = RenderLayer::CalculateDdgiFrameResourceLayout(settings, 37);

  EXPECT_EQ(layout.probe_count, 37u);
  EXPECT_EQ(layout.irradiance_atlas.columns, 5u);
  EXPECT_EQ(layout.irradiance_atlas.rows, 8u);
  EXPECT_EQ(layout.irradiance_atlas.resolution, glm::uvec2(40, 64));
  EXPECT_EQ(layout.visibility_atlas.resolution, glm::uvec2(80, 128));
  EXPECT_EQ(layout.variability_atlas.resolution, glm::uvec2(30, 48));
  EXPECT_EQ(layout.variability_reduction_extent, glm::uvec2(2, 3));
  EXPECT_EQ(layout.probe_metadata_byte_size, 37ull * sizeof(glm::vec4) * 3ull);
  EXPECT_EQ(layout.probe_state_byte_size, 37ull * sizeof(glm::vec4));
  EXPECT_EQ(layout.probe_update_index_byte_size, 37ull * sizeof(uint32_t));
  EXPECT_EQ(layout.ray_output_byte_size, 37ull * 11ull * sizeof(PointCloudSample));

  settings.runtime.ray_count = 23;
  auto resized_layout = RenderLayer::CalculateDdgiFrameResourceLayout(settings, 37);
  EXPECT_EQ(resized_layout.irradiance_atlas.resolution, layout.irradiance_atlas.resolution);
  EXPECT_EQ(resized_layout.visibility_atlas.resolution, layout.visibility_atlas.resolution);
  EXPECT_EQ(resized_layout.variability_atlas.resolution, layout.variability_atlas.resolution);
  EXPECT_EQ(resized_layout.variability_reduction_extent, layout.variability_reduction_extent);
  EXPECT_EQ(resized_layout.ray_output_byte_size, 37ull * 23ull * sizeof(PointCloudSample));

  settings.storage.irradiance_tile_resolution = 10;
  settings.storage.visibility_tile_resolution = 18;
  resized_layout = RenderLayer::CalculateDdgiFrameResourceLayout(settings, 37);
  EXPECT_EQ(resized_layout.irradiance_atlas.resolution, glm::uvec2(60, 96));
  EXPECT_EQ(resized_layout.visibility_atlas.resolution, glm::uvec2(100, 160));
  EXPECT_EQ(resized_layout.variability_atlas.resolution, glm::uvec2(50, 80));
  EXPECT_EQ(resized_layout.variability_reduction_extent, glm::uvec2(4, 5));
  EXPECT_EQ(resized_layout.ray_output_byte_size, 37ull * 23ull * sizeof(PointCloudSample));
}

TEST(DdgiVolume, RenderLayerVolumeBlendWeightMatchesRtxgiVolumeCoverage) {
  const auto probe_counts = glm::ivec3(5, 5, 5);
  const auto probe_step_lengths = glm::vec3(2.0f);

  EXPECT_FLOAT_EQ(RenderLayer::CalculateDdgiVolumeBlendWeight({2.0f, 2.0f, 2.0f}, probe_counts, probe_step_lengths),
                  1.0f);
  EXPECT_FLOAT_EQ(RenderLayer::CalculateDdgiVolumeBlendWeight({0.0f, 2.0f, 2.0f}, probe_counts, probe_step_lengths),
                  1.0f);
  EXPECT_FLOAT_EQ(RenderLayer::CalculateDdgiVolumeBlendWeight({4.0f, 2.0f, 2.0f}, probe_counts, probe_step_lengths),
                  1.0f);
  EXPECT_NEAR(RenderLayer::CalculateDdgiVolumeBlendWeight({-0.5f, 2.0f, 2.0f}, probe_counts, probe_step_lengths), 0.5f,
              0.0001f);
  EXPECT_FLOAT_EQ(RenderLayer::CalculateDdgiVolumeBlendWeight({-1.0f, 2.0f, 2.0f}, probe_counts, probe_step_lengths),
                  0.0f);
  EXPECT_FLOAT_EQ(RenderLayer::CalculateDdgiVolumeBlendWeight({-0.1f, 2.0f, 2.0f}, probe_counts, probe_step_lengths),
                  0.9f);
}

TEST(DdgiVolume, RenderLayerUsesVanillaDdgiUpdateHysteresis) {
  RenderLayer::DdgiSettings settings;
  settings.runtime.hysteresis = 0.97f;

  EXPECT_FLOAT_EQ(RenderLayer::CalculateDdgiUpdateHysteresis(settings, RenderLayer::DdgiUpdateReasonSteadyState),
                  0.97f);
  EXPECT_FLOAT_EQ(RenderLayer::CalculateDdgiUpdateHysteresis(
                      settings, RenderLayer::DdgiUpdateReasonSource | RenderLayer::DdgiUpdateReasonManualReset),
                  0.0f);
  EXPECT_FLOAT_EQ(RenderLayer::CalculateDdgiUpdateHysteresis(settings, RenderLayer::DdgiUpdateReasonSource), 0.0f);
  EXPECT_FLOAT_EQ(RenderLayer::CalculateDdgiUpdateHysteresis(settings, RenderLayer::DdgiUpdateReasonSceneInput), 0.0f);
  EXPECT_FLOAT_EQ(RenderLayer::CalculateDdgiUpdateHysteresis(
                      settings, RenderLayer::DdgiUpdateReasonSteadyState | RenderLayer::DdgiUpdateReasonWarmup, 0),
                  0.0f);
  EXPECT_NEAR(RenderLayer::CalculateDdgiUpdateHysteresis(
                  settings, RenderLayer::DdgiUpdateReasonSteadyState | RenderLayer::DdgiUpdateReasonWarmup, 8),
              0.97f * (8.0f / 15.0f), 0.0001f);
  EXPECT_FLOAT_EQ(RenderLayer::CalculateDdgiUpdateHysteresis(
                      settings, RenderLayer::DdgiUpdateReasonSteadyState | RenderLayer::DdgiUpdateReasonWarmup, 15),
                  0.97f);
  EXPECT_FLOAT_EQ(RenderLayer::CalculateDdgiUpdateHysteresis(
                      settings, RenderLayer::DdgiUpdateReasonSteadyState | RenderLayer::DdgiUpdateReasonWarmup, 16),
                  0.97f);
  EXPECT_FLOAT_EQ(RenderLayer::CalculateDdgiUpdateHysteresis(
                      settings, RenderLayer::DdgiUpdateReasonSource | RenderLayer::DdgiUpdateReasonWarmup, 8),
                  0.0f);

  settings.runtime.warmup_frames = 0;
  EXPECT_FLOAT_EQ(RenderLayer::CalculateDdgiUpdateHysteresis(
                      settings, RenderLayer::DdgiUpdateReasonSteadyState | RenderLayer::DdgiUpdateReasonWarmup, 0),
                  0.97f);
}

TEST(DdgiVolume, RenderLayerUsesRtxgiBrightnessThresholdDuringWarmup) {
  RenderLayer::DdgiSettings settings;
  settings.runtime.brightness_threshold = 0.1f;

  EXPECT_FLOAT_EQ(
      RenderLayer::CalculateDdgiUpdateBrightnessThreshold(settings, RenderLayer::DdgiUpdateReasonSteadyState), 0.1f);
  EXPECT_FLOAT_EQ(RenderLayer::CalculateDdgiUpdateBrightnessThreshold(settings, RenderLayer::DdgiUpdateReasonSource),
                  0.1f);
  EXPECT_FLOAT_EQ(
      RenderLayer::CalculateDdgiUpdateBrightnessThreshold(settings, RenderLayer::DdgiUpdateReasonManualReset), 0.1f);
  EXPECT_FLOAT_EQ(RenderLayer::CalculateDdgiUpdateBrightnessThreshold(
                      settings, RenderLayer::DdgiUpdateReasonSteadyState | RenderLayer::DdgiUpdateReasonWarmup),
                  0.1f);

  settings.runtime.warmup_frames = 0;
  EXPECT_FLOAT_EQ(RenderLayer::CalculateDdgiUpdateBrightnessThreshold(
                      settings, RenderLayer::DdgiUpdateReasonSteadyState | RenderLayer::DdgiUpdateReasonWarmup),
                  0.1f);
}

TEST(DdgiVolume, RenderLayerIsolatesFirstWarmupFrameFromDdgiHistory) {
  const auto render_layer_source =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "src" / "RenderLayer.cpp");
  const auto ray_hit_source =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "Internals" /
                   "DefaultResources" / "Shaders" / "RayTracing" / "ClosestHit" / "DDGIProbeDiagnostics.rchit");
  ASSERT_FALSE(render_layer_source.empty());
  ASSERT_FALSE(ray_hit_source.empty());

  EXPECT_NE(render_layer_source.find("ddgi_first_warmup_frame = ddgi_frame_probe_warmup_active_ &&"),
            std::string::npos);
  EXPECT_NE(render_layer_source.find("ddgi_probe_warmup_frame_index_ == 0u"), std::string::npos);
  EXPECT_NE(render_layer_source.find("ddgi_first_warmup_frame ? (std::numeric_limits<float>::max)()"),
            std::string::npos);
  EXPECT_NE(render_layer_source.find("skip_recursive_ddgi ? 1.0f : 0.0f"), std::string::npos);
  EXPECT_NE(render_layer_source.find("skip_inactive_probe_trace, ddgi_first_warmup_frame"), std::string::npos);
  EXPECT_NE(ray_hit_source.find("const bool skip_recursive_ddgi = trace_parameters.w > 0.5f;"), std::string::npos);
  EXPECT_NE(ray_hit_source.find("skip_recursive_ddgi ? vec3(0.0f) : EE_DDGI_RECURSIVE_IRRADIANCE"), std::string::npos);
}

TEST(DdgiVolume, RenderLayerFormatsDdgiUpdateReasonsForDebugging) {
  EXPECT_EQ(RenderLayer::FormatDdgiUpdateReasons(RenderLayer::DdgiUpdateReasonNone), "None");
  EXPECT_EQ(RenderLayer::FormatDdgiUpdateReasons(
                RenderLayer::DdgiUpdateReasonSource | RenderLayer::DdgiUpdateReasonManualReset |
                RenderLayer::DdgiUpdateReasonSteadyState | RenderLayer::DdgiUpdateReasonConverged |
                RenderLayer::DdgiUpdateReasonWarmup | RenderLayer::DdgiUpdateReasonSceneInput),
            "DDGI source, Manual reset, Steady state, Converged, Warm up, Scene input");
}
