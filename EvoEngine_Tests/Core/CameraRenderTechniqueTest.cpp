#include "EvoEngine_SDK_PCH.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include "Application.hpp"
#include "ApplicationContext.hpp"
#include "ApplicationInitializationSettings.hpp"
#include "Camera.hpp"
#include "RenderInstanceStorage.hpp"
#include "Serialization.hpp"

using namespace evo_engine;

namespace {
ApplicationInitializationSettings EmptyProjectSettings() {
  ApplicationInitializationSettings settings;
  settings.allow_empty_project = true;
  settings.load_default_resources = false;
  settings.load_project_assets = false;
  settings.load_project_start_scene = false;
  settings.enable_runtime_packages = false;
  return settings;
}

std::string ReadTextFile(const std::filesystem::path& path) {
  std::ifstream file(path);
  EXPECT_TRUE(file.good()) << path.string();
  return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}

std::filesystem::path SourcePath(const std::filesystem::path& relative_path) {
  return std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / relative_path;
}
}  // namespace

TEST(GraphicsInitializationSettings, DefaultsAllShadowsToHighResolution) {
  const GraphicsInitializationSettings settings;
  EXPECT_EQ(settings.shadow_map_resolution_quality, GraphicsInitializationSettings::ShadowMapResolutionQuality::High);
  EXPECT_EQ(settings.directional_light_shadow_map_resolution, 4096u);
  EXPECT_EQ(settings.point_light_shadow_map_resolution, 4096u);
  EXPECT_EQ(settings.spot_light_shadow_map_resolution, 4096u);
}

TEST(GraphicsInitializationSettings, ExplicitQualityUpdatesAllShadowResolutions) {
  GraphicsInitializationSettings settings;
  settings.SetShadowMapResolutionQuality(GraphicsInitializationSettings::ShadowMapResolutionQuality::VeryHigh);
  EXPECT_EQ(settings.directional_light_shadow_map_resolution, 8192u);
  EXPECT_EQ(settings.point_light_shadow_map_resolution, 8192u);
  EXPECT_EQ(settings.spot_light_shadow_map_resolution, 8192u);

  settings.SetShadowMapResolutionQuality(GraphicsInitializationSettings::ShadowMapResolutionQuality::Medium);
  EXPECT_EQ(settings.directional_light_shadow_map_resolution, 2048u);
  EXPECT_EQ(settings.point_light_shadow_map_resolution, 2048u);
  EXPECT_EQ(settings.spot_light_shadow_map_resolution, 2048u);
}

TEST(CameraRenderTechnique, NamesAndCanonicalSerializationExposeRasterRayTracingAndRayQuery) {
  const auto& names = Camera::GetCameraRenderModeNames();
  ASSERT_EQ(names.size(), Camera::kCameraRenderModeCount);
  EXPECT_EQ(names[static_cast<uint32_t>(Camera::CameraRenderMode::Rasterization)], "Rasterization");
  EXPECT_EQ(names[static_cast<uint32_t>(Camera::CameraRenderMode::RayTracing)], "RayTracing");
  EXPECT_EQ(names[static_cast<uint32_t>(Camera::CameraRenderMode::RayQuery)], "RayQuery");

  EXPECT_EQ(Camera::ParseCameraRenderMode("Rasterization", Camera::CameraRenderMode::RayTracing),
            Camera::CameraRenderMode::Rasterization);
  EXPECT_EQ(Camera::ParseCameraRenderMode("RayTracing"), Camera::CameraRenderMode::RayTracing);
  EXPECT_EQ(Camera::ParseCameraRenderMode("RayQuery"), Camera::CameraRenderMode::RayQuery);
  EXPECT_EQ(Camera::ParseCameraRenderMode("rasterization", Camera::CameraRenderMode::RayTracing),
            Camera::CameraRenderMode::RayTracing);
  EXPECT_EQ(Camera::ParseCameraRenderMode("2", Camera::CameraRenderMode::RayTracing),
            Camera::CameraRenderMode::RayTracing);
  EXPECT_EQ(Camera::NormalizeCameraRenderMode(999), Camera::CameraRenderMode::Rasterization);
  EXPECT_FALSE(Camera::IsRayCameraRenderMode(Camera::CameraRenderMode::Rasterization));
  EXPECT_TRUE(Camera::IsRayCameraRenderMode(Camera::CameraRenderMode::RayTracing));
  EXPECT_TRUE(Camera::IsRayCameraRenderMode(Camera::CameraRenderMode::RayQuery));

  const auto& background_names = Camera::GetBackgroundSourceNames();
  ASSERT_EQ(background_names.size(), Camera::kBackgroundSourceCount);
  EXPECT_EQ(background_names[static_cast<uint32_t>(Camera::BackgroundSource::ClearColor)], "Clear Color");
  EXPECT_EQ(background_names[static_cast<uint32_t>(Camera::BackgroundSource::Cubemap)], "Cubemap");
  EXPECT_EQ(background_names[static_cast<uint32_t>(Camera::BackgroundSource::EnvironmentalMap)], "Environmental Map");
  EXPECT_EQ(background_names[static_cast<uint32_t>(Camera::BackgroundSource::InheritEnvironmentalLighting)],
            "Inherit Environmental Lighting");
  EXPECT_EQ(background_names[static_cast<uint32_t>(Camera::BackgroundSource::EngineDefaultSkybox)],
            "Engine Default Skybox");
  EXPECT_EQ(Camera::ParseBackgroundSource("Clear Color"), Camera::BackgroundSource::ClearColor);
  EXPECT_EQ(Camera::ParseBackgroundSource("Cubemap"), Camera::BackgroundSource::Cubemap);
  EXPECT_EQ(Camera::ParseBackgroundSource("Environmental Map"), Camera::BackgroundSource::EnvironmentalMap);
  EXPECT_EQ(Camera::ParseBackgroundSource("Inherit Environmental Lighting"),
            Camera::BackgroundSource::InheritEnvironmentalLighting);
  EXPECT_EQ(Camera::ParseBackgroundSource("Engine Default Skybox"), Camera::BackgroundSource::EngineDefaultSkybox);
  EXPECT_EQ(Camera::ParseBackgroundSource("clear color", Camera::BackgroundSource::Cubemap),
            Camera::BackgroundSource::Cubemap);
  EXPECT_EQ(Camera::ParseBackgroundSource("4", Camera::BackgroundSource::Cubemap), Camera::BackgroundSource::Cubemap);
  EXPECT_EQ(Camera::NormalizeBackgroundSource(999), Camera::BackgroundSource::Cubemap);

  const auto& ser_names = Camera::GetShaderExecutionReorderingModeNames();
  ASSERT_EQ(ser_names.size(), Camera::kShaderExecutionReorderingModeCount);
  EXPECT_EQ(ser_names[static_cast<uint32_t>(CameraSettings::ShaderExecutionReorderingMode::Disabled)], "Disabled");
  EXPECT_EQ(ser_names[static_cast<uint32_t>(CameraSettings::ShaderExecutionReorderingMode::Automatic)], "Automatic");
  EXPECT_EQ(ser_names[static_cast<uint32_t>(CameraSettings::ShaderExecutionReorderingMode::Enabled)], "Enabled");
  EXPECT_EQ(
      Camera::ParseShaderExecutionReorderingMode("Disabled", CameraSettings::ShaderExecutionReorderingMode::Enabled),
      CameraSettings::ShaderExecutionReorderingMode::Disabled);
  EXPECT_EQ(Camera::ParseShaderExecutionReorderingMode("Automatic"),
            CameraSettings::ShaderExecutionReorderingMode::Automatic);
  EXPECT_EQ(Camera::ParseShaderExecutionReorderingMode("Enabled"),
            CameraSettings::ShaderExecutionReorderingMode::Enabled);
  EXPECT_EQ(Camera::ParseShaderExecutionReorderingMode("1", CameraSettings::ShaderExecutionReorderingMode::Enabled),
            CameraSettings::ShaderExecutionReorderingMode::Enabled);
  EXPECT_EQ(Camera::NormalizeShaderExecutionReorderingMode(999),
            CameraSettings::ShaderExecutionReorderingMode::Disabled);

  const auto& debug_views = Camera::GetRayDebugViewNames();
  ASSERT_EQ(debug_views.size(), Camera::kRayDebugViewCount);
  EXPECT_EQ(debug_views[static_cast<uint32_t>(CameraSettings::RayDebugView::Beauty)], "Beauty");
  EXPECT_EQ(debug_views[static_cast<uint32_t>(CameraSettings::RayDebugView::EmissivePdf)], "Emissive PDF");
  for (uint32_t index = 0; index < debug_views.size(); ++index) {
    EXPECT_EQ(Camera::ParseRayDebugView(debug_views[index]), static_cast<CameraSettings::RayDebugView>(index));
  }
  EXPECT_EQ(Camera::ParseRayDebugView("specular-f0", CameraSettings::RayDebugView::Emission),
            CameraSettings::RayDebugView::Emission);
  EXPECT_EQ(Camera::ParseRayDebugView("19", CameraSettings::RayDebugView::Emission),
            CameraSettings::RayDebugView::Emission);
  EXPECT_EQ(Camera::NormalizeRayDebugView(999), CameraSettings::RayDebugView::Beauty);
}

TEST(CameraRenderTechnique, CameraInfoBlockKeepsShaderArrayStrideAlignment) {
  EXPECT_EQ(offsetof(CameraInfoBlock, raster_lighting_flags), 684u);
  EXPECT_EQ(offsetof(CameraInfoBlock, shadow_split_distances), 688u);
  EXPECT_EQ(sizeof(CameraInfoBlock), 704u);
  EXPECT_LT(offsetof(CameraInfoBlock, firefly_clamp_enabled), offsetof(CameraInfoBlock, gamma));
  EXPECT_LT(offsetof(CameraInfoBlock, gamma), offsetof(CameraInfoBlock, sample_size));
  EXPECT_LT(offsetof(CameraInfoBlock, sample_size), offsetof(CameraInfoBlock, bounce));
  EXPECT_LT(offsetof(CameraInfoBlock, bounce), offsetof(CameraInfoBlock, firefly_clamp_threshold));
  EXPECT_LT(offsetof(CameraInfoBlock, firefly_clamp_threshold), offsetof(CameraInfoBlock, auto_spp_enabled));
  EXPECT_LT(offsetof(CameraInfoBlock, auto_spp_enabled), offsetof(CameraInfoBlock, auto_spp_min_samples));
  EXPECT_LT(offsetof(CameraInfoBlock, auto_spp_min_samples), offsetof(CameraInfoBlock, auto_spp_max_samples));
  EXPECT_LT(offsetof(CameraInfoBlock, auto_spp_max_samples), offsetof(CameraInfoBlock, auto_spp_convergence_threshold));
  EXPECT_LT(offsetof(CameraInfoBlock, auto_spp_convergence_threshold),
            offsetof(CameraInfoBlock, emissive_triangle_nee_enabled));
  EXPECT_LT(offsetof(CameraInfoBlock, emissive_triangle_nee_enabled), offsetof(CameraInfoBlock, ray_debug_view));
  EXPECT_LT(offsetof(CameraInfoBlock, ray_debug_view), offsetof(CameraInfoBlock, raster_lighting_flags));
}

TEST(CameraRenderTechnique, GtaoSpecularVisibilityUsesTheExistingCameraBlockLane) {
  EXPECT_EQ(CameraInfoBlock::kRasterLightingGtaoVisibility, 1u);
  CameraInfoBlock first;
  CameraInfoBlock second = first;
  second.raster_lighting_flags = CameraInfoBlock::kRasterLightingGtaoVisibility;
  EXPECT_TRUE(first != second);

  const auto cameras =
      ReadTextFile(SourcePath("EvoEngine_SDK/Internals/DefaultResources/Shaders/Includes/Cameras.glsl"));
  const auto lighting =
      ReadTextFile(SourcePath("EvoEngine_SDK/Internals/DefaultResources/Shaders/Includes/Lighting.glsl"));
  const auto camera_source = ReadTextFile(SourcePath("EvoEngine_SDK/src/Camera.cpp"));
  EXPECT_NE(cameras.find("uint raster_lighting_flags"), std::string::npos);
  EXPECT_NE(lighting.find("raster_lighting_flags & 1u"), std::string::npos);
  EXPECT_NE(camera_source.find("AmbientOcclusion::Algorithm::Gtao"), std::string::npos);
}

TEST(CameraRenderTechnique, DirectionalShadowSplitsUseTheSelectedCameraBlock) {
  const auto cameras =
      ReadTextFile(SourcePath("EvoEngine_SDK/Internals/DefaultResources/Shaders/Includes/Cameras.glsl"));
  const auto lighting =
      ReadTextFile(SourcePath("EvoEngine_SDK/Internals/DefaultResources/Shaders/Includes/Lighting.glsl"));
  const auto storage = ReadTextFile(SourcePath("EvoEngine_SDK/src/RenderInstanceStorage.cpp"));
  EXPECT_NE(cameras.find("vec4 shadow_split_distances"), std::string::npos);
  EXPECT_NE(lighting.find("EE_CAMERAS[EE_CAMERA_INDEX].shadow_split_distances"), std::string::npos);
  EXPECT_EQ(lighting.find("EE_RENDER_INFO.shadow_split_"), std::string::npos);
  EXPECT_NE(storage.find("camera_info_block.shadow_split_distances ="), std::string::npos);
  EXPECT_NE(storage.find("camera_info_blocks_[camera_index].shadow_split_distances"), std::string::npos);

  CameraInfoBlock first;
  CameraInfoBlock second = first;
  EXPECT_FALSE(first != second);
  second.shadow_split_distances = glm::vec4(20.0f, 60.0f, 150.0f, 400.0f);
  EXPECT_TRUE(first != second);
}

TEST(CameraRenderTechnique, ZeroToOneDepthHelpersUseProjectionTranslation) {
  const auto cameras =
      ReadTextFile(SourcePath("EvoEngine_SDK/Internals/DefaultResources/Shaders/Includes/Cameras.glsl"));
  const auto translation = cameras.find("float b = EE_CAMERAS[camera_index].projection[3][2];");
  ASSERT_NE(translation, std::string::npos);
  EXPECT_NE(cameras.find("float b = EE_CAMERAS[camera_index].projection[3][2];", translation + 1), std::string::npos);
  EXPECT_NE(cameras.find("return abs(b / a);"), std::string::npos);
  EXPECT_NE(cameras.find("return abs(b / (a + 1.f));"), std::string::npos);
  EXPECT_EQ(cameras.find("float b = EE_CAMERAS[camera_index].projection[2][3];"), std::string::npos);
}

TEST(CameraRenderTechnique, DirectionalAndPunctualPcfCountsUseSeparateRenderInfoFields) {
  Application app;
  ApplicationContextScope scope(app);
  RenderSettings settings;
  settings.directional_pcf_sample_amount = 7;
  settings.pcf_sample_amount = 23;
  settings.indirect_lighting_debug_view = RenderSettings::IndirectLightingDebugView::SpecularVisibility;
  RenderInstanceStorage::RenderInfoBlock render_info;
  render_info.Apply(settings);
  EXPECT_EQ(render_info.shadow_debug_parameters.w, 7);
  EXPECT_EQ(render_info.pcf_sample_amount, 23);
  EXPECT_FLOAT_EQ(render_info.shadow_fade_parameters.y, 3.0f);
}

TEST(CameraRenderTechnique, CameraRenderModesRoundTripYaml) {
  Application app;
  ApplicationContextScope scope(app);
  app.Initialize(EmptyProjectSettings());

  const std::vector<Camera::CameraRenderMode> render_modes = {Camera::CameraRenderMode::Rasterization,
                                                              Camera::CameraRenderMode::RayTracing,
                                                              Camera::CameraRenderMode::RayQuery};
  for (const auto render_mode : render_modes) {
    Camera camera;
    camera.camera_render_mode = render_mode;
    camera.camera_settings.background_source = Camera::BackgroundSource::EnvironmentalMap;
    camera.camera_settings.shader_execution_reordering_mode = CameraSettings::ShaderExecutionReorderingMode::Enabled;
    camera.camera_settings.firefly_clamp_enabled = false;
    camera.camera_settings.firefly_clamp_threshold = 3.5f;
    camera.camera_settings.emissive_triangle_nee_enabled = false;
    camera.camera_settings.ray_debug_view = CameraSettings::RayDebugView::SpecularF0;
    camera.camera_settings.auto_spp_enabled = true;
    camera.camera_settings.auto_spp_min_samples = 8;
    camera.camera_settings.auto_spp_max_samples = 64;
    camera.camera_settings.auto_spp_convergence_threshold = 0.025f;
    YAML::Emitter out;
    out << YAML::BeginMap;
    Serialization::SerializeObject(out, static_cast<IPrivateComponent&>(camera));
    out << YAML::EndMap;

    const auto node = YAML::Load(out.c_str());
    ASSERT_TRUE(node["render_mode"]);
    EXPECT_EQ(node["render_mode"].as<std::string>(), Camera::GetCameraRenderModeName(render_mode));
    ASSERT_TRUE(node["background_source"]);
    EXPECT_EQ(node["background_source"].as<std::string>(), "Environmental Map");
    ASSERT_TRUE(node["shader_execution_reordering_mode"]);
    EXPECT_EQ(node["shader_execution_reordering_mode"].as<std::string>(), "Enabled");
    ASSERT_TRUE(node["firefly_clamp_enabled"]);
    EXPECT_FALSE(node["firefly_clamp_enabled"].as<bool>());
    ASSERT_TRUE(node["firefly_clamp_threshold"]);
    EXPECT_FLOAT_EQ(node["firefly_clamp_threshold"].as<float>(), 3.5f);
    ASSERT_TRUE(node["emissive_triangle_nee_enabled"]);
    EXPECT_FALSE(node["emissive_triangle_nee_enabled"].as<bool>());
    ASSERT_TRUE(node["ray_debug_view"]);
    EXPECT_EQ(node["ray_debug_view"].as<std::string>(), "Specular F0");
    ASSERT_TRUE(node["auto_spp_enabled"]);
    EXPECT_TRUE(node["auto_spp_enabled"].as<bool>());
    ASSERT_TRUE(node["auto_spp_min_samples"]);
    EXPECT_EQ(node["auto_spp_min_samples"].as<int>(), 8);
    ASSERT_TRUE(node["auto_spp_max_samples"]);
    EXPECT_EQ(node["auto_spp_max_samples"].as<int>(), 64);
    ASSERT_TRUE(node["auto_spp_convergence_threshold"]);
    EXPECT_FLOAT_EQ(node["auto_spp_convergence_threshold"].as<float>(), 0.025f);

    Camera restored_camera;
    restored_camera.camera_render_mode = Camera::CameraRenderMode::Rasterization;
    Serialization::DeserializeObject(node, static_cast<IPrivateComponent&>(restored_camera));
    EXPECT_EQ(restored_camera.camera_render_mode, render_mode);
    EXPECT_EQ(restored_camera.camera_settings.background_source, Camera::BackgroundSource::EnvironmentalMap);
    EXPECT_EQ(restored_camera.camera_settings.shader_execution_reordering_mode,
              CameraSettings::ShaderExecutionReorderingMode::Enabled);
    EXPECT_FALSE(restored_camera.camera_settings.firefly_clamp_enabled);
    EXPECT_FLOAT_EQ(restored_camera.camera_settings.firefly_clamp_threshold, 3.5f);
    EXPECT_FALSE(restored_camera.camera_settings.emissive_triangle_nee_enabled);
    EXPECT_EQ(restored_camera.camera_settings.ray_debug_view, CameraSettings::RayDebugView::SpecularF0);
    EXPECT_TRUE(restored_camera.camera_settings.auto_spp_enabled);
    EXPECT_EQ(restored_camera.camera_settings.auto_spp_min_samples, 8);
    EXPECT_EQ(restored_camera.camera_settings.auto_spp_max_samples, 64);
    EXPECT_FLOAT_EQ(restored_camera.camera_settings.auto_spp_convergence_threshold, 0.025f);
  }

  Camera invalid_numeric_camera;
  invalid_numeric_camera.camera_render_mode = Camera::CameraRenderMode::RayTracing;
  Serialization::DeserializeObject(YAML::Load("{render_mode: 2}"),
                                   static_cast<IPrivateComponent&>(invalid_numeric_camera));
  EXPECT_EQ(invalid_numeric_camera.camera_render_mode, Camera::CameraRenderMode::RayTracing);

  Camera numeric_background_camera;
  numeric_background_camera.camera_settings.background_source = Camera::BackgroundSource::ClearColor;
  Serialization::DeserializeObject(YAML::Load("{background_source: 4}"),
                                   static_cast<IPrivateComponent&>(numeric_background_camera));
  EXPECT_EQ(numeric_background_camera.camera_settings.background_source, Camera::BackgroundSource::ClearColor);

  Camera invalid_ser_camera;
  invalid_ser_camera.camera_settings.shader_execution_reordering_mode =
      CameraSettings::ShaderExecutionReorderingMode::Enabled;
  Serialization::DeserializeObject(YAML::Load("{shader_execution_reordering_mode: 1}"),
                                   static_cast<IPrivateComponent&>(invalid_ser_camera));
  EXPECT_EQ(invalid_ser_camera.camera_settings.shader_execution_reordering_mode,
            CameraSettings::ShaderExecutionReorderingMode::Enabled);

  Camera numeric_debug_camera;
  numeric_debug_camera.camera_settings.ray_debug_view = CameraSettings::RayDebugView::Emission;
  Serialization::DeserializeObject(YAML::Load("{ray_debug_view: 19}"),
                                   static_cast<IPrivateComponent&>(numeric_debug_camera));
  EXPECT_EQ(numeric_debug_camera.camera_settings.ray_debug_view, CameraSettings::RayDebugView::Emission);
}
