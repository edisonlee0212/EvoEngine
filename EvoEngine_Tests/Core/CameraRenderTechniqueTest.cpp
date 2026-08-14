#include "EvoEngine_SDK_PCH.hpp"

#include <gtest/gtest.h>

#include <array>
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

  CameraSettings default_settings;
  EXPECT_EQ(default_settings.shader_execution_reordering_mode,
            CameraSettings::ShaderExecutionReorderingMode::Automatic);
  EXPECT_EQ(default_settings.sample_size, 1);
  EXPECT_FLOAT_EQ(default_settings.firefly_clamp_threshold, 10.0f);

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

  EXPECT_FALSE(default_settings.ray_outputs.AnyEnabled());
}

TEST(CameraRenderTechnique, RayOutputLayoutKeepsCpuAndShadersAligned) {
  EXPECT_EQ(static_cast<uint32_t>(RayCameraOptionalOutput::Albedo), 0u);
  EXPECT_EQ(static_cast<uint32_t>(RayCameraOptionalOutput::Normal), 1u);
  EXPECT_EQ(static_cast<uint32_t>(RayCameraOptionalOutput::RayCount), 2u);
  EXPECT_EQ(static_cast<uint32_t>(RayCameraOptionalOutput::PathLength), 3u);
  EXPECT_EQ(static_cast<uint32_t>(RayCameraOptionalOutput::Time), 4u);
  EXPECT_EQ(static_cast<uint32_t>(RayCameraOptionalOutput::Debug), 5u);
  EXPECT_EQ(kRayCameraOptionalOutputCount, 6u);
  EXPECT_EQ(kRayCameraOutputDescriptorBindingCount, 10u);

  const auto native_cameras =
      ReadTextFile(SourcePath("EvoEngine_SDK/Internals/DefaultResources/Shaders/Modules/EvoEngine/Cameras.slang"));
  const auto compatibility_cameras = ReadTextFile(
      SourcePath("EvoEngine_Packages/EcoSysLab/Internals/EcoSysLabResources/Shaders/Includes/Cameras.glsl"));
  const auto output_shader = ReadTextFile(
      SourcePath("EvoEngine_SDK/Internals/DefaultResources/Shaders/Modules/EvoEngine/CameraRayOutputs.slang"));
  const std::array output_names{"ALBEDO", "NORMAL", "RAY_COUNT", "PATH_LENGTH", "TIME", "DEBUG"};
  for (uint32_t index = 0u; index < output_names.size(); ++index) {
    const auto flag =
        std::string("EE_CAMERA_RAY_OUTPUT_") + output_names[index] + " = 1u << " + std::to_string(index) + "u";
    EXPECT_NE(native_cameras.find(flag), std::string::npos) << output_names[index];
    EXPECT_NE(compatibility_cameras.find(flag), std::string::npos) << output_names[index];
    const auto binding =
        "[[vk::binding(" + std::to_string(kRayCameraOutputDescriptorBaseBindingCount + index) + ", 2)]]";
    EXPECT_NE(output_shader.find(binding), std::string::npos) << output_names[index];
  }
}

TEST(CameraRenderTechnique, CameraInfoBlockKeepsShaderArrayStrideAlignment) {
  EXPECT_EQ(offsetof(CameraInfoBlock, raster_lighting_flags), 812u);
  EXPECT_EQ(offsetof(CameraInfoBlock, ray_output_flags), 816u);
  EXPECT_EQ(offsetof(CameraInfoBlock, camera_block_reserved0), 820u);
  EXPECT_EQ(offsetof(CameraInfoBlock, camera_block_reserved1), 824u);
  EXPECT_EQ(offsetof(CameraInfoBlock, camera_block_reserved2), 828u);
  EXPECT_EQ(offsetof(CameraInfoBlock, shadow_split_distances), 832u);
  EXPECT_EQ(sizeof(CameraInfoBlock), 848u);
  const auto camera_source = ReadTextFile(SourcePath("EvoEngine_SDK/src/Camera.cpp"));
  const auto cameras_header =
      ReadTextFile(SourcePath("EvoEngine_SDK/Internals/DefaultResources/Shaders/Modules/EvoEngine/Cameras.slang"));
  EXPECT_NE(camera_source.find("camera_info_block.ray_output_flags"), std::string::npos);
  EXPECT_NE(cameras_header.find("uint ray_output_flags"), std::string::npos);
  EXPECT_NE(cameras_header.find("uint camera_block_reserved0"), std::string::npos);
  EXPECT_NE(cameras_header.find("uint camera_block_reserved1"), std::string::npos);
  EXPECT_NE(cameras_header.find("uint camera_block_reserved2"), std::string::npos);
  EXPECT_NE(cameras_header.find("uint camera_block_reserved3"), std::string::npos);
  EXPECT_NE(cameras_header.find("uint camera_block_reserved4"), std::string::npos);
  EXPECT_NE(cameras_header.find("EE_CAMERA_RAY_OUTPUT_DEBUG"), std::string::npos);
  EXPECT_LT(offsetof(CameraInfoBlock, previous_projection_view),
            offsetof(CameraInfoBlock, previous_inverse_projection));
  EXPECT_LT(offsetof(CameraInfoBlock, previous_inverse_projection), offsetof(CameraInfoBlock, previous_inverse_view));
  EXPECT_LT(offsetof(CameraInfoBlock, previous_inverse_view), offsetof(CameraInfoBlock, unjittered_projection_view));
  EXPECT_LT(offsetof(CameraInfoBlock, camera_block_reserved3), offsetof(CameraInfoBlock, gamma));
  EXPECT_LT(offsetof(CameraInfoBlock, gamma), offsetof(CameraInfoBlock, sample_size));
  EXPECT_LT(offsetof(CameraInfoBlock, sample_size), offsetof(CameraInfoBlock, bounce));
  EXPECT_LT(offsetof(CameraInfoBlock, bounce), offsetof(CameraInfoBlock, firefly_clamp_threshold));
  EXPECT_LT(offsetof(CameraInfoBlock, firefly_clamp_threshold), offsetof(CameraInfoBlock, auto_spp_enabled));
  EXPECT_LT(offsetof(CameraInfoBlock, auto_spp_enabled), offsetof(CameraInfoBlock, auto_spp_min_samples));
  EXPECT_LT(offsetof(CameraInfoBlock, auto_spp_min_samples), offsetof(CameraInfoBlock, auto_spp_max_samples));
  EXPECT_LT(offsetof(CameraInfoBlock, auto_spp_max_samples), offsetof(CameraInfoBlock, auto_spp_convergence_threshold));
  EXPECT_LT(offsetof(CameraInfoBlock, auto_spp_convergence_threshold),
            offsetof(CameraInfoBlock, camera_block_reserved4));
  EXPECT_LT(offsetof(CameraInfoBlock, camera_block_reserved4), offsetof(CameraInfoBlock, ray_debug_view));
  EXPECT_LT(offsetof(CameraInfoBlock, ray_debug_view), offsetof(CameraInfoBlock, raster_lighting_flags));
}

TEST(CameraRenderTechnique, GtaoSpecularVisibilityUsesTheExistingCameraBlockLane) {
  EXPECT_EQ(CameraInfoBlock::kRasterLightingGtaoVisibility, 1u);
  CameraInfoBlock first;
  CameraInfoBlock second = first;
  second.ray_output_flags = 1u;
  EXPECT_TRUE(first != second);
  second = first;
  second.raster_lighting_flags = CameraInfoBlock::kRasterLightingGtaoVisibility;
  EXPECT_TRUE(first != second);

  const auto cameras =
      ReadTextFile(SourcePath("EvoEngine_SDK/Internals/DefaultResources/Shaders/Modules/EvoEngine/Cameras.slang"));
  const auto lighting =
      ReadTextFile(SourcePath("EvoEngine_SDK/Internals/DefaultResources/Shaders/Modules/EvoEngine/Lighting.slang"));
  const auto camera_source = ReadTextFile(SourcePath("EvoEngine_SDK/src/Camera.cpp"));
  EXPECT_NE(cameras.find("uint raster_lighting_flags"), std::string::npos);
  EXPECT_NE(lighting.find("raster_lighting_flags & 1u"), std::string::npos);
  EXPECT_NE(camera_source.find("AmbientOcclusion::Algorithm::Gtao"), std::string::npos);
}

TEST(CameraRenderTechnique, DirectionalShadowSplitsUseTheSelectedCameraBlock) {
  const auto cameras =
      ReadTextFile(SourcePath("EvoEngine_SDK/Internals/DefaultResources/Shaders/Modules/EvoEngine/Cameras.slang"));
  const auto lighting =
      ReadTextFile(SourcePath("EvoEngine_SDK/Internals/DefaultResources/Shaders/Modules/EvoEngine/Lighting.slang"));
  const auto storage = ReadTextFile(SourcePath("EvoEngine_SDK/src/RenderInstanceStorage.cpp"));
  EXPECT_NE(cameras.find("float4 shadow_split_distances"), std::string::npos);
  EXPECT_NE(lighting.find("EE_CAMERAS[EE_BASIC_CONSTANTS.camera_index].shadow_split_distances"), std::string::npos);
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
      ReadTextFile(SourcePath("EvoEngine_SDK/Internals/DefaultResources/Shaders/Modules/EvoEngine/Cameras.slang"));
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

  settings.indirect_lighting_debug_view = RenderSettings::IndirectLightingDebugView::DdgiProbeBlendLoss;
  render_info.Apply(settings);
  EXPECT_FLOAT_EQ(render_info.shadow_fade_parameters.y, 5.0f);
}

TEST(CameraRenderTechnique, CameraRenderModesRoundTripYaml) {
  Application app;
  ApplicationContextScope scope(app);
  app.Initialize(EmptyProjectSettings());

  for (const auto render_mode : {Camera::CameraRenderMode::Rasterization, Camera::CameraRenderMode::RayTracing,
                                 Camera::CameraRenderMode::RayQuery}) {
    Camera camera;
    camera.camera_render_mode = render_mode;
    camera.camera_settings.background_source = Camera::BackgroundSource::EnvironmentalMap;
    camera.camera_settings.shader_execution_reordering_mode = CameraSettings::ShaderExecutionReorderingMode::Enabled;
    camera.camera_settings.firefly_clamp_threshold = 3.5f;
    camera.camera_settings.ray_debug_view = CameraSettings::RayDebugView::SpecularF0;
    camera.camera_settings.auto_spp_enabled = true;
    camera.camera_settings.auto_spp_min_samples = 8;
    camera.camera_settings.auto_spp_max_samples = 64;
    camera.camera_settings.auto_spp_convergence_threshold = 0.025f;
    camera.camera_settings.ray_outputs.albedo = true;
    camera.camera_settings.ray_outputs.debug = true;

    YAML::Emitter emitter;
    emitter << YAML::BeginMap;
    Serialization::SerializeObject(emitter, static_cast<IPrivateComponent&>(camera));
    emitter << YAML::EndMap;
    const auto node = YAML::Load(emitter.c_str());
    EXPECT_EQ(node["render_mode"].as<std::string>(), Camera::GetCameraRenderModeName(render_mode));
    EXPECT_FALSE(node["ray_integrator"]);
    EXPECT_FALSE(node["restir_pt"]);
    EXPECT_FALSE(node["firefly_clamp_enabled"]);
    EXPECT_FALSE(node["emissive_triangle_nee_enabled"]);
    EXPECT_FALSE(node["ray_outputs"]["nrd_emission"]);

    Camera restored;
    Serialization::DeserializeObject(node, static_cast<IPrivateComponent&>(restored));
    EXPECT_EQ(restored.camera_render_mode, render_mode);
    EXPECT_EQ(restored.camera_settings.background_source, Camera::BackgroundSource::EnvironmentalMap);
    EXPECT_EQ(restored.camera_settings.shader_execution_reordering_mode,
              CameraSettings::ShaderExecutionReorderingMode::Enabled);
    EXPECT_FLOAT_EQ(restored.camera_settings.firefly_clamp_threshold, 3.5f);
    EXPECT_EQ(restored.camera_settings.ray_debug_view, CameraSettings::RayDebugView::SpecularF0);
    EXPECT_TRUE(restored.camera_settings.auto_spp_enabled);
    EXPECT_EQ(restored.camera_settings.auto_spp_min_samples, 8);
    EXPECT_EQ(restored.camera_settings.auto_spp_max_samples, 64);
    EXPECT_FLOAT_EQ(restored.camera_settings.auto_spp_convergence_threshold, 0.025f);
    EXPECT_TRUE(restored.camera_settings.ray_outputs.albedo);
    EXPECT_TRUE(restored.camera_settings.ray_outputs.debug);

    auto legacy_node = YAML::Load(emitter.c_str());
    legacy_node["ray_integrator"] = "ReSTIR PT";
    legacy_node["restir_pt"]["enable_temporal_reuse"] = true;
    legacy_node["firefly_clamp_enabled"] = false;
    legacy_node["emissive_triangle_nee_enabled"] = false;
    legacy_node["ray_outputs"]["nrd_diffuse_radiance_hit_distance"] = true;
    legacy_node["ray_outputs"]["nrd_specular_radiance_hit_distance"] = true;
    legacy_node["ray_outputs"]["nrd_residual_radiance_hit_distance"] = true;
    legacy_node["ray_outputs"]["nrd_emission"] = true;
    legacy_node["ray_outputs"]["nrd_diffuse_reflectance"] = true;
    legacy_node["ray_outputs"]["nrd_specular_reflectance"] = true;
    Camera legacy_restored;
    EXPECT_NO_THROW(Serialization::DeserializeObject(legacy_node, static_cast<IPrivateComponent&>(legacy_restored)));
    EXPECT_EQ(legacy_restored.camera_render_mode, render_mode);
    EXPECT_FLOAT_EQ(legacy_restored.camera_settings.firefly_clamp_threshold, 3.5f);
    EXPECT_TRUE(legacy_restored.camera_settings.ray_outputs.albedo);
    EXPECT_TRUE(legacy_restored.camera_settings.ray_outputs.debug);
  }
}
