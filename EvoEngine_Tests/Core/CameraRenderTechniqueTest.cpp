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

TEST(CameraRenderTechnique, NamesAndAliasesExposeRasterRayTracingAndRayQuery) {
  const auto& names = Camera::GetCameraRenderModeNames();
  ASSERT_EQ(names.size(), Camera::kCameraRenderModeCount);
  EXPECT_EQ(names[static_cast<uint32_t>(Camera::CameraRenderMode::Rasterization)], "Rasterization");
  EXPECT_EQ(names[static_cast<uint32_t>(Camera::CameraRenderMode::RayTracing)], "RayTracing");
  EXPECT_EQ(names[static_cast<uint32_t>(Camera::CameraRenderMode::RayQuery)], "RayQuery");

  EXPECT_EQ(Camera::ParseCameraRenderMode("raster"), Camera::CameraRenderMode::Rasterization);
  EXPECT_EQ(Camera::ParseCameraRenderMode("RayTracing"), Camera::CameraRenderMode::RayTracing);
  EXPECT_EQ(Camera::ParseCameraRenderMode("ray-tracing"), Camera::CameraRenderMode::RayTracing);
  EXPECT_EQ(Camera::ParseCameraRenderMode("path tracing"), Camera::CameraRenderMode::RayTracing);
  EXPECT_EQ(Camera::ParseCameraRenderMode("RayQuery"), Camera::CameraRenderMode::RayQuery);
  EXPECT_EQ(Camera::ParseCameraRenderMode("ray query"), Camera::CameraRenderMode::RayQuery);
  EXPECT_EQ(Camera::ParseCameraRenderMode("2"), Camera::CameraRenderMode::RayQuery);
  EXPECT_EQ(Camera::NormalizeCameraRenderMode(999), Camera::CameraRenderMode::Rasterization);
  EXPECT_FALSE(Camera::IsRayCameraRenderMode(Camera::CameraRenderMode::Rasterization));
  EXPECT_TRUE(Camera::IsRayCameraRenderMode(Camera::CameraRenderMode::RayTracing));
  EXPECT_TRUE(Camera::IsRayCameraRenderMode(Camera::CameraRenderMode::RayQuery));

  const auto& ser_names = Camera::GetShaderExecutionReorderingModeNames();
  ASSERT_EQ(ser_names.size(), Camera::kShaderExecutionReorderingModeCount);
  EXPECT_EQ(ser_names[static_cast<uint32_t>(CameraSettings::ShaderExecutionReorderingMode::Disabled)], "Disabled");
  EXPECT_EQ(ser_names[static_cast<uint32_t>(CameraSettings::ShaderExecutionReorderingMode::Automatic)], "Automatic");
  EXPECT_EQ(ser_names[static_cast<uint32_t>(CameraSettings::ShaderExecutionReorderingMode::Enabled)], "Enabled");
  EXPECT_EQ(Camera::ParseShaderExecutionReorderingMode("off"), CameraSettings::ShaderExecutionReorderingMode::Disabled);
  EXPECT_EQ(Camera::ParseShaderExecutionReorderingMode("auto"),
            CameraSettings::ShaderExecutionReorderingMode::Automatic);
  EXPECT_EQ(Camera::ParseShaderExecutionReorderingMode("enabled"),
            CameraSettings::ShaderExecutionReorderingMode::Enabled);
  EXPECT_EQ(Camera::NormalizeShaderExecutionReorderingMode(999),
            CameraSettings::ShaderExecutionReorderingMode::Disabled);

  const auto& debug_views = Camera::GetRayDebugViewNames();
  ASSERT_EQ(debug_views.size(), Camera::kRayDebugViewCount);
  EXPECT_EQ(debug_views[static_cast<uint32_t>(CameraSettings::RayDebugView::Beauty)], "Beauty");
  EXPECT_EQ(debug_views[static_cast<uint32_t>(CameraSettings::RayDebugView::ValidationAtlas)], "Validation Atlas");
  for (uint32_t index = 0; index < debug_views.size(); ++index) {
    EXPECT_EQ(Camera::ParseRayDebugView(debug_views[index]), static_cast<CameraSettings::RayDebugView>(index));
  }
  EXPECT_EQ(Camera::ParseRayDebugView("specular-f0"), CameraSettings::RayDebugView::SpecularF0);
  EXPECT_EQ(Camera::ParseRayDebugView("alpha coverage"), CameraSettings::RayDebugView::AlphaCoverage);
  EXPECT_EQ(Camera::ParseRayDebugView("20"), CameraSettings::RayDebugView::ValidationAtlas);
  EXPECT_EQ(Camera::NormalizeRayDebugView(999), CameraSettings::RayDebugView::Beauty);
}

TEST(CameraRenderTechnique, CameraInfoBlockKeepsShaderArrayStrideAlignment) {
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
  RenderInstanceStorage::RenderInfoBlock render_info;
  render_info.Apply(settings);
  EXPECT_EQ(render_info.shadow_debug_parameters.w, 7);
  EXPECT_EQ(render_info.pcf_sample_amount, 23);
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

  Camera legacy_numeric_camera;
  Serialization::DeserializeObject(YAML::Load("{render_mode: 2}"),
                                   static_cast<IPrivateComponent&>(legacy_numeric_camera));
  EXPECT_EQ(legacy_numeric_camera.camera_render_mode, Camera::CameraRenderMode::RayQuery);

  Camera legacy_ser_camera;
  Serialization::DeserializeObject(YAML::Load("{shader_execution_reordering_mode: 1}"),
                                   static_cast<IPrivateComponent&>(legacy_ser_camera));
  EXPECT_EQ(legacy_ser_camera.camera_settings.shader_execution_reordering_mode,
            CameraSettings::ShaderExecutionReorderingMode::Automatic);

  Camera legacy_debug_camera;
  Serialization::DeserializeObject(YAML::Load("{ray_debug_view: 20}"),
                                   static_cast<IPrivateComponent&>(legacy_debug_camera));
  EXPECT_EQ(legacy_debug_camera.camera_settings.ray_debug_view, CameraSettings::RayDebugView::ValidationAtlas);
}

TEST(CameraRenderTechnique, RayQueryTechniquePlumbingHasDedicatedCameraPath) {
  const auto camera_header = ReadTextFile(SourcePath("EvoEngine_SDK/include/Rendering/Camera.hpp"));
  const auto camera_source = ReadTextFile(SourcePath("EvoEngine_SDK/src/Camera.cpp"));
  const auto platform_header = ReadTextFile(SourcePath("EvoEngine_SDK/include/Rendering/Platform/Platform.hpp"));
  const auto platform_source = ReadTextFile(SourcePath("EvoEngine_SDK/src/Platform.cpp"));
  const auto render_layer_source = ReadTextFile(SourcePath("EvoEngine_SDK/src/RenderLayer.cpp"));
  const auto render_graph_header = ReadTextFile(SourcePath("EvoEngine_SDK/include/Rendering/RenderGraph.hpp"));
  const auto ray_camera_pass_header =
      ReadTextFile(SourcePath("EvoEngine_SDK/include/Rendering/RenderPasses/RayTracingCameraPass.hpp"));
  const auto ray_camera_pass_source =
      ReadTextFile(SourcePath("EvoEngine_SDK/src/RenderPasses/RayTracingCameraPass.cpp"));
  const auto ray_query_shader =
      ReadTextFile(SourcePath("EvoEngine_SDK/Internals/DefaultResources/Shaders/Compute/RayQueryCamera.comp")) +
      ReadTextFile(SourcePath("EvoEngine_SDK/Internals/DefaultResources/Shaders/Includes/CameraRayIntegrator.glsl")) +
      ReadTextFile(
          SourcePath("EvoEngine_SDK/Internals/DefaultResources/Shaders/Includes/CameraRayQueryTraversal.glsl"));
  const auto ray_tracing_basic =
      ReadTextFile(SourcePath("EvoEngine_SDK/Internals/DefaultResources/Shaders/Includes/RayTracingBasic.glsl"));
  const auto editor_layer_source = ReadTextFile(SourcePath("EvoEngine_SDK/src/EditorLayer.cpp"));
  const auto inspection_source = ReadTextFile(SourcePath("EvoEngine_SDK/src/Editor/SDKInspectionAdapters.cpp"));
  const auto application_source = ReadTextFile(SourcePath("EvoEngine_SDK/src/Application.cpp"));
  const auto editor_source = ReadTextFile(SourcePath("EvoEngine_App/src/EvoEngineEditor.cpp"));

  EXPECT_NE(camera_header.find("RayQuery"), std::string::npos);
  EXPECT_NE(camera_header.find("ResolveCameraRenderMode"), std::string::npos);
  EXPECT_NE(camera_source.find("Camera render mode RayQuery"), std::string::npos);
  EXPECT_EQ(camera_source.find("is not implemented yet"), std::string::npos);
  EXPECT_NE(camera_source.find("requested_mode == CameraRenderMode::RayQuery && !Platform::RayQueryEnabled()"),
            std::string::npos);
  EXPECT_NE(platform_header.find("support_ray_query"), std::string::npos);
  EXPECT_NE(platform_header.find("support_acceleration_structure"), std::string::npos);
  EXPECT_NE(platform_header.find("RayQueryEnabled"), std::string::npos);
  EXPECT_NE(platform_header.find("RayAccelerationStructureEnabled"), std::string::npos);
  EXPECT_NE(platform_header.find("support_shader_execution_reordering"), std::string::npos);
  EXPECT_NE(platform_header.find("ShaderExecutionReorderingEnabled"), std::string::npos);
  EXPECT_NE(platform_source.find("VK_KHR_RAY_QUERY_EXTENSION_NAME"), std::string::npos);
  EXPECT_NE(platform_source.find("VkPhysicalDeviceRayQueryFeaturesKHR"), std::string::npos);
  EXPECT_NE(platform_source.find("ray_query_features.rayQuery == VK_TRUE"), std::string::npos);
  EXPECT_NE(platform_source.find("ray_tracing_pipeline_features.rayTracingPipeline == VK_TRUE"), std::string::npos);
  EXPECT_NE(platform_source.find("acceleration_structure_features.accelerationStructure == VK_TRUE"),
            std::string::npos);
  EXPECT_NE(platform_source.find("VK_NV_RAY_TRACING_INVOCATION_REORDER_EXTENSION_NAME"), std::string::npos);
  EXPECT_NE(platform_source.find("VkPhysicalDeviceRayTracingInvocationReorderFeaturesNV"), std::string::npos);
  EXPECT_NE(render_graph_header.find("ray_query_camera"), std::string::npos);
  EXPECT_NE(render_layer_source.find("ray_query_camera_pipeline_"), std::string::npos);
  EXPECT_NE(render_layer_source.find("Shaders/Compute/RayQueryCamera.comp"), std::string::npos);
  EXPECT_NE(render_layer_source.find("RayQueryCameraPass::CreateDescriptor()"), std::string::npos);
  EXPECT_NE(render_layer_source.find("RayQueryCameraPass::Execute"), std::string::npos);
  EXPECT_NE(render_layer_source.find("VK_SHADER_STAGE_COMPUTE_BIT"), std::string::npos);
  EXPECT_NE(render_layer_source.find("Platform::RayAccelerationStructureEnabled()"), std::string::npos);
  EXPECT_NE(ray_camera_pass_header.find("class RayQueryCameraPass final"), std::string::npos);
  EXPECT_NE(ray_camera_pass_source.find("ComputePipeline"), std::string::npos);
  EXPECT_NE(ray_camera_pass_source.find("RenderPassNames::ray_query_camera"), std::string::npos);
  EXPECT_NE(ray_camera_pass_source.find("parameters.pipeline->Dispatch"), std::string::npos);
  EXPECT_NE(ray_query_shader.find("#extension GL_EXT_ray_query : require"), std::string::npos);
  EXPECT_NE(ray_query_shader.find("rayQueryInitializeEXT"), std::string::npos);
  EXPECT_NE(ray_query_shader.find("rayQueryConfirmIntersectionEXT"), std::string::npos);
  EXPECT_NE(ray_query_shader.find("#define EE_GLTF_TEXTURE_LOD 0.0"), std::string::npos);
  EXPECT_NE(ray_query_shader.find("EE_CAMERA_TEXTURE_GRADIENTS"), std::string::npos);
  EXPECT_NE(ray_query_shader.find("out uint material_index, out vec2 tex_coord_0, out vec2 tex_coord_1"),
            std::string::npos);
  EXPECT_NE(ray_query_shader.find("out vec2 tex_coord_2, out vec2 tex_coord_3"), std::string::npos);
  EXPECT_NE(ray_query_shader.find("EE_EVALUATE_GLTF_RAY_TRACING_PBR_MATERIAL"), std::string::npos);
  EXPECT_NE(ray_query_shader.find("imageStore(result_image, ivec2(pixel_coordinate), vec4(linear_radiance"),
            std::string::npos);
  EXPECT_NE(ray_tracing_basic.find("#ifndef EE_RAY_QUERY_SHADER"), std::string::npos);
  EXPECT_NE(editor_layer_source.find("Camera::GetCameraRenderModeNames()"), std::string::npos);
  EXPECT_NE(inspection_source.find("Camera::GetCameraRenderModeNames()"), std::string::npos);
  EXPECT_NE(inspection_source.find("Shader Execution Reordering"), std::string::npos);
  EXPECT_NE(inspection_source.find("SER unavailable; using standard ray tracing scheduling."), std::string::npos);
  EXPECT_NE(inspection_source.find("Firefly clamp"), std::string::npos);
  EXPECT_NE(inspection_source.find("Ray Debug View"), std::string::npos);
  EXPECT_NE(inspection_source.find("Firefly threshold"), std::string::npos);
  EXPECT_NE(inspection_source.find("Auto SPP"), std::string::npos);
  EXPECT_NE(inspection_source.find("Auto min SPP"), std::string::npos);
  EXPECT_NE(inspection_source.find("Auto max SPP"), std::string::npos);
  EXPECT_NE(inspection_source.find("Auto threshold"), std::string::npos);
  EXPECT_NE(application_source.find("render_mode"), std::string::npos);
  EXPECT_NE(application_source.find("shader_execution_reordering_mode"), std::string::npos);
  EXPECT_NE(application_source.find("firefly_clamp_enabled"), std::string::npos);
  EXPECT_NE(application_source.find("firefly_clamp_threshold"), std::string::npos);
  EXPECT_NE(application_source.find("auto_spp_enabled"), std::string::npos);
  EXPECT_NE(application_source.find("auto_spp_convergence_threshold"), std::string::npos);
  EXPECT_NE(application_source.find("ray_debug_view"), std::string::npos);
  EXPECT_NE(editor_layer_source.find("firefly_clamp_enabled"), std::string::npos);
  EXPECT_NE(editor_layer_source.find("firefly_clamp_threshold"), std::string::npos);
  EXPECT_NE(editor_layer_source.find("auto_spp_enabled"), std::string::npos);
  EXPECT_NE(editor_layer_source.find("auto_spp_convergence_threshold"), std::string::npos);
  EXPECT_NE(editor_layer_source.find("ray_debug_view"), std::string::npos);
  EXPECT_NE(editor_source.find("Camera::ParseCameraRenderMode(value"), std::string::npos);
  EXPECT_NE(editor_source.find("--preview-firefly-clamp"), std::string::npos);
  EXPECT_NE(editor_source.find("--preview-firefly-clamp-threshold"), std::string::npos);
  EXPECT_NE(editor_source.find("--preview-auto-spp"), std::string::npos);
  EXPECT_NE(editor_source.find("--preview-auto-spp-min-samples"), std::string::npos);
  EXPECT_NE(editor_source.find("--preview-auto-spp-max-samples"), std::string::npos);
  EXPECT_NE(editor_source.find("--preview-auto-spp-threshold"), std::string::npos);
  EXPECT_NE(editor_source.find("--preview-ray-debug"), std::string::npos);
  EXPECT_NE(editor_source.find("metrics[\"ray_debug_view\"]"), std::string::npos);
  EXPECT_NE(editor_source.find("--disable-ray-tracing-pipeline"), std::string::npos);
  EXPECT_NE(editor_source.find("capabilities.support_acceleration_structure"), std::string::npos);
  EXPECT_NE(editor_source.find("capabilities.support_ray_tracing"), std::string::npos);
  EXPECT_NE(editor_source.find("capabilities.support_ray_query"), std::string::npos);
  EXPECT_NE(editor_source.find("ParsePreviewBool"), std::string::npos);
}

TEST(CameraRenderTechnique, RayQuerySharedResourcesDoNotRequireRayTracingPipeline) {
  const auto camera_source = ReadTextFile(SourcePath("EvoEngine_SDK/src/Camera.cpp"));
  const auto graphics_resources = ReadTextFile(SourcePath("EvoEngine_SDK/src/GraphicsResources.cpp"));
  const auto mesh_source = ReadTextFile(SourcePath("EvoEngine_SDK/src/Mesh.cpp"));
  const auto skinned_mesh_source = ReadTextFile(SourcePath("EvoEngine_SDK/src/SkinnedMesh.cpp"));
  const auto skinned_renderer_source = ReadTextFile(SourcePath("EvoEngine_SDK/src/SkinnedMeshRenderer.cpp"));
  const auto render_storage_source = ReadTextFile(SourcePath("EvoEngine_SDK/src/RenderInstanceStorage.cpp"));
  const auto render_layer_source = ReadTextFile(SourcePath("EvoEngine_SDK/src/RenderLayer.cpp"));

  EXPECT_NE(camera_source.find("Platform::RayQueryEnabled() ? CameraRenderMode::RayQuery"), std::string::npos);
  EXPECT_NE(graphics_resources.find("VkPipelineStageFlags2 RayTraversalStageMask()"), std::string::npos);
  EXPECT_NE(graphics_resources.find("if (Platform::RayTracingEnabled())"), std::string::npos);
  EXPECT_NE(graphics_resources.find("if (Platform::RayQueryEnabled())"), std::string::npos);
  EXPECT_NE(mesh_source.find("if (Platform::RayAccelerationStructureEnabled())"), std::string::npos);
  EXPECT_NE(skinned_mesh_source.find("if (Platform::RayAccelerationStructureEnabled())"), std::string::npos);
  EXPECT_NE(skinned_renderer_source.find("if (!Platform::RayAccelerationStructureEnabled()"), std::string::npos);
  EXPECT_NE(render_storage_source.find("if (!Platform::RayAccelerationStructureEnabled()"), std::string::npos);
  EXPECT_NE(render_layer_source.find("update_ray_tracing && Platform::RayAccelerationStructureEnabled()"),
            std::string::npos);
  EXPECT_NE(render_layer_source.find("if (Platform::RayAccelerationStructureEnabled() &&"), std::string::npos);
  EXPECT_NE(render_layer_source.find("if (Platform::RayQueryEnabled() && !ray_query_camera_fallback_pipeline_)"),
            std::string::npos);
  EXPECT_NE(render_layer_source.find("if (Platform::RayTracingEnabled() && !ray_tracing_camera_fallback_pipeline_)"),
            std::string::npos);
}
