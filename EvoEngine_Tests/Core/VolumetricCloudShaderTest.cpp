#include "EvoEngine_SDK_PCH.hpp"

#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>

#include <gtest/gtest.h>

namespace {
std::string ReadTextFile(const std::filesystem::path& path) {
  std::ifstream file(path);
  EXPECT_TRUE(file.good()) << path.string();
  return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}

size_t CountOccurrences(const std::string& source, const std::string& pattern) {
  size_t count = 0;
  size_t offset = 0;
  while ((offset = source.find(pattern, offset)) != std::string::npos) {
    ++count;
    offset += pattern.size();
  }
  return count;
}
}  // namespace

TEST(VolumetricCloudShader, SharedLibraryDefinesV1Contract) {
  const auto shader_source =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "Internals" /
                   "DefaultResources" / "Shaders" / "Includes" / "VolumetricClouds.glsl");
  ASSERT_FALSE(shader_source.empty());

  EXPECT_NE(shader_source.find("struct VolumetricCloudSettingsGpu"), std::string::npos);
  EXPECT_NE(shader_source.find("struct VolumetricCloudMarchResult"), std::string::npos);
  EXPECT_NE(shader_source.find("struct VolumetricCloudDensitySample"), std::string::npos);
  EXPECT_NE(shader_source.find("EE_VOLUMETRIC_CLOUD_SampleDensity"), std::string::npos);
  EXPECT_NE(shader_source.find("EE_VOLUMETRIC_CLOUD_SampleDensityComponents"), std::string::npos);
  EXPECT_NE(shader_source.find("EE_VOLUMETRIC_CLOUD_HenyeyGreensteinPhase"), std::string::npos);
  EXPECT_NE(shader_source.find("EE_VOLUMETRIC_CLOUD_DirectionalPhase"), std::string::npos);
  EXPECT_NE(shader_source.find("EE_VOLUMETRIC_CLOUD_PowderEffect"), std::string::npos);
  EXPECT_NE(shader_source.find("EE_VOLUMETRIC_CLOUD_EdgeLighting"), std::string::npos);
  EXPECT_NE(shader_source.find("EE_VOLUMETRIC_CLOUD_TemporalJitter"), std::string::npos);
  EXPECT_NE(shader_source.find("EE_VOLUMETRIC_CLOUD_LightTransmittance"), std::string::npos);
  EXPECT_NE(shader_source.find("EE_VOLUMETRIC_CLOUD_LightDensity"), std::string::npos);
  EXPECT_NE(shader_source.find("EE_VOLUMETRIC_CLOUD_SurfaceShadowTransmittance"), std::string::npos);
  EXPECT_NE(shader_source.find("EE_VOLUMETRIC_CLOUD_IntersectLayerWithCenter"), std::string::npos);
  EXPECT_NE(shader_source.find("EE_VOLUMETRIC_CLOUD_March"), std::string::npos);
  EXPECT_NE(shader_source.find("EE_VOLUMETRIC_CLOUD_Composite"), std::string::npos);
  EXPECT_NE(shader_source.find("EE_VOLUMETRIC_CLOUD_Debug"), std::string::npos);
  EXPECT_NE(shader_source.find("EE_VOLUMETRIC_CLOUD_Remap"), std::string::npos);
  EXPECT_NE(shader_source.find("EE_VOLUMETRIC_CLOUD_RaySphereIntersection"), std::string::npos);
  EXPECT_NE(shader_source.find("EE_VOLUMETRIC_CLOUD_ProjectedShellPoint"), std::string::npos);
  EXPECT_NE(shader_source.find("EE_VOLUMETRIC_CLOUD_CloudLayerDensity"), std::string::npos);
  EXPECT_NE(shader_source.find("EE_VOLUMETRIC_CLOUD_HeightBiasCoverage"), std::string::npos);
  EXPECT_NE(shader_source.find("EE_VOLUMETRIC_CLOUD_ConeSample"), std::string::npos);
  EXPECT_NE(shader_source.find("EE_VOLUMETRIC_CLOUD_DebugAccumulation"), std::string::npos);
  EXPECT_NE(shader_source.find("uniform sampler3D EE_VOLUMETRIC_CLOUD_BASE_SHAPE_NOISE"), std::string::npos);
  EXPECT_NE(shader_source.find("uniform sampler3D EE_VOLUMETRIC_CLOUD_DETAIL_EROSION_NOISE"), std::string::npos);
  EXPECT_NE(shader_source.find("uniform sampler2D EE_VOLUMETRIC_CLOUD_WEATHER_COVERAGE"), std::string::npos);
  EXPECT_NE(shader_source.find("uniform sampler2D EE_VOLUMETRIC_CLOUD_CURL_NOISE"), std::string::npos);
  EXPECT_NE(shader_source.find("EE_VOLUMETRIC_CLOUD_SampleBaseShapeNoise"), std::string::npos);
  EXPECT_NE(shader_source.find("EE_VOLUMETRIC_CLOUD_SampleDetailErosionNoise"), std::string::npos);
  EXPECT_NE(shader_source.find("EE_VOLUMETRIC_CLOUD_SampleWeatherCoverage"), std::string::npos);
  EXPECT_NE(shader_source.find("texture(EE_VOLUMETRIC_CLOUD_BASE_SHAPE_NOISE"), std::string::npos);
  EXPECT_NE(shader_source.find("texture(EE_VOLUMETRIC_CLOUD_DETAIL_EROSION_NOISE"), std::string::npos);
  EXPECT_NE(shader_source.find("texture(EE_VOLUMETRIC_CLOUD_WEATHER_COVERAGE"), std::string::npos);
  EXPECT_NE(shader_source.find("texture(EE_VOLUMETRIC_CLOUD_CURL_NOISE"), std::string::npos);
  EXPECT_NE(shader_source.find("low_resolution_density"), std::string::npos);
  EXPECT_NE(shader_source.find("high_resolution_march"), std::string::npos);
  EXPECT_NE(shader_source.find("coarse_step_length"), std::string::npos);
  EXPECT_NE(shader_source.find("fine_step_length"), std::string::npos);
  EXPECT_NE(shader_source.find("fallback_misses"), std::string::npos);
  EXPECT_NE(shader_source.find("low_shape_density"), std::string::npos);
  EXPECT_NE(shader_source.find("local_coverage"), std::string::npos);
  EXPECT_NE(shader_source.find("weather_density"), std::string::npos);
  EXPECT_NE(shader_source.find("mix(0.35f, 1.35f, weather.r)"), std::string::npos);
  EXPECT_NE(shader_source.find("light_transmittance * phase * in_scattering * powder + edge_lighting"),
            std::string::npos);
  EXPECT_NE(shader_source.find("ambient_lighting + sun_lighting"), std::string::npos);
  EXPECT_NE(shader_source.find("mean_base_shape"), std::string::npos);
  EXPECT_NE(shader_source.find("mean_detail_erosion"), std::string::npos);
  EXPECT_NE(shader_source.find("mean_weather_coverage"), std::string::npos);
  EXPECT_NE(shader_source.find("cloud_type"), std::string::npos);
  EXPECT_NE(shader_source.find("height_coverage"), std::string::npos);
  EXPECT_NE(shader_source.find("EE_VOLUMETRIC_CLOUD_Remap(low_resolution_density, high_res_erosion"),
            std::string::npos);
  EXPECT_EQ(shader_source.find("VolumetricCloudDensitySample sample"), std::string::npos);
  EXPECT_EQ(shader_source.find("cloud_shape - coverage_threshold"), std::string::npos);
  EXPECT_NE(shader_source.find("base_noise_scale"), std::string::npos);
  EXPECT_NE(shader_source.find("detail_noise_scale"), std::string::npos);
  EXPECT_NE(shader_source.find("extinction_scale"), std::string::npos);
  EXPECT_NE(shader_source.find("use_spherical_atmosphere"), std::string::npos);
  EXPECT_NE(shader_source.find("atmosphere_radius"), std::string::npos);
  EXPECT_NE(shader_source.find("curl_strength"), std::string::npos);
  EXPECT_NE(shader_source.find("coarse_step_fraction"), std::string::npos);
  EXPECT_NE(shader_source.find("max_march_distance"), std::string::npos);
  EXPECT_NE(shader_source.find("enable_cloud_shadows"), std::string::npos);
  EXPECT_NE(shader_source.find("cloud_shadow_strength"), std::string::npos);
  EXPECT_NE(shader_source.find("cloud_shadow_step_count"), std::string::npos);
  EXPECT_NE(shader_source.find("settings.cloud_shadow_strength <= 0.0f"), std::string::npos);
  EXPECT_NE(shader_source.find("EE_VOLUMETRIC_CLOUD_SampleDensityComponents(settings, sample_position, "
                               "atmosphere_center, time_seconds, 0)"),
            std::string::npos);
  EXPECT_NE(shader_source.find("max(settings.extinction_scale"), std::string::npos);
}

TEST(VolumetricCloudShader, SharedLibraryStaysIndependentFromDdgiAndShaderForks) {
  const auto shader_source =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "Internals" /
                   "DefaultResources" / "Shaders" / "Includes" / "VolumetricClouds.glsl");
  ASSERT_FALSE(shader_source.empty());

  EXPECT_EQ(shader_source.find("DDGI"), std::string::npos);
  EXPECT_EQ(shader_source.find("HLSL"), std::string::npos);
  EXPECT_EQ(shader_source.find(".hlsl"), std::string::npos);
  EXPECT_EQ(shader_source.find("#ifdef RASTER"), std::string::npos);
  EXPECT_EQ(shader_source.find("#ifdef RAY_TRACING"), std::string::npos);
}

TEST(VolumetricCloudShader, RasterComputeUsesSharedLibraryAndRasterDepth) {
  const auto shader_source =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "Internals" /
                   "DefaultResources" / "Shaders" / "Compute" / "VolumetricClouds.comp");
  ASSERT_FALSE(shader_source.empty());

  EXPECT_NE(shader_source.find("#include \"VolumetricClouds.glsl\""), std::string::npos);
  EXPECT_NE(shader_source.find("layout(set = 1, binding = 0) uniform sampler2D inDepth"), std::string::npos);
  EXPECT_NE(shader_source.find("layout(set = 1, binding = 2, rgba16f) uniform writeonly image2D outCloudAccumulation"),
            std::string::npos);
  EXPECT_NE(shader_source.find("layout(set = 1, binding = 3, r16f) uniform writeonly image2D outCloudTransmittance"),
            std::string::npos);
  EXPECT_NE(shader_source.find("imageSize(outCloudAccumulation)"), std::string::npos);
  EXPECT_NE(shader_source.find("EE_DEPTH_TO_WORLD_POS"), std::string::npos);
  EXPECT_NE(shader_source.find("(flags.w & 1) != 0"), std::string::npos);
  EXPECT_NE(shader_source.find("hit_distance >= camera_far * 0.999f"), std::string::npos);
  EXPECT_NE(shader_source.find("noiseExtinctionMarchDistance.w"), std::string::npos);
  EXPECT_NE(shader_source.find("atmosphereCloudTypeCurl"), std::string::npos);
  EXPECT_NE(shader_source.find("marchControl"), std::string::npos);
  EXPECT_NE(shader_source.find("settings.use_spherical_atmosphere"), std::string::npos);
  EXPECT_NE(shader_source.find("EE_DIRECTIONAL_LIGHTS"), std::string::npos);
  EXPECT_NE(shader_source.find("EE_VOLUMETRIC_CLOUD_March"), std::string::npos);
  EXPECT_NE(shader_source.find("EE_VOLUMETRIC_CLOUD_DebugAccumulation"), std::string::npos);
  EXPECT_NE(shader_source.find("cloud.march_distance"), std::string::npos);
  EXPECT_EQ(shader_source.find("imageStore(inOutColor"), std::string::npos);
  EXPECT_EQ(shader_source.find("DDGI"), std::string::npos);
  EXPECT_EQ(shader_source.find("HLSL"), std::string::npos);
}

TEST(VolumetricCloudShader, CompositeComputeUpsamplesDepthAwareClouds) {
  const auto shader_source =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "Internals" /
                   "DefaultResources" / "Shaders" / "Compute" / "VolumetricCloudsComposite.comp");
  ASSERT_FALSE(shader_source.empty());

  EXPECT_NE(shader_source.find("#include \"VolumetricClouds.glsl\""), std::string::npos);
  EXPECT_NE(shader_source.find("layout(set = 1, binding = 1, rgba32f) uniform image2D inOutColor"), std::string::npos);
  EXPECT_NE(shader_source.find("layout(set = 1, binding = 4) uniform sampler2D inCloudAccumulation"),
            std::string::npos);
  EXPECT_NE(shader_source.find("layout(set = 1, binding = 5) uniform sampler2D inCloudTransmittance"),
            std::string::npos);
  EXPECT_NE(shader_source.find("layout(set = 1, binding = 10) uniform sampler2D inPreviousCloudAccumulation"),
            std::string::npos);
  EXPECT_NE(shader_source.find("layout(set = 1, binding = 11) uniform sampler2D inPreviousCloudTransmittance"),
            std::string::npos);
  EXPECT_NE(shader_source.find("layout(set = 1, binding = 12, rgba16f) uniform image2D "
                               "outCloudAccumulationHistory"),
            std::string::npos);
  EXPECT_NE(shader_source.find("layout(set = 1, binding = 13, r16f) uniform image2D "
                               "outCloudTransmittanceHistory"),
            std::string::npos);
  EXPECT_NE(shader_source.find("EE_VOLUMETRIC_CLOUD_DEPTH_AWARE_TEXEL"), std::string::npos);
  EXPECT_NE(shader_source.find("EE_VOLUMETRIC_CLOUD_APPLY_TEMPORAL_HISTORY"), std::string::npos);
  EXPECT_NE(shader_source.find("previous_projection_view"), std::string::npos);
  EXPECT_NE(shader_source.find("texture(inPreviousCloudAccumulation, previous_tex_coord)"), std::string::npos);
  EXPECT_NE(shader_source.find("imageStore(outCloudAccumulationHistory"), std::string::npos);
  EXPECT_NE(shader_source.find("EE_VOLUMETRIC_CLOUD_SurfaceShadowTransmittance"), std::string::npos);
  EXPECT_NE(shader_source.find("shadowed_scene_color *= mix"), std::string::npos);
  EXPECT_NE(shader_source.find("candidate_error = abs(candidate_distance - target_distance)"), std::string::npos);
  EXPECT_NE(shader_source.find("hit_distance >= camera_far * 0.999f"), std::string::npos);
  EXPECT_NE(shader_source.find("noiseExtinctionMarchDistance.w"), std::string::npos);
  EXPECT_NE(shader_source.find("if (debug_mode == 1) {\n    return vec4(cloud_radiance, 1.0f);"), std::string::npos);
  EXPECT_NE(shader_source.find("debug_mode == 4 || debug_mode == 5 || debug_mode == 6"), std::string::npos);
  EXPECT_NE(shader_source.find("cloud_radiance + scene_color * transmittance"), std::string::npos);
  EXPECT_NE(shader_source.find("imageStore(inOutColor"), std::string::npos);
}

TEST(VolumetricCloudShader, RasterPassBindsComputePipelineAndGraphResources) {
  const auto pass_source = ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "src" /
                                        "RenderPasses" / "VolumetricCloudsPass.cpp");
  ASSERT_FALSE(pass_source.empty());

  EXPECT_NE(pass_source.find("CreateGraphImageMipView(accumulation_binding->image, 0)"), std::string::npos);
  EXPECT_NE(pass_source.find("CreateGraphImageMipView(transmittance_binding->image, 0)"), std::string::npos);
  EXPECT_NE(pass_source.find("parameters.input_is_ray_hit_distance"), std::string::npos);
  EXPECT_NE(pass_source.find("settings.max_march_distance"), std::string::npos);
  EXPECT_NE(pass_source.find("noise_extinction_march_distance"), std::string::npos);
  EXPECT_NE(pass_source.find("VK_IMAGE_TYPE_3D"), std::string::npos);
  EXPECT_NE(pass_source.find("BuildBaseShapeNoiseBytes"), std::string::npos);
  EXPECT_NE(pass_source.find("BuildDetailErosionNoiseBytes"), std::string::npos);
  EXPECT_NE(pass_source.find("BuildWeatherCoverageNoiseBytes"), std::string::npos);
  EXPECT_NE(pass_source.find("BuildCurlNoiseBytes"), std::string::npos);
  EXPECT_NE(pass_source.find("CreateCloudWeatherImage"), std::string::npos);
  EXPECT_NE(pass_source.find("CreateCloudWeatherImageView"), std::string::npos);
  EXPECT_NE(pass_source.find("CreateCloudCurlNoiseImage"), std::string::npos);
  EXPECT_NE(pass_source.find("CreateCloudCurlNoiseImageView"), std::string::npos);
  EXPECT_NE(pass_source.find("atmosphere_cloud_type_curl"), std::string::npos);
  EXPECT_NE(pass_source.find("march_control"), std::string::npos);
  EXPECT_NE(pass_source.find("GetCloudNoiseResources"), std::string::npos);
  EXPECT_NE(pass_source.find("CreateGraphImageMipView(depth_binding->image, 0)"), std::string::npos);
  EXPECT_NE(pass_source.find("descriptor_set->UpdateImageDescriptorBinding(0, image_info)"), std::string::npos);
  EXPECT_NE(pass_source.find("descriptor_set->UpdateImageDescriptorBinding(1, image_info)"), std::string::npos);
  EXPECT_NE(pass_source.find("descriptor_set->UpdateImageDescriptorBinding(4, image_info)"), std::string::npos);
  EXPECT_NE(pass_source.find("descriptor_set->UpdateImageDescriptorBinding(5, image_info)"), std::string::npos);
  EXPECT_NE(pass_source.find("descriptor_set->UpdateImageDescriptorBinding(6, image_info)"), std::string::npos);
  EXPECT_NE(pass_source.find("descriptor_set->UpdateImageDescriptorBinding(7, image_info)"), std::string::npos);
  EXPECT_NE(pass_source.find("descriptor_set->UpdateImageDescriptorBinding(8, image_info)"), std::string::npos);
  EXPECT_NE(pass_source.find("descriptor_set->UpdateImageDescriptorBinding(9, image_info)"), std::string::npos);
  EXPECT_NE(pass_source.find("GetCloudHistoryResources"), std::string::npos);
  EXPECT_NE(pass_source.find("history_resources.frame_index % 2u"), std::string::npos);
  EXPECT_NE(pass_source.find("descriptor_set->UpdateImageDescriptorBinding(10, image_info)"), std::string::npos);
  EXPECT_NE(pass_source.find("descriptor_set->UpdateImageDescriptorBinding(11, image_info)"), std::string::npos);
  EXPECT_NE(pass_source.find("descriptor_set->UpdateImageDescriptorBinding(12, image_info)"), std::string::npos);
  EXPECT_NE(pass_source.find("descriptor_set->UpdateImageDescriptorBinding(13, image_info)"), std::string::npos);
  EXPECT_NE(pass_source.find("parameters.pipeline->Dispatch"), std::string::npos);
  EXPECT_NE(pass_source.find("parameters.composite_pipeline->Dispatch"), std::string::npos);

  const auto render_layer_source =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "src" / "RenderLayer.cpp");
  ASSERT_FALSE(render_layer_source.empty());
  EXPECT_NE(render_layer_source.find("Shaders/Compute/VolumetricClouds.comp"), std::string::npos);
  EXPECT_NE(render_layer_source.find("Shaders/Compute/VolumetricCloudsComposite.comp"), std::string::npos);
  EXPECT_NE(render_layer_source.find("volumetric_clouds_layout_->PushDescriptorBinding(6"), std::string::npos);
  EXPECT_NE(render_layer_source.find("volumetric_clouds_layout_->PushDescriptorBinding(7"), std::string::npos);
  EXPECT_NE(render_layer_source.find("volumetric_clouds_layout_->PushDescriptorBinding(8"), std::string::npos);
  EXPECT_NE(render_layer_source.find("volumetric_clouds_layout_->PushDescriptorBinding(9"), std::string::npos);
  EXPECT_NE(render_layer_source.find("volumetric_clouds_layout_->PushDescriptorBinding(10"), std::string::npos);
  EXPECT_NE(render_layer_source.find("volumetric_clouds_layout_->PushDescriptorBinding(11"), std::string::npos);
  EXPECT_NE(render_layer_source.find("volumetric_clouds_layout_->PushDescriptorBinding(12"), std::string::npos);
  EXPECT_NE(render_layer_source.find("volumetric_clouds_layout_->PushDescriptorBinding(13"), std::string::npos);
  EXPECT_NE(render_layer_source.find("volumetric_cloud_settings"), std::string::npos);
  EXPECT_NE(render_layer_source.find("volumetric_clouds_pipeline_"), std::string::npos);
  EXPECT_NE(render_layer_source.find("volumetric_clouds_composite_pipeline_"), std::string::npos);
}

TEST(VolumetricCloudShader, RayTracingCameraWritesHitDistanceForCloudPass) {
  const auto raygen_source =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "Internals" /
                   "DefaultResources" / "Shaders" / "RayTracing" / "RayGen" / "Camera.rgen");
  ASSERT_FALSE(raygen_source.empty());
  EXPECT_NE(raygen_source.find("layout(set = 2, binding = 1, r32f) uniform image2D ray_hit_distance_image"),
            std::string::npos);
  EXPECT_NE(raygen_source.find("primary_hit_distance"), std::string::npos);
  EXPECT_NE(raygen_source.find("EE_CAMERA_FAR"), std::string::npos);
  EXPECT_NE(raygen_source.find("hit_value.initial_position = vec3(0.0f)"), std::string::npos);
  EXPECT_NE(raygen_source.find("imageStore(ray_hit_distance_image"), std::string::npos);

  const auto ray_pass_source = ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "src" /
                                            "RenderPasses" / "RayTracingCameraPass.cpp");
  ASSERT_FALSE(ray_pass_source.empty());
  EXPECT_NE(ray_pass_source.find("RenderResourceNames::camera_ray_hit_distance"), std::string::npos);
  EXPECT_NE(ray_pass_source.find("CreateGraphImageMipView(hit_distance_binding->image, 0)"), std::string::npos);
  EXPECT_NE(ray_pass_source.find("output_descriptor_set->UpdateImageDescriptorBinding(1, image_info)"),
            std::string::npos);

  const auto render_layer_source =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "src" / "RenderLayer.cpp");
  ASSERT_FALSE(render_layer_source.empty());
  EXPECT_NE(render_layer_source.find("ray_tracing_camera_output_layout_"), std::string::npos);
  EXPECT_NE(render_layer_source.find("RenderResourceNames::camera_ray_hit_distance"), std::string::npos);
}

TEST(VolumetricCloudShader, RenderLayerSharesCloudPassBetweenRasterAndRayTracingCameras) {
  const auto render_layer_source =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "src" / "RenderLayer.cpp");
  const auto pass_source = ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "src" /
                                        "RenderPasses" / "VolumetricCloudsPass.cpp");
  ASSERT_FALSE(render_layer_source.empty());
  ASSERT_FALSE(pass_source.empty());

  EXPECT_EQ(CountOccurrences(render_layer_source, "VolumetricCloudsPass::Execute("), 2u);
  EXPECT_NE(render_layer_source.find("VolumetricCloudsPass::CreateRasterDescriptor(RenderPassNames::deferred_camera)"),
            std::string::npos);
  EXPECT_NE(render_layer_source.find("use_ray_query ? RenderPassNames::ray_query_camera : "
                                     "RenderPassNames::ray_tracing_camera"),
            std::string::npos);
  EXPECT_NE(render_layer_source.find("VolumetricCloudsPass::CreateRayTracingDescriptor(ray_camera_pass_name)"),
            std::string::npos);
  EXPECT_NE(render_layer_source.find("active_camera_transient_resources, volumetric_cloud_settings, camera_index"),
            std::string::npos);
  EXPECT_NE(render_layer_source.find("RenderResourceNames::camera_ray_hit_distance, true"), std::string::npos);
  EXPECT_EQ(render_layer_source.find("VolumetricCloudsRasterPass"), std::string::npos);
  EXPECT_EQ(render_layer_source.find("VolumetricCloudsRayTracingPass"), std::string::npos);

  EXPECT_NE(pass_source.find("GetCloudNoiseResources"), std::string::npos);
  EXPECT_NE(pass_source.find("base_shape_view"), std::string::npos);
  EXPECT_NE(pass_source.find("detail_erosion_view"), std::string::npos);
  EXPECT_NE(pass_source.find("weather_coverage_view"), std::string::npos);
  EXPECT_NE(pass_source.find("curl_noise_view"), std::string::npos);
  EXPECT_NE(pass_source.find("parameters.composite_pipeline"), std::string::npos);
}
