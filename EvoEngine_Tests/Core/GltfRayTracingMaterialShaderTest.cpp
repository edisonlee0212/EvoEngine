#include "EvoEngine_SDK_PCH.hpp"

#include "EnvironmentalMap.hpp"
#include "GraphicsResources.hpp"
#include "RayTracingPipeline.hpp"
#include "RenderInstanceStorage.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace {
std::string ReadTextFile(const std::filesystem::path& path) {
  std::ifstream file(path);
  EXPECT_TRUE(file.good()) << path.string();
  return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}

std::filesystem::path ShaderPath(const std::filesystem::path& relative_path) {
  return std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "Internals" / "DefaultResources" /
         "Shaders" / relative_path;
}

std::filesystem::path SdkPath(const std::filesystem::path& relative_path) {
  return std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / relative_path;
}

std::filesystem::path AppPath(const std::filesystem::path& relative_path) {
  return std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_App" / relative_path;
}

std::string ReadRayTracingCameraSource() {
  return ReadTextFile(ShaderPath("RayTracing/RayGen/Camera.rgen")) +
         ReadTextFile(ShaderPath("Includes/CameraRayIntegrator.glsl")) +
         ReadTextFile(ShaderPath("Includes/CameraRayTracingTraversal.glsl"));
}

std::string ReadRayQueryCameraSource() {
  return ReadTextFile(ShaderPath("Compute/RayQueryCamera.comp")) +
         ReadTextFile(ShaderPath("Includes/CameraRayIntegrator.glsl")) +
         ReadTextFile(ShaderPath("Includes/CameraRayQueryTraversal.glsl"));
}

VkAccelerationStructureInstanceKHR MakeTlasTestInstance(const VkDeviceAddress address = 1) {
  VkAccelerationStructureInstanceKHR instance{};
  instance.transform.matrix[0][0] = 1.0f;
  instance.transform.matrix[1][1] = 1.0f;
  instance.transform.matrix[2][2] = 1.0f;
  instance.mask = 0xff;
  instance.accelerationStructureReference = address;
  return instance;
}
}  // namespace

TEST(GltfRayTracingMaterial, CameraClosestHitRecordsCompactIntersectionIdentity) {
  const auto source = ReadTextFile(ShaderPath("RayTracing/ClosestHit/Camera.rchit"));
  ASSERT_FALSE(source.empty());

  EXPECT_EQ(source.find("EE_CAMERA_COMPACT_PAYLOAD"), std::string::npos);
  EXPECT_EQ(source.find("#include \"RayTracingBasic.glsl\""), std::string::npos);
  EXPECT_EQ(source.find("#include \"GltfRasterMaterial.glsl\""), std::string::npos);
  EXPECT_EQ(source.find("const Instance instance = EE_INSTANCES"), std::string::npos);
  EXPECT_EQ(source.find("const Vertex v0"), std::string::npos);
  EXPECT_NE(source.find("hit_value.hit_t = gl_HitTEXT"), std::string::npos);
  EXPECT_NE(source.find("hit_value.instance_index = uint(gl_InstanceCustomIndexEXT)"), std::string::npos);
  EXPECT_NE(source.find("hit_value.primitive_id = uint(gl_PrimitiveID)"), std::string::npos);
  EXPECT_NE(source.find("hit_value.barycentrics = attribs"), std::string::npos);
  EXPECT_EQ(source.find("world_geometric_normal"), std::string::npos);
  EXPECT_EQ(source.find("hit_value.material_index"), std::string::npos);
  EXPECT_NE(source.find("if (hit_value.type == EE_CAMERA_RAY_PAYLOAD_SHADOW)"), std::string::npos);
  EXPECT_NE(source.find("hit_value.shadow_transmission = vec3(0.0f)"), std::string::npos);
  EXPECT_EQ(source.find("traceRayEXT("), std::string::npos);
  EXPECT_EQ(source.find("combined_color"), std::string::npos);
  EXPECT_EQ(source.find("EE_CAMERA_DIRECT_LIGHTING"), std::string::npos);

  EXPECT_EQ(source.find(std::string("Material") + "Properties"), std::string::npos);
  EXPECT_EQ(source.find(std::string("EE_MATERIAL") + "_PROPERTIES"), std::string::npos);
  EXPECT_EQ(source.find("EE_SAMPLE_TEXTURE_2D"), std::string::npos);
}

TEST(GltfRayTracingMaterial, CameraPayloadAndRecursionStayTechniqueScoped) {
  const auto payload = ReadTextFile(ShaderPath("Includes/CameraRayTracingPayload.glsl"));
  const auto ray_query = ReadTextFile(ShaderPath("Includes/CameraRayQueryTraversal.glsl"));
  const auto ray_tracing = ReadTextFile(ShaderPath("Includes/CameraRayTracingTraversal.glsl"));
  const auto render_layer = ReadTextFile(SdkPath("src/RenderLayer.cpp"));
  const auto pipeline_header = ReadTextFile(SdkPath("include/Rendering/Platform/RayTracingPipeline.hpp"));
  const auto pipeline = ReadTextFile(SdkPath("src/RayTracingPipeline.cpp"));

  for (const auto* field : {"vec3 shadow_transmission", "uint seed", "vec2 barycentrics", "float hit_t",
                            "uint instance_index", "uint primitive_id", "uint type"}) {
    EXPECT_NE(payload.find(field), std::string::npos) << field;
  }
  for (const auto* removed_field : {"hit_count", "material_index", "environment_radiance", "environment_pdf",
                                    "initial_position", "initial_normal", "last_sample_pdf"}) {
    EXPECT_EQ(payload.find(removed_field), std::string::npos) << removed_field;
  }
  for (const auto& path :
       {"RayTracing/RayGen/Camera.rgen", "RayTracing/Miss/Camera.rmiss", "RayTracing/ClosestHit/Camera.rchit",
        "RayTracing/AnyHit/Camera.rahit", "Compute/RayQueryCamera.comp"}) {
    const auto source = ReadTextFile(ShaderPath(path));
    EXPECT_FALSE(source.empty()) << path;
    EXPECT_EQ(source.find("EE_CAMERA_COMPACT_PAYLOAD"), std::string::npos) << path;
  }

  const auto query_fill_begin = ray_query.find("void EE_CAMERA_RAY_QUERY_FILL_SURFACE_PAYLOAD");
  const auto query_fill_end = ray_query.find("void EE_CAMERA_RAY_QUERY_CANDIDATE_SURFACE", query_fill_begin);
  ASSERT_NE(query_fill_begin, std::string::npos);
  ASSERT_NE(query_fill_end, std::string::npos);
  EXPECT_EQ(ray_query.substr(query_fill_begin, query_fill_end - query_fill_begin).find("const Vertex v0"),
            std::string::npos);
  EXPECT_EQ(ray_tracing.find("const CameraRayTracingPayload path_payload"), std::string::npos);
  EXPECT_NE(ray_tracing.find("gl_RayFlagsTerminateOnFirstHitEXT | gl_RayFlagsSkipClosestHitShaderEXT"),
            std::string::npos);

  EXPECT_NE(render_layer.find("pipeline->SetMaxRecursionDepth(1)"), std::string::npos);
  EXPECT_NE(pipeline_header.find("uint32_t max_recursion_depth_ = 8"), std::string::npos);
  EXPECT_NE(pipeline.find("raytracing_pipeline_create_info.maxPipelineRayRecursionDepth = max_recursion_depth_"),
            std::string::npos);
  evo_engine::RayTracingPipeline pipeline_config;
  pipeline_config.SetMaxRecursionDepth(1u);
  EXPECT_FALSE(evo_engine::RayTracingPipeline::IsRecursionDepthSupported(0, 8));
  EXPECT_TRUE(evo_engine::RayTracingPipeline::IsRecursionDepthSupported(1, 1));
  EXPECT_TRUE(evo_engine::RayTracingPipeline::IsRecursionDepthSupported(8, 8));
  EXPECT_FALSE(evo_engine::RayTracingPipeline::IsRecursionDepthSupported(9, 8));
}

TEST(GltfRayTracingMaterial, ActiveRayCamerasShareOneIntegratorWithTraversalAdapters) {
  const auto raygen = ReadTextFile(ShaderPath("RayTracing/RayGen/Camera.rgen"));
  const auto ray_query = ReadTextFile(ShaderPath("Compute/RayQueryCamera.comp"));
  const auto integrator = ReadTextFile(ShaderPath("Includes/CameraRayIntegrator.glsl"));
  const auto ray_tracing_traversal = ReadTextFile(ShaderPath("Includes/CameraRayTracingTraversal.glsl"));
  const auto ray_query_traversal = ReadTextFile(ShaderPath("Includes/CameraRayQueryTraversal.glsl"));

  ASSERT_FALSE(raygen.empty());
  ASSERT_FALSE(ray_query.empty());
  ASSERT_FALSE(integrator.empty());
  ASSERT_FALSE(ray_tracing_traversal.empty());
  ASSERT_FALSE(ray_query_traversal.empty());

  EXPECT_NE(raygen.find("#define EE_CAMERA_RAY_TRACING_TRAVERSAL"), std::string::npos);
  EXPECT_NE(ray_query.find("#define EE_CAMERA_RAY_QUERY_TRAVERSAL"), std::string::npos);
  EXPECT_NE(raygen.find("#include \"CameraRayIntegrator.glsl\""), std::string::npos);
  EXPECT_NE(ray_query.find("#include \"CameraRayIntegrator.glsl\""), std::string::npos);
  EXPECT_EQ(raygen.find("EE_CAMERA_TRACE_PATH"), std::string::npos);
  EXPECT_EQ(ray_query.find("EE_CAMERA_TRACE_PATH"), std::string::npos);

  EXPECT_NE(integrator.find("vec3 EE_CAMERA_TRACE_PATH"), std::string::npos);
  EXPECT_NE(integrator.find("EE_CAMERA_PREPARE_DIRECT_LIGHTING"), std::string::npos);
  EXPECT_NE(integrator.find("EE_GLTF_RT_BSDF_SAMPLE"), std::string::npos);
  EXPECT_NE(integrator.find("EE_CAMERA_PROCESS_VOLUME_SEGMENT"), std::string::npos);
  EXPECT_NE(integrator.find("EE_CAMERA_RUSSIAN_ROULETTE_MIN_DEPTH"), std::string::npos);
  EXPECT_NE(integrator.find("void EE_CAMERA_RENDER_PIXEL"), std::string::npos);
  EXPECT_EQ(integrator.find("traceRayEXT("), std::string::npos);
  EXPECT_EQ(integrator.find("rayQueryInitializeEXT"), std::string::npos);
  EXPECT_EQ(integrator.find("gl_LaunchIDEXT"), std::string::npos);
  EXPECT_EQ(integrator.find("gl_GlobalInvocationID"), std::string::npos);

  EXPECT_NE(ray_tracing_traversal.find("traceRayEXT("), std::string::npos);
  EXPECT_EQ(ray_tracing_traversal.find("rayQueryInitializeEXT"), std::string::npos);
  EXPECT_NE(ray_query_traversal.find("rayQueryInitializeEXT"), std::string::npos);
  EXPECT_EQ(ray_query_traversal.find("traceRayEXT("), std::string::npos);
}

TEST(GltfRayTracingMaterial, CameraRayVariantsKeepLayoutAndTechniqueOwnershipIndependent) {
  const auto material = ReadTextFile(ShaderPath("Includes/GltfMaterial.glsl"));
  const auto raster = ReadTextFile(ShaderPath("Includes/GltfRasterMaterial.glsl"));
  const auto bsdf = ReadTextFile(ShaderPath("Includes/GltfRayTracingBsdf.glsl"));
  const auto integrator = ReadTextFile(ShaderPath("Includes/CameraRayIntegrator.glsl"));
  const auto render_layer = ReadTextFile(SdkPath("src/RenderLayer.cpp"));
  const auto variant_cache = ReadTextFile(SdkPath("src/RayCameraShaderVariantCache.cpp"));
  const auto editor = ReadTextFile(AppPath("src/EvoEngineEditor.cpp"));

  EXPECT_NE(material.find("#define EE_GLTF_USE_TRANSMISSION MAT_EXT_TRANSMISSION"), std::string::npos);
  EXPECT_NE(material.find("#define EE_GLTF_USE_TEXTURE_TRANSFORM MAT_EXT_TEXTURE_TRANSFORM"), std::string::npos);
  EXPECT_EQ(raster.find("#if MAT_EXT_"), std::string::npos);
  EXPECT_EQ(bsdf.find("#if MAT_EXT_"), std::string::npos);
  EXPECT_EQ(integrator.find("#if MAT_EXT_UNLIT"), std::string::npos);
  EXPECT_NE(integrator.find("#if EE_GLTF_USE_UNLIT"), std::string::npos);
  EXPECT_NE(render_layer.find("ShaderType::RayGen, shader_header"), std::string::npos);
  EXPECT_NE(render_layer.find("ShaderType::AnyHit, shader_header"), std::string::npos);
  EXPECT_NE(render_layer.find("ShaderType::Compute, shader_header"), std::string::npos);
  EXPECT_NE(variant_cache.find("void RayCameraShaderVariantCache::RequestRayQuery"), std::string::npos);
  EXPECT_NE(variant_cache.find("void RayCameraShaderVariantCache::RequestRayTracing"), std::string::npos);
  EXPECT_NE(variant_cache.find("Jobs::RunOnRenderThread"), std::string::npos);
  EXPECT_NE(editor.find("IsRayCameraShaderVariantReady"), std::string::npos);
}

TEST(GltfRayTracingMaterial, CameraAnyHitAppliesGltfAlphaCutoff) {
  const auto any_hit = ReadTextFile(ShaderPath("RayTracing/AnyHit/Camera.rahit"));
  const auto evaluator = ReadTextFile(ShaderPath("Includes/GltfRasterMaterial.glsl"));
  const auto render_layer = ReadTextFile(SdkPath("src/RenderLayer.cpp"));
  const auto pipeline = ReadTextFile(SdkPath("src/RayTracingPipeline.cpp"));
  const auto raygen = ReadRayTracingCameraSource();

  ASSERT_FALSE(any_hit.empty());
  ASSERT_FALSE(evaluator.empty());
  ASSERT_FALSE(render_layer.empty());
  ASSERT_FALSE(pipeline.empty());
  ASSERT_FALSE(raygen.empty());

  EXPECT_NE(any_hit.find("#include \"GltfRasterMaterial.glsl\""), std::string::npos);
  EXPECT_NE(any_hit.find("#include \"Random.glsl\""), std::string::npos);
  EXPECT_NE(any_hit.find("material_index, tex_coord_0, tex_coord_1, tex_coord_2, tex_coord_3, vertex_color.a"),
            std::string::npos);
  EXPECT_NE(evaluator.find("float EE_GLTF_RASTER_OPACITY_LOD0"), std::string::npos);
  EXPECT_NE(evaluator.find("material.alpha_mode == EE_GLTF_ALPHA_MODE_MASK"), std::string::npos);
  EXPECT_NE(evaluator.find("base_color_alpha *= vertex_alpha"), std::string::npos);
  EXPECT_NE(evaluator.find("material.pbr_base_color_texture, EE_GLTF_RASTER_TEXTURE_BASE_COLOR"), std::string::npos);
  EXPECT_NE(any_hit.find("vertex_color.a"), std::string::npos);
  EXPECT_NE(any_hit.find("EE_PCG_RANDOM(hit_value.seed) > opacity"), std::string::npos);
  EXPECT_NE(any_hit.find("ignoreIntersectionEXT"), std::string::npos);
  EXPECT_NE(render_layer.find("ShaderType::AnyHit"), std::string::npos);
  EXPECT_NE(render_layer.find("Shaders/RayTracing/AnyHit/Camera.rahit"), std::string::npos);
  EXPECT_NE(pipeline.find("closest_hit_group_ci.closestHitShader = closest_hit_shader_index"), std::string::npos);
  EXPECT_NE(pipeline.find("closest_hit_group_ci.anyHitShader = any_hit_shader_index"), std::string::npos);
  EXPECT_NE(pipeline.find("buffer_create_info.size = handle_size_aligned_"), std::string::npos);
  EXPECT_EQ(pipeline.find("buffer_create_info.size = handle_size;"), std::string::npos);
  EXPECT_NE(any_hit.find("hit_value.type != EE_CAMERA_RAY_PAYLOAD_SHADOW"), std::string::npos);
  EXPECT_LT(any_hit.find("hit_value.type != EE_CAMERA_RAY_PAYLOAD_SHADOW"),
            any_hit.find("const vec3 object_geometric_normal"));
  EXPECT_EQ(raygen.find("gl_RayFlagsNoOpaqueEXT"), std::string::npos);
  EXPECT_EQ(raygen.find("gl_RayFlagsOpaqueEXT | gl_RayFlagsTerminateOnFirstHitEXT"), std::string::npos);
  EXPECT_EQ(raygen.find("traceRayEXT(EE_TLAS, gl_RayFlagsOpaqueEXT, 0xff"), std::string::npos);
}

TEST(GltfRayTracingMaterial, CameraRaygenUsesRgbTransparentShadowTransmission) {
  const auto payload = ReadTextFile(ShaderPath("Includes/CameraRayTracingPayload.glsl"));
  const auto evaluator = ReadTextFile(ShaderPath("Includes/GltfRasterMaterial.glsl"));
  const auto raygen = ReadRayTracingCameraSource();
  const auto any_hit = ReadTextFile(ShaderPath("RayTracing/AnyHit/Camera.rahit"));
  const auto miss = ReadTextFile(ShaderPath("RayTracing/Miss/Camera.rmiss"));
  ASSERT_FALSE(payload.empty());
  ASSERT_FALSE(evaluator.empty());
  ASSERT_FALSE(raygen.empty());
  ASSERT_FALSE(any_hit.empty());
  ASSERT_FALSE(miss.empty());

  EXPECT_NE(payload.find("vec3 shadow_transmission"), std::string::npos);
  EXPECT_NE(payload.find("float shadow_previous_hit_t"), std::string::npos);
  EXPECT_NE(payload.find("uint shadow_is_inside"), std::string::npos);

  EXPECT_NE(raygen.find("const float EE_CAMERA_MIN_SHADOW_TRANSMISSION = 0.01f"), std::string::npos);
  EXPECT_NE(raygen.find("vec3 EE_CAMERA_SHADOW_TRANSMISSION"), std::string::npos);
  EXPECT_NE(raygen.find("hit_value.shadow_transmission = vec3(1.0f)"), std::string::npos);
  EXPECT_NE(raygen.find("hit_value.shadow_is_inside = initial_inside ? 1u : 0u"), std::string::npos);
  EXPECT_NE(raygen.find("gl_RayFlagsTerminateOnFirstHitEXT | gl_RayFlagsSkipClosestHitShaderEXT"), std::string::npos);
  EXPECT_EQ(raygen.find("gl_RayFlagsNoOpaqueEXT | gl_RayFlagsSkipClosestHitShaderEXT"), std::string::npos);
  EXPECT_NE(raygen.find("direct_light.radiance_over_pdf *= shadow_transmission"), std::string::npos);
  EXPECT_EQ(raygen.find("bool EE_CAMERA_SHADOW_VISIBLE"), std::string::npos);
  EXPECT_NE(raygen.find("hit_value.type == EE_CAMERA_RAY_PAYLOAD_MISS"), std::string::npos);
  EXPECT_EQ(raygen.find("const CameraRayTracingPayload path_payload"), std::string::npos);

  EXPECT_NE(any_hit.find("EE_CAMERA_SHADOW_TRANSMISSION"), std::string::npos);
  EXPECT_NE(any_hit.find("EE_GLTF_RASTER_SHADOW_TRANSMISSION_LOD0("), std::string::npos);
  EXPECT_NE(evaluator.find("effective_diffuse_transmission"), std::string::npos);
  EXPECT_NE(evaluator.find("EE_GLTF_RASTER_FRESNEL(specular_f0, vec3(specular_weight)"), std::string::npos);
  EXPECT_NE(evaluator.find("specular_transmission * base_color"), std::string::npos);
  EXPECT_NE(evaluator.find("const float remaining_energy"), std::string::npos);
  EXPECT_NE(evaluator.find("material.attenuation_color"), std::string::npos);
  EXPECT_NE(evaluator.find("scatter_coefficient"), std::string::npos);
  EXPECT_NE(any_hit.find("abs(gl_HitTEXT - hit_value.shadow_previous_hit_t)"), std::string::npos);
  EXPECT_NE(any_hit.find("hit_value.shadow_is_inside = is_inside ? 1u : 0u"), std::string::npos);
  const auto ray_query = ReadRayQueryCameraSource();
  ASSERT_FALSE(ray_query.empty());
  EXPECT_NE(ray_query.find("const float hit_t = rayQueryGetIntersectionTEXT(ray_query, false)"), std::string::npos);
  EXPECT_NE(ray_query.find("const float segment_length = max(0.0f, hit_t - previous_hit_t)"), std::string::npos);
  EXPECT_NE(ray_query.find("previous_hit_t = hit_t"), std::string::npos);
  EXPECT_NE(evaluator.find("mix(0.65, 1.0, roughness_effect)"), std::string::npos);
  EXPECT_NE(any_hit.find("hit_value.shadow_transmission *= transmission"), std::string::npos);
  EXPECT_NE(any_hit.find("terminateRayEXT"), std::string::npos);

  EXPECT_EQ(miss.find("EE_CAMERA_RAY_PAYLOAD_SHADOW"), std::string::npos);
  EXPECT_NE(miss.find("hit_value.type = EE_CAMERA_RAY_PAYLOAD_MISS"), std::string::npos);
  EXPECT_EQ(miss.find("shadow_transmission = vec3(0.0f)"), std::string::npos);
}

TEST(GltfRayTracingMaterial, RayShadersUseCanonicalMaterialBlockOnly) {
  const std::filesystem::path paths[] = {
      ShaderPath("RayTracing/RayGen/Camera.rgen"),
      ShaderPath("RayTracing/Miss/Camera.rmiss"),
      ShaderPath("RayTracing/ClosestHit/Camera.rchit"),
      ShaderPath("RayTracing/AnyHit/Camera.rahit"),
      ShaderPath("RayTracing/ClosestHit/PointCloud.rchit"),
      ShaderPath("RayTracing/ClosestHit/DDGIProbeDiagnostics.rchit"),
  };

  for (const auto& path : paths) {
    const auto source = ReadTextFile(path);
    ASSERT_FALSE(source.empty()) << path.string();
    EXPECT_EQ(source.find("#define EE_SKIP_LEGACY_MATERIALS"), std::string::npos) << path.string();
    EXPECT_EQ(source.find(std::string("EE_MATERIAL") + "_PROPERTIES"), std::string::npos) << path.string();
    EXPECT_EQ(source.find(std::string("Material") + "Properties"), std::string::npos) << path.string();
  }

  const auto per_frame = ReadTextFile(ShaderPath("Includes/PerFrame.glsl"));
  ASSERT_FALSE(per_frame.empty());
  EXPECT_EQ(per_frame.find("#ifndef EE_SKIP_LEGACY_MATERIALS"), std::string::npos);
  EXPECT_EQ(per_frame.find(std::string("#include \"Materials") + ".glsl\""), std::string::npos);
  EXPECT_NE(per_frame.find("#include \"GltfMaterial.glsl\""), std::string::npos);

  const auto evaluator = ReadTextFile(ShaderPath("Includes/GltfRasterMaterial.glsl"));
  ASSERT_FALSE(evaluator.empty());
  EXPECT_NE(evaluator.find("EE_GLTF_USE_EXPLICIT_TEXTURE_LOD"), std::string::npos);
  EXPECT_NE(evaluator.find("textureLod(EE_TEXTURE_2DS[nonuniformEXT(texture_info.index)], uv, EE_GLTF_TEXTURE_LOD)"),
            std::string::npos);
  EXPECT_NE(evaluator.find("vec4 gradients"), std::string::npos);
  EXPECT_NE(evaluator.find("textureGrad(EE_TEXTURE_2DS[nonuniformEXT(texture_info.index)], uv, ddx_uv, ddy_uv)"),
            std::string::npos);
}

TEST(GltfRayTracingMaterial, RayTracedTextureLodUsesRayFootprintGradients) {
  const auto evaluator = ReadTextFile(ShaderPath("Includes/GltfRasterMaterial.glsl"));
  const auto bsdf = ReadTextFile(ShaderPath("Includes/GltfRayTracingBsdf.glsl"));
  const auto raygen = ReadRayTracingCameraSource();
  const auto ray_query = ReadRayQueryCameraSource();
  const auto any_hit = ReadTextFile(ShaderPath("RayTracing/AnyHit/Camera.rahit"));
  ASSERT_FALSE(evaluator.empty());
  ASSERT_FALSE(bsdf.empty());
  ASSERT_FALSE(raygen.empty());
  ASSERT_FALSE(ray_query.empty());
  ASSERT_FALSE(any_hit.empty());

  EXPECT_NE(evaluator.find("vec4 EE_GLTF_SAMPLE_TEXTURE("), std::string::npos);
  EXPECT_NE(evaluator.find("GltfTexCoords tex_coords, vec4 fallback"), std::string::npos);
  EXPECT_NE(evaluator.find("tex_coords.gradients[clamp(texture_info.tex_coord, 0, 3)]"), std::string::npos);
  EXPECT_NE(evaluator.find("texture_info.uv_transform * vec3(tex_grad, 0.0, 0.0)"), std::string::npos);
  EXPECT_NE(evaluator.find("textureGrad(EE_TEXTURE_2DS[nonuniformEXT(texture_info.index)], uv, ddx_uv, ddy_uv)"),
            std::string::npos);
  EXPECT_NE(evaluator.find("vec2 tex_coord_2, vec2 tex_coord_3"), std::string::npos);

  EXPECT_NE(bsdf.find("uint material_index, GltfTexCoords tex_coords, vec4 vertex_color"), std::string::npos);
  EXPECT_NE(bsdf.find("EE_EVALUATE_GLTF_RASTER_SURFACE(material_index, tex_coords, vertex_color)"), std::string::npos);
  EXPECT_NE(bsdf.find("material.normal_texture, tex_coords"), std::string::npos);

  for (const auto* source : {&raygen, &ray_query}) {
    EXPECT_NE(source->find("#define EE_GLTF_USE_EXPLICIT_TEXTURE_LOD"), std::string::npos);
    EXPECT_NE(source->find("#define EE_GLTF_TEXTURE_LOD 0.0"), std::string::npos);
    EXPECT_NE(source->find("vec4 EE_CAMERA_TEXEL_DENSITY"), std::string::npos);
    EXPECT_NE(source->find("return sqrt(uv_area / max(world_area, 1e-20f))"), std::string::npos);
    EXPECT_NE(source->find("float EE_CAMERA_WORLD_FOOTPRINT"), std::string::npos);
    EXPECT_NE(source->find("float EE_CAMERA_RAY_SPREAD_ANGLE"), std::string::npos);
    EXPECT_NE(source->find("camera.inverse_projection[1][1]"), std::string::npos);
    EXPECT_NE(source->find("EE_CAMERA_RAY_SPREAD_ANGLE(float(image_size.y))"), std::string::npos);
    EXPECT_NE(source->find("EE_CAMERA_TEXTURE_GRADIENTS"), std::string::npos);
    EXPECT_NE(source->find("EE_EVALUATE_GLTF_RASTER_SURFACE"), std::string::npos);
    EXPECT_NE(source->find("ray_cone_width + hit_t * ray_spread_angle"), std::string::npos);
    EXPECT_NE(source->find("float ray_cone_width = 0.0f"), std::string::npos);
    EXPECT_NE(
        source->find("EE_CAMERA_RECONSTRUCT_SURFACE_HIT(is_inside, ray_direction, ray_cone_width, ray_spread_angle)"),
        std::string::npos);
    EXPECT_NE(source->find("ray_cone_width = EE_CAMERA_WORLD_FOOTPRINT(ray_cone_width, hit_value.hit_t"),
              std::string::npos);
    EXPECT_NE(source->find("inout float ray_cone_width, const float ray_spread_angle"), std::string::npos);
    EXPECT_NE(source->find("ray_cone_width += scatter_distance * ray_spread_angle"), std::string::npos);
  }

  EXPECT_NE(raygen.find("hit.tex_gradients = EE_CAMERA_TEXTURE_GRADIENTS(ray_cone_width"), std::string::npos);
  EXPECT_NE(raygen.find("hit.material_index, tex_coord_0, tex_coord_1, tex_coord_2, tex_coord_3, vertex_color"),
            std::string::npos);
  EXPECT_NE(raygen.find("is_inside, hit.tex_gradients"), std::string::npos);
  EXPECT_NE(raygen.find("v0.tex_coord_1 * barycentrics.x"), std::string::npos);
  EXPECT_NE(raygen.find("v0.tex_coord_2 * barycentrics.x"), std::string::npos);
  EXPECT_NE(raygen.find("v0.tex_coord_3 * barycentrics.x"), std::string::npos);
  EXPECT_NE(raygen.find("v0.color * barycentrics.x"), std::string::npos);
  EXPECT_NE(ray_query.find("void EE_CAMERA_RAY_QUERY_CANDIDATE_SURFACE"), std::string::npos);
  EXPECT_NE(ray_query.find("out uint material_index"), std::string::npos);
  EXPECT_NE(ray_query.find("material_index, tex_coord_0, tex_coord_1, tex_coord_2, tex_coord_3, vertex_color.a"),
            std::string::npos);
  EXPECT_NE(ray_query.find("EE_GLTF_RASTER_SHADOW_TRANSMISSION_LOD0("), std::string::npos);
  EXPECT_NE(ray_query.find("hit.tex_gradients = EE_CAMERA_TEXTURE_GRADIENTS(ray_cone_width"), std::string::npos);
  EXPECT_NE(ray_query.find("hit.material_index, tex_coord_0, tex_coord_1, tex_coord_2, tex_coord_3, vertex_color"),
            std::string::npos);
  EXPECT_NE(any_hit.find("material_index, tex_coord_0, tex_coord_1, tex_coord_2, tex_coord_3, vertex_color.a"),
            std::string::npos);
  EXPECT_NE(any_hit.find("v0.tex_coord_1 * barycentrics.x"), std::string::npos);
  EXPECT_NE(any_hit.find("EE_GLTF_RASTER_SHADOW_TRANSMISSION_LOD0("), std::string::npos);
  EXPECT_EQ(any_hit.find("EE_CAMERA_TEXTURE_GRAD"), std::string::npos);
  EXPECT_NE(evaluator.find("vec4 EE_GLTF_SAMPLE_TEXTURE_LOD0"), std::string::npos);
  EXPECT_NE(evaluator.find("float EE_GLTF_RASTER_OPACITY_LOD0"), std::string::npos);
  EXPECT_NE(evaluator.find("vec3 EE_GLTF_RASTER_SHADOW_TRANSMISSION_LOD0"), std::string::npos);
  EXPECT_NE(evaluator.find("specular_transmission * base_color"), std::string::npos);
  EXPECT_EQ(evaluator.find("surface.transmission * max(surface.base_color.rgb * vertex_color"), std::string::npos);
  EXPECT_NE(bsdf.find("vec3 specular_f0"), std::string::npos);
  EXPECT_NE(bsdf.find("? dielectric_fresnel / dielectric_fresnel_weight"), std::string::npos);
}

TEST(GltfRayTracingMaterial, CameraMissReportsMissToSharedIntegrator) {
  const auto source = ReadTextFile(ShaderPath("RayTracing/Miss/Camera.rmiss"));
  ASSERT_FALSE(source.empty());

  EXPECT_EQ(source.find("EE_CAMERA_RAY_PAYLOAD_SHADOW"), std::string::npos);
  EXPECT_NE(source.find("hit_value.type = EE_CAMERA_RAY_PAYLOAD_MISS"), std::string::npos);
  EXPECT_EQ(source.find("hit_value.hit_count"), std::string::npos);
  EXPECT_EQ(source.find("EE_CAMERA_ENVIRONMENT_RADIANCE"), std::string::npos);
}

TEST(GltfRayTracingMaterial, CameraRaygenOwnsPathTracingLoop) {
  const auto source = ReadRayTracingCameraSource();
  const auto ray_query = ReadRayQueryCameraSource();
  ASSERT_FALSE(source.empty());
  ASSERT_FALSE(ray_query.empty());

  EXPECT_NE(source.find("vec3 EE_CAMERA_TRACE_PATH"), std::string::npos);
  EXPECT_NE(source.find("uint surface_depth = 0u"), std::string::npos);
  EXPECT_NE(source.find("while (surface_depth < max_depth)"), std::string::npos);
  EXPECT_EQ(source.find("for (uint depth = 0u; depth < max_depth; ++depth)"), std::string::npos);
  EXPECT_NE(source.find("EE_CAMERA_RECONSTRUCT_SURFACE_HIT"), std::string::npos);
  EXPECT_NE(source.find("EE_EVALUATE_GLTF_RASTER_SURFACE"), std::string::npos);
  EXPECT_NE(source.find("* EE_TRANSFORM_HANDEDNESS(instance.model)"), std::string::npos);
  EXPECT_NE(source.find("vec3 bitangent"), std::string::npos);
  EXPECT_NE(source.find("vec3 shading_normal"), std::string::npos);
  EXPECT_NE(source.find("hit.shading_normal = hit.normal"), std::string::npos);
  EXPECT_NE(source.find("hit.bitangent = EE_CAMERA_SAFE_NORMALIZE(cross(hit.normal, hit.tangent) * "
                        "tangent_handedness"),
            std::string::npos);
  EXPECT_NE(source.find("const vec3 reflected_direction = reflect(EE_CAMERA_SAFE_NORMALIZE(ray_direction, "
                        "-hit.geometric_normal), hit.normal)"),
            std::string::npos);
  EXPECT_NE(source.find("dot(reflected_direction, hit.geometric_normal) < 0.0f"), std::string::npos);
  EXPECT_NE(source.find("hit.normal = hit.geometric_normal"), std::string::npos);
  EXPECT_NE(source.find("#include \"GltfRayTracingBsdf.glsl\""), std::string::npos);
  EXPECT_NE(source.find("GltfRayTracingPbrMaterial pbr"), std::string::npos);
  EXPECT_NE(source.find("EE_EVALUATE_GLTF_RAY_TRACING_PBR_MATERIAL"), std::string::npos);
  EXPECT_NE(source.find("tex_coord_2, tex_coord_3, vertex_color, hit.surface, hit.normal"), std::string::npos);
  EXPECT_NE(source.find("hit.normal = hit.pbr.normal"), std::string::npos);
  EXPECT_NE(source.find("dot(direct_light.direction, hit.shading_normal) <= 0.0f && "
                        "hit.pbr.diffuse_transmission_factor <= 0.0f"),
            std::string::npos);
  EXPECT_NE(source.find("struct EE_CAMERA_BOUNCE_SCRATCH"), std::string::npos);
  EXPECT_NE(source.find("EE_CAMERA_PREPARE_DIRECT_LIGHTING"), std::string::npos);
  EXPECT_NE(source.find("EE_CAMERA_RESOLVE_DIRECT_LIGHTING"), std::string::npos);
  EXPECT_EQ(source.find("EE_CAMERA_DIRECT_LIGHT_EXPOSURE"), std::string::npos);
  EXPECT_EQ(source.find("* light_radiance * n_dot_l * occlusion"), std::string::npos);
  EXPECT_NE(source.find("struct EE_CAMERA_DIRECT_LIGHT"), std::string::npos);
  EXPECT_NE(source.find("vec3 radiance_over_pdf"), std::string::npos);
  EXPECT_NE(source.find("EE_CAMERA_SAMPLE_DIRECT_LIGHT"), std::string::npos);
  EXPECT_NE(source.find("EE_CAMERA_GET_DIRECT_LIGHTING_TECHNIQUE_PROBABILITIES"), std::string::npos);
  EXPECT_NE(source.find("EE_CAMERA_SINGLE_LIGHT_CONTRIBUTION"), std::string::npos);
  EXPECT_NE(source.find("int(EE_CAMERA_INDEX) * MAX_DIRECTIONAL_LIGHT_SIZE + light_index"), std::string::npos);
  EXPECT_NE(source.find("EE_DIRECTIONAL_LIGHTS[directional_light_index]"), std::string::npos);
  EXPECT_NE(source.find("selection_pdf"), std::string::npos);
  EXPECT_NE(source.find("selection_pdf * light_weight"), std::string::npos);
  EXPECT_NE(source.find("light_weight * direct_light.pdf"), std::string::npos);
  EXPECT_NE(source.find("environment_weight * environment_pdf"), std::string::npos);
  EXPECT_NE(source.find("direct_light.direction = -contrib.incident_vector"), std::string::npos);
  EXPECT_EQ(source.find("EE_CAMERA_SAFE_NORMALIZE(-contrib.incident_vector"), std::string::npos);
  EXPECT_NE(ray_query.find("direct_light.direction = -contrib.incident_vector"), std::string::npos);
  EXPECT_EQ(ray_query.find("EE_CAMERA_SAFE_NORMALIZE(-contrib.incident_vector"), std::string::npos);
  EXPECT_NE(source.find("if (n.z < -0.99998796f)"), std::string::npos);
  EXPECT_NE(source.find("const float a = 1.0f / (1.0f + n.z)"), std::string::npos);
  EXPECT_NE(source.find("const float b = -n.x * n.y * a"), std::string::npos);
  EXPECT_EQ(source.find("abs(normal.x) > 0.99f"), std::string::npos);
  EXPECT_NE(ray_query.find("if (n.z < -0.99998796f)"), std::string::npos);
  EXPECT_NE(ray_query.find("const float a = 1.0f / (1.0f + n.z)"), std::string::npos);
  EXPECT_EQ(ray_query.find("abs(normal.x) > 0.99f"), std::string::npos);
  EXPECT_NE(source.find("direct_light.pdf"), std::string::npos);
  EXPECT_NE(source.find("direct_light.radiance_over_pdf"), std::string::npos);
  EXPECT_NE(source.find("direct_light.pdf = pdf_sum"), std::string::npos);
  EXPECT_EQ(source.find("EE_CAMERA_DISTANCE_LIGHT_ATTENUATION"), std::string::npos);
  EXPECT_EQ(source.find("EE_CAMERA_DIRECT_DIRECTIONAL_LIGHT"), std::string::npos);
  EXPECT_EQ(source.find("EE_CAMERA_DIRECT_POINT_LIGHT"), std::string::npos);
  EXPECT_EQ(source.find("EE_CAMERA_DIRECT_SPOT_LIGHT"), std::string::npos);
  EXPECT_EQ(source.find("EE_CAMERA_DIRECT_ENVIRONMENT_LIGHT"), std::string::npos);
  EXPECT_EQ(source.find("for (int i = 0; i < EE_RENDER_INFO.directional_light_size"), std::string::npos);
  EXPECT_EQ(source.find("for (int i = 0; i < EE_RENDER_INFO.point_light_size"), std::string::npos);
  EXPECT_EQ(source.find("for (int i = 0; i < EE_RENDER_INFO.spot_light_size"), std::string::npos);
  EXPECT_NE(source.find("EE_CAMERA_EVALUATE_DIRECT_BSDF"), std::string::npos);
  EXPECT_NE(source.find("EE_CAMERA_BALANCE_HEURISTIC(direct_light.pdf, bsdf_pdf)"), std::string::npos);
  EXPECT_NE(source.find("EE_GLTF_RT_BSDF_EVALUATE(eval_data, hit.pbr)"), std::string::npos);
  EXPECT_EQ(source.find("EE_CAMERA_PRIMARY_ENVIRONMENT_LIGHTING"), std::string::npos);
  EXPECT_EQ(ray_query.find("EE_CAMERA_PRIMARY_ENVIRONMENT_LIGHTING"), std::string::npos);
  EXPECT_EQ(source.find("#include \"DDGI.glsl\""), std::string::npos);
  EXPECT_EQ(source.find("EE_CAMERA_PRIMARY_DDGI_DIFFUSE"), std::string::npos);
  EXPECT_EQ(source.find("EE_RENDER_INFO.brdf_lut_map_index"), std::string::npos);
  EXPECT_EQ(ray_query.find("EE_RENDER_INFO.brdf_lut_map_index"), std::string::npos);
  EXPECT_NE(source.find("EE_CAMERA_PATH_ENVIRONMENT_RADIANCE"), std::string::npos);
  EXPECT_NE(source.find("EE_CAMERA_ENVIRONMENT_CUBEMAP_INDEX"), std::string::npos);
  EXPECT_NE(source.find("camera.skybox_tex_index"), std::string::npos);
  EXPECT_EQ(source.find("radiance += throughput * EE_CAMERA_PRIMARY_ENVIRONMENT_LIGHTING"), std::string::npos);
  EXPECT_EQ(source.find("radiance += throughput * EE_CAMERA_DIRECT_LIGHTING"), std::string::npos);
  EXPECT_NE(
      source.find("EE_CAMERA_PREPARE_DIRECT_LIGHTING(surface_hit, view_direction, throughput, surface_light_seed"),
      std::string::npos);
  EXPECT_NE(source.find("GltfRayTracingBsdfSampleData sample_data"), std::string::npos);
  EXPECT_NE(source.find("sample_data.k1 = view_direction"), std::string::npos);
  EXPECT_NE(source.find("sample_data.xi = EE_PCG_RANDOM_3(surface_bsdf_seed)"), std::string::npos);
  EXPECT_NE(source.find("EE_GLTF_RT_BSDF_SAMPLE(sample_data, surface_hit.pbr)"), std::string::npos);
  EXPECT_NE(
      source.find(
          "const vec3 direct_contribution = EE_CAMERA_RESOLVE_DIRECT_LIGHTING(bounce, surface_direct_shadow_seed)"),
      std::string::npos);
  EXPECT_NE(source.find("throughput *= sample_data.bsdf_over_pdf"), std::string::npos);
  EXPECT_NE(source.find("last_sample_pdf = sample_data.pdf"), std::string::npos);
  EXPECT_NE(source.find("vec2 max_roughness = vec2(0.0f)"), std::string::npos);
  EXPECT_NE(source.find("max_roughness = max(max_roughness, surface_hit.pbr.roughness)"), std::string::npos);
  EXPECT_NE(source.find("surface_hit.pbr.roughness = max_roughness"), std::string::npos);
  EXPECT_EQ(source.find("const float EE_CAMERA_FIREFLY_CLAMP_THRESHOLD = 10.0f"), std::string::npos);
  EXPECT_NE(source.find("vec3 EE_CAMERA_APPLY_FIREFLY_CLAMP"), std::string::npos);
  EXPECT_NE(source.find("const float luminance = dot(radiance, vec3(1.0f / 3.0f))"), std::string::npos);
  EXPECT_NE(source.find("sample_radiance = EE_CAMERA_APPLY_FIREFLY_CLAMP(camera"), std::string::npos);
  EXPECT_EQ(source.find("EE_CAMERA_SAMPLE_BSDF"), std::string::npos);
  EXPECT_EQ(source.find("EE_CAMERA_BSDF_PDF(hit.surface.metallic, hit.surface.roughness"), std::string::npos);
  EXPECT_EQ(source.find("EE_CAMERA_BSDF_THROUGHPUT"), std::string::npos);
  EXPECT_EQ(source.find("diffuse_brdf * occlusion * n_dot_l"), std::string::npos);
  EXPECT_EQ(source.find("reflection_factor"), std::string::npos);
  for (const auto* shader_source : {&source, &ray_query}) {
    EXPECT_NE(shader_source->find("environment_weight = EE_CAMERA_GET_DIRECT_LIGHTING_TECHNIQUE_PROBABILITIES()."
                                  "environment_weight"),
              std::string::npos);
    EXPECT_NE(shader_source->find("EE_CAMERA_ENVIRONMENT_HIT_MIS_WEIGHT(last_sample_pdf, environment_pdf, "
                                  "environment_weight)"),
              std::string::npos);
    EXPECT_NE(shader_source->find("max(environment_pdf, 0.0f) * max(environment_weight, 0.0f)"), std::string::npos);
  }
  EXPECT_NE(source.find("EE_CAMERA_RUSSIAN_ROULETTE_MIN_DEPTH"), std::string::npos);
  EXPECT_NE(source.find("if (surface_depth >= EE_CAMERA_RUSSIAN_ROULETTE_MIN_DEPTH)"), std::string::npos);
  EXPECT_NE(source.find("surface_depth += 1u"), std::string::npos);
  EXPECT_LT(
      source.find("EE_CAMERA_PREPARE_DIRECT_LIGHTING(surface_hit, view_direction, throughput, surface_light_seed"),
      source.find("EE_GLTF_RT_BSDF_SAMPLE(sample_data, surface_hit.pbr)"));
  EXPECT_LT(
      source.find("EE_GLTF_RT_BSDF_SAMPLE(sample_data, surface_hit.pbr)"),
      source.find(
          "const vec3 direct_contribution = EE_CAMERA_RESOLVE_DIRECT_LIGHTING(bounce, surface_direct_shadow_seed)"));
  EXPECT_LT(
      source.find(
          "const vec3 direct_contribution = EE_CAMERA_RESOLVE_DIRECT_LIGHTING(bounce, surface_direct_shadow_seed)"),
      source.find("if (surface_depth >= EE_CAMERA_RUSSIAN_ROULETTE_MIN_DEPTH)"));
  EXPECT_LT(source.find("if (surface_depth >= EE_CAMERA_RUSSIAN_ROULETTE_MIN_DEPTH)"),
            source.find("surface_depth += 1u"));
  EXPECT_NE(source.find("primary_hit_distance = min(primary_hit_distance, sample_hit_distance)"), std::string::npos);
  EXPECT_NE(source.find("hit.material_index = uint(instance.material_index)"), std::string::npos);
  EXPECT_NE(source.find("hit_value.instance_index"), std::string::npos);
  EXPECT_NE(source.find("hit_value.primitive_id"), std::string::npos);
  EXPECT_EQ(source.find("EE_CAMERA_HIT_DATA_DEBUG_COLOR"), std::string::npos);
  EXPECT_EQ(source.find("EE_CAMERA_MATERIAL_ID_COLOR"), std::string::npos);

  const auto bsdf = ReadTextFile(ShaderPath("Includes/GltfRayTracingBsdf.glsl"));
  ASSERT_FALSE(bsdf.empty());
  EXPECT_NE(bsdf.find("struct GltfRayTracingPbrMaterial"), std::string::npos);
  EXPECT_NE(bsdf.find("struct GltfRayTracingBsdfEvaluateData"), std::string::npos);
  EXPECT_NE(bsdf.find("struct GltfRayTracingBsdfSampleData"), std::string::npos);
  EXPECT_NE(bsdf.find("vec3 bsdf_over_pdf"), std::string::npos);
  EXPECT_NE(bsdf.find("vec3 normal, vec3 tangent, vec3 bitangent"), std::string::npos);
  EXPECT_NE(bsdf.find("EE_GLTF_HAS_TEXTURE(material.normal_texture)"), std::string::npos);
  EXPECT_NE(bsdf.find("normal_vector.xy *= material.normal_texture_scale"), std::string::npos);
  EXPECT_NE(bsdf.find("mat3(pbr.tangent, pbr.bitangent, pbr.normal) * normal_vector"), std::string::npos);
  EXPECT_NE(bsdf.find("const float basis_handedness"), std::string::npos);
  EXPECT_NE(bsdf.find("cross(pbr.bitangent, pbr.normal) * basis_handedness"), std::string::npos);
  EXPECT_NE(bsdf.find("EE_GLTF_RT_BSDF_EVALUATE"), std::string::npos);
  EXPECT_NE(bsdf.find("EE_GLTF_RT_BSDF_SAMPLE"), std::string::npos);
  EXPECT_NE(bsdf.find("material.transmission_factor"), std::string::npos);
  EXPECT_NE(bsdf.find("material.ior"), std::string::npos);
  EXPECT_NE(bsdf.find("material.clearcoat_factor"), std::string::npos);
  EXPECT_NE(bsdf.find("material.sheen_color_factor"), std::string::npos);
  EXPECT_NE(bsdf.find("material.anisotropy_strength"), std::string::npos);
  EXPECT_NE(bsdf.find("material.diffuse_transmission_factor"), std::string::npos);
  EXPECT_EQ(bsdf.find("bsdf_over_pdf * material.occlusion"), std::string::npos);
}

TEST(GltfRayTracingMaterial, CameraRaygenKeepsMappedNormalHemisphereGuard) {
  const auto source = ReadRayTracingCameraSource();
  const auto ray_query = ReadRayQueryCameraSource();
  const auto bsdf = ReadTextFile(ShaderPath("Includes/GltfRayTracingBsdf.glsl"));
  ASSERT_FALSE(source.empty());
  ASSERT_FALSE(ray_query.empty());
  ASSERT_FALSE(bsdf.empty());

  const auto expect_reference_hit_normal_order = [](const std::string& shader_source) {
    const auto reconstruction = shader_source.find("EE_CAMERA_SURFACE_HIT EE_CAMERA_RECONSTRUCT_SURFACE_HIT");
    ASSERT_NE(reconstruction, std::string::npos);
    const auto base_normal = shader_source.find("hit.normal = unflipped_shading_normal", reconstruction);
    const auto tangent_basis =
        shader_source.find("hit.bitangent = EE_CAMERA_SAFE_NORMALIZE(cross(hit.normal, hit.tangent)", reconstruction);
    const auto side_check = shader_source.find("if (dot(hit.normal, hit.geometric_normal) < 0.0f)", reconstruction);
    const auto tangent_flip = shader_source.find("hit.tangent = -hit.tangent", side_check);
    const auto reflection_clamp = shader_source.find("const vec3 reflected_direction = reflect", side_check);
    const auto hard_reset = shader_source.find("hit.normal = hit.geometric_normal", reflection_clamp);
    const auto shading_normal = shader_source.find("hit.shading_normal = hit.normal", hard_reset);
    const auto pbr_eval = shader_source.find("hit.pbr = EE_EVALUATE_GLTF_RAY_TRACING_PBR_MATERIAL", shading_normal);

    ASSERT_NE(base_normal, std::string::npos);
    ASSERT_NE(tangent_basis, std::string::npos);
    ASSERT_NE(side_check, std::string::npos);
    ASSERT_NE(tangent_flip, std::string::npos);
    ASSERT_NE(reflection_clamp, std::string::npos);
    ASSERT_NE(hard_reset, std::string::npos);
    ASSERT_NE(shading_normal, std::string::npos);
    ASSERT_NE(pbr_eval, std::string::npos);
    EXPECT_LT(base_normal, side_check);
    EXPECT_LT(base_normal, tangent_basis);
    EXPECT_LT(tangent_basis, side_check);
    EXPECT_LT(tangent_flip, reflection_clamp);
    EXPECT_LT(side_check, reflection_clamp);
    EXPECT_LT(reflection_clamp, hard_reset);
    EXPECT_LT(hard_reset, shading_normal);
    EXPECT_LT(shading_normal, pbr_eval);
  };

  const auto mapped_normal = bsdf.find("EE_GLTF_HAS_TEXTURE(material.normal_texture)");
  const auto tangent_update = bsdf.find("if (needs_tangent_update)");
  const auto pre_normal_tangent_projection = bsdf.find("pbr.tangent - pbr.normal * dot(pbr.normal, pbr.tangent)");

  ASSERT_NE(mapped_normal, std::string::npos);
  ASSERT_NE(tangent_update, std::string::npos);
  expect_reference_hit_normal_order(source);
  expect_reference_hit_normal_order(ray_query);
  EXPECT_LT(mapped_normal, tangent_update);
  EXPECT_EQ(pre_normal_tangent_projection, std::string::npos);
}

TEST(GltfRayTracingMaterial, CameraGeometricNormalsAreScaleIndependentWithVertexFallback) {
  const auto geometry = ReadTextFile(ShaderPath("Includes/CameraRayGeometry.glsl"));
  const auto integrator = ReadTextFile(ShaderPath("Includes/CameraRayIntegrator.glsl"));
  const auto any_hit = ReadTextFile(ShaderPath("RayTracing/AnyHit/Camera.rahit"));
  const auto ray_query = ReadTextFile(ShaderPath("Includes/CameraRayQueryTraversal.glsl"));
  ASSERT_FALSE(geometry.empty());
  ASSERT_FALSE(integrator.empty());
  ASSERT_FALSE(any_hit.empty());
  ASSERT_FALSE(ray_query.empty());

  EXPECT_NE(geometry.find("any(isnan(value)) || any(isinf(value))"), std::string::npos);
  EXPECT_NE(geometry.find("const float scale = max(max(abs(value.x), abs(value.y)), abs(value.z))"), std::string::npos);
  EXPECT_NE(geometry.find("const vec3 scaled_value = value / scale"), std::string::npos);
  EXPECT_NE(geometry.find("EE_CAMERA_SCALE_INDEPENDENT_NORMALIZE(edge_1, vec3(0.0f))"), std::string::npos);
  EXPECT_NE(geometry.find("EE_CAMERA_SCALE_INDEPENDENT_NORMALIZE(edge_2, vec3(0.0f))"), std::string::npos);
  EXPECT_NE(
      geometry.find("EE_CAMERA_SCALE_INDEPENDENT_NORMALIZE(cross(normalized_edge_1, normalized_edge_2), fallback)"),
      std::string::npos);
  EXPECT_EQ(geometry.find("0.00000001"), std::string::npos);

  for (const auto* source : {&integrator, &any_hit, &ray_query}) {
    EXPECT_NE(source->find("object_shading_normal"), std::string::npos);
    EXPECT_NE(source->find("EE_CAMERA_GEOMETRIC_NORMAL(v1.position - v0.position, v2.position - v0.position, "
                           "object_shading_normal)"),
              std::string::npos);
    EXPECT_EQ(source->find("EE_CAMERA_SAFE_NORMALIZE(cross(v1.position - v0.position"), std::string::npos);
  }
  EXPECT_NE(integrator.find("EE_CAMERA_SCALE_INDEPENDENT_NORMALIZE(normal_matrix * object_geometric_normal, "
                            "unflipped_shading_normal)"),
            std::string::npos);
  EXPECT_NE(integrator.find("object_light_shading_normal"), std::string::npos);
  EXPECT_NE(integrator.find("EE_CAMERA_SCALE_INDEPENDENT_NORMALIZE(light_normal_matrix * object_light_normal, "
                            "light_shading_normal)"),
            std::string::npos);
  EXPECT_NE(any_hit.find("EE_CAMERA_WORLD_NORMAL(object_geometric_normal, world_shading_normal)"), std::string::npos);
  EXPECT_NE(ray_query.find("EE_CAMERA_RAY_QUERY_WORLD_NORMAL(instance.model, object_geometric_normal"),
            std::string::npos);
}

TEST(GltfRayTracingMaterial, RayCamerasUseReferenceSafeOffsetsForSurfaceRays) {
  const auto raygen = ReadRayTracingCameraSource();
  const auto ray_query = ReadRayQueryCameraSource();
  ASSERT_FALSE(raygen.empty());
  ASSERT_FALSE(ray_query.empty());

  for (const auto* source : {&raygen, &ray_query}) {
    EXPECT_NE(source->find("vec3 shadow_position"), std::string::npos);
    EXPECT_NE(source->find("vec3 EE_CAMERA_POINT_OFFSET"), std::string::npos);
    EXPECT_NE(source->find("vec3 EE_CAMERA_SAFE_OFFSET_RAY"), std::string::npos);
    EXPECT_NE(source->find("floatBitsToInt(world_position.x)"), std::string::npos);
    EXPECT_NE(source->find("intBitsToFloat(floatBitsToInt(world_position.x)"), std::string::npos);
    EXPECT_NE(source->find("const float side_flip = dot(unflipped_geometric_normal, ray_direction) < 0.0f ? 1.0f : "
                           "-1.0f"),
              std::string::npos);
    EXPECT_NE(source->find("hit.geometric_normal = unflipped_geometric_normal * side_flip"), std::string::npos);
    EXPECT_NE(source->find("const vec3 v0_shadow_normal = v0.normal * side_flip"), std::string::npos);
    EXPECT_EQ(source->find("EE_CAMERA_SAFE_NORMALIZE(v0.normal, object_geometric_normal)"), std::string::npos);
    EXPECT_NE(source->find("hit.shadow_position = vec3(instance.model * vec4(object_shadow_position, 1.0f))"),
              std::string::npos);
    EXPECT_NE(source->find("const bool shadow_side_forward = dot(direct_light.direction, hit.shading_normal) > 0.0f"),
              std::string::npos);
    EXPECT_NE(source->find("? hit.shadow_position"), std::string::npos);
    EXPECT_NE(source->find("bounce.shadow_ray_origin = EE_CAMERA_SAFE_OFFSET_RAY(shadow_base, offset_direction)"),
              std::string::npos);
    EXPECT_NE(source->find("bounce.shadow_ray_distance = max(direct_light.distance, 0.0f)"), std::string::npos);
    EXPECT_NE(source->find("return EE_CAMERA_SAFE_OFFSET_RAY(hit.position, offset_direction)"), std::string::npos);
    EXPECT_EQ(source->find("hit.position + offset_direction * EE_CAMERA_RAY_EPSILON"), std::string::npos);
    EXPECT_EQ(source->find("bounce.shadow_ray_distance = max(direct_light.distance - EE_CAMERA_RAY_EPSILON"),
              std::string::npos);
  }
}

TEST(GltfRayTracingMaterial, RayTracingBsdfLetsLobesHandleBackfacingMappedNormals) {
  const auto bsdf = ReadTextFile(ShaderPath("Includes/GltfRayTracingBsdf.glsl"));
  ASSERT_FALSE(bsdf.empty());

  EXPECT_NE(bsdf.find("const vec3 forward_k1 = EE_GLTF_RT_SAFE_NORMALIZE(data.k1, material.normal)"),
            std::string::npos);
  EXPECT_NE(bsdf.find("EE_GLTF_RT_COMPUTE_LOBE_WEIGHTS(material, dot(material.normal, data.k1))"), std::string::npos);
  EXPECT_EQ(bsdf.find("clamped_v_dot_n"), std::string::npos);
  EXPECT_NE(bsdf.find("return mix(v_dot_n, sqrt(0.5f + 0.5f * v_dot_n)"), std::string::npos);
  EXPECT_NE(bsdf.find("EE_GLTF_RT_FRESNEL_COSINE_APPROXIMATION(v_dot_n"), std::string::npos);
  EXPECT_NE(bsdf.find("abs(v_dot_n)"), std::string::npos);
  EXPECT_EQ(bsdf.find("if (data.pdf <= EE_GLTF_RT_BSDF_EPSILON)"), std::string::npos);
  EXPECT_NE(bsdf.find("if (n_dot_l <= 0.0f)"), std::string::npos);
  EXPECT_NE(bsdf.find("if (n_dot_v <= 0.0f)"), std::string::npos);
  EXPECT_EQ(bsdf.find("if (dot(material.normal, forward_k1) <= 0.0f)"), std::string::npos);
}

TEST(GltfRayTracingMaterial, RayTracingDiffuseBsdfSamplesUseMaterialTangentFrame) {
  const auto bsdf = ReadTextFile(ShaderPath("Includes/GltfRayTracingBsdf.glsl"));
  const auto math = ReadTextFile(ShaderPath("Includes/GltfRayTracingMath.glsl"));
  ASSERT_FALSE(bsdf.empty());
  ASSERT_FALSE(math.empty());

  EXPECT_NE(math.find("vec3 EE_GLTF_RT_SAMPLE_COSINE_HEMISPHERE(const vec2 xi)"), std::string::npos);
  EXPECT_EQ(bsdf.find("mat3 EE_GLTF_RT_TANGENT_SPACE"), std::string::npos);
  EXPECT_EQ(bsdf.find("EE_GLTF_RT_SAMPLE_COSINE_HEMISPHERE(data.xi.xy, material.normal)"), std::string::npos);
  EXPECT_EQ(bsdf.find("EE_GLTF_RT_SAMPLE_COSINE_HEMISPHERE(data.xi.xy, -material.normal)"), std::string::npos);
  EXPECT_NE(bsdf.find("material.tangent * sampled_dir.x + material.bitangent * sampled_dir.y +\n"
                      "                                          material.normal * sampled_dir.z"),
            std::string::npos);
  EXPECT_NE(bsdf.find("material.tangent * sampled_dir.x + material.bitangent * sampled_dir.y -\n"
                      "                                          material.normal * sampled_dir.z"),
            std::string::npos);
}

TEST(GltfRayTracingMaterial, RetroreflectionSampleUsesMarginalBsdfAndPdf) {
  const auto bsdf = ReadTextFile(ShaderPath("Includes/GltfRayTracingBsdf.glsl"));
  const auto raygen = ReadRayTracingCameraSource();
  const auto ray_query = ReadRayQueryCameraSource();

  ASSERT_FALSE(bsdf.empty());
  ASSERT_FALSE(raygen.empty());
  ASSERT_FALSE(ray_query.empty());

  const auto sample_function = bsdf.find("void EE_GLTF_RT_BSDF_SAMPLE(inout");
  ASSERT_NE(sample_function, std::string::npos);
  const auto sample_body = bsdf.substr(sample_function);

  EXPECT_NE(sample_body.find("GltfRayTracingBsdfEvaluateData forward_data"), std::string::npos);
  EXPECT_NE(sample_body.find("EE_GLTF_RT_BSDF_EVALUATE_LOBE(forward_data"), std::string::npos);
  EXPECT_NE(sample_body.find("data.pdf = mix(forward_data.pdf, retro_data.pdf, retroreflection)"), std::string::npos);
  EXPECT_NE(sample_body.find("const vec3 mixture_bsdf = mix(forward_data.bsdf_diffuse + forward_data.bsdf_glossy"),
            std::string::npos);
  EXPECT_NE(sample_body.find("data.bsdf_over_pdf = data.pdf > EE_GLTF_RT_BSDF_EPSILON"), std::string::npos);
  const auto lobe_sample = sample_body.find("EE_GLTF_RT_BSDF_SAMPLE_LOBE(data, material, lobe, sample_weights)");
  ASSERT_NE(lobe_sample, std::string::npos);
  EXPECT_EQ(sample_body.find("material.occlusion"), std::string::npos);
  EXPECT_NE(sample_body.find("data.pdf <= EE_GLTF_RT_BSDF_MIN_PDF || any(isnan(data.bsdf_over_pdf))"),
            std::string::npos);
  EXPECT_NE(sample_body.find("data.event_type = EE_GLTF_RT_BSDF_EVENT_ABSORB"), std::string::npos);
  EXPECT_NE(sample_body.find("isinf(data.pdf)"), std::string::npos);
  EXPECT_NE(sample_body.find("EE_GLTF_RT_BSDF_EVENT_IMPULSE"), std::string::npos);
  EXPECT_NE(sample_body.find("data.pdf = EE_GLTF_RT_BSDF_DIRAC_PDF"), std::string::npos);

  for (const auto* source : {&raygen, &ray_query}) {
    EXPECT_NE(source->find("sample_data.pdf <= EE_CAMERA_PDF_EPSILON"), std::string::npos);
    EXPECT_NE(source->find("sample_data.pdf != EE_GLTF_RT_BSDF_DIRAC_PDF"), std::string::npos);
  }
}

TEST(GltfRayTracingMaterial, CameraPathRaysCullBackfacesButShadowRaysUseReferenceFlags) {
  const auto raygen = ReadRayTracingCameraSource();
  const auto ray_query = ReadRayQueryCameraSource();
  ASSERT_FALSE(raygen.empty());
  ASSERT_FALSE(ray_query.empty());

  EXPECT_NE(raygen.find("traceRayEXT(EE_TLAS, gl_RayFlagsCullBackFacingTrianglesEXT, 0xff"), std::string::npos);
  EXPECT_NE(ray_query.find("rayQueryInitializeEXT(ray_query, EE_TLAS, gl_RayFlagsCullBackFacingTrianglesEXT, 0xff"),
            std::string::npos);
  EXPECT_EQ(raygen.find("gl_RayFlagsNoOpaqueEXT | gl_RayFlagsCullBackFacingTrianglesEXT, 0xff"), std::string::npos);
  EXPECT_EQ(ray_query.find("gl_RayFlagsNoOpaqueEXT | gl_RayFlagsCullBackFacingTrianglesEXT, 0xff"), std::string::npos);
  EXPECT_NE(raygen.find("gl_RayFlagsTerminateOnFirstHitEXT | gl_RayFlagsSkipClosestHitShaderEXT"), std::string::npos);
  EXPECT_NE(ray_query.find("rayQueryInitializeEXT(ray_query, EE_TLAS, 0, EE_CAMERA_RAY_MASK_SHADOW"),
            std::string::npos);
  EXPECT_NE(ray_query.find("rayQueryGetIntersectionTypeEXT(ray_query, true) == "
                           "gl_RayQueryCommittedIntersectionTriangleEXT"),
            std::string::npos);
  EXPECT_EQ(raygen.find("gl_RayFlagsNoOpaqueEXT | gl_RayFlagsSkipClosestHitShaderEXT, EE_CAMERA_RAY_MASK_SHADOW"),
            std::string::npos);
  EXPECT_EQ(ray_query.find("gl_RayFlagsNoOpaqueEXT, EE_CAMERA_RAY_MASK_SHADOW"), std::string::npos);
}

TEST(GltfRayTracingMaterial, TopLevelAccelerationStructureUsesReferenceMaterialInstanceFlags) {
  const auto source = ReadTextFile(SdkPath("src/GraphicsResources.cpp"));
  ASSERT_FALSE(source.empty());

  EXPECT_NE(source.find("BuildGltfRayTracingInstanceFlags("), std::string::npos);
  EXPECT_NE(source.find("material.alpha_mode == static_cast<int32_t>(GltfAlphaMode::Opaque)"), std::string::npos);
  EXPECT_NE(source.find("material.transmission_factor == 0.0f"), std::string::npos);
  EXPECT_NE(source.find("material.diffuse_transmission_factor == 0.0f"), std::string::npos);
  EXPECT_NE(source.find("VK_GEOMETRY_INSTANCE_FORCE_OPAQUE_BIT_KHR"), std::string::npos);
  EXPECT_NE(source.find("material.double_sided != 0 || material.thickness_factor > 0.0f || "
                        "material.transmission_factor > 0.0f"),
            std::string::npos);
  EXPECT_NE(source.find("BuildGltfRayTracingInstanceFlags(*render_instance, gltf_shade_materials)"), std::string::npos);
  EXPECT_EQ(source.find("VK_GEOMETRY_INSTANCE_TRIANGLE_FLIP_FACING_BIT_KHR"), std::string::npos);
  EXPECT_EQ(source.find("acceleration_structure_instance.flags = "
                        "VK_GEOMETRY_INSTANCE_TRIANGLE_FACING_CULL_DISABLE_BIT_KHR;"),
            std::string::npos);
  EXPECT_NE(source.find("geometry.flags = VK_GEOMETRY_NO_DUPLICATE_ANY_HIT_INVOCATION_BIT_KHR;"), std::string::npos);
  EXPECT_EQ(source.find("VK_GEOMETRY_OPAQUE_BIT_KHR"), std::string::npos);
}

TEST(GltfRayTracingMaterial, TransmissionBsdfSplitsSpecularAndDiffuseLobes) {
  const auto bsdf = ReadTextFile(ShaderPath("Includes/GltfRayTracingBsdf.glsl"));
  const auto raygen = ReadRayTracingCameraSource();

  ASSERT_FALSE(bsdf.empty());
  ASSERT_FALSE(raygen.empty());

  EXPECT_NE(bsdf.find("EE_GLTF_RT_BSDF_LOBE_SPECULAR_TRANSMISSION"), std::string::npos);
  EXPECT_NE(bsdf.find("EE_GLTF_RT_BSDF_LOBE_DIFFUSE_TRANSMISSION"), std::string::npos);
  EXPECT_NE(bsdf.find("weights.specular_transmission = base_weight * transmission"), std::string::npos);
  EXPECT_NE(bsdf.find("weights.diffuse_transmission = remaining_weight * diffuse_transmission"), std::string::npos);
  EXPECT_NE(bsdf.find("EE_GLTF_RT_EVALUATE_GGX_TRANSMISSION"), std::string::npos);
  EXPECT_NE(bsdf.find("EE_GLTF_RT_SAMPLE_GGX_TRANSMISSION"), std::string::npos);
  EXPECT_NE(bsdf.find("EE_GLTF_RT_COMPUTE_HALF_VECTOR"), std::string::npos);
  EXPECT_NE(bsdf.find("EE_GLTF_RT_IS_TIR"), std::string::npos);
  EXPECT_NE(bsdf.find("EE_GLTF_RT_REFRACT"), std::string::npos);
  EXPECT_NE(bsdf.find("bool tir = false"), std::string::npos);
  EXPECT_NE(bsdf.find("tir ? EE_GLTF_RT_BSDF_EVENT_GLOSSY_REFLECTION"), std::string::npos);
  EXPECT_NE(bsdf.find("EE_GLTF_RT_EVALUATE_DIFFUSE_TRANSMISSION(data, material, material.diffuse_transmission_color)"),
            std::string::npos);
  EXPECT_NE(bsdf.find("EE_GLTF_RT_SAMPLE_DIFFUSE_TRANSMISSION(data, material, material.diffuse_transmission_color)"),
            std::string::npos);
  EXPECT_EQ(bsdf.find("const vec3 tint = mix(material.base_color, material.diffuse_transmission_color"),
            std::string::npos);
  EXPECT_EQ(bsdf.find("clamp(material.transmission + material.diffuse_transmission_factor"), std::string::npos);
  EXPECT_EQ(raygen.find("sample_data.event_type == EE_GLTF_RT_BSDF_EVENT_GLOSSY_TRANSMISSION"), std::string::npos);
  EXPECT_NE(raygen.find("(sample_data.event_type & EE_GLTF_RT_BSDF_EVENT_TRANSMISSION) != 0"), std::string::npos);
}

TEST(GltfRayTracingMaterial, RayTracingBtdfUsesReferenceGgxTransmissionModel) {
  const auto bsdf = ReadTextFile(ShaderPath("Includes/GltfRayTracingBsdf.glsl"));
  ASSERT_FALSE(bsdf.empty());

  EXPECT_NE(bsdf.find("EE_GLTF_RT_TRANSMISSION_IOR"), std::string::npos);
  EXPECT_NE(bsdf.find("EE_GLTF_RT_COMPUTE_DISPERSED_IOR"), std::string::npos);
  EXPECT_NE(bsdf.find("EE_GLTF_RT_WAVELENGTH_TO_RGB"), std::string::npos);
  EXPECT_NE(bsdf.find("EE_GLTF_RT_RERANDOMIZE"), std::string::npos);
  EXPECT_NE(bsdf.find("EE_GLTF_RT_COMPUTE_HALF_VECTOR"), std::string::npos);
  EXPECT_NE(bsdf.find("EE_GLTF_RT_HVD_GGX_SAMPLE_VNDF(local_k1, roughness, data.xi.xy)"), std::string::npos);
  EXPECT_NE(bsdf.find("EE_GLTF_RT_REFRACT(data.k1, half_vector"), std::string::npos);
  EXPECT_NE(bsdf.find("const bool thin_walled = material.thickness <="), std::string::npos);
  EXPECT_NE(bsdf.find("data.k2 = (2.0f * k_dot_h) * half_vector - data.k1"), std::string::npos);
  EXPECT_NE(bsdf.find("data.bsdf_over_pdf *= g2"), std::string::npos);
  EXPECT_NE(bsdf.find("EE_GLTF_RT_SAMPLE_GGX_TRANSMISSION(data, material, weights.tint)"), std::string::npos);

  EXPECT_EQ(bsdf.find("EE_GLTF_RT_SAMPLE_GGX_HALF_VECTOR"), std::string::npos);
  EXPECT_EQ(bsdf.find("EE_GLTF_RT_GGX_TRANSMISSION_PDF"), std::string::npos);
  EXPECT_EQ(bsdf.find("max(material.roughness.x, material.roughness.y)"), std::string::npos);
  EXPECT_EQ(bsdf.find("view_direction + light_direction * eta"), std::string::npos);
}

TEST(GltfRayTracingMaterial, RayTracingBsdfUsesReferenceLayeredLobeModel) {
  const auto bsdf = ReadTextFile(ShaderPath("Includes/GltfRayTracingBsdf.glsl"));
  const auto raygen = ReadRayTracingCameraSource();

  ASSERT_FALSE(bsdf.empty());
  ASSERT_FALSE(raygen.empty());

  EXPECT_NE(bsdf.find("struct GltfRayTracingBsdfLobeWeights"), std::string::npos);
  EXPECT_NE(bsdf.find("EE_GLTF_RT_COMPUTE_LOBE_WEIGHTS"), std::string::npos);
  EXPECT_NE(bsdf.find("EE_GLTF_RT_FIND_BSDF_LOBE"), std::string::npos);
  EXPECT_NE(bsdf.find("EE_GLTF_RT_BSDF_LOBE_DIFFUSE_REFLECTION"), std::string::npos);
  EXPECT_NE(bsdf.find("EE_GLTF_RT_BSDF_LOBE_SPECULAR_TRANSMISSION"), std::string::npos);
  EXPECT_NE(bsdf.find("EE_GLTF_RT_BSDF_LOBE_SPECULAR_REFLECTION"), std::string::npos);
  EXPECT_NE(bsdf.find("EE_GLTF_RT_BSDF_LOBE_METAL_REFLECTION"), std::string::npos);
  EXPECT_NE(bsdf.find("EE_GLTF_RT_BSDF_LOBE_SHEEN_REFLECTION"), std::string::npos);
  EXPECT_NE(bsdf.find("EE_GLTF_RT_BSDF_LOBE_CLEARCOAT_REFLECTION"), std::string::npos);
  EXPECT_NE(bsdf.find("EE_GLTF_RT_BSDF_LOBE_DIFFUSE_TRANSMISSION"), std::string::npos);
  EXPECT_NE(bsdf.find("weights.clearcoat_reflection = coat_fresnel"), std::string::npos);
  EXPECT_NE(bsdf.find("weights.sheen_reflection = base_weight * sheen"), std::string::npos);
  EXPECT_NE(bsdf.find("weights.metal_reflection = base_weight * clamp(material.metallic"), std::string::npos);
  EXPECT_NE(bsdf.find("weights.specular_reflection = base_weight * dielectric_fresnel_weight"), std::string::npos);
  EXPECT_EQ(bsdf.find("EE_GLTF_RT_BSDF_LOBE_PROBABILITIES"), std::string::npos);
  EXPECT_EQ(bsdf.find("glossy_weight ="), std::string::npos);

  EXPECT_NE(bsdf.find("EE_GLTF_RT_EVALUATE_GGX_REFLECTION_LOBE"), std::string::npos);
  EXPECT_NE(bsdf.find("EE_GLTF_RT_EVALUATE_SHEEN_REFLECTION"), std::string::npos);
  EXPECT_NE(bsdf.find("EE_GLTF_RT_SAMPLE_GGX_REFLECTION_LOBE"), std::string::npos);
  EXPECT_NE(bsdf.find("EE_GLTF_RT_SAMPLE_SHEEN_REFLECTION"), std::string::npos);
  EXPECT_EQ(bsdf.find("data.bsdf_glossy += material.clearcoat"), std::string::npos);
  EXPECT_EQ(bsdf.find("data.bsdf_glossy += material.sheen_color"), std::string::npos);

  EXPECT_NE(raygen.find("EE_CAMERA_EVALUATE_DIRECT_BSDF(const EE_CAMERA_SURFACE_HIT hit"), std::string::npos);
  EXPECT_NE(raygen.find("eval_data.xi = EE_PCG_RANDOM_3(seed)"), std::string::npos);
  EXPECT_EQ(raygen.find("eval_data.xi = vec3(0.0f);\n  EE_GLTF_RT_BSDF_EVALUATE(eval_data, hit.pbr);\n  bsdf_pdf"),
            std::string::npos);
}

TEST(GltfRayTracingMaterial, AdvancedRayExtensionsFollowKhronosAndShareOneBsdf) {
  const auto bsdf = ReadTextFile(ShaderPath("Includes/GltfRayTracingBsdf.glsl"));
  const auto material = ReadTextFile(ShaderPath("Includes/GltfMaterial.glsl"));
  const auto integrator = ReadRayTracingCameraSource();
  const auto any_hit = ReadTextFile(ShaderPath("RayTracing/AnyHit/Camera.rahit"));
  const auto ray_query = ReadTextFile(ShaderPath("Includes/CameraRayQueryTraversal.glsl"));

  ASSERT_FALSE(bsdf.empty());
  ASSERT_FALSE(material.empty());
  ASSERT_FALSE(integrator.empty());
  ASSERT_FALSE(any_hit.empty());
  ASSERT_FALSE(ray_query.empty());

  EXPECT_NE(bsdf.find("EE_GLTF_RT_IRIDESCENCE_SENSITIVITY"), std::string::npos);
  EXPECT_NE(bsdf.find("const vec3 base_ior = EE_GLTF_RT_F0_TO_IOR(base_f0)"), std::string::npos);
  EXPECT_NE(bsdf.find("material.specular * material.specular_f0"), std::string::npos);
  EXPECT_NE(bsdf.find("material.iridescence_ior, material.base_color"), std::string::npos);
  EXPECT_NE(bsdf.find("weights.dielectric_fresnel_weight = dielectric_fresnel_weight"), std::string::npos);

  EXPECT_NE(bsdf.find("const float c = material.anisotropy_rotation.x"), std::string::npos);
  EXPECT_NE(bsdf.find("const float s = material.anisotropy_rotation.y"), std::string::npos);
  EXPECT_NE(bsdf.find("c * anisotropy_direction.x - s * anisotropy_direction.y"), std::string::npos);
  EXPECT_NE(bsdf.find("s * anisotropy_direction.x + c * anisotropy_direction.y"), std::string::npos);
  EXPECT_EQ(bsdf.find("c * anisotropy_direction.x + s * anisotropy_direction.y"), std::string::npos);
  EXPECT_NE(bsdf.find("const float basis_handedness"), std::string::npos);
  EXPECT_EQ(bsdf.find("sign(dot(basis_bitangent"), std::string::npos);

  EXPECT_NE(bsdf.find("if (ior.x > ior.y)"), std::string::npos);
  EXPECT_NE(bsdf.find("ior.y = EE_GLTF_RT_COMPUTE_DISPERSED_IOR"), std::string::npos);
  EXPECT_EQ(bsdf.find("ior.x = EE_GLTF_RT_COMPUTE_DISPERSED_IOR(ior.x, material.dispersion, wavelength);\n    tint"),
            std::string::npos);

  EXPECT_NE(material.find("MAT_EXT_RETROREFLECTION"), std::string::npos);
  EXPECT_NE(bsdf.find("EE_GLTF_RT_IS_REFLECTION_LOBE"), std::string::npos);
  EXPECT_NE(bsdf.find("EE_GLTF_RT_BSDF_EVALUATE_LOBE"), std::string::npos);
  EXPECT_NE(bsdf.find("reflect(-data.k1, material.normal)"), std::string::npos);
  EXPECT_NE(bsdf.find("data.pdf = mix(forward_data.pdf, retro_data.pdf, retroreflection)"), std::string::npos);
  EXPECT_EQ(bsdf.find("path_probability"), std::string::npos);
  EXPECT_EQ(bsdf.find("data.bsdf_over_pdf /= path_probability"), std::string::npos);

  EXPECT_NE(bsdf.find("pbr.specular = clamp(material.specular_factor, 0.0f, 1.0f)"), std::string::npos);
  EXPECT_NE(bsdf.find("EE_GLTF_RT_WEIGHTED_SPECULAR_FRESNEL"), std::string::npos);
  EXPECT_NE(bsdf.find("vec3(weight)"), std::string::npos);
  EXPECT_NE(bsdf.find("pbr.specular_f0 * max(pbr.specular_color"), std::string::npos);
  EXPECT_NE(bsdf.find("normal_vector.xy *= material.clearcoat_normal_texture_scale"), std::string::npos);
  EXPECT_NE(bsdf.find("const vec3 clearcoat_basis_normal = pbr.normal"), std::string::npos);
  EXPECT_NE(bsdf.find("pbr.clearcoat_normal = clearcoat_basis_normal"), std::string::npos);
  EXPECT_NE(bsdf.find("EE_GLTF_RT_COATED_EMISSION"), std::string::npos);
  EXPECT_NE(integrator.find("EE_GLTF_RT_COATED_EMISSION_LOD0(uint(instance.material_index)"), std::string::npos);
  EXPECT_NE(integrator.find("EE_GLTF_RT_COATED_EMISSION(surface_hit.pbr, view_direction)"), std::string::npos);
  EXPECT_NE(bsdf.find("dielectric_ior = material.ior == 0.0f ? 0.0f"), std::string::npos);
  EXPECT_NE(bsdf.find("material.ior == 0.0f ? EE_GLTF_RT_IOR_COMPATIBILITY_INFINITY"), std::string::npos);
  EXPECT_NE(bsdf.find("if (material.ior != 0.0f)"), std::string::npos);
  EXPECT_EQ(bsdf.find("if (material.specular_factor > 0.0f)"), std::string::npos);
  EXPECT_NE(bsdf.find("pbr.thickness = material.thickness_factor"), std::string::npos);
  EXPECT_EQ(bsdf.find("pbr.thickness *=\n        EE_GLTF_SAMPLE_TEXTURE(material.thickness_texture"),
            std::string::npos);

  const auto unlit_test = integrator.find("EE_GLTF_MATERIALS[surface_hit.material_index].unlit > 0");
  const auto emissive_add = integrator.find("const vec3 emissive_contribution = throughput * emissive_hit_mis_weight");
  ASSERT_NE(unlit_test, std::string::npos);
  ASSERT_NE(emissive_add, std::string::npos);
  EXPECT_LT(unlit_test, emissive_add);
  EXPECT_NE(integrator.find("(sample_data.event_type & EE_GLTF_RT_BSDF_EVENT_TRANSMISSION) != 0"), std::string::npos);

  for (const auto* source : {&any_hit, &ray_query}) {
    EXPECT_NE(source->find("vertex_color.rgb"), std::string::npos);
  }
  EXPECT_NE(ray_query.find("EE_CAMERA_RAY_QUERY_SHADOW_TRANSMISSION"), std::string::npos);
}

TEST(GltfRayTracingMaterial, CameraRaygenProcessesVolumeSegmentBeforeSurfaceBounce) {
  const auto raygen = ReadRayTracingCameraSource();
  const auto bsdf = ReadTextFile(ShaderPath("Includes/GltfRayTracingBsdf.glsl"));

  ASSERT_FALSE(raygen.empty());
  ASSERT_FALSE(bsdf.empty());

  EXPECT_NE(bsdf.find("EE_GLTF_RT_MULTI_TO_SINGLE_SCATTER_ALBEDO"), std::string::npos);
  EXPECT_NE(bsdf.find("EE_GLTF_RT_VOLUME_EXTINCTION_COEFFICIENT"), std::string::npos);
  EXPECT_NE(bsdf.find("pbr.attenuation_distance = material.attenuation_distance"), std::string::npos);
  EXPECT_NE(bsdf.find("pbr.scatter_coefficient = attenuation_coefficient * single_scatter_albedo"), std::string::npos);

  EXPECT_NE(raygen.find("struct EE_CAMERA_VOLUME_MEDIUM"), std::string::npos);
  EXPECT_NE(raygen.find("EE_CAMERA_MAKE_VOLUME_MEDIUM"), std::string::npos);
  EXPECT_NE(raygen.find("EE_CAMERA_PROCESS_VOLUME_SEGMENT"), std::string::npos);
  EXPECT_NE(raygen.find("EE_CAMERA_HENYEY_GREENSTEIN_PDF"), std::string::npos);
  EXPECT_NE(raygen.find("EE_CAMERA_SAMPLE_HENYEY_GREENSTEIN"), std::string::npos);
  EXPECT_NE(raygen.find("EE_CAMERA_VOLUME_SCATTER_NEE"), std::string::npos);
  EXPECT_NE(raygen.find("throughput *= exp(-hit_distance * extinction)"), std::string::npos);
  EXPECT_NE(raygen.find("throughput *= exp(hit_distance * (vec3(max_extinction) - extinction))"), std::string::npos);
  EXPECT_NE(raygen.find("EE_CAMERA_SHADOW_TRANSMISSION(\n        shadow_origin, direct_light.direction"),
            std::string::npos);

  const auto reconstruct =
      raygen.find("EE_CAMERA_RECONSTRUCT_SURFACE_HIT(is_inside, ray_direction, ray_cone_width, ray_spread_angle)");
  const auto process_volume = raygen.find("EE_CAMERA_PROCESS_VOLUME_SEGMENT(", reconstruct);
  const auto direct_lighting =
      raygen.find("EE_CAMERA_PREPARE_DIRECT_LIGHTING(surface_hit, view_direction, throughput", process_volume);
  ASSERT_NE(reconstruct, std::string::npos);
  ASSERT_NE(process_volume, std::string::npos);
  ASSERT_NE(direct_lighting, std::string::npos);
  EXPECT_LT(reconstruct, process_volume);
  EXPECT_LT(process_volume, direct_lighting);
  const auto cone_advance = raygen.find("ray_cone_width += scatter_distance * ray_spread_angle");
  ASSERT_NE(cone_advance, std::string::npos);
  EXPECT_LT(cone_advance, process_volume);

  EXPECT_NE(raygen.find("volume_medium = entered_volume ? EE_CAMERA_MAKE_VOLUME_MEDIUM(surface_hit.pbr)"),
            std::string::npos);
  EXPECT_NE(raygen.find(": EE_CAMERA_EMPTY_VOLUME_MEDIUM()"), std::string::npos);
  EXPECT_NE(raygen.find("scatter_bounces >= EE_CAMERA_VOLUME_FREE_BUDGET"), std::string::npos);
  const auto volume_continue = raygen.find("continue;", process_volume);
  const auto surface_depth_increment = raygen.find("surface_depth += 1u", direct_lighting);
  ASSERT_NE(volume_continue, std::string::npos);
  ASSERT_NE(surface_depth_increment, std::string::npos);
  EXPECT_LT(volume_continue, surface_depth_increment);
}

TEST(GltfRayTracingMaterial, DistantLightAndEnvironmentShadowRangeIgnoreCameraFar) {
  const auto integrator = ReadTextFile(ShaderPath("Includes/CameraRayIntegrator.glsl"));
  const auto ray_tracing = ReadTextFile(ShaderPath("Includes/CameraRayTracingTraversal.glsl"));
  const auto ray_query = ReadTextFile(ShaderPath("Includes/CameraRayQueryTraversal.glsl"));
  const auto miss = ReadTextFile(ShaderPath("RayTracing/Miss/Camera.rmiss"));
  ASSERT_FALSE(integrator.empty());
  ASSERT_FALSE(ray_tracing.empty());
  ASSERT_FALSE(ray_query.empty());
  ASSERT_FALSE(miss.empty());

  EXPECT_NE(integrator.find("const float EE_CAMERA_MAX_TRACE_DISTANCE = 1e20f"), std::string::npos);
  EXPECT_NE(integrator.find("contrib.distance = EE_CAMERA_MAX_TRACE_DISTANCE"), std::string::npos);
  EXPECT_NE(integrator.find("direct_light.distance = EE_CAMERA_MAX_TRACE_DISTANCE"), std::string::npos);
  EXPECT_EQ(integrator.find("direct_light.distance = EE_CAMERA_FAR"), std::string::npos);
  EXPECT_NE(integrator.find("contrib.distance = distance"), std::string::npos);
  EXPECT_NE(ray_tracing.find("direction, EE_CAMERA_MAX_TRACE_DISTANCE, 0"), std::string::npos);
  EXPECT_NE(ray_query.find("direction,\n                        EE_CAMERA_MAX_TRACE_DISTANCE)"), std::string::npos);
  EXPECT_NE(integrator.find("primary_hit_distance = EE_CAMERA_FAR"), std::string::npos);
  EXPECT_NE(integrator.find("hit_value.hit_t = EE_CAMERA_FAR"), std::string::npos);
}

TEST(GltfRayTracingMaterial, CameraRaygenResolvesNeeBeforeSurfaceTermination) {
  const auto raygen = ReadRayTracingCameraSource();
  const auto ray_query = ReadRayQueryCameraSource();
  ASSERT_FALSE(raygen.empty());
  ASSERT_FALSE(ray_query.empty());

  EXPECT_NE(raygen.find("struct EE_CAMERA_BOUNCE_SCRATCH"), std::string::npos);
  EXPECT_NE(raygen.find("EE_CAMERA_PREPARE_DIRECT_LIGHTING"), std::string::npos);
  EXPECT_NE(raygen.find("EE_CAMERA_RESOLVE_DIRECT_LIGHTING"), std::string::npos);
  EXPECT_NE(raygen.find("bounce.next_event_valid = true"), std::string::npos);
  EXPECT_NE(raygen.find("bounce.contribution = contribution"), std::string::npos);
  EXPECT_NE(raygen.find("bounce.shadow_ray_origin"), std::string::npos);
  EXPECT_NE(raygen.find("bounce.shadow_ray_direction"), std::string::npos);
  EXPECT_NE(raygen.find("bounce.shadow_ray_distance"), std::string::npos);
  EXPECT_NE(raygen.find("EE_CAMERA_SHADOW_TRANSMISSION(\n      bounce.shadow_ray_origin"), std::string::npos);

  for (const auto* source : {&raygen, &ray_query}) {
    const auto prepare_function = source->find("void EE_CAMERA_PREPARE_DIRECT_LIGHTING");
    const auto resolve_function = source->find("vec3 EE_CAMERA_RESOLVE_DIRECT_LIGHTING", prepare_function);
    ASSERT_NE(prepare_function, std::string::npos);
    ASSERT_NE(resolve_function, std::string::npos);
    const auto prepare_body = source->substr(prepare_function, resolve_function - prepare_function);
    const auto pdf_gate = prepare_body.find("if (direct_light.pdf == 0.0f)");
    const auto valid = prepare_body.find("bounce.next_event_valid = true");
    const auto evaluate = prepare_body.find("EE_CAMERA_EVALUATE_DIRECT_BSDF", valid);
    const auto zero_contribution = prepare_body.find("if (max(contribution.x", evaluate);
    const auto assign_contribution = prepare_body.find("bounce.contribution = contribution", zero_contribution);
    ASSERT_NE(pdf_gate, std::string::npos);
    ASSERT_NE(valid, std::string::npos);
    ASSERT_NE(evaluate, std::string::npos);
    ASSERT_NE(zero_contribution, std::string::npos);
    ASSERT_NE(assign_contribution, std::string::npos);
    EXPECT_LT(pdf_gate, valid);
    EXPECT_LT(valid, evaluate);
    EXPECT_LT(evaluate, zero_contribution);
    EXPECT_LT(zero_contribution, assign_contribution);
    EXPECT_EQ(prepare_body.find("max(direct_light.radiance_over_pdf.x"), std::string::npos);
  }

  const auto prepare =
      raygen.find("EE_CAMERA_PREPARE_DIRECT_LIGHTING(surface_hit, view_direction, throughput, surface_light_seed");
  const auto sample = raygen.find("EE_GLTF_RT_BSDF_SAMPLE(sample_data, surface_hit.pbr)", prepare);
  const auto absorb = raygen.find("sample_data.event_type == EE_GLTF_RT_BSDF_EVENT_ABSORB", sample);
  const auto resolve = raygen.find(
      "const vec3 direct_contribution = EE_CAMERA_RESOLVE_DIRECT_LIGHTING(bounce, surface_direct_shadow_seed)", sample);
  const auto debug_termination_guard = raygen.find("#if EE_CAMERA_ENABLE_DEBUG_VIEWS", absorb);
  const auto beauty_termination_else = raygen.find("#else", debug_termination_guard);
  const auto beauty_termination = raygen.find("surface_depth = max_depth", beauty_termination_else);
  const auto termination_guard_end = raygen.find("#endif", beauty_termination);
  const auto terminate = raygen.find("if (terminate_path)", resolve);
  const auto roulette = raygen.find("if (surface_depth >= EE_CAMERA_RUSSIAN_ROULETTE_MIN_DEPTH)", terminate);
  ASSERT_NE(prepare, std::string::npos);
  ASSERT_NE(sample, std::string::npos);
  ASSERT_NE(absorb, std::string::npos);
  ASSERT_NE(debug_termination_guard, std::string::npos);
  ASSERT_NE(beauty_termination_else, std::string::npos);
  ASSERT_NE(beauty_termination, std::string::npos);
  ASSERT_NE(termination_guard_end, std::string::npos);
  ASSERT_NE(resolve, std::string::npos);
  ASSERT_NE(terminate, std::string::npos);
  ASSERT_NE(roulette, std::string::npos);
  EXPECT_LT(prepare, sample);
  EXPECT_LT(sample, resolve);
  EXPECT_LT(absorb, resolve);
  EXPECT_LT(debug_termination_guard, beauty_termination_else);
  EXPECT_LT(beauty_termination_else, beauty_termination);
  EXPECT_LT(beauty_termination, termination_guard_end);
  EXPECT_LT(termination_guard_end, resolve);
  EXPECT_LT(resolve, terminate);
  EXPECT_LT(terminate, roulette);
}

TEST(GltfRayTracingMaterial, CameraRaygenBuildsReferenceStylePrimaryRays) {
  const auto source = ReadRayTracingCameraSource();
  ASSERT_FALSE(source.empty());

  EXPECT_NE(source.find("const vec2 clip_coords = (sample_position + sample_offset) / image_size * 2.0f - 1.0f"),
            std::string::npos);
  EXPECT_NE(source.find("EE_CAMERA_CREATE_PRIMARY_RAY(camera, vec2(pixel_coordinate), sample_offset, "
                        "vec2(image_size))"),
            std::string::npos);
  EXPECT_NE(source.find("camera.inverse_projection * vec4(clip_coords, -1.0f, 1.0f)"), std::string::npos);
  EXPECT_NE(source.find("const vec3 origin = camera.inverse_view[3].xyz"), std::string::npos);
  EXPECT_NE(source.find("camera.inverse_view * view_position"), std::string::npos);
  EXPECT_NE(source.find("EE_CAMERA_TRACE_PATH(sample_seed, primary_ray.origin, primary_ray.direction"),
            std::string::npos);
  EXPECT_EQ(source.find("- vec2(0.5f)"), std::string::npos);
  EXPECT_EQ(source.find("camera.inverse_projection_view * vec4(d.x, d.y"), std::string::npos);
}

TEST(GltfRayTracingMaterial, RayTracingNormalMapsPreserveImportedTangentHandedness) {
  const auto prefab_source = ReadTextFile(SdkPath("src/Prefab.cpp"));
  const auto raster_material = ReadTextFile(ShaderPath("Includes/GltfRasterMaterial.glsl"));

  ASSERT_FALSE(prefab_source.empty());
  ASSERT_FALSE(raster_material.empty());

  EXPECT_NE(prefab_source.find("float ImportedTangentHandedness(const aiMesh* mesh, const int vertex_index)"),
            std::string::npos);
  EXPECT_NE(prefab_source.find("mesh->HasTangentsAndBitangents()"), std::string::npos);
  EXPECT_NE(prefab_source.find("mesh->mBitangents[vertex_index]"), std::string::npos);
  EXPECT_NE(prefab_source.find("glm::dot(glm::cross(tangent, bitangent), normal) < 0.0f ? -1.0f : 1.0f"),
            std::string::npos);
  EXPECT_NE(prefab_source.find("vertex.vertex_info3 = ImportedTangentHandedness(importer_mesh, i)"), std::string::npos);
  EXPECT_NE(raster_material.find("float tangent_handedness"), std::string::npos);
  EXPECT_NE(raster_material.find("tangent - n * dot(n, tangent)"), std::string::npos);
  EXPECT_NE(raster_material.find("vec3 b = cross(n, t) * bitangent_sign"), std::string::npos);
  EXPECT_NE(
      raster_material.find(
          "return EE_EVALUATE_GLTF_RASTER_NORMAL(material_index, tex_coord_0, tex_coord_1, normal, tangent, 1.0)"),
      std::string::npos);
}

TEST(GltfRayTracingMaterial, GeneratedTangentsUseMikkAndPreserveHandedness) {
  const auto mesh_source = ReadTextFile(SdkPath("src/Mesh.cpp"));
  const auto skinned_mesh_source = ReadTextFile(SdkPath("src/SkinnedMesh.cpp"));
  const auto mikk_source = ReadTextFile(SdkPath("src/MikkTangentSpace.cpp"));
  ASSERT_FALSE(mesh_source.empty());
  ASSERT_FALSE(skinned_mesh_source.empty());
  ASSERT_FALSE(mikk_source.empty());

  for (const auto* source : {&mesh_source, &skinned_mesh_source}) {
    EXPECT_NE(source->find("GenerateMikkTangents"), std::string::npos);
  }
  EXPECT_NE(mikk_source.find("genTangSpaceDefault(&context)"), std::string::npos);
  EXPECT_NE(mikk_source.find("vertices.emplace_back(vertices[source_index])"), std::string::npos);
  EXPECT_NE(mikk_source.find("vertices[target_index].vertex_info3 = tangent.w"), std::string::npos);
  EXPECT_NE(mikk_source.find("case 3:"), std::string::npos);
}

TEST(GltfRayTracingMaterial, CameraRaygenDoesNotBindPrimaryDdgiResources) {
  const auto source = ReadRayTracingCameraSource();
  ASSERT_FALSE(source.empty());
  EXPECT_EQ(source.find("layout(set = 2, binding = 17) uniform sampler2D EE_DDGI_IRRADIANCE_ATLAS"), std::string::npos);
  EXPECT_EQ(source.find("layout(set = 2, binding = 18) uniform sampler2D EE_DDGI_VISIBILITY_ATLAS"), std::string::npos);
  EXPECT_EQ(source.find("layout(set = 2, binding = 19) readonly buffer EE_DDGI_PROBE_STATE_BLOCK"), std::string::npos);
  EXPECT_EQ(source.find("EE_CAMERA_PRIMARY_DDGI_DIFFUSE"), std::string::npos);

  const auto pass = ReadTextFile(SdkPath("src/RenderPasses/RayTracingCameraPass.cpp"));
  ASSERT_FALSE(pass.empty());
  EXPECT_NE(pass.find("RayTracingCameraPass::CreateDescriptor()"), std::string::npos);
  EXPECT_EQ(pass.find("read_ddgi_resources"), std::string::npos);
  EXPECT_EQ(pass.find("RenderResourceNames::frame_ddgi_probe_state"), std::string::npos);
  EXPECT_EQ(pass.find("RenderResourceNames::frame_ddgi_irradiance_atlas"), std::string::npos);
  EXPECT_EQ(pass.find("RenderResourceNames::frame_ddgi_visibility_atlas"), std::string::npos);
  EXPECT_EQ(pass.find("CreateDdgiFallbackImageInfo()"), std::string::npos);
  EXPECT_EQ(pass.find("UpdateImageDescriptorBinding(17"), std::string::npos);
  EXPECT_EQ(pass.find("UpdateImageDescriptorBinding(18"), std::string::npos);
  EXPECT_EQ(pass.find("UpdateBufferDescriptorBinding(19"), std::string::npos);

  const auto render_layer = ReadTextFile(SdkPath("src/RenderLayer.cpp"));
  ASSERT_FALSE(render_layer.empty());
  EXPECT_EQ(render_layer.find("ray_tracing_camera_output_layout_->PushDescriptorBinding(17"), std::string::npos);
  EXPECT_EQ(render_layer.find("ray_tracing_camera_output_layout_->PushDescriptorBinding(18"), std::string::npos);
  EXPECT_EQ(render_layer.find("ray_tracing_camera_output_layout_->PushDescriptorBinding(19"), std::string::npos);
  EXPECT_EQ(render_layer.find("const bool ray_camera_samples_ddgi"), std::string::npos);
  EXPECT_EQ(render_layer.find("RayTracingCameraPass::CreateDescriptor(ray_camera_samples_ddgi)"), std::string::npos);
}

TEST(GltfRayTracingMaterial, PerFrameTextureBindingsExposeEnabledRayCameraStages) {
  const auto source = ReadTextFile(SdkPath("src/RenderLayer.cpp"));
  ASSERT_FALSE(source.empty());

  const auto binding_function = source.find("void PushPerFrameBindlessTextureDescriptorBindings");
  ASSERT_NE(binding_function, std::string::npos);
  const auto texture_2d_binding = source.find("9, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER", binding_function);
  ASSERT_NE(texture_2d_binding, std::string::npos);
  const auto cubemap_binding = source.find("10, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER");
  ASSERT_NE(cubemap_binding, std::string::npos);
  const auto material_buffer_binding = source.find("11, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER");
  ASSERT_NE(material_buffer_binding, std::string::npos);

  const auto ray_tracing_stage_gate = source.find("if (Platform::RayTracingEnabled())", binding_function);
  ASSERT_NE(ray_tracing_stage_gate, std::string::npos);
  EXPECT_LT(ray_tracing_stage_gate, texture_2d_binding);
  EXPECT_LT(source.find("VK_SHADER_STAGE_RAYGEN_BIT_KHR", ray_tracing_stage_gate), texture_2d_binding);
  EXPECT_NE(source.find("9, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, texture_2d_stages"), std::string::npos);
  EXPECT_NE(source.find("10, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, cubemap_stages"), std::string::npos);
  EXPECT_LT(texture_2d_binding, cubemap_binding);
  EXPECT_LT(cubemap_binding, material_buffer_binding);
}

TEST(GltfRayTracingMaterial, DemoPreviewRayCaptureWaitsForAccumulatedFramesAfterResets) {
  const auto source = ReadTextFile(AppPath("src/EvoEngineEditor.cpp"));
  ASSERT_FALSE(source.empty());

  EXPECT_NE(source.find("WaitForDemoPreviewSceneInputsReady()"), std::string::npos);
  EXPECT_NE(source.find("AssetManager::GetAssetLoadSnapshot().Active()"), std::string::npos);
  EXPECT_NE(source.find("TextureStorage::HasPendingUploads()"), std::string::npos);
  EXPECT_NE(source.find("Camera::ResolveCameraRenderMode(scene_camera->camera_render_mode)"), std::string::npos);
  EXPECT_NE(source.find("Camera::IsRayCameraRenderMode(resolved_render_mode)"), std::string::npos);
  EXPECT_NE(source.find("scene_camera->GetFrameCount() < warmup_frames"), std::string::npos);
  EXPECT_NE(source.find("Demo preview capture timed out before accumulating requested ray-tracing frames"),
            std::string::npos);
}

TEST(GltfRayTracingMaterial, DemoPreviewRayCaptureKeepsRequestedRenderResolution) {
  const auto editor_source = ReadTextFile(AppPath("src/EvoEngineEditor.cpp"));
  ASSERT_FALSE(editor_source.empty());
  EXPECT_NE(editor_source.find("editor_layer->SetSceneCameraResolutionOverride(preview_resolution)"),
            std::string::npos);
  EXPECT_NE(editor_source.find("const auto render_extent = render_texture->GetExtent()"), std::string::npos);
  EXPECT_NE(editor_source.find("Demo preview capture render texture resolution changed before saving"),
            std::string::npos);
  EXPECT_NE(editor_source.find("render_texture->Save(output_path)"), std::string::npos);
  EXPECT_EQ(editor_source.find("render_texture->StoreToPng(output_path, width, height)"), std::string::npos);

  const auto render_texture_source = ReadTextFile(SdkPath("src/RenderTexture.cpp"));
  ASSERT_FALSE(render_texture_source.empty());
  EXPECT_NE(render_texture_source.find("extension == \".hdr\""), std::string::npos);
  EXPECT_NE(render_texture_source.find("StoreToHdr(path.string())"), std::string::npos);

  const auto editor_layer_header = ReadTextFile(SdkPath("include/Layers/EditorLayer.hpp"));
  ASSERT_FALSE(editor_layer_header.empty());
  EXPECT_NE(editor_layer_header.find("SetSceneCameraResolutionOverride"), std::string::npos);
  EXPECT_NE(editor_layer_header.find("std::optional<glm::uvec2> scene_camera_resolution_override_"), std::string::npos);

  const auto editor_layer_source = ReadTextFile(SdkPath("src/EditorLayer.cpp"));
  ASSERT_FALSE(editor_layer_source.empty());
  EXPECT_NE(editor_layer_source.find("void EditorLayer::SetSceneCameraResolutionOverride"), std::string::npos);
  EXPECT_NE(editor_layer_source.find("if (scene_camera_resolution_override_)"), std::string::npos);
}

TEST(GltfRayTracingMaterial, StaticBlasBuilderUsesSharedCompactedGeometry) {
  const auto graphics = ReadTextFile(SdkPath("src/GraphicsResources.cpp"));
  const auto geometry = ReadTextFile(SdkPath("src/GeometryStorage.cpp"));
  const auto mesh = ReadTextFile(SdkPath("src/Mesh.cpp"));
  const auto skinned_mesh = ReadTextFile(SdkPath("src/SkinnedMesh.cpp"));
  const auto platform = ReadTextFile(SdkPath("src/Platform.cpp"));
  const auto application = ReadTextFile(SdkPath("src/Application.cpp"));
  const auto settings = ReadTextFile(SdkPath("include/ApplicationInitializationSettings.hpp"));
  const auto editor = ReadTextFile(AppPath("src/EvoEngineEditor.cpp"));
  ASSERT_FALSE(graphics.empty());
  ASSERT_FALSE(geometry.empty());
  ASSERT_FALSE(mesh.empty());
  ASSERT_FALSE(skinned_mesh.empty());
  ASSERT_FALSE(platform.empty());
  ASSERT_FALSE(application.empty());
  ASSERT_FALSE(settings.empty());
  ASSERT_FALSE(editor.empty());

  EXPECT_NE(mesh.find("BottomLevelAccelerationStructure::CreateStatic(meshlet_range_, triangle_range_, v_c)"),
            std::string::npos);
  EXPECT_NE(skinned_mesh.find("BottomLevelAccelerationStructure::CreateStatic(ray_tracing_meshlet_range_,"),
            std::string::npos);
  EXPECT_EQ(mesh.find("make_shared<BottomLevelAccelerationStructure>"), std::string::npos);
  EXPECT_NE(geometry.find("VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR"), std::string::npos);
  EXPECT_NE(geometry.find("BottomLevelAccelerationStructure::ProcessStaticBuilds()"), std::string::npos);
  EXPECT_NE(geometry.find("if (Platform::RayAccelerationStructureEnabled())"), std::string::npos);
  EXPECT_NE(graphics.find("VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_COMPACTION_BIT_KHR"), std::string::npos);
  EXPECT_NE(graphics.find("VK_QUERY_TYPE_ACCELERATION_STRUCTURE_COMPACTED_SIZE_KHR"), std::string::npos);
  EXPECT_NE(graphics.find("VK_COPY_ACCELERATION_STRUCTURE_MODE_COMPACT_KHR"), std::string::npos);
  EXPECT_NE(graphics.find("DestroyAccelerationStructure(record.original)"), std::string::npos);
  EXPECT_NE(graphics.find("if (!render_instance || !blas || !blas->IsReady())"), std::string::npos);
  EXPECT_NE(platform.find("add_queue_request(graphics_family, kBackgroundQueuePriority)"), std::string::npos);
  EXPECT_NE(platform.find("add_queue_request(graphics_family, kInteractiveQueuePriority)"), std::string::npos);
  const auto timestamp_enable = application.find("this->initialization_settings.enable_gpu_timestamp_capture");
  const auto resource_initialize = application.find("Resources::Initialize()");
  ASSERT_NE(timestamp_enable, std::string::npos);
  ASSERT_NE(resource_initialize, std::string::npos);
  EXPECT_LT(timestamp_enable, resource_initialize);
  EXPECT_NE(settings.find("bool enable_gpu_timestamp_capture = false"), std::string::npos);
  EXPECT_NE(editor.find("BottomLevelAccelerationStructure::HasPendingStaticBuilds()"), std::string::npos);
}

TEST(GltfRayTracingMaterial, TlasUpdateClassifierFollowsVulkanCompatibilityRules) {
  using Tlas = evo_engine::TopLevelAccelerationStructure;
  const std::vector original = {MakeTlasTestInstance()};
  EXPECT_EQ(Tlas::ClassifyUpdateMode(false, {}, original), Tlas::UpdateMode::Build);
  EXPECT_EQ(Tlas::ClassifyUpdateMode(true, original, original), Tlas::UpdateMode::NoOp);
  EXPECT_EQ(Tlas::ClassifyUpdateMode(true, original, original, {7}, {7}), Tlas::UpdateMode::NoOp);
  EXPECT_EQ(Tlas::ClassifyUpdateMode(true, original, original, {7}, {8}), Tlas::UpdateMode::Update);
  EXPECT_TRUE(Tlas::PlanInstanceUploadRanges(false, original, original).empty());

  auto transformed = original;
  transformed[0].transform.matrix[0][3] = 2.0f;
  EXPECT_EQ(Tlas::ClassifyUpdateMode(true, original, transformed), Tlas::UpdateMode::Update);
  auto remasked = original;
  remasked[0].mask = 0x01;
  EXPECT_EQ(Tlas::ClassifyUpdateMode(true, original, remasked), Tlas::UpdateMode::Update);
  auto reflaged = original;
  reflaged[0].flags = VK_GEOMETRY_INSTANCE_FORCE_OPAQUE_BIT_KHR;
  EXPECT_EQ(Tlas::ClassifyUpdateMode(true, original, reflaged), Tlas::UpdateMode::Update);
  auto changed_blas = original;
  changed_blas[0].accelerationStructureReference = 2;
  EXPECT_EQ(Tlas::ClassifyUpdateMode(true, original, changed_blas), Tlas::UpdateMode::Update);

  auto inactive = original;
  inactive[0].mask = 0;
  inactive[0].accelerationStructureReference = 0;
  EXPECT_EQ(Tlas::ClassifyUpdateMode(true, original, inactive), Tlas::UpdateMode::Build);
  auto added = original;
  added.emplace_back(MakeTlasTestInstance(2));
  EXPECT_EQ(Tlas::ClassifyUpdateMode(true, original, added), Tlas::UpdateMode::Build);
  EXPECT_EQ(Tlas::ClassifyUpdateMode(false, {}, inactive), Tlas::UpdateMode::Build);
  EXPECT_EQ(Tlas::ClassifyUpdateMode(true, inactive, inactive), Tlas::UpdateMode::NoOp);
}

TEST(GltfRayTracingMaterial, TlasUploadPlannerCoalescesOnlyChangedInstances) {
  using Tlas = evo_engine::TopLevelAccelerationStructure;
  std::vector<VkAccelerationStructureInstanceKHR> original(6, MakeTlasTestInstance());
  auto changed = original;
  changed[1].transform.matrix[0][3] = 1.0f;
  changed[2].mask = 0x01;
  changed[4].flags = VK_GEOMETRY_INSTANCE_FORCE_OPAQUE_BIT_KHR;

  EXPECT_TRUE(Tlas::PlanInstanceUploadRanges(false, original, original).empty());
  const auto dirty = Tlas::PlanInstanceUploadRanges(false, original, changed);
  ASSERT_EQ(dirty.size(), 2);
  EXPECT_EQ(dirty[0].first_instance, 1);
  EXPECT_EQ(dirty[0].instance_count, 2);
  EXPECT_EQ(dirty[1].first_instance, 4);
  EXPECT_EQ(dirty[1].instance_count, 1);

  const auto full = Tlas::PlanInstanceUploadRanges(true, original, changed);
  ASSERT_EQ(full.size(), 1);
  EXPECT_EQ(full[0].first_instance, 0);
  EXPECT_EQ(full[0].instance_count, changed.size());

  auto grown = original;
  grown.emplace_back(MakeTlasTestInstance(2));
  const auto growth = Tlas::PlanInstanceUploadRanges(false, original, grown);
  ASSERT_EQ(growth.size(), 1);
  EXPECT_EQ(growth[0].first_instance, original.size());
  EXPECT_EQ(growth[0].instance_count, 1);

  auto shrunk = original;
  shrunk.pop_back();
  EXPECT_TRUE(Tlas::PlanInstanceUploadRanges(false, original, shrunk).empty());

  std::vector<VkAccelerationStructureInstanceKHR> shifted;
  for (VkDeviceAddress address = 1; address <= 4; ++address) {
    auto instance = MakeTlasTestInstance(address);
    instance.instanceCustomIndex = static_cast<uint32_t>(address);
    shifted.emplace_back(instance);
  }
  auto erased = shifted;
  erased.erase(erased.begin() + 1);
  const auto shifted_suffix = Tlas::PlanInstanceUploadRanges(false, shifted, erased);
  ASSERT_EQ(shifted_suffix.size(), 1);
  EXPECT_EQ(shifted_suffix[0].first_instance, 1);
  EXPECT_EQ(shifted_suffix[0].instance_count, 2);
  EXPECT_TRUE(Tlas::PlanInstanceUploadRanges(false, {}, {}).empty());
}

TEST(GltfRayTracingMaterial, PersistentTlasUsesMainQueueAndRayOnlyParticleInstances) {
  const auto header = ReadTextFile(SdkPath("include/Rendering/Platform/GraphicsResources.hpp"));
  const auto graphics = ReadTextFile(SdkPath("src/GraphicsResources.cpp"));
  const auto storage_header = ReadTextFile(SdkPath("include/Rendering/RenderInstances/RenderInstanceStorage.hpp"));
  const auto storage = ReadTextFile(SdkPath("src/RenderInstanceStorage.cpp"));
  const auto platform = ReadTextFile(SdkPath("src/Platform.cpp"));
  const auto render_layer = ReadTextFile(SdkPath("src/RenderLayer.cpp"));
  ASSERT_FALSE(header.empty());
  ASSERT_FALSE(graphics.empty());
  ASSERT_FALSE(storage_header.empty());
  ASSERT_FALSE(storage.empty());
  ASSERT_FALSE(platform.empty());
  ASSERT_FALSE(render_layer.empty());

  EXPECT_NE(header.find("pending_frame_count_"), std::string::npos);
  EXPECT_NE(header.find("pending_submission_state_"), std::string::npos);
  EXPECT_NE(header.find("ClassifyUpdateMode"), std::string::npos);
  EXPECT_NE(graphics.find("Platform::RecordCommandsMainQueue"), std::string::npos);
  EXPECT_NE(graphics.find("vkCmdCopyBuffer"), std::string::npos);
  EXPECT_NE(graphics.find("VK_BUILD_ACCELERATION_STRUCTURE_MODE_UPDATE_KHR"), std::string::npos);
  EXPECT_NE(graphics.find("instance.accelerationStructureReference = 0"), std::string::npos);
  EXPECT_NE(graphics.find("dummy.accelerationStructureReference = 0"), std::string::npos);
  EXPECT_EQ(graphics.find("ImmediateSubmitWithGpuTimestamp(\"TLAS Build\""), std::string::npos);
  EXPECT_NE(storage_header.find("ray_tracing_instance_indices"), std::string::npos);
  EXPECT_NE(storage.find("ray_instance_block.model.value"), std::string::npos);
  EXPECT_NE(storage.find("mesh_top_level_acceleration_structure->Update(*this)"), std::string::npos);
  EXPECT_EQ(storage.find("mesh_top_level_acceleration_structure.reset()"), std::string::npos);
  EXPECT_NE(platform.find("FrameSubmissionState::Status::Discarded"), std::string::npos);
  EXPECT_NE(platform.find("FrameSubmissionState::Status::Submitted"), std::string::npos);
  EXPECT_NE(platform.find("Platform::TrackCurrentFrameSubmission"), std::string::npos);
  EXPECT_NE(graphics.find("reuse_barrier.srcAccessMask = VK_ACCESS_2_SHADER_READ_BIT"), std::string::npos);
  EXPECT_NE(graphics.find("reuse_barrier.dstAccessMask = VK_ACCESS_2_TRANSFER_WRITE_BIT"), std::string::npos);
  EXPECT_NE(graphics.find("PlanInstanceUploadRanges"), std::string::npos);
  EXPECT_NE(graphics.find("copy_regions"), std::string::npos);
  EXPECT_NE(graphics.find("upload_byte_size"), std::string::npos);
  EXPECT_NE(graphics.find("pending_submission_state_->status == FrameSubmissionState::Status::Submitted"),
            std::string::npos);
  EXPECT_NE(render_layer.find("particle_info.instance_matrix.value"), std::string::npos);
  EXPECT_NE(render_layer.find("particle_info.instance_color"), std::string::npos);
}

TEST(GltfRayTracingMaterial, RenderInstanceMaterialAndTextureChangesResetRayCameraAccumulation) {
  const auto storage = ReadTextFile(SdkPath("src/RenderInstanceStorage.cpp"));
  ASSERT_FALSE(storage.empty());

  EXPECT_NE(storage.find("GetGltfShadeMaterials() != other.GetGltfShadeMaterials()"), std::string::npos);
  EXPECT_NE(storage.find("GetGltfTextureInfos() != other.GetGltfTextureInfos()"), std::string::npos);
  EXPECT_NE(storage.find("texture_storage_version != other.texture_storage_version"), std::string::npos);

  const auto render_layer = ReadTextFile(SdkPath("src/RenderLayer.cpp"));
  ASSERT_FALSE(render_layer.empty());
  const auto render_instance_updated = render_layer.find("if (render_instance_updated)");
  ASSERT_NE(render_instance_updated, std::string::npos);
  EXPECT_NE(render_layer.find("camera->frame_count_ = 0", render_instance_updated), std::string::npos);

  const auto texture_storage = ReadTextFile(SdkPath("src/TextureStorage.cpp"));
  ASSERT_FALSE(texture_storage.empty());
  const auto upload_completion = texture_storage.find("gpu_upload_generation_last_sync_ != upload_generation");
  ASSERT_NE(upload_completion, std::string::npos);
  EXPECT_NE(texture_storage.find("storage.version_++", upload_completion), std::string::npos);
}

TEST(GltfRayTracingMaterial, CameraRaygenWritesLinearRadianceForPostTonemapping) {
  const std::pair<std::string, std::string> sources[] = {{"Camera.rgen", ReadRayTracingCameraSource()}};

  for (const auto& [name, source] : sources) {
    ASSERT_FALSE(source.empty()) << name;

    EXPECT_NE(source.find("linear_radiance"), std::string::npos) << name;
    EXPECT_NE(source.find("previous_linear_radiance"), std::string::npos) << name;
    EXPECT_EQ(source.find("display_color"), std::string::npos) << name;
    EXPECT_EQ(source.find("pow(max(linear_radiance"), std::string::npos) << name;
    EXPECT_EQ(source.find("pow(prev_color"), std::string::npos) << name;
    EXPECT_EQ(source.find("pow(previous_linear_radiance"), std::string::npos) << name;
    EXPECT_NE(source.find("imageStore(result_image"), std::string::npos) << name;
    EXPECT_NE(source.find("imageStore(radiance_history_image"), std::string::npos) << name;
    EXPECT_NE(source.find("vec4(linear_radiance"), std::string::npos) << name;
  }
}

TEST(GltfRayTracingMaterial, RayTracingCameraRoutesLinearOutputThroughPostTonemapping) {
  const auto render_layer = ReadTextFile(SdkPath("src/RenderLayer.cpp"));
  ASSERT_FALSE(render_layer.empty());
  EXPECT_NE(render_layer.find("PostProcessingPass::CreateRayTracingDescriptor(post_ray_tracing_dependency)"),
            std::string::npos);
  EXPECT_NE(render_layer.find("{camera, active_camera_transient_resources, false, true}"), std::string::npos);

  const auto post_processing_pass = ReadTextFile(SdkPath("src/RenderPasses/PostProcessingPass.cpp"));
  ASSERT_FALSE(post_processing_pass.empty());
  EXPECT_NE(post_processing_pass.find("CreateRayTracingDescriptor"), std::string::npos);
  EXPECT_NE(post_processing_pass.find("ray_camera"), std::string::npos);
  EXPECT_NE(post_processing_pass.find("post_processing_stack->ProcessRayCamera"), std::string::npos);
}

TEST(GltfRayTracingMaterial, ToneMappingShaderUsesReferenceNvshadersPresentationPath) {
  const auto tone_mapping = ReadTextFile(ShaderPath("Compute/PostProcessing/ToneMapping.comp"));
  const auto histogram = ReadTextFile(ShaderPath("Compute/PostProcessing/ToneMappingHistogram.comp"));
  const auto auto_exposure = ReadTextFile(ShaderPath("Compute/PostProcessing/ToneMappingAutoExposure.comp"));
  ASSERT_FALSE(tone_mapping.empty());
  ASSERT_FALSE(histogram.empty());
  ASSERT_FALSE(auto_exposure.empty());

  EXPECT_NE(tone_mapping.find("nvpro_core2 nvshaders tonemap_functions.h.slang"), std::string::npos);
  EXPECT_NE(tone_mapping.find("SPDX-License-Identifier: Apache-2.0"), std::string::npos);
  EXPECT_NE(tone_mapping.find("EE_TONEMAP_FILMIC"), std::string::npos);
  EXPECT_NE(tone_mapping.find("EE_TONEMAP_UNCHARTED2"), std::string::npos);
  EXPECT_NE(tone_mapping.find("EE_TONEMAP_ACES"), std::string::npos);
  EXPECT_NE(tone_mapping.find("EE_TONEMAP_AGX"), std::string::npos);
  EXPECT_NE(tone_mapping.find("EE_TONEMAP_KHRONOS_PBR"), std::string::npos);
  EXPECT_NE(tone_mapping.find("adapted_luminance"), std::string::npos);
  EXPECT_EQ(tone_mapping.find("pow(linear_radiance"), std::string::npos);

  EXPECT_NE(histogram.find("EE_TONE_MAPPING_HISTOGRAM_BUCKET"), std::string::npos);
  EXPECT_NE(histogram.find("ev_min_value"), std::string::npos);
  EXPECT_NE(auto_exposure.find("average_mode == 1"), std::string::npos);
  EXPECT_NE(auto_exposure.find("adapted_luminance += (target_luminance - adapted_luminance)"), std::string::npos);
  EXPECT_NE(auto_exposure.find("bins[i] = 0u"), std::string::npos);
}

TEST(GltfRayTracingMaterial, CameraMissLeavesEnvironmentEvaluationInSharedIntegrator) {
  const auto miss = ReadTextFile(ShaderPath("RayTracing/Miss/Camera.rmiss"));
  const auto integrator = ReadTextFile(ShaderPath("Includes/CameraRayIntegrator.glsl"));
  ASSERT_FALSE(miss.empty());
  ASSERT_FALSE(integrator.empty());

  EXPECT_NE(miss.find("EE_CAMERA_RAY_PAYLOAD_MISS"), std::string::npos);
  EXPECT_NE(miss.find("hit_value.type = EE_CAMERA_RAY_PAYLOAD_MISS"), std::string::npos);
  EXPECT_EQ(miss.find("environment_radiance"), std::string::npos);
  EXPECT_EQ(miss.find("environment_pdf"), std::string::npos);
  EXPECT_EQ(miss.find("camera.skybox_tex_index"), std::string::npos);
  EXPECT_NE(integrator.find("vec3 EE_CAMERA_BACKGROUND_RADIANCE"), std::string::npos);
  EXPECT_NE(integrator.find("vec3 EE_CAMERA_PATH_ENVIRONMENT_RADIANCE"), std::string::npos);
  EXPECT_NE(integrator.find("float EE_CAMERA_PATH_ENVIRONMENT_HIT_PDF"), std::string::npos);
  EXPECT_NE(integrator.find("EE_CAMERA_SAMPLE_CUBEMAP_RADIANCE(camera.skybox_tex_index"), std::string::npos);
  EXPECT_EQ(integrator.find(std::string("#include \"Physical") + "Sky.glsl\""), std::string::npos);
  EXPECT_EQ(integrator.find(std::string("EE_") + "PHYSICAL_SKY_EVALUATE"), std::string::npos);
  EXPECT_EQ(integrator.find(std::string("EE_") + "PHYSICAL_SKY_PDF"), std::string::npos);
}

TEST(GltfRayTracingMaterial, CameraRaygenUsesSkyColorForPrimaryMisses) {
  const auto source = ReadRayTracingCameraSource();
  ASSERT_FALSE(source.empty());

  const auto primary_miss = source.find("if (surface_depth == 0u)");
  const auto sky_add = source.find("throughput * EE_CAMERA_BACKGROUND_RADIANCE(miss_direction)", primary_miss);
  const auto debug_environment = source.find("debug_view == EE_CAMERA_DEBUG_DIRECT_ENVIRONMENT", sky_add);
  const auto miss_break = source.find("break;", debug_environment);
  ASSERT_NE(primary_miss, std::string::npos);
  ASSERT_NE(sky_add, std::string::npos);
  ASSERT_NE(debug_environment, std::string::npos);
  ASSERT_NE(miss_break, std::string::npos);
  EXPECT_LT(sky_add, debug_environment);
  EXPECT_LT(debug_environment, miss_break);
  EXPECT_LT(sky_add,
            source.find("EE_CAMERA_ENVIRONMENT_HIT_MIS_WEIGHT(last_sample_pdf, environment_pdf, environment_weight)"));
}

TEST(GltfRayTracingMaterial, CameraRaygenLightsFromSceneEnvironment) {
  const auto source = ReadRayTracingCameraSource();
  ASSERT_FALSE(source.empty());

  EXPECT_NE(source.find("vec3 EE_CAMERA_PATH_ENVIRONMENT_RADIANCE"), std::string::npos);
  EXPECT_NE(source.find("EE_CAMERA_ENVIRONMENT_CUBEMAP_INDEX"), std::string::npos);
  EXPECT_NE(source.find("EE_CAMERA_SAMPLE_CUBEMAP_RADIANCE(cubemap_index, ray_direction"), std::string::npos);
  EXPECT_EQ(source.find("vec3 EE_CAMERA_BACKGROUND_LIGHT_RADIANCE"), std::string::npos);
  EXPECT_EQ(source.find(std::string("#include \"Physical") + "Sky.glsl\""), std::string::npos);
  EXPECT_EQ(source.find(std::string("EE_CAMERA_ENVIRONMENT_TYPE_PHYSICAL") + "_SKY"), std::string::npos);
  EXPECT_EQ(source.find(std::string("EE_") + "PHYSICAL_SKY_SAMPLE"), std::string::npos);
  EXPECT_EQ(source.find(std::string("EE_") + "PHYSICAL_SKY_PDF"), std::string::npos);
  EXPECT_NE(source.find("EE_CAMERA_SAMPLE_PATH_ENVIRONMENT(seed)"), std::string::npos);
  EXPECT_EQ(source.find(std::string("EE_ENVIRONMENT.environment_type == EE_CAMERA_ENVIRONMENT_TYPE_PHYSICAL") + "_SKY"),
            std::string::npos);
  EXPECT_NE(source.find("if (EE_ENVIRONMENT.light_intensity <= 0.0f)"), std::string::npos);
  EXPECT_NE(source.find("camera.use_clear_color == 1"), std::string::npos);
  EXPECT_NE(source.find("camera.clear_color.xyz"), std::string::npos);
  EXPECT_NE(source.find("EE_CAMERA_SAMPLE_CUBEMAP_RADIANCE(camera.skybox_tex_index"), std::string::npos);
  EXPECT_NE(source.find("EE_ENVIRONMENT.light_intensity"), std::string::npos);
  EXPECT_EQ(source.find("camera.prefiltered_map_index"), std::string::npos);
  EXPECT_EQ(source.find("camera.irradiance_map_index"), std::string::npos);
}

TEST(GltfRayTracingMaterial, DirectSkyEnvironmentModeRemoved) {
  const auto environment_include = ReadTextFile(ShaderPath("Includes/Environment.glsl"));
  const auto scene_header = ReadTextFile(SdkPath("include/Core/ECS/Scene.hpp"));
  const auto render_storage_header =
      ReadTextFile(SdkPath("include/Rendering/RenderInstances/RenderInstanceStorage.hpp"));
  const auto render_storage_source = ReadTextFile(SdkPath("src/RenderInstanceStorage.cpp"));
  const auto editor_source = ReadTextFile(SdkPath("src/Editor/SDKInspectionAdapters.cpp"));
  const auto raygen = ReadRayTracingCameraSource();
  const auto miss = ReadTextFile(ShaderPath("RayTracing/Miss/Camera.rmiss"));
  const auto ray_query = ReadRayQueryCameraSource();

  ASSERT_FALSE(environment_include.empty());
  ASSERT_FALSE(scene_header.empty());
  ASSERT_FALSE(render_storage_header.empty());
  ASSERT_FALSE(render_storage_source.empty());
  ASSERT_FALSE(editor_source.empty());
  ASSERT_FALSE(raygen.empty());
  ASSERT_FALSE(miss.empty());
  ASSERT_FALSE(ray_query.empty());

  EXPECT_NE(environment_include.find("float environment_type"), std::string::npos);
  EXPECT_NE(render_storage_header.find("float environment_type"), std::string::npos);
  EXPECT_EQ(scene_header.find(std::string("Physical") + "Sky"), std::string::npos);
  EXPECT_EQ(editor_source.find(std::string("Physical") + " Sky"), std::string::npos);
  EXPECT_EQ(render_storage_source.find(std::string("EnvironmentType::Physical") + "Sky"), std::string::npos);
  EXPECT_EQ(render_storage_source.find(std::string("environment_info_block.environment_type = ") + "2.0f"),
            std::string::npos);
  EXPECT_EQ(raygen.find(std::string("EE_") + "PHYSICAL_SKY"), std::string::npos);
  EXPECT_EQ(miss.find(std::string("EE_") + "PHYSICAL_SKY"), std::string::npos);
  EXPECT_EQ(ray_query.find(std::string("EE_") + "PHYSICAL_SKY"), std::string::npos);
  EXPECT_FALSE(
      std::filesystem::exists(ShaderPath(std::filesystem::path("Includes") / (std::string("Physical") + "Sky.glsl"))));
}

TEST(GltfRayTracingMaterial, SkyIlluminationCubemapBuildUsesNishitaAtmosphere) {
  const auto atmosphere = ReadTextFile(ShaderPath("Includes/Atmosphere.glsl"));
  const auto atmosphere_to_cubemap = ReadTextFile(ShaderPath("Graphics/Fragment/Lighting/AtmosphereToCubemap.frag"));
  const auto cubemap_source = ReadTextFile(SdkPath("src/Cubemap.cpp"));
  const auto environmental_map_source = ReadTextFile(SdkPath("src/EnvironmentalMap.cpp"));
  const auto editor_source = ReadTextFile(SdkPath("src/Editor/SDKInspectionAdapters.cpp"));

  ASSERT_FALSE(atmosphere.empty());
  ASSERT_FALSE(atmosphere_to_cubemap.empty());
  ASSERT_FALSE(cubemap_source.empty());
  ASSERT_FALSE(environmental_map_source.empty());
  ASSERT_FALSE(editor_source.empty());

  EXPECT_NE(atmosphere.find("NishitaSkyIncidentLight"), std::string::npos);
  EXPECT_NE(atmosphere_to_cubemap.find("NishitaSkyIncidentLight(atmosphere"), std::string::npos);
  EXPECT_NE(cubemap_source.find("Cubemap::BuildSkyIllumination"), std::string::npos);
  EXPECT_NE(cubemap_source.find("AtmosphereToCubemap.frag"), std::string::npos);
  EXPECT_NE(environmental_map_source.find("EnvironmentalMap::BuildSkyIllumination"), std::string::npos);
  EXPECT_NE(editor_source.find("InspectSkyIllumination"), std::string::npos);
  EXPECT_NE(editor_source.find("cubemap.BuildSkyIllumination(sky_illumination)"), std::string::npos);
  EXPECT_NE(editor_source.find("environmental_map.BuildSkyIllumination(sky_illumination)"), std::string::npos);
}

TEST(GltfRayTracingMaterial, EnvironmentMapModeUsesGeneratedPdfTexture) {
  const auto environment_include = ReadTextFile(ShaderPath("Includes/Environment.glsl"));
  const auto conversion_shader =
      ReadTextFile(ShaderPath("Graphics/Fragment/Lighting/EquirectangularMapToCubemap.frag"));
  const auto raygen = ReadRayTracingCameraSource();
  const auto miss = ReadTextFile(ShaderPath("RayTracing/Miss/Camera.rmiss"));
  const auto environmental_map_header = ReadTextFile(SdkPath("include/Rendering/PBR/EnvironmentalMap.hpp"));
  const auto environmental_map_source = ReadTextFile(SdkPath("src/EnvironmentalMap.cpp"));
  const auto cubemap_source = ReadTextFile(SdkPath("src/Cubemap.cpp"));
  const auto render_storage_header =
      ReadTextFile(SdkPath("include/Rendering/RenderInstances/RenderInstanceStorage.hpp"));
  const auto render_storage_source = ReadTextFile(SdkPath("src/RenderInstanceStorage.cpp"));
  const auto application_source = ReadTextFile(SdkPath("src/Application.cpp"));
  const auto scene_source = ReadTextFile(SdkPath("src/Scene.cpp"));
  const auto lighting = ReadTextFile(ShaderPath("Includes/Lighting.glsl"));

  ASSERT_FALSE(environment_include.empty());
  ASSERT_FALSE(conversion_shader.empty());
  ASSERT_FALSE(raygen.empty());
  ASSERT_FALSE(miss.empty());
  ASSERT_FALSE(environmental_map_header.empty());
  ASSERT_FALSE(environmental_map_source.empty());
  ASSERT_FALSE(cubemap_source.empty());
  ASSERT_FALSE(render_storage_header.empty());
  ASSERT_FALSE(render_storage_source.empty());
  ASSERT_FALSE(application_source.empty());
  ASSERT_FALSE(scene_source.empty());
  ASSERT_FALSE(lighting.empty());

  EXPECT_NE(environment_include.find("float environment_pdf_texture_index"), std::string::npos);
  EXPECT_NE(environmental_map_header.find("AssetRef environment_pdf_texture"), std::string::npos);
  EXPECT_NE(environmental_map_header.find("AssetRef environment_cubemap"), std::string::npos);
  EXPECT_NE(environmental_map_header.find("BuildEnvironmentPdfData"), std::string::npos);
  EXPECT_NE(environmental_map_source.find("BuildEnvironmentPdfTexture"), std::string::npos);
  EXPECT_NE(environmental_map_source.find("SetRgbaChannelData(cdf_pixels, resolution)"), std::string::npos);
  EXPECT_NE(environmental_map_source.find("environment_cubemap = cubemap"), std::string::npos);
  EXPECT_NE(environmental_map_source.find("environment_cubemap = target_cubemap"), std::string::npos);
  EXPECT_NE(application_source.find("environment_cubemap.Save"), std::string::npos);
  EXPECT_NE(application_source.find("environment_cubemap.Load"), std::string::npos);
  EXPECT_NE(cubemap_source.find("CalculateEnvironmentPdfScale"), std::string::npos);
  EXPECT_NE(conversion_shader.find("float ENVIRONMENT_PDF_SCALE"), std::string::npos);
  EXPECT_NE(conversion_shader.find("FragColor = vec4(color, pdf)"), std::string::npos);
  EXPECT_NE(render_storage_header.find("float environment_pdf_texture_index"), std::string::npos);
  EXPECT_NE(render_storage_header.find("float environment_cubemap_index"), std::string::npos);
  EXPECT_NE(render_storage_header.find("float environment_rotation"), std::string::npos);
  EXPECT_NE(render_storage_source.find("environment_info_block.environment_pdf_texture_index = -1.0f"),
            std::string::npos);
  EXPECT_NE(render_storage_source.find("environmental_map->environment_pdf_texture.Get<Texture2D>()"),
            std::string::npos);
  EXPECT_NE(render_storage_source.find("environmental_map->environment_cubemap.Get<Cubemap>()"), std::string::npos);
  EXPECT_NE(environment_include.find("EE_ENVIRONMENT_LOCAL_DIRECTION"), std::string::npos);
  EXPECT_NE(environment_include.find("EE_ENVIRONMENT_WORLD_DIRECTION"), std::string::npos);
  EXPECT_NE(scene_source.find("out << YAML::Key << \"environment_rotation\""), std::string::npos);
  EXPECT_NE(scene_source.find("environment_rotation = in[\"environment_rotation\"].as<float>()"), std::string::npos);
  EXPECT_NE(raygen.find("EE_CAMERA_SAMPLE_ENVIRONMENT_MAP"), std::string::npos);
  EXPECT_NE(raygen.find("EE_CAMERA_FIND_ENVIRONMENT_MARGINAL_ROW"), std::string::npos);
  EXPECT_NE(raygen.find("EE_CAMERA_FIND_ENVIRONMENT_CONDITIONAL_COLUMN"), std::string::npos);
  EXPECT_NE(raygen.find("EE_CAMERA_ENVIRONMENT_MAP_PDF(light_direction)"), std::string::npos);
  EXPECT_NE(raygen.find("marginal_sample - marginal_previous"), std::string::npos);
  EXPECT_NE(raygen.find("conditional_sample - conditional_previous"), std::string::npos);
  EXPECT_NE(raygen.find("const float marginal_probability = marginal_current - marginal_previous"), std::string::npos);
  EXPECT_NE(raygen.find("const float conditional_probability = conditional_current - conditional_previous"),
            std::string::npos);
  EXPECT_EQ(raygen.find("max(marginal_current - marginal_previous, EE_CAMERA_PDF_EPSILON)"), std::string::npos);
  EXPECT_EQ(raygen.find("max(conditional_current - conditional_previous, EE_CAMERA_PDF_EPSILON)"), std::string::npos);
  EXPECT_NE(raygen.find("mix(sin(elevation_0), sin(elevation_1), row_fraction)"), std::string::npos);
  EXPECT_NE(raygen.find("ray_sample.direction = EE_CAMERA_ENVIRONMENT_SPHERICAL_DIRECTION(uv)"), std::string::npos);
  EXPECT_NE(raygen.find("ray_sample.direction = EE_ENVIRONMENT_WORLD_DIRECTION(ray_sample.direction)"),
            std::string::npos);
  EXPECT_EQ(raygen.find("vec2(column, row) + vec2(0.5f)"), std::string::npos);
  EXPECT_NE(raygen.find("EE_CAMERA_ENVIRONMENT_MAP_PDF(light_direction)"), std::string::npos);
  EXPECT_NE(raygen.find("const float environment_pdf = EE_CAMERA_PATH_ENVIRONMENT_HIT_PDF(miss_direction)"),
            std::string::npos);
  EXPECT_EQ(raygen.find("hit_value.environment_pdf > 0.0f ?"), std::string::npos);
  EXPECT_EQ(raygen.find("environment_pdf > EE_CAMERA_PDF_EPSILON"), std::string::npos);
  EXPECT_NE(raygen.find("EE_ENVIRONMENT.environment_cubemap_index"), std::string::npos);
  EXPECT_NE(lighting.find("EE_ENVIRONMENT_LOCAL_DIRECTION(direction)"), std::string::npos);
  EXPECT_NE(lighting.find("EE_ENVIRONMENT_LOCAL_DIRECTION(normal)"), std::string::npos);
  EXPECT_NE(lighting.find("EE_ENVIRONMENT_LOCAL_DIRECTION(R)"), std::string::npos);
}

TEST(GltfRayTracingMaterial, EnvironmentPdfNormalizesConstantAndHighContrastMaps) {
  constexpr double kPi = 3.14159265358979323846;
  const glm::uvec2 resolution(4u, 2u);
  const std::vector<glm::vec4> constant_pixels(8u, glm::vec4(1.0f));
  const auto constant_pdf = evo_engine::EnvironmentalMap::BuildEnvironmentPdfData(constant_pixels, resolution);
  ASSERT_EQ(constant_pdf.size(), constant_pixels.size());

  double pdf_integral = 0.0;
  double radiance_integral = 0.0;
  for (uint32_t y = 0; y < resolution.y; ++y) {
    const double elevation_0 = (static_cast<double>(y) / resolution.y - 0.5) * kPi;
    const double elevation_1 = (static_cast<double>(y + 1u) / resolution.y - 0.5) * kPi;
    const double texel_solid_angle = 2.0 * kPi / resolution.x * (std::sin(elevation_1) - std::sin(elevation_0));
    for (uint32_t x = 0; x < resolution.x; ++x) {
      const auto& entry = constant_pdf[static_cast<size_t>(y) * resolution.x + x];
      EXPECT_NEAR(entry.b, 1.0 / (4.0 * kPi), 1e-6);
      pdf_integral += entry.b * texel_solid_angle;
      radiance_integral += texel_solid_angle;
    }
    EXPECT_FLOAT_EQ(constant_pdf[static_cast<size_t>(y + 1u) * resolution.x - 1u].r, 1.0f);
  }
  EXPECT_FLOAT_EQ(constant_pdf.back().g, 1.0f);
  EXPECT_NEAR(pdf_integral, 1.0, 1e-6);
  EXPECT_NEAR(radiance_integral, 4.0 * kPi, 1e-6);

  auto high_contrast_pixels = std::vector<glm::vec4>(8u, glm::vec4(0.1f));
  high_contrast_pixels[2] = glm::vec4(10.0f);
  const auto high_contrast_pdf =
      evo_engine::EnvironmentalMap::BuildEnvironmentPdfData(high_contrast_pixels, resolution);
  ASSERT_EQ(high_contrast_pdf.size(), high_contrast_pixels.size());
  double exact_integral = 0.0;
  double uniform_variance = 0.0;
  double importance_variance = 0.0;
  for (uint32_t y = 0; y < resolution.y; ++y) {
    const double elevation_0 = (static_cast<double>(y) / resolution.y - 0.5) * kPi;
    const double elevation_1 = (static_cast<double>(y + 1u) / resolution.y - 0.5) * kPi;
    const double texel_solid_angle = 2.0 * kPi / resolution.x * (std::sin(elevation_1) - std::sin(elevation_0));
    for (uint32_t x = 0; x < resolution.x; ++x) {
      const size_t index = static_cast<size_t>(y) * resolution.x + x;
      exact_integral += static_cast<double>(high_contrast_pixels[index].r) * texel_solid_angle;
    }
  }
  for (uint32_t y = 0; y < resolution.y; ++y) {
    const double elevation_0 = (static_cast<double>(y) / resolution.y - 0.5) * kPi;
    const double elevation_1 = (static_cast<double>(y + 1u) / resolution.y - 0.5) * kPi;
    const double texel_solid_angle = 2.0 * kPi / resolution.x * (std::sin(elevation_1) - std::sin(elevation_0));
    for (uint32_t x = 0; x < resolution.x; ++x) {
      const size_t index = static_cast<size_t>(y) * resolution.x + x;
      const double radiance = high_contrast_pixels[index].r;
      const double pdf = high_contrast_pdf[index].b;
      const double uniform_estimate = 4.0 * kPi * radiance;
      const double importance_estimate = radiance / pdf;
      uniform_variance +=
          texel_solid_angle / (4.0 * kPi) * (uniform_estimate - exact_integral) * (uniform_estimate - exact_integral);
      importance_variance +=
          pdf * texel_solid_angle * (importance_estimate - exact_integral) * (importance_estimate - exact_integral);
    }
  }
  EXPECT_GT(uniform_variance, 1.0);
  EXPECT_NEAR(importance_variance, 0.0, 1e-8);
  EXPECT_TRUE(
      evo_engine::EnvironmentalMap::BuildEnvironmentPdfData(std::vector<glm::vec4>(8u, glm::vec4(0.0f)), resolution)
          .empty());
}

TEST(GltfRayTracingMaterial, EnvironmentBlockTracksRaySourceAndRotation) {
  evo_engine::RenderInstanceStorage::EnvironmentInfoBlock baseline;
  auto changed = baseline;
  EXPECT_FALSE(baseline != changed);
  changed.environment_cubemap_index = 3.0f;
  EXPECT_TRUE(baseline != changed);
  changed = baseline;
  changed.environment_rotation = 0.5f;
  EXPECT_TRUE(baseline != changed);
}

TEST(GltfRayTracingMaterial, CameraRaygenUsesDomainSeparatedPcgAndAccumulation) {
  const auto random = ReadTextFile(ShaderPath("Includes/Random.glsl"));
  const auto raygen = ReadRayTracingCameraSource();
  const auto ray_query = ReadTextFile(ShaderPath("Includes/CameraRayQueryTraversal.glsl"));
  const auto any_hit = ReadTextFile(ShaderPath("RayTracing/AnyHit/Camera.rahit"));
  const auto render_storage_header =
      ReadTextFile(SdkPath("include/Rendering/RenderInstances/RenderInstanceStorage.hpp"));
  const auto ray_tracing_pass_source = ReadTextFile(SdkPath("src/RenderPasses/RayTracingCameraPass.cpp"));
  const auto editor_source = ReadTextFile(AppPath("src/EvoEngineEditor.cpp"));

  ASSERT_FALSE(random.empty());
  ASSERT_FALSE(raygen.empty());
  ASSERT_FALSE(ray_query.empty());
  ASSERT_FALSE(any_hit.empty());
  ASSERT_FALSE(render_storage_header.empty());
  ASSERT_FALSE(ray_tracing_pass_source.empty());
  ASSERT_FALSE(editor_source.empty());

  EXPECT_NE(random.find("uint EE_XXHASH32"), std::string::npos);
  EXPECT_NE(random.find("uint EE_PCG"), std::string::npos);
  EXPECT_NE(random.find("float EE_PCG_RANDOM"), std::string::npos);
  EXPECT_NE(random.find("vec2 EE_PCG_RANDOM_2"), std::string::npos);
  EXPECT_NE(random.find("vec3 EE_PCG_RANDOM_3"), std::string::npos);
  EXPECT_NE(random.find("const float x = EE_PCG_RANDOM(seed)"), std::string::npos);
  EXPECT_NE(raygen.find("uint EE_CAMERA_RANDOM_STREAM"), std::string::npos);
  EXPECT_NE(raygen.find("previous_accumulated_samples + i"), std::string::npos);
  EXPECT_NE(raygen.find("EE_CAMERA_RANDOM_DOMAIN_CAMERA"), std::string::npos);
  EXPECT_NE(raygen.find("EE_CAMERA_RANDOM_DOMAIN_SURFACE_ALPHA"), std::string::npos);
  EXPECT_NE(raygen.find("EE_CAMERA_RANDOM_DOMAIN_SURFACE_LIGHT"), std::string::npos);
  EXPECT_NE(raygen.find("EE_CAMERA_RANDOM_DOMAIN_SURFACE_BSDF"), std::string::npos);
  EXPECT_NE(raygen.find("EE_CAMERA_RANDOM_DOMAIN_VOLUME_SCATTER"), std::string::npos);
  EXPECT_NE(raygen.find("EE_CAMERA_RANDOM_DOMAIN_SURFACE_ROULETTE"), std::string::npos);
  EXPECT_NE(raygen.find("const uint current_segment = path_segment++"), std::string::npos);
  EXPECT_NE(raygen.find("vec3 EE_CAMERA_TRACE_PATH(const uint sample_seed"), std::string::npos);
  EXPECT_EQ(raygen.find("EE_RANDOM("), std::string::npos);
  EXPECT_EQ(ray_query.find("EE_RANDOM("), std::string::npos);
  EXPECT_EQ(any_hit.find("EE_RANDOM("), std::string::npos);
  EXPECT_NE(raygen.find("EE_CAMERA_ANTIALIASING_STANDARD_DEVIATION = 0.4246609f"), std::string::npos);
  EXPECT_NE(raygen.find("vec2 EE_CAMERA_SAMPLE_GAUSSIAN"), std::string::npos);
  EXPECT_NE(raygen.find("vec2(0.5f) + EE_CAMERA_ANTIALIASING_STANDARD_DEVIATION"), std::string::npos);
  EXPECT_NE(raygen.find("EE_CAMERA_SAMPLE_GAUSSIAN(EE_PCG_RANDOM_2(camera_seed))"), std::string::npos);
  EXPECT_NE(raygen.find(": EE_PCG_RANDOM_2(camera_seed)"), std::string::npos);
  EXPECT_NE(raygen.find("(sample_position + sample_offset) / image_size * 2.0f - 1.0f"), std::string::npos);
  EXPECT_NE(raygen.find("EE_TOTAL_SAMPLES > 0u"), std::string::npos);
  EXPECT_NE(raygen.find("previous_linear_radiance * float(previous_accumulated_samples)"), std::string::npos);
  EXPECT_NE(raygen.find("linear_radiance * float(frame_sample_size)"), std::string::npos);
  EXPECT_EQ(raygen.find("1664525u * EE_FRAME_ID"), std::string::npos);

  EXPECT_NE(render_storage_header.find("uint32_t total_samples"), std::string::npos);
  EXPECT_NE(render_storage_header.find("uint32_t frame_samples"), std::string::npos);
  EXPECT_NE(ray_tracing_pass_source.find("push_constant.frame_samples"), std::string::npos);
  EXPECT_NE(ray_tracing_pass_source.find(
                "push_constant.total_samples = push_constant.frame_id * push_constant.frame_samples"),
            std::string::npos);
  EXPECT_NE(editor_source.find("--preview-deterministic"), std::string::npos);
  EXPECT_NE(editor_source.find("--preview-sample-size"), std::string::npos);
  EXPECT_NE(editor_source.find("--preview-camera-position"), std::string::npos);
  EXPECT_NE(editor_source.find("--preview-camera-look-at"), std::string::npos);
  EXPECT_NE(editor_source.find("glm::quatLookAt(normalized_front, up)"), std::string::npos);
  EXPECT_NE(editor_source.find("tone_mapping->auto_exposure = false"), std::string::npos);
  EXPECT_NE(editor_source.find("tone_mapping->auto_exposure_delta_time_override = 1.0f / 60.0f"), std::string::npos);
  EXPECT_NE(editor_source.find("tone_mapping->dither = false"), std::string::npos);

  uint32_t state = 0u;
  const auto pcg = [](uint32_t& value) {
    const uint32_t previous = value * 747796405u + 2891336453u;
    const uint32_t word = ((previous >> ((previous >> 28u) + 4u)) ^ previous) * 277803737u;
    value = previous;
    return (word >> 22u) ^ word;
  };
  EXPECT_EQ(pcg(state), 0x07bb2fe2u);
  EXPECT_EQ(pcg(state), 0x22b6b6bcu);
  EXPECT_EQ(pcg(state), 0x3bf6e0b1u);
  EXPECT_EQ(pcg(state), 0x572f7439u);
}

TEST(GltfRayTracingMaterial, SamplingAndRotationNumericalContracts) {
  const auto rotate_left = [](const uint32_t value, const uint32_t count) {
    return value << count | value >> (32u - count);
  };
  const auto xxhash = [&](const uint32_t x, const uint32_t y, const uint32_t z) {
    constexpr uint32_t prime_x = 2246822519u;
    constexpr uint32_t prime_y = 3266489917u;
    constexpr uint32_t prime_z = 668265263u;
    constexpr uint32_t prime_w = 374761393u;
    uint32_t hash = z + prime_w + x * prime_y;
    hash = prime_z * rotate_left(hash, 17u);
    hash += y * prime_y;
    hash = prime_z * rotate_left(hash, 17u);
    hash = prime_x * (hash ^ hash >> 15u);
    hash = prime_y * (hash ^ hash >> 13u);
    return hash ^ hash >> 16u;
  };
  const uint32_t sample_0 = xxhash(17u, 29u, 0u);
  const uint32_t sample_1 = xxhash(17u, 29u, 1u);
  EXPECT_EQ(sample_0, 0x3a4ec941u);
  EXPECT_EQ(sample_1, 0xe2171f95u);
  EXPECT_EQ(xxhash(sample_0, 0u, 0u), 0x0de2ed24u);
  EXPECT_EQ(xxhash(sample_0, 1u, 0u), 0xa062d8b2u);
  EXPECT_EQ(xxhash(sample_1, 15u, 3u), 0x8114401bu);

  const auto rotate_y = [](const glm::vec3 direction, const float angle) {
    const float cosine = std::cos(angle);
    const float sine = std::sin(angle);
    return glm::vec3(cosine * direction.x + sine * direction.z, direction.y,
                     -sine * direction.x + cosine * direction.z);
  };
  const auto spherical_uv = [](const glm::vec3 direction) {
    constexpr float pi = 3.14159265358979323846f;
    const auto normalized = glm::normalize(direction);
    return glm::vec2(std::atan2(normalized.z, normalized.x) / (2.0f * pi) + 0.5f,
                     std::asin(glm::clamp(normalized.y, -1.0f, 1.0f)) / pi + 0.5f);
  };
  const glm::vec3 local_direction = glm::normalize(glm::vec3(0.3f, 0.4f, 0.8f));
  constexpr float rotation = 1.1f;
  const glm::vec3 world_direction = rotate_y(local_direction, rotation);
  const glm::vec3 recovered_direction = rotate_y(world_direction, -rotation);
  EXPECT_NEAR(glm::length(recovered_direction - local_direction), 0.0f, 1.0e-6f);
  EXPECT_NEAR(glm::length(spherical_uv(recovered_direction) - spherical_uv(local_direction)), 0.0f, 1.0e-6f);

  constexpr float inverse_projection_y = 1.25f;
  const float full_spread = 2.0f * inverse_projection_y / 720.0f;
  const float atlas_spread = 2.0f * inverse_projection_y / 144.0f;
  EXPECT_NEAR(atlas_spread, full_spread * 5.0f, 1.0e-7f);
  constexpr float scatter_distance = 0.75f;
  const float entry_width = 4.0f * atlas_spread;
  const float scattered_width = entry_width + scatter_distance * atlas_spread;
  EXPECT_NEAR(scattered_width, 4.75f * atlas_spread, 1.0e-7f);
  constexpr float next_hit_distance = 2.0f;
  constexpr float incidence_cosine = 0.8f;
  constexpr float texel_density = 0.5f;
  constexpr float texture_extent = 64.0f;
  const float gradient_before = (entry_width + next_hit_distance * atlas_spread) / incidence_cosine * texel_density;
  const float gradient_after = (scattered_width + next_hit_distance * atlas_spread) / incidence_cosine * texel_density;
  const float lod_before = std::log2(texture_extent * gradient_before);
  const float lod_after = std::log2(texture_extent * gradient_after);
  EXPECT_GT(lod_after, lod_before);
  EXPECT_NEAR(lod_after - lod_before, std::log2(gradient_after / gradient_before), 1.0e-6f);
}

TEST(GltfRayTracingMaterial, CameraRaygenUsesAutoSppConvergenceControl) {
  const auto raygen = ReadRayTracingCameraSource();
  const auto ray_query = ReadRayQueryCameraSource();
  const auto cameras_include = ReadTextFile(ShaderPath("Includes/Cameras.glsl"));
  const auto camera_header = ReadTextFile(SdkPath("include/Rendering/Camera.hpp"));
  const auto camera_settings = ReadTextFile(SdkPath("include/Rendering/CameraSettings.hpp"));
  const auto camera_source = ReadTextFile(SdkPath("src/Camera.cpp"));
  const auto ray_tracing_pass_source = ReadTextFile(SdkPath("src/RenderPasses/RayTracingCameraPass.cpp"));
  const auto render_layer_source = ReadTextFile(SdkPath("src/RenderLayer.cpp"));
  const auto application_source = ReadTextFile(SdkPath("src/Application.cpp"));
  const auto editor_layer_source = ReadTextFile(SdkPath("src/EditorLayer.cpp"));
  const auto inspection_source = ReadTextFile(SdkPath("src/Editor/SDKInspectionAdapters.cpp"));
  const auto editor_source = ReadTextFile(AppPath("src/EvoEngineEditor.cpp"));
  const auto python_binding =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "PythonBinding" / "src" / "PyEcoSysLab.cpp");

  ASSERT_FALSE(raygen.empty());
  ASSERT_FALSE(ray_query.empty());
  ASSERT_FALSE(cameras_include.empty());
  ASSERT_FALSE(camera_header.empty());
  ASSERT_FALSE(camera_settings.empty());
  ASSERT_FALSE(camera_source.empty());
  ASSERT_FALSE(ray_tracing_pass_source.empty());
  ASSERT_FALSE(render_layer_source.empty());
  ASSERT_FALSE(application_source.empty());
  ASSERT_FALSE(editor_layer_source.empty());
  ASSERT_FALSE(inspection_source.empty());
  ASSERT_FALSE(editor_source.empty());
  ASSERT_FALSE(python_binding.empty());

  EXPECT_NE(raygen.find("layout(set = 2, binding = 3, rgba32f) uniform image2D convergence_history_image"),
            std::string::npos);
  EXPECT_NE(raygen.find("float EE_CAMERA_RELATIVE_LUMINANCE_DELTA"), std::string::npos);
  EXPECT_NE(raygen.find("camera.auto_spp_enabled != 0u"), std::string::npos);
  EXPECT_NE(raygen.find("previous_auto_metadata.z > 0.5f"), std::string::npos);
  EXPECT_NE(raygen.find("previous_auto_samples >= auto_spp_max_samples"), std::string::npos);
  EXPECT_NE(raygen.find("imageStore(convergence_history_image"), std::string::npos);
  EXPECT_NE(raygen.find("auto_sample_converged ? 1.0f : 0.0f"), std::string::npos);

  EXPECT_NE(cameras_include.find("uint auto_spp_enabled"), std::string::npos);
  EXPECT_NE(cameras_include.find("uint auto_spp_min_samples"), std::string::npos);
  EXPECT_NE(cameras_include.find("uint auto_spp_max_samples"), std::string::npos);
  EXPECT_NE(cameras_include.find("float auto_spp_convergence_threshold"), std::string::npos);
  EXPECT_NE(cameras_include.find("uint auto_spp_padding2"), std::string::npos);
  EXPECT_NE(camera_header.find("uint32_t auto_spp_enabled = 0"), std::string::npos);
  EXPECT_NE(camera_header.find("uint32_t auto_spp_min_samples = 16"), std::string::npos);
  EXPECT_NE(camera_header.find("uint32_t auto_spp_max_samples = 256"), std::string::npos);
  EXPECT_NE(camera_header.find("float auto_spp_convergence_threshold = 0.01f"), std::string::npos);
  EXPECT_NE(camera_header.find("uint32_t auto_spp_padding2 = 0"), std::string::npos);
  EXPECT_NE(camera_settings.find("bool auto_spp_enabled = false"), std::string::npos);
  EXPECT_NE(camera_settings.find("int auto_spp_min_samples = 16"), std::string::npos);
  EXPECT_NE(camera_settings.find("int auto_spp_max_samples = 256"), std::string::npos);
  EXPECT_NE(camera_settings.find("float auto_spp_convergence_threshold = 0.01f"), std::string::npos);
  EXPECT_NE(camera_source.find("auto_spp_enabled != other.auto_spp_enabled"), std::string::npos);
  EXPECT_NE(camera_source.find("camera_info_block.auto_spp_enabled"), std::string::npos);
  EXPECT_NE(camera_source.find("glm::max(camera_settings.auto_spp_min_samples, 1)"), std::string::npos);

  EXPECT_NE(ray_tracing_pass_source.find("convergence_image"), std::string::npos);
  EXPECT_NE(ray_tracing_pass_source.find("convergence_view"), std::string::npos);
  EXPECT_NE(ray_tracing_pass_source.find("UpdateImageDescriptorBinding(3"), std::string::npos);
  EXPECT_NE(render_layer_source.find("ray_tracing_camera_output_layout_->PushDescriptorBinding(3"), std::string::npos);

  EXPECT_NE(application_source.find("auto_spp_min_samples"), std::string::npos);
  EXPECT_NE(application_source.find("auto_spp_max_samples"), std::string::npos);
  EXPECT_NE(application_source.find("auto_spp_convergence_threshold"), std::string::npos);
  EXPECT_NE(editor_layer_source.find("auto_spp_min_samples"), std::string::npos);
  EXPECT_NE(editor_layer_source.find("auto_spp_max_samples"), std::string::npos);
  EXPECT_NE(editor_layer_source.find("auto_spp_convergence_threshold"), std::string::npos);
  EXPECT_NE(inspection_source.find("Auto SPP"), std::string::npos);
  EXPECT_NE(editor_source.find("--preview-auto-spp"), std::string::npos);
  EXPECT_NE(editor_source.find("--preview-auto-spp-min-samples"), std::string::npos);
  EXPECT_NE(editor_source.find("--preview-auto-spp-max-samples"), std::string::npos);
  EXPECT_NE(editor_source.find("--preview-auto-spp-threshold"), std::string::npos);
  EXPECT_NE(python_binding.find("auto_spp_enabled"), std::string::npos);

  EXPECT_NE(ray_query.find("camera.auto_spp_enabled != 0u"), std::string::npos);
  EXPECT_NE(ray_query.find("convergence_history_image"), std::string::npos);
}

TEST(GltfRayTracingMaterial, CameraRaygenUsesConfigurableFireflyClamp) {
  const auto raygen = ReadRayTracingCameraSource();
  const auto ray_query = ReadRayQueryCameraSource();
  const auto cameras_include = ReadTextFile(ShaderPath("Includes/Cameras.glsl"));
  const auto camera_header = ReadTextFile(SdkPath("include/Rendering/Camera.hpp"));
  const auto camera_settings = ReadTextFile(SdkPath("include/Rendering/CameraSettings.hpp"));
  const auto camera_source = ReadTextFile(SdkPath("src/Camera.cpp"));
  const auto application_source = ReadTextFile(SdkPath("src/Application.cpp"));
  const auto editor_layer_source = ReadTextFile(SdkPath("src/EditorLayer.cpp"));
  const auto inspection_source = ReadTextFile(SdkPath("src/Editor/SDKInspectionAdapters.cpp"));
  const auto editor_source = ReadTextFile(AppPath("src/EvoEngineEditor.cpp"));

  ASSERT_FALSE(raygen.empty());
  ASSERT_FALSE(ray_query.empty());
  ASSERT_FALSE(cameras_include.empty());
  ASSERT_FALSE(camera_header.empty());
  ASSERT_FALSE(camera_settings.empty());
  ASSERT_FALSE(camera_source.empty());
  ASSERT_FALSE(application_source.empty());
  ASSERT_FALSE(editor_layer_source.empty());
  ASSERT_FALSE(inspection_source.empty());
  ASSERT_FALSE(editor_source.empty());

  EXPECT_EQ(raygen.find("EE_CAMERA_FIREFLY_CLAMP_THRESHOLD"), std::string::npos);
  EXPECT_NE(raygen.find("bool EE_CAMERA_HAS_NONFINITE_RADIANCE"), std::string::npos);
  EXPECT_NE(raygen.find("any(isnan(radiance)) || any(isinf(radiance))"), std::string::npos);
  EXPECT_NE(raygen.find("vec3 EE_CAMERA_REJECT_INVALID_RADIANCE"), std::string::npos);
  EXPECT_NE(raygen.find("invalid_rejections += 1u"), std::string::npos);
  EXPECT_NE(raygen.find("vec3 EE_CAMERA_APPLY_FIREFLY_CLAMP"), std::string::npos);
  EXPECT_NE(raygen.find("camera.firefly_clamp_enabled == 0u"), std::string::npos);
  EXPECT_NE(raygen.find("camera.firefly_clamp_threshold"), std::string::npos);
  EXPECT_NE(raygen.find("firefly_clamps += 1u"), std::string::npos);
  EXPECT_NE(raygen.find("sample_radiance = EE_CAMERA_APPLY_FIREFLY_CLAMP(camera"), std::string::npos);
  EXPECT_NE(raygen.find("EE_CAMERA_PACK_SAMPLE_DIAGNOSTICS"), std::string::npos);
  EXPECT_NE(raygen.find("EE_CAMERA_PACK_SAMPLE_DIAGNOSTICS(invalid_radiance_rejections, firefly_clamp_count)"),
            std::string::npos);
  EXPECT_NE(raygen.find("imageStore(result_image, ivec2(pixel_coordinate), vec4(linear_radiance, 1.0f)"),
            std::string::npos);
  EXPECT_NE(raygen.find("imageStore(radiance_history_image"), std::string::npos);

  EXPECT_NE(cameras_include.find("uint firefly_clamp_enabled"), std::string::npos);
  EXPECT_NE(cameras_include.find("float firefly_clamp_threshold"), std::string::npos);
  EXPECT_NE(cameras_include.find("uint auto_spp_enabled"), std::string::npos);
  EXPECT_NE(camera_header.find("uint32_t firefly_clamp_enabled = 1"), std::string::npos);
  EXPECT_NE(camera_header.find("float firefly_clamp_threshold = 10.0f"), std::string::npos);
  EXPECT_NE(camera_header.find("uint32_t auto_spp_enabled = 0"), std::string::npos);
  EXPECT_NE(camera_settings.find("bool firefly_clamp_enabled = true"), std::string::npos);
  EXPECT_NE(camera_settings.find("float firefly_clamp_threshold = 10.0f"), std::string::npos);
  EXPECT_NE(camera_source.find("firefly_clamp_enabled != other.firefly_clamp_enabled"), std::string::npos);
  EXPECT_NE(camera_source.find("firefly_clamp_threshold != other.firefly_clamp_threshold"), std::string::npos);
  EXPECT_NE(camera_source.find("camera_info_block.firefly_clamp_enabled"), std::string::npos);
  EXPECT_NE(camera_source.find("camera_info_block.firefly_clamp_threshold"), std::string::npos);

  EXPECT_NE(application_source.find("firefly_clamp_enabled"), std::string::npos);
  EXPECT_NE(application_source.find("firefly_clamp_threshold"), std::string::npos);
  EXPECT_NE(editor_layer_source.find("firefly_clamp_enabled"), std::string::npos);
  EXPECT_NE(editor_layer_source.find("firefly_clamp_threshold"), std::string::npos);
  EXPECT_NE(inspection_source.find("Firefly clamp"), std::string::npos);
  EXPECT_NE(inspection_source.find("Firefly threshold"), std::string::npos);
  EXPECT_NE(editor_source.find("--preview-firefly-clamp"), std::string::npos);
  EXPECT_NE(editor_source.find("--preview-firefly-clamp-threshold"), std::string::npos);
  EXPECT_NE(editor_source.find("preview_capture_firefly_clamp_enabled"), std::string::npos);
  EXPECT_NE(editor_source.find("preview_capture_firefly_clamp_threshold"), std::string::npos);

  EXPECT_EQ(ray_query.find("EE_CAMERA_FIREFLY_CLAMP_THRESHOLD"), std::string::npos);
  EXPECT_NE(ray_query.find("camera.firefly_clamp_enabled == 0u"), std::string::npos);
  EXPECT_NE(ray_query.find("camera.firefly_clamp_threshold"), std::string::npos);
}

TEST(GltfRayTracingMaterial, CameraRaygenGatesSerWithHitObjectTrace) {
  const auto raygen = ReadRayTracingCameraSource();
  const auto platform_source = ReadTextFile(SdkPath("src/Platform.cpp"));
  const auto render_storage_header =
      ReadTextFile(SdkPath("include/Rendering/RenderInstances/RenderInstanceStorage.hpp"));
  const auto ray_tracing_pass_source = ReadTextFile(SdkPath("src/RenderPasses/RayTracingCameraPass.cpp"));
  const auto editor_source = ReadTextFile(AppPath("src/EvoEngineEditor.cpp"));

  ASSERT_FALSE(raygen.empty());
  ASSERT_FALSE(platform_source.empty());
  ASSERT_FALSE(render_storage_header.empty());
  ASSERT_FALSE(ray_tracing_pass_source.empty());
  ASSERT_FALSE(editor_source.empty());

  EXPECT_NE(platform_source.find("EE_SHADER_EXECUTION_REORDERING_SUPPORTED"), std::string::npos);
  EXPECT_NE(render_storage_header.find("uint32_t shader_execution_reordering"), std::string::npos);
  EXPECT_NE(ray_tracing_pass_source.find("Camera::ResolveShaderExecutionReorderingEnabled"), std::string::npos);
  EXPECT_NE(ray_tracing_pass_source.find("push_constant.shader_execution_reordering"), std::string::npos);
  EXPECT_NE(editor_source.find("--preview-ser"), std::string::npos);
  EXPECT_NE(editor_source.find("ParsePreviewShaderExecutionReorderingMode"), std::string::npos);
  EXPECT_NE(editor_source.find("preview_capture_ser_mode"), std::string::npos);
  EXPECT_NE(editor_source.find("scene_camera->camera_settings.shader_execution_reordering_mode"), std::string::npos);
  EXPECT_NE(editor_source.find("frames_per_second="), std::string::npos);

  EXPECT_NE(raygen.find("#if EE_SHADER_EXECUTION_REORDERING_SUPPORTED"), std::string::npos);
  EXPECT_NE(raygen.find("#extension GL_NV_shader_invocation_reorder : require"), std::string::npos);
  EXPECT_NE(raygen.find("uint EE_SHADER_EXECUTION_REORDERING"), std::string::npos);
  EXPECT_NE(raygen.find("void EE_CAMERA_TRACE_SURFACE"), std::string::npos);
  EXPECT_NE(raygen.find("if (EE_SHADER_EXECUTION_REORDERING != 0u)"), std::string::npos);
  EXPECT_NE(raygen.find("hitObjectTraceRayNV(hit_object, EE_TLAS, gl_RayFlagsCullBackFacingTrianglesEXT"),
            std::string::npos);
  EXPECT_NE(raygen.find("reorderThreadNV(hit_object)"), std::string::npos);
  EXPECT_NE(raygen.find("hitObjectExecuteShaderNV(hit_object, 0)"), std::string::npos);
  EXPECT_NE(raygen.find("traceRayEXT(EE_TLAS, gl_RayFlagsCullBackFacingTrianglesEXT"), std::string::npos);
  EXPECT_NE(raygen.find("EE_CAMERA_TRACE_SURFACE(ray_origin, ray_direction"), std::string::npos);
  EXPECT_EQ(raygen.find("reorderThreadNV(0u)"), std::string::npos);
}

TEST(GltfRayTracingMaterial, BistroParityCaptureDisablesUnrelatedStateAndLogsCounts) {
  const auto demo_scene_header = ReadTextFile(AppPath("include/DemoScene.hpp"));
  const auto demo_scene_source = ReadTextFile(AppPath("src/DemoScene.cpp"));
  const auto editor_source = ReadTextFile(AppPath("src/EvoEngineEditor.cpp"));

  ASSERT_FALSE(demo_scene_header.empty());
  ASSERT_FALSE(demo_scene_source.empty());
  ASSERT_FALSE(editor_source.empty());

  EXPECT_NE(demo_scene_header.find("ConfigureBistroParityCapture"), std::string::npos);
  EXPECT_NE(demo_scene_header.find("LogBistroParityCaptureState"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("ApplyBistroParityRendererState"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("scene->environment.ddgi_settings.runtime.enabled = false"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("scene->environment.ddgi_settings.debug.enabled = false"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("scene->environment.volumetric_cloud_settings.enabled = false"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("kBistroReferencePathTraceMaxDepth = 5"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("kBistroDirectionalLightIntensity = 10.0f"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("ApplyBistroDirectionalLightIntensity(scene)"), std::string::npos);
  EXPECT_EQ(demo_scene_source.find("kBistroReferenceSunStrength"), std::string::npos);
  EXPECT_EQ(demo_scene_source.find("ApplyBistroReferenceSunLightPolicy"), std::string::npos);
  EXPECT_EQ(demo_scene_source.find("Bistro reference Sun light policy"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("camera->camera_settings.bounce = kBistroReferencePathTraceMaxDepth"),
            std::string::npos);
  EXPECT_NE(demo_scene_source.find("main_camera->camera_settings.bounce = kBistroReferencePathTraceMaxDepth"),
            std::string::npos);
  EXPECT_NE(demo_scene_source.find("scene_camera->camera_settings.bounce = kBistroReferencePathTraceMaxDepth"),
            std::string::npos);
  EXPECT_NE(demo_scene_source.find("scene->environment.environment_type = Scene::EnvironmentType::Color"),
            std::string::npos);
  EXPECT_NE(demo_scene_source.find("scene->environment.background_intensity = 0.0f"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("scene->environment.ambient_light_intensity = 0.0f"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("ConfigureBistroReferenceToneMapping(scene_camera)"), std::string::npos);
  EXPECT_NE(editor_source.find("enable_ambient_occlusion = false"), std::string::npos);
  EXPECT_NE(editor_source.find("enable_bloom = false"), std::string::npos);
  EXPECT_NE(editor_source.find("enable_anti_aliasing = false"), std::string::npos);
  EXPECT_NE(editor_source.find("enable_tone_mapping = false"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("enable_screen_space_reflection = false"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("Bistro parity scene:"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("mesh_primitives="), std::string::npos);
  EXPECT_NE(demo_scene_source.find("triangle_count="), std::string::npos);
  EXPECT_NE(demo_scene_source.find("material_count="), std::string::npos);
  EXPECT_NE(demo_scene_source.find("texture_count="), std::string::npos);
  EXPECT_NE(demo_scene_source.find("light_count="), std::string::npos);
  EXPECT_NE(demo_scene_source.find("camera_fov="), std::string::npos);
  EXPECT_NE(demo_scene_source.find("camera_position="), std::string::npos);
  EXPECT_NE(demo_scene_source.find("camera_rotation="), std::string::npos);
  EXPECT_NE(demo_scene_source.find("environment_type="), std::string::npos);
  EXPECT_NE(editor_source.find("ConfigureBistroParityCapture(active_scene, scene_camera)"), std::string::npos);
  EXPECT_NE(editor_source.find("LogBistroParityCaptureState(active_scene, scene_camera"), std::string::npos);
}

TEST(GltfRayTracingMaterial, RenderingRegressionProfileIsWiredAndContainsRepresentativeProbes) {
  const auto demo_scene_header = ReadTextFile(AppPath("include/DemoScene.hpp"));
  const auto demo_scene_source = ReadTextFile(AppPath("src/DemoScene.cpp"));
  const auto demo_profiles_header = ReadTextFile(AppPath("include/DemoProfiles.hpp"));
  const auto demo_profiles_source = ReadTextFile(AppPath("src/DemoProfiles.cpp"));
  const auto editor_source = ReadTextFile(AppPath("src/EvoEngineEditor.cpp"));

  EXPECT_NE(demo_profiles_header.find("RenderingRegression"), std::string::npos);
  EXPECT_NE(demo_profiles_source.find("\"rendering-regression\""), std::string::npos);
  EXPECT_NE(demo_profiles_source.find("RenderingRegression.eveproj"), std::string::npos);
  EXPECT_NE(demo_scene_header.find("ConfigureRenderingRegressionDemoScene"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("PrepareRenderingRegressionGeneratedAssets"), std::string::npos);
  for (const auto* probe : {"Material Probe Dielectric", "Emissive NEE Constant Emitter", "Punctual Light Probe Point",
                            "Skinned Capoeira Probe", "Firefly Clamp Probe", "Auto SPP Convergence Probe"}) {
    EXPECT_NE(demo_scene_source.find(probe), std::string::npos) << probe;
  }
  EXPECT_NE(demo_scene_source.find("&GltfShadeMaterial::emissive_texture"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("GltfTextureColorSpace::Srgb"), std::string::npos);
  EXPECT_NE(editor_source.find("SetupDemoScene(DemoSetup::RenderingRegression"), std::string::npos);
  EXPECT_NE(editor_source.find("ConfigureRenderingRegressionDemoScene(ApplicationContext::Get().GetActiveScene())"),
            std::string::npos);
}

TEST(GltfRayTracingMaterial, MaterialAbiKeepsAdvancedExtensionTextureSlots) {
  const auto source = ReadTextFile(ShaderPath("Includes/GltfMaterial.glsl"));
  ASSERT_FALSE(source.empty());

  EXPECT_NE(source.find("uint16_t transmission_texture"), std::string::npos);
  EXPECT_NE(source.find("uint16_t clearcoat_texture"), std::string::npos);
  EXPECT_NE(source.find("uint16_t clearcoat_roughness_texture"), std::string::npos);
  EXPECT_NE(source.find("uint16_t clearcoat_normal_texture"), std::string::npos);
  EXPECT_NE(source.find("float clearcoat_normal_texture_scale"), std::string::npos);
  EXPECT_NE(source.find("uint16_t sheen_color_texture"), std::string::npos);
  EXPECT_NE(source.find("uint16_t sheen_roughness_texture"), std::string::npos);
  EXPECT_NE(source.find("uint16_t diffuse_transmission_texture"), std::string::npos);
  EXPECT_NE(source.find("uint16_t diffuse_transmission_color_texture"), std::string::npos);
  EXPECT_NE(source.find("uint16_t iridescence_texture"), std::string::npos);
  EXPECT_NE(source.find("uint16_t iridescence_thickness_texture"), std::string::npos);
  EXPECT_NE(source.find("uint16_t anisotropy_texture"), std::string::npos);
  EXPECT_NE(source.find("uint16_t retroreflection_texture"), std::string::npos);
  EXPECT_NE(source.find("float retroreflection_factor"), std::string::npos);
  EXPECT_NE(source.find("vec3 multiscatter_color_factor"), std::string::npos);
  EXPECT_NE(source.find("float scatter_anisotropy"), std::string::npos);
}

TEST(GltfRayTracingMaterial, EmissiveTriangleDistributionUsesQuantizedCdfIntervals) {
  using Candidate = evo_engine::RenderInstanceStorage::EmissiveTriangleCandidate;
  using Record = evo_engine::RenderInstanceStorage::EmissiveTriangleInfoBlock;
  static_assert(sizeof(Record) == 16);
  static_assert(offsetof(Record, instance_index) == 0);
  static_assert(offsetof(Record, primitive_id) == 4);
  static_assert(offsetof(Record, cdf) == 8);
  static_assert(offsetof(Record, area_pdf) == 12);

  const auto records = evo_engine::RenderInstanceStorage::BuildEmissiveTriangleInfoBlocks(
      {{2u, 5u, 2.0, 1.0},
       {1u, 3u, 1.0, 2.0},
       {0u, 0u, 0.0, 1.0},
       {4u, 0u, 1.0, std::numeric_limits<double>::quiet_NaN()}});
  ASSERT_EQ(records.size(), 2u);
  EXPECT_EQ(records[0].instance_index, 1u);
  EXPECT_EQ(records[0].primitive_id, 3u);
  EXPECT_FLOAT_EQ(records[0].cdf, 0.5f);
  EXPECT_FLOAT_EQ(records[0].area_pdf, 0.5f);
  EXPECT_EQ(records[1].instance_index, 2u);
  EXPECT_EQ(records[1].primitive_id, 5u);
  EXPECT_FLOAT_EQ(records[1].cdf, 1.0f);
  EXPECT_FLOAT_EQ(records[1].area_pdf, 0.25f);

  float previous_cdf = 0.0f;
  const float sorted_areas[] = {1.0f, 2.0f};
  for (size_t i = 0; i < records.size(); ++i) {
    EXPECT_FLOAT_EQ(records[i].area_pdf * sorted_areas[i], records[i].cdf - previous_cdf);
    previous_cdf = records[i].cdf;
  }
}

TEST(GltfRayTracingMaterial, EmissiveTriangleSolidAnglePdfAndBalanceWeightsMatch) {
  constexpr float selection_pdf = 0.25f;
  constexpr float area = 2.0f;
  constexpr float distance_squared = 4.0f;
  constexpr float light_cosine = 0.5f;
  constexpr float bsdf_pdf = 0.25f;
  const float solid_angle_pdf = selection_pdf / area * distance_squared / light_cosine;
  EXPECT_FLOAT_EQ(solid_angle_pdf, 1.0f);
  EXPECT_FLOAT_EQ(solid_angle_pdf / (solid_angle_pdf + bsdf_pdf), 0.8f);
  EXPECT_FLOAT_EQ(bsdf_pdf / (solid_angle_pdf + bsdf_pdf), 0.2f);
}

TEST(GltfRayTracingMaterial, StaticEmissiveTrianglesAreSharedByRayTracingAndRayQuery) {
  const auto basic = ReadTextFile(ShaderPath("Includes/RayTracingBasic.glsl"));
  const auto integrator = ReadTextFile(ShaderPath("Includes/CameraRayIntegrator.glsl"));
  const auto raster_material = ReadTextFile(ShaderPath("Includes/GltfRasterMaterial.glsl"));
  const auto render_storage = ReadTextFile(SdkPath("src/RenderInstanceStorage.cpp"));
  const auto render_storage_header =
      ReadTextFile(SdkPath("include/Rendering/RenderInstances/RenderInstanceStorage.hpp"));
  const auto geometry_header = ReadTextFile(SdkPath("include/Rendering/Geometry/GeometryStorage.hpp"));
  const auto render_layer = ReadTextFile(SdkPath("src/RenderLayer.cpp"));
  const auto editor = ReadTextFile(AppPath("src/EvoEngineEditor.cpp"));
  const auto demo_scene = ReadTextFile(AppPath("src/DemoScene.cpp"));

  EXPECT_NE(basic.find("struct EmissiveTriangleInfo"), std::string::npos);
  EXPECT_NE(basic.find("binding = 3"), std::string::npos);
  EXPECT_NE(integrator.find("EE_CAMERA_SAMPLE_EMISSIVE_TRIANGLE_RECORD"), std::string::npos);
  EXPECT_NE(integrator.find("EE_CAMERA_FIND_EMISSIVE_TRIANGLE"), std::string::npos);
  EXPECT_NE(integrator.find("record.area_pdf * distance_squared / light_cosine"), std::string::npos);
  EXPECT_NE(integrator.find("EE_CAMERA_PREPARE_EMISSIVE_LIGHTING"), std::string::npos);
  EXPECT_NE(integrator.find("EE_CAMERA_VOLUME_EMISSIVE_NEE"), std::string::npos);
  EXPECT_NE(integrator.find("EE_CAMERA_BALANCE_HEURISTIC(last_sample_pdf, emissive_pdf)"), std::string::npos);
  EXPECT_NE(integrator.find("EE_GLTF_RT_COATED_EMISSION_LOD0(uint(instance.material_index)"), std::string::npos);
  EXPECT_NE(integrator.find("EE_GLTF_RT_COATED_EMISSION(surface_hit.pbr, view_direction)"), std::string::npos);
  EXPECT_NE(raster_material.find("EE_GLTF_SAMPLE_TEXTURE_SLOT_LOD0"), std::string::npos);
  EXPECT_NE(demo_scene.find("kEmissiveTextureResolution = 32"), std::string::npos);
  EXPECT_NE(geometry_header.find("PeekTriangle"), std::string::npos);
  EXPECT_NE(render_storage.find("append_collection(deferred_render_instances)"), std::string::npos);
  EXPECT_NE(render_storage.find("append_collection(forward_render_instances)"), std::string::npos);
  EXPECT_EQ(render_storage.find("append_collection(transparent_render_instances)"), std::string::npos);
  const auto cache_gate = render_storage.find("emissive_triangle_instance_signatures_ == signatures");
  const auto triangle_walk = render_storage.find("for (const auto& emissive_instance : emissive_instances)");
  EXPECT_NE(cache_gate, std::string::npos);
  EXPECT_NE(triangle_walk, std::string::npos);
  EXPECT_LT(cache_gate, triangle_walk);
  EXPECT_NE(render_storage_header.find("double importance"), std::string::npos);
  EXPECT_NE(render_storage_header.find("emissive_triangle_info_dirty_"), std::string::npos);
  EXPECT_NE(render_storage.find("if (emissive_triangle_info_dirty_)"), std::string::npos);
  const auto clear = render_storage.find("void RenderInstanceStorage::Clear()");
  const auto upload_storage = render_storage.find("void RenderInstanceStorage::Upload()", clear);
  ASSERT_NE(clear, std::string::npos);
  ASSERT_NE(upload_storage, std::string::npos);
  EXPECT_EQ(render_storage.substr(clear, upload_storage - clear).find("emissive_triangle_info_blocks_.clear()"),
            std::string::npos);
  const auto upload = render_layer.find("current_render_instances->Upload()");
  const auto bind =
      render_layer.find("BindRenderInstanceStorage(current_frame_index, current_render_instances)", upload);
  const auto emissive_binding = render_layer.find("emissive_triangle_info_descriptor_buffer", bind);
  EXPECT_NE(upload, std::string::npos);
  EXPECT_NE(bind, std::string::npos);
  EXPECT_NE(emissive_binding, std::string::npos);
  EXPECT_NE(editor.find("--preview-emissive-nee"), std::string::npos);
}

TEST(GltfRayTracingMaterial, RayDebugViewsShareOneIntegrator) {
  const auto cameras = ReadTextFile(ShaderPath("Includes/Cameras.glsl"));
  const auto integrator = ReadTextFile(ShaderPath("Includes/CameraRayIntegrator.glsl"));
  const auto raygen = ReadTextFile(ShaderPath("RayTracing/RayGen/Camera.rgen"));
  const auto ray_query = ReadTextFile(ShaderPath("Compute/RayQueryCamera.comp"));
  const auto camera_settings = ReadTextFile(SdkPath("include/Rendering/CameraSettings.hpp"));
  const auto camera_header = ReadTextFile(SdkPath("include/Rendering/Camera.hpp"));
  const auto camera_source = ReadTextFile(SdkPath("src/Camera.cpp"));
  const auto render_layer = ReadTextFile(SdkPath("src/RenderLayer.cpp"));
  const auto variant_header = ReadTextFile(SdkPath("include/Rendering/RayCameraShaderVariantCache.hpp"));
  const auto variant_source = ReadTextFile(SdkPath("src/RayCameraShaderVariantCache.cpp"));
  const auto inspection = ReadTextFile(SdkPath("src/Editor/SDKInspectionAdapters.cpp"));
  const auto editor = ReadTextFile(AppPath("src/EvoEngineEditor.cpp"));
  const auto python_binding =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "PythonBinding/src/PyEcoSysLab.cpp");

  EXPECT_NE(cameras.find("uint ray_debug_view"), std::string::npos);
  EXPECT_NE(camera_settings.find("enum class RayDebugView"), std::string::npos);
  EXPECT_NE(camera_settings.find("RayDebugView ray_debug_view = RayDebugView::Beauty"), std::string::npos);
  EXPECT_NE(camera_header.find("uint32_t ray_debug_view = 0"), std::string::npos);
  EXPECT_NE(camera_source.find("camera_info_block.ray_debug_view"), std::string::npos);
  EXPECT_NE(inspection.find("Ray Debug View"), std::string::npos);
  EXPECT_NE(editor.find("--preview-ray-debug"), std::string::npos);
  EXPECT_NE(python_binding.find("py::enum_<CameraSettings::RayDebugView>"), std::string::npos);
  EXPECT_NE(raygen.find("#include \"CameraRayIntegrator.glsl\""), std::string::npos);
  EXPECT_NE(ray_query.find("#include \"CameraRayIntegrator.glsl\""), std::string::npos);
  EXPECT_NE(integrator.find("#define EE_CAMERA_ENABLE_DEBUG_VIEWS 1"), std::string::npos);
  EXPECT_NE(integrator.find("#if EE_CAMERA_ENABLE_DEBUG_VIEWS"), std::string::npos);
  EXPECT_NE(variant_header.find("kRayCameraDebugViewsFeature = 1u << 31u"), std::string::npos);
  EXPECT_NE(variant_source.find("need_ray_tracing_debug_views ? kRayCameraDebugViewsFeature : 0u"), std::string::npos);
  EXPECT_NE(variant_source.find("need_ray_query_debug_views ? kRayCameraDebugViewsFeature : 0u"), std::string::npos);
  EXPECT_NE(variant_source.find("? \":debug\" : \"\""), std::string::npos);
  EXPECT_NE(variant_source.find("result.requested_mask &= kGltfSceneAllFeatures"), std::string::npos);
  EXPECT_NE(variant_source.find("result.active_mask &= kGltfSceneAllFeatures"), std::string::npos);
  EXPECT_NE(variant_source.find("kRayCameraFallbackMask = kGltfSceneAllFeatures | kRayCameraDebugViewsFeature"),
            std::string::npos);
  EXPECT_NE(variant_source.find("feature_mask == kRayCameraFallbackMask"), std::string::npos);
  EXPECT_EQ(variant_source.find("feature_mask == kGltfSceneAllFeatures"), std::string::npos);
  EXPECT_NE(render_layer.find("#define EE_CAMERA_ENABLE_DEBUG_VIEWS"), std::string::npos);
  EXPECT_NE(render_layer.find("need_ray_tracing_debug_views"), std::string::npos);
  EXPECT_NE(render_layer.find("need_ray_query_debug_views"), std::string::npos);
  EXPECT_NE(render_layer.find("need_ray_tracing_debug_views || force_full_ray_camera_shader_variant"),
            std::string::npos);
  EXPECT_NE(render_layer.find("need_ray_query_debug_views || force_full_ray_camera_shader_variant"), std::string::npos);
  EXPECT_NE(integrator.find("EE_CAMERA_DEBUG_DIRECT_PUNCTUAL"), std::string::npos);
  EXPECT_NE(integrator.find("EE_CAMERA_DEBUG_DIRECT_ENVIRONMENT"), std::string::npos);
  EXPECT_NE(integrator.find("EE_CAMERA_DEBUG_DIRECT_EMISSIVE"), std::string::npos);
  EXPECT_NE(integrator.find("EE_CAMERA_DEBUG_INDIRECT_RADIANCE"), std::string::npos);
  EXPECT_NE(integrator.find("EE_CAMERA_ENCODE_PDF"), std::string::npos);
  EXPECT_NE(integrator.find("path_surface_count"), std::string::npos);
  EXPECT_NE(
      integrator.find(
          "const vec3 direct_contribution = EE_CAMERA_RESOLVE_DIRECT_LIGHTING(bounce, surface_direct_shadow_seed)"),
      std::string::npos);
}
