#include "EvoEngine_SDK_PCH.hpp"
#include "GraphicsResources.hpp"
#include "RenderInstanceStorage.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <string>
#include <utility>

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

TEST(GltfRayTracingMaterial, CameraClosestHitRecordsRaygenHitPayload) {
  const auto source = ReadTextFile(ShaderPath("RayTracing/ClosestHit/Camera.rchit"));
  ASSERT_FALSE(source.empty());

  EXPECT_NE(source.find("#include \"RayTracingBasic.glsl\""), std::string::npos);
  EXPECT_EQ(source.find("#include \"GltfRasterMaterial.glsl\""), std::string::npos);
  EXPECT_NE(source.find("const Instance instance = EE_INSTANCES[instance_index]"), std::string::npos);
  EXPECT_NE(source.find("const uint material_index = uint(instance.material_index)"), std::string::npos);
  EXPECT_NE(source.find("hit_value.hit_t = gl_HitTEXT"), std::string::npos);
  EXPECT_NE(source.find("hit_value.instance_index = uint(instance_index)"), std::string::npos);
  EXPECT_NE(source.find("hit_value.primitive_id = uint(gl_PrimitiveID)"), std::string::npos);
  EXPECT_NE(source.find("hit_value.barycentrics = attribs"), std::string::npos);
  EXPECT_NE(source.find("const bool front_face = dot(world_geometric_normal, gl_WorldRayDirectionEXT) < 0.0f"),
            std::string::npos);
  EXPECT_NE(source.find("if (!front_face)"), std::string::npos);
  EXPECT_NE(source.find("world_geometric_normal = -world_geometric_normal"), std::string::npos);
  EXPECT_NE(source.find("hit_value.geometric_normal = world_geometric_normal"), std::string::npos);
  EXPECT_NE(source.find("hit_value.material_index = material_index"), std::string::npos);
  EXPECT_NE(source.find("if (hit_value.type == EE_CAMERA_RAY_PAYLOAD_SHADOW)"), std::string::npos);
  EXPECT_NE(source.find("hit_value.shadow_transmission = vec3(0.0f)"), std::string::npos);
  EXPECT_EQ(source.find("traceRayEXT("), std::string::npos);
  EXPECT_EQ(source.find("combined_color"), std::string::npos);
  EXPECT_EQ(source.find("EE_CAMERA_DIRECT_LIGHTING"), std::string::npos);

  EXPECT_EQ(source.find(std::string("Material") + "Properties"), std::string::npos);
  EXPECT_EQ(source.find(std::string("EE_MATERIAL") + "_PROPERTIES"), std::string::npos);
  EXPECT_EQ(source.find("EE_SAMPLE_TEXTURE_2D"), std::string::npos);
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
  EXPECT_NE(any_hit.find("EE_GLTF_RASTER_OPACITY_LOD0(material_index, tex_coord_0, tex_coord_1, vertex_color.a)"),
            std::string::npos);
  EXPECT_NE(evaluator.find("float EE_GLTF_RASTER_OPACITY_LOD0"), std::string::npos);
  EXPECT_NE(evaluator.find("material.alpha_mode == EE_GLTF_ALPHA_MODE_MASK"), std::string::npos);
  EXPECT_NE(evaluator.find("base_color_alpha *= vertex_alpha"), std::string::npos);
  EXPECT_NE(evaluator.find("material.pbr_base_color_texture, EE_GLTF_RASTER_TEXTURE_BASE_COLOR"), std::string::npos);
  EXPECT_NE(any_hit.find("vertex_color.a"), std::string::npos);
  EXPECT_NE(any_hit.find("EE_RANDOM(hit_value.seed) > opacity"), std::string::npos);
  EXPECT_NE(any_hit.find("ignoreIntersectionEXT"), std::string::npos);
  EXPECT_NE(render_layer.find("ShaderType::AnyHit"), std::string::npos);
  EXPECT_NE(render_layer.find("Shaders/RayTracing/AnyHit/Camera.rahit"), std::string::npos);
  EXPECT_NE(pipeline.find("closest_hit_group_ci.closestHitShader = closest_hit_shader_index"), std::string::npos);
  EXPECT_NE(pipeline.find("closest_hit_group_ci.anyHitShader = any_hit_shader_index"), std::string::npos);
  EXPECT_NE(pipeline.find("buffer_create_info.size = handle_size_aligned_"), std::string::npos);
  EXPECT_EQ(pipeline.find("buffer_create_info.size = handle_size;"), std::string::npos);
  EXPECT_NE(any_hit.find("hit_value.type != EE_CAMERA_RAY_PAYLOAD_SHADOW"), std::string::npos);
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
  EXPECT_NE(raygen.find("traceRayEXT(EE_TLAS, 0, EE_CAMERA_RAY_MASK_SHADOW"), std::string::npos);
  EXPECT_EQ(raygen.find("gl_RayFlagsNoOpaqueEXT | gl_RayFlagsSkipClosestHitShaderEXT"), std::string::npos);
  EXPECT_NE(raygen.find("direct_light.radiance_over_pdf *= shadow_transmission"), std::string::npos);
  EXPECT_EQ(raygen.find("bool EE_CAMERA_SHADOW_VISIBLE"), std::string::npos);
  EXPECT_EQ(raygen.find("gl_RayFlagsTerminateOnFirstHitEXT | gl_RayFlagsSkipClosestHitShaderEXT"), std::string::npos);

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

  EXPECT_NE(miss.find("hit_value.type == EE_CAMERA_RAY_PAYLOAD_SHADOW"), std::string::npos);
  EXPECT_LT(miss.find("hit_value.type == EE_CAMERA_RAY_PAYLOAD_SHADOW"),
            miss.find("hit_value.environment_radiance = EE_CAMERA_ENVIRONMENT_RADIANCE(ray_direction)"));
  EXPECT_EQ(miss.find("shadow_transmission = vec3(0.0f)"), std::string::npos);
}

TEST(GltfRayTracingMaterial, RayShadersUseCanonicalMaterialBlockOnly) {
  const std::filesystem::path paths[] = {
      ShaderPath("RayTracing/RayGen/Camera.rgen"),
      ShaderPath("RayTracing/Miss/Camera.rmiss"),
      ShaderPath("RayTracing/ClosestHit/Camera.rchit"),
      ShaderPath("RayTracing/AnyHit/Camera.rahit"),
      ShaderPath("RayTracing/RayGen/CameraLegacy.rgen"),
      ShaderPath("RayTracing/Miss/CameraLegacy.rmiss"),
      ShaderPath("RayTracing/ClosestHit/CameraLegacy.rchit"),
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
  EXPECT_NE(evaluator.find("vec2 tex_gradients"), std::string::npos);
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
  EXPECT_NE(evaluator.find("fallback, vec2 tex_gradients"), std::string::npos);
  EXPECT_NE(evaluator.find("texture_info.tex_coord == 1 ? tex_gradients.y : tex_gradients.x"), std::string::npos);
  EXPECT_NE(evaluator.find("texture_info.uv_transform * vec3(tex_grad, 0.0, 0.0)"), std::string::npos);
  EXPECT_NE(evaluator.find("textureGrad(EE_TEXTURE_2DS[nonuniformEXT(texture_info.index)], uv, ddx_uv, ddy_uv)"),
            std::string::npos);
  EXPECT_NE(evaluator.find("uint material_index, vec2 tex_coord_0, vec2 tex_coord_1, vec4 vertex_color, "
                           "vec2 tex_gradients"),
            std::string::npos);

  EXPECT_NE(bsdf.find("vec3 bitangent, vec3 geometric_normal, bool is_inside, vec2 tex_grad"), std::string::npos);
  EXPECT_NE(bsdf.find("EE_EVALUATE_GLTF_RASTER_SURFACE(material_index, tex_coord_0, tex_coord_1, vertex_color, "
                      "tex_grad)"),
            std::string::npos);
  EXPECT_NE(bsdf.find("EE_GLTF_SAMPLE_TEXTURE(material.normal_texture, tex_coord_0, tex_coord_1"), std::string::npos);
  EXPECT_NE(bsdf.find("tex_grad).xyz"), std::string::npos);

  for (const auto* source : {&raygen, &ray_query}) {
    EXPECT_NE(source->find("#define EE_GLTF_USE_EXPLICIT_TEXTURE_LOD"), std::string::npos);
    EXPECT_NE(source->find("#define EE_GLTF_TEXTURE_LOD 0.0"), std::string::npos);
    EXPECT_NE(source->find("vec2 EE_CAMERA_TEXEL_DENSITY"), std::string::npos);
    EXPECT_NE(source->find("return sqrt(uv_area / max(world_area, 1e-20f))"), std::string::npos);
    EXPECT_NE(source->find("float EE_CAMERA_WORLD_FOOTPRINT"), std::string::npos);
    EXPECT_NE(source->find("camera.inverse_projection[1][1]"), std::string::npos);
    EXPECT_NE(source->find("EE_CAMERA_TEXTURE_GRADIENTS"), std::string::npos);
    EXPECT_NE(source->find("EE_EVALUATE_GLTF_RASTER_SURFACE"), std::string::npos);
    EXPECT_NE(source->find("ray_cone_width + hit_t * pixel_angle"), std::string::npos);
    EXPECT_NE(source->find("float ray_cone_width = 0.0f"), std::string::npos);
    EXPECT_NE(source->find("EE_CAMERA_RECONSTRUCT_SURFACE_HIT(is_inside, ray_direction, ray_cone_width)"),
              std::string::npos);
    EXPECT_NE(source->find("ray_cone_width =\n        EE_CAMERA_WORLD_FOOTPRINT(ray_cone_width, hit_value.hit_t"),
              std::string::npos);
  }

  EXPECT_NE(raygen.find("hit.tex_gradients = EE_CAMERA_TEXTURE_GRADIENTS(ray_cone_width"), std::string::npos);
  EXPECT_NE(raygen.find("hit.material_index, tex_coord_0, tex_coord_1, vertex_color"), std::string::npos);
  EXPECT_NE(raygen.find("is_inside, hit.tex_gradients"), std::string::npos);
  EXPECT_NE(raygen.find("v0.tex_coord_1 * barycentrics.x"), std::string::npos);
  EXPECT_NE(raygen.find("v0.color * barycentrics.x"), std::string::npos);
  EXPECT_NE(ray_query.find("out uint material_index, out vec2 tex_coord_0, out vec2 tex_coord_1"), std::string::npos);
  EXPECT_NE(ray_query.find("EE_GLTF_RASTER_OPACITY_LOD0(material_index, tex_coord_0, tex_coord_1, vertex_color.a)"),
            std::string::npos);
  EXPECT_NE(ray_query.find("EE_GLTF_RASTER_SHADOW_TRANSMISSION_LOD0("), std::string::npos);
  EXPECT_NE(ray_query.find("hit.tex_gradients = EE_CAMERA_TEXTURE_GRADIENTS(ray_cone_width"), std::string::npos);
  EXPECT_NE(ray_query.find("hit.material_index, tex_coord_0, tex_coord_1, vertex_color"), std::string::npos);
  EXPECT_NE(any_hit.find("layout(push_constant) uniform EE_CAMERA_CONSTANTS"), std::string::npos);
  EXPECT_NE(any_hit.find("EE_GLTF_RASTER_OPACITY_LOD0(material_index, tex_coord_0, tex_coord_1, vertex_color.a)"),
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

TEST(GltfRayTracingMaterial, CameraClosestHitCarriesAdvancedExtensionScalars) {
  const auto source = ReadTextFile(ShaderPath("RayTracing/ClosestHit/CameraLegacy.rchit"));
  ASSERT_FALSE(source.empty());

  EXPECT_NE(source.find("const GltfShadeMaterial material = EE_GLTF_MATERIALS[material_index]"), std::string::npos);
  EXPECT_NE(source.find("deferred_bsdf_extension_weight"), std::string::npos);
  EXPECT_NE(source.find("material.transmission_factor"), std::string::npos);
  EXPECT_NE(source.find("material.ior"), std::string::npos);
  EXPECT_NE(source.find("material.clearcoat_factor"), std::string::npos);
  EXPECT_NE(source.find("material.clearcoat_roughness"), std::string::npos);
  EXPECT_NE(source.find("material.sheen_color_factor"), std::string::npos);
  EXPECT_NE(source.find("material.sheen_roughness_factor"), std::string::npos);
}

TEST(GltfRayTracingMaterial, CameraClosestHitSamplesPunctualLightsWithShadowRays) {
  const auto source = ReadTextFile(ShaderPath("RayTracing/ClosestHit/CameraLegacy.rchit"));
  ASSERT_FALSE(source.empty());

  EXPECT_NE(source.find("EE_CAMERA_DIRECT_LIGHTING("), std::string::npos);
  EXPECT_NE(source.find("EE_CAMERA_DIRECT_LIGHT_EXPOSURE"), std::string::npos);
  EXPECT_NE(source.find("EE_CAMERA_DIRECT_DIRECTIONAL_LIGHT"), std::string::npos);
  EXPECT_NE(source.find("EE_CAMERA_DIRECT_POINT_LIGHT"), std::string::npos);
  EXPECT_NE(source.find("EE_CAMERA_DIRECT_SPOT_LIGHT"), std::string::npos);
  EXPECT_NE(source.find("EE_RENDER_INFO.directional_light_size"), std::string::npos);
  EXPECT_NE(source.find("EE_RENDER_INFO.point_light_size"), std::string::npos);
  EXPECT_NE(source.find("EE_RENDER_INFO.spot_light_size"), std::string::npos);
  EXPECT_NE(source.find("EE_CAMERA_DISTANCE_LIGHT_ATTENUATION"), std::string::npos);
  EXPECT_NE(source.find("light.constant_linear_quadratic_far"), std::string::npos);
  EXPECT_NE(source.find("light.cutoff_outer_inner_size_bias"), std::string::npos);
  EXPECT_NE(source.find("spot_intensity"), std::string::npos);
  EXPECT_NE(source.find("EE_CAMERA_SHADOW_VISIBLE"), std::string::npos);
  EXPECT_NE(source.find("EE_CAMERA_RAY_MASK_SHADOW"), std::string::npos);
  EXPECT_NE(source.find("gl_RayFlagsTerminateOnFirstHitEXT | gl_RayFlagsSkipClosestHitShaderEXT"), std::string::npos);
  EXPECT_NE(source.find("light.diffuse.w == 1.0f"), std::string::npos);
  EXPECT_NE(source.find("hit_value = primary_hit"), std::string::npos);
  EXPECT_LT(source.find("const vec3 direct_lighting"),
            source.find("combined_color + direct_lighting + surface.emissive"));
}

TEST(GltfRayTracingMaterial, CameraMissDistinguishesShadowVisibilityRays) {
  const auto source = ReadTextFile(ShaderPath("RayTracing/Miss/Camera.rmiss"));
  ASSERT_FALSE(source.empty());

  EXPECT_NE(source.find("EE_CAMERA_RAY_PAYLOAD_SHADOW"), std::string::npos);
  EXPECT_NE(source.find("hit_value.type == EE_CAMERA_RAY_PAYLOAD_SHADOW"), std::string::npos);
  EXPECT_NE(source.find("hit_value.hit_count = 0u;"), std::string::npos);
  EXPECT_LT(source.find("hit_value.type == EE_CAMERA_RAY_PAYLOAD_SHADOW"),
            source.find("hit_value.environment_radiance = EE_CAMERA_ENVIRONMENT_RADIANCE(ray_direction)"));
}

TEST(GltfRayTracingMaterial, CameraRayTracingPathCarriesMisPdfs) {
  const auto payload = ReadTextFile(ShaderPath("Includes/CameraRayTracingPayload.glsl"));
  ASSERT_FALSE(payload.empty());
  EXPECT_NE(payload.find("float last_sample_pdf"), std::string::npos);

  const auto raygen = ReadTextFile(ShaderPath("RayTracing/RayGen/CameraLegacy.rgen"));
  ASSERT_FALSE(raygen.empty());
  EXPECT_NE(raygen.find("hit_value.last_sample_pdf = 0.0f"), std::string::npos);

  const auto closest_hit = ReadTextFile(ShaderPath("RayTracing/ClosestHit/CameraLegacy.rchit"));
  ASSERT_FALSE(closest_hit.empty());
  EXPECT_NE(closest_hit.find("EE_CAMERA_BALANCE_HEURISTIC"), std::string::npos);
  EXPECT_NE(closest_hit.find("EE_CAMERA_BSDF_PDF"), std::string::npos);
  EXPECT_NE(closest_hit.find("EE_CAMERA_LIGHT_BSDF_MIS_WEIGHT"), std::string::npos);
  EXPECT_NE(closest_hit.find("EE_CAMERA_DIRECT_ENVIRONMENT_LIGHT"), std::string::npos);
  EXPECT_NE(closest_hit.find("EE_CAMERA_ENVIRONMENT_PDF"), std::string::npos);
  EXPECT_NE(closest_hit.find("EE_CAMERA_DIRAC_PDF"), std::string::npos);
  EXPECT_NE(closest_hit.find("EE_CAMERA_SANITIZE_RADIANCE"), std::string::npos);
  EXPECT_NE(closest_hit.find("hit_value.last_sample_pdf = bsdf_sample.pdf"), std::string::npos);
  EXPECT_LT(closest_hit.find("hit_value.last_sample_pdf = bsdf_sample.pdf"),
            closest_hit.find("traceRayEXT(EE_TLAS, gl_RayFlagsOpaqueEXT, 0xff"));
}

TEST(GltfRayTracingMaterial, CameraMissWeightsBsdfSampledEnvironmentHits) {
  const auto source = ReadTextFile(ShaderPath("RayTracing/Miss/CameraLegacy.rmiss"));
  ASSERT_FALSE(source.empty());

  EXPECT_NE(source.find("EE_CAMERA_ENVIRONMENT_HIT_MIS_WEIGHT"), std::string::npos);
  EXPECT_NE(source.find("EE_CAMERA_ENVIRONMENT_PDF(hit_value.normal, ray_direction)"), std::string::npos);
  EXPECT_NE(source.find("hit_value.last_sample_pdf"), std::string::npos);
  EXPECT_NE(source.find("safe_bsdf_pdf + safe_environment_pdf"), std::string::npos);
  EXPECT_NE(source.find("pdf_sum > EE_CAMERA_PDF_EPSILON"), std::string::npos);
  EXPECT_NE(source.find("EE_FUNC_ENV(ray_direction) * mis_weight"), std::string::npos);
}

TEST(GltfRayTracingMaterial, CameraRaygenOwnsPathTracingLoop) {
  const auto source = ReadRayTracingCameraSource();
  const auto ray_query = ReadRayQueryCameraSource();
  ASSERT_FALSE(source.empty());
  ASSERT_FALSE(ray_query.empty());

  EXPECT_NE(source.find("#include \"GltfRasterMaterial.glsl\""), std::string::npos);
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
  EXPECT_NE(source.find("hit.normal, hit.tangent, hit.bitangent"), std::string::npos);
  EXPECT_NE(source.find("hit.normal = hit.pbr.normal"), std::string::npos);
  EXPECT_NE(source.find("EE_CAMERA_DIRECT_LIGHTING"), std::string::npos);
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
  EXPECT_NE(source.find("EE_CAMERA_BACKGROUND_LIGHT_RADIANCE"), std::string::npos);
  EXPECT_NE(source.find("camera.skybox_tex_index"), std::string::npos);
  EXPECT_EQ(source.find("radiance += throughput * EE_CAMERA_PRIMARY_ENVIRONMENT_LIGHTING"), std::string::npos);
  EXPECT_EQ(source.find("radiance += throughput * EE_CAMERA_DIRECT_LIGHTING"), std::string::npos);
  EXPECT_NE(source.find("EE_CAMERA_PREPARE_DIRECT_LIGHTING(surface_hit, view_direction, throughput, seed, bounce)"),
            std::string::npos);
  EXPECT_NE(source.find("GltfRayTracingBsdfSampleData sample_data"), std::string::npos);
  EXPECT_NE(source.find("sample_data.k1 = view_direction"), std::string::npos);
  EXPECT_NE(source.find("sample_data.xi = vec3(EE_RANDOM(seed), EE_RANDOM(seed), EE_RANDOM(seed))"), std::string::npos);
  EXPECT_NE(source.find("EE_GLTF_RT_BSDF_SAMPLE(sample_data, surface_hit.pbr)"), std::string::npos);
  EXPECT_NE(source.find("radiance += EE_CAMERA_RESOLVE_DIRECT_LIGHTING(bounce, seed)"), std::string::npos);
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
  EXPECT_LT(source.find("EE_CAMERA_PREPARE_DIRECT_LIGHTING(surface_hit, view_direction, throughput, seed, bounce)"),
            source.find("EE_GLTF_RT_BSDF_SAMPLE(sample_data, surface_hit.pbr)"));
  EXPECT_LT(source.find("EE_GLTF_RT_BSDF_SAMPLE(sample_data, surface_hit.pbr)"),
            source.find("radiance += EE_CAMERA_RESOLVE_DIRECT_LIGHTING(bounce, seed)"));
  EXPECT_LT(source.find("radiance += EE_CAMERA_RESOLVE_DIRECT_LIGHTING(bounce, seed)"),
            source.find("if (surface_depth >= EE_CAMERA_RUSSIAN_ROULETTE_MIN_DEPTH)"));
  EXPECT_LT(source.find("if (surface_depth >= EE_CAMERA_RUSSIAN_ROULETTE_MIN_DEPTH)"),
            source.find("surface_depth += 1u"));
  EXPECT_NE(source.find("primary_hit_distance = min(primary_hit_distance, sample_hit_distance)"), std::string::npos);
  EXPECT_NE(source.find("hit_value.material_index"), std::string::npos);
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
    const auto base_normal = shader_source.find("hit.normal = EE_CAMERA_SAFE_NORMALIZE(normal_matrix * object_normal");
    const auto tangent_basis =
        shader_source.find("hit.bitangent = EE_CAMERA_SAFE_NORMALIZE(cross(hit.normal, hit.tangent)");
    const auto side_check = shader_source.find("if (dot(hit.normal, hit.geometric_normal) < 0.0f)");
    const auto tangent_flip = shader_source.find("hit.tangent = -hit.tangent", side_check);
    const auto reflection_clamp = shader_source.find("const vec3 reflected_direction = reflect");
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
    EXPECT_NE(source->find("hit.geometric_normal = EE_CAMERA_SAFE_NORMALIZE(hit_value.geometric_normal"),
              std::string::npos);
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
  ASSERT_FALSE(bsdf.empty());

  EXPECT_NE(bsdf.find("vec3 EE_GLTF_RT_SAMPLE_COSINE_HEMISPHERE(const vec2 xi)"), std::string::npos);
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
  EXPECT_NE(raygen.find("traceRayEXT(EE_TLAS, 0, EE_CAMERA_RAY_MASK_SHADOW"), std::string::npos);
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
  EXPECT_NE(raygen.find("eval_data.xi = vec3(EE_RANDOM(seed), EE_RANDOM(seed), EE_RANDOM(seed))"), std::string::npos);
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
  EXPECT_NE(bsdf.find("pbr.specular_f0 * max(pbr.specular_color"), std::string::npos);
  EXPECT_NE(bsdf.find("dielectric_ior = material.ior == 0.0f ? 0.0f"), std::string::npos);
  EXPECT_NE(bsdf.find("material.ior == 0.0f ? EE_GLTF_RT_IOR_COMPATIBILITY_INFINITY"), std::string::npos);
  EXPECT_NE(bsdf.find("if (material.ior != 0.0f)"), std::string::npos);
  EXPECT_EQ(bsdf.find("if (material.specular_factor > 0.0f)"), std::string::npos);
  EXPECT_NE(bsdf.find("pbr.thickness = material.thickness_factor"), std::string::npos);
  EXPECT_EQ(bsdf.find("pbr.thickness *=\n        EE_GLTF_SAMPLE_TEXTURE(material.thickness_texture"),
            std::string::npos);

  const auto unlit_test = integrator.find("EE_GLTF_MATERIALS[surface_hit.material_index].unlit > 0");
  const auto emissive_add = integrator.find("radiance += throughput * emissive_hit_mis_weight * emissive_radiance");
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

  const auto reconstruct = raygen.find("EE_CAMERA_RECONSTRUCT_SURFACE_HIT(is_inside, ray_direction, ray_cone_width)");
  const auto process_volume = raygen.find("EE_CAMERA_PROCESS_VOLUME_SEGMENT(hit_value.hit_t", reconstruct);
  const auto direct_lighting = raygen.find(
      "EE_CAMERA_PREPARE_DIRECT_LIGHTING(surface_hit, view_direction, throughput, seed, bounce)", process_volume);
  ASSERT_NE(reconstruct, std::string::npos);
  ASSERT_NE(process_volume, std::string::npos);
  ASSERT_NE(direct_lighting, std::string::npos);
  EXPECT_LT(reconstruct, process_volume);
  EXPECT_LT(process_volume, direct_lighting);

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
      raygen.find("EE_CAMERA_PREPARE_DIRECT_LIGHTING(surface_hit, view_direction, throughput, seed, bounce)");
  const auto sample = raygen.find("EE_GLTF_RT_BSDF_SAMPLE(sample_data, surface_hit.pbr)", prepare);
  const auto absorb = raygen.find("sample_data.event_type == EE_GLTF_RT_BSDF_EVENT_ABSORB", sample);
  const auto force_depth = raygen.find("surface_depth = max_depth", absorb);
  const auto resolve = raygen.find("radiance += EE_CAMERA_RESOLVE_DIRECT_LIGHTING(bounce, seed)", sample);
  const auto terminate = raygen.find("if (terminate_path)", resolve);
  const auto roulette = raygen.find("if (surface_depth >= EE_CAMERA_RUSSIAN_ROULETTE_MIN_DEPTH)", terminate);
  ASSERT_NE(prepare, std::string::npos);
  ASSERT_NE(sample, std::string::npos);
  ASSERT_NE(absorb, std::string::npos);
  ASSERT_NE(force_depth, std::string::npos);
  ASSERT_NE(resolve, std::string::npos);
  ASSERT_NE(terminate, std::string::npos);
  ASSERT_NE(roulette, std::string::npos);
  EXPECT_LT(prepare, sample);
  EXPECT_LT(sample, resolve);
  EXPECT_LT(absorb, resolve);
  EXPECT_LT(force_depth, resolve);
  EXPECT_LT(resolve, terminate);
  EXPECT_LT(terminate, roulette);
}

TEST(GltfRayTracingMaterial, CameraRaygenBuildsReferenceStylePrimaryRays) {
  const auto source = ReadRayTracingCameraSource();
  ASSERT_FALSE(source.empty());

  EXPECT_NE(source.find("const vec2 clip_coords = (sample_position + sample_offset) / image_size * 2.0f - 1.0f"),
            std::string::npos);
  EXPECT_NE(source.find("EE_CAMERA_CREATE_PRIMARY_RAY(camera, vec2(pixel_coordinate), sample_offset"),
            std::string::npos);
  EXPECT_NE(source.find("camera.inverse_projection * vec4(clip_coords, -1.0f, 1.0f)"), std::string::npos);
  EXPECT_NE(source.find("const vec3 origin = camera.inverse_view[3].xyz"), std::string::npos);
  EXPECT_NE(source.find("camera.inverse_view * view_position"), std::string::npos);
  EXPECT_NE(source.find("EE_CAMERA_TRACE_PATH(seed, primary_ray.origin, primary_ray.direction"), std::string::npos);
  EXPECT_EQ(source.find("- vec2(0.5f)"), std::string::npos);
  EXPECT_EQ(source.find("camera.inverse_projection_view * vec4(d.x, d.y"), std::string::npos);

  const auto legacy_source = ReadTextFile(ShaderPath("RayTracing/RayGen/CameraLegacy.rgen"));
  ASSERT_FALSE(legacy_source.empty());

  EXPECT_NE(legacy_source.find("const vec2 sample_position = vec2(pixel_coordinate) + vec2(EE_RANDOM(hit_value.seed), "
                               "EE_RANDOM(hit_value.seed))"),
            std::string::npos);
  EXPECT_NE(legacy_source.find("camera.inverse_projection * vec4(clip_coords, -1.0, 1.0)"), std::string::npos);
  EXPECT_NE(legacy_source.find("vec3 sample_origin = camera.inverse_view[3].xyz"), std::string::npos);
  EXPECT_NE(legacy_source.find("camera.inverse_view * view_position"), std::string::npos);
  EXPECT_NE(legacy_source.find("traceRayEXT(EE_TLAS, gl_RayFlagsOpaqueEXT, 0xff, 0, 0, 0, sample_origin"),
            std::string::npos);
  EXPECT_EQ(legacy_source.find("- vec2(0.5"), std::string::npos);
  EXPECT_EQ(legacy_source.find("camera.inverse_projection_view * vec4(d.x, d.y"), std::string::npos);
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

TEST(GltfRayTracingMaterial, GeneratedTangentsPreserveHandedness) {
  const auto mesh_source = ReadTextFile(SdkPath("src/Mesh.cpp"));
  const auto skinned_mesh_source = ReadTextFile(SdkPath("src/SkinnedMesh.cpp"));
  ASSERT_FALSE(mesh_source.empty());
  ASSERT_FALSE(skinned_mesh_source.empty());

  for (const auto* source : {&mesh_source, &skinned_mesh_source}) {
    EXPECT_NE(source->find("float TangentHandedness(const glm::vec3& tangent, const glm::vec3& bitangent"),
              std::string::npos);
    EXPECT_NE(source->find("auto handedness_sums = std::vector<float>()"), std::string::npos);
    EXPECT_NE(source->find("const auto bitangent ="), std::string::npos);
    EXPECT_NE(source->find("d21.x * e31.x - d31.x * e21.x"), std::string::npos);
    EXPECT_EQ(source->find("d31.x * e21.x - d21.x * e31.x"), std::string::npos);
    EXPECT_NE(source->find("TangentHandedness(tangent, bitangent"), std::string::npos);
    EXPECT_NE(source->find("vertex_info3 = handedness_sums[i] < 0.0f ? -1.0f : 1.0f"), std::string::npos);
  }
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

TEST(GltfRayTracingMaterial, RayBaselineCaptureRecordsLinearHdrAndGpuMetrics) {
  const auto editor_source = ReadTextFile(AppPath("src/EvoEngineEditor.cpp"));
  const auto platform_header = ReadTextFile(SdkPath("include/Rendering/Platform/Platform.hpp"));
  const auto platform_source = ReadTextFile(SdkPath("src/Platform.cpp"));
  const auto shader_source = ReadTextFile(SdkPath("src/Shader.cpp"));
  const auto ray_camera_pass = ReadTextFile(SdkPath("src/RenderPasses/RayTracingCameraPass.cpp"));
  const auto graphics_resources = ReadTextFile(SdkPath("src/GraphicsResources.cpp"));
  const auto runner =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "Scripts/run_raytracer_baseline.py");
  const auto reference_patch = ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) /
                                            "Scripts/reference_patches/vk_gltf_renderer_m0.patch");

  ASSERT_FALSE(editor_source.empty());
  ASSERT_FALSE(platform_header.empty());
  ASSERT_FALSE(platform_source.empty());
  ASSERT_FALSE(shader_source.empty());
  ASSERT_FALSE(ray_camera_pass.empty());
  ASSERT_FALSE(graphics_resources.empty());
  ASSERT_FALSE(runner.empty());
  ASSERT_FALSE(reference_patch.empty());

  EXPECT_NE(editor_source.find("extension != \".png\" && extension != \".hdr\""), std::string::npos);
  EXPECT_NE(editor_source.find("Linear HDR preview capture requires raytracing or rayquery mode"), std::string::npos);
  EXPECT_NE(editor_source.find("post_processing_stack->enable_tone_mapping = false"), std::string::npos);
  EXPECT_NE(editor_source.find("--preview-metrics-json"), std::string::npos);
  EXPECT_NE(editor_source.find("RAY_CAPTURE_JSON "), std::string::npos);
  EXPECT_NE(editor_source.find("metrics[\"effective_spp\"]"), std::string::npos);
  EXPECT_NE(editor_source.find("metrics[\"startup_gpu_sections\"]"), std::string::npos);
  EXPECT_NE(editor_source.find("metrics[\"gpu_sections\"]"), std::string::npos);
  EXPECT_NE(editor_source.find("Platform::SetGpuTimestampCaptureEnabled(true)"), std::string::npos);
  EXPECT_NE(editor_source.find("Platform::SetGpuTimestampCaptureEnabled(false)"), std::string::npos);
  EXPECT_NE(editor_source.find("editor_layer->show_camera_window = false"), std::string::npos);
  EXPECT_NE(editor_source.find("main_camera->SetRequireRendering(false)"), std::string::npos);
  EXPECT_NE(editor_source.find("Camera::IsRayCameraRenderMode(resolved_render_mode) && !temporal_motion_capture"),
            std::string::npos);

  EXPECT_NE(platform_header.find("struct GpuTimestampStats"), std::string::npos);
  EXPECT_NE(platform_header.find("immediate_gpu_timestamp_mutex_"), std::string::npos);
  EXPECT_NE(platform_header.find("gpu_timestamp_stats_mutex_"), std::string::npos);
  EXPECT_NE(platform_source.find("vkGetQueryPoolResults"), std::string::npos);
  EXPECT_NE(platform_source.find("timestamp_lock(graphics.immediate_gpu_timestamp_mutex_)"), std::string::npos);
  EXPECT_NE(platform_source.find("stats_lock(graphics.gpu_timestamp_stats_mutex_)"), std::string::npos);
  EXPECT_NE(platform_source.find("EVOENGINE_IMGUI_INI_PATH"), std::string::npos);
  EXPECT_NE(shader_source.find("EVOENGINE_SHADER_CACHE_DIR"), std::string::npos);
  EXPECT_NE(shader_source.find("GetShaderBinaryDirectory()"), std::string::npos);
  EXPECT_NE(ray_camera_pass.find("BeginGpuTimestampScope(vk_command_buffer, \"Path Trace (RTX)\")"), std::string::npos);
  EXPECT_NE(ray_camera_pass.find("BeginGpuTimestampScope(vk_command_buffer, \"Path Trace (RQ)\")"), std::string::npos);
  EXPECT_EQ(graphics_resources.find("ImmediateSubmitWithGpuTimestamp(\"TLAS Build\""), std::string::npos);
  EXPECT_NE(graphics_resources.find("RecordCommandsMainQueue"), std::string::npos);
  EXPECT_NE(graphics_resources.find("\"TLAS Build\""), std::string::npos);
  EXPECT_NE(graphics_resources.find("\"TLAS Update\""), std::string::npos);
  EXPECT_NE(graphics_resources.find("ImmediateSubmitWithGpuTimestamp(\"BLAS Build\""), std::string::npos);
  EXPECT_NE(graphics_resources.find("BeginGpuTimestampScope(vk_command_buffer, \"BLAS Update\")"), std::string::npos);

  EXPECT_NE(runner.find("PINNED_EVOENGINE_BASE"), std::string::npos);
  EXPECT_NE(runner.find("PINNED_REFERENCE"), std::string::npos);
  EXPECT_NE(runner.find("PINNED_NVPRO_CORE2"), std::string::npos);
  EXPECT_NE(runner.find("PINNED_BISTRO_SOURCE"), std::string::npos);
  EXPECT_NE(runner.find("PINNED_BISTRO_GLTF_SHA256"), std::string::npos);
  EXPECT_NE(runner.find("EXPECTED_EVO_BISTRO_CLOSURE"), std::string::npos);
  EXPECT_NE(runner.find("EXPECTED_REFERENCE_BISTRO_CLOSURE"), std::string::npos);
  EXPECT_NE(runner.find("generate_bistro_reference_asset"), std::string::npos);
  EXPECT_NE(runner.find("json.dumps(document, indent=2"), std::string::npos);
  EXPECT_NE(runner.find("runtime_binary_manifest"), std::string::npos);
  EXPECT_NE(runner.find("executable.parent.rglob"), std::string::npos);
  EXPECT_NE(runner.find("EvoEngine_SDK\" / \"RelWithDebInfo\" / \"EvoEngine_SDK.dll"), std::string::npos);
  EXPECT_NE(runner.find("EVOENGINE_SHADER_CACHE_DIR"), std::string::npos);
  EXPECT_NE(runner.find("EVOENGINE_IMGUI_INI_PATH"), std::string::npos);
  EXPECT_NE(runner.find("env=process_environment"), std::string::npos);
  EXPECT_NE(runner.find("shutil.rmtree(runtime_root)"), std::string::npos);
  EXPECT_EQ(runner.find("ignore_errors=True"), std::string::npos);
  EXPECT_NE(runner.find("manifest.dry-run.json"), std::string::npos);
  EXPECT_NE(runner.find("Profile(1280, 720, 16)"), std::string::npos);
  EXPECT_NE(runner.find("Profile(2560, 1440, 512)"), std::string::npos);
  EXPECT_NE(runner.find("--preview-metrics-json"), std::string::npos);
  EXPECT_NE(runner.find("project_path.unlink(missing_ok=True)"), std::string::npos);
  EXPECT_NE(runner.find("--bistro-asset-mode"), std::string::npos);
  EXPECT_NE(runner.find("--hdrEnvIntensity"), std::string::npos);
  EXPECT_NE(runner.find("--solidBackgroundColor"), std::string::npos);
  EXPECT_NE(reference_patch.find("gpu_timer_name"), std::string::npos);
}

TEST(GltfRayTracingMaterial, TlasUpdateClassifierFollowsVulkanCompatibilityRules) {
  using Tlas = evo_engine::TopLevelAccelerationStructure;
  const std::vector original = {MakeTlasTestInstance()};
  EXPECT_EQ(Tlas::ClassifyUpdateMode(false, {}, original), Tlas::UpdateMode::Build);
  EXPECT_EQ(Tlas::ClassifyUpdateMode(true, original, original), Tlas::UpdateMode::NoOp);
  EXPECT_EQ(Tlas::ClassifyUpdateMode(true, original, original, {7}, {7}), Tlas::UpdateMode::NoOp);
  EXPECT_EQ(Tlas::ClassifyUpdateMode(true, original, original, {7}, {8}), Tlas::UpdateMode::Update);

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
  EXPECT_NE(graphics.find("reuse_instance_buffer_barrier.srcAccessMask = VK_ACCESS_2_SHADER_READ_BIT"),
            std::string::npos);
  EXPECT_NE(graphics.find("reuse_instance_buffer_barrier.dstAccessMask = VK_ACCESS_2_TRANSFER_WRITE_BIT"),
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
  const auto upload_completion = texture_storage.find("gpu_upload_pending_last_sync_ && !upload_pending");
  ASSERT_NE(upload_completion, std::string::npos);
  EXPECT_NE(texture_storage.find("storage.version_++", upload_completion), std::string::npos);
}

TEST(GltfRayTracingMaterial, CameraRaygenWritesLinearRadianceForPostTonemapping) {
  const std::pair<std::string, std::string> sources[] = {
      {"Camera.rgen", ReadRayTracingCameraSource()},
      {"CameraLegacy.rgen", ReadTextFile(ShaderPath("RayTracing/RayGen/CameraLegacy.rgen"))},
  };

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
  EXPECT_NE(render_layer.find("PostProcessingPass::Execute(context, {camera, nullptr, false, true})"),
            std::string::npos);

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

TEST(GltfRayTracingMaterial, CameraMissRecordsEnvironmentForRaygenLoop) {
  const auto source = ReadTextFile(ShaderPath("RayTracing/Miss/Camera.rmiss"));
  ASSERT_FALSE(source.empty());

  EXPECT_NE(source.find("EE_CAMERA_RAY_PAYLOAD_MISS"), std::string::npos);
  EXPECT_NE(source.find("hit_value.environment_radiance = EE_CAMERA_ENVIRONMENT_RADIANCE(ray_direction)"),
            std::string::npos);
  EXPECT_NE(source.find("hit_value.environment_pdf = EE_CAMERA_ENVIRONMENT_PDF()"), std::string::npos);
  EXPECT_NE(source.find("hit_value.hit_t = EE_CAMERA_FAR(int(EE_CAMERA_INDEX))"), std::string::npos);
  EXPECT_NE(source.find("vec3 EE_CAMERA_SKY_RADIANCE"), std::string::npos);
  EXPECT_NE(source.find("vec3 EE_CAMERA_ENVIRONMENT_RADIANCE"), std::string::npos);
  EXPECT_EQ(source.find(std::string("#include \"Physical") + "Sky.glsl\""), std::string::npos);
  EXPECT_EQ(source.find(std::string("EE_CAMERA_ENVIRONMENT_TYPE_PHYSICAL") + "_SKY"), std::string::npos);
  EXPECT_EQ(source.find(std::string("EE_") + "PHYSICAL_SKY_EVALUATE"), std::string::npos);
  EXPECT_EQ(source.find(std::string("EE_") + "PHYSICAL_SKY_PDF"), std::string::npos);
  EXPECT_NE(source.find("EE_CAMERA_SAMPLE_CUBEMAP_RADIANCE(camera.skybox_tex_index"), std::string::npos);
  EXPECT_NE(source.find("camera.skybox_tex_index"), std::string::npos);
  EXPECT_NE(source.find("camera.clear_color.w"), std::string::npos);
  EXPECT_NE(source.find("hit_value.color = EE_CAMERA_SKY_RADIANCE(ray_direction)"), std::string::npos);
  EXPECT_LT(source.find("hit_value.environment_radiance = EE_CAMERA_ENVIRONMENT_RADIANCE(ray_direction)"),
            source.find("hit_value.color = EE_CAMERA_SKY_RADIANCE(ray_direction)"));
}

TEST(GltfRayTracingMaterial, CameraRaygenUsesSkyColorForPrimaryMisses) {
  const auto source = ReadRayTracingCameraSource();
  ASSERT_FALSE(source.empty());

  EXPECT_NE(
      source.find(
          "if (surface_depth == 0u) {\n        radiance += throughput * hit_value.color;\n        break;\n      }"),
      std::string::npos);
  EXPECT_LT(source.find("radiance += throughput * hit_value.color"),
            source.find("EE_CAMERA_ENVIRONMENT_HIT_MIS_WEIGHT(last_sample_pdf, environment_pdf, environment_weight)"));
}

TEST(GltfRayTracingMaterial, CameraRaygenLightsFromCameraBackground) {
  const auto source = ReadRayTracingCameraSource();
  ASSERT_FALSE(source.empty());

  EXPECT_NE(source.find("vec3 EE_CAMERA_BACKGROUND_LIGHT_RADIANCE"), std::string::npos);
  EXPECT_NE(source.find("return EE_CAMERA_BACKGROUND_LIGHT_RADIANCE(ray_direction)"), std::string::npos);
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
  EXPECT_NE(source.find("max(camera.clear_color.w, 0.0f) * EE_ENVIRONMENT.light_intensity"), std::string::npos);
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

  ASSERT_FALSE(environment_include.empty());
  ASSERT_FALSE(conversion_shader.empty());
  ASSERT_FALSE(raygen.empty());
  ASSERT_FALSE(miss.empty());
  ASSERT_FALSE(environmental_map_header.empty());
  ASSERT_FALSE(environmental_map_source.empty());
  ASSERT_FALSE(cubemap_source.empty());
  ASSERT_FALSE(render_storage_header.empty());
  ASSERT_FALSE(render_storage_source.empty());

  EXPECT_NE(environment_include.find("float environment_pdf_texture_index"), std::string::npos);
  EXPECT_NE(environmental_map_header.find("AssetRef environment_pdf_texture"), std::string::npos);
  EXPECT_NE(environmental_map_source.find("BuildEnvironmentPdfTexture"), std::string::npos);
  EXPECT_NE(environmental_map_source.find("SetRgbaChannelData(cdf_pixels, resolution)"), std::string::npos);
  EXPECT_NE(cubemap_source.find("CalculateEnvironmentPdfScale"), std::string::npos);
  EXPECT_NE(conversion_shader.find("float ENVIRONMENT_PDF_SCALE"), std::string::npos);
  EXPECT_NE(conversion_shader.find("FragColor = vec4(color, pdf)"), std::string::npos);
  EXPECT_NE(render_storage_header.find("float environment_pdf_texture_index"), std::string::npos);
  EXPECT_NE(render_storage_source.find("environment_info_block.environment_pdf_texture_index = -1.0f"),
            std::string::npos);
  EXPECT_NE(render_storage_source.find("environmental_map->environment_pdf_texture.Get<Texture2D>()"),
            std::string::npos);
  EXPECT_NE(raygen.find("EE_CAMERA_SAMPLE_ENVIRONMENT_MAP"), std::string::npos);
  EXPECT_NE(raygen.find("EE_CAMERA_FIND_ENVIRONMENT_MARGINAL_ROW"), std::string::npos);
  EXPECT_NE(raygen.find("EE_CAMERA_FIND_ENVIRONMENT_CONDITIONAL_COLUMN"), std::string::npos);
  EXPECT_NE(raygen.find("EE_CAMERA_ENVIRONMENT_MAP_PDF(light_direction)"), std::string::npos);
  EXPECT_NE(miss.find("EE_CAMERA_ENVIRONMENT_MAP_PDF(normalize(gl_WorldRayDirectionEXT))"), std::string::npos);
}

TEST(GltfRayTracingMaterial, CameraRaygenUsesReferenceRngJitterAndAccumulation) {
  const auto random = ReadTextFile(ShaderPath("Includes/Random.glsl"));
  const auto raygen = ReadRayTracingCameraSource();
  const auto render_storage_header =
      ReadTextFile(SdkPath("include/Rendering/RenderInstances/RenderInstanceStorage.hpp"));
  const auto ray_tracing_pass_source = ReadTextFile(SdkPath("src/RenderPasses/RayTracingCameraPass.cpp"));
  const auto editor_source = ReadTextFile(AppPath("src/EvoEngineEditor.cpp"));

  ASSERT_FALSE(random.empty());
  ASSERT_FALSE(raygen.empty());
  ASSERT_FALSE(render_storage_header.empty());
  ASSERT_FALSE(ray_tracing_pass_source.empty());
  ASSERT_FALSE(editor_source.empty());

  EXPECT_NE(random.find("uint EE_XXHASH32"), std::string::npos);
  EXPECT_NE(random.find("uint EE_PCG"), std::string::npos);
  EXPECT_NE(random.find("float EE_REFERENCE_RANDOM"), std::string::npos);
  EXPECT_NE(raygen.find("uint seed = EE_XXHASH32(uvec3(pixel_coordinate, EE_FRAME_ID))"), std::string::npos);
  EXPECT_NE(raygen.find("EE_CAMERA_ANTIALIASING_STANDARD_DEVIATION = 0.4246609f"), std::string::npos);
  EXPECT_NE(raygen.find("vec2 EE_CAMERA_SAMPLE_GAUSSIAN"), std::string::npos);
  EXPECT_NE(raygen.find("vec2(0.5f) + EE_CAMERA_ANTIALIASING_STANDARD_DEVIATION"), std::string::npos);
  EXPECT_NE(raygen.find("sample_offset = vec2(EE_REFERENCE_RANDOM(seed), EE_REFERENCE_RANDOM(seed))"),
            std::string::npos);
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
  const auto docs = ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "docs/rendering-validation.md");

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
  ASSERT_FALSE(docs.empty());

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
  EXPECT_NE(docs.find("Auto SPP convergence mode"), std::string::npos);
  EXPECT_EQ(docs.find("RayQuery Auto SPP support is deferred"), std::string::npos);
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
  const auto docs = ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "docs/rendering-validation.md");

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
  ASSERT_FALSE(docs.empty());

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
  EXPECT_NE(docs.find("firefly luminance clamp"), std::string::npos);
  EXPECT_NE(docs.find("enabled at luminance `10.0`"), std::string::npos);
  EXPECT_NE(docs.find("NaN/Inf radiance rejection"), std::string::npos);
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
  EXPECT_NE(editor_source.find("Demo preview capture timing:"), std::string::npos);
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

TEST(GltfRayTracingMaterial, RenderingRegressionProfileCoversCrossTechniqueProbes) {
  const auto demo_scene_header = ReadTextFile(AppPath("include/DemoScene.hpp"));
  const auto demo_scene_source = ReadTextFile(AppPath("src/DemoScene.cpp"));
  const auto demo_profiles_header = ReadTextFile(AppPath("include/DemoProfiles.hpp"));
  const auto demo_profiles_source = ReadTextFile(AppPath("src/DemoProfiles.cpp"));
  const auto editor_source = ReadTextFile(AppPath("src/EvoEngineEditor.cpp"));
  const auto docs = ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "docs/rendering-validation.md");
  const auto emissive_validation =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "Scripts/validate_emissive_triangle_nee.py");

  ASSERT_FALSE(demo_scene_header.empty());
  ASSERT_FALSE(demo_scene_source.empty());
  ASSERT_FALSE(demo_profiles_header.empty());
  ASSERT_FALSE(demo_profiles_source.empty());
  ASSERT_FALSE(editor_source.empty());
  ASSERT_FALSE(docs.empty());
  ASSERT_FALSE(emissive_validation.empty());

  EXPECT_NE(demo_profiles_header.find("RenderingRegression"), std::string::npos);
  EXPECT_NE(demo_profiles_source.find("\"rendering-regression\""), std::string::npos);
  EXPECT_NE(demo_profiles_source.find("RenderingRegression.eveproj"), std::string::npos);
  EXPECT_NE(demo_scene_header.find("ConfigureRenderingRegressionDemoScene"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("M42 Rendering Regression Root"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("PrepareRenderingRegressionGeneratedAssets"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("Sponza_FBX"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("M42 Material Probe Dielectric"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("M3a UV1 Transform Vertex Color Probe"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("M3a Specular Glossiness F0 Probe"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("M3a Opaque Ignores Alpha Probe"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("M3a Mirrored Double Sided Normal Probe"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("M3a Mirrored Single Sided Visibility Probe"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("M3a Mixed Mirrored Instanced Probe"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("M3a Vertex Alpha Mask Probe"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("M3a Vertex Alpha Blend Probe"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("M3b Iridescence Colored F0 Probe"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("M3b Anisotropy Rotation 0 Probe"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("M3b Anisotropy Rotation 90 CCW Probe"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("M3b Dispersion 0 Probe"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("M3b Dispersion 1 Probe"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("M3b Retroreflection 0.5 Energy Probe"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("M3b Explicit Specular Factor 0 Probe"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("M3b Specular Factor 0.5 F90 Probe"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("M3b Unlit Ignores Emissive Probe"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("M3b Retroreflection Camera Light"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("M4 Emissive NEE Constant Emitter"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("M4 Emissive NEE Textured Emitter"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("M4 Emissive NEE Receiver Floor"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("&GltfShadeMaterial::emissive_texture"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("&GltfShadeMaterial::iridescence_texture"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("&GltfShadeMaterial::iridescence_thickness_texture"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("&GltfShadeMaterial::anisotropy_texture"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("&GltfShadeMaterial::retroreflection_texture"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("GltfTextureColorSpace::Srgb"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("mesh->SetVertices(attributes, vertices, {glm::uvec3(0, 1, 2), "
                                   "glm::uvec3(0, 2, 3)}, true)"),
            std::string::npos);
  EXPECT_NE(demo_scene_source.find("M42 Imported Material Texture Probe"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("M42 Punctual Light Probe Directional"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("M42 Punctual Light Probe Point"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("M42 Punctual Light Probe Spot"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("M42 Skinned Capoeira Probe"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("M42 Firefly Clamp Probe"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("M42 Auto SPP Convergence Probe"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("camera_settings.firefly_clamp_enabled = true"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("camera_settings.auto_spp_min_samples = 4"), std::string::npos);
  EXPECT_NE(editor_source.find("DemoProfileId::RenderingRegression"), std::string::npos);
  EXPECT_NE(editor_source.find("SetupDemoScene(DemoSetup::RenderingRegression"), std::string::npos);
  EXPECT_NE(editor_source.find("ConfigureRenderingRegressionDemoScene(ApplicationContext::Get().GetActiveScene())"),
            std::string::npos);
  EXPECT_NE(docs.find("rendering-regression"), std::string::npos);
  EXPECT_NE(docs.find("## Bistro Reference Parity"), std::string::npos);
  EXPECT_NE(docs.find("### M3b Advanced-Material Parity"), std::string::npos);
  EXPECT_NE(docs.find("### M4 Static Emissive-Triangle NEE"), std::string::npos);
  EXPECT_NE(emissive_validation.find("VARIANCE_RMS_RATIO_LIMIT = 0.75"), std::string::npos);
  EXPECT_NE(emissive_validation.find("ray_query_independent_exact"), std::string::npos);
  EXPECT_NE(docs.find("Scripts\\run_raytracer_baseline.py"), std::string::npos);
}

TEST(GltfRayTracingMaterial, CameraLegacyRecursiveFallbackRemainsAvailable) {
  const auto raygen = ReadTextFile(ShaderPath("RayTracing/RayGen/CameraLegacy.rgen"));
  const auto miss = ReadTextFile(ShaderPath("RayTracing/Miss/CameraLegacy.rmiss"));
  const auto closest_hit = ReadTextFile(ShaderPath("RayTracing/ClosestHit/CameraLegacy.rchit"));
  ASSERT_FALSE(raygen.empty());
  ASSERT_FALSE(miss.empty());
  ASSERT_FALSE(closest_hit.empty());

  EXPECT_NE(raygen.find("traceRayEXT(EE_TLAS, gl_RayFlagsOpaqueEXT, 0xff"), std::string::npos);
  EXPECT_NE(miss.find("EE_CAMERA_ENVIRONMENT_HIT_MIS_WEIGHT"), std::string::npos);
  EXPECT_NE(closest_hit.find("EE_CAMERA_DIRECT_LIGHTING"), std::string::npos);
  EXPECT_NE(closest_hit.find("hit_value.last_sample_pdf = bsdf_sample.pdf"), std::string::npos);
}

TEST(GltfRayTracingMaterial, MaterialAbiKeepsAdvancedExtensionTextureSlots) {
  const auto source = ReadTextFile(ShaderPath("Includes/GltfMaterial.glsl"));
  ASSERT_FALSE(source.empty());

  EXPECT_NE(source.find("uint16_t transmission_texture"), std::string::npos);
  EXPECT_NE(source.find("uint16_t clearcoat_texture"), std::string::npos);
  EXPECT_NE(source.find("uint16_t clearcoat_roughness_texture"), std::string::npos);
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
  const auto validator =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "Scripts/validate_emissive_triangle_nee.py");

  EXPECT_NE(basic.find("struct EmissiveTriangleInfo"), std::string::npos);
  EXPECT_NE(basic.find("binding = 3"), std::string::npos);
  EXPECT_NE(integrator.find("EE_CAMERA_SAMPLE_EMISSIVE_TRIANGLE_RECORD"), std::string::npos);
  EXPECT_NE(integrator.find("EE_CAMERA_FIND_EMISSIVE_TRIANGLE"), std::string::npos);
  EXPECT_NE(integrator.find("record.area_pdf * distance_squared / light_cosine"), std::string::npos);
  EXPECT_NE(integrator.find("EE_CAMERA_PREPARE_EMISSIVE_LIGHTING"), std::string::npos);
  EXPECT_NE(integrator.find("EE_CAMERA_VOLUME_EMISSIVE_NEE"), std::string::npos);
  EXPECT_NE(integrator.find("EE_CAMERA_BALANCE_HEURISTIC(last_sample_pdf, emissive_pdf)"), std::string::npos);
  EXPECT_NE(integrator.find("EE_CAMERA_EMISSIVE_TRIANGLE_RADIANCE(surface_material, surface_hit.tex_coord_0, "
                            "surface_hit.tex_coord_1)"),
            std::string::npos);
  EXPECT_NE(integrator.find(": surface_hit.pbr.emissive"), std::string::npos);
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
  EXPECT_NE(editor.find("metrics[\"emissive_triangle_nee_enabled\"]"), std::string::npos);
  EXPECT_NE(editor.find("metrics[\"demo_profile\"]"), std::string::npos);
  EXPECT_NE(editor.find("metrics[\"camera_position_override\"]"), std::string::npos);
  EXPECT_NE(validator.find("validate_capture_evidence"), std::string::npos);
  EXPECT_NE(validator.find("rendering-regression"), std::string::npos);
  EXPECT_NE(validator.find("camera_position_override"), std::string::npos);
  EXPECT_NE(validator.find("effective_spp"), std::string::npos);
  EXPECT_NE(validator.find("ray_tracing_pipeline"), std::string::npos);
}
