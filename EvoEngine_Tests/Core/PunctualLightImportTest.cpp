#include "EvoEngine_SDK_PCH.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>

namespace {
std::string ReadTextFile(const std::filesystem::path& path) {
  std::ifstream file(path);
  EXPECT_TRUE(file.good()) << path.string();
  return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}

std::filesystem::path SourcePath(const std::filesystem::path& relative_path) {
  return std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / relative_path;
}

std::filesystem::path ShaderPath(const std::filesystem::path& relative_path) {
  return SourcePath("EvoEngine_SDK") / "Internals" / "DefaultResources" / "Shaders" / relative_path;
}
}  // namespace

TEST(PunctualLightImport, PrefabImporterConvertsAssimpPunctualLightsToNativeComponents) {
  const auto source = ReadTextFile(SourcePath("EvoEngine_SDK/src/Prefab.cpp"));
  ASSERT_FALSE(source.empty());

  EXPECT_NE(source.find("#include \"Lights.hpp\""), std::string::npos);
  EXPECT_NE(source.find("BuildImportedLightMap"), std::string::npos);
  EXPECT_NE(source.find("AttachImportedPunctualLight"), std::string::npos);
  EXPECT_NE(source.find("ReadImportedLightRange"), std::string::npos);
  EXPECT_NE(source.find("\"PBR_LightRange\""), std::string::npos);

  EXPECT_NE(source.find("Serialization::ProduceSerializable<DirectionalLight>()"), std::string::npos);
  EXPECT_NE(source.find("Serialization::ProduceSerializable<PointLight>()"), std::string::npos);
  EXPECT_NE(source.find("Serialization::ProduceSerializable<SpotLight>()"), std::string::npos);
  EXPECT_NE(source.find("ApplyImportedLightColor(imported_light, light->diffuse, light->diffuse_brightness)"),
            std::string::npos);
  EXPECT_NE(source.find("diffuse_brightness = brightness"), std::string::npos);
  EXPECT_NE(source.find("light->constant = imported_light.mAttenuationConstant"), std::string::npos);
  EXPECT_NE(source.find("light->linear = imported_light.mAttenuationLinear"), std::string::npos);
  EXPECT_NE(source.find("light->quadratic = imported_light.mAttenuationQuadratic"), std::string::npos);
  EXPECT_NE(source.find("light->inner_degrees = glm::degrees(imported_light.mAngleInnerCone)"), std::string::npos);
  EXPECT_NE(source.find("light->outer_degrees = glm::degrees(imported_light.mAngleOuterCone)"), std::string::npos);
  EXPECT_NE(source.find("light.range = *range"), std::string::npos);

  EXPECT_NE(source.find("SafeLookAt(-direction, up)"), std::string::npos);
  EXPECT_NE(source.find("SafeLookAt(direction, up)"), std::string::npos);
  EXPECT_NE(source.find("PushImportedLightPrefab"), std::string::npos);
  EXPECT_EQ(source.find("Imported punctual light count"), std::string::npos);
  EXPECT_EQ(source.find("Imported punctual light '"), std::string::npos);
  EXPECT_EQ(source.find("ImportedPunctualLightStats"), std::string::npos);
}

TEST(PunctualLightImport, RenderingDemoDisablesImportedSponzaLights) {
  const auto source = ReadTextFile(SourcePath("EvoEngine_App/src/DemoScene.cpp"));
  ASSERT_FALSE(source.empty());

  EXPECT_NE(source.find("DisableImportedLightsRecursive(scene, sponza_entity);"), std::string::npos);
  EXPECT_NE(source.find("DisableLightIfPresent<DirectionalLight>(scene, entity);"), std::string::npos);
  EXPECT_NE(source.find("DisableLightIfPresent<PointLight>(scene, entity);"), std::string::npos);
  EXPECT_NE(source.find("DisableLightIfPresent<SpotLight>(scene, entity);"), std::string::npos);
}

TEST(PunctualLightImport, NativePointAndSpotRangeReachGpuLightBlocks) {
  const auto lights_header = ReadTextFile(SourcePath("EvoEngine_SDK/include/Rendering/PBR/Lights.hpp"));
  const auto lights_source = ReadTextFile(SourcePath("EvoEngine_SDK/src/Lights.cpp"));
  const auto render_storage = ReadTextFile(SourcePath("EvoEngine_SDK/src/RenderInstanceStorage.cpp"));
  const auto application = ReadTextFile(SourcePath("EvoEngine_SDK/src/Application.cpp"));
  const auto inspector = ReadTextFile(SourcePath("EvoEngine_SDK/src/Editor/SDKInspectionAdapters.cpp"));

  ASSERT_FALSE(lights_header.empty());
  ASSERT_FALSE(lights_source.empty());
  ASSERT_FALSE(render_storage.empty());
  ASSERT_FALSE(application.empty());
  ASSERT_FALSE(inspector.empty());

  EXPECT_NE(lights_header.find("float range;"), std::string::npos);
  EXPECT_NE(lights_source.find("void SpotLight::OnCreate()"), std::string::npos);
  EXPECT_NE(lights_source.find("void PointLight::OnCreate()"), std::string::npos);
  EXPECT_NE(lights_source.find("range = 0.0f"), std::string::npos);
  EXPECT_NE(render_storage.find("plc->range > 0.0f ? plc->range : plc->GetFarPlane()"), std::string::npos);
  EXPECT_NE(render_storage.find("slc->range > 0.0f ? slc->range : slc->GetFarPlane()"), std::string::npos);
  EXPECT_NE(application.find("YAML::Key << \"range\""), std::string::npos);
  EXPECT_NE(application.find("light.range = in[\"range\"].as<float>()"), std::string::npos);
  EXPECT_NE(inspector.find("ImGui::DragFloat(\"Range\", &light.range"), std::string::npos);
}

TEST(PunctualLightImport, RayTracingShadersSeeSharedLightSsbo) {
  const auto render_layer = ReadTextFile(SourcePath("EvoEngine_SDK/src/RenderLayer.cpp"));
  const auto ray_tracing_basic = ReadTextFile(ShaderPath("Modules/EvoEngine/RayTracingBasic.slang"));
  const auto per_frame_module = ReadTextFile(ShaderPath("Modules/EvoEngine/PerFrame.slang"));
  const auto lights = ReadTextFile(ShaderPath("Modules/EvoEngine/Lights.slang"));
  const auto ddgi_closest_hit = ReadTextFile(ShaderPath("RayTracing/ClosestHit/DDGIProbeDiagnostics.slang"));
  const auto camera_raygen = ReadTextFile(ShaderPath("RayTracing/RayGen/Camera.slang"));

  ASSERT_FALSE(render_layer.empty());
  ASSERT_FALSE(ray_tracing_basic.empty());
  ASSERT_FALSE(per_frame_module.empty());
  ASSERT_FALSE(lights.empty());
  ASSERT_FALSE(ddgi_closest_hit.empty());
  ASSERT_FALSE(camera_raygen.empty());

  EXPECT_NE(ray_tracing_basic.find("__exported import EvoEngine.PerFrame;"), std::string::npos);
  EXPECT_NE(camera_raygen.find("import EvoEngine.CameraRayIntegrator;"), std::string::npos);
  EXPECT_NE(per_frame_module.find("__exported import EvoEngine.Lights;"), std::string::npos);

  EXPECT_NE(lights.find("[[vk::binding(6, 0)]]"), std::string::npos);
  EXPECT_NE(lights.find("StructuredBuffer<DirectionalLight, Std430DataLayout> EE_DIRECTIONAL_LIGHTS"),
            std::string::npos);
  EXPECT_NE(lights.find("[[vk::binding(7, 0)]]"), std::string::npos);
  EXPECT_NE(lights.find("StructuredBuffer<PointLight, Std430DataLayout> EE_POINT_LIGHTS"), std::string::npos);
  EXPECT_NE(lights.find("[[vk::binding(8, 0)]]"), std::string::npos);
  EXPECT_NE(lights.find("StructuredBuffer<SpotLight, Std430DataLayout> EE_SPOT_LIGHTS"), std::string::npos);
  EXPECT_NE(
      render_layer.find("UpdateBufferDescriptorBinding(6, render_instances->directional_light_info_descriptor_buffer)"),
      std::string::npos);
  EXPECT_NE(render_layer.find("UpdateBufferDescriptorBinding(7, render_instances->point_light_info_descriptor_buffer)"),
            std::string::npos);
  EXPECT_NE(render_layer.find("UpdateBufferDescriptorBinding(8, render_instances->spot_light_info_descriptor_buffer)"),
            std::string::npos);

  EXPECT_NE(ddgi_closest_hit.find("EE_DIRECTIONAL_LIGHTS[i]"), std::string::npos);
  EXPECT_NE(ddgi_closest_hit.find("EE_POINT_LIGHTS[i]"), std::string::npos);
  EXPECT_NE(ddgi_closest_hit.find("EE_SPOT_LIGHTS[i]"), std::string::npos);
}

TEST(PunctualLightImport, RayTracingCameraAdaptsSharedLightsToReferenceGltfShape) {
  const auto integrator = ReadTextFile(ShaderPath("Modules/EvoEngine/CameraRayIntegrator.slang"));
  ASSERT_FALSE(integrator.empty());

  EXPECT_NE(integrator.find("struct EE_CAMERA_GLTF_LIGHT"), std::string::npos);
  EXPECT_NE(integrator.find("EE_CAMERA_DIRECTIONAL_TO_GLTF_LIGHT"), std::string::npos);
  EXPECT_NE(integrator.find("EE_CAMERA_POINT_TO_GLTF_LIGHT"), std::string::npos);
  EXPECT_NE(integrator.find("EE_CAMERA_SPOT_TO_GLTF_LIGHT"), std::string::npos);
  EXPECT_NE(integrator.find("light.diffuse.rgb"), std::string::npos);
  EXPECT_NE(integrator.find("light.diffuse.w"), std::string::npos);
  EXPECT_NE(integrator.find("light.reserved_parameters.x"), std::string::npos);
  EXPECT_NE(integrator.find("light.reserved_parameters.y"), std::string::npos);
  EXPECT_NE(integrator.find("light.cutoff_outer_inner_size_bias.z"), std::string::npos);
  EXPECT_NE(integrator.find("EE_CAMERA_REFERENCE_INV_RANGE(light.constant_linear_quadratic_far.w)"), std::string::npos);
  EXPECT_NE(integrator.find("gltf_light.attenuation = max(light.constant_linear_quadratic_far.xyz, float3(0.0f))"),
            std::string::npos);
  EXPECT_NE(integrator.find("light.use_native_attenuation > 0.5f"), std::string::npos);
  EXPECT_NE(integrator.find(
                "light.attenuation.x + light.attenuation.y * distance + light.attenuation.z * distance * distance"),
            std::string::npos);
  EXPECT_NE(integrator.find("light.cutoff_outer_inner_size_bias.x"), std::string::npos);
  EXPECT_NE(integrator.find("light.cutoff_outer_inner_size_bias.y"), std::string::npos);
  EXPECT_NE(integrator.find("EE_CAMERA_GET_GLTF_LIGHT"), std::string::npos);
  EXPECT_NE(integrator.find("EE_CAMERA_PUNCTUAL_LIGHT_COUNT"), std::string::npos);
}

TEST(PunctualLightImport, DirectionalLightIntensityKeepsSceneUnitAcrossRenderPaths) {
  const auto prefab_source = ReadTextFile(SourcePath("EvoEngine_SDK/src/Prefab.cpp"));
  const auto render_storage = ReadTextFile(SourcePath("EvoEngine_SDK/src/RenderInstanceStorage.cpp"));
  const auto raster_lighting = ReadTextFile(ShaderPath("Modules/EvoEngine/Lighting.slang"));
  const auto ddgi_closest_hit = ReadTextFile(ShaderPath("RayTracing/ClosestHit/DDGIProbeDiagnostics.slang"));
  const auto integrator = ReadTextFile(ShaderPath("Modules/EvoEngine/CameraRayIntegrator.slang"));

  ASSERT_FALSE(prefab_source.empty());
  ASSERT_FALSE(render_storage.empty());
  ASSERT_FALSE(raster_lighting.empty());
  ASSERT_FALSE(ddgi_closest_hit.empty());
  ASSERT_FALSE(integrator.empty());

  EXPECT_NE(prefab_source.find("diffuse = color_with_intensity / brightness"), std::string::npos);
  EXPECT_NE(prefab_source.find("diffuse_brightness = brightness"), std::string::npos);
  EXPECT_NE(render_storage.find("glm::vec4(dlc->diffuse * dlc->diffuse_brightness, dlc->cast_shadow)"),
            std::string::npos);

  EXPECT_NE(raster_lighting.find("DirectionalLight light = EE_DIRECTIONAL_LIGHTS[i]"), std::string::npos);
  EXPECT_NE(raster_lighting.find("float3 radiance = light.diffuse.xyz"), std::string::npos);
  EXPECT_NE(ddgi_closest_hit.find("EE_DDGI_LAMBERT_IRRADIANCE(albedo, light.diffuse.rgb"), std::string::npos);

  EXPECT_NE(
      integrator.find("const EE_CAMERA_RADIANCE_SPLIT radiance = EE_CAMERA_SPLIT_LIGHT_RADIANCE(max(light.diffuse.rgb"),
      std::string::npos);
  EXPECT_NE(integrator.find("gltf_light.intensity = radiance.intensity"), std::string::npos);
  EXPECT_NE(integrator.find("irradiance = light.intensity"), std::string::npos);
}
