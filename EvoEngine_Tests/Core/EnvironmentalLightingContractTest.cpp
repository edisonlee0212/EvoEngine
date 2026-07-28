#include "EvoEngine_SDK_PCH.hpp"

#include <gtest/gtest.h>

#include "ResolvedEnvironmentalLighting.hpp"

#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>

using namespace evo_engine;

namespace {
std::string ReadTextFile(const std::filesystem::path& path) {
  std::ifstream file(path);
  EXPECT_TRUE(file.good()) << path.string();
  return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}

std::filesystem::path SourcePath(const std::filesystem::path& relative_path) {
  return std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / relative_path;
}
}  // namespace

TEST(EnvironmentalLightingContract, DefaultRuntimeViewEncodesNoAssetFallback) {
  ResolvedEnvironmentalLighting resolved;

  EXPECT_EQ(resolved.indirect_environment_source.kind,
            ResolvedEnvironmentalLighting::IndirectEnvironmentSourceKind::EngineDefault);
  EXPECT_FLOAT_EQ(resolved.environment_lighting_intensity,
                  ResolvedEnvironmentalLighting::kDefaultEnvironmentLightingIntensity);
  EXPECT_FLOAT_EQ(resolved.diffuse_fallback_intensity, ResolvedEnvironmentalLighting::kDefaultDiffuseFallbackIntensity);
  EXPECT_FLOAT_EQ(resolved.specular_fallback_intensity,
                  ResolvedEnvironmentalLighting::kDefaultSpecularFallbackIntensity);
  EXPECT_FLOAT_EQ(ResolvedEnvironmentalLighting::kDefaultEnvironmentLightingIntensity, 1.0f);
  EXPECT_FLOAT_EQ(ResolvedEnvironmentalLighting::kDefaultDiffuseFallbackIntensity, 1.0f);
  EXPECT_FLOAT_EQ(ResolvedEnvironmentalLighting::kDefaultSpecularFallbackIntensity, 1.0f);
  EXPECT_FALSE(resolved.environmental_lighting_asset_assigned);
  EXPECT_TRUE(resolved.uses_engine_default_indirect_environment_source);
  EXPECT_TRUE(resolved.local_reflection_probes.empty());
  EXPECT_TRUE(resolved.ddgi_volumes.empty());
}

TEST(EnvironmentalLightingContract, FallbackControlsApplyOnlyToFallbackUses) {
  EXPECT_TRUE(ResolvedEnvironmentalLighting::LightingUsageUsesEnvironmentLightingIntensity(
      ResolvedEnvironmentalLighting::LightingUsage::DdgiMissRadiance));
  EXPECT_TRUE(ResolvedEnvironmentalLighting::LightingUsageUsesEnvironmentLightingIntensity(
      ResolvedEnvironmentalLighting::LightingUsage::DiffuseIblFallback));
  EXPECT_TRUE(ResolvedEnvironmentalLighting::LightingUsageUsesEnvironmentLightingIntensity(
      ResolvedEnvironmentalLighting::LightingUsage::GlobalSpecularFallback));
  EXPECT_TRUE(ResolvedEnvironmentalLighting::LightingUsageUsesEnvironmentLightingIntensity(
      ResolvedEnvironmentalLighting::LightingUsage::RayCameraEnvironmentEvent));
  EXPECT_TRUE(ResolvedEnvironmentalLighting::LightingUsageUsesEnvironmentLightingIntensity(
      ResolvedEnvironmentalLighting::LightingUsage::ReflectionProbeBakeEnvironmentInput));
  EXPECT_FALSE(ResolvedEnvironmentalLighting::LightingUsageUsesEnvironmentLightingIntensity(
      ResolvedEnvironmentalLighting::LightingUsage::ValidDdgiSurfaceIrradiance));
  EXPECT_FALSE(ResolvedEnvironmentalLighting::LightingUsageUsesEnvironmentLightingIntensity(
      ResolvedEnvironmentalLighting::LightingUsage::ValidLocalReflectionProbeSample));

  EXPECT_TRUE(ResolvedEnvironmentalLighting::LightingUsageUsesDiffuseFallbackIntensity(
      ResolvedEnvironmentalLighting::LightingUsage::DdgiMissRadiance));
  EXPECT_TRUE(ResolvedEnvironmentalLighting::LightingUsageUsesDiffuseFallbackIntensity(
      ResolvedEnvironmentalLighting::LightingUsage::DiffuseIblFallback));
  EXPECT_FALSE(ResolvedEnvironmentalLighting::LightingUsageUsesDiffuseFallbackIntensity(
      ResolvedEnvironmentalLighting::LightingUsage::GlobalSpecularFallback));
  EXPECT_FALSE(ResolvedEnvironmentalLighting::LightingUsageUsesDiffuseFallbackIntensity(
      ResolvedEnvironmentalLighting::LightingUsage::RayCameraEnvironmentEvent));
  EXPECT_FALSE(ResolvedEnvironmentalLighting::LightingUsageUsesDiffuseFallbackIntensity(
      ResolvedEnvironmentalLighting::LightingUsage::ReflectionProbeBakeEnvironmentInput));
  EXPECT_FALSE(ResolvedEnvironmentalLighting::LightingUsageUsesDiffuseFallbackIntensity(
      ResolvedEnvironmentalLighting::LightingUsage::ValidDdgiSurfaceIrradiance));

  EXPECT_TRUE(ResolvedEnvironmentalLighting::LightingUsageUsesSpecularFallbackIntensity(
      ResolvedEnvironmentalLighting::LightingUsage::GlobalSpecularFallback));
  EXPECT_FALSE(ResolvedEnvironmentalLighting::LightingUsageUsesSpecularFallbackIntensity(
      ResolvedEnvironmentalLighting::LightingUsage::DdgiMissRadiance));
  EXPECT_FALSE(ResolvedEnvironmentalLighting::LightingUsageUsesSpecularFallbackIntensity(
      ResolvedEnvironmentalLighting::LightingUsage::DiffuseIblFallback));
  EXPECT_FALSE(ResolvedEnvironmentalLighting::LightingUsageUsesSpecularFallbackIntensity(
      ResolvedEnvironmentalLighting::LightingUsage::RayCameraEnvironmentEvent));
  EXPECT_FALSE(ResolvedEnvironmentalLighting::LightingUsageUsesSpecularFallbackIntensity(
      ResolvedEnvironmentalLighting::LightingUsage::ReflectionProbeBakeEnvironmentInput));
  EXPECT_FALSE(ResolvedEnvironmentalLighting::LightingUsageUsesSpecularFallbackIntensity(
      ResolvedEnvironmentalLighting::LightingUsage::ValidLocalReflectionProbeSample));

  EXPECT_FALSE(ResolvedEnvironmentalLighting::LocalReflectionProbePayloadsUseEnvironmentLightingIntensity());
  EXPECT_FALSE(ResolvedEnvironmentalLighting::ValidDdgiSurfaceIrradianceUsesEnvironmentLightingIntensity());
  EXPECT_FALSE(ResolvedEnvironmentalLighting::ReflectionProbeBakeUsesFallbackIntensities());
}

TEST(EnvironmentalLightingContract, RayCameraSeparatesVisibleBackgroundFromLightingSource) {
  EXPECT_TRUE(ResolvedEnvironmentalLighting::RayCameraPrimaryMissUsesCameraBackground());
  EXPECT_TRUE(ResolvedEnvironmentalLighting::RayCameraEnvironmentLightingUsesIndirectEnvironmentSource());
  EXPECT_FALSE(ResolvedEnvironmentalLighting::RayCameraUsesGlobalReflectionProbeAsRadianceSource());
  EXPECT_FALSE(ResolvedEnvironmentalLighting::RayCameraUsesFallbackIntensities());
}

TEST(EnvironmentalLightingContract, CameraBackgroundFieldsStayOutOfEnvironmentalLightingPreparation) {
  const auto camera_header = ReadTextFile(SourcePath("EvoEngine_SDK/include/Rendering/Camera.hpp"));
  const auto camera_settings = ReadTextFile(SourcePath("EvoEngine_SDK/include/Rendering/CameraSettings.hpp"));
  const auto render_instance_storage = ReadTextFile(SourcePath("EvoEngine_SDK/src/RenderInstanceStorage.cpp"));
  const auto render_layer = ReadTextFile(SourcePath("EvoEngine_SDK/src/RenderLayer.cpp"));

  ASSERT_FALSE(camera_header.empty());
  ASSERT_FALSE(camera_settings.empty());
  ASSERT_FALSE(render_instance_storage.empty());
  ASSERT_FALSE(render_layer.empty());

  EXPECT_NE(camera_header.find("using BackgroundSource = CameraSettings::BackgroundSource"), std::string::npos);
  EXPECT_NE(camera_header.find("AssetRef background_environment"), std::string::npos);
  EXPECT_NE(camera_settings.find("enum class BackgroundSource"), std::string::npos);
  EXPECT_NE(camera_settings.find("background_source = BackgroundSource::Cubemap"), std::string::npos);
  EXPECT_EQ(render_instance_storage.find("camera_settings.background_intensity"), std::string::npos);
  EXPECT_EQ(render_instance_storage.find("Camera::BackgroundSource"), std::string::npos);
  EXPECT_NE(render_layer.find("ResolveEnvironmentalLighting(scene)"), std::string::npos);
  EXPECT_EQ(render_layer.find("scene->environment.sky_light_intensity_scale"), std::string::npos);
  EXPECT_EQ(render_layer.find("main_camera->camera_settings.background_intensity"), std::string::npos);
}

TEST(EnvironmentalLightingContract, LocalProbeAndDdgiContractsPreserveCurrentRendererCaps) {
  EXPECT_EQ(ResolvedEnvironmentalLighting::kMaxLocalReflectionProbeCount, 32u);
  EXPECT_EQ(ResolvedEnvironmentalLighting::kMaxDdgiVolumeCount, 8u);

  ResolvedEnvironmentalLighting::LocalReflectionProbe local_probe;
  EXPECT_TRUE(local_probe.enabled);
  EXPECT_TRUE(local_probe.box_projection);
  EXPECT_FLOAT_EQ(local_probe.reflection_intensity, 1.0f);
  EXPECT_FLOAT_EQ(local_probe.blend_distance, 1.0f);

  ResolvedEnvironmentalLighting::DdgiVolume ddgi_volume;
  EXPECT_TRUE(ddgi_volume.enabled);
  EXPECT_EQ(ddgi_volume.probe_counts, glm::ivec3(10, 6, 16));
  EXPECT_EQ(ddgi_volume.probe_spacing, glm::vec3(1.5f));
}

TEST(EnvironmentalLightingContract, DocsAndHeaderCarryLockedFallbackTerminology) {
  const auto header = ReadTextFile(SourcePath("EvoEngine_SDK/include/Rendering/PBR/ResolvedEnvironmentalLighting.hpp"));
  const auto rendering_docs = ReadTextFile(SourcePath("docs/rendering.md"));
  const auto reflection_probe_docs = ReadTextFile(SourcePath("docs/reflection-probes.md"));

  ASSERT_FALSE(header.empty());
  ASSERT_FALSE(rendering_docs.empty());
  ASSERT_FALSE(reflection_probe_docs.empty());

  EXPECT_NE(header.find("environment_lighting_intensity"), std::string::npos);
  EXPECT_NE(header.find("kDefaultDiffuseFallbackIntensity"), std::string::npos);
  EXPECT_NE(header.find("kDefaultSpecularFallbackIntensity"), std::string::npos);
  EXPECT_NE(header.find("LightingUsageUsesDiffuseFallbackIntensity"), std::string::npos);
  EXPECT_NE(header.find("LightingUsageUsesSpecularFallbackIntensity"), std::string::npos);
  EXPECT_NE(header.find("LocalReflectionProbePayloadsUseEnvironmentLightingIntensity"), std::string::npos);
  EXPECT_NE(header.find("RayCameraUsesGlobalReflectionProbeAsRadianceSource"), std::string::npos);
  EXPECT_EQ(header.find("indirect_sky_intensity"), std::string::npos);
  EXPECT_EQ(header.find("global_reflection_fallback_intensity"), std::string::npos);

  EXPECT_NE(rendering_docs.find("Environmental Lighting Ownership Target"), std::string::npos);
  EXPECT_NE(rendering_docs.find("Camera primary miss:"), std::string::npos);
  EXPECT_NE(rendering_docs.find("Surface/volume environment sampling and secondary misses:"), std::string::npos);
  EXPECT_NE(rendering_docs.find("Ray cameras do not use `Scene::global_reflection_probe_fallback`"), std::string::npos);
  EXPECT_NE(rendering_docs.find("Ray cameras ignore `diffuse_fallback_intensity`"), std::string::npos);
  EXPECT_NE(rendering_docs.find("diffuse_fallback_intensity = 1.0f"), std::string::npos);
  EXPECT_NE(rendering_docs.find("specular_fallback_intensity = 1.0f"), std::string::npos);
  EXPECT_NE(rendering_docs.find("fallback contribution still uses the resolved fallback intensity"), std::string::npos);
  EXPECT_NE(rendering_docs.find("DDGI miss radiance is"), std::string::npos);
  EXPECT_NE(rendering_docs.find("Bake environment input uses `environment_lighting_intensity` only"),
            std::string::npos);
  EXPECT_NE(rendering_docs.find("valid runtime local reflection-probe payloads"), std::string::npos);
  EXPECT_NE(rendering_docs.find("environment_lighting_intensity = 1.0f"), std::string::npos);
  EXPECT_NE(rendering_docs.find("renderer consumption path exist"), std::string::npos);
  EXPECT_NE(rendering_docs.find("asset for local probes and DDGI volumes"), std::string::npos);
  EXPECT_EQ(rendering_docs.find("extract the current legacy authoring into a new `.eveenvironmentallighting` asset"),
            std::string::npos);
  EXPECT_EQ(rendering_docs.find("indirect_sky_intensity"), std::string::npos);
  EXPECT_EQ(rendering_docs.find("global_reflection_fallback_intensity"), std::string::npos);
  EXPECT_EQ(rendering_docs.find("Renderer consumption remains on the existing scene/DDGI component paths"),
            std::string::npos);

  EXPECT_NE(reflection_probe_docs.find("Scene::global_reflection_probe_fallback"), std::string::npos);
  EXPECT_NE(reflection_probe_docs.find("Asset-owned local probes are the runtime"), std::string::npos);
  EXPECT_NE(reflection_probe_docs.find("RenderLayer inspector's"), std::string::npos);
  EXPECT_NE(reflection_probe_docs.find("all-probe bounds toggle"), std::string::npos);
  EXPECT_NE(reflection_probe_docs.find("per-probe debug bounds"), std::string::npos);
  EXPECT_NE(reflection_probe_docs.find("Bake All Local Probe Payloads"), std::string::npos);
  EXPECT_NE(reflection_probe_docs.find("bake action queues the same"), std::string::npos);
  EXPECT_NE(reflection_probe_docs.find("reflection-probe capture path used by asset entries"), std::string::npos);
  EXPECT_NE(reflection_probe_docs.find("environment_lighting_intensity * specular_fallback_intensity"),
            std::string::npos);
  EXPECT_NE(reflection_probe_docs.find("forced to zero during capture"), std::string::npos);
  EXPECT_NE(reflection_probe_docs.find("current direct lighting plus incident diffuse IBL/DDGI lighting"),
            std::string::npos);
  EXPECT_NE(reflection_probe_docs.find("material's diffuse albedo and metallic response are applied"),
            std::string::npos);
  EXPECT_NE(reflection_probe_docs.find("not bake inputs."), std::string::npos);
  EXPECT_NE(reflection_probe_docs.find("does not run a stale scan or batch stale rebake"), std::string::npos);
  EXPECT_NE(reflection_probe_docs.find("diffuse/specular fallback factors"), std::string::npos);
  EXPECT_EQ(reflection_probe_docs.find("component probes"), std::string::npos);
  EXPECT_EQ(reflection_probe_docs.find("EnvironmentalMap::global_reflection_probe` supplies"), std::string::npos);
  EXPECT_EQ(reflection_probe_docs.find("live renderer still gathers placed `ReflectionProbe` components"),
            std::string::npos);
}

TEST(EnvironmentalLightingContract, GlobalSpecularFallbackUsesSceneReference) {
  const auto scene_header = ReadTextFile(SourcePath("EvoEngine_SDK/include/Core/ECS/Scene.hpp"));
  const auto camera_source = ReadTextFile(SourcePath("EvoEngine_SDK/src/Camera.cpp"));
  const auto render_layer_source = ReadTextFile(SourcePath("EvoEngine_SDK/src/RenderLayer.cpp"));
  const auto render_instance_storage_source = ReadTextFile(SourcePath("EvoEngine_SDK/src/RenderInstanceStorage.cpp"));
  const auto lighting_shader =
      ReadTextFile(SourcePath("EvoEngine_SDK/Internals/DefaultResources/Shaders/Includes/Lighting.slangh"));
  const auto inspector_source = ReadTextFile(SourcePath("EvoEngine_SDK/src/Editor/SDKInspectionAdapters.cpp"));
  const auto demo_scene_source = ReadTextFile(SourcePath("EvoEngine_App/src/DemoScene.cpp"));

  ASSERT_FALSE(scene_header.empty());
  ASSERT_FALSE(camera_source.empty());
  ASSERT_FALSE(render_layer_source.empty());
  ASSERT_FALSE(render_instance_storage_source.empty());
  ASSERT_FALSE(lighting_shader.empty());
  ASSERT_FALSE(inspector_source.empty());
  ASSERT_FALSE(demo_scene_source.empty());

  EXPECT_NE(scene_header.find("AssetRef global_reflection_probe_fallback"), std::string::npos);
  EXPECT_NE(scene_header.find("GetGlobalReflectionProbeFallback"), std::string::npos);
  EXPECT_NE(camera_source.find("scene->GetGlobalReflectionProbeFallback()"), std::string::npos);
  EXPECT_NE(render_layer_source.find("scene->GetGlobalReflectionProbeFallback(false)"), std::string::npos);
  EXPECT_NE(render_instance_storage_source.find("environment_info_block.diffuse_sky_intensity = "
                                                "environment_lighting_intensity"),
            std::string::npos);
  EXPECT_NE(render_instance_storage_source.find("environment_info_block.global_reflection_intensity = "
                                                "environment_lighting_intensity"),
            std::string::npos);
  EXPECT_NE(render_instance_storage_source.find("environment_info_block.diffuse_fallback_intensity = "
                                                "environment_lighting_intensity * diffuse_fallback_intensity"),
            std::string::npos);
  EXPECT_NE(render_instance_storage_source.find("environment_info_block.specular_fallback_intensity = "
                                                "environment_lighting_intensity * specular_fallback_intensity"),
            std::string::npos);
  EXPECT_NE(render_layer_source.find("environment_info_block.diffuse_fallback_intensity = 0.0f"), std::string::npos);
  EXPECT_NE(render_layer_source.find("environment_info_block.specular_fallback_intensity = 0.0f"), std::string::npos);
  EXPECT_NE(lighting_shader.find("globalPrefiltered"), std::string::npos);
  EXPECT_NE(lighting_shader.find("EE_ENVIRONMENT.diffuse_fallback_intensity"), std::string::npos);
  EXPECT_NE(lighting_shader.find("EE_ENVIRONMENT.specular_fallback_intensity"), std::string::npos);
  EXPECT_EQ(lighting_shader.find("EE_REFLECTION_LIGHTING_SCALE"), std::string::npos);
  EXPECT_NE(inspector_source.find("Global Reflection Probe Fallback"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("scene->global_reflection_probe_fallback = "
                                   "Resources::GetInstance().GetDefaultGlobalReflectionProbe()"),
            std::string::npos);
  EXPECT_EQ(demo_scene_source.find("environment->global_reflection_probe"), std::string::npos);
  EXPECT_EQ(scene_header.find("GetGlobalReflectionProbe()"), std::string::npos);
  EXPECT_EQ(camera_source.find("scene->environment.GetGlobalReflectionProbe()"), std::string::npos);
  EXPECT_EQ(render_layer_source.find("GetPrivateComponentOwnersList<ReflectionProbe>"), std::string::npos);
  EXPECT_EQ(render_layer_source.find("UnsafeGetPrivateComponentOwnersList<DdgiVolume>"), std::string::npos);
  EXPECT_EQ(render_instance_storage_source.find("GetPrivateComponentOwnersList<ReflectionProbe>"), std::string::npos);
}

TEST(EnvironmentalLightingContract, EnvironmentalMapNoLongerOwnsGlobalSpecularFallback) {
  const auto environmental_map_header =
      ReadTextFile(SourcePath("EvoEngine_SDK/include/Rendering/PBR/EnvironmentalMap.hpp"));
  const auto environmental_map_source = ReadTextFile(SourcePath("EvoEngine_SDK/src/EnvironmentalMap.cpp"));
  const auto application_source = ReadTextFile(SourcePath("EvoEngine_SDK/src/Application.cpp"));
  const auto resources_header = ReadTextFile(SourcePath("EvoEngine_SDK/include/Core/Resources.hpp"));
  const auto resources_source = ReadTextFile(SourcePath("EvoEngine_SDK/src/Resources.cpp"));
  const auto camera_source = ReadTextFile(SourcePath("EvoEngine_SDK/src/Camera.cpp"));
  const auto render_layer_source = ReadTextFile(SourcePath("EvoEngine_SDK/src/RenderLayer.cpp"));

  ASSERT_FALSE(environmental_map_header.empty());
  ASSERT_FALSE(environmental_map_source.empty());
  ASSERT_FALSE(application_source.empty());
  ASSERT_FALSE(resources_header.empty());
  ASSERT_FALSE(resources_source.empty());
  ASSERT_FALSE(camera_source.empty());
  ASSERT_FALSE(render_layer_source.empty());

  EXPECT_EQ(environmental_map_header.find("global_reflection_probe"), std::string::npos);
  EXPECT_EQ(environmental_map_source.find("global_reflection_probe"), std::string::npos);
  EXPECT_EQ(application_source.find("global_reflection_probe.Load"), std::string::npos);
  EXPECT_EQ(application_source.find("reflection_probe\", in"), std::string::npos);
  EXPECT_NE(resources_header.find("GetDefaultGlobalReflectionProbe"), std::string::npos);
  EXPECT_NE(resources_source.find("default_global_reflection_probe_->ConstructFromCubemap"), std::string::npos);
  EXPECT_NE(camera_source.find("Resources::GetInstance().GetDefaultGlobalReflectionProbe()"), std::string::npos);
  EXPECT_NE(render_layer_source.find("Resources::GetInstance().GetDefaultGlobalReflectionProbe()"), std::string::npos);
}
