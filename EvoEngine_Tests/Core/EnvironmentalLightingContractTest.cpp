#include "EvoEngine_SDK_PCH.hpp"

#include <gtest/gtest.h>

#include "EnvironmentalLighting.hpp"
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
  EXPECT_FALSE(ResolvedEnvironmentalLighting::LightingUsageUsesEnvironmentLightingIntensity(
      ResolvedEnvironmentalLighting::LightingUsage::DiffuseIblFallback));
  EXPECT_FALSE(ResolvedEnvironmentalLighting::LightingUsageUsesEnvironmentLightingIntensity(
      ResolvedEnvironmentalLighting::LightingUsage::GlobalSpecularFallback));
  EXPECT_TRUE(ResolvedEnvironmentalLighting::LightingUsageUsesEnvironmentLightingIntensity(
      ResolvedEnvironmentalLighting::LightingUsage::RayCameraEnvironmentEvent));
  EXPECT_FALSE(ResolvedEnvironmentalLighting::LightingUsageUsesEnvironmentLightingIntensity(
      ResolvedEnvironmentalLighting::LightingUsage::ReflectionProbeBakeEnvironmentInput));
  EXPECT_FALSE(ResolvedEnvironmentalLighting::LightingUsageUsesEnvironmentLightingIntensity(
      ResolvedEnvironmentalLighting::LightingUsage::ValidDdgiSurfaceIrradiance));
  EXPECT_FALSE(ResolvedEnvironmentalLighting::LightingUsageUsesEnvironmentLightingIntensity(
      ResolvedEnvironmentalLighting::LightingUsage::ValidLocalReflectionProbeSample));

  EXPECT_FALSE(ResolvedEnvironmentalLighting::LightingUsageUsesDiffuseFallbackIntensity(
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

  EnvironmentalLighting::LocalReflectionProbe authored_local_probe;
  EXPECT_FLOAT_EQ(authored_local_probe.blend_distance, 0.05f);

  ResolvedEnvironmentalLighting::LocalReflectionProbe local_probe;
  EXPECT_TRUE(local_probe.enabled);
  EXPECT_TRUE(local_probe.box_projection);
  EXPECT_FLOAT_EQ(local_probe.reflection_intensity, 1.0f);
  EXPECT_FLOAT_EQ(local_probe.blend_distance, 0.05f);

  ResolvedEnvironmentalLighting::DdgiVolume ddgi_volume;
  EXPECT_TRUE(ddgi_volume.enabled);
  EXPECT_EQ(ddgi_volume.probe_counts, glm::ivec3(10, 6, 16));
  EXPECT_EQ(ddgi_volume.probe_spacing, glm::vec3(1.5f));
}

TEST(EnvironmentalLightingContract, ProbeAuthoringGizmoIsInspectorActivatedTransientAndSceneScoped) {
  const auto inspector = ReadTextFile(SourcePath("EvoEngine_SDK/src/Editor/SDKInspectionAdapters.cpp"));
  const auto editor = ReadTextFile(SourcePath("EvoEngine_SDK/src/EditorLayer.cpp"));
  const auto asset_manager = ReadTextFile(SourcePath("EvoEngine_SDK/src/AssetManager.cpp"));
  const auto serialization = ReadTextFile(SourcePath("EvoEngine_SDK/src/EnvironmentalLighting.cpp"));

  ASSERT_FALSE(inspector.empty());
  ASSERT_FALSE(editor.empty());
  ASSERT_FALSE(asset_manager.empty());
  ASSERT_FALSE(serialization.empty());
  EXPECT_NE(inspector.find("Edit in Scene"), std::string::npos);
  EXPECT_NE(inspector.find("active_lighting.get() == &lighting"), std::string::npos);
  EXPECT_NE(inspector.find("ImGuiHoveredFlags_AllowWhenDisabled"), std::string::npos);
  EXPECT_NE(inspector.find("RenderActiveEnvironmentalLightingGizmoBound"), std::string::npos);
  EXPECT_NE(inspector.find("EnvironmentalLightingGizmoTargetType::LocalReflectionProbe"), std::string::npos);
  EXPECT_NE(inspector.find("EnvironmentalLightingGizmoTargetType::DdgiVolume"), std::string::npos);
  EXPECT_NE(inspector.find("Position##Authoring"), std::string::npos);
  EXPECT_NE(inspector.find("Rotation##Authoring"), std::string::npos);
  EXPECT_NE(inspector.find("Scale##Authoring"), std::string::npos);
  EXPECT_NE(inspector.find("SelectLocalTransformGizmoOperation(LocalTransformGizmoOperation::Rotate)"),
            std::string::npos);
  EXPECT_NE(inspector.find("SelectLocalTransformGizmoOperation(LocalTransformGizmoOperation::Scale)"),
            std::string::npos);
  EXPECT_NE(editor.find("CreateAuthoringGizmoTransform(transform, pivot)"), std::string::npos);
  EXPECT_NE(editor.find("TryConvertAuthoringGizmoTransform(gizmo_transform, pivot, normalized)"), std::string::npos);
  EXPECT_NE(editor.find("active_lighting != lighting"), std::string::npos);
  EXPECT_NE(editor.find("lighting->SetUnsaved()"), std::string::npos);
  EXPECT_NE(editor.find("selected_entity_.GetIndex() != 0 && !scene->IsEntityValid(selected_entity_)"),
            std::string::npos);
  EXPECT_NE(editor.find("else if (selected_entity_.GetIndex() != 0)"), std::string::npos);
  EXPECT_NE(editor.find("suppress_scene_camera_selection_ = true"), std::string::npos);
  EXPECT_NE(editor.find("ImGui::IsItemHovered() || ImGui::IsItemActive()"), std::string::npos);
  EXPECT_NE(editor.find("!suppress_scene_camera_selection_"), std::string::npos);
  EXPECT_NE(asset_manager.find("ClearEnvironmentalLightingGizmoTarget(asset->GetHandle())"), std::string::npos);
  EXPECT_EQ(serialization.find("EnvironmentalLightingGizmoTarget"), std::string::npos);
  EXPECT_EQ(serialization.find("Edit in Scene"), std::string::npos);
}

TEST(EnvironmentalLightingContract, BakeBackgroundAndLocalProbeMasterAreAssetOwned) {
  const auto header = ReadTextFile(SourcePath("EvoEngine_SDK/include/Rendering/PBR/EnvironmentalLighting.hpp"));
  const auto serialization = ReadTextFile(SourcePath("EvoEngine_SDK/src/EnvironmentalLighting.cpp"));
  const auto resolver = ReadTextFile(SourcePath("EvoEngine_SDK/src/EnvironmentalLightingResolver.cpp"));
  const auto inspector = ReadTextFile(SourcePath("EvoEngine_SDK/src/Editor/SDKInspectionAdapters.cpp"));
  const auto render_layer = ReadTextFile(SourcePath("EvoEngine_SDK/src/RenderLayer.cpp"));

  ASSERT_FALSE(header.empty());
  ASSERT_FALSE(serialization.empty());
  ASSERT_FALSE(resolver.empty());
  ASSERT_FALSE(inspector.empty());
  ASSERT_FALSE(render_layer.empty());
  EXPECT_NE(header.find("struct ReflectionProbeBakeBackground"), std::string::npos);
  EXPECT_NE(header.find("CameraSettings::BackgroundSource::InheritEnvironmentalLighting"), std::string::npos);
  EXPECT_NE(header.find("bool local_reflection_probes_enabled = true"), std::string::npos);
  EXPECT_NE(serialization.find("reflection_probe_bake_background"), std::string::npos);
  EXPECT_NE(serialization.find("local_reflection_probes_enabled"), std::string::npos);
  EXPECT_NE(resolver.find("if (!lighting.local_reflection_probes_enabled)"), std::string::npos);
  EXPECT_NE(inspector.find("bool InspectCameraBackground"), std::string::npos);
  const auto master = inspector.find("Enable local probe reflections");
  const auto background = inspector.find("InspectCameraBackground(editor_layer, background.source");
  const auto add = inspector.find("Add Local Reflection Probe");
  ASSERT_NE(master, std::string::npos);
  ASSERT_NE(background, std::string::npos);
  ASSERT_NE(add, std::string::npos);
  EXPECT_LT(master, background);
  EXPECT_LT(background, add);
  EXPECT_NE(render_layer.find("camera->camera_settings.background_source"), std::string::npos);
  EXPECT_NE(render_layer.find("camera->skybox = background.cubemap"), std::string::npos);
  EXPECT_NE(render_layer.find("camera->background_environment = background.environmental_map"), std::string::npos);
}

TEST(EnvironmentalLightingContract, InspectorSeparatesAuthoringIntoOwnershipTabs) {
  const auto source = ReadTextFile(SourcePath("EvoEngine_SDK/src/Editor/SDKInspectionAdapters.cpp"));
  ASSERT_FALSE(source.empty());

  const auto inspector_begin = source.find("bool InspectEnvironmentalLighting(");
  const auto inspector_end = source.find("bool InspectEnvironmentalMap(", inspector_begin);
  ASSERT_NE(inspector_begin, std::string::npos);
  ASSERT_NE(inspector_end, std::string::npos);
  const auto inspector = source.substr(inspector_begin, inspector_end - inspector_begin);

  const auto tab_bar = inspector.find("BeginTabBar(\"EnvironmentalLightingInspectionTabs\")");
  const auto general = inspector.find("BeginTabItem(\"General\")");
  const auto ddgi = inspector.find("BeginTabItem(\"DDGI\")");
  const auto reflection_probes = inspector.find("BeginTabItem(\"Reflection Probes\")");
  const auto tab_bar_end = inspector.find("EndTabBar()", reflection_probes);
  ASSERT_NE(tab_bar, std::string::npos);
  ASSERT_NE(general, std::string::npos);
  ASSERT_NE(ddgi, std::string::npos);
  ASSERT_NE(reflection_probes, std::string::npos);
  ASSERT_NE(tab_bar_end, std::string::npos);
  EXPECT_LT(tab_bar, general);
  EXPECT_LT(general, ddgi);
  EXPECT_LT(ddgi, reflection_probes);
  EXPECT_LT(reflection_probes, tab_bar_end);

  const auto environment_source = inspector.find("InspectEnvironmentalLightingSource(");
  const auto environment_intensity = inspector.find("Environment lighting intensity");
  const auto diffuse_intensity = inspector.find("Diffuse fallback intensity");
  const auto specular_intensity = inspector.find("Specular fallback intensity");
  const auto ddgi_settings = inspector.find("TreeNodeEx(\"Settings\"");
  const auto ddgi_volumes = inspector.find("TreeNodeEx(\"Volumes\"");
  const auto reflection_master = inspector.find("Enable local probe reflections");
  const auto bake_background = inspector.find("InspectCameraBackground(");
  const auto add_reflection_probe = inspector.find("Add Local Reflection Probe");
  EXPECT_GT(environment_source, general);
  EXPECT_GT(environment_intensity, general);
  EXPECT_GT(diffuse_intensity, general);
  EXPECT_GT(specular_intensity, general);
  EXPECT_LT(environment_source, ddgi);
  EXPECT_LT(environment_intensity, ddgi);
  EXPECT_LT(diffuse_intensity, ddgi);
  EXPECT_LT(specular_intensity, ddgi);
  EXPECT_GT(ddgi_settings, ddgi);
  EXPECT_GT(ddgi_volumes, ddgi);
  EXPECT_LT(ddgi_settings, reflection_probes);
  EXPECT_LT(ddgi_volumes, reflection_probes);
  EXPECT_GT(reflection_master, reflection_probes);
  EXPECT_GT(bake_background, reflection_probes);
  EXPECT_GT(add_reflection_probe, reflection_probes);
  EXPECT_LT(reflection_master, tab_bar_end);
  EXPECT_LT(bake_background, tab_bar_end);
  EXPECT_LT(add_reflection_probe, tab_bar_end);

  EXPECT_EQ(inspector.find("TreeNodeEx(\"DDGI settings\""), std::string::npos);
  EXPECT_EQ(inspector.find("TreeNodeEx(\"DDGI volumes\""), std::string::npos);
  EXPECT_EQ(inspector.find("TreeNodeEx(\"Local reflection probes\""), std::string::npos);
  const auto debug_bounds = inspector.find("RenderEnvironmentalLightingDebugProbeBounds", tab_bar_end);
  ASSERT_NE(debug_bounds, std::string::npos);
  EXPECT_GT(debug_bounds, tab_bar_end);
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
  EXPECT_NE(rendering_docs.find("Bake background intensity is independent"), std::string::npos);
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
  EXPECT_NE(reflection_probe_docs.find("specular_fallback_intensity` directly"), std::string::npos);
  EXPECT_NE(reflection_probe_docs.find("forced to zero during capture"), std::string::npos);
  EXPECT_NE(reflection_probe_docs.find("composed from material AO, eligible GTAO visibility, and DDGI probe"),
            std::string::npos);
  EXPECT_NE(reflection_probe_docs.find("probe visibility atlas/Chebyshev test, not irradiance RGB"), std::string::npos);
  EXPECT_NE(reflection_probe_docs.find("disabled, or invalid DDGI blends toward white visibility"), std::string::npos);
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
      ReadTextFile(SourcePath("EvoEngine_SDK/Internals/DefaultResources/Shaders/Modules/EvoEngine/Lighting.slang")) +
      ReadTextFile(
          SourcePath("EvoEngine_SDK/Internals/DefaultResources/Shaders/Modules/EvoEngine/LightingFixedSet3.slang"));
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
                                                "diffuse_fallback_intensity"),
            std::string::npos);
  EXPECT_NE(render_instance_storage_source.find("environment_info_block.specular_fallback_intensity = "
                                                "specular_fallback_intensity"),
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
