#include "EvoEngine_SDK_PCH.hpp"

#include <gtest/gtest.h>

#include "Application.hpp"
#include "ApplicationContext.hpp"
#include "ApplicationInitializationSettings.hpp"
#include "AssetManager.hpp"
#include "EnvironmentalLighting.hpp"
#include "EnvironmentalLightingResolver.hpp"
#include "EnvironmentalMap.hpp"
#include "GlobalReflectionProbe.hpp"
#include "Scene.hpp"
#include "Serialization.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <vector>

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

bool ContainsAssetHandle(const std::vector<AssetRef>& refs, const Handle handle) {
  return std::any_of(refs.begin(), refs.end(), [handle](const AssetRef& ref) {
    return ref.GetAssetHandle() == handle;
  });
}

bool ContainsLocalAsset(const YAML::Node& scene_node, const std::string& type_name, const Handle handle) {
  const auto local_assets = scene_node["LocalAssets"];
  if (!local_assets) {
    return false;
  }
  for (const auto& asset_node : local_assets) {
    if (asset_node["type_name"].as<std::string>() == type_name &&
        Handle(asset_node["handle"].as<uint64_t>()) == handle) {
      return true;
    }
  }
  return false;
}

bool ContainsStableId(const std::vector<ResolvedEnvironmentalLighting::LocalReflectionProbe>& probes,
                      const uint64_t stable_id) {
  return std::any_of(probes.begin(), probes.end(), [stable_id](const auto& probe) {
    return probe.stable_id == stable_id;
  });
}
}  // namespace

TEST(EnvironmentalLightingAsset, SerializesCompleteAuthoringSetup) {
  Application app;
  ApplicationContextScope scope(app);
  app.Initialize(EmptyProjectSettings());
  const auto lighting = AssetManager::CreateTemporaryAsset<EnvironmentalLighting>();
  const auto environment = AssetManager::CreateTemporaryAsset<EnvironmentalMap>();
  const auto local_probe_payload = AssetManager::CreateTemporaryAsset<GlobalReflectionProbe>();
  ASSERT_TRUE(lighting);
  ASSERT_TRUE(environment);
  ASSERT_TRUE(local_probe_payload);

  lighting->indirect_environment_source.kind = EnvironmentalLighting::IndirectEnvironmentSourceKind::EnvironmentalMap;
  lighting->indirect_environment_source.environmental_map = environment;
  lighting->indirect_environment_source.color = glm::vec3(0.2f, 0.4f, 0.6f);
  lighting->indirect_environment_source.gamma = 1.8f;
  lighting->indirect_environment_source.rotation = 0.25f;
  lighting->environment_lighting_intensity = 0.35f;
  lighting->diffuse_fallback_intensity = 0.45f;
  lighting->specular_fallback_intensity = 0.55f;
  lighting->ddgi_settings.runtime.enabled = true;
  lighting->ddgi_settings.runtime.ray_count = 96;
  lighting->ddgi_settings.volume_defaults.probe_counts = glm::ivec3(4, 5, 6);
  lighting->ddgi_settings.debug.visualize_probe_positions = false;

  EnvironmentalLighting::LocalReflectionProbe probe;
  probe.name = "Gallery Probe";
  probe.stable_id = 101;
  probe.global_reflection_probe = local_probe_payload;
  probe.transform[3] = glm::vec4(1.0f, 2.0f, 3.0f, 1.0f);
  probe.box_extents = glm::vec3(7.0f, 3.0f, 2.0f);
  probe.box_projection_extents = glm::vec3(6.0f, 2.5f, 1.5f);
  probe.sphere_radius = 8.0f;
  probe.blend_distance = 0.75f;
  probe.reflection_intensity = 1.25f;
  probe.artist_priority = 4;
  probe.shape = static_cast<int>(EnvironmentalLighting::LocalReflectionProbeShape::Sphere);
  probe.box_projection = false;
  lighting->local_reflection_probes.push_back(probe);

  EnvironmentalLighting::DdgiVolume volume;
  volume.name = "Gallery DDGI";
  volume.stable_id = 202;
  volume.transform[3] = glm::vec4(-2.0f, 1.0f, 5.0f, 1.0f);
  volume.probe_counts = glm::ivec3(3, 4, 5);
  volume.probe_spacing = glm::vec3(1.0f, 1.5f, 2.0f);
  volume.volume_origin = glm::vec3(-1.0f, 0.5f, 2.0f);
  volume.artist_priority = 7;
  volume.movement_type = static_cast<int>(DdgiVolumeMovementType::Scrolling);
  volume.emissive_mesh_sampling_mode = static_cast<int>(DdgiEmissiveMeshSamplingMode::Off);
  volume.enable_probe_relocation = false;
  volume.enable_probe_classification = true;
  volume.relocation_distance = 0.5f;
  volume.auto_invalidate_trigger_conditions =
      DdgiVolumeTriggerConditionLightEnableChanged | DdgiVolumeTriggerConditionGeometryChanged;
  volume.warmup_trigger_conditions = DdgiVolumeTriggerConditionAll;
  lighting->ddgi_volumes.push_back(volume);

  YAML::Emitter out;
  out << YAML::BeginMap;
  Serialization::SerializeObject(out, static_cast<IAsset&>(*lighting));
  out << YAML::EndMap;
  const auto node = YAML::Load(out.c_str());
  EXPECT_EQ(node["indirect_environment_source"]["kind"].as<uint32_t>(),
            static_cast<uint32_t>(EnvironmentalLighting::IndirectEnvironmentSourceKind::EnvironmentalMap));
  EXPECT_EQ(node["indirect_environment_source"]["environmental_map"]["asset_handle_"].as<uint64_t>(),
            environment->GetHandle().GetValue());
  EXPECT_FLOAT_EQ(node["environment_lighting_intensity"].as<float>(), 0.35f);
  EXPECT_FLOAT_EQ(node["diffuse_fallback_intensity"].as<float>(), 0.45f);
  EXPECT_FLOAT_EQ(node["specular_fallback_intensity"].as<float>(), 0.55f);
  EXPECT_EQ(node["ddgi_settings"]["runtime"]["ray_count"].as<int>(), 96);
  ASSERT_EQ(node["local_reflection_probes"].size(), 1u);
  ASSERT_EQ(node["ddgi_volumes"].size(), 1u);
  EXPECT_EQ(node["local_reflection_probes"][0]["global_reflection_probe"]["asset_handle_"].as<uint64_t>(),
            local_probe_payload->GetHandle().GetValue());
  EXPECT_EQ(node["ddgi_volumes"][0]["emissive_mesh_sampling_mode"].as<int>(),
            static_cast<int>(DdgiEmissiveMeshSamplingMode::Off));
  EXPECT_EQ(node["ddgi_volumes"][0]["auto_invalidate_trigger_conditions"].as<int>(),
            DdgiVolumeTriggerConditionLightEnableChanged | DdgiVolumeTriggerConditionGeometryChanged);

  std::vector<AssetRef> refs;
  Serialization::CollectAssetRefs(static_cast<IAsset&>(*lighting), refs);
  EXPECT_TRUE(ContainsAssetHandle(refs, environment->GetHandle()));
  EXPECT_TRUE(ContainsAssetHandle(refs, local_probe_payload->GetHandle()));

  EnvironmentalLighting restored;
  Serialization::DeserializeObject(node, static_cast<IAsset&>(restored));
  EXPECT_EQ(restored.indirect_environment_source.kind,
            EnvironmentalLighting::IndirectEnvironmentSourceKind::EnvironmentalMap);
  EXPECT_EQ(restored.indirect_environment_source.environmental_map.GetAssetHandle(), environment->GetHandle());
  EXPECT_FLOAT_EQ(restored.indirect_environment_source.gamma, 1.8f);
  EXPECT_FLOAT_EQ(restored.environment_lighting_intensity, 0.35f);
  EXPECT_FLOAT_EQ(restored.diffuse_fallback_intensity, 0.45f);
  EXPECT_FLOAT_EQ(restored.specular_fallback_intensity, 0.55f);
  EXPECT_TRUE(restored.ddgi_settings.runtime.enabled);
  EXPECT_EQ(restored.ddgi_settings.runtime.ray_count, 96);
  ASSERT_EQ(restored.local_reflection_probes.size(), 1u);
  ASSERT_EQ(restored.ddgi_volumes.size(), 1u);
  EXPECT_EQ(restored.local_reflection_probes.front().global_reflection_probe.GetAssetHandle(),
            local_probe_payload->GetHandle());
  EXPECT_EQ(restored.local_reflection_probes.front().shape,
            static_cast<int>(EnvironmentalLighting::LocalReflectionProbeShape::Sphere));
  EXPECT_FALSE(restored.local_reflection_probes.front().box_projection);
  EXPECT_EQ(restored.ddgi_volumes.front().probe_counts, glm::ivec3(3, 4, 5));
  EXPECT_EQ(restored.ddgi_volumes.front().movement_type, static_cast<int>(DdgiVolumeMovementType::Scrolling));
  EXPECT_TRUE(restored.ddgi_volumes.front().enable_probe_classification);
  EXPECT_EQ(restored.ddgi_volumes.front().auto_invalidate_trigger_conditions,
            DdgiVolumeTriggerConditionLightEnableChanged | DdgiVolumeTriggerConditionGeometryChanged);
}

TEST(EnvironmentalLightingAsset, SceneReferenceSerializesAndClonesWithoutRendererConsumption) {
  Application app;
  ApplicationContextScope scope(app);
  app.Initialize(EmptyProjectSettings());
  const auto scene = AssetManager::CreateTemporaryAsset<Scene>();
  const auto lighting = AssetManager::CreateTemporaryAsset<EnvironmentalLighting>();
  ASSERT_TRUE(scene);
  ASSERT_TRUE(lighting);
  scene->environmental_lighting = lighting;

  YAML::Emitter out;
  out << YAML::BeginMap;
  Serialization::SerializeObject(out, static_cast<IAsset&>(*scene));
  out << YAML::EndMap;
  const auto node = YAML::Load(out.c_str());
  ASSERT_TRUE(node["environmental_lighting"]);
  EXPECT_EQ(node["environmental_lighting"]["asset_handle_"].as<uint64_t>(), lighting->GetHandle().GetValue());

  bool embeds_environmental_lighting = false;
  for (const auto& asset_node : node["LocalAssets"]) {
    embeds_environmental_lighting =
        embeds_environmental_lighting || asset_node["type_name"].as<std::string>() == "EnvironmentalLighting";
  }
  EXPECT_TRUE(embeds_environmental_lighting);

  const auto restored = AssetManager::CreateTemporaryAsset<Scene>();
  ASSERT_TRUE(restored);
  Serialization::DeserializeObject(node, static_cast<IAsset&>(*restored));
  EXPECT_EQ(restored->environmental_lighting.GetAssetHandle(), lighting->GetHandle());

  const auto cloned = AssetManager::CreateTemporaryAsset<Scene>();
  ASSERT_TRUE(cloned);
  Scene::Clone(scene, cloned);
  EXPECT_EQ(cloned->environmental_lighting.GetAssetHandle(), lighting->GetHandle());
}

TEST(EnvironmentalLightingAsset, SceneCreatesAndEmbedsTemporaryEnvironmentalLightingWhenMissing) {
  Application app;
  ApplicationContextScope scope(app);
  app.Initialize(EmptyProjectSettings());
  const auto scene = AssetManager::CreateTemporaryAsset<Scene>();
  ASSERT_TRUE(scene);
  app.Attach(scene);

  const auto created_lighting = scene->environmental_lighting.Get<EnvironmentalLighting>();
  ASSERT_TRUE(created_lighting);
  EXPECT_TRUE(created_lighting->IsTemporary());

  const auto resolved = ResolveEnvironmentalLighting(scene);
  EXPECT_TRUE(resolved.environmental_lighting_asset_assigned);
  EXPECT_FALSE(resolved.environmental_lighting_asset_missing);
  EXPECT_EQ(resolved.indirect_environment_source.kind,
            ResolvedEnvironmentalLighting::IndirectEnvironmentSourceKind::EngineDefault);
  EXPECT_TRUE(resolved.uses_engine_default_indirect_environment_source);
  EXPECT_FLOAT_EQ(resolved.environment_lighting_intensity,
                  ResolvedEnvironmentalLighting::kDefaultEnvironmentLightingIntensity);
  EXPECT_FLOAT_EQ(resolved.diffuse_fallback_intensity, ResolvedEnvironmentalLighting::kDefaultDiffuseFallbackIntensity);
  EXPECT_FLOAT_EQ(resolved.specular_fallback_intensity,
                  ResolvedEnvironmentalLighting::kDefaultSpecularFallbackIntensity);

  scene->environmental_lighting.Clear();
  YAML::Emitter out;
  out << YAML::BeginMap;
  Serialization::SerializeObject(out, static_cast<IAsset&>(*scene));
  out << YAML::EndMap;
  const auto node = YAML::Load(out.c_str());
  const auto serialized_lighting = scene->environmental_lighting.Get<EnvironmentalLighting>();
  ASSERT_TRUE(serialized_lighting);
  ASSERT_TRUE(node["environmental_lighting"]);
  EXPECT_EQ(node["environmental_lighting"]["asset_handle_"].as<uint64_t>(),
            serialized_lighting->GetHandle().GetValue());
  EXPECT_TRUE(ContainsLocalAsset(node, "EnvironmentalLighting", serialized_lighting->GetHandle()));

  const auto restored = AssetManager::CreateTemporaryAsset<Scene>();
  ASSERT_TRUE(restored);
  Serialization::DeserializeObject(YAML::Load(R"(
entity_metadata_list: []
systems_: []
data_component_storage_list: []
)"),
                                   static_cast<IAsset&>(*restored));
  const auto restored_lighting = restored->environmental_lighting.Get<EnvironmentalLighting>();
  ASSERT_TRUE(restored_lighting);
  EXPECT_TRUE(restored_lighting->IsTemporary());
}

TEST(EnvironmentalLightingAsset, SourceContractRoutesRendererThroughResolverForE6) {
  const auto scene_header = ReadTextFile(SourcePath("EvoEngine_SDK/include/Core/ECS/Scene.hpp"));
  const auto asset_header = ReadTextFile(SourcePath("EvoEngine_SDK/include/Rendering/PBR/EnvironmentalLighting.hpp"));
  const auto application_source = ReadTextFile(SourcePath("EvoEngine_SDK/src/Application.cpp"));
  const auto inspector_source = ReadTextFile(SourcePath("EvoEngine_SDK/src/Editor/SDKInspectionAdapters.cpp"));
  const auto render_layer_header = ReadTextFile(SourcePath("EvoEngine_SDK/include/Layers/RenderLayer.hpp"));
  const auto render_layer_source = ReadTextFile(SourcePath("EvoEngine_SDK/src/RenderLayer.cpp"));
  const auto render_instance_source = ReadTextFile(SourcePath("EvoEngine_SDK/src/RenderInstanceStorage.cpp"));

  ASSERT_FALSE(scene_header.empty());
  ASSERT_FALSE(asset_header.empty());
  ASSERT_FALSE(application_source.empty());
  ASSERT_FALSE(inspector_source.empty());
  ASSERT_FALSE(render_layer_header.empty());
  ASSERT_FALSE(render_layer_source.empty());
  ASSERT_FALSE(render_instance_source.empty());

  EXPECT_NE(scene_header.find("AssetRef environmental_lighting"), std::string::npos);
  EXPECT_NE(asset_header.find("class EnvironmentalLighting final : public IAsset"), std::string::npos);
  EXPECT_NE(asset_header.find("std::vector<LocalReflectionProbe> local_reflection_probes"), std::string::npos);
  EXPECT_NE(asset_header.find("std::vector<DdgiVolume> ddgi_volumes"), std::string::npos);
  EXPECT_NE(asset_header.find("environment_lighting_intensity"), std::string::npos);
  EXPECT_NE(asset_header.find("diffuse_fallback_intensity"), std::string::npos);
  EXPECT_NE(asset_header.find("specular_fallback_intensity"), std::string::npos);
  EXPECT_NE(application_source.find(".eveenvironmentallighting"), std::string::npos);
  EXPECT_NE(inspector_source.find("InspectEnvironmentalLighting"), std::string::npos);
  EXPECT_EQ(inspector_source.find("#include \"EnvironmentalLightingResolver.hpp\""), std::string::npos);
  EXPECT_EQ(inspector_source.find("#include \"ProjectManager.hpp\""), std::string::npos);
  EXPECT_EQ(inspector_source.find("Extract Environmental Lighting Asset"), std::string::npos);
  EXPECT_EQ(inspector_source.find("ExtractEnvironmentalLightingFromLegacyScene"), std::string::npos);
  EXPECT_EQ(inspector_source.find("ProjectManager::SaveAsset"), std::string::npos);
  EXPECT_NE(inspector_source.find("RenderEnvironmentalLightingProbeBounds"), std::string::npos);
  EXPECT_NE(inspector_source.find("scene->environmental_lighting.Get<EnvironmentalLighting>()"), std::string::npos);
  EXPECT_NE(inspector_source.find("DDGI settings"), std::string::npos);
  EXPECT_NE(inspector_source.find("InspectDdgiRuntimeControls(lighting.ddgi_settings.runtime)"), std::string::npos);
  EXPECT_NE(inspector_source.find("Invalidate history now"), std::string::npos);
  EXPECT_EQ(inspector_source.find("Bake Stale Local Probe Payloads"), std::string::npos);
  EXPECT_NE(inspector_source.find("Bake Local Probe Payload"), std::string::npos);
  EXPECT_EQ(inspector_source.find("GetGlobalReflectionProbeBakeFingerprint"), std::string::npos);
  EXPECT_EQ(inspector_source.find("BakeStaleInScene"), std::string::npos);
  EXPECT_EQ(inspector_source.find("RegisterInspector<ReflectionProbe>"), std::string::npos);
  EXPECT_EQ(inspector_source.find("RegisterInspector<DdgiVolume>"), std::string::npos);
  EXPECT_EQ(render_layer_header.find("GetReflectionProbeSceneSourceFingerprint"), std::string::npos);
  EXPECT_EQ(render_layer_header.find("GetGlobalReflectionProbeBakeFingerprint"), std::string::npos);
  EXPECT_NE(render_layer_header.find("QueueGlobalReflectionProbeBake"), std::string::npos);
  EXPECT_NE(render_layer_source.find("#include \"EnvironmentalLightingResolver.hpp\""), std::string::npos);
  EXPECT_NE(render_layer_source.find("ResolveEnvironmentalLighting(scene)"), std::string::npos);
  EXPECT_NE(render_layer_source.find("ValidateGlobalReflectionProbeBakeRequest"), std::string::npos);
  EXPECT_NE(render_layer_source.find("BakeReflectionProbe(scene, position, target, source_fingerprint, error, retry)"),
            std::string::npos);
  EXPECT_NE(render_instance_source.find("#include \"EnvironmentalLightingResolver.hpp\""), std::string::npos);
  EXPECT_NE(render_instance_source.find("ResolveEnvironmentalLighting(target_scene)"), std::string::npos);
  EXPECT_EQ(render_layer_source.find("UnsafeGetPrivateComponentOwnersList<DdgiVolume>"), std::string::npos);
  EXPECT_EQ(render_instance_source.find("GetPrivateComponentOwnersList<ReflectionProbe>"), std::string::npos);
  EXPECT_EQ(application_source.find("RegisterPrivateComponent<DdgiVolume>"), std::string::npos);
  EXPECT_EQ(application_source.find("RegisterPrivateComponent<ReflectionProbe>"), std::string::npos);
}

TEST(EnvironmentalLightingAsset, LocalProbeInspectorDoesNotSynchronouslyLoadProbePayloads) {
  const auto asset_ref_header = ReadTextFile(SourcePath("EvoEngine_SDK/include/Core/ECS/AssetRef.hpp"));
  const auto asset_ref_source = ReadTextFile(SourcePath("EvoEngine_SDK/src/AssetRef.cpp"));
  const auto editor_layer_header = ReadTextFile(SourcePath("EvoEngine_SDK/include/Layers/EditorLayer.hpp"));
  const auto editor_layer_source = ReadTextFile(SourcePath("EvoEngine_SDK/src/EditorLayer.cpp"));
  const auto inspector_source = ReadTextFile(SourcePath("EvoEngine_SDK/src/Editor/SDKInspectionAdapters.cpp"));

  ASSERT_FALSE(asset_ref_header.empty());
  ASSERT_FALSE(asset_ref_source.empty());
  ASSERT_FALSE(editor_layer_header.empty());
  ASSERT_FALSE(editor_layer_source.empty());
  ASSERT_FALSE(inspector_source.empty());

  EXPECT_NE(asset_ref_header.find("std::shared_ptr<T> Peek() const"), std::string::npos);
  EXPECT_NE(asset_ref_source.find("AssetManager::PeekAssetImpl(asset_handle_)"), std::string::npos);
  EXPECT_NE(editor_layer_header.find("pending_asset_inspector_loads_"), std::string::npos);
  EXPECT_NE(editor_layer_source.find("AssetManager::RequestAssetLoad(asset_handle)"), std::string::npos);
  EXPECT_NE(editor_layer_source.find("PollPendingAssetInspectorLoads();"), std::string::npos);
  EXPECT_NE(editor_layer_header.find("const auto ptr = target.Peek<IAsset>();"), std::string::npos);
  EXPECT_EQ(editor_layer_header.find("const auto ptr = target.Get<IAsset>();"), std::string::npos);
  const auto payload_readout_start = inspector_source.find("void InspectEnvironmentalLightingLocalProbePayload");
  ASSERT_NE(payload_readout_start, std::string::npos);
  const auto payload_readout_end =
      inspector_source.find("bool InspectEnvironmentalLightingSource", payload_readout_start);
  ASSERT_NE(payload_readout_end, std::string::npos);
  const auto payload_readout =
      inspector_source.substr(payload_readout_start, payload_readout_end - payload_readout_start);
  EXPECT_NE(payload_readout.find("probe.global_reflection_probe.Peek<GlobalReflectionProbe>()"), std::string::npos);
  EXPECT_NE(payload_readout.find("Load / Inspect Probe Payload"), std::string::npos);
  EXPECT_EQ(payload_readout.find(".Get<GlobalReflectionProbe>()"), std::string::npos);
  EXPECT_EQ(payload_readout.find("Stale check:"), std::string::npos);
  EXPECT_EQ(payload_readout.find("GetGlobalReflectionProbeBakeFingerprint"), std::string::npos);
  const auto local_probe_tree = inspector_source.find("##EnvironmentalLightingLocalProbe");
  ASSERT_NE(local_probe_tree, std::string::npos);
  const auto payload_status_tree = inspector_source.find("ImGui::TreeNode(\"Payload status\")", local_probe_tree);
  ASSERT_NE(payload_status_tree, std::string::npos);
  const auto payload_status_call =
      inspector_source.find("InspectEnvironmentalLightingLocalProbePayload(context, probe)", payload_status_tree);
  ASSERT_NE(payload_status_call, std::string::npos);
  const auto payload_status_pop = inspector_source.find("ImGui::TreePop();", payload_status_call);
  ASSERT_NE(payload_status_pop, std::string::npos);
  EXPECT_LT(payload_status_call, payload_status_pop);
  EXPECT_NE(inspector_source.find("ImGui::TreeNode(\"Packed runtime quality\")"), std::string::npos);
  EXPECT_NE(inspector_source.find("ImGui::TreeNode(\"Cubemap face preview\")"), std::string::npos);
}

TEST(EnvironmentalLightingAsset, ResolverUsesAssignedAsset) {
  Application app;
  ApplicationContextScope scope(app);
  app.Initialize(EmptyProjectSettings());
  const auto scene = AssetManager::CreateTemporaryAsset<Scene>();
  const auto lighting = AssetManager::CreateTemporaryAsset<EnvironmentalLighting>();
  const auto fallback = AssetManager::CreateTemporaryAsset<GlobalReflectionProbe>();
  ASSERT_TRUE(scene);
  ASSERT_TRUE(lighting);
  ASSERT_TRUE(fallback);
  app.Attach(scene);
  scene->global_reflection_probe_fallback = fallback;
  scene->environmental_lighting = lighting;

  lighting->indirect_environment_source.kind = EnvironmentalLighting::IndirectEnvironmentSourceKind::Color;
  lighting->indirect_environment_source.color = glm::vec3(0.1f, 0.2f, 0.3f);
  lighting->environment_lighting_intensity = 0.25f;
  lighting->diffuse_fallback_intensity = 0.5f;
  lighting->specular_fallback_intensity = 0.75f;
  lighting->ddgi_settings.runtime.enabled = true;
  lighting->ddgi_settings.runtime.ray_count = 144;
  lighting->ddgi_settings.storage.max_probe_count = 4096;

  EnvironmentalLighting::LocalReflectionProbe high_priority;
  high_priority.stable_id = 30;
  high_priority.artist_priority = 4;
  high_priority.box_extents = glm::vec3(8.0f);
  lighting->local_reflection_probes.push_back(high_priority);
  EnvironmentalLighting::LocalReflectionProbe small_volume;
  small_volume.stable_id = 10;
  small_volume.artist_priority = 1;
  small_volume.box_extents = glm::vec3(1.0f);
  lighting->local_reflection_probes.push_back(small_volume);
  EnvironmentalLighting::LocalReflectionProbe missing_payload;
  missing_payload.stable_id = 20;
  missing_payload.artist_priority = 1;
  missing_payload.box_extents = glm::vec3(2.0f);
  lighting->local_reflection_probes.push_back(missing_payload);
  EnvironmentalLighting::LocalReflectionProbe disabled_probe;
  disabled_probe.stable_id = 99;
  disabled_probe.artist_priority = 99;
  disabled_probe.enabled = false;
  lighting->local_reflection_probes.push_back(disabled_probe);

  EnvironmentalLighting::DdgiVolume high_priority_volume;
  high_priority_volume.stable_id = 200;
  high_priority_volume.artist_priority = 4;
  high_priority_volume.probe_counts = glm::ivec3(2);
  high_priority_volume.probe_spacing = glm::vec3(8.0f);
  high_priority_volume.emissive_mesh_sampling_mode = static_cast<int>(DdgiEmissiveMeshSamplingMode::Off);
  high_priority_volume.auto_invalidate_trigger_conditions = DdgiVolumeTriggerConditionGeometryChanged;
  lighting->ddgi_volumes.push_back(high_priority_volume);
  EnvironmentalLighting::DdgiVolume dense_volume;
  dense_volume.stable_id = 100;
  dense_volume.artist_priority = 1;
  dense_volume.probe_counts = glm::ivec3(2);
  dense_volume.probe_spacing = glm::vec3(0.5f);
  dense_volume.enable_probe_classification = true;
  lighting->ddgi_volumes.push_back(dense_volume);
  EnvironmentalLighting::DdgiVolume disabled_volume;
  disabled_volume.stable_id = 999;
  disabled_volume.enabled = false;
  lighting->ddgi_volumes.push_back(disabled_volume);

  const auto resolved = ResolveEnvironmentalLighting(scene);
  EXPECT_TRUE(resolved.environmental_lighting_asset_assigned);
  EXPECT_FALSE(resolved.environmental_lighting_asset_missing);
  EXPECT_EQ(resolved.scene_global_reflection_probe_fallback.GetAssetHandle(), fallback->GetHandle());
  EXPECT_EQ(resolved.indirect_environment_source.kind,
            ResolvedEnvironmentalLighting::IndirectEnvironmentSourceKind::Color);
  EXPECT_EQ(resolved.indirect_environment_source.color, glm::vec3(0.1f, 0.2f, 0.3f));
  EXPECT_FALSE(resolved.uses_engine_default_indirect_environment_source);
  EXPECT_FLOAT_EQ(resolved.environment_lighting_intensity, 0.25f);
  EXPECT_FLOAT_EQ(resolved.diffuse_fallback_intensity, 0.5f);
  EXPECT_FLOAT_EQ(resolved.specular_fallback_intensity, 0.75f);
  EXPECT_TRUE(resolved.ddgi_settings.runtime.enabled);
  EXPECT_EQ(resolved.ddgi_settings.runtime.ray_count, 144);

  ASSERT_EQ(resolved.local_reflection_probes.size(), 3u);
  EXPECT_EQ(resolved.local_reflection_probes[0].stable_id, 30u);
  EXPECT_EQ(resolved.local_reflection_probes[1].stable_id, 10u);
  EXPECT_EQ(resolved.local_reflection_probes[2].stable_id, 20u);
  EXPECT_FALSE(ContainsStableId(resolved.local_reflection_probes, 99u));
  EXPECT_EQ(resolved.local_reflection_probes[2].global_reflection_probe.GetAssetHandle().GetValue(), 0u);

  ASSERT_EQ(resolved.ddgi_volumes.size(), 2u);
  EXPECT_EQ(resolved.ddgi_volumes[0].stable_id, 200u);
  EXPECT_EQ(resolved.ddgi_volumes[1].stable_id, 100u);
  EXPECT_EQ(resolved.ddgi_volumes[0].emissive_mesh_sampling_mode, static_cast<int>(DdgiEmissiveMeshSamplingMode::Off));
  EXPECT_EQ(resolved.ddgi_volumes[0].auto_invalidate_trigger_conditions, DdgiVolumeTriggerConditionGeometryChanged);
  EXPECT_TRUE(resolved.ddgi_volumes[1].enable_probe_classification);
}

TEST(EnvironmentalLightingAsset, ResolverDefaultsUseSceneTemporaryEnvironmentalLightingAsset) {
  Application app;
  ApplicationContextScope scope(app);
  app.Initialize(EmptyProjectSettings());
  const auto scene = AssetManager::CreateTemporaryAsset<Scene>();
  ASSERT_TRUE(scene);
  app.Attach(scene);

  const auto lighting = scene->environmental_lighting.Get<EnvironmentalLighting>();
  ASSERT_TRUE(lighting);
  EXPECT_TRUE(lighting->IsTemporary());

  const auto resolved = ResolveEnvironmentalLighting(scene);
  EXPECT_TRUE(resolved.environmental_lighting_asset_assigned);
  EXPECT_FALSE(resolved.environmental_lighting_asset_missing);
  EXPECT_EQ(resolved.indirect_environment_source.kind,
            ResolvedEnvironmentalLighting::IndirectEnvironmentSourceKind::EngineDefault);
  EXPECT_TRUE(resolved.uses_engine_default_indirect_environment_source);
  EXPECT_FLOAT_EQ(resolved.environment_lighting_intensity,
                  ResolvedEnvironmentalLighting::kDefaultEnvironmentLightingIntensity);
  EXPECT_FLOAT_EQ(resolved.diffuse_fallback_intensity, ResolvedEnvironmentalLighting::kDefaultDiffuseFallbackIntensity);
  EXPECT_FLOAT_EQ(resolved.specular_fallback_intensity,
                  ResolvedEnvironmentalLighting::kDefaultSpecularFallbackIntensity);
  EXPECT_TRUE(resolved.local_reflection_probes.empty());
  EXPECT_TRUE(resolved.ddgi_volumes.empty());
}

TEST(EnvironmentalLightingAsset, ResolverClampsInvalidFallbackIntensitiesToDefaults) {
  Application app;
  ApplicationContextScope scope(app);
  app.Initialize(EmptyProjectSettings());
  const auto scene = AssetManager::CreateTemporaryAsset<Scene>();
  const auto lighting = AssetManager::CreateTemporaryAsset<EnvironmentalLighting>();
  ASSERT_TRUE(scene);
  ASSERT_TRUE(lighting);
  scene->environmental_lighting = lighting;

  lighting->environment_lighting_intensity = std::numeric_limits<float>::quiet_NaN();
  lighting->diffuse_fallback_intensity = -1.0f;
  lighting->specular_fallback_intensity = std::numeric_limits<float>::infinity();

  const auto resolved = ResolveEnvironmentalLighting(scene);
  EXPECT_FLOAT_EQ(resolved.environment_lighting_intensity,
                  ResolvedEnvironmentalLighting::kDefaultEnvironmentLightingIntensity);
  EXPECT_FLOAT_EQ(resolved.diffuse_fallback_intensity, ResolvedEnvironmentalLighting::kDefaultDiffuseFallbackIntensity);
  EXPECT_FLOAT_EQ(resolved.specular_fallback_intensity,
                  ResolvedEnvironmentalLighting::kDefaultSpecularFallbackIntensity);
}

TEST(EnvironmentalLightingAsset, ResolverCapsAssetOwnedEntriesDeterministically) {
  Application app;
  ApplicationContextScope scope(app);
  app.Initialize(EmptyProjectSettings());
  const auto scene = AssetManager::CreateTemporaryAsset<Scene>();
  const auto lighting = AssetManager::CreateTemporaryAsset<EnvironmentalLighting>();
  ASSERT_TRUE(scene);
  ASSERT_TRUE(lighting);
  scene->environmental_lighting = lighting;
  lighting->ddgi_settings.storage.max_probe_count = 4096;

  for (uint32_t index = 0; index < ResolvedEnvironmentalLighting::kMaxLocalReflectionProbeCount + 1u; ++index) {
    EnvironmentalLighting::LocalReflectionProbe probe;
    probe.stable_id = index + 1u;
    probe.box_extents = glm::vec3(1.0f + static_cast<float>(index));
    lighting->local_reflection_probes.push_back(probe);
  }
  for (uint32_t index = 0; index < ResolvedEnvironmentalLighting::kMaxDdgiVolumeCount + 1u; ++index) {
    EnvironmentalLighting::DdgiVolume volume;
    volume.stable_id = index + 1u;
    volume.probe_counts = glm::ivec3(1);
    volume.probe_spacing = glm::vec3(1.0f + static_cast<float>(index));
    lighting->ddgi_volumes.push_back(volume);
  }

  const auto resolved = ResolveEnvironmentalLighting(scene);
  ASSERT_EQ(resolved.local_reflection_probes.size(), ResolvedEnvironmentalLighting::kMaxLocalReflectionProbeCount);
  ASSERT_EQ(resolved.ddgi_volumes.size(), ResolvedEnvironmentalLighting::kMaxDdgiVolumeCount);
  EXPECT_EQ(resolved.truncated_local_reflection_probe_count, 1u);
  EXPECT_EQ(resolved.truncated_ddgi_volume_count, 1u);
  EXPECT_EQ(resolved.local_reflection_probes.front().stable_id, 1u);
  EXPECT_EQ(resolved.local_reflection_probes.back().stable_id,
            ResolvedEnvironmentalLighting::kMaxLocalReflectionProbeCount);
  EXPECT_EQ(resolved.ddgi_volumes.front().stable_id, 1u);
  EXPECT_EQ(resolved.ddgi_volumes.back().stable_id, ResolvedEnvironmentalLighting::kMaxDdgiVolumeCount);
}
