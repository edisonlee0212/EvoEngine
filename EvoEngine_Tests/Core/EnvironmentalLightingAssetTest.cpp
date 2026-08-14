#include "EvoEngine_SDK_PCH.hpp"

#include <gtest/gtest.h>

#include "Application.hpp"
#include "ApplicationContext.hpp"
#include "ApplicationInitializationSettings.hpp"
#include "AssetManager.hpp"
#include "Camera.hpp"
#include "Cubemap.hpp"
#include "EditorLayer.hpp"
#include "EnvironmentalLighting.hpp"
#include "EnvironmentalLightingResolver.hpp"
#include "EnvironmentalMap.hpp"
#include "GlobalReflectionProbe.hpp"
#include "Scene.hpp"
#include "Serialization.hpp"
#include "Transform.hpp"

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

std::vector<uint8_t> ReadBinaryFile(const std::filesystem::path& path) {
  std::ifstream file(path, std::ios::binary);
  EXPECT_TRUE(file.good()) << path.string();
  return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}

void WriteBinaryFile(const std::filesystem::path& path, const std::vector<uint8_t>& bytes) {
  std::ofstream file(path, std::ios::binary | std::ios::trunc);
  ASSERT_TRUE(file.good()) << path.string();
  file.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
  ASSERT_TRUE(file.good()) << path.string();
}

void WriteU32(std::vector<uint8_t>& bytes, const size_t offset, const uint32_t value) {
  ASSERT_LE(offset + sizeof(value), bytes.size());
  for (uint32_t shift = 0u; shift < 32u; shift += 8u)
    bytes[offset + shift / 8u] = static_cast<uint8_t>(value >> shift);
}

void WriteU64(std::vector<uint8_t>& bytes, const size_t offset, const uint64_t value) {
  ASSERT_LE(offset + sizeof(value), bytes.size());
  for (uint32_t shift = 0u; shift < 64u; shift += 8u)
    bytes[offset + shift / 8u] = static_cast<uint8_t>(value >> shift);
}

uint64_t ReadU64(const std::vector<uint8_t>& bytes, const size_t offset) {
  EXPECT_LE(offset + sizeof(uint64_t), bytes.size());
  uint64_t value = 0u;
  for (uint32_t shift = 0u; shift < 64u; shift += 8u)
    value |= static_cast<uint64_t>(bytes[offset + shift / 8u]) << shift;
  return value;
}

void ExpectMatrixNear(const glm::mat4& actual, const glm::mat4& expected) {
  for (glm::length_t column = 0; column < 4; ++column) {
    for (glm::length_t row = 0; row < 4; ++row) {
      EXPECT_NEAR(actual[column][row], expected[column][row], 1.0e-5f);
    }
  }
}
}  // namespace

TEST(EnvironmentalLightingAsset, ReflectionProbePackBinaryRoundTripsDeterministically) {
  Application app;
  ApplicationContextScope scope(app);
  app.Initialize(EmptyProjectSettings());
  const auto pack = AssetManager::CreateTemporaryAsset<ReflectionProbePack>();
  ASSERT_TRUE(pack);
  pack->probes.reserve(2u);
  auto& valid = pack->probes.emplace_back();
  valid.name = "Gallery \xE2\x98\x85";
  valid.stable_id = 42u;
  valid.transform = glm::translate(glm::vec3(1.0f, 2.0f, 3.0f));
  valid.box_projection_extents = glm::vec3(7.0f, 2.5f, 4.0f);
  valid.sphere_radius = 9.0f;
  valid.blend_distance = 0.75f;
  valid.reflection_intensity = 1.5f;
  valid.artist_priority = -3;
  valid.shape = static_cast<int>(EnvironmentalLighting::LocalReflectionProbeShape::Sphere);
  valid.box_projection = false;
  valid.debug_draw_bounds = true;
  valid.payload = AssetManager::CreateTemporaryAsset<GlobalReflectionProbe>();
  ASSERT_TRUE(valid.payload);
  std::vector<uint16_t> canonical(GlobalReflectionProbe::kCanonicalTexelCount * 4u, 0u);
  for (size_t index = 0; index < canonical.size(); index += 4u) {
    canonical[index] = 0x3c00u;
    canonical[index + 1u] = 0x3800u;
    canonical[index + 2u] = 0x3400u;
    canonical[index + 3u] = 0x3c00u;
  }
  ASSERT_TRUE(valid.payload->SetCanonicalPayload(canonical));
  auto& invalid = pack->probes.emplace_back();
  invalid.name = "Unbaked";
  invalid.stable_id = 7u;
  invalid.enabled = false;

  const auto directory = std::filesystem::temp_directory_path() / "evoengine-reflection-probe-pack-test";
  std::filesystem::create_directories(directory);
  const auto first_path = directory / "first.evereflectionprobepack";
  const auto second_path = directory / "second.evereflectionprobepack";
  ASSERT_TRUE(pack->Export(first_path));
  ASSERT_TRUE(pack->Export(second_path));
  EXPECT_EQ(ReadBinaryFile(first_path), ReadBinaryFile(second_path));

  const auto restored = AssetManager::CreateTemporaryAsset<ReflectionProbePack>();
  ASSERT_TRUE(restored);
  ASSERT_TRUE(restored->Import(first_path));
  ASSERT_EQ(restored->probes.size(), 2u);
  const auto& restored_valid = restored->probes[0];
  EXPECT_EQ(restored_valid.name, valid.name);
  EXPECT_EQ(restored_valid.stable_id, valid.stable_id);
  ExpectMatrixNear(restored_valid.transform, valid.transform);
  EXPECT_EQ(restored_valid.box_projection_extents, valid.box_projection_extents);
  EXPECT_FLOAT_EQ(restored_valid.sphere_radius, valid.sphere_radius);
  EXPECT_FLOAT_EQ(restored_valid.blend_distance, valid.blend_distance);
  EXPECT_FLOAT_EQ(restored_valid.reflection_intensity, valid.reflection_intensity);
  EXPECT_EQ(restored_valid.artist_priority, valid.artist_priority);
  EXPECT_EQ(restored_valid.shape, valid.shape);
  EXPECT_FALSE(restored_valid.box_projection);
  EXPECT_TRUE(restored_valid.debug_draw_bounds);
  ASSERT_TRUE(restored_valid.payload);
  EXPECT_EQ(restored_valid.payload->GetPayloadHash(), GlobalReflectionProbe::CalculatePayloadHash(canonical));
  EXPECT_EQ(restored_valid.payload->GetCanonicalPayload(), canonical);
  EXPECT_EQ(restored->probes[1].name, "Unbaked");
  EXPECT_FALSE(restored->probes[1].payload);
  std::filesystem::remove_all(directory);
}

TEST(EnvironmentalLightingAsset, ReflectionProbePackRejectsCorruptionWithoutReplacingLiveState) {
  Application app;
  ApplicationContextScope scope(app);
  app.Initialize(EmptyProjectSettings());
  const auto source = AssetManager::CreateTemporaryAsset<ReflectionProbePack>();
  ASSERT_TRUE(source);
  source->probes.reserve(2u);
  auto& first = source->probes.emplace_back();
  first.name = "First";
  first.stable_id = 1u;
  first.payload = AssetManager::CreateTemporaryAsset<GlobalReflectionProbe>();
  ASSERT_TRUE(first.payload);
  std::vector<uint16_t> canonical(GlobalReflectionProbe::kCanonicalTexelCount * 4u, 0x3c00u);
  ASSERT_TRUE(first.payload->SetCanonicalPayload(canonical));
  auto& second = source->probes.emplace_back();
  second.name = "Second";
  second.stable_id = 2u;

  const auto directory = std::filesystem::temp_directory_path() / "evoengine-reflection-probe-pack-corruption-test";
  std::filesystem::create_directories(directory);
  const auto valid_path = directory / "valid.evereflectionprobepack";
  const auto invalid_path = directory / "invalid.evereflectionprobepack";
  ASSERT_TRUE(source->Export(valid_path));
  const auto valid_bytes = ReadBinaryFile(valid_path);
  const auto live = AssetManager::CreateTemporaryAsset<ReflectionProbePack>();
  ASSERT_TRUE(live);
  ASSERT_TRUE(live->Import(valid_path));

  const auto reject = [&](std::vector<uint8_t> bytes) {
    WriteBinaryFile(invalid_path, bytes);
    EXPECT_FALSE(live->Import(invalid_path));
    ASSERT_EQ(live->probes.size(), 2u);
    EXPECT_EQ(live->probes[0].name, "First");
    EXPECT_EQ(live->probes[0].stable_id, 1u);
    EXPECT_EQ(live->probes[0].payload->GetPayloadHash(), first.payload->GetPayloadHash());
  };

  auto bytes = valid_bytes;
  bytes.resize(bytes.size() - 1u);
  reject(bytes);
  bytes = valid_bytes;
  WriteU32(bytes, 8u, ReflectionProbePack::kSchemaVersion + 1u);
  reject(bytes);
  bytes = valid_bytes;
  WriteU32(bytes, 24u, (std::numeric_limits<uint32_t>::max)());
  reject(bytes);
  bytes = valid_bytes;
  WriteU64(bytes, 28u + 152u, 1u);
  reject(bytes);
  bytes = valid_bytes;
  WriteU64(bytes, 28u + 8u, (std::numeric_limits<uint64_t>::max)());
  reject(bytes);
  bytes = valid_bytes;
  WriteU64(bytes, 28u + 32u, 2u);
  reject(bytes);
  bytes = valid_bytes;
  WriteU64(bytes, 28u + 40u, ReadU64(valid_bytes, 28u + 40u) ^ 1u);
  reject(bytes);
  bytes = valid_bytes;
  WriteU32(bytes, 28u + 56u, 0x7f800000u);
  reject(bytes);
  bytes = valid_bytes;
  WriteU64(bytes, 28u + 152u + 8u, ReadU64(valid_bytes, 28u + 8u));
  reject(bytes);
  bytes = valid_bytes;
  bytes.push_back(0u);
  reject(bytes);
  std::filesystem::remove_all(directory);
}

TEST(EnvironmentalLightingAsset, DdgiVolumePackYamlRoundTripsClampsRepairsAndAllowsEmptyPacks) {
  Application app;
  ApplicationContextScope scope(app);
  app.Initialize(EmptyProjectSettings());
  DdgiVolumePack source;
  source.volumes.resize(2u);
  source.volumes[0].name = "Primary";
  source.volumes[0].stable_id = 9u;
  source.volumes[0].probe_spacing = glm::vec3(-1.0f, 0.01f, 20000.0f);
  source.volumes[0].random_ray_backface_threshold = -2.0f;
  source.volumes[1].name = "Duplicate";
  source.volumes[1].stable_id = 9u;
  YAML::Emitter out;
  out << YAML::BeginMap;
  SerializeDdgiVolumePack(out, source);
  out << YAML::EndMap;

  DdgiVolumePack restored;
  DeserializeDdgiVolumePack(YAML::Load(out.c_str()), restored);
  ASSERT_EQ(restored.volumes.size(), 2u);
  EXPECT_EQ(restored.volumes[0].stable_id, 9u);
  EXPECT_EQ(restored.volumes[1].stable_id, 1u);
  EXPECT_EQ(restored.volumes[0].probe_spacing, glm::vec3(0.05f, 0.05f, 10000.0f));
  EXPECT_FLOAT_EQ(restored.volumes[0].random_ray_backface_threshold, 0.0f);

  DdgiVolumePack empty;
  DeserializeDdgiVolumePack(YAML::Load("{}"), empty);
  EXPECT_TRUE(empty.volumes.empty());
}

TEST(EnvironmentalLightingAsset, AuthoringTransformDecomposesAndRecomposesTrs) {
  const glm::vec3 position(3.0f, -2.0f, 7.0f);
  const glm::vec3 rotation_degrees(20.0f, -35.0f, 70.0f);
  const glm::vec3 scale(1.5f, 0.75f, 2.25f);
  const auto expected =
      glm::translate(position) * glm::mat4_cast(glm::quat(glm::radians(rotation_degrees))) * glm::scale(scale);
  Transform transform;
  transform.value = expected;

  glm::vec3 decomposed_position;
  glm::vec3 decomposed_rotation;
  glm::vec3 decomposed_scale;
  ASSERT_TRUE(transform.Decompose(decomposed_position, decomposed_rotation, decomposed_scale));
  const auto recomposed = glm::translate(decomposed_position) * glm::mat4_cast(glm::quat(decomposed_rotation)) *
                          glm::scale(decomposed_scale);

  ExpectMatrixNear(recomposed, expected);
}

TEST(EnvironmentalLightingAsset, EditorAuthoringTransformNormalizesTrsAndRejectsInvalidMatrices) {
  const auto transform = EditorLayer::ComposeAuthoringTransform(
      glm::vec3(3.0f, -2.0f, 7.0f), glm::vec3(20.0f, -35.0f, 70.0f), glm::vec3(1.5f, 0.75f, 2.25f));
  glm::mat4 normalized(1.0f);
  glm::vec3 position(0.0f);
  glm::vec3 rotation_degrees(0.0f);
  glm::vec3 scale(1.0f);
  ASSERT_TRUE(EditorLayer::TryNormalizeAuthoringTransform(transform, normalized, position, rotation_degrees, scale));
  ExpectMatrixNear(normalized, transform);

  auto sheared = transform;
  sheared[1][0] += 0.25f;
  ASSERT_TRUE(EditorLayer::TryNormalizeAuthoringTransform(sheared, normalized, position, rotation_degrees, scale));
  EXPECT_GT(glm::length(glm::vec4(sheared[1] - normalized[1])), 1.0e-4f);

  auto singular = transform;
  singular[0] = glm::vec4(0.0f);
  EXPECT_FALSE(EditorLayer::TryNormalizeAuthoringTransform(singular, normalized, position, rotation_degrees, scale));
  auto non_finite = transform;
  non_finite[3][0] = std::numeric_limits<float>::infinity();
  EXPECT_FALSE(EditorLayer::TryNormalizeAuthoringTransform(non_finite, normalized, position, rotation_degrees, scale));
}

TEST(EnvironmentalLightingAsset, EditorGizmoConversionUsesAndPreservesDdgiGridCenterPivot) {
  const glm::vec3 pivot(2.0f, -1.0f, 4.0f);
  const auto stored = EditorLayer::ComposeAuthoringTransform(
      glm::vec3(-3.0f, 5.0f, 7.0f), glm::vec3(10.0f, 25.0f, -15.0f), glm::vec3(0.5f, 2.0f, 1.25f));
  const auto gizmo = EditorLayer::CreateAuthoringGizmoTransform(stored, pivot);
  ExpectMatrixNear(gizmo, stored * glm::translate(pivot));

  const glm::vec3 edited_center = glm::vec3(gizmo[3]) + glm::vec3(4.0f, -2.0f, 1.0f);
  const auto edited_gizmo = EditorLayer::ComposeAuthoringTransform(edited_center, glm::vec3(-20.0f, 50.0f, 5.0f),
                                                                   glm::vec3(1.75f, 0.8f, 2.5f));
  glm::mat4 converted(1.0f);
  ASSERT_TRUE(EditorLayer::TryConvertAuthoringGizmoTransform(edited_gizmo, pivot, converted));
  ExpectMatrixNear(EditorLayer::CreateAuthoringGizmoTransform(converted, pivot), edited_gizmo);

  glm::mat4 reflection_converted(1.0f);
  ASSERT_TRUE(EditorLayer::TryConvertAuthoringGizmoTransform(edited_gizmo, glm::vec3(0.0f), reflection_converted));
  ExpectMatrixNear(reflection_converted, edited_gizmo);

  auto singular_gizmo = edited_gizmo;
  singular_gizmo[2] = glm::vec4(0.0f);
  EXPECT_FALSE(EditorLayer::TryConvertAuthoringGizmoTransform(singular_gizmo, pivot, converted));
}

TEST(EnvironmentalLightingAsset, EnvironmentalLightingGizmoTargetIsTransientAndExclusiveWithEntitySelection) {
  Application app;
  ApplicationContextScope scope(app);
  const auto editor_layer = app.PushLayer<EditorLayer>("Editor Layer");
  const auto lighting = std::make_shared<EnvironmentalLighting>();
  ASSERT_TRUE(editor_layer);
  ASSERT_TRUE(lighting);

  editor_layer->SetEnvironmentalLightingGizmoTarget(
      lighting, EnvironmentalLightingGizmoTargetType::LocalReflectionProbe, 0u, 17u);
  EXPECT_TRUE(editor_layer->IsEnvironmentalLightingGizmoTarget(
      *lighting, EnvironmentalLightingGizmoTargetType::LocalReflectionProbe, 0u, 17u));
  EXPECT_FALSE(editor_layer->IsEnvironmentalLightingGizmoTarget(
      *lighting, EnvironmentalLightingGizmoTargetType::DdgiVolume, 0u, 23u));

  editor_layer->SetSelectedEntity({});
  EXPECT_FALSE(editor_layer->IsEnvironmentalLightingGizmoTarget(
      *lighting, EnvironmentalLightingGizmoTargetType::LocalReflectionProbe, 0u, 17u));

  editor_layer->SetEnvironmentalLightingGizmoTarget(lighting, EnvironmentalLightingGizmoTargetType::DdgiVolume, 0u,
                                                    23u);
  editor_layer->ClearEnvironmentalLightingGizmoTarget(lighting->GetHandle());
  EXPECT_FALSE(editor_layer->IsEnvironmentalLightingGizmoTarget(
      *lighting, EnvironmentalLightingGizmoTargetType::DdgiVolume, 0u, 23u));
}

TEST(EnvironmentalLightingAsset, RepairsMissingAndDuplicateStableIdsDeterministically) {
  Application app;
  ApplicationContextScope scope(app);
  app.Initialize(EmptyProjectSettings());
  EnvironmentalLighting lighting;
  const auto reflection_pack = lighting.GetOrCreateReflectionProbePack();
  const auto ddgi_pack = lighting.GetOrCreateDdgiVolumePack();
  reflection_pack->probes.resize(3u);
  reflection_pack->probes[1].stable_id = 7u;
  reflection_pack->probes[2].stable_id = 7u;
  ddgi_pack->volumes.resize(3u);
  ddgi_pack->volumes[1].stable_id = 9u;
  ddgi_pack->volumes[2].stable_id = 9u;

  EXPECT_TRUE(reflection_pack->RepairStableIds());
  EXPECT_TRUE(ddgi_pack->RepairStableIds());
  EXPECT_EQ(reflection_pack->probes[0].stable_id, 1u);
  EXPECT_EQ(reflection_pack->probes[1].stable_id, 7u);
  EXPECT_EQ(reflection_pack->probes[2].stable_id, 2u);
  EXPECT_EQ(ddgi_pack->volumes[0].stable_id, 1u);
  EXPECT_EQ(ddgi_pack->volumes[1].stable_id, 9u);
  EXPECT_EQ(ddgi_pack->volumes[2].stable_id, 2u);
  EXPECT_FALSE(reflection_pack->RepairStableIds());
  EXPECT_FALSE(ddgi_pack->RepairStableIds());

  const auto legacy_source = YAML::Load(R"(
ddgi_volumes:
  - stable_id: 0
  - stable_id: 9
  - stable_id: 9
)");
  EnvironmentalLighting legacy_lighting;
  DeserializeEnvironmentalLighting(legacy_source, legacy_lighting);
  EXPECT_TRUE(legacy_lighting.GetOrCreateDdgiVolumePack()->volumes.empty());
}

TEST(EnvironmentalLightingAsset, LocalTransformGizmoOperationSelectionIsExclusive) {
  Application app;
  ApplicationContextScope scope(app);
  const auto editor_layer = app.PushLayer<EditorLayer>("Editor Layer");
  ASSERT_TRUE(editor_layer);

  editor_layer->SelectLocalTransformGizmoOperation(LocalTransformGizmoOperation::Rotate);
  EXPECT_FALSE(editor_layer->LocalPositionSelected());
  EXPECT_TRUE(editor_layer->LocalRotationSelected());
  EXPECT_FALSE(editor_layer->LocalScaleSelected());

  editor_layer->SelectLocalTransformGizmoOperation(LocalTransformGizmoOperation::Scale);
  EXPECT_FALSE(editor_layer->LocalPositionSelected());
  EXPECT_FALSE(editor_layer->LocalRotationSelected());
  EXPECT_TRUE(editor_layer->LocalScaleSelected());

  editor_layer->SelectLocalTransformGizmoOperation(LocalTransformGizmoOperation::Translate);
  EXPECT_TRUE(editor_layer->LocalPositionSelected());
  EXPECT_FALSE(editor_layer->LocalRotationSelected());
  EXPECT_FALSE(editor_layer->LocalScaleSelected());
}

TEST(EnvironmentalLightingAsset, SerializesCompleteAuthoringSetup) {
  Application app;
  ApplicationContextScope scope(app);
  app.Initialize(EmptyProjectSettings());
  const auto lighting = AssetManager::CreateTemporaryAsset<EnvironmentalLighting>();
  const auto environment = AssetManager::CreateTemporaryAsset<EnvironmentalMap>();
  const auto bake_cubemap = AssetManager::CreateTemporaryAsset<Cubemap>();
  const auto local_probe_payload = AssetManager::CreateTemporaryAsset<GlobalReflectionProbe>();
  ASSERT_TRUE(lighting);
  ASSERT_TRUE(environment);
  ASSERT_TRUE(bake_cubemap);
  ASSERT_TRUE(local_probe_payload);

  lighting->indirect_environment_source.kind = EnvironmentalLighting::IndirectEnvironmentSourceKind::EnvironmentalMap;
  lighting->indirect_environment_source.environmental_map = environment;
  lighting->indirect_environment_source.color = glm::vec3(0.2f, 0.4f, 0.6f);
  lighting->indirect_environment_source.gamma = 1.8f;
  lighting->indirect_environment_source.rotation = 0.25f;
  lighting->reflection_probe_bake_background.source = CameraSettings::BackgroundSource::Cubemap;
  lighting->reflection_probe_bake_background.cubemap = bake_cubemap;
  lighting->reflection_probe_bake_background.environmental_map = environment;
  lighting->reflection_probe_bake_background.clear_color = glm::vec4(0.6f, 0.4f, 0.2f, 1.0f);
  lighting->dynamic_reflection_probe_settings.enabled = true;
  lighting->dynamic_reflection_probe_settings.faces_per_frame = 4;
  lighting->environment_lighting_intensity = 0.35f;
  lighting->diffuse_fallback_intensity = 0.45f;
  lighting->specular_fallback_intensity = 0.55f;
  lighting->ddgi_settings.runtime.enabled = true;
  lighting->ddgi_settings.runtime.ray_count = 96;
  lighting->ddgi_settings.volume_defaults.probe_counts = glm::ivec3(4, 5, 6);
  lighting->local_reflection_probes_enabled = false;

  EnvironmentalLighting::LocalReflectionProbe probe;
  probe.name = "Gallery Probe";
  probe.stable_id = 101;
  probe.payload = local_probe_payload;
  const auto probe_transform = glm::translate(glm::vec3(1.0f, 2.0f, 3.0f)) *
                               glm::mat4_cast(glm::quat(glm::radians(glm::vec3(15.0f, -25.0f, 40.0f)))) *
                               glm::scale(glm::vec3(1.5f, 0.75f, 2.0f));
  probe.transform = probe_transform;
  probe.box_projection_extents = glm::vec3(6.0f, 2.5f, 1.5f);
  probe.sphere_radius = 8.0f;
  probe.blend_distance = 0.75f;
  probe.reflection_intensity = 1.25f;
  probe.artist_priority = 4;
  probe.shape = static_cast<int>(EnvironmentalLighting::LocalReflectionProbeShape::Sphere);
  probe.box_projection = false;
  probe.debug_draw_bounds = true;
  const auto reflection_pack = lighting->GetOrCreateReflectionProbePack();
  reflection_pack->probes.push_back(probe);

  EnvironmentalLighting::DdgiVolume volume;
  volume.name = "Gallery DDGI";
  volume.stable_id = 202;
  const auto volume_transform = glm::translate(glm::vec3(-2.0f, 1.0f, 5.0f)) *
                                glm::mat4_cast(glm::quat(glm::radians(glm::vec3(-10.0f, 35.0f, 5.0f)))) *
                                glm::scale(glm::vec3(0.5f, 1.25f, 1.75f));
  volume.transform = volume_transform;
  volume.probe_counts = glm::ivec3(3, 4, 5);
  volume.probe_spacing = glm::vec3(1.0f, 1.5f, 2.0f);
  volume.volume_origin = glm::vec3(-1.0f, 0.5f, 2.0f);
  volume.artist_priority = 7;
  volume.movement_type = static_cast<int>(DdgiVolumeMovementType::Scrolling);
  volume.emissive_mesh_sampling_mode = static_cast<int>(DdgiEmissiveMeshSamplingMode::Off);
  volume.enable_probe_relocation = false;
  volume.enable_probe_classification = true;
  volume.pause_probe_updates_after_convergence = false;
  volume.relocation_distance = 0.5f;
  volume.hysteresis_boost_trigger_conditions = DdgiVolumeTriggerConditionAll;
  const auto ddgi_pack = lighting->GetOrCreateDdgiVolumePack();
  ddgi_pack->volumes.push_back(volume);

  YAML::Emitter out;
  out << YAML::BeginMap;
  Serialization::SerializeObject(out, static_cast<IAsset&>(*lighting));
  out << YAML::EndMap;
  const auto node = YAML::Load(out.c_str());
  EXPECT_EQ(node["indirect_environment_source"]["kind"].as<uint32_t>(),
            static_cast<uint32_t>(EnvironmentalLighting::IndirectEnvironmentSourceKind::EnvironmentalMap));
  EXPECT_EQ(node["indirect_environment_source"]["environmental_map"]["asset_handle_"].as<uint64_t>(),
            environment->GetHandle().GetValue());
  EXPECT_EQ(node["reflection_probe_bake_background"]["source"].as<std::string>(), "Cubemap");
  EXPECT_FALSE(node["reflection_probe_bake_background"]["intensity"]);
  EXPECT_EQ(node["reflection_probe_bake_background"]["cubemap"]["asset_handle_"].as<uint64_t>(),
            bake_cubemap->GetHandle().GetValue());
  EXPECT_TRUE(node["dynamic_reflection_probe_settings"]["enabled"].as<bool>());
  EXPECT_FALSE(node["dynamic_reflection_probe_settings"]["update_policy"]);
  EXPECT_EQ(node["dynamic_reflection_probe_settings"]["faces_per_frame"].as<int>(), 4);
  EXPECT_FLOAT_EQ(node["environment_lighting_intensity"].as<float>(), 0.35f);
  EXPECT_FLOAT_EQ(node["diffuse_fallback_intensity"].as<float>(), 0.45f);
  EXPECT_FLOAT_EQ(node["specular_fallback_intensity"].as<float>(), 0.55f);
  EXPECT_EQ(node["ddgi_settings"]["runtime"]["ray_count"].as<int>(), 96);
  EXPECT_FALSE(node["local_reflection_probes_enabled"].as<bool>());
  EXPECT_FALSE(node["local_reflection_probes"]);
  EXPECT_FALSE(node["ddgi_volumes"]);
  EXPECT_EQ(node["reflection_probe_pack"]["asset_handle_"].as<uint64_t>(), reflection_pack->GetHandle().GetValue());
  EXPECT_EQ(node["ddgi_volume_pack"]["asset_handle_"].as<uint64_t>(), ddgi_pack->GetHandle().GetValue());

  std::vector<AssetRef> refs;
  Serialization::CollectAssetRefs(static_cast<IAsset&>(*lighting), refs);
  EXPECT_TRUE(ContainsAssetHandle(refs, environment->GetHandle()));
  EXPECT_TRUE(ContainsAssetHandle(refs, bake_cubemap->GetHandle()));
  EXPECT_TRUE(ContainsAssetHandle(refs, reflection_pack->GetHandle()));
  EXPECT_TRUE(ContainsAssetHandle(refs, ddgi_pack->GetHandle()));
  EXPECT_FALSE(ContainsAssetHandle(refs, local_probe_payload->GetHandle()));

  EnvironmentalLighting restored;
  Serialization::DeserializeObject(node, static_cast<IAsset&>(restored));
  EXPECT_EQ(restored.indirect_environment_source.kind,
            EnvironmentalLighting::IndirectEnvironmentSourceKind::EnvironmentalMap);
  EXPECT_EQ(restored.indirect_environment_source.environmental_map.GetAssetHandle(), environment->GetHandle());
  EXPECT_FLOAT_EQ(restored.indirect_environment_source.gamma, 1.8f);
  EXPECT_EQ(restored.reflection_probe_bake_background.source, CameraSettings::BackgroundSource::Cubemap);
  EXPECT_EQ(restored.reflection_probe_bake_background.cubemap.GetAssetHandle(), bake_cubemap->GetHandle());
  EXPECT_EQ(restored.reflection_probe_bake_background.environmental_map.GetAssetHandle(), environment->GetHandle());
  EXPECT_EQ(restored.reflection_probe_bake_background.clear_color, glm::vec4(0.6f, 0.4f, 0.2f, 1.0f));
  EXPECT_TRUE(restored.dynamic_reflection_probe_settings.enabled);
  EXPECT_EQ(restored.dynamic_reflection_probe_settings.faces_per_frame, 4);
  EXPECT_FLOAT_EQ(restored.environment_lighting_intensity, 0.35f);
  EXPECT_FLOAT_EQ(restored.diffuse_fallback_intensity, 0.45f);
  EXPECT_FLOAT_EQ(restored.specular_fallback_intensity, 0.55f);
  EXPECT_TRUE(restored.ddgi_settings.runtime.enabled);
  EXPECT_EQ(restored.ddgi_settings.runtime.ray_count, 96);
  EXPECT_FALSE(restored.local_reflection_probes_enabled);
  ASSERT_EQ(restored.GetReflectionProbePack(), reflection_pack);
  ASSERT_EQ(restored.GetDdgiVolumePack(), ddgi_pack);
  EXPECT_EQ(reflection_pack->probes.front().payload, local_probe_payload);
  EXPECT_EQ(reflection_pack->probes.front().shape,
            static_cast<int>(EnvironmentalLighting::LocalReflectionProbeShape::Sphere));
  EXPECT_FALSE(reflection_pack->probes.front().box_projection);
  EXPECT_TRUE(reflection_pack->probes.front().debug_draw_bounds);
  ExpectMatrixNear(reflection_pack->probes.front().transform, probe_transform);
  EXPECT_EQ(ddgi_pack->volumes.front().probe_counts, glm::ivec3(3, 4, 5));
  ExpectMatrixNear(ddgi_pack->volumes.front().transform, volume_transform);
  EXPECT_EQ(ddgi_pack->volumes.front().movement_type, static_cast<int>(DdgiVolumeMovementType::Scrolling));
  EXPECT_TRUE(ddgi_pack->volumes.front().enable_probe_classification);
  EXPECT_FALSE(ddgi_pack->volumes.front().pause_probe_updates_after_convergence);
  EXPECT_EQ(ddgi_pack->volumes.front().hysteresis_boost_trigger_conditions, DdgiVolumeTriggerConditionAll);
}

TEST(EnvironmentalLightingAsset, LegacyInlineDdgiVolumesAreIgnored) {
  Application app;
  ApplicationContextScope scope(app);
  app.Initialize(EmptyProjectSettings());
  EnvironmentalLighting lighting;
  const auto legacy = YAML::Load(R"(
ddgi_volumes:
  - name: Legacy auto only
    auto_invalidate_trigger_conditions: 5
  - name: Legacy and warmup
    auto_invalidate_trigger_conditions: 1
    warmup_trigger_conditions: 2
  - name: Previous scene hysteresis
    scene_change_hysteresis_trigger_conditions: 2
  - name: Current and previous
    hysteresis_boost_trigger_conditions: 1
    scene_change_hysteresis_trigger_conditions: 4
)");
  DeserializeEnvironmentalLighting(legacy, lighting);

  EXPECT_TRUE(lighting.GetOrCreateDdgiVolumePack()->volumes.empty());
}

TEST(EnvironmentalLightingAsset, DynamicReflectionProbeSettingsDefaultClampResolveAndIgnoreLegacyPolicy) {
  Application app;
  ApplicationContextScope scope(app);
  app.Initialize(EmptyProjectSettings());
  EnvironmentalLighting lighting;
  EXPECT_TRUE(lighting.dynamic_reflection_probe_settings.enabled);
  EXPECT_EQ(lighting.dynamic_reflection_probe_settings.faces_per_frame, 6);

  const auto missing = YAML::Load("{}");
  DeserializeEnvironmentalLighting(missing, lighting);
  EXPECT_TRUE(lighting.dynamic_reflection_probe_settings.enabled);
  EXPECT_EQ(lighting.dynamic_reflection_probe_settings.faces_per_frame, 6);

  const auto malformed = YAML::Load(R"(
dynamic_reflection_probe_settings:
  enabled: true
  update_policy: 99
  faces_per_frame: 42
)");
  DeserializeEnvironmentalLighting(malformed, lighting);
  EXPECT_TRUE(lighting.dynamic_reflection_probe_settings.enabled);
  EXPECT_EQ(lighting.dynamic_reflection_probe_settings.faces_per_frame, 6);

  const auto scene = AssetManager::CreateTemporaryAsset<Scene>();
  const auto asset = AssetManager::CreateTemporaryAsset<EnvironmentalLighting>();
  ASSERT_TRUE(scene);
  ASSERT_TRUE(asset);
  asset->dynamic_reflection_probe_settings = lighting.dynamic_reflection_probe_settings;
  scene->environmental_lighting = asset;
  const auto resolved = ResolveEnvironmentalLighting(scene);
  EXPECT_TRUE(resolved.dynamic_reflection_probe_settings.enabled);
  EXPECT_EQ(resolved.dynamic_reflection_probe_settings.faces_per_frame, 6u);
}

TEST(EnvironmentalLightingAsset, ReflectionProbeBakeBackgroundRoundTripsEveryCameraSource) {
  Application app;
  ApplicationContextScope scope(app);
  app.Initialize(EmptyProjectSettings());
  for (uint32_t source_index = 0; source_index < Camera::kBackgroundSourceCount; ++source_index) {
    EnvironmentalLighting lighting;
    lighting.reflection_probe_bake_background.source = Camera::NormalizeBackgroundSource(source_index);
    YAML::Emitter out;
    out << YAML::BeginMap;
    SerializeEnvironmentalLighting(out, lighting);
    out << YAML::EndMap;

    EnvironmentalLighting restored;
    DeserializeEnvironmentalLighting(YAML::Load(out.c_str()), restored);
    EXPECT_EQ(restored.reflection_probe_bake_background.source, Camera::NormalizeBackgroundSource(source_index));
  }
}

TEST(EnvironmentalLightingAsset, ReflectionProbeBakeBackgroundIgnoresLegacyIntensity) {
  Application app;
  ApplicationContextScope scope(app);
  app.Initialize(EmptyProjectSettings());
  EnvironmentalLighting lighting;
  DeserializeEnvironmentalLighting(YAML::Load(R"(
reflection_probe_bake_background:
  source: Clear Color
  intensity: 4.5
  clear_color: [0.1, 0.2, 0.3, 1.0]
)"),
                                   lighting);
  EXPECT_EQ(lighting.reflection_probe_bake_background.source, CameraSettings::BackgroundSource::ClearColor);
  EXPECT_EQ(lighting.reflection_probe_bake_background.clear_color, glm::vec4(0.1f, 0.2f, 0.3f, 1.0f));

  YAML::Emitter out;
  out << YAML::BeginMap;
  SerializeEnvironmentalLighting(out, lighting);
  out << YAML::EndMap;
  EXPECT_FALSE(YAML::Load(out.c_str())["reflection_probe_bake_background"]["intensity"]);
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
  EXPECT_TRUE(resolved.dynamic_reflection_probe_settings.enabled);
  EXPECT_EQ(resolved.dynamic_reflection_probe_settings.faces_per_frame, 6u);
  EXPECT_EQ(created_lighting->reflection_probe_bake_background.source,
            CameraSettings::BackgroundSource::InheritEnvironmentalLighting);
  EXPECT_TRUE(created_lighting->local_reflection_probes_enabled);

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
  EXPECT_NE(asset_header.find("AssetRef reflection_probe_pack"), std::string::npos);
  EXPECT_NE(asset_header.find("AssetRef ddgi_volume_pack"), std::string::npos);
  EXPECT_NE(asset_header.find("environment_lighting_intensity"), std::string::npos);
  EXPECT_NE(asset_header.find("diffuse_fallback_intensity"), std::string::npos);
  EXPECT_NE(asset_header.find("specular_fallback_intensity"), std::string::npos);
  EXPECT_NE(asset_header.find("ReflectionProbeBakeBackground"), std::string::npos);
  EXPECT_NE(asset_header.find("local_reflection_probes_enabled"), std::string::npos);
  EXPECT_NE(application_source.find(".eveenvironmentallighting"), std::string::npos);
  EXPECT_NE(inspector_source.find("InspectEnvironmentalLighting"), std::string::npos);
  EXPECT_EQ(inspector_source.find("#include \"EnvironmentalLightingResolver.hpp\""), std::string::npos);
  EXPECT_EQ(inspector_source.find("#include \"ProjectManager.hpp\""), std::string::npos);
  EXPECT_EQ(inspector_source.find("Extract Environmental Lighting Asset"), std::string::npos);
  EXPECT_EQ(inspector_source.find("ExtractEnvironmentalLightingFromLegacyScene"), std::string::npos);
  EXPECT_EQ(inspector_source.find("ProjectManager::SaveAsset"), std::string::npos);
  EXPECT_NE(inspector_source.find("RenderEnvironmentalLightingProbeBounds"), std::string::npos);
  EXPECT_NE(inspector_source.find("RenderEnvironmentalLightingDebugProbeBounds"), std::string::npos);
  EXPECT_NE(inspector_source.find("scene->environmental_lighting.Get<EnvironmentalLighting>()"), std::string::npos);
  EXPECT_NE(inspector_source.find("BeginTabItem(\"DDGI\")"), std::string::npos);
  EXPECT_NE(inspector_source.find("TreeNodeEx(\"Settings\""), std::string::npos);
  EXPECT_NE(inspector_source.find("TreeNodeEx(\"Volumes\""), std::string::npos);
  EXPECT_NE(inspector_source.find("InspectDdgiRuntimeControls(runtime)"), std::string::npos);
  EXPECT_EQ(inspector_source.find("Invalidate history now"), std::string::npos);
  EXPECT_NE(inspector_source.find("Reset history"), std::string::npos);
  EXPECT_EQ(inspector_source.find("Bake Stale Local Probe Payloads"), std::string::npos);
  EXPECT_NE(inspector_source.find("Bake Local Probe Payload"), std::string::npos);
  EXPECT_NE(inspector_source.find("Bake All Local Probe Payloads"), std::string::npos);
  EXPECT_NE(inspector_source.find("Debug draw bounds"), std::string::npos);
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
  EXPECT_NE(render_layer_source.find("reflection_probe_bake_queue_.push_back"), std::string::npos);
  EXPECT_NE(render_layer_source.find("RecordPreparedReflectionProbeBake(current_render_instances)"), std::string::npos);
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
  EXPECT_NE(payload_readout.find("const auto& payload = probe.payload"), std::string::npos);
  EXPECT_NE(payload_readout.find("Bake Local Probe Payload"), std::string::npos);
  EXPECT_EQ(payload_readout.find("Load / Inspect Probe Payload"), std::string::npos);
  EXPECT_EQ(payload_readout.find(".Get<GlobalReflectionProbe>()"), std::string::npos);
  EXPECT_EQ(payload_readout.find("Stale check:"), std::string::npos);
  EXPECT_EQ(payload_readout.find("GetGlobalReflectionProbeBakeFingerprint"), std::string::npos);
  const auto local_probe_tree = inspector_source.find("##EnvironmentalLightingLocalProbe");
  ASSERT_NE(local_probe_tree, std::string::npos);
  const auto payload_status_tree = inspector_source.find("ImGui::TreeNode(\"Payload status\")", local_probe_tree);
  ASSERT_NE(payload_status_tree, std::string::npos);
  const auto payload_status_call = inspector_source.find(
      "InspectEnvironmentalLightingLocalProbePayload(context, reflection_pack, probe, !dynamic_settings.enabled)",
      payload_status_tree);
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
  high_priority.transform = glm::scale(glm::vec3(16.0f));
  const auto reflection_pack = lighting->GetOrCreateReflectionProbePack();
  const auto ddgi_pack = lighting->GetOrCreateDdgiVolumePack();
  reflection_pack->probes.push_back(high_priority);
  EnvironmentalLighting::LocalReflectionProbe small_volume;
  small_volume.stable_id = 10;
  small_volume.artist_priority = 1;
  small_volume.transform = glm::scale(glm::vec3(2.0f));
  small_volume.blend_distance = 0.75f;
  reflection_pack->probes.push_back(small_volume);
  EnvironmentalLighting::LocalReflectionProbe missing_payload;
  missing_payload.stable_id = 20;
  missing_payload.artist_priority = 1;
  missing_payload.transform = glm::scale(glm::vec3(4.0f));
  reflection_pack->probes.push_back(missing_payload);
  EnvironmentalLighting::LocalReflectionProbe disabled_probe;
  disabled_probe.stable_id = 99;
  disabled_probe.artist_priority = 99;
  disabled_probe.enabled = false;
  reflection_pack->probes.push_back(disabled_probe);

  EnvironmentalLighting::DdgiVolume high_priority_volume;
  high_priority_volume.stable_id = 200;
  high_priority_volume.artist_priority = 4;
  high_priority_volume.probe_counts = glm::ivec3(2);
  high_priority_volume.probe_spacing = glm::vec3(8.0f);
  high_priority_volume.emissive_mesh_sampling_mode = static_cast<int>(DdgiEmissiveMeshSamplingMode::Off);
  high_priority_volume.hysteresis_boost_trigger_conditions = DdgiVolumeTriggerConditionGeometryChanged;
  ddgi_pack->volumes.push_back(high_priority_volume);
  EnvironmentalLighting::DdgiVolume dense_volume;
  dense_volume.stable_id = 100;
  dense_volume.artist_priority = 1;
  dense_volume.probe_counts = glm::ivec3(2);
  dense_volume.probe_spacing = glm::vec3(0.5f);
  dense_volume.enable_probe_classification = true;
  ddgi_pack->volumes.push_back(dense_volume);
  EnvironmentalLighting::DdgiVolume disabled_volume;
  disabled_volume.stable_id = 999;
  disabled_volume.enabled = false;
  ddgi_pack->volumes.push_back(disabled_volume);

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
  EXPECT_FLOAT_EQ(resolved.local_reflection_probes[0].blend_distance, 0.05f);
  EXPECT_FLOAT_EQ(resolved.local_reflection_probes[1].blend_distance, 0.5f);
  EXPECT_FALSE(ContainsStableId(resolved.local_reflection_probes, 99u));
  EXPECT_FALSE(resolved.local_reflection_probes[2].payload);

  ASSERT_EQ(resolved.ddgi_volumes.size(), 2u);
  EXPECT_EQ(resolved.ddgi_volumes[0].stable_id, 200u);
  EXPECT_EQ(resolved.ddgi_volumes[1].stable_id, 100u);
  EXPECT_EQ(resolved.ddgi_volumes[0].emissive_mesh_sampling_mode, static_cast<int>(DdgiEmissiveMeshSamplingMode::Off));
  EXPECT_EQ(resolved.ddgi_volumes[0].hysteresis_boost_trigger_conditions, DdgiVolumeTriggerConditionGeometryChanged);
  EXPECT_TRUE(resolved.ddgi_volumes[1].enable_probe_classification);

  lighting->local_reflection_probes_enabled = false;
  const auto master_disabled = ResolveEnvironmentalLighting(scene);
  EXPECT_TRUE(master_disabled.local_reflection_probes.empty());
  EXPECT_EQ(master_disabled.ddgi_volumes.size(), 2u);
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

TEST(EnvironmentalLightingAsset, ResolverSafelyIgnoresWrongPackTypesAndSupportsSharedPacks) {
  Application app;
  ApplicationContextScope scope(app);
  app.Initialize(EmptyProjectSettings());
  const auto scene = AssetManager::CreateTemporaryAsset<Scene>();
  const auto lighting = AssetManager::CreateTemporaryAsset<EnvironmentalLighting>();
  const auto second_lighting = AssetManager::CreateTemporaryAsset<EnvironmentalLighting>();
  const auto reflection_pack = AssetManager::CreateTemporaryAsset<ReflectionProbePack>();
  const auto ddgi_pack = AssetManager::CreateTemporaryAsset<DdgiVolumePack>();
  const auto fallback = AssetManager::CreateTemporaryAsset<GlobalReflectionProbe>();
  ASSERT_TRUE(scene && lighting && second_lighting && reflection_pack && ddgi_pack && fallback);
  reflection_pack->probes.emplace_back().stable_id = 11u;
  ddgi_pack->volumes.emplace_back().stable_id = 22u;
  scene->environmental_lighting = lighting;
  scene->global_reflection_probe_fallback = fallback;

  lighting->reflection_probe_pack = ddgi_pack;
  lighting->ddgi_volume_pack = reflection_pack;
  auto resolved = ResolveEnvironmentalLighting(scene);
  EXPECT_TRUE(resolved.local_reflection_probes.empty());
  EXPECT_TRUE(resolved.ddgi_volumes.empty());
  EXPECT_EQ(resolved.scene_global_reflection_probe_fallback.GetAssetHandle(), fallback->GetHandle());

  lighting->reflection_probe_pack = reflection_pack;
  lighting->ddgi_volume_pack = ddgi_pack;
  second_lighting->reflection_probe_pack = reflection_pack;
  second_lighting->ddgi_volume_pack = ddgi_pack;
  resolved = ResolveEnvironmentalLighting(scene);
  ASSERT_EQ(resolved.local_reflection_probes.size(), 1u);
  ASSERT_EQ(resolved.ddgi_volumes.size(), 1u);
  EXPECT_EQ(second_lighting->GetReflectionProbePack(), reflection_pack);
  EXPECT_EQ(second_lighting->GetDdgiVolumePack(), ddgi_pack);
}

TEST(EnvironmentalLightingAsset, ResolverUsesDefaultsForNonFiniteFallbackValues) {
  Application app;
  ApplicationContextScope scope(app);
  app.Initialize(EmptyProjectSettings());
  const auto scene = AssetManager::CreateTemporaryAsset<Scene>();
  const auto lighting = AssetManager::CreateTemporaryAsset<EnvironmentalLighting>();
  ASSERT_TRUE(scene);
  ASSERT_TRUE(lighting);
  scene->environmental_lighting = lighting;

  lighting->environment_lighting_intensity = std::numeric_limits<float>::quiet_NaN();
  lighting->diffuse_fallback_intensity = std::numeric_limits<float>::quiet_NaN();
  lighting->specular_fallback_intensity = std::numeric_limits<float>::infinity();
  lighting->GetOrCreateReflectionProbePack()->probes.emplace_back().blend_distance =
      std::numeric_limits<float>::quiet_NaN();

  const auto resolved = ResolveEnvironmentalLighting(scene);
  EXPECT_FLOAT_EQ(resolved.environment_lighting_intensity,
                  ResolvedEnvironmentalLighting::kDefaultEnvironmentLightingIntensity);
  EXPECT_FLOAT_EQ(resolved.diffuse_fallback_intensity, ResolvedEnvironmentalLighting::kDefaultDiffuseFallbackIntensity);
  EXPECT_FLOAT_EQ(resolved.specular_fallback_intensity,
                  ResolvedEnvironmentalLighting::kDefaultSpecularFallbackIntensity);
  ASSERT_EQ(resolved.local_reflection_probes.size(), 1u);
  EXPECT_FLOAT_EQ(resolved.local_reflection_probes.front().blend_distance, 0.05f);
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
    probe.transform = glm::scale(glm::vec3(2.0f * (1.0f + static_cast<float>(index))));
    lighting->GetOrCreateReflectionProbePack()->probes.push_back(probe);
  }
  for (uint32_t index = 0; index < ResolvedEnvironmentalLighting::kMaxDdgiVolumeCount + 1u; ++index) {
    EnvironmentalLighting::DdgiVolume volume;
    volume.stable_id = index + 1u;
    volume.probe_counts = glm::ivec3(1);
    volume.probe_spacing = glm::vec3(1.0f + static_cast<float>(index));
    lighting->GetOrCreateDdgiVolumePack()->volumes.push_back(volume);
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
