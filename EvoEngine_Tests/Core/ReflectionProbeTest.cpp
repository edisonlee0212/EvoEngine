#include "EvoEngine_SDK_PCH.hpp"

#include <gtest/gtest.h>

#include "Application.hpp"
#include "ApplicationContext.hpp"
#include "ApplicationInitializationSettings.hpp"
#include "AssetManager.hpp"
#include "EnvironmentalLighting.hpp"
#include "GlobalReflectionProbe.hpp"
#include "RenderLayer.hpp"
#include "Scene.hpp"
#include "Serialization.hpp"

#include <glm/gtc/packing.hpp>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iterator>

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

class TemporaryDirectory {
 public:
  TemporaryDirectory() {
    const auto suffix = std::chrono::steady_clock::now().time_since_epoch().count();
    path_ = std::filesystem::temp_directory_path() / ("EvoEngineReflectionProbeTest_" + std::to_string(suffix));
    std::filesystem::create_directories(path_);
  }

  ~TemporaryDirectory() {
    std::error_code error;
    std::filesystem::remove_all(path_, error);
  }

  [[nodiscard]] const std::filesystem::path& Path() const {
    return path_;
  }

 private:
  std::filesystem::path path_;
};

std::vector<uint16_t> CanonicalPayload() {
  std::vector<uint16_t> payload(GlobalReflectionProbe::kCanonicalTexelCount * 4);
  const uint16_t one = glm::packHalf1x16(1.0f);
  for (size_t index = 3; index < payload.size(); index += 4) {
    payload[index] = one;
  }
  return payload;
}

std::string ReadTextFile(const std::filesystem::path& path) {
  std::ifstream file(path);
  EXPECT_TRUE(file.good()) << path.string();
  return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}

std::filesystem::path SourcePath(const std::filesystem::path& relative_path) {
  return std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / relative_path;
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

std::string ExtractBetween(const std::string& source, const std::string& begin, const std::string& end) {
  const auto begin_position = source.find(begin);
  EXPECT_NE(begin_position, std::string::npos) << begin;
  if (begin_position == std::string::npos) {
    return {};
  }
  const auto end_position = source.find(end, begin_position);
  EXPECT_NE(end_position, std::string::npos) << end;
  if (end_position == std::string::npos) {
    return {};
  }
  return source.substr(begin_position, end_position - begin_position);
}

}  // namespace

TEST(ReflectionProbe, CanonicalPayloadContractRejectsMalformedRadiance) {
  EXPECT_EQ(GlobalReflectionProbe::kResolution, 256u);
  EXPECT_EQ(GlobalReflectionProbe::kMipLevels, 9u);
  EXPECT_EQ(GlobalReflectionProbe::kCanonicalFormat, VK_FORMAT_R16G16B16A16_SFLOAT);
  EXPECT_EQ(GlobalReflectionProbe::kCanonicalTexelCount, 524286u);
  EXPECT_EQ(GlobalReflectionProbe::kCanonicalPayloadByteSize, 4194288u);
  EXPECT_EQ(GlobalReflectionProbe::kPackedRuntimeByteSize, 2097144u);
  EXPECT_FLOAT_EQ(GlobalReflectionProbe::kPackedMaxNormalizedRmsError, 0.002f);
  EXPECT_FLOAT_EQ(GlobalReflectionProbe::kPackedMaxRelativePeakError, 0.02f);
  EXPECT_FLOAT_EQ(GlobalReflectionProbe::kPackedMinMemorySaving, 0.49f);
  EXPECT_FLOAT_EQ(GlobalReflectionProbe::kPackedMinGpuTimeImprovement, 0.05f);

  auto payload = CanonicalPayload();
  std::string error;
  ASSERT_TRUE(GlobalReflectionProbe::ValidateCanonicalPayload(payload, error)) << error;
  EXPECT_TRUE(error.empty());
  const uint64_t original_hash = GlobalReflectionProbe::CalculatePayloadHash(payload);
  EXPECT_EQ(GlobalReflectionProbe::CalculatePayloadHash(payload), original_hash);

  float normalized_rms_error = -1.0f;
  float relative_peak_error = -1.0f;
  ASSERT_TRUE(GlobalReflectionProbe::EvaluatePackedRuntimeQuality(payload, normalized_rms_error, relative_peak_error));
  EXPECT_FLOAT_EQ(normalized_rms_error, 0.0f);
  EXPECT_FLOAT_EQ(relative_peak_error, 0.0f);

  payload.pop_back();
  EXPECT_FALSE(GlobalReflectionProbe::ValidateCanonicalPayload(payload, error));
  payload = CanonicalPayload();
  payload[0] = 0xbc00u;
  EXPECT_FALSE(GlobalReflectionProbe::ValidateCanonicalPayload(payload, error));
  payload[0] = 0x7e00u;
  EXPECT_FALSE(GlobalReflectionProbe::ValidateCanonicalPayload(payload, error));
  payload[0] = 0x7c00u;
  EXPECT_FALSE(GlobalReflectionProbe::ValidateCanonicalPayload(payload, error));

  payload = CanonicalPayload();
  payload[0] = glm::packHalf1x16(0.5f);
  EXPECT_NE(GlobalReflectionProbe::CalculatePayloadHash(payload), original_hash);
}

TEST(ReflectionProbe, RoughSpecularVisibilityCapsUntrustedGrazingOcclusion) {
  EXPECT_FLOAT_EQ(EnvironmentalLighting::kSpecularVisibilityGrazingOcclusionCap, 0.04f);
  EXPECT_FLOAT_EQ(EnvironmentalLighting::kSpecularVisibilityFullTrustStart, 0.8f);
  EXPECT_FLOAT_EQ(EnvironmentalLighting::EvaluateRoughSpecularVisibility(1.0f, 1.0f, 1.0f, 1.0f, 1.0f), 1.0f);
  EXPECT_FLOAT_EQ(EnvironmentalLighting::EvaluateRoughSpecularVisibility(0.4f, 0.5f, 1.0f, 1.0f, 1.0f), 0.4f);
  EXPECT_FLOAT_EQ(EnvironmentalLighting::EvaluateRoughSpecularVisibility(0.4f, 0.5f, 1.0f, 0.5f, 1.0f), 0.85f);
  EXPECT_FLOAT_EQ(EnvironmentalLighting::EvaluateRoughSpecularVisibility(0.0f, 0.0f, 1.0f, 0.0f, 1.0f), 1.0f);
  EXPECT_NEAR(EnvironmentalLighting::EvaluateRoughSpecularVisibility(0.0f, 0.0f, 1.0f, 1.0f, 0.0f), 0.96f, 1.0e-6f);
  EXPECT_NEAR(EnvironmentalLighting::EvaluateRoughSpecularVisibility(0.0f, 0.0f, 1.0f, 1.0f, 0.8f), 0.96f, 1.0e-6f);
  EXPECT_NEAR(EnvironmentalLighting::EvaluateRoughSpecularVisibility(0.0f, 0.0f, 1.0f, 1.0f, 0.9f), 0.48f, 1.0e-6f);
  EXPECT_LT(EnvironmentalLighting::EvaluateRoughSpecularVisibility(0.985f, 1.0f, 1.0f, 1.0f, 0.0f), 0.99f);
  EXPECT_FLOAT_EQ(EnvironmentalLighting::EvaluateRoughSpecularVisibility(0.25f, 1.0f, 1.0f, 1.0f, 1.0f), 0.25f);
  EXPECT_FLOAT_EQ(EnvironmentalLighting::EvaluateRoughSpecularVisibility(1.0f, 0.25f, 1.0f, 1.0f, 1.0f), 0.25f);
  EXPECT_FLOAT_EQ(EnvironmentalLighting::EvaluateRoughSpecularVisibility(1.0f, 1.0f, 0.25f, 1.0f, 1.0f), 0.25f);
  EXPECT_FLOAT_EQ(EnvironmentalLighting::EvaluateRoughSpecularVisibility(-1.0f, 2.0f, 2.0f, 1.0f, 1.0f), 0.0f);
  EXPECT_GT(EnvironmentalLighting::EvaluateRoughSpecularVisibility(0.2f, 0.4f, 1.0f, 0.2f, 1.0f),
            EnvironmentalLighting::EvaluateRoughSpecularVisibility(0.2f, 0.4f, 1.0f, 0.9f, 1.0f));
  EXPECT_GT(EnvironmentalLighting::EvaluateRoughSpecularVisibility(1.0f, 1.0f, 1.0f, 1.0f, 1.0f),
            EnvironmentalLighting::EvaluateRoughSpecularVisibility(1.0f, 1.0f, 0.2f, 1.0f, 1.0f));
}

TEST(ReflectionProbe, BakeEnvironmentInputUsesSourceIntensityOnly) {
  const auto render_layer = ReadTextFile(SourcePath("EvoEngine_SDK/src/RenderLayer.cpp"));
  ASSERT_FALSE(render_layer.empty());

  const auto fingerprint = ExtractBetween(render_layer, "uint64_t RenderLayer::GetReflectionProbeCaptureFingerprint",
                                          "bool RenderLayer::QueueGlobalReflectionProbeBake");
  const auto bake =
      ExtractBetween(render_layer, "bool RenderLayer::BakeReflectionProbe", "void RenderLayer::RenderAll");
  ASSERT_FALSE(fingerprint.empty());
  ASSERT_FALSE(bake.empty());

  EXPECT_NE(fingerprint.find("MixDdgiFloat(fingerprint, resolved_lighting.environment_lighting_intensity)"),
            std::string::npos);
  EXPECT_EQ(fingerprint.find("diffuse_fallback_intensity"), std::string::npos);
  EXPECT_EQ(fingerprint.find("specular_fallback_intensity"), std::string::npos);
  EXPECT_NE(bake.find("camera->camera_settings.background_intensity = "
                      "glm::max(resolved_lighting.environment_lighting_intensity, 0.0f)"),
            std::string::npos);
  EXPECT_EQ(bake.find("diffuse_fallback_intensity"), std::string::npos);
  EXPECT_EQ(bake.find("specular_fallback_intensity"), std::string::npos);
}

TEST(ReflectionProbe, DemoSceneReflectionProbeValidationUsesEnvironmentalLightingEntries) {
  const auto demo_scene = ReadTextFile(SourcePath("EvoEngine_App/src/DemoScene.cpp"));
  ASSERT_FALSE(demo_scene.empty());

  EXPECT_NE(demo_scene.find("AddReflectionProbeValidationLocalProbe"), std::string::npos);
  EXPECT_NE(demo_scene.find("RunEnvironmentalLightingLocalProbeBake"), std::string::npos);
  EXPECT_NE(demo_scene.find("FindEnvironmentalLightingLocalReflectionProbe(scene, kSponzaLocalProbeNames.front())"),
            std::string::npos);
  EXPECT_EQ(demo_scene.find("#include \"ReflectionProbe.hpp\""), std::string::npos);
  EXPECT_EQ(demo_scene.find("GetOrSetPrivateComponent<ReflectionProbe>"), std::string::npos);
  EXPECT_EQ(demo_scene.find("HasPrivateComponent<ReflectionProbe>"), std::string::npos);
  EXPECT_EQ(demo_scene.find("RemovePrivateComponent<ReflectionProbe>"), std::string::npos);
  EXPECT_EQ(demo_scene.find("BakeStaleInScene(scene)"), std::string::npos);
  EXPECT_EQ(demo_scene.find("IsEnvironmentalLightingLocalProbeBakeStale"), std::string::npos);
  EXPECT_EQ(demo_scene.find("GetGlobalReflectionProbeBakeFingerprint"), std::string::npos);
  EXPECT_EQ(demo_scene.find("stale_rebake"), std::string::npos);
  EXPECT_NE(demo_scene.find("explicit_rebake"), std::string::npos);
}

TEST(ReflectionProbe, SceneGlobalFallbackSerializesAndFiltersRuntimeReadiness) {
  Application app;
  ApplicationContextScope scope(app);
  app.Initialize(EmptyProjectSettings());
  const auto scene = AssetManager::CreateTemporaryAsset<Scene>();
  const auto fallback = AssetManager::CreateTemporaryAsset<GlobalReflectionProbe>();
  ASSERT_TRUE(scene);
  ASSERT_TRUE(fallback);
  ASSERT_FALSE(fallback->IsRuntimeReady());
  scene->global_reflection_probe_fallback = fallback;

  EXPECT_EQ(scene->GetGlobalReflectionProbeFallback(false), fallback);
  EXPECT_FALSE(scene->GetGlobalReflectionProbeFallback());

  YAML::Emitter out;
  out << YAML::BeginMap;
  Serialization::SerializeObject(out, static_cast<IAsset&>(*scene));
  out << YAML::EndMap;
  const auto node = YAML::Load(out.c_str());
  ASSERT_TRUE(node["global_reflection_probe_fallback"]);
  EXPECT_EQ(node["global_reflection_probe_fallback"]["asset_handle_"].as<uint64_t>(), fallback->GetHandle().GetValue());
  EXPECT_TRUE(ContainsLocalAsset(node, "GlobalReflectionProbe", fallback->GetHandle()));

  const auto restored = AssetManager::CreateTemporaryAsset<Scene>();
  ASSERT_TRUE(restored);
  Serialization::DeserializeObject(node, static_cast<IAsset&>(*restored));
  EXPECT_EQ(restored->global_reflection_probe_fallback.GetAssetHandle(), fallback->GetHandle());
  const auto restored_fallback = restored->GetGlobalReflectionProbeFallback(false);
  ASSERT_TRUE(restored_fallback);
  EXPECT_EQ(restored_fallback->GetHandle(), fallback->GetHandle());
  EXPECT_FALSE(restored->GetGlobalReflectionProbeFallback());
}

TEST(ReflectionProbe, GlobalAssetPersistenceIsVersionedAndRejectsMalformedDocuments) {
  Application app;
  ApplicationContextScope scope(app);
  app.Initialize(EmptyProjectSettings());
  TemporaryDirectory directory;
  const auto asset_path = directory.Path() / "roundtrip.evereflectionprobe";
  const auto source = AssetManager::CreateTemporaryAsset<GlobalReflectionProbe>();
  ASSERT_TRUE(source);
  ASSERT_TRUE(source->GetCubemap());
  EXPECT_EQ(source->GetCubemap()->GetResolution(), GlobalReflectionProbe::kResolution);
  EXPECT_EQ(source->GetCubemap()->GetMipLevels(), GlobalReflectionProbe::kMipLevels);
  EXPECT_EQ(source->GetCubemap()->GetFormat(), GlobalReflectionProbe::kCanonicalFormat);
  EXPECT_EQ(source->GetCanonicalPayloadByteSize(), GlobalReflectionProbe::kCanonicalPayloadByteSize);
  EXPECT_EQ(source->GetSourceKind(), GlobalReflectionProbe::SourceKind::Empty);
  EXPECT_FALSE(source->IsRuntimeReady());
  auto payload = CanonicalPayload();
  payload[0] = glm::packHalf1x16(2.0f);
  payload[1] = glm::packHalf1x16(0.5f);
  payload[2] = glm::packHalf1x16(0.25f);
  ASSERT_TRUE(source->SetCanonicalPayload(payload));
  EXPECT_TRUE(source->IsRuntimeReady());
  source->MarkBaked(0x123456789abcdef0ull);

  YAML::Emitter local_out;
  local_out << YAML::BeginMap;
  Serialization::SerializeObject(local_out, *source);
  local_out << YAML::EndMap;
  const auto local_restored = AssetManager::CreateTemporaryAsset<GlobalReflectionProbe>();
  ASSERT_TRUE(local_restored);
  Serialization::DeserializeObject(YAML::Load(local_out.c_str()), *local_restored);
  EXPECT_EQ(local_restored->GetSourceKind(), GlobalReflectionProbe::SourceKind::Baked);
  EXPECT_EQ(local_restored->GetSourceFingerprint(), source->GetSourceFingerprint());
  EXPECT_EQ(local_restored->GetPayloadHash(), source->GetPayloadHash());
  EXPECT_EQ(local_restored->GetCanonicalPayload(), payload);

  ASSERT_TRUE(Serialization::SaveAsset(*source, asset_path));
  EXPECT_TRUE(std::filesystem::is_regular_file(asset_path));
  EXPECT_EQ(std::count_if(std::filesystem::directory_iterator(directory.Path()), std::filesystem::directory_iterator(),
                          [](const auto& entry) {
                            return entry.path().filename().string().find(".tmp.") != std::string::npos;
                          }),
            0);

  const auto restored = AssetManager::CreateTemporaryAsset<GlobalReflectionProbe>();
  ASSERT_TRUE(restored);
  ASSERT_TRUE(Serialization::LoadAsset(*restored, asset_path));
  EXPECT_EQ(restored->GetSourceKind(), GlobalReflectionProbe::SourceKind::Baked);
  EXPECT_EQ(restored->GetSourceFingerprint(), 0x123456789abcdef0ull);
  EXPECT_EQ(restored->GetPayloadHash(), source->GetPayloadHash());
  EXPECT_EQ(restored->GetCanonicalPayloadByteSize(), GlobalReflectionProbe::kCanonicalPayloadByteSize);
  ASSERT_EQ(restored->GetCanonicalPayload().size(), payload.size());
  EXPECT_TRUE(
      std::equal(restored->GetCanonicalPayload().begin(), restored->GetCanonicalPayload().end(), payload.begin()));
  EXPECT_TRUE(restored->SupportsStagedLoading());
  const auto restored_kind = restored->GetSourceKind();
  const auto restored_fingerprint = restored->GetSourceFingerprint();
  const auto restored_hash = restored->GetPayloadHash();
  const auto restored_payload = restored->GetCanonicalPayload();

  const auto malformed_path = directory.Path() / "malformed.evereflectionprobe";
  {
    std::ofstream malformed(malformed_path, std::ios::trunc);
    malformed << "schema_version: 1\nformat: 97\npixels: !!binary AQID\n";
  }
  EXPECT_FALSE(Serialization::LoadAsset(*restored, malformed_path));
  EXPECT_EQ(restored->GetSourceKind(), restored_kind);
  EXPECT_EQ(restored->GetSourceFingerprint(), restored_fingerprint);
  EXPECT_EQ(restored->GetPayloadHash(), restored_hash);
  EXPECT_EQ(restored->GetCanonicalPayload(), restored_payload);

  const auto truncated_path = directory.Path() / "truncated.evereflectionprobe";
  {
    std::ofstream truncated(truncated_path, std::ios::trunc);
  }
  EXPECT_FALSE(Serialization::LoadAsset(*restored, truncated_path));
  EXPECT_EQ(restored->GetSourceKind(), restored_kind);
  EXPECT_EQ(restored->GetSourceFingerprint(), restored_fingerprint);
  EXPECT_EQ(restored->GetPayloadHash(), restored_hash);
  EXPECT_EQ(restored->GetCanonicalPayload(), restored_payload);

  const auto mismatched_hash_path = directory.Path() / "mismatched-hash.evereflectionprobe";
  auto mismatched_hash_node = YAML::LoadFile(asset_path.string());
  mismatched_hash_node["payload_hash"] = restored_hash + 1u;
  {
    std::ofstream mismatched_hash(mismatched_hash_path, std::ios::trunc);
    mismatched_hash << mismatched_hash_node;
  }
  EXPECT_FALSE(Serialization::LoadAsset(*restored, mismatched_hash_path));
  EXPECT_EQ(restored->GetSourceKind(), restored_kind);
  EXPECT_EQ(restored->GetSourceFingerprint(), restored_fingerprint);
  EXPECT_EQ(restored->GetPayloadHash(), restored_hash);
  EXPECT_EQ(restored->GetCanonicalPayload(), restored_payload);

  const auto legacy_path = directory.Path() / "legacy-empty.evereflectionprobe";
  {
    std::ofstream legacy(legacy_path, std::ios::trunc);
    legacy << "{}\n";
  }
  EXPECT_FALSE(Serialization::LoadAsset(*restored, legacy_path));
  EXPECT_EQ(restored->GetSourceKind(), restored_kind);
  EXPECT_EQ(restored->GetSourceFingerprint(), restored_fingerprint);
  EXPECT_EQ(restored->GetPayloadHash(), restored_hash);
  EXPECT_EQ(restored->GetCanonicalPayload(), restored_payload);
}
