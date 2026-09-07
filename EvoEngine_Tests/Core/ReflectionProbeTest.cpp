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

TEST(ReflectionProbe, BakeBackgroundUsesEnvironmentalLightingIntensity) {
  const auto render_layer = ReadTextFile(SourcePath("EvoEngine_SDK/src/RenderLayer.cpp"));
  const auto lighting =
      ReadTextFile(SourcePath("EvoEngine_SDK/Internals/DefaultResources/Shaders/Modules/EvoEngine/Lighting.slang"));
  ASSERT_FALSE(render_layer.empty());
  ASSERT_FALSE(lighting.empty());

  const auto prepare = ExtractBetween(render_layer, "void RenderLayer::PrepareReflectionProbeBake",
                                      "void RenderLayer::EnsureReflectionProbeCaptureRenderGraph");
  const auto dynamic_prepare = ExtractBetween(render_layer, "void RenderLayer::PrepareDynamicReflectionProbeUpdate",
                                              "void RenderLayer::RecordPreparedDynamicReflectionProbeUpdate");
  ASSERT_FALSE(prepare.empty());
  ASSERT_FALSE(dynamic_prepare.empty());

  EXPECT_NE(prepare.find("lighting->reflection_probe_bake_background"), std::string::npos);
  EXPECT_NE(prepare.find("face_camera->camera_settings.background_intensity"), std::string::npos);
  EXPECT_NE(prepare.find("resolved_lighting.environment_lighting_intensity"), std::string::npos);
  EXPECT_NE(prepare.find("face_camera->camera_settings.clear_color = background.clear_color"), std::string::npos);
  EXPECT_NE(prepare.find("face_camera->skybox = background.cubemap"), std::string::npos);
  EXPECT_NE(prepare.find("face_camera->background_environment = background.environmental_map"), std::string::npos);
  EXPECT_NE(dynamic_prepare.find("resolved.environment_lighting_intensity"), std::string::npos);
  EXPECT_EQ(render_layer.find("background.intensity"), std::string::npos);
  EXPECT_NE(lighting.find("albedo * EE_ENVIRONMENT.diffuse_fallback_intensity"), std::string::npos);
  EXPECT_NE(lighting.find("EE_BASIC_CONSTANTS.instance_index == 2 ? 0"), std::string::npos);
  EXPECT_NE(lighting.find("if (EE_BASIC_CONSTANTS.instance_index == 2)"), std::string::npos);
}

TEST(ReflectionProbe, ExplicitBakeDefersReadbackAndPersistenceUntilAssetSave) {
  const auto render_layer = ReadTextFile(SourcePath("EvoEngine_SDK/src/RenderLayer.cpp"));
  const auto probe = ReadTextFile(SourcePath("EvoEngine_SDK/src/GlobalReflectionProbe.cpp"));
  const auto inspector = ReadTextFile(SourcePath("EvoEngine_SDK/src/Editor/SDKInspectionAdapters.cpp"));
  ASSERT_FALSE(render_layer.empty());
  ASSERT_FALSE(probe.empty());
  ASSERT_FALSE(inspector.empty());

  const auto publish = ExtractBetween(render_layer, "void RenderLayer::PublishSubmittedReflectionProbeBake",
                                      "void RenderLayer::RenderAll");
  const auto serialize =
      ExtractBetween(probe, "void GlobalReflectionProbe::Serialize", "void GlobalReflectionProbe::Deserialize");
  const auto save =
      ExtractBetween(probe, "bool GlobalReflectionProbe::SaveInternal", "bool GlobalReflectionProbe::LoadInternal");
  ASSERT_FALSE(publish.empty());
  ASSERT_FALSE(serialize.empty());
  ASSERT_FALSE(save.empty());

  EXPECT_NE(publish.find("target->cubemap_ = output"), std::string::npos);
  EXPECT_NE(publish.find("target->source_kind_ = GlobalReflectionProbe::SourceKind::Baked"), std::string::npos);
  EXPECT_NE(publish.find("target->SetUnsaved()"), std::string::npos);
  EXPECT_EQ(render_layer.find("Serialization::SaveAsset"), std::string::npos);
  EXPECT_EQ(render_layer.find("target->Load()"), std::string::npos);
  EXPECT_EQ(publish.find("GetCanonicalPayload"), std::string::npos);
  EXPECT_NE(serialize.find("cubemap_->GetRgba16fData(payload, true)"), std::string::npos);
  EXPECT_NE(serialize.find("payload_hash_ = CalculatePayloadHash(payload)"), std::string::npos);
  EXPECT_NE(save.find("Serialize(out)"), std::string::npos);
  EXPECT_NE(inspector.find("Persistence: unsaved pack payload"), std::string::npos);
  EXPECT_NE(inspector.find("Payload: GPU resident; downloaded on save"), std::string::npos);
}

TEST(ReflectionProbe, PrefilterUsesDirectBaseMipAndProgressiveSampleBudgets) {
  const auto probe = ReadTextFile(SourcePath("EvoEngine_SDK/src/GlobalReflectionProbe.cpp"));
  const auto shader = ReadTextFile(SourcePath(
      "EvoEngine_SDK/Internals/DefaultResources/Shaders/Graphics/Fragment/Lighting/EnvironmentalMapPrefilter.slang"));
  ASSERT_FALSE(probe.empty());
  ASSERT_FALSE(shader.empty());

  const auto filtered = ExtractBetween(probe, "bool GlobalReflectionProbe::ConstructFilteredFromCubemap",
                                       "void GlobalReflectionProbe::MarkBaked");
  ASSERT_FALSE(filtered.empty());
  EXPECT_NE(probe.find("kPrefilterSampleCounts"), std::string::npos);
  EXPECT_NE(filtered.find("filtered_cubemap->Initialize(kResolution, kMipLevels, kCanonicalFormat)"),
            std::string::npos);
  EXPECT_EQ(filtered.find("SetRgba16fData"), std::string::npos);
  EXPECT_NE(probe.find("kPrefilterSampleCounts[mip]"), std::string::npos);
  EXPECT_NE(shader.find("constants.preset_value <= 0.0f"), std::string::npos);
  EXPECT_NE(shader.find("environmentMap.SampleLevel(n, 0.0f)"), std::string::npos);
  EXPECT_NE(shader.find("uint sample_count = max(constants.sample_count, 1u)"), std::string::npos);
  EXPECT_EQ(shader.find("SAMPLE_COUNT = 1024u"), std::string::npos);
}

TEST(ReflectionProbe, ExplicitBakesUseCachedGraphSharedBindingsAndNormalFrameSubmission) {
  const auto header = ReadTextFile(SourcePath("EvoEngine_SDK/include/Layers/RenderLayer.hpp"));
  const auto camera = ReadTextFile(SourcePath("EvoEngine_SDK/src/Camera.cpp"));
  const auto render_layer = ReadTextFile(SourcePath("EvoEngine_SDK/src/RenderLayer.cpp"));
  ASSERT_FALSE(header.empty());
  ASSERT_FALSE(camera.empty());
  ASSERT_FALSE(render_layer.empty());

  const auto create = ExtractBetween(
      render_layer, "const std::vector<std::shared_ptr<Camera>>& RenderLayer::GetOrCreateReflectionProbeCaptureCameras",
      "bool RenderLayer::PrepareReflectionProbeCaptureResources");
  const auto resources = ExtractBetween(render_layer, "bool RenderLayer::PrepareReflectionProbeCaptureResources",
                                        "void RenderLayer::FailReflectionProbeBakeBatch");
  const auto prepare = ExtractBetween(render_layer, "void RenderLayer::PrepareReflectionProbeBake",
                                      "void RenderLayer::EnsureReflectionProbeCaptureRenderGraph");
  const auto graph = ExtractBetween(render_layer, "void RenderLayer::EnsureReflectionProbeCaptureRenderGraph",
                                    "void RenderLayer::RecordPreparedReflectionProbeBake");
  const auto record = ExtractBetween(render_layer, "void RenderLayer::RecordPreparedReflectionProbeBake",
                                     "void RenderLayer::PublishSubmittedReflectionProbeBake");
  ASSERT_FALSE(create.empty());
  ASSERT_FALSE(resources.empty());
  ASSERT_FALSE(prepare.empty());
  ASSERT_FALSE(graph.empty());
  ASSERT_FALSE(record.empty());

  EXPECT_NE(header.find("std::vector<std::shared_ptr<Camera>> reflection_probe_capture_cameras_"), std::string::npos);
  EXPECT_NE(header.find("reflection_probe_capture_raw_cubemap_"), std::string::npos);
  EXPECT_NE(header.find("reflection_probe_capture_filtered_cubemap_"), std::string::npos);
  EXPECT_NE(create.find("InitializeRenderResources("), std::string::npos);
  EXPECT_NE(create.find("ResetRenderState()"), std::string::npos);
  EXPECT_NE(create.find("ResetFrameCount()"), std::string::npos);
  EXPECT_NE(resources.find("if (!reflection_probe_capture_raw_cubemap_)"), std::string::npos);
  EXPECT_NE(resources.find("if (!reflection_probe_capture_filtered_cubemap_)"), std::string::npos);
  EXPECT_NE(resources.find("GlobalReflectionProbe::AcquirePrefilterPipeline"), std::string::npos);
  EXPECT_NE(prepare.find("GetOrCreateReflectionProbeCaptureCameras(batch.requests.size() * 6u)"), std::string::npos);
  EXPECT_EQ(record.find("RenderSceneToCameraImmediately"), std::string::npos);
  EXPECT_EQ(record.find("ConstructBakedFromCubemap"), std::string::npos);
  EXPECT_EQ(record.find("ImmediateSubmit"), std::string::npos);
  EXPECT_EQ(record.find("WaitForFrameSubmissions"), std::string::npos);
  EXPECT_NE(record.find("Platform::RecordCommandsMainQueue"), std::string::npos);
  EXPECT_NE(graph.find("reflection_probe_capture_render_graph_plan_.valid"), std::string::npos);
  EXPECT_NE(graph.find("AddDefaultRasterCameraResources"), std::string::npos);
  EXPECT_NE(graph.find("geometry_descriptor.dependencies.clear()"), std::string::npos);
  EXPECT_NE(record.find("reflection_probe_capture_render_graph_.Execute"), std::string::npos);
  EXPECT_NE(record.find("emplace_back()"), std::string::npos);
  EXPECT_NE(record.find("capture_lighting_descriptor_set"), std::string::npos);
  EXPECT_EQ(record.find("face_lighting_descriptor_sets"), std::string::npos);
  EXPECT_EQ(record.find("PreparePointAndSpotLightShadowMap"), std::string::npos);
  EXPECT_NE(record.find("GlobalReflectionProbe::RecordPrefilter"), std::string::npos);
  EXPECT_EQ(prepare.find("ProduceSerializable<Camera>"), std::string::npos);
  EXPECT_EQ(prepare.find("camera->OnCreate()"), std::string::npos);
  EXPECT_EQ(prepare.find("camera->Resize("), std::string::npos);
  EXPECT_NE(render_layer.find("prepared_reflection_probe_bake_->injected_cameras"), std::string::npos);
  EXPECT_NE(camera.find("void Camera::OnCreate() {\n  InitializeRenderResources({1, 1});"), std::string::npos);
}

TEST(ReflectionProbe, SdfgiCapturesBindPublishedReadsAndRemainDiffuseOnly) {
  const auto render = ReadTextFile(SourcePath("EvoEngine_SDK/src/RenderLayer.cpp"));
  const auto graph = ExtractBetween(render, "void RenderLayer::EnsureReflectionProbeCaptureRenderGraph",
                                    "void RenderLayer::RecordPreparedReflectionProbeBake");
  EXPECT_NE(graph.find("publication->CameraReads()"), std::string::npos);
  EXPECT_NE(graph.find("publication->ImportCamera("), std::string::npos);
  EXPECT_NE(graph.find("capture.sdfgi_publication->descriptor_set"), std::string::npos);
  EXPECT_NE(graph.find("push_back(sdfgi_resources)"), std::string::npos);
  const auto composition = ReadTextFile(
      SourcePath("EvoEngine_SDK/Internals/DefaultResources/Shaders/Modules/EvoEngine/SdfgiLighting.slang"));
  EXPECT_NE(composition.find("EE_BASIC_CONSTANTS.instance_index == 2"), std::string::npos);
  EXPECT_NE(composition.find("roughness, !reflection_capture)"), std::string::npos);
  ASSERT_NE(composition.find("if (reflection_capture) return diffuse;"), std::string::npos);
  EXPECT_LT(composition.find("if (reflection_capture) return diffuse;"),
            composition.find("resources.prefilteredLevelCount()"));
}

TEST(ReflectionProbe, DynamicUpdatesAreContinuousBudgetedBlendedAndAssetIndependent) {
  const auto lighting_header =
      ReadTextFile(SourcePath("EvoEngine_SDK/include/Rendering/PBR/EnvironmentalLighting.hpp"));
  const auto render_header = ReadTextFile(SourcePath("EvoEngine_SDK/include/Layers/RenderLayer.hpp"));
  const auto render_layer = ReadTextFile(SourcePath("EvoEngine_SDK/src/RenderLayer.cpp"));
  const auto render_instances = ReadTextFile(SourcePath("EvoEngine_SDK/src/RenderInstanceStorage.cpp"));
  const auto inspector = ReadTextFile(SourcePath("EvoEngine_SDK/src/Editor/SDKInspectionAdapters.cpp"));
  const auto lighting_shader =
      ReadTextFile(SourcePath("EvoEngine_SDK/Internals/DefaultResources/Shaders/Modules/EvoEngine/Lighting.slang"));
  const auto fixed_lighting_shader = ReadTextFile(
      SourcePath("EvoEngine_SDK/Internals/DefaultResources/Shaders/Modules/EvoEngine/LightingFixedSet3.slang"));
  ASSERT_FALSE(lighting_header.empty());
  ASSERT_FALSE(render_header.empty());
  ASSERT_FALSE(render_layer.empty());
  ASSERT_FALSE(render_instances.empty());
  ASSERT_FALSE(inspector.empty());
  ASSERT_FALSE(lighting_shader.empty());
  ASSERT_FALSE(fixed_lighting_shader.empty());

  const auto prepare = ExtractBetween(render_layer, "void RenderLayer::PrepareDynamicReflectionProbeUpdate",
                                      "void RenderLayer::RecordPreparedDynamicReflectionProbeUpdate");
  const auto record = ExtractBetween(render_layer, "void RenderLayer::RecordPreparedDynamicReflectionProbeUpdate",
                                     "void RenderLayer::PublishSubmittedDynamicReflectionProbeUpdate");
  const auto publish = ExtractBetween(render_layer, "void RenderLayer::PublishSubmittedDynamicReflectionProbeUpdate",
                                      "void RenderLayer::RenderAll");
  const auto queue = ExtractBetween(render_layer, "uint32_t RenderLayer::QueueGlobalReflectionProbeBakeBatch",
                                    "bool RenderLayer::IsGlobalReflectionProbeBakePending");
  ASSERT_FALSE(prepare.empty());
  ASSERT_FALSE(record.empty());
  ASSERT_FALSE(publish.empty());
  ASSERT_FALSE(queue.empty());

  EXPECT_EQ(lighting_header.find("DynamicReflectionProbeUpdatePolicy"), std::string::npos);
  EXPECT_EQ(lighting_header.find("update_policy"), std::string::npos);
  EXPECT_EQ(lighting_header.find("Manual"), std::string::npos);
  EXPECT_NE(lighting_header.find("faces_per_frame = 6"), std::string::npos);
  EXPECT_NE(lighting_header.find("bool enabled = true"), std::string::npos);
  EXPECT_NE(render_header.find("std::array<std::shared_ptr<Cubemap>, 2> filtered_generations"), std::string::npos);
  EXPECT_NE(render_header.find("std::array<DynamicReflectionProbeRawSlot, 2>"), std::string::npos);
  EXPECT_NE(render_header.find("std::vector<DynamicReflectionProbeFilterJob> filter_jobs"), std::string::npos);
  EXPECT_NE(render_header.find("filtered_face_count"), std::string::npos);
  EXPECT_NE(render_header.find("transition_start_face_serial"), std::string::npos);
  EXPECT_NE(render_header.find("dynamic_reflection_probe_texture_overrides_"), std::string::npos);
  EXPECT_NE(prepare.find("dynamic_reflection_probe_filter_queue_.front()"), std::string::npos);
  EXPECT_NE(prepare.find("dynamic_reflection_probe_filter_queue_.clear()"), std::string::npos);
  EXPECT_NE(prepare.find("prepared.filter_jobs.push_back"), std::string::npos);
  EXPECT_NE(prepare.find("prepared.jobs.push_back"), std::string::npos);
  EXPECT_NE(prepare.find("settings.faces_per_frame"), std::string::npos);
  EXPECT_NE(prepare.find("6u - state.next_face"), std::string::npos);
  EXPECT_NE(prepare.find("state.position != position"), std::string::npos);
  EXPECT_NE(prepare.find("++state.capture_revision"), std::string::npos);
  EXPECT_NE(prepare.find("const auto has_dynamic_runtime"), std::string::npos);
  EXPECT_NE(prepare.find("runtime_identity_changed && has_dynamic_runtime()"), std::string::npos);
  const auto contribution_suspension_begin = prepare.find("if (!lighting->local_reflection_probes_enabled)");
  const auto contribution_suspension_end = prepare.find("const bool activating", contribution_suspension_begin);
  ASSERT_NE(contribution_suspension_begin, std::string::npos);
  ASSERT_NE(contribution_suspension_end, std::string::npos);
  const auto contribution_suspension =
      prepare.substr(contribution_suspension_begin, contribution_suspension_end - contribution_suspension_begin);
  EXPECT_NE(contribution_suspension.find("dynamic_reflection_probe_contributing_ = false"), std::string::npos);
  EXPECT_EQ(contribution_suspension.find("RetireDynamicReflectionProbeRuntime"), std::string::npos);
  EXPECT_NE(prepare.find("dynamic_reflection_probe_queue_.erase"), std::string::npos);
  EXPECT_NE(prepare.find("state.published_generation < 0 ? 1 : 1 - state.published_generation"), std::string::npos);
  EXPECT_NE(prepare.find("if (idle)"), std::string::npos);
  EXPECT_NE(prepare.find("UpdateDynamicReflectionProbeTransitions"), std::string::npos);
  EXPECT_EQ(prepare.find("probe.transform !="), std::string::npos);
  EXPECT_NE(record.find("dynamic_reflection_probe_raw_slots_"), std::string::npos);
  EXPECT_NE(record.find("reflection_probe_capture_filtered_cubemap_"), std::string::npos);
  EXPECT_NE(record.find("GlobalReflectionProbe::RecordPrefilterFaces"), std::string::npos);
  EXPECT_NE(record.find("GlobalReflectionProbe::kMipLevels) * job.face_count"), std::string::npos);
  EXPECT_NE(record.find("state.filtering = true"), std::string::npos);
  EXPECT_NE(record.find("state.completion_in_flight = true"), std::string::npos);
  EXPECT_NE(record.find("Platform::RecordCommandsMainQueue"), std::string::npos);
  EXPECT_EQ(record.find("ImmediateSubmit"), std::string::npos);
  EXPECT_EQ(record.find("WaitForFrameSubmissions"), std::string::npos);
  EXPECT_NE(publish.find("completion.output->MarkGpuContentValid()"), std::string::npos);
  EXPECT_NE(publish.find("texture_override.source_texture_index"), std::string::npos);
  EXPECT_NE(publish.find("texture_override.target_texture_index"), std::string::npos);
  EXPECT_NE(publish.find("texture_override.blend_weight = 0.0f"), std::string::npos);
  EXPECT_NE(publish.find("state.published_generation = completion.generation"), std::string::npos);
  EXPECT_NE(publish.find("completion.capture_revision != state.capture_revision"), std::string::npos);
  EXPECT_NE(publish.find("dynamic_reflection_probe_texture_overrides_"), std::string::npos);
  EXPECT_NE(render_instances.find("texture_overrides->find(probe.stable_id)"), std::string::npos);
  EXPECT_NE(queue.find("disable dynamic local probe updates first"), std::string::npos);
  EXPECT_NE(inspector.find("Enable dynamic local probe updates"), std::string::npos);
  EXPECT_EQ(inspector.find("Dynamic update policy"), std::string::npos);
  EXPECT_NE(inspector.find("Published A/B"), std::string::npos);
  EXPECT_NE(inspector.find("Current probe: none (0/6 faces)"), std::string::npos);
  EXPECT_NE(inspector.find("Current GGX filter: none (0/6 faces)"), std::string::npos);
  EXPECT_NE(inspector.find("Reset Dynamic Probe History"), std::string::npos);
  EXPECT_EQ(render_layer.find("InvalidateAllDynamicReflectionProbes"), std::string::npos);
  EXPECT_EQ(render_layer.find("DDGI runtime lighting updated"), std::string::npos);
  EXPECT_NE(render_layer.find("PreserveReflectionProbeTextureBindings"), std::string::npos);
  const auto comparison_start = render_layer.find("bool RenderLayer::UpdateRenderInstanceStorage(");
  ASSERT_NE(comparison_start, std::string::npos);
  const auto comparison = render_layer.substr(comparison_start);
  const auto preserve_bindings = comparison.find("PreserveReflectionProbeTextureBindings(");
  const auto ddgi_branch = comparison.find("if (track_ddgi_scene_inputs)");
  ASSERT_NE(preserve_bindings, std::string::npos);
  ASSERT_NE(ddgi_branch, std::string::npos);
  EXPECT_LT(preserve_bindings, ddgi_branch);
  const auto last_comparison =
      comparison.rfind("render_instance_updated = *current_render_instances != *previous_render_instances;");
  const auto restore_bindings = comparison.find(
      "current_render_instances->render_info_block.reflection_probes = current_render_info.reflection_probes;");
  ASSERT_NE(last_comparison, std::string::npos);
  ASSERT_NE(restore_bindings, std::string::npos);
  EXPECT_GT(restore_bindings, last_comparison);
  EXPECT_LT(restore_bindings, comparison.find("const auto camera_info_changed"));
  EXPECT_EQ(render_layer.find("PreserveReflectionProbeRenderInfo"), std::string::npos);
  EXPECT_EQ(record.find("SetUnsaved"), std::string::npos);
  EXPECT_EQ(record.find("Save"), std::string::npos);
  EXPECT_EQ(record.find("Download"), std::string::npos);
  EXPECT_NE(lighting_shader.find("lerp(source, target, blend_weight)"), std::string::npos);
  EXPECT_NE(lighting_shader.find("int(probe.identity_and_flags.x)"), std::string::npos);
  EXPECT_NE(lighting_shader.find("int(probe.transition_parameters.x)"), std::string::npos);
  EXPECT_NE(fixed_lighting_shader.find("EE_CUBEMAPS[NonUniformResourceIndex(texture_index)]"), std::string::npos);
  EXPECT_EQ(fixed_lighting_shader.find("EE_RASTER_REFLECTION_PROBES"), std::string::npos);
  EXPECT_NE(render_instances.find("transition_parameters.z = glm::floatBitsToUint"), std::string::npos);
}

TEST(ReflectionProbe, PrefilterSupportsAContiguousFaceRange) {
  const auto probe = ReadTextFile(SourcePath("EvoEngine_SDK/src/GlobalReflectionProbe.cpp"));
  ASSERT_FALSE(probe.empty());
  const auto prefilter = ExtractBetween(probe, "void GlobalReflectionProbe::RecordPrefilter(",
                                        "void GlobalReflectionProbe::RecordPrefilterFaces(");
  const auto face_range = ExtractBetween(probe, "void GlobalReflectionProbe::RecordPrefilterFaces(",
                                         "void GlobalReflectionProbe::MarkBaked");
  ASSERT_FALSE(prefilter.empty());
  ASSERT_FALSE(face_range.empty());
  EXPECT_NE(prefilter.find("RecordPrefilterFaces"), std::string::npos);
  EXPECT_NE(prefilter.find("0u, 6u"), std::string::npos);
  EXPECT_NE(face_range.find("first_face + face_count > 6u"), std::string::npos);
  EXPECT_NE(face_range.find("face = first_face; face < first_face + face_count"), std::string::npos);
}

TEST(ReflectionProbe, BakeAllReusesFramePointSpotAndDirectionalShadows) {
  const auto inspector = ReadTextFile(SourcePath("EvoEngine_SDK/src/Editor/SDKInspectionAdapters.cpp"));
  const auto render_layer = ReadTextFile(SourcePath("EvoEngine_SDK/src/RenderLayer.cpp"));
  const auto deferred_pass = ReadTextFile(SourcePath("EvoEngine_SDK/src/RenderPasses/DeferredComputeLightingPass.cpp"));
  const auto lighting =
      ReadTextFile(SourcePath("EvoEngine_SDK/Internals/DefaultResources/Shaders/Modules/EvoEngine/Lighting.slang"));
  ASSERT_FALSE(inspector.empty());
  ASSERT_FALSE(render_layer.empty());
  ASSERT_FALSE(deferred_pass.empty());
  ASSERT_FALSE(lighting.empty());

  const auto queue = ExtractBetween(inspector, "uint32_t QueueEnvironmentalLightingLocalProbeBakes",
                                    "void InspectEnvironmentalLightingLocalProbePayload");
  const auto record = ExtractBetween(render_layer, "void RenderLayer::RecordPreparedReflectionProbeBake",
                                     "void RenderLayer::PublishSubmittedReflectionProbeBake");
  const auto render_all = ExtractBetween(render_layer, "void RenderLayer::RenderAll", "void RenderLayer::RenderGizmos");
  ASSERT_FALSE(queue.empty());
  ASSERT_FALSE(record.empty());
  ASSERT_FALSE(render_all.empty());

  EXPECT_NE(queue.find("QueueGlobalReflectionProbeBakeBatch"), std::string::npos);
  EXPECT_EQ(queue.find("QueueEnvironmentalLightingLocalProbeBake(context, probe)"), std::string::npos);
  EXPECT_EQ(render_layer.find("GetReflectionProbeCaptureFingerprint"), std::string::npos);
  EXPECT_EQ(render_layer.find("source_fingerprint"), std::string::npos);
  EXPECT_EQ(record.find("Platform::WaitForFrameSubmissions"), std::string::npos);
  EXPECT_EQ(record.find("PreparePointAndSpotLightShadowMap"), std::string::npos);
  EXPECT_NE(render_all.find("PreparePointAndSpotLightShadowMap();"), std::string::npos);
  EXPECT_NE(record.find("for (size_t request_index = 0; request_index < requests.size(); ++request_index)"),
            std::string::npos);
  EXPECT_NE(record.find("directional_shadow_camera_index"), std::string::npos);
  EXPECT_NE(render_all.find("RecordPreparedReflectionProbeBake(current_render_instances)"), std::string::npos);
  EXPECT_NE(deferred_pass.find("-parameters.directional_shadow_camera_index - 1"), std::string::npos);
  EXPECT_NE(lighting.find("EE_FUNC_DIRECTIONAL_SHADOW_CAMERA_INDEX"), std::string::npos);
  EXPECT_NE(lighting.find("EE_CAMERAS[directionalShadowCameraIndex].view"), std::string::npos);
  EXPECT_NE(render_layer.find("preferred_shadow_camera = main_camera"), std::string::npos);
  EXPECT_NE(render_layer.find("preferred_shadow_camera = scene_camera"), std::string::npos);
}

TEST(ReflectionProbe, ExplicitBakeRecordsCpuAndGpuStageTimings) {
  const auto render_layer = ReadTextFile(SourcePath("EvoEngine_SDK/src/RenderLayer.cpp"));
  ASSERT_FALSE(render_layer.empty());
  const auto prepare = ExtractBetween(render_layer, "void RenderLayer::PrepareReflectionProbeBake",
                                      "void RenderLayer::EnsureReflectionProbeCaptureRenderGraph");
  const auto record = ExtractBetween(render_layer, "void RenderLayer::RecordPreparedReflectionProbeBake",
                                     "void RenderLayer::PublishSubmittedReflectionProbeBake");
  ASSERT_FALSE(prepare.empty());
  ASSERT_FALSE(record.empty());

  EXPECT_EQ(render_layer.find("Reflection Probe Bake Fingerprint CPU"), std::string::npos);
  EXPECT_EQ(render_layer.find("Reflection Probe Bake Frame Drain CPU"), std::string::npos);
  EXPECT_EQ(render_layer.find("Reflection Probe Bake Submit Wait CPU"), std::string::npos);
  EXPECT_NE(prepare.find("Reflection Probe Bake Prepare CPU"), std::string::npos);
  EXPECT_NE(record.find("Reflection Probe Bake Record CPU"), std::string::npos);
  EXPECT_NE(record.find("Reflection Probe Bake GPU Total"), std::string::npos);
  EXPECT_NE(record.find("Reflection Probe Face Capture"), std::string::npos);
  EXPECT_NE(record.find("Reflection Probe GGX Prefilter"), std::string::npos);
}

TEST(ReflectionProbe, DemoSceneReflectionProbeValidationUsesEnvironmentalLightingEntries) {
  const auto demo_scene = ReadTextFile(SourcePath("EvoEngine_App/src/DemoScene.cpp"));
  ASSERT_FALSE(demo_scene.empty());

  EXPECT_NE(demo_scene.find("AddReflectionProbeValidationLocalProbe"), std::string::npos);
  EXPECT_NE(demo_scene.find("RunEnvironmentalLightingLocalProbeBake"), std::string::npos);
  EXPECT_NE(demo_scene.find("RunEnvironmentalLightingLocalProbeBakeBatch"), std::string::npos);
  EXPECT_NE(demo_scene.find("QueueGlobalReflectionProbeBakeBatch"), std::string::npos);
  EXPECT_NE(demo_scene.find("Platform::ResetGpuTimestampStats()"), std::string::npos);
  EXPECT_NE(demo_scene.find("Reflection Probe Bake GPU Total"), std::string::npos);
  EXPECT_NE(demo_scene.find("Reflection Probe Bake Prepare CPU"), std::string::npos);
  EXPECT_NE(demo_scene.find("Reflection Probe Bake Record CPU"), std::string::npos);
  EXPECT_NE(demo_scene.find("EVOENGINE_SPONZA_LOCAL_PROBE_BATCH_TIMING"), std::string::npos);
  EXPECT_NE(demo_scene.find("SetEnvironmentalLightingFallbackIntensities(*lighting, 0.0f, 1.0f);"), std::string::npos);
  EXPECT_NE(demo_scene.find("editor_layer->OpenAssetInspector(lighting)"), std::string::npos);
  EXPECT_NE(demo_scene.find("capture(\"debug-bounds-off\")"), std::string::npos);
  EXPECT_NE(demo_scene.find("capture(\"debug-bounds-on\")"), std::string::npos);
  EXPECT_NE(demo_scene.find("FindEnvironmentalLightingLocalReflectionProbe(scene, kSponzaLocalProbeNames.front())"),
            std::string::npos);
  EXPECT_NE(demo_scene.find("probe_transform.SetScale(definition.size)"), std::string::npos);
  EXPECT_NE(demo_scene.find("constexpr float blend_distance = 0.03f;"), std::string::npos);
  EXPECT_NE(demo_scene.find("LocalReflectionProbeShape::Box, 1.0f, blend_distance, true"), std::string::npos);
  EXPECT_EQ(demo_scene.find("definition.blend_distance"), std::string::npos);
  EXPECT_NE(demo_scene.find("{4.0f, 6.2f, 14.5f}"), std::string::npos);
  EXPECT_NE(demo_scene.find("{3.0f, 5.0f, 14.5f}"), std::string::npos);
  EXPECT_NE(demo_scene.find("{3.4f, 5.0f, 5.8f}"), std::string::npos);
  EXPECT_EQ(demo_scene.find("definition.extents"), std::string::npos);
  EXPECT_EQ(demo_scene.find("probe.box_extents"), std::string::npos);
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

TEST(ReflectionProbe, EnvironmentalLightingInspectorUsesNormalizedTrsAuthoring) {
  const auto inspector = ReadTextFile(SourcePath("EvoEngine_SDK/src/Editor/SDKInspectionAdapters.cpp"));
  const auto editor = ReadTextFile(SourcePath("EvoEngine_SDK/src/EditorLayer.cpp"));
  ASSERT_FALSE(inspector.empty());
  ASSERT_FALSE(editor.empty());
  const auto authoring =
      ExtractBetween(inspector, "bool AuthoringTransformsEqual", "const char* GetGlobalReflectionProbeSourceKindName");
  const auto helpers = ExtractBetween(editor, "glm::mat4 EditorLayer::ComposeAuthoringTransform",
                                      "void EditorLayer::SetEnvironmentalLightingGizmoTarget");
  ASSERT_FALSE(authoring.empty());
  ASSERT_FALSE(helpers.empty());

  EXPECT_NE(helpers.find("decomposed_transform.Decompose(position, rotation_degrees, scale)"), std::string::npos);
  EXPECT_NE(helpers.find("glm::translate(position) * glm::mat4_cast(glm::quat(glm::radians(rotation_degrees))) *"),
            std::string::npos);
  EXPECT_NE(helpers.find("glm::determinant(glm::mat3(transform))"), std::string::npos);
  EXPECT_NE(authoring.find("transform = glm::mat4(1.0f)"), std::string::npos);
  EXPECT_NE(authoring.find("AuthoringTransformsEqual(transform, normalized)"), std::string::npos);
  EXPECT_NE(authoring.find("ImGui::DragFloat3(\"##AuthoringPosition\""), std::string::npos);
  EXPECT_NE(authoring.find("ImGui::DragFloat3(\"##AuthoringRotation\""), std::string::npos);
  EXPECT_NE(authoring.find("ImGui::DragFloat3(\"##AuthoringScale\""), std::string::npos);
  EXPECT_NE(authoring.find("ImGui::Selectable(\"Position##Authoring\""), std::string::npos);
  EXPECT_NE(authoring.find("ImGui::Selectable(\"Rotation##Authoring\""), std::string::npos);
  EXPECT_NE(authoring.find("ImGui::Selectable(\"Scale##Authoring\""), std::string::npos);
  EXPECT_EQ(authoring.find("Column 0"), std::string::npos);
  EXPECT_EQ(authoring.find("DragFloat4"), std::string::npos);
  EXPECT_NE(inspector.find("InspectAuthoringTransform(editor_layer, \"Transform\", probe.transform)"),
            std::string::npos);
  EXPECT_NE(inspector.find("DragFloat(\"Blend distance\", &probe.blend_distance, 0.01f"), std::string::npos);
}

TEST(ReflectionProbe, EnvironmentalLightingBoundsUseFilledDepthTestedVolumes) {
  const auto inspector = ReadTextFile(SourcePath("EvoEngine_SDK/src/Editor/SDKInspectionAdapters.cpp"));
  const auto render_layer = ReadTextFile(SourcePath("EvoEngine_SDK/src/RenderLayer.cpp"));
  const auto render_instance_storage = ReadTextFile(SourcePath("EvoEngine_SDK/src/RenderInstanceStorage.cpp"));
  const auto gizmo_constants = ReadTextFile(
      SourcePath("EvoEngine_SDK/Internals/DefaultResources/Shaders/Modules/EvoEngine/GizmosConstants.slang"));
  ASSERT_FALSE(inspector.empty());
  ASSERT_FALSE(render_layer.empty());
  ASSERT_FALSE(render_instance_storage.empty());
  ASSERT_FALSE(gizmo_constants.empty());
  const auto bounds = ExtractBetween(inspector, "GizmoSettings MakeBoundingVolumeGizmoSettings() {",
                                     "void RenderReflectionProbeBounds");
  const auto ddgi_bounds = ExtractBetween(inspector, "void InspectDdgiRuntime", "if (ImGui::TreeNodeEx(\"Overview\"");
  const auto gizmos = ExtractBetween(render_layer, "void RenderLayer::RenderGizmos() const",
                                     "void RenderLayer::ForEachCollectedCamera");
  ASSERT_FALSE(bounds.empty());
  ASSERT_FALSE(ddgi_bounds.empty());
  ASSERT_FALSE(gizmos.empty());

  EXPECT_NE(bounds.find("VK_POLYGON_MODE_FILL"), std::string::npos);
  EXPECT_NE(bounds.find("cull_mode = VK_CULL_MODE_NONE"), std::string::npos);
  EXPECT_NE(bounds.find("blending = true"), std::string::npos);
  EXPECT_NE(bounds.find("gizmo_settings.depth_test = true"), std::string::npos);
  EXPECT_NE(bounds.find("gizmo_settings.depth_write = false"), std::string::npos);
  EXPECT_EQ(bounds.find("VK_POLYGON_MODE_LINE"), std::string::npos);
  EXPECT_NE(bounds.find("DrawGizmoCube(color, probe.transform, 1.0f, gizmo_settings)"), std::string::npos);
  EXPECT_NE(bounds.find("glm::max(probe.sphere_radius, 0.001f)"), std::string::npos);
  EXPECT_NE(bounds.find("if (!editor_layer || (!include_disabled && !probe.enabled))"), std::string::npos);
  EXPECT_NE(bounds.find("if (probe.debug_draw_bounds &&"), std::string::npos);
  EXPECT_NE(bounds.find("glm::vec4(0.1f, 0.8f, 1.0f, 0.55f), true"), std::string::npos);
  EXPECT_NE(bounds.find("RenderEnvironmentalLightingProbeBound(editor_layer, probe, color)"), std::string::npos);
  EXPECT_NE(ddgi_bounds.find("MakeBoundingVolumeGizmoSettings()"), std::string::npos);
  EXPECT_NE(ddgi_bounds.find("glm::vec4(1.0f, 0.45f, 0.05f, 0.35f)"), std::string::npos);
  EXPECT_EQ(ddgi_bounds.find("VK_POLYGON_MODE_LINE"), std::string::npos);
  EXPECT_EQ(inspector.find("Box half extents"), std::string::npos);
  EXPECT_EQ(inspector.find("probe.box_extents"), std::string::npos);
  EXPECT_NE(render_instance_storage.find("info.shape_parameters = glm::vec4(glm::vec3(0.5f), probe.sphere_radius)"),
            std::string::npos);
  EXPECT_EQ(gizmos.find("RecordCommandsMainQueue([&]"), std::string::npos);
  size_t capture_count = 0;
  for (size_t position = 0; (position = gizmos.find("[this, i, current_frame_index", position)) != std::string::npos;
       position += 1u) {
    ++capture_count;
  }
  EXPECT_EQ(capture_count, 3u);
  EXPECT_EQ(gizmo_constants.find("float4x4(1.0f)"), std::string::npos);
  EXPECT_NE(gizmo_constants.find("float4x4(size, 0.0f, 0.0f, 0.0f"), std::string::npos);
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
  source->MarkBaked();

  YAML::Emitter local_out;
  local_out << YAML::BeginMap;
  Serialization::SerializeObject(local_out, *source);
  local_out << YAML::EndMap;
  const auto local_node = YAML::Load(local_out.c_str());
  EXPECT_EQ(local_node["schema_version"].as<uint32_t>(), GlobalReflectionProbe::kSchemaVersion);
  EXPECT_FALSE(local_node["source_fingerprint"]);
  const auto local_restored = AssetManager::CreateTemporaryAsset<GlobalReflectionProbe>();
  ASSERT_TRUE(local_restored);
  Serialization::DeserializeObject(local_node, *local_restored);
  EXPECT_EQ(local_restored->GetSourceKind(), GlobalReflectionProbe::SourceKind::Baked);
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
  EXPECT_EQ(restored->GetPayloadHash(), source->GetPayloadHash());
  EXPECT_EQ(restored->GetCanonicalPayloadByteSize(), GlobalReflectionProbe::kCanonicalPayloadByteSize);
  ASSERT_EQ(restored->GetCanonicalPayload().size(), payload.size());
  EXPECT_TRUE(
      std::equal(restored->GetCanonicalPayload().begin(), restored->GetCanonicalPayload().end(), payload.begin()));
  EXPECT_TRUE(restored->SupportsStagedLoading());
  const auto restored_kind = restored->GetSourceKind();
  const auto restored_hash = restored->GetPayloadHash();
  const auto restored_payload = restored->GetCanonicalPayload();

  const auto malformed_path = directory.Path() / "malformed.evereflectionprobe";
  {
    std::ofstream malformed(malformed_path, std::ios::trunc);
    malformed << "schema_version: 1\nformat: 97\npixels: !!binary AQID\n";
  }
  EXPECT_FALSE(Serialization::LoadAsset(*restored, malformed_path));
  EXPECT_EQ(restored->GetSourceKind(), restored_kind);
  EXPECT_EQ(restored->GetPayloadHash(), restored_hash);
  EXPECT_EQ(restored->GetCanonicalPayload(), restored_payload);

  const auto truncated_path = directory.Path() / "truncated.evereflectionprobe";
  {
    std::ofstream truncated(truncated_path, std::ios::trunc);
  }
  EXPECT_FALSE(Serialization::LoadAsset(*restored, truncated_path));
  EXPECT_EQ(restored->GetSourceKind(), restored_kind);
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
  EXPECT_EQ(restored->GetPayloadHash(), restored_hash);
  EXPECT_EQ(restored->GetCanonicalPayload(), restored_payload);

  const auto previous_schema_path = directory.Path() / "previous-schema.evereflectionprobe";
  auto previous_schema_node = YAML::LoadFile(asset_path.string());
  previous_schema_node["schema_version"] = 1u;
  previous_schema_node["source_fingerprint"] = 123u;
  {
    std::ofstream previous_schema(previous_schema_path, std::ios::trunc);
    previous_schema << previous_schema_node;
  }
  EXPECT_FALSE(Serialization::LoadAsset(*restored, previous_schema_path));
  EXPECT_EQ(restored->GetSourceKind(), restored_kind);
  EXPECT_EQ(restored->GetPayloadHash(), restored_hash);
  EXPECT_EQ(restored->GetCanonicalPayload(), restored_payload);

  const auto legacy_path = directory.Path() / "legacy-empty.evereflectionprobe";
  {
    std::ofstream legacy(legacy_path, std::ios::trunc);
    legacy << "{}\n";
  }
  EXPECT_FALSE(Serialization::LoadAsset(*restored, legacy_path));
  EXPECT_EQ(restored->GetSourceKind(), restored_kind);
  EXPECT_EQ(restored->GetPayloadHash(), restored_hash);
  EXPECT_EQ(restored->GetCanonicalPayload(), restored_payload);
}
