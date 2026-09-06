#include "EvoEngine_SDK_PCH.hpp"

#include "Application.hpp"
#include "ApplicationContext.hpp"
#include "ApplicationInitializationSettings.hpp"
#include "AssetManager.hpp"
#include "DdgiProbeRayData.hpp"
#include "DdgiRuntime.hpp"
#include "DdgiSampling.hpp"
#include "EnvironmentalLighting.hpp"
#include "EnvironmentalLightingResolver.hpp"
#include "PointCloudSample.hpp"
#include "RenderLayer.hpp"
#include "RenderPasses/DdgiProbeRayVisualizationPass.hpp"
#include "RenderPasses/DdgiProbeUpdatePass.hpp"
#include "RenderPasses/DdgiProbeVisualizationPass.hpp"
#include "Scene.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <numeric>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <glm/gtc/constants.hpp>

using namespace evo_engine;

TEST(DdgiHistoryTest, SupportedWindowsAndCheckedInteriorStorage) {
  for (int count = -1; count <= 35; ++count) {
    DdgiHistoryLayout layout;
    const bool supported = count >= 5 && count <= 30 && count % 5 == 0;
    ASSERT_EQ(DdgiHistoryLayout::Calculate(8192, 8, 16, count, layout), supported);
    if (!supported)
      continue;
    EXPECT_EQ(layout.buffer_bytes[DdgiHistoryLayout::IrradianceRing], 8192ull * 64 * count * 8);
    EXPECT_EQ(layout.buffer_bytes[DdgiHistoryLayout::IrradianceSum], 8192ull * 64 * 16);
    EXPECT_EQ(layout.buffer_bytes[DdgiHistoryLayout::VisibilityRing], 8192ull * 256 * count * 4);
    EXPECT_EQ(layout.buffer_bytes[DdgiHistoryLayout::VisibilitySum], 8192ull * 256 * 8);
  }
  DdgiHistoryLayout layout;
  ASSERT_TRUE(DdgiHistoryLayout::Calculate(1, 8, 16, 30, layout));
  const auto previous = layout.buffer_bytes;
  EXPECT_FALSE(DdgiHistoryLayout::Calculate((std::numeric_limits<uint64_t>::max)(), 128, 128, 30, layout));
  EXPECT_EQ(layout.buffer_bytes, previous);
  EXPECT_FALSE(DdgiHistoryLayout::Calculate(0, 8, 16, 30, layout));
  EXPECT_FALSE(DdgiHistoryLayout::Calculate(1, 0, 16, 30, layout));
}

TEST(DdgiHistoryTest, PaddedAllocationsShareStrictBudgetAtomically) {
  GiHistoryBudget budget;
  budget.bytes = kGiHistoryBudgetBytes - 1025;
  const std::array<uint64_t, 5> padded{256, 256, 256, 128, 128};
  ASSERT_TRUE(DdgiHistoryLayout::AddAllocationBytes(padded, budget));
  EXPECT_EQ(budget.bytes, kGiHistoryBudgetBytes - 1);
  const auto previous = budget.bytes;
  EXPECT_FALSE(DdgiHistoryLayout::AddAllocationBytes({0, 0, 0, 1}, budget));
  EXPECT_EQ(budget.bytes, previous);
  budget.bytes = kGiHistoryBudgetBytes - 1024;
  EXPECT_FALSE(DdgiHistoryLayout::AddAllocationBytes(padded, budget));
  EXPECT_EQ(budget.bytes, kGiHistoryBudgetBytes - 1024);
}

TEST(DdgiHistoryTest, QuantizedReplacementIsExactlyStableAfterFullWindow) {
  for (const uint32_t count : {5u, 10u, 15u, 20u, 25u, 30u}) {
    std::array<uint16_t, 30> ring{};
    uint32_t sum = 0;
    uint32_t complete_sum = 0;
    for (uint32_t update = 0; update < count * 1000; ++update) {
      const auto phase = update % count;
      const auto sample = QuantizeDdgiHistorySample(static_cast<float>(phase + 1), 64.0f);
      sum = sum - ring[phase] + sample;
      ring[phase] = sample;
      if (update == 0)
        EXPECT_NEAR(DecodeDdgiHistorySum(sum, 64.0f, count), 1.0f / count, 0.001f);
      if (update == count - 1)
        complete_sum = sum;
      if (update >= count)
        EXPECT_EQ(sum, complete_sum);
    }
    EXPECT_NEAR(DecodeDdgiHistorySum(sum, 64.0f, count), (count + 1) * 0.5f, 0.001f);
  }
}

TEST(DdgiHistoryTest, QuantizationBoundsAndVisibilityDecode) {
  EXPECT_EQ(QuantizeDdgiHistorySample(-1.0f, 64.0f), 0);
  EXPECT_EQ(QuantizeDdgiHistorySample(128.0f, 64.0f), 65535);
  EXPECT_EQ(QuantizeDdgiHistorySample(std::numeric_limits<float>::quiet_NaN(), 64.0f), 0);
  EXPECT_EQ(QuantizeDdgiHistorySample(1.0f, 0.0f), 0);
  for (const float bound : {0.0005f, 0.5f, 32.0f, 2048.0f}) {
    const auto sample = QuantizeDdgiHistorySample(bound * 0.25f, bound);
    EXPECT_NEAR(DecodeDdgiHistorySum(sample * 30u, bound, 30), bound * 0.25f, bound / 65535.0f);
    EXPECT_FLOAT_EQ(DecodeDdgiHistorySum(65535u * 30u, bound, 30), bound);
  }
}

TEST(DdgiHistoryTest, SettingsDefaultsMigrationAndAuthoredWindows) {
  DdgiSettings settings;
  EXPECT_EQ(settings.runtime.history_count, 30);
  settings.runtime.history_count = 5;
  DeserializeDdgiSettings(YAML::Load("runtime: {enabled: true}"), settings);
  EXPECT_EQ(settings.runtime.history_count, 30);
  for (int count = 5; count <= 30; count += 5) {
    settings.runtime.history_count = count;
    YAML::Emitter out;
    SerializeDdgiSettings(out, settings);
    DdgiSettings restored;
    DeserializeDdgiSettings(YAML::Load(out.c_str()), restored);
    EXPECT_EQ(restored.runtime.history_count, count);
  }
  DeserializeDdgiSettings(YAML::Load("runtime: {history_count: 7}"), settings);
  EXPECT_EQ(settings.runtime.history_count, 7);
  EXPECT_FALSE(DdgiRuntime::CalculateFrameResourceLayout(settings).valid);
}

TEST(DdgiHistoryTest, WindowChangesInvalidateLayoutButRayChangesDoNot) {
  DdgiSettings settings;
  const auto initial = DdgiRuntime::CalculateFrameResourceLayout(settings);
  ASSERT_TRUE(initial.valid) << initial.error;
  settings.runtime.history_count = 5;
  const auto shorter = DdgiRuntime::CalculateFrameResourceLayout(settings);
  ASSERT_TRUE(shorter.valid) << shorter.error;
  EXPECT_FALSE(DdgiRuntime::ArePersistentLayoutsCompatible(initial, shorter));
  settings.runtime.history_count = 30;
  settings.runtime.ray_count = 256;
  EXPECT_TRUE(
      DdgiRuntime::ArePersistentLayoutsCompatible(initial, DdgiRuntime::CalculateFrameResourceLayout(settings)));
  const auto range = *std::max_element(initial.history.buffer_bytes.begin(), initial.history.buffer_bytes.end());
  const auto unsupported = DdgiRuntime::CalculateFrameResourceLayout(settings, initial.probe_count, 32768, range - 1);
  EXPECT_FALSE(unsupported.valid);
}

TEST(DdgiHistoryTest, BothRaySequencesRepeatPerWindowAndUseStableVolumeSeeds) {
  for (uint32_t count = 5; count <= 30; count += 5) {
    for (uint32_t phase = 0; phase < count; ++phase) {
      const auto expected = CreateDdgiSamplingPhase(0xabcdef1234567890ull, phase, count, 42);
      EXPECT_NEAR(glm::length(expected.rotation), 1.0f, 0.000001f);
      for (uint32_t cycle = 1; cycle < 10; ++cycle) {
        const auto repeated = CreateDdgiSamplingPhase(0xabcdef1234567890ull, phase + cycle * count, count, 42);
        EXPECT_EQ(repeated.rotation, expected.rotation);
        EXPECT_EQ(repeated.emissive_seed, expected.emissive_seed);
      }
      EXPECT_NE(CreateDdgiSamplingPhase(0xabcdef1234567891ull, phase, count, 42).emissive_seed, expected.emissive_seed);
      EXPECT_NE(CreateDdgiSamplingPhase(0xabcdef1234567890ull, phase, count, 43).rotation, expected.rotation);
    }
  }
}

TEST(DdgiHistoryTest, ReflectionCaptureRequiresWindowAndRelocationWarmup) {
  EXPECT_FALSE(DdgiRuntime::IsReflectionProbeRuntimeReady(false, true, true, false));
  EXPECT_FALSE(DdgiRuntime::IsReflectionProbeRuntimeReady(true, false, true, false));
  EXPECT_FALSE(DdgiRuntime::IsReflectionProbeRuntimeReady(true, true, false, false));
  EXPECT_FALSE(DdgiRuntime::IsReflectionProbeRuntimeReady(true, true, true, true));
  EXPECT_TRUE(DdgiRuntime::IsReflectionProbeRuntimeReady(true, true, true, false));
}

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

std::string ExtractBetween(const std::string& source, const std::string& begin, const std::string& end) {
  const auto begin_offset = source.find(begin);
  if (begin_offset == std::string::npos) {
    return {};
  }
  const auto end_offset = source.find(end, begin_offset + begin.size());
  return end_offset == std::string::npos ? std::string{} : source.substr(begin_offset, end_offset - begin_offset);
}

size_t CountOccurrences(const std::string& source, const std::string& value) {
  size_t count = 0;
  for (auto offset = source.find(value); offset != std::string::npos;
       offset = source.find(value, offset + value.size())) {
    ++count;
  }
  return count;
}

uint32_t PackOctahedralDirection(const glm::vec3& input_direction) {
  const auto denominator = std::abs(input_direction.x) + std::abs(input_direction.y) + std::abs(input_direction.z);
  auto direction =
      denominator > 0.0f && std::isfinite(denominator) ? input_direction / denominator : glm::vec3(0.0f, 0.0f, 1.0f);
  auto encoded = glm::vec2(direction);
  if (direction.z < 0.0f) {
    const auto sign_not_zero = glm::vec2(encoded.x >= 0.0f ? 1.0f : -1.0f, encoded.y >= 0.0f ? 1.0f : -1.0f);
    encoded = (glm::vec2(1.0f) - glm::abs(glm::vec2(encoded.y, encoded.x))) * sign_not_zero;
  }
  const auto pack_snorm = [](const float value) {
    return static_cast<uint16_t>(static_cast<int16_t>(
        std::round(glm::clamp(value, -1.0f, 1.0f) * static_cast<float>((std::numeric_limits<int16_t>::max)()))));
  };
  return static_cast<uint32_t>(pack_snorm(encoded.x)) | static_cast<uint32_t>(pack_snorm(encoded.y)) << 16u;
}

glm::vec3 UnpackOctahedralDirection(const uint32_t packed_direction) {
  const auto unpack_snorm = [](const uint16_t value) {
    return (std::max)(static_cast<float>(static_cast<int16_t>(value)) /
                          static_cast<float>((std::numeric_limits<int16_t>::max)()),
                      -1.0f);
  };
  const auto encoded =
      glm::vec2(unpack_snorm(static_cast<uint16_t>(packed_direction)), unpack_snorm(packed_direction >> 16u));
  auto direction = glm::vec3(encoded, 1.0f - std::abs(encoded.x) - std::abs(encoded.y));
  if (direction.z < 0.0f) {
    const auto sign_not_zero = glm::vec2(direction.x >= 0.0f ? 1.0f : -1.0f, direction.y >= 0.0f ? 1.0f : -1.0f);
    direction =
        glm::vec3((glm::vec2(1.0f) - glm::abs(glm::vec2(direction.y, direction.x))) * sign_not_zero, direction.z);
  }
  return glm::normalize(direction);
}
}  // namespace

TEST(DdgiVolume, SessionDebugStateIsTransient) {
  YAML::Emitter emitter;
  SerializeDdgiSettings(emitter, DdgiSettings{});
  const std::string yaml = emitter.c_str();
  EXPECT_EQ(yaml.find("pause_updates"), std::string::npos);
  EXPECT_EQ(yaml.find("reset_probe_history"), std::string::npos);
  EXPECT_EQ(yaml.find("debug:"), std::string::npos);
  EXPECT_NE(yaml.find("ray_count: 192"), std::string::npos);
  EXPECT_NE(yaml.find("emissive_ray_count: 64"), std::string::npos);
  EXPECT_EQ(yaml.find("enable_probe_variability"), std::string::npos);
  EXPECT_EQ(yaml.find("pause_probe_updates_after_convergence"), std::string::npos);
  EXPECT_EQ(yaml.find("random_ray_backface_threshold"), std::string::npos);
  EXPECT_EQ(yaml.find("fixed_ray_backface_threshold"), std::string::npos);
  EXPECT_EQ(yaml.find("probe_variability_threshold"), std::string::npos);
  EXPECT_EQ(yaml.find("probe_variability_min_samples"), std::string::npos);

  RenderLayer::DdgiSessionState session;
  EXPECT_FALSE(session.pause_updates);
  EXPECT_FALSE(session.reset_history_requested);
  EXPECT_FALSE(session.show_probes);
  EXPECT_FALSE(session.show_rays);
  EXPECT_EQ(session.selected_volume_id, 0u);
}

TEST(DdgiVolume, RuntimeHelperContractsPreserveLayoutsAndSelection) {
  EXPECT_TRUE(DdgiSettings{}.runtime.enable_emissive_mesh_sampling);
  EXPECT_EQ(sizeof(DdgiProbeRayData), 16u);
  EXPECT_EQ(alignof(DdgiProbeRayData), 16u);
  EXPECT_EQ(offsetof(DdgiProbeRayData, radiance_and_signed_distance), 0u);
  EXPECT_EQ(sizeof(DdgiProbeRaySampleInfo), 8u);
  EXPECT_EQ(alignof(DdgiProbeRaySampleInfo), 8u);
  EXPECT_EQ(offsetof(DdgiProbeRaySampleInfo, packed_direction_and_inverse_pdf), 0u);
  EXPECT_EQ(DdgiSettings{}.runtime.ray_count, 192);
  EXPECT_EQ(DdgiSettings{}.runtime.emissive_ray_count, 64);
  EXPECT_EQ(sizeof(PointCloudSample), 128u);
  EXPECT_EQ(sizeof(DdgiProbeRayTracingPushConstant), 128u);
  EXPECT_EQ(offsetof(DdgiProbeRayTracingPushConstant, selected_probe_volume_flags_environment), 80u);
  EXPECT_EQ(sizeof(DdgiProbeAtlasUpdatePushConstant), 160u);
  EXPECT_EQ(offsetof(DdgiProbeAtlasUpdatePushConstant, atlas_columns_fixed_ray_count_and_update_mode), 16u);
  EXPECT_EQ(offsetof(DdgiProbeAtlasUpdatePushConstant, probe_counts_and_rotation), 32u);
  EXPECT_EQ(offsetof(DdgiProbeAtlasUpdatePushConstant, update_parameters), 48u);
  EXPECT_EQ(offsetof(DdgiProbeAtlasUpdatePushConstant, blend_parameters), 64u);
  EXPECT_EQ(offsetof(DdgiProbeAtlasUpdatePushConstant, probe_scroll_offset), 80u);
  EXPECT_EQ(offsetof(DdgiProbeAtlasUpdatePushConstant, probe_scroll_delta), 96u);
  EXPECT_EQ(offsetof(DdgiProbeAtlasUpdatePushConstant, probe_step_x), 112u);
  EXPECT_EQ(offsetof(DdgiProbeAtlasUpdatePushConstant, probe_step_y), 128u);
  EXPECT_EQ(offsetof(DdgiProbeAtlasUpdatePushConstant, probe_step_z), 144u);
  EXPECT_EQ(sizeof(DdgiProbeRelocationPushConstant), 128u);
  EXPECT_EQ(offsetof(DdgiProbeRelocationPushConstant, probe_scroll_offset), 48u);
  EXPECT_EQ(offsetof(DdgiProbeRelocationPushConstant, probe_scroll_delta), 64u);
  EXPECT_EQ(offsetof(DdgiProbeRelocationPushConstant, probe_step_x), 80u);
  EXPECT_EQ(sizeof(DdgiProbeVisualizationPushConstant), 32u);
  EXPECT_EQ(offsetof(DdgiProbeVisualizationPushConstant, radius_intensity_alpha_selected_scale), 16u);
  EXPECT_EQ(sizeof(DdgiProbeRayVisualizationPushConstant), 16u);
  EXPECT_EQ(offsetof(DdgiProbeRayVisualizationPushConstant, miss_distance_alpha), 8u);
  EXPECT_FLOAT_EQ(kDdgiProbeRayMissDistance, 1e27f);
  EXPECT_FLOAT_EQ(kDdgiProbeRayInactiveDistance, -1e27f);
  EXPECT_FLOAT_EQ(kDdgiProbeRayBackfaceDistanceScale, -0.2f);

  EXPECT_EQ(DdgiRuntime::GetFixedRayCount(1u, true), 0u);
  EXPECT_EQ(DdgiRuntime::GetFixedRayCount(2u, true), 1u);
  EXPECT_EQ(DdgiRuntime::GetFixedRayCount(32u, true), 31u);
  EXPECT_EQ(DdgiRuntime::GetFixedRayCount(33u, true), 32u);
  EXPECT_EQ(DdgiRuntime::GetFixedRayCount(128u, true), 32u);
  EXPECT_EQ(DdgiRuntime::GetFixedRayCount(128u, false), 0u);

  DdgiSettings settings{};
  settings.runtime.enabled = true;
  settings.storage.max_probe_count = 8192;
  settings.storage.atlas_probe_columns = 16;

  const auto runtime_layout = DdgiRuntime::CalculateFrameResourceLayout(settings, 256u, 4096u);
  EXPECT_TRUE(runtime_layout.valid);
  EXPECT_EQ(runtime_layout.probe_count, 256u);

  std::vector<DdgiVolumeRuntimeInfo> infos(2);
  infos[0].stable_entity_id = 20u;
  infos[0].artist_priority = 1;
  infos[0].probe_density = 1.0f;
  infos[1].stable_entity_id = 10u;
  infos[1].artist_priority = 2;
  infos[1].probe_density = 0.5f;
  DdgiRuntime::SortVolumeRuntimeInfos(infos);
  ASSERT_EQ(infos.size(), 2u);
  EXPECT_EQ(infos[0].stable_entity_id, 10u);
  EXPECT_EQ(infos[1].stable_entity_id, 20u);

  using RenderVariant = DdgiProbeUpdateVariant;
  DdgiProbeUpdateDeviceLimits limits{64u, 64u, 65535u, 65535u, DdgiRuntime::kProbeUpdateSharedMemoryBytes};

  EXPECT_EQ(DdgiRuntime::ParseProbeUpdateVariant("serial"), RenderVariant::Serial);
  EXPECT_EQ(DdgiRuntime::ParseProbeUpdateVariant("parallel-direct"), RenderVariant::ParallelDirect);
  EXPECT_EQ(DdgiRuntime::ParseProbeUpdateVariant("parallel-shared"), RenderVariant::ParallelShared);
  EXPECT_EQ(DdgiRuntime::ParseProbeUpdateVariant("invalid"), RenderVariant::Serial);
  EXPECT_EQ(DdgiRuntime::ResolveProbeUpdateVariant(RenderVariant::ParallelDirect, limits, 8192u, true, true),
            RenderVariant::ParallelDirect);
  EXPECT_EQ(DdgiRuntime::ResolveProbeUpdateVariant(RenderVariant::ParallelShared, limits, 8192u, true, true),
            RenderVariant::ParallelShared);
  EXPECT_EQ(DdgiRuntime::ResolveProbeUpdateVariant(RenderVariant::Serial, {}, 0u, false, false), RenderVariant::Serial);
  EXPECT_EQ(DdgiRuntime::ResolveProbeUpdateVariant(RenderVariant::ParallelDirect, limits, 0u, true, true),
            RenderVariant::Serial);

  limits.max_shared_memory_bytes = 0u;
  EXPECT_EQ(DdgiRuntime::ResolveProbeUpdateVariant(RenderVariant::ParallelDirect, limits, 1u, true, true),
            RenderVariant::ParallelDirect);
  limits.max_shared_memory_bytes = DdgiRuntime::kProbeUpdateSharedMemoryBytes;

  --limits.max_shared_memory_bytes;
  EXPECT_EQ(DdgiRuntime::ResolveProbeUpdateVariant(RenderVariant::ParallelShared, limits, 1u, true, true),
            RenderVariant::Serial);
  limits.max_shared_memory_bytes = DdgiRuntime::kProbeUpdateSharedMemoryBytes;
  limits.max_work_group_invocations = 63u;
  EXPECT_EQ(DdgiRuntime::ResolveProbeUpdateVariant(RenderVariant::ParallelDirect, limits, 1u, true, true),
            RenderVariant::Serial);
  limits.max_work_group_invocations = 64u;
  limits.max_work_group_size_x = 63u;
  EXPECT_EQ(DdgiRuntime::ResolveProbeUpdateVariant(RenderVariant::ParallelDirect, limits, 1u, true, true),
            RenderVariant::Serial);
  limits.max_work_group_size_x = 64u;
  limits.max_work_group_count_x = 2u;
  limits.max_work_group_count_y = 2u;
  EXPECT_EQ(DdgiRuntime::ResolveProbeUpdateVariant(RenderVariant::ParallelDirect, limits, 4u, true, true),
            RenderVariant::ParallelDirect);
  EXPECT_EQ(DdgiRuntime::ResolveProbeUpdateVariant(RenderVariant::ParallelDirect, limits, 5u, true, true),
            RenderVariant::Serial);
  limits.max_work_group_count_x = limits.max_work_group_count_y = 65535u;
  EXPECT_EQ(DdgiRuntime::ResolveProbeUpdateVariant(RenderVariant::ParallelDirect, limits, 1u, false, true),
            RenderVariant::Serial);
  EXPECT_EQ(DdgiRuntime::ResolveProbeUpdateVariant(RenderVariant::ParallelDirect, limits, 1u, true, false),
            RenderVariant::Serial);
}

TEST(DdgiVolume, ExactEmissiveResourcesUseDefaultPresetAndRemainOptional) {
  DdgiSettings settings;
  auto layout = DdgiRuntime::CalculateFrameResourceLayout(settings, 32u);
  ASSERT_TRUE(layout.valid) << layout.error;
  EXPECT_EQ(layout.ray_sample_info_byte_size, 32ull * 64ull * sizeof(DdgiProbeRaySampleInfo));
  EXPECT_EQ(layout.ray_output_byte_size, 32ull * 256ull * sizeof(DdgiProbeRayData));
  EXPECT_EQ(layout.selected_ray_diagnostics_byte_size, 256ull * sizeof(PointCloudSample));

  settings.runtime.ray_count = 96;
  settings.runtime.emissive_ray_count = 32;
  layout = DdgiRuntime::CalculateFrameResourceLayout(settings, 384u);
  ASSERT_TRUE(layout.valid) << layout.error;
  EXPECT_EQ(layout.ray_sample_info_byte_size, 384ull * 32ull * sizeof(DdgiProbeRaySampleInfo));

  settings.runtime.ray_count = 192;
  settings.runtime.enable_emissive_mesh_sampling = false;
  layout = DdgiRuntime::CalculateFrameResourceLayout(settings, 32u);
  ASSERT_TRUE(layout.valid) << layout.error;
  EXPECT_EQ(layout.ray_sample_info_byte_size, 0u);
  EXPECT_EQ(layout.ray_output_byte_size, 32ull * 192ull * sizeof(DdgiProbeRayData));
}

TEST(DdgiVolume, ExactEmissiveEstimatorUsesIndependentPdfAndPopulation) {
  constexpr double kRadiance = 4.0;
  constexpr double kCosine = 0.8;
  constexpr double kSolidAnglePdf = 2.0;
  constexpr uint32_t kEmissiveRayCount = 64u;
  double accumulated = 0.0;
  for (uint32_t index = 0; index < kEmissiveRayCount; ++index) {
    accumulated += kRadiance * kCosine / kSolidAnglePdf;
  }
  const auto result = accumulated / (2.0 * glm::pi<double>() * kEmissiveRayCount);
  EXPECT_NEAR(result, kRadiance * kCosine / (2.0 * glm::pi<double>() * kSolidAnglePdf), 1e-12);
}

TEST(DdgiVolume, CompactEmissiveRayMetadataPreservesDirectionAndPdfAccuracy) {
  constexpr uint32_t kDirectionCount = 1u << 16u;
  float maximum_angular_error_degrees = 0.0f;
  for (uint32_t index = 0u; index < kDirectionCount; ++index) {
    const auto z = 1.0f - 2.0f * (static_cast<float>(index) + 0.5f) / kDirectionCount;
    const auto phi = glm::two_pi<float>() * glm::fract(static_cast<float>(index) * 0.61803398875f);
    const auto radial = std::sqrt((std::max)(0.0f, 1.0f - z * z));
    const auto direction = glm::vec3(radial * std::cos(phi), radial * std::sin(phi), z);
    const auto packed = PackOctahedralDirection(direction);
    EXPECT_EQ(PackOctahedralDirection(direction), packed);
    const auto decoded = UnpackOctahedralDirection(packed);
    const auto direction_length =
        std::sqrt(static_cast<double>(direction.x) * direction.x + static_cast<double>(direction.y) * direction.y +
                  static_cast<double>(direction.z) * direction.z);
    const auto decoded_length =
        std::sqrt(static_cast<double>(decoded.x) * decoded.x + static_cast<double>(decoded.y) * decoded.y +
                  static_cast<double>(decoded.z) * decoded.z);
    const auto direction_dot =
        glm::clamp((static_cast<double>(direction.x) * decoded.x + static_cast<double>(direction.y) * decoded.y +
                    static_cast<double>(direction.z) * decoded.z) /
                       (direction_length * decoded_length),
                   -1.0, 1.0);
    const auto angular_error_degrees = glm::degrees(std::acos(direction_dot));
    maximum_angular_error_degrees =
        (std::max)(maximum_angular_error_degrees, static_cast<float>(angular_error_degrees));
  }
  EXPECT_LT(maximum_angular_error_degrees, 0.005f);

  for (const float inverse_pdf : {0.0f, 1.0f, 4.0f * glm::pi<float>(), 1.0e-8f, 1.0e8f}) {
    const DdgiProbeRaySampleInfo sample_info{glm::uvec2(
        PackOctahedralDirection(glm::normalize(glm::vec3(1.0f, -2.0f, 3.0f))), glm::floatBitsToUint(inverse_pdf))};
    EXPECT_EQ(glm::uintBitsToFloat(sample_info.packed_direction_and_inverse_pdf.y), inverse_pdf);
  }
}

TEST(DdgiVolume, DdgiProbeUpdateSchedulingCoversTexelsRaysAndBordersExactlyOnce) {
  constexpr uint32_t group_size = DdgiRuntime::kProbeUpdateGroupSize;
  for (uint32_t tile_size = 1u; tile_size <= 128u; ++tile_size) {
    const uint32_t texel_count = tile_size * tile_size;
    std::vector<uint32_t> direct_visits(texel_count, 0u);
    std::vector<uint32_t> shared_visibility_visits(texel_count, 0u);
    for (uint32_t lane = 0u; lane < group_size; ++lane) {
      for (uint32_t texel = lane; texel < texel_count; texel += group_size) {
        ++direct_visits[texel];
      }
      for (uint32_t batch = 0u; batch < texel_count; batch += group_size * 4u) {
        for (uint32_t slot = 0u; slot < 4u; ++slot) {
          const uint32_t texel = batch + lane + slot * group_size;
          if (texel < texel_count) {
            ++shared_visibility_visits[texel];
          }
        }
      }
    }
    EXPECT_TRUE(std::all_of(direct_visits.begin(), direct_visits.end(), [](const uint32_t count) {
      return count == 1u;
    })) << tile_size;
    EXPECT_TRUE(std::all_of(shared_visibility_visits.begin(), shared_visibility_visits.end(), [](const uint32_t count) {
      return count == 1u;
    })) << tile_size;

    const uint32_t stride = tile_size + 2u;
    std::vector<uint32_t> border_visits(stride * stride, 0u);
    for (uint32_t border_index = 0u; border_index < 4u * tile_size + 4u; ++border_index) {
      glm::uvec2 texel;
      if (border_index < stride) {
        texel = {border_index, 0u};
      } else if (border_index < 2u * stride) {
        texel = {border_index - stride, stride - 1u};
      } else if (const uint32_t column = border_index - 2u * stride; column < tile_size) {
        texel = {0u, column + 1u};
      } else {
        const uint32_t right_column = border_index - 2u * stride;
        texel = {stride - 1u, right_column - tile_size + 1u};
      }
      ++border_visits[texel.y * stride + texel.x];
    }
    for (uint32_t y = 0u; y < stride; ++y) {
      for (uint32_t x = 0u; x < stride; ++x) {
        const bool border = x == 0u || y == 0u || x + 1u == stride || y + 1u == stride;
        EXPECT_EQ(border_visits[y * stride + x], border ? 1u : 0u) << tile_size << ":" << x << "," << y;
      }
    }
  }

  for (const uint32_t ray_count : {1u, 2u, 31u, 32u, 33u, 63u, 64u, 65u, 255u, 256u, 257u, 4096u}) {
    const uint32_t first_ray = ray_count > 1u ? glm::min(32u, ray_count - 1u) : 0u;
    std::vector<uint32_t> visited;
    for (uint32_t chunk = first_ray; chunk < ray_count; chunk += 256u) {
      const uint32_t chunk_count = glm::min(256u, ray_count - chunk);
      for (uint32_t index = 0u; index < chunk_count; ++index) {
        visited.emplace_back(chunk + index);
      }
    }
    ASSERT_EQ(visited.size(), ray_count - first_ray);
    for (uint32_t index = 0u; index < visited.size(); ++index) {
      EXPECT_EQ(visited[index], first_ray + index) << ray_count;
    }
  }
}

TEST(DdgiVolume, DdgiProbeUpdateDispatchUsesTwoDimensionsAndSerialFallbackShape) {
  auto dispatch = DdgiProbeUpdatePass::CalculateDispatchSize(0u, true, 65535u, 65535u);
  EXPECT_FALSE(dispatch.valid);
  dispatch = DdgiProbeUpdatePass::CalculateDispatchSize(4u, true, 2u, 2u);
  EXPECT_TRUE(dispatch.valid);
  EXPECT_EQ(dispatch.x, 2u);
  EXPECT_EQ(dispatch.y, 2u);
  dispatch = DdgiProbeUpdatePass::CalculateDispatchSize(5u, true, 2u, 2u);
  EXPECT_FALSE(dispatch.valid);
  dispatch = DdgiProbeUpdatePass::CalculateDispatchSize(65u, false, 65535u, 65535u);
  EXPECT_TRUE(dispatch.valid);
  EXPECT_EQ(dispatch.x, 2u);
  EXPECT_EQ(dispatch.y, 1u);
  dispatch = DdgiProbeUpdatePass::CalculateDispatchSize(16777216u, false, 65535u, 65535u);
  EXPECT_TRUE(dispatch.valid);
  EXPECT_EQ(dispatch.x, 65535u);
  EXPECT_EQ(dispatch.y, 5u);
}

TEST(DdgiVolume, DefaultProbeGridMatchesSceneAuthoringDefaults) {
  EnvironmentalLighting::DdgiVolume volume;
  DdgiSettings settings;
  RenderSettings render_settings;

  EXPECT_EQ(volume.probe_counts, glm::ivec3(10, 6, 16));
  EXPECT_EQ(volume.probe_spacing, glm::vec3(1.5f));
  EXPECT_EQ(volume.volume_origin, glm::vec3(0.0f, 3.0f, 3.0f));
  EXPECT_EQ(volume.movement_type, static_cast<int>(DdgiVolumeMovementType::Default));
  EXPECT_EQ(volume.emissive_mesh_sampling_mode, static_cast<int>(DdgiEmissiveMeshSamplingMode::Inherit));
  EXPECT_TRUE(volume.enable_probe_relocation);
  EXPECT_FLOAT_EQ(volume.relocation_distance, 0.25f);
  EXPECT_EQ(volume.GetProbeAmount(), 960u);
  EXPECT_FLOAT_EQ(volume.GetProbeLocalPosition({0, 0, 0}).x, -6.75f);
  EXPECT_FLOAT_EQ(volume.GetProbeLocalPosition({0, 0, 0}).y, -0.75f);
  EXPECT_FLOAT_EQ(volume.GetProbeLocalPosition({0, 0, 0}).z, -8.25f);
  const auto& defaults = settings.volume_defaults;
  EXPECT_EQ(defaults.probe_counts, volume.probe_counts);
  EXPECT_EQ(defaults.probe_spacing, volume.probe_spacing);
  EXPECT_EQ(defaults.volume_origin, volume.volume_origin);
  EXPECT_FLOAT_EQ(settings.runtime.normal_bias, 0.1f);
  EXPECT_FLOAT_EQ(settings.runtime.view_bias, 0.1f);
  EXPECT_FLOAT_EQ(render_settings.ddgi_random_ray_backface_threshold, 0.1f);
  EXPECT_FLOAT_EQ(render_settings.ddgi_fixed_ray_backface_threshold, 0.25f);
  EXPECT_FLOAT_EQ(settings.runtime.max_ray_distance, 1e27f);
  EXPECT_FLOAT_EQ(settings.runtime.visibility_moment_bias, 0.02f);
  EXPECT_EQ(settings.runtime.warmup_frames, 16);
  EXPECT_FLOAT_EQ(settings.runtime.distance_exponent, 50.0f);
  EXPECT_EQ(settings.runtime.history_count, 30);
  EXPECT_TRUE(settings.runtime.enable_emissive_mesh_sampling);
  EXPECT_EQ(defaults.movement_type, static_cast<int>(DdgiVolumeMovementType::Default));
  EXPECT_TRUE(defaults.enable_probe_relocation);
  EXPECT_FLOAT_EQ(defaults.relocation_distance, 0.25f);
  EXPECT_EQ(DdgiRuntime::GetAllocatedProbeCount(settings), 960u);
}

TEST(DdgiVolume, ProbePositionsAreCenteredAroundVolumeOrigin) {
  EnvironmentalLighting::DdgiVolume volume;
  volume.probe_counts = {3, 2, 1};
  volume.probe_spacing = glm::vec3(2.0f);
  volume.volume_origin = {1.0f, 2.0f, 3.0f};
  volume.ClampSettings();

  EXPECT_EQ(volume.GetProbeAmount(), 6u);
  EXPECT_FLOAT_EQ(volume.GetLocalGridSize().x, 4.0f);
  EXPECT_FLOAT_EQ(volume.GetLocalGridSize().y, 2.0f);
  EXPECT_FLOAT_EQ(volume.GetLocalGridSize().z, 0.0f);

  const auto first_probe = volume.GetProbeLocalPosition({0, 0, 0});
  EXPECT_FLOAT_EQ(first_probe.x, -1.0f);
  EXPECT_FLOAT_EQ(first_probe.y, 1.0f);
  EXPECT_FLOAT_EQ(first_probe.z, 3.0f);

  const auto last_probe = volume.GetProbeLocalPosition({2, 1, 0});
  EXPECT_FLOAT_EQ(last_probe.x, 3.0f);
  EXPECT_FLOAT_EQ(last_probe.y, 3.0f);
  EXPECT_FLOAT_EQ(last_probe.z, 3.0f);

  volume.probe_counts = {4, 2, 1};
  volume.ClampSettings();
  const auto expanded_first_probe = volume.GetProbeLocalPosition({0, 0, 0});
  EXPECT_FLOAT_EQ(expanded_first_probe.x, -2.0f);
  EXPECT_FLOAT_EQ(expanded_first_probe.y, 1.0f);
  EXPECT_FLOAT_EQ(expanded_first_probe.z, 3.0f);
}

TEST(DdgiVolume, DdgiAppCornellComparisonAvoidsWallAnchoredProbes) {
  const auto profile_header =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_App" / "include" / "DemoProfiles.hpp");
  const auto profile_source =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_App" / "src" / "DemoProfiles.cpp");
  const auto app_source =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_App" / "src" / "DDGIApp.cpp");
  const auto runner_source =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "Scripts" / "run_ddgi_app_validation.py");
  ASSERT_FALSE(profile_header.empty());
  ASSERT_FALSE(profile_source.empty());
  ASSERT_FALSE(app_source.empty());
  ASSERT_FALSE(runner_source.empty());

  EXPECT_NE(profile_source.find("const glm::vec3 kDdgiCornellBoxVolumeOrigin = {0.0f, 0.0f, 0.0f};"),
            std::string::npos);
  EXPECT_NE(profile_source.find("const glm::ivec3 kDdgiCornellBoxProbeCounts = {13, 13, 14};"), std::string::npos);
  EXPECT_NE(profile_source.find("constexpr float kDdgiCornellBoxProbeSpacing = 0.14333334f;"), std::string::npos);
  EXPECT_NE(profile_header.find("float point_light_brightness = 2.0f;"), std::string::npos);
  EXPECT_NE(profile_header.find("float indirect_lighting_intensity = 1.0f;"), std::string::npos);
  EXPECT_NE(profile_source.find("const auto lighting = GetOrCreateTemporaryEnvironmentalLighting(scene);"),
            std::string::npos);
  EXPECT_NE(profile_source.find("auto& ddgi_settings = lighting->ddgi_settings;"), std::string::npos);
  EXPECT_NE(profile_source.find("target_volume.probe_spacing = glm::vec3(kDdgiCornellBoxProbeSpacing);"),
            std::string::npos);
  EXPECT_EQ(profile_source.find("GetOrSetPrivateComponent<DdgiVolume>"), std::string::npos);
  EXPECT_EQ(profile_source.find("UnsafeGetPrivateComponentOwnersList<DdgiVolume>"), std::string::npos);
  EXPECT_EQ(profile_source.find("FindEntityByName(scene, \"DDGI Probe Volume\")"), std::string::npos);
  EXPECT_NE(profile_header.find("bool enable_probe_relocation = true;"), std::string::npos);
  EXPECT_NE(profile_header.find("bool enable_probe_classification = true;"), std::string::npos);
  EXPECT_NE(profile_source.find("void DisablePostProcessing(const std::shared_ptr<Scene>& scene)"), std::string::npos);
  EXPECT_NE(profile_source.find("main_camera->post_processing_stack_ref.Clear();"), std::string::npos);
  EXPECT_NE(profile_source.find("camera->post_processing_stack_ref.Clear();"), std::string::npos);
  EXPECT_NE(profile_source.find("DisablePostProcessing(scene);"), std::string::npos);
  EXPECT_NE(profile_source.find("target_volume.relocation_distance = kDdgiCornellBoxProbeSpacing * 0.5f;"),
            std::string::npos);
  EXPECT_NE(app_source.find("argument == \"--disable-probe-relocation\""), std::string::npos);
  EXPECT_NE(app_source.find("argument == \"--disable-probe-classification\""), std::string::npos);
  EXPECT_NE(app_source.find("argument == \"--indirect-lighting-intensity\""), std::string::npos);
  EXPECT_EQ(app_source.find("--ddgi-indirect-intensity"), std::string::npos);
  EXPECT_NE(app_source.find("argument == \"--width\""), std::string::npos);
  EXPECT_NE(app_source.find("argument == \"--height\""), std::string::npos);
  EXPECT_NE(app_source.find("application_info.default_window_size ="), std::string::npos);
  EXPECT_NE(app_source.find("main_camera->Resize({command_line.width, command_line.height})"), std::string::npos);
  EXPECT_NE(app_source.find("DDGI_APP_DDGI_READY resolution="), std::string::npos);
  EXPECT_NE(app_source.find("observed_probe_update_count == 0"), std::string::npos);
  EXPECT_NE(app_source.find("observed_ray_sample_count == 0"), std::string::npos);
  EXPECT_NE(app_source.find("!performance.lighting_descriptors_bound"), std::string::npos);
  EXPECT_NE(runner_source.find("(args.width, args.height) != (1920, 1080)"), std::string::npos);
  EXPECT_NE(runner_source.find("environment[\"EVOENGINE_IMGUI_INI_PATH\"] = str(isolated_imgui_path)"),
            std::string::npos);
  EXPECT_NE(runner_source.find("\"launch_count\": 1"), std::string::npos);
  EXPECT_NE(runner_source.find("\"benchmark_gate\": \"four_frozen_m2_holdouts\""), std::string::npos);
}

TEST(DdgiVolume, RenderingDemoOffsetsWallAdjacentProbes) {
  const auto demo_source =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_App" / "src" / "DemoScene.cpp");
  const auto demo_app_source =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_App" / "src" / "DemoApp.cpp");
  const auto python_binding_source =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "PythonBinding" / "src" / "PyEvoEngine.cpp");
  ASSERT_FALSE(demo_source.empty());
  ASSERT_FALSE(demo_app_source.empty());
  ASSERT_FALSE(python_binding_source.empty());
  const auto rendering_scene_source =
      ExtractBetween(demo_source, "// DDGI_VALIDATION_RENDERING_SCENE_BEGIN", "// DDGI_VALIDATION_RENDERING_SCENE_END");
  const auto python_ddgi_ready_source = ExtractBetween(
      python_binding_source, "bool PyEvoEngine::IsCurrentSceneDdgiEnabled()", "\n}\n\nvoid PyEvoEngine::Run(");
  ASSERT_FALSE(rendering_scene_source.empty());
  ASSERT_FALSE(python_ddgi_ready_source.empty());

  EXPECT_NE(demo_source.find("AddEnvironmentalLightingDdgiVolume("), std::string::npos);
  EXPECT_NE(rendering_scene_source.find("ResetEnvironmentalLightingDdgiVolume(*lighting, \"DDGI Probe Volume\""),
            std::string::npos);
  EXPECT_NE(demo_source.find("lighting.GetOrCreateDdgiVolumePack()->volumes.clear();"), std::string::npos);
  EXPECT_EQ(rendering_scene_source.find("SetEnvironmentalLightingFallbackIntensities(*lighting, 1.0f, 1.0f)"),
            std::string::npos);
  EXPECT_NE(rendering_scene_source.find("environment, 1.0f, lighting->diffuse_fallback_intensity"), std::string::npos);
  EXPECT_NE(rendering_scene_source.find("lighting->specular_fallback_intensity"), std::string::npos);
  EXPECT_EQ(rendering_scene_source.find("GetOrSetPrivateComponent<DdgiVolume>"), std::string::npos);
  EXPECT_EQ(rendering_scene_source.find("CreateReflectionProbeComponent(scene"), std::string::npos);
  EXPECT_NE(rendering_scene_source.find("{10, 8, 16}"), std::string::npos);
  EXPECT_NE(rendering_scene_source.find("scene->SetEnable(capoeira_entity, true);"), std::string::npos);
  EXPECT_NE(demo_source.find("ddgi_volume.relocation_distance = 0.25f;"), std::string::npos);
  EXPECT_NE(demo_source.find("session.show_probes = true;"), std::string::npos);
  EXPECT_NE(demo_source.find("ddgi_volume.enable_probe_relocation = true;"), std::string::npos);
  EXPECT_NE(demo_source.find("ConfigureMaterial(moving_light_material, glm::vec3(1.0f, 0.8f, 0.0f), 1.0f, "
                             "1.0f, 2.0f);"),
            std::string::npos);
  EXPECT_NE(demo_source.find("moving_light->diffuse_brightness = 24.0f;"), std::string::npos);
  EXPECT_NE(demo_source.find("DisableImportedLightsRecursive(scene, sponza_entity);"), std::string::npos);
  EXPECT_NE(demo_source.find("DisableLightIfPresent<DirectionalLight>(scene, entity);"), std::string::npos);
  EXPECT_NE(demo_source.find("DisableLightIfPresent<PointLight>(scene, entity);"), std::string::npos);
  EXPECT_NE(demo_source.find("DisableLightIfPresent<SpotLight>(scene, entity);"), std::string::npos);
  EXPECT_NE(demo_app_source.find("ResolveEnvironmentalLighting(scene)"), std::string::npos);
  EXPECT_NE(demo_app_source.find("FindEnvironmentalLightingDdgiVolume(scene, \"DDGI Probe Volume\")"),
            std::string::npos);
  EXPECT_NE(demo_app_source.find("ddgi_settings.runtime.ray_count != 192"), std::string::npos);
  EXPECT_NE(demo_app_source.find("ddgi_settings.runtime.emissive_ray_count != 64"), std::string::npos);
  EXPECT_NE(demo_app_source.find("GetDdgiSessionState().show_probes"), std::string::npos);
  EXPECT_NE(demo_app_source.find("volume.probe_counts != glm::ivec3(10, 8, 16)"), std::string::npos);
  EXPECT_NE(demo_app_source.find("volume.probe_spacing != glm::vec3(1.5f)"), std::string::npos);
  EXPECT_NE(demo_app_source.find("volume.volume_origin != glm::vec3(0.0f, 3.0f, 3.0f)"), std::string::npos);
  EXPECT_NE(demo_app_source.find("volume.relocation_distance != 0.25f"), std::string::npos);
  EXPECT_NE(demo_app_source.find("!volume.enable_probe_relocation || volume.enable_probe_classification"),
            std::string::npos);
  EXPECT_NE(demo_app_source.find("Cornell environmental lighting asset is missing for DDGI validation"),
            std::string::npos);
  EXPECT_NE(demo_app_source.find("thin-wall environmental lighting asset is missing for DDGI validation"),
            std::string::npos);
  EXPECT_EQ(demo_app_source.find("FindDdgiProbeVolume"), std::string::npos);
  EXPECT_EQ(demo_app_source.find("UnsafeGetPrivateComponentOwnersList<DdgiVolume>"), std::string::npos);
  EXPECT_NE(python_ddgi_ready_source.find("ResolveEnvironmentalLighting(scene)"), std::string::npos);
  EXPECT_EQ(python_ddgi_ready_source.find("UnsafeGetPrivateComponentOwnersList<DdgiVolume>"), std::string::npos);
}

TEST(DdgiVolume, CornellAndThinWallDemosUseEnvironmentalLightingVolumes) {
  const auto demo_source =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_App" / "src" / "DemoScene.cpp");
  ASSERT_FALSE(demo_source.empty());
  const auto cornell_scene_source =
      ExtractBetween(demo_source, "// DDGI_VALIDATION_CORNELL_SCENE_BEGIN", "// DDGI_VALIDATION_CORNELL_SCENE_END");
  const auto thin_wall_scene_source =
      ExtractBetween(demo_source, "void ConfigureThinWallDdgi", "void RemoveGeneratedFiles");
  ASSERT_FALSE(cornell_scene_source.empty());
  ASSERT_FALSE(thin_wall_scene_source.empty());

  EXPECT_NE(cornell_scene_source.find("const auto lighting = GetOrCreateTemporaryEnvironmentalLighting(scene);"),
            std::string::npos);
  EXPECT_NE(cornell_scene_source.find("ResetEnvironmentalLightingDdgiVolume(*lighting, \"DDGI Probe Volume\""),
            std::string::npos);
  EXPECT_NE(cornell_scene_source.find("{9, 9, 9}, glm::vec3(0.3f), glm::vec3(0.0f)"), std::string::npos);
  EXPECT_EQ(cornell_scene_source.find("GetOrSetPrivateComponent<DdgiVolume>"), std::string::npos);
  EXPECT_EQ(cornell_scene_source.find("SyncTemporaryEnvironmentalLightingFromLegacyScene(scene)"), std::string::npos);

  EXPECT_NE(thin_wall_scene_source.find("const auto lighting = GetOrCreateTemporaryEnvironmentalLighting(scene);"),
            std::string::npos);
  EXPECT_NE(thin_wall_scene_source.find("ResetEnvironmentalLightingDdgiVolume(*lighting, \"DDGI Probe Volume\""),
            std::string::npos);
  EXPECT_NE(thin_wall_scene_source.find("{8, 6, 8}, glm::vec3(0.35f), glm::vec3(0.0f)"), std::string::npos);
  EXPECT_EQ(thin_wall_scene_source.find("GetOrSetPrivateComponent<DdgiVolume>"), std::string::npos);
  EXPECT_EQ(thin_wall_scene_source.find("SyncTemporaryEnvironmentalLightingFromLegacyScene(scene)"), std::string::npos);
}

TEST(DdgiVolume, GeneratedValidationFixturesUseEnvironmentalLightingVolumes) {
  const auto demo_source =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_App" / "src" / "DemoScene.cpp");
  const auto editor_source =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_App" / "src" / "EvoEngineEditor.cpp");
  ASSERT_FALSE(demo_source.empty());
  ASSERT_FALSE(editor_source.empty());
  const auto fixture_source = ExtractBetween(demo_source, "// DDGI_VALIDATION_FIXTURE_IMPLEMENTATION_BEGIN",
                                             "// DDGI_VALIDATION_FIXTURE_IMPLEMENTATION_END");
  const auto environment_validation_source =
      ExtractBetween(demo_source, "void evo_engine::ConfigureEnvironmentLightingValidationScene",
                     "namespace {\nstd::shared_ptr<GlobalReflectionProbe>");
  ASSERT_FALSE(fixture_source.empty());
  ASSERT_FALSE(environment_validation_source.empty());

  EXPECT_NE(fixture_source.find("AddEnvironmentalLightingDdgiVolume(*lighting, \"DDGI Validation Volume\""),
            std::string::npos);
  EXPECT_NE(fixture_source.find("FindEnvironmentalLightingDdgiVolume(scene, \"DDGI Validation Volume\")"),
            std::string::npos);
  EXPECT_NE(fixture_source.find("SyncTemporaryEnvironmentalLightingSettingsFromScene(scene)"), std::string::npos);
  EXPECT_NE(fixture_source.find("DDGI Enabled Emitter"), std::string::npos);
  EXPECT_NE(fixture_source.find("DDGI Moving Occluder"), std::string::npos);
  EXPECT_NE(fixture_source.find("kDdgiValidationEqualPowerSmallEmitterRadiance"), std::string::npos);
  EXPECT_NE(editor_source.find("\"emissive-enable\""), std::string::npos);
  EXPECT_NE(editor_source.find("\"geometry-moving\""), std::string::npos);
  EXPECT_EQ(fixture_source.find("GetOrSetPrivateComponent<DdgiVolume>"), std::string::npos);
  EXPECT_EQ(fixture_source.find("UnsafeGetPrivateComponentOwnersList<DdgiVolume>"), std::string::npos);
  EXPECT_EQ(fixture_source.find("SyncTemporaryEnvironmentalLightingFromLegacyScene(scene)"), std::string::npos);

  EXPECT_NE(
      environment_validation_source.find("FindEnvironmentalLightingDdgiVolume(scene, \"DDGI Validation Volume\")"),
      std::string::npos);
  EXPECT_NE(environment_validation_source.find("SyncTemporaryEnvironmentalLightingSettingsFromScene(scene)"),
            std::string::npos);
  EXPECT_NE(environment_validation_source.find("SetEnvironmentalLightingFallbackIntensities(*lighting, 1.0f, 1.0f)"),
            std::string::npos);
  EXPECT_EQ(environment_validation_source.find("GetOrSetPrivateComponent<DdgiVolume>"), std::string::npos);
  EXPECT_EQ(environment_validation_source.find("UnsafeGetPrivateComponentOwnersList<DdgiVolume>"), std::string::npos);
  EXPECT_EQ(environment_validation_source.find("SyncTemporaryEnvironmentalLightingFromLegacyScene(scene)"),
            std::string::npos);
  EXPECT_NE(editor_source.find("auto& ddgi = lighting->ddgi_settings;"), std::string::npos);
  EXPECT_NE(editor_source.find("environmental_lighting.Get<EnvironmentalLighting>()->ddgi_settings"),
            std::string::npos);
  EXPECT_EQ(editor_source.find("GetOrSetPrivateComponent<DdgiVolume>"), std::string::npos);
  EXPECT_EQ(editor_source.find("UnsafeGetPrivateComponentOwnersList<DdgiVolume>"), std::string::npos);
}

TEST(DdgiVolume, BistroDemoUsesEnvironmentalLightingVolume) {
  const auto demo_source =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_App" / "src" / "DemoScene.cpp");
  ASSERT_FALSE(demo_source.empty());
  const auto bistro_ddgi_source =
      ExtractBetween(demo_source, "void ConfigureBistroDemoDdgi", "void ApplyBistroDirectionalLightIntensity");
  ASSERT_FALSE(bistro_ddgi_source.empty());
  const auto bistro_scene_source =
      ExtractBetween(demo_source, "void evo_engine::ConfigureBistroDemoScene", "std::filesystem::path evo_engine");
  ASSERT_FALSE(bistro_scene_source.empty());

  EXPECT_NE(demo_source.find("const glm::ivec3 kBistroDdgiProbeCounts = glm::ivec3(22, 7, 26)"), std::string::npos);
  EXPECT_NE(demo_source.find("const glm::vec3 kBistroDdgiProbeSpacing = glm::vec3(4.0f)"), std::string::npos);
  EXPECT_NE(demo_source.find("const glm::vec3 kBistroDdgiVolumeOrigin = glm::vec3(-10.0f, 10.0f, -15.0f)"),
            std::string::npos);
  EXPECT_NE(bistro_ddgi_source.find("const auto lighting = GetOrCreateTemporaryEnvironmentalLighting(scene);"),
            std::string::npos);
  EXPECT_NE(bistro_ddgi_source.find("lighting->environment_lighting_intensity = "
                                    "EnvironmentalLighting::kDefaultEnvironmentLightingIntensity"),
            std::string::npos);
  EXPECT_NE(bistro_ddgi_source.find("SetEnvironmentalLightingFallbackIntensities(*lighting, 0.0f, 0.0f)"),
            std::string::npos);
  EXPECT_NE(bistro_ddgi_source.find("auto& settings = lighting->ddgi_settings;"), std::string::npos);
  EXPECT_NE(bistro_ddgi_source.find("settings.runtime.enable_emissive_mesh_sampling = true;"), std::string::npos);
  EXPECT_NE(bistro_ddgi_source.find("lighting->GetOrCreateReflectionProbePack()->probes.clear();"), std::string::npos);
  EXPECT_NE(bistro_ddgi_source.find("lighting->GetOrCreateDdgiVolumePack()->volumes.clear();"), std::string::npos);
  EXPECT_NE(bistro_ddgi_source.find("AddEnvironmentalLightingDdgiVolume(*lighting, kBistroDdgiVolumeName"),
            std::string::npos);
  EXPECT_EQ(CountOccurrences(bistro_scene_source,
                             "camera_settings.background_source = "
                             "Camera::BackgroundSource::InheritEnvironmentalLighting"),
            2u);
  EXPECT_EQ(CountOccurrences(bistro_scene_source, "camera_settings.background_intensity = 1.0f"), 2u);
  EXPECT_EQ(bistro_ddgi_source.find("scene->environment.ddgi_settings"), std::string::npos);
  EXPECT_NE(bistro_ddgi_source.find("scene->DeleteEntity(*existing_ddgi_volume);"), std::string::npos);
  EXPECT_EQ(bistro_ddgi_source.find("GetOrSetPrivateComponent<DdgiVolume>"), std::string::npos);
  EXPECT_EQ(bistro_ddgi_source.find("UnsafeGetPrivateComponentOwnersList<DdgiVolume>"), std::string::npos);
  EXPECT_EQ(bistro_ddgi_source.find("SyncTemporaryEnvironmentalLightingFromLegacyScene(scene)"), std::string::npos);
}

TEST(DdgiVolume, MultiVolumeValidationUsesEnvironmentalLightingVolumes) {
  const auto demo_source =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_App" / "src" / "DemoScene.cpp");
  ASSERT_FALSE(demo_source.empty());
  const auto validation_source =
      ExtractBetween(demo_source, "bool evo_engine::RunDdgiMultiVolumeValidationFromEnvironment",
                     "void evo_engine::ConfigureStrandMeshShaderValidation");
  ASSERT_FALSE(validation_source.empty());

  EXPECT_NE(validation_source.find("const auto lighting = GetOrCreateTemporaryEnvironmentalLighting(scene);"),
            std::string::npos);
  EXPECT_NE(validation_source.find("lighting->GetOrCreateDdgiVolumePack()->volumes.clear();"), std::string::npos);
  EXPECT_NE(validation_source.find("AddEnvironmentalLightingDdgiVolume(*lighting"), std::string::npos);
  EXPECT_EQ(validation_source.find("scene->environment.ddgi_settings"), std::string::npos);
  EXPECT_NE(validation_source.find("glm::translate(scrolling_volume->transform"), std::string::npos);
  EXPECT_NE(validation_source.find("std::remove_if(ddgi_pack->volumes.begin()"), std::string::npos);
  EXPECT_EQ(validation_source.find("GetOrSetPrivateComponent<DdgiVolume>"), std::string::npos);
  EXPECT_EQ(validation_source.find("UnsafeGetPrivateComponentOwnersList<DdgiVolume>"), std::string::npos);
  EXPECT_EQ(validation_source.find("SyncTemporaryEnvironmentalLightingFromLegacyScene(scene)"), std::string::npos);
  EXPECT_EQ(validation_source.find("GetDataComponent<Transform>(scrolling_entity)"), std::string::npos);
  EXPECT_EQ(validation_source.find("DeleteEntity(dense_entity)"), std::string::npos);
}

TEST(DdgiVolume, ClampSettingsPreservesRejectedProbeGridForDiagnostics) {
  EnvironmentalLighting::DdgiVolume volume;
  volume.probe_counts = {-4, 0, 512};
  volume.probe_spacing = {-1.0f, 0.01f, 20000.0f};
  volume.movement_type = 4;
  volume.emissive_mesh_sampling_mode = -4;
  volume.relocation_distance = -1.0f;
  volume.ClampSettings();

  EXPECT_EQ(volume.probe_counts, glm::ivec3(-4, 0, 512));
  EXPECT_EQ(volume.GetProbeAmount(), 0u);
  EXPECT_EQ(volume.probe_spacing, glm::vec3(0.05f, 0.05f, 10000.0f));
  EXPECT_EQ(volume.movement_type, static_cast<int>(DdgiVolumeMovementType::Scrolling));
  EXPECT_EQ(volume.emissive_mesh_sampling_mode, static_cast<int>(DdgiEmissiveMeshSamplingMode::Inherit));
  EXPECT_FLOAT_EQ(volume.relocation_distance, 0.0f);
  volume.emissive_mesh_sampling_mode = 4;
  volume.ClampSettings();
  EXPECT_EQ(volume.emissive_mesh_sampling_mode, static_cast<int>(DdgiEmissiveMeshSamplingMode::Off));
}

TEST(DdgiVolume, ProbeGridIndexUsesXFastestOrder) {
  EXPECT_EQ(DdgiRuntime::GetProbeGridIndex({4, 3, 2}, 17), glm::uvec3(1, 1, 1));
  EXPECT_EQ(DdgiRuntime::GetProbeGridIndex({4, 3, 2}, 23), glm::uvec3(3, 2, 1));
  EXPECT_EQ(DdgiRuntime::GetProbeGridIndex({4, 3, 2}, 99), glm::uvec3(3, 2, 1));
  EXPECT_EQ(DdgiRuntime::GetProbeGridIndex({0, 3, 2}, 0), glm::uvec3(0));
}

TEST(DdgiVolume, RenderLayerFrameResourceLayoutRejectsProbeCapOverflowWithoutTruncation) {
  DdgiSettings settings;
  settings.volume_defaults.probe_counts = {8, 8, 8};
  settings.storage.max_probe_count = 32;
  settings.storage.atlas_probe_columns = 6;
  settings.storage.irradiance_tile_resolution = 8;
  settings.storage.visibility_tile_resolution = 10;
  settings.runtime.ray_count = 64;

  const auto layout = DdgiRuntime::CalculateFrameResourceLayout(settings);

  EXPECT_EQ(DdgiRuntime::GetProbeCount(settings.volume_defaults.probe_counts), 512u);
  EXPECT_EQ(DdgiRuntime::GetAllocatedProbeCount(settings), 0u);
  EXPECT_FALSE(layout.valid);
  EXPECT_EQ(layout.probe_count, 0u);
  EXPECT_EQ(layout.probe_metadata_byte_size, 0u);
  EXPECT_EQ(layout.ray_output_byte_size, 0u);
  EXPECT_FALSE(layout.error.empty());
}

TEST(DdgiVolume, RenderLayerFrameResourceLayoutUsesActiveVolumeProbeCount) {
  DdgiSettings settings;
  settings.volume_defaults.probe_counts = {4, 4, 4};
  settings.storage.max_probe_count = 4096;
  settings.storage.atlas_probe_columns = 16;
  settings.storage.irradiance_tile_resolution = 8;
  settings.storage.visibility_tile_resolution = 10;
  settings.runtime.ray_count = 128;
  settings.runtime.emissive_ray_count = 0;

  const auto layout = DdgiRuntime::CalculateFrameResourceLayout(settings, 252);

  EXPECT_EQ(DdgiRuntime::GetAllocatedProbeCount(settings), 64u);
  EXPECT_EQ(DdgiRuntime::GetAllocatedProbeCount(settings, 252), 252u);
  EXPECT_EQ(layout.probe_count, 252u);
  EXPECT_EQ(layout.irradiance_atlas.rows, 16u);
  EXPECT_EQ(layout.irradiance_atlas.resolution, glm::uvec2(160, 160));
  EXPECT_EQ(layout.visibility_atlas.resolution, glm::uvec2(192, 192));
  EXPECT_EQ(layout.probe_metadata_byte_size, 252ull * sizeof(glm::vec4) * 3ull);
  EXPECT_EQ(layout.probe_state_byte_size, 252ull * sizeof(glm::vec4));
  EXPECT_EQ(layout.ray_output_byte_size, 252ull * 128ull * sizeof(DdgiProbeRayData));
  EXPECT_EQ(layout.selected_ray_diagnostics_byte_size, 128ull * sizeof(PointCloudSample));
}

TEST(DdgiVolume, CompactRayMemoryAccountingCoversRepresentativeAndMaximumLayouts) {
  DdgiSettings settings;
  settings.storage.max_probe_count = 8192;
  settings.storage.atlas_probe_columns = 128;
  settings.storage.irradiance_tile_resolution = 8;
  settings.storage.visibility_tile_resolution = 8;
  settings.runtime.ray_count = 256;
  settings.runtime.emissive_ray_count = 0;

  const auto representative = DdgiRuntime::CalculateFrameResourceLayout(settings, 8192);
  ASSERT_TRUE(representative.valid) << representative.error;
  EXPECT_EQ(representative.ray_output_byte_size, 33'554'432u);
  EXPECT_EQ(representative.selected_ray_diagnostics_byte_size, 32'768u);
  EXPECT_EQ(representative.per_frame_transient_byte_size,
            representative.ray_output_byte_size + representative.selected_ray_diagnostics_byte_size);
  EXPECT_EQ(representative.peak_resident_byte_size,
            representative.persistent_byte_size +
                Platform::kMaxFramesInFlight * representative.per_frame_transient_byte_size);

  settings.runtime.ray_count = 4096;
  const auto maximum = DdgiRuntime::CalculateFrameResourceLayout(settings, 8192);
  ASSERT_TRUE(maximum.valid) << maximum.error;
  EXPECT_EQ(maximum.ray_output_byte_size, 536'870'912u);
  EXPECT_EQ(maximum.selected_ray_diagnostics_byte_size, 524'288u);
}

TEST(DdgiVolume, CompactRayShadersMatchHostLayoutAndPreserveSelectedDiagnostics) {
  const auto root =
      std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "Internals" / "DefaultResources" / "Shaders";
  const auto compact = ReadTextFile(root / "Modules" / "EvoEngine" / "DDGIProbeRayData.slang");
  const auto payload = ReadTextFile(root / "Modules" / "EvoEngine" / "DDGIProbeRayPayload.slang");
  const auto raygen = ReadTextFile(root / "RayTracing" / "RayGen" / "DDGIProbeTrace.slang");
  const auto update = ReadTextFile(root / "Compute" / "DDGIProbeUpdate.slang");
  const auto relocation = ReadTextFile(root / "Compute" / "DDGIProbeRelocation.slang");
  const auto classification = ReadTextFile(root / "Compute" / "DDGIProbeClassification.slang");
  const auto source_root = std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "src";
  const auto ddgi_runtime = ReadTextFile(source_root / "DdgiRuntime.cpp");
  const auto render_layer = ReadTextFile(source_root / "RenderLayer.cpp");
  const auto ray_pass = ReadTextFile(source_root / "RenderPasses" / "DdgiProbeTracePass.cpp");
  const auto visualization_pass = ReadTextFile(source_root / "RenderPasses" / "DdgiProbeRayVisualizationPass.cpp");

  EXPECT_NE(compact.find("struct DdgiProbeRayData"), std::string::npos);
  EXPECT_NE(payload.find("struct DdgiProbeRayPayload"), std::string::npos);
  EXPECT_NE(payload.find("EE_DDGI_MIN_RAY_TRANSMISSION = 0.01f"), std::string::npos);
  EXPECT_NE(compact.find("float4 radiance_and_signed_distance;"), std::string::npos);
  EXPECT_NE(compact.find("struct DdgiProbeRaySampleInfo"), std::string::npos);
  EXPECT_NE(compact.find("uint2 packed_direction_and_inverse_pdf;"), std::string::npos);
  EXPECT_NE(compact.find("packSnorm2x16"), std::string::npos);
  EXPECT_NE(compact.find("unpackSnorm2x16ToFloat"), std::string::npos);
  EXPECT_NE(compact.find("asuint(inverse_mixture_pdf)"), std::string::npos);
  EXPECT_NE(compact.find("asfloat(sample_info.packed_direction_and_inverse_pdf.y)"), std::string::npos);
  EXPECT_EQ(compact.find("struct DdgiEmissiveGuideInfo"), std::string::npos);
  EXPECT_NE(compact.find("EE_DDGI_PROBE_RAY_MISS_DISTANCE = 1e27f"), std::string::npos);
  EXPECT_NE(compact.find("EE_DDGI_PROBE_RAY_INACTIVE_DISTANCE = -1e27f"), std::string::npos);
  EXPECT_NE(compact.find("asuint(signed_distance) & 0x80000000u"), std::string::npos);

  EXPECT_NE(raygen.find("[[vk::binding(0, 2)]] RWStructuredBuffer<DdgiProbeRayData> EE_DDGI_PROBE_RAY_DATA"),
            std::string::npos);
  EXPECT_NE(
      raygen.find("[[vk::binding(2, 2)]] RWStructuredBuffer<DdgiProbeRaySample> EE_DDGI_SELECTED_RAY_DIAGNOSTICS"),
      std::string::npos);
  EXPECT_NE(raygen.find("local_probe_index == EE_DDGI_PROBE_RAY_CONSTANTS.selected_probe_volume_flags_environment.x"),
            std::string::npos);
  EXPECT_NE(raygen.find("EE_DDGI_PROBE_RAY_DATA[sample_index] = ray_data;"), std::string::npos);
  EXPECT_NE(raygen.find("EE_DDGI_SELECTED_RAY_DIAGNOSTICS[ray_index] = diagnostic_sample;"), std::string::npos);

  for (const auto* source : {&update, &relocation, &classification}) {
    EXPECT_NE(source->find("import EvoEngine.DDGIProbeRayData;"), std::string::npos);
  }
  EXPECT_NE(update.find("uint fixed_ray_count = min(EE_DDGI_UPDATE_FIXED_RAY_COUNT(), uniform_ray_count);"),
            std::string::npos);
  EXPECT_NE(update.find("asfloat(constants.probe_counts_and_rotation.w)"), std::string::npos);
  EXPECT_NE(update.find("EE_DDGI_ROTATED_PROBE_RAY_DIRECTION"), std::string::npos);
  EXPECT_NE(relocation.find("uint fixed_ray_count = min(constants.probe_counts.w, uniform_ray_count);"),
            std::string::npos);
  EXPECT_NE(relocation.find("if (EE_DDGI_PROBE_RAY_BACKFACE_HIT(ray_data))"), std::string::npos);
  EXPECT_NE(classification.find("uint fixed_ray_count = min(constants.probe_counts.w, uniform_ray_count);"),
            std::string::npos);
  EXPECT_EQ(raygen.find("[[vk::binding(20, 2)]]"), std::string::npos);
  EXPECT_NE(raygen.find("[[vk::binding(21, 2)]] RWStructuredBuffer<DdgiProbeRaySampleInfo>"), std::string::npos);
  EXPECT_NE(raygen.find("import EvoEngine.EmissiveTriangleSampling;"), std::string::npos);
  EXPECT_NE(raygen.find("EE_SAMPLE_EMISSIVE_TRIANGLE("), std::string::npos);
  EXPECT_NE(raygen.find("selected_emitter_visible"), std::string::npos);
  EXPECT_NE(update.find("[[vk::binding(6, 1)]]"), std::string::npos);
  EXPECT_NE(update.find("EE_DDGI_PROBE_RAY_DIRECTION(sample_info)"), std::string::npos);
  EXPECT_NE(update.find("EE_DDGI_PROBE_RAY_INVERSE_PDF(sample_info)"), std::string::npos);
  EXPECT_NE(update.find("2.0f * EE_DDGI_PI * float(emissive_ray_count)"), std::string::npos);
  const auto visibility_call =
      ExtractBetween(update, "float2 directional_visibility =", "EE_DDGI_STORE_VISIBILITY_TEXEL");
  EXPECT_NE(visibility_call.find("EE_DDGI_DIRECTIONAL_VISIBILITY_MOMENTS"), std::string::npos);
  EXPECT_NE(visibility_call.find("uniform_ray_count"), std::string::npos);
  EXPECT_NE(update.find("EE_DDGI_WRITE_PROBE_METADATA(ray_offset, physical_probe_index, uniform_ray_count"),
            std::string::npos);
  EXPECT_NE(update.find("EE_DDGI_UPDATE_VISIBILITY_SHARED(ray_offset, first_blend_ray_index, uniform_ray_count"),
            std::string::npos);

  EXPECT_NE(ddgi_runtime.find("sizeof(DdgiProbeRayData)"), std::string::npos);
  EXPECT_NE(render_layer.find("runtime_state.selected_ray_diagnostics_buffers"), std::string::npos);
  EXPECT_NE(render_layer.find("ddgi_emissive_population_changed ||"), std::string::npos);
  EXPECT_NE(render_layer.find("!runtime_state.has_valid_probe_history && runtime_state.history_submissions.empty()"),
            std::string::npos);
  EXPECT_NE(render_layer.find("runtime_state.probe_ray_sequence_index + pending_updates"), std::string::npos);
  EXPECT_NE(render_layer.find("runtime_state.history_submission"), std::string::npos);
  EXPECT_EQ(render_layer.find("clear_probe_atlas_this_frame || ddgi_emissive_population_changed"), std::string::npos);
  EXPECT_NE(render_layer.find("CreateDdgiImportedBufferResourceDescriptor("
                              "RenderResourceNames::frame_ddgi_selected_ray_diagnostics"),
            std::string::npos);
  EXPECT_NE(render_layer.find("glm::floatBitsToUint(runtime_state.frame_ray_push_constant.probe_step_z.w)"),
            std::string::npos);
  EXPECT_NE(ray_pass.find("UpdateBufferDescriptorBinding(2, diagnostics_binding->buffer)"), std::string::npos);
  EXPECT_NE(ray_pass.find("diagnostics_binding->buffer->GetVkBuffer()"), std::string::npos);
  EXPECT_NE(visualization_pass.find("parameters.selected_ray_diagnostics_buffer"), std::string::npos);
}

TEST(DdgiVolume, RenderLayerAcceptsExactProbeCapAndRejectsCapPlusOne) {
  DdgiSettings settings;
  settings.storage.max_probe_count = 32;
  settings.storage.atlas_probe_columns = 4;
  settings.storage.irradiance_tile_resolution = 8;
  settings.storage.visibility_tile_resolution = 8;
  settings.runtime.ray_count = 16;
  settings.runtime.emissive_ray_count = 0;

  EXPECT_TRUE(DdgiRuntime::ValidateProbeGrid({4, 4, 2}, 32));
  EXPECT_FALSE(DdgiRuntime::ValidateProbeGrid({4, 4, 3}, 32));
  const auto exact_layout = DdgiRuntime::CalculateFrameResourceLayout(settings, 32, 512);
  const auto overflow_layout = DdgiRuntime::CalculateFrameResourceLayout(settings, 33, 512);

  EXPECT_TRUE(exact_layout.valid) << exact_layout.error;
  EXPECT_EQ(exact_layout.probe_count, 32u);
  EXPECT_EQ(exact_layout.probe_state_byte_size, 32ull * sizeof(glm::vec4));
  EXPECT_EQ(exact_layout.ray_output_byte_size, 32ull * 16ull * sizeof(DdgiProbeRayData));
  EXPECT_EQ(exact_layout.selected_ray_diagnostics_byte_size, 16ull * sizeof(PointCloudSample));
  EXPECT_FALSE(overflow_layout.valid);
  EXPECT_EQ(overflow_layout.probe_count, 0u);
}

TEST(DdgiVolume, RenderLayerRejectsAuthoredAtlasExtentWithoutReshapingColumns) {
  constexpr uint32_t probe_count = 32;
  const auto exact_layout = DdgiRuntime::CalculateAtlasLayout(probe_count, 8, 4, 80);
  const auto rejected_layout = DdgiRuntime::CalculateAtlasLayout(probe_count, 8, 4, 79);

  ASSERT_TRUE(exact_layout.valid) << exact_layout.error;
  EXPECT_EQ(exact_layout.columns, 4u);
  EXPECT_EQ(exact_layout.rows, 8u);
  EXPECT_EQ(exact_layout.resolution, glm::uvec2(40, 80));
  EXPECT_FALSE(rejected_layout.valid);
  EXPECT_EQ(rejected_layout.columns, 1u);
  EXPECT_FALSE(rejected_layout.error.empty());
}

TEST(DdgiVolume, RenderLayerRejectsZeroAtlasColumnsWithoutReshaping) {
  const auto layout = DdgiRuntime::CalculateAtlasLayout(16, 8, 0, 1024);
  EXPECT_FALSE(layout.valid);
  EXPECT_FALSE(layout.error.empty());

  DdgiSettings settings;
  settings.volume_defaults.probe_counts = {4, 2, 2};
  settings.storage.atlas_probe_columns = 0;
  EXPECT_FALSE(DdgiRuntime::CalculateFrameResourceLayout(settings).valid);
}

TEST(DdgiVolume, RenderLayerPropagatesVisibilityAtlasDeviceLimitFailure) {
  DdgiSettings settings;
  settings.storage.max_probe_count = 32;
  settings.storage.atlas_probe_columns = 4;
  settings.storage.irradiance_tile_resolution = 8;
  settings.storage.visibility_tile_resolution = 16;

  const auto layout = DdgiRuntime::CalculateFrameResourceLayout(settings, 32, 100);

  EXPECT_FALSE(layout.valid);
  EXPECT_TRUE(layout.irradiance_atlas.valid);
  EXPECT_FALSE(layout.visibility_atlas.valid);
  EXPECT_FALSE(layout.error.empty());
}

TEST(DdgiVolume, RenderLayerRejectsStorageBuffersBeyondDeviceRange) {
  DdgiSettings settings;
  settings.storage.max_probe_count = 32;
  settings.storage.atlas_probe_columns = 4;
  settings.runtime.ray_count = 16;
  const auto unrestricted = DdgiRuntime::CalculateFrameResourceLayout(settings, 32, 1024);
  ASSERT_TRUE(unrestricted.valid) << unrestricted.error;

  const auto maximum_buffer =
      std::max(unrestricted.ray_output_byte_size,
               *std::max_element(unrestricted.history.buffer_bytes.begin(), unrestricted.history.buffer_bytes.end()));
  const auto exact = DdgiRuntime::CalculateFrameResourceLayout(settings, 32, 1024, maximum_buffer);
  const auto overflow = DdgiRuntime::CalculateFrameResourceLayout(settings, 32, 1024, maximum_buffer - 1u);
  EXPECT_TRUE(exact.valid) << exact.error;
  EXPECT_FALSE(overflow.valid);
  EXPECT_FALSE(overflow.error.empty());
}

TEST(DdgiVolume, InspectorUsesAssetEditBoundaryWithoutRuntimeQueries) {
  const auto inspector_source = ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" /
                                             "src" / "Editor" / "SDKInspectionAdapters.cpp");
  ASSERT_FALSE(inspector_source.empty());

  EXPECT_EQ(inspector_source.find("GetDdgiMaxStorageBufferRange"), std::string::npos);
  EXPECT_EQ(inspector_source.find("GetDdgiMaxImageDimension2D"), std::string::npos);
  EXPECT_NE(inspector_source.find("if (changed) {"), std::string::npos);
  EXPECT_NE(inspector_source.find("ClampDdgiSettings(lighting.ddgi_settings);"), std::string::npos);
  EXPECT_EQ(inspector_source.find("previous_ray_count"), std::string::npos);
}

TEST(DdgiVolume, PausedHistoryRequiresCompatiblePersistentLayout) {
  DdgiSettings settings;
  settings.storage.max_probe_count = 64;
  settings.storage.atlas_probe_columns = 4;
  settings.runtime.ray_count = 16;
  const auto previous = DdgiRuntime::CalculateFrameResourceLayout(settings, 32, 1024);
  ASSERT_TRUE(previous.valid) << previous.error;

  settings.runtime.ray_count = 32;
  const auto transient_only_change = DdgiRuntime::CalculateFrameResourceLayout(settings, 32, 1024);
  EXPECT_TRUE(DdgiRuntime::ArePersistentLayoutsCompatible(previous, transient_only_change));

  settings.storage.atlas_probe_columns = 8;
  const auto atlas_change = DdgiRuntime::CalculateFrameResourceLayout(settings, 32, 1024);
  EXPECT_FALSE(DdgiRuntime::ArePersistentLayoutsCompatible(previous, atlas_change));
  EXPECT_FALSE(DdgiRuntime::ArePersistentLayoutsCompatible({}, previous));
}

TEST(DdgiVolume, ResolvedVolumeCollectionFeedsAtomicSetValidationWithoutMutatingAuthoring) {
  const auto resolver_source = ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "src" /
                                            "EnvironmentalLightingResolver.cpp");
  const auto render_layer_source =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "src" / "RenderLayer.cpp");
  const auto collect = resolver_source.find("evo_engine::CollectDdgiVolumeRuntimeInfos");
  const auto reserve = resolver_source.find("infos.reserve(lighting.ddgi_volumes.size())", collect);
  const auto iterate = resolver_source.find("for (size_t i = 0; i < lighting.ddgi_volumes.size(); ++i)", reserve);
  const auto accept = resolver_source.find("auto& info = infos.emplace_back();", iterate);
  ASSERT_NE(collect, std::string::npos);
  ASSERT_NE(reserve, std::string::npos);
  ASSERT_NE(iterate, std::string::npos);
  ASSERT_NE(accept, std::string::npos);
  EXPECT_LT(reserve, iterate);
  EXPECT_LT(iterate, accept);
  const auto collect_scope = resolver_source.substr(collect);
  EXPECT_EQ(collect_scope.find("UnsafeGetPrivateComponentOwnersList<DdgiVolume>"), std::string::npos);
  EXPECT_EQ(collect_scope.find("SetEnabled("), std::string::npos);
  EXPECT_EQ(collect_scope.find("rejected"), std::string::npos);

  const auto prepare = render_layer_source.find("void RenderLayer::PrepareDdgiFrameState");
  const auto prepare_worker = render_layer_source.find("void RenderLayer::PrepareDdgiVolumeFrameState", prepare);
  const auto prepare_resolve =
      render_layer_source.find("const auto resolved_lighting = ResolveEnvironmentalLighting(scene)", prepare);
  const auto prepare_iterate =
      render_layer_source.find("for (const auto& volume : resolved_lighting.ddgi_volumes)", prepare_resolve);
  const auto validate =
      render_layer_source.find("const auto validation = DdgiRuntime::ValidateVolumeSet", prepare_iterate);
  const auto reject_invalid = render_layer_source.find("reject_volume_set(validation.error);", validate);
  ASSERT_NE(prepare, std::string::npos);
  ASSERT_NE(prepare_worker, std::string::npos);
  ASSERT_NE(prepare_resolve, std::string::npos);
  ASSERT_NE(prepare_iterate, std::string::npos);
  ASSERT_NE(validate, std::string::npos);
  ASSERT_NE(reject_invalid, std::string::npos);
  EXPECT_LT(prepare_resolve, prepare_iterate);
  EXPECT_LT(prepare_iterate, validate);
  EXPECT_LT(validate, reject_invalid);
  EXPECT_LT(reject_invalid, prepare_worker);
  const auto prepare_scope = render_layer_source.substr(prepare, prepare_worker - prepare);
  EXPECT_EQ(prepare_scope.find("SetEnabled("), std::string::npos);
  EXPECT_EQ(prepare_scope.find("rejected_candidates"), std::string::npos);
  EXPECT_EQ(prepare_scope.find("rejected."), std::string::npos);
}

TEST(DdgiVolume, EmissiveMeshSamplingOverridesResolveAndPackIndependently) {
  const auto inherit = static_cast<int>(DdgiEmissiveMeshSamplingMode::Inherit);
  const auto on = static_cast<int>(DdgiEmissiveMeshSamplingMode::On);
  const auto off = static_cast<int>(DdgiEmissiveMeshSamplingMode::Off);
  EXPECT_FALSE(DdgiRuntime::ResolveEmissiveMeshSampling(false, inherit));
  EXPECT_TRUE(DdgiRuntime::ResolveEmissiveMeshSampling(true, inherit));
  EXPECT_TRUE(DdgiRuntime::ResolveEmissiveMeshSampling(false, on));
  EXPECT_TRUE(DdgiRuntime::ResolveEmissiveMeshSampling(true, on));
  EXPECT_FALSE(DdgiRuntime::ResolveEmissiveMeshSampling(false, off));
  EXPECT_FALSE(DdgiRuntime::ResolveEmissiveMeshSampling(true, off));

  EXPECT_EQ(DdgiRuntime::GetProbeRayFlags(false, false), 0u);
  EXPECT_EQ(DdgiRuntime::GetProbeRayFlags(true, false), DdgiRuntime::kProbeRayFlagSkipInactive);
  EXPECT_EQ(DdgiRuntime::GetProbeRayFlags(false, true), DdgiRuntime::kProbeRayFlagEmissiveMeshSampling);
  EXPECT_EQ(DdgiRuntime::GetProbeRayFlags(true, true),
            DdgiRuntime::kProbeRayFlagSkipInactive | DdgiRuntime::kProbeRayFlagEmissiveMeshSampling);
  EXPECT_EQ(DdgiRuntime::kProbeRayFlagSkipInactive, 1u << 0u);
  EXPECT_EQ(DdgiRuntime::kProbeRayFlagEmissiveMeshSampling, 1u << 1u);
}

TEST(DdgiVolume, EmissiveSamplingCandidateCountIsAnUpperBoundForNonFixedRays) {
  EXPECT_EQ(DdgiRuntime::CalculateEmissiveSamplingCandidateRayCount(3u, 64u, 6u, true, true), 174u);
  EXPECT_EQ(DdgiRuntime::CalculateEmissiveSamplingCandidateRayCount(3u, 64u, 80u, true, true), 0u);
  EXPECT_EQ(DdgiRuntime::CalculateEmissiveSamplingCandidateRayCount(3u, 64u, 6u, false, true), 0u);
  EXPECT_EQ(DdgiRuntime::CalculateEmissiveSamplingCandidateRayCount(3u, 64u, 6u, true, false), 0u);
  EXPECT_EQ(DdgiRuntime::CalculateEmissiveSamplingCandidateRayCount(0xffffffffu, 2u, 0u, true, true),
            static_cast<uint64_t>(0xffffffffu) * 2u);
}

TEST(DdgiVolume, EmissiveMeshAuthoringReportsPowerEligibilityAndRequiresExplicitAdjustment) {
  const auto inspector_source = ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" /
                                             "src" / "Editor" / "SDKInspectionAdapters.cpp");
  ASSERT_FALSE(inspector_source.empty());
  const auto authoring = ExtractBetween(inspector_source, "struct EmissiveMeshAuthoringEstimate",
                                        "bool InspectMeshRenderer(InspectorContext& context");
  ASSERT_FALSE(authoring.empty());
  EXPECT_NE(authoring.find("glm::pi<double>() * estimate.world_area * luminance"), std::string::npos);
  EXPECT_NE(authoring.find("shade_material.double_sided != 0 ? 2.0 : 1.0"), std::string::npos);
  EXPECT_NE(authoring.find("Estimated world area"), std::string::npos);
  EXPECT_NE(authoring.find("Estimated emitted power"), std::string::npos);
  EXPECT_NE(authoring.find("DDGI triangle candidates"), std::string::npos);
  EXPECT_NE(authoring.find("DDGI sampling: %s"), std::string::npos);
  EXPECT_NE(authoring.find("Factor-only preview"), std::string::npos);
  EXPECT_NE(authoring.find("Apply target power to radiance"), std::string::npos);
  EXPECT_NE(
      authoring.find("shade_material.emissive_factor *= static_cast<float>(target_power / estimate.emitted_power)"),
      std::string::npos);
  EXPECT_NE(authoring.find("it never creates or changes an analytic light"), std::string::npos);
  EXPECT_EQ(authoring.find("PointLight"), std::string::npos);
  EXPECT_EQ(authoring.find("SpotLight"), std::string::npos);

  const auto mesh_inspector = ExtractBetween(inspector_source, "bool InspectMeshRenderer(InspectorContext& context",
                                             "bool InspectParticles(InspectorContext& context");
  const auto skinned_inspector =
      ExtractBetween(inspector_source, "bool InspectSkinnedMeshRenderer(InspectorContext& context",
                     "bool InspectLodGroup(InspectorContext&");
  EXPECT_NE(mesh_inspector.find("InspectEmissiveMeshAuthoring"), std::string::npos);
  EXPECT_NE(skinned_inspector.find("InspectEmissiveMeshAuthoring"), std::string::npos);
  EXPECT_NE(skinned_inspector.find("true, &renderer"), std::string::npos);
}

TEST(DdgiVolume, RenderLayerRequiresFullResetWhenScrollInvalidatesAnAxis) {
  const glm::ivec3 probe_counts(8, 6, 4);
  EXPECT_FALSE(DdgiRuntime::RequiresFullScrollReset(probe_counts, {1, 0, 0}));
  EXPECT_FALSE(DdgiRuntime::RequiresFullScrollReset(probe_counts, {2, -1, 3}));
  EXPECT_TRUE(DdgiRuntime::RequiresFullScrollReset(probe_counts, {8, 0, 0}));
  EXPECT_TRUE(DdgiRuntime::RequiresFullScrollReset(probe_counts, {0, -7, 0}));
}

TEST(DdgiVolume, ProbeVisualizationFadesNearCameraWithAlphaBlend) {
  const auto shader_root = std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "Internals" /
                           "DefaultResources" / "Shaders" / "Graphics";
  const auto vertex_source = ReadTextFile(shader_root / "Vertex" / "DDGI" / "DDGIProbeVisualization.slang");
  const auto fragment_source = ReadTextFile(shader_root / "Fragment" / "DDGI" / "DDGIProbeVisualization.slang");
  ASSERT_FALSE(vertex_source.empty());
  ASSERT_FALSE(fragment_source.empty());

  EXPECT_NE(fragment_source.find("float probe_inactive = clamp(input.state.w, 0.0f, 1.0f);"), std::string::npos);
  EXPECT_NE(fragment_source.find("float3 EeDdgiDecodeDebugIrradiance"), std::string::npos);
  EXPECT_NE(fragment_source.find("irradiance_gamma * 0.5f"), std::string::npos);
  EXPECT_NE(fragment_source.find("EeDdgiTonemapDebugColor"), std::string::npos);
  EXPECT_NE(fragment_source.find("EE_DDGI_IRRADIANCE_ATLAS.Sample(irradiance_uv).rgb"), std::string::npos);
  EXPECT_NE(fragment_source.find("if (probe_inactive > 0.5f)"), std::string::npos);
  EXPECT_NE(vertex_source.find("float EeDdgiProbeCameraFadeAlpha"), std::string::npos);
  EXPECT_NE(vertex_source.find("distance(probe_position, EE_CAMERA_POSITION(int(camera_index)))"), std::string::npos);
  EXPECT_NE(vertex_source.find("return smoothstep(fade_start, fade_end, distance(probe_position"), std::string::npos);
  EXPECT_NE(vertex_source.find("output.camera_fade = EeDdgiProbeCameraFadeAlpha"), std::string::npos);
  EXPECT_NE(fragment_source.find("clamp(input.camera_fade, 0.0f, 1.0f)"), std::string::npos);
  EXPECT_NE(fragment_source.find("return float4(color, alpha);"), std::string::npos);
  EXPECT_EQ(fragment_source.find("return float4(color, 1.0f);"), std::string::npos);
  EXPECT_EQ(fragment_source.find("const float probe_active = input.state.w"), std::string::npos);
  EXPECT_EQ(fragment_source.find("hit_alpha"), std::string::npos);

  const auto pass_path = std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "src" / "RenderPasses" /
                         "DdgiProbeVisualizationPass.cpp";
  const auto pass_source = ReadTextFile(pass_path);
  ASSERT_FALSE(pass_source.empty());

  EXPECT_NE(pass_source.find("parameters.pipeline->states.color_blend_attachment_states[0].blendEnable = VK_TRUE;"),
            std::string::npos);
  EXPECT_NE(pass_source.find("parameters.pipeline->states.depth_write = false;"), std::string::npos);
}

TEST(DdgiVolume, DdgiDiffuseUsesRtxgiStyleEnergyEncoding) {
  const auto shader_root =
      std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "Internals" / "DefaultResources" / "Shaders";
  const auto lighting_source = ReadTextFile(shader_root / "Modules" / "EvoEngine" / "Lighting.slang");
  const auto probe_update_source = ReadTextFile(shader_root / "Compute" / "DDGIProbeUpdate.slang");
  const auto raygen_source = ReadTextFile(shader_root / "RayTracing" / "RayGen" / "DDGIProbeTrace.slang");
  const auto closest_hit_source = ReadTextFile(shader_root / "RayTracing" / "ClosestHit" / "DDGIProbeTrace.slang");
  const auto ddgi_helper_source = ReadTextFile(shader_root / "Modules" / "EvoEngine" / "DDGI.slang");
  const auto gather_single_source = ReadTextFile(shader_root / "Modules" / "EvoEngine" / "DDGIGatherSingle.slang");
  const auto gather_source = ReadTextFile(shader_root / "Modules" / "EvoEngine" / "DDGIGather.slang") +
                             ReadTextFile(shader_root / "Modules" / "EvoEngine" / "DDGIGatherMulti.slang");
  const auto scroll_source = ReadTextFile(shader_root / "Compute" / "DDGIProbeScroll.slang");
  const auto atlas_prepare_source = ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" /
                                                 "src" / "RenderPasses" / "DdgiAtlasPreparePass.cpp");
  const auto render_layer_source =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "src" / "RenderLayer.cpp");
  ASSERT_FALSE(lighting_source.empty());
  ASSERT_FALSE(probe_update_source.empty());
  ASSERT_FALSE(raygen_source.empty());
  ASSERT_FALSE(closest_hit_source.empty());
  ASSERT_FALSE(ddgi_helper_source.empty());
  ASSERT_FALSE(gather_single_source.empty());
  ASSERT_FALSE(gather_source.empty());
  ASSERT_FALSE(scroll_source.empty());
  ASSERT_FALSE(atlas_prepare_source.empty());
  ASSERT_FALSE(render_layer_source.empty());

  EXPECT_NE(probe_update_source.find("1.0f / (2.0f * max(uniform_accumulator.weight_sum, uniform_epsilon))"),
            std::string::npos);
  EXPECT_NE(probe_update_source.find("EE_DDGI_PROBE_MAX_VISIBILITY_DISTANCE()"), std::string::npos);
  EXPECT_NE(probe_update_source.find("float2(accumulator.first_moment, accumulator.second_moment) * "
                                     "(1.0f / (2.0f * accumulator.weight_sum))"),
            std::string::npos);
  EXPECT_NE(probe_update_source.find("float4(clamp(irradiance, float3(0.0f), float3(64.0f)), 1.0f)"),
            std::string::npos);
  EXPECT_EQ(probe_update_source.find("classification_enabled && classification_inside_geometry ? 1.0f : 0.0f"),
            std::string::npos);
  EXPECT_NE(
      probe_update_source.find("uint fixed_ray_count = min(EE_DDGI_UPDATE_FIXED_RAY_COUNT(), uniform_ray_count);"),
      std::string::npos);
  EXPECT_EQ(probe_update_source.find("!classification_has_nearby_geometry"), std::string::npos);
  EXPECT_EQ(raygen_source.find("uint EE_DDGI_RANDOM_ROTATION_SEED"), std::string::npos);
  EXPECT_EQ(raygen_source.find("EE_RANGED_RANDOM"), std::string::npos);
  EXPECT_NE(raygen_source.find("float4 EE_DDGI_PROBE_RAY_ROTATION()"), std::string::npos);
  EXPECT_NE(ddgi_helper_source.find("EE_DDGI_ROTATE_BY_CONJUGATE_QUATERNION"), std::string::npos);
  EXPECT_NE(raygen_source.find("RayDesc ray = {origin, 0.0f, direction"), std::string::npos);
  EXPECT_NE(raygen_source.find("hit_value.seed = fixed_ray ? EE_DDGI_FIXED_RAY_PAYLOAD_FLAG"), std::string::npos);
  EXPECT_NE(raygen_source.find("if (inactive_probe && !fixed_ray)"), std::string::npos);
  EXPECT_NE(ddgi_helper_source.find("float golden_ratio_fraction"), std::string::npos);
  EXPECT_NE(ddgi_helper_source.find("frac(float(sample_index) * golden_ratio_fraction)"), std::string::npos);
  EXPECT_NE(ddgi_helper_source.find("public static const uint EE_DDGI_FIXED_RAY_PAYLOAD_FLAG = 1u;"),
            std::string::npos);
  EXPECT_NE(ddgi_helper_source.find("EE_DDGI_SIGN_NOT_ZERO(direction.xy)"), std::string::npos);
  EXPECT_NE(gather_source.find("[[vk::binding(19, 2)]]"), std::string::npos);
  EXPECT_NE(lighting_source.find("__exported import EvoEngine.DDGIGatherMulti;"), std::string::npos);
  EXPECT_NE(lighting_source.find(
                "EE_FUNC_CALCULATE_DDGI_DIFFUSE(float3 albedo, float3 normal, float3 viewDir, float3 fragPos)"),
            std::string::npos);
  EXPECT_NE(lighting_source.find("EE_FUNC_CALCULATE_DDGI_ENVIRONMENTAL_LIGHT"), std::string::npos);
  EXPECT_NE(lighting_source.find("diffuse = lerp(diffuse, ddgi_diffuse, gather_weight)"), std::string::npos);
  EXPECT_NE(lighting_source.find("EE_DDGI_GATHER_WEIGHT(gather)"), std::string::npos);
  EXPECT_NE(lighting_source.find("EE_DDGI_GATHER_VISIBILITY(gather)"), std::string::npos);
  EXPECT_NE(lighting_source.find("const float ddgiSpecularVisibility = "
                                 "lerp(1.0f, EE_DDGI_GATHER_VISIBILITY(gather), gather_weight)"),
            std::string::npos);
  EXPECT_NE(lighting_source.find("reflectionProbeCapture ? 0.0f : clamp(ddgiWeight, 0.0f, 1.0f)"), std::string::npos);
  EXPECT_NE(lighting_source.find("max(ddgiIrradiance, float3(0.0f)) / EE_DDGI_PI"), std::string::npos);
  EXPECT_NE(lighting_source.find("reflectionFallback = lerp(globalPrefiltered, ddgiSpecularFallback"),
            std::string::npos);
  EXPECT_EQ(lighting_source.find("smoothstep(0.4f, 0.8f, roughness)"), std::string::npos);
  EXPECT_EQ(gather_source.find("specular_irradiance"), std::string::npos);
  EXPECT_NE(lighting_source.find("ddgiSpecularVisibility);"), std::string::npos);
  EXPECT_NE(lighting_source.find("EE_REFLECTION_PROBE_SCALAR_VISIBILITY(materialOcclusion, screenSpaceVisibility, "
                                 "ddgiVisibility)"),
            std::string::npos);
  EXPECT_NE(lighting_source.find("environment.specular"), std::string::npos);
  EXPECT_NE(lighting_source.find("const float3 diffuse_albedo = environment.diffuse_weight * albedo"),
            std::string::npos);
  EXPECT_EQ(lighting_source.find("const float3 ddgi_lighting = EE_DDGI_DIFFUSE_RADIANCE(gather, float3(1.0f))"),
            std::string::npos);
  EXPECT_EQ(lighting_source.find("diffuse_lighting = lerp(diffuse_lighting, ddgi_lighting, gather_weight)"),
            std::string::npos);
  EXPECT_EQ(lighting_source.find("EE_REFLECTION_LIGHTING_SCALE"), std::string::npos);
  EXPECT_NE(lighting_source.find(
                "const float indirectVisibility = clamp(materialOcclusion * screenSpaceVisibility, 0.0f, 1.0f)"),
            std::string::npos);
  EXPECT_NE(lighting_source.find("const float3 diffuseIndirect = diffuse * indirectVisibility *"), std::string::npos);
  EXPECT_NE(lighting_source.find("!any(isnan(ddgi_diffuse)) && !any(isinf(ddgi_diffuse))"), std::string::npos);
  EXPECT_EQ(lighting_source.find("EE_DDGI_PROBE_COORDINATE(fragPos"), std::string::npos);

  EXPECT_NE(gather_source.find("struct EeDdgiGatherResult"), std::string::npos);
  EXPECT_NE(gather_source.find("float visibility"), std::string::npos);
  EXPECT_NE(gather_source.find("result.visibility = 1.0f"), std::string::npos);
  EXPECT_NE(gather_source.find("irradiance_gamma * 0.5f"), std::string::npos);
  EXPECT_NE(gather_source.find("const float4 probe_state = resources.loadProbeState(volume_index, probe_index);"),
            std::string::npos);
  EXPECT_NE(gather_source.find("if (probe_state.w > 0.5f)"), std::string::npos);
  EXPECT_NE(gather_source.find("const float irradiance_validity = clamp(irradiance.a"), std::string::npos);
  EXPECT_EQ(gather_source.find("result.confidence = 1.0f;"), std::string::npos);
  EXPECT_NE(gather_source.find("valid_probe_weight_sum / active_probe_weight_sum"), std::string::npos);
  EXPECT_NE(atlas_prepare_source.find("MakeClearColor(0.0f, 0.0f, 0.0f, 0.0f)"), std::string::npos);
  EXPECT_NE(scroll_source.find("EE_DDGI_IRRADIANCE_ATLAS[int2"), std::string::npos);
  EXPECT_NE(scroll_source.find("float4(0.0f)"), std::string::npos);
  EXPECT_NE(gather_source.find("EE_DDGI_CHEBYSHEV_VISIBILITY(visibility_sample.rg, biased_probe_distance"),
            std::string::npos);
  EXPECT_NE(gather_source.find("visibility_sum += clamped_probe_visibility * valid_probe_weight"), std::string::npos);
  EXPECT_NE(gather_source.find("result.visibility = clamp(visibility_sum / valid_probe_weight_sum"), std::string::npos);
  EXPECT_NE(gather_source.find("float EE_DDGI_GATHER_VISIBILITY"), std::string::npos);
  EXPECT_NE(gather_source.find("EE_DDGI_VOLUME_BLEND_WEIGHT(volume_probe_coordinate"), std::string::npos);
  EXPECT_NE(gather_source.find("decoded_irradiance *= decoded_irradiance * (2.0f * EE_DDGI_PI);"), std::string::npos);
  EXPECT_NE(gather_source.find("float3 linear_irradiance"), std::string::npos);
  EXPECT_NE(gather_source.find("linear_irradiance_sum += pow(encoded_irradiance, float3(irradiance_gamma))"),
            std::string::npos);
  EXPECT_NE(gather_source.find("if (gather_linear_irradiance)"), std::string::npos);
  EXPECT_NE(gather_source.find("result.linear_irradiance = linear_irradiance;"), std::string::npos);
  EXPECT_NE(gather_source.find("primary.linear_irradiance * primary_effective_weight"), std::string::npos);
  EXPECT_NE(gather_source.find("EE_DDGI_PROBE_BLEND_LOSS"), std::string::npos);
  EXPECT_NE(lighting_source.find("if (debugView == 5)"), std::string::npos);
  EXPECT_NE(lighting_source.find("EE_DDGI_GATHER_IRRADIANCE(normal, viewDir, fragPos, debugView == 5)"),
            std::string::npos);
  EXPECT_NE(lighting_source.find("return EE_DDGI_PROBE_BLEND_LOSS(gather);"), std::string::npos);
  EXPECT_NE(gather_source.find("diffuse_albedo / EE_DDGI_PI * gather_result.irradiance"), std::string::npos);
  EXPECT_NE(gather_source.find("isnan(result.coverage) || isinf(result.coverage)"), std::string::npos);
  EXPECT_NE(gather_source.find("return isnan(weight) || isinf(weight) ? 0.0f"), std::string::npos);
  EXPECT_NE(ddgi_helper_source.find("float2 moments = max(2.0f * half_moments"), std::string::npos);
  EXPECT_NE(ddgi_helper_source.find("return max(visibility * visibility * visibility, 0.0f);"), std::string::npos);
  EXPECT_NE(ddgi_helper_source.find("view_direction * max(view_bias, 0.0f)"), std::string::npos);
  EXPECT_NE(ddgi_helper_source.find("float3 outside_distance = max(lower_distance, upper_distance);"),
            std::string::npos);
  EXPECT_NE(ddgi_helper_source.find("if (inside_volume)"), std::string::npos);
  EXPECT_NE(closest_hit_source.find("EE_DDGI_LAMBERT_IRRADIANCE(albedo, light.diffuse.rgb"), std::string::npos);
  EXPECT_NE(closest_hit_source.find("import EvoEngine.DDGIGatherSingle;"), std::string::npos);
  EXPECT_NE(closest_hit_source.find("EE_DDGI_GATHER_IRRADIANCE("), std::string::npos);
  EXPECT_NE(closest_hit_source.find("EE_DDGI_PROBE_RAY_CONSTANTS.selected_probe_volume_flags_environment.y"),
            std::string::npos);
  EXPECT_EQ(closest_hit_source.find("EE_DDGI_PROBE_COORDINATE(position"), std::string::npos);
  EXPECT_NE(closest_hit_source.find("import EvoEngine.RayTracingMaterial;"), std::string::npos);
  EXPECT_NE(closest_hit_source.find("EE_EVALUATE_GLTF_RASTER_SURFACE("), std::string::npos);
  EXPECT_NE(closest_hit_source.find("EE_EVALUATE_GLTF_RASTER_NORMAL("), std::string::npos);
  EXPECT_NE(closest_hit_source.find("const float3 emissive_radiance ="), std::string::npos);
  EXPECT_NE(closest_hit_source.find("EE_DDGI_EMISSIVE_RAY_COUNT() == 0u || material.unlit != 0"), std::string::npos);
  EXPECT_NE(closest_hit_source.find("emissive_radiance +"), std::string::npos);
  EXPECT_NE(closest_hit_source.find("EE_DDGI_DIRECT_IRRADIANCE(diffuse_albedo"), std::string::npos);
  EXPECT_NE(gather_single_source.find(
                "[[vk::binding(1, 2)]] public StructuredBuffer<float4, Std430DataLayout> EE_DDGI_PROBE_STATE"),
            std::string::npos);
  EXPECT_NE(closest_hit_source.find("material.double_sided"), std::string::npos);
  EXPECT_EQ(closest_hit_source.find(std::string("material") + "_properties.cull_mode"), std::string::npos);
  EXPECT_NE(closest_hit_source.find("if (!hit_face_is_culled && !fixed_probe_ray)"), std::string::npos);
  EXPECT_NE(closest_hit_source.find("signed_backface_hit ? -1.0f : 1.0f"), std::string::npos);
  EXPECT_EQ(closest_hit_source.find("backface_hit || fixed_probe_ray"), std::string::npos);
  EXPECT_NE(closest_hit_source.find("const bool fixed_probe_ray = hit_value.seed == EE_DDGI_FIXED_RAY_PAYLOAD_FLAG;"),
            std::string::npos);
  EXPECT_NE(closest_hit_source.find("float4(max(shaded_radiance, float3(0.0f))"), std::string::npos);
  EXPECT_EQ(closest_hit_source.find("clamp(shaded_radiance, float3(0.0f), float3(1.0f))"), std::string::npos);
  EXPECT_NE(closest_hit_source.find("surface_fresnel"), std::string::npos);
  EXPECT_NE(closest_hit_source.find("(float3(1.0f) - surface_fresnel)"), std::string::npos);
  EXPECT_NE(closest_hit_source.find("-ray_direction"), std::string::npos);
  EXPECT_NE(render_layer_source.find("CreateDdgiSamplingPhase"), std::string::npos);
  EXPECT_NE(render_layer_source.find("source.selected_volume_index"), std::string::npos);
  EXPECT_NE(render_layer_source.find("push_constant.first_probe = glm::vec4(source.first_probe, ray_rotation.x);"),
            std::string::npos);
  EXPECT_NE(render_layer_source.find("push_constant.probe_step_z = glm::vec4(source.probe_step_z, ray_rotation.w);"),
            std::string::npos);
  EXPECT_NE(render_layer_source.find("glm::max(ddgi_settings.runtime.view_bias, 0.0f)"), std::string::npos);
  EXPECT_NE(render_layer_source.find("glm::max(settings.runtime.irradiance_gamma, 1.0f)"), std::string::npos);
  EXPECT_NE(render_layer_source.find("lighting_layout_->PushDescriptorBinding(19, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER"),
            std::string::npos);
  EXPECT_NE(render_layer_source.find("UpdateBufferDescriptorBinding(kDdgiLightingProbeStateBinding"),
            std::string::npos);
  EXPECT_NE(render_layer_source.find("CreateDdgiFallbackProbeStateBuffer(const uint64_t byte_size)"),
            std::string::npos);
  EXPECT_NE(render_layer_source.find("glm::vec4(0.0f, 0.0f, 0.0f, 1.0f)"), std::string::npos);
  EXPECT_NE(render_layer_source.find("CreateDdgiFallbackProbeStateBuffer(sizeof(glm::vec4))"), std::string::npos);
}

TEST(DdgiVolume, DdgiAmbientCompositionReplacesOnlyValidDiffuseCoverage) {
  const auto smoothstep = [](const float edge0, const float edge1, const float value) {
    const float t = glm::clamp((value - edge0) / (edge1 - edge0), 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
  };
  const auto rough_specular_visibility = [&](const float material_occlusion, const float screen_space_visibility,
                                             const float ddgi_visibility, const float roughness,
                                             const float normal_dot_view) {
    const float material_visibility = glm::clamp(material_occlusion, 0.0f, 1.0f);
    const float screen_visibility = glm::clamp(screen_space_visibility, 0.0f, 1.0f);
    const float probe_visibility = glm::clamp(ddgi_visibility, 0.0f, 1.0f);
    const float scalar_visibility = (std::min)(material_visibility, (std::min)(screen_visibility, probe_visibility));
    const float clamped_roughness = glm::clamp(roughness, 0.0f, 1.0f);
    const float lobe_width = clamped_roughness * clamped_roughness;
    const float scalar_occlusion = 1.0f - scalar_visibility;
    const float grazing_occlusion = 0.04f * std::tanh(scalar_occlusion / 0.04f);
    const float view_confidence = smoothstep(0.8f, 1.0f, glm::clamp(normal_dot_view, 0.0f, 1.0f));
    const float trusted_occlusion = glm::mix(grazing_occlusion, scalar_occlusion, view_confidence);
    return 1.0f - lobe_width * trusted_occlusion;
  };
  const auto compose = [&](const float diffuse_ibl, const float ddgi_diffuse, const float specular_ibl,
                           const float diffuse_intensity, const float specular_intensity, const float coverage,
                           const float confidence, const float diffuse_occlusion = 1.0f,
                           const float screen_space_visibility = 1.0f, const float ddgi_visibility = 1.0f,
                           const float roughness = 0.0f, const float normal_dot_view = 1.0f) {
    const float weight = glm::clamp(coverage * confidence, 0.0f, 1.0f);
    const float ddgi_specular_visibility = glm::mix(1.0f, glm::clamp(ddgi_visibility, 0.0f, 1.0f), weight);
    const float specular_visibility = rough_specular_visibility(diffuse_occlusion, screen_space_visibility,
                                                                ddgi_specular_visibility, roughness, normal_dot_view);
    return glm::mix(diffuse_ibl * diffuse_intensity, ddgi_diffuse, weight) * glm::clamp(diffuse_occlusion, 0.0f, 1.0f) +
           specular_ibl * specular_intensity * specular_visibility;
  };

  EXPECT_FLOAT_EQ(compose(2.0f, 6.0f, 3.0f, 1.0f, 1.0f, 0.0f, 1.0f), 5.0f);
  EXPECT_FLOAT_EQ(compose(2.0f, 6.0f, 3.0f, 1.0f, 1.0f, 1.0f, 0.0f), 5.0f);
  EXPECT_FLOAT_EQ(compose(2.0f, 6.0f, 3.0f, 1.0f, 1.0f, 1.0f, 1.0f), 9.0f);
  EXPECT_FLOAT_EQ(compose(2.0f, 6.0f, 3.0f, 2.0f, 0.5f, 0.5f, 0.5f), 6.0f);
  EXPECT_FLOAT_EQ(compose(2.0f, 6.0f, 3.0f, 0.0f, 1.0f, 1.0f, 1.0f), 9.0f);
  EXPECT_FLOAT_EQ(compose(2.0f, 6.0f, 3.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.0f), 3.0f);
  EXPECT_FLOAT_EQ(compose(0.0f, 0.0f, 3.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.0f), 3.0f);
  EXPECT_FLOAT_EQ(compose(0.0f, 0.0f, 3.0f, 1.0f, 0.0f, 1.0f, 1.0f), 0.0f);
  EXPECT_FLOAT_EQ(compose(0.0f, 0.0f, 3.0f, 1.0f, 1.0f, 0.0f, 1.0f, 1.0f, 1.0f, 0.0f, 1.0f), 3.0f);
  EXPECT_FLOAT_EQ(compose(2.0f, 6.0f, 0.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.0f, 1.0f), 6.0f);
  EXPECT_NEAR(compose(0.0f, 0.0f, 3.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.0f, 1.0f), 0.0f, 0.0001f);
  EXPECT_GT(compose(0.0f, 0.0f, 3.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.0f, 0.0f),
            compose(0.0f, 0.0f, 3.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.0f, 1.0f));
  EXPECT_FLOAT_EQ((1.0f - 0.04f) * (1.0f - 1.0f), 0.0f);
}

TEST(DdgiVolume, DdgiRasterPathsKeepScalarAoInputsOutOfDirectAndEmission) {
  const auto shader_root =
      std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "Internals" / "DefaultResources" / "Shaders";
  const auto deferred = ReadTextFile(shader_root / "Compute" / "DeferredComputeLighting.slang");
  const auto transparent =
      ReadTextFile(shader_root / "Graphics" / "Fragment" / "Standard" / "StandardTransparent.slang");
  ASSERT_FALSE(deferred.empty());
  ASSERT_FALSE(transparent.empty());

  EXPECT_NE(deferred.find("resolved_material_ao,"), std::string::npos);
  EXPECT_NE(deferred.find("inAmbientOcclusion.SampleLevel"), std::string::npos);
  EXPECT_NE(transparent.find("surface.specular_f90, surface.occlusion,"), std::string::npos);
  EXPECT_NE(transparent.find("surface.occlusion, 1.0f"), std::string::npos);
  EXPECT_EQ(transparent.find("inAmbientOcclusion"), std::string::npos);
  EXPECT_EQ(deferred.find("ambient * ao"), std::string::npos);
  EXPECT_EQ(transparent.find("ambient * surface.occlusion"), std::string::npos);
  EXPECT_NE(deferred.find("indirect_lighting_debug_view != 0"), std::string::npos);
  EXPECT_NE(transparent.find("indirect_lighting_debug_view != 0"), std::string::npos);
}

TEST(DdgiVolume, DdgiGatherConfidenceAndDiffuseEnergyContracts) {
  const auto fresnel = [](const glm::vec3 f0, const glm::vec3 f90, const float cosine) {
    return f0 + (f90 - f0) * std::pow(1.0f - glm::clamp(cosine, 0.0f, 1.0f), 5.0f);
  };
  const glm::vec3 dielectric_fresnel = fresnel({0.04f, 0.2f, 0.8f}, glm::vec3(1.0f), 0.5f);
  const glm::vec3 diffuse_weight = (glm::vec3(1.0f) - dielectric_fresnel) * (1.0f - 0.25f);
  EXPECT_GT(diffuse_weight.r, diffuse_weight.g);
  EXPECT_GT(diffuse_weight.g, diffuse_weight.b);
  const glm::vec3 metallic_diffuse_weight = (glm::vec3(1.0f) - dielectric_fresnel) * (1.0f - 1.0f);
  EXPECT_FLOAT_EQ(metallic_diffuse_weight.r, 0.0f);
  EXPECT_FLOAT_EQ(metallic_diffuse_weight.g, 0.0f);
  EXPECT_FLOAT_EQ(metallic_diffuse_weight.b, 0.0f);

  struct GatherContractResult {
    float confidence;
    float radiance_weight;
  };
  const auto gather_contract = [](const std::vector<float>& weights, const std::vector<bool>& inactive,
                                  const std::vector<float>& validity, const std::vector<float>& visibility) {
    float active_weight = 0.0f;
    float valid_weight = 0.0f;
    float radiance_weight = 0.0f;
    for (size_t i = 0; i < weights.size(); ++i) {
      if (inactive[i]) {
        continue;
      }
      active_weight += weights[i];
      valid_weight += weights[i] * validity[i];
      radiance_weight += weights[i] * validity[i] * visibility[i];
    }
    return GatherContractResult{active_weight > 0.0f ? valid_weight / active_weight : 0.0f, radiance_weight};
  };
  const std::vector<float> weights(4, 0.25f);
  const std::vector<bool> inactive = {false, false, true, false};
  const std::vector<float> valid(4, 1.0f);
  const auto visible = gather_contract(weights, inactive, valid, std::vector<float>(4, 1.0f));
  const auto occluded = gather_contract(weights, inactive, valid, {0.05f, 0.2f, 0.01f, 0.5f});
  const auto warming = gather_contract(weights, inactive, {1.0f, 0.0f, 0.0f, 1.0f}, std::vector<float>(4, 1.0f));
  const auto valid_black = gather_contract(weights, inactive, valid, std::vector<float>(4, 0.0f));
  const auto cleared_black =
      gather_contract(weights, inactive, std::vector<float>(4, 0.0f), std::vector<float>(4, 0.0f));
  EXPECT_FLOAT_EQ(visible.confidence, 1.0f);
  EXPECT_FLOAT_EQ(occluded.confidence, visible.confidence);
  EXPECT_NEAR(warming.confidence, 2.0f / 3.0f, 1e-6f);
  EXPECT_FLOAT_EQ(valid_black.confidence, 1.0f);
  EXPECT_FLOAT_EQ(valid_black.radiance_weight, 0.0f);
  EXPECT_FLOAT_EQ(cleared_black.confidence, 0.0f);
  EXPECT_LT(occluded.radiance_weight, visible.radiance_weight);
  EXPECT_FLOAT_EQ(
      gather_contract(weights, inactive, std::vector<float>(4, 0.0f), std::vector<float>(4, 1.0f)).confidence, 0.0f);
  EXPECT_FLOAT_EQ(gather_contract(weights, std::vector<bool>(4, true), valid, std::vector<float>(4, 1.0f)).confidence,
                  0.0f);

  const glm::vec3 diffuse_albedo(0.5f, 0.25f, 0.125f);
  const glm::vec3 irradiance(2.0f);
  constexpr float kIndirectIntensity = 3.0f;
  constexpr float kPi = 3.14159265359f;
  const glm::vec3 raw_ddgi_diffuse = diffuse_albedo / kPi * irradiance;
  const glm::vec3 composed_ddgi_diffuse = raw_ddgi_diffuse * kIndirectIntensity;
  EXPECT_NEAR(composed_ddgi_diffuse.r, 3.0f / kPi, 1e-6f);
  EXPECT_NE(composed_ddgi_diffuse.r, (raw_ddgi_diffuse * kIndirectIntensity * kIndirectIntensity).r);
  EXPECT_FLOAT_EQ(glm::clamp(2.0f * 2.0f, 0.0f, 1.0f), 1.0f);
}

TEST(DdgiVolume, ProbeBlendLossMeasuresNonlinearCrossProbeDarkening) {
  const auto blend = [](const float first, const float second, const float weight) {
    const auto linear = glm::mix(first, second, weight);
    const auto nonlinear = std::pow(glm::mix(std::sqrt(first), std::sqrt(second), weight), 2.0f);
    return std::pair(linear, (std::max)(linear - nonlinear, 0.0f));
  };

  const auto [constant, constant_loss] = blend(0.5f, 0.5f, 0.5f);
  EXPECT_FLOAT_EQ(constant, 0.5f);
  EXPECT_NEAR(constant_loss, 0.0f, 1e-6f);
  const auto [high_contrast, high_contrast_loss] = blend(0.0f, 1.0f, 0.5f);
  EXPECT_FLOAT_EQ(high_contrast, 0.5f);
  EXPECT_FLOAT_EQ(high_contrast_loss, 0.25f);
}

TEST(DdgiVolume, DdgiProbeMissRaysSampleSceneEnvironment) {
  const auto shader_root =
      std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "Internals" / "DefaultResources" / "Shaders";
  const auto miss_source = ReadTextFile(shader_root / "RayTracing" / "Miss" / "DDGIProbeTrace.slang");
  const auto render_layer_source =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "src" / "RenderLayer.cpp");
  ASSERT_FALSE(miss_source.empty());
  ASSERT_FALSE(render_layer_source.empty());

  EXPECT_NE(miss_source.find("import EvoEngine.DDGIProbeRayConstants;"), std::string::npos);
  EXPECT_NE(miss_source.find("EE_DDGI_PROBE_RAY_CONSTANTS.selected_probe_volume_flags_environment.w"),
            std::string::npos);
  EXPECT_NE(miss_source.find(".SampleLevel(normalize(EE_ENVIRONMENT_LOCAL_DIRECTION(direction)), 0.0f)"),
            std::string::npos);
  EXPECT_NE(miss_source.find("EE_ENVIRONMENT_LOCAL_DIRECTION(direction)"), std::string::npos);
  EXPECT_NE(miss_source.find("EE_ENVIRONMENT.gamma"), std::string::npos);
  EXPECT_EQ(miss_source.find("EE_ENVIRONMENT.diffuse_fallback_intensity"), std::string::npos);
  EXPECT_NE(miss_source.find("EE_ENVIRONMENT.diffuse_sky_intensity"), std::string::npos);
  EXPECT_EQ(miss_source.find("EE_ENVIRONMENT.global_reflection_intensity"), std::string::npos);
  EXPECT_EQ(miss_source.find("EE_ENVIRONMENT.specular_fallback_intensity"), std::string::npos);
  EXPECT_EQ(miss_source.find("indirect_lighting_intensity"), std::string::npos);

  EXPECT_NE(render_layer_source.find("uint32_t GetDdgiEnvironmentCubemapIndex"), std::string::npos);
  EXPECT_NE(render_layer_source.find("ResolveEnvironmentalLighting(scene)"), std::string::npos);
  EXPECT_NE(render_layer_source.find("ResolveIndirectEnvironmentMap(resolved_lighting.indirect_environment_source)"),
            std::string::npos);
  EXPECT_NE(render_layer_source.find("Resources::GetInstance().GetDefaultEnvironmentalMap()"), std::string::npos);
  EXPECT_EQ(render_layer_source.find("scene->environment.GetGlobalReflectionProbe()"), std::string::npos);
  EXPECT_NE(render_layer_source.find("environment_cubemap_index"), std::string::npos);
  EXPECT_NE(render_layer_source.find("const auto ray_flags ="), std::string::npos);
}

TEST(DdgiVolume, DdgiProbeRelocationUsesStandaloneRtxgiStylePass) {
  const auto shader_root =
      std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "Internals" / "DefaultResources" / "Shaders";
  const auto relocation_source = ReadTextFile(shader_root / "Compute" / "DDGIProbeRelocation.slang");
  const auto update_source = ReadTextFile(shader_root / "Compute" / "DDGIProbeUpdate.slang");
  const auto render_layer_source =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "src" / "RenderLayer.cpp");
  const auto relocation_pass_source = ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" /
                                                   "src" / "RenderPasses" / "DdgiProbeRelocationPass.cpp");
  ASSERT_FALSE(relocation_source.empty());
  ASSERT_FALSE(update_source.empty());
  ASSERT_FALSE(render_layer_source.empty());
  ASSERT_FALSE(relocation_pass_source.empty());

  EXPECT_NE(relocation_source.find("[numthreads(32, 1, 1)]"), std::string::npos);
  EXPECT_NE(relocation_source.find("bool reset_offsets = (constants.probe_count_ray_count_and_flags.w & 1u) != 0u;"),
            std::string::npos);
  EXPECT_NE(
      relocation_source.find("bool scrolled_probes_only = (constants.probe_count_ray_count_and_flags.w & 2u) != 0u;"),
      std::string::npos);
  EXPECT_NE(
      relocation_source.find("EE_DDGI_PROBE_IS_NEWLY_EXPOSED(logical_probe_grid, constants.probe_scroll_delta.xyz"),
      std::string::npos);
  EXPECT_NE(relocation_source.find("state.xyz = float3(0.0f);"), std::string::npos);
  EXPECT_NE(relocation_source.find("uint fixed_ray_count = min(constants.probe_counts.w, uniform_ray_count);"),
            std::string::npos);
  EXPECT_NE(relocation_source.find("fixed_ray_backface_threshold"), std::string::npos);
  EXPECT_NE(relocation_source.find("hit_distance *= 5.0f;"), std::string::npos);
  EXPECT_NE(relocation_source.find("EE_DDGI_RELOCATION_OFFSET_INSIDE_VOXEL"), std::string::npos);
  EXPECT_NE(relocation_source.find("output_state.xyz = relocation_offset;"), std::string::npos);
  EXPECT_EQ(relocation_source.find("EE_DDGI_PROBE_STATE[physical_probe_index].w"), std::string::npos);
  EXPECT_EQ(update_source.find("fixed_ray_backface_threshold"), std::string::npos);
  EXPECT_NE(render_layer_source.find("DDGIProbeRelocation.slang"), std::string::npos);
  EXPECT_NE(render_layer_source.find("runtime_state.frame_probe_relocation_reset = reset_probe_state;"),
            std::string::npos);
  EXPECT_NE(render_layer_source.find("geometry_relocation_requested"), std::string::npos);
  EXPECT_NE(render_layer_source.find("relocate_scrolled_probes_only"), std::string::npos);
  EXPECT_NE(render_layer_source.find("runtime_state.frame_probe_relocation_enabled = relocation_requested;"),
            std::string::npos);
  EXPECT_EQ(render_layer_source.find(
                "runtime_state.frame_probe_relocation_enabled = ddgi_ray_source.enable_probe_relocation;"),
            std::string::npos);
  EXPECT_NE(render_layer_source.find("DdgiProbeRelocationPass::CreateDescriptor()"), std::string::npos);
  EXPECT_NE(relocation_pass_source.find("RenderPassNames::ddgi_probe_relocation"), std::string::npos);
  EXPECT_NE(relocation_pass_source.find("descriptor.dependencies = {RenderPassNames::ddgi_probe_update};"),
            std::string::npos);
  EXPECT_NE(relocation_pass_source.find("RenderResourceNames::frame_ddgi_probe_state, RenderResourceUsage::ReadWrite"),
            std::string::npos);
  EXPECT_EQ(relocation_source.find(".hlsl"), std::string::npos);
}

TEST(DdgiVolume, DdgiProbeClassificationUsesStandaloneRtxgiStylePass) {
  const auto shader_root =
      std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "Internals" / "DefaultResources" / "Shaders";
  const auto classification_source = ReadTextFile(shader_root / "Compute" / "DDGIProbeClassification.slang");
  const auto update_source = ReadTextFile(shader_root / "Compute" / "DDGIProbeUpdate.slang");
  const auto render_layer_source =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "src" / "RenderLayer.cpp");
  const auto classification_pass_source =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "src" / "RenderPasses" /
                   "DdgiProbeClassificationPass.cpp");
  ASSERT_FALSE(classification_source.empty());
  ASSERT_FALSE(update_source.empty());
  ASSERT_FALSE(render_layer_source.empty());
  ASSERT_FALSE(classification_pass_source.empty());

  EXPECT_NE(classification_source.find("[numthreads(32, 1, 1)]"), std::string::npos);
  EXPECT_NE(
      classification_source.find("bool reset_classification = constants.probe_count_ray_count_and_flags.w != 0u;"),
      std::string::npos);
  EXPECT_NE(classification_source.find("EE_DDGI_WRITE_PROBE_CLASSIFICATION(probe_index, 0.0f);"), std::string::npos);
  EXPECT_NE(classification_source.find("uint fixed_ray_count = min(constants.probe_counts.w, uniform_ray_count);"),
            std::string::npos);
  EXPECT_NE(classification_source.find("backface_count / float(fixed_ray_count) > fixed_ray_backface_threshold"),
            std::string::npos);
  EXPECT_NE(classification_source.find("EE_DDGI_VOXEL_PLANE_DISTANCE"), std::string::npos);
  EXPECT_NE(classification_source.find("EE_DDGI_WRITE_PROBE_CLASSIFICATION(physical_probe_index, 1.0f);"),
            std::string::npos);
  EXPECT_NE(classification_source.find("EE_DDGI_WRITE_PROBE_CLASSIFICATION(physical_probe_index, 0.0f);"),
            std::string::npos);
  EXPECT_EQ(classification_source.find("EE_DDGI_PROBE_STATE[physical_probe_index].xyz"), std::string::npos);
  EXPECT_EQ(update_source.find("classification_inside_geometry"), std::string::npos);
  EXPECT_EQ(update_source.find("!classification_has_nearby_geometry"), std::string::npos);
  EXPECT_NE(render_layer_source.find("DDGIProbeClassification.slang"), std::string::npos);
  EXPECT_NE(render_layer_source.find("runtime_state.frame_probe_classification_reset = reset_probe_state;"),
            std::string::npos);
  EXPECT_NE(render_layer_source.find(
                "runtime_state.frame_probe_classification_enabled = ddgi_ray_source.enable_probe_classification;"),
            std::string::npos);
  EXPECT_NE(render_layer_source.find("DdgiProbeClassificationPass::CreateDescriptor()"), std::string::npos);
  EXPECT_NE(classification_pass_source.find("RenderPassNames::ddgi_probe_classification"), std::string::npos);
  EXPECT_NE(classification_pass_source.find("descriptor.dependencies = {RenderPassNames::ddgi_probe_update};"),
            std::string::npos);
  EXPECT_NE(
      classification_pass_source.find("RenderResourceNames::frame_ddgi_probe_state, RenderResourceUsage::ReadWrite"),
      std::string::npos);
  EXPECT_EQ(classification_source.find(".hlsl"), std::string::npos);
}

TEST(DdgiVolume, DdgiInputSignaturesTrackOnlyRayVisibleMaterialClosure) {
  const auto render_layer_source =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "src" / "RenderLayer.cpp");
  const auto material_signature = ExtractBetween(render_layer_source, "uint64_t MakeDdgiMaterialSignature",
                                                 "std::vector<uint16_t> GetDdgiMaterialTextureSlots");
  const auto texture_slots = ExtractBetween(render_layer_source, "std::vector<uint16_t> GetDdgiMaterialTextureSlots",
                                            "DdgiMaterialInputSignatures CollectDdgiMaterialInputSignatures");
  ASSERT_FALSE(material_signature.empty());
  ASSERT_FALSE(texture_slots.empty());

  for (const auto* field : {"pbr_base_color_factor", "normal_texture_scale", "pbr_metallic_factor", "alpha_mode",
                            "alpha_cutoff", "double_sided", "ior", "transmission_factor", "thickness_factor",
                            "clearcoat_factor", "clearcoat_normal_texture_scale", "specular_color_factor",
                            "specular_factor", "unlit", "diffuse_transmission_factor"}) {
    EXPECT_NE(material_signature.find(field), std::string::npos) << field;
  }
  for (const auto* field :
       {"emissive_factor", "pbr_roughness_factor", "occlusion_texture_strength", "attenuation_color",
        "attenuation_distance", "clearcoat_roughness", "sheen_color_factor", "sheen_roughness_factor", "pbr_model",
        "pbr_diffuse_factor", "pbr_specular_factor", "pbr_glossiness_factor"}) {
    EXPECT_EQ(material_signature.find(field), std::string::npos) << field;
  }

  for (const auto* field :
       {"pbr_base_color_texture", "normal_texture", "pbr_metallic_roughness_texture", "clearcoat_texture",
        "clearcoat_normal_texture", "specular_texture", "specular_color_texture"}) {
    EXPECT_NE(texture_slots.find(field), std::string::npos) << field;
  }
  EXPECT_EQ(CountOccurrences(texture_slots, "material."), 7u);
  for (const auto* field : {"emissive_texture", "occlusion_texture", "transmission_texture", "thickness_texture",
                            "clearcoat_roughness_texture", "sheen_color_texture", "sheen_roughness_texture",
                            "pbr_diffuse_texture", "pbr_specular_glossiness_texture"}) {
    EXPECT_EQ(texture_slots.find(field), std::string::npos) << field;
  }
  EXPECT_EQ(render_layer_source.find("GetGltfMaterialTextureSlots"), std::string::npos);
  EXPECT_NE(render_layer_source.find("if (emissive_inventory_changed)"), std::string::npos);
  EXPECT_NE(render_layer_source.find("scene_change_triggers |= DdgiVolumeTriggerConditionLightingConditionChanged"),
            std::string::npos);
}

TEST(DdgiVolume, DdgiTextureAndEnvironmentSignaturesTrackGpuVisibleContent) {
  const auto sdk_root = std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK";
  const auto render_layer_source = ReadTextFile(sdk_root / "src" / "RenderLayer.cpp");
  const auto texture_storage_source = ReadTextFile(sdk_root / "src" / "TextureStorage.cpp");
  const auto cubemap_source = ReadTextFile(sdk_root / "src" / "Cubemap.cpp");
  const auto reflection_probe_source = ReadTextFile(sdk_root / "src" / "GlobalReflectionProbe.cpp");
  const auto light_probe_source = ReadTextFile(sdk_root / "src" / "LightProbe.cpp");
  const auto texture_signature =
      ExtractBetween(texture_storage_source, "bool TextureStorage::TryGetTexture2DContentSignature",
                     "bool TextureStorage::TryGetCubemapContentSignature");
  const auto cubemap_signature =
      ExtractBetween(texture_storage_source, "bool TextureStorage::TryGetCubemapContentSignature",
                     "bool TextureStorage::HasPendingTexture2DUpload");
  const auto pending_texture = ExtractBetween(texture_storage_source, "bool TextureStorage::HasPendingTexture2DUpload",
                                              "bool TextureStorage::HasPendingUploads");
  const auto environment_signature = ExtractBetween(render_layer_source, "uint64_t MakeDdgiEnvironmentSignature",
                                                    "struct DdgiMaterialInputSignatures");
  ASSERT_FALSE(texture_signature.empty());
  ASSERT_FALSE(cubemap_signature.empty());
  ASSERT_FALSE(pending_texture.empty());
  ASSERT_FALSE(environment_signature.empty());

  for (const auto* input :
       {"texture.image.get()", "texture.image_view.get()", "texture.sampler.get()", "texture.view_format_",
        "texture.samples_linear_srgb_", "texture.gpu_upload_generation->load()", "sampler.magFilter",
        "sampler.minFilter", "sampler.mipmapMode", "sampler.addressModeU", "sampler.addressModeV",
        "sampler.addressModeW", "sampler.mipLodBias", "sampler.maxAnisotropy", "sampler.minLod", "sampler.maxLod"}) {
    EXPECT_NE(texture_signature.find(input), std::string::npos) << input;
  }
  EXPECT_NE(cubemap_signature.find("cubemap.content_generation"), std::string::npos);
  EXPECT_NE(cubemap_source.find("++storage.content_generation;"), std::string::npos);
  EXPECT_NE(reflection_probe_source.find("filtered_cubemap->MarkGpuContentValid();"), std::string::npos);
  EXPECT_NE(light_probe_source.find("cubemap_->MarkGpuContentValid();"), std::string::npos);
  EXPECT_NE(environment_signature.find("selected_cubemap_index"), std::string::npos);
  EXPECT_NE(environment_signature.find("TryGetCubemapContentSignature"), std::string::npos);
  EXPECT_NE(environment_signature.find("environment.diffuse_sky_intensity"), std::string::npos);
  EXPECT_EQ(environment_signature.find("environment.diffuse_fallback_intensity"), std::string::npos);
  EXPECT_NE(environment_signature.find("environment.environment_rotation"), std::string::npos);
  EXPECT_EQ(environment_signature.find("environment.global_reflection_intensity"), std::string::npos);
  EXPECT_EQ(environment_signature.find("environment.specular_fallback_intensity"), std::string::npos);
  EXPECT_EQ(environment_signature.find("indirect_lighting_intensity"), std::string::npos);
  EXPECT_EQ(environment_signature.find("background_intensity"), std::string::npos);
  for (const auto* pending_input : {"texture.new_data_", "texture.new_compressed_data_", "IsGpuUploadPending()"}) {
    EXPECT_NE(pending_texture.find(pending_input), std::string::npos) << pending_input;
  }
  EXPECT_NE(environment_signature.find("environment.diffuse_sky_intensity <= 0.0f"), std::string::npos);
  EXPECT_NE(render_layer_source.find("GetDdgiEnvironmentCubemapIndex(scene)"), std::string::npos);
}

TEST(DdgiVolume, DdgiContributorFilterMatchesTlasAdmission) {
  const auto render_layer_source =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "src" / "RenderLayer.cpp");
  const auto contributor_filter =
      ExtractBetween(render_layer_source, "bool IsDdgiTlasContributor", "uint64_t MakeDdgiGeometrySignature");
  const auto geometry_signature =
      ExtractBetween(render_layer_source, "uint64_t MakeDdgiGeometrySignature", "bool DdgiTriggerConditionEnabled");
  ASSERT_FALSE(contributor_filter.empty());
  ASSERT_FALSE(geometry_signature.empty());

  for (const auto* input : {"MeshRenderInstance", "mesh->mesh", "SkinnedMeshRenderInstance", "skinned->skinned_mesh",
                            "InstancedRenderInstance", "ExternalRenderInstance", "HasDdgiRayTracingGeometry",
                            "IsReady()", "IsDdgiAccelerationStructureTransformValid"}) {
    EXPECT_NE(contributor_filter.find(input), std::string::npos) << input;
  }
  for (const auto* collection :
       {"deferred_render_instances", "deferred_skinned_render_instances", "deferred_instanced_render_instances",
        "deferred_masked_render_instances", "deferred_masked_skinned_render_instances",
        "deferred_masked_instanced_render_instances", "forward_render_instances", "forward_skinned_render_instances",
        "forward_instanced_render_instances", "transparent_render_instances", "transparent_skinned_render_instances",
        "transparent_instanced_render_instances", "external_render_instances"}) {
    EXPECT_NE(render_layer_source.find("collect_ddgi_geometry_signatures(current_render_instances->" +
                                       std::string(collection) + ")"),
              std::string::npos)
        << collection;
  }
  EXPECT_EQ(render_layer_source.find("collect_ddgi_geometry_signatures(current_render_instances->deferred_strands"),
            std::string::npos);
  EXPECT_EQ(render_layer_source.find("collect_ddgi_geometry_signatures(current_render_instances->gaussian"),
            std::string::npos);
  EXPECT_NE(geometry_signature.find("particle_info.instance_matrix.value"), std::string::npos);
  EXPECT_EQ(geometry_signature.find("particle_info.instance_color"), std::string::npos);
  EXPECT_EQ(geometry_signature.find("particle_info_list_version"), std::string::npos);
}

TEST(DdgiVolume, DdgiEmissiveFingerprintUsesStableSemanticKeys) {
  const auto render_instance_source = ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" /
                                                   "src" / "RenderInstanceStorage.cpp");
  const auto semantic_hash = ExtractBetween(render_instance_source, "uint64_t HashDdgiEmissiveInventorySignature",
                                            "GltfRasterMaterialClass ResolveRasterMaterialClass");
  const auto cache_equality = ExtractBetween(
      render_instance_source, "bool RenderInstanceStorage::EmissiveTriangleInstanceSignature::operator==",
      "std::vector<RenderInstanceStorage::EmissiveTriangleInfoBlock>");
  const auto inventory_builder =
      ExtractBetween(render_instance_source, "void RenderInstanceStorage::BuildEmissiveTriangleInfoBlocks()",
                     "void RenderInstanceStorage::CollectLights");
  ASSERT_FALSE(semantic_hash.empty());
  ASSERT_FALSE(cache_equality.empty());
  ASSERT_FALSE(inventory_builder.empty());
  EXPECT_NE(semantic_hash.find("entry.material_handle"), std::string::npos);
  EXPECT_NE(semantic_hash.find("entry.emissive_sampling_signature"), std::string::npos);
  EXPECT_NE(semantic_hash.find("entry.geometry_version"), std::string::npos);
  EXPECT_NE(semantic_hash.find("entry.model.value"), std::string::npos);
  EXPECT_NE(semantic_hash.find("entry.importance"), std::string::npos);
  EXPECT_EQ(semantic_hash.find("material_index"), std::string::npos);
  EXPECT_EQ(semantic_hash.find("instance_index"), std::string::npos);
  EXPECT_EQ(semantic_hash.find("triangle_offset"), std::string::npos);
  EXPECT_NE(cache_equality.find("material_index == other.material_index"), std::string::npos);
  EXPECT_NE(cache_equality.find("instance_index == other.instance_index"), std::string::npos);
  EXPECT_NE(cache_equality.find("triangle_offset == other.triangle_offset"), std::string::npos);
  EXPECT_NE(inventory_builder.find("struct DistributionKey"), std::string::npos);
  EXPECT_NE(inventory_builder.find("emissive_instance.unique_distribution ? emissive_instance.instance_index"),
            std::string::npos);
}

TEST(DdgiVolume, DdgiSceneChangeLatchesSurvivePauseUntilAProbeTraceRuns) {
  const auto render_layer_source =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "src" / "RenderLayer.cpp");
  const auto prepare_volume = render_layer_source.find("void RenderLayer::PrepareDdgiVolumeFrameState");
  const auto pause_begin = render_layer_source.find("if (ddgi_session_state_.pause_updates)", prepare_volume);
  const auto pause_end = render_layer_source.find("if (!ddgi_settings.runtime.enabled", pause_begin);
  const auto pause_block = render_layer_source.substr(pause_begin, pause_end - pause_begin);
  ASSERT_NE(prepare_volume, std::string::npos);
  ASSERT_NE(pause_begin, std::string::npos);
  ASSERT_NE(pause_end, std::string::npos);
  ASSERT_FALSE(pause_block.empty());
  EXPECT_EQ(pause_block.find("runtime_state.latched_scene_change_triggers ="), std::string::npos);
  EXPECT_EQ(pause_block.find("AdvanceHysteresisBoost"), std::string::npos);
  EXPECT_EQ(pause_block.find("runtime_state.current_probe_hysteresis ="), std::string::npos);
  EXPECT_EQ(pause_block.find("reset_probe_history = false"), std::string::npos);
  EXPECT_NE(pause_block.find("track_ddgi_environment_signature()"), std::string::npos);
  EXPECT_EQ(pause_block.find("runtime_state.previous_emissive_mesh_sampling_enabled ="), std::string::npos);
  const auto resolve_effective = render_layer_source.find(
      "runtime_state.emissive_mesh_sampling_enabled = ddgi_ray_source.emissive_mesh_sampling_enabled;");
  const auto pause_branch = render_layer_source.find("if (ddgi_session_state_.pause_updates)", resolve_effective);
  const auto compare_effective = render_layer_source.find(
      "runtime_state.previous_emissive_mesh_sampling_enabled != runtime_state.emissive_mesh_sampling_enabled",
      pause_branch);
  const auto latch_effective = render_layer_source.find(
      "runtime_state.previous_emissive_mesh_sampling_enabled = runtime_state.emissive_mesh_sampling_enabled",
      compare_effective);
  ASSERT_NE(resolve_effective, std::string::npos);
  ASSERT_NE(pause_branch, std::string::npos);
  ASSERT_NE(compare_effective, std::string::npos);
  ASSERT_NE(latch_effective, std::string::npos);
  EXPECT_LT(resolve_effective, pause_branch);
  EXPECT_LT(pause_branch, compare_effective);
  EXPECT_LT(compare_effective, latch_effective);
  EXPECT_NE(render_layer_source.find("runtime->latched_scene_change_triggers |= ddgi_latched_scene_change_triggers_"),
            std::string::npos);
  EXPECT_NE(render_layer_source.find("ddgi_latched_scene_change_triggers_ |= scene_change_triggers"),
            std::string::npos);
  EXPECT_EQ(render_layer_source.find("ddgi_latched_scene_geometry_changed_"), std::string::npos);
  EXPECT_EQ(render_layer_source.find("probe_state_geometry_changed"), std::string::npos);
  EXPECT_EQ(
      CountOccurrences(render_layer_source, "ddgi_latched_scene_change_triggers_ = DdgiVolumeTriggerConditionNone;"),
      2u);
  EXPECT_EQ(render_layer_source.find("if (!should_trace_probe_rays)"), std::string::npos);
  const auto no_trace_return = pause_branch;
  const auto clear_triggers = render_layer_source.find(
      "runtime_state.latched_scene_change_triggers = DdgiVolumeTriggerConditionNone;", no_trace_return);
  const auto trace_enabled = render_layer_source.find("runtime_state.frame_trace_probe_rays = true;", clear_triggers);
  ASSERT_NE(no_trace_return, std::string::npos);
  ASSERT_NE(clear_triggers, std::string::npos);
  ASSERT_NE(trace_enabled, std::string::npos);
  EXPECT_LT(no_trace_return, clear_triggers);
  EXPECT_LT(clear_triggers, trace_enabled);

  const auto prepare_set = ExtractBetween(render_layer_source, "void RenderLayer::PrepareDdgiFrameState",
                                          "void RenderLayer::PrepareDdgiVolumeFrameState");
  ASSERT_FALSE(prepare_set.empty());
  const auto latch_reset = prepare_set.find("runtime->manual_reset_pending |= reset_probe_history;");
  const auto consume_reset = prepare_set.find("runtime->manual_reset_pending = false;");
  const auto reset_consumed = prepare_set.find("const bool reset_consumed");
  const auto clear_reset = prepare_set.find("ddgi_session_state_.reset_history_requested = false;", reset_consumed);
  ASSERT_NE(latch_reset, std::string::npos);
  ASSERT_NE(consume_reset, std::string::npos);
  ASSERT_NE(reset_consumed, std::string::npos);
  ASSERT_NE(clear_reset, std::string::npos);
  EXPECT_LT(latch_reset, consume_reset);
  EXPECT_LT(consume_reset, clear_reset);
  EXPECT_NE(prepare_set.find("info.sorted_index, runtime->manual_reset_pending"), std::string::npos);
  EXPECT_NE(prepare_set.find("if (runtime->frame_trace_probe_rays)"), std::string::npos);
}

TEST(DdgiVolume, RenderLayerDefersProbeTracingUntilSceneInputsAreReady) {
  const auto source_root = std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK";
  const auto render_layer_source = ReadTextFile(source_root / "src" / "RenderLayer.cpp");
  const auto render_layer_header = ReadTextFile(source_root / "include" / "Layers" / "RenderLayer.hpp");
  const auto texture_storage_source = ReadTextFile(source_root / "src" / "TextureStorage.cpp");
  ASSERT_FALSE(render_layer_source.empty());
  ASSERT_FALSE(render_layer_header.empty());
  ASSERT_FALSE(texture_storage_source.empty());

  EXPECT_NE(render_layer_header.find("ddgi_deferred_scene_readiness_refresh_"), std::string::npos);
  EXPECT_NE(render_layer_header.find("uint32_t scene_input_settle_frame_count = 0;"), std::string::npos);
  EXPECT_NE(render_layer_header.find("ddgi_referenced_scene_inputs_pending_"), std::string::npos);
  EXPECT_NE(render_layer_source.find("#include \"ProjectManager.hpp\""), std::string::npos);
  EXPECT_NE(render_layer_source.find("kDdgiSceneInputSettleFrameCount = 1"), std::string::npos);
  const auto pending_gate =
      ExtractBetween(render_layer_source, "const bool startup_scene_inputs_pending", "const auto ray_count");
  ASSERT_FALSE(pending_gate.empty());
  EXPECT_NE(pending_gate.find("!compatible_history"), std::string::npos);
  EXPECT_NE(pending_gate.find("TextureStorage::HasPendingUploads()"), std::string::npos);
  EXPECT_NE(pending_gate.find("ProjectManager::HasProject() && !ProjectManager::IsProjectIdle()"), std::string::npos);
  EXPECT_NE(pending_gate.find("ddgi_referenced_scene_inputs_pending_ || startup_scene_inputs_pending"),
            std::string::npos);
  EXPECT_EQ(pending_gate.find("runtime_state.clear_probe_atlas_this_frame = true"), std::string::npos);
  const auto scene_pending_gate = render_layer_source.find("if (scene_inputs_pending) {");
  const auto ray_count_setup = render_layer_source.find("const auto ray_count", scene_pending_gate);
  ASSERT_NE(scene_pending_gate, std::string::npos);
  ASSERT_NE(ray_count_setup, std::string::npos);
  EXPECT_LT(scene_pending_gate, ray_count_setup);
  EXPECT_NE(render_layer_source.find("runtime_state.deferred_scene_readiness_refresh = true;"), std::string::npos);
  EXPECT_NE(render_layer_source.find("runtime_state.scene_input_settle_frame_count = 0;"), std::string::npos);
  const auto settle_gate = render_layer_source.find("runtime_state.scene_input_settle_frame_count <");
  ASSERT_NE(settle_gate, std::string::npos);
  EXPECT_LT(settle_gate, ray_count_setup);
  EXPECT_NE(render_layer_source.find("++runtime_state.scene_input_settle_frame_count;"), std::string::npos);
  EXPECT_NE(
      render_layer_source.find("const bool scene_readiness_refresh = runtime_state.deferred_scene_readiness_refresh;"),
      std::string::npos);
  const auto refresh_classification =
      ExtractBetween(render_layer_source, "const bool hard_ddgi_refresh", "if (hard_ddgi_refresh)");
  ASSERT_FALSE(refresh_classification.empty());
  EXPECT_EQ(refresh_classification.find("scene_readiness_refresh"), std::string::npos);
  EXPECT_NE(refresh_classification.find("ddgi_persistent_resource_changed"), std::string::npos);
  EXPECT_EQ(render_layer_source.find("should_trace_probe_rays"), std::string::npos);
  EXPECT_NE(texture_storage_source.find("bool TextureStorage::HasPendingTexture2DUpload"), std::string::npos);
  EXPECT_NE(render_layer_source.find("result.inputs_pending |= TextureStorage::HasPendingTexture2DUpload"),
            std::string::npos);
  const auto refresh_consumed = render_layer_source.find("ddgi_deferred_scene_readiness_refresh_ = false;");
  const auto tracing_enabled = render_layer_source.find("runtime_state.frame_trace_probe_rays = true;");
  ASSERT_NE(refresh_consumed, std::string::npos);
  ASSERT_NE(tracing_enabled, std::string::npos);
  EXPECT_LT(refresh_consumed, tracing_enabled);
  EXPECT_NE(render_layer_source.find("runtime_state.scene_input_settle_frame_count = 0;", refresh_consumed),
            std::string::npos);
}

TEST(DdgiVolume, OffscreenPreviewRenderingDoesNotTouchSceneDdgiTracking) {
  const auto source_root = std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK";
  const auto render_layer_source = ReadTextFile(source_root / "src" / "RenderLayer.cpp");
  ASSERT_FALSE(render_layer_source.empty());

  const auto immediate_render = render_layer_source.find("void RenderLayer::RenderSceneToCameraImmediately");
  const auto temporary_storage = render_layer_source.find(
      "render_instances_list_[current_frame_index] = std::make_shared<RenderInstanceStorage>();", immediate_render);
  const auto restore_definition = render_layer_source.find("const auto restore = [&]", immediate_render);
  const auto isolated_prepare = render_layer_source.find(
      "PrepareSceneForRendering(scene, false, false, false, false, reflection_probe_capture ? &injected_cameras : "
      "nullptr,",
      immediate_render);
  const auto camera_render = render_layer_source.find(
      "RenderToCamera(scene, camera_global_transform, camera, true, reflection_probe_capture);", immediate_render);
  const auto restore_storage = render_layer_source.find(
      "render_instances_list_[current_frame_index] = previous_render_instances;", immediate_render);
  const auto rebind_previous = render_layer_source.find(
      "BindRenderInstanceStorage(current_frame_index, previous_render_instances);", immediate_render);
  const auto restore_call = render_layer_source.find("  restore();", camera_render);
  ASSERT_NE(immediate_render, std::string::npos);
  ASSERT_NE(restore_definition, std::string::npos);
  ASSERT_NE(temporary_storage, std::string::npos);
  ASSERT_NE(isolated_prepare, std::string::npos);
  ASSERT_NE(camera_render, std::string::npos);
  ASSERT_NE(restore_storage, std::string::npos);
  ASSERT_NE(rebind_previous, std::string::npos);
  ASSERT_NE(restore_call, std::string::npos);
  EXPECT_LT(restore_definition, restore_storage);
  EXPECT_LT(restore_storage, rebind_previous);
  EXPECT_LT(rebind_previous, temporary_storage);
  EXPECT_LT(temporary_storage, isolated_prepare);
  EXPECT_LT(isolated_prepare, camera_render);
  EXPECT_LT(camera_render, restore_call);
}

TEST(DdgiVolume, OffscreenPreviewThumbnailGpuCopyCompletesBeforeReturn) {
  const auto source_root = std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK";
  const auto preview_source = ReadTextFile(source_root / "src" / "OffscreenPreviewRenderer.cpp");
  ASSERT_FALSE(preview_source.empty());

  const auto copy_texture =
      preview_source.find("std::shared_ptr<Texture2D> OffscreenPreviewRenderer::CopyColorTexture");
  const auto submit = preview_source.find("Platform::ImmediateSubmit", copy_texture);
  const auto copy = preview_source.find("vkCmdCopyImage", submit);
  const auto return_texture = preview_source.find("return texture;", copy);
  ASSERT_NE(copy_texture, std::string::npos);
  ASSERT_NE(submit, std::string::npos);
  ASSERT_NE(copy, std::string::npos);
  ASSERT_NE(return_texture, std::string::npos);
  EXPECT_LT(submit, copy);
  EXPECT_LT(copy, return_texture);
  EXPECT_EQ(preview_source.find("texture->UnsafeUploadDataImmediately();", copy_texture), std::string::npos);
}

TEST(DdgiVolume, OffscreenPreviewCreatesDirectionalLightWhenSceneHasNone) {
  const auto source_root = std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK";
  const auto preview_source = ReadTextFile(source_root / "src" / "OffscreenPreviewRenderer.cpp");
  ASSERT_FALSE(preview_source.empty());

  const auto configure_lighting = preview_source.find("void ConfigurePreviewLighting");
  const auto missing_light_gate =
      preview_source.find("if (!light_owners || light_owners->empty())", configure_lighting);
  const auto create_light =
      preview_source.find("configure_light(scene->CreateEntity(\"Preview Directional Light\"));", missing_light_gate);
  const auto shadow_off = preview_source.find("light->cast_shadow = false;", configure_lighting);
  const auto brightness = preview_source.find("light->diffuse_brightness = 2.4f;", configure_lighting);
  ASSERT_NE(configure_lighting, std::string::npos);
  ASSERT_NE(missing_light_gate, std::string::npos);
  ASSERT_NE(create_light, std::string::npos);
  EXPECT_LT(missing_light_gate, create_light);
  EXPECT_NE(shadow_off, std::string::npos);
  EXPECT_NE(brightness, std::string::npos);
}

TEST(DdgiVolume, RenderInstanceBuffersUploadBeforeDescriptorBinding) {
  const auto source_root = std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK";
  const auto render_layer_source = ReadTextFile(source_root / "src" / "RenderLayer.cpp");
  ASSERT_FALSE(render_layer_source.empty());

  const auto prepare = render_layer_source.find("void RenderLayer::PrepareSceneForRendering");
  const auto render_immediate = render_layer_source.find("void RenderLayer::RenderSceneToCameraImmediately", prepare);
  const auto upload = render_layer_source.find("current_render_instances->Upload(immediate_upload);", prepare);
  const auto bind =
      render_layer_source.find("BindRenderInstanceStorage(current_frame_index, current_render_instances);", upload);
  ASSERT_NE(prepare, std::string::npos);
  ASSERT_NE(render_immediate, std::string::npos);
  ASSERT_NE(upload, std::string::npos);
  ASSERT_NE(bind, std::string::npos);
  EXPECT_LT(upload, bind);
  EXPECT_LT(bind, render_immediate);
}

TEST(DdgiVolume, RasterTransmissionOpacityDrivesTransparentOutput) {
  const auto shader_root =
      std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "Internals" / "DefaultResources" / "Shaders";
  const auto raster_material_source =
      ReadTextFile(shader_root / "Modules" / "EvoEngine" / "GltfBindlessMaterial.slang");
  const auto transparent_source =
      ReadTextFile(shader_root / "Graphics" / "Fragment" / "Standard" / "StandardTransparent.slang");
  ASSERT_FALSE(raster_material_source.empty());
  ASSERT_FALSE(transparent_source.empty());

  EXPECT_NE(raster_material_source.find("float EE_GLTF_RASTER_OPACITY(GltfRasterMaterial surface)"), std::string::npos);
  EXPECT_NE(raster_material_source.find("max(surface.transmission, surface.diffuse_transmission_factor)"),
            std::string::npos);
  EXPECT_NE(raster_material_source.find("return alpha * (1.0 - transmission);"), std::string::npos);
  EXPECT_NE(transparent_source.find("return float4(direct + emission + ambient, EE_GLTF_RASTER_OPACITY(surface));"),
            std::string::npos);
}

TEST(DdgiVolume, RasterShadowFilteringUsesEightSampleShaderConstant) {
  const auto shader_root =
      std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "Internals" / "DefaultResources" / "Shaders";
  const auto lighting_source = ReadTextFile(shader_root / "Modules" / "EvoEngine" / "Lighting.slang");
  ASSERT_FALSE(lighting_source.empty());

  EXPECT_NE(lighting_source.find("float EE_FUNC_SPOT_SHADOW_DEPTH(SpotLight light, float2 lightUv)"),
            std::string::npos);
  EXPECT_NE(lighting_source.find("float EE_FUNC_POINT_SHADOW_DEPTH(PointLight light, int slice, float2 lightUv)"),
            std::string::npos);
  EXPECT_NE(lighting_source.find("light.cutoff_outer_inner_size_bias.z * 100.0f"), std::string::npos);
  EXPECT_NE(lighting_source.find("light.reserved_parameters.y * 100.0f"), std::string::npos);
  EXPECT_NE(lighting_source.find("static const int EE_SHADOW_PCF_SAMPLE_COUNT = 8;"), std::string::npos);
  EXPECT_EQ(CountOccurrences(lighting_source,
                             "for (int sampleIndex = 0; sampleIndex < EE_SHADOW_PCF_SAMPLE_COUNT; sampleIndex++)"),
            3u);
  EXPECT_EQ(lighting_source.find("shadow_sample_size"), std::string::npos);
  EXPECT_EQ(lighting_source.find("shadow_debug_parameters.w"), std::string::npos);
  EXPECT_EQ(lighting_source.find("BLOCKER_SEARCH"), std::string::npos);
  EXPECT_EQ(lighting_source.find("penumbraWidth"), std::string::npos);
  EXPECT_EQ(lighting_source.find("blockerDistance"), std::string::npos);
}

TEST(DdgiVolume, DdgiProbeHitsExplicitlyEvaluateAnalyticSceneLights) {
  const auto shader_root =
      std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "Internals" / "DefaultResources" / "Shaders";
  const auto closest_hit_source = ReadTextFile(shader_root / "RayTracing" / "ClosestHit" / "DDGIProbeTrace.slang");
  ASSERT_FALSE(closest_hit_source.empty());

  EXPECT_NE(closest_hit_source.find("EE_DDGI_EXPLICIT_DIRECTIONAL_LIGHT_IRRADIANCE"), std::string::npos);
  EXPECT_NE(closest_hit_source.find("EE_DDGI_EXPLICIT_POINT_LIGHT_IRRADIANCE"), std::string::npos);
  EXPECT_NE(closest_hit_source.find("EE_DDGI_EXPLICIT_SPOT_LIGHT_IRRADIANCE"), std::string::npos);
  EXPECT_NE(closest_hit_source.find("EE_DDGI_LAMBERT_IRRADIANCE"), std::string::npos);
  EXPECT_NE(closest_hit_source.find("EE_DDGI_DISTANCE_LIGHT_ATTENUATION"), std::string::npos);
  EXPECT_NE(closest_hit_source.find("light_distance >= constant_linear_quadratic_far.w"), std::string::npos);
  EXPECT_NE(closest_hit_source.find("EE_RENDER_INFO.directional_light_size"), std::string::npos);
  EXPECT_NE(closest_hit_source.find("EE_RENDER_INFO.point_light_size"), std::string::npos);
  EXPECT_NE(closest_hit_source.find("EE_RENDER_INFO.spot_light_size"), std::string::npos);
  const auto directional_loop = ExtractBetween(closest_hit_source, "for (int i = 0; i < EE_RENDER_INFO.directional",
                                               "for (int i = 0; i < EE_RENDER_INFO.point");
  const auto point_loop = ExtractBetween(closest_hit_source, "for (int i = 0; i < EE_RENDER_INFO.point",
                                         "for (int i = 0; i < EE_RENDER_INFO.spot");
  const auto spot_loop =
      ExtractBetween(closest_hit_source, "for (int i = 0; i < EE_RENDER_INFO.spot", "return irradiance;");
  EXPECT_NE(directional_loop.find("EE_DIRECTIONAL_LIGHTS[i]"), std::string::npos);
  EXPECT_NE(point_loop.find("EE_POINT_LIGHTS[i]"), std::string::npos);
  EXPECT_NE(spot_loop.find("EE_SPOT_LIGHTS[i]"), std::string::npos);
  for (const auto* loop : {&directional_loop, &point_loop, &spot_loop}) {
    EXPECT_NE(loop->find("albedo"), std::string::npos);
    EXPECT_NE(loop->find("normal"), std::string::npos);
    EXPECT_NE(loop->find("geometric_normal"), std::string::npos);
    EXPECT_NE(loop->find("position"), std::string::npos);
  }
  EXPECT_NE(closest_hit_source.find("light.diffuse.w == 1.0f"), std::string::npos);
  EXPECT_NE(closest_hit_source.find("EE_RT_OFFSET_RAY_ORIGIN(position, geometric_normal, light_direction)"),
            std::string::npos);
  EXPECT_EQ(closest_hit_source.find("position + normal * trace_parameters.y"), std::string::npos);
}

TEST(DdgiVolume, DdgiProbeHitsSampleSharedEmissiveTrianglesWithoutMis) {
  const auto shader_root =
      std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "Internals" / "DefaultResources" / "Shaders";
  const auto shared_sampling = ReadTextFile(shader_root / "Modules" / "EvoEngine" / "EmissiveTriangleSampling.slang");
  const auto raygen = ReadTextFile(shader_root / "RayTracing" / "RayGen" / "DDGIProbeTrace.slang");
  const auto closest_hit = ReadTextFile(shader_root / "RayTracing" / "ClosestHit" / "DDGIProbeTrace.slang");
  ASSERT_FALSE(shared_sampling.empty());
  ASSERT_FALSE(raygen.empty());
  ASSERT_FALSE(closest_hit.empty());

  EXPECT_NE(shared_sampling.find("EE_SAMPLE_EMISSIVE_INSTANCE_RECORD(record_sample.x, uniform_sampling)"),
            std::string::npos);
  EXPECT_NE(
      shared_sampling.find("EE_SAMPLE_EMISSIVE_DISTRIBUTION_RECORD(distribution, record_sample.y, uniform_sampling)"),
      std::string::npos);
  EXPECT_NE(shared_sampling.find("EE_GLTF_RASTER_ALPHA_MASK_PASSES_LOD0"), std::string::npos);
  EXPECT_NE(shared_sampling.find("max(radiance, float3(0.0f)) / solid_angle_pdf"), std::string::npos);
  EXPECT_NE(raygen.find("uint(EE_DDGI_PROBE_RAY_CONSTANTS.probe_scroll_offset.w), physical_probe_index, ray_index"),
            std::string::npos);
  EXPECT_NE(raygen.find("fixed_ray ? EE_DDGI_FIXED_RAY_PAYLOAD_FLAG"), std::string::npos);
  EXPECT_NE(raygen.find("primary_ray_seed | EE_DDGI_EMISSIVE_RAY_PAYLOAD_FLAG"), std::string::npos);
  EXPECT_NE(raygen.find("(EE_DDGI_PROBE_RAY_CONSTANTS.selected_probe_volume_flags_environment.z & (1u << 0u)) != 0u"),
            std::string::npos);
  EXPECT_NE(closest_hit.find("0x68bc21ebu"), std::string::npos);
  EXPECT_NE(closest_hit.find("0x967a889bu"), std::string::npos);
  EXPECT_NE(closest_hit.find("0x1b56c4e9u"), std::string::npos);
  EXPECT_NE(closest_hit.find("EE_DDGI_EMISSIVE_MESH_IRRADIANCE("), std::string::npos);
  const auto direct_hit = closest_hit.find("const float3 emissive_radiance =");
  const auto sampling_gate = closest_hit.find(
      "if ((EE_DDGI_PROBE_RAY_CONSTANTS.selected_probe_volume_flags_environment.z & (1u << 1u)) != 0u)");
  const auto explicit_sample =
      closest_hit.find("emissive_mesh_irradiance = EE_DDGI_EMISSIVE_MESH_IRRADIANCE(", sampling_gate);
  const auto shaded_radiance = closest_hit.find("shaded_radiance =", explicit_sample);
  ASSERT_NE(direct_hit, std::string::npos);
  ASSERT_NE(sampling_gate, std::string::npos);
  ASSERT_NE(explicit_sample, std::string::npos);
  ASSERT_NE(shaded_radiance, std::string::npos);
  EXPECT_LT(direct_hit, sampling_gate);
  EXPECT_LT(sampling_gate, explicit_sample);
  EXPECT_LT(explicit_sample, shaded_radiance);
  EXPECT_NE(closest_hit.find("max(max(diffuse_albedo.x, diffuse_albedo.y), diffuse_albedo.z) <= 0.0f"),
            std::string::npos);
  EXPECT_NE(closest_hit.find("diffuse_albedo / EE_DDGI_PI * emissive_sample.radiance_over_pdf"), std::string::npos);
  EXPECT_NE(closest_hit.find("hit_value.seed = shadow_seed"), std::string::npos);
  EXPECT_NE(closest_hit.find("hit_value = primary_hit"), std::string::npos);
  EXPECT_NE(closest_hit.find("emissive_sample.distance - origin_advance - EE_EMISSIVE_TRIANGLE_RAY_EPSILON"),
            std::string::npos);
  EXPECT_EQ(closest_hit.find("emissive_sample.distance - trace_parameters.y"), std::string::npos);
  EXPECT_EQ(closest_hit.find("BALANCE_HEURISTIC"), std::string::npos);
  EXPECT_EQ(closest_hit.find("emissive_radiance *"), std::string::npos);
}

TEST(DdgiVolume, DdgiEmissiveSamplingSeedsAndShadowEndpointAreStable) {
  const auto rotate_left = [](const uint32_t value, const uint32_t count) {
    return (value << count) | (value >> (32u - count));
  };
  const auto xxhash = [&](const uint32_t x, const uint32_t y, const uint32_t z) {
    constexpr uint32_t p0 = 2246822519u;
    constexpr uint32_t p1 = 3266489917u;
    constexpr uint32_t p2 = 668265263u;
    constexpr uint32_t p3 = 374761393u;
    uint32_t hash = z + p3 + x * p1;
    hash = p2 * rotate_left(hash, 17u);
    hash += y * p1;
    hash = p2 * rotate_left(hash, 17u);
    hash = p0 * (hash ^ (hash >> 15u));
    hash = p1 * (hash ^ (hash >> 13u));
    return hash ^ (hash >> 16u);
  };

  const uint32_t primary = xxhash(12345u, 17u, 63u);
  const uint32_t selection = xxhash(primary, 0x68bc21ebu, 0x02e5be93u);
  const uint32_t barycentric = xxhash(primary, 0x967a889bu, 0x368cc8b7u);
  const uint32_t shadow = xxhash(primary, 0x1b56c4e9u, 0xa54ff53au);
  EXPECT_EQ(primary, xxhash(12345u, 17u, 63u));
  EXPECT_NE(selection, barycentric);
  EXPECT_NE(selection, shadow);
  EXPECT_NE(barycentric, shadow);
  EXPECT_NE(1u ^ 0xa511e9b3u, 1u);

  constexpr float sample_distance = 2.0f;
  constexpr float origin_advance = 0.01f;
  constexpr float target_epsilon = 0.001f;
  constexpr float normal_bias = 0.1f;
  const float shadow_distance = (std::max)(sample_distance - origin_advance - target_epsilon, 0.0f);
  EXPECT_FLOAT_EQ(shadow_distance, 1.989f);
  EXPECT_GT(shadow_distance, sample_distance - origin_advance - normal_bias);
}

TEST(DdgiVolume, DdgiAtlasBorderCopyTexelsStayInsideSourceTile) {
  const auto border_copy_texel = [](const glm::uvec2 tile_origin, const uint32_t tile_size,
                                    const glm::uvec2 border_texel) {
    const uint32_t tile_stride = tile_size + 2u;
    glm::uvec2 copy_texel = tile_origin;
    const bool is_corner = (border_texel.x == 0u || border_texel.x == tile_stride - 1u) &&
                           (border_texel.y == 0u || border_texel.y == tile_stride - 1u);
    const bool is_row = border_texel.x > 0u && border_texel.x < tile_stride - 1u;
    if (is_corner) {
      copy_texel += glm::uvec2(border_texel.x > 0u ? 1u : tile_size, border_texel.y > 0u ? 1u : tile_size);
    } else if (is_row) {
      copy_texel += glm::uvec2((tile_stride - 1u) - border_texel.x,
                               border_texel.y > 0u ? border_texel.y - 1u : border_texel.y + 1u);
    } else {
      copy_texel += glm::uvec2(border_texel.x > 0u ? border_texel.x - 1u : border_texel.x + 1u,
                               (tile_stride - 1u) - border_texel.y);
    }
    return copy_texel;
  };

  const glm::uvec2 tile_origin(30, 50);
  for (const uint32_t tile_size : {1u, 2u, 8u, 16u}) {
    const uint32_t tile_stride = tile_size + 2u;
    for (uint32_t y = 0; y < tile_stride; ++y) {
      for (uint32_t x = 0; x < tile_stride; ++x) {
        const bool is_border = x == 0u || y == 0u || x == tile_stride - 1u || y == tile_stride - 1u;
        if (!is_border) {
          continue;
        }
        const glm::uvec2 copy_texel = border_copy_texel(tile_origin, tile_size, {x, y});
        EXPECT_GE(copy_texel.x, tile_origin.x + 1u);
        EXPECT_GE(copy_texel.y, tile_origin.y + 1u);
        EXPECT_LE(copy_texel.x, tile_origin.x + tile_size);
        EXPECT_LE(copy_texel.y, tile_origin.y + tile_size);
      }
    }
  }
}

TEST(DdgiVolume, DdgiScrollingClearsNewRegionsBeforeTracingAndKeepsCurrentRays) {
  const auto shader_root =
      std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "Internals" / "DefaultResources" / "Shaders";
  const auto ddgi_helper_source = ReadTextFile(shader_root / "Modules" / "EvoEngine" / "DDGI.slang");
  const auto probe_update_source = ReadTextFile(shader_root / "Compute" / "DDGIProbeUpdate.slang");
  const auto probe_scroll_source = ReadTextFile(shader_root / "Compute" / "DDGIProbeScroll.slang");
  const auto source_root = std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK";
  const auto render_layer_source = ReadTextFile(source_root / "src" / "RenderLayer.cpp");
  const auto atlas_prepare_source = ReadTextFile(source_root / "src" / "RenderPasses" / "DdgiAtlasPreparePass.cpp");
  ASSERT_FALSE(ddgi_helper_source.empty());
  ASSERT_FALSE(probe_update_source.empty());
  ASSERT_FALSE(probe_scroll_source.empty());
  ASSERT_FALSE(render_layer_source.empty());
  ASSERT_FALSE(atlas_prepare_source.empty());

  EXPECT_NE(ddgi_helper_source.find("EE_DDGI_PROBE_IS_NEWLY_EXPOSED"), std::string::npos);
  EXPECT_NE(ddgi_helper_source.find("logical_probe_grid[axis] >= probe_counts[axis] - uint(scroll_delta[axis])"),
            std::string::npos);
  EXPECT_NE(ddgi_helper_source.find("logical_probe_grid[axis] < uint(-scroll_delta[axis])"), std::string::npos);
  EXPECT_NE(probe_update_source.find("EE_DDGI_SCROLL_PROBE_GRID(logical_probe_grid, constants.probe_scroll_offset.xyz"),
            std::string::npos);
  EXPECT_NE(probe_update_source.find("blend_previous_probe_state && !newly_exposed_probe"), std::string::npos);
  EXPECT_EQ(probe_update_source.find("EE_DDGI_CLEAR_SCROLLED_PROBE"), std::string::npos);
  EXPECT_NE(probe_scroll_source.find("EE_DDGI_PROBE_STATE[physical_probe_index] = float4(0.0f)"), std::string::npos);
  EXPECT_NE(probe_scroll_source.find("EE_DDGI_CLEAR_IRRADIANCE_TILE"), std::string::npos);
  EXPECT_NE(probe_scroll_source.find("EE_DDGI_CLEAR_VISIBILITY_TILE"), std::string::npos);
  EXPECT_EQ(probe_scroll_source.find("EE_DDGI_CLEAR_VARIABILITY_TILE"), std::string::npos);
  EXPECT_NE(atlas_prepare_source.find("RenderResourceState::TransferDestinationGeneral"), std::string::npos);
  EXPECT_NE(render_layer_source.find("if (clear_ddgi_probe_atlas)"), std::string::npos);
  EXPECT_NE(render_layer_source.find("DdgiVolumeMovementType::Scrolling"), std::string::npos);
  EXPECT_NE(render_layer_source.find("CalculateDdgiEffectiveFirstProbe"), std::string::npos);
  EXPECT_NE(render_layer_source.find("NormalizeDdgiProbeScrollOrigin"), std::string::npos);
  EXPECT_NE(render_layer_source.find("complete_wraps * counts[axis]"), std::string::npos);
  EXPECT_NE(render_layer_source.find("DdgiRuntime::RequiresFullScrollReset(ddgi_ray_source.probe_counts"),
            std::string::npos);
  const auto scroll_pass_position = render_layer_source.find("DdgiProbeScrollPass::CreateDescriptor()");
  const auto ray_pass_position = render_layer_source.find("DdgiProbeTracePass::CreateDescriptor(");
  ASSERT_NE(scroll_pass_position, std::string::npos);
  ASSERT_NE(ray_pass_position, std::string::npos);
  EXPECT_LT(scroll_pass_position, ray_pass_position);
  EXPECT_NE(render_layer_source.find("PackDdgiProbeScrollFlags"), std::string::npos);
  EXPECT_NE(render_layer_source.find("std::floor(value)"), std::string::npos);
  EXPECT_NE(render_layer_source.find("std::ceil(value)"), std::string::npos);
  EXPECT_NE(render_layer_source.find("ddgi_scroll_clear_this_frame"), std::string::npos);
  EXPECT_EQ(ddgi_helper_source.find(".hlsl"), std::string::npos);
  EXPECT_EQ(probe_update_source.find(".hlsl"), std::string::npos);
}

TEST(DdgiVolume, PassStatsAcknowledgeOnlyRecordedDdgiWork) {
  const auto source_root = std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "src";
  const auto ray_source = ReadTextFile(source_root / "RenderPasses" / "DdgiProbeTracePass.cpp");
  const auto update_source = ReadTextFile(source_root / "RenderPasses" / "DdgiProbeUpdatePass.cpp");
  const auto render_layer_source = ReadTextFile(source_root / "RenderLayer.cpp");
  ASSERT_FALSE(ray_source.empty());
  ASSERT_FALSE(update_source.empty());
  ASSERT_FALSE(render_layer_source.empty());

  const auto trace = ray_source.find("parameters.pipeline->Trace");
  const auto trace_ack = ray_source.find("*parameters.recorded_ray_sample_count", trace);
  ASSERT_NE(trace, std::string::npos);
  ASSERT_NE(trace_ack, std::string::npos);
  EXPECT_LT(trace, trace_ack);
  const auto dispatch = update_source.find("pipeline->Dispatch");
  const auto update_ack = update_source.find("*parameters.recorded_probe_update_count", dispatch);
  ASSERT_NE(dispatch, std::string::npos);
  ASSERT_NE(update_ack, std::string::npos);
  EXPECT_LT(dispatch, update_ack);
  EXPECT_NE(render_layer_source.find("lighting_descriptors_bound = BindDdgiAtlasLightingDescriptors"),
            std::string::npos);
}

TEST(DdgiVolume, DdgiProbeUpdateUsesCooperativeVariantsAndExactSynchronization) {
  const auto root = std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR);
  const auto shader =
      ReadTextFile(root / "EvoEngine_SDK/Internals/DefaultResources/Shaders/Compute/DDGIProbeUpdate.slang");
  const auto render_layer = ReadTextFile(root / "EvoEngine_SDK/src/RenderLayer.cpp");
  const auto platform = ReadTextFile(root / "EvoEngine_SDK/src/Platform.cpp");
  const auto pass_root = root / "EvoEngine_SDK/src/RenderPasses";
  const auto update = ReadTextFile(pass_root / "DdgiProbeUpdatePass.cpp");
  const auto ray = ReadTextFile(pass_root / "DdgiProbeTracePass.cpp");
  const auto visualization = ReadTextFile(pass_root / "DdgiProbeVisualizationPass.cpp");
  const auto utilities = ReadTextFile(pass_root / "DdgiPassUtilities.cpp");
  ASSERT_FALSE(shader.empty());
  ASSERT_FALSE(render_layer.empty());
  ASSERT_FALSE(update.empty());
  ASSERT_FALSE(utilities.empty());

  EXPECT_NE(shader.find("EE_DDGI_PROBE_UPDATE_SHARED_RAY_COUNT 256u"), std::string::npos);
  EXPECT_NE(shader.find("groupshared float4 EE_DDGI_SHARED_PROBE_RAY_DATA"), std::string::npos);
  EXPECT_NE(shader.find("groupshared float4 EE_DDGI_SHARED_PROBE_RAY_DIRECTIONS"), std::string::npos);
  EXPECT_NE(shader.find("group_id.x + group_id.y * EE_DDGI_UPDATE_DISPATCH_GROUP_COUNT_X()"), std::string::npos);
  EXPECT_NE(shader.find("GroupMemoryBarrierWithGroupSync();"), std::string::npos);
  EXPECT_NE(shader.find("EE_DDGI_COPY_IRRADIANCE_BORDER_TEXELS_PARALLEL"), std::string::npos);
  EXPECT_NE(shader.find("EE_DDGI_COPY_VISIBILITY_BORDER_TEXELS_PARALLEL"), std::string::npos);
  EXPECT_NE(render_layer.find("EVOENGINE_DDGI_PROBE_UPDATE_VARIANT"), std::string::npos);
  EXPECT_NE(render_layer.find("environment_variant : \"parallel-shared\""), std::string::npos);
  EXPECT_NE(render_layer.find("ddgi_probe_update_irradiance_pipeline_"), std::string::npos);
  EXPECT_NE(render_layer.find("ddgi_probe_update_visibility_pipeline_"), std::string::npos);
  EXPECT_NE(update.find("DdgiProbeUpdatePass::CalculateDispatchSize"), std::string::npos);
  EXPECT_NE(update.find("irradiance_pipeline, descriptor_set, 1u"), std::string::npos);
  EXPECT_NE(update.find("visibility_pipeline, descriptor_set, 2u"), std::string::npos);
  EXPECT_NE(update.find("EVOENGINE_DDGI_PROBE_UPDATE_PATH executed="), std::string::npos);
  EXPECT_EQ(update.find("EverythingBarrier"), std::string::npos);
  for (const auto* file : {"DdgiProbeTracePass.cpp", "DdgiProbeRelocationPass.cpp", "DdgiProbeClassificationPass.cpp",
                           "DdgiProbeScrollPass.cpp"}) {
    EXPECT_EQ(ReadTextFile(pass_root / file).find("EverythingBarrier"), std::string::npos) << file;
  }
  EXPECT_NE(utilities.find("VkBufferMemoryBarrier2"), std::string::npos);
  EXPECT_NE(utilities.find("VkImageMemoryBarrier2"), std::string::npos);
  EXPECT_NE(render_layer.find("AcquireDdgiFrameResources"), std::string::npos);
  EXPECT_NE(render_layer.find("PublishDdgiFrameResources"), std::string::npos);
  EXPECT_NE(ray.find("atlas_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL"), std::string::npos);
  EXPECT_NE(visualization.find("image_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL"), std::string::npos);
  EXPECT_NE(platform.find("VK_VALIDATION_FEATURE_ENABLE_SYNCHRONIZATION_VALIDATION_EXT"), std::string::npos);
  EXPECT_NE(platform.find("EVOENGINE_VULKAN_SYNCHRONIZATION_VALIDATION enabled"), std::string::npos);
}

TEST(DdgiVolume, DdgiShadowRaysIgnoreNonShadowCastingLightVisualizers) {
  const auto shader_root =
      std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "Internals" / "DefaultResources" / "Shaders";
  const auto ddgi_helper_source = ReadTextFile(shader_root / "Modules" / "EvoEngine" / "DDGI.slang");
  const auto raygen_source = ReadTextFile(shader_root / "RayTracing" / "RayGen" / "DDGIProbeTrace.slang");
  const auto closest_hit_source = ReadTextFile(shader_root / "RayTracing" / "ClosestHit" / "DDGIProbeTrace.slang");
  const auto graphics_resources_source = ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) /
                                                      "EvoEngine_SDK" / "src" / "GraphicsResources.cpp");
  const auto demo_scene_source =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_App" / "src" / "DemoScene.cpp");

  EXPECT_NE(ddgi_helper_source.find("public static const uint EE_DDGI_RAY_MASK_GEOMETRY = 0x01u;"), std::string::npos);
  EXPECT_NE(ddgi_helper_source.find("public static const uint EE_DDGI_RAY_MASK_SHADOW = 0x02u;"), std::string::npos);
  EXPECT_NE(raygen_source.find("EE_DDGI_RAY_MASK_GEOMETRY"), std::string::npos);
  EXPECT_NE(closest_hit_source.find("EE_DDGI_RAY_MASK_SHADOW"), std::string::npos);
  EXPECT_NE(graphics_resources_source.find("kDdgiRayMaskGeometry | (render_instance->cast_shadow ? "
                                           "kDdgiRayMaskShadow : 0u)"),
            std::string::npos);
  EXPECT_NE(demo_scene_source.find("point_light_right_renderer->cast_shadow = false;"), std::string::npos);
}

TEST(DdgiVolume, RenderLayerFrameResourceLayoutRespondsToResourceSizingInputs) {
  DdgiSettings settings;
  settings.storage.max_probe_count = 1000;
  settings.storage.atlas_probe_columns = 5;
  settings.storage.irradiance_tile_resolution = 6;
  settings.storage.visibility_tile_resolution = 14;
  settings.runtime.ray_count = 11;
  settings.runtime.emissive_ray_count = 0;

  auto layout = DdgiRuntime::CalculateFrameResourceLayout(settings, 37);

  EXPECT_EQ(layout.probe_count, 37u);
  EXPECT_EQ(layout.irradiance_atlas.columns, 5u);
  EXPECT_EQ(layout.irradiance_atlas.rows, 8u);
  EXPECT_EQ(layout.irradiance_atlas.resolution, glm::uvec2(40, 64));
  EXPECT_EQ(layout.visibility_atlas.resolution, glm::uvec2(80, 128));
  EXPECT_EQ(layout.probe_metadata_byte_size, 37ull * sizeof(glm::vec4) * 3ull);
  EXPECT_EQ(layout.probe_state_byte_size, 37ull * sizeof(glm::vec4));
  EXPECT_EQ(layout.ray_output_byte_size, 37ull * 11ull * sizeof(DdgiProbeRayData));
  EXPECT_EQ(layout.selected_ray_diagnostics_byte_size, 11ull * sizeof(PointCloudSample));

  settings.runtime.ray_count = 23;
  auto resized_layout = DdgiRuntime::CalculateFrameResourceLayout(settings, 37);
  EXPECT_EQ(resized_layout.irradiance_atlas.resolution, layout.irradiance_atlas.resolution);
  EXPECT_EQ(resized_layout.visibility_atlas.resolution, layout.visibility_atlas.resolution);
  EXPECT_EQ(resized_layout.ray_output_byte_size, 37ull * 23ull * sizeof(DdgiProbeRayData));
  EXPECT_EQ(resized_layout.selected_ray_diagnostics_byte_size, 23ull * sizeof(PointCloudSample));

  settings.storage.irradiance_tile_resolution = 10;
  settings.storage.visibility_tile_resolution = 18;
  resized_layout = DdgiRuntime::CalculateFrameResourceLayout(settings, 37);
  EXPECT_EQ(resized_layout.irradiance_atlas.resolution, glm::uvec2(60, 96));
  EXPECT_EQ(resized_layout.visibility_atlas.resolution, glm::uvec2(100, 160));
  EXPECT_EQ(resized_layout.ray_output_byte_size, 37ull * 23ull * sizeof(DdgiProbeRayData));
  EXPECT_EQ(resized_layout.selected_ray_diagnostics_byte_size, 23ull * sizeof(PointCloudSample));
}

TEST(DdgiVolume, RenderLayerVolumeBlendWeightMatchesRtxgiVolumeCoverage) {
  const auto probe_counts = glm::ivec3(5, 5, 5);
  const auto probe_step_lengths = glm::vec3(2.0f);

  EXPECT_FLOAT_EQ(DdgiRuntime::CalculateVolumeBlendWeight({2.0f, 2.0f, 2.0f}, probe_counts, probe_step_lengths), 1.0f);
  EXPECT_FLOAT_EQ(DdgiRuntime::CalculateVolumeBlendWeight({0.0f, 2.0f, 2.0f}, probe_counts, probe_step_lengths), 1.0f);
  EXPECT_FLOAT_EQ(DdgiRuntime::CalculateVolumeBlendWeight({4.0f, 2.0f, 2.0f}, probe_counts, probe_step_lengths), 1.0f);
  EXPECT_NEAR(DdgiRuntime::CalculateVolumeBlendWeight({-0.5f, 2.0f, 2.0f}, probe_counts, probe_step_lengths), 0.5f,
              0.0001f);
  EXPECT_FLOAT_EQ(DdgiRuntime::CalculateVolumeBlendWeight({-1.0f, 2.0f, 2.0f}, probe_counts, probe_step_lengths), 0.0f);
  EXPECT_FLOAT_EQ(DdgiRuntime::CalculateVolumeBlendWeight({-0.1f, 2.0f, 2.0f}, probe_counts, probe_step_lengths), 0.9f);
}

TEST(DdgiVolume, MultiVolumeBudgetAcceptsExactLimitsAndRejectsLimitPlusOneAtomically) {
  std::vector<DdgiVolumeRuntimeInfo> infos(8);
  for (size_t i = 0; i < infos.size(); ++i) {
    infos[i].stable_entity_id = i + 1u;
    infos[i].probe_count = 1024u;
  }

  const auto exact = DdgiRuntime::ValidateVolumeSet(infos, 8192u);
  EXPECT_TRUE(exact.valid);
  EXPECT_EQ(exact.aggregate_probe_count, 8192u);

  auto zero_id = infos;
  zero_id.front().stable_entity_id = 0u;
  const auto zero_id_result = DdgiRuntime::ValidateVolumeSet(zero_id, 8192u);
  EXPECT_FALSE(zero_id_result.valid);
  EXPECT_NE(zero_id_result.error.find("must be nonzero"), std::string::npos);

  auto duplicate_id = infos;
  duplicate_id.back().stable_entity_id = duplicate_id.front().stable_entity_id;
  const auto duplicate_id_result = DdgiRuntime::ValidateVolumeSet(duplicate_id, 8192u);
  EXPECT_FALSE(duplicate_id_result.valid);
  EXPECT_NE(duplicate_id_result.error.find("Duplicate DDGI volume stable ID"), std::string::npos);

  auto too_many_volumes = infos;
  too_many_volumes.push_back({});
  too_many_volumes.back().stable_entity_id = 9u;
  too_many_volumes.back().probe_count = 1u;
  const auto volume_overflow = DdgiRuntime::ValidateVolumeSet(too_many_volumes, 8192u);
  EXPECT_FALSE(volume_overflow.valid);
  EXPECT_NE(volume_overflow.error.find("maximum is 8"), std::string::npos);

  auto too_many_probes = infos;
  ++too_many_probes.back().probe_count;
  const auto probe_overflow = DdgiRuntime::ValidateVolumeSet(too_many_probes, 8192u);
  EXPECT_FALSE(probe_overflow.valid);
  EXPECT_EQ(probe_overflow.aggregate_probe_count, 8193u);
  EXPECT_NE(probe_overflow.error.find("8193 resident probes"), std::string::npos);

  const auto configured_overflow = DdgiRuntime::ValidateVolumeSet(infos, 4096u);
  EXPECT_FALSE(configured_overflow.valid);
  EXPECT_NE(configured_overflow.error.find("maximum is 4096"), std::string::npos);
  EXPECT_EQ(infos.size(), 8u);
  EXPECT_EQ(infos.back().probe_count, 1024u);
}

TEST(DdgiVolume, MultiVolumeCandidateAccountingIncludesPausedAndExcludesDisabledVolumes) {
  Application app;
  ApplicationContextScope scope(app);
  app.Initialize(EmptyProjectSettings());
  const auto scene = AssetManager::CreateTemporaryAsset<Scene>();
  const auto lighting = AssetManager::CreateTemporaryAsset<EnvironmentalLighting>();
  ASSERT_TRUE(scene);
  ASSERT_TRUE(lighting);
  app.Attach(scene);
  scene->environmental_lighting = lighting;
  lighting->indirect_gi_provider = IndirectGiProvider::AuthoredDdgi;

  EnvironmentalLighting::DdgiVolume first;
  first.stable_id = 11u;
  first.probe_counts = {4, 4, 4};
  EnvironmentalLighting::DdgiVolume second;
  second.stable_id = 22u;
  second.probe_counts = {5, 5, 5};
  auto ddgi_pack = lighting->GetOrCreateDdgiVolumePack();
  ddgi_pack->volumes = {first, second};
  auto infos = CollectDdgiVolumeRuntimeInfos(ResolveEnvironmentalLighting(scene));
  EXPECT_EQ(infos.size(), 2u);

  ddgi_pack->volumes[1].enabled = false;
  infos = CollectDdgiVolumeRuntimeInfos(ResolveEnvironmentalLighting(scene));
  ASSERT_EQ(infos.size(), 1u);
  EXPECT_EQ(infos.front().stable_entity_id, first.stable_id);

  ddgi_pack->volumes[1].enabled = true;
  ddgi_pack->volumes[1].probe_counts = {0, 5, 5};
  infos = CollectDdgiVolumeRuntimeInfos(ResolveEnvironmentalLighting(scene));
  EXPECT_EQ(infos.size(), 1u);
}

TEST(DdgiVolume, MultiVolumeOrderingUsesPriorityDensityThenStableEntityId) {
  DdgiVolumeRuntimeInfo low_priority;
  low_priority.stable_entity_id = 1u;
  low_priority.artist_priority = 0;
  low_priority.probe_density = 100.0f;
  DdgiVolumeRuntimeInfo high_priority = low_priority;
  high_priority.stable_entity_id = 3u;
  high_priority.artist_priority = 1;
  high_priority.probe_density = 0.01f;
  DdgiVolumeRuntimeInfo density_tie_break = low_priority;
  density_tie_break.stable_entity_id = 2u;
  density_tie_break.probe_density = 200.0f;

  std::vector infos{low_priority, high_priority, density_tie_break};
  DdgiRuntime::SortVolumeRuntimeInfos(infos);
  ASSERT_EQ(infos.size(), 3u);
  EXPECT_EQ(infos[0].stable_entity_id, 3u);
  EXPECT_EQ(infos[1].stable_entity_id, 2u);
  EXPECT_EQ(infos[2].stable_entity_id, 1u);
  EXPECT_EQ(infos[0].sorted_index, 0u);
  EXPECT_EQ(infos[2].sorted_index, 2u);

  density_tie_break.probe_density = low_priority.probe_density;
  infos = {density_tie_break, low_priority};
  DdgiRuntime::SortVolumeRuntimeInfos(infos);
  EXPECT_EQ(infos[0].stable_entity_id, 1u);
  EXPECT_EQ(infos[1].stable_entity_id, 2u);

  EXPECT_FLOAT_EQ(DdgiRuntime::CalculateProbeDensity({2.0f, 0.0f, 0.0f}, {0.0f, 3.0f, 0.0f}, {0.0f, 0.0f, 4.0f}),
                  1.0f / 24.0f);
  EXPECT_FLOAT_EQ(DdgiRuntime::CalculateProbeDensity({}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}), 0.0f);
}

TEST(DdgiVolume, MultiVolumeSelectionIsDeterministicAndUsesAtMostOneBoundarySecondary) {
  const auto make_info = [](const uint64_t stable_id, const int priority, const glm::vec3 first_probe) {
    DdgiVolumeRuntimeInfo info;
    info.stable_entity_id = stable_id;
    info.artist_priority = priority;
    info.probe_density = 1.0f;
    info.probe_counts = {5, 5, 5};
    info.probe_count = 125u;
    info.first_probe = first_probe;
    info.probe_step_x = {1.0f, 0.0f, 0.0f};
    info.probe_step_y = {0.0f, 1.0f, 0.0f};
    info.probe_step_z = {0.0f, 0.0f, 1.0f};
    return info;
  };
  const auto primary = make_info(10u, 2, {0.0f, 0.0f, 0.0f});
  const auto secondary = make_info(20u, 1, {0.0f, 0.0f, 0.0f});
  const auto third = make_info(30u, 0, {0.0f, 0.0f, 0.0f});

  const auto deep_overlap = DdgiRuntime::SelectVolumes({third, secondary, primary}, {2.0f, 2.0f, 2.0f});
  EXPECT_TRUE(deep_overlap.valid);
  EXPECT_EQ(deep_overlap.primary_entity_id, 10u);
  EXPECT_EQ(deep_overlap.secondary_entity_id, 0u);
  EXPECT_FLOAT_EQ(deep_overlap.primary_weight, 1.0f);
  EXPECT_FLOAT_EQ(deep_overlap.ibl_weight, 0.0f);

  const auto boundary = DdgiRuntime::SelectVolumes({third, primary, secondary}, {0.5f, 2.0f, 2.0f});
  EXPECT_EQ(boundary.primary_entity_id, 10u);
  EXPECT_EQ(boundary.secondary_entity_id, 20u);
  EXPECT_FLOAT_EQ(boundary.primary_weight + boundary.secondary_weight, 1.0f);
  EXPECT_NEAR(boundary.primary_weight, 2.0f / 3.0f, 0.0001f);
  EXPECT_NEAR(boundary.secondary_weight, 1.0f / 3.0f, 0.0001f);
  EXPECT_FLOAT_EQ(boundary.ibl_weight, 0.0f);

  const auto near_boundary = DdgiRuntime::SelectVolumes({secondary, primary}, {0.99f, 2.0f, 2.0f});
  EXPECT_EQ(near_boundary.secondary_entity_id, 20u);
  EXPECT_NEAR(near_boundary.secondary_weight, 0.01f / 1.01f, 0.0001f);
  const auto exact_boundary = DdgiRuntime::SelectVolumes({secondary, primary}, {1.0f, 2.0f, 2.0f});
  EXPECT_EQ(exact_boundary.secondary_entity_id, 0u);
  EXPECT_LT(std::abs(exact_boundary.primary_weight - near_boundary.primary_weight), 0.011f);
  const auto beyond_boundary = DdgiRuntime::SelectVolumes({secondary, primary}, {1.01f, 2.0f, 2.0f});
  EXPECT_EQ(beyond_boundary.secondary_entity_id, 0u);

  auto reversed_infos = std::vector{third, secondary, primary};
  std::reverse(reversed_infos.begin(), reversed_infos.end());
  const auto reversed = DdgiRuntime::SelectVolumes(reversed_infos, {0.5f, 2.0f, 2.0f});
  EXPECT_EQ(reversed.primary_entity_id, boundary.primary_entity_id);
  EXPECT_EQ(reversed.secondary_entity_id, boundary.secondary_entity_id);
  EXPECT_FLOAT_EQ(reversed.primary_weight, boundary.primary_weight);
  EXPECT_FLOAT_EQ(reversed.secondary_weight, boundary.secondary_weight);

  const auto faded = DdgiRuntime::SelectVolumes({primary}, {-0.5f, 2.0f, 2.0f});
  EXPECT_EQ(faded.primary_entity_id, 10u);
  EXPECT_EQ(faded.secondary_entity_id, 0u);
  EXPECT_FLOAT_EQ(faded.primary_weight, 0.5f);
  EXPECT_FLOAT_EQ(faded.ibl_weight, 0.5f);

  const auto disjoint = make_info(40u, 5, {10.0f, 0.0f, 0.0f});
  const auto selected_disjoint = DdgiRuntime::SelectVolumes({primary, disjoint}, {12.0f, 2.0f, 2.0f});
  EXPECT_EQ(selected_disjoint.primary_entity_id, 40u);
  EXPECT_EQ(selected_disjoint.secondary_entity_id, 0u);
  EXPECT_FALSE(DdgiRuntime::SelectVolumes({primary, disjoint}, {7.0f, 2.0f, 2.0f}).valid);

  const auto nested = make_info(50u, 3, {1.0f, 1.0f, 1.0f});
  const auto selected_nested = DdgiRuntime::SelectVolumes({primary, nested}, {3.0f, 3.0f, 3.0f});
  EXPECT_EQ(selected_nested.primary_entity_id, 50u);
  EXPECT_EQ(selected_nested.secondary_entity_id, 0u);
  EXPECT_FLOAT_EQ(selected_nested.primary_weight, 1.0f);

  const auto outer = make_info(60u, 0, {0.0f, 0.0f, 0.0f});
  auto inner = make_info(70u, 1, {1.0f, 1.0f, 1.0f});
  inner.probe_counts = {3, 3, 3};
  inner.probe_count = 27u;
  const auto just_inside = DdgiRuntime::SelectVolumes({outer, inner}, {1.001f, 2.0f, 2.0f});
  const auto exact_crossing = DdgiRuntime::SelectVolumes({outer, inner}, {1.0f, 2.0f, 2.0f});
  const auto just_outside = DdgiRuntime::SelectVolumes({outer, inner}, {0.999f, 2.0f, 2.0f});
  EXPECT_EQ(just_inside.primary_entity_id, 70u);
  EXPECT_EQ(just_inside.secondary_entity_id, 60u);
  EXPECT_NEAR(just_inside.primary_weight, 1.0f / 1.999f, 0.0001f);
  EXPECT_NEAR(exact_crossing.primary_weight, 0.5f, 0.0001f);
  EXPECT_EQ(just_outside.primary_entity_id, 70u);
  EXPECT_EQ(just_outside.secondary_entity_id, 60u);
  EXPECT_NEAR(just_outside.primary_weight, 0.999f / 1.999f, 0.0001f);
  EXPECT_LT(std::abs(just_inside.primary_weight - just_outside.primary_weight), 0.001f);

  auto deep_outer = make_info(75u, 0, {0.0f, 0.0f, 0.0f});
  deep_outer.probe_counts = {11, 11, 11};
  deep_outer.probe_count = 1331u;
  auto approaching_inner = make_info(76u, 1, {4.0f, 4.0f, 4.0f});
  approaching_inner.probe_counts = {3, 3, 3};
  approaching_inner.probe_count = 27u;
  const auto exterior_boundary = DdgiRuntime::SelectVolumes({deep_outer, approaching_inner}, {3.5f, 5.0f, 5.0f});
  EXPECT_EQ(exterior_boundary.primary_entity_id, 76u);
  EXPECT_EQ(exterior_boundary.secondary_entity_id, 75u);
  EXPECT_NEAR(exterior_boundary.primary_weight, 1.0f / 3.0f, 0.0001f);
  EXPECT_NEAR(exterior_boundary.secondary_weight, 2.0f / 3.0f, 0.0001f);

  auto warming_inner = inner;
  warming_inner.contributes_lighting = false;
  const auto warming_fallback = DdgiRuntime::SelectVolumes({warming_inner, outer}, {2.0f, 2.0f, 2.0f});
  EXPECT_EQ(warming_fallback.primary_entity_id, 60u);
  EXPECT_EQ(warming_fallback.secondary_entity_id, 0u);
  EXPECT_FLOAT_EQ(warming_fallback.primary_weight, 1.0f);

  const auto fade_a = make_info(80u, 1, {0.0f, 0.0f, 0.0f});
  const auto fade_b = make_info(90u, 0, {5.5f, 0.0f, 0.0f});
  const auto before_fade_overlap = DdgiRuntime::SelectVolumes({fade_b, fade_a}, {4.5f, 2.0f, 2.0f});
  const auto after_fade_overlap = DdgiRuntime::SelectVolumes({fade_b, fade_a}, {4.5001f, 2.0f, 2.0f});
  EXPECT_FLOAT_EQ(before_fade_overlap.primary_weight, 0.5f);
  EXPECT_FLOAT_EQ(before_fade_overlap.ibl_weight, 0.5f);
  EXPECT_EQ(after_fade_overlap.secondary_entity_id, 90u);
  EXPECT_NEAR(after_fade_overlap.primary_weight, 0.4999f, 0.0002f);
  EXPECT_NEAR(after_fade_overlap.secondary_weight, 0.0001f, 0.0002f);
  EXPECT_NEAR(after_fade_overlap.ibl_weight, 0.5f, 0.0002f);
  EXPECT_LT(std::abs(before_fade_overlap.primary_weight - after_fade_overlap.primary_weight), 0.0002f);
}

TEST(DdgiVolume, MultiVolumeGpuContractOwnsEightSlotsAndSumsPerFrameTiming) {
  const auto root = std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR);
  const auto render_layer_header = ReadTextFile(root / "EvoEngine_SDK" / "include" / "Layers" / "RenderLayer.hpp");
  const auto ddgi_runtime_header =
      ReadTextFile(root / "EvoEngine_SDK" / "include" / "Rendering" / "PBR" / "DdgiRuntime.hpp");
  const auto render_storage_header =
      ReadTextFile(root / "EvoEngine_SDK" / "include" / "Rendering" / "RenderInstances" / "RenderInstanceStorage.hpp");
  const auto render_layer_source = ReadTextFile(root / "EvoEngine_SDK" / "src" / "RenderLayer.cpp");
  const auto platform_header =
      ReadTextFile(root / "EvoEngine_SDK" / "include" / "Rendering" / "Platform" / "Platform.hpp");
  const auto platform_source = ReadTextFile(root / "EvoEngine_SDK" / "src" / "Platform.cpp");
  const auto shader_root = root / "EvoEngine_SDK" / "Internals" / "DefaultResources" / "Shaders";
  const auto lighting_source = ReadTextFile(shader_root / "Modules" / "EvoEngine" / "Lighting.slang");
  const auto gather_source = ReadTextFile(shader_root / "Modules" / "EvoEngine" / "DDGIGatherMulti.slang");
  const auto closest_hit_source = ReadTextFile(shader_root / "RayTracing" / "ClosestHit" / "DDGIProbeTrace.slang");
  const auto inspection_source = ReadTextFile(root / "EvoEngine_SDK" / "src" / "Editor" / "SDKInspectionAdapters.cpp");

  EXPECT_NE(render_storage_header.find("static constexpr uint32_t kDdgiMaxVolumeCount = 8"), std::string::npos);
  EXPECT_NE(render_storage_header.find("static_assert(sizeof(DdgiVolumeInfoBlock) == 160)"), std::string::npos);
  EXPECT_NE(render_storage_header.find("static_assert(offsetof(RenderInfoBlock, indirect_lighting_intensity) == 28)"),
            std::string::npos);
  EXPECT_NE(render_storage_header.find("static_assert(offsetof(RenderInfoBlock, ddgi_volumes) == 128)"),
            std::string::npos);
  EXPECT_NE(render_storage_header.find("static_assert(offsetof(RenderInfoBlock, reflection_probe_header) == 1408)"),
            std::string::npos);
  EXPECT_NE(render_storage_header.find("static_assert(offsetof(RenderInfoBlock, reflection_probes) == 1424)"),
            std::string::npos);
  EXPECT_NE(render_storage_header.find("static_assert(sizeof(RenderInfoBlock) == 6032)"), std::string::npos);
  EXPECT_EQ(render_storage_header.find("glm::vec4 ddgi_first_probe"), std::string::npos);
  EXPECT_NE(render_layer_header.find("std::unordered_map<uint64_t, std::unique_ptr<DdgiVolumeRuntimeState>>"),
            std::string::npos);
  EXPECT_NE(render_layer_source.find("for (uint32_t slot = 0; slot < RenderInstanceStorage::kDdgiMaxVolumeCount"),
            std::string::npos);
  EXPECT_NE(render_layer_source.find("execute_ddgi_runtime(*runtime->second"), std::string::npos);
  EXPECT_NE(render_layer_source.find("current_frame_transient_resources.emplace_back()"), std::string::npos);
  EXPECT_EQ(render_layer_source.find("include_external_passes"), std::string::npos);
  EXPECT_EQ(render_layer_source.find("execute_ddgi_runtime(empty_runtime"), std::string::npos);
  EXPECT_NE(render_layer_source.find("for (size_t i = ddgi_ordered_volume_ids_.size(); i-- > 0u;)"), std::string::npos);
  EXPECT_NE(render_layer_source.find("ExecuteSceneFramePasses(scene);"), std::string::npos);
  EXPECT_NE(render_layer_source.find("gi_complete.dependencies.push_back(pass.name)"), std::string::npos);
  EXPECT_NE(render_layer_source.find("descriptor.dependencies.emplace_back(RenderPassNames::scene_gi_complete)"),
            std::string::npos);
  EXPECT_EQ(render_layer_source.find("Required DDGI Volume Removal Fence Wait"), std::string::npos);
  EXPECT_NE(render_layer_source.find("ResetDdgiRuntimeFrameState(*runtime);"), std::string::npos);
  EXPECT_NE(render_layer_source.find("if (!ddgi_volume_runtime_states_.empty())"), std::string::npos);
  EXPECT_EQ(render_layer_source.find("ddgi_debug_volume_id_"), std::string::npos);
  EXPECT_NE(
      render_layer_source.find(
          "const bool prepare_debug_data = ddgi_session_state_.selected_volume_id == runtime_state.stable_entity_id"),
      std::string::npos);
  EXPECT_NE(render_layer_source.find("reject_volume_set(\"volume \" + std::to_string(info.stable_entity_id)"),
            std::string::npos);
  const auto preflight = render_layer_source.find("std::vector<DdgiFrameResourceLayout> preflight_layouts;");
  const auto reconciliation = render_layer_source.find("std::vector<uint64_t> current_ids;");
  ASSERT_NE(preflight, std::string::npos);
  ASSERT_NE(reconciliation, std::string::npos);
  EXPECT_LT(preflight, reconciliation);
  const auto prepare_worker = ExtractBetween(render_layer_source, "void RenderLayer::PrepareDdgiVolumeFrameState",
                                             "void RenderLayer::RenderSceneToCameraImmediately");
  EXPECT_EQ(prepare_worker.find("GetSelectedPhysicalDevice"), std::string::npos);
  EXPECT_EQ(prepare_worker.find("CalculateDdgiFrameResourceLayout"), std::string::npos);
  const auto prepare_set = ExtractBetween(render_layer_source, "void RenderLayer::PrepareDdgiFrameState",
                                          "void RenderLayer::PrepareDdgiVolumeFrameState");
  const auto final_volume_header = prepare_set.rfind("render_info_block.ddgi_volume_header");
  ASSERT_NE(final_volume_header, std::string::npos);
  EXPECT_EQ(prepare_set.find("render_info_block.indirect_lighting_intensity"), std::string::npos);
  EXPECT_NE(render_layer_source.find("const auto resolved_lighting = ResolveEnvironmentalLighting(scene);"),
            std::string::npos);
  EXPECT_NE(render_layer_source.find("for (const auto& volume : resolved_lighting.ddgi_volumes)"), std::string::npos);
  EXPECT_EQ(render_layer_source.find("ddgi_volume_runtime_states_.find(rejected.stable_entity_id)"), std::string::npos);
  EXPECT_EQ(render_layer_source.find("The new DDGI volume was disabled."), std::string::npos);
  EXPECT_EQ(render_layer_source.find("ddgi_fallback_probe_state_buffer_->GetSize() <"), std::string::npos);
  EXPECT_EQ(render_layer_source.find("GetDebugDdgiVolumeRuntimeState"), std::string::npos);
  EXPECT_NE(gather_source.find("EE_DDGI_IRRADIANCE_ATLASES[EE_DDGI_MAX_VOLUME_COUNT]"), std::string::npos);
  EXPECT_NE(gather_source.find("EE_DDGI_PROBE_STATE_BLOCKS[EE_DDGI_MAX_VOLUME_COUNT]"), std::string::npos);
  EXPECT_NE(gather_source.find("EE_DDGI_PROBE_STATE_BLOCKS[volume_index][probe_index]"), std::string::npos);
  EXPECT_NE(gather_source.find("EE_DDGI_VOLUME_BOUNDARY_WEIGHT"), std::string::npos);
  EXPECT_EQ(gather_source.find("preceding_influence_index"), std::string::npos);
  EXPECT_NE(gather_source.find("if (coverage > 0.0f)"), std::string::npos);
  EXPECT_NE(gather_source.find("volume.lighting_parameters.x > 0.0f"), std::string::npos);
  EXPECT_NE(gather_source.find("weighted_secondary_coverage"), std::string::npos);
  EXPECT_NE(gather_source.find("result.coverage = clamp(coverage_sum, 0.0f, 1.0f)"), std::string::npos);
  EXPECT_NE(
      gather_source.find("const float primary_effective_weight = primary_coverage / coverage_sum * primary.confidence"),
      std::string::npos);
  EXPECT_EQ(gather_source.find("nonuniformEXT"), std::string::npos);
  EXPECT_NE(closest_hit_source.find("import EvoEngine.DDGIGatherSingle;"), std::string::npos);
  EXPECT_NE(closest_hit_source.find("EE_DDGI_PROBE_RAY_CONSTANTS.selected_probe_volume_flags_environment.y"),
            std::string::npos);
  EXPECT_NE(platform_header.find("kGpuTimestampQueriesPerFrame = 256"), std::string::npos);
  EXPECT_NE(inspection_source.find("session.selected_probe_grid + selected->probe_scroll_offset"), std::string::npos);
  EXPECT_NE(inspection_source.find("const auto wrapped ="), std::string::npos);
  EXPECT_NE(ddgi_runtime_header.find("bool emissive_mesh_sampling_enabled = false;"), std::string::npos);
  EXPECT_NE(
      render_layer_source.find("volume_stats.emissive_mesh_sampling_enabled = state.emissive_mesh_sampling_enabled"),
      std::string::npos);
  EXPECT_NE(render_layer_source.find("aggregate.emissive_triangle_count ="), std::string::npos);
  EXPECT_NE(
      render_layer_source.find("glm::max(aggregate.emissive_triangle_count, volume_stats.emissive_triangle_count)"),
      std::string::npos);
  EXPECT_NE(render_layer_source.find("aggregate.emissive_sampling_enabled_volume_count +="), std::string::npos);
  EXPECT_NE(render_layer_source.find("aggregate.emissive_sampling_candidate_ray_count +="), std::string::npos);
  EXPECT_NE(render_layer_source.find("render_info_block.emissive_triangle_parameters.x"), std::string::npos);
  EXPECT_NE(render_layer_source.find("DdgiRuntime::CalculateEmissiveSamplingCandidateRayCount("), std::string::npos);
  EXPECT_NE(inspection_source.find("Sampling enabled volumes / candidate rays"), std::string::npos);
}

TEST(DdgiVolume, RenderLayerIsolatesFirstWarmupFrameFromDdgiHistory) {
  const auto render_layer_source =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "src" / "RenderLayer.cpp");
  const auto ray_hit_source =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "Internals" /
                   "DefaultResources" / "Shaders" / "RayTracing" / "ClosestHit" / "DDGIProbeTrace.slang");
  ASSERT_FALSE(render_layer_source.empty());
  ASSERT_FALSE(ray_hit_source.empty());

  EXPECT_NE(render_layer_source.find("runtime_state.frame_probe_warmup_active &&"), std::string::npos);
  EXPECT_NE(render_layer_source.find("runtime_state.frame_probe_warmup_frame_index == 0u"), std::string::npos);
  EXPECT_EQ(render_layer_source.find("probe_warmup_preserves_history"), std::string::npos);
  EXPECT_NE(render_layer_source.find("skip_recursive_ddgi ? 1.0f : 0.0f"), std::string::npos);
  EXPECT_NE(render_layer_source.find("skip_inactive_probe_trace, ddgi_first_warmup_frame"), std::string::npos);
  EXPECT_NE(
      ray_hit_source.find("const bool skip_recursive_ddgi = EE_DDGI_PROBE_RAY_CONSTANTS.trace_parameters.w > 0.5f;"),
      std::string::npos);
  const auto recursive_guard = ray_hit_source.find("if (!skip_recursive_ddgi)");
  const auto recursive_gather = ray_hit_source.find("EE_DDGI_GATHER_IRRADIANCE", recursive_guard);
  ASSERT_NE(recursive_guard, std::string::npos);
  ASSERT_NE(recursive_gather, std::string::npos);
  EXPECT_LT(recursive_guard, recursive_gather);
  EXPECT_EQ(ray_hit_source.find("skip_recursive_ddgi ? float3(0.0f)"), std::string::npos);
}

TEST(DdgiVolume, RenderLayerFormatsDdgiUpdateReasonsForDebugging) {
  EXPECT_EQ(DdgiUpdateReasonSource, 1u << 0u);
  EXPECT_EQ(DdgiUpdateReasonManualReset, 1u << 1u);
  EXPECT_EQ(DdgiUpdateReasonSteadyState, 1u << 2u);
  EXPECT_EQ(DdgiUpdateReasonWarmup, 1u << 4u);
  EXPECT_EQ(DdgiUpdateReasonSceneChange, 1u << 5u);
  EXPECT_EQ(DdgiRuntime::FormatUpdateReasons(DdgiUpdateReasonNone), "None");
  EXPECT_EQ(DdgiRuntime::FormatUpdateReasons(DdgiUpdateReasonSource | DdgiUpdateReasonManualReset |
                                             DdgiUpdateReasonSteadyState | DdgiUpdateReasonWarmup |
                                             DdgiUpdateReasonSceneChange),
            "DDGI source, Manual reset, Steady state, Warm up, Scene change");
}
