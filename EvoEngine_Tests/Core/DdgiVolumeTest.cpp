#include "EvoEngine_SDK_PCH.hpp"

#include "Application.hpp"
#include "ApplicationContext.hpp"
#include "ApplicationInitializationSettings.hpp"
#include "AssetManager.hpp"
#include "DdgiProbeRayData.hpp"
#include "DdgiRuntime.hpp"
#include "EnvironmentalLighting.hpp"
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
#include <string>
#include <vector>

#include <gtest/gtest.h>

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
}  // namespace

TEST(DdgiVolume, RuntimeHelperContractsPreserveLayoutsAndSelection) {
  EXPECT_TRUE(DdgiSettings{}.runtime.enable_emissive_mesh_sampling);
  EXPECT_EQ(sizeof(DdgiProbeRayData), 16u);
  EXPECT_EQ(alignof(DdgiProbeRayData), 16u);
  EXPECT_EQ(offsetof(DdgiProbeRayData, radiance_and_signed_distance), 0u);
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
  EXPECT_EQ(sizeof(DdgiProbeVisualizationPushConstant), 32u);
  EXPECT_EQ(offsetof(DdgiProbeVisualizationPushConstant, radius_intensity_alpha_selected_scale), 16u);
  EXPECT_EQ(sizeof(DdgiProbeRayVisualizationPushConstant), 16u);
  EXPECT_EQ(offsetof(DdgiProbeRayVisualizationPushConstant, miss_distance_alpha), 8u);
  EXPECT_FLOAT_EQ(kDdgiProbeRayMissDistance, 1e27f);
  EXPECT_FLOAT_EQ(kDdgiProbeRayInactiveDistance, -1e27f);
  EXPECT_FLOAT_EQ(kDdgiProbeRayBackfaceDistanceScale, -0.2f);

  EXPECT_EQ(RenderLayer::GetDdgiFixedRayCount(1u, true), 0u);
  EXPECT_EQ(RenderLayer::GetDdgiFixedRayCount(2u, true), 1u);
  EXPECT_EQ(RenderLayer::GetDdgiFixedRayCount(32u, true), 31u);
  EXPECT_EQ(RenderLayer::GetDdgiFixedRayCount(33u, true), 32u);
  EXPECT_EQ(RenderLayer::GetDdgiFixedRayCount(128u, true), 32u);
  EXPECT_EQ(RenderLayer::GetDdgiFixedRayCount(128u, false), 0u);

  using RuntimeVariant = DdgiProbeUpdateVariant;

  EXPECT_EQ(RenderLayer::GetDdgiProbeCount({4, 5, 6}), DdgiRuntime::GetProbeCount({4, 5, 6}));
  EXPECT_EQ(RenderLayer::GetDdgiFixedRayCount(128u, true), DdgiRuntime::GetFixedRayCount(128u, true));
  EXPECT_EQ(RenderLayer::ParseDdgiProbeUpdateVariant("parallel-shared"),
            DdgiRuntime::ParseProbeUpdateVariant("parallel-shared"));
  EXPECT_EQ(RenderLayer::DdgiUpdateReasonSceneInput, DdgiUpdateReasonSceneInput);
  EXPECT_EQ(RenderLayer::kDdgiMaxVolumeCount, DdgiRuntime::kMaxVolumeCount);

  DdgiSettings settings{};
  settings.runtime.enabled = true;
  settings.debug.enabled = true;
  settings.storage.max_probe_count = 8192;
  settings.storage.atlas_probe_columns = 16;

  const auto render_layer_layout = RenderLayer::CalculateDdgiFrameResourceLayout(settings, 256u, 4096u);
  const auto runtime_layout = DdgiRuntime::CalculateFrameResourceLayout(settings, 256u, 4096u);
  EXPECT_EQ(render_layer_layout.valid, runtime_layout.valid);
  EXPECT_EQ(render_layer_layout.probe_count, runtime_layout.probe_count);
  EXPECT_EQ(render_layer_layout.ray_output_byte_size, runtime_layout.ray_output_byte_size);
  EXPECT_EQ(render_layer_layout.irradiance_atlas.resolution, runtime_layout.irradiance_atlas.resolution);
  EXPECT_EQ(render_layer_layout.visibility_atlas.resolution, runtime_layout.visibility_atlas.resolution);

  DdgiProbeUpdateDeviceLimits runtime_limits{64u, 64u, 65535u, 65535u, DdgiRuntime::kProbeUpdateSharedMemoryBytes};
  EXPECT_EQ(
      RenderLayer::ResolveDdgiProbeUpdateVariant(RuntimeVariant::ParallelShared, runtime_limits, 256u, true, true),
      DdgiRuntime::ResolveProbeUpdateVariant(RuntimeVariant::ParallelShared, runtime_limits, 256u, true, true));

  std::vector<DdgiVolumeRuntimeInfo> infos(2);
  infos[0].stable_entity_id = 20u;
  infos[0].artist_priority = 1;
  infos[0].probe_density = 1.0f;
  infos[1].stable_entity_id = 10u;
  infos[1].artist_priority = 2;
  infos[1].probe_density = 0.5f;
  auto render_layer_infos = infos;
  DdgiRuntime::SortVolumeRuntimeInfos(infos);
  RenderLayer::SortDdgiVolumeRuntimeInfos(render_layer_infos);
  ASSERT_EQ(render_layer_infos.size(), infos.size());
  EXPECT_EQ(render_layer_infos[0].stable_entity_id, infos[0].stable_entity_id);
  EXPECT_EQ(render_layer_infos[1].stable_entity_id, infos[1].stable_entity_id);

  using RenderVariant = RenderLayer::DdgiProbeUpdateVariant;
  RenderLayer::DdgiProbeUpdateDeviceLimits limits{64u, 64u, 65535u, 65535u,
                                                  RenderLayer::kDdgiProbeUpdateSharedMemoryBytes};

  EXPECT_EQ(RenderLayer::ParseDdgiProbeUpdateVariant("serial"), RenderVariant::Serial);
  EXPECT_EQ(RenderLayer::ParseDdgiProbeUpdateVariant("parallel-direct"), RenderVariant::ParallelDirect);
  EXPECT_EQ(RenderLayer::ParseDdgiProbeUpdateVariant("parallel-shared"), RenderVariant::ParallelShared);
  EXPECT_EQ(RenderLayer::ParseDdgiProbeUpdateVariant("invalid"), RenderVariant::Serial);
  EXPECT_EQ(RenderLayer::ResolveDdgiProbeUpdateVariant(RenderVariant::ParallelDirect, limits, 8192u, true, true),
            RenderVariant::ParallelDirect);
  EXPECT_EQ(RenderLayer::ResolveDdgiProbeUpdateVariant(RenderVariant::ParallelShared, limits, 8192u, true, true),
            RenderVariant::ParallelShared);
  EXPECT_EQ(RenderLayer::ResolveDdgiProbeUpdateVariant(RenderVariant::Serial, {}, 0u, false, false),
            RenderVariant::Serial);
  EXPECT_EQ(RenderLayer::ResolveDdgiProbeUpdateVariant(RenderVariant::ParallelDirect, limits, 0u, true, true),
            RenderVariant::Serial);

  limits.max_shared_memory_bytes = 0u;
  EXPECT_EQ(RenderLayer::ResolveDdgiProbeUpdateVariant(RenderVariant::ParallelDirect, limits, 1u, true, true),
            RenderVariant::ParallelDirect);
  limits.max_shared_memory_bytes = RenderLayer::kDdgiProbeUpdateSharedMemoryBytes;

  --limits.max_shared_memory_bytes;
  EXPECT_EQ(RenderLayer::ResolveDdgiProbeUpdateVariant(RenderVariant::ParallelShared, limits, 1u, true, true),
            RenderVariant::Serial);
  limits.max_shared_memory_bytes = RenderLayer::kDdgiProbeUpdateSharedMemoryBytes;
  limits.max_work_group_invocations = 63u;
  EXPECT_EQ(RenderLayer::ResolveDdgiProbeUpdateVariant(RenderVariant::ParallelDirect, limits, 1u, true, true),
            RenderVariant::Serial);
  limits.max_work_group_invocations = 64u;
  limits.max_work_group_size_x = 63u;
  EXPECT_EQ(RenderLayer::ResolveDdgiProbeUpdateVariant(RenderVariant::ParallelDirect, limits, 1u, true, true),
            RenderVariant::Serial);
  limits.max_work_group_size_x = 64u;
  limits.max_work_group_count_x = 2u;
  limits.max_work_group_count_y = 2u;
  EXPECT_EQ(RenderLayer::ResolveDdgiProbeUpdateVariant(RenderVariant::ParallelDirect, limits, 4u, true, true),
            RenderVariant::ParallelDirect);
  EXPECT_EQ(RenderLayer::ResolveDdgiProbeUpdateVariant(RenderVariant::ParallelDirect, limits, 5u, true, true),
            RenderVariant::Serial);
  limits.max_work_group_count_x = limits.max_work_group_count_y = 65535u;
  EXPECT_EQ(RenderLayer::ResolveDdgiProbeUpdateVariant(RenderVariant::ParallelDirect, limits, 1u, false, true),
            RenderVariant::Serial);
  EXPECT_EQ(RenderLayer::ResolveDdgiProbeUpdateVariant(RenderVariant::ParallelDirect, limits, 1u, true, false),
            RenderVariant::Serial);
}

TEST(DdgiVolume, DdgiProbeUpdateSchedulingCoversTexelsRaysAndBordersExactlyOnce) {
  constexpr uint32_t group_size = RenderLayer::kDdgiProbeUpdateGroupSize;
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

  EXPECT_EQ(volume.probe_counts, glm::ivec3(10, 6, 16));
  EXPECT_EQ(volume.probe_spacing, glm::vec3(1.5f));
  EXPECT_EQ(volume.volume_origin, glm::vec3(0.0f, 3.0f, 3.0f));
  EXPECT_EQ(volume.movement_type, static_cast<int>(DdgiVolumeMovementType::Default));
  EXPECT_EQ(volume.emissive_mesh_sampling_mode, static_cast<int>(DdgiEmissiveMeshSamplingMode::Inherit));
  EXPECT_TRUE(volume.enable_probe_relocation);
  EXPECT_TRUE(volume.enable_probe_variability);
  EXPECT_TRUE(volume.enable_probe_variability_gating);
  EXPECT_FLOAT_EQ(volume.relocation_distance, 0.25f);
  EXPECT_FLOAT_EQ(volume.random_ray_backface_threshold, 0.1f);
  EXPECT_FLOAT_EQ(volume.fixed_ray_backface_threshold, 0.25f);
  EXPECT_FLOAT_EQ(volume.probe_variability_threshold, 0.2f);
  EXPECT_EQ(volume.probe_variability_min_samples, 16);
  EXPECT_EQ(volume.auto_invalidate_trigger_conditions, DdgiVolumeTriggerConditionAll);
  EXPECT_EQ(volume.warmup_trigger_conditions, DdgiVolumeTriggerConditionLightEnableChanged);
  EXPECT_EQ(volume.variability_reset_trigger_conditions,
            DdgiVolumeTriggerConditionLightingConditionChanged | DdgiVolumeTriggerConditionGeometryChanged);
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
  EXPECT_FLOAT_EQ(settings.runtime.max_ray_distance, 1e27f);
  EXPECT_FLOAT_EQ(settings.runtime.visibility_moment_bias, 0.02f);
  EXPECT_EQ(settings.runtime.warmup_frames, 16);
  EXPECT_FLOAT_EQ(settings.runtime.distance_exponent, 50.0f);
  EXPECT_FLOAT_EQ(settings.runtime.irradiance_threshold, 0.25f);
  EXPECT_FLOAT_EQ(settings.runtime.brightness_threshold, 0.10f);
  EXPECT_TRUE(settings.runtime.enable_emissive_mesh_sampling);
  EXPECT_EQ(defaults.movement_type, static_cast<int>(DdgiVolumeMovementType::Default));
  EXPECT_TRUE(defaults.enable_probe_relocation);
  EXPECT_TRUE(defaults.enable_probe_variability);
  EXPECT_TRUE(defaults.enable_probe_variability_gating);
  EXPECT_FLOAT_EQ(defaults.relocation_distance, 0.25f);
  EXPECT_FLOAT_EQ(settings.debug.visualization_scale, 2.0f);
  EXPECT_FLOAT_EQ(defaults.random_ray_backface_threshold, 0.1f);
  EXPECT_FLOAT_EQ(defaults.fixed_ray_backface_threshold, 0.25f);
  EXPECT_FLOAT_EQ(defaults.probe_variability_threshold, 0.2f);
  EXPECT_EQ(defaults.probe_variability_min_samples, 16);
  EXPECT_EQ(RenderLayer::GetDdgiAllocatedProbeCount(settings), 960u);
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
  EXPECT_NE(demo_source.find("lighting.ddgi_volumes.clear();"), std::string::npos);
  EXPECT_NE(rendering_scene_source.find("SetEnvironmentalLightingFallbackIntensities(*lighting, 1.0f, 1.0f)"),
            std::string::npos);
  EXPECT_EQ(rendering_scene_source.find("GetOrSetPrivateComponent<DdgiVolume>"), std::string::npos);
  EXPECT_EQ(rendering_scene_source.find("CreateReflectionProbeComponent(scene"), std::string::npos);
  EXPECT_NE(demo_source.find("{10, 6, 16}"), std::string::npos);
  EXPECT_NE(demo_source.find("ddgi_volume.relocation_distance = 0.25f;"), std::string::npos);
  EXPECT_NE(demo_source.find("settings.debug.visualization_scale = 2.0f;"), std::string::npos);
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
  EXPECT_NE(demo_app_source.find("ddgi_settings.runtime.ray_count != 256"), std::string::npos);
  EXPECT_NE(demo_app_source.find("ddgi_settings.debug.visualization_scale != 2.0f"), std::string::npos);
  EXPECT_NE(demo_app_source.find("volume.probe_counts != glm::ivec3(10, 6, 16)"), std::string::npos);
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
  EXPECT_NE(editor_source.find("lighting->ddgi_settings = ddgi_settings;"), std::string::npos);
  EXPECT_NE(editor_source.find("auto& ddgi_settings = render_layer->GetDdgiSettings();"), std::string::npos);
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
  EXPECT_NE(bistro_ddgi_source.find("lighting->local_reflection_probes.clear();"), std::string::npos);
  EXPECT_NE(bistro_ddgi_source.find("lighting->ddgi_volumes.clear();"), std::string::npos);
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
  EXPECT_NE(validation_source.find("lighting->ddgi_volumes.clear();"), std::string::npos);
  EXPECT_NE(validation_source.find("AddEnvironmentalLightingDdgiVolume(*lighting"), std::string::npos);
  EXPECT_EQ(validation_source.find("scene->environment.ddgi_settings"), std::string::npos);
  EXPECT_NE(validation_source.find("glm::translate(scrolling_volume->transform"), std::string::npos);
  EXPECT_NE(validation_source.find("std::remove_if(lighting->ddgi_volumes.begin()"), std::string::npos);
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
  volume.random_ray_backface_threshold = -1.0f;
  volume.fixed_ray_backface_threshold = 2.0f;
  volume.probe_variability_threshold = 20.0f;
  volume.probe_variability_min_samples = -4;
  volume.auto_invalidate_trigger_conditions = 0xffff;
  volume.warmup_trigger_conditions = 0xffff;
  volume.variability_reset_trigger_conditions = 0xffff;
  volume.ClampSettings();

  EXPECT_EQ(volume.probe_counts, glm::ivec3(-4, 0, 512));
  EXPECT_EQ(volume.GetProbeAmount(), 0u);
  EXPECT_EQ(volume.probe_spacing, glm::vec3(0.05f, 0.05f, 10000.0f));
  EXPECT_EQ(volume.movement_type, static_cast<int>(DdgiVolumeMovementType::Scrolling));
  EXPECT_EQ(volume.emissive_mesh_sampling_mode, static_cast<int>(DdgiEmissiveMeshSamplingMode::Inherit));
  EXPECT_FLOAT_EQ(volume.relocation_distance, 0.0f);
  EXPECT_FLOAT_EQ(volume.random_ray_backface_threshold, 0.0f);
  EXPECT_FLOAT_EQ(volume.fixed_ray_backface_threshold, 1.0f);
  EXPECT_FLOAT_EQ(volume.probe_variability_threshold, 10.0f);
  EXPECT_EQ(volume.probe_variability_min_samples, 0);
  EXPECT_EQ(volume.auto_invalidate_trigger_conditions, DdgiVolumeTriggerConditionAll);
  EXPECT_EQ(volume.warmup_trigger_conditions, DdgiVolumeTriggerConditionAll);
  EXPECT_EQ(volume.variability_reset_trigger_conditions, DdgiVolumeTriggerConditionAll);
  volume.emissive_mesh_sampling_mode = 4;
  volume.ClampSettings();
  EXPECT_EQ(volume.emissive_mesh_sampling_mode, static_cast<int>(DdgiEmissiveMeshSamplingMode::Off));
}

TEST(DdgiVolume, DeserializesCanonicalDdgiSettings) {
  DdgiSettings restored_settings;
  DeserializeDdgiSettings(YAML::Load(R"(
runtime:
  enabled: true
  enable_emissive_mesh_sampling: false
  ray_count: 64
  warmup_frames: 12
  distance_exponent: 42.0
  irradiance_threshold: 0.4
  brightness_threshold: 0.7
volume_defaults:
  probe_counts: [5, 3, 7]
  probe_spacing: [2.0, 2.0, 2.0]
  volume_origin: [1.0, 2.0, 3.0]
  movement_type: 1
  random_ray_backface_threshold: 0.2
  fixed_ray_backface_threshold: 0.4
  enable_probe_variability: false
  enable_probe_variability_gating: true
  probe_variability_threshold: 0.75
  probe_variability_min_samples: 32
storage:
  max_probe_count: 1024
debug:
  enabled: true
  visualize_probe_illumination: false
  probe_visualization_mode: 2
  probe_visualization_depth_mode: 1
  probe_visualization_radius: 0.25
  probe_visualization_intensity: 3.5
  probe_visualization_alpha: 0.4
  selected_probe_visualization_scale: 5.0
  selected_probe_index: 42
)"),
                          restored_settings);
  EXPECT_TRUE(restored_settings.runtime.enabled);
  EXPECT_FALSE(restored_settings.runtime.enable_emissive_mesh_sampling);
  EXPECT_EQ(restored_settings.runtime.ray_count, 64);
  EXPECT_EQ(restored_settings.runtime.warmup_frames, 12);
  EXPECT_FLOAT_EQ(restored_settings.runtime.distance_exponent, 42.0f);
  EXPECT_FLOAT_EQ(restored_settings.runtime.irradiance_threshold, 0.4f);
  EXPECT_FLOAT_EQ(restored_settings.runtime.brightness_threshold, 0.7f);
  EXPECT_EQ(restored_settings.volume_defaults.probe_counts, glm::ivec3(5, 3, 7));
  EXPECT_EQ(restored_settings.volume_defaults.probe_spacing, glm::vec3(2.0f));
  EXPECT_EQ(restored_settings.volume_defaults.volume_origin, glm::vec3(1.0f, 2.0f, 3.0f));
  EXPECT_EQ(restored_settings.volume_defaults.movement_type, static_cast<int>(DdgiVolumeMovementType::Scrolling));
  EXPECT_FLOAT_EQ(restored_settings.volume_defaults.random_ray_backface_threshold, 0.2f);
  EXPECT_FLOAT_EQ(restored_settings.volume_defaults.fixed_ray_backface_threshold, 0.4f);
  EXPECT_FALSE(restored_settings.volume_defaults.enable_probe_variability);
  EXPECT_TRUE(restored_settings.volume_defaults.enable_probe_variability_gating);
  EXPECT_FLOAT_EQ(restored_settings.volume_defaults.probe_variability_threshold, 0.75f);
  EXPECT_EQ(restored_settings.volume_defaults.probe_variability_min_samples, 32);
  EXPECT_EQ(restored_settings.storage.max_probe_count, 1024);
  EXPECT_TRUE(restored_settings.debug.enabled);
  EXPECT_FALSE(restored_settings.debug.visualize_probe_illumination);
  EXPECT_EQ(restored_settings.debug.probe_visualization_mode, 2);
  EXPECT_EQ(restored_settings.debug.probe_visualization_depth_mode, 1);
  EXPECT_FLOAT_EQ(restored_settings.debug.probe_visualization_radius, 0.25f);
  EXPECT_FLOAT_EQ(restored_settings.debug.probe_visualization_intensity, 3.5f);
  EXPECT_FLOAT_EQ(restored_settings.debug.probe_visualization_alpha, 0.4f);
  EXPECT_FLOAT_EQ(restored_settings.debug.selected_probe_visualization_scale, 5.0f);
  EXPECT_EQ(restored_settings.debug.selected_probe_index, 42);
}

TEST(DdgiVolume, ProbeGridIndexUsesXFastestOrder) {
  EXPECT_EQ(RenderLayer::GetDdgiProbeGridIndex({4, 3, 2}, 17), glm::uvec3(1, 1, 1));
  EXPECT_EQ(RenderLayer::GetDdgiProbeGridIndex({4, 3, 2}, 23), glm::uvec3(3, 2, 1));
  EXPECT_EQ(RenderLayer::GetDdgiProbeGridIndex({4, 3, 2}, 99), glm::uvec3(3, 2, 1));
  EXPECT_EQ(RenderLayer::GetDdgiProbeGridIndex({0, 3, 2}, 0), glm::uvec3(0));
}

TEST(DdgiVolume, RenderLayerFrameResourceLayoutRejectsProbeCapOverflowWithoutTruncation) {
  RenderLayer::DdgiSettings settings;
  settings.volume_defaults.probe_counts = {8, 8, 8};
  settings.storage.max_probe_count = 32;
  settings.storage.atlas_probe_columns = 6;
  settings.storage.irradiance_tile_resolution = 8;
  settings.storage.visibility_tile_resolution = 10;
  settings.runtime.ray_count = 64;
  settings.debug.selected_probe_index = 50;

  const auto layout = RenderLayer::CalculateDdgiFrameResourceLayout(settings);

  EXPECT_EQ(RenderLayer::GetDdgiProbeCount(settings.volume_defaults.probe_counts), 512u);
  EXPECT_EQ(RenderLayer::GetDdgiAllocatedProbeCount(settings), 0u);
  EXPECT_FALSE(layout.valid);
  EXPECT_EQ(layout.probe_count, 0u);
  EXPECT_EQ(layout.probe_metadata_byte_size, 0u);
  EXPECT_EQ(layout.ray_output_byte_size, 0u);
  EXPECT_FALSE(layout.error.empty());
}

TEST(DdgiVolume, RenderLayerFrameResourceLayoutUsesActiveVolumeProbeCount) {
  RenderLayer::DdgiSettings settings;
  settings.volume_defaults.probe_counts = {4, 4, 4};
  settings.storage.max_probe_count = 4096;
  settings.storage.atlas_probe_columns = 16;
  settings.storage.irradiance_tile_resolution = 8;
  settings.storage.visibility_tile_resolution = 10;
  settings.runtime.ray_count = 128;

  const auto layout = RenderLayer::CalculateDdgiFrameResourceLayout(settings, 252);

  EXPECT_EQ(RenderLayer::GetDdgiAllocatedProbeCount(settings), 64u);
  EXPECT_EQ(RenderLayer::GetDdgiAllocatedProbeCount(settings, 252), 252u);
  EXPECT_EQ(layout.probe_count, 252u);
  EXPECT_EQ(layout.irradiance_atlas.rows, 16u);
  EXPECT_EQ(layout.irradiance_atlas.resolution, glm::uvec2(160, 160));
  EXPECT_EQ(layout.visibility_atlas.resolution, glm::uvec2(192, 192));
  EXPECT_EQ(layout.variability_atlas.resolution, glm::uvec2(128, 128));
  EXPECT_EQ(layout.variability_reduction_extent, glm::uvec2(8, 8));
  EXPECT_EQ(layout.probe_metadata_byte_size, 252ull * sizeof(glm::vec4) * 3ull);
  EXPECT_EQ(layout.probe_state_byte_size, 252ull * sizeof(glm::vec4));
  EXPECT_EQ(layout.ray_output_byte_size, 252ull * 128ull * sizeof(DdgiProbeRayData));
  EXPECT_EQ(layout.selected_ray_diagnostics_byte_size, 128ull * sizeof(PointCloudSample));
}

TEST(DdgiVolume, CompactRayMemoryAccountingCoversRepresentativeAndMaximumLayouts) {
  RenderLayer::DdgiSettings settings;
  settings.storage.max_probe_count = 8192;
  settings.storage.atlas_probe_columns = 128;
  settings.storage.irradiance_tile_resolution = 8;
  settings.storage.visibility_tile_resolution = 8;
  settings.runtime.ray_count = 256;

  const auto representative = RenderLayer::CalculateDdgiFrameResourceLayout(settings, 8192);
  ASSERT_TRUE(representative.valid) << representative.error;
  EXPECT_EQ(representative.ray_output_byte_size, 33'554'432u);
  EXPECT_EQ(representative.selected_ray_diagnostics_byte_size, 32'768u);
  EXPECT_EQ(representative.per_frame_transient_byte_size,
            representative.ray_output_byte_size + representative.selected_ray_diagnostics_byte_size +
                2u * representative.variability_reduction_byte_size + sizeof(glm::vec2));
  EXPECT_EQ(representative.peak_resident_byte_size,
            representative.persistent_byte_size +
                Platform::kMaxFramesInFlight * representative.per_frame_transient_byte_size);

  settings.runtime.ray_count = 4096;
  const auto maximum = RenderLayer::CalculateDdgiFrameResourceLayout(settings, 8192);
  ASSERT_TRUE(maximum.valid) << maximum.error;
  EXPECT_EQ(maximum.ray_output_byte_size, 536'870'912u);
  EXPECT_EQ(maximum.selected_ray_diagnostics_byte_size, 524'288u);
}

TEST(DdgiVolume, CompactRayShadersMatchHostLayoutAndPreserveSelectedDiagnostics) {
  const auto root =
      std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "Internals" / "DefaultResources" / "Shaders";
  const auto compact = ReadTextFile(root / "Modules" / "EvoEngine" / "DDGIProbeRayData.slang");
  const auto raygen = ReadTextFile(root / "RayTracing" / "RayGen" / "DDGIProbeDiagnostics.slang");
  const auto update = ReadTextFile(root / "Compute" / "DDGIProbeUpdate.slang");
  const auto relocation = ReadTextFile(root / "Compute" / "DDGIProbeRelocation.slang");
  const auto classification = ReadTextFile(root / "Compute" / "DDGIProbeClassification.slang");
  const auto source_root = std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "src";
  const auto ddgi_runtime = ReadTextFile(source_root / "DdgiRuntime.cpp");
  const auto render_layer = ReadTextFile(source_root / "RenderLayer.cpp");
  const auto ray_pass = ReadTextFile(source_root / "RenderPasses" / "DdgiRayDiagnosticsPass.cpp");
  const auto visualization_pass = ReadTextFile(source_root / "RenderPasses" / "DdgiProbeRayVisualizationPass.cpp");

  EXPECT_NE(compact.find("struct DdgiProbeRayData"), std::string::npos);
  EXPECT_NE(compact.find("float4 radiance_and_signed_distance;"), std::string::npos);
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
  EXPECT_NE(
      update.find("uint fixed_ray_count = min(constants.atlas_columns_fixed_ray_count_and_update_mode.z, ray_count);"),
      std::string::npos);
  EXPECT_NE(update.find("asfloat(constants.probe_counts_and_rotation.w)"), std::string::npos);
  EXPECT_NE(update.find("EE_DDGI_ROTATED_PROBE_RAY_DIRECTION"), std::string::npos);
  EXPECT_NE(relocation.find("uint fixed_ray_count = min(constants.probe_counts.w, ray_count);"), std::string::npos);
  EXPECT_NE(relocation.find("if (EE_DDGI_PROBE_RAY_BACKFACE_HIT(ray_data))"), std::string::npos);
  EXPECT_NE(classification.find("uint fixed_ray_count = min(constants.probe_counts.w, ray_count);"), std::string::npos);

  EXPECT_NE(ddgi_runtime.find("sizeof(DdgiProbeRayData)"), std::string::npos);
  EXPECT_NE(render_layer.find("runtime_state.selected_ray_diagnostics_buffers"), std::string::npos);
  EXPECT_NE(render_layer.find("CreateDdgiImportedBufferResourceDescriptor("
                              "RenderResourceNames::frame_ddgi_selected_ray_diagnostics"),
            std::string::npos);
  EXPECT_NE(render_layer.find("glm::floatBitsToUint(runtime_state.frame_ray_push_constant.probe_step_z.w)"),
            std::string::npos);
  EXPECT_NE(ray_pass.find("UpdateBufferDescriptorBinding(2, diagnostics_binding->buffer)"), std::string::npos);
  EXPECT_NE(ray_pass.find("diagnostics_binding->buffer->GetVkBuffer()"), std::string::npos);
  EXPECT_NE(visualization_pass.find("frame_ddgi_selected_ray_diagnostics"), std::string::npos);
}

TEST(DdgiVolume, RenderLayerAcceptsExactProbeCapAndRejectsCapPlusOne) {
  RenderLayer::DdgiSettings settings;
  settings.storage.max_probe_count = 32;
  settings.storage.atlas_probe_columns = 4;
  settings.storage.irradiance_tile_resolution = 8;
  settings.storage.visibility_tile_resolution = 8;
  settings.runtime.ray_count = 16;

  EXPECT_TRUE(RenderLayer::ValidateDdgiProbeGrid({4, 4, 2}, 32));
  EXPECT_FALSE(RenderLayer::ValidateDdgiProbeGrid({4, 4, 3}, 32));
  const auto exact_layout = RenderLayer::CalculateDdgiFrameResourceLayout(settings, 32, 512);
  const auto overflow_layout = RenderLayer::CalculateDdgiFrameResourceLayout(settings, 33, 512);

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
  const auto exact_layout = RenderLayer::CalculateDdgiAtlasLayout(probe_count, 8, 4, 80);
  const auto rejected_layout = RenderLayer::CalculateDdgiAtlasLayout(probe_count, 8, 4, 79);

  ASSERT_TRUE(exact_layout.valid) << exact_layout.error;
  EXPECT_EQ(exact_layout.columns, 4u);
  EXPECT_EQ(exact_layout.rows, 8u);
  EXPECT_EQ(exact_layout.resolution, glm::uvec2(40, 80));
  EXPECT_FALSE(rejected_layout.valid);
  EXPECT_EQ(rejected_layout.columns, 1u);
  EXPECT_FALSE(rejected_layout.error.empty());
}

TEST(DdgiVolume, RenderLayerRejectsZeroAtlasColumnsWithoutReshaping) {
  const auto layout = RenderLayer::CalculateDdgiAtlasLayout(16, 8, 0, 1024);
  EXPECT_FALSE(layout.valid);
  EXPECT_FALSE(layout.error.empty());

  RenderLayer::DdgiSettings settings;
  settings.volume_defaults.probe_counts = {4, 2, 2};
  settings.storage.atlas_probe_columns = 0;
  EXPECT_FALSE(RenderLayer::CalculateDdgiFrameResourceLayout(settings).valid);
}

TEST(DdgiVolume, RenderLayerPropagatesVisibilityAtlasDeviceLimitFailure) {
  RenderLayer::DdgiSettings settings;
  settings.storage.max_probe_count = 32;
  settings.storage.atlas_probe_columns = 4;
  settings.storage.irradiance_tile_resolution = 8;
  settings.storage.visibility_tile_resolution = 16;

  const auto layout = RenderLayer::CalculateDdgiFrameResourceLayout(settings, 32, 100);

  EXPECT_FALSE(layout.valid);
  EXPECT_TRUE(layout.irradiance_atlas.valid);
  EXPECT_FALSE(layout.visibility_atlas.valid);
  EXPECT_FALSE(layout.error.empty());
}

TEST(DdgiVolume, RenderLayerRejectsStorageBuffersBeyondDeviceRange) {
  RenderLayer::DdgiSettings settings;
  settings.storage.max_probe_count = 32;
  settings.storage.atlas_probe_columns = 4;
  settings.runtime.ray_count = 16;
  const auto unrestricted = RenderLayer::CalculateDdgiFrameResourceLayout(settings, 32, 1024);
  ASSERT_TRUE(unrestricted.valid) << unrestricted.error;

  const auto exact =
      RenderLayer::CalculateDdgiFrameResourceLayout(settings, 32, 1024, unrestricted.ray_output_byte_size);
  const auto overflow =
      RenderLayer::CalculateDdgiFrameResourceLayout(settings, 32, 1024, unrestricted.ray_output_byte_size - 1u);
  EXPECT_TRUE(exact.valid) << exact.error;
  EXPECT_FALSE(overflow.valid);
  EXPECT_FALSE(overflow.error.empty());
}

TEST(DdgiVolume, InspectorRejectsRayBufferEditsBeyondDeviceRange) {
  const auto inspector_source = ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" /
                                             "src" / "Editor" / "SDKInspectionAdapters.cpp");
  ASSERT_FALSE(inspector_source.empty());

  EXPECT_NE(inspector_source.find("uint64_t GetDdgiMaxStorageBufferRange()"), std::string::npos);
  EXPECT_NE(inspector_source.find("GetDdgiMaxImageDimension2D(), GetDdgiMaxStorageBufferRange()"), std::string::npos);
  EXPECT_NE(inspector_source.find("const auto previous_ray_count = runtime.ray_count;"), std::string::npos);
  EXPECT_NE(inspector_source.find("runtime.ray_count = previous_ray_count;"), std::string::npos);
  EXPECT_NE(inspector_source.find("DDGI ray-count edit rejected: "), std::string::npos);
}

TEST(DdgiVolume, PausedHistoryRequiresCompatiblePersistentLayout) {
  RenderLayer::DdgiSettings settings;
  settings.storage.max_probe_count = 64;
  settings.storage.atlas_probe_columns = 4;
  settings.runtime.ray_count = 16;
  const auto previous = RenderLayer::CalculateDdgiFrameResourceLayout(settings, 32, 1024);
  ASSERT_TRUE(previous.valid) << previous.error;

  settings.runtime.ray_count = 32;
  const auto transient_only_change = RenderLayer::CalculateDdgiFrameResourceLayout(settings, 32, 1024);
  EXPECT_TRUE(RenderLayer::AreDdgiPersistentLayoutsCompatible(previous, transient_only_change));

  settings.storage.atlas_probe_columns = 8;
  const auto atlas_change = RenderLayer::CalculateDdgiFrameResourceLayout(settings, 32, 1024);
  EXPECT_FALSE(RenderLayer::AreDdgiPersistentLayoutsCompatible(previous, atlas_change));
  EXPECT_FALSE(RenderLayer::AreDdgiPersistentLayoutsCompatible({}, previous));
}

TEST(DdgiVolume, ResolvedVolumeCollectionFeedsAtomicSetValidationWithoutMutatingAuthoring) {
  const auto render_layer_source =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "src" / "RenderLayer.cpp");
  const auto collect = render_layer_source.find(
      "std::vector<RenderLayer::DdgiVolumeRuntimeInfo> RenderLayer::"
      "CollectDdgiVolumeRuntimeInfos");
  const auto collect_end = render_layer_source.find(
      "const std::shared_ptr<DescriptorSetLayout>& RenderLayer::"
      "GetPerFrameDescriptorSetLayout",
      collect);
  const auto resolve =
      render_layer_source.find("const auto resolved_lighting = ResolveEnvironmentalLighting(scene)", collect);
  const auto reserve = render_layer_source.find("infos.reserve(resolved_lighting.ddgi_volumes.size())", resolve);
  const auto iterate =
      render_layer_source.find("for (size_t i = 0; i < resolved_lighting.ddgi_volumes.size(); ++i)", reserve);
  const auto source = render_layer_source.find("CreateDdgiProbeRayDiagnosticSourceFromResolvedVolume", iterate);
  const auto accept = render_layer_source.find("auto& info = infos.emplace_back();", source);
  ASSERT_NE(collect, std::string::npos);
  ASSERT_NE(collect_end, std::string::npos);
  ASSERT_NE(resolve, std::string::npos);
  ASSERT_NE(reserve, std::string::npos);
  ASSERT_NE(iterate, std::string::npos);
  ASSERT_NE(source, std::string::npos);
  ASSERT_NE(accept, std::string::npos);
  EXPECT_LT(resolve, reserve);
  EXPECT_LT(reserve, iterate);
  EXPECT_LT(iterate, source);
  EXPECT_LT(source, accept);
  EXPECT_LT(accept, collect_end);
  const auto collect_scope = render_layer_source.substr(collect, collect_end - collect);
  EXPECT_EQ(collect_scope.find("UnsafeGetPrivateComponentOwnersList<DdgiVolume>"), std::string::npos);
  EXPECT_EQ(collect_scope.find("SetEnabled("), std::string::npos);
  EXPECT_EQ(collect_scope.find("rejected"), std::string::npos);

  const auto prepare = render_layer_source.find("void RenderLayer::PrepareDdgiFrameState");
  const auto prepare_worker = render_layer_source.find("void RenderLayer::PrepareDdgiVolumeFrameState", prepare);
  const auto prepare_resolve =
      render_layer_source.find("const auto resolved_lighting = ResolveEnvironmentalLighting(scene)", prepare);
  const auto prepare_iterate =
      render_layer_source.find("for (const auto& volume : resolved_lighting.ddgi_volumes)", prepare_resolve);
  const auto validate = render_layer_source.find("const auto validation = ValidateDdgiVolumeSet", prepare_iterate);
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

TEST(DdgiVolume, RenderLayerSeparatesDdgiResourceAndTracePolicies) {
  RenderLayer::DdgiSettings settings;
  settings.runtime.enabled = true;
  auto policy = RenderLayer::ResolveDdgiRuntimePolicy(settings);
  EXPECT_TRUE(policy.use_resources);
  EXPECT_TRUE(policy.trace_probe_rays);

  settings.runtime.pause_updates = true;
  policy = RenderLayer::ResolveDdgiRuntimePolicy(settings);
  EXPECT_TRUE(policy.use_resources);
  EXPECT_FALSE(policy.trace_probe_rays);

  settings.runtime.enabled = false;
  settings.runtime.pause_updates = false;
  settings.debug.enabled = true;
  settings.debug.show_rays = true;
  policy = RenderLayer::ResolveDdgiRuntimePolicy(settings);
  EXPECT_TRUE(policy.use_resources);
  EXPECT_TRUE(policy.trace_probe_rays);
}

TEST(DdgiVolume, EmissiveMeshSamplingOverridesResolveAndPackIndependently) {
  const auto inherit = static_cast<int>(DdgiEmissiveMeshSamplingMode::Inherit);
  const auto on = static_cast<int>(DdgiEmissiveMeshSamplingMode::On);
  const auto off = static_cast<int>(DdgiEmissiveMeshSamplingMode::Off);
  EXPECT_FALSE(RenderLayer::ResolveDdgiEmissiveMeshSampling(false, inherit));
  EXPECT_TRUE(RenderLayer::ResolveDdgiEmissiveMeshSampling(true, inherit));
  EXPECT_TRUE(RenderLayer::ResolveDdgiEmissiveMeshSampling(false, on));
  EXPECT_TRUE(RenderLayer::ResolveDdgiEmissiveMeshSampling(true, on));
  EXPECT_FALSE(RenderLayer::ResolveDdgiEmissiveMeshSampling(false, off));
  EXPECT_FALSE(RenderLayer::ResolveDdgiEmissiveMeshSampling(true, off));

  EXPECT_EQ(RenderLayer::GetDdgiProbeRayFlags(false, false), 0u);
  EXPECT_EQ(RenderLayer::GetDdgiProbeRayFlags(true, false), RenderLayer::kDdgiProbeRayFlagSkipInactive);
  EXPECT_EQ(RenderLayer::GetDdgiProbeRayFlags(false, true), RenderLayer::kDdgiProbeRayFlagEmissiveMeshSampling);
  EXPECT_EQ(RenderLayer::GetDdgiProbeRayFlags(true, true),
            RenderLayer::kDdgiProbeRayFlagSkipInactive | RenderLayer::kDdgiProbeRayFlagEmissiveMeshSampling);
  EXPECT_EQ(RenderLayer::kDdgiProbeRayFlagSkipInactive, 1u << 0u);
  EXPECT_EQ(RenderLayer::kDdgiProbeRayFlagEmissiveMeshSampling, 1u << 1u);
}

TEST(DdgiVolume, EmissiveSamplingCandidateCountIsAnUpperBoundForNonFixedRays) {
  EXPECT_EQ(RenderLayer::CalculateDdgiEmissiveSamplingCandidateRayCount(3u, 64u, 6u, true, true), 174u);
  EXPECT_EQ(RenderLayer::CalculateDdgiEmissiveSamplingCandidateRayCount(3u, 64u, 80u, true, true), 0u);
  EXPECT_EQ(RenderLayer::CalculateDdgiEmissiveSamplingCandidateRayCount(3u, 64u, 6u, false, true), 0u);
  EXPECT_EQ(RenderLayer::CalculateDdgiEmissiveSamplingCandidateRayCount(3u, 64u, 6u, true, false), 0u);
  EXPECT_EQ(RenderLayer::CalculateDdgiEmissiveSamplingCandidateRayCount(0xffffffffu, 2u, 0u, true, true),
            static_cast<uint64_t>(0xffffffffu) * 2u);
}

TEST(DdgiVolume, RenderLayerRequiresFullResetWhenScrollInvalidatesAnAxis) {
  const glm::ivec3 probe_counts(8, 6, 4);
  EXPECT_FALSE(RenderLayer::RequiresDdgiFullScrollReset(probe_counts, {1, 0, 0}));
  EXPECT_FALSE(RenderLayer::RequiresDdgiFullScrollReset(probe_counts, {2, -1, 3}));
  EXPECT_TRUE(RenderLayer::RequiresDdgiFullScrollReset(probe_counts, {8, 0, 0}));
  EXPECT_TRUE(RenderLayer::RequiresDdgiFullScrollReset(probe_counts, {0, -7, 0}));
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
  const auto raygen_source = ReadTextFile(shader_root / "RayTracing" / "RayGen" / "DDGIProbeDiagnostics.slang");
  const auto closest_hit_source =
      ReadTextFile(shader_root / "RayTracing" / "ClosestHit" / "DDGIProbeDiagnostics.slang");
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

  EXPECT_NE(probe_update_source.find("1.0f / (2.0f * max(accumulator.weight_sum, epsilon))"), std::string::npos);
  EXPECT_NE(probe_update_source.find("EE_DDGI_PROBE_MAX_VISIBILITY_DISTANCE()"), std::string::npos);
  EXPECT_NE(probe_update_source.find("float2(accumulator.first_moment, accumulator.second_moment) * "
                                     "(1.0f / (2.0f * accumulator.weight_sum))"),
            std::string::npos);
  EXPECT_NE(probe_update_source.find("return float4(irradiance, 1.0f);"), std::string::npos);
  EXPECT_EQ(probe_update_source.find("classification_enabled && classification_inside_geometry ? 1.0f : 0.0f"),
            std::string::npos);
  EXPECT_NE(probe_update_source.find(
                "uint fixed_ray_count = min(constants.atlas_columns_fixed_ray_count_and_update_mode.z, ray_count);"),
            std::string::npos);
  EXPECT_EQ(probe_update_source.find("!classification_has_nearby_geometry"), std::string::npos);
  EXPECT_EQ(raygen_source.find("uint EE_DDGI_RANDOM_ROTATION_SEED"), std::string::npos);
  EXPECT_EQ(raygen_source.find("EE_RANGED_RANDOM"), std::string::npos);
  EXPECT_NE(raygen_source.find("float4 EE_DDGI_PROBE_RAY_ROTATION()"), std::string::npos);
  EXPECT_NE(ddgi_helper_source.find("EE_DDGI_ROTATE_BY_CONJUGATE_QUATERNION"), std::string::npos);
  EXPECT_NE(raygen_source.find("RayDesc ray = {origin, 0.0f, direction"), std::string::npos);
  EXPECT_NE(raygen_source.find("hit_value.seed = fixed_ray ? EE_DDGI_FIXED_RAY_PAYLOAD_FLAG : primary_ray_seed;"),
            std::string::npos);
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
  EXPECT_NE(closest_hit_source.find("const float3 emissive_radiance = EE_RT_COATED_EMISSION("), std::string::npos);
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
  EXPECT_NE(render_layer_source.find("CreateDdgiProbeRayRotationQuaternion"), std::string::npos);
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
  EXPECT_NE(
      render_layer_source.find("CreateDdgiFallbackProbeStateBuffer(kDdgiMaxResidentProbeCount * sizeof(glm::vec4))"),
      std::string::npos);
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
  const auto deferred =
      ReadTextFile(shader_root / "Graphics" / "Fragment" / "Standard" / "StandardDeferredLighting.slang");
  const auto scene_camera =
      ReadTextFile(shader_root / "Graphics" / "Fragment" / "Standard" / "StandardDeferredLightingSceneCamera.slang");
  const auto transparent =
      ReadTextFile(shader_root / "Graphics" / "Fragment" / "Standard" / "StandardTransparent.slang");
  ASSERT_FALSE(deferred.empty());
  ASSERT_FALSE(scene_camera.empty());
  ASSERT_FALSE(transparent.empty());

  EXPECT_NE(deferred.find("pbr_flags.x, normal_roughness.a, pbr_flags.yzw"), std::string::npos);
  EXPECT_NE(scene_camera.find("pbr_flags.x, normal_roughness.a, pbr_flags.yzw"), std::string::npos);
  EXPECT_NE(transparent.find("surface.specular_f90, surface.occlusion,"), std::string::npos);
  EXPECT_NE(transparent.find("surface.occlusion, 1.0f"), std::string::npos);
  EXPECT_EQ(transparent.find("inAmbientOcclusion"), std::string::npos);
  EXPECT_EQ(deferred.find("ambient * ao"), std::string::npos);
  EXPECT_EQ(scene_camera.find("ambient * ao"), std::string::npos);
  EXPECT_EQ(transparent.find("ambient * surface.occlusion"), std::string::npos);
  EXPECT_NE(deferred.find("indirect_debug_view != 0"), std::string::npos);
  EXPECT_NE(scene_camera.find("indirect_debug_view != 0"), std::string::npos);
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

TEST(DdgiVolume, DdgiProbeMissRaysSampleSceneEnvironment) {
  const auto shader_root =
      std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "Internals" / "DefaultResources" / "Shaders";
  const auto miss_source = ReadTextFile(shader_root / "RayTracing" / "Miss" / "DDGIProbeDiagnostics.slang");
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
  EXPECT_NE(miss_source.find("EE_ENVIRONMENT.diffuse_fallback_intensity"), std::string::npos);
  EXPECT_EQ(miss_source.find("EE_ENVIRONMENT.diffuse_sky_intensity"), std::string::npos);
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

TEST(DdgiVolume, DdgiProbeUpdateUsesRtxgiBlendWithoutTemporalClamp) {
  const auto shader_root =
      std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "Internals" / "DefaultResources" / "Shaders";
  const auto probe_update_source = ReadTextFile(shader_root / "Compute" / "DDGIProbeUpdate.slang");
  const auto render_layer_source =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "src" / "RenderLayer.cpp");
  ASSERT_FALSE(probe_update_source.empty());
  ASSERT_FALSE(render_layer_source.empty());

  EXPECT_EQ(probe_update_source.find("EE_DDGI_TEMPORAL_CLAMP"), std::string::npos);
  EXPECT_NE(probe_update_source.find("EE_DDGI_BLEND_IRRADIANCE_HISTORY"), std::string::npos);
  EXPECT_NE(probe_update_source.find("[vk::image_format(\"r16f\")]"), std::string::npos);
  EXPECT_NE(probe_update_source.find("RWTexture2D<float> EE_DDGI_PROBE_VARIABILITY_ATLAS;"), std::string::npos);
  EXPECT_NE(probe_update_source.find("hysteresis = max(0.0f, hysteresis - 0.75f);"), std::string::npos);
  EXPECT_NE(probe_update_source.find("delta *= 0.25f;"), std::string::npos);
  EXPECT_NE(probe_update_source.find("mean_luminance <= (1.0f / 1024.0f)"), std::string::npos);
  EXPECT_NE(probe_update_source.find("return float4(irradiance, 1.0f);"), std::string::npos);
  EXPECT_EQ(probe_update_source.find("weight_sum > epsilon ? 1.0f : 0.0f"), std::string::npos);
  EXPECT_NE(probe_update_source.find("if (directional_irradiance.a <= 0.0f)"), std::string::npos);
  EXPECT_NE(probe_update_source.find("EE_DDGI_BLEND_IRRADIANCE_HISTORY(directional_irradiance, "
                                     "history_irradiance, history_weight);"),
            std::string::npos);
  EXPECT_NE(probe_update_source.find("float2 visibility_moments = lerp(directional_visibility, "
                                     "history_visibility, history_weight);"),
            std::string::npos);
  EXPECT_NE(probe_update_source.find("float coefficient_of_variation"), std::string::npos);
  EXPECT_NE(probe_update_source.find("float3 irradiance_sample = directional_irradiance.rgb;"), std::string::npos);
  EXPECT_NE(probe_update_source.find("float3 irradiance_sigma2"), std::string::npos);
  EXPECT_NE(probe_update_source.find("float luminance_sigma2 = EE_DDGI_LUMINANCE(irradiance_sigma2);"),
            std::string::npos);
  EXPECT_NE(probe_update_source.find("sqrt(max(luminance_sigma2, 0.0f)) / mean_luminance"), std::string::npos);
  EXPECT_EQ(probe_update_source.find("sqrt(max(EE_DDGI_LUMINANCE(variance), 0.0f))"), std::string::npos);
  EXPECT_NE(probe_update_source.find("EE_DDGI_PROBE_VARIABILITY_ATLAS["), std::string::npos);
  const auto previous_state_read =
      probe_update_source.find("float4 previous_state = EE_DDGI_PROBE_STATE[physical_probe_index];");
  const auto previous_state_blend_guard =
      probe_update_source.find("bool blend_previous_probe_state = previous_inactive <= 0.5f;");
  const auto irradiance_blend = probe_update_source.find(
      "EE_DDGI_BLEND_IRRADIANCE_HISTORY(directional_irradiance, history_irradiance, "
      "history_weight);");
  ASSERT_NE(previous_state_read, std::string::npos);
  ASSERT_NE(previous_state_blend_guard, std::string::npos);
  ASSERT_NE(irradiance_blend, std::string::npos);
  EXPECT_LT(previous_state_read, previous_state_blend_guard);
  EXPECT_LT(previous_state_blend_guard, irradiance_blend);
  EXPECT_NE(probe_update_source.find("StructuredBuffer<float4> EE_DDGI_PROBE_STATE;"), std::string::npos);
  EXPECT_NE(probe_update_source.find("float4(relocation_offset, 1.0f - previous_inactive)"), std::string::npos);
  EXPECT_EQ(probe_update_source.find("EE_DDGI_PROBE_STATE[physical_probe_index] ="), std::string::npos);
  EXPECT_EQ(probe_update_source.find("state_value"), std::string::npos);
  EXPECT_EQ(probe_update_source.find("classification_inside_geometry"), std::string::npos);
  EXPECT_EQ(probe_update_source.find("relocation_candidate"), std::string::npos);
  EXPECT_EQ(probe_update_source.find("directional_irradiance *= 1.0f - inactive;"), std::string::npos);
  EXPECT_EQ(probe_update_source.find("history_visibility = EE_DDGI_TEMPORAL_CLAMP"), std::string::npos);
  EXPECT_NE(render_layer_source.find("settings.runtime.distance_exponent"), std::string::npos);
  EXPECT_NE(render_layer_source.find("settings.runtime.irradiance_threshold"), std::string::npos);
  EXPECT_NE(render_layer_source.find("settings.runtime.brightness_threshold"), std::string::npos);
  EXPECT_NE(render_layer_source.find("CalculateDdgiUpdateBrightnessThreshold"), std::string::npos);
  EXPECT_NE(render_layer_source.find("ddgi_update_brightness_threshold"), std::string::npos);
  EXPECT_EQ(render_layer_source.find("temporal_clamp"), std::string::npos);
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
  EXPECT_NE(relocation_source.find("bool reset_offsets = constants.probe_count_ray_count_and_flags.w != 0u;"),
            std::string::npos);
  EXPECT_NE(relocation_source.find("state.xyz = float3(0.0f);"), std::string::npos);
  EXPECT_NE(relocation_source.find("uint fixed_ray_count = min(constants.probe_counts.w, ray_count);"),
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
  EXPECT_NE(render_layer_source.find(
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
  EXPECT_NE(classification_source.find("uint fixed_ray_count = min(constants.probe_counts.w, ray_count);"),
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

TEST(DdgiVolume, DdgiProbeVariabilityUsesSlangReductionWithoutHlslPath) {
  const auto shader_root =
      std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "Internals" / "DefaultResources" / "Shaders";
  const auto reduce_source = ReadTextFile(shader_root / "Compute" / "DDGIProbeVariabilityReduce.slang");
  const auto extra_reduce_source = ReadTextFile(shader_root / "Compute" / "DDGIProbeVariabilityExtraReduce.slang");
  const auto update_source = ReadTextFile(shader_root / "Compute" / "DDGIProbeUpdate.slang");
  const auto render_layer_source =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "src" / "RenderLayer.cpp");
  ASSERT_FALSE(reduce_source.empty());
  ASSERT_FALSE(extra_reduce_source.empty());
  ASSERT_FALSE(update_source.empty());
  ASSERT_FALSE(render_layer_source.empty());

  EXPECT_NE(reduce_source.find("[vk::image_format(\"r16f\")]"), std::string::npos);
  EXPECT_NE(reduce_source.find("RWTexture2D<float> EE_DDGI_PROBE_VARIABILITY_ATLAS;"), std::string::npos);
  EXPECT_NE(reduce_source.find("EE_DDGI_PROBE_STATE[probe_index].w > 0.5f"), std::string::npos);
  EXPECT_NE(reduce_source.find("float total_possible_samples = 16.0f * 16.0f;"), std::string::npos);
  EXPECT_NE(reduce_source.find("float normalized_weight = weight_sum / total_possible_samples;"), std::string::npos);
  EXPECT_NE(reduce_source.find("EE_DDGI_VARIABILITY_REDUCTION_OUTPUT["), std::string::npos);
  EXPECT_NE(extra_reduce_source.find("[vk::image_format(\"rg32f\")]"), std::string::npos);
  EXPECT_NE(extra_reduce_source.find("RWTexture2D<float2> EE_DDGI_VARIABILITY_REDUCTION_INPUT;"), std::string::npos);
  EXPECT_NE(extra_reduce_source.find("weighted_sum += sample_value.r * sample_value.g"), std::string::npos);
  EXPECT_NE(extra_reduce_source.find("float total_possible_weight = 16.0f * 16.0f;"), std::string::npos);
  EXPECT_NE(extra_reduce_source.find("float normalized_weight = weight_sum / total_possible_weight;"),
            std::string::npos);
  EXPECT_NE(render_layer_source.find("DDGIProbeVariabilityReduce.slang"), std::string::npos);
  EXPECT_NE(render_layer_source.find("DDGIProbeVariabilityExtraReduce.slang"), std::string::npos);
  EXPECT_NE(update_source.find("sqrt(max(luminance_sigma2, 0.0f))"), std::string::npos);
  EXPECT_NE(render_layer_source.find("ReadDdgiProbeVariabilityObservation"), std::string::npos);
  EXPECT_NE(render_layer_source.find("result.y <= 0.0f"), std::string::npos);
  EXPECT_NE(render_layer_source.find("AdvanceDdgiProbeConvergence"), std::string::npos);
  EXPECT_NE(render_layer_source.find("probe_variability_min_samples = 16"), std::string::npos);
  EXPECT_NE(render_layer_source.find("probe_variability_threshold = 0.2f"), std::string::npos);
  EXPECT_NE(render_layer_source.find("probe_variability_gating_enabled"), std::string::npos);
  EXPECT_NE(render_layer_source.find("ddgi_ray_source.enable_probe_variability_gating"), std::string::npos);
  EXPECT_NE(render_layer_source.find("runtime_state.probe_variability_sample_count = 0;"), std::string::npos);
  EXPECT_NE(render_layer_source.find("runtime_state.probe_variability_stable_sample_count = 0;"), std::string::npos);
  EXPECT_NE(render_layer_source.find("DdgiUpdateReasonConverged"), std::string::npos);
  EXPECT_NE(render_layer_source.find("runtime_state.frame_probe_warmup_active"), std::string::npos);
  EXPECT_NE(render_layer_source.find("kDdgiProbeRefreshInterval"), std::string::npos);
  EXPECT_NE(render_layer_source.find("runtime_state.probe_variability_refresh_waiting"), std::string::npos);
  EXPECT_EQ(reduce_source.find(".hlsl"), std::string::npos);
  EXPECT_EQ(extra_reduce_source.find(".hlsl"), std::string::npos);
  EXPECT_EQ(render_layer_source.find("DXC"), std::string::npos);
}

TEST(DdgiVolume, ProbeConvergenceRequiresConsecutiveValidSamplesAndUsesExitHysteresis) {
  RenderLayer::DdgiProbeConvergenceState state{16u, 0u, false};
  const RenderLayer::DdgiProbeVariabilityObservation stable{true, 0.19f, 1.0f};
  auto update = RenderLayer::AdvanceDdgiProbeConvergence(state, stable, 16u, 0.2f);
  EXPECT_EQ(update.state.sample_count, 17u);
  EXPECT_EQ(update.state.stable_sample_count, 1u);
  EXPECT_FALSE(update.state.converged);

  state = update.state;
  update = RenderLayer::AdvanceDdgiProbeConvergence(state, {true, 0.21f, 1.0f}, 16u, 0.2f);
  EXPECT_EQ(update.state.stable_sample_count, 0u);
  EXPECT_FALSE(update.state.converged);

  state = {16u, 0u, false};
  for (uint32_t sample_index = 0; sample_index < RenderLayer::kDdgiProbeVariabilityStableSampleCount; ++sample_index) {
    update = RenderLayer::AdvanceDdgiProbeConvergence(state, stable, 16u, 0.2f);
    state = update.state;
  }
  EXPECT_TRUE(update.entered_convergence);
  EXPECT_TRUE(state.converged);
  EXPECT_EQ(state.stable_sample_count, RenderLayer::kDdgiProbeVariabilityStableSampleCount);

  update = RenderLayer::AdvanceDdgiProbeConvergence(state, {true, 0.24f, 1.0f}, 16u, 0.2f);
  EXPECT_TRUE(update.state.converged);
  update = RenderLayer::AdvanceDdgiProbeConvergence(update.state, {true, 0.251f, 1.0f}, 16u, 0.2f);
  EXPECT_FALSE(update.state.converged);

  const auto invalid = RenderLayer::AdvanceDdgiProbeConvergence(state, {false, 0.0f, 0.0f}, 16u, 0.2f);
  EXPECT_EQ(invalid.state.sample_count, state.sample_count);
  EXPECT_EQ(invalid.state.stable_sample_count, state.stable_sample_count);
  EXPECT_EQ(invalid.state.converged, state.converged);

  const RenderLayer::DdgiProbeConvergenceState partial_streak{17u, 2u, false};
  const std::vector<RenderLayer::DdgiProbeVariabilityObservation> invalid_observations{
      {false, 0.1f, 1.0f},
      {true, (std::numeric_limits<float>::quiet_NaN)(), 1.0f},
      {true, 0.1f, (std::numeric_limits<float>::infinity)()},
      {true, -0.1f, 1.0f},
      {true, 0.1f, 0.0f},
      {true, 0.1f, -1.0f},
  };
  for (const auto& observation : invalid_observations) {
    const auto interrupted = RenderLayer::AdvanceDdgiProbeConvergence(partial_streak, observation, 16u, 0.2f);
    EXPECT_EQ(interrupted.state.sample_count, partial_streak.sample_count);
    EXPECT_EQ(interrupted.state.stable_sample_count, 0u);
    EXPECT_FALSE(interrupted.state.converged);
  }
}

TEST(DdgiVolume, ProbeRefreshIsDeterministic) {
  EXPECT_FALSE(RenderLayer::IsDdgiPeriodicRefreshDue(true, true, false, RenderLayer::kDdgiProbeRefreshInterval - 1u));
  EXPECT_TRUE(RenderLayer::IsDdgiPeriodicRefreshDue(true, true, false, RenderLayer::kDdgiProbeRefreshInterval));
  EXPECT_FALSE(RenderLayer::IsDdgiPeriodicRefreshDue(true, true, true, RenderLayer::kDdgiProbeRefreshInterval));
  EXPECT_FALSE(RenderLayer::IsDdgiPeriodicRefreshDue(false, true, false, RenderLayer::kDdgiProbeRefreshInterval));
}

TEST(DdgiVolume, ReflectionProbeBakeRequiresReadyDdgiRuntime) {
  EXPECT_FALSE(RenderLayer::IsDdgiReflectionProbeRuntimeReady(false, true, false, false));
  EXPECT_FALSE(RenderLayer::IsDdgiReflectionProbeRuntimeReady(true, false, false, false));
  EXPECT_TRUE(RenderLayer::IsDdgiReflectionProbeRuntimeReady(true, true, false, false));
  EXPECT_FALSE(RenderLayer::IsDdgiReflectionProbeRuntimeReady(true, true, true, false));
  EXPECT_TRUE(RenderLayer::IsDdgiReflectionProbeRuntimeReady(true, true, true, true));
}

TEST(DdgiVolume, DdgiWarmupFrameCountIsGlobalAndTriggerPolicyIsPerVolume) {
  const auto settings_source = ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" /
                                            "include" / "Rendering" / "PBR" / "DdgiSettings.hpp");
  const auto volume_source = ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" /
                                          "include" / "Rendering" / "PBR" / "EnvironmentalLighting.hpp");
  const auto ddgi_settings_source =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "src" / "DdgiSettings.cpp");
  const auto inspector_source = ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" /
                                             "src" / "Editor" / "SDKInspectionAdapters.cpp");
  ASSERT_FALSE(settings_source.empty());
  ASSERT_FALSE(volume_source.empty());
  ASSERT_FALSE(ddgi_settings_source.empty());
  ASSERT_FALSE(inspector_source.empty());

  EXPECT_NE(settings_source.find("int warmup_frames = 16;"), std::string::npos);
  EXPECT_EQ(volume_source.find("warmup_frames"), std::string::npos);
  EXPECT_NE(ddgi_settings_source.find("\"warmup_frames\""), std::string::npos);
  EXPECT_NE(inspector_source.find("runtime.warmup_frames = glm::clamp(runtime.warmup_frames, 0, 4096);"),
            std::string::npos);
  EXPECT_NE(inspector_source.find("ImGui::DragInt(\"Warm up frames\""), std::string::npos);
  EXPECT_EQ(settings_source.find("reset_conditions"), std::string::npos);
  EXPECT_EQ(ddgi_settings_source.find("\"reset_conditions\""), std::string::npos);
  EXPECT_EQ(ddgi_settings_source.find("DeserializeDdgiProbeSpacing"), std::string::npos);
  EXPECT_EQ(ddgi_settings_source.find("\"volume_offset\""), std::string::npos);
  EXPECT_NE(ddgi_settings_source.find("volume_defaults[\"probe_spacing\"].as<glm::vec3>()"), std::string::npos);
  EXPECT_NE(volume_source.find("int auto_invalidate_trigger_conditions = DdgiVolumeTriggerConditionAll;"),
            std::string::npos);
  EXPECT_NE(volume_source.find("int warmup_trigger_conditions = DdgiVolumeTriggerConditionLightEnableChanged;"),
            std::string::npos);
  EXPECT_NE(volume_source.find("int variability_reset_trigger_conditions ="), std::string::npos);
  EXPECT_NE(inspector_source.find("DrawDdgiVolumeTriggerConditionCheckbox"), std::string::npos);
  EXPECT_NE(inspector_source.find("InspectDdgiVolumeTriggerConditions"), std::string::npos);
  EXPECT_NE(inspector_source.find("Auto invalidate history on scene changes"), std::string::npos);
  EXPECT_NE(inspector_source.find("Warmup triggers"), std::string::npos);
  EXPECT_NE(inspector_source.find("Variability reset triggers"), std::string::npos);
  EXPECT_NE(inspector_source.find("Light enable changed"), std::string::npos);
  EXPECT_NE(inspector_source.find("Lighting condition changed"), std::string::npos);
  EXPECT_NE(inspector_source.find("Geometry changed"), std::string::npos);
  EXPECT_NE(inspector_source.find("Invalidate history now"), std::string::npos);
  EXPECT_NE(inspector_source.find("Invalidate history next frame"), std::string::npos);
  EXPECT_EQ(inspector_source.find("Reset pending"), std::string::npos);
  EXPECT_EQ(inspector_source.find("ImGui::DragInt(\"Auto invalidate trigger conditions\""), std::string::npos);
  EXPECT_EQ(inspector_source.find("ImGui::DragInt(\"Warmup trigger conditions\""), std::string::npos);
  EXPECT_EQ(inspector_source.find("ImGui::DragInt(\"Variability reset trigger conditions\""), std::string::npos);
  EXPECT_EQ(inspector_source.find("Probe volume defaults"), std::string::npos);
}

TEST(DdgiVolume, RenderLayerUsesPerVolumeDdgiTriggerPolicy) {
  const auto render_layer_source =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "src" / "RenderLayer.cpp");
  ASSERT_FALSE(render_layer_source.empty());

  EXPECT_NE(render_layer_source.find("std::vector<uint64_t> CollectDdgiLightSignatures"), std::string::npos);
  EXPECT_NE(render_layer_source.find("std::vector<uint64_t> CollectDdgiActiveLightKeys"), std::string::npos);
  EXPECT_NE(render_layer_source.find("bool DdgiTriggerConditionEnabled"), std::string::npos);
  EXPECT_NE(render_layer_source.find("CollectDdgiLightSignatures<DirectionalLight>"), std::string::npos);
  EXPECT_NE(render_layer_source.find("CollectDdgiLightSignatures<PointLight>"), std::string::npos);
  EXPECT_NE(render_layer_source.find("CollectDdgiLightSignatures<SpotLight>"), std::string::npos);
  EXPECT_NE(render_layer_source.find("CollectDdgiActiveLightKeys<DirectionalLight>"), std::string::npos);
  EXPECT_EQ(render_layer_source.find("MakeDdgiLightSignatureBase"), std::string::npos);
  EXPECT_NE(render_layer_source.find("light.range > 0.0f ? light.range : light.GetFarPlane()"), std::string::npos);
  EXPECT_NE(render_layer_source.find("MakeDdgiEnvironmentSignature"), std::string::npos);
  EXPECT_NE(render_layer_source.find("CollectDdgiMaterialInputSignatures"), std::string::npos);
  EXPECT_NE(render_layer_source.find("GetDdgiMaterialTextureSlots"), std::string::npos);
  EXPECT_NE(render_layer_source.find("TryGetTexture2DContentSignature"), std::string::npos);
  EXPECT_EQ(render_layer_source.find("ddgi_previous_texture_storage_version_"), std::string::npos);
  EXPECT_NE(render_layer_source.find("GetDdgiEmissiveInventorySignature"), std::string::npos);
  EXPECT_NE(render_layer_source.find("light_signatures != ddgi_previous_light_signatures_"), std::string::npos);
  EXPECT_NE(render_layer_source.find("collect_ddgi_geometry_signatures"), std::string::npos);
  EXPECT_NE(render_layer_source.find("geometry_signatures != ddgi_previous_geometry_signatures_"), std::string::npos);
  EXPECT_NE(render_layer_source.find("render_instance->geometry_version"), std::string::npos);
  EXPECT_NE(render_layer_source.find("render_instance->model.value"), std::string::npos);
  EXPECT_EQ(render_layer_source.find("collect_ddgi_geometry_signatures(current_render_instances->"
                                     "deferred_strands_render_instances)"),
            std::string::npos);
  EXPECT_NE(render_layer_source.find("external->HasDdgiRayTracingGeometry()"), std::string::npos);
  EXPECT_NE(render_layer_source.find("reset_ddgi_warmup_state"), std::string::npos);
  EXPECT_NE(render_layer_source.find("reset_ddgi_variability_state"), std::string::npos);
  EXPECT_NE(render_layer_source.find("scene_auto_invalidate"), std::string::npos);
  EXPECT_NE(render_layer_source.find("auto_invalidate_triggers"), std::string::npos);
  EXPECT_NE(render_layer_source.find("ddgi_latched_scene_change_triggers_"), std::string::npos);
  EXPECT_NE(render_layer_source.find("ddgi_latched_scene_geometry_changed_"), std::string::npos);
  EXPECT_NE(render_layer_source.find("DdgiUpdateReasonSceneInput"), std::string::npos);
  EXPECT_NE(render_layer_source.find("ddgi_ray_source.auto_invalidate_trigger_conditions"), std::string::npos);
  EXPECT_NE(render_layer_source.find("ddgi_ray_source.warmup_trigger_conditions"), std::string::npos);
  EXPECT_NE(render_layer_source.find("ddgi_ray_source.variability_reset_trigger_conditions"), std::string::npos);
  EXPECT_NE(render_layer_source.find("DdgiVolumeTriggerConditionLightEnableChanged"), std::string::npos);
  EXPECT_NE(render_layer_source.find("DdgiVolumeTriggerConditionLightingConditionChanged"), std::string::npos);
  EXPECT_NE(render_layer_source.find("DdgiVolumeTriggerConditionGeometryChanged"), std::string::npos);
  EXPECT_EQ(render_layer_source.find("reset_ddgi_convergence_state"), std::string::npos);
  EXPECT_EQ(render_layer_source.find("DdgiResetConditionEnabled"), std::string::npos);
  EXPECT_EQ(render_layer_source.find("runtime.reset_conditions"), std::string::npos);
  EXPECT_NE(render_layer_source.find("runtime_state.frame_probe_warmup_active"), std::string::npos);
  const auto preserve_ddgi = render_layer_source.find(
      "PreserveDdgiRenderInfo(current_render_instances->render_info_block, "
      "previous_render_instances->render_info_block)");
  const auto compare_render_instances =
      render_layer_source.find("render_instance_updated = *current_render_instances != *previous_render_instances");
  const auto restore_ddgi = render_layer_source.find(
      "PreserveDdgiRenderInfo(current_render_instances->render_info_block, current_render_info)");
  ASSERT_NE(preserve_ddgi, std::string::npos);
  ASSERT_NE(compare_render_instances, std::string::npos);
  ASSERT_NE(restore_ddgi, std::string::npos);
  EXPECT_LT(preserve_ddgi, compare_render_instances);
  EXPECT_LT(compare_render_instances, restore_ddgi);
  EXPECT_EQ(render_layer_source.find("blocks_changed(current_render_instances->directional_light_info_blocks_"),
            std::string::npos);
  EXPECT_EQ(render_layer_source.find("blocks_changed(current_render_instances->GetInstanceInfoBlocks()"),
            std::string::npos);
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

  for (const auto* field :
       {"pbr_base_color_factor", "emissive_factor", "normal_texture_scale", "pbr_metallic_factor", "alpha_mode",
        "alpha_cutoff", "double_sided", "ior", "transmission_factor", "thickness_factor", "clearcoat_factor",
        "clearcoat_normal_texture_scale", "specular_color_factor", "specular_factor", "unlit", "pbr_model",
        "pbr_diffuse_factor", "pbr_specular_factor", "diffuse_transmission_factor"}) {
    EXPECT_NE(material_signature.find(field), std::string::npos) << field;
  }
  for (const auto* field :
       {"pbr_roughness_factor", "occlusion_texture_strength", "attenuation_color", "attenuation_distance",
        "clearcoat_roughness", "sheen_color_factor", "sheen_roughness_factor", "pbr_glossiness_factor"}) {
    EXPECT_EQ(material_signature.find(field), std::string::npos) << field;
  }

  for (const auto* field : {"pbr_base_color_texture", "normal_texture", "pbr_metallic_roughness_texture",
                            "emissive_texture", "clearcoat_texture", "clearcoat_normal_texture", "specular_texture",
                            "specular_color_texture", "pbr_diffuse_texture", "pbr_specular_glossiness_texture"}) {
    EXPECT_NE(texture_slots.find(field), std::string::npos) << field;
  }
  EXPECT_EQ(CountOccurrences(texture_slots, "material."), 10u);
  for (const auto* field : {"occlusion_texture", "transmission_texture", "thickness_texture",
                            "clearcoat_roughness_texture", "sheen_color_texture", "sheen_roughness_texture"}) {
    EXPECT_EQ(texture_slots.find(field), std::string::npos) << field;
  }
  EXPECT_EQ(render_layer_source.find("GetGltfMaterialTextureSlots"), std::string::npos);
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
  EXPECT_NE(cubemap_source.find("++RefStorage().content_generation;"), std::string::npos);
  EXPECT_NE(reflection_probe_source.find("filtered_cubemap->MarkGpuContentValid();"), std::string::npos);
  EXPECT_NE(light_probe_source.find("cubemap_->MarkGpuContentValid();"), std::string::npos);
  EXPECT_NE(environment_signature.find("selected_cubemap_index"), std::string::npos);
  EXPECT_NE(environment_signature.find("TryGetCubemapContentSignature"), std::string::npos);
  EXPECT_NE(environment_signature.find("environment.diffuse_sky_intensity"), std::string::npos);
  EXPECT_NE(environment_signature.find("environment.diffuse_fallback_intensity"), std::string::npos);
  EXPECT_NE(environment_signature.find("environment.environment_rotation"), std::string::npos);
  EXPECT_EQ(environment_signature.find("environment.global_reflection_intensity"), std::string::npos);
  EXPECT_EQ(environment_signature.find("environment.specular_fallback_intensity"), std::string::npos);
  EXPECT_EQ(environment_signature.find("indirect_lighting_intensity"), std::string::npos);
  EXPECT_EQ(environment_signature.find("background_intensity"), std::string::npos);
  for (const auto* pending_input : {"texture.new_data_", "texture.new_compressed_data_", "IsGpuUploadPending()"}) {
    EXPECT_NE(pending_texture.find(pending_input), std::string::npos) << pending_input;
  }
  EXPECT_NE(environment_signature.find("environment.diffuse_fallback_intensity <= 0.0f"), std::string::npos);
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
        "forward_render_instances", "forward_skinned_render_instances", "forward_instanced_render_instances",
        "transparent_render_instances", "transparent_skinned_render_instances",
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
                                            "bool UsesTransparentRasterPass");
  const auto cache_equality = ExtractBetween(
      render_instance_source, "bool RenderInstanceStorage::EmissiveTriangleInstanceSignature::operator==",
      "std::vector<RenderInstanceStorage::EmissiveTriangleInfoBlock>");
  ASSERT_FALSE(semantic_hash.empty());
  ASSERT_FALSE(cache_equality.empty());
  EXPECT_NE(semantic_hash.find("entry.material_handle"), std::string::npos);
  EXPECT_NE(semantic_hash.find("entry.geometry_version"), std::string::npos);
  EXPECT_NE(semantic_hash.find("entry.model.value"), std::string::npos);
  EXPECT_NE(semantic_hash.find("entry.importance"), std::string::npos);
  EXPECT_EQ(semantic_hash.find("material_index"), std::string::npos);
  EXPECT_EQ(semantic_hash.find("instance_index"), std::string::npos);
  EXPECT_EQ(semantic_hash.find("triangle_offset"), std::string::npos);
  EXPECT_NE(cache_equality.find("material_index == other.material_index"), std::string::npos);
  EXPECT_NE(cache_equality.find("instance_index == other.instance_index"), std::string::npos);
  EXPECT_NE(cache_equality.find("triangle_offset == other.triangle_offset"), std::string::npos);
}

TEST(DdgiVolume, DdgiPolicyAndSceneChangesCannotLeaveConvergedHistoryStale) {
  const auto render_layer_source =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "src" / "RenderLayer.cpp");
  const auto policy_block = ExtractBetween(render_layer_source, "const glm::vec4 probe_variability_parameters",
                                           "const auto ddgi_ray_source_common_changed");
  const auto source_change_block = ExtractBetween(render_layer_source, "const auto ddgi_ray_source_common_changed",
                                                  "const auto first_probe_changed");
  const auto source_update_block =
      ExtractBetween(render_layer_source, "if (ddgi_ray_source_common_changed)", "} else if (first_probe_changed)");
  const auto atlas_clear_block = ExtractBetween(
      render_layer_source, "runtime_state.clear_probe_atlas_this_frame = runtime_state.clear_probe_atlas_this_frame ||",
      "const auto probe_variability_enabled");
  const auto full_refresh_block =
      ExtractBetween(render_layer_source, "uint32_t full_refresh_reasons", "runtime_state.last_probe_update_reasons =");
  const auto hard_refresh_block =
      ExtractBetween(render_layer_source, "const bool hard_ddgi_refresh", "const bool transport_refresh");
  const auto variability_reset_block =
      ExtractBetween(render_layer_source, "const bool reset_ddgi_variability_state", "if (!probe_variability_enabled");
  ASSERT_FALSE(policy_block.empty());
  ASSERT_FALSE(source_change_block.empty());
  ASSERT_FALSE(source_update_block.empty());
  ASSERT_FALSE(atlas_clear_block.empty());
  ASSERT_FALSE(full_refresh_block.empty());
  ASSERT_FALSE(hard_refresh_block.empty());
  ASSERT_FALSE(variability_reset_block.empty());
  EXPECT_NE(policy_block.find("probe_variability_threshold"), std::string::npos);
  EXPECT_NE(policy_block.find("probe_variability_min_samples"), std::string::npos);
  EXPECT_NE(policy_block.find("enable_probe_variability"), std::string::npos);
  EXPECT_NE(policy_block.find("enable_probe_variability_gating"), std::string::npos);
  EXPECT_NE(policy_block.find("runtime_state.previous_probe_variability_parameters != probe_variability_parameters"),
            std::string::npos);
  EXPECT_NE(
      source_change_block.find(
          "runtime_state.previous_emissive_mesh_sampling_enabled != runtime_state.emissive_mesh_sampling_enabled"),
      std::string::npos);
  EXPECT_NE(source_update_block.find(
                "runtime_state.previous_emissive_mesh_sampling_enabled = runtime_state.emissive_mesh_sampling_enabled"),
            std::string::npos);
  EXPECT_NE(full_refresh_block.find("if (ddgi_ray_source_changed)"), std::string::npos);
  EXPECT_NE(full_refresh_block.find("full_refresh_reasons |= DdgiUpdateReasonSource"), std::string::npos);
  EXPECT_NE(full_refresh_block.find("if (ddgi_variability_policy_changed)"), std::string::npos);
  EXPECT_NE(full_refresh_block.find("full_refresh_reasons |= DdgiUpdateReasonVariabilityPolicy"), std::string::npos);
  EXPECT_NE(full_refresh_block.find("if (scene_auto_invalidate || scene_readiness_refresh)"), std::string::npos);
  EXPECT_EQ(full_refresh_block.find("ddgi_ray_source_changed || ddgi_variability_policy_changed"), std::string::npos);
  EXPECT_NE(variability_reset_block.find("hard_ddgi_refresh"), std::string::npos);
  EXPECT_NE(variability_reset_block.find("ddgi_variability_policy_changed"), std::string::npos);
  EXPECT_NE(variability_reset_block.find("variability_trigger_refresh"), std::string::npos);
  EXPECT_NE(variability_reset_block.find("ddgi_scroll_clear_this_frame"), std::string::npos);
  EXPECT_EQ(atlas_clear_block.find("ddgi_variability_policy_changed"), std::string::npos);
  EXPECT_NE(atlas_clear_block.find("scene_auto_invalidate"), std::string::npos);
  EXPECT_EQ(hard_refresh_block.find("ddgi_variability_policy_changed"), std::string::npos);
  EXPECT_NE(hard_refresh_block.find("scene_auto_invalidate"), std::string::npos);
  EXPECT_NE(render_layer_source.find("const auto auto_invalidate_triggers ="), std::string::npos);
  EXPECT_NE(render_layer_source.find("DdgiVolumeTriggerConditionGeometryChanged"), std::string::npos);
  EXPECT_NE(render_layer_source.find("scene_auto_invalidate || scene_readiness_refresh ||"), std::string::npos);
  EXPECT_NE(render_layer_source.find("DdgiTriggerConditionEnabled(auto_invalidate_triggers"), std::string::npos);
  EXPECT_NE(render_layer_source.find("if (runtime_state.probe_variability_converged)"), std::string::npos);
  EXPECT_NE(render_layer_source.find("(!runtime_state.probe_variability_converged || periodic_refresh_due)"),
            std::string::npos);
}

TEST(DdgiVolume, DdgiSceneChangeLatchesSurvivePauseUntilAProbeTraceRuns) {
  const auto render_layer_source =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "src" / "RenderLayer.cpp");
  const auto pause_block = ExtractBetween(render_layer_source, "if (ddgi_settings.runtime.pause_updates)",
                                          "runtime_state.frame_resource_layout = layout;");
  ASSERT_FALSE(pause_block.empty());
  EXPECT_EQ(pause_block.find("runtime_state.latched_scene_change_triggers ="), std::string::npos);
  EXPECT_EQ(pause_block.find("runtime_state.latched_scene_geometry_changed ="), std::string::npos);
  EXPECT_EQ(pause_block.find("reset_probe_history = false"), std::string::npos);
  EXPECT_NE(pause_block.find("track_ddgi_environment_signature()"), std::string::npos);
  EXPECT_EQ(pause_block.find("runtime_state.previous_emissive_mesh_sampling_enabled ="), std::string::npos);
  const auto resolve_effective = render_layer_source.find(
      "runtime_state.emissive_mesh_sampling_enabled = ddgi_ray_source.emissive_mesh_sampling_enabled;");
  const auto pause_branch = render_layer_source.find("if (ddgi_settings.runtime.pause_updates)", resolve_effective);
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
  EXPECT_NE(render_layer_source.find("runtime->latched_scene_geometry_changed |= ddgi_latched_scene_geometry_changed_"),
            std::string::npos);
  EXPECT_NE(render_layer_source.find("ddgi_latched_scene_change_triggers_ |= scene_change_triggers"),
            std::string::npos);
  EXPECT_NE(render_layer_source.find("ddgi_latched_scene_geometry_changed_ |= probe_state_geometry_changed"),
            std::string::npos);
  EXPECT_EQ(
      CountOccurrences(render_layer_source, "ddgi_latched_scene_change_triggers_ = DdgiVolumeTriggerConditionNone;"),
      1u);
  EXPECT_EQ(CountOccurrences(render_layer_source, "ddgi_latched_scene_geometry_changed_ = false;"), 1u);
  const auto no_trace_return = render_layer_source.find("if (!should_trace_probe_rays) {");
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
  const auto clear_reset = prepare_set.find("assigned_lighting->ddgi_settings.runtime.reset_probe_history = false;");
  ASSERT_NE(latch_reset, std::string::npos);
  ASSERT_NE(consume_reset, std::string::npos);
  ASSERT_NE(clear_reset, std::string::npos);
  EXPECT_LT(latch_reset, consume_reset);
  EXPECT_LT(consume_reset, clear_reset);
  EXPECT_NE(prepare_set.find("info.sorted_index, runtime->manual_reset_pending"), std::string::npos);
  EXPECT_NE(prepare_set.find("if (runtime->frame_trace_probe_rays)"), std::string::npos);
  EXPECT_NE(prepare_set.find("if (assigned_lighting && (!reset_probe_history || !infos.empty()))"), std::string::npos);
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
  EXPECT_NE(render_layer_source.find("scene_auto_invalidate || scene_readiness_refresh"), std::string::npos);
  const auto atlas_clear = ExtractBetween(
      render_layer_source, "runtime_state.clear_probe_atlas_this_frame = runtime_state.clear_probe_atlas_this_frame ||",
      "const auto probe_variability_enabled");
  const auto refresh_classification =
      ExtractBetween(render_layer_source, "const bool hard_ddgi_refresh", "if (transport_refresh)");
  const auto forced_trace =
      ExtractBetween(render_layer_source, "const bool forced_probe_trace", "const bool should_trace_probe_rays");
  ASSERT_FALSE(atlas_clear.empty());
  ASSERT_FALSE(refresh_classification.empty());
  ASSERT_FALSE(forced_trace.empty());
  EXPECT_EQ(atlas_clear.find("scene_readiness_refresh"), std::string::npos);
  const auto transport_refresh = refresh_classification.find("const bool transport_refresh");
  ASSERT_NE(transport_refresh, std::string::npos);
  EXPECT_EQ(refresh_classification.substr(0, transport_refresh).find("scene_readiness_refresh"), std::string::npos);
  EXPECT_NE(refresh_classification.substr(transport_refresh).find("scene_readiness_refresh"), std::string::npos);
  EXPECT_NE(forced_trace.find("scene_readiness_refresh"), std::string::npos);
  EXPECT_NE(render_layer_source.find("hard_ddgi_refresh || scene_probe_state_reset"), std::string::npos);
  const auto readback_allocation =
      ExtractBetween(render_layer_source, "if (!ddgi_variability_readback_buffer", "if (!ShouldTraceDdgiProbeRays");
  ASSERT_FALSE(readback_allocation.empty());
  EXPECT_EQ(readback_allocation.find("ddgi_persistent_resource_changed = true"), std::string::npos);
  EXPECT_NE(refresh_classification.find("ddgi_persistent_resource_changed"), std::string::npos);
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
      "PrepareSceneForRendering(scene, false, false, false, false, reflection_probe_capture ? &injected_camera : "
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

TEST(DdgiVolume, OffscreenPreviewThumbnailUploadCompletesBeforeReturn) {
  const auto source_root = std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK";
  const auto preview_source = ReadTextFile(source_root / "src" / "OffscreenPreviewRenderer.cpp");
  ASSERT_FALSE(preview_source.empty());

  const auto read_texture =
      preview_source.find("std::shared_ptr<Texture2D> OffscreenPreviewRenderer::ReadColorTexture");
  const auto set_data = preview_source.find("texture->SetRgbaChannelData(pixels, resolution, false);", read_texture);
  const auto upload = preview_source.find("texture->UnsafeUploadDataImmediately();", set_data);
  const auto return_texture = preview_source.find("return texture;", set_data);
  ASSERT_NE(read_texture, std::string::npos);
  ASSERT_NE(set_data, std::string::npos);
  ASSERT_NE(upload, std::string::npos);
  ASSERT_NE(return_texture, std::string::npos);
  EXPECT_LT(set_data, upload);
  EXPECT_LT(upload, return_texture);
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
  const auto upload = render_layer_source.find("current_render_instances->Upload();", prepare);
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
  const auto raster_material_source = ReadTextFile(shader_root / "Modules" / "EvoEngine" / "GltfRasterMaterial.slang");
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

TEST(DdgiVolume, PunctualShadowFilteringUsesFixedPcf) {
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
  EXPECT_NE(lighting_source.find("clamp(EE_RENDER_INFO.shadow_sample_size, 1, 64)"), std::string::npos);
  EXPECT_EQ(lighting_source.find("BLOCKER_SEARCH"), std::string::npos);
  EXPECT_EQ(lighting_source.find("penumbraWidth"), std::string::npos);
  EXPECT_EQ(lighting_source.find("blockerDistance"), std::string::npos);
}

TEST(DdgiVolume, DdgiProbeHitsExplicitlyEvaluateAnalyticSceneLights) {
  const auto shader_root =
      std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "Internals" / "DefaultResources" / "Shaders";
  const auto closest_hit_source =
      ReadTextFile(shader_root / "RayTracing" / "ClosestHit" / "DDGIProbeDiagnostics.slang");
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
  EXPECT_NE(closest_hit_source.find("EE_DIRECTIONAL_LIGHTS[i], albedo, normal, geometric_normal, position"),
            std::string::npos);
  EXPECT_NE(closest_hit_source.find("EE_POINT_LIGHTS[i], albedo, normal, geometric_normal, position"),
            std::string::npos);
  EXPECT_NE(closest_hit_source.find("EE_SPOT_LIGHTS[i], albedo, normal, geometric_normal, position"),
            std::string::npos);
  EXPECT_NE(closest_hit_source.find("light.diffuse.w == 1.0f"), std::string::npos);
  EXPECT_NE(closest_hit_source.find("EE_RT_OFFSET_RAY_ORIGIN(position, geometric_normal, light_direction)"),
            std::string::npos);
  EXPECT_EQ(closest_hit_source.find("position + normal * trace_parameters.y"), std::string::npos);
}

TEST(DdgiVolume, DdgiProbeHitsSampleSharedEmissiveTrianglesWithoutMis) {
  const auto shader_root =
      std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "Internals" / "DefaultResources" / "Shaders";
  const auto shared_sampling = ReadTextFile(shader_root / "Modules" / "EvoEngine" / "EmissiveTriangleSampling.slang");
  const auto raygen = ReadTextFile(shader_root / "RayTracing" / "RayGen" / "DDGIProbeDiagnostics.slang");
  const auto closest_hit = ReadTextFile(shader_root / "RayTracing" / "ClosestHit" / "DDGIProbeDiagnostics.slang");
  ASSERT_FALSE(shared_sampling.empty());
  ASSERT_FALSE(raygen.empty());
  ASSERT_FALSE(closest_hit.empty());

  EXPECT_NE(shared_sampling.find("EE_SAMPLE_EMISSIVE_TRIANGLE_RECORD(record_sample, uniform_sampling)"),
            std::string::npos);
  EXPECT_NE(shared_sampling.find("EE_GLTF_RASTER_ALPHA_MASK_PASSES_LOD0"), std::string::npos);
  EXPECT_NE(shared_sampling.find("max(radiance, float3(0.0f)) / solid_angle_pdf"), std::string::npos);
  EXPECT_NE(raygen.find("uint(EE_DDGI_PROBE_RAY_CONSTANTS.probe_scroll_offset.w), probe_index, ray_index"),
            std::string::npos);
  EXPECT_NE(raygen.find("fixed_ray ? EE_DDGI_FIXED_RAY_PAYLOAD_FLAG : primary_ray_seed"), std::string::npos);
  EXPECT_NE(raygen.find("(EE_DDGI_PROBE_RAY_CONSTANTS.selected_probe_volume_flags_environment.z & (1u << 0u)) != 0u"),
            std::string::npos);
  EXPECT_NE(closest_hit.find("0x68bc21ebu"), std::string::npos);
  EXPECT_NE(closest_hit.find("0x967a889bu"), std::string::npos);
  EXPECT_NE(closest_hit.find("0x1b56c4e9u"), std::string::npos);
  EXPECT_NE(closest_hit.find("EE_DDGI_EMISSIVE_MESH_IRRADIANCE("), std::string::npos);
  const auto direct_hit = closest_hit.find("const float3 emissive_radiance = EE_RT_COATED_EMISSION(");
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
  EXPECT_NE(probe_scroll_source.find("EE_DDGI_CLEAR_VARIABILITY_TILE"), std::string::npos);
  EXPECT_NE(atlas_prepare_source.find("RenderResourceState::TransferDestinationGeneral"), std::string::npos);
  EXPECT_NE(render_layer_source.find("if (clear_ddgi_probe_atlas)"), std::string::npos);
  EXPECT_NE(render_layer_source.find("DdgiVolumeMovementType::Scrolling"), std::string::npos);
  EXPECT_NE(render_layer_source.find("CalculateDdgiEffectiveFirstProbe"), std::string::npos);
  EXPECT_NE(render_layer_source.find("NormalizeDdgiProbeScrollOrigin"), std::string::npos);
  EXPECT_NE(render_layer_source.find("complete_wraps * counts[axis]"), std::string::npos);
  EXPECT_NE(render_layer_source.find("RequiresDdgiFullScrollReset(ddgi_ray_source.probe_counts"), std::string::npos);
  const auto scroll_pass_position = render_layer_source.find("DdgiProbeScrollPass::CreateDescriptor()");
  const auto ray_pass_position = render_layer_source.find("DdgiRayDiagnosticsPass::CreateDescriptor()");
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
  const auto ray_source = ReadTextFile(source_root / "RenderPasses" / "DdgiRayDiagnosticsPass.cpp");
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
  const auto ray = ReadTextFile(pass_root / "DdgiRayDiagnosticsPass.cpp");
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
  EXPECT_NE(update.find("DDGI Irradiance Update"), std::string::npos);
  EXPECT_NE(update.find("DDGI Visibility Update"), std::string::npos);
  EXPECT_NE(update.find("EVOENGINE_DDGI_PROBE_UPDATE_PATH executed="), std::string::npos);
  EXPECT_EQ(update.find("EverythingBarrier"), std::string::npos);
  for (const auto* file :
       {"DdgiRayDiagnosticsPass.cpp", "DdgiProbeRelocationPass.cpp", "DdgiProbeClassificationPass.cpp",
        "DdgiProbeVariabilityPass.cpp", "DdgiProbeScrollPass.cpp"}) {
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
  const auto raygen_source = ReadTextFile(shader_root / "RayTracing" / "RayGen" / "DDGIProbeDiagnostics.slang");
  const auto closest_hit_source =
      ReadTextFile(shader_root / "RayTracing" / "ClosestHit" / "DDGIProbeDiagnostics.slang");
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
  RenderLayer::DdgiSettings settings;
  settings.storage.max_probe_count = 1000;
  settings.storage.atlas_probe_columns = 5;
  settings.storage.irradiance_tile_resolution = 6;
  settings.storage.visibility_tile_resolution = 14;
  settings.runtime.ray_count = 11;

  auto layout = RenderLayer::CalculateDdgiFrameResourceLayout(settings, 37);

  EXPECT_EQ(layout.probe_count, 37u);
  EXPECT_EQ(layout.irradiance_atlas.columns, 5u);
  EXPECT_EQ(layout.irradiance_atlas.rows, 8u);
  EXPECT_EQ(layout.irradiance_atlas.resolution, glm::uvec2(40, 64));
  EXPECT_EQ(layout.visibility_atlas.resolution, glm::uvec2(80, 128));
  EXPECT_EQ(layout.variability_atlas.resolution, glm::uvec2(30, 48));
  EXPECT_EQ(layout.variability_reduction_extent, glm::uvec2(2, 3));
  EXPECT_EQ(layout.probe_metadata_byte_size, 37ull * sizeof(glm::vec4) * 3ull);
  EXPECT_EQ(layout.probe_state_byte_size, 37ull * sizeof(glm::vec4));
  EXPECT_EQ(layout.ray_output_byte_size, 37ull * 11ull * sizeof(DdgiProbeRayData));
  EXPECT_EQ(layout.selected_ray_diagnostics_byte_size, 11ull * sizeof(PointCloudSample));

  settings.runtime.ray_count = 23;
  auto resized_layout = RenderLayer::CalculateDdgiFrameResourceLayout(settings, 37);
  EXPECT_EQ(resized_layout.irradiance_atlas.resolution, layout.irradiance_atlas.resolution);
  EXPECT_EQ(resized_layout.visibility_atlas.resolution, layout.visibility_atlas.resolution);
  EXPECT_EQ(resized_layout.variability_atlas.resolution, layout.variability_atlas.resolution);
  EXPECT_EQ(resized_layout.variability_reduction_extent, layout.variability_reduction_extent);
  EXPECT_EQ(resized_layout.ray_output_byte_size, 37ull * 23ull * sizeof(DdgiProbeRayData));
  EXPECT_EQ(resized_layout.selected_ray_diagnostics_byte_size, 23ull * sizeof(PointCloudSample));

  settings.storage.irradiance_tile_resolution = 10;
  settings.storage.visibility_tile_resolution = 18;
  resized_layout = RenderLayer::CalculateDdgiFrameResourceLayout(settings, 37);
  EXPECT_EQ(resized_layout.irradiance_atlas.resolution, glm::uvec2(60, 96));
  EXPECT_EQ(resized_layout.visibility_atlas.resolution, glm::uvec2(100, 160));
  EXPECT_EQ(resized_layout.variability_atlas.resolution, glm::uvec2(50, 80));
  EXPECT_EQ(resized_layout.variability_reduction_extent, glm::uvec2(4, 5));
  EXPECT_EQ(resized_layout.ray_output_byte_size, 37ull * 23ull * sizeof(DdgiProbeRayData));
  EXPECT_EQ(resized_layout.selected_ray_diagnostics_byte_size, 23ull * sizeof(PointCloudSample));
}

TEST(DdgiVolume, RenderLayerVolumeBlendWeightMatchesRtxgiVolumeCoverage) {
  const auto probe_counts = glm::ivec3(5, 5, 5);
  const auto probe_step_lengths = glm::vec3(2.0f);

  EXPECT_FLOAT_EQ(RenderLayer::CalculateDdgiVolumeBlendWeight({2.0f, 2.0f, 2.0f}, probe_counts, probe_step_lengths),
                  1.0f);
  EXPECT_FLOAT_EQ(RenderLayer::CalculateDdgiVolumeBlendWeight({0.0f, 2.0f, 2.0f}, probe_counts, probe_step_lengths),
                  1.0f);
  EXPECT_FLOAT_EQ(RenderLayer::CalculateDdgiVolumeBlendWeight({4.0f, 2.0f, 2.0f}, probe_counts, probe_step_lengths),
                  1.0f);
  EXPECT_NEAR(RenderLayer::CalculateDdgiVolumeBlendWeight({-0.5f, 2.0f, 2.0f}, probe_counts, probe_step_lengths), 0.5f,
              0.0001f);
  EXPECT_FLOAT_EQ(RenderLayer::CalculateDdgiVolumeBlendWeight({-1.0f, 2.0f, 2.0f}, probe_counts, probe_step_lengths),
                  0.0f);
  EXPECT_FLOAT_EQ(RenderLayer::CalculateDdgiVolumeBlendWeight({-0.1f, 2.0f, 2.0f}, probe_counts, probe_step_lengths),
                  0.9f);
}

TEST(DdgiVolume, MultiVolumeBudgetAcceptsExactLimitsAndRejectsLimitPlusOneAtomically) {
  std::vector<RenderLayer::DdgiVolumeRuntimeInfo> infos(8);
  for (size_t i = 0; i < infos.size(); ++i) {
    infos[i].stable_entity_id = i + 1u;
    infos[i].probe_count = 1024u;
  }

  const auto exact = RenderLayer::ValidateDdgiVolumeSet(infos, 8192u);
  EXPECT_TRUE(exact.valid);
  EXPECT_EQ(exact.aggregate_probe_count, 8192u);

  auto too_many_volumes = infos;
  too_many_volumes.push_back({});
  too_many_volumes.back().stable_entity_id = 9u;
  too_many_volumes.back().probe_count = 1u;
  const auto volume_overflow = RenderLayer::ValidateDdgiVolumeSet(too_many_volumes, 8192u);
  EXPECT_FALSE(volume_overflow.valid);
  EXPECT_NE(volume_overflow.error.find("maximum is 8"), std::string::npos);

  auto too_many_probes = infos;
  ++too_many_probes.back().probe_count;
  const auto probe_overflow = RenderLayer::ValidateDdgiVolumeSet(too_many_probes, 8192u);
  EXPECT_FALSE(probe_overflow.valid);
  EXPECT_EQ(probe_overflow.aggregate_probe_count, 8193u);
  EXPECT_NE(probe_overflow.error.find("8193 resident probes"), std::string::npos);

  const auto configured_overflow = RenderLayer::ValidateDdgiVolumeSet(infos, 4096u);
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

  EnvironmentalLighting::DdgiVolume first;
  first.stable_id = 11u;
  first.probe_counts = {4, 4, 4};
  EnvironmentalLighting::DdgiVolume second;
  second.stable_id = 22u;
  second.probe_counts = {5, 5, 5};
  lighting->ddgi_volumes = {first, second};
  lighting->ddgi_settings.runtime.pause_updates = true;

  RenderLayer::DdgiSettings settings;
  auto infos = RenderLayer::CollectDdgiVolumeRuntimeInfos(scene, settings);
  EXPECT_EQ(infos.size(), 2u);

  lighting->ddgi_volumes[1].enabled = false;
  infos = RenderLayer::CollectDdgiVolumeRuntimeInfos(scene, settings);
  ASSERT_EQ(infos.size(), 1u);
  EXPECT_EQ(infos.front().stable_entity_id, first.stable_id);

  lighting->ddgi_volumes[1].enabled = true;
  lighting->ddgi_volumes[1].probe_counts = {0, 5, 5};
  infos = RenderLayer::CollectDdgiVolumeRuntimeInfos(scene, settings);
  EXPECT_EQ(infos.size(), 1u);
}

TEST(DdgiVolume, MultiVolumeOrderingUsesPriorityDensityThenStableEntityId) {
  RenderLayer::DdgiVolumeRuntimeInfo low_priority;
  low_priority.stable_entity_id = 1u;
  low_priority.artist_priority = 0;
  low_priority.probe_density = 100.0f;
  RenderLayer::DdgiVolumeRuntimeInfo high_priority = low_priority;
  high_priority.stable_entity_id = 3u;
  high_priority.artist_priority = 1;
  high_priority.probe_density = 0.01f;
  RenderLayer::DdgiVolumeRuntimeInfo density_tie_break = low_priority;
  density_tie_break.stable_entity_id = 2u;
  density_tie_break.probe_density = 200.0f;

  std::vector infos{low_priority, high_priority, density_tie_break};
  RenderLayer::SortDdgiVolumeRuntimeInfos(infos);
  ASSERT_EQ(infos.size(), 3u);
  EXPECT_EQ(infos[0].stable_entity_id, 3u);
  EXPECT_EQ(infos[1].stable_entity_id, 2u);
  EXPECT_EQ(infos[2].stable_entity_id, 1u);
  EXPECT_EQ(infos[0].sorted_index, 0u);
  EXPECT_EQ(infos[2].sorted_index, 2u);

  density_tie_break.probe_density = low_priority.probe_density;
  infos = {density_tie_break, low_priority};
  RenderLayer::SortDdgiVolumeRuntimeInfos(infos);
  EXPECT_EQ(infos[0].stable_entity_id, 1u);
  EXPECT_EQ(infos[1].stable_entity_id, 2u);

  EXPECT_FLOAT_EQ(RenderLayer::CalculateDdgiProbeDensity({2.0f, 0.0f, 0.0f}, {0.0f, 3.0f, 0.0f}, {0.0f, 0.0f, 4.0f}),
                  1.0f / 24.0f);
  EXPECT_FLOAT_EQ(RenderLayer::CalculateDdgiProbeDensity({}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}), 0.0f);
}

TEST(DdgiVolume, MultiVolumeSelectionIsDeterministicAndUsesAtMostOneBoundarySecondary) {
  const auto make_info = [](const uint64_t stable_id, const int priority, const glm::vec3 first_probe) {
    RenderLayer::DdgiVolumeRuntimeInfo info;
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

  const auto deep_overlap = RenderLayer::SelectDdgiVolumes({third, secondary, primary}, {2.0f, 2.0f, 2.0f});
  EXPECT_TRUE(deep_overlap.valid);
  EXPECT_EQ(deep_overlap.primary_entity_id, 10u);
  EXPECT_EQ(deep_overlap.secondary_entity_id, 0u);
  EXPECT_FLOAT_EQ(deep_overlap.primary_weight, 1.0f);
  EXPECT_FLOAT_EQ(deep_overlap.ibl_weight, 0.0f);

  const auto boundary = RenderLayer::SelectDdgiVolumes({third, primary, secondary}, {0.5f, 2.0f, 2.0f});
  EXPECT_EQ(boundary.primary_entity_id, 10u);
  EXPECT_EQ(boundary.secondary_entity_id, 20u);
  EXPECT_FLOAT_EQ(boundary.primary_weight + boundary.secondary_weight, 1.0f);
  EXPECT_NEAR(boundary.primary_weight, 2.0f / 3.0f, 0.0001f);
  EXPECT_NEAR(boundary.secondary_weight, 1.0f / 3.0f, 0.0001f);
  EXPECT_FLOAT_EQ(boundary.ibl_weight, 0.0f);

  const auto near_boundary = RenderLayer::SelectDdgiVolumes({secondary, primary}, {0.99f, 2.0f, 2.0f});
  EXPECT_EQ(near_boundary.secondary_entity_id, 20u);
  EXPECT_NEAR(near_boundary.secondary_weight, 0.01f / 1.01f, 0.0001f);
  const auto exact_boundary = RenderLayer::SelectDdgiVolumes({secondary, primary}, {1.0f, 2.0f, 2.0f});
  EXPECT_EQ(exact_boundary.secondary_entity_id, 0u);
  EXPECT_LT(std::abs(exact_boundary.primary_weight - near_boundary.primary_weight), 0.011f);
  const auto beyond_boundary = RenderLayer::SelectDdgiVolumes({secondary, primary}, {1.01f, 2.0f, 2.0f});
  EXPECT_EQ(beyond_boundary.secondary_entity_id, 0u);

  auto reversed_infos = std::vector{third, secondary, primary};
  std::reverse(reversed_infos.begin(), reversed_infos.end());
  const auto reversed = RenderLayer::SelectDdgiVolumes(reversed_infos, {0.5f, 2.0f, 2.0f});
  EXPECT_EQ(reversed.primary_entity_id, boundary.primary_entity_id);
  EXPECT_EQ(reversed.secondary_entity_id, boundary.secondary_entity_id);
  EXPECT_FLOAT_EQ(reversed.primary_weight, boundary.primary_weight);
  EXPECT_FLOAT_EQ(reversed.secondary_weight, boundary.secondary_weight);

  const auto faded = RenderLayer::SelectDdgiVolumes({primary}, {-0.5f, 2.0f, 2.0f});
  EXPECT_EQ(faded.primary_entity_id, 10u);
  EXPECT_EQ(faded.secondary_entity_id, 0u);
  EXPECT_FLOAT_EQ(faded.primary_weight, 0.5f);
  EXPECT_FLOAT_EQ(faded.ibl_weight, 0.5f);

  const auto disjoint = make_info(40u, 5, {10.0f, 0.0f, 0.0f});
  const auto selected_disjoint = RenderLayer::SelectDdgiVolumes({primary, disjoint}, {12.0f, 2.0f, 2.0f});
  EXPECT_EQ(selected_disjoint.primary_entity_id, 40u);
  EXPECT_EQ(selected_disjoint.secondary_entity_id, 0u);
  EXPECT_FALSE(RenderLayer::SelectDdgiVolumes({primary, disjoint}, {7.0f, 2.0f, 2.0f}).valid);

  const auto nested = make_info(50u, 3, {1.0f, 1.0f, 1.0f});
  const auto selected_nested = RenderLayer::SelectDdgiVolumes({primary, nested}, {3.0f, 3.0f, 3.0f});
  EXPECT_EQ(selected_nested.primary_entity_id, 50u);
  EXPECT_EQ(selected_nested.secondary_entity_id, 0u);
  EXPECT_FLOAT_EQ(selected_nested.primary_weight, 1.0f);

  const auto outer = make_info(60u, 0, {0.0f, 0.0f, 0.0f});
  auto inner = make_info(70u, 1, {1.0f, 1.0f, 1.0f});
  inner.probe_counts = {3, 3, 3};
  inner.probe_count = 27u;
  const auto just_inside = RenderLayer::SelectDdgiVolumes({outer, inner}, {1.001f, 2.0f, 2.0f});
  const auto exact_crossing = RenderLayer::SelectDdgiVolumes({outer, inner}, {1.0f, 2.0f, 2.0f});
  const auto just_outside = RenderLayer::SelectDdgiVolumes({outer, inner}, {0.999f, 2.0f, 2.0f});
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
  const auto exterior_boundary = RenderLayer::SelectDdgiVolumes({deep_outer, approaching_inner}, {3.5f, 5.0f, 5.0f});
  EXPECT_EQ(exterior_boundary.primary_entity_id, 76u);
  EXPECT_EQ(exterior_boundary.secondary_entity_id, 75u);
  EXPECT_NEAR(exterior_boundary.primary_weight, 1.0f / 3.0f, 0.0001f);
  EXPECT_NEAR(exterior_boundary.secondary_weight, 2.0f / 3.0f, 0.0001f);

  auto warming_inner = inner;
  warming_inner.contributes_lighting = false;
  const auto warming_fallback = RenderLayer::SelectDdgiVolumes({warming_inner, outer}, {2.0f, 2.0f, 2.0f});
  EXPECT_EQ(warming_fallback.primary_entity_id, 60u);
  EXPECT_EQ(warming_fallback.secondary_entity_id, 0u);
  EXPECT_FLOAT_EQ(warming_fallback.primary_weight, 1.0f);

  const auto fade_a = make_info(80u, 1, {0.0f, 0.0f, 0.0f});
  const auto fade_b = make_info(90u, 0, {5.5f, 0.0f, 0.0f});
  const auto before_fade_overlap = RenderLayer::SelectDdgiVolumes({fade_b, fade_a}, {4.5f, 2.0f, 2.0f});
  const auto after_fade_overlap = RenderLayer::SelectDdgiVolumes({fade_b, fade_a}, {4.5001f, 2.0f, 2.0f});
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
  const auto closest_hit_source =
      ReadTextFile(shader_root / "RayTracing" / "ClosestHit" / "DDGIProbeDiagnostics.slang");
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
  EXPECT_NE(render_storage_header.find("static_assert(sizeof(RenderInfoBlock) == 5520)"), std::string::npos);
  EXPECT_EQ(render_storage_header.find("glm::vec4 ddgi_first_probe"), std::string::npos);
  EXPECT_NE(render_layer_header.find("std::unordered_map<uint64_t, std::unique_ptr<DdgiVolumeRuntimeState>>"),
            std::string::npos);
  EXPECT_NE(render_layer_source.find("for (uint32_t slot = 0; slot < RenderInstanceStorage::kDdgiMaxVolumeCount"),
            std::string::npos);
  EXPECT_NE(render_layer_source.find("execute_ddgi_runtime(*runtime->second"), std::string::npos);
  EXPECT_NE(render_layer_source.find("current_frame_transient_resources.emplace_back()"), std::string::npos);
  EXPECT_NE(render_layer_source.find("include_external_passes"), std::string::npos);
  EXPECT_NE(render_layer_source.find("for (size_t i = ddgi_ordered_volume_ids_.size(); i-- > 0u;)"), std::string::npos);
  EXPECT_NE(render_layer_source.find("static_cast<uint32_t>(i), i == 0u"), std::string::npos);
  EXPECT_NE(render_layer_source.find("ddgi_volumes_complete.dependencies.push_back(pass.name)"), std::string::npos);
  EXPECT_NE(render_layer_source.find("descriptor.dependencies.emplace_back(RenderPassNames::ddgi_volumes_complete)"),
            std::string::npos);
  EXPECT_EQ(render_layer_source.find("Required DDGI Volume Removal Fence Wait"), std::string::npos);
  EXPECT_NE(render_layer_source.find("ResetDdgiRuntimeFrameState(*runtime);"), std::string::npos);
  EXPECT_NE(render_layer_source.find("if (!ddgi_volume_runtime_states_.empty())"), std::string::npos);
  EXPECT_EQ(render_layer_source.find("ddgi_debug_volume_id_"), std::string::npos);
  EXPECT_NE(render_layer_source.find("const bool prepare_debug_data = sorted_index == 0u"), std::string::npos);
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
  EXPECT_NE(platform_source.find("frame_totals[scope.name] +="), std::string::npos);
  EXPECT_NE(inspection_source.find("irradiance_coordinates.grid_index), active_probe_counts"), std::string::npos);
  EXPECT_NE(inspection_source.find("probe_scroll_offset, active_probe_counts"), std::string::npos);
  EXPECT_NE(ddgi_runtime_header.find("bool emissive_mesh_sampling_enabled = false;"), std::string::npos);
  EXPECT_NE(render_layer_source.find("state.emissive_mesh_sampling_enabled, state.resource_ids"), std::string::npos);
  EXPECT_NE(render_layer_source.find("aggregate.emissive_triangle_count ="), std::string::npos);
  EXPECT_NE(
      render_layer_source.find("glm::max(aggregate.emissive_triangle_count, volume_stats.emissive_triangle_count)"),
      std::string::npos);
  EXPECT_NE(render_layer_source.find("aggregate.emissive_sampling_enabled_volume_count +="), std::string::npos);
  EXPECT_NE(render_layer_source.find("aggregate.emissive_sampling_candidate_ray_count +="), std::string::npos);
  EXPECT_NE(render_layer_source.find("render_info_block.emissive_triangle_parameters.x"), std::string::npos);
  EXPECT_NE(render_layer_source.find("CalculateDdgiEmissiveSamplingCandidateRayCount("), std::string::npos);
  EXPECT_NE(inspection_source.find("DDGI emissive candidate rays (upper bound)"), std::string::npos);
  EXPECT_NE(inspection_source.find("Primary emissive sampling: %s"), std::string::npos);
}

TEST(DdgiVolume, RenderLayerDdgiUpdatePolicyContracts) {
  RenderLayer::DdgiSettings settings;
  settings.runtime.hysteresis = 0.97f;

  EXPECT_FLOAT_EQ(RenderLayer::CalculateDdgiUpdateHysteresis(settings, RenderLayer::DdgiUpdateReasonSteadyState),
                  0.97f);
  EXPECT_FLOAT_EQ(RenderLayer::CalculateDdgiUpdateHysteresis(
                      settings, RenderLayer::DdgiUpdateReasonSource | RenderLayer::DdgiUpdateReasonManualReset),
                  0.0f);
  EXPECT_FLOAT_EQ(RenderLayer::CalculateDdgiUpdateHysteresis(settings, RenderLayer::DdgiUpdateReasonSource), 0.0f);
  EXPECT_FLOAT_EQ(RenderLayer::CalculateDdgiUpdateHysteresis(settings, RenderLayer::DdgiUpdateReasonSceneInput), 0.0f);
  EXPECT_FLOAT_EQ(RenderLayer::CalculateDdgiUpdateHysteresis(settings, RenderLayer::DdgiUpdateReasonVariabilityPolicy),
                  0.0f);
  EXPECT_FLOAT_EQ(RenderLayer::CalculateDdgiUpdateHysteresis(
                      settings, RenderLayer::DdgiUpdateReasonSteadyState | RenderLayer::DdgiUpdateReasonWarmup, 0),
                  0.0f);
  EXPECT_NEAR(RenderLayer::CalculateDdgiUpdateHysteresis(
                  settings, RenderLayer::DdgiUpdateReasonSteadyState | RenderLayer::DdgiUpdateReasonWarmup, 8),
              0.97f * (8.0f / 15.0f), 0.0001f);
  EXPECT_FLOAT_EQ(RenderLayer::CalculateDdgiUpdateHysteresis(
                      settings, RenderLayer::DdgiUpdateReasonSteadyState | RenderLayer::DdgiUpdateReasonWarmup, 15),
                  0.97f);
  EXPECT_FLOAT_EQ(RenderLayer::CalculateDdgiUpdateHysteresis(
                      settings, RenderLayer::DdgiUpdateReasonSteadyState | RenderLayer::DdgiUpdateReasonWarmup, 16),
                  0.97f);
  EXPECT_FLOAT_EQ(RenderLayer::CalculateDdgiUpdateHysteresis(
                      settings, RenderLayer::DdgiUpdateReasonSource | RenderLayer::DdgiUpdateReasonWarmup, 8),
                  0.0f);
  EXPECT_FLOAT_EQ(
      RenderLayer::CalculateDdgiUpdateHysteresis(
          settings, RenderLayer::DdgiUpdateReasonVariabilityPolicy | RenderLayer::DdgiUpdateReasonWarmup, 8),
      0.0f);

  settings.runtime.warmup_frames = 0;
  EXPECT_FLOAT_EQ(RenderLayer::CalculateDdgiUpdateHysteresis(
                      settings, RenderLayer::DdgiUpdateReasonSteadyState | RenderLayer::DdgiUpdateReasonWarmup, 0),
                  0.97f);

  settings.runtime.brightness_threshold = 0.1f;
  EXPECT_FLOAT_EQ(RenderLayer::CalculateDdgiUpdateBrightnessThreshold(settings), 0.1f);
  settings.runtime.brightness_threshold = -0.1f;
  EXPECT_FLOAT_EQ(RenderLayer::CalculateDdgiUpdateBrightnessThreshold(settings), 0.0f);
  settings.runtime.brightness_threshold = 1.1f;
  EXPECT_FLOAT_EQ(RenderLayer::CalculateDdgiUpdateBrightnessThreshold(settings), 1.0f);
}

TEST(DdgiVolume, RenderLayerIsolatesFirstWarmupFrameFromDdgiHistory) {
  const auto render_layer_source =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "src" / "RenderLayer.cpp");
  const auto ray_hit_source =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "Internals" /
                   "DefaultResources" / "Shaders" / "RayTracing" / "ClosestHit" / "DDGIProbeDiagnostics.slang");
  ASSERT_FALSE(render_layer_source.empty());
  ASSERT_FALSE(ray_hit_source.empty());

  EXPECT_NE(render_layer_source.find("runtime_state.frame_probe_warmup_active &&"), std::string::npos);
  EXPECT_NE(render_layer_source.find("runtime_state.probe_warmup_frame_index == 0u"), std::string::npos);
  const auto first_warmup_brightness =
      ExtractBetween(render_layer_source, "const auto ddgi_update_brightness_threshold",
                     "runtime_state.frame_probe_update_hysteresis");
  ASSERT_FALSE(first_warmup_brightness.empty());
  EXPECT_NE(first_warmup_brightness.find("ddgi_first_warmup_frame"), std::string::npos);
  EXPECT_NE(first_warmup_brightness.find("(std::numeric_limits<float>::max)()"), std::string::npos);
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
  EXPECT_EQ(RenderLayer::DdgiUpdateReasonSource, 1u << 0u);
  EXPECT_EQ(RenderLayer::DdgiUpdateReasonManualReset, 1u << 1u);
  EXPECT_EQ(RenderLayer::DdgiUpdateReasonSteadyState, 1u << 2u);
  EXPECT_EQ(RenderLayer::DdgiUpdateReasonConverged, 1u << 3u);
  EXPECT_EQ(RenderLayer::DdgiUpdateReasonWarmup, 1u << 4u);
  EXPECT_EQ(RenderLayer::DdgiUpdateReasonSceneInput, 1u << 5u);
  EXPECT_EQ(RenderLayer::DdgiUpdateReasonPeriodicRefresh, 1u << 6u);
  EXPECT_EQ(RenderLayer::DdgiUpdateReasonVariabilityPolicy, 1u << 7u);
  EXPECT_EQ(RenderLayer::FormatDdgiUpdateReasons(RenderLayer::DdgiUpdateReasonNone), "None");
  EXPECT_EQ(RenderLayer::FormatDdgiUpdateReasons(
                RenderLayer::DdgiUpdateReasonSource | RenderLayer::DdgiUpdateReasonManualReset |
                RenderLayer::DdgiUpdateReasonSteadyState | RenderLayer::DdgiUpdateReasonConverged |
                RenderLayer::DdgiUpdateReasonWarmup | RenderLayer::DdgiUpdateReasonSceneInput |
                RenderLayer::DdgiUpdateReasonPeriodicRefresh | RenderLayer::DdgiUpdateReasonVariabilityPolicy),
            "DDGI source, Manual reset, Steady state, Converged, Warm up, Scene input, Periodic refresh, Variability "
            "policy");
}
