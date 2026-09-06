#pragma once

#include "DdgiHistory.hpp"
#include "DdgiSettings.hpp"
#include "PointCloudSample.hpp"
#include "RenderInstanceStorage.hpp"

#include <array>
#include <cstdint>
#include <glm/glm.hpp>
#include <string>
#include <string_view>
#include <vector>

namespace evo_engine {
struct DdgiAtlasLayout {
  bool valid = false;
  std::string error{};
  uint32_t probe_count = 1;
  uint32_t tile_resolution = 1;
  uint32_t columns = 1;
  uint32_t rows = 1;
  glm::uvec2 resolution = {1, 1};
};

struct DdgiFrameResourceLayout {
  bool valid = false;
  std::string error{};
  uint32_t probe_count = 1;
  DdgiHistoryLayout history;
  uint32_t history_count = 30;
  uint64_t history_allocation_bytes = 0;
  DdgiAtlasLayout irradiance_atlas{};
  DdgiAtlasLayout visibility_atlas{};
  uint64_t probe_metadata_byte_size = 0;
  uint64_t probe_state_byte_size = 0;
  uint64_t ray_output_byte_size = 0;
  uint64_t ray_sample_info_byte_size = 0;
  uint64_t selected_ray_diagnostics_byte_size = 0;
  uint64_t irradiance_atlas_byte_size = 0;
  uint64_t visibility_atlas_byte_size = 0;
  uint64_t persistent_byte_size = 0;
  uint64_t per_frame_transient_byte_size = 0;
  uint64_t peak_resident_byte_size = 0;
};

struct DdgiPerformanceStats {
  uint32_t active_probe_count = 0;
  uint32_t storage_probe_count = 0;
  uint32_t updated_probe_count = 0;
  uint32_t ray_count = 0;
  uint32_t emissive_ray_count = 0;
  uint32_t ray_sample_count = 0;
  uint32_t emissive_triangle_count = 0;
  uint32_t emissive_eligible_instance_count = 0;
  uint32_t emissive_distribution_count = 0;
  uint32_t emissive_fallback_distribution_count = 0;
  uint64_t emissive_logical_triangle_count = 0;
  uint64_t emissive_stored_triangle_count = 0;
  double emissive_distribution_build_ms = 0.0;
  double emissive_distribution_upload_ms = 0.0;
  uint32_t emissive_excluded_instance_count = 0;
  uint32_t emissive_unrepresentable_probability_count = 0;
  double emissive_estimated_power = 0.0;
  uint32_t emissive_sampling_enabled_volume_count = 0;
  uint64_t emissive_sampling_candidate_ray_count = 0;
  uint32_t recorded_ray_sample_count = 0;
  uint32_t recorded_probe_update_count = 0;
  uint32_t selected_ray_sample_count = 0;
  uint32_t visualized_probe_count = 0;
  uint64_t probe_metadata_byte_size = 0;
  uint64_t probe_state_byte_size = 0;
  uint64_t ray_output_byte_size = 0;
  uint64_t ray_sample_info_byte_size = 0;
  uint64_t selected_ray_diagnostics_byte_size = 0;
  uint64_t irradiance_atlas_byte_size = 0;
  uint64_t visibility_atlas_byte_size = 0;
  uint64_t persistent_byte_size = 0;
  uint64_t per_frame_transient_byte_size = 0;
  uint64_t peak_resident_byte_size = 0;
  glm::uvec2 irradiance_atlas_extent = {0, 0};
  glm::uvec2 visibility_atlas_extent = {0, 0};
  bool history_window_complete = false;
  uint32_t probe_warmup_frame_index = 0;
  uint32_t probe_warmup_frame_count = 0;
  bool probe_warmup_active = false;
  bool lighting_descriptors_bound = false;
};

enum DdgiUpdateReason : uint32_t {
  DdgiUpdateReasonNone = 0u,
  DdgiUpdateReasonSource = 1u << 0u,
  DdgiUpdateReasonManualReset = 1u << 1u,
  DdgiUpdateReasonSteadyState = 1u << 2u,
  DdgiUpdateReasonWarmup = 1u << 4u,
  DdgiUpdateReasonSceneChange = 1u << 5u,
};

struct DdgiVolumeRuntimeInfo {
  uint32_t sorted_index = 0;
  uint64_t stable_entity_id = 0;
  int artist_priority = 0;
  float probe_density = 0.0f;
  glm::ivec3 probe_counts = {1, 1, 1};
  uint32_t probe_count = 1;
  bool contributes_lighting = true;
  glm::vec3 first_probe = glm::vec3(0.0f);
  glm::vec3 probe_step_x = glm::vec3(1.0f, 0.0f, 0.0f);
  glm::vec3 probe_step_y = glm::vec3(0.0f, 1.0f, 0.0f);
  glm::vec3 probe_step_z = glm::vec3(0.0f, 0.0f, 1.0f);
};

struct DdgiVolumeRuntimeStats {
  std::string name{};
  uint64_t stable_entity_id = 0;
  uint32_t sorted_index = 0;
  int artist_priority = 0;
  float probe_density = 0.0f;
  glm::ivec3 probe_counts = glm::ivec3(0);
  uint32_t probe_count = 0;
  uint32_t history_phase = 0;
  uint32_t history_count = 30;
  uint32_t history_completed_updates = 0;
  glm::ivec3 probe_scroll_offset = glm::ivec3(0);
  glm::ivec3 last_probe_scroll_delta = glm::ivec3(0);
  bool has_valid_probe_history = false;
  bool contributes_lighting = false;
  bool resources_ready = false;
  bool emissive_mesh_sampling_enabled = false;
  uint32_t last_probe_update_reasons = DdgiUpdateReasonNone;
  uint32_t warmup_frame_index = 0;
  uint32_t warmup_frame_count = 0;
  bool warmup_active = false;
  bool sampling_complete = false;
  bool pending_scene_changes = false;
  uint64_t resident_byte_size = 0;
  glm::vec3 first_probe = glm::vec3(0.0f);
  glm::vec3 probe_step_x = glm::vec3(0.0f);
  glm::vec3 probe_step_y = glm::vec3(0.0f);
  glm::vec3 probe_step_z = glm::vec3(0.0f);
  std::array<uint64_t, 4> resource_ids{};
};

struct DdgiVolumeSetValidation {
  bool valid = false;
  uint32_t aggregate_probe_count = 0;
  std::string error{};
};

struct DdgiVolumeSelection {
  bool valid = false;
  uint64_t primary_entity_id = 0;
  uint64_t secondary_entity_id = 0;
  float primary_weight = 0.0f;
  float secondary_weight = 0.0f;
  float ibl_weight = 1.0f;
};

enum class DdgiProbeUpdateVariant { Serial, ParallelDirect, ParallelShared };

struct DdgiProbeUpdateDeviceLimits {
  uint32_t max_work_group_invocations = 1;
  uint32_t max_work_group_size_x = 1;
  uint32_t max_work_group_count_x = 1;
  uint32_t max_work_group_count_y = 1;
  uint32_t max_shared_memory_bytes = 1;
};

struct DdgiProbeDebugDataView {
  const std::vector<glm::vec4>* metadata = nullptr;
  const std::vector<PointCloudSample>* selected_ray_samples = nullptr;
  uint32_t probe_count = 0;
  uint32_t selected_ray_probe_index = 0;
  uint32_t selected_ray_physical_probe_index = 0;
  uint32_t selected_ray_sample_count = 0;
  bool selected_ray_samples_available = false;
};

class EVOENGINE_API DdgiRuntime final {
 public:
  static constexpr uint32_t kProbeUpdateGroupSize = 64u;
  static constexpr uint32_t kProbeUpdateSharedMemoryBytes = 2u * 256u * sizeof(glm::vec4);
  static constexpr uint32_t kMaxVolumeCount = RenderInstanceStorage::kDdgiMaxVolumeCount;
  static constexpr uint32_t kMaxResidentProbeCount = 8192u;
  static constexpr uint32_t kProbeRayFlagSkipInactive = 1u << 0u;
  static constexpr uint32_t kProbeRayFlagEmissiveMeshSampling = 1u << 1u;

  [[nodiscard]] static uint32_t GetProbeCount(const glm::ivec3& probe_counts);
  [[nodiscard]] static bool AddDeviceHistoryAllocations(const DdgiHistoryLayout& layout, GiHistoryBudget& budget,
                                                        std::string& error);
  [[nodiscard]] static uint32_t GetFixedRayCount(uint32_t ray_count, bool fixed_rays_enabled);
  [[nodiscard]] static DdgiProbeUpdateVariant ParseProbeUpdateVariant(std::string_view value);
  [[nodiscard]] static bool IsReflectionProbeRuntimeReady(bool has_valid_history, bool lighting_descriptors_bound,
                                                          bool history_window_complete, bool relocation_warmup_active);
  [[nodiscard]] static DdgiProbeUpdateVariant ResolveProbeUpdateVariant(DdgiProbeUpdateVariant requested,
                                                                        const DdgiProbeUpdateDeviceLimits& limits,
                                                                        uint32_t probe_count,
                                                                        bool irradiance_pipeline_ready,
                                                                        bool visibility_pipeline_ready);
  [[nodiscard]] static uint32_t GetAllocatedProbeCount(const DdgiSettings& settings);
  [[nodiscard]] static uint32_t GetAllocatedProbeCount(const DdgiSettings& settings, uint32_t probe_count);
  [[nodiscard]] static bool ValidateProbeGrid(const glm::ivec3& probe_counts, uint32_t max_probe_count,
                                              std::string* error = nullptr);
  [[nodiscard]] static bool ResolveEmissiveMeshSampling(bool global_enabled, int volume_mode);
  [[nodiscard]] static uint32_t GetProbeRayFlags(bool skip_inactive_probes, bool emissive_mesh_sampling);
  [[nodiscard]] static uint64_t CalculateEmissiveSamplingCandidateRayCount(uint32_t updated_probe_count,
                                                                           uint32_t ray_count, uint32_t fixed_ray_count,
                                                                           bool emissive_mesh_sampling,
                                                                           bool trace_probe_rays);
  [[nodiscard]] static bool RequiresFullScrollReset(const glm::ivec3& probe_counts, const glm::ivec3& scroll_delta);
  [[nodiscard]] static glm::uvec3 GetProbeGridIndex(const glm::ivec3& probe_counts, uint32_t probe_index);
  [[nodiscard]] static DdgiAtlasLayout CalculateAtlasLayout(uint32_t probe_count, uint32_t tile_resolution,
                                                            uint32_t preferred_columns);
  [[nodiscard]] static DdgiAtlasLayout CalculateAtlasLayout(uint32_t probe_count, uint32_t tile_resolution,
                                                            uint32_t preferred_columns,
                                                            uint32_t max_image_dimension_2d);
  [[nodiscard]] static DdgiFrameResourceLayout CalculateFrameResourceLayout(const DdgiSettings& settings);
  [[nodiscard]] static DdgiFrameResourceLayout CalculateFrameResourceLayout(const DdgiSettings& settings,
                                                                            uint32_t probe_count);
  [[nodiscard]] static DdgiFrameResourceLayout CalculateFrameResourceLayout(const DdgiSettings& settings,
                                                                            uint32_t probe_count,
                                                                            uint32_t max_image_dimension_2d);
  [[nodiscard]] static DdgiFrameResourceLayout CalculateFrameResourceLayout(const DdgiSettings& settings,
                                                                            uint32_t probe_count,
                                                                            uint32_t max_image_dimension_2d,
                                                                            uint64_t max_storage_buffer_range);
  [[nodiscard]] static bool ArePersistentLayoutsCompatible(const DdgiFrameResourceLayout& previous,
                                                           const DdgiFrameResourceLayout& current);
  [[nodiscard]] static std::string FormatUpdateReasons(uint32_t reasons);
  [[nodiscard]] static float CalculateVolumeBlendWeight(const glm::vec3& probe_coordinate,
                                                        const glm::ivec3& probe_counts,
                                                        const glm::vec3& probe_step_lengths);
  [[nodiscard]] static float CalculateProbeDensity(const glm::vec3& probe_step_x, const glm::vec3& probe_step_y,
                                                   const glm::vec3& probe_step_z);
  static void SortVolumeRuntimeInfos(std::vector<DdgiVolumeRuntimeInfo>& infos);
  [[nodiscard]] static DdgiVolumeSetValidation ValidateVolumeSet(const std::vector<DdgiVolumeRuntimeInfo>& infos,
                                                                 uint32_t configured_probe_limit);
  [[nodiscard]] static DdgiVolumeSelection SelectVolumes(const std::vector<DdgiVolumeRuntimeInfo>& infos,
                                                         const glm::vec3& world_position);
};
}  // namespace evo_engine
