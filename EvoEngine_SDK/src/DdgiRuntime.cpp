#include "DdgiRuntime.hpp"
#include "DdgiProbeRayData.hpp"
#include "GiProbeSettings.hpp"
#include "Platform.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>

using namespace evo_engine;

namespace {
glm::ivec3 ClampDdgiProbeCounts(const glm::ivec3& value) {
  return glm::clamp(value, glm::ivec3(1), glm::ivec3(257));
}

}  // namespace

uint32_t DdgiRuntime::GetProbeCount(const glm::ivec3& probe_counts) {
  if (!ValidateProbeGrid(probe_counts, (std::numeric_limits<uint32_t>::max)())) {
    return 0u;
  }
  return static_cast<uint32_t>(probe_counts.x) * static_cast<uint32_t>(probe_counts.y) *
         static_cast<uint32_t>(probe_counts.z);
}

bool DdgiRuntime::AddDeviceHistoryAllocations(const DdgiHistoryLayout& layout, GiHistoryBudget& budget,
                                              std::string& error) {
  std::array<uint64_t, DdgiHistoryLayout::BufferCount> allocations{};
  for (size_t i = 0; i < allocations.size(); ++i) {
    VkBufferCreateInfo info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
    info.size = layout.buffer_bytes[i];
    info.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT |
                 VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT;
    info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    VkBuffer buffer = VK_NULL_HANDLE;
    if (vkCreateBuffer(Platform::GetVkDevice(), &info, nullptr, &buffer) != VK_SUCCESS) {
      error = "DDGI history buffer is unsupported by the current device.";
      return false;
    }
    VkMemoryRequirements requirements{};
    vkGetBufferMemoryRequirements(Platform::GetVkDevice(), buffer, &requirements);
    vkDestroyBuffer(Platform::GetVkDevice(), buffer, nullptr);
    allocations[i] = requirements.size;
  }
  if (!DdgiHistoryLayout::AddAllocationBytes(allocations, budget)) {
    error = "Combined device-padded GI histories must remain strictly below 4 GiB.";
    return false;
  }
  return true;
}

uint32_t DdgiRuntime::GetFixedRayCount(const uint32_t ray_count, const bool fixed_rays_enabled) {
  return fixed_rays_enabled && ray_count > 1u ? glm::min(32u, ray_count - 1u) : 0u;
}

DdgiProbeUpdateVariant DdgiRuntime::ParseProbeUpdateVariant(const std::string_view value) {
  if (value == "serial") {
    return DdgiProbeUpdateVariant::Serial;
  }
  if (value == "parallel-shared") {
    return DdgiProbeUpdateVariant::ParallelShared;
  }
  if (value == "parallel-direct") {
    return DdgiProbeUpdateVariant::ParallelDirect;
  }
  return DdgiProbeUpdateVariant::Serial;
}

bool DdgiRuntime::IsReflectionProbeRuntimeReady(const bool has_valid_history, const bool lighting_descriptors_bound,
                                                const bool history_window_complete,
                                                const bool relocation_warmup_active) {
  return has_valid_history && lighting_descriptors_bound && history_window_complete && !relocation_warmup_active;
}

DdgiProbeUpdateVariant DdgiRuntime::ResolveProbeUpdateVariant(const DdgiProbeUpdateVariant requested,
                                                              const DdgiProbeUpdateDeviceLimits& limits,
                                                              const uint32_t probe_count,
                                                              const bool irradiance_pipeline_ready,
                                                              const bool visibility_pipeline_ready) {
  if (requested == DdgiProbeUpdateVariant::Serial) {
    return requested;
  }
  const auto dispatch_capacity = static_cast<uint64_t>(limits.max_work_group_count_x) * limits.max_work_group_count_y;
  if (probe_count == 0u || limits.max_work_group_invocations < kProbeUpdateGroupSize ||
      limits.max_work_group_size_x < kProbeUpdateGroupSize || probe_count > dispatch_capacity ||
      !irradiance_pipeline_ready || !visibility_pipeline_ready ||
      (requested == DdgiProbeUpdateVariant::ParallelShared &&
       limits.max_shared_memory_bytes < kProbeUpdateSharedMemoryBytes)) {
    return DdgiProbeUpdateVariant::Serial;
  }
  return requested;
}

uint32_t DdgiRuntime::GetAllocatedProbeCount(const DdgiSettings& settings) {
  return GetAllocatedProbeCount(settings, GetProbeCount(GiProbeSettings{}.ProbeSize()));
}

uint32_t DdgiRuntime::GetAllocatedProbeCount(const DdgiSettings&, const uint32_t probe_count) {
  return probe_count;
}

bool DdgiRuntime::ValidateProbeGrid(const glm::ivec3& probe_counts, const uint32_t max_probe_count,
                                    std::string* error) {
  const auto fail = [&](const std::string& message) {
    if (error) {
      *error = message;
    }
    return false;
  };
  for (int axis = 0; axis < 3; ++axis) {
    if (probe_counts[axis] < 1 || probe_counts[axis] > 257) {
      return fail("DDGI probe counts must be between 1 and 257 on every axis.");
    }
  }
  if (max_probe_count == 0u) {
    return fail("DDGI maximum probe count must be greater than zero.");
  }
  const auto probe_count = static_cast<uint64_t>(probe_counts.x) * static_cast<uint64_t>(probe_counts.y) *
                           static_cast<uint64_t>(probe_counts.z);
  if (probe_count > max_probe_count) {
    return fail("DDGI probe grid requires " + std::to_string(probe_count) + " probes, exceeding the limit of " +
                std::to_string(max_probe_count) + ".");
  }
  if (error) {
    error->clear();
  }
  return true;
}

uint32_t DdgiRuntime::GetProbeRayFlags(const bool skip_inactive_probes, const bool emissive_mesh_sampling) {
  return (skip_inactive_probes ? kProbeRayFlagSkipInactive : 0u) |
         (emissive_mesh_sampling ? kProbeRayFlagEmissiveMeshSampling : 0u);
}

uint64_t DdgiRuntime::CalculateEmissiveSamplingCandidateRayCount(const uint32_t updated_probe_count,
                                                                 const uint32_t ray_count,
                                                                 const uint32_t fixed_ray_count,
                                                                 const bool emissive_mesh_sampling,
                                                                 const bool trace_probe_rays) {
  if (!emissive_mesh_sampling || !trace_probe_rays) {
    return 0u;
  }
  return static_cast<uint64_t>(updated_probe_count) *
         static_cast<uint64_t>(ray_count - glm::min(fixed_ray_count, ray_count));
}

bool DdgiRuntime::RequiresFullScrollReset(const glm::ivec3& probe_counts, const glm::ivec3& scroll_delta) {
  if (!ValidateProbeGrid(probe_counts, (std::numeric_limits<uint32_t>::max)())) {
    return true;
  }
  for (int axis = 0; axis < 3; ++axis) {
    if (std::abs(static_cast<int64_t>(scroll_delta[axis])) >= static_cast<int64_t>(probe_counts[axis])) {
      return true;
    }
  }
  return false;
}

glm::uvec3 DdgiRuntime::GetProbeGridIndex(const glm::ivec3& probe_counts, const uint32_t probe_index) {
  if (!ValidateProbeGrid(probe_counts, (std::numeric_limits<uint32_t>::max)())) {
    return {0u, 0u, 0u};
  }
  const auto counts = probe_counts;
  const auto x_count = static_cast<uint32_t>(counts.x);
  const auto y_count = static_cast<uint32_t>(counts.y);
  const auto safe_index = glm::min(probe_index, GetProbeCount(counts) - 1u);
  return {safe_index % x_count, (safe_index / x_count) % y_count, safe_index / (x_count * y_count)};
}

DdgiAtlasLayout DdgiRuntime::CalculateAtlasLayout(const uint32_t probe_count, const uint32_t tile_resolution,
                                                  const uint32_t preferred_columns) {
  return CalculateAtlasLayout(probe_count, tile_resolution, preferred_columns, (std::numeric_limits<uint32_t>::max)());
}

DdgiAtlasLayout DdgiRuntime::CalculateAtlasLayout(const uint32_t probe_count, const uint32_t tile_resolution,
                                                  const uint32_t preferred_columns,
                                                  const uint32_t max_image_dimension_2d) {
  DdgiAtlasLayout layout;
  if (probe_count == 0u) {
    layout.error = "DDGI atlas requires at least one probe.";
    return layout;
  }
  if (tile_resolution == 0u) {
    layout.error = "DDGI atlas tile resolution must be greater than zero.";
    return layout;
  }
  if (preferred_columns == 0u) {
    layout.error = "DDGI atlas column count must be greater than zero.";
    return layout;
  }
  if (max_image_dimension_2d == 0u) {
    layout.error = "The Vulkan device reports a zero 2D image-dimension limit.";
    return layout;
  }
  const uint64_t tile_stride = static_cast<uint64_t>(tile_resolution) + 2ull;
  const uint64_t max_tiles = max_image_dimension_2d / tile_stride;
  if (!max_tiles || uint64_t(probe_count) > max_tiles * max_tiles) {
    layout.error = "DDGI probes cannot fit within the Vulkan 2D image limit.";
    return layout;
  }
  const uint64_t minimum_columns = (uint64_t(probe_count) + max_tiles - 1) / max_tiles;
  const uint64_t columns =
      std::clamp(uint64_t(preferred_columns), minimum_columns, std::min(uint64_t(probe_count), max_tiles));
  const uint64_t rows = (static_cast<uint64_t>(probe_count) + columns - 1ull) / columns;
  const uint64_t width = columns * tile_stride;
  const uint64_t height = rows * tile_stride;
  if (width > max_image_dimension_2d || height > max_image_dimension_2d ||
      width > (std::numeric_limits<uint32_t>::max)() || height > (std::numeric_limits<uint32_t>::max)()) {
    layout.error = "DDGI atlas extent " + std::to_string(width) + "x" + std::to_string(height) +
                   " exceeds the Vulkan 2D image limit " + std::to_string(max_image_dimension_2d) + ".";
    return layout;
  }
  layout.valid = true;
  layout.probe_count = probe_count;
  layout.tile_resolution = tile_resolution;
  layout.columns = static_cast<uint32_t>(columns);
  layout.rows = static_cast<uint32_t>(rows);
  layout.resolution = {static_cast<uint32_t>(width), static_cast<uint32_t>(height)};
  return layout;
}

DdgiFrameResourceLayout DdgiRuntime::CalculateFrameResourceLayout(const DdgiSettings& settings) {
  return CalculateFrameResourceLayout(settings, GetProbeCount(GiProbeSettings{}.ProbeSize()));
}

DdgiFrameResourceLayout DdgiRuntime::CalculateFrameResourceLayout(const DdgiSettings& settings,
                                                                  const uint32_t probe_count) {
  return CalculateFrameResourceLayout(settings, probe_count, (std::numeric_limits<uint32_t>::max)());
}

DdgiFrameResourceLayout DdgiRuntime::CalculateFrameResourceLayout(const DdgiSettings& settings,
                                                                  const uint32_t probe_count,
                                                                  const uint32_t max_image_dimension_2d) {
  return CalculateFrameResourceLayout(settings, probe_count, max_image_dimension_2d,
                                      (std::numeric_limits<uint64_t>::max)());
}

DdgiFrameResourceLayout DdgiRuntime::CalculateFrameResourceLayout(const DdgiSettings& settings,
                                                                  const uint32_t probe_count,
                                                                  const uint32_t max_image_dimension_2d,
                                                                  const uint64_t max_storage_buffer_range) {
  DdgiFrameResourceLayout layout;
  if (!std::isfinite(settings.runtime.visibility_smoothing) || settings.runtime.visibility_smoothing < 0.0f ||
      settings.runtime.visibility_smoothing > 0.99f) {
    layout.error = "DDGI visibility smoothing must be finite and between 0 and 0.99.";
    return layout;
  }
  if (settings.runtime.ray_count < 1 || settings.runtime.ray_count > 4096 || settings.runtime.emissive_ray_count < 0 ||
      settings.runtime.emissive_ray_count > 4096) {
    layout.error = "DDGI ray settings are outside their supported ranges.";
    return layout;
  }
  if (settings.storage.irradiance_tile_resolution < 1 || settings.storage.irradiance_tile_resolution > 128 ||
      settings.storage.visibility_tile_resolution < 1 || settings.storage.visibility_tile_resolution > 128 ||
      settings.storage.atlas_probe_columns < 1 || settings.storage.atlas_probe_columns > 4096) {
    layout.error = "DDGI storage settings are outside their supported ranges.";
    return layout;
  }
  layout.probe_count = GetAllocatedProbeCount(settings, probe_count);
  if (layout.probe_count == 0u) {
    layout.error = probe_count == 0u ? "DDGI probe grid is invalid."
                                     : "DDGI probe grid exceeds the configured maximum probe count.";
    return layout;
  }
  layout.irradiance_atlas =
      CalculateAtlasLayout(layout.probe_count, static_cast<uint32_t>(settings.storage.irradiance_tile_resolution),
                           static_cast<uint32_t>(settings.storage.atlas_probe_columns), max_image_dimension_2d);
  layout.visibility_atlas =
      CalculateAtlasLayout(layout.probe_count, static_cast<uint32_t>(settings.storage.visibility_tile_resolution),
                           static_cast<uint32_t>(settings.storage.atlas_probe_columns), max_image_dimension_2d);
  if (!layout.irradiance_atlas.valid || !layout.visibility_atlas.valid) {
    layout.error = !layout.irradiance_atlas.valid ? layout.irradiance_atlas.error : layout.visibility_atlas.error;
    return layout;
  }
  if (!DdgiHistoryLayout::Calculate(layout.probe_count, layout.irradiance_atlas.tile_resolution,
                                    settings.runtime.history_count, layout.history)) {
    layout.error = "DDGI history count must be 5 to 30 in steps of 5, with representable storage sizes.";
    return layout;
  }
  GiHistoryBudget history_budget;
  layout.history_count = static_cast<uint32_t>(settings.runtime.history_count);
  if (!DdgiHistoryLayout::AddAllocationBytes(layout.history.buffer_bytes, history_budget)) {
    layout.error = "DDGI history storage must remain strictly below 4 GiB.";
    return layout;
  }
  for (const auto bytes : layout.history.buffer_bytes) {
    if (bytes > max_storage_buffer_range) {
      layout.error = "DDGI history storage exceeds the Vulkan maxStorageBufferRange limit.";
      return layout;
    }
  }
  layout.history_allocation_bytes = history_budget.bytes;
  layout.probe_metadata_byte_size = static_cast<uint64_t>(layout.probe_count) * sizeof(glm::vec4) * 3ull;
  layout.probe_state_byte_size = static_cast<uint64_t>(layout.probe_count) * sizeof(glm::vec4);
  const auto uniform_ray_count = static_cast<uint64_t>(glm::max(settings.runtime.ray_count, 1));
  const auto emissive_ray_count = settings.runtime.enable_emissive_mesh_sampling
                                      ? static_cast<uint64_t>(glm::max(settings.runtime.emissive_ray_count, 0))
                                      : 0ull;
  const auto total_ray_count = uniform_ray_count + emissive_ray_count;
  layout.ray_output_byte_size = static_cast<uint64_t>(layout.probe_count) * total_ray_count * sizeof(DdgiProbeRayData);
  if (emissive_ray_count > 0u) {
    layout.ray_sample_info_byte_size =
        static_cast<uint64_t>(layout.probe_count) * emissive_ray_count * sizeof(DdgiProbeRaySampleInfo);
  }
  layout.selected_ray_diagnostics_byte_size = total_ray_count * sizeof(PointCloudSample);
  layout.irradiance_atlas_byte_size = static_cast<uint64_t>(layout.irradiance_atlas.resolution.x) *
                                      static_cast<uint64_t>(layout.irradiance_atlas.resolution.y) * 8ull;
  layout.visibility_atlas_byte_size = static_cast<uint64_t>(layout.visibility_atlas.resolution.x) *
                                      static_cast<uint64_t>(layout.visibility_atlas.resolution.y) * 4ull;
  layout.persistent_byte_size = layout.probe_metadata_byte_size + layout.probe_state_byte_size +
                                layout.irradiance_atlas_byte_size + layout.visibility_atlas_byte_size +
                                history_budget.bytes;
  layout.per_frame_transient_byte_size =
      layout.ray_output_byte_size + layout.selected_ray_diagnostics_byte_size + layout.ray_sample_info_byte_size;
  layout.peak_resident_byte_size =
      layout.persistent_byte_size + Platform::kMaxFramesInFlight * layout.per_frame_transient_byte_size;
  if (max_storage_buffer_range == 0u || layout.probe_metadata_byte_size > max_storage_buffer_range ||
      layout.probe_state_byte_size > max_storage_buffer_range ||
      layout.ray_output_byte_size > max_storage_buffer_range ||
      layout.ray_sample_info_byte_size > max_storage_buffer_range ||
      layout.selected_ray_diagnostics_byte_size > max_storage_buffer_range) {
    layout.error = "DDGI storage-buffer allocation exceeds the Vulkan maxStorageBufferRange limit of " +
                   std::to_string(max_storage_buffer_range) + " bytes.";
    return layout;
  }
  layout.valid = true;
  return layout;
}

bool DdgiRuntime::ArePersistentLayoutsCompatible(const DdgiFrameResourceLayout& previous,
                                                 const DdgiFrameResourceLayout& current) {
  const auto atlas_matches = [](const DdgiAtlasLayout& lhs, const DdgiAtlasLayout& rhs) {
    return lhs.valid && rhs.valid && lhs.probe_count == rhs.probe_count && lhs.tile_resolution == rhs.tile_resolution &&
           lhs.columns == rhs.columns && lhs.rows == rhs.rows && lhs.resolution == rhs.resolution;
  };
  return previous.valid && current.valid && previous.probe_count == current.probe_count &&
         previous.history.buffer_bytes == current.history.buffer_bytes &&
         previous.probe_metadata_byte_size == current.probe_metadata_byte_size &&
         previous.probe_state_byte_size == current.probe_state_byte_size &&
         atlas_matches(previous.irradiance_atlas, current.irradiance_atlas) &&
         atlas_matches(previous.visibility_atlas, current.visibility_atlas);
}

std::string DdgiRuntime::FormatUpdateReasons(const uint32_t reasons) {
  if (reasons == DdgiUpdateReasonNone) {
    return "None";
  }
  std::string result;
  const auto append_reason = [&](const uint32_t reason, const char* label) {
    if ((reasons & reason) == 0u) {
      return;
    }
    if (!result.empty()) {
      result += ", ";
    }
    result += label;
  };
  append_reason(DdgiUpdateReasonSource, "DDGI source");
  append_reason(DdgiUpdateReasonManualReset, "Manual reset");
  append_reason(DdgiUpdateReasonSteadyState, "Steady state");
  append_reason(DdgiUpdateReasonWarmup, "Warm up");
  append_reason(DdgiUpdateReasonSceneChange, "Scene change");
  return result.empty() ? "Unknown" : result;
}

DdgiCascadeSetValidation DdgiRuntime::ValidateCascadeSet(const std::vector<DdgiCascadeRuntimeInfo>& infos,
                                                         const uint32_t configured_probe_limit) {
  DdgiCascadeSetValidation result;
  if (infos.size() > kMaxVolumeCount) {
    result.error = "DDGI volume limit exceeded: " + std::to_string(infos.size()) + " enabled volumes; maximum is " +
                   std::to_string(kMaxVolumeCount) + ".";
    return result;
  }
  const auto probe_limit = configured_probe_limit;
  if (probe_limit == 0u) {
    result.error = "DDGI aggregate probe limit must be greater than zero.";
    return result;
  }
  uint64_t aggregate_probe_count = 0u;
  std::unordered_set<uint64_t> stable_ids;
  for (const auto& info : infos) {
    if (info.stable_entity_id == 0u) {
      result.error = "DDGI volume stable IDs must be nonzero.";
      return result;
    }
    if (!stable_ids.emplace(info.stable_entity_id).second) {
      result.error = "Duplicate DDGI volume stable ID: " + std::to_string(info.stable_entity_id) + ".";
      return result;
    }
    aggregate_probe_count += info.probe_count;
    if (aggregate_probe_count > probe_limit) {
      result.aggregate_probe_count =
          static_cast<uint32_t>(std::min<uint64_t>(aggregate_probe_count, (std::numeric_limits<uint32_t>::max)()));
      result.error = "DDGI aggregate probe limit exceeded: " + std::to_string(aggregate_probe_count) +
                     " resident probes; maximum is " + std::to_string(probe_limit) + ".";
      return result;
    }
  }
  result.aggregate_probe_count = static_cast<uint32_t>(aggregate_probe_count);
  result.valid = true;
  return result;
}
