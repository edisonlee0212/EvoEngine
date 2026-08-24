#include "DdgiRuntime.hpp"
#include "DdgiProbeRayData.hpp"
#include "Platform.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>

using namespace evo_engine;

namespace {
glm::ivec3 ClampDdgiProbeCounts(const glm::ivec3& value) {
  return {glm::clamp(value.x, 1, 256), glm::clamp(value.y, 1, 256), glm::clamp(value.z, 1, 256)};
}

float AxisCoordinate(const glm::vec3& delta, const glm::vec3& axis) {
  const auto length_squared = glm::dot(axis, axis);
  if (!(length_squared > 1e-8f)) {
    return 0.0f;
  }
  return glm::dot(delta, axis) / length_squared;
}

}  // namespace

uint32_t DdgiRuntime::GetProbeCount(const glm::ivec3& probe_counts) {
  if (!ValidateProbeGrid(probe_counts, (std::numeric_limits<uint32_t>::max)())) {
    return 0u;
  }
  return static_cast<uint32_t>(probe_counts.x) * static_cast<uint32_t>(probe_counts.y) *
         static_cast<uint32_t>(probe_counts.z);
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

DdgiProbeConvergenceUpdate DdgiRuntime::AdvanceProbeConvergence(const DdgiProbeConvergenceState& state,
                                                                const DdgiProbeVariabilityObservation& observation,
                                                                const float entry_threshold) {
  DdgiProbeConvergenceUpdate update{state};
  if (!observation.valid || !std::isfinite(observation.average) || !std::isfinite(observation.maximum) ||
      !std::isfinite(observation.unstable_fraction) || !std::isfinite(observation.weight) ||
      observation.average < 0.0f || observation.maximum < 0.0f || observation.unstable_fraction < 0.0f ||
      observation.unstable_fraction > 1.0f || observation.weight <= 0.0f) {
    if (!state.converged) {
      update.state.stable_sample_count = 0u;
    }
    return update;
  }
  update.state.sample_count++;
  const auto enter = glm::max(entry_threshold, 0.0f);
  if (state.converged) {
    if (observation.average > enter * kProbeVariabilityExitThresholdScale ||
        observation.maximum > enter * kProbeVariabilityMaximumThresholdScale * kProbeVariabilityExitThresholdScale ||
        observation.unstable_fraction >
            kProbeVariabilityAllowedUnstableFraction * kProbeVariabilityExitUnstableFractionScale) {
      update.state.converged = false;
      update.state.stable_sample_count = 0u;
    }
    return update;
  }
  const bool stable = observation.average <= enter &&
                      observation.maximum <= enter * kProbeVariabilityMaximumThresholdScale &&
                      observation.unstable_fraction <= kProbeVariabilityAllowedUnstableFraction;
  update.state.stable_sample_count = stable ? state.stable_sample_count + 1u : 0u;
  if (update.state.stable_sample_count >= kProbeVariabilityStableSampleCount) {
    update.state.stable_sample_count = kProbeVariabilityStableSampleCount;
    update.state.converged = true;
    update.entered_convergence = true;
  }
  return update;
}

DdgiProbeVariabilityBudgetState DdgiRuntime::ResetProbeVariabilityBudget(const DdgiProbeVariabilityBudgetState& state) {
  DdgiProbeVariabilityBudgetState reset;
  reset.cycle = state.cycle + 1u;
  if (reset.cycle == 0u) {
    reset.cycle = 1u;
  }
  return reset;
}

DdgiProbeVariabilityBudgetUpdate DdgiRuntime::ReserveProbeVariabilityBudgetFrame(
    const DdgiProbeVariabilityBudgetState& state, const uint32_t maximum_frame_count) {
  DdgiProbeVariabilityBudgetUpdate update{state};
  const auto maximum = glm::max(maximum_frame_count, 1u);
  if (state.completed_frame_count + state.pending_frame_count >= maximum) {
    update.maximum_reached = state.completed_frame_count >= maximum;
    return update;
  }
  ++update.state.pending_frame_count;
  update.accepted = true;
  return update;
}

DdgiProbeVariabilityBudgetUpdate DdgiRuntime::ResolveProbeVariabilityBudgetFrame(
    const DdgiProbeVariabilityBudgetState& state, const uint64_t ticket_cycle, const bool valid_observation,
    const uint32_t maximum_frame_count) {
  DdgiProbeVariabilityBudgetUpdate update{state};
  const auto maximum = glm::max(maximum_frame_count, 1u);
  if (ticket_cycle != state.cycle || state.pending_frame_count == 0u) {
    update.maximum_reached = state.completed_frame_count >= maximum;
    return update;
  }
  --update.state.pending_frame_count;
  if (valid_observation) {
    ++update.state.completed_frame_count;
  }
  update.accepted = true;
  update.maximum_reached = update.state.completed_frame_count >= maximum;
  return update;
}

bool DdgiRuntime::IsReflectionProbeRuntimeReady(const bool has_valid_history, const bool lighting_descriptors_bound,
                                                const bool variability_gating_enabled,
                                                const bool variability_sampling_complete) {
  return has_valid_history && lighting_descriptors_bound &&
         (!variability_gating_enabled || variability_sampling_complete);
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
  return GetAllocatedProbeCount(settings, GetProbeCount(settings.volume_defaults.probe_counts));
}

uint32_t DdgiRuntime::GetAllocatedProbeCount(const DdgiSettings& settings, const uint32_t probe_count) {
  if (settings.storage.max_probe_count <= 0 || probe_count == 0u ||
      probe_count > static_cast<uint32_t>(settings.storage.max_probe_count)) {
    return 0u;
  }
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
    if (probe_counts[axis] < 1 || probe_counts[axis] > 256) {
      return fail("DDGI probe counts must be between 1 and 256 on every axis.");
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

bool DdgiRuntime::ResolveEmissiveMeshSampling(const bool global_enabled, const int volume_mode) {
  if (volume_mode == static_cast<int>(DdgiEmissiveMeshSamplingMode::On)) {
    return true;
  }
  if (volume_mode == static_cast<int>(DdgiEmissiveMeshSamplingMode::Off)) {
    return false;
  }
  return global_enabled;
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
  const uint64_t columns = glm::min(static_cast<uint64_t>(probe_count), static_cast<uint64_t>(preferred_columns));
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
  return CalculateFrameResourceLayout(settings, GetProbeCount(settings.volume_defaults.probe_counts));
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
  if (settings.runtime.ray_count < 1 || settings.runtime.ray_count > 4096 || settings.runtime.emissive_ray_count < 0 ||
      settings.runtime.emissive_ray_count > 4096) {
    layout.error = "DDGI ray settings are outside their supported ranges.";
    return layout;
  }
  if (settings.storage.max_probe_count < 1 || settings.storage.max_probe_count > 16777216 ||
      settings.storage.irradiance_tile_resolution < 1 || settings.storage.irradiance_tile_resolution > 128 ||
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
  layout.variability_atlas = layout.irradiance_atlas;
  layout.variability_atlas.resolution = {layout.variability_atlas.columns * layout.variability_atlas.tile_resolution,
                                         layout.variability_atlas.rows * layout.variability_atlas.tile_resolution};
  layout.variability_reduction_extent = {glm::max(1u, (layout.variability_atlas.resolution.x + 15u) / 16u),
                                         glm::max(1u, (layout.variability_atlas.resolution.y + 15u) / 16u)};
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
  layout.variability_atlas_byte_size = static_cast<uint64_t>(layout.variability_atlas.resolution.x) *
                                       static_cast<uint64_t>(layout.variability_atlas.resolution.y) * sizeof(uint16_t);
  layout.variability_reduction_byte_size = static_cast<uint64_t>(layout.variability_reduction_extent.x) *
                                           static_cast<uint64_t>(layout.variability_reduction_extent.y) *
                                           sizeof(glm::vec4);
  layout.persistent_byte_size = layout.probe_metadata_byte_size + layout.probe_state_byte_size +
                                layout.irradiance_atlas_byte_size + layout.visibility_atlas_byte_size +
                                layout.variability_atlas_byte_size;
  layout.per_frame_transient_byte_size = layout.ray_output_byte_size + layout.selected_ray_diagnostics_byte_size +
                                         layout.ray_sample_info_byte_size +
                                         2ull * layout.variability_reduction_byte_size + sizeof(glm::vec4);
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
         previous.probe_metadata_byte_size == current.probe_metadata_byte_size &&
         previous.probe_state_byte_size == current.probe_state_byte_size &&
         atlas_matches(previous.irradiance_atlas, current.irradiance_atlas) &&
         atlas_matches(previous.visibility_atlas, current.visibility_atlas) &&
         atlas_matches(previous.variability_atlas, current.variability_atlas);
}

evo_engine::DdgiHysteresisBoostUpdate DdgiRuntime::AdvanceHysteresisBoost(
    const float current_hysteresis, const bool active, const float normal_hysteresis, const float boosted_hysteresis,
    const float restore_speed, const bool scene_changed) {
  const auto normal = glm::clamp(normal_hysteresis, 0.0f, 1.0f);
  const auto boosted = glm::clamp(boosted_hysteresis, 0.0f, 1.0f);
  if (scene_changed) {
    return {boosted, boosted != normal, true, false};
  }
  if (!active) {
    return {normal, false, false, false};
  }
  const auto speed = glm::clamp(restore_speed, 0.0f, 1.0f);
  const auto current = glm::clamp(current_hysteresis, 0.0f, 1.0f);
  const auto delta = normal - current;
  const bool reached_normal = delta == 0.0f || (speed > 0.0f && std::abs(delta) <= speed + 1e-6f);
  const auto restored = reached_normal ? normal : current + std::copysign(speed, delta);
  return {restored, !reached_normal, true, true};
}

float DdgiRuntime::CalculateUpdateHysteresis(const float update_hysteresis, const uint32_t warmup_frame_count,
                                             const uint32_t update_reasons, const uint32_t warmup_frame_index) {
  if ((update_reasons & (DdgiUpdateReasonManualReset | DdgiUpdateReasonVariabilityPolicy)) != 0u) {
    return 0.0f;
  }
  const auto hysteresis = glm::clamp(update_hysteresis, 0.0f, 1.0f);
  if ((update_reasons & (DdgiUpdateReasonSceneChange | DdgiUpdateReasonHysteresisRestore)) != 0u) {
    return hysteresis;
  }
  if ((update_reasons & DdgiUpdateReasonWarmup) == 0u || warmup_frame_count == 0u ||
      warmup_frame_index >= warmup_frame_count) {
    return hysteresis;
  }
  const auto denominator = static_cast<float>(glm::max(warmup_frame_count - 1u, 1u));
  return hysteresis * (static_cast<float>(warmup_frame_index) / denominator);
}

float DdgiRuntime::CalculateUpdateBrightnessThreshold(const DdgiSettings& settings) {
  const auto brightness_threshold = glm::clamp(settings.runtime.brightness_threshold, 0.0f, 1.0f);
  return brightness_threshold;
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
  append_reason(DdgiUpdateReasonConverged, "Converged");
  append_reason(DdgiUpdateReasonWarmup, "Warm up");
  append_reason(DdgiUpdateReasonSceneChange, "Scene change");
  append_reason(DdgiUpdateReasonVariabilityPolicy, "Variability policy");
  append_reason(DdgiUpdateReasonHysteresisRestore, "Hysteresis restore");
  append_reason(DdgiUpdateReasonVariabilityMaximum, "Variability maximum");
  return result.empty() ? "Unknown" : result;
}

float DdgiRuntime::CalculateVolumeBlendWeight(const glm::vec3& probe_coordinate, const glm::ivec3& probe_counts,
                                              const glm::vec3& probe_step_lengths) {
  const auto counts = ClampDdgiProbeCounts(probe_counts);
  const auto max_probe_coordinate = glm::vec3(counts - glm::ivec3(1));
  const auto inside_volume = !glm::any(glm::lessThan(probe_coordinate, glm::vec3(0.0f))) &&
                             !glm::any(glm::greaterThan(probe_coordinate, max_probe_coordinate));
  if (inside_volume) {
    return 1.0f;
  }

  const auto step_lengths = glm::max(probe_step_lengths, glm::vec3(0.0001f));
  const auto lower_distance = glm::max(-probe_coordinate * step_lengths, glm::vec3(0.0f));
  const auto upper_distance = glm::max((probe_coordinate - max_probe_coordinate) * step_lengths, glm::vec3(0.0f));
  const auto outside_distance = glm::max(lower_distance, upper_distance);
  const auto axis_weight =
      glm::vec3(1.0f) - glm::clamp(outside_distance / step_lengths, glm::vec3(0.0f), glm::vec3(1.0f));
  return axis_weight.x * axis_weight.y * axis_weight.z;
}

float DdgiRuntime::CalculateProbeDensity(const glm::vec3& probe_step_x, const glm::vec3& probe_step_y,
                                         const glm::vec3& probe_step_z) {
  const auto cell_volume = std::abs(glm::dot(probe_step_x, glm::cross(probe_step_y, probe_step_z)));
  return std::isfinite(cell_volume) && cell_volume > 1e-8f ? 1.0f / cell_volume : 0.0f;
}

void DdgiRuntime::SortVolumeRuntimeInfos(std::vector<DdgiVolumeRuntimeInfo>& infos) {
  std::sort(infos.begin(), infos.end(), [](const auto& lhs, const auto& rhs) {
    if (lhs.artist_priority != rhs.artist_priority) {
      return lhs.artist_priority > rhs.artist_priority;
    }
    if (lhs.probe_density != rhs.probe_density) {
      return lhs.probe_density > rhs.probe_density;
    }
    return lhs.stable_entity_id < rhs.stable_entity_id;
  });
  for (size_t i = 0; i < infos.size(); ++i) {
    infos[i].sorted_index = static_cast<uint32_t>(i);
  }
}

DdgiVolumeSetValidation DdgiRuntime::ValidateVolumeSet(const std::vector<DdgiVolumeRuntimeInfo>& infos,
                                                       const uint32_t configured_probe_limit) {
  DdgiVolumeSetValidation result;
  if (infos.size() > kMaxVolumeCount) {
    result.error = "DDGI volume limit exceeded: " + std::to_string(infos.size()) + " enabled volumes; maximum is " +
                   std::to_string(kMaxVolumeCount) + ".";
    return result;
  }
  const auto probe_limit = glm::min(configured_probe_limit, kMaxResidentProbeCount);
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

DdgiVolumeSelection DdgiRuntime::SelectVolumes(const std::vector<DdgiVolumeRuntimeInfo>& infos,
                                               const glm::vec3& world_position) {
  struct Candidate {
    const DdgiVolumeRuntimeInfo* info = nullptr;
    glm::vec3 probe_coordinate = glm::vec3(0.0f);
    float coverage = 0.0f;
    bool inside = false;
  };
  auto sorted_infos = infos;
  SortVolumeRuntimeInfos(sorted_infos);
  std::vector<Candidate> candidates;
  candidates.reserve(sorted_infos.size());
  for (const auto& info : sorted_infos) {
    if (!info.contributes_lighting) {
      continue;
    }
    const auto delta = world_position - info.first_probe;
    const glm::vec3 coordinate{AxisCoordinate(delta, info.probe_step_x), AxisCoordinate(delta, info.probe_step_y),
                               AxisCoordinate(delta, info.probe_step_z)};
    const auto max_coordinate = glm::vec3(ClampDdgiProbeCounts(info.probe_counts) - glm::ivec3(1));
    const auto inside = !glm::any(glm::lessThan(coordinate, glm::vec3(0.0f))) &&
                        !glm::any(glm::greaterThan(coordinate, max_coordinate));
    const glm::vec3 step_lengths{glm::length(info.probe_step_x), glm::length(info.probe_step_y),
                                 glm::length(info.probe_step_z)};
    const auto coverage = CalculateVolumeBlendWeight(coordinate, info.probe_counts, step_lengths);
    candidates.push_back({&info, coordinate, coverage, inside});
  }
  const auto primary = std::find_if(candidates.begin(), candidates.end(), [](const auto& candidate) {
    return candidate.coverage > 0.0f;
  });
  DdgiVolumeSelection result;
  if (primary == candidates.end()) {
    return result;
  }
  result.valid = true;
  result.primary_entity_id = primary->info->stable_entity_id;
  result.primary_weight = glm::clamp(primary->coverage, 0.0f, 1.0f);
  result.ibl_weight = 1.0f - result.primary_weight;

  float boundary_weight = 1.0f;
  if (primary->inside) {
    const auto max_coordinate = glm::vec3(ClampDdgiProbeCounts(primary->info->probe_counts) - glm::ivec3(1));
    const auto distance_to_boundary = glm::min(primary->probe_coordinate, max_coordinate - primary->probe_coordinate);
    const auto nearest_boundary =
        glm::min(distance_to_boundary.x, glm::min(distance_to_boundary.y, distance_to_boundary.z));
    boundary_weight = glm::clamp(1.0f - nearest_boundary, 0.0f, 1.0f);
  }
  if (!(boundary_weight > 0.0f)) {
    return result;
  }
  const auto secondary = std::find_if(candidates.begin(), candidates.end(), [&](const auto& candidate) {
    return candidate.info->stable_entity_id != primary->info->stable_entity_id && candidate.coverage > 0.0f;
  });
  if (secondary == candidates.end()) {
    return result;
  }
  const auto weighted_secondary_coverage = secondary->coverage * boundary_weight;
  const auto weight_sum = primary->coverage + weighted_secondary_coverage;
  if (!(weight_sum > 0.0f) || !std::isfinite(weight_sum)) {
    return result;
  }
  result.secondary_entity_id = secondary->info->stable_entity_id;
  const auto combined_coverage = glm::clamp(weight_sum, 0.0f, 1.0f);
  result.primary_weight = primary->coverage / weight_sum * combined_coverage;
  result.secondary_weight = weighted_secondary_coverage / weight_sum * combined_coverage;
  result.ibl_weight = 1.0f - combined_coverage;
  return result;
}
