#include "DdgiVolume.hpp"

using namespace evo_engine;

namespace {
glm::ivec3 ClampProbeCounts(const glm::ivec3& value) {
  return {glm::clamp(value.x, 1, 256), glm::clamp(value.y, 1, 256), glm::clamp(value.z, 1, 256)};
}

glm::vec3 ClampProbeSpacing(const glm::vec3& value) {
  return glm::clamp(value, glm::vec3(0.05f), glm::vec3(10000.0f));
}
}  // namespace

void DdgiVolume::OnCreate() {
  SetEnabled(true);
  ClampSettings();
}

void DdgiVolume::ClampSettings() {
  probe_counts = ClampProbeCounts(probe_counts);
  probe_spacing = ClampProbeSpacing(probe_spacing);
  movement_type = glm::clamp(movement_type, static_cast<int>(DdgiVolumeMovementType::Default),
                             static_cast<int>(DdgiVolumeMovementType::Scrolling));
  relocation_distance = glm::clamp(relocation_distance, 0.0f, 10000.0f);
  random_ray_backface_threshold = glm::clamp(random_ray_backface_threshold, 0.0f, 1.0f);
  fixed_ray_backface_threshold = glm::clamp(fixed_ray_backface_threshold, 0.0f, 1.0f);
  probe_variability_threshold = glm::clamp(probe_variability_threshold, 0.0f, 10.0f);
  probe_variability_min_samples = glm::clamp(probe_variability_min_samples, 0, 4096);
  warmup_trigger_conditions &= DdgiVolumeTriggerConditionAll;
  variability_reset_trigger_conditions &= DdgiVolumeTriggerConditionAll;
  max_visualized_probes = glm::clamp(max_visualized_probes, 1, 16777216);
  probe_visualization_size = glm::clamp(probe_visualization_size, 0.001f, 1000.0f);
}

uint32_t DdgiVolume::GetProbeAmount() const {
  const auto counts = ClampProbeCounts(probe_counts);
  return static_cast<uint32_t>(counts.x * counts.y * counts.z);
}

glm::vec3 DdgiVolume::GetLocalGridSize() const {
  const auto counts = ClampProbeCounts(probe_counts);
  const auto spacing = ClampProbeSpacing(probe_spacing);
  return glm::vec3(counts - glm::ivec3(1)) * spacing;
}

glm::vec3 DdgiVolume::GetProbeLocalPosition(const glm::ivec3& probe_index) const {
  const auto counts = ClampProbeCounts(probe_counts);
  const auto spacing = ClampProbeSpacing(probe_spacing);
  const auto clamped_index =
      glm::ivec3(glm::clamp(probe_index.x, 0, counts.x - 1), glm::clamp(probe_index.y, 0, counts.y - 1),
                 glm::clamp(probe_index.z, 0, counts.z - 1));
  const auto grid_shift = glm::vec3(counts - glm::ivec3(1)) * spacing * 0.5f;
  return volume_origin + glm::vec3(clamped_index) * spacing - grid_shift;
}
