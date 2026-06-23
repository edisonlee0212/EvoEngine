#pragma once
#include "DdgiSettings.hpp"
#include "IPrivateComponent.hpp"

namespace evo_engine {
class DdgiVolume final : public IPrivateComponent {
 public:
  glm::ivec3 probe_counts = {10, 6, 16};
  glm::vec3 probe_spacing = glm::vec3(1.5f);
  glm::vec3 volume_origin = {0.0f, 3.0f, 3.0f};
  int movement_type = static_cast<int>(DdgiVolumeMovementType::Default);
  bool enable_probe_relocation = true;
  bool enable_probe_classification = false;
  bool enable_probe_variability = true;
  bool enable_probe_variability_gating = true;
  float relocation_distance = 0.25f;
  float random_ray_backface_threshold = 0.1f;
  float fixed_ray_backface_threshold = 0.25f;
  float probe_variability_threshold = 0.2f;
  int probe_variability_min_samples = 16;
  int warmup_trigger_conditions = DdgiVolumeTriggerConditionLightEnableChanged;
  int variability_reset_trigger_conditions =
      DdgiVolumeTriggerConditionLightingConditionChanged | DdgiVolumeTriggerConditionGeometryChanged;
  bool visualize_bounds = true;
  bool visualize_probe_positions = true;
  int max_visualized_probes = 8192;
  float probe_visualization_size = 0.08f;

  void OnCreate() override;
  void ClampSettings();
  [[nodiscard]] uint32_t GetProbeAmount() const;
  [[nodiscard]] glm::vec3 GetLocalGridSize() const;
  [[nodiscard]] glm::vec3 GetProbeLocalPosition(const glm::ivec3& probe_index) const;
};
}  // namespace evo_engine
