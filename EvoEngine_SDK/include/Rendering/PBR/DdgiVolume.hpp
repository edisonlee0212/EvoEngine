#pragma once
#include "DdgiSettings.hpp"
#include "IPrivateComponent.hpp"

namespace evo_engine {
class DdgiVolume final : public IPrivateComponent {
 public:
  glm::ivec3 probe_counts = {16, 12, 28};
  glm::vec3 probe_spacing = glm::vec3(1.5f);
  glm::vec3 volume_origin = {4.5f, 4.25f, 10.25f};
  int movement_type = static_cast<int>(DdgiVolumeMovementType::Default);
  bool enable_probe_relocation = false;
  bool enable_probe_classification = false;
  bool enable_probe_variability = true;
  bool enable_probe_variability_gating = false;
  float relocation_distance = 1.0f;
  float random_ray_backface_threshold = 0.1f;
  float fixed_ray_backface_threshold = 0.25f;
  float probe_variability_threshold = 0.05f;
  int probe_variability_min_samples = 128;
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
