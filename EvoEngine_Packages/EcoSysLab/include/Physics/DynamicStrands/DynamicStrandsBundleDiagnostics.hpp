#pragma once

#include "DynamicStrands.hpp"

namespace eco_sys_lab_package {

class DsBundle;

struct BundleMomentum {
  glm::vec3 linear{};
  glm::vec3 angular{};
};

struct BundleExperimentDiagnostics {
  std::string experiment;
  BundleSolverMode mode = BundleSolverMode::Legacy;
  uint32_t segment_count = 0;
  uint32_t pair_count = 0;
  int32_t cross_section_node_handle = -1;
  glm::quat cross_section_rotation{};
  glm::vec3 center_translation{};
  glm::vec3 far_side_displacement{};
  float constraint_rms = 0.f;
  glm::vec3 linear_momentum_residual{};
  glm::vec3 angular_momentum_residual{};
  uint64_t bundle_dispatches_per_projection = 0;
  uint64_t strand_buffer_bytes = 0;
  uint64_t gpu_sample_count = 0;
  double gpu_median_milliseconds = 0.0;
  double gpu_p95_milliseconds = 0.0;

  void Save(const std::filesystem::path& path) const;
};

[[nodiscard]] BundleMomentum CalculateBundleMomentum(const DynamicStrands& dynamic_strands);
[[nodiscard]] BundleExperimentDiagnostics CaptureBundleExperimentDiagnostics(const std::string& experiment,
                                                                             const DynamicStrands& dynamic_strands,
                                                                             const DsBundle& bundle,
                                                                             const BundleMomentum& reference_momentum);

}  // namespace eco_sys_lab_package
