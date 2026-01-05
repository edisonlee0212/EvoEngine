#pragma once

#include <memory>
#include <string>
#include <vector>

namespace eco_sys_lab_package {

class Tree;

/// Shared Kinetic volumetric meshing demos (checklist + UI buttons + headless precompute).
enum class VolumetricTreeExperimentId {
  SmallTrunk = 0,
  SmallTrunkAlpha1e9,
  NormalTrunk,
  StockyTrunk,
  OakThickStump100,
  OakThickStump200,
  OakThickStump300,
  OakThickStump400,
  OakThickStump100Alpha10,
  OakThickStump200Alpha10,
  OakThickStump300Alpha10,
  OakThickStump400Alpha10,
  OakThickStump100Alpha20,
  OakThickStump200Alpha20,
  OakThickStump300Alpha20,
  OakThickStump400Alpha20,
  OakTwoYearSparse,
  OakTwoYear20,
  OakTwoYear50,
  OakTwoYear100,
  OakThreeYear4,
  OakThreeYear10,
  OakThreeYear20,
  OakThreeYear50,
  OakFourYearSparse,
  OakSixYearSparse,
  OakEightYearSparse,
  Count
};

struct VolumetricTreeExperiment {
  VolumetricTreeExperimentId id = VolumetricTreeExperimentId::SmallTrunk;
  /// Stable CLI / log id (no spaces), e.g. "SmallTrunk".
  const char* cli_id = "";
  /// UI label, e.g. "Small Trunk".
  const char* display_name = "";
  /// Project-relative tree descriptor asset path.
  const char* tree_descriptor_path = "";
  /// Growth years when @ref growth_iterations is 0.
  float growth_years = 0.f;
  /// When > 0, EcoSysLab auto-grow uses iteration count instead of years.
  int growth_iterations = 0;
  /// Strand preset: trunk full-process / stocky / oak stump with this end_node_strands.
  enum class StrandPreset {
    OakTrunkFullProcess,
    StockyTrunk,
    OakThickStump
  } strand_preset = StrandPreset::OakTrunkFullProcess;
  int end_node_strands = 100;
  /// Applied as both @c alpha_cutoff and @c branch_alpha_cutoff for OakThickStump presets.
  double alpha_cutoff = 10.0;
  /// Fixed RNG seed for tree growth + strand subdivision (must be >= 0 for deterministic MeshBuffers hashes).
  int seed = 0;
  float min_segment_length = 0.005f;
  float max_segment_length = 0.01f;
  /// Free-form note stored in MeshBuffers YML metadata (same as demo buttons).
  const char* meshing_buffer_description = "";
  const char* tooltip = "";
};

[[nodiscard]] const std::vector<VolumetricTreeExperiment>& GetVolumetricTreeExperiments();
[[nodiscard]] const VolumetricTreeExperiment* FindVolumetricTreeExperiment(const char* cli_id);
[[nodiscard]] const VolumetricTreeExperiment* FindVolumetricTreeExperiment(VolumetricTreeExperimentId id);

void ApplyVolumetricTreeStrandPreset(const VolumetricTreeExperiment& experiment, const std::shared_ptr<Tree>& tree);

}  // namespace eco_sys_lab_package
