#pragma once

#include "DerivationEngine.hpp"
#include "LSystemRuleHelpers.hpp"  // SampleDistribution, SamplePlotted, SampleUnit01,
                                   // HashNodeSeed, MakeNodeRng, TropismEntry, SampledTropism.
#include <Plot2D.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <random>
#include <vector>
#include "ProductionRule.hpp"
#include "SorghumModules.hpp"

namespace l_system_package {

// Sorghum thermal calendar conversion factor: GDD per chronological year.
// Anchored loosely to base-10degC sorghum thermal time (~1500 GDD per
// ~120-day vegetative cycle). Used by SorghumGrowthModel to convert
// chronological clock advances into the GDD axis used by topology rules
// (parity with kPineGddPerYear in ScotsPineDescriptor.hpp).
constexpr float kSorghumGddPerYear = 1500.0f;

inline float NormalizeSorghumDegrees(const float degrees) {
  float wrapped = std::fmod(degrees, 360.0f);
  if (wrapped < 0.0f) {
    wrapped += 360.0f;
  }
  return wrapped;
}

// ---------------------------------------------------------------------------
// SampledSorghumParams
//
// Concrete per-instance parameter set produced by SorghumLSDescriptor::Sample.
// Owns PlottedDistributions by value (rule lambdas re-evaluate them per
// emission to remain ScalarRandom-deterministic; same idiom as Pine/Tassel).
// ---------------------------------------------------------------------------
struct SampledSorghumParams {
  // ===== Culm topology =====
  int total_phytomer_count = 14;       ///< Total phytomers on main culm.
  float phyllotaxis_angle = 180.0f;    ///< Distichous default; cultivar-tunable.
  float branch_azimuth_offset = 0.0f;  ///< Roll offset of the first lateral.

  // ===== Internode morphology (rank-indexed plotted distributions) =====
  evo_engine::PlottedDistribution<float> internode_length;     ///< (m) by rank
  evo_engine::PlottedDistribution<float> internode_thickness;  ///< (m) by rank

  // ===== Leaf morphology (rank-indexed plotted distributions) =====
  evo_engine::PlottedDistribution<float> leaf_blade_length;            ///< (m)
  evo_engine::PlottedDistribution<float> leaf_blade_max_width;         ///< mature full blade width (m)
  evo_engine::PlottedDistribution<float> leaf_blade_thickness;         ///< mature blade thickness (m)
  evo_engine::PlottedDistribution<float> leaf_sheath_thickness;        ///< mature sheath wall thickness (m)
  float leaf_width_scale = 1.0f;                                       ///< legacy migration multiplier
  evo_engine::PlottedDistribution<float> leaf_sheath_length;           ///< (m)
  evo_engine::PlottedDistribution<float> leaf_neck_length;             ///< (m)
  evo_engine::PlottedDistribution<float> leaf_sheath_end_width_ratio;  ///< ratio at sheath u=1
  evo_engine::PlottedDistribution<float> leaf_neck_end_width_ratio;    ///< ratio at neck u=1
  evo_engine::PlottedDistribution<float> leaf_blade_end_width_ratio;   ///< ratio at blade u=1
  evo_engine::PlottedDistribution<float> leaf_insertion_angle;         ///< (deg)
  evo_engine::PlottedDistribution<float> leaf_roll_angle;  ///< (deg) extra azimuth on top of distichous base
  evo_engine::PlottedDistribution<float> leaf_curling;     ///< (deg) per SorghumLeafState
  evo_engine::PlottedDistribution<float> leaf_bending;     ///< (deg)
  evo_engine::PlottedDistribution<float> leaf_waviness;    ///< amplitude
  float leaf_waviness_frequency = 8.0f;                    ///< constant cycles/length
  float leaf_sheath_radius_ratio = 1.05f;
  float leaf_sheath_wrap_angle = 390.0f;
  float leaf_blade_stage1_length_ratio = 0.33f;  ///< [deprecated] blade internal stage split
  float leaf_blade_stage2_length_ratio = 0.34f;  ///< [deprecated] blade internal stage split
  float leaf_blade_stage3_length_ratio = 0.33f;  ///< [deprecated] blade internal stage split
  float leaf_blade_stage1_width_scale = 0.85f;   ///< [deprecated] blade internal stage width
  float leaf_blade_stage2_width_scale = 1.0f;    ///< [deprecated] blade internal stage width
  float leaf_blade_stage3_width_scale = 0.4f;    ///< [deprecated] blade internal stage width

  // ===== Leaf lifecycle (chronological, mirrors ScotsPine) =====
  evo_engine::SingleDistribution<float> leaf_lifespan_years{2.5f};
  evo_engine::SingleDistribution<float> leaf_wilting_years{0.5f};

  // ===== Tillering =====
  int tiller_count = 4;
  std::vector<int> tiller_origin_ranks;
  std::array<int, 6> tiller_emergence_main_leaf_stages{5, 5, 6, 7, 8, 9};
  evo_engine::SingleDistribution<float> tiller_insertion_angle{35.0f};
  evo_engine::SingleDistribution<float> tiller_final_lean_angle{0.0f};
  evo_engine::SingleDistribution<float> tiller_azimuth_jitter{0.0f};
  float tiller_recovery_axis_fraction = 1.0f;
  evo_engine::SingleDistribution<float> tiller_leaf_count_ratio{0.90f, 0.03f};
  evo_engine::SingleDistribution<float> tiller_height_ratio{0.90f, 0.03f};
  evo_engine::PlottedDistribution<float> tiller_leaf_area_ratio_by_origin;
  evo_engine::SingleDistribution<float> tiller_thickness_ratio{0.80f};
  float tiller_max_axis_length_ratio = 1.10f;

  // ===== Thermal block (verbatim layout from SampledTasselParams) =====
  float plastochron_gdd = 50.0f;
  float maturity_gdd = 600.0f;  ///< Thermal age at organ maturity.
  float main_axis_plastochron_scale = 1.0f;
  float lateral_axis_plastochron_scale = 1.0f;
  float lateral_bud_plastochron_scale = 1.0f;
  float maturity_initiation_coupling = 0.0f;
  float reference_maturity_gdd = 600.0f;
  float gdd_step = 1.0f;

  // ===== Growth curves =====
  evo_engine::PlottedDistribution<float> internode_elongation_curve;
  evo_engine::PlottedDistribution<float> internode_thickness_curve;
  evo_engine::PlottedDistribution<float> leaf_sheath_length_growth_curve;
  evo_engine::PlottedDistribution<float> leaf_neck_length_growth_curve;
  evo_engine::PlottedDistribution<float> leaf_blade_growth_curve;
  evo_engine::PlottedDistribution<float> leaf_sheath_width_growth_curve;
  evo_engine::PlottedDistribution<float> leaf_neck_width_growth_curve;
  evo_engine::PlottedDistribution<float> leaf_width_growth_curve;
  evo_engine::PlottedDistribution<float> leaf_angle_development_curve;
  evo_engine::PlottedDistribution<float> leaf_curling_development_curve;
  evo_engine::PlottedDistribution<float> leaf_bending_development_curve;

  // Cross-section profiles consumed by the leaf mesher (Phase E).
  evo_engine::PlottedDistribution<float> width_along_sheath;
  evo_engine::PlottedDistribution<float> width_along_neck;
  evo_engine::PlottedDistribution<float> width_along_leaf;
  evo_engine::PlottedDistribution<float> bending_along_leaf;
  evo_engine::PlottedDistribution<float> curling_along_leaf;
  evo_engine::PlottedDistribution<float> waviness_along_leaf;

  // ===== Tropisms =====
  std::vector<SampledTropism> tropisms;
};

inline int ComputeSorghumTillerSelectionIndex(const SampledSorghumParams& params, const int origin_rank) {
  const auto it = std::find(params.tiller_origin_ranks.begin(), params.tiller_origin_ranks.end(), origin_rank);
  return it == params.tiller_origin_ranks.end()
             ? -1
             : static_cast<int>(std::distance(params.tiller_origin_ranks.begin(), it));
}

inline int ComputeSorghumTillerLeafBudget(const int main_leaf_count, const float leaf_count_ratio) {
  const int main_count = std::max(1, main_leaf_count);
  const int minimum = std::max(1, static_cast<int>(std::ceil(0.75f * main_count)));
  const int maximum = std::max(minimum, static_cast<int>(std::floor(1.05f * main_count)));
  return std::clamp(static_cast<int>(std::round(main_count * leaf_count_ratio)), minimum, maximum);
}

inline float ComputeSorghumAxisRankPosition(const int rank, const int axis_phytomer_count) {
  if (axis_phytomer_count <= 1)
    return 0.5f;
  return std::clamp(static_cast<float>(rank) / static_cast<float>(axis_phytomer_count - 1), 0.0f, 1.0f);
}

inline float ComputeSorghumTillerCatchUpPlastochron(const float main_phyllochron, const int remaining_main_phytomers,
                                                    const int tiller_leaf_budget,
                                                    const float lateral_axis_plastochron_scale) {
  return std::max(1.0f, std::max(1.0f, main_phyllochron) * static_cast<float>(std::max(1, remaining_main_phytomers)) /
                            static_cast<float>(std::max(1, tiller_leaf_budget)) *
                            std::max(0.1f, lateral_axis_plastochron_scale));
}

// ---------------------------------------------------------------------------
// Rule type aliases
// ---------------------------------------------------------------------------

using SorghumRule = ProductionRule<SorghumGraph, SorghumModuleData>;
using SorghumEngine = DerivationEngine<SorghumGraphData, SorghumFlowData, SorghumModuleData>;

inline float ComputeSorghumTillerInternodeBranchAngle(const int rank, const int axis_phytomer_count,
                                                      const float insertion_angle, const float final_lean_angle = 15.0f,
                                                      const float recovery_axis_fraction = 1.0f) {
  const float basal_angle = std::max(0.0f, insertion_angle);
  if (rank <= 0) {
    return basal_angle;
  }
  const int recovery_ranks =
      std::max(1, static_cast<int>(std::ceil(static_cast<float>(std::max(2, axis_phytomer_count) - 1) *
                                             std::clamp(recovery_axis_fraction, 0.05f, 1.0f))));
  if (rank <= recovery_ranks) {
    const auto smoothstep = [](const float t) {
      const float clamped = std::clamp(t, 0.0f, 1.0f);
      return clamped * clamped * (3.0f - 2.0f * clamped);
    };
    const float previous = smoothstep(static_cast<float>(rank - 1) / static_cast<float>(recovery_ranks));
    const float current = smoothstep(static_cast<float>(rank) / static_cast<float>(recovery_ranks));
    return -(basal_angle - final_lean_angle) * (current - previous);
  }
  return 0.0f;
}

// ---------------------------------------------------------------------------
// Rule factories (implemented in SorghumRules.cpp).
//
// CreateSorghumTopologyRules returns:
//   R-Apex-Phytomer-Order0 - main-culm phytomer and axillary-bud emission.
//   R-Apex-Terminate       - Apex(vigor<=0) -> terminal placeholder.
//   R-TillerBud-Activate   - TillerBud -> order-1 Apex at its main-leaf stage.
//   R-Apex-Phytomer-Order1 - tiller phytomer emission (rank-aligned curves).
//
// CreateSorghumGrowthRules returns:
//   G-Apex-Age     - apex.age_gdd += gdd_step.
//   G-Internode    - interpolate length/thickness toward target.
//   G-Leaf-Thermal - pre-maturity organ development.
// ---------------------------------------------------------------------------

std::vector<SorghumRule> CreateSorghumTopologyRules(const SampledSorghumParams& params);
std::vector<SorghumRule> CreateSorghumGrowthRules(const SampledSorghumParams& params);

}  // namespace l_system_package
