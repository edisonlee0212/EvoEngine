#pragma once

#include "DerivationEngine.hpp"
#include "LSystemRuleHelpers.hpp"  // SampleDistribution, SamplePlotted, SampleUnit01,
                                   // HashNodeSeed, MakeNodeRng, TropismEntry, SampledTropism.
#include "ProductionRule.hpp"
#include "SorghumModules.hpp"
#include <Plot2D.hpp>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <random>
#include <vector>

namespace l_system_package {

// Sorghum thermal calendar conversion factor: GDD per chronological year.
// Anchored loosely to base-10degC sorghum thermal time (~1500 GDD per
// ~120-day vegetative cycle). Used by SorghumGrowthModel to convert
// chronological clock advances into the GDD axis used by topology rules
// (parity with kPineGddPerYear in ScotsPineDescriptor.hpp).
constexpr float kSorghumGddPerYear = 1500.0f;
constexpr float kSorghumTillerPhyllotaxisAngleDeg = 137.5f;

inline float NormalizeSorghumDegrees(const float degrees) {
  float wrapped = std::fmod(degrees, 360.0f);
  if (wrapped < 0.0f) {
    wrapped += 360.0f;
  }
  return wrapped;
}

inline float ComputeSorghumTillerBudAzimuth(const int index,
                                            const float branch_azimuth_offset) {
  return NormalizeSorghumDegrees(
      branch_azimuth_offset +
      kSorghumTillerPhyllotaxisAngleDeg * static_cast<float>(std::max(0, index)));
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
  int total_phytomer_count = 14;          ///< Total phytomers on main culm.
  float phyllotaxis_angle = 180.0f;       ///< Distichous default; cultivar-tunable.
  float branch_azimuth_offset = 0.0f;     ///< Roll offset of the first lateral.

  // ===== Internode morphology (rank-indexed plotted distributions) =====
  evo_engine::PlottedDistribution<float> internode_length;     ///< (m) by rank
  evo_engine::PlottedDistribution<float> internode_thickness;  ///< (m) by rank

  // ===== Leaf morphology (rank-indexed plotted distributions) =====
  evo_engine::PlottedDistribution<float> leaf_blade_length;     ///< (m)
  evo_engine::PlottedDistribution<float> leaf_blade_max_width;  ///< [deprecated] legacy absolute blade width (m)
  evo_engine::PlottedDistribution<float> leaf_sheath_length;    ///< (m)
  evo_engine::PlottedDistribution<float> leaf_neck_length;      ///< (m)
  evo_engine::PlottedDistribution<float> leaf_sheath_end_width_ratio;  ///< ratio at sheath u=1
  evo_engine::PlottedDistribution<float> leaf_neck_end_width_ratio;    ///< ratio at neck u=1
  evo_engine::PlottedDistribution<float> leaf_blade_end_width_ratio;   ///< ratio at blade u=1
  evo_engine::PlottedDistribution<float> leaf_insertion_angle;  ///< (deg)
  evo_engine::PlottedDistribution<float> leaf_roll_angle;       ///< (deg) extra azimuth on top of distichous base
  evo_engine::PlottedDistribution<float> leaf_curling;          ///< (deg) per SorghumLeafState
  evo_engine::PlottedDistribution<float> leaf_bending;          ///< (deg)
  evo_engine::PlottedDistribution<float> leaf_waviness;         ///< amplitude
  float leaf_waviness_frequency = 8.0f;                          ///< constant cycles/length
  float leaf_sheath_radius_ratio = 1.05f;                        ///< [deprecated] min sheath radius / internode radius
  float leaf_blade_stage1_length_ratio = 0.33f;                  ///< [deprecated] blade internal stage split
  float leaf_blade_stage2_length_ratio = 0.34f;                  ///< [deprecated] blade internal stage split
  float leaf_blade_stage3_length_ratio = 0.33f;                  ///< [deprecated] blade internal stage split
  float leaf_blade_stage1_width_scale = 0.85f;                   ///< [deprecated] blade internal stage width
  float leaf_blade_stage2_width_scale = 1.0f;                    ///< [deprecated] blade internal stage width
  float leaf_blade_stage3_width_scale = 0.4f;                    ///< [deprecated] blade internal stage width

  // ===== Leaf lifecycle (chronological, mirrors ScotsPine) =====
  evo_engine::SingleDistribution<float> leaf_lifespan_years{2.5f};
  evo_engine::SingleDistribution<float> leaf_wilting_years{0.5f};

  // ===== Tillering =====
  int tiller_count = 3;
  evo_engine::PlottedDistribution<float> tiller_initiation_delay_gdd; ///< per-bud dormancy budget
  float tiller_insertion_angle = 30.0f;
  float tiller_phytomer_count_scale = 0.7f;
  float tiller_thickness_ratio = 0.6f;

  // ===== Thermal block (verbatim layout from SampledTasselParams) =====
  float plastochron_gdd = 50.0f;
  float maturity_gdd = 600.0f;            ///< Thermal age at organ maturity.
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
  evo_engine::PlottedDistribution<float> curling_along_leaf;
  evo_engine::PlottedDistribution<float> waviness_along_leaf;

  // ===== Tropisms =====
  std::vector<SampledTropism> tropisms;
};

// ---------------------------------------------------------------------------
// Rule type aliases
// ---------------------------------------------------------------------------

using SorghumRule = ProductionRule<SorghumGraph, SorghumModuleData>;
using SorghumEngine = DerivationEngine<SorghumGraphData, SorghumFlowData, SorghumModuleData>;

inline float ComputeSorghumTillerInternodeBranchAngle(const int rank,
                                                      const int total_tiller_phytomers,
                                                      const float insertion_angle) {
  const float basal_angle = std::max(25.0f, insertion_angle);
  if (rank <= 0) {
    return basal_angle;
  }

  const int recovery_ranks = std::min(2, std::max(0, total_tiller_phytomers - 1));
  if (rank <= recovery_ranks) {
    return -basal_angle / static_cast<float>(recovery_ranks);
  }
  return 0.0f;
}

// ---------------------------------------------------------------------------
// Rule factories (implemented in Phase C / SorghumRules.cpp).
//
// CreateSorghumTopologyRules returns:
//   R-Init-Tillers           — single-shot at t=0 on root Apex(order=0).
//   R-Apex-Phytomer-Order0   — main-culm phytomer emission.
//   R-Apex-Terminate         — Apex(vigor<=0) -> PanicleBud placeholder.
//   R-TillerBud-Activate     — TillerBud -> order-1 Apex when dormancy expires.
//   R-Apex-Phytomer-Order1   — tiller phytomer emission (lateral curves).
//
// CreateSorghumGrowthRules returns:
//   G-Apex-Age               — apex.age_gdd += gdd_step; bud dormancy decremented.
//   G-Internode              — interpolate length/thickness toward target.
//   G-Leaf-Thermal           — pre-maturity leaf growth + maturity stamp.
//   G-Leaf-Chronological-Senescence — post-maturity wilting/abscission.
// ---------------------------------------------------------------------------

std::vector<SorghumRule> CreateSorghumTopologyRules(const SampledSorghumParams& params);
std::vector<SorghumRule> CreateSorghumGrowthRules(const SampledSorghumParams& params);

}  // namespace l_system_package
