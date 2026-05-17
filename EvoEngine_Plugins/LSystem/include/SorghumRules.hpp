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

namespace l_system_plugin {

// Sorghum thermal calendar conversion factor: GDD per chronological year.
// Anchored loosely to base-10degC sorghum thermal time (~1500 GDD per
// ~120-day vegetative cycle). Used by SorghumGrowthModel to convert
// chronological clock advances into the GDD axis used by topology rules
// (parity with kPineGddPerYear in ScotsPineDescriptor.hpp).
constexpr float kSorghumGddPerYear = 1500.0f;

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
  evo_engine::PlottedDistribution<float> leaf_blade_max_width;  ///< (m)
  evo_engine::PlottedDistribution<float> leaf_sheath_length;    ///< (m)
  evo_engine::PlottedDistribution<float> leaf_insertion_angle;  ///< (deg)
  evo_engine::PlottedDistribution<float> leaf_roll_angle;       ///< (deg) extra azimuth on top of distichous base
  evo_engine::PlottedDistribution<float> leaf_curling;          ///< (deg) per SorghumLeafState
  evo_engine::PlottedDistribution<float> leaf_bending;          ///< (deg)
  evo_engine::PlottedDistribution<float> leaf_waviness;         ///< amplitude
  float leaf_waviness_frequency = 8.0f;                          ///< constant cycles/length

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
  evo_engine::PlottedDistribution<float> leaf_blade_growth_curve;
  evo_engine::PlottedDistribution<float> leaf_width_growth_curve;
  evo_engine::PlottedDistribution<float> leaf_angle_development_curve;
  evo_engine::PlottedDistribution<float> leaf_curling_development_curve;
  evo_engine::PlottedDistribution<float> leaf_bending_development_curve;

  // Cross-section profiles consumed by the leaf mesher (Phase E).
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

}  // namespace l_system_plugin
