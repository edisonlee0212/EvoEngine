#pragma once

#include <IAsset.hpp>
#include <Plot2D.hpp>
#include "LSystemRuleHelpers.hpp"
#include "ParamSpaceExplorer.hpp"
#include "ILSystemExplorableDescriptor.hpp"
#include <cstdint>
#include <glm/vec4.hpp>
#include <random>
#include <vector>

namespace l_system_plugin {

/// 1 simulated year ~= 1500 GDD (base 5C, mid-latitude). Used as the unit
/// conversion between physiological-time fields (GDD on disk) and the
/// year-based clock used in production rules and growth state.
constexpr float kPineGddPerYear = 1500.0f;

/**
 * @brief Per-emission sampling distributions copied from the descriptor.
 *
 * Carried inside `SampledPineParams` so that production rules can resample
 * each parameter at every consumption site (per phytomer, per whorl bud,
 * per needle cluster) using a per-node RNG, instead of being locked to the
 * single plant-wide draw stored in the scalar fields below.
 *
 * Field semantics: each member is a copy of the descriptor's matching
 * SingleDistribution<float>. Rules call `SampleDistribution(params.distributions.X, node_rng)`
 * inside production lambdas. Time-domain fields stay in GDD (callers convert
 * to years via `kPineGddPerYear`) and integer-valued fields are sampled as
 * floats and rounded by the rule (matching `Sample()`'s clamp/round contract).
 */
struct PineSamplingDistributions {
  // Phytomer scheduling.
  evo_engine::SingleDistribution<float> max_branching_order;
  evo_engine::SingleDistribution<float> plastochron_gdd;
  evo_engine::SingleDistribution<float> max_phytomers_per_seasonal_growth;
  // Whorl architecture.
  evo_engine::SingleDistribution<float> branches_per_whorl;
  // Overwintering dormancy is a chronological/photoperiod process, not a
  // heat-sum process, so this is in years (not GDD). See FSPM Rule of Ontogeny.
  evo_engine::SingleDistribution<float> whorl_dormancy_years;
  evo_engine::SingleDistribution<float> branch_insertion_angle_deg;
  evo_engine::SingleDistribution<float> branch_roll_phyllotaxis_deg;
  // Phytomer dimensions.
  evo_engine::SingleDistribution<float> internode_length_m;
  evo_engine::SingleDistribution<float> leader_internode_thickness_m;
  evo_engine::SingleDistribution<float> lateral_length_ratio;
  evo_engine::SingleDistribution<float> lateral_thickness_ratio;
  // Maturity-driven organ shape multipliers (x=normalized maturity age, y=[0,1]).
  evo_engine::PlottedDistribution<float> internode_length_maturity_curve;
  evo_engine::PlottedDistribution<float> internode_width_maturity_curve;
  // Needles.
  evo_engine::SingleDistribution<float> bare_zone_fraction;
  evo_engine::SingleDistribution<float> needle_count_per_cluster;
  evo_engine::SingleDistribution<float> needle_length_m;
  evo_engine::PlottedDistribution<float> needle_length_maturity_curve;
  evo_engine::SingleDistribution<float> needle_cross_section_width_max_m;
  evo_engine::SingleDistribution<float> needle_cross_section_thickness_max_m;
  evo_engine::PlottedDistribution<float> needle_cross_section_width_profile;
  evo_engine::PlottedDistribution<float> needle_cross_section_thickness_profile;
  evo_engine::PlottedDistribution<float> needle_cross_section_temporal_maturity_curve;
  // Needle senescence is post-maturity chronological aging, not heat-sum
  // driven. Stored in years (see FSPM Rule of Ontogeny).
  evo_engine::SingleDistribution<float> needle_lifespan_years;
  evo_engine::SingleDistribution<float> needle_browning_years;
  evo_engine::SingleDistribution<float> needle_flush_delay_gdd;
  evo_engine::SingleDistribution<float> internode_maturation_gdd;
  evo_engine::SingleDistribution<float> needle_maturation_gdd;
  evo_engine::SingleDistribution<float> needle_branching_angle_deg;
  evo_engine::SingleDistribution<float> needle_branching_relax_gdd;
  // Needle curvature.
  evo_engine::SingleDistribution<float> needle_curvature_adaxial_bias;
  evo_engine::SingleDistribution<float> needle_curvature_abaxial_bias;
  evo_engine::SingleDistribution<float> needle_curvature_gradient_per_arclen;
  evo_engine::SingleDistribution<float> needle_diameter_for_curvature_m;
  evo_engine::SingleDistribution<float> needle_sinusoidal_amplitude_deg;
  evo_engine::SingleDistribution<float> needle_sinusoidal_frequency_cycles;
  evo_engine::SingleDistribution<float> needle_sinusoidal_phase_randomness_deg;
  // Needle mechanics.
  evo_engine::SingleDistribution<float> needle_young_modulus_baseline_Pa;
  evo_engine::SingleDistribution<float> needle_lignification_maturation_years;
  evo_engine::SingleDistribution<float> needle_density_kg_m3;
  // Per-needle CV.
  evo_engine::SingleDistribution<float> needle_per_needle_length_cv;
  evo_engine::SingleDistribution<float> needle_per_needle_curvature_cv;
  evo_engine::SingleDistribution<float> needle_per_needle_radius_cv;
  evo_engine::SingleDistribution<float> needle_per_needle_modulus_cv;
  evo_engine::SingleDistribution<float> needle_per_needle_density_cv;
  evo_engine::SingleDistribution<float> needle_per_needle_wave_amplitude_cv;
  evo_engine::SingleDistribution<float> needle_per_needle_wave_frequency_cv;
  evo_engine::SingleDistribution<float> needle_per_needle_wave_phase_cv;
  // Tropism.
  evo_engine::SingleDistribution<float> gravitropism_first_order;
  // Per-shoot CV.
  evo_engine::SingleDistribution<float> internode_length_per_node_cv;
  evo_engine::SingleDistribution<float> internode_thickness_per_node_cv;
  evo_engine::SingleDistribution<float> branch_angle_per_node_sigma_deg;
  evo_engine::SingleDistribution<float> roll_phyllotaxis_per_node_sigma_deg;
};

/**
 * @brief Sampled, deterministic Scots pine parameter set produced by
 *        ScotsPineDescriptor::Sample at the start of a derivation run.
 *
 * Phytomer model: an immortal PineApex emits one phytomer per plastochron
 * during its active growth season, capped at `max_phytomers_per_seasonal_growth`
 * phytomers per year. The first `bare_zone_fraction` of each year's seasonal
 * growth produces internode-only phytomers (no needle cluster); subsequent
 * phytomers each carry one needle cluster of `needle_count_per_cluster`
 * needles.
 */
struct SampledPineParams {
  // -- Phytomer scheduling --
  int max_branching_order = 1;              ///< 0 = leader only, 1 = primary laterals, ...
  float plastochron_years = 1.0f;           ///< Physiological time between consecutive phytomer events.
  int max_phytomers_per_seasonal_growth = 12; ///< Phytomers emitted per active season before apex pauses.

  // -- Whorl architecture --
  int branches_per_whorl = 6;
  float whorl_dormancy_years = 1.0f;        ///< Overwintering = 1 year for Pinus sylvestris.
  float branch_insertion_angle_deg = 70.0f;
  float branch_roll_phyllotaxis_deg = 137.5f; ///< Golden angle (also drives needle phyllotaxis).

  // -- Phytomer dimensions (SI metres) --
  float internode_length_m = 0.012f;        ///< Length of one phytomer's internode.
  float leader_internode_thickness_m = 0.0015f; ///< Main stem thickness (diameter), metres.
  float lateral_length_ratio = 0.7f;        ///< lateral length = leader * ratio^order.
  float lateral_thickness_ratio = 0.6f;

  // -- Needles --
  float bare_zone_fraction = 0.08f;          ///< Temporal fraction at the start of each year that emits internode-only phytomers [0, 0.95).
  int needle_count_per_cluster = 2;         ///< Pinus sylvestris fascicle.
  int needle_segment_count = 11;            ///< Longitudinal segments per needle (mesh stations = segments + 1).
  float needle_length_m = 0.025f;
  int needle_lifespan_years = 4;
  float needle_browning_years = 0.8f;       ///< Years from senescence onset to abscission.
  float needle_flush_delay_years = 0.08f;    ///< Delay from phytomer emergence to needle flush start.
  float internode_maturation_years = 0.053f; ///< Time from emergence to mature internode length.
  float needle_maturation_years = 0.093f;    ///< Time from flush to mature needle length.
  float needle_cross_section_width_max_m = 0.0018f;      ///< Major-axis full width (diameter) at the broadest position.
  float needle_cross_section_thickness_max_m = 0.0011f;  ///< Minor-axis full thickness (diameter) at the broadest position.
  float needle_order_length_attenuation = 0.12f;   ///< Linear reduction per branch order.
  float needle_order_radius_attenuation = 0.10f;   ///< Linear thickness reduction per branch order.
  float needle_order_min_length_scale = 0.55f;     ///< Floor after order attenuation.
  float needle_order_min_radius_scale = 0.60f;     ///< Thickness floor after order attenuation.
  // Intra-year initiation-time capacity mapping (per phytomer index in season).
  float needle_intra_year_base_ratio = 0.35f;                 ///< Early-season lower bound in [0,1].
  float needle_intra_year_sigmoid_steepness = 8.0f;           ///< Steepness k for logistic ramp.
  float needle_intra_year_sigmoid_midpoint_fraction = 0.45f;  ///< Midpoint m in normalized index [0,1].
  float needle_intra_year_late_decay_start_fraction = 1.0f;   ///< >=1 disables late-season decay.
  float needle_intra_year_late_decay_end_scale = 1.0f;        ///< End-of-season multiplicative scale.
  // Inter-year capacity mapping (primary -> fascicular transition).
  int needle_fascicular_start_year = 1;                        ///< Year index where fascicular multipliers activate.
  float needle_year2plus_length_multiplier = 1.0f;            ///< Length multiplier for year >= start_year.
  float needle_year2plus_width_multiplier = 1.0f;             ///< Width multiplier for year >= start_year.
  float needle_year2plus_thickness_multiplier = 1.0f;         ///< Thickness multiplier for year >= start_year.
  float needle_lignification_factor_year1 = 0.75f;            ///< Year-1 lignification response multiplier.
  float needle_lignification_factor_year2plus = 1.15f;        ///< Year-2+ lignification response multiplier.
  float needle_stomatal_strip_density_year1 = 0.35f;          ///< Procedural strip density proxy for year-1 cohorts [0,1].
  float needle_stomatal_strip_density_year2plus = 0.65f;      ///< Procedural strip density proxy for year-2+ cohorts [0,1].
  float needle_basal_taper_ratio_year1 = 0.82f;               ///< Radius multiplier at needle base for year-1 cohorts.
  float needle_basal_taper_ratio_year2plus = 0.92f;           ///< Radius multiplier at needle base for year-2+ cohorts.
  float needle_fascicle_sheath_budget_years = 0.28f;          ///< Characteristic years for sheath visual maturation near base.
  float needle_specularity_plasticity_year1 = 0.25f;          ///< How much micro-variation tracks maturity in year-1 cohorts [0,1].
  float needle_specularity_plasticity_year2plus = 0.65f;      ///< How much micro-variation tracks maturity in year-2+ cohorts [0,1].
  // Bud-storage proxy coupling from previous-year completion to current-year potential.
  float needle_bud_storage_vigor_strength = 0.0f;             ///< 0 = disabled, 1 = fully applied.
  float needle_bud_storage_completion_floor = 0.60f;          ///< Lower clamp for completion-derived vigor.
  /// [deprecated] Legacy cap-ratio field kept for descriptor compatibility.
  /// Needle width/thickness are now uncapped and this value is ignored.
  float needle_radius_to_stem_thickness_max_ratio = 0.45f;

  // -- Needle bilateral curvature (default near-inert; diameter > 0 activates) --
  float needle_curvature_adaxial_bias = 0.003f;
  float needle_curvature_abaxial_bias = 0.010f;
  float needle_curvature_gradient_per_arclen = 0.0015f;
  float needle_diameter_for_curvature_m = 0.001f;
  float needle_sinusoidal_amplitude_deg = 0.0f;
  float needle_sinusoidal_frequency_cycles = 0.0f;
  float needle_sinusoidal_phase_randomness_deg = 0.0f;

  // -- Needle mechanics (defaults inert -> elastica skipped) --
  float needle_young_modulus_baseline_Pa = 0.0f;
  float needle_lignification_maturation_years = 0.0f;
  float needle_density_kg_m3 = 0.0f;
  float gravity_m_s2 = 0.0f;

  // -- Per-needle variability (CV-style multipliers within each fascicle) --
  float needle_per_needle_length_cv = 0.0f;
  float needle_per_needle_curvature_cv = 0.0f;
  float needle_per_needle_radius_cv = 0.0f;
  float needle_per_needle_modulus_cv = 0.0f;
  float needle_per_needle_density_cv = 0.0f;
  float needle_per_needle_wave_amplitude_cv = 0.0f;
  float needle_per_needle_wave_frequency_cv = 0.0f;
  float needle_per_needle_wave_phase_cv = 0.0f;

  // -- Tropism --
  float gravitropism_first_order = 0.0001f; ///< deg/GDD applied to leader internodes (order 0) only.

  // -- Per-shoot stochastic noise (CV / sigma; 0 = deterministic) --
  float internode_length_per_node_cv = 0.0f;
  float internode_thickness_per_node_cv = 0.0f;
  float branch_angle_per_node_sigma_deg = 0.0f;
  float roll_phyllotaxis_per_node_sigma_deg = 0.0f;

  // -- Plant-wide --
  float plant_random = 0.5f;
  float initial_orientation_yaw_deg = 0.0f;  ///< Sampled once per plant; applied as root yaw around +Y.

  // -- Sampled tropism array (used by future growth-step tropism integration) --
  std::vector<SampledTropism> tropisms;

  /// Copies of the descriptor's distributions, used by production rules to
  /// resample each parameter at every consumption site (per phytomer, per
  /// whorl bud, per needle cluster). The scalar fields above remain the
  /// "plant identity" baseline used by PineGrowthModel for plant-wide
  /// initialization (root apex stamping, gravity, etc.); the rules read
  /// from `distributions` for per-emission stochasticity.
  PineSamplingDistributions distributions;
};

/**
 * @brief Genotype asset for Scots pine (Pinus sylvestris) L-system generation.
 *
 * Annual-shoot architecture. Sampled per-instance to produce
 * SampledPineParams for the derivation engine.
 *
 * File extension: .spine
 */
class ScotsPineDescriptor : public evo_engine::IAsset, public ILSystemExplorableDescriptor {
 public:
  ScotsPineDescriptor();

  // ============================================================================
  // Field declarations are organized into 14 functional groups below, plus a
  // trailing [deprecated] block for fields with no current runtime consumer.
  // Time-domain fields stay in GDD on disk and are converted to years inside
  // Sample(). YAML key strings, RNG draw order in Sample(), and the explorer
  // axis count are intentionally unchanged by this reorganization to preserve
  // on-disk asset compatibility, seed reproducibility, and explorer cache
  // fingerprints.
  // ============================================================================

  // ===== 1. Plant identity & global time =====
  evo_engine::SingleDistribution<float> initial_orientation_yaw_deg{0.0f}; ///< Sampled once per plant; rotation around +Y.
  evo_engine::SingleDistribution<float> target_gdd{3000.0f};               ///< Per-instance target growth GDD.
  // GDD/day couples with LSystemLayer chronological day speed:
  //   delta_gdd = sampled_gdd_per_day * delta_days
  // where delta_days = chronological_days_per_second * dt.
  // Consumed by LSystemLayer::SamplePineTemporalParameters() and applied
  // in the per-pine update path. Mean of season-day fields is clamped to
  // [0, 365]; deviation is treated as integer days inside OnInspect.
  evo_engine::SingleDistribution<float> gdd_per_day{2.0f};                         ///< Per-pine thermal rate (GDD/day).
  evo_engine::SingleDistribution<float> growing_season_start_day{60.0f};           ///< Per-pine active season start (DOY 0-365).
  evo_engine::SingleDistribution<float> growing_season_end_day{334.0f};            ///< Per-pine active season end (DOY 0-365).

  // ===== 2. Phytomer scheduling =====
  evo_engine::SingleDistribution<float> max_branching_order{1.0f};
  evo_engine::SingleDistribution<float> plastochron_gdd{1500.0f};    ///< ~1 yr between consecutive phytomers.
  evo_engine::SingleDistribution<float> max_phytomers_per_seasonal_growth{12.0f};

  // ===== 3. Whorl architecture =====
  evo_engine::SingleDistribution<float> branches_per_whorl{6.0f};
  /// Overwintering dormancy for whorl buds. Chronological, NOT GDD: bud
  /// release is driven by chilling + photoperiod, not heat sums. Default
  /// 1 yr = annual Scots pine whorl cycle.
  evo_engine::SingleDistribution<float> whorl_dormancy_years{1.0f};
  evo_engine::SingleDistribution<float> branch_insertion_angle_deg{70.0f};
  evo_engine::SingleDistribution<float> branch_roll_phyllotaxis_deg{137.5f};

  // ===== 4. Stem geometry (SI metres) =====
  // Internode size, lateral attenuation, stem maturation, and the maturity
  // curves that drive stem length/width over age. (Previously scattered: the
  // `internode_length_maturity_curve` and `internode_width_maturity_curve`
  // fields were declared inside the Needles block, which was misleading.)
  evo_engine::SingleDistribution<float> internode_length_m{0.012f};
  evo_engine::SingleDistribution<float> leader_internode_thickness_m{0.0015f}; ///< Main stem width (diameter), metres.
  evo_engine::SingleDistribution<float> lateral_length_ratio{0.7f};
  evo_engine::SingleDistribution<float> lateral_thickness_ratio{0.6f};
  evo_engine::SingleDistribution<float> internode_maturation_gdd{80.0f};
  evo_engine::PlottedDistribution<float> internode_length_maturity_curve;     ///< Stem-axis curve (was in Needles).
  evo_engine::PlottedDistribution<float> internode_width_maturity_curve;      ///< Stem-axis curve (was in Needles).
  float internode_age_exponent = 1.0f;      ///< >1 delays visible stem aging toward max age.

  // ===== 5. Stem stochastic noise (per-shoot CV / sigma; 0 = deterministic) =====
  evo_engine::SingleDistribution<float> internode_length_per_node_cv{0.15f};
  evo_engine::SingleDistribution<float> internode_thickness_per_node_cv{0.0f};
  evo_engine::SingleDistribution<float> branch_angle_per_node_sigma_deg{0.0f};
  evo_engine::SingleDistribution<float> roll_phyllotaxis_per_node_sigma_deg{0.0f};

  // ===== 6. Stem tropism =====
  evo_engine::SingleDistribution<float> gravitropism_first_order{0.0001f}; ///< Main-stem-only curvature (deg/GDD).

  // ===== 7. Needle layout =====
  evo_engine::SingleDistribution<float> bare_zone_fraction{0.08f};
  evo_engine::SingleDistribution<float> needle_count_per_cluster{2.0f};
  int needle_segment_count = 11;  ///< Longitudinal segments per needle (mesh stations = segments + 1).
  evo_engine::SingleDistribution<float> needle_branching_angle_deg{72.0f};
  /// GDD-equivalent duration converted to years at node creation.
  /// Runtime applies this against chronological age so relaxation continues
  /// even when thermal accumulation is paused outside the active season.
  evo_engine::SingleDistribution<float> needle_branching_relax_gdd{220.0f};
  /// [deprecated] Legacy cap-ratio field kept for schema compatibility.
  /// Needle width/thickness are now uncapped and this value is ignored.
  float needle_radius_to_stem_thickness_max_ratio = 0.45f;

  // ===== 8. Needle lifecycle =====
  // Lifespan and browning are post-maturity chronological aging (FSPM Rule
  // of Ontogeny: once the organ has reached its final size, time -- not
  // heat -- governs its remaining biology). Flush delay and maturation are
  // active-expansion processes, so those stay in GDD.
  evo_engine::SingleDistribution<float> needle_lifespan_years{4.0f};
  evo_engine::SingleDistribution<float> needle_browning_years{0.8f}; ///< Years from senescence onset to abscission.
  evo_engine::SingleDistribution<float> needle_flush_delay_gdd{120.0f};
  evo_engine::SingleDistribution<float> needle_maturation_gdd{140.0f};

  // ===== 9. Needle dimensions & cross-section =====
  evo_engine::SingleDistribution<float> needle_length_m{0.025f};
  evo_engine::PlottedDistribution<float> needle_length_maturity_curve;
  evo_engine::SingleDistribution<float> needle_cross_section_width_max_m{0.0018f};
  evo_engine::SingleDistribution<float> needle_cross_section_thickness_max_m{0.0011f};
  evo_engine::PlottedDistribution<float> needle_cross_section_width_profile;
  evo_engine::PlottedDistribution<float> needle_cross_section_thickness_profile;
  evo_engine::PlottedDistribution<float> needle_cross_section_temporal_maturity_curve;

  // ===== 10. Initiation capacity (order attenuation, intra-year, inter-year, bud storage) =====
  // Order attenuation (linear reduction per branch order).
  float needle_order_length_attenuation = 0.12f;
  float needle_order_radius_attenuation = 0.10f;
  float needle_order_min_length_scale = 0.55f;
  float needle_order_min_radius_scale = 0.60f;
  // Intra-year initiation-time capacity mapping.
  float needle_intra_year_base_ratio = 0.35f;
  float needle_intra_year_sigmoid_steepness = 8.0f;
  float needle_intra_year_sigmoid_midpoint_fraction = 0.45f;
  float needle_intra_year_late_decay_start_fraction = 1.0f;
  float needle_intra_year_late_decay_end_scale = 1.0f;
  // Inter-year primary/fascicular transition (year1 vs year2+).
  int needle_fascicular_start_year = 1;
  float needle_year2plus_length_multiplier = 1.0f;
  float needle_year2plus_width_multiplier = 1.0f;
  float needle_year2plus_thickness_multiplier = 1.0f;
  float needle_lignification_factor_year1 = 0.75f;
  float needle_lignification_factor_year2plus = 1.15f;
  float needle_stomatal_strip_density_year1 = 0.35f;
  float needle_stomatal_strip_density_year2plus = 0.65f;
  float needle_basal_taper_ratio_year1 = 0.82f;
  float needle_basal_taper_ratio_year2plus = 0.92f;
  float needle_fascicle_sheath_budget_gdd = 420.0f;
  float needle_specularity_plasticity_year1 = 0.25f;
  float needle_specularity_plasticity_year2plus = 0.65f;
  // Bud-storage proxy controls.
  float needle_bud_storage_vigor_strength = 0.0f;
  float needle_bud_storage_completion_floor = 0.60f;

  // ===== 11. Needle curvature & waviness (near-inert defaults; activate via diameter > 0) =====
  evo_engine::SingleDistribution<float> needle_curvature_adaxial_bias{0.003f};
  evo_engine::SingleDistribution<float> needle_curvature_abaxial_bias{0.010f};
  evo_engine::SingleDistribution<float> needle_curvature_gradient_per_arclen{0.0015f};
  evo_engine::SingleDistribution<float> needle_diameter_for_curvature_m{0.001f};
  evo_engine::SingleDistribution<float> needle_sinusoidal_amplitude_deg{0.0f};
  evo_engine::SingleDistribution<float> needle_sinusoidal_frequency_cycles{0.0f};
  evo_engine::SingleDistribution<float> needle_sinusoidal_phase_randomness_deg{0.0f};

  // ===== 12. Needle mechanics (elastica; defaults inert) =====
  evo_engine::SingleDistribution<float> needle_young_modulus_baseline_Pa{0.0f};
  evo_engine::SingleDistribution<float> needle_lignification_maturation_years{0.0f};
  evo_engine::SingleDistribution<float> needle_density_kg_m3{0.0f};
  evo_engine::SingleDistribution<float> gravity_m_s2{0.0f};

  // ===== 13. Per-needle variability (CV-style multipliers within each fascicle) =====
  evo_engine::SingleDistribution<float> needle_per_needle_length_cv{0.0f};
  evo_engine::SingleDistribution<float> needle_per_needle_curvature_cv{0.0f};
  evo_engine::SingleDistribution<float> needle_per_needle_radius_cv{0.0f};
  evo_engine::SingleDistribution<float> needle_per_needle_modulus_cv{0.0f};
  evo_engine::SingleDistribution<float> needle_per_needle_density_cv{0.0f};
  evo_engine::SingleDistribution<float> needle_per_needle_wave_amplitude_cv{0.0f};
  evo_engine::SingleDistribution<float> needle_per_needle_wave_frequency_cv{0.0f};
  evo_engine::SingleDistribution<float> needle_per_needle_wave_phase_cv{0.0f};

  // ===== 14. Visualization (color tints, axial age gradient) =====
  // Read by the geometry/material build path (Shaded and ByType color
  // modes) and by the needle young->old color blend. Plain runtime values
  // (not distributions); serialized to YAML and round-tripped through the
  // inspector.
  glm::vec4 main_stem_color_rgba{0.83f, 0.72f, 0.50f, 1.0f};       ///< Young main-stem bark tint.
  glm::vec4 main_stem_old_color_rgba{0.45f, 0.30f, 0.20f, 1.0f};   ///< Aged main-stem bark tint.
  glm::vec4 needle_color_rgba{0.18f, 0.42f, 0.20f, 1.0f};          ///< Young needle tint.
  glm::vec4 needle_old_color_rgba{0.45f, 0.32f, 0.15f, 1.0f};      ///< Aged/senescent needle tint.
  float needle_axial_age_span = 0.35f;     ///< [-1,1] Positive makes tip older than base.
  float needle_axial_age_exponent = 1.0f;  ///< >1 concentrates axial aging near tip/base.

  // ===== 15. [deprecated] No runtime consumer =====
  // Kept declared per workspace policy ("never remove unused code unless
  // explicitly told to — comment with [deprecated]").
  /// [deprecated] Pine-side dynamic tropism array. Sampled into
  /// `SampledPineParams::tropisms` but unused by the pine growth path. The
  /// active stem tropism is the scalar `gravitropism_first_order` above.
  /// The Maize tassel side does consume an analogous vector, so the type
  /// stays available; only the Scots pine wiring is dormant.
  std::vector<TropismEntry> tropisms;

  /// Sample all distributions deterministically; lock RNG draw order.
  SampledPineParams Sample(std::mt19937& rng) const;

  [[nodiscard]] evo_engine::Entity Instantiate() const;

  // ===== IAsset =====
  bool OnInspect(const std::shared_ptr<evo_engine::EditorLayer>& editor_layer) override;
  [[nodiscard]] bool SupportsDefaultsOverwrite() const override {
    return true;
  }
  [[nodiscard]] std::filesystem::path ResolveWritableDefaultsPath() const override;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;

  // ===== ILSystemExplorableDescriptor =====
  void RegisterExplorableAxes(ParamSpaceExplorer& explorer) override;
  uint64_t ExplorableSchemaFingerprint() const override {
    // Bump when the explorable axis schema changes shape (added/removed
    // axes). The dynamic tropism count is folded in to keep the existing
    // contract that adding tropism entries also invalidates cached layouts.
    // Constant 0x4E45454458534537 spells "NEEDXSE7".
    // change.
    return 0x4E45454458534537ull ^ static_cast<uint64_t>(tropisms.size());
  }

  // ===== Editor preferences (serialized) =====
  bool live_preview = false;
  float live_preview_rate_hz = 12.0f;
  bool live_preview_representative_only = true;
  bool live_preview_cap_target_gdd = true;
  float live_preview_max_gdd = 6000.0f;
  int live_preview_max_growth_steps = 64;
  int grid_rows = 5;
  int grid_cols = 5;
  float grid_spacing = 3.0f;
  float triangle_side_length = 3.0f;

  ParamSpaceExplorer explorer_;

 private:
  // Live-preview scheduling state (not serialized).
  bool live_preview_dirty_ = false;
  double live_preview_last_apply_seconds_ = -1.0;
  bool live_preview_was_dragging_ = false;
  bool live_preview_needs_full_apply_ = false;

  uint32_t live_preview_request_count_ = 0;
  uint32_t live_preview_apply_count_ = 0;
  uint32_t live_preview_coalesced_count_ = 0;
  double live_preview_last_apply_ms_ = 0.0;
  double live_preview_total_apply_ms_ = 0.0;
};

}  // namespace l_system_plugin
