#pragma once

#include <IAsset.hpp>
#include <Plot2D.hpp>
#include <cstdint>
#include <glm/vec4.hpp>
#include <random>
#include <string>
#include <vector>
#include "ILSystemExplorableDescriptor.hpp"
#include "LSystemRuleHelpers.hpp"
#include "ParamSpaceExplorer.hpp"

namespace l_system_package {

/// 1 simulated year ~= 1500 GDD (base 5C, mid-latitude). Used as the unit
/// conversion between physiological-time fields (GDD on disk) and the
/// year-based clock used in production rules and growth state.
constexpr float kPineGddPerYear = 1500.0f;

/**
 * @brief Per-emission sampling distributions copied from the descriptor.
 *
 * Carried inside `SampledPineParams` so that production rules can resample
 * each parameter at every consumption site (per phytomer, per whorl bud,
 * per fascicle sheath) using a per-node RNG, instead of being locked to the
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
  evo_engine::SingleDistribution<float> annual_whorl_probability;
  evo_engine::SingleDistribution<float> whorl_position_norm;
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
  evo_engine::SingleDistribution<float> fascicle_sheath_length_m;
  evo_engine::SingleDistribution<float> fascicle_sheath_width_m;
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
 * growth produces internode-only phytomers (no needle fascicle); subsequent
 * phytomers each carry one fascicle sheath whose child needles are counted by
 * `needle_count_per_cluster`.
 */
struct SampledPineParams {
  // -- Phytomer scheduling --
  int max_branching_order = 1;                 ///< 0 = leader only, 1 = primary laterals, ...
  float plastochron_years = 130.0f / kPineGddPerYear;  ///< Physiological time between consecutive phytomer events.
  int max_phytomers_per_seasonal_growth = 14;           ///< Phytomers emitted per active season before apex pauses.

  // -- Whorl architecture --
  float annual_whorl_probability = 0.22f;
  float whorl_position_norm = 0.92f;
  int branches_per_whorl = 2;
  float whorl_dormancy_years = 1.0f;  ///< Overwintering = 1 year for Pinus sylvestris.
  float branch_insertion_angle_deg = 60.0f;
  float branch_roll_phyllotaxis_deg = 137.5f;  ///< Golden angle (also drives needle phyllotaxis).

  // -- Phytomer dimensions (SI metres) --
  float internode_length_m = 0.0020f;            ///< Length of one phytomer's internode.
  float leader_internode_thickness_m = 0.0015f;  ///< Main stem thickness (diameter), metres.
  float lateral_length_ratio = 0.55f;            ///< lateral length = leader * ratio^order.
  float lateral_thickness_ratio = 0.5f;

  // -- Needles --
  float bare_zone_fraction =
      0.12f;  ///< Temporal fraction at the start of each year that emits internode-only phytomers [0, 0.95).
  int needle_count_per_cluster = 2;  ///< Pinus sylvestris needles per fascicle sheath.
  int needle_segment_count = 11;     ///< Logical longitudinal segments per needle; renderer pads hidden controls.
  float fascicle_sheath_length_m = 0.006f;
  float fascicle_sheath_width_m = 0.0012f;
  float needle_length_m = 0.104f;
  int needle_lifespan_years = 4;
  float needle_browning_years = 0.8f;                ///< Years from senescence onset to abscission.
  float needle_flush_delay_years = 0.08f;            ///< Delay from phytomer emergence to needle flush start.
  float internode_maturation_years = 0.053f;         ///< Time from emergence to mature internode length.
  float needle_maturation_years = 0.093f;            ///< Time from flush to mature needle length.
  float needle_cross_section_width_max_m = 0.0018f;  ///< Flat adaxial full width at the broadest position.
  float needle_cross_section_thickness_max_m =
      0.0011f;                                    ///< Convex abaxial radius/depth at the broadest position.
  float needle_order_length_attenuation = 0.12f;  ///< Linear reduction per branch order.
  float needle_order_radius_attenuation = 0.10f;  ///< Linear thickness reduction per branch order.
  float needle_order_min_length_scale = 0.55f;    ///< Floor after order attenuation.
  float needle_order_min_radius_scale = 0.60f;    ///< Thickness floor after order attenuation.
  // Intra-year initiation-time capacity mapping (per phytomer index in season).
  evo_engine::Plot2D<float> needle_intra_year_capacity_curve;  ///< x=normalized seasonal phytomer index, y=capacity.
  // Year-0-only capacity mapping. Later cohorts use neutral values.
  float needle_year0_length_multiplier = 0.65f;      ///< Length multiplier for initiation year 0.
  float needle_lignification_factor_year0 = 0.75f;   ///< Year-0 lignification response multiplier.
  float needle_stomatal_strip_density_year0 = 0.35f; ///< Procedural strip density proxy for year-0 cohorts [0,1].
  float needle_basal_taper_ratio_year0 = 0.82f;      ///< Radius multiplier at needle base for year-0 cohorts.
  float needle_fascicle_sheath_budget_years = 0.28f;  ///< Characteristic years for sheath visual maturation near base.
  float needle_specularity_plasticity_year0 =
      0.25f;  ///< How much micro-variation tracks maturity in year-0 cohorts [0,1].
  // Bud-storage proxy coupling from previous-year completion to current-year potential.
  float needle_bud_storage_vigor_strength = 0.35f;    ///< 0 = disabled, 1 = fully applied.
  float needle_bud_storage_completion_floor = 0.60f;  ///< Lower clamp for completion-derived vigor.

  // -- Needle bilateral curvature (default near-inert; diameter > 0 activates) --
  float needle_curvature_adaxial_bias = 0.003f;
  float needle_curvature_abaxial_bias = 0.010f;
  float needle_curvature_gradient_per_arclen = 0.0015f;
  float needle_diameter_for_curvature_m = 0.0012f;
  float needle_sinusoidal_amplitude_deg = 4.0f;
  float needle_sinusoidal_frequency_cycles = 1.5f;
  float needle_sinusoidal_phase_randomness_deg = 30.0f;

  // -- Needle mechanics (defaults inert -> elastica skipped) --
  float needle_young_modulus_baseline_Pa = 0.0f;
  float needle_lignification_maturation_years = 0.0f;
  float needle_density_kg_m3 = 800.0f;
  float gravity_m_s2 = 9.81f;

  // -- Per-needle variability (CV-style multipliers within each fascicle) --
  float needle_per_needle_length_cv = 0.08f;
  float needle_per_needle_curvature_cv = 0.15f;
  float needle_per_needle_radius_cv = 0.06f;
  float needle_per_needle_modulus_cv = 0.0f;
  float needle_per_needle_density_cv = 0.0f;
  float needle_per_needle_wave_amplitude_cv = 0.20f;
  float needle_per_needle_wave_frequency_cv = 0.10f;
  float needle_per_needle_wave_phase_cv = 0.20f;

  // -- Tropism --
  float gravitropism_first_order = 0.0001f;  ///< deg/GDD applied to leader internodes (order 0) only.

  // -- Per-shoot stochastic noise (CV / sigma; 0 = deterministic) --
  float internode_length_per_node_cv = 0.08f;
  float internode_thickness_per_node_cv = 0.05f;
  float branch_angle_per_node_sigma_deg = 6.0f;
  float roll_phyllotaxis_per_node_sigma_deg = 8.0f;

  // -- Plant-wide --
  float plant_random = 0.5f;
  float initial_orientation_yaw_deg = 0.0f;  ///< Sampled once per plant; applied as root yaw around +Y.

  /// Copies of the descriptor's distributions, used by production rules to
  /// resample each parameter at every consumption site (per phytomer, per
  /// whorl bud, per fascicle sheath). The scalar fields above remain the
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
  // Field declarations are organized into 14 functional groups below.
  // Time-domain fields stay in GDD on disk and are converted to years inside
  // Sample(). YAML key strings, RNG draw order in Sample(), and the explorer
  // axis count intentionally track the current schema only.
  // ============================================================================

  // ===== 1. Plant identity & global time =====
  evo_engine::SingleDistribution<float> initial_orientation_yaw_deg{
      0.0f};                                                  ///< Sampled once per plant; rotation around +Y.
  evo_engine::SingleDistribution<float> target_gdd{3000.0f, 500.0f};  ///< Per-instance target growth GDD.
  // GDD/day couples directly with LSystemLayer chronological day speed:
  //   delta_gdd = sampled_gdd_per_day * delta_days
  evo_engine::SingleDistribution<float> gdd_per_day{10.0f, 2.0f};  ///< Per-pine thermal rate (GDD/day).

  // ===== 2. Phytomer scheduling =====
  evo_engine::SingleDistribution<float> max_branching_order{1.0f, 0.0f};
  evo_engine::SingleDistribution<float> plastochron_gdd{130.0f, 25.0f};
  evo_engine::SingleDistribution<float> max_phytomers_per_seasonal_growth{14.0f, 3.0f};

  // ===== 3. Whorl architecture =====
  evo_engine::SingleDistribution<float> annual_whorl_probability{0.22f, 0.08f};
  evo_engine::SingleDistribution<float> whorl_position_norm{0.92f, 0.04f};
  evo_engine::SingleDistribution<float> branches_per_whorl{2.0f, 0.75f};
  /// Overwintering dormancy for whorl buds. Chronological, NOT GDD: bud
  /// release is driven by chilling + photoperiod, not heat sums. Default
  /// 1 yr = annual Scots pine whorl cycle.
  evo_engine::SingleDistribution<float> whorl_dormancy_years{1.0f, 0.0f};
  evo_engine::SingleDistribution<float> branch_insertion_angle_deg{60.0f, 10.0f};
  evo_engine::SingleDistribution<float> branch_roll_phyllotaxis_deg{137.5f, 1.0f};

  // ===== 4. Stem geometry (SI metres) =====
  // Internode size, lateral attenuation, stem maturation, and the maturity
  // curves that drive stem length/width over age. (Previously scattered: the
  // `internode_length_maturity_curve` and `internode_width_maturity_curve`
  // fields were declared inside the Needles block, which was misleading.)
  evo_engine::SingleDistribution<float> internode_length_m{0.0020f, 0.00045f};
  evo_engine::SingleDistribution<float> leader_internode_thickness_m{
      0.0015f, 0.00035f};  ///< Main stem width (diameter), metres.
  evo_engine::SingleDistribution<float> lateral_length_ratio{0.55f, 0.1f};
  evo_engine::SingleDistribution<float> lateral_thickness_ratio{0.5f, 0.08f};
  evo_engine::SingleDistribution<float> internode_maturation_gdd{250.0f, 25.0f};
  evo_engine::PlottedDistribution<float> internode_length_maturity_curve;  ///< Stem-axis curve (was in Needles).
  evo_engine::PlottedDistribution<float> internode_width_maturity_curve;   ///< Stem-axis curve (was in Needles).
  float internode_age_exponent = 4.0f;  ///< >1 delays visible stem aging toward max age.

  // ===== 5. Stem stochastic noise (per-shoot CV / sigma; 0 = deterministic) =====
  evo_engine::SingleDistribution<float> internode_length_per_node_cv{0.08f, 0.02f};
  evo_engine::SingleDistribution<float> internode_thickness_per_node_cv{0.05f, 0.015f};
  evo_engine::SingleDistribution<float> branch_angle_per_node_sigma_deg{6.0f, 2.0f};
  evo_engine::SingleDistribution<float> roll_phyllotaxis_per_node_sigma_deg{8.0f, 3.0f};

  // ===== 6. Stem tropism =====
  evo_engine::SingleDistribution<float> gravitropism_first_order{0.0001f};  ///< Main-stem-only curvature (deg/GDD).

  // ===== 7. Needle layout =====
  evo_engine::SingleDistribution<float> bare_zone_fraction{0.12f, 0.04f};
  evo_engine::SingleDistribution<float> needle_count_per_cluster{2.0f, 0.0f};
  int needle_segment_count = 20;  ///< Logical longitudinal segments per needle; renderer pads hidden controls.
  evo_engine::SingleDistribution<float> fascicle_sheath_length_m{0.006f, 0.0015f};
  evo_engine::SingleDistribution<float> fascicle_sheath_width_m{0.0012f, 0.00023f};
  evo_engine::SingleDistribution<float> needle_branching_angle_deg{72.0f, 14.0f};
  /// GDD-equivalent duration converted to years at node creation.
  /// Runtime applies this against chronological age so relaxation continues
  /// even when thermal accumulation is paused outside the active season.
  evo_engine::SingleDistribution<float> needle_branching_relax_gdd{450.0f, 100.0f};

  // ===== 8. Needle lifecycle =====
  // Lifespan and browning are post-maturity chronological aging (FSPM Rule
  // of Ontogeny: once the organ has reached its final size, time -- not
  // heat -- governs its remaining biology). Flush delay and maturation are
  // active-expansion processes, so those stay in GDD.
  evo_engine::SingleDistribution<float> needle_lifespan_years{3.5f, 0.6f};
  evo_engine::SingleDistribution<float> needle_browning_years{0.8f,
                                                              0.2f};  ///< Years from senescence onset to abscission.
  evo_engine::SingleDistribution<float> needle_flush_delay_gdd{120.0f, 35.0f};
  evo_engine::SingleDistribution<float> needle_maturation_gdd{650.0f, 150.0f};

  // ===== 9. Needle dimensions & cross-section =====
  evo_engine::SingleDistribution<float> needle_length_m{0.104f, 0.018f};
  evo_engine::PlottedDistribution<float> needle_length_maturity_curve;
  evo_engine::SingleDistribution<float> needle_cross_section_width_max_m{0.0018f, 0.0003f};
  evo_engine::SingleDistribution<float> needle_cross_section_thickness_max_m{0.0009f, 0.00015f};
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
  evo_engine::Plot2D<float> needle_intra_year_capacity_curve;  ///< x=normalized seasonal phytomer index, y=capacity.
  // Year-0-only controls. Later cohorts use neutral values.
  float needle_year0_length_multiplier = 0.65f;
  float needle_lignification_factor_year0 = 0.75f;
  float needle_stomatal_strip_density_year0 = 0.35f;
  float needle_basal_taper_ratio_year0 = 0.82f;
  float needle_fascicle_sheath_budget_gdd = 420.0f;
  float needle_specularity_plasticity_year0 = 0.25f;
  // Bud-storage proxy controls.
  float needle_bud_storage_vigor_strength = 0.35f;
  float needle_bud_storage_completion_floor = 0.60f;

  // ===== 11. Needle curvature & waviness (near-inert defaults; activate via diameter > 0) =====
  evo_engine::SingleDistribution<float> needle_curvature_adaxial_bias{0.003f, 0.0015f};
  evo_engine::SingleDistribution<float> needle_curvature_abaxial_bias{0.010f, 0.0025f};
  evo_engine::SingleDistribution<float> needle_curvature_gradient_per_arclen{0.0015f, 0.0005f};
  evo_engine::SingleDistribution<float> needle_diameter_for_curvature_m{0.0012f, 0.0002f};
  evo_engine::SingleDistribution<float> needle_sinusoidal_amplitude_deg{4.0f, 2.0f};
  evo_engine::SingleDistribution<float> needle_sinusoidal_frequency_cycles{1.5f, 0.4f};
  evo_engine::SingleDistribution<float> needle_sinusoidal_phase_randomness_deg{30.0f, 10.0f};

  // ===== 12. Needle mechanics (elastica; defaults inert) =====
  evo_engine::SingleDistribution<float> needle_young_modulus_baseline_Pa{0.0f};
  evo_engine::SingleDistribution<float> needle_lignification_maturation_years{0.0f};
  evo_engine::SingleDistribution<float> needle_density_kg_m3{800.0f, 0.0f};
  evo_engine::SingleDistribution<float> gravity_m_s2{9.81f, 0.0f};

  // ===== 13. Per-needle variability (CV-style multipliers within each fascicle) =====
  evo_engine::SingleDistribution<float> needle_per_needle_length_cv{0.08f, 0.02f};
  evo_engine::SingleDistribution<float> needle_per_needle_curvature_cv{0.15f, 0.03f};
  evo_engine::SingleDistribution<float> needle_per_needle_radius_cv{0.06f, 0.02f};
  evo_engine::SingleDistribution<float> needle_per_needle_modulus_cv{0.0f};
  evo_engine::SingleDistribution<float> needle_per_needle_density_cv{0.0f};
  evo_engine::SingleDistribution<float> needle_per_needle_wave_amplitude_cv{0.20f, 0.05f};
  evo_engine::SingleDistribution<float> needle_per_needle_wave_frequency_cv{0.10f, 0.03f};
  evo_engine::SingleDistribution<float> needle_per_needle_wave_phase_cv{0.20f, 0.05f};

  // ===== 14. Descriptor-native material model =====
  // Compact editable controls derived from measured RGB/HSV/Lab histograms.
  // Full histogram provenance remains outside the runtime descriptor.
  int material_profile_version = 1;
  std::string material_profile_source = "derived_from_sam3_20pot";
  std::string material_profile_hash;
  float biological_material_model_strength = 0.0f;  ///< 0 = direct palettes, 1 = biological pigment overlay.
  float biological_chlorophyll_scale = 1.0f;        ///< Live green pigment strength for needles.
  float biological_carotenoid_gold_scale = 1.0f;    ///< Golden/yellow contribution in senescent needles.
  float biological_lignin_bark_scale = 1.0f;        ///< Bark/lignin browning in older stems.
  float biological_senescence_bias = 0.0f;          ///< Signed global age/dryness bias for needles.
  float biological_cuticle_wax = 0.0f;              ///< Waxy pale/desaturated lift on live needles.
  float biological_individual_variation = 0.0f;     ///< Extra deterministic per-organ color variation.
  float biological_facet_contrast = 0.0f;           ///< Faceted needle edge/specular contrast.
  float biological_tip_darkening_strength = 0.0f;   ///< Extra live-needle darkening toward the tip.
  float biological_stem_age_browning_scale = 1.0f;  ///< Multiplier for age-driven stem browning.
  glm::vec4 young_needle_palette_rgba{0.0f, 0.7455683f, 0.051418442f, 1.0f};
  glm::vec4 older_needle_palette_rgba{0.21606492f, 0.28904234f, 0.20766766f, 1.0f};
  glm::vec4 dry_brown_needle_palette_rgba{0.77059436f, 0.5399785f, 0.0f, 1.0f};
  glm::vec4 main_stem_palette_rgba{0.7861458f, 0.9165798f, 0.47310424f, 1.0f};
  glm::vec4 mature_bark_stem_palette_rgba{0.7009804f, 0.45447198f, 0.18555366f, 1.0f};
  glm::vec4 node_sheath_brown_palette_rgba{0.49f, 0.31f, 0.13f, 1.0f};
  glm::vec4 fascicle_sheath_palette_rgba{0.42f, 0.34f, 0.24f, 1.0f};
  evo_engine::Plot2D<float> needle_axial_color_curve;  ///< x=base-to-tip, y=older/darker color blend.
  evo_engine::Plot2D<float> needle_y_age_color_curve;  ///< x=cohort age/height proxy, y=older/dry palette blend.
  evo_engine::Plot2D<float> stem_age_gradient_curve;   ///< x=internode age, y=mature/bark blend.
  float needle_micro_variation = 0.012f;
  float stem_micro_variation = 0.012f;
  float young_needle_roughness = 0.92f;
  float old_needle_roughness = 0.97f;
  float young_needle_specular = 0.09f;
  float old_needle_specular = 0.04f;
  float stem_roughness = 0.86f;
  float stem_specular = 0.12f;
  float node_browning_strength = 0.28f;
  float sheath_browning_strength = 0.40f;
  float node_browning_radius_norm = 0.18f;
  float needle_twist_turns = 0.25f;
  float needle_edge_darkening = 0.08f;
  float needle_tip_color_mix_start = 0.35f;  ///< Base-to-tip position where later-cohort tip darkening starts.
  float needle_tip_color_exponent = 1.35f;   ///< >1 delays distal darkening until closer to the needle tip.
  float needle_old_thinning_fraction = 0.33f;  ///< Fractional strand-radius loss as needle segments reach old color.
  float needle_min_strand_thickness_m = 0.00002f;  ///< Minimum rendered strand radius for needle visibility.
  float needle_axial_age_span = 0.34f;     ///< [-1,1] Positive makes tip older than base.
  float needle_axial_age_exponent = 1.0f;  ///< >1 concentrates axial aging near tip/base.

  /// Sample all distributions deterministically; lock RNG draw order.
  SampledPineParams Sample(std::mt19937& rng) const;

  [[nodiscard]] evo_engine::Entity Instantiate() const;

  // ===== IAsset =====
  bool DrawEditorControls(const std::shared_ptr<evo_engine::EditorLayer>& editor_layer);
  [[nodiscard]] bool SupportsDefaultsOverwrite() const {
    return true;
  }
  [[nodiscard]] std::filesystem::path ResolveWritableDefaultsPath() const;
  bool SaveCurrentAsDefaultSnapshot(std::string* error_message = nullptr) const;

  // ===== ILSystemExplorableDescriptor =====
  void RegisterExplorableAxes(ParamSpaceExplorer& explorer) override;
  uint64_t ExplorableSchemaFingerprint() const override {
    return 0x53434F545350494Eull;  // "SCOTSPIN"
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

}  // namespace l_system_package
