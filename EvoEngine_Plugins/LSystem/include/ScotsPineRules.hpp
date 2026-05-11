#pragma once

#include "DerivationEngine.hpp"
#include "LSystemRuleHelpers.hpp"
#include "ProductionRule.hpp"
#include "ScotsPineDescriptor.hpp"
#include "ScotsPineModules.hpp"
#include <algorithm>
#include <cstdint>
#include <cmath>
#include <limits>
#include <random>

namespace l_system_plugin {

using PineRule = ProductionRule<PineGraph, PineModuleData>;
using PineEngine = DerivationEngine<PineGraphData, PineFlowData, PineModuleData>;

namespace pine_detail {

inline float NormalizeDegrees(float degrees) {
  float w = std::fmod(degrees, 360.0f);
  if (w < 0.0f) w += 360.0f;
  return w;
}

inline float NormalizeRadians(float radians) {
  constexpr float kTwoPi = 6.28318530717958647692f;
  float w = std::fmod(radians, kTwoPi);
  if (w < 0.0f) w += kTwoPi;
  return w;
}

inline uint32_t MixBits(const uint32_t value) {
  uint32_t x = value;
  x ^= x >> 16;
  x *= 0x7feb352du;
  x ^= x >> 15;
  x *= 0x846ca68bu;
  x ^= x >> 16;
  return x;
}

inline float HashToUnit01(const uint32_t value) {
  return static_cast<float>(value & 0x00ffffffu) / 16777215.0f;
}

inline float DeterministicSignedJitter(const float cluster_random,
                                       const int needle_index,
                                       const uint32_t channel_tag) {
  const float clamped = std::clamp(cluster_random, 0.0f, 1.0f);
  const uint32_t base = static_cast<uint32_t>(std::round(clamped * 16777215.0f));
  const uint32_t key = base ^ (static_cast<uint32_t>(needle_index + 1) * 0x9e3779b9u) ^ channel_tag;
  return HashToUnit01(MixBits(key)) * 2.0f - 1.0f;
}

inline float DeterministicDistributionSample(const evo_engine::SingleDistribution<float>& distribution,
                                             const float cluster_random,
                                             const uint32_t sample_index,
                                             const uint32_t channel_tag) {
  const float clamped = std::clamp(cluster_random, 0.0f, 1.0f);
  const uint32_t base = static_cast<uint32_t>(std::round(clamped * 16777215.0f));
  const uint32_t key = base ^ ((sample_index + 1u) * 0x9e3779b9u) ^ channel_tag;
  std::mt19937 local_rng(MixBits(key));
  return SampleDistribution(distribution, local_rng);
}

inline float ComputeOrderVigorScale(int order,
                                    float attenuation,
                                    float min_scale) {
  const int safe_order = std::max(0, order);
  const float safe_attn = std::clamp(attenuation, 0.0f, 1.0f);
  const float safe_min = std::clamp(min_scale, 0.10f, 1.0f);
  const float raw = 1.0f - safe_attn * static_cast<float>(safe_order);
  return std::clamp(raw, safe_min, 1.0f);
}

inline float ComputeNormalizedPhytomerProgress(int phytomer_index_this_year,
                                               int phytomers_per_season) {
  if (phytomers_per_season <= 1) {
    return 1.0f;
  }
  const float numerator = static_cast<float>(std::max(0, phytomer_index_this_year));
  const float denominator = static_cast<float>(std::max(1, phytomers_per_season - 1));
  return std::clamp(numerator / denominator, 0.0f, 1.0f);
}

inline float ComputeIntraYearCapacityWeight(float normalized_index,
                                            float base_ratio,
                                            float sigmoid_steepness,
                                            float sigmoid_midpoint,
                                            float late_decay_start,
                                            float late_decay_end_scale) {
  const float x = std::clamp(normalized_index, 0.0f, 1.0f);
  const float base = std::clamp(base_ratio, 0.0f, 1.0f);
  const float k = std::max(0.01f, sigmoid_steepness);
  const float m = std::clamp(sigmoid_midpoint, 0.0f, 1.0f);

  const float logistic = 1.0f / (1.0f + std::exp(-k * (x - m)));
  float weight = base + (1.0f - base) * logistic;

  const float decay_start = std::clamp(late_decay_start, 0.0f, 1.0f);
  if (decay_start < 1.0f && x > decay_start) {
    const float t = (x - decay_start) / std::max(1.0e-6f, 1.0f - decay_start);
    const float end_scale = std::clamp(late_decay_end_scale, 0.0f, 2.0f);
    weight *= glm::mix(1.0f, end_scale, std::clamp(t, 0.0f, 1.0f));
  }

  return std::max(0.0f, weight);
}

inline float ComputeBudStorageVigorWeight(float previous_season_vigor,
                                          float vigor_strength,
                                          float vigor_floor) {
  const float floor = std::clamp(vigor_floor, 0.0f, 1.0f);
  const float proxy = std::clamp(previous_season_vigor, floor, 1.0f);
  const float strength = std::clamp(vigor_strength, 0.0f, 1.0f);
  return glm::mix(1.0f, proxy, strength);
}

/// Build the single phytomer internode for `apex`.
/// `is_first_on_axis = true` carries the lateral insertion angle + initial
/// phyllotactic roll set when the bud spawned the apex; subsequent phytomers
/// on the same axis grow straight ahead (branch_angle = 0, no extra roll).
///
/// Per-emission sampling: every parameter that was previously `p.X` (a
/// plant-wide drawn scalar) is now drawn fresh from
/// `p.distributions.X` using the per-emission `node_rng`. Clamps mirror the
/// bounds previously enforced inside `ScotsPineDescriptor::Sample`.
inline PineInternode MakePhytomerInternode(const SampledPineParams& p,
                                           const PineApex& apex,
                                           bool is_first_on_axis,
                                           std::mt19937& node_rng,
                                           float t_init_years) {
  // ---- Per-emission draws (see SampledPineParams::distributions) ----------
  const float internode_length_m =
      std::clamp(SampleDistribution(p.distributions.internode_length_m, node_rng), 0.0001f, 0.50f);
  const float leader_thickness_raw =
      std::clamp(SampleDistribution(p.distributions.leader_internode_thickness_m, node_rng),
                 0.0002f, 0.05f);
  const float max_sane_thickness =
      std::max(0.0002f, 0.40f * internode_length_m * 8.0f);
  const float leader_internode_thickness_m =
      std::min(leader_thickness_raw, max_sane_thickness);
  const float lateral_length_ratio =
      std::clamp(SampleDistribution(p.distributions.lateral_length_ratio, node_rng), 0.05f, 1.0f);
  const float lateral_thickness_ratio =
      std::clamp(SampleDistribution(p.distributions.lateral_thickness_ratio, node_rng), 0.05f, 1.5f);
  const float internode_length_per_node_cv = std::max(
      0.0f, SampleDistribution(p.distributions.internode_length_per_node_cv, node_rng));
  const float internode_thickness_per_node_cv = std::max(
      0.0f, SampleDistribution(p.distributions.internode_thickness_per_node_cv, node_rng));
  const float branch_insertion_angle_deg = std::clamp(
      SampleDistribution(p.distributions.branch_insertion_angle_deg, node_rng), -85.0f, 85.0f);
  const float branch_angle_per_node_sigma_deg = std::max(
      0.0f, SampleDistribution(p.distributions.branch_angle_per_node_sigma_deg, node_rng));
  const float roll_phyllotaxis_per_node_sigma_deg = std::max(
      0.0f, SampleDistribution(p.distributions.roll_phyllotaxis_per_node_sigma_deg, node_rng));
  const float internode_maturation_years = std::clamp(
      SampleDistribution(p.distributions.internode_maturation_gdd, node_rng) / kPineGddPerYear,
      0.0f, 4.0f);
    const float main_stem_tropism_deg_per_gdd =
      SampleDistribution(p.distributions.gravitropism_first_order, node_rng);

  const int order = std::max(0, apex.order);
  const float length_scale = std::pow(std::max(0.05f, lateral_length_ratio),
                                      static_cast<float>(order));
  const float thickness_scale = std::pow(std::max(0.05f, lateral_thickness_ratio),
                                         static_cast<float>(order));
  float base_length = std::max(0.0001f, internode_length_m * length_scale);
  float base_thickness = std::max(0.00005f, leader_internode_thickness_m * thickness_scale);

  // Per-internode Gaussian noise on length and thickness (CV interpretation).
  if (internode_length_per_node_cv > 0.0f) {
    std::normal_distribution<float> n(0.0f, internode_length_per_node_cv);
    base_length = std::max(0.0001f, base_length * (1.0f + n(node_rng)));
  }
  if (internode_thickness_per_node_cv > 0.0f) {
    std::normal_distribution<float> n(0.0f, internode_thickness_per_node_cv);
    base_thickness = std::max(0.00005f, base_thickness * (1.0f + n(node_rng)));
  }

  PineInternode internode;
  internode.target_length = base_length;
  internode.target_thickness = base_thickness;
  // Continuous growth substrate: zero on creation, animated by UpdateNodeInfoImpl.
  const bool continuous = internode_maturation_years > 0.0f;
  internode.length = continuous ? 0.0f : base_length;
  internode.thickness = continuous ? 0.0f : base_thickness;

  // Insertion + roll. Only the first internode on a lateral carries the
  // bud's branch angle and phyllotactic phase; subsequent phytomers on the
  // same axis grow straight ahead.
  float branch_angle = 0.0f;
  float roll_angle = 0.0f;
  if (is_first_on_axis && order >= 1) {
    branch_angle = branch_insertion_angle_deg;
    roll_angle = apex.phyllotaxis_phase;
    if (branch_angle_per_node_sigma_deg > 0.0f) {
      std::normal_distribution<float> nb(0.0f, branch_angle_per_node_sigma_deg);
      branch_angle = std::clamp(branch_angle + nb(node_rng), -85.0f, 85.0f);
    }
    if (roll_phyllotaxis_per_node_sigma_deg > 0.0f) {
      std::normal_distribution<float> nr(0.0f, roll_phyllotaxis_per_node_sigma_deg);
      roll_angle = NormalizeDegrees(roll_angle + nr(node_rng));
    }
  }
  internode.branch_angle = branch_angle;
  internode.roll_angle = roll_angle;
  internode.bend_axis_local = glm::vec3(1.0f, 0.0f, 0.0f);
    // Main-stem tropism: per-GDD curvature is integrated in growth-rule space;
    // only the leader (order 0) receives this bending term.
    internode.curvature = (order == 0)
      ? -main_stem_tropism_deg_per_gdd * kPineGddPerYear
      : 0.0f;
  internode.growth_progress = continuous ? 0.0f : 1.0f;
  internode.year_produced = apex.year_index;
  internode.age_years = 0;
  internode.order = order;
  internode.node_random = SampleUnit01(node_rng);
  internode.continuous_growth.t_init_years = t_init_years;
  internode.continuous_growth.maturation_years = std::max(0.0f, internode_maturation_years);
  internode.continuous_growth.function.kind = GrowthFunctionKind::Sinusoidal;
  return internode;
}

/// Build a needle cluster anchored on the parent phytomer's internode
/// with azimuthal `roll_offset_deg`. Birth time is offset by the species
/// flush delay so needles emerge AFTER the internode has begun elongating.
///
/// Per-emission sampling: every parameter is drawn fresh per cluster from
/// `p.distributions.X` using `node_rng`. Plain (non-distribution) descriptor
/// scalars (`needle_order_*`, `needle_axial_*`, `internode_age_exponent`)
/// remain plant-wide and read from the SampledPineParams scalar fields.
inline PineNeedleCluster MakeNeedleCluster(const SampledPineParams& p,
                                           float s_along_norm,
                                           float roll_offset_deg,
                                           std::mt19937& node_rng,
                                           float internode_t_init_years,
                                           int parent_axis_order,
                                           int phytomer_index_this_year,
                                           int phytomers_per_season,
                                           int initiation_year_index,
                                           float previous_season_vigor) {
  // ---- Per-cluster draws --------------------------------------------------
  const int needle_count_per_cluster = std::clamp(
      static_cast<int>(std::round(SampleDistribution(p.distributions.needle_count_per_cluster, node_rng))),
      1, 6);
  const float needle_length_m =
      std::clamp(SampleDistribution(p.distributions.needle_length_m, node_rng), 0.005f, 0.30f);
  // Chronological lifespan and browning (years on disk per FSPM Rule of
  // Ontogeny). No GDD->years conversion needed.
  const int needle_lifespan_years = std::max(
      1, static_cast<int>(std::round(
             SampleDistribution(p.distributions.needle_lifespan_years, node_rng))));
    const float needle_browning_years = std::max(
      0.0f, SampleDistribution(p.distributions.needle_browning_years, node_rng));
  const float needle_flush_delay_years = std::max(
      0.0f, SampleDistribution(p.distributions.needle_flush_delay_gdd, node_rng) / kPineGddPerYear);
  const float needle_maturation_years = std::clamp(
      SampleDistribution(p.distributions.needle_maturation_gdd, node_rng) / kPineGddPerYear,
      0.0f, 1.0f);
  const float needle_curvature_adaxial_bias =
      SampleDistribution(p.distributions.needle_curvature_adaxial_bias, node_rng);
  const float needle_curvature_abaxial_bias =
      SampleDistribution(p.distributions.needle_curvature_abaxial_bias, node_rng);
  const float needle_curvature_gradient_per_arclen =
      SampleDistribution(p.distributions.needle_curvature_gradient_per_arclen, node_rng);
  const float needle_diameter_for_curvature_m = std::clamp(
      SampleDistribution(p.distributions.needle_diameter_for_curvature_m, node_rng),
      0.0f, 0.005f);
  const float needle_sinusoidal_amplitude_deg = std::max(
      0.0f, SampleDistribution(p.distributions.needle_sinusoidal_amplitude_deg, node_rng));
  const float needle_sinusoidal_frequency_cycles = std::max(
      0.0f, SampleDistribution(p.distributions.needle_sinusoidal_frequency_cycles, node_rng));
  const float needle_sinusoidal_phase_randomness_deg = std::max(
      0.0f, SampleDistribution(p.distributions.needle_sinusoidal_phase_randomness_deg, node_rng));
  const float needle_young_modulus_baseline_Pa = std::max(
      0.0f, SampleDistribution(p.distributions.needle_young_modulus_baseline_Pa, node_rng));
  const float needle_lignification_maturation_years = std::max(
      0.0f, SampleDistribution(p.distributions.needle_lignification_maturation_years, node_rng));
  // Draw maximum ellipsoid cross-section axes (full diameters in metres)
  // and convert to semi-axis radii for rendering/mechanics. Keep width and
  // thickness uncapped and independent.
  const float sampled_needle_cross_section_width_max_m = std::max(
      0.0f, SampleDistribution(p.distributions.needle_cross_section_width_max_m, node_rng));
  const float sampled_needle_cross_section_thickness_max_m = std::max(
      0.0f, SampleDistribution(p.distributions.needle_cross_section_thickness_max_m, node_rng));
    const float sampled_needle_cross_section_width_radius_m =
      sampled_needle_cross_section_width_max_m * 0.5f;
    const float sampled_needle_cross_section_thickness_radius_m =
      sampled_needle_cross_section_thickness_max_m * 0.5f;

    const float normalized_phytomer_progress = ComputeNormalizedPhytomerProgress(
      phytomer_index_this_year, phytomers_per_season);
    const float intra_year_capacity_weight = ComputeIntraYearCapacityWeight(
      normalized_phytomer_progress,
      p.needle_intra_year_base_ratio,
      p.needle_intra_year_sigmoid_steepness,
      p.needle_intra_year_sigmoid_midpoint_fraction,
      p.needle_intra_year_late_decay_start_fraction,
      p.needle_intra_year_late_decay_end_scale);

    const bool is_fascicular_year =
      initiation_year_index >= std::max(0, p.needle_fascicular_start_year);
    const float year_length_multiplier = is_fascicular_year
      ? std::max(0.0f, p.needle_year2plus_length_multiplier)
      : 1.0f;
    const float year_width_multiplier = is_fascicular_year
      ? std::max(0.0f, p.needle_year2plus_width_multiplier)
      : 1.0f;
    const float year_thickness_multiplier = is_fascicular_year
      ? std::max(0.0f, p.needle_year2plus_thickness_multiplier)
      : 1.0f;
    const float bud_storage_vigor_weight = ComputeBudStorageVigorWeight(
      previous_season_vigor,
      p.needle_bud_storage_vigor_strength,
      p.needle_bud_storage_completion_floor);

    const float inter_year_length_capacity_weight =
      year_length_multiplier * bud_storage_vigor_weight;
    const float inter_year_width_capacity_weight =
      year_width_multiplier * bud_storage_vigor_weight;
    const float inter_year_thickness_capacity_weight =
      year_thickness_multiplier * bud_storage_vigor_weight;

    const float adjusted_needle_length_m = std::max(
      0.0f,
      needle_length_m * intra_year_capacity_weight * inter_year_length_capacity_weight);
    const float adjusted_width_radius_m = std::max(
      0.0f,
      sampled_needle_cross_section_width_radius_m *
          intra_year_capacity_weight * inter_year_width_capacity_weight);
    const float adjusted_thickness_radius_m = std::max(
      0.0f,
      sampled_needle_cross_section_thickness_radius_m *
          intra_year_capacity_weight * inter_year_thickness_capacity_weight);

  const float needle_cross_section_width_radius_m = adjusted_width_radius_m;
  const float needle_cross_section_thickness_radius_m = adjusted_thickness_radius_m;
  const float needle_density_kg_m3 = std::max(
      0.0f, SampleDistribution(p.distributions.needle_density_kg_m3, node_rng));
  const float needle_per_needle_length_cv = std::max(
      0.0f, SampleDistribution(p.distributions.needle_per_needle_length_cv, node_rng));
  const float needle_per_needle_curvature_cv = std::max(
      0.0f, SampleDistribution(p.distributions.needle_per_needle_curvature_cv, node_rng));
  const float needle_per_needle_radius_cv = std::max(
      0.0f, SampleDistribution(p.distributions.needle_per_needle_radius_cv, node_rng));
  const float needle_per_needle_modulus_cv = std::max(
      0.0f, SampleDistribution(p.distributions.needle_per_needle_modulus_cv, node_rng));
  const float needle_per_needle_density_cv = std::max(
      0.0f, SampleDistribution(p.distributions.needle_per_needle_density_cv, node_rng));
  const float needle_per_needle_wave_amplitude_cv = std::max(
      0.0f, SampleDistribution(p.distributions.needle_per_needle_wave_amplitude_cv, node_rng));
  const float needle_per_needle_wave_frequency_cv = std::max(
      0.0f, SampleDistribution(p.distributions.needle_per_needle_wave_frequency_cv, node_rng));
  const float needle_per_needle_wave_phase_cv = std::max(
      0.0f, SampleDistribution(p.distributions.needle_per_needle_wave_phase_cv, node_rng));

  PineNeedleCluster c;
  c.count = std::max(1, needle_count_per_cluster);

  const float order_length_scale = ComputeOrderVigorScale(
      parent_axis_order,
      p.needle_order_length_attenuation,
      p.needle_order_min_length_scale);
  const float order_radius_scale = ComputeOrderVigorScale(
      parent_axis_order,
      p.needle_order_radius_attenuation,
      p.needle_order_min_radius_scale);

  c.target_length = std::max(0.001f, adjusted_needle_length_m * order_length_scale);
  c.render_radius_scale = std::clamp(order_radius_scale, 0.10f, 2.0f);
  c.cross_section_width_radius_m = needle_cross_section_width_radius_m;
  c.cross_section_thickness_radius_m = needle_cross_section_thickness_radius_m;
  c.initiation_year_index = std::max(0, initiation_year_index);
  c.initiation_phytomer_index = std::max(0, phytomer_index_this_year);
  c.intra_year_capacity_weight = intra_year_capacity_weight;
  c.inter_year_capacity_weight = inter_year_length_capacity_weight;
  c.bud_storage_vigor_weight = bud_storage_vigor_weight;
  c.age_years = 0;
  c.lifespan_years = std::max(1, needle_lifespan_years);
  c.browning_years = needle_browning_years;
  c.alive = true;
  c.s_along_parent_norm = std::clamp(s_along_norm, 0.0f, 1.0f);
  c.roll_offset_deg = NormalizeDegrees(roll_offset_deg);
  c.senescence_phase = 0.0f;
  c.node_random = SampleUnit01(node_rng);

    // Keep this deterministic and isolated from node_rng so adding angle
    // relaxation does not perturb topology stochasticity in downstream rules.
    c.branching_angle_deg = std::clamp(
      DeterministicDistributionSample(
        p.distributions.needle_branching_angle_deg,
        c.node_random,
        0u,
        0x2d3e4f50u),
      0.0f,
      89.5f);
    const float sampled_branching_relax_gdd = std::max(
      0.0f,
      DeterministicDistributionSample(
        p.distributions.needle_branching_relax_gdd,
        c.node_random,
        0u,
        0x17a3c8d2u));
    c.branching_relax_years = sampled_branching_relax_gdd / kPineGddPerYear;

  // Continuous-growth: needle flush is delayed relative to shoot emergence.
  const float t_flush = internode_t_init_years + std::max(0.0f, needle_flush_delay_years);
  c.continuous_growth.t_init_years = t_flush;
  c.continuous_growth.maturation_years = std::max(0.0f, needle_maturation_years);
  c.continuous_growth.function.kind = GrowthFunctionKind::Sinusoidal;
  c.length = (c.continuous_growth.maturation_years > 0.0f) ? 0.0f : c.target_length;

  // Bilateral differential growth field (defaults are inert -> straight needle).
  // Use node_random to slightly modulate magnitude so siblings differ.
  const float curvature_scale = 0.80f + 0.40f * c.node_random;
  c.growth_field.adaxial_bias = needle_curvature_adaxial_bias * curvature_scale;
  c.growth_field.abaxial_bias = needle_curvature_abaxial_bias * curvature_scale;
  c.growth_field.gradient_per_arclen = needle_curvature_gradient_per_arclen * curvature_scale;
  c.growth_field.diameter_m = needle_diameter_for_curvature_m;
  c.sinusoidal_amplitude_deg = std::clamp(needle_sinusoidal_amplitude_deg, 0.0f, 45.0f);
  c.sinusoidal_frequency_cycles = std::clamp(needle_sinusoidal_frequency_cycles, 0.0f, 12.0f);
  c.sinusoidal_phase_rad = 0.0f;

  // Mechanical material profile (defaults are inert -> elastica skipped).
  c.material_profile.young_modulus_baseline_Pa = needle_young_modulus_baseline_Pa;
  c.material_profile.base_radius_m = needle_cross_section_width_radius_m;
  c.material_profile.tip_radius_m = needle_cross_section_width_radius_m;
  c.material_profile.density_kg_m3 = needle_density_kg_m3;
  c.material_profile.lignification.t_init_years = t_flush;
  c.material_profile.lignification.maturation_years = needle_lignification_maturation_years;
  c.material_profile.lignification.function.kind = GrowthFunctionKind::Logistic;

  // Explicit per-needle profiles within a fascicle. Keep this deterministic
  // and local to the cluster so topology and upstream RNG sequencing remain stable.
  c.per_needle_profiles.clear();
  c.per_needle_profiles.reserve(static_cast<size_t>(std::max(1, c.count)));
  const float length_cv = std::max(0.0f, needle_per_needle_length_cv);
  const float curvature_cv = std::max(0.0f, needle_per_needle_curvature_cv);
  const float radius_cv = std::max(0.0f, needle_per_needle_radius_cv);
  const float modulus_cv = std::max(0.0f, needle_per_needle_modulus_cv);
  const float density_cv = std::max(0.0f, needle_per_needle_density_cv);
  const float wave_amplitude_cv = std::max(0.0f, needle_per_needle_wave_amplitude_cv);
  const float wave_frequency_cv = std::max(0.0f, needle_per_needle_wave_frequency_cv);
  const float wave_phase_cv = std::max(0.0f, needle_per_needle_wave_phase_cv);
  const int needle_count = std::max(1, c.count);
  for (int i = 0; i < needle_count; ++i) {
    PineNeedleInstanceProfile profile;
    profile.growth_field = c.growth_field;
    profile.material_profile = c.material_profile;
    const float profile_branching_relax_gdd = std::max(
      0.0f,
      DeterministicDistributionSample(
        p.distributions.needle_branching_relax_gdd,
        c.node_random,
        static_cast<uint32_t>(i),
        0x4ab14377u));
    profile.branching_relax_years = profile_branching_relax_gdd / kPineGddPerYear;
    profile.sinusoidal_amplitude_deg = c.sinusoidal_amplitude_deg;
    profile.sinusoidal_frequency_cycles = c.sinusoidal_frequency_cycles;
    profile.sinusoidal_phase_rad = c.sinusoidal_phase_rad;

    const float length_jitter = DeterministicSignedJitter(c.node_random, i, 0x13579bdfu);
    const float curvature_jitter = DeterministicSignedJitter(c.node_random, i, 0x2468ace0u);
    const float radius_jitter = DeterministicSignedJitter(c.node_random, i, 0x0f1e2d3cu);
    const float modulus_jitter = DeterministicSignedJitter(c.node_random, i, 0x55aa55aau);
    const float density_jitter = DeterministicSignedJitter(c.node_random, i, 0xa55aa55au);
    const float wave_amplitude_jitter = DeterministicSignedJitter(c.node_random, i, 0x5e7711a9u);
    const float wave_frequency_jitter = DeterministicSignedJitter(c.node_random, i, 0x1f6c3d2bu);
    const float wave_phase_base = DeterministicSignedJitter(c.node_random, i, 0x8bc4ef01u);
    const float wave_phase_jitter = DeterministicSignedJitter(c.node_random, i, 0x6395ad72u);

    profile.length_scale = std::clamp(1.0f + length_cv * length_jitter, 0.05f, 3.0f);
    profile.radius_scale = std::clamp(1.0f + radius_cv * radius_jitter, 0.05f, 3.0f);

    const float curvature_scale_per_needle =
        std::clamp(1.0f + curvature_cv * curvature_jitter, 0.0f, 3.0f);
    profile.growth_field.adaxial_bias *= curvature_scale_per_needle;
    profile.growth_field.abaxial_bias *= curvature_scale_per_needle;
    profile.growth_field.gradient_per_arclen *= curvature_scale_per_needle;

    const float modulus_scale = std::clamp(1.0f + modulus_cv * modulus_jitter, 0.0f, 3.0f);
    const float density_scale = std::clamp(1.0f + density_cv * density_jitter, 0.0f, 3.0f);
    profile.material_profile.young_modulus_baseline_Pa *= modulus_scale;
    profile.material_profile.base_radius_m *= profile.radius_scale;
    profile.material_profile.tip_radius_m *= profile.radius_scale;
    profile.material_profile.density_kg_m3 *= density_scale;

    constexpr float kPi = 3.14159265358979323846f;
    constexpr float kTwoPi = 6.28318530717958647692f;
    const float wave_amplitude_scale =
      std::clamp(1.0f + wave_amplitude_cv * wave_amplitude_jitter, 0.0f, 3.0f);
    const float wave_frequency_scale =
      std::clamp(1.0f + wave_frequency_cv * wave_frequency_jitter, 0.0f, 3.0f);
    profile.sinusoidal_amplitude_deg = std::clamp(
      profile.sinusoidal_amplitude_deg * wave_amplitude_scale, 0.0f, 45.0f);
    profile.sinusoidal_frequency_cycles = std::clamp(
      profile.sinusoidal_frequency_cycles * wave_frequency_scale, 0.0f, 12.0f);
    const float base_phase_rad = (0.5f * (wave_phase_base + 1.0f)) * kTwoPi;
    const float phase_randomness_deg = std::clamp(
      needle_sinusoidal_phase_randomness_deg *
        std::clamp(1.0f + wave_phase_cv * wave_phase_jitter, 0.0f, 3.0f),
      0.0f,
      180.0f);
    const float phase_jitter_rad =
      phase_randomness_deg * wave_phase_jitter * (kPi / 180.0f);
    profile.sinusoidal_phase_rad = NormalizeRadians(base_phase_rad + phase_jitter_rad);

    c.per_needle_profiles.emplace_back(profile);
  }
  return c;
}

/// Draw fresh plastochron (years) for stamping on a (root or lateral) apex.
/// Clamp matches the bound previously enforced inside `Sample()`.
inline float SamplePlastochronYears(const SampledPineParams& p, std::mt19937& node_rng) {
  return std::clamp(
      SampleDistribution(p.distributions.plastochron_gdd, node_rng) / kPineGddPerYear,
      0.05f, 4.0f);
}

/// Draw fresh max_phytomers_per_seasonal_growth for stamping on an apex at
/// year boundaries. Clamp matches `Sample()`.
inline int SampleMaxPhytomersPerSeasonalGrowth(const SampledPineParams& p,
                                               std::mt19937& node_rng) {
  return std::clamp(
      static_cast<int>(std::round(
          SampleDistribution(p.distributions.max_phytomers_per_seasonal_growth, node_rng))),
      1, 200);
}

/// Draw fresh per-year bare-phytomer count from the descriptor's
/// `bare_zone_fraction` distribution. Stamped on the apex at year rollover
/// (R0) and at apex creation, so R1's needle gate is a stable integer
/// comparison `phytomers_this_year < bare_phytomers_this_year` for the
/// entire year. `season_phytomers` is the per-year cap stamped on the same
/// apex so the bare count cannot exceed the year's emissions.
inline int SampleBarePhytomersThisYear(const SampledPineParams& p,
                                       int season_phytomers,
                                       std::mt19937& node_rng) {
  const float fraction = std::clamp(
      SampleDistribution(p.distributions.bare_zone_fraction, node_rng),
      0.0f, 0.95f);
  const int cap = std::max(0, season_phytomers);
  const int bare = static_cast<int>(std::round(fraction * static_cast<float>(cap)));
  return std::clamp(bare, 0, cap);
}

/// Lateral apex factory. Draws fresh per-apex stamped values for
/// plastochron and per-season cap so each lateral evolves with its own
/// (resampled) seasonal cadence.
inline PineApex MakeLateralApex(const SampledPineParams& params,
                                int order,
                                float phyllotaxis_phase_deg,
                                float t_init_years,
                                int year_index,
                                std::mt19937& node_rng) {
  PineApex apex;
  apex.order = order;
  apex.phyllotaxis_phase = phyllotaxis_phase_deg;
  apex.node_random = SampleUnit01(node_rng);
  apex.year_index = year_index;
  apex.phytomers_this_year = 0;
  // Lateral apices start their plastochron clock from the moment the whorl
  // bud activates, so the first phytomer waits a full plastochron after bud
  // break before extending.
  apex.t_last_emission_years = t_init_years;
  apex.sampled_plastochron_years = SamplePlastochronYears(params, node_rng);
  apex.sampled_max_phytomers_per_seasonal_growth =
      SampleMaxPhytomersPerSeasonalGrowth(params, node_rng);
  apex.bare_phytomers_this_year = SampleBarePhytomersThisYear(
      params, apex.sampled_max_phytomers_per_seasonal_growth, node_rng);
  apex.previous_season_completion_ratio = 1.0f;
  apex.previous_season_vigor = 1.0f;
  return apex;
}

}  // namespace pine_detail

// ---------------------------------------------------------------------------
// Topology rules (phytomer grammar).
//
//   R0. Apex(year_index < clock.YearIndex())
//        -> Apex(year_index = clock.YearIndex(),
//                 phytomers_this_year = 0,
//                 t_last_emission_years = now - plastochron,
//                 sampled_max_phytomers_per_seasonal_growth = <fresh draw>,
//                 sampled_plastochron_years            = <fresh draw>,
//                 bare_phytomers_this_year             = round(
//                     <fresh bare_zone_fraction draw> *
//                     sampled_max_phytomers_per_seasonal_growth))
//                                                  (year rollover; immortal)
//
//   R1. Apex(in_active_season &&
//            phytomers_this_year < sampled_max_phytomers_per_seasonal_growth &&
//            now - t_last_emission >= sampled_plastochron_years)
//        -> Internode(phytomer)
//           + (NeedleCluster IF phytomers_this_year >= bare_phytomers_this_year)
//           + WhorlBud(if max_branching_order >= order+1 && branches_per_whorl > 0)
//           + Apex(phytomers_this_year + 1, t_last_emission = now)
//
//   R3. WhorlBud(dormancy > 0) -> WhorlBud                  (dormant hold)
//
//   R4. WhorlBud(dormancy <= 0) -> N * Apex(branch, order+1) (activation)
// ---------------------------------------------------------------------------

inline std::vector<PineRule> CreatePineTopologyRules(const SampledPineParams& params) {
  std::vector<PineRule> rules;

  // R0: year rollover. Highest priority so a stale apex always refreshes its
  // seasonal-growth budget before R1 considers emitting a new phytomer.
  {
    PineRule rule;
    rule.predecessor_symbol = PineSymbol::Apex;
    rule.priority = 30;
    rule.condition = [](const RuleContext<PineGraph>& ctx) -> bool {
      const auto& apex = ctx.self.data.Get<PineApex>();
      return ctx.graph.data.clock.YearIndex() > apex.year_index;
    };
    rule.produce = [params](RuleContext<PineGraph>& ctx) -> ProductionResult<PineModuleData> {
      const auto& apex = ctx.self.data.Get<PineApex>();
      const auto& clock = ctx.graph.data.clock;
      const float now_years = clock.NowYears();
      const float previous_cap = static_cast<float>(std::max(
        1, apex.sampled_max_phytomers_per_seasonal_growth));
      const float previous_completion_ratio = std::clamp(
        static_cast<float>(apex.phytomers_this_year) / previous_cap,
        0.0f, 1.0f);
      const float previous_vigor = std::clamp(
        previous_completion_ratio,
        std::clamp(params.needle_bud_storage_completion_floor, 0.0f, 1.0f),
        1.0f);
      // Year rollover: draw fresh stamped plastochron + per-season cap so
      // each year evolves with its own (resampled) cadence.
      auto node_rng = MakeNodeRng(apex.node_random, 0xA1F00D00u);
      ProductionResult<PineModuleData> result;
      Successor<PineModuleData> s;
      s.is_branch = false;
      s.symbol_id = PineSymbol::Apex;
      PineApex next = apex;
      next.year_index = clock.YearIndex();
      next.phytomers_this_year = 0;
      next.sampled_plastochron_years =
          pine_detail::SamplePlastochronYears(params, node_rng);
      next.sampled_max_phytomers_per_seasonal_growth =
          pine_detail::SampleMaxPhytomersPerSeasonalGrowth(params, node_rng);
      next.bare_phytomers_this_year = pine_detail::SampleBarePhytomersThisYear(
          params, next.sampled_max_phytomers_per_seasonal_growth, node_rng);
        next.previous_season_completion_ratio = previous_completion_ratio;
        next.previous_season_vigor = previous_vigor;
      next.node_random = SampleUnit01(node_rng);
      // Allow the first phytomer of the new year to fire immediately.
      next.t_last_emission_years =
          now_years - std::max(0.0f, next.sampled_plastochron_years);
      s.data.Set<PineApex>(next);
      result.successors.push_back(std::move(s));
      return result;
    };
    rules.push_back(std::move(rule));
  }

  // R1: phytomer extension.
  {
    PineRule rule;
    rule.predecessor_symbol = PineSymbol::Apex;
    rule.priority = 10;
    rule.condition = [](const RuleContext<PineGraph>& ctx) -> bool {
      const auto& apex = ctx.self.data.Get<PineApex>();
      const auto& clock = ctx.graph.data.clock;
      // Only emit during the active growth season.
      if (!clock.InActiveSeason()) return false;
      // Stop after this year's phytomer cap is reached; R0 will reset on
      // the next dormant->active edge. Cap is stamped per-year in R0 so
      // condition + produce see the same value.
      if (apex.phytomers_this_year >= apex.sampled_max_phytomers_per_seasonal_growth) {
        return false;
      }
      // Plastochron gate. Stamped per-emission on the apex by R0 / R1.
      const float now_years = clock.NowYears();
      const float elapsed = now_years - apex.t_last_emission_years;
      return elapsed >= std::max(0.0f, apex.sampled_plastochron_years);
    };
    rule.produce = [params](RuleContext<PineGraph>& ctx) -> ProductionResult<PineModuleData> {
      const auto& apex = ctx.self.data.Get<PineApex>();
      const auto& clock = ctx.graph.data.clock;
      auto node_rng = MakeNodeRng(apex.node_random, 0xA1F00D01u);
      const float now_years = clock.NowYears();
      const bool is_first_on_axis = (apex.phytomers_this_year == 0 && apex.year_index == 0);

      // Per-emission resamples for fields used in this production. Note that
      // `bare_zone_fraction` is intentionally NOT resampled here — it was
      // stamped on the apex at year rollover (R0) as an integer
      // `bare_phytomers_this_year` count, so the needle gate below is a
      // stable per-year decision.
      const int max_branching_order = std::clamp(
          static_cast<int>(std::round(
              SampleDistribution(params.distributions.max_branching_order, node_rng))),
          0, 8);
      // Roll advance for the continuation apex (the next phytomer's phyllotactic phase).
      const float roll_phyllotaxis_deg =
          SampleDistribution(params.distributions.branch_roll_phyllotaxis_deg, node_rng);

      ProductionResult<PineModuleData> result;

      // 1) The phytomer's internode.
      {
        Successor<PineModuleData> s;
        s.is_branch = false;
        s.symbol_id = PineSymbol::Internode;
        auto internode = pine_detail::MakePhytomerInternode(
            params, apex, is_first_on_axis, node_rng, now_years);
        s.data.Set<PineInternode>(internode);
        result.successors.push_back(std::move(s));
      }

      // 2) Optional needle cluster: count-based bare-zone gate. The first
      //    `bare_phytomers_this_year` phytomers of every year emit an
      //    internode without a needle cluster; every subsequent phytomer
      //    in the same year carries one cluster. This replaces the
      //    previous `(now - YearStart) / SeasonLength >= bare_zone`
      //    temporal gate, which could deterministically drop ALL needles
      //    in years where (a) the cap was small enough that no phytomer
      //    crossed the bare fraction before the season ended, or (b) the
      //    per-emission bare_zone draw landed above the achievable
      //    temporal_progress. The new gate is independent of the
      //    GDD<->calendar coupling and stable across plastochron resamples.
      if (apex.phytomers_this_year >= apex.bare_phytomers_this_year) {
        Successor<PineModuleData> s;
        s.is_branch = true;
        s.symbol_id = PineSymbol::NeedleCluster;
        // Anchor the single cluster at the distal end of the phytomer; with
        // one phytomer per cluster this is geometrically a point on the new
        // internode rather than a fractional position along an annual shoot.
        s.data.Set<PineNeedleCluster>(
            pine_detail::MakeNeedleCluster(
                params,
                /*s_along_norm=*/1.0f,
                /*roll_offset_deg=*/apex.phyllotaxis_phase,
                node_rng,
                /*internode_t_init_years=*/now_years,
                /*parent_axis_order=*/apex.order,
                /*phytomer_index_this_year=*/apex.phytomers_this_year,
                /*phytomers_per_season=*/apex.sampled_max_phytomers_per_seasonal_growth,
                /*initiation_year_index=*/apex.year_index,
                /*previous_season_vigor=*/apex.previous_season_vigor));
        result.successors.push_back(std::move(s));
      }

      // 3) Overwintering whorl bud (if the parent can carry laterals).
      // Per-whorl resamples: branches_per_whorl, branch_insertion_angle,
      // whorl_dormancy_years (chronological, not GDD). max_branching_order
      // is also per-emission.
      const int branches_per_whorl = std::clamp(
          static_cast<int>(std::round(
              SampleDistribution(params.distributions.branches_per_whorl, node_rng))),
          0, 32);
      const int next_order = apex.order + 1;
      if (max_branching_order >= next_order && branches_per_whorl > 0) {
        const float branch_insertion_angle_deg = std::clamp(
            SampleDistribution(params.distributions.branch_insertion_angle_deg, node_rng),
            -85.0f, 85.0f);
        // Chronological: bud release is chilling/photoperiod-driven, not heat-driven.
        const float whorl_dormancy_years = std::max(
            0.0f,
            SampleDistribution(params.distributions.whorl_dormancy_years, node_rng));
        Successor<PineModuleData> s;
        s.is_branch = true;
        s.symbol_id = PineSymbol::WhorlBud;
        PineWhorlBud bud;
        bud.insertion_angle = branch_insertion_angle_deg;
        bud.lateral_count = branches_per_whorl;
        bud.dormancy_years_remaining =
            std::max(0, static_cast<int>(std::round(whorl_dormancy_years)));
        bud.initial_dormancy_years = bud.dormancy_years_remaining;
        bud.order = next_order;
        bud.phyllotaxis_phase = apex.phyllotaxis_phase;
        bud.node_random = SampleUnit01(node_rng);
        s.data.Set<PineWhorlBud>(bud);
        result.successors.push_back(std::move(s));
      }

      // 4) Continuation apex. Re-stamp plastochron (per-emission cadence)
      //    while preserving the per-season cap stamped at year boundaries.
      {
        Successor<PineModuleData> s;
        s.is_branch = false;
        s.symbol_id = PineSymbol::Apex;
        PineApex next = apex;
        next.phytomers_this_year = apex.phytomers_this_year + 1;
        next.phyllotaxis_phase = pine_detail::NormalizeDegrees(
            apex.phyllotaxis_phase + roll_phyllotaxis_deg);
        next.node_random = SampleUnit01(node_rng);
        next.t_last_emission_years = now_years;
        next.sampled_plastochron_years =
            pine_detail::SamplePlastochronYears(params, node_rng);
        // Per-season cap inherits unchanged from `apex` via the copy above.
        s.data.Set<PineApex>(next);
        result.successors.push_back(std::move(s));
      }
      return result;
    };
    rules.push_back(std::move(rule));
  }

  // R3: whorl bud dormant hold.
  // Chronological dormancy countdown is maintained in PineGrowthModel.
  {
    PineRule rule;
    rule.predecessor_symbol = PineSymbol::WhorlBud;
    rule.priority = 10;
    rule.condition = [](const RuleContext<PineGraph>& ctx) -> bool {
      return ctx.self.data.Get<PineWhorlBud>().dormancy_years_remaining > 0;
    };
    rule.produce = [](RuleContext<PineGraph>& ctx) -> ProductionResult<PineModuleData> {
      const auto bud = ctx.self.data.Get<PineWhorlBud>();
      ProductionResult<PineModuleData> result;
      Successor<PineModuleData> s;
      s.is_branch = false;
      s.symbol_id = PineSymbol::WhorlBud;
      s.data.Set<PineWhorlBud>(bud);
      result.successors.push_back(std::move(s));
      return result;
    };
    rules.push_back(std::move(rule));
  }

  // R4: whorl bud activation -> N lateral apices.
  {
    PineRule rule;
    rule.predecessor_symbol = PineSymbol::WhorlBud;
    rule.priority = 5;
    rule.condition = [](const RuleContext<PineGraph>& ctx) -> bool {
      return ctx.self.data.Get<PineWhorlBud>().dormancy_years_remaining <= 0;
    };
    rule.produce = [params](RuleContext<PineGraph>& ctx) -> ProductionResult<PineModuleData> {
      const auto& bud = ctx.self.data.Get<PineWhorlBud>();
      auto node_rng = MakeNodeRng(bud.node_random, 0xA1F00D04u);
      const auto& clock = ctx.graph.data.clock;
      const float now_years = clock.NowYears();
      const int year_index = clock.YearIndex();
      ProductionResult<PineModuleData> result;
      const int n = std::max(0, bud.lateral_count);
      for (int i = 0; i < n; ++i) {
        // Per-lateral resample of phyllotaxis spread.
        const float roll_phyllotaxis_deg =
            SampleDistribution(params.distributions.branch_roll_phyllotaxis_deg, node_rng);
        Successor<PineModuleData> s;
        s.is_branch = true;
        s.symbol_id = PineSymbol::Apex;
        const float phase = pine_detail::NormalizeDegrees(
            bud.phyllotaxis_phase + static_cast<float>(i) * roll_phyllotaxis_deg);
        s.data.Set<PineApex>(
            pine_detail::MakeLateralApex(params, bud.order, phase, now_years, year_index, node_rng));
        result.successors.push_back(std::move(s));
      }
      return result;
    };
    rules.push_back(std::move(rule));
  }

  return rules;
}

// ---------------------------------------------------------------------------
// Growth rules.
//
// Senescence and abscission are driven continuously in
// PineGrowthModel::UpdateNodeInfoImpl from `cluster.continuous_growth.t_init`,
// `lifespan_years`, and `needle_browning_years`. No discrete growth rules
// are required.
// ---------------------------------------------------------------------------

inline std::vector<PineRule> CreatePineGrowthRules(const SampledPineParams& /*params*/) {
  return {};
}

}  // namespace l_system_plugin
