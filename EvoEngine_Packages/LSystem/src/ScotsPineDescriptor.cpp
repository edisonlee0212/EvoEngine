#include "ScotsPineDescriptor.hpp"
#include <yaml-cpp/yaml.h>
#include <Application.hpp>
#include <Scene.hpp>
#include <Transform.hpp>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include "LSystemDescriptorDefaults.hpp"
#include "LSystemSerializationAdapters.hpp"
#include "ScotsPine.hpp"

using namespace l_system_package;
using namespace evo_engine;

// ===========================================================================
// File-local helpers (defaults file resolution, loading, target_gdd sampling).
// File extension: .spine
// ===========================================================================
namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr char kScotsPineDescriptorName[] = "ScotsPineDescriptor";

const std::array<std::filesystem::path, 8> kScotsPineResourceCandidates = {
    std::filesystem::path("./LSystemProjectAssets/Assets/New ScotsPineDescriptor.spine"),
    std::filesystem::path("./Resources/DigitalAgricultureProject/Assets/New ScotsPineDescriptor.spine"),
    std::filesystem::path("./DigitalAgricultureProject/Assets/New ScotsPineDescriptor.spine"),
    std::filesystem::path("./04_EvoEngine/Resources/DigitalAgricultureProject/Assets/") /
        "New ScotsPineDescriptor.spine",
    std::filesystem::path("./LSystemResources/Defaults/ScotsPineDescriptor_Default.spine"),
    std::filesystem::path("./EvoEngine_Packages/LSystem/Internals/LSystemResources/Defaults/") /
        "ScotsPineDescriptor_Default.spine",
    std::filesystem::path("./EvoEngine_Plugins/LSystem/Internals/LSystemResources/Defaults/") /
        "ScotsPineDescriptor_Default.spine",
    std::filesystem::path("./04_EvoEngine/EvoEngine_Packages/LSystem/Internals/") /
        "LSystemResources/Defaults/ScotsPineDescriptor_Default.spine"};

const std::array<std::filesystem::path, 3> kScotsPineProjectAssetCandidates = {
    std::filesystem::path("LSystemProjectAssets") / "Assets" / "New ScotsPineDescriptor.spine",
    std::filesystem::path("LSystem") / "New ScotsPineDescriptor.spine", "New ScotsPineDescriptor.spine"};

const std::array<std::filesystem::path, 2> kScotsPineWritableTemplateCandidates = {
    std::filesystem::path("./Resources/DigitalAgricultureProject/Assets/") / "New ScotsPineDescriptor.spine",
    std::filesystem::path("./EvoEngine_Packages/LSystem/Internals/LSystemResources/Defaults/") /
        "ScotsPineDescriptor_Default.spine"};

const std::filesystem::path kScotsPineFallbackDefaultsPath =
    std::filesystem::path("./LSystemResources/Defaults/ScotsPineDescriptor_Default.spine");

void SetCurveToSinusoidalRange(evo_engine::Curve2D& curve, const float y0, const float y1,
                               const int sample_count = 17) {
  curve.SetTangent(false);
  auto& values = curve.UnsafeGetValues();
  values.clear();
  const int safe_samples = std::max(2, sample_count);
  const float v0 = std::clamp(y0, 0.0f, 1.0f);
  const float v1 = std::clamp(y1, 0.0f, 1.0f);
  for (int i = 0; i < safe_samples; ++i) {
    const float x = static_cast<float>(i) / static_cast<float>(safe_samples - 1);
    const float ramp = 0.5f - 0.5f * std::cos(kPi * x);
    const float y = v0 + (v1 - v0) * ramp;
    values.emplace_back(x, y);
  }
}

void SetCurveToSinusoidal01(evo_engine::Curve2D& curve, const int sample_count = 17) {
  SetCurveToSinusoidalRange(curve, 0.0f, 1.0f, sample_count);
}

void SetCurveLinear01(evo_engine::Curve2D& curve, const float y0, const float y1, const int sample_count = 9) {
  curve.SetTangent(false);
  auto& values = curve.UnsafeGetValues();
  values.clear();
  const int safe_samples = std::max(2, sample_count);
  const float v0 = std::clamp(y0, 0.0f, 1.0f);
  const float v1 = std::clamp(y1, 0.0f, 1.0f);
  for (int i = 0; i < safe_samples; ++i) {
    const float x = static_cast<float>(i) / static_cast<float>(safe_samples - 1);
    const float y = v0 + (v1 - v0) * x;
    values.emplace_back(x, y);
  }
}

void ConfigureMaturityDistributionDefaults(evo_engine::PlottedDistribution<float>& distribution,
                                           const float default_deviation_max = 0.0f) {
  distribution.mean.min_value = 0.0f;
  distribution.mean.max_value = 1.0f;
  SetCurveToSinusoidal01(distribution.mean.curve);

  distribution.deviation.min_value = 0.0f;
  distribution.deviation.max_value = std::max(0.0f, default_deviation_max);
  SetCurveToSinusoidal01(distribution.deviation.curve);
}

void ConfigureNeedleCrossSectionProfileDefaults(evo_engine::PlottedDistribution<float>& distribution,
                                                const float base_multiplier, const float tip_multiplier,
                                                const float default_deviation_max = 0.0f) {
  distribution.mean.min_value = 0.0f;
  distribution.mean.max_value = 1.0f;
  SetCurveLinear01(distribution.mean.curve, base_multiplier, tip_multiplier, 2);

  distribution.deviation.min_value = 0.0f;
  distribution.deviation.max_value = std::max(0.0f, default_deviation_max);
  SetCurveLinear01(distribution.deviation.curve, 0.0f, 0.0f, 2);
}

void ConfigureNeedleCrossSectionTemporalMaturityDefaults(evo_engine::PlottedDistribution<float>& distribution,
                                                         const float default_deviation_max = 0.0f) {
  distribution.mean.min_value = 0.0f;
  distribution.mean.max_value = 1.0f;
  SetCurveToSinusoidalRange(distribution.mean.curve, 0.25f, 1.0f);

  distribution.deviation.min_value = 0.0f;
  distribution.deviation.max_value = std::max(0.0f, default_deviation_max);
  SetCurveLinear01(distribution.deviation.curve, 0.0f, 0.0f);
}

void ConfigurePineMaturityDefaults(ScotsPineDescriptor& descriptor) {
  ConfigureMaturityDistributionDefaults(descriptor.internode_length_maturity_curve);
  ConfigureMaturityDistributionDefaults(descriptor.internode_width_maturity_curve);
  ConfigureMaturityDistributionDefaults(descriptor.needle_length_maturity_curve);
  ConfigureNeedleCrossSectionProfileDefaults(descriptor.needle_cross_section_width_profile, 0.348f, 0.098f);
  ConfigureNeedleCrossSectionProfileDefaults(descriptor.needle_cross_section_thickness_profile, 0.5f, 0.184f);
  ConfigureNeedleCrossSectionTemporalMaturityDefaults(descriptor.needle_cross_section_temporal_maturity_curve);
}

double GetSteadyTimeSeconds() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

std::filesystem::path ResolveDefaultScotsPineDescriptorPath() {
  return descriptor_defaults::ResolveExistingDefaultsPath(kScotsPineResourceCandidates,
                                                          kScotsPineProjectAssetCandidates);
}

std::filesystem::path ResolveWritableScotsPineDescriptorDefaultsPath() {
  return descriptor_defaults::ResolveWritableDefaultsPath(
      kScotsPineResourceCandidates, kScotsPineProjectAssetCandidates, kScotsPineWritableTemplateCandidates,
      kScotsPineFallbackDefaultsPath);
}

void LoadSingleDistributionWithScalarFallback(const YAML::Node& in, const char* key,
                                              evo_engine::SingleDistribution<float>& distribution) {
  if (!in[key])
    return;
  const auto& node = in[key];
  if (node.IsMap()) {
    distribution.Load(key, in);
    return;
  }
  if (node.IsScalar()) {
    distribution.mean = node.as<float>();
    distribution.deviation = 0.0f;
  }
}

/// Backward-compat loader for fields that were renamed from `*_gdd` (heat
/// sum) to `*_years` (chronological) by the FSPM Rule of Ontogeny audit.
/// If `legacy_key` exists in `in`, read its value (map-or-scalar) and
/// divide both mean and deviation by `kPineGddPerYear` so the distribution
/// lands in years on the renamed member. Called only when the new
/// `*_years` key is absent.
void LoadLegacyGddDistributionAsYears(const YAML::Node& in, const char* legacy_key,
                                      evo_engine::SingleDistribution<float>& years_distribution) {
  if (!in[legacy_key])
    return;
  evo_engine::SingleDistribution<float> tmp{};
  LoadSingleDistributionWithScalarFallback(in, legacy_key, tmp);
  years_distribution.mean = std::max(0.0f, tmp.mean) / kPineGddPerYear;
  years_distribution.deviation = std::max(0.0f, tmp.deviation) / kPineGddPerYear;
}

bool LoadScotsPineDescriptorDefaultsFromFile(ScotsPineDescriptor& descriptor, const std::filesystem::path& file_path) {
  YAML::Node defaults;
  if (!descriptor_defaults::LoadDefaultsYamlMap(file_path, defaults, kScotsPineDescriptorName)) {
    return false;
  }
  DeserializeScotsPineDescriptor(defaults, descriptor);
  return true;
}

}  // namespace

// ===========================================================================
// Constructor - load defaults from disk if available.
// ===========================================================================
ScotsPineDescriptor::ScotsPineDescriptor() {
  ConfigurePineMaturityDefaults(*this);
  const auto defaults_path = ResolveDefaultScotsPineDescriptorPath();
  if (!LoadScotsPineDescriptorDefaultsFromFile(*this, defaults_path)) {
    static bool warned_once = false;
    if (!warned_once) {
      warned_once = true;
      EVOENGINE_WARNING("ScotsPineDescriptor defaults file not found or invalid. Using inline member defaults.");
    }
  }
}

float ScotsPineDescriptor::SampleTargetGdd(const uint32_t seed) const {
  std::mt19937 rng(seed);
  return std::max(0.0f, SampleDistribution(target_gdd, rng));
}

std::filesystem::path ScotsPineDescriptor::ResolveWritableDefaultsPath() const {
  return ResolveWritableScotsPineDescriptorDefaultsPath();
}

// ===========================================================================
// Sample
//
// RNG draw order (LOCKED - append-only for reproducibility across schema
// extensions). The phytomer model schema is the canonical baseline.
//
//   1. plant_random
//   2. max_branching_order
//   3. plastochron_gdd                 -> years = gdd / kPineGddPerYear
//   4. max_phytomers_per_seasonal_growth
//   5. branches_per_whorl
//   6. whorl_dormancy_years            -> chronological (years on disk)
//   7. branch_insertion_angle_deg
//   8. branch_roll_phyllotaxis_deg
//   9. internode_length_m
//  10. leader_internode_thickness_m
//  11. lateral_length_ratio
//  12. lateral_thickness_ratio
//  13. bare_zone_fraction
//  14. needle_count_per_cluster
//  15. needle_length_m
//  16. needle_lifespan_years           -> chronological (years on disk)
//  17. needle_browning_years           -> chronological (years on disk)
//  18. needle_flush_delay_gdd          -> years = gdd / kPineGddPerYear
//  19. internode_maturation_gdd        -> years = gdd / kPineGddPerYear
//  20. needle_maturation_gdd           -> years = gdd / kPineGddPerYear
//  21. gravitropism_first_order         -> main-stem-only curvature (leader order 0)
//  22. tropism array (per entry: usage roll, dir_x/y/z, strength)
//  23. internode_length_per_node_cv
//  24. internode_thickness_per_node_cv
//  25. branch_angle_per_node_sigma_deg
//  26. roll_phyllotaxis_per_node_sigma_deg
//  27. needle_curvature_adaxial_bias
//  28. needle_curvature_abaxial_bias
//  29. needle_curvature_gradient_per_arclen
//  30. needle_diameter_for_curvature_m
//  31. needle_young_modulus_baseline_Pa
//  32. needle_lignification_maturation_years
//  33. needle_cross_section_width_max_m
//  34. needle_cross_section_thickness_max_m
//  35. needle_density_kg_m3
//  36. gravity_m_s2
//  37. needle_per_needle_length_cv
//  38. needle_per_needle_curvature_cv
//  39. needle_per_needle_radius_cv
//  40. needle_per_needle_modulus_cv
//  41. needle_per_needle_density_cv
//  42. needle_sinusoidal_amplitude_deg
//  43. needle_sinusoidal_frequency_cycles
//  44. needle_sinusoidal_phase_randomness_deg
//  45. needle_per_needle_wave_amplitude_cv
//  46. needle_per_needle_wave_frequency_cv
//  47. needle_per_needle_wave_phase_cv
//  48. initial_orientation_yaw_deg      -> sampled once per plant (root yaw around +Y)
//
// Non-stochastic scalar controls (for example `needle_segment_count`) are
// copied/clamped without RNG draws to preserve the locked draw order above.
// ===========================================================================
SampledPineParams ScotsPineDescriptor::Sample(std::mt19937& rng) const {
  SampledPineParams out;

  out.plant_random = SampleUnit01(rng);

  out.max_branching_order =
      std::clamp(static_cast<int>(std::round(SampleDistribution(max_branching_order, rng))), 0, 4);
  out.plastochron_years = std::max(1e-4f, SampleDistribution(plastochron_gdd, rng) / kPineGddPerYear);
  out.max_phytomers_per_seasonal_growth =
      std::clamp(static_cast<int>(std::round(SampleDistribution(max_phytomers_per_seasonal_growth, rng))), 1, 1024);

  out.branches_per_whorl = std::clamp(static_cast<int>(std::round(SampleDistribution(branches_per_whorl, rng))), 0, 12);
  // Chronological (years). No GDD->years conversion.
  out.whorl_dormancy_years = std::max(0.0f, SampleDistribution(whorl_dormancy_years, rng));
  out.branch_insertion_angle_deg = std::clamp(SampleDistribution(branch_insertion_angle_deg, rng), -85.0f, 85.0f);
  out.branch_roll_phyllotaxis_deg = SampleDistribution(branch_roll_phyllotaxis_deg, rng);

  out.internode_length_m = std::clamp(SampleDistribution(internode_length_m, rng), 0.0001f, 0.50f);
  out.leader_internode_thickness_m = std::clamp(SampleDistribution(leader_internode_thickness_m, rng), 0.0002f, 0.05f);
  // Keep stem slenderness physically plausible (avoid "fat stump" defaults).
  const float max_sane_thickness = std::max(0.0002f, 0.40f * out.internode_length_m * 8.0f);
  out.leader_internode_thickness_m = std::min(out.leader_internode_thickness_m, max_sane_thickness);
  out.lateral_length_ratio = std::clamp(SampleDistribution(lateral_length_ratio, rng), 0.05f, 1.0f);
  out.lateral_thickness_ratio = std::clamp(SampleDistribution(lateral_thickness_ratio, rng), 0.05f, 1.5f);

  out.bare_zone_fraction = std::clamp(SampleDistribution(bare_zone_fraction, rng), 0.0f, 0.95f);
  out.needle_count_per_cluster =
      std::clamp(static_cast<int>(std::round(SampleDistribution(needle_count_per_cluster, rng))), 1, 6);
  out.needle_segment_count = std::clamp(needle_segment_count, 3, 128);
  out.needle_length_m = std::clamp(SampleDistribution(needle_length_m, rng), 0.005f, 0.30f);
  // Chronological lifespan and browning (years). No GDD->years conversion.
  out.needle_lifespan_years = std::max(1, static_cast<int>(std::round(SampleDistribution(needle_lifespan_years, rng))));
  out.needle_browning_years = std::max(0.0f, SampleDistribution(needle_browning_years, rng));
  out.needle_flush_delay_years = std::max(0.0f, SampleDistribution(needle_flush_delay_gdd, rng) / kPineGddPerYear);
  out.internode_maturation_years =
      std::clamp(SampleDistribution(internode_maturation_gdd, rng) / kPineGddPerYear, 0.0f, 4.0f);
  out.needle_maturation_years =
      std::clamp(SampleDistribution(needle_maturation_gdd, rng) / kPineGddPerYear, 0.0f, 1.0f);
  out.needle_order_length_attenuation = std::clamp(needle_order_length_attenuation, 0.0f, 1.0f);
  out.needle_order_radius_attenuation = std::clamp(needle_order_radius_attenuation, 0.0f, 1.0f);
  out.needle_order_min_length_scale = std::clamp(needle_order_min_length_scale, 0.10f, 1.0f);
  out.needle_order_min_radius_scale = std::clamp(needle_order_min_radius_scale, 0.10f, 1.0f);
  out.needle_intra_year_base_ratio = std::clamp(needle_intra_year_base_ratio, 0.0f, 1.0f);
  out.needle_intra_year_sigmoid_steepness = std::max(0.01f, needle_intra_year_sigmoid_steepness);
  out.needle_intra_year_sigmoid_midpoint_fraction = std::clamp(needle_intra_year_sigmoid_midpoint_fraction, 0.0f, 1.0f);
  out.needle_intra_year_late_decay_start_fraction = std::clamp(needle_intra_year_late_decay_start_fraction, 0.0f, 1.0f);
  out.needle_intra_year_late_decay_end_scale = std::clamp(needle_intra_year_late_decay_end_scale, 0.0f, 2.0f);
  out.needle_fascicular_start_year = std::clamp(needle_fascicular_start_year, 0, 16);
  out.needle_year2plus_length_multiplier = std::max(0.0f, needle_year2plus_length_multiplier);
  out.needle_year2plus_width_multiplier = std::max(0.0f, needle_year2plus_width_multiplier);
  out.needle_year2plus_thickness_multiplier = std::max(0.0f, needle_year2plus_thickness_multiplier);
  out.needle_lignification_factor_year1 = std::clamp(needle_lignification_factor_year1, 0.0f, 2.0f);
  out.needle_lignification_factor_year2plus = std::clamp(needle_lignification_factor_year2plus, 0.0f, 2.0f);
  out.needle_stomatal_strip_density_year1 = std::clamp(needle_stomatal_strip_density_year1, 0.0f, 1.0f);
  out.needle_stomatal_strip_density_year2plus = std::clamp(needle_stomatal_strip_density_year2plus, 0.0f, 1.0f);
  out.needle_basal_taper_ratio_year1 = std::clamp(needle_basal_taper_ratio_year1, 0.6f, 1.2f);
  out.needle_basal_taper_ratio_year2plus = std::clamp(needle_basal_taper_ratio_year2plus, 0.6f, 1.2f);
  out.needle_fascicle_sheath_budget_years = std::max(0.0f, needle_fascicle_sheath_budget_gdd / kPineGddPerYear);
  out.needle_specularity_plasticity_year1 = std::clamp(needle_specularity_plasticity_year1, 0.0f, 1.0f);
  out.needle_specularity_plasticity_year2plus = std::clamp(needle_specularity_plasticity_year2plus, 0.0f, 1.0f);
  out.needle_bud_storage_vigor_strength = std::clamp(needle_bud_storage_vigor_strength, 0.0f, 1.0f);
  out.needle_bud_storage_completion_floor = std::clamp(needle_bud_storage_completion_floor, 0.0f, 1.0f);
  // Per-plant copy of the user-exposed needle radius cap. Bounded to a wide
  // but sane envelope: 0 disables the cap entirely (uncapped needles), and 4x
  // is well above what any realistic Scots pine needle would need relative
  // to its parent shoot.
  out.needle_radius_to_stem_thickness_max_ratio = std::clamp(needle_radius_to_stem_thickness_max_ratio, 0.0f, 4.0f);

  out.gravitropism_first_order = SampleDistribution(gravitropism_first_order, rng);

  // Dynamic tropism array (mirrors MaizeTasselDescriptor logic).
  for (const auto& entry : tropisms) {
    const float usage_chance = std::clamp(entry.usage_chance_percent, 0.0f, 100.0f);
    if (SampleUnit01(rng) * 100.0f > usage_chance) {
      continue;
    }
    SampledTropism st;
    const float dx = SampleDistribution(entry.direction_x, rng);
    const float dy = SampleDistribution(entry.direction_y, rng);
    const float dz = SampleDistribution(entry.direction_z, rng);
    const glm::vec3 dir(dx, dy, dz);
    const float len = glm::length(dir);
    st.direction = (len > 0.001f) ? dir / len : glm::vec3(0.0f, -1.0f, 0.0f);
    st.strength = SampleDistribution(entry.strength, rng);
    st.order_response = entry.order_response;
    out.tropisms.push_back(std::move(st));
  }

  // Per-shoot stochastic noise.
  out.internode_length_per_node_cv = std::max(0.0f, SampleDistribution(internode_length_per_node_cv, rng));
  out.internode_thickness_per_node_cv = std::max(0.0f, SampleDistribution(internode_thickness_per_node_cv, rng));
  out.branch_angle_per_node_sigma_deg = std::max(0.0f, SampleDistribution(branch_angle_per_node_sigma_deg, rng));
  out.roll_phyllotaxis_per_node_sigma_deg =
      std::max(0.0f, SampleDistribution(roll_phyllotaxis_per_node_sigma_deg, rng));

  // Needle curvature.
  out.needle_curvature_adaxial_bias = SampleDistribution(needle_curvature_adaxial_bias, rng);
  out.needle_curvature_abaxial_bias = SampleDistribution(needle_curvature_abaxial_bias, rng);
  out.needle_curvature_gradient_per_arclen = SampleDistribution(needle_curvature_gradient_per_arclen, rng);
  out.needle_diameter_for_curvature_m =
      std::clamp(SampleDistribution(needle_diameter_for_curvature_m, rng), 0.0f, 0.005f);

  // Needle mechanics.
  out.needle_young_modulus_baseline_Pa = std::max(0.0f, SampleDistribution(needle_young_modulus_baseline_Pa, rng));
  out.needle_lignification_maturation_years =
      std::max(0.0f, SampleDistribution(needle_lignification_maturation_years, rng));
  // Keep width/thickness uncapped and independent. Only enforce
  // non-negativity so invalid negatives do not propagate.
  out.needle_cross_section_width_max_m = std::max(0.0f, SampleDistribution(needle_cross_section_width_max_m, rng));
  out.needle_cross_section_thickness_max_m =
      std::max(0.0f, SampleDistribution(needle_cross_section_thickness_max_m, rng));
  out.needle_density_kg_m3 = std::max(0.0f, SampleDistribution(needle_density_kg_m3, rng));
  out.gravity_m_s2 = std::max(0.0f, SampleDistribution(gravity_m_s2, rng));
  out.needle_per_needle_length_cv = std::max(0.0f, SampleDistribution(needle_per_needle_length_cv, rng));
  out.needle_per_needle_curvature_cv = std::max(0.0f, SampleDistribution(needle_per_needle_curvature_cv, rng));
  out.needle_per_needle_radius_cv = std::max(0.0f, SampleDistribution(needle_per_needle_radius_cv, rng));
  out.needle_per_needle_modulus_cv = std::max(0.0f, SampleDistribution(needle_per_needle_modulus_cv, rng));
  out.needle_per_needle_density_cv = std::max(0.0f, SampleDistribution(needle_per_needle_density_cv, rng));
  out.needle_sinusoidal_amplitude_deg = std::max(0.0f, SampleDistribution(needle_sinusoidal_amplitude_deg, rng));
  out.needle_sinusoidal_frequency_cycles = std::max(0.0f, SampleDistribution(needle_sinusoidal_frequency_cycles, rng));
  out.needle_sinusoidal_phase_randomness_deg =
      std::max(0.0f, SampleDistribution(needle_sinusoidal_phase_randomness_deg, rng));
  out.needle_per_needle_wave_amplitude_cv =
      std::max(0.0f, SampleDistribution(needle_per_needle_wave_amplitude_cv, rng));
  out.needle_per_needle_wave_frequency_cv =
      std::max(0.0f, SampleDistribution(needle_per_needle_wave_frequency_cv, rng));
  out.needle_per_needle_wave_phase_cv = std::max(0.0f, SampleDistribution(needle_per_needle_wave_phase_cv, rng));

  // Plant-wide random initial orientation around +Y.
  out.initial_orientation_yaw_deg = SampleDistribution(initial_orientation_yaw_deg, rng);

  // ---- Per-emission distributions -----------------------------------------
  // Copy each distribution verbatim so production rules can resample at
  // every consumption site (per phytomer / per whorl / per needle cluster).
  // No additional RNG draws here; the rule call sites will draw fresh from
  // their own per-node RNGs (see `MakeNodeRng` in LSystemRuleHelpers.hpp).
  out.distributions.max_branching_order = max_branching_order;
  out.distributions.plastochron_gdd = plastochron_gdd;
  out.distributions.max_phytomers_per_seasonal_growth = max_phytomers_per_seasonal_growth;
  out.distributions.branches_per_whorl = branches_per_whorl;
  out.distributions.whorl_dormancy_years = whorl_dormancy_years;
  out.distributions.branch_insertion_angle_deg = branch_insertion_angle_deg;
  out.distributions.branch_roll_phyllotaxis_deg = branch_roll_phyllotaxis_deg;
  out.distributions.internode_length_m = internode_length_m;
  out.distributions.leader_internode_thickness_m = leader_internode_thickness_m;
  out.distributions.lateral_length_ratio = lateral_length_ratio;
  out.distributions.lateral_thickness_ratio = lateral_thickness_ratio;
  out.distributions.internode_length_maturity_curve = internode_length_maturity_curve;
  out.distributions.internode_width_maturity_curve = internode_width_maturity_curve;
  out.distributions.bare_zone_fraction = bare_zone_fraction;
  out.distributions.needle_count_per_cluster = needle_count_per_cluster;
  out.distributions.needle_length_m = needle_length_m;
  out.distributions.needle_length_maturity_curve = needle_length_maturity_curve;
  out.distributions.needle_cross_section_width_max_m = needle_cross_section_width_max_m;
  out.distributions.needle_cross_section_thickness_max_m = needle_cross_section_thickness_max_m;
  out.distributions.needle_cross_section_width_profile = needle_cross_section_width_profile;
  out.distributions.needle_cross_section_thickness_profile = needle_cross_section_thickness_profile;
  out.distributions.needle_cross_section_temporal_maturity_curve = needle_cross_section_temporal_maturity_curve;
  out.distributions.needle_lifespan_years = needle_lifespan_years;
  out.distributions.needle_browning_years = needle_browning_years;
  out.distributions.needle_flush_delay_gdd = needle_flush_delay_gdd;
  out.distributions.internode_maturation_gdd = internode_maturation_gdd;
  out.distributions.needle_maturation_gdd = needle_maturation_gdd;
  out.distributions.needle_branching_angle_deg = needle_branching_angle_deg;
  out.distributions.needle_branching_relax_gdd = needle_branching_relax_gdd;
  out.distributions.needle_curvature_adaxial_bias = needle_curvature_adaxial_bias;
  out.distributions.needle_curvature_abaxial_bias = needle_curvature_abaxial_bias;
  out.distributions.needle_curvature_gradient_per_arclen = needle_curvature_gradient_per_arclen;
  out.distributions.needle_diameter_for_curvature_m = needle_diameter_for_curvature_m;
  out.distributions.needle_sinusoidal_amplitude_deg = needle_sinusoidal_amplitude_deg;
  out.distributions.needle_sinusoidal_frequency_cycles = needle_sinusoidal_frequency_cycles;
  out.distributions.needle_sinusoidal_phase_randomness_deg = needle_sinusoidal_phase_randomness_deg;
  out.distributions.needle_young_modulus_baseline_Pa = needle_young_modulus_baseline_Pa;
  out.distributions.needle_lignification_maturation_years = needle_lignification_maturation_years;
  out.distributions.needle_density_kg_m3 = needle_density_kg_m3;
  out.distributions.needle_per_needle_length_cv = needle_per_needle_length_cv;
  out.distributions.needle_per_needle_curvature_cv = needle_per_needle_curvature_cv;
  out.distributions.needle_per_needle_radius_cv = needle_per_needle_radius_cv;
  out.distributions.needle_per_needle_modulus_cv = needle_per_needle_modulus_cv;
  out.distributions.needle_per_needle_density_cv = needle_per_needle_density_cv;
  out.distributions.needle_per_needle_wave_amplitude_cv = needle_per_needle_wave_amplitude_cv;
  out.distributions.needle_per_needle_wave_frequency_cv = needle_per_needle_wave_frequency_cv;
  out.distributions.needle_per_needle_wave_phase_cv = needle_per_needle_wave_phase_cv;
  out.distributions.gravitropism_first_order = gravitropism_first_order;
  out.distributions.internode_length_per_node_cv = internode_length_per_node_cv;
  out.distributions.internode_thickness_per_node_cv = internode_thickness_per_node_cv;
  out.distributions.branch_angle_per_node_sigma_deg = branch_angle_per_node_sigma_deg;
  out.distributions.roll_phyllotaxis_per_node_sigma_deg = roll_phyllotaxis_per_node_sigma_deg;

  return out;
}

// ===========================================================================
// Instantiate - create entity with ScotsPine private component
// ===========================================================================
Entity ScotsPineDescriptor::Instantiate() const {
  const auto scene = GetApplication().GetActiveScene();
  if (!scene)
    return {};

  const auto entity = scene->CreateEntity(GetTitle());
  const auto pine = scene->GetOrSetPrivateComponent<ScotsPine>(entity).lock();
  pine->descriptor_ref = GetSelf();
  pine->target_gdd = SampleTargetGdd(pine->seed);
  pine->GenerateGeometryEntities();

  return entity;
}

// ===========================================================================
// Inspector UI
// ===========================================================================

// ===========================================================================
// Serialize
// ===========================================================================
void l_system_package::SerializeScotsPineDescriptor(YAML::Emitter& out, const ScotsPineDescriptor& target) {
  // Phytomer scheduling.
  target.max_branching_order.Save("max_branching_order", out);
  target.plastochron_gdd.Save("plastochron_gdd", out);
  target.max_phytomers_per_seasonal_growth.Save("max_phytomers_per_seasonal_growth", out);

  // Whorl architecture.
  target.branches_per_whorl.Save("branches_per_whorl", out);
  target.whorl_dormancy_years.Save("whorl_dormancy_years", out);
  target.branch_insertion_angle_deg.Save("branch_insertion_angle_deg", out);
  target.branch_roll_phyllotaxis_deg.Save("branch_roll_phyllotaxis_deg", out);

  // Phytomer dimensions.
  target.internode_length_m.Save("internode_length_m", out);
  target.leader_internode_thickness_m.Save("leader_internode_thickness_m", out);
  // Alias for discoverability in .spine files.
  target.leader_internode_thickness_m.Save("main_stem_width_m", out);
  target.lateral_length_ratio.Save("lateral_length_ratio", out);
  target.lateral_thickness_ratio.Save("lateral_thickness_ratio", out);
  out << YAML::Key << "main_stem_color_rgba" << YAML::Value << target.main_stem_color_rgba;
  out << YAML::Key << "main_stem_old_color_rgba" << YAML::Value << target.main_stem_old_color_rgba;
  out << YAML::Key << "internode_age_exponent" << YAML::Value << target.internode_age_exponent;

  // Needles.
  target.bare_zone_fraction.Save("bare_zone_fraction", out);
  target.needle_count_per_cluster.Save("needle_count_per_cluster", out);
  out << YAML::Key << "needle_segment_count" << YAML::Value << target.needle_segment_count;
  target.needle_length_m.Save("needle_length_m", out);
  target.needle_lifespan_years.Save("needle_lifespan_years", out);
  target.needle_browning_years.Save("needle_browning_years", out);
  target.needle_flush_delay_gdd.Save("needle_flush_delay_gdd", out);
  target.internode_maturation_gdd.Save("internode_maturation_gdd", out);
  target.needle_maturation_gdd.Save("needle_maturation_gdd", out);
  target.needle_branching_angle_deg.Save("needle_branching_angle_deg", out);
  target.needle_branching_relax_gdd.Save("needle_branching_relax_gdd", out);
  target.internode_length_maturity_curve.Save("internode_length_maturity_curve", out);
  target.internode_width_maturity_curve.Save("internode_width_maturity_curve", out);
  target.needle_length_maturity_curve.Save("needle_length_maturity_curve", out);
  target.needle_cross_section_width_max_m.Save("needle_cross_section_width_max_m", out);
  target.needle_cross_section_thickness_max_m.Save("needle_cross_section_thickness_max_m", out);
  target.needle_cross_section_width_profile.Save("needle_cross_section_width_profile", out);
  target.needle_cross_section_thickness_profile.Save("needle_cross_section_thickness_profile", out);
  target.needle_cross_section_temporal_maturity_curve.Save("needle_cross_section_temporal_maturity_curve", out);
  out << YAML::Key << "needle_order_length_attenuation" << YAML::Value << target.needle_order_length_attenuation;
  out << YAML::Key << "needle_order_radius_attenuation" << YAML::Value << target.needle_order_radius_attenuation;
  out << YAML::Key << "needle_order_min_length_scale" << YAML::Value << target.needle_order_min_length_scale;
  out << YAML::Key << "needle_order_min_radius_scale" << YAML::Value << target.needle_order_min_radius_scale;
  out << YAML::Key << "needle_intra_year_base_ratio" << YAML::Value << target.needle_intra_year_base_ratio;
  out << YAML::Key << "needle_intra_year_sigmoid_steepness" << YAML::Value
      << target.needle_intra_year_sigmoid_steepness;
  out << YAML::Key << "needle_intra_year_sigmoid_midpoint_fraction" << YAML::Value
      << target.needle_intra_year_sigmoid_midpoint_fraction;
  out << YAML::Key << "needle_intra_year_late_decay_start_fraction" << YAML::Value
      << target.needle_intra_year_late_decay_start_fraction;
  out << YAML::Key << "needle_intra_year_late_decay_end_scale" << YAML::Value
      << target.needle_intra_year_late_decay_end_scale;
  out << YAML::Key << "needle_fascicular_start_year" << YAML::Value << target.needle_fascicular_start_year;
  out << YAML::Key << "needle_year2plus_length_multiplier" << YAML::Value << target.needle_year2plus_length_multiplier;
  out << YAML::Key << "needle_year2plus_width_multiplier" << YAML::Value << target.needle_year2plus_width_multiplier;
  out << YAML::Key << "needle_year2plus_thickness_multiplier" << YAML::Value
      << target.needle_year2plus_thickness_multiplier;
  out << YAML::Key << "needle_lignification_factor_year1" << YAML::Value << target.needle_lignification_factor_year1;
  out << YAML::Key << "needle_lignification_factor_year2plus" << YAML::Value
      << target.needle_lignification_factor_year2plus;
  out << YAML::Key << "needle_stomatal_strip_density_year1" << YAML::Value
      << target.needle_stomatal_strip_density_year1;
  out << YAML::Key << "needle_stomatal_strip_density_year2plus" << YAML::Value
      << target.needle_stomatal_strip_density_year2plus;
  out << YAML::Key << "needle_basal_taper_ratio_year1" << YAML::Value << target.needle_basal_taper_ratio_year1;
  out << YAML::Key << "needle_basal_taper_ratio_year2plus" << YAML::Value << target.needle_basal_taper_ratio_year2plus;
  out << YAML::Key << "needle_fascicle_sheath_budget_gdd" << YAML::Value << target.needle_fascicle_sheath_budget_gdd;
  out << YAML::Key << "needle_specularity_plasticity_year1" << YAML::Value
      << target.needle_specularity_plasticity_year1;
  out << YAML::Key << "needle_specularity_plasticity_year2plus" << YAML::Value
      << target.needle_specularity_plasticity_year2plus;
  out << YAML::Key << "needle_bud_storage_vigor_strength" << YAML::Value << target.needle_bud_storage_vigor_strength;
  out << YAML::Key << "needle_bud_storage_completion_floor" << YAML::Value
      << target.needle_bud_storage_completion_floor;
  out << YAML::Key << "needle_radius_to_stem_thickness_max_ratio" << YAML::Value
      << target.needle_radius_to_stem_thickness_max_ratio;
  out << YAML::Key << "needle_color_rgba" << YAML::Value << target.needle_color_rgba;
  out << YAML::Key << "needle_old_color_rgba" << YAML::Value << target.needle_old_color_rgba;
  out << YAML::Key << "needle_axial_age_span" << YAML::Value << target.needle_axial_age_span;
  out << YAML::Key << "needle_axial_age_exponent" << YAML::Value << target.needle_axial_age_exponent;

  // Needle curvature.
  target.needle_curvature_adaxial_bias.Save("needle_curvature_adaxial_bias", out);
  target.needle_curvature_abaxial_bias.Save("needle_curvature_abaxial_bias", out);
  target.needle_curvature_gradient_per_arclen.Save("needle_curvature_gradient_per_arclen", out);
  target.needle_diameter_for_curvature_m.Save("needle_diameter_for_curvature_m", out);
  target.needle_sinusoidal_amplitude_deg.Save("needle_sinusoidal_amplitude_deg", out);
  target.needle_sinusoidal_frequency_cycles.Save("needle_sinusoidal_frequency_cycles", out);
  target.needle_sinusoidal_phase_randomness_deg.Save("needle_sinusoidal_phase_randomness_deg", out);

  // Needle mechanics.
  target.needle_young_modulus_baseline_Pa.Save("needle_young_modulus_baseline_Pa", out);
  target.needle_lignification_maturation_years.Save("needle_lignification_maturation_years", out);
  target.needle_density_kg_m3.Save("needle_density_kg_m3", out);
  target.gravity_m_s2.Save("gravity_m_s2", out);
  target.needle_per_needle_length_cv.Save("needle_per_needle_length_cv", out);
  target.needle_per_needle_curvature_cv.Save("needle_per_needle_curvature_cv", out);
  target.needle_per_needle_radius_cv.Save("needle_per_needle_radius_cv", out);
  target.needle_per_needle_modulus_cv.Save("needle_per_needle_modulus_cv", out);
  target.needle_per_needle_density_cv.Save("needle_per_needle_density_cv", out);
  target.needle_per_needle_wave_amplitude_cv.Save("needle_per_needle_wave_amplitude_cv", out);
  target.needle_per_needle_wave_frequency_cv.Save("needle_per_needle_wave_frequency_cv", out);
  target.needle_per_needle_wave_phase_cv.Save("needle_per_needle_wave_phase_cv", out);

  // Tropism.
  target.gravitropism_first_order.Save("gravitropism_first_order", out);
  target.initial_orientation_yaw_deg.Save("initial_orientation_yaw_deg", out);

  // Per-instance target.
  target.target_gdd.Save("target_gdd", out);
  target.gdd_per_day.Save("gdd_per_day", out);
  target.growing_season_start_day.Save("growing_season_start_day", out);
  target.growing_season_end_day.Save("growing_season_end_day", out);

  // Per-shoot stochastic noise.
  target.internode_length_per_node_cv.Save("internode_length_per_node_cv", out);
  target.internode_thickness_per_node_cv.Save("internode_thickness_per_node_cv", out);
  target.branch_angle_per_node_sigma_deg.Save("branch_angle_per_node_sigma_deg", out);
  target.roll_phyllotaxis_per_node_sigma_deg.Save("roll_phyllotaxis_per_node_sigma_deg", out);

  // Editor preferences.
  out << YAML::Key << "live_preview" << YAML::Value << target.live_preview;
  out << YAML::Key << "live_preview_rate_hz" << YAML::Value << target.live_preview_rate_hz;
  out << YAML::Key << "live_preview_representative_only" << YAML::Value << target.live_preview_representative_only;
  out << YAML::Key << "live_preview_cap_target_gdd" << YAML::Value << target.live_preview_cap_target_gdd;
  out << YAML::Key << "live_preview_max_gdd" << YAML::Value << target.live_preview_max_gdd;
  out << YAML::Key << "live_preview_max_growth_steps" << YAML::Value << target.live_preview_max_growth_steps;
  out << YAML::Key << "grid_rows" << YAML::Value << target.grid_rows;
  out << YAML::Key << "grid_cols" << YAML::Value << target.grid_cols;
  out << YAML::Key << "grid_spacing" << YAML::Value << target.grid_spacing;
  out << YAML::Key << "triangle_side_length" << YAML::Value << target.triangle_side_length;

  // Tropism array.
  out << YAML::Key << "tropism_count" << YAML::Value << static_cast<int>(target.tropisms.size());
  for (size_t i = 0; i < target.tropisms.size(); ++i) {
    const std::string prefix = "tropism_" + std::to_string(i) + "_";
    const auto& entry = target.tropisms[i];
    entry.direction_x.Save(prefix + "dir_x", out);
    entry.direction_y.Save(prefix + "dir_y", out);
    entry.direction_z.Save(prefix + "dir_z", out);
    entry.strength.Save(prefix + "strength", out);
    out << YAML::Key << (prefix + "usage_chance_percent") << YAML::Value
        << std::clamp(entry.usage_chance_percent, 0.0f, 100.0f);
    entry.order_response.Save(prefix + "order_response", out);
  }
}

// ===========================================================================
// Deserialize
//
// Backward-compat policy:
// - Legacy base/tip radius and simple taper keys are mapped into the new
//   ellipsoid cross-section width/thickness controls when new keys are absent.
// - Other older experimental keys remain ignored.
// Missing keys retain inline member defaults.
// ===========================================================================
void l_system_package::DeserializeScotsPineDescriptor(const YAML::Node& in, ScotsPineDescriptor& target) {
  ConfigurePineMaturityDefaults(target);

  LoadSingleDistributionWithScalarFallback(in, "max_branching_order", target.max_branching_order);
  LoadSingleDistributionWithScalarFallback(in, "plastochron_gdd", target.plastochron_gdd);
  LoadSingleDistributionWithScalarFallback(in, "max_phytomers_per_seasonal_growth",
                                           target.max_phytomers_per_seasonal_growth);

  LoadSingleDistributionWithScalarFallback(in, "branches_per_whorl", target.branches_per_whorl);
  // Clock-rule fix: prefer years key; fall back to legacy GDD key with /1500.
  if (in["whorl_dormancy_years"]) {
    LoadSingleDistributionWithScalarFallback(in, "whorl_dormancy_years", target.whorl_dormancy_years);
  } else {
    LoadLegacyGddDistributionAsYears(in, "whorl_dormancy_gdd", target.whorl_dormancy_years);
  }
  LoadSingleDistributionWithScalarFallback(in, "branch_insertion_angle_deg", target.branch_insertion_angle_deg);
  LoadSingleDistributionWithScalarFallback(in, "branch_roll_phyllotaxis_deg", target.branch_roll_phyllotaxis_deg);

  LoadSingleDistributionWithScalarFallback(in, "internode_length_m", target.internode_length_m);
  LoadSingleDistributionWithScalarFallback(in, "leader_internode_thickness_m", target.leader_internode_thickness_m);
  // Backward/forward alias support.
  LoadSingleDistributionWithScalarFallback(in, "main_stem_width_m", target.leader_internode_thickness_m);
  LoadSingleDistributionWithScalarFallback(in, "lateral_length_ratio", target.lateral_length_ratio);
  LoadSingleDistributionWithScalarFallback(in, "lateral_thickness_ratio", target.lateral_thickness_ratio);
  if (in["main_stem_color_rgba"]) {
    target.main_stem_color_rgba = in["main_stem_color_rgba"].as<glm::vec4>();
  }
  if (in["main_stem_old_color_rgba"]) {
    target.main_stem_old_color_rgba = in["main_stem_old_color_rgba"].as<glm::vec4>();
  }
  if (in["internode_age_exponent"]) {
    target.internode_age_exponent = std::clamp(in["internode_age_exponent"].as<float>(), 0.1f, 4.0f);
  }

  LoadSingleDistributionWithScalarFallback(in, "bare_zone_fraction", target.bare_zone_fraction);
  LoadSingleDistributionWithScalarFallback(in, "needle_count_per_cluster", target.needle_count_per_cluster);
  if (in["needle_segment_count"]) {
    target.needle_segment_count = std::clamp(in["needle_segment_count"].as<int>(), 3, 128);
  }
  LoadSingleDistributionWithScalarFallback(in, "needle_length_m", target.needle_length_m);
  // Clock-rule fix: prefer years keys; fall back to legacy GDD keys with /1500.
  if (in["needle_lifespan_years"]) {
    LoadSingleDistributionWithScalarFallback(in, "needle_lifespan_years", target.needle_lifespan_years);
  } else {
    LoadLegacyGddDistributionAsYears(in, "needle_lifespan_gdd", target.needle_lifespan_years);
  }
  if (in["needle_browning_years"]) {
    LoadSingleDistributionWithScalarFallback(in, "needle_browning_years", target.needle_browning_years);
  } else {
    LoadLegacyGddDistributionAsYears(in, "needle_browning_gdd", target.needle_browning_years);
  }
  LoadSingleDistributionWithScalarFallback(in, "needle_flush_delay_gdd", target.needle_flush_delay_gdd);
  LoadSingleDistributionWithScalarFallback(in, "internode_maturation_gdd", target.internode_maturation_gdd);
  LoadSingleDistributionWithScalarFallback(in, "needle_maturation_gdd", target.needle_maturation_gdd);
  LoadSingleDistributionWithScalarFallback(in, "needle_branching_angle_deg", target.needle_branching_angle_deg);
  LoadSingleDistributionWithScalarFallback(in, "needle_branching_relax_gdd", target.needle_branching_relax_gdd);
  const bool has_new_cross_section_width_max = static_cast<bool>(in["needle_cross_section_width_max_m"]);
  const bool has_new_cross_section_thickness_max = static_cast<bool>(in["needle_cross_section_thickness_max_m"]);
  const bool has_new_cross_section_width_profile = static_cast<bool>(in["needle_cross_section_width_profile"]);
  const bool has_new_cross_section_thickness_profile = static_cast<bool>(in["needle_cross_section_thickness_profile"]);
  target.internode_length_maturity_curve.Load("internode_length_maturity_curve", in);
  target.internode_width_maturity_curve.Load("internode_width_maturity_curve", in);
  target.needle_length_maturity_curve.Load("needle_length_maturity_curve", in);
  LoadSingleDistributionWithScalarFallback(in, "needle_cross_section_width_max_m",
                                           target.needle_cross_section_width_max_m);
  LoadSingleDistributionWithScalarFallback(in, "needle_cross_section_thickness_max_m",
                                           target.needle_cross_section_thickness_max_m);
  target.needle_cross_section_width_profile.Load("needle_cross_section_width_profile", in);
  target.needle_cross_section_thickness_profile.Load("needle_cross_section_thickness_profile", in);
  target.needle_cross_section_temporal_maturity_curve.Load("needle_cross_section_temporal_maturity_curve", in);

  evo_engine::SingleDistribution<float> legacy_base_radius_m{0.0f};
  evo_engine::SingleDistribution<float> legacy_tip_radius_m{0.0f};
  const bool has_legacy_base_radius = static_cast<bool>(in["needle_base_radius_m"]);
  const bool has_legacy_tip_radius = static_cast<bool>(in["needle_tip_radius_m"]);
  if (has_legacy_base_radius) {
    LoadSingleDistributionWithScalarFallback(in, "needle_base_radius_m", legacy_base_radius_m);
  }
  if (has_legacy_tip_radius) {
    LoadSingleDistributionWithScalarFallback(in, "needle_tip_radius_m", legacy_tip_radius_m);
  }

  if ((!has_new_cross_section_width_max || !has_new_cross_section_thickness_max) &&
      (has_legacy_base_radius || has_legacy_tip_radius)) {
    const float legacy_width_max_mean_m = std::max(0.0f, legacy_base_radius_m.mean) * 2.0f;
    const float legacy_width_max_dev_m = std::max(0.0f, legacy_base_radius_m.deviation) * 2.0f;
    const float legacy_tip_diameter_mean_m = std::max(0.0f, legacy_tip_radius_m.mean) * 2.0f;
    const float legacy_tip_diameter_dev_m = std::max(0.0f, legacy_tip_radius_m.deviation) * 2.0f;
    if (!has_new_cross_section_width_max) {
      target.needle_cross_section_width_max_m.mean = legacy_width_max_mean_m;
      target.needle_cross_section_width_max_m.deviation = legacy_width_max_dev_m;
    }
    if (!has_new_cross_section_thickness_max) {
      const float fallback_thickness_mean_m =
          (legacy_tip_diameter_mean_m > 0.0f) ? legacy_tip_diameter_mean_m : legacy_width_max_mean_m;
      const float fallback_thickness_dev_m =
          (legacy_tip_diameter_dev_m > 0.0f) ? legacy_tip_diameter_dev_m : legacy_width_max_dev_m;
      target.needle_cross_section_thickness_max_m.mean = fallback_thickness_mean_m;
      target.needle_cross_section_thickness_max_m.deviation = fallback_thickness_dev_m;
    }
  }

  float legacy_tip_taper_ratio = 0.36f;
  if (in["needle_simple_tip_taper_ratio"]) {
    legacy_tip_taper_ratio = std::clamp(in["needle_simple_tip_taper_ratio"].as<float>(), 0.0f, 1.0f);
  }
  if (!has_new_cross_section_width_profile) {
    ConfigureNeedleCrossSectionProfileDefaults(target.needle_cross_section_width_profile, 1.0f, legacy_tip_taper_ratio);
  }
  if (!has_new_cross_section_thickness_profile) {
    ConfigureNeedleCrossSectionProfileDefaults(target.needle_cross_section_thickness_profile, 1.0f,
                                               legacy_tip_taper_ratio);
  }

  target.needle_cross_section_width_max_m.mean = std::max(0.0f, target.needle_cross_section_width_max_m.mean);
  target.needle_cross_section_width_max_m.deviation = std::max(0.0f, target.needle_cross_section_width_max_m.deviation);
  target.needle_cross_section_thickness_max_m.mean = std::max(0.0f, target.needle_cross_section_thickness_max_m.mean);
  target.needle_cross_section_thickness_max_m.deviation =
      std::max(0.0f, target.needle_cross_section_thickness_max_m.deviation);
  auto clamp_plot_range = [](evo_engine::Plot2D<float>& plot, const float max_value) {
    plot.min_value = std::clamp(plot.min_value, 0.0f, 1.0f);
    plot.max_value = std::clamp(plot.max_value, 0.0f, max_value);
    if (plot.max_value < plot.min_value) {
      std::swap(plot.min_value, plot.max_value);
    }
  };
  clamp_plot_range(target.needle_cross_section_width_profile.mean, 4.0f);
  clamp_plot_range(target.needle_cross_section_width_profile.deviation, 4.0f);
  clamp_plot_range(target.needle_cross_section_thickness_profile.mean, 4.0f);
  clamp_plot_range(target.needle_cross_section_thickness_profile.deviation, 4.0f);
  clamp_plot_range(target.needle_cross_section_temporal_maturity_curve.mean, 1.0f);
  clamp_plot_range(target.needle_cross_section_temporal_maturity_curve.deviation, 1.0f);
  if (in["needle_order_length_attenuation"]) {
    target.needle_order_length_attenuation = std::clamp(in["needle_order_length_attenuation"].as<float>(), 0.0f, 1.0f);
  }
  if (in["needle_order_radius_attenuation"]) {
    target.needle_order_radius_attenuation = std::clamp(in["needle_order_radius_attenuation"].as<float>(), 0.0f, 1.0f);
  }
  if (in["needle_order_min_length_scale"]) {
    target.needle_order_min_length_scale = std::clamp(in["needle_order_min_length_scale"].as<float>(), 0.10f, 1.00f);
  }
  if (in["needle_order_min_radius_scale"]) {
    target.needle_order_min_radius_scale = std::clamp(in["needle_order_min_radius_scale"].as<float>(), 0.10f, 1.00f);
  }
  if (in["needle_intra_year_base_ratio"]) {
    target.needle_intra_year_base_ratio = std::clamp(in["needle_intra_year_base_ratio"].as<float>(), 0.0f, 1.0f);
  }
  if (in["needle_intra_year_sigmoid_steepness"]) {
    target.needle_intra_year_sigmoid_steepness = std::max(0.01f, in["needle_intra_year_sigmoid_steepness"].as<float>());
  }
  if (in["needle_intra_year_sigmoid_midpoint_fraction"]) {
    target.needle_intra_year_sigmoid_midpoint_fraction =
        std::clamp(in["needle_intra_year_sigmoid_midpoint_fraction"].as<float>(), 0.0f, 1.0f);
  }
  if (in["needle_intra_year_late_decay_start_fraction"]) {
    target.needle_intra_year_late_decay_start_fraction =
        std::clamp(in["needle_intra_year_late_decay_start_fraction"].as<float>(), 0.0f, 1.0f);
  }
  if (in["needle_intra_year_late_decay_end_scale"]) {
    target.needle_intra_year_late_decay_end_scale =
        std::clamp(in["needle_intra_year_late_decay_end_scale"].as<float>(), 0.0f, 2.0f);
  }
  if (in["needle_fascicular_start_year"]) {
    target.needle_fascicular_start_year = std::clamp(in["needle_fascicular_start_year"].as<int>(), 0, 16);
  }
  if (in["needle_year2plus_length_multiplier"]) {
    target.needle_year2plus_length_multiplier = std::max(0.0f, in["needle_year2plus_length_multiplier"].as<float>());
  }
  if (in["needle_year2plus_width_multiplier"]) {
    target.needle_year2plus_width_multiplier = std::max(0.0f, in["needle_year2plus_width_multiplier"].as<float>());
  }
  if (in["needle_year2plus_thickness_multiplier"]) {
    target.needle_year2plus_thickness_multiplier =
        std::max(0.0f, in["needle_year2plus_thickness_multiplier"].as<float>());
  }
  if (in["needle_lignification_factor_year1"]) {
    target.needle_lignification_factor_year1 =
        std::clamp(in["needle_lignification_factor_year1"].as<float>(), 0.0f, 2.0f);
  }
  if (in["needle_lignification_factor_year2plus"]) {
    target.needle_lignification_factor_year2plus =
        std::clamp(in["needle_lignification_factor_year2plus"].as<float>(), 0.0f, 2.0f);
  }
  if (in["needle_stomatal_strip_density_year1"]) {
    target.needle_stomatal_strip_density_year1 =
        std::clamp(in["needle_stomatal_strip_density_year1"].as<float>(), 0.0f, 1.0f);
  }
  if (in["needle_stomatal_strip_density_year2plus"]) {
    target.needle_stomatal_strip_density_year2plus =
        std::clamp(in["needle_stomatal_strip_density_year2plus"].as<float>(), 0.0f, 1.0f);
  }
  if (in["needle_basal_taper_ratio_year1"]) {
    target.needle_basal_taper_ratio_year1 = std::clamp(in["needle_basal_taper_ratio_year1"].as<float>(), 0.6f, 1.2f);
  }
  if (in["needle_basal_taper_ratio_year2plus"]) {
    target.needle_basal_taper_ratio_year2plus =
        std::clamp(in["needle_basal_taper_ratio_year2plus"].as<float>(), 0.6f, 1.2f);
  }
  if (in["needle_fascicle_sheath_budget_gdd"]) {
    target.needle_fascicle_sheath_budget_gdd = std::max(0.0f, in["needle_fascicle_sheath_budget_gdd"].as<float>());
  }
  if (in["needle_specularity_plasticity_year1"]) {
    target.needle_specularity_plasticity_year1 =
        std::clamp(in["needle_specularity_plasticity_year1"].as<float>(), 0.0f, 1.0f);
  }
  if (in["needle_specularity_plasticity_year2plus"]) {
    target.needle_specularity_plasticity_year2plus =
        std::clamp(in["needle_specularity_plasticity_year2plus"].as<float>(), 0.0f, 1.0f);
  }
  if (in["needle_bud_storage_vigor_strength"]) {
    target.needle_bud_storage_vigor_strength =
        std::clamp(in["needle_bud_storage_vigor_strength"].as<float>(), 0.0f, 1.0f);
  }
  if (in["needle_bud_storage_completion_floor"]) {
    target.needle_bud_storage_completion_floor =
        std::clamp(in["needle_bud_storage_completion_floor"].as<float>(), 0.0f, 1.0f);
  }
  if (in["needle_radius_to_stem_thickness_max_ratio"]) {
    target.needle_radius_to_stem_thickness_max_ratio =
        std::clamp(in["needle_radius_to_stem_thickness_max_ratio"].as<float>(), 0.0f, 4.0f);
  }
  if (in["needle_color_rgba"]) {
    target.needle_color_rgba = in["needle_color_rgba"].as<glm::vec4>();
  }
  if (in["needle_old_color_rgba"]) {
    target.needle_old_color_rgba = in["needle_old_color_rgba"].as<glm::vec4>();
  }
  if (in["needle_axial_age_span"]) {
    target.needle_axial_age_span = std::clamp(in["needle_axial_age_span"].as<float>(), -1.0f, 1.0f);
  }
  if (in["needle_axial_age_exponent"]) {
    target.needle_axial_age_exponent = std::clamp(in["needle_axial_age_exponent"].as<float>(), 0.1f, 4.0f);
  }

  LoadSingleDistributionWithScalarFallback(in, "needle_curvature_adaxial_bias", target.needle_curvature_adaxial_bias);
  LoadSingleDistributionWithScalarFallback(in, "needle_curvature_abaxial_bias", target.needle_curvature_abaxial_bias);
  LoadSingleDistributionWithScalarFallback(in, "needle_curvature_gradient_per_arclen",
                                           target.needle_curvature_gradient_per_arclen);
  LoadSingleDistributionWithScalarFallback(in, "needle_diameter_for_curvature_m",
                                           target.needle_diameter_for_curvature_m);
  LoadSingleDistributionWithScalarFallback(in, "needle_sinusoidal_amplitude_deg",
                                           target.needle_sinusoidal_amplitude_deg);
  LoadSingleDistributionWithScalarFallback(in, "needle_sinusoidal_frequency_cycles",
                                           target.needle_sinusoidal_frequency_cycles);
  LoadSingleDistributionWithScalarFallback(in, "needle_sinusoidal_phase_randomness_deg",
                                           target.needle_sinusoidal_phase_randomness_deg);

  LoadSingleDistributionWithScalarFallback(in, "needle_young_modulus_baseline_Pa",
                                           target.needle_young_modulus_baseline_Pa);
  LoadSingleDistributionWithScalarFallback(in, "needle_lignification_maturation_years",
                                           target.needle_lignification_maturation_years);
  LoadSingleDistributionWithScalarFallback(in, "needle_density_kg_m3", target.needle_density_kg_m3);
  LoadSingleDistributionWithScalarFallback(in, "gravity_m_s2", target.gravity_m_s2);
  LoadSingleDistributionWithScalarFallback(in, "needle_per_needle_length_cv", target.needle_per_needle_length_cv);
  LoadSingleDistributionWithScalarFallback(in, "needle_per_needle_curvature_cv", target.needle_per_needle_curvature_cv);
  LoadSingleDistributionWithScalarFallback(in, "needle_per_needle_radius_cv", target.needle_per_needle_radius_cv);
  LoadSingleDistributionWithScalarFallback(in, "needle_per_needle_modulus_cv", target.needle_per_needle_modulus_cv);
  LoadSingleDistributionWithScalarFallback(in, "needle_per_needle_density_cv", target.needle_per_needle_density_cv);
  LoadSingleDistributionWithScalarFallback(in, "needle_per_needle_wave_amplitude_cv",
                                           target.needle_per_needle_wave_amplitude_cv);
  LoadSingleDistributionWithScalarFallback(in, "needle_per_needle_wave_frequency_cv",
                                           target.needle_per_needle_wave_frequency_cv);
  LoadSingleDistributionWithScalarFallback(in, "needle_per_needle_wave_phase_cv",
                                           target.needle_per_needle_wave_phase_cv);

  LoadSingleDistributionWithScalarFallback(in, "gravitropism_first_order", target.gravitropism_first_order);
  LoadSingleDistributionWithScalarFallback(in, "initial_orientation_yaw_deg", target.initial_orientation_yaw_deg);

  LoadSingleDistributionWithScalarFallback(in, "target_gdd", target.target_gdd);
  LoadSingleDistributionWithScalarFallback(in, "gdd_per_day", target.gdd_per_day);
  LoadSingleDistributionWithScalarFallback(in, "growing_season_start_day", target.growing_season_start_day);
  LoadSingleDistributionWithScalarFallback(in, "growing_season_end_day", target.growing_season_end_day);

  target.gdd_per_day.mean = std::max(0.0f, target.gdd_per_day.mean);
  target.gdd_per_day.deviation = std::max(0.0f, target.gdd_per_day.deviation);
  target.growing_season_start_day.mean = std::clamp(target.growing_season_start_day.mean, 0.0f, 365.0f);
  target.growing_season_start_day.deviation = std::max(0.0f, std::round(target.growing_season_start_day.deviation));
  target.growing_season_end_day.mean = std::clamp(target.growing_season_end_day.mean, 0.0f, 365.0f);
  target.growing_season_end_day.deviation = std::max(0.0f, std::round(target.growing_season_end_day.deviation));

  LoadSingleDistributionWithScalarFallback(in, "internode_length_per_node_cv", target.internode_length_per_node_cv);
  LoadSingleDistributionWithScalarFallback(in, "internode_thickness_per_node_cv",
                                           target.internode_thickness_per_node_cv);
  LoadSingleDistributionWithScalarFallback(in, "branch_angle_per_node_sigma_deg",
                                           target.branch_angle_per_node_sigma_deg);
  LoadSingleDistributionWithScalarFallback(in, "roll_phyllotaxis_per_node_sigma_deg",
                                           target.roll_phyllotaxis_per_node_sigma_deg);

  if (in["live_preview"])
    target.live_preview = in["live_preview"].as<bool>();
  if (in["live_preview_rate_hz"])
    target.live_preview_rate_hz = in["live_preview_rate_hz"].as<float>();
  if (in["live_preview_representative_only"])
    target.live_preview_representative_only = in["live_preview_representative_only"].as<bool>();
  if (in["live_preview_cap_target_gdd"])
    target.live_preview_cap_target_gdd = in["live_preview_cap_target_gdd"].as<bool>();
  if (in["live_preview_max_gdd"])
    target.live_preview_max_gdd = in["live_preview_max_gdd"].as<float>();
  if (in["live_preview_max_growth_steps"])
    target.live_preview_max_growth_steps = in["live_preview_max_growth_steps"].as<int>();
  if (in["grid_rows"])
    target.grid_rows = in["grid_rows"].as<int>();
  if (in["grid_cols"])
    target.grid_cols = in["grid_cols"].as<int>();
  if (in["grid_spacing"])
    target.grid_spacing = in["grid_spacing"].as<float>();
  if (in["triangle_side_length"])
    target.triangle_side_length = in["triangle_side_length"].as<float>();

  target.tropisms.clear();
  if (in["tropism_count"]) {
    const int count = std::max(0, in["tropism_count"].as<int>());
    target.tropisms.reserve(count);
    for (int i = 0; i < count; ++i) {
      const std::string prefix = "tropism_" + std::to_string(i) + "_";
      TropismEntry entry;
      LoadSingleDistributionWithScalarFallback(in, (prefix + "dir_x").c_str(), entry.direction_x);
      LoadSingleDistributionWithScalarFallback(in, (prefix + "dir_y").c_str(), entry.direction_y);
      LoadSingleDistributionWithScalarFallback(in, (prefix + "dir_z").c_str(), entry.direction_z);
      LoadSingleDistributionWithScalarFallback(in, (prefix + "strength").c_str(), entry.strength);
      const std::string usage_key = prefix + "usage_chance_percent";
      if (in[usage_key])
        entry.usage_chance_percent = in[usage_key].as<float>();
      entry.order_response.Load(prefix + "order_response", in);
      target.tropisms.emplace_back(std::move(entry));
    }
  }
}
