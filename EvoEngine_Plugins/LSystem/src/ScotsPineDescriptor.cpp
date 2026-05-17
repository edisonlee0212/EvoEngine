#include "ScotsPineDescriptor.hpp"
#include "LSystemDescriptorDefaults.hpp"
#include "ScotsPine.hpp"
#include <Application.hpp>
#include <EditorLayer.hpp>
#include <Scene.hpp>
#include <Transform.hpp>
#include <array>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <yaml-cpp/yaml.h>

using namespace l_system_plugin;
using namespace evo_engine;

// ===========================================================================
// File-local helpers (defaults file resolution, loading, target_gdd sampling).
// File extension: .spine
// ===========================================================================
namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr char kScotsPineDescriptorName[] = "ScotsPineDescriptor";

const std::array<std::filesystem::path, 6> kScotsPineResourceCandidates = {
  std::filesystem::path("./LSystemResources/Defaults/ScotsPineDescriptor_Default.spine"),
  std::filesystem::path("./EvoEngine_Plugins/LSystem/Internals/LSystemResources/Defaults/") /
    "ScotsPineDescriptor_Default.spine",
  std::filesystem::path("./Resources/DigitalAgricultureProject/Assets/New ScotsPineDescriptor.spine"),
  std::filesystem::path("./DigitalAgricultureProject/Assets/New ScotsPineDescriptor.spine"),
  std::filesystem::path("./04_EvoEngine/Resources/DigitalAgricultureProject/Assets/") /
    "New ScotsPineDescriptor.spine",
  std::filesystem::path("./04_EvoEngine/EvoEngine_Plugins/LSystem/Internals/") /
    "LSystemResources/Defaults/ScotsPineDescriptor_Default.spine"};

const std::array<std::filesystem::path, 2> kScotsPineProjectAssetCandidates = {
  std::filesystem::path("LSystem") / "New ScotsPineDescriptor.spine",
  "New ScotsPineDescriptor.spine"};

const std::array<std::filesystem::path, 2> kScotsPineWritableTemplateCandidates = {
  std::filesystem::path("./Resources/DigitalAgricultureProject/Assets/") /
    "New ScotsPineDescriptor.spine",
  std::filesystem::path("./04_EvoEngine/Resources/DigitalAgricultureProject/Assets/") /
    "New ScotsPineDescriptor.spine"};

const std::filesystem::path kScotsPineFallbackDefaultsPath =
  std::filesystem::path("./LSystemResources/Defaults/ScotsPineDescriptor_Default.spine");

void SetCurveToSinusoidalRange(evo_engine::Curve2D& curve,
                               const float y0,
                               const float y1,
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

void SetCurveLinear01(evo_engine::Curve2D& curve,
                      const float y0,
                      const float y1,
                      const int sample_count = 9) {
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

void ConfigureNeedleCrossSectionProfileDefaults(
    evo_engine::PlottedDistribution<float>& distribution,
    const float tip_multiplier,
    const float default_deviation_max = 0.0f) {
  distribution.mean.min_value = 0.0f;
  distribution.mean.max_value = 2.0f;
  SetCurveLinear01(distribution.mean.curve, 1.0f, tip_multiplier);

  distribution.deviation.min_value = 0.0f;
  distribution.deviation.max_value = std::max(0.0f, default_deviation_max);
  SetCurveLinear01(distribution.deviation.curve, 0.0f, 0.0f);
}

void ConfigureNeedleCrossSectionTemporalMaturityDefaults(
    evo_engine::PlottedDistribution<float>& distribution,
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
  ConfigureNeedleCrossSectionProfileDefaults(
      descriptor.needle_cross_section_width_profile, 0.36f);
  ConfigureNeedleCrossSectionProfileDefaults(
      descriptor.needle_cross_section_thickness_profile, 0.36f);
  ConfigureNeedleCrossSectionTemporalMaturityDefaults(
      descriptor.needle_cross_section_temporal_maturity_curve);
}

double GetSteadyTimeSeconds() {
  return std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}

std::filesystem::path ResolveDefaultScotsPineDescriptorPath() {
  return descriptor_defaults::ResolveExistingDefaultsPath(
      kScotsPineResourceCandidates,
      kScotsPineProjectAssetCandidates);
}

std::filesystem::path ResolveWritableScotsPineDescriptorDefaultsPath() {
  return descriptor_defaults::ResolveWritableDefaultsPath(
      kScotsPineResourceCandidates,
      kScotsPineProjectAssetCandidates,
      kScotsPineWritableTemplateCandidates,
      kScotsPineFallbackDefaultsPath);
}

void LoadSingleDistributionWithScalarFallback(const YAML::Node& in,
                                              const char* key,
                                              evo_engine::SingleDistribution<float>& distribution) {
  if (!in[key]) return;
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
void LoadLegacyGddDistributionAsYears(const YAML::Node& in,
                                      const char* legacy_key,
                                      evo_engine::SingleDistribution<float>& years_distribution) {
  if (!in[legacy_key]) return;
  evo_engine::SingleDistribution<float> tmp{};
  LoadSingleDistributionWithScalarFallback(in, legacy_key, tmp);
  years_distribution.mean = std::max(0.0f, tmp.mean) / kPineGddPerYear;
  years_distribution.deviation = std::max(0.0f, tmp.deviation) / kPineGddPerYear;
}

float SampleTargetGddForSeed(const evo_engine::SingleDistribution<float>& distribution,
                             const uint32_t seed) {
  std::mt19937 rng(seed);
  return std::max(0.0f, SampleDistribution(distribution, rng));
}

bool LoadScotsPineDescriptorDefaultsFromFile(ScotsPineDescriptor& descriptor,
                                             const std::filesystem::path& file_path) {
  YAML::Node defaults;
  if (!descriptor_defaults::LoadDefaultsYamlMap(
          file_path,
          defaults,
          kScotsPineDescriptorName)) {
    return false;
  }
  descriptor.Deserialize(defaults);
  return true;
}

}  // namespace

// ===========================================================================
// Constructor — load defaults from disk if available.
// ===========================================================================
ScotsPineDescriptor::ScotsPineDescriptor() {
  ConfigurePineMaturityDefaults(*this);
  const auto defaults_path = ResolveDefaultScotsPineDescriptorPath();
  if (!LoadScotsPineDescriptorDefaultsFromFile(*this, defaults_path)) {
    static bool warned_once = false;
    if (!warned_once) {
      warned_once = true;
      EVOENGINE_WARNING(
          "ScotsPineDescriptor defaults file not found or invalid. Using inline member defaults.");
    }
  }
}

std::filesystem::path ScotsPineDescriptor::ResolveWritableDefaultsPath() const {
  return ResolveWritableScotsPineDescriptorDefaultsPath();
}

// ===========================================================================
// Sample
//
// RNG draw order (LOCKED — append-only for reproducibility across schema
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

  out.max_branching_order = std::clamp(
      static_cast<int>(std::round(SampleDistribution(max_branching_order, rng))), 0, 4);
  out.plastochron_years =
      std::max(1e-4f, SampleDistribution(plastochron_gdd, rng) / kPineGddPerYear);
  out.max_phytomers_per_seasonal_growth = std::clamp(
      static_cast<int>(std::round(SampleDistribution(max_phytomers_per_seasonal_growth, rng))),
      1, 1024);

  out.branches_per_whorl = std::clamp(
      static_cast<int>(std::round(SampleDistribution(branches_per_whorl, rng))), 0, 12);
  // Chronological (years). No GDD->years conversion.
  out.whorl_dormancy_years =
      std::max(0.0f, SampleDistribution(whorl_dormancy_years, rng));
  out.branch_insertion_angle_deg =
      std::clamp(SampleDistribution(branch_insertion_angle_deg, rng), -85.0f, 85.0f);
  out.branch_roll_phyllotaxis_deg = SampleDistribution(branch_roll_phyllotaxis_deg, rng);

  out.internode_length_m =
      std::clamp(SampleDistribution(internode_length_m, rng), 0.0001f, 0.50f);
  out.leader_internode_thickness_m =
      std::clamp(SampleDistribution(leader_internode_thickness_m, rng), 0.0002f, 0.05f);
  // Keep stem slenderness physically plausible (avoid "fat stump" defaults).
  const float max_sane_thickness =
      std::max(0.0002f, 0.40f * out.internode_length_m * 8.0f);
  out.leader_internode_thickness_m =
      std::min(out.leader_internode_thickness_m, max_sane_thickness);
  out.lateral_length_ratio = std::clamp(SampleDistribution(lateral_length_ratio, rng), 0.05f, 1.0f);
  out.lateral_thickness_ratio = std::clamp(SampleDistribution(lateral_thickness_ratio, rng), 0.05f, 1.5f);

  out.bare_zone_fraction =
      std::clamp(SampleDistribution(bare_zone_fraction, rng), 0.0f, 0.95f);
  out.needle_count_per_cluster = std::clamp(
      static_cast<int>(std::round(SampleDistribution(needle_count_per_cluster, rng))), 1, 6);
    out.needle_segment_count = std::clamp(needle_segment_count, 3, 128);
  out.needle_length_m = std::clamp(SampleDistribution(needle_length_m, rng), 0.005f, 0.30f);
  // Chronological lifespan and browning (years). No GDD->years conversion.
  out.needle_lifespan_years = std::max(
      1, static_cast<int>(std::round(SampleDistribution(needle_lifespan_years, rng))));
  out.needle_browning_years =
      std::max(0.0f, SampleDistribution(needle_browning_years, rng));
  out.needle_flush_delay_years =
      std::max(0.0f, SampleDistribution(needle_flush_delay_gdd, rng) / kPineGddPerYear);
  out.internode_maturation_years =
      std::clamp(SampleDistribution(internode_maturation_gdd, rng) / kPineGddPerYear, 0.0f, 4.0f);
  out.needle_maturation_years =
      std::clamp(SampleDistribution(needle_maturation_gdd, rng) / kPineGddPerYear, 0.0f, 1.0f);
  out.needle_order_length_attenuation =
      std::clamp(needle_order_length_attenuation, 0.0f, 1.0f);
  out.needle_order_radius_attenuation =
      std::clamp(needle_order_radius_attenuation, 0.0f, 1.0f);
  out.needle_order_min_length_scale =
      std::clamp(needle_order_min_length_scale, 0.10f, 1.0f);
  out.needle_order_min_radius_scale =
      std::clamp(needle_order_min_radius_scale, 0.10f, 1.0f);
    out.needle_intra_year_base_ratio =
      std::clamp(needle_intra_year_base_ratio, 0.0f, 1.0f);
    out.needle_intra_year_sigmoid_steepness =
      std::max(0.01f, needle_intra_year_sigmoid_steepness);
    out.needle_intra_year_sigmoid_midpoint_fraction =
      std::clamp(needle_intra_year_sigmoid_midpoint_fraction, 0.0f, 1.0f);
    out.needle_intra_year_late_decay_start_fraction =
      std::clamp(needle_intra_year_late_decay_start_fraction, 0.0f, 1.0f);
    out.needle_intra_year_late_decay_end_scale =
      std::clamp(needle_intra_year_late_decay_end_scale, 0.0f, 2.0f);
    out.needle_fascicular_start_year =
      std::clamp(needle_fascicular_start_year, 0, 16);
    out.needle_year2plus_length_multiplier =
      std::max(0.0f, needle_year2plus_length_multiplier);
    out.needle_year2plus_width_multiplier =
      std::max(0.0f, needle_year2plus_width_multiplier);
    out.needle_year2plus_thickness_multiplier =
      std::max(0.0f, needle_year2plus_thickness_multiplier);
    out.needle_lignification_factor_year1 =
      std::clamp(needle_lignification_factor_year1, 0.0f, 2.0f);
    out.needle_lignification_factor_year2plus =
      std::clamp(needle_lignification_factor_year2plus, 0.0f, 2.0f);
    out.needle_stomatal_strip_density_year1 =
      std::clamp(needle_stomatal_strip_density_year1, 0.0f, 1.0f);
    out.needle_stomatal_strip_density_year2plus =
      std::clamp(needle_stomatal_strip_density_year2plus, 0.0f, 1.0f);
    out.needle_basal_taper_ratio_year1 =
      std::clamp(needle_basal_taper_ratio_year1, 0.6f, 1.2f);
    out.needle_basal_taper_ratio_year2plus =
      std::clamp(needle_basal_taper_ratio_year2plus, 0.6f, 1.2f);
    out.needle_fascicle_sheath_budget_years =
      std::max(0.0f, needle_fascicle_sheath_budget_gdd / kPineGddPerYear);
    out.needle_specularity_plasticity_year1 =
      std::clamp(needle_specularity_plasticity_year1, 0.0f, 1.0f);
    out.needle_specularity_plasticity_year2plus =
      std::clamp(needle_specularity_plasticity_year2plus, 0.0f, 1.0f);
    out.needle_bud_storage_vigor_strength =
      std::clamp(needle_bud_storage_vigor_strength, 0.0f, 1.0f);
    out.needle_bud_storage_completion_floor =
      std::clamp(needle_bud_storage_completion_floor, 0.0f, 1.0f);
  // Per-plant copy of the user-exposed needle radius cap. Bounded to a wide
  // but sane envelope: 0 disables the cap entirely (uncapped needles), and 4x
  // is well above what any realistic Scots pine needle would need relative
  // to its parent shoot.
  out.needle_radius_to_stem_thickness_max_ratio =
      std::clamp(needle_radius_to_stem_thickness_max_ratio, 0.0f, 4.0f);

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
  out.internode_length_per_node_cv =
      std::max(0.0f, SampleDistribution(internode_length_per_node_cv, rng));
  out.internode_thickness_per_node_cv =
      std::max(0.0f, SampleDistribution(internode_thickness_per_node_cv, rng));
  out.branch_angle_per_node_sigma_deg =
      std::max(0.0f, SampleDistribution(branch_angle_per_node_sigma_deg, rng));
  out.roll_phyllotaxis_per_node_sigma_deg =
      std::max(0.0f, SampleDistribution(roll_phyllotaxis_per_node_sigma_deg, rng));

  // Needle curvature.
  out.needle_curvature_adaxial_bias = SampleDistribution(needle_curvature_adaxial_bias, rng);
  out.needle_curvature_abaxial_bias = SampleDistribution(needle_curvature_abaxial_bias, rng);
  out.needle_curvature_gradient_per_arclen =
      SampleDistribution(needle_curvature_gradient_per_arclen, rng);
  out.needle_diameter_for_curvature_m =
      std::clamp(SampleDistribution(needle_diameter_for_curvature_m, rng), 0.0f, 0.005f);

  // Needle mechanics.
  out.needle_young_modulus_baseline_Pa =
      std::max(0.0f, SampleDistribution(needle_young_modulus_baseline_Pa, rng));
  out.needle_lignification_maturation_years =
      std::max(0.0f, SampleDistribution(needle_lignification_maturation_years, rng));
  // Keep width/thickness uncapped and independent. Only enforce
  // non-negativity so invalid negatives do not propagate.
  out.needle_cross_section_width_max_m =
      std::max(0.0f, SampleDistribution(needle_cross_section_width_max_m, rng));
  out.needle_cross_section_thickness_max_m =
      std::max(0.0f, SampleDistribution(needle_cross_section_thickness_max_m, rng));
  out.needle_density_kg_m3 = std::max(0.0f, SampleDistribution(needle_density_kg_m3, rng));
  out.gravity_m_s2 = std::max(0.0f, SampleDistribution(gravity_m_s2, rng));
    out.needle_per_needle_length_cv =
      std::max(0.0f, SampleDistribution(needle_per_needle_length_cv, rng));
    out.needle_per_needle_curvature_cv =
      std::max(0.0f, SampleDistribution(needle_per_needle_curvature_cv, rng));
    out.needle_per_needle_radius_cv =
      std::max(0.0f, SampleDistribution(needle_per_needle_radius_cv, rng));
    out.needle_per_needle_modulus_cv =
      std::max(0.0f, SampleDistribution(needle_per_needle_modulus_cv, rng));
    out.needle_per_needle_density_cv =
      std::max(0.0f, SampleDistribution(needle_per_needle_density_cv, rng));
      out.needle_sinusoidal_amplitude_deg =
        std::max(0.0f, SampleDistribution(needle_sinusoidal_amplitude_deg, rng));
      out.needle_sinusoidal_frequency_cycles =
        std::max(0.0f, SampleDistribution(needle_sinusoidal_frequency_cycles, rng));
      out.needle_sinusoidal_phase_randomness_deg =
        std::max(0.0f, SampleDistribution(needle_sinusoidal_phase_randomness_deg, rng));
      out.needle_per_needle_wave_amplitude_cv =
        std::max(0.0f, SampleDistribution(needle_per_needle_wave_amplitude_cv, rng));
      out.needle_per_needle_wave_frequency_cv =
        std::max(0.0f, SampleDistribution(needle_per_needle_wave_frequency_cv, rng));
      out.needle_per_needle_wave_phase_cv =
        std::max(0.0f, SampleDistribution(needle_per_needle_wave_phase_cv, rng));

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
// Instantiate — create entity with ScotsPine private component
// ===========================================================================
Entity ScotsPineDescriptor::Instantiate() const {
  const auto scene = Application::GetActiveScene();
  if (!scene) return {};

  const auto entity = scene->CreateEntity(GetTitle());
  const auto pine = scene->GetOrSetPrivateComponent<ScotsPine>(entity).lock();
  pine->descriptor_ref = GetSelf();
  pine->target_gdd = SampleTargetGddForSeed(target_gdd, pine->seed);
  pine->GenerateGeometryEntities();

  return entity;
}

// ===========================================================================
// Inspector UI
// ===========================================================================
bool ScotsPineDescriptor::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  bool editor_preferences_changed = false;

  const auto show_item_hover_description = [](const char* description) {
    if (!description || description[0] == '\0') return;
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
      ImGui::SetTooltip("%s", description);
    }
  };

  // -- Instantiation controls --
  if (ImGui::Button("Instantiate")) {
    editor_layer->SetSelectedEntity(Instantiate());
  }
  show_item_hover_description("Create a new ScotsPine entity using this descriptor and select it in the scene.");

  ImGui::SameLine();
  if (ImGui::Checkbox("Live Preview", &live_preview)) {
    editor_preferences_changed = true;
    if (!live_preview) {
      live_preview_dirty_ = false;
      live_preview_was_dragging_ = false;
      live_preview_needs_full_apply_ = false;
    }
  }
  show_item_hover_description("Regenerate matching pines while editing this descriptor.");

  if (ImGui::DragFloat("Live Preview Rate (Hz)", &live_preview_rate_hz, 0.25f, 1.0f, 60.0f, "%.1f")) {
    live_preview_rate_hz = std::clamp(live_preview_rate_hz, 1.0f, 60.0f);
    editor_preferences_changed = true;
  }
  show_item_hover_description("Maximum live-preview apply frequency.");

  if (ImGui::Checkbox("Representative Only While Dragging", &live_preview_representative_only)) {
    editor_preferences_changed = true;
  }
  show_item_hover_description("While dragging controls, preview only one matching pine.");

  if (ImGui::Checkbox("Cap Preview Target GDD", &live_preview_cap_target_gdd)) {
    editor_preferences_changed = true;
  }
  show_item_hover_description("Clamp preview simulation GDD so live updates stay fast on very mature trees.");

  if (ImGui::DragFloat("Preview Max GDD", &live_preview_max_gdd, 50.0f, 0.0f, 100000.0f, "%.1f")) {
    live_preview_max_gdd = std::max(0.0f, live_preview_max_gdd);
    editor_preferences_changed = true;
  }
  show_item_hover_description("Upper GDD limit used when preview capping is enabled.");

  if (ImGui::DragInt("Preview Max Growth Steps", &live_preview_max_growth_steps, 1.0f, 1, 10000)) {
    live_preview_max_growth_steps = std::clamp(live_preview_max_growth_steps, 1, 10000);
    editor_preferences_changed = true;
  }
  show_item_hover_description("Maximum derivation/growth iterations used by drag-time preview updates.");

  if (live_preview_apply_count_ > 0) {
    const double avg_apply_ms = live_preview_total_apply_ms_ /
                                static_cast<double>(live_preview_apply_count_);
    ImGui::Text("Preview last/avg ms: %.3f / %.3f", live_preview_last_apply_ms_, avg_apply_ms);
  }
  ImGui::Text("Preview requests/applied/coalesced: %u / %u / %u",
              live_preview_request_count_, live_preview_apply_count_, live_preview_coalesced_count_);

  if (ImGui::SmallButton("Reset Preview Stats")) {
    live_preview_request_count_ = 0;
    live_preview_apply_count_ = 0;
    live_preview_coalesced_count_ = 0;
    live_preview_last_apply_ms_ = 0.0;
    live_preview_total_apply_ms_ = 0.0;
  }
  show_item_hover_description("Reset live-preview timing and coalescing counters.");

  // -- Grid instantiation --
  if (ImGui::TreeNodeEx("Grid Instantiate")) {
    ImGui::DragInt("Rows", &grid_rows, 1, 1, 50);
    show_item_hover_description("Number of rows for grid instantiation.");
    ImGui::DragInt("Cols", &grid_cols, 1, 1, 50);
    show_item_hover_description("Number of columns for grid instantiation.");
    ImGui::DragFloat("Spacing", &grid_spacing, 0.1f, 0.5f, 50.0f);
    show_item_hover_description("World-space spacing between neighboring grid pines.");

    if (ImGui::Button("Instantiate Grid")) {
      const auto scene = Application::GetActiveScene();
      if (scene) {
        const auto container = scene->CreateEntity("Pine Grid");
        const float offset_y = (static_cast<float>(grid_rows) - 1.0f) * grid_spacing * 0.5f;
        const float offset_z = (static_cast<float>(grid_cols) - 1.0f) * grid_spacing * 0.5f;
        const auto base_seed = static_cast<unsigned int>(
            std::chrono::steady_clock::now().time_since_epoch().count() & 0xFFFFFFFFu);
        for (int i = 0; i < grid_rows; i++) {
          for (int j = 0; j < grid_cols; j++) {
            const auto entity =
                scene->CreateEntity(GetTitle() + " [" + std::to_string(i) + "," + std::to_string(j) + "]");
            const auto pine = scene->GetOrSetPrivateComponent<ScotsPine>(entity).lock();
            pine->descriptor_ref = GetSelf();
            pine->seed = base_seed + static_cast<unsigned int>(i * grid_cols + j);
            pine->target_gdd = SampleTargetGddForSeed(target_gdd, pine->seed);

            scene->SetParent(entity, container, false);

            Transform transform;
            transform.SetPosition(glm::vec3(
                0.0f,
                static_cast<float>(i) * grid_spacing - offset_y,
                static_cast<float>(j) * grid_spacing - offset_z));
            scene->SetDataComponent(entity, transform);

            pine->GenerateGeometryEntities();
          }
        }
      }
    }
    show_item_hover_description("Spawn a grid of ScotsPine entities from this descriptor with unique seeds.");

    ImGui::SameLine();
    if (ImGui::Button("Delete Grid")) {
      const auto scene = Application::GetActiveScene();
      if (scene) {
        const auto* pine_entities_ptr = scene->UnsafeGetPrivateComponentOwnersList<ScotsPine>();
        if (pine_entities_ptr) {
          const std::vector<Entity> pine_entities = *pine_entities_ptr;
          std::vector<Entity> to_delete;
          std::vector<Entity> containers;
          for (const auto& entity : pine_entities) {
            if (!scene->IsEntityValid(entity)) continue;
            auto pine = scene->GetOrSetPrivateComponent<ScotsPine>(entity).lock();
            if (!pine) continue;
            if (pine->descriptor_ref.Get<ScotsPineDescriptor>().get() == this) {
              to_delete.push_back(entity);
              const auto parent = scene->GetParent(entity);
              if (scene->IsEntityValid(parent) && scene->GetEntityName(parent) == "Pine Grid") {
                containers.push_back(parent);
              }
            }
          }
          for (const auto& entity : to_delete) scene->DeleteEntity(entity);
          std::sort(containers.begin(), containers.end(),
                    [](const Entity& a, const Entity& b) { return a.GetIndex() < b.GetIndex(); });
          containers.erase(std::unique(containers.begin(), containers.end()), containers.end());
          for (const auto& container : containers) {
            if (scene->IsEntityValid(container)) scene->DeleteEntity(container);
          }
        }
      }
    }
    show_item_hover_description(
        "Delete ScotsPine entities that use this descriptor and remove now-empty grid containers.");

    ImGui::TreePop();
  }

  // -- Triangle instantiation --
  if (ImGui::TreeNodeEx("Triangle Instantiate")) {
    ImGui::DragFloat("Side Length", &triangle_side_length, 0.1f);
    show_item_hover_description(
      "World-space side length for an equilateral 3-pine triangle on the horizontal XZ plane.");

    if (ImGui::Button("Instantiate Triangle")) {
      const auto scene = Application::GetActiveScene();
      if (scene) {
        const auto container = scene->CreateEntity("Pine Triangle");
        const float side_length = triangle_side_length;
        const float half_side = side_length * 0.5f;
        const float triangle_height = side_length * std::sqrt(3.0f) * 0.5f;
        const float centroid_to_apex = (2.0f / 3.0f) * triangle_height;
        const float centroid_to_base = (1.0f / 3.0f) * triangle_height;
        const std::array<glm::vec3, 3> triangle_positions = {
          glm::vec3(0.0f, 0.0f, centroid_to_apex),
          glm::vec3(-half_side, 0.0f, -centroid_to_base),
          glm::vec3(half_side, 0.0f, -centroid_to_base),
        };

        const auto base_seed = static_cast<unsigned int>(
            std::chrono::steady_clock::now().time_since_epoch().count() & 0xFFFFFFFFu);
        for (int i = 0; i < static_cast<int>(triangle_positions.size()); ++i) {
          const auto entity = scene->CreateEntity(GetTitle() + " [T" + std::to_string(i) + "]");
          const auto pine = scene->GetOrSetPrivateComponent<ScotsPine>(entity).lock();
          pine->descriptor_ref = GetSelf();
          pine->seed = base_seed + static_cast<unsigned int>(i);
          pine->target_gdd = SampleTargetGddForSeed(target_gdd, pine->seed);

          scene->SetParent(entity, container, false);

          Transform transform;
          transform.SetPosition(triangle_positions[static_cast<size_t>(i)]);
          scene->SetDataComponent(entity, transform);

          pine->GenerateGeometryEntities();
        }
        editor_layer->SetSelectedEntity(container);
      }
    }
    show_item_hover_description(
        "Spawn 3 ScotsPine entities in an equilateral triangle with unique seeds.");

    ImGui::SameLine();
    if (ImGui::Button("Delete Triangle")) {
      const auto scene = Application::GetActiveScene();
      if (scene) {
        const auto* pine_entities_ptr = scene->UnsafeGetPrivateComponentOwnersList<ScotsPine>();
        if (pine_entities_ptr) {
          const std::vector<Entity> pine_entities = *pine_entities_ptr;
          std::vector<Entity> to_delete;
          std::vector<Entity> containers;
          for (const auto& entity : pine_entities) {
            if (!scene->IsEntityValid(entity)) continue;
            auto pine = scene->GetOrSetPrivateComponent<ScotsPine>(entity).lock();
            if (!pine) continue;
            if (pine->descriptor_ref.Get<ScotsPineDescriptor>().get() == this) {
              to_delete.push_back(entity);
              const auto parent = scene->GetParent(entity);
              if (scene->IsEntityValid(parent) && scene->GetEntityName(parent) == "Pine Triangle") {
                containers.push_back(parent);
              }
            }
          }
          for (const auto& entity : to_delete) scene->DeleteEntity(entity);
          std::sort(containers.begin(), containers.end(),
                    [](const Entity& a, const Entity& b) { return a.GetIndex() < b.GetIndex(); });
          containers.erase(std::unique(containers.begin(), containers.end()), containers.end());
          for (const auto& container : containers) {
            if (scene->IsEntityValid(container)) scene->DeleteEntity(container);
          }
        }
      }
    }
    show_item_hover_description(
        "Delete ScotsPine entities that use this descriptor and remove now-empty triangle containers.");

    ImGui::TreePop();
  }

  ImGui::Separator();

  // -- Parameter Space Explorer --
  if (ImGui::TreeNodeEx("Parameter Space Explorer")) {
    if (!explorer_.IsBound()) explorer_.Bind(*this);
    if (explorer_.OnInspect()) {
      changed = true;
    }
    show_item_hover_description(
        "Interactive parameter sweep and sensitivity exploration tools for this descriptor.");
    ImGui::TreePop();
  }

  ImGui::Separator();

  // -- Global development clock --
  if (ImGui::TreeNodeEx("Global Development", ImGuiTreeNodeFlags_DefaultOpen)) {
    changed |= target_gdd.OnInspect("Target GDD", 1.0f,
        "Distribution of target GDD used by Instantiate, Grid spawn, and Triangle spawn.", "%.1f");
    changed |= plastochron_gdd.OnInspect("Plastochron (GDD)", 10.0f,
        "Physiological time between consecutive phytomer events on an axis.", "%.1f");
    changed |= max_phytomers_per_seasonal_growth.OnInspect("Max Phytomers per Seasonal Growth", 0.5f,
        "Phytomers (internode + optional needle cluster) emitted per active season before the apex pauses until next year.",
        "%.1f");

    // -- Per-pine calendar / GDD-per-day fields --
    // Consumed by LSystemLayer::SamplePineTemporalParameters() and applied
    // in the per-pine update path. delta_gdd = sampled_gdd_per_day * delta_days
    // where delta_days = chronological_days_per_second * dt.
    changed |= gdd_per_day.OnInspect(
        "GDD per Day", 0.1f,
        "Per-pine thermal accumulation rate. Sampled per plant; multiplied by chronological day delta from LSystemLayer.",
        "%.3f");
    changed |= growing_season_start_day.OnInspect(
        "Growing Season Start Day", 1.0f,
        "Per-pine active season start day-of-year (0-365). Sampled per plant; gates pine growth in LSystemLayer.",
        "%.1f");
    changed |= growing_season_end_day.OnInspect(
        "Growing Season End Day", 1.0f,
        "Per-pine active season end day-of-year (0-365). Sampled per plant; gates pine growth in LSystemLayer.",
        "%.1f");

    auto clamp_nonnegative_distribution = [&](evo_engine::SingleDistribution<float>& distribution) {
      const float old_mean = distribution.mean;
      const float old_deviation = distribution.deviation;
      distribution.mean = std::max(0.0f, distribution.mean);
      distribution.deviation = std::max(0.0f, distribution.deviation);
      if (std::abs(distribution.mean - old_mean) > 1.0e-6f ||
          std::abs(distribution.deviation - old_deviation) > 1.0e-6f) {
        changed = true;
      }
    };
    auto clamp_integer_day_distribution = [&](evo_engine::SingleDistribution<float>& distribution) {
      const float old_mean = distribution.mean;
      const float old_deviation = distribution.deviation;
      distribution.mean = std::clamp(distribution.mean, 0.0f, 365.0f);
      distribution.deviation = std::max(0.0f, std::round(distribution.deviation));
      if (std::abs(distribution.mean - old_mean) > 1.0e-6f ||
          std::abs(distribution.deviation - old_deviation) > 1.0e-6f) {
        changed = true;
      }
    };
    clamp_nonnegative_distribution(gdd_per_day);
    clamp_integer_day_distribution(growing_season_start_day);
    clamp_integer_day_distribution(growing_season_end_day);

    ImGui::TreePop();
  }

  // -- Main stem geometry --
  if (ImGui::TreeNodeEx("Main Stem (Leader Axis)", ImGuiTreeNodeFlags_DefaultOpen)) {
    changed |= internode_length_m.OnInspect("Phytomer Internode Length (m)", 0.001f,
        "Length of one phytomer's internode in metres.", "%.6f");
    changed |= leader_internode_thickness_m.OnInspect("Main Stem Width (Diameter, m)", 0.0001f,
        "Main stem thickness control. This is the leader internode diameter in metres.", "%.6f");
    changed |= initial_orientation_yaw_deg.OnInspect("Initial Orientation Yaw (deg)", 1.0f,
        "Sampled once per plant and applied as root yaw around +Y. Set deviation > 0 for random initial orientation.",
        "%.3f");
    if (ImGui::ColorEdit4("Main Stem Color", &main_stem_color_rgba.x)) {
      changed = true;
    }
    show_item_hover_description(
        "Young stem color used by stem and branch internodes in Shaded and ByType modes.");
    if (ImGui::ColorEdit4("Main Stem Old Color", &main_stem_old_color_rgba.x)) {
      changed = true;
    }
    show_item_hover_description(
        "Old stem color reached as internodes approach descriptor max age.");
    if (ImGui::DragFloat("Main Stem Age Exponent", &internode_age_exponent, 0.05f, 0.1f, 4.0f, "%.2f")) {
      internode_age_exponent = std::clamp(internode_age_exponent, 0.1f, 4.0f);
      changed = true;
    }
    show_item_hover_description(
        "Response curve for stem aging color. 1 = linear, >1 delays browning, <1 accelerates it.");
    ImGui::Text("Mean Radius (m): %.6f", std::max(0.0f, leader_internode_thickness_m.mean) * 0.5f);
    changed |= lateral_length_ratio.OnInspect("Lateral Length Ratio", 0.05f);
    show_item_hover_description("Lateral shoot length = leader_length * ratio^order.");
    changed |= lateral_thickness_ratio.OnInspect("Lateral Thickness Ratio", 0.05f);
    show_item_hover_description("Lateral shoot thickness = leader_thickness * ratio^order.");
    ImGui::TreePop();
  }

  // -- Main stem branching --
  if (ImGui::TreeNodeEx("Main Stem Branching (Whorl Buds)", ImGuiTreeNodeFlags_DefaultOpen)) {
    changed |= max_branching_order.OnInspect("Max Branching Order", 0.5f);
    show_item_hover_description("0 = leader only, 1 = primary laterals, 2 = secondary laterals.");
    changed |= branches_per_whorl.OnInspect("Branches per Whorl", 0.5f);
    show_item_hover_description("Lateral count spawned at whorl bud activation.");
    changed |= whorl_dormancy_years.OnInspect("Whorl Dormancy (years)", 0.05f,
        "Chronological years a whorl bud waits before activating laterals. "
        "Bud release is chilling/photoperiod-driven, NOT heat-sum-driven "
        "(FSPM Rule of Ontogeny). Default 1 yr = annual Scots pine cycle.", "%.3f");
    changed |= branch_insertion_angle_deg.OnInspect("Branch Insertion Angle (deg)", 1.0f);
    show_item_hover_description("Angle laterals depart parent (degrees).");
    changed |= branch_roll_phyllotaxis_deg.OnInspect("Branch Roll Phyllotaxis (deg)", 1.0f);
    show_item_hover_description("Golden-angle azimuth offset between consecutive laterals and needles.");
    ImGui::TreePop();
  }

  // -- Maturity shape curves --
  if (ImGui::TreeNodeEx("Maturity Shape Curves", ImGuiTreeNodeFlags_DefaultOpen)) {
    static int selected_maturity_variable = 0;
    constexpr const char* kMaturityVariables[] = {
      "Internode Length",
      "Internode Width",
      "Needle Length"
    };

    ImGui::Combo("Variable", &selected_maturity_variable,
                 kMaturityVariables, IM_ARRAYSIZE(kMaturityVariables));
    show_item_hover_description(
        "Choose which maturity-controlled variable to edit. "
        "x = normalized maturity age of the specific organ instance; "
        "y = multiplier in [0,1], where 1 means use full max length/width.");

    evo_engine::PlottedDistribution<float>* selected_distribution =
        &internode_length_maturity_curve;
    const char* selected_label = "Internode Length Maturity Response";
    switch (std::clamp(selected_maturity_variable, 0, 2)) {
      case 0:
      default:
        selected_distribution = &internode_length_maturity_curve;
        selected_label = "Internode Length Maturity Response";
        break;
      case 1:
        selected_distribution = &internode_width_maturity_curve;
        selected_label = "Internode Width Maturity Response";
        break;
      case 2:
        selected_distribution = &needle_length_maturity_curve;
        selected_label = "Needle Length Maturity Response";
        break;
    }

    evo_engine::PlottedDistributionSettings maturity_settings;
    maturity_settings.tip =
        "Two plotted controls are exposed: mean and variance over maturity age. "
        "For a fixed organ instance, runtime samples one deterministic realization and "
        "applies it along this curve over age.";
    maturity_settings.mean_settings.m_tip =
        "Mean maturity response curve. x = maturity age fraction [0,1], y = size multiplier [0,1].";
    maturity_settings.dev_settings.m_tip =
        "Variance (sigma) over maturity age. Runtime uses a fixed per-organ realization (no frame jitter).";
    changed |= selected_distribution->OnInspect(selected_label, maturity_settings);

    auto clamp_plot_01 = [](evo_engine::Plot2D<float>& plot) {
      plot.min_value = std::clamp(plot.min_value, 0.0f, 1.0f);
      plot.max_value = std::clamp(plot.max_value, 0.0f, 1.0f);
      if (plot.max_value < plot.min_value) {
        std::swap(plot.min_value, plot.max_value);
      }
    };
    clamp_plot_01(selected_distribution->mean);
    clamp_plot_01(selected_distribution->deviation);

    ImGui::TreePop();
  }

  // -- Needles: Quick Shape Presets --
  if (ImGui::TreeNodeEx("Needles - Quick Shape Presets", ImGuiTreeNodeFlags_DefaultOpen)) {
    static const char* kShapePresets[] = {
        "(no change)", "Straight", "Slight Curve", "Strong Curve", "Wavy",
        "Drooping (gravity)"};
    static int s_selected_shape_preset = 0;
    if (ImGui::Combo("Shape Preset##quick_needle_shape",
                     &s_selected_shape_preset, kShapePresets,
                     IM_ARRAYSIZE(kShapePresets))) {
      auto apply_preset = [&](float adaxial, float abaxial, float gradient,
                              float diameter_for_curvature_m, float wave_amp_deg,
                              float wave_freq, float wave_phase_rand_deg,
                              bool enable_droop) {
        needle_curvature_adaxial_bias.mean = adaxial;
        needle_curvature_abaxial_bias.mean = abaxial;
        needle_curvature_gradient_per_arclen.mean = gradient;
        needle_diameter_for_curvature_m.mean = diameter_for_curvature_m;
        needle_sinusoidal_amplitude_deg.mean = wave_amp_deg;
        needle_sinusoidal_frequency_cycles.mean = wave_freq;
        needle_sinusoidal_phase_randomness_deg.mean = wave_phase_rand_deg;
        if (enable_droop) {
          if (needle_young_modulus_baseline_Pa.mean <= 0.0f) {
            needle_young_modulus_baseline_Pa.mean = 1.8e7f;
          }
          if (needle_density_kg_m3.mean <= 0.0f) {
            needle_density_kg_m3.mean = 800.0f;
          }
          if (gravity_m_s2.mean <= 0.0f) {
            gravity_m_s2.mean = 9.81f;
          }
        }
      };
      switch (s_selected_shape_preset) {
        case 1:
          apply_preset(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false);
          changed = true;
          break;
        case 2:
          apply_preset(0.000f, 0.010f, 0.0015f, 0.001f, 0.0f, 0.0f, 0.0f, false);
          changed = true;
          break;
        case 3:
          apply_preset(0.000f, 0.030f, 0.0040f, 0.002f, 0.0f, 0.0f, 0.0f, false);
          changed = true;
          break;
        case 4:
          apply_preset(0.0f, 0.0f, 0.0f, 0.0f, 4.0f, 2.0f, 5.0f, false);
          changed = true;
          break;
        case 5:
          apply_preset(0.000f, 0.005f, 0.0010f, 0.001f, 0.0f, 0.0f, 0.0f, true);
          changed = true;
          break;
        case 0:
        default:
          break;
      }
      s_selected_shape_preset = 0;
    }
    show_item_hover_description(
        "Writes a coordinated set of curvature, waviness, and (for Drooping) "
        "mechanics fields.");
    ImGui::TreePop();
  }

  // -- Needle cross-section controls --
  if (ImGui::TreeNodeEx("Needle Cross Section", ImGuiTreeNodeFlags_DefaultOpen)) {
    auto clamp_nonnegative_distribution = [&](evo_engine::SingleDistribution<float>& distribution) {
      const float old_mean = distribution.mean;
      const float old_deviation = distribution.deviation;
      distribution.mean = std::max(0.0f, distribution.mean);
      distribution.deviation = std::max(0.0f, distribution.deviation);
      if (std::abs(distribution.mean - old_mean) > 1.0e-6f ||
          std::abs(distribution.deviation - old_deviation) > 1.0e-6f) {
        changed = true;
      }
    };
    auto clamp_profile_distribution = [&](evo_engine::PlottedDistribution<float>& distribution,
                                          const float max_value) {
      auto clamp_plot = [&](evo_engine::Plot2D<float>& plot) {
        const float old_min = plot.min_value;
        const float old_max = plot.max_value;
        plot.min_value = std::clamp(plot.min_value, 0.0f, 1.0f);
        plot.max_value = std::clamp(plot.max_value, 0.0f, max_value);
        if (plot.max_value < plot.min_value) {
          std::swap(plot.min_value, plot.max_value);
        }
        if (std::abs(plot.min_value - old_min) > 1.0e-6f ||
            std::abs(plot.max_value - old_max) > 1.0e-6f) {
          changed = true;
        }
      };
      clamp_plot(distribution.mean);
      clamp_plot(distribution.deviation);
    };
    auto inspect_axis = [&](const char* axis_name,
                            evo_engine::SingleDistribution<float>& max_distribution,
                            evo_engine::PlottedDistribution<float>& profile_distribution,
                            const char* max_label,
                            const char* profile_label,
                            const char* tooltip) {
      if (ImGui::TreeNodeEx(axis_name, ImGuiTreeNodeFlags_DefaultOpen)) {
        changed |= max_distribution.OnInspect(max_label, 0.00005f, tooltip, "%.6f");
        clamp_nonnegative_distribution(max_distribution);

        evo_engine::PlottedDistributionSettings profile_settings;
        profile_settings.tip =
            "Base-to-tip multiplier profile for the selected cross-section axis. "
            "x = normalized arc length from base (0) to tip (1).";
        profile_settings.mean_settings.m_tip =
          "Mean multiplier profile in [0,4]. 1 keeps the max diameter; 0 collapses axis radius; "
          "values >1 enlarge it.";
        profile_settings.dev_settings.m_tip =
          "Variance (sigma) profile in [0,4] around the mean profile.";
        changed |= profile_distribution.OnInspect(profile_label, profile_settings);
        clamp_profile_distribution(profile_distribution, 4.0f);

        ImGui::Text("Current mean max diameter: %.3f mm",
                    std::max(0.0f, max_distribution.mean) * 1000.0f);
        ImGui::TreePop();
      }
    };

    inspect_axis("Width", needle_cross_section_width_max_m,
                 needle_cross_section_width_profile,
                 "Max Width Diameter (m)",
                 "Width Profile (Base -> Tip)",
                 "Maximum full width (major axis diameter) before profile multiplier.");
    inspect_axis("Thickness", needle_cross_section_thickness_max_m,
                 needle_cross_section_thickness_profile,
                 "Max Thickness Diameter (m)",
                 "Thickness Profile (Base -> Tip)",
                 "Maximum full thickness (minor axis diameter) before profile multiplier.");

    evo_engine::PlottedDistributionSettings temporal_settings;
    temporal_settings.tip =
      "Shared temporal multiplier applied to both width and thickness. "
      "x = normalized maturity age where x=1 corresponds to 2 years since initiation.";
    temporal_settings.mean_settings.m_tip =
      "Mean multiplier in [0,1]. Default is sinusoidal: 0.25 at t=0 years to 1.0 at t=2 years.";
    temporal_settings.dev_settings.m_tip =
      "Variance (sigma) profile around the temporal mean in [0,1].";
    changed |= needle_cross_section_temporal_maturity_curve.OnInspect(
      "Shared Temporal Width/Thickness Maturity", temporal_settings);
    clamp_profile_distribution(needle_cross_section_temporal_maturity_curve, 1.0f);

    ImGui::TreePop();
  }

  // -- Needles --
  if (ImGui::TreeNodeEx("Needles (Layout and Lifecycle)", ImGuiTreeNodeFlags_DefaultOpen)) {
    changed |= bare_zone_fraction.OnInspect("Bare Zone Fraction", 0.01f,
        "Temporal fraction at the start of each year that emits internode-only phytomers [0, 0.95).", "%.3f");
    changed |= needle_count_per_cluster.OnInspect("Needles per Cluster", 0.5f);
    show_item_hover_description("Pinus sylvestris fascicle count (typically 2).");
    if (ImGui::DragInt("Needle Segments", &needle_segment_count, 1.0f, 3, 128)) {
      needle_segment_count = std::clamp(needle_segment_count, 3, 128);
      changed = true;
    }
    show_item_hover_description(
      "Longitudinal segments per needle centerline. Mesh stations = segments + 1.");
    changed |= needle_length_m.OnInspect("Needle Length (m)", 0.001f,
        "Length of needles in metres.", "%.6f");
    changed |= needle_lifespan_years.OnInspect("Needle Lifespan (years)", 0.1f,
        "Chronological years a needle stays alive post-maturity. Senescence "
        "is calendar-driven, NOT heat-sum-driven (FSPM Rule of Ontogeny). "
        "Scots pine typical: 3-4 yr.", "%.2f");
    changed |= needle_browning_years.OnInspect("Needle Browning (years)", 0.05f,
        "Chronological years from senescence onset to abscission.", "%.3f");
    changed |= needle_flush_delay_gdd.OnInspect("Needle Flush Delay (GDD)", 10.0f,
        "Delay from phytomer emergence to needle flush.", "%.1f");
    changed |= internode_maturation_gdd.OnInspect("Shoot Maturation (GDD)", 10.0f,
        "Thermal time from emergence to mature internode length.", "%.1f");
    changed |= needle_maturation_gdd.OnInspect("Needle Maturation (GDD)", 10.0f,
        "Thermal time from flush to mature needle length.", "%.1f");
    changed |= needle_branching_angle_deg.OnInspect("Needle Branching Angle (deg)", 0.25f,
      "Final branching angle from the parent axis reached after relaxation.", "%.2f");
    changed |= needle_branching_relax_gdd.OnInspect("Needle Branching Relaxation (GDD-equivalent)", 10.0f,
      "Converted using 1500 GDD/year, then applied against chronological age so relaxation continues during dormant season.",
      "%.1f");
    if (ImGui::DragFloat("Order Needle Length Attenuation", &needle_order_length_attenuation,
                         0.01f, 0.0f, 1.0f, "%.3f")) {
      needle_order_length_attenuation = std::clamp(needle_order_length_attenuation, 0.0f, 1.0f);
      changed = true;
    }
    show_item_hover_description(
        "Linear per-order reduction applied to needle length: scale = max(min, 1 - attenuation*order).");
    if (ImGui::DragFloat("Order Needle Radius Attenuation", &needle_order_radius_attenuation,
                         0.01f, 0.0f, 1.0f, "%.3f")) {
      needle_order_radius_attenuation = std::clamp(needle_order_radius_attenuation, 0.0f, 1.0f);
      changed = true;
    }
    show_item_hover_description(
        "Linear per-order reduction applied to needle thickness: scale = max(min, 1 - attenuation*order).");
    if (ImGui::DragFloat("Order Needle Length Min Scale", &needle_order_min_length_scale,
                         0.01f, 0.10f, 1.00f, "%.3f")) {
      needle_order_min_length_scale = std::clamp(needle_order_min_length_scale, 0.10f, 1.00f);
      changed = true;
    }
    show_item_hover_description("Lower bound for branch-order needle length scaling.");
    if (ImGui::DragFloat("Order Needle Radius Min Scale", &needle_order_min_radius_scale,
                         0.01f, 0.10f, 1.00f, "%.3f")) {
      needle_order_min_radius_scale = std::clamp(needle_order_min_radius_scale, 0.10f, 1.00f);
      changed = true;
    }
    show_item_hover_description("Lower bound for branch-order needle thickness scaling.");
    if (ImGui::TreeNodeEx("Initiation Capacity Mapping", ImGuiTreeNodeFlags_DefaultOpen)) {
      if (ImGui::DragFloat("Intra-Year Base Ratio", &needle_intra_year_base_ratio,
                           0.01f, 0.0f, 1.0f, "%.3f")) {
        needle_intra_year_base_ratio =
            std::clamp(needle_intra_year_base_ratio, 0.0f, 1.0f);
        changed = true;
      }
      show_item_hover_description(
          "Lower bound for early-phytomer capacity."
          " Runtime applies L_actual = L_max * w_intra(i) * w_inter(year,vigor)."
      );

      if (ImGui::DragFloat("Intra-Year Sigmoid Steepness",
                           &needle_intra_year_sigmoid_steepness,
                           0.10f, 0.01f, 32.0f, "%.3f")) {
        needle_intra_year_sigmoid_steepness =
            std::max(0.01f, needle_intra_year_sigmoid_steepness);
        changed = true;
      }
      show_item_hover_description("Steepness k of the intra-year sigmoid over normalized phytomer index.");

      if (ImGui::DragFloat("Intra-Year Sigmoid Midpoint",
                           &needle_intra_year_sigmoid_midpoint_fraction,
                           0.01f, 0.0f, 1.0f, "%.3f")) {
        needle_intra_year_sigmoid_midpoint_fraction =
            std::clamp(needle_intra_year_sigmoid_midpoint_fraction, 0.0f, 1.0f);
        changed = true;
      }
      show_item_hover_description("Normalized phytomer index where intra-year capacity reaches 50% ramp.");

      if (ImGui::DragFloat("Late-Season Decay Start",
                           &needle_intra_year_late_decay_start_fraction,
                           0.01f, 0.0f, 1.0f, "%.3f")) {
        needle_intra_year_late_decay_start_fraction =
            std::clamp(needle_intra_year_late_decay_start_fraction, 0.0f, 1.0f);
        changed = true;
      }
      show_item_hover_description("Normalized phytomer index where optional late-season decay begins.");

      if (ImGui::DragFloat("Late-Season End Scale",
                           &needle_intra_year_late_decay_end_scale,
                           0.01f, 0.0f, 2.0f, "%.3f")) {
        needle_intra_year_late_decay_end_scale =
            std::clamp(needle_intra_year_late_decay_end_scale, 0.0f, 2.0f);
        changed = true;
      }
      show_item_hover_description("Multiplier reached at the final phytomer if late-season decay is active.");

      if (ImGui::DragInt("Fascicular Start Year", &needle_fascicular_start_year,
                         1.0f, 0, 16)) {
        needle_fascicular_start_year = std::clamp(needle_fascicular_start_year, 0, 16);
        changed = true;
      }
      show_item_hover_description(
          "Year index where year2+ multipliers become active. 0 applies them from first-year shoots.");

      if (ImGui::DragFloat("Year2+ Length Multiplier", &needle_year2plus_length_multiplier,
                           0.01f, 0.0f, 8.0f, "%.3f")) {
        needle_year2plus_length_multiplier = std::max(0.0f, needle_year2plus_length_multiplier);
        changed = true;
      }
      show_item_hover_description("Inter-year multiplier for needle target length.");

      if (ImGui::DragFloat("Year2+ Width Multiplier", &needle_year2plus_width_multiplier,
                           0.01f, 0.0f, 8.0f, "%.3f")) {
        needle_year2plus_width_multiplier = std::max(0.0f, needle_year2plus_width_multiplier);
        changed = true;
      }
      show_item_hover_description("Inter-year multiplier for needle major-axis width.");

      if (ImGui::DragFloat("Year2+ Thickness Multiplier", &needle_year2plus_thickness_multiplier,
                           0.01f, 0.0f, 8.0f, "%.3f")) {
        needle_year2plus_thickness_multiplier =
            std::max(0.0f, needle_year2plus_thickness_multiplier);
        changed = true;
      }
      show_item_hover_description("Inter-year multiplier for needle minor-axis thickness.");

        if (ImGui::DragFloat("Year1 Lignification Factor", &needle_lignification_factor_year1,
                   0.01f, 0.0f, 2.0f, "%.3f")) {
        needle_lignification_factor_year1 =
          std::clamp(needle_lignification_factor_year1, 0.0f, 2.0f);
        changed = true;
        }
        show_item_hover_description(
          "Scales visual maturation response for first-year needle cohorts.");

        if (ImGui::DragFloat("Year2+ Lignification Factor", &needle_lignification_factor_year2plus,
                   0.01f, 0.0f, 2.0f, "%.3f")) {
        needle_lignification_factor_year2plus =
          std::clamp(needle_lignification_factor_year2plus, 0.0f, 2.0f);
        changed = true;
        }
        show_item_hover_description(
          "Scales visual maturation response for year2+ needle cohorts.");

        if (ImGui::DragFloat("Year1 Stomatal Strip Density", &needle_stomatal_strip_density_year1,
                   0.01f, 0.0f, 1.0f, "%.3f")) {
        needle_stomatal_strip_density_year1 =
          std::clamp(needle_stomatal_strip_density_year1, 0.0f, 1.0f);
        changed = true;
        }
        show_item_hover_description(
          "Proxy density for procedural stomatal striping in first-year cohorts.");

        if (ImGui::DragFloat("Year2+ Stomatal Strip Density", &needle_stomatal_strip_density_year2plus,
                   0.01f, 0.0f, 1.0f, "%.3f")) {
        needle_stomatal_strip_density_year2plus =
          std::clamp(needle_stomatal_strip_density_year2plus, 0.0f, 1.0f);
        changed = true;
        }
        show_item_hover_description(
          "Proxy density for procedural stomatal striping in year2+ cohorts.");

        if (ImGui::DragFloat("Year1 Basal Taper Ratio", &needle_basal_taper_ratio_year1,
                   0.01f, 0.6f, 1.2f, "%.3f")) {
        needle_basal_taper_ratio_year1 =
          std::clamp(needle_basal_taper_ratio_year1, 0.6f, 1.2f);
        changed = true;
        }
        show_item_hover_description(
          "Needle-base radius multiplier for first-year cohorts. 1.0 disables base taper.");

        if (ImGui::DragFloat("Year2+ Basal Taper Ratio", &needle_basal_taper_ratio_year2plus,
                   0.01f, 0.6f, 1.2f, "%.3f")) {
        needle_basal_taper_ratio_year2plus =
          std::clamp(needle_basal_taper_ratio_year2plus, 0.6f, 1.2f);
        changed = true;
        }
        show_item_hover_description(
          "Needle-base radius multiplier for year2+ cohorts. 1.0 disables base taper.");

        if (ImGui::DragFloat("Fascicle Sheath Budget (GDD)", &needle_fascicle_sheath_budget_gdd,
                   10.0f, 0.0f, 5000.0f, "%.1f")) {
        needle_fascicle_sheath_budget_gdd =
          std::max(0.0f, needle_fascicle_sheath_budget_gdd);
        changed = true;
        }
        show_item_hover_description(
          "Characteristic thermal budget for sheath maturation near needle bases.");

        if (ImGui::DragFloat("Year1 Specularity Plasticity", &needle_specularity_plasticity_year1,
                   0.01f, 0.0f, 1.0f, "%.3f")) {
        needle_specularity_plasticity_year1 =
          std::clamp(needle_specularity_plasticity_year1, 0.0f, 1.0f);
        changed = true;
        }
        show_item_hover_description(
          "How strongly first-year micro-variation tracks maturity cues.");

        if (ImGui::DragFloat("Year2+ Specularity Plasticity", &needle_specularity_plasticity_year2plus,
                   0.01f, 0.0f, 1.0f, "%.3f")) {
        needle_specularity_plasticity_year2plus =
          std::clamp(needle_specularity_plasticity_year2plus, 0.0f, 1.0f);
        changed = true;
        }
        show_item_hover_description(
          "How strongly year2+ micro-variation tracks maturity cues.");

      if (ImGui::DragFloat("Bud-Storage Vigor Strength", &needle_bud_storage_vigor_strength,
                           0.01f, 0.0f, 1.0f, "%.3f")) {
        needle_bud_storage_vigor_strength =
            std::clamp(needle_bud_storage_vigor_strength, 0.0f, 1.0f);
        changed = true;
      }
      show_item_hover_description(
          "Blend factor from 1.0 to previous-season vigor proxy for inter-year capacity.");

      if (ImGui::DragFloat("Bud-Storage Completion Floor", &needle_bud_storage_completion_floor,
                           0.01f, 0.0f, 1.0f, "%.3f")) {
        needle_bud_storage_completion_floor =
            std::clamp(needle_bud_storage_completion_floor, 0.0f, 1.0f);
        changed = true;
      }
      show_item_hover_description("Lower clamp applied to completion ratio before vigor carry-over.");
      ImGui::TreePop();
    }
    if (ImGui::DragFloat("[deprecated] Needle Width Cap (unused)",
                         &needle_radius_to_stem_thickness_max_ratio, 0.05f, 0.0f, 4.0f, "%.3f")) {
      needle_radius_to_stem_thickness_max_ratio =
          std::clamp(needle_radius_to_stem_thickness_max_ratio, 0.0f, 4.0f);
      changed = true;
    }
    show_item_hover_description(
        "Deprecated no-op. Needle width and thickness are uncapped and no "
        "longer tied to stem thickness.");
    if (ImGui::ColorEdit4("Needle Color", &needle_color_rgba.x)) {
      changed = true;
    }
    show_item_hover_description(
      "Young needle color used for newly flushed or low-age segments.");
    if (ImGui::ColorEdit4("Needle Old Color", &needle_old_color_rgba.x)) {
      changed = true;
    }
    show_item_hover_description(
      "Old needle color reached by high-age or strongly senescent segments.");
    if (ImGui::DragFloat("Needle Axial Age Span", &needle_axial_age_span, 0.01f, -1.0f, 1.0f, "%.3f")) {
      needle_axial_age_span = std::clamp(needle_axial_age_span, -1.0f, 1.0f);
      changed = true;
    }
    show_item_hover_description(
      "Along-needle age shift. Positive biases older color toward tip; negative biases older color toward base.");
    if (ImGui::DragFloat("Needle Axial Age Exponent", &needle_axial_age_exponent, 0.05f, 0.1f, 4.0f, "%.2f")) {
      needle_axial_age_exponent = std::clamp(needle_axial_age_exponent, 0.1f, 4.0f);
      changed = true;
    }
    show_item_hover_description(
      "Shape of along-needle gradient response. 1 = linear, >1 concentrates changes near one end.");
    ImGui::TreePop();
  }

  // -- Needle curvature (bilateral differential growth field) --
  if (ImGui::TreeNodeEx("Needle Shape (Curvature Field)")) {
    changed |= needle_curvature_adaxial_bias.OnInspect(
        "Adaxial Elongation Bias", 0.001f,
        "Dimensionless adaxial side elongation. Positive bends needle toward stem.", "%.4f");
    changed |= needle_curvature_abaxial_bias.OnInspect(
        "Abaxial Elongation Bias", 0.001f,
        "Dimensionless abaxial side elongation. Positive bends needle away from stem.", "%.4f");
    changed |= needle_curvature_gradient_per_arclen.OnInspect(
        "Curvature Gradient (per s_norm)", 0.001f,
        "Linear gradient added to (abaxial - adaxial) along normalized arc length.", "%.4f");
    changed |= needle_diameter_for_curvature_m.OnInspect(
        "Effective Diameter (m)", 0.0001f,
        "Cross-section diameter used to convert strain differential into curvature. "
        "Set > 0 to activate the field.", "%.6f");
    changed |= needle_sinusoidal_amplitude_deg.OnInspect(
      "Sinusoidal Wave Amplitude (deg)", 0.10f,
      "Additional intrinsic waviness amplitude applied along the needle; 0 keeps arc-only behavior.",
      "%.3f");
    changed |= needle_sinusoidal_frequency_cycles.OnInspect(
      "Sinusoidal Wave Frequency (cycles)", 0.05f,
      "Number of waviness cycles along full needle length.", "%.3f");
    changed |= needle_sinusoidal_phase_randomness_deg.OnInspect(
      "Sinusoidal Phase Randomness (deg)", 0.10f,
      "Sampled phase jitter magnitude combined with deterministic per-needle phase.", "%.3f");
    ImGui::TreePop();
  }

  // -- Needle mechanics (elastica) --
  if (ImGui::TreeNodeEx("Needle Mechanics (Elastica)")) {
    changed |= needle_young_modulus_baseline_Pa.OnInspect(
        "Young's Modulus Baseline (Pa)", 1e6f,
        "Asymptotic Young's modulus at maturity. 0 = solver disabled.", "%.0f");
    changed |= needle_lignification_maturation_years.OnInspect(
        "Lignification Maturation (yr)", 0.05f,
        "Sigmoid maturation duration for E(t).", "%.3f");
    changed |= needle_density_kg_m3.OnInspect(
        "Tissue Density (kg/m^3)", 10.0f,
        "Used to derive distributed weight per unit arc length.", "%.1f");
    changed |= gravity_m_s2.OnInspect(
        "Gravity (m/s^2)", 0.1f,
        "World-frame gravity magnitude. 0 = no body force.", "%.3f");
    ImGui::TreePop();
  }

    if (ImGui::TreeNodeEx("Needle Per-Needle Variability")) {
    changed |= needle_per_needle_length_cv.OnInspect(
      "Length CV", 0.01f,
      "CV-style variation across needles within a cluster for length scale.", "%.4f");
    changed |= needle_per_needle_curvature_cv.OnInspect(
      "Curvature CV", 0.01f,
      "CV-style variation across needles within a cluster for curvature-field magnitude.", "%.4f");
    changed |= needle_per_needle_radius_cv.OnInspect(
      "Radius CV", 0.01f,
      "CV-style variation across needles within a cluster for cross-section axis scale.", "%.4f");
    changed |= needle_per_needle_modulus_cv.OnInspect(
      "Young's Modulus CV", 0.01f,
      "CV-style variation across needles within a cluster for baseline Young's modulus.", "%.4f");
    changed |= needle_per_needle_density_cv.OnInspect(
      "Density CV", 0.01f,
      "CV-style variation across needles within a cluster for tissue density.", "%.4f");
    changed |= needle_per_needle_wave_amplitude_cv.OnInspect(
      "Wave Amplitude CV", 0.01f,
      "CV-style variation across needles within a cluster for sinusoidal waviness amplitude.", "%.4f");
    changed |= needle_per_needle_wave_frequency_cv.OnInspect(
      "Wave Frequency CV", 0.01f,
      "CV-style variation across needles within a cluster for sinusoidal waviness frequency.", "%.4f");
    changed |= needle_per_needle_wave_phase_cv.OnInspect(
      "Wave Phase CV", 0.01f,
      "CV-style scaling of per-needle sinusoidal phase randomness.", "%.4f");
    ImGui::TreePop();
    }

  // -- Tropism --
  if (ImGui::TreeNodeEx("Tropism (Global)")) {
    changed |= gravitropism_first_order.OnInspect("Main Stem Tropism (deg/GDD)", 0.0001f,
        "Per-GDD curvature applied to leader internodes only (branch order 0). Positive bends upward.", "%.6f");
    ImGui::TreePop();
  }

  // -- Per-shoot stochastic noise --
  if (ImGui::TreeNodeEx("Stochastic Variation (Per Shoot)")) {
    changed |= internode_length_per_node_cv.OnInspect(
        "Internode Length CV", 0.005f,
        "Per-internode Gaussian CV on phytomer length. 0 = deterministic.", "%.4f");
    changed |= internode_thickness_per_node_cv.OnInspect(
        "Internode Thickness CV", 0.005f,
        "Per-internode Gaussian CV on shoot thickness. 0 = deterministic.", "%.4f");
    changed |= branch_angle_per_node_sigma_deg.OnInspect(
        "Branch Angle Sigma (deg)", 0.5f,
        "Per-lateral additive Gaussian sigma on insertion angle.", "%.3f");
    changed |= roll_phyllotaxis_per_node_sigma_deg.OnInspect(
        "Roll Phyllotaxis Sigma (deg)", 0.5f,
        "Per-lateral additive Gaussian sigma on phyllotaxis roll.", "%.3f");
    ImGui::TreePop();
  }

  // ==============================================================================
  // Deprecated controls (no runtime effect).
  //
  // These fields are sampled and serialized, but no consumer in the current
  // pine growth path iterates them. They are kept declared per the workspace
  // policy ("never remove unused code unless explicitly told to"). The foldout
  // is collapsed by default so they stay out of the way.
  // ==============================================================================
  ImGui::Separator();
  if (ImGui::TreeNodeEx("Deprecated (no runtime effect)")) {
    ImGui::TextWrapped(
        "The field(s) below are serialized and sampled but have no consumer"
        " in the current pine growth path. Edits round-trip through YAML but"
        " do not affect generated geometry.");

    // -- Dynamic tropism array ([deprecated] for pine) --
    // The pine-side tropisms vector is sampled into SampledPineParams::tropisms
    // but no pine consumer iterates it. The Maize tassel side does iterate
    // its analogous vector (MaizeTasselRules.hpp), so the type stays alive.
    // The active stem tropism for pine is the scalar `gravitropism_first_order`
    // in the "Tropism (Global)" group above.
    if (ImGui::TreeNodeEx("Dynamic Tropisms  [deprecated]")) {
      ImGui::TextWrapped(
          "[deprecated] No pine consumer reads sampled.tropisms. Use"
          " \"Tropism (Global)\" -> Main Stem Tropism instead.");
      if (ImGui::Button("+ Add Tropism")) {
        tropisms.emplace_back();
        changed = true;
      }
      show_item_hover_description("Add a directional tropism entry. [deprecated] no runtime effect.");

      int remove_index = -1;
      for (size_t i = 0; i < tropisms.size(); ++i) {
        ImGui::PushID(static_cast<int>(i));
        const std::string label = "Tropism #" + std::to_string(i);
        if (ImGui::TreeNodeEx(label.c_str(), ImGuiTreeNodeFlags_DefaultOpen)) {
          auto& entry = tropisms[i];
          changed |= entry.direction_x.OnInspect("Direction X", 0.05f);
          changed |= entry.direction_y.OnInspect("Direction Y", 0.05f);
          changed |= entry.direction_z.OnInspect("Direction Z", 0.05f);
          changed |= entry.strength.OnInspect("Strength", 0.05f);
          if (ImGui::DragFloat("Usage Chance (%)", &entry.usage_chance_percent, 1.0f, 0.0f, 100.0f, "%.1f")) {
            entry.usage_chance_percent = std::clamp(entry.usage_chance_percent, 0.0f, 100.0f);
            changed = true;
          }
          changed |= entry.order_response.OnInspect("Order Response (vs branching order)");
          if (ImGui::Button("Remove")) remove_index = static_cast<int>(i);
          ImGui::TreePop();
        }
        ImGui::PopID();
      }
      if (remove_index >= 0) {
        tropisms.erase(tropisms.begin() + remove_index);
        changed = true;
      }
      ImGui::TreePop();
    }

    ImGui::TreePop();
  }

  if (editor_preferences_changed) {
    // Editor preferences are persisted via Serialize/Deserialize; they don't
    // mark the asset content "changed" for revision tracking.
  }

  return changed;
}

// ===========================================================================
// Serialize
// ===========================================================================
void ScotsPineDescriptor::Serialize(YAML::Emitter& out) const {
  // Phytomer scheduling.
  max_branching_order.Save("max_branching_order", out);
  plastochron_gdd.Save("plastochron_gdd", out);
  max_phytomers_per_seasonal_growth.Save("max_phytomers_per_seasonal_growth", out);

  // Whorl architecture.
  branches_per_whorl.Save("branches_per_whorl", out);
  whorl_dormancy_years.Save("whorl_dormancy_years", out);
  branch_insertion_angle_deg.Save("branch_insertion_angle_deg", out);
  branch_roll_phyllotaxis_deg.Save("branch_roll_phyllotaxis_deg", out);

  // Phytomer dimensions.
  internode_length_m.Save("internode_length_m", out);
  leader_internode_thickness_m.Save("leader_internode_thickness_m", out);
  // Alias for discoverability in .spine files.
  leader_internode_thickness_m.Save("main_stem_width_m", out);
  lateral_length_ratio.Save("lateral_length_ratio", out);
  lateral_thickness_ratio.Save("lateral_thickness_ratio", out);
  out << YAML::Key << "main_stem_color_rgba" << YAML::Value << main_stem_color_rgba;
  out << YAML::Key << "main_stem_old_color_rgba" << YAML::Value << main_stem_old_color_rgba;
  out << YAML::Key << "internode_age_exponent" << YAML::Value << internode_age_exponent;

  // Needles.
  bare_zone_fraction.Save("bare_zone_fraction", out);
  needle_count_per_cluster.Save("needle_count_per_cluster", out);
  out << YAML::Key << "needle_segment_count" << YAML::Value << needle_segment_count;
  needle_length_m.Save("needle_length_m", out);
  needle_lifespan_years.Save("needle_lifespan_years", out);
  needle_browning_years.Save("needle_browning_years", out);
  needle_flush_delay_gdd.Save("needle_flush_delay_gdd", out);
  internode_maturation_gdd.Save("internode_maturation_gdd", out);
  needle_maturation_gdd.Save("needle_maturation_gdd", out);
  needle_branching_angle_deg.Save("needle_branching_angle_deg", out);
  needle_branching_relax_gdd.Save("needle_branching_relax_gdd", out);
  internode_length_maturity_curve.Save("internode_length_maturity_curve", out);
  internode_width_maturity_curve.Save("internode_width_maturity_curve", out);
  needle_length_maturity_curve.Save("needle_length_maturity_curve", out);
  needle_cross_section_width_max_m.Save("needle_cross_section_width_max_m", out);
  needle_cross_section_thickness_max_m.Save("needle_cross_section_thickness_max_m", out);
  needle_cross_section_width_profile.Save("needle_cross_section_width_profile", out);
  needle_cross_section_thickness_profile.Save("needle_cross_section_thickness_profile", out);
  needle_cross_section_temporal_maturity_curve.Save("needle_cross_section_temporal_maturity_curve", out);
  out << YAML::Key << "needle_order_length_attenuation"
      << YAML::Value << needle_order_length_attenuation;
  out << YAML::Key << "needle_order_radius_attenuation"
      << YAML::Value << needle_order_radius_attenuation;
  out << YAML::Key << "needle_order_min_length_scale"
      << YAML::Value << needle_order_min_length_scale;
  out << YAML::Key << "needle_order_min_radius_scale"
      << YAML::Value << needle_order_min_radius_scale;
    out << YAML::Key << "needle_intra_year_base_ratio"
      << YAML::Value << needle_intra_year_base_ratio;
    out << YAML::Key << "needle_intra_year_sigmoid_steepness"
      << YAML::Value << needle_intra_year_sigmoid_steepness;
    out << YAML::Key << "needle_intra_year_sigmoid_midpoint_fraction"
      << YAML::Value << needle_intra_year_sigmoid_midpoint_fraction;
    out << YAML::Key << "needle_intra_year_late_decay_start_fraction"
      << YAML::Value << needle_intra_year_late_decay_start_fraction;
    out << YAML::Key << "needle_intra_year_late_decay_end_scale"
      << YAML::Value << needle_intra_year_late_decay_end_scale;
    out << YAML::Key << "needle_fascicular_start_year"
      << YAML::Value << needle_fascicular_start_year;
    out << YAML::Key << "needle_year2plus_length_multiplier"
      << YAML::Value << needle_year2plus_length_multiplier;
    out << YAML::Key << "needle_year2plus_width_multiplier"
      << YAML::Value << needle_year2plus_width_multiplier;
    out << YAML::Key << "needle_year2plus_thickness_multiplier"
      << YAML::Value << needle_year2plus_thickness_multiplier;
    out << YAML::Key << "needle_lignification_factor_year1"
      << YAML::Value << needle_lignification_factor_year1;
    out << YAML::Key << "needle_lignification_factor_year2plus"
      << YAML::Value << needle_lignification_factor_year2plus;
    out << YAML::Key << "needle_stomatal_strip_density_year1"
      << YAML::Value << needle_stomatal_strip_density_year1;
    out << YAML::Key << "needle_stomatal_strip_density_year2plus"
      << YAML::Value << needle_stomatal_strip_density_year2plus;
    out << YAML::Key << "needle_basal_taper_ratio_year1"
      << YAML::Value << needle_basal_taper_ratio_year1;
    out << YAML::Key << "needle_basal_taper_ratio_year2plus"
      << YAML::Value << needle_basal_taper_ratio_year2plus;
    out << YAML::Key << "needle_fascicle_sheath_budget_gdd"
      << YAML::Value << needle_fascicle_sheath_budget_gdd;
    out << YAML::Key << "needle_specularity_plasticity_year1"
      << YAML::Value << needle_specularity_plasticity_year1;
    out << YAML::Key << "needle_specularity_plasticity_year2plus"
      << YAML::Value << needle_specularity_plasticity_year2plus;
    out << YAML::Key << "needle_bud_storage_vigor_strength"
      << YAML::Value << needle_bud_storage_vigor_strength;
    out << YAML::Key << "needle_bud_storage_completion_floor"
      << YAML::Value << needle_bud_storage_completion_floor;
  out << YAML::Key << "needle_radius_to_stem_thickness_max_ratio"
      << YAML::Value << needle_radius_to_stem_thickness_max_ratio;
  out << YAML::Key << "needle_color_rgba" << YAML::Value << needle_color_rgba;
  out << YAML::Key << "needle_old_color_rgba" << YAML::Value << needle_old_color_rgba;
  out << YAML::Key << "needle_axial_age_span" << YAML::Value << needle_axial_age_span;
  out << YAML::Key << "needle_axial_age_exponent" << YAML::Value << needle_axial_age_exponent;

  // Needle curvature.
  needle_curvature_adaxial_bias.Save("needle_curvature_adaxial_bias", out);
  needle_curvature_abaxial_bias.Save("needle_curvature_abaxial_bias", out);
  needle_curvature_gradient_per_arclen.Save("needle_curvature_gradient_per_arclen", out);
  needle_diameter_for_curvature_m.Save("needle_diameter_for_curvature_m", out);
  needle_sinusoidal_amplitude_deg.Save("needle_sinusoidal_amplitude_deg", out);
  needle_sinusoidal_frequency_cycles.Save("needle_sinusoidal_frequency_cycles", out);
  needle_sinusoidal_phase_randomness_deg.Save("needle_sinusoidal_phase_randomness_deg", out);

  // Needle mechanics.
  needle_young_modulus_baseline_Pa.Save("needle_young_modulus_baseline_Pa", out);
  needle_lignification_maturation_years.Save("needle_lignification_maturation_years", out);
  needle_density_kg_m3.Save("needle_density_kg_m3", out);
  gravity_m_s2.Save("gravity_m_s2", out);
  needle_per_needle_length_cv.Save("needle_per_needle_length_cv", out);
  needle_per_needle_curvature_cv.Save("needle_per_needle_curvature_cv", out);
  needle_per_needle_radius_cv.Save("needle_per_needle_radius_cv", out);
  needle_per_needle_modulus_cv.Save("needle_per_needle_modulus_cv", out);
  needle_per_needle_density_cv.Save("needle_per_needle_density_cv", out);
  needle_per_needle_wave_amplitude_cv.Save("needle_per_needle_wave_amplitude_cv", out);
  needle_per_needle_wave_frequency_cv.Save("needle_per_needle_wave_frequency_cv", out);
  needle_per_needle_wave_phase_cv.Save("needle_per_needle_wave_phase_cv", out);

  // Tropism.
  gravitropism_first_order.Save("gravitropism_first_order", out);
  initial_orientation_yaw_deg.Save("initial_orientation_yaw_deg", out);

  // Per-instance target.
  target_gdd.Save("target_gdd", out);
  gdd_per_day.Save("gdd_per_day", out);
  growing_season_start_day.Save("growing_season_start_day", out);
  growing_season_end_day.Save("growing_season_end_day", out);

  // Per-shoot stochastic noise.
  internode_length_per_node_cv.Save("internode_length_per_node_cv", out);
  internode_thickness_per_node_cv.Save("internode_thickness_per_node_cv", out);
  branch_angle_per_node_sigma_deg.Save("branch_angle_per_node_sigma_deg", out);
  roll_phyllotaxis_per_node_sigma_deg.Save("roll_phyllotaxis_per_node_sigma_deg", out);

  // Editor preferences.
  out << YAML::Key << "live_preview" << YAML::Value << live_preview;
  out << YAML::Key << "live_preview_rate_hz" << YAML::Value << live_preview_rate_hz;
  out << YAML::Key << "live_preview_representative_only" << YAML::Value
      << live_preview_representative_only;
  out << YAML::Key << "live_preview_cap_target_gdd" << YAML::Value << live_preview_cap_target_gdd;
  out << YAML::Key << "live_preview_max_gdd" << YAML::Value << live_preview_max_gdd;
  out << YAML::Key << "live_preview_max_growth_steps" << YAML::Value << live_preview_max_growth_steps;
  out << YAML::Key << "grid_rows" << YAML::Value << grid_rows;
  out << YAML::Key << "grid_cols" << YAML::Value << grid_cols;
  out << YAML::Key << "grid_spacing" << YAML::Value << grid_spacing;
  out << YAML::Key << "triangle_side_length" << YAML::Value << triangle_side_length;

  // Tropism array.
  out << YAML::Key << "tropism_count" << YAML::Value << static_cast<int>(tropisms.size());
  for (size_t i = 0; i < tropisms.size(); ++i) {
    const std::string prefix = "tropism_" + std::to_string(i) + "_";
    const auto& entry = tropisms[i];
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
void ScotsPineDescriptor::Deserialize(const YAML::Node& in) {
  ConfigurePineMaturityDefaults(*this);

  LoadSingleDistributionWithScalarFallback(in, "max_branching_order", max_branching_order);
  LoadSingleDistributionWithScalarFallback(in, "plastochron_gdd", plastochron_gdd);
  LoadSingleDistributionWithScalarFallback(in, "max_phytomers_per_seasonal_growth",
                                           max_phytomers_per_seasonal_growth);

  LoadSingleDistributionWithScalarFallback(in, "branches_per_whorl", branches_per_whorl);
  // Clock-rule fix: prefer years key; fall back to legacy GDD key with /1500.
  if (in["whorl_dormancy_years"]) {
    LoadSingleDistributionWithScalarFallback(in, "whorl_dormancy_years", whorl_dormancy_years);
  } else {
    LoadLegacyGddDistributionAsYears(in, "whorl_dormancy_gdd", whorl_dormancy_years);
  }
  LoadSingleDistributionWithScalarFallback(in, "branch_insertion_angle_deg", branch_insertion_angle_deg);
  LoadSingleDistributionWithScalarFallback(in, "branch_roll_phyllotaxis_deg",
                                           branch_roll_phyllotaxis_deg);

  LoadSingleDistributionWithScalarFallback(in, "internode_length_m", internode_length_m);
  LoadSingleDistributionWithScalarFallback(in, "leader_internode_thickness_m",
                                           leader_internode_thickness_m);
  // Backward/forward alias support.
  LoadSingleDistributionWithScalarFallback(in, "main_stem_width_m",
                                           leader_internode_thickness_m);
  LoadSingleDistributionWithScalarFallback(in, "lateral_length_ratio", lateral_length_ratio);
  LoadSingleDistributionWithScalarFallback(in, "lateral_thickness_ratio", lateral_thickness_ratio);
  if (in["main_stem_color_rgba"]) {
    main_stem_color_rgba = in["main_stem_color_rgba"].as<glm::vec4>();
  }
  if (in["main_stem_old_color_rgba"]) {
    main_stem_old_color_rgba = in["main_stem_old_color_rgba"].as<glm::vec4>();
  }
  if (in["internode_age_exponent"]) {
    internode_age_exponent = std::clamp(in["internode_age_exponent"].as<float>(), 0.1f, 4.0f);
  }

  LoadSingleDistributionWithScalarFallback(in, "bare_zone_fraction", bare_zone_fraction);
  LoadSingleDistributionWithScalarFallback(in, "needle_count_per_cluster", needle_count_per_cluster);
  if (in["needle_segment_count"]) {
    needle_segment_count = std::clamp(in["needle_segment_count"].as<int>(), 3, 128);
  }
  LoadSingleDistributionWithScalarFallback(in, "needle_length_m", needle_length_m);
  // Clock-rule fix: prefer years keys; fall back to legacy GDD keys with /1500.
  if (in["needle_lifespan_years"]) {
    LoadSingleDistributionWithScalarFallback(in, "needle_lifespan_years", needle_lifespan_years);
  } else {
    LoadLegacyGddDistributionAsYears(in, "needle_lifespan_gdd", needle_lifespan_years);
  }
  if (in["needle_browning_years"]) {
    LoadSingleDistributionWithScalarFallback(in, "needle_browning_years", needle_browning_years);
  } else {
    LoadLegacyGddDistributionAsYears(in, "needle_browning_gdd", needle_browning_years);
  }
  LoadSingleDistributionWithScalarFallback(in, "needle_flush_delay_gdd", needle_flush_delay_gdd);
  LoadSingleDistributionWithScalarFallback(in, "internode_maturation_gdd", internode_maturation_gdd);
  LoadSingleDistributionWithScalarFallback(in, "needle_maturation_gdd", needle_maturation_gdd);
  LoadSingleDistributionWithScalarFallback(in, "needle_branching_angle_deg", needle_branching_angle_deg);
  LoadSingleDistributionWithScalarFallback(in, "needle_branching_relax_gdd", needle_branching_relax_gdd);
  const bool has_new_cross_section_width_max = static_cast<bool>(in["needle_cross_section_width_max_m"]);
  const bool has_new_cross_section_thickness_max = static_cast<bool>(in["needle_cross_section_thickness_max_m"]);
  const bool has_new_cross_section_width_profile = static_cast<bool>(in["needle_cross_section_width_profile"]);
  const bool has_new_cross_section_thickness_profile = static_cast<bool>(in["needle_cross_section_thickness_profile"]);
  internode_length_maturity_curve.Load("internode_length_maturity_curve", in);
  internode_width_maturity_curve.Load("internode_width_maturity_curve", in);
  needle_length_maturity_curve.Load("needle_length_maturity_curve", in);
  LoadSingleDistributionWithScalarFallback(in, "needle_cross_section_width_max_m",
                                           needle_cross_section_width_max_m);
  LoadSingleDistributionWithScalarFallback(in, "needle_cross_section_thickness_max_m",
                                           needle_cross_section_thickness_max_m);
  needle_cross_section_width_profile.Load("needle_cross_section_width_profile", in);
  needle_cross_section_thickness_profile.Load("needle_cross_section_thickness_profile", in);
  needle_cross_section_temporal_maturity_curve.Load("needle_cross_section_temporal_maturity_curve", in);

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
      needle_cross_section_width_max_m.mean = legacy_width_max_mean_m;
      needle_cross_section_width_max_m.deviation = legacy_width_max_dev_m;
    }
    if (!has_new_cross_section_thickness_max) {
      const float fallback_thickness_mean_m = (legacy_tip_diameter_mean_m > 0.0f)
          ? legacy_tip_diameter_mean_m
          : legacy_width_max_mean_m;
      const float fallback_thickness_dev_m = (legacy_tip_diameter_dev_m > 0.0f)
          ? legacy_tip_diameter_dev_m
          : legacy_width_max_dev_m;
      needle_cross_section_thickness_max_m.mean = fallback_thickness_mean_m;
      needle_cross_section_thickness_max_m.deviation = fallback_thickness_dev_m;
    }
  }

  float legacy_tip_taper_ratio = 0.36f;
  if (in["needle_simple_tip_taper_ratio"]) {
    legacy_tip_taper_ratio =
        std::clamp(in["needle_simple_tip_taper_ratio"].as<float>(), 0.0f, 1.0f);
  }
  if (!has_new_cross_section_width_profile) {
    ConfigureNeedleCrossSectionProfileDefaults(needle_cross_section_width_profile,
                                               legacy_tip_taper_ratio);
  }
  if (!has_new_cross_section_thickness_profile) {
    ConfigureNeedleCrossSectionProfileDefaults(needle_cross_section_thickness_profile,
                                               legacy_tip_taper_ratio);
  }

  needle_cross_section_width_max_m.mean = std::max(0.0f, needle_cross_section_width_max_m.mean);
  needle_cross_section_width_max_m.deviation =
      std::max(0.0f, needle_cross_section_width_max_m.deviation);
    needle_cross_section_thickness_max_m.mean =
      std::max(0.0f, needle_cross_section_thickness_max_m.mean);
  needle_cross_section_thickness_max_m.deviation =
      std::max(0.0f, needle_cross_section_thickness_max_m.deviation);
  auto clamp_plot_range = [](evo_engine::Plot2D<float>& plot, const float max_value) {
    plot.min_value = std::clamp(plot.min_value, 0.0f, 1.0f);
    plot.max_value = std::clamp(plot.max_value, 0.0f, max_value);
    if (plot.max_value < plot.min_value) {
      std::swap(plot.min_value, plot.max_value);
    }
  };
  clamp_plot_range(needle_cross_section_width_profile.mean, 4.0f);
  clamp_plot_range(needle_cross_section_width_profile.deviation, 4.0f);
  clamp_plot_range(needle_cross_section_thickness_profile.mean, 4.0f);
  clamp_plot_range(needle_cross_section_thickness_profile.deviation, 4.0f);
  clamp_plot_range(needle_cross_section_temporal_maturity_curve.mean, 1.0f);
  clamp_plot_range(needle_cross_section_temporal_maturity_curve.deviation, 1.0f);
  if (in["needle_order_length_attenuation"]) {
    needle_order_length_attenuation =
        std::clamp(in["needle_order_length_attenuation"].as<float>(), 0.0f, 1.0f);
  }
  if (in["needle_order_radius_attenuation"]) {
    needle_order_radius_attenuation =
        std::clamp(in["needle_order_radius_attenuation"].as<float>(), 0.0f, 1.0f);
  }
  if (in["needle_order_min_length_scale"]) {
    needle_order_min_length_scale =
        std::clamp(in["needle_order_min_length_scale"].as<float>(), 0.10f, 1.00f);
  }
  if (in["needle_order_min_radius_scale"]) {
    needle_order_min_radius_scale =
        std::clamp(in["needle_order_min_radius_scale"].as<float>(), 0.10f, 1.00f);
  }
  if (in["needle_intra_year_base_ratio"]) {
    needle_intra_year_base_ratio =
        std::clamp(in["needle_intra_year_base_ratio"].as<float>(), 0.0f, 1.0f);
  }
  if (in["needle_intra_year_sigmoid_steepness"]) {
    needle_intra_year_sigmoid_steepness =
        std::max(0.01f, in["needle_intra_year_sigmoid_steepness"].as<float>());
  }
  if (in["needle_intra_year_sigmoid_midpoint_fraction"]) {
    needle_intra_year_sigmoid_midpoint_fraction = std::clamp(
        in["needle_intra_year_sigmoid_midpoint_fraction"].as<float>(), 0.0f, 1.0f);
  }
  if (in["needle_intra_year_late_decay_start_fraction"]) {
    needle_intra_year_late_decay_start_fraction = std::clamp(
        in["needle_intra_year_late_decay_start_fraction"].as<float>(), 0.0f, 1.0f);
  }
  if (in["needle_intra_year_late_decay_end_scale"]) {
    needle_intra_year_late_decay_end_scale =
        std::clamp(in["needle_intra_year_late_decay_end_scale"].as<float>(), 0.0f, 2.0f);
  }
  if (in["needle_fascicular_start_year"]) {
    needle_fascicular_start_year =
        std::clamp(in["needle_fascicular_start_year"].as<int>(), 0, 16);
  }
  if (in["needle_year2plus_length_multiplier"]) {
    needle_year2plus_length_multiplier =
        std::max(0.0f, in["needle_year2plus_length_multiplier"].as<float>());
  }
  if (in["needle_year2plus_width_multiplier"]) {
    needle_year2plus_width_multiplier =
        std::max(0.0f, in["needle_year2plus_width_multiplier"].as<float>());
  }
  if (in["needle_year2plus_thickness_multiplier"]) {
    needle_year2plus_thickness_multiplier =
        std::max(0.0f, in["needle_year2plus_thickness_multiplier"].as<float>());
  }
  if (in["needle_lignification_factor_year1"]) {
    needle_lignification_factor_year1 =
        std::clamp(in["needle_lignification_factor_year1"].as<float>(), 0.0f, 2.0f);
  }
  if (in["needle_lignification_factor_year2plus"]) {
    needle_lignification_factor_year2plus =
        std::clamp(in["needle_lignification_factor_year2plus"].as<float>(), 0.0f, 2.0f);
  }
  if (in["needle_stomatal_strip_density_year1"]) {
    needle_stomatal_strip_density_year1 = std::clamp(
        in["needle_stomatal_strip_density_year1"].as<float>(), 0.0f, 1.0f);
  }
  if (in["needle_stomatal_strip_density_year2plus"]) {
    needle_stomatal_strip_density_year2plus = std::clamp(
        in["needle_stomatal_strip_density_year2plus"].as<float>(), 0.0f, 1.0f);
  }
  if (in["needle_basal_taper_ratio_year1"]) {
    needle_basal_taper_ratio_year1 =
        std::clamp(in["needle_basal_taper_ratio_year1"].as<float>(), 0.6f, 1.2f);
  }
  if (in["needle_basal_taper_ratio_year2plus"]) {
    needle_basal_taper_ratio_year2plus =
        std::clamp(in["needle_basal_taper_ratio_year2plus"].as<float>(), 0.6f, 1.2f);
  }
  if (in["needle_fascicle_sheath_budget_gdd"]) {
    needle_fascicle_sheath_budget_gdd =
        std::max(0.0f, in["needle_fascicle_sheath_budget_gdd"].as<float>());
  }
  if (in["needle_specularity_plasticity_year1"]) {
    needle_specularity_plasticity_year1 = std::clamp(
        in["needle_specularity_plasticity_year1"].as<float>(), 0.0f, 1.0f);
  }
  if (in["needle_specularity_plasticity_year2plus"]) {
    needle_specularity_plasticity_year2plus = std::clamp(
        in["needle_specularity_plasticity_year2plus"].as<float>(), 0.0f, 1.0f);
  }
  if (in["needle_bud_storage_vigor_strength"]) {
    needle_bud_storage_vigor_strength =
        std::clamp(in["needle_bud_storage_vigor_strength"].as<float>(), 0.0f, 1.0f);
  }
  if (in["needle_bud_storage_completion_floor"]) {
    needle_bud_storage_completion_floor =
        std::clamp(in["needle_bud_storage_completion_floor"].as<float>(), 0.0f, 1.0f);
  }
  if (in["needle_radius_to_stem_thickness_max_ratio"]) {
    needle_radius_to_stem_thickness_max_ratio =
        std::clamp(in["needle_radius_to_stem_thickness_max_ratio"].as<float>(), 0.0f, 4.0f);
  }
  if (in["needle_color_rgba"]) {
    needle_color_rgba = in["needle_color_rgba"].as<glm::vec4>();
  }
  if (in["needle_old_color_rgba"]) {
    needle_old_color_rgba = in["needle_old_color_rgba"].as<glm::vec4>();
  }
  if (in["needle_axial_age_span"]) {
    needle_axial_age_span = std::clamp(in["needle_axial_age_span"].as<float>(), -1.0f, 1.0f);
  }
  if (in["needle_axial_age_exponent"]) {
    needle_axial_age_exponent = std::clamp(in["needle_axial_age_exponent"].as<float>(), 0.1f, 4.0f);
  }

  LoadSingleDistributionWithScalarFallback(in, "needle_curvature_adaxial_bias",
                                           needle_curvature_adaxial_bias);
  LoadSingleDistributionWithScalarFallback(in, "needle_curvature_abaxial_bias",
                                           needle_curvature_abaxial_bias);
  LoadSingleDistributionWithScalarFallback(in, "needle_curvature_gradient_per_arclen",
                                           needle_curvature_gradient_per_arclen);
  LoadSingleDistributionWithScalarFallback(in, "needle_diameter_for_curvature_m",
                                           needle_diameter_for_curvature_m);
  LoadSingleDistributionWithScalarFallback(in, "needle_sinusoidal_amplitude_deg",
                                           needle_sinusoidal_amplitude_deg);
  LoadSingleDistributionWithScalarFallback(in, "needle_sinusoidal_frequency_cycles",
                                           needle_sinusoidal_frequency_cycles);
  LoadSingleDistributionWithScalarFallback(in, "needle_sinusoidal_phase_randomness_deg",
                                           needle_sinusoidal_phase_randomness_deg);

  LoadSingleDistributionWithScalarFallback(in, "needle_young_modulus_baseline_Pa",
                                           needle_young_modulus_baseline_Pa);
  LoadSingleDistributionWithScalarFallback(in, "needle_lignification_maturation_years",
                                           needle_lignification_maturation_years);
  LoadSingleDistributionWithScalarFallback(in, "needle_density_kg_m3", needle_density_kg_m3);
  LoadSingleDistributionWithScalarFallback(in, "gravity_m_s2", gravity_m_s2);
  LoadSingleDistributionWithScalarFallback(in, "needle_per_needle_length_cv",
                                           needle_per_needle_length_cv);
  LoadSingleDistributionWithScalarFallback(in, "needle_per_needle_curvature_cv",
                                           needle_per_needle_curvature_cv);
  LoadSingleDistributionWithScalarFallback(in, "needle_per_needle_radius_cv",
                                           needle_per_needle_radius_cv);
  LoadSingleDistributionWithScalarFallback(in, "needle_per_needle_modulus_cv",
                                           needle_per_needle_modulus_cv);
  LoadSingleDistributionWithScalarFallback(in, "needle_per_needle_density_cv",
                                           needle_per_needle_density_cv);
  LoadSingleDistributionWithScalarFallback(in, "needle_per_needle_wave_amplitude_cv",
                                           needle_per_needle_wave_amplitude_cv);
  LoadSingleDistributionWithScalarFallback(in, "needle_per_needle_wave_frequency_cv",
                                           needle_per_needle_wave_frequency_cv);
  LoadSingleDistributionWithScalarFallback(in, "needle_per_needle_wave_phase_cv",
                                           needle_per_needle_wave_phase_cv);

  LoadSingleDistributionWithScalarFallback(in, "gravitropism_first_order", gravitropism_first_order);
  LoadSingleDistributionWithScalarFallback(in, "initial_orientation_yaw_deg",
                                           initial_orientation_yaw_deg);

  LoadSingleDistributionWithScalarFallback(in, "target_gdd", target_gdd);
    LoadSingleDistributionWithScalarFallback(in, "gdd_per_day", gdd_per_day);
    LoadSingleDistributionWithScalarFallback(in, "growing_season_start_day", growing_season_start_day);
    LoadSingleDistributionWithScalarFallback(in, "growing_season_end_day", growing_season_end_day);

    gdd_per_day.mean = std::max(0.0f, gdd_per_day.mean);
    gdd_per_day.deviation = std::max(0.0f, gdd_per_day.deviation);
    growing_season_start_day.mean = std::clamp(growing_season_start_day.mean, 0.0f, 365.0f);
    growing_season_start_day.deviation =
      std::max(0.0f, std::round(growing_season_start_day.deviation));
    growing_season_end_day.mean = std::clamp(growing_season_end_day.mean, 0.0f, 365.0f);
    growing_season_end_day.deviation =
      std::max(0.0f, std::round(growing_season_end_day.deviation));

  LoadSingleDistributionWithScalarFallback(in, "internode_length_per_node_cv",
                                           internode_length_per_node_cv);
  LoadSingleDistributionWithScalarFallback(in, "internode_thickness_per_node_cv",
                                           internode_thickness_per_node_cv);
  LoadSingleDistributionWithScalarFallback(in, "branch_angle_per_node_sigma_deg",
                                           branch_angle_per_node_sigma_deg);
  LoadSingleDistributionWithScalarFallback(in, "roll_phyllotaxis_per_node_sigma_deg",
                                           roll_phyllotaxis_per_node_sigma_deg);

  if (in["live_preview"]) live_preview = in["live_preview"].as<bool>();
  if (in["live_preview_rate_hz"]) live_preview_rate_hz = in["live_preview_rate_hz"].as<float>();
  if (in["live_preview_representative_only"])
    live_preview_representative_only = in["live_preview_representative_only"].as<bool>();
  if (in["live_preview_cap_target_gdd"])
    live_preview_cap_target_gdd = in["live_preview_cap_target_gdd"].as<bool>();
  if (in["live_preview_max_gdd"])
    live_preview_max_gdd = in["live_preview_max_gdd"].as<float>();
  if (in["live_preview_max_growth_steps"])
    live_preview_max_growth_steps = in["live_preview_max_growth_steps"].as<int>();
  if (in["grid_rows"]) grid_rows = in["grid_rows"].as<int>();
  if (in["grid_cols"]) grid_cols = in["grid_cols"].as<int>();
  if (in["grid_spacing"]) grid_spacing = in["grid_spacing"].as<float>();
  if (in["triangle_side_length"])
    triangle_side_length = in["triangle_side_length"].as<float>();

  tropisms.clear();
  if (in["tropism_count"]) {
    const int count = std::max(0, in["tropism_count"].as<int>());
    tropisms.reserve(count);
    for (int i = 0; i < count; ++i) {
      const std::string prefix = "tropism_" + std::to_string(i) + "_";
      TropismEntry entry;
      LoadSingleDistributionWithScalarFallback(in, (prefix + "dir_x").c_str(), entry.direction_x);
      LoadSingleDistributionWithScalarFallback(in, (prefix + "dir_y").c_str(), entry.direction_y);
      LoadSingleDistributionWithScalarFallback(in, (prefix + "dir_z").c_str(), entry.direction_z);
      LoadSingleDistributionWithScalarFallback(in, (prefix + "strength").c_str(), entry.strength);
      const std::string usage_key = prefix + "usage_chance_percent";
      if (in[usage_key]) entry.usage_chance_percent = in[usage_key].as<float>();
      entry.order_response.Load(prefix + "order_response", in);
      tropisms.emplace_back(std::move(entry));
    }
  }
}

// ===========================================================================
// ParamSpaceExplorer axis registration.
// ===========================================================================
void ScotsPineDescriptor::RegisterExplorableAxes(ParamSpaceExplorer& explorer) {
  auto& d = *this;

  // -- Phytomer scheduling --
  explorer.AddSingle("max_branching_order", "MBO", d.max_branching_order, 0.0f, 4.0f, 2.0f);
  explorer.AddSingle("plastochron_gdd", "PLG", d.plastochron_gdd, 50.0f, 6000.0f, 1500.0f);
  explorer.AddSingle("max_phytomers_per_seasonal_growth", "MPS",
                     d.max_phytomers_per_seasonal_growth, 1.0f, 64.0f, 12.0f);

  // -- Whorl architecture --
  explorer.AddSingle("branches_per_whorl", "BPW", d.branches_per_whorl, 0.0f, 10.0f, 5.0f);
  explorer.AddSingle("whorl_dormancy_years", "WDY", d.whorl_dormancy_years, 0.0f, 4.0f, 1.0f);
  explorer.AddSingle("branch_insertion_angle_deg", "BIA", d.branch_insertion_angle_deg,
                     -85.0f, 85.0f, 60.0f);
  explorer.AddSingle("branch_roll_phyllotaxis_deg", "BRP", d.branch_roll_phyllotaxis_deg,
                     0.0f, 360.0f, 137.5f);

  // -- Phytomer dimensions --
  explorer.AddSingle("internode_length_m", "ILM", d.internode_length_m,
                     0.0001f, 0.500f, 0.012f);
  explorer.AddSingle("main_stem_width_m", "MSW", d.leader_internode_thickness_m,
                     0.0001f, 0.0500f, 0.0030f);
  explorer.AddSingle("lateral_length_ratio", "LLR", d.lateral_length_ratio, 0.1f, 1.5f, 0.7f);
  explorer.AddSingle("lateral_thickness_ratio", "LTR", d.lateral_thickness_ratio, 0.1f, 1.5f, 0.6f);

  // -- Needles --
  explorer.AddSingle("bare_zone_fraction", "BZF", d.bare_zone_fraction, 0.0f, 0.95f, 0.0f);
  explorer.AddSingle("needle_count_per_cluster", "NCC", d.needle_count_per_cluster,
                     1.0f, 6.0f, 2.0f);
  {
    auto* value_ptr = &d.needle_segment_count;
    explorer.AddAxis("needle_segment_count", "NSG", 3.0f, 128.0f,
                     [value_ptr]() {
                       return static_cast<float>(*value_ptr);
                     },
                     [value_ptr](float value) {
                       *value_ptr =
                           std::clamp(static_cast<int>(std::round(value)), 3, 128);
                     });
  }
  explorer.AddSingle("needle_length_m", "NLM", d.needle_length_m, 0.001f, 0.200f, 0.025f);
  explorer.AddSingle("needle_lifespan_years", "NLY", d.needle_lifespan_years,
                     0.0f, 10.0f, 4.0f);
  explorer.AddSingle("needle_browning_years", "NBY", d.needle_browning_years,
                     0.0f, 4.0f, 1.0f);
  explorer.AddSingle("needle_flush_delay_gdd", "NFD", d.needle_flush_delay_gdd,
                     0.0f, 3000.0f, 0.0f);
  explorer.AddSingle("internode_maturation_gdd", "IMG", d.internode_maturation_gdd,
                     0.0f, 3000.0f, 60.0f);
  explorer.AddSingle("needle_maturation_gdd", "NMG", d.needle_maturation_gdd,
                     0.0f, 6000.0f, 120.0f);
  explorer.AddSingle("needle_branching_angle_deg", "NBA", d.needle_branching_angle_deg,
                     0.0f, 89.5f, 72.0f);
  explorer.AddSingle("needle_branching_relax_gdd", "NRG", d.needle_branching_relax_gdd,
                     0.0f, 6000.0f, 220.0f);
  explorer.AddPlotted("internode_length_maturity_curve", "ILC",
                      d.internode_length_maturity_curve);
  explorer.AddPlotted("internode_width_maturity_curve", "IWC",
                      d.internode_width_maturity_curve);
  explorer.AddPlotted("needle_length_maturity_curve", "NLC",
                      d.needle_length_maturity_curve);
  explorer.AddSingle("needle_cross_section_width_max_m", "NCW",
                     d.needle_cross_section_width_max_m, 0.0f, 0.02f, 0.0018f);
  explorer.AddSingle("needle_cross_section_thickness_max_m", "NCT",
                     d.needle_cross_section_thickness_max_m, 0.0f, 0.02f, 0.0011f);
  explorer.AddPlotted("needle_cross_section_width_profile", "NWP",
                      d.needle_cross_section_width_profile);
  explorer.AddPlotted("needle_cross_section_thickness_profile", "NTP",
                      d.needle_cross_section_thickness_profile);
  explorer.AddPlotted("needle_cross_section_temporal_maturity_curve", "NTM",
                      d.needle_cross_section_temporal_maturity_curve);
  {
    auto* value_ptr = &d.needle_intra_year_base_ratio;
    explorer.AddAxis("needle_intra_year_base_ratio", "NIB", 0.0f, 1.0f,
                     [value_ptr]() { return *value_ptr; },
                     [value_ptr](float value) {
                       *value_ptr = std::clamp(value, 0.0f, 1.0f);
                     });
  }
  {
    auto* value_ptr = &d.needle_intra_year_sigmoid_steepness;
    explorer.AddAxis("needle_intra_year_sigmoid_steepness", "NIS", 0.01f, 32.0f,
                     [value_ptr]() { return *value_ptr; },
                     [value_ptr](float value) {
                       *value_ptr = std::max(0.01f, value);
                     });
  }
  {
    auto* value_ptr = &d.needle_intra_year_sigmoid_midpoint_fraction;
    explorer.AddAxis("needle_intra_year_sigmoid_midpoint_fraction", "NIM", 0.0f, 1.0f,
                     [value_ptr]() { return *value_ptr; },
                     [value_ptr](float value) {
                       *value_ptr = std::clamp(value, 0.0f, 1.0f);
                     });
  }
  {
    auto* value_ptr = &d.needle_intra_year_late_decay_start_fraction;
    explorer.AddAxis("needle_intra_year_late_decay_start_fraction", "NDS", 0.0f, 1.0f,
                     [value_ptr]() { return *value_ptr; },
                     [value_ptr](float value) {
                       *value_ptr = std::clamp(value, 0.0f, 1.0f);
                     });
  }
  {
    auto* value_ptr = &d.needle_intra_year_late_decay_end_scale;
    explorer.AddAxis("needle_intra_year_late_decay_end_scale", "NDE", 0.0f, 2.0f,
                     [value_ptr]() { return *value_ptr; },
                     [value_ptr](float value) {
                       *value_ptr = std::clamp(value, 0.0f, 2.0f);
                     });
  }
  {
    auto* value_ptr = &d.needle_fascicular_start_year;
    explorer.AddAxis("needle_fascicular_start_year", "NFY", 0.0f, 16.0f,
                     [value_ptr]() {
                       return static_cast<float>(*value_ptr);
                     },
                     [value_ptr](float value) {
                       *value_ptr = std::clamp(static_cast<int>(std::round(value)), 0, 16);
                     });
  }
  {
    auto* value_ptr = &d.needle_year2plus_length_multiplier;
    explorer.AddAxis("needle_year2plus_length_multiplier", "N2L", 0.0f, 8.0f,
                     [value_ptr]() { return *value_ptr; },
                     [value_ptr](float value) {
                       *value_ptr = std::max(0.0f, value);
                     });
  }
  {
    auto* value_ptr = &d.needle_year2plus_width_multiplier;
    explorer.AddAxis("needle_year2plus_width_multiplier", "N2W", 0.0f, 8.0f,
                     [value_ptr]() { return *value_ptr; },
                     [value_ptr](float value) {
                       *value_ptr = std::max(0.0f, value);
                     });
  }
  {
    auto* value_ptr = &d.needle_year2plus_thickness_multiplier;
    explorer.AddAxis("needle_year2plus_thickness_multiplier", "N2T", 0.0f, 8.0f,
                     [value_ptr]() { return *value_ptr; },
                     [value_ptr](float value) {
                       *value_ptr = std::max(0.0f, value);
                     });
  }
  {
    auto* value_ptr = &d.needle_lignification_factor_year1;
    explorer.AddAxis("needle_lignification_factor_year1", "NL1", 0.0f, 2.0f,
                     [value_ptr]() { return *value_ptr; },
                     [value_ptr](float value) {
                       *value_ptr = std::clamp(value, 0.0f, 2.0f);
                     });
  }
  {
    auto* value_ptr = &d.needle_lignification_factor_year2plus;
    explorer.AddAxis("needle_lignification_factor_year2plus", "NL2", 0.0f, 2.0f,
                     [value_ptr]() { return *value_ptr; },
                     [value_ptr](float value) {
                       *value_ptr = std::clamp(value, 0.0f, 2.0f);
                     });
  }
  {
    auto* value_ptr = &d.needle_stomatal_strip_density_year1;
    explorer.AddAxis("needle_stomatal_strip_density_year1", "NS1", 0.0f, 1.0f,
                     [value_ptr]() { return *value_ptr; },
                     [value_ptr](float value) {
                       *value_ptr = std::clamp(value, 0.0f, 1.0f);
                     });
  }
  {
    auto* value_ptr = &d.needle_stomatal_strip_density_year2plus;
    explorer.AddAxis("needle_stomatal_strip_density_year2plus", "NS2", 0.0f, 1.0f,
                     [value_ptr]() { return *value_ptr; },
                     [value_ptr](float value) {
                       *value_ptr = std::clamp(value, 0.0f, 1.0f);
                     });
  }
  {
    auto* value_ptr = &d.needle_basal_taper_ratio_year1;
    explorer.AddAxis("needle_basal_taper_ratio_year1", "NB1", 0.6f, 1.2f,
                     [value_ptr]() { return *value_ptr; },
                     [value_ptr](float value) {
                       *value_ptr = std::clamp(value, 0.6f, 1.2f);
                     });
  }
  {
    auto* value_ptr = &d.needle_basal_taper_ratio_year2plus;
    explorer.AddAxis("needle_basal_taper_ratio_year2plus", "NB2", 0.6f, 1.2f,
                     [value_ptr]() { return *value_ptr; },
                     [value_ptr](float value) {
                       *value_ptr = std::clamp(value, 0.6f, 1.2f);
                     });
  }
  {
    auto* value_ptr = &d.needle_fascicle_sheath_budget_gdd;
    explorer.AddAxis("needle_fascicle_sheath_budget_gdd", "NSB", 0.0f, 5000.0f,
                     [value_ptr]() { return *value_ptr; },
                     [value_ptr](float value) {
                       *value_ptr = std::max(0.0f, value);
                     });
  }
  {
    auto* value_ptr = &d.needle_specularity_plasticity_year1;
    explorer.AddAxis("needle_specularity_plasticity_year1", "NP1", 0.0f, 1.0f,
                     [value_ptr]() { return *value_ptr; },
                     [value_ptr](float value) {
                       *value_ptr = std::clamp(value, 0.0f, 1.0f);
                     });
  }
  {
    auto* value_ptr = &d.needle_specularity_plasticity_year2plus;
    explorer.AddAxis("needle_specularity_plasticity_year2plus", "NP2", 0.0f, 1.0f,
                     [value_ptr]() { return *value_ptr; },
                     [value_ptr](float value) {
                       *value_ptr = std::clamp(value, 0.0f, 1.0f);
                     });
  }
  {
    auto* value_ptr = &d.needle_bud_storage_vigor_strength;
    explorer.AddAxis("needle_bud_storage_vigor_strength", "NBV", 0.0f, 1.0f,
                     [value_ptr]() { return *value_ptr; },
                     [value_ptr](float value) {
                       *value_ptr = std::clamp(value, 0.0f, 1.0f);
                     });
  }
  {
    auto* value_ptr = &d.needle_bud_storage_completion_floor;
    explorer.AddAxis("needle_bud_storage_completion_floor", "NBC", 0.0f, 1.0f,
                     [value_ptr]() { return *value_ptr; },
                     [value_ptr](float value) {
                       *value_ptr = std::clamp(value, 0.0f, 1.0f);
                     });
  }

  // -- Needle curvature --
  explorer.AddSingle("needle_curvature_adaxial_bias", "NCA", d.needle_curvature_adaxial_bias,
                     -0.1f, 0.1f, 0.003f);
  explorer.AddSingle("needle_curvature_abaxial_bias", "NCB", d.needle_curvature_abaxial_bias,
                     -0.1f, 0.1f, 0.010f);
  explorer.AddSingle("needle_curvature_gradient_per_arclen", "NCG",
                     d.needle_curvature_gradient_per_arclen, -0.05f, 0.05f, 0.0015f);
  explorer.AddSingle("needle_diameter_for_curvature_m", "NDC",
                     d.needle_diameter_for_curvature_m, 0.0f, 0.005f, 0.001f);
  explorer.AddSingle("needle_sinusoidal_amplitude_deg", "NSA",
                     d.needle_sinusoidal_amplitude_deg, 0.0f, 45.0f, 0.0f);
  explorer.AddSingle("needle_sinusoidal_frequency_cycles", "NSF",
                     d.needle_sinusoidal_frequency_cycles, 0.0f, 12.0f, 0.0f);
  explorer.AddSingle("needle_sinusoidal_phase_randomness_deg", "NSP",
                     d.needle_sinusoidal_phase_randomness_deg, 0.0f, 180.0f, 0.0f);

  // -- Needle mechanics --
  explorer.AddSingle("needle_young_modulus_baseline_Pa", "YMB",
                     d.needle_young_modulus_baseline_Pa, 0.0f, 5e9f, 1e9f);
  explorer.AddSingle("needle_lignification_maturation_years", "LMY",
                     d.needle_lignification_maturation_years, 0.0f, 5.0f, 1.0f);
  // Width cap exposed as a sweepable axis. Default 0.45 = legacy behavior.
  // Note: this is a plain scalar field; AddAxis is used so the explorer can
  // read/write it directly without needing a SingleDistribution wrapper.
  {
    auto* ratio_ptr = &d.needle_radius_to_stem_thickness_max_ratio;
    explorer.AddAxis("needle_radius_to_stem_thickness_max_ratio", "NRC",
                     0.0f, 4.0f,
                     [ratio_ptr]() { return *ratio_ptr; },
                     [ratio_ptr](float v) {
                       *ratio_ptr = std::clamp(v, 0.0f, 4.0f);
                     });
  }
  explorer.AddSingle("needle_density_kg_m3", "NDK", d.needle_density_kg_m3,
                     0.0f, 2000.0f, 800.0f);
  explorer.AddSingle("gravity_m_s2", "GRV", d.gravity_m_s2, 0.0f, 25.0f, 9.81f);
  explorer.AddSingle("needle_per_needle_length_cv", "NLC",
                     d.needle_per_needle_length_cv, 0.0f, 1.0f, 0.0f);
  explorer.AddSingle("needle_per_needle_curvature_cv", "NCCV",
                     d.needle_per_needle_curvature_cv, 0.0f, 1.0f, 0.0f);
  explorer.AddSingle("needle_per_needle_radius_cv", "NRCV",
                     d.needle_per_needle_radius_cv, 0.0f, 1.0f, 0.0f);
  explorer.AddSingle("needle_per_needle_modulus_cv", "NMCV",
                     d.needle_per_needle_modulus_cv, 0.0f, 1.0f, 0.0f);
  explorer.AddSingle("needle_per_needle_density_cv", "NDCV",
                     d.needle_per_needle_density_cv, 0.0f, 1.0f, 0.0f);
  explorer.AddSingle("needle_per_needle_wave_amplitude_cv", "NWAC",
                     d.needle_per_needle_wave_amplitude_cv, 0.0f, 1.0f, 0.0f);
  explorer.AddSingle("needle_per_needle_wave_frequency_cv", "NWFC",
                     d.needle_per_needle_wave_frequency_cv, 0.0f, 1.0f, 0.0f);
  explorer.AddSingle("needle_per_needle_wave_phase_cv", "NWPC",
                     d.needle_per_needle_wave_phase_cv, 0.0f, 1.0f, 0.0f);

  // -- Tropism --
  explorer.AddSingle("gravitropism_first_order", "GFO", d.gravitropism_first_order,
                     0.0f, 0.001f, 0.0001f);
  explorer.AddSingle("initial_orientation_yaw_deg", "IOY", d.initial_orientation_yaw_deg,
                     -180.0f, 180.0f, 0.0f);

  // -- Per-instance growth target --
  explorer.AddSingle("target_gdd", "TGD", d.target_gdd, 0.0f, 30000.0f, 6000.0f);
  explorer.AddSingle("gdd_per_day", "GPD", d.gdd_per_day, 0.0f, 50.0f, 2.0f);
  explorer.AddSingle("growing_season_start_day", "GSS",
                     d.growing_season_start_day, 0.0f, 365.0f, 60.0f);
  explorer.AddSingle("growing_season_end_day", "GSE",
                     d.growing_season_end_day, 0.0f, 365.0f, 334.0f);

  // -- Per-shoot stochastic noise --
  explorer.AddSingle("internode_length_per_node_cv", "ILC",
                     d.internode_length_per_node_cv, 0.0f, 1.0f, 0.1f);
  explorer.AddSingle("internode_thickness_per_node_cv", "STC",
                     d.internode_thickness_per_node_cv, 0.0f, 1.0f, 0.1f);
  explorer.AddSingle("branch_angle_per_node_sigma_deg", "BAS",
                     d.branch_angle_per_node_sigma_deg, 0.0f, 30.0f, 5.0f);
  explorer.AddSingle("roll_phyllotaxis_per_node_sigma_deg", "RPS",
                     d.roll_phyllotaxis_per_node_sigma_deg, 0.0f, 30.0f, 5.0f);

  // -- Dynamic tropism dimensions --
  for (size_t i = 0; i < d.tropisms.size(); i++) {
    auto& tropism = d.tropisms[i];
    const std::string p = "tropism[" + std::to_string(i) + "]";
    const std::string s = "T" + std::to_string(i);

    explorer.AddSingle(p + ".direction_x", s + "X", tropism.direction_x, -1.0f, 1.0f, 1.0f);
    explorer.AddSingle(p + ".direction_y", s + "Y", tropism.direction_y, -1.0f, 1.0f, 1.0f);
    explorer.AddSingle(p + ".direction_z", s + "Z", tropism.direction_z, -1.0f, 1.0f, 1.0f);
    explorer.AddSingle(p + ".strength", s + "S", tropism.strength, -5.0f, 5.0f, 5.0f);

    auto* tropism_ptr = &d.tropisms[i];
    explorer.AddAxis(p + ".usage_chance_percent", s + "U", 0.0f, 100.0f,
                     [tropism_ptr]() { return tropism_ptr->usage_chance_percent; },
                     [tropism_ptr](float v) {
                       tropism_ptr->usage_chance_percent = std::clamp(v, 0.0f, 100.0f);
                     });

    explorer.AddPlotted(p + ".order_response", s + "O", tropism.order_response);
  }
}
