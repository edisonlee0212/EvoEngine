#include "ScotsPineDescriptor.hpp"
#include <yaml-cpp/yaml.h>
#include <Application.hpp>
#include <EditorLayer.hpp>
#include <Scene.hpp>
#include <Transform.hpp>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
#include "LSystemDescriptorDefaults.hpp"
#include "LSystemInspectionAdapters.hpp"
#include "LSystemSerializationAdapters.hpp"
#include "ScotsPine.hpp"
#include "ScotsPineTemporalGrowth.hpp"

using namespace l_system_package;
using namespace evo_engine;

bool l_system_package::InspectScotsPineDescriptor(InspectorContext& context, ScotsPineDescriptor& descriptor) {
  return descriptor.DrawEditorControls(context.editor_layer);
}

// ===========================================================================
// File-local helpers (defaults file resolution, loading, target_gdd sampling).
// File extension: .spine
// ===========================================================================
namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr char kScotsPineDescriptorName[] = "ScotsPineDescriptor";
constexpr char kScotsPineFloatFormat[] = "%.5f";

const std::filesystem::path kScotsPinePackageDefaultsPath =
    std::filesystem::path("EvoEngine_Packages/LSystem/Internals/LSystemResources/Defaults/") /
    "ScotsPineDescriptor_Default.spine";

const std::array<std::filesystem::path, 10> kScotsPineResourceCandidates = {
    std::filesystem::path("./EvoEngine_Packages/LSystem/Internals/LSystemResources/Defaults/") /
        "ScotsPineDescriptor_Default.spine",
    std::filesystem::path("../../../../../EvoEngine_Packages/LSystem/Internals/LSystemResources/Defaults/") /
        "ScotsPineDescriptor_Default.spine",
    std::filesystem::path("../../../../EvoEngine_Packages/LSystem/Internals/LSystemResources/Defaults/") /
        "ScotsPineDescriptor_Default.spine",
    std::filesystem::path("./04_EvoEngine/EvoEngine_Packages/LSystem/Internals/") /
        "LSystemResources/Defaults/ScotsPineDescriptor_Default.spine",
    std::filesystem::path("./LSystemResources/Defaults/ScotsPineDescriptor_Default.spine"),
    std::filesystem::path("./EvoEngine_Plugins/LSystem/Internals/LSystemResources/Defaults/") /
        "ScotsPineDescriptor_Default.spine",
    std::filesystem::path("./LSystemProjectAssets/Assets/New ScotsPineDescriptor.spine"),
    std::filesystem::path("./Resources/DigitalAgricultureProject/Assets/New ScotsPineDescriptor.spine"),
    std::filesystem::path("./DigitalAgricultureProject/Assets/New ScotsPineDescriptor.spine"),
    std::filesystem::path("./04_EvoEngine/Resources/DigitalAgricultureProject/Assets/") /
        "New ScotsPineDescriptor.spine"};

const std::array<std::filesystem::path, 3> kScotsPineProjectAssetCandidates = {
    std::filesystem::path("LSystemProjectAssets") / "Assets" / "New ScotsPineDescriptor.spine",
    std::filesystem::path("LSystem") / "New ScotsPineDescriptor.spine", "New ScotsPineDescriptor.spine"};

const std::array<std::filesystem::path, 4> kScotsPineWritableDefaultsCandidates = {
    std::filesystem::path("./EvoEngine_Packages/LSystem/Internals/LSystemResources/Defaults/") /
        "ScotsPineDescriptor_Default.spine",
    std::filesystem::path("../../../../../EvoEngine_Packages/LSystem/Internals/LSystemResources/Defaults/") /
        "ScotsPineDescriptor_Default.spine",
    std::filesystem::path("../../../../EvoEngine_Packages/LSystem/Internals/LSystemResources/Defaults/") /
        "ScotsPineDescriptor_Default.spine",
    std::filesystem::path("./04_EvoEngine/EvoEngine_Packages/LSystem/Internals/LSystemResources/Defaults/") /
        "ScotsPineDescriptor_Default.spine"};

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

void ConfigureNeedleIntraYearCapacityDefaults(evo_engine::Plot2D<float>& plot) {
  plot.min_value = 0.0f;
  plot.max_value = 1.0f;
  plot.curve.SetTangent(false);
  auto& values = plot.curve.UnsafeGetValues();
  values.clear();
  values.emplace_back(0.0f, 0.75f);
  values.emplace_back(0.125f, 0.756f);
  values.emplace_back(0.25f, 0.772f);
  values.emplace_back(0.375f, 0.812f);
  values.emplace_back(0.5f, 0.875f);
  values.emplace_back(0.625f, 0.938f);
  values.emplace_back(0.75f, 0.978f);
  values.emplace_back(0.875f, 0.994f);
  values.emplace_back(1.0f, 1.0f);
}

void ConfigureMaterialModelCurveDefaults(evo_engine::Plot2D<float>& plot, const float y0, const float y1) {
  plot.min_value = 0.0f;
  plot.max_value = 1.0f;
  SetCurveLinear01(plot.curve, y0, y1, 5);
}

void ConfigurePineMaturityDefaults(ScotsPineDescriptor& descriptor) {
  ConfigureMaturityDistributionDefaults(descriptor.internode_length_maturity_curve);
  ConfigureMaturityDistributionDefaults(descriptor.internode_width_maturity_curve);
  ConfigureMaturityDistributionDefaults(descriptor.needle_length_maturity_curve);
  ConfigureNeedleCrossSectionProfileDefaults(descriptor.needle_cross_section_width_profile, 0.348f, 0.098f);
  ConfigureNeedleCrossSectionProfileDefaults(descriptor.needle_cross_section_thickness_profile, 0.5f, 0.184f);
  ConfigureNeedleCrossSectionTemporalMaturityDefaults(descriptor.needle_cross_section_temporal_maturity_curve);
  ConfigureNeedleIntraYearCapacityDefaults(descriptor.needle_intra_year_capacity_curve);
  ConfigureMaterialModelCurveDefaults(descriptor.needle_axial_color_curve, 0.0f, 1.0f);
  ConfigureMaterialModelCurveDefaults(descriptor.needle_y_age_color_curve, 0.0f, 1.0f);
  ConfigureMaterialModelCurveDefaults(descriptor.stem_age_gradient_curve, 0.0f, 1.0f);
}

double GetSteadyTimeSeconds() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

std::filesystem::path ResolveDefaultScotsPineDescriptorPath() {
  return descriptor_defaults::ResolveExistingDefaultsPath(kScotsPineResourceCandidates,
                                                          kScotsPineProjectAssetCandidates);
}

std::filesystem::path ResolveWritableScotsPineDescriptorDefaultsPath() {
  for (const auto& candidate : kScotsPineWritableDefaultsCandidates) {
    const auto absolute_candidate = std::filesystem::absolute(candidate);
    const auto parent = absolute_candidate.parent_path();
    if (parent.empty() || std::filesystem::exists(parent)) {
      return absolute_candidate;
    }
  }
  return std::filesystem::absolute(kScotsPinePackageDefaultsPath);
}

glm::vec4 ClampFiniteColor01(const glm::vec4& color, const glm::vec4& fallback) {
  glm::vec4 result = color;
  if (!std::isfinite(result.r) || !std::isfinite(result.g) || !std::isfinite(result.b) || !std::isfinite(result.a)) {
    result = fallback;
  }
  result = glm::clamp(result, glm::vec4(0.0f), glm::vec4(1.0f));
  result.a = 1.0f;
  return result;
}

void ClampMaterialModel(ScotsPineDescriptor& descriptor) {
  descriptor.material_profile_version = std::max(1, descriptor.material_profile_version);
  descriptor.biological_material_model_strength =
      std::clamp(descriptor.biological_material_model_strength, 0.0f, 1.0f);
  descriptor.biological_chlorophyll_scale = std::clamp(descriptor.biological_chlorophyll_scale, 0.2f, 2.0f);
  descriptor.biological_carotenoid_gold_scale =
      std::clamp(descriptor.biological_carotenoid_gold_scale, 0.0f, 2.0f);
  descriptor.biological_lignin_bark_scale = std::clamp(descriptor.biological_lignin_bark_scale, 0.0f, 2.0f);
  descriptor.biological_senescence_bias = std::clamp(descriptor.biological_senescence_bias, -0.5f, 0.5f);
  descriptor.biological_cuticle_wax = std::clamp(descriptor.biological_cuticle_wax, 0.0f, 1.0f);
  descriptor.biological_individual_variation =
      std::clamp(descriptor.biological_individual_variation, 0.0f, 1.0f);
  descriptor.biological_facet_contrast = std::clamp(descriptor.biological_facet_contrast, 0.0f, 1.0f);
  descriptor.biological_tip_darkening_strength =
      std::clamp(descriptor.biological_tip_darkening_strength, 0.0f, 1.0f);
  descriptor.biological_stem_age_browning_scale =
      std::clamp(descriptor.biological_stem_age_browning_scale, 0.0f, 2.0f);
  descriptor.young_needle_palette_rgba =
      ClampFiniteColor01(descriptor.young_needle_palette_rgba, glm::vec4(0.0f, 0.7455683f, 0.051418442f, 1.0f));
  descriptor.older_needle_palette_rgba =
      ClampFiniteColor01(descriptor.older_needle_palette_rgba, glm::vec4(0.21606492f, 0.28904234f, 0.20766766f, 1.0f));
  descriptor.dry_brown_needle_palette_rgba =
      ClampFiniteColor01(descriptor.dry_brown_needle_palette_rgba, glm::vec4(0.77059436f, 0.5399785f, 0.0f, 1.0f));
  descriptor.main_stem_palette_rgba =
      ClampFiniteColor01(descriptor.main_stem_palette_rgba, glm::vec4(0.7861458f, 0.9165798f, 0.47310424f, 1.0f));
  descriptor.mature_bark_stem_palette_rgba =
      ClampFiniteColor01(descriptor.mature_bark_stem_palette_rgba, glm::vec4(0.7009804f, 0.45447198f, 0.18555366f, 1.0f));
  descriptor.node_sheath_brown_palette_rgba =
      ClampFiniteColor01(descriptor.node_sheath_brown_palette_rgba, glm::vec4(0.49f, 0.31f, 0.13f, 1.0f));
  descriptor.fascicle_sheath_palette_rgba =
      ClampFiniteColor01(descriptor.fascicle_sheath_palette_rgba, glm::vec4(0.42f, 0.34f, 0.24f, 1.0f));
  auto clamp_plot = [](evo_engine::Plot2D<float>& plot) {
    plot.min_value = std::clamp(plot.min_value, 0.0f, 1.0f);
    plot.max_value = std::clamp(plot.max_value, 0.0f, 1.0f);
    if (plot.max_value < plot.min_value) {
      std::swap(plot.min_value, plot.max_value);
    }
  };
  clamp_plot(descriptor.needle_axial_color_curve);
  clamp_plot(descriptor.needle_y_age_color_curve);
  clamp_plot(descriptor.stem_age_gradient_curve);
  descriptor.needle_micro_variation = std::clamp(descriptor.needle_micro_variation, 0.0f, 0.25f);
  descriptor.stem_micro_variation = std::clamp(descriptor.stem_micro_variation, 0.0f, 0.25f);
  descriptor.young_needle_roughness = std::clamp(descriptor.young_needle_roughness, 0.02f, 1.0f);
  descriptor.old_needle_roughness = std::clamp(descriptor.old_needle_roughness, 0.02f, 1.0f);
  descriptor.young_needle_specular = std::clamp(descriptor.young_needle_specular, 0.0f, 1.0f);
  descriptor.old_needle_specular = std::clamp(descriptor.old_needle_specular, 0.0f, 1.0f);
  descriptor.stem_roughness = std::clamp(descriptor.stem_roughness, 0.02f, 1.0f);
  descriptor.stem_specular = std::clamp(descriptor.stem_specular, 0.0f, 1.0f);
  descriptor.node_browning_strength = std::clamp(descriptor.node_browning_strength, 0.0f, 1.0f);
  descriptor.sheath_browning_strength = std::clamp(descriptor.sheath_browning_strength, 0.0f, 1.0f);
  descriptor.node_browning_radius_norm = std::clamp(descriptor.node_browning_radius_norm, 0.0f, 1.0f);
  descriptor.needle_twist_turns = std::clamp(descriptor.needle_twist_turns, -8.0f, 8.0f);
  descriptor.needle_edge_darkening = std::clamp(descriptor.needle_edge_darkening, 0.0f, 0.75f);
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

float SampleTargetGddForSeed(const evo_engine::SingleDistribution<float>& distribution, const uint32_t seed) {
  std::mt19937 rng(seed);
  return std::max(0.0f, SampleDistribution(distribution, rng));
}

bool LoadScotsPineDescriptorDefaultsFromFile(ScotsPineDescriptor& descriptor, const std::filesystem::path& file_path) {
  YAML::Node defaults;
  if (!descriptor_defaults::LoadDefaultsYamlMap(file_path, defaults, kScotsPineDescriptorName)) {
    return false;
  }
  DeserializeScotsPineDescriptor(defaults, descriptor);
  return true;
}

std::string SerializeDescriptorMapToString(const ScotsPineDescriptor& descriptor) {
  YAML::Emitter out;
  out << YAML::BeginMap;
  SerializeScotsPineDescriptor(out, descriptor);
  out << YAML::EndMap;
  return out.c_str() ? std::string(out.c_str()) : std::string{};
}

bool ValidateScotsPineDefaultsRoundTrip(const ScotsPineDescriptor& expected, const std::filesystem::path& file_path,
                                        std::string* error_message) {
  ScotsPineDescriptor loaded;
  if (!LoadScotsPineDescriptorDefaultsFromFile(loaded, file_path)) {
    if (error_message) {
      *error_message = "Saved defaults could not be loaded from " + file_path.string();
    }
    return false;
  }

  const std::string expected_yaml = SerializeDescriptorMapToString(expected);
  const std::string loaded_yaml = SerializeDescriptorMapToString(loaded);
  if (expected_yaml != loaded_yaml) {
    if (error_message) {
      *error_message = "Saved defaults failed full descriptor round-trip validation at " + file_path.string();
    }
    return false;
  }
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

std::filesystem::path ScotsPineDescriptor::ResolveWritableDefaultsPath() const {
  return ResolveWritableScotsPineDescriptorDefaultsPath();
}

bool ScotsPineDescriptor::SaveCurrentAsDefaultSnapshot(std::string* error_message) const {
  const auto defaults_path = ResolveWritableDefaultsPath();
  if (defaults_path.empty()) {
    if (error_message) {
      *error_message = "ScotsPineDescriptor default snapshot path is empty.";
    }
    return false;
  }

  try {
    const auto parent_path = defaults_path.parent_path();
    if (!parent_path.empty() && !std::filesystem::exists(parent_path)) {
      std::filesystem::create_directories(parent_path);
    }
    std::ofstream out(defaults_path, std::ios::trunc);
    if (!out.is_open()) {
      if (error_message) {
        *error_message = "Failed to write " + defaults_path.string();
      }
      return false;
    }
    out << SerializeDescriptorMapToString(*this);
    out.close();
    if (!out) {
      if (error_message) {
        *error_message = "Failed to finish writing " + defaults_path.string();
      }
      return false;
    }
    if (!ValidateScotsPineDefaultsRoundTrip(*this, defaults_path, error_message)) {
      return false;
    }
  } catch (const std::exception& e) {
    if (error_message) {
      *error_message = e.what();
    }
    EVOENGINE_ERROR("Failed to save ScotsPineDescriptor defaults snapshot: " + std::string(e.what()))
    return false;
  }

  EVOENGINE_LOG("Saved ScotsPineDescriptor defaults snapshot to " + defaults_path.string())
  return true;
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

  out.annual_whorl_probability = std::clamp(annual_whorl_probability.mean, 0.0f, 1.0f);
  out.whorl_position_norm = std::clamp(whorl_position_norm.mean, 0.75f, 1.0f);
  out.branches_per_whorl = std::clamp(static_cast<int>(std::round(SampleDistribution(branches_per_whorl, rng))), 1, 5);
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
  out.needle_segment_count = std::clamp(needle_segment_count, 1, 128);
  out.fascicle_sheath_length_m = std::clamp(fascicle_sheath_length_m.mean, 0.0005f, 0.030f);
  out.fascicle_sheath_width_m = std::clamp(fascicle_sheath_width_m.mean, 0.0002f, 0.010f);
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
  out.needle_intra_year_capacity_curve = needle_intra_year_capacity_curve;
  out.needle_intra_year_capacity_curve.min_value =
      std::clamp(out.needle_intra_year_capacity_curve.min_value, 0.0f, 2.0f);
  out.needle_intra_year_capacity_curve.max_value =
      std::clamp(out.needle_intra_year_capacity_curve.max_value, 0.0f, 2.0f);
  if (out.needle_intra_year_capacity_curve.max_value < out.needle_intra_year_capacity_curve.min_value) {
    std::swap(out.needle_intra_year_capacity_curve.min_value, out.needle_intra_year_capacity_curve.max_value);
  }
  out.needle_year0_length_multiplier = std::max(0.0f, needle_year0_length_multiplier);
  out.needle_lignification_factor_year0 = std::clamp(needle_lignification_factor_year0, 0.0f, 2.0f);
  out.needle_stomatal_strip_density_year0 = std::clamp(needle_stomatal_strip_density_year0, 0.0f, 1.0f);
  out.needle_basal_taper_ratio_year0 = std::clamp(needle_basal_taper_ratio_year0, 0.6f, 1.2f);
  out.needle_fascicle_sheath_budget_years = std::max(0.0f, needle_fascicle_sheath_budget_gdd / kPineGddPerYear);
  out.needle_specularity_plasticity_year0 = std::clamp(needle_specularity_plasticity_year0, 0.0f, 1.0f);
  out.needle_bud_storage_vigor_strength = std::clamp(needle_bud_storage_vigor_strength, 0.0f, 1.0f);
  out.needle_bud_storage_completion_floor = std::clamp(needle_bud_storage_completion_floor, 0.0f, 1.0f);
  out.gravitropism_first_order = SampleDistribution(gravitropism_first_order, rng);

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
  // every consumption site (per phytomer / per whorl / per fascicle sheath).
  // No additional RNG draws here; the rule call sites will draw fresh from
  // their own per-node RNGs (see `MakeNodeRng` in LSystemRuleHelpers.hpp).
  out.distributions.max_branching_order = max_branching_order;
  out.distributions.plastochron_gdd = plastochron_gdd;
  out.distributions.max_phytomers_per_seasonal_growth = max_phytomers_per_seasonal_growth;
  out.distributions.annual_whorl_probability = annual_whorl_probability;
  out.distributions.whorl_position_norm = whorl_position_norm;
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
  out.distributions.fascicle_sheath_length_m = fascicle_sheath_length_m;
  out.distributions.fascicle_sheath_width_m = fascicle_sheath_width_m;
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
  pine->target_gdd = SampleTargetGddForSeed(target_gdd, pine->seed);
  pine->GenerateGeometryEntities();

  return entity;
}

// ===========================================================================
// Inspector UI
// ===========================================================================
bool ScotsPineDescriptor::DrawEditorControls(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  bool editor_preferences_changed = false;

  const auto show_item_hover_description = [](const char* description) {
    if (!description || description[0] == '\0')
      return;
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
  static std::string default_snapshot_status;
  if (ImGui::Button("Set as New Descriptor Defaults")) {
    default_snapshot_status.clear();
    ImGui::OpenPopup("Confirm ScotsPine Defaults Snapshot");
  }
  show_item_hover_description("Overwrite the canonical ScotsPineDescriptor defaults used by newly created assets.");

  if (ImGui::BeginPopupModal("Confirm ScotsPine Defaults Snapshot", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
    const auto defaults_path = ResolveWritableDefaultsPath();
    const auto active_defaults_path = ResolveDefaultScotsPineDescriptorPath();
    ImGui::TextWrapped("Overwrite the default snapshot used by future ScotsPineDescriptor assets?");
    ImGui::Separator();
    ImGui::TextWrapped("Save target: %s", defaults_path.string().c_str());
    ImGui::TextWrapped("Active load target: %s", active_defaults_path.string().c_str());
    if (!default_snapshot_status.empty()) {
      ImGui::Separator();
      ImGui::TextWrapped("%s", default_snapshot_status.c_str());
    }

    if (ImGui::Button("Confirm")) {
      std::string error_message;
      if (SaveCurrentAsDefaultSnapshot(&error_message)) {
        default_snapshot_status = "Saved and round-trip validated defaults snapshot.";
        ImGui::CloseCurrentPopup();
      } else {
        default_snapshot_status = error_message.empty() ? "Failed to save defaults snapshot." : error_message;
      }
    }
    ImGui::SameLine();
    if (ImGui::Button("Cancel")) {
      default_snapshot_status.clear();
      ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
  }

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

  if (ImGui::DragFloat("Live Preview Rate (Hz)", &live_preview_rate_hz, 0.25f, 1.0f, 60.0f, kScotsPineFloatFormat)) {
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

  if (ImGui::DragFloat("Preview Max GDD", &live_preview_max_gdd, 50.0f, 0.0f, 100000.0f, kScotsPineFloatFormat)) {
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
    const double avg_apply_ms = live_preview_total_apply_ms_ / static_cast<double>(live_preview_apply_count_);
    ImGui::Text("Preview last/avg ms: %.3f / %.3f", live_preview_last_apply_ms_, avg_apply_ms);
  }
  ImGui::Text("Preview requests/applied/coalesced: %u / %u / %u", live_preview_request_count_,
              live_preview_apply_count_, live_preview_coalesced_count_);

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
    ImGui::DragFloat("Spacing", &grid_spacing, 0.1f, 0.5f, 50.0f, kScotsPineFloatFormat);
    show_item_hover_description("World-space spacing between neighboring grid pines.");

    if (ImGui::Button("Instantiate Grid")) {
      const auto scene = GetApplication().GetActiveScene();
      if (scene) {
        const auto container = scene->CreateEntity("Pine Grid");
        const float offset_y = (static_cast<float>(grid_rows) - 1.0f) * grid_spacing * 0.5f;
        const float offset_z = (static_cast<float>(grid_cols) - 1.0f) * grid_spacing * 0.5f;
        const auto base_seed =
            static_cast<unsigned int>(std::chrono::steady_clock::now().time_since_epoch().count() & 0xFFFFFFFFu);
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
            transform.SetPosition(glm::vec3(0.0f, static_cast<float>(i) * grid_spacing - offset_y,
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
      const auto scene = GetApplication().GetActiveScene();
      if (scene) {
        const auto* pine_entities_ptr = scene->UnsafeGetPrivateComponentOwnersList<ScotsPine>();
        if (pine_entities_ptr) {
          const std::vector<Entity> pine_entities = *pine_entities_ptr;
          std::vector<Entity> to_delete;
          std::vector<Entity> containers;
          for (const auto& entity : pine_entities) {
            if (!scene->IsEntityValid(entity))
              continue;
            auto pine = scene->GetOrSetPrivateComponent<ScotsPine>(entity).lock();
            if (!pine)
              continue;
            if (pine->descriptor_ref.Get<ScotsPineDescriptor>().get() == this) {
              to_delete.push_back(entity);
              const auto parent = scene->GetParent(entity);
              if (scene->IsEntityValid(parent) && scene->GetEntityName(parent) == "Pine Grid") {
                containers.push_back(parent);
              }
            }
          }
          for (const auto& entity : to_delete)
            scene->DeleteEntity(entity);
          std::sort(containers.begin(), containers.end(), [](const Entity& a, const Entity& b) {
            return a.GetIndex() < b.GetIndex();
          });
          containers.erase(std::unique(containers.begin(), containers.end()), containers.end());
          for (const auto& container : containers) {
            if (scene->IsEntityValid(container))
              scene->DeleteEntity(container);
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
    ImGui::DragFloat("Side Length", &triangle_side_length, 0.1f, 0.0f, 0.0f, kScotsPineFloatFormat);
    show_item_hover_description(
        "World-space side length for an equilateral 3-pine triangle on the horizontal XZ plane.");

    if (ImGui::Button("Instantiate Triangle")) {
      const auto scene = GetApplication().GetActiveScene();
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

        const auto base_seed =
            static_cast<unsigned int>(std::chrono::steady_clock::now().time_since_epoch().count() & 0xFFFFFFFFu);
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
    show_item_hover_description("Spawn 3 ScotsPine entities in an equilateral triangle with unique seeds.");

    ImGui::SameLine();
    if (ImGui::Button("Delete Triangle")) {
      const auto scene = GetApplication().GetActiveScene();
      if (scene) {
        const auto* pine_entities_ptr = scene->UnsafeGetPrivateComponentOwnersList<ScotsPine>();
        if (pine_entities_ptr) {
          const std::vector<Entity> pine_entities = *pine_entities_ptr;
          std::vector<Entity> to_delete;
          std::vector<Entity> containers;
          for (const auto& entity : pine_entities) {
            if (!scene->IsEntityValid(entity))
              continue;
            auto pine = scene->GetOrSetPrivateComponent<ScotsPine>(entity).lock();
            if (!pine)
              continue;
            if (pine->descriptor_ref.Get<ScotsPineDescriptor>().get() == this) {
              to_delete.push_back(entity);
              const auto parent = scene->GetParent(entity);
              if (scene->IsEntityValid(parent) && scene->GetEntityName(parent) == "Pine Triangle") {
                containers.push_back(parent);
              }
            }
          }
          for (const auto& entity : to_delete)
            scene->DeleteEntity(entity);
          std::sort(containers.begin(), containers.end(), [](const Entity& a, const Entity& b) {
            return a.GetIndex() < b.GetIndex();
          });
          containers.erase(std::unique(containers.begin(), containers.end()), containers.end());
          for (const auto& container : containers) {
            if (scene->IsEntityValid(container))
              scene->DeleteEntity(container);
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
    if (!explorer_.IsBound())
      explorer_.Bind(*this);
    if (explorer_.DrawGui()) {
      changed = true;
    }
    show_item_hover_description("Interactive parameter sweep and sensitivity exploration tools for this descriptor.");
    ImGui::TreePop();
  }

  ImGui::Separator();

  // -- Global development clock --
  if (ImGui::TreeNodeEx("Core: Growth Timeline and Shoot Density", ImGuiTreeNodeFlags_DefaultOpen)) {
    changed |= target_gdd.Draw("Target GDD", 1.0f,
                               "Distribution of target GDD used by Instantiate, Grid spawn, and Triangle spawn.");
    changed |= plastochron_gdd.Draw("Plastochron (GDD)", 10.0f,
                                    "Physiological time between consecutive phytomer events on an axis.");
    changed |= max_phytomers_per_seasonal_growth.Draw("Max Phytomers per Seasonal Growth", 0.5f,
                                                      "Phytomers (internode + optional fascicle sheath) emitted "
                                                      "per active season before the apex pauses until next year.");
    changed |= gdd_per_day.Draw("GDD per Day", 0.1f,
                                "Per-pine thermal accumulation rate. Multiplied directly by chronological day delta.");

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
    clamp_nonnegative_distribution(gdd_per_day);

    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Parameter Sanity", ImGuiTreeNodeFlags_DefaultOpen)) {
    constexpr float kCalendarDaysPerYear = 365.0f;
    const float expected_gdd_per_year = std::max(0.0f, gdd_per_day.mean) * kCalendarDaysPerYear;
    const float mean_plastochron_gdd = std::max(1.0e-4f, plastochron_gdd.mean);
    const float active_day_phytomers_from_gdd =
        std::max(0.0f, gdd_per_day.mean) / mean_plastochron_gdd;
    const float effective_plastochron_days =
        std::max(0.0f, gdd_per_day.mean) > 1.0e-6f ? mean_plastochron_gdd / std::max(0.0f, gdd_per_day.mean) : 0.0f;
    const float expected_phytomers_per_year = std::max(0.0f, max_phytomers_per_seasonal_growth.mean);
    const float expected_bare_phytomers =
        std::round(std::clamp(bare_zone_fraction.mean, 0.0f, 0.95f) * expected_phytomers_per_year);
    const float expected_fascicle_sheaths_per_year =
        std::max(0.0f, expected_phytomers_per_year - expected_bare_phytomers);
    const float expected_needles_per_year =
        expected_fascicle_sheaths_per_year * std::max(0.0f, needle_count_per_cluster.mean);
    const float expected_year0_needle_m =
        std::max(0.0f, needle_length_m.mean) * std::max(0.0f, needle_year0_length_multiplier);
    const float expected_later_needle_m = std::max(0.0f, needle_length_m.mean);
    const float expected_seasonal_leader_m =
        std::max(0.0f, internode_length_m.mean) * std::max(0.0f, max_phytomers_per_seasonal_growth.mean);
    const float expected_two_season_leader_m = expected_seasonal_leader_m * 2.0f;
    ImGui::Text("Expected active days/year: %.0f", kCalendarDaysPerYear);
    ImGui::Text("Expected thermal budget/year: %.1f GDD", expected_gdd_per_year);
    ImGui::Text("GDD-driven phytomers/active day: %.2f", active_day_phytomers_from_gdd);
    ImGui::Text("Effective plastochron at mean GDD/day: %.2f days", effective_plastochron_days);
    ImGui::Text("Expected phytomers/year: %.1f", expected_phytomers_per_year);
    ImGui::Text("Expected bare phytomers/year: %.1f", expected_bare_phytomers);
    ImGui::Text("Expected fascicle sheaths/year: %.1f", expected_fascicle_sheaths_per_year);
    ImGui::Text("Expected visible needles/year: %.1f", expected_needles_per_year);
    if (expected_fascicle_sheaths_per_year < 1.0f || expected_needles_per_year < 2.0f) {
      ImGui::TextColored(ImVec4(1.0f, 0.55f, 0.15f, 1.0f),
                         "Warning: these parameters imply near-zero annual needle initiation.");
    }
    ImGui::Text("Mature needle length field: %.1f mm", needle_length_m.mean * 1000.0f);
    ImGui::Text("Expected year-0 needle: %.1f mm", expected_year0_needle_m * 1000.0f);
    ImGui::Text("Expected later-cohort needle: %.1f mm", expected_later_needle_m * 1000.0f);
    ImGui::Text("Expected seasonal leader elongation: %.1f mm", expected_seasonal_leader_m * 1000.0f);
    ImGui::Text("Expected two-season leader elongation: %.1f mm", expected_two_season_leader_m * 1000.0f);
    ImGui::Text("Rendered stem diameter field: %.2f mm (cylinder radius uses half)",
                leader_internode_thickness_m.mean * 1000.0f);
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Core: Biological Material Model", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (ImGui::DragFloat("Biological Model Strength", &biological_material_model_strength, 0.01f, 0.0f, 1.0f,
                         kScotsPineFloatFormat)) {
      biological_material_model_strength = std::clamp(biological_material_model_strength, 0.0f, 1.0f);
      changed = true;
    }
    show_item_hover_description("0 keeps direct palette/curve rendering. 1 lets the pigment and organ-position controls below drive color.");
    if (ImGui::DragFloat("Chlorophyll / Live Green", &biological_chlorophyll_scale, 0.01f, 0.2f, 2.0f,
                         kScotsPineFloatFormat)) {
      biological_chlorophyll_scale = std::clamp(biological_chlorophyll_scale, 0.2f, 2.0f);
      changed = true;
    }
    show_item_hover_description("Higher values make live needles greener; lower values reduce chlorophyll.");
    if (ImGui::DragFloat("Carotenoid / Gold", &biological_carotenoid_gold_scale, 0.01f, 0.0f, 2.0f,
                         kScotsPineFloatFormat)) {
      biological_carotenoid_gold_scale = std::clamp(biological_carotenoid_gold_scale, 0.0f, 2.0f);
      changed = true;
    }
    show_item_hover_description("Golden-yellow contribution in senescent or strongly lit old needles.");
    if (ImGui::DragFloat("Lignin / Bark Brown", &biological_lignin_bark_scale, 0.01f, 0.0f, 2.0f,
                         kScotsPineFloatFormat)) {
      biological_lignin_bark_scale = std::clamp(biological_lignin_bark_scale, 0.0f, 2.0f);
      changed = true;
    }
    show_item_hover_description("Brown bark/lignin contribution in older stems and nodes.");
    if (ImGui::DragFloat("Senescence / Dryness Bias", &biological_senescence_bias, 0.01f, -0.5f, 0.5f,
                         kScotsPineFloatFormat)) {
      biological_senescence_bias = std::clamp(biological_senescence_bias, -0.5f, 0.5f);
      changed = true;
    }
    show_item_hover_description("Signed bias: positive pushes needles toward older/drier color, negative keeps them younger.");
    if (ImGui::DragFloat("Cuticle Wax", &biological_cuticle_wax, 0.01f, 0.0f, 1.0f, kScotsPineFloatFormat)) {
      biological_cuticle_wax = std::clamp(biological_cuticle_wax, 0.0f, 1.0f);
      changed = true;
    }
    show_item_hover_description("Pale/desaturated waxy lift on live needles.");
    if (ImGui::DragFloat("Individual Variation", &biological_individual_variation, 0.01f, 0.0f, 1.0f,
                         kScotsPineFloatFormat)) {
      biological_individual_variation = std::clamp(biological_individual_variation, 0.0f, 1.0f);
      changed = true;
    }
    show_item_hover_description("Extra deterministic needle/stem-to-needle/stem color variation.");
    if (ImGui::DragFloat("Facet Contrast", &biological_facet_contrast, 0.01f, 0.0f, 1.0f,
                         kScotsPineFloatFormat)) {
      biological_facet_contrast = std::clamp(biological_facet_contrast, 0.0f, 1.0f);
      changed = true;
    }
    show_item_hover_description("Increases faceted needle edge darkening and small specular differences.");
    if (ImGui::DragFloat("Needle Tip Darkening", &biological_tip_darkening_strength, 0.01f, 0.0f, 1.0f,
                         kScotsPineFloatFormat)) {
      biological_tip_darkening_strength = std::clamp(biological_tip_darkening_strength, 0.0f, 1.0f);
      changed = true;
    }
    show_item_hover_description("Extra live-needle darkening from mid needle toward the tip.");
    if (ImGui::DragFloat("Stem Age Browning", &biological_stem_age_browning_scale, 0.01f, 0.0f, 2.0f,
                         kScotsPineFloatFormat)) {
      biological_stem_age_browning_scale = std::clamp(biological_stem_age_browning_scale, 0.0f, 2.0f);
      changed = true;
    }
    show_item_hover_description("Scales how strongly stem age maps to mature bark brown.");
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Core: Material Model", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (ImGui::ColorEdit4("Young Needle Palette", &young_needle_palette_rgba.x)) {
      changed = true;
    }
    show_item_hover_description("Current-year green needle swatch derived from measured plant-tissue histograms.");
    if (ImGui::ColorEdit4("Older Needle Palette", &older_needle_palette_rgba.x)) {
      changed = true;
    }
    show_item_hover_description("Older/live needle swatch used by axial and age curves before dry browning.");
    if (ImGui::ColorEdit4("Dry/Brown Needle Palette", &dry_brown_needle_palette_rgba.x)) {
      changed = true;
    }
    show_item_hover_description("Senescent and dry needle swatch.");
    if (ImGui::ColorEdit4("Main Stem Palette", &main_stem_palette_rgba.x)) {
      changed = true;
    }
    show_item_hover_description("Young main-stem and branch internode swatch.");
    if (ImGui::ColorEdit4("Mature/Bark Stem Palette", &mature_bark_stem_palette_rgba.x)) {
      changed = true;
    }
    show_item_hover_description("Older stem and bark-like internode swatch.");
    if (ImGui::ColorEdit4("Node/Sheath Browning", &node_sheath_brown_palette_rgba.x)) {
      changed = true;
    }
    show_item_hover_description("Brown node, whorl, and fascicle-sheath swatch mixed near organ bases.");
    if (ImGui::ColorEdit4("Fascicle Sheath Palette", &fascicle_sheath_palette_rgba.x)) {
      changed = true;
    }
    show_item_hover_description("Base sheath cylinder swatch for pine needle fascicles.");

    evo_engine::CurveDescriptorSettings material_curve_settings;
    material_curve_settings.speed = 0.01f;
    material_curve_settings.min_max_control = true;
    material_curve_settings.m_tip = "0 = first palette endpoint, 1 = second palette endpoint.";
    changed |= needle_axial_color_curve.Draw("Needle Axial Color Curve", material_curve_settings);
    show_item_hover_description("Base-to-tip color blend along each needle.");
    changed |= needle_y_age_color_curve.Draw("Needle Y/Age Color Curve", material_curve_settings);
    show_item_hover_description("Cohort age/height blend from young/older needle palettes toward dry brown.");
    changed |= stem_age_gradient_curve.Draw("Stem Age Gradient", material_curve_settings);
    show_item_hover_description("Internode age blend from main-stem palette toward mature/bark palette.");

    if (ImGui::DragFloat("Needle Micro Variation", &needle_micro_variation, 0.001f, 0.0f, 0.25f,
                         kScotsPineFloatFormat)) {
      needle_micro_variation = std::clamp(needle_micro_variation, 0.0f, 0.25f);
      changed = true;
    }
    if (ImGui::DragFloat("Stem Micro Variation", &stem_micro_variation, 0.001f, 0.0f, 0.25f,
                         kScotsPineFloatFormat)) {
      stem_micro_variation = std::clamp(stem_micro_variation, 0.0f, 0.25f);
      changed = true;
    }
    if (ImGui::DragFloat("Young Needle Roughness", &young_needle_roughness, 0.01f, 0.02f, 1.0f,
                         kScotsPineFloatFormat)) {
      young_needle_roughness = std::clamp(young_needle_roughness, 0.02f, 1.0f);
      changed = true;
    }
    if (ImGui::DragFloat("Old Needle Roughness", &old_needle_roughness, 0.01f, 0.02f, 1.0f,
                         kScotsPineFloatFormat)) {
      old_needle_roughness = std::clamp(old_needle_roughness, 0.02f, 1.0f);
      changed = true;
    }
    if (ImGui::DragFloat("Young Needle Specularity", &young_needle_specular, 0.01f, 0.0f, 1.0f,
                         kScotsPineFloatFormat)) {
      young_needle_specular = std::clamp(young_needle_specular, 0.0f, 1.0f);
      changed = true;
    }
    if (ImGui::DragFloat("Old Needle Specularity", &old_needle_specular, 0.01f, 0.0f, 1.0f,
                         kScotsPineFloatFormat)) {
      old_needle_specular = std::clamp(old_needle_specular, 0.0f, 1.0f);
      changed = true;
    }
    if (ImGui::DragFloat("Stem Roughness", &stem_roughness, 0.01f, 0.02f, 1.0f, kScotsPineFloatFormat)) {
      stem_roughness = std::clamp(stem_roughness, 0.02f, 1.0f);
      changed = true;
    }
    if (ImGui::DragFloat("Stem Specularity", &stem_specular, 0.01f, 0.0f, 1.0f, kScotsPineFloatFormat)) {
      stem_specular = std::clamp(stem_specular, 0.0f, 1.0f);
      changed = true;
    }
    if (ImGui::DragFloat("Node Browning Strength", &node_browning_strength, 0.01f, 0.0f, 1.0f,
                         kScotsPineFloatFormat)) {
      node_browning_strength = std::clamp(node_browning_strength, 0.0f, 1.0f);
      changed = true;
    }
    if (ImGui::DragFloat("Sheath Browning Strength", &sheath_browning_strength, 0.01f, 0.0f, 1.0f,
                         kScotsPineFloatFormat)) {
      sheath_browning_strength = std::clamp(sheath_browning_strength, 0.0f, 1.0f);
      changed = true;
    }
    if (ImGui::DragFloat("Node Browning Radius", &node_browning_radius_norm, 0.01f, 0.0f, 1.0f,
                         kScotsPineFloatFormat)) {
      node_browning_radius_norm = std::clamp(node_browning_radius_norm, 0.0f, 1.0f);
      changed = true;
    }
    if (ImGui::DragFloat("Needle Twist (turns)", &needle_twist_turns, 0.01f, -8.0f, 8.0f,
                         kScotsPineFloatFormat)) {
      needle_twist_turns = std::clamp(needle_twist_turns, -8.0f, 8.0f);
      changed = true;
    }
    if (ImGui::DragFloat("Needle Edge Darkening", &needle_edge_darkening, 0.01f, 0.0f, 0.75f,
                         kScotsPineFloatFormat)) {
      needle_edge_darkening = std::clamp(needle_edge_darkening, 0.0f, 0.75f);
      changed = true;
    }
    ImGui::Text("Material profile: v%d %s", material_profile_version, material_profile_source.c_str());
    if (!material_profile_hash.empty()) {
      ImGui::Text("Profile hash: %s", material_profile_hash.c_str());
    }
    ImGui::TreePop();
  }

  // -- Main stem geometry --
  if (ImGui::TreeNodeEx("Core: Stem Size and Appearance", ImGuiTreeNodeFlags_DefaultOpen)) {
    changed |= internode_length_m.Draw("Phytomer Internode Length (m)", 0.001f,
                                       "Length of one phytomer's internode in metres.");
    changed |= leader_internode_thickness_m.Draw(
        "Main Stem Width (Diameter, m)", 0.0001f,
        "Main stem thickness control. This is the leader internode diameter in metres.");
    changed |= initial_orientation_yaw_deg.Draw(
        "Initial Orientation Yaw (deg)", 1.0f,
        "Sampled once per plant and applied as root yaw around +Y. Set deviation > 0 for random initial orientation.");
    if (ImGui::DragFloat("Main Stem Age Exponent", &internode_age_exponent, 0.05f, 0.1f, 4.0f, kScotsPineFloatFormat)) {
      internode_age_exponent = std::clamp(internode_age_exponent, 0.1f, 4.0f);
      changed = true;
    }
    show_item_hover_description(
        "Response curve for stem aging color. 1 = linear, >1 delays browning, <1 accelerates it.");
    ImGui::Text("Mean Radius (m): %.5f", std::max(0.0f, leader_internode_thickness_m.mean) * 0.5f);
    changed |= lateral_length_ratio.Draw("Lateral Length Ratio", 0.05f);
    show_item_hover_description("Lateral shoot length = leader_length * ratio^order.");
    changed |= lateral_thickness_ratio.Draw("Lateral Thickness Ratio", 0.05f);
    show_item_hover_description("Lateral shoot thickness = leader_thickness * ratio^order.");
    ImGui::TreePop();
  }

  // -- Main stem branching --
  if (ImGui::TreeNodeEx("Core: Branching Structure", ImGuiTreeNodeFlags_DefaultOpen)) {
    changed |= max_branching_order.Draw("Max Branching Order", 0.5f);
    show_item_hover_description("0 = leader only, 1 = primary laterals, 2 = secondary laterals.");
    changed |= annual_whorl_probability.Draw("Annual Whorl Probability", 0.01f,
                                             "Per-axis probability of one branch pseudo-whorl in a growth year.");
    changed |= whorl_position_norm.Draw("Whorl Position In Annual Shoot", 0.01f,
                                        "Normalized phytomer position for the annual terminal pseudo-whorl.");
    changed |= branches_per_whorl.Draw("Branches per Whorl", 0.5f);
    show_item_hover_description("Lateral count spawned at whorl bud activation.");
    changed |= whorl_dormancy_years.Draw("Whorl Dormancy (years)", 0.05f,
                                         "Chronological years a whorl bud waits before activating laterals. "
                                         "Bud release is chilling/photoperiod-driven, NOT heat-sum-driven "
                                         "(FSPM Rule of Ontogeny). Default 1 yr = annual Scots pine cycle.");
    changed |= branch_insertion_angle_deg.Draw("Branch Insertion Angle (deg)", 1.0f);
    show_item_hover_description("Angle laterals depart parent (degrees).");
    changed |= branch_roll_phyllotaxis_deg.Draw("Branch Roll Phyllotaxis (deg)", 1.0f);
    show_item_hover_description("Golden-angle azimuth offset between consecutive laterals and needles.");
    ImGui::TreePop();
  }

  // -- Maturity shape curves --
  if (ImGui::TreeNodeEx("Advanced: Maturity Shape Curves")) {
    static int selected_maturity_variable = 0;
    constexpr const char* kMaturityVariables[] = {"Internode Length", "Internode Width", "Needle Length"};

    ImGui::Combo("Variable", &selected_maturity_variable, kMaturityVariables, IM_ARRAYSIZE(kMaturityVariables));
    show_item_hover_description(
        "Choose which maturity-controlled variable to edit. "
        "x = normalized maturity age of the specific organ instance; "
        "y = multiplier in [0,1], where 1 means use full max length/width.");

    evo_engine::PlottedDistribution<float>* selected_distribution = &internode_length_maturity_curve;
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
    changed |= selected_distribution->Draw(selected_label, maturity_settings);

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
  if (ImGui::TreeNodeEx("Advanced: Needle Shape Presets")) {
    static const char* kShapePresets[] = {"(no change)",  "Straight", "Slight Curve",
                                          "Strong Curve", "Wavy",     "Drooping (gravity)"};
    static int s_selected_shape_preset = 0;
    if (ImGui::Combo("Shape Preset##quick_needle_shape", &s_selected_shape_preset, kShapePresets,
                     IM_ARRAYSIZE(kShapePresets))) {
      auto apply_preset = [&](float adaxial, float abaxial, float gradient, float diameter_for_curvature_m,
                              float wave_amp_deg, float wave_freq, float wave_phase_rand_deg, bool enable_droop) {
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
  if (ImGui::TreeNodeEx("Advanced: Needle Cross Section Profiles")) {
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
    auto clamp_profile_distribution = [&](evo_engine::PlottedDistribution<float>& distribution, const float max_value) {
      auto clamp_plot = [&](evo_engine::Plot2D<float>& plot) {
        const float old_min = plot.min_value;
        const float old_max = plot.max_value;
        plot.min_value = std::clamp(plot.min_value, 0.0f, 1.0f);
        plot.max_value = std::clamp(plot.max_value, 0.0f, max_value);
        if (plot.max_value < plot.min_value) {
          std::swap(plot.min_value, plot.max_value);
        }
        if (std::abs(plot.min_value - old_min) > 1.0e-6f || std::abs(plot.max_value - old_max) > 1.0e-6f) {
          changed = true;
        }
      };
      clamp_plot(distribution.mean);
      clamp_plot(distribution.deviation);
    };
    auto inspect_axis = [&](const char* axis_name, evo_engine::SingleDistribution<float>& max_distribution,
                            evo_engine::PlottedDistribution<float>& profile_distribution, const char* max_label,
                            const char* profile_label, const char* tooltip) {
      if (ImGui::TreeNodeEx(axis_name, ImGuiTreeNodeFlags_DefaultOpen)) {
        changed |= max_distribution.Draw(max_label, 0.00005f, tooltip);
        clamp_nonnegative_distribution(max_distribution);

        evo_engine::PlottedDistributionSettings profile_settings;
        profile_settings.tip =
            "Base-to-tip multiplier profile for the selected cross-section measure. "
            "x = normalized arc length from base (0) to tip (1).";
        profile_settings.mean_settings.m_tip =
            "Mean multiplier profile in [0,4]. 1 keeps the max measure; 0 collapses it; "
            "values >1 enlarge it.";
        profile_settings.dev_settings.m_tip = "Variance (sigma) profile in [0,4] around the mean profile.";
        changed |= profile_distribution.Draw(profile_label, profile_settings);
        clamp_profile_distribution(profile_distribution, 4.0f);

        ImGui::Text("Current mean max measure: %.5f mm", std::max(0.0f, max_distribution.mean) * 1000.0f);
        ImGui::TreePop();
      }
    };

    inspect_axis("Width", needle_cross_section_width_max_m, needle_cross_section_width_profile,
                 "Adaxial Width Diameter (m)", "Width Profile (Base -> Tip)",
                 "Flat adaxial chord width before profile multiplier.");
    inspect_axis("Thickness", needle_cross_section_thickness_max_m, needle_cross_section_thickness_profile,
                 "Abaxial Thickness Radius (m)", "Thickness Profile (Base -> Tip)",
                 "Convex abaxial radius/depth before profile multiplier.");

    evo_engine::PlottedDistributionSettings temporal_settings;
    temporal_settings.tip =
        "Shared temporal multiplier applied to both width and thickness. "
        "x = normalized maturity age where x=1 corresponds to 2 years since initiation.";
    temporal_settings.mean_settings.m_tip =
        "Mean multiplier in [0,1]. Default is sinusoidal: 0.25 at t=0 years to 1.0 at t=2 years.";
    temporal_settings.dev_settings.m_tip = "Variance (sigma) profile around the temporal mean in [0,1].";
    changed |= needle_cross_section_temporal_maturity_curve.Draw("Shared Temporal Width/Thickness Maturity",
                                                                 temporal_settings);
    clamp_profile_distribution(needle_cross_section_temporal_maturity_curve, 1.0f);

    ImGui::TreePop();
  }

  // -- Needles --
  if (ImGui::TreeNodeEx("Core: Needles and Appearance", ImGuiTreeNodeFlags_DefaultOpen)) {
    changed |= bare_zone_fraction.Draw(
        "Bare Zone Fraction", 0.01f,
        "Temporal fraction at the start of each year that emits internode-only phytomers [0, 0.95).");
    changed |= needle_count_per_cluster.Draw("Needles per Fascicle Sheath", 0.5f);
    show_item_hover_description("Pinus sylvestris needles per fascicle sheath (typically 2).");
    if (ImGui::DragInt("Needle Segments", &needle_segment_count, 1.0f, 1, 128)) {
      needle_segment_count = std::clamp(needle_segment_count, 1, 128);
      changed = true;
    }
    show_item_hover_description(
        "Visible/logical longitudinal segments per needle. One segment is a straight strand from base to tip; the "
        "renderer pads hidden cubic controls so the visible needle starts and ends at the logical endpoints.");
    changed |= fascicle_sheath_length_m.Draw(
        "Fascicle Sheath Length (m)", 0.0005f,
        "Length of the persistent basal sheath cylinder that needles emerge from.");
    changed |= fascicle_sheath_width_m.Draw(
        "Fascicle Sheath Width (m)", 0.0001f, "Diameter of the persistent basal sheath cylinder.");
    changed |= needle_length_m.Draw("Needle Length (m)", 0.001f, "Length of needles in metres.");
    changed |= needle_lifespan_years.Draw("Needle Lifespan (years)", 0.1f,
                                          "Chronological years a needle stays alive post-maturity. Senescence "
                                          "is calendar-driven, NOT heat-sum-driven (FSPM Rule of Ontogeny). "
                                          "Scots pine typical: 3-4 yr.");
    changed |= needle_browning_years.Draw("Needle Browning (years)", 0.05f,
                                          "Chronological years from senescence onset to abscission.");
    changed |= needle_flush_delay_gdd.Draw("Needle Flush Delay (GDD)", 10.0f,
                                           "Delay from phytomer emergence to needle flush.");
    changed |= internode_maturation_gdd.Draw("Shoot Maturation (GDD)", 10.0f,
                                             "Thermal time from emergence to mature internode length.");
    changed |= needle_maturation_gdd.Draw("Needle Maturation (GDD)", 10.0f,
                                          "Thermal time from flush to mature needle length.");
    changed |= needle_branching_angle_deg.Draw("Needle Branching Angle (deg)", 0.25f,
                                               "Final branching angle from the parent axis reached after relaxation.");
    changed |= needle_branching_relax_gdd.Draw("Needle Branching Relaxation (GDD-equivalent)", 10.0f,
                                               "Converted using 1500 GDD/year, then applied against chronological "
                                               "age so relaxation continues during dormant season.");
    if (ImGui::DragFloat("Order Needle Length Attenuation", &needle_order_length_attenuation, 0.01f, 0.0f, 1.0f,
                         kScotsPineFloatFormat)) {
      needle_order_length_attenuation = std::clamp(needle_order_length_attenuation, 0.0f, 1.0f);
      changed = true;
    }
    show_item_hover_description(
        "Linear per-order reduction applied to needle length: scale = max(min, 1 - attenuation*order).");
    if (ImGui::DragFloat("Order Needle Radius Attenuation", &needle_order_radius_attenuation, 0.01f, 0.0f, 1.0f,
                         kScotsPineFloatFormat)) {
      needle_order_radius_attenuation = std::clamp(needle_order_radius_attenuation, 0.0f, 1.0f);
      changed = true;
    }
    show_item_hover_description(
        "Linear per-order reduction applied to needle thickness: scale = max(min, 1 - attenuation*order).");
    if (ImGui::DragFloat("Order Needle Length Min Scale", &needle_order_min_length_scale, 0.01f, 0.10f, 1.00f,
                         kScotsPineFloatFormat)) {
      needle_order_min_length_scale = std::clamp(needle_order_min_length_scale, 0.10f, 1.00f);
      changed = true;
    }
    show_item_hover_description("Lower bound for branch-order needle length scaling.");
    if (ImGui::DragFloat("Order Needle Radius Min Scale", &needle_order_min_radius_scale, 0.01f, 0.10f, 1.00f,
                         kScotsPineFloatFormat)) {
      needle_order_min_radius_scale = std::clamp(needle_order_min_radius_scale, 0.10f, 1.00f);
      changed = true;
    }
    show_item_hover_description("Lower bound for branch-order needle thickness scaling.");
    if (ImGui::TreeNodeEx("Advanced: Initiation Capacity Mapping")) {
      evo_engine::CurveDescriptorSettings capacity_curve_settings;
      capacity_curve_settings.speed = 0.01f;
      capacity_curve_settings.min_max_control = true;
      capacity_curve_settings.m_tip =
          "Mean multiplier applied to new needle length, width, and thickness by normalized seasonal phytomer index. "
          "Default ramps from 75% early-season capacity to 100% late-season capacity.";
      changed |= needle_intra_year_capacity_curve.Draw("Intra-Year Capacity Curve", capacity_curve_settings);
      needle_intra_year_capacity_curve.min_value = std::clamp(needle_intra_year_capacity_curve.min_value, 0.0f, 2.0f);
      needle_intra_year_capacity_curve.max_value = std::clamp(needle_intra_year_capacity_curve.max_value, 0.0f, 2.0f);
      if (needle_intra_year_capacity_curve.max_value < needle_intra_year_capacity_curve.min_value) {
        std::swap(needle_intra_year_capacity_curve.min_value, needle_intra_year_capacity_curve.max_value);
        changed = true;
      }
      show_item_hover_description("Runtime applies L_actual = L_max * curve(phytomer_index) * w_inter(year,vigor).");

      if (ImGui::DragFloat("Year 0 Length Multiplier", &needle_year0_length_multiplier, 0.01f, 0.0f, 8.0f,
                           kScotsPineFloatFormat)) {
        needle_year0_length_multiplier = std::max(0.0f, needle_year0_length_multiplier);
        changed = true;
      }
      show_item_hover_description("Year-0 multiplier for needle target length. Later cohorts use 1.");

      if (ImGui::DragFloat("Year 0 Lignification Factor", &needle_lignification_factor_year0, 0.01f, 0.0f, 2.0f,
                           kScotsPineFloatFormat)) {
        needle_lignification_factor_year0 = std::clamp(needle_lignification_factor_year0, 0.0f, 2.0f);
        changed = true;
      }
      show_item_hover_description("Scales visual maturation response for year-0 needle cohorts. Later cohorts use 1.");

      if (ImGui::DragFloat("Year 0 Stomatal Strip Density", &needle_stomatal_strip_density_year0, 0.01f, 0.0f, 1.0f,
                           kScotsPineFloatFormat)) {
        needle_stomatal_strip_density_year0 = std::clamp(needle_stomatal_strip_density_year0, 0.0f, 1.0f);
        changed = true;
      }
      show_item_hover_description("Proxy density for procedural stomatal striping in year-0 cohorts. Later cohorts use 1.");

      if (ImGui::DragFloat("Year 0 Basal Taper Ratio", &needle_basal_taper_ratio_year0, 0.01f, 0.6f, 1.2f,
                           kScotsPineFloatFormat)) {
        needle_basal_taper_ratio_year0 = std::clamp(needle_basal_taper_ratio_year0, 0.6f, 1.2f);
        changed = true;
      }
      show_item_hover_description("Needle-base radius multiplier for year-0 cohorts. Later cohorts use 1.");

      if (ImGui::DragFloat("Fascicle Sheath Budget (GDD)", &needle_fascicle_sheath_budget_gdd, 10.0f, 0.0f, 5000.0f,
                           kScotsPineFloatFormat)) {
        needle_fascicle_sheath_budget_gdd = std::max(0.0f, needle_fascicle_sheath_budget_gdd);
        changed = true;
      }
      show_item_hover_description("Characteristic thermal budget for sheath maturation near needle bases.");

      if (ImGui::DragFloat("Year 0 Specularity Plasticity", &needle_specularity_plasticity_year0, 0.01f, 0.0f, 1.0f,
                           kScotsPineFloatFormat)) {
        needle_specularity_plasticity_year0 = std::clamp(needle_specularity_plasticity_year0, 0.0f, 1.0f);
        changed = true;
      }
      show_item_hover_description("How strongly year-0 micro-variation tracks maturity cues. Later cohorts use 1.");

      if (ImGui::DragFloat("Bud-Storage Vigor Strength", &needle_bud_storage_vigor_strength, 0.01f, 0.0f, 1.0f,
                           kScotsPineFloatFormat)) {
        needle_bud_storage_vigor_strength = std::clamp(needle_bud_storage_vigor_strength, 0.0f, 1.0f);
        changed = true;
      }
      show_item_hover_description("Blend factor from 1.0 to previous-season vigor proxy for inter-year capacity.");

      if (ImGui::DragFloat("Bud-Storage Completion Floor", &needle_bud_storage_completion_floor, 0.01f, 0.0f, 1.0f,
                           kScotsPineFloatFormat)) {
        needle_bud_storage_completion_floor = std::clamp(needle_bud_storage_completion_floor, 0.0f, 1.0f);
        changed = true;
      }
      show_item_hover_description("Lower clamp applied to completion ratio before vigor carry-over.");
      ImGui::TreePop();
    }
    if (ImGui::DragFloat("Needle Old Thinning Fraction", &needle_old_thinning_fraction, 0.01f, 0.0f, 0.95f, kScotsPineFloatFormat)) {
      needle_old_thinning_fraction = std::clamp(needle_old_thinning_fraction, 0.0f, 0.95f);
      changed = true;
    }
    show_item_hover_description("Fractional strand-radius reduction applied as needle segments reach old color.");
    if (ImGui::DragFloat("Needle Min Strand Thickness M", &needle_min_strand_thickness_m, 0.00001f, 0.000001f,
                         0.002f, kScotsPineFloatFormat)) {
      needle_min_strand_thickness_m = std::clamp(needle_min_strand_thickness_m, 0.000001f, 0.002f);
      changed = true;
    }
    show_item_hover_description("Minimum rendered radius for needle strand points.");
    ImGui::TreePop();
  }

  // -- Needle curvature (bilateral differential growth field) --
  if (ImGui::TreeNodeEx("Needle Shape (Curvature Field)")) {
    changed |= needle_curvature_adaxial_bias.Draw(
        "Adaxial Elongation Bias", 0.001f, "Dimensionless adaxial side elongation. Positive bends needle toward stem.");
    changed |= needle_curvature_abaxial_bias.Draw(
        "Abaxial Elongation Bias", 0.001f,
        "Dimensionless abaxial side elongation. Positive bends needle away from stem.");
    changed |= needle_curvature_gradient_per_arclen.Draw(
        "Curvature Gradient (per s_norm)", 0.001f,
        "Linear gradient added to (abaxial - adaxial) along normalized arc length.");
    changed |= needle_diameter_for_curvature_m.Draw(
        "Effective Diameter (m)", 0.0001f,
        "Cross-section diameter used to convert strain differential into curvature. "
        "Set > 0 to activate the field.");
    changed |= needle_sinusoidal_amplitude_deg.Draw(
        "Sinusoidal Wave Amplitude (deg)", 0.10f,
        "Additional intrinsic waviness amplitude applied along the needle; 0 keeps arc-only behavior.");
    changed |= needle_sinusoidal_frequency_cycles.Draw("Sinusoidal Wave Frequency (cycles)", 0.05f,
                                                       "Number of waviness cycles along full needle length.");
    changed |= needle_sinusoidal_phase_randomness_deg.Draw(
        "Sinusoidal Phase Randomness (deg)", 0.10f,
        "Sampled phase jitter magnitude combined with deterministic per-needle phase.");
    ImGui::TreePop();
  }

  // -- Needle mechanics (elastica) --
  if (ImGui::TreeNodeEx("Needle Mechanics (Elastica)")) {
    changed |= needle_young_modulus_baseline_Pa.Draw("Young's Modulus Baseline (Pa)", 1e6f,
                                                     "Asymptotic Young's modulus at maturity. 0 = solver disabled.");
    changed |= needle_lignification_maturation_years.Draw("Lignification Maturation (yr)", 0.05f,
                                                          "Sigmoid maturation duration for E(t).");
    changed |= needle_density_kg_m3.Draw("Tissue Density (kg/m^3)", 10.0f,
                                         "Used to derive distributed weight per unit arc length.");
    changed |= gravity_m_s2.Draw("Gravity (m/s^2)", 0.1f, "World-frame gravity magnitude. 0 = no body force.");
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Needle Per-Needle Variability")) {
    changed |= needle_per_needle_length_cv.Draw("Length CV", 0.01f,
                                                "CV-style variation across needles within a fascicle sheath for length scale.");
    changed |= needle_per_needle_curvature_cv.Draw(
        "Curvature CV", 0.01f, "CV-style variation across needles within a fascicle sheath for curvature-field magnitude.");
    changed |= needle_per_needle_radius_cv.Draw(
        "Radius CV", 0.01f, "CV-style variation across needles within a fascicle sheath for cross-section axis scale.");
    changed |= needle_per_needle_modulus_cv.Draw(
        "Young's Modulus CV", 0.01f,
        "CV-style variation across needles within a fascicle sheath for baseline Young's modulus.");
    changed |= needle_per_needle_density_cv.Draw(
        "Density CV", 0.01f, "CV-style variation across needles within a fascicle sheath for tissue density.");
    changed |= needle_per_needle_wave_amplitude_cv.Draw(
        "Wave Amplitude CV", 0.01f,
        "CV-style variation across needles within a fascicle sheath for sinusoidal waviness amplitude.");
    changed |= needle_per_needle_wave_frequency_cv.Draw(
        "Wave Frequency CV", 0.01f,
        "CV-style variation across needles within a fascicle sheath for sinusoidal waviness frequency.");
    changed |= needle_per_needle_wave_phase_cv.Draw("Wave Phase CV", 0.01f,
                                                    "CV-style scaling of per-needle sinusoidal phase randomness.");
    ImGui::TreePop();
  }

  // -- Tropism --
  if (ImGui::TreeNodeEx("Tropism (Global)")) {
    changed |= gravitropism_first_order.Draw(
        "Main Stem Tropism (deg/GDD)", 0.0001f,
        "Per-GDD curvature applied to leader internodes only (branch order 0). Positive bends upward.");
    ImGui::TreePop();
  }

  // -- Per-shoot stochastic noise --
  if (ImGui::TreeNodeEx("Stochastic Variation (Per Shoot)")) {
    changed |= internode_length_per_node_cv.Draw("Internode Length CV", 0.005f,
                                                 "Per-internode Gaussian CV on phytomer length. 0 = deterministic.");
    changed |= internode_thickness_per_node_cv.Draw("Internode Thickness CV", 0.005f,
                                                    "Per-internode Gaussian CV on shoot thickness. 0 = deterministic.");
    changed |= branch_angle_per_node_sigma_deg.Draw("Branch Angle Sigma (deg)", 0.5f,
                                                    "Per-lateral additive Gaussian sigma on insertion angle.");
    changed |= roll_phyllotaxis_per_node_sigma_deg.Draw("Roll Phyllotaxis Sigma (deg)", 0.5f,
                                                        "Per-lateral additive Gaussian sigma on phyllotaxis roll.");
    ImGui::TreePop();
  }

  if (editor_preferences_changed) {
    // Editor preferences are persisted via Serialize/Deserialize; they don't
    // mark the asset content "changed" for revision tracking.
  }

  ClampMaterialModel(*this);
  return changed;
}

// ===========================================================================
// Serialize
// ===========================================================================
void l_system_package::SerializeScotsPineDescriptor(YAML::Emitter& out, const ScotsPineDescriptor& target) {
  // Phytomer scheduling.
  target.max_branching_order.Save("max_branching_order", out);
  target.plastochron_gdd.Save("plastochron_gdd", out);
  target.max_phytomers_per_seasonal_growth.Save("max_phytomers_per_seasonal_growth", out);

  // Whorl architecture.
  target.annual_whorl_probability.Save("annual_whorl_probability", out);
  target.whorl_position_norm.Save("whorl_position_norm", out);
  target.branches_per_whorl.Save("branches_per_whorl", out);
  target.whorl_dormancy_years.Save("whorl_dormancy_years", out);
  target.branch_insertion_angle_deg.Save("branch_insertion_angle_deg", out);
  target.branch_roll_phyllotaxis_deg.Save("branch_roll_phyllotaxis_deg", out);

  // Phytomer dimensions.
  target.internode_length_m.Save("internode_length_m", out);
  target.leader_internode_thickness_m.Save("leader_internode_thickness_m", out);
  target.lateral_length_ratio.Save("lateral_length_ratio", out);
  target.lateral_thickness_ratio.Save("lateral_thickness_ratio", out);
  out << YAML::Key << "internode_age_exponent" << YAML::Value << target.internode_age_exponent;

  // Descriptor-native material model.
  out << YAML::Key << "material_profile_version" << YAML::Value << target.material_profile_version;
  out << YAML::Key << "material_profile_source" << YAML::Value << target.material_profile_source;
  out << YAML::Key << "material_profile_hash" << YAML::Value << target.material_profile_hash;
  out << YAML::Key << "biological_material_model_strength" << YAML::Value
      << target.biological_material_model_strength;
  out << YAML::Key << "biological_chlorophyll_scale" << YAML::Value << target.biological_chlorophyll_scale;
  out << YAML::Key << "biological_carotenoid_gold_scale" << YAML::Value
      << target.biological_carotenoid_gold_scale;
  out << YAML::Key << "biological_lignin_bark_scale" << YAML::Value << target.biological_lignin_bark_scale;
  out << YAML::Key << "biological_senescence_bias" << YAML::Value << target.biological_senescence_bias;
  out << YAML::Key << "biological_cuticle_wax" << YAML::Value << target.biological_cuticle_wax;
  out << YAML::Key << "biological_individual_variation" << YAML::Value
      << target.biological_individual_variation;
  out << YAML::Key << "biological_facet_contrast" << YAML::Value << target.biological_facet_contrast;
  out << YAML::Key << "biological_tip_darkening_strength" << YAML::Value
      << target.biological_tip_darkening_strength;
  out << YAML::Key << "biological_stem_age_browning_scale" << YAML::Value
      << target.biological_stem_age_browning_scale;
  out << YAML::Key << "young_needle_palette_rgba" << YAML::Value << target.young_needle_palette_rgba;
  out << YAML::Key << "older_needle_palette_rgba" << YAML::Value << target.older_needle_palette_rgba;
  out << YAML::Key << "dry_brown_needle_palette_rgba" << YAML::Value << target.dry_brown_needle_palette_rgba;
  out << YAML::Key << "main_stem_palette_rgba" << YAML::Value << target.main_stem_palette_rgba;
  out << YAML::Key << "mature_bark_stem_palette_rgba" << YAML::Value << target.mature_bark_stem_palette_rgba;
  out << YAML::Key << "node_sheath_brown_palette_rgba" << YAML::Value << target.node_sheath_brown_palette_rgba;
  out << YAML::Key << "fascicle_sheath_palette_rgba" << YAML::Value << target.fascicle_sheath_palette_rgba;
  target.needle_axial_color_curve.Save("needle_axial_color_curve", out);
  target.needle_y_age_color_curve.Save("needle_y_age_color_curve", out);
  target.stem_age_gradient_curve.Save("stem_age_gradient_curve", out);
  out << YAML::Key << "needle_micro_variation" << YAML::Value << target.needle_micro_variation;
  out << YAML::Key << "stem_micro_variation" << YAML::Value << target.stem_micro_variation;
  out << YAML::Key << "young_needle_roughness" << YAML::Value << target.young_needle_roughness;
  out << YAML::Key << "old_needle_roughness" << YAML::Value << target.old_needle_roughness;
  out << YAML::Key << "young_needle_specular" << YAML::Value << target.young_needle_specular;
  out << YAML::Key << "old_needle_specular" << YAML::Value << target.old_needle_specular;
  out << YAML::Key << "stem_roughness" << YAML::Value << target.stem_roughness;
  out << YAML::Key << "stem_specular" << YAML::Value << target.stem_specular;
  out << YAML::Key << "node_browning_strength" << YAML::Value << target.node_browning_strength;
  out << YAML::Key << "sheath_browning_strength" << YAML::Value << target.sheath_browning_strength;
  out << YAML::Key << "node_browning_radius_norm" << YAML::Value << target.node_browning_radius_norm;
  out << YAML::Key << "needle_twist_turns" << YAML::Value << target.needle_twist_turns;
  out << YAML::Key << "needle_edge_darkening" << YAML::Value << target.needle_edge_darkening;
  out << YAML::Key << "needle_tip_color_mix_start" << YAML::Value << target.needle_tip_color_mix_start;
  out << YAML::Key << "needle_tip_color_exponent" << YAML::Value << target.needle_tip_color_exponent;
  out << YAML::Key << "needle_old_thinning_fraction" << YAML::Value << target.needle_old_thinning_fraction;
  out << YAML::Key << "needle_min_strand_thickness_m" << YAML::Value << target.needle_min_strand_thickness_m;
  out << YAML::Key << "needle_axial_age_span" << YAML::Value << target.needle_axial_age_span;
  out << YAML::Key << "needle_axial_age_exponent" << YAML::Value << target.needle_axial_age_exponent;

  // Needles.
  target.bare_zone_fraction.Save("bare_zone_fraction", out);
  target.needle_count_per_cluster.Save("needle_count_per_cluster", out);
  out << YAML::Key << "needle_segment_count" << YAML::Value << target.needle_segment_count;
  target.fascicle_sheath_length_m.Save("fascicle_sheath_length_m", out);
  target.fascicle_sheath_width_m.Save("fascicle_sheath_width_m", out);
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
  target.needle_intra_year_capacity_curve.Save("needle_intra_year_capacity_curve", out);
  out << YAML::Key << "needle_year0_length_multiplier" << YAML::Value << target.needle_year0_length_multiplier;
  out << YAML::Key << "needle_lignification_factor_year0" << YAML::Value << target.needle_lignification_factor_year0;
  out << YAML::Key << "needle_stomatal_strip_density_year0" << YAML::Value
      << target.needle_stomatal_strip_density_year0;
  out << YAML::Key << "needle_basal_taper_ratio_year0" << YAML::Value << target.needle_basal_taper_ratio_year0;
  out << YAML::Key << "needle_fascicle_sheath_budget_gdd" << YAML::Value << target.needle_fascicle_sheath_budget_gdd;
  out << YAML::Key << "needle_specularity_plasticity_year0" << YAML::Value
      << target.needle_specularity_plasticity_year0;
  out << YAML::Key << "needle_bud_storage_vigor_strength" << YAML::Value << target.needle_bud_storage_vigor_strength;
  out << YAML::Key << "needle_bud_storage_completion_floor" << YAML::Value
      << target.needle_bud_storage_completion_floor;

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

}

// ===========================================================================
// Deserialize
// ===========================================================================
void l_system_package::DeserializeScotsPineDescriptor(const YAML::Node& in, ScotsPineDescriptor& target) {
  ConfigurePineMaturityDefaults(target);

  LoadSingleDistributionWithScalarFallback(in, "max_branching_order", target.max_branching_order);
  LoadSingleDistributionWithScalarFallback(in, "plastochron_gdd", target.plastochron_gdd);
  LoadSingleDistributionWithScalarFallback(in, "max_phytomers_per_seasonal_growth",
                                           target.max_phytomers_per_seasonal_growth);

  LoadSingleDistributionWithScalarFallback(in, "annual_whorl_probability", target.annual_whorl_probability);
  LoadSingleDistributionWithScalarFallback(in, "whorl_position_norm", target.whorl_position_norm);
  LoadSingleDistributionWithScalarFallback(in, "branches_per_whorl", target.branches_per_whorl);
  LoadSingleDistributionWithScalarFallback(in, "whorl_dormancy_years", target.whorl_dormancy_years);
  LoadSingleDistributionWithScalarFallback(in, "branch_insertion_angle_deg", target.branch_insertion_angle_deg);
  LoadSingleDistributionWithScalarFallback(in, "branch_roll_phyllotaxis_deg", target.branch_roll_phyllotaxis_deg);

  LoadSingleDistributionWithScalarFallback(in, "internode_length_m", target.internode_length_m);
  LoadSingleDistributionWithScalarFallback(in, "leader_internode_thickness_m", target.leader_internode_thickness_m);
  LoadSingleDistributionWithScalarFallback(in, "lateral_length_ratio", target.lateral_length_ratio);
  LoadSingleDistributionWithScalarFallback(in, "lateral_thickness_ratio", target.lateral_thickness_ratio);
  if (in["internode_age_exponent"]) {
    target.internode_age_exponent = std::clamp(in["internode_age_exponent"].as<float>(), 0.1f, 4.0f);
  }

  LoadSingleDistributionWithScalarFallback(in, "bare_zone_fraction", target.bare_zone_fraction);
  LoadSingleDistributionWithScalarFallback(in, "needle_count_per_cluster", target.needle_count_per_cluster);
  if (in["needle_segment_count"]) {
    target.needle_segment_count = std::clamp(in["needle_segment_count"].as<int>(), 1, 128);
  }
  LoadSingleDistributionWithScalarFallback(in, "fascicle_sheath_length_m", target.fascicle_sheath_length_m);
  LoadSingleDistributionWithScalarFallback(in, "fascicle_sheath_width_m", target.fascicle_sheath_width_m);
  LoadSingleDistributionWithScalarFallback(in, "needle_length_m", target.needle_length_m);
  LoadSingleDistributionWithScalarFallback(in, "needle_lifespan_years", target.needle_lifespan_years);
  LoadSingleDistributionWithScalarFallback(in, "needle_browning_years", target.needle_browning_years);
  LoadSingleDistributionWithScalarFallback(in, "needle_flush_delay_gdd", target.needle_flush_delay_gdd);
  LoadSingleDistributionWithScalarFallback(in, "internode_maturation_gdd", target.internode_maturation_gdd);
  LoadSingleDistributionWithScalarFallback(in, "needle_maturation_gdd", target.needle_maturation_gdd);
  LoadSingleDistributionWithScalarFallback(in, "needle_branching_angle_deg", target.needle_branching_angle_deg);
  LoadSingleDistributionWithScalarFallback(in, "needle_branching_relax_gdd", target.needle_branching_relax_gdd);
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

  if (!has_new_cross_section_width_profile) {
    ConfigureNeedleCrossSectionProfileDefaults(target.needle_cross_section_width_profile, 1.0f, 0.36f);
  }
  if (!has_new_cross_section_thickness_profile) {
    ConfigureNeedleCrossSectionProfileDefaults(target.needle_cross_section_thickness_profile, 1.0f, 0.36f);
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
  if (in["needle_intra_year_capacity_curve"]) {
    target.needle_intra_year_capacity_curve.Load("needle_intra_year_capacity_curve", in);
  }
  target.needle_intra_year_capacity_curve.min_value =
      std::clamp(target.needle_intra_year_capacity_curve.min_value, 0.0f, 2.0f);
  target.needle_intra_year_capacity_curve.max_value =
      std::clamp(target.needle_intra_year_capacity_curve.max_value, 0.0f, 2.0f);
  if (target.needle_intra_year_capacity_curve.max_value < target.needle_intra_year_capacity_curve.min_value) {
    std::swap(target.needle_intra_year_capacity_curve.min_value, target.needle_intra_year_capacity_curve.max_value);
  }
  if (in["needle_year0_length_multiplier"]) {
    target.needle_year0_length_multiplier = std::max(0.0f, in["needle_year0_length_multiplier"].as<float>());
  }
  if (in["needle_lignification_factor_year0"]) {
    target.needle_lignification_factor_year0 =
        std::clamp(in["needle_lignification_factor_year0"].as<float>(), 0.0f, 2.0f);
  }
  if (in["needle_stomatal_strip_density_year0"]) {
    target.needle_stomatal_strip_density_year0 =
        std::clamp(in["needle_stomatal_strip_density_year0"].as<float>(), 0.0f, 1.0f);
  }
  if (in["needle_basal_taper_ratio_year0"]) {
    target.needle_basal_taper_ratio_year0 = std::clamp(in["needle_basal_taper_ratio_year0"].as<float>(), 0.6f, 1.2f);
  }
  if (in["needle_fascicle_sheath_budget_gdd"]) {
    target.needle_fascicle_sheath_budget_gdd = std::max(0.0f, in["needle_fascicle_sheath_budget_gdd"].as<float>());
  }
  if (in["needle_specularity_plasticity_year0"]) {
    target.needle_specularity_plasticity_year0 =
        std::clamp(in["needle_specularity_plasticity_year0"].as<float>(), 0.0f, 1.0f);
  }
  if (in["needle_bud_storage_vigor_strength"]) {
    target.needle_bud_storage_vigor_strength =
        std::clamp(in["needle_bud_storage_vigor_strength"].as<float>(), 0.0f, 1.0f);
  }
  if (in["needle_bud_storage_completion_floor"]) {
    target.needle_bud_storage_completion_floor =
        std::clamp(in["needle_bud_storage_completion_floor"].as<float>(), 0.0f, 1.0f);
  }
  if (in["needle_tip_color_mix_start"]) {
    target.needle_tip_color_mix_start = std::clamp(in["needle_tip_color_mix_start"].as<float>(), 0.0f, 1.0f);
  }
  if (in["needle_tip_color_exponent"]) {
    target.needle_tip_color_exponent = std::clamp(in["needle_tip_color_exponent"].as<float>(), 0.1f, 6.0f);
  }
  if (in["needle_old_thinning_fraction"]) {
    target.needle_old_thinning_fraction = std::clamp(in["needle_old_thinning_fraction"].as<float>(), 0.0f, 0.95f);
  }
  if (in["needle_min_strand_thickness_m"]) {
    target.needle_min_strand_thickness_m =
        std::clamp(in["needle_min_strand_thickness_m"].as<float>(), 0.000001f, 0.002f);
  }
  if (in["needle_axial_age_span"]) {
    target.needle_axial_age_span = std::clamp(in["needle_axial_age_span"].as<float>(), -1.0f, 1.0f);
  }
  if (in["needle_axial_age_exponent"]) {
    target.needle_axial_age_exponent = std::clamp(in["needle_axial_age_exponent"].as<float>(), 0.1f, 4.0f);
  }

  if (in["material_profile_version"]) {
    target.material_profile_version = std::max(1, in["material_profile_version"].as<int>());
  }
  if (in["material_profile_source"]) {
    target.material_profile_source = in["material_profile_source"].as<std::string>();
  }
  if (in["material_profile_hash"]) {
    target.material_profile_hash = in["material_profile_hash"].as<std::string>();
  }
  if (in["biological_material_model_strength"]) {
    target.biological_material_model_strength = in["biological_material_model_strength"].as<float>();
  }
  if (in["biological_chlorophyll_scale"]) {
    target.biological_chlorophyll_scale = in["biological_chlorophyll_scale"].as<float>();
  }
  if (in["biological_carotenoid_gold_scale"]) {
    target.biological_carotenoid_gold_scale = in["biological_carotenoid_gold_scale"].as<float>();
  }
  if (in["biological_lignin_bark_scale"]) {
    target.biological_lignin_bark_scale = in["biological_lignin_bark_scale"].as<float>();
  }
  if (in["biological_senescence_bias"]) {
    target.biological_senescence_bias = in["biological_senescence_bias"].as<float>();
  }
  if (in["biological_cuticle_wax"]) {
    target.biological_cuticle_wax = in["biological_cuticle_wax"].as<float>();
  }
  if (in["biological_individual_variation"]) {
    target.biological_individual_variation = in["biological_individual_variation"].as<float>();
  }
  if (in["biological_facet_contrast"]) {
    target.biological_facet_contrast = in["biological_facet_contrast"].as<float>();
  }
  if (in["biological_tip_darkening_strength"]) {
    target.biological_tip_darkening_strength = in["biological_tip_darkening_strength"].as<float>();
  }
  if (in["biological_stem_age_browning_scale"]) {
    target.biological_stem_age_browning_scale = in["biological_stem_age_browning_scale"].as<float>();
  }
  if (in["young_needle_palette_rgba"]) {
    target.young_needle_palette_rgba = in["young_needle_palette_rgba"].as<glm::vec4>();
  }
  if (in["older_needle_palette_rgba"]) {
    target.older_needle_palette_rgba = in["older_needle_palette_rgba"].as<glm::vec4>();
  }
  if (in["dry_brown_needle_palette_rgba"]) {
    target.dry_brown_needle_palette_rgba = in["dry_brown_needle_palette_rgba"].as<glm::vec4>();
  }
  if (in["main_stem_palette_rgba"]) {
    target.main_stem_palette_rgba = in["main_stem_palette_rgba"].as<glm::vec4>();
  }
  if (in["mature_bark_stem_palette_rgba"]) {
    target.mature_bark_stem_palette_rgba = in["mature_bark_stem_palette_rgba"].as<glm::vec4>();
  }
  if (in["node_sheath_brown_palette_rgba"]) {
    target.node_sheath_brown_palette_rgba = in["node_sheath_brown_palette_rgba"].as<glm::vec4>();
  }
  if (in["fascicle_sheath_palette_rgba"]) {
    target.fascicle_sheath_palette_rgba = in["fascicle_sheath_palette_rgba"].as<glm::vec4>();
  } else {
    target.fascicle_sheath_palette_rgba = target.node_sheath_brown_palette_rgba;
  }
  if (in["needle_axial_color_curve"]) {
    target.needle_axial_color_curve.Load("needle_axial_color_curve", in);
  }
  if (in["needle_y_age_color_curve"]) {
    target.needle_y_age_color_curve.Load("needle_y_age_color_curve", in);
  }
  if (in["stem_age_gradient_curve"]) {
    target.stem_age_gradient_curve.Load("stem_age_gradient_curve", in);
  }
  if (in["needle_micro_variation"]) {
    target.needle_micro_variation = in["needle_micro_variation"].as<float>();
  }
  if (in["stem_micro_variation"]) {
    target.stem_micro_variation = in["stem_micro_variation"].as<float>();
  }
  if (in["young_needle_roughness"]) {
    target.young_needle_roughness = in["young_needle_roughness"].as<float>();
  }
  if (in["old_needle_roughness"]) {
    target.old_needle_roughness = in["old_needle_roughness"].as<float>();
  }
  if (in["young_needle_specular"]) {
    target.young_needle_specular = in["young_needle_specular"].as<float>();
  }
  if (in["old_needle_specular"]) {
    target.old_needle_specular = in["old_needle_specular"].as<float>();
  }
  if (in["stem_roughness"]) {
    target.stem_roughness = in["stem_roughness"].as<float>();
  }
  if (in["stem_specular"]) {
    target.stem_specular = in["stem_specular"].as<float>();
  }
  if (in["node_browning_strength"]) {
    target.node_browning_strength = in["node_browning_strength"].as<float>();
  }
  if (in["sheath_browning_strength"]) {
    target.sheath_browning_strength = in["sheath_browning_strength"].as<float>();
  }
  if (in["node_browning_radius_norm"]) {
    target.node_browning_radius_norm = in["node_browning_radius_norm"].as<float>();
  }
  if (in["needle_twist_turns"]) {
    target.needle_twist_turns = in["needle_twist_turns"].as<float>();
  }
  if (in["needle_edge_darkening"]) {
    target.needle_edge_darkening = in["needle_edge_darkening"].as<float>();
  }
  ClampMaterialModel(target);

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

  target.gdd_per_day.mean = std::max(0.0f, target.gdd_per_day.mean);
  target.gdd_per_day.deviation = std::max(0.0f, target.gdd_per_day.deviation);

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

}

// ===========================================================================
// ParamSpaceExplorer axis registration.
// ===========================================================================
void ScotsPineDescriptor::RegisterExplorableAxes(ParamSpaceExplorer& explorer) {
  auto& d = *this;

  // -- Phytomer scheduling --
  explorer.AddSingle("max_branching_order", "MBO", d.max_branching_order, 0.0f, 4.0f, 2.0f);
  explorer.AddSingle("plastochron_gdd", "PLG", d.plastochron_gdd, 50.0f, 6000.0f, 1500.0f);
  explorer.AddSingle("max_phytomers_per_seasonal_growth", "MPS", d.max_phytomers_per_seasonal_growth, 1.0f, 64.0f,
                     12.0f);

  // -- Whorl architecture --
  explorer.AddSingle("annual_whorl_probability", "AWP", d.annual_whorl_probability, 0.0f, 1.0f, 0.22f);
  explorer.AddSingle("whorl_position_norm", "WPN", d.whorl_position_norm, 0.75f, 1.0f, 0.92f);
  explorer.AddSingle("branches_per_whorl", "BPW", d.branches_per_whorl, 1.0f, 5.0f, 2.0f);
  explorer.AddSingle("whorl_dormancy_years", "WDY", d.whorl_dormancy_years, 0.0f, 4.0f, 1.0f);
  explorer.AddSingle("branch_insertion_angle_deg", "BIA", d.branch_insertion_angle_deg, -85.0f, 85.0f, 60.0f);
  explorer.AddSingle("branch_roll_phyllotaxis_deg", "BRP", d.branch_roll_phyllotaxis_deg, 0.0f, 360.0f, 137.5f);

  // -- Phytomer dimensions --
  explorer.AddSingle("internode_length_m", "ILM", d.internode_length_m, 0.0001f, 0.500f, 0.012f);
  explorer.AddSingle("leader_internode_thickness_m", "LIT", d.leader_internode_thickness_m, 0.0001f, 0.0500f,
                     0.0030f);
  explorer.AddSingle("lateral_length_ratio", "LLR", d.lateral_length_ratio, 0.1f, 1.5f, 0.7f);
  explorer.AddSingle("lateral_thickness_ratio", "LTR", d.lateral_thickness_ratio, 0.1f, 1.5f, 0.6f);

  // -- Needles --
  explorer.AddSingle("bare_zone_fraction", "BZF", d.bare_zone_fraction, 0.0f, 0.95f, 0.0f);
  explorer.AddSingle("needle_count_per_cluster", "NCC", d.needle_count_per_cluster, 1.0f, 6.0f, 2.0f);
  explorer.AddSingle("fascicle_sheath_length_m", "FSL", d.fascicle_sheath_length_m, 0.0005f, 0.030f, 0.006f);
  explorer.AddSingle("fascicle_sheath_width_m", "FSW", d.fascicle_sheath_width_m, 0.0002f, 0.010f, 0.0012f);
  {
    auto* value_ptr = &d.needle_segment_count;
    explorer.AddAxis(
        "needle_segment_count", "NSG", 1.0f, 128.0f,
        [value_ptr]() {
          return static_cast<float>(*value_ptr);
        },
        [value_ptr](float value) {
          *value_ptr = std::clamp(static_cast<int>(std::round(value)), 1, 128);
        });
  }
  explorer.AddSingle("needle_length_m", "NLM", d.needle_length_m, 0.001f, 0.200f, 0.025f);
  explorer.AddSingle("needle_lifespan_years", "NLY", d.needle_lifespan_years, 0.0f, 10.0f, 4.0f);
  explorer.AddSingle("needle_browning_years", "NBY", d.needle_browning_years, 0.0f, 4.0f, 1.0f);
  explorer.AddSingle("needle_flush_delay_gdd", "NFD", d.needle_flush_delay_gdd, 0.0f, 3000.0f, 0.0f);
  explorer.AddSingle("internode_maturation_gdd", "IMG", d.internode_maturation_gdd, 0.0f, 3000.0f, 60.0f);
  explorer.AddSingle("needle_maturation_gdd", "NMG", d.needle_maturation_gdd, 0.0f, 6000.0f, 120.0f);
  explorer.AddSingle("needle_branching_angle_deg", "NBA", d.needle_branching_angle_deg, 0.0f, 179.5f, 72.0f);
  explorer.AddSingle("needle_branching_relax_gdd", "NRG", d.needle_branching_relax_gdd, 0.0f, 6000.0f, 220.0f);
  explorer.AddPlotted("internode_length_maturity_curve", "ILC", d.internode_length_maturity_curve);
  explorer.AddPlotted("internode_width_maturity_curve", "IWC", d.internode_width_maturity_curve);
  explorer.AddPlotted("needle_length_maturity_curve", "NLC", d.needle_length_maturity_curve);
  explorer.AddSingle("needle_cross_section_width_max_m", "NCW", d.needle_cross_section_width_max_m, 0.0f, 0.02f,
                     0.0018f);
  explorer.AddSingle("needle_cross_section_thickness_max_m", "NCT", d.needle_cross_section_thickness_max_m, 0.0f, 0.02f,
                     0.0011f);
  explorer.AddPlotted("needle_cross_section_width_profile", "NWP", d.needle_cross_section_width_profile);
  explorer.AddPlotted("needle_cross_section_thickness_profile", "NTP", d.needle_cross_section_thickness_profile);
  explorer.AddPlotted("needle_cross_section_temporal_maturity_curve", "NTM",
                      d.needle_cross_section_temporal_maturity_curve);
  {
    auto* value_ptr = &d.needle_intra_year_capacity_curve.min_value;
    explorer.AddAxis(
        "needle_intra_year_capacity_curve.min", "NICm", 0.0f, 2.0f,
        [value_ptr]() {
          return *value_ptr;
        },
        [value_ptr](float value) {
          *value_ptr = std::clamp(value, 0.0f, 2.0f);
        });
  }
  {
    auto* value_ptr = &d.needle_intra_year_capacity_curve.max_value;
    explorer.AddAxis(
        "needle_intra_year_capacity_curve.max", "NICx", 0.0f, 2.0f,
        [value_ptr]() {
          return *value_ptr;
        },
        [value_ptr](float value) {
          *value_ptr = std::clamp(value, 0.0f, 2.0f);
        });
  }
  explorer.AddCurve("needle_intra_year_capacity_curve.curve", "NIC", d.needle_intra_year_capacity_curve.curve);
  {
    auto* value_ptr = &d.needle_year0_length_multiplier;
    explorer.AddAxis(
        "needle_year0_length_multiplier", "N0L", 0.0f, 8.0f,
        [value_ptr]() {
          return *value_ptr;
        },
        [value_ptr](float value) {
          *value_ptr = std::max(0.0f, value);
        });
  }
  {
    auto* value_ptr = &d.needle_lignification_factor_year0;
    explorer.AddAxis(
        "needle_lignification_factor_year0", "NL0", 0.0f, 2.0f,
        [value_ptr]() {
          return *value_ptr;
        },
        [value_ptr](float value) {
          *value_ptr = std::clamp(value, 0.0f, 2.0f);
        });
  }
  {
    auto* value_ptr = &d.needle_stomatal_strip_density_year0;
    explorer.AddAxis(
        "needle_stomatal_strip_density_year0", "NS0", 0.0f, 1.0f,
        [value_ptr]() {
          return *value_ptr;
        },
        [value_ptr](float value) {
          *value_ptr = std::clamp(value, 0.0f, 1.0f);
        });
  }
  {
    auto* value_ptr = &d.needle_basal_taper_ratio_year0;
    explorer.AddAxis(
        "needle_basal_taper_ratio_year0", "NB0", 0.6f, 1.2f,
        [value_ptr]() {
          return *value_ptr;
        },
        [value_ptr](float value) {
          *value_ptr = std::clamp(value, 0.6f, 1.2f);
        });
  }
  {
    auto* value_ptr = &d.needle_fascicle_sheath_budget_gdd;
    explorer.AddAxis(
        "needle_fascicle_sheath_budget_gdd", "NSB", 0.0f, 5000.0f,
        [value_ptr]() {
          return *value_ptr;
        },
        [value_ptr](float value) {
          *value_ptr = std::max(0.0f, value);
        });
  }
  {
    auto* value_ptr = &d.needle_specularity_plasticity_year0;
    explorer.AddAxis(
        "needle_specularity_plasticity_year0", "NP0", 0.0f, 1.0f,
        [value_ptr]() {
          return *value_ptr;
        },
        [value_ptr](float value) {
          *value_ptr = std::clamp(value, 0.0f, 1.0f);
        });
  }
  {
    auto* value_ptr = &d.needle_bud_storage_vigor_strength;
    explorer.AddAxis(
        "needle_bud_storage_vigor_strength", "NBV", 0.0f, 1.0f,
        [value_ptr]() {
          return *value_ptr;
        },
        [value_ptr](float value) {
          *value_ptr = std::clamp(value, 0.0f, 1.0f);
        });
  }
  {
    auto* value_ptr = &d.needle_bud_storage_completion_floor;
    explorer.AddAxis(
        "needle_bud_storage_completion_floor", "NBC", 0.0f, 1.0f,
        [value_ptr]() {
          return *value_ptr;
        },
        [value_ptr](float value) {
          *value_ptr = std::clamp(value, 0.0f, 1.0f);
        });
  }

  // -- Needle curvature --
  explorer.AddSingle("needle_curvature_adaxial_bias", "NCA", d.needle_curvature_adaxial_bias, -0.1f, 0.1f, 0.003f);
  explorer.AddSingle("needle_curvature_abaxial_bias", "NCB", d.needle_curvature_abaxial_bias, -0.1f, 0.1f, 0.010f);
  explorer.AddSingle("needle_curvature_gradient_per_arclen", "NCG", d.needle_curvature_gradient_per_arclen, -0.05f,
                     0.05f, 0.0015f);
  explorer.AddSingle("needle_diameter_for_curvature_m", "NDC", d.needle_diameter_for_curvature_m, 0.0f, 0.005f, 0.001f);
  explorer.AddSingle("needle_sinusoidal_amplitude_deg", "NSA", d.needle_sinusoidal_amplitude_deg, 0.0f, 45.0f, 0.0f);
  explorer.AddSingle("needle_sinusoidal_frequency_cycles", "NSF", d.needle_sinusoidal_frequency_cycles, 0.0f, 12.0f,
                     0.0f);
  explorer.AddSingle("needle_sinusoidal_phase_randomness_deg", "NSP", d.needle_sinusoidal_phase_randomness_deg, 0.0f,
                     180.0f, 0.0f);

  // -- Needle mechanics --
  explorer.AddSingle("needle_young_modulus_baseline_Pa", "YMB", d.needle_young_modulus_baseline_Pa, 0.0f, 5e9f, 1e9f);
  explorer.AddSingle("needle_lignification_maturation_years", "LMY", d.needle_lignification_maturation_years, 0.0f,
                     5.0f, 1.0f);
  explorer.AddSingle("needle_density_kg_m3", "NDK", d.needle_density_kg_m3, 0.0f, 2000.0f, 800.0f);
  explorer.AddSingle("gravity_m_s2", "GRV", d.gravity_m_s2, 0.0f, 25.0f, 9.81f);
  explorer.AddSingle("needle_per_needle_length_cv", "NLC", d.needle_per_needle_length_cv, 0.0f, 1.0f, 0.0f);
  explorer.AddSingle("needle_per_needle_curvature_cv", "NCCV", d.needle_per_needle_curvature_cv, 0.0f, 1.0f, 0.0f);
  explorer.AddSingle("needle_per_needle_radius_cv", "NRCV", d.needle_per_needle_radius_cv, 0.0f, 1.0f, 0.0f);
  explorer.AddSingle("needle_per_needle_modulus_cv", "NMCV", d.needle_per_needle_modulus_cv, 0.0f, 1.0f, 0.0f);
  explorer.AddSingle("needle_per_needle_density_cv", "NDCV", d.needle_per_needle_density_cv, 0.0f, 1.0f, 0.0f);
  explorer.AddSingle("needle_per_needle_wave_amplitude_cv", "NWAC", d.needle_per_needle_wave_amplitude_cv, 0.0f, 1.0f,
                     0.0f);
  explorer.AddSingle("needle_per_needle_wave_frequency_cv", "NWFC", d.needle_per_needle_wave_frequency_cv, 0.0f, 1.0f,
                     0.0f);
  explorer.AddSingle("needle_per_needle_wave_phase_cv", "NWPC", d.needle_per_needle_wave_phase_cv, 0.0f, 1.0f, 0.0f);

  // -- Tropism --
  explorer.AddSingle("gravitropism_first_order", "GFO", d.gravitropism_first_order, 0.0f, 0.001f, 0.0001f);
  explorer.AddSingle("initial_orientation_yaw_deg", "IOY", d.initial_orientation_yaw_deg, -180.0f, 180.0f, 0.0f);

  // -- Per-instance growth target --
  explorer.AddSingle("target_gdd", "TGD", d.target_gdd, 0.0f, 30000.0f, 6000.0f);
  explorer.AddSingle("gdd_per_day", "GPD", d.gdd_per_day, 0.0f, 50.0f, 2.0f);

  // -- Per-shoot stochastic noise --
  explorer.AddSingle("internode_length_per_node_cv", "ILC", d.internode_length_per_node_cv, 0.0f, 1.0f, 0.1f);
  explorer.AddSingle("internode_thickness_per_node_cv", "STC", d.internode_thickness_per_node_cv, 0.0f, 1.0f, 0.1f);
  explorer.AddSingle("branch_angle_per_node_sigma_deg", "BAS", d.branch_angle_per_node_sigma_deg, 0.0f, 30.0f, 5.0f);
  explorer.AddSingle("roll_phyllotaxis_per_node_sigma_deg", "RPS", d.roll_phyllotaxis_per_node_sigma_deg, 0.0f, 30.0f,
                     5.0f);

}
