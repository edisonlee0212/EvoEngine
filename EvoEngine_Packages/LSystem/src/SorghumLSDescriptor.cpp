#include "SorghumLSDescriptor.hpp"
#include <yaml-cpp/yaml.h>
#include <Application.hpp>
#include <EditorLayer.hpp>
#include <Scene.hpp>
#include <Texture2D.hpp>
#include <Transform.hpp>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include "DistributionDefaults.hpp"
#include "LSystemDescriptorDefaults.hpp"
#include "LSystemInspectionAdapters.hpp"
#include "LSystemLayer.hpp"
#include "LSystemSerializationAdapters.hpp"
#include "SorghumLS.hpp"

using namespace l_system_package;
using namespace evo_engine;

// ---------------------------------------------------------------------------
// SorghumLSDescriptor
//
// Foundation pass: minimal but complete asset wiring.
//   - Constructor sets inline plot defaults so the asset is usable without a
//     defaults file. If a defaults file is present at the standard search
//     locations it is loaded as in MaizeTasselDescriptor.
//   - Sample() copies all PlottedDistribution fields by value into
//     SampledSorghumParams and pulls SingleDistribution scalars per-instance.
//   - Instantiate() is a Phase B stub: creates an empty entity. Phase E.2
//     replaces the body to attach the SorghumLS private component.
//   - OnInspect() shows an Instantiate button only. Full inspector UI is a
//     follow-up; the field set is locked so future UI work is additive.
// ---------------------------------------------------------------------------

namespace {

constexpr char kSorghumDescriptorName[] = "SorghumLSDescriptor";
constexpr float kLeafCurlingLegacyNormalizedThreshold = 1.01f;
constexpr float kLeafCurlingLegacyScaleDeg = 90.0f;

const std::array<std::filesystem::path, 6> kSorghumResourceCandidates = {
    std::filesystem::path("./LSystemResources/Defaults/SorghumLSDescriptor_Default.sorghumls"),
    std::filesystem::path("./EvoEngine_Plugins/LSystem/Internals/LSystemResources/Defaults/") /
        "SorghumLSDescriptor_Default.sorghumls",
    std::filesystem::path("./Resources/DigitalAgricultureProject/Assets/New SorghumLSDescriptor.sorghumls"),
    std::filesystem::path("./DigitalAgricultureProject/Assets/New SorghumLSDescriptor.sorghumls"),
    std::filesystem::path("./04_EvoEngine/Resources/DigitalAgricultureProject/Assets/") /
        "New SorghumLSDescriptor.sorghumls",
    std::filesystem::path("./04_EvoEngine/EvoEngine_Plugins/LSystem/Internals/") /
        "LSystemResources/Defaults/SorghumLSDescriptor_Default.sorghumls"};

const std::array<std::filesystem::path, 2> kSorghumProjectAssetCandidates = {
    std::filesystem::path("LSystem") / "New SorghumLSDescriptor.sorghumls", "New SorghumLSDescriptor.sorghumls"};

const std::array<std::filesystem::path, 2> kSorghumWritableTemplateCandidates = {
    std::filesystem::path("./Resources/DigitalAgricultureProject/Assets/") / "New SorghumLSDescriptor.sorghumls",
    std::filesystem::path("./04_EvoEngine/Resources/DigitalAgricultureProject/Assets/") /
        "New SorghumLSDescriptor.sorghumls"};

const std::filesystem::path kSorghumFallbackDefaultsPath =
    std::filesystem::path("./LSystemResources/Defaults/SorghumLSDescriptor_Default.sorghumls");

double GetSteadyTimeSeconds() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

std::filesystem::path ResolveDefaultSorghumLSDescriptorPath() {
  return descriptor_defaults::ResolveExistingDefaultsPath(kSorghumResourceCandidates, kSorghumProjectAssetCandidates);
}

std::filesystem::path ResolveWritableSorghumLSDescriptorDefaultsPath() {
  return descriptor_defaults::ResolveWritableDefaultsPath(kSorghumResourceCandidates, kSorghumProjectAssetCandidates,
                                                          kSorghumWritableTemplateCandidates,
                                                          kSorghumFallbackDefaultsPath);
}

bool LoadSorghumLSDescriptorDefaultsFromFile(SorghumLSDescriptor& descriptor, const std::filesystem::path& file_path) {
  YAML::Node defaults;
  if (!descriptor_defaults::LoadDefaultsYamlMap(file_path, defaults, kSorghumDescriptorName)) {
    return false;
  }
  DeserializeSorghumLSDescriptor(defaults, descriptor);
  return true;
}

void LoadSingleDistributionWithScalarFallback(const YAML::Node& in, const char* key,
                                              evo_engine::SingleDistribution<float>& distribution) {
  if (!in[key]) {
    return;
  }
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

bool IsPlottedDistributionNode(const YAML::Node& node) {
  if (!node || !node.IsMap()) {
    return false;
  }
  return node["mean"] || node["deviation"] || node["m_mean"] || node["m_deviation"];
}

void LoadGrowthCurveWithLegacyFallback(const YAML::Node& in, const char* key,
                                       evo_engine::PlottedDistribution<float>& distribution) {
  if (!in[key]) {
    return;
  }

  const auto& node = in[key];
  if (IsPlottedDistributionNode(node)) {
    distribution.Load(key, in);
    return;
  }

  if (!node.IsMap()) {
    return;
  }

  evo_engine::Curve2D legacy_curve;
  legacy_curve.Load(key, in);

  distribution.mean.min_value = 0.0f;
  distribution.mean.max_value = 1.0f;
  distribution.mean.curve = legacy_curve;
  DistributionDefaults::ApplyStdPlotDefaults(distribution.deviation);
}

float SampleTargetGddForSeed(const evo_engine::SingleDistribution<float>& distribution, const uint32_t seed) {
  std::mt19937 rng(seed);
  return std::max(0.0f, SampleDistribution(distribution, rng));
}

bool IsLikelyLegacyNormalizedLeafCurling(const evo_engine::PlottedDistribution<float>& distribution) {
  const float mean_min = distribution.mean.min_value;
  const float mean_max = distribution.mean.max_value;
  const float dev_min = distribution.deviation.min_value;
  const float dev_max = distribution.deviation.max_value;

  if (!std::isfinite(mean_min) || !std::isfinite(mean_max) || !std::isfinite(dev_min) || !std::isfinite(dev_max)) {
    return false;
  }

  return mean_min >= -1.0e-3f && mean_max <= kLeafCurlingLegacyNormalizedThreshold && dev_min >= -1.0e-3f &&
         dev_max <= kLeafCurlingLegacyNormalizedThreshold;
}

void ScalePlottedDistributionRange(evo_engine::PlottedDistribution<float>& distribution, const float scale) {
  distribution.mean.min_value *= scale;
  distribution.mean.max_value *= scale;
  distribution.deviation.min_value *= scale;
  distribution.deviation.max_value *= scale;
}

void SetConstantPlottedDistribution(evo_engine::PlottedDistribution<float>& distribution, const float value) {
  distribution.mean.min_value = value;
  distribution.mean.max_value = value;
  distribution.deviation.min_value = 0.0f;
  distribution.deviation.max_value = 0.0f;
}

void SetTillerLeafAreaRatioDefaults(evo_engine::PlottedDistribution<float>& distribution) {
  constexpr std::array<float, 6> kRatios{0.77f, 0.87f, 0.87f, 0.87f, 0.61f, 0.61f};
  distribution.mean.min_value = 0.5f;
  distribution.mean.max_value = 1.0f;
  distribution.mean.curve.SetTangent(false);
  auto& values = distribution.mean.curve.UnsafeGetValues();
  values.clear();
  for (size_t i = 0; i < kRatios.size(); ++i) {
    values.emplace_back(static_cast<float>(i) / static_cast<float>(kRatios.size() - 1), (kRatios[i] - 0.5f) / 0.5f);
  }
  DistributionDefaults::ApplyStdPlotDefaults(distribution.deviation);
}

template <size_t N>
void SaveIntArray(YAML::Emitter& out, const char* key, const std::array<int, N>& values) {
  out << YAML::Key << key << YAML::Value << YAML::Flow << YAML::BeginSeq;
  for (const int value : values) {
    out << value;
  }
  out << YAML::EndSeq;
}

template <size_t N>
void LoadIntArray(const YAML::Node& in, const char* key, std::array<int, N>& values) {
  const auto node = in[key];
  if (!node || !node.IsSequence()) {
    return;
  }
  const size_t count = std::min(N, node.size());
  for (size_t i = 0; i < count; ++i) {
    values[i] = node[i].as<int>();
  }
}

// Helper: convert a TropismEntry array into the runtime SampledTropism vector,
// using rng for direction/strength sampling and the entry's usage_chance_percent
// gate. Mirrors MaizeTasselDescriptor::Sample tropism handling.
void SampleTropisms(const std::vector<TropismEntry>& entries, std::mt19937& rng, std::vector<SampledTropism>& out) {
  for (const auto& entry : entries) {
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
    out.push_back(std::move(st));
  }
}

void ClampSingleDistribution(evo_engine::SingleDistribution<float>& distribution, const float mean_min,
                             const float mean_max, const float deviation_max = std::numeric_limits<float>::max()) {
  distribution.mean = std::clamp(distribution.mean, mean_min, mean_max);
  distribution.deviation = std::clamp(distribution.deviation, 0.0f, deviation_max);
}

void ClampPlottedDistributionRange(evo_engine::PlottedDistribution<float>& distribution, const float mean_min,
                                   const float mean_max, const float deviation_max) {
  distribution.mean.min_value = std::clamp(distribution.mean.min_value, mean_min, mean_max);
  distribution.mean.max_value = std::clamp(distribution.mean.max_value, mean_min, mean_max);
  if (distribution.mean.max_value < distribution.mean.min_value) {
    std::swap(distribution.mean.min_value, distribution.mean.max_value);
  }

  distribution.deviation.min_value = std::clamp(distribution.deviation.min_value, 0.0f, deviation_max);
  distribution.deviation.max_value = std::clamp(distribution.deviation.max_value, 0.0f, deviation_max);
  if (distribution.deviation.max_value < distribution.deviation.min_value) {
    std::swap(distribution.deviation.min_value, distribution.deviation.max_value);
  }
}

void ClampDescriptorValues(SorghumLSDescriptor& descriptor) {
  ClampSingleDistribution(descriptor.total_phytomer_count, 1.0f, 128.0f, 64.0f);
  ClampSingleDistribution(descriptor.phyllotaxis_angle, 0.0f, 360.0f, 180.0f);
  ClampSingleDistribution(descriptor.branch_azimuth_offset, -180.0f, 180.0f, 180.0f);
  ClampSingleDistribution(descriptor.main_culm_lean_angle, 0.0f, 30.0f, 20.0f);

  ClampPlottedDistributionRange(descriptor.internode_length, 0.0f, 1.0f, 0.5f);
  ClampPlottedDistributionRange(descriptor.internode_thickness, 0.0f, 0.1f, 0.05f);

  ClampPlottedDistributionRange(descriptor.leaf_blade_length, 0.0f, 1.5f, 0.75f);
  ClampPlottedDistributionRange(descriptor.leaf_blade_max_width, 0.0f, 0.2f, 0.1f);
  ClampPlottedDistributionRange(descriptor.leaf_blade_thickness, 0.00001f, 0.005f, 0.0025f);
  ClampPlottedDistributionRange(descriptor.leaf_sheath_thickness, 0.00001f, 0.005f, 0.0025f);
  descriptor.leaf_width_scale = std::clamp(descriptor.leaf_width_scale, 0.05f, 3.0f);
  ClampPlottedDistributionRange(descriptor.leaf_sheath_length, 0.0f, 0.6f, 0.3f);
  ClampPlottedDistributionRange(descriptor.leaf_neck_length, 0.0f, 0.6f, 0.3f);
  ClampPlottedDistributionRange(descriptor.leaf_sheath_end_width_ratio, 0.05f, 8.0f, 4.0f);
  ClampPlottedDistributionRange(descriptor.leaf_neck_end_width_ratio, 0.05f, 8.0f, 4.0f);
  ClampPlottedDistributionRange(descriptor.leaf_blade_end_width_ratio, 0.05f, 8.0f, 4.0f);
  ClampPlottedDistributionRange(descriptor.leaf_insertion_angle, -90.0f, 120.0f, 90.0f);
  ClampPlottedDistributionRange(descriptor.leaf_roll_angle, -180.0f, 180.0f, 180.0f);
  ClampPlottedDistributionRange(descriptor.leaf_curling, 0.0f, 90.0f, 45.0f);
  ClampPlottedDistributionRange(descriptor.leaf_bending, -180.0f, 180.0f, 180.0f);
  ClampPlottedDistributionRange(descriptor.leaf_waviness, 0.0f, 0.3f, 0.2f);
  ClampPlottedDistributionRange(descriptor.leaf_waviness_width_fraction, 0.0f, 1.0f, 0.5f);
  ClampSingleDistribution(descriptor.leaf_waviness_frequency, 0.0f, 100.0f, 50.0f);
  ClampSingleDistribution(descriptor.leaf_waviness_wavelength_m, 0.0f, 5.0f, 2.5f);
  ClampSingleDistribution(descriptor.leaf_centerline_waviness_fraction, 0.0f, 0.1f, 0.05f);
  ClampSingleDistribution(descriptor.leaf_static_wind_deflection_fraction, 0.0f, 0.2f, 0.1f);
  ClampSingleDistribution(descriptor.leaf_axial_twist_max_degrees, 0.0f, 45.0f, 30.0f);
  ClampSingleDistribution(descriptor.leaf_axial_twist_frequency_ratio_min, 0.0f, 0.5f, 0.5f);
  ClampSingleDistribution(descriptor.leaf_axial_twist_frequency_ratio_max, 0.0f, 0.5f, 0.5f);
  descriptor.leaf_axial_twist_frequency_ratio_max.mean = std::max(descriptor.leaf_axial_twist_frequency_ratio_min.mean,
                                                                  descriptor.leaf_axial_twist_frequency_ratio_max.mean);
  ClampSingleDistribution(descriptor.leaf_gravity_droop_compliance, 0.0f, 5.0f, 2.5f);
  ClampPlottedDistributionRange(descriptor.leaf_gravity_droop_age_response, 0.0f, 1.0f, 1.0f);
  ClampPlottedDistributionRange(descriptor.leaf_flexural_stiffness_along_leaf, 0.01f, 2.0f, 1.0f);
  ClampSingleDistribution(descriptor.leaf_damage_severity, 0.0f, 1.0f, 0.5f);
  ClampSingleDistribution(descriptor.leaf_sheath_radius_ratio, 1.0f, 3.0f, 1.0f);
  ClampSingleDistribution(descriptor.leaf_sheath_cross_section_ratio, 1.0f, 3.0f, 1.0f);
  ClampSingleDistribution(descriptor.leaf_sheath_wrap_angle, 180.0f, 540.0f, 180.0f);
  ClampSingleDistribution(descriptor.leaf_blade_stage1_length_ratio, 0.0f, 1.0f, 1.0f);
  ClampSingleDistribution(descriptor.leaf_blade_stage2_length_ratio, 0.0f, 1.0f, 1.0f);
  ClampSingleDistribution(descriptor.leaf_blade_stage3_length_ratio, 0.0f, 1.0f, 1.0f);
  ClampSingleDistribution(descriptor.leaf_blade_stage1_width_scale, 0.05f, 2.0f, 1.0f);
  ClampSingleDistribution(descriptor.leaf_blade_stage2_width_scale, 0.05f, 2.0f, 1.0f);
  ClampSingleDistribution(descriptor.leaf_blade_stage3_width_scale, 0.05f, 2.0f, 1.0f);

  ClampSingleDistribution(descriptor.leaf_lifespan_years, 0.1f, 8.0f, 4.0f);
  ClampSingleDistribution(descriptor.leaf_wilting_years, 0.05f, 8.0f, 4.0f);
  descriptor.leaf_wilting_years.mean =
      std::clamp(descriptor.leaf_wilting_years.mean, 0.05f, std::max(0.05f, descriptor.leaf_lifespan_years.mean));
  ClampSingleDistribution(descriptor.flag_leaf_length_scale, 0.1f, 1.5f, 0.7f);
  ClampSingleDistribution(descriptor.flag_leaf_width_scale, 0.1f, 1.5f, 0.7f);
  ClampSingleDistribution(descriptor.flag_leaf_insertion_angle_offset, -90.0f, 90.0f, 45.0f);
  ClampSingleDistribution(descriptor.flag_leaf_bending_scale, 0.0f, 1.5f, 0.7f);

  ClampSingleDistribution(descriptor.panicle_initiation_gdd, 0.0f, 2000.0f, 1000.0f);
  ClampSingleDistribution(descriptor.panicle_maturity_gdd, 1.0f, 3000.0f, 1500.0f);
  ClampSingleDistribution(descriptor.panicle_peduncle_length_m, 0.0f, 1.0f, 0.5f);
  ClampSingleDistribution(descriptor.panicle_rachis_length_m, 0.02f, 1.0f, 0.5f);
  ClampSingleDistribution(descriptor.panicle_rachis_radius_m, 0.0005f, 0.04f, 0.02f);
  ClampSingleDistribution(descriptor.panicle_primary_branch_count, 1.0f, 64.0f, 32.0f);
  ClampSingleDistribution(descriptor.panicle_spikelet_pairs_per_branch, 1.0f, 32.0f, 16.0f);
  ClampSingleDistribution(descriptor.panicle_branch_length_m, 0.005f, 0.5f, 0.25f);
  ClampSingleDistribution(descriptor.panicle_branch_length_taper, 0.05f, 1.0f, 0.5f);
  ClampSingleDistribution(descriptor.panicle_branch_radius_m, 0.0001f, 0.02f, 0.01f);
  ClampSingleDistribution(descriptor.panicle_branch_angle_degrees, 0.0f, 85.0f, 45.0f);
  ClampSingleDistribution(descriptor.panicle_spikelet_length_m, 0.001f, 0.05f, 0.025f);
  ClampSingleDistribution(descriptor.panicle_spikelet_radius_m, 0.0002f, 0.02f, 0.01f);
  ClampSingleDistribution(descriptor.panicle_pedicel_length_m, 0.0f, 0.05f, 0.025f);

  ClampSingleDistribution(descriptor.tiller_count, 0.0f, 20.0f, 10.0f);
  descriptor.tiller_count_min = std::clamp(descriptor.tiller_count_min, 0, 6);
  descriptor.tiller_count_max = std::clamp(descriptor.tiller_count_max, descriptor.tiller_count_min, 6);
  for (auto& rank : descriptor.tiller_origin_rank_order) {
    rank = std::clamp(rank, 1, 6);
  }
  for (auto& stage : descriptor.tiller_emergence_main_leaf_stages) {
    stage = std::clamp(stage, 1, 128);
  }
  ClampPlottedDistributionRange(descriptor.tiller_initiation_delay_gdd, 0.0f, 2000.0f, 1000.0f);
  ClampSingleDistribution(descriptor.tiller_insertion_angle, 0.0f, 120.0f, 60.0f);
  ClampSingleDistribution(descriptor.tiller_final_lean_angle, -30.0f, 30.0f, 30.0f);
  ClampSingleDistribution(descriptor.tiller_azimuth_jitter, -45.0f, 45.0f, 45.0f);
  ClampSingleDistribution(descriptor.tiller_same_side_splay_angle, 0.0f, 45.0f, 20.0f);
  descriptor.tiller_recovery_axis_fraction = std::clamp(descriptor.tiller_recovery_axis_fraction, 0.05f, 1.0f);
  ClampSingleDistribution(descriptor.tiller_leaf_count_ratio, 0.5f, 1.1f, 0.3f);
  ClampSingleDistribution(descriptor.tiller_height_ratio, 0.5f, 1.1f, 0.3f);
  ClampPlottedDistributionRange(descriptor.tiller_leaf_area_ratio_by_origin, 0.1f, 1.2f, 0.5f);
  ClampSingleDistribution(descriptor.tiller_phytomer_count_scale, 0.05f, 2.0f, 1.0f);
  ClampSingleDistribution(descriptor.tiller_thickness_ratio, 0.05f, 1.2f, 0.5f);
  ClampSingleDistribution(descriptor.tiller_max_axis_length_ratio, 0.1f, 1.5f, 0.7f);

  ClampSingleDistribution(descriptor.target_gdd, 0.0f, 5000.0f, 5000.0f);
  ClampSingleDistribution(descriptor.gdd_per_day, 0.0f, 500.0f, 500.0f);
  ClampSingleDistribution(descriptor.plastochron_gdd, 1.0f, 400.0f, 400.0f);
  ClampSingleDistribution(descriptor.maturity_gdd, 1.0f, 3000.0f, 3000.0f);
  descriptor.maturity_gdd.mean = std::max(descriptor.maturity_gdd.mean, descriptor.plastochron_gdd.mean);
  ClampSingleDistribution(descriptor.main_axis_plastochron_scale, 0.1f, 5.0f, 5.0f);
  ClampSingleDistribution(descriptor.lateral_axis_plastochron_scale, 0.1f, 5.0f, 5.0f);
  ClampSingleDistribution(descriptor.lateral_bud_plastochron_scale, 0.1f, 5.0f, 5.0f);
  ClampSingleDistribution(descriptor.maturity_initiation_coupling, 0.0f, 2.0f, 2.0f);
  ClampSingleDistribution(descriptor.reference_maturity_gdd, 1.0f, 5000.0f, 5000.0f);

  for (auto& tropism : descriptor.tropisms) {
    ClampSingleDistribution(tropism.direction_x, -1.0f, 1.0f, 1.0f);
    ClampSingleDistribution(tropism.direction_y, -1.0f, 1.0f, 1.0f);
    ClampSingleDistribution(tropism.direction_z, -1.0f, 1.0f, 1.0f);
    ClampSingleDistribution(tropism.strength, -5.0f, 5.0f, 5.0f);
    ClampPlottedDistributionRange(tropism.order_response, -2.0f, 2.0f, 2.0f);
    tropism.usage_chance_percent = std::clamp(tropism.usage_chance_percent, 0.0f, 100.0f);
  }

  descriptor.grid_rows = std::clamp(descriptor.grid_rows, 1, 50);
  descriptor.grid_cols = std::clamp(descriptor.grid_cols, 1, 50);
  descriptor.grid_spacing = std::clamp(descriptor.grid_spacing, 0.1f, 50.0f);
  descriptor.leaf_material_albedo_color =
      glm::clamp(descriptor.leaf_material_albedo_color, glm::vec3(0.0f), glm::vec3(1.0f));
  descriptor.leaf_material_roughness = std::clamp(descriptor.leaf_material_roughness, 0.0f, 1.0f);
  descriptor.leaf_material_metallic = std::clamp(descriptor.leaf_material_metallic, 0.0f, 1.0f);
  descriptor.leaf_material_specular = std::clamp(descriptor.leaf_material_specular, 0.0f, 1.0f);
  descriptor.leaf_material_subsurface_factor = std::clamp(descriptor.leaf_material_subsurface_factor, 0.0f, 1.0f);
  descriptor.leaf_material_subsurface_color =
      glm::clamp(descriptor.leaf_material_subsurface_color, glm::vec3(0.0f), glm::vec3(1.0f));
  descriptor.leaf_material_subsurface_radius = glm::max(descriptor.leaf_material_subsurface_radius, glm::vec3(0.0f));
  descriptor.stem_material_albedo_color =
      glm::clamp(descriptor.stem_material_albedo_color, glm::vec3(0.0f), glm::vec3(1.0f));
  descriptor.stem_material_roughness = std::clamp(descriptor.stem_material_roughness, 0.0f, 1.0f);
  descriptor.stem_material_metallic = std::clamp(descriptor.stem_material_metallic, 0.0f, 1.0f);
  descriptor.stem_material_specular = std::clamp(descriptor.stem_material_specular, 0.0f, 1.0f);
  descriptor.panicle_immature_color = glm::clamp(descriptor.panicle_immature_color, glm::vec3(0.0f), glm::vec3(1.0f));
  descriptor.panicle_mature_color = glm::clamp(descriptor.panicle_mature_color, glm::vec3(0.0f), glm::vec3(1.0f));
  descriptor.panicle_material_roughness = std::clamp(descriptor.panicle_material_roughness, 0.0f, 1.0f);
  descriptor.culm_radial_segments = std::clamp(descriptor.culm_radial_segments, 8u, 64u);
  descriptor.culm_node_radius_scale = std::clamp(descriptor.culm_node_radius_scale, 1.0f, 1.5f);
  descriptor.culm_texture_repeat_m = std::clamp(descriptor.culm_texture_repeat_m, 0.01f, 5.0f);
}

}  // namespace

SorghumLSDescriptor::SorghumLSDescriptor() {
  ApplyMeanStdPlotDefaultsToAll(
      internode_length, internode_thickness, leaf_blade_length, leaf_blade_max_width, leaf_blade_thickness,
      leaf_sheath_thickness, leaf_sheath_length, leaf_neck_length, leaf_sheath_end_width_ratio,
      leaf_neck_end_width_ratio, leaf_blade_end_width_ratio, leaf_insertion_angle, leaf_roll_angle, leaf_curling,
      leaf_bending, leaf_waviness, leaf_waviness_width_fraction, leaf_gravity_droop_age_response,
      leaf_flexural_stiffness_along_leaf, tiller_leaf_area_ratio_by_origin, tiller_initiation_delay_gdd);

  SetConstantPlottedDistribution(leaf_blade_thickness, 0.00045f);
  SetConstantPlottedDistribution(leaf_sheath_thickness, 0.00045f);
  SetConstantPlottedDistribution(leaf_waviness_width_fraction, 0.0f);
  SetConstantPlottedDistribution(leaf_gravity_droop_age_response, 1.0f);
  SetConstantPlottedDistribution(leaf_flexural_stiffness_along_leaf, 1.0f);

  SetConstantPlottedDistribution(leaf_sheath_end_width_ratio, 1.35f);
  SetConstantPlottedDistribution(leaf_neck_end_width_ratio, 1.7f);
  SetConstantPlottedDistribution(leaf_blade_end_width_ratio, 0.35f);
  SetTillerLeafAreaRatioDefaults(tiller_leaf_area_ratio_by_origin);

  ApplyLinearGrowthCurveDefaultsToAll(
      internode_elongation_curve, internode_thickness_curve, leaf_sheath_length_growth_curve,
      leaf_neck_length_growth_curve, leaf_blade_growth_curve, leaf_sheath_width_growth_curve,
      leaf_neck_width_growth_curve, leaf_width_growth_curve, leaf_angle_development_curve,
      leaf_curling_development_curve, leaf_bending_development_curve, width_along_sheath, width_along_neck,
      width_along_leaf, bending_along_leaf, curling_along_leaf, waviness_along_leaf);

  const auto defaults_path = ResolveDefaultSorghumLSDescriptorPath();
  if (!LoadSorghumLSDescriptorDefaultsFromFile(*this, defaults_path)) {
    static bool warned_once = false;
    if (!warned_once) {
      warned_once = true;
      EVOENGINE_WARNING("SorghumLSDescriptor defaults file not found or invalid. Using inline member defaults.");
    }
  }
  ClampDescriptorValues(*this);
}

std::filesystem::path SorghumLSDescriptor::ResolveWritableDefaultsPath() const {
  return ResolveWritableSorghumLSDescriptorDefaultsPath();
}

// ---------------------------------------------------------------------------
// Sampling
// ---------------------------------------------------------------------------

SampledSorghumParams SorghumLSDescriptor::Sample(std::mt19937& rng) const {
  SampledSorghumParams p;

  // Culm topology.
  p.total_phytomer_count = std::max(1, static_cast<int>(std::round(SampleDistribution(total_phytomer_count, rng))));
  p.phyllotaxis_angle = SampleDistribution(phyllotaxis_angle, rng);
  p.branch_azimuth_offset = SampleDistribution(branch_azimuth_offset, rng);
  p.main_culm_lean_angle = std::clamp(SampleDistribution(main_culm_lean_angle, rng), 0.0f, 30.0f);

  // Internode + leaf morphology distributions are forwarded by value; rule
  // lambdas re-evaluate them per emission to keep deterministic sampling
  // (same idiom as MaizeTasselDescriptor::Sample).
  p.internode_length = internode_length;
  p.internode_thickness = internode_thickness;

  p.leaf_blade_length = leaf_blade_length;
  p.leaf_blade_max_width = leaf_blade_max_width;
  p.leaf_blade_thickness = leaf_blade_thickness;
  p.leaf_sheath_thickness = leaf_sheath_thickness;
  p.leaf_width_scale = std::clamp(leaf_width_scale, 0.05f, 3.0f);
  p.leaf_sheath_length = leaf_sheath_length;
  p.leaf_neck_length = leaf_neck_length;
  p.leaf_sheath_end_width_ratio = leaf_sheath_end_width_ratio;
  p.leaf_neck_end_width_ratio = leaf_neck_end_width_ratio;
  p.leaf_blade_end_width_ratio = leaf_blade_end_width_ratio;
  p.leaf_insertion_angle = leaf_insertion_angle;
  p.leaf_roll_angle = leaf_roll_angle;
  p.leaf_curling = leaf_curling;
  p.leaf_bending = leaf_bending;
  p.leaf_waviness = leaf_waviness;
  p.leaf_waviness_frequency = std::max(0.0f, SampleDistribution(leaf_waviness_frequency, rng));
  p.leaf_waviness_width_fraction = leaf_waviness_width_fraction;
  p.leaf_waviness_wavelength_m = std::max(0.0f, SampleDistribution(leaf_waviness_wavelength_m, rng));
  p.leaf_centerline_waviness_fraction = std::max(0.0f, SampleDistribution(leaf_centerline_waviness_fraction, rng));
  p.leaf_static_wind_deflection_fraction =
      std::max(0.0f, SampleDistribution(leaf_static_wind_deflection_fraction, rng));
  p.leaf_axial_twist_max_degrees = std::max(0.0f, SampleDistribution(leaf_axial_twist_max_degrees, rng));
  p.leaf_axial_twist_frequency_ratio_min =
      std::clamp(SampleDistribution(leaf_axial_twist_frequency_ratio_min, rng), 0.0f, 0.5f);
  p.leaf_axial_twist_frequency_ratio_max = std::clamp(SampleDistribution(leaf_axial_twist_frequency_ratio_max, rng),
                                                      p.leaf_axial_twist_frequency_ratio_min, 0.5f);
  p.leaf_static_wind_azimuth_degrees = std::uniform_real_distribution<float>(0.0f, 360.0f)(rng);
  p.leaf_gravity_droop_compliance = std::max(0.0f, SampleDistribution(leaf_gravity_droop_compliance, rng));
  p.leaf_gravity_droop_age_response = leaf_gravity_droop_age_response;
  p.leaf_flexural_stiffness_along_leaf = leaf_flexural_stiffness_along_leaf;
  p.leaf_damage_severity = leaf_damage_severity;
  p.leaf_sheath_radius_ratio = std::clamp(SampleDistribution(leaf_sheath_radius_ratio, rng), 1.0f, 3.0f);
  p.leaf_sheath_cross_section_ratio = std::clamp(SampleDistribution(leaf_sheath_cross_section_ratio, rng), 1.0f, 3.0f);
  p.leaf_sheath_wrap_angle = std::clamp(SampleDistribution(leaf_sheath_wrap_angle, rng), 180.0f, 540.0f);
  p.leaf_blade_stage1_length_ratio = std::clamp(SampleDistribution(leaf_blade_stage1_length_ratio, rng), 0.0f, 1.0f);
  p.leaf_blade_stage2_length_ratio = std::clamp(SampleDistribution(leaf_blade_stage2_length_ratio, rng), 0.0f, 1.0f);
  p.leaf_blade_stage3_length_ratio = std::clamp(SampleDistribution(leaf_blade_stage3_length_ratio, rng), 0.0f, 1.0f);
  p.leaf_blade_stage1_width_scale = std::clamp(SampleDistribution(leaf_blade_stage1_width_scale, rng), 0.05f, 2.0f);
  p.leaf_blade_stage2_width_scale = std::clamp(SampleDistribution(leaf_blade_stage2_width_scale, rng), 0.05f, 2.0f);
  p.leaf_blade_stage3_width_scale = std::clamp(SampleDistribution(leaf_blade_stage3_width_scale, rng), 0.05f, 2.0f);

  // Leaf lifecycle (chronological).
  p.leaf_lifespan_years = leaf_lifespan_years;
  p.leaf_lifespan_years.mean = std::max(0.1f, p.leaf_lifespan_years.mean);
  p.leaf_lifespan_years.deviation = std::max(0.0f, p.leaf_lifespan_years.deviation);
  p.leaf_wilting_years = leaf_wilting_years;
  p.leaf_wilting_years.mean = std::max(0.05f, p.leaf_wilting_years.mean);
  p.leaf_wilting_years.deviation = std::max(0.0f, p.leaf_wilting_years.deviation);
  p.flag_leaf_length_scale = std::clamp(SampleDistribution(flag_leaf_length_scale, rng), 0.1f, 1.5f);
  p.flag_leaf_width_scale = std::clamp(SampleDistribution(flag_leaf_width_scale, rng), 0.1f, 1.5f);
  p.flag_leaf_insertion_angle_offset = SampleDistribution(flag_leaf_insertion_angle_offset, rng);
  p.flag_leaf_bending_scale = std::clamp(SampleDistribution(flag_leaf_bending_scale, rng), 0.0f, 1.5f);

  p.enable_panicle = enable_panicle;
  p.panicle_initiation_gdd = std::max(0.0f, SampleDistribution(panicle_initiation_gdd, rng));
  p.panicle_maturity_gdd = std::max(1.0f, SampleDistribution(panicle_maturity_gdd, rng));
  p.panicle_peduncle_length_m = std::max(0.0f, SampleDistribution(panicle_peduncle_length_m, rng));
  p.panicle_rachis_length_m = std::max(0.02f, SampleDistribution(panicle_rachis_length_m, rng));
  p.panicle_rachis_radius_m = std::max(0.0005f, SampleDistribution(panicle_rachis_radius_m, rng));
  p.panicle_primary_branch_count =
      std::clamp(static_cast<int>(std::round(SampleDistribution(panicle_primary_branch_count, rng))), 1, 64);
  p.panicle_spikelet_pairs_per_branch =
      std::clamp(static_cast<int>(std::round(SampleDistribution(panicle_spikelet_pairs_per_branch, rng))), 1, 32);
  p.panicle_branch_length_m = std::max(0.005f, SampleDistribution(panicle_branch_length_m, rng));
  p.panicle_branch_length_taper = std::clamp(SampleDistribution(panicle_branch_length_taper, rng), 0.05f, 1.0f);
  p.panicle_branch_radius_m = std::max(0.0001f, SampleDistribution(panicle_branch_radius_m, rng));
  p.panicle_branch_angle_degrees = std::clamp(SampleDistribution(panicle_branch_angle_degrees, rng), 0.0f, 85.0f);
  p.panicle_spikelet_length_m = std::max(0.001f, SampleDistribution(panicle_spikelet_length_m, rng));
  p.panicle_spikelet_radius_m = std::max(0.0002f, SampleDistribution(panicle_spikelet_radius_m, rng));
  p.panicle_pedicel_length_m = std::max(0.0f, SampleDistribution(panicle_pedicel_length_m, rng));

  // Tillering.
  p.tiller_count = std::clamp(static_cast<int>(std::round(SampleDistribution(tiller_count, rng))), tiller_count_min,
                              tiller_count_max);
  p.tiller_origin_ranks.assign(tiller_origin_rank_order.begin(), tiller_origin_rank_order.begin() + p.tiller_count);
  p.tiller_emergence_main_leaf_stages = tiller_emergence_main_leaf_stages;
  p.tiller_insertion_angle = tiller_insertion_angle;
  p.tiller_final_lean_angle = tiller_final_lean_angle;
  p.tiller_azimuth_jitter = tiller_azimuth_jitter;
  p.tiller_same_side_splay_angle = std::clamp(SampleDistribution(tiller_same_side_splay_angle, rng), 0.0f, 45.0f);
  p.tiller_recovery_axis_fraction = tiller_recovery_axis_fraction;
  p.tiller_leaf_count_ratio = tiller_leaf_count_ratio;
  p.tiller_height_ratio = tiller_height_ratio;
  p.tiller_leaf_area_ratio_by_origin = tiller_leaf_area_ratio_by_origin;
  p.tiller_thickness_ratio = tiller_thickness_ratio;
  p.tiller_max_axis_length_ratio = std::clamp(SampleDistribution(tiller_max_axis_length_ratio, rng), 0.1f, 1.5f);

  // Thermal block.
  p.plastochron_gdd = std::max(1.0f, SampleDistribution(plastochron_gdd, rng));
  p.maturity_gdd = std::max(1.0f, SampleDistribution(maturity_gdd, rng));
  p.main_axis_plastochron_scale = std::max(0.1f, SampleDistribution(main_axis_plastochron_scale, rng));
  p.lateral_axis_plastochron_scale = std::max(0.1f, SampleDistribution(lateral_axis_plastochron_scale, rng));
  p.lateral_bud_plastochron_scale = std::max(0.1f, SampleDistribution(lateral_bud_plastochron_scale, rng));
  p.maturity_initiation_coupling = std::max(0.0f, SampleDistribution(maturity_initiation_coupling, rng));
  p.reference_maturity_gdd = std::max(1.0f, SampleDistribution(reference_maturity_gdd, rng));
  p.gdd_step = 1.0f;

  // Growth curves.
  p.internode_elongation_curve = internode_elongation_curve;
  p.internode_thickness_curve = internode_thickness_curve;
  p.leaf_sheath_length_growth_curve = leaf_sheath_length_growth_curve;
  p.leaf_neck_length_growth_curve = leaf_neck_length_growth_curve;
  p.leaf_blade_growth_curve = leaf_blade_growth_curve;
  p.leaf_sheath_width_growth_curve = leaf_sheath_width_growth_curve;
  p.leaf_neck_width_growth_curve = leaf_neck_width_growth_curve;
  p.leaf_width_growth_curve = leaf_width_growth_curve;
  p.leaf_angle_development_curve = leaf_angle_development_curve;
  p.leaf_curling_development_curve = leaf_curling_development_curve;
  p.leaf_bending_development_curve = leaf_bending_development_curve;
  p.width_along_sheath = width_along_sheath;
  p.width_along_neck = width_along_neck;
  p.width_along_leaf = width_along_leaf;
  p.bending_along_leaf = bending_along_leaf;
  p.curling_along_leaf = curling_along_leaf;
  p.waviness_along_leaf = waviness_along_leaf;

  // Tropisms.
  SampleTropisms(tropisms, rng, p.tropisms);

  return p;
}

// ---------------------------------------------------------------------------
// Instantiate
//
// Create entity with SorghumLS private component.
// ---------------------------------------------------------------------------

Entity SorghumLSDescriptor::Instantiate() const {
  const auto scene = GetApplication().GetActiveScene();
  if (!scene)
    return {};

  const auto entity = scene->CreateEntity(GetTitle());
  const auto sorghum = scene->GetOrSetPrivateComponent<SorghumLS>(entity).lock();
  sorghum->descriptor_ref = GetSelf();
  sorghum->target_gdd = SampleTargetGddForSeed(target_gdd, sorghum->seed);
  sorghum->GenerateGeometryEntities();

  return entity;
}

// ---------------------------------------------------------------------------
// Inspector UI
// ---------------------------------------------------------------------------

bool SorghumLSDescriptor::DrawEditorControls(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  bool editor_preferences_changed = false;

  const auto show_item_hover_description = [](const char* description) {
    if (!description || description[0] == '\0') {
      return;
    }
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
      ImGui::SetTooltip("%s", description);
    }
  };

  const auto inspect_plotted_distribution = [](const char* label,
                                               evo_engine::PlottedDistribution<float>& distribution) {
    return distribution.Draw(label, DistributionDefaults::MakePlottedGuiSettings());
  };

  if (ImGui::Button("Instantiate")) {
    editor_layer->SetSelectedEntity(Instantiate());
  }
  show_item_hover_description("Create a new SorghumLS entity from this descriptor and select it.");

  ImGui::SameLine();
  if (ImGui::Checkbox("Live Preview", &live_preview)) {
    editor_preferences_changed = true;
    if (!live_preview) {
      live_preview_dirty_ = false;
      live_preview_was_dragging_ = false;
      live_preview_needs_full_apply_ = false;
    }
  }
  show_item_hover_description("Regenerate matching SorghumLS entities while editing this descriptor.");

  if (ImGui::Checkbox("Representative Only While Dragging", &live_preview_representative_only)) {
    editor_preferences_changed = true;
  }
  show_item_hover_description("While dragging controls, preview one matching SorghumLS for responsiveness.");

  if (ImGui::Checkbox("Cap Preview Target GDD", &live_preview_cap_target_gdd)) {
    editor_preferences_changed = true;
  }
  show_item_hover_description("Clamp preview simulation age while dragging.");

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

  if (ImGui::TreeNodeEx("Grid Instantiate")) {
    ImGui::DragInt("Rows", &grid_rows, 1, 1, 50);
    show_item_hover_description("Number of rows for grid instantiation.");
    ImGui::DragInt("Cols", &grid_cols, 1, 1, 50);
    show_item_hover_description("Number of columns for grid instantiation.");
    ImGui::DragFloat("Spacing", &grid_spacing, 0.1f, 0.5f, 50.0f);
    show_item_hover_description("World-space spacing between neighboring sorghum plants.");

    if (ImGui::Button("Instantiate Grid")) {
      const auto scene = GetApplication().GetActiveScene();
      if (scene) {
        const auto container = scene->CreateEntity("Sorghum Grid");
        const float offset_y = (static_cast<float>(grid_rows) - 1.0f) * grid_spacing * 0.5f;
        const float offset_z = (static_cast<float>(grid_cols) - 1.0f) * grid_spacing * 0.5f;
        const auto base_seed =
            static_cast<unsigned int>(std::chrono::steady_clock::now().time_since_epoch().count() & 0xFFFFFFFFu);
        for (int i = 0; i < grid_rows; i++) {
          for (int j = 0; j < grid_cols; j++) {
            const auto entity =
                scene->CreateEntity(GetTitle() + " [" + std::to_string(i) + "," + std::to_string(j) + "]");
            const auto sorghum = scene->GetOrSetPrivateComponent<SorghumLS>(entity).lock();
            sorghum->descriptor_ref = GetSelf();
            sorghum->seed = base_seed + static_cast<unsigned int>(i * grid_cols + j);
            sorghum->target_gdd = SampleTargetGddForSeed(target_gdd, sorghum->seed);

            scene->SetParent(entity, container, false);

            Transform transform;
            transform.SetPosition(glm::vec3(0.0f, static_cast<float>(i) * grid_spacing - offset_y,
                                            static_cast<float>(j) * grid_spacing - offset_z));
            scene->SetDataComponent(entity, transform);

            sorghum->GenerateGeometryEntities();
          }
        }
      }
    }
    show_item_hover_description("Spawn a grid of SorghumLS entities using this descriptor.");

    ImGui::SameLine();
    if (ImGui::Button("Delete Grid")) {
      const auto scene = GetApplication().GetActiveScene();
      if (scene) {
        const auto* sorghum_entities_ptr = scene->UnsafeGetPrivateComponentOwnersList<SorghumLS>();
        if (sorghum_entities_ptr) {
          const std::vector<Entity> sorghum_entities = *sorghum_entities_ptr;
          std::vector<Entity> to_delete;
          std::vector<Entity> containers;
          for (const auto& entity : sorghum_entities) {
            if (!scene->IsEntityValid(entity))
              continue;
            auto sorghum = scene->GetOrSetPrivateComponent<SorghumLS>(entity).lock();
            if (!sorghum)
              continue;
            if (sorghum->descriptor_ref.Get<SorghumLSDescriptor>().get() == this) {
              to_delete.push_back(entity);
              const auto parent = scene->GetParent(entity);
              if (scene->IsEntityValid(parent) && scene->GetEntityName(parent) == "Sorghum Grid") {
                containers.push_back(parent);
              }
            }
          }
          for (const auto& entity : to_delete) {
            scene->DeleteEntity(entity);
          }
          std::sort(containers.begin(), containers.end(), [](const Entity& a, const Entity& b) {
            return a.GetIndex() < b.GetIndex();
          });
          containers.erase(std::unique(containers.begin(), containers.end()), containers.end());
          for (const auto& container : containers) {
            if (scene->IsEntityValid(container)) {
              scene->DeleteEntity(container);
            }
          }
        }
      }
    }
    show_item_hover_description("Delete SorghumLS entities driven by this descriptor.");

    ImGui::TreePop();
  }

  ImGui::Separator();

  if (ImGui::TreeNodeEx("Parameter Space Explorer")) {
    if (!explorer_.IsBound()) {
      explorer_.Bind(*this);
    }
    if (explorer_.DrawGui()) {
      changed = true;
    }
    show_item_hover_description("Interactive parameter sweeps and sensitivity exploration.");
    ImGui::TreePop();
  }

  ImGui::Separator();

  if (ImGui::TreeNodeEx("Leaf Material")) {
    changed |= editor_layer->DragAndDropButton<Texture2D>(leaf_atlas_albedo_texture, "Atlas Albedo");
    show_item_hover_description("Sorghum atlas albedo. Leaf blades use V 0.5..1.0; sheath and neck use V 0.0..0.5.");
    changed |= editor_layer->DragAndDropButton<Texture2D>(leaf_atlas_normal_texture, "Atlas Normal");
    changed |= editor_layer->DragAndDropButton<Texture2D>(leaf_atlas_roughness_texture, "Atlas Roughness");
    changed |= editor_layer->DragAndDropButton<Texture2D>(leaf_atlas_metallic_texture, "Atlas Metallic");
    changed |= editor_layer->DragAndDropButton<Texture2D>(leaf_atlas_ao_texture, "Atlas AO");
    int atlas_columns = static_cast<int>(leaf_atlas_variant_columns);
    if (ImGui::DragInt("Variant Columns", &atlas_columns, 1, 1, 64)) {
      leaf_atlas_variant_columns = static_cast<uint32_t>(std::max(1, atlas_columns));
      leaf_atlas_variant_count =
          std::clamp(leaf_atlas_variant_count, 1u, leaf_atlas_variant_columns * std::max(1u, leaf_atlas_variant_rows));
      changed = true;
    }
    int atlas_rows = static_cast<int>(leaf_atlas_variant_rows);
    if (ImGui::DragInt("Variant Rows", &atlas_rows, 1, 1, 64)) {
      leaf_atlas_variant_rows = static_cast<uint32_t>(std::max(1, atlas_rows));
      leaf_atlas_variant_count =
          std::clamp(leaf_atlas_variant_count, 1u, std::max(1u, leaf_atlas_variant_columns) * leaf_atlas_variant_rows);
      changed = true;
    }
    int atlas_count = static_cast<int>(leaf_atlas_variant_count);
    if (ImGui::DragInt("Variant Count", &atlas_count, 1, 1, 4096)) {
      const uint32_t max_count = std::max(1u, leaf_atlas_variant_columns) * std::max(1u, leaf_atlas_variant_rows);
      leaf_atlas_variant_count = std::clamp(static_cast<uint32_t>(std::max(1, atlas_count)), 1u, max_count);
      changed = true;
    }
    if (ImGui::DragFloat("Tile UV Inset", &leaf_atlas_tile_uv_inset, 0.0001f, 0.0f, 0.1f, "%.4f")) {
      leaf_atlas_tile_uv_inset = std::max(0.0f, leaf_atlas_tile_uv_inset);
      changed = true;
    }
    if (ImGui::Checkbox("Distal Region Uses Top Half", &leaf_atlas_distal_region_uses_top_half)) {
      changed = true;
    }
    changed |= ImGui::Checkbox("2x2 Surface Semantics", &leaf_atlas_semantic_quadrants);
    show_item_hover_description(
        "Within each variant: top blade, bottom blade, sheath/neck exterior, sheath/neck interior.");
    changed |= ImGui::ColorEdit3("Leaf Albedo Fallback", &leaf_material_albedo_color.x);
    changed |= ImGui::DragFloat("Leaf Roughness", &leaf_material_roughness, 0.01f, 0.0f, 1.0f);
    changed |= ImGui::DragFloat("Leaf Metallic", &leaf_material_metallic, 0.01f, 0.0f, 1.0f);
    changed |= ImGui::DragFloat("Leaf Specular", &leaf_material_specular, 0.01f, 0.0f, 1.0f);
    changed |= ImGui::DragFloat("Leaf Subsurface", &leaf_material_subsurface_factor, 0.01f, 0.0f, 1.0f);
    changed |= ImGui::ColorEdit3("Subsurface Color", &leaf_material_subsurface_color.x);
    changed |=
        ImGui::DragFloat3("Subsurface Radius (m)", &leaf_material_subsurface_radius.x, 0.0001f, 0.0f, 0.02f, "%.4f");
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Stem Material")) {
    changed |= editor_layer->DragAndDropButton<Texture2D>(stem_albedo_texture, "Stem Albedo");
    changed |= editor_layer->DragAndDropButton<Texture2D>(stem_normal_texture, "Stem Normal");
    changed |= editor_layer->DragAndDropButton<Texture2D>(stem_roughness_texture, "Stem Roughness");
    changed |= editor_layer->DragAndDropButton<Texture2D>(stem_metallic_texture, "Stem Metallic");
    changed |= editor_layer->DragAndDropButton<Texture2D>(stem_ao_texture, "Stem AO");
    changed |= ImGui::ColorEdit3("Stem Albedo Fallback", &stem_material_albedo_color.x);
    changed |= ImGui::DragFloat("Stem Roughness", &stem_material_roughness, 0.01f, 0.0f, 1.0f);
    changed |= ImGui::DragFloat("Stem Metallic", &stem_material_metallic, 0.01f, 0.0f, 1.0f);
    changed |= ImGui::DragFloat("Stem Specular", &stem_material_specular, 0.01f, 0.0f, 1.0f);
    ImGui::TreePop();
  }

  ImGui::Separator();

  if (ImGui::TreeNodeEx("Measured Endpoint Controls", ImGuiTreeNodeFlags_DefaultOpen)) {
    changed |= total_phytomer_count.Draw("Phytomer Count", 0.5f);
    changed |= inspect_plotted_distribution("Internode Length", internode_length);
    changed |= inspect_plotted_distribution("Internode Thickness", internode_thickness);
    changed |= inspect_plotted_distribution("Sheath Length", leaf_sheath_length);
    changed |= inspect_plotted_distribution("Neck Length", leaf_neck_length);
    changed |= inspect_plotted_distribution("Blade Length", leaf_blade_length);
    changed |= inspect_plotted_distribution("Sheath End Width Ratio", leaf_sheath_end_width_ratio);
    changed |= inspect_plotted_distribution("Neck End Width Ratio", leaf_neck_end_width_ratio);
    changed |= inspect_plotted_distribution("Blade End Width Ratio", leaf_blade_end_width_ratio);
    ImGui::TextDisabled("Continuity: sheath u=0 matches internode width; neck/blade u=0 inherit previous stage width.");
    changed |= inspect_plotted_distribution("Blade Max Width (m)", leaf_blade_max_width);
    changed |= inspect_plotted_distribution("Blade Thickness (m)", leaf_blade_thickness);
    changed |= inspect_plotted_distribution("Sheath Thickness (m)", leaf_sheath_thickness);
    changed |= ImGui::DragFloat("Leaf Width Scale", &leaf_width_scale, 0.01f, 0.05f, 3.0f);
    changed |= inspect_plotted_distribution("Leaf Insertion Angle", leaf_insertion_angle);
    changed |= inspect_plotted_distribution("Leaf Curling / Opening (deg)", leaf_curling);
    changed |= tiller_count.Draw("Tiller Count", 0.5f);
    changed |= target_gdd.Draw("Target GDD", 5.0f);
    changed |= maturity_gdd.Draw("Maturity GDD", 5.0f);
    ImGui::TreePop();
  }

  ImGui::Separator();

  if (ImGui::TreeNodeEx("Culm Topology")) {
    changed |= total_phytomer_count.Draw("Total Phytomer Count", 0.5f);
    changed |= phyllotaxis_angle.Draw("Phyllotaxis Angle (deg)", 1.0f);
    changed |= branch_azimuth_offset.Draw("Branch Azimuth Offset (deg)", 0.5f);
    changed |= main_culm_lean_angle.Draw("Main Culm Lean (deg)", 0.25f);
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Internode Morphology")) {
    changed |= inspect_plotted_distribution("Internode Length (Rank)", internode_length);
    changed |= inspect_plotted_distribution("Internode Thickness (Rank)", internode_thickness);
    int radial_segments = static_cast<int>(culm_radial_segments);
    if (ImGui::DragInt("Culm Radial Segments", &radial_segments, 1.0f, 8, 64)) {
      culm_radial_segments = static_cast<uint32_t>(std::clamp(radial_segments, 8, 64));
      changed = true;
    }
    changed |= ImGui::DragFloat("Node Radius Scale", &culm_node_radius_scale, 0.01f, 1.0f, 1.5f);
    changed |= ImGui::DragFloat("Texture Repeat (m)", &culm_texture_repeat_m, 0.01f, 0.01f, 5.0f);
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Leaf Morphology")) {
    changed |= inspect_plotted_distribution("Sheath Length (Rank)", leaf_sheath_length);
    changed |= inspect_plotted_distribution("Neck Length (Rank)", leaf_neck_length);
    changed |= inspect_plotted_distribution("Blade Length (Rank)", leaf_blade_length);
    changed |= inspect_plotted_distribution("Sheath End Width Ratio (Rank)", leaf_sheath_end_width_ratio);
    changed |= inspect_plotted_distribution("Neck End Width Ratio (Rank)", leaf_neck_end_width_ratio);
    changed |= inspect_plotted_distribution("Blade End Width Ratio (Rank)", leaf_blade_end_width_ratio);
    changed |= inspect_plotted_distribution("Blade Max Width (Rank, m)", leaf_blade_max_width);
    changed |= inspect_plotted_distribution("Blade Thickness (Rank, m)", leaf_blade_thickness);
    changed |= inspect_plotted_distribution("Sheath Thickness (Rank, m)", leaf_sheath_thickness);
    changed |= leaf_sheath_radius_ratio.Draw("Sheath Radius / Culm Radius", 0.01f);
    changed |= leaf_sheath_cross_section_ratio.Draw("Sheath Cross-Section Ratio", 0.01f);
    changed |= leaf_sheath_wrap_angle.Draw("Sheath Wrap (deg)", 1.0f);
    changed |= ImGui::DragFloat("Leaf Width Scale", &leaf_width_scale, 0.01f, 0.05f, 3.0f);
    changed |= inspect_plotted_distribution("Insertion Angle (Rank)", leaf_insertion_angle);
    changed |= inspect_plotted_distribution("Roll Angle (Rank)", leaf_roll_angle);
    changed |= inspect_plotted_distribution("Leaf Curling / Opening (Rank, deg)", leaf_curling);
    changed |= inspect_plotted_distribution("Leaf Bending (Rank)", leaf_bending);
    changed |= inspect_plotted_distribution("Leaf Waviness (Rank)", leaf_waviness);
    changed |= inspect_plotted_distribution("Leaf Waviness / Half Width (Rank)", leaf_waviness_width_fraction);
    changed |= leaf_waviness_frequency.Draw("Waviness Frequency", 0.1f);
    changed |= leaf_waviness_wavelength_m.Draw("Waviness Wavelength (m)", 0.01f);
    changed |= leaf_centerline_waviness_fraction.Draw("Centerline Waviness / Length", 0.001f);
    changed |= leaf_static_wind_deflection_fraction.Draw("Static Wind Deflection / Length", 0.001f);
    changed |= leaf_axial_twist_max_degrees.Draw("Axial Twist Maximum (deg)", 0.25f);
    changed |= leaf_axial_twist_frequency_ratio_min.Draw("Axial Twist / Edge Frequency Minimum", 0.01f);
    changed |= leaf_axial_twist_frequency_ratio_max.Draw("Axial Twist / Edge Frequency Maximum", 0.01f);
    changed |= leaf_gravity_droop_compliance.Draw("Gravity Droop Compliance", 0.01f);
    changed |= leaf_damage_severity.Draw("Leaf Edge Damage", 0.01f);
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Leaf Lifecycle")) {
    changed |= leaf_lifespan_years.Draw("Leaf Lifespan (years)", 0.05f);
    changed |= leaf_wilting_years.Draw("Leaf Wilting Duration (years)", 0.02f);
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Flag Leaf")) {
    ImGui::TextDisabled("The terminal main-culm phytomer is tagged as the flag leaf.");
    changed |= flag_leaf_length_scale.Draw("Blade Length Scale", 0.01f);
    changed |= flag_leaf_width_scale.Draw("Blade Width Scale", 0.01f);
    changed |= flag_leaf_insertion_angle_offset.Draw("Insertion-Angle Offset (deg)", 0.25f);
    changed |= flag_leaf_bending_scale.Draw("Bending Scale", 0.01f);
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Panicle")) {
    changed |= ImGui::Checkbox("Enable Panicle", &enable_panicle);
    changed |= panicle_initiation_gdd.Draw("Initiation GDD", 1.0f);
    changed |= panicle_maturity_gdd.Draw("Maturity GDD", 5.0f);
    changed |= panicle_peduncle_length_m.Draw("Exserted Peduncle Length (m)", 0.005f);
    changed |= panicle_rachis_length_m.Draw("Rachis Length (m)", 0.005f);
    changed |= panicle_rachis_radius_m.Draw("Rachis Radius (m)", 0.0002f);
    changed |= panicle_primary_branch_count.Draw("Primary Branch Count", 0.5f);
    changed |= panicle_spikelet_pairs_per_branch.Draw("Spikelet Triads per Branch", 0.5f);
    changed |= panicle_branch_length_m.Draw("Branch Length (m)", 0.002f);
    changed |= panicle_branch_length_taper.Draw("Branch Envelope Taper", 0.01f);
    changed |= panicle_branch_radius_m.Draw("Branch Radius (m)", 0.0001f);
    changed |= panicle_branch_angle_degrees.Draw("Branch Angle (deg)", 0.25f);
    changed |= panicle_spikelet_length_m.Draw("Spikelet Length (m)", 0.0005f);
    changed |= panicle_spikelet_radius_m.Draw("Spikelet Radius (m)", 0.0001f);
    changed |= panicle_pedicel_length_m.Draw("Pedicel Length (m)", 0.0005f);
    changed |= ImGui::ColorEdit3("Immature Color", &panicle_immature_color.x);
    changed |= ImGui::ColorEdit3("Mature Color", &panicle_mature_color.x);
    changed |= ImGui::DragFloat("Roughness", &panicle_material_roughness, 0.01f, 0.0f, 1.0f);
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Tillering")) {
    ImGui::TextDisabled("v4 crown-attached peer-height primary tillers; no secondary tillers");
    changed |= tiller_count.Draw("Tiller Count", 0.5f);
    changed |= ImGui::DragInt("Minimum Tiller Count", &tiller_count_min, 1.0f, 0, 6);
    changed |= ImGui::DragInt("Maximum Tiller Count", &tiller_count_max, 1.0f, 0, 6);
    for (size_t i = 0; i < tiller_origin_rank_order.size(); ++i) {
      changed |= ImGui::DragInt(("Origin Priority " + std::to_string(i + 1) + "##tiller_origin").c_str(),
                                &tiller_origin_rank_order[i], 1.0f, 1, 6);
    }
    for (size_t i = 0; i < tiller_emergence_main_leaf_stages.size(); ++i) {
      changed |= ImGui::DragInt(("T" + std::to_string(i + 1) + " Emergence (expanded main leaves)").c_str(),
                                &tiller_emergence_main_leaf_stages[i], 1.0f, 1, 128);
    }
    changed |= tiller_insertion_angle.Draw("Tiller Insertion Angle", 0.5f);
    changed |= tiller_final_lean_angle.Draw("Tiller Final Lean Angle", 0.25f);
    changed |= tiller_azimuth_jitter.Draw("Tiller Azimuth Jitter", 0.25f);
    changed |= tiller_same_side_splay_angle.Draw("Same-Side Tiller Splay", 0.25f);
    changed |= ImGui::DragFloat("Recovery Axis Fraction", &tiller_recovery_axis_fraction, 0.01f, 0.05f, 1.0f);
    changed |= tiller_leaf_count_ratio.Draw("Tiller/Main Leaf Count Ratio", 0.01f);
    changed |= tiller_height_ratio.Draw("Tiller/Main Culm-Tip Height Ratio", 0.01f);
    changed |= inspect_plotted_distribution("Tiller Leaf Area Ratio by Origin", tiller_leaf_area_ratio_by_origin);
    changed |= tiller_thickness_ratio.Draw("Tiller Thickness Ratio", 0.01f);
    changed |= tiller_max_axis_length_ratio.Draw("Maximum Tiller/Main Axis Length", 0.01f);
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Thermal Development")) {
    changed |= target_gdd.Draw("Target GDD", 5.0f);
    changed |= gdd_per_day.Draw("Thermal GDD/day", 0.25f);
    changed |= plastochron_gdd.Draw("Plastochron GDD", 1.0f);
    changed |= maturity_gdd.Draw("Maturity GDD", 5.0f);
    changed |= main_axis_plastochron_scale.Draw("Main-Axis Plastochron Scale", 0.01f);
    changed |= lateral_axis_plastochron_scale.Draw("Lateral-Axis Plastochron Scale", 0.01f);
    changed |= lateral_bud_plastochron_scale.Draw("Lateral-Bud Plastochron Scale", 0.01f);
    changed |= maturity_initiation_coupling.Draw("Maturity->Initiation Coupling", 0.01f);
    changed |= reference_maturity_gdd.Draw("Reference Maturity GDD", 5.0f);
    changed |= ImGui::Checkbox("Finalize Authored Snapshot", &finalize_snapshot_morphology);
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Growth Curves")) {
    changed |= DistributionDefaults::InspectPlottedDistributionCategory(
        "Internode Development Curves",
        {
            {"Internode Elongation", &internode_elongation_curve,
             "Controls normalized internode length progression over thermal age."},
            {"Internode Thickness", &internode_thickness_curve,
             "Controls normalized internode thickness progression over thermal age."},
        },
        ImGuiTreeNodeFlags_DefaultOpen);

    changed |= DistributionDefaults::InspectPlottedDistributionCategory(
        "Leaf Temporal Development Curves",
        {
            {"Sheath Length", &leaf_sheath_length_growth_curve,
             "Controls normalized sheath elongation over thermal age."},
            {"Neck Length", &leaf_neck_length_growth_curve, "Controls normalized neck elongation over thermal age."},
            {"Blade Length", &leaf_blade_growth_curve, "Controls normalized blade elongation over thermal age."},
            {"Sheath Width", &leaf_sheath_width_growth_curve,
             "Controls normalized sheath width-ratio progression over thermal age."},
            {"Neck Width", &leaf_neck_width_growth_curve,
             "Controls normalized neck width-ratio progression over thermal age."},
            {"Blade Width", &leaf_width_growth_curve,
             "Controls normalized blade width-ratio progression over thermal age."},
            {"Leaf Angle", &leaf_angle_development_curve,
             "Controls normalized insertion-angle progression over thermal age."},
            {"Leaf Curling", &leaf_curling_development_curve,
             "Controls normalized blade opening/curling development over thermal age."},
            {"Leaf Bending", &leaf_bending_development_curve,
             "Controls normalized bending development over thermal age."},
            {"Gravity Droop Age Response", &leaf_gravity_droop_age_response,
             "Controls effective leaf compliance over normalized thermal age."},
        },
        ImGuiTreeNodeFlags_DefaultOpen);

    changed |= DistributionDefaults::InspectPlottedDistributionCategory(
        "Leaf Spatial Profile Curves",
        {
            {"Sheath Width Profile Along Stage", &width_along_sheath,
             "Profile for sheath width ratio from stage start (x=0) to sheath end (x=1)."},
            {"Neck Width Profile Along Stage", &width_along_neck,
             "Profile for neck width ratio from stage start (x=0) to neck end (x=1)."},
            {"Blade Width Profile Along Stage", &width_along_leaf,
             "Profile for blade width ratio from stage start (x=0) to blade tip (x=1)."},
            {"Bending Along Leaf", &bending_along_leaf,
             "Profile multiplier of longitudinal bending from collar (x=0) to tip (x=1)."},
            {"Curling Along Leaf", &curling_along_leaf,
             "Profile multiplier of blade opening/curling from collar (x=0) to tip (x=1)."},
            {"Waviness Along Leaf", &waviness_along_leaf,
             "Profile of waviness amplitude from collar (x=0) to tip (x=1)."},
            {"Flexural Stiffness Along Leaf", &leaf_flexural_stiffness_along_leaf,
             "Relative longitudinal bending stiffness from blade base to tip."},
        },
        ImGuiTreeNodeFlags_DefaultOpen);

    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Tropisms")) {
    if (ImGui::Button("+ Add Tropism")) {
      tropisms.emplace_back();
      DistributionDefaults::ApplyMeanStdPlotDefaults(tropisms.back().order_response);
      changed = true;
    }
    int remove_idx = -1;
    for (int i = 0; i < static_cast<int>(tropisms.size()); i++) {
      ImGui::PushID(i);
      const std::string header = "Tropism " + std::to_string(i);
      if (ImGui::TreeNodeEx(header.c_str())) {
        changed |= tropisms[i].direction_x.Draw("Direction X", 0.01f);
        changed |= tropisms[i].direction_y.Draw("Direction Y", 0.01f);
        changed |= tropisms[i].direction_z.Draw("Direction Z", 0.01f);
        changed |= tropisms[i].strength.Draw("Strength", 0.01f);
        if (ImGui::DragFloat("Usage Chance (%)", &tropisms[i].usage_chance_percent, 0.5f, 0.0f, 100.0f, "%.1f")) {
          tropisms[i].usage_chance_percent = std::clamp(tropisms[i].usage_chance_percent, 0.0f, 100.0f);
          changed = true;
        }
        changed |= inspect_plotted_distribution("Order Response", tropisms[i].order_response);
        if (ImGui::Button("Remove")) {
          remove_idx = i;
        }
        ImGui::TreePop();
      }
      ImGui::PopID();
    }
    if (remove_idx >= 0) {
      tropisms.erase(tropisms.begin() + remove_idx);
      changed = true;
    }
    ImGui::TreePop();
  }

  ClampDescriptorValues(*this);

  if (changed && live_preview) {
    live_preview_dirty_ = true;
    live_preview_request_count_++;
  }

  const bool drag_active = ImGui::IsMouseDown(ImGuiMouseButton_Left) && ImGui::IsAnyItemActive();
  const bool drag_ended = live_preview_was_dragging_ && !drag_active;
  live_preview_was_dragging_ = live_preview && drag_active;

  if (drag_ended && live_preview && live_preview_needs_full_apply_) {
    live_preview_dirty_ = true;
  }

  if (changed || editor_preferences_changed) {
    SetUnsaved();
  }

  const auto regenerate_matching_plants = [&](const bool representative_only, const bool preview) {
    const float preview_target = live_preview_cap_target_gdd ? 1000.0f : std::numeric_limits<float>::max();
    if (const auto layer = GetApplication().GetLayer<LSystemLayer>()) {
      return layer->RegenerateSorghumDescriptor(*this, representative_only, preview, preview_target) > 0;
    }

    const auto scene = GetApplication().GetActiveScene();
    if (!scene) {
      return false;
    }
    const auto* owners = scene->UnsafeGetPrivateComponentOwnersList<SorghumLS>();
    if (!owners) {
      return false;
    }
    bool regenerated = false;
    for (const auto& entity : *owners) {
      if (!scene->IsEntityValid(entity)) {
        continue;
      }
      const auto plant = scene->GetOrSetPrivateComponent<SorghumLS>(entity).lock();
      if (!plant || plant->descriptor_ref.Get<SorghumLSDescriptor>().get() != this) {
        continue;
      }
      plant->target_gdd = SampleTargetGddForSeed(target_gdd, plant->seed);
      if (preview) {
        plant->GeneratePreviewGeometryEntities(preview_target, 64u);
      } else {
        plant->GenerateGeometryEntities(true, true);
      }
      regenerated = true;
      if (representative_only) {
        break;
      }
    }
    return regenerated;
  };

  if (live_preview && live_preview_dirty_) {
    const double now_seconds = GetSteadyTimeSeconds();
    const double min_interval_seconds = 1.0 / 12.0;  // Fixed 12Hz preview rate

    const bool throttle_ready = live_preview_last_apply_seconds_ < 0.0 ||
                                (now_seconds - live_preview_last_apply_seconds_) >= min_interval_seconds;

    if (!drag_active || throttle_ready) {
      const double apply_start_seconds = GetSteadyTimeSeconds();
      bool applied_any = false;
      if (drag_active) {
        applied_any = regenerate_matching_plants(live_preview_representative_only, true);
        live_preview_needs_full_apply_ |= applied_any;
      } else {
        applied_any = regenerate_matching_plants(false, false);
        live_preview_needs_full_apply_ = false;
      }

      const double apply_end_seconds = GetSteadyTimeSeconds();
      live_preview_last_apply_seconds_ = apply_end_seconds;
      live_preview_last_apply_ms_ = (apply_end_seconds - apply_start_seconds) * 1000.0;
      if (applied_any) {
        live_preview_total_apply_ms_ += live_preview_last_apply_ms_;
        live_preview_apply_count_++;
      }
      live_preview_dirty_ = false;
    } else {
      live_preview_coalesced_count_++;
    }
  }

  return changed || editor_preferences_changed;
}

// ---------------------------------------------------------------------------
// Serialize / Deserialize
// ---------------------------------------------------------------------------

bool l_system_package::InspectSorghumLSDescriptor(InspectorContext& context, SorghumLSDescriptor& descriptor) {
  return descriptor.DrawEditorControls(context.editor_layer);
}

void l_system_package::SerializeSorghumLSDescriptor(YAML::Emitter& out, const SorghumLSDescriptor& target) {
  // Culm topology.
  target.total_phytomer_count.Save("total_phytomer_count", out);
  target.phyllotaxis_angle.Save("phyllotaxis_angle", out);
  target.branch_azimuth_offset.Save("branch_azimuth_offset", out);
  target.main_culm_lean_angle.Save("main_culm_lean_angle", out);

  // Internode + leaf morphology.
  target.internode_length.Save("internode_length", out);
  target.internode_thickness.Save("internode_thickness", out);
  target.leaf_blade_length.Save("leaf_blade_length", out);
  target.leaf_blade_max_width.Save("leaf_blade_max_width", out);
  target.leaf_blade_thickness.Save("leaf_blade_thickness", out);
  target.leaf_sheath_thickness.Save("leaf_sheath_thickness", out);
  out << YAML::Key << "leaf_width_scale" << YAML::Value << target.leaf_width_scale;
  target.leaf_sheath_length.Save("leaf_sheath_length", out);
  target.leaf_neck_length.Save("leaf_neck_length", out);
  target.leaf_sheath_end_width_ratio.Save("leaf_sheath_end_width_ratio", out);
  target.leaf_neck_end_width_ratio.Save("leaf_neck_end_width_ratio", out);
  target.leaf_blade_end_width_ratio.Save("leaf_blade_end_width_ratio", out);
  target.leaf_insertion_angle.Save("leaf_insertion_angle", out);
  target.leaf_roll_angle.Save("leaf_roll_angle", out);
  target.leaf_curling.Save("leaf_curling", out);
  target.leaf_bending.Save("leaf_bending", out);
  target.leaf_waviness.Save("leaf_waviness", out);
  target.leaf_waviness_frequency.Save("leaf_waviness_frequency", out);
  target.leaf_waviness_width_fraction.Save("leaf_waviness_width_fraction", out);
  target.leaf_waviness_wavelength_m.Save("leaf_waviness_wavelength_m", out);
  target.leaf_centerline_waviness_fraction.Save("leaf_centerline_waviness_fraction", out);
  target.leaf_static_wind_deflection_fraction.Save("leaf_static_wind_deflection_fraction", out);
  target.leaf_axial_twist_max_degrees.Save("leaf_axial_twist_max_degrees", out);
  target.leaf_axial_twist_frequency_ratio_min.Save("leaf_axial_twist_frequency_ratio_min", out);
  target.leaf_axial_twist_frequency_ratio_max.Save("leaf_axial_twist_frequency_ratio_max", out);
  target.leaf_gravity_droop_compliance.Save("leaf_gravity_droop_compliance", out);
  target.leaf_gravity_droop_age_response.Save("leaf_gravity_droop_age_response", out);
  target.leaf_flexural_stiffness_along_leaf.Save("leaf_flexural_stiffness_along_leaf", out);
  target.leaf_damage_severity.Save("leaf_damage_severity", out);
  target.leaf_sheath_radius_ratio.Save("leaf_sheath_radius_ratio", out);
  target.leaf_sheath_cross_section_ratio.Save("leaf_sheath_cross_section_ratio", out);
  target.leaf_sheath_wrap_angle.Save("leaf_sheath_wrap_angle", out);
  target.leaf_blade_stage1_length_ratio.Save("leaf_blade_stage1_length_ratio", out);
  target.leaf_blade_stage2_length_ratio.Save("leaf_blade_stage2_length_ratio", out);
  target.leaf_blade_stage3_length_ratio.Save("leaf_blade_stage3_length_ratio", out);
  target.leaf_blade_stage1_width_scale.Save("leaf_blade_stage1_width_scale", out);
  target.leaf_blade_stage2_width_scale.Save("leaf_blade_stage2_width_scale", out);
  target.leaf_blade_stage3_width_scale.Save("leaf_blade_stage3_width_scale", out);

  // Leaf lifecycle.
  target.leaf_lifespan_years.Save("leaf_lifespan_years", out);
  target.leaf_wilting_years.Save("leaf_wilting_years", out);
  target.flag_leaf_length_scale.Save("flag_leaf_length_scale", out);
  target.flag_leaf_width_scale.Save("flag_leaf_width_scale", out);
  target.flag_leaf_insertion_angle_offset.Save("flag_leaf_insertion_angle_offset", out);
  target.flag_leaf_bending_scale.Save("flag_leaf_bending_scale", out);

  out << YAML::Key << "enable_panicle" << YAML::Value << target.enable_panicle;
  target.panicle_initiation_gdd.Save("panicle_initiation_gdd", out);
  target.panicle_maturity_gdd.Save("panicle_maturity_gdd", out);
  target.panicle_peduncle_length_m.Save("panicle_peduncle_length_m", out);
  target.panicle_rachis_length_m.Save("panicle_rachis_length_m", out);
  target.panicle_rachis_radius_m.Save("panicle_rachis_radius_m", out);
  target.panicle_primary_branch_count.Save("panicle_primary_branch_count", out);
  target.panicle_spikelet_pairs_per_branch.Save("panicle_spikelet_pairs_per_branch", out);
  target.panicle_branch_length_m.Save("panicle_branch_length_m", out);
  target.panicle_branch_length_taper.Save("panicle_branch_length_taper", out);
  target.panicle_branch_radius_m.Save("panicle_branch_radius_m", out);
  target.panicle_branch_angle_degrees.Save("panicle_branch_angle_degrees", out);
  target.panicle_spikelet_length_m.Save("panicle_spikelet_length_m", out);
  target.panicle_spikelet_radius_m.Save("panicle_spikelet_radius_m", out);
  target.panicle_pedicel_length_m.Save("panicle_pedicel_length_m", out);

  // Tillering.
  out << YAML::Key << "tiller_model_version" << YAML::Value << target.tiller_model_version;
  target.tiller_count.Save("tiller_count", out);
  out << YAML::Key << "tiller_count_min" << YAML::Value << target.tiller_count_min;
  out << YAML::Key << "tiller_count_max" << YAML::Value << target.tiller_count_max;
  SaveIntArray(out, "tiller_origin_rank_order", target.tiller_origin_rank_order);
  SaveIntArray(out, "tiller_emergence_main_leaf_stages", target.tiller_emergence_main_leaf_stages);
  target.tiller_insertion_angle.Save("tiller_insertion_angle", out);
  target.tiller_final_lean_angle.Save("tiller_final_lean_angle", out);
  target.tiller_azimuth_jitter.Save("tiller_azimuth_jitter", out);
  target.tiller_same_side_splay_angle.Save("tiller_same_side_splay_angle", out);
  out << YAML::Key << "tiller_recovery_axis_fraction" << YAML::Value << target.tiller_recovery_axis_fraction;
  target.tiller_leaf_count_ratio.Save("tiller_leaf_count_ratio", out);
  target.tiller_height_ratio.Save("tiller_height_ratio", out);
  target.tiller_leaf_area_ratio_by_origin.Save("tiller_leaf_area_ratio_by_origin", out);
  target.tiller_thickness_ratio.Save("tiller_thickness_ratio", out);
  target.tiller_max_axis_length_ratio.Save("tiller_max_axis_length_ratio", out);

  // Thermal.
  target.target_gdd.Save("target_gdd", out);
  target.gdd_per_day.Save("gdd_per_day", out);
  target.plastochron_gdd.Save("plastochron_gdd", out);
  target.maturity_gdd.Save("maturity_gdd", out);
  target.main_axis_plastochron_scale.Save("main_axis_plastochron_scale", out);
  target.lateral_axis_plastochron_scale.Save("lateral_axis_plastochron_scale", out);
  target.lateral_bud_plastochron_scale.Save("lateral_bud_plastochron_scale", out);
  target.maturity_initiation_coupling.Save("maturity_initiation_coupling", out);
  target.reference_maturity_gdd.Save("reference_maturity_gdd", out);
  out << YAML::Key << "finalize_snapshot_morphology" << YAML::Value << target.finalize_snapshot_morphology;

  // Growth curves.
  target.internode_elongation_curve.Save("internode_elongation_curve", out);
  target.internode_thickness_curve.Save("internode_thickness_curve", out);
  target.leaf_sheath_length_growth_curve.Save("leaf_sheath_length_growth_curve", out);
  target.leaf_neck_length_growth_curve.Save("leaf_neck_length_growth_curve", out);
  target.leaf_blade_growth_curve.Save("leaf_blade_growth_curve", out);
  target.leaf_sheath_width_growth_curve.Save("leaf_sheath_width_growth_curve", out);
  target.leaf_neck_width_growth_curve.Save("leaf_neck_width_growth_curve", out);
  target.leaf_width_growth_curve.Save("leaf_width_growth_curve", out);
  target.leaf_angle_development_curve.Save("leaf_angle_development_curve", out);
  target.leaf_curling_development_curve.Save("leaf_curling_development_curve", out);
  target.leaf_bending_development_curve.Save("leaf_bending_development_curve", out);
  target.width_along_sheath.Save("width_along_sheath", out);
  target.width_along_neck.Save("width_along_neck", out);
  target.width_along_leaf.Save("width_along_leaf", out);
  target.bending_along_leaf.Save("bending_along_leaf", out);
  target.curling_along_leaf.Save("curling_along_leaf", out);
  target.waviness_along_leaf.Save("waviness_along_leaf", out);

  // Leaf material atlas.
  target.leaf_atlas_albedo_texture.Save("leaf_atlas_albedo_texture", out);
  target.leaf_atlas_normal_texture.Save("leaf_atlas_normal_texture", out);
  target.leaf_atlas_roughness_texture.Save("leaf_atlas_roughness_texture", out);
  target.leaf_atlas_metallic_texture.Save("leaf_atlas_metallic_texture", out);
  target.leaf_atlas_ao_texture.Save("leaf_atlas_ao_texture", out);
  out << YAML::Key << "leaf_atlas_variant_columns" << YAML::Value << target.leaf_atlas_variant_columns;
  out << YAML::Key << "leaf_atlas_variant_rows" << YAML::Value << target.leaf_atlas_variant_rows;
  out << YAML::Key << "leaf_atlas_variant_count" << YAML::Value << target.leaf_atlas_variant_count;
  out << YAML::Key << "leaf_atlas_tile_uv_inset" << YAML::Value << target.leaf_atlas_tile_uv_inset;
  out << YAML::Key << "leaf_atlas_distal_region_uses_top_half" << YAML::Value
      << target.leaf_atlas_distal_region_uses_top_half;
  out << YAML::Key << "leaf_atlas_semantic_quadrants" << YAML::Value << target.leaf_atlas_semantic_quadrants;
  out << YAML::Key << "leaf_material_albedo_color" << YAML::Value << target.leaf_material_albedo_color;
  out << YAML::Key << "leaf_material_roughness" << YAML::Value << target.leaf_material_roughness;
  out << YAML::Key << "leaf_material_metallic" << YAML::Value << target.leaf_material_metallic;
  out << YAML::Key << "leaf_material_specular" << YAML::Value << target.leaf_material_specular;
  out << YAML::Key << "leaf_material_subsurface_factor" << YAML::Value << target.leaf_material_subsurface_factor;
  out << YAML::Key << "leaf_material_subsurface_color" << YAML::Value << target.leaf_material_subsurface_color;
  out << YAML::Key << "leaf_material_subsurface_radius" << YAML::Value << target.leaf_material_subsurface_radius;

  target.stem_albedo_texture.Save("stem_albedo_texture", out);
  target.stem_normal_texture.Save("stem_normal_texture", out);
  target.stem_roughness_texture.Save("stem_roughness_texture", out);
  target.stem_metallic_texture.Save("stem_metallic_texture", out);
  target.stem_ao_texture.Save("stem_ao_texture", out);
  out << YAML::Key << "stem_material_albedo_color" << YAML::Value << target.stem_material_albedo_color;
  out << YAML::Key << "stem_material_roughness" << YAML::Value << target.stem_material_roughness;
  out << YAML::Key << "stem_material_metallic" << YAML::Value << target.stem_material_metallic;
  out << YAML::Key << "stem_material_specular" << YAML::Value << target.stem_material_specular;
  out << YAML::Key << "panicle_immature_color" << YAML::Value << target.panicle_immature_color;
  out << YAML::Key << "panicle_mature_color" << YAML::Value << target.panicle_mature_color;
  out << YAML::Key << "panicle_material_roughness" << YAML::Value << target.panicle_material_roughness;
  out << YAML::Key << "culm_radial_segments" << YAML::Value << target.culm_radial_segments;
  out << YAML::Key << "culm_node_radius_scale" << YAML::Value << target.culm_node_radius_scale;
  out << YAML::Key << "culm_texture_repeat_m" << YAML::Value << target.culm_texture_repeat_m;

  out << YAML::Key << "live_preview" << YAML::Value << target.live_preview;
  out << YAML::Key << "live_preview_representative_only" << YAML::Value << target.live_preview_representative_only;
  out << YAML::Key << "live_preview_cap_target_gdd" << YAML::Value << target.live_preview_cap_target_gdd;
  out << YAML::Key << "grid_rows" << YAML::Value << target.grid_rows;
  out << YAML::Key << "grid_cols" << YAML::Value << target.grid_cols;
  out << YAML::Key << "grid_spacing" << YAML::Value << target.grid_spacing;

  // Explorer preferences.
  out << YAML::Key << "explorer_mode" << YAML::Value << static_cast<int>(target.explorer_.mode);
  out << YAML::Key << "explorer_speed" << YAML::Value << target.explorer_.speed;

  // Tropisms.
  out << YAML::Key << "tropism_count" << YAML::Value << static_cast<int>(target.tropisms.size());
  for (int i = 0; i < static_cast<int>(target.tropisms.size()); ++i) {
    const std::string prefix = "tropism_" + std::to_string(i) + "_";
    target.tropisms[i].direction_x.Save(prefix + "dir_x", out);
    target.tropisms[i].direction_y.Save(prefix + "dir_y", out);
    target.tropisms[i].direction_z.Save(prefix + "dir_z", out);
    target.tropisms[i].strength.Save(prefix + "strength", out);
    out << YAML::Key << prefix + "usage_chance_percent" << YAML::Value
        << std::clamp(target.tropisms[i].usage_chance_percent, 0.0f, 100.0f);
    target.tropisms[i].order_response.Save(prefix + "order_response", out);
  }
}

void l_system_package::DeserializeSorghumLSDescriptor(const YAML::Node& in, SorghumLSDescriptor& target) {
  auto& total_phytomer_count = target.total_phytomer_count;
  auto& phyllotaxis_angle = target.phyllotaxis_angle;
  auto& branch_azimuth_offset = target.branch_azimuth_offset;
  auto& main_culm_lean_angle = target.main_culm_lean_angle;
  auto& internode_length = target.internode_length;
  auto& internode_thickness = target.internode_thickness;
  auto& leaf_blade_length = target.leaf_blade_length;
  auto& leaf_blade_max_width = target.leaf_blade_max_width;
  auto& leaf_blade_thickness = target.leaf_blade_thickness;
  auto& leaf_sheath_thickness = target.leaf_sheath_thickness;
  auto& leaf_width_scale = target.leaf_width_scale;
  auto& leaf_sheath_length = target.leaf_sheath_length;
  auto& leaf_neck_length = target.leaf_neck_length;
  auto& leaf_sheath_end_width_ratio = target.leaf_sheath_end_width_ratio;
  auto& leaf_neck_end_width_ratio = target.leaf_neck_end_width_ratio;
  auto& leaf_blade_end_width_ratio = target.leaf_blade_end_width_ratio;
  auto& leaf_insertion_angle = target.leaf_insertion_angle;
  auto& leaf_roll_angle = target.leaf_roll_angle;
  auto& leaf_curling = target.leaf_curling;
  auto& leaf_bending = target.leaf_bending;
  auto& leaf_waviness = target.leaf_waviness;
  auto& leaf_waviness_frequency = target.leaf_waviness_frequency;
  auto& leaf_waviness_width_fraction = target.leaf_waviness_width_fraction;
  auto& leaf_waviness_wavelength_m = target.leaf_waviness_wavelength_m;
  auto& leaf_centerline_waviness_fraction = target.leaf_centerline_waviness_fraction;
  auto& leaf_static_wind_deflection_fraction = target.leaf_static_wind_deflection_fraction;
  auto& leaf_axial_twist_max_degrees = target.leaf_axial_twist_max_degrees;
  auto& leaf_axial_twist_frequency_ratio_min = target.leaf_axial_twist_frequency_ratio_min;
  auto& leaf_axial_twist_frequency_ratio_max = target.leaf_axial_twist_frequency_ratio_max;
  auto& leaf_gravity_droop_compliance = target.leaf_gravity_droop_compliance;
  auto& leaf_gravity_droop_age_response = target.leaf_gravity_droop_age_response;
  auto& leaf_flexural_stiffness_along_leaf = target.leaf_flexural_stiffness_along_leaf;
  auto& leaf_damage_severity = target.leaf_damage_severity;
  auto& leaf_sheath_radius_ratio = target.leaf_sheath_radius_ratio;
  auto& leaf_sheath_cross_section_ratio = target.leaf_sheath_cross_section_ratio;
  auto& leaf_sheath_wrap_angle = target.leaf_sheath_wrap_angle;
  auto& leaf_blade_stage1_length_ratio = target.leaf_blade_stage1_length_ratio;
  auto& leaf_blade_stage2_length_ratio = target.leaf_blade_stage2_length_ratio;
  auto& leaf_blade_stage3_length_ratio = target.leaf_blade_stage3_length_ratio;
  auto& leaf_blade_stage1_width_scale = target.leaf_blade_stage1_width_scale;
  auto& leaf_blade_stage2_width_scale = target.leaf_blade_stage2_width_scale;
  auto& leaf_blade_stage3_width_scale = target.leaf_blade_stage3_width_scale;
  auto& leaf_lifespan_years = target.leaf_lifespan_years;
  auto& leaf_wilting_years = target.leaf_wilting_years;
  auto& flag_leaf_length_scale = target.flag_leaf_length_scale;
  auto& flag_leaf_width_scale = target.flag_leaf_width_scale;
  auto& flag_leaf_insertion_angle_offset = target.flag_leaf_insertion_angle_offset;
  auto& flag_leaf_bending_scale = target.flag_leaf_bending_scale;
  auto& enable_panicle = target.enable_panicle;
  auto& panicle_initiation_gdd = target.panicle_initiation_gdd;
  auto& panicle_maturity_gdd = target.panicle_maturity_gdd;
  auto& panicle_peduncle_length_m = target.panicle_peduncle_length_m;
  auto& panicle_rachis_length_m = target.panicle_rachis_length_m;
  auto& panicle_rachis_radius_m = target.panicle_rachis_radius_m;
  auto& panicle_primary_branch_count = target.panicle_primary_branch_count;
  auto& panicle_spikelet_pairs_per_branch = target.panicle_spikelet_pairs_per_branch;
  auto& panicle_branch_length_m = target.panicle_branch_length_m;
  auto& panicle_branch_length_taper = target.panicle_branch_length_taper;
  auto& panicle_branch_radius_m = target.panicle_branch_radius_m;
  auto& panicle_branch_angle_degrees = target.panicle_branch_angle_degrees;
  auto& panicle_spikelet_length_m = target.panicle_spikelet_length_m;
  auto& panicle_spikelet_radius_m = target.panicle_spikelet_radius_m;
  auto& panicle_pedicel_length_m = target.panicle_pedicel_length_m;
  auto& tiller_model_version = target.tiller_model_version;
  auto& tiller_count = target.tiller_count;
  auto& tiller_count_min = target.tiller_count_min;
  auto& tiller_count_max = target.tiller_count_max;
  auto& tiller_origin_rank_order = target.tiller_origin_rank_order;
  auto& tiller_emergence_main_leaf_stages = target.tiller_emergence_main_leaf_stages;
  auto& tiller_initiation_delay_gdd = target.tiller_initiation_delay_gdd;
  auto& tiller_insertion_angle = target.tiller_insertion_angle;
  auto& tiller_final_lean_angle = target.tiller_final_lean_angle;
  auto& tiller_azimuth_jitter = target.tiller_azimuth_jitter;
  auto& tiller_same_side_splay_angle = target.tiller_same_side_splay_angle;
  auto& tiller_recovery_axis_fraction = target.tiller_recovery_axis_fraction;
  auto& tiller_leaf_count_ratio = target.tiller_leaf_count_ratio;
  auto& tiller_height_ratio = target.tiller_height_ratio;
  auto& tiller_leaf_area_ratio_by_origin = target.tiller_leaf_area_ratio_by_origin;
  auto& tiller_phytomer_count_scale = target.tiller_phytomer_count_scale;
  auto& tiller_thickness_ratio = target.tiller_thickness_ratio;
  auto& tiller_max_axis_length_ratio = target.tiller_max_axis_length_ratio;
  auto& target_gdd = target.target_gdd;
  auto& gdd_per_day = target.gdd_per_day;
  auto& plastochron_gdd = target.plastochron_gdd;
  auto& maturity_gdd = target.maturity_gdd;
  auto& main_axis_plastochron_scale = target.main_axis_plastochron_scale;
  auto& lateral_axis_plastochron_scale = target.lateral_axis_plastochron_scale;
  auto& lateral_bud_plastochron_scale = target.lateral_bud_plastochron_scale;
  auto& maturity_initiation_coupling = target.maturity_initiation_coupling;
  auto& reference_maturity_gdd = target.reference_maturity_gdd;
  auto& finalize_snapshot_morphology = target.finalize_snapshot_morphology;
  auto& internode_elongation_curve = target.internode_elongation_curve;
  auto& internode_thickness_curve = target.internode_thickness_curve;
  auto& leaf_sheath_length_growth_curve = target.leaf_sheath_length_growth_curve;
  auto& leaf_neck_length_growth_curve = target.leaf_neck_length_growth_curve;
  auto& leaf_blade_growth_curve = target.leaf_blade_growth_curve;
  auto& leaf_sheath_width_growth_curve = target.leaf_sheath_width_growth_curve;
  auto& leaf_neck_width_growth_curve = target.leaf_neck_width_growth_curve;
  auto& leaf_width_growth_curve = target.leaf_width_growth_curve;
  auto& leaf_angle_development_curve = target.leaf_angle_development_curve;
  auto& leaf_curling_development_curve = target.leaf_curling_development_curve;
  auto& leaf_bending_development_curve = target.leaf_bending_development_curve;
  auto& width_along_sheath = target.width_along_sheath;
  auto& width_along_neck = target.width_along_neck;
  auto& width_along_leaf = target.width_along_leaf;
  auto& bending_along_leaf = target.bending_along_leaf;
  auto& curling_along_leaf = target.curling_along_leaf;
  auto& waviness_along_leaf = target.waviness_along_leaf;
  auto& leaf_atlas_albedo_texture = target.leaf_atlas_albedo_texture;
  auto& leaf_atlas_normal_texture = target.leaf_atlas_normal_texture;
  auto& leaf_atlas_roughness_texture = target.leaf_atlas_roughness_texture;
  auto& leaf_atlas_metallic_texture = target.leaf_atlas_metallic_texture;
  auto& leaf_atlas_ao_texture = target.leaf_atlas_ao_texture;
  auto& leaf_atlas_variant_columns = target.leaf_atlas_variant_columns;
  auto& leaf_atlas_variant_rows = target.leaf_atlas_variant_rows;
  auto& leaf_atlas_variant_count = target.leaf_atlas_variant_count;
  auto& leaf_atlas_tile_uv_inset = target.leaf_atlas_tile_uv_inset;
  auto& leaf_atlas_distal_region_uses_top_half = target.leaf_atlas_distal_region_uses_top_half;
  auto& leaf_atlas_semantic_quadrants = target.leaf_atlas_semantic_quadrants;
  auto& leaf_material_albedo_color = target.leaf_material_albedo_color;
  auto& leaf_material_roughness = target.leaf_material_roughness;
  auto& leaf_material_metallic = target.leaf_material_metallic;
  auto& leaf_material_specular = target.leaf_material_specular;
  auto& leaf_material_subsurface_factor = target.leaf_material_subsurface_factor;
  auto& leaf_material_subsurface_color = target.leaf_material_subsurface_color;
  auto& leaf_material_subsurface_radius = target.leaf_material_subsurface_radius;
  auto& stem_albedo_texture = target.stem_albedo_texture;
  auto& stem_normal_texture = target.stem_normal_texture;
  auto& stem_roughness_texture = target.stem_roughness_texture;
  auto& stem_metallic_texture = target.stem_metallic_texture;
  auto& stem_ao_texture = target.stem_ao_texture;
  auto& stem_material_albedo_color = target.stem_material_albedo_color;
  auto& stem_material_roughness = target.stem_material_roughness;
  auto& stem_material_metallic = target.stem_material_metallic;
  auto& stem_material_specular = target.stem_material_specular;
  auto& panicle_immature_color = target.panicle_immature_color;
  auto& panicle_mature_color = target.panicle_mature_color;
  auto& panicle_material_roughness = target.panicle_material_roughness;
  auto& culm_radial_segments = target.culm_radial_segments;
  auto& culm_node_radius_scale = target.culm_node_radius_scale;
  auto& culm_texture_repeat_m = target.culm_texture_repeat_m;
  auto& live_preview = target.live_preview;
  auto& live_preview_representative_only = target.live_preview_representative_only;
  auto& live_preview_cap_target_gdd = target.live_preview_cap_target_gdd;
  auto& grid_rows = target.grid_rows;
  auto& grid_cols = target.grid_cols;
  auto& grid_spacing = target.grid_spacing;
  auto& explorer_ = target.explorer_;
  auto& tropisms = target.tropisms;

  LoadSingleDistributionWithScalarFallback(in, "total_phytomer_count", total_phytomer_count);
  LoadSingleDistributionWithScalarFallback(in, "phyllotaxis_angle", phyllotaxis_angle);
  LoadSingleDistributionWithScalarFallback(in, "branch_azimuth_offset", branch_azimuth_offset);
  LoadSingleDistributionWithScalarFallback(in, "main_culm_lean_angle", main_culm_lean_angle);

  internode_length.Load("internode_length", in);
  internode_thickness.Load("internode_thickness", in);
  leaf_blade_length.Load("leaf_blade_length", in);
  leaf_blade_max_width.Load("leaf_blade_max_width", in);
  leaf_blade_thickness.Load("leaf_blade_thickness", in);
  leaf_sheath_thickness.Load("leaf_sheath_thickness", in);
  if (in["leaf_width_scale"])
    leaf_width_scale = in["leaf_width_scale"].as<float>();
  leaf_sheath_length.Load("leaf_sheath_length", in);
  leaf_neck_length.Load("leaf_neck_length", in);
  leaf_sheath_end_width_ratio.Load("leaf_sheath_end_width_ratio", in);
  leaf_neck_end_width_ratio.Load("leaf_neck_end_width_ratio", in);
  leaf_blade_end_width_ratio.Load("leaf_blade_end_width_ratio", in);
  leaf_insertion_angle.Load("leaf_insertion_angle", in);
  leaf_roll_angle.Load("leaf_roll_angle", in);
  leaf_curling.Load("leaf_curling", in);
  if (IsLikelyLegacyNormalizedLeafCurling(leaf_curling)) {
    ScalePlottedDistributionRange(leaf_curling, kLeafCurlingLegacyScaleDeg);
  }
  leaf_bending.Load("leaf_bending", in);
  leaf_waviness.Load("leaf_waviness", in);
  LoadSingleDistributionWithScalarFallback(in, "leaf_waviness_frequency", leaf_waviness_frequency);
  if (in["leaf_waviness_width_fraction"])
    leaf_waviness_width_fraction.Load("leaf_waviness_width_fraction", in);
  LoadSingleDistributionWithScalarFallback(in, "leaf_waviness_wavelength_m", leaf_waviness_wavelength_m);
  LoadSingleDistributionWithScalarFallback(in, "leaf_centerline_waviness_fraction", leaf_centerline_waviness_fraction);
  LoadSingleDistributionWithScalarFallback(in, "leaf_static_wind_deflection_fraction",
                                           leaf_static_wind_deflection_fraction);
  if (in["leaf_axial_twist_max_degrees"])
    LoadSingleDistributionWithScalarFallback(in, "leaf_axial_twist_max_degrees", leaf_axial_twist_max_degrees);
  else
    LoadSingleDistributionWithScalarFallback(in, "leaf_static_twist_degrees", leaf_axial_twist_max_degrees);
  if (in["leaf_axial_twist_frequency_ratio_min"])
    LoadSingleDistributionWithScalarFallback(in, "leaf_axial_twist_frequency_ratio_min",
                                             leaf_axial_twist_frequency_ratio_min);
  if (in["leaf_axial_twist_frequency_ratio_max"])
    LoadSingleDistributionWithScalarFallback(in, "leaf_axial_twist_frequency_ratio_max",
                                             leaf_axial_twist_frequency_ratio_max);
  LoadSingleDistributionWithScalarFallback(in, "leaf_gravity_droop_compliance", leaf_gravity_droop_compliance);
  if (in["leaf_gravity_droop_age_response"])
    leaf_gravity_droop_age_response.Load("leaf_gravity_droop_age_response", in);
  if (in["leaf_flexural_stiffness_along_leaf"])
    leaf_flexural_stiffness_along_leaf.Load("leaf_flexural_stiffness_along_leaf", in);
  LoadSingleDistributionWithScalarFallback(in, "leaf_damage_severity", leaf_damage_severity);
  LoadSingleDistributionWithScalarFallback(in, "leaf_sheath_radius_ratio", leaf_sheath_radius_ratio);
  LoadSingleDistributionWithScalarFallback(in, "leaf_sheath_cross_section_ratio", leaf_sheath_cross_section_ratio);
  LoadSingleDistributionWithScalarFallback(in, "leaf_sheath_wrap_angle", leaf_sheath_wrap_angle);
  LoadSingleDistributionWithScalarFallback(in, "leaf_blade_stage1_length_ratio", leaf_blade_stage1_length_ratio);
  LoadSingleDistributionWithScalarFallback(in, "leaf_blade_stage2_length_ratio", leaf_blade_stage2_length_ratio);
  LoadSingleDistributionWithScalarFallback(in, "leaf_blade_stage3_length_ratio", leaf_blade_stage3_length_ratio);
  LoadSingleDistributionWithScalarFallback(in, "leaf_blade_stage1_width_scale", leaf_blade_stage1_width_scale);
  LoadSingleDistributionWithScalarFallback(in, "leaf_blade_stage2_width_scale", leaf_blade_stage2_width_scale);
  LoadSingleDistributionWithScalarFallback(in, "leaf_blade_stage3_width_scale", leaf_blade_stage3_width_scale);

  if (!in["leaf_sheath_end_width_ratio"] && in["leaf_blade_stage1_width_scale"]) {
    SetConstantPlottedDistribution(leaf_sheath_end_width_ratio, std::max(0.05f, leaf_blade_stage1_width_scale.mean));
  }
  if (!in["leaf_neck_end_width_ratio"] && in["leaf_blade_stage2_width_scale"]) {
    SetConstantPlottedDistribution(leaf_neck_end_width_ratio, std::max(0.05f, leaf_blade_stage2_width_scale.mean));
  }
  if (!in["leaf_blade_end_width_ratio"] && in["leaf_blade_stage3_width_scale"]) {
    SetConstantPlottedDistribution(leaf_blade_end_width_ratio, std::max(0.05f, leaf_blade_stage3_width_scale.mean));
  }

  LoadSingleDistributionWithScalarFallback(in, "leaf_lifespan_years", leaf_lifespan_years);
  LoadSingleDistributionWithScalarFallback(in, "leaf_wilting_years", leaf_wilting_years);
  LoadSingleDistributionWithScalarFallback(in, "flag_leaf_length_scale", flag_leaf_length_scale);
  LoadSingleDistributionWithScalarFallback(in, "flag_leaf_width_scale", flag_leaf_width_scale);
  LoadSingleDistributionWithScalarFallback(in, "flag_leaf_insertion_angle_offset", flag_leaf_insertion_angle_offset);
  LoadSingleDistributionWithScalarFallback(in, "flag_leaf_bending_scale", flag_leaf_bending_scale);

  if (in["enable_panicle"]) {
    enable_panicle = in["enable_panicle"].as<bool>();
  }
  LoadSingleDistributionWithScalarFallback(in, "panicle_initiation_gdd", panicle_initiation_gdd);
  LoadSingleDistributionWithScalarFallback(in, "panicle_maturity_gdd", panicle_maturity_gdd);
  LoadSingleDistributionWithScalarFallback(in, "panicle_peduncle_length_m", panicle_peduncle_length_m);
  LoadSingleDistributionWithScalarFallback(in, "panicle_rachis_length_m", panicle_rachis_length_m);
  LoadSingleDistributionWithScalarFallback(in, "panicle_rachis_radius_m", panicle_rachis_radius_m);
  LoadSingleDistributionWithScalarFallback(in, "panicle_primary_branch_count", panicle_primary_branch_count);
  LoadSingleDistributionWithScalarFallback(in, "panicle_spikelet_pairs_per_branch", panicle_spikelet_pairs_per_branch);
  LoadSingleDistributionWithScalarFallback(in, "panicle_branch_length_m", panicle_branch_length_m);
  LoadSingleDistributionWithScalarFallback(in, "panicle_branch_length_taper", panicle_branch_length_taper);
  LoadSingleDistributionWithScalarFallback(in, "panicle_branch_radius_m", panicle_branch_radius_m);
  LoadSingleDistributionWithScalarFallback(in, "panicle_branch_angle_degrees", panicle_branch_angle_degrees);
  LoadSingleDistributionWithScalarFallback(in, "panicle_spikelet_length_m", panicle_spikelet_length_m);
  LoadSingleDistributionWithScalarFallback(in, "panicle_spikelet_radius_m", panicle_spikelet_radius_m);
  LoadSingleDistributionWithScalarFallback(in, "panicle_pedicel_length_m", panicle_pedicel_length_m);

  if (!in["tiller_model_version"] || in["tiller_model_version"].as<uint32_t>() != 4u) {
    throw std::runtime_error("Sorghum descriptor requires tiller_model_version 4");
  }
  tiller_model_version = 4u;
  LoadSingleDistributionWithScalarFallback(in, "tiller_count", tiller_count);
  if (in["tiller_count_min"])
    tiller_count_min = in["tiller_count_min"].as<int>();
  if (in["tiller_count_max"])
    tiller_count_max = in["tiller_count_max"].as<int>();
  LoadIntArray(in, "tiller_origin_rank_order", tiller_origin_rank_order);
  LoadIntArray(in, "tiller_emergence_main_leaf_stages", tiller_emergence_main_leaf_stages);
  tiller_initiation_delay_gdd.Load("tiller_initiation_delay_gdd", in);
  LoadSingleDistributionWithScalarFallback(in, "tiller_insertion_angle", tiller_insertion_angle);
  LoadSingleDistributionWithScalarFallback(in, "tiller_final_lean_angle", tiller_final_lean_angle);
  LoadSingleDistributionWithScalarFallback(in, "tiller_azimuth_jitter", tiller_azimuth_jitter);
  LoadSingleDistributionWithScalarFallback(in, "tiller_same_side_splay_angle", tiller_same_side_splay_angle);
  if (in["tiller_recovery_axis_fraction"]) {
    tiller_recovery_axis_fraction = in["tiller_recovery_axis_fraction"].as<float>();
  }
  LoadSingleDistributionWithScalarFallback(in, "tiller_leaf_count_ratio", tiller_leaf_count_ratio);
  LoadSingleDistributionWithScalarFallback(in, "tiller_height_ratio", tiller_height_ratio);
  tiller_leaf_area_ratio_by_origin.Load("tiller_leaf_area_ratio_by_origin", in);
  LoadSingleDistributionWithScalarFallback(in, "tiller_phytomer_count_scale", tiller_phytomer_count_scale);
  LoadSingleDistributionWithScalarFallback(in, "tiller_thickness_ratio", tiller_thickness_ratio);
  LoadSingleDistributionWithScalarFallback(in, "tiller_max_axis_length_ratio", tiller_max_axis_length_ratio);

  LoadSingleDistributionWithScalarFallback(in, "target_gdd", target_gdd);
  if (in["gdd_per_day"]) {
    LoadSingleDistributionWithScalarFallback(in, "gdd_per_day", gdd_per_day);
  } else {
    // Legacy compatibility for assets authored before descriptor thermal-rate migration.
    LoadSingleDistributionWithScalarFallback(in, "gdd_per_second", gdd_per_day);
  }
  LoadSingleDistributionWithScalarFallback(in, "plastochron_gdd", plastochron_gdd);
  LoadSingleDistributionWithScalarFallback(in, "maturity_gdd", maturity_gdd);
  LoadSingleDistributionWithScalarFallback(in, "main_axis_plastochron_scale", main_axis_plastochron_scale);
  LoadSingleDistributionWithScalarFallback(in, "lateral_axis_plastochron_scale", lateral_axis_plastochron_scale);
  LoadSingleDistributionWithScalarFallback(in, "lateral_bud_plastochron_scale", lateral_bud_plastochron_scale);
  LoadSingleDistributionWithScalarFallback(in, "maturity_initiation_coupling", maturity_initiation_coupling);
  LoadSingleDistributionWithScalarFallback(in, "reference_maturity_gdd", reference_maturity_gdd);
  if (in["finalize_snapshot_morphology"]) {
    finalize_snapshot_morphology = in["finalize_snapshot_morphology"].as<bool>();
  }

  LoadGrowthCurveWithLegacyFallback(in, "internode_elongation_curve", internode_elongation_curve);
  LoadGrowthCurveWithLegacyFallback(in, "internode_thickness_curve", internode_thickness_curve);
  LoadGrowthCurveWithLegacyFallback(in, "leaf_sheath_length_growth_curve", leaf_sheath_length_growth_curve);
  LoadGrowthCurveWithLegacyFallback(in, "leaf_neck_length_growth_curve", leaf_neck_length_growth_curve);
  LoadGrowthCurveWithLegacyFallback(in, "leaf_blade_growth_curve", leaf_blade_growth_curve);
  LoadGrowthCurveWithLegacyFallback(in, "leaf_sheath_width_growth_curve", leaf_sheath_width_growth_curve);
  LoadGrowthCurveWithLegacyFallback(in, "leaf_neck_width_growth_curve", leaf_neck_width_growth_curve);
  LoadGrowthCurveWithLegacyFallback(in, "leaf_width_growth_curve", leaf_width_growth_curve);
  LoadGrowthCurveWithLegacyFallback(in, "leaf_angle_development_curve", leaf_angle_development_curve);
  LoadGrowthCurveWithLegacyFallback(in, "leaf_curling_development_curve", leaf_curling_development_curve);
  LoadGrowthCurveWithLegacyFallback(in, "leaf_bending_development_curve", leaf_bending_development_curve);
  LoadGrowthCurveWithLegacyFallback(in, "width_along_sheath", width_along_sheath);
  LoadGrowthCurveWithLegacyFallback(in, "width_along_neck", width_along_neck);
  LoadGrowthCurveWithLegacyFallback(in, "width_along_leaf", width_along_leaf);
  if (in["bending_along_leaf"]) {
    LoadGrowthCurveWithLegacyFallback(in, "bending_along_leaf", bending_along_leaf);
  } else {
    bending_along_leaf = leaf_bending_development_curve;
  }
  LoadGrowthCurveWithLegacyFallback(in, "curling_along_leaf", curling_along_leaf);
  LoadGrowthCurveWithLegacyFallback(in, "waviness_along_leaf", waviness_along_leaf);

  leaf_atlas_albedo_texture.Load("leaf_atlas_albedo_texture", in);
  leaf_atlas_normal_texture.Load("leaf_atlas_normal_texture", in);
  leaf_atlas_roughness_texture.Load("leaf_atlas_roughness_texture", in);
  leaf_atlas_metallic_texture.Load("leaf_atlas_metallic_texture", in);
  leaf_atlas_ao_texture.Load("leaf_atlas_ao_texture", in);
  if (in["leaf_atlas_variant_columns"]) {
    leaf_atlas_variant_columns = std::max(1u, in["leaf_atlas_variant_columns"].as<uint32_t>());
  }
  if (in["leaf_atlas_variant_rows"]) {
    leaf_atlas_variant_rows = std::max(1u, in["leaf_atlas_variant_rows"].as<uint32_t>());
  }
  if (in["leaf_atlas_variant_count"]) {
    leaf_atlas_variant_count = std::max(1u, in["leaf_atlas_variant_count"].as<uint32_t>());
  }
  leaf_atlas_variant_count = std::clamp(
      leaf_atlas_variant_count, 1u, std::max(1u, leaf_atlas_variant_columns) * std::max(1u, leaf_atlas_variant_rows));
  if (in["leaf_atlas_tile_uv_inset"]) {
    leaf_atlas_tile_uv_inset = std::max(0.0f, in["leaf_atlas_tile_uv_inset"].as<float>());
  }
  if (in["leaf_atlas_distal_region_uses_top_half"]) {
    leaf_atlas_distal_region_uses_top_half = in["leaf_atlas_distal_region_uses_top_half"].as<bool>();
  }
  if (in["leaf_atlas_semantic_quadrants"]) {
    leaf_atlas_semantic_quadrants = in["leaf_atlas_semantic_quadrants"].as<bool>();
  }
  if (in["leaf_material_albedo_color"]) {
    leaf_material_albedo_color = in["leaf_material_albedo_color"].as<glm::vec3>();
  }
  if (in["leaf_material_roughness"])
    leaf_material_roughness = in["leaf_material_roughness"].as<float>();
  if (in["leaf_material_metallic"])
    leaf_material_metallic = in["leaf_material_metallic"].as<float>();
  if (in["leaf_material_specular"])
    leaf_material_specular = in["leaf_material_specular"].as<float>();
  if (in["leaf_material_subsurface_factor"])
    leaf_material_subsurface_factor = in["leaf_material_subsurface_factor"].as<float>();
  if (in["leaf_material_subsurface_color"])
    leaf_material_subsurface_color = in["leaf_material_subsurface_color"].as<glm::vec3>();
  if (in["leaf_material_subsurface_radius"])
    leaf_material_subsurface_radius = in["leaf_material_subsurface_radius"].as<glm::vec3>();

  stem_albedo_texture.Load("stem_albedo_texture", in);
  stem_normal_texture.Load("stem_normal_texture", in);
  stem_roughness_texture.Load("stem_roughness_texture", in);
  stem_metallic_texture.Load("stem_metallic_texture", in);
  stem_ao_texture.Load("stem_ao_texture", in);
  if (in["stem_material_albedo_color"]) {
    stem_material_albedo_color = in["stem_material_albedo_color"].as<glm::vec3>();
  }
  if (in["stem_material_roughness"])
    stem_material_roughness = in["stem_material_roughness"].as<float>();
  if (in["stem_material_metallic"])
    stem_material_metallic = in["stem_material_metallic"].as<float>();
  if (in["stem_material_specular"])
    stem_material_specular = in["stem_material_specular"].as<float>();
  if (in["panicle_immature_color"])
    panicle_immature_color = in["panicle_immature_color"].as<glm::vec3>();
  if (in["panicle_mature_color"])
    panicle_mature_color = in["panicle_mature_color"].as<glm::vec3>();
  if (in["panicle_material_roughness"])
    panicle_material_roughness = in["panicle_material_roughness"].as<float>();
  if (in["culm_radial_segments"])
    culm_radial_segments = in["culm_radial_segments"].as<uint32_t>();
  if (in["culm_node_radius_scale"])
    culm_node_radius_scale = in["culm_node_radius_scale"].as<float>();
  if (in["culm_texture_repeat_m"])
    culm_texture_repeat_m = in["culm_texture_repeat_m"].as<float>();

  if (in["live_preview"]) {
    live_preview = in["live_preview"].as<bool>();
  }
  if (in["live_preview_representative_only"]) {
    live_preview_representative_only = in["live_preview_representative_only"].as<bool>();
  }
  if (in["live_preview_cap_target_gdd"]) {
    live_preview_cap_target_gdd = in["live_preview_cap_target_gdd"].as<bool>();
  }
  if (in["grid_rows"]) {
    grid_rows = in["grid_rows"].as<int>();
  }
  if (in["grid_cols"]) {
    grid_cols = in["grid_cols"].as<int>();
  }
  if (in["grid_spacing"]) {
    grid_spacing = in["grid_spacing"].as<float>();
  }

  if (in["explorer_mode"]) {
    const int mode = in["explorer_mode"].as<int>();
    if (mode >= 0 && mode <= 3) {
      explorer_.mode = static_cast<ParamMotionMode>(mode);
    }
  }
  if (in["explorer_speed"]) {
    explorer_.speed = std::clamp(in["explorer_speed"].as<float>(), 0.01f, 10.0f);
  }

  // Tropism array.
  tropisms.clear();
  if (in["tropism_count"]) {
    const int count = std::max(0, in["tropism_count"].as<int>());
    tropisms.reserve(count);
    for (int i = 0; i < count; ++i) {
      const std::string prefix = "tropism_" + std::to_string(i) + "_";
      TropismEntry e;
      e.direction_x.Load(prefix + "dir_x", in);
      e.direction_y.Load(prefix + "dir_y", in);
      e.direction_z.Load(prefix + "dir_z", in);
      e.strength.Load(prefix + "strength", in);
      if (in[prefix + "usage_chance_percent"]) {
        e.usage_chance_percent = std::clamp(in[prefix + "usage_chance_percent"].as<float>(), 0.0f, 100.0f);
      }
      e.order_response.Load(prefix + "order_response", in);
      tropisms.push_back(std::move(e));
    }
  }

  ClampDescriptorValues(target);
}

void SorghumLSDescriptor::RegisterExplorableAxes(ParamSpaceExplorer& explorer) {
  auto& d = *this;

  explorer.AddSingle("total_phytomer_count", "TPC", d.total_phytomer_count, 1.0f, 128.0f, 64.0f);
  explorer.AddSingle("phyllotaxis_angle", "PHA", d.phyllotaxis_angle, 0.0f, 360.0f, 180.0f);
  explorer.AddSingle("branch_azimuth_offset", "BAO", d.branch_azimuth_offset, -180.0f, 180.0f, 180.0f);
  explorer.AddSingle("main_culm_lean_angle", "MCL", d.main_culm_lean_angle, 0.0f, 30.0f, 20.0f);

  explorer.AddPlotted("internode_length", "INL", d.internode_length);
  explorer.AddPlotted("internode_thickness", "INT", d.internode_thickness);

  explorer.AddPlotted("leaf_blade_length", "LBL", d.leaf_blade_length);
  explorer.AddPlotted("leaf_sheath_length", "LSH", d.leaf_sheath_length);
  explorer.AddPlotted("leaf_neck_length", "LNK", d.leaf_neck_length);
  explorer.AddPlotted("leaf_sheath_end_width_ratio", "SWR", d.leaf_sheath_end_width_ratio);
  explorer.AddPlotted("leaf_neck_end_width_ratio", "NWR", d.leaf_neck_end_width_ratio);
  explorer.AddPlotted("leaf_blade_end_width_ratio", "BWR", d.leaf_blade_end_width_ratio);
  explorer.AddPlotted("leaf_blade_max_width", "LBW", d.leaf_blade_max_width);
  explorer.AddPlotted("leaf_insertion_angle", "LIA", d.leaf_insertion_angle);
  explorer.AddPlotted("leaf_roll_angle", "LRA", d.leaf_roll_angle);
  explorer.AddPlotted("leaf_curling", "LCR", d.leaf_curling);
  explorer.AddPlotted("leaf_bending", "LBN", d.leaf_bending);
  explorer.AddPlotted("leaf_waviness", "LWA", d.leaf_waviness);
  explorer.AddSingle("leaf_waviness_frequency", "LWF", d.leaf_waviness_frequency, 0.0f, 100.0f, 50.0f);
  explorer.AddPlotted("leaf_waviness_width_fraction", "LWR", d.leaf_waviness_width_fraction);
  explorer.AddSingle("leaf_waviness_wavelength_m", "LWW", d.leaf_waviness_wavelength_m, 0.0f, 5.0f, 2.5f);
  explorer.AddSingle("leaf_centerline_waviness_fraction", "LCW", d.leaf_centerline_waviness_fraction, 0.0f, 0.1f,
                     0.05f);
  explorer.AddSingle("leaf_static_wind_deflection_fraction", "LWD", d.leaf_static_wind_deflection_fraction, 0.0f, 0.2f,
                     0.1f);
  explorer.AddSingle("leaf_axial_twist_max_degrees", "LTA", d.leaf_axial_twist_max_degrees, 0.0f, 45.0f, 30.0f);
  explorer.AddSingle("leaf_axial_twist_frequency_ratio_min", "LTF0", d.leaf_axial_twist_frequency_ratio_min, 0.0f, 0.5f,
                     0.5f);
  explorer.AddSingle("leaf_axial_twist_frequency_ratio_max", "LTF1", d.leaf_axial_twist_frequency_ratio_max, 0.0f, 0.5f,
                     0.5f);
  explorer.AddSingle("leaf_gravity_droop_compliance", "LGC", d.leaf_gravity_droop_compliance, 0.0f, 5.0f, 2.5f);
  explorer.AddPlotted("leaf_gravity_droop_age_response", "LGA", d.leaf_gravity_droop_age_response);
  explorer.AddPlotted("leaf_flexural_stiffness_along_leaf", "LFS", d.leaf_flexural_stiffness_along_leaf);
  explorer.AddSingle("leaf_damage_severity", "LDS", d.leaf_damage_severity, 0.0f, 1.0f, 0.5f);
  explorer.AddSingle("leaf_sheath_cross_section_ratio", "LSC", d.leaf_sheath_cross_section_ratio, 1.0f, 3.0f, 1.0f);

  explorer.AddSingle("leaf_lifespan_years", "LLY", d.leaf_lifespan_years, 0.1f, 8.0f, 4.0f);
  explorer.AddSingle("leaf_wilting_years", "LWY", d.leaf_wilting_years, 0.05f, 8.0f, 4.0f);

  explorer.AddSingle("tiller_count", "TCT", d.tiller_count, 0.0f, 20.0f, 10.0f);
  explorer.AddSingle("tiller_insertion_angle", "TIA", d.tiller_insertion_angle, 0.0f, 120.0f, 60.0f);
  explorer.AddSingle("tiller_final_lean_angle", "TFL", d.tiller_final_lean_angle, -30.0f, 30.0f, 30.0f);
  explorer.AddSingle("tiller_azimuth_jitter", "TAJ", d.tiller_azimuth_jitter, -45.0f, 45.0f, 45.0f);
  explorer.AddSingle("tiller_same_side_splay_angle", "TSA", d.tiller_same_side_splay_angle, 0.0f, 45.0f, 20.0f);
  explorer.AddSingle("tiller_leaf_count_ratio", "TLR", d.tiller_leaf_count_ratio, 0.5f, 1.1f, 0.3f);
  explorer.AddSingle("tiller_height_ratio", "THR", d.tiller_height_ratio, 0.5f, 1.1f, 0.3f);
  explorer.AddPlotted("tiller_leaf_area_ratio_by_origin", "TAR", d.tiller_leaf_area_ratio_by_origin);
  explorer.AddSingle("tiller_thickness_ratio", "TTR", d.tiller_thickness_ratio, 0.05f, 1.2f, 0.5f);
  explorer.AddSingle("tiller_max_axis_length_ratio", "TML", d.tiller_max_axis_length_ratio, 0.1f, 1.5f, 0.7f);

  explorer.AddSingle("target_gdd", "TGD", d.target_gdd, 0.0f, 5000.0f, 5000.0f);
  explorer.AddSingle("gdd_per_day", "GPD", d.gdd_per_day, 0.0f, 500.0f, 500.0f);
  explorer.AddSingle("plastochron_gdd", "PGD", d.plastochron_gdd, 1.0f, 400.0f, 400.0f);
  explorer.AddSingle("maturity_gdd", "MGD", d.maturity_gdd, 1.0f, 3000.0f, 3000.0f);
  explorer.AddSingle("main_axis_plastochron_scale", "MAP", d.main_axis_plastochron_scale, 0.1f, 5.0f, 5.0f);
  explorer.AddSingle("lateral_axis_plastochron_scale", "LAP", d.lateral_axis_plastochron_scale, 0.1f, 5.0f, 5.0f);
  explorer.AddSingle("lateral_bud_plastochron_scale", "LBP", d.lateral_bud_plastochron_scale, 0.1f, 5.0f, 5.0f);
  explorer.AddSingle("maturity_initiation_coupling", "MIC", d.maturity_initiation_coupling, 0.0f, 2.0f, 2.0f);
  explorer.AddSingle("reference_maturity_gdd", "RMG", d.reference_maturity_gdd, 1.0f, 5000.0f, 5000.0f);

  explorer.AddPlotted("internode_elongation_curve", "IEC", d.internode_elongation_curve);
  explorer.AddPlotted("internode_thickness_curve", "ITC", d.internode_thickness_curve);
  explorer.AddPlotted("leaf_sheath_length_growth_curve", "SLG", d.leaf_sheath_length_growth_curve);
  explorer.AddPlotted("leaf_neck_length_growth_curve", "NLG", d.leaf_neck_length_growth_curve);
  explorer.AddPlotted("leaf_blade_growth_curve", "LBG", d.leaf_blade_growth_curve);
  explorer.AddPlotted("leaf_sheath_width_growth_curve", "SWG", d.leaf_sheath_width_growth_curve);
  explorer.AddPlotted("leaf_neck_width_growth_curve", "NWG", d.leaf_neck_width_growth_curve);
  explorer.AddPlotted("leaf_width_growth_curve", "LWG", d.leaf_width_growth_curve);
  explorer.AddPlotted("leaf_angle_development_curve", "LAD", d.leaf_angle_development_curve);
  explorer.AddPlotted("leaf_curling_development_curve", "LCD", d.leaf_curling_development_curve);
  explorer.AddPlotted("leaf_bending_development_curve", "LBE", d.leaf_bending_development_curve);
  explorer.AddPlotted("width_along_sheath", "WAS", d.width_along_sheath);
  explorer.AddPlotted("width_along_neck", "WAN", d.width_along_neck);
  explorer.AddPlotted("width_along_leaf", "WAL", d.width_along_leaf);
  explorer.AddPlotted("bending_along_leaf", "BAL", d.bending_along_leaf);
  explorer.AddPlotted("curling_along_leaf", "CAL", d.curling_along_leaf);
  explorer.AddPlotted("waviness_along_leaf", "WVL", d.waviness_along_leaf);

  for (size_t i = 0; i < d.tropisms.size(); i++) {
    auto& tropism = d.tropisms[i];
    const std::string p = "tropism[" + std::to_string(i) + "]";
    const std::string s = "T" + std::to_string(i);

    explorer.AddSingle(p + ".direction_x", s + "X", tropism.direction_x, -1.0f, 1.0f, 1.0f);
    explorer.AddSingle(p + ".direction_y", s + "Y", tropism.direction_y, -1.0f, 1.0f, 1.0f);
    explorer.AddSingle(p + ".direction_z", s + "Z", tropism.direction_z, -1.0f, 1.0f, 1.0f);
    explorer.AddSingle(p + ".strength", s + "S", tropism.strength, -5.0f, 5.0f, 5.0f);

    auto* tropism_ptr = &d.tropisms[i];
    explorer.AddAxis(
        p + ".usage_chance_percent", s + "U", 0.0f, 100.0f,
        [tropism_ptr]() {
          return tropism_ptr->usage_chance_percent;
        },
        [tropism_ptr](float v) {
          tropism_ptr->usage_chance_percent = std::clamp(v, 0.0f, 100.0f);
        });

    explorer.AddPlotted(p + ".order_response", s + "O", tropism.order_response);
  }
}
