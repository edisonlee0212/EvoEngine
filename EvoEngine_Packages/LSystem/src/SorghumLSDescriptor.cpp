#include "SorghumLSDescriptor.hpp"
#include "DistributionDefaults.hpp"
#include "LSystemDescriptorDefaults.hpp"
#include "LSystemInspectionAdapters.hpp"
#include "LSystemSerializationAdapters.hpp"
#include "SorghumLS.hpp"
#include <Application.hpp>
#include <EditorLayer.hpp>
#include <Scene.hpp>
#include <Texture2D.hpp>
#include <Transform.hpp>
#include <array>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <yaml-cpp/yaml.h>

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
    std::filesystem::path("LSystem") / "New SorghumLSDescriptor.sorghumls",
    "New SorghumLSDescriptor.sorghumls"};

const std::array<std::filesystem::path, 2> kSorghumWritableTemplateCandidates = {
    std::filesystem::path("./Resources/DigitalAgricultureProject/Assets/") /
        "New SorghumLSDescriptor.sorghumls",
    std::filesystem::path("./04_EvoEngine/Resources/DigitalAgricultureProject/Assets/") /
        "New SorghumLSDescriptor.sorghumls"};

const std::filesystem::path kSorghumFallbackDefaultsPath =
    std::filesystem::path("./LSystemResources/Defaults/SorghumLSDescriptor_Default.sorghumls");

double GetSteadyTimeSeconds() {
  return std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}

std::filesystem::path ResolveDefaultSorghumLSDescriptorPath() {
  return descriptor_defaults::ResolveExistingDefaultsPath(
      kSorghumResourceCandidates,
      kSorghumProjectAssetCandidates);
}

std::filesystem::path ResolveWritableSorghumLSDescriptorDefaultsPath() {
  return descriptor_defaults::ResolveWritableDefaultsPath(
      kSorghumResourceCandidates,
      kSorghumProjectAssetCandidates,
      kSorghumWritableTemplateCandidates,
      kSorghumFallbackDefaultsPath);
}

bool LoadSorghumLSDescriptorDefaultsFromFile(SorghumLSDescriptor& descriptor,
                                             const std::filesystem::path& file_path) {
  YAML::Node defaults;
  if (!descriptor_defaults::LoadDefaultsYamlMap(
          file_path,
          defaults,
          kSorghumDescriptorName)) {
    return false;
  }
  DeserializeSorghumLSDescriptor(defaults, descriptor);
  return true;
}

void LoadSingleDistributionWithScalarFallback(const YAML::Node& in,
                                              const char* key,
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

void LoadGrowthCurveWithLegacyFallback(const YAML::Node& in,
                                       const char* key,
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

float SampleTargetGddForSeed(const evo_engine::SingleDistribution<float>& distribution,
                             const uint32_t seed) {
  std::mt19937 rng(seed);
  return std::max(0.0f, SampleDistribution(distribution, rng));
}

bool IsLikelyLegacyNormalizedLeafCurling(
    const evo_engine::PlottedDistribution<float>& distribution) {
  const float mean_min = distribution.mean.min_value;
  const float mean_max = distribution.mean.max_value;
  const float dev_min = distribution.deviation.min_value;
  const float dev_max = distribution.deviation.max_value;

  if (!std::isfinite(mean_min) || !std::isfinite(mean_max) ||
      !std::isfinite(dev_min) || !std::isfinite(dev_max)) {
    return false;
  }

  return mean_min >= -1.0e-3f &&
         mean_max <= kLeafCurlingLegacyNormalizedThreshold &&
         dev_min >= -1.0e-3f &&
         dev_max <= kLeafCurlingLegacyNormalizedThreshold;
}

void ScalePlottedDistributionRange(evo_engine::PlottedDistribution<float>& distribution,
                                   const float scale) {
  distribution.mean.min_value *= scale;
  distribution.mean.max_value *= scale;
  distribution.deviation.min_value *= scale;
  distribution.deviation.max_value *= scale;
}

void SetConstantPlottedDistribution(evo_engine::PlottedDistribution<float>& distribution,
                                    const float value) {
  distribution.mean.min_value = value;
  distribution.mean.max_value = value;
  distribution.deviation.min_value = 0.0f;
  distribution.deviation.max_value = 0.0f;
}

// Helper: convert a TropismEntry array into the runtime SampledTropism vector,
// using rng for direction/strength sampling and the entry's usage_chance_percent
// gate. Mirrors MaizeTasselDescriptor::Sample tropism handling.
void SampleTropisms(const std::vector<TropismEntry>& entries,
                    std::mt19937& rng,
                    std::vector<SampledTropism>& out) {
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

void ClampSingleDistribution(evo_engine::SingleDistribution<float>& distribution,
                             const float mean_min,
                             const float mean_max,
                             const float deviation_max = std::numeric_limits<float>::max()) {
  distribution.mean = std::clamp(distribution.mean, mean_min, mean_max);
  distribution.deviation = std::clamp(distribution.deviation, 0.0f, deviation_max);
}

void ClampPlottedDistributionRange(evo_engine::PlottedDistribution<float>& distribution,
                                   const float mean_min,
                                   const float mean_max,
                                   const float deviation_max) {
  distribution.mean.min_value = std::clamp(distribution.mean.min_value, mean_min, mean_max);
  distribution.mean.max_value = std::clamp(distribution.mean.max_value, mean_min, mean_max);
  if (distribution.mean.max_value < distribution.mean.min_value) {
    std::swap(distribution.mean.min_value, distribution.mean.max_value);
  }

  distribution.deviation.min_value =
      std::clamp(distribution.deviation.min_value, 0.0f, deviation_max);
  distribution.deviation.max_value =
      std::clamp(distribution.deviation.max_value, 0.0f, deviation_max);
  if (distribution.deviation.max_value < distribution.deviation.min_value) {
    std::swap(distribution.deviation.min_value, distribution.deviation.max_value);
  }
}

void ClampDescriptorValues(SorghumLSDescriptor& descriptor) {
  ClampSingleDistribution(descriptor.total_phytomer_count, 1.0f, 128.0f, 64.0f);
  ClampSingleDistribution(descriptor.phyllotaxis_angle, 0.0f, 360.0f, 180.0f);
  ClampSingleDistribution(descriptor.branch_azimuth_offset, -180.0f, 180.0f, 180.0f);

  ClampPlottedDistributionRange(descriptor.internode_length, 0.0f, 1.0f, 0.5f);
  ClampPlottedDistributionRange(descriptor.internode_thickness, 0.0f, 0.1f, 0.05f);

  ClampPlottedDistributionRange(descriptor.leaf_blade_length, 0.0f, 1.5f, 0.75f);
  ClampPlottedDistributionRange(descriptor.leaf_blade_max_width, 0.0f, 0.2f, 0.1f);
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
  ClampSingleDistribution(descriptor.leaf_waviness_frequency, 0.0f, 100.0f, 50.0f);
  ClampSingleDistribution(descriptor.leaf_sheath_radius_ratio, 1.0f, 3.0f, 1.0f);
  ClampSingleDistribution(descriptor.leaf_blade_stage1_length_ratio, 0.0f, 1.0f, 1.0f);
  ClampSingleDistribution(descriptor.leaf_blade_stage2_length_ratio, 0.0f, 1.0f, 1.0f);
  ClampSingleDistribution(descriptor.leaf_blade_stage3_length_ratio, 0.0f, 1.0f, 1.0f);
  ClampSingleDistribution(descriptor.leaf_blade_stage1_width_scale, 0.05f, 2.0f, 1.0f);
  ClampSingleDistribution(descriptor.leaf_blade_stage2_width_scale, 0.05f, 2.0f, 1.0f);
  ClampSingleDistribution(descriptor.leaf_blade_stage3_width_scale, 0.05f, 2.0f, 1.0f);

  ClampSingleDistribution(descriptor.leaf_lifespan_years, 0.1f, 8.0f, 4.0f);
  ClampSingleDistribution(descriptor.leaf_wilting_years, 0.05f, 8.0f, 4.0f);
  descriptor.leaf_wilting_years.mean = std::clamp(
      descriptor.leaf_wilting_years.mean,
      0.05f,
      std::max(0.05f, descriptor.leaf_lifespan_years.mean));

  ClampSingleDistribution(descriptor.tiller_count, 0.0f, 20.0f, 10.0f);
  ClampPlottedDistributionRange(descriptor.tiller_initiation_delay_gdd, 0.0f, 2000.0f, 1000.0f);
  ClampSingleDistribution(descriptor.tiller_insertion_angle, -30.0f, 120.0f, 60.0f);
  ClampSingleDistribution(descriptor.tiller_phytomer_count_scale, 0.05f, 2.0f, 1.0f);
  ClampSingleDistribution(descriptor.tiller_thickness_ratio, 0.05f, 1.0f, 0.5f);

  ClampSingleDistribution(descriptor.target_gdd, 0.0f, 5000.0f, 5000.0f);
  ClampSingleDistribution(descriptor.gdd_per_day, 0.0f, 500.0f, 500.0f);
  ClampSingleDistribution(descriptor.plastochron_gdd, 1.0f, 400.0f, 400.0f);
  ClampSingleDistribution(descriptor.maturity_gdd, 1.0f, 3000.0f, 3000.0f);
  descriptor.maturity_gdd.mean =
      std::max(descriptor.maturity_gdd.mean, descriptor.plastochron_gdd.mean);
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
}

}  // namespace

SorghumLSDescriptor::SorghumLSDescriptor() {
  ApplyMeanStdPlotDefaultsToAll(
      internode_length,
      internode_thickness,
      leaf_blade_length,
      leaf_blade_max_width,
      leaf_sheath_length,
      leaf_neck_length,
      leaf_sheath_end_width_ratio,
      leaf_neck_end_width_ratio,
      leaf_blade_end_width_ratio,
      leaf_insertion_angle,
      leaf_roll_angle,
      leaf_curling,
      leaf_bending,
      leaf_waviness,
      tiller_initiation_delay_gdd);

    SetConstantPlottedDistribution(leaf_sheath_end_width_ratio, 1.35f);
    SetConstantPlottedDistribution(leaf_neck_end_width_ratio, 1.7f);
    SetConstantPlottedDistribution(leaf_blade_end_width_ratio, 0.35f);

  ApplyLinearGrowthCurveDefaultsToAll(
      internode_elongation_curve,
      internode_thickness_curve,
      leaf_sheath_length_growth_curve,
      leaf_neck_length_growth_curve,
      leaf_blade_growth_curve,
      leaf_sheath_width_growth_curve,
      leaf_neck_width_growth_curve,
      leaf_width_growth_curve,
      leaf_angle_development_curve,
      leaf_curling_development_curve,
      leaf_bending_development_curve,
      width_along_sheath,
      width_along_neck,
      width_along_leaf,
      curling_along_leaf,
      waviness_along_leaf);

  const auto defaults_path = ResolveDefaultSorghumLSDescriptorPath();
  if (!LoadSorghumLSDescriptorDefaultsFromFile(*this, defaults_path)) {
    static bool warned_once = false;
    if (!warned_once) {
      warned_once = true;
      EVOENGINE_WARNING(
          "SorghumLSDescriptor defaults file not found or invalid. Using inline member defaults.");
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
  p.total_phytomer_count = std::max(
      1, static_cast<int>(std::round(SampleDistribution(total_phytomer_count, rng))));
  p.phyllotaxis_angle = SampleDistribution(phyllotaxis_angle, rng);
  p.branch_azimuth_offset = SampleDistribution(branch_azimuth_offset, rng);

  // Internode + leaf morphology distributions are forwarded by value; rule
  // lambdas re-evaluate them per emission to keep deterministic sampling
  // (same idiom as MaizeTasselDescriptor::Sample).
  p.internode_length = internode_length;
  p.internode_thickness = internode_thickness;

  p.leaf_blade_length = leaf_blade_length;
  p.leaf_blade_max_width = leaf_blade_max_width;
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
  p.leaf_sheath_radius_ratio = std::clamp(
      SampleDistribution(leaf_sheath_radius_ratio, rng), 1.0f, 3.0f);
  p.leaf_blade_stage1_length_ratio = std::clamp(
      SampleDistribution(leaf_blade_stage1_length_ratio, rng), 0.0f, 1.0f);
  p.leaf_blade_stage2_length_ratio = std::clamp(
      SampleDistribution(leaf_blade_stage2_length_ratio, rng), 0.0f, 1.0f);
  p.leaf_blade_stage3_length_ratio = std::clamp(
      SampleDistribution(leaf_blade_stage3_length_ratio, rng), 0.0f, 1.0f);
  p.leaf_blade_stage1_width_scale = std::clamp(
      SampleDistribution(leaf_blade_stage1_width_scale, rng), 0.05f, 2.0f);
  p.leaf_blade_stage2_width_scale = std::clamp(
      SampleDistribution(leaf_blade_stage2_width_scale, rng), 0.05f, 2.0f);
  p.leaf_blade_stage3_width_scale = std::clamp(
      SampleDistribution(leaf_blade_stage3_width_scale, rng), 0.05f, 2.0f);

  // Leaf lifecycle (chronological).
  p.leaf_lifespan_years = leaf_lifespan_years;
  p.leaf_lifespan_years.mean = std::max(0.1f, p.leaf_lifespan_years.mean);
  p.leaf_lifespan_years.deviation = std::max(0.0f, p.leaf_lifespan_years.deviation);
  p.leaf_wilting_years = leaf_wilting_years;
  p.leaf_wilting_years.mean = std::max(0.05f, p.leaf_wilting_years.mean);
  p.leaf_wilting_years.deviation = std::max(0.0f, p.leaf_wilting_years.deviation);

  // Tillering.
  p.tiller_count = std::max(0, static_cast<int>(std::round(SampleDistribution(tiller_count, rng))));
  p.tiller_initiation_delay_gdd = tiller_initiation_delay_gdd;
  p.tiller_insertion_angle = SampleDistribution(tiller_insertion_angle, rng);
  p.tiller_phytomer_count_scale = std::clamp(
      SampleDistribution(tiller_phytomer_count_scale, rng), 0.05f, 2.0f);
  p.tiller_thickness_ratio = std::clamp(
      SampleDistribution(tiller_thickness_ratio, rng), 0.05f, 1.0f);

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
    const double avg_apply_ms = live_preview_total_apply_ms_ /
                                static_cast<double>(live_preview_apply_count_);
    ImGui::Text("Preview last/avg ms: %.3f / %.3f",
                live_preview_last_apply_ms_,
                avg_apply_ms);
  }
  ImGui::Text("Preview requests/applied/coalesced: %u / %u / %u",
              live_preview_request_count_,
              live_preview_apply_count_,
              live_preview_coalesced_count_);

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
        const auto base_seed = static_cast<unsigned int>(
            std::chrono::steady_clock::now().time_since_epoch().count() & 0xFFFFFFFFu);
        for (int i = 0; i < grid_rows; i++) {
          for (int j = 0; j < grid_cols; j++) {
            const auto entity = scene->CreateEntity(
                GetTitle() + " [" + std::to_string(i) + "," + std::to_string(j) + "]");
            const auto sorghum = scene->GetOrSetPrivateComponent<SorghumLS>(entity).lock();
            sorghum->descriptor_ref = GetSelf();
            sorghum->seed = base_seed + static_cast<unsigned int>(i * grid_cols + j);
            sorghum->target_gdd = SampleTargetGddForSeed(target_gdd, sorghum->seed);

            scene->SetParent(entity, container, false);

            Transform transform;
            transform.SetPosition(glm::vec3(
                0.0f,
                static_cast<float>(i) * grid_spacing - offset_y,
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
          std::sort(containers.begin(), containers.end(),
                    [](const Entity& a, const Entity& b) { return a.GetIndex() < b.GetIndex(); });
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

  if (ImGui::TreeNodeEx("Leaf Material", ImGuiTreeNodeFlags_DefaultOpen)) {
    changed |= editor_layer->DragAndDropButton<Texture2D>(leaf_atlas_albedo_texture, "Atlas Albedo");
    show_item_hover_description("Sorghum atlas albedo. Leaf blades use V 0.5..1.0; sheath and neck use V 0.0..0.5.");
    changed |= editor_layer->DragAndDropButton<Texture2D>(leaf_atlas_normal_texture, "Atlas Normal");
    changed |= editor_layer->DragAndDropButton<Texture2D>(leaf_atlas_roughness_texture, "Atlas Roughness");
    changed |= editor_layer->DragAndDropButton<Texture2D>(leaf_atlas_metallic_texture, "Atlas Metallic");
    changed |= editor_layer->DragAndDropButton<Texture2D>(leaf_atlas_ao_texture, "Atlas AO");
    int atlas_columns = static_cast<int>(leaf_atlas_variant_columns);
    if (ImGui::DragInt("Variant Columns", &atlas_columns, 1, 1, 64)) {
      leaf_atlas_variant_columns = static_cast<uint32_t>(std::max(1, atlas_columns));
      leaf_atlas_variant_count = std::clamp(
          leaf_atlas_variant_count,
          1u,
          leaf_atlas_variant_columns * std::max(1u, leaf_atlas_variant_rows));
      changed = true;
    }
    int atlas_rows = static_cast<int>(leaf_atlas_variant_rows);
    if (ImGui::DragInt("Variant Rows", &atlas_rows, 1, 1, 64)) {
      leaf_atlas_variant_rows = static_cast<uint32_t>(std::max(1, atlas_rows));
      leaf_atlas_variant_count = std::clamp(
          leaf_atlas_variant_count,
          1u,
          std::max(1u, leaf_atlas_variant_columns) * leaf_atlas_variant_rows);
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
    ImGui::TreePop();
  }

  ImGui::Separator();

  if (ImGui::TreeNodeEx("Basic Controls", ImGuiTreeNodeFlags_DefaultOpen)) {
    changed |= total_phytomer_count.Draw("Phytomer Count", 0.5f);
    changed |= inspect_plotted_distribution("Internode Length", internode_length);
    changed |= inspect_plotted_distribution("Internode Thickness", internode_thickness);
    changed |= inspect_plotted_distribution("Sheath Length", leaf_sheath_length);
    changed |= inspect_plotted_distribution("Neck Length", leaf_neck_length);
    changed |= inspect_plotted_distribution("Blade Length", leaf_blade_length);
    changed |= inspect_plotted_distribution("Sheath End Width Ratio", leaf_sheath_end_width_ratio);
    changed |= inspect_plotted_distribution("Neck End Width Ratio", leaf_neck_end_width_ratio);
    changed |= inspect_plotted_distribution("Blade End Width Ratio", leaf_blade_end_width_ratio);
    ImGui::TextDisabled(
        "Continuity: sheath u=0 matches internode width; neck/blade u=0 inherit previous stage width.");
    changed |= inspect_plotted_distribution("Blade Max Width (Legacy, Deprecated)", leaf_blade_max_width);
    changed |= inspect_plotted_distribution("Leaf Insertion Angle", leaf_insertion_angle);
    changed |= inspect_plotted_distribution("Leaf Curling / Opening (deg)", leaf_curling);
    changed |= tiller_count.Draw("Tiller Count", 0.5f);
    changed |= target_gdd.Draw("Target GDD", 5.0f);
    changed |= maturity_gdd.Draw("Maturity GDD", 5.0f);
    ImGui::TreePop();
  }

  ImGui::Separator();

  if (ImGui::TreeNodeEx("Culm Topology", ImGuiTreeNodeFlags_DefaultOpen)) {
    changed |= total_phytomer_count.Draw("Total Phytomer Count", 0.5f);
    changed |= phyllotaxis_angle.Draw("Phyllotaxis Angle (deg)", 1.0f);
    changed |= branch_azimuth_offset.Draw("Branch Azimuth Offset (deg)", 0.5f);
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Internode Morphology", ImGuiTreeNodeFlags_DefaultOpen)) {
    changed |= inspect_plotted_distribution("Internode Length (Rank)", internode_length);
    changed |= inspect_plotted_distribution("Internode Thickness (Rank)", internode_thickness);
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Leaf Morphology", ImGuiTreeNodeFlags_DefaultOpen)) {
    changed |= inspect_plotted_distribution("Sheath Length (Rank)", leaf_sheath_length);
    changed |= inspect_plotted_distribution("Neck Length (Rank)", leaf_neck_length);
    changed |= inspect_plotted_distribution("Blade Length (Rank)", leaf_blade_length);
    changed |= inspect_plotted_distribution("Sheath End Width Ratio (Rank)", leaf_sheath_end_width_ratio);
    changed |= inspect_plotted_distribution("Neck End Width Ratio (Rank)", leaf_neck_end_width_ratio);
    changed |= inspect_plotted_distribution("Blade End Width Ratio (Rank)", leaf_blade_end_width_ratio);
    changed |= inspect_plotted_distribution("Blade Max Width (Rank, Legacy, Deprecated)", leaf_blade_max_width);
    changed |= inspect_plotted_distribution("Insertion Angle (Rank)", leaf_insertion_angle);
    changed |= inspect_plotted_distribution("Roll Angle (Rank)", leaf_roll_angle);
    changed |= inspect_plotted_distribution("Leaf Curling / Opening (Rank, deg)", leaf_curling);
    changed |= inspect_plotted_distribution("Leaf Bending (Rank)", leaf_bending);
    changed |= inspect_plotted_distribution("Leaf Waviness (Rank)", leaf_waviness);
    changed |= leaf_waviness_frequency.Draw("Waviness Frequency", 0.1f);
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Leaf Lifecycle", ImGuiTreeNodeFlags_DefaultOpen)) {
    changed |= leaf_lifespan_years.Draw("Leaf Lifespan (years)", 0.05f);
    changed |= leaf_wilting_years.Draw("Leaf Wilting Duration (years)", 0.02f);
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Tillering", ImGuiTreeNodeFlags_DefaultOpen)) {
    changed |= tiller_count.Draw("Tiller Count", 0.5f);
    changed |= inspect_plotted_distribution("Tiller Initiation Delay (GDD)", tiller_initiation_delay_gdd);
    changed |= tiller_insertion_angle.Draw("Tiller Insertion Angle", 0.5f);
    changed |= tiller_phytomer_count_scale.Draw("Tiller Phytomer Count Scale", 0.01f);
    changed |= tiller_thickness_ratio.Draw("Tiller Thickness Ratio", 0.01f);
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Thermal Development", ImGuiTreeNodeFlags_DefaultOpen)) {
    changed |= target_gdd.Draw("Target GDD", 5.0f);
    changed |= gdd_per_day.Draw("Thermal GDD/day", 0.25f);
    changed |= plastochron_gdd.Draw("Plastochron GDD", 1.0f);
    changed |= maturity_gdd.Draw("Maturity GDD", 5.0f);
    changed |= main_axis_plastochron_scale.Draw("Main-Axis Plastochron Scale", 0.01f);
    changed |= lateral_axis_plastochron_scale.Draw("Lateral-Axis Plastochron Scale", 0.01f);
    changed |= lateral_bud_plastochron_scale.Draw("Lateral-Bud Plastochron Scale", 0.01f);
    changed |= maturity_initiation_coupling.Draw("Maturity->Initiation Coupling", 0.01f);
    changed |= reference_maturity_gdd.Draw("Reference Maturity GDD", 5.0f);
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Growth Curves", ImGuiTreeNodeFlags_DefaultOpen)) {
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
        {"Neck Length", &leaf_neck_length_growth_curve,
         "Controls normalized neck elongation over thermal age."},
        {"Blade Length", &leaf_blade_growth_curve,
         "Controls normalized blade elongation over thermal age."},
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
            {"Curling Along Leaf", &curling_along_leaf,
             "Profile multiplier of blade opening/curling from collar (x=0) to tip (x=1)."},
            {"Waviness Along Leaf", &waviness_along_leaf,
             "Profile of waviness amplitude from collar (x=0) to tip (x=1)."},
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
        if (ImGui::DragFloat("Usage Chance (%)",
                             &tropisms[i].usage_chance_percent,
                             0.5f,
                             0.0f,
                             100.0f,
                             "%.1f")) {
          tropisms[i].usage_chance_percent =
              std::clamp(tropisms[i].usage_chance_percent, 0.0f, 100.0f);
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

  if (live_preview && live_preview_dirty_) {
    const double now_seconds = GetSteadyTimeSeconds();
    const double min_interval_seconds = 1.0 / 12.0; // Fixed 12Hz preview rate

    const bool throttle_ready = live_preview_last_apply_seconds_ < 0.0 ||
                                (now_seconds - live_preview_last_apply_seconds_) >= min_interval_seconds;

    if (!drag_active || throttle_ready) {
      const double apply_start_seconds = GetSteadyTimeSeconds();
      bool applied_any = false;
      const auto scene = GetApplication().GetActiveScene();
      if (scene) {
        const auto* sorghum_entities_ptr = scene->UnsafeGetPrivateComponentOwnersList<SorghumLS>();
        if (sorghum_entities_ptr) {
          const std::vector<Entity> sorghum_entities = *sorghum_entities_ptr;
          if (drag_active) {
            bool preview_applied = false;

            for (const auto& entity : sorghum_entities) {
              if (!scene->IsEntityValid(entity))
                continue;
              auto sorghum = scene->GetOrSetPrivateComponent<SorghumLS>(entity).lock();
              if (!sorghum)
                continue;
              if (sorghum->descriptor_ref.Get<SorghumLSDescriptor>().get() != this)
                continue;

              const float sampled_target = SampleTargetGddForSeed(target_gdd, sorghum->seed);
              const float preview_target = live_preview_cap_target_gdd
                  ? std::min(sampled_target, 1000.0f) // default hardcoded cap if enabled
                  : sampled_target;

              sorghum->target_gdd = sampled_target;
                // Hardcoded preview cap to avoid huge drag hitches.
                sorghum->GeneratePreviewGeometryEntities(preview_target, 64u);

              preview_applied = true;
              applied_any = true;

              if (live_preview_representative_only)
                break;
            }

            if (preview_applied) {
              live_preview_needs_full_apply_ = true;
            }
          } else {
            for (const auto& entity : sorghum_entities) {
              if (!scene->IsEntityValid(entity))
                continue;
              auto sorghum = scene->GetOrSetPrivateComponent<SorghumLS>(entity).lock();
              if (!sorghum)
                continue;
              if (sorghum->descriptor_ref.Get<SorghumLSDescriptor>().get() != this)
                continue;

              const float sampled_target = SampleTargetGddForSeed(target_gdd, sorghum->seed);
              sorghum->target_gdd = sampled_target;
              sorghum->GenerateGeometryEntities(true);
              applied_any = true;
            }
            live_preview_needs_full_apply_ = false;
          }
        }
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

  // Internode + leaf morphology.
  target.internode_length.Save("internode_length", out);
  target.internode_thickness.Save("internode_thickness", out);
  target.leaf_blade_length.Save("leaf_blade_length", out);
  target.leaf_blade_max_width.Save("leaf_blade_max_width", out);
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
  target.leaf_sheath_radius_ratio.Save("leaf_sheath_radius_ratio", out);  // [deprecated]
  target.leaf_blade_stage1_length_ratio.Save("leaf_blade_stage1_length_ratio", out);
  target.leaf_blade_stage2_length_ratio.Save("leaf_blade_stage2_length_ratio", out);
  target.leaf_blade_stage3_length_ratio.Save("leaf_blade_stage3_length_ratio", out);
  target.leaf_blade_stage1_width_scale.Save("leaf_blade_stage1_width_scale", out);
  target.leaf_blade_stage2_width_scale.Save("leaf_blade_stage2_width_scale", out);
  target.leaf_blade_stage3_width_scale.Save("leaf_blade_stage3_width_scale", out);

  // Leaf lifecycle.
  target.leaf_lifespan_years.Save("leaf_lifespan_years", out);
  target.leaf_wilting_years.Save("leaf_wilting_years", out);

  // Tillering.
  target.tiller_count.Save("tiller_count", out);
  target.tiller_initiation_delay_gdd.Save("tiller_initiation_delay_gdd", out);
  target.tiller_insertion_angle.Save("tiller_insertion_angle", out);
  target.tiller_phytomer_count_scale.Save("tiller_phytomer_count_scale", out);
  target.tiller_thickness_ratio.Save("tiller_thickness_ratio", out);

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

  out << YAML::Key << "live_preview" << YAML::Value << target.live_preview;
  out << YAML::Key << "live_preview_representative_only" << YAML::Value
      << target.live_preview_representative_only;
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
  auto& internode_length = target.internode_length;
  auto& internode_thickness = target.internode_thickness;
  auto& leaf_blade_length = target.leaf_blade_length;
  auto& leaf_blade_max_width = target.leaf_blade_max_width;
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
  auto& leaf_sheath_radius_ratio = target.leaf_sheath_radius_ratio;
  auto& leaf_blade_stage1_length_ratio = target.leaf_blade_stage1_length_ratio;
  auto& leaf_blade_stage2_length_ratio = target.leaf_blade_stage2_length_ratio;
  auto& leaf_blade_stage3_length_ratio = target.leaf_blade_stage3_length_ratio;
  auto& leaf_blade_stage1_width_scale = target.leaf_blade_stage1_width_scale;
  auto& leaf_blade_stage2_width_scale = target.leaf_blade_stage2_width_scale;
  auto& leaf_blade_stage3_width_scale = target.leaf_blade_stage3_width_scale;
  auto& leaf_lifespan_years = target.leaf_lifespan_years;
  auto& leaf_wilting_years = target.leaf_wilting_years;
  auto& tiller_count = target.tiller_count;
  auto& tiller_initiation_delay_gdd = target.tiller_initiation_delay_gdd;
  auto& tiller_insertion_angle = target.tiller_insertion_angle;
  auto& tiller_phytomer_count_scale = target.tiller_phytomer_count_scale;
  auto& tiller_thickness_ratio = target.tiller_thickness_ratio;
  auto& target_gdd = target.target_gdd;
  auto& gdd_per_day = target.gdd_per_day;
  auto& plastochron_gdd = target.plastochron_gdd;
  auto& maturity_gdd = target.maturity_gdd;
  auto& main_axis_plastochron_scale = target.main_axis_plastochron_scale;
  auto& lateral_axis_plastochron_scale = target.lateral_axis_plastochron_scale;
  auto& lateral_bud_plastochron_scale = target.lateral_bud_plastochron_scale;
  auto& maturity_initiation_coupling = target.maturity_initiation_coupling;
  auto& reference_maturity_gdd = target.reference_maturity_gdd;
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

  internode_length.Load("internode_length", in);
  internode_thickness.Load("internode_thickness", in);
  leaf_blade_length.Load("leaf_blade_length", in);
  leaf_blade_max_width.Load("leaf_blade_max_width", in);
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
  LoadSingleDistributionWithScalarFallback(in, "leaf_sheath_radius_ratio", leaf_sheath_radius_ratio);
  LoadSingleDistributionWithScalarFallback(in, "leaf_blade_stage1_length_ratio", leaf_blade_stage1_length_ratio);
  LoadSingleDistributionWithScalarFallback(in, "leaf_blade_stage2_length_ratio", leaf_blade_stage2_length_ratio);
  LoadSingleDistributionWithScalarFallback(in, "leaf_blade_stage3_length_ratio", leaf_blade_stage3_length_ratio);
  LoadSingleDistributionWithScalarFallback(in, "leaf_blade_stage1_width_scale", leaf_blade_stage1_width_scale);
  LoadSingleDistributionWithScalarFallback(in, "leaf_blade_stage2_width_scale", leaf_blade_stage2_width_scale);
  LoadSingleDistributionWithScalarFallback(in, "leaf_blade_stage3_width_scale", leaf_blade_stage3_width_scale);

  if (!in["leaf_sheath_end_width_ratio"] && in["leaf_blade_stage1_width_scale"]) {
    SetConstantPlottedDistribution(
        leaf_sheath_end_width_ratio,
        std::max(0.05f, leaf_blade_stage1_width_scale.mean));
  }
  if (!in["leaf_neck_end_width_ratio"] && in["leaf_blade_stage2_width_scale"]) {
    SetConstantPlottedDistribution(
        leaf_neck_end_width_ratio,
        std::max(0.05f, leaf_blade_stage2_width_scale.mean));
  }
  if (!in["leaf_blade_end_width_ratio"] && in["leaf_blade_stage3_width_scale"]) {
    SetConstantPlottedDistribution(
        leaf_blade_end_width_ratio,
        std::max(0.05f, leaf_blade_stage3_width_scale.mean));
  }

  LoadSingleDistributionWithScalarFallback(in, "leaf_lifespan_years", leaf_lifespan_years);
  LoadSingleDistributionWithScalarFallback(in, "leaf_wilting_years", leaf_wilting_years);

  LoadSingleDistributionWithScalarFallback(in, "tiller_count", tiller_count);
  tiller_initiation_delay_gdd.Load("tiller_initiation_delay_gdd", in);
  LoadSingleDistributionWithScalarFallback(in, "tiller_insertion_angle", tiller_insertion_angle);
  LoadSingleDistributionWithScalarFallback(in, "tiller_phytomer_count_scale", tiller_phytomer_count_scale);
  LoadSingleDistributionWithScalarFallback(in, "tiller_thickness_ratio", tiller_thickness_ratio);

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
      leaf_atlas_variant_count,
      1u,
      std::max(1u, leaf_atlas_variant_columns) * std::max(1u, leaf_atlas_variant_rows));
  if (in["leaf_atlas_tile_uv_inset"]) {
    leaf_atlas_tile_uv_inset = std::max(0.0f, in["leaf_atlas_tile_uv_inset"].as<float>());
  }
  if (in["leaf_atlas_distal_region_uses_top_half"]) {
    leaf_atlas_distal_region_uses_top_half = in["leaf_atlas_distal_region_uses_top_half"].as<bool>();
  }

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
        e.usage_chance_percent =
            std::clamp(in[prefix + "usage_chance_percent"].as<float>(), 0.0f, 100.0f);
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

  explorer.AddPlotted("internode_length", "INL", d.internode_length);
  explorer.AddPlotted("internode_thickness", "INT", d.internode_thickness);

  explorer.AddPlotted("leaf_blade_length", "LBL", d.leaf_blade_length);
  explorer.AddPlotted("leaf_sheath_length", "LSH", d.leaf_sheath_length);
  explorer.AddPlotted("leaf_neck_length", "LNK", d.leaf_neck_length);
  explorer.AddPlotted("leaf_sheath_end_width_ratio", "SWR", d.leaf_sheath_end_width_ratio);
  explorer.AddPlotted("leaf_neck_end_width_ratio", "NWR", d.leaf_neck_end_width_ratio);
  explorer.AddPlotted("leaf_blade_end_width_ratio", "BWR", d.leaf_blade_end_width_ratio);
  explorer.AddPlotted("leaf_blade_max_width", "LBW", d.leaf_blade_max_width);  // [deprecated]
  explorer.AddPlotted("leaf_insertion_angle", "LIA", d.leaf_insertion_angle);
  explorer.AddPlotted("leaf_roll_angle", "LRA", d.leaf_roll_angle);
  explorer.AddPlotted("leaf_curling", "LCR", d.leaf_curling);
  explorer.AddPlotted("leaf_bending", "LBN", d.leaf_bending);
  explorer.AddPlotted("leaf_waviness", "LWA", d.leaf_waviness);
  explorer.AddSingle("leaf_waviness_frequency", "LWF", d.leaf_waviness_frequency, 0.0f, 100.0f, 50.0f);

  explorer.AddSingle("leaf_lifespan_years", "LLY", d.leaf_lifespan_years, 0.1f, 8.0f, 4.0f);
  explorer.AddSingle("leaf_wilting_years", "LWY", d.leaf_wilting_years, 0.05f, 8.0f, 4.0f);

  explorer.AddSingle("tiller_count", "TCT", d.tiller_count, 0.0f, 20.0f, 10.0f);
  explorer.AddPlotted("tiller_initiation_delay_gdd", "TID", d.tiller_initiation_delay_gdd);
  explorer.AddSingle("tiller_insertion_angle", "TIA", d.tiller_insertion_angle, -30.0f, 120.0f, 60.0f);
  explorer.AddSingle("tiller_phytomer_count_scale", "TPS", d.tiller_phytomer_count_scale, 0.05f, 2.0f, 1.0f);
  explorer.AddSingle("tiller_thickness_ratio", "TTR", d.tiller_thickness_ratio, 0.05f, 1.0f, 0.5f);

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
    explorer.AddAxis(p + ".usage_chance_percent", s + "U", 0.0f, 100.0f,
                     [tropism_ptr]() { return tropism_ptr->usage_chance_percent; },
                     [tropism_ptr](float v) {
                       tropism_ptr->usage_chance_percent = std::clamp(v, 0.0f, 100.0f);
                     });

    explorer.AddPlotted(p + ".order_response", s + "O", tropism.order_response);
  }
}
