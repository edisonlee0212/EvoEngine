#include "MaizeTasselDescriptor.hpp"
#include "LSystemDescriptorDefaults.hpp"
#include "MaizeTassel.hpp"
#include <Application.hpp>
#include <EditorLayer.hpp>
#include <Scene.hpp>
#include <Transform.hpp>
#include <array>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <yaml-cpp/yaml.h>

using namespace l_system_plugin;
using namespace evo_engine;

namespace {

constexpr char kMaizeDescriptorName[] = "MaizeTasselDescriptor";

const std::array<std::filesystem::path, 6> kMaizeResourceCandidates = {
    std::filesystem::path("./LSystemResources/Defaults/MaizeTasselDescriptor_Default.mtassel"),
    std::filesystem::path("./EvoEngine_Plugins/LSystem/Internals/LSystemResources/Defaults/") /
        "MaizeTasselDescriptor_Default.mtassel",
    std::filesystem::path("./Resources/DigitalAgricultureProject/Assets/New MaizeTasselDescriptor.mtassel"),
    std::filesystem::path("./DigitalAgricultureProject/Assets/New MaizeTasselDescriptor.mtassel"),
    std::filesystem::path("./04_EvoEngine/Resources/DigitalAgricultureProject/Assets/") /
        "New MaizeTasselDescriptor.mtassel",
    std::filesystem::path("./04_EvoEngine/EvoEngine_Plugins/LSystem/Internals/") /
        "LSystemResources/Defaults/MaizeTasselDescriptor_Default.mtassel"};

const std::array<std::filesystem::path, 2> kMaizeProjectAssetCandidates = {
    std::filesystem::path("LSystem") / "New MaizeTasselDescriptor.mtassel",
    "New MaizeTasselDescriptor.mtassel"};

const std::array<std::filesystem::path, 2> kMaizeWritableTemplateCandidates = {
    std::filesystem::path("./Resources/DigitalAgricultureProject/Assets/") /
        "New MaizeTasselDescriptor.mtassel",
    std::filesystem::path("./04_EvoEngine/Resources/DigitalAgricultureProject/Assets/") /
        "New MaizeTasselDescriptor.mtassel"};

const std::filesystem::path kMaizeFallbackDefaultsPath =
    std::filesystem::path("./LSystemResources/Defaults/MaizeTasselDescriptor_Default.mtassel");

double GetSteadyTimeSeconds() {
  return std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}

std::filesystem::path ResolveDefaultMaizeTasselDescriptorPath() {
  return descriptor_defaults::ResolveExistingDefaultsPath(
      kMaizeResourceCandidates,
      kMaizeProjectAssetCandidates);
}

std::filesystem::path ResolveWritableMaizeTasselDescriptorDefaultsPath() {
  return descriptor_defaults::ResolveWritableDefaultsPath(
      kMaizeResourceCandidates,
      kMaizeProjectAssetCandidates,
      kMaizeWritableTemplateCandidates,
      kMaizeFallbackDefaultsPath);
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

float SampleTargetGddForSeed(const evo_engine::SingleDistribution<float>& distribution,
                             const uint32_t seed) {
  std::mt19937 rng(seed);
  return std::max(0.0f, SampleDistribution(distribution, rng));
}

bool LoadMaizeTasselDescriptorDefaultsFromFile(MaizeTasselDescriptor& descriptor,
                                               const std::filesystem::path& file_path) {
  YAML::Node defaults;
  if (!descriptor_defaults::LoadDefaultsYamlMap(
          file_path,
          defaults,
          kMaizeDescriptorName)) {
    return false;
  }
  descriptor.Deserialize(defaults);
  return true;
}
}

MaizeTasselDescriptor::MaizeTasselDescriptor() {
  const auto defaults_path = ResolveDefaultMaizeTasselDescriptorPath();
  if (!LoadMaizeTasselDescriptorDefaultsFromFile(*this, defaults_path)) {
    static bool warned_once = false;
    if (!warned_once) {
      warned_once = true;
      EVOENGINE_WARNING(
          "MaizeTasselDescriptor defaults file not found or invalid. Using inline member defaults.");
    }
  }
}

std::filesystem::path MaizeTasselDescriptor::ResolveWritableDefaultsPath() const {
  return ResolveWritableMaizeTasselDescriptorDefaultsPath();
}

// ---------------------------------------------------------------------------
// Sampling
// ---------------------------------------------------------------------------

SampledTasselParams MaizeTasselDescriptor::Sample(std::mt19937& rng) const {
  SampledTasselParams p;

  // Branch zone (lower rachis).
  p.branch_node_count = std::max(0, static_cast<int>(std::round(SampleDistribution(branch_node_count, rng))));
  p.branch_internode_length = branch_internode_length;
  p.branch_internode_thickness = branch_internode_thickness;
  p.lateral_insertion_angle = lateral_insertion_angle;
  p.lateral_internode_length = lateral_internode_length;
  p.lateral_node_count = lateral_node_count;
  p.peduncle_branch_probability = peduncle_branch_probability;

  // Central spike (upper rachis).
  p.spike_node_count = std::max(0, static_cast<int>(std::round(SampleDistribution(spike_node_count, rng))));
  p.spike_internode_length = spike_internode_length;
  p.spike_internode_thickness = spike_internode_thickness;
  p.spike_zone_branch_probability = spike_zone_branch_probability;

  // Main-rachis pair morphology.
  p.main_pair_proximal_scale_x = main_pair_proximal_scale_x;
  p.main_pair_proximal_scale_y = main_pair_proximal_scale_y;
  p.main_pair_proximal_scale_z = main_pair_proximal_scale_z;
  p.main_pair_proximal_angle = main_pair_proximal_angle;
  p.main_pair_internode_length = main_pair_internode_length;
  p.main_pair_internode_thickness = main_pair_internode_thickness;
  p.main_pair_internode_angle = main_pair_internode_angle;
  p.main_pair_distal_scale_x = main_pair_distal_scale_x;
  p.main_pair_distal_scale_y = main_pair_distal_scale_y;
  p.main_pair_distal_scale_z = main_pair_distal_scale_z;
  p.main_pair_distal_angle = main_pair_distal_angle;

  // Non-main-axis pair morphology.
  p.branch_pair_proximal_scale_x = branch_pair_proximal_scale_x;
  p.branch_pair_proximal_scale_y = branch_pair_proximal_scale_y;
  p.branch_pair_proximal_scale_z = branch_pair_proximal_scale_z;
  p.branch_pair_proximal_angle = branch_pair_proximal_angle;
  p.branch_pair_internode_length = branch_pair_internode_length;
  p.branch_pair_internode_thickness = branch_pair_internode_thickness;
  p.branch_pair_internode_angle = branch_pair_internode_angle;
  p.branch_pair_distal_scale_x = branch_pair_distal_scale_x;
  p.branch_pair_distal_scale_y = branch_pair_distal_scale_y;
  p.branch_pair_distal_scale_z = branch_pair_distal_scale_z;
  p.branch_pair_distal_angle = branch_pair_distal_angle;

  // Branch timing and branch probabilities.
  p.lateral_initiation_delay_gdd = lateral_initiation_delay_gdd;
  p.spike_anthesis_offset_gdd = spike_anthesis_offset_gdd;
  p.primary_lateral_branch_probability = primary_lateral_branch_probability;
  p.secondary_lateral_branch_probability = secondary_lateral_branch_probability;

  // Shared.
  p.phyllotaxis_angle = SampleDistribution(phyllotaxis_angle, rng);
  p.branch_azimuth_offset = branch_azimuth_offset;
  p.lateral_thickness_ratio = std::max(0.01f, SampleDistribution(lateral_thickness_ratio, rng));
  p.final_age_gdd = final_age_gdd;
  p.final_age_gdd.mean = std::max(1.0f, p.final_age_gdd.mean);
  p.final_age_gdd.deviation = std::max(0.0f, p.final_age_gdd.deviation);

  // Secondary branches.
  p.secondary_insertion_angle = SampleDistribution(secondary_insertion_angle, rng);
  p.secondary_internode_length = std::max(0.01f, SampleDistribution(secondary_internode_length, rng));
  p.secondary_internode_thickness = std::max(0.01f, SampleDistribution(secondary_internode_thickness, rng));
  p.secondary_node_count = std::max(0, static_cast<int>(std::round(SampleDistribution(secondary_node_count, rng))));

  // Growth curves.
  p.rachis_elongation_curve = rachis_elongation_curve;
  p.rachis_thickness_curve = rachis_thickness_curve;
  p.lateral_elongation_curve = lateral_elongation_curve;
  p.lateral_thickness_curve = lateral_thickness_curve;
  p.lateral_angle_development_curve = lateral_angle_development_curve;
  p.pair_proximal_scale_curve = pair_proximal_scale_curve;
  p.pair_proximal_angle_curve = pair_proximal_angle_curve;
  p.pair_internode_length_curve = pair_internode_length_curve;
  p.pair_internode_thickness_curve = pair_internode_thickness_curve;
  p.pair_internode_angle_curve = pair_internode_angle_curve;
  p.pair_distal_scale_curve = pair_distal_scale_curve;
  p.pair_distal_angle_curve = pair_distal_angle_curve;

  // Tropisms.
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
    p.tropisms.push_back(std::move(st));
  }

  // Thermal timing.
  p.plastochron_gdd = std::max(1.0f, SampleDistribution(plastochron_gdd, rng));
  p.anthesis_gdd = std::max(0.0f, SampleDistribution(anthesis_gdd, rng));
  p.maturity_gdd = std::max(p.anthesis_gdd + 1.0f, SampleDistribution(maturity_gdd, rng));
  p.main_axis_plastochron_scale = std::max(0.1f, SampleDistribution(main_axis_plastochron_scale, rng));
  p.lateral_axis_plastochron_scale = std::max(0.1f, SampleDistribution(lateral_axis_plastochron_scale, rng));
  p.lateral_bud_plastochron_scale = std::max(0.1f, SampleDistribution(lateral_bud_plastochron_scale, rng));
  p.maturity_initiation_coupling = std::max(0.0f, SampleDistribution(maturity_initiation_coupling, rng));
  p.reference_maturity_gdd = std::max(1.0f, SampleDistribution(reference_maturity_gdd, rng));
  p.branch_angle_relaxation = std::clamp(SampleDistribution(branch_angle_relaxation, rng), 0.001f, 1.0f);
  p.pair_angle_relaxation = std::clamp(SampleDistribution(pair_angle_relaxation, rng), 0.001f, 1.0f);
  p.stage_1_end_t = std::clamp(SampleDistribution(stage_1_end_t, rng), 0.0f, 1.0f);
  p.stage_2_end_t = std::clamp(
    SampleDistribution(stage_2_end_t, rng),
    std::min(1.0f, p.stage_1_end_t + 0.02f),
    1.0f);
  p.stage_3_end_t = std::clamp(
    SampleDistribution(stage_3_end_t, rng),
    std::min(1.0f, p.stage_2_end_t + 0.02f),
    1.0f);
  p.secondary_ramp_start_t = std::clamp(SampleDistribution(secondary_ramp_start_t, rng), 0.0f, 1.0f);
  p.secondary_ramp_end_t = std::clamp(
    SampleDistribution(secondary_ramp_end_t, rng),
    std::min(1.0f, p.secondary_ramp_start_t + 0.02f),
    1.0f);
  p.mature_droop_start_t = std::clamp(SampleDistribution(mature_droop_start_t, rng), 0.0f, 1.0f);
  p.mature_droop_strength = std::max(0.0f, SampleDistribution(mature_droop_strength, rng));

  return p;
}

// ---------------------------------------------------------------------------
// Instantiate — create entity with MaizeTassel component
// ---------------------------------------------------------------------------

Entity MaizeTasselDescriptor::Instantiate() const {
  const auto scene = Application::GetActiveScene();
  if (!scene)
    return {};

  const auto entity = scene->CreateEntity(GetTitle());
  const auto tassel = scene->GetOrSetPrivateComponent<MaizeTassel>(entity).lock();
  tassel->descriptor_ref = GetSelf();
  tassel->target_gdd = SampleTargetGddForSeed(target_gdd, tassel->seed);
  tassel->GenerateGeometryEntities();

  return entity;
}

// ---------------------------------------------------------------------------
// Inspector UI
// ---------------------------------------------------------------------------

bool MaizeTasselDescriptor::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
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

  // -- Instantiation controls --
  if (ImGui::Button("Instantiate")) {
    editor_layer->SetSelectedEntity(Instantiate());
  }
  show_item_hover_description("Create a new MaizeTassel entity using this descriptor and select it in the scene.");

  ImGui::SameLine();
  if (ImGui::Checkbox("Live Preview", &live_preview)) {
    editor_preferences_changed = true;
    if (!live_preview) {
      live_preview_dirty_ = false;
      live_preview_was_dragging_ = false;
      live_preview_needs_full_apply_ = false;
    }
  }
  show_item_hover_description("Regenerate matching tassels while editing this descriptor.");

  if (ImGui::DragFloat("Live Preview Rate (Hz)",
                       &live_preview_rate_hz,
                       0.25f,
                       1.0f,
                       60.0f,
                       "%.1f")) {
    live_preview_rate_hz = std::clamp(live_preview_rate_hz, 1.0f, 60.0f);
    editor_preferences_changed = true;
  }
  show_item_hover_description("Maximum live-preview apply frequency. Higher values update more often but cost more CPU.");

  if (ImGui::Checkbox("Representative Only While Dragging", &live_preview_representative_only)) {
    editor_preferences_changed = true;
  }
  show_item_hover_description("While dragging controls, preview only one matching tassel for responsiveness.");

  if (ImGui::Checkbox("Cap Preview Target GDD", &live_preview_cap_target_gdd)) {
    editor_preferences_changed = true;
  }
  show_item_hover_description("Clamp preview simulation age so live updates stay fast on very mature tassels.");

  if (ImGui::DragFloat("Preview Max GDD",
                       &live_preview_max_gdd,
                       5.0f,
                       0.0f,
                       5000.0f,
                       "%.1f")) {
    live_preview_max_gdd = std::max(0.0f, live_preview_max_gdd);
    editor_preferences_changed = true;
  }
  show_item_hover_description("Upper GDD limit used when preview capping is enabled.");

  if (ImGui::DragInt("Preview Max Growth Steps",
                     &live_preview_max_growth_steps,
                     1.0f,
                     1,
                     10000)) {
    live_preview_max_growth_steps = std::clamp(live_preview_max_growth_steps, 1, 10000);
    editor_preferences_changed = true;
  }
  show_item_hover_description("Maximum derivation/growth iterations used by drag-time preview updates.");

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

  // -- Grid instantiation --
  if (ImGui::TreeNodeEx("Grid Instantiate")) {
    ImGui::DragInt("Rows", &grid_rows, 1, 1, 50);
    show_item_hover_description("Number of rows for grid instantiation.");
    ImGui::DragInt("Cols", &grid_cols, 1, 1, 50);
    show_item_hover_description("Number of columns for grid instantiation.");
    ImGui::DragFloat("Spacing", &grid_spacing, 0.1f, 0.5f, 50.0f);
    show_item_hover_description("World-space spacing between neighboring grid tassels.");

    if (ImGui::Button("Instantiate Grid")) {
      const auto scene = Application::GetActiveScene();
      if (scene) {
        const auto container = scene->CreateEntity("Tassel Grid");
        const float offset_y = (static_cast<float>(grid_rows) - 1.0f) * grid_spacing * 0.5f;
        const float offset_z = (static_cast<float>(grid_cols) - 1.0f) * grid_spacing * 0.5f;
        const auto base_seed = static_cast<unsigned int>(
            std::chrono::steady_clock::now().time_since_epoch().count() & 0xFFFFFFFFu);
        for (int i = 0; i < grid_rows; i++) {
          for (int j = 0; j < grid_cols; j++) {
            const auto entity = scene->CreateEntity(GetTitle() + " [" + std::to_string(i) + "," + std::to_string(j) + "]");
            const auto tassel = scene->GetOrSetPrivateComponent<MaizeTassel>(entity).lock();
            tassel->descriptor_ref = GetSelf();
            tassel->seed = base_seed + static_cast<unsigned int>(i * grid_cols + j);
            tassel->target_gdd = SampleTargetGddForSeed(target_gdd, tassel->seed);

            scene->SetParent(entity, container, false);

            Transform transform;
            transform.SetPosition(glm::vec3(
                0.0f,
                static_cast<float>(i) * grid_spacing - offset_y,
                static_cast<float>(j) * grid_spacing - offset_z));
            scene->SetDataComponent(entity, transform);

            tassel->GenerateGeometryEntities();
          }
        }
      }
    }
    show_item_hover_description("Spawn a grid of MaizeTassel entities from this descriptor with unique seeds.");

    ImGui::SameLine();
    if (ImGui::Button("Delete Grid")) {
      const auto scene = Application::GetActiveScene();
      if (scene) {
        const auto* tassel_entities_ptr = scene->UnsafeGetPrivateComponentOwnersList<MaizeTassel>();
        if (tassel_entities_ptr) {
          // Copy before iterating to avoid dangling pointer if p_owners_collections_list_ reallocates.
          const std::vector<Entity> tassel_entities = *tassel_entities_ptr;
          // Collect matching tassel entities and their container parents.
          std::vector<Entity> to_delete;
          std::vector<Entity> containers;
          for (const auto& entity : tassel_entities) {
            if (!scene->IsEntityValid(entity))
              continue;
            auto tassel = scene->GetOrSetPrivateComponent<MaizeTassel>(entity).lock();
            if (!tassel)
              continue;
            if (tassel->descriptor_ref.Get<MaizeTasselDescriptor>().get() == this) {
              to_delete.push_back(entity);
              const auto parent = scene->GetParent(entity);
              if (scene->IsEntityValid(parent) && scene->GetEntityName(parent) == "Tassel Grid") {
                containers.push_back(parent);
              }
            }
          }
          for (const auto& entity : to_delete) {
            scene->DeleteEntity(entity);
          }
          // Deduplicate and delete now-empty container entities.
          std::sort(containers.begin(), containers.end(),
                    [](const Entity& a, const Entity& b) { return a.GetIndex() < b.GetIndex(); });
          containers.erase(std::unique(containers.begin(), containers.end()), containers.end());
          for (const auto& container : containers) {
            if (scene->IsEntityValid(container))
              scene->DeleteEntity(container);
          }
        }
      }
    }
    show_item_hover_description("Delete MaizeTassel entities that use this descriptor and remove now-empty grid containers.");

    ImGui::TreePop();
  }

  ImGui::Separator();

  // -- Parameter Space Explorer --
  if (ImGui::TreeNodeEx("Parameter Space Explorer")) {
    if (!explorer_.IsBound()) explorer_.Bind(*this);
    if (explorer_.OnInspect()) {
      changed = true;
    }
    show_item_hover_description("Interactive parameter sweep and sensitivity exploration tools for this descriptor.");
    ImGui::TreePop();
  }

  ImGui::Separator();

  if (ImGui::TreeNodeEx("Main Axis (Peduncle + Spike Zone)", ImGuiTreeNodeFlags_DefaultOpen)) {
    changed |= branch_node_count.OnInspect("Peduncle Node Count", 0.5f);
    show_item_hover_description("Mean and deviation for peduncle node count before the spike zone.");
    changed |= branch_internode_length.OnInspect("Peduncle Internode Length (Position)");
    show_item_hover_description("Position-dependent peduncle internode length profile along normalized main-axis position.");
    changed |= branch_internode_thickness.OnInspect("Peduncle Internode Thickness (Position)");
    show_item_hover_description("Position-dependent peduncle internode radius/thickness profile.");

    changed |= spike_node_count.OnInspect("Spike-Zone Node Count", 0.5f);
    show_item_hover_description("Mean and deviation for node count in the upper spike zone.");
    changed |= spike_internode_length.OnInspect("Spike-Zone Internode Length (Position)");
    show_item_hover_description("Position-dependent spike-zone internode length profile.");
    changed |= spike_internode_thickness.OnInspect("Spike-Zone Internode Thickness (Position)");
    show_item_hover_description("Position-dependent spike-zone internode thickness profile.");

    ImGui::PushID("curve_rachis_elongation");
    ImGui::TextUnformatted("Age Curve: Main-axis internode elongation progression");
    changed |= rachis_elongation_curve.OnInspect("Main Axis Length Growth");
    show_item_hover_description("Curve editor for main-axis elongation over internode age (x: normalized age, y: length multiplier).");
    ImGui::PopID();

    ImGui::PushID("curve_rachis_thickness");
    ImGui::TextUnformatted("Age Curve: Main-axis internode thickness progression");
    changed |= rachis_thickness_curve.OnInspect("Main Axis Thickness Growth");
    show_item_hover_description("Curve editor for main-axis thickness development over internode age.");
    ImGui::PopID();

    changed |= phyllotaxis_angle.OnInspect("Base Phyllotaxis Angle", 1.0f);
    show_item_hover_description("Baseline angular separation between successive organs around the main axis.");
    changed |= branch_azimuth_offset.OnInspect("Branch Azimuth Offset", 0.5f);
    show_item_hover_description("Global azimuth offset applied to lateral branch orientation.");
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Main Stem Branching (Peduncle)", ImGuiTreeNodeFlags_DefaultOpen)) {
    changed |= peduncle_branch_probability.OnInspect("Peduncle Branch Probability (Position)");
    show_item_hover_description("Position-dependent probability of lateral branch initiation on peduncle nodes.");
    changed |= lateral_initiation_delay_gdd.OnInspect("Lateral Initiation Delay (GDD)");
    show_item_hover_description("Thermal-time delay between node availability and lateral branch initiation.");
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Main Stem Branching (Spike Zone)", ImGuiTreeNodeFlags_DefaultOpen)) {
    changed |= spike_zone_branch_probability.OnInspect("Spike-Zone Branch Probability (Position)");
    show_item_hover_description("Position-dependent probability of branch/spikelet initiation in the spike zone.");
    changed |= spike_anthesis_offset_gdd.OnInspect("Anthesis Offset (GDD)");
    show_item_hover_description("Thermal-time offset from initiation to anthesis timing.");
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Primary Lateral Branches", ImGuiTreeNodeFlags_DefaultOpen)) {
    changed |= lateral_insertion_angle.OnInspect("Insertion Angle (Position)");
    show_item_hover_description("Position-dependent insertion angle of primary lateral branches.");
    changed |= lateral_internode_length.OnInspect("Internode Length (Position)");
    show_item_hover_description("Position-dependent internode length profile for primary laterals.");
    changed |= lateral_node_count.OnInspect("Relative Node Count (Position)");
    show_item_hover_description("Relative node-count profile for primary lateral branches along their axis.");
    changed |= primary_lateral_branch_probability.OnInspect("Primary->Secondary Branch Probability (Position)");
    show_item_hover_description("Probability that a primary lateral node emits a secondary branch.");
    changed |= lateral_thickness_ratio.OnInspect("Lateral Thickness Ratio", 0.01f);
    show_item_hover_description("Thickness ratio of lateral branches relative to their parent axis.");

    ImGui::PushID("curve_lateral_elongation");
    ImGui::TextUnformatted("Age Curve: Lateral internode elongation progression");
    changed |= lateral_elongation_curve.OnInspect("Lateral Length Growth");
    show_item_hover_description("Curve editor for lateral internode elongation over age.");
    ImGui::PopID();

    ImGui::PushID("curve_lateral_thickness");
    ImGui::TextUnformatted("Age Curve: Lateral internode thickness progression");
    changed |= lateral_thickness_curve.OnInspect("Lateral Thickness Growth");
    show_item_hover_description("Curve editor for lateral internode thickness development over age.");
    ImGui::PopID();

    ImGui::PushID("curve_lateral_angle");
    ImGui::TextUnformatted("Age Curve: Lateral insertion angle opening progression");
    changed |= lateral_angle_development_curve.OnInspect("Lateral Angle Growth");
    show_item_hover_description("Curve editor for lateral opening angle progression over age.");
    ImGui::PopID();

    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Secondary Branches", ImGuiTreeNodeFlags_DefaultOpen)) {
    changed |= secondary_lateral_branch_probability.OnInspect("Secondary->Secondary Branch Probability (Position)");
    show_item_hover_description("Probability of tertiary branching from secondary axes.");
    changed |= secondary_insertion_angle.OnInspect("Secondary Insertion Angle", 0.5f);
    show_item_hover_description("Mean and deviation of insertion angle for secondary branches.");
    changed |= secondary_internode_length.OnInspect("Secondary Internode Length", 0.1f);
    show_item_hover_description("Mean and deviation of internode length on secondary branches.");
    changed |= secondary_internode_thickness.OnInspect("Secondary Internode Thickness", 0.01f);
    show_item_hover_description("Mean and deviation of internode thickness on secondary branches.");
    changed |= secondary_node_count.OnInspect("Secondary Relative Node Count", 0.5f);
    show_item_hover_description("Mean and deviation of relative node count for secondary branches.");
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Spikelet Pair Morphology (Main Rachis)", ImGuiTreeNodeFlags_DefaultOpen)) {
    changed |= main_pair_proximal_scale_x.OnInspect("Proximal Ellipsoid Scale X (Position)");
    show_item_hover_description("Position-dependent X scale of proximal spikelet ellipsoids on the main rachis.");
    changed |= main_pair_proximal_scale_y.OnInspect("Proximal Ellipsoid Scale Y (Position)");
    show_item_hover_description("Position-dependent Y scale of proximal spikelet ellipsoids on the main rachis.");
    changed |= main_pair_proximal_scale_z.OnInspect("Proximal Ellipsoid Scale Z (Position)");
    show_item_hover_description("Position-dependent Z scale of proximal spikelet ellipsoids on the main rachis.");
    changed |= main_pair_proximal_angle.OnInspect("Proximal Ellipsoid Branch Angle (Position)");
    show_item_hover_description("Position-dependent branch angle for proximal spikelet elements on the main rachis.");
    changed |= main_pair_internode_length.OnInspect("Pair Internode Length (Position)");
    show_item_hover_description("Position-dependent internode length between paired spikelet elements on the main rachis.");
    changed |= main_pair_internode_thickness.OnInspect("Pair Internode Thickness (Position)");
    show_item_hover_description("Position-dependent internode thickness for spikelet pairs on the main rachis.");
    changed |= main_pair_internode_angle.OnInspect("Pair Internode Branch Angle (Position)");
    show_item_hover_description("Position-dependent branch angle of pair internodes on the main rachis.");
    changed |= main_pair_distal_scale_x.OnInspect("Distal Ellipsoid Scale X (Position)");
    show_item_hover_description("Position-dependent X scale of distal spikelet ellipsoids on the main rachis.");
    changed |= main_pair_distal_scale_y.OnInspect("Distal Ellipsoid Scale Y (Position)");
    show_item_hover_description("Position-dependent Y scale of distal spikelet ellipsoids on the main rachis.");
    changed |= main_pair_distal_scale_z.OnInspect("Distal Ellipsoid Scale Z (Position)");
    show_item_hover_description("Position-dependent Z scale of distal spikelet ellipsoids on the main rachis.");
    changed |= main_pair_distal_angle.OnInspect("Distal Ellipsoid Branch Angle (Position)");
    show_item_hover_description("Position-dependent branch angle for distal spikelet elements on the main rachis.");
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Spikelet Pair Morphology (Peduncle Branches)", ImGuiTreeNodeFlags_DefaultOpen)) {
    changed |= branch_pair_proximal_scale_x.OnInspect("Proximal Ellipsoid Scale X (Position)");
    show_item_hover_description("Position-dependent X scale of proximal spikelet ellipsoids on peduncle branches.");
    changed |= branch_pair_proximal_scale_y.OnInspect("Proximal Ellipsoid Scale Y (Position)");
    show_item_hover_description("Position-dependent Y scale of proximal spikelet ellipsoids on peduncle branches.");
    changed |= branch_pair_proximal_scale_z.OnInspect("Proximal Ellipsoid Scale Z (Position)");
    show_item_hover_description("Position-dependent Z scale of proximal spikelet ellipsoids on peduncle branches.");
    changed |= branch_pair_proximal_angle.OnInspect("Proximal Ellipsoid Branch Angle (Position)");
    show_item_hover_description("Position-dependent branch angle for proximal spikelet elements on peduncle branches.");
    changed |= branch_pair_internode_length.OnInspect("Pair Internode Length (Position)");
    show_item_hover_description("Position-dependent internode length between paired spikelets on peduncle branches.");
    changed |= branch_pair_internode_thickness.OnInspect("Pair Internode Thickness (Position)");
    show_item_hover_description("Position-dependent internode thickness for peduncle-branch spikelet pairs.");
    changed |= branch_pair_internode_angle.OnInspect("Pair Internode Branch Angle (Position)");
    show_item_hover_description("Position-dependent pair internode branch angle on peduncle branches.");
    changed |= branch_pair_distal_scale_x.OnInspect("Distal Ellipsoid Scale X (Position)");
    show_item_hover_description("Position-dependent X scale of distal spikelet ellipsoids on peduncle branches.");
    changed |= branch_pair_distal_scale_y.OnInspect("Distal Ellipsoid Scale Y (Position)");
    show_item_hover_description("Position-dependent Y scale of distal spikelet ellipsoids on peduncle branches.");
    changed |= branch_pair_distal_scale_z.OnInspect("Distal Ellipsoid Scale Z (Position)");
    show_item_hover_description("Position-dependent Z scale of distal spikelet ellipsoids on peduncle branches.");
    changed |= branch_pair_distal_angle.OnInspect("Distal Ellipsoid Branch Angle (Position)");
    show_item_hover_description("Position-dependent branch angle for distal spikelet elements on peduncle branches.");
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Spikelet Pair Growth Curves", ImGuiTreeNodeFlags_DefaultOpen)) {
    changed |= pair_proximal_scale_curve.OnInspect("Proximal Scale Growth");
    show_item_hover_description("Curve editor for proximal spikelet size growth over age.");
    changed |= pair_proximal_angle_curve.OnInspect("Proximal Angle Growth");
    show_item_hover_description("Curve editor for proximal spikelet branch-angle development over age.");
    changed |= pair_internode_length_curve.OnInspect("Pair Internode Length Growth");
    show_item_hover_description("Curve editor for spikelet-pair internode length progression over age.");
    changed |= pair_internode_thickness_curve.OnInspect("Pair Internode Thickness Growth");
    show_item_hover_description("Curve editor for spikelet-pair internode thickness progression over age.");
    changed |= pair_internode_angle_curve.OnInspect("Pair Internode Angle Growth");
    show_item_hover_description("Curve editor for spikelet-pair internode branch-angle progression over age.");
    changed |= pair_distal_scale_curve.OnInspect("Distal Scale Growth");
    show_item_hover_description("Curve editor for distal spikelet size growth over age.");
    changed |= pair_distal_angle_curve.OnInspect("Distal Angle Growth");
    show_item_hover_description("Curve editor for distal spikelet branch-angle development over age.");
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Tropisms")) {
    if (ImGui::Button("+ Add Tropism")) {
      tropisms.emplace_back();
      changed = true;
    }
    show_item_hover_description("Append a new tropism rule entry to the active tropism set.");
    int remove_idx = -1;
    for (int i = 0; i < static_cast<int>(tropisms.size()); i++) {
      ImGui::PushID(i);
      const std::string header = "Tropism " + std::to_string(i);
      if (ImGui::TreeNodeEx(header.c_str())) {
        changed |= tropisms[i].direction_x.OnInspect("Direction X", 0.01f);
        show_item_hover_description("X component of tropism direction before normalization.");
        changed |= tropisms[i].direction_y.OnInspect("Direction Y", 0.01f);
        show_item_hover_description("Y component of tropism direction before normalization.");
        changed |= tropisms[i].direction_z.OnInspect("Direction Z", 0.01f);
        show_item_hover_description("Z component of tropism direction before normalization.");
        changed |= tropisms[i].strength.OnInspect("Strength", 0.01f);
        show_item_hover_description("Magnitude of tropism influence when this entry is active.");
        if (ImGui::DragFloat("Usage Chance (%)",
                             &tropisms[i].usage_chance_percent,
                             0.5f,
                             0.0f,
                             100.0f,
                             "%.1f")) {
          tropisms[i].usage_chance_percent = std::clamp(tropisms[i].usage_chance_percent, 0.0f, 100.0f);
          changed = true;
        }
        show_item_hover_description("Probability that this tropism entry is included during sampling.");
        changed |= tropisms[i].order_response.OnInspect("Order Response");
        show_item_hover_description("Branch-order response multipliers for this tropism entry.");
        if (ImGui::Button("Remove")) {
          remove_idx = i;
        }
        show_item_hover_description("Delete this tropism entry.");
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

  if (ImGui::TreeNodeEx("Global Development", ImGuiTreeNodeFlags_DefaultOpen)) {
    const auto inspect_global_distribution_row = [&](const char* label,
                             evo_engine::SingleDistribution<float>& distribution,
                             const float speed,
                             const float mean_min,
                             const float mean_max,
                             const float deviation_max,
                             const char* description) {
      bool row_changed = false;
      ImGui::PushID(label);

      ImGui::TableNextRow();

      ImGui::TableSetColumnIndex(0);
      ImGui::AlignTextToFramePadding();
      ImGui::TextUnformatted(label);
      show_item_hover_description(description);

      ImGui::TableSetColumnIndex(1);
      row_changed |= ImGui::DragFloat("##mean", &distribution.mean, speed, mean_min, mean_max);
      show_item_hover_description(description);

      ImGui::TableSetColumnIndex(2);
      row_changed |= ImGui::DragFloat("##deviation", &distribution.deviation, speed, 0.0f, deviation_max);
      show_item_hover_description(description);

      ImGui::PopID();
      return row_changed;
    };

    if (ImGui::BeginTable("GlobalDevelopmentTable",
                3,
                ImGuiTableFlags_SizingStretchSame |
                  ImGuiTableFlags_BordersInnerV |
                  ImGuiTableFlags_RowBg)) {
      ImGui::TableSetupColumn("Parameter");
      ImGui::TableSetupColumn("Mean");
      ImGui::TableSetupColumn("Deviation");
      ImGui::TableHeadersRow();

      changed |= inspect_global_distribution_row(
        "Target GDD",
        target_gdd,
        5.0f,
        0.0f,
        4000.0f,
        4000.0f,
        "Per-instance thermal target used to set MaizeTassel growth stop (target_gdd)."
      );
      changed |= inspect_global_distribution_row(
        "Thermal GDD/day",
        gdd_per_day,
        0.25f,
        0.0f,
        500.0f,
        500.0f,
        "Per-instance thermal progression rate used by auto-grow (GDD per day)."
      );
      changed |= inspect_global_distribution_row(
        "Plastochron GDD",
        plastochron_gdd,
        1.0f,
        1.0f,
        200.0f,
        200.0f,
        "Thermal time between successive organ initiations on the development clock.");
      changed |= inspect_global_distribution_row(
        "Anthesis GDD",
        anthesis_gdd,
        5.0f,
        10.0f,
        1000.0f,
        1000.0f,
        "Target GDD for anthesis timing in the developmental schedule.");
      changed |= inspect_global_distribution_row(
        "Internode Maturity GDD",
        maturity_gdd,
        5.0f,
        10.0f,
        2000.0f,
        2000.0f,
        "Thermal age at which internodes are considered mature.");
      changed |= inspect_global_distribution_row(
        "Main-Axis Plastochron Scale",
        main_axis_plastochron_scale,
        0.01f,
        0.1f,
        5.0f,
        5.0f,
        "Multiplier applied to main-axis plastochron timing.");
      changed |= inspect_global_distribution_row(
        "Lateral-Axis Plastochron Scale",
        lateral_axis_plastochron_scale,
        0.01f,
        0.1f,
        5.0f,
        5.0f,
        "Multiplier applied to lateral-axis plastochron timing.");
      changed |= inspect_global_distribution_row(
        "Lateral-Bud Plastochron Scale",
        lateral_bud_plastochron_scale,
        0.01f,
        0.1f,
        5.0f,
        5.0f,
        "Multiplier applied to lateral-bud initiation plastochron timing.");
      changed |= inspect_global_distribution_row(
        "Maturity->Initiation Coupling",
        maturity_initiation_coupling,
        0.01f,
        0.0f,
        2.0f,
        2.0f,
        "Coupling strength between organ maturity progression and new initiation timing.");
      changed |= inspect_global_distribution_row(
        "Reference Maturity GDD",
        reference_maturity_gdd,
        5.0f,
        1.0f,
        4000.0f,
        4000.0f,
        "Reference thermal age used to normalize maturity-dependent timing effects.");
      changed |= inspect_global_distribution_row(
        "Branch Angle Relaxation",
        branch_angle_relaxation,
        0.001f,
        0.001f,
        1.0f,
        1.0f,
        "Smoothing factor controlling how quickly branch insertion angles approach target values.");
      changed |= inspect_global_distribution_row(
        "Pair Angle Relaxation",
        pair_angle_relaxation,
        0.001f,
        0.001f,
        1.0f,
        1.0f,
        "Smoothing factor controlling how quickly spikelet pair angles approach target values.");
      changed |= inspect_global_distribution_row(
        "Stage 1 End (normalized)",
        stage_1_end_t,
        0.005f,
        0.0f,
        1.0f,
        1.0f,
        "End of compressed vertical phase, normalized by maturity GDD.");
      changed |= inspect_global_distribution_row(
        "Stage 2 End (normalized)",
        stage_2_end_t,
        0.005f,
        0.0f,
        1.0f,
        1.0f,
        "End of early separation phase, normalized by maturity GDD.");
      changed |= inspect_global_distribution_row(
        "Stage 3 End (normalized)",
        stage_3_end_t,
        0.005f,
        0.0f,
        1.0f,
        1.0f,
        "End of progressive unfurling phase, normalized by maturity GDD.");
      changed |= inspect_global_distribution_row(
        "Secondary Ramp Start (normalized)",
        secondary_ramp_start_t,
        0.005f,
        0.0f,
        1.0f,
        1.0f,
        "Normalized age where secondary-branch probability starts ramping up.");
      changed |= inspect_global_distribution_row(
        "Secondary Ramp End (normalized)",
        secondary_ramp_end_t,
        0.005f,
        0.0f,
        1.0f,
        1.0f,
        "Normalized age where secondary-branch probability reaches full strength.");
      changed |= inspect_global_distribution_row(
        "Mature Droop Start (normalized)",
        mature_droop_start_t,
        0.005f,
        0.0f,
        1.0f,
        1.0f,
        "Normalized age where downward mature-stage droop starts.");
      changed |= inspect_global_distribution_row(
        "Mature Droop Strength",
        mature_droop_strength,
        0.01f,
        0.0f,
        20.0f,
        20.0f,
        "Additional downward bending strength applied after mature droop start.");
      changed |= inspect_global_distribution_row(
        "Spikelet Pair Final Age GDD",
        final_age_gdd,
        1.0f,
        1.0f,
        4000.0f,
        4000.0f,
        "Distribution of final spikelet-pair age at full maturity (GDD since spikelet birth).");

      ImGui::EndTable();
    }

    const auto clamp_distribution = [](evo_engine::SingleDistribution<float>& distribution,
                       const float min_mean,
                       const float max_mean) {
      distribution.mean = std::clamp(distribution.mean, min_mean, max_mean);
      distribution.deviation = std::max(0.0f, distribution.deviation);
    };

    clamp_distribution(target_gdd, 0.0f, 4000.0f);
    clamp_distribution(gdd_per_day, 0.0f, 500.0f);
    clamp_distribution(plastochron_gdd, 1.0f, 200.0f);
    clamp_distribution(anthesis_gdd, 10.0f, 1000.0f);
    clamp_distribution(maturity_gdd, 10.0f, 2000.0f);
    clamp_distribution(main_axis_plastochron_scale, 0.1f, 5.0f);
    clamp_distribution(lateral_axis_plastochron_scale, 0.1f, 5.0f);
    clamp_distribution(lateral_bud_plastochron_scale, 0.1f, 5.0f);
    clamp_distribution(maturity_initiation_coupling, 0.0f, 2.0f);
    clamp_distribution(reference_maturity_gdd, 1.0f, 4000.0f);
    clamp_distribution(branch_angle_relaxation, 0.001f, 1.0f);
    clamp_distribution(pair_angle_relaxation, 0.001f, 1.0f);

    clamp_distribution(stage_1_end_t, 0.0f, 1.0f);
    stage_2_end_t.mean = std::clamp(stage_2_end_t.mean, std::min(1.0f, stage_1_end_t.mean + 0.02f), 1.0f);
    stage_2_end_t.deviation = std::max(0.0f, stage_2_end_t.deviation);
    stage_3_end_t.mean = std::clamp(stage_3_end_t.mean, std::min(1.0f, stage_2_end_t.mean + 0.02f), 1.0f);
    stage_3_end_t.deviation = std::max(0.0f, stage_3_end_t.deviation);

    clamp_distribution(secondary_ramp_start_t, 0.0f, 1.0f);
    secondary_ramp_end_t.mean = std::clamp(
      secondary_ramp_end_t.mean,
      std::min(1.0f, secondary_ramp_start_t.mean + 0.02f),
      1.0f);
    secondary_ramp_end_t.deviation = std::max(0.0f, secondary_ramp_end_t.deviation);

    clamp_distribution(mature_droop_start_t, 0.0f, 1.0f);
    clamp_distribution(mature_droop_strength, 0.0f, 20.0f);
    final_age_gdd.mean = std::max(1.0f, final_age_gdd.mean);
    final_age_gdd.deviation = std::max(0.0f, final_age_gdd.deviation);

    ImGui::TreePop();
  }

  // -- Live preview: coalesce edits and regenerate matching tassels at capped cadence --
  if (changed && live_preview) {
    live_preview_dirty_ = true;
    live_preview_request_count_++;
  }

  const bool drag_active = ImGui::IsMouseDown(ImGuiMouseButton_Left) && ImGui::IsAnyItemActive();
  const bool drag_ended = live_preview_was_dragging_ && !drag_active;
  live_preview_was_dragging_ = live_preview && drag_active;

  if (drag_ended && live_preview && live_preview_needs_full_apply_) {
    // Commit drag-session preview edits to every matching tassel when the drag ends.
    live_preview_dirty_ = true;
  }

  if (changed || editor_preferences_changed) {
    SetUnsaved();
  }

  if (live_preview && live_preview_dirty_) {
    live_preview_rate_hz = std::clamp(live_preview_rate_hz, 1.0f, 60.0f);
    const double now_seconds = GetSteadyTimeSeconds();
    const double min_interval_seconds = 1.0 / static_cast<double>(live_preview_rate_hz);

    const bool throttle_ready = live_preview_last_apply_seconds_ < 0.0 ||
                                (now_seconds - live_preview_last_apply_seconds_) >= min_interval_seconds;

    if (!drag_active || throttle_ready) {
      const double apply_start_seconds = GetSteadyTimeSeconds();
      bool applied_any = false;
      const auto scene = Application::GetActiveScene();
      if (scene) {
        const auto* tassel_entities_ptr = scene->UnsafeGetPrivateComponentOwnersList<MaizeTassel>();
        if (tassel_entities_ptr) {
          const std::vector<Entity> tassel_entities = *tassel_entities_ptr;
          if (drag_active) {
            bool preview_applied = false;
            const uint32_t preview_step_cap = static_cast<uint32_t>(
                std::max(1, live_preview_max_growth_steps));
            const float preview_target_cap = std::max(0.0f, live_preview_max_gdd);

            for (const auto& entity : tassel_entities) {
              if (!scene->IsEntityValid(entity))
                continue;
              auto tassel = scene->GetOrSetPrivateComponent<MaizeTassel>(entity).lock();
              if (!tassel)
                continue;
              if (tassel->descriptor_ref.Get<MaizeTasselDescriptor>().get() != this)
                continue;

              tassel->target_gdd = SampleTargetGddForSeed(target_gdd, tassel->seed);

              const float preview_target_gdd = live_preview_cap_target_gdd
                  ? std::min(tassel->target_gdd, preview_target_cap)
                  : tassel->target_gdd;
              tassel->GeneratePreviewGeometryEntities(preview_target_gdd, preview_step_cap);
              preview_applied = true;
              applied_any = true;

              if (live_preview_representative_only)
                break;
            }

            if (preview_applied) {
              live_preview_needs_full_apply_ = true;
            }
          } else {
            for (const auto& entity : tassel_entities) {
              if (!scene->IsEntityValid(entity))
                continue;
              auto tassel = scene->GetOrSetPrivateComponent<MaizeTassel>(entity).lock();
              if (!tassel)
                continue;
              if (tassel->descriptor_ref.Get<MaizeTasselDescriptor>().get() != this)
                continue;

              tassel->target_gdd = SampleTargetGddForSeed(target_gdd, tassel->seed);
              tassel->GenerateGeometryEntities(true);
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
// Serialization
// ---------------------------------------------------------------------------

void MaizeTasselDescriptor::Serialize(YAML::Emitter& out) const {
  // Branch zone.
  branch_node_count.Save("branch_node_count", out);
  branch_internode_length.Save("branch_internode_length", out);
  branch_internode_thickness.Save("branch_internode_thickness", out);
  lateral_insertion_angle.Save("lateral_insertion_angle", out);
  lateral_internode_length.Save("lateral_internode_length", out);
  lateral_node_count.Save("lateral_node_count", out);
  peduncle_branch_probability.Save("peduncle_branch_probability", out);

  // Central spike.
  spike_node_count.Save("spike_node_count", out);
  spike_internode_length.Save("spike_internode_length", out);
  spike_internode_thickness.Save("spike_internode_thickness", out);
  spike_zone_branch_probability.Save("spike_zone_branch_probability", out);

  // Main-rachis pair morphology.
  main_pair_proximal_scale_x.Save("main_pair_proximal_scale_x", out);
  main_pair_proximal_scale_y.Save("main_pair_proximal_scale_y", out);
  main_pair_proximal_scale_z.Save("main_pair_proximal_scale_z", out);
  main_pair_proximal_angle.Save("main_pair_proximal_angle", out);
  main_pair_internode_length.Save("main_pair_internode_length", out);
  main_pair_internode_thickness.Save("main_pair_internode_thickness", out);
  main_pair_internode_angle.Save("main_pair_internode_angle", out);
  main_pair_distal_scale_x.Save("main_pair_distal_scale_x", out);
  main_pair_distal_scale_y.Save("main_pair_distal_scale_y", out);
  main_pair_distal_scale_z.Save("main_pair_distal_scale_z", out);
  main_pair_distal_angle.Save("main_pair_distal_angle", out);

  // Non-main-axis pair morphology.
  branch_pair_proximal_scale_x.Save("branch_pair_proximal_scale_x", out);
  branch_pair_proximal_scale_y.Save("branch_pair_proximal_scale_y", out);
  branch_pair_proximal_scale_z.Save("branch_pair_proximal_scale_z", out);
  branch_pair_proximal_angle.Save("branch_pair_proximal_angle", out);
  branch_pair_internode_length.Save("branch_pair_internode_length", out);
  branch_pair_internode_thickness.Save("branch_pair_internode_thickness", out);
  branch_pair_internode_angle.Save("branch_pair_internode_angle", out);
  branch_pair_distal_scale_x.Save("branch_pair_distal_scale_x", out);
  branch_pair_distal_scale_y.Save("branch_pair_distal_scale_y", out);
  branch_pair_distal_scale_z.Save("branch_pair_distal_scale_z", out);
  branch_pair_distal_angle.Save("branch_pair_distal_angle", out);

  // Branch timing and probabilities.
  lateral_initiation_delay_gdd.Save("lateral_initiation_delay_gdd", out);
  spike_anthesis_offset_gdd.Save("spike_anthesis_offset_gdd", out);
  primary_lateral_branch_probability.Save("primary_lateral_branch_probability", out);
  secondary_lateral_branch_probability.Save("secondary_lateral_branch_probability", out);

  // Shared.
  phyllotaxis_angle.Save("phyllotaxis_angle", out);
  branch_azimuth_offset.Save("branch_azimuth_offset", out);
  lateral_thickness_ratio.Save("lateral_thickness_ratio", out);
  final_age_gdd.Save("final_age_gdd", out);

  secondary_insertion_angle.Save("secondary_insertion_angle", out);
  secondary_internode_length.Save("secondary_internode_length", out);
  secondary_internode_thickness.Save("secondary_internode_thickness", out);
  secondary_node_count.Save("secondary_node_count", out);

  rachis_elongation_curve.Save("rachis_elongation_curve", out);
  rachis_thickness_curve.Save("rachis_thickness_curve", out);
  lateral_elongation_curve.Save("lateral_elongation_curve", out);
  lateral_thickness_curve.Save("lateral_thickness_curve", out);
  lateral_angle_development_curve.Save("lateral_angle_development_curve", out);
  pair_proximal_scale_curve.Save("pair_proximal_scale_curve", out);
  pair_proximal_angle_curve.Save("pair_proximal_angle_curve", out);
  pair_internode_length_curve.Save("pair_internode_length_curve", out);
  pair_internode_thickness_curve.Save("pair_internode_thickness_curve", out);
  pair_internode_angle_curve.Save("pair_internode_angle_curve", out);
  pair_distal_scale_curve.Save("pair_distal_scale_curve", out);
  pair_distal_angle_curve.Save("pair_distal_angle_curve", out);

  target_gdd.Save("target_gdd", out);
  gdd_per_day.Save("gdd_per_day", out);
  plastochron_gdd.Save("plastochron_gdd", out);
  anthesis_gdd.Save("anthesis_gdd", out);
  maturity_gdd.Save("max_gdd", out);
  main_axis_plastochron_scale.Save("main_axis_plastochron_scale", out);
  lateral_axis_plastochron_scale.Save("lateral_axis_plastochron_scale", out);
  lateral_bud_plastochron_scale.Save("lateral_bud_plastochron_scale", out);
  maturity_initiation_coupling.Save("maturity_initiation_coupling", out);
  reference_maturity_gdd.Save("reference_maturity_gdd", out);
  branch_angle_relaxation.Save("branch_angle_relaxation", out);
  pair_angle_relaxation.Save("pair_angle_relaxation", out);
  stage_1_end_t.Save("stage_1_end_t", out);
  stage_2_end_t.Save("stage_2_end_t", out);
  stage_3_end_t.Save("stage_3_end_t", out);
  secondary_ramp_start_t.Save("secondary_ramp_start_t", out);
  secondary_ramp_end_t.Save("secondary_ramp_end_t", out);
  mature_droop_start_t.Save("mature_droop_start_t", out);
  mature_droop_strength.Save("mature_droop_strength", out);

  out << YAML::Key << "live_preview" << YAML::Value << live_preview;
  out << YAML::Key << "live_preview_rate_hz" << YAML::Value << live_preview_rate_hz;
  out << YAML::Key << "live_preview_representative_only" << YAML::Value
      << live_preview_representative_only;
  out << YAML::Key << "live_preview_cap_target_gdd" << YAML::Value
      << live_preview_cap_target_gdd;
  out << YAML::Key << "live_preview_max_gdd" << YAML::Value << live_preview_max_gdd;
  out << YAML::Key << "live_preview_max_growth_steps" << YAML::Value
      << live_preview_max_growth_steps;
  out << YAML::Key << "grid_rows" << YAML::Value << grid_rows;
  out << YAML::Key << "grid_cols" << YAML::Value << grid_cols;
  out << YAML::Key << "grid_spacing" << YAML::Value << grid_spacing;

  // Explorer preferences.
  out << YAML::Key << "explorer_mode" << YAML::Value << static_cast<int>(explorer_.mode);
  out << YAML::Key << "explorer_speed" << YAML::Value << explorer_.speed;

  // Tropism array.
  out << YAML::Key << "tropism_count" << YAML::Value << static_cast<int>(tropisms.size());
  for (int i = 0; i < static_cast<int>(tropisms.size()); i++) {
    const std::string prefix = "tropism_" + std::to_string(i) + "_";
    tropisms[i].direction_x.Save(prefix + "dir_x", out);
    tropisms[i].direction_y.Save(prefix + "dir_y", out);
    tropisms[i].direction_z.Save(prefix + "dir_z", out);
    tropisms[i].strength.Save(prefix + "strength", out);
    out << YAML::Key << prefix + "usage_chance_percent" << YAML::Value
        << std::clamp(tropisms[i].usage_chance_percent, 0.0f, 100.0f);
    tropisms[i].order_response.Save(prefix + "order_response", out);
  }
}

void MaizeTasselDescriptor::Deserialize(const YAML::Node& in) {
  // Branch zone.
  branch_node_count.Load("branch_node_count", in);
  branch_internode_length.Load("branch_internode_length", in);
  branch_internode_thickness.Load("branch_internode_thickness", in);
  lateral_insertion_angle.Load("lateral_insertion_angle", in);
  lateral_internode_length.Load("lateral_internode_length", in);
  lateral_node_count.Load("lateral_node_count", in);
  peduncle_branch_probability.Load("peduncle_branch_probability", in);
  if (in["primary_branch_probability"]) {
    peduncle_branch_probability.Load("primary_branch_probability", in);
  }

  // Central spike.
  spike_node_count.Load("spike_node_count", in);
  spike_internode_length.Load("spike_internode_length", in);
  spike_internode_thickness.Load("spike_internode_thickness", in);
  spike_zone_branch_probability.Load("spike_zone_branch_probability", in);
  if (in["primary_branch_probability"]) {
    spike_zone_branch_probability.Load("primary_branch_probability", in);
  }

  // Main-rachis pair morphology.
  main_pair_proximal_scale_x.Load("main_pair_proximal_scale_x", in);
  main_pair_proximal_scale_y.Load("main_pair_proximal_scale_y", in);
  main_pair_proximal_scale_z.Load("main_pair_proximal_scale_z", in);
  main_pair_proximal_angle.Load("main_pair_proximal_angle", in);
  main_pair_internode_length.Load("main_pair_internode_length", in);
  main_pair_internode_thickness.Load("main_pair_internode_thickness", in);
  main_pair_internode_angle.Load("main_pair_internode_angle", in);
  main_pair_distal_scale_x.Load("main_pair_distal_scale_x", in);
  main_pair_distal_scale_y.Load("main_pair_distal_scale_y", in);
  main_pair_distal_scale_z.Load("main_pair_distal_scale_z", in);
  main_pair_distal_angle.Load("main_pair_distal_angle", in);

  // Non-main-axis pair morphology.
  branch_pair_proximal_scale_x.Load("branch_pair_proximal_scale_x", in);
  branch_pair_proximal_scale_y.Load("branch_pair_proximal_scale_y", in);
  branch_pair_proximal_scale_z.Load("branch_pair_proximal_scale_z", in);
  branch_pair_proximal_angle.Load("branch_pair_proximal_angle", in);
  branch_pair_internode_length.Load("branch_pair_internode_length", in);
  branch_pair_internode_thickness.Load("branch_pair_internode_thickness", in);
  branch_pair_internode_angle.Load("branch_pair_internode_angle", in);
  branch_pair_distal_scale_x.Load("branch_pair_distal_scale_x", in);
  branch_pair_distal_scale_y.Load("branch_pair_distal_scale_y", in);
  branch_pair_distal_scale_z.Load("branch_pair_distal_scale_z", in);
  branch_pair_distal_angle.Load("branch_pair_distal_angle", in);

  // Branch timing and branch probabilities.
  lateral_initiation_delay_gdd.Load("lateral_initiation_delay_gdd", in);
  spike_anthesis_offset_gdd.Load("spike_anthesis_offset_gdd", in);
  primary_lateral_branch_probability.Load("primary_lateral_branch_probability", in);
  secondary_lateral_branch_probability.Load("secondary_lateral_branch_probability", in);
  if (in["secondary_probability"]) {
    primary_lateral_branch_probability.Load("secondary_probability", in);
    secondary_lateral_branch_probability.Load("secondary_probability", in);
  }

  // Shared.
  phyllotaxis_angle.Load("phyllotaxis_angle", in);
  branch_azimuth_offset.Load("branch_azimuth_offset", in);
  lateral_thickness_ratio.Load("lateral_thickness_ratio", in);
  final_age_gdd.Load("final_age_gdd", in);

  secondary_insertion_angle.Load("secondary_insertion_angle", in);
  secondary_internode_length.Load("secondary_internode_length", in);
  secondary_internode_thickness.Load("secondary_internode_thickness", in);
  secondary_node_count.Load("secondary_node_count", in);

  rachis_elongation_curve.Load("rachis_elongation_curve", in);
  rachis_thickness_curve.Load("rachis_thickness_curve", in);
  lateral_elongation_curve.Load("lateral_elongation_curve", in);
  lateral_thickness_curve.Load("lateral_thickness_curve", in);
  lateral_angle_development_curve.Load("lateral_angle_development_curve", in);
  pair_proximal_scale_curve.Load("pair_proximal_scale_curve", in);
  pair_proximal_angle_curve.Load("pair_proximal_angle_curve", in);
  pair_internode_length_curve.Load("pair_internode_length_curve", in);
  pair_internode_thickness_curve.Load("pair_internode_thickness_curve", in);
  pair_internode_angle_curve.Load("pair_internode_angle_curve", in);
  pair_distal_scale_curve.Load("pair_distal_scale_curve", in);
  pair_distal_angle_curve.Load("pair_distal_angle_curve", in);

  // Legacy growth-curve mappings.
  if (in["spikelet_scale_curve"]) {
    pair_proximal_scale_curve.Load("spikelet_scale_curve", in);
    pair_distal_scale_curve.Load("spikelet_scale_curve", in);
  }
  if (in["spikelet_pedicel_curve"]) {
    pair_internode_length_curve.Load("spikelet_pedicel_curve", in);
    pair_internode_thickness_curve.Load("spikelet_pedicel_curve", in);
  }
  if (in["spikelet_outward_curve"]) {
    pair_proximal_angle_curve.Load("spikelet_outward_curve", in);
    pair_internode_angle_curve.Load("spikelet_outward_curve", in);
    pair_distal_angle_curve.Load("spikelet_outward_curve", in);
  }

  // Legacy fallback mappings.
  if (in["spike_spikelet_scale"]) {
    main_pair_proximal_scale_x.Load("spike_spikelet_scale", in);
    main_pair_proximal_scale_y.Load("spike_spikelet_scale", in);
    main_pair_proximal_scale_z.Load("spike_spikelet_scale", in);
    main_pair_distal_scale_x.Load("spike_spikelet_scale", in);
    main_pair_distal_scale_y.Load("spike_spikelet_scale", in);
    main_pair_distal_scale_z.Load("spike_spikelet_scale", in);
  }
  if (in["lateral_spikelet_scale"]) {
    branch_pair_proximal_scale_x.Load("lateral_spikelet_scale", in);
    branch_pair_proximal_scale_y.Load("lateral_spikelet_scale", in);
    branch_pair_proximal_scale_z.Load("lateral_spikelet_scale", in);
    branch_pair_distal_scale_x.Load("lateral_spikelet_scale", in);
    branch_pair_distal_scale_y.Load("lateral_spikelet_scale", in);
    branch_pair_distal_scale_z.Load("lateral_spikelet_scale", in);
  }
  if (in["spike_pedicel_length"]) {
    main_pair_internode_length.Load("spike_pedicel_length", in);
  }
  if (in["lateral_pedicel_length"]) {
    branch_pair_internode_length.Load("lateral_pedicel_length", in);
  }
  if (in["spikelet_outward_angle"]) {
    main_pair_proximal_angle.Load("spikelet_outward_angle", in);
    main_pair_internode_angle.Load("spikelet_outward_angle", in);
    main_pair_distal_angle.Load("spikelet_outward_angle", in);
    branch_pair_proximal_angle.Load("spikelet_outward_angle", in);
    branch_pair_internode_angle.Load("spikelet_outward_angle", in);
    branch_pair_distal_angle.Load("spikelet_outward_angle", in);
  }

  if (in["breakage_threshold"]) {
    // Legacy compatibility: keep loading old assets that still contain this key.
  }

  LoadSingleDistributionWithScalarFallback(in, "target_gdd", target_gdd);
  if (in["gdd_per_day"]) {
    LoadSingleDistributionWithScalarFallback(in, "gdd_per_day", gdd_per_day);
  } else {
    // Legacy compatibility for assets authored before descriptor thermal-rate migration.
    LoadSingleDistributionWithScalarFallback(in, "gdd_per_second", gdd_per_day);
  }
  LoadSingleDistributionWithScalarFallback(in, "plastochron_gdd", plastochron_gdd);
  LoadSingleDistributionWithScalarFallback(in, "anthesis_gdd", anthesis_gdd);
  if (in["max_gdd"]) {
    if (in["max_gdd"].IsMap()) {
      maturity_gdd.Load("max_gdd", in);
    } else if (in["max_gdd"].IsScalar()) {
      maturity_gdd.mean = in["max_gdd"].as<float>();
      maturity_gdd.deviation = 0.0f;
    }
  } else {
    LoadSingleDistributionWithScalarFallback(in, "maturity_gdd", maturity_gdd);
  }
  LoadSingleDistributionWithScalarFallback(in, "main_axis_plastochron_scale", main_axis_plastochron_scale);
  LoadSingleDistributionWithScalarFallback(in, "lateral_axis_plastochron_scale", lateral_axis_plastochron_scale);
  LoadSingleDistributionWithScalarFallback(in, "lateral_bud_plastochron_scale", lateral_bud_plastochron_scale);
  LoadSingleDistributionWithScalarFallback(in, "maturity_initiation_coupling", maturity_initiation_coupling);
  LoadSingleDistributionWithScalarFallback(in, "reference_maturity_gdd", reference_maturity_gdd);
  LoadSingleDistributionWithScalarFallback(in, "branch_angle_relaxation", branch_angle_relaxation);
  LoadSingleDistributionWithScalarFallback(in, "pair_angle_relaxation", pair_angle_relaxation);
  LoadSingleDistributionWithScalarFallback(in, "stage_1_end_t", stage_1_end_t);
  LoadSingleDistributionWithScalarFallback(in, "stage_2_end_t", stage_2_end_t);
  LoadSingleDistributionWithScalarFallback(in, "stage_3_end_t", stage_3_end_t);
  LoadSingleDistributionWithScalarFallback(in, "secondary_ramp_start_t", secondary_ramp_start_t);
  LoadSingleDistributionWithScalarFallback(in, "secondary_ramp_end_t", secondary_ramp_end_t);
  LoadSingleDistributionWithScalarFallback(in, "mature_droop_start_t", mature_droop_start_t);
  LoadSingleDistributionWithScalarFallback(in, "mature_droop_strength", mature_droop_strength);

  target_gdd.mean = std::clamp(target_gdd.mean, 0.0f, 4000.0f);
  target_gdd.deviation = std::max(0.0f, target_gdd.deviation);
  gdd_per_day.mean = std::clamp(gdd_per_day.mean, 0.0f, 500.0f);
  gdd_per_day.deviation = std::max(0.0f, gdd_per_day.deviation);
  plastochron_gdd.mean = std::max(1.0f, plastochron_gdd.mean);
  plastochron_gdd.deviation = std::max(0.0f, plastochron_gdd.deviation);
  anthesis_gdd.mean = std::max(0.0f, anthesis_gdd.mean);
  anthesis_gdd.deviation = std::max(0.0f, anthesis_gdd.deviation);
  maturity_gdd.mean = std::max(anthesis_gdd.mean + 1.0f, maturity_gdd.mean);
  maturity_gdd.deviation = std::max(0.0f, maturity_gdd.deviation);
  main_axis_plastochron_scale.mean = std::max(0.1f, main_axis_plastochron_scale.mean);
  main_axis_plastochron_scale.deviation = std::max(0.0f, main_axis_plastochron_scale.deviation);
  lateral_axis_plastochron_scale.mean = std::max(0.1f, lateral_axis_plastochron_scale.mean);
  lateral_axis_plastochron_scale.deviation = std::max(0.0f, lateral_axis_plastochron_scale.deviation);
  lateral_bud_plastochron_scale.mean = std::max(0.1f, lateral_bud_plastochron_scale.mean);
  lateral_bud_plastochron_scale.deviation = std::max(0.0f, lateral_bud_plastochron_scale.deviation);
  maturity_initiation_coupling.mean = std::max(0.0f, maturity_initiation_coupling.mean);
  maturity_initiation_coupling.deviation = std::max(0.0f, maturity_initiation_coupling.deviation);
  reference_maturity_gdd.mean = std::max(1.0f, reference_maturity_gdd.mean);
  reference_maturity_gdd.deviation = std::max(0.0f, reference_maturity_gdd.deviation);
  branch_angle_relaxation.mean = std::clamp(branch_angle_relaxation.mean, 0.001f, 1.0f);
  branch_angle_relaxation.deviation = std::max(0.0f, branch_angle_relaxation.deviation);
  pair_angle_relaxation.mean = std::clamp(pair_angle_relaxation.mean, 0.001f, 1.0f);
  pair_angle_relaxation.deviation = std::max(0.0f, pair_angle_relaxation.deviation);
  stage_1_end_t.mean = std::clamp(stage_1_end_t.mean, 0.0f, 1.0f);
  stage_1_end_t.deviation = std::max(0.0f, stage_1_end_t.deviation);
  stage_2_end_t.mean = std::clamp(stage_2_end_t.mean, std::min(1.0f, stage_1_end_t.mean + 0.02f), 1.0f);
  stage_2_end_t.deviation = std::max(0.0f, stage_2_end_t.deviation);
  stage_3_end_t.mean = std::clamp(stage_3_end_t.mean, std::min(1.0f, stage_2_end_t.mean + 0.02f), 1.0f);
  stage_3_end_t.deviation = std::max(0.0f, stage_3_end_t.deviation);
  secondary_ramp_start_t.mean = std::clamp(secondary_ramp_start_t.mean, 0.0f, 1.0f);
  secondary_ramp_start_t.deviation = std::max(0.0f, secondary_ramp_start_t.deviation);
  secondary_ramp_end_t.mean = std::clamp(
      secondary_ramp_end_t.mean,
      std::min(1.0f, secondary_ramp_start_t.mean + 0.02f),
      1.0f);
  secondary_ramp_end_t.deviation = std::max(0.0f, secondary_ramp_end_t.deviation);
  mature_droop_start_t.mean = std::clamp(mature_droop_start_t.mean, 0.0f, 1.0f);
  mature_droop_start_t.deviation = std::max(0.0f, mature_droop_start_t.deviation);
  mature_droop_strength.mean = std::max(0.0f, mature_droop_strength.mean);
  mature_droop_strength.deviation = std::max(0.0f, mature_droop_strength.deviation);

  if (in["live_preview"]) live_preview = in["live_preview"].as<bool>();
  if (in["live_preview_rate_hz"]) live_preview_rate_hz = in["live_preview_rate_hz"].as<float>();
  live_preview_rate_hz = std::clamp(live_preview_rate_hz, 1.0f, 60.0f);
  if (in["live_preview_representative_only"]) {
    live_preview_representative_only = in["live_preview_representative_only"].as<bool>();
  }
  if (in["live_preview_cap_target_gdd"]) {
    live_preview_cap_target_gdd = in["live_preview_cap_target_gdd"].as<bool>();
  }
  if (in["live_preview_max_gdd"]) {
    live_preview_max_gdd = in["live_preview_max_gdd"].as<float>();
  }
  live_preview_max_gdd = std::max(0.0f, live_preview_max_gdd);
  if (in["live_preview_max_growth_steps"]) {
    live_preview_max_growth_steps = in["live_preview_max_growth_steps"].as<int>();
  }
  live_preview_max_growth_steps = std::clamp(live_preview_max_growth_steps, 1, 10000);
  if (in["grid_rows"]) grid_rows = in["grid_rows"].as<int>();
  if (in["grid_cols"]) grid_cols = in["grid_cols"].as<int>();
  if (in["grid_spacing"]) grid_spacing = in["grid_spacing"].as<float>();

  // Explorer preferences.
  if (in["explorer_mode"]) {
    int m = in["explorer_mode"].as<int>();
    if (m >= 0 && m <= 3) explorer_.mode = static_cast<ParamMotionMode>(m);
  }
  if (in["explorer_speed"]) {
    explorer_.speed = std::clamp(in["explorer_speed"].as<float>(), 0.01f, 10.0f);
  }

  // Tropism array.
  tropisms.clear();
  if (in["tropism_count"]) {
    const int count = in["tropism_count"].as<int>();
    for (int i = 0; i < count; i++) {
      TropismEntry entry;
      const std::string prefix = "tropism_" + std::to_string(i) + "_";
      entry.direction_x.Load(prefix + "dir_x", in);
      entry.direction_y.Load(prefix + "dir_y", in);
      entry.direction_z.Load(prefix + "dir_z", in);
      entry.strength.Load(prefix + "strength", in);
      if (in[prefix + "usage_chance_percent"]) {
        entry.usage_chance_percent =
            std::clamp(in[prefix + "usage_chance_percent"].as<float>(), 0.0f, 100.0f);
      }
      entry.order_response.Load(prefix + "order_response", in);
      tropisms.push_back(std::move(entry));
    }
  }
}

// ---------------------------------------------------------------------------
// ILSystemExplorableDescriptor: enumerate every tunable field for the
// generic ParamSpaceExplorer panel. Lifted verbatim (incl. label keys) from
// the previous MaizeTassel-specific ParamSpaceExplorer::RebuildAxes body so
// axis ordering, ranges, and schema signatures stay bit-identical.
// ---------------------------------------------------------------------------
void MaizeTasselDescriptor::RegisterExplorableAxes(ParamSpaceExplorer& explorer) {
  auto& d = *this;

  // Single distributions (mean + deviation).
  explorer.AddSingle("branch_node_count", "BNC", d.branch_node_count, 0.0f, 80.0f, 20.0f);
  explorer.AddSingle("spike_node_count", "SNC", d.spike_node_count, 0.0f, 120.0f, 30.0f);
  explorer.AddSingle("phyllotaxis_angle", "PHY", d.phyllotaxis_angle, 0.0f, 360.0f, 180.0f);
  explorer.AddSingle("branch_azimuth_offset", "BAO", d.branch_azimuth_offset, -180.0f, 180.0f, 180.0f);
  explorer.AddSingle("lateral_thickness_ratio", "LTR", d.lateral_thickness_ratio, 0.0f, 2.0f, 1.0f);
  explorer.AddSingle("secondary_insertion_angle", "SIA", d.secondary_insertion_angle, 0.0f, 120.0f, 60.0f);
  explorer.AddSingle("secondary_internode_length", "SIL", d.secondary_internode_length, 0.0f, 20.0f, 10.0f);
  explorer.AddSingle("secondary_internode_thickness", "SIT", d.secondary_internode_thickness, 0.0f, 2.0f, 1.0f);
  explorer.AddSingle("secondary_node_count", "SNN", d.secondary_node_count, 0.0f, 20.0f, 10.0f);
  explorer.AddSingle("final_age_gdd", "FAG", d.final_age_gdd, 0.0f, 3000.0f, 1500.0f);

  // Global development distributions.
  explorer.AddSingle("target_gdd", "TGD", d.target_gdd, 0.0f, 5000.0f, 2500.0f);
  explorer.AddSingle("gdd_per_day", "GPD", d.gdd_per_day, 0.0f, 500.0f, 500.0f);
  explorer.AddSingle("plastochron_gdd", "PGD", d.plastochron_gdd, 1.0f, 500.0f, 250.0f);
  explorer.AddSingle("anthesis_gdd", "AGD", d.anthesis_gdd, 0.0f, 3000.0f, 1500.0f);
  explorer.AddSingle("maturity_gdd", "MGD", d.maturity_gdd, 0.0f, 5000.0f, 2500.0f);

  // All plotted distributions: ranges + all curve control points.
  explorer.AddPlotted("branch_internode_length", "BIL", d.branch_internode_length);
  explorer.AddPlotted("branch_internode_thickness", "BIT", d.branch_internode_thickness);
  explorer.AddPlotted("lateral_insertion_angle", "LIA", d.lateral_insertion_angle);
  explorer.AddPlotted("lateral_internode_length", "LIL", d.lateral_internode_length);
  explorer.AddPlotted("lateral_node_count", "LNC", d.lateral_node_count);
  explorer.AddPlotted("peduncle_branch_probability", "PBP", d.peduncle_branch_probability);
  explorer.AddPlotted("spike_internode_length", "SIL", d.spike_internode_length);
  explorer.AddPlotted("spike_internode_thickness", "SIT", d.spike_internode_thickness);
  explorer.AddPlotted("spike_zone_branch_probability", "SZP", d.spike_zone_branch_probability);

  explorer.AddPlotted("main_pair_proximal_scale_x", "MPX", d.main_pair_proximal_scale_x);
  explorer.AddPlotted("main_pair_proximal_scale_y", "MPY", d.main_pair_proximal_scale_y);
  explorer.AddPlotted("main_pair_proximal_scale_z", "MPZ", d.main_pair_proximal_scale_z);
  explorer.AddPlotted("main_pair_proximal_angle", "MPA", d.main_pair_proximal_angle);
  explorer.AddPlotted("main_pair_internode_length", "MIL", d.main_pair_internode_length);
  explorer.AddPlotted("main_pair_internode_thickness", "MIT", d.main_pair_internode_thickness);
  explorer.AddPlotted("main_pair_internode_angle", "MIA", d.main_pair_internode_angle);
  explorer.AddPlotted("main_pair_distal_scale_x", "MDX", d.main_pair_distal_scale_x);
  explorer.AddPlotted("main_pair_distal_scale_y", "MDY", d.main_pair_distal_scale_y);
  explorer.AddPlotted("main_pair_distal_scale_z", "MDZ", d.main_pair_distal_scale_z);
  explorer.AddPlotted("main_pair_distal_angle", "MDA", d.main_pair_distal_angle);

  explorer.AddPlotted("branch_pair_proximal_scale_x", "BPX", d.branch_pair_proximal_scale_x);
  explorer.AddPlotted("branch_pair_proximal_scale_y", "BPY", d.branch_pair_proximal_scale_y);
  explorer.AddPlotted("branch_pair_proximal_scale_z", "BPZ", d.branch_pair_proximal_scale_z);
  explorer.AddPlotted("branch_pair_proximal_angle", "BPA", d.branch_pair_proximal_angle);
  explorer.AddPlotted("branch_pair_internode_length", "BIL", d.branch_pair_internode_length);
  explorer.AddPlotted("branch_pair_internode_thickness", "BIT", d.branch_pair_internode_thickness);
  explorer.AddPlotted("branch_pair_internode_angle", "BIA", d.branch_pair_internode_angle);
  explorer.AddPlotted("branch_pair_distal_scale_x", "BDX", d.branch_pair_distal_scale_x);
  explorer.AddPlotted("branch_pair_distal_scale_y", "BDY", d.branch_pair_distal_scale_y);
  explorer.AddPlotted("branch_pair_distal_scale_z", "BDZ", d.branch_pair_distal_scale_z);
  explorer.AddPlotted("branch_pair_distal_angle", "BDA", d.branch_pair_distal_angle);

  explorer.AddPlotted("lateral_initiation_delay_gdd", "LID", d.lateral_initiation_delay_gdd);
  explorer.AddPlotted("spike_anthesis_offset_gdd", "SAO", d.spike_anthesis_offset_gdd);
  explorer.AddPlotted("primary_lateral_branch_probability", "PLP", d.primary_lateral_branch_probability);
  explorer.AddPlotted("secondary_lateral_branch_probability", "SLP", d.secondary_lateral_branch_probability);

  // Direct curve axes.
  explorer.AddCurve("rachis_elongation_curve", "REC", d.rachis_elongation_curve);
  explorer.AddCurve("rachis_thickness_curve", "RTC", d.rachis_thickness_curve);
  explorer.AddCurve("lateral_elongation_curve", "LEC", d.lateral_elongation_curve);
  explorer.AddCurve("lateral_thickness_curve", "LTC", d.lateral_thickness_curve);
  explorer.AddCurve("lateral_angle_development_curve", "LAC", d.lateral_angle_development_curve);
  explorer.AddCurve("pair_proximal_scale_curve", "PPS", d.pair_proximal_scale_curve);
  explorer.AddCurve("pair_proximal_angle_curve", "PPA", d.pair_proximal_angle_curve);
  explorer.AddCurve("pair_internode_length_curve", "PIL", d.pair_internode_length_curve);
  explorer.AddCurve("pair_internode_thickness_curve", "PIT", d.pair_internode_thickness_curve);
  explorer.AddCurve("pair_internode_angle_curve", "PIA", d.pair_internode_angle_curve);
  explorer.AddCurve("pair_distal_scale_curve", "PDS", d.pair_distal_scale_curve);
  explorer.AddCurve("pair_distal_angle_curve", "PDA", d.pair_distal_angle_curve);

  // Dynamic tropism dimensions.
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

