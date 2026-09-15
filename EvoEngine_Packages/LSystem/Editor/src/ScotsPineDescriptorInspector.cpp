#include "CurveEditors.hpp"
#include "EditorLayer.hpp"
#include "EditorPackage.hpp"
#include "LSystemInspectionAdapters.hpp"
#include "ParamSpaceExplorer.hpp"
#include "Scene.hpp"
#include "ScotsPine.hpp"
#include "ScotsPineDescriptor.hpp"
#include "Transform.hpp"

using namespace evo_engine;
using namespace l_system_package;

namespace {
class ScotsPineDescriptorEditor final : public ILSystemExplorableDescriptor {
  ScotsPineDescriptor& target_;
  ParamSpaceExplorer explorer_;
  int selected_maturity_variable = 0;
  int s_selected_shape_preset = 0;

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

 public:
  explicit ScotsPineDescriptorEditor(ScotsPineDescriptor& target) : target_(target) {
  }
  bool Inspect(InspectorContext& context);
  // ===== ILSystemExplorableDescriptor =====
  void RegisterExplorableAxes(ParamSpaceExplorer& explorer) override;
  uint64_t ExplorableSchemaFingerprint() const override {
    // Bump when the explorable axis schema changes shape (added/removed
    // axes). The dynamic tropism count is folded in to keep the existing
    // contract that adding tropism entries also invalidates cached layouts.
    // Constant 0x4E45454458534537 spells "NEEDXSE7".
    // change.
    return 0x4E45454458534537ull ^ static_cast<uint64_t>(target_.tropisms.size());
  }
};

bool ScotsPineDescriptorEditor::Inspect(InspectorContext& context) {
  const auto& editor_layer = context.editor_layer;
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
    editor_layer->SetSelectedEntity(target_.Instantiate());
  }
  show_item_hover_description("Create a new ScotsPine entity using this descriptor and select it in the scene.");

  ImGui::SameLine();
  if (ImGui::Checkbox("Live Preview", &target_.live_preview)) {
    editor_preferences_changed = true;
    if (!target_.live_preview) {
      live_preview_dirty_ = false;
      live_preview_was_dragging_ = false;
      live_preview_needs_full_apply_ = false;
    }
  }
  show_item_hover_description("Regenerate matching pines while editing this descriptor.");

  if (ImGui::DragFloat("Live Preview Rate (Hz)", &target_.live_preview_rate_hz, 0.25f, 1.0f, 60.0f, "%.1f")) {
    target_.live_preview_rate_hz = std::clamp(target_.live_preview_rate_hz, 1.0f, 60.0f);
    editor_preferences_changed = true;
  }
  show_item_hover_description("Maximum live-preview apply frequency.");

  if (ImGui::Checkbox("Representative Only While Dragging", &target_.live_preview_representative_only)) {
    editor_preferences_changed = true;
  }
  show_item_hover_description("While dragging controls, preview only one matching pine.");

  if (ImGui::Checkbox("Cap Preview Target GDD", &target_.live_preview_cap_target_gdd)) {
    editor_preferences_changed = true;
  }
  show_item_hover_description("Clamp preview simulation GDD so live updates stay fast on very mature trees.");

  if (ImGui::DragFloat("Preview Max GDD", &target_.live_preview_max_gdd, 50.0f, 0.0f, 100000.0f, "%.1f")) {
    target_.live_preview_max_gdd = std::max(0.0f, target_.live_preview_max_gdd);
    editor_preferences_changed = true;
  }
  show_item_hover_description("Upper GDD limit used when preview capping is enabled.");

  if (ImGui::DragInt("Preview Max Growth Steps", &target_.live_preview_max_growth_steps, 1.0f, 1, 10000)) {
    target_.live_preview_max_growth_steps = std::clamp(target_.live_preview_max_growth_steps, 1, 10000);
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
    ImGui::DragInt("Rows", &target_.grid_rows, 1, 1, 50);
    show_item_hover_description("Number of rows for grid instantiation.");
    ImGui::DragInt("Cols", &target_.grid_cols, 1, 1, 50);
    show_item_hover_description("Number of columns for grid instantiation.");
    ImGui::DragFloat("Spacing", &target_.grid_spacing, 0.1f, 0.5f, 50.0f);
    show_item_hover_description("World-space spacing between neighboring grid pines.");

    if (ImGui::Button("Instantiate Grid")) {
      const auto scene = target_.GetApplication().GetActiveScene();
      if (scene) {
        const auto container = scene->CreateEntity("Pine Grid");
        const float offset_y = (static_cast<float>(target_.grid_rows) - 1.0f) * target_.grid_spacing * 0.5f;
        const float offset_z = (static_cast<float>(target_.grid_cols) - 1.0f) * target_.grid_spacing * 0.5f;
        const auto base_seed =
            static_cast<unsigned int>(std::chrono::steady_clock::now().time_since_epoch().count() & 0xFFFFFFFFu);
        for (int i = 0; i < target_.grid_rows; i++) {
          for (int j = 0; j < target_.grid_cols; j++) {
            const auto entity =
                scene->CreateEntity(target_.GetTitle() + " [" + std::to_string(i) + "," + std::to_string(j) + "]");
            const auto pine = scene->GetOrSetPrivateComponent<ScotsPine>(entity).lock();
            pine->descriptor_ref = target_.GetSelf();
            pine->seed = base_seed + static_cast<unsigned int>(i * target_.grid_cols + j);
            pine->target_gdd = target_.SampleTargetGdd(pine->seed);

            scene->SetParent(entity, container, false);

            Transform transform;
            transform.SetPosition(glm::vec3(0.0f, static_cast<float>(i) * target_.grid_spacing - offset_y,
                                            static_cast<float>(j) * target_.grid_spacing - offset_z));
            scene->SetDataComponent(entity, transform);

            pine->GenerateGeometryEntities();
          }
        }
      }
    }
    show_item_hover_description("Spawn a grid of ScotsPine entities from this descriptor with unique seeds.");

    ImGui::SameLine();
    if (ImGui::Button("Delete Grid")) {
      const auto scene = target_.GetApplication().GetActiveScene();
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
            if (pine->descriptor_ref.Get<ScotsPineDescriptor>().get() == &target_) {
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
    ImGui::DragFloat("Side Length", &target_.triangle_side_length, 0.1f);
    show_item_hover_description(
        "World-space side length for an equilateral 3-pine triangle on the horizontal XZ plane.");

    if (ImGui::Button("Instantiate Triangle")) {
      const auto scene = target_.GetApplication().GetActiveScene();
      if (scene) {
        const auto container = scene->CreateEntity("Pine Triangle");
        const float side_length = target_.triangle_side_length;
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
          const auto entity = scene->CreateEntity(target_.GetTitle() + " [T" + std::to_string(i) + "]");
          const auto pine = scene->GetOrSetPrivateComponent<ScotsPine>(entity).lock();
          pine->descriptor_ref = target_.GetSelf();
          pine->seed = base_seed + static_cast<unsigned int>(i);
          pine->target_gdd = target_.SampleTargetGdd(pine->seed);

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
      const auto scene = target_.GetApplication().GetActiveScene();
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
            if (pine->descriptor_ref.Get<ScotsPineDescriptor>().get() == &target_) {
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
  if (ImGui::TreeNodeEx("Global Development", ImGuiTreeNodeFlags_DefaultOpen)) {
    changed |= editor_widgets::Draw(target_.target_gdd, "Target GDD", 1.0f,
                                    "Distribution of target GDD used by Instantiate, Grid spawn, and Triangle spawn.");
    changed |= editor_widgets::Draw(target_.plastochron_gdd, "Plastochron (GDD)", 10.0f,
                                    "Physiological time between consecutive phytomer events on an axis.");
    changed |=
        editor_widgets::Draw(target_.max_phytomers_per_seasonal_growth, "Max Phytomers per Seasonal Growth", 0.5f,
                             "Phytomers (internode + optional needle cluster) emitted "
                             "per active season before the apex pauses until next year.");

    // -- Per-pine calendar / GDD-per-day fields --
    // Consumed by LSystemLayer::SamplePineTemporalParameters() and applied
    // in the per-pine update path. delta_gdd = sampled_gdd_per_day * delta_days
    // where delta_days = chronological_days_per_second * dt.
    changed |= editor_widgets::Draw(target_.gdd_per_day, "GDD per Day", 0.1f,
                                    "Per-pine thermal accumulation rate. Sampled per plant; multiplied by "
                                    "chronological day delta from LSystemLayer.");
    changed |= editor_widgets::Draw(
        target_.growing_season_start_day, "Growing Season Start Day", 1.0f,
        "Per-pine active season start day-of-year (0-365). Sampled per plant; gates pine growth in LSystemLayer.");
    changed |= editor_widgets::Draw(
        target_.growing_season_end_day, "Growing Season End Day", 1.0f,
        "Per-pine active season end day-of-year (0-365). Sampled per plant; gates pine growth in LSystemLayer.");

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
    clamp_nonnegative_distribution(target_.gdd_per_day);
    clamp_integer_day_distribution(target_.growing_season_start_day);
    clamp_integer_day_distribution(target_.growing_season_end_day);

    ImGui::TreePop();
  }

  // -- Main stem geometry --
  if (ImGui::TreeNodeEx("Main Stem (Leader Axis)", ImGuiTreeNodeFlags_DefaultOpen)) {
    changed |= editor_widgets::Draw(target_.internode_length_m, "Phytomer Internode Length (m)", 0.001f,
                                    "Length of one phytomer's internode in metres.");
    changed |= editor_widgets::Draw(target_.leader_internode_thickness_m, "Main Stem Width (Diameter, m)", 0.0001f,
                                    "Main stem thickness control. This is the leader internode diameter in metres.");
    changed |= editor_widgets::Draw(
        target_.initial_orientation_yaw_deg, "Initial Orientation Yaw (deg)", 1.0f,
        "Sampled once per plant and applied as root yaw around +Y. Set deviation > 0 for random initial orientation.");
    if (ImGui::ColorEdit4("Main Stem Color", &target_.main_stem_color_rgba.x)) {
      changed = true;
    }
    show_item_hover_description("Young stem color used by stem and branch internodes in Shaded and ByType modes.");
    if (ImGui::ColorEdit4("Main Stem Old Color", &target_.main_stem_old_color_rgba.x)) {
      changed = true;
    }
    show_item_hover_description("Old stem color reached as internodes approach descriptor max age.");
    if (ImGui::DragFloat("Main Stem Age Exponent", &target_.internode_age_exponent, 0.05f, 0.1f, 4.0f, "%.2f")) {
      target_.internode_age_exponent = std::clamp(target_.internode_age_exponent, 0.1f, 4.0f);
      changed = true;
    }
    show_item_hover_description(
        "Response curve for stem aging color. 1 = linear, >1 delays browning, <1 accelerates it.");
    ImGui::Text("Mean Radius (m): %.6f", std::max(0.0f, target_.leader_internode_thickness_m.mean) * 0.5f);
    changed |= editor_widgets::Draw(target_.lateral_length_ratio, "Lateral Length Ratio", 0.05f);
    show_item_hover_description("Lateral shoot length = leader_length * ratio^order.");
    changed |= editor_widgets::Draw(target_.lateral_thickness_ratio, "Lateral Thickness Ratio", 0.05f);
    show_item_hover_description("Lateral shoot thickness = leader_thickness * ratio^order.");
    ImGui::TreePop();
  }

  // -- Main stem branching --
  if (ImGui::TreeNodeEx("Main Stem Branching (Whorl Buds)", ImGuiTreeNodeFlags_DefaultOpen)) {
    changed |= editor_widgets::Draw(target_.max_branching_order, "Max Branching Order", 0.5f);
    show_item_hover_description("0 = leader only, 1 = primary laterals, 2 = secondary laterals.");
    changed |= editor_widgets::Draw(target_.branches_per_whorl, "Branches per Whorl", 0.5f);
    show_item_hover_description("Lateral count spawned at whorl bud activation.");
    changed |= editor_widgets::Draw(target_.whorl_dormancy_years, "Whorl Dormancy (years)", 0.05f,
                                    "Chronological years a whorl bud waits before activating laterals. "
                                    "Bud release is chilling/photoperiod-driven, NOT heat-sum-driven "
                                    "(FSPM Rule of Ontogeny). Default 1 yr = annual Scots pine cycle.");
    changed |= editor_widgets::Draw(target_.branch_insertion_angle_deg, "Branch Insertion Angle (deg)", 1.0f);
    show_item_hover_description("Angle laterals depart parent (degrees).");
    changed |= editor_widgets::Draw(target_.branch_roll_phyllotaxis_deg, "Branch Roll Phyllotaxis (deg)", 1.0f);
    show_item_hover_description("Golden-angle azimuth offset between consecutive laterals and needles.");
    ImGui::TreePop();
  }

  // -- Maturity shape curves --
  if (ImGui::TreeNodeEx("Maturity Shape Curves", ImGuiTreeNodeFlags_DefaultOpen)) {
    constexpr const char* kMaturityVariables[] = {"Internode Length", "Internode Width", "Needle Length"};

    ImGui::Combo("Variable", &selected_maturity_variable, kMaturityVariables, IM_ARRAYSIZE(kMaturityVariables));
    show_item_hover_description(
        "Choose which maturity-controlled variable to edit. "
        "x = normalized maturity age of the specific organ instance; "
        "y = multiplier in [0,1], where 1 means use full max length/width.");

    evo_engine::PlottedDistribution<float>* selected_distribution = &target_.internode_length_maturity_curve;
    const char* selected_label = "Internode Length Maturity Response";
    switch (std::clamp(selected_maturity_variable, 0, 2)) {
      case 0:
      default:
        selected_distribution = &target_.internode_length_maturity_curve;
        selected_label = "Internode Length Maturity Response";
        break;
      case 1:
        selected_distribution = &target_.internode_width_maturity_curve;
        selected_label = "Internode Width Maturity Response";
        break;
      case 2:
        selected_distribution = &target_.needle_length_maturity_curve;
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
    changed |= editor_widgets::Draw(*selected_distribution, selected_label, maturity_settings);

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
    static const char* kShapePresets[] = {"(no change)",  "Straight", "Slight Curve",
                                          "Strong Curve", "Wavy",     "Drooping (gravity)"};
    if (ImGui::Combo("Shape Preset##quick_needle_shape", &s_selected_shape_preset, kShapePresets,
                     IM_ARRAYSIZE(kShapePresets))) {
      auto apply_preset = [&](float adaxial, float abaxial, float gradient, float diameter_for_curvature_m,
                              float wave_amp_deg, float wave_freq, float wave_phase_rand_deg, bool enable_droop) {
        target_.needle_curvature_adaxial_bias.mean = adaxial;
        target_.needle_curvature_abaxial_bias.mean = abaxial;
        target_.needle_curvature_gradient_per_arclen.mean = gradient;
        target_.needle_diameter_for_curvature_m.mean = diameter_for_curvature_m;
        target_.needle_sinusoidal_amplitude_deg.mean = wave_amp_deg;
        target_.needle_sinusoidal_frequency_cycles.mean = wave_freq;
        target_.needle_sinusoidal_phase_randomness_deg.mean = wave_phase_rand_deg;
        if (enable_droop) {
          if (target_.needle_young_modulus_baseline_Pa.mean <= 0.0f) {
            target_.needle_young_modulus_baseline_Pa.mean = 1.8e7f;
          }
          if (target_.needle_density_kg_m3.mean <= 0.0f) {
            target_.needle_density_kg_m3.mean = 800.0f;
          }
          if (target_.gravity_m_s2.mean <= 0.0f) {
            target_.gravity_m_s2.mean = 9.81f;
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
        changed |= editor_widgets::Draw(max_distribution, max_label, 0.00005f, tooltip);
        clamp_nonnegative_distribution(max_distribution);

        evo_engine::PlottedDistributionSettings profile_settings;
        profile_settings.tip =
            "Base-to-tip multiplier profile for the selected cross-section axis. "
            "x = normalized arc length from base (0) to tip (1).";
        profile_settings.mean_settings.m_tip =
            "Mean multiplier profile in [0,4]. 1 keeps the max diameter; 0 collapses axis radius; "
            "values >1 enlarge it.";
        profile_settings.dev_settings.m_tip = "Variance (sigma) profile in [0,4] around the mean profile.";
        changed |= editor_widgets::Draw(profile_distribution, profile_label, profile_settings);
        clamp_profile_distribution(profile_distribution, 4.0f);

        ImGui::Text("Current mean max diameter: %.3f mm", std::max(0.0f, max_distribution.mean) * 1000.0f);
        ImGui::TreePop();
      }
    };

    inspect_axis("Width", target_.needle_cross_section_width_max_m, target_.needle_cross_section_width_profile,
                 "Max Width Diameter (m)", "Width Profile (Base -> Tip)",
                 "Maximum full width (major axis diameter) before profile multiplier.");
    inspect_axis("Thickness", target_.needle_cross_section_thickness_max_m,
                 target_.needle_cross_section_thickness_profile, "Max Thickness Diameter (m)",
                 "Thickness Profile (Base -> Tip)",
                 "Maximum full thickness (minor axis diameter) before profile multiplier.");

    evo_engine::PlottedDistributionSettings temporal_settings;
    temporal_settings.tip =
        "Shared temporal multiplier applied to both width and thickness. "
        "x = normalized maturity age where x=1 corresponds to 2 years since initiation.";
    temporal_settings.mean_settings.m_tip =
        "Mean multiplier in [0,1]. Default is sinusoidal: 0.25 at t=0 years to 1.0 at t=2 years.";
    temporal_settings.dev_settings.m_tip = "Variance (sigma) profile around the temporal mean in [0,1].";
    changed |= editor_widgets::Draw(target_.needle_cross_section_temporal_maturity_curve,
                                    "Shared Temporal Width/Thickness Maturity", temporal_settings);
    clamp_profile_distribution(target_.needle_cross_section_temporal_maturity_curve, 1.0f);

    ImGui::TreePop();
  }

  // -- Needles --
  if (ImGui::TreeNodeEx("Needles (Layout and Lifecycle)", ImGuiTreeNodeFlags_DefaultOpen)) {
    changed |= editor_widgets::Draw(
        target_.bare_zone_fraction, "Bare Zone Fraction", 0.01f,
        "Temporal fraction at the start of each year that emits internode-only phytomers [0, 0.95).");
    changed |= editor_widgets::Draw(target_.needle_count_per_cluster, "Needles per Cluster", 0.5f);
    show_item_hover_description("Pinus sylvestris fascicle count (typically 2).");
    if (ImGui::DragInt("Needle Segments", &target_.needle_segment_count, 1.0f, 3, 128)) {
      target_.needle_segment_count = std::clamp(target_.needle_segment_count, 3, 128);
      changed = true;
    }
    show_item_hover_description("Longitudinal segments per needle centerline. Mesh stations = segments + 1.");
    changed |=
        editor_widgets::Draw(target_.needle_length_m, "Needle Length (m)", 0.001f, "Length of needles in metres.");
    changed |= editor_widgets::Draw(target_.needle_lifespan_years, "Needle Lifespan (years)", 0.1f,
                                    "Chronological years a needle stays alive post-maturity. Senescence "
                                    "is calendar-driven, NOT heat-sum-driven (FSPM Rule of Ontogeny). "
                                    "Scots pine typical: 3-4 yr.");
    changed |= editor_widgets::Draw(target_.needle_browning_years, "Needle Browning (years)", 0.05f,
                                    "Chronological years from senescence onset to abscission.");
    changed |= editor_widgets::Draw(target_.needle_flush_delay_gdd, "Needle Flush Delay (GDD)", 10.0f,
                                    "Delay from phytomer emergence to needle flush.");
    changed |= editor_widgets::Draw(target_.internode_maturation_gdd, "Shoot Maturation (GDD)", 10.0f,
                                    "Thermal time from emergence to mature internode length.");
    changed |= editor_widgets::Draw(target_.needle_maturation_gdd, "Needle Maturation (GDD)", 10.0f,
                                    "Thermal time from flush to mature needle length.");
    changed |= editor_widgets::Draw(target_.needle_branching_angle_deg, "Needle Branching Angle (deg)", 0.25f,
                                    "Final branching angle from the parent axis reached after relaxation.");
    changed |=
        editor_widgets::Draw(target_.needle_branching_relax_gdd, "Needle Branching Relaxation (GDD-equivalent)", 10.0f,
                             "Converted using 1500 GDD/year, then applied against chronological "
                             "age so relaxation continues during dormant season.");
    if (ImGui::DragFloat("Order Needle Length Attenuation", &target_.needle_order_length_attenuation, 0.01f, 0.0f, 1.0f,
                         "%.3f")) {
      target_.needle_order_length_attenuation = std::clamp(target_.needle_order_length_attenuation, 0.0f, 1.0f);
      changed = true;
    }
    show_item_hover_description(
        "Linear per-order reduction applied to needle length: scale = max(min, 1 - attenuation*order).");
    if (ImGui::DragFloat("Order Needle Radius Attenuation", &target_.needle_order_radius_attenuation, 0.01f, 0.0f, 1.0f,
                         "%.3f")) {
      target_.needle_order_radius_attenuation = std::clamp(target_.needle_order_radius_attenuation, 0.0f, 1.0f);
      changed = true;
    }
    show_item_hover_description(
        "Linear per-order reduction applied to needle thickness: scale = max(min, 1 - attenuation*order).");
    if (ImGui::DragFloat("Order Needle Length Min Scale", &target_.needle_order_min_length_scale, 0.01f, 0.10f, 1.00f,
                         "%.3f")) {
      target_.needle_order_min_length_scale = std::clamp(target_.needle_order_min_length_scale, 0.10f, 1.00f);
      changed = true;
    }
    show_item_hover_description("Lower bound for branch-order needle length scaling.");
    if (ImGui::DragFloat("Order Needle Radius Min Scale", &target_.needle_order_min_radius_scale, 0.01f, 0.10f, 1.00f,
                         "%.3f")) {
      target_.needle_order_min_radius_scale = std::clamp(target_.needle_order_min_radius_scale, 0.10f, 1.00f);
      changed = true;
    }
    show_item_hover_description("Lower bound for branch-order needle thickness scaling.");
    if (ImGui::TreeNodeEx("Initiation Capacity Mapping", ImGuiTreeNodeFlags_DefaultOpen)) {
      if (ImGui::DragFloat("Intra-Year Base Ratio", &target_.needle_intra_year_base_ratio, 0.01f, 0.0f, 1.0f, "%.3f")) {
        target_.needle_intra_year_base_ratio = std::clamp(target_.needle_intra_year_base_ratio, 0.0f, 1.0f);
        changed = true;
      }
      show_item_hover_description(
          "Lower bound for early-phytomer capacity."
          " Runtime applies L_actual = L_max * w_intra(i) * w_inter(year,vigor).");

      if (ImGui::DragFloat("Intra-Year Sigmoid Steepness", &target_.needle_intra_year_sigmoid_steepness, 0.10f, 0.01f,
                           32.0f, "%.3f")) {
        target_.needle_intra_year_sigmoid_steepness = std::max(0.01f, target_.needle_intra_year_sigmoid_steepness);
        changed = true;
      }
      show_item_hover_description("Steepness k of the intra-year sigmoid over normalized phytomer index.");

      if (ImGui::DragFloat("Intra-Year Sigmoid Midpoint", &target_.needle_intra_year_sigmoid_midpoint_fraction, 0.01f,
                           0.0f, 1.0f, "%.3f")) {
        target_.needle_intra_year_sigmoid_midpoint_fraction =
            std::clamp(target_.needle_intra_year_sigmoid_midpoint_fraction, 0.0f, 1.0f);
        changed = true;
      }
      show_item_hover_description("Normalized phytomer index where intra-year capacity reaches 50% ramp.");

      if (ImGui::DragFloat("Late-Season Decay Start", &target_.needle_intra_year_late_decay_start_fraction, 0.01f, 0.0f,
                           1.0f, "%.3f")) {
        target_.needle_intra_year_late_decay_start_fraction =
            std::clamp(target_.needle_intra_year_late_decay_start_fraction, 0.0f, 1.0f);
        changed = true;
      }
      show_item_hover_description("Normalized phytomer index where optional late-season decay begins.");

      if (ImGui::DragFloat("Late-Season End Scale", &target_.needle_intra_year_late_decay_end_scale, 0.01f, 0.0f, 2.0f,
                           "%.3f")) {
        target_.needle_intra_year_late_decay_end_scale =
            std::clamp(target_.needle_intra_year_late_decay_end_scale, 0.0f, 2.0f);
        changed = true;
      }
      show_item_hover_description("Multiplier reached at the final phytomer if late-season decay is active.");

      if (ImGui::DragInt("Fascicular Start Year", &target_.needle_fascicular_start_year, 1.0f, 0, 16)) {
        target_.needle_fascicular_start_year = std::clamp(target_.needle_fascicular_start_year, 0, 16);
        changed = true;
      }
      show_item_hover_description(
          "Year index where year2+ multipliers become active. 0 applies them from first-year shoots.");

      if (ImGui::DragFloat("Year2+ Length Multiplier", &target_.needle_year2plus_length_multiplier, 0.01f, 0.0f, 8.0f,
                           "%.3f")) {
        target_.needle_year2plus_length_multiplier = std::max(0.0f, target_.needle_year2plus_length_multiplier);
        changed = true;
      }
      show_item_hover_description("Inter-year multiplier for needle target length.");

      if (ImGui::DragFloat("Year2+ Width Multiplier", &target_.needle_year2plus_width_multiplier, 0.01f, 0.0f, 8.0f,
                           "%.3f")) {
        target_.needle_year2plus_width_multiplier = std::max(0.0f, target_.needle_year2plus_width_multiplier);
        changed = true;
      }
      show_item_hover_description("Inter-year multiplier for needle major-axis width.");

      if (ImGui::DragFloat("Year2+ Thickness Multiplier", &target_.needle_year2plus_thickness_multiplier, 0.01f, 0.0f,
                           8.0f, "%.3f")) {
        target_.needle_year2plus_thickness_multiplier = std::max(0.0f, target_.needle_year2plus_thickness_multiplier);
        changed = true;
      }
      show_item_hover_description("Inter-year multiplier for needle minor-axis thickness.");

      if (ImGui::DragFloat("Year1 Lignification Factor", &target_.needle_lignification_factor_year1, 0.01f, 0.0f, 2.0f,
                           "%.3f")) {
        target_.needle_lignification_factor_year1 = std::clamp(target_.needle_lignification_factor_year1, 0.0f, 2.0f);
        changed = true;
      }
      show_item_hover_description("Scales visual maturation response for first-year needle cohorts.");

      if (ImGui::DragFloat("Year2+ Lignification Factor", &target_.needle_lignification_factor_year2plus, 0.01f, 0.0f,
                           2.0f, "%.3f")) {
        target_.needle_lignification_factor_year2plus =
            std::clamp(target_.needle_lignification_factor_year2plus, 0.0f, 2.0f);
        changed = true;
      }
      show_item_hover_description("Scales visual maturation response for year2+ needle cohorts.");

      if (ImGui::DragFloat("Year1 Stomatal Strip Density", &target_.needle_stomatal_strip_density_year1, 0.01f, 0.0f,
                           1.0f, "%.3f")) {
        target_.needle_stomatal_strip_density_year1 =
            std::clamp(target_.needle_stomatal_strip_density_year1, 0.0f, 1.0f);
        changed = true;
      }
      show_item_hover_description("Proxy density for procedural stomatal striping in first-year cohorts.");

      if (ImGui::DragFloat("Year2+ Stomatal Strip Density", &target_.needle_stomatal_strip_density_year2plus, 0.01f,
                           0.0f, 1.0f, "%.3f")) {
        target_.needle_stomatal_strip_density_year2plus =
            std::clamp(target_.needle_stomatal_strip_density_year2plus, 0.0f, 1.0f);
        changed = true;
      }
      show_item_hover_description("Proxy density for procedural stomatal striping in year2+ cohorts.");

      if (ImGui::DragFloat("Year1 Basal Taper Ratio", &target_.needle_basal_taper_ratio_year1, 0.01f, 0.6f, 1.2f,
                           "%.3f")) {
        target_.needle_basal_taper_ratio_year1 = std::clamp(target_.needle_basal_taper_ratio_year1, 0.6f, 1.2f);
        changed = true;
      }
      show_item_hover_description("Needle-base radius multiplier for first-year cohorts. 1.0 disables base taper.");

      if (ImGui::DragFloat("Year2+ Basal Taper Ratio", &target_.needle_basal_taper_ratio_year2plus, 0.01f, 0.6f, 1.2f,
                           "%.3f")) {
        target_.needle_basal_taper_ratio_year2plus = std::clamp(target_.needle_basal_taper_ratio_year2plus, 0.6f, 1.2f);
        changed = true;
      }
      show_item_hover_description("Needle-base radius multiplier for year2+ cohorts. 1.0 disables base taper.");

      if (ImGui::DragFloat("Fascicle Sheath Budget (GDD)", &target_.needle_fascicle_sheath_budget_gdd, 10.0f, 0.0f,
                           5000.0f, "%.1f")) {
        target_.needle_fascicle_sheath_budget_gdd = std::max(0.0f, target_.needle_fascicle_sheath_budget_gdd);
        changed = true;
      }
      show_item_hover_description("Characteristic thermal budget for sheath maturation near needle bases.");

      if (ImGui::DragFloat("Year1 Specularity Plasticity", &target_.needle_specularity_plasticity_year1, 0.01f, 0.0f,
                           1.0f, "%.3f")) {
        target_.needle_specularity_plasticity_year1 =
            std::clamp(target_.needle_specularity_plasticity_year1, 0.0f, 1.0f);
        changed = true;
      }
      show_item_hover_description("How strongly first-year micro-variation tracks maturity cues.");

      if (ImGui::DragFloat("Year2+ Specularity Plasticity", &target_.needle_specularity_plasticity_year2plus, 0.01f,
                           0.0f, 1.0f, "%.3f")) {
        target_.needle_specularity_plasticity_year2plus =
            std::clamp(target_.needle_specularity_plasticity_year2plus, 0.0f, 1.0f);
        changed = true;
      }
      show_item_hover_description("How strongly year2+ micro-variation tracks maturity cues.");

      if (ImGui::DragFloat("Bud-Storage Vigor Strength", &target_.needle_bud_storage_vigor_strength, 0.01f, 0.0f, 1.0f,
                           "%.3f")) {
        target_.needle_bud_storage_vigor_strength = std::clamp(target_.needle_bud_storage_vigor_strength, 0.0f, 1.0f);
        changed = true;
      }
      show_item_hover_description("Blend factor from 1.0 to previous-season vigor proxy for inter-year capacity.");

      if (ImGui::DragFloat("Bud-Storage Completion Floor", &target_.needle_bud_storage_completion_floor, 0.01f, 0.0f,
                           1.0f, "%.3f")) {
        target_.needle_bud_storage_completion_floor =
            std::clamp(target_.needle_bud_storage_completion_floor, 0.0f, 1.0f);
        changed = true;
      }
      show_item_hover_description("Lower clamp applied to completion ratio before vigor carry-over.");
      ImGui::TreePop();
    }
    if (ImGui::DragFloat("[deprecated] Needle Width Cap (unused)", &target_.needle_radius_to_stem_thickness_max_ratio,
                         0.05f, 0.0f, 4.0f, "%.3f")) {
      target_.needle_radius_to_stem_thickness_max_ratio =
          std::clamp(target_.needle_radius_to_stem_thickness_max_ratio, 0.0f, 4.0f);
      changed = true;
    }
    show_item_hover_description(
        "Deprecated no-op. Needle width and thickness are uncapped and no "
        "longer tied to stem thickness.");
    if (ImGui::ColorEdit4("Needle Color", &target_.needle_color_rgba.x)) {
      changed = true;
    }
    show_item_hover_description("Young needle color used for newly flushed or low-age segments.");
    if (ImGui::ColorEdit4("Needle Old Color", &target_.needle_old_color_rgba.x)) {
      changed = true;
    }
    show_item_hover_description("Old needle color reached by high-age or strongly senescent segments.");
    if (ImGui::DragFloat("Needle Axial Age Span", &target_.needle_axial_age_span, 0.01f, -1.0f, 1.0f, "%.3f")) {
      target_.needle_axial_age_span = std::clamp(target_.needle_axial_age_span, -1.0f, 1.0f);
      changed = true;
    }
    show_item_hover_description(
        "Along-needle age shift. Positive biases older color toward tip; negative biases older color toward base.");
    if (ImGui::DragFloat("Needle Axial Age Exponent", &target_.needle_axial_age_exponent, 0.05f, 0.1f, 4.0f, "%.2f")) {
      target_.needle_axial_age_exponent = std::clamp(target_.needle_axial_age_exponent, 0.1f, 4.0f);
      changed = true;
    }
    show_item_hover_description(
        "Shape of along-needle gradient response. 1 = linear, >1 concentrates changes near one end.");
    ImGui::TreePop();
  }

  // -- Needle curvature (bilateral differential growth field) --
  if (ImGui::TreeNodeEx("Needle Shape (Curvature Field)")) {
    changed |= editor_widgets::Draw(target_.needle_curvature_adaxial_bias, "Adaxial Elongation Bias", 0.001f,
                                    "Dimensionless adaxial side elongation. Positive bends needle toward stem.");
    changed |= editor_widgets::Draw(target_.needle_curvature_abaxial_bias, "Abaxial Elongation Bias", 0.001f,
                                    "Dimensionless abaxial side elongation. Positive bends needle away from stem.");
    changed |=
        editor_widgets::Draw(target_.needle_curvature_gradient_per_arclen, "Curvature Gradient (per s_norm)", 0.001f,
                             "Linear gradient added to (abaxial - adaxial) along normalized arc length.");
    changed |= editor_widgets::Draw(target_.needle_diameter_for_curvature_m, "Effective Diameter (m)", 0.0001f,
                                    "Cross-section diameter used to convert strain differential into curvature. "
                                    "Set > 0 to activate the field.");
    changed |= editor_widgets::Draw(
        target_.needle_sinusoidal_amplitude_deg, "Sinusoidal Wave Amplitude (deg)", 0.10f,
        "Additional intrinsic waviness amplitude applied along the needle; 0 keeps arc-only behavior.");
    changed |= editor_widgets::Draw(target_.needle_sinusoidal_frequency_cycles, "Sinusoidal Wave Frequency (cycles)",
                                    0.05f, "Number of waviness cycles along full needle length.");
    changed |=
        editor_widgets::Draw(target_.needle_sinusoidal_phase_randomness_deg, "Sinusoidal Phase Randomness (deg)", 0.10f,
                             "Sampled phase jitter magnitude combined with deterministic per-needle phase.");
    ImGui::TreePop();
  }

  // -- Needle mechanics (elastica) --
  if (ImGui::TreeNodeEx("Needle Mechanics (Elastica)")) {
    changed |= editor_widgets::Draw(target_.needle_young_modulus_baseline_Pa, "Young's Modulus Baseline (Pa)", 1e6f,
                                    "Asymptotic Young's modulus at maturity. 0 = solver disabled.");
    changed |= editor_widgets::Draw(target_.needle_lignification_maturation_years, "Lignification Maturation (yr)",
                                    0.05f, "Sigmoid maturation duration for E(t).");
    changed |= editor_widgets::Draw(target_.needle_density_kg_m3, "Tissue Density (kg/m^3)", 10.0f,
                                    "Used to derive distributed weight per unit arc length.");
    changed |= editor_widgets::Draw(target_.gravity_m_s2, "Gravity (m/s^2)", 0.1f,
                                    "World-frame gravity magnitude. 0 = no body force.");
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Needle Per-Needle Variability")) {
    changed |= editor_widgets::Draw(target_.needle_per_needle_length_cv, "Length CV", 0.01f,
                                    "CV-style variation across needles within a cluster for length scale.");
    changed |=
        editor_widgets::Draw(target_.needle_per_needle_curvature_cv, "Curvature CV", 0.01f,
                             "CV-style variation across needles within a cluster for curvature-field magnitude.");
    changed |= editor_widgets::Draw(target_.needle_per_needle_radius_cv, "Radius CV", 0.01f,
                                    "CV-style variation across needles within a cluster for cross-section axis scale.");
    changed |= editor_widgets::Draw(target_.needle_per_needle_modulus_cv, "Young's Modulus CV", 0.01f,
                                    "CV-style variation across needles within a cluster for baseline Young's modulus.");
    changed |= editor_widgets::Draw(target_.needle_per_needle_density_cv, "Density CV", 0.01f,
                                    "CV-style variation across needles within a cluster for tissue density.");
    changed |=
        editor_widgets::Draw(target_.needle_per_needle_wave_amplitude_cv, "Wave Amplitude CV", 0.01f,
                             "CV-style variation across needles within a cluster for sinusoidal waviness amplitude.");
    changed |=
        editor_widgets::Draw(target_.needle_per_needle_wave_frequency_cv, "Wave Frequency CV", 0.01f,
                             "CV-style variation across needles within a cluster for sinusoidal waviness frequency.");
    changed |= editor_widgets::Draw(target_.needle_per_needle_wave_phase_cv, "Wave Phase CV", 0.01f,
                                    "CV-style scaling of per-needle sinusoidal phase randomness.");
    ImGui::TreePop();
  }

  // -- Tropism --
  if (ImGui::TreeNodeEx("Tropism (Global)")) {
    changed |= editor_widgets::Draw(
        target_.gravitropism_first_order, "Main Stem Tropism (deg/GDD)", 0.0001f,
        "Per-GDD curvature applied to leader internodes only (branch order 0). Positive bends upward.");
    ImGui::TreePop();
  }

  // -- Per-shoot stochastic noise --
  if (ImGui::TreeNodeEx("Stochastic Variation (Per Shoot)")) {
    changed |= editor_widgets::Draw(target_.internode_length_per_node_cv, "Internode Length CV", 0.005f,
                                    "Per-internode Gaussian CV on phytomer length. 0 = deterministic.");
    changed |= editor_widgets::Draw(target_.internode_thickness_per_node_cv, "Internode Thickness CV", 0.005f,
                                    "Per-internode Gaussian CV on shoot thickness. 0 = deterministic.");
    changed |= editor_widgets::Draw(target_.branch_angle_per_node_sigma_deg, "Branch Angle Sigma (deg)", 0.5f,
                                    "Per-lateral additive Gaussian sigma on insertion angle.");
    changed |= editor_widgets::Draw(target_.roll_phyllotaxis_per_node_sigma_deg, "Roll Phyllotaxis Sigma (deg)", 0.5f,
                                    "Per-lateral additive Gaussian sigma on phyllotaxis roll.");
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
        target_.tropisms.emplace_back();
        changed = true;
      }
      show_item_hover_description("Add a directional tropism entry. [deprecated] no runtime effect.");

      int remove_index = -1;
      for (size_t i = 0; i < target_.tropisms.size(); ++i) {
        ImGui::PushID(static_cast<int>(i));
        const std::string label = "Tropism #" + std::to_string(i);
        if (ImGui::TreeNodeEx(label.c_str(), ImGuiTreeNodeFlags_DefaultOpen)) {
          auto& entry = target_.tropisms[i];
          changed |= editor_widgets::Draw(entry.direction_x, "Direction X", 0.05f);
          changed |= editor_widgets::Draw(entry.direction_y, "Direction Y", 0.05f);
          changed |= editor_widgets::Draw(entry.direction_z, "Direction Z", 0.05f);
          changed |= editor_widgets::Draw(entry.strength, "Strength", 0.05f);
          if (ImGui::DragFloat("Usage Chance (%)", &entry.usage_chance_percent, 1.0f, 0.0f, 100.0f, "%.1f")) {
            entry.usage_chance_percent = std::clamp(entry.usage_chance_percent, 0.0f, 100.0f);
            changed = true;
          }
          changed |= editor_widgets::Draw(entry.order_response, "Order Response (vs branching order)");
          if (ImGui::Button("Remove"))
            remove_index = static_cast<int>(i);
          ImGui::TreePop();
        }
        ImGui::PopID();
      }
      if (remove_index >= 0) {
        target_.tropisms.erase(target_.tropisms.begin() + remove_index);
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

void ScotsPineDescriptorEditor::RegisterExplorableAxes(ParamSpaceExplorer& explorer) {
  auto& d = target_;

  // -- Phytomer scheduling --
  explorer.AddSingle("max_branching_order", "MBO", d.max_branching_order, 0.0f, 4.0f, 2.0f);
  explorer.AddSingle("plastochron_gdd", "PLG", d.plastochron_gdd, 50.0f, 6000.0f, 1500.0f);
  explorer.AddSingle("max_phytomers_per_seasonal_growth", "MPS", d.max_phytomers_per_seasonal_growth, 1.0f, 64.0f,
                     12.0f);

  // -- Whorl architecture --
  explorer.AddSingle("branches_per_whorl", "BPW", d.branches_per_whorl, 0.0f, 10.0f, 5.0f);
  explorer.AddSingle("whorl_dormancy_years", "WDY", d.whorl_dormancy_years, 0.0f, 4.0f, 1.0f);
  explorer.AddSingle("branch_insertion_angle_deg", "BIA", d.branch_insertion_angle_deg, -85.0f, 85.0f, 60.0f);
  explorer.AddSingle("branch_roll_phyllotaxis_deg", "BRP", d.branch_roll_phyllotaxis_deg, 0.0f, 360.0f, 137.5f);

  // -- Phytomer dimensions --
  explorer.AddSingle("internode_length_m", "ILM", d.internode_length_m, 0.0001f, 0.500f, 0.012f);
  explorer.AddSingle("main_stem_width_m", "MSW", d.leader_internode_thickness_m, 0.0001f, 0.0500f, 0.0030f);
  explorer.AddSingle("lateral_length_ratio", "LLR", d.lateral_length_ratio, 0.1f, 1.5f, 0.7f);
  explorer.AddSingle("lateral_thickness_ratio", "LTR", d.lateral_thickness_ratio, 0.1f, 1.5f, 0.6f);

  // -- Needles --
  explorer.AddSingle("bare_zone_fraction", "BZF", d.bare_zone_fraction, 0.0f, 0.95f, 0.0f);
  explorer.AddSingle("needle_count_per_cluster", "NCC", d.needle_count_per_cluster, 1.0f, 6.0f, 2.0f);
  {
    auto* value_ptr = &d.needle_segment_count;
    explorer.AddAxis(
        "needle_segment_count", "NSG", 3.0f, 128.0f,
        [value_ptr]() {
          return static_cast<float>(*value_ptr);
        },
        [value_ptr](float value) {
          *value_ptr = std::clamp(static_cast<int>(std::round(value)), 3, 128);
        });
  }
  explorer.AddSingle("needle_length_m", "NLM", d.needle_length_m, 0.001f, 0.200f, 0.025f);
  explorer.AddSingle("needle_lifespan_years", "NLY", d.needle_lifespan_years, 0.0f, 10.0f, 4.0f);
  explorer.AddSingle("needle_browning_years", "NBY", d.needle_browning_years, 0.0f, 4.0f, 1.0f);
  explorer.AddSingle("needle_flush_delay_gdd", "NFD", d.needle_flush_delay_gdd, 0.0f, 3000.0f, 0.0f);
  explorer.AddSingle("internode_maturation_gdd", "IMG", d.internode_maturation_gdd, 0.0f, 3000.0f, 60.0f);
  explorer.AddSingle("needle_maturation_gdd", "NMG", d.needle_maturation_gdd, 0.0f, 6000.0f, 120.0f);
  explorer.AddSingle("needle_branching_angle_deg", "NBA", d.needle_branching_angle_deg, 0.0f, 89.5f, 72.0f);
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
    auto* value_ptr = &d.needle_intra_year_base_ratio;
    explorer.AddAxis(
        "needle_intra_year_base_ratio", "NIB", 0.0f, 1.0f,
        [value_ptr]() {
          return *value_ptr;
        },
        [value_ptr](float value) {
          *value_ptr = std::clamp(value, 0.0f, 1.0f);
        });
  }
  {
    auto* value_ptr = &d.needle_intra_year_sigmoid_steepness;
    explorer.AddAxis(
        "needle_intra_year_sigmoid_steepness", "NIS", 0.01f, 32.0f,
        [value_ptr]() {
          return *value_ptr;
        },
        [value_ptr](float value) {
          *value_ptr = std::max(0.01f, value);
        });
  }
  {
    auto* value_ptr = &d.needle_intra_year_sigmoid_midpoint_fraction;
    explorer.AddAxis(
        "needle_intra_year_sigmoid_midpoint_fraction", "NIM", 0.0f, 1.0f,
        [value_ptr]() {
          return *value_ptr;
        },
        [value_ptr](float value) {
          *value_ptr = std::clamp(value, 0.0f, 1.0f);
        });
  }
  {
    auto* value_ptr = &d.needle_intra_year_late_decay_start_fraction;
    explorer.AddAxis(
        "needle_intra_year_late_decay_start_fraction", "NDS", 0.0f, 1.0f,
        [value_ptr]() {
          return *value_ptr;
        },
        [value_ptr](float value) {
          *value_ptr = std::clamp(value, 0.0f, 1.0f);
        });
  }
  {
    auto* value_ptr = &d.needle_intra_year_late_decay_end_scale;
    explorer.AddAxis(
        "needle_intra_year_late_decay_end_scale", "NDE", 0.0f, 2.0f,
        [value_ptr]() {
          return *value_ptr;
        },
        [value_ptr](float value) {
          *value_ptr = std::clamp(value, 0.0f, 2.0f);
        });
  }
  {
    auto* value_ptr = &d.needle_fascicular_start_year;
    explorer.AddAxis(
        "needle_fascicular_start_year", "NFY", 0.0f, 16.0f,
        [value_ptr]() {
          return static_cast<float>(*value_ptr);
        },
        [value_ptr](float value) {
          *value_ptr = std::clamp(static_cast<int>(std::round(value)), 0, 16);
        });
  }
  {
    auto* value_ptr = &d.needle_year2plus_length_multiplier;
    explorer.AddAxis(
        "needle_year2plus_length_multiplier", "N2L", 0.0f, 8.0f,
        [value_ptr]() {
          return *value_ptr;
        },
        [value_ptr](float value) {
          *value_ptr = std::max(0.0f, value);
        });
  }
  {
    auto* value_ptr = &d.needle_year2plus_width_multiplier;
    explorer.AddAxis(
        "needle_year2plus_width_multiplier", "N2W", 0.0f, 8.0f,
        [value_ptr]() {
          return *value_ptr;
        },
        [value_ptr](float value) {
          *value_ptr = std::max(0.0f, value);
        });
  }
  {
    auto* value_ptr = &d.needle_year2plus_thickness_multiplier;
    explorer.AddAxis(
        "needle_year2plus_thickness_multiplier", "N2T", 0.0f, 8.0f,
        [value_ptr]() {
          return *value_ptr;
        },
        [value_ptr](float value) {
          *value_ptr = std::max(0.0f, value);
        });
  }
  {
    auto* value_ptr = &d.needle_lignification_factor_year1;
    explorer.AddAxis(
        "needle_lignification_factor_year1", "NL1", 0.0f, 2.0f,
        [value_ptr]() {
          return *value_ptr;
        },
        [value_ptr](float value) {
          *value_ptr = std::clamp(value, 0.0f, 2.0f);
        });
  }
  {
    auto* value_ptr = &d.needle_lignification_factor_year2plus;
    explorer.AddAxis(
        "needle_lignification_factor_year2plus", "NL2", 0.0f, 2.0f,
        [value_ptr]() {
          return *value_ptr;
        },
        [value_ptr](float value) {
          *value_ptr = std::clamp(value, 0.0f, 2.0f);
        });
  }
  {
    auto* value_ptr = &d.needle_stomatal_strip_density_year1;
    explorer.AddAxis(
        "needle_stomatal_strip_density_year1", "NS1", 0.0f, 1.0f,
        [value_ptr]() {
          return *value_ptr;
        },
        [value_ptr](float value) {
          *value_ptr = std::clamp(value, 0.0f, 1.0f);
        });
  }
  {
    auto* value_ptr = &d.needle_stomatal_strip_density_year2plus;
    explorer.AddAxis(
        "needle_stomatal_strip_density_year2plus", "NS2", 0.0f, 1.0f,
        [value_ptr]() {
          return *value_ptr;
        },
        [value_ptr](float value) {
          *value_ptr = std::clamp(value, 0.0f, 1.0f);
        });
  }
  {
    auto* value_ptr = &d.needle_basal_taper_ratio_year1;
    explorer.AddAxis(
        "needle_basal_taper_ratio_year1", "NB1", 0.6f, 1.2f,
        [value_ptr]() {
          return *value_ptr;
        },
        [value_ptr](float value) {
          *value_ptr = std::clamp(value, 0.6f, 1.2f);
        });
  }
  {
    auto* value_ptr = &d.needle_basal_taper_ratio_year2plus;
    explorer.AddAxis(
        "needle_basal_taper_ratio_year2plus", "NB2", 0.6f, 1.2f,
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
    auto* value_ptr = &d.needle_specularity_plasticity_year1;
    explorer.AddAxis(
        "needle_specularity_plasticity_year1", "NP1", 0.0f, 1.0f,
        [value_ptr]() {
          return *value_ptr;
        },
        [value_ptr](float value) {
          *value_ptr = std::clamp(value, 0.0f, 1.0f);
        });
  }
  {
    auto* value_ptr = &d.needle_specularity_plasticity_year2plus;
    explorer.AddAxis(
        "needle_specularity_plasticity_year2plus", "NP2", 0.0f, 1.0f,
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
  // Width cap exposed as a sweepable axis. Default 0.45 = legacy behavior.
  // Note: this is a plain scalar field; AddAxis is used so the explorer can
  // read/write it directly without needing a SingleDistribution wrapper.
  {
    auto* ratio_ptr = &d.needle_radius_to_stem_thickness_max_ratio;
    explorer.AddAxis(
        "needle_radius_to_stem_thickness_max_ratio", "NRC", 0.0f, 4.0f,
        [ratio_ptr]() {
          return *ratio_ptr;
        },
        [ratio_ptr](float v) {
          *ratio_ptr = std::clamp(v, 0.0f, 4.0f);
        });
  }
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
  explorer.AddSingle("growing_season_start_day", "GSS", d.growing_season_start_day, 0.0f, 365.0f, 60.0f);
  explorer.AddSingle("growing_season_end_day", "GSE", d.growing_season_end_day, 0.0f, 365.0f, 334.0f);

  // -- Per-shoot stochastic noise --
  explorer.AddSingle("internode_length_per_node_cv", "ILC", d.internode_length_per_node_cv, 0.0f, 1.0f, 0.1f);
  explorer.AddSingle("internode_thickness_per_node_cv", "STC", d.internode_thickness_per_node_cv, 0.0f, 1.0f, 0.1f);
  explorer.AddSingle("branch_angle_per_node_sigma_deg", "BAS", d.branch_angle_per_node_sigma_deg, 0.0f, 30.0f, 5.0f);
  explorer.AddSingle("roll_phyllotaxis_per_node_sigma_deg", "RPS", d.roll_phyllotaxis_per_node_sigma_deg, 0.0f, 30.0f,
                     5.0f);

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
}  // namespace

bool l_system_package::RegisterScotsPineDescriptorInspector(EditorPackageRegistrar& registrar) {
  struct Entry {
    std::weak_ptr<IAsset> asset;
    std::unique_ptr<ScotsPineDescriptorEditor> editor;
  };
  auto entries = std::make_shared<std::unordered_map<const ScotsPineDescriptor*, Entry>>();
  return registrar.RegisterInspector<ScotsPineDescriptor>(
      [entries](InspectorContext& context, ScotsPineDescriptor& descriptor) {
        for (auto it = entries->begin(); it != entries->end();)
          if (it->second.asset.expired())
            it = entries->erase(it);
          else
            ++it;
        auto& entry = (*entries)[&descriptor];
        if (!entry.editor) {
          entry.asset = descriptor.GetSelf();
          entry.editor = std::make_unique<ScotsPineDescriptorEditor>(descriptor);
        }
        return entry.editor->Inspect(context);
      },
      "ScotsPineDescriptor");
}
