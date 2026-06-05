#include "LSystemLayer.hpp"

#include "Application.hpp"
#include "EditorLayer.hpp"
#include "Scene.hpp"
#include "ScotsPine.hpp"
#include "ScotsPineDescriptor.hpp"
#include "Times.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <random>

using namespace l_system_package;
using namespace evo_engine;

namespace {

template <typename Selector>
double AverageProfileMetric(const std::vector<LSystemLayer::ProfileFrame>& frames, Selector selector) {
  if (frames.empty()) {
    return 0.0;
  }
  double sum = 0.0;
  for (const auto& frame : frames) {
    sum += selector(frame);
  }
  return sum / static_cast<double>(frames.size());
}

template <typename Selector>
double MaxProfileMetric(const std::vector<LSystemLayer::ProfileFrame>& frames, Selector selector) {
  double max_value = 0.0;
  for (const auto& frame : frames) {
    max_value = std::max(max_value, selector(frame));
  }
  return max_value;
}

float SampleDescriptorTargetGddForPine(ScotsPine& pine) {
  auto descriptor = pine.descriptor_ref.Get<ScotsPineDescriptor>();
  if (pine.enable_repot_profile_switch && pine.growth_model.IsPostRepotProfileActive()) {
    if (const auto post_descriptor = pine.post_repot_descriptor_ref.Get<ScotsPineDescriptor>()) {
      descriptor = post_descriptor;
    }
  }
  if (!descriptor) {
    return -1.0f;
  }

  std::mt19937 rng(pine.seed);
  return std::max(0.0f, SampleDistribution(descriptor->target_gdd, rng));
}

float NormalizeDayOfYear(float day) {
  if (!std::isfinite(day)) {
    return 0.0f;
  }
  day = std::fmod(day, 365.0f);
  if (day < 0.0f) {
    day += 365.0f;
  }
  return day;
}

bool IsInActiveSeason(const float simulation_day_of_year, const int season_start_day, const int season_end_day) {
  const int day = static_cast<int>(std::floor(NormalizeDayOfYear(simulation_day_of_year)));
  const int start = std::clamp(season_start_day, 0, 364);
  const int end = std::clamp(season_end_day, 0, 364);
  if (start <= end) {
    return day >= start && day <= end;
  }
  return day >= start || day <= end;
}

int ClampColorModeIndex(const int mode) {
  return std::clamp(mode, 0, 6);
}

int ResolveEffectiveColorMode(const int selected_mode, const bool scene_plant_view_tint_enabled) {
  return scene_plant_view_tint_enabled ? ClampColorModeIndex(selected_mode) : 0;
}

void ApplyGlobalPlantColorMode(const int selected_mode, const bool scene_plant_view_tint_enabled) {
  const int effective_mode = ResolveEffectiveColorMode(selected_mode, scene_plant_view_tint_enabled);
  ScotsPine::SetGlobalColorMode(static_cast<ScotsPine::ColorMode>(effective_mode));
}

void ApplyPineStemOnlyMode(const std::shared_ptr<Scene>& scene, const bool stem_only_mode,
                           const bool regenerate_existing_pines) {
  ScotsPine::SetGenerateNeedleTopologyEnabled(!stem_only_mode);

  if (!regenerate_existing_pines || !scene) {
    return;
  }

  if (const auto* pine_entities_ptr = scene->UnsafeGetPrivateComponentOwnersList<ScotsPine>()) {
    const std::vector<Entity> pine_entities = *pine_entities_ptr;
    for (const auto& entity : pine_entities) {
      if (!scene->IsEntityValid(entity)) {
        continue;
      }
      if (const auto pine = scene->GetOrSetPrivateComponent<ScotsPine>(entity).lock()) {
        pine->GrowToTargetGDD();
      }
    }
  }
}

void RebuildAllPlantGeometry(const std::shared_ptr<Scene>& scene) {
  if (!scene) {
    return;
  }

  if (const auto* pine_entities_ptr = scene->UnsafeGetPrivateComponentOwnersList<ScotsPine>()) {
    const std::vector<Entity> pine_entities = *pine_entities_ptr;
    for (const auto& entity : pine_entities) {
      if (!scene->IsEntityValid(entity)) {
        continue;
      }
      if (const auto pine = scene->GetOrSetPrivateComponent<ScotsPine>(entity).lock()) {
        pine->RebuildGeometry();
      }
    }
  }
}

struct PineTemporalSample {
  float gdd_per_day = 2.0f;
  int season_start_day = 60;
  int season_end_day = 334;
};

PineTemporalSample SamplePineTemporalParameters(ScotsPine& pine) {
  PineTemporalSample sample;
  auto descriptor = pine.descriptor_ref.Get<ScotsPineDescriptor>();
  if (pine.enable_repot_profile_switch && pine.growth_model.IsPostRepotProfileActive()) {
    if (const auto post_descriptor = pine.post_repot_descriptor_ref.Get<ScotsPineDescriptor>()) {
      descriptor = post_descriptor;
    }
  }
  if (!descriptor) {
    return sample;
  }

  std::mt19937 rng(static_cast<uint32_t>(pine.seed) ^ 0x5f3759dfu);
  sample.gdd_per_day = std::max(0.0f, SampleDistribution(descriptor->gdd_per_day, rng));

  const auto sample_day = [&](const SingleDistribution<float>& distribution) {
    const float sampled_day = std::clamp(SampleDistribution(distribution, rng), 0.0f, 365.0f);
    return static_cast<int>(std::floor(NormalizeDayOfYear(std::round(sampled_day))));
  };

  sample.season_start_day = sample_day(descriptor->growing_season_start_day);
  sample.season_end_day = sample_day(descriptor->growing_season_end_day);
  return sample;
}

}  // namespace

void LSystemLayer::OnCreate() {
  simulation_day_of_year = NormalizeDayOfYear(static_cast<float>(std::clamp(season_start_day, 0, 364)));
  ApplyGlobalPlantColorMode(tassel_color_mode, scene_plant_view_tint_enabled);
  ApplyPineStemOnlyMode(GetScene(), pine_stem_only_mode, true);
}

void LSystemLayer::OnDestroy() {
}

void LSystemLayer::PushProfileFrame(const ProfileFrame& frame) {
  if (profiling_history_size <= 0) {
    return;
  }
  if (profiling_history.size() >= static_cast<size_t>(profiling_history_size)) {
    profiling_history.erase(profiling_history.begin());
  }
  profiling_history.push_back(frame);
}

void LSystemLayer::ExportProfileCsv(const std::string& path) const {
  std::ofstream out(path, std::ios::trunc);
  if (!out.is_open()) {
    return;
  }

  out << "frame,update_ms,grow_ms,rebuild_ms,pines,growth_steps,nodes,internodes,needles,invalid_instances\n";
  for (size_t i = 0; i < profiling_history.size(); i++) {
    const auto& f = profiling_history[i];
    out << i << "," << std::fixed << std::setprecision(4) << f.update_ms << "," << f.grow_ms << "," << f.rebuild_ms
        << "," << f.pine_count << "," << f.growth_steps << "," << f.node_count << "," << f.internode_count << ","
        << f.needle_count << "," << f.invalid_instance_count << "\n";
  }
}

void LSystemLayer::Update() {
  auto& times = GetApplication().GetTimes();
  const double update_start = times.Now();
  ProfileFrame frame{};

  if (!auto_grow) {
    return;
  }

  const auto scene = GetScene();
  if (!scene) {
    return;
  }

  const float dt = static_cast<float>(times.DeltaTime());
  if (dt > 0.0f) {
    const float fps = 1.0f / dt;
    if (fps < kAutoGrowFailsafeMinFps) {
      auto_grow = false;
      fps_failsafe_tripped_ = true;
      last_failsafe_fps_ = fps;
      return;
    }
  }

  const float delta_days = std::max(0.0f, chronological_days_per_second) * std::max(0.0f, dt);
  if (seasonality_enabled && delta_days > 0.0f) {
    simulation_day_of_year = NormalizeDayOfYear(simulation_day_of_year + delta_days);
  }
  const float delta_years = seasonality_enabled ? (delta_days / 365.0f) : 0.0f;

  if (const auto* pine_entities_ptr = scene->UnsafeGetPrivateComponentOwnersList<ScotsPine>()) {
    const std::vector<Entity> pine_entities = *pine_entities_ptr;
    for (const auto& entity : pine_entities) {
      if (!scene->IsEntityValid(entity)) {
        continue;
      }

      auto pine = scene->GetOrSetPrivateComponent<ScotsPine>(entity).lock();
      if (!pine) {
        continue;
      }
      frame.pine_count++;

      const PineTemporalSample pine_temporal = SamplePineTemporalParameters(*pine);
      const bool pine_in_active_season =
          !seasonality_enabled ||
          IsInActiveSeason(simulation_day_of_year, pine_temporal.season_start_day, pine_temporal.season_end_day);
      const float pine_delta_gdd =
          pine_in_active_season ? std::max(0.0f, pine_temporal.gdd_per_day) * delta_days : 0.0f;

      const float pine_season_days =
          (pine_temporal.season_end_day >= pine_temporal.season_start_day)
              ? static_cast<float>(pine_temporal.season_end_day - pine_temporal.season_start_day + 1)
              : static_cast<float>(365 - pine_temporal.season_start_day + pine_temporal.season_end_day + 1);
      const float pine_season_length_years = std::max(1e-3f, pine_season_days / 365.0f);

      pine->SetSeasonalChronologicalMode(seasonality_enabled);

      if (seasonality_enabled && delta_years > 0.0f) {
        if (pine_in_active_season) {
          if (!pine->growth_model.IsInitialized()) {
            if (const auto descriptor = pine->descriptor_ref.Get<ScotsPineDescriptor>()) {
              std::shared_ptr<ScotsPineDescriptor> post_descriptor = nullptr;
              float repot_switch_gdd = -1.0f;
              if (pine->enable_repot_profile_switch) {
                post_descriptor = pine->post_repot_descriptor_ref.Get<ScotsPineDescriptor>();
                if (post_descriptor) {
                  repot_switch_gdd = std::max(0.0f, pine->repot_switch_gdd);
                }
              }

              pine->growth_model.Initialize(*descriptor, pine->seed, glm::vec3(0), kDefaultRootRotation,
                                            post_descriptor.get(), repot_switch_gdd,
                                            ScotsPine::IsGenerateNeedleTopologyEnabled());
            }
          }
          pine->growth_model.AdvanceChronologicalYears(delta_years);
        } else {
          pine->AdvanceChronologicalAging(delta_years);
        }
      }

      if (!pine_in_active_season) {
        if (pine->growth_model.IsInitialized()) {
          pine->growth_model.graph.data.clock.SyncSeasonalState(seasonality_enabled, pine_in_active_season,
                                                                pine_season_length_years);
        }
        continue;
      }

      if (pine->growth_model.IsInitialized()) {
        pine->growth_model.graph.data.clock.SyncSeasonalState(seasonality_enabled, pine_in_active_season,
                                                              pine_season_length_years);
      }

      const float descriptor_target_gdd = SampleDescriptorTargetGddForPine(*pine);
      const float next_target_gdd = std::max(0.0f, pine->target_gdd + pine_delta_gdd);
      pine->target_gdd =
          descriptor_target_gdd >= 0.0f ? std::min(next_target_gdd, descriptor_target_gdd) : next_target_gdd;

      pine->GrowToTargetGDD();

      if (profiling_enabled) {
        frame.grow_ms += pine->last_grow_seconds * 1000.0;
        frame.rebuild_ms += pine->last_rebuild_seconds * 1000.0;
        frame.growth_steps += pine->growth_model.last_growth_steps;
        frame.node_count += pine->last_node_count;
        frame.internode_count += pine->last_internode_count;
        frame.needle_count += pine->last_needle_count;
        frame.invalid_instance_count += pine->last_invalid_instance_count;
      }
    }
  }

  if (profiling_enabled) {
    frame.update_ms = (times.Now() - update_start) * 1000.0;
    last_profile_frame = frame;
    PushProfileFrame(frame);
  }
}

void LSystemLayer::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  auto reset_all_lsystems = [this]() {
    const auto scene = GetScene();
    if (!scene) {
      return;
    }

    unsigned int base_seed = 0u;
    if (reseed_on_reset) {
      base_seed = static_cast<unsigned int>(std::chrono::steady_clock::now().time_since_epoch().count() & 0xffffffffu);
    }
    unsigned int seed_offset = 0u;

    if (const auto* pine_entities_ptr = scene->UnsafeGetPrivateComponentOwnersList<ScotsPine>()) {
      const std::vector<Entity> pine_entities = *pine_entities_ptr;
      for (const auto& entity : pine_entities) {
        if (!scene->IsEntityValid(entity)) {
          continue;
        }
        auto pine = scene->GetOrSetPrivateComponent<ScotsPine>(entity).lock();
        if (!pine) {
          continue;
        }

        if (reseed_on_reset) {
          pine->seed = base_seed + seed_offset++;
        }
        // Hard reset for Ctrl+W: clear all generated geometry and leave the
        // plant at zero thermal target (no warm-start regrowth).
        pine->target_gdd = 0.0f;
        pine->growth_model.Reset();
        pine->ClearGeometryEntities();
      }
    }

    simulation_day_of_year = NormalizeDayOfYear(static_cast<float>(std::clamp(season_start_day, 0, 364)));
  };

  const auto left_ctrl_state = EditorLayer::GetKey(GLFW_KEY_LEFT_CONTROL);
  const auto right_ctrl_state = EditorLayer::GetKey(GLFW_KEY_RIGHT_CONTROL);
  const bool ctrl_down =
      left_ctrl_state == Input::KeyActionType::Hold || left_ctrl_state == Input::KeyActionType::Press ||
      right_ctrl_state == Input::KeyActionType::Hold || right_ctrl_state == Input::KeyActionType::Press;

  if (ctrl_down) {
    if (EditorLayer::GetKey(GLFW_KEY_F) == Input::KeyActionType::Press) {
      auto_grow = !auto_grow;
      if (auto_grow) {
        fps_failsafe_tripped_ = false;
        last_failsafe_fps_ = 0.0f;
      }
    }
    if (EditorLayer::GetKey(GLFW_KEY_W) == Input::KeyActionType::Press) {
      auto& app = GetApplication();
      if (app.IsPlaying()) {
        app.Stop();
      }
      auto_grow = false;
      reset_all_lsystems();
    }
  }

  if (ImGui::Checkbox("Auto-Grow (Ctrl+F)", &auto_grow) && auto_grow) {
    fps_failsafe_tripped_ = false;
    last_failsafe_fps_ = 0.0f;
  }
  ImGui::TextDisabled("Thermal rates are descriptor-owned and sampled per pine.");

  ImGui::SeparatorText("Seasonality");
  ImGui::Checkbox("Enable Calendar Seasonality", &seasonality_enabled);
  if (ImGui::DragInt("Season Start Day", &season_start_day, 1.0f, 0, 364)) {
    season_start_day = std::clamp(season_start_day, 0, 364);
    simulation_day_of_year = NormalizeDayOfYear(static_cast<float>(season_start_day));
  }
  ImGui::DragInt("Season End Day", &season_end_day, 1.0f, 0, 364);
  ImGui::DragFloat("Calendar Days/sec (Chronology)", &chronological_days_per_second, 0.25f, 0.0f, 365.0f, "%.2f");
  if (ImGui::DragFloat("Simulation Day Of Year", &simulation_day_of_year, 0.25f, 0.0f, 364.999f, "%.2f")) {
    simulation_day_of_year = NormalizeDayOfYear(simulation_day_of_year);
  }
  const bool inspector_active_season = IsInActiveSeason(simulation_day_of_year, season_start_day, season_end_day);
  ImGui::Text("Season State: %s", inspector_active_season ? "Active" : "Dormant");

  ImGui::Checkbox("Reseed on Reset (Ctrl+W when stopped)", &reseed_on_reset);
  if (fps_failsafe_tripped_) {
    ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.2f, 1.0f), "Auto-grow stopped by 1 FPS failsafe (last: %.2f FPS).",
                       last_failsafe_fps_);
    ImGui::TextDisabled("Re-enable Auto-Grow to resume growth.");
  }

  if (ImGui::Checkbox("Scots Pine Stem-Only Mode", &pine_stem_only_mode)) {
    ApplyPineStemOnlyMode(GetScene(), pine_stem_only_mode, true);
  }

  if (ImGui::Checkbox("Enable Scene/Plant View Tint", &scene_plant_view_tint_enabled)) {
    ApplyGlobalPlantColorMode(tassel_color_mode, scene_plant_view_tint_enabled);
    RebuildAllPlantGeometry(GetScene());
  }
  if (!scene_plant_view_tint_enabled) {
    ImGui::TextDisabled("Tint disabled: effective mode forced to Shaded.");
  }

  {
    const char* color_mode_items[] = {
        "Shaded", "By Type", "By Instance", "By Node", "Needle Lignification", "Needle Stripe Proxy", "Needle Sheath"};
    if (ImGui::Combo("Plant Color Mode", &tassel_color_mode, color_mode_items, IM_ARRAYSIZE(color_mode_items))) {
      tassel_color_mode = ClampColorModeIndex(tassel_color_mode);
      ApplyGlobalPlantColorMode(tassel_color_mode, scene_plant_view_tint_enabled);
      RebuildAllPlantGeometry(GetScene());
    }
  }

  if (ImGui::Button("Reset Plant Color View (Shaded)")) {
    tassel_color_mode = 0;
    scene_plant_view_tint_enabled = false;
    ApplyGlobalPlantColorMode(tassel_color_mode, scene_plant_view_tint_enabled);
    RebuildAllPlantGeometry(GetScene());
  }

  if (ImGui::Button("Reset All LSystems (Ctrl+W)")) {
    reset_all_lsystems();
  }

  ImGui::Separator();
  ImGui::Checkbox("Enable LSystem Profiling", &profiling_enabled);
  ImGui::DragInt("Profile History Size", &profiling_history_size, 1.0f, 30, 4000);

  static char export_path_buffer[260] = "lsystem_profile.csv";
  static std::string last_loaded_export_path;
  if (last_loaded_export_path != profiling_export_path && profiling_export_path.size() < sizeof(export_path_buffer)) {
    std::snprintf(export_path_buffer, sizeof(export_path_buffer), "%s", profiling_export_path.c_str());
    last_loaded_export_path = profiling_export_path;
  }
  if (ImGui::InputText("Profile CSV Path", export_path_buffer, sizeof(export_path_buffer))) {
    profiling_export_path = export_path_buffer;
    last_loaded_export_path = profiling_export_path;
  }

  if (ImGui::Button("Export Profile CSV")) {
    ExportProfileCsv(profiling_export_path);
  }
  if (ImGui::Button("Clear Profile History")) {
    profiling_history.clear();
  }

  if (profiling_enabled) {
    const double avg_update_ms = AverageProfileMetric(profiling_history, [](const ProfileFrame& f) {
      return f.update_ms;
    });
    const double avg_grow_ms = AverageProfileMetric(profiling_history, [](const ProfileFrame& f) {
      return f.grow_ms;
    });
    const double avg_rebuild_ms = AverageProfileMetric(profiling_history, [](const ProfileFrame& f) {
      return f.rebuild_ms;
    });

    const double max_update_ms = MaxProfileMetric(profiling_history, [](const ProfileFrame& f) {
      return f.update_ms;
    });
    const double max_grow_ms = MaxProfileMetric(profiling_history, [](const ProfileFrame& f) {
      return f.grow_ms;
    });
    const double max_rebuild_ms = MaxProfileMetric(profiling_history, [](const ProfileFrame& f) {
      return f.rebuild_ms;
    });

    ImGui::SeparatorText("LSystem Profiling (Rolling)");
    ImGui::Text("History Frames: %d", static_cast<int>(profiling_history.size()));
    ImGui::Text("Last Update: %.3f ms", last_profile_frame.update_ms);
    ImGui::Text("Last Grow: %.3f ms", last_profile_frame.grow_ms);
    ImGui::Text("Last Rebuild: %.3f ms", last_profile_frame.rebuild_ms);
    ImGui::Text("Last Pines: %u", last_profile_frame.pine_count);
    ImGui::Text("Last Growth Steps: %u", last_profile_frame.growth_steps);
    ImGui::Text("Last Nodes/Internodes/Needles: %u / %u / %u", last_profile_frame.node_count,
                last_profile_frame.internode_count, last_profile_frame.needle_count);
    ImGui::Text("Last Invalid Instances: %u", last_profile_frame.invalid_instance_count);

    ImGui::SeparatorText("Averages / Maxima");
    ImGui::Text("Update ms avg/max: %.3f / %.3f", avg_update_ms, max_update_ms);
    ImGui::Text("Grow ms avg/max: %.3f / %.3f", avg_grow_ms, max_grow_ms);
    ImGui::Text("Rebuild ms avg/max: %.3f / %.3f", avg_rebuild_ms, max_rebuild_ms);
  }
}

void LSystemLayer::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "auto_grow" << YAML::Value << auto_grow;
  out << YAML::Key << "seasonality_enabled" << YAML::Value << seasonality_enabled;
  out << YAML::Key << "season_start_day" << YAML::Value << season_start_day;
  out << YAML::Key << "season_end_day" << YAML::Value << season_end_day;
  out << YAML::Key << "chronological_days_per_second" << YAML::Value << chronological_days_per_second;
  out << YAML::Key << "simulation_day_of_year" << YAML::Value << NormalizeDayOfYear(simulation_day_of_year);
  out << YAML::Key << "reseed_on_reset" << YAML::Value << reseed_on_reset;
  out << YAML::Key << "tassel_color_mode" << YAML::Value << tassel_color_mode;
  out << YAML::Key << "scene_plant_view_tint_enabled" << YAML::Value << scene_plant_view_tint_enabled;
  out << YAML::Key << "pine_stem_only_mode" << YAML::Value << pine_stem_only_mode;
  out << YAML::Key << "profiling_enabled" << YAML::Value << profiling_enabled;
  out << YAML::Key << "profiling_history_size" << YAML::Value << profiling_history_size;
  out << YAML::Key << "profiling_export_path" << YAML::Value << profiling_export_path;
}

void LSystemLayer::Deserialize(const YAML::Node& in) {
  if (in["auto_grow"]) {
    auto_grow = in["auto_grow"].as<bool>();
  }
  if (in["seasonality_enabled"]) {
    seasonality_enabled = in["seasonality_enabled"].as<bool>();
  }
  if (in["season_start_day"]) {
    season_start_day = std::clamp(in["season_start_day"].as<int>(), 0, 364);
  }
  if (in["season_end_day"]) {
    season_end_day = std::clamp(in["season_end_day"].as<int>(), 0, 364);
  }
  if (in["chronological_days_per_second"]) {
    chronological_days_per_second = std::max(0.0f, in["chronological_days_per_second"].as<float>());
  }
  simulation_day_of_year = NormalizeDayOfYear(static_cast<float>(season_start_day));
  if (in["simulation_day_of_year"]) {
    simulation_day_of_year = NormalizeDayOfYear(in["simulation_day_of_year"].as<float>());
  }
  if (in["reseed_on_reset"]) {
    reseed_on_reset = in["reseed_on_reset"].as<bool>();
  }
  if (in["tassel_color_mode"]) {
    tassel_color_mode = ClampColorModeIndex(in["tassel_color_mode"].as<int>());
  }
  if (in["scene_plant_view_tint_enabled"]) {
    scene_plant_view_tint_enabled = in["scene_plant_view_tint_enabled"].as<bool>();
  }
  if (in["pine_stem_only_mode"]) {
    pine_stem_only_mode = in["pine_stem_only_mode"].as<bool>();
  }
  if (in["profiling_enabled"]) {
    profiling_enabled = in["profiling_enabled"].as<bool>();
  }
  if (in["profiling_history_size"]) {
    profiling_history_size = std::max(30, in["profiling_history_size"].as<int>());
  }
  if (in["profiling_export_path"]) {
    profiling_export_path = in["profiling_export_path"].as<std::string>();
  }

  ApplyGlobalPlantColorMode(tassel_color_mode, scene_plant_view_tint_enabled);
  ApplyPineStemOnlyMode(GetScene(), pine_stem_only_mode, true);
  RebuildAllPlantGeometry(GetScene());
}
