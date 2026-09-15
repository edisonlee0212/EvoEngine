#include "EditorLayer.hpp"
#include "LSystemInspectionAdapters.hpp"
#include "LSystemLayer.hpp"
#include "LSystemSimulation.hpp"
#include "Scene.hpp"
#include "ScotsPine.hpp"
#include "Times.hpp"

using namespace evo_engine;
using namespace l_system_package;
using namespace l_system_package::simulation;
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
}  // namespace
bool l_system_package::InspectLSystemLayer(InspectorContext& context, LSystemLayer& layer) {
  (void)context;
  auto& auto_grow = layer.auto_grow;
  auto& reseed_on_reset = layer.reseed_on_reset;
  auto& seasonality_enabled = layer.seasonality_enabled;
  auto& season_start_day = layer.season_start_day;
  auto& season_end_day = layer.season_end_day;
  auto& chronological_days_per_second = layer.chronological_days_per_second;
  auto& simulation_day_of_year = layer.simulation_day_of_year;
  auto& tassel_color_mode = layer.tassel_color_mode;
  auto& scene_plant_view_tint_enabled = layer.scene_plant_view_tint_enabled;
  auto& pine_stem_only_mode = layer.pine_stem_only_mode;
  auto& profiling_enabled = layer.profiling_enabled;
  auto& profiling_history_size = layer.profiling_history_size;
  auto& profiling_export_path = layer.profiling_export_path;
  auto& last_profile_frame = layer.last_profile_frame;
  auto& profiling_history = layer.profiling_history;
  auto& fps_failsafe_tripped_ = layer.fps_failsafe_tripped_;
  auto& last_failsafe_fps_ = layer.last_failsafe_fps_;

  auto reset_all_lsystems = [&]() {
    const auto scene = layer.GetScene();
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
      auto& app = layer.GetApplication();
      if (app.IsPlaying()) {
        app.Stop();
      }
      auto_grow = false;
      reset_all_lsystems();
    }
  }

  const auto window_title = layer.GetLayerName();
  bool open = layer.enable_inspection;
  if (!ImGui::Begin(window_title.c_str(), &open)) {
    ImGui::End();
    layer.enable_inspection = open;
    return false;
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
    ApplyPineStemOnlyMode(layer.GetScene(), pine_stem_only_mode, true);
  }

  if (ImGui::Checkbox("Enable Scene/Plant View Tint", &scene_plant_view_tint_enabled)) {
    ApplyGlobalPlantColorMode(tassel_color_mode, scene_plant_view_tint_enabled);
    RebuildAllPlantGeometry(layer.GetScene());
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
      RebuildAllPlantGeometry(layer.GetScene());
    }
  }

  if (ImGui::Button("Reset Plant Color View (Shaded)")) {
    tassel_color_mode = 0;
    scene_plant_view_tint_enabled = false;
    ApplyGlobalPlantColorMode(tassel_color_mode, scene_plant_view_tint_enabled);
    RebuildAllPlantGeometry(layer.GetScene());
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
    layer.ExportProfileCsv(profiling_export_path);
  }
  if (ImGui::Button("Clear Profile History")) {
    profiling_history.clear();
  }

  if (profiling_enabled) {
    const double avg_update_ms = AverageProfileMetric(profiling_history, [](const LSystemLayer::ProfileFrame& f) {
      return f.update_ms;
    });
    const double avg_grow_ms = AverageProfileMetric(profiling_history, [](const LSystemLayer::ProfileFrame& f) {
      return f.grow_ms;
    });
    const double avg_rebuild_ms = AverageProfileMetric(profiling_history, [](const LSystemLayer::ProfileFrame& f) {
      return f.rebuild_ms;
    });

    const double max_update_ms = MaxProfileMetric(profiling_history, [](const LSystemLayer::ProfileFrame& f) {
      return f.update_ms;
    });
    const double max_grow_ms = MaxProfileMetric(profiling_history, [](const LSystemLayer::ProfileFrame& f) {
      return f.grow_ms;
    });
    const double max_rebuild_ms = MaxProfileMetric(profiling_history, [](const LSystemLayer::ProfileFrame& f) {
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
  ImGui::End();
  layer.enable_inspection = open;
  return false;
}
