#include "LSystemLayer.hpp"
#include "ClassRegistry.hpp"
#include "EditorLayer.hpp"
#include "LSystemDescriptor.hpp"
#include "MaizeTassel.hpp"
#include "MaizeTasselDescriptor.hpp"
#include "Scene.hpp"
#include "Application.hpp"
#include "Times.hpp"
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <numeric>

using namespace l_system_plugin;
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
}  // namespace

AssetRegistration<LSystemDescriptor> lsys_desc_registry("LSystemDescriptor", {".lsys"});
AssetRegistration<MaizeTasselDescriptor> maize_tassel_desc_registry("MaizeTasselDescriptor", {".mtassel"});
PrivateComponentRegistration<MaizeTassel> maize_tassel_component_registry("MaizeTassel");

void LSystemLayer::OnCreate() {
  MaizeTassel::SetGlobalColorMode(static_cast<MaizeTassel::ColorMode>(std::clamp(tassel_color_mode, 0, 3)));
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

  out << "frame,update_ms,grow_ms,rebuild_ms,"
      << "apply_growth_rules_ms,apply_topology_rules_ms,sort_lists_ms,update_node_info_ms,"
      << "propagate_geometry_ms,topology_scan_ms,"
      << "rebuild_internode_collect_ms,rebuild_internode_upload_ms,rebuild_spikelet_collect_ms,rebuild_spikelet_upload_ms,"
      << "tassels,growth_steps,nodes,internodes,spikelets,invalid_instances\n";
  for (size_t i = 0; i < profiling_history.size(); i++) {
    const auto& f = profiling_history[i];
    out << i << ","
        << std::fixed << std::setprecision(4)
        << f.update_ms << ","
        << f.grow_ms << ","
        << f.rebuild_ms << ","
        << f.apply_growth_rules_ms << ","
        << f.apply_topology_rules_ms << ","
        << f.sort_lists_ms << ","
        << f.update_node_info_ms << ","
        << f.propagate_geometry_ms << ","
        << f.topology_scan_ms << ","
        << f.rebuild_internode_collect_ms << ","
        << f.rebuild_internode_upload_ms << ","
        << f.rebuild_spikelet_collect_ms << ","
        << f.rebuild_spikelet_upload_ms << ","
        << f.tassel_count << ","
        << f.growth_steps << ","
        << f.node_count << ","
        << f.internode_count << ","
        << f.spikelet_count << ","
        << f.invalid_instance_count << "\n";
  }
}

void LSystemLayer::Update() {
  const double update_start = Times::Now();
  ProfileFrame frame{};

  if (!auto_grow)
    return;

  const auto scene = GetScene();
  if (!scene)
    return;

  const float dt = static_cast<float>(Times::DeltaTime());
  if (dt > 0.0f) {
    const float fps = 1.0f / dt;
    if (fps < kAutoGrowFailsafeMinFps) {
      auto_grow = false;
      fps_failsafe_tripped_ = true;
      last_failsafe_fps_ = fps;
      return;
    }
  }

  const float raw_delta_gdd = gdd_per_second * dt;
  const float delta_gdd = max_gdd_per_frame > 0.0f
                              ? std::min(raw_delta_gdd, max_gdd_per_frame)
                              : raw_delta_gdd;

  const auto* tassel_entities_ptr = scene->UnsafeGetPrivateComponentOwnersList<MaizeTassel>();
  if (!tassel_entities_ptr)
    return;

  // Copy before iterating: GrowToTargetGDD() can create geometry entities with new component types
  // (MeshRenderer, Particles) for the first time, causing p_owners_collections_list_ to reallocate
  // and invalidating the raw pointer returned by UnsafeGetPrivateComponentOwnersList.
  const std::vector<Entity> tassel_entities = *tassel_entities_ptr;

  for (const auto& entity : tassel_entities) {
    if (!scene->IsEntityValid(entity))
      continue;
    auto tassel = scene->GetOrSetPrivateComponent<MaizeTassel>(entity).lock();
    if (!tassel)
      continue;
    frame.tassel_count++;
    tassel->target_gdd += delta_gdd;
    tassel->max_growth_steps_per_frame =
        static_cast<uint32_t>(std::max(0, max_growth_steps_per_frame));
    tassel->GrowToTargetGDD();

    if (profiling_enabled) {
      const auto& grow_profile = tassel->growth_model.last_grow_to_gdd_profile;
      frame.grow_ms += tassel->last_grow_seconds * 1000.0;
      frame.rebuild_ms += tassel->last_rebuild_seconds * 1000.0;
      frame.apply_growth_rules_ms += grow_profile.apply_growth_rules_seconds * 1000.0;
      frame.apply_topology_rules_ms += grow_profile.apply_topology_rules_seconds * 1000.0;
      frame.sort_lists_ms += grow_profile.sort_lists_seconds * 1000.0;
      frame.update_node_info_ms += grow_profile.update_node_info_seconds * 1000.0;
      frame.propagate_geometry_ms += grow_profile.propagate_geometry_seconds * 1000.0;
      frame.topology_scan_ms += grow_profile.topology_scan_seconds * 1000.0;
      frame.rebuild_internode_collect_ms +=
        tassel->last_rebuild_internode_collect_seconds * 1000.0;
      frame.rebuild_internode_upload_ms +=
        tassel->last_rebuild_internode_upload_seconds * 1000.0;
      frame.rebuild_spikelet_collect_ms +=
        tassel->last_rebuild_spikelet_collect_seconds * 1000.0;
      frame.rebuild_spikelet_upload_ms +=
        tassel->last_rebuild_spikelet_upload_seconds * 1000.0;
      frame.growth_steps += tassel->growth_model.last_growth_steps;
      frame.node_count += tassel->last_node_count;
      frame.internode_count += tassel->last_internode_count;
      frame.spikelet_count += tassel->last_spikelet_count;
      frame.invalid_instance_count += tassel->last_invalid_instance_count;
    }
  }

  if (profiling_enabled) {
    frame.update_ms = (Times::Now() - update_start) * 1000.0;
    last_profile_frame = frame;
    PushProfileFrame(frame);
  }
}

void LSystemLayer::OnInspect(const std::shared_ptr<evo_engine::EditorLayer>& editor_layer) {
  auto reset_all_tassels = [this]() {
    const auto scene = GetScene();
    if (!scene)
      return;

    const auto* tassel_entities_ptr = scene->UnsafeGetPrivateComponentOwnersList<MaizeTassel>();
    if (!tassel_entities_ptr)
      return;

    const std::vector<Entity> tassel_entities = *tassel_entities_ptr;
    unsigned int base_seed = 0u;
    if (reseed_on_reset) {
      base_seed = static_cast<unsigned int>(
          std::chrono::steady_clock::now().time_since_epoch().count() & 0xFFFFFFFFu);
    }

    unsigned int seed_offset = 0u;
    for (const auto& entity : tassel_entities) {
      if (!scene->IsEntityValid(entity))
        continue;
      auto tassel = scene->GetOrSetPrivateComponent<MaizeTassel>(entity).lock();
      if (!tassel)
        continue;

      if (reseed_on_reset) {
        tassel->seed = base_seed + seed_offset;
        seed_offset++;
      }

      tassel->target_gdd = 0.0f;
      tassel->GenerateGeometryEntities();
    }

    auto_grow = false;
  };

  // --- Keyboard shortcuts (polling, matching EcoSysLab pattern) ---
  if (EditorLayer::GetKey(GLFW_KEY_LEFT_CONTROL) == Input::KeyActionType::Hold ||
      EditorLayer::GetKey(GLFW_KEY_RIGHT_CONTROL) == Input::KeyActionType::Hold) {
    if (EditorLayer::GetKey(GLFW_KEY_F) == Input::KeyActionType::Press) {
      auto_grow = !auto_grow;
      if (auto_grow) {
        fps_failsafe_tripped_ = false;
        last_failsafe_fps_ = 0.0f;
      }
    }
    if (EditorLayer::GetKey(GLFW_KEY_W) == Input::KeyActionType::Press) {
      reset_all_tassels();
    }
  }

  // --- UI ---
  if (ImGui::Checkbox("Auto-Grow (Ctrl+F)", &auto_grow) && auto_grow) {
    fps_failsafe_tripped_ = false;
    last_failsafe_fps_ = 0.0f;
  }
  ImGui::DragFloat("GDD/sec", &gdd_per_second, 1.0f, 1.0f, 500.0f);
  ImGui::DragFloat("Max GDD/Frame (0=Unlimited)", &max_gdd_per_frame, 0.1f, 0.0f, 500.0f);
  ImGui::DragInt("Max Growth Steps/Frame (0=Unlimited)",
                 &max_growth_steps_per_frame,
                 1.0f,
                 0,
                 5000);
  ImGui::TextDisabled("Use both caps to smooth auto-grow after long frames.");
  ImGui::Checkbox("Reseed on Reset (Ctrl+W)", &reseed_on_reset);
  if (fps_failsafe_tripped_) {
    ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.2f, 1.0f),
                       "Auto-grow stopped by 5 FPS failsafe (last: %.2f FPS).",
                       last_failsafe_fps_);
    ImGui::TextDisabled("Re-enable Auto-Grow to resume growth.");
  }

  {
    const char* color_mode_items[] = {"Shaded", "By Type", "By Instance", "By Node"};
    if (ImGui::Combo("Tassel Color Mode", &tassel_color_mode, color_mode_items, IM_ARRAYSIZE(color_mode_items))) {
      tassel_color_mode = std::clamp(tassel_color_mode, 0, 3);
      MaizeTassel::SetGlobalColorMode(static_cast<MaizeTassel::ColorMode>(tassel_color_mode));

      const auto scene = GetScene();
      if (scene) {
        if (const auto* tassel_entities_ptr = scene->UnsafeGetPrivateComponentOwnersList<MaizeTassel>()) {
          const std::vector<Entity> tassel_entities = *tassel_entities_ptr;
          for (const auto& entity : tassel_entities) {
            if (!scene->IsEntityValid(entity))
              continue;
            auto tassel = scene->GetOrSetPrivateComponent<MaizeTassel>(entity).lock();
            if (tassel) {
              tassel->RebuildGeometry();
            }
          }
        }
      }
    }
  }

  if (ImGui::Button("Reset All Tassels (Ctrl+W)")) {
    reset_all_tassels();
  }

  ImGui::Separator();
  ImGui::Checkbox("Enable LSystem Profiling", &profiling_enabled);
  ImGui::DragInt("Profile History Size", &profiling_history_size, 1.0f, 30, 4000);

  static char export_path_buffer[260] = "lsystem_profile.csv";
  static std::string last_loaded_export_path;
  if (last_loaded_export_path != profiling_export_path &&
      profiling_export_path.size() < sizeof(export_path_buffer)) {
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
    const double avg_apply_growth_rules_ms = AverageProfileMetric(profiling_history, [](const ProfileFrame& f) {
      return f.apply_growth_rules_ms;
    });
    const double avg_apply_topology_rules_ms = AverageProfileMetric(profiling_history, [](const ProfileFrame& f) {
      return f.apply_topology_rules_ms;
    });
    const double avg_sort_lists_ms = AverageProfileMetric(profiling_history, [](const ProfileFrame& f) {
      return f.sort_lists_ms;
    });
    const double avg_propagate_geometry_ms = AverageProfileMetric(profiling_history, [](const ProfileFrame& f) {
      return f.propagate_geometry_ms;
    });
    const double avg_rebuild_internode_collect_ms =
        AverageProfileMetric(profiling_history, [](const ProfileFrame& f) {
          return f.rebuild_internode_collect_ms;
        });
    const double avg_rebuild_spikelet_collect_ms =
        AverageProfileMetric(profiling_history, [](const ProfileFrame& f) {
          return f.rebuild_spikelet_collect_ms;
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
    ImGui::Text("Last Tassels: %u", last_profile_frame.tassel_count);
    ImGui::Text("Last Growth Steps: %u", last_profile_frame.growth_steps);
    ImGui::Text("Last Nodes/Internodes/Spikelets: %u / %u / %u",
                last_profile_frame.node_count,
                last_profile_frame.internode_count,
                last_profile_frame.spikelet_count);
    ImGui::Text("Last Invalid Instances: %u", last_profile_frame.invalid_instance_count);
    ImGui::Text("Last Growth phases ms (rules/topology/sort/prop): %.3f / %.3f / %.3f / %.3f",
                last_profile_frame.apply_growth_rules_ms,
                last_profile_frame.apply_topology_rules_ms,
                last_profile_frame.sort_lists_ms,
                last_profile_frame.propagate_geometry_ms);
    ImGui::Text("Last Rebuild phases ms (internode collect/upload, spikelet collect/upload): %.3f / %.3f, %.3f / %.3f",
                last_profile_frame.rebuild_internode_collect_ms,
                last_profile_frame.rebuild_internode_upload_ms,
                last_profile_frame.rebuild_spikelet_collect_ms,
                last_profile_frame.rebuild_spikelet_upload_ms);

    ImGui::SeparatorText("Averages / Maxima");
    ImGui::Text("Update ms avg/max: %.3f / %.3f", avg_update_ms, max_update_ms);
    ImGui::Text("Grow ms avg/max: %.3f / %.3f", avg_grow_ms, max_grow_ms);
    ImGui::Text("Rebuild ms avg/max: %.3f / %.3f", avg_rebuild_ms, max_rebuild_ms);
    ImGui::Text("Avg Growth phases ms (rules/topology/sort/prop): %.3f / %.3f / %.3f / %.3f",
                avg_apply_growth_rules_ms,
                avg_apply_topology_rules_ms,
                avg_sort_lists_ms,
                avg_propagate_geometry_ms);
    ImGui::Text("Avg Rebuild collect ms (internode/spikelet): %.3f / %.3f",
                avg_rebuild_internode_collect_ms,
                avg_rebuild_spikelet_collect_ms);
  }
}

void LSystemLayer::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "auto_grow" << YAML::Value << auto_grow;
  out << YAML::Key << "gdd_per_second" << YAML::Value << gdd_per_second;
  out << YAML::Key << "max_gdd_per_frame" << YAML::Value << max_gdd_per_frame;
  out << YAML::Key << "max_growth_steps_per_frame" << YAML::Value << max_growth_steps_per_frame;
  out << YAML::Key << "reseed_on_reset" << YAML::Value << reseed_on_reset;
  out << YAML::Key << "tassel_color_mode" << YAML::Value << tassel_color_mode;
  out << YAML::Key << "profiling_enabled" << YAML::Value << profiling_enabled;
  out << YAML::Key << "profiling_history_size" << YAML::Value << profiling_history_size;
  out << YAML::Key << "profiling_export_path" << YAML::Value << profiling_export_path;
}

void LSystemLayer::Deserialize(const YAML::Node& in) {
  if (in["auto_grow"])
    auto_grow = in["auto_grow"].as<bool>();
  if (in["gdd_per_second"])
    gdd_per_second = in["gdd_per_second"].as<float>();
  if (in["max_gdd_per_frame"])
    max_gdd_per_frame = std::max(0.0f, in["max_gdd_per_frame"].as<float>());
  if (in["max_growth_steps_per_frame"])
    max_growth_steps_per_frame = std::max(0, in["max_growth_steps_per_frame"].as<int>());
  if (in["reseed_on_reset"])
    reseed_on_reset = in["reseed_on_reset"].as<bool>();
  if (in["tassel_color_mode"])
    tassel_color_mode = std::clamp(in["tassel_color_mode"].as<int>(), 0, 3);
  MaizeTassel::SetGlobalColorMode(static_cast<MaizeTassel::ColorMode>(tassel_color_mode));
  if (const auto scene = GetScene()) {
    if (const auto* tassel_entities_ptr = scene->UnsafeGetPrivateComponentOwnersList<MaizeTassel>()) {
      const std::vector<Entity> tassel_entities = *tassel_entities_ptr;
      for (const auto& entity : tassel_entities) {
        if (!scene->IsEntityValid(entity))
          continue;
        auto tassel = scene->GetOrSetPrivateComponent<MaizeTassel>(entity).lock();
        if (tassel) {
          tassel->RebuildGeometry();
        }
      }
    }
  }
  if (in["profiling_enabled"])
    profiling_enabled = in["profiling_enabled"].as<bool>();
  if (in["profiling_history_size"])
    profiling_history_size = std::max(30, in["profiling_history_size"].as<int>());
  if (in["profiling_export_path"])
    profiling_export_path = in["profiling_export_path"].as<std::string>();
}
