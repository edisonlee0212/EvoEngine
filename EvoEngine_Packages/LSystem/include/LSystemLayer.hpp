#pragma once

#include "ILayer.hpp"
#include "Input.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace l_system_package {
class SorghumLSDescriptor;
bool InspectLSystemLayer(evo_engine::InspectorContext& context, class LSystemLayer& layer);
void SerializeLSystemLayer(YAML::Emitter& out, const class LSystemLayer& target);
void DeserializeLSystemLayer(const YAML::Node& in, class LSystemLayer& target);

class LSystemLayer : public evo_engine::ILayer {
  friend bool InspectLSystemLayer(evo_engine::InspectorContext& context, LSystemLayer& layer);
  friend void SerializeLSystemLayer(YAML::Emitter& out, const LSystemLayer& target);
  friend void DeserializeLSystemLayer(const YAML::Node& in, LSystemLayer& target);

 public:
  struct ProfileFrame {
    double update_ms = 0.0;
    double grow_ms = 0.0;
    double growth_rules_ms = 0.0;
    double topology_rules_ms = 0.0;
    double sort_lists_ms = 0.0;
    double update_node_info_ms = 0.0;
    double propagate_geometry_ms = 0.0;
    double topology_scan_ms = 0.0;
    double rebuild_ms = 0.0;
    double internode_rebuild_ms = 0.0;
    double leaf_spline_ms = 0.0;
    double leaf_mesh_ms = 0.0;
    double mesh_upload_ms = 0.0;
    uint32_t pine_count = 0;
    uint32_t sorghum_count = 0;
    uint32_t growth_steps = 0;
    uint32_t node_count = 0;
    uint32_t internode_count = 0;
    uint32_t needle_count = 0;
    uint32_t leaf_count = 0;
    uint32_t live_leaf_count = 0;
    uint32_t invalid_instance_count = 0;
  };

  bool auto_grow = false;
  bool auto_grow_fps_failsafe_enabled = true;
  bool reseed_on_reset = false;

  bool seasonality_enabled = false;
  int season_start_day = 60;
  int season_end_day = 334;
  float chronological_days_per_second = 30.0f;
  float auto_grow_max_delta_time = 1.0f / 30.0f;
  float simulation_day_of_year = 60.0f;
  uint32_t sorghum_growth_step_cap_per_update = 0;

  int tassel_color_mode = 0;
  bool scene_plant_view_tint_enabled = true;
  bool pine_stem_only_mode = false;

  bool profiling_enabled = false;
  int profiling_history_size = 240;
  std::string profiling_export_path = "lsystem_profile.csv";

  ProfileFrame last_profile_frame{};
  std::vector<ProfileFrame> profiling_history{};

  void OnCreate() override;
  void OnDestroy() override;
  void PreUpdate() override;
  void Update() override;
  void ExportProfileCsv(const std::string& path) const;
  size_t RegenerateSorghumScene(float evaluation_gdd, int seed_base = -1, bool update_render_geometry = true) const;
  size_t AdvanceSorghumScene(float evaluation_gdd, int seed_base = -1, bool update_render_geometry = true) const;
  size_t RestoreSorghumScene(bool update_render_geometry = true) const;
  size_t RegenerateSorghumDescriptor(const SorghumLSDescriptor& descriptor, bool representative_only, bool preview,
                                     float preview_target_gdd = 1000.0f, uint32_t preview_max_growth_steps = 64) const;

 private:
  static constexpr float kAutoGrowFailsafeMinFps = 1.0f;
  bool fps_failsafe_tripped_ = false;
  float last_failsafe_fps_ = 0.0f;

  void PushProfileFrame(const ProfileFrame& frame);
};

}  // namespace l_system_package
