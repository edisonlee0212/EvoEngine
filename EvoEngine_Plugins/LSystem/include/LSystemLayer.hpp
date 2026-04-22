#pragma once

#include "ILayer.hpp"
#include "Input.hpp"
#include <cstdint>
#include <string>
#include <vector>

namespace l_system_plugin {

class LSystemLayer : public evo_engine::ILayer {
 public:
  struct ProfileFrame {
    double update_ms = 0.0;
    double grow_ms = 0.0;
    double rebuild_ms = 0.0;
    double apply_growth_rules_ms = 0.0;
    double apply_topology_rules_ms = 0.0;
    double sort_lists_ms = 0.0;
    double update_node_info_ms = 0.0;
    double propagate_geometry_ms = 0.0;
    double topology_scan_ms = 0.0;
    double rebuild_internode_collect_ms = 0.0;
    double rebuild_internode_upload_ms = 0.0;
    double rebuild_spikelet_collect_ms = 0.0;
    double rebuild_spikelet_upload_ms = 0.0;
    uint32_t tassel_count = 0;
    uint32_t growth_steps = 0;
    uint32_t node_count = 0;
    uint32_t internode_count = 0;
    uint32_t spikelet_count = 0;
    uint32_t invalid_instance_count = 0;
  };

  /// When true, all MaizeTassel components auto-grow each frame.
  bool auto_grow = false;

  /// When true, Ctrl+W reset assigns fresh seeds to every tassel before regeneration.
  bool reseed_on_reset = false;

  /// Simulated GDD accumulation per second during auto-grow.
  float gdd_per_second = 60.0f;

  /// Optional per-frame GDD cap during auto-grow (0 = unlimited).
  float max_gdd_per_frame = 10.0f;

  /// Optional per-tassel growth step cap per frame (0 = unlimited).
  int max_growth_steps_per_frame = 10;

  /// Global tassel coloring mode: current, by module type, or by tassel instance.
  int tassel_color_mode = 0;

  /// Profiling controls.
  bool profiling_enabled = false;
  int profiling_history_size = 240;
  std::string profiling_export_path = "lsystem_profile.csv";

  /// Last frame + rolling history (not serialized as assets).
  ProfileFrame last_profile_frame{};
  std::vector<ProfileFrame> profiling_history{};

  void OnCreate() override;
  void OnDestroy() override;
  void Update() override;
  void OnInspect(const std::shared_ptr<evo_engine::EditorLayer>& editor_layer) override;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;

 private:
    static constexpr float kAutoGrowFailsafeMinFps = 5.0f;
    bool fps_failsafe_tripped_ = false;
    float last_failsafe_fps_ = 0.0f;

  void PushProfileFrame(const ProfileFrame& frame);
  void ExportProfileCsv(const std::string& path) const;
};

}  // namespace l_system_plugin
