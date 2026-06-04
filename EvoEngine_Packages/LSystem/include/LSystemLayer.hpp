#pragma once

#include "ILayer.hpp"
#include "Input.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace l_system_package {

class LSystemLayer : public evo_engine::ILayer {
 public:
  struct ProfileFrame {
    double update_ms = 0.0;
    double grow_ms = 0.0;
    double rebuild_ms = 0.0;
    uint32_t pine_count = 0;
    uint32_t growth_steps = 0;
    uint32_t node_count = 0;
    uint32_t internode_count = 0;
    uint32_t needle_count = 0;
    uint32_t invalid_instance_count = 0;
  };

  bool auto_grow = false;
  bool reseed_on_reset = false;

  bool seasonality_enabled = false;
  int season_start_day = 60;
  int season_end_day = 334;
  float chronological_days_per_second = 30.0f;
  float simulation_day_of_year = 60.0f;

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
  void Update() override;
  void OnInspect(const std::shared_ptr<evo_engine::EditorLayer>& editor_layer) override;
  void Serialize(YAML::Emitter& out) const;
  void Deserialize(const YAML::Node& in);

 private:
  static constexpr float kAutoGrowFailsafeMinFps = 1.0f;
  bool fps_failsafe_tripped_ = false;
  float last_failsafe_fps_ = 0.0f;

  void PushProfileFrame(const ProfileFrame& frame);
  void ExportProfileCsv(const std::string& path) const;
};

}  // namespace l_system_package
