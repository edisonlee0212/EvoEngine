#pragma once

#include "ILayer.hpp"
#include "Input.hpp"

#include <array>
#include <cstdint>
#include <filesystem>
#include <future>
#include <glm/vec4.hpp>
#include <memory>
#include <string>
#include <vector>

namespace evo_engine {
class Scene;
class Texture2D;
}

namespace l_system_package {
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

  int calendar_start_year = 0;
  float calendar_start_day_of_year = 0.0f;
  float chronological_days_per_second = 30.0f;
  int simulation_year = 0;
  float simulation_day_of_year = 0.0f;
  bool calendar_end_enabled = true;
  int calendar_end_year = 2;
  float calendar_end_day_of_year = 0.0f;
  bool stop_auto_grow_at_calendar_end = true;

  int tassel_color_mode = 0;
  bool scene_plant_view_tint_enabled = true;
  bool seasonal_sky_tint_enabled = false;
  bool seasonal_plant_tint_enabled = false;
  float seasonal_plant_tint_strength = 0.25f;
  std::array<glm::vec4, 4> seasonal_tint_colors = {
      glm::vec4(0.42f, 0.74f, 0.44f, 1.0f),
      glm::vec4(0.90f, 0.82f, 0.30f, 1.0f),
      glm::vec4(0.91f, 0.49f, 0.20f, 1.0f),
      glm::vec4(0.35f, 0.58f, 0.86f, 1.0f),
  };

  bool profiling_enabled = false;
  int profiling_history_size = 240;
  std::string profiling_export_path = "lsystem_profile.csv";

  int synthetic_profile_render_mode = 0;
  int synthetic_profile_ray_trace_samples = 4;
  int synthetic_profile_ray_trace_bounces = 4;
  std::string synthetic_profile_output_root = "output/scots_pine_synthetic/editor_profiles";
  std::string synthetic_profile_last_status = "Not run.";
  std::string synthetic_profile_last_output{};
  double synthetic_profile_last_total_ms = 0.0;
  double synthetic_profile_samples_per_hour = 0.0;

  std::array<char, 512> full_fidelity_output_root{};
  std::array<char, 128> full_fidelity_output_name{};
  std::string full_fidelity_last_status = "Not run.";
  std::string full_fidelity_last_output{};
  std::string full_fidelity_last_rgb_path{};
  std::string full_fidelity_last_config_path{};

  ProfileFrame last_profile_frame{};
  std::vector<ProfileFrame> profiling_history{};

  void OnCreate() override;
  void OnDestroy() override;
  void Update() override;
  bool SupportsProjectStateSerialization() const override;
  void SerializeProjectState(YAML::Emitter& out) const override;
  void DeserializeProjectState(const YAML::Node& in) override;

 private:
  static constexpr float kAutoGrowFailsafeMinFps = 1.0f;
  bool fps_failsafe_tripped_ = false;
  float last_failsafe_fps_ = 0.0f;
  bool synthetic_profile_requested_ = false;
  bool full_fidelity_export_requested_ = false;
  std::weak_ptr<evo_engine::Scene> auto_regenerated_scene_;
  bool seasonal_scene_camera_override_active_ = false;
  bool seasonal_scene_camera_prev_use_clear_color_ = false;
  glm::vec4 seasonal_scene_camera_prev_clear_color_ = glm::vec4(59.0f / 255.0f, 85.0f / 255.0f, 143.0f / 255.0f,
                                                                 1.0f);
  struct FullFidelityExportResult {
    int exit_code = -1;
    double elapsed_ms = 0.0;
    std::filesystem::path output_root{};
    std::filesystem::path rgb_path{};
    std::filesystem::path config_path{};
    std::string error{};
  };
  std::future<FullFidelityExportResult> full_fidelity_export_future_{};
  std::shared_ptr<evo_engine::Texture2D> full_fidelity_preview_texture_{};

  void PushProfileFrame(const ProfileFrame& frame);
  void ExportProfileCsv(const std::string& path) const;
  void EnsureLoadedSceneScotsPinesGenerated();
  void RunSyntheticRenderProfile();
  void InitializeFullFidelitySyntheticExportDefaults();
  void DrawFullFidelitySyntheticExportUi();
  void RequestFullFidelitySyntheticExport();
  void PollFullFidelitySyntheticExport();
  void LoadFullFidelityPreviewTexture(const std::filesystem::path& path);
  void ApplySeasonalSceneTint(const std::shared_ptr<evo_engine::EditorLayer>& editor_layer);
  void RestoreSeasonalSceneTint(const std::shared_ptr<evo_engine::EditorLayer>& editor_layer);
};

}  // namespace l_system_package
