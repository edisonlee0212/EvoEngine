#pragma once

#include "PointCloudScannerUtils.hpp"

#include <cstdint>
#include <deque>
#include <string>

namespace dataset_generation_plugin {
using namespace evo_engine;

struct TasselPointCloudPointSettings {
  // Distance-scaled Gaussian hit jitter and isotropic hit-ball jitter radius.
  float distance_sigma_scale = 0.015f;
  float distance_sigma_scale_deviation = 0.0f;
  float hit_ball_jitter_radius = 0.01f;
  float hit_ball_jitter_radius_deviation = 0.0f;

  // Range-dependent Gaussian noise: sigma = range_noise_base_sigma + range_noise_scale * distance.
  float range_noise_base_sigma = 0.0f;
  float range_noise_base_sigma_deviation = 0.0f;
  float range_noise_scale = 0.0f;
  float range_noise_scale_deviation = 0.0f;

  // Per-ray probability of producing no return (0 = perfect detection, 1 = all dropped).
  float dropout_probability = 0.0f;
  float dropout_probability_deviation = 0.0f;

  // Angular jitter applied to each ray direction before tracing (radians, Gaussian sigma).
  float angular_noise_sigma = 0.0f;
  float angular_noise_sigma_deviation = 0.0f;

  // Detection range limits.  Hits outside [min_range, max_range] are discarded.
  float min_range = 0.0f;
  float min_range_deviation = 0.0f;
  float max_range = 0.0f;  // 0 = unlimited
  float max_range_deviation = 0.0f;

  // When true, per-point RGB color (from surface hit) is written to the PLY.
  bool color_output = false;

  bool type_index = true;       // 0 = stem, 1 = spikelet
  bool instance_index = true;

  float bounding_box_limit = 1.f;
  float bounding_box_limit_deviation = 0.0f;

  bool OnInspect();
  void Save(const std::string& name, YAML::Emitter& out) const;
  void Load(const std::string& name, const YAML::Node& in);
};

enum class TasselScanMode { Hemisphere, Gantry, Circular };

class TasselPointCloudGridCaptureSettings : public PointCloudCaptureSettings {
 public:
  float bounding_box_size = 1.f;
  float bounding_box_size_deviation = 0.0f;

  // Shared grid parameters.
  glm::ivec2 grid_size = {1, 1};
  glm::ivec2 grid_size_deviation = {0, 0};
  float grid_distance = 0.5f;
  float grid_distance_deviation = 0.0f;
  float step = 0.005f;
  float step_deviation = 0.0f;
  int samples_per_step = 256;
  int samples_per_step_deviation = 0;
  float sample_height = 1.0f;
  float sample_height_deviation = 0.0f;

  // World-space scanner origin offset (mainly X/Z recentering onto plant bounds).
  glm::vec3 scan_center = glm::vec3(0.0f);
  glm::vec3 scan_center_deviation = glm::vec3(0.0f);

  // World-space Y target used by circular mode look-at. 0 keeps legacy behavior.
  float look_target_height = 0.0f;
  float look_target_height_deviation = 0.0f;

  // Scan pattern selection.
  TasselScanMode scan_mode = TasselScanMode::Hemisphere;

  // --- Gantry mode parameters ---
  std::vector<float> scanner_angles = {30.f};  // Degrees from vertical.
  std::vector<float> scanner_angle_deviations = {0.0f};

  // --- Circular mode parameters ---
  float scanner_distance = 0.5f;   // Radius from object center.
  float scanner_distance_deviation = 0.0f;
  int pitch_angle_start = -30;     // Elevation sweep start (degrees).
  int pitch_angle_start_deviation = 0;
  int pitch_angle_end = 60;        // Elevation sweep end.
  int pitch_angle_end_deviation = 0;
  int pitch_angle_step = 10;
  int pitch_angle_step_deviation = 0;
  int turn_angle_start = 0;        // Azimuth sweep start.
  int turn_angle_start_deviation = 0;
  int turn_angle_end = 360;
  int turn_angle_end_deviation = 0;
  int turn_angle_step = 10;
  int turn_angle_step_deviation = 0;
  float fov = 60.f;                // Camera field-of-view (degrees).
  float fov_deviation = 0.0f;
  int scan_resolution = 128;       // Rays per image edge.
  int scan_resolution_deviation = 0;

  // --- Circular mode point-budget controls ---
  bool point_budget_enabled = false;
  int target_points = 0;
  int target_points_deviation = 0;
  float target_points_tolerance_ratio = 0.10f;
  int target_points_max_retry_passes = 1;
  bool downsample_to_budget_max = true;

  bool OnInspect() override;
  void Save(const std::string& name, YAML::Emitter& out) const override;
  void Load(const std::string& name, const YAML::Node& in) override;
  void GenerateSamples(std::vector<PointCloudSample>& point_cloud_samples) override;
  bool SampleFilter(const PointCloudSample& sample) override;

 private:
  void GenerateHemisphereSamples(std::vector<PointCloudSample>& point_cloud_samples) const;
  void GenerateGantrySamples(std::vector<PointCloudSample>& point_cloud_samples) const;
  void GenerateCircularSamples(std::vector<PointCloudSample>& point_cloud_samples) const;
};

class TasselPointCloudScannerDescriptor : public IAsset {
 public:
  TasselPointCloudScannerDescriptor();

  TasselPointCloudPointSettings point_settings{};
  TasselPointCloudGridCaptureSettings capture_settings{};

  // Deterministic seed used by all scanner-side stochastic streams.
  int scan_seed = 0;

  bool gpu_only_mode = false;  // [deprecated] Legacy flag kept for backward compatibility.

  bool visual_scan_interactive_enabled = false;
  bool visual_scan_show_scanner = true;
  bool visual_scan_show_beams = true;
  bool visual_scan_show_heat_points = true;
  bool visual_scan_depth_test_beams = true;
  bool visual_scan_depth_test_heat_points = true;
  bool visual_scan_interpolate_motion = true;
  bool visual_scan_keep_points_after_finish = true;  // [deprecated] Points now always persist until cleared.
  float visual_scan_playback_speed = 1.0f;
  float visual_scan_playback_speed_deviation = 0.0f;
  float visual_scan_view_dwell_seconds = 0.08f;
  float visual_scan_view_dwell_seconds_deviation = 0.0f;
  float visual_scan_transition_seconds = 0.03f;
  float visual_scan_transition_seconds_deviation = 0.0f;
  int visual_scan_beam_stride = 32;
  int visual_scan_beam_stride_deviation = 0;
  int visual_scan_max_beams_per_view = 2048;
  int visual_scan_max_beams_per_view_deviation = 0;
  float visual_scan_beam_width = 0.003f;
  float visual_scan_beam_width_deviation = 0.0f;
  float visual_scan_beam_alpha = 0.75f;
  float visual_scan_beam_alpha_deviation = 0.0f;
  float visual_scan_beam_speed = 6.0f;
  float visual_scan_beam_speed_deviation = 0.0f;
  float visual_scan_beam_fire_fraction = 0.35f;
  float visual_scan_beam_fire_fraction_deviation = 0.0f;
  float visual_scan_beam_max_length = 1.5f;
  float visual_scan_beam_max_length_deviation = 0.0f;
  int visual_scan_max_heat_points = 20000;
  int visual_scan_max_heat_points_deviation = 0;
  int visual_scan_max_hits_per_view = 1024;
  int visual_scan_max_hits_per_view_deviation = 0;
  float visual_scan_point_size = 0.012f;
  float visual_scan_point_size_deviation = 0.0f;
  float visual_scan_scanner_size = 0.03f;
  float visual_scan_scanner_size_deviation = 0.0f;
  float visual_scan_hot_seconds = 0.15f;
  float visual_scan_hot_seconds_deviation = 0.0f;
  float visual_scan_warm_seconds = 0.35f;
  float visual_scan_warm_seconds_deviation = 0.0f;
  float visual_scan_cool_seconds = 1.25f;
  float visual_scan_cool_seconds_deviation = 0.0f;

  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
};

class TasselPointCloudScanner : public IPrivateComponent {
 public:
  void SetScannerDescriptor(const std::shared_ptr<TasselPointCloudScannerDescriptor>& descriptor);
  bool TryGetActiveSettings(TasselPointCloudPointSettings& point_settings,
                            TasselPointCloudGridCaptureSettings& capture_settings) const;

  void Scan(const std::shared_ptr<PointCloudCaptureSettings>& capture_settings, std::vector<glm::vec3>& points,
            std::vector<int>& instance_indices, std::vector<int>& type_indices,
            std::vector<glm::vec3>& colors) const;

  void SavePointCloud(const std::filesystem::path& save_path, const std::vector<glm::vec3>& points,
                      const std::vector<int>& instance_indices, const std::vector<int>& type_indices,
                      const std::vector<glm::vec3>& colors) const;

  void Capture(const std::filesystem::path& save_path,
               const std::shared_ptr<PointCloudCaptureSettings>& capture_settings) const;

  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void Update() override;
  void OnDestroy() override;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
  void CollectAssetRef(std::vector<AssetRef>& list) override;

 private:
  struct ScanExecutionResult {
    struct CountSummary {
      size_t generated_samples = 0;
      size_t ray_hit_samples = 0;
      size_t sample_filter_rejected = 0;
      size_t range_filter_rejected = 0;
      size_t dropout_rejected = 0;
      size_t bound_rejected = 0;
      size_t kept_before_budget = 0;
      size_t kept_after_budget = 0;
      int scan_resolution_used = 0;
      int circular_view_count = 0;
    };

    struct BudgetSummary {
      bool enabled = false;
      int target_points = 0;
      int min_points = 0;
      int max_points = 0;
      float tolerance_ratio = 0.0f;
      int max_retry_passes = 0;
      int retry_passes_used = 0;
      bool downsample_to_budget_max = false;
      bool downsample_applied = false;
      std::string status = "disabled";
    };

    std::vector<PointCloudSample> samples;
    std::vector<uint8_t> sample_kept;
    std::vector<glm::vec3> sample_points;
    std::vector<glm::vec3> points;
    std::vector<int> instance_indices;
    std::vector<int> type_indices;
    std::vector<glm::vec3> colors;

    CountSummary counts{};
    BudgetSummary budget{};
    std::string scan_mode_name = "Unknown";
    int scan_seed = 0;
    TasselPointCloudPointSettings effective_point_settings{};
    bool has_effective_capture_settings = false;
    TasselPointCloudGridCaptureSettings effective_capture_settings{};
  };

  bool ExecuteDeterministicScan(const std::shared_ptr<TasselPointCloudScannerDescriptor>& active_descriptor,
                                const std::shared_ptr<PointCloudCaptureSettings>& capture_settings,
                                ScanExecutionResult& out) const;
  void RunScanVisualization(const std::shared_ptr<TasselPointCloudScannerDescriptor>& active_descriptor);

  struct VisualScanViewBatch {
    glm::vec3 scanner_origin = glm::vec3(0.0f);
    std::vector<glm::vec3> beam_starts;
    std::vector<glm::vec3> beam_ends;
    std::vector<glm::vec3> accepted_hits;
  };

  struct VisualHeatPoint {
    glm::vec3 position = glm::vec3(0.0f);
    float age_seconds = 0.0f;
  };

  enum class VisualPlaybackState { Idle, Playing, Finished };
  enum class PrivateScanMode { OneShot, Animated };

  bool BuildVisualScanBatches(const std::shared_ptr<TasselPointCloudScannerDescriptor>& active_descriptor);
  void StartVisualScanPlayback(const std::shared_ptr<TasselPointCloudScannerDescriptor>& active_descriptor);
  void StopVisualScanPlayback(bool clear_heat_points);
  void TickVisualScanPlayback(const std::shared_ptr<TasselPointCloudScannerDescriptor>& active_descriptor,
                              double delta_seconds);
  void RenderVisualScanPlayback(const std::shared_ptr<EditorLayer>& editor_layer,
                                const std::shared_ptr<TasselPointCloudScannerDescriptor>& active_descriptor);
  void ClearVisualScanRuntimeState(bool clear_batches, bool clear_heat_points);

  AssetRef scanner_descriptor_ref_;

  PrivateScanMode private_scan_mode_ = PrivateScanMode::OneShot;
  float one_shot_repeat_hz_ = 0.0f;
  float one_shot_repeat_hz_deviation_ = 0.0f;
  bool advance_seed_per_playback_ = false;
  double one_shot_last_trigger_seconds_ = -1.0;

  AssetRef visual_scan_beam_particle_info_ref_;
  AssetRef visual_scan_heat_particle_info_ref_;

  VisualPlaybackState visual_scan_playback_state_ = VisualPlaybackState::Idle;
  std::vector<VisualScanViewBatch> visual_scan_batches_;
  std::deque<VisualHeatPoint> visual_scan_heat_points_;

  size_t visual_scan_active_view_index_ = 0;
  double visual_scan_view_elapsed_seconds_ = 0.0;
  double visual_scan_last_tick_seconds_ = -1.0;
  double visual_scan_last_build_seconds_ = 0.0;
  bool visual_scan_current_view_injected_ = false;

  glm::vec3 visual_scan_scanner_position_ = glm::vec3(0.0f);
  size_t visual_scan_last_rendered_beams_ = 0;
  size_t visual_scan_last_rendered_heat_points_ = 0;
  size_t visual_scan_total_injected_hits_ = 0;

  std::vector<size_t> visual_scan_active_beam_indices_;
  std::vector<float> visual_scan_active_beam_fire_times_;
};
}  // namespace dataset_generation_plugin
