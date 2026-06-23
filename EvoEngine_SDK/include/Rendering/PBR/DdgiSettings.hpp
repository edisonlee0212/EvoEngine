#pragma once

#include <glm/glm.hpp>

namespace evo_engine {
enum class DdgiVolumeMovementType : int { Default = 0, Scrolling = 1 };

enum DdgiVolumeTriggerCondition : int {
  DdgiVolumeTriggerConditionNone = 0,
  DdgiVolumeTriggerConditionLightEnableChanged = 1 << 0,
  DdgiVolumeTriggerConditionLightingConditionChanged = 1 << 1,
  DdgiVolumeTriggerConditionGeometryChanged = 1 << 2,
  DdgiVolumeTriggerConditionAll = DdgiVolumeTriggerConditionLightEnableChanged |
                                  DdgiVolumeTriggerConditionLightingConditionChanged |
                                  DdgiVolumeTriggerConditionGeometryChanged
};

struct DdgiSettings {
  struct RuntimeSettings {
    bool enabled = false;
    bool pause_updates = false;
    bool reset_probe_history = false;
    int ray_count = 256;
    int warmup_frames = 16;
    float hysteresis = 0.97f;
    float normal_bias = 0.1f;
    float view_bias = 0.1f;
    float max_ray_distance = 1e27f;
    float distance_exponent = 50.0f;
    float irradiance_gamma = 5.0f;
    float visibility_moment_bias = 0.02f;
    float indirect_intensity = 1.0f;
    float irradiance_threshold = 0.25f;
    float brightness_threshold = 0.10f;
  };

  struct VolumeDefaults {
    glm::ivec3 probe_counts = {10, 6, 16};
    glm::vec3 probe_spacing = glm::vec3(1.5f);
    glm::vec3 volume_origin = {0.0f, 3.0f, 3.0f};
    int movement_type = static_cast<int>(DdgiVolumeMovementType::Default);
    bool enable_probe_relocation = true;
    bool enable_probe_classification = false;
    bool enable_probe_variability = true;
    bool enable_probe_variability_gating = true;
    float relocation_distance = 0.25f;
    float random_ray_backface_threshold = 0.1f;
    float fixed_ray_backface_threshold = 0.25f;
    float probe_variability_threshold = 0.2f;
    int probe_variability_min_samples = 16;
  };

  struct StorageSettings {
    int max_probe_count = 8192;
    int irradiance_tile_resolution = 8;
    int visibility_tile_resolution = 16;
    int atlas_probe_columns = 16;
  };

  struct DebugSettings {
    bool enabled = false;
    bool visualize_volume_bounds = true;
    bool visualize_probe_positions = true;
    bool visualize_selected_probe = false;
    bool visualize_probe_state = false;
    bool visualize_probe_illumination = true;
    bool show_atlas_preview = false;
    bool show_update_age = false;
    bool show_rays = false;
    bool show_irradiance = false;
    bool show_visibility = false;
    bool show_sampling_weights = false;
    int selected_probe_index = 0;
    int atlas_layer = 0;
    float visualization_scale = 2.0f;
    int probe_visualization_mode = 0;
    int probe_visualization_depth_mode = 0;
    float probe_visualization_radius = 0.08f;
    float probe_visualization_intensity = 1.0f;
    float probe_visualization_alpha = 0.95f;
    float selected_probe_visualization_scale = 2.5f;
  };

  RuntimeSettings runtime{};
  VolumeDefaults volume_defaults{};
  StorageSettings storage{};
  DebugSettings debug{};
};
}  // namespace evo_engine
