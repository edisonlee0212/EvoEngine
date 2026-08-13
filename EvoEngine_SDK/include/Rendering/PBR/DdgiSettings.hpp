#pragma once

#include <cstdint>
#include <glm/glm.hpp>

namespace YAML {
class Emitter;
class Node;
}  // namespace YAML

namespace evo_engine {
enum class DdgiVolumeMovementType : int { Default = 0, Scrolling = 1 };
enum class DdgiEmissiveMeshSamplingMode : int { Inherit = 0, On = 1, Off = 2 };

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
    bool enable_emissive_mesh_sampling = true;
    int ray_count = 256;
    int guided_ray_count = 0;
    int guided_emitter_count = 4;
    int warmup_frames = 16;
    float normal_bias = 0.1f;
    float view_bias = 0.1f;
    float max_ray_distance = 1e27f;
    float distance_exponent = 50.0f;
    float irradiance_gamma = 5.0f;
    float visibility_moment_bias = 0.02f;
    float irradiance_threshold = 0.25f;
    float brightness_threshold = 0.10f;
    bool deterministic_ray_seed_enabled = false;
    uint32_t deterministic_ray_seed = 0;
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
    float probe_variability_threshold = 0.03f;
    int probe_variability_min_samples = 128;
  };

  struct StorageSettings {
    int max_probe_count = 8192;
    int irradiance_tile_resolution = 8;
    int visibility_tile_resolution = 16;
    int atlas_probe_columns = 16;
  };

  RuntimeSettings runtime{};
  VolumeDefaults volume_defaults{};
  StorageSettings storage{};

  void ClampSettings();
};

void SerializeDdgiSettings(YAML::Emitter& out, const DdgiSettings& settings);
void DeserializeDdgiSettings(const YAML::Node& in, DdgiSettings& settings);
}  // namespace evo_engine
