#pragma once

#include <cstdint>
#include <glm/glm.hpp>

namespace YAML {
class Emitter;
class EVOENGINE_API Node;
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

struct EVOENGINE_API DdgiSettings {
  struct RuntimeSettings {
    bool enabled = false;
    bool enable_emissive_mesh_sampling = true;
    int ray_count = 192;
    int emissive_ray_count = 64;
    int warmup_frames = 16;
    int history_count = 30;
    float normal_bias = 0.1f;
    float view_bias = 0.1f;
    float max_ray_distance = 1e27f;
    float distance_exponent = 50.0f;
    float irradiance_gamma = 5.0f;
    float visibility_moment_bias = 0.02f;
    float visibility_smoothing = 0.90f;
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
    float relocation_distance = 0.25f;
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

EVOENGINE_API void SerializeDdgiSettings(YAML::Emitter& out, const DdgiSettings& settings);
EVOENGINE_API void DeserializeDdgiSettings(const YAML::Node& in, DdgiSettings& settings);
}  // namespace evo_engine
