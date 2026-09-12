#pragma once

#include <cstdint>
#include <glm/glm.hpp>

namespace YAML {
class Emitter;
class EVOENGINE_API Node;
}  // namespace YAML

namespace evo_engine {

enum DdgiSceneChange : int {
  DdgiSceneChangeNone = 0,
  DdgiSceneChangeLightEnableChanged = 1 << 0,
  DdgiSceneChangeLightingConditionChanged = 1 << 1,
  DdgiSceneChangeGeometryChanged = 1 << 2,
  DdgiSceneChangeAll =
      DdgiSceneChangeLightEnableChanged | DdgiSceneChangeLightingConditionChanged | DdgiSceneChangeGeometryChanged
};

struct EVOENGINE_API DdgiSettings {
  struct RuntimeSettings {
    bool enabled = false;
    bool enable_emissive_mesh_sampling = true;
    int ray_count = 64;
    int emissive_ray_count = 8;
    int warmup_frames = 16;
    int history_count = 30;
    float normal_bias = 0.1f;
    float view_bias = 0.1f;
    float max_ray_distance = 1e27f;
    float distance_exponent = 50.0f;
    float irradiance_gamma = 5.0f;
    float visibility_moment_bias = 0.02f;
    bool enable_probe_relocation = true;
    bool use_voxel_occlusion = false;
    bool enable_probe_classification = true;
    float relocation_distance = 0.25f;
    float random_ray_backface_threshold = 0.1f;
    float fixed_ray_backface_threshold = 0.25f;
    bool deterministic_ray_seed_enabled = false;
    uint32_t deterministic_ray_seed = 0;
  };

  RuntimeSettings runtime{};

  void ClampSettings();
  [[nodiscard]] bool operator==(const DdgiSettings& other) const;
};

EVOENGINE_API void SerializeDdgiSettings(YAML::Emitter& out, const DdgiSettings& settings);
EVOENGINE_API void DeserializeDdgiSettings(const YAML::Node& in, DdgiSettings& settings);
}  // namespace evo_engine
