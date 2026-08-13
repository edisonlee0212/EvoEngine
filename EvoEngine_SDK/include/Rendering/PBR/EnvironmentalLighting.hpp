#pragma once

#include "AssetRef.hpp"
#include "CameraSettings.hpp"
#include "DdgiSettings.hpp"
#include "IAsset.hpp"

#include <cstdint>
#include <glm/glm.hpp>
#include <string>
#include <vector>

namespace evo_engine {

class EnvironmentalLighting final : public IAsset {
 public:
  enum class IndirectEnvironmentSourceKind : uint32_t {
    EngineDefault = 0,
    Color = 1,
    EnvironmentalMap = 2,
  };

  enum class LocalReflectionProbeShape : int {
    Box = 0,
    Sphere = 1,
  };

  static constexpr uint32_t kMaxLocalReflectionProbeCount = 32u;
  static constexpr uint32_t kMaxDdgiVolumeCount = 8u;
  static constexpr int kMaxExactLocalReflectionProbePriority = 1 << 24;
  static constexpr float kDefaultEnvironmentLightingIntensity = 1.0f;
  static constexpr float kDefaultDiffuseFallbackIntensity = 0.0f;
  static constexpr float kDefaultSpecularFallbackIntensity = 1.0f;
  static constexpr float kSpecularVisibilityGrazingOcclusionCap = 0.04f;
  static constexpr float kSpecularVisibilityFullTrustStart = 0.8f;

  struct IndirectEnvironmentSource {
    IndirectEnvironmentSourceKind kind = IndirectEnvironmentSourceKind::EngineDefault;
    AssetRef environmental_map;
    glm::vec3 color = glm::vec3(0.0f);
    float gamma = 2.2f;
    float rotation = 0.0f;

    void CollectAssetRef(std::vector<AssetRef>& list);
  };

  struct ReflectionProbeBakeBackground {
    CameraSettings::BackgroundSource source = CameraSettings::BackgroundSource::InheritEnvironmentalLighting;
    AssetRef cubemap;
    AssetRef environmental_map;
    glm::vec4 clear_color = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);

    void CollectAssetRef(std::vector<AssetRef>& list);
  };

  struct DynamicReflectionProbeSettings {
    int faces_per_frame = 6;
    bool enabled = true;

    void Clamp();
  };

  struct LocalReflectionProbe {
    std::string name = "Local Reflection Probe";
    uint64_t stable_id = 0;
    AssetRef global_reflection_probe;
    glm::mat4 transform = glm::mat4(1.0f);
    glm::vec3 box_projection_extents = glm::vec3(0.5f);
    float sphere_radius = 5.0f;
    float blend_distance = 0.05f;
    float reflection_intensity = 1.0f;
    int artist_priority = 0;
    int shape = static_cast<int>(LocalReflectionProbeShape::Box);
    bool box_projection = true;
    bool enabled = true;
    bool debug_draw_bounds = false;

    void CollectAssetRef(std::vector<AssetRef>& list);
  };

  struct DdgiVolume {
    std::string name = "DDGI Volume";
    uint64_t stable_id = 0;
    glm::mat4 transform = glm::mat4(1.0f);
    glm::ivec3 probe_counts = glm::ivec3(10, 6, 16);
    glm::vec3 probe_spacing = glm::vec3(1.5f);
    glm::vec3 volume_origin = glm::vec3(0.0f, 3.0f, 3.0f);
    int artist_priority = 0;
    int movement_type = static_cast<int>(DdgiVolumeMovementType::Default);
    int emissive_mesh_sampling_mode = static_cast<int>(DdgiEmissiveMeshSamplingMode::Inherit);
    bool enabled = true;
    bool enable_probe_relocation = true;
    bool enable_probe_classification = false;
    bool enable_probe_variability = true;
    bool enable_probe_variability_gating = true;
    float relocation_distance = 0.25f;
    float random_ray_backface_threshold = 0.1f;
    float fixed_ray_backface_threshold = 0.25f;
    float probe_variability_threshold = 0.03f;
    int probe_variability_min_samples = 128;
    int hysteresis_boost_trigger_conditions = DdgiVolumeTriggerConditionAll;
    int variability_reset_trigger_conditions =
        DdgiVolumeTriggerConditionLightingConditionChanged | DdgiVolumeTriggerConditionGeometryChanged;

    void ClampSettings();
    [[nodiscard]] uint32_t GetProbeAmount() const;
    [[nodiscard]] glm::vec3 GetLocalGridSize() const;
    [[nodiscard]] glm::vec3 GetProbeLocalPosition(const glm::ivec3& probe_index) const;
  };

  IndirectEnvironmentSource indirect_environment_source{};
  ReflectionProbeBakeBackground reflection_probe_bake_background{};
  DynamicReflectionProbeSettings dynamic_reflection_probe_settings{};
  float environment_lighting_intensity = kDefaultEnvironmentLightingIntensity;
  float diffuse_fallback_intensity = kDefaultDiffuseFallbackIntensity;
  float specular_fallback_intensity = kDefaultSpecularFallbackIntensity;
  DdgiSettings ddgi_settings{};
  bool local_reflection_probes_enabled = true;
  std::vector<LocalReflectionProbe> local_reflection_probes;
  std::vector<DdgiVolume> ddgi_volumes;

  bool RepairStableIds();
  void CollectAssetRef(std::vector<AssetRef>& list);

  [[nodiscard]] static float EvaluateRoughSpecularVisibility(float material_occlusion, float screen_space_visibility,
                                                             float ddgi_visibility, float roughness,
                                                             float normal_dot_view);
};

void SerializeEnvironmentalLighting(YAML::Emitter& out, const EnvironmentalLighting& lighting);
void DeserializeEnvironmentalLighting(const YAML::Node& in, EnvironmentalLighting& lighting);

}  // namespace evo_engine
