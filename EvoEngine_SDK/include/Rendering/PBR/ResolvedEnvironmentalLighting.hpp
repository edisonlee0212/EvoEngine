#pragma once

#include "AssetRef.hpp"
#include "DdgiSettings.hpp"

#include <cstdint>
#include <glm/glm.hpp>
#include <vector>

namespace evo_engine {

/**
 * @brief Runtime view for resolved environmental-lighting ownership.
 *
 * Defines the shape and fallback semantics consumed by renderer lighting paths.
 */
struct ResolvedEnvironmentalLighting {
  enum class IndirectEnvironmentSourceKind : uint32_t {
    EngineDefault = 0,
    Color = 1,
    EnvironmentalMap = 2,
  };

  enum class LightingUsage : uint32_t {
    DdgiMissRadiance = 0,
    DiffuseIblFallback = 1,
    GlobalSpecularFallback = 2,
    RayCameraEnvironmentEvent = 3,
    ReflectionProbeBakeEnvironmentInput = 4,
    ValidDdgiSurfaceIrradiance = 5,
    ValidLocalReflectionProbeSample = 6,
  };

  static constexpr uint32_t kMaxLocalReflectionProbeCount = 32u;
  static constexpr uint32_t kMaxDdgiVolumeCount = 8u;
  static constexpr float kDefaultEnvironmentLightingIntensity = 1.0f;
  static constexpr float kDefaultDiffuseFallbackIntensity = 1.0f;
  static constexpr float kDefaultSpecularFallbackIntensity = 1.0f;

  struct IndirectEnvironmentSource {
    IndirectEnvironmentSourceKind kind = IndirectEnvironmentSourceKind::EngineDefault;
    AssetRef environmental_map;
    glm::vec3 color = glm::vec3(0.0f);
    float gamma = 2.2f;
    float rotation = 0.0f;
  };

  struct LocalReflectionProbe {
    AssetRef global_reflection_probe;
    glm::mat4 transform = glm::mat4(1.0f);
    glm::vec3 box_extents = glm::vec3(5.0f);
    glm::vec3 box_projection_extents = glm::vec3(5.0f);
    float sphere_radius = 5.0f;
    float blend_distance = 1.0f;
    float reflection_intensity = 1.0f;
    uint64_t stable_id = 0;
    int artist_priority = 0;
    int shape = 0;
    bool box_projection = true;
    bool enabled = true;
  };

  struct DdgiVolume {
    glm::mat4 transform = glm::mat4(1.0f);
    glm::ivec3 probe_counts = glm::ivec3(10, 6, 16);
    glm::vec3 probe_spacing = glm::vec3(1.5f);
    glm::vec3 volume_origin = glm::vec3(0.0f, 3.0f, 3.0f);
    uint64_t stable_id = 0;
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
    float probe_variability_threshold = 0.2f;
    int probe_variability_min_samples = 16;
    int auto_invalidate_trigger_conditions = DdgiVolumeTriggerConditionAll;
    int warmup_trigger_conditions = DdgiVolumeTriggerConditionLightEnableChanged;
    int variability_reset_trigger_conditions =
        DdgiVolumeTriggerConditionLightingConditionChanged | DdgiVolumeTriggerConditionGeometryChanged;
  };

  AssetRef scene_global_reflection_probe_fallback;
  IndirectEnvironmentSource indirect_environment_source{};
  float environment_lighting_intensity = kDefaultEnvironmentLightingIntensity;
  float diffuse_fallback_intensity = kDefaultDiffuseFallbackIntensity;
  float specular_fallback_intensity = kDefaultSpecularFallbackIntensity;
  DdgiSettings ddgi_settings{};
  std::vector<LocalReflectionProbe> local_reflection_probes;
  std::vector<DdgiVolume> ddgi_volumes;
  bool environmental_lighting_asset_assigned = false;
  bool environmental_lighting_asset_missing = false;
  bool uses_engine_default_indirect_environment_source = true;
  uint32_t truncated_local_reflection_probe_count = 0;
  uint32_t truncated_ddgi_volume_count = 0;

  [[nodiscard]] static constexpr bool LightingUsageUsesEnvironmentLightingIntensity(const LightingUsage usage) {
    switch (usage) {
      case LightingUsage::DdgiMissRadiance:
      case LightingUsage::DiffuseIblFallback:
      case LightingUsage::GlobalSpecularFallback:
      case LightingUsage::RayCameraEnvironmentEvent:
      case LightingUsage::ReflectionProbeBakeEnvironmentInput:
        return true;
      case LightingUsage::ValidDdgiSurfaceIrradiance:
      case LightingUsage::ValidLocalReflectionProbeSample:
        return false;
    }
    return false;
  }

  [[nodiscard]] static constexpr bool LightingUsageUsesDiffuseFallbackIntensity(const LightingUsage usage) {
    switch (usage) {
      case LightingUsage::DdgiMissRadiance:
      case LightingUsage::DiffuseIblFallback:
        return true;
      case LightingUsage::GlobalSpecularFallback:
      case LightingUsage::RayCameraEnvironmentEvent:
      case LightingUsage::ReflectionProbeBakeEnvironmentInput:
      case LightingUsage::ValidDdgiSurfaceIrradiance:
      case LightingUsage::ValidLocalReflectionProbeSample:
        return false;
    }
    return false;
  }

  [[nodiscard]] static constexpr bool LightingUsageUsesSpecularFallbackIntensity(const LightingUsage usage) {
    switch (usage) {
      case LightingUsage::GlobalSpecularFallback:
        return true;
      case LightingUsage::DdgiMissRadiance:
      case LightingUsage::DiffuseIblFallback:
      case LightingUsage::RayCameraEnvironmentEvent:
      case LightingUsage::ReflectionProbeBakeEnvironmentInput:
      case LightingUsage::ValidDdgiSurfaceIrradiance:
      case LightingUsage::ValidLocalReflectionProbeSample:
        return false;
    }
    return false;
  }

  [[nodiscard]] static constexpr bool LocalReflectionProbePayloadsUseEnvironmentLightingIntensity() {
    return false;
  }

  [[nodiscard]] static constexpr bool ValidDdgiSurfaceIrradianceUsesEnvironmentLightingIntensity() {
    return false;
  }

  [[nodiscard]] static constexpr bool RayCameraPrimaryMissUsesCameraBackground() {
    return true;
  }

  [[nodiscard]] static constexpr bool RayCameraEnvironmentLightingUsesIndirectEnvironmentSource() {
    return true;
  }

  [[nodiscard]] static constexpr bool RayCameraUsesGlobalReflectionProbeAsRadianceSource() {
    return false;
  }

  [[nodiscard]] static constexpr bool RayCameraUsesFallbackIntensities() {
    return false;
  }

  [[nodiscard]] static constexpr bool ReflectionProbeBakeUsesFallbackIntensities() {
    return false;
  }
};

}  // namespace evo_engine
