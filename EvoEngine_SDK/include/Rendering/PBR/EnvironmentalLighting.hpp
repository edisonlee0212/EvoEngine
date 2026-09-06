#pragma once

#include "AssetRef.hpp"
#include "CameraSettings.hpp"
#include "DdgiSettings.hpp"
#include "DdgiVolumePack.hpp"
#include "IAsset.hpp"
#include "ReflectionProbePack.hpp"
#include "SdfgiSettings.hpp"

#include <cstdint>
#include <glm/glm.hpp>
#include <string>
#include <vector>

namespace evo_engine {

class EVOENGINE_API EnvironmentalLighting final : public IAsset {
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
  static constexpr float kDefaultDiffuseFallbackIntensity = 1.0f;
  static constexpr float kDefaultSpecularFallbackIntensity = 1.0f;
  static constexpr float kSpecularVisibilityGrazingOcclusionCap = 0.04f;
  static constexpr float kSpecularVisibilityFullTrustStart = 0.8f;

  struct EVOENGINE_API IndirectEnvironmentSource {
    IndirectEnvironmentSourceKind kind = IndirectEnvironmentSourceKind::EngineDefault;
    AssetRef environmental_map;
    glm::vec3 color = glm::vec3(0.0f);
    float gamma = 2.2f;
    float rotation = 0.0f;

    void CollectAssetRef(std::vector<AssetRef>& list);
  };

  struct EVOENGINE_API ReflectionProbeBakeBackground {
    CameraSettings::BackgroundSource source = CameraSettings::BackgroundSource::InheritEnvironmentalLighting;
    AssetRef cubemap;
    AssetRef environmental_map;
    glm::vec4 clear_color = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);

    void CollectAssetRef(std::vector<AssetRef>& list);
  };

  struct EVOENGINE_API DynamicReflectionProbeSettings {
    int faces_per_frame = 6;
    bool enabled = true;

    void Clamp();
  };

  using LocalReflectionProbe = ReflectionProbePack::Probe;
  using DdgiVolume = DdgiVolumePack::Volume;

  IndirectEnvironmentSource indirect_environment_source{};
  ReflectionProbeBakeBackground reflection_probe_bake_background{};
  DynamicReflectionProbeSettings dynamic_reflection_probe_settings{};
  float environment_lighting_intensity = kDefaultEnvironmentLightingIntensity;
  float diffuse_fallback_intensity = kDefaultDiffuseFallbackIntensity;
  float specular_fallback_intensity = kDefaultSpecularFallbackIntensity;
  DdgiSettings ddgi_settings{};
  IndirectGiProvider indirect_gi_provider = IndirectGiProvider::AutomaticSdfgi;
  SdfgiSettings sdfgi_settings{};
  bool local_reflection_probes_enabled = true;
  AssetRef reflection_probe_pack;
  AssetRef ddgi_volume_pack;

  [[nodiscard]] std::shared_ptr<ReflectionProbePack> GetReflectionProbePack() const;
  [[nodiscard]] std::shared_ptr<DdgiVolumePack> GetDdgiVolumePack() const;
  [[nodiscard]] std::shared_ptr<ReflectionProbePack> GetOrCreateReflectionProbePack();
  [[nodiscard]] std::shared_ptr<DdgiVolumePack> GetOrCreateDdgiVolumePack();
  void CollectAssetRef(std::vector<AssetRef>& list);

  [[nodiscard]] static float EvaluateRoughSpecularVisibility(float material_occlusion, float screen_space_visibility,
                                                             float ddgi_visibility, float roughness,
                                                             float normal_dot_view);
};

EVOENGINE_API void SerializeEnvironmentalLighting(YAML::Emitter& out, const EnvironmentalLighting& lighting);
EVOENGINE_API void DeserializeEnvironmentalLighting(const YAML::Node& in, EnvironmentalLighting& lighting);

}  // namespace evo_engine
