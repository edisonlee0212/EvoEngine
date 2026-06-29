#pragma once
#include "EvoEngine_SDK_PCH.hpp"

#include "Application.hpp"
#include "LightProbeGroup.hpp"

#include "CUDAModule.hpp"
#include "IPrivateComponent.hpp"

namespace evo_engine {
class IlluminationLightmapEstimator : public IPrivateComponent {
  LightProbeGroup light_probe_group_;
  std::vector<glm::vec3> vertex_lightmap_;

 public:
  void PrepareLightProbeGroup();
  void SampleLightProbeGroup(const RayProperties& ray_properties, int seed, float push_normal_distance);
  void ApplyLightmapToVertices(float exposure = 1.0f, bool tone_mapping = true) const;
  void ExportLightmappedObj(const std::filesystem::path& path, float exposure = 1.0f, bool tone_mapping = true) const;
  void ExportRawLightmapCsv(const std::filesystem::path& path) const;
  bool DrawGui(const std::shared_ptr<EditorLayer>& editor_layer);
  const LightProbeGroup& PeekProbes() const;
  const std::vector<glm::vec3>& PeekLightmap() const;
  void Serialize(YAML::Emitter& out) const;
  void Deserialize(const YAML::Node& in);

  LightProbeGroup GetLightProbeGroup() const {
    return light_probe_group_;
  }
};

}  // namespace evo_engine
