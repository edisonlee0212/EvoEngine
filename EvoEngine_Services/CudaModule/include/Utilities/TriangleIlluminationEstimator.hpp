#pragma once
#include "EvoEngine_SDK_PCH.hpp"

#include "Application.hpp"
#include "LightProbeGroup.hpp"

#include "CUDAModule.hpp"
#include "IPrivateComponent.hpp"

namespace evo_engine {
class TriangleIlluminationEstimator : public IPrivateComponent {
  LightProbeGroup light_probe_group_;

 public:
  void PrepareLightProbeGroup();
  void SampleLightProbeGroup(const RayProperties& ray_properties, int seed, float push_normal_distance);
  float total_area = 0.0f;
  glm::vec3 total_flux = glm::vec3(0.0f);
  glm::vec3 average_flux = glm::vec3(0.0f);
  bool DrawGui(const std::shared_ptr<EditorLayer>& editor_layer);
  const LightProbeGroup& PeekProbes() const;
  void Serialize(YAML::Emitter& out) const;
  void Deserialize(const YAML::Node& in);

  LightProbeGroup GetLightProbeGroup() const{
    return light_probe_group_;
  }
};

}  // namespace evo_engine
