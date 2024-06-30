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
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;

  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
};

}  // namespace evo_engine
