#pragma once
#include "EvoEngine_SDK_PCH.hpp"

#include "Application.hpp"
#include "LightProbeGroup.hpp"

#include "CUDAModule.hpp"
#include "IPrivateComponent.hpp"
#include "SurfaceIlluminationResult.hpp"

#include <map>

namespace evo_engine {
class TriangleIlluminationEstimator : public IPrivateComponent {
  LightProbeGroup light_probe_group_;
  std::vector<std::string> probe_entity_names_;
  std::map<std::string, SurfaceIlluminationResult> mesh_illumination_;

 public:
  void PrepareLightProbeGroup(const std::unordered_set<std::string>& double_sided_entity_names = {});
  void SampleLightProbeGroup(const RayProperties& ray_properties, int seed, float push_normal_distance);
  float total_area = 0.0f;
  glm::vec3 total_flux = glm::vec3(0.0f);
  glm::vec3 average_flux = glm::vec3(0.0f);
  bool DrawGui(const std::shared_ptr<EditorLayer>& editor_layer);
  const LightProbeGroup& PeekProbes() const;
  const std::map<std::string, SurfaceIlluminationResult>& PeekMeshIllumination() const;
};

}  // namespace evo_engine
