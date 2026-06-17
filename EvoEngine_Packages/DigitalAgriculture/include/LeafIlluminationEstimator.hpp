#pragma once
#include "EvoEngine_SDK_PCH.hpp"

#ifdef CUDA_MODULE_SERVICE
#  include "Application.hpp"
#  include "CUDAModule.hpp"
#  include "IPrivateComponent.hpp"
#  include "LightProbeGroup.hpp"

namespace digital_agriculture_package {
class LeafIlluminationEstimator : public evo_engine::IPrivateComponent {
 public:
  struct LeafIlluminationInfo {
    evo_engine::Entity entity;
    std::string name;
    size_t probe_offset = 0;
    size_t probe_count = 0;
    float area = 0.0f;
    glm::vec3 center = glm::vec3(0.0f);
    glm::vec3 total_illumination = glm::vec3(0.0f);
    glm::vec3 average_illumination = glm::vec3(0.0f);
  };

 private:
  evo_engine::LightProbeGroup light_probe_group_;
  std::vector<LeafIlluminationInfo> leaf_illumination_infos_;

 public:
  void PrepareLightProbeGroup();
  void SampleLightProbeGroup(const evo_engine::RayProperties& ray_properties, int seed, float push_normal_distance);
  bool DrawGui(const std::shared_ptr<evo_engine::EditorLayer>& editor_layer);
  const evo_engine::LightProbeGroup& PeekProbes() const;
  const std::vector<LeafIlluminationInfo>& PeekLeafIlluminationInfos() const;

  evo_engine::LightProbeGroup GetLightProbeGroup() const {
    return light_probe_group_;
  }
};
}  // namespace digital_agriculture_package
#endif
