#pragma once
#ifdef BUILD_WITH_RAYTRACER

#  include <CUDAModule.hpp>
using namespace evo_engine;
namespace eco_sys_lab {
class PARSensorGroup : public IAsset {
 public:
  std::vector<IlluminationSampler<glm::vec3>> m_samplers;
  void CalculateIllumination(const RayProperties& ray_properties, int seed, float push_normal_distance);
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
};
}  // namespace eco_sys_lab
#endif