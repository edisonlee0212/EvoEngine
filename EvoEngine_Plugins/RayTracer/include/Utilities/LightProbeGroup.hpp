#pragma once
#include "EvoEngine_SDK_PCH.hpp"

#include <Application.hpp>
#include <CUDAModule.hpp>

namespace evo_engine {
struct LightProbeGroup {
  std::vector<IlluminationSampler<glm::vec3>> light_probes;
  void CalculateIllumination(const RayProperties& ray_properties, int seed, float push_normal_distance);
  bool OnInspect();
};
}  // namespace evo_engine