//
// Created by lllll on 11/6/2021.
//

#include "LightProbeGroup.hpp"
#include "RayTracerLayer.hpp"
using namespace evo_engine;
void LightProbeGroup::CalculateIllumination(const RayProperties& ray_properties, int seed, float push_normal_distance) {
  if (light_probes.empty())
    return;
  CudaModule::EstimateIlluminationRayTracing(Application::GetLayer<RayTracerLayer>()->environment_properties,
                                             ray_properties, light_probes, seed, push_normal_distance);
}

bool LightProbeGroup::OnInspect() {
  ImGui::Text("Light probes size: %llu", light_probes.size());
  return false;
}
