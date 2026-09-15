#pragma once
#include "InspectorRegistry.hpp"
#include "SpatialPlantDistributionSimulator.hpp"
#include "imgui.h"
namespace eco_sys_lab_package {
struct SpatialPlantDistributionInspector {
  glm::vec2 scrolling = glm::vec2(0.0f);
  float zoomFactor = 2.f;
  bool enableUniformSize = false;
  float uniformSize = 4.f;
  float plantSizeFactor = 4.f;
  int plantSize = 10;
  float initialRadius = 1.f;
  int parameterHandle = 0;
  bool setParent = true;
  float range = 10.0f;
  glm::vec2 offset = glm::vec2(.0f);
  float positionZoom = 0.5f;
  bool Inspect(evo_engine::InspectorContext& context, SpatialPlantDistributionSimulator& target);
  void DrawCanvas(const SpatialPlantDistribution& spatialPlantDistribution, const std::function<void(glm::vec2)>& func,
                  const std::function<void(ImVec2, float, ImDrawList*)>& drawFunc);
};
}  // namespace eco_sys_lab_package
