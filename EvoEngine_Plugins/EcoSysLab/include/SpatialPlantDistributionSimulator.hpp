#pragma once
#include "SpatialPlantDistribution.hpp"
#include "TreeModel.hpp"
using namespace evo_engine;
namespace eco_sys_lab {
class SpatialPlantDistributionSimulator : public IPrivateComponent {
 public:
  TreeGrowthSettings m_treeGrowthSettings;
  std::vector<AssetRef> m_treeDescriptors{};

  SpatialPlantDistribution m_distribution{};
  bool m_simulate = false;
  static void OnInspectSpatialPlantDistributionFunction(
      const SpatialPlantDistribution& spatialPlantDistribution, const std::function<void(glm::vec2 position)>& func,
      const std::function<void(ImVec2 origin, float zoomFactor, ImDrawList*)>& drawFunc);
  bool OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;
  void FixedUpdate() override;
  void OnCreate() override;
};

}  // namespace eco_sys_lab