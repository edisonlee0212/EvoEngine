#pragma once

#include "Application.hpp"
#include "EditorLayer.hpp"
#include "Graphics.hpp"
#include "Jobs.hpp"
#include "StrandModel.hpp"
#include "TreeModel.hpp"
using namespace evo_engine;
namespace eco_sys_lab {
enum class ShootVisualizerMode {
  Default,
  Order,
  Level,
  MaxDescendantLightIntensity,
  LightIntensity,
  LightDirection,

  DesiredGrowthRate,
  GrowthPotential,
  GrowthRate,

  IsMaxChild,
  AllocatedVigor,

  SaggingStress,

  Locked
};

enum class RootVisualizerMode {
  Default,
  AllocatedVigor,
};

struct TreeVisualizerColorSettings {
  int m_shootVisualizationMode = static_cast<int>(ShootVisualizerMode::Default);
  float m_shootColorMultiplier = 1.0f;
};

class TreeVisualizer {
  bool m_initialized = false;

  std::vector<glm::vec4> m_randomColors;

  std::shared_ptr<ParticleInfoList> m_internodeMatrices;

  bool DrawInternodeInspectionGui(TreeModel& treeModel, SkeletonNodeHandle internodeHandle, bool& deleted,
                                  const unsigned& hierarchyLevel);

  void PeekNodeInspectionGui(const ShootSkeleton& skeleton, SkeletonNodeHandle nodeHandle,
                             const unsigned& hierarchyLevel);

  void PeekInternode(const ShootSkeleton& shootSkeleton, SkeletonNodeHandle internodeHandle) const;

  bool InspectInternode(ShootSkeleton& shootSkeleton, SkeletonNodeHandle internodeHandle);

 public:
  bool RayCastSelection(const std::shared_ptr<Camera>& cameraComponent, const glm::vec2& mousePosition,
                        const ShootSkeleton& skeleton, const GlobalTransform& globalTransform);

  bool ScreenCurveSelection(const std::function<void(SkeletonNodeHandle)>& handler,
                            std::vector<glm::vec2>& mousePositions, ShootSkeleton& skeleton,
                            const GlobalTransform& globalTransform);

  std::vector<SkeletonNodeHandle> m_selectedInternodeHierarchyList;
  SkeletonNodeHandle m_selectedInternodeHandle = -1;
  bool m_visualization = true;
  TreeVisualizerColorSettings m_settings;
  float m_lineThickness = 0.f;
  bool m_profileGui = true;
  bool m_treeHierarchyGui = false;
  float m_selectedInternodeLengthFactor = 0.0f;
  int m_checkpointIteration = 0;
  bool m_needUpdate = false;

  [[nodiscard]] bool Initialized() const;
  void ClearSelections();

  void Initialize();

  void SetSelectedNode(const ShootSkeleton& skeleton, SkeletonNodeHandle nodeHandle);

  void SyncMatrices(const ShootSkeleton& skeleton, const std::shared_ptr<ParticleInfoList>& particleInfoList,
                    SkeletonNodeHandle selectedNodeHandle);

  bool OnInspect(TreeModel& treeModel);

  void Visualize(const TreeModel& treeModel, const GlobalTransform& globalTransform);
  void Visualize(StrandModel& strandModel);
  void Reset(TreeModel& treeModel);

  void Clear();
};
}  // namespace eco_sys_lab