#pragma once

#include "Climate.hpp"
#include "SimulationSettings.hpp"
#include "Soil.hpp"
#include "Strands.hpp"
#include "Tree.hpp"
using namespace evo_engine;
namespace eco_sys_lab {
struct Fruit {
  GlobalTransform m_globalTransform;
  float m_maturity = 0.0f;
  float m_health = 1.0f;
};

enum class OperatorMode { Select, Rotate, Prune, Invigorate, Reduce };

struct Leaf {
  GlobalTransform m_globalTransform;
  float m_maturity = 0.0f;
  float m_health = 1.0f;
};

class EcoSysLabLayer : public ILayer {
  unsigned m_operatorMode = static_cast<unsigned>(OperatorMode::Select);
  float m_reduceRate = 0.1f;
  bool m_autoGenerateMeshAfterEditing = false;
  bool m_autoGenerateSkeletalGraphEveryFrame = false;
  bool m_autoGenerateStrandsAfterEditing = false;
  bool m_autoGenerateStrandMeshAfterEditing = false;

  friend class TreeVisualizer;
  friend class Tree;
  bool m_displayShootStem = true;
  bool m_displayFoliage = true;
  bool m_displayFruit = true;
  bool m_displayBoundingBox = false;
  bool m_displaySoil = false;
  bool m_displayGroundFruit = true;
  bool m_displayGroundLeaves = true;

  bool m_visualization = true;
  std::vector<int> m_shootVersions;
  std::vector<glm::vec3> m_randomColors;

  std::vector<glm::uint> m_shootStemSegments;
  std::vector<StrandPoint> m_shootStemPoints;

  AssetRef m_shootStemStrands;

  std::shared_ptr<ParticleInfoList> m_boundingBoxMatrices;

  std::shared_ptr<ParticleInfoList> m_foliageMatrices;
  std::shared_ptr<ParticleInfoList> m_fruitMatrices;

  std::shared_ptr<ParticleInfoList> m_groundFruitMatrices;
  std::shared_ptr<ParticleInfoList> m_groundLeafMatrices;

  float m_lastUsedTime = 0.0f;
  float m_totalTime = 0.0f;
  int m_internodeSize = 0;
  int m_leafSize = 0;
  int m_fruitSize = 0;
  int m_shootStemSize = 0;
  int m_rootNodeSize = 0;
  int m_rootStemSize = 0;

  bool m_needFlowUpdateForSelection = false;
  int m_lastSelectedTreeIndex = -1;

  int m_soilVersion = -1;
  bool m_vectorEnable = false;
  bool m_scalarEnable = true;
  bool m_updateVectorMatrices = false;
  bool m_updateScalarMatrices = false;
  float m_vectorMultiplier = 50.0f;
  glm::vec4 m_vectorBaseColor = glm::vec4(1.0f, 1.0f, 1.0f, 0.8f);
  unsigned m_vectorSoilProperty = 4;
  float m_vectorLineWidthFactor = 0.1f;
  float m_vectorLineMaxWidth = 0.1f;
  std::shared_ptr<ParticleInfoList> m_vectorMatrices;

  float m_scalarMultiplier = 1.0f;
  float m_scalarBoxSize = 1.0f;
  float m_scalarMinAlpha = 0.00f;

  std::vector<glm::vec4> m_soilLayerColors;

  friend class Soil;

  float m_soilCutoutXDepth = 0.0f;
  float m_soilCutoutZDepth = 0.0f;

  glm::vec3 m_scalarBaseColor = glm::vec3(0.0f, 0.0f, 1.0f);
  unsigned m_scalarSoilProperty = 1;
  std::shared_ptr<ParticleInfoList> m_scalarMatrices;

  bool m_showShadowGrid = false;
  bool m_showLightingGrid = false;
  std::shared_ptr<ParticleInfoList> m_shadowGridParticleInfoList;
  std::shared_ptr<ParticleInfoList> m_lightingGridParticleInfoList;

  void PreUpdate() override;

  void OnCreate() override;

  void Visualization();

  void OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;

  void OnSoilVisualizationMenu();

  void UpdateFlows(const std::vector<Entity>* treeEntities, const std::shared_ptr<Strands>& branchStrands);

  void ClearGroundFruitAndLeaf();

  void UpdateGroundFruitAndLeaves() const;

  // helper functions to structure code a bit
  void SoilVisualization();

  void SoilVisualizationScalar(VoxelSoilModel& soilModel);  // called during LateUpdate()
  void SoilVisualizationVector(VoxelSoilModel& soilModel);  // called during LateUpdate()

  float m_simulatedTime;

  std::vector<Fruit> m_fruits;
  std::vector<Leaf> m_leaves;

  std::shared_ptr<Camera> m_visualizationCamera;

  glm::vec2 m_visualizationCameraMousePosition;
  bool m_visualizationCameraWindowFocused = false;

 public:
  [[nodiscard]] float GetSimulatedTime() const;
  void ExportAllTrees(const std::filesystem::path& path) const;

  SimulationSettings m_simulationSettings{};

  bool m_needFullFlowUpdate = false;

  int m_visualizationCameraResolutionX = 1;
  int m_visualizationCameraResolutionY = 1;

  TreeMeshGeneratorSettings m_meshGeneratorSettings;
  StrandModelMeshGeneratorSettings m_strandMeshGeneratorSettings{};
  SkeletalGraphSettings m_skeletalGraphSettings{};

  Entity m_selectedTree = {};

  [[nodiscard]] glm::vec2 GetMouseSceneCameraPosition() const;

  void Simulate(const SimulationSettings& simulationSettings);
  void Simulate();

  void GenerateMeshes(const TreeMeshGeneratorSettings& meshGeneratorSettings) const;
  void GenerateSkeletalGraphs(const SkeletalGraphSettings& skeletalGraphSettings) const;
  void ClearMeshes() const;
  void ClearSkeletalGraphs() const;
  void GenerateStrandModelProfiles() const;
  void GenerateStrandModelMeshes(const StrandModelMeshGeneratorSettings& strandModelMeshGeneratorSettings) const;
  void ClearStrandModelMeshes() const;

  void GenerateStrandRenderers() const;
  void ClearStrandRenderers() const;

  void ResetAllTrees(const std::vector<Entity>* treeEntities);

  static std::weak_ptr<Climate> FindClimate();
  static std::weak_ptr<Soil> FindSoil();

  const std::vector<glm::vec3>& RandomColors();
};

}  // namespace eco_sys_lab
