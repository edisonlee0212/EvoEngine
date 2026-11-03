#pragma once

#include <cstdint>
#include "Climate.hpp"
#include "DynamicStrands.hpp"
#include "DynamicStrandsVisualizationParameters.hpp"
#include "EcoSysLabSystem.hpp"
#include "ILayer.hpp"
#include "SimulationSettings.hpp"
#include "Soil.hpp"
#include "Strands.hpp"
#include "Tree.hpp"

namespace eco_sys_lab_package {
using namespace evo_engine;
class GpuTreeMeshRenderer;
class GpuTreePhysics;
class DynamicTreeSkeleton;
class WindParticleField;
class EcoSysLabSystem;
class EcoSysLabDemoBootstrap;
class EcoSysLabLayer : public ILayer {
 public:
  EcoSysLabLayer();
  bool visualize_wind_particles = true;
  bool visualize_air_grid = false;
  bool visualize_air_coverage = false;
  std::vector<glm::vec4> soil_layer_colors;
  void ClearMeshes() const;
  void ClearSkeletalGraphs() const;
  void ClearStrandModelMeshes() const;
  void ClearStrandRenderers() const;
  void ExportAllTrees(const std::filesystem::path& path) const;
  void GenerateDynamicStrandsForAllTrees() const;
  void GenerateMeshes(const TreeMeshGeneratorSettings& target_mesh_generator_settings) const;
  void GenerateSkeletalGraphs(const SkeletalGraphSettings& target_skeletal_graph_settings) const;
  void GenerateStrandModelMeshes(
      const StrandModelMeshGeneratorSettings& target_strand_model_mesh_generator_settings) const;
  void GenerateStrandModelProfiles() const;
  void GenerateStrandRenderers() const;
  [[nodiscard]] float GetSimulatedTime() const;
  [[nodiscard]] bool IsDynamicStrandsPhysicsRunning() const;
  [[nodiscard]] bool IsDynamicStrandsFungusRunning() const;
  [[nodiscard]] int GetDynamicStrandsFungusStepsPerFrame() const;
  EcoSysLabSystem::Dynamics::StrandsSettings& GetDynamicStrandsSettings() const;
  void OnDestroy() override;
  void RetireDynamics(std::shared_ptr<GpuTreePhysics> physics, std::shared_ptr<WindParticleField> wind,
                      std::shared_ptr<GpuTreeClusterForest> clusters = {});
  void RefreshMeshForAllDynamicStrands() const;
  void ResetAllTrees(const std::vector<Entity>* tree_entities);
  void ResetAirGrid(float cell_size = -1.0f);
  bool Simulate();
  TreeMeshGeneratorSettings mesh_generator_settings;
  SkeletalGraphSettings skeletal_graph_settings{};
  StrandModelMeshGeneratorSettings strand_mesh_generator_settings{};
  bool Simulate(const SimulationSettings& target_simulation_settings, SimulationStats& target_simulation_stats);
  bool SimulateForDataset(const SimulationSettings& settings, SimulationStats& stats);

 private:
  struct RetiredDynamics {
    std::shared_ptr<FrameSubmissionState> submission;
    std::shared_ptr<GpuTreePhysics> physics;
    std::shared_ptr<WindParticleField> wind;
    std::shared_ptr<GpuTreeClusterForest> clusters;
  };
  std::vector<RetiredDynamics> retired_dynamics_;
  friend class EcoSysLabEditorLayer;
  friend class EcoSysLabSystem;
  friend class EcoSysLabDemoBootstrap;
  void ClearGroundFruitAndLeaf();
  void OnCreate() override;
  void RegisterStrandRenderingProcedure() const;
  std::shared_ptr<GpuTreeMeshRenderer> gpu_tree_mesh_renderer_;
  void RegisterTypes(Application& application) override;
  void Update() override;
  void UpdateDemoTreeGrowth();
  std::shared_ptr<EcoSysLabDemoBootstrap> demo_bootstrap_;
  bool demo_growth_enabled_ = true;
};
}  // namespace eco_sys_lab_package
