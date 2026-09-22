#pragma once

#include <cstdint>
#include "Climate.hpp"
#include "DynamicSkeleton.hpp"
#include "DynamicStrands.hpp"
#include "DynamicStrandsVisualizationParameters.hpp"
#include "ILayer.hpp"
#include "SimulationSettings.hpp"
#include "Soil.hpp"
#include "Strands.hpp"
#include "Tree.hpp"

namespace eco_sys_lab_package {
using namespace evo_engine;
class EcoSysLabLayer : public ILayer {
 public:
  EcoSysLabLayer();
  std::vector<glm::vec4> soil_layer_colors;
  void ClearMeshes() const;
  void ClearSkeletalGraphs() const;
  void ClearStrandModelMeshes() const;
  void ClearStrandRenderers() const;
  void ExportAllTrees(const std::filesystem::path& path) const;
  static std::weak_ptr<Climate> FindClimate();
  static std::weak_ptr<Soil> FindSoil();
  void GenerateDynamicSkeletonForAllTrees() const;
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
  void OnDestroy() override;
  void RefreshMeshForAllDynamicStrands() const;
  void ResetAllTrees(const std::vector<Entity>* tree_entities);
  bool Simulate();
  TreeMeshGeneratorSettings mesh_generator_settings;
  SimulationSettings simulation_settings{};
  SimulationStats simulation_stats{};
  SkeletalGraphSettings skeletal_graph_settings{};
  StrandModelMeshGeneratorSettings strand_mesh_generator_settings{};
  bool Simulate(const SimulationSettings& target_simulation_settings, SimulationStats& target_simulation_stats);

 private:
  friend class EcoSysLabEditorLayer;
  friend struct DynamicTreeSkeletonInspector;
  void ClearGroundFruitAndLeaf();
  void DynamicSkeletonPhysics() const;
  struct DynamicSkeletonSettings {
    bool enable_physics = true;
    DynamicSkeleton::PhysicsParameters physics_parameters{};
  };
  void DynamicStrandSimulation();
  struct DynamicStrandsSettings {
    FoliageRenderParameters foliage_render_parameters{};
    SegmentPairsRenderParameters segment_pairs_render_parameters{};
    DynamicStrands::PhysicsParameters physics_parameters{};
    DynamicStrands::FungusParameters fungus_parameters{};
    bool enable_physics = true;
    bool enable_fungus = true;
    bool enable_geometry_updates = true;
    bool enable_rendering = true;
    int remaining_step = 0;
    int remaining_fungus_step = 0;
    int remaining_geometry_step = 0;
    int fungus_sub_step = 25;
  };
  struct Flower {
    GlobalTransform global_transform;  ///< The global transform of the leaf.
    float flower_maturity = 0.0f;      ///< The maturity level of the leaf.
    float flower_health = 1.0f;        ///< The health level of the leaf.
  };
  struct Fruit {
    GlobalTransform global_transform;  ///< The global transform of the fruit.
    float fruit_maturity = 0.0f;       ///< The maturity level of the fruit.
    float fruit_health = 1.0f;         ///< The health level of the fruit.
  };
  struct Leaf {
    GlobalTransform global_transform;  ///< The global transform of the leaf.
    float leaf_maturity = 0.0f;        ///< The maturity level of the leaf.
    float leaf_health = 1.0f;          ///< The health level of the leaf.
  };
  void OnCreate() override;
  void RegisterStrandRenderingProcedure() const;
  void RegisterTypes(Application& application) override;
  void Update() override;
  void UpdateDemoTreeGrowth();
  Entity demo_tree_entity_{};
  bool demo_tree_growth_finished_ = false;
  bool demo_tree_initialized_ = false;
  DynamicSkeletonSettings dynamic_skeleton_settings_;
  DynamicStrandsSettings dynamic_strands_settings_;
  std::vector<Flower> flowers_;
  std::vector<Fruit> fruits_;
  std::vector<Leaf> leaves_;
  float simulated_time_ = 0.0f;
  uint64_t simulation_revision_ = 0;
  uint64_t reset_revision_ = 0;
  bool demo_growth_enabled_ = true;
};
}  // namespace eco_sys_lab_package
