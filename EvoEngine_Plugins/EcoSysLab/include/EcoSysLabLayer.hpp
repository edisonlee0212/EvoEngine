#pragma once

#include "Climate.hpp"
#include "DynamicSkeleton.hpp"
#include "DynamicStrands.hpp"
#include "SimulationSettings.hpp"
#include "Soil.hpp"
#include "Strands.hpp"
#include "Tree.hpp"
using namespace evo_engine;
namespace eco_sys_lab_plugin {
class EcoSysLabLayer : public ILayer {
 public:
  [[nodiscard]] float GetSimulatedTime() const;
  void ExportAllTrees(const std::filesystem::path& path) const;

  SimulationSettings simulation_settings{};
  SimulationStats simulation_stats{};
  bool need_full_flow_update = false;

  int visualization_camera_resolution_x = 1;
  int visualization_camera_resolution_y = 1;
  glm::vec2 visualization_camera_mouse_position;

  TreeMeshGeneratorSettings mesh_generator_settings;
  StrandModelMeshGeneratorSettings strand_mesh_generator_settings{};
  SkeletalGraphSettings skeletal_graph_settings{};

  Entity selected_tree = {};

  [[nodiscard]] glm::vec2 GetMouseSceneCameraPosition() const;

  bool Simulate(const SimulationSettings& target_simulation_settings, SimulationStats& target_simulation_stats);
  bool Simulate();

  void GenerateMeshes(const TreeMeshGeneratorSettings& target_mesh_generator_settings) const;
  void GenerateSkeletalGraphs(const SkeletalGraphSettings& target_skeletal_graph_settings) const;
  void ClearMeshes() const;
  void ClearSkeletalGraphs() const;
  void GenerateStrandModelProfiles() const;
  void GenerateStrandModelMeshes(
      const StrandModelMeshGeneratorSettings& target_strand_model_mesh_generator_settings) const;
  void ClearStrandModelMeshes() const;
  void GenerateDynamicStrandsForAllTrees() const;
  void GenerateDynamicSkeletonForAllTrees() const;
  void GenerateStrandRenderers() const;
  void ClearStrandRenderers() const;

  void ResetAllTrees(const std::vector<Entity>* tree_entities);

  static std::weak_ptr<Climate> FindClimate();
  static std::weak_ptr<Soil> FindSoil();

  const std::vector<glm::vec3>& RandomColors();

 private:
  struct Fruit {
    GlobalTransform global_transform;
    float fruit_maturity = 0.0f;
    float fruit_health = 1.0f;
  };

  struct Leaf {
    GlobalTransform global_transform;
    float leaf_maturity = 0.0f;
    float leaf_health = 1.0f;
  };

  enum class TreeOperatorMode { Disabled, Select, Rotate, Prune, Invigorate, Reduce };
  unsigned tree_operator_mode = static_cast<unsigned>(TreeOperatorMode::Disabled);
  float tree_reduce_rate = 0.1f;
  struct TreeVisualizationSettings {
    bool enable = true;

    bool display_shoot_stem = true;
    bool display_foliage = true;
    bool display_fruit = true;
    bool display_bounding_box = false;

    bool display_ground_fruit = true;
    bool display_ground_leaves = true;
    bool show_shadow_grid = false;
    bool show_lighting_grid = false;
    bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);
  };

  struct DynamicStrandsSettings {
    DynamicStrands::BranchesRenderParameters branches_render_parameters{};
    DynamicStrands::SmallSegmentsRenderParameters small_segments_render_parameters{};
    DynamicStrands::FoliageRenderParameters foliage_render_parameters{};
    DynamicStrands::PhysicsParameters physics_parameters{};
    DynamicStrands::VisualizationParameters visualization_parameters{};

    float drag_multiplier = 1.f;
    enum class TransformMode { Disabled, Translate, Rotate };
    unsigned transform_mode = static_cast<unsigned>(TransformMode::Disabled);
    enum class OperatorMode { Drag, Saw, LineCut };
    unsigned operator_mode = static_cast<unsigned>(OperatorMode::Drag);
    bool cut_bend_twist_bundle_only = false;
    bool enable = true;
    bool enable_physics = true;
    bool enable_rendering = true;
    void OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);
  };
  TreeVisualizationSettings tree_visualization_settings_;

  struct DynamicSkeletonSettings {
    bool enable_physics = true;
    bool enable_visualization = true;
    DynamicSkeleton::PhysicsParameters physics_parameters{};
    DynamicSkeleton::VisualizationParameters visualization_parameters{};
    void OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);
  };

  DynamicStrandsSettings dynamic_strands_settings_;
  DynamicSkeletonSettings dynamic_skeleton_settings_;
  bool auto_generate_mesh_after_editing_ = false;
  bool auto_generate_skeletal_graph_every_frame_ = false;
  bool auto_generate_strands_after_editing_ = false;
  bool auto_generate_strand_mesh_after_editing_ = false;

  friend class TreeVisualizer;
  friend class Tree;

  std::vector<int> shoot_versions_;
  std::vector<glm::vec3> random_colors_;

  std::vector<glm::uint> shoot_stem_segments_;
  std::vector<StrandPoint> shoot_stem_points_;

  AssetRef shoot_stem_strands_;

  std::shared_ptr<ParticleInfoList> bounding_box_matrices_;

  std::shared_ptr<ParticleInfoList> foliage_matrices_;
  std::shared_ptr<ParticleInfoList> fruit_matrices_;

  std::shared_ptr<ParticleInfoList> ground_fruit_matrices_;
  std::shared_ptr<ParticleInfoList> ground_leaf_matrices_;

  struct SoilVisualizationSettings {
    bool enable = false;
    bool vector_enable = false;
    bool scalar_enable = true;
    bool update_vector_matrices = false;
    bool update_scalar_matrices = false;
    float vector_multiplier = 50.0f;
    glm::vec4 vector_base_color = glm::vec4(1.0f, 1.0f, 1.0f, 0.8f);
    unsigned vector_soil_property = 4;
    float vector_line_width_factor = 0.1f;
    float vector_line_max_width = 0.1f;
    float scalar_multiplier = 1.0f;
    float scalar_box_size = 1.0f;
    float scalar_min_alpha = 0.00f;
    glm::vec3 scalar_base_color = glm::vec3(0.0f, 0.0f, 1.0f);
    unsigned scalar_soil_property = 1;
    float soil_cutout_x_depth = 0.0f;
    float soil_cutout_z_depth = 0.0f;
    std::vector<glm::vec4> soil_layer_colors;
    SoilVisualizationSettings();
    bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);
  };

  SoilVisualizationSettings soil_visualization_settings_{};

  bool need_flow_update_for_selection_ = false;
  int last_selected_tree_index_ = -1;

  int soil_version_ = -1;

  std::shared_ptr<ParticleInfoList> vector_matrices_;

  friend class Soil;

  std::shared_ptr<ParticleInfoList> scalar_matrices_;

  std::shared_ptr<ParticleInfoList> shadow_grid_particle_info_list_;
  std::shared_ptr<ParticleInfoList> lighting_grid_particle_info_list_;

  float simulated_time_;
  std::vector<Fruit> fruits_;
  std::vector<Leaf> leaves_;
  std::shared_ptr<Camera> visualization_camera_;

  bool visualization_camera_window_focused_ = false;

  void PreUpdate() override;
  void Update() override;
  void LateUpdate() override;
  void OnCreate() override;
  void TreeVisualization(const std::shared_ptr<EditorLayer>& editor_layer);
  void DynamicStrandsVisualization(const std::shared_ptr<EditorLayer>& editor_layer) const;
  void OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void UpdateFlows(const std::vector<Entity>* tree_entities, const std::shared_ptr<Strands>& branch_strands);
  void ClearGroundFruitAndLeaf();
  void UpdateGroundFruitAndLeaves() const;
  // helper functions to structure code a bit
  void SoilVisualization();
  void SoilVisualizationScalar(const VoxelSoilModel& soil_model);  // called during LateUpdate()
  void SoilVisualizationVector(const VoxelSoilModel& soil_model);  // called during LateUpdate()
  // This has to happen before LateUpdate.
  void RegisterStrandRenderingProcedure() const;
  void DynamicStrandPhysics() const;
  void DynamicSkeletonPhysics() const;
  void DynamicSkeletonVisualization() const;
  // This has to happen during LateUpdate.
  void DynamicStrandVisualization() const;
};
}  // namespace eco_sys_lab_plugin
