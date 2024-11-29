#pragma once

#include "Climate.hpp"
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

  bool need_full_flow_update = false;

  int visualization_camera_resolution_x = 1;
  int visualization_camera_resolution_y = 1;
  glm::vec2 visualization_camera_mouse_position;

  TreeMeshGeneratorSettings mesh_generator_settings;
  StrandModelMeshGeneratorSettings strand_mesh_generator_settings{};
  SkeletalGraphSettings skeletal_graph_settings{};

  Entity selected_tree = {};

  [[nodiscard]] glm::vec2 GetMouseSceneCameraPosition() const;

  void Simulate(const SimulationSettings& target_simulation_settings);
  void Simulate();

  void GenerateMeshes(const TreeMeshGeneratorSettings& target_mesh_generator_settings) const;
  void GenerateSkeletalGraphs(const SkeletalGraphSettings& target_skeletal_graph_settings) const;
  void ClearMeshes() const;
  void ClearSkeletalGraphs() const;
  void GenerateStrandModelProfiles() const;
  void GenerateStrandModelMeshes(
      const StrandModelMeshGeneratorSettings& target_strand_model_mesh_generator_settings) const;
  void ClearStrandModelMeshes() const;

  void GenerateStrandRenderers() const;
  void ClearStrandRenderers() const;

  void ResetAllTrees(const std::vector<Entity>* tree_entities);

  static std::weak_ptr<Climate> FindClimate();
  static std::weak_ptr<Soil> FindSoil();

  const std::vector<glm::vec3>& RandomColors();

 private:
  struct Fruit {
    GlobalTransform global_transform;
    float m_maturity = 0.0f;
    float m_health = 1.0f;
  };

  struct Leaf {
    GlobalTransform global_transform;
    float m_maturity = 0.0f;
    float m_health = 1.0f;
  };

  enum class TreeOperatorMode { None, Select, Rotate, Prune, Invigorate, Reduce };
  unsigned tree_operator_mode = static_cast<unsigned>(TreeOperatorMode::None);
  float tree_reduce_rate = 0.1f;
  struct TreeVisualizerSettings {
    bool display_shoot_stem = true;
    bool display_foliage = true;
    bool display_fruit = true;
    bool display_bounding_box = false;
    bool display_soil = false;
    bool display_ground_fruit = true;
    bool display_ground_leaves = true;
    bool show_shadow_grid = false;
    bool show_lighting_grid = false;
    void OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);
  };

  struct DynamicStrandsVisualizerSettings {
    float drag_multiplier = 1.f;
    enum class DynamicStrandsTransformMode { None, Translate, Rotate };
    unsigned transform_mode = static_cast<unsigned>(DynamicStrandsTransformMode::Translate);
    void OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);
  };
  TreeVisualizerSettings tree_visualizer_settings_;
  DynamicStrandsVisualizerSettings dynamic_strands_visualizer_settings_;
  bool auto_generate_mesh_after_editing_ = false;
  bool auto_generate_skeletal_graph_every_frame_ = false;
  bool auto_generate_strands_after_editing_ = false;
  bool auto_generate_strand_mesh_after_editing_ = false;

  friend class TreeVisualizer;
  friend class Tree;
  bool show_trees = true;
  bool show_strands = true;
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

  float last_used_time_ = 0.0f;
  float total_time_ = 0.0f;
  int internode_size_ = 0;
  int leaf_size_ = 0;
  int fruit_size_ = 0;
  int shoot_stem_size_ = 0;
  int root_node_size_ = 0;
  int root_stem_size_ = 0;

  bool need_flow_update_for_selection_ = false;
  int last_selected_tree_index_ = -1;

  int soil_version_ = -1;
  bool vector_enable_ = false;
  bool scalar_enable_ = true;
  bool update_vector_matrices_ = false;
  bool update_scalar_matrices_ = false;
  float vector_multiplier_ = 50.0f;
  glm::vec4 vector_base_color_ = glm::vec4(1.0f, 1.0f, 1.0f, 0.8f);
  unsigned vector_soil_property_ = 4;
  float vector_line_width_factor_ = 0.1f;
  float vector_line_max_width_ = 0.1f;
  std::shared_ptr<ParticleInfoList> vector_matrices_;

  float scalar_multiplier_ = 1.0f;
  float scalar_box_size_ = 1.0f;
  float scalar_min_alpha_ = 0.00f;

  std::vector<glm::vec4> soil_layer_colors_;

  friend class Soil;

  float soil_cutout_x_depth_ = 0.0f;
  float soil_cutout_z_depth_ = 0.0f;

  glm::vec3 scalar_base_color_ = glm::vec3(0.0f, 0.0f, 1.0f);
  unsigned scalar_soil_property_ = 1;
  std::shared_ptr<ParticleInfoList> scalar_matrices_;

  std::shared_ptr<ParticleInfoList> shadow_grid_particle_info_list_;
  std::shared_ptr<ParticleInfoList> lighting_grid_particle_info_list_;

  float simulated_time_;
  std::vector<Fruit> fruits_;
  std::vector<Leaf> leaves_;
  std::shared_ptr<Camera> visualization_camera_;

  bool visualization_camera_window_focused_ = false;

  void PreUpdate() override;
  void OnCreate() override;

  void LateUpdate() override;
  void TreeVisualization(const std::shared_ptr<EditorLayer>& editor_layer);
  void StrandVisualization(const std::shared_ptr<EditorLayer>& editor_layer);
  void OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void OnSoilVisualizationMenu();
  void UpdateFlows(const std::vector<Entity>* tree_entities, const std::shared_ptr<Strands>& branch_strands);
  void ClearGroundFruitAndLeaf();
  void UpdateGroundFruitAndLeaves() const;
  // helper functions to structure code a bit
  void SoilVisualization();
  void SoilVisualizationScalar(const VoxelSoilModel& soil_model);  // called during LateUpdate()
  void SoilVisualizationVector(const VoxelSoilModel& soil_model);  // called during LateUpdate()
};
}  // namespace eco_sys_lab_plugin
