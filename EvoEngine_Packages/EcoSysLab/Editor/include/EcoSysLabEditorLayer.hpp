#pragma once
#include <unordered_map>
#include "EcoSysLabLayer.hpp"
#include "EditorLayer.hpp"
#include "TreeEditorState.hpp"
namespace eco_sys_lab_package {
class EcoSysLabEditorLayer : public evo_engine::ILayer {
  friend class ShootVisualizer;
  friend class RootVisualizer;
  friend struct DynamicTreeSkeletonInspector;
  struct Entry {
    std::weak_ptr<evo_engine::IPrivateComponent> owner;
    std::unique_ptr<TreeEditorState> state;
  };
  std::weak_ptr<evo_engine::Scene> scene_;
  std::unordered_map<const Tree*, Entry> trees_;

  void DrawDynamicStrandsSettingsGui(const std::shared_ptr<EditorLayer>& editor_layer);
  void DynamicStrandVisualization() const;
  void DynamicStrandsVisualization(const std::shared_ptr<EditorLayer>& editor_layer);
  void SoilVisualization();
  void SoilVisualizationScalar(const VoxelSoilModel& soil_model);
  struct SoilVisualizationSettings {
    bool force_update = false;
    bool enable = false;                  ///< Enables or disables soil visualization.
    bool vector_enable = false;           ///< Enables or disables vector field visualization.
    bool scalar_enable = true;            ///< Enables or disables scalar field visualization.
    bool update_vector_matrices = false;  ///< Flag indicating whether vector matrices should be updated.
    bool update_scalar_matrices = false;  ///< Flag indicating whether scalar matrices should be updated.

    float vector_multiplier = 50.0f;                                  ///< Multiplier for vector visualization.
    glm::vec4 vector_base_color = glm::vec4(1.0f, 1.0f, 1.0f, 0.8f);  ///< Base color for vector visualization.
    unsigned vector_soil_property = 4;  ///< Index of the soil property used for vector visualization.

    float vector_line_width_factor = 0.1f;  ///< Factor controlling the width of vector lines.
    float vector_line_max_width = 0.1f;     ///< Maximum width of vector lines.

    float scalar_multiplier = 1.0f;  ///< Multiplier for scalar visualization.
    float scalar_box_size = 1.0f;    ///< Box size for scalar visualization.
    float scalar_min_alpha = 0.00f;  ///< Minimum alpha transparency for scalar visualization.

    glm::vec3 scalar_base_color = glm::vec3(0.0f, 0.0f, 1.0f);  ///< Base color for scalar visualization.
    unsigned scalar_soil_property = 1;  ///< Index of the soil property used for scalar visualization.

    float soil_cutout_x_depth = 0.0f;  ///< Depth of the X-axis cutout in soil visualization.
    float soil_cutout_z_depth = 0.0f;  ///< Depth of the Z-axis cutout in soil visualization.

    /**
     * @brief Handles the inspection of soil visualization settings in the editor.
     * @param editor_layer The editor layer instance.
     * @return True if the asset content has not been modified.
     */
    bool DrawGui(const std::shared_ptr<EditorLayer>& editor_layer);
  };
  void SoilVisualizationVector(const VoxelSoilModel& soil_model);
  enum class TreeOperatorMode { Disabled, Select, Rotate, Prune, Invigorate, Reduce };
  void TreeVisualization(const std::shared_ptr<EditorLayer>& editor_layer);
  struct TreeVisualizationSettings {
    bool enable = true;  ///< Enables or disables tree visualization.

    bool display_shoot_stem = true;      ///< Toggles the display of shoot stems.
    bool display_foliage = true;         ///< Toggles the display of foliage.
    bool display_flowers = true;         ///< Toggles the display of flowers.
    bool display_fruits = true;          ///< Toggles the display of fruit.
    bool display_bounding_box = false;   ///< Toggles the display of bounding boxes.
    bool display_ground_flowers = true;  ///< Toggles the display of fallen flowers.
    bool display_ground_fruits = true;   ///< Toggles the display of fallen fruit.
    bool display_ground_leaves = true;   ///< Toggles the display of fallen leaves.
    bool show_shadow_grid = false;       ///< Toggles the display of the shadow grid.
    bool show_lighting_grid = false;     ///< Toggles the display of the lighting grid.

    /**
     * @brief Handles the inspection of tree visualization settings in the editor.
     * @param editor_layer The editor layer instance.
     * @return True if the asset content has not been modified.
     */
    bool DrawGui(const std::shared_ptr<EditorLayer>& editor_layer);
  };
  void UpdateFlows(const std::vector<Entity>* tree_entities, const std::shared_ptr<Strands>& branch_strands);
  void UpdateGroundFruitAndLeaves() const;
  bool auto_generate_mesh_after_editing_ = false;
  bool auto_generate_skeletal_graph_every_frame_ = false;
  bool auto_generate_strand_mesh_after_editing_ = false;
  bool auto_generate_strands_after_editing_ = false;
  std::shared_ptr<ParticleInfoList> bounding_box_matrices_;
  std::shared_ptr<ParticleInfoList> flower_matrices_;
  std::shared_ptr<ParticleInfoList> foliage_matrices_;
  std::shared_ptr<ParticleInfoList> fruit_matrices_;
  std::shared_ptr<ParticleInfoList> ground_flower_matrices_;
  std::shared_ptr<ParticleInfoList> ground_fruit_matrices_;
  std::shared_ptr<ParticleInfoList> ground_leaf_matrices_;
  int last_selected_tree_index_ = -1;
  std::shared_ptr<ParticleInfoList> lighting_grid_particle_info_list_;
  bool need_flow_update_for_selection_ = false;
  std::vector<glm::vec3> random_colors_;
  std::shared_ptr<ParticleInfoList> scalar_matrices_;
  std::shared_ptr<ParticleInfoList> shadow_grid_particle_info_list_;
  std::vector<StrandPoint> shoot_stem_points_;
  std::vector<glm::uint> shoot_stem_segments_;
  AssetRef shoot_stem_strands_;
  std::vector<int> shoot_versions_;
  std::shared_ptr<ParticleInfoList> soil_matrices_;
  int soil_version_ = -1;
  SoilVisualizationSettings soil_visualization_settings_{};
  unsigned tree_operator_mode = static_cast<unsigned>(TreeOperatorMode::Disabled);
  float tree_reduce_rate = 0.1f;
  TreeVisualizationSettings tree_visualization_settings_;
  std::shared_ptr<ParticleInfoList> vector_matrices_;
  std::shared_ptr<Camera> visualization_camera_;
  bool visualization_camera_window_focused_ = false;
  struct DynamicStrandsSettings {
    DynamicStrandsVisualizationParameters visualization_parameters{};
    float drag_multiplier = 1.f;
    float point_cut_thickness = 2.f;
    float fungus_injection_amount = 1.0f;
    bool fungus_white_rot = true;
    bool fungus_brown_rot = false;
    enum class TransformMode { Disabled, Translate, Rotate };
    unsigned transform_mode = static_cast<unsigned>(TransformMode::Disabled);
    enum class OperatorMode { Drag, Saw, LineCut, PointCut, FungusInjection };
    unsigned operator_mode = static_cast<unsigned>(OperatorMode::FungusInjection);
    bool cut_bend_twist_bundle_only = false;
    bool enable_visualization = true;
  };
  struct DynamicSkeletonSettings {
    bool enable_visualization = true;
    DynamicSkeleton::VisualizationParameters visualization_parameters{};
    void DrawGui(const std::shared_ptr<EditorLayer>& editor_layer);
  };
  DynamicStrandsSettings dynamic_strands_settings_;
  DynamicSkeletonSettings dynamic_skeleton_settings_;
  EditorCameraFreeFlyState visualization_camera_free_fly_state_;
  uint64_t simulation_revision_ = 0;
  uint64_t reset_revision_ = 0;
  void ClearSceneVisualization();
  bool auto_time_grow = false;
  float target_time = 0.0f;
  float extra_time = 4.f;
  int corner = 1;
  bool is_box_selection_previously = false;
  bool is_operating_previously = false;
  glm::vec2 strands_operator_mouse_start;
  glm::vec2 strands_operator_mouse_current;
  std::vector<glm::vec2> strand_operator_mouse_points;
  bool may_need_geometry_generation = false;
  std::vector<glm::vec2> mouse_positions{};
  bool last_gizmos_used = false;
  bool last_frame_invigorate = false;
  bool last_frame_reduce = false;
  float target_age = 0.0f;

 public:
  void OnCreate() override;
  void OnDestroy() override;
  void Update() override;
  void LateUpdate() override;
  TreeEditorState& GetTreeState(Tree& tree);
  const std::vector<glm::vec3>& RandomColors();
  void DrawGui(const std::shared_ptr<EditorLayer>& editor_layer);
  [[nodiscard]] glm::vec2 GetMouseSceneCameraPosition() const;
  void VisualizationCameraDragAndDrop() const;
  bool enable_visualization_background = false;
  bool need_full_flow_update = false;
  Entity selected_tree = {};
  bool show_visualization_camera_info = true;
  glm::vec2 visualization_camera_mouse_position;
  int visualization_camera_resolution_x = 1;
  int visualization_camera_resolution_y = 1;
};
}  // namespace eco_sys_lab_package
