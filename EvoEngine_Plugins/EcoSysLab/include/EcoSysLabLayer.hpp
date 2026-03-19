#pragma once

#include "Climate.hpp"
#include "DynamicSkeleton.hpp"
#include "DynamicStrands.hpp"
#include "DynamicStrandsVisualizationParameters.hpp"
#include "SimulationSettings.hpp"
#include "Soil.hpp"
#include "Strands.hpp"
#include "Tree.hpp"

namespace YAML {
class Emitter;
class Node;
}

namespace eco_sys_lab_plugin {
using namespace evo_engine;

/**
 * @class EcoSysLabLayer
 * @brief Represents a layer for managing the EcoSysLab plugin, providing simulation and visualization functionalities.
 */
class EcoSysLabLayer : public ILayer {
 public:
  /**
   * @brief Enables drag and drop functionality for the visualization camera.
   */
  void VisualizationCameraDragAndDrop() const;

  /**
   * @brief Gets the current simulated time.
   * @return The simulated time in float format.
   */
  [[nodiscard]] float GetSimulatedTime() const;

  /**
   * @brief Exports all tree data to a specified file path.
   * @param path The file path where data should be exported.
   */
  void ExportAllTrees(const std::filesystem::path& path) const;
  bool show_visualization_camera_info = true;
  bool enable_visualization_background = false;
  SimulationSettings simulation_settings{};  ///< The simulation settings used for eco-system simulation.
  SimulationStats simulation_stats{};        ///< The statistics of the simulation.
  bool need_full_flow_update = false;        ///< Flag indicating if a full flow update is required.

  int visualization_camera_resolution_x = 1;      ///< Resolution X for the visualization camera.
  int visualization_camera_resolution_y = 1;      ///< Resolution Y for the visualization camera.
  glm::vec2 visualization_camera_mouse_position;  ///< The mouse position in the visualization camera.

  TreeMeshGeneratorSettings mesh_generator_settings;                  ///< Settings for generating tree meshes.
  StrandModelMeshGeneratorSettings strand_mesh_generator_settings{};  ///< Settings for generating strand model meshes.
  SkeletalGraphSettings skeletal_graph_settings{};                    ///< Settings for skeletal graph generation.

  Entity selected_tree = {};  ///< The currently selected tree entity.

  /**
   * @brief Gets the mouse position in the scene camera space.
   * @return The mouse position as a 2D vector.
   */
  [[nodiscard]] glm::vec2 GetMouseSceneCameraPosition() const;

  /**
   * @brief Simulates the eco-system based on the given settings.
   * @param target_simulation_settings The target simulation settings.
   * @param target_simulation_stats The simulation statistics to store results.
   * @return True if the simulation was successful.
   */
  bool Simulate(const SimulationSettings& target_simulation_settings, SimulationStats& target_simulation_stats);

  /**
   * @brief Runs the simulation with the current settings.
   * @return True if the simulation was successful.
   */
  bool Simulate();

  /**
   * @brief Generates tree meshes based on the specified generator settings.
   * @param target_mesh_generator_settings The settings for mesh generation.
   */
  void GenerateMeshes(const TreeMeshGeneratorSettings& target_mesh_generator_settings) const;

  /**
   * @brief Generates skeletal graphs for trees based on the specified settings.
   * @param target_skeletal_graph_settings The settings for skeletal graph generation.
   */
  void GenerateSkeletalGraphs(const SkeletalGraphSettings& target_skeletal_graph_settings) const;

  /**
   * @brief Clears all meshes.
   */
  void ClearMeshes() const;

  /**
   * @brief Clears all skeletal graphs.
   */
  void ClearSkeletalGraphs() const;

  /**
   * @brief Generates strand model profiles.
   */
  void GenerateStrandModelProfiles() const;

  /**
   * @brief Generates strand model meshes based on the specified settings.
   * @param target_strand_model_mesh_generator_settings The settings for strand model mesh generation.
   */
  void GenerateStrandModelMeshes(
      const StrandModelMeshGeneratorSettings& target_strand_model_mesh_generator_settings) const;

  /**
   * @brief Clears all strand model meshes.
   */
  void ClearStrandModelMeshes() const;

  /**
   * @brief Generates dynamic strands for all trees.
   */
  void GenerateDynamicStrandsForAllTrees() const;

  /**
   * @brief Refreshes the mesh for all dynamic strands.
   */
  void RefreshMeshForAllDynamicStrands() const;

  /**
   * @brief Generates dynamic skeletons for all trees.
   */
  void GenerateDynamicSkeletonForAllTrees() const;

  /**
   * @brief Generates renderers for strands.
   */
  void GenerateStrandRenderers() const;

  /**
   * @brief Clears all strand renderers.
   */
  void ClearStrandRenderers() const;

  /**
   * @brief Resets all tree simulations for the given entities.
   * @param tree_entities The list of tree entities to reset.
   */
  void ResetAllTrees(const std::vector<Entity>* tree_entities);

  /**
   * @brief Finds and retrieves the climate settings.
   * @return A weak pointer to the Climate object.
   */
  static std::weak_ptr<Climate> FindClimate();

  /**
   * @brief Finds and retrieves the soil settings.
   * @return A weak pointer to the Soil object.
   */
  static std::weak_ptr<Soil> FindSoil();

  /**
   * @brief Returns a list of random colors.
   * @return A reference to the vector of random colors.
   */
  const std::vector<glm::vec3>& RandomColors();

 private:
  /**
   * @struct Fruit
   * @brief Represents a fruit with its transformation and properties.
   */
  struct Fruit {
    GlobalTransform global_transform;  ///< The global transform of the fruit.
    float fruit_maturity = 0.0f;       ///< The maturity level of the fruit.
    float fruit_health = 1.0f;         ///< The health level of the fruit.
  };

  /**
   * @struct Leaf
   * @brief Represents a leaf with its transformation and properties.
   */
  struct Leaf {
    GlobalTransform global_transform;  ///< The global transform of the leaf.
    float leaf_maturity = 0.0f;        ///< The maturity level of the leaf.
    float leaf_health = 1.0f;          ///< The health level of the leaf.
  };

  /**
   * @struct Leaf
   * @brief Represents a leaf with its transformation and properties.
   */
  struct Flower {
    GlobalTransform global_transform;  ///< The global transform of the leaf.
    float flower_maturity = 0.0f;      ///< The maturity level of the leaf.
    float flower_health = 1.0f;        ///< The health level of the leaf.
  };

  /**
   * @enum TreeOperatorMode
   * @brief Defines different operational modes for tree manipulation.
   */
  enum class TreeOperatorMode { Disabled, Select, Rotate, Prune, Invigorate, Reduce };

  unsigned tree_operator_mode = static_cast<unsigned>(TreeOperatorMode::Disabled);  ///< Current tree operator mode.
  float tree_reduce_rate = 0.1f;  ///< The reduction rate for tree pruning.

  /**
   * @struct TreeVisualizationSettings
   * @brief Holds settings for tree visualization.
   */
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
    bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);
  };

  /**
   * @struct DynamicStrandsSettings
   * @brief Contains settings for dynamic strand rendering and physics.
   */
  struct DynamicStrandsSettings {
    FoliageRenderParameters foliage_render_parameters{};             ///< Rendering parameters for foliage.
    SegmentPairsRenderParameters segment_pairs_render_parameters{};  ///< Rendering parameters for segment pairs.

    DynamicStrands::PhysicsParameters physics_parameters{};            ///< Physics simulation parameters.
    DynamicStrandsVisualizationParameters visualization_parameters{};  ///< Visualization settings.
    float drag_multiplier = 1.f;                                       ///< The multiplier for drag forces.
    float point_cut_thickness = 2.f;                                   ///< The thickness for point cuts.
    float fungus_injection_amount = 1.0f;                              ///< The amount of fungus injected.
    bool fungus_white_rot = true;                                      ///< Flag for white rot fungus type.
    bool fungus_brown_rot = false;                                     ///< Flag for brown rot fungus type.

    /**
     * @enum TransformMode
     * @brief Defines different transformation modes for strands.
     */
    enum class TransformMode { Disabled, Translate, Rotate };

    unsigned transform_mode = static_cast<unsigned>(TransformMode::Disabled);  ///< Current transformation mode.

    /**
     * @enum OperatorMode
     * @brief Defines different operator modes for strand manipulation.
     */
    enum class OperatorMode { Drag, Saw, LineCut, PointCut, FungusInjection };

    unsigned operator_mode = static_cast<unsigned>(OperatorMode::FungusInjection);  ///< Current operator mode.

    bool cut_bend_twist_bundle_only = false;  ///< Flag to restrict cutting to bend/twist bundles only.
    bool enable_visualization = true;         ///< Enables or disables strand visualization.
    bool enable_physics = true;               ///< Enables or disables physics simulation.
    bool enable_rendering = true;             ///< Enables or disables rendering of strands.

    int remaining_step = 0;  ///< Remaining simulation steps.
  };

  TreeVisualizationSettings tree_visualization_settings_;  ///< Settings for tree visualization.
  DynamicStrandsSettings dynamic_strands_settings_;        ///< Settings for dynamic strands.

  /**
   * @struct DynamicSkeletonSettings
   * @brief Contains settings for dynamic skeleton physics and visualization.
   */
  struct DynamicSkeletonSettings {
    bool enable_physics = true;  ///< Enables or disables physics for dynamic skeletons.

    bool enable_visualization = true;  ///< Enables or disables visualization of dynamic skeletons.
    DynamicSkeleton::PhysicsParameters physics_parameters{};  ///< Physics simulation parameters for dynamic skeletons.
    DynamicSkeleton::VisualizationParameters
        visualization_parameters{};  ///< Visualization settings for dynamic skeletons.

    /**
     * @brief Handles the inspection of dynamic skeleton settings in the editor.
     * @param editor_layer The editor layer instance.
     */
    void OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);
  };

  DynamicSkeletonSettings dynamic_skeleton_settings_;  ///< Settings for dynamic skeletons.

  bool auto_generate_mesh_after_editing_ = false;          ///< Automatically generates meshes after editing.
  bool auto_generate_skeletal_graph_every_frame_ = false;  ///< Automatically generates skeletal graphs every frame.
  bool auto_generate_strands_after_editing_ = false;       ///< Automatically generates strands after editing.
  bool auto_generate_strand_mesh_after_editing_ = false;   ///< Automatically generates strand meshes after editing.

  bool auto_update_strand_renderer_ = false;               ///< Periodically rebuild strand renderers.
  float auto_update_strand_renderer_interval_ = 5.0f;      ///< Seconds between strand renderer rebuilds.
  bool auto_update_strand_model_mesh_ = false;             ///< Periodically rebuild strand model meshes.
  float auto_update_strand_model_mesh_interval_ = 5.0f;    ///< Seconds between strand model mesh rebuilds.

  double next_strand_renderer_update_time_ = 0.0;
  double next_strand_model_mesh_update_time_ = 0.0;

  friend class ShootVisualizer;
  friend class RootVisualizer;
  friend class Tree;
  friend class DynamicTreeSkeleton;

  std::vector<int> shoot_versions_;       ///< Stores shoot versions for tracking changes.
  std::vector<glm::vec3> random_colors_;  ///< Stores random colors for visualization.

  std::vector<glm::uint> shoot_stem_segments_;  ///< Stores shoot stem segment indices.
  std::vector<StrandPoint> shoot_stem_points_;  ///< Stores shoot stem point data.

  AssetRef shoot_stem_strands_;  ///< Reference to shoot stem strands.

  std::shared_ptr<ParticleInfoList> soil_matrices_;

  std::shared_ptr<ParticleInfoList> bounding_box_matrices_;  ///< Stores bounding box particle matrices.
  std::shared_ptr<ParticleInfoList> foliage_matrices_;       ///< Stores foliage particle matrices.
  std::shared_ptr<ParticleInfoList> flower_matrices_;        ///< Stores flower particle matrices.

  std::shared_ptr<ParticleInfoList> fruit_matrices_;  ///< Stores fruit particle matrices.

  std::shared_ptr<ParticleInfoList> ground_flower_matrices_;  ///< Stores ground fruit particle matrices.

  std::shared_ptr<ParticleInfoList> ground_fruit_matrices_;  ///< Stores ground fruit particle matrices.
  std::shared_ptr<ParticleInfoList> ground_leaf_matrices_;   ///< Stores ground leaf particle matrices.

  /**
   * @struct SoilVisualizationSettings
   * @brief Contains settings for visualizing soil properties.
   */
  struct SoilVisualizationSettings {
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

    std::vector<glm::vec4> soil_layer_colors;  ///< Colors representing the soil layers.

    /**
     * @brief Constructor for SoilVisualizationSettings.
     */
    SoilVisualizationSettings();

    /**
     * @brief Handles the inspection of soil visualization settings in the editor.
     * @param editor_layer The editor layer instance.
     * @return True if the asset content has not been modified.
     */
    bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);
  };

  SoilVisualizationSettings soil_visualization_settings_{};  ///< Settings for soil visualization.

  bool need_flow_update_for_selection_ = false;  ///< Flag indicating if flow updates are needed for selection.
  int last_selected_tree_index_ = -1;            ///< Stores the index of the last selected tree.

  int soil_version_ = -1;  ///< Tracks soil version changes.

  std::shared_ptr<ParticleInfoList> vector_matrices_;  ///< Stores matrices for vector soil visualization.

  friend class Soil;

  std::shared_ptr<ParticleInfoList> scalar_matrices_;  ///< Stores matrices for scalar soil visualization.

  std::shared_ptr<ParticleInfoList> shadow_grid_particle_info_list_;    ///< Stores data for shadow grid rendering.
  std::shared_ptr<ParticleInfoList> lighting_grid_particle_info_list_;  ///< Stores data for lighting grid rendering.

  float simulated_time_;         ///< The current simulated time.
  bool auto_time_grow_ = false;  ///< Whether auto-growth mode is currently active.
  float auto_time_target_ = 0.f; ///< Target simulation time for auto-growth.
  float extra_time_years_ = 4.f; ///< UI value for grow duration in years.

  std::vector<Fruit> fruits_;    ///< Stores fruit entities.
  std::vector<Leaf> leaves_;     ///< Stores leaf entities.
  std::vector<Flower> flowers_;  ///< Stores leaf entities.

  std::shared_ptr<Camera> visualization_camera_;  ///< Camera used for visualization.

  bool visualization_camera_window_focused_ = false;  ///< Flag indicating if the visualization window is focused.

  /**
   * @brief Updates the EcoSysLab layer.
   */
  void Update() override;

  /**
   * @brief Performs late update operations for the EcoSysLab layer.
   */
  void LateUpdate() override;

  /**
   * @brief Called upon creating the EcoSysLab layer.
   */
  void OnCreate() override;

  /**
   * @brief Visualizes trees within the editor.
   * @param editor_layer The editor layer instance.
   */
  void TreeVisualization(const std::shared_ptr<EditorLayer>& editor_layer);

  /**
   * @brief Handles visualization of dynamic strands.
   * @param editor_layer The editor layer instance.
   */
  void DynamicStrandsVisualization(const std::shared_ptr<EditorLayer>& editor_layer);

  /**
   * @brief Handles the inspection of the EcoSysLab layer in the editor.
   * @param editor_layer The editor layer instance.
   */
  void OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;

  /**
   * @brief Serializes EcoSysLab editor/runtime UI state.
   * @param out YAML emitter.
   */
  void Serialize(YAML::Emitter& out) const override;

  /**
   * @brief Deserializes EcoSysLab editor/runtime UI state.
   * @param in YAML node.
   */
  void Deserialize(const YAML::Node& in) override;

  /**
   * @brief Handles the inspection of dynamic strand settings in the editor.
   * @param editor_layer The editor layer instance.
   */
  void OnInspectDynamicStrandsSettings(const std::shared_ptr<EditorLayer>& editor_layer);

  /**
   * @brief Updates fluid flow simulations for the given tree entities.
   * @param tree_entities The list of tree entities.
   * @param branch_strands The strands associated with tree branches.
   */
  void UpdateFlows(const std::vector<Entity>* tree_entities, const std::shared_ptr<Strands>& branch_strands);

  /**
   * @brief Clears rendered representations of ground fruit and leaves.
   */
  void ClearGroundFruitAndLeaf();

  /**
   * @brief Updates the visualization of ground fruit and leaves.
   */
  void UpdateGroundFruitAndLeaves() const;

  /**
   * @brief Handles soil visualization.
   */
  void SoilVisualization();

  /**
   * @brief Performs scalar-based soil visualization.
   * @param soil_model The voxel-based soil model.
   */
  void SoilVisualizationScalar(const VoxelSoilModel& soil_model);

  /**
   * @brief Performs vector-based soil visualization.
   * @param soil_model The voxel-based soil model.
   */
  void SoilVisualizationVector(const VoxelSoilModel& soil_model);

  /**
   * @brief Registers the procedure for rendering strand models.
   */
  void RegisterStrandRenderingProcedure() const;

  /**
   * @brief Simulates the dynamics of strands using physics models.
   */
  void DynamicStrandSimulation();

  /**
   * @brief Simulates the physics of dynamic skeleton structures.
   */
  void DynamicSkeletonPhysics() const;

  /**
   * @brief Handles visualization of dynamic skeleton structures.
   */
  void DynamicSkeletonVisualization() const;

  /**
   * @brief Handles visualization rendering of dynamic strands.
   */
  void DynamicStrandVisualization() const;
};
}  // namespace eco_sys_lab_plugin