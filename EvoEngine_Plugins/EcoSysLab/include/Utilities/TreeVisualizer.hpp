
#pragma once

#include "Application.hpp"
#include "EditorLayer.hpp"
#include "Jobs.hpp"
#include "Platform.hpp"
#include "StrandModel.hpp"
#include "TreeModel.hpp"
using namespace evo_engine;

namespace eco_sys_lab_plugin {

/**
 * @brief Enumeration for different shoot visualization modes.
 */
enum class ShootVisualizerMode {
  Default,                      ///< Default visualization mode.
  Order,                        ///< Visualize by shoot order.
  Level,                        ///< Visualize by hierarchical level.
  MaxDescendantLightIntensity,  ///< Visualize based on maximum descendant light intensity.
  LightIntensity,               ///< Visualize based on light intensity.
  LightDirection,               ///< Visualize based on light direction.
  DesiredGrowthRate,            ///< Visualize based on desired growth rate.
  GrowthPotential,              ///< Visualize based on growth potential.
  GrowthRate,                   ///< Visualize based on growth rate.
  IsMaxChild,                   ///< Visualize based on max child node property.
  AllocatedVigor,               ///< Visualize based on allocated vigor.
  SaggingStress,                ///< Visualize based on sagging stress.
  Locked                        ///< Locked visualization mode.
};

/**
 * @brief Enumeration for different root visualization modes.
 */
enum class RootVisualizerMode {
  Default,         ///< Default visualization mode.
  AllocatedVigor,  ///< Visualize based on allocated vigor.
};

/**
 * @brief Structure to hold tree visualizer color settings.
 */
struct TreeVisualizerColorSettings {
  int shoot_visualization_mode =
      static_cast<int>(ShootVisualizerMode::Default);  ///< The active shoot visualization mode.
  float shoot_color_multiplier = 1.0f;                 ///< Multiplier for shoot color intensity.
};

/**
 * @brief Class for visualizing tree structures and internodes.
 */
class TreeVisualizer {
  bool initialized_ = false;  ///< Flag to check if the visualizer is initialized.

  std::vector<glm::vec4> random_colors_;  ///< Stores generated random colors.

  std::shared_ptr<ParticleInfoList> internode_matrices_;  ///< Stores internode transformation matrices.

  /**
   * @brief Draws the GUI for inspecting an internode.
   * @param tree_model Reference to the tree model.
   * @param internode_handle Handle of the internode to inspect.
   * @param deleted Output flag indicating if the internode is deleted.
   * @param hierarchy_level The hierarchy level of the internode.
   * @return True if the inspection was successful, otherwise false.
   */
  bool DrawInternodeInspectionGui(TreeModel& tree_model, SkeletonNodeHandle internode_handle, bool& deleted,
                                  const unsigned& hierarchy_level);

  /**
   * @brief Displays GUI elements for inspecting a tree node.
   * @param skeleton Reference to the shoot skeleton.
   * @param node_handle Handle of the node to inspect.
   * @param hierarchy_level The hierarchy level of the node.
   */
  void PeekNodeInspectionGui(const ShootSkeleton& skeleton, SkeletonNodeHandle node_handle,
                             const unsigned& hierarchy_level);

  /**
   * @brief Peeks at an internode without modifying it.
   * @param shoot_skeleton Reference to the shoot skeleton.
   * @param internode_handle Handle of the internode.
   */
  void PeekInternode(const ShootSkeleton& shoot_skeleton, SkeletonNodeHandle internode_handle) const;

  /**
   * @brief Inspects a specific internode.
   * @param shoot_skeleton Reference to the shoot skeleton.
   * @param internode_handle Handle of the internode to inspect.
   * @return True if inspection is successful, otherwise false.
   */
  bool InspectInternode(ShootSkeleton& shoot_skeleton, SkeletonNodeHandle internode_handle);

 public:
  /**
   * @brief Performs a ray-casting selection for an internode.
   * @param camera_component Shared pointer to the camera component.
   * @param mouse_position The mouse position in screen coordinates.
   * @param skeleton The shoot skeleton.
   * @param global_transform The global transformation matrix.
   * @return True if an internode is selected, otherwise false.
   */
  bool RayCastSelection(const std::shared_ptr<Camera>& camera_component, const glm::vec2& mouse_position,
                        const ShootSkeleton& skeleton, const GlobalTransform& global_transform);

  /**
   * @brief Handles selection of internodes along a screen-drawn curve.
   * @param handler Callback function executed on selected nodes.
   * @param mouse_positions List of mouse positions forming a curve.
   * @param skeleton The shoot skeleton.
   * @param global_transform The global transformation matrix.
   * @return True if selection is successful, otherwise false.
   */
  bool ScreenCurveSelection(const std::function<void(SkeletonNodeHandle)>& handler,
                            std::vector<glm::vec2>& mouse_positions, ShootSkeleton& skeleton,
                            const GlobalTransform& global_transform);

  std::vector<SkeletonNodeHandle> selected_internode_hierarchy_list;  ///< List of selected internode hierarchy nodes.
  SkeletonNodeHandle selected_internode_handle = -1;                  ///< Handle of the selected internode.
  bool visualization = true;                                          ///< Flag to enable or disable visualization.
  TreeVisualizerColorSettings tree_visualizer_color_settings;         ///< Settings for visualization color.
  float line_thickness = 0.f;                                         ///< Thickness of visualized lines.
  bool profile_gui = true;                                            ///< Flag to toggle profile GUI.
  bool tree_hierarchy_gui = false;                                    ///< Flag to toggle tree hierarchy GUI.
  float selected_internode_length_factor = 0.0f;                      ///< Length factor for the selected internode.
  int checkpoint_iteration = 0;                                       ///< Iteration count for checkpoints.
  bool need_update = false;                                           ///< Flag indicating if an update is needed.

  /**
   * @brief Checks if the visualizer is initialized.
   * @return True if initialized, otherwise false.
   */
  [[nodiscard]] bool Initialized() const;

  /**
   * @brief Clears all selections in the visualizer.
   */
  void ClearSelections();

  /**
   * @brief Initializes the tree visualizer.
   */
  void Initialize();

  /**
   * @brief Sets the selected node in the shoot skeleton.
   * @param skeleton Reference to the shoot skeleton.
   * @param node_handle Handle of the node to select.
   */
  void SetSelectedNode(const ShootSkeleton& skeleton, SkeletonNodeHandle node_handle);

  /**
   * @brief Synchronizes transformation matrices between skeleton and internode list.
   * @param skeleton Reference to the shoot skeleton.
   * @param particle_info_list Shared pointer to the list of particles.
   * @param selected_node_handle Handle of the selected internode.
   */
  void SyncMatrices(const ShootSkeleton& skeleton, const std::shared_ptr<ParticleInfoList>& particle_info_list,
                    SkeletonNodeHandle selected_node_handle);

  /**
   * @brief Handles inspection of the tree model.
   * @param tree_model Reference to the tree model.
   * @return True if contents remain unmodified, otherwise false.
   */
  bool OnInspect(TreeModel& tree_model);

  /**
   * @brief Visualizes the given tree model.
   * @param tree_model The tree model to visualize.
   * @param global_transform The global transformation matrix.
   */
  void Visualize(const TreeModel& tree_model, const GlobalTransform& global_transform);

  /**
   * @brief Visualizes the given strand model.
   * @param strand_model The strand model to visualize.
   */
  void Visualize(StrandModel& strand_model);

  /**
   * @brief Resets the visualization of a tree model.
   * @param tree_model The tree model to reset.
   */
  void Reset(const TreeModel& tree_model);

  /**
   * @brief Clears all visualization and data.
   */
  void Clear();
};

}  // namespace eco_sys_lab_plugin
