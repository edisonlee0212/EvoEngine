
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
  int m_shootVisualizationMode =
      static_cast<int>(ShootVisualizerMode::Default);  ///< The active shoot visualization mode.
  float m_shootColorMultiplier = 1.0f;                 ///< Multiplier for shoot color intensity.
};

/**
 * @brief Class for visualizing tree structures and internodes.
 */
class TreeVisualizer {
  bool m_initialized = false;  ///< Flag to check if the visualizer is initialized.

  std::vector<glm::vec4> m_randomColors;  ///< Stores generated random colors.

  std::shared_ptr<ParticleInfoList> m_internodeMatrices;  ///< Stores internode transformation matrices.

  /**
   * @brief Draws the GUI for inspecting an internode.
   * @param treeModel Reference to the tree model.
   * @param internodeHandle Handle of the internode to inspect.
   * @param deleted Output flag indicating if the internode is deleted.
   * @param hierarchyLevel The hierarchy level of the internode.
   * @return True if the inspection was successful, otherwise false.
   */
  bool DrawInternodeInspectionGui(TreeModel& treeModel, SkeletonNodeHandle internodeHandle, bool& deleted,
                                  const unsigned& hierarchyLevel);

  /**
   * @brief Displays GUI elements for inspecting a tree node.
   * @param skeleton Reference to the shoot skeleton.
   * @param nodeHandle Handle of the node to inspect.
   * @param hierarchyLevel The hierarchy level of the node.
   */
  void PeekNodeInspectionGui(const ShootSkeleton& skeleton, SkeletonNodeHandle nodeHandle,
                             const unsigned& hierarchyLevel);

  /**
   * @brief Peeks at an internode without modifying it.
   * @param shootSkeleton Reference to the shoot skeleton.
   * @param internodeHandle Handle of the internode.
   */
  void PeekInternode(const ShootSkeleton& shootSkeleton, SkeletonNodeHandle internodeHandle) const;

  /**
   * @brief Inspects a specific internode.
   * @param shootSkeleton Reference to the shoot skeleton.
   * @param internodeHandle Handle of the internode to inspect.
   * @return True if inspection is successful, otherwise false.
   */
  bool InspectInternode(ShootSkeleton& shootSkeleton, SkeletonNodeHandle internodeHandle);

 public:
  /**
   * @brief Performs a ray-casting selection for an internode.
   * @param cameraComponent Shared pointer to the camera component.
   * @param mousePosition The mouse position in screen coordinates.
   * @param skeleton The shoot skeleton.
   * @param globalTransform The global transformation matrix.
   * @return True if an internode is selected, otherwise false.
   */
  bool RayCastSelection(const std::shared_ptr<Camera>& cameraComponent, const glm::vec2& mousePosition,
                        const ShootSkeleton& skeleton, const GlobalTransform& globalTransform);

  /**
   * @brief Handles selection of internodes along a screen-drawn curve.
   * @param handler Callback function executed on selected nodes.
   * @param mousePositions List of mouse positions forming a curve.
   * @param skeleton The shoot skeleton.
   * @param globalTransform The global transformation matrix.
   * @return True if selection is successful, otherwise false.
   */
  bool ScreenCurveSelection(const std::function<void(SkeletonNodeHandle)>& handler,
                            std::vector<glm::vec2>& mousePositions, ShootSkeleton& skeleton,
                            const GlobalTransform& globalTransform);

  std::vector<SkeletonNodeHandle> m_selectedInternodeHierarchyList;  ///< List of selected internode hierarchy nodes.
  SkeletonNodeHandle m_selectedInternodeHandle = -1;                 ///< Handle of the selected internode.
  bool m_visualization = true;                                       ///< Flag to enable or disable visualization.
  TreeVisualizerColorSettings m_settings;                            ///< Settings for visualization color.
  float m_lineThickness = 0.f;                                       ///< Thickness of visualized lines.
  bool m_profileGui = true;                                          ///< Flag to toggle profile GUI.
  bool m_treeHierarchyGui = false;                                   ///< Flag to toggle tree hierarchy GUI.
  float m_selectedInternodeLengthFactor = 0.0f;                      ///< Length factor for the selected internode.
  int m_checkpointIteration = 0;                                     ///< Iteration count for checkpoints.
  bool m_needUpdate = false;                                         ///< Flag indicating if an update is needed.

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
   * @param nodeHandle Handle of the node to select.
   */
  void SetSelectedNode(const ShootSkeleton& skeleton, SkeletonNodeHandle nodeHandle);

  /**
   * @brief Synchronizes transformation matrices between skeleton and internode list.
   * @param skeleton Reference to the shoot skeleton.
   * @param particleInfoList Shared pointer to the list of particles.
   * @param selectedNodeHandle Handle of the selected internode.
   */
  void SyncMatrices(const ShootSkeleton& skeleton, const std::shared_ptr<ParticleInfoList>& particleInfoList,
                    SkeletonNodeHandle selectedNodeHandle);

  /**
   * @brief Handles inspection of the tree model.
   * @param treeModel Reference to the tree model.
   * @return True if contents remain unmodified, otherwise false.
   */
  bool OnInspect(TreeModel& treeModel);

  /**
   * @brief Visualizes the given tree model.
   * @param treeModel The tree model to visualize.
   * @param globalTransform The global transformation matrix.
   */
  void Visualize(const TreeModel& treeModel, const GlobalTransform& globalTransform);

  /**
   * @brief Visualizes the given strand model.
   * @param strandModel The strand model to visualize.
   */
  void Visualize(StrandModel& strandModel);

  /**
   * @brief Resets the visualization of a tree model.
   * @param treeModel The tree model to reset.
   */
  void Reset(TreeModel& treeModel);

  /**
   * @brief Clears all visualization and data.
   */
  void Clear();
};

}  // namespace eco_sys_lab_plugin
