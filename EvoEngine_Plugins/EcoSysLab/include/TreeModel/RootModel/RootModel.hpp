#pragma once
#include "PlantModel.hpp"
#include "VoxelSoilModel.hpp"

#include "Octree.hpp"
#include "TreeControllers.hpp"
#include "TreeGrowthSettings.hpp"

namespace eco_sys_lab_plugin {
using namespace evo_engine;

class RootModel : public PlantModel {
  /**
   * @brief Calculates the thickness of root nodes in the tree.
   * @param root_growth_controller The controller that regulates root growth.
   */
  void CalculateThickness(const RootGrowthController& root_growth_controller);

  /**
   * @brief Performs pre-processing computations after root growth.
   * @param root_growth_controller The controller that regulates root growth.
   */
  void CalculateGrowthData(const RootGrowthController& root_growth_controller);
  friend class Tree;

  RootSkeleton root_skeleton_;             ///< The skeletal structure representing the root.
  std::deque<RootSkeleton> root_history_;  ///< History of previous root skeleton states.

 public:
  /// Records a topology addition (Extend) during a Grow() step.
  struct GrowthEvent {
    SkeletonNodeHandle parent_handle;  ///< The parent node that was extended.
    SkeletonNodeHandle new_handle;     ///< The newly created node handle.
    bool branching;                    ///< True if this was a branching event, false for prolongation.
  };

 private:
  /// Records topology additions during a single Grow() step.
  std::vector<GrowthEvent> growth_events_;

  /// Ordered pruning batches removed during a single Grow() step.
  /// Each batch is captured immediately before a RemoveNodes() call.
  std::vector<std::vector<SkeletonNodeHandle>> pruning_event_batches_;

  /// True if PruneRootNodes removed any nodes during the last Grow() call.
  /// When true, growth_events_ are stale (handles invalidated by swap-and-pop removal).
  bool pruning_occurred_ = false;

 public:
  /// Returns the growth events recorded during the last Grow() call.
  [[nodiscard]] const std::vector<GrowthEvent>& PeekGrowthEvents() const { return growth_events_; }

  /// Returns ordered pruning batches removed during the last Grow() call.
  [[nodiscard]] const std::vector<std::vector<SkeletonNodeHandle>>& PeekPruningEventBatches() const {
    return pruning_event_batches_;
  }

  /// Returns true if pruning occurred during the last Grow() call.
  /// When true, growth events are invalidated and must not be used for incremental replay.
  [[nodiscard]] bool PruningOccurred() const { return pruning_occurred_; }
  float shoot_skeleton_base_thickness = 0.0f;

  TreeGrowthSettings tree_growth_settings;  ///< Growth settings used for simulation.
  /**
   * @brief Initializes the tree model with the given root growth controller.
   * @param root_growth_controller The controller that regulates root growth.
   */
  void Initialize(const RootGrowthController& root_growth_controller);

  /**
   * @brief Computes the root flux based on environmental parameters.
   * @param global_transform The global transformation of the tree.
   * @param soil_model The soil influencing the tree.
   * @param root_growth_controller The procedural growth parameters.
   */
  [[nodiscard]] Vigor SampleRootFlux(const glm::mat4& global_transform, const VoxelSoilModel& soil_model,
                                     const RootGrowthController& root_growth_controller);

  /**
   * @brief Computes the transformation of all internodes in the tree.
   * @param root_growth_controller The procedural growth parameters.
   */
  void CalculateTransform(const RootGrowthController& root_growth_controller);

  void DistributeVigor(const RootGrowthController& root_growth_controller, Vigor vigor);

  void CalculateSourceSinkStrength(const RootGrowthController& root_growth_controller,
                                   const ClimateModel& climate_model, float delta_time);

  /**
   * @brief Provides direct access to the root skeleton for modifications.
   * @return A reference to the root skeleton.
   */
  [[nodiscard]] RootSkeleton& RefRootSkeleton();

  /**
   * @brief Retrieves a previous state of the shoot skeleton.
   * @param iteration The iteration index (-1 for the latest state).
   * @return A constant reference to the requested shoot skeleton.
   */
  [[nodiscard]] const RootSkeleton& PeekRootSkeleton(int iteration = -1) const;
  /**
   * @brief Erases all data related to the tree.
   */
  void Clear();
  /**
   * @brief Clears stored growth history.
   */
  void ClearHistory();

  /**
   * @brief Advances the tree growth by one time step.
   */
  void Step();

  /**
   * @brief Removes the most recent growth state from history.
   */
  void Pop();

  /**
   * @brief Returns the current growth iteration count.
   * @return The iteration count.
   */
  [[nodiscard]] int CurrentIteration() const;

  /**
   * @brief Reverts the tree to a previous iteration.
   * @param iteration The iteration index to revert to.
   */
  void Reverse(int iteration);

  /**
   * @brief Simulates one growth iteration for the entire tree.
   * @param delta_time Time step for the iteration.
   * @param global_transform The global transformation matrix of the tree.
   * @param climate_model The climate influencing the tree.
   * @param soil_model The soil influencing the tree.
   * @param root_growth_controller Procedural parameters for branch growth.
   * @param fine_root_controller Procedural parameters for foliage growth.
   * @param reproduction_controller Procedural parameters for reproduction units.
   * @param root_pruning_controller Procedural parameters for branch pruning.
   * @param pruning Whether pruning should be applied automatically.
   * @return Whether structural changes occurred during growth.
   */
  bool Grow(float delta_time, const glm::mat4& global_transform, const ClimateModel& climate_model,
            const VoxelSoilModel& soil_model, const RootGrowthController& root_growth_controller,
            const FineRootController& fine_root_controller, const RootReproductionController& reproduction_controller,
            const RootPruningController& root_pruning_controller, bool pruning = true);

  /**
   * @brief Grows a single root node under climate and growth constraints.
   * @param node_handle Handle to the root node being grown.
   * @param root_growth_controller The controller that regulates root growth.
   * @param fine_root_controller The controller that regulates fine root growth.
   * @param reproduction_controller The controller that control growth of fruits.
   * @return Whether the root node grew successfully.
   */
  bool GrowRootNode(SkeletonNodeHandle node_handle, const RootGrowthController& root_growth_controller,
                    const FineRootController& fine_root_controller,
                    const RootReproductionController& reproduction_controller);
  /**
   * @brief Extends the length of an internode during growth.
   * @param extended_length The desired additional length.
   * @param internode_handle Handle to the internode being elongated.
   * @param root_growth_controller The controller that regulates root growth.
   * @param fine_root_controller The controller that regulates fine root growth.
   * @param reproduction_controller The controller for flowers and fruits.
   * @param collected_inhibitor The collected growth inhibitor value.
   * @return Whether the internode elongated successfully.
   */
  bool ElongateRootNode(float extended_length, SkeletonNodeHandle internode_handle,
                        const RootGrowthController& root_growth_controller,
                        const FineRootController& fine_root_controller,
                        const RootReproductionController& reproduction_controller, float& collected_inhibitor);
  /**
   * @brief Prunes root node that do not contribute positively to the tree's growth.
   * @param global_transform The global transformation matrix of the tree.
   * @param climate_model The model representing environmental conditions.
   * @param soil_model The model representing environmental conditions.
   * @param root_growth_controller The controller that regulates root growth.
   * @param root_pruning_controller The controller that regulates root pruning.
   * @return Whether any root nodes were pruned.
   */
  bool PruneRootNodes(const glm::mat4& global_transform, const ClimateModel& climate_model,
                      const VoxelSoilModel& soil_model, const RootGrowthController& root_growth_controller,
                      const RootPruningController& root_pruning_controller);
};
}  // namespace eco_sys_lab_plugin