
#pragma once
// #include "VoxelSoilModel.hpp"
#include "ClimateModel.hpp"
#include "Octree.hpp"
#include "TreeControllers.hpp"
using namespace evo_engine;

namespace eco_sys_lab_plugin {

/**
 * @brief Settings related to tree growth algorithms.
 */
struct TreeGrowthSettings {
  float node_developmental_vigor_filling_rate = 1.0f;  ///< The rate at which developmental vigor is filled.
  bool use_space_colonization = false;                 ///< Whether to use space colonization for branching.
  bool space_colonization_auto_resize = false;         ///< Whether the space colonization method resizes automatically.
  float space_colonization_removal_distance_factor = 2;    ///< Distance factor for removing colonization nodes.
  float space_colonization_detection_distance_factor = 4;  ///< Distance factor for detecting colonization nodes.
  float space_colonization_theta = 90.0f;                  ///< The angle parameter used by space colonization.

  /**
   * @brief Inspects pruning settings in an editor.
   * @param editor_layer The editor layer managing inspection.
   * @return True if data was not modified during inspection.
   */
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);

  /**
   * @brief Saves pruning settings to a YAML emitter.
   * @param name The name of the settings entry.
   * @param out The YAML emitter to serialize data into.
   */
  void Save(const std::string& name, YAML::Emitter& out) const;

  /**
   * @brief Loads pruning settings from a YAML node.
   * @param name The name of the settings entry.
   * @param in The YAML node containing serialized data.
   */
  void Load(const std::string& name, const YAML::Node& in);
};

/**
 * @brief Represents the procedural structure and behavior of a tree model.
 */
class ShootModel {
#pragma region Tree Growth
  /**
   * @brief Collects the shoot flux of the tree based on its internode list.
   * @return The computed shoot flux.
   */
  ShootFlux CollectShootFlux();

  /**
   * @brief Calculates the growth rate for the tree.
   * @param sorted_internode_list A sorted list of internode handles.
   * @param factor Growth factor applied to the calculation.
   */
  void CalculateGrowthRate(const std::vector<SkeletonNodeHandle>& sorted_internode_list, float factor);

  /**
   * @brief Computes the growth potential based on current growth conditions.
   * @param shoot_growth_controller The controller that regulates shoot growth.
   * @return The computed growth potential.
   */
  float CalculateGrowthPotential(const ShootGrowthController& shoot_growth_controller);

  /**
   * @brief Prunes internodes that do not contribute positively to the tree's growth.
   * @param global_transform The global transformation matrix of the tree.
   * @param climate_model The model representing environmental conditions.
   * @param shoot_growth_controller The controller that regulates shoot growth.
   * @param shoot_pruning_controller The controller that regulates shoot pruning.
   * @return Whether any internodes were pruned.
   */
  bool PruneInternodes(const glm::mat4& global_transform, ClimateModel& climate_model,
                       const ShootGrowthController& shoot_growth_controller,
                       const ShootPruningController& shoot_pruning_controller);

  /**
   * @brief Calculates the thickness of internodes in the tree.
   * @param shoot_growth_controller The controller that regulates shoot growth.
   */
  void CalculateThickness(const ShootGrowthController& shoot_growth_controller);

  /**
   * @brief Computes the biomass of a given internode.
   * @param internode_handle Handle to the internode being computed.
   * @param shoot_growth_controller The controller that regulates shoot growth.
   */
  void CalculateBiomass(SkeletonNodeHandle internode_handle, const ShootGrowthController& shoot_growth_controller);

  /**
   * @brief Calculates the sagging stress on the tree's internodes.
   * @param internode_handle Handle to the internode affected by sagging stress.
   * @param shoot_pruning_controller The controller that regulates shoot growth.
   */
  void CalculateSaggingStress(SkeletonNodeHandle internode_handle,
                              const ShootPruningController& shoot_pruning_controller);

  /**
   * @brief Computes the hierarchical level of each internode in the tree.
   */
  void CalculateLevel();

  /**
   * @brief Grows a single internode under climate and growth constraints.
   * @param climate_model The model representing environmental conditions.
   * @param internode_handle Handle to the internode being grown.
   * @param shoot_growth_controller The controller that regulates shoot growth.
   * @param foliage_controller The controller that regulates foliage growth.
   * @param reproduction_controller The controller that control growth of flowers and fruits.
   * @return Whether the internode grew successfully.
   */
  bool GrowInternode(ClimateModel& climate_model, SkeletonNodeHandle internode_handle,
                     const ShootGrowthController& shoot_growth_controller, const FoliageController& foliage_controller,
                     const ReproductionController& reproduction_controller);

  /**
   * @brief Grows reproductive modules such as flowers and fruits.
   * @param delta_time Time elapsed this iteration.
   * @param climate_model The model representing environmental conditions.
   * @param global_transform The global transform of tree.
   * @param internode_handle Handle to the internode where reproductive modules grow.
   * @param foliage_controller The controller that regulates foliage growth.
   * @return Whether the reproductive modules were successfully grown.
   */
  bool GrowFoliage(float delta_time, ClimateModel& climate_model, const glm::mat4& global_transform,
                   SkeletonNodeHandle internode_handle, const FoliageController& foliage_controller);

  void FormulateFoliage(const ClimateModel& climate_model, const glm::mat4& global_transform,
                        SkeletonNodeHandle internode_handle, const FoliageController& foliage_controller);

  /**
   * @brief Grows reproductive modules such as flowers and fruits.
   * @param delta_time Time elapsed this iteration.
   * @param climate_model The model representing environmental conditions.
   * @param global_transform The global transform of tree.
   * @param internode_handle Handle to the internode where reproductive modules grow.
   * @param reproduction_controller The controller that controls reproduction units.
   * @return Whether the reproductive modules were successfully grown.
   */
  bool GrowReproductiveModules(float delta_time, ClimateModel& climate_model, const glm::mat4& global_transform,
                               SkeletonNodeHandle internode_handle,
                               const ReproductionController& reproduction_controller);
  void FormulateReproductiveModules(const ClimateModel& climate_model, const glm::mat4& global_transform,
                                    SkeletonNodeHandle internode_handle,
                                    const ReproductionController& reproduction_controller);

  /**
   * @brief Extends the length of an internode during growth.
   * @param extended_length The desired additional length.
   * @param internode_handle Handle to the internode being elongated.
   * @param shoot_growth_controller The controller that regulates shoot growth.
   * @param foliage_controller The controller that regulates foliage growth.
   * @param reproduction_controller The controller for flowers and fruits.
   * @param collected_inhibitor The collected growth inhibitor value.
   * @return Whether the internode elongated successfully.
   */
  bool ElongateInternode(float extended_length, SkeletonNodeHandle internode_handle,
                         const ShootGrowthController& shoot_growth_controller,
                         const FoliageController& foliage_controller,
                         const ReproductionController& reproduction_controller, float& collected_inhibitor);

  /**
   * @brief Performs pre-processing computations after shoot growth.
   * @param shoot_growth_controller The controller that regulates shoot growth.
   */
  void CalculateGrowthData(const ShootGrowthController& shoot_growth_controller);

  friend class Tree;
#pragma endregion

  bool initialized_ = false;  ///< Tracks whether the model has been initialized.

  ShootSkeleton shoot_skeleton_;             ///< The skeletal structure representing the shoot.
  std::deque<ShootSkeleton> shoot_history_;  ///< History of previous shoot skeleton states.

  int age_in_year_ = 0;              ///< Integer representation of the tree's age in years.
  float current_delta_time_ = 1.0f;  ///< Time step used for growth calculations.

  /**
   * @brief Resets the reproductive modules in the tree.
   */
  void ResetOrgans();

  void CreateOrgansForInternode(SkeletonNode<InternodeGrowthData>& internode,
                                const FoliageController& foliage_controller,
                                const ReproductionController& reproduction_controller);

  std::mt19937 random_engine_;  ///< Random number generator engine.

  int iteration_ = 0;  ///< The current growth iteration of the tree.

  uint32_t bud_count = 0;
  uint32_t leaf_count_ = 0;
  uint32_t flower_count_ = 0;
  uint32_t fruit_count_ = 0;

 public:
  void RemoveNodes(const std::vector<SkeletonNodeHandle>& pruning_node_handles);

  /**
   * @brief Initializes the tree model with the given shoot growth controller.
   * @param shoot_growth_controller The controller that regulates shoot growth.
   * @param foliage_controller The controller that control foliage generation and growth.
   * @param reproduction_controller The controller that control growth of flower and fruit.
   */
  void Initialize(const ShootGrowthController& shoot_growth_controller, const FoliageController& foliage_controller,
                  const ReproductionController& reproduction_controller);

  /**
   * @brief Initializes the tree model by cloning data from an existing skeleton.
   * @param src_skeleton The source skeleton used for initialization.
   */
  template <typename SrcSkeletonData, typename SrcFlowData, typename SrcNodeData>
  void Initialize(const Skeleton<SrcSkeletonData, SrcFlowData, SrcNodeData>& src_skeleton);

  /**
   * @brief Retrieves the maximum age of a subtree starting from the given internode.
   * @param base_internode_handle The handle to the internode serving as the root of the subtree.
   * @return The maximum age found within the subtree.
   */
  float GetSubTreeMaxAge(SkeletonNodeHandle base_internode_handle) const;

  /**
   * @brief Reduces the tree structure by removing older internodes.
   * @param shoot_growth_controller The controller that regulates shoot growth.
   * @param base_internode_handle The base internode from which reduction begins.
   * @param target_age The target age threshold for reduction.
   * @return Whether reduction was successful.
   */
  bool Reduce(const ShootGrowthController& shoot_growth_controller, SkeletonNodeHandle base_internode_handle,
              float target_age);

  /**
   * @brief Computes the transformation of all internodes in the tree.
   * @param shoot_growth_controller The procedural growth parameters.
   * @param sagging Whether to apply sagging effects in the transformation.
   */
  void CalculateTransform(const ShootGrowthController& shoot_growth_controller, bool sagging);

  int seed = 0;  ///< The seed value for random number generation.

  /**
   * @brief Registers the tree within a voxel-based occupancy grid.
   * @param global_transform The global transformation of the tree.
   * @param climate_model The climate model affecting growth.
   */
  void RegisterVoxel(const glm::mat4& global_transform, ClimateModel& climate_model);

  TreeOccupancyGrid tree_occupancy_grid{};  ///< The tree's occupancy grid.

  /**
   * @brief Computes the shoot flux based on environmental parameters.
   * @param global_transform The global transformation of the tree.
   * @param climate_model The climate influencing the tree.
   * @param shoot_growth_controller The procedural growth parameters.
   */
  void CalculateShootFlux(const glm::mat4& global_transform, const ClimateModel& climate_model,
                          const ShootGrowthController& shoot_growth_controller);

  /**
   * @brief Harvests fruits based on a user-defined selection function.
   * @param harvest_function A function that returns true if the fruit should be harvested.
   */
  void HarvestFruits(const std::function<bool(const ShootOrgan& fruit)>& harvest_function);

  /**
   * @brief Applies tropism effects to a directional vector.
   * @param target_dir The target direction vector.
   * @param tropism The strength of the tropism effect.
   * @param front The front-facing direction to be adjusted.
   * @param up The up direction to be maintained.
   */
  static void ApplyTropism(const glm::vec3& target_dir, float tropism, glm::vec3& front, glm::vec3& up);

  /**
   * @brief Applies tropism effects to a rotation quaternion.
   * @param target_dir The target direction vector.
   * @param tropism The strength of the tropism effect.
   * @param rotation The quaternion rotation to be modified.
   */
  static void ApplyTropism(const glm::vec3& target_dir, float tropism, glm::quat& rotation);

  std::vector<int> internode_order_counts;  ///< Order count of internodes.

  TreeGrowthSettings tree_growth_settings;  ///< Growth settings used for simulation.

  /**
   * @brief Erases all data related to the tree.
   */
  void Clear();

  /**
   * @brief Returns the current number of leaves in the tree.
   * @return The leaf count.
   */
  [[nodiscard]] int GetLeafCount() const;

  /**
   * @brief Returns the current number of flowers in the tree.
   * @return The flower count.
   */
  [[nodiscard]] int GetFlowerCount() const;

  /**
   * @brief Returns the current number of fruits in the tree.
   * @return The fruit count.
   */
  [[nodiscard]] int GetFruitCount() const;

  /**
   * @brief Simulates one growth iteration for the entire tree.
   * @param delta_time Time step for the iteration.
   * @param global_transform The global transformation matrix of the tree.
   * @param climate_model The climate influencing the tree.
   * @param shoot_growth_controller Procedural parameters for branch growth.
   * @param foliage_controller Procedural parameters for foliage growth.
   * @param reproduction_controller Procedural parameters for reproduction units.
   * @param shoot_pruning_controller Procedural parameters for branch pruning.
   * @param pruning Whether pruning should be applied automatically.
   * @return Whether structural changes occurred during growth.
   */
  bool Grow(float delta_time, const glm::mat4& global_transform, ClimateModel& climate_model,
            const ShootGrowthController& shoot_growth_controller, const FoliageController& foliage_controller,
            const ReproductionController& reproduction_controller,
            const ShootPruningController& shoot_pruning_controller, bool pruning = true);

  /**
   * @brief Simulates one growth iteration for a subtree.
   * @param delta_time Time step for the iteration.
   * @param base_internode_handle The internode handle indicating the subtree root.
   * @param global_transform The global transformation matrix of the tree.
   * @param climate_model The climate influencing the tree.
   * @param shoot_growth_controller Procedural parameters for branch growth.
   * @param foliage_controller Procedural parameters for foliage growth.
   * @param reproduction_controller Procedural parameters for reproduction units.
   * @param shoot_pruning_controller Procedural parameters for branch pruning.
   * @param pruning Whether pruning should be applied automatically.
   * @return Whether structural changes occurred during growth.
   */
  bool Grow(float delta_time, SkeletonNodeHandle base_internode_handle, const glm::mat4& global_transform,
            ClimateModel& climate_model, const ShootGrowthController& shoot_growth_controller,
            const FoliageController& foliage_controller, const ReproductionController& reproduction_controller,
            const ShootPruningController& shoot_pruning_controller, bool pruning = true);

  int history_limit = -1;  ///< The limit for stored history states.

  /**
   * @brief Samples the temperature at the tree's location.
   * @param global_transform The global transformation matrix of the tree.
   * @param climate_model The climate model providing temperature data.
   */
  void SampleTemperature(const glm::mat4& global_transform, ClimateModel& climate_model);

  /**
   * @brief Provides direct access to the shoot skeleton for modifications.
   * @return A reference to the shoot skeleton.
   */
  [[nodiscard]] ShootSkeleton& RefShootSkeleton();

  /**
   * @brief Retrieves a previous state of the shoot skeleton.
   * @param iteration The iteration index (-1 for the latest state).
   * @return A constant reference to the requested shoot skeleton.
   */
  [[nodiscard]] const ShootSkeleton& PeekShootSkeleton(int iteration = -1) const;

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
   * @brief Saves to a YAML emitter.
   * @param name The name of the settings entry.
   * @param out The YAML emitter to serialize data into.
   */
  void Save(const std::string& name, YAML::Emitter& out) const;

  /**
   * @brief Loads from a YAML node.
   * @param name The name of the settings entry.
   * @param in The YAML node containing serialized data.
   */
  void Load(const std::string& name, const YAML::Node& in);
};

template <typename SrcSkeletonData, typename SrcFlowData, typename SrcNodeData>
/**
 * @brief Initializes the tree model by cloning data from an existing skeleton.
 * @tparam SrcSkeletonData Type of the source skeleton data.
 * @tparam SrcFlowData Type of the source flow data.
 * @tparam SrcNodeData Type of the source node data.
 * @param src_skeleton The source skeleton used for initialization.
 */
void ShootModel::Initialize(const Skeleton<SrcSkeletonData, SrcFlowData, SrcNodeData>& src_skeleton) {
  if (initialized_)
    Clear();
  random_engine_ = std::mt19937(static_cast<uint32_t>(seed));
  shoot_skeleton_.Clone(src_skeleton);
  shoot_skeleton_.CalculateDistance();
  shoot_skeleton_.CalculateRegulatedGlobalRotation();
  shoot_skeleton_.SortLists();
  initialized_ = true;
}

}  // namespace eco_sys_lab_plugin
