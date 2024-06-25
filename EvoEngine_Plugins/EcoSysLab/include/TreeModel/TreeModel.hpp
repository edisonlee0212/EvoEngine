#pragma once
// #include "VoxelSoilModel.hpp"
#include "ClimateModel.hpp"
#include "Octree.hpp"
#include "TreeGrowthController.hpp"

using namespace evo_engine;
namespace eco_sys_lab {
struct TreeGrowthSettings {
  float node_developmental_vigor_filling_rate = 1.0f;
  bool use_space_colonization = false;
  bool space_colonization_auto_resize = false;
  float space_colonization_removal_distance_factor = 2;
  float space_colonization_detection_distance_factor = 4;
  float space_colonization_theta = 90.0f;
};

class TreeModel {
#pragma region Tree Growth
  ShootFlux CollectShootFlux(const std::vector<SkeletonNodeHandle>& sorted_internode_list);
  void CalculateInternodeStrength(const std::vector<SkeletonNodeHandle>& sorted_internode_list,
                                  const ShootGrowthController& shoot_growth_controller);
  void CalculateGrowthRate(const std::vector<SkeletonNodeHandle>& sorted_internode_list, float factor);

  float CalculateGrowthPotential(const std::vector<SkeletonNodeHandle>& sorted_internode_list,
                                 const ShootGrowthController& shoot_growth_controller);

  bool PruneInternodes(const glm::mat4& global_transform, ClimateModel& climate_model,
                       const ShootGrowthController& shoot_growth_controller);

  void CalculateThickness(const ShootGrowthController& shoot_growth_controller);

  void CalculateBiomass(SkeletonNodeHandle internode_handle, const ShootGrowthController& shoot_growth_controller);

  void CalculateSaggingStress(SkeletonNodeHandle internode_handle, const ShootGrowthController& shoot_growth_controller);

  void CalculateLevel();

  bool GrowInternode(ClimateModel& climate_model, SkeletonNodeHandle internode_handle,
                     const ShootGrowthController& shoot_growth_controller);

  bool GrowReproductiveModules(ClimateModel& climate_model, SkeletonNodeHandle internode_handle,
                               const ShootGrowthController& shoot_growth_controller);

  bool ElongateInternode(float extended_length, SkeletonNodeHandle internode_handle,
                         const ShootGrowthController& shoot_growth_controller, float& collected_inhibitor);

  void ShootGrowthPostProcess(const ShootGrowthController& shoot_growth_controller);

  friend class Tree;
#pragma endregion

  bool initialized_ = false;

  ShootSkeleton shoot_skeleton_;

  std::deque<ShootSkeleton> history_;

  int leaf_count_ = 0;
  int fruit_count_ = 0;

  float age_ = 0;
  int age_in_year_ = 0;
  float current_delta_time_ = 1.0f;

  bool enable_shoot_ = true;

  void ResetReproductiveModule();

  int current_seed_value_ = 0;

 public:
  void Initialize(const ShootGrowthController& shoot_growth_controller);
  template <typename SrcSkeletonData, typename SrcFlowData, typename SrcNodeData>
  void Initialize(const Skeleton<SrcSkeletonData, SrcFlowData, SrcNodeData>& src_skeleton);

  float GetSubTreeMaxAge(SkeletonNodeHandle base_internode_handle) const;
  bool Reduce(const ShootGrowthController& shoot_growth_controller, SkeletonNodeHandle base_internode_handle,
              float target_age);

  void CalculateTransform(const ShootGrowthController& shoot_growth_controller, bool sagging);

  int m_seed = 0;

  void RegisterVoxel(const glm::mat4& global_transform, ClimateModel& climate_model,
                     const ShootGrowthController& shoot_growth_controller);
  TreeOccupancyGrid tree_occupancy_grid{};

  void PruneInternode(SkeletonNodeHandle internode_handle);

  void CalculateShootFlux(const glm::mat4& global_transform, const ClimateModel& climate_model,
                          const ShootGrowthController& shoot_growth_controller);

  void HarvestFruits(const std::function<bool(const ReproductiveModule& fruit)>& harvest_function);

  int iteration = 0;

  static void ApplyTropism(const glm::vec3& target_dir, float tropism, glm::vec3& front, glm::vec3& up);

  static void ApplyTropism(const glm::vec3& target_dir, float tropism, glm::quat& rotation);

  std::vector<int> internode_order_counts;

  TreeGrowthSettings tree_growth_settings;

  glm::vec3 current_gravity_direction = glm::vec3(0, -1, 0);

  /**
   * Erase the entire tree.
   */
  void Clear();

  [[nodiscard]] int GetLeafCount() const;
  [[nodiscard]] int GetFruitCount() const;
  /**
   * Grow one iteration of the tree, given the nutrients and the procedural parameters.
   * @param delta_time The real world time for this iteration.
   * @param global_transform The global transform of tree in world space.
   * @param climate_model The climate model.
   * @param shoot_growth_controller The procedural parameters that guides the growth of the branches.
   * @param pruning If we want auto pruning to be enabled.
   * @param overrideGrowthRate If positive (clamped to below 1), the growth rate will be overwritten instead of
   * calculating by available resources.
   * @return Whether the growth caused a structural change during the growth.
   */
  bool Grow(float delta_time, const glm::mat4& global_transform, ClimateModel& climate_model,
            const ShootGrowthController& shoot_growth_controller, bool pruning = true);

  /**
   * Grow one iteration of the tree, given the nutrients and the procedural parameters.
   * @param delta_time The real world time for this iteration.
   * @param base_internode_handle The base internode of the subtree to grow. If set to 0 or -1, this equals with growing
   * full tree.
   * @param global_transform The global transform of tree in world space.
   * @param climate_model The climate model.
   * @param shoot_growth_controller The procedural parameters that guides the growth of the branches.
   * @param pruning If we want auto pruning to be enabled.
   * calculating by available resources.
   * @return Whether the growth caused a structural change during the growth.
   */
  bool Grow(float delta_time, SkeletonNodeHandle base_internode_handle, const glm::mat4& global_transform,
            ClimateModel& climate_model, const ShootGrowthController& shoot_growth_controller, bool pruning = true);

  int history_limit = -1;

  void SampleTemperature(const glm::mat4& global_transform, ClimateModel& climate_model);
  [[nodiscard]] ShootSkeleton& RefShootSkeleton();

  [[nodiscard]] const ShootSkeleton& PeekShootSkeleton(int iteration = -1) const;

  void ClearHistory();

  void Step();

  void Pop();

  [[nodiscard]] int CurrentIteration() const;

  void Reverse(int iteration);
};

template <typename SrcSkeletonData, typename SrcFlowData, typename SrcNodeData>
void TreeModel::Initialize(const Skeleton<SrcSkeletonData, SrcFlowData, SrcNodeData>& src_skeleton) {
  if (initialized_)
    Clear();
  shoot_skeleton_.Clone(src_skeleton);
  shoot_skeleton_.CalculateDistance();
  shoot_skeleton_.CalculateRegulatedGlobalRotation();
  shoot_skeleton_.SortLists();
  current_seed_value_ = m_seed;
  initialized_ = true;
}
}  // namespace eco_sys_lab
