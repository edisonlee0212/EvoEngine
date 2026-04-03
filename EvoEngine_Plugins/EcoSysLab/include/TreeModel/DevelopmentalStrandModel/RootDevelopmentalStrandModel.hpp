#pragma once
#include "DevelopmentalStrandModel.hpp"
#include "GpuProfileSimulator.hpp"
#include "RootDevelopmentalStrandModelData.hpp"
#include "RootModel.hpp"

namespace eco_sys_lab_plugin {
using namespace evo_engine;

/**
 * @brief Procedural strand model that co-evolves strands alongside root skeleton growth.
 *
 * Mirrors DevelopmentalStrandModel but operates on the root skeleton.
 * Roots have no foliage/fruit/buds, so wound detection and cladoptosis are simplified.
 */
class RootDevelopmentalStrandModel {
  std::mt19937 random_engine_;
  uint16_t current_growth_step_ = 0;

  void InitializeStrandsForNewNode(SkeletonNodeHandle new_node_handle, SkeletonNodeHandle parent_handle);

  void SplitProfileForBranching(SkeletonNodeHandle new_branch_handle, SkeletonNodeHandle parent_handle,
                                const glm::quat& branch_global_rotation, const glm::quat& parent_global_rotation);

  void AllocateNewStrandsForEndNode(SkeletonNodeHandle end_node_handle, const StrandModelParameters& params);

  void AllocateStrandsForEndNode(SkeletonNodeHandle end_node_handle, int strand_count,
                                 const StrandModelParameters& params);

  void SyncInitialPositions();
  void FreezeInteriorParticles();
  void ProcessWounds(const StrandModelParameters& params);

 public:
  bool enabled = false;
  bool gpu_profile_packing = true;
  int gpu_packing_iterations = 50;
  StrandColorMode strand_color_mode = StrandColorMode::kDefault;

  [[nodiscard]] uint16_t CurrentGrowthStep() const { return current_growth_step_; }

  RootDevelopmentalStrandModelSkeleton skeleton;
  GpuProfileSimulator gpu_profile_simulator;
  int seed = 0;

  void Enable(const RootSkeleton& root_skeleton, const StrandModelParameters& params);
  void Disable();
  void Reset();

  void OnGrowthStep(const RootSkeleton& root_skeleton, const std::vector<RootModel::GrowthEvent>& growth_events,
                    const std::vector<std::vector<SkeletonNodeHandle>>& pruning_event_batches, bool pruning_occurred,
                    const StrandModelParameters& params);

  void ApplyPruningEvents(const RootSkeleton& root_skeleton,
                          const std::vector<std::vector<SkeletonNodeHandle>>& pruning_event_batches,
                          const StrandModelParameters& params);

  void ResyncAfterPruning(const RootSkeleton& root_skeleton, const StrandModelParameters& params);

  void SyncNodeInfo(const RootSkeleton& root_skeleton);

  void ApplyProfiles(const StrandModelParameters& params);

  std::shared_ptr<Strands> GenerateStrands(int node_max_count = -1) const;

  void Save(const std::string& name, YAML::Emitter& out) const;
  void Load(const std::string& name, const YAML::Node& in);
};

}  // namespace eco_sys_lab_plugin
