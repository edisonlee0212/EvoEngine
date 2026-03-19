#pragma once
#include "GpuProfileSimulator.hpp"
#include "ProceduralStrandModelData.hpp"
#include "ShootModel.hpp"

namespace eco_sys_lab_plugin {
using namespace evo_engine;

/**
 * @brief Procedural strand model that co-evolves strands alongside skeleton growth.
 *
 * Unlike the existing StrandModel which rebuilds all strands post-hoc after growth completes,
 * ProceduralStrandModel maintains a unified skeleton where strand data lives alongside
 * internode growth data. This enables incremental strand growth (O(new_nodes) per step
 * rather than O(total_nodes)), cambial layer tracking, and biological phenomena like
 * annual rings and wound response.
 */
class ProceduralStrandModel {
  /// Random engine for stochastic strand operations.
  std::mt19937 random_engine_;

  /// Growth step counter, incremented each OnGrowthStep() call.
  /// Used to stamp birth_step on newly created particles for future ring tracking.
  uint16_t current_growth_step_ = 0;

  /**
   * @brief For prolongation: extends ALL parent strands into a new child node.
   *
   * Copies the parent's profile layout, extending each strand with a new segment at the
   * new node and creating a matching particle in the new node's profile.
   * @param new_node_handle The newly created node (via prolongation Extend).
   * @param parent_handle The parent node from which strands are inherited.
   */
  void InitializeStrandsForNewNode(SkeletonNodeHandle new_node_handle, SkeletonNodeHandle parent_handle);

  /**
   * @brief For branching: extends only bud-facing strands into a new branch node.
   *
   * Projects the branch direction into the parent's cross-section plane and selects
   * particles on the bud-facing side. Only those strands are extended into the branch.
   * @param new_branch_handle The newly created branch node.
   * @param parent_handle The parent node at the branch junction.
   * @param branch_global_rotation Global rotation of the new branch.
   * @param parent_global_rotation Global rotation of the parent.
   */
  void SplitProfileForBranching(SkeletonNodeHandle new_branch_handle, SkeletonNodeHandle parent_handle,
                                const glm::quat& branch_global_rotation, const glm::quat& parent_global_rotation);

  /**
   * @brief Allocates new strands from root to the given end node.
   *
   * Each new end node (created by branching) should own `end_node_strands` independent
   * strands running root-to-tip, matching the behaviour of Enable(). Segments and
   * particles are created at every ancestor node along the path, so the trunk naturally
   * accumulates strands from all descendant branches.
   * @param end_node_handle The new end (tip) node that needs its own strands.
   * @param params Strand model parameters (supplies end_node_strands, physics settings).
   */
  void AllocateNewStrandsForEndNode(SkeletonNodeHandle end_node_handle, const StrandModelParameters& params);

  /**
   * @brief Copies each particle's current (post-packing) position to its initial_position.
   *
   * Must be called after GPU profile packing (DownloadToSkeleton) so that
   * ApplyProfiles() reads a stable, converged layout rather than the live solver state.
   */
  void SyncInitialPositions();

  /**
   * @brief Marks non-boundary particles as Frozen across all profiles.
   *
   * After GPU packing converges and SyncInitialPositions() snapshots the layout,
   * this method freezes interior particles so they act as immovable collision obstacles
   * on subsequent packing passes. Boundary particles remain Active.
   */
  void FreezeInteriorParticles();

  /**
   * @brief Advances wound state transitions for all particles.
   *
   * Called each growth step: kWounded -> kHealing -> kHealed (probabilistic).
   * @param params Strand model parameters containing wound_healing_rate.
   */
  void ProcessWounds(const StrandModelParameters& params);

  /**
   * @brief Adds a cambial layer of particles at the profile boundary.
   *
   * Simulates secondary growth by adding new particles around the boundary of the node's
   * cross-section profile. Each new particle creates a new strand originating at this node.
   * These particles carry their birth age for annual ring tracking.
   * @param node_handle The node to add cambial growth to.
   * @param growth_age The current growth age/time (for ring tracking).
   * @param params Strand model parameters controlling strand allocation.
   */
  void AddCambialLayer(SkeletonNodeHandle node_handle, float growth_age, const StrandModelParameters& params);

 public:
  /// Whether the procedural strand model is active. When false, the skeleton
  /// behaves identically to a plain ShootSkeleton — strand optional fields remain empty.
  bool enabled = false;

  /// Whether to run profile packing on GPU (true) or skip it (false).
  bool gpu_profile_packing = true;

  /// Number of GPU profile packing iterations per growth step.
  int gpu_packing_iterations = 50;

  /// The unified skeleton combining internode growth and strand model data.
  ProceduralStrandModelSkeleton skeleton;

  /// GPU profile packing simulator.
  GpuProfileSimulator gpu_profile_simulator;

  /// Seed value for random number generation in strand operations.
  int seed = 0;

  /**
   * @brief Enables the procedural strand model, initializing from an existing shoot skeleton.
   *
   * Clones the topology from the shoot skeleton, initializes strand data on all nodes,
   * and builds initial strands for existing nodes (retroactive initialization).
   * @param shoot_skeleton The existing shoot skeleton to clone topology from.
   * @param params Parameters defining the strand model profile.
   */
  void Enable(const ShootSkeleton& shoot_skeleton, const StrandModelParameters& params);

  /**
   * @brief Disables the procedural strand model and clears strand data from all nodes.
   */
  void Disable();

  /**
   * @brief Resets the entire model (skeleton + strand data) to initial state.
   */
  void Reset();

  /**
   * @brief Called after each ShootModel::Grow() step to incrementally update strands.
   *
   * Mirrors topology changes from the shoot skeleton to the unified skeleton,
   * copies updated internode growth data, and incrementally initializes strands
   * for newly created nodes.
   * @param shoot_skeleton The shoot skeleton after the growth step.
   * @param growth_events The topology changes that occurred during this growth step.
   * @param params Strand model parameters.
   */
  void OnGrowthStep(const ShootSkeleton& shoot_skeleton, const std::vector<ShootModel::GrowthEvent>& growth_events,
                    bool pruning_occurred, const StrandModelParameters& params);

  /**
   * @brief Re-synchronizes the procedural skeleton from the shoot skeleton after pruning.
   *
   * Pruning uses swap-and-pop node removal which invalidates growth event handles.
   * This method re-clones the shoot skeleton topology and re-initializes strand data
   * for all nodes that survived pruning, preserving strand group state where possible.
   * @param shoot_skeleton The shoot skeleton in its post-pruning state.
   * @param params Strand model parameters.
   */
  void ResyncAfterPruning(const ShootSkeleton& shoot_skeleton, const StrandModelParameters& params);

  /**
   * @brief Syncs node structural info (positions, rotations, thickness) from the shoot skeleton.
   *
   * Should be called after growth data calculations (CalculateGrowthData, CalculateTransform)
   * to keep the unified skeleton's node info up to date.
   * @param shoot_skeleton The shoot skeleton with updated node info.
   */
  void SyncNodeInfo(const ShootSkeleton& shoot_skeleton);

  /**
   * @brief Converts 2D profile particle positions into 3D strand segment positions.
   *
   * For each node, projects the profile particles' 2D positions onto the node's
   * cross-section plane using the node's coordinate frame, producing 3D end_position,
   * rotation, and color for each strand segment.
   * @param params Strand model parameters (for colors, cladoptosis).
   */
  void ApplyProfiles(const StrandModelParameters& params);

  /**
   * @brief Builds a renderable Strands asset from the strand group.
   * @param node_max_count Maximum number of segments per strand (-1 for unlimited).
   * @return Shared pointer to the Strands asset.
   */
  std::shared_ptr<Strands> GenerateStrands(int node_max_count = -1) const;

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

}  // namespace eco_sys_lab_plugin
