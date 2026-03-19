#include "ProceduralStrandModel.hpp"
#include <unordered_set>

using namespace eco_sys_lab_plugin;

namespace {
constexpr uint32_t kMaxGpuPackingParticles = 300000;
constexpr size_t kMaxRenderableStrandPoints = 4000000;
}

void ProceduralStrandModel::Enable(const ShootSkeleton& shoot_skeleton, const StrandModelParameters& params) {
  enabled = true;
  current_growth_step_ = 0;
  random_engine_ = std::mt19937(static_cast<uint32_t>(seed));

  // Clone topology from the shoot skeleton into the unified skeleton.
  skeleton.Clone(shoot_skeleton);

  // Initialize skeleton-level strand data.
  skeleton.data.InitializeStrandData();
  skeleton.data.strand_data->strand_group = {};

  // Initialize per-node data: copy internode data and init strand data.
  const auto& sorted_list = skeleton.PeekSortedNodeList();
  for (const auto& node_handle : sorted_list) {
    auto& node = skeleton.RefNode(node_handle);
    // Copy internode growth data from the shoot skeleton.
    node.data.internode_data = shoot_skeleton.PeekNode(node_handle).data;
    // Copy node info.
    node.info = shoot_skeleton.PeekNode(node_handle).info;
    // Initialize strand data.
    node.data.InitializeStrandData();
    node.data.strand_data->profile.Reset(0.001f);
    node.data.strand_data->profile.particle_physics_settings = params.profile_physics_settings;
  }

  // Build initial strands for existing nodes (retroactive initialization).
  // For each end node, create initial strands running from root to that tip.
  auto& strand_group = skeleton.data.strand_data->strand_group;
  for (const auto& node_handle : sorted_list) {
    const auto& node = skeleton.PeekNode(node_handle);
    if (!node.IsEndNode())
      continue;

    // Build parent chain from this end node to root.
    std::vector<SkeletonNodeHandle> root_to_node_chain;
    SkeletonNodeHandle walker = node_handle;
    while (walker >= 0) {
      root_to_node_chain.push_back(walker);
      walker = skeleton.PeekNode(walker).GetParentHandle();
    }
    std::reverse(root_to_node_chain.begin(), root_to_node_chain.end());

    // Allocate initial strands for this end node.
    const int strand_count = params.end_node_strands;
    for (int i = 0; i < strand_count; i++) {
      const auto strand_handle = strand_group.AllocateStrand();

      // Seed position only at the root of the chain; propagate down for
      // cross-node angular coherence.  GPU packing then refines each node.
      glm::vec2 coherent_position(0.f);
      bool is_chain_root = true;

      // Create segments for each node in the chain.
      for (const auto chain_handle : root_to_node_chain) {
        auto& chain_node = skeleton.RefNode(chain_handle);
        auto& profile = chain_node.data.strand_data->profile;

        const auto new_segment_handle = strand_group.Extend(strand_handle);
        const auto new_particle_handle = profile.AllocateParticle();
        auto& particle = profile.RefParticle(new_particle_handle);
        particle.strand_handle = strand_handle;
        particle.strand_segment_handle = new_segment_handle;
        particle.base = (chain_handle == node_handle);

        // Only the root of the chain gets a random position.
        // All subsequent nodes inherit it so the same strand keeps a
        // consistent angular position across nodes.
        if (is_chain_root) {
          const int existing = static_cast<int>(profile.PeekParticles().size());
          const float disk_radius = glm::sqrt(static_cast<float>(existing));
          coherent_position = existing <= 1 ? glm::vec2(0.f)
                                            : Random::Disk(random_engine_, disk_radius);
          is_chain_root = false;
        }
        particle.SetPosition(coherent_position);
        particle.SetInitialPosition(coherent_position);

        auto& seg_data = strand_group.RefStrandSegmentData(new_segment_handle);
        seg_data.node_handle = chain_handle;
        seg_data.profile_particle_handle = new_particle_handle;
      }
    }
  }

  // Build particle_map and strand_count for all nodes.
  for (const auto& node_handle : sorted_list) {
    auto& node = skeleton.RefNode(node_handle);
    auto& strand_d = *node.data.strand_data;
    strand_d.particle_map.clear();
    for (auto& particle : strand_d.profile.RefParticles()) {
      strand_d.particle_map.insert({particle.strand_handle, particle.GetHandle()});
    }
    strand_d.strand_count = static_cast<int>(strand_d.profile.PeekParticles().size());
  }

  skeleton.data.strand_data->num_of_particles = 0;
  for (const auto& node_handle : sorted_list) {
    skeleton.data.strand_data->num_of_particles +=
        static_cast<int>(skeleton.PeekNode(node_handle).data.strand_data->profile.PeekParticles().size());
  }

  // Run initial GPU profile packing so particles are spread out before the first render.
  // Without this, all non-tip particles default to (0,0) and appear as a degenerate point.
  if (gpu_profile_packing) {
    if (skeleton.data.strand_data->num_of_particles > static_cast<int>(kMaxGpuPackingParticles)) {
      EVOENGINE_WARNING(
          "ProceduralStrandModel::Enable skipped GPU profile packing due to particle count " +
          std::to_string(skeleton.data.strand_data->num_of_particles) +
          ". Reduce strand density or disable GPU Profile Packing.");
    } else {
      std::vector<SkeletonNodeHandle> all_nodes(sorted_list.begin(), sorted_list.end());
      gpu_profile_simulator.UploadFromSkeleton(skeleton, all_nodes);
      gpu_profile_simulator.Simulate(static_cast<uint32_t>(gpu_packing_iterations), params);
      gpu_profile_simulator.DownloadToSkeleton(skeleton);

      // Top-down propagation: copy each parent's converged position to its children
      // as a warm-start, then re-pack.  This establishes cross-node angular coherence
      // analogous to the legacy model's bottom-up MergeTask.
      for (const auto& nh : sorted_list) {
        const auto& node = skeleton.PeekNode(nh);
        const auto ph = node.GetParentHandle();
        if (ph < 0)
          continue;

        const auto& parent_sd = *skeleton.PeekNode(ph).data.strand_data;
        auto& child_sd = *skeleton.RefNode(nh).data.strand_data;

        for (const auto& [strand_h, particle_h] : child_sd.particle_map) {
          const auto parent_it = parent_sd.particle_map.find(strand_h);
          if (parent_it != parent_sd.particle_map.end()) {
            const auto& parent_p = parent_sd.profile.PeekParticle(parent_it->second);
            child_sd.profile.RefParticle(particle_h).SetPosition(parent_p.GetPosition());
          }
        }
      }

      // Second packing pass from the coherent warm-start.
      gpu_profile_simulator.UploadFromSkeleton(skeleton, all_nodes);
      gpu_profile_simulator.Simulate(static_cast<uint32_t>(gpu_packing_iterations), params);
      gpu_profile_simulator.DownloadToSkeleton(skeleton);
    }
  }

  // Snapshot converged positions so ApplyProfiles reads a stable layout.
  SyncInitialPositions();

  // Freeze interior particles so only boundary particles are mobile on future packing passes.
  FreezeInteriorParticles();
}

void ProceduralStrandModel::Disable() {
  enabled = false;
  skeleton.data.strand_data.reset();
  auto& raw_nodes = skeleton.RefRawNodes();
  for (auto& node : raw_nodes) {
    node.data.strand_data.reset();
  }
}

void ProceduralStrandModel::Reset() {
  enabled = false;
  seed = 0;
  skeleton = {};
}

// ---------------------------------------------------------------------------
// Phase 2: Incremental strand growth methods
// ---------------------------------------------------------------------------

void ProceduralStrandModel::InitializeStrandsForNewNode(const SkeletonNodeHandle new_node_handle,
                                                        const SkeletonNodeHandle parent_handle) {
  auto& new_node = skeleton.RefNode(new_node_handle);
  new_node.data.InitializeStrandData();
  auto& new_strand_data = *new_node.data.strand_data;
  new_strand_data.profile.Reset(0.001f);

  if (parent_handle < 0)
    return;

  const auto& parent_node = skeleton.PeekNode(parent_handle);
  if (!parent_node.data.HasStrandData())
    return;

  auto& strand_group = skeleton.data.strand_data->strand_group;
  const auto& parent_strand_data = *parent_node.data.strand_data;

  // Inherit profile physics settings from parent.
  new_strand_data.profile.particle_physics_settings = parent_strand_data.profile.particle_physics_settings;

  // For each strand passing through the parent, extend it into the new node.
  for (const auto& [strand_handle, parent_particle_handle] : parent_strand_data.particle_map) {
    const auto& parent_particle = parent_strand_data.profile.PeekParticle(parent_particle_handle);

    // Extend the strand with a new segment.
    const auto new_segment_handle = strand_group.Extend(strand_handle);

    // Allocate a particle in the new node's profile.
    const auto new_particle_handle = new_strand_data.profile.AllocateParticle();
    auto& new_particle = new_strand_data.profile.RefParticle(new_particle_handle);
    new_particle.strand_handle = strand_handle;
    new_particle.strand_segment_handle = new_segment_handle;
    new_particle.base = false;
    new_particle.data.birth_step = current_growth_step_;
    // Inherit 2D position from parent particle.
    new_particle.SetPosition(parent_particle.GetPosition());
    new_particle.SetInitialPosition(parent_particle.GetInitialPosition());

    // Link the segment data.
    auto& seg_data = strand_group.RefStrandSegmentData(new_segment_handle);
    seg_data.node_handle = new_node_handle;
    seg_data.profile_particle_handle = new_particle_handle;

    // Record in particle map.
    new_strand_data.particle_map.insert({strand_handle, new_particle_handle});
  }

  new_strand_data.strand_count = static_cast<int>(new_strand_data.profile.PeekParticles().size());
}

void ProceduralStrandModel::SplitProfileForBranching(const SkeletonNodeHandle new_branch_handle,
                                                     const SkeletonNodeHandle parent_handle,
                                                     const glm::quat& branch_global_rotation,
                                                     const glm::quat& parent_global_rotation) {
  auto& branch_node = skeleton.RefNode(new_branch_handle);
  branch_node.data.InitializeStrandData();
  auto& branch_strand_data = *branch_node.data.strand_data;
  branch_strand_data.profile.Reset(0.001f);

  if (parent_handle < 0)
    return;

  const auto& parent_node = skeleton.PeekNode(parent_handle);
  if (!parent_node.data.HasStrandData())
    return;

  auto& strand_group = skeleton.data.strand_data->strand_group;
  const auto& parent_strand_data = *parent_node.data.strand_data;

  // Inherit profile physics settings from parent.
  branch_strand_data.profile.particle_physics_settings = parent_strand_data.profile.particle_physics_settings;

  // Compute 2D branch direction in parent's cross-section plane.
  // The parent's cross-section plane is perpendicular to the parent's forward axis.
  // The branch direction projected into this plane gives us the 2D direction for partitioning.
  const glm::vec3 parent_forward = glm::normalize(parent_global_rotation * glm::vec3(0, 0, -1));
  const glm::vec3 parent_up = glm::normalize(parent_global_rotation * glm::vec3(0, 1, 0));
  const glm::vec3 parent_left = glm::normalize(parent_global_rotation * glm::vec3(1, 0, 0));
  const glm::vec3 branch_forward = glm::normalize(branch_global_rotation * glm::vec3(0, 0, -1));

  // Project branch direction onto parent's cross-section plane.
  const glm::vec3 branch_in_plane = branch_forward - glm::dot(branch_forward, parent_forward) * parent_forward;
  glm::vec2 branch_dir_2d(0.f);
  if (glm::length(branch_in_plane) > 1e-6f) {
    const glm::vec3 normalized = glm::normalize(branch_in_plane);
    branch_dir_2d = glm::vec2(glm::dot(normalized, parent_left), glm::dot(normalized, parent_up));
  }

  // Select particles on the branch-facing side.
  // A particle is "branch-facing" if its 2D position dot branch_dir_2d > 0,
  // or if there are very few particles (ensure at least 1 strand routes into branch).
  const auto& parent_particles = parent_strand_data.profile.PeekParticles();
  const int total_particles = static_cast<int>(parent_particles.size());

  // Collect strand handles that should route into the branch.
  std::vector<std::pair<StrandHandle, ParticleHandle>> branch_strands;
  for (const auto& [strand_handle, particle_handle] : parent_strand_data.particle_map) {
    const auto& particle = parent_strand_data.profile.PeekParticle(particle_handle);
    const glm::vec2 pos = particle.GetPosition();
    const float alignment = glm::dot(pos, branch_dir_2d);

    // Route this strand to branch if it's on the branch-facing side.
    if (alignment > 0.0f || total_particles <= 1) {
      branch_strands.emplace_back(strand_handle, particle_handle);
    }
  }

  // Ensure at least one strand if parent has strands.
  if (branch_strands.empty() && !parent_strand_data.particle_map.empty()) {
    // Pick the strand closest to the branch direction.
    float best_alignment = -FLT_MAX;
    std::pair<StrandHandle, ParticleHandle> best = {-1, -1};
    for (const auto& [strand_handle, particle_handle] : parent_strand_data.particle_map) {
      const auto& particle = parent_strand_data.profile.PeekParticle(particle_handle);
      const float alignment = glm::dot(particle.GetPosition(), branch_dir_2d);
      if (alignment > best_alignment) {
        best_alignment = alignment;
        best = {strand_handle, particle_handle};
      }
    }
    if (best.first >= 0) {
      branch_strands.push_back(best);
    }
  }

  // Find the prolongation sibling — the child of parent that was already processed
  // by InitializeStrandsForNewNode.  Its strand extensions must be retracted for
  // strands that are selected for the branch, otherwise the same strand visits both
  // prolongation and branch sequentially (the "noodle" bug).
  SkeletonNodeHandle prolongation_handle = -1;
  for (const auto child_h : skeleton.PeekNode(parent_handle).PeekChildHandles()) {
    if (child_h != new_branch_handle) {
      prolongation_handle = child_h;
      break;
    }
  }

  // Route the selected strands into the branch node.
  for (const auto& [strand_handle, parent_particle_handle] : branch_strands) {
    const auto& parent_particle = parent_strand_data.profile.PeekParticle(parent_particle_handle);

    StrandSegmentHandle seg_handle = -1;

    // If the strand was already extended into the prolongation child, reassign that
    // segment to the branch instead of allocating a new one.
    if (prolongation_handle >= 0) {
      auto& prolongation_node = skeleton.RefNode(prolongation_handle);
      if (prolongation_node.data.HasStrandData()) {
        auto& prolongation_strand_data = *prolongation_node.data.strand_data;
        auto it = prolongation_strand_data.particle_map.find(strand_handle);
        if (it != prolongation_strand_data.particle_map.end()) {
          const auto prolongation_particle_handle = it->second;
          seg_handle = prolongation_strand_data.profile.PeekParticle(prolongation_particle_handle)
                           .strand_segment_handle;
          // Remove strand from prolongation child's routing.
          prolongation_strand_data.particle_map.erase(it);
          prolongation_strand_data.strand_count =
              static_cast<int>(prolongation_strand_data.particle_map.size());
        }
      }
    }

    if (seg_handle < 0) {
      // No prolongation extension to steal — allocate a new segment.
      seg_handle = strand_group.Extend(strand_handle);
    }

    const auto new_particle_handle = branch_strand_data.profile.AllocateParticle();
    auto& new_particle = branch_strand_data.profile.RefParticle(new_particle_handle);
    new_particle.strand_handle = strand_handle;
    new_particle.strand_segment_handle = seg_handle;
    new_particle.base = false;
    new_particle.data.birth_step = current_growth_step_;
    new_particle.SetPosition(parent_particle.GetPosition());
    new_particle.SetInitialPosition(parent_particle.GetInitialPosition());

    auto& seg_data = strand_group.RefStrandSegmentData(seg_handle);
    seg_data.node_handle = new_branch_handle;
    seg_data.profile_particle_handle = new_particle_handle;

    branch_strand_data.particle_map.insert({strand_handle, new_particle_handle});
  }

  branch_strand_data.strand_count = static_cast<int>(branch_strand_data.profile.PeekParticles().size());
}

void ProceduralStrandModel::AllocateNewStrandsForEndNode(const SkeletonNodeHandle end_node_handle,
                                                         const StrandModelParameters& params) {
  auto& strand_group = skeleton.data.strand_data->strand_group;

  // Build root-to-tip chain.
  std::vector<SkeletonNodeHandle> root_to_node_chain;
  SkeletonNodeHandle walker = end_node_handle;
  while (walker >= 0) {
    root_to_node_chain.push_back(walker);
    walker = skeleton.PeekNode(walker).GetParentHandle();
  }
  std::reverse(root_to_node_chain.begin(), root_to_node_chain.end());

  const int strand_count = params.end_node_strands;
  for (int i = 0; i < strand_count; i++) {
    const auto strand_handle = strand_group.AllocateStrand();

    // Seed position only at the root of the chain; propagate down for coherence.
    glm::vec2 coherent_position(0.f);
    bool is_chain_root = true;

    for (const auto chain_handle : root_to_node_chain) {
      auto& chain_node = skeleton.RefNode(chain_handle);
      if (!chain_node.data.HasStrandData()) {
        chain_node.data.InitializeStrandData();
        chain_node.data.strand_data->profile.Reset(0.001f);
        chain_node.data.strand_data->profile.particle_physics_settings = params.profile_physics_settings;
      }
      auto& strand_data = *chain_node.data.strand_data;
      auto& profile = strand_data.profile;

      const auto new_segment_handle = strand_group.Extend(strand_handle);
      const auto new_particle_handle = profile.AllocateParticle();
      auto& particle = profile.RefParticle(new_particle_handle);
      particle.strand_handle = strand_handle;
      particle.strand_segment_handle = new_segment_handle;
      particle.base = (chain_handle == end_node_handle);
      particle.data.birth_step = current_growth_step_;

      // Only the root of the chain gets a random position.
      // Subsequent nodes inherit it for cross-node angular coherence.
      if (is_chain_root) {
        const int existing = static_cast<int>(strand_data.particle_map.size()) + 1;
        const float disk_radius = glm::sqrt(static_cast<float>(existing));
        coherent_position = existing <= 1 ? glm::vec2(0.f)
                                          : Random::Disk(random_engine_, disk_radius);
        is_chain_root = false;
      }
      particle.SetPosition(coherent_position);
      particle.SetInitialPosition(coherent_position);

      auto& seg_data = strand_group.RefStrandSegmentData(new_segment_handle);
      seg_data.node_handle = chain_handle;
      seg_data.profile_particle_handle = new_particle_handle;

      strand_data.particle_map.insert({strand_handle, new_particle_handle});
      strand_data.strand_count = static_cast<int>(strand_data.particle_map.size());
    }
  }
}

void ProceduralStrandModel::AddCambialLayer(const SkeletonNodeHandle node_handle, const float growth_age,
                                            const StrandModelParameters& params) {
  auto& node = skeleton.RefNode(node_handle);
  if (!node.data.HasStrandData())
    return;

  auto& strand_data = *node.data.strand_data;
  auto& profile = strand_data.profile;
  auto& strand_group = skeleton.data.strand_data->strand_group;

  // Only add cambial growth if there are existing particles to determine the boundary.
  const auto& particles = profile.PeekParticles();
  if (particles.empty())
    return;

  // Find the maximum radial distance (boundary extent) of existing particles.
  float max_radius = 0.0f;
  for (const auto& particle : particles) {
    max_radius = glm::max(max_radius, glm::length(particle.GetPosition()));
  }

  // Place new particles in a ring just outside the current boundary.
  // Number of new particles scales with circumference.
  const float new_radius = max_radius + strand_data.strand_radius * 2.0f;
  const int num_new_particles = glm::max(1, static_cast<int>(glm::ceil(2.0f * glm::pi<float>() * new_radius /
                                                                        (strand_data.strand_radius * 2.0f))));

  for (int i = 0; i < num_new_particles; i++) {
    const float angle = 2.0f * glm::pi<float>() * static_cast<float>(i) / static_cast<float>(num_new_particles);
    const glm::vec2 position(new_radius * glm::cos(angle), new_radius * glm::sin(angle));

    // Allocate a new strand for this cambial particle.
    const auto strand_handle = strand_group.AllocateStrand();
    const auto new_segment_handle = strand_group.Extend(strand_handle);
    const auto new_particle_handle = profile.AllocateParticle();
    auto& new_particle = profile.RefParticle(new_particle_handle);
    new_particle.strand_handle = strand_handle;
    new_particle.strand_segment_handle = new_segment_handle;
    new_particle.base = true;
    new_particle.SetPosition(position);
    new_particle.SetInitialPosition(position);

    auto& seg_data = strand_group.RefStrandSegmentData(new_segment_handle);
    seg_data.node_handle = node_handle;
    seg_data.profile_particle_handle = new_particle_handle;

    strand_data.particle_map.insert({strand_handle, new_particle_handle});
  }

  strand_data.strand_count = static_cast<int>(profile.PeekParticles().size());
}

// ---------------------------------------------------------------------------
// Main growth step hook
// ---------------------------------------------------------------------------

void ProceduralStrandModel::OnGrowthStep(const ShootSkeleton& shoot_skeleton,
                                         const std::vector<ShootModel::GrowthEvent>& growth_events,
                                         const bool pruning_occurred,
                                         const StrandModelParameters& params) {
  if (!enabled)
    return;

  ++current_growth_step_;

  if (!skeleton.data.HasStrandData()) {
    skeleton.data.InitializeStrandData();
    skeleton.data.strand_data->strand_group = {};
  }

  // If pruning occurred, the growth events contain stale handles (RemoveNodes uses
  // swap-and-pop which reassigns node handles).  Re-sync the entire skeleton.
  if (pruning_occurred) {
    ResyncAfterPruning(shoot_skeleton, params);
    return;
  }

  // Mirror each topology change from the shoot skeleton.
  for (const auto& event : growth_events) {
    // Extend our skeleton to match the shoot skeleton's topology.
    const auto new_handle = skeleton.Extend(event.parent_handle, event.branching);
    // Verify our handle matches the shoot skeleton's (deterministic allocation).
    assert(new_handle == event.new_handle);

    // Copy node structural info and internode data from the shoot skeleton.
    auto& new_node = skeleton.RefNode(new_handle);
    const auto& src_node = shoot_skeleton.PeekNode(new_handle);
    new_node.info = src_node.info;
    new_node.data.internode_data = src_node.data;

    // Initialize strands for the new node.
    if (event.branching) {
      // Branching: the new branch is a new end node and needs its own strands
      // running root-to-tip, matching Enable() semantics.  The prolongation child
      // (processed earlier) keeps ALL parent strands; the branch gets fresh ones.
      AllocateNewStrandsForEndNode(new_handle, params);
    } else {
      // Prolongation: all parent strands continue into the new node.
      InitializeStrandsForNewNode(new_handle, event.parent_handle);
    }
  }

  // Sort lists if topology changed.
  if (!growth_events.empty()) {
    skeleton.SortLists();
  }

  // Sync node info for all existing nodes (positions/rotations may have changed).
  SyncNodeInfo(shoot_skeleton);

  // GPU profile packing for new/modified nodes.
  if (gpu_profile_packing && !growth_events.empty()) {
    std::unordered_set<SkeletonNodeHandle> active_set;
    for (const auto& event : growth_events) {
      active_set.insert(event.new_handle);
      active_set.insert(event.parent_handle);
      // For branching events the new strands added particles at every ancestor;
      // all those nodes need packing so the new particles spread out.
      if (event.branching) {
        SkeletonNodeHandle walker = event.parent_handle;
        while (walker >= 0) {
          active_set.insert(walker);
          walker = skeleton.PeekNode(walker).GetParentHandle();
        }
      }
    }
    std::vector<SkeletonNodeHandle> active_nodes(active_set.begin(), active_set.end());
    size_t active_particle_count = 0;
    for (const auto node_handle : active_nodes) {
      const auto& node = skeleton.PeekNode(node_handle);
      if (!node.data.HasStrandData())
        continue;
      active_particle_count += node.data.strand_data->profile.PeekParticles().size();
    }

    if (active_particle_count > kMaxGpuPackingParticles) {
      EVOENGINE_WARNING(
          "ProceduralStrandModel::OnGrowthStep skipped GPU profile packing due to active particle count " +
          std::to_string(active_particle_count) +
          ". Reduce strand density or disable GPU Profile Packing.");
    } else {
      gpu_profile_simulator.UploadFromSkeleton(skeleton, active_nodes, true);
      gpu_profile_simulator.Simulate(static_cast<uint32_t>(gpu_packing_iterations), params);
      gpu_profile_simulator.DownloadToSkeleton(skeleton);
    }
  }

  // Snapshot converged positions so ApplyProfiles reads a stable layout.
  SyncInitialPositions();

  // Freeze interior particles so only boundary particles are mobile on future packing passes.
  FreezeInteriorParticles();

  // Advance wound healing for particles wounded in previous steps.
  ProcessWounds(params);
}

void ProceduralStrandModel::ResyncAfterPruning(const ShootSkeleton& shoot_skeleton,
                                               const StrandModelParameters& params) {
  // Re-initialize the entire procedural strand model from the current shoot skeleton.
  // This discards accumulated strand developmental history for the pruned step but
  // guarantees correct topology.  Incremental pruning replay can be added later.
  Enable(shoot_skeleton, params);

  // Detect wound sites: nodes flagged as force_sink (manual pruning stubs) or end-nodes
  // whose flushed lateral buds lost their child branches (natural pruning).
  const auto& sorted_list = skeleton.PeekSortedNodeList();
  for (const auto& node_handle : sorted_list) {
    auto& node = skeleton.RefNode(node_handle);
    if (!node.data.HasStrandData())
      continue;

    bool is_wound_site = false;

    // Manual pruning: force_sink is set on parents of pruned branches.
    if (node.data.internode_data.force_sink) {
      is_wound_site = true;
    }

    // Natural pruning: end-node with flushed lateral buds that no longer have children.
    if (!is_wound_site && node.IsEndNode()) {
      for (const auto& bud : node.data.internode_data.buds) {
        if (bud.type == BudType::Lateral && bud.status == OrganStatus::Flushed) {
          is_wound_site = true;
          break;
        }
      }
    }

    if (is_wound_site) {
      // Mark boundary particles at wound sites as wounded.
      auto& profile = node.data.strand_data->profile;
      for (auto& particle : profile.RefParticles()) {
        if (particle.status == ParticleStatus::kDisabled)
          continue;
        if (particle.IsBoundary()) {
          particle.data.wound_state = WoundState::kWounded;
        }
      }
    }
  }
}

void ProceduralStrandModel::SyncInitialPositions() {
  const auto& sorted_list = skeleton.PeekSortedNodeList();
  for (const auto& node_handle : sorted_list) {
    auto& node = skeleton.RefNode(node_handle);
    if (!node.data.HasStrandData())
      continue;
    for (auto& particle : node.data.strand_data->profile.RefParticles()) {
      particle.SetInitialPosition(particle.GetPosition());
    }
  }
}

void ProceduralStrandModel::FreezeInteriorParticles() {
  const auto& sorted_list = skeleton.PeekSortedNodeList();
  for (const auto& node_handle : sorted_list) {
    auto& node = skeleton.RefNode(node_handle);
    if (!node.data.HasStrandData())
      continue;

    auto& profile = node.data.strand_data->profile;
    auto& particles = profile.RefParticles();
    if (particles.size() < 3)
      continue;

    // Recalculate boundary flags from the converged positions.
    profile.CalculateBoundaries(false);

    // Freeze everything that is not on the boundary.
    // Boundary particles are classified as living bark; interior particles as sapwood.
    for (auto& particle : particles) {
      if (particle.status == ParticleStatus::kDisabled)
        continue;
      if (particle.IsBoundary()) {
        particle.status = ParticleStatus::kActive;
        particle.data.tissue_type = TissueType::kBarkLiving;
      } else {
        particle.status = ParticleStatus::kFrozen;
        // Interior particles that were previously bark (before new boundary particles
        // pushed them inward) revert to sapwood.
        if (particle.data.tissue_type == TissueType::kBarkLiving ||
            particle.data.tissue_type == TissueType::kBarkDead) {
          particle.data.tissue_type = TissueType::kSapwood;
        }
      }
    }
  }
}

void ProceduralStrandModel::ProcessWounds(const StrandModelParameters& params) {
  const auto& sorted_list = skeleton.PeekSortedNodeList();
  for (const auto& node_handle : sorted_list) {
    auto& node = skeleton.RefNode(node_handle);
    if (!node.data.HasStrandData())
      continue;

    for (auto& particle : node.data.strand_data->profile.RefParticles()) {
      if (particle.status == ParticleStatus::kDisabled)
        continue;

      switch (particle.data.wound_state) {
        case WoundState::kWounded:
          // Begin healing: transition from fresh wound to callus formation.
          particle.data.wound_state = WoundState::kHealing;
          break;
        case WoundState::kHealing: {
          // Probabilistic healing based on wound_healing_rate.
          // Each growth step, a healing particle has a chance to fully seal.
          const float roll = std::uniform_real_distribution<float>(0.0f, 1.0f)(random_engine_);
          if (roll < params.wound_healing_rate) {
            particle.data.wound_state = WoundState::kHealed;
          }
          break;
        }
        default:
          break;
      }
    }
  }
}

void ProceduralStrandModel::SyncNodeInfo(const ShootSkeleton& shoot_skeleton) {
  const auto& sorted_list = skeleton.PeekSortedNodeList();
  for (const auto& node_handle : sorted_list) {
    auto& node = skeleton.RefNode(node_handle);
    if (node_handle < static_cast<int>(shoot_skeleton.PeekRawNodes().size())) {
      node.info = shoot_skeleton.PeekNode(node_handle).info;
      node.data.internode_data = shoot_skeleton.PeekNode(node_handle).data;
    }
  }
}

// ---------------------------------------------------------------------------
// 2D-to-3D profile application (Phase 4)
// ---------------------------------------------------------------------------

void ProceduralStrandModel::ApplyProfiles(const StrandModelParameters& params) {
  if (!enabled || !skeleton.data.HasStrandData())
    return;

  auto& strand_group = skeleton.data.strand_data->strand_group;
  const auto& sorted_list = skeleton.PeekSortedNodeList();

  // Compute max_root_distance for strand_radius_distribution lookup.
  float max_root_distance = 0.0f;
  for (const auto& node_handle : sorted_list) {
    const auto& node = skeleton.PeekNode(node_handle);
    max_root_distance = glm::max(max_root_distance, node.info.root_distance + node.info.length);
  }
  if (max_root_distance < glm::epsilon<float>())
    max_root_distance = 1.0f;

  for (const auto& node_handle : sorted_list) {
    auto& node = skeleton.RefNode(node_handle);
    if (!node.data.HasStrandData())
      continue;

    auto& strand_data = *node.data.strand_data;
    if (strand_data.particle_map.empty())
      continue;

    // Compute per-node strand_radius from the distribution mean (deterministic).
    // Using the full GetValue() (which calls gaussRand) would produce a different
    // random radius every frame, causing strands to visually pop in and out when
    // the random draw yields a near-zero or negative thickness.
    strand_data.strand_radius =
        glm::max(0.0f, params.strand_radius_distribution.mean.GetValue(node.info.root_distance / max_root_distance));

    // Node's local coordinate frame.
    const auto current_left = node.info.regulated_global_rotation * glm::vec3(1, 0, 0);
    const auto current_up = node.info.regulated_global_rotation * glm::vec3(0, 1, 0);
    const float radius = strand_data.strand_radius;
    const bool wound = node.IsEndNode();
    const auto current_front = wound ? node.info.regulated_global_rotation * glm::vec3(0, 0, -1) : glm::vec3(0);

    for (const auto& [strand_handle, particle_handle] : strand_data.particle_map) {
      const auto& particle = strand_data.profile.PeekParticle(particle_handle);
      auto& segment = strand_group.RefStrandSegment(particle.strand_segment_handle);
      auto& segment_data = strand_group.RefStrandSegmentData(particle.strand_segment_handle);
      segment.end_thickness = radius;

      // Use the stable post-packing snapshot (initial_position), not the live solver state.
      const glm::vec2 pos2d = particle.GetInitialPosition();

      glm::vec3 start_position;
      const auto prev_handle = segment.GetPrevHandle();
      if (prev_handle == -1) {
        start_position = node.info.global_position + (radius * pos2d.x * current_left + radius * pos2d.y * current_up);
        auto& strand = strand_group.RefStrand(segment.GetStrandHandle());
        strand.start_position = start_position;
        strand.start_thickness = radius;
        switch (particle.data.tissue_type) {
          case TissueType::kBarkLiving:  strand.start_color = params.boundary_point_color; break;
          case TissueType::kBarkDead:    strand.start_color = params.bark_dead_color;       break;
          case TissueType::kHeartwood:   strand.start_color = params.heartwood_color;       break;
          default:                       strand.start_color = params.content_point_color;   break;
        }
      } else {
        start_position = strand_group.RefStrandSegment(prev_handle).end_position;
      }

      segment.end_position =
          node.info.GetGlobalEndPosition() + (radius * pos2d.x * current_left + radius * pos2d.y * current_up);

      const auto direction = glm::normalize(segment.end_position - start_position);
      segment.rotation = glm::quatLookAt(direction, glm::vec3(direction.y, direction.z, direction.x));

      if (wound) {
        segment.end_position +=
            current_front *
            glm::max(0.0f, params.cladoptosis_distribution.GetValue(glm::max(
                               0.0f, (params.cladoptosis_range - particle.GetDistanceToBoundary()) /
                                         params.cladoptosis_range))) *
            .5f;
      }

      segment_data.initial_distance_to_boundary = particle.GetInitialDistanceToBoundary();
      segment_data.profile_position = pos2d;
      segment_data.is_boundary = particle.IsBoundary();

      // Copy biological metadata from the 2D particle to the 3D segment.
      segment_data.tissue_type = particle.data.tissue_type;
      segment_data.wound_state = particle.data.wound_state;

      // Color based on biological tissue type and wound state.
      glm::vec4 color;
      switch (particle.data.wound_state) {
        case WoundState::kWounded:
          color = params.wound_color;
          break;
        case WoundState::kHealing:
          color = params.callus_color;
          break;
        default:
          switch (particle.data.tissue_type) {
            case TissueType::kBarkLiving:  color = params.boundary_point_color; break;
            case TissueType::kBarkDead:    color = params.bark_dead_color;       break;
            case TissueType::kHeartwood:   color = params.heartwood_color;       break;
            default:                       color = params.content_point_color;   break;
          }
          break;
      }
      segment.end_color = color;
    }
  }

  strand_group.RegulateRotations();
}

std::shared_ptr<Strands> ProceduralStrandModel::GenerateStrands(const int node_max_count) const {
  if (!enabled || !skeleton.data.HasStrandData())
    return {};

  std::vector<glm::uint> strands_list;
  std::vector<StrandPoint> points;
  skeleton.data.strand_data->strand_group.BuildStrands(strands_list, points, node_max_count);
  if (strands_list.empty() || points.size() < 4)
    return {};

  if (points.size() > kMaxRenderableStrandPoints) {
    EVOENGINE_WARNING("ProceduralStrandModel::GenerateStrands skipped because generated strand point count " +
                      std::to_string(points.size()) + " exceeds safety limit " +
                      std::to_string(kMaxRenderableStrandPoints) + ".");
    return {};
  }

  strands_list.emplace_back(points.size());

  const auto strands_asset = AssetManager::CreateTemporaryAsset<Strands>();
  StrandPointAttributes strand_point_attributes{};
  strand_point_attributes.color = true;
  strands_asset->SetStrands(strand_point_attributes, strands_list, points);
  return strands_asset;
}

// ---------------------------------------------------------------------------
// Serialization
// ---------------------------------------------------------------------------

void ProceduralStrandModel::Save(const std::string& name, YAML::Emitter& out) const {
  out << YAML::Key << name << YAML::Value << YAML::BeginMap;
  {
    out << YAML::Key << "enabled" << YAML::Value << enabled;
    out << YAML::Key << "seed" << YAML::Value << seed;
    out << YAML::Key << "gpu_profile_packing" << YAML::Value << gpu_profile_packing;
    out << YAML::Key << "gpu_packing_iterations" << YAML::Value << gpu_packing_iterations;
    out << YAML::Key << "current_growth_step" << YAML::Value << current_growth_step_;
  }
  out << YAML::EndMap;
}

void ProceduralStrandModel::Load(const std::string& name, const YAML::Node& in) {
  if (in[name]) {
    const auto& node = in[name];
    if (node["enabled"])
      enabled = node["enabled"].as<bool>();
    if (node["seed"])
      seed = node["seed"].as<int>();
    if (node["gpu_profile_packing"])
      gpu_profile_packing = node["gpu_profile_packing"].as<bool>();
    if (node["gpu_packing_iterations"])
      gpu_packing_iterations = node["gpu_packing_iterations"].as<int>();
    if (node["current_growth_step"])
      current_growth_step_ = static_cast<uint16_t>(node["current_growth_step"].as<int>());
  }
}
