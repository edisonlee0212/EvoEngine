#include "RootDevelopmentalStrandModel.hpp"
#include <algorithm>
#include <unordered_map>
#include <unordered_set>

using namespace eco_sys_lab_plugin;

namespace {
constexpr uint32_t kMaxGpuPackingParticles = 300000;
constexpr size_t kMaxRenderableStrandPoints = 4000000;
}  // namespace

void RootDevelopmentalStrandModel::Enable(const RootSkeleton& root_skeleton, const StrandModelParameters& params) {
  enabled = true;
  current_growth_step_ = 0;
  random_engine_ = std::mt19937(static_cast<uint32_t>(seed));

  skeleton.Clone(root_skeleton);

  skeleton.data.InitializeStrandData();
  skeleton.data.strand_data->strand_group = {};

  const auto& sorted_list = skeleton.PeekSortedNodeList();
  for (const auto& node_handle : sorted_list) {
    auto& node = skeleton.RefNode(node_handle);
    node.data.root_node_data = root_skeleton.PeekNode(node_handle).data;
    node.info = root_skeleton.PeekNode(node_handle).info;
    node.data.InitializeStrandData();
    node.data.strand_data->profile.Reset(0.001f);
    node.data.strand_data->profile.particle_physics_settings = params.profile_physics_settings;
  }

  auto& strand_group = skeleton.data.strand_data->strand_group;
  for (const auto& node_handle : sorted_list) {
    const auto& node = skeleton.PeekNode(node_handle);
    if (!node.IsEndNode())
      continue;

    std::vector<SkeletonNodeHandle> root_to_node_chain;
    SkeletonNodeHandle walker = node_handle;
    while (walker >= 0) {
      root_to_node_chain.push_back(walker);
      walker = skeleton.PeekNode(walker).GetParentHandle();
    }
    std::reverse(root_to_node_chain.begin(), root_to_node_chain.end());

    const int strand_count = params.end_node_strands;
    const glm::vec3 end_node_position = skeleton.PeekNode(node_handle).info.global_position;

    for (int i = 0; i < strand_count; i++) {
      const auto strand_handle = strand_group.AllocateStrand();

      std::vector<std::pair<SkeletonNodeHandle, ParticleHandle>> strand_particles;
      strand_particles.reserve(root_to_node_chain.size());

      for (const auto chain_handle : root_to_node_chain) {
        auto& chain_node = skeleton.RefNode(chain_handle);
        auto& profile = chain_node.data.strand_data->profile;

        const auto new_segment_handle = strand_group.Extend(strand_handle);
        const auto new_particle_handle = profile.AllocateParticle();
        auto& particle = profile.RefParticle(new_particle_handle);
        particle.strand_handle = strand_handle;
        particle.strand_segment_handle = new_segment_handle;
        particle.base = (chain_handle == node_handle);
        particle.SetPosition(glm::vec2(0.f));

        auto& seg_data = strand_group.RefStrandSegmentData(new_segment_handle);
        seg_data.node_handle = chain_handle;
        seg_data.profile_particle_handle = new_particle_handle;

        strand_particles.emplace_back(chain_handle, new_particle_handle);
      }

      const auto chain_len = static_cast<int>(strand_particles.size());
      for (int k = chain_len - 1; k >= 0; --k) {
        const auto [chain_handle, p_handle] = strand_particles[k];
        auto& chain_node = skeleton.RefNode(chain_handle);
        auto& profile = chain_node.data.strand_data->profile;
        auto& particle = profile.RefParticle(p_handle);

        glm::vec2 pos(0.f);
        if (chain_handle == node_handle) {
          const int existing = static_cast<int>(profile.PeekParticles().size());
          const float tip_radius = glm::sqrt(static_cast<float>(glm::max(1, existing)));
          pos = (existing <= 1) ? glm::vec2(0.f) : Random::Disk(random_engine_, tip_radius);
        } else if (k == chain_len - 2) {
          const glm::quat anc_rot = chain_node.info.regulated_global_rotation;
          const glm::vec3 anc_fwd = glm::normalize(anc_rot * glm::vec3(0, 0, -1));
          const glm::vec3 anc_left = glm::normalize(anc_rot * glm::vec3(1, 0, 0));
          const glm::vec3 anc_up = glm::normalize(anc_rot * glm::vec3(0, 1, 0));
          const glm::vec3 to_branch = end_node_position - chain_node.info.global_position;
          const glm::vec3 in_plane = to_branch - glm::dot(to_branch, anc_fwd) * anc_fwd;

          if (glm::length(in_plane) > 1e-6f) {
            const glm::vec3 norm_dir = glm::normalize(in_plane);
            const glm::vec2 branch_dir_2d(glm::dot(norm_dir, anc_left), glm::dot(norm_dir, anc_up));
            const float boundary_radius = profile.GetDistanceToOrigin(branch_dir_2d, glm::vec2(0.f));
            pos = (boundary_radius + 1.0f) * glm::normalize(branch_dir_2d);
          } else {
            const int existing = static_cast<int>(profile.PeekParticles().size());
            pos = Random::Disk(random_engine_, glm::sqrt(static_cast<float>(glm::max(1, existing))));
          }
        } else {
          const auto [child_handle, child_p_handle] = strand_particles[k + 1];
          const auto& child_node = skeleton.PeekNode(child_handle);
          pos = child_node.data.strand_data->profile.PeekParticle(child_p_handle).GetPosition();
        }

        particle.SetPosition(pos);
        particle.SetInitialPosition(pos);
      }
    }
  }

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

  if (gpu_profile_packing) {
    if (skeleton.data.strand_data->num_of_particles > static_cast<int>(kMaxGpuPackingParticles)) {
      EVOENGINE_WARNING(
          "RootDevelopmentalStrandModel::Enable skipped GPU profile packing due to particle count " +
          std::to_string(skeleton.data.strand_data->num_of_particles));
    } else {
      std::vector<SkeletonNodeHandle> all_nodes(sorted_list.begin(), sorted_list.end());
      gpu_profile_simulator.UploadFromSkeleton(skeleton, all_nodes);
      gpu_profile_simulator.Simulate(static_cast<uint32_t>(gpu_packing_iterations), params);
      gpu_profile_simulator.DownloadToSkeleton(skeleton);

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

      gpu_profile_simulator.UploadFromSkeleton(skeleton, all_nodes);
      gpu_profile_simulator.Simulate(static_cast<uint32_t>(gpu_packing_iterations), params);
      gpu_profile_simulator.DownloadToSkeleton(skeleton);
    }
  }

  SyncInitialPositions();
  FreezeInteriorParticles();
}

void RootDevelopmentalStrandModel::Disable() {
  enabled = false;
  skeleton.data.strand_data.reset();
  auto& raw_nodes = skeleton.RefRawNodes();
  for (auto& node : raw_nodes) {
    node.data.strand_data.reset();
  }
}

void RootDevelopmentalStrandModel::Reset() {
  enabled = false;
  seed = 0;
  skeleton = {};
}

// ---------------------------------------------------------------------------
// Incremental strand growth methods
// ---------------------------------------------------------------------------

void RootDevelopmentalStrandModel::InitializeStrandsForNewNode(const SkeletonNodeHandle new_node_handle,
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

  new_strand_data.profile.particle_physics_settings = parent_strand_data.profile.particle_physics_settings;

  for (const auto& [strand_handle, parent_particle_handle] : parent_strand_data.particle_map) {
    if (strand_group.PeekStrandData(strand_handle).growth_locked)
      continue;

    const auto& parent_particle = parent_strand_data.profile.PeekParticle(parent_particle_handle);

    const auto new_segment_handle = strand_group.Extend(strand_handle);
    const auto new_particle_handle = new_strand_data.profile.AllocateParticle();
    auto& new_particle = new_strand_data.profile.RefParticle(new_particle_handle);
    new_particle.strand_handle = strand_handle;
    new_particle.strand_segment_handle = new_segment_handle;
    new_particle.base = false;
    new_particle.data.birth_step = current_growth_step_;
    new_particle.SetPosition(parent_particle.GetPosition());
    new_particle.SetInitialPosition(parent_particle.GetInitialPosition());

    auto& seg_data = strand_group.RefStrandSegmentData(new_segment_handle);
    seg_data.node_handle = new_node_handle;
    seg_data.profile_particle_handle = new_particle_handle;

    new_strand_data.particle_map.insert({strand_handle, new_particle_handle});
  }

  new_strand_data.strand_count = static_cast<int>(new_strand_data.profile.PeekParticles().size());
}

void RootDevelopmentalStrandModel::SplitProfileForBranching(const SkeletonNodeHandle new_branch_handle,
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

  branch_strand_data.profile.particle_physics_settings = parent_strand_data.profile.particle_physics_settings;

  const glm::vec3 parent_forward = glm::normalize(parent_global_rotation * glm::vec3(0, 0, -1));
  const glm::vec3 parent_up = glm::normalize(parent_global_rotation * glm::vec3(0, 1, 0));
  const glm::vec3 parent_left = glm::normalize(parent_global_rotation * glm::vec3(1, 0, 0));
  const glm::vec3 branch_forward = glm::normalize(branch_global_rotation * glm::vec3(0, 0, -1));

  const glm::vec3 branch_in_plane = branch_forward - glm::dot(branch_forward, parent_forward) * parent_forward;
  glm::vec2 branch_dir_2d(0.f);
  if (glm::length(branch_in_plane) > 1e-6f) {
    const glm::vec3 normalized = glm::normalize(branch_in_plane);
    branch_dir_2d = glm::vec2(glm::dot(normalized, parent_left), glm::dot(normalized, parent_up));
  }

  const int total_particles = static_cast<int>(parent_strand_data.profile.PeekParticles().size());

  std::vector<std::pair<StrandHandle, ParticleHandle>> branch_strands;
  for (const auto& [strand_handle, particle_handle] : parent_strand_data.particle_map) {
    if (strand_group.PeekStrandData(strand_handle).growth_locked)
      continue;

    const auto& particle = parent_strand_data.profile.PeekParticle(particle_handle);
    const glm::vec2 pos = particle.GetPosition();
    const float alignment = glm::dot(pos, branch_dir_2d);

    if (alignment > 0.0f || total_particles <= 1) {
      branch_strands.emplace_back(strand_handle, particle_handle);
    }
  }

  if (branch_strands.empty() && !parent_strand_data.particle_map.empty()) {
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

  SkeletonNodeHandle prolongation_handle = -1;
  for (const auto child_h : skeleton.PeekNode(parent_handle).PeekChildHandles()) {
    if (child_h != new_branch_handle) {
      prolongation_handle = child_h;
      break;
    }
  }

  for (const auto& [strand_handle, parent_particle_handle] : branch_strands) {
    const auto& parent_particle = parent_strand_data.profile.PeekParticle(parent_particle_handle);

    StrandSegmentHandle seg_handle = -1;

    if (prolongation_handle >= 0) {
      auto& prolongation_node = skeleton.RefNode(prolongation_handle);
      if (prolongation_node.data.HasStrandData()) {
        auto& prolongation_strand_data = *prolongation_node.data.strand_data;
        auto it = prolongation_strand_data.particle_map.find(strand_handle);
        if (it != prolongation_strand_data.particle_map.end()) {
          const auto prolongation_particle_handle = it->second;
          seg_handle = prolongation_strand_data.profile.PeekParticle(prolongation_particle_handle)
                           .strand_segment_handle;
          prolongation_strand_data.particle_map.erase(it);
          prolongation_strand_data.strand_count =
              static_cast<int>(prolongation_strand_data.particle_map.size());
        }
      }
    }

    if (seg_handle < 0) {
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

void RootDevelopmentalStrandModel::AllocateNewStrandsForEndNode(const SkeletonNodeHandle end_node_handle,
                                                                const StrandModelParameters& params) {
  AllocateStrandsForEndNode(end_node_handle, params.end_node_strands, params);
}

void RootDevelopmentalStrandModel::AllocateStrandsForEndNode(const SkeletonNodeHandle end_node_handle,
                                                             const int strand_count,
                                                             const StrandModelParameters& params) {
  auto& strand_group = skeleton.data.strand_data->strand_group;

  std::vector<SkeletonNodeHandle> root_to_node_chain;
  SkeletonNodeHandle walker = end_node_handle;
  while (walker >= 0) {
    root_to_node_chain.push_back(walker);
    walker = skeleton.PeekNode(walker).GetParentHandle();
  }
  std::reverse(root_to_node_chain.begin(), root_to_node_chain.end());

  const glm::vec3 branch_position = skeleton.PeekNode(end_node_handle).info.global_position;

  for (int i = 0; i < strand_count; i++) {
    const auto strand_handle = strand_group.AllocateStrand();

    std::vector<std::pair<SkeletonNodeHandle, ParticleHandle>> strand_particles;
    strand_particles.reserve(root_to_node_chain.size());

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
      particle.SetPosition(glm::vec2(0.f));

      auto& seg_data = strand_group.RefStrandSegmentData(new_segment_handle);
      seg_data.node_handle = chain_handle;
      seg_data.profile_particle_handle = new_particle_handle;

      strand_data.particle_map.insert({strand_handle, new_particle_handle});
      strand_data.strand_count = static_cast<int>(strand_data.particle_map.size());

      strand_particles.emplace_back(chain_handle, new_particle_handle);
    }

    const auto chain_len = static_cast<int>(strand_particles.size());
    for (int k = chain_len - 1; k >= 0; --k) {
      const auto [chain_handle, p_handle] = strand_particles[k];
      auto& chain_node = skeleton.RefNode(chain_handle);
      auto& profile = chain_node.data.strand_data->profile;
      auto& particle = profile.RefParticle(p_handle);

      glm::vec2 pos(0.f);
      if (chain_handle == end_node_handle) {
        const int existing = static_cast<int>(chain_node.data.strand_data->particle_map.size());
        const float tip_radius = glm::sqrt(static_cast<float>(glm::max(1, existing)));
        pos = (existing <= 1) ? glm::vec2(0.f) : Random::Disk(random_engine_, tip_radius);
      } else if (k == chain_len - 2) {
        const glm::quat anc_rot = chain_node.info.regulated_global_rotation;
        const glm::vec3 anc_fwd = glm::normalize(anc_rot * glm::vec3(0, 0, -1));
        const glm::vec3 anc_left = glm::normalize(anc_rot * glm::vec3(1, 0, 0));
        const glm::vec3 anc_up = glm::normalize(anc_rot * glm::vec3(0, 1, 0));
        const glm::vec3 to_branch = branch_position - chain_node.info.global_position;
        const glm::vec3 in_plane = to_branch - glm::dot(to_branch, anc_fwd) * anc_fwd;

        if (glm::length(in_plane) > 1e-6f) {
          const glm::vec3 norm_dir = glm::normalize(in_plane);
          const glm::vec2 branch_dir_2d(glm::dot(norm_dir, anc_left), glm::dot(norm_dir, anc_up));
          const float boundary_radius = profile.GetDistanceToOrigin(branch_dir_2d, glm::vec2(0.f));
          pos = (boundary_radius + 1.0f) * glm::normalize(branch_dir_2d);
        } else {
          const int existing = static_cast<int>(chain_node.data.strand_data->particle_map.size());
          pos = Random::Disk(random_engine_, glm::sqrt(static_cast<float>(glm::max(1, existing))));
        }
      } else {
        const auto [child_handle, child_p_handle] = strand_particles[k + 1];
        const auto& child_node = skeleton.PeekNode(child_handle);
        pos = child_node.data.strand_data->profile.PeekParticle(child_p_handle).GetPosition();
      }

      particle.SetPosition(pos);
      particle.SetInitialPosition(pos);
    }
  }
}

// ---------------------------------------------------------------------------
// Main growth step hook
// ---------------------------------------------------------------------------

void RootDevelopmentalStrandModel::OnGrowthStep(
    const RootSkeleton& root_skeleton, const std::vector<RootModel::GrowthEvent>& growth_events,
    const std::vector<std::vector<SkeletonNodeHandle>>& pruning_event_batches, const bool pruning_occurred,
    const StrandModelParameters& params) {
  if (!enabled)
    return;

  ++current_growth_step_;

  if (!skeleton.data.HasStrandData()) {
    skeleton.data.InitializeStrandData();
    skeleton.data.strand_data->strand_group = {};
  }

  if (pruning_occurred) {
    if (!pruning_event_batches.empty()) {
      ApplyPruningEvents(root_skeleton, pruning_event_batches, params);
    } else {
      ResyncAfterPruning(root_skeleton, params);
    }
    return;
  }

  const auto root_node_count = static_cast<int>(root_skeleton.PeekRawNodes().size());
  for (const auto& event : growth_events) {
    if (event.new_handle < 0 || event.new_handle >= root_node_count) {
      ResyncAfterPruning(root_skeleton, params);
      return;
    }

    const auto new_handle = skeleton.Extend(event.parent_handle, event.branching);
    if (new_handle != event.new_handle || new_handle < 0 || new_handle >= root_node_count) {
      ResyncAfterPruning(root_skeleton, params);
      return;
    }

    auto& new_node = skeleton.RefNode(new_handle);
    const auto& src_node = root_skeleton.PeekNode(new_handle);
    new_node.info = src_node.info;
    new_node.data.root_node_data = src_node.data;

    if (event.branching) {
      AllocateNewStrandsForEndNode(new_handle, params);
    } else {
      InitializeStrandsForNewNode(new_handle, event.parent_handle);
    }
  }

  if (!growth_events.empty()) {
    skeleton.SortLists();
  }

  SyncNodeInfo(root_skeleton);

  if (gpu_profile_packing && !growth_events.empty()) {
    std::unordered_set<SkeletonNodeHandle> active_set;
    for (const auto& event : growth_events) {
      active_set.insert(event.new_handle);
      active_set.insert(event.parent_handle);
      if (event.branching) {
        SkeletonNodeHandle w = event.parent_handle;
        while (w >= 0) {
          active_set.insert(w);
          w = skeleton.PeekNode(w).GetParentHandle();
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
          "RootDevelopmentalStrandModel::OnGrowthStep skipped GPU profile packing due to active particle count " +
          std::to_string(active_particle_count));
    } else {
      gpu_profile_simulator.UploadFromSkeleton(skeleton, active_nodes, true);
      gpu_profile_simulator.Simulate(static_cast<uint32_t>(gpu_packing_iterations), params);
      gpu_profile_simulator.DownloadToSkeleton(skeleton);
    }
  }

  SyncInitialPositions();
  FreezeInteriorParticles();
  ProcessWounds(params);
}

// ---------------------------------------------------------------------------
// Pruning
// ---------------------------------------------------------------------------

void RootDevelopmentalStrandModel::ApplyPruningEvents(
    const RootSkeleton& root_skeleton, const std::vector<std::vector<SkeletonNodeHandle>>& pruning_event_batches,
    const StrandModelParameters& params) {
  if (!enabled || !skeleton.data.HasStrandData())
    return;

  if (pruning_event_batches.empty()) {
    ResyncAfterPruning(root_skeleton, params);
    return;
  }

  auto& strand_group = skeleton.data.strand_data->strand_group;

  auto rebuild_particle_maps = [&]() {
    skeleton.data.strand_data->num_of_particles = 0;
    for (const auto node_handle : skeleton.PeekSortedNodeList()) {
      auto& node = skeleton.RefNode(node_handle);
      if (!node.data.HasStrandData())
        continue;
      auto& strand_data = *node.data.strand_data;
      auto validated_map = std::unordered_map<StrandHandle, ParticleHandle>{};
      validated_map.reserve(strand_data.particle_map.size());

      const auto& particles = strand_data.profile.PeekParticles();
      const auto s_count = static_cast<int>(strand_group.PeekStrands().size());
      const auto seg_count = static_cast<int>(strand_group.PeekStrandSegments().size());

      for (const auto& [strand_handle, particle_handle] : strand_data.particle_map) {
        if (strand_handle < 0 || strand_handle >= s_count)
          continue;
        if (particle_handle < 0 || particle_handle >= static_cast<int>(particles.size()))
          continue;

        const auto& particle = particles[particle_handle];
        if (particle.strand_handle != strand_handle)
          continue;

        const auto segment_handle = particle.strand_segment_handle;
        if (segment_handle < 0 || segment_handle >= seg_count)
          continue;
        if (strand_group.PeekStrandSegment(segment_handle).GetStrandHandle() != strand_handle)
          continue;

        auto& seg_data = strand_group.RefStrandSegmentData(segment_handle);
        seg_data.node_handle = node_handle;
        seg_data.profile_particle_handle = particle_handle;

        validated_map[strand_handle] = particle_handle;
      }
      strand_data.particle_map = std::move(validated_map);
      strand_data.strand_count = static_cast<int>(strand_data.particle_map.size());
      skeleton.data.strand_data->num_of_particles += static_cast<int>(strand_data.profile.PeekParticles().size());
    }
  };

  for (const auto& pruning_roots : pruning_event_batches) {
    if (pruning_roots.empty())
      continue;

    std::vector<SkeletonNodeHandle> valid_pruning_roots;
    valid_pruning_roots.reserve(pruning_roots.size());
    std::unordered_set<SkeletonNodeHandle> unique_roots;
    std::unordered_set<StrandHandle> lock_candidates;

    for (const auto pruning_root : pruning_roots) {
      if (pruning_root < 0 || pruning_root >= static_cast<int>(skeleton.PeekRawNodes().size()))
        continue;
      if (!unique_roots.insert(pruning_root).second)
        continue;
      valid_pruning_roots.push_back(pruning_root);

      const auto& cut_node = skeleton.PeekNode(pruning_root);
      if (!cut_node.data.HasStrandData())
        continue;

      std::vector<StrandHandle> cut_strands;
      cut_strands.reserve(cut_node.data.strand_data->particle_map.size());
      for (const auto& [strand_handle, _] : cut_node.data.strand_data->particle_map) {
        cut_strands.push_back(strand_handle);
      }

      for (const auto strand_handle : cut_strands) {
        if (strand_handle < 0 || strand_handle >= static_cast<int>(strand_group.PeekStrands().size()))
          continue;

        StrandSegmentHandle cut_segment_handle = -1;
        for (const auto seg_handle : strand_group.PeekStrand(strand_handle).PeekStrandSegmentHandles()) {
          if (seg_handle < 0 || seg_handle >= static_cast<int>(strand_group.PeekStrandSegmentDataList().size()))
            continue;
          if (strand_group.PeekStrandSegmentData(seg_handle).node_handle == pruning_root) {
            cut_segment_handle = seg_handle;
            break;
          }
        }
        if (cut_segment_handle < 0)
          continue;

        strand_group.CutLatter(cut_segment_handle);
        if (strand_group.PeekStrandSegment(cut_segment_handle).GetPrevHandle() >= 0) {
          lock_candidates.insert(strand_handle);
        }
      }
    }

    for (const auto strand_handle : lock_candidates) {
      if (strand_handle >= 0 && strand_handle < static_cast<int>(strand_group.PeekStrands().size())) {
        strand_group.RefStrandData(strand_handle).growth_locked = true;
      }
    }

    if (valid_pruning_roots.empty())
      continue;

    skeleton.RemoveNodes(valid_pruning_roots);
    skeleton.SortLists();
    rebuild_particle_maps();
  }

  if (skeleton.PeekRawNodes().size() != root_skeleton.PeekRawNodes().size()) {
    ResyncAfterPruning(root_skeleton, params);
    return;
  }

  for (const auto node_handle : skeleton.PeekSortedNodeList()) {
    auto& node = skeleton.RefNode(node_handle);
    const auto& src_node = root_skeleton.PeekNode(node_handle);
    node.info = src_node.info;
    node.data.root_node_data = src_node.data;
  }

  // Roots don't have buds — wound cap detection is simpler.
  // Mark force_sink nodes as wound profiles.
  std::vector<StrandHandle> cap_strands_to_remove;
  for (const auto node_handle : skeleton.PeekSortedNodeList()) {
    auto& node = skeleton.RefNode(node_handle);
    if (!node.data.HasStrandData())
      continue;
    auto& strand_data = *node.data.strand_data;
    strand_data.is_wound_profile = false;
    strand_data.wound_cap_ring_particle_handles.clear();
    if (strand_data.wound_cap_strand_handle >= 0) {
      cap_strands_to_remove.push_back(strand_data.wound_cap_strand_handle);
      strand_data.wound_cap_strand_handle = -1;
    }
  }
  std::sort(cap_strands_to_remove.begin(), cap_strands_to_remove.end());
  cap_strands_to_remove.erase(std::unique(cap_strands_to_remove.begin(), cap_strands_to_remove.end()),
                              cap_strands_to_remove.end());
  for (auto it = cap_strands_to_remove.rbegin(); it != cap_strands_to_remove.rend(); ++it) {
    if (*it >= 0 && *it < static_cast<int>(strand_group.PeekStrands().size())) {
      const auto& cap_segments = strand_group.PeekStrand(*it).PeekStrandSegmentHandles();
      if (!cap_segments.empty()) {
        strand_group.CutLatter(cap_segments.front());
      }
    }
  }

  rebuild_particle_maps();
}

void RootDevelopmentalStrandModel::ResyncAfterPruning(const RootSkeleton& root_skeleton,
                                                      const StrandModelParameters& params) {
  auto make_pos_key = [](const glm::vec3& p) -> std::string {
    const auto qx = static_cast<int>(glm::round(p.x * 10000.0f));
    const auto qy = static_cast<int>(glm::round(p.y * 10000.0f));
    const auto qz = static_cast<int>(glm::round(p.z * 10000.0f));
    return std::to_string(qx) + "|" + std::to_string(qy) + "|" + std::to_string(qz);
  };
  std::unordered_map<std::string, int> pre_prune_strand_count_by_pos;
  if (skeleton.data.HasStrandData()) {
    const auto& old_sorted = skeleton.PeekSortedNodeList();
    for (const auto node_handle : old_sorted) {
      const auto& node = skeleton.PeekNode(node_handle);
      if (!node.data.HasStrandData())
        continue;
      const auto key = make_pos_key(node.info.global_position);
      const int count = static_cast<int>(node.data.strand_data->particle_map.size());
      const auto it = pre_prune_strand_count_by_pos.find(key);
      if (it == pre_prune_strand_count_by_pos.end())
        pre_prune_strand_count_by_pos[key] = count;
      else
        it->second = glm::max(it->second, count);
    }
  }

  Enable(root_skeleton, params);

  // Roots have no buds — wound detection only checks end nodes that previously had children.
  const auto& sorted_list = skeleton.PeekSortedNodeList();
  for (const auto& node_handle : sorted_list) {
    auto& node = skeleton.RefNode(node_handle);
    if (!node.data.HasStrandData())
      continue;

    if (!node.IsEndNode())
      continue;

    const auto key = make_pos_key(node.info.global_position);
    const auto it = pre_prune_strand_count_by_pos.find(key);
    if (it == pre_prune_strand_count_by_pos.end())
      continue;

    auto& strand_data = *node.data.strand_data;
    const int current_count = static_cast<int>(strand_data.particle_map.size());
    if (it->second > current_count) {
      AllocateStrandsForEndNode(node_handle, it->second - current_count, params);
    }

    strand_data.is_wound_profile = true;

    auto& profile = strand_data.profile;
    profile.CalculateBoundaries(false);
    for (auto& particle : profile.RefParticles()) {
      if (particle.status == ParticleStatus::kDisabled)
        continue;
      if (particle.IsBoundary()) {
        particle.data.wound_state = WoundState::kWounded;
      }
    }
  }

  skeleton.data.strand_data->num_of_particles = 0;
  for (const auto node_handle : skeleton.PeekSortedNodeList()) {
    const auto& node = skeleton.PeekNode(node_handle);
    if (!node.data.HasStrandData())
      continue;
    skeleton.data.strand_data->num_of_particles +=
        static_cast<int>(node.data.strand_data->profile.PeekParticles().size());
  }
}

// ---------------------------------------------------------------------------
// Utility methods
// ---------------------------------------------------------------------------

void RootDevelopmentalStrandModel::SyncInitialPositions() {
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

void RootDevelopmentalStrandModel::FreezeInteriorParticles() {
  const auto& sorted_list = skeleton.PeekSortedNodeList();
  for (const auto& node_handle : sorted_list) {
    auto& node = skeleton.RefNode(node_handle);
    if (!node.data.HasStrandData())
      continue;

    auto& profile = node.data.strand_data->profile;
    auto& particles = profile.RefParticles();
    if (particles.size() < 3)
      continue;

    profile.CalculateBoundaries(false);

    for (auto& particle : particles) {
      if (particle.status == ParticleStatus::kDisabled)
        continue;
      if (particle.IsBoundary()) {
        particle.status = ParticleStatus::kActive;
        particle.data.tissue_type = TissueType::kBarkLiving;
      } else {
        particle.status = ParticleStatus::kFrozen;
        if (particle.data.tissue_type == TissueType::kBarkLiving ||
            particle.data.tissue_type == TissueType::kBarkDead) {
          particle.data.tissue_type = TissueType::kSapwood;
        }
      }
    }
  }
}

void RootDevelopmentalStrandModel::ProcessWounds(const StrandModelParameters& params) {
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
          particle.data.wound_state = WoundState::kHealing;
          break;
        case WoundState::kHealing: {
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

void RootDevelopmentalStrandModel::SyncNodeInfo(const RootSkeleton& root_skeleton) {
  const auto& sorted_list = skeleton.PeekSortedNodeList();
  for (const auto& node_handle : sorted_list) {
    auto& node = skeleton.RefNode(node_handle);
    if (node_handle < static_cast<int>(root_skeleton.PeekRawNodes().size())) {
      node.info = root_skeleton.PeekNode(node_handle).info;
      node.data.root_node_data = root_skeleton.PeekNode(node_handle).data;
    }
  }
}

// ---------------------------------------------------------------------------
// 2D-to-3D profile application
// ---------------------------------------------------------------------------

void RootDevelopmentalStrandModel::ApplyProfiles(const StrandModelParameters& params) {
  if (!enabled || !skeleton.data.HasStrandData())
    return;

  auto& strand_group = skeleton.data.strand_data->strand_group;
  const auto& sorted_list = skeleton.PeekSortedNodeList();

  const bool age_mode = (strand_color_mode == StrandColorMode::kAge);
  const float max_step = glm::max(static_cast<float>(current_growth_step_), 1.0f);

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

    strand_data.strand_radius =
        glm::max(0.0f, params.strand_radius_distribution.mean.GetValue(node.info.root_distance / max_root_distance));

    const auto current_left = node.info.regulated_global_rotation * glm::vec3(1, 0, 0);
    const auto current_up = node.info.regulated_global_rotation * glm::vec3(0, 1, 0);
    const float radius = strand_data.strand_radius;

    // Roots have no cladoptosis — skip the wound tip offset logic.
    const auto strand_count_local = static_cast<int>(strand_data.particle_map.size());
    StrandAgeCategory age_cat;
    if (strand_count_local < params.light_strand_threshold)
      age_cat = StrandAgeCategory::kYoung;
    else if (strand_count_local < params.medium_strand_threshold)
      age_cat = StrandAgeCategory::kAdolescent;
    else
      age_cat = StrandAgeCategory::kOld;

    const float age_alpha = static_cast<float>(static_cast<uint8_t>(age_cat)) * 0.5f;

    for (const auto& [strand_handle, particle_handle] : strand_data.particle_map) {
      const auto& particle = strand_data.profile.PeekParticle(particle_handle);
      auto& segment = strand_group.RefStrandSegment(particle.strand_segment_handle);
      auto& segment_data = strand_group.RefStrandSegmentData(particle.strand_segment_handle);
      segment.end_thickness = radius;

      const glm::vec2 pos2d = particle.GetInitialPosition();

      glm::vec3 start_position;
      const auto prev_handle = segment.GetPrevHandle();
      if (prev_handle == -1) {
        start_position = node.info.global_position + (radius * pos2d.x * current_left + radius * pos2d.y * current_up);
        auto& strand = strand_group.RefStrand(segment.GetStrandHandle());
        strand.start_position = start_position;
        strand.start_thickness = radius;
        if (age_mode) {
          const float t = 1.0f - static_cast<float>(particle.data.birth_step) / max_step;
          strand.start_color = glm::vec4(t, t, t, age_alpha);
        } else {
          switch (particle.data.tissue_type) {
            case TissueType::kBarkLiving:  strand.start_color = params.boundary_point_color; break;
            case TissueType::kBarkDead:    strand.start_color = params.bark_dead_color;       break;
            case TissueType::kHeartwood:   strand.start_color = params.heartwood_color;       break;
            default:                       strand.start_color = params.content_point_color;   break;
          }
          strand.start_color.a = age_alpha;
        }
      } else {
        start_position = strand_group.RefStrandSegment(prev_handle).end_position;
      }

      segment.end_position =
          node.info.GetGlobalEndPosition() + (radius * pos2d.x * current_left + radius * pos2d.y * current_up);

      const auto direction = glm::normalize(segment.end_position - start_position);
      segment.rotation = glm::quatLookAt(direction, glm::vec3(direction.y, direction.z, direction.x));

      segment_data.initial_distance_to_boundary = particle.GetInitialDistanceToBoundary();
      segment_data.profile_position = pos2d;
      segment_data.is_boundary = particle.IsBoundary();
      segment_data.tissue_type = particle.data.tissue_type;
      segment_data.wound_state = particle.data.wound_state;
      segment_data.age_category = age_cat;

      if (age_mode) {
        const float t = 1.0f - static_cast<float>(particle.data.birth_step) / max_step;
        segment.end_color = glm::vec4(t, t, t, age_alpha);
      } else {
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
        segment.end_color.a = age_alpha;
      }
    }
  }

  strand_group.RegulateRotations();
}

std::shared_ptr<Strands> RootDevelopmentalStrandModel::GenerateStrands(const int node_max_count) const {
  if (!enabled || !skeleton.data.HasStrandData())
    return {};

  std::vector<glm::uint> strands_list;
  std::vector<StrandPoint> points;
  skeleton.data.strand_data->strand_group.BuildStrands(strands_list, points, node_max_count);
  if (strands_list.empty() || points.size() < 4)
    return {};

  if (points.size() > kMaxRenderableStrandPoints) {
    EVOENGINE_WARNING("RootDevelopmentalStrandModel::GenerateStrands skipped because point count " +
                      std::to_string(points.size()) + " exceeds limit " +
                      std::to_string(kMaxRenderableStrandPoints));
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

void RootDevelopmentalStrandModel::Save(const std::string& name, YAML::Emitter& out) const {
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

void RootDevelopmentalStrandModel::Load(const std::string& name, const YAML::Node& in) {
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
