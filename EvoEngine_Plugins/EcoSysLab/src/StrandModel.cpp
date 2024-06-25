#include "StrandModel.hpp"

#include <any>
using namespace evo_engine;
using namespace eco_sys_lab;

void StrandModel::ResetAllProfiles(const StrandModelParameters& strand_model_parameters) {
  strand_model_skeleton.data.strand_group = {};
  for (auto& internode : strand_model_skeleton.RefRawNodes()) {
    auto& profile = internode.data.profile;
    profile.particle_physics_settings = strand_model_parameters.profile_physics_settings;
    profile.Reset(0.001f);
    internode.data.particle_map.clear();
    internode.data.strand_count = 0;
  }
}

void StrandModel::InitializeProfiles(const StrandModelParameters& strand_model_parameters) {
  strand_model_skeleton.data.strand_group = {};
  auto& strand_group = strand_model_skeleton.data.strand_group;
  const auto& sorted_internode_list = strand_model_skeleton.PeekSortedNodeList();
  for (const auto& internode_handle : sorted_internode_list) {
    auto& internode = strand_model_skeleton.RefNode(internode_handle);
    auto& profile = internode.data.profile;
    profile.particle_physics_settings = strand_model_parameters.profile_physics_settings;
    if (internode.IsEndNode())
      continue;
    profile.Reset(0.001f);
  }

  for (const auto& internode_handle : sorted_internode_list) {
    auto& internode = strand_model_skeleton.RefNode(internode_handle);
    auto& profile = internode.data.profile;

    std::vector<SkeletonNodeHandle> parent_node_to_root_chain;
    SkeletonNodeHandle walker = internode.GetParentHandle();
    while (walker >= 0) {
      parent_node_to_root_chain.emplace_back(walker);
      walker = strand_model_skeleton.PeekNode(walker).GetParentHandle();
    }
    if (!internode.IsEndNode()) {
      for (int i = 0; i < strand_model_parameters.strands_along_branch; i++) {
        const auto strand_handle = strand_group.AllocateStrand();
        for (auto it = parent_node_to_root_chain.rbegin(); it != parent_node_to_root_chain.rend(); ++it) {
          const auto new_strand_segment_handle = strand_group.Extend(strand_handle);
          auto& node_on_chain = strand_model_skeleton.RefNode(*it);
          const auto new_particle_handle = node_on_chain.data.profile.AllocateParticle();
          auto& new_particle = node_on_chain.data.profile.RefParticle(new_particle_handle);
          new_particle.strand_handle = strand_handle;
          new_particle.strand_segment_handle = new_strand_segment_handle;
          new_particle.base = false;

          auto& new_segment = strand_group.RefStrandSegment(new_strand_segment_handle);
          new_segment.data.node_handle = *it;
          new_segment.data.profile_particle_handle = new_particle_handle;
        }
        constexpr auto position = glm::vec3(0.0f);
        const auto new_strand_segment_handle = strand_group.Extend(strand_handle);
        const auto new_particle_handle = profile.AllocateParticle();
        auto& new_particle = profile.RefParticle(new_particle_handle);
        new_particle.strand_handle = strand_handle;
        new_particle.strand_segment_handle = new_strand_segment_handle;
        new_particle.base = true;
        new_particle.SetPosition(position);
        new_particle.SetInitialPosition(position);
        auto& new_segment = strand_group.RefStrandSegment(new_strand_segment_handle);
        new_segment.data.node_handle = internode_handle;
        new_segment.data.profile_particle_handle = new_particle_handle;
      }
    } else {
      if (true || profile.RefParticles().empty()) {
        for (int i = 0; i < strand_model_parameters.end_node_strands; i++) {
          const auto strand_handle = strand_group.AllocateStrand();
          for (auto it = parent_node_to_root_chain.rbegin(); it != parent_node_to_root_chain.rend(); ++it) {
            const auto new_strand_segment_handle = strand_group.Extend(strand_handle);
            auto& node_on_chain = strand_model_skeleton.RefNode(*it);
            const auto new_particle_handle = node_on_chain.data.profile.AllocateParticle();
            auto& new_particle = node_on_chain.data.profile.RefParticle(new_particle_handle);
            new_particle.strand_handle = strand_handle;
            new_particle.strand_segment_handle = new_strand_segment_handle;
            new_particle.base = false;
            auto& new_segment = strand_group.RefStrandSegment(new_strand_segment_handle);
            new_segment.data.node_handle = *it;
            new_segment.data.profile_particle_handle = new_particle_handle;
          }
          const auto position =
              strand_model_parameters.end_node_strands == 1
                  ? glm::vec2(0.f)
                  : glm::diskRand(glm::sqrt(static_cast<float>(strand_model_parameters.end_node_strands)));
          const auto new_strand_segment_handle = strand_group.Extend(strand_handle);
          const auto new_particle_handle = profile.AllocateParticle();
          auto& new_particle = profile.RefParticle(new_particle_handle);
          new_particle.strand_handle = strand_handle;
          new_particle.strand_segment_handle = new_strand_segment_handle;
          new_particle.base = true;
          new_particle.SetPosition(position);
          new_particle.SetInitialPosition(position);
          auto& new_segment = strand_group.RefStrandSegment(new_strand_segment_handle);
          new_segment.data.node_handle = internode_handle;
          new_segment.data.profile_particle_handle = new_particle_handle;
        }
      } else {
        for (ParticleHandle particle_handle = 0; particle_handle < profile.RefParticles().size(); particle_handle++) {
          const auto strand_handle = strand_group.AllocateStrand();
          for (auto it = parent_node_to_root_chain.rbegin(); it != parent_node_to_root_chain.rend(); ++it) {
            const auto new_strand_segment_handle = strand_group.Extend(strand_handle);
            auto& node_on_chain = strand_model_skeleton.RefNode(*it);
            const auto new_particle_handle = node_on_chain.data.profile.AllocateParticle();
            auto& new_particle = node_on_chain.data.profile.RefParticle(new_particle_handle);
            new_particle.strand_handle = strand_handle;
            new_particle.strand_segment_handle = new_strand_segment_handle;
            new_particle.base = false;
            auto& new_segment = strand_group.RefStrandSegment(new_strand_segment_handle);
            new_segment.data.node_handle = *it;
            new_segment.data.profile_particle_handle = new_particle_handle;
          }
          const auto new_strand_segment_handle = strand_group.Extend(strand_handle);
          auto& particle_2d = profile.RefParticle(particle_handle);
          particle_2d.strand_handle = strand_handle;
          particle_2d.strand_segment_handle = new_strand_segment_handle;
          particle_2d.base = true;
          particle_2d.SetPosition(particle_2d.GetInitialPosition());
          auto& new_segment = strand_group.RefStrandSegment(new_strand_segment_handle);
          new_segment.data.node_handle = internode_handle;
          new_segment.data.profile_particle_handle = particle_handle;
        }
      }
    }
  }

  for (const auto& internode_handle : sorted_internode_list) {
    auto& internode = strand_model_skeleton.RefNode(internode_handle);
    internode.data.particle_map.clear();
    auto& profile = internode.data.profile;
    for (auto& particle : profile.RefParticles()) {
      internode.data.particle_map.insert({particle.strand_handle, particle.GetHandle()});
    }
    internode.data.strand_count = profile.RefParticles().size();
  }
  strand_model_skeleton.data.num_of_particles = 0;
  for (const auto& internode_handle : sorted_internode_list) {
    auto& internode = strand_model_skeleton.RefNode(internode_handle);
    int max_child_size = 0;
    SkeletonNodeHandle max_child_handle = -1;
    for (const auto& child_handle : internode.PeekChildHandles()) {
      auto& child_internode = strand_model_skeleton.RefNode(child_handle);
      if (const auto child_size = static_cast<float>(child_internode.data.particle_map.size());
          child_size > max_child_size) {
        max_child_size = child_size;
        max_child_handle = child_handle;
      }
    }
    for (const auto& child_handle : internode.PeekChildHandles()) {
      auto& child_internode = strand_model_skeleton.RefNode(child_handle);
      if (child_handle == max_child_handle)
        child_internode.data.split = false;
      if (const auto child_size = static_cast<float>(child_internode.data.particle_map.size());
          child_size > max_child_size * strand_model_parameters.overlap_threshold) {
        child_internode.data.split = true;
      } else {
        child_internode.data.split = false;
      }
    }
    strand_model_skeleton.data.num_of_particles += internode.data.profile.PeekParticles().size();
  }
}

JobHandle StrandModel::CalculateProfiles(const StrandModelParameters& strand_model_parameters) {
  const auto& sorted_internode_list = strand_model_skeleton.PeekSortedNodeList();
  if (sorted_internode_list.empty())
    return {};

  Jobs::RunParallelFor(sorted_internode_list.size(), [&](unsigned i) {
    auto& internode = strand_model_skeleton.RefNode(sorted_internode_list[i]);
    for (auto& particle : internode.data.profile.RefParticles()) {
      if (!internode.IsEndNode())
        particle.SetPosition(glm::vec2(0.0f));
      particle.SetVelocity(glm::vec2(0.0f), 0.002f);
      particle.SetAcceleration(glm::vec2(0.0f));
    }
  });
  const auto& base_node = strand_model_skeleton.PeekNode(0);
  const float max_root_distance = base_node.info.end_distance + base_node.info.length;

  for (auto it = sorted_internode_list.rbegin(); it != sorted_internode_list.rend(); ++it) {
    CalculateProfile(max_root_distance, *it, strand_model_parameters);
    strand_model_skeleton.data.num_of_particles +=
        strand_model_skeleton.RefNode(*it).data.profile.PeekParticles().size();
  }
  std::vector<JobHandle> ret_val;
  for (const auto node_handle : strand_model_skeleton.PeekBaseNodeList()) {
    if (const auto& node = strand_model_skeleton.PeekNode(node_handle); node.data.job.Valid()) {
      ret_val.emplace_back(node.data.job);
    }
  }
  return Jobs::Combine(ret_val);
}

void StrandModel::CalculateProfile(const float max_root_distance, const SkeletonNodeHandle node_handle,
                                   const StrandModelParameters& strand_model_parameters) {
  std::vector<JobHandle> dependencies;
  for (const auto& child_handle : strand_model_skeleton.RefNode(node_handle).PeekChildHandles()) {
    if (const auto worker_handle = strand_model_skeleton.RefNode(child_handle).data.job; worker_handle.Valid()) {
      dependencies.emplace_back(worker_handle);
    }
  }
  strand_model_skeleton.RefNode(node_handle).data.job = {};
  strand_model_skeleton.RefNode(node_handle).data.job = Jobs::Run(dependencies, [&, node_handle]() {
    MergeTask(max_root_distance, node_handle, strand_model_parameters);
    auto& internode = strand_model_skeleton.RefNode(node_handle);
    if (internode.data.profile.PeekParticles().size() > 1) {
      PackTask(node_handle, strand_model_parameters);
      if (internode.PeekChildHandles().empty())
        CopyFrontToBackTask(node_handle);
    }
    internode.data.profile.CalculateBoundaries(true, strand_model_parameters.boundary_point_distance);
  });
  Jobs::Execute(strand_model_skeleton.RefNode(node_handle).data.job);
}

void StrandModel::PackTask(const SkeletonNodeHandle node_handle, const StrandModelParameters& strand_model_parameters) {
  auto& internode = strand_model_skeleton.RefNode(node_handle);
  auto& internode_data = internode.data;

  const auto iterations = internode_data.packing_iteration;

  int timeout = strand_model_parameters.junction_profile_packing_max_iteration;
  if (!internode_data.profile_constraints.boundaries.empty())
    timeout = strand_model_parameters.modified_profile_packing_max_iteration;
  for (int i = 0; i < iterations; i++) {
    internode_data.profile.Simulate(
        1,
        [&](auto& grid, const bool grid_resized) {
          if (grid_resized || internode_data.boundaries_updated)
            grid.ApplyBoundaries(internode_data.profile_constraints);
          internode_data.boundaries_updated = false;
        },
        [&](auto& particle) {
          auto acceleration = glm::vec2(0.f);
          if (!internode_data.profile.particle_grid_2d.PeekCells().empty()) {
            const auto& cell = internode_data.profile.particle_grid_2d.RefCell(particle.GetPosition());
            if (glm::length(cell.target) > glm::epsilon<float>()) {
              acceleration += strand_model_parameters.center_attraction_strength * glm::normalize(cell.target);
            }
          }
          particle.SetAcceleration(acceleration);
        });
    if (timeout > 0 && i > timeout)
      break;
  }
}

void StrandModel::MergeTask(float max_root_distance, SkeletonNodeHandle node_handle,
                            const StrandModelParameters& strand_model_parameters) {
  auto& internode = strand_model_skeleton.RefNode(node_handle);
  auto& internode_data = internode.data;
  internode_data.twist_angle = 0.0f;
  const auto& child_handles = internode.PeekChildHandles();
  int max_child_size = -1;
  SkeletonNodeHandle max_child_handle = -1;
  for (const auto& child_handle : child_handles) {
    auto& child_internode = strand_model_skeleton.RefNode(child_handle);
    if (const auto child_size = static_cast<float>(child_internode.data.particle_map.size());
        child_size > max_child_size) {
      max_child_size = child_size;
      max_child_handle = child_handle;
    }
  }

  internode_data.center_direction_radius = 0.0f;
  if (!internode_data.profile_constraints.boundaries.empty()) {
    internode_data.packing_iteration = glm::min(strand_model_parameters.modified_profile_packing_max_iteration,
                                                 static_cast<int>(internode_data.profile.RefParticles().size()) *
                                                     strand_model_parameters.max_simulation_iteration_cell_factor);
  }
  if (child_handles.empty()) {
    if (internode_data.profile_constraints.boundaries.empty())
      internode.data.packing_iteration = glm::min(strand_model_parameters.junction_profile_packing_max_iteration,
                                                   static_cast<int>(internode_data.profile.RefParticles().size()) *
                                                       strand_model_parameters.max_simulation_iteration_cell_factor);
    int particle_index = 0;
    for (const auto& particle : internode_data.profile.RefParticles()) {
      const auto node_particle_handle = internode_data.particle_map.at(particle.strand_handle);
      auto& node_particle = internode_data.profile.RefParticle(node_particle_handle);
      particle_index++;
      node_particle.SetColor(particle.GetColor());
      node_particle.main_child = true;
      node_particle.corresponding_child_node_handle = -1;
    }
    return;
  }

  if (max_child_handle == -1)
    max_child_handle = child_handles.front();
  auto& main_child_node = strand_model_skeleton.RefNode(max_child_handle);
  const auto& main_child_physics_2d = main_child_node.data.profile;
  for (const auto& child_handle : child_handles) {
    auto& child_node = strand_model_skeleton.RefNode(child_handle);
    const auto child_node_front = glm::inverse(internode.info.regulated_global_rotation) *
                                  child_node.info.regulated_global_rotation * glm::vec3(0, 0, -1);
    auto direction = glm::normalize(glm::vec2(child_node_front.x, child_node_front.y));
    const auto main_child_radius = main_child_physics_2d.GetDistanceToOrigin(direction, glm::vec2(0.0f));
    auto offset = glm::vec2(0.0f);
    offset = (main_child_radius - child_node.data.center_direction_radius + 2.0f) * direction;
    child_node.data.offset = offset;
  }
  const auto branch_twist_angle =
      strand_model_parameters.branch_twist_distribution.GetValue(internode.info.root_distance / max_root_distance);
  const auto junction_twist_angle =
      strand_model_parameters.junction_twist_distribution.GetValue(internode.info.root_distance / max_root_distance);
  if (child_handles.size() == 1) {
    // Copy from child flow start to self flow start
    auto& child_node = strand_model_skeleton.RefNode(child_handles.front());
    const auto& child_physics_2d = child_node.data.profile;
    child_node.data.twist_angle = branch_twist_angle;

    for (const auto& child_particle : child_physics_2d.PeekParticles()) {
      const auto node_particle_handle = internode_data.particle_map.at(child_particle.strand_handle);
      auto& node_particle = internode_data.profile.RefParticle(node_particle_handle);
      node_particle.SetColor(child_particle.GetColor());
      auto polar_position = child_particle.GetPolarPosition();
      polar_position.y += glm::radians(branch_twist_angle);
      node_particle.SetPolarPosition(polar_position);
      node_particle.SetInitialPosition(node_particle.GetPosition());
      node_particle.main_child = true;
      node_particle.corresponding_child_node_handle = child_handles.front();
    }
    if (internode_data.profile_constraints.boundaries.empty())
      internode.data.packing_iteration = glm::min(strand_model_parameters.branch_profile_packing_max_iteration,
                                                   static_cast<int>(internode_data.profile.RefParticles().size()) *
                                                       strand_model_parameters.max_simulation_iteration_cell_factor);
    return;
  }
  if (internode_data.profile_constraints.boundaries.empty())
    internode.data.packing_iteration = glm::min(strand_model_parameters.junction_profile_packing_max_iteration,
                                                 static_cast<int>(internode_data.profile.RefParticles().size()) *
                                                     strand_model_parameters.max_simulation_iteration_cell_factor);

  main_child_node.data.twist_angle = junction_twist_angle;
  for (const auto& main_child_particle : main_child_physics_2d.PeekParticles()) {
    const auto node_particle_handle = internode_data.particle_map.at(main_child_particle.strand_handle);
    auto& node_particle = internode_data.profile.RefParticle(node_particle_handle);
    node_particle.SetColor(main_child_particle.GetColor());
    auto polar_position = main_child_particle.GetPolarPosition();
    polar_position.y += glm::radians(junction_twist_angle);
    node_particle.SetPolarPosition(polar_position);
    node_particle.SetInitialPosition(node_particle.GetPosition());

    node_particle.main_child = true;
    node_particle.corresponding_child_node_handle = max_child_handle;
  }

  if (strand_model_parameters.pre_merge) {
    bool need_simulation = false;
    for (const auto& child_handle : child_handles) {
      if (child_handle == max_child_handle)
        continue;
      auto& child_node = strand_model_skeleton.RefNode(child_handle);
      auto& child_physics_2d = child_node.data.profile;
      if (!child_node.data.split) {
        need_simulation = true;
        const auto child_node_front = glm::inverse(internode.info.regulated_global_rotation) *
                                      child_node.info.regulated_global_rotation * glm::vec3(0, 0, -1);
        auto direction = glm::normalize(glm::vec2(child_node_front.x, child_node_front.y));
        if (glm::isnan(direction.x) || glm::isnan(direction.y)) {
          direction = glm::vec2(1, 0);
        }
        child_node.data.center_direction_radius = child_physics_2d.GetDistanceToOrigin(-direction, glm::vec2(0.0f));

        for (auto& child_particle : child_physics_2d.RefParticles()) {
          child_particle.SetPosition(child_particle.GetPosition() + child_node.data.offset);

          const auto node_particle_handle = internode_data.particle_map.at(child_particle.strand_handle);
          auto& node_particle = internode_data.profile.RefParticle(node_particle_handle);
          node_particle.SetColor(child_particle.GetColor());
          auto polar_position = child_particle.GetPolarPosition();
          polar_position.y += glm::radians(junction_twist_angle);
          node_particle.SetPolarPosition(polar_position);
          node_particle.SetPolarPosition(node_particle.GetPosition());
          node_particle.enable = true;
          node_particle.main_child = false;
          node_particle.corresponding_child_node_handle = child_handle;
        }
      } else {
        for (auto& child_particle : child_physics_2d.RefParticles()) {
          const auto node_particle_handle = internode_data.particle_map.at(child_particle.strand_handle);
          auto& node_particle = internode_data.profile.RefParticle(node_particle_handle);
          node_particle.SetColor(child_particle.GetColor());
          auto polar_position = child_particle.GetPolarPosition();
          polar_position.y += glm::radians(junction_twist_angle);
          node_particle.SetPolarPosition(polar_position);
          node_particle.SetPolarPosition(node_particle.GetPosition());
          node_particle.enable = false;
          node_particle.main_child = false;
          node_particle.corresponding_child_node_handle = child_handle;
        }
      }
    }
    if (need_simulation)
      PackTask(node_handle, strand_model_parameters);
    for (const auto& child_handle : child_handles) {
      if (child_handle == max_child_handle)
        continue;
      if (auto& child_node = strand_model_skeleton.RefNode(child_handle); child_node.data.split) {
        auto& child_physics_2d = child_node.data.profile;
        const auto child_node_front = glm::inverse(internode.info.regulated_global_rotation) *
                                      child_node.info.regulated_global_rotation * glm::vec3(0, 0, -1);
        auto direction = glm::normalize(glm::vec2(child_node_front.x, child_node_front.y));
        if (glm::isnan(direction.x) || glm::isnan(direction.y)) {
          direction = glm::vec2(1, 0);
        }
        child_node.data.center_direction_radius = child_physics_2d.GetDistanceToOrigin(-direction, glm::vec2(0.0f));
        const auto center_radius = internode_data.profile.GetDistanceToOrigin(direction, glm::vec2(0.0f));
        auto offset = glm::vec2(0.0f);
        offset = (center_radius + child_node.data.center_direction_radius + 2.0f) * direction;
        child_node.data.offset = offset;
        for (auto& child_particle : child_physics_2d.RefParticles()) {
          auto polar_position = child_particle.GetPolarPosition();
          polar_position.y += glm::radians(junction_twist_angle);
          child_particle.SetPolarPosition(polar_position);
          child_particle.SetPosition(child_particle.GetPosition() + offset);

          const auto node_particle_handle = internode_data.particle_map.at(child_particle.strand_handle);
          auto& node_particle = internode_data.profile.RefParticle(node_particle_handle);
          node_particle.SetColor(child_particle.GetColor());
          node_particle.SetPosition(child_particle.GetPosition());
          node_particle.SetInitialPosition(child_particle.GetPosition());
          node_particle.main_child = false;
          node_particle.corresponding_child_node_handle = child_handle;
        }
      }
    }
    CopyFrontToBackTask(node_handle);
    internode_data.profile.SetEnableAllParticles(true);
  } else {
    for (const auto& child_handle : child_handles) {
      if (child_handle == max_child_handle)
        continue;
      auto& child_node = strand_model_skeleton.RefNode(child_handle);
      child_node.data.twist_angle = junction_twist_angle;
      auto& child_physics_2d = child_node.data.profile;
      const auto child_node_front = glm::inverse(internode.info.regulated_global_rotation) *
                                    child_node.info.regulated_global_rotation * glm::vec3(0, 0, -1);
      auto direction = glm::normalize(glm::vec2(child_node_front.x, child_node_front.y));
      if (glm::isnan(direction.x) || glm::isnan(direction.y)) {
        direction = glm::vec2(1, 0);
      }
      child_node.data.center_direction_radius = child_physics_2d.GetDistanceToOrigin(-direction, glm::vec2(0.0f));
      const auto main_child_radius = main_child_physics_2d.GetDistanceToOrigin(direction, glm::vec2(0.0f));
      auto offset = glm::vec2(0.0f);
      if (child_node.data.split) {
        offset = (main_child_radius + child_node.data.center_direction_radius + 2.0f) * direction;
      } else {
        offset = (main_child_radius - child_node.data.center_direction_radius) * direction;
      }
      child_node.data.offset = offset;
      for (auto& child_particle : child_physics_2d.RefParticles()) {
        child_particle.SetPosition(child_particle.GetPosition() + offset);

        const auto node_particle_handle = internode_data.particle_map.at(child_particle.strand_handle);
        auto& node_particle = internode_data.profile.RefParticle(node_particle_handle);
        node_particle.SetColor(child_particle.GetColor());
        auto polar_position = child_particle.GetPolarPosition();
        polar_position.y += glm::radians(strand_model_parameters.junction_twist_distribution.GetValue(
            internode.info.root_distance / max_root_distance));
        node_particle.SetPolarPosition(polar_position);
        node_particle.SetInitialPosition(node_particle.GetPosition());
        node_particle.main_child = false;
        node_particle.corresponding_child_node_handle = child_handle;
      }
    }
  }
}

void StrandModel::CopyFrontToBackTask(const SkeletonNodeHandle node_handle) {
  auto& internode = strand_model_skeleton.RefNode(node_handle);
  auto& internode_data = internode.data;
  for (int i = 0; i < internode_data.profile.RefParticles().size(); i++) {
    internode_data.profile.RefParticle(i).SetInitialPosition(internode_data.profile.RefParticle(i).GetPosition());
  }
}

void StrandModel::ApplyProfile(const StrandModelParameters& strand_model_parameters,
                               const SkeletonNodeHandle node_handle) {
  auto& node = strand_model_skeleton.RefNode(node_handle);
  const auto current_front = node.info.regulated_global_rotation * glm::vec3(0, 0, -1);
  const auto current_up = node.info.regulated_global_rotation * glm::vec3(0, 1, 0);
  const auto current_left = node.info.regulated_global_rotation * glm::vec3(1, 0, 0);
  const auto& parameters = strand_model_parameters;
  const bool wound = node.IsEndNode();
  for (const auto& [strandHandle, particleHandle] : node.data.particle_map) {
    const auto& particle = node.data.profile.PeekParticle(particleHandle);
    auto& new_strand_segment =
        strand_model_skeleton.data.strand_group.RefStrandSegment(particle.strand_segment_handle);
    new_strand_segment.info.thickness = node.data.strand_radius;
    new_strand_segment.info.global_position =
        node.info.GetGlobalEndPosition() + node.data.strand_radius * particle.GetInitialPosition().x * current_left +
        node.data.strand_radius * particle.GetInitialPosition().y * current_up;
    if (glm::any(glm::isnan(new_strand_segment.info.global_position))) {
      EVOENGINE_ERROR("Nan!");
    }
    if (wound) {
      new_strand_segment.info.global_position +=
          current_front *
          glm::max(0.0f, strand_model_parameters.cladoptosis_distribution.GetValue(glm::max(
                             0.0f, (strand_model_parameters.cladoptosis_range - particle.GetDistanceToBoundary()) /
                                       strand_model_parameters.cladoptosis_range)));
    }
    new_strand_segment.info.color =
        particle.IsBoundary() ? parameters.boundary_point_color : parameters.content_point_color;
    new_strand_segment.info.is_boundary = particle.IsBoundary();
  }
}

void StrandModel::ApplyProfiles(const StrandModelParameters& strand_model_parameters) {
  auto& strand_group = strand_model_skeleton.data.strand_group;
  const auto& sorted_internode_list = strand_model_skeleton.PeekSortedNodeList();
  for (const auto& node_handle : sorted_internode_list) {
    const auto& node = strand_model_skeleton.RefNode(node_handle);
    if (node.GetParentHandle() == -1) {
      const auto parent_global_rotation = node.info.regulated_global_rotation;

      const auto current_up = parent_global_rotation * glm::vec3(0, 1, 0);
      const auto current_left = parent_global_rotation * glm::vec3(1, 0, 0);
      const auto base_radius = strand_model_parameters.strand_radius_distribution.GetValue(0.0f);
      for (const auto& [strandHandle, particleHandle] : node.data.particle_map) {
        const auto& particle = node.data.profile.PeekParticle(particleHandle);
        auto& strand = strand_group.RefStrand(strandHandle);
        strand.info.base_info.thickness = base_radius;
        strand.info.base_info.global_position =
            strand.info.base_info.thickness * particle.GetPosition().x * current_left +
            strand.info.base_info.thickness * particle.GetPosition().y * current_up;
        strand.info.base_info.is_boundary = particle.IsBoundary();
        strand.info.base_info.color = particle.IsBoundary() ? strand_model_parameters.boundary_point_color
                                                            : strand_model_parameters.content_point_color;
      }
    }

    ApplyProfile(strand_model_parameters, node_handle);
  }
}

void StrandModel::CalculateStrandProfileAdjustedTransforms(const StrandModelParameters& strand_model_parameters) {
  const auto& sorted_internode_list = strand_model_skeleton.PeekSortedNodeList();
  float max_root_distance = 0.0f;
  for (const auto& node_handle : sorted_internode_list) {
    auto& node = strand_model_skeleton.RefNode(node_handle);
    if (const auto parent_handle = node.GetParentHandle(); parent_handle != -1)
      break;
    node.info.global_position = node.info.global_position;
    node.info.global_rotation = node.info.regulated_global_rotation;
    max_root_distance = glm::max(max_root_distance, node.info.end_distance + node.info.length);
  }

  for (const auto& node_handle : sorted_internode_list) {
    auto& node = strand_model_skeleton.RefNode(node_handle);
    const auto parent_handle = node.GetParentHandle();
    if (parent_handle == -1) {
      node.info.global_position = node.info.global_position;
      node.info.global_rotation = node.info.regulated_global_rotation;
      node.data.strand_radius =
          strand_model_parameters.strand_radius_distribution.GetValue(node.info.root_distance / max_root_distance);
      continue;
    }
    const auto& parent_node = strand_model_skeleton.PeekNode(parent_handle);
    node.info.global_position = parent_node.info.GetGlobalEndPosition();
    glm::quat parent_global_rotation = parent_node.info.global_rotation;
    node.data.strand_radius =
        strand_model_parameters.strand_radius_distribution.GetValue(node.info.root_distance / max_root_distance);
    const auto global_direction = node.info.GetGlobalDirection();
    auto local_position = global_direction * node.info.length;
    auto new_global_end_position = node.info.global_position;
    node.info.global_rotation = parent_global_rotation * (glm::inverse(parent_node.info.regulated_global_rotation) *
                                                          node.info.regulated_global_rotation);

    const auto parent_up = parent_global_rotation * glm::vec3(0, 1, 0);
    const auto parent_left = parent_global_rotation * glm::vec3(1, 0, 0);
    const auto parent_front = parent_global_rotation * glm::vec3(0, 0, -1);

    const auto front = node.info.global_rotation * glm::vec3(0, 0, -1);
    const float offset_length = glm::length(node.data.offset);
    const float cos_front = glm::dot(front, parent_front);                            // Horizontal
    const float sin_front = glm::sin(glm::acos(glm::clamp(cos_front, -1.0f, 1.0f)));  // Vertical

    glm::vec2 parent_center = glm::vec2(0.0f);
    int particle_count = 0;

    for (const auto& parent_particle : parent_node.data.profile.PeekParticles()) {
      if (parent_particle.corresponding_child_node_handle == node_handle) {
        parent_center += parent_particle.GetPosition();
        particle_count++;
      }
    }
    if (particle_count > 0) {
      parent_center /= particle_count;
    }
    glm::vec3 side_shift = glm::vec3(0.f);
    if (node.IsApical() && strand_model_parameters.side_push_factor > 0.f) {
      side_shift += parent_up * parent_center.y * strand_model_parameters.side_push_factor * node.data.strand_radius;
      side_shift += parent_left * parent_center.x * strand_model_parameters.side_push_factor * node.data.strand_radius;
    } else if (!node.IsApical() && strand_model_parameters.apical_side_push_factor > 0.f) {
      side_shift +=
          parent_up * parent_center.y * strand_model_parameters.apical_side_push_factor * node.data.strand_radius;
      side_shift +=
          parent_left * parent_center.x * strand_model_parameters.apical_side_push_factor * node.data.strand_radius;
    }
    glm::vec3 rotation_shift = glm::vec3(0.f);
    if (offset_length > glm::epsilon<float>()) {
      const auto offset_direction = glm::normalize(node.data.offset);
      float max_radius = node.data.profile.GetMaxDistanceToCenter();
      for (const auto& parent_particle : parent_node.data.profile.PeekParticles()) {
        if (parent_particle.corresponding_child_node_handle == node_handle) {
          const auto distance = glm::length(glm::closestPointOnLine(parent_particle.GetPosition(), parent_center,
                                                                    parent_center + node.data.offset * 1000.0f));
          max_radius = glm::max(max_radius, distance);
        }
      }
      for (const auto& particle : parent_node.data.profile.PeekParticles()) {
        const auto distance =
            glm::length(glm::closestPointOnLine(particle.GetPosition(), glm::vec2(0.f), node.data.offset * 1000.0f));
        max_radius = glm::max(max_radius, distance);
      }

      if (node.IsApical()) {
        rotation_shift += parent_up * offset_direction.y * cos_front * max_radius *
                          strand_model_parameters.apical_branch_rotation_push_factor * node.data.strand_radius;
        rotation_shift += parent_left * offset_direction.x * cos_front * max_radius *
                          strand_model_parameters.apical_branch_rotation_push_factor * node.data.strand_radius;
        rotation_shift += parent_front * sin_front * max_radius *
                          strand_model_parameters.apical_branch_rotation_push_factor * node.data.strand_radius;
      } else {
        rotation_shift += parent_up * offset_direction.y * cos_front * max_radius *
                          strand_model_parameters.rotation_push_factor * node.data.strand_radius;
        rotation_shift += parent_left * offset_direction.x * cos_front * max_radius *
                          strand_model_parameters.rotation_push_factor * node.data.strand_radius;
        rotation_shift += parent_front * sin_front * max_radius * strand_model_parameters.rotation_push_factor *
                          node.data.strand_radius;
      }
    }
    if (!node.IsApical()) {
      if (const auto projected_shift = global_direction * glm::dot(rotation_shift + side_shift, global_direction);
          glm::length(projected_shift) > glm::length(local_position)) {
        new_global_end_position += projected_shift;
      } else {
        new_global_end_position += local_position;
      }
    } else {
      new_global_end_position += local_position + side_shift;
    }
    assert(!glm::any(glm::isnan(node.info.global_position)));
    assert(!glm::any(glm::isnan(new_global_end_position)));

    const auto diff = new_global_end_position - node.info.global_position;
    node.info.length = glm::length(diff);
  }
  strand_model_skeleton.CalculateRegulatedGlobalRotation();
  strand_model_skeleton.CalculateDistance();
  for (const auto& node_handle : sorted_internode_list) {
    auto& node = strand_model_skeleton.RefNode(node_handle);
    node.info.global_rotation = node.info.regulated_global_rotation;
  }
}

evo_engine::StrandPoint operator/(const evo_engine::StrandPoint& lhs, const float& rhs) {
  evo_engine::StrandPoint ret_val;
  ret_val.thickness = lhs.thickness / rhs;
  ret_val.position = lhs.position / rhs;
  ret_val.color = lhs.color / rhs;
  ret_val.tex_coord = lhs.tex_coord / rhs;
  return ret_val;
}

evo_engine::StrandPoint operator*(const evo_engine::StrandPoint& lhs, const float& rhs) {
  evo_engine::StrandPoint ret_val;
  ret_val.thickness = lhs.thickness * rhs;
  ret_val.position = lhs.position * rhs;
  ret_val.color = lhs.color * rhs;
  ret_val.tex_coord = lhs.tex_coord * rhs;
  return ret_val;
}

evo_engine::StrandPoint operator+(const evo_engine::StrandPoint& lhs, const evo_engine::StrandPoint& rhs) {
  evo_engine::StrandPoint ret_val;
  ret_val.thickness = lhs.thickness + rhs.thickness;
  ret_val.position = lhs.position + rhs.position;
  ret_val.color = lhs.color + rhs.color;
  ret_val.tex_coord = lhs.tex_coord + rhs.tex_coord;
  return ret_val;
}

evo_engine::StrandPoint operator-(const evo_engine::StrandPoint& lhs, const evo_engine::StrandPoint& rhs) {
  evo_engine::StrandPoint ret_val;
  ret_val.thickness = lhs.thickness - rhs.thickness;
  ret_val.position = lhs.position - rhs.position;
  ret_val.color = lhs.color - rhs.color;
  ret_val.tex_coord = lhs.tex_coord - rhs.tex_coord;
  return ret_val;
}

glm::vec3 StrandModel::InterpolateStrandSegmentPosition(const StrandSegmentHandle strand_segment_handle,
                                                        float a) const {
  assert(strand_segment_handle >= 0);
  assert(a >= 0.f && a <= 1.f);
  a = glm::clamp(a, 0.0f, 1.0f);
  const auto& strand_group = strand_model_skeleton.data.strand_group;
  assert(strand_group.PeekStrandSegments().size() > strand_segment_handle);
  const auto& strand_segment = strand_group.PeekStrandSegment(strand_segment_handle);
  const auto& strand = strand_group.PeekStrand(strand_segment.GetStrandHandle());
  auto& base_info = strand.info.base_info;

  const auto& strand_segment_handles = strand.PeekStrandSegmentHandles();

  glm::vec3 p[4];

  p[2] = strand_segment.info.global_position;
  if (strand_segment_handle == strand_segment_handles.front()) {
    p[1] = base_info.global_position;
    p[0] = p[1] * 2.0f - p[2];
  } else if (strand_segment_handle == strand_segment_handles.at(1)) {
    p[0] = base_info.global_position;
    p[1] = strand_group.PeekStrandSegment(strand_segment_handles.front()).info.global_position;
  } else {
    const auto& prev_segment = strand_group.PeekStrandSegment(strand_segment.GetPrevHandle());
    p[1] = prev_segment.info.global_position;
    const auto& prev_prev_segment = strand_group.PeekStrandSegment(prev_segment.GetPrevHandle());
    p[0] = prev_prev_segment.info.global_position;
  }
  if (strand_segment_handle == strand_segment_handles.back()) {
    p[3] = p[2] * 2.0f - p[1];
  } else {
    const auto& next_segment = strand_group.PeekStrandSegment(strand_segment.GetNextHandle());
    p[3] = next_segment.info.global_position;
  }
  glm::vec3 position, tangent;
  Strands::CubicInterpolation(p[0], p[1], p[2], p[3], position, tangent, a);
  if (glm::any(glm::isnan(position))) {
    EVOENGINE_ERROR("nan");
  }
  return position;
}

glm::vec3 StrandModel::InterpolateStrandSegmentAxis(StrandSegmentHandle strand_segment_handle, const float a) const {
  assert(strand_segment_handle >= 0);
  assert(a >= 0.f && a <= 1.f);
  const auto& strand_group = strand_model_skeleton.data.strand_group;
  assert(strand_group.PeekStrandSegments().size() > strand_segment_handle);
  const auto& strand_segment = strand_group.PeekStrandSegment(strand_segment_handle);
  const auto& strand = strand_group.PeekStrand(strand_segment.GetStrandHandle());
  auto& base_info = strand.info.base_info;

  const auto& strand_segment_handles = strand.PeekStrandSegmentHandles();

  glm::vec3 p[4];

  p[2] = strand_segment.info.global_position;
  if (strand_segment_handle == strand_segment_handles.front()) {
    p[1] = base_info.global_position;
    p[0] = p[1] * 2.0f - p[2];
  } else if (strand_segment_handle == strand_segment_handles.at(1)) {
    p[0] = base_info.global_position;
    p[1] = strand_group.PeekStrandSegment(strand_segment_handles.front()).info.global_position;
  } else {
    const auto& prev_segment = strand_group.PeekStrandSegment(strand_segment.GetPrevHandle());
    p[1] = prev_segment.info.global_position;
    const auto& prev_prev_segment = strand_group.PeekStrandSegment(prev_segment.GetPrevHandle());
    p[0] = prev_prev_segment.info.global_position;
  }
  if (strand_segment_handle == strand_segment_handles.back()) {
    p[3] = p[2] * 2.0f - p[1];
  } else {
    const auto& next_segment = strand_group.PeekStrandSegment(strand_segment.GetNextHandle());
    p[3] = next_segment.info.global_position;
  }
  glm::vec3 position, tangent;
  Strands::CubicInterpolation(p[0], p[1], p[2], p[3], position, tangent, a);
  return tangent;
}

float StrandModel::InterpolateStrandSegmentRadius(StrandSegmentHandle strand_segment_handle, const float a) const {
  assert(strand_segment_handle >= 0);
  assert(a >= 0.f && a <= 1.f);
  const auto& strand_group = strand_model_skeleton.data.strand_group;
  assert(strand_group.PeekStrandSegments().size() > strand_segment_handle);
  const auto& strand_segment = strand_group.PeekStrandSegment(strand_segment_handle);
  const auto& strand = strand_group.PeekStrand(strand_segment.GetStrandHandle());
  auto& base_info = strand.info.base_info;

  const auto& strand_segment_handles = strand.PeekStrandSegmentHandles();

  float p[4];

  p[2] = strand_segment.info.thickness;
  if (strand_segment_handle == strand_segment_handles.front()) {
    p[1] = base_info.thickness;
    p[0] = p[1] * 2.0f - p[2];
  } else if (strand_segment_handle == strand_segment_handles.at(1)) {
    p[0] = base_info.thickness;
    p[1] = strand_group.PeekStrandSegment(strand_segment_handles.front()).info.thickness;
  } else {
    const auto& prev_segment = strand_group.PeekStrandSegment(strand_segment.GetPrevHandle());
    p[1] = prev_segment.info.thickness;
    const auto& prev_prev_segment = strand_group.PeekStrandSegment(prev_segment.GetPrevHandle());
    p[0] = prev_prev_segment.info.thickness;
  }
  if (strand_segment_handle == strand_segment_handles.back()) {
    p[3] = p[2] * 2.0f - p[1];
  } else {
    const auto& next_segment = strand_group.PeekStrandSegment(strand_segment.GetNextHandle());
    p[3] = next_segment.info.thickness;
  }
  float radius, tangent;
  Strands::CubicInterpolation(p[0], p[1], p[2], p[3], radius, tangent, a);
  return radius;
}
