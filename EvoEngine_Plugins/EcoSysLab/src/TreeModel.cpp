//
// Created by lllll on 10/21/2022.
//

#include "TreeModel.hpp"

using namespace eco_sys_lab;
void ReproductiveModule::Reset() {
  maturity = 0.0f;
  health = 1.0f;
  transform = glm::mat4(0.0f);
}

void TreeModel::ResetReproductiveModule() {
  const auto& sorted_internode_list = shoot_skeleton_.PeekSortedNodeList();
  for (auto it = sorted_internode_list.rbegin(); it != sorted_internode_list.rend(); ++it) {
    auto& internode = shoot_skeleton_.RefNode(*it);
    auto& internode_data = internode.data;
    auto& buds = internode_data.buds;
    for (auto& bud : buds) {
      if (bud.type == BudType::Fruit || bud.type == BudType::Leaf) {
        bud.status = BudStatus::Dormant;
        bud.reproductive_module.Reset();
      }
    }
  }
  fruit_count_ = leaf_count_ = 0;
}

void TreeModel::RegisterVoxel(const glm::mat4& global_transform, ClimateModel& climate_model,
                              const ShootGrowthController& shoot_growth_controller) {
  const auto& sorted_internode_list = shoot_skeleton_.PeekSortedNodeList();
  auto& environment_grid = climate_model.environment_grid;
  for (auto it = sorted_internode_list.rbegin(); it != sorted_internode_list.rend(); ++it) {
    const auto& internode = shoot_skeleton_.RefNode(*it);
    const auto& internode_info = internode.info;
    const float shadow_size = shoot_growth_controller.m_internodeShadowFactor;
    const float biomass = internode_info.thickness;
    const glm::vec3 world_position = global_transform * glm::vec4(internode_info.global_position, 1.0f);
    environment_grid.AddShadowValue(world_position, shadow_size);
    environment_grid.AddBiomass(world_position, biomass);
    if (internode.IsEndNode()) {
      InternodeVoxelRegistration registration;
      registration.position = world_position;
      registration.node_handle = *it;
      registration.tree_skeleton_index = shoot_skeleton_.data.index;
      registration.thickness = internode_info.thickness;
      environment_grid.AddNode(registration);
    }
  }
}

void TreeModel::PruneInternode(const SkeletonNodeHandle internode_handle) {
  const auto& internode = shoot_skeleton_.RefNode(internode_handle);

  if (!internode.IsRecycled() && !internode.info.locked) {
    auto& parent_node = shoot_skeleton_.RefNode(internode.GetParentHandle());
    parent_node.info.wounds.emplace_back();
    auto& wound = parent_node.info.wounds.back();
    wound.apical = internode.IsApical();
    wound.thickness = internode.info.thickness;
    wound.healing = 0.f;
    wound.local_rotation = glm::inverse(parent_node.info.global_rotation) * internode.info.global_rotation;

    shoot_skeleton_.RecycleNode(
        internode_handle,
        [&](SkeletonFlowHandle flow_handle) {
        },
        [&](SkeletonNodeHandle node_handle) {

        });
  }
}

void TreeModel::HarvestFruits(const std::function<bool(const ReproductiveModule& fruit)>& harvest_function) {
  const auto& sorted_internode_list = shoot_skeleton_.PeekSortedNodeList();
  fruit_count_ = 0;

  for (auto it = sorted_internode_list.rbegin(); it != sorted_internode_list.rend(); ++it) {
    auto& internode = shoot_skeleton_.RefNode(*it);
    auto& internode_data = internode.data;
    auto& buds = internode_data.buds;
    for (auto& bud : buds) {
      if (bud.type != BudType::Fruit || bud.status != BudStatus::Died)
        continue;

      if (harvest_function(bud.reproductive_module)) {
        bud.reproductive_module.Reset();
      } else if (bud.reproductive_module.maturity > 0)
        fruit_count_++;
    }
  }
}

void TreeModel::ApplyTropism(const glm::vec3& target_dir, float tropism, glm::vec3& front, glm::vec3& up) {
  const glm::vec3 dir = glm::normalize(target_dir);
  if (const float dot_p = glm::abs(glm::dot(front, dir)); dot_p < 0.99f && dot_p > -0.99f) {
    const glm::vec3 left = glm::cross(front, dir);
    const float max_angle = glm::acos(dot_p);
    const float rotate_angle = max_angle * tropism;
    front = glm::normalize(glm::rotate(front, glm::min(max_angle, rotate_angle), left));
    up = glm::normalize(glm::cross(glm::cross(front, up), front));
  }
}

void TreeModel::ApplyTropism(const glm::vec3& target_dir, float tropism, glm::quat& rotation) {
  auto front = rotation * glm::vec3(0, 0, -1);
  auto up = rotation * glm::vec3(0, 1, 0);
  ApplyTropism(target_dir, tropism, front, up);
  rotation = glm::quatLookAt(front, up);
}

bool TreeModel::Grow(float delta_time, const glm::mat4& global_transform, ClimateModel& climate_model,
                     const ShootGrowthController& shoot_growth_controller, const bool pruning) {
  current_delta_time_ = delta_time;
  age_ += current_delta_time_;
  bool tree_structure_changed = false;
  if (!initialized_) {
    Initialize(shoot_growth_controller);
    tree_structure_changed = true;
  }
  shoot_skeleton_.SortLists();
  CalculateShootFlux(global_transform, climate_model, shoot_growth_controller);
  SampleTemperature(global_transform, climate_model);

  if (pruning) {
    const bool any_branch_pruned = PruneInternodes(global_transform, climate_model, shoot_growth_controller);
    if (any_branch_pruned)
      shoot_skeleton_.SortLists();
    tree_structure_changed = tree_structure_changed || any_branch_pruned;
  }
  bool any_branch_grown = false;
  {
    const auto& sorted_node_list = shoot_skeleton_.PeekSortedNodeList();
    if (!tree_growth_settings.use_space_colonization) {
      const auto total_shoot_flux = CollectShootFlux(sorted_node_list);
      RootFlux total_root_flux;
      total_root_flux.value = total_shoot_flux.value;
      const auto total_flux = glm::min(total_shoot_flux.value, total_root_flux.value);
      CalculateInternodeStrength(sorted_node_list, shoot_growth_controller);
      const float required_vigor = CalculateGrowthPotential(sorted_node_list, shoot_growth_controller);
      CalculateGrowthRate(sorted_node_list, total_flux / required_vigor);
    }
    for (auto it = sorted_node_list.rbegin(); it != sorted_node_list.rend(); ++it) {
      const bool graph_changed = GrowInternode(climate_model, *it, shoot_growth_controller);
      any_branch_grown = any_branch_grown || graph_changed;
    }
  }

  if (any_branch_grown) {
    shoot_skeleton_.SortLists();
  }
  {
    const auto& sorted_node_list = shoot_skeleton_.PeekSortedNodeList();
    for (auto it = sorted_node_list.rbegin(); it != sorted_node_list.rend(); ++it) {
      const bool reproductive_module_changed = GrowReproductiveModules(climate_model, *it, shoot_growth_controller);
      any_branch_grown = any_branch_grown || reproductive_module_changed;
    }
  }
  tree_structure_changed = tree_structure_changed || any_branch_grown;
  ShootGrowthPostProcess(shoot_growth_controller);

  const int year = climate_model.time;
  if (year != age_in_year_) {
    ResetReproductiveModule();
    age_in_year_ = year;
  }
  iteration++;

  return tree_structure_changed;
}

bool TreeModel::Grow(const float delta_time, const SkeletonNodeHandle base_internode_handle,
                     const glm::mat4& global_transform, ClimateModel& climate_model,
                     const ShootGrowthController& shoot_growth_controller, const bool pruning) {
  current_delta_time_ = delta_time;
  age_ += current_delta_time_;
  bool tree_structure_changed = false;
  if (!initialized_) {
    Initialize(shoot_growth_controller);
    tree_structure_changed = true;
  }
  if (shoot_skeleton_.RefRawNodes().size() <= base_internode_handle)
    return false;

  shoot_skeleton_.SortLists();
  CalculateShootFlux(global_transform, climate_model, shoot_growth_controller);
  SampleTemperature(global_transform, climate_model);
  if (pruning) {
    const bool any_branch_pruned = PruneInternodes(global_transform, climate_model, shoot_growth_controller);
    if (any_branch_pruned)
      shoot_skeleton_.SortLists();
    tree_structure_changed = tree_structure_changed || any_branch_pruned;
  }
  bool any_branch_grown = false;
  auto sorted_sub_tree_internode_list = shoot_skeleton_.GetSubTree(base_internode_handle);

  if (!tree_growth_settings.use_space_colonization) {
    const auto total_shoot_flux = CollectShootFlux(sorted_sub_tree_internode_list);
    RootFlux total_root_flux;
    total_root_flux.value = total_shoot_flux.value;
    const auto total_flux = glm::min(total_shoot_flux.value, total_root_flux.value);
    const float required_vigor = CalculateGrowthPotential(sorted_sub_tree_internode_list, shoot_growth_controller);
    CalculateGrowthRate(sorted_sub_tree_internode_list, total_flux / required_vigor);
  }
  for (auto it = sorted_sub_tree_internode_list.rbegin(); it != sorted_sub_tree_internode_list.rend(); ++it) {
    const bool graph_changed = GrowInternode(climate_model, *it, shoot_growth_controller);
    any_branch_grown = any_branch_grown || graph_changed;
  }
  if (any_branch_grown) {
    shoot_skeleton_.SortLists();
    sorted_sub_tree_internode_list = shoot_skeleton_.GetSubTree(base_internode_handle);
  }
  for (auto it = sorted_sub_tree_internode_list.rbegin(); it != sorted_sub_tree_internode_list.rend(); ++it) {
    const bool reproductive_module_changed = GrowReproductiveModules(climate_model, *it, shoot_growth_controller);
    any_branch_grown = any_branch_grown || reproductive_module_changed;
  }
  tree_structure_changed = tree_structure_changed || any_branch_grown;
  ShootGrowthPostProcess(shoot_growth_controller);

  const int year = climate_model.time;
  if (year != age_in_year_) {
    ResetReproductiveModule();
    age_in_year_ = year;
  }
  iteration++;

  return tree_structure_changed;
}

void TreeModel::Initialize(const ShootGrowthController& shoot_growth_controller) {
  if (initialized_)
    Clear();
  {
    shoot_skeleton_ = ShootSkeleton(shoot_growth_controller.m_baseInternodeCount);
    shoot_skeleton_.SortLists();
    for (const auto& node_handle : shoot_skeleton_.PeekSortedNodeList()) {
      auto& node = shoot_skeleton_.RefNode(node_handle);
      node.info.thickness = shoot_growth_controller.m_endNodeThickness;
      node.data.internode_length = 0.0f;
      node.data.buds.emplace_back();
      auto& apical_bud = node.data.buds.back();
      apical_bud.type = BudType::Apical;
      apical_bud.status = BudStatus::Dormant;
      apical_bud.local_rotation = glm::vec3(glm::radians(shoot_growth_controller.m_baseNodeApicalAngle(node)), 0.0f,
                                            glm::radians(glm::linearRand(0.f, 360.f)));
    }
  }

  if (tree_growth_settings.use_space_colonization && tree_growth_settings.space_colonization_auto_resize) {
    const auto grid_radius =
        tree_growth_settings.space_colonization_detection_distance_factor * shoot_growth_controller.m_internodeLength;
    tree_occupancy_grid.Initialize(
        glm::vec3(-grid_radius, 0.0f, -grid_radius), glm::vec3(grid_radius), shoot_growth_controller.m_internodeLength,
        tree_growth_settings.space_colonization_removal_distance_factor, tree_growth_settings.space_colonization_theta,
        tree_growth_settings.space_colonization_detection_distance_factor);
  }

  current_seed_value_ = m_seed;
  initialized_ = true;
}

void TreeModel::CalculateShootFlux(const glm::mat4& global_transform, const ClimateModel& climate_model,
                                   const ShootGrowthController& shoot_growth_controller) {
  auto& shoot_data = shoot_skeleton_.data;
  shoot_data.max_marker_count = 0;
  const auto& sorted_internode_list = shoot_skeleton_.PeekSortedNodeList();
  if (tree_growth_settings.use_space_colonization) {
    if (tree_growth_settings.space_colonization_auto_resize) {
      auto min_bound = shoot_skeleton_.data.desired_min;
      auto max_bound = shoot_skeleton_.data.desired_max;
      const auto original_min = tree_occupancy_grid.GetMin();
      const auto original_max = tree_occupancy_grid.GetMax();
      if (const float detection_range =
          tree_growth_settings.space_colonization_detection_distance_factor * shoot_growth_controller.m_internodeLength; min_bound.x - detection_range < original_min.x || min_bound.y < original_min.y ||
                                                                                                                         min_bound.z - detection_range < original_min.z || max_bound.x + detection_range > original_max.x ||
                                                                                                                         max_bound.y + detection_range > original_max.y || max_bound.z + detection_range > original_max.z) {
        min_bound -= glm::vec3(1.0f);
        max_bound += glm::vec3(1.0f);
        tree_occupancy_grid.Resize(min_bound, max_bound);
      }
    }
    auto& voxel_grid = tree_occupancy_grid.RefGrid();
    for (const auto& internode_handle : sorted_internode_list) {
      auto& internode = shoot_skeleton_.RefNode(internode_handle);
      auto& internode_data = internode.data;
      for (auto& bud : internode_data.buds) {
        bud.marker_direction = glm::vec3(0.0f);
        bud.marker_count = 0;
      }
      internode_data.light_direction = glm::vec3(0.0f);
      const auto dot_min = glm::cos(glm::radians(tree_occupancy_grid.GetTheta()));
      voxel_grid.ForEach(
          internode_data.desired_global_position,
          tree_growth_settings.space_colonization_removal_distance_factor * shoot_growth_controller.m_internodeLength,
          [&](TreeOccupancyGridVoxelData& voxel_data) {
            for (auto& marker : voxel_data.markers) {
              const auto diff = marker.position - internode_data.desired_global_position;
              const auto distance = glm::length(diff);
              const auto direction = glm::normalize(diff);
              if (distance < tree_growth_settings.space_colonization_detection_distance_factor *
                                 shoot_growth_controller.m_internodeLength) {
                if (marker.node_handle != -1)
                  continue;
                if (distance < tree_growth_settings.space_colonization_removal_distance_factor *
                                   shoot_growth_controller.m_internodeLength) {
                  marker.node_handle = internode_handle;
                } else {
                  for (auto& bud : internode_data.buds) {
                    if (auto bud_direction =
                        glm::normalize(internode.info.global_rotation * bud.local_rotation * glm::vec3(0, 0, -1)); glm::dot(direction, bud_direction) > dot_min) {
                      bud.marker_direction += direction;
                      bud.marker_count++;
                    }
                  }
                }
              }
            }
          });
    }
  }
  for (const auto& internode_handle : sorted_internode_list) {
    auto& internode = shoot_skeleton_.RefNode(internode_handle);
    auto& internode_data = internode.data;
    auto& internode_info = internode.info;
    internode_data.light_intensity = 0.0f;
    internode_data.light_direction = -current_gravity_direction;
    bool sample_light_intensity = false;

    for (const auto& bud : internode_data.buds) {
      sample_light_intensity = true;
      if (tree_growth_settings.use_space_colonization) {
        shoot_data.max_marker_count = glm::max(shoot_data.max_marker_count, bud.marker_count);
      }
    }
    const glm::vec3 position = global_transform * glm::vec4(internode_info.global_position, 1.0f);
    if (sample_light_intensity) {
      internode_data.light_intensity =
          glm::clamp(climate_model.environment_grid.Sample(position, internode_data.light_direction), 0.f, 1.f);
      if (internode_data.light_intensity <= glm::epsilon<float>()) {
        internode_data.light_direction = glm::normalize(internode_info.GetGlobalDirection());
      }
    }
    internode_data.space_occupancy = climate_model.environment_grid.voxel_grid.Peek(position).total_biomass;
  }
}
ShootFlux TreeModel::CollectShootFlux(const std::vector<SkeletonNodeHandle>& sorted_internode_list) {
  ShootFlux total_shoot_flux;
  total_shoot_flux.value = 0.0f;
  for (const auto& internode_handle : sorted_internode_list) {
    auto& internode = shoot_skeleton_.RefNode(internode_handle);
    const auto& internode_data = internode.data;
    total_shoot_flux.value += internode_data.light_intensity;
  }

  for (auto it = sorted_internode_list.rbegin(); it != sorted_internode_list.rend(); ++it) {
    auto& internode = shoot_skeleton_.RefNode(*it);
    auto& internode_data = internode.data;
    internode_data.max_descendant_light_intensity = glm::clamp(internode_data.light_intensity, 0.f, 1.f);
    for (const auto& child_handle : internode.PeekChildHandles()) {
      internode_data.max_descendant_light_intensity =
          glm::max(internode_data.max_descendant_light_intensity,
                   shoot_skeleton_.RefNode(child_handle).data.max_descendant_light_intensity);
    }
  }
  return total_shoot_flux;
}

void TreeModel::CalculateInternodeStrength(const std::vector<SkeletonNodeHandle>& sorted_internode_list,
                                           const ShootGrowthController& shoot_growth_controller) {
  for (const auto& internode_handle : sorted_internode_list) {
    auto& internode = shoot_skeleton_.RefNode(internode_handle);
    auto& internode_data = internode.data;
    internode_data.strength = shoot_growth_controller.m_internodeStrength(internode);
  }
}

void TreeModel::ShootGrowthPostProcess(const ShootGrowthController& shoot_growth_controller) {
  {
    shoot_skeleton_.min = glm::vec3(FLT_MAX);
    shoot_skeleton_.max = glm::vec3(-FLT_MAX);
    shoot_skeleton_.data.desired_min = glm::vec3(FLT_MAX);
    shoot_skeleton_.data.desired_max = glm::vec3(-FLT_MAX);

    shoot_skeleton_.CalculateDistance();
    CalculateThickness(shoot_growth_controller);
    const auto& sorted_internode_list = shoot_skeleton_.PeekSortedNodeList();
    for (auto it = sorted_internode_list.rbegin(); it != sorted_internode_list.rend(); ++it) {
      CalculateBiomass(*it, shoot_growth_controller);
      CalculateSaggingStress(*it, shoot_growth_controller);
    }
    CalculateLevel();
    CalculateTransform(shoot_growth_controller, true);
  };

  internode_order_counts.clear();
  fruit_count_ = leaf_count_ = 0;
  {
    int max_order = 0;
    const auto& sorted_flow_list = shoot_skeleton_.PeekSortedFlowList();
    for (const auto& flow_handle : sorted_flow_list) {
      auto& flow = shoot_skeleton_.RefFlow(flow_handle);
      auto& flow_data = flow.data;
      if (flow.GetParentHandle() == -1) {
        flow_data.order = 0;
      } else {
        const auto& parent_flow = shoot_skeleton_.RefFlow(flow.GetParentHandle());
        if (flow.IsApical())
          flow_data.order = parent_flow.data.order;
        else
          flow_data.order = parent_flow.data.order + 1;
      }
      max_order = glm::max(max_order, flow_data.order);
    }
    internode_order_counts.resize(max_order + 1);
    std::fill(internode_order_counts.begin(), internode_order_counts.end(), 0);
    const auto& sorted_internode_list = shoot_skeleton_.PeekSortedNodeList();
    for (const auto& internode_handle : sorted_internode_list) {
      auto& internode = shoot_skeleton_.RefNode(internode_handle);
      internode.info.leaves = shoot_growth_controller.m_leaf(internode);
      const auto order = shoot_skeleton_.RefFlow(internode.GetFlowHandle()).data.order;
      internode.data.order = order;
      internode_order_counts[order]++;

      for (const auto& bud : internode.data.buds) {
        if (bud.status != BudStatus::Died || bud.reproductive_module.maturity <= 0)
          continue;
        if (bud.type == BudType::Fruit) {
          fruit_count_++;
        } else if (bud.type == BudType::Leaf) {
          leaf_count_++;
        }
      }
    }
    shoot_skeleton_.CalculateFlows();
  }
}

float TreeModel::GetSubTreeMaxAge(const SkeletonNodeHandle base_internode_handle) const {
  const auto sorted_sub_tree_internode_list = shoot_skeleton_.GetSubTree(base_internode_handle);
  float max_age = 0.0f;

  for (const auto& internode_handle : sorted_sub_tree_internode_list) {
    const auto age = shoot_skeleton_.PeekNode(internode_handle).data.start_age;
    max_age = glm::max(age, max_age);
  }
  return max_age;
}

bool TreeModel::Reduce(const ShootGrowthController& shoot_growth_controller, const SkeletonNodeHandle base_internode_handle,
                       float target_age) {
  const auto sorted_sub_tree_internode_list = shoot_skeleton_.GetSubTree(base_internode_handle);
  if (sorted_sub_tree_internode_list.size() == 1)
    return false;
  bool reduced = false;
  for (auto it = sorted_sub_tree_internode_list.rbegin(); it != sorted_sub_tree_internode_list.rend(); ++it) {
    auto& internode = shoot_skeleton_.RefNode(*it);
    if (internode.data.start_age > target_age) {
      if (const auto parent_handle = internode.GetParentHandle(); parent_handle != -1) {
        auto& parent = shoot_skeleton_.RefNode(parent_handle);
        parent.info.thickness = shoot_growth_controller.m_endNodeThickness;
        parent.data.buds[internode.data.index_of_parent_bud].status = BudStatus::Dormant;
      }
      PruneInternode(*it);
      reduced = true;
    }
  }

  if (reduced)
    shoot_skeleton_.SortLists();
  ShootGrowthPostProcess(shoot_growth_controller);
  return reduced;
}

void TreeModel::CalculateTransform(const ShootGrowthController& shoot_growth_controller, bool sagging) {
  const auto& sorted_internode_list = shoot_skeleton_.PeekSortedNodeList();
  for (const auto& internode_handle : sorted_internode_list) {
    auto& internode = shoot_skeleton_.RefNode(internode_handle);
    auto& internode_data = internode.data;
    auto& internode_info = internode.info;

    internode_info.length =
        internode_data.internode_length * glm::pow(internode_info.thickness / shoot_growth_controller.m_endNodeThickness,
                                                   shoot_growth_controller.m_internodeLengthThicknessFactor);

    if (internode.GetParentHandle() == -1) {
      internode_info.global_position = internode_data.desired_global_position = glm::vec3(0.0f);
      internode_data.desired_local_rotation = glm::vec3(0.0f);
      internode_info.global_rotation = internode_info.regulated_global_rotation = internode_data.desired_global_rotation =
          glm::vec3(glm::radians(90.0f), 0.0f, 0.0f);
      internode_info.GetGlobalDirection() = glm::normalize(internode_info.global_rotation * glm::vec3(0, 0, -1));
    } else {
      auto& parent_internode = shoot_skeleton_.RefNode(internode.GetParentHandle());
      internode_data.sagging = shoot_growth_controller.m_sagging(internode);
      auto parent_global_rotation = parent_internode.info.global_rotation;
      internode_info.global_rotation = parent_global_rotation * internode_data.desired_local_rotation;
      auto front = glm::normalize(internode_info.global_rotation * glm::vec3(0, 0, -1));
      auto up = glm::normalize(internode_info.global_rotation * glm::vec3(0, 1, 0));
      if (sagging) {
        float dot_p = glm::abs(glm::dot(front, current_gravity_direction));
        ApplyTropism(current_gravity_direction, internode_data.sagging * (1.0f - dot_p), front, up);
        internode_info.global_rotation = glm::quatLookAt(front, up);
      }
      auto parent_regulated_up = parent_internode.info.regulated_global_rotation * glm::vec3(0, 1, 0);
      auto regulated_up = glm::normalize(glm::cross(glm::cross(front, parent_regulated_up), front));
      internode_info.regulated_global_rotation = glm::quatLookAt(front, regulated_up);

      internode_info.GetGlobalDirection() = glm::normalize(internode_info.global_rotation * glm::vec3(0, 0, -1));
      internode_info.global_position = parent_internode.info.global_position +
                                       parent_internode.info.length * parent_internode.info.GetGlobalDirection();

      if (shoot_growth_controller.m_branchPush && !internode.IsApical()) {
        const auto relative_front =
            glm::inverse(parent_internode.info.global_rotation) * internode_info.global_rotation * glm::vec3(0, 0, -1);
        auto parent_up = glm::normalize(parent_internode.info.global_rotation * glm::vec3(0, 1, 0));
        auto parent_left = glm::normalize(parent_internode.info.global_rotation * glm::vec3(1, 0, 0));
        auto parent_front = glm::normalize(parent_internode.info.global_rotation * glm::vec3(0, 0, -1));
        const auto sin_value = glm::sin(glm::acos(glm::dot(parent_front, front)));
        const auto offset = glm::normalize(glm::vec2(relative_front.x, relative_front.y)) * sin_value;
        internode_info.global_position += parent_left * parent_internode.info.thickness * offset.x;
        internode_info.global_position += parent_up * parent_internode.info.thickness * offset.y;
        internode_info.global_position += parent_front * parent_internode.info.thickness * sin_value;
      }

      internode_data.desired_global_rotation =
          parent_internode.data.desired_global_rotation * internode_data.desired_local_rotation;
      auto parent_desired_front = parent_internode.data.desired_global_rotation * glm::vec3(0, 0, -1);
      internode_data.desired_global_position =
          parent_internode.data.desired_global_position + parent_internode.info.length * parent_desired_front;
    }

    shoot_skeleton_.min = glm::min(shoot_skeleton_.min, internode_info.global_position);
    shoot_skeleton_.max = glm::max(shoot_skeleton_.max, internode_info.global_position);
    const auto end_position =
        internode_info.global_position + internode_info.length * internode_info.GetGlobalDirection();
    shoot_skeleton_.min = glm::min(shoot_skeleton_.min, end_position);
    shoot_skeleton_.max = glm::max(shoot_skeleton_.max, end_position);

    shoot_skeleton_.data.desired_min =
        glm::min(shoot_skeleton_.data.desired_min, internode_data.desired_global_position);
    shoot_skeleton_.data.desired_max =
        glm::max(shoot_skeleton_.data.desired_max, internode_data.desired_global_position);
    const auto desired_global_direction = internode_data.desired_global_rotation * glm::vec3(0, 0, -1);
    const auto desired_end_position =
        internode_data.desired_global_position + internode_info.length * desired_global_direction;
    shoot_skeleton_.data.desired_min = glm::min(shoot_skeleton_.data.desired_min, desired_end_position);
    shoot_skeleton_.data.desired_max = glm::max(shoot_skeleton_.data.desired_max, desired_end_position);
  }
}

bool TreeModel::ElongateInternode(float extended_length, SkeletonNodeHandle internode_handle,
                                  const ShootGrowthController& shoot_growth_controller, float& collected_inhibitor) {
  bool graph_changed = false;
  auto& internode = shoot_skeleton_.RefNode(internode_handle);
  const auto internode_length = shoot_growth_controller.m_internodeLength;
  auto& internode_data = internode.data;
  const auto& internode_info = internode.info;
  internode_data.internode_length += extended_length;
  const float extra_length = internode_data.internode_length - internode_length;
  // If we need to add a new end node
  assert(internode_data.buds.size() == 1);
  if (extra_length >= 0) {
    graph_changed = true;
    internode_data.internode_length = internode_length;
    const auto desired_global_rotation = internode_info.global_rotation * internode_data.buds.front().local_rotation;
    auto desired_global_front = desired_global_rotation * glm::vec3(0, 0, -1);
    auto desired_global_up = desired_global_rotation * glm::vec3(0, 1, 0);
    if (internode_handle != 0) {
      ApplyTropism(-current_gravity_direction, shoot_growth_controller.m_gravitropism(internode), desired_global_front,
                   desired_global_up);
      ApplyTropism(internode_data.light_direction, shoot_growth_controller.m_phototropism(internode), desired_global_front,
                   desired_global_up);
    }
    // First, remove only apical bud.
    internode.data.buds.clear();
    // Allocate Lateral bud for current internode

    const auto lateral_bud_count = shoot_growth_controller.m_lateralBudCount(internode);
    for (int i = 0; i < lateral_bud_count; i++) {
      internode_data.buds.emplace_back();
      auto& new_lateral_bud = internode_data.buds.back();
      new_lateral_bud.type = BudType::Lateral;
      new_lateral_bud.status = BudStatus::Dormant;
      new_lateral_bud.local_rotation = glm::vec3(0.f, glm::radians(shoot_growth_controller.m_branchingAngle(internode)),
                                                glm::linearRand(0.0f, 360.0f));
    }

    // Allocate Fruit bud for current internode
    {
      constexpr auto fruit_bud_count = 1;
      for (int i = 0; i < fruit_bud_count; i++) {
        internode_data.buds.emplace_back();
        auto& new_fruit_bud = internode_data.buds.back();
        new_fruit_bud.type = BudType::Fruit;
        new_fruit_bud.status = BudStatus::Dormant;
        new_fruit_bud.local_rotation = glm::vec3(glm::radians(shoot_growth_controller.m_branchingAngle(internode)), 0.0f,
                                                glm::radians(glm::linearRand(0.0f, 360.0f)));
      }
    }
    // Allocate Leaf bud for current internode
    {
      constexpr auto leaf_bud_count = 1;
      for (int i = 0; i < leaf_bud_count; i++) {
        internode_data.buds.emplace_back();
        auto& new_leaf_bud = internode_data.buds.back();
        // Hack: Leaf bud will be given vigor for the first time.
        new_leaf_bud.type = BudType::Leaf;
        new_leaf_bud.status = BudStatus::Dormant;
        new_leaf_bud.local_rotation = glm::vec3(glm::radians(shoot_growth_controller.m_branchingAngle(internode)), 0.0f,
                                               glm::radians(glm::linearRand(0.0f, 360.0f)));
      }
    }

    // Create new internode
    const auto new_internode_handle = shoot_skeleton_.Extend(internode_handle, false);
    auto& old_internode = shoot_skeleton_.RefNode(internode_handle);
    auto& new_internode = shoot_skeleton_.RefNode(new_internode_handle);

    new_internode.data = {};
    new_internode.data.index_of_parent_bud = 0;
    new_internode.data.light_intensity = old_internode.data.light_intensity;
    new_internode.data.light_direction = old_internode.data.light_direction;
    old_internode.data.finish_age = new_internode.data.start_age = age_;
    new_internode.data.finish_age = 0.0f;
    new_internode.data.order = old_internode.data.order;
    new_internode.data.inhibitor_sink = 0.0f;
    new_internode.data.internode_length = glm::clamp(extended_length, 0.0f, internode_length);
    new_internode.info.root_distance = old_internode.info.root_distance + new_internode.data.internode_length;
    new_internode.info.thickness = shoot_growth_controller.m_endNodeThickness;
    new_internode.info.global_rotation = glm::quatLookAt(desired_global_front, desired_global_up);
    new_internode.data.desired_local_rotation =
        glm::inverse(old_internode.info.global_rotation) * new_internode.info.global_rotation;
    if (shoot_growth_controller.m_m_apicalBudExtinctionRate(old_internode) < glm::linearRand(0.0f, 1.0f)) {
      // Allocate apical bud for new internode
      new_internode.data.buds.emplace_back();
      auto& new_apical_bud = new_internode.data.buds.back();
      new_apical_bud.type = BudType::Apical;
      new_apical_bud.status = BudStatus::Dormant;
      new_apical_bud.local_rotation = glm::vec3(glm::radians(shoot_growth_controller.m_apicalAngle(new_internode)), 0.0f,
                                               glm::radians(shoot_growth_controller.m_rollAngle(new_internode)));
      if (extra_length > internode_length) {
        float child_inhibitor = 0.0f;
        ElongateInternode(extra_length - internode_length, new_internode_handle, shoot_growth_controller, child_inhibitor);
        auto& current_new_internode = shoot_skeleton_.RefNode(new_internode_handle);
        current_new_internode.data.inhibitor_sink +=
            glm::max(0.0f, child_inhibitor * glm::clamp(1.0f - shoot_growth_controller.m_apicalDominanceLoss, 0.0f, 1.0f));
        collected_inhibitor +=
            current_new_internode.data.inhibitor_sink + shoot_growth_controller.m_apicalDominance(current_new_internode);
      } else {
        collected_inhibitor += shoot_growth_controller.m_apicalDominance(new_internode);
      }
    } else {
      new_internode.info.thickness = old_internode.info.thickness;
    }
  }
  return graph_changed;
}

bool TreeModel::GrowInternode(ClimateModel& climate_model, const SkeletonNodeHandle internode_handle,
                              const ShootGrowthController& shoot_growth_controller) {
  bool graph_changed = false;
  {
    auto& internode = shoot_skeleton_.RefNode(internode_handle);
    auto& internode_data = internode.data;
    internode_data.inhibitor_sink = 0;
    for (const auto& child_handle : internode.PeekChildHandles()) {
      auto& child_node = shoot_skeleton_.RefNode(child_handle);
      float child_node_inhibitor = 0.f;
      if (!child_node.data.buds.empty() && child_node.data.buds[0].type == BudType::Apical) {
        child_node_inhibitor = shoot_growth_controller.m_apicalDominance(child_node);
      }
      internode_data.inhibitor_sink +=
          glm::max(0.0f, (child_node_inhibitor + child_node.data.inhibitor_sink) *
                             glm::clamp(1.0f - shoot_growth_controller.m_apicalDominanceLoss, 0.0f, 1.0f));
    }
    if (!internode.data.buds.empty()) {
      const auto& bud = internode.data.buds[0];
      if (bud.type == BudType::Apical) {
        assert(internode.data.buds.size() == 1);
        float elongate_length = 0.0f;
        if (tree_growth_settings.use_space_colonization) {
          if (shoot_skeleton_.data.max_marker_count > 0)
            elongate_length = static_cast<float>(bud.marker_count) / shoot_skeleton_.data.max_marker_count *
                             shoot_growth_controller.m_internodeLength;
        } else {
          elongate_length = internode_data.growth_rate * current_delta_time_ * shoot_growth_controller.m_internodeLength *
                           shoot_growth_controller.m_internodeGrowthRate;
        }
        // Use up the vigor stored in this bud.
        float collected_inhibitor = 0.0f;
        graph_changed = ElongateInternode(elongate_length, internode_handle, shoot_growth_controller, collected_inhibitor) ||
                       graph_changed;
        shoot_skeleton_.RefNode(internode_handle).data.inhibitor_sink += glm::max(
            0.0f, collected_inhibitor * glm::clamp(1.0f - shoot_growth_controller.m_apicalDominanceLoss, 0.0f, 1.0f));
      }
    }
  }

  auto bud_size = shoot_skeleton_.RefNode(internode_handle).data.buds.size();
  for (int bud_index = 0; bud_index < bud_size; bud_index++) {
    auto& internode = shoot_skeleton_.RefNode(internode_handle);
    auto& bud = internode.data.buds[bud_index];
    const auto& internode_data = internode.data;
    const auto& internode_info = internode.info;
    if (bud.type == BudType::Lateral && bud.status == BudStatus::Dormant) {
      float flush_probability = bud.flushing_rate = shoot_growth_controller.m_lateralBudFlushingRate(internode);
      if (tree_growth_settings.use_space_colonization) {
        if (shoot_skeleton_.data.max_marker_count > 0)
          flush_probability *= static_cast<float>(bud.marker_count) / shoot_skeleton_.data.max_marker_count;
      } else {
        flush_probability *=
            internode_data.growth_rate * current_delta_time_ * shoot_growth_controller.m_internodeGrowthRate;
      }
      if (flush_probability >= glm::linearRand(0.0f, 1.0f)) {
        graph_changed = true;
        // Prepare information for new internode
        const auto desired_global_rotation = internode_info.global_rotation * bud.local_rotation;
        auto desired_global_front = desired_global_rotation * glm::vec3(0, 0, -1);
        auto desired_global_up = desired_global_rotation * glm::vec3(0, 1, 0);
        ApplyTropism(-current_gravity_direction, shoot_growth_controller.m_gravitropism(internode), desired_global_front,
                     desired_global_up);
        ApplyTropism(internode_data.light_direction, shoot_growth_controller.m_phototropism(internode),
                     desired_global_front, desired_global_up);
        if (const auto horizontal_direction = glm::vec3(desired_global_front.x, 0.0f, desired_global_front.z); glm::length(horizontal_direction) > glm::epsilon<float>()) {
          ApplyTropism(glm::normalize(horizontal_direction), shoot_growth_controller.m_horizontalTropism(internode),
                       desired_global_front, desired_global_up);
        }
        // Remove current lateral bud.
        internode.data.buds[bud_index] = internode.data.buds.back();
        internode.data.buds.pop_back();
        bud_index--;
        bud_size--;

        // Create new internode
        const auto new_internode_handle = shoot_skeleton_.Extend(internode_handle, true);
        const auto& old_internode = shoot_skeleton_.PeekNode(internode_handle);
        auto& new_internode = shoot_skeleton_.RefNode(new_internode_handle);
        new_internode.data = {};
        new_internode.data.index_of_parent_bud = 0;
        new_internode.data.start_age = age_;
        new_internode.data.finish_age = 0.0f;
        new_internode.data.order = old_internode.data.order + 1;
        new_internode.data.internode_length = 0.0f;
        new_internode.info.root_distance = old_internode.info.root_distance;
        new_internode.info.thickness = shoot_growth_controller.m_endNodeThickness;
        new_internode.data.desired_local_rotation =
            glm::inverse(old_internode.info.global_rotation) * glm::quatLookAt(desired_global_front, desired_global_up);
        // Allocate apical bud
        new_internode.data.buds.emplace_back();
        auto& apical_bud = new_internode.data.buds.back();
        apical_bud.type = BudType::Apical;
        apical_bud.status = BudStatus::Dormant;
        apical_bud.local_rotation = glm::vec3(0.f);
      }
    }
  }
  return graph_changed;
}

bool TreeModel::GrowReproductiveModules(ClimateModel& climate_model, const SkeletonNodeHandle internode_handle,
                                        const ShootGrowthController& shoot_growth_controller) {
  constexpr bool status_changed = false;
  const auto bud_size = shoot_skeleton_.RefNode(internode_handle).data.buds.size();
  for (int bud_index = 0; bud_index < bud_size; bud_index++) {
    auto& internode = shoot_skeleton_.RefNode(internode_handle);
    auto& bud = internode.data.buds[bud_index];
    auto& internode_data = internode.data;
    auto& internode_info = internode.info;
    // Calculate vigor used for maintenance and development.
    if (bud.type == BudType::Fruit) {
      /*
      if (!seasonality)
      {
              const float maxMaturityIncrease = availableDevelopmentVigor /
      shoot_growth_controller.m_fruitVigorRequirement; const auto developmentVigor =
      bud.m_vigorSink.SubtractVigor(maxMaturityIncrease * shoot_growth_controller.m_fruitVigorRequirement);
      }
      else if (bud.status == BudStatus::Dormant) {
              const float flushProbability = current_delta_time_ *
      shoot_growth_controller.m_fruitBudFlushingProbability(internode); if (flushProbability >=
      glm::linearRand(0.0f, 1.0f))
              {
                      bud.status = BudStatus::Flushed;
              }
      }
      else if (bud.status == BudStatus::Flushed)
      {
              //Make the fruit larger;
              const float maxMaturityIncrease = availableDevelopmentVigor /
      shoot_growth_controller.m_fruitVigorRequirement; const float maturityIncrease = glm::min(maxMaturityIncrease,
      glm::min(current_delta_time_ * shoot_growth_controller.m_fruitGrowthRate, 1.0f -
      bud.reproductive_module.maturity)); bud.reproductive_module.maturity += maturityIncrease; const auto
      developmentVigor = bud.m_vigorSink.SubtractVigor(maturityIncrease *
      shoot_growth_controller.m_fruitVigorRequirement); auto fruitSize = shoot_growth_controller.maxFruitSize *
      glm::sqrt(bud.reproductive_module.maturity); float angle = glm::radians(glm::linearRand(0.0f, 360.0f));
              glm::quat rotation = internode_data.desired_local_rotation * bud.local_rotation;
              auto up = rotation * glm::vec3(0, 1, 0);
              auto front = rotation * glm::vec3(0, 0, -1);
              ApplyTropism(internode_data.light_direction, 0.3f, up, front);
              rotation = glm::quatLookAt(front, up);
              auto fruitPosition = internode_info.global_position + front * (fruitSize.z * 1.f);
              bud.reproductive_module.transform = glm::translate(fruitPosition) *
      glm::mat4_cast(glm::quat(glm::vec3(0.0f))) * glm::scale(fruitSize);

              bud.reproductive_module.health -= current_delta_time_ * shoot_growth_controller.m_fruitDamage(internode);
              bud.reproductive_module.health = glm::clamp(bud.reproductive_module.health, 0.0f, 1.0f);

              //Handle fruit drop here.
              if (bud.reproductive_module.maturity >= 0.95f || bud.reproductive_module.health <= 0.05f)
              {
                      auto dropProbability = current_delta_time_ *
      shoot_growth_controller.m_fruitFallProbability(internode); if (dropProbability >= glm::linearRand(0.0f, 1.0f))
                      {
                              bud.status = BudStatus::Died;
                              shoot_skeleton_.data.m_droppedFruits.emplace_back(bud.reproductive_module);
                              bud.reproductive_module.Reset();
                      }
              }

      }*/
    } else if (bud.type == BudType::Leaf) {
      if (bud.status == BudStatus::Dormant) {
        if (const float flush_probability = current_delta_time_ * 1.; flush_probability >= glm::linearRand(0.0f, 1.0f)) {
          bud.status = BudStatus::Died;
        }
      } else if (bud.status == BudStatus::Died) {
        // Make the leaf larger
        // const float maxMaturityIncrease = availableDevelopmentVigor / shoot_growth_controller.m_leafVigorRequirement;
        // const float maturityIncrease = glm::min(maxMaturityIncrease, glm::min(current_delta_time_ *
        // shoot_growth_controller.m_leafGrowthRate, 1.0f - bud.reproductive_module.maturity));
        // bud.reproductive_module.maturity += maturityIncrease;
        // const auto developmentVigor = bud.m_vigorSink.SubtractVigor(maturityIncrease *
        // shoot_growth_controller.m_leafVigorRequirement);
        /*
        auto leafSize = shoot_growth_controller.maxLeafSize * glm::sqrt(bud.reproductive_module.maturity);
        glm::quat rotation = internode_data.desired_local_rotation * bud.local_rotation;
        auto up = rotation * glm::vec3(0, 1, 0);
        auto front = rotation * glm::vec3(0, 0, -1);
        ApplyTropism(internode_data.light_direction, 0.3f, up, front);
        rotation = glm::quatLookAt(front, up);
        auto foliagePosition = internode_info.global_position + front * (leafSize.z);
        bud.reproductive_module.transform = glm::translate(foliagePosition) * glm::mat4_cast(rotation) *
        glm::scale(leafSize);

        bud.reproductive_module.health -= current_delta_time_ * shoot_growth_controller.m_leafDamage(internode);
        bud.reproductive_module.health = glm::clamp(bud.reproductive_module.health, 0.0f, 1.0f);

        //Handle leaf drop here.
        if (bud.reproductive_module.health <= 0.05f)
        {
                const auto dropProbability = current_delta_time_ *
        shoot_growth_controller.m_leafFallProbability(internode); if (dropProbability >= glm::linearRand(0.0f, 1.0f))
                {
                        bud.status = BudStatus::Died;
                        shoot_skeleton_.data.m_droppedLeaves.emplace_back(bud.reproductive_module);
                        bud.reproductive_module.Reset();
                }
        }*/
      }
    }
  }
  return status_changed;
}

void TreeModel::CalculateLevel() {
  auto& sorted_internode_list = shoot_skeleton_.PeekSortedNodeList();
  for (const auto& internode_handle : sorted_internode_list) {
    auto& node = shoot_skeleton_.RefNode(internode_handle);
    if (node.GetParentHandle() == -1) {
      node.data.level = 0;
    } else {
      float max_biomass = 0.0f;
      SkeletonNodeHandle max_child = -1;
      for (const auto& child_handle : node.PeekChildHandles()) {
        auto& child_node = shoot_skeleton_.PeekNode(child_handle);
        if (const auto child_biomass = child_node.data.descendant_total_biomass + child_node.data.biomass; child_biomass > max_biomass) {
          max_biomass = child_biomass;
          max_child = child_handle;
        }
      }
      for (const auto& child_handle : node.PeekChildHandles()) {
        auto& child_node = shoot_skeleton_.RefNode(child_handle);
        if (child_handle == max_child) {
          child_node.data.level = node.data.level;
          child_node.data.max_child = true;
        } else {
          child_node.data.level = node.data.level + 1;
          child_node.data.max_child = false;
        }
      }
    }
    shoot_skeleton_.data.max_level = glm::max(shoot_skeleton_.data.max_level, node.data.level);
    shoot_skeleton_.data.max_order = glm::max(shoot_skeleton_.data.max_order, node.data.order);
  }
}

void TreeModel::CalculateGrowthRate(const std::vector<SkeletonNodeHandle>& sorted_internode_list, const float factor) {
  const float clamped_factor = glm::clamp(factor, 0.0f, 1.0f);
  for (const auto& internode_handle : sorted_internode_list) {
    auto& node = shoot_skeleton_.RefNode(internode_handle);
    // You cannot give more than enough resources.
    node.data.growth_rate = clamped_factor * node.data.desired_growth_rate;
  }
}

float TreeModel::CalculateGrowthPotential(const std::vector<SkeletonNodeHandle>& sorted_internode_list,
                                          const ShootGrowthController& shoot_growth_controller) {
  const float apical_control = shoot_growth_controller.m_apicalControl;
  const float root_distance_control = shoot_growth_controller.m_rootDistanceControl;
  const float height_control = shoot_growth_controller.m_heightControl;

  float max_grow_potential = 0.0f;

  std::vector<float> apical_control_values{};

  const auto max_val = shoot_growth_controller.m_useLevelForApicalControl ? shoot_skeleton_.data.max_level
                                                                       : shoot_skeleton_.data.max_order;
  apical_control_values.resize(max_val + 1);
  apical_control_values[0] = 1.0f;
  for (int i = 1; i < (max_val + 1); i++) {
    apical_control_values[i] = apical_control_values[i - 1] * apical_control;
  }

  for (const auto& internode_handle : sorted_internode_list) {
    auto& node = shoot_skeleton_.RefNode(internode_handle);

    float local_apical_control;
    if (apical_control > 1.0f) {
      local_apical_control =
          1.0f / apical_control_values[(shoot_growth_controller.m_useLevelForApicalControl ? node.data.level
                                                                                       : node.data.order)];
    } else if (apical_control < 1.0f) {
      local_apical_control =
          apical_control_values[max_val - (shoot_growth_controller.m_useLevelForApicalControl ? node.data.level
                                                                                         : node.data.order)];
    } else {
      local_apical_control = 1.0f;
    }
    float local_root_distance_control;
    if (root_distance_control != 0.f) {
      float distance = node.info.root_distance + node.info.length;
      if (distance == 0.f)
        distance = 1.f;
      local_root_distance_control = glm::pow(1.f / distance, root_distance_control);
    } else {
      local_root_distance_control = 1.f;
    }
    float local_height_control;
    if (height_control != 0.f) {
      float y = node.info.GetGlobalEndPosition().y;
      if (y == 0.f)
        y = 1.f;
      local_height_control = glm::pow(1.f / y, height_control);
    } else {
      local_height_control = 1.f;
    }
    node.data.growth_potential = local_apical_control * local_root_distance_control * local_height_control;
    max_grow_potential = glm::max(max_grow_potential, node.data.growth_potential);
  }
  float total_desired_growth_rate = 1.0f;
  for (const auto& internode_handle : sorted_internode_list) {
    auto& node = shoot_skeleton_.RefNode(internode_handle);
    if (max_grow_potential > 0.0f)
      node.data.growth_potential /= max_grow_potential;
    node.data.desired_growth_rate = node.data.light_intensity * node.data.growth_potential;
    total_desired_growth_rate += node.data.desired_growth_rate;
  }
  return total_desired_growth_rate;
}

void TreeModel::CalculateThickness(const ShootGrowthController& shoot_growth_controller) {
  auto& sorted_internode_list = shoot_skeleton_.PeekSortedNodeList();
  for (auto it = sorted_internode_list.rbegin(); it != sorted_internode_list.rend(); ++it) {
    const auto internode_handle = *it;
    auto& internode = shoot_skeleton_.RefNode(internode_handle);
    const auto& internode_data = internode.data;
    auto& internode_info = internode.info;
    float child_thickness_collection = 0.0f;
    for (const auto& i : internode.PeekChildHandles()) {
      const auto& child_internode = shoot_skeleton_.PeekNode(i);
      child_thickness_collection +=
          glm::pow(child_internode.info.thickness, 1.0f / shoot_growth_controller.m_thicknessAccumulationFactor);
    }
    child_thickness_collection += shoot_growth_controller.m_thicknessAgeFactor * shoot_growth_controller.m_endNodeThickness *
                                shoot_growth_controller.m_internodeGrowthRate * (age_ - internode_data.start_age);
    if (child_thickness_collection != 0.0f) {
      internode_info.thickness =
          glm::max(internode_info.thickness,
                   glm::pow(child_thickness_collection, shoot_growth_controller.m_thicknessAccumulationFactor));
    } else {
      internode_info.thickness = glm::max(internode_info.thickness, shoot_growth_controller.m_endNodeThickness);
    }
  }
}
void TreeModel::CalculateBiomass(SkeletonNodeHandle internode_handle,
                                 const ShootGrowthController& shoot_growth_controller) {
  auto& internode = shoot_skeleton_.RefNode(internode_handle);
  auto& internode_data = internode.data;
  const auto& internode_info = internode.info;
  internode_data.descendant_total_biomass = internode_data.biomass = 0.0f;
  const auto relative_thickness = internode_info.thickness / shoot_growth_controller.m_endNodeThickness;
  internode_data.biomass = internode_data.density * (relative_thickness * relative_thickness) *
                            internode_data.internode_length / shoot_growth_controller.m_internodeLength;
  glm::vec3 positioned_sum = glm::vec3(0.f);
  glm::vec3 desired_position_sum = glm::vec3(0.f);
  for (const auto& i : internode.PeekChildHandles()) {
    const auto& child_internode = shoot_skeleton_.RefNode(i);
    internode_data.descendant_total_biomass +=
        child_internode.data.descendant_total_biomass + child_internode.data.biomass;
    positioned_sum += child_internode.data.biomass *
                     (child_internode.info.global_position + child_internode.info.GetGlobalEndPosition()) * .5f;
    positioned_sum += child_internode.data.descendant_weight_center * child_internode.data.descendant_total_biomass;

    glm::vec3 child_desired_global_end_position = child_internode.data.desired_global_position;
    child_desired_global_end_position +=
        child_internode.info.length * (child_internode.data.desired_global_rotation * glm::vec3(0, 0, -1));
    desired_position_sum += child_internode.data.biomass *
                          (child_internode.data.desired_global_position + child_desired_global_end_position) * .5f;
    desired_position_sum +=
        child_internode.data.desired_descendant_weight_center * child_internode.data.descendant_total_biomass;
  }
  if (!internode.PeekChildHandles().empty() && internode_data.descendant_total_biomass != 0.f) {
    internode_data.descendant_weight_center = positioned_sum / internode_data.descendant_total_biomass;
    internode_data.desired_descendant_weight_center = desired_position_sum / internode_data.descendant_total_biomass;
  } else {
    internode_data.descendant_weight_center = internode.info.GetGlobalEndPosition();

    glm::vec3 desired_global_end_position = internode.data.desired_global_position;
    desired_global_end_position +=
        internode.info.length * (internode.data.desired_global_rotation * glm::vec3(0, 0, -1));
    internode_data.desired_descendant_weight_center = desired_global_end_position;
  }
}

void TreeModel::CalculateSaggingStress(const SkeletonNodeHandle internode_handle,
                                       const ShootGrowthController& shoot_growth_controller) {
  auto& internode = shoot_skeleton_.RefNode(internode_handle);
  if (internode.IsEndNode() || internode.info.thickness == 0.f || internode.info.length == 0.f) {
    internode.data.sagging_force = 0.f;
    internode.data.sagging_stress = 0.f;
    return;
  }
  const auto weight_center_relative_position = internode.info.global_position - internode.data.descendant_weight_center;
  // const auto horizontalDistanceToEnd = glm::length(glm::vec2(weightCenterRelativePosition.x,
  // weightCenterRelativePosition.z)); const auto front = glm::normalize(internode.info.global_rotation * glm::vec3(0,
  // 0, -1)); const auto frontVector = internode.info.length * front; const auto baseVector =
  // glm::vec2(glm::length(glm::vec2(frontVector.x, frontVector.z)), glm::abs(frontVector.y)); const auto combinedVector
  // = glm::vec2(horizontalDistanceToEnd, glm::abs(weightCenterRelativePosition.y)) + baseVector; const auto
  // projectedVector = baseVector * glm::dot(combinedVector, baseVector); const auto forceArm =
  // glm::length(projectedVector) / shoot_growth_controller.m_endNodeThickness;

  // const auto normalizedCombinedVector = glm::normalize(combinedVector);
  // const float cosTheta = glm::abs(glm::dot(normalizedCombinedVector, glm::normalize(glm::vec2(baseVector.y,
  // -baseVector.x)))); float sinTheta = 1.0f; if(cosTheta != 1.f) sinTheta = glm::sqrt(1 - cosTheta * cosTheta); const
  // float tangentForce = (internode.data.biomass + internode.data.descendant_total_biomass) * sinTheta *
  // glm::length(glm::vec2(0, -1) * glm::dot(normalizedCombinedVector, glm::vec2(0, -1)));
  const float cos_theta = glm::abs(glm::dot(glm::normalize(weight_center_relative_position), glm::vec3(0, -1, 0)));
  float sin_theta = 0.0f;
  if (cos_theta != 1.f)
    sin_theta = glm::sqrt(1 - cos_theta * cos_theta);
  const float tangent_force = (internode.data.biomass + internode.data.descendant_total_biomass) * sin_theta *
                             glm::length(weight_center_relative_position);
  internode.data.sagging_force = tangent_force;
  if (glm::isnan(internode.data.sagging_force)) {
    internode.data.sagging_force = 0.f;
  }
  const auto breaking_force = shoot_growth_controller.m_breakingForce(internode);
  internode.data.sagging_stress = internode.data.sagging_force / breaking_force;
}

void TreeModel::Clear() {
  shoot_skeleton_ = {};
  history_ = {};
  initialized_ = false;

  if (tree_growth_settings.use_space_colonization && !tree_growth_settings.space_colonization_auto_resize) {
    tree_occupancy_grid.ResetMarkers();
  } else {
    tree_occupancy_grid = {};
  }

  age_ = 0;
  iteration = 0;
}

int TreeModel::GetLeafCount() const {
  return leaf_count_;
}

int TreeModel::GetFruitCount() const {
  return fruit_count_;
}

bool TreeModel::PruneInternodes(const glm::mat4& global_transform, ClimateModel& climate_model,
                                const ShootGrowthController& shoot_growth_controller) {
  const auto& sorted_internode_list = shoot_skeleton_.PeekSortedNodeList();

  bool root_to_end_pruned = false;
  for (const auto& internode_handle : sorted_internode_list) {
    if (shoot_skeleton_.PeekNode(internode_handle).IsRecycled())
      continue;
    auto& internode = shoot_skeleton_.RefNode(internode_handle);
    if (internode_handle == 0)
      continue;
    if (internode.info.locked)
      continue;
    // Pruning here.
    bool pruning = false;
    if (internode.info.global_position.y <= 0.05f && internode.data.order != 0) {
      auto handle_walker = internode_handle;
      int i = 0;
      while (i < 4 && handle_walker != -1 && shoot_skeleton_.PeekNode(handle_walker).IsApical()) {
        handle_walker = shoot_skeleton_.PeekNode(handle_walker).GetParentHandle();
        i++;
      }
      if (handle_walker != -1) {
        if (auto& target_internode = shoot_skeleton_.PeekNode(handle_walker); target_internode.data.order != 0) {
          pruning = true;
        }
      }
    }
    if (const float pruning_probability =
        shoot_growth_controller.m_rootToEndPruningFactor(global_transform, climate_model, shoot_skeleton_, internode) *
        current_delta_time_; !pruning && pruning_probability > glm::linearRand(0.0f, 1.0f))
      pruning = true;

    if (pruning) {
      PruneInternode(internode_handle);
      root_to_end_pruned = true;
    }
  }
  bool end_to_root_pruned = false;
  for (auto it = sorted_internode_list.rbegin(); it != sorted_internode_list.rend(); ++it) {
    const auto internode_handle = *it;
    if (shoot_skeleton_.PeekNode(internode_handle).IsRecycled())
      continue;
    CalculateBiomass(internode_handle, shoot_growth_controller);
    CalculateSaggingStress(internode_handle, shoot_growth_controller);
    auto& internode = shoot_skeleton_.RefNode(internode_handle);
    if (internode_handle == 0)
      continue;
    if (internode.info.locked)
      continue;
    // Pruning here.
    bool pruning = false;
    if (internode.info.global_position.y <= 0.05f && internode.data.order != 0) {
      auto handle_walker = internode_handle;
      int i = 0;
      while (i < 4 && handle_walker != -1 && shoot_skeleton_.PeekNode(handle_walker).IsApical()) {
        handle_walker = shoot_skeleton_.PeekNode(handle_walker).GetParentHandle();
        i++;
      }
      if (handle_walker != -1) {
        if (auto& target_internode = shoot_skeleton_.PeekNode(handle_walker); target_internode.data.order != 0) {
          pruning = true;
        }
      }
    }

    if (const float pruning_probability =
        shoot_growth_controller.m_endToRootPruningFactor(global_transform, climate_model, shoot_skeleton_, internode) *
        current_delta_time_; !pruning && pruning_probability > glm::linearRand(0.0f, 1.0f))
      pruning = true;
    if (pruning) {
      PruneInternode(internode_handle);
      end_to_root_pruned = true;
    }
  }
  shoot_skeleton_.CalculateDistance();
  CalculateLevel();
  return root_to_end_pruned || end_to_root_pruned;
}

void TreeModel::SampleTemperature(const glm::mat4& global_transform, ClimateModel& climate_model) {
  const auto& sorted_internode_list = shoot_skeleton_.PeekSortedNodeList();
  for (auto it = sorted_internode_list.rbegin(); it != sorted_internode_list.rend(); ++it) {
    auto& internode = shoot_skeleton_.RefNode(*it);
    auto& internode_data = internode.data;
    auto& internode_info = internode.info;
    internode_data.temperature =
        climate_model.GetTemperature(global_transform * glm::translate(internode_info.global_position)[3]);
  }
}

ShootSkeleton& TreeModel::RefShootSkeleton() {
  return shoot_skeleton_;
}

const ShootSkeleton& TreeModel::PeekShootSkeleton(const int iteration) const {
  assert(iteration < 0 || iteration <= history_.size());
  if (iteration == history_.size() || iteration < 0)
    return shoot_skeleton_;
  return history_.at(iteration);
}

void TreeModel::ClearHistory() {
  history_.clear();
}

void TreeModel::Step() {
  history_.emplace_back(shoot_skeleton_);
  if (history_limit > 0) {
    while (history_.size() > history_limit) {
      history_.pop_front();
    }
  }
}

void TreeModel::Pop() {
  history_.pop_back();
}

int TreeModel::CurrentIteration() const {
  return history_.size();
}

void TreeModel::Reverse(int iteration) {
  assert(iteration >= 0 && iteration < history_.size());
  shoot_skeleton_ = history_[iteration];
  history_.erase((history_.begin() + iteration), history_.end());
}
