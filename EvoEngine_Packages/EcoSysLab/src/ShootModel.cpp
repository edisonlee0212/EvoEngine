//
// Created by lllll on 10/21/2022.
//
#include "ShootModel.hpp"
#include "SkeletonSerializer.hpp"
#include "Soil.hpp"
using namespace eco_sys_lab_package;

void ShootOrgan::Reset() {
  maturity = 0.0f;
  health = 1.0f;
  status = OrganStatus::Inactive;
  rotation = glm::vec3(0.0f);
  scale = glm::vec3(0.0f);
  position = glm::vec3(0.0f);
}

void ShootModel::ResetOrgans() {
  const auto& sorted_internode_list = shoot_skeleton_.PeekSortedNodeList();
  for (auto it = sorted_internode_list.rbegin(); it != sorted_internode_list.rend(); ++it) {
    auto& internode = shoot_skeleton_.RefNode(*it);
    for (auto& i : internode.data.leaves) {
      i.Reset();
    }
    for (auto& i : internode.data.fruits) {
      i.Reset();
    }
    for (auto& i : internode.data.flowers) {
      i.Reset();
    }
  }
}

void ShootModel::CreateOrgansForInternode(SkeletonNode<InternodeGrowthData>& internode,
                                          const FoliageController& foliage_controller,
                                          const ShootReproductionController& reproduction_controller) {
  if (foliage_controller.Initialized()) {
    const auto leaf_count = foliage_controller.leaf_count(random_engine_, shoot_skeleton_.data, internode);
    for (uint32_t i = 0; i < leaf_count; i++) {
      internode.data.leaves.emplace_back();
    }
  }
  if (reproduction_controller.Initialized()) {
    const auto reproductive_module_count =
        reproduction_controller.module_count(random_engine_, shoot_skeleton_.data, internode);
    for (uint32_t i = 0; i < reproductive_module_count; i++) {
      internode.data.fruits.emplace_back();
      internode.data.flowers.emplace_back();
      internode.data.flowers.back().fruit_index = i;
    }
  }
}

void ShootModel::RegisterVoxel(const glm::mat4& global_transform, ClimateModel& climate_model) {
  const auto& sorted_internode_list = shoot_skeleton_.PeekSortedNodeList();
  auto& environment_grid = climate_model.environment_grid;
  for (auto it = sorted_internode_list.rbegin(); it != sorted_internode_list.rend(); ++it) {
    const auto& internode = shoot_skeleton_.RefNode(*it);
    const auto& internode_info = internode.info;
    const float biomass = internode_info.thickness;
    const glm::vec3 world_position = global_transform * glm::vec4(internode_info.global_position, 1.0f);
    environment_grid.AddShadowValue(world_position, internode.data.shadow_size);
    environment_grid.AddBiomass(world_position, biomass);
    if (internode.IsEndNode()) {
      InternodeVoxelRegistration registration;
      registration.position = world_position;
      registration.node_handle = *it;
      registration.tree_skeleton_index = shoot_skeleton_.data.entity_index;
      registration.thickness = internode_info.thickness;
      environment_grid.AddNode(registration);
    }
  }
}

void ShootModel::HarvestFruits(const std::function<bool(const ShootOrgan& fruit)>& harvest_function) {
  const auto& sorted_internode_list = shoot_skeleton_.PeekSortedNodeList();

  for (auto it = sorted_internode_list.rbegin(); it != sorted_internode_list.rend(); ++it) {
    auto& internode = shoot_skeleton_.RefNode(*it);
    auto& internode_data = internode.data;
    for (auto& fruit : internode_data.fruits) {
      if (harvest_function(fruit)) {
        fruit.Reset();
      }
    }
  }
}

bool ShootModel::Grow(const float delta_time, const glm::mat4& global_transform, const ClimateModel& climate_model,
                      const VoxelSoilModel& soil_model, const ShootGrowthController& shoot_growth_controller,
                      const FoliageController& foliage_controller,
                      const ShootReproductionController& reproduction_controller,
                      const ShootPruningController& shoot_pruning_controller, const bool pruning) {
  if (!initialized_) {
    EVOENGINE_ERROR("ShootModel not initialized!")
    return false;
  }
  current_delta_time_ = delta_time;
  shoot_skeleton_.data.age += current_delta_time_;
  bool structure_changed = false;
  {
    const auto& sorted_node_list = shoot_skeleton_.PeekSortedNodeList();
    for (auto it = sorted_node_list.rbegin(); it != sorted_node_list.rend(); ++it) {
      const bool graph_changed =
          GrowInternode(*it, shoot_growth_controller, foliage_controller, reproduction_controller);
      structure_changed = structure_changed || graph_changed;
    }
    if (structure_changed) {
      shoot_skeleton_.SortLists();
    }
  }
  {
    const auto& sorted_node_list = shoot_skeleton_.PeekSortedNodeList();
    for (auto it = sorted_node_list.rbegin(); it != sorted_node_list.rend(); ++it) {
      if (foliage_controller.Initialized() &&
          GrowFoliage(current_delta_time_, climate_model, global_transform, *it, foliage_controller)) {
        structure_changed = true;
      }
      if (reproduction_controller.Initialized() &&
          GrowReproductiveModules(current_delta_time_, climate_model, global_transform, *it, reproduction_controller)) {
        structure_changed = true;
      }
    }
  }
  const int year = climate_model.time / 365.f;
  if (year != age_in_year_) {
    ResetOrgans();
    age_in_year_ = year;
    const auto& sorted_node_list = shoot_skeleton_.PeekSortedNodeList();
    for (auto it = sorted_node_list.rbegin(); it != sorted_node_list.rend(); ++it) {
      if (foliage_controller.Initialized()) {
        FormulateFoliage(climate_model, global_transform, *it, foliage_controller);
      }
      if (reproduction_controller.Initialized()) {
        FormulateReproductiveModules(climate_model, global_transform, *it, reproduction_controller);
      }
    }
    structure_changed = true;
  }
  if (pruning && shoot_pruning_controller.Initialized()) {
    CalculateGrowthData(shoot_growth_controller);
    if (PruneInternodes(global_transform, climate_model, soil_model, shoot_growth_controller,
                        shoot_pruning_controller)) {
      shoot_skeleton_.SortLists();
      structure_changed = true;
    }
  }
  CalculateGrowthData(shoot_growth_controller);

  iteration_++;
  return structure_changed;
}

bool ShootModel::Grow(const float delta_time, const SkeletonNodeHandle base_internode_handle,
                      const glm::mat4& global_transform, const ClimateModel& climate_model,
                      const VoxelSoilModel& soil_model, const ShootGrowthController& shoot_growth_controller,
                      const FoliageController& foliage_controller,
                      const ShootReproductionController& reproduction_controller,
                      const ShootPruningController& shoot_pruning_controller, const bool pruning) {
  if (!initialized_) {
    EVOENGINE_ERROR("ShootModel not initialized!")
    return false;
  }
  if (shoot_skeleton_.RefRawNodes().size() <= base_internode_handle)
    return false;

  current_delta_time_ = delta_time;
  shoot_skeleton_.data.age += current_delta_time_;
  bool tree_structure_changed = false;
  auto sorted_sub_tree_internode_list = shoot_skeleton_.GetSubTree(base_internode_handle);
  {
    for (auto it = sorted_sub_tree_internode_list.rbegin(); it != sorted_sub_tree_internode_list.rend(); ++it) {
      const bool graph_changed =
          GrowInternode(*it, shoot_growth_controller, foliage_controller, reproduction_controller);
      tree_structure_changed = tree_structure_changed || graph_changed;
    }
    if (tree_structure_changed) {
      shoot_skeleton_.SortLists();
      sorted_sub_tree_internode_list = shoot_skeleton_.GetSubTree(base_internode_handle);
    }
  }
  for (auto it = sorted_sub_tree_internode_list.rbegin(); it != sorted_sub_tree_internode_list.rend(); ++it) {
    if (GrowFoliage(current_delta_time_, climate_model, global_transform, *it, foliage_controller)) {
      tree_structure_changed = true;
    }
    if (GrowReproductiveModules(current_delta_time_, climate_model, global_transform, *it, reproduction_controller)) {
      tree_structure_changed = true;
    }
  }
  const int year = climate_model.time / 365.f;
  if (year != age_in_year_) {
    ResetOrgans();
    age_in_year_ = year;
    const auto& sorted_node_list = shoot_skeleton_.PeekSortedNodeList();
    for (auto it = sorted_node_list.rbegin(); it != sorted_node_list.rend(); ++it) {
      FormulateFoliage(climate_model, global_transform, *it, foliage_controller);
      FormulateReproductiveModules(climate_model, global_transform, *it, reproduction_controller);
    }
    tree_structure_changed = true;
  }
  if (pruning) {
    CalculateGrowthData(shoot_growth_controller);
    if (PruneInternodes(global_transform, climate_model, soil_model, shoot_growth_controller,
                        shoot_pruning_controller)) {
      shoot_skeleton_.SortLists();
      tree_structure_changed = true;
    }
  }
  CalculateGrowthData(shoot_growth_controller);
  iteration_++;

  return tree_structure_changed;
}

void ShootModel::Initialize(const ShootGrowthController& shoot_growth_controller,
                            const FoliageController& foliage_controller,
                            const ShootReproductionController& reproduction_controller) {
  if (initialized_)
    Clear();
  random_engine_ = std::mt19937(static_cast<uint32_t>(seed));
  {
    shoot_skeleton_ = ShootSkeleton(shoot_growth_controller.base_internode_count);
    shoot_skeleton_.SortLists();
    for (const auto& node_handle : shoot_skeleton_.PeekSortedNodeList()) {
      auto& node = shoot_skeleton_.RefNode(node_handle);
      shoot_growth_controller.base_node_initialization(random_engine_, shoot_skeleton_.data, node);
      CreateOrgansForInternode(node, foliage_controller, reproduction_controller);
    }
  }

  if (tree_growth_settings.use_space_colonization && tree_growth_settings.space_colonization_auto_resize) {
    const auto grid_radius = tree_growth_settings.space_colonization_detection_distance_factor *
                             shoot_growth_controller.base_internode_length;
    tree_occupancy_grid.Initialize(glm::vec3(-grid_radius, 0.0f, -grid_radius), glm::vec3(grid_radius),
                                   shoot_growth_controller.base_internode_length,
                                   tree_growth_settings.space_colonization_removal_distance_factor,
                                   tree_growth_settings.space_colonization_theta,
                                   tree_growth_settings.space_colonization_detection_distance_factor);
  }

  initialized_ = true;
}

Vigor ShootModel::SampleShootFlux(const glm::mat4& global_transform, const ClimateModel& climate_model,
                                  const ShootGrowthController& shoot_growth_controller) {
  shoot_skeleton_.SortLists();
  auto& shoot_data = shoot_skeleton_.data;
  shoot_data.max_marker_count = 0;
  const auto& sorted_internode_list = shoot_skeleton_.PeekSortedNodeList();
  if (tree_growth_settings.use_space_colonization) {
    if (tree_growth_settings.space_colonization_auto_resize) {
      auto min_bound = shoot_skeleton_.data.desired_min;
      auto max_bound = shoot_skeleton_.data.desired_max;
      const auto original_min = tree_occupancy_grid.GetMin();
      const auto original_max = tree_occupancy_grid.GetMax();
      if (const float detection_range = tree_growth_settings.space_colonization_detection_distance_factor *
                                        shoot_growth_controller.base_internode_length;
          min_bound.x - detection_range < original_min.x || min_bound.y < original_min.y ||
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
      voxel_grid.RefEach(internode_data.desired_global_position,
                         tree_growth_settings.space_colonization_removal_distance_factor *
                             shoot_growth_controller.base_internode_length,
                         [&](TreeOccupancyGridVoxelData& voxel_data) {
                           for (auto& marker : voxel_data.markers) {
                             const auto diff = marker.position - internode_data.desired_global_position;
                             const auto distance = glm::length(diff);
                             const auto direction = glm::normalize(diff);
                             if (distance < tree_growth_settings.space_colonization_detection_distance_factor *
                                                shoot_growth_controller.base_internode_length) {
                               if (marker.node_handle != -1)
                                 continue;
                               if (distance < tree_growth_settings.space_colonization_removal_distance_factor *
                                                  shoot_growth_controller.base_internode_length) {
                                 marker.node_handle = internode_handle;
                               } else {
                                 for (auto& bud : internode_data.buds) {
                                   if (auto bud_direction = glm::normalize(internode.info.global_rotation *
                                                                           bud.local_rotation * glm::vec3(0, 0, -1));
                                       glm::dot(direction, bud_direction) > dot_min) {
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
    internode_data.light_intake = 0.0f;
    internode_data.light_direction = -shoot_skeleton_.data.gravity_direction;
    bool sample_light_intensity = false;

    for (const auto& bud : internode_data.buds) {
      sample_light_intensity = true;
      if (tree_growth_settings.use_space_colonization) {
        shoot_data.max_marker_count = glm::max(shoot_data.max_marker_count, bud.marker_count);
      }
    }
    const glm::vec3 position = global_transform * glm::vec4(internode_info.global_position, 1.0f);
    if (sample_light_intensity) {
      internode_data.light_intake =
          glm::clamp(climate_model.environment_grid.Sample(position, internode_data.light_direction), 0.f, 1.f);
      if (internode_data.light_intake <= glm::epsilon<float>()) {
        internode_data.light_direction = glm::normalize(internode_info.GetGlobalDirection());
      }
    }
    internode_data.space_occupancy = climate_model.environment_grid.voxel_grid.Peek(position).total_biomass;
  }
  for (auto it = sorted_internode_list.rbegin(); it != sorted_internode_list.rend(); ++it) {
    auto& internode = shoot_skeleton_.RefNode(*it);
    auto& internode_data = internode.data;
    auto& internode_info = internode.info;
    internode_data.temperature =
        climate_model.GetHighTemp(global_transform * glm::translate(internode_info.global_position)[3]);
  }
  CalculateGrowthData(shoot_growth_controller);

  Vigor total_shoot_flux;
  total_shoot_flux.value = 0.0f;
  for (const auto& internode_handle : sorted_internode_list) {
    auto& internode = shoot_skeleton_.RefNode(internode_handle);
    const auto& internode_data = internode.data;
    total_shoot_flux.value += internode_data.light_intake;
  }

  for (auto it = sorted_internode_list.rbegin(); it != sorted_internode_list.rend(); ++it) {
    auto& internode = shoot_skeleton_.RefNode(*it);
    auto& internode_data = internode.data;
    internode_data.descendant_total_light_intake = glm::clamp(internode_data.light_intake, 0.f, 1.f);
    for (const auto& child_handle : internode.PeekChildHandles()) {
      const auto& child_internode_data = shoot_skeleton_.RefNode(child_handle).data;
      internode_data.descendant_total_light_intake +=
          child_internode_data.light_intake + child_internode_data.descendant_total_light_intake;
    }
  }
  return total_shoot_flux;
}

void ShootModel::DistributeVigor(const ShootGrowthController& shoot_growth_controller, const Vigor vigor) {
  const auto& sorted_internode_list = shoot_skeleton_.PeekSortedNodeList();
  float max_grow_potential = 0.0f;
  for (const auto& internode_handle : sorted_internode_list) {
    auto& internode = shoot_skeleton_.RefNode(internode_handle);
    internode.data.growth_potential =
        shoot_growth_controller.growth_potential(random_engine_, shoot_skeleton_, internode);
    max_grow_potential = glm::max(max_grow_potential, internode.data.growth_potential);
  }
  float total_desired_growth_rate = 1.0f;
  for (const auto& internode_handle : sorted_internode_list) {
    auto& node = shoot_skeleton_.RefNode(internode_handle);
    if (max_grow_potential > 0.0f)
      node.data.growth_potential /= max_grow_potential;
    node.data.desired_growth_rate = node.data.light_intake * node.data.growth_potential;
    total_desired_growth_rate += node.data.desired_growth_rate;
  }
  const float clamped_factor = vigor.value / total_desired_growth_rate;
  for (const auto& internode_handle : sorted_internode_list) {
    auto& node = shoot_skeleton_.RefNode(internode_handle);
    // You cannot give more than enough resources.
    node.data.growth_rate = clamped_factor * node.data.desired_growth_rate;
  }
}

float ShootModel::GetSubTreeMaxAge(const SkeletonNodeHandle base_internode_handle) const {
  const auto sorted_sub_tree_internode_list = shoot_skeleton_.GetSubTree(base_internode_handle);
  float max_age = 0.0f;

  for (const auto& internode_handle : sorted_sub_tree_internode_list) {
    const auto age = shoot_skeleton_.PeekNode(internode_handle).data.start_age;
    max_age = glm::max(age, max_age);
  }
  return max_age;
}

bool ShootModel::Reduce(const ShootGrowthController& shoot_growth_controller,
                        const SkeletonNodeHandle base_internode_handle, float target_age) {
  const auto sorted_sub_tree_internode_list = shoot_skeleton_.GetSubTree(base_internode_handle);
  if (sorted_sub_tree_internode_list.size() == 1)
    return false;
  bool reduced = false;
  std::vector<SkeletonNodeHandle> pruning_node_handles{};
  for (auto it = sorted_sub_tree_internode_list.rbegin(); it != sorted_sub_tree_internode_list.rend(); ++it) {
    auto& internode = shoot_skeleton_.RefNode(*it);
    if (internode.data.start_age > target_age) {
      if (const auto parent_handle = internode.GetParentHandle(); parent_handle != -1) {
        auto& parent = shoot_skeleton_.RefNode(parent_handle);
        parent.info.thickness = shoot_growth_controller.base_thickness;
        if (!parent.data.buds.empty())
          parent.data.buds[0].status = OrganStatus::Flushed;
      }

      pruning_node_handles.emplace_back(*it);
      reduced = true;
    }
  }

  shoot_skeleton_.RemoveNodes(pruning_node_handles);
  CalculateGrowthData(shoot_growth_controller);
  return reduced;
}

void ShootModel::CalculateTransform(const ShootGrowthController& shoot_growth_controller, bool sagging) {
  shoot_skeleton_.min = glm::vec3(FLT_MAX);
  shoot_skeleton_.max = glm::vec3(-FLT_MAX);
  shoot_skeleton_.data.desired_min = glm::vec3(FLT_MAX);
  shoot_skeleton_.data.desired_max = glm::vec3(-FLT_MAX);
  const auto& sorted_internode_list = shoot_skeleton_.PeekSortedNodeList();
  for (const auto& internode_handle : sorted_internode_list) {
    auto& internode = shoot_skeleton_.RefNode(internode_handle);
    auto& internode_data = internode.data;
    auto& internode_info = internode.info;

    internode_info.length = shoot_growth_controller.internode_length(random_engine_, shoot_skeleton_.data, internode);

    if (internode.GetParentHandle() != -1) {
      auto& parent_internode = shoot_skeleton_.RefNode(internode.GetParentHandle());
      internode_data.sagging = shoot_growth_controller.sagging(random_engine_, shoot_skeleton_.data, internode);
      auto parent_global_rotation = parent_internode.info.global_rotation;
      internode_info.global_rotation = parent_global_rotation * internode_data.desired_local_rotation;
      auto front = glm::normalize(internode_info.global_rotation * glm::vec3(0, 0, -1));
      auto up = glm::normalize(internode_info.global_rotation * glm::vec3(0, 1, 0));
      if (sagging) {
        float dot_p = glm::abs(glm::dot(front, shoot_skeleton_.data.gravity_direction));
        ApplyTropism(shoot_skeleton_.data.gravity_direction, internode_data.sagging * (1.0f - dot_p), front, up);
        internode_info.global_rotation = glm::quatLookAt(front, up);
      }
      auto parent_regulated_up = parent_internode.info.regulated_global_rotation * glm::vec3(0, 1, 0);
      auto regulated_up = glm::normalize(glm::cross(glm::cross(front, parent_regulated_up), front));
      internode_info.regulated_global_rotation = glm::quatLookAt(front, regulated_up);

      internode_info.GetGlobalDirection() = glm::normalize(internode_info.global_rotation * glm::vec3(0, 0, -1));
      internode_info.global_position = parent_internode.info.global_position +
                                       parent_internode.info.length * parent_internode.info.GetGlobalDirection();

      if (shoot_growth_controller.branch_push && !internode.IsApical()) {
        const auto relative_front =
            glm::inverse(parent_internode.info.global_rotation) * internode_info.global_rotation * glm::vec3(0, 0, -1);
        auto parent_up = glm::normalize(parent_internode.info.global_rotation * glm::vec3(0, 1, 0));
        auto parent_left = glm::normalize(parent_internode.info.global_rotation * glm::vec3(1, 0, 0));
        auto parent_front = glm::normalize(parent_internode.info.global_rotation * glm::vec3(0, 0, -1));
        const auto sin_value = glm::sin(glm::acos(glm::dot(parent_front, front)));
        const auto offset = glm::normalize(glm::vec2(relative_front.x, relative_front.y)) * sin_value;
        internode_info.global_position += parent_left * parent_internode.info.thickness * .5f * offset.x;
        internode_info.global_position += parent_up * parent_internode.info.thickness * .5f * offset.y;
        internode_info.global_position += parent_front * parent_internode.info.thickness * .5f * sin_value;
      }

      internode_data.desired_global_rotation =
          parent_internode.data.desired_global_rotation * internode_data.desired_local_rotation;
      auto parent_desired_front = parent_internode.data.desired_global_rotation * glm::vec3(0, 0, -1);
      internode_data.desired_global_position =
          parent_internode.data.desired_global_position + parent_internode.info.length * parent_desired_front;

      internode_data.shadow_size = shoot_growth_controller.shadow_size(random_engine_, shoot_skeleton_.data, internode);
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

bool ShootModel::ElongateInternode(const float extended_length, const SkeletonNodeHandle internode_handle,
                                   const ShootGrowthController& shoot_growth_controller,
                                   const FoliageController& foliage_controller,
                                   const ShootReproductionController& reproduction_controller,
                                   float& collected_inhibitor) {
  bool graph_changed = false;
  auto& internode = shoot_skeleton_.RefNode(internode_handle);
  const auto internode_length = shoot_growth_controller.base_internode_length;
  auto& internode_data = internode.data;
  internode_data.internode_length += extended_length;
  const float extra_length = internode_data.internode_length - internode_length;
  // If we need to add a new end node
  assert(internode_data.buds.size() == 1);
  if (extra_length >= 0) {
    graph_changed = true;
    internode_data.internode_length = internode_length;
    auto& apical_bud = internode.data.buds.front();

    apical_bud.status = OrganStatus::Dormant;
    auto desired_global_rotation = internode.info.global_rotation * apical_bud.local_rotation;
    // Allocate Lateral bud for current internode

    const auto lateral_bud_count =
        shoot_growth_controller.lateral_bud_count(random_engine_, shoot_skeleton_.data, internode);
    for (int i = 0; i < lateral_bud_count; i++) {
      internode.data.buds.emplace_back();
      auto& new_lateral_bud = internode.data.buds.back();
      new_lateral_bud.type = BudType::Lateral;
      new_lateral_bud.status = OrganStatus::Flushed;
      new_lateral_bud.index = i + 1;
      new_lateral_bud.local_rotation =
          shoot_growth_controller.bud_rotation(random_engine_, shoot_skeleton_.data, internode, new_lateral_bud);
    }
    // Create new internode
    const auto new_internode_handle = shoot_skeleton_.Extend(internode_handle, false);
    auto& old_internode = shoot_skeleton_.RefNode(internode_handle);
    auto& new_internode = shoot_skeleton_.RefNode(new_internode_handle);

    new_internode.data = {};
    new_internode.data.light_intake = old_internode.data.light_intake;
    new_internode.data.light_direction = old_internode.data.light_direction;
    old_internode.data.finish_age = new_internode.data.start_age = shoot_skeleton_.data.age;
    new_internode.data.finish_age = 0.0f;
    new_internode.info.order = old_internode.info.order;
    new_internode.data.inhibitor_sink = 0.0f;
    new_internode.data.internode_length = glm::clamp(extended_length, 0.0f, internode_length);
    new_internode.info.root_distance = old_internode.info.root_distance + new_internode.data.internode_length;

    if (internode_handle != 0) {
      shoot_growth_controller.tropism(random_engine_, shoot_skeleton_.data, old_internode, new_internode,
                                      desired_global_rotation);
    }
    new_internode.info.global_rotation = desired_global_rotation;
    new_internode.data.desired_local_rotation =
        glm::inverse(old_internode.info.global_rotation) * new_internode.info.global_rotation;

    new_internode.data.internode_thickness = 1.f;
    new_internode.info.thickness = shoot_growth_controller.base_thickness;

    if (shoot_growth_controller.bud_flushing_rate(random_engine_, shoot_skeleton_.data, old_internode) >=
        Random::Uniform(random_engine_, 0.f, 1.f)) {
      // Allocate apical bud for new internode
      new_internode.data.buds.emplace_back();
      auto& new_apical_bud = new_internode.data.buds.back();
      new_apical_bud.type = BudType::Apical;
      new_apical_bud.status = OrganStatus::Flushed;
      new_apical_bud.index = 0;
      new_apical_bud.local_rotation =
          shoot_growth_controller.bud_rotation(random_engine_, shoot_skeleton_.data, old_internode, new_apical_bud);

      CreateOrgansForInternode(new_internode, foliage_controller, reproduction_controller);

      if (extra_length > internode_length) {
        float child_inhibitor = 0.0f;
        ElongateInternode(extra_length - internode_length, new_internode_handle, shoot_growth_controller,
                          foliage_controller, reproduction_controller, child_inhibitor);
        auto& current_new_internode = shoot_skeleton_.RefNode(new_internode_handle);
        current_new_internode.data.inhibitor_sink +=
            glm::max(0.0f, shoot_growth_controller.growth_inhibitor_transport(random_engine_, shoot_skeleton_.data,
                                                                              child_inhibitor, current_new_internode));
        collected_inhibitor +=
            current_new_internode.data.inhibitor_sink +
            shoot_growth_controller.growth_inhibitor(random_engine_, shoot_skeleton_.data, current_new_internode);
      } else {
        collected_inhibitor +=
            shoot_growth_controller.growth_inhibitor(random_engine_, shoot_skeleton_.data, new_internode);
      }
    } else {
      new_internode.info.thickness = old_internode.info.thickness;
    }
  }
  return graph_changed;
}

void ShootModel::CalculateGrowthData(const ShootGrowthController& shoot_growth_controller) {
  const auto& sorted_internode_list = shoot_skeleton_.PeekSortedNodeList();
  {
    CalculateThickness(shoot_growth_controller);

    shoot_skeleton_.CalculateDistanceVolumeLevel();
    const auto volume_factor = 1.f / shoot_growth_controller.base_thickness / shoot_growth_controller.base_thickness /
                               shoot_growth_controller.base_internode_length;
    for (auto it = sorted_internode_list.rbegin(); it != sorted_internode_list.rend(); ++it) {
      CalculateBiomassFactor(*it, shoot_growth_controller, volume_factor);
    }
    CalculateTransform(shoot_growth_controller, true);
  }
  shoot_skeleton_.CalculateFlows();

  internode_order_counts.clear();
  internode_order_counts.resize(shoot_skeleton_.GetMaxOrder() + 1);
  std::fill(internode_order_counts.begin(), internode_order_counts.end(), 0);

  bud_count = 0;
  leaf_count_ = 0;
  fruit_count_ = 0;
  flower_count_ = 0;

  for (const auto& internode_handle : sorted_internode_list) {
    auto& internode = shoot_skeleton_.RefNode(internode_handle);
    internode_order_counts[internode.info.order]++;

    for (const auto& i : internode.data.buds) {
      if (i.status != OrganStatus::Inactive) {
        bud_count++;
      }
    }
    for (const auto& i : internode.data.leaves) {
      if (i.status != OrganStatus::Inactive) {
        leaf_count_++;
      }
    }
    for (const auto& i : internode.data.flowers) {
      if (i.status != OrganStatus::Inactive) {
        flower_count_++;
      }
    }
    for (const auto& i : internode.data.fruits) {
      if (i.status != OrganStatus::Inactive) {
        fruit_count_++;
      }
    }
  }
}

bool ShootModel::GrowInternode(const SkeletonNodeHandle internode_handle,
                               const ShootGrowthController& shoot_growth_controller,
                               const FoliageController& foliage_controller,
                               const ShootReproductionController& reproduction_controller) {
  bool graph_changed = false;
  {
    auto& internode = shoot_skeleton_.RefNode(internode_handle);
    auto& internode_data = internode.data;
    internode_data.inhibitor_sink = 0;
    for (const auto& child_handle : internode.PeekChildHandles()) {
      auto& child_node = shoot_skeleton_.RefNode(child_handle);
      float child_node_inhibitor = 0.f;
      if (!child_node.data.buds.empty()) {
        const auto& apical_bud = child_node.data.buds[0];
        if (apical_bud.type == BudType::Apical && apical_bud.status == OrganStatus::Flushed) {
          child_node_inhibitor =
              shoot_growth_controller.growth_inhibitor(random_engine_, shoot_skeleton_.data, child_node);
        }
      }

      internode_data.inhibitor_sink += glm::max(
          0.0f,
          shoot_growth_controller.growth_inhibitor_transport(
              random_engine_, shoot_skeleton_.data, child_node_inhibitor + child_node.data.inhibitor_sink, internode));
    }
    if (!internode.data.buds.empty()) {
      const auto& apical_bud = internode.data.buds[0];
      if (apical_bud.type == BudType::Apical && apical_bud.status == OrganStatus::Flushed) {
        assert(internode.data.buds.size() == 1);
        float elongate_length = 0.0f;
        if (tree_growth_settings.use_space_colonization) {
          if (shoot_skeleton_.data.max_marker_count > 0)
            elongate_length = static_cast<float>(apical_bud.marker_count) / shoot_skeleton_.data.max_marker_count *
                              shoot_growth_controller.base_internode_length;
        } else {
          elongate_length = internode_data.growth_rate * current_delta_time_ / 365.f *
                            shoot_growth_controller.base_internode_length *
                            shoot_growth_controller.internode_growth_rate;
        }
        // Use up the vigor stored in this bud.
        float collected_inhibitor = 0.0f;
        graph_changed = ElongateInternode(elongate_length, internode_handle, shoot_growth_controller,
                                          foliage_controller, reproduction_controller, collected_inhibitor) ||
                        graph_changed;
        auto& current_internode = shoot_skeleton_.RefNode(internode_handle);

        current_internode.data.inhibitor_sink +=
            glm::max(0.0f, shoot_growth_controller.growth_inhibitor_transport(random_engine_, shoot_skeleton_.data,
                                                                              collected_inhibitor, current_internode));
      }
    }
  }

  const auto bud_size = shoot_skeleton_.RefNode(internode_handle).data.buds.size();
  for (uint32_t bud_i = 0; bud_i < bud_size; bud_i++) {
    auto& internode = shoot_skeleton_.RefNode(internode_handle);
    auto& lateral_bud = internode.data.buds[bud_i];
    const auto& internode_data = internode.data;
    if (lateral_bud.type == BudType::Lateral && lateral_bud.status == OrganStatus::Flushed) {
      float flush_probability =
          shoot_growth_controller.bud_flushing_rate(random_engine_, shoot_skeleton_.data, internode);
      if (tree_growth_settings.use_space_colonization) {
        if (shoot_skeleton_.data.max_marker_count > 0)
          flush_probability *= static_cast<float>(lateral_bud.marker_count) / shoot_skeleton_.data.max_marker_count;
      } else {
        flush_probability *=
            internode_data.growth_rate * current_delta_time_ / 365.f * shoot_growth_controller.internode_growth_rate;
      }
      if (flush_probability >= Random::Uniform(random_engine_, 0.f, 1.f)) {
        graph_changed = true;
        // Prepare information for new internode
        // Remove current lateral bud.
        lateral_bud.status = OrganStatus::Dormant;
        auto desired_global_rotation = internode.info.global_rotation * lateral_bud.local_rotation;

        // Create new internode
        const auto new_internode_handle = shoot_skeleton_.Extend(internode_handle, true);
        const auto& old_internode = shoot_skeleton_.PeekNode(internode_handle);
        auto& new_internode = shoot_skeleton_.RefNode(new_internode_handle);
        new_internode.data = {};
        new_internode.data.start_age = shoot_skeleton_.data.age;
        new_internode.data.finish_age = 0.0f;
        new_internode.info.order = old_internode.info.order + 1;
        new_internode.data.internode_length = 0.0f;
        new_internode.info.root_distance = old_internode.info.root_distance;

        shoot_growth_controller.tropism(random_engine_, shoot_skeleton_.data, old_internode, new_internode,
                                        desired_global_rotation);
        new_internode.info.global_rotation = desired_global_rotation;
        new_internode.data.desired_local_rotation =
            glm::inverse(old_internode.info.global_rotation) * desired_global_rotation;

        // Allocate apical bud
        new_internode.data.buds.emplace_back();
        auto& apical_bud = new_internode.data.buds.back();
        apical_bud.type = BudType::Apical;

        apical_bud.status = OrganStatus::Flushed;
        apical_bud.local_rotation = glm::vec3(0.f);
        CreateOrgansForInternode(new_internode, foliage_controller, reproduction_controller);
        new_internode.data.internode_thickness = 1.f;
        new_internode.info.thickness = shoot_growth_controller.base_thickness;
      }
    }
  }
  return graph_changed;
}

bool ShootModel::GrowFoliage(float delta_time, const ClimateModel& climate_model, const glm::mat4& global_transform,
                             const SkeletonNodeHandle internode_handle, const FoliageController& foliage_controller) {
  bool status_changed = false;

  auto& internode = shoot_skeleton_.RefNode(internode_handle);
  for (auto& leaf : internode.data.leaves) {
    if (leaf.status != OrganStatus::Inactive) {
      foliage_controller.leaf_growth(random_engine_, global_transform, delta_time, leaf, climate_model, shoot_skeleton_,
                                     internode);
      if (leaf.health == 0.f) {
        leaf.hang_time -= delta_time;
        if (leaf.hang_time <= 0.f) {
          leaf.status = OrganStatus::Inactive;
          shoot_skeleton_.data.dropped_leaves.emplace_back(leaf);
        }
      }
    }
  }

  return status_changed;
}

void ShootModel::FormulateFoliage(const ClimateModel& climate_model, const glm::mat4& global_transform,
                                  SkeletonNodeHandle internode_handle, const FoliageController& foliage_controller) {
  auto& internode = shoot_skeleton_.RefNode(internode_handle);
  for (auto& leaf : internode.data.leaves) {
    if (leaf.status == OrganStatus::Inactive && leaf.maturity == 0.f) {
      if (foliage_controller.leaf_formulation(random_engine_, global_transform, leaf, climate_model, shoot_skeleton_,
                                              internode)) {
        leaf.status = OrganStatus::Dormant;
      }
    }
  }
}

bool ShootModel::GrowReproductiveModules(float delta_time, const ClimateModel& climate_model,
                                         const glm::mat4& global_transform, const SkeletonNodeHandle internode_handle,
                                         const ShootReproductionController& reproduction_controller) {
  bool status_changed = false;

  auto& internode = shoot_skeleton_.RefNode(internode_handle);
  for (auto& flower : internode.data.flowers) {
    if (flower.status != OrganStatus::Inactive) {
      reproduction_controller.flower_growth(random_engine_, global_transform, delta_time, flower, climate_model,
                                            shoot_skeleton_, internode);
      if (flower.maturity == 1.f && flower.health > 0.f) {
        flower.pollination_time -= delta_time;
        flower.hang_time -= delta_time;
        if (flower.pollination_time <= 0.f) {
          auto& fruit = internode.data.fruits[flower.fruit_index];
          if (fruit.status == OrganStatus::Inactive && fruit.maturity == 0.f) {
            if (reproduction_controller.fruit_formulation(random_engine_, global_transform, fruit, climate_model,
                                                          shoot_skeleton_, internode)) {
              fruit.status = OrganStatus::Dormant;
            }
          }
          status_changed = true;
        }
        if (flower.hang_time <= 0.f) {
          flower.status = OrganStatus::Inactive;
          shoot_skeleton_.data.dropped_flowers.emplace_back(flower);
          status_changed = true;
        }
      }
    }
  }
  for (auto& fruit : internode.data.fruits) {
    if (fruit.status != OrganStatus::Inactive) {
      reproduction_controller.fruit_growth(random_engine_, global_transform, delta_time, fruit, climate_model,
                                           shoot_skeleton_, internode);
      if (fruit.maturity >= 1.f) {
        fruit.hang_time -= delta_time;
        if (fruit.hang_time <= 0.f) {
          fruit.status = OrganStatus::Inactive;
          shoot_skeleton_.data.dropped_fruits.emplace_back(fruit);
        }
      }
    }
  }
  return status_changed;
}

void ShootModel::FormulateReproductiveModules(const ClimateModel& climate_model, const glm::mat4& global_transform,
                                              SkeletonNodeHandle internode_handle,
                                              const ShootReproductionController& reproduction_controller) {
  auto& internode = shoot_skeleton_.RefNode(internode_handle);
  for (auto& flower : internode.data.flowers) {
    if (flower.status == OrganStatus::Inactive && flower.maturity == 0.f) {
      if (reproduction_controller.flower_formulation(random_engine_, global_transform, flower, climate_model,
                                                     shoot_skeleton_, internode)) {
        flower.status = OrganStatus::Dormant;
      }
    }
  }
}

void ShootModel::CalculateThickness(const ShootGrowthController& shoot_growth_controller) {
  auto& sorted_internode_list = shoot_skeleton_.PeekSortedNodeList();
  for (auto it = sorted_internode_list.rbegin(); it != sorted_internode_list.rend(); ++it) {
    const auto internode_handle = *it;
    auto& internode = shoot_skeleton_.RefNode(internode_handle);
    auto& internode_info = internode.info;
    auto& internode_data = internode.data;
    float child_thickness_collection = 0.0f;

    const float thickness_accumulation_factor = glm::clamp(
        shoot_growth_controller.thickness_accumulation_factor(random_engine_, shoot_skeleton_.data, internode), 0.0f,
        1.0f);

    for (const auto& i : internode.PeekChildHandles()) {
      const auto& child_internode = shoot_skeleton_.PeekNode(i);
      child_thickness_collection +=
          glm::pow(child_internode.data.internode_thickness, 1.0f / thickness_accumulation_factor);
    }
    child_thickness_collection += shoot_growth_controller.thickness(random_engine_, shoot_skeleton_.data, internode);
    if (child_thickness_collection != 0.0f) {
      internode_data.internode_thickness = glm::max(
          internode_data.internode_thickness, glm::pow(child_thickness_collection, thickness_accumulation_factor));
    } else {
      internode_data.internode_thickness = glm::max(internode_data.internode_thickness, 1.f);
    }
    internode_info.thickness = internode_data.internode_thickness * shoot_growth_controller.base_thickness;
  }
}

void ShootModel::CalculateBiomassFactor(SkeletonNodeHandle internode_handle,
                                        const ShootGrowthController& shoot_growth_controller, float volume_factor) {
  auto& internode = shoot_skeleton_.RefNode(internode_handle);
  auto& internode_data = internode.data;
  internode_data.descendant_total_biomass_factor = internode_data.biomass_factor = 0.0f;
  internode_data.biomass_factor = internode.info.volume * volume_factor;
  //(internode_data.internode_thickness * internode_data.internode_thickness) *
  // internode_data.internode_length / shoot_growth_controller.base_internode_length;
  auto positioned_sum = glm::vec3(0.f);
  auto desired_position_sum = glm::vec3(0.f);
  for (const auto& i : internode.PeekChildHandles()) {
    const auto& child_internode = shoot_skeleton_.RefNode(i);
    internode_data.descendant_total_biomass_factor = internode.info.descendant_total_volume * volume_factor;
    positioned_sum += child_internode.data.biomass_factor *
                      (child_internode.info.global_position + child_internode.info.GetGlobalEndPosition()) * .5f;
    positioned_sum +=
        child_internode.data.descendant_weight_center * child_internode.data.descendant_total_biomass_factor;

    glm::vec3 child_desired_global_end_position = child_internode.data.desired_global_position;
    child_desired_global_end_position +=
        child_internode.info.length * (child_internode.data.desired_global_rotation * glm::vec3(0, 0, -1));
    desired_position_sum += child_internode.data.biomass_factor *
                            (child_internode.data.desired_global_position + child_desired_global_end_position) * .5f;
    desired_position_sum +=
        child_internode.data.desired_descendant_weight_center * child_internode.data.descendant_total_biomass_factor;
  }
  if (!internode.PeekChildHandles().empty() && internode_data.descendant_total_biomass_factor != 0.f) {
    internode_data.descendant_weight_center = positioned_sum / internode_data.descendant_total_biomass_factor;
    internode_data.desired_descendant_weight_center =
        desired_position_sum / internode_data.descendant_total_biomass_factor;
  } else {
    internode_data.descendant_weight_center = internode.info.GetGlobalEndPosition();

    glm::vec3 desired_global_end_position = internode.data.desired_global_position;
    desired_global_end_position +=
        internode.info.length * (internode.data.desired_global_rotation * glm::vec3(0, 0, -1));
    internode_data.desired_descendant_weight_center = desired_global_end_position;
  }
}

void ShootModel::CalculateSaggingStress(const SkeletonNodeHandle internode_handle,
                                        const ShootPruningController& shoot_pruning_controller) {
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
  // glm::length(projectedVector) / shoot_growth_controller.end_node_thickness;

  // const auto normalizedCombinedVector = glm::normalize(combinedVector);
  // const float cosTheta = glm::abs(glm::dot(normalizedCombinedVector, glm::normalize(glm::vec2(baseVector.y,
  // -baseVector.x)))); float sinTheta = 1.0f; if(cosTheta != 1.f) sinTheta = glm::sqrt(1 - cosTheta * cosTheta); const
  // float tangentForce = (internode.data.biomass_factor + internode.data.descendant_total_biomass_factor) * sinTheta *
  // glm::length(glm::vec2(0, -1) * glm::dot(normalizedCombinedVector, glm::vec2(0, -1)));
  const float cos_theta = glm::abs(glm::dot(glm::normalize(weight_center_relative_position), glm::vec3(0, -1, 0)));
  float sin_theta = 0.0f;
  if (cos_theta != 1.f)
    sin_theta = glm::sqrt(1 - cos_theta * cos_theta);
  const float tangent_force = (internode.data.biomass_factor + internode.data.descendant_total_biomass_factor) *
                              sin_theta * glm::length(weight_center_relative_position);
  internode.data.sagging_force = tangent_force;
  if (glm::isnan(internode.data.sagging_force)) {
    internode.data.sagging_force = 0.f;
  }
  const auto breaking_force = shoot_pruning_controller.breaking_force(random_engine_, shoot_skeleton_.data, internode);
  internode.data.sagging_stress = internode.data.sagging_force / breaking_force;
}

void ShootModel::Clear() {
  shoot_skeleton_ = {};
  shoot_history_ = {};
  initialized_ = false;

  if (tree_growth_settings.use_space_colonization && !tree_growth_settings.space_colonization_auto_resize) {
    tree_occupancy_grid.ResetMarkers();
  } else {
    tree_occupancy_grid = {};
  }
  bud_count = 0;
  leaf_count_ = 0;
  flower_count_ = 0;
  fruit_count_ = 0;
  iteration_ = 0;
}

int ShootModel::GetLeafCount() const {
  return leaf_count_;
}

int ShootModel::GetFlowerCount() const {
  return flower_count_;
}

int ShootModel::GetFruitCount() const {
  return fruit_count_;
}

bool ShootModel::PruneInternodes(const glm::mat4& global_transform, const ClimateModel& climate_model,
                                 const VoxelSoilModel& soil_model, const ShootGrowthController& shoot_growth_controller,
                                 const ShootPruningController& shoot_pruning_controller) {
  bool root_to_end_pruned = false;

  {
    const auto& sorted_internode_list = shoot_skeleton_.PeekSortedNodeList();
    std::vector<SkeletonNodeHandle> pruning_node_handles{};
    for (const auto& internode_handle : sorted_internode_list) {
      auto& internode = shoot_skeleton_.RefNode(internode_handle);
      internode.data.strength =
          shoot_pruning_controller.internode_strength(random_engine_, shoot_skeleton_.data, internode);
      if (internode_handle == 0)
        continue;
      if (internode.info.locked)
        continue;
      // Pruning here.
      bool pruning = false;
      if (internode.info.global_position.y <= 0.05f && internode.info.order != 0) {
        auto handle_walker = internode_handle;
        int i = 0;
        while (i < 4 && handle_walker != -1 && shoot_skeleton_.PeekNode(handle_walker).IsApical()) {
          handle_walker = shoot_skeleton_.PeekNode(handle_walker).GetParentHandle();
          i++;
        }
        if (handle_walker != -1) {
          if (auto& target_internode = shoot_skeleton_.PeekNode(handle_walker); target_internode.info.order != 0) {
            pruning = true;
          }
        }
      }

      if (const float pruning_probability =
              shoot_pruning_controller.base_to_end_pruning_factor(random_engine_, global_transform, climate_model,
                                                                  soil_model, shoot_skeleton_, internode) *
              current_delta_time_ / 365.f;
          !pruning && pruning_probability > Random::Uniform(random_engine_, 0.f, 1.f))
        pruning = true;

      if (pruning) {
        pruning_node_handles.emplace_back(internode_handle);
        root_to_end_pruned = true;
      }
    }
    shoot_skeleton_.RemoveNodes(pruning_node_handles);
  }

  bool end_to_root_pruned = false;
  {
    std::vector<SkeletonNodeHandle> pruning_node_handles{};
    const auto& sorted_internode_list = shoot_skeleton_.PeekSortedNodeList();
    const auto volume_factor = 1.f / shoot_growth_controller.base_thickness / shoot_growth_controller.base_thickness /
                               shoot_growth_controller.base_internode_length;
    for (auto it = sorted_internode_list.rbegin(); it != sorted_internode_list.rend(); ++it) {
      const auto internode_handle = *it;
      CalculateBiomassFactor(internode_handle, shoot_growth_controller, volume_factor);
      CalculateSaggingStress(internode_handle, shoot_pruning_controller);
      auto& internode = shoot_skeleton_.RefNode(internode_handle);
      if (internode_handle == 0)
        continue;
      if (internode.info.locked)
        continue;
      // Pruning here.
      bool pruning = false;
      if (internode.info.global_position.y <= 0.05f && internode.info.order != 0) {
        auto handle_walker = internode_handle;
        int i = 0;
        while (i < 4 && handle_walker != -1 && shoot_skeleton_.PeekNode(handle_walker).IsApical()) {
          handle_walker = shoot_skeleton_.PeekNode(handle_walker).GetParentHandle();
          i++;
        }
        if (handle_walker != -1) {
          if (auto& target_internode = shoot_skeleton_.PeekNode(handle_walker); target_internode.info.order != 0) {
            pruning = true;
          }
        }
      }
      if (const float pruning_probability =
              shoot_pruning_controller.end_to_base_pruning_factor(random_engine_, global_transform, climate_model,
                                                                  soil_model, shoot_skeleton_, internode) *
              current_delta_time_ / 365.f;
          !pruning && pruning_probability > Random::Uniform(random_engine_, 0.f, 1.f))
        pruning = true;
      if (pruning) {
        pruning_node_handles.emplace_back(internode_handle);
        end_to_root_pruned = true;
      }
    }

    shoot_skeleton_.RemoveNodes(pruning_node_handles);
  }
  shoot_skeleton_.CalculateDistanceVolumeLevel();
  return root_to_end_pruned || end_to_root_pruned;
}

ShootSkeleton& ShootModel::RefShootSkeleton() {
  return shoot_skeleton_;
}

const ShootSkeleton& ShootModel::PeekShootSkeleton(const int iteration) const {
  assert(iteration < 0 || iteration <= shoot_history_.size());
  if (iteration == shoot_history_.size() || iteration < 0)
    return shoot_skeleton_;
  return shoot_history_.at(iteration);
}

void ShootModel::ClearHistory() {
  shoot_history_.clear();
}

void ShootModel::Step() {
  shoot_history_.emplace_back(shoot_skeleton_);
  if (history_limit > 0) {
    while (shoot_history_.size() > history_limit) {
      shoot_history_.pop_front();
    }
  }
}

void ShootModel::Pop() {
  shoot_history_.pop_back();
}

int ShootModel::CurrentIteration() const {
  return shoot_history_.size();
}

void ShootModel::Reverse(int iteration) {
  assert(iteration >= 0 && iteration < shoot_history_.size());
  shoot_skeleton_ = shoot_history_[iteration];
  shoot_history_.erase((shoot_history_.begin() + iteration), shoot_history_.end());
}

void ShootModel::Save(const std::string& name, YAML::Emitter& out) const {
  out << YAML::Key << name << YAML::Value << YAML::BeginMap;
  {
    out << YAML::Key << "shoot_skeleton" << YAML::Value << YAML::BeginMap;
    {
      SkeletonSerializer<ShootGrowthData, ShootStemGrowthData, InternodeGrowthData>::Serialize(
          out, shoot_skeleton_,
          [&](YAML::Emitter& node_out, const InternodeGrowthData& node_data) {
            node_out << YAML::Key << "buds" << YAML::Value << YAML::BeginSeq;
            for (const auto& bud : node_data.buds) {
              node_out << YAML::BeginMap;
              {
                node_out << YAML::Key << "T" << YAML::Value << static_cast<unsigned>(bud.type);
                node_out << YAML::Key << "I" << YAML::Value << bud.index;
                node_out << YAML::Key << "S" << YAML::Value << static_cast<unsigned>(bud.status);
                node_out << YAML::Key << "LR" << YAML::Value << bud.local_rotation;
              }
              node_out << YAML::EndMap;
            }
            node_out << YAML::EndSeq;

            node_out << YAML::Key << "leaves" << YAML::Value << YAML::BeginSeq;
            for (const auto& leaf : node_data.leaves) {
              node_out << YAML::BeginMap;
              {
                node_out << YAML::Key << "M" << YAML::Value << leaf.maturity;
                node_out << YAML::Key << "H" << YAML::Value << leaf.health;
                node_out << YAML::Key << "R" << YAML::Value << leaf.rotation;
                node_out << YAML::Key << "P" << YAML::Value << leaf.position;
                node_out << YAML::Key << "C" << YAML::Value << leaf.scale;
                node_out << YAML::Key << "S" << YAML::Value << static_cast<unsigned>(leaf.status);
              }
              node_out << YAML::EndMap;
            }
            node_out << YAML::EndSeq;

            node_out << YAML::Key << "fruits" << YAML::Value << YAML::BeginSeq;
            for (const auto& fruit : node_data.fruits) {
              node_out << YAML::BeginMap;
              {
                node_out << YAML::Key << "M" << YAML::Value << fruit.maturity;
                node_out << YAML::Key << "H" << YAML::Value << fruit.health;
                node_out << YAML::Key << "R" << YAML::Value << fruit.rotation;
                node_out << YAML::Key << "P" << YAML::Value << fruit.position;
                node_out << YAML::Key << "C" << YAML::Value << fruit.scale;
                node_out << YAML::Key << "S" << YAML::Value << static_cast<unsigned>(fruit.status);
              }
              node_out << YAML::EndMap;
            }
            node_out << YAML::EndSeq;

            node_out << YAML::Key << "flower" << YAML::Value << YAML::BeginSeq;
            for (const auto& flowers : node_data.flowers) {
              node_out << YAML::BeginMap;
              {
                node_out << YAML::Key << "M" << YAML::Value << flowers.maturity;
                node_out << YAML::Key << "H" << YAML::Value << flowers.health;
                node_out << YAML::Key << "R" << YAML::Value << flowers.rotation;
                node_out << YAML::Key << "P" << YAML::Value << flowers.position;
                node_out << YAML::Key << "C" << YAML::Value << flowers.scale;
                node_out << YAML::Key << "S" << YAML::Value << static_cast<unsigned>(flowers.status);
              }
              node_out << YAML::EndMap;
            }
            node_out << YAML::EndSeq;
          },
          [&](YAML::Emitter& flow_out, const ShootStemGrowthData& flow_data) {
          },
          [&](YAML::Emitter& skeleton_out, const ShootGrowthData& skeleton_data) {
            skeleton_out << YAML::Key << "desired_min" << YAML::Value << skeleton_data.desired_min;
            skeleton_out << YAML::Key << "desired_max" << YAML::Value << skeleton_data.desired_max;
            skeleton_out << YAML::Key << "age" << YAML::Value << skeleton_data.age;
            skeleton_out << YAML::Key << "gravity_direction" << YAML::Value << skeleton_data.gravity_direction;
            const auto node_size = shoot_skeleton_.PeekRawNodes().size();
            auto internode_length = std::vector<float>(node_size);
            auto internode_thickness = std::vector<float>(node_size);
            auto start_age = std::vector<float>(node_size);
            auto finish_age = std::vector<float>(node_size);
            auto desired_local_rotation = std::vector<glm::quat>(node_size);
            auto desired_global_rotation = std::vector<glm::quat>(node_size);
            auto desired_global_position = std::vector<glm::vec3>(node_size);

            auto extra_mass = std::vector<float>(node_size);
            auto density = std::vector<float>(node_size);

            for (int node_index = 0; node_index < node_size; node_index++) {
              const auto& node = shoot_skeleton_.PeekRawNodes().at(node_index);
              internode_length.at(node_index) = node.data.internode_length;
              internode_thickness.at(node_index) = node.data.internode_thickness;
              start_age.at(node_index) = node.data.start_age;
              finish_age.at(node_index) = node.data.finish_age;
              desired_local_rotation.at(node_index) = node.data.desired_local_rotation;
              desired_global_rotation.at(node_index) = node.data.desired_global_rotation;
              desired_global_position.at(node_index) = node.data.desired_global_position;

              extra_mass.at(node_index) = node.data.extra_mass;
              density.at(node_index) = node.data.density;
            }
            if (node_size != 0) {
              skeleton_out << YAML::Key << "node.data.internode_length" << YAML::Value
                           << YAML::Binary(reinterpret_cast<const unsigned char*>(internode_length.data()),
                                           internode_length.size() * sizeof(float));
              skeleton_out << YAML::Key << "node.data.internode_thickness" << YAML::Value
                           << YAML::Binary(reinterpret_cast<const unsigned char*>(internode_thickness.data()),
                                           internode_length.size() * sizeof(float));
              skeleton_out << YAML::Key << "node.data.start_age" << YAML::Value
                           << YAML::Binary(reinterpret_cast<const unsigned char*>(start_age.data()),
                                           start_age.size() * sizeof(float));
              skeleton_out << YAML::Key << "node.data.finish_age" << YAML::Value
                           << YAML::Binary(reinterpret_cast<const unsigned char*>(finish_age.data()),
                                           finish_age.size() * sizeof(float));
              skeleton_out << YAML::Key << "node.data.desired_local_rotation" << YAML::Value
                           << YAML::Binary(reinterpret_cast<const unsigned char*>(desired_local_rotation.data()),
                                           desired_local_rotation.size() * sizeof(glm::quat));
              skeleton_out << YAML::Key << "node.data.desired_global_rotation" << YAML::Value
                           << YAML::Binary(reinterpret_cast<const unsigned char*>(desired_global_rotation.data()),
                                           desired_global_rotation.size() * sizeof(glm::quat));
              skeleton_out << YAML::Key << "node.data.desired_global_position" << YAML::Value
                           << YAML::Binary(reinterpret_cast<const unsigned char*>(desired_global_position.data()),
                                           desired_global_position.size() * sizeof(glm::vec3));

              skeleton_out << YAML::Key << "node.data.extra_mass" << YAML::Value
                           << YAML::Binary(reinterpret_cast<const unsigned char*>(extra_mass.data()),
                                           extra_mass.size() * sizeof(float));
              skeleton_out << YAML::Key << "node.data.density" << YAML::Value
                           << YAML::Binary(reinterpret_cast<const unsigned char*>(density.data()),
                                           density.size() * sizeof(float));
            }
          });
    }
    out << YAML::EndMap;
  }
  out << YAML::EndMap;
}

void ShootModel::Load(const std::string& name, const YAML::Node& in) {
  if (in[name]) {
    if (const auto& in_tree_model = in[name]) {
      const auto& in_shoot_skeleton = in_tree_model["shoot_skeleton"];
      SkeletonSerializer<ShootGrowthData, ShootStemGrowthData, InternodeGrowthData>::Deserialize(
          in_shoot_skeleton, shoot_skeleton_,
          [&](const YAML::Node& node_in, InternodeGrowthData& node_data) {
            node_data.buds.clear();
            if (node_in["buds"]) {
              const auto& in_buds = node_in["buds"];
              for (const auto& in_bud : in_buds) {
                node_data.buds.emplace_back();
                auto& bud = node_data.buds.back();
                if (in_bud["T"])
                  bud.type = static_cast<BudType>(in_bud["T"].as<unsigned>());
                if (in_bud["I"])
                  bud.index = in_bud["I"].as<int>();
                if (in_bud["S"])
                  bud.status = static_cast<OrganStatus>(in_bud["S"].as<unsigned>());
                if (in_bud["LR"])
                  bud.local_rotation = in_bud["LR"].as<glm::quat>();
              }
            }

            node_data.leaves.clear();
            if (node_in["leaves"]) {
              const auto& in_leaves = node_in["leaves"];
              for (const auto& in_leaf : in_leaves) {
                node_data.leaves.emplace_back();
                auto& leaf = node_data.leaves.back();
                if (in_leaf["M"])
                  leaf.maturity = in_leaf["M"].as<float>();
                if (in_leaf["H"])
                  leaf.health = in_leaf["H"].as<float>();
                if (in_leaf["R"])
                  leaf.rotation = in_leaf["R"].as<glm::quat>();
                if (in_leaf["P"])
                  leaf.position = in_leaf["P"].as<glm::vec3>();
                if (in_leaf["C"])
                  leaf.scale = in_leaf["C"].as<glm::vec3>();
                if (in_leaf["S"])
                  leaf.status = static_cast<OrganStatus>(in_leaf["S"].as<unsigned>());
              }
            }

            node_data.flowers.clear();
            if (node_in["flowers"]) {
              const auto& in_flowers = node_in["flowers"];
              for (const auto& in_flower : in_flowers) {
                node_data.flowers.emplace_back();
                auto& flower = node_data.flowers.back();
                if (in_flower["M"])
                  flower.maturity = in_flower["M"].as<float>();
                if (in_flower["H"])
                  flower.health = in_flower["H"].as<float>();
                if (in_flower["R"])
                  flower.rotation = in_flower["R"].as<glm::quat>();
                if (in_flower["P"])
                  flower.position = in_flower["P"].as<glm::vec3>();
                if (in_flower["C"])
                  flower.scale = in_flower["C"].as<glm::vec3>();
                if (in_flower["S"])
                  flower.status = static_cast<OrganStatus>(in_flower["S"].as<unsigned>());
              }
            }

            node_data.fruits.clear();
            if (node_in["fruits"]) {
              const auto& in_fruits = node_in["fruits"];
              for (const auto& in_fruit : in_fruits) {
                node_data.fruits.emplace_back();
                auto& fruit = node_data.fruits.back();
                if (in_fruit["M"])
                  fruit.maturity = in_fruit["M"].as<float>();
                if (in_fruit["H"])
                  fruit.health = in_fruit["H"].as<float>();
                if (in_fruit["R"])
                  fruit.rotation = in_fruit["R"].as<glm::quat>();
                if (in_fruit["P"])
                  fruit.position = in_fruit["P"].as<glm::vec3>();
                if (in_fruit["C"])
                  fruit.scale = in_fruit["C"].as<glm::vec3>();
                if (in_fruit["S"])
                  fruit.status = static_cast<OrganStatus>(in_fruit["S"].as<unsigned>());
              }
            }
          },
          [&](const YAML::Node& flow_in, ShootStemGrowthData& flow_data) {
          },
          [&](const YAML::Node& skeleton_in, ShootGrowthData& skeleton_data) {
            if (skeleton_in["desired_min"])
              skeleton_data.desired_min = skeleton_in["desired_min"].as<glm::vec3>();
            if (skeleton_in["desired_max"])
              skeleton_data.desired_max = skeleton_in["desired_max"].as<glm::vec3>();
            if (skeleton_in["age"])
              skeleton_data.age = skeleton_in["age"].as<float>();
            if (skeleton_in["gravity_direction"])
              skeleton_data.gravity_direction = skeleton_in["gravity_direction"].as<glm::vec3>();
            if (skeleton_in["node.data.internode_length"]) {
              auto list = std::vector<float>();
              const auto data = skeleton_in["node.data.internode_length"].as<YAML::Binary>();
              list.resize(data.size() / sizeof(float));
              std::memcpy(list.data(), data.data(), data.size());
              for (size_t i = 0; i < list.size(); i++) {
                auto& node = shoot_skeleton_.RefNode(i);
                node.data.internode_length = list[i];
              }
            }
            if (skeleton_in["node.data.internode_thickness"]) {
              auto list = std::vector<float>();
              const auto data = skeleton_in["node.data.internode_thickness"].as<YAML::Binary>();
              list.resize(data.size() / sizeof(float));
              std::memcpy(list.data(), data.data(), data.size());
              for (size_t i = 0; i < list.size(); i++) {
                auto& node = shoot_skeleton_.RefNode(i);
                node.data.internode_thickness = list[i];
              }
            }

            if (skeleton_in["node.data.start_age"]) {
              auto list = std::vector<float>();
              const auto data = skeleton_in["node.data.start_age"].as<YAML::Binary>();
              list.resize(data.size() / sizeof(float));
              std::memcpy(list.data(), data.data(), data.size());
              for (size_t i = 0; i < list.size(); i++) {
                auto& node = shoot_skeleton_.RefNode(i);
                node.data.start_age = list[i];
              }
            }

            if (skeleton_in["node.data.finish_age"]) {
              auto list = std::vector<float>();
              const auto data = skeleton_in["node.data.finish_age"].as<YAML::Binary>();
              list.resize(data.size() / sizeof(float));
              std::memcpy(list.data(), data.data(), data.size());
              for (size_t i = 0; i < list.size(); i++) {
                auto& node = shoot_skeleton_.RefNode(i);
                node.data.finish_age = list[i];
              }
            }

            if (skeleton_in["node.data.desired_local_rotation"]) {
              auto list = std::vector<glm::quat>();
              const auto data = skeleton_in["node.data.desired_local_rotation"].as<YAML::Binary>();
              list.resize(data.size() / sizeof(glm::quat));
              std::memcpy(list.data(), data.data(), data.size());
              for (size_t i = 0; i < list.size(); i++) {
                auto& node = shoot_skeleton_.RefNode(i);
                node.data.desired_local_rotation = list[i];
              }
            }

            if (skeleton_in["node.data.desired_global_rotation"]) {
              auto list = std::vector<glm::quat>();
              const auto data = skeleton_in["node.data.desired_global_rotation"].as<YAML::Binary>();
              list.resize(data.size() / sizeof(glm::quat));
              std::memcpy(list.data(), data.data(), data.size());
              for (size_t i = 0; i < list.size(); i++) {
                auto& node = shoot_skeleton_.RefNode(i);
                node.data.desired_global_rotation = list[i];
              }
            }

            if (skeleton_in["node.data.desired_global_position"]) {
              auto list = std::vector<glm::vec3>();
              const auto data = skeleton_in["node.data.desired_global_position"].as<YAML::Binary>();
              list.resize(data.size() / sizeof(glm::vec3));
              std::memcpy(list.data(), data.data(), data.size());
              for (size_t i = 0; i < list.size(); i++) {
                auto& node = shoot_skeleton_.RefNode(i);
                node.data.desired_global_position = list[i];
              }
            }

            if (skeleton_in["node.data.extra_mass"]) {
              auto list = std::vector<float>();
              const auto data = skeleton_in["node.data.extra_mass"].as<YAML::Binary>();
              list.resize(data.size() / sizeof(float));
              std::memcpy(list.data(), data.data(), data.size());
              for (size_t i = 0; i < list.size(); i++) {
                auto& node = shoot_skeleton_.RefNode(i);
                node.data.extra_mass = list[i];
              }
            }

            if (skeleton_in["node.data.density"]) {
              auto list = std::vector<float>();
              const auto data = skeleton_in["node.data.density"].as<YAML::Binary>();
              list.resize(data.size() / sizeof(float));
              std::memcpy(list.data(), data.data(), data.size());
              for (size_t i = 0; i < list.size(); i++) {
                auto& node = shoot_skeleton_.RefNode(i);
                node.data.density = list[i];
              }
            }
          });
      initialized_ = true;
    }
  }
}