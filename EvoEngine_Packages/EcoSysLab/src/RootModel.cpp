//
// Created by lllll on 10/21/2022.
//

#include "RootModel.hpp"

using namespace eco_sys_lab_package;

void RootModel::CalculateThickness(const RootGrowthController& root_growth_controller) {
  auto& sorted_node_list = root_skeleton_.PeekSortedNodeList();
  for (auto it = sorted_node_list.rbegin(); it != sorted_node_list.rend(); ++it) {
    const auto node_handle = *it;
    auto& node = root_skeleton_.RefNode(node_handle);
    auto& node_info = node.info;
    auto& node_data = node.data;
    float child_thickness_collection = 0.0f;

    const float thickness_accumulation_factor = glm::clamp(
        root_growth_controller.thickness_accumulation_factor(random_engine_, root_skeleton_.data, node), 0.0f, 1.0f);

    for (const auto& i : node.PeekChildHandles()) {
      const auto& child_internode = root_skeleton_.PeekNode(i);
      child_thickness_collection += glm::pow(child_internode.data.node_thickness, 1.0f / thickness_accumulation_factor);
    }
    child_thickness_collection += root_growth_controller.thickness(random_engine_, root_skeleton_.data, node);
    if (child_thickness_collection != 0.0f) {
      node_data.node_thickness =
          glm::max(node_data.node_thickness, glm::pow(child_thickness_collection, thickness_accumulation_factor));
    } else {
      node_data.node_thickness = glm::max(node_data.node_thickness, 1.f);
    }
    node_info.thickness = node_data.node_thickness * root_growth_controller.base_thickness;
  }
  // Scale thickness to match shoot model.
  if (!sorted_node_list.empty()) {
    const float scale_factor = shoot_skeleton_base_thickness / root_skeleton_.RefNode(0).info.thickness;
    for (const auto& node_handle : sorted_node_list) {
      auto& node = root_skeleton_.RefNode(node_handle);
      auto& node_info = node.info;
      auto& node_data = node.data;
      node_info.thickness *= scale_factor;
      node_data.node_thickness *= scale_factor;
    }
  }
}

void RootModel::CalculateGrowthData(const RootGrowthController& root_growth_controller) {
  {
    CalculateThickness(root_growth_controller);
    root_skeleton_.CalculateDistanceVolumeLevel();
    CalculateTransform(root_growth_controller);
  }
  root_skeleton_.CalculateFlows();
}

void RootModel::Initialize(const RootGrowthController& root_growth_controller) {
  if (initialized_)
    Clear();
  random_engine_ = std::mt19937(static_cast<uint32_t>(seed));
  {
    root_skeleton_ = RootSkeleton(root_growth_controller.base_root_node_count);
    root_skeleton_.SortLists();
    for (const auto& node_handle : root_skeleton_.PeekSortedNodeList()) {
      auto& node = root_skeleton_.RefNode(node_handle);
      node.data.node_thickness = 1.f;
      node.info.thickness = root_growth_controller.base_thickness;
      node.data.node_length = 0.0f;
      root_growth_controller.base_node_initialization(random_engine_, root_skeleton_.data, node);
    }
  }
  initialized_ = true;
}

Vigor RootModel::SampleRootFlux(const glm::mat4& global_transform, const VoxelSoilModel& soil_model,
                                const RootGrowthController& root_growth_controller) {
  root_skeleton_.SortLists();
  Vigor total_shoot_flux;
  total_shoot_flux.value = 0.0f;
  const auto& sorted_root_node_list = root_skeleton_.PeekSortedNodeList();
  for (const auto& node_handle : sorted_root_node_list) {
    auto& root_node = root_skeleton_.RefNode(node_handle);
    auto& root_node_info = root_node.info;
    const glm::vec3 position = global_transform * glm::vec4(root_node_info.global_position, 1.0f);
    // root_node.data.water = soil_model.IntegrateWater(position, 0.2f);
    // root_node.data.nutrient = soil_model.IntegrateNutrient(position, 0.2f);
    // root_node.data.soil_density = soil_model.GetDensity(position);
    root_node.data.water = 1.f;
    root_node.data.nutrient = 1.f;
    if (position.y < 0.f) {
      root_node.data.soil_density = 1.2f - position.y * 0.5f;
    } else {
      root_node.data.soil_density = 0.0f;
    }
    total_shoot_flux.value += root_node.data.water;
  }
  CalculateGrowthData(root_growth_controller);
  return total_shoot_flux;
}

void RootModel::CalculateTransform(const RootGrowthController& root_growth_controller) {
  root_skeleton_.min = glm::vec3(FLT_MAX);
  root_skeleton_.max = glm::vec3(-FLT_MAX);
  root_skeleton_.data.desired_min = glm::vec3(FLT_MAX);
  root_skeleton_.data.desired_max = glm::vec3(-FLT_MAX);
  const auto& sorted_node_list = root_skeleton_.PeekSortedNodeList();
  for (const auto& node_handle : sorted_node_list) {
    auto& node = root_skeleton_.RefNode(node_handle);
    auto& node_data = node.data;
    auto& node_info = node.info;

    node_info.length = root_growth_controller.root_node_length(random_engine_, root_skeleton_.data, node);

    if (node.GetParentHandle() != -1) {
      auto& parent_internode = root_skeleton_.RefNode(node.GetParentHandle());
      auto parent_global_rotation = parent_internode.info.global_rotation;
      node_info.global_rotation = parent_global_rotation * node_data.desired_local_rotation;
      auto front = glm::normalize(node_info.global_rotation * glm::vec3(0, 0, -1));
      auto up = glm::normalize(node_info.global_rotation * glm::vec3(0, 1, 0));

      auto parent_regulated_up = parent_internode.info.regulated_global_rotation * glm::vec3(0, 1, 0);
      auto regulated_up = glm::normalize(glm::cross(glm::cross(front, parent_regulated_up), front));
      node_info.regulated_global_rotation = glm::quatLookAt(front, regulated_up);

      node_info.GetGlobalDirection() = glm::normalize(node_info.global_rotation * glm::vec3(0, 0, -1));
      node_info.global_position = parent_internode.info.global_position +
                                  parent_internode.info.length * parent_internode.info.GetGlobalDirection();

      node_data.desired_global_rotation =
          parent_internode.data.desired_global_rotation * node_data.desired_local_rotation;
      auto parent_desired_front = parent_internode.data.desired_global_rotation * glm::vec3(0, 0, -1);
      node_data.desired_global_position =
          parent_internode.data.desired_global_position + parent_internode.info.length * parent_desired_front;
    }

    root_skeleton_.min = glm::min(root_skeleton_.min, node_info.global_position);
    root_skeleton_.max = glm::max(root_skeleton_.max, node_info.global_position);
    const auto end_position = node_info.global_position + node_info.length * node_info.GetGlobalDirection();
    root_skeleton_.min = glm::min(root_skeleton_.min, end_position);
    root_skeleton_.max = glm::max(root_skeleton_.max, end_position);

    root_skeleton_.data.desired_min = glm::min(root_skeleton_.data.desired_min, node_data.desired_global_position);
    root_skeleton_.data.desired_max = glm::max(root_skeleton_.data.desired_max, node_data.desired_global_position);
    const auto desired_global_direction = node_data.desired_global_rotation * glm::vec3(0, 0, -1);
    const auto desired_end_position = node_data.desired_global_position + node_info.length * desired_global_direction;
    root_skeleton_.data.desired_min = glm::min(root_skeleton_.data.desired_min, desired_end_position);
    root_skeleton_.data.desired_max = glm::max(root_skeleton_.data.desired_max, desired_end_position);
  }
}

void RootModel::DistributeVigor(const RootGrowthController& root_growth_controller, const Vigor vigor) {
  const auto& sorted_node_list = root_skeleton_.PeekSortedNodeList();
  float max_grow_potential = 0.0f;
  for (const auto& node_handle : sorted_node_list) {
    auto& node = root_skeleton_.RefNode(node_handle);
    node.data.growth_potential = root_growth_controller.growth_potential(random_engine_, root_skeleton_, node);
    max_grow_potential = glm::max(max_grow_potential, node.data.growth_potential);
  }
  float total_desired_growth_rate = 1.0f;
  for (const auto& node_handle : sorted_node_list) {
    auto& node = root_skeleton_.RefNode(node_handle);
    if (max_grow_potential > 0.0f)
      node.data.growth_potential /= max_grow_potential;
    node.data.desired_growth_rate = node.data.nutrient * node.data.growth_potential;
    total_desired_growth_rate += node.data.desired_growth_rate;
  }
  const float clamped_factor = vigor.value / total_desired_growth_rate;
  for (const auto& node_handle : sorted_node_list) {
    auto& node = root_skeleton_.RefNode(node_handle);
    // You cannot give more than enough resources.
    node.data.growth_rate = clamped_factor * node.data.desired_growth_rate;
  }
}

RootSkeleton& RootModel::RefRootSkeleton() {
  return root_skeleton_;
}

const RootSkeleton& RootModel::PeekRootSkeleton(int iteration) const {
  assert(iteration < 0 || iteration <= root_history_.size());
  if (iteration == root_history_.size() || iteration < 0)
    return root_skeleton_;
  return root_history_.at(iteration);
}

void RootModel::Clear() {
  root_skeleton_ = {};
  root_history_ = {};
  initialized_ = false;
  iteration_ = 0;
}

void RootModel::ClearHistory() {
  root_history_.clear();
}

void RootModel::Step() {
  root_history_.emplace_back(root_skeleton_);
  if (history_limit > 0) {
    while (root_history_.size() > history_limit) {
      root_history_.pop_front();
    }
  }
}

void RootModel::Pop() {
  root_history_.pop_back();
}

int RootModel::CurrentIteration() const {
  return root_history_.size();
}

void RootModel::Reverse(const int iteration) {
  assert(iteration >= 0 && iteration < root_history_.size());
  root_skeleton_ = root_history_[iteration];
  root_history_.erase((root_history_.begin() + iteration), root_history_.end());
}

bool RootModel::Grow(float delta_time, const glm::mat4& global_transform, const ClimateModel& climate_model,
                     const VoxelSoilModel& soil_model, const RootGrowthController& root_growth_controller,
                     const FineRootController& fine_root_controller,
                     const RootReproductionController& reproduction_controller,
                     const RootPruningController& root_pruning_controller, bool pruning) {
  if (!initialized_) {
    EVOENGINE_ERROR("ShootModel not initialized!")
    return false;
  }
  current_delta_time_ = delta_time;
  root_skeleton_.data.age += current_delta_time_;
  bool structure_changed = false;
  {
    const auto& sorted_node_list = root_skeleton_.PeekSortedNodeList();
    for (auto it = sorted_node_list.rbegin(); it != sorted_node_list.rend(); ++it) {
      const bool graph_changed =
          GrowRootNode(*it, root_growth_controller, fine_root_controller, reproduction_controller);
      structure_changed = structure_changed || graph_changed;
    }
    if (structure_changed) {
      root_skeleton_.SortLists();
    }
  }
  if (pruning && root_pruning_controller.Initialized()) {
    CalculateGrowthData(root_growth_controller);
    if (PruneRootNodes(global_transform, climate_model, soil_model, root_growth_controller, root_pruning_controller)) {
      root_skeleton_.SortLists();
      structure_changed = true;
    }
  }
  CalculateGrowthData(root_growth_controller);

  iteration_++;
  return structure_changed;
}

bool RootModel::GrowRootNode(SkeletonNodeHandle node_handle, const RootGrowthController& root_growth_controller,
                             const FineRootController& fine_root_controller,
                             const RootReproductionController& reproduction_controller) {
  bool graph_changed = false;
  auto& node = root_skeleton_.RefNode(node_handle);
  auto& node_data = node.data;
  node_data.inhibitor_sink = 0;
  for (const auto& child_handle : node.PeekChildHandles()) {
    auto& child_node = root_skeleton_.RefNode(child_handle);
    float child_node_inhibitor = 0.f;
    if (child_node.IsEndNode()) {
      child_node_inhibitor = root_growth_controller.growth_inhibitor(random_engine_, root_skeleton_.data, child_node);
    }

    node_data.inhibitor_sink += glm::max(
        0.0f, root_growth_controller.growth_inhibitor_transport(
                  random_engine_, root_skeleton_.data, child_node_inhibitor + child_node.data.inhibitor_sink, node));
  }

  if (node.PeekChildHandles().empty()) {
    const float elongate_length = node_data.growth_rate * current_delta_time_ / 365.f *
                                  root_growth_controller.base_root_node_length *
                                  root_growth_controller.root_node_growth_rate;

    float collected_inhibitor = 0.0f;
    graph_changed = ElongateRootNode(elongate_length, node_handle, root_growth_controller, fine_root_controller,
                                     reproduction_controller, collected_inhibitor) ||
                    graph_changed;
    auto& current_internode = root_skeleton_.RefNode(node_handle);

    current_internode.data.inhibitor_sink +=
        glm::max(0.0f, root_growth_controller.growth_inhibitor_transport(random_engine_, root_skeleton_.data,
                                                                         collected_inhibitor, current_internode));
  } else {
    const float flush_probability =
        root_growth_controller.lateral_node_flushing_rate(random_engine_, root_skeleton_.data, node);
    if (flush_probability >= Random::Uniform(random_engine_, 0.f, 1.f)) {
      graph_changed = true;
      // Create new root node
      const auto new_internode_handle = root_skeleton_.Extend(node_handle, true);
      const auto& old_internode = root_skeleton_.PeekNode(node_handle);
      auto& new_internode = root_skeleton_.RefNode(new_internode_handle);
      // Prepare information for new internode

      new_internode.data = old_internode.data;
      new_internode.data.start_age = root_skeleton_.data.age;
      new_internode.data.finish_age = 0.0f;
      new_internode.info.order = old_internode.info.order + 1;
      new_internode.data.node_length = 0.0f;
      new_internode.info.root_distance = old_internode.info.root_distance;

      auto desired_global_rotation =
          old_internode.info.global_rotation *
          root_growth_controller.node_rotation(random_engine_, root_skeleton_.data, old_internode, new_internode);
      root_growth_controller.tropism(random_engine_, root_skeleton_.data, old_internode, new_internode,
                                     desired_global_rotation);
      new_internode.data.desired_local_rotation =
          glm::inverse(old_internode.info.global_rotation) * desired_global_rotation;
      new_internode.data.node_thickness = 1.f;
      new_internode.info.thickness = root_growth_controller.base_thickness;
    }
  }

  return graph_changed;
}

bool RootModel::ElongateRootNode(float extended_length, SkeletonNodeHandle internode_handle,
                                 const RootGrowthController& root_growth_controller,
                                 const FineRootController& fine_root_controller,
                                 const RootReproductionController& reproduction_controller,
                                 float& collected_inhibitor) {
  bool graph_changed = false;
  auto& internode = root_skeleton_.RefNode(internode_handle);
  const auto internode_length = root_growth_controller.base_root_node_length;
  auto& internode_data = internode.data;
  internode_data.node_length += extended_length;
  const float extra_length = internode_data.node_length - internode_length;
  // If we need to add a new end node
  if (extra_length >= 0) {
    graph_changed = true;
    internode_data.node_length = internode_length;

    // Create new internode
    const auto new_internode_handle = root_skeleton_.Extend(internode_handle, false);
    auto& old_internode = root_skeleton_.RefNode(internode_handle);
    auto& new_internode = root_skeleton_.RefNode(new_internode_handle);

    new_internode.data = old_internode.data;
    new_internode.data.water = old_internode.data.water;
    new_internode.data.nutrient = old_internode.data.nutrient;
    old_internode.data.finish_age = new_internode.data.start_age = root_skeleton_.data.age;
    new_internode.data.finish_age = 0.0f;
    new_internode.info.order = old_internode.info.order;
    new_internode.data.inhibitor_sink = 0.0f;
    new_internode.data.node_length = glm::clamp(extended_length, 0.0f, internode_length);
    new_internode.info.root_distance = old_internode.info.root_distance + new_internode.data.node_length;

    auto desired_global_rotation =
        old_internode.info.global_rotation *
        root_growth_controller.node_rotation(random_engine_, root_skeleton_.data, old_internode, new_internode);
    if (internode_handle != 0) {
      root_growth_controller.tropism(random_engine_, root_skeleton_.data, old_internode, new_internode,
                                     desired_global_rotation);
    }
    new_internode.info.global_rotation = desired_global_rotation;
    new_internode.data.desired_local_rotation =
        glm::inverse(old_internode.info.global_rotation) * new_internode.info.global_rotation;

    new_internode.data.node_thickness = 1.f;
    new_internode.info.thickness = root_growth_controller.base_thickness;

    if (extra_length > internode_length) {
      float child_inhibitor = 0.0f;
      ElongateRootNode(extra_length - internode_length, new_internode_handle, root_growth_controller,
                       fine_root_controller, reproduction_controller, child_inhibitor);
      auto& current_new_internode = root_skeleton_.RefNode(new_internode_handle);
      current_new_internode.data.inhibitor_sink +=
          glm::max(0.0f, root_growth_controller.growth_inhibitor_transport(random_engine_, root_skeleton_.data,
                                                                           child_inhibitor, current_new_internode));
      collected_inhibitor +=
          current_new_internode.data.inhibitor_sink +
          root_growth_controller.growth_inhibitor(random_engine_, root_skeleton_.data, current_new_internode);
    } else {
      collected_inhibitor +=
          root_growth_controller.growth_inhibitor(random_engine_, root_skeleton_.data, new_internode);
    }
  }
  return graph_changed;
}

bool RootModel::PruneRootNodes(const glm::mat4& global_transform, const ClimateModel& climate_model,
                               const VoxelSoilModel& soil_model, const RootGrowthController& root_growth_controller,
                               const RootPruningController& root_pruning_controller) {
  bool root_to_end_pruned = false;
  {
    const auto& sorted_internode_list = root_skeleton_.PeekSortedNodeList();
    std::vector<SkeletonNodeHandle> pruning_node_handles{};
    for (const auto& internode_handle : sorted_internode_list) {
      auto& internode = root_skeleton_.RefNode(internode_handle);
      if (internode_handle == 0)
        continue;
      if (internode.info.locked)
        continue;
      // Pruning here.
      bool pruning = false;
      if (const float pruning_probability =
              root_pruning_controller.base_to_end_pruning_factor(random_engine_, global_transform, climate_model,
                                                                 soil_model, root_skeleton_, internode) *
              current_delta_time_ / 365.f;
          !pruning && pruning_probability > Random::Uniform(random_engine_, 0.f, 1.f))
        pruning = true;
      if (pruning) {
        pruning_node_handles.emplace_back(internode_handle);
        root_to_end_pruned = true;
      }
    }
    root_skeleton_.RemoveNodes(pruning_node_handles);
  }

  bool end_to_root_pruned = false;
  {
    std::vector<SkeletonNodeHandle> pruning_node_handles{};
    const auto& sorted_internode_list = root_skeleton_.PeekSortedNodeList();
    for (auto it = sorted_internode_list.rbegin(); it != sorted_internode_list.rend(); ++it) {
      const auto internode_handle = *it;
      auto& internode = root_skeleton_.RefNode(internode_handle);
      if (internode_handle == 0)
        continue;
      if (internode.info.locked)
        continue;
      // Pruning here.
      bool pruning = false;
      if (const float pruning_probability =
              root_pruning_controller.end_to_base_pruning_factor(random_engine_, global_transform, climate_model,
                                                                 soil_model, root_skeleton_, internode) *
              current_delta_time_ / 365.f;
          !pruning && pruning_probability > Random::Uniform(random_engine_, 0.f, 1.f))
        pruning = true;
      if (pruning) {
        pruning_node_handles.emplace_back(internode_handle);
        end_to_root_pruned = true;
      }
    }

    root_skeleton_.RemoveNodes(pruning_node_handles);
  }
  root_skeleton_.CalculateDistanceVolumeLevel();
  return root_to_end_pruned || end_to_root_pruned;
}