#include "DynamicTreeSkeleton.hpp"
#include "GpuTreePhysics.hpp"

using namespace eco_sys_lab_package;

DynamicTreeSkeleton::GpuRestState DynamicTreeSkeleton::ExportGpuRestState() const {
  GpuRestState state;
  if (physics_rods_.empty())
    return state;
  state.initial_model = previous_model_;
  state.vertices.reserve(physics_vertices_.size());
  state.rods.reserve(physics_rods_.size());
  state.angles.reserve(physics_angles_.size());
  for (size_t i = 0; i < physics_vertices_.size(); ++i) {
    const auto& vertex = physics_vertices_[i];
    const auto& incident = physics_vertex_rods_[i];
    state.vertices.push_back(
        {glm::vec4(vertex.position, vertex.inverse_mass),
         glm::vec4(vertex.velocity, vertex.radius),
         glm::vec4(physics_rest_positions_[i], 0.0f),
         {static_cast<uint32_t>(state.vertex_rods.size()), static_cast<uint32_t>(incident.size()), 0u, 0u}});
    for (const int rod : incident)
      state.vertex_rods.push_back(static_cast<uint32_t>(rod));
  }
  for (size_t i = 0; i < physics_rods_.size(); ++i) {
    const auto& rod = physics_rods_[i];
    const auto& incident = physics_rod_angles_[i];
    state.rods.push_back(
        {{static_cast<uint32_t>(rod.start), static_cast<uint32_t>(rod.end), static_cast<uint32_t>(rod.node),
          rod.pinned ? 1u : 0u},
         {rod.length, rod.structural_length_scale, rod.mass, rod.radius},
         {rod.rotation.x, rod.rotation.y, rod.rotation.z, rod.rotation.w},
         {rod.rest_rotation.x, rod.rest_rotation.y, rod.rest_rotation.z, rod.rest_rotation.w},
         {static_cast<uint32_t>(state.rod_angles.size()), static_cast<uint32_t>(incident.size()), 0u, 0u}});
    for (const int angle : incident)
      state.rod_angles.push_back(static_cast<uint32_t>(angle));
  }
  for (const auto& angle : physics_angles_)
    state.angles.push_back(
        {{static_cast<uint32_t>(angle.first), static_cast<uint32_t>(angle.second), 0u, 0u},
         {angle.rest_relative.x, angle.rest_relative.y, angle.rest_relative.z, angle.rest_relative.w},
         {angle.bend_scale, 0.0f, 0.0f, 0.0f}});
  state.node_rods.reserve(physics_node_rods_.size());
  for (const int rod : physics_node_rods_)
    state.node_rods.push_back(rod < 0 ? UINT32_MAX : static_cast<uint32_t>(rod));
  return state;
}

void DynamicTreeSkeleton::RestoreGpuPoseAfterLoad(Tree& tree) {
  if (!restore_gpu_pose_on_load_)
    return;
  restore_gpu_pose_on_load_ = false;
  if (gpu_physics_enabled)
    InitializePhysics(tree);
}

bool DynamicTreeSkeleton::SyncGpuPhysics(Tree& tree, const GlobalTransform& model, const bool growing) {
  if (!gpu_physics_enabled)
    return false;
  if (HasPhysics(tree) && GpuScaleChanged(model))
    ResetPhysics(tree);
  if (!HasPhysics(tree) && !tree.shoot_model.PeekShootSkeleton().PeekSortedNodeList().empty())
    InitializePhysics(tree, model);
  physics_running = HasPhysics(tree) && !growing;
  return HasPhysics(tree);
}

void DynamicTreeSkeleton::ResetPhysics(Tree& tree) {
  physics_running = false;
  restore_gpu_pose_on_load_ = false;
  SetGpuPhysics(nullptr, 0);
  model_revision_ = 0;
  has_detached_branches_ = false;
  decltype(physics_vertices_){}.swap(physics_vertices_);
  decltype(physics_rods_){}.swap(physics_rods_);
  decltype(physics_angles_){}.swap(physics_angles_);
  decltype(physics_node_rods_){}.swap(physics_node_rods_);
  decltype(physics_predicted_){}.swap(physics_predicted_);
  decltype(physics_previous_){}.swap(physics_previous_);
  decltype(physics_vertex_rods_){}.swap(physics_vertex_rods_);
  decltype(physics_rod_angles_){}.swap(physics_rod_angles_);
  decltype(physics_rest_positions_){}.swap(physics_rest_positions_);
  tree.shoot_model.ResetPhysicsPose();
}
