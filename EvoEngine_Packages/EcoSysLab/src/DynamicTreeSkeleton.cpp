#include "DynamicTreeSkeleton.hpp"

using namespace eco_sys_lab_package;

void DynamicTreeSkeleton::ResetPhysics(Tree& tree) {
  physics_running = false;
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
