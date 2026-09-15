#include "DynamicTreeSkeleton.hpp"

using namespace eco_sys_lab_package;

void DynamicTreeSkeleton::PhysicsStep(const DynamicSkeleton::PhysicsParameters& physics_parameters) {
  if (const auto& sorted_node_list = dynamic_skeleton.dts_skeleton.PeekSortedNodeList(); !sorted_node_list.empty()) {
    const auto scene = GetScene();
    const auto owner = GetOwner();
    const auto global_transform = scene->GetDataComponent<GlobalTransform>(owner);
    auto& root_node = dynamic_skeleton.dts_skeleton.RefNode(sorted_node_list.front());
    root_node.data.particle0.x = root_node.data.particle0.last_x = global_transform.GetPosition();
    root_node.data.particle1.x = root_node.data.particle1.last_x = global_transform.TransformPoint(
        root_node.data.particle1.x0 - initialize_parameters.root_transform.GetPosition());
    root_node.data.q = root_node.data.last_q = global_transform.GetRotation() *
                                               glm::inverse(initialize_parameters.root_transform.GetRotation()) *
                                               root_node.data.q0;
    root_node.data.inv_mass = 0.f;
    dynamic_skeleton.Physics(
        physics_parameters,
        []() {
        },
        []() {
        });
  }
}
