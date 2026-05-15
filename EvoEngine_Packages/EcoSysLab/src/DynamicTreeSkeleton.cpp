#include "DynamicTreeSkeleton.hpp"

#include "EcoSysLabLayer.hpp"
#include "Tree.hpp"

using namespace eco_sys_lab_package;
PrivateComponentRef dynamic_tree_skeleton_tree_ref{};

bool DynamicTreeSkeleton::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::Checkbox("Physics", &simulate))
    changed = true;
  if (!simulate) {
    if (ImGui::Button("Step")) {
      PhysicsStep(ApplicationContext::Get().GetLayer<EcoSysLabLayer>()->dynamic_skeleton_settings_.physics_parameters);
    }
  }
  if (editor_layer->DragAndDropButton<Tree>(dynamic_tree_skeleton_tree_ref, "Download Skeleton from Tree...")) {
    if (const auto tree = dynamic_tree_skeleton_tree_ref.Get<Tree>()) {
      const auto scene = GetScene();
      const auto owner = GetOwner();
      initialize_parameters.root_transform = scene->GetDataComponent<GlobalTransform>(owner);
      dynamic_skeleton.Initialize(initialize_parameters, tree->shoot_model.PeekShootSkeleton());
      dynamic_tree_skeleton_tree_ref.Clear();
    }
  }
  editor_layer->DrawGizmoCubes(debug_matrices);
  return changed;
}

void DynamicTreeSkeleton::LateUpdate() {
}

void DynamicTreeSkeleton::OnCreate() {
  debug_matrices = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
}

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

void DynamicTreeSkeleton::Visualization(
    const std::shared_ptr<Camera>& target_camera,
    const DynamicSkeleton::VisualizationParameters& visualization_parameters) const {
  const auto& sorted_node_list = dynamic_skeleton.dts_skeleton.PeekSortedNodeList();
  if (!sorted_node_list.empty()) {
    std::vector<ParticleInfo> particle_infos(sorted_node_list.size());
    Jobs::RunParallelFor(sorted_node_list.size(), [&](const auto node_index) {
      const auto& node = dynamic_skeleton.dts_skeleton.PeekNode(sorted_node_list[node_index]);
      const auto& node_data = node.data;
      auto& particle_info = particle_infos[node_index];
      particle_info.instance_color = node.info.color;
      const auto& start = node_data.particle0.x;
      const auto& end = node_data.particle1.x;
      const auto direction = glm::normalize(end - start);
      const auto rotation = glm::quatLookAt(direction, glm::vec3(direction.y, direction.z, direction.x));
      const auto rotation_mat = glm::mat4_cast(rotation);
      const auto model = glm::translate((start + end) / 2.0f) * rotation_mat *
                         glm::scale(glm::vec3(node.info.thickness, node.info.thickness, glm::distance(end, start)));
      particle_info.instance_matrix.value = model;
    });
    debug_matrices->SetParticleInfos(particle_infos);
  }
}