#include "DynamicTreeSkeleton.hpp"
#include "EcoSysLabEditorLayer.hpp"
#include "EcoSysLabLayer.hpp"
#include "EcoSysLabObjectInspectors.hpp"
#include "EditorLayer.hpp"
#include "Tree.hpp"

using namespace evo_engine;
using namespace eco_sys_lab_package;

namespace {
void UpdateSkeletonVisualization(const DynamicTreeSkeleton& target, const std::shared_ptr<ParticleInfoList>& matrices) {
  const auto& sorted_node_list = target.dynamic_skeleton.dts_skeleton.PeekSortedNodeList();
  if (!sorted_node_list.empty()) {
    std::vector<ParticleInfo> particle_infos(sorted_node_list.size());
    Jobs::RunParallelFor(sorted_node_list.size(), [&](const auto node_index) {
      const auto& node = target.dynamic_skeleton.dts_skeleton.PeekNode(sorted_node_list[node_index]);
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
    matrices->SetParticleInfos(particle_infos);
  }
}
}  // namespace
bool DynamicTreeSkeletonInspector::Inspect(InspectorContext& context, DynamicTreeSkeleton& target) {
  const auto& editor_layer = context.editor_layer;
  bool changed = false;
  if (ImGui::Checkbox("Physics", &target.simulate))
    changed = true;
  if (!target.simulate) {
    if (ImGui::Button("Step")) {
      if (const auto layer = ApplicationContext::Get().GetLayer<EcoSysLabLayer>())
        target.PhysicsStep(layer->dynamic_skeleton_settings_.physics_parameters);
    }
  }
  if (editor_layer->DragAndDropButton<Tree>(dynamic_tree_skeleton_tree_ref, "Download Skeleton from Tree...")) {
    if (const auto tree = dynamic_tree_skeleton_tree_ref.Get<Tree>()) {
      const auto scene = target.GetScene();
      const auto owner = target.GetOwner();
      target.initialize_parameters.root_transform = scene->GetDataComponent<GlobalTransform>(owner);
      target.dynamic_skeleton.Initialize(target.initialize_parameters, tree->shoot_model.PeekShootSkeleton());
      dynamic_tree_skeleton_tree_ref.Clear();
    }
  }
  const auto target_scene = target.GetScene();
  if (scene.lock() != target_scene) {
    previews.clear();
    scene = target_scene;
  }
  for (auto it = previews.begin(); it != previews.end();) {
    if (it->second.owner.expired())
      it = previews.erase(it);
    else
      ++it;
  }
  auto& preview = previews[&target];
  if (!preview.matrices) {
    preview.owner = target_scene->GetPrivateComponent(target.GetOwner(), "DynamicTreeSkeleton");
    preview.matrices = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
  }
  if (const auto layer = ApplicationContext::Get().GetLayer<EcoSysLabEditorLayer>();
      layer && layer->dynamic_skeleton_settings_.enable_visualization) {
    UpdateSkeletonVisualization(target, preview.matrices);
  }
  editor_layer->DrawGizmoCubes(preview.matrices);
  return changed;
}
