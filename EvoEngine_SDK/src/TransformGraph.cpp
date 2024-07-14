// #include "ProfilerLayer.hpp"
// #include "RigidBody.hpp"
#include "TransformGraph.hpp"
#include "Application.hpp"
#include "ClassRegistry.hpp"
#include "Jobs.hpp"
#include "Scene.hpp"
using namespace evo_engine;

void TransformGraph::Initialize() {
  auto& transform_graph = GetInstance();
  transform_graph.transform_query_ = Entities::CreateEntityQuery();
  Entities::SetEntityQueryAllFilters(transform_graph.transform_query_, Transform(), GlobalTransform());
}

void TransformGraph::CalculateTransformGraph(const std::shared_ptr<Scene>& scene,
                                             const std::vector<EntityMetadata>& entity_infos,
                                             const GlobalTransform& parent_global_transform, const Entity& parent) {
  const auto& entity_info = entity_infos.at(parent.GetIndex());
  for (const auto& child : entity_info.children) {
    auto* transform_status = static_cast<TransformUpdateFlag*>(
        scene->GetDataComponentPointer(child.GetIndex(), typeid(TransformUpdateFlag).hash_code()));
    transform_status->transform_modified = false;
    GlobalTransform ltw;
    if (transform_status->global_transform_modified) {
      ltw = scene->GetDataComponent<GlobalTransform>(child.GetIndex());
      static_cast<Transform*>(scene->GetDataComponentPointer(child.GetIndex(), typeid(Transform).hash_code()))
          ->value = glm::inverse(parent_global_transform.value) * ltw.value;
      transform_status->global_transform_modified = false;
    } else {
      auto ltp = scene->GetDataComponent<Transform>(child.GetIndex());
      ltw.value = parent_global_transform.value * ltp.value;
      *static_cast<GlobalTransform*>(
          scene->GetDataComponentPointer(child.GetIndex(), typeid(GlobalTransform).hash_code())) = ltw;
    }
    CalculateTransformGraph(scene, entity_infos, ltw, child);
  }
}
void TransformGraph::CalculateTransformGraphs(const std::shared_ptr<Scene>& scene, const bool check_static) {
  if (!scene)
    return;
  auto& transform_graph = GetInstance();
  const auto& entity_infos = scene->scene_data_storage_.entity_metadata_list;
  // ProfilerLayer::StartEvent("TransformManager");
  Jobs::Wait(scene->ForEach<Transform, GlobalTransform, TransformUpdateFlag>(
      {}, transform_graph.transform_query_,
      [&](int i, const Entity entity, Transform& transform, GlobalTransform& global_transform,
          TransformUpdateFlag& transform_status) {
        const EntityMetadata& entity_info = scene->scene_data_storage_.entity_metadata_list.at(entity.GetIndex());
        transform_status.transform_modified = false;
        if (entity_info.parent.GetIndex() != 0) {
          return;
        }
        if (check_static && entity_info.entity_static) {
          return;
        }
        if (transform_status.global_transform_modified) {
          transform.value = global_transform.value;
          transform_status.global_transform_modified = false;
        } else {
          global_transform.value = transform.value;
        }
        transform_graph.CalculateTransformGraph(scene, entity_infos, global_transform, entity);
      },
      false));

  transform_graph.physics_system_override_ = false;
  // ProfilerLayer::EndEvent("TransformManager");
}
void TransformGraph::CalculateTransformGraphForDescendants(const std::shared_ptr<Scene>& scene, const Entity& entity) {
  if (!scene)
    return;
  auto& transform_graph = GetInstance();
  const auto& entity_index = entity.GetIndex();
  const auto& entity_infos = scene->scene_data_storage_.entity_metadata_list;
  const EntityMetadata& entity_info = scene->scene_data_storage_.entity_metadata_list.at(entity_index);
  auto* transform_status = static_cast<TransformUpdateFlag*>(
      scene->GetDataComponentPointer(entity_index, typeid(TransformUpdateFlag).hash_code()));
  auto* transform = static_cast<Transform*>(scene->GetDataComponentPointer(entity_index, typeid(Transform).hash_code()));
  auto* global_transform = static_cast<GlobalTransform*>(scene->GetDataComponentPointer(entity_index, typeid(GlobalTransform).hash_code()));

  if (entity_info.parent.GetIndex() != 0) {
    //Not root
    const auto parent_global_transform = scene->GetDataComponent<GlobalTransform>(entity_info.parent);
    GlobalTransform ltw;
    if (transform_status->global_transform_modified) {
      ltw = scene->GetDataComponent<GlobalTransform>(entity_index);
      transform->value = glm::inverse(parent_global_transform.value) * ltw.value;
      transform_status->global_transform_modified = false;
    } else {
      const auto ltp = scene->GetDataComponent<Transform>(entity_index);
      ltw.value = parent_global_transform.value * ltp.value;
      global_transform->value = ltw.value;
    }
  } else {
    //Root
    if (transform_status->global_transform_modified) {
      transform->value = global_transform->value;
      transform_status->global_transform_modified = false;
    } else {
      global_transform->value = transform->value;
    }
  }
  transform_graph.CalculateTransformGraph(scene, entity_infos,
                                          scene->GetDataComponent<GlobalTransform>(entity_index), entity);
}
