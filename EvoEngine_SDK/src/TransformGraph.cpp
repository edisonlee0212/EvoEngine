// #include "ProfilerLayer.hpp"
// #include "RigidBody.hpp"
#include "TransformGraph.hpp"
#include "Application.hpp"
#include "ClassRegistry.hpp"
#include "Jobs.hpp"
#include "Scene.hpp"
using namespace evo_engine;

DataComponentRegistration<Transform> transform_registry("Transform");
DataComponentRegistration<GlobalTransform> global_transform_registry("GlobalTransform");
DataComponentRegistration<TransformUpdateFlag> transform_update_status_registry("TransformUpdateFlag");

void TransformGraph::Initialize() {
  auto& transform_graph = GetInstance();
  transform_graph.transform_query_ = Entities::CreateEntityQuery();
  Entities::SetEntityQueryAllFilters(transform_graph.transform_query_, Transform(), GlobalTransform());
}

void TransformGraph::CalculateTransformGraph(const std::shared_ptr<Scene>& scene,
                                             const std::vector<EntityMetadata>& entity_infos,
                                             const GlobalTransform& parent_global_transform, const Entity& parent) {
  const auto& entity_info = entity_infos.at(parent.GetIndex());
  for (const auto& entity : entity_info.children) {
    auto* transform_status = reinterpret_cast<TransformUpdateFlag*>(
        scene->GetDataComponentPointer(entity.GetIndex(), typeid(TransformUpdateFlag).hash_code()));
    GlobalTransform ltw;
    if (transform_status->global_transform_modified) {
      ltw = scene->GetDataComponent<GlobalTransform>(entity.GetIndex());
      reinterpret_cast<Transform*>(scene->GetDataComponentPointer(entity.GetIndex(), typeid(Transform).hash_code()))
          ->value = glm::inverse(parent_global_transform.value) * ltw.value;
      transform_status->global_transform_modified = false;
    } else {
      auto ltp = scene->GetDataComponent<Transform>(entity.GetIndex());
      ltw.value = parent_global_transform.value * ltp.value;
      *reinterpret_cast<GlobalTransform*>(
          scene->GetDataComponentPointer(entity.GetIndex(), typeid(GlobalTransform).hash_code())) = ltw;
    }
    transform_status->transform_modified = false;
    CalculateTransformGraph(scene, entity_infos, ltw, entity);
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
      [&](int i, Entity entity, Transform& transform, GlobalTransform& global_transform,
          TransformUpdateFlag& transform_status) {
        const EntityMetadata& entity_info = scene->scene_data_storage_.entity_metadata_list.at(entity.GetIndex());
        if (entity_info.parent.GetIndex() != 0) {
          transform_status.transform_modified = false;
          return;
        }
        if (check_static && entity_info.entity_static) {
          transform_status.transform_modified = false;
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
  const auto& entity_infos = scene->scene_data_storage_.entity_metadata_list;
  transform_graph.CalculateTransformGraph(scene, entity_infos,
                                         scene->GetDataComponent<GlobalTransform>(entity.GetIndex()), entity);
}
