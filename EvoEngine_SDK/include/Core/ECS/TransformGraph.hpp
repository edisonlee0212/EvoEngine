#pragma once
#include "Entities.hpp"
#include "EntityMetadata.hpp"
#include "ILayer.hpp"
#include "Transform.hpp"
namespace evo_engine {
class TransformGraph : public ISingleton<TransformGraph> {
  friend class PhysicsSystem;
  friend class Application;
  EntityQuery transform_query_;
  bool physics_system_override_ = false;
  static void CalculateTransformGraph(const std::shared_ptr<Scene>& scene,
                                      const std::vector<EntityMetadata>& entity_infos,
                                      const GlobalTransform& parent_global_transform, const Entity& parent);
  static void Initialize();

 public:
  static void CalculateTransformGraphForDescendants(const std::shared_ptr<Scene>& scene, const Entity& entity);
  static void CalculateTransformGraphs(const std::shared_ptr<Scene>& scene, bool check_static = true);
};
}  // namespace evo_engine