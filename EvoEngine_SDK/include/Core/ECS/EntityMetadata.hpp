#pragma once
#include "Entity.hpp"
#include "IPrivateComponent.hpp"
namespace evo_engine {
class Scene;

struct EntityMetadata {
  std::string entity_name;
  bool entity_static = false;
  bool ancestor_selected = false;
  unsigned entity_version = 1;
  bool entity_enabled = true;
  Entity parent = Entity();
  Entity root = Entity();
  std::vector<PrivateComponentElement> private_component_elements;
  std::vector<Entity> children;
  size_t data_component_storage_index = 0;
  size_t chunk_array_index = 0;
  Handle entity_handle;
  void Serialize(YAML::Emitter &out, const std::shared_ptr<Scene> &scene) const;
  void Deserialize(const YAML::Node &in, const std::shared_ptr<Scene> &scene);
  void Clone(const std::unordered_map<Handle, Handle> &entity_map, const EntityMetadata &source,
             const std::shared_ptr<Scene> &scene);
};

}  // namespace evo_engine