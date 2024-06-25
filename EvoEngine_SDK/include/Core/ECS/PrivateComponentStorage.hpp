#pragma once
#include <utility>
#include "Entity.hpp"
#include "Serialization.hpp"
namespace evo_engine {
struct POwnersCollection {
  std::unordered_map<Entity, size_t, Entity> owners_map;
  std::vector<Entity> owners_list;
  POwnersCollection() {
    owners_list = std::vector<Entity>();
    owners_map = std::unordered_map<Entity, size_t, Entity>();
  }
};
class Scene;
class PrivateComponentStorage {
  std::unordered_map<size_t, size_t> p_owners_collections_map_;
  std::vector<std::pair<size_t, POwnersCollection>> p_owners_collections_list_;
  std::unordered_map<size_t, std::vector<std::shared_ptr<IPrivateComponent>>> private_component_pool_;

 public:
  std::weak_ptr<Scene> owner_scene;
  void RemovePrivateComponent(const Entity &entity, size_t type_index,
                              const std::shared_ptr<IPrivateComponent> &private_component);
  void DeleteEntity(const Entity &entity);
  template <typename T = IPrivateComponent>
  std::shared_ptr<T> GetOrSetPrivateComponent(const Entity &entity);
  void SetPrivateComponent(const Entity &entity, size_t id);
  template <typename T = IPrivateComponent>
  void RemovePrivateComponent(const Entity &entity, const std::shared_ptr<IPrivateComponent> &private_component);
  template <typename T>
  const std::vector<Entity> *UnsafeGetOwnersList();
  template <typename T>
  std::vector<Entity> GetOwnersList();
};

template <typename T>
std::shared_ptr<T> PrivateComponentStorage::GetOrSetPrivateComponent(const Entity &entity) {
  size_t id = typeid(T).hash_code();
  if (const auto search = p_owners_collections_map_.find(id); search != p_owners_collections_map_.end()) {
    if (const auto search2 = p_owners_collections_list_[search->second].second.owners_map.find(entity);
        search2 == p_owners_collections_list_[search->second].second.owners_map.end()) {
      p_owners_collections_list_[search->second].second.owners_map.insert(
          {entity, p_owners_collections_list_[search->second].second.owners_list.size()});
      p_owners_collections_list_[search->second].second.owners_list.push_back(entity);
    }
  } else {
    POwnersCollection collection;
    collection.owners_map.insert({entity, 0});
    collection.owners_list.push_back(entity);
    p_owners_collections_map_.insert({id, p_owners_collections_list_.size()});
    p_owners_collections_list_.emplace_back(id, std::move(collection));
  }
  if (const auto p_search = private_component_pool_.find(id);
      p_search != private_component_pool_.end() && !p_search->second.empty()) {
    const auto back = p_search->second.back();
    p_search->second.pop_back();
    back->handle_ = Handle();
    return std::dynamic_pointer_cast<T>(back);
  }
  return Serialization::ProduceSerializable<T>();
}

template <typename T>
void PrivateComponentStorage::RemovePrivateComponent(const Entity &entity,
                                                     const std::shared_ptr<IPrivateComponent> &private_component) {
  RemovePrivateComponent(entity, typeid(T).hash_code(), private_component);
}

template <typename T>
const std::vector<Entity> *PrivateComponentStorage::UnsafeGetOwnersList() {
  if (const auto search = p_owners_collections_map_.find(typeid(T).hash_code());
      search != p_owners_collections_map_.end()) {
    return &p_owners_collections_list_[search->second].second.owners_list;
  }
  return nullptr;
}
}  // namespace evo_engine
