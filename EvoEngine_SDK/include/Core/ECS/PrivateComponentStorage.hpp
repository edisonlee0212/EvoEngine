
#pragma once
#include <utility>
#include "Entity.hpp"
#include "Serialization.hpp"

namespace evo_engine {

/**
 * @struct POwnersCollection
 * @brief Represents a collection of owners associated with private components.
 */
struct POwnersCollection {
  /**
   * @brief Maps an entity to its index in the owners list.
   */
  std::unordered_map<Entity, size_t, Entity> owners_map;

  /**
   * @brief Stores a list of entities that own private components.
   */
  std::vector<Entity> owners_list;

  /**
   * @brief Default constructor to initialize the owners collection.
   */
  POwnersCollection() {
    owners_list = std::vector<Entity>();
    owners_map = std::unordered_map<Entity, size_t, Entity>();
  }
};

class EVOENGINE_API Scene;

/**
 * @class PrivateComponentStorage
 * @brief Manages the storage and lifecycle of private components associated with entities.
 */
class EVOENGINE_API PrivateComponentStorage {
  /**
   * @brief Maps a type ID to its index in the owners collections list.
   */
  std::unordered_map<size_t, size_t> p_owners_collections_map_;

  /**
   * @brief Stores a list of type ID and associated owners collections.
   */
  std::vector<std::pair<size_t, POwnersCollection>> p_owners_collections_list_;

  /**
   * @brief Maps type IDs to a pool of private component instances.
   */
  std::unordered_map<size_t, std::vector<std::shared_ptr<IPrivateComponent>>> private_component_pool_;

 public:
  /**
   * @brief The scene that owns this private component storage.
   */
  std::weak_ptr<Scene> owner_scene;

  /**
   * @brief Removes a private component from an entity.
   * @param entity The entity from which to remove the private component.
   * @param type_index The type index of the private component.
   * @param private_component The private component to remove.
   */
  void RemovePrivateComponent(const Entity &entity, size_t type_index,
                              const std::shared_ptr<IPrivateComponent> &private_component);

  /**
   * @brief Deletes an entity and its associated private components.
   * @param entity The entity to delete.
   */
  void DeleteEntity(const Entity &entity);

  /**
   * @brief Gets or sets a private component of type T for an entity.
   * @tparam T The type of the private component.
   * @param entity The entity for which to get or set the private component.
   * @return A shared pointer to the private component of type T.
   */
  template <typename T = IPrivateComponent>
  std::shared_ptr<T> GetOrSetPrivateComponent(const Entity &entity);

  /**
   * @brief Gets or sets a private component for an entity by type ID.
   * @param entity The entity for which to get or set the private component.
   * @param type_id The type ID of the private component.
   * @return A shared pointer to the private component.
   */
  std::shared_ptr<IPrivateComponent> GetOrSetPrivateComponent(const Entity &entity, const size_t &type_id);

  /**
   * @brief Sets a private component for an entity by ID.
   * @param entity The entity for which to set the private component.
   * @param id The ID of the private component.
   */
  void SetPrivateComponent(const Entity &entity, size_t id);

  /**
   * @brief Checks whether any entity currently owns a private component type.
   * @param type_id The private component type ID.
   * @return True when at least one owner exists.
   */
  [[nodiscard]] bool HasPrivateComponentOwners(const size_t &type_id) const;

  /**
   * @brief Clears cached private components of a type from the reuse pool.
   * @param type_id The private component type ID.
   * @return The number of pooled components released.
   */
  size_t ClearPrivateComponentPool(const size_t &type_id);

  /**
   * @brief Removes a private component of type T from an entity.
   * @tparam T The type of the private component.
   * @param entity The entity from which to remove the private component.
   * @param private_component The private component instance to remove.
   */
  template <typename T = IPrivateComponent>
  void RemovePrivateComponent(const Entity &entity, const std::shared_ptr<IPrivateComponent> &private_component);

  /**
   * @brief Unsafely retrieves the owners list for a private component type.
   * @tparam T The type of the private component.
   * @return A pointer to a vector of entities owning the private component type, or nullptr if not found.
   */
  template <typename T>
  const std::vector<Entity> *UnsafeGetOwnersList();

  /**
   * @brief Retrieves the owners list for a private component type.
   * @tparam T The type of the private component.
   * @return A vector of entities owning the private component type.
   */
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
    back->enabled_ = true;
    back->started_ = false;
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

template <typename T>
std::vector<Entity> PrivateComponentStorage::GetOwnersList() {
  if (const auto search = p_owners_collections_map_.find(typeid(T).hash_code());
      search != p_owners_collections_map_.end()) {
    return p_owners_collections_list_[search->second].second.owners_list;
  }
  return {};
}

}  // namespace evo_engine
