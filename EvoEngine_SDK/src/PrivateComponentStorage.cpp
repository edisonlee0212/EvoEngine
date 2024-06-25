#include "PrivateComponentStorage.hpp"
#include "Entities.hpp"
#include "Scene.hpp"
using namespace evo_engine;
void PrivateComponentStorage::RemovePrivateComponent(const Entity &entity, const size_t type_index,
                                                     const std::shared_ptr<IPrivateComponent> &private_component) {
  if (const auto search = p_owners_collections_map_.find(type_index); search != p_owners_collections_map_.end()) {
    auto &collection = p_owners_collections_list_[search->second].second;
    if (const auto entity_search = collection.owners_map.find(entity); entity_search != collection.owners_map.end()) {
      if (entity != entity_search->first) {
        EVOENGINE_ERROR("RemovePrivateComponent: Entity mismatch!");
        return;
      }
      private_component->OnDestroy();
      private_component->version_++;
      private_component_pool_[type_index].push_back(private_component);
      if (collection.owners_list.size() == 1) {
        const auto erase_hash = type_index;
        const auto erase_index = search->second;
        const auto back_hash = p_owners_collections_list_.back().first;
        p_owners_collections_map_[back_hash] = erase_index;
        std::swap(p_owners_collections_list_[erase_index], p_owners_collections_list_.back());
        p_owners_collections_map_.erase(erase_hash);
        p_owners_collections_list_.pop_back();
      } else {
        const auto erase_index = entity_search->second;
        const auto back_entity = collection.owners_list.back();
        collection.owners_map[back_entity] = erase_index;
        collection.owners_map.erase(entity);
        collection.owners_list[erase_index] = back_entity;
        collection.owners_list.pop_back();
      }
    }
  }
}

void PrivateComponentStorage::DeleteEntity(const Entity &entity) {
  const auto scene = owner_scene.lock();
  for (auto &element :
       scene->scene_data_storage_.entity_metadata_list.at(entity.GetIndex()).private_component_elements) {
    RemovePrivateComponent(entity, element.type_index, element.private_component_data);
  }
}

void PrivateComponentStorage::SetPrivateComponent(const Entity &entity, size_t id) {
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
}
template <typename T>
std::vector<Entity> PrivateComponentStorage::GetOwnersList() {
  if (const auto search = p_owners_collections_map_.find(typeid(T).hash_code());
      search != p_owners_collections_map_.end()) {
    return p_owners_collections_list_[search->second].second.owners_list;
  }
  return {};
}
