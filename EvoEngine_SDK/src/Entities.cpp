#include "Entities.hpp"
#include "Entity.hpp"
#include "Scene.hpp"
using namespace evo_engine;

#pragma region EntityManager

size_t Entities::GetArchetypeChunkSize() {
  const auto &entities = GetInstance();
  return entities.archetype_chunk_size_;
}

EntityArchetype Entities::CreateEntityArchetype(const std::string &name, const std::vector<DataComponentType> &types) {
  auto &entities = GetInstance();
  EntityArchetypeInfo entity_archetype_info;
  entity_archetype_info.archetype_name = name;
  std::vector<DataComponentType> actual_types;
  actual_types.push_back(Typeof<Transform>());
  actual_types.push_back(Typeof<GlobalTransform>());
  actual_types.push_back(Typeof<TransformUpdateFlag>());
  actual_types.insert(actual_types.end(), types.begin(), types.end());
  std::sort(actual_types.begin() + 3, actual_types.end(), ComponentTypeComparator);
  size_t offset = 0;
  DataComponentType prev = actual_types[0];
  // Erase duplicates
  std::vector<DataComponentType> copy;
  copy.insert(copy.begin(), actual_types.begin(), actual_types.end());
  actual_types.clear();
  for (const auto &i : copy) {
    bool found = false;
    for (const auto j : actual_types) {
      if (i == j) {
        found = true;
        break;
      }
    }
    if (found)
      continue;
    actual_types.push_back(i);
  }

  for (auto &i : actual_types) {
    i.type_offset = offset;
    offset += i.type_size;
  }
  entity_archetype_info.data_component_types = actual_types;
  entity_archetype_info.entity_size = entity_archetype_info.data_component_types.back().type_offset +
                                    entity_archetype_info.data_component_types.back().type_size;
  entity_archetype_info.chunk_capacity = entities.archetype_chunk_size_ / entity_archetype_info.entity_size;
  return CreateEntityArchetypeHelper(entity_archetype_info);
}

EntityArchetype Entities::GetDefaultEntityArchetype() {
  auto &entities = GetInstance();
  return entities.basic_archetype_;
}

EntityArchetypeInfo Entities::GetArchetypeInfo(const EntityArchetype &entity_archetype) {
  auto &entities = GetInstance();
  return entities.entity_archetype_infos_[entity_archetype.index_];
}

EntityQuery Entities::CreateEntityQuery() {
  EntityQuery ret_val;
  auto &entities = GetInstance();
  ret_val.index_ = entities.entity_query_infos_.size();
  EntityQueryInfo info;
  info.query_index = ret_val.index_;
  entities.entity_query_infos_.resize(entities.entity_query_infos_.size() + 1);
  entities.entity_query_infos_[info.query_index] = info;
  return ret_val;
}

std::string Entities::GetEntityArchetypeName(const EntityArchetype &entity_archetype) {
  auto &entities = GetInstance();
  return entities.entity_archetype_infos_[entity_archetype.index_].archetype_name;
}

void Entities::SetEntityArchetypeName(const EntityArchetype &entity_archetype, const std::string &name) {
  auto &entities = GetInstance();
  entities.entity_archetype_infos_[entity_archetype.index_].archetype_name = name;
}

void Entities::Initialize() {
  auto &entities = GetInstance();
  entities.entity_archetype_infos_.emplace_back();
  entities.entity_query_infos_.emplace_back();

  entities.basic_archetype_ =
      CreateEntityArchetype("Basic", Transform(), GlobalTransform(), TransformUpdateFlag());
}

EntityArchetype Entities::CreateEntityArchetypeHelper(const EntityArchetypeInfo &info) {
  EntityArchetype ret_val;
  auto &entity_manager = GetInstance();
  auto &entity_archetype_infos = entity_manager.entity_archetype_infos_;
  int duplicate_index = -1;
  for (size_t i = 1; i < entity_archetype_infos.size(); i++) {
    EntityArchetypeInfo &compare_info = entity_archetype_infos[i];
    if (info.chunk_capacity != compare_info.chunk_capacity)
      continue;
    if (info.entity_size != compare_info.entity_size)
      continue;
    bool type_check = true;

    for (auto &component_type : info.data_component_types) {
      if (!compare_info.HasType(component_type.type_index))
        type_check = false;
    }
    if (type_check) {
      duplicate_index = i;
      break;
    }
  }
  if (duplicate_index == -1) {
    ret_val.index_ = entity_archetype_infos.size();
    entity_archetype_infos.push_back(info);
  } else {
    ret_val.index_ = duplicate_index;
  }
  return ret_val;
}

#pragma endregion
