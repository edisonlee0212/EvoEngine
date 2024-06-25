#pragma once
#include "Console.hpp"
#include "Entity.hpp"
#include "ISingleton.hpp"
#include "Serialization.hpp"
#include "Transform.hpp"

namespace evo_engine {
template <typename T>
DataComponentType Typeof() {
  DataComponentType type;
  type.type_name = Serialization::GetDataComponentTypeName<T>();
  type.type_size = sizeof(T);
  type.type_offset = 0;
  type.type_index = typeid(T).hash_code();
  return type;
}
inline bool ComponentTypeComparator(const DataComponentType &a, const DataComponentType &b) {
  return a.type_index < b.type_index;
}

class Entities final : ISingleton<Entities> {
  friend class PhysicsSystem;
  friend class EditorLayer;
  friend class PrefabHolder;
  friend class PrivateComponentStorage;
  friend class TransformGraph;

  friend class Scene;
  friend class Serialization;
  friend struct EntityArchetype;
  friend struct EntityQuery;
  friend struct Entity;
  friend class Application;
  friend class PrivateComponentRef;
  friend class Prefab;
  size_t archetype_chunk_size_ = archetype_chunk_size;
  EntityArchetype basic_archetype_ = EntityArchetype();

  std::vector<EntityArchetypeInfo> entity_archetype_infos_;
  std::vector<EntityQueryInfo> entity_query_infos_;

#pragma region Helpers
  static EntityArchetype CreateEntityArchetypeHelper(const EntityArchetypeInfo &info);

  template <typename T = IDataComponent>
  static bool CheckDataComponentTypes(T arg);
  template <typename T = IDataComponent, typename... Ts>
  static bool CheckDataComponentTypes(T arg, Ts... args);
  template <typename T = IDataComponent>
  static size_t CollectDataComponentTypes(std::vector<DataComponentType> *component_types, T arg);
  template <typename T = IDataComponent, typename... Ts>
  static size_t CollectDataComponentTypes(std::vector<DataComponentType> *component_types, T arg, Ts... args);
  template <typename T = IDataComponent, typename... Ts>
  static std::vector<DataComponentType> CollectDataComponentTypes(T arg, Ts... args);

 public:
  static EntityArchetype CreateEntityArchetype(const std::string &name, const std::vector<DataComponentType> &types);

#pragma endregion

#pragma region EntityArchetype Methods
  static std::string GetEntityArchetypeName(const EntityArchetype &entity_archetype);
  static void SetEntityArchetypeName(const EntityArchetype &entity_archetype, const std::string &name);
#pragma endregion
#pragma region EntityQuery Methods
  template <typename T = IDataComponent, typename... Ts>
  static void SetEntityQueryAllFilters(const EntityQuery &entity_query, T arg, Ts... args);
  template <typename T = IDataComponent, typename... Ts>
  static void SetEntityQueryAnyFilters(const EntityQuery &entity_query, T arg, Ts... args);
  template <typename T = IDataComponent, typename... Ts>
  static void SetEntityQueryNoneFilters(const EntityQuery &entity_query, T arg, Ts... args);

  static EntityArchetype GetDefaultEntityArchetype();

  static size_t GetArchetypeChunkSize();
  static EntityArchetypeInfo GetArchetypeInfo(const EntityArchetype &entity_archetype);

  template <typename T = IDataComponent, typename... Ts>
  static EntityArchetype CreateEntityArchetype(const std::string &name, T arg, Ts... args);
  static EntityQuery CreateEntityQuery();

  static void Initialize();
};

#pragma endregion

#pragma region Functions

template <typename T, typename... Ts>
void Entities::SetEntityQueryAllFilters(const EntityQuery &entity_query, T arg, Ts... args) {
  assert(entity_query.IsValid());
  GetInstance().entity_query_infos_[entity_query.index_].all_data_component_types =
      CollectDataComponentTypes(arg, args...);
}

template <typename T, typename... Ts>
void Entities::SetEntityQueryAnyFilters(const EntityQuery &entity_query, T arg, Ts... args) {
  assert(entity_query.IsValid());
  GetInstance().entity_query_infos_[entity_query.index_].any_data_component_types =
      CollectDataComponentTypes(arg, args...);
}

template <typename T, typename... Ts>
void Entities::SetEntityQueryNoneFilters(const EntityQuery &entity_query, T arg, Ts... args) {
  assert(entity_query.IsValid());
  GetInstance().entity_query_infos_[entity_query.index_].none_data_component_types =
      CollectDataComponentTypes(arg, args...);
}
#pragma region Collectors

template <typename T>
bool Entities::CheckDataComponentTypes(T arg) {
  return std::is_standard_layout<T>::value;
}

template <typename T, typename... Ts>
bool Entities::CheckDataComponentTypes(T arg, Ts... args) {
  return std::is_standard_layout<T>::value && CheckDataComponentTypes(args...);
}

template <typename T>
size_t Entities::CollectDataComponentTypes(std::vector<DataComponentType> *component_types, T arg) {
  const auto type = Typeof<T>();
  component_types->push_back(type);
  return type.type_size;
}

template <typename T, typename... Ts>
size_t Entities::CollectDataComponentTypes(std::vector<DataComponentType> *component_types, T arg, Ts... args) {
  auto offset = CollectDataComponentTypes(component_types, args...);
  DataComponentType type = Typeof<T>();
  component_types->push_back(type);
  return type.type_size + offset;
}

template <typename T, typename... Ts>
std::vector<DataComponentType> Entities::CollectDataComponentTypes(T arg, Ts... args) {
  auto ret_val = std::vector<DataComponentType>();
  ret_val.push_back(Typeof<Transform>());
  ret_val.push_back(Typeof<GlobalTransform>());
  ret_val.push_back(Typeof<TransformUpdateFlag>());
  CollectDataComponentTypes(&ret_val, arg, args...);
  std::sort(ret_val.begin() + 3, ret_val.end(), ComponentTypeComparator);
  size_t offset = 0;

  std::vector<DataComponentType> copy;
  copy.insert(copy.begin(), ret_val.begin(), ret_val.end());
  ret_val.clear();
  for (const auto &i : copy) {
    bool found = false;
    for (const auto j : ret_val) {
      if (i == j) {
        found = true;
        break;
      }
    }
    if (found)
      continue;
    ret_val.push_back(i);
  }
  for (auto &i : ret_val) {
    i.type_offset = offset;
    offset += i.type_size;
  }
  return ret_val;
}
#pragma endregion

#pragma region Others

template <typename T, typename... Ts>
EntityArchetype Entities::CreateEntityArchetype(const std::string &name, T arg, Ts... args) {
  auto return_value = EntityArchetype();
  if (!CheckDataComponentTypes(arg, args...)) {
    EVOENGINE_ERROR("CreateEntityArchetype failed: Standard Layout");
    return return_value;
  }
  EntityArchetypeInfo info;
  info.archetype_name = name;
  info.data_component_types = CollectDataComponentTypes(arg, args...);
  info.entity_size = info.data_component_types.back().type_offset + info.data_component_types.back().type_size;
  info.chunk_capacity = GetInstance().archetype_chunk_size_ / info.entity_size;
  return_value = CreateEntityArchetypeHelper(info);
  return return_value;
}
#pragma endregion

template <typename T>
T ComponentDataChunk::GetData(const size_t &offset) {
  return T(*reinterpret_cast<T *>(static_cast<char *>(chunk_data) + offset));
}

template <typename T>
void ComponentDataChunk::SetData(const size_t &offset, const T &data) {
  *reinterpret_cast<T *>(static_cast<char *>(chunk_data) + offset) = data;
}

template <typename T, typename... Ts>
void EntityQuery::SetAllFilters(T arg, Ts... args) {
  Entities::SetEntityQueryAllFilters(*this, arg, args...);
}

template <typename T, typename... Ts>
void EntityQuery::SetAnyFilters(T arg, Ts... args) {
  Entities::SetEntityQueryAnyFilters(*this, arg, args...);
}

template <typename T, typename... Ts>
void EntityQuery::SetNoneFilters(T arg, Ts... args) {
  Entities::SetEntityQueryNoneFilters(*this, arg, args...);
}

#pragma endregion

}  // namespace evo_engine
