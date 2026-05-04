
#pragma once
#include "Console.hpp"
#include "Entity.hpp"
#include "Serialization.hpp"
#include "Transform.hpp"

namespace evo_engine {

/**
 * @brief Generates a DataComponentType object for the given type.
 * @tparam T Data type for which the DataComponentType object is to be created.
 * @return DataComponentType object representing the type information.
 */
template <typename T>
DataComponentType Typeof() {
  DataComponentType type;
  type.type_name = Serialization::GetDataComponentTypeName<T>();
  type.type_size = sizeof(T);
  type.type_offset = 0;
  type.type_index = typeid(T).hash_code();
  return type;
}

/**
 * @brief Compares two DataComponentType objects based on their type_index.
 * @param a First DataComponentType object.
 * @param b Second DataComponentType object.
 * @return True if `a` has a smaller type_index than `b`, otherwise false.
 */
inline bool ComponentTypeComparator(const DataComponentType &a, const DataComponentType &b) {
  return a.type_index < b.type_index;
}

/**
 * @brief Manages and operates on entities within the engine framework.
 */
class Entities final {
 public:
  static Entities &GetInstance();

 private:
  // Granting access to specific classes and structures
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
  /**
   * @brief Creates an entity archetype based on the provided archetype info.
   * @param info Information about the entity archetype to be created.
   * @return A newly created EntityArchetype object.
   */
  static EntityArchetype CreateEntityArchetypeHelper(const EntityArchetypeInfo &info);

  /**
   * @brief Checks if the given data component type is standard-layout compliant.
   * @tparam T Type of the data component to be checked.
   * @param arg The data component instance.
   * @return True if the type is standard-layout compliant, otherwise false.
   */
  template <typename T = IDataComponent>
  static bool CheckDataComponentTypes(T arg);

  /**
   * @brief Recursively checks if all provided data component types are standard-layout compliant.
   * @tparam T First type to check.
   * @tparam Ts Remaining types to check.
   * @param arg First data component instance.
   * @param args Remaining data component instances.
   * @return True if all types are standard-layout compliant, otherwise false.
   */
  template <typename T = IDataComponent, typename... Ts>
  static bool CheckDataComponentTypes(T arg, Ts... args);

  /**
   * @brief Collects the type information for a data component and appends it to the provided type vector.
   * @tparam T Data type for which type information is collected.
   * @param component_types Vector to hold collected type information.
   * @param arg Data component instance.
   * @return Total size of the data type.
   */
  template <typename T = IDataComponent>
  static size_t CollectDataComponentTypes(std::vector<DataComponentType> *component_types, T arg);

  /**
   * @brief Recursively collects type information for multiple data components and appends them to the provided type
   * vector.
   * @tparam T First type to collect.
   * @tparam Ts Remaining types to collect.
   * @param component_types Vector to hold collected type information.
   * @param arg First data component instance.
   * @param args Remaining data component instances.
   * @return Cumulative size of all data types.
   */
  template <typename T = IDataComponent, typename... Ts>
  static size_t CollectDataComponentTypes(std::vector<DataComponentType> *component_types, T arg, Ts... args);

  /**
   * @brief Collects type information for multiple data components and returns them in a vector.
   * @tparam T First type to collect.
   * @tparam Ts Remaining types to collect.
   * @param arg First data component instance.
   * @param args Remaining data component instances.
   * @return Vector containing type information of all collected data components.
   */
  template <typename T = IDataComponent, typename... Ts>
  static std::vector<DataComponentType> CollectDataComponentTypes(T arg, Ts... args);

 public:
  /**
   * @brief Creates an entity archetype with the given name and types.
   * @param name Name of the entity archetype.
   * @param types A vector of DataComponentType describes the owned data components of the target type entity.
   * @return The created EntityArchetype.
   */
  static EntityArchetype CreateEntityArchetype(const std::string &name, const std::vector<DataComponentType> &types);

#pragma endregion

#pragma region EntityArchetype Methods

  /**
   * @brief Retrieves the name of an entity archetype.
   * @param entity_archetype The entity archetype whose name is to be retrieved.
   * @return The name of the entity archetype.
   */
  static std::string GetEntityArchetypeName(const EntityArchetype &entity_archetype);

  /**
   * @brief Sets the name of an entity archetype.
   * @param entity_archetype The entity archetype whose name is to be set.
   * @param name The new name for the entity archetype.
   */
  static void SetEntityArchetypeName(const EntityArchetype &entity_archetype, const std::string &name);

#pragma endregion

#pragma region EntityQuery Methods

  /**
   * @brief Sets 'all' filter types for the given entity query.
   * @tparam T First type to filter by.
   * @tparam Ts Remaining types to filter by.
   * @param entity_query The entity query to set filters for.
   * @param arg First filter type instance.
   * @param args Remaining filter type instances.
   */
  template <typename T = IDataComponent, typename... Ts>
  static void SetEntityQueryAllFilters(const EntityQuery &entity_query, T arg, Ts... args);

  /**
   * @brief ... (continue with the rest of the Doxygen comments) ...
   */

  /**
   * @brief Sets 'any' filter types for the given entity query.
   * @tparam T First type to filter by.
   * @tparam Ts Remaining types to filter by.
   * @param entity_query The entity query to set filters for.
   * @param arg First filter type instance.
   * @param args Remaining filter type instances.
   */
  template <typename T = IDataComponent, typename... Ts>
  static void SetEntityQueryAnyFilters(const EntityQuery &entity_query, T arg, Ts... args);

  /**
   * @brief Sets 'none' filter types for the given entity query.
   * @tparam T First type to exclude.
   * @tparam Ts Remaining types to exclude.
   * @param entity_query The entity query to set filters for.
   * @param arg First filter type instance.
   * @param args Remaining filter type instances.
   */
  template <typename T = IDataComponent, typename... Ts>
  static void SetEntityQueryNoneFilters(const EntityQuery &entity_query, T arg, Ts... args);

  /**
   * @brief Retrieves the default entity archetype used by the Entities class.
   * @return The default EntityArchetype.
   */
  static EntityArchetype GetDefaultEntityArchetype();

  /**
   * @brief Retrieves the size of chunks used for storing archetypes.
   * @return The archetype chunk size.
   */
  static size_t GetArchetypeChunkSize();

  /**
   * @brief Retrieves the information associated with a specific entity archetype.
   * @param entity_archetype The entity archetype whose information is to be retrieved.
   * @return EntityArchetypeInfo object containing the archetype's information.
   */
  static EntityArchetypeInfo GetArchetypeInfo(const EntityArchetype &entity_archetype);

  /**
   * @brief Creates an entity archetype using the given name and types.
   * @tparam T First type for the archetype.
   * @tparam Ts Additional types for the archetype.
   * @param name Name of the entity archetype.
   * @param arg First type instance.
   * @param args Additional type instances.
   * @return The created EntityArchetype.
   */
  template <typename T = IDataComponent, typename... Ts>
  static EntityArchetype CreateEntityArchetype(const std::string &name, T arg, Ts... args);

  /**
   * @brief Creates a new entity query object.
   * @return The created EntityQuery.
   */
  static EntityQuery CreateEntityQuery();

  /**
   * @brief Initializes the Entities class and its associated resources.
   */
  static void Initialize();
};
#pragma endregion

#pragma region Functions

/**
 * @brief Sets 'all' filters for the specified entity query.
 * @tparam T First type to filter by.
 * @tparam Ts Remaining types to filter by.
 * @param entity_query The entity query to set filters for.
 * @param arg First filter type instance.
 * @param args Remaining filter type instances.
 */
template <typename T, typename... Ts>
void Entities::SetEntityQueryAllFilters(const EntityQuery &entity_query, T arg, Ts... args) {
  assert(entity_query.IsValid());
  GetInstance().entity_query_infos_[entity_query.index_].all_data_component_types =
      CollectDataComponentTypes(arg, args...);
}

/**
 * @brief Sets 'any' filters for the specified entity query.
 * @tparam T First type to filter by.
 * @tparam Ts Remaining types to filter by.
 * @param entity_query The entity query to set filters for.
 * @param arg First filter type instance.
 * @param args Remaining filter type instances.
 */
template <typename T, typename... Ts>
void Entities::SetEntityQueryAnyFilters(const EntityQuery &entity_query, T arg, Ts... args) {
  assert(entity_query.IsValid());
  GetInstance().entity_query_infos_[entity_query.index_].any_data_component_types =
      CollectDataComponentTypes(arg, args...);
}

/**
 * @brief Sets 'none' filters for the specified entity query.
 * @tparam T First type to exclude.
 * @tparam Ts Remaining types to exclude.
 * @param entity_query The entity query to set filters for.
 * @param arg First filter type instance.
 * @param args Remaining filter type instances.
 */
template <typename T, typename... Ts>
void Entities::SetEntityQueryNoneFilters(const EntityQuery &entity_query, T arg, Ts... args) {
  assert(entity_query.IsValid());
  GetInstance().entity_query_infos_[entity_query.index_].none_data_component_types =
      CollectDataComponentTypes(arg, args...);
}
#pragma region Collectors

/**
 * @brief Checks if a data component type is standard-layout compliant.
 * @tparam T The type to check.
 * @param arg The data component.
 * @return True if the type is compliant, otherwise false.
 */
template <typename T>
bool Entities::CheckDataComponentTypes(T arg) {
  return std::is_standard_layout_v<T>;
}

/**
 * @brief Recursively checks if all provided types are standard-layout compliant.
 * @tparam T First type to check.
 * @tparam Ts Remaining types to check.
 * @param arg First data component.
 * @param args Remaining data components.
 * @return True if all types are compliant, otherwise false.
 */
template <typename T, typename... Ts>
bool Entities::CheckDataComponentTypes(T arg, Ts... args) {
  return std::is_standard_layout_v<T> && CheckDataComponentTypes(args...);
}

/**
 * @brief Collects type information for a single data component.
 * @tparam T The type of the data component.
 * @param component_types Vector to store collected type information.
 * @param arg The data component.
 * @return Size of the data type.
 */
template <typename T>
size_t Entities::CollectDataComponentTypes(std::vector<DataComponentType> *component_types, T arg) {
  const auto type = Typeof<T>();
  component_types->push_back(type);
  return type.type_size;
}

/**
 * @brief Recursively collects type information for multiple data components.
 * @tparam T First type to process.
 * @tparam Ts Remaining types to process.
 * @param component_types Vector to store collected type information.
 * @param arg First data component.
 * @param args Remaining data components.
 * @return Cumulative size of all data types.
 */
template <typename T, typename... Ts>
size_t Entities::CollectDataComponentTypes(std::vector<DataComponentType> *component_types, T arg, Ts... args) {
  auto offset = CollectDataComponentTypes(component_types, args...);
  DataComponentType type = Typeof<T>();
  component_types->push_back(type);
  return type.type_size + offset;
}

/**
 * @brief Collects type information for multiple data components and returns them as a vector.
 * @tparam T First type to process.
 * @tparam Ts Remaining types to process.
 * @param arg First data component.
 * @param args Remaining data components.
 * @return A vector containing collected type information for all data components.
 */
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

/**
 * @brief Creates an entity archetype with the specified name and data component types.
 * @tparam T First data component type.
 * @tparam Ts Remaining data component types.
 * @param name The name of the entity archetype.
 * @param arg First data component type instance.
 * @param args Remaining data component type instances.
 * @return A newly created EntityArchetype object.
 */
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

/**
 * @brief Retrieves the data from a component data chunk at the specified offset.
 * @tparam T The data type to retrieve.
 * @param offset Offset from the beginning of the chunk.
 * @return The value of type T located at the given offset in the chunk.
 */
template <typename T>
T ComponentDataChunk::GetData(const size_t &offset) const {
  return T(*reinterpret_cast<const T *>(chunk_data_.data() + offset));
}

/**
 * @brief Adds 'all' filters to the entity query for multiple data component types.
 * @tparam T First data component type.
 * @tparam Ts Remaining data component types.
 * @param arg First data component type instance.
 * @param args Remaining data component type instances.
 */
template <typename T, typename... Ts>
void EntityQuery::SetAllFilters(T arg, Ts... args) {
  Entities::SetEntityQueryAllFilters(*this, arg, args...);
}

/**
 * @brief Adds 'any' filters to the entity query for multiple data component types.
 * @tparam T First data component type.
 * @tparam Ts Remaining data component types.
 * @param arg First data component type instance.
 * @param args Remaining data component type instances.
 */
template <typename T, typename... Ts>
void EntityQuery::SetAnyFilters(T arg, Ts... args) {
  Entities::SetEntityQueryAnyFilters(*this, arg, args...);
}

/**
 * @brief Adds 'none' filters to the entity query for multiple data component types.
 * @tparam T First data component type.
 * @tparam Ts Remaining data component types.
 * @param arg First data component type instance.
 * @param args Remaining data component type instances.
 */
template <typename T, typename... Ts>
void EntityQuery::SetNoneFilters(T arg, Ts... args) {
  Entities::SetEntityQueryNoneFilters(*this, arg, args...);
}

#pragma endregion

}  // namespace evo_engine
