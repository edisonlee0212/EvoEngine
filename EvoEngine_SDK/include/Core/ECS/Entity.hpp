
#pragma once
#include "IDataComponent.hpp"
#include "IHandle.hpp"

namespace evo_engine {

#pragma region EntityManager
#pragma region Entity

class Scene;

/**
 * @brief Represents metadata information about a data component type.
 */
struct DataComponentType final {
  std::string type_name;   ///< Name of the data component type.
  size_t type_index = 0;   ///< Unique index of the data component type.
  size_t type_size = 0;    ///< Size of the data component type in bytes.
  size_t type_offset = 0;  ///< Memory offset of the data component type.

  /**
   * @brief Default constructor for DataComponentType.
   */
  DataComponentType() = default;

  /**
   * @brief Constructs a DataComponentType with specified properties.
   * @param name Name of the data component type.
   * @param id Unique index of the data component type.
   * @param size Size of the data component type in bytes.
   */
  DataComponentType(const std::string &name, const size_t &id, const size_t &size);

  /**
   * @brief Equality operator for comparing DataComponentType objects.
   * @param other The other DataComponentType to compare with.
   * @return True if the objects are equal, false otherwise.
   */
  bool operator==(const DataComponentType &other) const;

  /**
   * @brief Inequality operator for comparing DataComponentType objects.
   * @param other The other DataComponentType to compare with.
   * @return True if the objects are not equal, false otherwise.
   */
  bool operator!=(const DataComponentType &other) const;
};

/**
 * @brief Represents an entity archetype within the engine.
 */
struct EntityArchetype final {
 private:
  friend class Entities;
  friend class Serialization;
  friend class Scene;

  size_t index_ = 0;  ///< Index of the archetype.

 public:
  /**
   * @brief Gets the index of the entity archetype.
   * @return Index of the entity archetype.
   */
  size_t GetIndex() const;

  /**
   * @brief Checks if the archetype is null.
   * @return True if the archetype is null, false otherwise.
   */
  [[nodiscard]] bool IsNull() const;

  /**
   * @brief Checks if the archetype is valid.
   * @return True if the archetype is valid, false otherwise.
   */
  [[nodiscard]] bool IsValid() const;

  /**
   * @brief Gets the name of the archetype.
   * @return Name of the archetype.
   */
  [[nodiscard]] std::string GetName() const;

  /**
   * @brief Sets the name of the archetype.
   * @param name The new name for the archetype.
   */
  void SetName(const std::string &name) const;
};

class IPrivateComponent;

/**
 * @brief Represents an individual entity in the engine.
 */
struct Entity final {
 private:
  friend class Entities;
  friend class Scene;
  friend struct EntityMetadata;
  friend class Serialization;

  uint32_t index_ = 0;    ///< Index of the entity.
  uint32_t version_ = 0;  ///< Version of the entity.

 public:
  /**
   * @brief Gets the index of the entity.
   * @return Index of the entity.
   */
  [[nodiscard]] uint32_t GetIndex() const;

  /**
   * @brief Gets the version of the entity.
   * @return Version of the entity.
   */
  [[nodiscard]] uint32_t GetVersion() const;

  /**
   * @brief Equality operator for entities.
   * @param other The other entity to compare with.
   * @return True if the entities are equal, false otherwise.
   */
  bool operator==(const Entity &other) const;

  /**
   * @brief Inequality operator for entities.
   * @param other The other entity to compare with.
   * @return True if the entities are not equal, false otherwise.
   */
  bool operator!=(const Entity &other) const;

  /**
   * @brief Hash function for the entity.
   * @param key Entity key.
   * @return Hash value of the entity.
   */
  uint32_t operator()(Entity const &key) const;
};

#pragma region Storage

/**
 * @brief Represents a reference to an entity.
 */
class EntityRef final {
  Entity value_ = Entity();           ///< Entity value.
  Handle entity_handle_ = Handle(0);  ///< Handle representing the entity.

  /**
   * @brief Updates the internal reference to the entity.
   */
  void Update();

 public:
  /**
   * @brief Serializes the EntityRef into YAML format.
   * @param out The YAML emitter to serialize into.
   */
  void Serialize(YAML::Emitter &out) const {
    out << YAML::Key << "entity_handle_" << YAML::Value << entity_handle_;
  }

  /**
   * @brief Deserializes the EntityRef from a YAML node.
   * @param in The YAML node containing the serialized data.
   */
  void Deserialize(const YAML::Node &in) {
    entity_handle_ = Handle(in["entity_handle_"].as<uint64_t>());
  }

  /**
   * @brief Default constructor for EntityRef.
   */
  EntityRef() {
    entity_handle_ = Handle(0);
    value_ = Entity();
  }

  /**
   * @brief Constructs an EntityRef from a given Entity.
   * @param other The entity to reference.
   */
  EntityRef(const Entity &other) {
    Set(other);
  }

  /**
   * @brief Assignment operator for EntityRef.
   * @param other The entity to assign to the reference.
   * @return Reference to the current EntityRef instance.
   */
  EntityRef &operator=(const Entity &other) {
    Set(other);
    return *this;
  }

  /**
   * @brief Move assignment operator for EntityRef.
   * @param other The entity to move assign to the reference.
   * @return Reference to the current EntityRef instance.
   */
  EntityRef &operator=(Entity &&other) noexcept {
    Set(other);
    return *this;
  }

  /**
   * @brief Relinks the entity handle using a mapping of handles.
   * @param map Mapping of old handles to new handles.
   */
  void Relink(const std::unordered_map<Handle, Handle> &map) {
    if (entity_handle_.GetValue() == 0)
      return;
    if (const auto search = map.find(entity_handle_); search != map.end()) {
      entity_handle_ = search->second;
      value_ = Entity();
    } else {
      Clear();
    }
  }

  /**
   * @brief Gets the entity referenced by the EntityRef.
   * @return The referenced entity.
   */
  [[nodiscard]] Entity Get() {
    Update();
    return value_;
  }

  /**
   * @brief Gets the handle of the entity referenced by the EntityRef.
   * @return The handle of the entity.
   */
  [[nodiscard]] Handle GetEntityHandle() const {
    return entity_handle_;
  }

  /**
   * @brief Sets the entity to reference.
   * @param target The entity to reference.
   */
  void Set(const Entity &target);

  /**
   * @brief Clears the reference to the entity.
   */
  void Clear();

  /**
   * @brief Saves the EntityRef to YAML format with a specified name.
   * @param name The name for the saved data.
   * @param out The YAML emitter to serialize into.
   */
  void Save(const std::string &name, YAML::Emitter &out) const {
    out << YAML::Key << name << YAML::Value << YAML::BeginMap;
    Serialize(out);
    out << YAML::EndMap;
  }

  /**
   * @brief Loads the EntityRef from YAML format with a specified name.
   * @param name The name of the saved data.
   * @param in The YAML node containing the serialized data.
   */
  void Load(const std::string &name, const YAML::Node &in) {
    if (in[name])
      Deserialize(in[name]);
  }
};

/**
 * @brief Saves a list of EntityRef objects to YAML format.
 * @param name The name for the saved data.
 * @param target The list of EntityRef objects to save.
 * @param out The YAML emitter to serialize into.
 */
inline void SaveList(const std::string &name, const std::vector<EntityRef> &target, YAML::Emitter &out) {
  if (target.empty())
    return;
  out << YAML::Key << name << YAML::Value << YAML::BeginSeq;
  for (auto &i : target) {
    out << YAML::BeginMap;
    i.Serialize(out);
    out << YAML::EndMap;
  }
  out << YAML::EndSeq;
}

/**
 * @brief Loads a list of EntityRef objects from YAML format.
 * @param name The name of the saved data.
 * @param target The list to populate with the loaded EntityRef objects.
 * @param in The YAML node containing the serialized data.
 */
inline void LoadList(const std::string &name, std::vector<EntityRef> &target, const YAML::Node &in) {
  if (in[name]) {
    target.clear();
    for (const auto &i : in[name]) {
      EntityRef instance;
      instance.Deserialize(i);
      target.push_back(instance);
    }
  }
}
constexpr size_t archetype_chunk_size = 16384;
/**
 * @brief Represents a chunk of memory for storing component data.
 */
class ComponentDataChunk {
  std::vector<char> chunk_data_ = std::vector<char>(archetype_chunk_size);  ///< Internal memory chunk.

 public:
  /**
   * @brief Retrieves data of a specified type from the chunk.
   * @tparam T The type of the data to retrieve.
   * @param offset The offset in the chunk where the data is located.
   * @return The data of type T at the specified offset.
   */
  template <typename T>
  T GetData(const size_t &offset) const;

  /**
   * @brief Gets a reference to raw data at a specified offset.
   * @param offset The offset in the chunk where the data is located.
   * @return Pointer to the raw data.
   */
  [[nodiscard]] void *RefData(const size_t &offset);

  /**
   * @brief Gets a constant reference to raw data at a specified offset.
   * @param offset The offset in the chunk where the data is located.
   * @return Constant pointer to the raw data.
   */
  [[nodiscard]] const void *PeekData(const size_t &offset) const;

  /**
   * @brief Sets data in the chunk at a specified offset.
   * @param offset The offset in the chunk to set the data.
   * @param size The size of the data to set.
   * @param data Pointer to the data to set.
   */
  void SetData(const size_t &offset, const size_t &size, const void *data);

  /**
   * @brief Clears data in the chunk at a specified offset.
   * @param offset The offset in the chunk to clear the data.
   * @param size The size of the data to clear.
   */
  void ClearData(const size_t &offset, const size_t &size);
};

/**
 * @brief Represents an array of data component chunks and their associated entities.
 */
struct DataComponentChunkArray {
  std::vector<Entity> entity_array{};        ///< Array of entities.
  std::vector<ComponentDataChunk> chunks{};  ///< Array of memory chunks for component data.

  /**
   * @brief Assignment operator for DataComponentChunkArray.
   * @param source The source DataComponentChunkArray to copy from.
   * @return Reference to the current DataComponentChunkArray instance.
   */
  DataComponentChunkArray &operator=(const DataComponentChunkArray &source);
};

/**
 * @brief Information about an entity archetype.
 */
struct EntityArchetypeInfo {
  std::string archetype_name = "New Entity Archetype";    ///< Name of the archetype.
  size_t entity_size = 0;                                 ///< Size of a single entity.
  size_t chunk_capacity = 0;                              ///< Capacity of each chunk.
  std::vector<DataComponentType> data_component_types{};  ///< Types of components associated with the archetype.

  /**
   * @brief Checks if the archetype has a specific type.
   * @tparam T The type to check for.
   * @return True if the archetype has the specified type, false otherwise.
   */
  template <typename T>
  bool HasType() const;

  /**
   * @brief Checks if the archetype has a specific type by index.
   * @param type_index The index of the type to check for.
   * @return True if the archetype has the specified type, false otherwise.
   */
  bool HasType(const size_t &type_index) const;
};

/**
 * @brief Represents a query for entities matching specified filters.
 */
struct EntityQuery final {
 private:
  friend class Entities;
  friend class Scene;
  friend class Serialization;

  size_t index_ = 0;  ///< Index of the entity query.

 public:
  /**
   * @brief Gets the index of the entity query.
   * @return The index of the entity query.
   */
  size_t GetIndex() const;

  /**
   * @brief Equality operator for EntityQuery objects.
   * @param other The other EntityQuery to compare with.
   * @return True if the queries are equal, false otherwise.
   */
  bool operator==(const EntityQuery &other) const;

  /**
   * @brief Inequality operator for EntityQuery objects.
   * @param other The other EntityQuery to compare with.
   * @return True if the queries are not equal, false otherwise.
   */
  bool operator!=(const EntityQuery &other) const;

  /**
   * @brief Hash function for the entity query.
   * @param key The entity query key.
   * @return Hash value of the entity query.
   */
  size_t operator()(const EntityQuery &key) const;

  /**
   * @brief Checks if the query is null.
   * @return True if the query is null, false otherwise.
   */
  [[nodiscard]] bool IsNull() const;

  /**
   * @brief Checks if the query is valid.
   * @return True if the query is valid, false otherwise.
   */
  [[nodiscard]] bool IsValid() const;

  /**
   * @brief Sets all-filters for the query.
   * @tparam T The first component type.
   * @tparam Ts The rest of the component types.
   * @param arg The first filter component.
   * @param args The rest of the filter components.
   */
  template <typename T = IDataComponent, typename... Ts>
  void SetAllFilters(T arg, Ts... args);

  /**
   * @brief Sets any-filters for the query.
   * @tparam T The first component type.
   * @tparam Ts The rest of the component types.
   * @param arg The first filter component.
   * @param args The rest of the filter components.
   */
  template <typename T = IDataComponent, typename... Ts>
  void SetAnyFilters(T arg, Ts... args);

  /**
   * @brief Sets none-filters for the query.
   * @tparam T The first component type.
   * @tparam Ts The rest of the component types.
   * @param arg The first filter component.
   * @param args The rest of the filter components.
   */
  template <typename T = IDataComponent, typename... Ts>
  void SetNoneFilters(T arg, Ts... args);
};

/**
 * @brief Represents storage for data components in the engine.
 */
struct DataComponentStorage {
  std::vector<DataComponentType> data_component_types;  ///< List of data component types in the storage.
  size_t entity_size = 0;                               ///< Size of an individual entity.
  size_t chunk_capacity = 0;                            ///< Number of entities a chunk can hold.
  size_t entity_count = 0;                              ///< Total number of entities in the storage.
  size_t entity_alive_count = 0;                        ///< Number of currently active (alive) entities.
  DataComponentChunkArray chunk_array{};                ///< Array of memory chunks for this storage.

  /**
   * @brief Default constructor for DataComponentStorage.
   */
  DataComponentStorage() = default;

  /**
   * @brief Constructs a DataComponentStorage from the provided archetype information.
   * @param entity_archetype_info Information about the entity archetype.
   */
  DataComponentStorage(const EntityArchetypeInfo &entity_archetype_info);

  /**
   * @brief Assignment operator for DataComponentStorage.
   * @param source The source DataComponentStorage to copy from.
   * @return Reference to the current DataComponentStorage instance.
   */
  DataComponentStorage &operator=(const DataComponentStorage &source);

  /**
   * @brief Checks if the storage contains a specific type.
   * @tparam T The type to check for.
   * @return True if the storage contains the specified type, false otherwise.
   */
  template <typename T>
  bool HasType() const;

  /**
   * @brief Checks if the storage contains a specific type by its index.
   * @param type_id The index of the type to check for.
   * @return True if the storage contains the specified type, false otherwise.
   */
  bool HasType(const size_t &type_id) const;
};

/**
 * @brief Represents information about an entity query.
 */
struct EntityQueryInfo {
  size_t query_index = 0;                                    ///< Index of the query.
  std::vector<DataComponentType> all_data_component_types;   ///< Required component types (all filters).
  std::vector<DataComponentType> any_data_component_types;   ///< Optional component types (any filters).
  std::vector<DataComponentType> none_data_component_types;  ///< Excluded component types (none filters).
};

/**
 * @brief Checks if the archetype contains a specific type.
 * @tparam T The type to check for.
 * @return True if the archetype contains the specified type, false otherwise.
 */
template <typename T>
bool EntityArchetypeInfo::HasType() const {
  for (const auto &i : data_component_types) {
    if (i.type_index == typeid(T).hash_code())
      return true;
  }
  return false;
}

/**
 * @brief Checks if the storage contains a specific type.
 * @tparam T The type to check for.
 * @return True if the storage contains the specified type, false otherwise.
 */
template <typename T>
bool DataComponentStorage::HasType() const {
  for (const auto &i : data_component_types) {
    if (i.type_index == typeid(T).hash_code())
      return true;
  }
  return false;
}

}  // namespace evo_engine
