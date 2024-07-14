#pragma once
#include <IDataComponent.hpp>
#include "IHandle.hpp"
#include "ISerializable.hpp"
namespace evo_engine {
#pragma region EntityManager
#pragma region Entity
class Scene;
struct DataComponentType final {
  std::string type_name;
  size_t type_index = 0;
  size_t type_size = 0;
  size_t type_offset = 0;
  DataComponentType() = default;
  DataComponentType(const std::string &name, const size_t &id, const size_t &size);
  bool operator==(const DataComponentType &other) const;
  bool operator!=(const DataComponentType &other) const;
};

struct EntityArchetype final {
 private:
  friend class Entities;
  friend class Serialization;
  friend class Scene;
  size_t index_ = 0;

 public:
  size_t GetIndex() const;
  [[nodiscard]] bool IsNull() const;
  [[nodiscard]] bool IsValid() const;
  [[nodiscard]] std::string GetName() const;
  void SetName(const std::string &name) const;
};
class IPrivateComponent;

struct Entity final {
 private:
  friend class Entities;
  friend class Scene;
  friend class EntityMetadata;
  friend class Serialization;
  unsigned index_ = 0;
  unsigned version_ = 0;

 public:
  [[nodiscard]] unsigned GetIndex() const;
  [[nodiscard]] unsigned GetVersion() const;
  bool operator==(const Entity &other) const;
  bool operator!=(const Entity &other) const;
  size_t operator()(Entity const &key) const;
};
#pragma region Storage

class EntityRef final {
  Entity value_ = Entity();
  Handle entity_handle_ = Handle(0);
  void Update();

 public:
  void Serialize(YAML::Emitter &out) const {
    out << YAML::Key << "entity_handle_" << YAML::Value << entity_handle_;
  }
  void Deserialize(const YAML::Node &in) {
    entity_handle_ = Handle(in["entity_handle_"].as<uint64_t>());
  }
  EntityRef() {
    entity_handle_ = Handle(0);
    value_ = Entity();
  }
  EntityRef(const Entity &other) {
    Set(other);
  }
  EntityRef &operator=(const Entity &other) {
    Set(other);
    return *this;
  }

  EntityRef &operator=(Entity &&other) noexcept {
    Set(other);
    return *this;
  }

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
  [[nodiscard]] Entity Get() {
    Update();
    return value_;
  }
  [[nodiscard]] Handle GetEntityHandle() const {
    return entity_handle_;
  }
  void Set(const Entity &target);
  void Clear();

  void Save(const std::string &name, YAML::Emitter &out) const {
    out << YAML::Key << name << YAML::Value << YAML::BeginMap;
    Serialize(out);
    out << YAML::EndMap;
  }
  void Load(const std::string &name, const YAML::Node &in) {
    if (in[name])
      Deserialize(in[name]);
  }
};

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

class ComponentDataChunk {
  std::vector<char> chunk_data_ = std::vector<char>(archetype_chunk_size);
public:
  template <typename T>
  T GetData(const size_t &offset);
  [[nodiscard]] void *RefData(const size_t &offset);
  [[nodiscard]] const void *PeekData(const size_t &offset) const;
  void SetData(const size_t &offset, const size_t &size, const void *data);
  void ClearData(const size_t &offset, const size_t &size);
};

struct DataComponentChunkArray {
  std::vector<Entity> entity_array{};
  std::vector<ComponentDataChunk> chunks{};
  DataComponentChunkArray &operator=(const DataComponentChunkArray &source);
};

struct EntityArchetypeInfo {
  std::string archetype_name = "New Entity Archetype";
  size_t entity_size = 0;
  size_t chunk_capacity = 0;
  std::vector<DataComponentType> data_component_types{};
  template <typename T>
  bool HasType() const;
  bool HasType(const size_t &type_index) const;
};

struct EntityQuery final {
 private:
  friend class Entities;
  friend class Scene;
  friend class Serialization;
  size_t index_ = 0;

 public:
  size_t GetIndex() const;
  bool operator==(const EntityQuery &other) const;
  bool operator!=(const EntityQuery &other) const;
  size_t operator()(const EntityQuery &key) const;
  [[nodiscard]] bool IsNull() const;
  [[nodiscard]] bool IsValid() const;
  template <typename T = IDataComponent, typename... Ts>
  void SetAllFilters(T arg, Ts... args);
  template <typename T = IDataComponent, typename... Ts>
  void SetAnyFilters(T arg, Ts... args);
  template <typename T = IDataComponent, typename... Ts>
  void SetNoneFilters(T arg, Ts... args);
};
struct DataComponentStorage {
  std::vector<DataComponentType> data_component_types;
  size_t entity_size = 0;
  size_t chunk_capacity = 0;
  size_t entity_count = 0;
  size_t entity_alive_count = 0;
  template <typename T>
  bool HasType() const;
  bool HasType(const size_t &type_id) const;
  DataComponentChunkArray chunk_array{};
  DataComponentStorage() = default;
  DataComponentStorage(const EntityArchetypeInfo &entity_archetype_info);
  DataComponentStorage &operator=(const DataComponentStorage &source);
};

struct EntityQueryInfo {
  size_t query_index = 0;
  std::vector<DataComponentType> all_data_component_types;
  std::vector<DataComponentType> any_data_component_types;
  std::vector<DataComponentType> none_data_component_types;
};
#pragma endregion
#pragma endregion
#pragma endregion
template <typename T>
bool EntityArchetypeInfo::HasType() const {
  for (const auto &i : data_component_types) {
    if (i.type_index == typeid(T).hash_code())
      return true;
  }
  return false;
}
template <typename T>
bool DataComponentStorage::HasType() const {
  for (const auto &i : data_component_types) {
    if (i.type_index == typeid(T).hash_code())
      return true;
  }
  return false;
}

}  // namespace evo_engine