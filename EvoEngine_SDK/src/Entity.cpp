#include "Entity.hpp"
#include "Entities.hpp"
#include "EntityMetadata.hpp"
#include "IPrivateComponent.hpp"
#include "ISerializable.hpp"
// #include "ProjectManager.hpp"
#include "Application.hpp"
#include "Scene.hpp"
using namespace evo_engine;

DataComponentType::DataComponentType(const std::string &name, const size_t &id, const size_t &size) {
  type_name = name;
  type_index = id;
  type_size = size;
  type_offset = 0;
}

bool DataComponentType::operator==(const DataComponentType &other) const {
  return (other.type_index == type_index) && (other.type_size == type_size);
}

bool DataComponentType::operator!=(const DataComponentType &other) const {
  return (other.type_index != type_index) || (other.type_size != type_size);
}

bool Entity::operator==(const Entity &other) const {
  return (other.index_ == index_) && (other.version_ == version_);
}

bool Entity::operator!=(const Entity &other) const {
  return (other.index_ != index_) || (other.version_ != version_);
}

size_t Entity::operator()(Entity const &key) const {
  return static_cast<size_t>(index_);
}

unsigned Entity::GetIndex() const {
  return index_;
}
unsigned Entity::GetVersion() const {
  return version_;
}

IDataComponent *ComponentDataChunk::GetDataPointer(const size_t &offset) const {
  return reinterpret_cast<IDataComponent *>(static_cast<char *>(chunk_data) + offset);
}

void ComponentDataChunk::SetData(const size_t &offset, const size_t &size, IDataComponent *data) const {
  memcpy(static_cast<void *>(static_cast<char *>(chunk_data) + offset), data, size);
}

void ComponentDataChunk::ClearData(const size_t &offset, const size_t &size) const {
  memset(static_cast<void *>(static_cast<char *>(chunk_data) + offset), 0, size);
}

ComponentDataChunk &ComponentDataChunk::operator=(const ComponentDataChunk &source) {
  chunk_data = static_cast<void *>(calloc(1, Entities::GetArchetypeChunkSize()));
  memcpy(chunk_data, source.chunk_data, Entities::GetArchetypeChunkSize());
  return *this;
}

bool EntityArchetype::IsNull() const {
  return index_ == 0;
}

bool EntityArchetype::IsValid() const {
  return index_ != 0 && Entities::GetInstance().entity_archetype_infos_.size() > index_;
}

std::string EntityArchetype::GetName() const {
  return Entities::GetEntityArchetypeName(*this);
}
void EntityArchetype::SetName(const std::string &name) const {
  Entities::SetEntityArchetypeName(*this, name);
}
size_t EntityArchetype::GetIndex() const {
  return index_;
}

bool EntityArchetypeInfo::HasType(const size_t &type_index) const {
  for (const auto &type : data_component_types) {
    if (type_index == type.type_index)
      return true;
  }
  return false;
}

bool EntityQuery::operator==(const EntityQuery &other) const {
  return other.index_ == index_;
}

bool EntityQuery::operator!=(const EntityQuery &other) const {
  return other.index_ != index_;
}

size_t EntityQuery::operator()(const EntityQuery &key) const {
  return index_;
}

bool EntityQuery::IsNull() const {
  return index_ == 0;
}
size_t EntityQuery::GetIndex() const {
  return index_;
}
bool EntityQuery::IsValid() const {
  return index_ != 0 && Entities::GetInstance().entity_query_infos_.size() > index_;
}

DataComponentStorage::DataComponentStorage(const EntityArchetypeInfo &entity_archetype_info) {
  data_component_types = entity_archetype_info.data_component_types;
  entity_size = entity_archetype_info.entity_size;
  chunk_capacity = entity_archetype_info.chunk_capacity;
}

DataComponentStorage &DataComponentStorage::operator=(const DataComponentStorage &source) {
  data_component_types = source.data_component_types;
  entity_size = source.entity_size;
  chunk_capacity = source.chunk_capacity;
  entity_count = source.entity_count;
  entity_alive_count = source.entity_alive_count;
  chunk_array = source.chunk_array;
  return *this;
}

bool DataComponentStorage::HasType(const size_t &type_id) const {
  for (const auto &type : data_component_types) {
    if (type_id == type.type_index)
      return true;
  }
  return false;
}

void EntityRef::Set(const Entity &target) {
  if (target.GetIndex() == 0) {
    Clear();
  } else {
    auto scene = Application::GetActiveScene();
    entity_handle_ = scene->GetEntityHandle(target);
    value_ = target;
  }
}
void EntityRef::Clear() {
  value_ = Entity();
  entity_handle_ = Handle(0);
}
void EntityRef::Update() {
  auto scene = Application::GetActiveScene();
  if (entity_handle_.GetValue() == 0) {
    Clear();
    return;
  } 
  if (value_.GetIndex() == 0) {
    if (!scene)
      Clear();
    else {
      value_ = scene->GetEntity(entity_handle_);
      if (value_.GetIndex() == 0) {
        Clear();
      }
    }
  }
  if (!scene->IsEntityValid(value_)) {
    Clear();
  }
}

DataComponentChunkArray &DataComponentChunkArray::operator=(const DataComponentChunkArray &source) {
  entity_array = source.entity_array;
  chunk_array.resize(source.chunk_array.size());
  for (int i = 0; i < chunk_array.size(); i++)
    chunk_array[i] = source.chunk_array[i];
  return *this;
}
