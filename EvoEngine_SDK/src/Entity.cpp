#include "Entity.hpp"
#include "Entities.hpp"
#include "ISerializable.hpp"
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

uint32_t Entity::operator()(Entity const &key) const {
  return index_;
}

uint32_t Entity::GetIndex() const {
  return index_;
}
uint32_t Entity::GetVersion() const {
  return version_;
}

void *ComponentDataChunk::RefData(const size_t &offset) {
  return &chunk_data_[offset];
}

const void *ComponentDataChunk::PeekData(const size_t &offset) const {
  return &chunk_data_[offset];
}

void ComponentDataChunk::SetData(const size_t &offset, const size_t &size, const void *data) {
  memcpy(&chunk_data_[offset], data, size);
}

void ComponentDataChunk::ClearData(const size_t &offset, const size_t &size) {
  memset(&chunk_data_[offset], 0, size);
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

DataComponentStorage &DataComponentStorage::operator=(const DataComponentStorage &source) = default;

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
    const auto scene = Application::GetActiveScene();
    entity_handle_ = scene->GetEntityHandle(target);
    value_ = target;
  }
}
void EntityRef::Clear() {
  value_ = Entity();
  entity_handle_ = Handle(0);
}
void EntityRef::Update() {
  const auto scene = Application::GetActiveScene();
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
  chunks.resize(source.chunks.size());
  for (size_t i = 0; i < chunks.size(); i++)
    chunks[i] = source.chunks[i];
  return *this;
}
