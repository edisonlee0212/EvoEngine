#include "Serialization.hpp"

#include "Application.hpp"
#include "ApplicationContext.hpp"
#include "Console.hpp"
using namespace evo_engine;

Serialization &Serialization::GetInstance() {
  return ApplicationContext::Get().GetSerialization();
}

std::string Serialization::GetDataComponentTypeName(const size_t &type_id) {
  const auto &serialization = GetInstance();
  if (const auto search = serialization.data_component_names_.find(type_id);
      search != serialization.data_component_names_.end()) {
    return search->second;
  }
  throw std::invalid_argument("Data component type is unregistered!");
}

std::string Serialization::GetSerializableTypeName(const size_t &type_id) {
  const auto &serialization = GetInstance();
  if (const auto search = serialization.serializable_names_.find(type_id);
      search != serialization.serializable_names_.end()) {
    return search->second;
  }
  throw std::invalid_argument("Serializable type is unregistered!");
}

bool Serialization::RegisterDataComponentType(
    const std::string &type_name, const size_t &type_index, const size_t &type_size,
    const std::function<std::shared_ptr<IDataComponent>(size_t &, size_t &)> &func) {
  auto &serialization = GetInstance();
  if (serialization.data_component_names_.find(type_index) != GetInstance().data_component_names_.end()) {
    EVOENGINE_ERROR("DataComponent already registered!")
    return false;
  }
  serialization.data_component_names_[type_index] = type_name;
  serialization.data_component_sizes_[type_index] = type_size;
  serialization.data_component_ids_[type_name] = type_index;
  return serialization.data_component_generators_.insert({type_name, func}).second;
}

std::shared_ptr<IDataComponent> Serialization::ProduceDataComponent(const std::string &type_name, size_t &hash_code,
                                                                    size_t &size) {
  auto &factory = GetInstance();
  const auto it = factory.data_component_generators_.find(type_name);
  if (it != factory.data_component_generators_.end()) {
    return it->second(hash_code, size);
  }
  throw std::runtime_error("DataComponent " + type_name + "is not registered!");
}

bool Serialization::RegisterSerializableType(const std::string &type_name, const size_t &type_index,
                                             const std::function<std::shared_ptr<ISerializable>(size_t &)> &func) {
  auto &serialization_manger = GetInstance();
  if (serialization_manger.serializable_names_.find(type_index) != serialization_manger.serializable_names_.end() ||
      serialization_manger.serializable_ids_.find(type_name) != serialization_manger.serializable_ids_.end()) {
    EVOENGINE_ERROR(type_name + " already registered!")
    return false;
  }
  serialization_manger.serializable_names_[type_index] = type_name;
  serialization_manger.serializable_ids_[type_name] = type_index;
  return serialization_manger.serializable_generators_.insert({type_name, func}).second;
}
bool Serialization::RegisterPrivateComponentType(
    const std::string &type_name, const size_t &type_index,
    const std::function<void(std::shared_ptr<IPrivateComponent>, const std::shared_ptr<IPrivateComponent> &)>
        &clone_func) {
  auto &serialization = GetInstance();
  if (serialization.private_component_names_.find(type_index) != serialization.private_component_names_.end() ||
      serialization.private_component_ids_.find(type_name) != serialization.private_component_ids_.end()) {
    EVOENGINE_ERROR(type_name + " already registered!")
    return false;
  }
  serialization.private_component_names_[type_index] = type_name;
  serialization.private_component_ids_[type_name] = type_index;
  return serialization.private_component_cloners_.insert({type_name, clone_func}).second;
}
bool Serialization::RegisterSystemType(
    const std::string &type_name, const size_t &type_index,
    const std::function<void(std::shared_ptr<ISystem>, const std::shared_ptr<ISystem> &)> &clone_func) {
  auto &serialization = GetInstance();
  if (serialization.system_names_.find(type_index) != serialization.system_names_.end() ||
      serialization.system_ids_.find(type_name) != serialization.system_ids_.end()) {
    EVOENGINE_ERROR(type_name + " already registered!")
    return false;
  }
  serialization.system_names_[type_index] = type_name;
  serialization.system_ids_[type_name] = type_index;
  return serialization.system_cloners_.insert({type_name, clone_func}).second;
}

bool Serialization::RegisterAssetType(const std::string &type_name, const size_t &type_index,
                                      const std::vector<std::string> &extensions,
                                      const std::function<std::shared_ptr<ISerializable>(size_t &)> &func) {
  auto &serialization = GetInstance();
  serialization.asset_extensions_[type_name] = extensions;
  for (const auto &extension : extensions) {
    serialization.type_names_[extension] = type_name;
  }
  return RegisterSerializableType(type_name, type_index, func);
}

bool Serialization::UnregisterSerializableType(const std::string &type_name) {
  auto &serialization = GetInstance();
  const auto id_search = serialization.serializable_ids_.find(type_name);
  if (id_search == serialization.serializable_ids_.end()) {
    return false;
  }
  const auto type_id = id_search->second;
  serialization.serializable_generators_.erase(type_name);
  serialization.serializable_ids_.erase(id_search);
  serialization.serializable_names_.erase(type_id);
  serialization.serializable_type_owners_.erase(type_name);
  serialization.serializable_type_id_owners_.erase(type_id);
  return true;
}

bool Serialization::UnregisterPrivateComponentType(const std::string &type_name) {
  auto &serialization = GetInstance();
  const auto id_search = serialization.private_component_ids_.find(type_name);
  if (id_search == serialization.private_component_ids_.end()) {
    return false;
  }
  const auto type_id = id_search->second;
  serialization.private_component_cloners_.erase(type_name);
  serialization.private_component_ids_.erase(id_search);
  serialization.private_component_names_.erase(type_id);
  serialization.private_component_type_owners_.erase(type_name);
  serialization.private_component_type_id_owners_.erase(type_id);
  return true;
}

bool Serialization::UnregisterAssetType(const std::string &type_name) {
  auto &serialization = GetInstance();
  if (const auto extension_search = serialization.asset_extensions_.find(type_name);
      extension_search != serialization.asset_extensions_.end()) {
    for (const auto &extension : extension_search->second) {
      serialization.type_names_.erase(extension);
    }
    serialization.asset_extensions_.erase(extension_search);
  }
  return UnregisterSerializableType(type_name);
}

bool Serialization::UnregisterDataComponentType(const std::string &type_name) {
  auto &serialization = GetInstance();
  const auto id_search = serialization.data_component_ids_.find(type_name);
  if (id_search == serialization.data_component_ids_.end()) {
    return false;
  }
  const auto type_id = id_search->second;
  serialization.data_component_generators_.erase(type_name);
  serialization.data_component_ids_.erase(id_search);
  serialization.data_component_sizes_.erase(type_id);
  serialization.data_component_names_.erase(type_id);
  serialization.data_component_type_owners_.erase(type_name);
  serialization.data_component_type_id_owners_.erase(type_id);
  return true;
}

bool Serialization::UnregisterSystemType(const std::string &type_name) {
  auto &serialization = GetInstance();
  const auto id_search = serialization.system_ids_.find(type_name);
  if (id_search == serialization.system_ids_.end()) {
    return false;
  }
  const auto type_id = id_search->second;
  serialization.system_cloners_.erase(type_name);
  serialization.system_ids_.erase(id_search);
  serialization.system_names_.erase(type_id);
  serialization.system_type_owners_.erase(type_name);
  serialization.system_type_id_owners_.erase(type_id);
  return true;
}

void Serialization::SetSerializableTypeOwner(const std::string &type_name, const std::string &owner_name) {
  auto &serialization = GetInstance();
  if (const auto id_search = serialization.serializable_ids_.find(type_name);
      id_search != serialization.serializable_ids_.end()) {
    serialization.serializable_type_owners_[type_name] = owner_name;
    serialization.serializable_type_id_owners_[id_search->second] = owner_name;
  }
}

void Serialization::SetPrivateComponentTypeOwner(const std::string &type_name, const std::string &owner_name) {
  auto &serialization = GetInstance();
  if (const auto id_search = serialization.private_component_ids_.find(type_name);
      id_search != serialization.private_component_ids_.end()) {
    serialization.private_component_type_owners_[type_name] = owner_name;
    serialization.private_component_type_id_owners_[id_search->second] = owner_name;
  }
}

void Serialization::SetDataComponentTypeOwner(const std::string &type_name, const std::string &owner_name) {
  auto &serialization = GetInstance();
  if (const auto id_search = serialization.data_component_ids_.find(type_name);
      id_search != serialization.data_component_ids_.end()) {
    serialization.data_component_type_owners_[type_name] = owner_name;
    serialization.data_component_type_id_owners_[id_search->second] = owner_name;
  }
}

void Serialization::SetSystemTypeOwner(const std::string &type_name, const std::string &owner_name) {
  auto &serialization = GetInstance();
  if (const auto id_search = serialization.system_ids_.find(type_name); id_search != serialization.system_ids_.end()) {
    serialization.system_type_owners_[type_name] = owner_name;
    serialization.system_type_id_owners_[id_search->second] = owner_name;
  }
}

std::vector<size_t> Serialization::GetPackageOwnedPrivateComponentTypeIds(const std::string &owner_name) {
  const auto &serialization = GetInstance();
  std::vector<size_t> ret_val;
  for (const auto &[type_id, owner] : serialization.private_component_type_id_owners_) {
    if (owner == owner_name) {
      ret_val.emplace_back(type_id);
    }
  }
  return ret_val;
}

std::vector<size_t> Serialization::GetPackageOwnedDataComponentTypeIds(const std::string &owner_name) {
  const auto &serialization = GetInstance();
  std::vector<size_t> ret_val;
  for (const auto &[type_id, owner] : serialization.data_component_type_id_owners_) {
    if (owner == owner_name) {
      ret_val.emplace_back(type_id);
    }
  }
  return ret_val;
}

std::vector<size_t> Serialization::GetPackageOwnedSystemTypeIds(const std::string &owner_name) {
  const auto &serialization = GetInstance();
  std::vector<size_t> ret_val;
  for (const auto &[type_id, owner] : serialization.system_type_id_owners_) {
    if (owner == owner_name) {
      ret_val.emplace_back(type_id);
    }
  }
  return ret_val;
}

void Serialization::UnregisterPackageOwnedTypes(const std::string &owner_name) {
  auto &serialization = GetInstance();
  std::vector<std::string> data_component_names;
  for (const auto &[type_name, owner] : serialization.data_component_type_owners_) {
    if (owner == owner_name) {
      data_component_names.emplace_back(type_name);
    }
  }
  for (const auto &type_name : data_component_names) {
    UnregisterDataComponentType(type_name);
  }

  std::vector<std::string> private_component_names;
  for (const auto &[type_name, owner] : serialization.private_component_type_owners_) {
    if (owner == owner_name) {
      private_component_names.emplace_back(type_name);
    }
  }
  for (const auto &type_name : private_component_names) {
    UnregisterPrivateComponentType(type_name);
  }

  std::vector<std::string> system_names;
  for (const auto &[type_name, owner] : serialization.system_type_owners_) {
    if (owner == owner_name) {
      system_names.emplace_back(type_name);
    }
  }
  for (const auto &type_name : system_names) {
    UnregisterSystemType(type_name);
  }

  std::vector<std::string> serializable_names;
  for (const auto &[type_name, owner] : serialization.serializable_type_owners_) {
    if (owner == owner_name) {
      serializable_names.emplace_back(type_name);
    }
  }
  for (const auto &type_name : serializable_names) {
    if (HasAssetType(type_name)) {
      UnregisterAssetType(type_name);
    } else {
      UnregisterSerializableType(type_name);
    }
  }
}

std::shared_ptr<ISerializable> Serialization::ProduceSerializable(const std::string &type_name, size_t &hash_code) {
  auto &serialization = GetInstance();
  if (const auto it = serialization.serializable_generators_.find(type_name);
      it != serialization.serializable_generators_.end()) {
    auto ret_val = it->second(hash_code);
    ret_val->type_name_ = type_name;
    ret_val->handle_ = Handle();
    ret_val->application_ = &ApplicationContext::Get();
    return ret_val;
  }
  EVOENGINE_ERROR("Serializable " + type_name + " is not registered!")
  return nullptr;
}

std::shared_ptr<ISerializable> Serialization::ProduceSerializable(const std::string &type_name) {
  auto &serialization = GetInstance();
  if (const auto it = serialization.serializable_generators_.find(type_name);
      it != serialization.serializable_generators_.end()) {
    size_t temp;
    auto ret_val = it->second(temp);
    ret_val->type_name_ = type_name;
    ret_val->handle_ = Handle();
    ret_val->application_ = &ApplicationContext::Get();
    return ret_val;
  }
  EVOENGINE_ERROR("Serializable " + type_name + " is not registered!")
  return nullptr;
}

std::shared_ptr<ISerializable> Serialization::ProduceSerializable(const std::string &type_name, size_t &hash_code,
                                                                  const Handle &handle) {
  auto &serialization = GetInstance();
  const auto it = serialization.serializable_generators_.find(type_name);
  if (it != serialization.serializable_generators_.end()) {
    auto ret_val = it->second(hash_code);
    ret_val->type_name_ = type_name;
    ret_val->handle_ = handle;
    ret_val->application_ = &ApplicationContext::Get();
    return ret_val;
  }
  EVOENGINE_ERROR("PrivateComponent " + type_name + " is not registered!")
  return nullptr;
}
YAML::Emitter &evo_engine::operator<<(YAML::Emitter &out, const glm::vec2 &v) {
  out << YAML::Flow;
  out << YAML::BeginSeq << v.x << v.y << YAML::EndSeq;
  return out;
}

YAML::Emitter &evo_engine::operator<<(YAML::Emitter &out, const glm::vec3 &v) {
  out << YAML::Flow;
  out << YAML::BeginSeq << v.x << v.y << v.z << YAML::EndSeq;
  return out;
}

YAML::Emitter &evo_engine::operator<<(YAML::Emitter &out, const glm::vec4 &v) {
  out << YAML::Flow;
  out << YAML::BeginSeq << v.x << v.y << v.z << v.w << YAML::EndSeq;
  return out;
}

YAML::Emitter &evo_engine::operator<<(YAML::Emitter &out, const glm::quat &v) {
  out << YAML::Flow;
  out << YAML::BeginSeq << v.x << v.y << v.z << v.w << YAML::EndSeq;
  return out;
}

YAML::Emitter &evo_engine::operator<<(YAML::Emitter &out, const glm::mat4 &v) {
  out << YAML::Flow;
  out << YAML::BeginSeq << v[0] << v[1] << v[2] << v[3] << YAML::EndSeq;
  return out;
}
YAML::Emitter &evo_engine::operator<<(YAML::Emitter &out, const glm::dvec2 &v) {
  out << YAML::Flow;
  out << YAML::BeginSeq << v.x << v.y << YAML::EndSeq;
  return out;
}

YAML::Emitter &evo_engine::operator<<(YAML::Emitter &out, const glm::dvec3 &v) {
  out << YAML::Flow;
  out << YAML::BeginSeq << v.x << v.y << v.z << YAML::EndSeq;
  return out;
}

YAML::Emitter &evo_engine::operator<<(YAML::Emitter &out, const glm::dvec4 &v) {
  out << YAML::Flow;
  out << YAML::BeginSeq << v.x << v.y << v.z << v.w << YAML::EndSeq;
  return out;
}

YAML::Emitter &evo_engine::operator<<(YAML::Emitter &out, const glm::ivec2 &v) {
  out << YAML::Flow;
  out << YAML::BeginSeq << v.x << v.y << YAML::EndSeq;
  return out;
}

YAML::Emitter &evo_engine::operator<<(YAML::Emitter &out, const glm::ivec3 &v) {
  out << YAML::Flow;
  out << YAML::BeginSeq << v.x << v.y << v.z << YAML::EndSeq;
  return out;
}

YAML::Emitter &evo_engine::operator<<(YAML::Emitter &out, const glm::ivec4 &v) {
  out << YAML::Flow;
  out << YAML::BeginSeq << v.x << v.y << v.z << v.w << YAML::EndSeq;
  return out;
}
YAML::Emitter &evo_engine::operator<<(YAML::Emitter &out, const glm::uvec2 &v) {
  out << YAML::Flow;
  out << YAML::BeginSeq << v.x << v.y << YAML::EndSeq;
  return out;
}

YAML::Emitter &evo_engine::operator<<(YAML::Emitter &out, const glm::uvec3 &v) {
  out << YAML::Flow;
  out << YAML::BeginSeq << v.x << v.y << v.z << YAML::EndSeq;
  return out;
}

YAML::Emitter &evo_engine::operator<<(YAML::Emitter &out, const glm::uvec4 &v) {
  out << YAML::Flow;
  out << YAML::BeginSeq << v.x << v.y << v.z << v.w << YAML::EndSeq;
  return out;
}

YAML::Emitter &evo_engine::operator<<(YAML::Emitter &out, const glm::u16vec4 &v) {
  out << YAML::Flow;
  out << YAML::BeginSeq << v.x << v.y << v.z << v.w << YAML::EndSeq;
  return out;
}

size_t Serialization::GetDataComponentTypeId(const std::string &type_name) {
  const auto &serialization = GetInstance();
  return serialization.data_component_ids_.at(type_name);
}

size_t Serialization::GetDataComponentTypeSize(const size_t &type_id) {
  const auto &serialization = GetInstance();
  return serialization.data_component_sizes_.at(type_id);
}

void Serialization::SaveAssetList(const std::string &name, const std::vector<AssetRef> &target, YAML::Emitter &out) {
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

void Serialization::LoadAssetList(const std::string &name, std::vector<AssetRef> &target, const YAML::Node &in) {
  if (in[name]) {
    target.clear();
    for (const auto &i : in[name]) {
      AssetRef instance;
      instance.Deserialize(i);
      target.push_back(instance);
    }
  }
}
void Serialization::OnDestroy() {
  auto &serialization = GetInstance();
  serialization = {};
}

size_t Serialization::GetSerializableTypeId(const std::string &type_name) {
  const auto &serialization = GetInstance();
  return serialization.serializable_ids_.at(type_name);
}
bool Serialization::HasSerializableType(const std::string &type_name) {
  const auto &serialization = GetInstance();
  return serialization.serializable_ids_.find(type_name) != serialization.serializable_ids_.end();
}

bool Serialization::HasSerializableType(const size_t &type_id) {
  const auto &serialization = GetInstance();
  return serialization.serializable_names_.find(type_id) != serialization.serializable_names_.end();
}

bool Serialization::HasComponentDataType(const std::string &type_name) {
  const auto &serialization = GetInstance();
  return serialization.data_component_ids_.find(type_name) != serialization.data_component_ids_.end();
}

bool Serialization::HasComponentDataType(const size_t &type_id) {
  const auto &serialization = GetInstance();
  return serialization.data_component_names_.find(type_id) != serialization.data_component_names_.end();
}

bool Serialization::HasAssetType(const std::string &type_name) {
  const auto &serialization = GetInstance();
  return serialization.asset_extensions_.find(type_name) != serialization.asset_extensions_.end();
}

const std::vector<std::string> &Serialization::PeekAssetExtensions(const std::string &type_name) {
  const auto &serialization = GetInstance();
  if (const auto search = serialization.asset_extensions_.find(type_name);
      search != serialization.asset_extensions_.end()) {
    return search->second;
  }
  throw std::runtime_error("Asset type not registered!");
}

std::string Serialization::GetAssetTypeName(const std::string &extension) {
  const auto &serialization = GetInstance();
  if (const auto search = serialization.type_names_.find(extension); search != serialization.type_names_.end()) {
    return search->second;
  }
  return "Binary";
}

void Serialization::ClonePrivateComponent(const std::shared_ptr<IPrivateComponent> &target,
                                          const std::shared_ptr<IPrivateComponent> &source) {
  const auto target_type_name = target->GetTypeName();
  const auto source_type_name = source->GetTypeName();
  assert(target_type_name == source_type_name);
  if (auto &serialization = GetInstance(); serialization.HasSerializableType(target_type_name)) {
    serialization.private_component_cloners_.at(target_type_name)(target, source);
  } else {
    EVOENGINE_ERROR("PrivateComponent " + target_type_name + "is not registered!")
  }
}
void Serialization::CloneSystem(const std::shared_ptr<ISystem> &target, const std::shared_ptr<ISystem> &source) {
  const auto target_type_name = target->GetTypeName();
  auto source_type_name = source->GetTypeName();
  assert(target_type_name == source_type_name);
  const auto &serialization = GetInstance();
  if (serialization.HasSerializableType(target_type_name)) {
    serialization.system_cloners_.at(target_type_name)(target, source);
  } else {
    EVOENGINE_ERROR("System " + target_type_name + "is not registered!")
  }
}

YAML::Emitter &evo_engine::operator<<(YAML::Emitter &out, const glm::u8vec4 &v) {
  out << YAML::Flow;
  out << YAML::BeginSeq << v.x << v.y << v.z << v.w << YAML::EndSeq;
  return out;
}

YAML::Emitter &evo_engine::operator<<(YAML::Emitter &out, const glm::i8vec4 &v) {
  out << YAML::Flow;
  out << YAML::BeginSeq << v.x << v.y << v.z << v.w << YAML::EndSeq;
  return out;
}

YAML::Emitter &evo_engine::operator<<(YAML::Emitter &out, const glm::i16vec4 &v) {
  out << YAML::Flow;
  out << YAML::BeginSeq << v.x << v.y << v.z << v.w << YAML::EndSeq;
  return out;
}
