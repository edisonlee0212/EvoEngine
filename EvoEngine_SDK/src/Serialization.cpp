#include "Serialization.hpp"
#include "Console.hpp"
using namespace evo_engine;

std::string Serialization::GetSerializableTypeName(const size_t &type_index) {
  return GetInstance().serializable_names_.find(type_index)->second;
}

bool Serialization::RegisterDataComponentType(
    const std::string &type_name, const size_t &type_index,
    const std::function<std::shared_ptr<IDataComponent>(size_t &, size_t &)> &func) {
  auto &serialization = GetInstance();
  if (serialization.data_component_names_.find(type_index) != GetInstance().data_component_names_.end()) {
    EVOENGINE_ERROR("DataComponent already registered!")
    return false;
  }
  serialization.data_component_names_[type_index] = type_name;
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
  if (serialization_manger.serializable_names_.find(type_index) != serialization_manger.serializable_names_.end()) {
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
  auto &serialization_manager = GetInstance();
  return serialization_manager.private_component_cloners_.insert({type_name, clone_func}).second;
}
std::shared_ptr<ISerializable> Serialization::ProduceSerializable(const std::string &type_name, size_t &hash_code) {
  auto &serialization_manager = GetInstance();
  const auto it = serialization_manager.serializable_generators_.find(type_name);
  if (it != serialization_manager.serializable_generators_.end()) {
    auto ret_val = it->second(hash_code);
    ret_val->type_name_ = type_name;
    ret_val->handle_ = Handle();
    return ret_val;
  }
  EVOENGINE_ERROR("Serializable " + type_name + " is not registered!")
  return nullptr;
}
std::shared_ptr<ISerializable> Serialization::ProduceSerializable(const std::string &type_name, size_t &hash_code,
                                                                  const Handle &handle) {
  auto &serialization_manager = GetInstance();
  const auto it = serialization_manager.serializable_generators_.find(type_name);
  if (it != serialization_manager.serializable_generators_.end()) {
    auto ret_val = it->second(hash_code);
    ret_val->type_name_ = type_name;
    ret_val->handle_ = handle;
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

size_t Serialization::GetDataComponentTypeId(const std::string &type_index) {
  auto &serialization_manager = GetInstance();
  if (HasComponentDataType(type_index)) {
    return serialization_manager.data_component_ids_[type_index];
  }
  return 0;
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

size_t Serialization::GetSerializableTypeId(const std::string &type_index) {
  auto &serialization_manager = GetInstance();
  if (HasSerializableType(type_index)) {
    return serialization_manager.serializable_ids_[type_index];
  }
  return 0;
}
bool Serialization::HasSerializableType(const std::string &type_index) {
  return GetInstance().serializable_ids_.find(type_index) != GetInstance().serializable_ids_.end();
}
bool Serialization::HasComponentDataType(const std::string &type_index) {
  return GetInstance().data_component_ids_.find(type_index) != GetInstance().data_component_ids_.end();
}

void Serialization::ClonePrivateComponent(const std::shared_ptr<IPrivateComponent> &target,
                                          const std::shared_ptr<IPrivateComponent> &source) {
  const auto target_type_name = target->GetTypeName();
  const auto source_type_name = source->GetTypeName();
  assert(target_type_name == source_type_name);
  if (auto &serialization_manager = GetInstance(); serialization_manager.HasSerializableType(target_type_name)) {
    serialization_manager.private_component_cloners_[target_type_name](target, source);
  } else {
    EVOENGINE_ERROR("PrivateComponent " + target_type_name + "is not registered!")
  }
}
void Serialization::CloneSystem(const std::shared_ptr<ISystem> &target, const std::shared_ptr<ISystem> &source) {
  const auto target_type_name = target->GetTypeName();
  auto source_type_name = source->GetTypeName();
  assert(target_type_name == source_type_name);
  auto &serialization_manager = GetInstance();
  if (serialization_manager.HasSerializableType(target_type_name)) {
    serialization_manager.system_cloners_[target_type_name](target, source);
  } else {
    EVOENGINE_ERROR("System " + target_type_name + "is not registered!")
  }
}
bool Serialization::RegisterSystemType(
    const std::string &type_name,
    const std::function<void(std::shared_ptr<ISystem>, const std::shared_ptr<ISystem> &)> &clone_func) {
  auto &serialization_manager = GetInstance();
  return serialization_manager.system_cloners_.insert({type_name, clone_func}).second;
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
