#pragma once
#include "Console.hpp"
#include "IAsset.hpp"
#include "IDataComponent.hpp"
#include "IPrivateComponent.hpp"
#include "ISerializable.hpp"
#include "ISingleton.hpp"
#include "ISystem.hpp"
namespace YAML {
class Node;
class Emitter;
template <>
struct convert<glm::vec2> {
  static Node encode(const glm::vec2& rhs) {
    Node node;
    node.push_back(rhs.x);
    node.push_back(rhs.y);
    return node;
  }

  static bool decode(const Node& node, glm::vec2& rhs) {
    if (!node.IsSequence() || node.size() != 2) {
      return false;
    }

    rhs.x = node[0].as<float>();
    rhs.y = node[1].as<float>();
    return true;
  }
};

template <>
struct convert<glm::vec3> {
  static Node encode(const glm::vec3& rhs) {
    Node node;
    node.push_back(rhs.x);
    node.push_back(rhs.y);
    node.push_back(rhs.z);
    return node;
  }

  static bool decode(const Node& node, glm::vec3& rhs) {
    if (!node.IsSequence() || node.size() != 3) {
      return false;
    }

    rhs.x = node[0].as<float>();
    rhs.y = node[1].as<float>();
    rhs.z = node[2].as<float>();
    return true;
  }
};
template <>
struct convert<glm::vec4> {
  static Node encode(const glm::vec4& rhs) {
    Node node;
    node.push_back(rhs.x);
    node.push_back(rhs.y);
    node.push_back(rhs.z);
    node.push_back(rhs.w);
    return node;
  }

  static bool decode(const Node& node, glm::vec4& rhs) {
    if (!node.IsSequence() || node.size() != 4) {
      return false;
    }

    rhs.x = node[0].as<float>();
    rhs.y = node[1].as<float>();
    rhs.z = node[2].as<float>();
    rhs.w = node[3].as<float>();
    return true;
  }
};
template <>
struct convert<glm::quat> {
  static Node encode(const glm::quat& rhs) {
    Node node;
    node.push_back(rhs.x);
    node.push_back(rhs.y);
    node.push_back(rhs.z);
    node.push_back(rhs.w);
    return node;
  }

  static bool decode(const Node& node, glm::quat& rhs) {
    if (!node.IsSequence() || node.size() != 4) {
      return false;
    }

    rhs.x = node[0].as<float>();
    rhs.y = node[1].as<float>();
    rhs.z = node[2].as<float>();
    rhs.w = node[3].as<float>();
    return true;
  }
};
template <>
struct convert<glm::mat4> {
  static Node encode(const glm::mat4& rhs) {
    Node node;
    node.push_back(rhs[0]);
    node.push_back(rhs[1]);
    node.push_back(rhs[2]);
    node.push_back(rhs[3]);
    return node;
  }

  static bool decode(const Node& node, glm::mat4& rhs) {
    if (!node.IsSequence() || node.size() != 4) {
      return false;
    }

    rhs[0] = node[0].as<glm::vec4>();
    rhs[1] = node[1].as<glm::vec4>();
    rhs[2] = node[2].as<glm::vec4>();
    rhs[3] = node[3].as<glm::vec4>();
    return true;
  }
};
template <>
struct convert<glm::dvec2> {
  static Node encode(const glm::dvec2& rhs) {
    Node node;
    node.push_back(rhs.x);
    node.push_back(rhs.y);
    return node;
  }

  static bool decode(const Node& node, glm::dvec2& rhs) {
    if (!node.IsSequence() || node.size() != 2) {
      return false;
    }

    rhs.x = node[0].as<double>();
    rhs.y = node[1].as<double>();
    return true;
  }
};

template <>
struct convert<glm::dvec3> {
  static Node encode(const glm::dvec3& rhs) {
    Node node;
    node.push_back(rhs.x);
    node.push_back(rhs.y);
    node.push_back(rhs.z);
    return node;
  }

  static bool decode(const Node& node, glm::dvec3& rhs) {
    if (!node.IsSequence() || node.size() != 3) {
      return false;
    }

    rhs.x = node[0].as<double>();
    rhs.y = node[1].as<double>();
    rhs.z = node[2].as<double>();
    return true;
  }
};
template <>
struct convert<glm::dvec4> {
  static Node encode(const glm::dvec4& rhs) {
    Node node;
    node.push_back(rhs.x);
    node.push_back(rhs.y);
    node.push_back(rhs.z);
    node.push_back(rhs.w);
    return node;
  }

  static bool decode(const Node& node, glm::dvec4& rhs) {
    if (!node.IsSequence() || node.size() != 4) {
      return false;
    }

    rhs.x = node[0].as<double>();
    rhs.y = node[1].as<double>();
    rhs.z = node[2].as<double>();
    rhs.w = node[3].as<double>();
    return true;
  }
};

template <>
struct convert<glm::ivec2> {
  static Node encode(const glm::ivec2& rhs) {
    Node node;
    node.push_back(rhs.x);
    node.push_back(rhs.y);
    return node;
  }

  static bool decode(const Node& node, glm::ivec2& rhs) {
    if (!node.IsSequence() || node.size() != 2) {
      return false;
    }

    rhs.x = node[0].as<int>();
    rhs.y = node[1].as<int>();
    return true;
  }
};

template <>
struct convert<glm::ivec3> {
  static Node encode(const glm::ivec3& rhs) {
    Node node;
    node.push_back(rhs.x);
    node.push_back(rhs.y);
    node.push_back(rhs.z);
    return node;
  }

  static bool decode(const Node& node, glm::ivec3& rhs) {
    if (!node.IsSequence() || node.size() != 3) {
      return false;
    }

    rhs.x = node[0].as<int>();
    rhs.y = node[1].as<int>();
    rhs.z = node[2].as<int>();
    return true;
  }
};
template <>
struct convert<glm::ivec4> {
  static Node encode(const glm::ivec4& rhs) {
    Node node;
    node.push_back(rhs.x);
    node.push_back(rhs.y);
    node.push_back(rhs.z);
    node.push_back(rhs.w);
    return node;
  }

  static bool decode(const Node& node, glm::ivec4& rhs) {
    if (!node.IsSequence() || node.size() != 4) {
      return false;
    }

    rhs.x = node[0].as<int>();
    rhs.y = node[1].as<int>();
    rhs.z = node[2].as<int>();
    rhs.w = node[3].as<int>();
    return true;
  }
};
template <>
struct convert<glm::uvec2> {
  static Node encode(const glm::uvec2& rhs) {
    Node node;
    node.push_back(rhs.x);
    node.push_back(rhs.y);
    return node;
  }

  static bool decode(const Node& node, glm::uvec2& rhs) {
    if (!node.IsSequence() || node.size() != 2) {
      return false;
    }

    rhs.x = node[0].as<unsigned>();
    rhs.y = node[1].as<unsigned>();
    return true;
  }
};

template <>
struct convert<glm::uvec3> {
  static Node encode(const glm::uvec3& rhs) {
    Node node;
    node.push_back(rhs.x);
    node.push_back(rhs.y);
    node.push_back(rhs.z);
    return node;
  }

  static bool decode(const Node& node, glm::uvec3& rhs) {
    if (!node.IsSequence() || node.size() != 3) {
      return false;
    }

    rhs.x = node[0].as<unsigned>();
    rhs.y = node[1].as<unsigned>();
    rhs.z = node[2].as<unsigned>();
    return true;
  }
};
template <>
struct convert<glm::uvec4> {
  static Node encode(const glm::uvec4& rhs) {
    Node node;
    node.push_back(rhs.x);
    node.push_back(rhs.y);
    node.push_back(rhs.z);
    node.push_back(rhs.w);
    return node;
  }

  static bool decode(const Node& node, glm::uvec4& rhs) {
    if (!node.IsSequence() || node.size() != 4) {
      return false;
    }

    rhs.x = node[0].as<unsigned>();
    rhs.y = node[1].as<unsigned>();
    rhs.z = node[2].as<unsigned>();
    rhs.w = node[3].as<unsigned>();
    return true;
  }
};

template <>
struct convert<glm::i8vec4> {
  static Node encode(const glm::i8vec4& rhs) {
    Node node;
    node.push_back(rhs.x);
    node.push_back(rhs.y);
    node.push_back(rhs.z);
    node.push_back(rhs.w);
    return node;
  }

  static bool decode(const Node& node, glm::i8vec4& rhs) {
    if (!node.IsSequence() || node.size() != 4) {
      return false;
    }

    rhs.x = node[0].as<signed char>();
    rhs.y = node[1].as<signed char>();
    rhs.z = node[2].as<signed char>();
    rhs.w = node[3].as<signed char>();
    return true;
  }
};
template <>
struct convert<glm::u8vec4> {
  static Node encode(const glm::u8vec4& rhs) {
    Node node;
    node.push_back(rhs.x);
    node.push_back(rhs.y);
    node.push_back(rhs.z);
    node.push_back(rhs.w);
    return node;
  }

  static bool decode(const Node& node, glm::u8vec4& rhs) {
    if (!node.IsSequence() || node.size() != 4) {
      return false;
    }

    rhs.x = node[0].as<unsigned char>();
    rhs.y = node[1].as<unsigned char>();
    rhs.z = node[2].as<unsigned char>();
    rhs.w = node[3].as<unsigned char>();
    return true;
  }
};

template <>
struct convert<glm::i16vec4> {
  static Node encode(const glm::i16vec4& rhs) {
    Node node;
    node.push_back(rhs.x);
    node.push_back(rhs.y);
    node.push_back(rhs.z);
    node.push_back(rhs.w);
    return node;
  }

  static bool decode(const Node& node, glm::i16vec4& rhs) {
    if (!node.IsSequence() || node.size() != 4) {
      return false;
    }

    rhs.x = node[0].as<signed short>();
    rhs.y = node[1].as<signed short>();
    rhs.z = node[2].as<signed short>();
    rhs.w = node[3].as<signed short>();
    return true;
  }
};
template <>
struct convert<glm::u16vec4> {
  static Node encode(const glm::u16vec4& rhs) {
    Node node;
    node.push_back(rhs.x);
    node.push_back(rhs.y);
    node.push_back(rhs.z);
    node.push_back(rhs.w);
    return node;
  }

  static bool decode(const Node& node, glm::u16vec4& rhs) {
    if (!node.IsSequence() || node.size() != 4) {
      return false;
    }

    rhs.x = node[0].as<unsigned short>();
    rhs.y = node[1].as<unsigned short>();
    rhs.z = node[2].as<unsigned short>();
    rhs.w = node[3].as<unsigned short>();
    return true;
  }
};
}  // namespace YAML
#define EXPORT_PARAM(x, y) (x) << "{" << (y) << "}"
#define IMPORT_PARAM(x, y, temp) (x) >> (temp) >> (y) >> (temp)
namespace evo_engine {
#pragma region Component Factory
class Serialization final {
  EVOENGINE_SINGLETON_INSTANCE(Serialization)
  friend class ISerializable;
  friend class ProjectManager;
  friend class ClassRegistry;
  friend class EditorLayer;
  friend class Scene;
  std::unordered_map<std::string, std::function<std::shared_ptr<IDataComponent>(size_t&, size_t&)>>
      data_component_generators_;
  std::unordered_map<std::string, std::function<std::shared_ptr<ISerializable>(size_t&)>> serializable_generators_;
  std::unordered_map<std::string,
                     std::function<void(std::shared_ptr<IPrivateComponent>, const std::shared_ptr<IPrivateComponent>&)>>
      private_component_cloners_;
  std::unordered_map<std::string, std::function<void(std::shared_ptr<ISystem>, const std::shared_ptr<ISystem>&)>>
      system_cloners_;

  std::unordered_map<std::string, size_t> data_component_ids_;
  std::unordered_map<size_t, size_t> data_component_sizes_;
  std::unordered_map<size_t, std::string> data_component_names_;

  std::unordered_map<std::string, size_t> private_component_ids_;
  std::unordered_map<size_t, std::string> private_component_names_;

  std::unordered_map<std::string, size_t> system_ids_;
  std::unordered_map<size_t, std::string> system_names_;

  std::unordered_map<std::string, size_t> serializable_ids_;
  std::unordered_map<size_t, std::string> serializable_names_;

  std::unordered_map<std::string, std::vector<std::string>> asset_extensions_;
  std::map<std::string, std::string> type_names_;

  template <typename T = IDataComponent>
  static bool RegisterDataComponentType(const std::string& name);
  template <typename T = ISerializable>
  static bool RegisterSerializableType(const std::string& name);
  template <typename T = IPrivateComponent>
  static bool RegisterPrivateComponentType(const std::string& name);
  template <typename T = ISystem>
  static bool RegisterSystemType(const std::string& name);

  template <typename T = IAsset>
  static bool RegisterAssetType(const std::string& name, const std::vector<std::string>& extensions);

  static bool RegisterDataComponentType(const std::string& type_name, const size_t& type_index, const size_t& type_size,
                                        const std::function<std::shared_ptr<IDataComponent>(size_t&, size_t&)>& func);
  static bool RegisterSerializableType(const std::string& type_name, const size_t& type_index,
                                       const std::function<std::shared_ptr<ISerializable>(size_t&)>& func);

  static bool RegisterPrivateComponentType(
      const std::string& type_name, const size_t& type_index,
      const std::function<void(std::shared_ptr<IPrivateComponent>, const std::shared_ptr<IPrivateComponent>&)>&
          clone_func);
  static bool RegisterSystemType(
      const std::string& type_name, const size_t& type_index,
      const std::function<void(std::shared_ptr<ISystem>, const std::shared_ptr<ISystem>&)>& clone_func);
  static bool RegisterAssetType(const std::string& type_name, const size_t& type_index,
                                const std::vector<std::string>& extensions,
                                const std::function<std::shared_ptr<ISerializable>(size_t&)>& func);

 public:
  static std::shared_ptr<IDataComponent> ProduceDataComponent(const std::string& type_name, size_t& hash_code,
                                                              size_t& size);
  static void ClonePrivateComponent(const std::shared_ptr<IPrivateComponent>& target,
                                    const std::shared_ptr<IPrivateComponent>& source);
  static void CloneSystem(const std::shared_ptr<ISystem>& target, const std::shared_ptr<ISystem>& source);
  static std::shared_ptr<ISerializable> ProduceSerializable(const std::string& type_name, size_t& hash_code);
  static std::shared_ptr<ISerializable> ProduceSerializable(const std::string& type_name);
  static auto ProduceSerializable(const std::string& type_name, size_t& hash_code, const Handle& handle)
      -> std::shared_ptr<ISerializable>;
  template <typename T = ISerializable>
  static std::shared_ptr<T> ProduceSerializable();
  template <typename T = IDataComponent>
  static std::string GetDataComponentTypeName();
  template <typename T = ISerializable>
  static std::string GetSerializableTypeName();
  static std::string GetSerializableTypeName(const size_t& type_id);
  static bool HasSerializableType(const std::string& type_name);
  static bool HasSerializableType(const size_t& type_id);

  static bool HasComponentDataType(const std::string& type_name);
  static bool HasComponentDataType(const size_t& type_id);

  static bool HasAssetType(const std::string& type_name);
  static const std::vector<std::string>& PeekAssetExtensions(const std::string& type_name);
  static std::string GetAssetTypeName(const std::string& extension);

  static size_t GetSerializableTypeId(const std::string& type_name);
  static size_t GetDataComponentTypeId(const std::string& type_name);
  static size_t GetDataComponentTypeSize(const size_t& type_id);
  static void SaveAssetList(const std::string& name, const std::vector<AssetRef>& target, YAML::Emitter& out);
  static void LoadAssetList(const std::string& name, std::vector<AssetRef>& target, const YAML::Node& in);

  template <typename T>
  static void SerializeVector(const std::string& name, const std::vector<T>& target, YAML::Emitter& out);
  template <typename T>
  static void DeserializeVector(const std::string& name, std::vector<T>& target, const YAML::Node& in);
};

template <typename T>
std::string Serialization::GetDataComponentTypeName() {
  const auto& serialization = GetInstance();
  return serialization.data_component_names_.find(typeid(T).hash_code())->second;
}
template <typename T>
std::string Serialization::GetSerializableTypeName() {
  const auto& serialization = GetInstance();
  return serialization.serializable_names_.find(typeid(T).hash_code())->second;
}

template <typename T>
void Serialization::SerializeVector(const std::string& name, const std::vector<T>& target, YAML::Emitter& out) {
  if (!target.empty()) {
    out << YAML::Key << name << YAML::Value
        << YAML::Binary(reinterpret_cast<const unsigned char*>(target.data()), target.size() * sizeof(T));
  }
}

template <typename T>
void Serialization::DeserializeVector(const std::string& name, std::vector<T>& target, const YAML::Node& in) {
  if (in[name]) {
    const auto& data = in[name].as<YAML::Binary>();
    target.resize(data.size() / sizeof(T));
    std::memcpy(target.data(), data.data(), data.size());
  }
}

template <typename T>
bool Serialization::RegisterDataComponentType(const std::string& name) {
  return RegisterDataComponentType(name, typeid(T).hash_code(), sizeof(T), [](size_t& hash_code, size_t& size) {
    hash_code = typeid(T).hash_code();
    size = sizeof(T);
    return std::move(std::dynamic_pointer_cast<IDataComponent>(std::make_shared<T>()));
  });
}

template <typename T>
bool Serialization::RegisterSerializableType(const std::string& name) {
  return RegisterSerializableType(name, typeid(T).hash_code(), [](size_t& hash_code) {
    hash_code = typeid(T).hash_code();
    auto ptr = std::static_pointer_cast<ISerializable>(std::make_shared<T>());
    return ptr;
  });
}
template <typename T>
bool Serialization::RegisterPrivateComponentType(const std::string& name) {
  return RegisterPrivateComponentType(
      name, typeid(T).hash_code(),
      [](const std::shared_ptr<IPrivateComponent>& target, const std::shared_ptr<IPrivateComponent>& source) {
        target->handle_ = source->handle_;
        target->enabled_ = source->enabled_;
        target->owner_ = source->owner_;
        *std::dynamic_pointer_cast<T>(target) = *std::dynamic_pointer_cast<T>(source);
        target->started_ = false;
        target->PostCloneAction(source);
      });
}
template <typename T>
bool Serialization::RegisterSystemType(const std::string& name) {
  return RegisterSystemType(name, typeid(T).hash_code(),
                            [](const std::shared_ptr<ISystem>& target, const std::shared_ptr<ISystem>& source) {
                              target->handle_ = source->handle_;
                              target->rank_ = source->rank_;
                              target->enabled_ = source->enabled_;
                              *std::dynamic_pointer_cast<T>(target) = *std::dynamic_pointer_cast<T>(source);
                              target->started_ = false;
                              target->PostCloneAction(source);
                            });
}

template <typename T>
bool Serialization::RegisterAssetType(const std::string& name, const std::vector<std::string>& extensions) {
  return RegisterAssetType(name, typeid(T).hash_code(), extensions, [](size_t& hash_code) {
    hash_code = typeid(T).hash_code();
    auto ptr = std::static_pointer_cast<ISerializable>(std::make_shared<T>());
    return ptr;
  });
}
#pragma endregion

YAML::Emitter& operator<<(YAML::Emitter& out, const glm::vec2& v);
YAML::Emitter& operator<<(YAML::Emitter& out, const glm::vec3& v);
YAML::Emitter& operator<<(YAML::Emitter& out, const glm::vec4& v);

YAML::Emitter& operator<<(YAML::Emitter& out, const glm::quat& v);
YAML::Emitter& operator<<(YAML::Emitter& out, const glm::mat4& v);

YAML::Emitter& operator<<(YAML::Emitter& out, const glm::dvec2& v);
YAML::Emitter& operator<<(YAML::Emitter& out, const glm::dvec3& v);
YAML::Emitter& operator<<(YAML::Emitter& out, const glm::dvec4& v);

YAML::Emitter& operator<<(YAML::Emitter& out, const glm::ivec2& v);
YAML::Emitter& operator<<(YAML::Emitter& out, const glm::ivec3& v);
YAML::Emitter& operator<<(YAML::Emitter& out, const glm::ivec4& v);

YAML::Emitter& operator<<(YAML::Emitter& out, const glm::uvec2& v);
YAML::Emitter& operator<<(YAML::Emitter& out, const glm::uvec3& v);
YAML::Emitter& operator<<(YAML::Emitter& out, const glm::uvec4& v);

YAML::Emitter& operator<<(YAML::Emitter& out, const glm::u8vec4& v);
YAML::Emitter& operator<<(YAML::Emitter& out, const glm::i8vec4& v);
YAML::Emitter& operator<<(YAML::Emitter& out, const glm::u16vec4& v);
YAML::Emitter& operator<<(YAML::Emitter& out, const glm::i16vec4& v);

template <typename T>
std::shared_ptr<T> Serialization::ProduceSerializable() {
  auto& serialization_manager = GetInstance();
  const auto type_name = GetSerializableTypeName<T>();
  const auto it = serialization_manager.serializable_generators_.find(type_name);
  if (it != serialization_manager.serializable_generators_.end()) {
    size_t hash_code;
    auto ret_val = it->second(hash_code);
    ret_val->type_name_ = type_name;
    return std::move(std::static_pointer_cast<T>(ret_val));
  }
  EVOENGINE_ERROR("Serializable " + type_name + "is not registered!")
  throw 1;
}
}  // namespace evo_engine
