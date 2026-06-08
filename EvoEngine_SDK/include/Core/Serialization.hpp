
#pragma once
#include "Console.hpp"
#include "EvoEngineAPI.hpp"
#include "IAsset.hpp"
#include "IDataComponent.hpp"
#include "IPrivateComponent.hpp"
#include "ISerializable.hpp"
#include "ISystem.hpp"
namespace YAML {

/**
 * @brief YAML specialization to serialize and deserialize glm::vec2.
 */
class Node;
class Emitter;
template <>
struct convert<glm::vec2> {
  /**
   * @brief Serializes a glm::vec2 to a YAML node.
   * @param rhs The glm::vec2 object to serialize.
   * @return The serialized YAML Node.
   */
  static Node encode(const glm::vec2& rhs) {
    Node node;
    node.push_back(rhs.x);
    node.push_back(rhs.y);
    return node;
  }

  /**
   * @brief Deserializes a YAML node to a glm::vec2.
   * @param node The input YAML node.
   * @param rhs Reference to a glm::vec2 to store the deserialized value.
   * @return True if decoding was successful, false otherwise.
   */
  static bool decode(const Node& node, glm::vec2& rhs) {
    if (!node.IsSequence() || node.size() != 2) {
      return false;
    }

    rhs.x = node[0].as<float>();
    rhs.y = node[1].as<float>();
    return true;
  }
};

/**
 * @brief YAML specialization to serialize and deserialize glm::vec3.
 */
template <>
struct convert<glm::vec3> {
  /**
   * @brief Serializes a glm::vec3 to a YAML node.
   * @param rhs The glm::vec3 object to serialize.
   * @return The serialized YAML Node.
   */
  static Node encode(const glm::vec3& rhs) {
    Node node;
    node.push_back(rhs.x);
    node.push_back(rhs.y);
    node.push_back(rhs.z);
    return node;
  }

  /**
   * @brief Deserializes a YAML node to a glm::vec3.
   * @param node The input YAML node.
   * @param rhs Reference to a glm::vec3 to store the deserialized value.
   * @return True if decoding was successful, false otherwise.
   */
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

/**
 * @brief YAML specialization to serialize and deserialize glm::vec4.
 */
template <>
struct convert<glm::vec4> {
  /**
   * @brief Serializes a glm::vec4 to a YAML node.
   * @param rhs The glm::vec4 object to serialize.
   * @return The serialized YAML Node.
   */
  static Node encode(const glm::vec4& rhs) {
    Node node;
    node.push_back(rhs.x);
    node.push_back(rhs.y);
    node.push_back(rhs.z);
    node.push_back(rhs.w);
    return node;
  }

  /**
   * @brief Deserializes a YAML node to a glm::vec4.
   * @param node The input YAML node.
   * @param rhs Reference to a glm::vec4 to store the deserialized value.
   * @return True if decoding was successful, false otherwise.
   */
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

/**
 * @brief YAML specialization to serialize and deserialize glm::quat.
 */
template <>
struct convert<glm::quat> {
  /**
   * @brief Serializes a glm::quat to a YAML node.
   * @param rhs The glm::quat object to serialize.
   * @return The serialized YAML Node.
   */
  static Node encode(const glm::quat& rhs) {
    Node node;
    node.push_back(rhs.x);
    node.push_back(rhs.y);
    node.push_back(rhs.z);
    node.push_back(rhs.w);
    return node;
  }

  /**
   * @brief Deserializes a YAML node to a glm::quat.
   * @param node The input YAML node.
   * @param rhs Reference to a glm::quat to store the deserialized value.
   * @return True if decoding was successful, false otherwise.
   */
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

/**
 * @brief YAML specialization to serialize and deserialize glm::mat4.
 */
template <>
struct convert<glm::mat4> {
  /**
   * @brief Serializes a glm::mat4 to a YAML node.
   * @param rhs The glm::mat4 object to serialize.
   * @return The serialized YAML Node.
   */
  static Node encode(const glm::mat4& rhs) {
    Node node;
    node.push_back(rhs[0]);
    node.push_back(rhs[1]);
    node.push_back(rhs[2]);
    node.push_back(rhs[3]);
    return node;
  }

  /**
   * @brief Deserializes a YAML node to a glm::mat4.
   * @param node The input YAML node.
   * @param rhs Reference to a glm::mat4 to store the deserialized value.
   * @return True if decoding was successful, false otherwise.
   */
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

/**
 * @brief YAML specialization to serialize and deserialize glm::dvec2.
 */
template <>
struct convert<glm::dvec2> {
  /**
   * @brief Serializes a glm::dvec2 to a YAML node.
   * @param rhs The glm::dvec2 object to serialize.
   * @return The serialized YAML Node.
   */
  static Node encode(const glm::dvec2& rhs) {
    Node node;
    node.push_back(rhs.x);
    node.push_back(rhs.y);
    return node;
  }

  /**
   * @brief Deserializes a YAML node to a glm::dvec2.
   * @param node The input YAML node.
   * @param rhs Reference to a glm::dvec2 to store the deserialized value.
   * @return True if decoding was successful, false otherwise.
   */
  static bool decode(const Node& node, glm::dvec2& rhs) {
    if (!node.IsSequence() || node.size() != 2) {
      return false;
    }

    rhs.x = node[0].as<double>();
    rhs.y = node[1].as<double>();
    return true;
  }
};

/**
 * @brief YAML specialization to serialize and deserialize glm::dvec3.
 */
template <>
struct convert<glm::dvec3> {
  /**
   * @brief Serializes a glm::dvec3 to a YAML node.
   * @param rhs The glm::dvec3 object to serialize.
   * @return The serialized YAML Node.
   */
  static Node encode(const glm::dvec3& rhs) {
    Node node;
    node.push_back(rhs.x);
    node.push_back(rhs.y);
    node.push_back(rhs.z);
    return node;
  }

  /**
   * @brief Deserializes a YAML node to a glm::dvec3.
   * @param node The input YAML node.
   * @param rhs Reference to a glm::dvec3 to store the deserialized value.
   * @return True if decoding was successful, false otherwise.
   */
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

/**
 * @brief YAML specialization to serialize and deserialize glm::dvec4.
 */
template <>
struct convert<glm::dvec4> {
  /**
   * @brief Serializes a glm::dvec4 to a YAML node.
   * @param rhs The glm::dvec4 object to serialize.
   * @return The serialized YAML Node.
   */
  static Node encode(const glm::dvec4& rhs) {
    Node node;
    node.push_back(rhs.x);
    node.push_back(rhs.y);
    node.push_back(rhs.z);
    node.push_back(rhs.w);
    return node;
  }

  /**
   * @brief Deserializes a YAML node to a glm::dvec4.
   * @param node The input YAML node.
   * @param rhs Reference to a glm::dvec4 to store the deserialized value.
   * @return True if decoding was successful, false otherwise.
   */
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

/**
 * @brief YAML specialization to serialize and deserialize glm::ivec2.
 */
template <>
struct convert<glm::ivec2> {
  /**
   * @brief Serializes a glm::ivec2 to a YAML node.
   * @param rhs The glm::ivec2 object to serialize.
   * @return The serialized YAML Node.
   */
  static Node encode(const glm::ivec2& rhs) {
    Node node;
    node.push_back(rhs.x);
    node.push_back(rhs.y);
    return node;
  }

  /**
   * @brief Deserializes a YAML node to a glm::ivec2.
   * @param node The input YAML node.
   * @param rhs Reference to a glm::ivec2 to store the deserialized value.
   * @return True if decoding was successful, false otherwise.
   */
  static bool decode(const Node& node, glm::ivec2& rhs) {
    if (!node.IsSequence() || node.size() != 2) {
      return false;
    }

    rhs.x = node[0].as<int>();
    rhs.y = node[1].as<int>();
    return true;
  }
};

/**
 * @brief YAML specialization to serialize and deserialize glm::ivec3.
 */
template <>
struct convert<glm::ivec3> {
  /**
   * @brief Serializes a glm::ivec3 to a YAML node.
   * @param rhs The glm::ivec3 object to serialize.
   * @return The serialized YAML Node.
   */
  static Node encode(const glm::ivec3& rhs) {
    Node node;
    node.push_back(rhs.x);
    node.push_back(rhs.y);
    node.push_back(rhs.z);
    return node;
  }

  /**
   * @brief Deserializes a YAML node to a glm::ivec3.
   * @param node The input YAML node.
   * @param rhs Reference to a glm::ivec3 to store the deserialized value.
   * @return True if decoding was successful, false otherwise.
   */
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

/**
 * @brief YAML specialization to serialize and deserialize glm::ivec4.
 */
template <>
struct convert<glm::ivec4> {
  /**
   * @brief Serializes a glm::ivec4 to a YAML node.
   * @param rhs The glm::ivec4 object to serialize.
   * @return The serialized YAML Node.
   */
  static Node encode(const glm::ivec4& rhs) {
    Node node;
    node.push_back(rhs.x);
    node.push_back(rhs.y);
    node.push_back(rhs.z);
    node.push_back(rhs.w);
    return node;
  }

  /**
   * @brief Deserializes a YAML node to a glm::ivec4.
   * @param node The input YAML node.
   * @param rhs Reference to a glm::ivec4 to store the deserialized value.
   * @return True if decoding was successful, false otherwise.
   */
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

/**
 * @brief YAML specialization to serialize and deserialize glm::uvec2.
 */
template <>
struct convert<glm::uvec2> {
  /**
   * @brief Serializes a glm::uvec2 to a YAML node.
   * @param rhs The glm::uvec2 object to serialize.
   * @return The serialized YAML Node.
   */
  static Node encode(const glm::uvec2& rhs) {
    Node node;
    node.push_back(rhs.x);
    node.push_back(rhs.y);
    return node;
  }

  /**
   * @brief Deserializes a YAML node to a glm::uvec2.
   * @param node The input YAML node.
   * @param rhs Reference to a glm::uvec2 to store the deserialized value.
   * @return True if decoding was successful, false otherwise.
   */
  static bool decode(const Node& node, glm::uvec2& rhs) {
    if (!node.IsSequence() || node.size() != 2) {
      return false;
    }

    rhs.x = node[0].as<unsigned>();
    rhs.y = node[1].as<unsigned>();
    return true;
  }
};

/**
 * @brief YAML specialization to serialize and deserialize glm::uvec3.
 */
template <>
struct convert<glm::uvec3> {
  /**
   * @brief Serializes a glm::uvec3 to a YAML node.
   * @param rhs The glm::uvec3 object to serialize.
   * @return The serialized YAML Node.
   */
  static Node encode(const glm::uvec3& rhs) {
    Node node;
    node.push_back(rhs.x);
    node.push_back(rhs.y);
    node.push_back(rhs.z);
    return node;
  }

  /**
   * @brief Deserializes a YAML node to a glm::uvec3.
   * @param node The input YAML node.
   * @param rhs Reference to a glm::uvec3 to store the deserialized value.
   * @return True if decoding was successful, false otherwise.
   */
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

/**
 * @brief YAML specialization to serialize and deserialize glm::uvec4.
 */
template <>
struct convert<glm::uvec4> {
  /**
   * @brief Serializes a glm::uvec4 to a YAML node.
   * @param rhs The glm::uvec4 object to serialize.
   * @return The serialized YAML Node.
   */
  static Node encode(const glm::uvec4& rhs) {
    Node node;
    node.push_back(rhs.x);
    node.push_back(rhs.y);
    node.push_back(rhs.z);
    node.push_back(rhs.w);
    return node;
  }

  /**
   * @brief Deserializes a YAML node to a glm::uvec4.
   * @param node The input YAML node.
   * @param rhs Reference to a glm::uvec4 to store the deserialized value.
   * @return True if decoding was successful, false otherwise.
   */
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

/**
 * @brief YAML specialization to serialize and deserialize glm::i8vec4.
 */
template <>
struct convert<glm::i8vec4> {
  /**
   * @brief Serializes a glm::i8vec4 to a YAML node.
   * @param rhs The glm::i8vec4 object to serialize.
   * @return The serialized YAML Node.
   */
  static Node encode(const glm::i8vec4& rhs) {
    Node node;
    node.push_back(rhs.x);
    node.push_back(rhs.y);
    node.push_back(rhs.z);
    node.push_back(rhs.w);
    return node;
  }

  /**
   * @brief Deserializes a YAML node to a glm::i8vec4.
   * @param node The input YAML node.
   * @param rhs Reference to a glm::i8vec4 to store the deserialized value.
   * @return True if decoding was successful, false otherwise.
   */
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

/**
 * @brief YAML specialization to serialize and deserialize glm::u8vec4.
 */
template <>
struct convert<glm::u8vec4> {
  /**
   * @brief Serializes a glm::u8vec4 to a YAML node.
   * @param rhs The glm::u8vec4 object to serialize.
   * @return The serialized YAML Node.
   */
  static Node encode(const glm::u8vec4& rhs) {
    Node node;
    node.push_back(rhs.x);
    node.push_back(rhs.y);
    node.push_back(rhs.z);
    node.push_back(rhs.w);
    return node;
  }

  /**
   * @brief Deserializes a YAML node to a glm::u8vec4.
   * @param node The input YAML node.
   * @param rhs Reference to a glm::u8vec4 to store the deserialized value.
   * @return True if decoding was successful, false otherwise.
   */
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

/**
 * @brief YAML specialization to serialize and deserialize glm::i16vec4.
 */
template <>
struct convert<glm::i16vec4> {
  /**
   * @brief Serializes a glm::i16vec4 to a YAML node.
   * @param rhs The glm::i16vec4 object to serialize.
   * @return The serialized YAML Node.
   */
  static Node encode(const glm::i16vec4& rhs) {
    Node node;
    node.push_back(rhs.x);
    node.push_back(rhs.y);
    node.push_back(rhs.z);
    node.push_back(rhs.w);
    return node;
  }

  /**
   * @brief Deserializes a YAML node to a glm::i16vec4.
   * @param node The input YAML node.
   * @param rhs Reference to a glm::i16vec4 to store the deserialized value.
   * @return True if decoding was successful, false otherwise.
   */
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

/**
 * @brief YAML specialization to serialize and deserialize glm::u16vec4.
 */
template <>
struct convert<glm::u16vec4> {
  /**
   * @brief Serializes a glm::u16vec4 to a YAML node.
   * @param rhs The glm::u16vec4 object to serialize.
   * @return The serialized YAML Node.
   */
  static Node encode(const glm::u16vec4& rhs) {
    Node node;
    node.push_back(rhs.x);
    node.push_back(rhs.y);
    node.push_back(rhs.z);
    node.push_back(rhs.w);
    return node;
  }

  /**
   * @brief Deserializes a YAML node to a glm::u16vec4.
   * @param node The input YAML node.
   * @param rhs Reference to a glm::u16vec4 to store the deserialized value.
   * @return True if decoding was successful, false otherwise.
   */
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
namespace evo_engine {
class ProjectContentBrowserPanel;

/**
 * @brief Application-owned registry responsible for serialization and reflected type registration.
 */
class Serialization final {
 public:
  Serialization() = default;
  static EVOENGINE_API Serialization& GetInstance();

 private:
  friend class Application;
  friend class ISerializable;
  friend class ProjectManager;
  friend class ClassRegistry;
  friend class EditorLayer;
  friend class ProjectContentBrowserPanel;
  friend class Scene;
  friend class PackageManager;
  friend class PackageRegistrar;

  /**
   * @brief Map to store generators for data components.
   */
  std::unordered_map<std::string, std::function<std::shared_ptr<IDataComponent>(size_t&, size_t&)>>
      data_component_generators_{};

  /**
   * @brief Map to store generators for serializable components.
   */
  std::unordered_map<std::string, std::function<std::shared_ptr<ISerializable>(size_t&)>> serializable_generators_{};

  /**
   * @brief Map to store clone functions for private components.
   */
  std::unordered_map<std::string,
                     std::function<void(std::shared_ptr<IPrivateComponent>, const std::shared_ptr<IPrivateComponent>&)>>
      private_component_cloners_{};

  /**
   * @brief Map to store clone functions for systems.
   */
  std::unordered_map<std::string, std::function<void(std::shared_ptr<ISystem>, const std::shared_ptr<ISystem>&)>>
      system_cloners_{};

  /**
   * @brief Map to store IDs for data components.
   */
  std::map<std::string, size_t> data_component_ids_{};

  /**
   * @brief Map to store sizes of data components by their IDs.
   */
  std::unordered_map<size_t, size_t> data_component_sizes_{};

  /**
   * @brief Map to store names of data components by their IDs.
   */
  std::unordered_map<size_t, std::string> data_component_names_{};

  /**
   * @brief Maps package-owned data component type names and IDs to their owning package.
   */
  std::unordered_map<std::string, std::string> data_component_type_owners_{};
  std::unordered_map<size_t, std::string> data_component_type_id_owners_{};

  /**
   * @brief Map to store IDs for private components.
   */
  std::map<std::string, size_t> private_component_ids_{};

  /**
   * @brief Map to store names of private components by their IDs.
   */
  std::unordered_map<size_t, std::string> private_component_names_{};

  /**
   * @brief Maps package-owned private component type names and IDs to their owning package.
   */
  std::unordered_map<std::string, std::string> private_component_type_owners_{};
  std::unordered_map<size_t, std::string> private_component_type_id_owners_{};

  /**
   * @brief Map to store IDs for systems.
   */
  std::map<std::string, size_t> system_ids_{};

  /**
   * @brief Map to store names of systems by their IDs.
   */
  std::unordered_map<size_t, std::string> system_names_{};

  /**
   * @brief Maps package-owned system type names and IDs to their owning package.
   */
  std::unordered_map<std::string, std::string> system_type_owners_{};
  std::unordered_map<size_t, std::string> system_type_id_owners_{};

  /**
   * @brief Map to store IDs for serializable components.
   */
  std::unordered_map<std::string, size_t> serializable_ids_{};

  /**
   * @brief Map to store names of serializable components by their IDs.
   */
  std::unordered_map<size_t, std::string> serializable_names_{};

  /**
   * @brief Maps package-owned serializable type names and IDs to their owning package.
   */
  std::unordered_map<std::string, std::string> serializable_type_owners_{};
  std::unordered_map<size_t, std::string> serializable_type_id_owners_{};

  /**
   * @brief Map to store extensions for asset types.
   */
  std::map<std::string, std::vector<std::string>> asset_extensions_{};

  /**
   * @brief Map to store type names.
   */
  std::map<std::string, std::string> type_names_{};

  /**
   * @brief Register a type of data component.
   * @tparam T Type of data component.
   * @param name Name of the type.
   * @return True if registration is successful, false otherwise.
   */
  template <typename T = IDataComponent>
  static bool RegisterDataComponentType(const std::string& name);

  /**
   * @brief Register a type of serializable component.
   * @tparam T Type of the serializable component.
   * @param name Name of the type.
   * @return True if registration is successful, false otherwise.
   */
  template <typename T = ISerializable>
  static bool RegisterSerializableType(const std::string& name);

  /**
   * @brief Register a type of private component.
   * @tparam T Type of private component.
   * @param name Name of the type.
   * @return True if registration is successful, false otherwise.
   */
  template <typename T = IPrivateComponent>
  static bool RegisterPrivateComponentType(const std::string& name);

  /**
   * @brief Register a type of system.
   * @tparam T Type of the system.
   * @param name Name of the type.
   * @return True if registration is successful, false otherwise.
   */
  template <typename T = ISystem>
  static bool RegisterSystemType(const std::string& name);

  /**
   * @brief Register a type of asset.
   * @tparam T Type of the asset.
   * @param name Name of the type.
   * @param extensions List of file extensions associated with this type.
   * @return True if registration is successful, false otherwise.
   */
  template <typename T = IAsset>
  static bool RegisterAssetType(const std::string& name, const std::vector<std::string>& extensions);

  /**
   * @brief Register a type of data component with custom attributes.
   * @param type_name Name of the type.
   * @param type_index Unique index for the type.
   * @param type_size Size of the type in bytes.
   * @param func Generator function to produce the component.
   * @return True if registration is successful, false otherwise.
   */
  static bool RegisterDataComponentType(const std::string& type_name, const size_t& type_index, const size_t& type_size,
                                        const std::function<std::shared_ptr<IDataComponent>(size_t&, size_t&)>& func);

  /**
   * @brief Register a type of serializable component with custom attributes.
   * @param type_name Name of the type.
   * @param type_index Unique index for the type.
   * @param func Generator function to produce the component.
   * @return True if registration is successful, false otherwise.
   */
  static bool RegisterSerializableType(const std::string& type_name, const size_t& type_index,
                                       const std::function<std::shared_ptr<ISerializable>(size_t&)>& func);

  /**
   * @brief Register a type of private component with cloning functionality.
   * @param type_name Name of the type.
   * @param type_index Unique index for the type.
   * @param clone_func Function to clone the component.
   * @return True if registration is successful, false otherwise.
   */
  static bool RegisterPrivateComponentType(
      const std::string& type_name, const size_t& type_index,
      const std::function<void(std::shared_ptr<IPrivateComponent>, const std::shared_ptr<IPrivateComponent>&)>&
          clone_func);

  /**
   * @brief Register a type of system with cloning functionality.
   * @param type_name Name of the type.
   * @param type_index Unique index for the type.
   * @param clone_func Function to clone the system.
   * @return True if registration is successful, false otherwise.
   */
  static bool RegisterSystemType(
      const std::string& type_name, const size_t& type_index,
      const std::function<void(std::shared_ptr<ISystem>, const std::shared_ptr<ISystem>&)>& clone_func);

  /**
   * @brief Register an asset type with custom attributes.
   * @param type_name Name of the type.
   * @param type_index Unique index for the type.
   * @param extensions List of file extensions associated with this type.
   * @param func Function to serialize the asset.
   * @return True if registration is successful, false otherwise.
   */
  static bool RegisterAssetType(const std::string& type_name, const size_t& type_index,
                                const std::vector<std::string>& extensions,
                                const std::function<std::shared_ptr<ISerializable>(size_t&)>& func);

  static bool UnregisterSerializableType(const std::string& type_name);
  static bool UnregisterPrivateComponentType(const std::string& type_name);
  static bool UnregisterAssetType(const std::string& type_name);
  static bool UnregisterDataComponentType(const std::string& type_name);
  static bool UnregisterSystemType(const std::string& type_name);
  static void SetSerializableTypeOwner(const std::string& type_name, const std::string& owner_name);
  static void SetPrivateComponentTypeOwner(const std::string& type_name, const std::string& owner_name);
  static void SetDataComponentTypeOwner(const std::string& type_name, const std::string& owner_name);
  static void SetSystemTypeOwner(const std::string& type_name, const std::string& owner_name);
  static std::vector<size_t> GetPackageOwnedPrivateComponentTypeIds(const std::string& owner_name);
  static std::vector<size_t> GetPackageOwnedDataComponentTypeIds(const std::string& owner_name);
  static std::vector<size_t> GetPackageOwnedSystemTypeIds(const std::string& owner_name);
  static void UnregisterPackageOwnedTypes(const std::string& owner_name);

 public:
  /**
   * @brief Creates an instance of a data component by type name.
   * @param type_name Name of the data component type.
   * @param hash_code Reference to store the hash code of the type.
   * @param size Reference to store the size of the component.
   * @return A shared pointer to the created data component.
   */
  static std::shared_ptr<IDataComponent> ProduceDataComponent(const std::string& type_name, size_t& hash_code,
                                                              size_t& size);

  /**
   * @brief Clones a private component from the source to the target.
   * @param target The private component to overwrite.
   * @param source The private component to clone from.
   */
  static void ClonePrivateComponent(const std::shared_ptr<IPrivateComponent>& target,
                                    const std::shared_ptr<IPrivateComponent>& source);

  /**
   * @brief Clones a system from the source to the target.
   * @param target The system to overwrite.
   * @param source The system to clone from.
   */
  static void CloneSystem(const std::shared_ptr<ISystem>& target, const std::shared_ptr<ISystem>& source);

  /**
   * @brief Creates an instance of a serializable object by type name.
   * @param type_name Name of the serializable type.
   * @param hash_code Reference to store the hash code of the type.
   * @return A shared pointer to the created ISerializable object.
   */
  static std::shared_ptr<ISerializable> ProduceSerializable(const std::string& type_name, size_t& hash_code);

  /**
   * @brief Creates an instance of a serializable object by type name.
   * @param type_name Name of the serializable type.
   * @return A shared pointer to the created ISerializable object.
   */
  static std::shared_ptr<ISerializable> ProduceSerializable(const std::string& type_name);

  /**
   * @brief Produces a serializable object for a given type name and handle.
   * @param type_name Name of the serializable type.
   * @param hash_code Reference to the hash code of the type.
   * @param handle A handle used in object creation.
   * @return A shared pointer to the created ISerializable object.
   */
  static auto ProduceSerializable(const std::string& type_name, size_t& hash_code, const Handle& handle)
      -> std::shared_ptr<ISerializable>;

  /**
   * @brief Creates an instance of a serializable object of type T.
   * @tparam T The type of the serializable object.
   * @return A shared pointer to the created object of type T.
   */
  template <typename T = ISerializable>
  static std::shared_ptr<T> ProduceSerializable();

  /**
   * @brief Gets the name of a data component type.
   * @tparam T The type of the data component.
   * @return The name of the data component type.
   * @throws std::invalid_argument if the type is unregistered.
   */
  template <typename T = IDataComponent>
  static std::string GetDataComponentTypeName();

  /**
   * @brief Gets the name of a data component type by type ID.
   * @param type_id The type ID of the data component.
   * @return The name of the data component type.
   */
  static std::string GetDataComponentTypeName(const size_t& type_id);

  /**
   * @brief Gets the name of a serializable type.
   * @tparam T The type of the serializable component.
   * @return The name of the serializable type.
   * @throws std::invalid_argument if the type is unregistered.
   */
  template <typename T = ISerializable>
  static std::string GetSerializableTypeName();

  /**
   * @brief Gets the name of a serializable type by type ID.
   * @param type_id The type ID of the serializable component.
   * @return The name of the serializable type.
   */
  static std::string GetSerializableTypeName(const size_t& type_id);

  /**
   * @brief Checks if a serializable type is registered by name.
   * @param type_name Name of the serializable type.
   * @return True if the type is registered, false otherwise.
   */
  static bool HasSerializableType(const std::string& type_name);

  /**
   * @brief Checks if a serializable type is registered by type ID.
   * @param type_id The type ID of the serializable type.
   * @return True if the type is registered, false otherwise.
   */
  static bool HasSerializableType(const size_t& type_id);

  /**
   * @brief Checks if a data component type is registered by name.
   * @param type_name Name of the data component type.
   * @return True if the type is registered, false otherwise.
   */
  static bool HasComponentDataType(const std::string& type_name);

  /**
   * @brief Checks if a data component type is registered by type ID.
   * @param type_id The type ID of the data component.
   * @return True if the type is registered, false otherwise.
   */
  static bool HasComponentDataType(const size_t& type_id);

  /**
   * @brief Checks if an asset type is registered by name.
   * @param type_name Name of the asset type.
   * @return True if the type is registered, false otherwise.
   */
  static bool HasAssetType(const std::string& type_name);

  /**
   * @brief Gets the file extensions for a registered asset type.
   * @param type_name Name of the asset type.
   * @return A vector containing the file extensions.
   */
  static const std::vector<std::string>& PeekAssetExtensions(const std::string& type_name);

  /**
   * @brief Gets the name of an asset type by file extension.
   * @param extension The file extension.
   * @return The name of the asset type.
   */
  static std::string GetAssetTypeName(const std::string& extension);

  /**
   * @brief Gets the type ID of a serializable type by name.
   * @param type_name Name of the serializable type.
   * @return The type ID of the serializable type.
   */
  static size_t GetSerializableTypeId(const std::string& type_name);

  /**
   * @brief Gets the type ID of a data component by name.
   * @param type_name Name of the data component.
   * @return The type ID of the data component.
   */
  static size_t GetDataComponentTypeId(const std::string& type_name);

  /**
   * @brief Gets the size of a data component by its type ID.
   * @param type_id The type ID of the data component.
   * @return The size of the data component.
   */
  static size_t GetDataComponentTypeSize(const size_t& type_id);

  /**
   * @brief Saves a list of assets to a YAML emitter.
   * @param name The property name for the list.
   * @param target The list of assets to save.
   * @param out The YAML emitter.
   */
  static void SaveAssetList(const std::string& name, const std::vector<AssetRef>& target, YAML::Emitter& out);

  /**
   * @brief Loads a list of assets from a YAML node.
   * @param name The property name for the list.
   * @param target The vector to store the loaded assets.
   * @param in The input YAML node.
   */
  static void LoadAssetList(const std::string& name, std::vector<AssetRef>& target, const YAML::Node& in);

  /**
   * @brief Serializes a vector to a YAML emitter.
   * @tparam T The type of the elements in the vector.
   * @param name The property name for the vector.
   * @param target The vector to serialize.
   * @param out The YAML emitter.
   */
  template <typename T>
  static void SerializeVector(const std::string& name, const std::vector<T>& target, YAML::Emitter& out);

  /**
   * @brief Deserializes a vector from a YAML node.
   * @tparam T The type of the elements in the vector.
   * @param name The property name for the vector.
   * @param target The vector to store the deserialized elements.
   * @param in The input YAML node.
   */
  template <typename T>
  static void DeserializeVector(const std::string& name, std::vector<T>& target, const YAML::Node& in);

  /**
   * @brief Clear all registered classes.
   */
  static void OnDestroy();
};

template <typename T>
std::string Serialization::GetDataComponentTypeName() {
  return GetDataComponentTypeName(typeid(T).hash_code());
}
template <typename T>
std::string Serialization::GetSerializableTypeName() {
  return GetSerializableTypeName(typeid(T).hash_code());
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
  const auto type_name = GetSerializableTypeName<T>();
  size_t hash_code;
  auto ret_val = ProduceSerializable(type_name, hash_code);
  if (ret_val) {
    return std::static_pointer_cast<T>(ret_val);
  }
  throw std::invalid_argument("Type is unregistered!");
}
}  // namespace evo_engine
