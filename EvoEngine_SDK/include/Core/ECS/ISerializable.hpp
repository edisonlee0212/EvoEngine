
#pragma once
#include "IHandle.hpp"

namespace YAML {
class Emitter;
class Node;
}  // namespace YAML

namespace evo_engine {
/**
 * @class ISerializable
 * @brief Represents an abstract class for serializable objects in the engine.
 *
 * This class allows derived classes to implement custom serialization
 * and deserialization logic. It provides functionality to save
 * and load data using YAML emitters and nodes.
 */
class ISerializable : public IHandle {
  friend class Serialization;    ///< Grants access to the Serialization class.
  friend class IAsset;           ///< Grants access to the IAsset class.
  friend class Entities;         ///< Grants access to the Entities class.
  friend class Scene;            ///< Grants access to the Scene class.
  friend class Serialization;    ///< Grants access to the Serialization class.
  friend struct EntityMetadata;  ///< Grants access to the EntityMetadata structure.
  friend class File;             ///< Grants access to the File class.
  friend class Folder;           ///< Grants access to the Folder class.

  std::string type_name_;  ///< The type name associated with the serializable object.

 public:
  /**
   * @brief Saves the current object state to a YAML emitter.
   *
   * @param name The name under which to save the object.
   * @param out The YAML emitter to output the serialized data.
   */
  void Save(const std::string &name, YAML::Emitter &out) const;

  /**
   * @brief Loads the object state from a given YAML node.
   *
   * @param name The name under which the object data is stored.
   * @param in The YAML node containing the serialized data.
   */
  void Load(const std::string &name, const YAML::Node &in);

  /**
   * @brief Retrieves the type name of the current object.
   *
   * @return A string representing the type name.
   */
  [[nodiscard]] std::string GetTypeName() {
    return type_name_;
  }

  /**
   * @brief Virtual destructor for the ISerializable class.
   */
  virtual ~ISerializable() = default;

  /**
   * @brief Serializes the object to the specified YAML emitter.
   *
   * This is a virtual method that can be overridden in derived classes
   * to define specific serialization logic.
   *
   * @param out The YAML emitter to output the serialized data.
   */
  virtual void Serialize(YAML::Emitter &out) const {
  }

  /**
   * @brief Deserializes the object from the specified YAML node.
   *
   * This is a virtual method that can be overridden in derived classes
   * to define specific deserialization logic.
   *
   * @param in The YAML node containing the serialized data.
   */
  virtual void Deserialize(const YAML::Node &in) {
  }
};
}  // namespace evo_engine
