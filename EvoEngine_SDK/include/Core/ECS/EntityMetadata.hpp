
#pragma once
#include "Entity.hpp"
#include "IPrivateComponent.hpp"

namespace evo_engine {

/**
 * @brief Represents the metadata associated with an Entity in the scene.
 */
struct EntityMetadata {
  /**
   * @brief The name of the entity.
   */
  std::string entity_name;

  /**
   * @brief Indicates whether the entity is static (non-dynamic).
   */
  bool entity_static = false;

  /**
   * @brief Indicates if one of the ancestors of this entity is selected.
   */
  bool ancestor_selected = false;

  /**
   * @brief The version of the entity.
   */
  unsigned entity_version = 1;

  /**
   * @brief Indicates whether the entity is enabled.
   */
  bool entity_enabled = true;

  /**
   * @brief Indicates whether this entity is persisted when saving the scene.
   */
  bool entity_serializable = true;

  /**
   * @brief The parent entity of this entity.
   */
  Entity parent = Entity();

  /**
   * @brief The root entity of this hierarchy.
   */
  Entity root = Entity();

  /**
   * @brief A collection of private component elements associated with this entity.
   */
  std::vector<PrivateComponentElement> private_component_elements;

  /**
   * @brief A collection of child entities of this entity.
   */
  std::vector<Entity> children;

  /**
   * @brief The index of the data component storage for this entity.
   */
  size_t data_component_storage_index = 0;

  /**
   * @brief The index array for chunk storage related to this entity.
   */
  size_t chunk_array_index = 0;

  /**
   * @brief The handle associated with this entity.
   */
  Handle entity_handle;

  /**
   * @brief Serializes the entity metadata into a YAML output stream.
   *
   * @param out The YAML emitter used for serialization.
   * @param scene A shared pointer to the scene to which the entity belongs.
   */
  void Serialize(YAML::Emitter &out, const std::shared_ptr<Scene> &scene) const;

  /**
   * @brief Deserializes the entity metadata from a YAML node.
   *
   * @param in The YAML node that contains the serialized metadata.
   * @param scene A shared pointer to the scene to which the entity belongs.
   */
  void Deserialize(const YAML::Node &in, const std::shared_ptr<Scene> &scene);

  /**
   * @brief Clones the entity metadata from a source, mapping entity handles
   *        using a given entity map.
   *
   * @param entity_map A map of old entity handles to new entity handles.
   * @param source The source metadata to clone from.
   * @param scene A shared pointer to the scene to which the entity belongs.
   */
  void Clone(const std::unordered_map<Handle, Handle> &entity_map, const EntityMetadata &source,
             const std::shared_ptr<Scene> &scene);
};

}  // namespace evo_engine
