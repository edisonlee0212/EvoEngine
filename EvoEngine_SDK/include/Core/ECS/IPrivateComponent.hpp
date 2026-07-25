
#pragma once
#include "AssetRef.hpp"
#include "Entity.hpp"

namespace evo_engine {
class EditorLayer;

/**
 * @class IPrivateComponent
 * @brief Represents a private component in the EVO engine ecosystem.
 *
 * This interface provides various lifecycle callbacks and utility methods
 * to manage private components associated with entities in a scene.
 */
class IPrivateComponent : public ISerializable {
  friend class Entities;
  friend class EditorLayer;
  friend struct PrivateComponentElement;
  friend class PrivateComponentStorage;
  friend class Serialization;
  friend class Scene;
  friend void DeserializeScene(const YAML::Node& in, Scene& scene);
  friend class Prefab;
  friend class PackageRegistrar;
  friend struct EntityMetadata;

  bool enabled_ = true;         ///< Indicates whether the component is enabled.
  Entity owner_ = Entity();     ///< The entity that owns this component.
  bool started_ = false;        ///< Indicates whether the component has started.
  size_t version_ = 0;          ///< Version of the component (useful for tracking changes).
  std::weak_ptr<Scene> scene_;  ///< Weak pointer to the scene this component belongs to.

 public:
  /**
   * @brief Return the scene this component belongs to.
   * @return The scene this component belongs to.
   */
  [[nodiscard]] std::shared_ptr<Scene> GetScene() const;

  /**
   * @brief Get the owner of this component.
   * @return The entity that contains this component.
   */
  [[nodiscard]] Entity GetOwner() const;

  /**
   * @brief Get the version of the current component.
   * @return The version of the current component.
   */
  [[nodiscard]] size_t GetVersion() const;

  /**
   * @brief Enable or disable this component.
   *
   * Disabled components will not be updated or fixed-updated in the simulation.
   *
   * @param value Target property indicating whether the component should be enabled.
   */
  void SetEnabled(const bool& value);

  /** @brief Returns whether this component may transition to enabled state. */
  [[nodiscard]] virtual bool CanEnable() const {
    return true;
  }

  /**
   * @brief Get current enabled status of the component.
   * @return True if the component is enabled, false otherwise.
   */
  [[nodiscard]] bool IsEnabled() const;

  /**
   * @brief Check if the component has started.
   * @return True if the component has started, false otherwise.
   */
  [[nodiscard]] bool Started() const;

  /**
   * @brief Virtual function called during the fixed update phase.
   */
  virtual void FixedUpdate() {
  }

  /**
   * @brief Virtual function called during the regular update phase.
   */
  virtual void Update() {
  }

  /**
   * @brief Virtual function called during the late update phase.
   */
  virtual void LateUpdate() {
  }

  /**
   * @brief Lifecycle callback invoked upon creation of the component.
   */
  virtual void OnCreate() {
  }

  /**
   * @brief Lifecycle callback invoked when the component starts.
   */
  virtual void Start() {
  }

  /**
   * @brief Lifecycle callback invoked when the component is enabled.
   */
  virtual void OnEnable() {
  }

  /**
   * @brief Lifecycle callback invoked when the component is disabled.
   */
  virtual void OnDisable() {
  }

  /**
   * @brief Lifecycle callback invoked when the associated entity is enabled.
   */
  virtual void OnEntityEnable() {
  }

  /**
   * @brief Lifecycle callback invoked when the associated entity is disabled.
   */
  virtual void OnEntityDisable() {
  }

  /**
   * @brief Lifecycle callback invoked when the component is destroyed.
   */
  virtual void OnDestroy() {
  }

  /**
   * @brief Perform additional actions after the component has been cloned.
   *
   * This function is invoked with the cloned target component, allowing for additional
   * setup or initialization steps after the cloning process.
   *
   * @param target The cloned component instance.
   */
  virtual void PostCloneAction(const std::shared_ptr<IPrivateComponent>& target) {
  }
};

/**
 * @struct PrivateComponentElement
 * @brief Represents a single element in the private component storage system.
 *
 * This structure is used for managing private components associated
 * with specific entity instances.
 */
struct PrivateComponentElement {
  size_t type_index;                                          ///< Type index of the private component.
  std::shared_ptr<IPrivateComponent> private_component_data;  ///< Shared pointer to the private component data.

  /**
   * @brief Default constructor for `PrivateComponentElement`.
   */
  PrivateComponentElement() = default;

  /**
   * @brief Constructs a `PrivateComponentElement` with the given parameters.
   *
   * @param id The type index of the private component.
   * @param data Shared pointer to the private component data.
   * @param owner The owner entity of the private component.
   * @param scene Shared pointer to the scene this component is part of.
   */
  PrivateComponentElement(size_t id, const std::shared_ptr<IPrivateComponent>& data, const Entity& owner,
                          const std::shared_ptr<Scene>& scene);

  /**
   * @brief Reset the owner of this private component to a new entity.
   *
   * Updates the owner to a new entity and associates it with the given scene.
   *
   * @param new_owner The new entity that will own this component.
   * @param scene Shared pointer to the new scene the component is part of.
   */
  void ResetOwner(const Entity& new_owner, const std::shared_ptr<Scene>& scene) const;
};

}  // namespace evo_engine
