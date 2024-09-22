#pragma once
#include "AssetRef.hpp"
#include "Entity.hpp"

namespace evo_engine {
class EditorLayer;
class IPrivateComponent : public ISerializable {
  friend class Entities;

  friend class EditorLayer;
  friend struct PrivateComponentElement;
  friend class PrivateComponentStorage;
  friend class Serialization;
  friend class Scene;
  friend class Prefab;
  friend struct EntityMetadata;
  bool enabled_ = true;
  Entity owner_ = Entity();
  bool started_ = false;
  size_t version_ = 0;
  std::weak_ptr<Scene> scene_;

 public:
  /**
   * \brief Return the scene this component belongs to.
   * \return The scene this component belongs to.
   */
  [[nodiscard]] std::shared_ptr<Scene> GetScene() const;
  /**
   * \brief Get the owner.
   * \return The entity that contains this component.
   */
  [[nodiscard]] Entity GetOwner() const;
  /**
   * \brief Get the version of current component.
   * \return The version of current component.
   */
  [[nodiscard]] size_t GetVersion() const;
  /**
   * \brief Enable/Disable this component. Disabled component will not be updated/fixupdated.
   * \param value Target property.
   */
  void SetEnabled(const bool& value);
  /**
   * \brief Get current enabled status.
   * \return If the current component is enabled.
   */
  [[nodiscard]] bool IsEnabled() const;
  [[nodiscard]] bool Started() const;
  virtual bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
    return false;
  }
  virtual void FixedUpdate() {
  }
  virtual void Update() {
  }
  virtual void LateUpdate() {
  }

  virtual void OnCreate() {
  }
  virtual void Start() {
  }
  virtual void OnEnable() {
  }
  virtual void OnDisable() {
  }
  virtual void OnEntityEnable() {
  }
  virtual void OnEntityDisable() {
  }
  virtual void OnDestroy() {
  }
  /**
   * \brief Must set this up to keep track of AssetRef during serialization/deserialization.
   * \param list List of collected AssetRef. You must add all AssetRef members!.
   */
  virtual void CollectAssetRef(std::vector<AssetRef>& list) {
  }

  /**
   * \brief Must set this up to map EntityRef members to new scene during serialization/deserialization/prefab initialization.
   * \param map Map of original saved owner to actual owner.
   * \param scene Target scene.
   */
  virtual void Relink(const std::unordered_map<Handle, Handle>& map, const std::shared_ptr<Scene>& scene) {
  }
  virtual void PostCloneAction(const std::shared_ptr<IPrivateComponent>& target) {
  }
};
struct PrivateComponentElement {
  size_t type_index;
  std::shared_ptr<IPrivateComponent> private_component_data;
  PrivateComponentElement() = default;
  PrivateComponentElement(size_t id, const std::shared_ptr<IPrivateComponent>& data, const Entity& owner,
                          const std::shared_ptr<Scene>& scene);
  void ResetOwner(const Entity& new_owner, const std::shared_ptr<Scene>& scene) const;
};

}  // namespace evo_engine