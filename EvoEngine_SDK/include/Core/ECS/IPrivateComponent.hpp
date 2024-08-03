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
  [[nodiscard]] std::shared_ptr<Scene> GetScene() const;
  [[nodiscard]] Entity GetOwner() const;
  [[nodiscard]] size_t GetVersion() const;
  void SetEnabled(const bool& value);
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

  virtual void CollectAssetRef(std::vector<AssetRef>& list) {
  }
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