#pragma once
#include "AssetRef.hpp"
#include "ISerializable.hpp"
namespace evo_engine {
class ThreadPool;
class Scene;
class EditorLayer;
class ISystem : public ISerializable {
  friend class Scene;
  friend class Entities;
  friend class Serialization;
  bool enabled_;
  float rank_ = 0.0f;
  bool started_ = false;
  std::weak_ptr<Scene> scene_;

 protected:
  virtual void OnEnable() {
  }
  virtual void OnDisable() {
  }

 public:
  [[nodiscard]] std::shared_ptr<Scene> GetScene() const;
  [[nodiscard]] float GetRank() const;
  ISystem();
  void Enable();
  void Disable();
  [[nodiscard]] bool Enabled() const;
  virtual void OnCreate() {
  }
  virtual void Start() {
  }
  virtual void OnDestroy() {
  }
  virtual void Update() {
  }
  virtual void FixedUpdate() {
  }
  virtual void LateUpdate() {
  }
  // Will only exec when editor is enabled, and no matter application is running or not.
  virtual bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
    return false;
  }
  virtual void CollectAssetRef(std::vector<AssetRef>& list) {
  }
  virtual void PostCloneAction(const std::shared_ptr<ISystem>& target) {
  }
};

class SystemRef : public ISerializable {
  friend class Prefab;
  std::optional<std::weak_ptr<ISystem>> value_;
  Handle system_handle_ = Handle(0);
  std::string system_type_name_;
  bool Update();

 protected:
  void Serialize(YAML::Emitter& out) const override {
    out << YAML::Key << "system_handle_" << YAML::Value << system_handle_;
    out << YAML::Key << "system_type_name_" << YAML::Value << system_type_name_;
  }
  void Deserialize(const YAML::Node& in) override {
    system_handle_ = Handle(in["system_handle_"].as<uint64_t>());
    system_type_name_ = in["system_type_name_"].as<std::string>();
    Update();
  }

 public:
  SystemRef() {
    system_handle_ = Handle(0);
    system_type_name_ = "";
  }
  template <typename T = ISystem>
  SystemRef(const std::shared_ptr<T>& other) {
    Set(other);
  }
  template <typename T = ISystem>
  SystemRef& operator=(const std::shared_ptr<T>& other) {
    Set(other);
    return *this;
  }
  template <typename T = ISystem>
  SystemRef& operator=(std::shared_ptr<T>&& other) noexcept {
    Set(other);
    return *this;
  }

  bool operator==(const SystemRef& rhs) const {
    return system_handle_ == rhs.system_handle_;
  }
  bool operator!=(const SystemRef& rhs) const {
    return system_handle_ != rhs.system_handle_;
  }

  void Relink(const std::unordered_map<Handle, Handle>& map) {
    if (const auto search = map.find(system_handle_); search != map.end())
      system_handle_ = search->second;
    else
      system_handle_ = Handle(0);
    value_.reset();
  };

  template <typename T = ISystem>
  [[nodiscard]] std::shared_ptr<T> Get() {
    if (Update()) {
      return std::static_pointer_cast<T>(value_.value().lock());
    }
    return nullptr;
  }
  template <typename T = ISystem>
  void Set(const std::shared_ptr<T>& target) {
    if (target) {
      auto system = std::dynamic_pointer_cast<ISystem>(target);
      system_type_name_ = system->GetTypeName();
      system_handle_ = system->GetHandle();
      value_ = system;
    } else {
      system_handle_ = Handle(0);
      value_.reset();
    }
  }

  void Clear() {
    value_.reset();
    system_handle_ = Handle(0);
  }

  [[nodiscard]] Handle GetEntityHandle() const {
    return system_handle_;
  }
};
}  // namespace evo_engine