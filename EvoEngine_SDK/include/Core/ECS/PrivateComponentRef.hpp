#pragma once
#include "IPrivateComponent.hpp"
namespace evo_engine {
class PrivateComponentRef final : public ISerializable {
  friend class Prefab;
  friend class Scene;
  std::weak_ptr<IPrivateComponent> value_;
  Handle entity_handle_ = Handle(0);
  std::weak_ptr<Scene> scene_;
  std::string private_component_type_name_{};
  bool Update();

 public:
  void Serialize(YAML::Emitter& out) const override {
    out << YAML::Key << "entity_handle_" << YAML::Value << entity_handle_;
    out << YAML::Key << "private_component_type_name_" << YAML::Value << private_component_type_name_;
  }

  void Deserialize(const YAML::Node& in) override {
    entity_handle_ = Handle(in["entity_handle_"].as<uint64_t>());
    private_component_type_name_ = in["private_component_type_name_"].as<std::string>();
    scene_.reset();
  }

  void Deserialize(const YAML::Node& in, const std::shared_ptr<Scene>& scene) {
    entity_handle_ = Handle(in["entity_handle_"].as<uint64_t>());
    private_component_type_name_ = in["private_component_type_name_"].as<std::string>();
    scene_ = scene;
  }

  PrivateComponentRef() {
    entity_handle_ = Handle(0);
    private_component_type_name_ = "";
    scene_.reset();
  }

  template <typename T = IPrivateComponent>
  PrivateComponentRef(const std::shared_ptr<T>& other) {
    Set(other);
  }
  template <typename T = IPrivateComponent>
  PrivateComponentRef& operator=(const std::shared_ptr<T>& other) {
    Set(other);
    return *this;
  }
  template <typename T = IPrivateComponent>
  PrivateComponentRef& operator=(std::shared_ptr<T>&& other) noexcept {
    Set(other);
    return *this;
  }

  void Relink(const std::unordered_map<Handle, Handle>& map, const std::shared_ptr<Scene>& scene) {
    if (const auto search = map.find(entity_handle_); search != map.end()) {
      entity_handle_ = search->second;
      value_.reset();
      scene_ = scene;
    } else
      Clear();
  }

  void ResetScene(const std::shared_ptr<Scene>& scene) {
    value_.reset();
    scene_ = scene;
  }

  template <typename T = IPrivateComponent>
  [[nodiscard]] std::shared_ptr<T> Get() {
    if (Update()) {
      return std::dynamic_pointer_cast<T>(value_.lock());
    }
    return nullptr;
  }
  template <typename T = IPrivateComponent>
  void Set(const std::shared_ptr<T>& target) {
    if (target) {
      auto private_component = std::dynamic_pointer_cast<IPrivateComponent>(target);
      scene_ = private_component->GetScene();
      private_component_type_name_ = private_component->GetTypeName();
      entity_handle_ = private_component->GetScene()->GetEntityHandle(private_component->GetOwner());
      value_ = private_component;
      handle_ = private_component->GetHandle();
    } else {
      Clear();
    }
  }

  void Clear();

  [[nodiscard]] Handle GetEntityHandle() const {
    return entity_handle_;
  }

  void Load(const std::string& name, const YAML::Node& in, const std::shared_ptr<Scene>& scene) {
    if (in[name])
      Deserialize(in[name], scene);
  }
};
}  // namespace evo_engine