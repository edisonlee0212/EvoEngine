
#pragma once
#include "IPrivateComponent.hpp"

namespace evo_engine {

/**
 * @class PrivateComponentRef
 * @brief A reference to a private component, managing serialization, deserialization,
 *        and dynamic linking to scenes and entities.
 */
class PrivateComponentRef final : public ISerializable {
  friend class Prefab;  ///< Allow Prefab to access private members.
  friend class Scene;   ///< Allow Scene to access private members.

  std::weak_ptr<IPrivateComponent> value_;     ///< Weak pointer to the private component.
  Handle entity_handle_ = Handle(0);           ///< Handle to the associated entity.
  std::weak_ptr<Scene> scene_;                 ///< Weak pointer to the associated scene.
  std::string private_component_type_name_{};  ///< Type name of the private component.

  /**
   * @brief Updates the internal state of the reference.
   * @return True if the update was successful, false otherwise.
   */
  bool Update();

 public:
  /**
   * @brief Serializes the private component reference to a YAML emitter.
   * @param[out] out The YAML emitter to write to.
   */
  void Serialize(YAML::Emitter& out) const override {
    out << YAML::Key << "entity_handle_" << YAML::Value << entity_handle_;
    out << YAML::Key << "private_component_type_name_" << YAML::Value << private_component_type_name_;
  }

  /**
   * @brief Deserializes the private component reference from a YAML node.
   * @param[in] in The YAML node to read from.
   */
  void Deserialize(const YAML::Node& in) override {
    entity_handle_ = Handle(in["entity_handle_"].as<uint64_t>());
    private_component_type_name_ = in["private_component_type_name_"].as<std::string>();
    scene_.reset();
  }

  /**
   * @brief Deserializes the private component reference with an associated scene.
   * @param[in] in The YAML node to read from.
   * @param[in] scene The scene to associate with this reference.
   */
  void Deserialize(const YAML::Node& in, const std::shared_ptr<Scene>& scene) {
    entity_handle_ = Handle(in["entity_handle_"].as<uint64_t>());
    private_component_type_name_ = in["private_component_type_name_"].as<std::string>();
    scene_ = scene;
  }

  /**
   * @brief Default constructor. Initializes the reference with default values.
   */
  PrivateComponentRef() {
    entity_handle_ = Handle(0);
    private_component_type_name_ = "";
    scene_.reset();
  }

  /**
   * @brief Constructor that initializes the reference from a shared pointer to a private component.
   * @tparam T The type of the private component (must derive from IPrivateComponent).
   * @param[in] other The shared pointer to the private component.
   */
  template <typename T = IPrivateComponent>
  PrivateComponentRef(const std::shared_ptr<T>& other) {
    Set(other);
  }

  /**
   * @brief Assignment operator from a shared pointer to a private component.
   * @tparam T The type of the private component (must derive from IPrivateComponent).
   * @param[in] other The shared pointer to the private component.
   * @return Reference to this PrivateComponentRef after assignment.
   */
  template <typename T = IPrivateComponent>
  PrivateComponentRef& operator=(const std::shared_ptr<T>& other) {
    Set(other);
    return *this;
  }

  /**
   * @brief Move assignment operator from a shared pointer to a private component.
   * @tparam T The type of the private component (must derive from IPrivateComponent).
   * @param[in] other The rvalue shared pointer to the private component.
   * @return Reference to this PrivateComponentRef after assignment.
   */
  template <typename T = IPrivateComponent>
  PrivateComponentRef& operator=(std::shared_ptr<T>&& other) noexcept {
    Set(other);
    return *this;
  }

  /**
   * @brief Relinks the reference based on a mapping of entity handles and a new scene.
   * @param[in] map A mapping from old entity handles to new handles.
   * @param[in] scene The new scene to associate with this reference.
   */
  void Relink(const std::unordered_map<Handle, Handle>& map, const std::shared_ptr<Scene>& scene) {
    if (const auto search = map.find(entity_handle_); search != map.end()) {
      entity_handle_ = search->second;
      value_.reset();
      scene_ = scene;
    } else
      Clear();
  }

  /**
   * @brief Resets the scene associated with this reference.
   * @param[in] scene The new scene to associate.
   */
  void ResetScene(const std::shared_ptr<Scene>& scene) {
    value_.reset();
    scene_ = scene;
  }

  /**
   * @brief Retrieves the private component referenced by this object.
   * @tparam T The type of the private component to retrieve (must derive from IPrivateComponent).
   * @return Shared pointer to the private component, or nullptr if not available.
   */
  template <typename T = IPrivateComponent>
  [[nodiscard]] std::shared_ptr<T> Get() {
    if (Update()) {
      return std::dynamic_pointer_cast<T>(value_.lock());
    }
    return nullptr;
  }

  /**
   * @brief Sets this reference to point to the specified private component.
   * @tparam T The type of the private component to set (must derive from IPrivateComponent).
   * @param[in] target The shared pointer to the private component.
   */
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

  /**
   * @brief Clears the reference to the private component.
   */
  void Clear();

  /**
   * @brief Gets the entity handle associated with this reference.
   * @return The handle to the associated entity.
   */
  [[nodiscard]] Handle GetEntityHandle() const {
    return entity_handle_;
  }

  /**
   * @brief Loads and deserializes the private component reference from a YAML node.
   * @param[in] name The key name in the YAML node.
   * @param[in] in The YAML node to read from.
   * @param[in] scene The scene to associate with this reference.
   */
  void Load(const std::string& name, const YAML::Node& in, const std::shared_ptr<Scene>& scene) {
    if (in[name])
      Deserialize(in[name], scene);
  }
};
}  // namespace evo_engine
