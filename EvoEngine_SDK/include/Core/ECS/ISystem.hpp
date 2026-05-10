
#pragma once
#include "AssetRef.hpp"
#include "ISerializable.hpp"

namespace evo_engine {

/**
 * @brief ThreadPool class declaration.
 */
class ThreadPool;

/**
 * @brief Scene class declaration.
 */
class Scene;

/**
 * @brief EditorLayer class declaration.
 */
class EditorLayer;

/**
 * @brief Base class for all systems in the engine. Provides functionality for
 * enabling, disabling, and managing system-related operations.
 */
class ISystem : public ISerializable {
  friend class Scene;
  friend class Entities;
  friend class Serialization;
  friend class PackageManager;
  friend class PackageRegistrar;
  /**
   * @brief Indicates whether the system is enabled.
   */
  bool enabled_;

  /**
   * @brief Priority rank of the system.
   */
  float rank_ = 0.0f;

  /**
   * @brief Indicates whether the system has been started.
   */
  bool started_ = false;

  /**
   * @brief Weak pointer to the associated Scene object.
   */
  std::weak_ptr<Scene> scene_;

 protected:
  /**
   * @brief Called when the system is enabled.
   */
  virtual void OnEnable() {
  }

  /**
   * @brief Called when the system is disabled.
   */
  virtual void OnDisable() {
  }

 public:
  /**
   * @brief Returns the Scene associated with this system.
   * @return Shared pointer to the Scene.
   */
  [[nodiscard]] std::shared_ptr<Scene> GetScene() const;

  /**
   * @brief Gets the rank of the system.
   * @return The rank value.
   */
  [[nodiscard]] float GetRank() const;

  /**
   * @brief Default constructor for ISystem.
   */
  ISystem();

  /**
   * @brief Enables the system.
   */
  void Enable();

  /**
   * @brief Disables the system.
   */
  void Disable();

  /**
   * @brief Checks if the system is enabled.
   * @return True if the system is enabled, false otherwise.
   */
  [[nodiscard]] bool Enabled() const;

  /**
   * @brief Called when the system is created.
   */
  virtual void OnCreate() {
  }

  /**
   * @brief Starts the system. Typically used to initialize resources.
   */
  virtual void Start() {
  }

  /**
   * @brief Called when the system is destroyed.
   */
  virtual void OnDestroy() {
  }

  /**
   * @brief Updates the system logic. Called every frame.
   */
  virtual void Update() {
  }

  /**
   * @brief Performs fixed-step updates. Typically called at a constant rate.
   */
  virtual void FixedUpdate() {
  }

  /**
   * @brief Performs late updates after the regular update calls.
   */
  virtual void LateUpdate() {
  }

  /**
   * @brief Allows inspection and modification of the system in the editor.
   * @param editor_layer Editor layer instance.
   * @return True if the inspection was successful, false otherwise.
   */
  virtual bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
    return false;
  }

  /**
   * @brief Collects asset references used by the system.
   * @param list Vector to store the collected asset references.
   */
  virtual void CollectAssetRef(std::vector<AssetRef>& list) {
  }

  /**
   * @brief Performs actions after the system is cloned.
   * @param target Shared pointer to the cloned ISystem instance.
   */
  virtual void PostCloneAction(const std::shared_ptr<ISystem>& target) {
  }
};

/**
 * @brief Wrapper class for managing system references.
 */
class SystemRef : public ISerializable {
  friend class Prefab;

  /**
   * @brief Optional weak pointer to the ISystem instance.
   */
  std::optional<std::weak_ptr<ISystem>> value_;

  /**
   * @brief Handle to the associated system.
   */
  Handle system_handle_ = Handle(0);

  /**
   * @brief Type name of the system.
   */
  std::string system_type_name_;

  /**
   * @brief Updates the internal state of the SystemRef object.
   * @return True if the update was successful, false otherwise.
   */
  bool Update();

 protected:
  /**
   * @brief Serializes the SystemRef to a YAML emitter.
   * @param out YAML emitter instance.
   */
  void Serialize(YAML::Emitter& out) const override {
    out << YAML::Key << "system_handle_" << YAML::Value << system_handle_;
    out << YAML::Key << "system_type_name_" << YAML::Value << system_type_name_;
  }

  /**
   * @brief Deserializes the SystemRef from a YAML node.
   * @param in YAML node containing serialized data.
   */
  void Deserialize(const YAML::Node& in) override {
    system_handle_ = Handle(in["system_handle_"].as<uint64_t>());
    system_type_name_ = in["system_type_name_"].as<std::string>();
    Update();
  }

 public:
  /**
   * @brief Default constructor for SystemRef.
   */
  SystemRef() {
    system_handle_ = Handle(0);
    system_type_name_ = "";
  }

  /**
   * @brief Constructs a SystemRef from a shared pointer to a system.
   * @tparam T The type of system.
   * @param other Shared pointer to the system instance.
   */
  template <typename T = ISystem>
  SystemRef(const std::shared_ptr<T>& other) {
    Set(other);
  }

  /**
   * @brief Assignment operator for assigning a shared pointer to the SystemRef.
   * @tparam T The type of system.
   * @param other Shared pointer to the system instance.
   * @return Reference to the updated SystemRef.
   */
  template <typename T = ISystem>
  SystemRef& operator=(const std::shared_ptr<T>& other) {
    Set(other);
    return *this;
  }

  /**
   * @brief Move assignment operator for assigning a shared pointer to the SystemRef.
   * @tparam T The type of system.
   * @param other Rvalue reference to the shared pointer of the system instance.
   * @return Reference to the updated SystemRef.
   */
  template <typename T = ISystem>
  SystemRef& operator=(std::shared_ptr<T>&& other) noexcept {
    Set(other);
    return *this;
  }

  /**
   * @brief Equality operator for comparing two SystemRef instances.
   * @param rhs The other SystemRef instance.
   * @return True if both SystemRef instances are equal, false otherwise.
   */
  bool operator==(const SystemRef& rhs) const {
    return system_handle_ == rhs.system_handle_;
  }

  /**
   * @brief Inequality operator for comparing two SystemRef instances.
   * @param rhs The other SystemRef instance.
   * @return True if both SystemRef instances are not equal, false otherwise.
   */
  bool operator!=(const SystemRef& rhs) const {
    return system_handle_ != rhs.system_handle_;
  }

  /**
   * @brief Relinks the system reference handle using a mapping.
   * @param map Mapping of old handles to new handles.
   */
  void Relink(const std::unordered_map<Handle, Handle>& map) {
    if (const auto search = map.find(system_handle_); search != map.end())
      system_handle_ = search->second;
    else
      system_handle_ = Handle(0);
    value_.reset();
  }

  /**
   * @brief Retrieves the system instance.
   * @tparam T The type of system.
   * @return Shared pointer to the system instance, or nullptr if not available.
   */
  template <typename T = ISystem>
  [[nodiscard]] std::shared_ptr<T> Get() {
    if (Update()) {
      return std::static_pointer_cast<T>(value_.value().lock());
    }
    return nullptr;
  }

  /**
   * @brief Sets the system reference to the specified system.
   * @tparam T The type of system.
   * @param target Shared pointer to the system instance.
   */
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

  /**
   * @brief Clears the system reference.
   */
  void Clear() {
    value_.reset();
    system_handle_ = Handle(0);
  }

  /**
   * @brief Retrieves the handle of the associated system.
   * @return The system handle.
   */
  [[nodiscard]] Handle GetEntityHandle() const {
    return system_handle_;
  }
};

}  // namespace evo_engine
