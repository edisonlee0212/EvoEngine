
#pragma once
#include "AssetRef.hpp"
#include "Serialization.hpp"

namespace evo_engine {

/**
 * @class Resources
 * @brief A singleton class that manages resources within the engine.
 */
class Resources {
  EVOENGINE_SINGLETON_INSTANCE(Resources)

  /**
   * @brief Stores the handle for the next resource to be created.
   */
  Handle current_max_handle_ = Handle(1);

  /**
   * @brief A map of resources grouped by type name and their corresponding handles.
   */
  std::unordered_map<std::string, std::unordered_map<Handle, std::shared_ptr<IAsset>>> typed_resources_;

  /**
   * @brief A map of resources identified by their associated names.
   */
  std::unordered_map<std::string, std::shared_ptr<IAsset>> named_resources_;

  /**
   * @brief A map of resource handles to their associated names.
   */
  std::unordered_map<Handle, std::string> resource_names_;

  /**
   * @brief A map of resource handles to their corresponding resource objects.
   */
  std::unordered_map<Handle, std::shared_ptr<IAsset>> resources_;

  /**
   * @brief Loads primitive resources into the engine (implementation-specific).
   */
  static void LoadPrimitives();

  /**
   * @brief Indicates whether assets should be displayed in the interface.
   */
  bool show_assets_ = true;

  /**
   * @brief Initializes the resources system.
   */
  static void Initialize();

  /**
   * @brief Generates a new unique handle for a resource.
   * @return A new unique Handle.
   */
  [[nodiscard]] Handle GenerateNewHandle();

  friend class ProjectManager;
  friend class Application;

 public:
  /**
   * @brief Displays the resource assets in the editor interface.
   * @param editor_layer The editor layer instance used for displaying assets.
   */
  static void OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);

  /**
   * @brief Creates a new resource of type T with the specified name.
   * @tparam T The type of the resource to be created.
   * @param name The name of the resource.
   * @return A shared pointer to the newly created resource.
   */
  template <class T>
  static std::shared_ptr<T> CreateResource(const std::string& name);

  /**
   * @brief Checks if a resource exists that matches the specified handle.
   * @param handle The handle to check for an associated resource.
   * @return True if a resource exists, false otherwise.
   */
  [[nodiscard]] static bool IsResource(const Handle& handle);

  /**
   * @brief Checks if a resource exists that matches the specified asset.
   * @param target The asset to check for an associated resource.
   * @return True if a resource exists, false otherwise.
   */
  [[nodiscard]] static bool IsResource(const std::shared_ptr<IAsset>& target);

  /**
   * @brief Checks if a resource exists that matches the specified asset reference.
   * @param target The asset reference to check for an associated resource.
   * @return True if a resource exists, false otherwise.
   */
  [[nodiscard]] static bool IsResource(const AssetRef& target);

  /**
   * @brief Tries to retrieve a resource by its name.
   * @tparam T The type of the resource to retrieve.
   * @param name The name of the resource.
   * @return A shared pointer to the resource if found, otherwise an empty shared pointer.
   */
  template <class T>
  [[nodiscard]] static std::shared_ptr<T> TryGetResource(const std::string& name);

  /**
   * @brief Tries to retrieve a resource by its handle.
   * @tparam T The type of the resource to retrieve.
   * @param handle The handle of the resource.
   * @return A shared pointer to the resource if found, otherwise an empty shared pointer.
   */
  template <class T>
  [[nodiscard]] static std::shared_ptr<T> TryGetResource(const Handle& handle);

  /**
   * @brief Cleans up and destroys all resources.
   */
  static void OnDestroy();
};

template <class T>
/**
 * @brief Creates a new resource and registers it in the resource system.
 * @tparam T The type of the resource to be created.
 * @param name The name to assign the new resource.
 * @return A shared pointer to the created resource.
 */
std::shared_ptr<T> Resources::CreateResource(const std::string& name) {
  auto& resources = GetInstance();
  assert(resources.named_resources_.find(name) == resources.named_resources_.end());
  auto type_name = Serialization::GetSerializableTypeName<T>();
  const auto handle = resources.GenerateNewHandle();
  auto ret_val = std::dynamic_pointer_cast<IAsset>(Serialization::ProduceSerializable<T>());
  ret_val->self_ = ret_val;
  ret_val->handle_ = handle;

  resources.resource_names_[handle] = name;
  resources.typed_resources_[type_name][handle] = ret_val;
  resources.named_resources_[name] = ret_val;
  resources.resources_[handle] = ret_val;
  ret_val->OnCreate();
  return std::dynamic_pointer_cast<T>(ret_val);
}

template <class T>
/**
 * @brief Attempts to retrieve an existing resource by its name.
 * @tparam T The type of the resource to retrieve.
 * @param name The name of the resource.
 * @return A shared pointer to the matching resource, or an empty shared pointer if not found.
 */
std::shared_ptr<T> Resources::TryGetResource(const std::string& name) {
  const auto& resources = GetInstance();
  if (const auto search = resources.named_resources_.find(name); search != resources.named_resources_.end())
    return std::dynamic_pointer_cast<T>(search->second);
  return {};
}

template <class T>
/**
 * @brief Attempts to retrieve an existing resource by its handle.
 * @tparam T The type of the resource to retrieve.
 * @param handle The handle of the resource.
 * @return A shared pointer to the matching resource, or an empty shared pointer if not found.
 */
std::shared_ptr<T> Resources::TryGetResource(const Handle& handle) {
  const auto& resources = GetInstance();
  if (const auto search = resources.resources_.find(handle); search != resources.resources_.end())
    return std::dynamic_pointer_cast<T>(search->second);
  return {};
}

}  // namespace evo_engine
