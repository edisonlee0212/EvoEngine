
#pragma once
#include "AssetRef.hpp"
#include "EvoEngineAPI.hpp"
#include "Serialization.hpp"

namespace evo_engine {
class Texture2D;
class Mesh;
class Cubemap;
class EnvironmentalMap;

/**
 * @class Resources
 * @brief A singleton class that manages resources within the engine.
 */
class Resources {
 public:
  static Resources& GetInstance();

 private:
 public:
  static EVOENGINE_API std::shared_ptr<Texture2D> missing_texture;
  static EVOENGINE_API std::shared_ptr<Cubemap> default_skybox;
  static EVOENGINE_API std::shared_ptr<EnvironmentalMap> default_environmental_map;

  class Primitives {
    friend class Resources;
    static void Load();
    static void OnDestroy();

   public:
    static EVOENGINE_API std::shared_ptr<Mesh> quad;
    static EVOENGINE_API std::shared_ptr<Mesh> sphere;
    static EVOENGINE_API std::shared_ptr<Mesh> cube;
    static EVOENGINE_API std::shared_ptr<Mesh> cone;
    static EVOENGINE_API std::shared_ptr<Mesh> cylinder;
    static EVOENGINE_API std::shared_ptr<Mesh> torus;
    static EVOENGINE_API std::shared_ptr<Mesh> monkey;
    static EVOENGINE_API std::shared_ptr<Mesh> capsule;
  };

 private:
  /**
   * @brief Stores the handle for the next resource to be created.
   */
  Handle current_max_handle_ = Handle(1);

  /**
   * @brief A map of resources grouped by type name and their corresponding handles.
   */
  std::unordered_map<std::string, std::unordered_map<Handle, std::shared_ptr<IAsset>>> typed_resources_;

  /**
   * @brief A map of resource handles to their corresponding resource objects.
   */
  std::unordered_map<Handle, std::shared_ptr<IAsset>> resources_;

  /**
   * @brief Loads primitive resources into the engine (implementation-specific).
   */
  static void LoadPrimitives();

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

  /**
   * @brief Creates a new resource of type T with the specified name.
   * @tparam T The type of the resource to be created.
   * @return A shared pointer to the newly created resource.
   */
  template <class T>
  static std::shared_ptr<T> CreateResource();

  static std::shared_ptr<Texture2D> default_environmental_map_texture;
  static std::shared_ptr<Texture2D> default_skybox_texture;

  /** @cond DOXYGEN_SHOULD_SKIP_THIS */
  friend class AssetManager;
  friend class EditorLayer;
  friend class Scene;
  friend class Cubemap;
  friend class LightProbe;
  friend class ReflectionProbe;
  friend class Bloom;
  friend class PostProcessingStack;
  friend class ScreenSpaceAmbientOcclusion;
  friend class ScreenSpaceReflection;
  friend class RenderLayer;
  friend class WindowLayer;
  friend class Prefab;
  /** @endcond */
  static std::shared_ptr<Mesh> texture_pass_through_quad;
  static std::shared_ptr<Mesh> rendering_cube;

  bool show_resources_ = false;

  /**
   * @brief Displays the resource assets in the editor interface.
   * @param editor_layer The editor layer instance used for displaying assets.
   */
  static void OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);

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
 * @return A shared pointer to the created resource.
 */
std::shared_ptr<T> Resources::CreateResource() {
  auto& resources = GetInstance();
  auto type_name = Serialization::GetSerializableTypeName<T>();
  const auto handle = resources.GenerateNewHandle();
  auto ret_val = std::dynamic_pointer_cast<IAsset>(Serialization::ProduceSerializable<T>());
  ret_val->self_ = ret_val;
  ret_val->handle_ = handle;

  resources.typed_resources_[type_name][handle] = ret_val;
  resources.resources_[handle] = ret_val;
  ret_val->OnCreate();
  return std::dynamic_pointer_cast<T>(ret_val);
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
