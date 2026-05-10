
#pragma once
#include <stack>

#include "IHandle.hpp"
#include "Serialization.hpp"

namespace evo_engine {
class EditorLayer;
class File;
class Folder;
class IAsset;

/**
 * @class AssetManager
 * @brief Manages the loading, retrieval, and creation of assets in the engine.
 */
class AssetManager {
 public:
  static AssetManager& GetInstance();

 private:
 public:
  /**
   * @brief Retrieves an asset of type T corresponding to the given handle.
   * @tparam T The asset type to retrieve.
   * @param asset_handle The handle associated with the asset.
   * @return A shared pointer to the requested asset of type T.
   */
  template <typename T>
  [[nodiscard]] static std::shared_ptr<T> GetAsset(const Handle& asset_handle);

  /**
   * @brief Retrieves a future object to asynchronously access an asset of type T.
   * @tparam T The asset type to retrieve.
   * @param asset_handle The handle associated with the asset.
   * @return A shared future containing a shared pointer to the requested asset of type T.
   */
  template <typename T>
  [[nodiscard]] static std::shared_future<std::shared_ptr<T>> GetAssetFuture(const Handle& asset_handle);

  /**
   * @brief Creates a temporary asset of type T for immediate use.
   * @tparam T The type of asset to create.
   * @return A shared pointer to the created temporary asset of type T.
   */
  template <typename T>
  [[nodiscard]] static std::shared_ptr<T> CreateTemporaryAsset();
  /**
   * @brief Creates a temporary asset given its typename.
   * @param type_name The typename of the asset to create.
   * @return A shared pointer to the created temporary asset.
   */
  [[nodiscard]] static std::shared_ptr<IAsset> CreateTemporaryAsset(const std::string& type_name);

  /**
   * @brief Retrieves an asset given its typename and handle.
   * @param asset_handle The handle associated with the asset.
   * @return A shared pointer to the requested asset.
   */
  [[nodiscard]] static std::shared_ptr<IAsset> GetAsset(const Handle& asset_handle);

 private:
  /**
   * @brief Retrieves an asset given its typename and handle.
   * @param type_name The typename of the asset.
   * @param asset_handle The handle associated with the asset.
   * @return A shared pointer to the requested asset.
   */
  [[nodiscard]] static std::shared_ptr<IAsset> GetAsset(const std::string& type_name, const Handle& asset_handle);

  /**
   * @class AssetRegistry
   * @brief Internal registry for managing assets and their corresponding handles.
   */
  class AssetRegistry {
    std::mutex asset_registry_mutex;  ///< Mutex for synchronizing access to the asset registry.
    std::unordered_map<Handle, std::weak_ptr<IAsset>> assets_;  ///< Map storing assets by their handles.

    friend class AssetManager;  ///< AssetManager has access to private members of AssetRegistry.
  };

  AssetRegistry asset_registry_;  ///< The asset registry instance.
  bool initialized = false;       ///< Flag indicating if the AssetManager is initialized.

  /**
   * @brief Indicates whether assets should be displayed in the interface.
   */
  bool show_asset_inspector_ = true;
  /**
   * @brief Initializes the AssetManager.
   */
  static void Initialize();
  /**
   * @brief Displays the resource assets in the editor interface.
   * @param editor_layer The editor layer instance used for displaying assets.
   */
  static void OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);
  /**
   * @brief Cleans up resources when destroying the AssetManager.
   */
  static void OnDestroy();

  /**
   * @brief Clears all loaded assets from the asset manager.
   */
  static void Clear();
  static size_t RestoreUnknownAssets();

  friend class ProjectManager;  ///< Grants ProjectManager access to private and protected members of AssetManager.
  friend class Application;     ///< Grants Application access to private and protected members of AssetManager.
  friend class PackageManager;  ///< Grants PackageManager access to runtime asset restoration.
  friend class IAsset;          ///< Grants IAsset access to private and protected members of AssetManager.
  friend class Prefab;          ///< Grants Prefab access to private and protected members of AssetManager.
  friend class Scene;           ///< Grants Scene access to private and protected members of AssetManager.
  friend class AssetRef;        ///< Grants AssetRef access to private and protected members of AssetManager.
  friend class File;            ///< Grants File access to private and protected members of AssetManager.
  friend class Folder;          ///< Grants Folder access to private and protected members of AssetManager.
  friend class EditorLayer;     ///< Grants EditorLayer access to private and protected members of AssetManager.

  /**
   * @brief Creates a temporary asset implementation with the given typename and handle.
   * @param type_name The typename of the asset to create.
   * @param asset_handle The handle associated with the asset.
   * @return A shared pointer to the created temporary asset.
   */
  [[nodiscard]] static std::shared_ptr<IAsset> CreateTemporaryAssetImpl(const std::string& type_name,
                                                                        const Handle& asset_handle);

  /**
   * @brief Removes an asset from the AssetManager given its handle.
   * @param asset_handle The handle associated with the asset to remove.
   */
  static void RemoveAssetImpl(const Handle& asset_handle);

  /**
   * @brief Retrieves an asset implementation given its handle.
   * @param asset_handle The handle associated with the asset.
   * @return A shared pointer to the requested asset.
   */
  static std::shared_ptr<IAsset> GetAssetImpl(const Handle& asset_handle);

  /**
   * @brief Retrieves a future object for asynchronously accessing an asset given its handle.
   * @param asset_handle The handle associated with the asset.
   * @return A future containing a shared pointer to the requested asset.
   */
  static std::future<std::shared_ptr<IAsset>> GetAssetFutureImpl(const Handle& asset_handle);
};

/**
 * @brief Retrieves an asset of type T corresponding to the given handle.
 * @tparam T The asset type to retrieve.
 * @param asset_handle The handle associated with the asset.
 * @return A shared pointer to the requested asset of type T.
 */
template <typename T>
std::shared_ptr<T> AssetManager::GetAsset(const Handle& asset_handle) {
  try {
    const auto type_name = Serialization::GetSerializableTypeName<T>();
    return GetAsset(type_name, asset_handle);
  } catch (const std::exception& e) {
    EVOENGINE_ERROR(e.what());
    return {};
  }
}

/**
 * @brief Retrieves a future object to asynchronously access an asset of type T.
 * @tparam T The asset type to retrieve.
 * @param asset_handle The handle associated with the asset.
 * @return A shared future containing a shared pointer to the requested asset of type T.
 */
template <typename T>
std::shared_future<std::shared_ptr<T>> AssetManager::GetAssetFuture(const Handle& asset_handle) {
  try {
    return std::dynamic_pointer_cast<T>(GetAssetFutureImpl(asset_handle));
  } catch (const std::exception& e) {
    EVOENGINE_ERROR(e.what());
    return {};
  }
}

/**
 * @brief Creates a temporary asset of type T for immediate use.
 * @tparam T The type of asset to create.
 * @return A shared pointer to the created temporary asset of type T.
 */
template <typename T>
std::shared_ptr<T> AssetManager::CreateTemporaryAsset() {
  try {
    const auto type_name = Serialization::GetSerializableTypeName<T>();
    return std::dynamic_pointer_cast<T>(CreateTemporaryAsset(type_name));
  } catch (const std::exception& e) {
    EVOENGINE_ERROR(e.what());
    return {};
  }
}

}  // namespace evo_engine
