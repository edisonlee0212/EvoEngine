
#pragma once
#include <deque>
#include <functional>
#include <future>
#include <set>
#include <stack>
#include <vector>

#include "IHandle.hpp"
#include "Serialization.hpp"

namespace evo_engine {
class EditorLayer;
class File;
class Folder;
class IAsset;
class ProjectContentBrowserPanel;

/**
 * @class AssetManager
 * @brief Manages the loading, retrieval, and creation of assets in the engine.
 */
class AssetManager {
 public:
  enum class AssetLoadState {
    Discovered,
    Queued,
    LoadingCpu,
    WaitingForFinalize,
    GpuPending,
    Loaded,
    Failed,
    Cancelled
  };

  struct AssetLoadSnapshot {
    size_t total = 0;
    size_t completed = 0;
    size_t failed = 0;
    size_t cancelled = 0;
    size_t queued = 0;
    size_t loading_cpu = 0;
    size_t waiting_for_finalize = 0;
    size_t gpu_pending = 0;
    Handle active_asset_handle = 0;
    AssetLoadState active_state = AssetLoadState::Discovered;
    std::string active_asset_name;
    std::string message;

    [[nodiscard]] bool Active() const;
  };

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

  /**
   * @brief Enqueues an asset load request and returns the shared future without blocking the caller.
   */
  [[nodiscard]] static std::shared_future<std::shared_ptr<IAsset>> RequestAssetLoad(const Handle& asset_handle);

  /**
   * @brief Enqueues a batch of asset loads for project/UI progress tracking.
   */
  [[nodiscard]] static std::vector<std::shared_future<std::shared_ptr<IAsset>>> RequestAssetLoads(
      const std::set<Handle>& asset_handles);

  /**
   * @brief Returns a thread-safe snapshot of the current asset-service loading progress.
   */
  [[nodiscard]] static AssetLoadSnapshot GetAssetLoadSnapshot();

  /**
   * @brief Runs queued main-thread asset construction/finalization tasks.
   */
  static size_t ExecuteMainThreadAssetTasks(size_t max_task_size = 1);

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
    struct AssetLoadingRecord {
      std::shared_future<std::shared_ptr<IAsset>> future;  ///< Shared result for all waiters on the asset load.
      std::weak_ptr<IAsset> loading_asset;                 ///< Partially constructed asset for same-thread recursion.
      std::thread::id owner_thread_id;                     ///< Thread currently performing the load.
      bool async = false;                                  ///< Whether this record was created by async access.
      bool allow_same_thread_partial_access =
          false;  ///< True only while a synchronous Load() call may recursively request itself.
      AssetLoadState state = AssetLoadState::Discovered;
      std::string asset_name;
      std::string message;
    };

    std::mutex asset_registry_mutex;  ///< Mutex for synchronizing access to the asset registry.
    std::unordered_map<Handle, std::weak_ptr<IAsset>> assets_;       ///< Map storing assets by their handles.
    std::unordered_map<Handle, AssetLoadingRecord> loading_assets_;  ///< In-flight loads by asset handle.
    std::deque<std::function<void()>> main_thread_asset_tasks_;      ///< Asset tasks that must run on main.
    AssetLoadSnapshot load_snapshot_;                                ///< Current asset-service progress snapshot.

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
  static void DrawAssetInspectorContent(const std::shared_ptr<EditorLayer>& editor_layer);
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
  friend class ProjectContentBrowserPanel;

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
   * @brief Performs the actual uncached asset creation and load work for a handle.
   */
  static std::shared_ptr<IAsset> LoadAssetImpl(const Handle& asset_handle);

  /**
   * @brief Starts service-thread loading for an asset request.
   */
  static void StartAssetServiceLoadImpl(const Handle& asset_handle,
                                        const std::shared_ptr<std::promise<std::shared_ptr<IAsset>>>& promise);

  /**
   * @brief Waits for a load future while allowing the main thread to run queued finalization tasks.
   */
  static std::shared_ptr<IAsset> WaitForAssetLoadFutureImpl(
      const std::shared_future<std::shared_ptr<IAsset>>& asset_future);

  /**
   * @brief Gets or creates the shared in-flight load for an asset handle.
   */
  static std::shared_future<std::shared_ptr<IAsset>> GetOrCreateAssetLoadFutureImpl(const Handle& asset_handle,
                                                                                    bool async);

  static void ScheduleMainThreadAssetTaskImpl(const std::function<void()>& action);

  static void ResetAssetLoadSnapshotImpl(size_t total);

  static void UpdateAssetLoadStateImpl(const Handle& asset_handle, AssetLoadState state, const std::string& message);

  /**
   * @brief Publishes the partially constructed asset for same-thread recursive access during Load().
   */
  static void SetLoadingAssetImpl(const Handle& asset_handle, const std::shared_ptr<IAsset>& asset,
                                  bool allow_same_thread_partial_access);

  /**
   * @brief Clears the in-flight load record for an asset handle.
   */
  static void FinishAssetLoadingImpl(const Handle& asset_handle);

  /**
   * @brief Retrieves a future object for asynchronously accessing an asset given its handle.
   * @param asset_handle The handle associated with the asset.
   * @return A shared future containing a shared pointer to the requested asset.
   */
  static std::shared_future<std::shared_ptr<IAsset>> GetAssetFutureImpl(const Handle& asset_handle);
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
    return std::dynamic_pointer_cast<T>(GetAsset(type_name, asset_handle));
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
    auto asset_future = GetAssetFutureImpl(asset_handle);
    return std::async(std::launch::deferred,
                      [asset_future = std::move(asset_future)]() mutable {
                        return std::dynamic_pointer_cast<T>(WaitForAssetLoadFutureImpl(asset_future));
                      })
        .share();
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
