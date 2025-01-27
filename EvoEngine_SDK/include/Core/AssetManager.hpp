#pragma once
#include <stack>

#include "IHandle.hpp"
#include "ISingleton.hpp"
#include "Serialization.hpp"

namespace evo_engine {
class AssetManager {
  EVOENGINE_SINGLETON_INSTANCE(AssetManager)
 public:
  template <typename T>
  [[nodiscard]] static std::shared_ptr<T> GetAsset(const Handle& asset_handle);
  template <typename T>
  [[nodiscard]] static std::shared_future<std::shared_ptr<T>> GetAssetFuture(const Handle& asset_handle);
  template <typename T>
  [[nodiscard]] static std::shared_ptr<T> CreateTemporaryAsset();

 private:
  class AssetRegistry {
    std::mutex asset_registry_mutex;
    std::unordered_map<Handle, std::weak_ptr<IAsset>> assets_;
    friend class AssetManager;
  };

  AssetRegistry asset_registry_;
  bool initialized = false;
  static void Initialize();
  static void OnDestroy();
  static void Clear();
  friend class ProjectManager;
  friend class Application;
  friend class IAsset;
  friend class Prefab;
  friend class Scene;
  friend class AssetRef;
  friend class File;
  friend class Folder;
  friend class EditorLayer;
  [[nodiscard]] static std::shared_ptr<IAsset> CreateTemporaryAsset(const std::string& type_name);
  [[nodiscard]] static std::shared_ptr<IAsset> GetAsset(const std::string& type_name, const Handle& asset_handle);

  [[nodiscard]] static std::shared_ptr<IAsset> CreateTemporaryAssetImpl(const std::string& type_name,
                                                                        const Handle& asset_handle);
  static void RemoveAssetImpl(const Handle& asset_handle);
  static std::shared_ptr<IAsset> GetAssetImpl(const Handle& asset_handle);
  static std::future<std::shared_ptr<IAsset>> GetAssetFutureImpl(const Handle& asset_handle);
};

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

template <typename T>
std::shared_future<std::shared_ptr<T>> AssetManager::GetAssetFuture(const Handle& asset_handle) {
  try {
    return std::dynamic_pointer_cast<T>(GetAssetFutureImpl(asset_handle));
  } catch (const std::exception& e) {
    EVOENGINE_ERROR(e.what());
    return {};
  }
}

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