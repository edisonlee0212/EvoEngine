#pragma once
#include "Application.hpp"
#include "NativeBuildIdentity.hpp"
#include "Profiler.hpp"
#include "Serialization.hpp"

namespace evo_engine {
constexpr uint32_t EVOENGINE_PACKAGE_API_VERSION = 3;

struct PackageDescriptor {
  uint32_t api_version = EVOENGINE_PACKAGE_API_VERSION;
  const char* name = nullptr;
  const char* version = nullptr;
  const char* description = nullptr;
  NativeBuildIdentity build_identity{};
  const char* package_source_id = nullptr;
};

class EVOENGINE_API PackageManager;

class EVOENGINE_API PackageRegistrar {
  friend class PackageManager;

  std::string package_name_;
  std::vector<std::string>* registered_private_component_names_ = nullptr;
  std::vector<std::string>* registered_asset_names_ = nullptr;
  std::vector<std::string>* registered_data_component_names_ = nullptr;
  std::vector<std::string>* registered_system_names_ = nullptr;
  std::vector<std::string>* registered_layer_names_ = nullptr;

  PackageRegistrar(std::string package_name, std::vector<std::string>& registered_private_component_names,
                   std::vector<std::string>& registered_asset_names,
                   std::vector<std::string>& registered_data_component_names,
                   std::vector<std::string>& registered_system_names, std::vector<std::string>& registered_layer_names);

  static std::shared_ptr<ISerializable> AdoptObject(ISerializable* object, const std::string& package_name);

 public:
  template <typename T>
  bool RegisterPrivateComponent(const std::string& name);

  template <typename T>
  bool RegisterAsset(const std::string& name, const std::vector<std::string>& extensions);

  template <typename T>
  bool RegisterDataComponent(const std::string& name);

  template <typename T>
  bool RegisterSystem(const std::string& name);

  template <typename T>
  bool RegisterLayer(const std::string& name);

  [[nodiscard]] ProfilerItemHandle RegisterProfilerItem(const ProfilerItemDescriptor& descriptor);
};

using EvoEnginePackageGetDescriptorFn = const PackageDescriptor* (*)();
using EvoEnginePackageRegisterTypesFn = bool (*)(PackageRegistrar*);
using EvoEnginePackageLoadFn = bool (*)(PackageRegistrar*);
using EvoEnginePackageUnloadFn = void (*)(PackageRegistrar*);

#if defined(_WIN32)
#  define EVOENGINE_PACKAGE_EXPORT extern "C" __declspec(dllexport)
#else
#  define EVOENGINE_PACKAGE_EXPORT extern "C" __attribute__((visibility("default")))
#endif

struct LoadedPackageInfo {
  const void* module_identity = nullptr;
  std::string name;
  std::string version;
  std::string description;
  std::string package_source_id;
  std::vector<std::string> dependencies;
  std::filesystem::path original_path;
  std::filesystem::path loaded_path;
  std::filesystem::path manifest_path;
  std::vector<std::string> private_component_types;
  std::vector<std::string> asset_types;
  std::vector<std::string> data_component_types;
  std::vector<std::string> system_types;
  std::vector<std::string> layer_types;
  size_t live_object_count = 0;
  bool ready = true;
};

struct PackageLifecycleCallbacks {
  std::function<bool(const LoadedPackageInfo&)> validate;
  std::function<bool(const LoadedPackageInfo&)> activate;
  std::function<bool(const LoadedPackageInfo&)> prepare_unload;
  std::function<bool(const LoadedPackageInfo&)> deactivate;
  std::function<void()> shutdown;
};

struct AvailablePackageInfo {
  std::string name;
  std::string version;
  std::string description;
  std::vector<std::string> dependencies;
  std::filesystem::path manifest_path;
  std::filesystem::path library_path;
  bool library_exists = false;
  bool loaded = false;
};

class EVOENGINE_API PackageManager final {
 public:
  static PackageManager& GetInstance();

 private:
  friend class PackageRegistrar;
  class Mutation;

  struct LoadedPackage {
    LoadedPackageInfo info;
    std::shared_ptr<void> library;
    EvoEnginePackageUnloadFn unload = nullptr;
  };

  struct PackageManifest {
    std::string name;
    std::string library;
    std::string version;
    std::string description;
    std::string sdk_source_id;
    std::string package_source_id;
    std::string compiler_id;
    std::string compiler_version;
    std::string configuration;
    std::string platform;
    std::string architecture;
    std::string library_sha256;
    bool with_editor = false;
    std::vector<std::string> dependencies;
    std::filesystem::path manifest_path;
    std::filesystem::path library_path;
  };

  std::mutex mutex_;
  std::vector<std::filesystem::path> search_paths_;
  std::unordered_map<std::string, PackageManifest> package_manifests_;
  std::unordered_map<std::string, LoadedPackage> loaded_packages_;
  std::unordered_map<std::string, size_t> live_object_counts_;
  uint64_t next_type_cleanup_id_ = 0;
  std::map<uint64_t, std::function<void(const std::string&)>> type_cleanup_callbacks_;
  PackageLifecycleCallbacks lifecycle_callbacks_;
  std::unordered_set<std::string> active_mutations_;
  bool shutting_down_lifecycle_ = false;
  static void NotifyTypeCleanup(const std::string& package_name);

  static bool IsRuntimeBusy();
  static std::vector<std::filesystem::path> BuildDefaultSearchPaths();
  static bool ReadManifest(const std::filesystem::path& manifest_path, PackageManifest& manifest);
  static bool ValidateManifestCompatibility(const PackageManifest& manifest);
  static bool ValidateDescriptorCompatibility(const PackageDescriptor& descriptor, const PackageManifest* manifest);
  static void RefreshManifests();
  static bool LoadManifestWithDependencies(const std::string& package_name, std::vector<std::string>& loading_stack);
  static bool HasLoadedDependents(const std::string& package_name, std::string* dependent_name = nullptr);
  static bool HasLivePrivateComponentOwners(const std::vector<size_t>& type_ids);
  static void ClearPrivateComponentPools(const std::vector<size_t>& type_ids);
  static void RestoreUnknownRuntimeTypes();

 public:
  static void Initialize(const std::vector<std::filesystem::path>& package_search_paths = {},
                         const std::vector<std::string>& startup_packages = {});
  static bool Load(const std::string& package_name);
  static bool Load(const char* package_name);
  static bool Load(const std::filesystem::path& package_path);
  static bool LoadAll();
  static bool Unload(const std::string& package_name);
  static bool Reload(const std::string& package_name);
  static void UnloadAll();
  static bool CanModifyPackages();
  static bool SetLifecycleCallbacks(PackageLifecycleCallbacks callbacks);
  static void ShutdownLifecycleCallbacks();
  static uint64_t RegisterTypeCleanupCallback(std::function<void(const std::string&)> callback);
  static void UnregisterTypeCleanupCallback(uint64_t id);
  static void ScanAvailablePackages();
  static std::vector<std::filesystem::path> GetSearchPaths();
  static std::vector<AvailablePackageInfo> GetAvailablePackages();
  static std::vector<LoadedPackageInfo> GetLoadedPackages();

  static void IncrementLiveObject(const std::string& package_name);
  static void DecrementLiveObject(const std::string& package_name);
  static size_t GetLiveObjectCount(const std::string& package_name);
};

template <typename T>
bool PackageRegistrar::RegisterPrivateComponent(const std::string& name) {
  if (!registered_private_component_names_) {
    EVOENGINE_ERROR("Package registrar is not initialized.")
    return false;
  }
  if (Serialization::HasSerializableType(name)) {
    EVOENGINE_ERROR("Package private component " + name + " is already registered.")
    return false;
  }

  const auto package_name = package_name_;
  const auto serializable_registered =
      Serialization::RegisterSerializableType(name, typeid(T).hash_code(), [package_name](size_t& hash_code) {
        hash_code = typeid(T).hash_code();
        return AdoptObject(new T(), package_name);
      });
  if (!serializable_registered) {
    return false;
  }

  const auto private_component_registered = Serialization::RegisterPrivateComponentType(
      name, typeid(T).hash_code(),
      [](const std::shared_ptr<IPrivateComponent>& target, const std::shared_ptr<IPrivateComponent>& source) {
        target->handle_ = source->handle_;
        target->enabled_ = source->enabled_;
        target->owner_ = source->owner_;
        *std::dynamic_pointer_cast<T>(target) = *std::dynamic_pointer_cast<T>(source);
        target->started_ = false;
        target->PostCloneAction(source);
      });
  if (!private_component_registered) {
    Serialization::UnregisterSerializableType(name);
    return false;
  }

  Serialization::SetSerializableTypeOwner(name, package_name);
  Serialization::SetPrivateComponentTypeOwner(name, package_name);
  Serialization::RegisterDefaultSerializationHandler<T>(package_name, name);
  Serialization::RegisterDefaultSerializationSupportHandler<T>(package_name, name);
  registered_private_component_names_->push_back(name);
  return true;
}

template <typename T>
bool PackageRegistrar::RegisterAsset(const std::string& name, const std::vector<std::string>& extensions) {
  if (!registered_asset_names_) {
    EVOENGINE_ERROR("Package registrar is not initialized.")
    return false;
  }
  if (Serialization::HasSerializableType(name) || Serialization::HasAssetType(name)) {
    EVOENGINE_ERROR("Package asset " + name + " is already registered.")
    return false;
  }

  const auto package_name = package_name_;
  const auto asset_registered =
      Serialization::RegisterAssetType(name, typeid(T).hash_code(), extensions, [package_name](size_t& hash_code) {
        hash_code = typeid(T).hash_code();
        return AdoptObject(new T(), package_name);
      });
  if (!asset_registered) {
    return false;
  }

  Serialization::SetSerializableTypeOwner(name, package_name);
  Serialization::RegisterDefaultSerializationHandler<T>(package_name, name);
  Serialization::RegisterDefaultSerializationSupportHandler<T>(package_name, name);
  Serialization::RegisterDefaultAssetIoHandler<T>(package_name, name);
  registered_asset_names_->push_back(name);
  return true;
}

template <typename T>
bool PackageRegistrar::RegisterDataComponent(const std::string& name) {
  if (!registered_data_component_names_) {
    EVOENGINE_ERROR("Package registrar is not initialized.")
    return false;
  }
  if (Serialization::HasComponentDataType(name)) {
    EVOENGINE_ERROR("Package data component " + name + " is already registered.")
    return false;
  }

  if (!Serialization::RegisterDataComponentType<T>(name)) {
    return false;
  }
  Serialization::SetDataComponentTypeOwner(name, package_name_);
  registered_data_component_names_->push_back(name);
  return true;
}

template <typename T>
bool PackageRegistrar::RegisterSystem(const std::string& name) {
  if (!registered_system_names_) {
    EVOENGINE_ERROR("Package registrar is not initialized.")
    return false;
  }
  if (Serialization::HasSerializableType(name)) {
    EVOENGINE_ERROR("Package system " + name + " is already registered.")
    return false;
  }

  const auto package_name = package_name_;
  const auto serializable_registered =
      Serialization::RegisterSerializableType(name, typeid(T).hash_code(), [package_name](size_t& hash_code) {
        hash_code = typeid(T).hash_code();
        return AdoptObject(new T(), package_name);
      });
  if (!serializable_registered) {
    return false;
  }

  const auto system_registered = Serialization::RegisterSystemType(
      name, typeid(T).hash_code(), [](const std::shared_ptr<ISystem>& target, const std::shared_ptr<ISystem>& source) {
        target->handle_ = source->handle_;
        target->rank_ = source->rank_;
        target->enabled_ = source->enabled_;
        *std::dynamic_pointer_cast<T>(target) = *std::dynamic_pointer_cast<T>(source);
        target->started_ = false;
        target->PostCloneAction(source);
      });
  if (!system_registered) {
    Serialization::UnregisterSerializableType(name);
    return false;
  }

  Serialization::SetSerializableTypeOwner(name, package_name);
  Serialization::SetSystemTypeOwner(name, package_name);
  Serialization::RegisterDefaultSerializationHandler<T>(package_name, name);
  Serialization::RegisterDefaultSerializationSupportHandler<T>(package_name, name);
  registered_system_names_->push_back(name);
  return true;
}

template <typename T>
bool PackageRegistrar::RegisterLayer(const std::string& name) {
  if (!registered_layer_names_) {
    EVOENGINE_ERROR("Package registrar is not initialized.")
    return false;
  }
  const auto layer = ApplicationContext::Get().PushLayer<T>(name, package_name_);
  if (!layer) {
    return false;
  }
  registered_layer_names_->push_back(name);
  return true;
}
}  // namespace evo_engine
