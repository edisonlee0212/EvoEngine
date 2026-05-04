#pragma once
#include "Serialization.hpp"

namespace evo_engine {
constexpr uint32_t EVOENGINE_PACKAGE_API_VERSION = 1;

struct PackageDescriptor {
  uint32_t api_version = EVOENGINE_PACKAGE_API_VERSION;
  const char* name = nullptr;
  const char* version = nullptr;
  const char* description = nullptr;
};

class PackageManager;

class PackageRegistrar {
  friend class PackageManager;

  std::string package_name_;
  std::vector<std::string>* registered_private_component_names_ = nullptr;

  PackageRegistrar(std::string package_name, std::vector<std::string>& registered_private_component_names);

 public:
  template <typename T>
  bool RegisterPrivateComponent(const std::string& name);
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
  std::string name;
  std::string version;
  std::string description;
  std::filesystem::path original_path;
  std::filesystem::path loaded_path;
  std::vector<std::string> private_component_types;
  size_t live_object_count = 0;
};

class PackageManager final {
 public:
  static PackageManager& GetInstance();

 private:
  friend class PackageRegistrar;

  struct LoadedPackage {
    LoadedPackageInfo info;
    void* library_handle = nullptr;
    EvoEnginePackageUnloadFn unload = nullptr;
  };

  std::mutex mutex_;
  std::vector<std::filesystem::path> search_paths_;
  std::unordered_map<std::string, LoadedPackage> loaded_packages_;
  std::unordered_map<std::string, size_t> live_object_counts_;
  uint64_t shadow_copy_index_ = 0;

  static bool IsRuntimeBusy();
  static bool OpenLibrary(const std::filesystem::path& path, void*& handle);
  static void CloseLibrary(void* handle);
  static void* GetSymbol(void* handle, const char* name);
  static std::filesystem::path CreateShadowCopy(const std::filesystem::path& source);
  static std::vector<std::filesystem::path> BuildDefaultSearchPaths();
  static bool HasLivePrivateComponentOwners(const std::vector<size_t>& type_ids);
  static void ClearPrivateComponentPools(const std::vector<size_t>& type_ids);

 public:
  static void Initialize(const std::vector<std::filesystem::path>& package_search_paths = {});
  static bool Load(const std::filesystem::path& package_path);
  static bool LoadAll();
  static bool Unload(const std::string& package_name);
  static bool Reload(const std::string& package_name);
  static void UnloadAll();
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
        PackageManager::IncrementLiveObject(package_name);
        std::shared_ptr<T> ptr(new T(), [package_name](T* value) {
          delete value;
          PackageManager::DecrementLiveObject(package_name);
        });
        return std::static_pointer_cast<ISerializable>(ptr);
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
  registered_private_component_names_->push_back(name);
  return true;
}
}  // namespace evo_engine
