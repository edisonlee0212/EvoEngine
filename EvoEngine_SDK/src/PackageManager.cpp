#include "PackageManager.hpp"

#include "Application.hpp"
#include "AssetManager.hpp"
#include "ProjectManager.hpp"
#include "Scene.hpp"

#include <cstring>

#if !defined(_WIN32)
#  include <dlfcn.h>
#endif

using namespace evo_engine;

PackageRegistrar::PackageRegistrar(std::string package_name,
                                   std::vector<std::string>& registered_private_component_names,
                                   std::vector<std::string>& registered_asset_names,
                                   std::vector<std::string>& registered_data_component_names,
                                   std::vector<std::string>& registered_system_names,
                                   std::vector<std::string>& registered_layer_names)
    : package_name_(std::move(package_name)),
      registered_private_component_names_(&registered_private_component_names),
      registered_asset_names_(&registered_asset_names),
      registered_data_component_names_(&registered_data_component_names),
      registered_system_names_(&registered_system_names),
      registered_layer_names_(&registered_layer_names) {
}

bool PackageManager::IsRuntimeBusy() {
  const auto status = ApplicationContext::Get().GetApplicationStatus();
  return status == Application::ExecutionStatus::Playing || status == Application::ExecutionStatus::Step;
}

bool PackageManager::OpenLibrary(const std::filesystem::path& path, void*& handle) {
#if defined(_WIN32)
  handle = LoadLibraryW(path.wstring().c_str());
  if (!handle) {
    EVOENGINE_ERROR("Failed to load runtime package library: " + path.string() +
                    ". Windows error: " + std::to_string(GetLastError()))
    return false;
  }
#else
  handle = dlopen(path.string().c_str(), RTLD_NOW);
  if (!handle) {
    EVOENGINE_ERROR("Failed to load runtime package library: " + path.string() + ". " + dlerror())
    return false;
  }
#endif
  return true;
}

void PackageManager::CloseLibrary(void* handle) {
  if (!handle)
    return;
#if defined(_WIN32)
  FreeLibrary(static_cast<HMODULE>(handle));
#else
  dlclose(handle);
#endif
}

void* PackageManager::GetSymbol(void* handle, const char* name) {
  if (!handle)
    return nullptr;
#if defined(_WIN32)
  return reinterpret_cast<void*>(GetProcAddress(static_cast<HMODULE>(handle), name));
#else
  return dlsym(handle, name);
#endif
}

std::filesystem::path PackageManager::CreateShadowCopy(const std::filesystem::path& source) {
#if defined(_WIN32)
  auto& manager = GetInstance();
  uint64_t copy_index;
  {
    std::lock_guard lock(manager.mutex_);
    copy_index = ++manager.shadow_copy_index_;
  }

  std::error_code ec;
  const auto shadow_root =
      std::filesystem::temp_directory_path(ec) / "EvoEnginePackageShadow" / std::to_string(GetCurrentProcessId());
  if (ec) {
    EVOENGINE_ERROR("Failed to find temporary directory for runtime package shadow copy: " + ec.message())
    return {};
  }
  std::filesystem::create_directories(shadow_root, ec);
  if (ec) {
    EVOENGINE_ERROR("Failed to create runtime package shadow directory: " + shadow_root.string())
    return {};
  }

  const auto timestamp =
      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch())
          .count();
  const auto shadow_path = shadow_root / (source.stem().string() + "_" + std::to_string(timestamp) + "_" +
                                          std::to_string(copy_index) + source.extension().string());
  std::filesystem::copy_file(source, shadow_path, std::filesystem::copy_options::overwrite_existing, ec);
  if (ec) {
    EVOENGINE_ERROR("Failed to shadow-copy runtime package " + source.string() + ": " + ec.message())
    return {};
  }
  return shadow_path;
#else
  return source;
#endif
}

std::vector<std::filesystem::path> PackageManager::BuildDefaultSearchPaths() {
  std::vector<std::filesystem::path> ret_val;
  auto push_unique = [&](const std::filesystem::path& path) {
    std::error_code ec;
    auto absolute_path = std::filesystem::absolute(path, ec);
    if (ec)
      absolute_path = path;
    for (const auto& existing : ret_val) {
      if (existing == absolute_path)
        return;
    }
    ret_val.emplace_back(absolute_path);
  };

  push_unique(std::filesystem::current_path() / "Packages");
#if defined(_WIN32)
  std::vector<wchar_t> buffer(32768);
  const auto size = GetModuleFileNameW(nullptr, buffer.data(), static_cast<DWORD>(buffer.size()));
  if (size > 0 && size < buffer.size()) {
    push_unique(std::filesystem::path(buffer.data(), buffer.data() + size).parent_path() / "Packages");
  }
#elif defined(__linux__)
  std::error_code executable_path_ec;
  const auto executable_path = std::filesystem::read_symlink("/proc/self/exe", executable_path_ec);
  if (!executable_path_ec) {
    push_unique(executable_path.parent_path() / "Packages");
  }
#endif
  return ret_val;
}

bool PackageManager::HasLivePrivateComponentOwners(const std::vector<size_t>& type_ids) {
  std::vector<std::shared_ptr<Scene>> scenes;
  if (const auto active_scene = ApplicationContext::Get().GetActiveScene()) {
    scenes.emplace_back(active_scene);
  }
  if (const auto start_scene = ProjectManager::GetStartScene().lock()) {
    bool exists = false;
    for (const auto& scene : scenes) {
      if (scene.get() == start_scene.get()) {
        exists = true;
        break;
      }
    }
    if (!exists)
      scenes.emplace_back(start_scene);
  }

  for (const auto& scene : scenes) {
    for (const auto type_id : type_ids) {
      if (scene->HasPrivateComponentOwners(type_id)) {
        return true;
      }
    }
  }
  return false;
}

void PackageManager::ClearPrivateComponentPools(const std::vector<size_t>& type_ids) {
  std::vector<std::shared_ptr<Scene>> scenes;
  if (const auto active_scene = ApplicationContext::Get().GetActiveScene()) {
    scenes.emplace_back(active_scene);
  }
  if (const auto start_scene = ProjectManager::GetStartScene().lock()) {
    bool exists = false;
    for (const auto& scene : scenes) {
      if (scene.get() == start_scene.get()) {
        exists = true;
        break;
      }
    }
    if (!exists)
      scenes.emplace_back(start_scene);
  }

  for (const auto& scene : scenes) {
    for (const auto type_id : type_ids) {
      scene->ClearPrivateComponentPool(type_id);
    }
  }
}

void PackageManager::RestoreUnknownRuntimeTypes() {
  std::vector<std::shared_ptr<Scene>> scenes;
  if (const auto active_scene = ApplicationContext::Get().GetActiveScene()) {
    scenes.emplace_back(active_scene);
  }
  if (const auto start_scene = ProjectManager::GetStartScene().lock()) {
    bool exists = false;
    for (const auto& scene : scenes) {
      if (scene.get() == start_scene.get()) {
        exists = true;
        break;
      }
    }
    if (!exists) {
      scenes.emplace_back(start_scene);
    }
  }

  size_t restored_count = AssetManager::RestoreUnknownAssets();
  for (const auto& scene : scenes) {
    restored_count += scene->RestoreUnknownRuntimeTypes();
  }
  if (restored_count > 0) {
    EVOENGINE_LOG("Restored " + std::to_string(restored_count) + " runtime objects after package load.")
  }
}

void PackageManager::Initialize(const std::vector<std::filesystem::path>& package_search_paths) {
  auto& manager = GetInstance();
  {
    std::lock_guard lock(manager.mutex_);
    manager.search_paths_ = BuildDefaultSearchPaths();
    for (const auto& path : package_search_paths) {
      std::error_code ec;
      auto absolute_path = std::filesystem::absolute(path, ec);
      if (ec)
        absolute_path = path;
      bool exists = false;
      for (const auto& existing : manager.search_paths_) {
        if (existing == absolute_path) {
          exists = true;
          break;
        }
      }
      if (!exists)
        manager.search_paths_.emplace_back(absolute_path);
    }
  }
  LoadAll();
}

bool PackageManager::Load(const std::filesystem::path& package_path) {
  std::error_code ec;
  const auto original_path = std::filesystem::absolute(package_path, ec);
  if (ec || !std::filesystem::exists(original_path)) {
    EVOENGINE_ERROR("Runtime package does not exist: " + package_path.string())
    return false;
  }

  const auto loaded_path = CreateShadowCopy(original_path);
  if (loaded_path.empty())
    return false;

  void* library_handle = nullptr;
  if (!OpenLibrary(loaded_path, library_handle))
    return false;

  const auto get_descriptor =
      reinterpret_cast<EvoEnginePackageGetDescriptorFn>(GetSymbol(library_handle, "EvoEnginePackageGetDescriptor"));
  const auto register_types =
      reinterpret_cast<EvoEnginePackageRegisterTypesFn>(GetSymbol(library_handle, "EvoEnginePackageRegisterTypes"));
  const auto load = reinterpret_cast<EvoEnginePackageLoadFn>(GetSymbol(library_handle, "EvoEnginePackageLoad"));
  const auto unload = reinterpret_cast<EvoEnginePackageUnloadFn>(GetSymbol(library_handle, "EvoEnginePackageUnload"));
  if (!get_descriptor || !load || !unload) {
    EVOENGINE_ERROR("Runtime package is missing required entrypoints: " + original_path.string())
    CloseLibrary(library_handle);
    return false;
  }

  const auto descriptor = get_descriptor();
  if (!descriptor || descriptor->api_version != EVOENGINE_PACKAGE_API_VERSION) {
    EVOENGINE_ERROR("Runtime package has an incompatible descriptor: " + original_path.string())
    CloseLibrary(library_handle);
    return false;
  }

  const std::string package_name =
      descriptor->name && std::strlen(descriptor->name) > 0 ? descriptor->name : original_path.stem().string();
  {
    auto& manager = GetInstance();
    std::lock_guard lock(manager.mutex_);
    if (manager.loaded_packages_.find(package_name) != manager.loaded_packages_.end()) {
      EVOENGINE_WARNING("Runtime package already loaded: " + package_name)
      CloseLibrary(library_handle);
      return true;
    }
    manager.live_object_counts_[package_name] = 0;
  }

  std::vector<std::string> registered_private_component_names;
  std::vector<std::string> registered_asset_names;
  std::vector<std::string> registered_data_component_names;
  std::vector<std::string> registered_system_names;
  std::vector<std::string> registered_layer_names;
  PackageRegistrar registrar(package_name, registered_private_component_names, registered_asset_names,
                             registered_data_component_names, registered_system_names, registered_layer_names);
  if (register_types && !register_types(&registrar)) {
    EVOENGINE_ERROR("Runtime package type registration failed: " + package_name)
    Serialization::UnregisterPackageOwnedTypes(package_name);
    {
      auto& manager = GetInstance();
      std::lock_guard lock(manager.mutex_);
      manager.live_object_counts_.erase(package_name);
    }
    CloseLibrary(library_handle);
    return false;
  }

  if (!load(&registrar)) {
    EVOENGINE_ERROR("Runtime package load callback failed: " + package_name)
    Serialization::UnregisterPackageOwnedTypes(package_name);
    {
      auto& manager = GetInstance();
      std::lock_guard lock(manager.mutex_);
      manager.live_object_counts_.erase(package_name);
    }
    CloseLibrary(library_handle);
    return false;
  }

  LoadedPackage package;
  package.info.name = package_name;
  package.info.version = descriptor->version ? descriptor->version : "";
  package.info.description = descriptor->description ? descriptor->description : "";
  package.info.original_path = original_path;
  package.info.loaded_path = loaded_path;
  package.info.private_component_types = std::move(registered_private_component_names);
  package.info.asset_types = std::move(registered_asset_names);
  package.info.data_component_types = std::move(registered_data_component_names);
  package.info.system_types = std::move(registered_system_names);
  package.info.layer_types = std::move(registered_layer_names);
  package.info.live_object_count = GetLiveObjectCount(package_name);
  package.library_handle = library_handle;
  package.unload = unload;

  {
    auto& manager = GetInstance();
    std::lock_guard lock(manager.mutex_);
    manager.loaded_packages_[package_name] = std::move(package);
  }
  RestoreUnknownRuntimeTypes();
  EVOENGINE_LOG("Runtime package loaded: " + package_name)
  return true;
}

bool PackageManager::LoadAll() {
  std::vector<std::filesystem::path> search_paths;
  {
    auto& manager = GetInstance();
    std::lock_guard lock(manager.mutex_);
    search_paths = manager.search_paths_;
  }
  if (search_paths.empty()) {
    search_paths = BuildDefaultSearchPaths();
  }

  bool success = true;
#if defined(_WIN32)
  const std::string package_extension = ".dll";
#elif defined(__APPLE__)
  const std::string package_extension = ".dylib";
#else
  const std::string package_extension = ".so";
#endif
  for (const auto& search_path : search_paths) {
    std::error_code ec;
    if (!std::filesystem::exists(search_path, ec) || !std::filesystem::is_directory(search_path, ec)) {
      continue;
    }
    for (const auto& entry : std::filesystem::directory_iterator(search_path, ec)) {
      if (ec)
        break;
      if (!entry.is_regular_file() || entry.path().extension().string() != package_extension) {
        continue;
      }
      success = Load(entry.path()) && success;
    }
  }
  return success;
}

bool PackageManager::Unload(const std::string& package_name) {
  if (IsRuntimeBusy()) {
    EVOENGINE_WARNING("Cannot unload runtime package while the application is playing or stepping: " + package_name)
    return false;
  }

  LoadedPackage package;
  {
    auto& manager = GetInstance();
    std::lock_guard lock(manager.mutex_);
    const auto search = manager.loaded_packages_.find(package_name);
    if (search == manager.loaded_packages_.end()) {
      EVOENGINE_WARNING("Runtime package is not loaded: " + package_name)
      return false;
    }
    package = search->second;
  }

  const auto type_ids = Serialization::GetPackageOwnedPrivateComponentTypeIds(package_name);
  if (HasLivePrivateComponentOwners(type_ids)) {
    EVOENGINE_WARNING("Cannot unload runtime package because package-owned private component instances still exist: " +
                      package_name)
    return false;
  }

  if (!ApplicationContext::Get().RemoveLayersOwnedByPackage(package_name)) {
    EVOENGINE_WARNING("Cannot unload runtime package because package-owned layers are still active: " + package_name)
    return false;
  }

  ClearPrivateComponentPools(type_ids);
  if (const auto live_count = GetLiveObjectCount(package_name); live_count > 0) {
    EVOENGINE_WARNING("Cannot unload runtime package because " + std::to_string(live_count) +
                      " package-owned objects are still alive: " + package_name)
    return false;
  }

  std::vector<std::string> registered_private_component_names = package.info.private_component_types;
  std::vector<std::string> registered_asset_names = package.info.asset_types;
  std::vector<std::string> registered_data_component_names = package.info.data_component_types;
  std::vector<std::string> registered_system_names = package.info.system_types;
  std::vector<std::string> registered_layer_names = package.info.layer_types;
  PackageRegistrar registrar(package_name, registered_private_component_names, registered_asset_names,
                             registered_data_component_names, registered_system_names, registered_layer_names);
  package.unload(&registrar);
  Serialization::UnregisterPackageOwnedTypes(package_name);
  CloseLibrary(package.library_handle);

#if defined(_WIN32)
  if (!package.info.loaded_path.empty() && package.info.loaded_path != package.info.original_path) {
    std::error_code ec;
    std::filesystem::remove(package.info.loaded_path, ec);
  }
#endif

  {
    auto& manager = GetInstance();
    std::lock_guard lock(manager.mutex_);
    manager.loaded_packages_.erase(package_name);
    manager.live_object_counts_.erase(package_name);
  }
  EVOENGINE_LOG("Runtime package unloaded: " + package_name)
  return true;
}

bool PackageManager::Reload(const std::string& package_name) {
  std::filesystem::path original_path;
  {
    auto& manager = GetInstance();
    std::lock_guard lock(manager.mutex_);
    const auto search = manager.loaded_packages_.find(package_name);
    if (search == manager.loaded_packages_.end()) {
      EVOENGINE_WARNING("Runtime package is not loaded: " + package_name)
      return false;
    }
    original_path = search->second.info.original_path;
  }
  if (!Unload(package_name)) {
    return false;
  }
  return Load(original_path);
}

void PackageManager::UnloadAll() {
  std::vector<std::string> package_names;
  {
    auto& manager = GetInstance();
    std::lock_guard lock(manager.mutex_);
    for (const auto& [name, package] : manager.loaded_packages_) {
      package_names.emplace_back(name);
    }
  }
  for (const auto& name : package_names) {
    Unload(name);
  }
}

std::vector<LoadedPackageInfo> PackageManager::GetLoadedPackages() {
  auto& manager = GetInstance();
  std::lock_guard lock(manager.mutex_);
  std::vector<LoadedPackageInfo> ret_val;
  for (const auto& [name, package] : manager.loaded_packages_) {
    auto info = package.info;
    if (const auto search = manager.live_object_counts_.find(name); search != manager.live_object_counts_.end()) {
      info.live_object_count = search->second;
    }
    ret_val.emplace_back(std::move(info));
  }
  return ret_val;
}

void PackageManager::IncrementLiveObject(const std::string& package_name) {
  auto& manager = GetInstance();
  std::lock_guard lock(manager.mutex_);
  ++manager.live_object_counts_[package_name];
}

void PackageManager::DecrementLiveObject(const std::string& package_name) {
  auto& manager = GetInstance();
  std::lock_guard lock(manager.mutex_);
  if (const auto search = manager.live_object_counts_.find(package_name);
      search != manager.live_object_counts_.end() && search->second > 0) {
    --search->second;
  }
}

size_t PackageManager::GetLiveObjectCount(const std::string& package_name) {
  auto& manager = GetInstance();
  std::lock_guard lock(manager.mutex_);
  if (const auto search = manager.live_object_counts_.find(package_name); search != manager.live_object_counts_.end()) {
    return search->second;
  }
  return 0;
}
