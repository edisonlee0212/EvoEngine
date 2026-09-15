#include "PackageManager.hpp"

#include "Application.hpp"
#include "AssetManager.hpp"
#include "NativeLibrary.hpp"
#include "PathUtils.hpp"
#include "Platform.hpp"
#include "ProjectManager.hpp"
#include "RuntimePaths.hpp"
#include "Scene.hpp"

#include <cstring>

using namespace evo_engine;

class PackageManager::Mutation {
  PackageManager& manager_;
  std::string name_;
  bool active_;

 public:
  explicit Mutation(std::string name) : manager_(GetInstance()), name_(std::move(name)) {
    std::lock_guard lock(manager_.mutex_);
    active_ = !manager_.shutting_down_lifecycle_ && manager_.active_mutations_.empty();
    if (active_)
      manager_.active_mutations_.insert(name_);
    if (!active_)
      EVOENGINE_ERROR("A package mutation is already active: " + name_)
  }
  ~Mutation() {
    if (active_) {
      std::lock_guard lock(manager_.mutex_);
      manager_.active_mutations_.erase(name_);
    }
  }
  explicit operator bool() const {
    return active_;
  }
};

bool PackageManager::SetLifecycleCallbacks(PackageLifecycleCallbacks callbacks) {
  auto& manager = GetInstance();
  std::lock_guard lock(manager.mutex_);
  if (!manager.active_mutations_.empty() || !manager.loaded_packages_.empty() || manager.shutting_down_lifecycle_) {
    EVOENGINE_ERROR("Package lifecycle integration must be installed before packages are loaded.")
    return false;
  }
  manager.lifecycle_callbacks_ = std::move(callbacks);
  return true;
}

void PackageManager::ShutdownLifecycleCallbacks() {
  auto& manager = GetInstance();
  PackageLifecycleCallbacks callbacks;
  {
    std::lock_guard lock(manager.mutex_);
    manager.shutting_down_lifecycle_ = true;
    callbacks = std::move(manager.lifecycle_callbacks_);
    manager.lifecycle_callbacks_ = {};
  }
  AssetManager::WaitForPendingLoads();
  if (callbacks.shutdown)
    callbacks.shutdown();
  {
    std::lock_guard lock(manager.mutex_);
    manager.shutting_down_lifecycle_ = false;
  }
}

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

std::shared_ptr<ISerializable> PackageRegistrar::AdoptObject(ISerializable* object, const std::string& package_name) {
  PackageManager::IncrementLiveObject(package_name);
  // Weak references may outlive the DLL; their control block must belong to the SDK.
  return std::shared_ptr<ISerializable>(object, [package_name](ISerializable* value) {
    delete value;
    PackageManager::DecrementLiveObject(package_name);
  });
}

ProfilerItemHandle PackageRegistrar::RegisterProfilerItem(const ProfilerItemDescriptor& descriptor) {
  return Profiler::GetInstance().RegisterItem(package_name_, descriptor);
}

bool PackageManager::IsRuntimeBusy() {
  const auto status = ApplicationContext::Get().GetApplicationStatus();
  return status == Application::ExecutionStatus::Playing || status == Application::ExecutionStatus::Step ||
         status == Application::ExecutionStatus::Pause;
}

bool PackageManager::CanModifyPackages() {
  return !ApplicationContext::Get().RuntimeOperationBusy() && !IsRuntimeBusy();
}

namespace {
bool RejectPackageMutationWhenBusy(const std::string& operation, const std::string& package_name = {}) {
  if (ApplicationContext::Get().IsDispatchingLayers()) {
    EVOENGINE_WARNING("Package mutations must be queued until the application frame finishes.")
    return true;
  }
  if (PackageManager::CanModifyPackages()) {
    return false;
  }
  auto message = "Cannot " + operation + " runtime packages while the application is playing, stepping, or paused";
  if (!package_name.empty()) {
    message += ": " + package_name;
  }
  EVOENGINE_WARNING(message)
  return true;
}
}  // namespace

std::vector<std::filesystem::path> PackageManager::BuildDefaultSearchPaths() {
  if (runtime_paths::IsStrict()) {
    return {runtime_paths::Resolve("Packages")};
  }
  std::vector<std::filesystem::path> ret_val;
  path_utils::AddUniqueNormalizedPath(ret_val, std::filesystem::current_path() / "Packages");
  if (const auto executable_path = path_utils::CurrentExecutablePath(); !executable_path.empty()) {
    path_utils::AddUniqueNormalizedPath(ret_val, executable_path.parent_path() / "Packages");
  }
  return ret_val;
}

bool PackageManager::ReadManifest(const std::filesystem::path& manifest_path, PackageManifest& manifest) {
  try {
    const auto in = YAML::LoadFile(manifest_path.string());
    if (!in["name"] || !in["library"]) {
      EVOENGINE_WARNING("Runtime package manifest is missing name or library: " + manifest_path.string())
      return false;
    }

    manifest.name = in["name"].as<std::string>();
    manifest.library = in["library"].as<std::string>();
    manifest.version = in["version"] ? in["version"].as<std::string>() : "";
    manifest.description = in["description"] ? in["description"].as<std::string>() : "";
    manifest.sdk_source_id = in["sdk_source_id"] ? in["sdk_source_id"].as<std::string>() : "";
    manifest.package_source_id = in["package_source_id"] ? in["package_source_id"].as<std::string>() : "";
    manifest.compiler_id = in["compiler_id"] ? in["compiler_id"].as<std::string>() : "";
    manifest.compiler_version = in["compiler_version"] ? in["compiler_version"].as<std::string>() : "";
    manifest.configuration = in["configuration"] ? in["configuration"].as<std::string>() : "";
    manifest.platform = in["platform"] ? in["platform"].as<std::string>() : "";
    manifest.architecture = in["architecture"] ? in["architecture"].as<std::string>() : "";
    manifest.with_editor = in["with_editor"] ? in["with_editor"].as<bool>() : false;
    manifest.library_sha256 = in["library_sha256"] ? in["library_sha256"].as<std::string>() : "";
    manifest.dependencies.clear();
    if (in["dependencies"] && in["dependencies"].IsSequence()) {
      for (const auto& dependency : in["dependencies"]) {
        manifest.dependencies.emplace_back(dependency.as<std::string>());
      }
    }
    manifest.manifest_path = std::filesystem::absolute(manifest_path);
    manifest.library_path = manifest.manifest_path.parent_path() / manifest.library;
    return true;
  } catch (const std::exception& e) {
    EVOENGINE_WARNING("Failed to read runtime package manifest " + manifest_path.string() + ": " + e.what())
    return false;
  }
}

bool PackageManager::ValidateManifestCompatibility(const PackageManifest& manifest) {
  const auto& expected = GetNativeBuildIdentity();
  NativeBuildIdentity candidate{manifest.sdk_source_id.c_str(),
                                manifest.compiler_id.c_str(),
                                manifest.compiler_version.c_str(),
                                manifest.configuration.c_str(),
                                manifest.platform.c_str(),
                                manifest.architecture.c_str(),
                                manifest.with_editor};
  std::string reason;
  if (!IsNativeBuildCompatible(expected, candidate, &reason)) {
    EVOENGINE_ERROR("Runtime package manifest is incompatible [" + manifest.name + "]: " + reason)
    return false;
  }
  if (manifest.package_source_id.empty() || manifest.library_sha256.empty()) {
    EVOENGINE_ERROR("Runtime package manifest has empty provenance fields: " + manifest.name)
    return false;
  }
  return native_library::VerifyLibraryHash(manifest.library_path, manifest.library_sha256);
}

bool PackageManager::ValidateDescriptorCompatibility(const PackageDescriptor& descriptor,
                                                     const PackageManifest* manifest) {
  std::string reason;
  if (!IsNativeBuildCompatible(GetNativeBuildIdentity(), descriptor.build_identity, &reason)) {
    EVOENGINE_ERROR("Runtime package descriptor is incompatible: " + reason)
    return false;
  }
  const std::string_view package_source_id = descriptor.package_source_id ? descriptor.package_source_id : "";
  const std::string_view package_name = descriptor.name ? descriptor.name : "";
  if (package_name.empty() || package_source_id.empty()) {
    EVOENGINE_ERROR("Runtime package descriptor is missing its name or package source ID.")
    return false;
  }
  if (manifest && (manifest->name != package_name || manifest->package_source_id != package_source_id)) {
    EVOENGINE_ERROR("Runtime package descriptor does not match its manifest: " + manifest->name)
    return false;
  }
  return true;
}

void PackageManager::RefreshManifests() {
  auto& manager = GetInstance();
  std::vector<std::filesystem::path> search_paths;
  {
    std::lock_guard lock(manager.mutex_);
    search_paths = manager.search_paths_;
  }
  if (search_paths.empty()) {
    search_paths = BuildDefaultSearchPaths();
  }

  std::unordered_map<std::string, PackageManifest> manifests;
  for (const auto& search_path : search_paths) {
    std::error_code ec;
    if (!std::filesystem::exists(search_path, ec) || !std::filesystem::is_directory(search_path, ec)) {
      continue;
    }
    for (const auto& entry : std::filesystem::directory_iterator(search_path, ec)) {
      if (ec)
        break;
      if (!entry.is_regular_file() || entry.path().extension().string() != ".evepackage") {
        continue;
      }
      PackageManifest manifest;
      if (ReadManifest(entry.path(), manifest)) {
        manifests[manifest.name] = std::move(manifest);
      }
    }
  }

  {
    std::lock_guard lock(manager.mutex_);
    manager.package_manifests_ = std::move(manifests);
  }
}

bool PackageManager::LoadManifestWithDependencies(const std::string& package_name,
                                                  std::vector<std::string>& loading_stack) {
  {
    auto& manager = GetInstance();
    std::lock_guard lock(manager.mutex_);
    if (manager.shutting_down_lifecycle_ ||
        manager.active_mutations_.find(package_name) != manager.active_mutations_.end())
      return false;
    if (const auto loaded = manager.loaded_packages_.find(package_name); loaded != manager.loaded_packages_.end())
      return loaded->second.info.ready;
  }

  if (std::find(loading_stack.begin(), loading_stack.end(), package_name) != loading_stack.end()) {
    EVOENGINE_ERROR("Runtime package dependency cycle detected while loading: " + package_name)
    return false;
  }

  PackageManifest manifest;
  {
    auto& manager = GetInstance();
    std::lock_guard lock(manager.mutex_);
    const auto search = manager.package_manifests_.find(package_name);
    if (search == manager.package_manifests_.end()) {
      EVOENGINE_ERROR("Runtime package manifest not found: " + package_name)
      return false;
    }
    manifest = search->second;
  }

  loading_stack.emplace_back(package_name);
  for (const auto& dependency_name : manifest.dependencies) {
    if (!LoadManifestWithDependencies(dependency_name, loading_stack)) {
      loading_stack.pop_back();
      EVOENGINE_ERROR("Failed to load runtime package dependency [" + dependency_name + "] for [" + package_name + "]")
      return false;
    }
  }
  loading_stack.pop_back();

  if (!std::filesystem::exists(manifest.library_path)) {
    EVOENGINE_ERROR("Runtime package library from manifest does not exist: " + manifest.library_path.string())
    return false;
  }
  return Load(manifest.library_path);
}

bool PackageManager::HasLoadedDependents(const std::string& package_name, std::string* dependent_name) {
  auto& manager = GetInstance();
  std::lock_guard lock(manager.mutex_);
  for (const auto& [loaded_package_name, package] : manager.loaded_packages_) {
    if (loaded_package_name == package_name) {
      continue;
    }
    if (std::find(package.info.dependencies.begin(), package.info.dependencies.end(), package_name) !=
        package.info.dependencies.end()) {
      if (dependent_name) {
        *dependent_name = loaded_package_name;
      }
      return true;
    }
  }
  return false;
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
    EVOENGINE_WARNING("Restored " + std::to_string(restored_count) + " runtime objects after package load.")
  }
}

void PackageManager::Initialize(const std::vector<std::filesystem::path>& package_search_paths,
                                const std::vector<std::string>& startup_packages) {
  auto& manager = GetInstance();
  {
    std::lock_guard lock(manager.mutex_);
    manager.search_paths_ = BuildDefaultSearchPaths();
    if (!runtime_paths::IsStrict()) {
      for (const auto& path : package_search_paths) {
        path_utils::AddUniqueNormalizedPath(manager.search_paths_, path);
      }
    }
  }
  RefreshManifests();
  for (const auto& package_name : startup_packages) {
    Load(package_name);
  }
}

bool PackageManager::Load(const std::string& package_name) {
  if (RejectPackageMutationWhenBusy("load", package_name)) {
    return false;
  }
  RefreshManifests();
  std::vector<std::string> loading_stack;
  return LoadManifestWithDependencies(package_name, loading_stack);
}

bool PackageManager::Load(const char* package_name) {
  if (!package_name) {
    return false;
  }
  return Load(std::string(package_name));
}

bool PackageManager::Load(const std::filesystem::path& package_path) {
  if (RejectPackageMutationWhenBusy("load", package_path.string())) {
    return false;
  }
  std::error_code ec;
  const auto original_path = std::filesystem::absolute(package_path, ec);
  if (ec || !std::filesystem::exists(original_path)) {
    EVOENGINE_ERROR("Runtime package does not exist: " + package_path.string())
    return false;
  }

  if (runtime_paths::IsStrict() && !path_utils::IsSameOrChildPath(original_path, runtime_paths::Resolve("Packages"))) {
    EVOENGINE_ERROR("Runtime packages must be inside the distribution Packages directory.")
    return false;
  }

  std::optional<PackageManifest> package_manifest;
  {
    auto& manager = GetInstance();
    std::lock_guard lock(manager.mutex_);
    for (const auto& [_, manifest] : manager.package_manifests_) {
      std::error_code manifest_ec;
      const auto manifest_library_path = std::filesystem::absolute(manifest.library_path, manifest_ec);
      if (!manifest_ec && manifest_library_path == original_path) {
        package_manifest = manifest;
        break;
      }
    }
  }
  if (runtime_paths::IsStrict() && !package_manifest) {
    EVOENGINE_ERROR("Runtime package is missing its distribution manifest: " + original_path.string())
    return false;
  }
  if (package_manifest && !ValidateManifestCompatibility(*package_manifest)) {
    return false;
  }

  const auto loaded_path = native_library::CreateShadowCopy(original_path);
  if (loaded_path.empty())
    return false;

  void* library_handle = nullptr;
  const bool opened =
      (!package_manifest || native_library::VerifyLibraryHash(loaded_path, package_manifest->library_sha256)) &&
      native_library::OpenLibrary(loaded_path, library_handle);
  const auto library = std::shared_ptr<void>(library_handle, [loaded_path, original_path](void* handle) {
    native_library::CloseLibrary(handle);
    if (loaded_path != original_path) {
      std::error_code error;
      std::filesystem::remove(loaded_path, error);
      std::filesystem::remove(loaded_path.parent_path(), error);
    }
  });
  if (!opened)
    return false;

  const auto get_descriptor = reinterpret_cast<EvoEnginePackageGetDescriptorFn>(
      native_library::GetSymbol(library_handle, "EvoEnginePackageGetDescriptor"));
  const auto register_types = reinterpret_cast<EvoEnginePackageRegisterTypesFn>(
      native_library::GetSymbol(library_handle, "EvoEnginePackageRegisterTypes"));
  const auto load =
      reinterpret_cast<EvoEnginePackageLoadFn>(native_library::GetSymbol(library_handle, "EvoEnginePackageLoad"));
  const auto unload =
      reinterpret_cast<EvoEnginePackageUnloadFn>(native_library::GetSymbol(library_handle, "EvoEnginePackageUnload"));
  if (!get_descriptor || !load || !unload) {
    EVOENGINE_ERROR("Runtime package is missing required entrypoints: " + original_path.string())
    return false;
  }

  const auto descriptor = get_descriptor();
  if (!descriptor || descriptor->api_version != EVOENGINE_PACKAGE_API_VERSION) {
    EVOENGINE_ERROR("Runtime package has an incompatible descriptor: " + original_path.string())
    return false;
  }
  if (!ValidateDescriptorCompatibility(*descriptor, package_manifest ? &*package_manifest : nullptr)) {
    return false;
  }

  const std::string package_name =
      descriptor->name && std::strlen(descriptor->name) > 0 ? descriptor->name : original_path.stem().string();
  const auto get_module_identity = reinterpret_cast<const void* (*)()>(
      native_library::GetSymbol(library_handle, ("EvoEngineRuntimeModuleIdentity_" + package_name).c_str()));
  if (!get_module_identity) {
    EVOENGINE_ERROR("Runtime package is missing its module identity: " + package_name)
    return false;
  }
  Mutation mutation(package_name);
  if (!mutation) {
    return false;
  }
  PackageLifecycleCallbacks callbacks;

  {
    auto& manager = GetInstance();
    std::lock_guard lock(manager.mutex_);
    if (manager.loaded_packages_.find(package_name) != manager.loaded_packages_.end()) {
      EVOENGINE_WARNING("Runtime package already loaded: " + package_name)
      return manager.loaded_packages_.at(package_name).info.ready;
    }
    manager.live_object_counts_[package_name] = 0;
    callbacks = manager.lifecycle_callbacks_;
  }

  LoadedPackage package;
  package.info.module_identity = get_module_identity();
  package.info.name = package_name;
  package.info.version = descriptor->version ? descriptor->version : "";
  package.info.description = descriptor->description ? descriptor->description : "";
  package.info.package_source_id = descriptor->package_source_id;
  if (package_manifest) {
    package.info.dependencies = package_manifest->dependencies;
    package.info.manifest_path = package_manifest->manifest_path;
  }
  package.info.original_path = original_path;
  package.info.loaded_path = loaded_path;
  package.library = library;
  package.unload = unload;
  if (callbacks.validate && !callbacks.validate(package.info)) {
    {
      auto& manager = GetInstance();
      std::lock_guard lock(manager.mutex_);
      manager.live_object_counts_.erase(package_name);
    }
    return false;
  }

  std::vector<std::string> registered_private_component_names;
  std::vector<std::string> registered_asset_names;
  std::vector<std::string> registered_data_component_names;
  std::vector<std::string> registered_system_names;
  std::vector<std::string> registered_layer_names;
  PackageRegistrar registrar(package_name, registered_private_component_names, registered_asset_names,
                             registered_data_component_names, registered_system_names, registered_layer_names);
  bool activated = false;
  try {
    activated = (!register_types || register_types(&registrar)) && load(&registrar);
    package.info.private_component_types = registered_private_component_names;
    package.info.asset_types = registered_asset_names;
    package.info.data_component_types = registered_data_component_names;
    package.info.system_types = registered_system_names;
    package.info.layer_types = registered_layer_names;
    activated = activated && (!callbacks.activate || callbacks.activate(package.info));
  } catch (const std::exception& error) {
    activated = false;
    EVOENGINE_ERROR("Package activation failed: " + package_name + ". " + error.what())
  }
  if (!activated) {
    AssetManager::WaitForPendingLoads();
    bool can_deactivate = !callbacks.prepare_unload || callbacks.prepare_unload(package.info);
    if (can_deactivate && callbacks.deactivate)
      can_deactivate = callbacks.deactivate(package.info);
    bool layers_removed = false;
    if (can_deactivate) {
      const auto type_ids = Serialization::GetPackageOwnedPrivateComponentTypeIds(package_name);
      layers_removed = ApplicationContext::Get().RemoveLayersOwnedByPackage(package_name);
      ClearPrivateComponentPools(type_ids);
      if (layers_removed) {
        unload(&registrar);
        package.unload = nullptr;
      }
      Platform::DrainGpuResourceWork();
    }
    if (!layers_removed || GetLiveObjectCount(package_name) != 0) {
      package.info.ready = false;
      auto& manager = GetInstance();
      std::lock_guard lock(manager.mutex_);
      manager.loaded_packages_[package_name] = std::move(package);
      EVOENGINE_ERROR("Package activation failed; release its remaining objects and unload it: " + package_name)
      return false;
    }
    Serialization::UnregisterPackageOwnedTypes(package_name);
    NotifyTypeCleanup(package_name);
    Platform::RemoveGpuTimestampOwnerHistory(package_name);
    Profiler::GetInstance().UnregisterOwner(package_name);
    {
      auto& manager = GetInstance();
      std::lock_guard lock(manager.mutex_);
      manager.live_object_counts_.erase(package_name);
    }
    EVOENGINE_ERROR("Package activation failed: " + package_name)
    return false;
  }

  {
    auto& manager = GetInstance();
    std::lock_guard lock(manager.mutex_);
    manager.loaded_packages_[package_name] = std::move(package);
  }
  RestoreUnknownRuntimeTypes();
  EVOENGINE_WARNING("Runtime package loaded: " + package_name)
  return true;
}

bool PackageManager::LoadAll() {
  if (RejectPackageMutationWhenBusy("load")) {
    return false;
  }
  RefreshManifests();
  std::vector<std::string> manifest_package_names;
  {
    auto& manager = GetInstance();
    std::lock_guard lock(manager.mutex_);
    for (const auto& [package_name, _] : manager.package_manifests_) {
      manifest_package_names.emplace_back(package_name);
    }
  }
  std::sort(manifest_package_names.begin(), manifest_package_names.end());

  bool success = true;
  for (const auto& package_name : manifest_package_names) {
    success = Load(package_name) && success;
  }

  std::vector<std::filesystem::path> search_paths;
  {
    auto& manager = GetInstance();
    std::lock_guard lock(manager.mutex_);
    search_paths = manager.search_paths_;
  }
  if (search_paths.empty()) {
    search_paths = BuildDefaultSearchPaths();
  }

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
      bool covered_by_manifest = false;
      {
        auto& manager = GetInstance();
        std::lock_guard lock(manager.mutex_);
        for (const auto& [_, manifest] : manager.package_manifests_) {
          std::error_code manifest_ec;
          if (std::filesystem::absolute(manifest.library_path, manifest_ec) ==
              std::filesystem::absolute(entry.path(), manifest_ec)) {
            covered_by_manifest = true;
            break;
          }
        }
      }
      if (covered_by_manifest) {
        continue;
      }
      success = Load(entry.path()) && success;
    }
  }
  return success;
}

bool PackageManager::Unload(const std::string& package_name) {
  if (RejectPackageMutationWhenBusy("unload", package_name)) {
    return false;
  }

  Mutation mutation(package_name);
  if (!mutation)
    return false;
  PackageLifecycleCallbacks callbacks;
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
    callbacks = manager.lifecycle_callbacks_;
  }

  std::string dependent_name;
  if (HasLoadedDependents(package_name, &dependent_name)) {
    EVOENGINE_WARNING("Cannot unload runtime package [" + package_name + "] while dependent runtime package [" +
                      dependent_name + "] is loaded.")
    return false;
  }

  if (callbacks.prepare_unload && !callbacks.prepare_unload(package.info))
    return false;
  AssetManager::WaitForPendingLoads();
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
  if (callbacks.deactivate && !callbacks.deactivate(package.info)) {
    auto& manager = GetInstance();
    std::lock_guard lock(manager.mutex_);
    manager.loaded_packages_.at(package_name).info.ready = false;
    EVOENGINE_ERROR("Package deactivation failed; release its remaining objects and unload it: " + package_name)
    return false;
  }
  if (package.unload)
    package.unload(&registrar);
  Platform::DrainGpuResourceWork();
  Serialization::UnregisterPackageOwnedTypes(package_name);
  NotifyTypeCleanup(package_name);
  Platform::RemoveGpuTimestampOwnerHistory(package_name);
  Profiler::GetInstance().UnregisterOwner(package_name);

  {
    auto& manager = GetInstance();
    std::lock_guard lock(manager.mutex_);
    manager.loaded_packages_.erase(package_name);
    manager.live_object_counts_.erase(package_name);
  }
  EVOENGINE_WARNING("Runtime package unloaded: " + package_name)
  return true;
}

bool PackageManager::Reload(const std::string& package_name) {
  if (RejectPackageMutationWhenBusy("reload", package_name)) {
    return false;
  }
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
  if (RejectPackageMutationWhenBusy("unload")) {
    return;
  }
  while (true) {
    std::vector<std::string> package_names;
    {
      auto& manager = GetInstance();
      std::lock_guard lock(manager.mutex_);
      for (const auto& [name, _] : manager.loaded_packages_) {
        package_names.emplace_back(name);
      }
    }
    if (package_names.empty()) {
      return;
    }

    bool unloaded_any = false;
    for (const auto& name : package_names) {
      if (!HasLoadedDependents(name)) {
        unloaded_any = Unload(name) || unloaded_any;
      }
    }

    if (!unloaded_any) {
      EVOENGINE_WARNING("Unable to unload all runtime packages because their dependencies could not be resolved.")
      return;
    }
  }
}

void PackageManager::ScanAvailablePackages() {
  RefreshManifests();
}

std::vector<std::filesystem::path> PackageManager::GetSearchPaths() {
  auto& manager = GetInstance();
  std::lock_guard lock(manager.mutex_);
  if (manager.search_paths_.empty()) {
    return BuildDefaultSearchPaths();
  }
  return manager.search_paths_;
}

std::vector<AvailablePackageInfo> PackageManager::GetAvailablePackages() {
  auto& manager = GetInstance();
  std::vector<AvailablePackageInfo> ret_val;
  {
    std::lock_guard lock(manager.mutex_);
    ret_val.reserve(manager.package_manifests_.size());
    for (const auto& [package_name, manifest] : manager.package_manifests_) {
      std::error_code ec;
      AvailablePackageInfo info;
      info.name = package_name;
      info.version = manifest.version;
      info.description = manifest.description;
      info.dependencies = manifest.dependencies;
      info.manifest_path = manifest.manifest_path;
      info.library_path = manifest.library_path;
      info.library_exists = std::filesystem::exists(manifest.library_path, ec);
      info.loaded = manager.loaded_packages_.find(package_name) != manager.loaded_packages_.end();
      ret_val.emplace_back(std::move(info));
    }
  }
  std::sort(ret_val.begin(), ret_val.end(), [](const AvailablePackageInfo& lhs, const AvailablePackageInfo& rhs) {
    return lhs.name < rhs.name;
  });
  return ret_val;
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

uint64_t PackageManager::RegisterTypeCleanupCallback(std::function<void(const std::string&)> callback) {
  auto& manager = GetInstance();
  std::lock_guard lock(manager.mutex_);
  const auto id = ++manager.next_type_cleanup_id_;
  manager.type_cleanup_callbacks_.emplace(id, std::move(callback));
  return id;
}
void PackageManager::UnregisterTypeCleanupCallback(const uint64_t id) {
  auto& manager = GetInstance();
  std::lock_guard lock(manager.mutex_);
  manager.type_cleanup_callbacks_.erase(id);
}
void PackageManager::NotifyTypeCleanup(const std::string& package_name) {
  auto& manager = GetInstance();
  std::map<uint64_t, std::function<void(const std::string&)>> callbacks;
  {
    std::lock_guard lock(manager.mutex_);
    callbacks = manager.type_cleanup_callbacks_;
  }
  for (const auto& [id, callback] : callbacks)
    if (callback)
      callback(package_name);
}
