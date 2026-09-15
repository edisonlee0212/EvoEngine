#include "EditorPackage.hpp"
#include <string_view>
#include "EvoEngineEditorBuildIdentity.hpp"

#include "AssetManager.hpp"
#include "EditorLayer.hpp"
#include "NativeLibrary.hpp"
#include "Platform.hpp"

using namespace evo_engine;

const char* evo_engine::GetEditorSourceId() {
  return EVOENGINE_EDITOR_SOURCE_ID;
}

struct EditorPackageRegistrar::Module {
  void* handle = nullptr;
  std::filesystem::path original_path;
  std::filesystem::path loaded_path;
  size_t active_callbacks = 0;

  ~Module() {
    native_library::CloseLibrary(handle);
    if (!loaded_path.empty() && loaded_path != original_path) {
      std::error_code error;
      std::filesystem::remove(loaded_path, error);
      std::filesystem::remove(loaded_path.parent_path(), error);
    }
  }
};

EditorPackageRegistrar::EditorPackageRegistrar(std::shared_ptr<Module> module, std::string owner)
    : module_(std::move(module)), owner_(std::move(owner)) {
}

namespace {
template <typename Module, typename Result, typename... Args>
std::function<Result(Args...)> WithModule(std::shared_ptr<Module> module, std::function<Result(Args...)> handler) {
  struct Callback {
    std::shared_ptr<Module> module;
    std::function<Result(Args...)> handler;
  };
  const auto callback = std::make_shared<Callback>(Callback{std::move(module), std::move(handler)});
  return [callback](Args... args) -> Result {
    const auto active_callback = callback;
    struct Invocation {
      Module& module;
      explicit Invocation(Module& module) : module(module) {
        ++module.active_callbacks;
      }
      ~Invocation() {
        --module.active_callbacks;
      }
    } invocation(*active_callback->module);
    return active_callback->handler(std::forward<Args>(args)...);
  };
}
}  // namespace

bool EditorPackageRegistrar::RegisterInspector(const std::type_info& type, InspectorRegistry::Handler handler,
                                               std::string type_name) {
  return handler && InspectorRegistry::GetInstance().RegisterInspector(type, WithModule(module_, std::move(handler)),
                                                                       owner_, std::move(type_name));
}

bool EditorPackageRegistrar::RegisterAssetPreviewHandler(size_t type_id,
                                                         AssetPreviewRegistry::AssetPreviewHandler handler,
                                                         std::string type_name, uint32_t version) {
  return handler && AssetPreviewRegistry::RegisterAssetPreviewHandler(type_id, WithModule(module_, std::move(handler)),
                                                                      owner_, type_name, version);
}

void EditorPackageRegistrar::RegisterCleanup(std::function<void()> function) {
  if (function)
    cleanup_functions_.push_back(std::move(function));
}

struct EditorPackageCoordinator::Companion {
  std::string name;
  std::shared_ptr<EditorPackageRegistrar::Module> module;
  std::unique_ptr<EditorPackageRegistrar> registrar;
  EvoEngineEditorPackageUnloadFn unload = nullptr;
};

EditorPackageCoordinator::EditorPackageCoordinator() = default;
EditorPackageCoordinator::~EditorPackageCoordinator() = default;

bool EditorPackageCoordinator::ReadDeclaration(const LoadedPackageInfo& package, Declaration& declaration) {
  declaration = {};
  if (package.manifest_path.empty())
    return true;
  try {
    const auto runtime_manifest = YAML::LoadFile(package.manifest_path.string());
    if (!runtime_manifest["editor_manifest"])
      return true;
    const auto manifest_path =
        package.manifest_path.parent_path() / runtime_manifest["editor_manifest"].as<std::string>();
    const auto manifest = YAML::LoadFile(manifest_path.string());
    std::vector<std::string> declared_dependencies;
    if (const auto values = manifest["dependencies"]; values && !values.IsNull()) {
      if (!values.IsSequence())
        throw std::runtime_error("Editor dependencies must be a sequence.");
      for (const auto& value : values) {
        const auto dependency = value.as<std::string>();
        if (dependency == package.name ||
            std::find(package.dependencies.begin(), package.dependencies.end(), dependency) ==
                package.dependencies.end() ||
            std::find(declared_dependencies.begin(), declared_dependencies.end(), dependency) !=
                declared_dependencies.end())
          throw std::runtime_error("Invalid editor dependency: " + dependency);
        declared_dependencies.push_back(dependency);
      }
    }
    declaration.dependencies = std::move(declared_dependencies);
    const auto sdk = manifest["sdk_source_id"].as<std::string>();
    const auto compiler = manifest["compiler_id"].as<std::string>();
    const auto compiler_version = manifest["compiler_version"].as<std::string>();
    const auto configuration = manifest["configuration"].as<std::string>();
    const auto platform = manifest["platform"].as<std::string>();
    const auto architecture = manifest["architecture"].as<std::string>();
    const NativeBuildIdentity identity{
        sdk.c_str(),      compiler.c_str(),     compiler_version.c_str(),          configuration.c_str(),
        platform.c_str(), architecture.c_str(), manifest["with_editor"].as<bool>()};
    if (manifest["name"].as<std::string>() != package.name ||
        manifest["editor_api_version"].as<uint32_t>() != EVOENGINE_EDITOR_PACKAGE_API_VERSION ||
        manifest["package_source_id"].as<std::string>() != package.package_source_id ||
        manifest["editor_source_id"].as<std::string>() != GetEditorSourceId() ||
        !IsNativeBuildCompatible(GetNativeBuildIdentity(), identity))
      throw std::runtime_error("Editor companion identity mismatch.");
    declaration.editor_package_source_id = manifest["editor_package_source_id"].as<std::string>();
    if (declaration.editor_package_source_id.size() != 64)
      throw std::runtime_error("Invalid editor package source identity.");
    declaration.library = manifest_path.parent_path() / manifest["library"].as<std::string>();
    declaration.library_hash = manifest["library_sha256"].as<std::string>();
    return native_library::VerifyLibraryHash(declaration.library, declaration.library_hash);
  } catch (const std::exception& error) {
    EVOENGINE_ERROR("Cannot validate editor companion for " + package.name + ": " + error.what())
    return false;
  }
}

bool EditorPackageCoordinator::Activate(const LoadedPackageInfo& package) {
  auto module = std::make_shared<EditorPackageRegistrar::Module>();
  Declaration declaration;
  if (!ReadDeclaration(package, declaration))
    return false;
  module->original_path = declaration.library;
  if (module->original_path.empty())
    return true;
  for (const auto& dependency : declaration.dependencies) {
    if (std::none_of(companions_.begin(), companions_.end(), [&](const auto& companion) {
          return companion->name == dependency;
        })) {
      EVOENGINE_ERROR("Editor companion dependency is not loaded: " + package.name + " -> " + dependency)
      return false;
    }
  }
  module->loaded_path = module->original_path;
  if (package.loaded_path != package.original_path) {
    module->loaded_path = package.loaded_path.parent_path() / "Editor" / module->original_path.filename();
    std::filesystem::create_directories(module->loaded_path.parent_path());
    std::filesystem::copy_file(module->original_path, module->loaded_path);
  }
  if (!native_library::VerifyLibraryHash(module->loaded_path, declaration.library_hash))
    return false;
  if (module->loaded_path.empty() || !native_library::OpenLibrary(module->loaded_path, module->handle))
    return false;
  const auto get_descriptor = reinterpret_cast<EvoEngineEditorPackageGetDescriptorFn>(
      native_library::GetSymbol(module->handle, "EvoEngineEditorPackageGetDescriptor"));
  const auto load = reinterpret_cast<EvoEngineEditorPackageLoadFn>(
      native_library::GetSymbol(module->handle, "EvoEngineEditorPackageLoad"));
  const auto unload = reinterpret_cast<EvoEngineEditorPackageUnloadFn>(
      native_library::GetSymbol(module->handle, "EvoEngineEditorPackageUnload"));
  const auto* descriptor = get_descriptor ? get_descriptor() : nullptr;
  if (!descriptor || !load || !unload || descriptor->api_version != EVOENGINE_EDITOR_PACKAGE_API_VERSION ||
      !descriptor->runtime_module_identity || descriptor->runtime_module_identity != package.module_identity ||
      !descriptor->editor_source_id || std::string_view(descriptor->editor_source_id) != GetEditorSourceId() ||
      !descriptor->editor_package_source_id ||
      descriptor->editor_package_source_id != declaration.editor_package_source_id || !descriptor->package_name ||
      descriptor->package_name != package.name || !descriptor->package_source_id ||
      descriptor->package_source_id != package.package_source_id ||
      !IsNativeBuildCompatible(GetNativeBuildIdentity(), descriptor->build_identity)) {
    EVOENGINE_ERROR("Invalid editor companion descriptor: " + package.name)
    return false;
  }
  auto companion = std::make_unique<Companion>();
  companion->name = package.name;
  companion->module = module;
  companion->registrar.reset(new EditorPackageRegistrar(module, package.name + "/Editor"));
  companion->unload = unload;
  auto* registrar = companion->registrar.get();
  companions_.push_back(std::move(companion));
  return load(registrar);
}

bool EditorPackageCoordinator::PrepareUnload(const LoadedPackageInfo& package) {
  for (const auto& companion : companions_)
    if (companion->name == package.name && companion->module->active_callbacks != 0)
      return false;
  if (!ApplicationContext::Get().CanRemoveLayersOwnedByPackage(package.name + "/Editor"))
    return false;
  AssetManager::WaitForPendingLoads();
  if (const auto editor = ApplicationContext::Get().GetLayer<EditorLayer>())
    editor->ClearAssetInspectors();
  Platform::DrainGpuResourceWork();
  return true;
}

bool EditorPackageCoordinator::Deactivate(const LoadedPackageInfo& package, const bool shutting_down) {
  const auto found = std::find_if(companions_.begin(), companions_.end(), [&](const auto& companion) {
    return companion->name == package.name;
  });
  if (found == companions_.end())
    return true;
  if (!ApplicationContext::Get().RemoveLayersOwnedByPackage((*found)->registrar->owner_) && !shutting_down)
    return false;
  InspectorRegistry::GetInstance().UnregisterOwner((*found)->registrar->owner_);
  AssetPreviewRegistry::UnregisterAssetPreviewHandlersByOwner((*found)->registrar->owner_);
  for (auto it = (*found)->registrar->cleanup_functions_.rbegin(); it != (*found)->registrar->cleanup_functions_.rend();
       ++it)
    (*it)();
  (*found)->registrar->cleanup_functions_.clear();
  (*found)->unload();
  Platform::DrainGpuResourceWork();
  companions_.erase(found);
  return true;
}

void EditorPackageCoordinator::Shutdown() {
  while (!companions_.empty()) {
    LoadedPackageInfo package;
    package.name = companions_.back()->name;
    PrepareUnload(package);
    Deactivate(package, true);
  }
}

bool EditorPackageCoordinator::Initialize() {
  InspectorRegistry::GetInstance();
  AssetPreviewRegistry::GetInstance();
  const auto coordinator = std::make_shared<EditorPackageCoordinator>();
  if (!PackageManager::SetLifecycleCallbacks({[](const LoadedPackageInfo& package) {
                                                Declaration declaration;
                                                return ReadDeclaration(package, declaration);
                                              },
                                              [coordinator](const LoadedPackageInfo& package) {
                                                return coordinator->Activate(package);
                                              },
                                              [coordinator](const LoadedPackageInfo& package) {
                                                return coordinator->PrepareUnload(package);
                                              },
                                              [coordinator](const LoadedPackageInfo& package) {
                                                return coordinator->Deactivate(package);
                                              },
                                              [coordinator] {
                                                coordinator->Shutdown();
                                              }}))
    return false;
  static_cast<void>(ApplicationContext::Get().RegisterCleanupFunction([] {
    PackageManager::ShutdownLifecycleCallbacks();
  }));
  return true;
}
