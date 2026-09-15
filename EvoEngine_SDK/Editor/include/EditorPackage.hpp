#pragma once

#include "AssetPreviewRegistry.hpp"
#include "InspectorRegistry.hpp"
#include "PackageManager.hpp"

namespace evo_engine {
constexpr uint32_t EVOENGINE_EDITOR_PACKAGE_API_VERSION = 2;
EVOENGINE_EDITOR_API const char* GetEditorSourceId();

struct EditorPackageDescriptor {
  uint32_t api_version = EVOENGINE_EDITOR_PACKAGE_API_VERSION;
  const char* package_name = nullptr;
  const char* package_source_id = nullptr;
  NativeBuildIdentity build_identity{};
  const void* runtime_module_identity = nullptr;
  const char* editor_source_id = nullptr;
  const char* editor_package_source_id = nullptr;
};

class EVOENGINE_EDITOR_API EditorPackageRegistrar final {
  friend class EditorPackageCoordinator;
  struct Module;
  std::shared_ptr<Module> module_;
  std::string owner_;
  std::vector<std::function<void()>> cleanup_functions_;
  explicit EditorPackageRegistrar(std::shared_ptr<Module> module, std::string owner);

 public:
  EditorPackageRegistrar(const EditorPackageRegistrar&) = delete;
  EditorPackageRegistrar& operator=(const EditorPackageRegistrar&) = delete;
  template <typename T>
  bool RegisterInspector(std::function<bool(InspectorContext&, T&)> handler, std::string type_name = {}) {
    if (!handler)
      return false;
    return RegisterInspector(
        typeid(T),
        [handler = std::move(handler)](InspectorContext& context, void* target) {
          return handler(context, *static_cast<T*>(target));
        },
        std::move(type_name));
  }
  bool RegisterInspector(const std::type_info& type, InspectorRegistry::Handler handler, std::string type_name = {});

  template <typename T>
  bool RegisterLayer(const std::string& name) {
    return ApplicationContext::Get().PushLayer<T>(name, owner_, module_) != nullptr;
  }
  template <typename T>
  bool RegisterAssetPreviewHandler(
      std::function<std::shared_ptr<Texture2D>(const std::shared_ptr<T>&, const OffscreenPreviewSettings&)> handler,
      std::string type_name = {}, uint32_t version = 0) {
    if (!handler)
      return false;
    return RegisterAssetPreviewHandler(
        typeid(T).hash_code(),
        [handler = std::move(handler)](const std::shared_ptr<IAsset>& asset, const OffscreenPreviewSettings& settings) {
          const auto typed_asset = std::dynamic_pointer_cast<T>(asset);
          return typed_asset ? handler(typed_asset, settings) : nullptr;
        },
        std::move(type_name), version);
  }
  bool RegisterAssetPreviewHandler(size_t type_id, AssetPreviewRegistry::AssetPreviewHandler handler,
                                   std::string type_name = {}, uint32_t version = 0);
  void RegisterCleanup(std::function<void()> function);
};

using EvoEngineEditorPackageGetDescriptorFn = const EditorPackageDescriptor* (*)();
using EvoEngineEditorPackageLoadFn = bool (*)(EditorPackageRegistrar*);
using EvoEngineEditorPackageUnloadFn = void (*)();

class EditorPackageCoordinator final {
 public:
  EVOENGINE_EDITOR_API static bool Initialize();

 private:
  struct Companion;
  struct Declaration {
    std::filesystem::path library;
    std::string library_hash;
    std::string editor_package_source_id;
    std::vector<std::string> dependencies;
  };
  std::vector<std::unique_ptr<Companion>> companions_;
  static bool ReadDeclaration(const LoadedPackageInfo& package, Declaration& declaration);
  bool Activate(const LoadedPackageInfo& package);
  bool PrepareUnload(const LoadedPackageInfo& package);
  bool Deactivate(const LoadedPackageInfo& package, bool shutting_down = false);
  void Shutdown();

 public:
  EditorPackageCoordinator();
  ~EditorPackageCoordinator();
};
}  // namespace evo_engine
