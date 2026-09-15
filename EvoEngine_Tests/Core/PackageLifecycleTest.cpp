#include "EvoEngine_SDK_PCH.hpp"

#include <gtest/gtest.h>

#include "Application.hpp"
#include "AssetManager.hpp"
#include "EditorPackage.hpp"
#include "NativeLibrary.hpp"
#include "PackageManager.hpp"
#include "Serialization.hpp"

using namespace evo_engine;

namespace {
class PackageLifecycle : public testing::Test {
 protected:
  Application app;
  ApplicationContextScope scope{app};
  std::filesystem::path packages;

  void SetUp() override {
#ifdef EVOENGINE_TEST_APP_DIR
    packages = std::filesystem::path(EVOENGINE_TEST_APP_DIR) / "Packages";
#else
    GTEST_SKIP() << "Runtime packages are not configured.";
#endif
    ApplicationInitializationSettings settings;
    settings.application_mode = ApplicationMode::Headless;
    settings.allow_empty_project = true;
    settings.load_default_resources = false;
    settings.load_project_assets = false;
    settings.load_project_start_scene = false;
    app.Initialize(settings);
    PackageManager::Initialize({packages});
  }
  void TearDown() override {
    if (app.GetApplicationStatus() != Application::ExecutionStatus::Uninitialized)
      app.Terminate();
  }
};
}  // namespace

TEST_F(PackageLifecycle, ValidationPrecedesRuntimeRegistration) {
  bool validated = false;
  PackageLifecycleCallbacks callbacks;
  callbacks.validate = [&](const LoadedPackageInfo& info) {
    validated = true;
    EXPECT_FALSE(info.manifest_path.empty());
    EXPECT_FALSE(Serialization::HasSerializableType("TextureBaking"));
    return false;
  };
  ASSERT_TRUE(PackageManager::SetLifecycleCallbacks(std::move(callbacks)));
  EXPECT_FALSE(PackageManager::Load("TextureBaking"));
  EXPECT_TRUE(validated);
  EXPECT_TRUE(PackageManager::GetLoadedPackages().empty());
  EXPECT_FALSE(Serialization::HasSerializableType("TextureBaking"));
  PackageManager::ShutdownLifecycleCallbacks();
}

TEST_F(PackageLifecycle, ActivationFailureRollsBackTypesAndRejectsReentrantMutation) {
  bool deactivated = false;
  PackageLifecycleCallbacks callbacks;
  callbacks.activate = [](const LoadedPackageInfo&) {
    EXPECT_TRUE(Serialization::HasSerializableType("TextureBaking"));
    EXPECT_FALSE(PackageManager::Load("TextureBaking"));
    return false;
  };
  callbacks.deactivate = [&](const LoadedPackageInfo&) {
    deactivated = true;
    return true;
  };
  ASSERT_TRUE(PackageManager::SetLifecycleCallbacks(std::move(callbacks)));
  EXPECT_FALSE(PackageManager::Load("TextureBaking"));
  EXPECT_TRUE(deactivated);
  EXPECT_TRUE(PackageManager::GetLoadedPackages().empty());
  EXPECT_FALSE(Serialization::HasSerializableType("TextureBaking"));
  PackageManager::ShutdownLifecycleCallbacks();
}

TEST_F(PackageLifecycle, FailedActivationRetainsLibraryUntilLiveObjectsAreReleased) {
  std::shared_ptr<ISerializable> retained;
  PackageLifecycleCallbacks callbacks;
  callbacks.activate = [&](const LoadedPackageInfo&) {
    retained = Serialization::ProduceSerializable("TextureBaking");
    return false;
  };
  ASSERT_TRUE(PackageManager::SetLifecycleCallbacks(std::move(callbacks)));
  EXPECT_FALSE(PackageManager::Load("TextureBaking"));
  const auto loaded = PackageManager::GetLoadedPackages();
  ASSERT_EQ(loaded.size(), 1);
  EXPECT_FALSE(loaded.front().ready);
  EXPECT_FALSE(PackageManager::Load("TextureBaking"));
  EXPECT_FALSE(PackageManager::Unload("TextureBaking"));
  retained.reset();
  EXPECT_TRUE(PackageManager::Unload("TextureBaking"));
  PackageManager::ShutdownLifecycleCallbacks();
}

TEST_F(PackageLifecycle, RefusedUnloadPreservesActivation) {
  bool allow_unload = false;
  bool deactivated = false;
  PackageLifecycleCallbacks callbacks;
  callbacks.prepare_unload = [&](const LoadedPackageInfo&) {
    return allow_unload;
  };
  callbacks.deactivate = [&](const LoadedPackageInfo&) {
    deactivated = true;
    return true;
  };
  ASSERT_TRUE(PackageManager::SetLifecycleCallbacks(std::move(callbacks)));
  ASSERT_TRUE(PackageManager::Load("TextureBaking"));
  EXPECT_FALSE(PackageManager::Unload("TextureBaking"));
  EXPECT_FALSE(deactivated);
  EXPECT_TRUE(Serialization::HasSerializableType("TextureBaking"));
  allow_unload = true;
  EXPECT_TRUE(PackageManager::Unload("TextureBaking"));
  EXPECT_TRUE(deactivated);
  PackageManager::ShutdownLifecycleCallbacks();
}

TEST_F(PackageLifecycle, WeakObjectReferencesCanBeDestroyedAfterUnloadingAndReloading) {
  ASSERT_TRUE(PackageManager::Load("TextureBaking"));
  auto object = Serialization::ProduceSerializable("TextureBaking");
  ASSERT_NE(object, nullptr);
  std::weak_ptr<ISerializable> weak = object;
  object.reset();
  ASSERT_TRUE(PackageManager::Unload("TextureBaking"));
  EXPECT_TRUE(weak.expired());
  ASSERT_TRUE(PackageManager::Load("TextureBaking"));
  weak.reset();
  EXPECT_TRUE(PackageManager::Unload("TextureBaking"));
}

TEST_F(PackageLifecycle, FrameCallbacksMustDeferPackageMutation) {
  bool callback_ran = false;
  bool deferred_load = false;
  app.RegisterUpdateFunction([&] {
    callback_ran = true;
    EXPECT_TRUE(app.IsDispatchingLayers());
    EXPECT_FALSE(PackageManager::Load("TextureBaking"));
    app.QueueEndOfLoopAction([&] {
      EXPECT_FALSE(app.IsDispatchingLayers());
      deferred_load = PackageManager::Load("TextureBaking");
    });
  });
  ASSERT_TRUE(app.Loop());
  EXPECT_TRUE(callback_ran);
  EXPECT_TRUE(deferred_load);
  EXPECT_TRUE(PackageManager::Unload("TextureBaking"));
}

TEST_F(PackageLifecycle, CompanionReferencesTheExactRuntimeModuleInItsShadowGeneration) {
  ASSERT_TRUE(EditorPackageCoordinator::Initialize());
  ASSERT_TRUE(PackageManager::Load("TextureBaking"));
  const auto package = PackageManager::GetLoadedPackages().front();
  const auto manifest = YAML::LoadFile((packages / "Editor/TextureBaking.eveeditorpackage").string());
  const auto companion_path = package.loaded_path.parent_path() / "Editor" / manifest["library"].as<std::string>();
  ASSERT_TRUE(std::filesystem::is_regular_file(companion_path));
  void* library = nullptr;
  ASSERT_TRUE(native_library::OpenLibrary(companion_path, library));
  const auto reference = std::shared_ptr<void>(library, native_library::CloseLibrary);
  const auto get_descriptor = reinterpret_cast<EvoEngineEditorPackageGetDescriptorFn>(
      native_library::GetSymbol(library, "EvoEngineEditorPackageGetDescriptor"));
  ASSERT_NE(get_descriptor, nullptr);
  EXPECT_EQ(get_descriptor()->runtime_module_identity, package.module_identity);
}

TEST_F(PackageLifecycle, CompanionLayerReferenceRefusesUnloadAndWeakReferencesOutliveTheDll) {
  ASSERT_TRUE(EditorPackageCoordinator::Initialize());
  ASSERT_TRUE(PackageManager::Load("Universe"));
  auto layer = app.GetLayers().back();
  ASSERT_EQ(layer->GetLayerName(), "Universe View");
  std::weak_ptr<ILayer> weak = layer;
  EXPECT_FALSE(PackageManager::Unload("Universe"));
  layer.reset();
  EXPECT_TRUE(PackageManager::Unload("Universe"));
  EXPECT_TRUE(weak.expired());
  weak.reset();
}

TEST_F(PackageLifecycle, WeakLayerReferencesCanBeDestroyedAfterUnloading) {
  ASSERT_TRUE(PackageManager::Load("LSystem"));
  ASSERT_FALSE(app.GetLayers().empty());
  EXPECT_EQ(app.GetLayers().back()->GetLayerName(), "LSystem Layer");
  std::weak_ptr<ILayer> layer = app.GetLayers().back();
  ASSERT_TRUE(PackageManager::Unload("LSystem"));
  EXPECT_TRUE(layer.expired());
  layer.reset();
}

TEST_F(PackageLifecycle, LSystemCompanionPreservesSerializedDescriptorSettings) {
  ASSERT_TRUE(EditorPackageCoordinator::Initialize());
  ASSERT_TRUE(PackageManager::Load("LSystem"));
  auto asset = AssetManager::CreateTemporaryAsset("ScotsPineDescriptor");
  ASSERT_NE(asset, nullptr);
  const auto* inspector = InspectorRegistry::GetInstance().FindInspector(typeid(*asset));
  ASSERT_NE(inspector, nullptr);
  EXPECT_EQ(inspector->owner_name, "LSystem/Editor");
  const auto settings = YAML::Load("live_preview: true\nlive_preview_rate_hz: 23.5\ngrid_rows: 7\ngrid_cols: 9\n");
  Serialization::DeserializeObject(settings, *asset);
  YAML::Emitter output;
  output << YAML::BeginMap;
  Serialization::SerializeObject(output, *asset);
  output << YAML::EndMap;
  const auto saved = YAML::Load(output.c_str());
  EXPECT_TRUE(saved["live_preview"].as<bool>());
  EXPECT_FLOAT_EQ(saved["live_preview_rate_hz"].as<float>(), 23.5f);
  EXPECT_EQ(saved["grid_rows"].as<int>(), 7);
  EXPECT_EQ(saved["grid_cols"].as<int>(), 9);
  asset.reset();
  EXPECT_TRUE(PackageManager::Unload("LSystem"));
}

TEST_F(PackageLifecycle, CompanionInspectorSurvivesRefusedUnloadAndIsReplacedOnReload) {
  ASSERT_TRUE(EditorPackageCoordinator::Initialize());
  ASSERT_TRUE(PackageManager::Load("TextureBaking"));
  auto object = Serialization::ProduceSerializable("TextureBaking");
  ASSERT_NE(object, nullptr);
  auto& registry = InspectorRegistry::GetInstance();
  ASSERT_TRUE(registry.HasInspector(typeid(*object)));
  EXPECT_EQ(registry.FindInspector(typeid(*object))->owner_name, "TextureBaking/Editor");
  EXPECT_FALSE(PackageManager::Unload("TextureBaking"));
  EXPECT_TRUE(registry.HasInspector(typeid(*object)));
  object.reset();
  ASSERT_TRUE(PackageManager::Reload("TextureBaking"));
  object = Serialization::ProduceSerializable("TextureBaking");
  EXPECT_TRUE(registry.HasInspector(typeid(*object)));
  object.reset();
  ASSERT_TRUE(PackageManager::Unload("TextureBaking"));
  EXPECT_EQ(registry.UnregisterOwner("TextureBaking/Editor"), 0);
}

TEST_F(PackageLifecycle, ShutdownRemovesCompanionBeforeApplicationCleanup) {
  ASSERT_TRUE(EditorPackageCoordinator::Initialize());
  ASSERT_TRUE(PackageManager::Load("TextureBaking"));
  bool checked = false;
  static_cast<void>(app.RegisterCleanupFunction([&] {
    checked = true;
    EXPECT_EQ(InspectorRegistry::GetInstance().UnregisterOwner("TextureBaking/Editor"), 0);
  }));
  app.Terminate();
  EXPECT_TRUE(checked);
}

TEST_F(PackageLifecycle, DeclaredCompanionMustExistAndMatchItsRuntimePackageAndHash) {
  ASSERT_TRUE(EditorPackageCoordinator::Initialize());
  struct Scratch {
    std::filesystem::path path =
        std::filesystem::temp_directory_path() /
        ("EvoEngineCompanionTest_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    ~Scratch() {
      std::filesystem::remove_all(path);
    }
  } scratch;
  std::filesystem::create_directories(scratch.path / "Editor");
  const auto runtime_manifest = YAML::LoadFile((packages / "TextureBaking.evepackage").string());
  const auto editor_manifest = YAML::LoadFile((packages / "Editor/TextureBaking.eveeditorpackage").string());
  for (const auto& relative : {std::filesystem::path("TextureBaking.evepackage"),
                               std::filesystem::path(runtime_manifest["library"].as<std::string>()),
                               std::filesystem::path("Editor") / editor_manifest["library"].as<std::string>()})
    std::filesystem::copy_file(packages / relative, scratch.path / relative);
  PackageManager::Initialize({scratch.path});
  EXPECT_FALSE(PackageManager::Load("TextureBaking"));
  EXPECT_FALSE(Serialization::HasSerializableType("TextureBaking"));
  for (const auto* field : {"editor_api_version", "package_source_id", "sdk_source_id", "editor_source_id",
                            "editor_package_source_id", "library_sha256", "library"}) {
    auto invalid = YAML::Clone(editor_manifest);
    invalid[field] = "invalid";
    {
      std::ofstream output(scratch.path / "Editor/TextureBaking.eveeditorpackage");
      output << invalid;
    }
    EXPECT_FALSE(PackageManager::Load("TextureBaking")) << field;
    EXPECT_TRUE(PackageManager::GetLoadedPackages().empty()) << field;
    EXPECT_FALSE(Serialization::HasSerializableType("TextureBaking")) << field;
  }
  for (const auto* dependencies : {"[TextureBaking]", "[MissingCompanion]", "invalid"}) {
    auto invalid = YAML::Clone(editor_manifest);
    invalid["dependencies"] = YAML::Load(dependencies);
    {
      std::ofstream output(scratch.path / "Editor/TextureBaking.eveeditorpackage");
      output << invalid;
    }
    EXPECT_FALSE(PackageManager::Load("TextureBaking")) << dependencies;
    EXPECT_TRUE(PackageManager::GetLoadedPackages().empty());
    EXPECT_FALSE(Serialization::HasSerializableType("TextureBaking"));
  }
}

TEST_F(PackageLifecycle, EditorDependencyRequiresALoadedCompanion) {
  ASSERT_TRUE(EditorPackageCoordinator::Initialize());
  struct Scratch {
    std::filesystem::path path = std::filesystem::temp_directory_path() /
                                 ("EvoEngineEditorDependencyTest_" +
                                  std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    ~Scratch() {
      PackageManager::UnloadAll();
      std::error_code error;
      std::filesystem::remove_all(path, error);
    }
  } scratch;
  std::filesystem::create_directories(scratch.path / "Editor");
  for (const auto* name : {"TextureBaking", "BillboardClouds"}) {
    auto runtime = YAML::LoadFile((packages / (std::string(name) + ".evepackage")).string());
    auto editor = YAML::LoadFile((packages / "Editor" / (std::string(name) + ".eveeditorpackage")).string());
    std::filesystem::copy_file(packages / runtime["library"].as<std::string>(),
                               scratch.path / runtime["library"].as<std::string>());
    std::filesystem::copy_file(packages / "Editor" / editor["library"].as<std::string>(),
                               scratch.path / "Editor" / editor["library"].as<std::string>());
    if (std::string(name) == "TextureBaking") {
      runtime["dependencies"] = YAML::Load("[BillboardClouds]");
      editor["dependencies"] = YAML::Load("[BillboardClouds]");
    } else {
      runtime.remove("editor_manifest");
    }
    std::ofstream(scratch.path / (std::string(name) + ".evepackage")) << runtime;
    std::ofstream(scratch.path / "Editor" / (std::string(name) + ".eveeditorpackage")) << editor;
  }
  PackageManager::Initialize({scratch.path});
  EXPECT_FALSE(PackageManager::Load("TextureBaking"));
  EXPECT_FALSE(Serialization::HasSerializableType("TextureBaking"));
  ASSERT_TRUE(PackageManager::Unload("BillboardClouds"));
  std::filesystem::copy_file(packages / "BillboardClouds.evepackage", scratch.path / "BillboardClouds.evepackage",
                             std::filesystem::copy_options::overwrite_existing);
  PackageManager::Initialize({scratch.path});
  ASSERT_TRUE(PackageManager::Load("TextureBaking"));
  EXPECT_TRUE(Serialization::HasSerializableType("TextureBaking"));
  EXPECT_FALSE(PackageManager::Unload("BillboardClouds"));
  ASSERT_TRUE(PackageManager::Unload("TextureBaking"));
  EXPECT_TRUE(PackageManager::Unload("BillboardClouds"));
}
