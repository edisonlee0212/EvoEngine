#include "EvoEngine_SDK_PCH.hpp"

#include <gtest/gtest.h>

#include "Animation.hpp"
#include "AnimationPlayer.hpp"
#include "Animator.hpp"
#include "Application.hpp"
#include "ApplicationContext.hpp"
#include "AssetPreviewRegistry.hpp"
#include "Cubemap.hpp"
#include "EditorLayer.hpp"
#include "EnvironmentalMap.hpp"
#include "GaussianSplat.hpp"
#include "GaussianSplatRenderer.hpp"
#include "GlobalReflectionProbe.hpp"
#include "IAsset.hpp"
#include "ILayer.hpp"
#include "IPrivateComponent.hpp"
#include "ISystem.hpp"
#include "InspectorRegistry.hpp"
#include "Json.hpp"
#include "LightProbe.hpp"
#include "Lights.hpp"
#include "LodGroup.hpp"
#include "Material.hpp"
#include "Mesh.hpp"
#include "MeshRenderer.hpp"
#include "PackageManager.hpp"
#include "Particles.hpp"
#include "PlayerController.hpp"
#include "PointCloud.hpp"
#include "PointCloudScanner.hpp"
#include "Prefab.hpp"
#include "RenderLayer.hpp"
#include "Scene.hpp"
#include "Shader.hpp"
#include "SkinnedMesh.hpp"
#include "SkinnedMeshRenderer.hpp"
#include "SkyIllumination.hpp"
#include "Strands.hpp"
#include "StrandsRenderer.hpp"
#include "Texture2D.hpp"
#include "UnknownPrivateComponent.hpp"
#include "WayPoints.hpp"

using namespace evo_engine;

namespace {
class TestAsset final : public IAsset {
 public:
  int inspect_count = 0;
  bool inspect_result = false;
};

class TestPrivateComponent final : public IPrivateComponent {
 public:
  int inspect_count = 0;
  bool inspect_result = false;
};

class TestSystem final : public ISystem {
 public:
  int inspect_count = 0;
  bool inspect_result = false;
};

class TestLayer final : public ILayer {
 public:
  int inspect_count = 0;
};

class InspectorOnlyEditorLayer final : public EditorLayer {
  void OnCreate() override {
  }
  void OnDestroy() override {
  }
};

class OffsetBase {
 public:
  virtual ~OffsetBase() = default;

  int offset_base_value = 0;
};

class OffsetAsset final : public OffsetBase, public IAsset {
 public:
  int inspect_count = 0;
  int handler_value = 0;
};
}  // namespace

TEST(InspectorRegistry, PackageUnloadRemovesEditorHandlersBeforeClosingLibrary) {
#ifdef EVOENGINE_TEST_APP_DIR
  Application app;
  ApplicationContextScope scope(app);
  ApplicationInitializationSettings settings;
  settings.application_mode = ApplicationMode::Headless;
  settings.allow_empty_project = true;
  settings.load_default_resources = false;
  settings.load_project_assets = false;
  settings.load_project_start_scene = false;
  app.Initialize(settings);
  PackageManager::Initialize({std::filesystem::path(EVOENGINE_TEST_APP_DIR) / "Packages"});
  ASSERT_TRUE(PackageManager::Load("BillboardClouds"));
  auto lifetime = std::make_shared<int>(1);
  std::weak_ptr<int> retained = lifetime;
  auto& registry = InspectorRegistry::GetInstance();
  registry.RegisterInspector<TestAsset>(
      [lifetime](InspectorContext&, TestAsset&) {
        return false;
      },
      "BillboardClouds");
  registry.RegisterInspector<TestLayer>(
      [](InspectorContext&, TestLayer&) {
        return false;
      },
      "kept-owner");
  AssetPreviewRegistry::RegisterAssetPreviewHandler<TestAsset>(
      [lifetime](const std::shared_ptr<TestAsset>&, const OffscreenPreviewSettings&) {
        return std::shared_ptr<Texture2D>{};
      },
      "BillboardClouds");
  lifetime.reset();
  EXPECT_FALSE(retained.expired());
  bool removed_callback_called = false;
  const auto callback = PackageManager::RegisterTypeCleanupCallback([&](const std::string&) {
    removed_callback_called = true;
  });
  PackageManager::UnregisterTypeCleanupCallback(callback);
  ASSERT_TRUE(PackageManager::Unload("BillboardClouds"));
  EXPECT_FALSE(registry.HasInspector<TestAsset>());
  EXPECT_FALSE(AssetPreviewRegistry::HasAssetPreviewHandler<TestAsset>());
  EXPECT_TRUE(registry.HasInspector<TestLayer>());
  EXPECT_TRUE(retained.expired());
  EXPECT_FALSE(removed_callback_called);
  app.Terminate();
#else
  GTEST_SKIP() << "The editor application and its runtime packages are not configured.";
#endif
}

TEST(InspectorRegistry, MissingInspectorsDoNotCallConcreteInspectors) {
  Application app;
  ApplicationContextScope scope(app);
  auto& registry = InspectorRegistry::GetInstance();
  registry.Clear();
  InspectorContext context;

  TestAsset asset;
  asset.inspect_result = true;
  EXPECT_FALSE(registry.Inspect(context, asset));
  EXPECT_EQ(asset.inspect_count, 0);

  TestPrivateComponent component;
  component.inspect_result = true;
  EXPECT_FALSE(registry.Inspect(context, component));
  EXPECT_EQ(component.inspect_count, 0);

  TestSystem system;
  system.inspect_result = true;
  EXPECT_FALSE(registry.Inspect(context, system));
  EXPECT_EQ(system.inspect_count, 0);

  TestLayer layer;
  EXPECT_FALSE(registry.Inspect(context, layer));
  EXPECT_EQ(layer.inspect_count, 0);
}

TEST(InspectorRegistry, BatchInspectorsRequireExplicitRegistrationAndReceiveEveryTarget) {
  Application app;
  ApplicationContextScope scope(app);
  auto& registry = InspectorRegistry::GetInstance();
  registry.Clear();
  InspectorContext context;
  std::vector<std::shared_ptr<IPrivateComponent>> components = {std::make_shared<TestPrivateComponent>(),
                                                                std::make_shared<TestPrivateComponent>()};

  EXPECT_FALSE(registry.InspectBatch(context, components));
  int calls = 0;
  ASSERT_TRUE(registry.RegisterBatchInspector<TestPrivateComponent>(
      [&](InspectorContext&, const std::vector<std::reference_wrapper<TestPrivateComponent>>& targets) {
        calls++;
        EXPECT_EQ(targets.size(), 2u);
        for (auto& target : targets)
          target.get().inspect_count++;
        return true;
      },
      "test-owner", "TestPrivateComponent"));
  EXPECT_TRUE(registry.HasBatchInspector<TestPrivateComponent>());
  EXPECT_TRUE(registry.InspectBatch(context, components));
  EXPECT_EQ(calls, 1);
  EXPECT_EQ(static_cast<TestPrivateComponent&>(*components[0]).inspect_count, 1);
  EXPECT_EQ(static_cast<TestPrivateComponent&>(*components[1]).inspect_count, 1);
  EXPECT_EQ(registry.UnregisterOwner("test-owner"), 1u);
  EXPECT_FALSE(registry.HasBatchInspector<TestPrivateComponent>());
}

TEST(InspectorRegistry, RuntimeTypeRegistrationDoesNotInstallImplicitInspectors) {
  Application app;
  ApplicationContextScope scope(app);
  auto& registry = InspectorRegistry::GetInstance();
  registry.Clear();

  app.RegisterAsset<TestAsset>("TestAsset", {".testasset"});
  app.RegisterPrivateComponent<TestPrivateComponent>("TestPrivateComponent");
  app.RegisterSystem<TestSystem>("TestSystem");
  ASSERT_TRUE(app.PushLayer<TestLayer>("TestLayer"));
  EXPECT_FALSE(registry.HasInspector<TestAsset>());
  EXPECT_FALSE(registry.HasInspector<TestPrivateComponent>());
  EXPECT_FALSE(registry.HasInspector<TestSystem>());
  EXPECT_FALSE(registry.HasInspector<TestLayer>());
}

TEST(InspectorRegistry, PushedBuiltInLayersDoNotRegisterImplicitInspectors) {
  Application app;
  ApplicationContextScope scope(app);
  auto& registry = InspectorRegistry::GetInstance();
  registry.Clear();

  ASSERT_TRUE(app.PushLayer<RenderLayer>("Render Layer"));
  ASSERT_TRUE(app.PushLayer<EditorLayer>("Editor Layer"));

  EXPECT_EQ(registry.FindInspector(typeid(RenderLayer)), nullptr);
  EXPECT_EQ(registry.FindInspector(typeid(EditorLayer)), nullptr);
}

TEST(InspectorRegistry, EditorLayerRegistersSdkInspectorsDuringStartup) {
  Application app;
  ApplicationContextScope scope(app);
  app.PushLayer<InspectorOnlyEditorLayer>("Inspector registration");
  ApplicationInitializationSettings settings;
  settings.allow_empty_project = true;
  settings.load_default_resources = false;
  app.Initialize(settings);

  EXPECT_NE(InspectorRegistry::GetInstance().FindInspector(typeid(Animation)), nullptr);
  EXPECT_NE(InspectorRegistry::GetInstance().FindInspector(typeid(AnimationPlayer)), nullptr);
  EXPECT_NE(InspectorRegistry::GetInstance().FindInspector(typeid(Animator)), nullptr);
  EXPECT_NE(InspectorRegistry::GetInstance().FindInspector(typeid(Cubemap)), nullptr);
  EXPECT_NE(InspectorRegistry::GetInstance().FindInspector(typeid(DirectionalLight)), nullptr);
  EXPECT_NE(InspectorRegistry::GetInstance().FindInspector(typeid(EnvironmentalMap)), nullptr);
  EXPECT_NE(InspectorRegistry::GetInstance().FindInspector(typeid(EditorLayer)), nullptr);
  EXPECT_EQ(InspectorRegistry::GetInstance().FindInspector(typeid(Json)), nullptr);
  EXPECT_NE(InspectorRegistry::GetInstance().FindInspector(typeid(LightProbe)), nullptr);
  EXPECT_NE(InspectorRegistry::GetInstance().FindInspector(typeid(LodGroup)), nullptr);
  EXPECT_NE(InspectorRegistry::GetInstance().FindInspector(typeid(Material)), nullptr);
  EXPECT_NE(InspectorRegistry::GetInstance().FindInspector(typeid(Mesh)), nullptr);
  EXPECT_NE(InspectorRegistry::GetInstance().FindInspector(typeid(MeshRenderer)), nullptr);
  EXPECT_NE(InspectorRegistry::GetInstance().FindInspector(typeid(Particles)), nullptr);
  EXPECT_NE(InspectorRegistry::GetInstance().FindInspector(typeid(PointCloud)), nullptr);
  EXPECT_NE(InspectorRegistry::GetInstance().FindInspector(typeid(GaussianSplat)), nullptr);
  EXPECT_NE(InspectorRegistry::GetInstance().FindInspector(typeid(GaussianSplatRenderer)), nullptr);
  EXPECT_NE(InspectorRegistry::GetInstance().FindInspector(typeid(PointCloudScanner)), nullptr);
  EXPECT_NE(InspectorRegistry::GetInstance().FindInspector(typeid(Prefab)), nullptr);
  EXPECT_NE(InspectorRegistry::GetInstance().FindInspector(typeid(PointLight)), nullptr);
  EXPECT_NE(InspectorRegistry::GetInstance().FindInspector(typeid(PlayerController)), nullptr);
  EXPECT_NE(InspectorRegistry::GetInstance().FindInspector(typeid(GlobalReflectionProbe)), nullptr);
  EXPECT_NE(InspectorRegistry::GetInstance().FindInspector(typeid(RenderLayer)), nullptr);
  EXPECT_NE(InspectorRegistry::GetInstance().FindInspector(typeid(Scene)), nullptr);
  EXPECT_NE(InspectorRegistry::GetInstance().FindInspector(typeid(Shader)), nullptr);
  EXPECT_NE(InspectorRegistry::GetInstance().FindInspector(typeid(SkinnedMesh)), nullptr);
  EXPECT_NE(InspectorRegistry::GetInstance().FindInspector(typeid(SkinnedMeshRenderer)), nullptr);
  EXPECT_NE(InspectorRegistry::GetInstance().FindInspector(typeid(SkyIllumination)), nullptr);
  EXPECT_NE(InspectorRegistry::GetInstance().FindInspector(typeid(SpotLight)), nullptr);
  EXPECT_NE(InspectorRegistry::GetInstance().FindInspector(typeid(Strands)), nullptr);
  EXPECT_NE(InspectorRegistry::GetInstance().FindInspector(typeid(StrandsRenderer)), nullptr);
  EXPECT_NE(InspectorRegistry::GetInstance().FindInspector(typeid(Texture2D)), nullptr);
  EXPECT_NE(InspectorRegistry::GetInstance().FindInspector(typeid(UnknownAsset)), nullptr);
  EXPECT_NE(InspectorRegistry::GetInstance().FindInspector(typeid(UnknownLayer)), nullptr);
  EXPECT_NE(InspectorRegistry::GetInstance().FindInspector(typeid(UnknownPrivateComponent)), nullptr);
  EXPECT_NE(InspectorRegistry::GetInstance().FindInspector(typeid(UnknownSystem)), nullptr);
  EXPECT_NE(InspectorRegistry::GetInstance().FindInspector(typeid(WayPoints)), nullptr);
}

TEST(InspectorRegistry, UsesExactRegisteredInspectorBeforeDefaultInspector) {
  Application app;
  ApplicationContextScope scope(app);
  auto& registry = InspectorRegistry::GetInstance();
  registry.Clear();
  InspectorContext context;

  int handler_count = 0;
  ASSERT_TRUE(registry.RegisterInspector<TestAsset>(
      [&](InspectorContext&, TestAsset& asset) {
        ++handler_count;
        asset.inspect_result = true;
        return true;
      },
      "test-owner", "TestAsset"));

  TestAsset asset;
  EXPECT_TRUE(registry.HasInspector<TestAsset>());
  ASSERT_NE(registry.FindInspector(typeid(TestAsset)), nullptr);
  EXPECT_TRUE(registry.Inspect(context, asset));
  EXPECT_EQ(handler_count, 1);
  EXPECT_EQ(asset.inspect_count, 0);
  EXPECT_TRUE(asset.inspect_result);
}

TEST(InspectorRegistry, ExactRegisteredInspectorReceivesMostDerivedObject) {
  Application app;
  ApplicationContextScope scope(app);
  auto& registry = InspectorRegistry::GetInstance();
  registry.Clear();
  InspectorContext context;

  ASSERT_TRUE(registry.RegisterInspector<OffsetAsset>([](InspectorContext&, OffsetAsset& asset) {
    asset.handler_value = 42;
    return true;
  }));

  OffsetAsset asset;
  EXPECT_TRUE(registry.Inspect(context, static_cast<IAsset&>(asset)));
  EXPECT_EQ(asset.inspect_count, 0);
  EXPECT_EQ(asset.handler_value, 42);
  EXPECT_EQ(asset.offset_base_value, 0);
}

TEST(InspectorRegistry, BaseCategoryInspectorDoesNotHandleDerivedTypeWithoutExactRegistration) {
  Application app;
  ApplicationContextScope scope(app);
  auto& registry = InspectorRegistry::GetInstance();
  registry.Clear();
  InspectorContext context;

  int handler_count = 0;
  ASSERT_TRUE(registry.RegisterInspector<IAsset>([&](InspectorContext&, IAsset&) {
    ++handler_count;
    return true;
  }));

  TestAsset asset;
  EXPECT_FALSE(registry.Inspect(context, asset));
  EXPECT_EQ(handler_count, 0);
  EXPECT_EQ(asset.inspect_count, 0);
}

TEST(InspectorRegistry, UnregistersPackageOwnedInspectors) {
  Application app;
  ApplicationContextScope scope(app);
  auto& registry = InspectorRegistry::GetInstance();
  registry.Clear();
  InspectorContext context;

  ASSERT_TRUE(registry.RegisterInspector<TestAsset>(
      [](InspectorContext&, TestAsset&) {
        return true;
      },
      "owner-a"));
  ASSERT_TRUE(registry.RegisterInspector<TestPrivateComponent>(
      [](InspectorContext&, TestPrivateComponent&) {
        return true;
      },
      "owner-a"));
  ASSERT_TRUE(registry.RegisterInspector<TestSystem>(
      [](InspectorContext&, TestSystem&) {
        return true;
      },
      "owner-b"));

  EXPECT_EQ(registry.UnregisterOwner("owner-a"), 2);
  EXPECT_FALSE(registry.HasInspector<TestAsset>());
  EXPECT_FALSE(registry.HasInspector<TestPrivateComponent>());
  EXPECT_TRUE(registry.HasInspector<TestSystem>());

  TestAsset asset;
  EXPECT_FALSE(registry.Inspect(context, asset));
  EXPECT_EQ(asset.inspect_count, 0);

  TestSystem system;
  EXPECT_TRUE(registry.Inspect(context, system));
  EXPECT_EQ(system.inspect_count, 0);
}

TEST(InspectorRegistry, ApplicationCleanupIsOrderedAndCanBeUnregistered) {
  std::vector<int> order;
  {
    Application application;
    static_cast<void>(application.RegisterCleanupFunction([&] {
      order.push_back(1);
    }));
    const auto removed = application.RegisterCleanupFunction([&] {
      order.push_back(2);
    });
    static_cast<void>(application.RegisterCleanupFunction([&] {
      order.push_back(3);
    }));
    application.UnregisterCleanupFunction(removed);
  }
  EXPECT_EQ(order, (std::vector<int>{3, 1}));
}

TEST(InspectorRegistry, ApplicationsIsolateAndReleaseInspectorCallbacks) {
  Application first;
  ApplicationContextScope first_scope(first);
  auto lifetime = std::make_shared<int>(1);
  std::weak_ptr<int> released;
  ASSERT_TRUE(InspectorRegistry::GetInstance().RegisterInspector<TestAsset>([lifetime](InspectorContext&, TestAsset&) {
    return *lifetime == 1;
  }));
  {
    Application second;
    ApplicationContextScope second_scope(second);
    EXPECT_FALSE(InspectorRegistry::GetInstance().HasInspector<TestAsset>());
    auto second_lifetime = std::make_shared<int>(2);
    released = second_lifetime;
    ASSERT_TRUE(
        InspectorRegistry::GetInstance().RegisterInspector<TestAsset>([second_lifetime](InspectorContext&, TestAsset&) {
          return *second_lifetime == 2;
        }));
  }
  ApplicationContextScope restored_scope(first);
  EXPECT_TRUE(released.expired());
  EXPECT_TRUE(InspectorRegistry::GetInstance().HasInspector<TestAsset>());
}

TEST(InspectorRegistry, InspectsPlainRuntimeValuesWithoutGuiMembers) {
  InspectorRegistry registry;
  struct Settings {
    int value = 0;
  } settings;
  InspectorContext context;
  EXPECT_FALSE(registry.InspectValue(context, settings));
  ASSERT_TRUE(registry.RegisterInspector<Settings>([](InspectorContext&, Settings& target) {
    target.value = 17;
    return true;
  }));
  EXPECT_TRUE(registry.InspectValue(context, settings));
  EXPECT_EQ(settings.value, 17);
}

TEST(InspectorRegistry, ValueDispatchUsesMostDerivedAddressAndRetainsExecutingHandler) {
  struct Left {
    virtual ~Left() = default;
    int padding = 1;
  };
  struct Right {
    virtual ~Right() = default;
    int padding = 2;
  };
  struct Value : Left, Right {
    int value = 0;
  } value;
  InspectorRegistry registry;
  InspectorContext context;
  auto lifetime = std::make_shared<int>(19);
  std::weak_ptr<int> weak = lifetime;
  ASSERT_TRUE(registry.RegisterInspector<Value>(
      [lifetime, &registry, &weak](InspectorContext&, Value& target) {
        registry.UnregisterOwner("value-editor");
        EXPECT_FALSE(weak.expired());
        target.value = *lifetime;
        return true;
      },
      "value-editor"));
  lifetime.reset();
  EXPECT_TRUE(registry.InspectValue(context, static_cast<Right&>(value)));
  EXPECT_EQ(value.value, 19);
  EXPECT_TRUE(weak.expired());
  EXPECT_FALSE(registry.InspectValue(context, static_cast<Left&>(value)));
}
