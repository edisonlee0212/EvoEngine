#include "EvoEngine_SDK_PCH.hpp"

#include "Application.hpp"
#include "ApplicationContext.hpp"
#include "ApplicationInitializationSettings.hpp"
#include "AssetManager.hpp"
#include "MeshRenderer.hpp"
#include "RenderLayer.hpp"
#include "Scene.hpp"
#include "TransformGraph.hpp"
#include "gtest/gtest.h"

using namespace evo_engine;

namespace evo_engine {
class StaticSceneRenderTestAccess {
 public:
  static size_t PendingChangeCount(const RenderLayer& render_layer) {
    return render_layer.pending_static_entity_changes_.size();
  }

  static void Consume(RenderLayer& render_layer, const std::shared_ptr<Scene>& scene) {
    render_layer.ConsumeStaticEntityChanges(scene);
  }
};
}  // namespace evo_engine

namespace {
ApplicationInitializationSettings EmptyStaticSceneSettings() {
  ApplicationInitializationSettings settings;
  settings.allow_empty_project = true;
  settings.load_default_resources = false;
  settings.load_project_assets = false;
  settings.load_project_start_scene = false;
  settings.enable_runtime_packages = false;
  return settings;
}

struct StaticSceneTestContext {
  Application app;
  ApplicationContextScope scope{app};
  std::shared_ptr<Scene> scene;
  std::shared_ptr<RenderLayer> render_layer;

  StaticSceneTestContext() {
    app.Initialize(EmptyStaticSceneSettings());
    scene = AssetManager::CreateTemporaryAsset<Scene>();
    app.Attach(scene);
    render_layer = std::make_shared<RenderLayer>();
  }
};
}  // namespace

TEST(StaticSceneRender, StructureRevisionTracksAutomaticInvalidations) {
  StaticSceneTestContext context;
  auto revision = context.scene->GetRenderStructureRevision();
  const auto parent = context.scene->CreateEntity("Parent");
  EXPECT_GT(context.scene->GetRenderStructureRevision(), revision);

  revision = context.scene->GetRenderStructureRevision();
  context.scene->SetEntityStatic(parent, true);
  EXPECT_GT(context.scene->GetRenderStructureRevision(), revision);

  revision = context.scene->GetRenderStructureRevision();
  context.scene->GetOrSetPrivateComponent<MeshRenderer>(parent);
  EXPECT_GT(context.scene->GetRenderStructureRevision(), revision);

  revision = context.scene->GetRenderStructureRevision();
  context.scene->SetEnable(parent, false);
  EXPECT_GT(context.scene->GetRenderStructureRevision(), revision);
}

TEST(StaticSceneRender, NotificationDeduplicatesAndRecalculatesStaticSubtree) {
  StaticSceneTestContext context;
  ASSERT_TRUE(context.render_layer);
  const auto parent = context.scene->CreateEntity("Parent");
  const auto child = context.scene->CreateEntity("Child");
  context.scene->SetParent(child, parent);
  context.scene->SetEntityStatic(parent, true);
  TransformGraph::CalculateTransformGraphs(context.scene, false);

  GlobalTransform overridden;
  overridden.SetPosition(glm::vec3(3.0f, 4.0f, 5.0f));
  context.scene->SetDataComponent(child, overridden);
  context.render_layer->NotifyStaticEntityChanged(context.scene, child);
  context.render_layer->NotifyStaticEntityChanged(context.scene, child);
  EXPECT_EQ(StaticSceneRenderTestAccess::PendingChangeCount(*context.render_layer), 1u);

  StaticSceneRenderTestAccess::Consume(*context.render_layer, context.scene);
  EXPECT_EQ(StaticSceneRenderTestAccess::PendingChangeCount(*context.render_layer), 0u);
  EXPECT_EQ(context.scene->GetDataComponent<GlobalTransform>(child).GetPosition(), glm::vec3(3.0f, 4.0f, 5.0f));
  EXPECT_EQ(context.scene->GetDataComponent<Transform>(child).GetPosition(), glm::vec3(3.0f, 4.0f, 5.0f));

  const auto dynamic = context.scene->CreateEntity("Dynamic");
  context.render_layer->NotifyStaticEntityChanged(context.scene, dynamic);
  EXPECT_EQ(StaticSceneRenderTestAccess::PendingChangeCount(*context.render_layer), 0u);
}

TEST(StaticSceneRender, ParentNotificationCoversChildNotification) {
  StaticSceneTestContext context;
  ASSERT_TRUE(context.render_layer);
  const auto parent = context.scene->CreateEntity("Parent");
  const auto child = context.scene->CreateEntity("Child");
  context.scene->SetParent(child, parent);
  context.scene->SetEntityStatic(parent, true);

  Transform parent_transform;
  parent_transform.SetPosition(glm::vec3(2.0f, 0.0f, 0.0f));
  context.scene->SetDataComponent(parent, parent_transform);
  Transform child_transform;
  child_transform.SetPosition(glm::vec3(1.0f, 0.0f, 0.0f));
  context.scene->SetDataComponent(child, child_transform);
  context.render_layer->NotifyStaticEntityChanged(context.scene, child);
  context.render_layer->NotifyStaticEntityChanged(context.scene, parent);

  StaticSceneRenderTestAccess::Consume(*context.render_layer, context.scene);
  EXPECT_EQ(context.scene->GetDataComponent<GlobalTransform>(parent).GetPosition(), glm::vec3(2.0f, 0.0f, 0.0f));
  EXPECT_EQ(context.scene->GetDataComponent<GlobalTransform>(child).GetPosition(), glm::vec3(3.0f, 0.0f, 0.0f));
}
