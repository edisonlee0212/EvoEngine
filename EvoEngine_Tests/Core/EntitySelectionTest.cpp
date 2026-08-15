#include "EvoEngine_SDK_PCH.hpp"

#include "Application.hpp"
#include "ApplicationContext.hpp"
#include "ApplicationInitializationSettings.hpp"
#include "AssetManager.hpp"
#include "EntitySelection.hpp"
#include "EntitySelectionHighlight.hpp"
#include "RenderInstanceStorage.hpp"
#include "Scene.hpp"
#include "gtest/gtest.h"

using namespace evo_engine;

namespace {

ApplicationInitializationSettings EmptyProjectSettings() {
  ApplicationInitializationSettings settings;
  settings.allow_empty_project = true;
  settings.load_default_resources = false;
  settings.load_project_assets = false;
  settings.load_project_start_scene = false;
  settings.enable_runtime_packages = false;
  return settings;
}

struct SelectionTestContext {
  Application app;
  ApplicationContextScope scope{app};
  std::shared_ptr<Scene> scene;

  SelectionTestContext() {
    app.Initialize(EmptyProjectSettings());
    scene = AssetManager::CreateTemporaryAsset<Scene>();
    app.Attach(scene);
  }
};

EntitySelection::RequestOptions UserAnchor(const Entity entity) {
  return {EntitySelection::RequestSource::User, EntitySelection::AnchorPolicy::Set, entity};
}

EntitySelection::RequestOptions UserClearAnchor() {
  return {EntitySelection::RequestSource::User, EntitySelection::AnchorPolicy::Clear, {}};
}

}  // namespace

TEST(EntitySelection, ReplaceAddToggleAndPrimaryPromotionAreDeterministic) {
  SelectionTestContext context;
  const auto scene = context.scene;
  const auto first = scene->CreateEntity("First");
  const auto second = scene->CreateEntity("Second");
  const auto third = scene->CreateEntity("Third");
  EntitySelection selection;

  EXPECT_EQ(selection.BindScene(scene), EntitySelection::Result::Changed);
  EXPECT_EQ(selection.Replace(first, UserAnchor(first)), EntitySelection::Result::Changed);
  EXPECT_EQ(selection.Add(second, UserAnchor(second)), EntitySelection::Result::Changed);
  EXPECT_EQ(selection.Add(third, UserAnchor(third)), EntitySelection::Result::Changed);
  EXPECT_EQ(selection.GetEntities(), (std::vector<Entity>{first, second, third}));
  EXPECT_EQ(selection.GetPrimary(), third);
  EXPECT_EQ(selection.GetAnchor(), third);

  EXPECT_EQ(selection.Toggle(third, UserAnchor(third)), EntitySelection::Result::Changed);
  EXPECT_EQ(selection.GetEntities(), (std::vector<Entity>{first, second}));
  EXPECT_EQ(selection.GetPrimary(), second);
  EXPECT_EQ(selection.GetAnchor(), second);
  EXPECT_EQ(selection.Toggle(first, UserAnchor(first)), EntitySelection::Result::Changed);
  EXPECT_EQ(selection.GetEntities(), (std::vector<Entity>{second}));
  EXPECT_EQ(selection.GetPrimary(), second);
  EXPECT_EQ(selection.GetAnchor(), second);
}

TEST(EntitySelection, BulkRequestsAreAtomicAndUseClickedTargetPrimaryRules) {
  SelectionTestContext context;
  const auto scene = context.scene;
  const auto first = scene->CreateEntity("First");
  const auto second = scene->CreateEntity("Second");
  const auto third = scene->CreateEntity("Third");
  EntitySelection selection;
  selection.BindScene(scene);

  const auto before_replace = selection.GetRevision();
  EXPECT_EQ(selection.ReplaceMany({first, second, third}, third, UserAnchor(first)), EntitySelection::Result::Changed);
  EXPECT_EQ(selection.GetRevision(), before_replace + 1);
  EXPECT_EQ(selection.GetPrimary(), third);
  EXPECT_EQ(selection.GetAnchor(), first);

  const auto unchanged_revision = selection.GetRevision();
  EXPECT_EQ(selection.AddMany({first, second, third}, second, UserAnchor(first)), EntitySelection::Result::Unchanged);
  EXPECT_EQ(selection.GetRevision(), unchanged_revision);
  EXPECT_EQ(selection.GetPrimary(), third);

  const auto fourth = scene->CreateEntity("Fourth");
  EXPECT_EQ(selection.AddMany({second, fourth}, second, UserAnchor(first)), EntitySelection::Result::Changed);
  EXPECT_EQ(selection.GetEntities().back(), second);
  EXPECT_EQ(selection.GetPrimary(), second);
  EXPECT_TRUE(selection.Contains(fourth));
}

TEST(EntitySelection, LockRejectsUserRequestsButClearAndProgrammaticRequestsBypassIt) {
  SelectionTestContext context;
  const auto scene = context.scene;
  const auto first = scene->CreateEntity("First");
  const auto second = scene->CreateEntity("Second");
  EntitySelection selection;
  selection.BindScene(scene);
  selection.Replace(first);
  EXPECT_EQ(selection.SetLocked(true), EntitySelection::Result::Changed);

  EXPECT_EQ(selection.Replace(second), EntitySelection::Result::RejectedLocked);
  EXPECT_EQ(selection.GetPrimary(), first);
  EXPECT_EQ(selection.Replace(second,
                              {EntitySelection::RequestSource::Programmatic, EntitySelection::AnchorPolicy::Clear, {}}),
            EntitySelection::Result::Changed);
  EXPECT_TRUE(selection.IsLocked());
  EXPECT_EQ(selection.Clear(), EntitySelection::Result::Changed);
  EXPECT_TRUE(selection.Empty());
  EXPECT_FALSE(selection.IsLocked());
}

TEST(EntitySelection, AnchorOnlyChangesHaveOneRevision) {
  SelectionTestContext context;
  const auto scene = context.scene;
  const auto entity = scene->CreateEntity("Entity");
  EntitySelection selection;
  selection.BindScene(scene);
  selection.Replace(entity, UserAnchor(entity));
  const auto revision = selection.GetRevision();

  EXPECT_EQ(selection.Add(entity, UserClearAnchor()), EntitySelection::Result::Changed);
  EXPECT_EQ(selection.GetRevision(), revision + 1);
  EXPECT_EQ(selection.GetPrimary(), entity);
  EXPECT_TRUE(selection.GetAnchor().GetIndex() == 0);
  EXPECT_EQ(selection.Add(entity, UserClearAnchor()), EntitySelection::Result::Unchanged);
  EXPECT_EQ(selection.GetRevision(), revision + 1);
}

TEST(EntitySelection, InvalidRequestsAreAtomicAndPruningRepairsPrimaryAndAnchor) {
  SelectionTestContext context;
  const auto scene = context.scene;
  const auto first = scene->CreateEntity("First");
  const auto second = scene->CreateEntity("Second");
  const auto invalid = MakeSceneEntity(100000, 7);
  EntitySelection selection;
  selection.BindScene(scene);
  selection.ReplaceMany({first, second}, second, UserAnchor(second));
  const auto revision = selection.GetRevision();

  EXPECT_EQ(selection.AddMany({first, invalid}, invalid), EntitySelection::Result::RejectedInvalid);
  EXPECT_EQ(selection.GetRevision(), revision);
  EXPECT_EQ(selection.GetEntities(), (std::vector<Entity>{first, second}));

  scene->DeleteEntity(second);
  EXPECT_EQ(selection.PruneInvalid(), EntitySelection::Result::Changed);
  EXPECT_EQ(selection.GetEntities(), (std::vector<Entity>{first}));
  EXPECT_EQ(selection.GetPrimary(), first);
  EXPECT_EQ(selection.GetAnchor(), first);
}

TEST(EntitySelection, BindingAnotherSceneClearsMembershipAndLock) {
  SelectionTestContext context;
  const auto first_scene = context.scene;
  const auto second_scene = AssetManager::CreateTemporaryAsset<Scene>();
  const auto entity = first_scene->CreateEntity("Entity");
  EntitySelection selection;
  selection.BindScene(first_scene);
  selection.Replace(entity, UserAnchor(entity));
  selection.SetLocked(true);
  const auto revision = selection.GetRevision();

  EXPECT_EQ(selection.BindScene(second_scene), EntitySelection::Result::Changed);
  EXPECT_EQ(selection.GetRevision(), revision + 1);
  EXPECT_TRUE(selection.Empty());
  EXPECT_FALSE(selection.IsLocked());
  EXPECT_TRUE(selection.GetPrimary().GetIndex() == 0);
  EXPECT_TRUE(selection.GetAnchor().GetIndex() == 0);
}

TEST(EntitySelection, SnapshotIsAnImmutableValueCopy) {
  SelectionTestContext context;
  const auto scene = context.scene;
  const auto first = scene->CreateEntity("First");
  const auto second = scene->CreateEntity("Second");
  EntitySelection selection;
  selection.BindScene(scene);
  selection.Replace(first);
  const auto snapshot = selection.GetSnapshot();

  selection.Add(second);
  EXPECT_EQ(snapshot.entities, (std::vector<Entity>{first}));
  EXPECT_EQ(snapshot.primary, first);
  EXPECT_TRUE(snapshot.Contains(first));
  EXPECT_FALSE(snapshot.Contains(second));
}

TEST(EntitySelection, ClearAnchorInvalidatesRangeStateWithoutChangingMembership) {
  SelectionTestContext context;
  const auto entity = context.scene->CreateEntity("Entity");
  EntitySelection selection;
  selection.BindScene(context.scene);
  selection.Replace(entity, UserAnchor(entity));
  const auto revision = selection.GetRevision();

  EXPECT_EQ(selection.ClearAnchor(EntitySelection::RequestSource::Lifecycle), EntitySelection::Result::Changed);
  EXPECT_EQ(selection.GetRevision(), revision + 1);
  EXPECT_EQ(selection.GetEntities(), (std::vector<Entity>{entity}));
  EXPECT_EQ(selection.GetPrimary(), entity);
  EXPECT_EQ(selection.GetAnchor(), Entity{});
  EXPECT_EQ(selection.ClearAnchor(EntitySelection::RequestSource::Lifecycle), EntitySelection::Result::Unchanged);
}

TEST(EntitySelection, HighlightFadeOnlyRestartsForActivationAndEnableTransitions) {
  EntitySelectionHighlight highlight;

  highlight.Update(true, 0.1f);
  EXPECT_FLOAT_EQ(highlight.GetSnapshot().fade_progress, 0.5f);
  EXPECT_TRUE(highlight.GetSnapshot().active);
  highlight.Update(true, 0.1f);
  EXPECT_FLOAT_EQ(highlight.GetSnapshot().fade_progress, 1.0f);

  highlight.SetEnabled(false, true);
  EXPECT_FALSE(highlight.GetSnapshot().active);
  highlight.SetEnabled(true, true);
  EXPECT_FLOAT_EQ(highlight.GetSnapshot().fade_progress, 0.0f);
  highlight.Update(true, 0.1f);
  EXPECT_FLOAT_EQ(highlight.GetSnapshot().fade_progress, 0.5f);

  highlight.Update(false, 1.0f);
  EXPECT_FLOAT_EQ(highlight.GetSnapshot().fade_progress, 0.0f);
  EXPECT_FALSE(highlight.GetSnapshot().active);
}

TEST(EntitySelection, HighlightBitDoesNotClassifyRenderInstancesAsSceneChanges) {
  RenderInstanceStorage::InstanceInfoBlock unselected;
  auto selected = unselected;
  selected.info_index = 1;

  EXPECT_FALSE(unselected != selected);
  EXPECT_FALSE(selected != unselected);
}
