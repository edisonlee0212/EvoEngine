#include "EvoEngine_SDK_PCH.hpp"

#include "Application.hpp"
#include "ApplicationContext.hpp"
#include "ApplicationInitializationSettings.hpp"
#include "AssetManager.hpp"
#include "Camera.hpp"
#include "EntityBatchInspector.hpp"
#include "MeshRenderer.hpp"
#include "Particles.hpp"
#include "Prefab.hpp"
#include "Scene.hpp"
#include "TransformGraph.hpp"
#include "gtest/gtest.h"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iterator>

using namespace evo_engine;

namespace {
ApplicationInitializationSettings EmptyBatchProjectSettings() {
  ApplicationInitializationSettings settings;
  settings.allow_empty_project = true;
  settings.load_default_resources = false;
  settings.load_project_assets = false;
  settings.load_project_start_scene = false;
  settings.enable_runtime_packages = false;
  return settings;
}

std::string ReadBatchInspectorSource(const std::filesystem::path& relative_path) {
  std::ifstream file(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / relative_path);
  EXPECT_TRUE(file.good()) << relative_path.string();
  return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}

struct BatchInspectorTestContext {
  Application app;
  ApplicationContextScope scope{app};
  std::shared_ptr<Scene> scene;

  BatchInspectorTestContext() {
    app.Initialize(EmptyBatchProjectSettings());
    scene = AssetManager::CreateTemporaryAsset<Scene>();
    app.Attach(scene);
  }
};
}  // namespace

TEST(EntityBatchInspector, IntersectsComponentsAndPatchesOnlyEditedTransformAxis) {
  BatchInspectorTestContext context;
  const auto first = context.scene->CreateEntity("First");
  const auto second = context.scene->CreateEntity("Second");
  context.scene->AddDataComponent(first, Ray{});
  ASSERT_TRUE(context.scene->GetOrSetPrivateComponent<Camera>(first).lock());
  ASSERT_TRUE(context.scene->GetOrSetPrivateComponent<Camera>(second).lock());
  ASSERT_TRUE(context.scene->GetOrSetPrivateComponent<MeshRenderer>(first).lock());

  Transform first_transform;
  first_transform.SetValue({1.0f, 2.0f, 3.0f}, glm::vec3(0.0f), {1.0f, 2.0f, 3.0f});
  context.scene->SetDataComponent(first, first_transform);
  Transform second_transform;
  second_transform.SetValue({4.0f, 5.0f, 6.0f}, glm::vec3(0.0f), {4.0f, 5.0f, 6.0f});
  context.scene->SetDataComponent(second, second_transform);

  const auto batch = EntityBatchInspector::BuildContext(context.scene, {first, second}, second);
  ASSERT_EQ(batch.targets, (std::vector<Entity>{first, second}));
  EXPECT_EQ(batch.primary, second);
  ASSERT_EQ(batch.common_data_components.size(), 1u);
  EXPECT_NE(std::find_if(batch.common_data_components.begin(), batch.common_data_components.end(),
                         [](const auto& type) {
                           return type.type_index == typeid(Transform).hash_code();
                         }),
            batch.common_data_components.end());
  EXPECT_EQ(batch.hidden_data_component_types, 1u);
  ASSERT_EQ(batch.common_private_components.size(), 1u);
  EXPECT_EQ(batch.common_private_components.front().type_index, typeid(Camera).hash_code());
  EXPECT_EQ(batch.hidden_private_component_types, 1u);

  const auto positions = EntityBatchInspector::ReadLocalPosition(context.scene, batch.targets);
  EXPECT_TRUE(positions.mixed);
  EXPECT_TRUE(positions.mixed_axes[0]);
  EXPECT_TRUE(
      EntityBatchInspector::WriteLocalTransformField(context.scene, batch.targets, 0, {10.0f, 20.0f, 30.0f}, 0));
  EXPECT_EQ(context.scene->GetDataComponent<Transform>(first).GetPosition(), glm::vec3(10.0f, 2.0f, 3.0f));
  EXPECT_EQ(context.scene->GetDataComponent<Transform>(second).GetPosition(), glm::vec3(10.0f, 5.0f, 6.0f));
  EXPECT_TRUE(EntityBatchInspector::WriteLocalTransformField(context.scene, batch.targets, 2, {1.0f, 1.0f, 1.0f}, 1));
  EXPECT_EQ(context.scene->GetDataComponent<Transform>(first).GetScale(), glm::vec3(1.0f, 1.0f, 3.0f));
  EXPECT_EQ(context.scene->GetDataComponent<Transform>(second).GetScale(), glm::vec3(4.0f, 1.0f, 6.0f));
}

TEST(EntityBatchInspector, AppliesRelativeTransformDragFromCapturedValues) {
  BatchInspectorTestContext context;
  const auto first = context.scene->CreateEntity("First");
  const auto second = context.scene->CreateEntity("Second");
  Transform first_transform;
  first_transform.SetValue({1.0f, 2.0f, 3.0f}, glm::radians(glm::vec3(10.0f, 0.0f, 0.0f)), {2.0f, 1.0f, 1.0f});
  Transform second_transform;
  second_transform.SetValue({4.0f, 5.0f, 6.0f}, glm::radians(glm::vec3(30.0f, 0.0f, 0.0f)), {4.0f, 1.0f, 1.0f});
  context.scene->SetDataComponent(first, first_transform);
  context.scene->SetDataComponent(second, second_transform);
  const std::vector<Entity> targets{first, second};
  const std::vector<Transform> originals{first_transform, second_transform};

  ASSERT_TRUE(
      EntityBatchInspector::WriteRelativeLocalTransformField(context.scene, targets, originals, 0, 0, 1.0f, 3.5f));
  EXPECT_EQ(context.scene->GetDataComponent<Transform>(first).GetPosition(), glm::vec3(3.5f, 2.0f, 3.0f));
  EXPECT_EQ(context.scene->GetDataComponent<Transform>(second).GetPosition(), glm::vec3(6.5f, 5.0f, 6.0f));

  ASSERT_TRUE(
      EntityBatchInspector::WriteRelativeLocalTransformField(context.scene, targets, originals, 1, 0, 10.0f, 25.0f));
  EXPECT_NEAR(glm::degrees(context.scene->GetDataComponent<Transform>(first).GetEulerRotation()).x, 25.0f, 1.0e-4f);
  EXPECT_NEAR(glm::degrees(context.scene->GetDataComponent<Transform>(second).GetEulerRotation()).x, 45.0f, 1.0e-4f);

  ASSERT_TRUE(
      EntityBatchInspector::WriteRelativeLocalTransformField(context.scene, targets, originals, 2, 0, 2.0f, 3.0f));
  const auto first_scale = context.scene->GetDataComponent<Transform>(first).GetScale();
  const auto second_scale = context.scene->GetDataComponent<Transform>(second).GetScale();
  EXPECT_NEAR(glm::distance(first_scale, glm::vec3(3.0f, 1.0f, 1.0f)), 0.0f, 1.0e-5f);
  EXPECT_NEAR(glm::distance(second_scale, glm::vec3(6.0f, 1.0f, 1.0f)), 0.0f, 1.0e-5f);
}

TEST(EntityBatchInspector, UsesAdditiveScaleDragWhenRepresentativeStartsAtZero) {
  BatchInspectorTestContext context;
  const auto first = context.scene->CreateEntity("First");
  const auto second = context.scene->CreateEntity("Second");
  Transform first_transform;
  first_transform.SetScale({1.0e-7f, 1.0f, 1.0f});
  Transform second_transform;
  second_transform.SetScale({2.0f, 1.0f, 1.0f});
  context.scene->SetDataComponent(first, first_transform);
  context.scene->SetDataComponent(second, second_transform);

  ASSERT_TRUE(EntityBatchInspector::WriteRelativeLocalTransformField(
      context.scene, {first, second}, {first_transform, second_transform}, 2, 0, 1.0e-7f, 0.5f));
  EXPECT_NEAR(context.scene->GetDataComponent<Transform>(first).GetScale().x, 0.5000001f, 1.0e-5f);
  EXPECT_EQ(context.scene->GetDataComponent<Transform>(second).GetScale(), glm::vec3(2.5f, 1.0f, 1.0f));
}

namespace {
struct ModelImportTestPrefab : Prefab {
  using Prefab::LoadModelInternal;
};

std::shared_ptr<Prefab> FindStaticMeshRendererPrefab(const std::shared_ptr<Prefab>& prefab) {
  if (prefab->GetPrivateComponent<MeshRenderer>())
    return prefab;
  for (const auto& child : prefab->child_prefabs) {
    if (const auto result = FindStaticMeshRendererPrefab(child))
      return result;
  }
  return {};
}
}  // namespace

TEST(PrefabModelImport, CentersStaticMeshRendererOriginsByDefaultAndCanPreserveLegacyOrigin) {
  BatchInspectorTestContext context;
  const auto path = std::filesystem::temp_directory_path() / "evoengine_offset_mesh_origin_test.obj";
  {
    std::ofstream file(path);
    file << "o Offset\n"
            "v 10 0 0\n"
            "v 12 0 0\n"
            "v 10 2 0\n"
            "f 1 2 3\n";
  }

  ModelImportTestPrefab centered;
  ASSERT_TRUE(centered.LoadModelInternal(path));
  const auto centered_child = FindStaticMeshRendererPrefab(std::shared_ptr<Prefab>(&centered, [](Prefab*) {
  }));
  ASSERT_TRUE(centered_child);
  const auto centered_renderer = centered_child->GetPrivateComponent<MeshRenderer>();
  ASSERT_TRUE(centered_renderer);
  const auto centered_mesh = centered_renderer->mesh.Get<Mesh>();
  ASSERT_TRUE(centered_mesh);
  ASSERT_FALSE(centered_child->data_components.empty());
  const auto centered_transform =
      std::static_pointer_cast<Transform>(centered_child->data_components.front().data_component);
  ASSERT_TRUE(centered_transform);
  EXPECT_EQ(centered_transform->GetPosition(), glm::vec3(11.0f, 1.0f, 0.0f));
  EXPECT_EQ((centered_mesh->GetBound().min + centered_mesh->GetBound().max) * 0.5f, glm::vec3(0.0f));

  ModelImportTestPrefab legacy;
  PrefabModelImportOptions options;
  options.center_mesh_renderer_origins = false;
  ASSERT_TRUE(legacy.LoadModelInternal(
      path, false, aiProcess_Triangulate | aiProcess_CalcTangentSpace | aiProcess_GenSmoothNormals, options));
  const auto legacy_child = FindStaticMeshRendererPrefab(std::shared_ptr<Prefab>(&legacy, [](Prefab*) {
  }));
  ASSERT_TRUE(legacy_child);
  const auto legacy_renderer = legacy_child->GetPrivateComponent<MeshRenderer>();
  ASSERT_TRUE(legacy_renderer);
  const auto legacy_mesh = legacy_renderer->mesh.Get<Mesh>();
  ASSERT_TRUE(legacy_mesh);
  const auto legacy_transform =
      std::static_pointer_cast<Transform>(legacy_child->data_components.front().data_component);
  ASSERT_TRUE(legacy_transform);
  EXPECT_EQ(legacy_transform->GetPosition(), glm::vec3(0.0f));
  EXPECT_EQ((legacy_mesh->GetBound().min + legacy_mesh->GetBound().max) * 0.5f, glm::vec3(11.0f, 1.0f, 0.0f));
  ASSERT_EQ(centered_mesh->PeekVertices().size(), legacy_mesh->PeekVertices().size());
  for (size_t i = 0; i < centered_mesh->PeekVertices().size(); ++i) {
    EXPECT_EQ(centered_mesh->PeekVertices()[i].position + centered_transform->GetPosition(),
              legacy_mesh->PeekVertices()[i].position);
  }
  std::filesystem::remove(path);
}

TEST(EntityBatchInspector, ResolvesSingleAndBatchInspectorDispatchWithoutRepeatingSingleInspectors) {
  EXPECT_EQ(EntityBatchInspector::ResolveInspectorDispatch(0, true, true), EntityInspectorDispatch::Unsupported);
  EXPECT_EQ(EntityBatchInspector::ResolveInspectorDispatch(1, true, false), EntityInspectorDispatch::Single);
  EXPECT_EQ(EntityBatchInspector::ResolveInspectorDispatch(1, false, true), EntityInspectorDispatch::Batch);
  EXPECT_EQ(EntityBatchInspector::ResolveInspectorDispatch(1, true, true), EntityInspectorDispatch::Single);
  EXPECT_EQ(EntityBatchInspector::ResolveInspectorDispatch(1, true, true, true), EntityInspectorDispatch::Batch);
  EXPECT_EQ(EntityBatchInspector::ResolveInspectorDispatch(2, true, false), EntityInspectorDispatch::Unsupported);
  EXPECT_EQ(EntityBatchInspector::ResolveInspectorDispatch(2, true, true), EntityInspectorDispatch::Batch);
}

TEST(EntityBatchInspector, SingleTargetContextHidesInternalTransformComponents) {
  BatchInspectorTestContext context;
  const auto entity = context.scene->CreateEntity("Entity");
  const auto batch = EntityBatchInspector::BuildContext(context.scene, {entity}, entity);

  ASSERT_EQ(batch.common_data_components.size(), 1u);
  EXPECT_EQ(batch.common_data_components.front().type_index, typeid(Transform).hash_code());
  EXPECT_EQ(batch.hidden_data_component_types, 0u);
}

TEST(EntityBatchInspector, GizmoHandleUsesSharedSelectionBoundForBothPivotModes) {
  const auto source = ReadBatchInspectorSource("EvoEngine_SDK/src/EditorLayer.cpp");
  EXPECT_NE(source.find("ResolveSelectionGizmoBound(scene, selection)"), std::string::npos);
  EXPECT_NE(source.find("entity_gizmo_session_->handle"), std::string::npos);
  EXPECT_EQ(source.find("std::vector<Entity>{reference}"), std::string::npos);
}

TEST(EntityBatchInspector, GizmoParticipantsExcludeSelectedDescendantsAndResolveReferenceAncestor) {
  BatchInspectorTestContext context;
  const auto parent = context.scene->CreateEntity("Parent");
  const auto child = context.scene->CreateEntity("Child");
  const auto other = context.scene->CreateEntity("Other");
  context.scene->SetParent(child, parent);

  const auto participants = EntityBatchInspector::BuildGizmoParticipants(context.scene, {parent, child, other});
  EXPECT_EQ(participants, (std::vector<Entity>{parent, other}));
  EXPECT_EQ(EntityBatchInspector::FindGizmoReference(context.scene, participants, child), parent);
  EXPECT_EQ(EntityBatchInspector::FindGizmoReference(context.scene, participants, other), other);
}

TEST(EntityBatchInspector, SelectionWorldBoundUsesVisibleDescendantRenderersAndFallsBackToParticipantOrigins) {
  BatchInspectorTestContext context;
  const auto parent = context.scene->CreateEntity("Parent");
  const auto child = context.scene->CreateEntity("Child");
  const auto meshless = context.scene->CreateEntity("Meshless");
  context.scene->SetParent(child, parent);

  Transform parent_transform;
  parent_transform.SetPosition({10.0f, 0.0f, 0.0f});
  context.scene->SetDataComponent(parent, parent_transform);
  Transform child_transform;
  child_transform.SetValue({2.0f, 0.0f, 0.0f}, glm::vec3(0.0f), {2.0f, 1.0f, 1.0f});
  context.scene->SetDataComponent(child, child_transform);
  Transform meshless_transform;
  meshless_transform.SetPosition({100.0f, 0.0f, 0.0f});
  context.scene->SetDataComponent(meshless, meshless_transform);
  TransformGraph::CalculateTransformGraphs(context.scene, false);

  VertexAttributes attributes;
  std::vector<Vertex> vertices(3);
  vertices[0].position = {1.0f, -1.0f, -1.0f};
  vertices[1].position = {3.0f, 1.0f, -1.0f};
  vertices[2].position = {1.0f, -1.0f, 1.0f};
  const auto mesh = AssetManager::CreateTemporaryAsset<Mesh>();
  mesh->SetVertices(attributes, vertices, {glm::uvec3(0, 1, 2)});
  const auto mesh_renderer = context.scene->GetOrSetPrivateComponent<MeshRenderer>(child).lock();
  ASSERT_TRUE(mesh_renderer);
  mesh_renderer->mesh = mesh;

  const auto participants = EntityBatchInspector::BuildGizmoParticipants(context.scene, {parent, child, meshless});
  ASSERT_EQ(participants, (std::vector<Entity>{parent, meshless}));
  auto bound = EntityBatchInspector::BuildSelectionWorldBound(context.scene, participants);
  ASSERT_TRUE(bound.valid);
  EXPECT_TRUE(bound.has_renderable_bounds);
  EXPECT_EQ(bound.world_bound.min, glm::vec3(14.0f, -1.0f, -1.0f));
  EXPECT_EQ(bound.world_bound.max, glm::vec3(18.0f, 1.0f, 1.0f));

  const auto particles = context.scene->GetOrSetPrivateComponent<Particles>(meshless).lock();
  ASSERT_TRUE(particles);
  particles->bounding_box.min = {-2.0f, -3.0f, -4.0f};
  particles->bounding_box.max = {2.0f, 3.0f, 4.0f};
  bound = EntityBatchInspector::BuildSelectionWorldBound(context.scene, participants);
  ASSERT_TRUE(bound.valid);
  EXPECT_TRUE(bound.has_renderable_bounds);
  EXPECT_EQ(bound.world_bound.min, glm::vec3(14.0f, -3.0f, -4.0f));
  EXPECT_EQ(bound.world_bound.max, glm::vec3(102.0f, 3.0f, 4.0f));

  mesh_renderer->SetEnabled(false);
  particles->SetEnabled(false);
  bound = EntityBatchInspector::BuildSelectionWorldBound(context.scene, participants);
  ASSERT_TRUE(bound.valid);
  EXPECT_FALSE(bound.has_renderable_bounds);
  EXPECT_EQ(bound.world_bound.min, glm::vec3(10.0f, 0.0f, 0.0f));
  EXPECT_EQ(bound.world_bound.max, glm::vec3(100.0f, 0.0f, 0.0f));
}

TEST(EntityBatchInspector, GizmoMathUsesTheImmutableHandleAndHonorsPivotModes) {
  const auto initial_handle = glm::translate(glm::vec3(1.0f, 0.0f, 0.0f));
  const auto participant = glm::translate(glm::vec3(3.0f, 0.0f, 0.0f));
  glm::mat4 candidate(1.0f);

  ASSERT_TRUE(EntityBatchInspector::TryApplyGizmoTransform(
      initial_handle, glm::translate(glm::vec3(2.0f, 2.0f, 0.0f)), participant, EntityBatchGizmoOperation::Translate,
      EntityBatchGizmoPivot::Pivot, EntityBatchGizmoOrientation::Local, candidate));
  EXPECT_NEAR(candidate[3].x, 4.0f, 0.0001f);
  EXPECT_NEAR(candidate[3].y, 2.0f, 0.0001f);

  const auto rotated_handle = glm::translate(glm::vec3(1.0f, 0.0f, 0.0f)) *
                              glm::mat4_cast(glm::angleAxis(glm::radians(90.0f), glm::vec3(0.0f, 0.0f, 1.0f)));
  ASSERT_TRUE(EntityBatchInspector::TryApplyGizmoTransform(
      initial_handle, rotated_handle, participant, EntityBatchGizmoOperation::Rotate, EntityBatchGizmoPivot::Center,
      EntityBatchGizmoOrientation::Global, candidate));
  EXPECT_NEAR(candidate[3].x, 1.0f, 0.0001f);
  EXPECT_NEAR(candidate[3].y, 2.0f, 0.0001f);

  ASSERT_TRUE(EntityBatchInspector::TryApplyGizmoTransform(
      initial_handle, rotated_handle, participant, EntityBatchGizmoOperation::Rotate, EntityBatchGizmoPivot::Pivot,
      EntityBatchGizmoOrientation::Global, candidate));
  EXPECT_NEAR(candidate[3].x, 3.0f, 0.0001f);
  EXPECT_NEAR(candidate[3].y, 0.0f, 0.0001f);

  ASSERT_TRUE(EntityBatchInspector::TryApplyGizmoTransform(
      initial_handle, glm::scale(initial_handle, glm::vec3(2.0f)), participant, EntityBatchGizmoOperation::Scale,
      EntityBatchGizmoPivot::Pivot, EntityBatchGizmoOrientation::Local, candidate));
  EXPECT_NEAR(candidate[3].x, 3.0f, 0.0001f);
  EXPECT_NEAR(candidate[0][0], 2.0f, 0.0001f);

  ASSERT_TRUE(EntityBatchInspector::TryApplyGizmoTransform(
      initial_handle, glm::scale(initial_handle, glm::vec3(2.0f)), participant, EntityBatchGizmoOperation::Scale,
      EntityBatchGizmoPivot::Center, EntityBatchGizmoOrientation::Local, candidate));
  EXPECT_NEAR(candidate[3].x, 5.0f, 0.0001f);
  EXPECT_NEAR(candidate[0][0], 2.0f, 0.0001f);
}

TEST(EntityBatchInspector, GizmoMathTracksProgressiveAbsoluteRotationAndScaleHandles) {
  const glm::mat4 initial_handle(1.0f);
  const glm::mat4 participant(1.0f);
  glm::mat4 candidate(1.0f);

  const auto rotate_30 = glm::mat4_cast(glm::angleAxis(glm::radians(30.0f), glm::vec3(0.0f, 0.0f, 1.0f)));
  ASSERT_TRUE(EntityBatchInspector::TryApplyGizmoTransform(
      initial_handle, rotate_30, participant, EntityBatchGizmoOperation::Rotate, EntityBatchGizmoPivot::Pivot,
      EntityBatchGizmoOrientation::Global, candidate));
  EXPECT_NEAR(candidate[0][0], std::cos(glm::radians(30.0f)), 0.0001f);

  const auto rotate_60 = glm::mat4_cast(glm::angleAxis(glm::radians(60.0f), glm::vec3(0.0f, 0.0f, 1.0f)));
  ASSERT_TRUE(EntityBatchInspector::TryApplyGizmoTransform(
      initial_handle, rotate_60, participant, EntityBatchGizmoOperation::Rotate, EntityBatchGizmoPivot::Pivot,
      EntityBatchGizmoOrientation::Global, candidate));
  EXPECT_NEAR(candidate[0][0], std::cos(glm::radians(60.0f)), 0.0001f);

  ASSERT_TRUE(EntityBatchInspector::TryApplyGizmoTransform(
      initial_handle, glm::scale(glm::mat4(1.0f), glm::vec3(1.25f)), participant, EntityBatchGizmoOperation::Scale,
      EntityBatchGizmoPivot::Pivot, EntityBatchGizmoOrientation::Local, candidate));
  EXPECT_NEAR(candidate[0][0], 1.25f, 0.0001f);
  ASSERT_TRUE(EntityBatchInspector::TryApplyGizmoTransform(
      initial_handle, glm::scale(glm::mat4(1.0f), glm::vec3(1.75f)), participant, EntityBatchGizmoOperation::Scale,
      EntityBatchGizmoPivot::Pivot, EntityBatchGizmoOrientation::Local, candidate));
  EXPECT_NEAR(candidate[0][0], 1.75f, 0.0001f);
}
