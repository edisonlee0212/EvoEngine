#include <gtest/gtest.h>
#include <cmath>
#include <glm/gtx/quaternion.hpp>
#include "Application.hpp"
#include "ApplicationContext.hpp"
#include "ApplicationInitializationSettings.hpp"
#include "AssetManager.hpp"
#include "DsColliders.hpp"
#include "DynamicTreeSkeleton.hpp"
#include "EcoSysLabSerializationAdapters.hpp"
#include "EvoEngine_SDK_PCH.hpp"
#include "GpuTreeMeshData.hpp"
#include "PrivateComponentRef.hpp"
#include "Scene.hpp"
#include "Serialization.hpp"
#include "Strands.hpp"
#include "Tree.hpp"
#include "TreePhysicsCollision.hpp"

using namespace eco_sys_lab_package;

TEST(Blower, SettingsRoundTrip) {
  Blower source;
  source.speed = 12.0f;
  source.particle_rate = 2400.0f;
  source.particle_radius = 0.35f;
  source.air_density = 2.45f;
  source.spread_degrees = 15.0f;
  source.seed = 42;
  YAML::Emitter out;
  out << YAML::BeginMap;
  SerializeBlower(out, source);
  out << YAML::EndMap;

  Blower loaded;
  DeserializeBlower(YAML::Load(out.c_str()), loaded);
  EXPECT_FLOAT_EQ(loaded.speed, source.speed);
  EXPECT_FLOAT_EQ(loaded.particle_rate, source.particle_rate);
  EXPECT_FLOAT_EQ(loaded.particle_radius, source.particle_radius);
  EXPECT_FLOAT_EQ(loaded.air_density, source.air_density);
  EXPECT_FLOAT_EQ(loaded.spread_degrees, source.spread_degrees);
  EXPECT_EQ(loaded.seed, source.seed);
}

namespace {
BaseSkeleton MakeFork() {
  BaseSkeleton skeleton;
  auto& trunk = skeleton.RefNode(0).info;
  trunk.length = 1.0f;
  trunk.thickness = 0.1f;
  trunk.global_rotation = glm::rotation(glm::vec3(0, 0, -1), glm::vec3(0, 1, 0));
  const auto left = skeleton.Extend(0, false);
  const auto right = skeleton.Extend(0, false);
  for (const auto handle : {left, right}) {
    auto& info = skeleton.RefNode(handle).info;
    info.global_position = {0, 1, 0};
    info.length = 0.8f;
    info.thickness = 0.05f;
    info.global_rotation =
        glm::rotation(glm::vec3(0, 0, -1), glm::normalize(glm::vec3(handle == left ? -1.0f : 1.0f, 1, 0)));
  }
  skeleton.SortLists();
  return skeleton;
}

bool FinitePose(const ShootModel::PhysicsPose& pose) {
  return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) && std::isfinite(pose.position.z) &&
         std::isfinite(pose.length);
}

struct EcoSysLabResourceDirectory {
  std::filesystem::path previous = std::filesystem::current_path();
  EcoSysLabResourceDirectory() {
    std::filesystem::current_path(std::filesystem::path(__FILE__).parent_path().parent_path().parent_path() /
                                  "EvoEngine_Packages/EcoSysLab/Internals");
  }
  ~EcoSysLabResourceDirectory() {
    std::filesystem::current_path(previous);
  }
};
}  // namespace

TEST(TreePhysics, BoxContactProjectsRodRadiusOutsideGround) {
  const glm::quat rotation{1, 0, 0, 0};
  const glm::vec3 half_extent{5, 0.5f, 5};
  const glm::vec3 center{0, -0.5f, 0};
  const auto projected = ProjectOutsideBox({0, -0.01f, 0}, center, rotation, half_extent, 0.05f);
  EXPECT_NEAR(projected.y, 0.05f, 1e-6f);
  EXPECT_EQ(ProjectOutsideBox({0, 1, 0}, center, rotation, half_extent, 0.05f), glm::vec3(0, 1, 0));
  const glm::quat wall_rotation = glm::angleAxis(1.5707963f, glm::vec3(0, 0, 1));
  const auto wall_contact = ProjectOutsideBox({-0.05f, 0, 0}, {}, wall_rotation, {0.5f, 0.1f, 1}, 0.05f);
  EXPECT_NEAR(wall_contact.x, -0.15f, 1e-6f);
}

TEST(TreePhysics, SphereContactProjectsRodRadius) {
  const glm::vec3 center{1, 2, 3};
  EXPECT_EQ(ProjectOutsideSphere({1, 2, 3}, center, 0.5f, 0.1f), glm::vec3(1, 2.6f, 3));
  EXPECT_NEAR(ProjectOutsideSphere({1.55f, 2, 3}, center, 0.5f, 0.1f).x, 1.6f, 1e-6f);
  EXPECT_EQ(ProjectOutsideSphere({2, 2, 3}, center, 0.5f, 0.1f), glm::vec3(2, 2, 3));
}

TEST(TreePhysics, CylinderContactProjectsRodRadiusAtSideCapAndRim) {
  const glm::quat rotation{1, 0, 0, 0};
  EXPECT_NEAR(ProjectOutsideCylinder({0.95f, 0, 0}, {}, rotation, 1, 2, 0.1f).x, 1.1f, 1e-6f);
  EXPECT_NEAR(ProjectOutsideCylinder({0, 1.95f, 0}, {}, rotation, 1, 2, 0.1f).y, 2.1f, 1e-6f);
  EXPECT_NEAR(ProjectOutsideCylinder({0, 0, 0}, {}, rotation, 1, 2, 0.1f).z, 1.1f, 1e-6f);
  const auto rim = ProjectOutsideCylinder({1.05f, 2.05f, 0}, {}, rotation, 1, 2, 0.1f);
  EXPECT_NEAR(glm::length(rim - glm::vec3(1, 2, 0)), 0.1f, 1e-6f);
  EXPECT_EQ(ProjectOutsideCylinder({1.2f, 2.2f, 0}, {}, rotation, 1, 2, 0.1f), glm::vec3(1.2f, 2.2f, 0));
  const auto tilted = glm::angleAxis(glm::half_pi<float>(), glm::vec3(0, 0, 1));
  EXPECT_NEAR(ProjectOutsideCylinder({-1.95f, 0, 0}, {}, tilted, 1, 2, 0.1f).x, -2.1f, 1e-6f);
}

TEST(TreePhysics, InteriorContactTurnsRodThroughItsEndpoints) {
  glm::vec3 start{-1, 0.2f, 0};
  glm::vec3 end{1, 0.2f, 0};
  constexpr float t = 0.75f;
  const glm::vec3 contact = glm::mix(start, end, t);
  const glm::vec3 projected = ProjectOutsideSphere(contact, {0.5f, 0, 0}, 0.3f, 0.1f);
  ASSERT_TRUE(SolveRodContact(start, end, 1, 1, t, projected, contact, 0));
  EXPECT_GT(end.y - start.y, 0.1f);
  EXPECT_NEAR(glm::mix(start, end, t).y, 0.4f, 1e-5f);
}

TEST(TreePhysics, ContactFrictionOpposesLateralSlip) {
  glm::vec3 start{-1, 0.2f, 0};
  glm::vec3 end{1, 0.2f, 0};
  const glm::vec3 contact = glm::mix(start, end, 0.5f);
  ASSERT_TRUE(
      SolveRodContact(start, end, 1, 1, 0.5f, contact + glm::vec3(0, 0.1f, 0), contact - glm::vec3(0.1f, 0, 0), 1.0f));
  EXPECT_NEAR(glm::mix(start, end, 0.5f).x, -0.1f, 1e-5f);
  EXPECT_NEAR(glm::mix(start, end, 0.5f).y, 0.3f, 1e-5f);
}

TEST(TreePhysics, SupportedContactFrictionStopsSlidingWithoutPenetration) {
  const glm::vec3 normal{0, 1, 0};
  const glm::vec3 gravity{0, -10, 0};
  glm::vec3 velocity{0.05f, 0, 0};
  velocity = ApplySupportFriction(velocity, normal, gravity, 0.01f, 0.5f);
  EXPECT_NEAR(velocity.x, 0, 1e-6f);
  EXPECT_EQ(ApplySupportFriction({1, 0, 0}, normal, gravity, 0.01f, 0), glm::vec3(1, 0, 0));
  EXPECT_EQ(ApplySupportFriction({1, 0, 0}, normal, {0, 0, 0}, 0.01f, 0.5f), glm::vec3(1, 0, 0));
}

TEST(TreePhysics, ForkSharesJunctionUnderGravity) {
  evo_engine::Application application;
  evo_engine::ApplicationContextScope scope(application);
  Tree tree;
  DynamicTreeSkeleton physics;
  const auto skeleton = MakeFork();
  tree.shoot_model.Initialize(skeleton);
  ASSERT_TRUE(physics.InitializePhysics(tree));
  EXPECT_FALSE(physics.physics_running);
  const auto rest = tree.shoot_model.PeekPhysicsPose();
  ASSERT_EQ(rest.size(), 3);
  physics.physics_settings.stretch_stiffness = 100.0f;
  physics.physics_settings.bend_stiffness = 5.0f;
  physics.physics_running = true;
  for (int i = 0; i < 120; ++i)
    physics.StepPhysics(tree);
  const auto& pose = tree.shoot_model.PeekPhysicsPose();
  ASSERT_EQ(pose.size(), rest.size());
  for (const auto& node : pose)
    EXPECT_TRUE(FinitePose(node));
  EXPECT_NEAR(glm::length(pose[0].position - rest[0].position), 0.0f, 1e-5f);
  const auto junction = pose[0].position + pose[0].rotation * glm::vec3(0, 0, -1) * pose[0].length;
  EXPECT_NEAR(glm::length(junction - pose[1].position), 0.0f, 1e-3f);
  EXPECT_NEAR(glm::length(junction - pose[2].position), 0.0f, 1e-3f);
  EXPECT_GT(glm::length(pose[1].position + pose[1].rotation * glm::vec3(0, 0, -1) * pose[1].length -
                        (rest[1].position + rest[1].rotation * glm::vec3(0, 0, -1) * rest[1].length)),
            0.01f);
  EXPECT_NEAR(glm::length(tree.shoot_model.PeekShootSkeleton().PeekNode(1).info.global_position - rest[1].position),
              0.0f, 1e-6f);
  physics.ResetPhysics(tree);
  EXPECT_FALSE(physics.HasPhysics(tree));
  EXPECT_NEAR(glm::length(tree.shoot_model.PeekPhysicsPose()[1].position - rest[1].position), 0.0f, 1e-6f);
}

TEST(TreePhysics, GpuRestStatePreservesForkTopologyAndDownstreamScaling) {
  evo_engine::Application application;
  evo_engine::ApplicationContextScope scope(application);
  Tree tree;
  tree.shoot_model.Initialize(MakeFork());
  DynamicTreeSkeleton physics;
  ASSERT_TRUE(physics.InitializePhysics(tree));
  const auto state = physics.ExportGpuRestState();
  ASSERT_EQ(state.vertices.size(), 4u);
  ASSERT_EQ(state.rods.size(), 3u);
  ASSERT_EQ(state.angles.size(), 2u);
  EXPECT_FLOAT_EQ(state.vertices[0].position_inverse_mass.w, 0.0f);
  const uint32_t junction = state.rods[0].links.y;
  EXPECT_EQ(state.rods[1].links.x, junction);
  EXPECT_EQ(state.rods[2].links.x, junction);
  EXPECT_EQ(state.vertices[junction].incident_rods.y, 3u);
  EXPECT_GT(state.rods[0].length_scale_mass_radius.y, state.rods[1].length_scale_mass_radius.y);
  EXPECT_GT(state.rods[0].length_scale_mass_radius.z, state.rods[1].length_scale_mass_radius.z);
  EXPECT_EQ(state.node_rods[0], 0u);
  EXPECT_EQ(state.node_rods[1], 1u);
  EXPECT_EQ(state.node_rods[2], 2u);
  EXPECT_FLOAT_EQ(state.rods[0].length_scale_mass_radius.w, 0.05f);
  EXPECT_FLOAT_EQ(state.rods[1].length_scale_mass_radius.w, 0.025f);
  for (const auto& angle : state.angles)
    EXPECT_GT(angle.bend_scale.x, 0.0f);
}

TEST(TreePhysics, ZeroLengthShootNodeHasNoRodButItsChildDoes) {
  evo_engine::Application application;
  evo_engine::ApplicationContextScope scope(application);
  Tree tree;
  auto skeleton = MakeFork();
  skeleton.RefNode(1).info.length = 0.0f;
  const auto child = skeleton.Extend(1, false);
  auto& info = skeleton.RefNode(child).info;
  info.global_position = {0, 1, 0};
  info.length = 0.4f;
  info.thickness = 0.02f;
  info.global_rotation = glm::rotation(glm::vec3(0, 0, -1), glm::vec3(0, 1, 0));
  skeleton.SortLists();
  tree.shoot_model.Initialize(skeleton);
  DynamicTreeSkeleton physics;
  ASSERT_TRUE(physics.InitializePhysics(tree));
  EXPECT_FALSE(physics.HasPhysicsRodForNode(1));
  EXPECT_TRUE(physics.HasPhysicsRodForNode(child));
  const auto state = physics.ExportGpuRestState();
  EXPECT_EQ(state.node_rods[1], UINT32_MAX);
  EXPECT_NE(state.node_rods[child], UINT32_MAX);
}

TEST(TreePhysics, SustainedBendDetachesAndDropsBranchWithoutChangingGrowthSkeleton) {
  evo_engine::Application application;
  evo_engine::ApplicationContextScope scope(application);
  Tree tree;
  DynamicTreeSkeleton physics;
  tree.shoot_model.Initialize(MakeFork());
  ASSERT_TRUE(physics.InitializePhysics(tree));
  physics.physics_settings.stretch_stiffness = 100.0f;
  physics.physics_settings.bend_stiffness = 5.0f;
  physics.physics_settings.fracture_enabled = true;
  physics.physics_settings.fracture_angle_degrees = 1.0f;
  physics.physics_settings.fracture_duration = 0.04f;
  physics.physics_running = true;
  physics.StepPhysics(tree);
  EXPECT_FALSE(tree.shoot_model.PeekPhysicsPose()[1].detached);
  for (int i = 0; i < 120; ++i)
    physics.StepPhysics(tree);
  const auto& pose = tree.shoot_model.PeekPhysicsPose();
  ASSERT_TRUE(pose[1].detached || pose[2].detached);
  const int broken = pose[1].detached ? 1 : 2;
  const auto junction = pose[0].position + pose[0].rotation * glm::vec3(0, 0, -1) * pose[0].length;
  EXPECT_GT(glm::length(pose[broken].position - junction), 0.1f);
  EXPECT_EQ(tree.shoot_model.PeekShootSkeleton().PeekNode(broken).GetParentHandle(), 0);
  std::vector<GpuTreeNode> packed;
  AppendGpuTreeNodes(tree.shoot_model.PeekShootSkeleton(), TreeMeshGeneratorSettings{}, nullptr, 0, true, packed,
                     &pose);
  EXPECT_EQ(packed[broken].links.x, UINT32_MAX);
  physics.ResetPhysics(tree);
  EXPECT_FALSE(tree.shoot_model.PeekPhysicsPose()[broken].detached);
}

TEST(TreePhysics, ThickJunctionNeedsLessAngularDeflectionForTheSameBendStrain) {
  DynamicTreeSkeleton::PhysicsSettings settings;
  const float reference = settings.FractureBendLimit(0.05f, 1.0f);
  EXPECT_NEAR(reference, glm::radians(settings.fracture_angle_degrees), 1e-6f);
  EXPECT_LT(settings.FractureBendLimit(0.2f, 1.0f), reference);
  EXPECT_GT(settings.FractureBendLimit(0.01f, 1.0f), reference);
  EXPECT_LT(settings.FractureBendLimit(0.05f, 0.5f), reference);
}

TEST(TreePhysics, StrongShakeCanDetachThickBranch) {
  evo_engine::Application application;
  evo_engine::ApplicationContextScope scope(application);
  Tree tree;
  DynamicTreeSkeleton physics;
  auto skeleton = MakeFork();
  skeleton.RefNode(0).info.thickness = 0.25f;
  skeleton.RefNode(1).info.thickness = 0.15f;
  skeleton.RefNode(2).info.thickness = 0.01f;
  tree.shoot_model.Initialize(skeleton);
  evo_engine::GlobalTransform model;
  ASSERT_TRUE(physics.InitializePhysics(tree, model));
  physics.physics_settings.gravity = {};
  physics.physics_settings.fracture_enabled = true;
  physics.physics_running = true;
  for (int i = 0; i < 180 && !tree.shoot_model.PeekPhysicsPose()[1].detached; ++i) {
    model.SetValue({}, glm::angleAxis(i % 40 < 20 ? 2.0f : -2.0f, glm::vec3(0, 0, 1)), {1, 1, 1});
    physics.StepPhysics(tree, model);
  }
  EXPECT_TRUE(tree.shoot_model.PeekPhysicsPose()[1].detached);
}

TEST(TreePhysics, DetachedBranchAcceleratesUnderGravity) {
  evo_engine::Application application;
  evo_engine::ApplicationContextScope scope(application);
  Tree tree;
  DynamicTreeSkeleton physics;
  tree.shoot_model.Initialize(MakeFork());
  ASSERT_TRUE(physics.InitializePhysics(tree));
  physics.physics_settings.stretch_stiffness = 100.0f;
  physics.physics_settings.bend_stiffness = 5.0f;
  physics.physics_settings.fracture_enabled = true;
  physics.physics_settings.fracture_angle_degrees = 0;
  physics.physics_settings.fracture_duration = 0;
  physics.physics_running = true;
  int broken = -1;
  for (int i = 0; i < 120 && broken < 0; ++i) {
    physics.StepPhysics(tree);
    const auto& pose = tree.shoot_model.PeekPhysicsPose();
    broken = pose[1].detached ? 1 : pose[2].detached ? 2 : -1;
  }
  ASSERT_GE(broken, 0);
  physics.physics_settings.stretch_stiffness = 8192.0f;
  physics.physics_settings.bend_stiffness = 512.0f;
  const float y0 = tree.shoot_model.PeekPhysicsPose()[broken].position.y;
  for (int i = 0; i < 20; ++i)
    physics.StepPhysics(tree);
  const float y1 = tree.shoot_model.PeekPhysicsPose()[broken].position.y;
  for (int i = 0; i < 20; ++i)
    physics.StepPhysics(tree);
  const float y2 = tree.shoot_model.PeekPhysicsPose()[broken].position.y;
  EXPECT_LT(y1, y0);
  EXPECT_LT(y2 - y1, y1 - y0 - 0.5f);
}

TEST(TreePhysics, FallenBranchRotatesAfterTipHitsGround) {
  evo_engine::Application application;
  evo_engine::ApplicationContextScope scope(application);
  evo_engine::ApplicationInitializationSettings settings;
  settings.allow_empty_project = true;
  settings.load_default_resources = false;
  settings.load_project_assets = false;
  settings.load_project_start_scene = false;
  settings.enable_runtime_packages = false;
  application.Initialize(settings);
  application.RegisterPrivateComponent<Tree>("Tree");
  application.RegisterPrivateComponent<DsBoxCollider>("DsBoxCollider");
  auto scene = evo_engine::AssetManager::CreateTemporaryAsset<evo_engine::Scene>();
  application.Attach(scene);

  EcoSysLabResourceDirectory resource_directory;

  const auto ground_entity = scene->CreateEntity("Ground");
  const auto ground = scene->GetOrSetPrivateComponent<DsBoxCollider>(ground_entity).lock();
  ASSERT_TRUE(ground);
  ground->scale = {10, 0.5f, 10};
  evo_engine::GlobalTransform ground_transform;
  ground_transform.SetPosition({0, -0.5f, 0});
  scene->SetDataComponent(ground_entity, ground_transform);

  const auto tree_entity = scene->CreateEntity("Tree");
  const auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock();
  ASSERT_TRUE(tree);
  auto skeleton = MakeFork();
  skeleton.RefNode(1).info.thickness = 0.15f;
  skeleton.RefNode(1).info.global_rotation = glm::rotation(glm::vec3(0, 0, -1), glm::normalize(glm::vec3(1, -1, 0)));
  tree->shoot_model.Initialize(skeleton);
  DynamicTreeSkeleton physics;
  ASSERT_TRUE(physics.InitializePhysics(*tree));
  physics.physics_settings.stretch_stiffness = 100.0f;
  physics.physics_settings.bend_stiffness = 5.0f;
  physics.physics_settings.fracture_enabled = true;
  physics.physics_settings.fracture_angle_degrees = 0;
  physics.physics_settings.fracture_duration = 0;
  physics.physics_running = true;
  bool tip_contact = false;
  constexpr float contact_height = 0.075f;
  for (int i = 0; i < 240; ++i) {
    physics.StepPhysics(*tree);
    const auto& pose = tree->shoot_model.PeekPhysicsPose()[1];
    const glm::vec3 tip = pose.position + pose.rotation * glm::vec3(0, 0, -1) * pose.length;
    if (pose.detached && tip.y <= contact_height + 0.01f && pose.position.y > tip.y + 0.3f) {
      tip_contact = true;
      break;
    }
  }
  ASSERT_TRUE(tip_contact);
  physics.physics_settings.stretch_stiffness = 8192.0f;
  physics.physics_settings.bend_stiffness = 512.0f;
  const auto& contact_pose = tree->shoot_model.PeekPhysicsPose()[1];
  const float initial_direction_y = (contact_pose.rotation * glm::vec3(0, 0, -1)).y;
  for (int i = 0; i < 20; ++i)
    physics.StepPhysics(*tree);
  const auto& settled_pose = tree->shoot_model.PeekPhysicsPose()[1];
  const glm::vec3 settled_tip =
      settled_pose.position + settled_pose.rotation * glm::vec3(0, 0, -1) * settled_pose.length;
  EXPECT_NEAR(settled_tip.y, contact_height, 0.05f);
  EXPECT_LT(settled_pose.position.y, contact_height + 0.15f);
  EXPECT_NEAR(settled_pose.length, 0.8f, 0.025f);
  EXPECT_GT((settled_pose.rotation * glm::vec3(0, 0, -1)).y, initial_direction_y + 0.2f);
}

TEST(TreePhysics, NearlyUprightFallenBranchAcceleratesAroundGroundedTip) {
  evo_engine::Application application;
  evo_engine::ApplicationContextScope scope(application);
  evo_engine::ApplicationInitializationSettings settings;
  settings.allow_empty_project = true;
  settings.load_default_resources = false;
  settings.load_project_assets = false;
  settings.load_project_start_scene = false;
  settings.enable_runtime_packages = false;
  application.Initialize(settings);
  application.RegisterPrivateComponent<Tree>("Tree");
  application.RegisterPrivateComponent<DsBoxCollider>("DsBoxCollider");
  auto scene = evo_engine::AssetManager::CreateTemporaryAsset<evo_engine::Scene>();
  application.Attach(scene);
  EcoSysLabResourceDirectory resource_directory;
  const auto ground_entity = scene->CreateEntity("Ground");
  const auto ground = scene->GetOrSetPrivateComponent<DsBoxCollider>(ground_entity).lock();
  ASSERT_TRUE(ground);
  ground->scale = {10, 0.5f, 10};
  evo_engine::GlobalTransform ground_transform;
  ground_transform.SetPosition({0, -0.5f, 0});
  scene->SetDataComponent(ground_entity, ground_transform);

  const auto tree_entity = scene->CreateEntity("Tree");
  const auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock();
  ASSERT_TRUE(tree);
  auto skeleton = MakeFork();
  skeleton.RefNode(1).info.thickness = 0.15f;
  skeleton.RefNode(1).info.global_rotation = glm::rotation(glm::vec3(0, 0, -1), glm::normalize(glm::vec3(0.2f, -1, 0)));
  tree->shoot_model.Initialize(skeleton);
  DynamicTreeSkeleton physics;
  ASSERT_TRUE(physics.InitializePhysics(*tree));
  physics.physics_settings.stretch_stiffness = 100;
  physics.physics_settings.bend_stiffness = 5;
  physics.physics_settings.fracture_enabled = true;
  physics.physics_settings.fracture_angle_degrees = 0;
  physics.physics_settings.fracture_duration = 0;
  physics.physics_running = true;
  bool tip_contact = false;
  for (int i = 0; i < 240; ++i) {
    physics.StepPhysics(*tree);
    const auto& pose = tree->shoot_model.PeekPhysicsPose()[1];
    const glm::vec3 tip = pose.position + pose.rotation * glm::vec3(0, 0, -1) * pose.length;
    if (pose.detached && tip.y <= 0.085f && pose.position.y > tip.y + 0.3f) {
      tip_contact = true;
      break;
    }
  }
  ASSERT_TRUE(tip_contact);
  physics.physics_settings.stretch_stiffness = 8192;
  physics.physics_settings.bend_stiffness = 512;
  const auto angle = [&] {
    const glm::vec3 direction = tree->shoot_model.PeekPhysicsPose()[1].rotation * glm::vec3(0, 0, -1);
    return std::atan2(direction.x, -direction.y);
  };
  const float initial_angle = angle();
  for (int i = 0; i < 5; ++i)
    physics.StepPhysics(*tree);
  const float first_angle = angle();
  for (int i = 0; i < 5; ++i)
    physics.StepPhysics(*tree);
  const float second_angle = angle();
  for (int i = 0; i < 5; ++i)
    physics.StepPhysics(*tree);
  const float third_angle = angle();
  EXPECT_GT(second_angle - first_angle, 1.2f * (first_angle - initial_angle));
  EXPECT_GT(third_angle - second_angle, 1.2f * (second_angle - first_angle));
  EXPECT_LT(tree->shoot_model.PeekPhysicsPose()[1].position.y, 0.91f);
  for (int i = 0; i < 120; ++i)
    physics.StepPhysics(*tree);
  const auto& settled = tree->shoot_model.PeekPhysicsPose()[1];
  const glm::vec3 settled_tip = settled.position + settled.rotation * glm::vec3(0, 0, -1) * settled.length;
  EXPECT_TRUE(FinitePose(settled));
  EXPECT_NEAR(settled.position.y, 0.075f, 0.05f);
  EXPECT_NEAR(settled_tip.y, 0.075f, 0.05f);
  const glm::vec3 resting_position = settled.position;
  for (int i = 0; i < 120; ++i)
    physics.StepPhysics(*tree);
  const glm::vec3 later_position = tree->shoot_model.PeekPhysicsPose()[1].position;
  EXPECT_LT(glm::length(glm::vec2(later_position.x - resting_position.x, later_position.z - resting_position.z)), 0.1f);
}

TEST(TreePhysics, MultilevelTreeRemainsFiniteUnderGravityAndMovingAnchor) {
  evo_engine::Application application;
  evo_engine::ApplicationContextScope scope(application);
  Tree tree;
  DynamicTreeSkeleton physics;
  auto skeleton = MakeFork();
  const auto child = skeleton.Extend(1, false);
  auto& info = skeleton.RefNode(child).info;
  const auto& parent = skeleton.PeekNode(1).info;
  info.global_position = parent.GetGlobalEndPosition();
  info.global_rotation = parent.global_rotation;
  info.length = 0.6f;
  info.thickness = 0.025f;
  skeleton.SortLists();
  tree.shoot_model.Initialize(skeleton);
  ASSERT_TRUE(physics.InitializePhysics(tree));
  const auto rest_base = tree.shoot_model.PeekPhysicsPose()[0].position;
  physics.physics_settings.time_step = 1.0f / 240.0f;
  physics.physics_settings.anchor_offset = {0.1f, 0, 0};
  physics.physics_running = true;
  for (int i = 0; i < 240; ++i)
    physics.StepPhysics(tree);
  const auto& pose = tree.shoot_model.PeekPhysicsPose();
  ASSERT_EQ(pose.size(), 4);
  for (const auto& node : pose)
    EXPECT_TRUE(FinitePose(node));
  EXPECT_NEAR(glm::length(pose[0].position - rest_base - physics.physics_settings.anchor_offset), 0.0f, 1e-5f);
  const auto junction = pose[1].position + pose[1].rotation * glm::vec3(0, 0, -1) * pose[1].length;
  EXPECT_NEAR(glm::length(junction - pose[child].position), 0.0f, 1e-3f);
}

TEST(TreePhysics, CantileverReturnsTowardRestAfterGravityIsRemoved) {
  evo_engine::Application application;
  evo_engine::ApplicationContextScope scope(application);
  Tree tree;
  DynamicTreeSkeleton physics;
  BaseSkeleton skeleton;
  auto& info = skeleton.RefNode(0).info;
  info.length = 1.0f;
  info.thickness = 0.05f;
  const auto child = skeleton.Extend(0, false);
  skeleton.RefNode(child).info.global_position = {0, 0, -1};
  skeleton.RefNode(child).info.length = 1.0f;
  skeleton.RefNode(child).info.thickness = 0.05f;
  skeleton.SortLists();
  tree.shoot_model.Initialize(skeleton);
  ASSERT_TRUE(physics.InitializePhysics(tree));
  physics.physics_settings.damping = 3.0f;
  physics.physics_running = true;
  for (int i = 0; i < 120; ++i)
    physics.StepPhysics(tree);
  const auto& bent = tree.shoot_model.PeekPhysicsPose()[child];
  const auto bent_end = bent.position + bent.rotation * glm::vec3(0, 0, -1) * bent.length;
  const float bent_distance = glm::length(bent_end - glm::vec3(0, 0, -2));
  EXPECT_GT(bent_distance, 0.001f);
  physics.physics_settings.gravity = {};
  for (int i = 0; i < 600; ++i)
    physics.StepPhysics(tree);
  const auto& relaxed = tree.shoot_model.PeekPhysicsPose()[child];
  const auto relaxed_end = relaxed.position + relaxed.rotation * glm::vec3(0, 0, -1) * relaxed.length;
  EXPECT_LT(glm::length(relaxed_end - glm::vec3(0, 0, -2)), bent_distance);
}

TEST(TreePhysics, LateralVisualOffsetFollowsBentJunction) {
  evo_engine::Application application;
  evo_engine::ApplicationContextScope scope(application);
  Tree tree;
  DynamicTreeSkeleton physics;
  auto skeleton = MakeFork();
  skeleton.RefNode(2).info.global_position += glm::vec3(0.05f, 0, 0);
  tree.shoot_model.Initialize(skeleton);
  evo_engine::GlobalTransform model;
  ASSERT_TRUE(physics.InitializePhysics(tree, model));
  physics.physics_settings.gravity = {};
  physics.physics_running = true;
  for (int i = 0; i < 120; ++i) {
    model.SetPosition({0.2f * std::sin(6.2831853f * i / 30.0f), 0, 0});
    physics.StepPhysics(tree, model);
  }
  const auto& pose = tree.shoot_model.PeekPhysicsPose();
  const auto junction = pose[0].position + pose[0].rotation * glm::vec3(0, 0, -1) * pose[0].length;
  EXPECT_NEAR(glm::length(pose[2].position - junction), 0.05f, 1e-3f);
}

TEST(TreePhysics, EntityMotionShakesForkWithoutMovingLocalRoot) {
  evo_engine::Application application;
  evo_engine::ApplicationContextScope scope(application);
  Tree tree;
  DynamicTreeSkeleton physics;
  tree.shoot_model.Initialize(MakeFork());
  evo_engine::GlobalTransform model;
  model.SetValue({3, 0, 1}, glm::quat(glm::vec3(0, 0.3f, 0)), {1, 1, 1});
  ASSERT_TRUE(physics.InitializePhysics(tree, model));
  const auto rest = tree.shoot_model.PeekPhysicsPose();
  physics.physics_settings.gravity = {};
  physics.physics_running = true;
  float maximum_tip_offset = 0;
  float maximum_trunk_offset = 0;
  float maximum_sibling_offset = 0;
  for (int i = 0; i < 90; ++i) {
    const float phase = 6.2831853f * i / 30.0f;
    model.SetValue({3 + 0.2f * std::sin(phase), 0, 1}, glm::quat(glm::vec3(0, 0.3f, 0)), {1, 1, 1});
    physics.StepPhysics(tree, model);
    const auto& pose = tree.shoot_model.PeekPhysicsPose();
    ASSERT_TRUE(physics.HasPhysics(tree));
    for (const auto& node : pose)
      ASSERT_TRUE(FinitePose(node));
    EXPECT_NEAR(glm::length(pose[0].position - rest[0].position), 0.0f, 1e-4f);
    const auto junction = pose[0].position + pose[0].rotation * glm::vec3(0, 0, -1) * pose[0].length;
    EXPECT_NEAR(glm::length(pose[1].position - junction), 0.0f, 1e-3f);
    EXPECT_NEAR(glm::length(pose[2].position - junction), 0.0f, 1e-3f);
    const auto tip = pose[1].position + pose[1].rotation * glm::vec3(0, 0, -1) * pose[1].length;
    const auto rest_tip = rest[1].position + rest[1].rotation * glm::vec3(0, 0, -1) * rest[1].length;
    maximum_tip_offset = glm::max(maximum_tip_offset, glm::length(tip - rest_tip));
    const auto rest_junction = rest[0].position + rest[0].rotation * glm::vec3(0, 0, -1) * rest[0].length;
    maximum_trunk_offset = glm::max(maximum_trunk_offset, glm::length(junction - rest_junction));
    const auto sibling_tip = pose[2].position + pose[2].rotation * glm::vec3(0, 0, -1) * pose[2].length;
    const auto rest_sibling_tip = rest[2].position + rest[2].rotation * glm::vec3(0, 0, -1) * rest[2].length;
    maximum_sibling_offset = glm::max(maximum_sibling_offset, glm::length(sibling_tip - rest_sibling_tip));
  }
  EXPECT_GT(maximum_tip_offset, 0.005f);
  EXPECT_GT(maximum_trunk_offset, 0.001f);
  EXPECT_GT(maximum_sibling_offset, 0.001f);

  model.SetValue({3, 0, 1}, glm::quat(glm::vec3(0, 0.3f, 0)), {1, 1, 1});
  ASSERT_TRUE(physics.InitializePhysics(tree, model));
  physics.physics_running = true;
  float maximum_rotation_offset = 0;
  for (int i = 0; i < 60; ++i) {
    const float phase = 6.2831853f * i / 30.0f;
    model.SetValue({3, 0, 1}, glm::quat(glm::vec3(0, 0.3f + 0.15f * std::sin(phase), 0)), {1, 1, 1});
    physics.StepPhysics(tree, model);
    const auto& pose = tree.shoot_model.PeekPhysicsPose();
    const auto tip = pose[1].position + pose[1].rotation * glm::vec3(0, 0, -1) * pose[1].length;
    const auto rest_tip = rest[1].position + rest[1].rotation * glm::vec3(0, 0, -1) * rest[1].length;
    maximum_rotation_offset = glm::max(maximum_rotation_offset, glm::length(tip - rest_tip));
  }
  EXPECT_GT(maximum_rotation_offset, 0.001f);

  physics.physics_running = false;
  const auto paused = tree.shoot_model.PeekPhysicsPose();
  model.SetValue({5, 0, 1}, glm::quat(glm::vec3(0, 0.7f, 0)), {1, 1, 1});
  physics.StepPhysics(tree, model);
  EXPECT_NEAR(glm::length(tree.shoot_model.PeekPhysicsPose()[1].position - paused[1].position), 0.0f, 1e-6f);
  physics.physics_running = true;
  physics.StepPhysics(tree, model);
  EXPECT_TRUE(physics.HasPhysics(tree));
  model.SetScale({2, 2, 2});
  physics.StepPhysics(tree, model);
  EXPECT_FALSE(physics.HasPhysics(tree));
}

TEST(TreePhysics, ThickBranchResistsShakingMoreThanThinBranch) {
  evo_engine::Application application;
  evo_engine::ApplicationContextScope scope(application);
  Tree tree;
  DynamicTreeSkeleton physics;
  auto skeleton = MakeFork();
  skeleton.RefNode(0).info.thickness = 0.2f;
  skeleton.RefNode(1).info.thickness = 0.16f;
  skeleton.RefNode(2).info.thickness = 0.02f;
  tree.shoot_model.Initialize(skeleton);
  evo_engine::GlobalTransform model;
  ASSERT_TRUE(physics.InitializePhysics(tree, model));
  physics.physics_settings.gravity = {};
  physics.physics_running = true;
  const auto rest = tree.shoot_model.PeekPhysicsPose();
  physics.StepPhysics(tree, model);
  for (int handle : {1, 2}) {
    const auto& pose = tree.shoot_model.PeekPhysicsPose()[handle];
    const auto tip = pose.position + pose.rotation * glm::vec3(0, 0, -1) * pose.length;
    const auto rest_tip = rest[handle].position + rest[handle].rotation * glm::vec3(0, 0, -1) * rest[handle].length;
    EXPECT_NEAR(glm::length(tip - rest_tip), 0.0f, 1e-3f);
  }
  float thick_offset = 0;
  float thin_offset = 0;
  for (int i = 0; i < 90; ++i) {
    model.SetPosition({0.2f * std::sin(6.2831853f * i / 30.0f), 0, 0});
    physics.StepPhysics(tree, model);
    const auto& pose = tree.shoot_model.PeekPhysicsPose();
    for (const auto [handle, maximum] : {std::pair{1, &thick_offset}, std::pair{2, &thin_offset}}) {
      const auto tip = pose[handle].position + pose[handle].rotation * glm::vec3(0, 0, -1) * pose[handle].length;
      const auto rest_tip = rest[handle].position + rest[handle].rotation * glm::vec3(0, 0, -1) * rest[handle].length;
      *maximum = glm::max(*maximum, glm::length(tip - rest_tip));
    }
  }
  EXPECT_GT(thin_offset, 0.01f);
  EXPECT_LT(thick_offset, thin_offset) << "thick=" << thick_offset << " thin=" << thin_offset;
}

TEST(TreePhysics, DefaultDampingRetainsMotionAfterAnchorShake) {
  evo_engine::Application application;
  evo_engine::ApplicationContextScope scope(application);
  const auto motion_after_shake = [](const float damping) {
    Tree tree;
    DynamicTreeSkeleton physics;
    tree.shoot_model.Initialize(MakeFork());
    evo_engine::GlobalTransform model;
    EXPECT_TRUE(physics.InitializePhysics(tree, model));
    physics.physics_settings.gravity = {};
    physics.physics_settings.damping = damping;
    physics.physics_running = true;
    const auto& rest = tree.shoot_model.PeekPhysicsPose()[1];
    const auto rest_tip = rest.position + rest.rotation * glm::vec3(0, 0, -1) * rest.length;
    for (int i = 0; i < 30; ++i) {
      model.SetPosition({0.2f * std::sin(6.2831853f * i / 30.0f), 0, 0});
      physics.StepPhysics(tree, model);
    }
    model.SetPosition({0, 0, 0});
    float motion = 0;
    for (int i = 0; i < 120; ++i) {
      physics.StepPhysics(tree, model);
      const auto& pose = tree.shoot_model.PeekPhysicsPose()[1];
      const auto tip = pose.position + pose.rotation * glm::vec3(0, 0, -1) * pose.length;
      motion += glm::length(tip - rest_tip);
    }
    return motion;
  };
  const float default_motion = motion_after_shake(DynamicTreeSkeleton::PhysicsSettings{}.damping);
  const float heavily_damped_motion = motion_after_shake(2.0f);
  EXPECT_GT(default_motion, heavily_damped_motion * 1.05f);
}

TEST(TreePhysics, NestedThinBranchSettlesAfterEntityTranslation) {
  evo_engine::Application application;
  evo_engine::ApplicationContextScope scope(application);
  Tree tree;
  DynamicTreeSkeleton physics;
  auto skeleton = MakeFork();
  const auto child = skeleton.Extend(1, false);
  auto& info = skeleton.RefNode(child).info;
  info.global_position = skeleton.PeekNode(1).info.GetGlobalEndPosition();
  info.global_rotation = skeleton.PeekNode(1).info.global_rotation;
  info.length = 0.4f;
  info.thickness = 0.01f;
  skeleton.SortLists();
  tree.shoot_model.Initialize(skeleton);
  evo_engine::GlobalTransform model;
  ASSERT_TRUE(physics.InitializePhysics(tree, model));
  const auto rest = tree.shoot_model.PeekPhysicsPose();
  const auto rest_tip = rest[child].position + rest[child].rotation * glm::vec3(0, 0, -1) * rest[child].length;
  physics.physics_settings.gravity = {};
  physics.physics_settings.damping = 1.0f;
  physics.physics_running = true;
  model.SetPosition({0.5f, 0, 0});
  for (int i = 0; i < 600; ++i)
    physics.StepPhysics(tree, model);
  const auto& pose = tree.shoot_model.PeekPhysicsPose();
  const auto tip = pose[child].position + pose[child].rotation * glm::vec3(0, 0, -1) * pose[child].length;
  EXPECT_NEAR(glm::length(tip - rest_tip), 0.0f, 0.02f);
  const auto parent_tip = pose[1].position + pose[1].rotation * glm::vec3(0, 0, -1) * pose[1].length;
  EXPECT_NEAR(glm::length(parent_tip - pose[child].position), 0.0f, 1e-3f);
}

TEST(TreePhysics, ComponentSettingsRoundTripWithoutSimulationState) {
  DynamicTreeSkeleton source;
  source.physics_settings.time_step = 0.01f;
  source.physics_settings.bend_stiffness = 120.0f;
  source.physics_settings.wind_drag = 7.5f;
  source.physics_settings.contact_friction = 0.8f;
  source.physics_settings.gravity = {1, -2, 3};
  source.physics_settings.anchor_offset = {0.1f, 0.2f, 0.3f};
  source.physics_settings.fracture_enabled = true;
  source.physics_settings.fracture_angle_degrees = 30.0f;
  source.physics_settings.fracture_duration = 0.2f;
  source.physics_running = true;
  source.gpu_physics_enabled = true;
  YAML::Emitter out;
  out << YAML::BeginMap;
  SerializeDynamicTreeSkeleton(out, source);
  out << YAML::EndMap;

  DynamicTreeSkeleton loaded;
  DeserializeDynamicTreeSkeleton(YAML::Load(out.c_str()), loaded);
  EXPECT_FLOAT_EQ(loaded.physics_settings.time_step, 0.01f);
  EXPECT_FLOAT_EQ(loaded.physics_settings.bend_stiffness, 120.0f);
  EXPECT_FLOAT_EQ(loaded.physics_settings.wind_drag, 7.5f);
  EXPECT_FLOAT_EQ(loaded.physics_settings.contact_friction, 0.8f);
  EXPECT_EQ(loaded.physics_settings.gravity, glm::vec3(1, -2, 3));
  EXPECT_EQ(loaded.physics_settings.anchor_offset, glm::vec3(0.1f, 0.2f, 0.3f));
  EXPECT_TRUE(loaded.physics_settings.fracture_enabled);
  EXPECT_FLOAT_EQ(loaded.physics_settings.fracture_angle_degrees, 30.0f);
  EXPECT_FLOAT_EQ(loaded.physics_settings.fracture_duration, 0.2f);
  EXPECT_FALSE(loaded.physics_running);
  EXPECT_TRUE(loaded.gpu_physics_enabled);
}

TEST(TreePhysics, LoadedGpuPhysicsStartsPausedFromOriginalPose) {
  evo_engine::Application application;
  evo_engine::ApplicationContextScope scope(application);
  evo_engine::Serialization::RegisterSerializationHandler<evo_engine::AssetRef>(
      [](YAML::Emitter& out, const evo_engine::AssetRef& reference) {
        reference.Serialize(out);
      },
      [](const YAML::Node& in, evo_engine::AssetRef& reference) {
        reference.Deserialize(in);
      },
      {}, "AssetRef");
  Tree source;
  source.shoot_model.Initialize(MakeFork());
  YAML::Emitter tree_out;
  tree_out << YAML::BeginMap;
  SerializeTree(tree_out, source);
  tree_out << YAML::EndMap;

  DynamicTreeSkeleton saved_physics;
  saved_physics.gpu_physics_enabled = true;
  saved_physics.physics_running = true;
  YAML::Emitter physics_out;
  physics_out << YAML::BeginMap;
  SerializeDynamicTreeSkeleton(physics_out, saved_physics);
  physics_out << YAML::EndMap;

  Tree loaded_tree;
  DynamicTreeSkeleton loaded_physics;
  DeserializeDynamicTreeSkeleton(YAML::Load(physics_out.c_str()), loaded_physics);
  DeserializeTree(YAML::Load(tree_out.c_str()), loaded_tree);
  loaded_physics.RestoreGpuPoseAfterLoad(loaded_tree);
  ASSERT_TRUE(loaded_physics.HasPhysics(loaded_tree));
  EXPECT_FALSE(loaded_physics.physics_running);
  const auto state = loaded_physics.ExportGpuRestState();
  ASSERT_EQ(state.rods.size(), 3u);
  const auto& pose = loaded_tree.shoot_model.PeekPhysicsPose();
  ASSERT_EQ(pose.size(), 3u);
  for (uint32_t node = 0; node < pose.size(); ++node) {
    const auto& original = loaded_tree.shoot_model.PeekShootSkeleton().PeekNode(node).info;
    EXPECT_EQ(pose[node].position, original.global_position);
    EXPECT_EQ(pose[node].rotation, original.global_rotation);
    const auto rod = state.node_rods[node];
    ASSERT_NE(rod, UINT32_MAX);
    const auto vertex = state.rods[rod].links.x;
    EXPECT_LT(glm::length(glm::vec3(state.vertices[vertex].position_inverse_mass) - original.global_position), 1e-5f);
  }

  loaded_physics.ResetPhysics(loaded_tree);
  loaded_physics.RestoreGpuPoseAfterLoad(loaded_tree);
  EXPECT_FALSE(loaded_physics.HasPhysics(loaded_tree));
}

TEST(TreePhysics, GpuGrowthReplacesDynamicRestAndResumesAfterward) {
  evo_engine::Application application;
  evo_engine::ApplicationContextScope scope(application);
  Tree tree;
  tree.shoot_model.Initialize(MakeFork());
  DynamicTreeSkeleton physics;
  physics.gpu_physics_enabled = true;
  ASSERT_TRUE(physics.SyncGpuPhysics(tree, GlobalTransform{}, false));
  EXPECT_TRUE(physics.physics_running);
  const auto original = physics.ExportGpuRestState();

  physics.ResetPhysics(tree);
  auto grown = MakeFork();
  grown.RefNode(1).info.length = 1.2f;
  tree.shoot_model.Initialize(grown);
  ASSERT_TRUE(physics.SyncGpuPhysics(tree, GlobalTransform{}, true));
  EXPECT_FALSE(physics.physics_running);
  const auto current = physics.ExportGpuRestState();
  const auto tip = [](const DynamicTreeSkeleton::GpuRestState& state) {
    return glm::vec3(state.vertices[state.rods[state.node_rods[1]].links.y].position_inverse_mass);
  };
  EXPECT_GT(glm::length(tip(current) - tip(original)), 0.3f);
  EXPECT_FLOAT_EQ(tree.shoot_model.PeekShootSkeleton().PeekNode(1).info.length, 1.2f);

  ASSERT_TRUE(physics.SyncGpuPhysics(tree, GlobalTransform{}, false));
  EXPECT_TRUE(physics.physics_running);
  EXPECT_LT(glm::length(tip(physics.ExportGpuRestState()) - tip(current)), 1e-5f);
}
