#include <gtest/gtest.h>
#include <cmath>
#include <glm/gtx/quaternion.hpp>
#include "Application.hpp"
#include "ApplicationContext.hpp"
#include "DynamicTreeSkeleton.hpp"
#include "EcoSysLabSerializationAdapters.hpp"
#include "EvoEngine_SDK_PCH.hpp"
#include "PrivateComponentRef.hpp"
#include "Strands.hpp"
#include "Tree.hpp"

using namespace eco_sys_lab_package;

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
}  // namespace

TEST(TreePhysics, ForkSharesJunctionAndRespondsToTipForce) {
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
  physics.physics_settings.gravity = {};
  physics.physics_settings.stretch_stiffness = 100.0f;
  physics.physics_settings.bend_stiffness = 5.0f;
  physics.physics_settings.tip_force = {10, 0, 0};
  physics.physics_settings.force_node = 1;
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

TEST(TreePhysics, CantileverReturnsTowardRestAfterForceIsRemoved) {
  evo_engine::Application application;
  evo_engine::ApplicationContextScope scope(application);
  Tree tree;
  DynamicTreeSkeleton physics;
  BaseSkeleton skeleton;
  auto& info = skeleton.RefNode(0).info;
  info.length = 1.0f;
  info.thickness = 0.05f;
  skeleton.SortLists();
  tree.shoot_model.Initialize(skeleton);
  ASSERT_TRUE(physics.InitializePhysics(tree));
  physics.physics_settings.gravity = {};
  physics.physics_settings.damping = 3.0f;
  physics.physics_settings.force_node = 0;
  physics.physics_settings.tip_force = {0, -20, 0};
  physics.physics_running = true;
  for (int i = 0; i < 120; ++i)
    physics.StepPhysics(tree);
  const auto& bent = tree.shoot_model.PeekPhysicsPose()[0];
  const auto bent_end = bent.position + bent.rotation * glm::vec3(0, 0, -1) * bent.length;
  const float bent_distance = glm::length(bent_end - glm::vec3(0, 0, -1));
  EXPECT_GT(bent_distance, 0.001f);
  physics.physics_settings.tip_force = {};
  for (int i = 0; i < 600; ++i)
    physics.StepPhysics(tree);
  const auto& relaxed = tree.shoot_model.PeekPhysicsPose()[0];
  const auto relaxed_end = relaxed.position + relaxed.rotation * glm::vec3(0, 0, -1) * relaxed.length;
  EXPECT_LT(glm::length(relaxed_end - glm::vec3(0, 0, -1)), bent_distance);
}

TEST(TreePhysics, LateralVisualOffsetFollowsBentJunction) {
  evo_engine::Application application;
  evo_engine::ApplicationContextScope scope(application);
  Tree tree;
  DynamicTreeSkeleton physics;
  auto skeleton = MakeFork();
  skeleton.RefNode(2).info.global_position += glm::vec3(0.05f, 0, 0);
  tree.shoot_model.Initialize(skeleton);
  ASSERT_TRUE(physics.InitializePhysics(tree));
  physics.physics_settings.gravity = {};
  physics.physics_settings.tip_force = {0, -40, 0};
  physics.physics_settings.force_node = 1;
  physics.physics_running = true;
  for (int i = 0; i < 120; ++i)
    physics.StepPhysics(tree);
  const auto& pose = tree.shoot_model.PeekPhysicsPose();
  const auto junction = pose[0].position + pose[0].rotation * glm::vec3(0, 0, -1) * pose[0].length;
  EXPECT_NEAR(glm::length(pose[2].position - junction), 0.05f, 1e-3f);
}

TEST(TreePhysics, ComponentSettingsRoundTripWithoutSimulationState) {
  DynamicTreeSkeleton source;
  source.physics_settings.time_step = 0.01f;
  source.physics_settings.bend_stiffness = 120.0f;
  source.physics_settings.gravity = {1, -2, 3};
  source.physics_settings.force_node = 7;
  source.physics_running = true;
  YAML::Emitter out;
  out << YAML::BeginMap;
  SerializeDynamicTreeSkeleton(out, source);
  out << YAML::EndMap;

  DynamicTreeSkeleton loaded;
  DeserializeDynamicTreeSkeleton(YAML::Load(out.c_str()), loaded);
  EXPECT_FLOAT_EQ(loaded.physics_settings.time_step, 0.01f);
  EXPECT_FLOAT_EQ(loaded.physics_settings.bend_stiffness, 120.0f);
  EXPECT_EQ(loaded.physics_settings.gravity, glm::vec3(1, -2, 3));
  EXPECT_EQ(loaded.physics_settings.force_node, 7);
  EXPECT_FALSE(loaded.physics_running);
}
