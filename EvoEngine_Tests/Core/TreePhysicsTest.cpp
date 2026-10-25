#include <gtest/gtest.h>
#include <glm/gtx/quaternion.hpp>
#include "Application.hpp"
#include "ApplicationContext.hpp"
#include "Blower.hpp"
#include "DynamicTreeSkeleton.hpp"
#include "EcoSysLabSerializationAdapters.hpp"
#include "EvoEngine_SDK_PCH.hpp"
#include "Serialization.hpp"
#include "Tree.hpp"

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

}  // namespace

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
  saved_physics.physics_running = true;
  YAML::Emitter physics_out;
  physics_out << YAML::BeginMap;
  SerializeDynamicTreeSkeleton(physics_out, saved_physics);
  physics_out << YAML::Key << "gpu_physics_enabled" << YAML::Value << false;
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
  ASSERT_EQ(state.node_rods.size(), 3u);
  for (uint32_t node = 0; node < state.node_rods.size(); ++node) {
    const auto& original = loaded_tree.shoot_model.PeekShootSkeleton().PeekNode(node).info;
    const auto rod = state.node_rods[node];
    ASSERT_NE(rod, UINT32_MAX);
    const auto vertex = state.rods[rod].links.x;
    EXPECT_LT(glm::length(glm::vec3(state.vertices[vertex].position_inverse_mass) - original.global_position), 1e-5f);
  }

  loaded_physics.ResetPhysics();
  loaded_physics.RestoreGpuPoseAfterLoad(loaded_tree);
  EXPECT_FALSE(loaded_physics.HasPhysics(loaded_tree));
}

TEST(TreePhysics, GpuGrowthReplacesDynamicRestAndResumesAfterward) {
  evo_engine::Application application;
  evo_engine::ApplicationContextScope scope(application);
  Tree tree;
  tree.shoot_model.Initialize(MakeFork());
  DynamicTreeSkeleton physics;
  ASSERT_TRUE(physics.SyncGpuPhysics(tree, GlobalTransform{}, false));
  EXPECT_TRUE(physics.physics_running);
  const auto original = physics.ExportGpuRestState();

  physics.ResetPhysics();
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
