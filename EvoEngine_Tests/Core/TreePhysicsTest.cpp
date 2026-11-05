#include <gtest/gtest.h>
#include <glm/gtx/quaternion.hpp>
#include "AirEnvironment.hpp"
#include "Application.hpp"
#include "ApplicationContext.hpp"
#include "ApplicationInitializationSettings.hpp"
#include "Blower.hpp"
#include "Climate.hpp"
#include "DynamicTreeSkeleton.hpp"
#include "EcoSysLabSerializationAdapters.hpp"
#include "EvoEngine_SDK_PCH.hpp"
#include "Serialization.hpp"
#include "SimulationSettings.hpp"
#include "Tree.hpp"

using namespace eco_sys_lab_package;

TEST(Blower, SettingsRoundTrip) {
  Blower source;
  source.speed = 100.0f;
  source.speed_noise = true;
  source.speed_max = 200.0f;
  source.speed_noise_frequency = 0.3f;
  source.particle_rate = 2400.0f;
  source.particle_radius = 0.35f;
  source.wind_strength = 30.0f;
  source.temperature_c = 32.0f;
  source.relative_humidity = 0.8f;
  source.spread_degrees = 15.0f;
  source.seed = 42;
  YAML::Emitter out;
  out << YAML::BeginMap;
  SerializeBlower(out, source);
  out << YAML::EndMap;

  Blower loaded;
  DeserializeBlower(YAML::Load(out.c_str()), loaded);
  EXPECT_FLOAT_EQ(loaded.speed, source.speed);
  EXPECT_EQ(loaded.speed_noise, source.speed_noise);
  EXPECT_FLOAT_EQ(loaded.speed_max, source.speed_max);
  EXPECT_FLOAT_EQ(loaded.speed_noise_frequency, source.speed_noise_frequency);
  EXPECT_FLOAT_EQ(loaded.particle_rate, source.particle_rate);
  EXPECT_FLOAT_EQ(loaded.particle_radius, source.particle_radius);
  EXPECT_FLOAT_EQ(loaded.wind_strength, source.wind_strength);
  EXPECT_FLOAT_EQ(loaded.temperature_c, source.temperature_c);
  EXPECT_FLOAT_EQ(loaded.relative_humidity, source.relative_humidity);
  EXPECT_FLOAT_EQ(loaded.spread_degrees, source.spread_degrees);
  EXPECT_EQ(loaded.seed, source.seed);
}

TEST(Blower, LegacyDensityMigratesToWindStrength) {
  Blower loaded;
  DeserializeBlower(YAML::Load("air_density: 2.45"), loaded);
  EXPECT_FLOAT_EQ(loaded.wind_strength, 2.0f);
}

TEST(AirEnvironment, SettingsRoundTrip) {
  AirEnvironment source;
  source.temperature_c = 35.0f;
  source.relative_humidity = 0.8f;
  source.pressure_pa = 93000.0f;
  source.coupling_rate = 6.0f;
  source.friction = 1.5f;
  YAML::Emitter out;
  out << YAML::BeginMap;
  SerializeAirEnvironment(out, source);
  out << YAML::EndMap;
  AirEnvironment loaded;
  DeserializeAirEnvironment(YAML::Load(out.c_str()), loaded);
  EXPECT_FLOAT_EQ(loaded.temperature_c, source.temperature_c);
  EXPECT_FLOAT_EQ(loaded.relative_humidity, source.relative_humidity);
  EXPECT_FLOAT_EQ(loaded.pressure_pa, source.pressure_pa);
  EXPECT_FLOAT_EQ(loaded.coupling_rate, source.coupling_rate);
  EXPECT_FLOAT_EQ(loaded.friction, source.friction);
  EXPECT_GT(loaded.SpecificHumidity(), AirEnvironment{}.SpecificHumidity());
}

TEST(Climate, AirSettingsLoadAndHumidity) {
  Climate loaded;
  DeserializeClimate(YAML::Load("air_temperature_c: 35\nair_relative_humidity: 0.8\n"
                                "air_pressure_pa: 93000\nair_coupling_rate: 6\nair_friction: 1.5\n"
                                "air_cell_size: 0.1"),
                     loaded);
  EXPECT_FLOAT_EQ(loaded.air_temperature_c, 35.0f);
  EXPECT_FLOAT_EQ(loaded.air_relative_humidity, 0.8f);
  EXPECT_FLOAT_EQ(loaded.air_pressure_pa, 93000.0f);
  EXPECT_FLOAT_EQ(loaded.air_coupling_rate, 6.0f);
  EXPECT_FLOAT_EQ(loaded.air_friction, 1.5f);
  EXPECT_FLOAT_EQ(loaded.air_cell_size, 0.1f);
  EXPECT_GT(loaded.AirSpecificHumidity(), Climate{}.AirSpecificHumidity());
  Climate legacy;
  DeserializeClimate(YAML::Load("air_temperature_c: 20"), legacy);
  EXPECT_FLOAT_EQ(legacy.air_cell_size, Climate::default_air_cell_size);
  EXPECT_FLOAT_EQ(legacy.air_cell_size, 0.1f);
  DeserializeClimate(YAML::Load("air_cell_size: 0.005"), legacy);
  EXPECT_FLOAT_EQ(legacy.air_cell_size, 0.05f);
  DeserializeClimate(YAML::Load("air_cell_size: 0.5"), legacy);
  EXPECT_FLOAT_EQ(legacy.air_cell_size, 0.2f);
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

TEST(TreePhysics, LocalCylinderMassUsesEachRodsGeometry) {
  evo_engine::Application application;
  evo_engine::ApplicationContextScope scope(application);
  Tree tree;
  tree.shoot_model.Initialize(MakeFork());
  DynamicTreeSkeleton physics;
  physics.physics_settings.use_local_rod_mass = true;
  ASSERT_TRUE(physics.InitializePhysics(tree));
  const auto state = physics.ExportGpuRestState();
  ASSERT_FALSE(state.rods.empty());
  for (const auto& rod : state.rods) {
    const auto& data = rod.length_scale_mass_radius;
    EXPECT_FLOAT_EQ(data.z, state.density * glm::pi<float>() * (data.w * data.w * data.x));
  }
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
  source.physics_settings.bend_thickness_exponent = 3.0f;
  source.physics_settings.bend_downstream_exponent = 1.5f;
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
  EXPECT_FLOAT_EQ(loaded.physics_settings.bend_thickness_exponent, 3.0f);
  EXPECT_FLOAT_EQ(loaded.physics_settings.bend_downstream_exponent, 1.5f);
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

TEST(TreePhysics, ClusterQualityRoundTripsIndependentlyFromPhysicalProperties) {
  DynamicTreeSkeleton source;
  source.physics_settings.use_subtree_clusters = true;
  source.physics_settings.cluster_lod.enabled_policies = 7;
  source.physics_settings.cluster_lod.rules[1] = {0.2f, 0.4f, 5, 6};
  source.physics_settings.bend_stiffness = 321;
  YAML::Emitter out;
  out << YAML::BeginMap;
  SerializeDynamicTreeSkeleton(out, source);
  out << YAML::EndMap;
  DynamicTreeSkeleton loaded;
  DeserializeDynamicTreeSkeleton(YAML::Load(out.c_str()), loaded);
  EXPECT_TRUE(loaded.physics_settings.use_subtree_clusters);
  EXPECT_EQ(loaded.physics_settings.cluster_lod.enabled_policies, 7u);
  EXPECT_EQ(loaded.physics_settings.cluster_lod.rules[1], source.physics_settings.cluster_lod.rules[1]);
  EXPECT_FLOAT_EQ(loaded.physics_settings.bend_stiffness, 321);
  DynamicTreeSkeleton legacy;
  DeserializeDynamicTreeSkeleton(YAML::Load("bend_stiffness: 123"), legacy);
  EXPECT_FALSE(legacy.physics_settings.use_subtree_clusters);
  EXPECT_FLOAT_EQ(legacy.physics_settings.bend_stiffness, 123);
}

TEST(TreeGrowthEnvironment, SingleVoxelLookupPreservesAccumulationAndTipOrder) {
  ShootModel model;
  auto& skeleton = model.RefShootSkeleton();
  for (int i = 0; i < 3; ++i)
    skeleton.Extend(0, true);
  skeleton.SortLists();
  for (const auto handle : skeleton.PeekSortedNodeList()) {
    auto& node = skeleton.RefNode(handle);
    node.info.global_position = {0.1f * handle, 0.1f, -0.1f};
    node.info.thickness = 0.01f * (handle + 1);
    node.data.shadow_size = 0.03f * (handle + 1);
  }
  ClimateModel actual, expected, staged;
  actual.environment_grid.voxel_grid.Initialize(1.0f, glm::vec3(-2), glm::vec3(2));
  staged.environment_grid.voxel_grid.Initialize(1.0f, glm::vec3(-2), glm::vec3(2));
  std::vector<ShootModel::VoxelRegistration> records;
  expected.environment_grid.voxel_grid.Initialize(1.0f, glm::vec3(-2), glm::vec3(2));
  const auto transform =
      glm::translate(glm::vec3(0.2f, 0.3f, -0.1f)) * glm::mat4_cast(glm::quat(glm::vec3(0, 0.2f, 0.1f)));
  for (uint32_t tree_id : {3u, 7u}) {
    skeleton.data.entity_index = tree_id;
    model.RegisterVoxel(transform, actual);
    model.CollectVoxelRegistrations(transform, records);
    ShootModel::RegisterVoxel(records, staged);
    const auto& nodes = skeleton.PeekSortedNodeList();
    for (auto it = nodes.rbegin(); it != nodes.rend(); ++it) {
      const auto& node = skeleton.PeekNode(*it);
      const glm::vec3 position = transform * glm::vec4(node.info.global_position, 1);
      expected.environment_grid.AddShadowValue(position, node.data.shadow_size);
      expected.environment_grid.AddBiomass(position, node.info.thickness);
      if (node.IsEndNode())
        expected.environment_grid.AddNode({position, *it, tree_id, node.info.thickness});
    }
  }
  for (int i = 0; i < actual.environment_grid.voxel_grid.GetVoxelCount(); ++i) {
    const auto& a = actual.environment_grid.voxel_grid.Peek(i);
    const auto& b = expected.environment_grid.voxel_grid.Peek(i);
    const auto& c = staged.environment_grid.voxel_grid.Peek(i);
    EXPECT_EQ(a.self_shadow, c.self_shadow);
    EXPECT_EQ(a.total_biomass, c.total_biomass);
    EXPECT_EQ(a.tree_owner, c.tree_owner);
    ASSERT_EQ(a.internode_voxel_registrations.size(), c.internode_voxel_registrations.size());
    for (size_t j = 0; j < a.internode_voxel_registrations.size(); ++j) {
      const auto& ar = a.internode_voxel_registrations[j];
      const auto& cr = c.internode_voxel_registrations[j];
      EXPECT_EQ(ar.position, cr.position);
      EXPECT_EQ(ar.node_handle, cr.node_handle);
      EXPECT_EQ(ar.tree_skeleton_index, cr.tree_skeleton_index);
      EXPECT_EQ(ar.thickness, cr.thickness);
    }
    EXPECT_EQ(a.self_shadow, b.self_shadow);
    EXPECT_EQ(a.total_biomass, b.total_biomass);
    EXPECT_EQ(a.tree_owner, b.tree_owner);
    EXPECT_EQ(a.light_intensity, b.light_intensity);
    ASSERT_EQ(a.internode_voxel_registrations.size(), b.internode_voxel_registrations.size());
    for (size_t j = 0; j < a.internode_voxel_registrations.size(); ++j) {
      const auto& ar = a.internode_voxel_registrations[j];
      const auto& br = b.internode_voxel_registrations[j];
      EXPECT_EQ(ar.position, br.position);
      EXPECT_EQ(ar.node_handle, br.node_handle);
      EXPECT_EQ(ar.tree_skeleton_index, br.tree_skeleton_index);
      EXPECT_EQ(ar.thickness, br.thickness);
    }
  }
}

TEST(TreeGrowthEnvironment, ForeignNodeQueryMatchesLegacyTraversal) {
  EnvironmentGrid grid;
  grid.voxel_grid.Initialize(0.2f, glm::vec3(-2.f), glm::vec3(2.f));
  const auto legacy = [&](const glm::vec3& position, float distance, unsigned tree) {
    bool found = false;
    grid.voxel_grid.PeekEach(position, distance * 2.f, [&](const EnvironmentVoxel& cell) {
      if (found)
        return;
      for (const auto& node : cell.internode_voxel_registrations) {
        if (node.tree_skeleton_index != tree && glm::distance(position, node.position) < distance)
          found = true;
      }
    });
    return found;
  };
  std::mt19937 rng(42);
  std::uniform_real_distribution<float> coordinate(-1.9f, 1.9f), query(-2.5f, 2.5f), radius(0.001f, 0.6f);
  for (int pass = 0; pass < 2; ++pass) {
    grid.voxel_grid.Reset();
    for (int i = 0; i < 2000; ++i) {
      const glm::vec3 position(coordinate(rng), coordinate(rng), coordinate(rng));
      grid.AddNode({position, i, static_cast<unsigned>(i % 5), 0.01f});
    }
    for (int i = 0; i < 5000; ++i) {
      const glm::vec3 position(query(rng), query(rng), query(rng));
      const float distance = radius(rng);
      const unsigned tree = i % 6;
      ASSERT_EQ(grid.HasForeignNodeWithin(position, distance, tree), legacy(position, distance, tree)) << i;
    }
    for (const float coordinate : {-2.01f, -2.f, -1.8f, 0.f, 0.2f, 1.8f, 2.f, 2.21f}) {
      const glm::vec3 position(coordinate, 0.f, 0.f);
      for (const float distance : {0.f, 0.15f, 0.2f})
        EXPECT_EQ(grid.HasForeignNodeWithin(position, distance, 0), legacy(position, distance, 0));
    }
  }
}

TEST(TreeGrowthEnvironment, OwnershipAndStrictDistanceSurviveResetAndMixedRegistration) {
  EnvironmentGrid grid;
  grid.voxel_grid.Initialize(0.2f, glm::vec3(-1.f), glm::vec3(1.f));
  const glm::vec3 point(0.f);
  EXPECT_FALSE(grid.HasForeignNodeWithin(point, 0.15f, 7));
  grid.AddNode({point, 0, 7, 0.01f});
  grid.AddNode({point, 1, 7, 0.01f});
  EXPECT_EQ(grid.voxel_grid.Peek(point).tree_owner, 7);
  EXPECT_FALSE(grid.HasForeignNodeWithin(point, 0.15f, 7));
  EXPECT_TRUE(grid.HasForeignNodeWithin(point, 0.15f, 8));
  grid.AddNode({point, 2, 8, 0.01f});
  EXPECT_EQ(grid.voxel_grid.Peek(point).tree_owner, -1);
  EXPECT_TRUE(grid.HasForeignNodeWithin(point, 0.15f, 7));
  grid.AddNode({point, 3, 7, 0.01f});
  EXPECT_EQ(grid.voxel_grid.Peek(point).tree_owner, -1);
  grid.voxel_grid.Reset();
  EXPECT_EQ(grid.voxel_grid.Peek(point).tree_owner, -1);
  EXPECT_FALSE(grid.HasForeignNodeWithin(point, 0.15f, 7));
  grid.AddNode({point, 4, std::numeric_limits<unsigned>::max(), 0.01f});
  EXPECT_FALSE(grid.HasForeignNodeWithin(point, 0.15f, std::numeric_limits<unsigned>::max()));
  const float threshold = 0.15f;
  EXPECT_FALSE(grid.HasForeignNodeWithin({threshold, 0, 0}, threshold, 7));
  EXPECT_TRUE(grid.HasForeignNodeWithin({std::nextafter(threshold, 0.f), 0, 0}, threshold, 7));
  EXPECT_FALSE(grid.HasForeignNodeWithin({std::nextafter(threshold, 1.f), 0, 0}, threshold, 7));
}

TEST(TreeGrowthEnvironment, TightQueryBoundsPreserveTranslatedGridBoundaryMatches) {
  std::mt19937 rng(123);
  std::uniform_real_distribution<float> coordinate(2.f, 26.f), component(-1.f, 1.f);
  for (float size : {0.01f, 0.2f, 1.f}) {
    for (float origin : {-4096.f, -4.f, 0.f, 4096.f}) {
      EnvironmentGrid grid;
      grid.voxel_grid.Initialize(size, glm::ivec3(32), glm::vec3(origin));
      for (int sample = 0; sample < 1000; ++sample) {
        const glm::vec3 cell(coordinate(rng), coordinate(rng), coordinate(rng));
        const glm::vec3 point = glm::vec3(origin) + (sample % 2 ? glm::floor(cell) : cell) * size;
        const float distance = size * (sample % 3 == 0 ? 0.01f : 0.75f);
        const auto direction = glm::normalize(glm::vec3(component(rng), component(rng), component(rng)));
        const glm::vec3 query = point + direction * distance;
        grid.AddNode({point, 0, 7, 0.01f});
        for (int axis = 0; axis < 3; ++axis) {
          for (float toward : {-std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity()}) {
            auto adjacent = query;
            adjacent[axis] = std::nextafter(adjacent[axis], toward);
            bool expected = false;
            grid.voxel_grid.PeekEach(adjacent, 2.f * distance, [&](const EnvironmentVoxel& voxel) {
              for (const auto& record : voxel.internode_voxel_registrations)
                expected |= glm::distance(adjacent, record.position) < distance;
            });
            ASSERT_EQ(grid.HasForeignNodeWithin(adjacent, distance, 8), expected)
                << size << ":" << origin << ":" << sample << ":" << axis;
          }
        }
        grid.voxel_grid.Ref(point) = EnvironmentVoxel{};
      }
    }
  }
}

namespace {
void LegacyLightPropagation(EnvironmentGrid& grid, const SimulationSettings& simulation_settings) {
  auto& voxel_grid = grid.voxel_grid;
  const auto voxel_size = grid.voxel_size;
  const auto resolution = voxel_grid.GetResolution();
  const int shadow_disk_size = glm::ceil(simulation_settings.detection_radius / voxel_size);
  Jobs::RunParallelFor(resolution.x * resolution.z, [&](size_t i) {
    const int x = i / resolution.z;
    const int z = i % resolution.z;
    auto& target_voxel = voxel_grid.Ref(glm::ivec3(x, resolution.y - 1, z));
    target_voxel.light_intensity = simulation_settings.skylight_intensity;
  });
  for (int y = resolution.y - 2; y >= 0; y--) {
    Jobs::RunParallelFor(resolution.x * resolution.z, [&](size_t i) {
      const int x = i / resolution.z;
      const int z = i % resolution.z;
      float sum = 0.0f;
      float max = 0.f;

      for (int x_offset = -shadow_disk_size; x_offset <= shadow_disk_size; x_offset++) {
        for (int z_offset = -shadow_disk_size; z_offset <= shadow_disk_size; z_offset++) {
          if (y + 1 > resolution.y - 1)
            continue;
          const auto other_voxel_center = glm::ivec3(x + x_offset, y + 1, z + z_offset);
          const auto position_diff =
              voxel_grid.GetPosition(other_voxel_center) - voxel_grid.GetPosition(glm::ivec3(x, y, z));
          const float distance = glm::length(position_diff);
          if (distance > simulation_settings.detection_radius)
            continue;
          const float base_loss_factor = voxel_size / distance;
          const float distance_loss =
              glm::pow(glm::max(0.0f, base_loss_factor), simulation_settings.shadow_distance_loss);
          if (x + x_offset < 0 || x + x_offset > resolution.x - 1 || z + z_offset < 0 ||
              z + z_offset > resolution.z - 1) {
            sum += distance_loss;
            max += distance_loss;
          } else {
            const auto& target_voxel = voxel_grid.Ref(other_voxel_center);
            sum += glm::max(target_voxel.light_intensity * distance_loss * (1.f - target_voxel.self_shadow), 0.0f);
            max += distance_loss;
          }
        }
      }
      auto& voxel = voxel_grid.Ref(glm::ivec3(x, y, z));
      voxel.light_intensity = glm::clamp(sum / max, 0.0f, 1.0f - simulation_settings.environment_light_intensity) +
                              simulation_settings.environment_light_intensity;
    });
  }
  for (int iteration = 0; iteration < simulation_settings.blur_iteration; iteration++) {
    for (int y = resolution.y - 2; y >= 0; y--) {
      Jobs::RunParallelFor(resolution.x * resolution.z, [&](size_t i) {
        const int x = i / resolution.z;
        const int z = i % resolution.z;

        const float self_intensity = voxel_grid.Ref(glm::ivec3(x, y, z)).light_intensity;
        float intensity = self_intensity * .4f;
        intensity += voxel_grid.Ref(glm::ivec3(x, y + 1, z)).light_intensity * .1f;
        if (y > 0)
          intensity += voxel_grid.Ref(glm::ivec3(x, y - 1, z)).light_intensity * .1f;
        else
          intensity += self_intensity * .1f;

        if (x > 0)
          intensity += voxel_grid.Ref(glm::ivec3(x - 1, y, z)).light_intensity * .1f;
        else
          intensity += self_intensity * .1f;

        if (z > 0)
          intensity += voxel_grid.Ref(glm::ivec3(x, y, z - 1)).light_intensity * .1f;
        else
          intensity += self_intensity * .1f;

        if (x < resolution.x - 1)
          intensity += voxel_grid.Ref(glm::ivec3(x + 1, y, z)).light_intensity * .1f;
        else
          intensity += self_intensity * .1f;

        if (z < resolution.z - 1)
          intensity += voxel_grid.Ref(glm::ivec3(x, y, z + 1)).light_intensity * .1f;
        else
          intensity += self_intensity * .1f;

        voxel_grid.Ref(glm::ivec3(x, y, z)).light_intensity = intensity;
      });
    }
  }
  const int light_space_size = glm::ceil(simulation_settings.detection_radius / voxel_size);
  for (int y = resolution.y - 1; y >= 0; y--) {
    Jobs::RunParallelFor(resolution.x * resolution.z, [&](size_t i) {
      const int x = i / resolution.z;
      const int z = i % resolution.z;
      glm::vec3 sum = glm::vec3(0.0f);
      for (int x_offset = -light_space_size; x_offset <= light_space_size; x_offset++) {
        for (int z_offset = -light_space_size; z_offset <= light_space_size; z_offset++) {
          for (int y_offset = 1; y_offset <= light_space_size; y_offset++) {
            if (y + y_offset < 0 || y + y_offset > resolution.y - 1)
              continue;
            const auto other_voxel_center = glm::ivec3(x + x_offset, y + y_offset, z + z_offset);
            const auto position_diff =
                voxel_grid.GetPosition(other_voxel_center) - voxel_grid.GetPosition(glm::ivec3(x, y, z));
            const float distance = glm::length(position_diff);
            if (distance > simulation_settings.detection_radius)
              continue;

            if (x + x_offset < 0 || x + x_offset > resolution.x - 1 || z + z_offset < 0 ||
                z + z_offset > resolution.z - 1) {
              sum += position_diff;
            } else {
              const auto& target_voxel = voxel_grid.Ref(other_voxel_center);
              sum += target_voxel.light_intensity * position_diff;
            }
          }
        }
      }
      auto& voxel = voxel_grid.Ref(glm::ivec3(x, y, z));
      if (glm::length(sum) > glm::epsilon<float>())
        voxel.light_direction = glm::normalize(sum);
      else
        voxel.light_direction = glm::vec3(0.0f, 1.0f, 0.0f);
    });
  }
}

}  // namespace

TEST(TreeGrowthEnvironment, FlatLightDirectionsMatchLayeredReference) {
  Application application;
  ApplicationContextScope scope(application);
  ApplicationInitializationSettings app_settings;
  app_settings.allow_empty_project = true;
  app_settings.load_default_resources = false;
  app_settings.load_project_assets = false;
  app_settings.load_project_start_scene = false;
  app_settings.enable_runtime_packages = false;
  application.Initialize(app_settings);
  std::mt19937 rng(19);
  std::uniform_real_distribution<float> shadow(0.f, 1.5f);
  for (const auto resolution : {glm::ivec3(1), glm::ivec3(4, 1, 3), glm::ivec3(7, 8, 6)}) {
    for (float origin : {-4096.f, -0.3f, 0.f, 4096.f}) {
      EnvironmentGrid actual;
      actual.voxel_size = 0.2f;
      actual.voxel_grid.Initialize(actual.voxel_size, resolution, glm::vec3(origin));
      for (auto& voxel : actual.voxel_grid.RefData())
        voxel.self_shadow = shadow(rng);
      auto expected = actual;
      SimulationSettings settings;
      settings.blur_iteration = 0;
      settings.detection_radius = 0.5f;
      LegacyLightPropagation(expected, settings);
      actual.LightPropagation(settings);
      for (int i = 0; i < actual.voxel_grid.GetVoxelCount(); ++i) {
        EXPECT_EQ(actual.voxel_grid.Peek(i).light_intensity, expected.voxel_grid.Peek(i).light_intensity);
        EXPECT_EQ(actual.voxel_grid.Peek(i).light_direction, expected.voxel_grid.Peek(i).light_direction);
      }
    }
  }
  application.Terminate();
}

TEST(TreePhysics, BendScalingChangesOnlyBendingAndReducesThicknessContrast) {
  evo_engine::Application application;
  evo_engine::ApplicationContextScope scope(application);
  for (const float thickness : {0.01f, 0.1f}) {
    Tree tree;
    auto skeleton = MakeFork();
    for (const auto handle : {0, 1, 2})
      skeleton.RefNode(handle).info.thickness = thickness;
    tree.shoot_model.Initialize(skeleton);
    DynamicTreeSkeleton physics;
    physics.physics_settings.bend_thickness_exponent = 4;
    physics.physics_settings.bend_downstream_exponent = 1.8f;
    ASSERT_TRUE(physics.InitializePhysics(tree));
    const auto original = physics.ExportGpuRestState();
    physics.physics_settings.bend_thickness_exponent = 2;
    physics.physics_settings.bend_downstream_exponent = 1;
    ASSERT_TRUE(physics.InitializePhysics(tree));
    const auto candidate = physics.ExportGpuRestState();
    ASSERT_EQ(original.rods.size(), candidate.rods.size());
    for (size_t i = 0; i < original.rods.size(); ++i) {
      EXPECT_EQ(original.rods[i].length_scale_mass_radius, candidate.rods[i].length_scale_mass_radius);
      EXPECT_EQ(original.rods[i].links, candidate.rods[i].links);
    }
    for (size_t i = 0; i < original.vertices.size(); ++i)
      EXPECT_EQ(original.vertices[i].position_inverse_mass, candidate.vertices[i].position_inverse_mass);
    // Incoming-edge-inclusive structural lengths: root 5.2, fork 4.2, each tip 0.8.
    const float downstream = (5.2f + 4.2f + 4.2f + 0.8f) * 0.25f;
    for (size_t i = 0; i < original.angles.size(); ++i) {
      const float expected = 4 * downstream * std::pow(thickness / 0.05f, 2.0f);
      EXPECT_NEAR(candidate.angles[i].bend_scale.x, expected, expected * 1e-5f);
      if (thickness < 0.05f)
        EXPECT_GT(candidate.angles[i].bend_scale.x, original.angles[i].bend_scale.x);
      else
        EXPECT_LT(candidate.angles[i].bend_scale.x, original.angles[i].bend_scale.x);
    }
  }
}

TEST(TreePhysics, BendScalingDefaultsAndPropertyEquality) {
  TreePhysicsProperties defaults, loaded;
  DeserializeTreePhysicsProperties(YAML::Load("bend_stiffness: 512"), loaded);
  EXPECT_FLOAT_EQ(loaded.bend_thickness_exponent, 3.5f);
  EXPECT_FLOAT_EQ(loaded.bend_downstream_exponent, 1.5f);
  EXPECT_TRUE(loaded == defaults);
  loaded.bend_thickness_exponent = 4;
  EXPECT_FALSE(loaded == defaults);
  loaded = defaults;
  loaded.bend_downstream_exponent = 1.8f;
  EXPECT_FALSE(loaded == defaults);
}

TEST(Blower, PerlinSpeedIsBoundedSmoothAndDeterministic) {
  Blower blower;
  blower.speed = 100;
  blower.speed_max = 200;
  EXPECT_FLOAT_EQ(blower.SampleSpeed(13), 100);
  blower.speed_noise = true;
  float previous = blower.SampleSpeed(0), minimum = previous, maximum = previous;
  for (int i = 0; i < 6000; ++i) {
    const float t = i * 0.016f;
    const float speed = blower.SampleSpeed(t);
    EXPECT_GE(speed, 100);
    EXPECT_LE(speed, 200);
    EXPECT_FLOAT_EQ(speed, blower.SampleSpeed(t));
    EXPECT_LT(std::abs(speed - previous), 2);
    minimum = std::min(minimum, speed);
    maximum = std::max(maximum, speed);
    previous = speed;
  }
  EXPECT_GT(maximum - minimum, 10);
  blower.speed_max = 50;
  EXPECT_FLOAT_EQ(blower.SampleSpeed(2), 100);
  Blower legacy;
  DeserializeBlower(YAML::Load("speed: 8"), legacy);
  EXPECT_FALSE(legacy.speed_noise);
  EXPECT_FLOAT_EQ(legacy.SampleSpeed(10), 8);
}
