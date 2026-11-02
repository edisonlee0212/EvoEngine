#include <gtest/gtest.h>
#include "Application.hpp"
#include "ApplicationContext.hpp"
#include "EcoSysLabSerializationAdapters.hpp"
#include "EcoSysLabSystem.hpp"
#include "EvoEngine_SDK_PCH.hpp"
#include "Serialization.hpp"
#include "Soil.hpp"
#include "Tree.hpp"
using namespace eco_sys_lab_package;

TEST(EcoSysLabSystem, QueuedGrowthConsumesOneStepWithPartialFinalStep) {
  EcoSysLabSystem::Growth growth;
  growth.settings.delta_time = 30.0f;
  growth.Queue(75.0f);
  EXPECT_FLOAT_EQ(growth.TakeStep(), 30.0f);
  EXPECT_FLOAT_EQ(growth.TakeStep(), 30.0f);
  EXPECT_FLOAT_EQ(growth.TakeStep(), 15.0f);
  EXPECT_FLOAT_EQ(growth.TakeStep(), 0.0f);
  growth.Queue(10.0f);
  growth.Stop();
  EXPECT_FLOAT_EQ(growth.TakeStep(), 0.0f);
}

TEST(EcoSysLabSystem, DefaultSoilContainsWaterAndNutrientsBelowGround) {
  evo_engine::Application application;
  evo_engine::ApplicationContextScope scope(application);
  auto descriptor = std::make_shared<SoilDescriptor>();
  descriptor->soil_parameters.m_voxelResolution = {8, 8, 8};
  descriptor->soil_parameters.m_boundingBoxMin = {-0.4f, -0.7f, -0.4f};
  descriptor->texture_resolution = {4, 4};
  VoxelSoilModel model;
  BuildSoilModel(model, descriptor);
  EXPECT_GT(model.GetWaterDensity({0.0f, -0.2f, 0.0f}), 0.0f);
  EXPECT_GT(model.GetNutrientDensity({0.0f, -0.2f, 0.0f}), 0.0f);
}

TEST(EcoSysLabSystem, SceneConfigurationRoundTripsWithoutTransientGrowth) {
  evo_engine::Application application;
  evo_engine::ApplicationContextScope scope(application);
  EcoSysLabSystem source;
  source.environment.air_temperature_c = 32.0f;
  source.environment.air_cell_size = 0.05f;
  source.growth.settings.delta_time = 12.0f;
  source.growth.Queue(90.0f);
  source.dynamics.strands.enable_physics = false;
  source.dynamics.strands.physics_parameters.time_step = 0.025f;
  source.dynamics.strands.fungus_parameters.matrixAw[0][0] = 3.0f;
  source.dynamics.strands.segment_pairs_render_parameters.thickness_multiplier = 0.75f;
  source.dynamics.strands.remaining_step = 5;
  YAML::Emitter out;
  out << YAML::BeginMap;
  SerializeEcoSysLabSystem(out, source);
  out << YAML::EndMap;
  EcoSysLabSystem restored;
  DeserializeEcoSysLabSystem(YAML::Load(out.c_str()), restored);
  EXPECT_FLOAT_EQ(restored.environment.air_temperature_c, 32.0f);
  EXPECT_FLOAT_EQ(restored.environment.air_cell_size, 0.05f);
  EXPECT_FLOAT_EQ(restored.growth.settings.delta_time, 12.0f);
  EXPECT_FLOAT_EQ(restored.growth.remaining_time, 0.0f);
  EXPECT_FALSE(restored.dynamics.strands.enable_physics);
  EXPECT_FLOAT_EQ(restored.dynamics.strands.physics_parameters.time_step, 0.025f);
  EXPECT_FLOAT_EQ(restored.dynamics.strands.fungus_parameters.matrixAw[0][0], 3.0f);
  EXPECT_FLOAT_EQ(restored.dynamics.strands.segment_pairs_render_parameters.thickness_multiplier, 0.75f);
  EXPECT_EQ(restored.dynamics.strands.remaining_step, 0);
}
namespace {
std::string SaveTree(const Tree& tree) {
  YAML::Emitter out;
  out << YAML::BeginMap;
  SerializeTree(out, tree);
  out << YAML::EndMap;
  return out.c_str();
}
}  // namespace
TEST(TreeModelRevision, EditorPoseAndTopologyChangesInvalidateOnlyShootState) {
  Tree tree;
  const auto shoot = tree.GetShootModelRevision();
  const auto root = tree.GetRootModelRevision();
  tree.NotifyShootModelChanged(false);
  EXPECT_EQ(tree.GetShootModelRevision().content, shoot.content + 1);
  EXPECT_EQ(tree.GetShootModelRevision().topology, shoot.topology);
  tree.NotifyShootModelChanged();
  EXPECT_EQ(tree.GetShootModelRevision().content, shoot.content + 2);
  EXPECT_EQ(tree.GetShootModelRevision().topology, shoot.topology + 1);
  EXPECT_EQ(tree.GetRootModelRevision().content, root.content);
  EXPECT_EQ(tree.GetRootModelRevision().topology, root.topology);
}
TEST(TreeModelRevision, RootEditsInvalidateOnlyRootState) {
  Tree tree;
  const auto root = tree.GetRootModelRevision();
  const auto shoot = tree.GetShootModelRevision();
  tree.NotifyRootModelChanged(false);
  EXPECT_EQ(tree.GetRootModelRevision().content, root.content + 1);
  EXPECT_EQ(tree.GetRootModelRevision().topology, root.topology);
  tree.NotifyRootModelChanged();
  EXPECT_EQ(tree.GetRootModelRevision().content, root.content + 2);
  EXPECT_EQ(tree.GetRootModelRevision().topology, root.topology + 1);
  EXPECT_EQ(tree.GetShootModelRevision().content, shoot.content);
  EXPECT_EQ(tree.GetShootModelRevision().topology, shoot.topology);
}

TEST(TreeModelRevision, DeserializationInvalidatesDerivedStateWithoutChangingSavedFormat) {
  evo_engine::Application application;
  evo_engine::ApplicationContextScope scope(application);
  ASSERT_TRUE(evo_engine::Serialization::RegisterSerializationHandler<evo_engine::AssetRef>(
      [](YAML::Emitter& out, const evo_engine::AssetRef& reference) {
        reference.Serialize(out);
      },
      [](const YAML::Node& in, evo_engine::AssetRef& reference) {
        reference.Deserialize(in);
      },
      {}, "AssetRef"));
  Tree tree;
  const auto saved = SaveTree(tree);
  const auto shoot_before = tree.GetShootModelRevision();
  const auto root_before = tree.GetRootModelRevision();
  DeserializeTree(YAML::Load("{}"), tree);
  EXPECT_GT(tree.GetShootModelRevision().content, shoot_before.content);
  EXPECT_GT(tree.GetShootModelRevision().topology, shoot_before.topology);
  EXPECT_GT(tree.GetRootModelRevision().content, root_before.content);
  EXPECT_GT(tree.GetRootModelRevision().topology, root_before.topology);
  EXPECT_EQ(SaveTree(tree), saved);
  const auto revision = tree.GetShootModelRevision();
  DeserializeTree(YAML::Load("{}"), tree);
  EXPECT_GT(tree.GetShootModelRevision().content, revision.content);
  EXPECT_EQ(SaveTree(tree), saved);
}
