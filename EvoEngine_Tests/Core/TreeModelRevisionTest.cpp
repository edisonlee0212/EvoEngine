#include <gtest/gtest.h>
#include "Application.hpp"
#include "ApplicationContext.hpp"
#include "BasicPruningDescriptor.hpp"
#include "BasicShootDescriptor.hpp"
#include "BranchRenderer.hpp"
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
  source.dynamics.use_subtree_clusters = true;
  source.dynamics.cluster_lod.enabled_policies = 5;
  source.dynamics.cluster_lod.rules[2] = {0.4f, 0.3f, 4, 7};
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
  EXPECT_TRUE(restored.dynamics.use_subtree_clusters);
  EXPECT_EQ(restored.dynamics.cluster_lod.enabled_policies, 5u);
  EXPECT_EQ(restored.dynamics.cluster_lod.rules[2], source.dynamics.cluster_lod.rules[2]);
  EXPECT_FALSE(restored.dynamics.cluster_forest);
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

TEST(EcoSysLabSystem, RenderingSettingsRoundTripAndLegacyDefaults) {
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
  EcoSysLabSystem source;
  source.rendering.screen_space_tessellation = false;
  source.rendering.camera_impostors = false;
  source.rendering.canopy_shadows = false;
  source.rendering.canopy_shadow_texels = 128;
  source.rendering.impostor_diameter_pixels = 4;
  source.rendering.pixel_error = 1.25f;
  source.rendering.segment_length_pixels = 24;
  YAML::Emitter out;
  out << YAML::BeginMap;
  SerializeEcoSysLabSystem(out, source);
  out << YAML::EndMap;
  EcoSysLabSystem restored;
  DeserializeEcoSysLabSystem(YAML::Load(out.c_str()), restored);
  EXPECT_FALSE(restored.rendering.screen_space_tessellation);
  EXPECT_FALSE(restored.rendering.camera_impostors);
  EXPECT_FALSE(restored.rendering.canopy_shadows);
  EXPECT_FLOAT_EQ(restored.rendering.canopy_shadow_texels, 128);
  EXPECT_FLOAT_EQ(restored.rendering.impostor_diameter_pixels, 4);
  EXPECT_FLOAT_EQ(restored.rendering.pixel_error, 1.25f);
  EXPECT_FLOAT_EQ(restored.rendering.segment_length_pixels, 24);
  EcoSysLabSystem legacy;
  DeserializeEcoSysLabSystem(YAML::Load("{}"), legacy);
  EXPECT_TRUE(legacy.rendering.screen_space_tessellation);
  EXPECT_TRUE(legacy.rendering.camera_impostors);
  EXPECT_TRUE(legacy.rendering.canopy_shadows);
  EXPECT_FLOAT_EQ(legacy.rendering.canopy_shadow_texels, 16);
  EXPECT_FLOAT_EQ(legacy.rendering.impostor_diameter_pixels, 3);
  EXPECT_FLOAT_EQ(legacy.rendering.pixel_error, 0.5f);
  EXPECT_FLOAT_EQ(legacy.rendering.segment_length_pixels, 12);
}

TEST(ShootGrowthData, FinalCountsTrackOrgansAndPrunedSubtrees) {
  evo_engine::Application application;
  evo_engine::ApplicationContextScope scope(application);
  BasicShootDescriptor descriptor;
  BasicPruningDescriptor pruning_descriptor;
  ShootGrowthController growth;
  ShootPruningController pruning;
  descriptor.PrepareController(growth);
  SimulationSettings settings;
  pruning_descriptor.PrepareController(settings, pruning);
  FoliageController foliage;
  const auto grow_organ = [](auto&, const auto&, float, auto&, const auto&, const auto&, const auto&) {
    return false;
  };
  foliage.leaf_growth = grow_organ;
  ShootReproductionController reproduction;
  reproduction.flower_growth = grow_organ;
  reproduction.fruit_growth = grow_organ;
  ClimateModel climate;
  VoxelSoilModel soil;
  for (const bool prune : {false, true}) {
    ShootModel model;
    model.Initialize(growth, foliage, reproduction);
    auto& skeleton = model.RefShootSkeleton();
    const auto child = skeleton.Extend(0, false);
    skeleton.SortLists();
    for (const auto handle : skeleton.PeekSortedNodeList()) {
      auto& node = skeleton.RefNode(handle);
      node.data.buds.clear();
      node.data.internode_length = 1.f;
      node.data.internode_thickness = 1.f;
      node.data.leaves.resize(3);
      node.data.leaves[0].status = OrganStatus::Dormant;
      node.data.leaves[1].status = OrganStatus::Flushed;
      node.data.flowers.resize(1);
      node.data.flowers[0].status = OrganStatus::Dormant;
      node.data.fruits.resize(1);
      node.data.fruits[0].status = OrganStatus::Dormant;
    }
    pruning.base_to_end_pruning_factor = [child](auto&, const auto&, const auto&, const auto&, const auto&,
                                                 const auto& node) {
      return node.GetHandle() == child ? 1000.f : 0.f;
    };
    model.Grow(1.f, 0, glm::mat4(1.f), climate, soil, growth, foliage, reproduction, pruning, prune);
    const int nodes = prune ? 1 : 2;
    EXPECT_EQ(skeleton.PeekSortedNodeList().size(), nodes);
    EXPECT_EQ(model.GetLeafCount(), 2 * nodes);
    EXPECT_EQ(model.GetFlowerCount(), nodes);
    EXPECT_EQ(model.GetFruitCount(), nodes);
    EXPECT_EQ(std::accumulate(model.internode_order_counts.begin(), model.internode_order_counts.end(), 0), nodes);
  }
}

TEST(PlantTropism, DirectRotationMatchesOriginalFormula) {
  std::mt19937 rng(42);
  std::uniform_real_distribution<float> component(-1.f, 1.f), strength(-0.1f, 1.2f);
  const auto reference = [](const glm::vec3& target, float tropism, glm::vec3& front, glm::vec3& up) {
    const auto dir = glm::normalize(target);
    if (const float dot = glm::abs(glm::dot(front, dir)); dot < 0.99f && dot > -0.99f) {
      const auto left = glm::cross(front, dir);
      const float max_angle = glm::acos(dot);
      front = glm::normalize(glm::rotate(front, glm::min(max_angle, max_angle * tropism), left));
      up = glm::normalize(glm::cross(glm::cross(front, up), front));
    }
  };
  for (int sample = 0; sample < 20000; ++sample) {
    const glm::vec3 target(component(rng), component(rng), component(rng));
    auto front = glm::normalize(glm::vec3(component(rng), component(rng), component(rng)));
    auto up = glm::normalize(glm::cross(front, glm::vec3(component(rng), component(rng), component(rng))));
    auto expected_front = front, expected_up = up;
    const float tropism = sample % 4 == 0 ? 0.f : strength(rng);
    for (int pass = 0; pass < 3; ++pass) {
      reference(target, tropism, expected_front, expected_up);
      PlantModel::ApplyTropism(target, tropism, front, up);
      ASSERT_EQ(front, expected_front) << sample << ":" << pass;
      ASSERT_EQ(up, expected_up) << sample << ":" << pass;
    }
  }
  for (const float dot : {0.f, 0.98999f, 0.99f, 0.99001f, 1.f, -1.f}) {
    for (const float tropism : {-0.1f, 0.f, 0.5f, 1.f, 2.f}) {
      const glm::vec3 target(0, 1, 0);
      glm::vec3 front(glm::sqrt(1.f - dot * dot), dot, 0), up(0, 0, 1);
      auto expected_front = front, expected_up = up;
      reference(target, tropism, expected_front, expected_up);
      PlantModel::ApplyTropism(target, tropism, front, up);
      EXPECT_EQ(front, expected_front);
      EXPECT_EQ(up, expected_up);
    }
  }
}
