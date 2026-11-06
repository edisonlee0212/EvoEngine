#include "EvoEngine_SDK_PCH.hpp"

#ifdef min
#  undef min
#endif
#ifdef max
#  undef max
#endif

#include <gtest/gtest.h>

#include "Application.hpp"
#include "ApplicationContext.hpp"
#include "ApplicationInitializationSettings.hpp"
#include "LSystemSerializationAdapters.hpp"
#include "SorghumGrowthModel.hpp"

using namespace evo_engine;
using namespace l_system_package;

namespace {
struct SorghumInitialLeafAzimuthTest : testing::Test {
  Application app;
  ApplicationContextScope scope{app};

  void SetUp() override {
    ApplicationInitializationSettings settings;
    settings.allow_empty_project = true;
    settings.load_default_resources = false;
    settings.load_project_assets = false;
    settings.load_project_start_scene = false;
    settings.enable_runtime_packages = false;
    app.Initialize(settings);
  }
};
}  // namespace

TEST_F(SorghumInitialLeafAzimuthTest, FixedPhasePreservesOtherRandomDrawsAndTraits) {
  SorghumLSDescriptor descriptor;
  descriptor.enable_panicle = false;
  descriptor.total_phytomer_count = {8.0f, 1.0f};
  descriptor.tiller_count = {2.0f, 0.5f};
  descriptor.plastochron_gdd = {30.0f};
  descriptor.maturity_gdd = {90.0f};
  ASSERT_TRUE(descriptor.randomize_initial_leaf_azimuth);
  for (const unsigned seed : {7101u, 7212u, 7155u}) {
    SorghumGrowthModel random, fixed;
    descriptor.randomize_initial_leaf_azimuth = true;
    random.Initialize(descriptor, seed);
    descriptor.randomize_initial_leaf_azimuth = false;
    descriptor.initial_leaf_azimuth_degrees = 0.0f;
    fixed.Initialize(descriptor, seed);
    EXPECT_FLOAT_EQ(fixed.initial_leaf_azimuth_degrees, 0.0f);
    const auto& original_apex = random.graph.PeekNode(1).data.Get<SorghumApex>();
    const auto& fixed_apex = fixed.graph.PeekNode(1).data.Get<SorghumApex>();
    EXPECT_FLOAT_EQ(random.initial_leaf_azimuth_degrees, std::fmod(original_apex.node_random * 360.0f, 360.0f));
    EXPECT_FLOAT_EQ(original_apex.node_random, fixed_apex.node_random);
    EXPECT_EQ(random.graph.data.main_internode_target_lengths, fixed.graph.data.main_internode_target_lengths);
    random.GrowToGDD(1000.0f, 0u);
    fixed.GrowToGDD(1000.0f, 0u);
    random.FinalizeSnapshotMorphology();
    fixed.FinalizeSnapshotMorphology();
    ASSERT_EQ(random.graph.PeekSortedNodeList(), fixed.graph.PeekSortedNodeList());
    int leaves = 0;
    for (const auto handle : random.graph.PeekSortedNodeList()) {
      const auto& a = random.graph.PeekNode(handle);
      const auto& b = fixed.graph.PeekNode(handle);
      ASSERT_EQ(a.symbol_id, b.symbol_id);
      if (!a.data.Is<SorghumLeaf>())
        continue;
      ++leaves;
      const auto& x = a.data.Get<SorghumLeaf>();
      const auto& y = b.data.Get<SorghumLeaf>();
      EXPECT_FLOAT_EQ(x.node_random, y.node_random);
      EXPECT_FLOAT_EQ(x.target_blade_length, y.target_blade_length);
      EXPECT_FLOAT_EQ(x.target_blade_max_width, y.target_blade_max_width);
      EXPECT_FLOAT_EQ(x.target_sheath_length, y.target_sheath_length);
      EXPECT_FLOAT_EQ(x.target_insertion_angle_deg, y.target_insertion_angle_deg);
      EXPECT_FLOAT_EQ(x.target_bending, y.target_bending);
      EXPECT_FLOAT_EQ(x.target_curling, y.target_curling);
      EXPECT_FLOAT_EQ(x.axial_twist_amplitude_deg, y.axial_twist_amplitude_deg);
    }
    EXPECT_GT(leaves, 0);
  }
}

TEST_F(SorghumInitialLeafAzimuthTest, DescriptorRoundTripAndOldFileDefault) {
  SorghumLSDescriptor source, restored;
  source.randomize_initial_leaf_azimuth = false;
  source.initial_leaf_azimuth_degrees = 75.0f;
  source.initial_leaf_azimuth_range_degrees = 90.0f;
  YAML::Emitter emitter;
  emitter << YAML::BeginMap;
  SerializeSorghumLSDescriptor(emitter, source);
  emitter << YAML::EndMap;
  auto yaml = YAML::Load(emitter.c_str());
  DeserializeSorghumLSDescriptor(yaml, restored);
  EXPECT_FALSE(restored.randomize_initial_leaf_azimuth);
  EXPECT_FLOAT_EQ(restored.initial_leaf_azimuth_degrees, 75.0f);
  EXPECT_FLOAT_EQ(restored.initial_leaf_azimuth_range_degrees, 90.0f);
  SorghumGrowthModel model;
  model.Initialize(restored, 7101u);
  EXPECT_FLOAT_EQ(model.initial_leaf_azimuth_degrees, 75.0f);
  yaml.remove("randomize_initial_leaf_azimuth");
  yaml.remove("initial_leaf_azimuth_degrees");
  yaml.remove("initial_leaf_azimuth_range_degrees");
  SorghumLSDescriptor legacy;
  DeserializeSorghumLSDescriptor(yaml, legacy);
  EXPECT_TRUE(legacy.randomize_initial_leaf_azimuth);
  EXPECT_FLOAT_EQ(legacy.initial_leaf_azimuth_range_degrees, 360.0f);
}

TEST_F(SorghumInitialLeafAzimuthTest, BoundedArcPreservesRandomDrawAndWrapsThroughZero) {
  for (const float width : {0.0f, 30.0f, 90.0f, 180.0f, 360.0f}) {
    for (const unsigned seed : {7101u, 7110u, 7155u, 7362u}) {
      SorghumLSDescriptor descriptor;
      SorghumGrowthModel original, bounded;
      original.Initialize(descriptor, seed);
      descriptor.initial_leaf_azimuth_degrees = 360.0f - width / 2.0f;
      descriptor.initial_leaf_azimuth_range_degrees = width;
      bounded.Initialize(descriptor, seed);
      const auto& a = original.graph.PeekNode(1).data.Get<SorghumApex>();
      const auto& b = bounded.graph.PeekNode(1).data.Get<SorghumApex>();
      EXPECT_FLOAT_EQ(a.node_random, b.node_random);
      EXPECT_FLOAT_EQ(original.initial_leaf_azimuth_degrees, std::fmod(a.node_random * 360.0f, 360.0f));
      EXPECT_FLOAT_EQ(bounded.initial_leaf_azimuth_degrees,
                      std::fmod(descriptor.initial_leaf_azimuth_degrees + a.node_random * width, 360.0f));
      const float signed_phase = std::fmod(bounded.initial_leaf_azimuth_degrees + 180.0f, 360.0f) - 180.0f;
      EXPECT_LE(std::abs(signed_phase), width / 2.0f + 0.0001f);
      EXPECT_EQ(original.graph.data.main_internode_target_lengths, bounded.graph.data.main_internode_target_lengths);
    }
  }
}
