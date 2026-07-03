#include "EvoEngine_SDK_PCH.hpp"

#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif

#include <gtest/gtest.h>

#include "GeometryPass.hpp"
#include "SorghumLeafMesh.hpp"
#include "SorghumModules.hpp"
#include "SorghumRules.hpp"

#include <array>

using namespace l_system_package;

TEST(LSystemLeafAtlasUv, DefaultLayoutPreservesLocalUv) {
  const SorghumLeafAtlasLayout layout;
  const glm::vec2 uv(0.25f, 0.75f);

  const auto remapped = RemapSorghumLeafAtlasUv(uv, layout);

  EXPECT_FLOAT_EQ(remapped.x, uv.x);
  EXPECT_FLOAT_EQ(remapped.y, uv.y);
}

TEST(LSystemLeafAtlasUv, MapsFirstVariantToTopLeftTile) {
  SorghumLeafAtlasLayout layout;
  layout.variant_columns = 3u;
  layout.variant_rows = 3u;
  layout.variant_count = 9u;
  layout.variant_index = 0u;
  layout.tile_uv_inset = 0.001f;

  const auto lower_left = RemapSorghumLeafAtlasUv(glm::vec2(0.0f, 0.0f), layout);
  const auto upper_right = RemapSorghumLeafAtlasUv(glm::vec2(1.0f, 1.0f), layout);

  EXPECT_NEAR(lower_left.x, 0.001f, 1.0e-6f);
  EXPECT_NEAR(lower_left.y, 2.0f / 3.0f + 0.001f, 1.0e-6f);
  EXPECT_NEAR(upper_right.x, 1.0f / 3.0f - 0.001f, 1.0e-6f);
  EXPECT_NEAR(upper_right.y, 1.0f - 0.001f, 1.0e-6f);
}

TEST(LSystemLeafAtlasUv, MapsLastVariantToBottomRightTile) {
  SorghumLeafAtlasLayout layout;
  layout.variant_columns = 3u;
  layout.variant_rows = 3u;
  layout.variant_count = 9u;
  layout.variant_index = 8u;
  layout.tile_uv_inset = 0.001f;

  const auto lower_left = RemapSorghumLeafAtlasUv(glm::vec2(0.0f, 0.0f), layout);
  const auto upper_right = RemapSorghumLeafAtlasUv(glm::vec2(1.0f, 1.0f), layout);

  EXPECT_NEAR(lower_left.x, 2.0f / 3.0f + 0.001f, 1.0e-6f);
  EXPECT_NEAR(lower_left.y, 0.001f, 1.0e-6f);
  EXPECT_NEAR(upper_right.x, 1.0f - 0.001f, 1.0e-6f);
  EXPECT_NEAR(upper_right.y, 1.0f / 3.0f - 0.001f, 1.0e-6f);
}

TEST(LSystemLeafAtlasUv, SingleVariantAlwaysSelectsFirstTile) {
  EXPECT_EQ(ComputeSorghumLeafAtlasVariant(37u, 12u, 0.0f, 1u), 0u);
  EXPECT_EQ(ComputeSorghumLeafAtlasVariant(37u, 12u, 1.0f, 1u), 0u);
}

TEST(LSystemLeafAtlasUv, LeafRandomChangesVariantSelection) {
  std::array<bool, 9> seen{};
  for (uint32_t i = 0; i < 100u; ++i) {
    const float leaf_random = (static_cast<float>(i) + 0.5f) / 100.0f;
    seen[ComputeSorghumLeafAtlasVariant(37u, 12u, leaf_random, 9u)] = true;
  }

  size_t seen_count = 0;
  for (const bool selected : seen) {
    if (selected) {
      ++seen_count;
    }
  }
  EXPECT_GT(seen_count, 1u);
}

TEST(LSystemLeafAtlasUv, ThreeByThreeAtlasVariantsAreReachablePerLeaf) {
  std::array<bool, 9> seen{};
  for (uint32_t i = 0; i < 256u; ++i) {
    const float leaf_random = (static_cast<float>((i * 37u) % 256u) + 0.5f) / 256.0f;
    seen[ComputeSorghumLeafAtlasVariant(1234u, i, leaf_random, 9u)] = true;
  }

  for (const bool selected : seen) {
    EXPECT_TRUE(selected);
  }
}

TEST(LSystemSorghumTillers, RootChildrenStartAtRootAndNonRootChildrenUseParentTip) {
  SorghumGraph graph(1);

  auto& root = graph.RefNode(0);
  root.symbol_id = SorghumSymbol::Root;
  root.data.Set<SorghumRoot>(SorghumRoot{});
  root.info.length = 0.5f;

  SorghumInternode internode;
  internode.order = 0;
  internode.rank = 0;
  internode.length = 0.5f;

  const auto bud_handle = graph.Extend(0, true);
  graph.RefNode(bud_handle).symbol_id = SorghumSymbol::TillerBud;
  graph.RefNode(bud_handle).data.Set<SorghumTillerBud>(SorghumTillerBud{});

  const auto activated_apex_handle = graph.Extend(0, true);
  SorghumApex activated_apex;
  activated_apex.order = 1;
  activated_apex.phytomer_count = 0;
  graph.RefNode(activated_apex_handle).symbol_id = SorghumSymbol::Apex;
  graph.RefNode(activated_apex_handle).data.Set<SorghumApex>(activated_apex);

  const auto internode_handle = graph.Extend(0, true);
  graph.RefNode(internode_handle).symbol_id = SorghumSymbol::Internode;
  graph.RefNode(internode_handle).data.Set<SorghumInternode>(internode);
  graph.RefNode(internode_handle).info.length = internode.length;

  const auto leaf_handle = graph.Extend(internode_handle, true);
  graph.RefNode(leaf_handle).symbol_id = SorghumSymbol::Leaf;
  graph.RefNode(leaf_handle).data.Set<SorghumLeaf>(SorghumLeaf{});

  graph.SortLists();
  GeometryPass::Execute(
      graph, glm::vec3(0.0f), kDefaultRootRotation,
      std::function<glm::quat(const SorghumNode&, const SorghumNode&)>(),
      std::function<glm::vec3(const SorghumNode&, const SorghumNode&)>(
          ComputeSorghumChildGlobalPosition));

  const auto& propagated_root = graph.PeekNode(0);
  const auto& propagated_internode = graph.PeekNode(internode_handle);
  const glm::vec3 root_base = propagated_root.info.global_position;
  const glm::vec3 root_tip = propagated_root.info.GetGlobalEndPosition();
  const glm::vec3 internode_tip = propagated_internode.info.GetGlobalEndPosition();

  const auto expect_near = [](const glm::vec3& actual, const glm::vec3& expected) {
    EXPECT_NEAR(actual.x, expected.x, 1.0e-5f);
    EXPECT_NEAR(actual.y, expected.y, 1.0e-5f);
    EXPECT_NEAR(actual.z, expected.z, 1.0e-5f);
  };

  EXPECT_EQ(graph.PeekNode(bud_handle).GetParentHandle(), 0);
  EXPECT_EQ(graph.PeekNode(activated_apex_handle).GetParentHandle(), 0);
  EXPECT_EQ(graph.PeekNode(internode_handle).GetParentHandle(), 0);
  EXPECT_EQ(graph.PeekNode(leaf_handle).GetParentHandle(), internode_handle);

  expect_near(graph.PeekNode(bud_handle).info.global_position, root_base);
  expect_near(graph.PeekNode(activated_apex_handle).info.global_position, root_base);
  expect_near(graph.PeekNode(internode_handle).info.global_position, root_base);
  expect_near(graph.PeekNode(leaf_handle).info.global_position, internode_tip);
  EXPECT_GT(glm::distance(root_tip, root_base), 0.1f);
}

TEST(LSystemSorghumTillers, TillerBudAzimuthsFollowGoldenAnglePhyllotaxis) {
  EXPECT_NEAR(ComputeSorghumTillerBudAzimuth(0, 10.0f), 10.0f, 1.0e-5f);
  EXPECT_NEAR(ComputeSorghumTillerBudAzimuth(1, 10.0f), 147.5f, 1.0e-5f);
  EXPECT_NEAR(ComputeSorghumTillerBudAzimuth(2, 10.0f), 285.0f, 1.0e-5f);
  EXPECT_NEAR(ComputeSorghumTillerBudAzimuth(3, 10.0f), 62.5f, 1.0e-5f);
}

TEST(LSystemSorghumTillers, TillerBranchAnglesRecoverUprightOverTwoRanks) {
  constexpr float kInsertionAngle = 40.0f;

  EXPECT_FLOAT_EQ(ComputeSorghumTillerInternodeBranchAngle(0, 5, kInsertionAngle), 40.0f);
  EXPECT_FLOAT_EQ(ComputeSorghumTillerInternodeBranchAngle(1, 5, kInsertionAngle), -20.0f);
  EXPECT_FLOAT_EQ(ComputeSorghumTillerInternodeBranchAngle(2, 5, kInsertionAngle), -20.0f);
  EXPECT_FLOAT_EQ(ComputeSorghumTillerInternodeBranchAngle(3, 5, kInsertionAngle), 0.0f);
}

TEST(LSystemSorghumTillers, TwoPhytomerTillersRecoverOnSecondRank) {
  constexpr float kInsertionAngle = 40.0f;

  EXPECT_FLOAT_EQ(ComputeSorghumTillerInternodeBranchAngle(0, 2, kInsertionAngle), 40.0f);
  EXPECT_FLOAT_EQ(ComputeSorghumTillerInternodeBranchAngle(1, 2, kInsertionAngle), -40.0f);
  EXPECT_FLOAT_EQ(ComputeSorghumTillerInternodeBranchAngle(2, 2, kInsertionAngle), 0.0f);
}

TEST(LSystemSorghumTillers, TillerBranchAnglesKeepMinimumDepartureAngle) {
  EXPECT_FLOAT_EQ(ComputeSorghumTillerInternodeBranchAngle(0, 4, 10.0f), 25.0f);
  EXPECT_FLOAT_EQ(ComputeSorghumTillerInternodeBranchAngle(1, 4, 10.0f), -12.5f);
  EXPECT_FLOAT_EQ(ComputeSorghumTillerInternodeBranchAngle(2, 4, 10.0f), -12.5f);
}
