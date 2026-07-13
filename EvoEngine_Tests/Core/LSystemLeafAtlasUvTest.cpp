#include "EvoEngine_SDK_PCH.hpp"

#ifdef min
#  undef min
#endif
#ifdef max
#  undef max
#endif

#include <gtest/gtest.h>

#include "DistributionDefaults.hpp"
#include "GeometryPass.hpp"
#include "SorghumLeafMesh.hpp"
#include "SorghumModules.hpp"
#include "SorghumRules.hpp"

#include <array>
#include <map>
#include <set>

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

TEST(LSystemLeafMorphology, SpatialBendingProfileIsIndependentFromTemporalDevelopment) {
  SampledSorghumParams params;
  ApplyLinearGrowthCurveDefaultsToAll(params.width_along_sheath, params.width_along_neck, params.width_along_leaf,
                                      params.curling_along_leaf, params.waviness_along_leaf,
                                      params.leaf_bending_development_curve, params.bending_along_leaf);

  SorghumLeaf leaf;
  leaf.blade_length = leaf.target_blade_length = 1.0f;
  leaf.neck_length = leaf.target_neck_length = 0.0f;
  leaf.sheath_length = leaf.target_sheath_length = 0.0f;
  leaf.sheath_end_width_ratio = leaf.target_sheath_end_width_ratio = 1.0f;
  leaf.neck_end_width_ratio = leaf.target_neck_end_width_ratio = 1.0f;
  leaf.blade_end_width_ratio = leaf.target_blade_end_width_ratio = 0.05f;
  leaf.bending = leaf.target_bending = 60.0f;
  leaf.curling = leaf.target_curling = 89.0f;
  leaf.insertion_angle_deg = leaf.target_insertion_angle_deg = 0.0f;
  leaf.node_random = 0.5f;

  StemContext stem;
  stem.segments.emplace_back();
  SorghumLeafMeshSettings settings;
  settings.vertical_subdivision_length = 0.05f;

  SorghumSpline first;
  BuildLeafSplineFromState(leaf, stem, params, settings, first);
  ASSERT_FALSE(first.segments.empty());
  const glm::vec3 first_tip = first.segments.back().position;

  params.leaf_bending_development_curve.mean.curve.UnsafeGetValues() = {{0.0f, 0.0f}, {1.0f, 0.0f}};
  SorghumSpline temporal_changed;
  BuildLeafSplineFromState(leaf, stem, params, settings, temporal_changed);
  ASSERT_FALSE(temporal_changed.segments.empty());
  const glm::vec3 temporal_tip = temporal_changed.segments.back().position;
  EXPECT_NEAR(glm::distance(first_tip, temporal_tip), 0.0f, 1.0e-6f);

  params.bending_along_leaf.mean.curve.UnsafeGetValues() = {{0.0f, 0.0f}, {1.0f, 0.0f}};
  SorghumSpline spatial_changed;
  BuildLeafSplineFromState(leaf, stem, params, settings, spatial_changed);
  ASSERT_FALSE(spatial_changed.segments.empty());
  EXPECT_GT(glm::distance(first_tip, spatial_changed.segments.back().position), 0.1f);
}

TEST(LSystemLeafMorphology, AbsoluteBladeWidthIsIndependentOfHostCulmRadius) {
  SampledSorghumParams params;
  ApplyLinearGrowthCurveDefaultsToAll(params.width_along_sheath, params.width_along_neck, params.width_along_leaf,
                                      params.curling_along_leaf, params.waviness_along_leaf, params.bending_along_leaf);
  params.width_along_leaf.mean.curve.UnsafeGetValues() = {{0.0f, 0.35f}, {0.5f, 1.0f}, {1.0f, 0.0f}};
  params.leaf_sheath_radius_ratio = 1.05f;
  params.leaf_sheath_wrap_angle = 390.0f;

  SorghumLeaf leaf;
  leaf.blade_length = leaf.target_blade_length = 1.0f;
  leaf.blade_max_width = leaf.target_blade_max_width = 0.12f;
  leaf.neck_length = leaf.target_neck_length = 0.1f;
  leaf.sheath_length = leaf.target_sheath_length = 0.2f;
  leaf.curling = leaf.target_curling = 89.0f;
  leaf.node_random = 0.5f;

  SorghumLeafMeshSettings settings;
  settings.vertical_subdivision_length = 0.01f;
  const auto distal_max_radius = [&](const float culm_radius) {
    StemContext stem;
    StemContext::Segment base;
    base.radius = culm_radius;
    stem.segments.push_back(base);
    auto tip = base;
    tip.position = glm::vec3(0.0f, 1.0f, 0.0f);
    stem.segments.push_back(tip);
    SorghumSpline spline;
    BuildLeafSplineFromState(leaf, stem, params, settings, spline);
    float maximum = 0.0f;
    for (const auto& segment : spline.segments) {
      if (segment.theta <= 90.0f)
        maximum = std::max(maximum, segment.radius);
    }
    return maximum;
  };

  const float narrow_culm_width = distal_max_radius(0.005f);
  const float wide_culm_width = distal_max_radius(0.015f);
  EXPECT_NEAR(narrow_culm_width, wide_culm_width, 1.0e-6f);
  EXPECT_NEAR(narrow_culm_width, 0.06f, 5.0e-4f);
}

TEST(LSystemSorghumTillers, MatureGraphHasThreeCrownAttachedPrimaryTillers) {
  SampledSorghumParams params;
  params.total_phytomer_count = 10;
  params.phyllotaxis_angle = 180.0f;
  params.tiller_count = 3;
  params.tiller_origin_ranks = {3, 4, 2};
  params.tiller_emergence_main_leaf_stages = {1, 1, 1, 1, 1, 1};
  params.tiller_leaf_count_ratio = {0.90f, 0.0f};
  params.tiller_height_ratio = {0.90f, 0.0f};
  params.plastochron_gdd = 1.0f;
  params.maturity_gdd = 1.0f;
  params.gdd_step = 1.0f;
  ApplyMeanStdPlotDefaultsToAll(params.internode_length, params.internode_thickness, params.leaf_blade_length,
                                params.leaf_blade_max_width, params.leaf_sheath_length, params.leaf_neck_length,
                                params.leaf_sheath_end_width_ratio, params.leaf_neck_end_width_ratio,
                                params.leaf_blade_end_width_ratio, params.leaf_insertion_angle, params.leaf_roll_angle,
                                params.leaf_curling, params.leaf_bending, params.leaf_waviness,
                                params.tiller_leaf_area_ratio_by_origin);
  ApplyLinearGrowthCurveDefaultsToAll(
      params.internode_elongation_curve, params.internode_thickness_curve, params.leaf_sheath_length_growth_curve,
      params.leaf_neck_length_growth_curve, params.leaf_blade_growth_curve, params.leaf_sheath_width_growth_curve,
      params.leaf_neck_width_growth_curve, params.leaf_width_growth_curve, params.leaf_angle_development_curve,
      params.leaf_curling_development_curve, params.leaf_bending_development_curve, params.bending_along_leaf);

  SorghumGraph graph(1);
  graph.RefNode(0).symbol_id = SorghumSymbol::Root;
  graph.RefNode(0).data.Set<SorghumRoot>(SorghumRoot{});
  const auto apex_handle = graph.Extend(0, false);
  SorghumApex apex;
  apex.vigor = params.total_phytomer_count;
  apex.sampled_plastochron_gdd = 1.0f;
  apex.age_gdd = 1.0f;
  apex.reference_main_phytomer_count = params.total_phytomer_count;
  apex.axis_phytomer_count = params.total_phytomer_count;
  graph.RefNode(apex_handle).symbol_id = SorghumSymbol::Apex;
  graph.RefNode(apex_handle).data.Set<SorghumApex>(apex);
  graph.data.main_expanded_leaf_count = params.total_phytomer_count;
  graph.SortLists();

  SorghumEngine engine;
  engine.topology_rules = CreateSorghumTopologyRules(params);
  engine.growth_rules = CreateSorghumGrowthRules(params);
  std::mt19937 rng(17u);
  for (int step = 0; step < 64; ++step) {
    engine.ApplyGrowthRules(graph, rng);
    engine.ApplyTopologyRules(graph, rng);
    graph.SortLists();
  }

  std::set<int> tiller_axes;
  std::map<int, int> leaves_per_axis;
  int main_leaves = 0;
  for (const auto handle : graph.PeekSortedNodeList()) {
    const auto& node = graph.PeekNode(handle);
    if (node.data.Is<SorghumLeaf>()) {
      const auto& leaf = node.data.Get<SorghumLeaf>();
      if (leaf.order == 0) {
        ++main_leaves;
      } else {
        ++leaves_per_axis[leaf.axis_id];
      }
    }
    if (!node.data.Is<SorghumInternode>()) {
      continue;
    }
    const auto& internode = node.data.Get<SorghumInternode>();
    if (internode.order != 1) {
      continue;
    }
    EXPECT_EQ(internode.axis_phytomer_count, 9);
    EXPECT_EQ(internode.origin_rank, params.tiller_origin_ranks[static_cast<size_t>(internode.axis_id - 1)]);
    tiller_axes.insert(internode.axis_id);
    if (internode.rank != 0) {
      continue;
    }
    const auto& parent = graph.PeekNode(node.GetParentHandle());
    ASSERT_TRUE(parent.data.Is<SorghumRoot>());
  }

  EXPECT_EQ(main_leaves, 10);
  EXPECT_EQ(tiller_axes, (std::set<int>{1, 2, 3}));
  for (const int axis_id : tiller_axes) {
    EXPECT_EQ(leaves_per_axis[axis_id], ComputeSorghumTillerLeafBudget(10, 0.90f));
  }
}

TEST(LSystemSorghumTillers, TillerBranchAnglesRecoverToFinalLeanSmoothly) {
  constexpr float kInsertionAngle = 40.0f;
  constexpr float kFinalLean = 15.0f;
  constexpr int kAxisPhytomers = 10;

  const float first = ComputeSorghumTillerInternodeBranchAngle(0, kAxisPhytomers, kInsertionAngle, kFinalLean, 1.0f);
  float recovery_sum = 0.0f;
  for (int rank = 1; rank < kAxisPhytomers; ++rank) {
    const float recovery =
        ComputeSorghumTillerInternodeBranchAngle(rank, kAxisPhytomers, kInsertionAngle, kFinalLean, 1.0f);
    EXPECT_LT(recovery, 0.0f);
    recovery_sum += recovery;
  }

  EXPECT_FLOAT_EQ(first, kInsertionAngle);
  EXPECT_NEAR(recovery_sum, -(kInsertionAngle - kFinalLean), 1.0e-5f);
  EXPECT_FLOAT_EQ(
      ComputeSorghumTillerInternodeBranchAngle(kAxisPhytomers, kAxisPhytomers, kInsertionAngle, kFinalLean, 1.0f),
      0.0f);
}

TEST(LSystemSorghumTillers, SelectionAndLeafBudgetsUsePeerAxisRatios) {
  SampledSorghumParams params;
  params.tiller_origin_ranks = {3, 4, 2, 1};

  EXPECT_EQ(ComputeSorghumTillerSelectionIndex(params, 3), 0);
  EXPECT_EQ(ComputeSorghumTillerSelectionIndex(params, 4), 1);
  EXPECT_EQ(ComputeSorghumTillerSelectionIndex(params, 2), 2);
  EXPECT_EQ(ComputeSorghumTillerSelectionIndex(params, 1), 3);
  EXPECT_EQ(ComputeSorghumTillerSelectionIndex(params, 5), -1);
  EXPECT_EQ(ComputeSorghumTillerLeafBudget(18, 0.90f), 16);
  EXPECT_EQ(ComputeSorghumTillerLeafBudget(10, 0.93f), 9);
  EXPECT_FLOAT_EQ(ComputeSorghumAxisRankPosition(0, 9), 0.0f);
  EXPECT_FLOAT_EQ(ComputeSorghumAxisRankPosition(8, 9), 1.0f);
  EXPECT_NEAR(ComputeSorghumTillerCatchUpPlastochron(50.0f, 9, 16, 1.0f), 28.125f, 1.0e-6f);
}
