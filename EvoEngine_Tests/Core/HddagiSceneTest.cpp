#include <gtest/gtest.h>
#include "EvoEngine_SDK_PCH.hpp"
#include "HddagiGather.hpp"
#include "HddagiScene.hpp"

using namespace evo_engine;

TEST(HddagiGather, CameraResolutionAndSharedAnchorMetadata) {
  const auto full = BuildHddagiCameraLayout({2560, 1440});
  EXPECT_EQ(full.gi, glm::uvec2(2560, 1440));
  EXPECT_EQ(full.pixel_stride, 1u);
  EXPECT_EQ(full.reflection_filter_radius, 12u);
  EXPECT_EQ(BuildHddagiCameraLayout({2561, 1441}).gi, glm::uvec2(2561, 1441));
  EXPECT_EQ(BuildHddagiCameraLayout({0, 0}).gi, glm::uvec2(1));
  EXPECT_EQ(BuildHddagiCameraLayout({1, 1}).gi, glm::uvec2(1));
  GiProbeSettings probes;
  probes.probe_count_x = 25;
  probes.probe_count_y = 11;
  probes.cascade_count = 2;
  probes.vertical_scale = GiProbeSettings::VerticalScale::Percent50;
  const glm::vec3 anchor(-37, 15, -101);
  HddagiUpdatePlan plan;
  ASSERT_TRUE(BuildHddagiUpdatePlan(probes, {}, anchor, {}, {}, true, plan).empty());
  const auto data = BuildHddagiGatherData(probes, {}, plan.cascades, anchor);
  EXPECT_EQ(data.grid, glm::ivec3(192, 80, 192));
  EXPECT_EQ(data.anchor_origin, glm::vec3(-37, 30, -101));
  for (uint32_t c = 0; c < probes.cascade_count; ++c) {
    const auto& input = plan.cascades[c];
    const auto& output = data.cascades.data[c];
    const auto minimum = input.position - input.size / 2;
    EXPECT_EQ(output.region_world_offset * 8, minimum);
    const glm::vec3 world_point = glm::vec3(minimum + glm::ivec3(11, 13, 17)) * input.cell_size;
    const glm::vec3 local = (world_point - data.anchor_origin - output.offset) * output.to_cell;
    EXPECT_LT(glm::length(local - glm::vec3(11, 13, 17)), 0.0001f);
  }
}

TEST(HddagiUpdates, EnteringRegionUnionHasNoOverlapAndRetainsWorldCells) {
  GiProbeSettings probes;
  probes.probe_count_x = 25;
  probes.probe_count_y = 11;
  probes.cascade_count = 1;
  probes.base_probe_distance = 8;
  HddagiUpdatePlan initial;
  ASSERT_TRUE(BuildHddagiUpdatePlan(probes, {}, glm::vec3(0), {}, {}, false, initial).empty());
  ASSERT_EQ(initial.regions.size(), 1u);
  EXPECT_EQ(initial.full_cascades, 1u);
  const auto grid = initial.cascades[0].size;
  const auto dimensions = grid / 8;
  const auto volume = dimensions.x * dimensions.y * dimensions.z;
  EXPECT_EQ(initial.region_count, volume);
  for (const auto anchor : {glm::vec3(8, 8, 8), glm::vec3(-8, -8, 0), glm::vec3(0), glm::vec3(1000)}) {
    SCOPED_TRACE(::testing::Message() << anchor.x << "," << anchor.y << "," << anchor.z);
    HddagiUpdatePlan plan;
    ASSERT_TRUE(BuildHddagiUpdatePlan(probes, {}, anchor, initial.cascades, {}, false, plan).empty());
    std::vector<uint8_t> marked(volume, 0);
    for (const auto& region : plan.regions) {
      if (region.core.size == glm::ivec3(0)) {
        EXPECT_TRUE(glm::any(glm::equal(region.light.size, glm::ivec3(1))));
        EXPECT_TRUE(glm::any(glm::equal(region.raster.size, glm::ivec3(2))));
        continue;
      }
      EXPECT_EQ(region.core.offset % 8, glm::ivec3(0));
      EXPECT_EQ(region.core.size % 8, glm::ivec3(0));
      EXPECT_EQ(region.light.offset, glm::max(region.core.offset - 1, glm::ivec3(0)));
      EXPECT_EQ(region.raster.offset, glm::max(region.core.offset - 2, glm::ivec3(0)));
      EXPECT_EQ(region.raster.offset + region.raster.size, glm::min(region.core.offset + region.core.size + 2, grid));
      for (int z = region.core.offset.z / 8; z < (region.core.offset.z + region.core.size.z) / 8; ++z)
        for (int y = region.core.offset.y / 8; y < (region.core.offset.y + region.core.size.y) / 8; ++y)
          for (int x = region.core.offset.x / 8; x < (region.core.offset.x + region.core.size.x) / 8; ++x)
            EXPECT_EQ(marked[x + dimensions.x * (y + dimensions.y * z)]++, 0u);
    }
    uint64_t expected_count = 0;
    for (int z = 0; z < dimensions.z; ++z)
      for (int y = 0; y < dimensions.y; ++y)
        for (int x = 0; x < dimensions.x; ++x) {
          const auto world_min = plan.cascades[0].position - grid / 2 + glm::ivec3(x, y, z) * 8;
          const bool expected = glm::any(glm::greaterThanEqual(world_min, grid / 2)) ||
                                glm::any(glm::lessThanEqual(world_min + 8, -grid / 2));
          EXPECT_EQ(marked[x + dimensions.x * (y + dimensions.y * z)] != 0, expected);
          expected_count += expected;
        }
    EXPECT_EQ(plan.region_count, expected_count);
    if (anchor == glm::vec3(0))
      EXPECT_TRUE(plan.regions.empty());
    else if (anchor != glm::vec3(1000))
      EXPECT_LT(plan.region_count, volume);
  }
}

TEST(HddagiUpdates, EditsCoverOldAndNewBoundsAndUncertainBoundsForceRebuild) {
  GiProbeSettings probes;
  probes.probe_count_x = probes.probe_count_y = 9;
  probes.cascade_count = 1;
  probes.base_probe_distance = 8;
  HddagiUpdatePlan initial;
  ASSERT_TRUE(BuildHddagiUpdatePlan(probes, {}, glm::vec3(0), {}, {}, false, initial).empty());
  SdfgiContributorChange edit;
  edit.flags = SdfgiTransformChanged;
  edit.before = SdfgiContributor{};
  edit.after = SdfgiContributor{};
  edit.before->world_bounds = {glm::vec3(-1), glm::vec3(1)};
  edit.after->world_bounds = {glm::vec3(20, -1, -1), glm::vec3(22, 1, 1)};
  HddagiUpdatePlan plan;
  ASSERT_TRUE(BuildHddagiUpdatePlan(probes, {}, glm::vec3(0), initial.cascades, {edit}, false, plan).empty());
  EXPECT_EQ(plan.region_count, 12u);
  EXPECT_EQ(plan.full_cascades, 0u);
  edit.flags = SdfgiUncertainBounds;
  ASSERT_TRUE(BuildHddagiUpdatePlan(probes, {}, glm::vec3(0), initial.cascades, {edit}, false, plan).empty());
  EXPECT_EQ(plan.region_count, 512u);
  EXPECT_EQ(plan.full_cascades, 1u);
  edit.flags = SdfgiPayloadChanged;
  edit.before.reset();
  edit.after->world_bounds = {glm::vec3(1e20f), glm::vec3(2e20f)};
  ASSERT_TRUE(BuildHddagiUpdatePlan(probes, {}, glm::vec3(0), initial.cascades, {edit}, false, plan).empty());
  EXPECT_TRUE(plan.regions.empty());
  ASSERT_TRUE(BuildHddagiUpdatePlan(probes, {}, glm::vec3(0), initial.cascades, {}, true, plan).empty());
  EXPECT_EQ(plan.full_cascades, 1u);
  const auto before = plan.region_count;
  EXPECT_FALSE(BuildHddagiUpdatePlan(probes, {}, glm::vec3(INFINITY), initial.cascades, {}, false, plan).empty());
  EXPECT_EQ(plan.region_count, before);
  probes.cascade_count = 2;
  ASSERT_TRUE(BuildHddagiUpdatePlan(probes, {}, glm::vec3(0), {}, {}, false, initial).empty());
  edit.after->world_bounds = {glm::vec3(48, -1, -1), glm::vec3(50, 1, 1)};
  ASSERT_TRUE(BuildHddagiUpdatePlan(probes, {}, glm::vec3(0), initial.cascades, {edit}, false, plan).empty());
  ASSERT_FALSE(plan.regions.empty());
  for (const auto& region : plan.regions)
    EXPECT_EQ(region.core.cascade, 1u);
  EXPECT_EQ(plan.reset_history_cascades, 3u);
}
