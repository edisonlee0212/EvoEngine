#include "EvoEngine_SDK_PCH.hpp"

#include "Platform.hpp"

#include <gtest/gtest.h>

#include <limits>

using namespace evo_engine;

TEST(PlatformQueueFamilies, FallsBackToGraphicsComputeFamilyWhenDedicatedComputeIsUnavailable) {
  const auto selection = Platform::SelectQueueFamilies(
      {{VK_QUEUE_GRAPHICS_BIT | VK_QUEUE_COMPUTE_BIT, true, 2}, {VK_QUEUE_TRANSFER_BIT, false, 1}});

  ASSERT_TRUE(selection.graphics_and_compute_family.has_value());
  ASSERT_TRUE(selection.compute_family.has_value());
  ASSERT_TRUE(selection.present_family.has_value());
  EXPECT_EQ(selection.graphics_and_compute_family.value(), 0u);
  EXPECT_EQ(selection.compute_family.value(), 0u);
  EXPECT_EQ(selection.present_family.value(), 0u);
  EXPECT_FALSE(selection.HasDedicatedComputeFamily());
  EXPECT_TRUE(selection.IsComplete(true));
}

TEST(PlatformQueueFamilies, PrefersDedicatedComputeFamilyWhenAvailable) {
  const auto selection = Platform::SelectQueueFamilies({{VK_QUEUE_GRAPHICS_BIT | VK_QUEUE_COMPUTE_BIT, true, 2},
                                                        {VK_QUEUE_COMPUTE_BIT | VK_QUEUE_TRANSFER_BIT, false, 1}});

  ASSERT_TRUE(selection.graphics_and_compute_family.has_value());
  ASSERT_TRUE(selection.compute_family.has_value());
  ASSERT_TRUE(selection.present_family.has_value());
  EXPECT_EQ(selection.graphics_and_compute_family.value(), 0u);
  EXPECT_EQ(selection.compute_family.value(), 1u);
  EXPECT_EQ(selection.present_family.value(), 0u);
  EXPECT_TRUE(selection.HasDedicatedComputeFamily());
  EXPECT_TRUE(selection.IsComplete(true));
}

TEST(PlatformQueueFamilies, IgnoresFamiliesWithoutQueues) {
  const auto selection = Platform::SelectQueueFamilies(
      {{VK_QUEUE_COMPUTE_BIT, false, 0}, {VK_QUEUE_GRAPHICS_BIT | VK_QUEUE_COMPUTE_BIT, false, 1}});

  ASSERT_TRUE(selection.graphics_and_compute_family.has_value());
  ASSERT_TRUE(selection.compute_family.has_value());
  EXPECT_FALSE(selection.present_family.has_value());
  EXPECT_EQ(selection.graphics_and_compute_family.value(), 1u);
  EXPECT_EQ(selection.compute_family.value(), 1u);
  EXPECT_FALSE(selection.HasDedicatedComputeFamily());
  EXPECT_TRUE(selection.IsComplete(false));
  EXPECT_FALSE(selection.IsComplete(true));
}

TEST(PlatformQueueFamilies, AliasesAllGraphicsRolesOnSingleQueueFamily) {
  const auto selection = Platform::SelectQueueFamilies({{VK_QUEUE_GRAPHICS_BIT | VK_QUEUE_COMPUTE_BIT, true, 1}});
  const auto plan = Platform::BuildQueuePlan({1}, selection, true);

  EXPECT_EQ(plan.immediate_queue_index, 0u);
  EXPECT_EQ(plan.main_queue_index, 0u);
  EXPECT_EQ(plan.present_queue_index, 0u);
  ASSERT_EQ(plan.family_priorities.size(), 1u);
  EXPECT_EQ(plan.family_priorities.at(0), (std::vector<float>{1.0f}));
}

TEST(PlatformQueueFamilies, UsesOnlyAvailableGraphicsQueues) {
  const auto selection = Platform::SelectQueueFamilies({{VK_QUEUE_GRAPHICS_BIT | VK_QUEUE_COMPUTE_BIT, true, 2}});
  const auto plan = Platform::BuildQueuePlan({2}, selection, true);

  EXPECT_EQ(plan.immediate_queue_index, 0u);
  EXPECT_EQ(plan.main_queue_index, 1u);
  EXPECT_EQ(plan.present_queue_index, 1u);
  ASSERT_EQ(plan.family_priorities.size(), 1u);
  EXPECT_EQ(plan.family_priorities.at(0), (std::vector<float>{0.0f, 1.0f}));
}

TEST(PlatformQueueFamilies, SeparatesGraphicsRolesWhenThreeQueuesAreAvailable) {
  const auto selection = Platform::SelectQueueFamilies({{VK_QUEUE_GRAPHICS_BIT | VK_QUEUE_COMPUTE_BIT, true, 3}});
  const auto plan = Platform::BuildQueuePlan({3}, selection, true);

  EXPECT_EQ(plan.immediate_queue_index, 0u);
  EXPECT_EQ(plan.main_queue_index, 1u);
  EXPECT_EQ(plan.present_queue_index, 2u);
  ASSERT_EQ(plan.family_priorities.size(), 1u);
  EXPECT_EQ(plan.family_priorities.at(0), (std::vector<float>{0.0f, 1.0f, 1.0f}));
}

TEST(PlatformQueueFamilies, RequestsOneValidQueueFromEachSelectedFamily) {
  const auto selection = Platform::SelectQueueFamilies({{VK_QUEUE_GRAPHICS_BIT | VK_QUEUE_COMPUTE_BIT, false, 1},
                                                        {VK_QUEUE_COMPUTE_BIT, false, 1},
                                                        {VK_QUEUE_TRANSFER_BIT, true, 1}});
  const auto plan = Platform::BuildQueuePlan({1, 1, 1}, selection, true);

  ASSERT_EQ(plan.family_priorities.size(), 3u);
  EXPECT_EQ(plan.family_priorities.at(0), (std::vector<float>{1.0f}));
  EXPECT_EQ(plan.family_priorities.at(1), (std::vector<float>{0.0f}));
  EXPECT_EQ(plan.family_priorities.at(2), (std::vector<float>{1.0f}));
}

TEST(GpuTimestampStats, AggregatesValidSamples) {
  GpuTimestampStats stats;
  stats.name = "Path Trace (RTX)";
  stats.AddSample(2.0);
  stats.AddSample(1.0);
  stats.AddSample(4.0);

  EXPECT_EQ(stats.sample_count, 3u);
  EXPECT_DOUBLE_EQ(stats.last_milliseconds, 4.0);
  EXPECT_DOUBLE_EQ(stats.minimum_milliseconds, 1.0);
  EXPECT_DOUBLE_EQ(stats.maximum_milliseconds, 4.0);
  EXPECT_DOUBLE_EQ(stats.AverageMilliseconds(), 7.0 / 3.0);
  EXPECT_DOUBLE_EQ(stats.MedianMilliseconds(), 2.0);
  EXPECT_DOUBLE_EQ(stats.PercentileMilliseconds(0.95), 3.8);
}

TEST(GpuTimestampStats, InterpolatesEvenSamplePercentilesWithoutMutatingInsertionOrder) {
  GpuTimestampStats stats;
  stats.AddSample(8.0);
  stats.AddSample(2.0);
  stats.AddSample(6.0);
  stats.AddSample(4.0);

  EXPECT_DOUBLE_EQ(stats.MedianMilliseconds(), 5.0);
  EXPECT_DOUBLE_EQ(stats.PercentileMilliseconds(0.0), 2.0);
  EXPECT_DOUBLE_EQ(stats.PercentileMilliseconds(1.0), 8.0);
  EXPECT_EQ(stats.samples_milliseconds, (std::vector<double>{8.0, 2.0, 6.0, 4.0}));
}

TEST(GpuTimestampStats, IgnoresInvalidSamples) {
  GpuTimestampStats stats;
  stats.AddSample(-1.0);
  stats.AddSample(std::numeric_limits<double>::infinity());
  stats.AddSample(std::numeric_limits<double>::quiet_NaN());

  EXPECT_EQ(stats.sample_count, 0u);
  EXPECT_DOUBLE_EQ(stats.AverageMilliseconds(), 0.0);
  EXPECT_DOUBLE_EQ(stats.MedianMilliseconds(), 0.0);
  EXPECT_TRUE(stats.samples_milliseconds.empty());
}
