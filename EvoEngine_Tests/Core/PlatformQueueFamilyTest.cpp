#include "EvoEngine_SDK_PCH.hpp"

#include "Platform.hpp"

#include <gtest/gtest.h>

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
