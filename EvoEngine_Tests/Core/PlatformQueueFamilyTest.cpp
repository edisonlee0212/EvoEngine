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

TEST(GpuTimestampFrames, PreservesOriginSessionOrderingAndRepeatedInstances) {
  const GpuTimestampScopeMetadata first{"geometry", "Deferred Geometry", "Geometry", GpuTimestampQueue::Graphics, 17,
                                        0};
  const GpuTimestampScopeMetadata second{"geometry", "Deferred Geometry", "Geometry", GpuTimestampQueue::Graphics, 42,
                                         1};
  const std::vector<GpuTimestampQuerySample> queries = {
      {second, 1, 2, 3},
      {first, 0, 0, 1},
  };

  const auto frame = BuildGpuTimestampFrameSnapshot(91, 3, 256, 4, 2, queries, {10, 30, 40, 90}, 1000.0, 64);
  ASSERT_TRUE(frame.results_available);
  EXPECT_EQ(frame.application_frame_index, 91u);
  EXPECT_EQ(frame.capture_session_index, 3u);
  EXPECT_EQ(frame.query_capacity, 256u);
  EXPECT_EQ(frame.query_used, 4u);
  EXPECT_EQ(frame.skipped_scope_count, 2u);
  ASSERT_EQ(frame.samples.size(), 2u);
  EXPECT_EQ(frame.samples[0].metadata.view_id, 17u);
  EXPECT_EQ(frame.samples[1].metadata.view_id, 42u);
  EXPECT_DOUBLE_EQ(frame.samples[0].begin_offset_milliseconds, 0.0);
  EXPECT_DOUBLE_EQ(frame.samples[1].begin_offset_milliseconds, 0.03);
  EXPECT_DOUBLE_EQ(frame.samples[0].duration_milliseconds, 0.02);
  EXPECT_DOUBLE_EQ(frame.samples[1].duration_milliseconds, 0.05);
  EXPECT_DOUBLE_EQ(frame.span_milliseconds, 0.08);

  const auto aggregates = BuildGpuTimestampPassAggregates(frame);
  ASSERT_EQ(aggregates.size(), 1u);
  EXPECT_EQ(aggregates[0].call_count, 2u);
  EXPECT_DOUBLE_EQ(aggregates[0].total_milliseconds, 0.07);
  ASSERT_EQ(aggregates[0].instances.size(), 2u);
  EXPECT_EQ(aggregates[0].instances[1].metadata.instance_id, 1u);
}

TEST(GpuTimestampFrames, AggregatesStableGroupsWithMissingFramesAndDutyCycle) {
  const GpuTimestampScopeMetadata geometry{"geometry", "Deferred Geometry", "Geometry"};
  const GpuTimestampScopeMetadata ddgi{"ddgi", "DDGI Probe Trace", "AO / DDGI"};
  const GpuTimestampScopeMetadata marker{
      "marker", "Zero Duration Marker", "Instrumentation", GpuTimestampQueue::Graphics, 0, 0, false};
  GpuTimestampFrameSnapshot first;
  first.results_available = true;
  first.span_milliseconds = 5.0;
  first.samples = {
      {geometry, 0, 0, 1, 0.0, 2.0, 2.0},
      {ddgi, 1, 2, 3, 2.0, 3.0, 1.0},
      {marker, 2, 4, 5, 3.0, 3.0, 0.0},
  };
  GpuTimestampFrameSnapshot second;
  second.results_available = true;
  second.span_milliseconds = 4.0;
  second.samples = {{geometry, 0, 0, 1, 0.0, 3.0, 3.0}};

  const auto history = BuildGpuTimestampHistoryStats({first, second}, 3);
  EXPECT_EQ(history.frame_count, 3);
  EXPECT_EQ(history.available_frame_count, 2);
  EXPECT_DOUBLE_EQ(history.span.stats.AverageMilliseconds(), 3.0);
  EXPECT_DOUBLE_EQ(history.span.selected_milliseconds, 0.0);
  EXPECT_DOUBLE_EQ(history.summed_work.stats.AverageMilliseconds(), 2.0);
  ASSERT_EQ(history.groups.size(), 3);
  EXPECT_EQ(history.groups[0].name, "Geometry");
  ASSERT_EQ(history.groups[0].passes.size(), 1);
  EXPECT_EQ(history.groups[0].passes[0].duration.observed_frame_count, 2);
  EXPECT_DOUBLE_EQ(history.groups[0].passes[0].duration.duty_cycle, 2.0 / 3.0);
  EXPECT_DOUBLE_EQ(history.groups[0].passes[0].duration.stats.AverageMilliseconds(), 5.0 / 3.0);
  EXPECT_EQ(history.groups[1].passes[0].duration.observed_frame_count, 1);
  EXPECT_DOUBLE_EQ(history.groups[1].passes[0].duration.duty_cycle, 1.0 / 3.0);
  EXPECT_EQ(history.groups[2].passes[0].duration.observed_frame_count, 1);
  EXPECT_DOUBLE_EQ(history.groups[2].passes[0].duration.duty_cycle, 1.0 / 3.0);
}

TEST(GpuTimestampFrames, ReportsDelayedUnavailableAndMalformedQueryResults) {
  const GpuTimestampScopeMetadata metadata{"lighting", "Deferred Lighting", "Lighting"};
  const std::vector<GpuTimestampQuerySample> queries = {
      {metadata, 0, 0, 1},
      {metadata, 1, 2, 3},
  };

  const auto delayed = BuildGpuTimestampFrameSnapshot(12, 2, 8, 5, 0, queries, {}, 1.0, 64);
  EXPECT_FALSE(delayed.results_available);
  EXPECT_EQ(delayed.unresolved_scope_count, 2u);
  EXPECT_TRUE(delayed.samples.empty());

  const std::vector<GpuTimestampQuerySample> malformed_queries = {
      {metadata, 0, 0, 1},
      {metadata, 1, 2, 5},
  };
  const auto malformed = BuildGpuTimestampFrameSnapshot(12, 2, 8, 5, 0, malformed_queries, {1, 2, 3, 4, 5}, 1.0, 64);
  EXPECT_TRUE(malformed.results_available);
  EXPECT_EQ(malformed.unresolved_scope_count, 1u);
  EXPECT_EQ(malformed.samples.size(), 1u);
}

TEST(GpuTimestampFrames, HandlesTimestampWrapUsingQueueValidBits) {
  const GpuTimestampScopeMetadata metadata{"post", "Post Processing", "Post"};
  const auto frame = BuildGpuTimestampFrameSnapshot(5, 1, 2, 2, 0, {{metadata, 0, 0, 1}}, {250, 5}, 1.0, 8);
  ASSERT_EQ(frame.samples.size(), 1u);
  EXPECT_DOUBLE_EQ(frame.samples[0].duration_milliseconds, 11.0 / 1.0e6);
  EXPECT_DOUBLE_EQ(frame.span_milliseconds, 11.0 / 1.0e6);
}
