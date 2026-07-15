#include "EvoEngine_SDK_PCH.hpp"

#include "ComputePipeline.hpp"
#include "Platform.hpp"
#include "RayCameraShaderVariantCache.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
#include <utility>
#include <vector>

namespace evo_engine {
class RayCameraShaderVariantCacheTestAccess {
 public:
  static std::shared_ptr<RayCameraShaderVariantCache> Create() {
    return std::make_shared<RayCameraShaderVariantCache>(nullptr, std::make_shared<ComputePipeline>(), nullptr,
                                                         nullptr);
  }

  static std::shared_ptr<ComputePipeline> AddSuccess(RayCameraShaderVariantCache& cache, const uint32_t mask,
                                                     const uint64_t serial) {
    auto entry = std::make_shared<RayCameraShaderVariantCache::Entry<ComputePipeline>>();
    entry->mask = mask;
    entry->pipeline = std::make_shared<ComputePipeline>();
    entry->success = true;
    entry->published = true;
    entry->last_access_serial = serial;
    cache.ray_query_.entries[mask] = entry;
    return entry->pipeline;
  }

  static void AddFailure(RayCameraShaderVariantCache& cache, const uint32_t mask, const uint64_t serial) {
    auto entry = std::make_shared<RayCameraShaderVariantCache::Entry<ComputePipeline>>();
    entry->mask = mask;
    entry->success = false;
    entry->published = true;
    entry->last_access_serial = serial;
    cache.ray_query_.entries[mask] = entry;
  }

  static void MakeFailureRetryable(RayCameraShaderVariantCache& cache, const uint32_t mask) {
    cache.update_serial_ = 120;
    cache.ray_query_.entries.at(mask)->retry_after_update = cache.update_serial_;
  }

  static void RetryFailure(RayCameraShaderVariantCache& cache, const uint32_t mask) {
    cache.UpdateRayQuery(mask);
  }

  static void AddPending(RayCameraShaderVariantCache& cache, const uint32_t mask) {
    auto entry = std::make_shared<RayCameraShaderVariantCache::Entry<ComputePipeline>>();
    entry->mask = mask;
    cache.ray_query_.entries[mask] = entry;
  }

  static void AddBuiltUnpublished(RayCameraShaderVariantCache& cache, const uint32_t mask) {
    auto entry = std::make_shared<RayCameraShaderVariantCache::Entry<ComputePipeline>>();
    entry->mask = mask;
    entry->pipeline = std::make_shared<ComputePipeline>();
    entry->success = true;
    cache.ray_query_.entries[mask] = entry;
  }

  static void SetFactory(RayCameraShaderVariantCache& cache, RayCameraShaderVariantCache::RayQueryFactory factory) {
    cache.ray_query_factory_ = std::move(factory);
  }

  static void Request(RayCameraShaderVariantCache& cache, const uint32_t mask) {
    cache.RequestRayQuery(mask);
  }

  static void Prune(RayCameraShaderVariantCache& cache, const uint32_t requested_mask) {
    cache.PruneEntries(cache.ray_query_, requested_mask);
  }

  static void SetActive(RayCameraShaderVariantCache& cache, const std::shared_ptr<ComputePipeline>& pipeline) {
    cache.ray_query_.active = pipeline;
  }

  static std::pair<uint64_t, uint64_t> TouchWithoutUpdate(RayCameraShaderVariantCache& cache, const uint32_t mask) {
    cache.update_serial_ = 41;
    cache.access_serial_ = 7;
    const auto& entry = cache.ray_query_.entries.at(mask);
    const std::lock_guard lock(entry->mutex);
    cache.TouchEntry(entry);
    return {cache.update_serial_, entry->last_access_serial};
  }

  static std::vector<uint32_t> Masks(const RayCameraShaderVariantCache& cache) {
    std::vector<uint32_t> masks;
    for (const auto& entry : cache.ray_query_.entries)
      masks.push_back(entry.first);
    std::sort(masks.begin(), masks.end());
    return masks;
  }

  static size_t SuccessCount(const RayCameraShaderVariantCache& cache) {
    return static_cast<size_t>(
        std::count_if(cache.ray_query_.entries.begin(), cache.ray_query_.entries.end(), [](const auto& pair) {
          return pair.second->success;
        }));
  }

  static size_t FailureCount(const RayCameraShaderVariantCache& cache) {
    return static_cast<size_t>(
        std::count_if(cache.ray_query_.entries.begin(), cache.ray_query_.entries.end(), [](const auto& pair) {
          return pair.second->published && !pair.second->success;
        }));
  }

  static std::weak_ptr<ComputePipeline> RetainPendingThenEvict(RayCameraShaderVariantCache& cache) {
    auto pipeline = AddSuccess(cache, 1, 1);
    auto state = std::make_shared<FrameSubmissionState>();
    cache.ray_query_.retained_submissions.push_back({pipeline, state, 1});
    cache.ray_query_.active = cache.ray_query_.fallback;
    cache.ray_query_.entries.clear();
    return pipeline;
  }

  static std::shared_ptr<FrameSubmissionState> Submission(RayCameraShaderVariantCache& cache) {
    return cache.ray_query_.retained_submissions.front().submission;
  }

  static void Release(RayCameraShaderVariantCache& cache) {
    cache.ReleaseCompletedSubmissions(cache.ray_query_);
  }
};
}  // namespace evo_engine

using namespace evo_engine;

TEST(RayCameraShaderVariantCache, BoundsSuccessfulVariantsWithDeterministicLruAndPreservesActive) {
  const auto cache = RayCameraShaderVariantCacheTestAccess::Create();
  std::shared_ptr<ComputePipeline> active;
  for (uint32_t mask = 1; mask <= 10; ++mask) {
    auto pipeline = RayCameraShaderVariantCacheTestAccess::AddSuccess(*cache, mask, mask);
    if (mask == 1)
      active = std::move(pipeline);
  }
  RayCameraShaderVariantCacheTestAccess::SetActive(*cache, active);
  RayCameraShaderVariantCacheTestAccess::Prune(*cache, 10);
  EXPECT_EQ(RayCameraShaderVariantCacheTestAccess::SuccessCount(*cache), 8u);
  EXPECT_EQ(RayCameraShaderVariantCacheTestAccess::Masks(*cache), (std::vector<uint32_t>{1, 4, 5, 6, 7, 8, 9, 10}));
  const auto stats = cache->GetStats(RayCameraShaderTechnique::RayQuery);
  EXPECT_EQ(stats.eviction_count, 2u);
  EXPECT_EQ(stats.variant_capacity, 8u);
}

TEST(RayCameraShaderVariantCache, LruTouchDoesNotAdvanceRetryClock) {
  const auto cache = RayCameraShaderVariantCacheTestAccess::Create();
  RayCameraShaderVariantCacheTestAccess::AddSuccess(*cache, 1, 1);
  const auto [update_serial, access_serial] = RayCameraShaderVariantCacheTestAccess::TouchWithoutUpdate(*cache, 1);
  EXPECT_EQ(update_serial, 41u);
  EXPECT_EQ(access_serial, 8u);
}

TEST(RayCameraShaderVariantCache, CoalescesDifferentMasksWhileOneBuildIsPending) {
  const auto cache = RayCameraShaderVariantCacheTestAccess::Create();
  RayCameraShaderVariantCacheTestAccess::AddPending(*cache, 1);
  uint32_t factory_calls = 0;
  RayCameraShaderVariantCacheTestAccess::SetFactory(*cache, [&](const uint32_t) {
    ++factory_calls;
    return std::make_shared<ComputePipeline>();
  });
  RayCameraShaderVariantCacheTestAccess::Request(*cache, 2);
  EXPECT_EQ(factory_calls, 0u);
  EXPECT_EQ(RayCameraShaderVariantCacheTestAccess::Masks(*cache), (std::vector<uint32_t>{1}));
}

TEST(RayCameraShaderVariantCache, DoesNotEvictBuiltEntryUntilItsJobIsPublished) {
  const auto cache = RayCameraShaderVariantCacheTestAccess::Create();
  RayCameraShaderVariantCacheTestAccess::AddBuiltUnpublished(*cache, 1);
  for (uint32_t mask = 2; mask <= 9; ++mask)
    RayCameraShaderVariantCacheTestAccess::AddSuccess(*cache, mask, mask);
  uint32_t factory_calls = 0;
  RayCameraShaderVariantCacheTestAccess::SetFactory(*cache, [&](const uint32_t) {
    ++factory_calls;
    return std::make_shared<ComputePipeline>();
  });
  RayCameraShaderVariantCacheTestAccess::Request(*cache, 10);
  EXPECT_EQ(factory_calls, 0u);
  RayCameraShaderVariantCacheTestAccess::Prune(*cache, 9);
  EXPECT_EQ(RayCameraShaderVariantCacheTestAccess::Masks(*cache).size(), 9u);
  const auto stats = cache->GetStats(RayCameraShaderTechnique::RayQuery);
  EXPECT_EQ(stats.resident_variant_count, 8u);
  EXPECT_EQ(stats.pending_build_count, 1u);
}

TEST(RayCameraShaderVariantCache, BoundsFailuresAndLongRunningMaskChurn) {
  const auto cache = RayCameraShaderVariantCacheTestAccess::Create();
  for (uint32_t mask = 1; mask <= 12; ++mask)
    RayCameraShaderVariantCacheTestAccess::AddFailure(*cache, mask, mask);
  RayCameraShaderVariantCacheTestAccess::Prune(*cache, 12);
  EXPECT_EQ(RayCameraShaderVariantCacheTestAccess::FailureCount(*cache), 8u);
  const auto masks = RayCameraShaderVariantCacheTestAccess::Masks(*cache);
  EXPECT_NE(std::find(masks.begin(), masks.end(), 12u), masks.end());

  for (uint32_t mask = 13; mask <= 1000; ++mask) {
    RayCameraShaderVariantCacheTestAccess::AddSuccess(*cache, mask, mask);
    RayCameraShaderVariantCacheTestAccess::Prune(*cache, mask);
    EXPECT_LE(RayCameraShaderVariantCacheTestAccess::SuccessCount(*cache), 8u);
    EXPECT_LE(RayCameraShaderVariantCacheTestAccess::FailureCount(*cache), 8u);
  }
}

TEST(RayCameraShaderVariantCache, RetryBoundaryRemovesFailedPublicationBeforeRebuild) {
  const auto cache = RayCameraShaderVariantCacheTestAccess::Create();
  RayCameraShaderVariantCacheTestAccess::AddFailure(*cache, 7, 1);
  RayCameraShaderVariantCacheTestAccess::MakeFailureRetryable(*cache, 7);
  RayCameraShaderVariantCacheTestAccess::RetryFailure(*cache, 7);
  EXPECT_TRUE(RayCameraShaderVariantCacheTestAccess::Masks(*cache).empty());
}

TEST(RayCameraShaderVariantCache, RetainsEvictedPipelineUntilSubmissionResolves) {
  const auto cache = RayCameraShaderVariantCacheTestAccess::Create();
  const auto retained = RayCameraShaderVariantCacheTestAccess::RetainPendingThenEvict(*cache);
  ASSERT_FALSE(retained.expired());
  const auto submission = RayCameraShaderVariantCacheTestAccess::Submission(*cache);
  RayCameraShaderVariantCacheTestAccess::Release(*cache);
  EXPECT_FALSE(retained.expired());
  submission->status = FrameSubmissionState::Status::Submitted;
  RayCameraShaderVariantCacheTestAccess::Release(*cache);
  EXPECT_TRUE(retained.expired());
}
