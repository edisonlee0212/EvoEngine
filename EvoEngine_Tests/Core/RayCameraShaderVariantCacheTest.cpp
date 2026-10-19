#include "EvoEngine_SDK_PCH.hpp"

#include "ComputePipeline.hpp"
#include "Platform.hpp"
#include "RayCameraShaderVariantCache.hpp"
#include "ShaderPreparationQueue.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
#include <utility>
#include <vector>

TEST(ShaderPreparation, PrioritizesRequiredWorkAndPublishesOnlyWhenPolled) {
  evo_engine::ShaderPreparationQueue queue(1);
  const auto owner = std::this_thread::get_id();
  std::thread::id worker;
  queue.BeginRequests();
  queue.SetOptional(true);
  queue.Request<int>("optional", [] {
    return std::make_shared<int>(2);
  });
  queue.SetOptional(false);
  queue.Request<int>("selected", [&] {
    worker = std::this_thread::get_id();
    return std::make_shared<int>(1);
  });
  EXPECT_FALSE(queue.RequiredReady());
  queue.Tick();
  EXPECT_EQ(queue.GetProgress().current, "selected");
  EXPECT_FALSE(queue.GetResult<int>("selected"));
  queue.Wait();
  EXPECT_NE(owner, worker);
  ASSERT_TRUE(queue.GetResult<int>("selected"));
  EXPECT_EQ(*queue.GetResult<int>("selected"), 1);
  EXPECT_TRUE(queue.RequiredReady());
  EXPECT_EQ(queue.GetProgress().completed, 1u);
  queue.Tick();
  queue.Wait();
  EXPECT_EQ(queue.GetProgress().completed, 2u);
}

TEST(ShaderPreparation, PromotesModeChangesAndReportsFailuresWithoutRetryLoops) {
  evo_engine::ShaderPreparationQueue queue(1);
  queue.BeginRequests();
  queue.SetOptional(true);
  queue.Request<int>("previous", [] {
    return std::make_shared<int>(1);
  });
  queue.Request<int>("new camera", []() -> std::shared_ptr<int> {
    throw std::runtime_error("compile failed");
  });
  queue.BeginRequests();
  queue.SetOptional(false);
  queue.Require("new camera");
  queue.Tick();
  EXPECT_EQ(queue.GetProgress().current, "new camera");
  queue.Wait();
  EXPECT_TRUE(queue.IsDone("new camera"));
  EXPECT_NE(queue.GetProgress().error.find("compile failed"), std::string::npos);
  queue.Tick();
  EXPECT_EQ(queue.GetProgress().current, "previous");
  queue.Wait();
  EXPECT_EQ(queue.GetProgress().completed, queue.GetProgress().total);
}

TEST(ShaderPreparation, CancellationDropsUnstartedWorkAndEvictionReleasesResults) {
  evo_engine::ShaderPreparationQueue queue(1);
  const auto factory = [] {
    return std::make_shared<int>(7);
  };
  queue.Request<int>("variant", factory);
  queue.Tick();
  queue.Wait();
  std::weak_ptr<int> lifetime = queue.GetResult<int>("variant");
  EXPECT_FALSE(lifetime.expired());
  queue.ReleaseResult("variant");
  EXPECT_TRUE(lifetime.expired());
  queue.Request<int>("variant", factory);
  queue.Tick();
  queue.Wait();
  ASSERT_TRUE(queue.GetResult<int>("variant"));
  bool unused_built = false;
  queue.Request<int>("unused", [&] {
    unused_built = true;
    return factory();
  });
  queue.Clear();
  EXPECT_FALSE(unused_built);
  EXPECT_EQ(queue.GetProgress().total, 0u);
}

TEST(ShaderPreparation, HeadlessRequestsCanRemainSynchronous) {
  evo_engine::ShaderPreparationQueue queue(1);
  queue.SetSynchronous(true);
  const auto owner = std::this_thread::get_id();
  const auto result = queue.Request<int>("headless", [owner] {
    EXPECT_EQ(owner, std::this_thread::get_id());
    return std::make_shared<int>(3);
  });
  ASSERT_TRUE(result);
  EXPECT_EQ(*result, 3);
  EXPECT_TRUE(queue.RequiredReady());
}

TEST(ShaderPreparation, QueuedFallbackIsPendingRatherThanUnavailable) {
  evo_engine::ShaderPreparationQueue queue(1);
  evo_engine::RayCameraShaderVariantCache cache(nullptr, nullptr, nullptr, nullptr, &queue, "Camera ");
  for (const auto technique :
       {evo_engine::RayCameraShaderTechnique::RayTracing, evo_engine::RayCameraShaderTechnique::RayQuery}) {
    EXPECT_FALSE(cache.IsReady(technique));
    EXPECT_TRUE(cache.GetStats(technique).pending);
  }
  cache.Update(0, true, true);
  EXPECT_FALSE(cache.IsReady(evo_engine::RayCameraShaderTechnique::RayTracing));
  EXPECT_FALSE(cache.IsReady(evo_engine::RayCameraShaderTechnique::RayQuery));
  const auto progress = cache.Prewarm(0, evo_engine::RayCameraShaderTechnique::RayQuery);
  EXPECT_FALSE(progress.ready);
  EXPECT_TRUE(progress.pending);
  EXPECT_FALSE(progress.failed);
  EXPECT_TRUE(progress.last_error.empty());
}

TEST(ShaderPreparation, RunsTwoBuildsConcurrentlyAndBoundsOutstandingWork) {
  evo_engine::ShaderPreparationQueue queue(2);
  std::promise<void> release;
  auto gate = release.get_future().share();
  std::array<std::promise<void>, 2> started;
  std::atomic<int> third_started = 0;
  for (size_t i = 0; i < started.size(); ++i)
    queue.Request<int>(std::to_string(i), [&, i] {
      started[i].set_value();
      gate.wait();
      return std::make_shared<int>(static_cast<int>(i));
    });
  queue.Request<int>("third", [&] {
    ++third_started;
    return std::make_shared<int>(3);
  });
  queue.Tick();
  for (auto& signal : started)
    EXPECT_EQ(signal.get_future().wait_for(std::chrono::seconds(5)), std::future_status::ready);
  EXPECT_EQ(third_started.load(), 0);
  EXPECT_FALSE(queue.GetResult<int>("0"));
  EXPECT_NE(queue.GetProgress().current.find(" | "), std::string::npos);
  release.set_value();
  queue.Wait();
  EXPECT_EQ(queue.GetProgress().completed, 2u);
  queue.Tick();
  queue.Wait();
  EXPECT_EQ(third_started.load(), 1);
  EXPECT_TRUE(queue.RequiredReady());
}

TEST(ShaderPreparation, EightWorkerLimitRunsConcurrentlyAndLeavesExcessWorkQueued) {
  evo_engine::ShaderPreparationQueue queue(99);
  std::promise<void> release;
  auto gate = release.get_future().share();
  std::array<std::promise<void>, 8> started;
  std::atomic<bool> excess_started = false;
  for (size_t i = 0; i < started.size(); ++i)
    queue.Request<int>(std::to_string(i), [&, i] {
      started[i].set_value();
      gate.wait();
      return std::make_shared<int>(static_cast<int>(i));
    });
  queue.Request<int>("excess", [&] {
    excess_started = true;
    return std::make_shared<int>(9);
  });
  queue.Tick();
  for (auto& signal : started)
    EXPECT_EQ(signal.get_future().wait_for(std::chrono::seconds(5)), std::future_status::ready);
  EXPECT_FALSE(excess_started.load());
  release.set_value();
  queue.Wait();
  EXPECT_EQ(queue.GetProgress().completed, 8u);
  queue.Tick();
  queue.Wait();
  EXPECT_TRUE(excess_started.load());
}

TEST(ShaderPreparation, DoesNotStartOptionalWorkWhileRequiredWorkIsPending) {
  evo_engine::ShaderPreparationQueue queue(2);
  queue.Request<int>("required", [] {
    return std::make_shared<int>(1);
  });
  queue.SetOptional(true);
  std::atomic<bool> optional_started = false;
  queue.Request<int>("optional", [&] {
    optional_started = true;
    return std::make_shared<int>(2);
  });
  queue.Tick();
  queue.Wait();
  EXPECT_FALSE(optional_started.load());
  queue.Tick();
  queue.Wait();
  EXPECT_TRUE(optional_started.load());
}

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

TEST(RayCameraShaderVariantCache, PrewarmDoesNotActivateBeautyOrDebugVariants) {
  const auto cache = RayCameraShaderVariantCacheTestAccess::Create();
  const auto active = cache->GetRayQueryPipeline();
  const auto active_mask = cache->GetStats(RayCameraShaderTechnique::RayQuery).active_mask;
  const auto beauty = RayCameraShaderVariantCacheTestAccess::AddSuccess(*cache, 0u, 1u);
  const auto debug = RayCameraShaderVariantCacheTestAccess::AddSuccess(*cache, kRayCameraDebugViewsFeature, 2u);
  EXPECT_TRUE(cache->Prewarm(0u, RayCameraShaderTechnique::RayQuery).ready);
  EXPECT_TRUE(cache->Prewarm(0u, RayCameraShaderTechnique::RayQuery, true).ready);
  EXPECT_EQ(cache->GetRayQueryPipeline(), active);
  EXPECT_EQ(cache->GetStats(RayCameraShaderTechnique::RayQuery).active_mask, active_mask);
  uint32_t builds = 0;
  RayCameraShaderVariantCacheTestAccess::SetFactory(*cache, [&](uint32_t) {
    ++builds;
    return std::make_shared<ComputePipeline>();
  });
  cache->Update(0u, false, true);
  EXPECT_EQ(cache->GetRayQueryPipeline(), beauty);
  cache->Update(0u, false, true, false, true);
  EXPECT_EQ(cache->GetRayQueryPipeline(), debug);
  EXPECT_EQ(builds, 0u);
}

TEST(RayCameraShaderVariantCache, PrewarmDistinguishesPendingAndFailedBuilds) {
  const auto cache = RayCameraShaderVariantCacheTestAccess::Create();
  RayCameraShaderVariantCacheTestAccess::AddPending(*cache, 0u);
  auto status = cache->Prewarm(0u, RayCameraShaderTechnique::RayQuery);
  EXPECT_TRUE(status.pending);
  EXPECT_FALSE(status.ready);
  EXPECT_FALSE(status.failed);
  RayCameraShaderVariantCacheTestAccess::AddFailure(*cache, 0u, 1u);
  status = cache->Prewarm(0u, RayCameraShaderTechnique::RayQuery);
  EXPECT_FALSE(status.pending);
  EXPECT_FALSE(status.ready);
  EXPECT_TRUE(status.failed);
}

TEST(RayCameraShaderVariantCache, PrewarmReusesAllFeatureFallbackWithoutBuilding) {
  const auto cache = RayCameraShaderVariantCacheTestAccess::Create();
  uint32_t builds = 0;
  RayCameraShaderVariantCacheTestAccess::SetFactory(*cache, [&](uint32_t) {
    ++builds;
    return std::make_shared<ComputePipeline>();
  });
  EXPECT_TRUE(cache->Prewarm(kGltfSceneAllFeatures, RayCameraShaderTechnique::RayQuery, true).ready);
  EXPECT_EQ(builds, 0u);
  const auto unavailable = cache->Prewarm(0u, RayCameraShaderTechnique::RayTracing);
  EXPECT_TRUE(unavailable.failed);
  EXPECT_FALSE(unavailable.pending);
}

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

TEST(RayCameraShaderVariantCache, SssAndOrdinaryPtRemainSpecializedForBothTechniques) {
  RayCameraShaderVariantCache cache(nullptr, nullptr, nullptr, nullptr);
  const auto sss = static_cast<uint32_t>(GltfSceneFeature::Subsurface);
  cache.Update(sss, true, true);
  EXPECT_EQ(cache.GetStats(RayCameraShaderTechnique::RayTracing).requested_key, "rtx:0x4000");
  EXPECT_EQ(cache.GetStats(RayCameraShaderTechnique::RayQuery).requested_key, "rq:0x4000");
  EXPECT_EQ(cache.Prewarm(sss, RayCameraShaderTechnique::RayTracing).requested_key, "rtx:0x4000");
  EXPECT_EQ(cache.Prewarm(sss, RayCameraShaderTechnique::RayQuery).requested_key, "rq:0x4000");
  cache.Update(0u, true, true);
  EXPECT_EQ(cache.GetStats(RayCameraShaderTechnique::RayTracing).requested_key, "rtx:0x0000");
  EXPECT_EQ(cache.GetStats(RayCameraShaderTechnique::RayQuery).requested_key, "rq:0x0000");
}

TEST(RayCameraShaderVariantCache, DebugViewsSelectIndependentVariantsForEachTechnique) {
  RayCameraShaderVariantCache cache(nullptr, nullptr, nullptr, nullptr);
  for (const bool ray_tracing_debug : {false, true}) {
    for (const bool ray_query_debug : {false, true}) {
      cache.Update(0u, true, true, ray_tracing_debug, ray_query_debug);
      for (const auto technique : {RayCameraShaderTechnique::RayTracing, RayCameraShaderTechnique::RayQuery}) {
        const bool ray_tracing = technique == RayCameraShaderTechnique::RayTracing;
        const bool debug = ray_tracing ? ray_tracing_debug : ray_query_debug;
        const std::string expected = std::string(ray_tracing ? "rtx:0x0000" : "rq:0x0000") + (debug ? ":debug" : "");
        EXPECT_EQ(cache.GetStats(technique).requested_key, expected);
        EXPECT_EQ(cache.GetStats(technique).requested_mask, 0u);
        EXPECT_EQ(cache.Prewarm(0u, technique, debug).requested_key, expected);
      }
    }
  }
}
