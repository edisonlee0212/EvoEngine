#include "RayCameraShaderVariantCache.hpp"

#include "ComputePipeline.hpp"
#include "Platform.hpp"
#include "RayTracingPipeline.hpp"
#include "Shader.hpp"

#include <algorithm>
#include <cstdint>

using namespace evo_engine;

namespace {
constexpr uint32_t kRayCameraFallbackMask = kGltfSceneAllFeatures | kRayCameraDebugViewsFeature;
constexpr uint32_t kRayCameraVariantCapacity = 8;

std::string VariantKey(const RayCameraShaderTechnique technique, const uint32_t mask) {
  return std::string(technique == RayCameraShaderTechnique::RayTracing ? "rtx:" : "rq:") +
         FormatGltfSceneFeatureMask(mask) + ((mask & kRayCameraDebugViewsFeature) != 0u ? ":debug" : "");
}

std::string CacheSource(const ShaderCompileCacheStats& before, const ShaderCompileCacheStats& after) {
  if (after.compilations > before.compilations)
    return "compiled";
  if (after.disk_hits > before.disk_hits)
    return "disk";
  if (after.memory_hits > before.memory_hits || after.coalesced_waits > before.coalesced_waits)
    return "memory";
  return "cache";
}
}  // namespace

RayCameraShaderVariantCache::RayCameraShaderVariantCache(std::shared_ptr<RayTracingPipeline> ray_tracing_fallback,
                                                         std::shared_ptr<ComputePipeline> ray_query_fallback,
                                                         RayTracingFactory ray_tracing_factory,
                                                         RayQueryFactory ray_query_factory)
    : ray_tracing_factory_(std::move(ray_tracing_factory)), ray_query_factory_(std::move(ray_query_factory)) {
  ray_tracing_.fallback = std::move(ray_tracing_fallback);
  ray_tracing_.active = ray_tracing_.fallback;
  ray_tracing_.stats.requested_mask = kRayCameraFallbackMask;
  ray_tracing_.stats.active_mask = kRayCameraFallbackMask;
  ray_tracing_.stats.requested_key = VariantKey(RayCameraShaderTechnique::RayTracing, kRayCameraFallbackMask);
  ray_tracing_.stats.active_key = ray_tracing_.stats.requested_key;
  ray_query_.fallback = std::move(ray_query_fallback);
  ray_query_.active = ray_query_.fallback;
  ray_query_.stats.requested_mask = kRayCameraFallbackMask;
  ray_query_.stats.active_mask = kRayCameraFallbackMask;
  ray_query_.stats.requested_key = VariantKey(RayCameraShaderTechnique::RayQuery, kRayCameraFallbackMask);
  ray_query_.stats.active_key = ray_query_.stats.requested_key;
}

RayCameraShaderVariantCache::~RayCameraShaderVariantCache() {
  WaitForJobs();
}

void RayCameraShaderVariantCache::PollCompletedJobs() {
  const auto poll = [](auto& state) {
    for (auto& [mask, entry] : state.entries) {
      if (entry->job.Valid() && Jobs::IsCompleted(entry->job)) {
        Jobs::Wait(entry->job);
        const std::lock_guard entry_lock(entry->mutex);
        entry->job = {};
        entry->published = true;
      }
    }
  };
  poll(ray_tracing_);
  poll(ray_query_);
}

template <typename Pipeline>
void RayCameraShaderVariantCache::ReleaseCompletedSubmissions(TechniqueState<Pipeline>& state) {
  state.retained_submissions.erase(std::remove_if(state.retained_submissions.begin(), state.retained_submissions.end(),
                                                  [](const auto& retained) {
                                                    return !retained.submission ||
                                                           retained.submission->status !=
                                                               FrameSubmissionState::Status::Pending;
                                                  }),
                                   state.retained_submissions.end());
}

template <typename Pipeline>
void RayCameraShaderVariantCache::TouchEntry(const std::shared_ptr<Entry<Pipeline>>& entry) {
  entry->last_access_serial = ++access_serial_;
}

template <typename Pipeline>
void RayCameraShaderVariantCache::PruneEntries(TechniqueState<Pipeline>& state, const uint32_t requested_mask) {
  const auto prune = [&](const bool successful, const size_t capacity) {
    while (true) {
      size_t count = 0;
      auto victim = state.entries.end();
      uint64_t victim_serial = UINT64_MAX;
      uint32_t victim_mask = UINT32_MAX;
      for (auto candidate = state.entries.begin(); candidate != state.entries.end(); ++candidate) {
        const auto& entry = candidate->second;
        const std::lock_guard entry_lock(entry->mutex);
        if (!entry->published || entry->success != successful)
          continue;
        ++count;
        if (successful && entry->pipeline == state.active)
          continue;
        if (!successful && candidate->first == requested_mask)
          continue;
        if (entry->last_access_serial < victim_serial ||
            (entry->last_access_serial == victim_serial && candidate->first < victim_mask)) {
          victim = candidate;
          victim_serial = entry->last_access_serial;
          victim_mask = candidate->first;
        }
      }
      if (count <= capacity || victim == state.entries.end())
        return;
      state.entries.erase(victim);
      ++state.stats.eviction_count;
    }
  };
  prune(true, kRayCameraVariantCapacity);
  prune(false, kRayCameraVariantCapacity);

  state.stats.resident_variant_count = 0;
  state.stats.pending_build_count = 0;
  state.stats.failed_entry_count = 0;
  for (const auto& [mask, entry] : state.entries) {
    const std::lock_guard entry_lock(entry->mutex);
    if (!entry->published)
      ++state.stats.pending_build_count;
    else if (entry->success)
      ++state.stats.resident_variant_count;
    else
      ++state.stats.failed_entry_count;
  }
  state.stats.retained_submission_count = static_cast<uint32_t>(state.retained_submissions.size());
  state.stats.variant_capacity = kRayCameraVariantCapacity;
}

void RayCameraShaderVariantCache::RequestRayTracing(const uint32_t feature_mask) {
  if (!ray_tracing_factory_ || !ray_tracing_.fallback ||
      ray_tracing_.entries.find(feature_mask) != ray_tracing_.entries.end())
    return;
  for (const auto& [mask, pending] : ray_tracing_.entries) {
    const std::lock_guard entry_lock(pending->mutex);
    if (!pending->published)
      return;
  }
  auto entry = std::make_shared<Entry<RayTracingPipeline>>();
  entry->mask = feature_mask;
  TouchEntry(entry);
  const auto factory = ray_tracing_factory_;
  entry->job = Jobs::RunOnRenderThread([entry, factory]() {
    const auto before = Shader::GetCompileCacheStats();
    try {
      auto pipeline = factory(entry->mask);
      const bool success = pipeline && pipeline->Initialized();
      const std::lock_guard lock(entry->mutex);
      entry->pipeline = success ? std::move(pipeline) : nullptr;
      entry->success = success;
      entry->cache_source = CacheSource(before, Shader::GetCompileCacheStats());
      entry->error = success ? std::string() : "Ray-tracing pipeline initialization failed.";
    } catch (const std::exception& error) {
      const std::lock_guard lock(entry->mutex);
      entry->error = error.what();
    } catch (...) {
      const std::lock_guard lock(entry->mutex);
      entry->error = "Unknown ray-tracing pipeline build failure.";
    }
  });
  ray_tracing_.entries.emplace(feature_mask, entry);
  Jobs::Execute(entry->job);
}

void RayCameraShaderVariantCache::RequestRayQuery(const uint32_t feature_mask) {
  if (!ray_query_factory_ || !ray_query_.fallback || ray_query_.entries.find(feature_mask) != ray_query_.entries.end())
    return;
  for (const auto& [mask, pending] : ray_query_.entries) {
    const std::lock_guard entry_lock(pending->mutex);
    if (!pending->published)
      return;
  }
  auto entry = std::make_shared<Entry<ComputePipeline>>();
  entry->mask = feature_mask;
  TouchEntry(entry);
  const auto factory = ray_query_factory_;
  entry->job = Jobs::RunOnRenderThread([entry, factory]() {
    const auto before = Shader::GetCompileCacheStats();
    try {
      auto pipeline = factory(entry->mask);
      const bool success = pipeline && pipeline->Initialized();
      const std::lock_guard lock(entry->mutex);
      entry->pipeline = success ? std::move(pipeline) : nullptr;
      entry->success = success;
      entry->cache_source = CacheSource(before, Shader::GetCompileCacheStats());
      entry->error = success ? std::string() : "Ray Query pipeline initialization failed.";
    } catch (const std::exception& error) {
      const std::lock_guard lock(entry->mutex);
      entry->error = error.what();
    } catch (...) {
      const std::lock_guard lock(entry->mutex);
      entry->error = "Unknown Ray Query pipeline build failure.";
    }
  });
  ray_query_.entries.emplace(feature_mask, entry);
  Jobs::Execute(entry->job);
}

bool RayCameraShaderVariantCache::UpdateRayTracing(const uint32_t feature_mask) {
  auto& stats = ray_tracing_.stats;
  stats.requested_mask = feature_mask;
  stats.requested_key = VariantKey(RayCameraShaderTechnique::RayTracing, feature_mask);
  stats.failed = false;
  stats.last_error.clear();
  if (!ray_tracing_.fallback)
    return false;
  if (feature_mask == kRayCameraFallbackMask) {
    const bool changed = ray_tracing_.active != ray_tracing_.fallback;
    ray_tracing_.active = ray_tracing_.fallback;
    stats.active_mask = kRayCameraFallbackMask;
    stats.active_key = stats.requested_key;
    stats.cache_source = "fallback";
    stats.pending = false;
    stats.ready = true;
    return changed;
  }
  if (const auto search = ray_tracing_.entries.find(feature_mask); search != ray_tracing_.entries.end()) {
    const auto entry = search->second;
    const std::lock_guard entry_lock(entry->mutex);
    if (entry->published && entry->success) {
      TouchEntry(entry);
      const bool changed = ray_tracing_.active != entry->pipeline;
      ray_tracing_.active = entry->pipeline;
      stats.active_mask = feature_mask;
      stats.active_key = stats.requested_key;
      stats.cache_source = entry->cache_source;
      stats.pending = false;
      stats.ready = true;
      return changed;
    }
    if (entry->published) {
      if (entry->retry_after_update == 0)
        entry->retry_after_update = update_serial_ + 120u;
      if (update_serial_ >= entry->retry_after_update) {
        ray_tracing_.entries.erase(search);
        RequestRayTracing(feature_mask);
        stats.pending = true;
        stats.ready = false;
        const bool changed = ray_tracing_.active != ray_tracing_.fallback;
        ray_tracing_.active = ray_tracing_.fallback;
        stats.active_mask = kRayCameraFallbackMask;
        stats.active_key = VariantKey(RayCameraShaderTechnique::RayTracing, kRayCameraFallbackMask);
        stats.cache_source = "fallback";
        return changed;
      }
      stats.pending = false;
      stats.ready = false;
      stats.failed = true;
      stats.last_error = entry->error;
    } else {
      stats.pending = true;
      stats.ready = false;
    }
  } else {
    RequestRayTracing(feature_mask);
    stats.pending = true;
    stats.ready = false;
  }
  if ((stats.active_mask & feature_mask) != feature_mask) {
    const bool changed = ray_tracing_.active != ray_tracing_.fallback;
    ray_tracing_.active = ray_tracing_.fallback;
    stats.active_mask = kRayCameraFallbackMask;
    stats.active_key = VariantKey(RayCameraShaderTechnique::RayTracing, kRayCameraFallbackMask);
    stats.cache_source = "fallback";
    return changed;
  }
  return false;
}

bool RayCameraShaderVariantCache::UpdateRayQuery(const uint32_t feature_mask) {
  auto& stats = ray_query_.stats;
  stats.requested_mask = feature_mask;
  stats.requested_key = VariantKey(RayCameraShaderTechnique::RayQuery, feature_mask);
  stats.failed = false;
  stats.last_error.clear();
  if (!ray_query_.fallback)
    return false;
  if (feature_mask == kRayCameraFallbackMask) {
    const bool changed = ray_query_.active != ray_query_.fallback;
    ray_query_.active = ray_query_.fallback;
    stats.active_mask = kRayCameraFallbackMask;
    stats.active_key = stats.requested_key;
    stats.cache_source = "fallback";
    stats.pending = false;
    stats.ready = true;
    return changed;
  }
  if (const auto search = ray_query_.entries.find(feature_mask); search != ray_query_.entries.end()) {
    const auto entry = search->second;
    const std::lock_guard entry_lock(entry->mutex);
    if (entry->published && entry->success) {
      TouchEntry(entry);
      const bool changed = ray_query_.active != entry->pipeline;
      ray_query_.active = entry->pipeline;
      stats.active_mask = feature_mask;
      stats.active_key = stats.requested_key;
      stats.cache_source = entry->cache_source;
      stats.pending = false;
      stats.ready = true;
      return changed;
    }
    if (entry->published) {
      if (entry->retry_after_update == 0)
        entry->retry_after_update = update_serial_ + 120u;
      if (update_serial_ >= entry->retry_after_update) {
        ray_query_.entries.erase(search);
        RequestRayQuery(feature_mask);
        stats.pending = true;
        stats.ready = false;
        const bool changed = ray_query_.active != ray_query_.fallback;
        ray_query_.active = ray_query_.fallback;
        stats.active_mask = kRayCameraFallbackMask;
        stats.active_key = VariantKey(RayCameraShaderTechnique::RayQuery, kRayCameraFallbackMask);
        stats.cache_source = "fallback";
        return changed;
      }
      stats.pending = false;
      stats.ready = false;
      stats.failed = true;
      stats.last_error = entry->error;
    } else {
      stats.pending = true;
      stats.ready = false;
    }
  } else {
    RequestRayQuery(feature_mask);
    stats.pending = true;
    stats.ready = false;
  }
  if ((stats.active_mask & feature_mask) != feature_mask) {
    const bool changed = ray_query_.active != ray_query_.fallback;
    ray_query_.active = ray_query_.fallback;
    stats.active_mask = kRayCameraFallbackMask;
    stats.active_key = VariantKey(RayCameraShaderTechnique::RayQuery, kRayCameraFallbackMask);
    stats.cache_source = "fallback";
    return changed;
  }
  return false;
}

RayCameraShaderVariantUpdate RayCameraShaderVariantCache::Update(const uint32_t feature_mask,
                                                                 const bool need_ray_tracing, const bool need_ray_query,
                                                                 const bool need_ray_tracing_debug_views,
                                                                 const bool need_ray_query_debug_views) {
  const std::lock_guard lock(mutex_);
  ++update_serial_;
  PollCompletedJobs();
  ReleaseCompletedSubmissions(ray_tracing_);
  ReleaseCompletedSubmissions(ray_query_);
  RayCameraShaderVariantUpdate result;
  const auto promoted_mask = PromoteGltfSceneFeatures(feature_mask);
  if (need_ray_tracing)
    result.ray_tracing_activated =
        UpdateRayTracing(promoted_mask | (need_ray_tracing_debug_views ? kRayCameraDebugViewsFeature : 0u));
  if (need_ray_query)
    result.ray_query_activated =
        UpdateRayQuery(promoted_mask | (need_ray_query_debug_views ? kRayCameraDebugViewsFeature : 0u));
  PruneEntries(ray_tracing_, promoted_mask | (need_ray_tracing_debug_views ? kRayCameraDebugViewsFeature : 0u));
  PruneEntries(ray_query_, promoted_mask | (need_ray_query_debug_views ? kRayCameraDebugViewsFeature : 0u));
  return result;
}

std::shared_ptr<RayTracingPipeline> RayCameraShaderVariantCache::GetRayTracingPipeline() const {
  const std::lock_guard lock(mutex_);
  return ray_tracing_.active;
}

std::shared_ptr<ComputePipeline> RayCameraShaderVariantCache::GetRayQueryPipeline() const {
  const std::lock_guard lock(mutex_);
  return ray_query_.active;
}

RayCameraShaderVariantStats RayCameraShaderVariantCache::GetStats(const RayCameraShaderTechnique technique) const {
  const std::lock_guard lock(mutex_);
  auto result = technique == RayCameraShaderTechnique::RayTracing ? ray_tracing_.stats : ray_query_.stats;
  result.requested_mask &= kGltfSceneAllFeatures;
  result.active_mask &= kGltfSceneAllFeatures;
  if (technique == RayCameraShaderTechnique::RayTracing && ray_tracing_.active)
    result.pipeline_creation = ray_tracing_.active->GetCreationFeedback();
  if (technique == RayCameraShaderTechnique::RayQuery && ray_query_.active)
    result.pipeline_creation = ray_query_.active->GetCreationFeedback();
  return result;
}

bool RayCameraShaderVariantCache::IsReady(const RayCameraShaderTechnique technique) const {
  return GetStats(technique).ready;
}

void RayCameraShaderVariantCache::RecordActiveUse(const RayCameraShaderTechnique technique) {
  const std::lock_guard lock(mutex_);
  const auto record = [&](auto& state) {
    ReleaseCompletedSubmissions(state);
    if (!state.active || state.active == state.fallback)
      return;
    for (auto& [mask, entry] : state.entries) {
      const std::lock_guard entry_lock(entry->mutex);
      if (entry->pipeline == state.active) {
        TouchEntry(entry);
        break;
      }
    }
    const auto frame_index = Platform::GetFrameCount();
    if (!state.retained_submissions.empty()) {
      const auto& latest = state.retained_submissions.back();
      if (latest.frame_index == frame_index && latest.pipeline == state.active)
        return;
    }
    state.retained_submissions.push_back({state.active, Platform::TrackCurrentFrameSubmission(), frame_index});
    state.stats.retained_submission_count = static_cast<uint32_t>(state.retained_submissions.size());
  };
  if (technique == RayCameraShaderTechnique::RayTracing)
    record(ray_tracing_);
  else
    record(ray_query_);
}

void RayCameraShaderVariantCache::WaitForJobs() {
  std::vector<JobHandle> jobs;
  {
    const std::lock_guard lock(mutex_);
    if (shutting_down_)
      return;
    shutting_down_ = true;
    for (const auto& [mask, entry] : ray_tracing_.entries) {
      if (entry->job.Valid())
        jobs.emplace_back(entry->job);
    }
    for (const auto& [mask, entry] : ray_query_.entries) {
      if (entry->job.Valid())
        jobs.emplace_back(entry->job);
    }
  }
  for (const auto& job : jobs)
    Jobs::Wait(job);
}
