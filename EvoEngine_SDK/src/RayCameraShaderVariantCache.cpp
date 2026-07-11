#include "RayCameraShaderVariantCache.hpp"

#include "ComputePipeline.hpp"
#include "RayTracingPipeline.hpp"

using namespace evo_engine;

namespace {
std::string VariantKey(const RayCameraShaderTechnique technique, const uint32_t mask) {
  return std::string(technique == RayCameraShaderTechnique::RayTracing ? "rtx:" : "rq:") +
         FormatGltfSceneFeatureMask(mask);
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
  ray_tracing_.stats.requested_key = VariantKey(RayCameraShaderTechnique::RayTracing, kGltfSceneAllFeatures);
  ray_tracing_.stats.active_key = ray_tracing_.stats.requested_key;
  ray_query_.fallback = std::move(ray_query_fallback);
  ray_query_.active = ray_query_.fallback;
  ray_query_.stats.requested_key = VariantKey(RayCameraShaderTechnique::RayQuery, kGltfSceneAllFeatures);
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
        entry->job = {};
      }
    }
  };
  poll(ray_tracing_);
  poll(ray_query_);
}

void RayCameraShaderVariantCache::RequestRayTracing(const uint32_t feature_mask) {
  if (!ray_tracing_factory_ || !ray_tracing_.fallback ||
      ray_tracing_.entries.find(feature_mask) != ray_tracing_.entries.end())
    return;
  auto entry = std::make_shared<Entry<RayTracingPipeline>>();
  entry->mask = feature_mask;
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
      entry->completed = true;
    } catch (const std::exception& error) {
      const std::lock_guard lock(entry->mutex);
      entry->error = error.what();
      entry->completed = true;
    } catch (...) {
      const std::lock_guard lock(entry->mutex);
      entry->error = "Unknown ray-tracing pipeline build failure.";
      entry->completed = true;
    }
  });
  ray_tracing_.entries.emplace(feature_mask, entry);
  ++ray_tracing_.stats.build_count;
  Jobs::Execute(entry->job);
}

void RayCameraShaderVariantCache::RequestRayQuery(const uint32_t feature_mask) {
  if (!ray_query_factory_ || !ray_query_.fallback || ray_query_.entries.find(feature_mask) != ray_query_.entries.end())
    return;
  auto entry = std::make_shared<Entry<ComputePipeline>>();
  entry->mask = feature_mask;
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
      entry->completed = true;
    } catch (const std::exception& error) {
      const std::lock_guard lock(entry->mutex);
      entry->error = error.what();
      entry->completed = true;
    } catch (...) {
      const std::lock_guard lock(entry->mutex);
      entry->error = "Unknown Ray Query pipeline build failure.";
      entry->completed = true;
    }
  });
  ray_query_.entries.emplace(feature_mask, entry);
  ++ray_query_.stats.build_count;
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
  if (feature_mask == kGltfSceneAllFeatures) {
    const bool changed = ray_tracing_.active != ray_tracing_.fallback;
    ray_tracing_.active = ray_tracing_.fallback;
    stats.active_mask = kGltfSceneAllFeatures;
    stats.active_key = stats.requested_key;
    stats.cache_source = "fallback";
    stats.pending = false;
    stats.ready = true;
    stats.fallback_active = true;
    stats.activation_count += changed ? 1u : 0u;
    return changed;
  }
  if (const auto search = ray_tracing_.entries.find(feature_mask); search != ray_tracing_.entries.end()) {
    const auto entry = search->second;
    const std::lock_guard entry_lock(entry->mutex);
    if (entry->completed && entry->success) {
      const bool changed = ray_tracing_.active != entry->pipeline;
      ray_tracing_.active = entry->pipeline;
      stats.active_mask = feature_mask;
      stats.active_key = stats.requested_key;
      stats.cache_source = entry->cache_source;
      stats.pending = false;
      stats.ready = true;
      stats.fallback_active = false;
      stats.activation_count += changed ? 1u : 0u;
      return changed;
    }
    if (entry->completed) {
      if (entry->retry_after_update == 0)
        entry->retry_after_update = update_serial_ + 120u;
      if (update_serial_ >= entry->retry_after_update) {
        ray_tracing_.entries.erase(search);
        RequestRayTracing(feature_mask);
        stats.pending = true;
        stats.ready = false;
        const bool changed = ray_tracing_.active != ray_tracing_.fallback;
        ray_tracing_.active = ray_tracing_.fallback;
        stats.active_mask = kGltfSceneAllFeatures;
        stats.active_key = VariantKey(RayCameraShaderTechnique::RayTracing, kGltfSceneAllFeatures);
        stats.cache_source = "fallback";
        stats.fallback_active = true;
        stats.activation_count += changed ? 1u : 0u;
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
    stats.active_mask = kGltfSceneAllFeatures;
    stats.active_key = VariantKey(RayCameraShaderTechnique::RayTracing, kGltfSceneAllFeatures);
    stats.cache_source = "fallback";
    stats.fallback_active = true;
    stats.activation_count += changed ? 1u : 0u;
    return changed;
  }
  stats.fallback_active = stats.active_mask != feature_mask;
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
  if (feature_mask == kGltfSceneAllFeatures) {
    const bool changed = ray_query_.active != ray_query_.fallback;
    ray_query_.active = ray_query_.fallback;
    stats.active_mask = kGltfSceneAllFeatures;
    stats.active_key = stats.requested_key;
    stats.cache_source = "fallback";
    stats.pending = false;
    stats.ready = true;
    stats.fallback_active = true;
    stats.activation_count += changed ? 1u : 0u;
    return changed;
  }
  if (const auto search = ray_query_.entries.find(feature_mask); search != ray_query_.entries.end()) {
    const auto entry = search->second;
    const std::lock_guard entry_lock(entry->mutex);
    if (entry->completed && entry->success) {
      const bool changed = ray_query_.active != entry->pipeline;
      ray_query_.active = entry->pipeline;
      stats.active_mask = feature_mask;
      stats.active_key = stats.requested_key;
      stats.cache_source = entry->cache_source;
      stats.pending = false;
      stats.ready = true;
      stats.fallback_active = false;
      stats.activation_count += changed ? 1u : 0u;
      return changed;
    }
    if (entry->completed) {
      if (entry->retry_after_update == 0)
        entry->retry_after_update = update_serial_ + 120u;
      if (update_serial_ >= entry->retry_after_update) {
        ray_query_.entries.erase(search);
        RequestRayQuery(feature_mask);
        stats.pending = true;
        stats.ready = false;
        const bool changed = ray_query_.active != ray_query_.fallback;
        ray_query_.active = ray_query_.fallback;
        stats.active_mask = kGltfSceneAllFeatures;
        stats.active_key = VariantKey(RayCameraShaderTechnique::RayQuery, kGltfSceneAllFeatures);
        stats.cache_source = "fallback";
        stats.fallback_active = true;
        stats.activation_count += changed ? 1u : 0u;
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
    stats.active_mask = kGltfSceneAllFeatures;
    stats.active_key = VariantKey(RayCameraShaderTechnique::RayQuery, kGltfSceneAllFeatures);
    stats.cache_source = "fallback";
    stats.fallback_active = true;
    stats.activation_count += changed ? 1u : 0u;
    return changed;
  }
  stats.fallback_active = stats.active_mask != feature_mask;
  return false;
}

RayCameraShaderVariantUpdate RayCameraShaderVariantCache::Update(const uint32_t feature_mask,
                                                                 const bool need_ray_tracing,
                                                                 const bool need_ray_query) {
  const std::lock_guard lock(mutex_);
  ++update_serial_;
  PollCompletedJobs();
  RayCameraShaderVariantUpdate result;
  const auto promoted_mask = PromoteGltfSceneFeatures(feature_mask);
  if (need_ray_tracing)
    result.ray_tracing_activated = UpdateRayTracing(promoted_mask);
  if (need_ray_query)
    result.ray_query_activated = UpdateRayQuery(promoted_mask);
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
  result.shader_cache = Shader::GetCompileCacheStats();
  return result;
}

bool RayCameraShaderVariantCache::IsReady(const RayCameraShaderTechnique technique) const {
  return GetStats(technique).ready;
}

void RayCameraShaderVariantCache::RecordFallbackFrame(const RayCameraShaderTechnique technique) {
  const std::lock_guard lock(mutex_);
  auto& stats = technique == RayCameraShaderTechnique::RayTracing ? ray_tracing_.stats : ray_query_.stats;
  if (!stats.ready)
    ++stats.fallback_frame_count;
}

void RayCameraShaderVariantCache::RecordAccumulationReset(const RayCameraShaderTechnique technique) {
  const std::lock_guard lock(mutex_);
  auto& stats = technique == RayCameraShaderTechnique::RayTracing ? ray_tracing_.stats : ray_query_.stats;
  ++stats.accumulation_reset_count;
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
