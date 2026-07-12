#pragma once

#include "GltfSceneFeatures.hpp"
#include "Jobs.hpp"
#include "Shader.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

namespace evo_engine {

class ComputePipeline;
class RayTracingPipeline;

enum class RayCameraShaderTechnique { RayTracing, RayQuery };

constexpr uint32_t kRayCameraDebugViewsFeature = 1u << 31u;
static_assert((kRayCameraDebugViewsFeature & kGltfSceneAllFeatures) == 0u);

struct RayCameraShaderVariantStats {
  uint32_t requested_mask = kGltfSceneAllFeatures;
  uint32_t active_mask = kGltfSceneAllFeatures;
  std::string requested_key;
  std::string active_key;
  std::string cache_source = "fallback";
  std::string last_error;
  bool pending = false;
  bool ready = true;
  bool fallback_active = true;
  bool failed = false;
  uint64_t build_count = 0;
  uint64_t activation_count = 0;
  uint64_t accumulation_reset_count = 0;
  uint64_t fallback_frame_count = 0;
  double build_milliseconds = 0.0;
  double request_to_ready_milliseconds = 0.0;
  double fallback_build_milliseconds = 0.0;
  ShaderCompileCacheStats shader_cache;
};

struct RayCameraShaderVariantUpdate {
  bool ray_tracing_activated = false;
  bool ray_query_activated = false;
};

class RayCameraShaderVariantCache final {
 public:
  using RayTracingFactory = std::function<std::shared_ptr<RayTracingPipeline>(uint32_t feature_mask)>;
  using RayQueryFactory = std::function<std::shared_ptr<ComputePipeline>(uint32_t feature_mask)>;

  RayCameraShaderVariantCache(std::shared_ptr<RayTracingPipeline> ray_tracing_fallback,
                              std::shared_ptr<ComputePipeline> ray_query_fallback,
                              RayTracingFactory ray_tracing_factory, RayQueryFactory ray_query_factory,
                              double ray_tracing_fallback_build_milliseconds = 0.0,
                              double ray_query_fallback_build_milliseconds = 0.0);
  ~RayCameraShaderVariantCache();

  RayCameraShaderVariantUpdate Update(uint32_t feature_mask, bool need_ray_tracing, bool need_ray_query,
                                      bool need_ray_tracing_debug_views = false,
                                      bool need_ray_query_debug_views = false);
  [[nodiscard]] std::shared_ptr<RayTracingPipeline> GetRayTracingPipeline() const;
  [[nodiscard]] std::shared_ptr<ComputePipeline> GetRayQueryPipeline() const;
  [[nodiscard]] RayCameraShaderVariantStats GetStats(RayCameraShaderTechnique technique) const;
  [[nodiscard]] bool IsReady(RayCameraShaderTechnique technique) const;
  void RecordFallbackFrame(RayCameraShaderTechnique technique);
  void RecordAccumulationReset(RayCameraShaderTechnique technique);
  void WaitForJobs();

 private:
  template <typename Pipeline>
  struct Entry {
    uint32_t mask = 0;
    JobHandle job;
    mutable std::mutex mutex;
    std::shared_ptr<Pipeline> pipeline;
    std::string cache_source;
    std::string error;
    std::chrono::steady_clock::time_point requested_at{};
    double build_milliseconds = 0.0;
    double request_to_ready_milliseconds = 0.0;
    uint64_t retry_after_update = 0;
    bool completed = false;
    bool success = false;
  };

  template <typename Pipeline>
  struct TechniqueState {
    std::shared_ptr<Pipeline> fallback;
    std::shared_ptr<Pipeline> active;
    std::unordered_map<uint32_t, std::shared_ptr<Entry<Pipeline>>> entries;
    RayCameraShaderVariantStats stats;
  };

  void PollCompletedJobs();
  bool UpdateRayTracing(uint32_t feature_mask);
  bool UpdateRayQuery(uint32_t feature_mask);
  void RequestRayTracing(uint32_t feature_mask);
  void RequestRayQuery(uint32_t feature_mask);

  mutable std::mutex mutex_;
  bool shutting_down_ = false;
  uint64_t update_serial_ = 0;
  TechniqueState<RayTracingPipeline> ray_tracing_;
  TechniqueState<ComputePipeline> ray_query_;
  RayTracingFactory ray_tracing_factory_;
  RayQueryFactory ray_query_factory_;
};

}  // namespace evo_engine
