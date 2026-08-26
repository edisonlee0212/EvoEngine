#pragma once

#include "Platform.hpp"
#include "Profiler.hpp"

namespace evo_engine {

enum class GpuProfilerRecordedQueue : uint8_t { Main, Compute };

class GpuProfilerCommandScope final {
 public:
  GpuProfilerCommandScope(VkCommandBuffer command_buffer, ProfilerItemHandle handle);
  ~GpuProfilerCommandScope();
  GpuProfilerCommandScope(const GpuProfilerCommandScope&) = delete;
  GpuProfilerCommandScope& operator=(const GpuProfilerCommandScope&) = delete;

 private:
  VkCommandBuffer command_buffer_ = VK_NULL_HANDLE;
  GpuTimestampScopeToken token_{};
};

class RecordedGpuProfilerScope final {
 public:
  explicit RecordedGpuProfilerScope(ProfilerItemHandle handle,
                                    GpuProfilerRecordedQueue queue = GpuProfilerRecordedQueue::Main);
  ~RecordedGpuProfilerScope();
  RecordedGpuProfilerScope(const RecordedGpuProfilerScope&) = delete;
  RecordedGpuProfilerScope& operator=(const RecordedGpuProfilerScope&) = delete;

 private:
  GpuProfilerRecordedQueue queue_ = GpuProfilerRecordedQueue::Main;
  GpuTimestampScopeToken token_{};
};

}  // namespace evo_engine
