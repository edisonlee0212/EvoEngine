#include "GpuProfiler.hpp"

using namespace evo_engine;

namespace {
GpuTimestampQueue ToTimestampQueue(const ProfilerGpuQueue queue) {
  switch (queue) {
    case ProfilerGpuQueue::Compute:
      return GpuTimestampQueue::Compute;
    case ProfilerGpuQueue::Transfer:
      return GpuTimestampQueue::Transfer;
    case ProfilerGpuQueue::RayTracing:
      return GpuTimestampQueue::RayTracing;
    case ProfilerGpuQueue::Immediate:
      return GpuTimestampQueue::Immediate;
    case ProfilerGpuQueue::Graphics:
      return GpuTimestampQueue::Graphics;
  }
  return GpuTimestampQueue::Graphics;
}

GpuTimestampScopeMetadata ResolveMetadata(const ProfilerItemHandle handle) {
  const auto item = Profiler::GetInstance().FindRegisteredItem(handle);
  if (!item || !item->descriptor.gpu)
    return {};
  return {item->stable_id,
          item->descriptor.display_name,
          item->descriptor.gpu_group.empty() ? item->owner_name : item->descriptor.gpu_group,
          ToTimestampQueue(item->descriptor.gpu_queue),
          0,
          0,
          item->descriptor.gpu_contributes_to_frame_total,
          item->owner_name};
}

void Record(const GpuProfilerRecordedQueue queue, const std::function<void(VkCommandBuffer)>& action) {
  if (queue == GpuProfilerRecordedQueue::Compute)
    Platform::RecordCommandsComputeQueue(action);
  else
    Platform::RecordCommandsMainQueue(action);
}
}  // namespace

GpuProfilerCommandScope::GpuProfilerCommandScope(const VkCommandBuffer command_buffer, const ProfilerItemHandle handle)
    : command_buffer_(command_buffer) {
  if (!Platform::GpuTimestampCaptureEnabled())
    return;
  const auto metadata = ResolveMetadata(handle);
  if (!metadata.stable_pass_id.empty())
    token_ = Platform::BeginGpuTimestampScope(command_buffer_, metadata);
}

GpuProfilerCommandScope::~GpuProfilerCommandScope() {
  Platform::EndGpuTimestampScope(command_buffer_, token_);
}

RecordedGpuProfilerScope::RecordedGpuProfilerScope(const ProfilerItemHandle handle,
                                                   const GpuProfilerRecordedQueue queue)
    : queue_(queue) {
  if (!Platform::GpuTimestampCaptureEnabled())
    return;
  const auto metadata = ResolveMetadata(handle);
  if (!metadata.stable_pass_id.empty())
    Record(queue_, [&](const VkCommandBuffer command_buffer) {
      token_ = Platform::BeginGpuTimestampScope(command_buffer, metadata);
    });
}

RecordedGpuProfilerScope::~RecordedGpuProfilerScope() {
  if (!token_.valid)
    return;
  Record(queue_, [&](const VkCommandBuffer command_buffer) {
    Platform::EndGpuTimestampScope(command_buffer, token_);
  });
}
