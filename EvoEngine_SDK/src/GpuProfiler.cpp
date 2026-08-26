#include "GpuProfiler.hpp"

using namespace evo_engine;

GpuTimestampScopeMetadata evo_engine::MakeGpuTimestampScopeMetadata(const RegisteredProfilerItem& item) {
  GpuTimestampQueue queue = GpuTimestampQueue::Graphics;
  switch (item.descriptor.gpu_queue) {
    case ProfilerGpuQueue::Compute:
      queue = GpuTimestampQueue::Compute;
      break;
    case ProfilerGpuQueue::Transfer:
      queue = GpuTimestampQueue::Transfer;
      break;
    case ProfilerGpuQueue::RayTracing:
      queue = GpuTimestampQueue::RayTracing;
      break;
    case ProfilerGpuQueue::Immediate:
      queue = GpuTimestampQueue::Immediate;
      break;
    case ProfilerGpuQueue::Graphics:
      break;
  }
  return {item.stable_id,
          item.descriptor.display_name,
          item.descriptor.gpu_group.empty() ? item.owner_name : item.descriptor.gpu_group,
          queue,
          0,
          0,
          item.descriptor.gpu_contributes_to_frame_total,
          item.owner_name};
}

namespace {
std::optional<GpuTimestampScopeMetadata> ResolveMetadata(const ProfilerItemHandle handle) {
  const auto item = Profiler::GetInstance().FindRegisteredItem(handle);
  if (!item || !item->descriptor.gpu)
    return std::nullopt;
  return MakeGpuTimestampScopeMetadata(*item);
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
  if (const auto metadata = ResolveMetadata(handle))
    token_ = Platform::BeginGpuTimestampScope(command_buffer_, *metadata);
}

GpuProfilerCommandScope::~GpuProfilerCommandScope() {
  Platform::EndGpuTimestampScope(command_buffer_, token_);
}

RecordedGpuProfilerScope::RecordedGpuProfilerScope(const ProfilerItemHandle handle,
                                                   const GpuProfilerRecordedQueue queue)
    : queue_(queue) {
  if (!Platform::GpuTimestampCaptureEnabled())
    return;
  if (const auto metadata = ResolveMetadata(handle))
    Record(queue_, [&](const VkCommandBuffer command_buffer) {
      token_ = Platform::BeginGpuTimestampScope(command_buffer, *metadata);
    });
}

RecordedGpuProfilerScope::~RecordedGpuProfilerScope() {
  if (!token_.valid)
    return;
  Record(queue_, [&](const VkCommandBuffer command_buffer) {
    Platform::EndGpuTimestampScope(command_buffer, token_);
  });
}
