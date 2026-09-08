#include "RenderPasses/DdgiAtlasPreparePass.hpp"

#include "GraphicsResources.hpp"
#include "Platform.hpp"
#include "RenderPasses/RenderPassUtilities.hpp"

using namespace evo_engine;

namespace {
void FillGraphBuffer(const VkCommandBuffer vk_command_buffer, const RenderGraphExecutionContext& context,
                     const char* resource_name, const uint32_t value) {
  const auto* binding = context.GetResourceBinding(resource_name);
  if (binding && binding->buffer && binding->buffer->GetSize() != 0) {
    binding->buffer->Fill(vk_command_buffer, 0, binding->buffer->GetSize(), value);
  }
}

void ClearGraphImage(const VkCommandBuffer vk_command_buffer, const RenderGraphExecutionContext& context,
                     const char* resource_name, const VkClearColorValue& value) {
  const auto* binding = context.GetResourceBinding(resource_name);
  if (binding && binding->image) {
    ClearGraphColorImage(vk_command_buffer, binding->image, value);
  }
}

VkClearColorValue MakeClearColor(const float x, const float y, const float z, const float w) {
  VkClearColorValue value{};
  value.float32[0] = x;
  value.float32[1] = y;
  value.float32[2] = z;
  value.float32[3] = w;
  return value;
}

void RecordAtlasPrepare(const VkCommandBuffer vk_command_buffer, const RenderGraphExecutionContext& context) {
  ApplyGraphResourceBarriers(vk_command_buffer, context);
  const RenderPassGpuTimestampScope gpu_timestamp(vk_command_buffer, context);
  FillGraphBuffer(vk_command_buffer, context, RenderResourceNames::frame_ddgi_probe_metadata, 0u);
  FillGraphBuffer(vk_command_buffer, context, RenderResourceNames::frame_ddgi_probe_state, 0u);
  ClearGraphImage(vk_command_buffer, context, RenderResourceNames::frame_ddgi_irradiance_atlas,
                  MakeClearColor(0.0f, 0.0f, 0.0f, 0.0f));
  ClearGraphImage(vk_command_buffer, context, RenderResourceNames::frame_ddgi_visibility_atlas,
                  MakeClearColor(1.0f, 0.0f, 0.0f, 1.0f));
  ApplyGraphResourceReleaseBarriers(vk_command_buffer, context, RenderPassQueue::Graphics);
}
}  // namespace

RenderPassDescriptor DdgiAtlasPreparePass::CreateDescriptor() {
  RenderPassDescriptor descriptor{RenderPassNames::ddgi_atlas_prepare, RenderPassQueue::Graphics,
                                  RenderPassScope::Frame};
  descriptor.resources = {{RenderResourceNames::frame_ddgi_probe_metadata, RenderResourceUsage::Write,
                           RenderResourceState::TransferDestination},
                          {RenderResourceNames::frame_ddgi_probe_state, RenderResourceUsage::Write,
                           RenderResourceState::TransferDestination},
                          {RenderResourceNames::frame_ddgi_irradiance_atlas, RenderResourceUsage::Write,
                           RenderResourceState::TransferDestinationGeneral},
                          {RenderResourceNames::frame_ddgi_visibility_atlas, RenderResourceUsage::Write,
                           RenderResourceState::TransferDestinationGeneral}};
  descriptor.profiler_group = RenderPassProfilerGroup::AmbientOcclusionAndDdgi;
  descriptor.profiler_display_name = "DDGI Atlas Prepare";
  return descriptor;
}

void DdgiAtlasPreparePass::Execute(const RenderGraphExecutionContext& context) {
  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    RecordAtlasPrepare(vk_command_buffer, context);
  });
}
