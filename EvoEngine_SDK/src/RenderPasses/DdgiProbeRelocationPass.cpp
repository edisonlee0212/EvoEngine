#include "RenderPasses/DdgiProbeRelocationPass.hpp"

#include "ComputePipeline.hpp"
#include "GraphicsResources.hpp"
#include "Platform.hpp"
#include "RenderPasses/DdgiPassUtilities.hpp"
#include "RenderPasses/RenderPassUtilities.hpp"

using namespace evo_engine;

namespace {
void DispatchRelocation(const VkCommandBuffer vk_command_buffer, const DdgiProbeRelocationPass::Parameters& parameters,
                        const DdgiProbeRelocationPushConstant& push_constant) {
  const auto probe_count = push_constant.probe_count_ray_count_and_flags.x;
  if (probe_count == 0u) {
    return;
  }
  parameters.pipeline->PushConstant(vk_command_buffer, 0, push_constant);
  parameters.pipeline->Dispatch(vk_command_buffer, Platform::DivUp(probe_count, 32));
}

void RecordProbeRelocation(const VkCommandBuffer vk_command_buffer, const RenderGraphExecutionContext& context,
                           const DdgiProbeRelocationPass::Parameters& parameters) {
  if (!parameters.pipeline || !parameters.pipeline->Initialized() || !parameters.descriptor_set_layout ||
      !parameters.transient_resources || (!parameters.reset_offsets && !parameters.relocate_probes)) {
    return;
  }
  const auto* ray_output_binding = context.GetResourceBinding(RenderResourceNames::frame_ddgi_ray_output);
  const auto* state_binding = context.GetResourceBinding(RenderResourceNames::frame_ddgi_probe_state);
  if (!ray_output_binding || !ray_output_binding->buffer || !state_binding || !state_binding->buffer) {
    return;
  }
  ApplyGraphResourceBarriers(vk_command_buffer, context);
  const RenderPassGpuTimestampScope gpu_timestamp(vk_command_buffer, context);

  const auto descriptor_set = std::make_shared<DescriptorSet>(parameters.descriptor_set_layout);
  descriptor_set->UpdateBufferDescriptorBinding(0, ray_output_binding->buffer);
  descriptor_set->UpdateBufferDescriptorBinding(1, state_binding->buffer);

  parameters.pipeline->Bind(vk_command_buffer);
  parameters.pipeline->BindDescriptorSet(vk_command_buffer, 0, descriptor_set->GetVkDescriptorSet());
  if (parameters.reset_offsets) {
    DispatchRelocation(vk_command_buffer, parameters, parameters.reset_push_constant);
  }
  if (parameters.reset_offsets && parameters.relocate_probes) {
    ApplyDdgiBufferDependency(vk_command_buffer, state_binding->buffer, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                              VK_ACCESS_2_SHADER_WRITE_BIT, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                              VK_ACCESS_2_SHADER_READ_BIT | VK_ACCESS_2_SHADER_WRITE_BIT);
  }
  if (parameters.relocate_probes) {
    DispatchRelocation(vk_command_buffer, parameters, parameters.update_push_constant);
  }
  parameters.transient_resources->RetainDescriptorSet(descriptor_set);
  ApplyGraphResourceReleaseBarriers(vk_command_buffer, context, RenderPassQueue::Graphics);
}
}  // namespace

RenderPassDescriptor DdgiProbeRelocationPass::CreateDescriptor() {
  RenderPassDescriptor descriptor{
      RenderPassNames::ddgi_probe_relocation,
      RenderPassQueue::Graphics,
      RenderPassScope::Frame,
      {{RenderResourceNames::frame_ddgi_ray_output, RenderResourceUsage::Read, RenderResourceState::ShaderRead},
       {RenderResourceNames::frame_ddgi_probe_state, RenderResourceUsage::ReadWrite,
        RenderResourceState::StorageReadWrite}}};
  descriptor.dependencies = {RenderPassNames::ddgi_probe_update};
  descriptor.profiler_group = RenderPassProfilerGroup::AmbientOcclusionAndDdgi;
  descriptor.profiler_display_name = "DDGI Probe Relocation";
  return descriptor;
}

void DdgiProbeRelocationPass::Execute(const RenderGraphExecutionContext& context, const Parameters& parameters) {
  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    RecordProbeRelocation(vk_command_buffer, context, parameters);
  });
}
