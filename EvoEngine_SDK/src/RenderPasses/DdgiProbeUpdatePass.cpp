#include "RenderPasses/DdgiProbeUpdatePass.hpp"

#include "ComputePipeline.hpp"
#include "GraphicsResources.hpp"
#include "Platform.hpp"
#include "RenderPasses/DdgiPassUtilities.hpp"
#include "RenderPasses/RenderPassUtilities.hpp"

#include <algorithm>

using namespace evo_engine;

namespace {
void DispatchProbeUpdate(const VkCommandBuffer vk_command_buffer, const RenderGraphExecutionContext& context,
                         const DdgiProbeUpdatePass::Parameters& parameters,
                         const std::shared_ptr<ComputePipeline>& pipeline,
                         const std::shared_ptr<DescriptorSet>& descriptor_set, const uint32_t update_mode,
                         const uint32_t group_count_x, const uint32_t group_count_y) {
  pipeline->Bind(vk_command_buffer);
  pipeline->BindDescriptorSet(vk_command_buffer, 0, parameters.per_frame_descriptor_set->GetVkDescriptorSet());
  pipeline->BindDescriptorSet(vk_command_buffer, 1, descriptor_set->GetVkDescriptorSet());
  auto push_constant = parameters.push_constant;
  push_constant.atlas_columns_fixed_ray_count_and_update_mode.w = update_mode;
  push_constant.probe_scroll_offset.w = static_cast<int32_t>(std::max(group_count_x, 1u));
  pipeline->PushConstant(vk_command_buffer, 0, push_constant);
  const RenderPassGpuTimestampScope gpu_timestamp(vk_command_buffer, context, 0, update_mode);
  pipeline->Dispatch(vk_command_buffer, group_count_x, group_count_y);
}

void RecordProbeUpdate(const VkCommandBuffer vk_command_buffer, const RenderGraphExecutionContext& context,
                       const DdgiProbeUpdatePass::Parameters& parameters) {
  if (!parameters.pipeline || !parameters.pipeline->Initialized() || !parameters.per_frame_descriptor_set ||
      !parameters.descriptor_set_layout || !parameters.transient_resources) {
    return;
  }
  const auto* ray_output_binding = context.GetResourceBinding(RenderResourceNames::frame_ddgi_ray_output);
  const auto* irradiance_binding = context.GetResourceBinding(RenderResourceNames::frame_ddgi_irradiance_atlas);
  const auto* visibility_binding = context.GetResourceBinding(RenderResourceNames::frame_ddgi_visibility_atlas);
  const auto* metadata_binding = context.GetResourceBinding(RenderResourceNames::frame_ddgi_probe_metadata);
  const auto* state_binding = context.GetResourceBinding(RenderResourceNames::frame_ddgi_probe_state);
  const auto* sample_info_binding = parameters.use_emissive_sampling
                                        ? context.GetResourceBinding(RenderResourceNames::frame_ddgi_ray_sample_info)
                                        : nullptr;
  if (!ray_output_binding || !ray_output_binding->buffer || !irradiance_binding || !irradiance_binding->image ||
      !visibility_binding || !visibility_binding->image || !metadata_binding || !metadata_binding->buffer ||
      !state_binding || !state_binding->buffer) {
    return;
  }
  if (parameters.use_emissive_sampling && (!sample_info_binding || !sample_info_binding->buffer)) {
    return;
  }
  const auto irradiance_view = CreateGraphImageMipView(irradiance_binding->image, 0);
  const auto visibility_view = CreateGraphImageMipView(visibility_binding->image, 0);
  if (!irradiance_view || !visibility_view) {
    return;
  }
  parameters.transient_resources->RetainImageView(irradiance_view);
  parameters.transient_resources->RetainImageView(visibility_view);

  const auto descriptor_set = std::make_shared<DescriptorSet>(parameters.descriptor_set_layout);
  std::array<std::shared_ptr<Buffer>, DdgiHistoryLayout::BufferCount> history_buffers;
  for (size_t i = 0; i < history_buffers.size(); ++i) {
    const auto* binding = context.GetResourceBinding(RenderResourceNames::frame_ddgi_history[i]);
    if (!binding || !binding->buffer)
      return;
    history_buffers[i] = binding->buffer;
    descriptor_set->UpdateBufferDescriptorBinding(static_cast<uint32_t>(7 + i), binding->buffer);
  }
  descriptor_set->UpdateBufferDescriptorBinding(0, ray_output_binding->buffer);
  VkDescriptorImageInfo image_info{};
  image_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
  image_info.imageView = irradiance_view->GetVkImageView();
  descriptor_set->UpdateImageDescriptorBinding(1, image_info);
  image_info.imageView = visibility_view->GetVkImageView();
  descriptor_set->UpdateImageDescriptorBinding(2, image_info);
  descriptor_set->UpdateBufferDescriptorBinding(3, metadata_binding->buffer);
  descriptor_set->UpdateBufferDescriptorBinding(4, state_binding->buffer);
  descriptor_set->UpdateBufferDescriptorBinding(
      6, parameters.use_emissive_sampling ? sample_info_binding->buffer : state_binding->buffer);

  const auto physical_device = Platform::GetSelectedPhysicalDevice();
  if (!physical_device) {
    return;
  }
  const auto& limits = physical_device->properties.limits;
  const bool parallel_pipelines_ready =
      parameters.parallel_irradiance_pipeline && parameters.parallel_irradiance_pipeline->Initialized() &&
      parameters.parallel_visibility_pipeline && parameters.parallel_visibility_pipeline->Initialized();
  const auto parallel_dispatch = DdgiProbeUpdatePass::CalculateDispatchSize(
      parameters.push_constant.probe_count_ray_count_and_tile_sizes.x, true, limits.maxComputeWorkGroupCount[0],
      limits.maxComputeWorkGroupCount[1]);
  const bool use_parallel = !parameters.invalidate_moved_history && parameters.use_parallel &&
                            parallel_pipelines_ready && parallel_dispatch.valid;
  const auto dispatch = use_parallel ? parallel_dispatch
                                     : DdgiProbeUpdatePass::CalculateDispatchSize(
                                           parameters.push_constant.probe_count_ray_count_and_tile_sizes.x, false,
                                           limits.maxComputeWorkGroupCount[0], limits.maxComputeWorkGroupCount[1]);
  if (!dispatch.valid) {
    return;
  }
  ApplyGraphResourceBarriers(vk_command_buffer, context);
  for (const auto& buffer : history_buffers) {
    ApplyDdgiBufferDependency(
        vk_command_buffer, buffer, VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT,
        VK_ACCESS_2_MEMORY_WRITE_BIT | VK_ACCESS_2_MEMORY_READ_BIT,
        parameters.clear_history ? VK_PIPELINE_STAGE_2_TRANSFER_BIT : VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
        parameters.clear_history ? VK_ACCESS_2_TRANSFER_WRITE_BIT
                                 : VK_ACCESS_2_SHADER_READ_BIT | VK_ACCESS_2_SHADER_WRITE_BIT);
    if (parameters.clear_history) {
      vkCmdFillBuffer(vk_command_buffer, buffer->GetVkBuffer(), 0, VK_WHOLE_SIZE, 0);
      ApplyDdgiBufferDependency(vk_command_buffer, buffer, VK_PIPELINE_STAGE_2_TRANSFER_BIT,
                                VK_ACCESS_2_TRANSFER_WRITE_BIT, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                VK_ACCESS_2_SHADER_READ_BIT | VK_ACCESS_2_SHADER_WRITE_BIT);
    }
  }
  if (parameters.invalidate_moved_history) {
    DispatchProbeUpdate(vk_command_buffer, context, parameters, parameters.pipeline, descriptor_set, 3u, dispatch.x,
                        dispatch.y);
    parameters.transient_resources->RetainDescriptorSet(descriptor_set);
    ApplyGraphResourceReleaseBarriers(vk_command_buffer, context, RenderPassQueue::Graphics);
    return;
  }
  const auto& irradiance_pipeline = use_parallel ? parameters.parallel_irradiance_pipeline : parameters.pipeline;
  const auto& visibility_pipeline = use_parallel ? parameters.parallel_visibility_pipeline : parameters.pipeline;
  DispatchProbeUpdate(vk_command_buffer, context, parameters, irradiance_pipeline, descriptor_set, 1u, dispatch.x,
                      dispatch.y);
  DispatchProbeUpdate(vk_command_buffer, context, parameters, visibility_pipeline, descriptor_set, 2u, dispatch.x,
                      dispatch.y);
  if (parameters.recorded_probe_update_count) {
    *parameters.recorded_probe_update_count += parameters.push_constant.probe_count_ray_count_and_tile_sizes.x;
  }
  if (parameters.metadata_readback_buffer &&
      parameters.metadata_readback_buffer->GetSize() >= metadata_binding->buffer->GetSize() &&
      metadata_binding->buffer->GetSize() != 0) {
    ApplyDdgiBufferDependency(vk_command_buffer, metadata_binding->buffer, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                              VK_ACCESS_2_SHADER_WRITE_BIT, VK_PIPELINE_STAGE_2_TRANSFER_BIT,
                              VK_ACCESS_2_TRANSFER_READ_BIT);
    VkBufferCopy copy_region{};
    copy_region.size = metadata_binding->buffer->GetSize();
    vkCmdCopyBuffer(vk_command_buffer, metadata_binding->buffer->GetVkBuffer(),
                    parameters.metadata_readback_buffer->GetVkBuffer(), 1, &copy_region);
    parameters.transient_resources->RetainBuffer(parameters.metadata_readback_buffer);
    if (parameters.metadata_readback_recorded) {
      *parameters.metadata_readback_recorded = true;
    }
  }
  parameters.transient_resources->RetainDescriptorSet(descriptor_set);
  ApplyGraphResourceReleaseBarriers(vk_command_buffer, context, RenderPassQueue::Graphics);
}
}  // namespace

DdgiProbeUpdatePass::DispatchSize DdgiProbeUpdatePass::CalculateDispatchSize(const uint32_t probe_count,
                                                                             const bool parallel,
                                                                             const uint32_t max_group_count_x,
                                                                             const uint32_t max_group_count_y) {
  if (probe_count == 0u || max_group_count_x == 0u || max_group_count_y == 0u) {
    return {};
  }
  const uint32_t group_count = parallel ? probe_count : probe_count / 64u + (probe_count % 64u != 0u ? 1u : 0u);
  DispatchSize dispatch;
  dispatch.x = std::min(group_count, max_group_count_x);
  dispatch.y = group_count / dispatch.x + (group_count % dispatch.x != 0u ? 1u : 0u);
  dispatch.valid = dispatch.y <= max_group_count_y;
  return dispatch;
}

RenderPassDescriptor DdgiProbeUpdatePass::CreateDescriptor(const bool use_emissive_sampling) {
  RenderPassDescriptor descriptor{
      RenderPassNames::ddgi_probe_update,
      RenderPassQueue::Graphics,
      RenderPassScope::Frame,
      {{RenderResourceNames::frame_ddgi_ray_output, RenderResourceUsage::Read, RenderResourceState::ShaderRead},
       {RenderResourceNames::frame_ddgi_irradiance_atlas, RenderResourceUsage::ReadWrite,
        RenderResourceState::StorageReadWrite},
       {RenderResourceNames::frame_ddgi_visibility_atlas, RenderResourceUsage::ReadWrite,
        RenderResourceState::StorageReadWrite},
       {RenderResourceNames::frame_ddgi_probe_metadata, RenderResourceUsage::Write,
        RenderResourceState::StorageReadWrite},
       {RenderResourceNames::frame_ddgi_probe_state, RenderResourceUsage::Read, RenderResourceState::ShaderRead}}};
  descriptor.dependencies = {RenderPassNames::ddgi_probe_trace};
  for (const auto* resource : RenderResourceNames::frame_ddgi_history)
    descriptor.resources.push_back({resource, RenderResourceUsage::ReadWrite, RenderResourceState::StorageReadWrite});
  descriptor.profiler_group = RenderPassProfilerGroup::AmbientOcclusionAndDdgi;
  descriptor.profiler_display_name = "DDGI Probe Update";
  if (use_emissive_sampling) {
    descriptor.resources.push_back(
        {RenderResourceNames::frame_ddgi_ray_sample_info, RenderResourceUsage::Read, RenderResourceState::ShaderRead});
  }
  return descriptor;
}

RenderPassDescriptor DdgiProbeUpdatePass::CreateHistoryInvalidationDescriptor(const bool relocation,
                                                                              const bool classification) {
  auto descriptor = CreateDescriptor();
  descriptor.name = RenderPassNames::ddgi_history_invalidate;
  descriptor.dependencies = {RenderPassNames::ddgi_probe_update};
  if (relocation)
    descriptor.dependencies.push_back(RenderPassNames::ddgi_probe_relocation);
  if (classification)
    descriptor.dependencies.push_back(RenderPassNames::ddgi_probe_classification);
  descriptor.profiler_display_name = "DDGI History Invalidation";
  return descriptor;
}

void DdgiProbeUpdatePass::Execute(const RenderGraphExecutionContext& context, const Parameters& parameters) {
  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    RecordProbeUpdate(vk_command_buffer, context, parameters);
  });
}
