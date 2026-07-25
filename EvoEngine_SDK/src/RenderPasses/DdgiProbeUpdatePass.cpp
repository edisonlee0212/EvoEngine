#include "RenderPasses/DdgiProbeUpdatePass.hpp"

#include "ComputePipeline.hpp"
#include "GraphicsResources.hpp"
#include "Platform.hpp"
#include "RenderPasses/DdgiPassUtilities.hpp"
#include "RenderPasses/RenderPassUtilities.hpp"

#include <algorithm>
#include <chrono>

using namespace evo_engine;

namespace {
using Clock = std::chrono::steady_clock;

float ElapsedMilliseconds(const Clock::time_point start) {
  return std::chrono::duration<float, std::milli>(Clock::now() - start).count();
}

void DispatchProbeUpdate(const VkCommandBuffer vk_command_buffer, const DdgiProbeUpdatePass::Parameters& parameters,
                         const std::shared_ptr<ComputePipeline>& pipeline,
                         const std::shared_ptr<DescriptorSet>& descriptor_set, const uint32_t update_mode,
                         const std::string& timestamp_name, const uint32_t group_count_x,
                         const uint32_t group_count_y) {
  pipeline->Bind(vk_command_buffer);
  pipeline->BindDescriptorSet(vk_command_buffer, 0, parameters.per_frame_descriptor_set->GetVkDescriptorSet());
  pipeline->BindDescriptorSet(vk_command_buffer, 1, descriptor_set->GetVkDescriptorSet());
  auto push_constant = parameters.push_constant;
  push_constant.atlas_columns_fixed_ray_count_and_update_mode.w = update_mode;
  pipeline->PushConstant(vk_command_buffer, 0, push_constant);
  const auto gpu_timestamp = Platform::BeginGpuTimestampScope(vk_command_buffer, timestamp_name);
  pipeline->Dispatch(vk_command_buffer, group_count_x, group_count_y);
  Platform::EndGpuTimestampScope(vk_command_buffer, gpu_timestamp);
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
  const auto* variability_binding = context.GetResourceBinding(RenderResourceNames::frame_ddgi_variability_atlas);
  const auto* metadata_binding = context.GetResourceBinding(RenderResourceNames::frame_ddgi_probe_metadata);
  const auto* state_binding = context.GetResourceBinding(RenderResourceNames::frame_ddgi_probe_state);
  if (!ray_output_binding || !ray_output_binding->buffer || !irradiance_binding || !irradiance_binding->image ||
      !visibility_binding || !visibility_binding->image || !variability_binding || !variability_binding->image ||
      !metadata_binding || !metadata_binding->buffer || !state_binding || !state_binding->buffer) {
    return;
  }
  const auto irradiance_view = CreateGraphImageMipView(irradiance_binding->image, 0);
  const auto visibility_view = CreateGraphImageMipView(visibility_binding->image, 0);
  const auto variability_view = CreateGraphImageMipView(variability_binding->image, 0);
  if (!irradiance_view || !visibility_view || !variability_view) {
    return;
  }
  parameters.transient_resources->RetainImageView(irradiance_view);
  parameters.transient_resources->RetainImageView(visibility_view);
  parameters.transient_resources->RetainImageView(variability_view);

  const auto descriptor_set = std::make_shared<DescriptorSet>(parameters.descriptor_set_layout);
  descriptor_set->UpdateBufferDescriptorBinding(0, ray_output_binding->buffer);
  VkDescriptorImageInfo image_info{};
  image_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
  image_info.imageView = irradiance_view->GetVkImageView();
  descriptor_set->UpdateImageDescriptorBinding(1, image_info);
  image_info.imageView = visibility_view->GetVkImageView();
  descriptor_set->UpdateImageDescriptorBinding(2, image_info);
  descriptor_set->UpdateBufferDescriptorBinding(3, metadata_binding->buffer);
  descriptor_set->UpdateBufferDescriptorBinding(4, state_binding->buffer);
  image_info.imageView = variability_view->GetVkImageView();
  descriptor_set->UpdateImageDescriptorBinding(5, image_info);

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
  const bool use_parallel = parameters.use_parallel && parallel_pipelines_ready && parallel_dispatch.valid;
  const auto dispatch = use_parallel ? parallel_dispatch
                                     : DdgiProbeUpdatePass::CalculateDispatchSize(
                                           parameters.push_constant.probe_count_ray_count_and_tile_sizes.x, false,
                                           limits.maxComputeWorkGroupCount[0], limits.maxComputeWorkGroupCount[1]);
  if (!dispatch.valid) {
    return;
  }
  ApplyGraphResourceBarriers(vk_command_buffer, context);
  if (parameters.path_reported && !*parameters.path_reported) {
    *parameters.path_reported = true;
    EVOENGINE_LOG(std::string("EVOENGINE_DDGI_PROBE_UPDATE_PATH executed=") + (use_parallel ? "parallel" : "serial") +
                  " probe_count=" + std::to_string(parameters.push_constant.probe_count_ray_count_and_tile_sizes.x) +
                  " groups=" + std::to_string(dispatch.x) + "x" + std::to_string(dispatch.y))
  }
  const auto& irradiance_pipeline = use_parallel ? parameters.parallel_irradiance_pipeline : parameters.pipeline;
  const auto& visibility_pipeline = use_parallel ? parameters.parallel_visibility_pipeline : parameters.pipeline;
  const auto atlas_update_timestamp = Platform::BeginGpuTimestampScope(vk_command_buffer, "DDGI Atlas Update Total");
  DispatchProbeUpdate(vk_command_buffer, parameters, irradiance_pipeline, descriptor_set, 1u, "DDGI Irradiance Update",
                      dispatch.x, dispatch.y);
  DispatchProbeUpdate(vk_command_buffer, parameters, visibility_pipeline, descriptor_set, 2u, "DDGI Visibility Update",
                      dispatch.x, dispatch.y);
  Platform::EndGpuTimestampScope(vk_command_buffer, atlas_update_timestamp);
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

RenderPassDescriptor DdgiProbeUpdatePass::CreateDescriptor() {
  RenderPassDescriptor descriptor{
      RenderPassNames::ddgi_probe_update,
      RenderPassQueue::Graphics,
      RenderPassScope::Frame,
      {{RenderResourceNames::frame_ddgi_ray_output, RenderResourceUsage::Read, RenderResourceState::ShaderRead},
       {RenderResourceNames::frame_ddgi_irradiance_atlas, RenderResourceUsage::ReadWrite,
        RenderResourceState::StorageReadWrite},
       {RenderResourceNames::frame_ddgi_visibility_atlas, RenderResourceUsage::ReadWrite,
        RenderResourceState::StorageReadWrite},
       {RenderResourceNames::frame_ddgi_variability_atlas, RenderResourceUsage::Write,
        RenderResourceState::StorageReadWrite},
       {RenderResourceNames::frame_ddgi_probe_metadata, RenderResourceUsage::Write,
        RenderResourceState::StorageReadWrite},
       {RenderResourceNames::frame_ddgi_probe_state, RenderResourceUsage::Read, RenderResourceState::ShaderRead}}};
  descriptor.dependencies = {RenderPassNames::ddgi_ray_diagnostics};
  return descriptor;
}

void DdgiProbeUpdatePass::Execute(const RenderGraphExecutionContext& context, const Parameters& parameters) {
  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    const auto timer = Clock::now();
    RecordProbeUpdate(vk_command_buffer, context, parameters);
    if (parameters.record_time_ms) {
      *parameters.record_time_ms += ElapsedMilliseconds(timer);
    }
  });
}
