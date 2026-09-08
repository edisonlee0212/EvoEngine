#include "RenderPasses/DdgiProbeTracePass.hpp"

#include "GraphicsResources.hpp"
#include "Platform.hpp"
#include "PointCloudSample.hpp"
#include "RayTracingPipeline.hpp"
#include "RenderPasses/DdgiPassUtilities.hpp"
#include "RenderPasses/RenderPassUtilities.hpp"

using namespace evo_engine;

namespace {
void RecordProbeTrace(const VkCommandBuffer vk_command_buffer, const RenderGraphExecutionContext& context,
                      const DdgiProbeTracePass::Parameters& parameters) {
  if (!parameters.pipeline || !parameters.pipeline->Initialized() || !parameters.per_frame_descriptor_set ||
      !parameters.ray_tracing_descriptor_set || !parameters.ray_output_layout || !parameters.transient_resources ||
      !parameters.atlas_sampler) {
    return;
  }
  const auto* binding = context.GetResourceBinding(RenderResourceNames::frame_ddgi_ray_output);
  const auto* diagnostics_binding =
      context.GetResourceBinding(RenderResourceNames::frame_ddgi_selected_ray_diagnostics);
  const auto* state_binding = context.GetResourceBinding(RenderResourceNames::frame_ddgi_probe_state);
  const auto* irradiance_binding = context.GetResourceBinding(RenderResourceNames::frame_ddgi_irradiance_atlas);
  const auto* visibility_binding = context.GetResourceBinding(RenderResourceNames::frame_ddgi_visibility_atlas);
  const auto* sample_info_binding = parameters.use_emissive_sampling
                                        ? context.GetResourceBinding(RenderResourceNames::frame_ddgi_ray_sample_info)
                                        : nullptr;
  if (!binding || !binding->buffer || !diagnostics_binding || !diagnostics_binding->buffer || !state_binding ||
      !state_binding->buffer || !irradiance_binding || !irradiance_binding->image || !visibility_binding ||
      !visibility_binding->image) {
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

  const auto ray_output_descriptor_set = std::make_shared<DescriptorSet>(parameters.ray_output_layout);
  ray_output_descriptor_set->UpdateBufferDescriptorBinding(0, binding->buffer);
  ray_output_descriptor_set->UpdateBufferDescriptorBinding(1, state_binding->buffer);
  ray_output_descriptor_set->UpdateBufferDescriptorBinding(2, diagnostics_binding->buffer);
  ray_output_descriptor_set->UpdateBufferDescriptorBinding(
      21, parameters.use_emissive_sampling ? sample_info_binding->buffer : binding->buffer);
  VkDescriptorImageInfo atlas_info{};
  atlas_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
  atlas_info.sampler = parameters.atlas_sampler->GetVkSampler();
  for (uint32_t cascade = 0; cascade < parameters.cascades.size(); ++cascade) {
    const auto& resources = parameters.cascades[cascade];
    const auto irradiance = resources.irradiance ? CreateGraphImageMipView(resources.irradiance, 0) : irradiance_view;
    const auto visibility = resources.visibility ? CreateGraphImageMipView(resources.visibility, 0) : visibility_view;
    const auto state = resources.probe_state ? resources.probe_state : state_binding->buffer;
    parameters.transient_resources->RetainImageView(irradiance);
    parameters.transient_resources->RetainImageView(visibility);
    parameters.transient_resources->RetainImage(resources.irradiance);
    parameters.transient_resources->RetainImage(resources.visibility);
    parameters.transient_resources->RetainBuffer(state);
    atlas_info.imageView = irradiance->GetVkImageView();
    ray_output_descriptor_set->UpdateImageDescriptorBinding(17, atlas_info, cascade);
    atlas_info.imageView = visibility->GetVkImageView();
    ray_output_descriptor_set->UpdateImageDescriptorBinding(18, atlas_info, cascade);
    ray_output_descriptor_set->UpdateBufferDescriptorBinding(19, state, cascade);
  }
  ApplyGraphResourceBarriers(vk_command_buffer, context, RenderPassQueue::RayTracing);
  parameters.pipeline->Bind(vk_command_buffer);
  parameters.pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                         parameters.per_frame_descriptor_set->GetVkDescriptorSet());
  parameters.pipeline->BindDescriptorSet(vk_command_buffer, 1,
                                         parameters.ray_tracing_descriptor_set->GetVkDescriptorSet());
  parameters.pipeline->BindDescriptorSet(vk_command_buffer, 2, ray_output_descriptor_set->GetVkDescriptorSet());
  parameters.pipeline->PushConstant(vk_command_buffer, 0, parameters.push_constant);
  const auto sample_count = parameters.probe_update_count * parameters.push_constant.probe_counts_and_ray_count.w;
  {
    const RenderPassGpuTimestampScope gpu_timestamp(vk_command_buffer, context);
    parameters.pipeline->Trace(vk_command_buffer, sample_count, 1, 1);
  }
  const auto selected_ray_byte_size =
      static_cast<VkDeviceSize>(parameters.selected_ray_sample_count) * sizeof(PointCloudSample);
  if (parameters.selected_ray_readback_buffer && selected_ray_byte_size != 0u &&
      parameters.selected_ray_readback_buffer->GetSize() >= selected_ray_byte_size &&
      diagnostics_binding->buffer->GetSize() >= selected_ray_byte_size) {
    ApplyDdgiBufferDependency(vk_command_buffer, diagnostics_binding->buffer,
                              VK_PIPELINE_STAGE_2_RAY_TRACING_SHADER_BIT_KHR, VK_ACCESS_2_SHADER_WRITE_BIT,
                              VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_TRANSFER_READ_BIT, 0,
                              selected_ray_byte_size);
    VkBufferCopy copy_region{};
    copy_region.size = selected_ray_byte_size;
    vkCmdCopyBuffer(vk_command_buffer, diagnostics_binding->buffer->GetVkBuffer(),
                    parameters.selected_ray_readback_buffer->GetVkBuffer(), 1, &copy_region);
    parameters.transient_resources->RetainBuffer(parameters.selected_ray_readback_buffer);
    if (parameters.selected_ray_readback_recorded) {
      *parameters.selected_ray_readback_recorded = true;
    }
  }
  if (parameters.recorded_ray_sample_count) {
    *parameters.recorded_ray_sample_count += sample_count;
  }
  parameters.transient_resources->RetainDescriptorSet(ray_output_descriptor_set);
  ApplyGraphResourceReleaseBarriers(vk_command_buffer, context, RenderPassQueue::RayTracing);
}
}  // namespace

RenderPassDescriptor DdgiProbeTracePass::CreateDescriptor(const bool use_emissive_sampling) {
  RenderPassDescriptor descriptor{
      RenderPassNames::ddgi_probe_trace,
      RenderPassQueue::RayTracing,
      RenderPassScope::Frame,
      {{RenderResourceNames::frame_per_frame_descriptor_set, RenderResourceUsage::Read, RenderResourceState::General},
       {RenderResourceNames::frame_ray_tracing_descriptor_set, RenderResourceUsage::Read, RenderResourceState::General},
       {RenderResourceNames::scene_mesh_tlas, RenderResourceUsage::Read,
        RenderResourceState::AccelerationStructureRead},
       {RenderResourceNames::frame_ddgi_probe_state, RenderResourceUsage::Read, RenderResourceState::ShaderRead},
       {RenderResourceNames::frame_ddgi_irradiance_atlas, RenderResourceUsage::Read, RenderResourceState::General},
       {RenderResourceNames::frame_ddgi_visibility_atlas, RenderResourceUsage::Read, RenderResourceState::General},
       {RenderResourceNames::frame_ddgi_ray_output, RenderResourceUsage::Write, RenderResourceState::StorageReadWrite},
       {RenderResourceNames::frame_ddgi_selected_ray_diagnostics, RenderResourceUsage::Write,
        RenderResourceState::StorageReadWrite}}};
  if (use_emissive_sampling) {
    descriptor.resources.push_back({RenderResourceNames::frame_ddgi_ray_sample_info, RenderResourceUsage::Write,
                                    RenderResourceState::StorageReadWrite});
  }
  descriptor.profiler_group = RenderPassProfilerGroup::AmbientOcclusionAndDdgi;
  descriptor.profiler_display_name = "DDGI Probe Trace";
  return descriptor;
}

void DdgiProbeTracePass::Execute(const RenderGraphExecutionContext& context, const Parameters& parameters) {
  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    RecordProbeTrace(vk_command_buffer, context, parameters);
  });
}
