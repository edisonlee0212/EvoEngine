#include "RenderPasses/DdgiProbeUpdatePass.hpp"

#include "ComputePipeline.hpp"
#include "GraphicsResources.hpp"
#include "Platform.hpp"
#include "PointCloudSample.hpp"
#include "RenderPasses/RenderPassUtilities.hpp"

#include <chrono>

using namespace evo_engine;

namespace {
using Clock = std::chrono::steady_clock;

float ElapsedMilliseconds(const Clock::time_point start) {
  return std::chrono::duration<float, std::milli>(Clock::now() - start).count();
}

void RecordProbeUpdate(const VkCommandBuffer vk_command_buffer, const RenderGraphExecutionContext& context,
                       const DdgiProbeUpdatePass::Parameters& parameters) {
  if (!parameters.pipeline || !parameters.pipeline->Initialized() || !parameters.per_frame_descriptor_set ||
      !parameters.descriptor_set_layout || !parameters.transient_resources) {
    return;
  }
  const auto* ray_output_binding = context.GetResourceBinding(RenderResourceNames::frame_ddgi_ray_output);
  const auto* update_indices_binding = context.GetResourceBinding(RenderResourceNames::frame_ddgi_probe_update_indices);
  const auto* irradiance_binding = context.GetResourceBinding(RenderResourceNames::frame_ddgi_irradiance_atlas);
  const auto* visibility_binding = context.GetResourceBinding(RenderResourceNames::frame_ddgi_visibility_atlas);
  const auto* variability_binding = context.GetResourceBinding(RenderResourceNames::frame_ddgi_variability_atlas);
  const auto* metadata_binding = context.GetResourceBinding(RenderResourceNames::frame_ddgi_probe_metadata);
  const auto* state_binding = context.GetResourceBinding(RenderResourceNames::frame_ddgi_probe_state);
  if (!ray_output_binding || !ray_output_binding->buffer || !update_indices_binding ||
      !update_indices_binding->buffer || !irradiance_binding || !irradiance_binding->image || !visibility_binding ||
      !visibility_binding->image || !variability_binding || !variability_binding->image || !metadata_binding ||
      !metadata_binding->buffer || !state_binding || !state_binding->buffer) {
    return;
  }
  ApplyGraphResourceBarriers(vk_command_buffer, context);

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
  descriptor_set->UpdateBufferDescriptorBinding(5, update_indices_binding->buffer);
  image_info.imageView = variability_view->GetVkImageView();
  descriptor_set->UpdateImageDescriptorBinding(6, image_info);

  parameters.pipeline->Bind(vk_command_buffer);
  parameters.pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                         parameters.per_frame_descriptor_set->GetVkDescriptorSet());
  parameters.pipeline->BindDescriptorSet(vk_command_buffer, 1, descriptor_set->GetVkDescriptorSet());
  parameters.pipeline->PushConstant(vk_command_buffer, 0, parameters.push_constant);
  parameters.pipeline->Dispatch(vk_command_buffer,
                                Platform::DivUp(parameters.push_constant.probe_count_ray_count_and_tile_sizes.x, 64));
  if (parameters.metadata_readback_buffer &&
      parameters.metadata_readback_buffer->GetSize() >= metadata_binding->buffer->GetSize() &&
      metadata_binding->buffer->GetSize() != 0) {
    Platform::EverythingBarrier(vk_command_buffer);
    VkBufferCopy copy_region{};
    copy_region.size = metadata_binding->buffer->GetSize();
    vkCmdCopyBuffer(vk_command_buffer, metadata_binding->buffer->GetVkBuffer(),
                    parameters.metadata_readback_buffer->GetVkBuffer(), 1, &copy_region);
  }
  if (parameters.selected_ray_readback_buffer && parameters.selected_ray_sample_count != 0u &&
      parameters.selected_ray_local_probe_index != (std::numeric_limits<uint32_t>::max)()) {
    const auto selected_ray_byte_size = static_cast<VkDeviceSize>(parameters.selected_ray_sample_count) *
                                        static_cast<VkDeviceSize>(sizeof(PointCloudSample));
    const auto selected_ray_src_offset =
        static_cast<VkDeviceSize>(parameters.selected_ray_local_probe_index) * selected_ray_byte_size;
    if (selected_ray_byte_size != 0 && parameters.selected_ray_readback_buffer->GetSize() >= selected_ray_byte_size &&
        ray_output_binding->buffer->GetSize() >= selected_ray_src_offset + selected_ray_byte_size) {
      Platform::EverythingBarrier(vk_command_buffer);
      VkBufferCopy copy_region{};
      copy_region.srcOffset = selected_ray_src_offset;
      copy_region.size = selected_ray_byte_size;
      vkCmdCopyBuffer(vk_command_buffer, ray_output_binding->buffer->GetVkBuffer(),
                      parameters.selected_ray_readback_buffer->GetVkBuffer(), 1, &copy_region);
    }
  }
  parameters.transient_resources->RetainDescriptorSet(descriptor_set);
  Platform::EverythingBarrier(vk_command_buffer);
  ApplyGraphResourceReleaseBarriers(vk_command_buffer, context, RenderPassQueue::Graphics);
}
}  // namespace

RenderPassDescriptor DdgiProbeUpdatePass::CreateDescriptor() {
  RenderPassDescriptor descriptor{
      RenderPassNames::ddgi_probe_update,
      RenderPassQueue::Graphics,
      RenderPassScope::Frame,
      {{RenderResourceNames::frame_ddgi_ray_output, RenderResourceUsage::Read, RenderResourceState::ShaderRead},
       {RenderResourceNames::frame_ddgi_probe_update_indices, RenderResourceUsage::Read,
        RenderResourceState::ShaderRead},
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
