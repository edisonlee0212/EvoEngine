#include "RenderPasses/DdgiProbeScrollPass.hpp"

#include "ComputePipeline.hpp"
#include "GraphicsResources.hpp"
#include "Platform.hpp"
#include "RenderPasses/RenderPassUtilities.hpp"

#include <chrono>

using namespace evo_engine;

namespace {
using Clock = std::chrono::steady_clock;

float ElapsedMilliseconds(const Clock::time_point start) {
  return std::chrono::duration<float, std::milli>(Clock::now() - start).count();
}

void RecordProbeScroll(const VkCommandBuffer vk_command_buffer, const RenderGraphExecutionContext& context,
                       const DdgiProbeScrollPass::Parameters& parameters) {
  if (!parameters.pipeline || !parameters.pipeline->Initialized() || !parameters.descriptor_set_layout ||
      !parameters.transient_resources) {
    return;
  }
  const auto* irradiance_binding = context.GetResourceBinding(RenderResourceNames::frame_ddgi_irradiance_atlas);
  const auto* visibility_binding = context.GetResourceBinding(RenderResourceNames::frame_ddgi_visibility_atlas);
  const auto* variability_binding = context.GetResourceBinding(RenderResourceNames::frame_ddgi_variability_atlas);
  const auto* metadata_binding = context.GetResourceBinding(RenderResourceNames::frame_ddgi_probe_metadata);
  const auto* state_binding = context.GetResourceBinding(RenderResourceNames::frame_ddgi_probe_state);
  if (!irradiance_binding || !irradiance_binding->image || !visibility_binding || !visibility_binding->image ||
      !variability_binding || !variability_binding->image || !metadata_binding || !metadata_binding->buffer ||
      !state_binding || !state_binding->buffer) {
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

  ApplyGraphResourceBarriers(vk_command_buffer, context);
  parameters.pipeline->Bind(vk_command_buffer);
  parameters.pipeline->BindDescriptorSet(vk_command_buffer, 0, descriptor_set->GetVkDescriptorSet());
  parameters.pipeline->PushConstant(vk_command_buffer, 0, parameters.push_constant);
  parameters.pipeline->Dispatch(
      vk_command_buffer, Platform::DivUp(parameters.push_constant.atlas_columns_visibility_tile_and_probe_count.w, 64));
  parameters.transient_resources->RetainDescriptorSet(descriptor_set);
  ApplyGraphResourceReleaseBarriers(vk_command_buffer, context, RenderPassQueue::Graphics);
}
}  // namespace

RenderPassDescriptor DdgiProbeScrollPass::CreateDescriptor() {
  RenderPassDescriptor descriptor{RenderPassNames::ddgi_probe_scroll,
                                  RenderPassQueue::Graphics,
                                  RenderPassScope::Frame,
                                  {{RenderResourceNames::frame_ddgi_irradiance_atlas, RenderResourceUsage::Write,
                                    RenderResourceState::StorageReadWrite},
                                   {RenderResourceNames::frame_ddgi_visibility_atlas, RenderResourceUsage::Write,
                                    RenderResourceState::StorageReadWrite},
                                   {RenderResourceNames::frame_ddgi_variability_atlas, RenderResourceUsage::Write,
                                    RenderResourceState::StorageReadWrite},
                                   {RenderResourceNames::frame_ddgi_probe_metadata, RenderResourceUsage::Write,
                                    RenderResourceState::StorageReadWrite},
                                   {RenderResourceNames::frame_ddgi_probe_state, RenderResourceUsage::Write,
                                    RenderResourceState::StorageReadWrite}}};
  return descriptor;
}

void DdgiProbeScrollPass::Execute(const RenderGraphExecutionContext& context, const Parameters& parameters) {
  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    const auto timer = Clock::now();
    RecordProbeScroll(vk_command_buffer, context, parameters);
    if (parameters.record_time_ms) {
      *parameters.record_time_ms += ElapsedMilliseconds(timer);
    }
  });
}
