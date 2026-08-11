#include "RenderPasses/DdgiProbeVariabilityPass.hpp"

#include "ComputePipeline.hpp"
#include "GraphicsResources.hpp"
#include "Platform.hpp"
#include "RenderPasses/DdgiPassUtilities.hpp"
#include "RenderPasses/RenderPassUtilities.hpp"

#include <chrono>

using namespace evo_engine;

namespace {
using Clock = std::chrono::steady_clock;

float ElapsedMilliseconds(const Clock::time_point start) {
  return std::chrono::duration<float, std::milli>(Clock::now() - start).count();
}

std::shared_ptr<DescriptorSet> CreateVariabilityDescriptorSet(const std::shared_ptr<DescriptorSetLayout>& layout,
                                                              const std::shared_ptr<ImageView>& input_view,
                                                              const std::shared_ptr<Buffer>& state_buffer,
                                                              const std::shared_ptr<ImageView>& output_view) {
  const auto descriptor_set = std::make_shared<DescriptorSet>(layout);
  VkDescriptorImageInfo image_info{};
  image_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
  image_info.imageView = input_view->GetVkImageView();
  descriptor_set->UpdateImageDescriptorBinding(0, image_info);
  if (state_buffer) {
    descriptor_set->UpdateBufferDescriptorBinding(1, state_buffer);
  }
  image_info.imageView = output_view->GetVkImageView();
  descriptor_set->UpdateImageDescriptorBinding(2, image_info);
  return descriptor_set;
}

void DispatchReduction(const VkCommandBuffer vk_command_buffer, const std::shared_ptr<ComputePipeline>& pipeline,
                       const std::shared_ptr<DescriptorSetLayout>& descriptor_set_layout,
                       RenderGraphTransientResourceStore& transient_resources,
                       const std::shared_ptr<ImageView>& input_view, const std::shared_ptr<Buffer>& state_buffer,
                       const std::shared_ptr<ImageView>& output_view, const glm::uvec2 input_extent,
                       const glm::uvec2 output_extent, const uint32_t tile_resolution, const uint32_t atlas_columns,
                       const uint32_t probe_count, const float variability_threshold) {
  const auto descriptor_set =
      CreateVariabilityDescriptorSet(descriptor_set_layout, input_view, state_buffer, output_view);
  DdgiProbeVariabilityPushConstant push_constant;
  push_constant.input_output_extent = {glm::max(input_extent.x, 1u), glm::max(input_extent.y, 1u),
                                       glm::max(output_extent.x, 1u), glm::max(output_extent.y, 1u)};
  push_constant.atlas_parameters = {glm::max(tile_resolution, 1u), glm::max(atlas_columns, 1u),
                                    glm::max(probe_count, 1u), glm::floatBitsToUint(variability_threshold)};
  pipeline->Bind(vk_command_buffer);
  pipeline->BindDescriptorSet(vk_command_buffer, 0, descriptor_set->GetVkDescriptorSet());
  pipeline->PushConstant(vk_command_buffer, 0, push_constant);
  pipeline->Dispatch(vk_command_buffer, Platform::DivUp(push_constant.input_output_extent.z, 8),
                     Platform::DivUp(push_constant.input_output_extent.w, 8));
  transient_resources.RetainDescriptorSet(descriptor_set);
}

bool CopyReductionResultToReadback(const VkCommandBuffer vk_command_buffer, const std::shared_ptr<Image>& image,
                                   const std::shared_ptr<Buffer>& readback_buffer) {
  if (!image || !readback_buffer || readback_buffer->GetSize() < sizeof(glm::vec4)) {
    return false;
  }
  ApplyDdgiImageDependency(vk_command_buffer, image, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                           VK_ACCESS_2_SHADER_WRITE_BIT, VK_PIPELINE_STAGE_2_TRANSFER_BIT,
                           VK_ACCESS_2_TRANSFER_READ_BIT);
  VkBufferImageCopy copy_region{};
  copy_region.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
  copy_region.imageSubresource.mipLevel = 0;
  copy_region.imageSubresource.baseArrayLayer = 0;
  copy_region.imageSubresource.layerCount = 1;
  copy_region.imageExtent = {1, 1, 1};
  vkCmdCopyImageToBuffer(vk_command_buffer, image->GetVkImage(), VK_IMAGE_LAYOUT_GENERAL,
                         readback_buffer->GetVkBuffer(), 1, &copy_region);
  return true;
}

void RecordProbeVariability(const VkCommandBuffer vk_command_buffer, const RenderGraphExecutionContext& context,
                            const DdgiProbeVariabilityPass::Parameters& parameters) {
  if (!parameters.reduce_pipeline || !parameters.reduce_pipeline->Initialized() || !parameters.extra_reduce_pipeline ||
      !parameters.extra_reduce_pipeline->Initialized() || !parameters.descriptor_set_layout ||
      !parameters.transient_resources) {
    return;
  }
  const auto* variability_binding = context.GetResourceBinding(RenderResourceNames::frame_ddgi_variability_atlas);
  const auto* state_binding = context.GetResourceBinding(RenderResourceNames::frame_ddgi_probe_state);
  const auto* reduction_a_binding = context.GetResourceBinding(RenderResourceNames::frame_ddgi_variability_reduction_a);
  const auto* reduction_b_binding = context.GetResourceBinding(RenderResourceNames::frame_ddgi_variability_reduction_b);
  if (!variability_binding || !variability_binding->image || !state_binding || !state_binding->buffer ||
      !reduction_a_binding || !reduction_a_binding->image || !reduction_b_binding || !reduction_b_binding->image) {
    return;
  }
  const auto variability_view = CreateGraphImageMipView(variability_binding->image, 0);
  const auto reduction_a_view = CreateGraphImageMipView(reduction_a_binding->image, 0);
  const auto reduction_b_view = CreateGraphImageMipView(reduction_b_binding->image, 0);
  if (!variability_view || !reduction_a_view || !reduction_b_view) {
    return;
  }
  parameters.transient_resources->RetainImageView(variability_view);
  parameters.transient_resources->RetainImageView(reduction_a_view);
  parameters.transient_resources->RetainImageView(reduction_b_view);

  auto input_view = variability_view;
  auto output_view = reduction_a_view;
  auto input_extent = glm::max(parameters.layout.resolution, glm::uvec2(1u));
  auto output_extent = glm::max(parameters.layout.reduction_extent, glm::uvec2(1u));
  ApplyGraphResourceBarriers(vk_command_buffer, context);
  const auto gpu_timestamp = Platform::BeginGpuTimestampScope(vk_command_buffer, "DDGI Variability Reduction");
  DispatchReduction(vk_command_buffer, parameters.reduce_pipeline, parameters.descriptor_set_layout,
                    *parameters.transient_resources, input_view, state_binding->buffer, output_view, input_extent,
                    output_extent, parameters.layout.tile_resolution, parameters.layout.columns,
                    parameters.layout.probe_count, parameters.variability_threshold);

  auto current_input_view = reduction_a_view;
  auto current_output_view = reduction_b_view;
  auto current_input_image = reduction_a_binding->image;
  auto current_output_image = reduction_b_binding->image;
  auto current_input_extent = output_extent;
  while (current_input_extent.x > 1u || current_input_extent.y > 1u) {
    ApplyDdgiImageDependency(vk_command_buffer, current_input_image, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                             VK_ACCESS_2_SHADER_WRITE_BIT, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                             VK_ACCESS_2_SHADER_READ_BIT);
    const glm::uvec2 current_output_extent{glm::max(1u, (current_input_extent.x + 15u) / 16u),
                                           glm::max(1u, (current_input_extent.y + 15u) / 16u)};
    DispatchReduction(vk_command_buffer, parameters.extra_reduce_pipeline, parameters.descriptor_set_layout,
                      *parameters.transient_resources, current_input_view, {}, current_output_view,
                      current_input_extent, current_output_extent, parameters.layout.tile_resolution,
                      parameters.layout.columns, parameters.layout.probe_count, parameters.variability_threshold);
    current_input_view = current_output_view;
    current_input_image = current_output_image;
    current_output_view = current_input_view == reduction_a_view ? reduction_b_view : reduction_a_view;
    current_output_image =
        current_input_image == reduction_a_binding->image ? reduction_b_binding->image : reduction_a_binding->image;
    current_input_extent = current_output_extent;
  }
  if (CopyReductionResultToReadback(vk_command_buffer, current_input_image, parameters.readback_buffer)) {
    parameters.transient_resources->RetainBuffer(parameters.readback_buffer);
    if (parameters.readback_recorded) {
      *parameters.readback_recorded = true;
    }
  }
  Platform::EndGpuTimestampScope(vk_command_buffer, gpu_timestamp);
  ApplyGraphResourceReleaseBarriers(vk_command_buffer, context, RenderPassQueue::Graphics);
}
}  // namespace

RenderPassDescriptor DdgiProbeVariabilityPass::CreateDescriptor() {
  RenderPassDescriptor descriptor{
      RenderPassNames::ddgi_probe_variability,
      RenderPassQueue::Graphics,
      RenderPassScope::Frame,
      {{RenderResourceNames::frame_ddgi_variability_atlas, RenderResourceUsage::Read,
        RenderResourceState::StorageReadWrite},
       {RenderResourceNames::frame_ddgi_probe_state, RenderResourceUsage::Read, RenderResourceState::ShaderRead},
       {RenderResourceNames::frame_ddgi_variability_reduction_a, RenderResourceUsage::ReadWrite,
        RenderResourceState::StorageReadWrite},
       {RenderResourceNames::frame_ddgi_variability_reduction_b, RenderResourceUsage::ReadWrite,
        RenderResourceState::StorageReadWrite}}};
  descriptor.dependencies = {RenderPassNames::ddgi_probe_update};
  return descriptor;
}

void DdgiProbeVariabilityPass::Execute(const RenderGraphExecutionContext& context, const Parameters& parameters) {
  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    const auto timer = Clock::now();
    RecordProbeVariability(vk_command_buffer, context, parameters);
    if (parameters.record_time_ms) {
      *parameters.record_time_ms += ElapsedMilliseconds(timer);
    }
  });
}
