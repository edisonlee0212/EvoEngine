#include "RenderPasses/MotionVectorPass.hpp"

#include "Camera.hpp"
#include "ComputePipeline.hpp"
#include "GraphicsResources.hpp"
#include "Platform.hpp"
#include "RenderInstanceStorage.hpp"
#include "RenderPasses/RenderPassUtilities.hpp"
#include "RenderTexture.hpp"

using namespace evo_engine;

namespace {
struct MotionVectorPushConstant {
  int32_t camera_index = 0;
  int32_t instance_count = 0;
};

void ClearMotionVectors(const VkCommandBuffer vk_command_buffer, const std::shared_ptr<Image>& motion_vectors) {
  VkClearColorValue clear_value{};
  clear_value.float32[0] = 0.0f;
  clear_value.float32[1] = 0.0f;
  clear_value.float32[2] = 0.0f;
  clear_value.float32[3] = 0.0f;
  ClearGraphColorImage(vk_command_buffer, motion_vectors, clear_value);
}
}  // namespace

RenderPassDescriptor MotionVectorPass::CreateDescriptor() {
  return {
      RenderPassNames::motion_vectors,
      RenderPassQueue::Graphics,
      RenderPassScope::Camera,
      {{RenderResourceNames::camera_depth, RenderResourceUsage::Read, RenderResourceState::ShaderRead},
       {RenderResourceNames::camera_g_buffer, RenderResourceUsage::Read, RenderResourceState::ShaderRead},
       {RenderResourceNames::camera_motion_vectors, RenderResourceUsage::Write, RenderResourceState::StorageReadWrite}},
      {RenderPassNames::deferred_geometry}};
}

void MotionVectorPass::Execute(const RenderGraphExecutionContext& context, const Parameters& parameters) {
  if (!parameters.record_commands) {
    return;
  }
  parameters.record_commands([&](const VkCommandBuffer vk_command_buffer) {
    ApplyGraphResourceBarriers(vk_command_buffer, context);
    const auto* motion_binding = context.GetResourceBinding(RenderResourceNames::camera_motion_vectors);
    const auto release_barriers = [&] {
      ApplyGraphResourceReleaseBarriers(vk_command_buffer, context, RenderPassQueue::Graphics);
    };
    if (!motion_binding || !motion_binding->image) {
      release_barriers();
      return;
    }
    const auto motion_vectors = motion_binding->image;
    if (!parameters.camera || !parameters.render_instances || !parameters.per_frame_descriptor_set ||
        !parameters.pipeline || !parameters.pipeline->Initialized() || !parameters.descriptor_set_layout ||
        !parameters.transient_resources || !parameters.render_instances->previous_instance_info_descriptor_buffer ||
        parameters.render_instances->GetPreviousInstanceInfoBlocks().empty()) {
      ClearMotionVectors(vk_command_buffer, motion_vectors);
      release_barriers();
      return;
    }
    const auto extent = motion_vectors->GetExtent();
    if (extent.width == 0 || extent.height == 0) {
      release_barriers();
      return;
    }
    auto motion_vectors_view = CreateGraphImageMipView(motion_vectors, 0);
    parameters.transient_resources->RetainImageView(motion_vectors_view);

    const auto descriptor_set = std::make_shared<DescriptorSet>(parameters.descriptor_set_layout);
    VkDescriptorImageInfo image_info{};
    image_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
    image_info.imageView = motion_vectors_view->GetVkImageView();
    image_info.sampler = VK_NULL_HANDLE;
    descriptor_set->UpdateImageDescriptorBinding(0, image_info);
    descriptor_set->UpdateBufferDescriptorBinding(
        1, parameters.render_instances->previous_instance_info_descriptor_buffer);

    MotionVectorPushConstant push_constant{};
    push_constant.camera_index = parameters.camera_index;
    push_constant.instance_count = static_cast<int32_t>(parameters.render_instances->GetInstanceInfoBlocks().size());

    parameters.pipeline->Bind(vk_command_buffer);
    parameters.pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                           parameters.per_frame_descriptor_set->GetVkDescriptorSet());
    parameters.pipeline->BindDescriptorSet(vk_command_buffer, 1,
                                           parameters.camera->GetGBufferDescriptorSet()->GetVkDescriptorSet());
    parameters.pipeline->BindDescriptorSet(vk_command_buffer, 2, descriptor_set->GetVkDescriptorSet());
    parameters.pipeline->PushConstant(vk_command_buffer, 0, push_constant);
    parameters.pipeline->Dispatch(vk_command_buffer, Platform::DivUp(extent.width, 16),
                                  Platform::DivUp(extent.height, 16));
    parameters.transient_resources->RetainDescriptorSet(descriptor_set);
    Platform::EverythingBarrier(vk_command_buffer);
    release_barriers();
  });
}
