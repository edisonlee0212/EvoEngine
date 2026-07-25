#include "RenderPasses/DdgiGatherTimingPass.hpp"

#include "Camera.hpp"
#include "GeometryStorage.hpp"
#include "GraphicsPipeline.hpp"
#include "GraphicsResources.hpp"
#include "Mesh.hpp"
#include "Platform.hpp"
#include "RenderInstanceStorage.hpp"
#include "RenderPasses/RenderPassUtilities.hpp"
#include "Resources.hpp"

using namespace evo_engine;

RenderPassDescriptor DdgiGatherTimingPass::CreateDescriptor() {
  return {
      RenderPassNames::ddgi_gather_timing,
      RenderPassQueue::Graphics,
      RenderPassScope::Camera,
      {{RenderResourceNames::frame_per_frame_descriptor_set, RenderResourceUsage::Read, RenderResourceState::General},
       {RenderResourceNames::camera_g_buffer, RenderResourceUsage::Read, RenderResourceState::ShaderRead},
       {RenderResourceNames::camera_ddgi_gather_timing, RenderResourceUsage::Write,
        RenderResourceState::ColorAttachment}},
      {RenderPassNames::depth_pyramid}};
}

void DdgiGatherTimingPass::Execute(const RenderGraphExecutionContext& context, const Parameters& parameters) {
  if (!parameters.record_commands) {
    return;
  }
  parameters.record_commands([&](const VkCommandBuffer vk_command_buffer) {
    const auto* output_binding = context.GetResourceBinding(RenderResourceNames::camera_ddgi_gather_timing);
    const auto& g_buffer_descriptor_set = parameters.camera ? parameters.camera->GetGBufferDescriptorSet() : nullptr;
    if (!output_binding || !output_binding->image || !parameters.pipeline || !parameters.pipeline->Initialized() ||
        !parameters.per_frame_descriptor_set || !g_buffer_descriptor_set || !parameters.lighting_descriptor_set ||
        !parameters.raster_lighting_texture_descriptor_set || !parameters.transient_resources ||
        parameters.camera_index < 0) {
      return;
    }

    const auto output_view = CreateGraphImageMipView(output_binding->image, 0);
    if (!output_view) {
      return;
    }
    parameters.transient_resources->RetainImageView(output_view);
    ApplyGraphResourceBarriers(vk_command_buffer, context);
    const auto extent = output_binding->image->GetExtent();
    VkRenderingAttachmentInfo attachment{};
    attachment.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;
    attachment.imageView = output_view->GetVkImageView();
    attachment.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    attachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    attachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    VkRenderingInfo render_info{};
    render_info.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
    render_info.renderArea.extent = {extent.width, extent.height};
    render_info.layerCount = 1;
    render_info.colorAttachmentCount = 1;
    render_info.pColorAttachments = &attachment;
    const glm::ivec4 viewport{0, 0, static_cast<int>(extent.width), static_cast<int>(extent.height)};

    const auto gpu_timestamp = Platform::BeginGpuTimestampScope(vk_command_buffer, "DDGI Final Gather");
    Platform::RecordRenderCommands(render_info, vk_command_buffer, [&] {
      parameters.pipeline->states.ResetAllStates(1);
      parameters.pipeline->states.depth_test = false;
      parameters.pipeline->states.SetViewportScissor(viewport);
      parameters.pipeline->Bind(vk_command_buffer);
      parameters.pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                             parameters.per_frame_descriptor_set->GetVkDescriptorSet());
      parameters.pipeline->BindDescriptorSet(vk_command_buffer, 1, g_buffer_descriptor_set->GetVkDescriptorSet());
      parameters.pipeline->BindDescriptorSet(vk_command_buffer, 2,
                                             parameters.lighting_descriptor_set->GetVkDescriptorSet());
      parameters.pipeline->BindDescriptorSet(vk_command_buffer, 3,
                                             parameters.raster_lighting_texture_descriptor_set->GetVkDescriptorSet());
      RenderInstancePushConstant push_constant{};
      push_constant.camera_index = parameters.camera_index;
      parameters.pipeline->PushConstant(vk_command_buffer, 0, push_constant);
      GeometryStorage::BindVertices(vk_command_buffer);
      Resources::GetInstance().GetTexturePassThroughQuad()->DrawIndexed(vk_command_buffer, parameters.pipeline->states,
                                                                        1);
    });
    Platform::EndGpuTimestampScope(vk_command_buffer, gpu_timestamp);
    ApplyGraphResourceReleaseBarriers(vk_command_buffer, context, RenderPassQueue::Graphics);
  });
}
