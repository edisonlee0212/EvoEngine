#include "RenderPasses/EntitySelectionHighlightPass.hpp"

#include "Camera.hpp"
#include "GeometryStorage.hpp"
#include "GraphicsPipeline.hpp"
#include "Mesh.hpp"
#include "Platform.hpp"
#include "RenderPasses/RenderPassUtilities.hpp"
#include "Resources.hpp"

using namespace evo_engine;

RenderPassDescriptor EntitySelectionHighlightPass::CreateDescriptor() {
  return {RenderPassNames::entity_selection_highlight,
          RenderPassQueue::Graphics,
          RenderPassScope::Camera,
          {{RenderResourceNames::camera_g_buffer, RenderResourceUsage::Read, RenderResourceState::ShaderRead},
           {RenderResourceNames::camera_color, RenderResourceUsage::ReadWrite, RenderResourceState::ColorAttachment}},
          {RenderPassNames::post_processing}};
}

void EntitySelectionHighlightPass::Execute(const RenderGraphExecutionContext& context, const Parameters& parameters) {
  if (!parameters.record_commands || !parameters.presentation.active || !parameters.camera ||
      !parameters.camera->GetRenderTexture() || !parameters.pipeline || !parameters.pipeline->Initialized() ||
      !parameters.camera->GetGBufferDescriptorSet()) {
    return;
  }
  parameters.record_commands([&](const VkCommandBuffer vk_command_buffer) {
    ApplyGraphResourceBarriers(vk_command_buffer, context);
    std::vector<VkRenderingAttachmentInfo> color_attachments;
    parameters.camera->GetRenderTexture()->AppendColorAttachmentInfos(color_attachments, VK_ATTACHMENT_LOAD_OP_LOAD,
                                                                      VK_ATTACHMENT_STORE_OP_STORE);
    VkRenderingInfo render_info{};
    render_info.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
    render_info.renderArea.extent.width = parameters.camera->GetSize().x;
    render_info.renderArea.extent.height = parameters.camera->GetSize().y;
    render_info.layerCount = 1;
    render_info.colorAttachmentCount = static_cast<uint32_t>(color_attachments.size());
    render_info.pColorAttachments = color_attachments.data();
    Platform::RecordRenderCommands(render_info, vk_command_buffer, [&]() {
      parameters.pipeline->states.ResetAllStates(color_attachments.size());
      parameters.pipeline->states.depth_test = false;
      parameters.pipeline->states.SetViewportScissor(
          {0, 0, static_cast<int>(parameters.camera->GetSize().x), static_cast<int>(parameters.camera->GetSize().y)});
      auto& blend = parameters.pipeline->states.color_blend_attachment_states.front();
      blend.blendEnable = VK_TRUE;
      parameters.pipeline->Bind(vk_command_buffer);
      parameters.pipeline->BindDescriptorSet(vk_command_buffer, 1,
                                             parameters.camera->GetGBufferDescriptorSet()->GetVkDescriptorSet());
      EntitySelectionHighlightPushConstant push_constant;
      push_constant.outline_color = parameters.presentation.outline_color;
      push_constant.settings = {parameters.presentation.fade_progress, parameters.presentation.focus_strength,
                                parameters.presentation.radius, 0.0f};
      parameters.pipeline->PushConstant(vk_command_buffer, 0, push_constant);
      GeometryStorage::BindVertices(vk_command_buffer);
      Resources::GetInstance().GetTexturePassThroughQuad()->DrawIndexed(vk_command_buffer, parameters.pipeline->states,
                                                                        1);
    });
    ApplyGraphResourceReleaseBarriers(vk_command_buffer, context, RenderPassQueue::Graphics);
  });
}
