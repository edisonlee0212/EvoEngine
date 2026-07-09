#include "RenderPasses/DeferredLightingPass.hpp"

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

RenderPassDescriptor DeferredLightingPass::CreateDescriptor() {
  return {
      RenderPassNames::deferred_camera,
      RenderPassQueue::Graphics,
      RenderPassScope::Camera,
      {{RenderResourceNames::frame_per_frame_descriptor_set, RenderResourceUsage::Read, RenderResourceState::General},
       {RenderResourceNames::lighting_directional_shadow_map, RenderResourceUsage::Read,
        RenderResourceState::ShaderRead},
       {RenderResourceNames::camera_depth, RenderResourceUsage::Read, RenderResourceState::ShaderRead},
       {RenderResourceNames::camera_g_buffer, RenderResourceUsage::Read, RenderResourceState::ShaderRead},
       {RenderResourceNames::camera_depth_pyramid, RenderResourceUsage::Read, RenderResourceState::ShaderRead},
       {RenderResourceNames::camera_color, RenderResourceUsage::Write, RenderResourceState::ColorAttachment}},
      {RenderPassNames::depth_pyramid}};
}

void DeferredLightingPass::Execute(const RenderGraphExecutionContext& context, const Parameters& parameters) {
  if (!parameters.record_commands || !parameters.camera || !parameters.camera->GetRenderTexture()) {
    return;
  }
  parameters.record_commands([&](const VkCommandBuffer vk_command_buffer) {
    VkRect2D render_area;
    render_area.offset = {0, 0};
    render_area.extent.width = parameters.camera->GetSize().x;
    render_area.extent.height = parameters.camera->GetSize().y;
    const glm::ivec4 viewport{0, 0, static_cast<int>(parameters.camera->GetSize().x),
                              static_cast<int>(parameters.camera->GetSize().y)};

    ApplyGraphResourceBarriers(vk_command_buffer, context);

    const auto& g_buffer_descriptor_set = parameters.camera->GetGBufferDescriptorSet();
    if (parameters.pipeline && parameters.pipeline->Initialized() && parameters.per_frame_descriptor_set &&
        g_buffer_descriptor_set && parameters.lighting_descriptor_set &&
        parameters.raster_lighting_texture_descriptor_set) {
      GeometryStorage::BindVertices(vk_command_buffer);
      std::vector<VkRenderingAttachmentInfo> color_attachment_infos;
      parameters.camera->GetRenderTexture()->AppendColorAttachmentInfos(
          color_attachment_infos, VK_ATTACHMENT_LOAD_OP_CLEAR, VK_ATTACHMENT_STORE_OP_STORE);
      VkRenderingInfo render_info{};
      render_info.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
      render_info.renderArea = render_area;
      render_info.layerCount = 1;
      render_info.colorAttachmentCount = color_attachment_infos.size();
      render_info.pColorAttachments = color_attachment_infos.data();
      render_info.pDepthAttachment = VK_NULL_HANDLE;
      Platform::RecordRenderCommands(render_info, vk_command_buffer, [&]() {
        parameters.pipeline->states.ResetAllStates(color_attachment_infos.size());
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
        RenderInstancePushConstant push_constant;
        push_constant.camera_index = parameters.camera_index;
        push_constant.light_split_index =
            parameters.fade_selection ? glm::max(128, 256 - parameters.selection_alpha) : 256;
        push_constant.instance_index = parameters.fade_selection ? 1 : 0;
        parameters.pipeline->PushConstant(vk_command_buffer, 0, push_constant);
        const auto mesh = Resources::GetInstance().GetTexturePassThroughQuad();
        mesh->DrawIndexed(vk_command_buffer, parameters.pipeline->states, 1);
        if (parameters.count_draw_calls) {
          Platform::CountRenderPassDraw(RenderPassDrawBucket::DeferredLighting, RenderDrawCallKind::Direct,
                                        parameters.current_frame_index, mesh->GetTriangleAmount() * 3u);
        }
      });
    }

    if (parameters.external_forward_rendering) {
      parameters.external_forward_rendering(vk_command_buffer, viewport);
    }
    ApplyGraphResourceReleaseBarriers(vk_command_buffer, context, RenderPassQueue::Graphics);
  });
}
