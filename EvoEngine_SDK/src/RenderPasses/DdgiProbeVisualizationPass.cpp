#include "RenderPasses/DdgiProbeVisualizationPass.hpp"

#include "Camera.hpp"
#include "GeometryStorage.hpp"
#include "GraphicsPipeline.hpp"
#include "GraphicsResources.hpp"
#include "Mesh.hpp"
#include "RenderPasses/RenderPassUtilities.hpp"
#include "Resources.hpp"

#include <chrono>

using namespace evo_engine;

namespace {
using Clock = std::chrono::steady_clock;

float ElapsedMilliseconds(const Clock::time_point start) {
  return std::chrono::duration<float, std::milli>(Clock::now() - start).count();
}

void RecordProbeVisualization(const VkCommandBuffer vk_command_buffer, const RenderGraphExecutionContext& context,
                              const DdgiProbeVisualizationPass::Parameters& parameters) {
  if (!parameters.pipeline || !parameters.pipeline->Initialized() || !parameters.per_frame_descriptor_set ||
      !parameters.descriptor_set_layout || !parameters.transient_resources || !parameters.camera ||
      !parameters.camera->GetRenderTexture() || !parameters.atlas_sampler || parameters.probe_count == 0u) {
    return;
  }
  const auto* metadata_binding = context.GetResourceBinding(RenderResourceNames::frame_ddgi_probe_metadata);
  const auto* state_binding = context.GetResourceBinding(RenderResourceNames::frame_ddgi_probe_state);
  const auto* irradiance_binding = context.GetResourceBinding(RenderResourceNames::frame_ddgi_irradiance_atlas);
  if (!metadata_binding || !metadata_binding->buffer || !state_binding || !state_binding->buffer ||
      !irradiance_binding || !irradiance_binding->image) {
    return;
  }

  const auto irradiance_view = CreateGraphImageMipView(irradiance_binding->image, 0);
  if (!irradiance_view) {
    return;
  }
  parameters.transient_resources->RetainImageView(irradiance_view);

  const auto descriptor_set = std::make_shared<DescriptorSet>(parameters.descriptor_set_layout);
  descriptor_set->UpdateBufferDescriptorBinding(0, metadata_binding->buffer);
  descriptor_set->UpdateBufferDescriptorBinding(1, state_binding->buffer);
  VkDescriptorImageInfo image_info{};
  image_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
  image_info.sampler = parameters.atlas_sampler->GetVkSampler();
  image_info.imageView = irradiance_view->GetVkImageView();
  descriptor_set->UpdateImageDescriptorBinding(17, image_info);
  parameters.transient_resources->RetainDescriptorSet(descriptor_set);

  ApplyGraphResourceBarriers(vk_command_buffer, context);
  std::vector<VkRenderingAttachmentInfo> color_attachment_infos;
  parameters.camera->GetRenderTexture()->AppendColorAttachmentInfos(color_attachment_infos, VK_ATTACHMENT_LOAD_OP_LOAD,
                                                                    VK_ATTACHMENT_STORE_OP_STORE);
  auto depth_attachment = parameters.camera->GetRenderTexture()->GetDepthAttachmentInfo(VK_ATTACHMENT_LOAD_OP_LOAD,
                                                                                        VK_ATTACHMENT_STORE_OP_STORE);

  VkRect2D render_area{};
  render_area.offset = {0, 0};
  render_area.extent.width = parameters.camera->GetSize().x;
  render_area.extent.height = parameters.camera->GetSize().y;
  VkRenderingInfo render_info{};
  render_info.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
  render_info.renderArea = render_area;
  render_info.layerCount = 1;
  render_info.colorAttachmentCount = static_cast<uint32_t>(color_attachment_infos.size());
  render_info.pColorAttachments = color_attachment_infos.data();
  render_info.pDepthAttachment = &depth_attachment;

  Platform::RecordRenderCommands(render_info, vk_command_buffer, [&]() {
    const glm::ivec4 viewport{0, 0, static_cast<int>(parameters.camera->GetSize().x),
                              static_cast<int>(parameters.camera->GetSize().y)};
    parameters.pipeline->states.ResetAllStates(color_attachment_infos.size());
    parameters.pipeline->states.SetViewportScissor(viewport);
    parameters.pipeline->states.depth_test = parameters.depth_test;
    parameters.pipeline->states.depth_write = false;
    parameters.pipeline->states.depth_compare = VK_COMPARE_OP_LESS_OR_EQUAL;
    parameters.pipeline->states.cull_mode = VK_CULL_MODE_NONE;
    if (!parameters.pipeline->states.color_blend_attachment_states.empty()) {
      parameters.pipeline->states.color_blend_attachment_states[0].blendEnable = VK_TRUE;
    }

    parameters.pipeline->Bind(vk_command_buffer);
    parameters.pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                           parameters.per_frame_descriptor_set->GetVkDescriptorSet());
    parameters.pipeline->BindDescriptorSet(vk_command_buffer, 1, descriptor_set->GetVkDescriptorSet());
    parameters.pipeline->PushConstant(vk_command_buffer, 0, parameters.push_constant);
    GeometryStorage::BindVertices(vk_command_buffer);
    Resources::GetInstance().GetPrimitives().sphere->DrawIndexed(vk_command_buffer, parameters.pipeline->states,
                                                                 parameters.probe_count);
  });
  ApplyGraphResourceReleaseBarriers(vk_command_buffer, context, RenderPassQueue::Graphics);
}
}  // namespace

RenderPassDescriptor DdgiProbeVisualizationPass::CreateDescriptor() {
  return {
      RenderPassNames::ddgi_probe_visualization,
      RenderPassQueue::Graphics,
      RenderPassScope::Camera,
      {{RenderResourceNames::frame_per_frame_descriptor_set, RenderResourceUsage::Read, RenderResourceState::General},
       {RenderResourceNames::camera_color, RenderResourceUsage::ReadWrite, RenderResourceState::ColorAttachment},
       {RenderResourceNames::camera_depth, RenderResourceUsage::Read, RenderResourceState::DepthAttachment},
       {RenderResourceNames::frame_ddgi_probe_metadata, RenderResourceUsage::Read, RenderResourceState::ShaderRead},
       {RenderResourceNames::frame_ddgi_probe_state, RenderResourceUsage::Read, RenderResourceState::ShaderRead},
       {RenderResourceNames::frame_ddgi_irradiance_atlas, RenderResourceUsage::Read, RenderResourceState::General}},
      {RenderPassNames::deferred_camera}};
}

void DdgiProbeVisualizationPass::Execute(const RenderGraphExecutionContext& context, const Parameters& parameters) {
  if (!parameters.record_commands) {
    return;
  }
  parameters.record_commands([&](const VkCommandBuffer vk_command_buffer) {
    const auto timer = Clock::now();
    RecordProbeVisualization(vk_command_buffer, context, parameters);
    if (parameters.record_time_ms) {
      *parameters.record_time_ms += ElapsedMilliseconds(timer);
    }
  });
}
