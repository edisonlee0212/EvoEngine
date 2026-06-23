#include "RenderPasses/DdgiProbeRayVisualizationPass.hpp"

#include "Camera.hpp"
#include "GraphicsPipeline.hpp"
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

void RecordProbeRayVisualization(const VkCommandBuffer vk_command_buffer, const RenderGraphExecutionContext& context,
                                 const DdgiProbeRayVisualizationPass::Parameters& parameters) {
  if (!parameters.pipeline || !parameters.pipeline->Initialized() || !parameters.per_frame_descriptor_set ||
      !parameters.descriptor_set_layout || !parameters.transient_resources || !parameters.camera ||
      !parameters.camera->GetRenderTexture() ||
      parameters.push_constant.camera_selected_probe_ray_count_flags.z == 0u) {
    return;
  }
  const auto* ray_output_binding = context.GetResourceBinding(RenderResourceNames::frame_ddgi_ray_output);
  if (!ray_output_binding || !ray_output_binding->buffer) {
    return;
  }

  const auto descriptor_set = std::make_shared<DescriptorSet>(parameters.descriptor_set_layout);
  descriptor_set->UpdateBufferDescriptorBinding(0, ray_output_binding->buffer);
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
    parameters.pipeline->states.line_width = 1.0f;
    if (!parameters.pipeline->states.color_blend_attachment_states.empty()) {
      parameters.pipeline->states.color_blend_attachment_states[0].blendEnable = VK_TRUE;
    }

    parameters.pipeline->Bind(vk_command_buffer);
    parameters.pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                           parameters.per_frame_descriptor_set->GetVkDescriptorSet());
    parameters.pipeline->BindDescriptorSet(vk_command_buffer, 1, descriptor_set->GetVkDescriptorSet());
    auto push_constant = parameters.push_constant;
    push_constant.camera_selected_probe_ray_count_flags.x = parameters.camera_index;
    parameters.pipeline->PushConstant(vk_command_buffer, 0, push_constant);
    vkCmdDraw(vk_command_buffer, push_constant.camera_selected_probe_ray_count_flags.z * 2u, 1, 0, 0);
  });
  ApplyGraphResourceReleaseBarriers(vk_command_buffer, context, RenderPassQueue::Graphics);
}
}  // namespace

RenderPassDescriptor DdgiProbeRayVisualizationPass::CreateDescriptor(const char* dependency) {
  RenderPassDescriptor descriptor{
      RenderPassNames::ddgi_probe_ray_visualization,
      RenderPassQueue::Graphics,
      RenderPassScope::Camera,
      {{RenderResourceNames::frame_per_frame_descriptor_set, RenderResourceUsage::Read, RenderResourceState::General},
       {RenderResourceNames::camera_color, RenderResourceUsage::ReadWrite, RenderResourceState::ColorAttachment},
       {RenderResourceNames::camera_depth, RenderResourceUsage::Read, RenderResourceState::DepthAttachment},
       {RenderResourceNames::frame_ddgi_ray_output, RenderResourceUsage::Read, RenderResourceState::ShaderRead}}};
  descriptor.dependencies = {dependency ? dependency : RenderPassNames::deferred_camera};
  return descriptor;
}

void DdgiProbeRayVisualizationPass::Execute(const RenderGraphExecutionContext& context, const Parameters& parameters) {
  if (!parameters.record_commands) {
    return;
  }
  parameters.record_commands([&](const VkCommandBuffer vk_command_buffer) {
    const auto timer = Clock::now();
    RecordProbeRayVisualization(vk_command_buffer, context, parameters);
    if (parameters.record_time_ms) {
      *parameters.record_time_ms += ElapsedMilliseconds(timer);
    }
  });
}
