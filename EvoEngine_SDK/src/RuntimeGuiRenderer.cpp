#include "RuntimeGuiRenderer.hpp"
#include <imgui_impl_vulkan.h>
#include <imgui_internal.h>
#include "GeometryStorage.hpp"
#include "GuiTextureRegistry.hpp"
#include "Mesh.hpp"
#include "Platform.hpp"
#include "RenderTexture.hpp"
#include "Resources.hpp"
#include "Shader.hpp"

using namespace evo_engine;

void RuntimeGuiRenderer::BeginView(const ImVec2 origin, const ImVec2 size) {
  if (size.x < 32 || size.y < 32)
    return;
  frame_ = ImGui::GetFrameCount();
  context_.BeginView(origin, size, 1);
  const auto scale = ImGui::GetIO().DisplayFramebufferScale;
  const VkExtent3D extent{static_cast<uint32_t>(size.x * scale.x), static_cast<uint32_t>(size.y * scale.y), 1};
  if (!overlay_ || overlay_->GetExtent().width != extent.width || overlay_->GetExtent().height != extent.height)
    overlay_ = std::make_shared<RenderTexture>(RenderTextureCreateInfo{
        extent, VK_IMAGE_VIEW_TYPE_2D, Platform::Constants::swap_chain_image_format, true, false});
  texture_id_ = GuiTextureRegistry::GetColorTextureId(*overlay_);
  ImGui::GetWindowDrawList()->AddCallback(Composite, this);
  ImGui::GetWindowDrawList()->AddCallback(ImDrawCallback_ResetRenderState, nullptr);
}

void RuntimeGuiRenderer::RenderOverlay() {
  if (frame_ != ImGui::GetFrameCount())
    return;
  context_.PartitionDrawData();
  if (!compositor_) {
    compositor_ = std::make_shared<GraphicsPipeline>();
    compositor_->vertex_shader = Shader::CreateTemporary(
        ShaderType::Vertex, Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Vertex/TexturePassThrough.slang");
    compositor_->fragment_shader =
        Shader::CreateTemporary(ShaderType::Fragment, Resources::GetDefaultResourcesPath() /
                                                          "Shaders/Graphics/Fragment/RuntimeGuiComposite.slang");
    compositor_->geometry_type = GeometryType::Mesh;
    compositor_->vertex_input_attribute_set = VertexInputAttributeSet::PositionTexCoord;
    auto texture_layout = std::make_shared<DescriptorSetLayout>();
    texture_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_FRAGMENT_BIT,
                                          0);
    texture_layout->Initialize();
    compositor_->descriptor_set_layouts.push_back(texture_layout);
    compositor_->depth_attachment_format = VK_FORMAT_UNDEFINED;
    compositor_->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    compositor_->color_attachment_formats = {Platform::Constants::swap_chain_image_format};
    compositor_->Initialize();
  }
  Platform::RecordCommandsMainQueue([this](const VkCommandBuffer commands) {
    Platform::EverythingBarrier(commands);
    overlay_->Render(commands, VK_ATTACHMENT_LOAD_OP_CLEAR, VK_ATTACHMENT_STORE_OP_STORE, [&] {
      ImGui_ImplVulkan_RenderDrawData(context_.GetDrawData(), commands);
    });
    Platform::EverythingBarrier(commands);
  });
}

void RuntimeGuiRenderer::Composite(const ImDrawList*, const ImDrawCmd* command) {
  auto& renderer = *static_cast<RuntimeGuiRenderer*>(command->UserCallbackData);
  if (!renderer.compositor_)
    return;
  const auto* render_state = static_cast<ImGui_ImplVulkan_RenderState*>(ImGui::GetPlatformIO().Renderer_RenderState);
  const auto* data = renderer.context_.GetDrawData();
  const auto origin = renderer.context_.GetOrigin();
  const auto size = renderer.context_.GetSize();
  const auto* viewport = data->OwnerViewport;
  const auto scale = data->FramebufferScale;
  auto& pipeline = *renderer.compositor_;
  pipeline.states.view_port = {(origin.x - viewport->Pos.x) * scale.x,
                               (origin.y - viewport->Pos.y) * scale.y,
                               size.x * scale.x,
                               size.y * scale.y,
                               0,
                               1};
  const float left = std::max(origin.x, command->ClipRect.x);
  const float top = std::max(origin.y, command->ClipRect.y);
  const float right = std::min(origin.x + size.x, command->ClipRect.z);
  const float bottom = std::min(origin.y + size.y, command->ClipRect.w);
  if (right <= left || bottom <= top)
    return;
  pipeline.states.scissor = {
      {static_cast<int32_t>((left - viewport->Pos.x) * scale.x),
       static_cast<int32_t>((top - viewport->Pos.y) * scale.y)},
      {static_cast<uint32_t>((right - left) * scale.x), static_cast<uint32_t>((bottom - top) * scale.y)}};
  pipeline.states.depth_test = pipeline.states.depth_write = false;
  pipeline.states.cull_mode = VK_CULL_MODE_NONE;
  VkPipelineColorBlendAttachmentState blend{};
  blend.blendEnable = VK_TRUE;
  blend.srcColorBlendFactor = blend.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
  blend.dstColorBlendFactor = blend.dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
  blend.colorWriteMask = 15;
  pipeline.states.color_blend_attachment_states = {blend};
  pipeline.Bind(render_state->CommandBuffer);
  pipeline.BindDescriptorSet(render_state->CommandBuffer, 0, reinterpret_cast<VkDescriptorSet>(renderer.texture_id_));
  GeometryStorage::BindVertices(render_state->CommandBuffer);
  pipeline.states.ApplyAllStates(render_state->CommandBuffer, true);
  Resources::GetInstance().GetTexturePassThroughQuad()->DrawIndexed(render_state->CommandBuffer, pipeline.states, 1);
}
