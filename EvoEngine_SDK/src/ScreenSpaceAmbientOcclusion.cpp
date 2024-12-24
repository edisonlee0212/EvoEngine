#include "PostProcessingStack.hpp"

#include "Application.hpp"
#include "Camera.hpp"
#include "EditorLayer.hpp"
#include "GeometryStorage.hpp"
#include "GraphicsPipeline.hpp"
#include "Mesh.hpp"
#include "Platform.hpp"
#include "RenderLayer.hpp"
#include "Resources.hpp"
#include "Shader.hpp"
using namespace evo_engine;

bool ScreenSpaceAmbientOcclusion::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;

  if (ImGui::DragInt("Kernel size", &kernel_size, 1, 1, 64)) {
    changed = true;
  }
  if (ImGui::DragFloat("Disk radius", &radius, 0.001f, 0.0f, 10.f)) {
    changed = true;
  }
  if (ImGui::DragFloat("Bias", &bias, 0.001f, 0.0f, 1.f)) {
    changed = true;
  }
  if (ImGui::DragFloat("Factor", &factor, 0.01f, 0.0f, 5.f)) {
    changed = true;
  }
  if (ImGui::DragFloat("Intensity", &intensity, 0.01f, 0.0f, 5.f)) {
    changed = true;
  }
  if (ImGui::DragFloat("Avoid distance", &avoid_distance, 0.1f, 0.0f, 100.f)) {
    changed = true;
  }
  if (ImGui::Button("Rebuild pipelines")) {
    BuildPipelines();
  }
  return changed;
}

void ScreenSpaceAmbientOcclusion::Process(const PostProcessingStack& post_processing_stack,
                                          const std::shared_ptr<Camera>& target_camera) {
  if (!geometry_pipeline || !geometry_pipeline->Initialized() || !combine_pipeline || !combine_pipeline->Initialized())
    return;
  const auto render_layer = Application::GetLayer<RenderLayer>();
  {
    VkDescriptorImageInfo image_info;
    image_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
    image_info.imageView = post_processing_stack.source_color_texture->GetColorImageView()->GetVkImageView();
    image_info.sampler = post_processing_stack.source_color_texture->GetColorSampler()->GetVkSampler();
    combine_descriptor_set->UpdateImageDescriptorBinding(0, image_info);
    image_info.imageView = post_processing_stack.result_texture->GetColorImageView()->GetVkImageView();
    image_info.sampler = post_processing_stack.result_texture->GetColorSampler()->GetVkSampler();
    combine_descriptor_set->UpdateImageDescriptorBinding(1, image_info);
  }
  {
    VkDescriptorImageInfo image_info;
    image_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
    image_info.imageView = post_processing_stack.result_texture->GetColorImageView()->GetVkImageView();
    image_info.sampler = post_processing_stack.result_texture->GetColorSampler()->GetVkSampler();
    blur_horizontal_descriptor_set->UpdateImageDescriptorBinding(0, image_info);
  }
  {
    VkDescriptorImageInfo image_info;
    image_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
    image_info.imageView = post_processing_stack.swap_texture->GetColorImageView()->GetVkImageView();
    image_info.sampler = post_processing_stack.swap_texture->GetColorSampler()->GetVkSampler();
    blur_vertical_descriptor_set->UpdateImageDescriptorBinding(0, image_info);
  }
  const auto size = target_camera->GetSize();
  PushConstant push_constant;
  push_constant.kernel_size = kernel_size;
  push_constant.radius = radius;
  push_constant.bias = bias;
  push_constant.factor = factor;
  push_constant.intensity = intensity;
  push_constant.camera_index =
      render_layer->GetCurrentRenderInstanceStorage()->GetCameraIndex(target_camera->GetHandle());
  const auto mesh = Resources::GetResource<Mesh>("PRIMITIVE_TEX_PASS_THROUGH");
  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
#pragma region Viewport and scissor
    VkRect2D render_area;
    render_area.offset = {0, 0};
    render_area.extent.width = size.x;
    render_area.extent.height = size.y;
    VkViewport viewport;
    viewport.x = 0.0f;
    viewport.y = 0.0f;
    viewport.width = size.x;
    viewport.height = size.y;
    viewport.minDepth = 0.0f;
    viewport.maxDepth = 1.0f;

    VkRect2D scissor;
    scissor.offset = {0, 0};
    scissor.extent.width = size.x;
    scissor.extent.height = size.y;
#pragma endregion
    GeometryStorage::BindVertices(vk_command_buffer);

    std::vector<VkRenderingAttachmentInfo> color_attachment_infos;
    VkRenderingInfo render_info2{};
    render_info2.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
    render_info2.renderArea = render_area;
    render_info2.layerCount = 1;
    render_info2.pDepthAttachment = VK_NULL_HANDLE;

    // Input texture
    target_camera->TransitGBufferImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    target_camera->GetRenderTexture()->GetColorImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);
    // Attachments
    post_processing_stack.source_color_texture->GetColorImage()->TransitImageLayout(vk_command_buffer,
                                                                                    VK_IMAGE_LAYOUT_GENERAL);
    post_processing_stack.result_texture->GetColorImage()->TransitImageLayout(vk_command_buffer,
                                                                              VK_IMAGE_LAYOUT_GENERAL);
    post_processing_stack.source_color_texture->AppendColorAttachmentInfos(
        color_attachment_infos, VK_ATTACHMENT_LOAD_OP_DONT_CARE, VK_ATTACHMENT_STORE_OP_STORE);
    post_processing_stack.result_texture->AppendColorAttachmentInfos(
        color_attachment_infos, VK_ATTACHMENT_LOAD_OP_DONT_CARE, VK_ATTACHMENT_STORE_OP_STORE);
    render_info2.colorAttachmentCount = color_attachment_infos.size();
    render_info2.pColorAttachments = color_attachment_infos.data();

    vkCmdBeginRendering(vk_command_buffer, &render_info2);
    geometry_pipeline->states.depth_test = false;
    geometry_pipeline->states.color_blend_attachment_states.clear();
    geometry_pipeline->states.color_blend_attachment_states.resize(color_attachment_infos.size());
    for (auto& i : geometry_pipeline->states.color_blend_attachment_states) {
      i.colorWriteMask =
          VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
      i.blendEnable = VK_FALSE;
    }
    geometry_pipeline->Bind(vk_command_buffer);
    geometry_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                         render_layer->GetPerFrameDescriptorSet()->GetVkDescriptorSet());
    geometry_pipeline->BindDescriptorSet(vk_command_buffer, 1,
                                         target_camera->GetGBufferDescriptorSet()->GetVkDescriptorSet());
    geometry_pipeline->BindDescriptorSet(
        vk_command_buffer, 2, target_camera->GetRenderTexture()->GetColorPresentDescriptorSet()->GetVkDescriptorSet());
    geometry_pipeline->states.view_port = viewport;
    geometry_pipeline->states.scissor = scissor;

    geometry_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
    mesh->DrawIndexed(vk_command_buffer, geometry_pipeline->states, 1);
    vkCmdEndRendering(vk_command_buffer);
  });
  BlurPushConstant blur_push_constant{};
  blur_push_constant.avoid_distance = avoid_distance;
  blur_push_constant.camera_near = target_camera->near_distance;
  blur_push_constant.camera_far = target_camera->far_distance;
  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
#pragma region Viewport and scissor
    VkRect2D render_area;
    render_area.offset = {0, 0};
    render_area.extent.width = size.x;
    render_area.extent.height = size.y;
    VkViewport viewport;
    viewport.x = 0.0f;
    viewport.y = 0.0f;
    viewport.width = size.x;
    viewport.height = size.y;
    viewport.minDepth = 0.0f;
    viewport.maxDepth = 1.0f;

    VkRect2D scissor;
    scissor.offset = {0, 0};
    scissor.extent.width = size.x;
    scissor.extent.height = size.y;
#pragma endregion
    GeometryStorage::BindVertices(vk_command_buffer);
    std::vector<VkRenderingAttachmentInfo> color_attachment_infos;
    color_attachment_infos.clear();
    VkRenderingInfo render_info2{};
    render_info2.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
    render_info2.renderArea = render_area;
    render_info2.layerCount = 1;
    render_info2.pDepthAttachment = VK_NULL_HANDLE;
    post_processing_stack.result_texture->GetColorImage()->TransitImageLayout(vk_command_buffer,
                                                                              VK_IMAGE_LAYOUT_GENERAL);
    post_processing_stack.swap_texture->GetColorImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);

    post_processing_stack.swap_texture->AppendColorAttachmentInfos(color_attachment_infos, VK_ATTACHMENT_LOAD_OP_CLEAR,
                                                                   VK_ATTACHMENT_STORE_OP_STORE);
    render_info2.colorAttachmentCount = color_attachment_infos.size();
    render_info2.pColorAttachments = color_attachment_infos.data();

    {
      vkCmdBeginRendering(vk_command_buffer, &render_info2);
      blur_pipeline->states.depth_test = false;
      blur_pipeline->states.color_blend_attachment_states.clear();
      blur_pipeline->states.color_blend_attachment_states.resize(color_attachment_infos.size());
      for (auto& i : blur_pipeline->states.color_blend_attachment_states) {
        i.colorWriteMask =
            VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
        i.blendEnable = VK_FALSE;
      }
      blur_pipeline->Bind(vk_command_buffer);
      blur_pipeline->BindDescriptorSet(vk_command_buffer, 0, blur_horizontal_descriptor_set->GetVkDescriptorSet());
      blur_pipeline->BindDescriptorSet(vk_command_buffer, 1,
                                       target_camera->GetGBufferDescriptorSet()->GetVkDescriptorSet());
      blur_pipeline->states.view_port = viewport;
      blur_pipeline->states.scissor = scissor;
      blur_push_constant.horizontal = true;
      blur_pipeline->PushConstant(vk_command_buffer, 0, blur_push_constant);
      mesh->DrawIndexed(vk_command_buffer, blur_pipeline->states, 1);
      vkCmdEndRendering(vk_command_buffer);
    }

    color_attachment_infos.clear();
    post_processing_stack.result_texture->AppendColorAttachmentInfos(
        color_attachment_infos, VK_ATTACHMENT_LOAD_OP_CLEAR, VK_ATTACHMENT_STORE_OP_STORE);
    render_info2.colorAttachmentCount = color_attachment_infos.size();
    render_info2.pColorAttachments = color_attachment_infos.data();
    {
      vkCmdBeginRendering(vk_command_buffer, &render_info2);
      blur_pipeline->states.depth_test = false;
      blur_pipeline->states.color_blend_attachment_states.clear();
      blur_pipeline->states.color_blend_attachment_states.resize(color_attachment_infos.size());
      for (auto& i : blur_pipeline->states.color_blend_attachment_states) {
        i.colorWriteMask =
            VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
        i.blendEnable = VK_FALSE;
      }
      blur_pipeline->Bind(vk_command_buffer);
      blur_pipeline->BindDescriptorSet(vk_command_buffer, 0, blur_vertical_descriptor_set->GetVkDescriptorSet());
      blur_pipeline->BindDescriptorSet(vk_command_buffer, 1,
                                       target_camera->GetGBufferDescriptorSet()->GetVkDescriptorSet());
      blur_pipeline->states.view_port = viewport;
      blur_pipeline->states.scissor = scissor;
      blur_push_constant.horizontal = false;
      blur_pipeline->PushConstant(vk_command_buffer, 0, blur_push_constant);
      mesh->DrawIndexed(vk_command_buffer, blur_pipeline->states, 1);
      vkCmdEndRendering(vk_command_buffer);
    }
  });

  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
#pragma region Viewport and scissor
    VkRect2D render_area;
    render_area.offset = {0, 0};
    render_area.extent.width = size.x;
    render_area.extent.height = size.y;
    VkViewport viewport;
    viewport.x = 0.0f;
    viewport.y = 0.0f;
    viewport.width = size.x;
    viewport.height = size.y;
    viewport.minDepth = 0.0f;
    viewport.maxDepth = 1.0f;

    VkRect2D scissor;
    scissor.offset = {0, 0};
    scissor.extent.width = size.x;
    scissor.extent.height = size.y;
#pragma endregion
    GeometryStorage::BindVertices(vk_command_buffer);

    std::vector<VkRenderingAttachmentInfo> color_attachment_infos{};
    VkRenderingInfo render_info2{};
    render_info2.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
    render_info2.renderArea = render_area;
    render_info2.layerCount = 1;
    render_info2.pDepthAttachment = VK_NULL_HANDLE;
    // Input texture
    target_camera->GetRenderTexture()->GetColorImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);
    // Attachments
    post_processing_stack.source_color_texture->GetColorImage()->TransitImageLayout(vk_command_buffer,
                                                                                    VK_IMAGE_LAYOUT_GENERAL);
    post_processing_stack.result_texture->GetColorImage()->TransitImageLayout(vk_command_buffer,
                                                                              VK_IMAGE_LAYOUT_GENERAL);

    target_camera->GetRenderTexture()->AppendColorAttachmentInfos(
        color_attachment_infos, VK_ATTACHMENT_LOAD_OP_DONT_CARE, VK_ATTACHMENT_STORE_OP_STORE);
    render_info2.colorAttachmentCount = color_attachment_infos.size();
    render_info2.pColorAttachments = color_attachment_infos.data();
    {
      vkCmdBeginRendering(vk_command_buffer, &render_info2);
      combine_pipeline->states.depth_test = false;
      combine_pipeline->states.color_blend_attachment_states.clear();
      combine_pipeline->states.color_blend_attachment_states.resize(color_attachment_infos.size());
      for (auto& i : combine_pipeline->states.color_blend_attachment_states) {
        i.colorWriteMask =
            VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
        i.blendEnable = VK_FALSE;
      }
      combine_pipeline->Bind(vk_command_buffer);
      combine_pipeline->BindDescriptorSet(vk_command_buffer, 0, combine_descriptor_set->GetVkDescriptorSet());
      combine_pipeline->states.view_port = viewport;
      combine_pipeline->states.scissor = scissor;
      mesh->DrawIndexed(vk_command_buffer, combine_pipeline->states, 1);
      vkCmdEndRendering(vk_command_buffer);
    }
  });
}

void ScreenSpaceAmbientOcclusion::BuildPipelines() {
  combine_layout = std::make_shared<DescriptorSetLayout>();
  combine_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_FRAGMENT_BIT, 0);
  combine_layout->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_FRAGMENT_BIT, 0);
  combine_layout->Initialize();

  geometry_pipeline = std::make_shared<GraphicsPipeline>();
  geometry_pipeline->vertex_shader =
      Shader::CreateTemporary(ShaderType::Vertex, std::filesystem::path("./DefaultResources") /
                                                      "Shaders/Graphics/Vertex/TexturePassThrough.vert");
  geometry_pipeline->fragment_shader = Shader::CreateTemporary(
      ShaderType::Fragment, Platform::Constants::shader_global_defines,
      std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Fragment/PostProcessing/SSAOGeometry.frag");
  geometry_pipeline->geometry_type = GeometryType::Mesh;
  geometry_pipeline->descriptor_set_layouts.emplace_back(RenderLayer::per_frame_layout);
  geometry_pipeline->descriptor_set_layouts.emplace_back(Camera::g_buffer_layout);
  geometry_pipeline->descriptor_set_layouts.emplace_back(RenderTexture::render_texture_present_layout);
  geometry_pipeline->depth_attachment_format = VK_FORMAT_UNDEFINED;
  geometry_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
  geometry_pipeline->color_attachment_formats = {2, Platform::Constants::render_texture_color};
  auto& ssr_reflect_pipeline_push_constant_range = geometry_pipeline->push_constant_ranges.emplace_back();
  ssr_reflect_pipeline_push_constant_range.size = sizeof(PushConstant);
  ssr_reflect_pipeline_push_constant_range.offset = 0;
  ssr_reflect_pipeline_push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
  geometry_pipeline->Initialize();

  combine_pipeline = std::make_shared<GraphicsPipeline>();
  combine_pipeline->vertex_shader =
      Shader::CreateTemporary(ShaderType::Vertex, std::filesystem::path("./DefaultResources") /
                                                      "Shaders/Graphics/Vertex/TexturePassThrough.vert");
  combine_pipeline->fragment_shader = Shader::CreateTemporary(
      ShaderType::Fragment, Platform::Constants::shader_global_defines,
      std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Fragment/PostProcessing/SSAOCombine.frag");
  combine_pipeline->geometry_type = GeometryType::Mesh;
  combine_pipeline->descriptor_set_layouts.emplace_back(combine_layout);
  combine_pipeline->depth_attachment_format = VK_FORMAT_UNDEFINED;
  combine_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
  combine_pipeline->color_attachment_formats = {1, Platform::Constants::render_texture_color};
  combine_pipeline->Initialize();

  if (!combine_descriptor_set) {
    combine_descriptor_set = std::make_shared<DescriptorSet>(combine_layout);
  }

  if (!blur_layout) {
    blur_layout = std::make_shared<DescriptorSetLayout>();
    blur_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_FRAGMENT_BIT, 0);
    blur_layout->Initialize();
  }

  if (!blur_horizontal_descriptor_set) {
    blur_horizontal_descriptor_set = std::make_shared<DescriptorSet>(blur_layout);
  }
  if (!blur_vertical_descriptor_set) {
    blur_vertical_descriptor_set = std::make_shared<DescriptorSet>(blur_layout);
  }

  if (!blur_pipeline) {
    blur_pipeline = std::make_shared<GraphicsPipeline>();
    blur_pipeline->vertex_shader =
        Shader::CreateTemporary(ShaderType::Vertex, std::filesystem::path("./DefaultResources") /
                                                        "Shaders/Graphics/Vertex/TexturePassThrough.vert");
    blur_pipeline->fragment_shader =
        Shader::CreateTemporary(ShaderType::Fragment, std::filesystem::path("./DefaultResources") /
                                                          "Shaders/Graphics/Fragment/PostProcessing/SSAOBlur.frag");
    blur_pipeline->geometry_type = GeometryType::Mesh;
    blur_pipeline->descriptor_set_layouts.emplace_back(blur_layout);
    blur_pipeline->descriptor_set_layouts.emplace_back(Camera::g_buffer_layout);
    blur_pipeline->depth_attachment_format = VK_FORMAT_UNDEFINED;
    blur_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    blur_pipeline->color_attachment_formats = {1, Platform::Constants::render_texture_color};
    auto& ssr_blur_pipeline_push_constant_range = blur_pipeline->push_constant_ranges.emplace_back();
    ssr_blur_pipeline_push_constant_range.size = sizeof(BlurPushConstant);
    ssr_blur_pipeline_push_constant_range.offset = 0;
    ssr_blur_pipeline_push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    blur_pipeline->Initialize();
  }
}