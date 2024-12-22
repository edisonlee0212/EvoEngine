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

bool ScreenSpaceReflection::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::DragFloat("Max distance", &max_distance, 0.01f, 0.01f, 1.0f))
    changed = false;
  if (ImGui::DragFloat("Resolution", &resolution, 0.01f, 0.0f, 1.0f))
    changed = false;
  if (ImGui::DragInt("Steps", &initial_steps, 1, 1, 16))
    changed = false;
  if (ImGui::DragFloat("Thickness", &thickness, 0.01f, 0.0f, 1.0f))
    changed = false;

  if (ImGui::Button("Rebuild pipelines")) {
    BuildPipelines();
  }

  return changed;
}

void ScreenSpaceReflection::Process(const std::shared_ptr<RenderTexture>& render_texture0,
                                    const std::shared_ptr<RenderTexture>& render_texture1,
                                    const std::shared_ptr<RenderTexture>& render_texture2,
                                    const std::shared_ptr<Camera>& target_camera) const {
  if (!ssr_reflect_pipeline || !ssr_blur_pipeline || !ssr_combine_pipeline || !ssr_reflect_pipeline->Initialized() ||
      !ssr_blur_pipeline->Initialized() || !ssr_combine_pipeline->Initialized())
    return;
  const auto render_layer = Application::GetLayer<RenderLayer>();
  if (!ssr_reflect_descriptor_set) {
    ssr_reflect_descriptor_set = std::make_shared<DescriptorSet>(ssr_reflect_layout);
  }
  if (!ssr_blur_horizontal_descriptor_set) {
    ssr_blur_horizontal_descriptor_set = std::make_shared<DescriptorSet>(ssr_blur_layout);
  }
  if (!ssr_blur_vertical_descriptor_set) {
    ssr_blur_vertical_descriptor_set = std::make_shared<DescriptorSet>(ssr_blur_layout);
  }
  if (!ssr_combine_descriptor_set) {
    ssr_combine_descriptor_set = std::make_shared<DescriptorSet>(ssr_combine_layout);
  }
  {
    VkDescriptorImageInfo image_info;
    image_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
    image_info.imageView = target_camera->GetRenderTexture()->GetDepthImageView()->GetVkImageView();
    image_info.sampler = target_camera->GetRenderTexture()->GetDepthSampler()->GetVkSampler();
    ssr_reflect_descriptor_set->UpdateImageDescriptorBinding(17, image_info);
    image_info.imageView = target_camera->g_buffer_normal_view_->GetVkImageView();
    image_info.sampler = target_camera->g_buffer_normal_sampler_->GetVkSampler();
    ssr_reflect_descriptor_set->UpdateImageDescriptorBinding(18, image_info);
    image_info.imageView = target_camera->g_buffer_material_view_->GetVkImageView();
    image_info.sampler = target_camera->g_buffer_material_sampler_->GetVkSampler();
    ssr_reflect_descriptor_set->UpdateImageDescriptorBinding(19, image_info);

    image_info.imageView = target_camera->GetRenderTexture()->GetColorImageView()->GetVkImageView();
    image_info.sampler = target_camera->GetRenderTexture()->GetColorSampler()->GetVkSampler();
    ssr_reflect_descriptor_set->UpdateImageDescriptorBinding(20, image_info);
  }
  {
    VkDescriptorImageInfo image_info;
    image_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
    image_info.imageView = render_texture1->GetColorImageView()->GetVkImageView();
    image_info.sampler = render_texture1->GetColorSampler()->GetVkSampler();
    ssr_blur_horizontal_descriptor_set->UpdateImageDescriptorBinding(0, image_info);
  }
  {
    VkDescriptorImageInfo image_info;
    image_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
    image_info.imageView = render_texture2->GetColorImageView()->GetVkImageView();
    image_info.sampler = render_texture2->GetColorSampler()->GetVkSampler();
    ssr_blur_vertical_descriptor_set->UpdateImageDescriptorBinding(0, image_info);
  }
  {
    VkDescriptorImageInfo image_info;
    image_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
    image_info.imageView = render_texture0->GetColorImageView()->GetVkImageView();
    image_info.sampler = render_texture0->GetColorSampler()->GetVkSampler();
    ssr_combine_descriptor_set->UpdateImageDescriptorBinding(0, image_info);
    image_info.imageView = render_texture1->GetColorImageView()->GetVkImageView();
    image_info.sampler = render_texture1->GetColorSampler()->GetVkSampler();
    ssr_combine_descriptor_set->UpdateImageDescriptorBinding(1, image_info);
  }

  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
#pragma region Viewport and scissor
    VkRect2D render_area;
    render_area.offset = {0, 0};
    render_area.extent.width = target_camera->GetSize().x;
    render_area.extent.height = target_camera->GetSize().y;
    VkViewport viewport;
    viewport.x = 0.0f;
    viewport.y = 0.0f;
    viewport.width = target_camera->GetSize().x;
    viewport.height = target_camera->GetSize().y;
    viewport.minDepth = 0.0f;
    viewport.maxDepth = 1.0f;

    VkRect2D scissor;
    scissor.offset = {0, 0};
    scissor.extent.width = target_camera->GetSize().x;
    scissor.extent.height = target_camera->GetSize().y;
#pragma endregion
    GeometryStorage::BindVertices(vk_command_buffer);
    {
      PushConstant push_constant{};
      push_constant.max_distance = max_distance;
      push_constant.resolution = resolution;
      push_constant.initial_steps = initial_steps;
      push_constant.thickness = thickness;
      push_constant.camera_index =
          render_layer->GetCurrentRenderInstanceStorage()->GetCameraIndex(target_camera->GetHandle());
      const auto mesh = Resources::GetResource<Mesh>("PRIMITIVE_TEX_PASS_THROUGH");
      std::vector<VkRenderingAttachmentInfo> color_attachment_infos;
      VkRenderingInfo render_info2{};
      render_info2.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
      render_info2.renderArea = render_area;
      render_info2.layerCount = 1;
      render_info2.pDepthAttachment = VK_NULL_HANDLE;

      // Input texture
      target_camera->TransitGBufferImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);
      target_camera->render_texture_->GetDepthImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);
      target_camera->render_texture_->GetColorImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);
      // Attachments
      render_texture0->GetColorImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);
      render_texture1->GetColorImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);
      render_texture0->AppendColorAttachmentInfos(color_attachment_infos, VK_ATTACHMENT_LOAD_OP_DONT_CARE,
                                                  VK_ATTACHMENT_STORE_OP_STORE);
      render_texture1->AppendColorAttachmentInfos(color_attachment_infos, VK_ATTACHMENT_LOAD_OP_DONT_CARE,
                                                  VK_ATTACHMENT_STORE_OP_STORE);
      render_info2.colorAttachmentCount = color_attachment_infos.size();
      render_info2.pColorAttachments = color_attachment_infos.data();

      {
        vkCmdBeginRendering(vk_command_buffer, &render_info2);
        ssr_reflect_pipeline->states.depth_test = false;
        ssr_reflect_pipeline->states.color_blend_attachment_states.clear();
        ssr_reflect_pipeline->states.color_blend_attachment_states.resize(color_attachment_infos.size());
        for (auto& i : ssr_reflect_pipeline->states.color_blend_attachment_states) {
          i.colorWriteMask =
              VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
          i.blendEnable = VK_FALSE;
        }
        ssr_reflect_pipeline->Bind(vk_command_buffer);
        ssr_reflect_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                                render_layer->GetPerFrameDescriptorSet()->GetVkDescriptorSet());
        ssr_reflect_pipeline->BindDescriptorSet(vk_command_buffer, 1, ssr_reflect_descriptor_set->GetVkDescriptorSet());
        ssr_reflect_pipeline->states.view_port = viewport;
        ssr_reflect_pipeline->states.scissor = scissor;

        ssr_reflect_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
        mesh->DrawIndexed(vk_command_buffer, ssr_reflect_pipeline->states, 1);
        vkCmdEndRendering(vk_command_buffer);
      }

      // Input texture
      render_texture1->GetColorImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);
      // Attachments
      color_attachment_infos.clear();
      render_texture2->GetColorImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);
      render_texture2->AppendColorAttachmentInfos(color_attachment_infos, VK_ATTACHMENT_LOAD_OP_CLEAR,
                                                  VK_ATTACHMENT_STORE_OP_STORE);
      render_info2.colorAttachmentCount = color_attachment_infos.size();
      render_info2.pColorAttachments = color_attachment_infos.data();
      {
        vkCmdBeginRendering(vk_command_buffer, &render_info2);
        ssr_blur_pipeline->states.depth_test = false;
        ssr_blur_pipeline->states.color_blend_attachment_states.clear();
        ssr_blur_pipeline->states.color_blend_attachment_states.resize(color_attachment_infos.size());
        for (auto& i : ssr_blur_pipeline->states.color_blend_attachment_states) {
          i.colorWriteMask =
              VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
          i.blendEnable = VK_FALSE;
        }
        ssr_blur_pipeline->Bind(vk_command_buffer);
        ssr_blur_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                             ssr_blur_horizontal_descriptor_set->GetVkDescriptorSet());
        ssr_blur_pipeline->states.view_port = viewport;
        ssr_blur_pipeline->states.scissor = scissor;
        push_constant.horizontal = true;
        ssr_blur_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
        mesh->DrawIndexed(vk_command_buffer, ssr_blur_pipeline->states, 1);
        vkCmdEndRendering(vk_command_buffer);
      }
      // Input texture
      render_texture2->GetColorImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);
      // Attachments
      color_attachment_infos.clear();
      render_texture1->GetColorImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);
      render_texture1->AppendColorAttachmentInfos(color_attachment_infos, VK_ATTACHMENT_LOAD_OP_CLEAR,
                                                  VK_ATTACHMENT_STORE_OP_STORE);
      render_info2.colorAttachmentCount = color_attachment_infos.size();
      render_info2.pColorAttachments = color_attachment_infos.data();
      {
        vkCmdBeginRendering(vk_command_buffer, &render_info2);
        ssr_blur_pipeline->states.depth_test = false;
        ssr_blur_pipeline->states.color_blend_attachment_states.clear();
        ssr_blur_pipeline->states.color_blend_attachment_states.resize(color_attachment_infos.size());
        for (auto& i : ssr_blur_pipeline->states.color_blend_attachment_states) {
          i.colorWriteMask =
              VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
          i.blendEnable = VK_FALSE;
        }
        ssr_blur_pipeline->Bind(vk_command_buffer);
        ssr_blur_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                             ssr_blur_vertical_descriptor_set->GetVkDescriptorSet());
        ssr_blur_pipeline->states.view_port = viewport;
        ssr_blur_pipeline->states.scissor = scissor;
        push_constant.horizontal = false;
        ssr_blur_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
        mesh->DrawIndexed(vk_command_buffer, ssr_blur_pipeline->states, 1);
        vkCmdEndRendering(vk_command_buffer);
      }
      // Input texture
      render_texture0->GetColorImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);
      render_texture1->GetColorImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);
      // Attachments
      color_attachment_infos.clear();
      target_camera->GetRenderTexture()->GetColorImage()->TransitImageLayout(vk_command_buffer,
                                                                             VK_IMAGE_LAYOUT_GENERAL);
      target_camera->GetRenderTexture()->AppendColorAttachmentInfos(
          color_attachment_infos, VK_ATTACHMENT_LOAD_OP_DONT_CARE, VK_ATTACHMENT_STORE_OP_STORE);
      render_info2.colorAttachmentCount = color_attachment_infos.size();
      render_info2.pColorAttachments = color_attachment_infos.data();
      {
        vkCmdBeginRendering(vk_command_buffer, &render_info2);
        ssr_combine_pipeline->states.depth_test = false;
        ssr_combine_pipeline->states.color_blend_attachment_states.clear();
        ssr_combine_pipeline->states.color_blend_attachment_states.resize(color_attachment_infos.size());
        for (auto& i : ssr_combine_pipeline->states.color_blend_attachment_states) {
          i.colorWriteMask =
              VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
          i.blendEnable = VK_FALSE;
        }
        ssr_combine_pipeline->Bind(vk_command_buffer);
        ssr_combine_pipeline->BindDescriptorSet(vk_command_buffer, 0, ssr_combine_descriptor_set->GetVkDescriptorSet());
        ssr_combine_pipeline->states.view_port = viewport;
        ssr_combine_pipeline->states.scissor = scissor;
        ssr_combine_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
        mesh->DrawIndexed(vk_command_buffer, ssr_combine_pipeline->states, 1);
        vkCmdEndRendering(vk_command_buffer);
      }
    }
  });
}

void ScreenSpaceReflection::BuildPipelines() {
  ssr_reflect_layout = std::make_shared<DescriptorSetLayout>();
  ssr_reflect_layout->PushDescriptorBinding(17, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_FRAGMENT_BIT,
                                            0);
  ssr_reflect_layout->PushDescriptorBinding(18, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_FRAGMENT_BIT,
                                            0);
  ssr_reflect_layout->PushDescriptorBinding(19, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_FRAGMENT_BIT,
                                            0);
  ssr_reflect_layout->PushDescriptorBinding(20, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_FRAGMENT_BIT,
                                            0);
  ssr_reflect_layout->Initialize();

  ssr_blur_layout = std::make_shared<DescriptorSetLayout>();
  ssr_blur_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_FRAGMENT_BIT, 0);
  ssr_blur_layout->Initialize();

  ssr_combine_layout = std::make_shared<DescriptorSetLayout>();
  ssr_combine_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_FRAGMENT_BIT,
                                            0);
  ssr_combine_layout->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_FRAGMENT_BIT,
                                            0);
  ssr_combine_layout->Initialize();

  ssr_reflect_pipeline = std::make_shared<GraphicsPipeline>();
  ssr_reflect_pipeline->vertex_shader = Resources::GetResource<Shader>("TEXTURE_PASS_THROUGH_VERT");
  ssr_reflect_pipeline->fragment_shader = Shader::CreateTemporary(
      ShaderType::Fragment, Platform::Constants::shader_global_defines,
      std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Fragment/PostProcessing/SSRReflect.frag");
  ssr_reflect_pipeline->geometry_type = GeometryType::Mesh;
  ssr_reflect_pipeline->descriptor_set_layouts.emplace_back(RenderLayer::per_frame_layout);
  ssr_reflect_pipeline->descriptor_set_layouts.emplace_back(ssr_reflect_layout);
  ssr_reflect_pipeline->depth_attachment_format = VK_FORMAT_UNDEFINED;
  ssr_reflect_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
  ssr_reflect_pipeline->color_attachment_formats = {2, Platform::Constants::render_texture_color};
  auto& ssr_reflect_pipeline_push_constant_range = ssr_reflect_pipeline->push_constant_ranges.emplace_back();
  ssr_reflect_pipeline_push_constant_range.size = sizeof(PushConstant);
  ssr_reflect_pipeline_push_constant_range.offset = 0;
  ssr_reflect_pipeline_push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
  ssr_reflect_pipeline->Initialize();

  ssr_blur_pipeline = std::make_shared<GraphicsPipeline>();
  ssr_blur_pipeline->vertex_shader = Resources::GetResource<Shader>("TEXTURE_PASS_THROUGH_VERT");
  ssr_blur_pipeline->fragment_shader =
      Shader::CreateTemporary(ShaderType::Fragment, std::filesystem::path("./DefaultResources") /
                                                        "Shaders/Graphics/Fragment/PostProcessing/SSRBlur.frag");
  ssr_blur_pipeline->geometry_type = GeometryType::Mesh;
  ssr_blur_pipeline->descriptor_set_layouts.emplace_back(ssr_blur_layout);
  ssr_blur_pipeline->depth_attachment_format = VK_FORMAT_UNDEFINED;
  ssr_blur_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
  ssr_blur_pipeline->color_attachment_formats = {1, Platform::Constants::render_texture_color};
  auto& ssr_blur_pipeline_push_constant_range = ssr_blur_pipeline->push_constant_ranges.emplace_back();
  ssr_blur_pipeline_push_constant_range.size = sizeof(PushConstant);
  ssr_blur_pipeline_push_constant_range.offset = 0;
  ssr_blur_pipeline_push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
  ssr_blur_pipeline->Initialize();

  ssr_combine_pipeline = std::make_shared<GraphicsPipeline>();
  ssr_combine_pipeline->vertex_shader = Resources::GetResource<Shader>("TEXTURE_PASS_THROUGH_VERT");
  ssr_combine_pipeline->fragment_shader =
      Shader::CreateTemporary(ShaderType::Fragment, std::filesystem::path("./DefaultResources") /
                                                        "Shaders/Graphics/Fragment/PostProcessing/SSRCombine.frag");
  ssr_combine_pipeline->geometry_type = GeometryType::Mesh;
  ssr_combine_pipeline->descriptor_set_layouts.emplace_back(ssr_combine_layout);
  ssr_combine_pipeline->depth_attachment_format = VK_FORMAT_UNDEFINED;
  ssr_combine_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
  ssr_combine_pipeline->color_attachment_formats = {1, Platform::Constants::render_texture_color};
  auto& ssr_combine_pipeline_push_constant_range = ssr_combine_pipeline->push_constant_ranges.emplace_back();
  ssr_combine_pipeline_push_constant_range.size = sizeof(PushConstant);
  ssr_combine_pipeline_push_constant_range.offset = 0;
  ssr_combine_pipeline_push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
  ssr_combine_pipeline->Initialize();
}

void PostProcessingStack::Resize(const glm::uvec2& size) const {
  if (size.x == 0 || size.y == 0)
    return;
  if (size.x > 16384 || size.y >= 16384)
    return;
  render_texture0->Resize({size.x, size.y, 1});
  render_texture1->Resize({size.x, size.y, 1});
  render_texture2->Resize({size.x, size.y, 1});
}

void PostProcessingStack::OnCreate() {
  RenderTextureCreateInfo render_texture_create_info{};
  render_texture_create_info.depth = false;
  render_texture0 = std::make_unique<RenderTexture>(render_texture_create_info);
  render_texture1 = std::make_unique<RenderTexture>(render_texture_create_info);
  render_texture2 = std::make_unique<RenderTexture>(render_texture_create_info);

  screen_space_reflection.BuildPipelines();
}

bool PostProcessingStack::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;

  if (ImGui::Checkbox("SSR", &enable_screen_space_reflection))
    changed = true;

  if (enable_screen_space_reflection && ImGui::TreeNodeEx("SSR", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (screen_space_reflection.OnInspect(editor_layer))
      changed = true;
    ImGui::TreePop();
  }

  if (ImGui::TreeNode("Debug")) {
    static float debug_scale = 0.25f;
    ImGui::DragFloat("Scale", &debug_scale, 0.01f, 0.1f, 1.0f);
    debug_scale = glm::clamp(debug_scale, 0.1f, 1.0f);
    if (ImGui::TreeNode("Render Texture 0")) {
      ImGui::Image(
          render_texture0->GetColorImTextureId(),
          ImVec2(render_texture0->GetExtent().width * debug_scale, render_texture0->GetExtent().height * debug_scale),
          ImVec2(0, 1), ImVec2(1, 0));
      ImGui::TreePop();
    }
    if (ImGui::TreeNode("Render Texture 1")) {
      ImGui::Image(
          render_texture1->GetColorImTextureId(),
          ImVec2(render_texture1->GetExtent().width * debug_scale, render_texture1->GetExtent().height * debug_scale),
          ImVec2(0, 1), ImVec2(1, 0));
      ImGui::TreePop();
    }
    if (ImGui::TreeNode("Render Texture 2")) {
      ImGui::Image(
          render_texture2->GetColorImTextureId(),
          ImVec2(render_texture2->GetExtent().width * debug_scale, render_texture2->GetExtent().height * debug_scale),
          ImVec2(0, 1), ImVec2(1, 0));
      ImGui::TreePop();
    }
    ImGui::TreePop();
  }
  return changed;
}

void PostProcessingStack::Process(const std::shared_ptr<Camera>& target_camera) const {
  const auto render_layer = Application::GetLayer<RenderLayer>();
  if (enable_screen_space_ambient_occlusion) {
  }
  if (enable_screen_space_reflection) {
    screen_space_reflection.Process(render_texture0, render_texture1, render_texture2, target_camera);
  }
  if (enable_bloom) {
  }
}
