#include "PostProcessingStack.hpp"

#include "Application.hpp"
#include "Camera.hpp"
#include "GeometryStorage.hpp"
#include "Platform.hpp"
#include "Mesh.hpp"
#include "RenderLayer.hpp"
#include "Resources.hpp"
using namespace evo_engine;

void PostProcessingStack::Resize(const glm::uvec2& size) const {
  if (size.x == 0 || size.y == 0)
    return;
  if (size.x > 16384 || size.y >= 16384)
    return;
  render_texture0_->Resize({size.x, size.y, 1});
  render_texture1_->Resize({size.x, size.y, 1});
  render_texture2_->Resize({size.x, size.y, 1});
}

void PostProcessingStack::OnCreate() {
  RenderTextureCreateInfo render_texture_create_info{};
  render_texture_create_info.depth = false;
  render_texture0_ = std::make_unique<RenderTexture>(render_texture_create_info);
  render_texture1_ = std::make_unique<RenderTexture>(render_texture_create_info);
  render_texture2_ = std::make_unique<RenderTexture>(render_texture_create_info);

  ssr_reflect_descriptor_set_ = std::make_shared<DescriptorSet>(Platform::GetDescriptorSetLayout("SSR_REFLECT_LAYOUT"));
  ssr_blur_horizontal_descriptor_set_ =
      std::make_shared<DescriptorSet>(Platform::GetDescriptorSetLayout("RENDER_TEXTURE_PRESENT_LAYOUT"));
  ssr_blur_vertical_descriptor_set_ =
      std::make_shared<DescriptorSet>(Platform::GetDescriptorSetLayout("RENDER_TEXTURE_PRESENT_LAYOUT"));
  ssr_combine_descriptor_set_ = std::make_shared<DescriptorSet>(Platform::GetDescriptorSetLayout("SSR_COMBINE_LAYOUT"));
}

bool PostProcessingStack::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ssr && ImGui::TreeNode("SSR Settings")) {
    if (ImGui::DragFloat("Step size", &ssr_settings.step, 0.1, 0.1, 10.0f))
      changed = false;
    if (ImGui::DragInt("Max steps", &ssr_settings.max_steps, 1, 1, 32))
      changed = false;
    if (ImGui::DragInt("Binary search steps", &ssr_settings.num_binary_search_steps, 1, 1, 16))
      changed = false;
    ImGui::TreePop();
  }

  return changed;
}

void PostProcessingStack::Process(const std::shared_ptr<Camera>& target_camera) const {
  const auto render_layer = Application::GetLayer<RenderLayer>();
  const auto size = target_camera->GetSize();
  render_texture0_->Resize({size.x, size.y, 1});
  render_texture1_->Resize({size.x, size.y, 1});
  render_texture2_->Resize({size.x, size.y, 1});
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  if (ssao) {
  }
  if (ssr) {
    {
      VkDescriptorImageInfo image_info;
      image_info.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
      image_info.imageView = target_camera->GetRenderTexture()->GetDepthImageView()->GetVkImageView();
      image_info.sampler = target_camera->GetRenderTexture()->GetDepthSampler()->GetVkSampler();
      ssr_reflect_descriptor_set_->UpdateImageDescriptorBinding(17, image_info);
      image_info.imageView = target_camera->g_buffer_normal_view_->GetVkImageView();
      image_info.sampler = target_camera->g_buffer_normal_sampler_->GetVkSampler();
      ssr_reflect_descriptor_set_->UpdateImageDescriptorBinding(18, image_info);
      image_info.imageView = target_camera->g_buffer_material_view_->GetVkImageView();
      image_info.sampler = target_camera->g_buffer_material_sampler_->GetVkSampler();
      ssr_reflect_descriptor_set_->UpdateImageDescriptorBinding(19, image_info);

      image_info.imageView = target_camera->GetRenderTexture()->GetColorImageView()->GetVkImageView();
      image_info.sampler = target_camera->GetRenderTexture()->GetColorSampler()->GetVkSampler();
      ssr_reflect_descriptor_set_->UpdateImageDescriptorBinding(20, image_info);
      
    }
    {
      VkDescriptorImageInfo image_info;
      image_info.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
      image_info.imageView = render_texture1_->GetColorImageView()->GetVkImageView();
      image_info.sampler = render_texture1_->GetColorSampler()->GetVkSampler();
      ssr_blur_horizontal_descriptor_set_->UpdateImageDescriptorBinding(0, image_info);
    }
    {
      VkDescriptorImageInfo image_info;
      image_info.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
      image_info.imageView = render_texture2_->GetColorImageView()->GetVkImageView();
      image_info.sampler = render_texture2_->GetColorSampler()->GetVkSampler();
      ssr_blur_vertical_descriptor_set_->UpdateImageDescriptorBinding(0, image_info);
    }
    {
      VkDescriptorImageInfo image_info;
      image_info.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
      image_info.imageView = render_texture0_->GetColorImageView()->GetVkImageView();
      image_info.sampler = render_texture0_->GetColorSampler()->GetVkSampler();
      ssr_combine_descriptor_set_->UpdateImageDescriptorBinding(0, image_info);
      image_info.imageView = render_texture1_->GetColorImageView()->GetVkImageView();
      image_info.sampler = render_texture1_->GetColorSampler()->GetVkSampler();
      ssr_combine_descriptor_set_->UpdateImageDescriptorBinding(1, image_info);
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
      VkRenderingInfo render_info{};
      render_info.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
      render_info.renderArea = render_area;
      render_info.layerCount = 1;
#pragma endregion
      GeometryStorage::BindVertices(vk_command_buffer);
      {
        SsrPushConstant push_constant{};
        push_constant.num_binary_search_steps = ssr_settings.num_binary_search_steps;
        push_constant.step = ssr_settings.step;
        push_constant.max_steps = ssr_settings.max_steps;
        push_constant.camera_index = render_layer->GetCameraIndex(target_camera->GetHandle());
        const auto mesh = Resources::GetResource<Mesh>("PRIMITIVE_TEX_PASS_THROUGH");
        std::vector<VkRenderingAttachmentInfo> color_attachment_infos;
        VkRenderingInfo render_info2{};
        render_info2.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
        render_info2.renderArea = render_area;
        render_info2.layerCount = 1;
        render_info2.pDepthAttachment = VK_NULL_HANDLE;

        // Input texture
        target_camera->TransitGBufferImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
        target_camera->render_texture_->GetDepthImage()->TransitImageLayout(vk_command_buffer,
                                                                            VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
        target_camera->render_texture_->GetColorImage()->TransitImageLayout(vk_command_buffer,
                                                                            VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
        // Attachments
        render_texture0_->GetColorImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL);
        render_texture1_->GetColorImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL);
        render_texture0_->AppendColorAttachmentInfos(color_attachment_infos, VK_ATTACHMENT_LOAD_OP_DONT_CARE,
                                                     VK_ATTACHMENT_STORE_OP_STORE);
        render_texture1_->AppendColorAttachmentInfos(color_attachment_infos, VK_ATTACHMENT_LOAD_OP_DONT_CARE,
                                                     VK_ATTACHMENT_STORE_OP_STORE);
        render_info2.colorAttachmentCount = color_attachment_infos.size();
        render_info2.pColorAttachments = color_attachment_infos.data();

        {
          const auto& ssr_reflect_pipeline = Platform::GetGraphicsPipeline("SSR_REFLECT");
          vkCmdBeginRendering(vk_command_buffer, &render_info2);
          ssr_reflect_pipeline->states.depth_test = false;
          ssr_reflect_pipeline->states.color_blend_attachment_states.clear();
          ssr_reflect_pipeline->states.color_blend_attachment_states.resize(color_attachment_infos.size());
          for (auto& i : ssr_reflect_pipeline->states.color_blend_attachment_states) {
            i.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT |
                               VK_COLOR_COMPONENT_A_BIT;
            i.blendEnable = VK_FALSE;
          }
          ssr_reflect_pipeline->Bind(vk_command_buffer);
          ssr_reflect_pipeline->BindDescriptorSet(
              vk_command_buffer, 0, render_layer->per_frame_descriptor_sets_[current_frame_index]->GetVkDescriptorSet());
          ssr_reflect_pipeline->BindDescriptorSet(vk_command_buffer, 1, ssr_reflect_descriptor_set_->GetVkDescriptorSet());
          ssr_reflect_pipeline->states.view_port = viewport;
          ssr_reflect_pipeline->states.scissor = scissor;

          ssr_reflect_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
          mesh->DrawIndexed(vk_command_buffer, ssr_reflect_pipeline->states, 1);
          vkCmdEndRendering(vk_command_buffer);
        }
        // Input texture
        render_texture1_->GetColorImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
        // Attachments
        color_attachment_infos.clear();
        render_texture2_->GetColorImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL);
        render_texture2_->AppendColorAttachmentInfos(color_attachment_infos, VK_ATTACHMENT_LOAD_OP_CLEAR,
                                                     VK_ATTACHMENT_STORE_OP_STORE);
        render_info2.colorAttachmentCount = color_attachment_infos.size();
        render_info2.pColorAttachments = color_attachment_infos.data();
        {
          const auto& ssr_blur_pipeline = Platform::GetGraphicsPipeline("SSR_BLUR");
          vkCmdBeginRendering(vk_command_buffer, &render_info2);
          ssr_blur_pipeline->states.depth_test = false;
          ssr_blur_pipeline->states.color_blend_attachment_states.clear();
          ssr_blur_pipeline->states.color_blend_attachment_states.resize(color_attachment_infos.size());
          for (auto& i : ssr_blur_pipeline->states.color_blend_attachment_states) {
            i.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT |
                               VK_COLOR_COMPONENT_A_BIT;
            i.blendEnable = VK_FALSE;
          }
          ssr_blur_pipeline->Bind(vk_command_buffer);
          ssr_blur_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                               ssr_blur_horizontal_descriptor_set_->GetVkDescriptorSet());
          ssr_blur_pipeline->states.view_port = viewport;
          ssr_blur_pipeline->states.scissor = scissor;
          push_constant.horizontal = true;
          ssr_blur_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
          mesh->DrawIndexed(vk_command_buffer, ssr_blur_pipeline->states, 1);
          vkCmdEndRendering(vk_command_buffer);
        }
        // Input texture
        render_texture2_->GetColorImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
        // Attachments
        color_attachment_infos.clear();
        render_texture1_->GetColorImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL);
        render_texture1_->AppendColorAttachmentInfos(color_attachment_infos, VK_ATTACHMENT_LOAD_OP_CLEAR,
                                                     VK_ATTACHMENT_STORE_OP_STORE);
        render_info2.colorAttachmentCount = color_attachment_infos.size();
        render_info2.pColorAttachments = color_attachment_infos.data();
        {
          const auto& ssr_blur_pipeline = Platform::GetGraphicsPipeline("SSR_BLUR");
          vkCmdBeginRendering(vk_command_buffer, &render_info2);
          ssr_blur_pipeline->states.depth_test = false;
          ssr_blur_pipeline->states.color_blend_attachment_states.clear();
          ssr_blur_pipeline->states.color_blend_attachment_states.resize(color_attachment_infos.size());
          for (auto& i : ssr_blur_pipeline->states.color_blend_attachment_states) {
            i.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT |
                               VK_COLOR_COMPONENT_A_BIT;
            i.blendEnable = VK_FALSE;
          }
          ssr_blur_pipeline->Bind(vk_command_buffer);
          ssr_blur_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                               ssr_blur_vertical_descriptor_set_->GetVkDescriptorSet());
          ssr_blur_pipeline->states.view_port = viewport;
          ssr_blur_pipeline->states.scissor = scissor;
          push_constant.horizontal = false;
          ssr_blur_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
          mesh->DrawIndexed(vk_command_buffer, ssr_blur_pipeline->states, 1);
          vkCmdEndRendering(vk_command_buffer);
        }
        // Input texture
        render_texture0_->GetColorImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
        render_texture1_->GetColorImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
        // Attachments
        color_attachment_infos.clear();
        target_camera->render_texture_->GetColorImage()->TransitImageLayout(vk_command_buffer,
                                                                            VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL);
        target_camera->render_texture_->AppendColorAttachmentInfos(
            color_attachment_infos, VK_ATTACHMENT_LOAD_OP_DONT_CARE, VK_ATTACHMENT_STORE_OP_STORE);
        render_info2.colorAttachmentCount = color_attachment_infos.size();
        render_info2.pColorAttachments = color_attachment_infos.data();
        {
          const auto& ssr_combine_pipeline = Platform::GetGraphicsPipeline("SSR_COMBINE");
          vkCmdBeginRendering(vk_command_buffer, &render_info2);
          ssr_combine_pipeline->states.depth_test = false;
          ssr_combine_pipeline->states.color_blend_attachment_states.clear();
          ssr_combine_pipeline->states.color_blend_attachment_states.resize(color_attachment_infos.size());
          for (auto& i : ssr_combine_pipeline->states.color_blend_attachment_states) {
            i.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT |
                               VK_COLOR_COMPONENT_A_BIT;
            i.blendEnable = VK_FALSE;
          }
          ssr_combine_pipeline->Bind(vk_command_buffer);
          ssr_combine_pipeline->BindDescriptorSet(vk_command_buffer, 0, ssr_combine_descriptor_set_->GetVkDescriptorSet());
          ssr_combine_pipeline->states.view_port = viewport;
          ssr_combine_pipeline->states.scissor = scissor;
          ssr_combine_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
          mesh->DrawIndexed(vk_command_buffer, ssr_combine_pipeline->states, 1);
          vkCmdEndRendering(vk_command_buffer);
        }
      }
    });
  }
  if (bloom) {
  }
}
