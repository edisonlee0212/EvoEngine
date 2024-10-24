#include "ReflectionProbe.hpp"
#include "EditorLayer.hpp"
#include "Mesh.hpp"
#include "Resources.hpp"
#include "TextureStorage.hpp"
using namespace evo_engine;
struct EquirectangularToCubemapConstant {
  glm::mat4 projection_view = {};
  float m_preset = 0;
};

void ReflectionProbe::Initialize(uint32_t resolution) {
  cubemap_ = ProjectManager::CreateTemporaryAsset<Cubemap>();
  const uint32_t mip_levels = static_cast<uint32_t>(std::floor(std::log2(std::max(resolution, resolution)))) + 1;

  cubemap_->Initialize(resolution, mip_levels);

  Platform::ImmediateSubmit([&](const VkCommandBuffer vk_command_buffer) {
    cubemap_->RefStorage().image->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
    cubemap_->RefStorage().image->GenerateMipmaps(vk_command_buffer);
    cubemap_->RefStorage().image->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
  });

  for (int i = 0; i < 6; i++) {
    mip_map_views_.emplace_back();
    for (unsigned int mip = 0; mip < mip_levels; ++mip) {
      VkImageViewCreateInfo view_info{};
      view_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
      view_info.image = cubemap_->RefStorage().image->GetVkImage();
      view_info.viewType = VK_IMAGE_VIEW_TYPE_2D;
      view_info.format = Platform::Constants::texture_2d;
      view_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
      view_info.subresourceRange.baseMipLevel = mip;
      view_info.subresourceRange.levelCount = 1;
      view_info.subresourceRange.baseArrayLayer = i;
      view_info.subresourceRange.layerCount = 1;

      mip_map_views_[i].emplace_back(std::make_shared<ImageView>(view_info));
    }
  }
}

std::shared_ptr<Cubemap> ReflectionProbe::GetCubemap() const {
  return cubemap_;
}
void ReflectionProbe::ConstructFromCubemap(const std::shared_ptr<Cubemap>& target_cubemap) {
  if (!cubemap_)
    Initialize();

  if (!target_cubemap->RefStorage().image) {
    EVOENGINE_ERROR("Target cubemap doesn't contain any content!")
    return;
  }

#pragma region Depth
  VkImageCreateInfo depth_image_info{};
  depth_image_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
  depth_image_info.imageType = VK_IMAGE_TYPE_2D;
  depth_image_info.extent.width = cubemap_->RefStorage().image->GetExtent().width;
  depth_image_info.extent.height = cubemap_->RefStorage().image->GetExtent().height;
  depth_image_info.extent.depth = 1;
  depth_image_info.mipLevels = 1;
  depth_image_info.arrayLayers = 1;
  depth_image_info.format = Platform::Constants::shadow_map;
  depth_image_info.tiling = VK_IMAGE_TILING_OPTIMAL;
  depth_image_info.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
  depth_image_info.usage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;
  depth_image_info.samples = VK_SAMPLE_COUNT_1_BIT;
  depth_image_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

  const auto depth_image = std::make_shared<Image>(depth_image_info);
  Platform::ImmediateSubmit([&](const VkCommandBuffer vk_command_buffer) {
    depth_image->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL);
  });

  VkImageViewCreateInfo depth_view_info{};
  depth_view_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
  depth_view_info.image = depth_image->GetVkImage();
  depth_view_info.viewType = VK_IMAGE_VIEW_TYPE_2D;
  depth_view_info.format = Platform::Constants::shadow_map;
  depth_view_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;
  depth_view_info.subresourceRange.baseMipLevel = 0;
  depth_view_info.subresourceRange.levelCount = 1;
  depth_view_info.subresourceRange.baseArrayLayer = 0;
  depth_view_info.subresourceRange.layerCount = 1;
  const auto depth_image_view = std::make_shared<ImageView>(depth_view_info);
#pragma endregion

  const std::unique_ptr<DescriptorSet> temp_set =
      std::make_unique<DescriptorSet>(Platform::GetDescriptorSetLayout("RENDER_TEXTURE_PRESENT_LAYOUT"));
  VkDescriptorImageInfo descriptor_image_info;
  descriptor_image_info.imageView = target_cubemap->GetImageView()->GetVkImageView();
  descriptor_image_info.imageLayout = target_cubemap->GetImage()->GetLayout();
  descriptor_image_info.sampler = target_cubemap->GetSampler()->GetVkSampler();

  temp_set->UpdateImageDescriptorBinding(0, descriptor_image_info);

  const glm::mat4 capture_projection = glm::perspective(glm::radians(90.0f), 1.0f, 0.1f, 10.0f);
  const glm::mat4 capture_views[] = {
      glm::lookAt(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(0.0f, -1.0f, 0.0f)),
      glm::lookAt(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(-1.0f, 0.0f, 0.0f), glm::vec3(0.0f, -1.0f, 0.0f)),
      glm::lookAt(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f)),
      glm::lookAt(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, -1.0f, 0.0f), glm::vec3(0.0f, 0.0f, -1.0f)),
      glm::lookAt(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f), glm::vec3(0.0f, -1.0f, 0.0f)),
      glm::lookAt(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 0.0f, -1.0f), glm::vec3(0.0f, -1.0f, 0.0f))};
  const auto max_mip_levels = cubemap_->RefStorage().image->GetMipLevels();

  const auto prefilter_construct = Platform::GetGraphicsPipeline("PREFILTER_CONSTRUCT");
  Platform::ImmediateSubmit([&](const VkCommandBuffer vk_command_buffer) {
    cubemap_->RefStorage().image->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL);

    for (uint32_t mip = 0; mip < max_mip_levels; ++mip) {
      unsigned int mip_width = cubemap_->RefStorage().image->GetExtent().width * std::pow(0.5, mip);
      float roughness = static_cast<float>(mip) / static_cast<float>(max_mip_levels - 1);
#pragma region Viewport and scissor
      VkRect2D render_area;
      render_area.offset = {0, 0};
      render_area.extent.width = mip_width;
      render_area.extent.height = mip_width;
      VkViewport viewport;
      viewport.x = 0.0f;
      viewport.y = 0.0f;
      viewport.width = mip_width;
      viewport.height = mip_width;
      viewport.minDepth = 0.0f;
      viewport.maxDepth = 1.0f;

      VkRect2D scissor;
      scissor.offset = {0, 0};
      scissor.extent.width = mip_width;
      scissor.extent.height = mip_width;
      prefilter_construct->states.view_port = viewport;
      prefilter_construct->states.scissor = scissor;
#pragma endregion
      GeometryStorage::BindVertices(vk_command_buffer);
      for (int i = 0; i < 6; i++) {
#pragma region Lighting pass
        VkRenderingAttachmentInfo attachment{};
        attachment.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;

        attachment.imageLayout = VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL;
        attachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        attachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;

        attachment.clearValue = {0, 0, 0, 1};
        attachment.imageView = mip_map_views_[i][mip]->GetVkImageView();

        VkRenderingAttachmentInfo depth_attachment{};
        depth_attachment.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;

        depth_attachment.imageLayout = VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL;
        depth_attachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        depth_attachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;

        depth_attachment.clearValue.depthStencil = {1, 0};
        depth_attachment.imageView = depth_image_view->GetVkImageView();

        VkRenderingInfo render_info{};
        render_info.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
        render_info.renderArea = render_area;
        render_info.layerCount = 1;
        render_info.colorAttachmentCount = 1;
        render_info.pColorAttachments = &attachment;
        render_info.pDepthAttachment = &depth_attachment;
        prefilter_construct->states.cull_mode = VK_CULL_MODE_NONE;
        prefilter_construct->states.color_blend_attachment_states.clear();
        prefilter_construct->states.color_blend_attachment_states.resize(1);
        for (auto& i : prefilter_construct->states.color_blend_attachment_states) {
          i.colorWriteMask =
              VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
          i.blendEnable = VK_FALSE;
        }
        vkCmdBeginRendering(vk_command_buffer, &render_info);
        prefilter_construct->Bind(vk_command_buffer);
        prefilter_construct->BindDescriptorSet(vk_command_buffer, 0, temp_set->GetVkDescriptorSet());
        const auto mesh = Resources::GetResource<Mesh>("PRIMITIVE_RENDERING_CUBE");
        EquirectangularToCubemapConstant constant{};
        constant.projection_view = capture_projection * capture_views[i];
        constant.m_preset = roughness;
        prefilter_construct->PushConstant(vk_command_buffer, 0, constant);
        mesh->DrawIndexed(vk_command_buffer, prefilter_construct->states, 1);
        vkCmdEndRendering(vk_command_buffer);
#pragma endregion

        Platform::EverythingBarrier(vk_command_buffer);
      }
    }
    cubemap_->RefStorage().image->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
  });
}

bool ReflectionProbe::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (!cubemap_->RefStorage().im_texture_ids.empty()) {
    static float debug_scale = 0.25f;
    ImGui::DragFloat("Scale", &debug_scale, 0.01f, 0.1f, 1.0f);
    debug_scale = glm::clamp(debug_scale, 0.1f, 1.0f);
    for (int i = 0; i < 6; i++) {
      ImGui::Image(cubemap_->RefStorage().im_texture_ids[i],
                   ImVec2(cubemap_->RefStorage().image->GetExtent().width * debug_scale,
                          cubemap_->RefStorage().image->GetExtent().height * debug_scale),
                   ImVec2(0, 1), ImVec2(1, 0));
    }
  }

  return changed;
}
