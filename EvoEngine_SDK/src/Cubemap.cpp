#include "Cubemap.hpp"

#include "Application.hpp"
#include "Console.hpp"
#include "Graphics.hpp"
#include "RenderLayer.hpp"
#include "Resources.hpp"
#include "TextureStorage.hpp"
using namespace evo_engine;
#include "EditorLayer.hpp"

Cubemap::Cubemap() {
  texture_storage_handle_ = TextureStorage::RegisterCubemap();
}

const CubemapStorage& Cubemap::PeekStorage() const {
  return TextureStorage::PeekCubemapStorage(texture_storage_handle_);
}

CubemapStorage& Cubemap::RefStorage() const {
  return TextureStorage::RefCubemapStorage(texture_storage_handle_);
}

Cubemap::~Cubemap() {
  TextureStorage::UnRegisterCubemap(texture_storage_handle_);
}

void Cubemap::Initialize(const uint32_t resolution, const uint32_t mip_levels) const {
  RefStorage().Initialize(resolution, mip_levels);
}

uint32_t Cubemap::GetTextureStorageIndex() const {
  return texture_storage_handle_->value;
}

void Cubemap::ConvertFromEquirectangularTexture(const std::shared_ptr<Texture2D>& target_texture) const {
  Initialize(1024);
  auto& storage = RefStorage();
  if (!target_texture->GetImage()) {
    EVOENGINE_ERROR("Target texture doesn't contain any content!");
    return;
  }
#pragma region Depth
  VkImageCreateInfo depth_image_info{};
  depth_image_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
  depth_image_info.imageType = VK_IMAGE_TYPE_2D;
  depth_image_info.extent.width = storage.image->GetExtent().width;
  depth_image_info.extent.height = storage.image->GetExtent().height;
  depth_image_info.extent.depth = 1;
  depth_image_info.mipLevels = 1;
  depth_image_info.arrayLayers = 1;
  depth_image_info.format = Graphics::Constants::shadow_map;
  depth_image_info.tiling = VK_IMAGE_TILING_OPTIMAL;
  depth_image_info.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
  depth_image_info.usage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;
  depth_image_info.samples = VK_SAMPLE_COUNT_1_BIT;
  depth_image_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

  const auto depth_image = std::make_shared<Image>(depth_image_info);
  Graphics::ImmediateSubmit([&](const VkCommandBuffer command_buffer) {
    depth_image->TransitImageLayout(command_buffer, VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL);
  });

  VkImageViewCreateInfo depth_view_info{};
  depth_view_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
  depth_view_info.image = depth_image->GetVkImage();
  depth_view_info.viewType = VK_IMAGE_VIEW_TYPE_2D;
  depth_view_info.format = Graphics::Constants::shadow_map;
  depth_view_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;
  depth_view_info.subresourceRange.baseMipLevel = 0;
  depth_view_info.subresourceRange.levelCount = 1;
  depth_view_info.subresourceRange.baseArrayLayer = 0;
  depth_view_info.subresourceRange.layerCount = 1;
  const auto depth_image_view = std::make_shared<ImageView>(depth_view_info);
#pragma endregion

  const std::unique_ptr<DescriptorSet> temp_set =
      std::make_unique<DescriptorSet>(Graphics::GetDescriptorSetLayout("RENDER_TEXTURE_PRESENT_LAYOUT"));
  VkDescriptorImageInfo descriptor_image_info{};
  descriptor_image_info.imageView = target_texture->GetVkImageView();
  descriptor_image_info.imageLayout = target_texture->GetLayout();
  descriptor_image_info.sampler = target_texture->GetVkSampler();

  temp_set->UpdateImageDescriptorBinding(0, descriptor_image_info);

  const glm::mat4 capture_projection = glm::perspective(glm::radians(90.0f), 1.0f, 0.1f, 10.0f);
  const glm::mat4 capture_views[] = {
      glm::lookAt(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(0.0f, -1.0f, 0.0f)),
      glm::lookAt(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(-1.0f, 0.0f, 0.0f), glm::vec3(0.0f, -1.0f, 0.0f)),
      glm::lookAt(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f)),
      glm::lookAt(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, -1.0f, 0.0f), glm::vec3(0.0f, 0.0f, -1.0f)),
      glm::lookAt(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f), glm::vec3(0.0f, -1.0f, 0.0f)),
      glm::lookAt(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 0.0f, -1.0f), glm::vec3(0.0f, -1.0f, 0.0f))};

  const auto equirectangular_to_cubemap = Graphics::GetGraphicsPipeline("EQUIRECTANGULAR_TO_CUBEMAP");
  Graphics::ImmediateSubmit([&](VkCommandBuffer command_buffer) {
    storage.image->TransitImageLayout(command_buffer, VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL);
#pragma region Viewport and scissor
    VkRect2D render_area;
    render_area.offset = {0, 0};
    render_area.extent.width = storage.image->GetExtent().width;
    render_area.extent.height = storage.image->GetExtent().height;
    VkViewport viewport;
    viewport.x = 0.0f;
    viewport.y = 0.0f;
    viewport.width = storage.image->GetExtent().width;
    viewport.height = storage.image->GetExtent().height;
    viewport.minDepth = 0.0f;
    viewport.maxDepth = 1.0f;

    VkRect2D scissor;
    scissor.offset = {0, 0};
    scissor.extent.width = storage.image->GetExtent().width;
    scissor.extent.height = storage.image->GetExtent().height;
    equirectangular_to_cubemap->states.view_port = viewport;
    equirectangular_to_cubemap->states.scissor = scissor;
#pragma endregion
    for (int i = 0; i < 6; i++) {
#pragma region Lighting pass
      VkRenderingAttachmentInfo attachment{};
      attachment.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;

      attachment.imageLayout = VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL;
      attachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
      attachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;

      attachment.clearValue = {0, 0, 0, 1};
      attachment.imageView = storage.face_views[i]->GetVkImageView();

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
      equirectangular_to_cubemap->states.cull_mode = VK_CULL_MODE_NONE;
      equirectangular_to_cubemap->states.color_blend_attachment_states.clear();
      equirectangular_to_cubemap->states.color_blend_attachment_states.resize(1);
      for (auto& i : equirectangular_to_cubemap->states.color_blend_attachment_states) {
        i.colorWriteMask =
            VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
        i.blendEnable = VK_FALSE;
      }
      vkCmdBeginRendering(command_buffer, &render_info);
      equirectangular_to_cubemap->Bind(command_buffer);
      equirectangular_to_cubemap->BindDescriptorSet(command_buffer, 0, temp_set->GetVkDescriptorSet());
      const auto mesh = Resources::GetResource<Mesh>("PRIMITIVE_RENDERING_CUBE");
      GeometryStorage::BindVertices(command_buffer);
      EquirectangularToCubemapConstant constant{};
      constant.projection_view = capture_projection * capture_views[i];
      equirectangular_to_cubemap->PushConstant(command_buffer, 0, constant);
      mesh->DrawIndexed(command_buffer, equirectangular_to_cubemap->states, 1);
      vkCmdEndRendering(command_buffer);
#pragma endregion

      Graphics::EverythingBarrier(command_buffer);
    }
    storage.image->TransitImageLayout(command_buffer, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
  });
}
bool Cubemap::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (const auto& storage = RefStorage(); !storage.im_texture_ids.empty()) {
    static float debug_scale = 0.25f;
    ImGui::DragFloat("Scale", &debug_scale, 0.01f, 0.1f, 1.0f);
    debug_scale = glm::clamp(debug_scale, 0.1f, 1.0f);
    for (int i = 0; i < 6; i++) {
      ImGui::Image(
          storage.im_texture_ids[i],
          ImVec2(storage.image->GetExtent().width * debug_scale, storage.image->GetExtent().height * debug_scale),
          ImVec2(0, 1), ImVec2(1, 0));
    }
  }

  return changed;
}

const std::shared_ptr<Image>& Cubemap::GetImage() const {
  auto& storage = RefStorage();
  return storage.image;
}

const std::shared_ptr<ImageView>& Cubemap::GetImageView() const {
  auto& storage = RefStorage();
  return storage.image_view;
}

const std::shared_ptr<Sampler>& Cubemap::GetSampler() const {
  auto& storage = RefStorage();
  return storage.sampler;
}

const std::vector<std::shared_ptr<ImageView>>& Cubemap::GetFaceViews() const {
  auto& storage = RefStorage();
  return storage.face_views;
}
