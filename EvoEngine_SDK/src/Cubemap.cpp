#include "Cubemap.hpp"
#include "Application.hpp"
#include "Console.hpp"
#include "Platform.hpp"
#include "RenderLayer.hpp"
#include "Resources.hpp"
#include "Shader.hpp"
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

void Cubemap::BuildSkyIllumination(const SkyIllumination& sky_illumination, uint32_t resolution) const {
  const auto render_layer = Application::GetLayer<RenderLayer>();
  if (!render_layer)
    return;
  Initialize(resolution);
  auto& storage = RefStorage();
#pragma region Depth
  VkImageCreateInfo depth_image_info{};
  depth_image_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
  depth_image_info.imageType = VK_IMAGE_TYPE_2D;
  depth_image_info.extent.width = storage.image->GetExtent().width;
  depth_image_info.extent.height = storage.image->GetExtent().height;
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

  const glm::mat4 capture_projection = glm::perspective(glm::radians(90.0f), 1.0f, 0.1f, 10.0f);
  const glm::mat4 capture_views[] = {
      glm::lookAt(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(0.0f, -1.0f, 0.0f)),
      glm::lookAt(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(-1.0f, 0.0f, 0.0f), glm::vec3(0.0f, -1.0f, 0.0f)),
      glm::lookAt(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f)),
      glm::lookAt(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, -1.0f, 0.0f), glm::vec3(0.0f, 0.0f, -1.0f)),
      glm::lookAt(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f), glm::vec3(0.0f, -1.0f, 0.0f)),
      glm::lookAt(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 0.0f, -1.0f), glm::vec3(0.0f, -1.0f, 0.0f))};

  static std::shared_ptr<GraphicsPipeline> atmosphere_to_cubemap;

  struct PushConstant {
    glm::mat4 projection_view;
    Atmosphere atmosphere;
    glm::vec3 sun_direction;
    float gamma;

    glm::vec3 ground_color;
    float ground_transmittance;
  };
  PushConstant push_constant;
  push_constant.atmosphere = sky_illumination.atmosphere;
  push_constant.sun_direction = sky_illumination.sun_direction;
  push_constant.gamma = sky_illumination.gamma;
  push_constant.ground_color = sky_illumination.ground_color;
  push_constant.ground_transmittance = sky_illumination.ground_transmittance;
  if (!atmosphere_to_cubemap) {
    atmosphere_to_cubemap = std::make_shared<GraphicsPipeline>();
    atmosphere_to_cubemap->vertex_shader =
        Shader::CreateTemporary(ShaderType::Vertex, std::filesystem::path("./DefaultResources") /
                                                        "Shaders/Graphics/Vertex/Lighting/AtmosphereToCubemap.vert");
    atmosphere_to_cubemap->fragment_shader = Shader::CreateTemporary(
        ShaderType::Fragment,
        std::filesystem::path("./DefaultResources") / "Shaders/Graphics/Fragment/Lighting/AtmosphereToCubemap.frag");
    atmosphere_to_cubemap->geometry_type = GeometryType::Mesh;

    atmosphere_to_cubemap->depth_attachment_format = Platform::Constants::shadow_map;
    atmosphere_to_cubemap->stencil_attachment_format = VK_FORMAT_UNDEFINED;

    atmosphere_to_cubemap->color_attachment_formats = {1, Platform::Constants::texture_2d};
    atmosphere_to_cubemap->descriptor_set_layouts.emplace_back(RenderTexture::render_texture_present_layout);

    auto& push_constant_range = atmosphere_to_cubemap->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(PushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;

    atmosphere_to_cubemap->Initialize();
  }
  Platform::ImmediateSubmit([&](const VkCommandBuffer vk_command_buffer) {
    storage.image->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL);
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
    atmosphere_to_cubemap->states.view_port = viewport;
    atmosphere_to_cubemap->states.scissor = scissor;
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
      atmosphere_to_cubemap->states.cull_mode = VK_CULL_MODE_NONE;
      atmosphere_to_cubemap->states.color_blend_attachment_states.clear();
      atmosphere_to_cubemap->states.color_blend_attachment_states.resize(1);
      for (auto& i : atmosphere_to_cubemap->states.color_blend_attachment_states) {
        i.colorWriteMask =
            VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
        i.blendEnable = VK_FALSE;
      }
      vkCmdBeginRendering(vk_command_buffer, &render_info);
      atmosphere_to_cubemap->Bind(vk_command_buffer);
      const auto mesh = Resources::TryGetResource<Mesh>("PRIMITIVE_RENDERING_CUBE");
      GeometryStorage::BindVertices(vk_command_buffer);
      push_constant.projection_view = capture_projection * capture_views[i];
      atmosphere_to_cubemap->PushConstant(vk_command_buffer, 0, push_constant);
      mesh->DrawIndexed(vk_command_buffer, atmosphere_to_cubemap->states, 1);
      vkCmdEndRendering(vk_command_buffer);
#pragma endregion
      Platform::EverythingBarrier(vk_command_buffer);
    }
    storage.image->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
  });
}

void Cubemap::ConvertFromEquirectangularTexture(const std::shared_ptr<Texture2D>& target_texture) const {
  const auto render_layer = Application::GetLayer<RenderLayer>();
  if (!render_layer)
    return;
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
      std::make_unique<DescriptorSet>(RenderTexture::render_texture_present_layout);
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

  static std::shared_ptr<GraphicsPipeline> equirectangular_to_cubemap;
  if (!equirectangular_to_cubemap) {
    equirectangular_to_cubemap = std::make_shared<GraphicsPipeline>();
    equirectangular_to_cubemap->vertex_shader =
        Shader::CreateTemporary(ShaderType::Vertex, std::filesystem::path("./DefaultResources") /
                                                        "Shaders/Graphics/Vertex/Lighting/CubemapProcess.vert");
    equirectangular_to_cubemap->fragment_shader = Shader::CreateTemporary(
        ShaderType::Fragment, std::filesystem::path("./DefaultResources") /
                                  "Shaders/Graphics/Fragment/Lighting/EquirectangularMapToCubemap.frag");
    equirectangular_to_cubemap->geometry_type = GeometryType::Mesh;

    equirectangular_to_cubemap->depth_attachment_format = Platform::Constants::shadow_map;
    equirectangular_to_cubemap->stencil_attachment_format = VK_FORMAT_UNDEFINED;

    equirectangular_to_cubemap->color_attachment_formats = {1, Platform::Constants::texture_2d};
    equirectangular_to_cubemap->descriptor_set_layouts.emplace_back(RenderTexture::render_texture_present_layout);

    auto& push_constant_range = equirectangular_to_cubemap->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(glm::mat4) + sizeof(float);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;

    equirectangular_to_cubemap->Initialize();
  }
  Platform::ImmediateSubmit([&](const VkCommandBuffer vk_command_buffer) {
    storage.image->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL);
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
      vkCmdBeginRendering(vk_command_buffer, &render_info);
      equirectangular_to_cubemap->Bind(vk_command_buffer);
      equirectangular_to_cubemap->BindDescriptorSet(vk_command_buffer, 0, temp_set->GetVkDescriptorSet());
      const auto mesh = Resources::TryGetResource<Mesh>("PRIMITIVE_RENDERING_CUBE");
      GeometryStorage::BindVertices(vk_command_buffer);
      EquirectangularToCubemapConstant constant{};
      constant.projection_view = capture_projection * capture_views[i];
      equirectangular_to_cubemap->PushConstant(vk_command_buffer, 0, constant);
      mesh->DrawIndexed(vk_command_buffer, equirectangular_to_cubemap->states, 1);
      vkCmdEndRendering(vk_command_buffer);
#pragma endregion

      Platform::EverythingBarrier(vk_command_buffer);
    }
    storage.image->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
  });
}

bool Cubemap::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::TreeNode("Sky illumination")) {
    static bool auto_rebuild = true;
    ImGui::Checkbox("Auto refresh", &auto_rebuild);
    static SkyIllumination sky_illumination{};
    const bool rebuild = sky_illumination.OnInspect(editor_layer);
    if (ImGui::Button("Build") || (auto_rebuild && rebuild)) {
      BuildSkyIllumination(sky_illumination);
      changed = true;
    }
    ImGui::TreePop();
  }

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
