#include "Cubemap.hpp"
#include "Application.hpp"
#include "Console.hpp"
#include "GpuService.hpp"
#include "Platform.hpp"
#include "RenderLayer.hpp"
#include "Resources.hpp"
#include "Shader.hpp"
#include "TextureStorage.hpp"

#include <limits>

using namespace evo_engine;

namespace {
constexpr float kEnvironmentPi = 3.14159265358979323846f;

void SynchronizeCubemapResourceMutation() {
  const auto action = [] {
    Platform::WaitForDeviceIdle();
  };
  if (const auto gpu_service = Platform::TryGetGpuService();
      gpu_service && gpu_service->Initialized() && !gpu_service->IsGpuThread()) {
    GpuWorkOptions options;
    options.debug_name = "Cubemap::SynchronizeResourceMutation";
    const auto handle = gpu_service->Enqueue(options, action);
    gpu_service->Wait(handle);
  } else {
    action();
  }
}

size_t CalculateCubemapPixelCount(const uint32_t resolution, const uint32_t mip_levels) {
  if (resolution == 0 || mip_levels == 0) {
    return 0;
  }
  uint32_t max_mip_levels = 1;
  for (uint32_t size = resolution; size > 1; size /= 2) {
    ++max_mip_levels;
  }
  if (mip_levels > max_mip_levels) {
    return 0;
  }
  size_t face_pixels = 0;
  uint32_t size = resolution;
  for (uint32_t mip_level = 0; mip_level < mip_levels; ++mip_level) {
    if (size > std::numeric_limits<size_t>::max() / size) {
      return 0;
    }
    const size_t mip_pixels = static_cast<size_t>(size) * size;
    if (face_pixels > std::numeric_limits<size_t>::max() - mip_pixels) {
      return 0;
    }
    face_pixels += mip_pixels;
    size = glm::max(size / 2, 1u);
  }
  if (face_pixels > std::numeric_limits<size_t>::max() / 6) {
    return 0;
  }
  const size_t pixel_count = face_pixels * 6;
  return pixel_count <= std::numeric_limits<size_t>::max() / sizeof(glm::vec4) ? pixel_count : 0;
}

std::vector<VkBufferImageCopy> BuildCubemapCopyRegions(const uint32_t resolution, const uint32_t mip_levels) {
  std::vector<VkBufferImageCopy> regions;
  regions.reserve(static_cast<size_t>(mip_levels) * 6);
  VkDeviceSize offset = 0;
  for (uint32_t face = 0; face < 6; ++face) {
    uint32_t size = resolution;
    for (uint32_t mip_level = 0; mip_level < mip_levels; ++mip_level) {
      VkBufferImageCopy region{};
      region.bufferOffset = offset;
      region.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
      region.imageSubresource.mipLevel = mip_level;
      region.imageSubresource.baseArrayLayer = face;
      region.imageSubresource.layerCount = 1;
      region.imageExtent = {size, size, 1};
      regions.emplace_back(region);
      offset += static_cast<VkDeviceSize>(size) * size * sizeof(glm::vec4);
      size = glm::max(size / 2, 1u);
    }
  }
  return regions;
}

float EnvironmentLuminance(const glm::vec3& value) {
  return glm::max(glm::dot(glm::max(value, glm::vec3(0.0f)), glm::vec3(0.2126f, 0.7152f, 0.0722f)), 0.0f);
}

float CalculateEnvironmentPdfScale(const std::shared_ptr<Texture2D>& texture) {
  if (!texture)
    return 0.0f;
  const auto resolution = texture->GetResolution();
  const auto& pixels = texture->GetLocalData();
  if (resolution.x == 0 || resolution.y == 0 || pixels.size() != static_cast<size_t>(resolution.x) * resolution.y)
    return 0.0f;

  const float d_azimuth = 2.0f * kEnvironmentPi / static_cast<float>(resolution.x);
  double integral = 0.0;
  for (uint32_t y = 0; y < resolution.y; ++y) {
    const float elevation_0 = (static_cast<float>(y) / static_cast<float>(resolution.y) - 0.5f) * kEnvironmentPi;
    const float elevation_1 = (static_cast<float>(y + 1u) / static_cast<float>(resolution.y) - 0.5f) * kEnvironmentPi;
    const float solid_angle = d_azimuth * (glm::sin(elevation_1) - glm::sin(elevation_0));
    for (uint32_t x = 0; x < resolution.x; ++x) {
      integral += EnvironmentLuminance(glm::vec3(pixels[static_cast<size_t>(y) * resolution.x + x])) * solid_angle;
    }
  }
  return integral > 0.0 ? 1.0f / static_cast<float>(integral) : 0.0f;
}
}  // namespace

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
  if (Platform::Initialized() && PeekStorage().image) {
    SynchronizeCubemapResourceMutation();
  }
  resolution_ = resolution;
  mip_levels_ = mip_levels;
  local_data_.clear();
  local_data_dirty_ = false;
  gpu_content_valid_ = false;
  RefStorage().Initialize(resolution, mip_levels);
}

void Cubemap::UploadLocalData() const {
  const size_t pixel_count = CalculatePixelCount(resolution_, mip_levels_);
  if (!Platform::Initialized() || !local_data_dirty_ || local_data_.size() != pixel_count) {
    return;
  }
  SynchronizeCubemapResourceMutation();
  auto& storage = RefStorage();
  gpu_content_valid_ = false;
  storage.Initialize(resolution_, mip_levels_);
  if (!storage.image) {
    return;
  }
  Buffer staging_buffer(pixel_count * sizeof(glm::vec4));
  staging_buffer.UploadVector(local_data_);
  const auto copy_regions = BuildCubemapCopyRegions(resolution_, mip_levels_);
  Platform::ImmediateSubmit([&](const VkCommandBuffer vk_command_buffer) {
    storage.image->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
    storage.image->CopyFromBuffer(vk_command_buffer, staging_buffer.GetVkBuffer(), copy_regions);
    storage.image->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
  });
  local_data_dirty_ = false;
  MarkGpuContentValid();
}

void Cubemap::MarkGpuContentValid() const {
  gpu_content_valid_ = true;
}

void Cubemap::BeginGpuWrite() const {
  if (Platform::Initialized() && gpu_content_valid_ && PeekStorage().image) {
    SynchronizeCubemapResourceMutation();
  }
  local_data_.clear();
  local_data_dirty_ = false;
  gpu_content_valid_ = false;
}

bool Cubemap::SetRgbaChannelData(const std::vector<glm::vec4>& pixels, const uint32_t resolution,
                                 const uint32_t mip_levels) {
  const size_t pixel_count = CalculatePixelCount(resolution, mip_levels);
  if (pixel_count == 0 || pixels.size() != pixel_count) {
    return false;
  }
  resolution_ = resolution;
  mip_levels_ = mip_levels;
  local_data_ = pixels;
  local_data_dirty_ = true;
  gpu_content_valid_ = false;
  UploadLocalData();
  SetUnsaved();
  return true;
}

void Cubemap::Reset() {
  resolution_ = 0;
  mip_levels_ = 1;
  local_data_.clear();
  local_data_dirty_ = false;
  gpu_content_valid_ = false;
  if (Platform::Initialized()) {
    SynchronizeCubemapResourceMutation();
    RefStorage().Initialize(1, 1);
  }
  SetUnsaved();
}

void Cubemap::GetRgbaChannelData(std::vector<glm::vec4>& pixels, const bool force_gpu_readback) const {
  const size_t pixel_count = CalculatePixelCount(GetResolution(), GetMipLevels());
  if (pixel_count == 0) {
    pixels.clear();
    return;
  }
  if (!force_gpu_readback && local_data_.size() == pixel_count) {
    pixels = local_data_;
    return;
  }
  const auto& storage = PeekStorage();
  if (!Platform::Initialized() || !storage.image || !gpu_content_valid_ ||
      storage.image->GetLayout() == VK_IMAGE_LAYOUT_UNDEFINED) {
    pixels = local_data_.size() == pixel_count ? local_data_ : std::vector<glm::vec4>{};
    return;
  }
  SynchronizeCubemapResourceMutation();
  Buffer image_buffer(pixel_count * sizeof(glm::vec4));
  const auto copy_regions = BuildCubemapCopyRegions(GetResolution(), GetMipLevels());
  Platform::ImmediateSubmit([&](const VkCommandBuffer vk_command_buffer) {
    const auto previous_layout = storage.image->GetLayout();
    storage.image->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);
    vkCmdCopyImageToBuffer(vk_command_buffer, storage.image->GetVkImage(), VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                           image_buffer.GetVkBuffer(), static_cast<uint32_t>(copy_regions.size()), copy_regions.data());
    storage.image->TransitImageLayout(vk_command_buffer, previous_layout);
  });
  image_buffer.DownloadVector(local_data_, pixel_count);
  local_data_dirty_ = false;
  pixels = local_data_;
}

const std::vector<glm::vec4>& Cubemap::PeekLocalData() const {
  return local_data_;
}

size_t Cubemap::CalculatePixelCount(const uint32_t resolution, const uint32_t mip_levels) {
  return CalculateCubemapPixelCount(resolution, mip_levels);
}

uint32_t Cubemap::GetResolution() const {
  return resolution_;
}

uint32_t Cubemap::GetMipLevels() const {
  return mip_levels_;
}

uint32_t Cubemap::GetTextureStorageIndex() const {
  return texture_storage_handle_->value;
}

void Cubemap::BuildSkyIllumination(const SkyIllumination& sky_illumination, uint32_t resolution) const {
  const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>();
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
  if (!atmosphere_to_cubemap_pipeline_) {
    atmosphere_to_cubemap_pipeline_ = std::make_shared<GraphicsPipeline>();
    atmosphere_to_cubemap_pipeline_->vertex_shader =
        Shader::CreateTemporary(ShaderType::Vertex, Resources::GetDefaultResourcesPath() /
                                                        "Shaders/Graphics/Vertex/Lighting/AtmosphereToCubemap.vert");
    atmosphere_to_cubemap_pipeline_->fragment_shader = Shader::CreateTemporary(
        ShaderType::Fragment,
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Fragment/Lighting/AtmosphereToCubemap.frag");
    atmosphere_to_cubemap_pipeline_->geometry_type = GeometryType::Mesh;

    atmosphere_to_cubemap_pipeline_->depth_attachment_format = Platform::Constants::shadow_map;
    atmosphere_to_cubemap_pipeline_->stencil_attachment_format = VK_FORMAT_UNDEFINED;

    atmosphere_to_cubemap_pipeline_->color_attachment_formats = {1, Platform::Constants::texture_2d};
    atmosphere_to_cubemap_pipeline_->descriptor_set_layouts.emplace_back(
        ApplicationContext::Get().GetLayer<RenderLayer>()->GetRenderTexturePresentDescriptorSetLayout());

    auto& push_constant_range = atmosphere_to_cubemap_pipeline_->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(PushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;

    atmosphere_to_cubemap_pipeline_->Initialize();
  }
  BeginGpuWrite();
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
    viewport.width = static_cast<float>(storage.image->GetExtent().width);
    viewport.height = static_cast<float>(storage.image->GetExtent().height);
    viewport.minDepth = 0.0f;
    viewport.maxDepth = 1.0f;

    VkRect2D scissor;
    scissor.offset = {0, 0};
    scissor.extent.width = storage.image->GetExtent().width;
    scissor.extent.height = storage.image->GetExtent().height;
    atmosphere_to_cubemap_pipeline_->states.view_port = viewport;
    atmosphere_to_cubemap_pipeline_->states.scissor = scissor;
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
      atmosphere_to_cubemap_pipeline_->states.cull_mode = VK_CULL_MODE_NONE;
      atmosphere_to_cubemap_pipeline_->states.color_blend_attachment_states.clear();
      atmosphere_to_cubemap_pipeline_->states.color_blend_attachment_states.resize(1);
      for (auto& i : atmosphere_to_cubemap_pipeline_->states.color_blend_attachment_states) {
        i.colorWriteMask =
            VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
        i.blendEnable = VK_FALSE;
      }
      Platform::BeginRendering(vk_command_buffer, render_info);
      atmosphere_to_cubemap_pipeline_->Bind(vk_command_buffer);
      const auto mesh = Resources::GetInstance().GetRenderingCube();
      GeometryStorage::BindVertices(vk_command_buffer);
      push_constant.projection_view = capture_projection * capture_views[i];
      atmosphere_to_cubemap_pipeline_->PushConstant(vk_command_buffer, 0, push_constant);
      mesh->DrawIndexed(vk_command_buffer, atmosphere_to_cubemap_pipeline_->states, 1);
      Platform::EndRendering(vk_command_buffer);
#pragma endregion
      Platform::EverythingBarrier(vk_command_buffer);
    }
    storage.image->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
  });
  MarkGpuContentValid();
}

void Cubemap::ConvertFromEquirectangularTexture(const std::shared_ptr<Texture2D>& target_texture) const {
  const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>();
  if (!render_layer)
    return;
  if (!target_texture || !target_texture->GetImage()) {
    EVOENGINE_ERROR("Target texture doesn't contain any content!");
    return;
  }
  Initialize(1024);
  auto& storage = RefStorage();
  const float environment_pdf_scale = CalculateEnvironmentPdfScale(target_texture);
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

  const std::unique_ptr<DescriptorSet> temp_set = std::make_unique<DescriptorSet>(
      ApplicationContext::Get().GetLayer<RenderLayer>()->GetRenderTexturePresentDescriptorSetLayout());
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
  if (!equirectangular_to_cubemap_pipeline_) {
    equirectangular_to_cubemap_pipeline_ = std::make_shared<GraphicsPipeline>();
    equirectangular_to_cubemap_pipeline_->vertex_shader =
        Shader::CreateTemporary(ShaderType::Vertex, Resources::GetDefaultResourcesPath() /
                                                        "Shaders/Graphics/Vertex/Lighting/CubemapProcess.vert");
    equirectangular_to_cubemap_pipeline_->fragment_shader = Shader::CreateTemporary(
        ShaderType::Fragment,
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Fragment/Lighting/EquirectangularMapToCubemap.frag");
    equirectangular_to_cubemap_pipeline_->geometry_type = GeometryType::Mesh;

    equirectangular_to_cubemap_pipeline_->depth_attachment_format = Platform::Constants::shadow_map;
    equirectangular_to_cubemap_pipeline_->stencil_attachment_format = VK_FORMAT_UNDEFINED;

    equirectangular_to_cubemap_pipeline_->color_attachment_formats = {1, Platform::Constants::texture_2d};
    equirectangular_to_cubemap_pipeline_->descriptor_set_layouts.emplace_back(
        ApplicationContext::Get().GetLayer<RenderLayer>()->GetRenderTexturePresentDescriptorSetLayout());

    auto& push_constant_range = equirectangular_to_cubemap_pipeline_->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(glm::mat4) + sizeof(float);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;

    equirectangular_to_cubemap_pipeline_->Initialize();
  }
  BeginGpuWrite();
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
    viewport.width = static_cast<float>(storage.image->GetExtent().width);
    viewport.height = static_cast<float>(storage.image->GetExtent().height);
    viewport.minDepth = 0.0f;
    viewport.maxDepth = 1.0f;

    VkRect2D scissor;
    scissor.offset = {0, 0};
    scissor.extent.width = storage.image->GetExtent().width;
    scissor.extent.height = storage.image->GetExtent().height;
    equirectangular_to_cubemap_pipeline_->states.view_port = viewport;
    equirectangular_to_cubemap_pipeline_->states.scissor = scissor;
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
      equirectangular_to_cubemap_pipeline_->states.cull_mode = VK_CULL_MODE_NONE;
      equirectangular_to_cubemap_pipeline_->states.color_blend_attachment_states.clear();
      equirectangular_to_cubemap_pipeline_->states.color_blend_attachment_states.resize(1);
      for (auto& i : equirectangular_to_cubemap_pipeline_->states.color_blend_attachment_states) {
        i.colorWriteMask =
            VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
        i.blendEnable = VK_FALSE;
      }
      Platform::BeginRendering(vk_command_buffer, render_info);
      equirectangular_to_cubemap_pipeline_->Bind(vk_command_buffer);
      equirectangular_to_cubemap_pipeline_->BindDescriptorSet(vk_command_buffer, 0, temp_set->GetVkDescriptorSet());
      const auto mesh = Resources::GetInstance().GetRenderingCube();
      GeometryStorage::BindVertices(vk_command_buffer);
      EquirectangularToCubemapConstant constant{};
      constant.projection_view = capture_projection * capture_views[i];
      constant.environment_pdf_scale = environment_pdf_scale;
      equirectangular_to_cubemap_pipeline_->PushConstant(vk_command_buffer, 0, constant);
      mesh->DrawIndexed(vk_command_buffer, equirectangular_to_cubemap_pipeline_->states, 1);
      Platform::EndRendering(vk_command_buffer);
#pragma endregion

      Platform::EverythingBarrier(vk_command_buffer);
    }
    storage.image->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
  });
  MarkGpuContentValid();
}

const std::shared_ptr<Image>& Cubemap::GetImage() const {
  UploadLocalData();
  auto& storage = RefStorage();
  return storage.image;
}

const std::shared_ptr<ImageView>& Cubemap::GetImageView() const {
  UploadLocalData();
  auto& storage = RefStorage();
  return storage.image_view;
}

const std::shared_ptr<Sampler>& Cubemap::GetSampler() const {
  UploadLocalData();
  auto& storage = RefStorage();
  return storage.sampler;
}

const std::vector<std::shared_ptr<ImageView>>& Cubemap::GetFaceViews() const {
  UploadLocalData();
  auto& storage = RefStorage();
  return storage.face_views;
}
