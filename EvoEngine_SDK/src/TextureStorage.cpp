#include "TextureStorage.hpp"

#include "Application.hpp"
#include "EditorLayer.hpp"
#include "RenderLayer.hpp"
using namespace evo_engine;

void CubemapStorage::Initialize(uint32_t resolution, uint32_t mip_levels) {
  Clear();
  VkImageCreateInfo image_info{};
  image_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
  image_info.imageType = VK_IMAGE_TYPE_2D;
  image_info.extent.width = resolution;
  image_info.extent.height = resolution;
  image_info.extent.depth = 1;
  image_info.mipLevels = mip_levels;
  image_info.arrayLayers = 6;
  image_info.format = Platform::Constants::texture_2d;
  image_info.tiling = VK_IMAGE_TILING_OPTIMAL;
  image_info.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
  image_info.usage = VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT |
                     VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
  image_info.samples = VK_SAMPLE_COUNT_1_BIT;
  image_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  image_info.flags = VK_IMAGE_CREATE_CUBE_COMPATIBLE_BIT;
  image = std::make_shared<Image>(image_info);

  VkImageViewCreateInfo view_info{};
  view_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
  view_info.image = image->GetVkImage();
  view_info.viewType = VK_IMAGE_VIEW_TYPE_CUBE;
  view_info.format = Platform::Constants::texture_2d;
  view_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
  view_info.subresourceRange.baseMipLevel = 0;
  view_info.subresourceRange.levelCount = mip_levels;
  view_info.subresourceRange.baseArrayLayer = 0;
  view_info.subresourceRange.layerCount = 6;

  image_view = std::make_shared<ImageView>(view_info);

  VkSamplerCreateInfo sampler_info{};
  sampler_info.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
  sampler_info.magFilter = VK_FILTER_LINEAR;
  sampler_info.minFilter = VK_FILTER_LINEAR;
  sampler_info.addressModeU = VK_SAMPLER_ADDRESS_MODE_REPEAT;
  sampler_info.addressModeV = VK_SAMPLER_ADDRESS_MODE_REPEAT;
  sampler_info.addressModeW = VK_SAMPLER_ADDRESS_MODE_REPEAT;
  sampler_info.anisotropyEnable = VK_TRUE;
  sampler_info.maxAnisotropy =
      Platform::GetSelectedPhysicalDevice()->properties.limits.maxSamplerAnisotropy;
  sampler_info.borderColor = VK_BORDER_COLOR_INT_OPAQUE_BLACK;
  sampler_info.unnormalizedCoordinates = VK_FALSE;
  sampler_info.compareEnable = VK_FALSE;
  sampler_info.compareOp = VK_COMPARE_OP_ALWAYS;
  sampler_info.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
  if (mip_levels > 1) {
    sampler_info.minLod = 0;
    sampler_info.maxLod = static_cast<float>(mip_levels);
  }
  sampler = std::make_shared<Sampler>(sampler_info);

  Platform::ImmediateSubmit([&](const VkCommandBuffer vk_command_buffer) {
    image->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
  });

  for (int i = 0; i < 6; i++) {
    VkImageViewCreateInfo face_view_info{};
    face_view_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
    face_view_info.image = image->GetVkImage();
    face_view_info.viewType = VK_IMAGE_VIEW_TYPE_2D;
    face_view_info.format = Platform::Constants::texture_2d;
    face_view_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    face_view_info.subresourceRange.baseMipLevel = 0;
    face_view_info.subresourceRange.levelCount = 1;
    face_view_info.subresourceRange.baseArrayLayer = i;
    face_view_info.subresourceRange.layerCount = 1;

    face_views.emplace_back(std::make_shared<ImageView>(face_view_info));
  }

  im_texture_ids.resize(6);
  for (int i = 0; i < 6; i++) {
    EditorLayer::UpdateTextureId(im_texture_ids[i], sampler->GetVkSampler(), face_views[i]->GetVkImageView(),
                                 VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
  }
}

VkImageLayout CubemapStorage::GetLayout() const {
  return image->GetLayout();
}

VkImage CubemapStorage::GetVkImage() const {
  if (image) {
    return image->GetVkImage();
  }
  return VK_NULL_HANDLE;
}

VkImageView CubemapStorage::GetVkImageView() const {
  if (image_view) {
    return image_view->GetVkImageView();
  }
  return VK_NULL_HANDLE;
}

VkImageLayout Texture2DStorage::GetLayout() const {
  return image->GetLayout();
}

VkImage Texture2DStorage::GetVkImage() const {
  if (image) {
    return image->GetVkImage();
  }
  return VK_NULL_HANDLE;
}

VkImageView Texture2DStorage::GetVkImageView() const {
  if (image_view) {
    return image_view->GetVkImageView();
  }
  return VK_NULL_HANDLE;
}

VkSampler Texture2DStorage::GetVkSampler() const {
  if (sampler) {
    return sampler->GetVkSampler();
  }
  return VK_NULL_HANDLE;
}
VkSampler CubemapStorage::GetVkSampler() const {
  if (sampler) {
    return sampler->GetVkSampler();
  }
  return VK_NULL_HANDLE;
}
std::shared_ptr<Image> Texture2DStorage::GetImage() const {
  return image;
}
std::shared_ptr<Image> CubemapStorage::GetImage() const {
  return image;
}

void Texture2DStorage::Initialize(const glm::uvec2& resolution) {
  Clear();
  constexpr uint32_t mip_levels = 1;
  VkImageCreateInfo image_info{};
  image_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
  image_info.imageType = VK_IMAGE_TYPE_2D;
  image_info.extent.width = resolution.x;
  image_info.extent.height = resolution.y;
  image_info.extent.depth = 1;
  image_info.mipLevels = mip_levels;
  image_info.arrayLayers = 1;
  image_info.format = Platform::Constants::texture_2d;
  image_info.tiling = VK_IMAGE_TILING_OPTIMAL;
  image_info.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
  image_info.usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT |
                     VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_STORAGE_BIT;
  image_info.samples = VK_SAMPLE_COUNT_1_BIT;
  image_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

  image = std::make_shared<Image>(image_info);
  VkImageViewCreateInfo view_info{};
  view_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
  view_info.image = image->GetVkImage();
  view_info.viewType = VK_IMAGE_VIEW_TYPE_2D;
  view_info.format = Platform::Constants::texture_2d;
  view_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
  view_info.subresourceRange.baseMipLevel = 0;
  view_info.subresourceRange.levelCount = image_info.mipLevels;
  view_info.subresourceRange.baseArrayLayer = 0;
  view_info.subresourceRange.layerCount = 1;

  image_view = std::make_shared<ImageView>(view_info);

  VkSamplerCreateInfo sampler_info{};
  sampler_info.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
  sampler_info.magFilter = VK_FILTER_LINEAR;
  sampler_info.minFilter = VK_FILTER_LINEAR;
  sampler_info.addressModeU = VK_SAMPLER_ADDRESS_MODE_REPEAT;
  sampler_info.addressModeV = VK_SAMPLER_ADDRESS_MODE_REPEAT;
  sampler_info.addressModeW = VK_SAMPLER_ADDRESS_MODE_REPEAT;
  sampler_info.anisotropyEnable = VK_TRUE;
  sampler_info.maxAnisotropy =
      Platform::GetSelectedPhysicalDevice()->properties.limits.maxSamplerAnisotropy;
  sampler_info.borderColor = VK_BORDER_COLOR_INT_OPAQUE_BLACK;
  sampler_info.unnormalizedCoordinates = VK_FALSE;
  sampler_info.compareEnable = VK_FALSE;
  sampler_info.compareOp = VK_COMPARE_OP_ALWAYS;
  sampler_info.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
  sampler_info.minLod = 0;
  sampler_info.maxLod = static_cast<float>(image_info.mipLevels);
  sampler_info.mipLodBias = 0.0f;

  sampler = std::make_shared<Sampler>(sampler_info);

  Platform::ImmediateSubmit([&](const VkCommandBuffer vk_command_buffer) {
    image->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
  });

  EditorLayer::UpdateTextureId(im_texture_id, sampler->GetVkSampler(), image_view->GetVkImageView(),
                               image->GetLayout());
}

void Texture2DStorage::SetDataImmediately(const std::vector<glm::vec4>& data, const glm::uvec2& resolution) {
  UploadData(data, resolution);
}

void Texture2DStorage::UploadData(const std::vector<glm::vec4>& data, const glm::uvec2& resolution) {
  Initialize(resolution);
  const auto image_size = resolution.x * resolution.y * sizeof(glm::vec4);
  VkBufferCreateInfo staging_buffer_create_info{};
  staging_buffer_create_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  staging_buffer_create_info.size = image_size;
  staging_buffer_create_info.usage =
      VK_IMAGE_USAGE_STORAGE_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
  staging_buffer_create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  VmaAllocationCreateInfo staging_buffer_vma_allocation_create_info{};
  staging_buffer_vma_allocation_create_info.usage = VMA_MEMORY_USAGE_AUTO;
  staging_buffer_vma_allocation_create_info.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT;

  const Buffer staging_buffer{staging_buffer_create_info, staging_buffer_vma_allocation_create_info};
  void* device_data = nullptr;
  vmaMapMemory(Platform::GetVmaAllocator(), staging_buffer.GetVmaAllocation(), &device_data);
  memcpy(device_data, data.data(), image_size);
  vmaUnmapMemory(Platform::GetVmaAllocator(), staging_buffer.GetVmaAllocation());

  Platform::ImmediateSubmit([&](const VkCommandBuffer vk_command_buffer) {
    image->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
    image->CopyFromBuffer(vk_command_buffer, staging_buffer.GetVkBuffer());
    image->GenerateMipmaps(vk_command_buffer);
    image->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
  });
}

void Texture2DStorage::Clear() {
  if (im_texture_id != nullptr) {
    ImGui_ImplVulkan_RemoveTexture(static_cast<VkDescriptorSet>(im_texture_id));
    im_texture_id = nullptr;
  }
  sampler.reset();
  image_view.reset();
  image.reset();
}

void CubemapStorage::Clear() {
  for (auto& im_texture_id : im_texture_ids) {
    if (im_texture_id != nullptr) {
      ImGui_ImplVulkan_RemoveTexture(static_cast<VkDescriptorSet>(im_texture_id));
      im_texture_id = nullptr;
    }
  }
  sampler.reset();
  image_view.reset();
  image.reset();
  face_views.clear();
}
void Texture2DStorage::SetData(const std::vector<glm::vec4>& data, const glm::uvec2& resolution) {
  new_data_ = data;
  new_resolution_ = resolution;
}

void Texture2DStorage::UploadDataImmediately() {
  SetDataImmediately(new_data_, new_resolution_);
}

void TextureStorage::DeviceSync() {
  auto& storage = GetInstance();
  for (int texture_index = 0; texture_index < storage.texture_2ds_.size(); texture_index++) {
    if (const auto& texture_storage = storage.texture_2ds_[texture_index]; texture_storage.pending_delete) {
      storage.texture_2ds_[texture_index] = storage.texture_2ds_.back();
      storage.texture_2ds_[texture_index].handle->value = texture_index;
      storage.texture_2ds_.pop_back();
      texture_index--;
    }
  }

  for (int texture_index = 0; texture_index < storage.texture_2ds_.size(); texture_index++) {
    if (auto& texture_storage = storage.texture_2ds_[texture_index]; !texture_storage.new_data_.empty()) {
      texture_storage.UploadData(texture_storage.new_data_, texture_storage.new_resolution_);
      texture_storage.new_data_.clear();
      texture_storage.new_resolution_ = {};
    }
  }

  for (int texture_index = 0; texture_index < storage.cubemaps_.size(); texture_index++) {
    if (const auto& texture_storage = storage.cubemaps_[texture_index]; texture_storage.pending_delete) {
      storage.cubemaps_[texture_index] = storage.cubemaps_.back();
      storage.cubemaps_[texture_index].handle->value = texture_index;
      storage.cubemaps_.pop_back();
      texture_index--;
    }
  }

  for (int texture_index = 0; texture_index < storage.cubemaps_.size(); texture_index++) {
    auto& texture_storage = storage.cubemaps_[texture_index];
    /*
    if (!texture_storage.new_data_.empty()) {
      texture_storage.UploadData(texture_storage.new_data_, texture_storage.new_resolution_);
      texture_storage.new_data_.clear();
      texture_storage.new_resolution_ = {};
    }
    */
  }
}

void TextureStorage::BindTexture2DToDescriptorSet(const std::shared_ptr<DescriptorSet>& descriptor_set,
                                                  const uint32_t binding) {
  const auto& storage = GetInstance();
  for (int texture_index = 0; texture_index < storage.texture_2ds_.size(); texture_index++) {
    auto& texture_storage = storage.texture_2ds_[texture_index];
    if (texture_storage.GetLayout() == VK_IMAGE_LAYOUT_UNDEFINED)
      continue;
    VkDescriptorImageInfo image_info;
    image_info.imageLayout = texture_storage.GetLayout();
    image_info.imageView = texture_storage.GetVkImageView();
    image_info.sampler = texture_storage.GetVkSampler();
    descriptor_set->UpdateImageDescriptorBinding(binding, image_info, texture_index);
  }
}

void TextureStorage::BindCubemapToDescriptorSet(const std::shared_ptr<DescriptorSet>& descriptor_set,
                                                const uint32_t binding) {
  const auto& storage = GetInstance();
  for (int texture_index = 0; texture_index < storage.cubemaps_.size(); texture_index++) {
    auto& texture_storage = storage.cubemaps_[texture_index];
    if (texture_storage.GetLayout() == VK_IMAGE_LAYOUT_UNDEFINED)
      continue;
    VkDescriptorImageInfo image_info;
    image_info.imageLayout = texture_storage.GetLayout();
    image_info.imageView = texture_storage.GetVkImageView();
    image_info.sampler = texture_storage.GetVkSampler();
    descriptor_set->UpdateImageDescriptorBinding(binding, image_info, texture_index);
  }
}

const Texture2DStorage& TextureStorage::PeekTexture2DStorage(const std::shared_ptr<TextureStorageHandle>& handle) {
  auto& storage = GetInstance();
  return storage.texture_2ds_.at(handle->value);
}

Texture2DStorage& TextureStorage::RefTexture2DStorage(const std::shared_ptr<TextureStorageHandle>& handle) {
  auto& storage = GetInstance();
  return storage.texture_2ds_.at(handle->value);
}

const CubemapStorage& TextureStorage::PeekCubemapStorage(const std::shared_ptr<TextureStorageHandle>& handle) {
  auto& storage = GetInstance();
  return storage.cubemaps_.at(handle->value);
}

CubemapStorage& TextureStorage::RefCubemapStorage(const std::shared_ptr<TextureStorageHandle>& handle) {
  auto& storage = GetInstance();
  return storage.cubemaps_.at(handle->value);
}

void TextureStorage::UnRegisterTexture2D(const std::shared_ptr<TextureStorageHandle>& handle) {
  auto& storage = GetInstance();
  storage.texture_2ds_[handle->value].pending_delete = true;
}

void TextureStorage::UnRegisterCubemap(const std::shared_ptr<TextureStorageHandle>& handle) {
  auto& storage = GetInstance();
  storage.cubemaps_[handle->value].pending_delete = true;
}

std::shared_ptr<TextureStorageHandle> TextureStorage::RegisterTexture2D() {
  auto& storage = GetInstance();
  const auto ret_val = std::make_shared<TextureStorageHandle>();
  ret_val->value = storage.texture_2ds_.size();
  storage.texture_2ds_.emplace_back();
  auto& new_texture_2d_storage = storage.texture_2ds_.back();
  new_texture_2d_storage.handle = ret_val;
  storage.texture_2ds_.back().Initialize({1, 1});
  return ret_val;
}

std::shared_ptr<TextureStorageHandle> TextureStorage::RegisterCubemap() {
  auto& storage = GetInstance();
  const auto ret_val = std::make_shared<TextureStorageHandle>();
  ret_val->value = storage.cubemaps_.size();
  storage.cubemaps_.emplace_back();
  auto& new_cubemap_storage = storage.cubemaps_.back();
  new_cubemap_storage.handle = ret_val;
  storage.cubemaps_.back().Initialize(1, 1);
  return ret_val;
}

void TextureStorage::Initialize() {
}
