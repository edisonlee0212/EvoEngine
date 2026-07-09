#include "TextureStorage.hpp"

#include "Application.hpp"
#include "EditorLayer.hpp"
#include "RenderLayer.hpp"

#include <utility>

using namespace evo_engine;

namespace {
class PendingGpuUploadCompletion {
  std::shared_ptr<std::atomic_size_t> counter_;

 public:
  explicit PendingGpuUploadCompletion(std::shared_ptr<std::atomic_size_t> counter) : counter_(std::move(counter)) {
  }

  ~PendingGpuUploadCompletion() {
    if (counter_) {
      counter_->fetch_sub(1);
    }
  }

  PendingGpuUploadCompletion(const PendingGpuUploadCompletion&) = delete;
  PendingGpuUploadCompletion& operator=(const PendingGpuUploadCompletion&) = delete;
};

std::shared_ptr<std::vector<std::byte>> BuildTextureUploadBytes(const std::vector<glm::vec4>& data,
                                                                const glm::uvec2& resolution) {
  const auto pixel_size = static_cast<size_t>(resolution.x) * static_cast<size_t>(resolution.y);
  if (pixel_size == 0 || data.size() < pixel_size) {
    return {};
  }

  switch (Platform::Constants::texture_2d) {
    case VK_FORMAT_R32G32B32A32_SFLOAT: {
      auto bytes = std::make_shared<std::vector<std::byte>>(pixel_size * sizeof(glm::vec4));
      memcpy(bytes->data(), data.data(), bytes->size());
      return bytes;
    }
    case VK_FORMAT_R16G16B16A16_SFLOAT: {
      std::vector<glm::detail::hdata> half_size_data(4 * pixel_size);
      Jobs::RunParallelFor(pixel_size, [&](const auto i) {
        half_size_data[i * 4] = glm::detail::toFloat16(data[i][0]);
        half_size_data[i * 4 + 1] = glm::detail::toFloat16(data[i][1]);
        half_size_data[i * 4 + 2] = glm::detail::toFloat16(data[i][2]);
        half_size_data[i * 4 + 3] = glm::detail::toFloat16(data[i][3]);
      });
      auto bytes = std::make_shared<std::vector<std::byte>>(half_size_data.size() * sizeof(glm::detail::hdata));
      memcpy(bytes->data(), half_size_data.data(), bytes->size());
      return bytes;
    }
    default:
      throw std::runtime_error("Unsupported Texture2D upload format.");
  }
}

bool SupportsSampledTextureFormat(const VkFormat format) {
  if (!Platform::Initialized()) {
    return false;
  }
  const auto& physical_device = Platform::GetSelectedPhysicalDevice();
  if (!physical_device || physical_device->vk_physical_device == VK_NULL_HANDLE) {
    return false;
  }

  VkFormatProperties format_properties{};
  vkGetPhysicalDeviceFormatProperties(physical_device->vk_physical_device, format, &format_properties);
  constexpr VkFormatFeatureFlags required_features =
      VK_FORMAT_FEATURE_TRANSFER_DST_BIT | VK_FORMAT_FEATURE_SAMPLED_IMAGE_BIT;
  return (format_properties.optimalTilingFeatures & required_features) == required_features;
}

size_t CompressedTextureBlockSize(const VkFormat format) {
  switch (format) {
    case VK_FORMAT_BC7_UNORM_BLOCK:
    case VK_FORMAT_BC7_SRGB_BLOCK:
      return 16;
    default:
      return 0;
  }
}

std::vector<VkBufferImageCopy> BuildCompressedMipCopyRegions(const glm::uvec2& resolution, const uint32_t mip_levels,
                                                             const VkFormat format) {
  std::vector<VkBufferImageCopy> regions;
  const auto block_size = CompressedTextureBlockSize(format);
  if (resolution.x == 0 || resolution.y == 0 || mip_levels == 0 || block_size == 0) {
    return regions;
  }

  regions.reserve(mip_levels);
  VkDeviceSize offset = 0;
  uint32_t width = resolution.x;
  uint32_t height = resolution.y;
  for (uint32_t mip_level = 0; mip_level < mip_levels; ++mip_level) {
    VkBufferImageCopy region{};
    region.bufferOffset = offset;
    region.bufferRowLength = 0;
    region.bufferImageHeight = 0;
    region.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    region.imageSubresource.mipLevel = mip_level;
    region.imageSubresource.baseArrayLayer = 0;
    region.imageSubresource.layerCount = 1;
    region.imageOffset = {0, 0, 0};
    region.imageExtent = {width, height, 1};
    regions.emplace_back(region);

    const size_t block_width = (static_cast<size_t>(width) + 3) / 4;
    const size_t block_height = (static_cast<size_t>(height) + 3) / 4;
    offset += block_width * block_height * block_size;
    width = width > 1 ? width / 2 : 1u;
    height = height > 1 ? height / 2 : 1u;
  }
  return regions;
}

GpuWorkHandle EnqueueTextureUpload(const std::shared_ptr<Image>& target_image,
                                   const std::shared_ptr<std::atomic_size_t>& pending_counter,
                                   const std::shared_ptr<std::vector<std::byte>>& upload_bytes,
                                   const bool generate_mipmaps, const std::string& debug_name,
                                   std::vector<VkBufferImageCopy> copy_regions = {}) {
  if (!target_image || !upload_bytes || upload_bytes->empty()) {
    return {};
  }

  pending_counter->fetch_add(1);
  GpuWorkOptions options;
  options.debug_name = debug_name;
  auto& gpu_service = Platform::GetGpuService();
  try {
    return gpu_service.EnqueueStaging(
        upload_bytes->size(), options,
        [target_image, pending_counter, upload_bytes, generate_mipmaps, copy_regions = std::move(copy_regions)]() {
          const PendingGpuUploadCompletion pending_completion(pending_counter);
          auto& gpu_service = Platform::GetGpuService();
          auto staging_buffer = gpu_service.AcquireStagingBuffer(upload_bytes->size(), false);
          try {
            void* mapping;
            Platform::CheckVk(vmaMapMemory(Platform::GetVmaAllocator(), staging_buffer.vma_allocation, &mapping));
            memcpy(mapping, upload_bytes->data(), upload_bytes->size());
            vmaUnmapMemory(Platform::GetVmaAllocator(), staging_buffer.vma_allocation);
            gpu_service.SubmitImmediate([target_image, staging_vk_buffer = staging_buffer.vk_buffer, generate_mipmaps,
                                         copy_regions](const VkCommandBuffer vk_command_buffer) {
              target_image->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
              if (copy_regions.empty()) {
                target_image->CopyFromBuffer(vk_command_buffer, staging_vk_buffer);
              } else {
                target_image->CopyFromBuffer(vk_command_buffer, staging_vk_buffer, copy_regions);
              }
              if (generate_mipmaps) {
                target_image->GenerateMipmaps(vk_command_buffer);
              } else {
                target_image->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
              }
            });
            gpu_service.ReleaseStagingBuffer(staging_buffer);
          } catch (...) {
            gpu_service.ReleaseStagingBuffer(staging_buffer);
            throw;
          }
        });
  } catch (...) {
    pending_counter->fetch_sub(1);
    throw;
  }
}
}  // namespace

void CubemapStorage::Initialize(uint32_t resolution, uint32_t mip_levels) {
  if (!Platform::Initialized())
    return;
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
  sampler_info.maxAnisotropy = Platform::GetSelectedPhysicalDevice()->properties.limits.maxSamplerAnisotropy;
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

bool Texture2DStorage::IsGpuUploadPending() const {
  return gpu_upload_in_flight && gpu_upload_in_flight->load() != 0;
}

void Texture2DStorage::Initialize(const glm::uvec2& resolution) {
  Initialize(resolution, Platform::Constants::texture_2d, true);
}

void Texture2DStorage::Initialize(const glm::uvec2& resolution, const VkFormat format, const bool storage_image,
                                  const uint32_t mip_levels) {
  if (!Platform::Initialized())
    return;
  Clear();
  const uint32_t resolved_mip_levels = mip_levels > 0 ? mip_levels : 1u;
  VkImageCreateInfo image_info{};
  image_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
  image_info.imageType = VK_IMAGE_TYPE_2D;
  image_info.extent.width = resolution.x;
  image_info.extent.height = resolution.y;
  image_info.extent.depth = 1;
  image_info.mipLevels = resolved_mip_levels;
  image_info.arrayLayers = 1;
  image_info.format = format;
  image_info.tiling = VK_IMAGE_TILING_OPTIMAL;
  image_info.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
  image_info.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
  if (storage_image) {
    image_info.usage |=
        VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_STORAGE_BIT;
  }
  image_info.samples = VK_SAMPLE_COUNT_1_BIT;
  image_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

  image = std::make_shared<Image>(image_info);
  VkImageViewCreateInfo view_info{};
  view_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
  view_info.image = image->GetVkImage();
  view_info.viewType = VK_IMAGE_VIEW_TYPE_2D;
  view_info.format = format;
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
  sampler_info.anisotropyEnable = VK_FALSE;
  sampler_info.maxAnisotropy = 1.0f;
  sampler_info.borderColor = VK_BORDER_COLOR_INT_OPAQUE_BLACK;
  sampler_info.unnormalizedCoordinates = VK_FALSE;
  sampler_info.compareEnable = VK_FALSE;
  sampler_info.compareOp = VK_COMPARE_OP_ALWAYS;
  sampler_info.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
  sampler_info.minLod = 0;
  sampler_info.maxLod = VK_LOD_CLAMP_NONE;
  sampler_info.mipLodBias = 0.0f;

  sampler = std::make_shared<Sampler>(sampler_info);

  Platform::ImmediateSubmit([&](const VkCommandBuffer vk_command_buffer) {
    image->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
  });

  EditorLayer::UpdateTextureId(im_texture_id, sampler->GetVkSampler(), image_view->GetVkImageView(),
                               image->GetLayout());
}

GpuWorkHandle Texture2DStorage::SetDataAsync(const std::vector<glm::vec4>& data, const glm::uvec2& resolution) {
  if (!Platform::Initialized() || data.empty() || resolution.x == 0 || resolution.y == 0) {
    return {};
  }
  Initialize(resolution);
  auto upload_bytes = BuildTextureUploadBytes(data, resolution);
  if (!upload_bytes || upload_bytes->empty()) {
    return {};
  }

  return EnqueueTextureUpload(image, gpu_upload_in_flight, upload_bytes, true, "Texture2DStorage::SetDataAsync");
}

GpuWorkHandle Texture2DStorage::SetCompressedDataAsync(const std::vector<std::byte>& data, const glm::uvec2& resolution,
                                                       const VkFormat format, const uint32_t mip_levels) {
  if (!Platform::Initialized() || data.empty() || resolution.x == 0 || resolution.y == 0 ||
      format == VK_FORMAT_UNDEFINED) {
    return {};
  }
  if (!SupportsSampledTextureFormat(format)) {
    EVOENGINE_ERROR("Texture2D compressed upload format is not supported by the selected device.")
    return {};
  }
  const uint32_t resolved_mip_levels = mip_levels > 0 ? mip_levels : 1u;
  Initialize(resolution, format, false, resolved_mip_levels);
  auto upload_bytes = std::make_shared<std::vector<std::byte>>(data);
  auto copy_regions = BuildCompressedMipCopyRegions(resolution, resolved_mip_levels, format);
  return EnqueueTextureUpload(image, gpu_upload_in_flight, upload_bytes, false,
                              "Texture2DStorage::SetCompressedDataAsync", std::move(copy_regions));
}

void Texture2DStorage::Clear() {
  if (!Platform::Initialized())
    return;
  if (im_texture_id != 0) {
    ImGui_ImplVulkan_RemoveTexture(reinterpret_cast<VkDescriptorSet>(im_texture_id));
    im_texture_id = 0;
  }
  sampler.reset();
  image_view.reset();
  image.reset();
}

void CubemapStorage::Clear() {
  if (!Platform::Initialized())
    return;
  for (auto& im_texture_id : im_texture_ids) {
    if (im_texture_id != 0) {
      ImGui_ImplVulkan_RemoveTexture(reinterpret_cast<VkDescriptorSet>(im_texture_id));
      im_texture_id = 0;
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
  new_compressed_data_.clear();
  new_compressed_resolution_ = {};
  new_compressed_format_ = VK_FORMAT_UNDEFINED;
  new_compressed_mip_levels_ = 1;
}

void Texture2DStorage::SetCompressedData(const std::vector<std::byte>& data, const glm::uvec2& resolution,
                                         const VkFormat format, const uint32_t mip_levels) {
  new_compressed_data_ = data;
  new_compressed_resolution_ = resolution;
  new_compressed_format_ = format;
  new_compressed_mip_levels_ = mip_levels > 0 ? mip_levels : 1u;
  new_data_.clear();
  new_resolution_ = {};
}

VkFormat Texture2DStorage::GetFormat() const {
  if (image) {
    return image->GetFormat();
  }
  if (!new_compressed_data_.empty()) {
    return new_compressed_format_;
  }
  return Platform::Constants::texture_2d;
}

uint32_t Texture2DStorage::GetMipLevels() const {
  if (image) {
    return image->GetMipLevels();
  }
  if (!new_compressed_data_.empty()) {
    return new_compressed_mip_levels_;
  }
  return 1;
}

void Texture2DStorage::UploadPendingDataImmediately() {
  if (new_data_.empty() && new_compressed_data_.empty()) {
    return;
  }
  GpuWorkHandle upload;
  if (!new_compressed_data_.empty()) {
    upload = SetCompressedDataAsync(new_compressed_data_, new_compressed_resolution_, new_compressed_format_,
                                    new_compressed_mip_levels_);
    new_compressed_data_.clear();
    new_compressed_resolution_ = {};
    new_compressed_format_ = VK_FORMAT_UNDEFINED;
    new_compressed_mip_levels_ = 1;
  } else {
    upload = SetDataAsync(new_data_, new_resolution_);
    new_data_.clear();
    new_resolution_ = {};
  }
  if (upload.Valid()) {
    Platform::GetGpuService().Wait(upload);
  }
}

uint32_t TextureStorage::GetVersion() {
  return GetInstance().version_;
}

bool TextureStorage::HasPendingUploads() {
  const auto& storage = GetInstance();
  for (const auto& texture_storage : storage.texture_2ds_) {
    if (!texture_storage.new_data_.empty() || !texture_storage.new_compressed_data_.empty() ||
        texture_storage.IsGpuUploadPending()) {
      return true;
    }
  }
  return false;
}

void TextureStorage::DeviceSync() {
  if (!Platform::Initialized())
    return;
  auto& storage = GetInstance();
  for (int texture_index = 0; texture_index < storage.texture_2ds_.size(); texture_index++) {
    if (const auto& texture_storage = storage.texture_2ds_[texture_index]; texture_storage.pending_delete) {
      storage.texture_2ds_[texture_index] = storage.texture_2ds_.back();
      storage.texture_2ds_[texture_index].handle->value = texture_index;
      storage.texture_2ds_.pop_back();
      storage.version_++;
      texture_index--;
    }
  }

  for (int texture_index = 0; texture_index < storage.texture_2ds_.size(); texture_index++) {
    auto& texture_storage = storage.texture_2ds_[texture_index];
    if (!texture_storage.new_compressed_data_.empty()) {
      (void)texture_storage.SetCompressedDataAsync(
          texture_storage.new_compressed_data_, texture_storage.new_compressed_resolution_,
          texture_storage.new_compressed_format_, texture_storage.new_compressed_mip_levels_);
      texture_storage.new_compressed_data_.clear();
      texture_storage.new_compressed_resolution_ = {};
      texture_storage.new_compressed_format_ = VK_FORMAT_UNDEFINED;
      texture_storage.new_compressed_mip_levels_ = 1;
      storage.version_++;
    } else if (!texture_storage.new_data_.empty()) {
      (void)texture_storage.SetDataAsync(texture_storage.new_data_, texture_storage.new_resolution_);
      texture_storage.new_data_.clear();
      texture_storage.new_resolution_ = {};
      storage.version_++;
    }
    const bool upload_pending = texture_storage.IsGpuUploadPending();
    if (texture_storage.gpu_upload_pending_last_sync_ && !upload_pending) {
      storage.version_++;
    }
    texture_storage.gpu_upload_pending_last_sync_ = upload_pending;
  }

  for (int texture_index = 0; texture_index < storage.cubemaps_.size(); texture_index++) {
    if (const auto& texture_storage = storage.cubemaps_[texture_index]; texture_storage.pending_delete) {
      storage.cubemaps_[texture_index] = storage.cubemaps_.back();
      storage.cubemaps_[texture_index].handle->value = texture_index;
      storage.cubemaps_.pop_back();
      storage.version_++;
      texture_index--;
    }
  }
}

void TextureStorage::BindTexture2DToDescriptorSet(const std::shared_ptr<DescriptorSet>& descriptor_set,
                                                  const uint32_t binding) {
  const auto& storage = GetInstance();
  for (int texture_index = 0; texture_index < storage.texture_2ds_.size(); texture_index++) {
    VkDescriptorImageInfo image_info;
    if (TryGetTexture2DDescriptorImageInfo(static_cast<uint32_t>(texture_index), image_info)) {
      descriptor_set->UpdateImageDescriptorBinding(binding, image_info, texture_index);
    }
  }
}

bool TextureStorage::TryGetTexture2DDescriptorImageInfo(const uint32_t texture_index,
                                                        VkDescriptorImageInfo& image_info) {
  const auto& storage = GetInstance();
  if (texture_index >= storage.texture_2ds_.size()) {
    return false;
  }
  const auto& texture_storage = storage.texture_2ds_[texture_index];
  if (!texture_storage.image || !texture_storage.image_view || !texture_storage.sampler ||
      texture_storage.IsGpuUploadPending()) {
    return false;
  }
  const auto layout = texture_storage.GetLayout();
  if (layout == VK_IMAGE_LAYOUT_UNDEFINED || texture_storage.GetVkImageView() == VK_NULL_HANDLE ||
      texture_storage.GetVkSampler() == VK_NULL_HANDLE) {
    return false;
  }
  image_info.imageLayout = layout;
  image_info.imageView = texture_storage.GetVkImageView();
  image_info.sampler = texture_storage.GetVkSampler();
  return true;
}

bool TextureStorage::TryGetCubemapDescriptorImageInfo(const uint32_t texture_index, VkDescriptorImageInfo& image_info) {
  const auto& storage = GetInstance();
  if (texture_index >= storage.cubemaps_.size()) {
    return false;
  }
  const auto& texture_storage = storage.cubemaps_[texture_index];
  const auto layout = texture_storage.GetLayout();
  if (layout == VK_IMAGE_LAYOUT_UNDEFINED || texture_storage.GetVkImageView() == VK_NULL_HANDLE ||
      texture_storage.GetVkSampler() == VK_NULL_HANDLE) {
    return false;
  }
  image_info.imageLayout = layout;
  image_info.imageView = texture_storage.GetVkImageView();
  image_info.sampler = texture_storage.GetVkSampler();
  return true;
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
  if (storage.initialized)
    storage.texture_2ds_[handle->value].pending_delete = true;
}

void TextureStorage::UnRegisterCubemap(const std::shared_ptr<TextureStorageHandle>& handle) {
  auto& storage = GetInstance();
  if (storage.initialized)
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
  auto& storage = GetInstance();
  storage.initialized = true;
}

void TextureStorage::OnDestroy() {
  auto& storage = GetInstance();
  storage.texture_2ds_.clear();
  storage.cubemaps_.clear();
  storage.initialized = false;
}
