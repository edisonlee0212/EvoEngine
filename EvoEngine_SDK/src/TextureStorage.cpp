#include "TextureStorage.hpp"

#include "Application.hpp"
#include "EditorLayer.hpp"
#include "RenderLayer.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <utility>

using namespace evo_engine;

namespace {
std::array<VkFormat, 2> CompatibleViewFormats(const VkFormat format) {
  switch (format) {
    case VK_FORMAT_BC7_UNORM_BLOCK:
    case VK_FORMAT_BC7_SRGB_BLOCK:
      return {VK_FORMAT_BC7_UNORM_BLOCK, VK_FORMAT_BC7_SRGB_BLOCK};
    case VK_FORMAT_R8G8B8A8_UNORM:
    case VK_FORMAT_R8G8B8A8_SRGB:
      return {VK_FORMAT_R8G8B8A8_UNORM, VK_FORMAT_R8G8B8A8_SRGB};
    default:
      return {format, format};
  }
}

bool AreViewFormatsCompatible(const VkFormat image_format, const VkFormat view_format) {
  const auto formats = CompatibleViewFormats(image_format);
  return view_format == formats[0] || view_format == formats[1];
}

bool IsSampledDescriptorImageLayout(const VkImageLayout layout) {
  return layout == VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL || layout == VK_IMAGE_LAYOUT_READ_ONLY_OPTIMAL ||
         layout == VK_IMAGE_LAYOUT_GENERAL;
}

uint64_t MixTextureContentSignature(const uint64_t seed, const uint64_t value) {
  return seed ^ (value + 0x9e3779b97f4a7c15ull + (seed << 6u) + (seed >> 2u));
}

void RemoveImGuiTexture(const ImTextureID texture_id) {
  if (texture_id != 0 && ImGui::GetCurrentContext()) {
    ImGui_ImplVulkan_RemoveTexture(reinterpret_cast<VkDescriptorSet>(texture_id));
  }
}

class PendingGpuUploadCompletion {
  std::shared_ptr<std::atomic_size_t> counter_;
  std::shared_ptr<std::atomic_size_t> generation_;

 public:
  PendingGpuUploadCompletion(std::shared_ptr<std::atomic_size_t> counter,
                             std::shared_ptr<std::atomic_size_t> generation)
      : counter_(std::move(counter)), generation_(std::move(generation)) {
  }

  ~PendingGpuUploadCompletion() {
    if (counter_) {
      counter_->fetch_sub(1);
    }
    if (generation_) {
      generation_->fetch_add(1);
    }
  }

  PendingGpuUploadCompletion(const PendingGpuUploadCompletion&) = delete;
  PendingGpuUploadCompletion& operator=(const PendingGpuUploadCompletion&) = delete;
};

std::shared_ptr<std::vector<std::byte>> BuildTextureUploadBytes(const std::vector<glm::vec4>& data,
                                                                const glm::uvec2& resolution, const VkFormat format) {
  const auto pixel_size = static_cast<size_t>(resolution.x) * static_cast<size_t>(resolution.y);
  if (pixel_size == 0 || data.size() < pixel_size) {
    return {};
  }

  switch (format) {
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
    case VK_FORMAT_R8G8B8A8_SRGB: {
      auto bytes = std::make_shared<std::vector<std::byte>>(pixel_size * 4);
      Jobs::RunParallelFor(pixel_size, [&](const auto i) {
        for (size_t channel = 0; channel < 4; ++channel) {
          const auto encoded =
              static_cast<unsigned char>(glm::clamp(glm::round(data[i][channel] * 255.0f), 0.0f, 255.0f));
          (*bytes)[i * 4 + channel] = static_cast<std::byte>(encoded);
        }
      });
      return bytes;
    }
    default:
      throw std::runtime_error("Unsupported Texture2D upload format.");
  }
}

float SrgbToLinear(const float value) {
  return value <= 0.04045f ? value / 12.92f : std::pow((value + 0.055f) / 1.055f, 2.4f);
}

float LinearToSrgb(const float value) {
  return value <= 0.0031308f ? value * 12.92f : 1.055f * std::pow(value, 1.0f / 2.4f) - 0.055f;
}

std::vector<glm::vec4> DecodeSrgbPixels(const std::vector<glm::vec4>& encoded) {
  auto linear = encoded;
  Jobs::RunParallelFor(linear.size(), [&](const size_t index) {
    linear[index] = glm::vec4(SrgbToLinear(linear[index].r), SrgbToLinear(linear[index].g),
                              SrgbToLinear(linear[index].b), linear[index].a);
  });
  return linear;
}

struct CpuMipChain {
  std::shared_ptr<std::vector<std::byte>> bytes = std::make_shared<std::vector<std::byte>>();
  std::vector<VkBufferImageCopy> regions;
};

CpuMipChain BuildSrgbMipChain(const std::vector<glm::vec4>& data, const glm::uvec2& resolution) {
  CpuMipChain result;
  std::vector<glm::vec4> level(data.begin(), data.begin() + static_cast<size_t>(resolution.x) * resolution.y);
  glm::uvec2 level_resolution = resolution;
  while (true) {
    const auto level_bytes = BuildTextureUploadBytes(level, level_resolution, VK_FORMAT_R8G8B8A8_SRGB);
    VkBufferImageCopy region{};
    region.bufferOffset = result.bytes->size();
    region.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    region.imageSubresource.mipLevel = static_cast<uint32_t>(result.regions.size());
    region.imageSubresource.layerCount = 1;
    region.imageExtent = {level_resolution.x, level_resolution.y, 1};
    result.regions.emplace_back(region);
    result.bytes->insert(result.bytes->end(), level_bytes->begin(), level_bytes->end());
    if (level_resolution.x == 1 && level_resolution.y == 1) {
      break;
    }

    const glm::uvec2 next_resolution(glm::max(level_resolution.x / 2, 1u), glm::max(level_resolution.y / 2, 1u));
    std::vector<glm::vec4> next(static_cast<size_t>(next_resolution.x) * next_resolution.y);
    Jobs::RunParallelFor(next.size(), [&](const size_t index) {
      const uint32_t target_x = static_cast<uint32_t>(index) % next_resolution.x;
      const uint32_t target_y = static_cast<uint32_t>(index) / next_resolution.x;
      glm::vec4 sum(0.0f);
      float total_weight = 0.0f;
      const float source_x0 = static_cast<float>(target_x) * level_resolution.x / next_resolution.x;
      const float source_x1 = static_cast<float>(target_x + 1) * level_resolution.x / next_resolution.x;
      const float source_y0 = static_cast<float>(target_y) * level_resolution.y / next_resolution.y;
      const float source_y1 = static_cast<float>(target_y + 1) * level_resolution.y / next_resolution.y;
      for (uint32_t y = static_cast<uint32_t>(std::floor(source_y0)); y < static_cast<uint32_t>(std::ceil(source_y1));
           ++y) {
        const float y_weight =
            glm::max(0.0f, glm::min(source_y1, static_cast<float>(y + 1)) - glm::max(source_y0, static_cast<float>(y)));
        for (uint32_t x = static_cast<uint32_t>(std::floor(source_x0)); x < static_cast<uint32_t>(std::ceil(source_x1));
             ++x) {
          const float x_weight = glm::max(
              0.0f, glm::min(source_x1, static_cast<float>(x + 1)) - glm::max(source_x0, static_cast<float>(x)));
          const float weight = x_weight * y_weight;
          const auto& source = level[static_cast<size_t>(y) * level_resolution.x + x];
          sum += weight * glm::vec4(SrgbToLinear(source.r), SrgbToLinear(source.g), SrgbToLinear(source.b), source.a);
          total_weight += weight;
        }
      }
      const glm::vec4 average = sum / glm::max(total_weight, 1.0e-8f);
      next[index] = glm::vec4(LinearToSrgb(average.r), LinearToSrgb(average.g), LinearToSrgb(average.b), average.a);
    });
    level = std::move(next);
    level_resolution = next_resolution;
  }
  return result;
}

VkSamplerCreateInfo DefaultTextureSamplerCreateInfo() {
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
  sampler_info.minLod = 0.0f;
  sampler_info.maxLod = VK_LOD_CLAMP_NONE;
  sampler_info.mipLodBias = 0.0f;
  return sampler_info;
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

bool SupportsLinearBlitTextureFormat(const VkFormat format) {
  if (!Platform::Initialized()) {
    return false;
  }
  const auto& physical_device = Platform::GetSelectedPhysicalDevice();
  if (!physical_device || physical_device->vk_physical_device == VK_NULL_HANDLE) {
    return false;
  }

  VkFormatProperties format_properties{};
  vkGetPhysicalDeviceFormatProperties(physical_device->vk_physical_device, format, &format_properties);
  constexpr VkFormatFeatureFlags required_features = VK_FORMAT_FEATURE_BLIT_SRC_BIT | VK_FORMAT_FEATURE_BLIT_DST_BIT |
                                                     VK_FORMAT_FEATURE_SAMPLED_IMAGE_FILTER_LINEAR_BIT;
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
                                   const std::shared_ptr<std::atomic_size_t>& generation,
                                   const std::shared_ptr<std::vector<std::byte>>& upload_bytes,
                                   const bool generate_mipmaps, const std::string& debug_name,
                                   std::vector<VkBufferImageCopy> copy_regions = {}) {
  if (!target_image || !upload_bytes || upload_bytes->empty()) {
    return {};
  }

  pending_counter->fetch_add(1);
  generation->fetch_add(1);
  GpuWorkOptions options;
  options.debug_name = debug_name;
  auto& gpu_service = Platform::GetGpuService();
  try {
    return gpu_service.EnqueueStaging(
        upload_bytes->size(), options,
        [target_image, pending_counter, generation, upload_bytes, generate_mipmaps,
         copy_regions = std::move(copy_regions)]() {
          const PendingGpuUploadCompletion pending_completion(pending_counter, generation);
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
    generation->fetch_add(1);
    throw;
  }
}
}  // namespace

void CubemapStorage::Initialize(uint32_t resolution, uint32_t mip_levels, const VkFormat format) {
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
  image_info.format = format;
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
  view_info.format = format;
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
    face_view_info.format = format;
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

bool Texture2DStorage::SamplesLinearSrgb() const {
  if (!new_data_.empty()) {
    return new_data_samples_linear_srgb_ || new_data_format_ == VK_FORMAT_R8G8B8A8_SRGB;
  }
  if (!new_compressed_data_.empty()) {
    return new_compressed_format_ == VK_FORMAT_BC7_SRGB_BLOCK;
  }
  return samples_linear_srgb_;
}

void Texture2DStorage::Initialize(const glm::uvec2& resolution) {
  uint32_t mip_levels = 1;
  if (SupportsLinearBlitTextureFormat(Platform::Constants::texture_2d)) {
    for (auto dimension = glm::max(resolution.x, resolution.y); dimension > 1; dimension /= 2) {
      ++mip_levels;
    }
  }
  Initialize(resolution, Platform::Constants::texture_2d, true, mip_levels);
}

void Texture2DStorage::Initialize(const glm::uvec2& resolution, const VkFormat format, const bool storage_image,
                                  const uint32_t mip_levels) {
  if (!Platform::Initialized())
    return;
  RetireCurrentResources();
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
  if (storage_image || (resolved_mip_levels > 1 && SupportsLinearBlitTextureFormat(format))) {
    image_info.usage |= VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
  }
  if (storage_image) {
    image_info.usage |= VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_STORAGE_BIT;
  }
  image_info.samples = VK_SAMPLE_COUNT_1_BIT;
  image_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

  const auto compatible_formats = CompatibleViewFormats(format);
  VkImageFormatListCreateInfo format_list{};
  if (compatible_formats[0] != compatible_formats[1]) {
    image_info.flags |= VK_IMAGE_CREATE_MUTABLE_FORMAT_BIT;
    format_list.sType = VK_STRUCTURE_TYPE_IMAGE_FORMAT_LIST_CREATE_INFO;
    format_list.viewFormatCount = static_cast<uint32_t>(compatible_formats.size());
    format_list.pViewFormats = compatible_formats.data();
    image_info.pNext = &format_list;
  }

  image = std::make_shared<Image>(image_info);
  view_format_ = format;
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

  const auto sampler_info = sampler_create_info_.sType == VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO
                                ? sampler_create_info_
                                : DefaultTextureSamplerCreateInfo();
  sampler = std::make_shared<Sampler>(sampler_info);

  Platform::ImmediateSubmit([&](const VkCommandBuffer vk_command_buffer) {
    image->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
  });

  EditorLayer::UpdateTextureId(im_texture_id, sampler->GetVkSampler(), image_view->GetVkImageView(),
                               image->GetLayout());
}

GpuWorkHandle Texture2DStorage::SetDataAsync(const std::vector<glm::vec4>& data, const glm::uvec2& resolution,
                                             const VkFormat format, const bool samples_linear_srgb) {
  const size_t pixel_count = static_cast<size_t>(resolution.x) * resolution.y;
  if (!Platform::Initialized() || pixel_count == 0 || data.size() < pixel_count) {
    return {};
  }
  const auto resolved_format = format == VK_FORMAT_UNDEFINED ? Platform::Constants::texture_2d : format;
  if (!SupportsSampledTextureFormat(resolved_format)) {
    if (resolved_format != VK_FORMAT_R8G8B8A8_SRGB) {
      EVOENGINE_ERROR("Texture2D upload format is not supported by the selected device.")
    }
    return {};
  }
  samples_linear_srgb_ = samples_linear_srgb || resolved_format == VK_FORMAT_R8G8B8A8_SRGB;
  uint32_t mip_levels = 1;
  const bool supports_linear_blit = SupportsLinearBlitTextureFormat(resolved_format);
  if (supports_linear_blit || resolved_format == VK_FORMAT_R8G8B8A8_SRGB) {
    for (auto dimension = glm::max(resolution.x, resolution.y); dimension > 1; dimension /= 2) {
      ++mip_levels;
    }
  }
  Initialize(resolution, resolved_format, resolved_format == Platform::Constants::texture_2d, mip_levels);
  if (resolved_format == VK_FORMAT_R8G8B8A8_SRGB && !supports_linear_blit) {
    auto mip_chain = BuildSrgbMipChain(data, resolution);
    return EnqueueTextureUpload(image, gpu_upload_in_flight, gpu_upload_generation, mip_chain.bytes, false,
                                "Texture2DStorage::SetDataAsync (CPU sRGB mips)", std::move(mip_chain.regions));
  }
  auto upload_bytes = BuildTextureUploadBytes(data, resolution, resolved_format);
  if (!upload_bytes || upload_bytes->empty()) {
    return {};
  }

  return EnqueueTextureUpload(image, gpu_upload_in_flight, gpu_upload_generation, upload_bytes, true,
                              "Texture2DStorage::SetDataAsync");
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
  samples_linear_srgb_ = format == VK_FORMAT_BC7_SRGB_BLOCK;
  Initialize(resolution, format, false, resolved_mip_levels);
  auto upload_bytes = std::make_shared<std::vector<std::byte>>(data);
  auto copy_regions = BuildCompressedMipCopyRegions(resolution, resolved_mip_levels, format);
  return EnqueueTextureUpload(image, gpu_upload_in_flight, gpu_upload_generation, upload_bytes, false,
                              "Texture2DStorage::SetCompressedDataAsync", std::move(copy_regions));
}

void Texture2DStorage::Clear() {
  if (!Platform::Initialized())
    return;
  if (im_texture_id != 0) {
    RemoveImGuiTexture(im_texture_id);
    im_texture_id = 0;
  }
  sampler.reset();
  image_view.reset();
  image.reset();
  view_format_ = VK_FORMAT_UNDEFINED;
  for (const auto& retired : retired_texture_ids_) {
    if (retired.id != 0) {
      RemoveImGuiTexture(retired.id);
    }
  }
  retired_texture_ids_.clear();
  for (const auto& retired : retired_resources_) {
    if (retired.texture_id != 0) {
      RemoveImGuiTexture(retired.texture_id);
    }
  }
  retired_resources_.clear();
  retired_samplers_.clear();
}

void Texture2DStorage::RetireCurrentResources() {
  if (!image && !image_view && !sampler && im_texture_id == 0) {
    return;
  }
  retired_resources_.push_back({std::move(image), std::move(image_view), std::move(sampler), im_texture_id,
                                Platform::GetMaxFramesInFlight() + 1u});
  im_texture_id = 0;
  view_format_ = VK_FORMAT_UNDEFINED;
}

bool Texture2DStorage::ShareImage(const Texture2DStorage& source, const VkFormat view_format,
                                  const VkSamplerCreateInfo& sampler_create_info) {
  if (this == &source || !Platform::Initialized() || !source.image || source.IsGpuUploadPending() ||
      source.GetLayout() != VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL ||
      !AreViewFormatsCompatible(source.image->GetFormat(), view_format)) {
    return false;
  }

  VkImageViewCreateInfo view_info{};
  view_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
  view_info.image = source.image->GetVkImage();
  view_info.viewType = VK_IMAGE_VIEW_TYPE_2D;
  view_info.format = view_format;
  view_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
  view_info.subresourceRange.levelCount = source.image->GetMipLevels();
  view_info.subresourceRange.layerCount = 1;
  auto replacement_view = std::make_shared<ImageView>(view_info);
  auto replacement_sampler = std::make_shared<Sampler>(sampler_create_info);
  if (replacement_view->GetVkImageView() == VK_NULL_HANDLE || replacement_sampler->GetVkSampler() == VK_NULL_HANDLE) {
    return false;
  }

  RetireCurrentResources();
  image = source.image;
  image_view = std::move(replacement_view);
  sampler = std::move(replacement_sampler);
  view_format_ = view_format;
  samples_linear_srgb_ = view_format == VK_FORMAT_BC7_SRGB_BLOCK || view_format == VK_FORMAT_R8G8B8A8_SRGB;
  sampler_create_info_ = sampler_create_info;
  sampler_create_info_.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
  sampler_create_info_.pNext = nullptr;
  EditorLayer::UpdateTextureId(im_texture_id, sampler->GetVkSampler(), image_view->GetVkImageView(),
                               image->GetLayout());
  TextureStorage::GetInstance().version_++;
  return true;
}

void CubemapStorage::Clear() {
  if (!Platform::Initialized())
    return;
  for (auto& im_texture_id : im_texture_ids) {
    if (im_texture_id != 0) {
      RemoveImGuiTexture(im_texture_id);
      im_texture_id = 0;
    }
  }
  sampler.reset();
  image_view.reset();
  image.reset();
  face_views.clear();
}
void Texture2DStorage::SetData(const std::vector<glm::vec4>& data, const glm::uvec2& resolution, const VkFormat format,
                               const bool samples_linear_srgb) {
  new_data_ = data;
  new_resolution_ = resolution;
  new_data_format_ = format;
  new_data_samples_linear_srgb_ = samples_linear_srgb || format == VK_FORMAT_R8G8B8A8_SRGB;
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
  new_data_format_ = VK_FORMAT_UNDEFINED;
  new_data_samples_linear_srgb_ = false;
  samples_linear_srgb_ = format == VK_FORMAT_BC7_SRGB_BLOCK;
}

VkFormat Texture2DStorage::GetFormat() const {
  if (image) {
    return view_format_ == VK_FORMAT_UNDEFINED ? image->GetFormat() : view_format_;
  }
  if (!new_compressed_data_.empty()) {
    return new_compressed_format_;
  }
  if (!new_data_.empty()) {
    return new_data_format_ == VK_FORMAT_UNDEFINED ? Platform::Constants::texture_2d : new_data_format_;
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
    const auto data_format = new_data_format_;
    upload = SetDataAsync(new_data_, new_resolution_, data_format, new_data_samples_linear_srgb_);
    if (!upload.Valid() && data_format == VK_FORMAT_R8G8B8A8_SRGB) {
      upload = SetDataAsync(DecodeSrgbPixels(new_data_), new_resolution_, VK_FORMAT_UNDEFINED, true);
    }
    new_data_.clear();
    new_resolution_ = {};
    new_data_format_ = VK_FORMAT_UNDEFINED;
    new_data_samples_linear_srgb_ = false;
  }
  if (upload.Valid()) {
    Platform::GetGpuService().Wait(upload);
  }
}

void Texture2DStorage::SetSampler(const VkSamplerCreateInfo& sampler_create_info) {
  sampler_create_info_ = sampler_create_info;
  sampler_create_info_.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
  sampler_create_info_.pNext = nullptr;
  if (!Platform::Initialized() || !image || !image_view) {
    return;
  }
  auto replacement_sampler = std::make_shared<Sampler>(sampler_create_info_);
  const auto previous_texture_id = im_texture_id;
  im_texture_id = 0;
  EditorLayer::UpdateTextureId(im_texture_id, replacement_sampler->GetVkSampler(), image_view->GetVkImageView(),
                               image->GetLayout());
  if (previous_texture_id != 0) {
    retired_texture_ids_.push_back({previous_texture_id, Platform::GetMaxFramesInFlight() + 1u});
  }
  if (sampler) {
    retired_samplers_.push_back({sampler, Platform::GetMaxFramesInFlight() + 1u});
  }
  sampler = std::move(replacement_sampler);
  TextureStorage::GetInstance().version_++;
}

uint32_t TextureStorage::GetVersion() {
  return GetInstance().version_;
}

bool TextureStorage::TryGetTexture2DContentSignature(const uint32_t texture_index, uint64_t& signature) {
  const auto& textures = GetInstance().texture_2ds_;
  if (texture_index >= textures.size() || textures[texture_index].pending_delete || !textures[texture_index].handle) {
    return false;
  }
  const auto& texture = textures[texture_index];
  signature = static_cast<uint64_t>(static_cast<uint32_t>(texture.handle->value));
  signature = MixTextureContentSignature(signature, reinterpret_cast<uintptr_t>(texture.image.get()));
  signature = MixTextureContentSignature(signature, reinterpret_cast<uintptr_t>(texture.image_view.get()));
  signature = MixTextureContentSignature(signature, reinterpret_cast<uintptr_t>(texture.sampler.get()));
  signature = MixTextureContentSignature(signature, static_cast<uint32_t>(texture.view_format_));
  signature = MixTextureContentSignature(signature, texture.samples_linear_srgb_ ? 1u : 0u);
  signature = MixTextureContentSignature(signature, texture.gpu_upload_generation->load());
  const auto& sampler = texture.sampler_create_info_;
  signature = MixTextureContentSignature(signature, sampler.flags);
  signature = MixTextureContentSignature(signature, sampler.magFilter);
  signature = MixTextureContentSignature(signature, sampler.minFilter);
  signature = MixTextureContentSignature(signature, sampler.mipmapMode);
  signature = MixTextureContentSignature(signature, sampler.addressModeU);
  signature = MixTextureContentSignature(signature, sampler.addressModeV);
  signature = MixTextureContentSignature(signature, sampler.addressModeW);
  signature = MixTextureContentSignature(signature, glm::floatBitsToUint(sampler.mipLodBias));
  signature = MixTextureContentSignature(signature, sampler.anisotropyEnable);
  signature = MixTextureContentSignature(signature, glm::floatBitsToUint(sampler.maxAnisotropy));
  signature = MixTextureContentSignature(signature, sampler.compareEnable);
  signature = MixTextureContentSignature(signature, sampler.compareOp);
  signature = MixTextureContentSignature(signature, glm::floatBitsToUint(sampler.minLod));
  signature = MixTextureContentSignature(signature, glm::floatBitsToUint(sampler.maxLod));
  signature = MixTextureContentSignature(signature, sampler.borderColor);
  signature = MixTextureContentSignature(signature, sampler.unnormalizedCoordinates);
  return true;
}

bool TextureStorage::TryGetCubemapContentSignature(const uint32_t texture_index, uint64_t& signature) {
  const auto& cubemaps = GetInstance().cubemaps_;
  if (texture_index >= cubemaps.size() || cubemaps[texture_index].pending_delete || !cubemaps[texture_index].handle) {
    return false;
  }
  const auto& cubemap = cubemaps[texture_index];
  signature = static_cast<uint64_t>(static_cast<uint32_t>(cubemap.handle->value));
  signature = MixTextureContentSignature(signature, reinterpret_cast<uintptr_t>(cubemap.image.get()));
  signature = MixTextureContentSignature(signature, reinterpret_cast<uintptr_t>(cubemap.image_view.get()));
  signature = MixTextureContentSignature(signature, reinterpret_cast<uintptr_t>(cubemap.sampler.get()));
  signature = MixTextureContentSignature(signature, cubemap.content_generation);
  return true;
}

bool TextureStorage::HasPendingTexture2DUpload(const uint32_t texture_index) {
  const auto& textures = GetInstance().texture_2ds_;
  if (texture_index >= textures.size() || textures[texture_index].pending_delete) {
    return false;
  }
  const auto& texture = textures[texture_index];
  return !texture.new_data_.empty() || !texture.new_compressed_data_.empty() || texture.IsGpuUploadPending();
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

bool TextureStorage::HasPendingDeletes() {
  const auto& storage = GetInstance();
  return std::any_of(storage.texture_2ds_.begin(), storage.texture_2ds_.end(),
                     [](const auto& texture) {
                       return texture.pending_delete;
                     }) ||
         std::any_of(storage.cubemaps_.begin(), storage.cubemaps_.end(), [](const auto& texture) {
           return texture.pending_delete;
         });
}

void TextureStorage::DeviceSync() {
  if (!Platform::Initialized())
    return;
  auto& storage = GetInstance();
  for (int texture_index = 0; texture_index < storage.texture_2ds_.size(); texture_index++) {
    if (storage.texture_2ds_[texture_index].pending_delete) {
      storage.texture_2ds_[texture_index].Clear();
      if (texture_index != storage.texture_2ds_.size() - 1) {
        storage.texture_2ds_[texture_index] = std::move(storage.texture_2ds_.back());
        storage.texture_2ds_[texture_index].handle->value = texture_index;
      }
      storage.texture_2ds_.pop_back();
      storage.version_++;
      texture_index--;
    }
  }

  for (int texture_index = 0; texture_index < storage.texture_2ds_.size(); texture_index++) {
    auto& texture_storage = storage.texture_2ds_[texture_index];
    texture_storage.retired_samplers_.erase(
        std::remove_if(texture_storage.retired_samplers_.begin(), texture_storage.retired_samplers_.end(),
                       [](auto& retired) {
                         if (retired.remaining_frames == 0) {
                           return true;
                         }
                         --retired.remaining_frames;
                         return false;
                       }),
        texture_storage.retired_samplers_.end());
    texture_storage.retired_texture_ids_.erase(
        std::remove_if(texture_storage.retired_texture_ids_.begin(), texture_storage.retired_texture_ids_.end(),
                       [](auto& retired) {
                         if (retired.remaining_frames == 0) {
                           RemoveImGuiTexture(retired.id);
                           return true;
                         }
                         --retired.remaining_frames;
                         return false;
                       }),
        texture_storage.retired_texture_ids_.end());
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
      const auto data_format = texture_storage.new_data_format_;
      auto upload = texture_storage.SetDataAsync(texture_storage.new_data_, texture_storage.new_resolution_,
                                                 data_format, texture_storage.new_data_samples_linear_srgb_);
      if (!upload.Valid() && data_format == VK_FORMAT_R8G8B8A8_SRGB) {
        upload = texture_storage.SetDataAsync(DecodeSrgbPixels(texture_storage.new_data_),
                                              texture_storage.new_resolution_, VK_FORMAT_UNDEFINED, true);
      }
      texture_storage.new_data_.clear();
      texture_storage.new_resolution_ = {};
      texture_storage.new_data_format_ = VK_FORMAT_UNDEFINED;
      texture_storage.new_data_samples_linear_srgb_ = false;
      storage.version_++;
    }
    const auto upload_generation = texture_storage.gpu_upload_generation->load();
    if (texture_storage.gpu_upload_generation_last_sync_ != upload_generation) {
      storage.version_++;
      texture_storage.gpu_upload_generation_last_sync_ = upload_generation;
    }
    texture_storage.retired_resources_.erase(
        std::remove_if(texture_storage.retired_resources_.begin(), texture_storage.retired_resources_.end(),
                       [&texture_storage](auto& retired) {
                         if (texture_storage.IsGpuUploadPending()) {
                           return false;
                         }
                         if (retired.remaining_frames == 0) {
                           if (retired.texture_id != 0) {
                             RemoveImGuiTexture(retired.texture_id);
                           }
                           return true;
                         }
                         --retired.remaining_frames;
                         return false;
                       }),
        texture_storage.retired_resources_.end());
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
  if (!IsSampledDescriptorImageLayout(layout) || texture_storage.GetVkImageView() == VK_NULL_HANDLE ||
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
  if (!IsSampledDescriptorImageLayout(layout) || texture_storage.GetVkImageView() == VK_NULL_HANDLE ||
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
    VkDescriptorImageInfo image_info;
    if (TryGetCubemapDescriptorImageInfo(static_cast<uint32_t>(texture_index), image_info)) {
      descriptor_set->UpdateImageDescriptorBinding(binding, image_info, texture_index);
    }
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
  storage.cubemaps_.back().Initialize(1, 1, Platform::Constants::texture_2d);
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
