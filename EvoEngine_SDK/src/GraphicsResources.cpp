#include "GraphicsResources.hpp"

#include "Application.hpp"
#include "Console.hpp"
#include "Mesh.hpp"
#include "Platform.hpp"
#include "Utilities.hpp"
using namespace evo_engine;

Fence::Fence(const VkFenceCreateInfo& vk_fence_create_info) {
  Platform::CheckVk(vkCreateFence(Platform::GetVkDevice(), &vk_fence_create_info, nullptr, &vk_fence_));
  flags_ = vk_fence_create_info.flags;
}

Fence::~Fence() {
  if (vk_fence_ != VK_NULL_HANDLE) {
    vkDestroyFence(Platform::GetVkDevice(), vk_fence_, nullptr);
    vk_fence_ = nullptr;
  }
}

const VkFence& Fence::GetVkFence() const {
  return vk_fence_;
}

Semaphore::Semaphore(const VkSemaphoreCreateInfo& semaphore_create_info) {
  Platform::CheckVk(vkCreateSemaphore(Platform::GetVkDevice(), &semaphore_create_info, nullptr, &vk_semaphore_));
  flags_ = semaphore_create_info.flags;
}

Semaphore::~Semaphore() {
  if (vk_semaphore_ != VK_NULL_HANDLE) {
    vkDestroySemaphore(Platform::GetVkDevice(), vk_semaphore_, nullptr);
    vk_semaphore_ = VK_NULL_HANDLE;
  }
}

const VkSemaphore& Semaphore::GetVkSemaphore() const {
  return vk_semaphore_;
}
#ifdef _WIN64
void* Semaphore::GetVkSemaphoreHandle(VkExternalSemaphoreHandleTypeFlagBitsKHR external_semaphore_handle_type) const {
  void* handle;

  VkSemaphoreGetWin32HandleInfoKHR vulkan_semaphore_get_win32_handle_info_khr = {};
  vulkan_semaphore_get_win32_handle_info_khr.sType = VK_STRUCTURE_TYPE_SEMAPHORE_GET_WIN32_HANDLE_INFO_KHR;
  vulkan_semaphore_get_win32_handle_info_khr.pNext = nullptr;
  vulkan_semaphore_get_win32_handle_info_khr.semaphore = vk_semaphore_;
  vulkan_semaphore_get_win32_handle_info_khr.handleType = external_semaphore_handle_type;
  auto func =
      PFN_vkGetSemaphoreWin32HandleKHR(vkGetDeviceProcAddr(Platform::GetVkDevice(), "vkGetSemaphoreWin32HandleKHR"));
  func(Platform::GetVkDevice(), &vulkan_semaphore_get_win32_handle_info_khr, &handle);

  return handle;
}
#else
int Semaphore::GetVkSemaphoreHandle(VkExternalSemaphoreHandleTypeFlagBitsKHR externalSemaphoreHandleType) const {
  if (externalSemaphoreHandleType == VK_EXTERNAL_SEMAPHORE_HANDLE_TYPE_OPAQUE_FD_BIT) {
    int fd;

    VkSemaphoreGetFdInfoKHR vulkanSemaphoreGetFdInfoKHR = {};
    vulkanSemaphoreGetFdInfoKHR.sType = VK_STRUCTURE_TYPE_SEMAPHORE_GET_FD_INFO_KHR;
    vulkanSemaphoreGetFdInfoKHR.pNext = NULL;
    vulkanSemaphoreGetFdInfoKHR.semaphore = vk_semaphore_;
    vulkanSemaphoreGetFdInfoKHR.handleType = VK_EXTERNAL_SEMAPHORE_HANDLE_TYPE_OPAQUE_FD_BIT_KHR;

    vkGetSemaphoreFdKHR(Platform::GetVkDevice(), &vulkanSemaphoreGetFdInfoKHR, &fd);

    return fd;
  }
  return -1;
}
#endif
Swapchain::Swapchain(const VkSwapchainCreateInfoKHR& swap_chain_create_info) {
  const auto& device = Platform::GetVkDevice();
  Platform::CheckVk(vkCreateSwapchainKHR(Platform::GetVkDevice(), &swap_chain_create_info, nullptr, &vk_swapchain_));
  uint32_t image_count = 0;
  vkGetSwapchainImagesKHR(device, vk_swapchain_, &image_count, nullptr);
  vk_images_.resize(image_count);
  vkGetSwapchainImagesKHR(device, vk_swapchain_, &image_count, vk_images_.data());
  flags_ = swap_chain_create_info.flags;
  surface_ = swap_chain_create_info.surface;
  min_image_count_ = swap_chain_create_info.minImageCount;
  image_format_ = swap_chain_create_info.imageFormat;
  image_extent_ = swap_chain_create_info.imageExtent;
  image_array_layers_ = swap_chain_create_info.imageArrayLayers;
  image_usage_ = swap_chain_create_info.imageUsage;
  image_sharing_mode_ = swap_chain_create_info.imageSharingMode;
  ApplyVector(queue_family_indices_, swap_chain_create_info.queueFamilyIndexCount,
              swap_chain_create_info.pQueueFamilyIndices);
  pre_transform_ = swap_chain_create_info.preTransform;
  composite_alpha_ = swap_chain_create_info.compositeAlpha;
  present_mode_ = swap_chain_create_info.presentMode;
  clipped_ = swap_chain_create_info.clipped;

  vk_image_views_.clear();
  for (size_t i = 0; i < vk_images_.size(); i++) {
    VkImageViewCreateInfo image_view_create_info{};
    image_view_create_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
    image_view_create_info.image = vk_images_[i];
    image_view_create_info.viewType = VK_IMAGE_VIEW_TYPE_2D;
    image_view_create_info.format = image_format_;
    image_view_create_info.components.r = VK_COMPONENT_SWIZZLE_IDENTITY;
    image_view_create_info.components.g = VK_COMPONENT_SWIZZLE_IDENTITY;
    image_view_create_info.components.b = VK_COMPONENT_SWIZZLE_IDENTITY;
    image_view_create_info.components.a = VK_COMPONENT_SWIZZLE_IDENTITY;
    image_view_create_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    image_view_create_info.subresourceRange.baseMipLevel = 0;
    image_view_create_info.subresourceRange.levelCount = 1;
    image_view_create_info.subresourceRange.baseArrayLayer = 0;
    image_view_create_info.subresourceRange.layerCount = 1;
    auto image_view = std::make_shared<ImageView>(image_view_create_info);
    vk_image_views_.emplace_back(image_view);
  }
}

Swapchain::~Swapchain() {
  vk_image_views_.clear();
  if (vk_swapchain_ != VK_NULL_HANDLE) {
    vkDestroySwapchainKHR(Platform::GetVkDevice(), vk_swapchain_, nullptr);
    vk_swapchain_ = VK_NULL_HANDLE;
  }
}

VkSwapchainKHR Swapchain::GetVkSwapchain() const {
  return vk_swapchain_;
}

const std::vector<VkImage>& Swapchain::GetAllVkImages() const {
  return vk_images_;
}

const VkImage& Swapchain::GetVkImage() const {
  return vk_images_[Platform::GetNextImageIndex()];
}

const VkImageView& Swapchain::GetVkImageView() const {
  return vk_image_views_[Platform::GetNextImageIndex()]->vk_image_view_;
}

const std::vector<std::shared_ptr<ImageView>>& Swapchain::GetAllImageViews() const {
  return vk_image_views_;
}

VkFormat Swapchain::GetImageFormat() const {
  return image_format_;
}

VkExtent2D Swapchain::GetImageExtent() const {
  return image_extent_;
}

ImageView::ImageView(const VkImageViewCreateInfo& image_view_create_info) {
  Platform::CheckVk(vkCreateImageView(Platform::GetVkDevice(), &image_view_create_info, nullptr, &vk_image_view_));
  image_ = nullptr;
  flags_ = image_view_create_info.flags;
  view_type_ = image_view_create_info.viewType;
  format_ = image_view_create_info.format;
  components_ = image_view_create_info.components;
  subresource_range_ = image_view_create_info.subresourceRange;
}

ImageView::ImageView(const VkImageViewCreateInfo& image_view_create_info, const std::shared_ptr<Image>& image) {
  Platform::CheckVk(vkCreateImageView(Platform::GetVkDevice(), &image_view_create_info, nullptr, &vk_image_view_));
  image_ = image;
  flags_ = image_view_create_info.flags;
  view_type_ = image_view_create_info.viewType;
  format_ = image->GetFormat();
  components_ = image_view_create_info.components;
  subresource_range_ = image_view_create_info.subresourceRange;
}

ImageView::~ImageView() {
  if (vk_image_view_ != VK_NULL_HANDLE) {
    vkDestroyImageView(Platform::GetVkDevice(), vk_image_view_, nullptr);
    vk_image_view_ = VK_NULL_HANDLE;
  }
}

VkImageView ImageView::GetVkImageView() const {
  return vk_image_view_;
}

const std::shared_ptr<Image>& ImageView::GetImage() const {
  return image_;
}

ShaderModule::~ShaderModule() {
  if (vk_shader_module_ != VK_NULL_HANDLE) {
    vkDestroyShaderModule(Platform::GetVkDevice(), vk_shader_module_, nullptr);
    vk_shader_module_ = VK_NULL_HANDLE;
  }
}

ShaderModule::ShaderModule(const VkShaderModuleCreateInfo& create_info) {
  Platform::CheckVk(vkCreateShaderModule(Platform::GetVkDevice(), &create_info, nullptr, &vk_shader_module_));
}

VkShaderModule ShaderModule::GetVkShaderModule() const {
  return vk_shader_module_;
}

PipelineLayout::PipelineLayout(const VkPipelineLayoutCreateInfo& pipeline_layout_create_info) {
  Platform::CheckVk(
      vkCreatePipelineLayout(Platform::GetVkDevice(), &pipeline_layout_create_info, nullptr, &vk_pipeline_layout_));

  flags_ = pipeline_layout_create_info.flags;
  ApplyVector(set_layouts_, pipeline_layout_create_info.setLayoutCount, pipeline_layout_create_info.pSetLayouts);
  ApplyVector(push_constant_ranges_, pipeline_layout_create_info.pushConstantRangeCount,
              pipeline_layout_create_info.pPushConstantRanges);
}

PipelineLayout::~PipelineLayout() {
  if (vk_pipeline_layout_ != VK_NULL_HANDLE) {
    vkDestroyPipelineLayout(Platform::GetVkDevice(), vk_pipeline_layout_, nullptr);
    vk_pipeline_layout_ = VK_NULL_HANDLE;
  }
}

VkPipelineLayout PipelineLayout::GetVkPipelineLayout() const {
  return vk_pipeline_layout_;
}

CommandPool::CommandPool(const VkCommandPoolCreateInfo& command_pool_create_info) {
  Platform::CheckVk(
      vkCreateCommandPool(Platform::GetVkDevice(), &command_pool_create_info, nullptr, &vk_command_pool_));
}

CommandPool::~CommandPool() {
  if (vk_command_pool_ != VK_NULL_HANDLE) {
    vkDestroyCommandPool(Platform::GetVkDevice(), vk_command_pool_, nullptr);
    vk_command_pool_ = VK_NULL_HANDLE;
  }
}

VkCommandPool CommandPool::GetVkCommandPool() const {
  return vk_command_pool_;
}

uint32_t Image::GetMipLevels() const {
  return mip_levels_;
}

Image::Image(VkImageCreateInfo image_create_info) {
  VkExternalMemoryImageCreateInfo vk_external_mem_image_create_info = {};
  vk_external_mem_image_create_info.sType = VK_STRUCTURE_TYPE_EXTERNAL_MEMORY_IMAGE_CREATE_INFO;
  vk_external_mem_image_create_info.pNext = nullptr;
#ifdef _WIN64
  vk_external_mem_image_create_info.handleTypes = VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_WIN32_BIT;
#else
  vkExternalMemImageCreateInfo.handleTypes = VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_FD_BIT_KHR;
#endif

  image_create_info.pNext = &vk_external_mem_image_create_info;

  VmaAllocationCreateInfo alloc_info = {};
  alloc_info.usage = VMA_MEMORY_USAGE_AUTO;
  if (vmaCreateImage(Platform::GetVmaAllocator(), &image_create_info, &alloc_info, &vk_image_, &vma_allocation_,
                     &vma_allocation_info_)) {
    throw std::runtime_error("Failed to create image!");
  }
  flags_ = image_create_info.flags;
  image_type_ = image_create_info.imageType;
  format_ = image_create_info.format;
  extent_ = image_create_info.extent;
  mip_levels_ = image_create_info.mipLevels;
  array_layers_ = image_create_info.arrayLayers;
  samples_ = image_create_info.samples;
  tiling_ = image_create_info.tiling;
  usage_ = image_create_info.usage;
  sharing_mode_ = image_create_info.sharingMode;
  ApplyVector(queue_family_indices_, image_create_info.queueFamilyIndexCount, image_create_info.pQueueFamilyIndices);

  layout_ = initial_layout_ = image_create_info.initialLayout;
}

Image::Image(VkImageCreateInfo image_create_info, const VmaAllocationCreateInfo& vma_allocation_create_info) {
  VkExternalMemoryImageCreateInfo vk_external_mem_image_create_info = {};
  vk_external_mem_image_create_info.sType = VK_STRUCTURE_TYPE_EXTERNAL_MEMORY_IMAGE_CREATE_INFO;
  vk_external_mem_image_create_info.pNext = nullptr;
#ifdef _WIN64
  vk_external_mem_image_create_info.handleTypes = VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_WIN32_BIT;
#else
  vkExternalMemImageCreateInfo.handleTypes = VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_FD_BIT_KHR;
#endif

  image_create_info.pNext = &vk_external_mem_image_create_info;

  if (vmaCreateImage(Platform::GetVmaAllocator(), &image_create_info, &vma_allocation_create_info, &vk_image_,
                     &vma_allocation_, &vma_allocation_info_)) {
    throw std::runtime_error("Failed to create image!");
  }
  flags_ = image_create_info.flags;
  image_type_ = image_create_info.imageType;
  format_ = image_create_info.format;
  extent_ = image_create_info.extent;
  mip_levels_ = image_create_info.mipLevels;
  array_layers_ = image_create_info.arrayLayers;
  samples_ = image_create_info.samples;
  tiling_ = image_create_info.tiling;
  usage_ = image_create_info.usage;
  sharing_mode_ = image_create_info.sharingMode;
  ApplyVector(queue_family_indices_, image_create_info.queueFamilyIndexCount, image_create_info.pQueueFamilyIndices);

  layout_ = initial_layout_ = image_create_info.initialLayout;
}

bool Image::HasStencilComponent() const {
  return format_ == VK_FORMAT_D32_SFLOAT_S8_UINT || format_ == VK_FORMAT_D24_UNORM_S8_UINT;
}

void Image::CopyFromBuffer(const VkCommandBuffer vk_command_buffer, const VkBuffer& src_buffer,
                           VkDeviceSize src_offset) const {
  VkBufferImageCopy region{};
  region.bufferOffset = src_offset;
  region.bufferRowLength = 0;
  region.bufferImageHeight = 0;
  region.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
  region.imageSubresource.mipLevel = 0;
  region.imageSubresource.baseArrayLayer = 0;
  region.imageSubresource.layerCount = 1;
  region.imageOffset = {0, 0, 0};
  region.imageExtent = extent_;
  vkCmdCopyBufferToImage(vk_command_buffer, src_buffer, vk_image_, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &region);
}

void Image::GenerateMipmaps(const VkCommandBuffer vk_command_buffer) {
  VkImageMemoryBarrier barrier{};
  barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
  barrier.image = vk_image_;
  barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
  barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
  barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
  barrier.subresourceRange.baseArrayLayer = 0;
  barrier.subresourceRange.layerCount = array_layers_;
  barrier.subresourceRange.levelCount = 1;

  int32_t mip_width = extent_.width;
  int32_t mip_height = extent_.height;

  for (uint32_t i = 1; i < mip_levels_; i++) {
    barrier.subresourceRange.baseMipLevel = i - 1;
    barrier.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
    barrier.newLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
    barrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
    barrier.dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;

    vkCmdPipelineBarrier(vk_command_buffer, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0,
                         nullptr, 0, nullptr, 1, &barrier);

    VkImageBlit blit{};
    blit.srcOffsets[0] = {0, 0, 0};
    blit.srcOffsets[1] = {mip_width, mip_height, 1};
    blit.srcSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    blit.srcSubresource.mipLevel = i - 1;
    blit.srcSubresource.baseArrayLayer = 0;
    blit.srcSubresource.layerCount = array_layers_;
    blit.dstOffsets[0] = {0, 0, 0};
    blit.dstOffsets[1] = {mip_width > 1 ? mip_width / 2 : 1, mip_height > 1 ? mip_height / 2 : 1, 1};
    blit.dstSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    blit.dstSubresource.mipLevel = i;
    blit.dstSubresource.baseArrayLayer = 0;
    blit.dstSubresource.layerCount = array_layers_;

    vkCmdBlitImage(vk_command_buffer, vk_image_, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, vk_image_,
                   VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &blit, VK_FILTER_LINEAR);

    barrier.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
    barrier.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    barrier.srcAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
    barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;

    vkCmdPipelineBarrier(vk_command_buffer, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, 0, 0,
                         nullptr, 0, nullptr, 1, &barrier);

    if (mip_width > 1)
      mip_width /= 2;
    if (mip_height > 1)
      mip_height /= 2;
  }
  barrier.subresourceRange.baseMipLevel = mip_levels_ - 1;
  barrier.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
  barrier.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
  barrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
  barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;

  vkCmdPipelineBarrier(vk_command_buffer, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, 0, 0,
                       nullptr, 0, nullptr, 1, &barrier);
  layout_ = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
}

VkImage Image::GetVkImage() const {
  return vk_image_;
}

VkFormat Image::GetFormat() const {
  return format_;
}

VmaAllocation Image::GetVmaAllocation() const {
  return vma_allocation_;
}

VkExtent3D Image::GetExtent() const {
  return extent_;
}

VkImageLayout Image::GetLayout() const {
  return layout_;
}

Image::~Image() {
  if (vk_image_ != VK_NULL_HANDLE || vma_allocation_ != VK_NULL_HANDLE) {
    vmaDestroyImage(Platform::GetVmaAllocator(), vk_image_, vma_allocation_);
    vk_image_ = VK_NULL_HANDLE;
    vma_allocation_ = VK_NULL_HANDLE;
    vma_allocation_info_ = {};
  }
}

void Image::TransitImageLayout(VkCommandBuffer vk_command_buffer, const VkImageLayout new_layout) {
  // if (newLayout == layout_) return;
  Platform::TransitImageLayout(vk_command_buffer, vk_image_, format_, array_layers_, layout_, new_layout, mip_levels_);
  layout_ = new_layout;
}

const VmaAllocationInfo& Image::GetVmaAllocationInfo() const {
  return vma_allocation_info_;
}

#  ifdef _WIN64
void* Image::GetVkImageMemHandle(VkExternalMemoryHandleTypeFlagsKHR external_memory_handle_type) const {
#  if ENABLE_EXTERNAL_MEMORY
  void* handle;

  VkMemoryGetWin32HandleInfoKHR vk_memory_get_win32_handle_info_khr = {};
  vk_memory_get_win32_handle_info_khr.sType = VK_STRUCTURE_TYPE_MEMORY_GET_WIN32_HANDLE_INFO_KHR;
  vk_memory_get_win32_handle_info_khr.pNext = nullptr;
  vk_memory_get_win32_handle_info_khr.memory = vma_allocation_info_.deviceMemory;
  vk_memory_get_win32_handle_info_khr.handleType =
      static_cast<VkExternalMemoryHandleTypeFlagBitsKHR>(external_memory_handle_type);
  vkGetMemoryWin32HandleKHR(Platform::GetVkDevice(), &vk_memory_get_win32_handle_info_khr, &handle);
  return handle;
#else
  return nullptr;
#endif
}
#  else
int Image::GetVkImageMemHandle(VkExternalMemoryHandleTypeFlagsKHR externalMemoryHandleType) const {
#  if ENABLE_EXTERNAL_MEMORY
  if (externalMemoryHandleType == VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_FD_BIT_KHR) {
    int fd;

    VkMemoryGetFdInfoKHR vkMemoryGetFdInfoKHR = {};
    vkMemoryGetFdInfoKHR.sType = VK_STRUCTURE_TYPE_MEMORY_GET_FD_INFO_KHR;
    vkMemoryGetFdInfoKHR.pNext = NULL;
    vkMemoryGetFdInfoKHR.memory = vma_allocation_info_.deviceMemory;
    vkMemoryGetFdInfoKHR.handleType = VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_FD_BIT_KHR;

    vkGetMemoryFdKHR(Platform::GetVkDevice(), &vkMemoryGetFdInfoKHR, &fd);

    return fd;
  }
  return -1;
#  else
  return -1;
#  endif
}
#  endif
Sampler::Sampler(const VkSamplerCreateInfo& sampler_create_info) {
  Platform::CheckVk(vkCreateSampler(Platform::GetVkDevice(), &sampler_create_info, nullptr, &vk_sampler_));
}

Sampler::~Sampler() {
  if (vk_sampler_ != VK_NULL_HANDLE) {
    vkDestroySampler(Platform::GetVkDevice(), vk_sampler_, nullptr);
    vk_sampler_ = VK_NULL_HANDLE;
  }
}

VkSampler Sampler::GetVkSampler() const {
  return vk_sampler_;
}

void Buffer::UploadData(const size_t size, const void* src) {
  if (size > size_)
    Resize(size);
  if (vma_allocation_create_info_.flags & VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT ||
      vma_allocation_create_info_.flags & VMA_ALLOCATION_CREATE_HOST_ACCESS_RANDOM_BIT) {
    void* mapping;
    vmaMapMemory(Platform::GetVmaAllocator(), vma_allocation_, &mapping);
    memcpy(mapping, src, size);
    vmaUnmapMemory(Platform::GetVmaAllocator(), vma_allocation_);
  } else {
    Buffer staging_buffer(size);
    staging_buffer.UploadData(size, src);
    CopyFromBuffer(staging_buffer, size, 0, 0);
  }
}

void Buffer::DownloadData(const size_t size, void* dst) {
  if (size > size_)
    Resize(size);
  if (vma_allocation_create_info_.flags & VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT ||
      vma_allocation_create_info_.flags & VMA_ALLOCATION_CREATE_HOST_ACCESS_RANDOM_BIT) {
    void* mapping;
    vmaMapMemory(Platform::GetVmaAllocator(), vma_allocation_, &mapping);
    memcpy(dst, mapping, size);
    vmaUnmapMemory(Platform::GetVmaAllocator(), vma_allocation_);
  } else {
    Buffer staging_buffer(size);
    staging_buffer.CopyFromBuffer(*this, size, 0, 0);
    staging_buffer.DownloadData(size, dst);
  }
}

void Buffer::Allocate(VkBufferCreateInfo buffer_create_info,
                      const VmaAllocationCreateInfo& vma_allocation_create_info) {
#if ENABLE_EXTERNAL_MEMORY
  VkExternalMemoryBufferCreateInfo vk_external_mem_buffer_create_info;
  vk_external_mem_buffer_create_info.sType = VK_STRUCTURE_TYPE_EXTERNAL_MEMORY_BUFFER_CREATE_INFO;
  vk_external_mem_buffer_create_info.pNext = NULL;
#  ifdef _WIN64
  vk_external_mem_buffer_create_info.handleTypes = VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_WIN32_BIT;
#  else
  vkExternalMemBufferCreateInfo.handleTypes = VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_FD_BIT_KHR;
#  endif

  buffer_create_info.pNext = &vk_external_mem_buffer_create_info;
#endif
  if (vmaCreateBuffer(Platform::GetVmaAllocator(), &buffer_create_info, &vma_allocation_create_info, &vk_buffer_,
                      &vma_allocation_, &vma_allocation_info_)) {
    throw std::runtime_error("Failed to create buffer!");
  }
  assert(buffer_create_info.usage != 0);
  flags_ = buffer_create_info.flags;
  size_ = buffer_create_info.size;
  usage_ = buffer_create_info.usage;
  sharing_mode_ = buffer_create_info.sharingMode;
  ApplyVector(queue_family_indices_, buffer_create_info.queueFamilyIndexCount, buffer_create_info.pQueueFamilyIndices);
  vma_allocation_create_info_ = vma_allocation_create_info;
}

Buffer::Buffer(const size_t staging_buffer_size, bool random_access) {
  VkBufferCreateInfo staging_buffer_create_info{};
  staging_buffer_create_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  staging_buffer_create_info.size = staging_buffer_size;
  staging_buffer_create_info.usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
  staging_buffer_create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  VmaAllocationCreateInfo staging_buffer_vma_allocation_create_info{};
  staging_buffer_vma_allocation_create_info.usage = VMA_MEMORY_USAGE_AUTO;
  staging_buffer_vma_allocation_create_info.flags = random_access
                                                        ? VMA_ALLOCATION_CREATE_HOST_ACCESS_RANDOM_BIT
                                                        : VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT;
  Allocate(staging_buffer_create_info, staging_buffer_vma_allocation_create_info);
}

Buffer::Buffer(const VkBufferCreateInfo& buffer_create_info) {
  VmaAllocationCreateInfo alloc_info = {};
  alloc_info.usage = VMA_MEMORY_USAGE_AUTO;
  Allocate(buffer_create_info, alloc_info);
}

Buffer::Buffer(const VkBufferCreateInfo& buffer_create_info,
               const VmaAllocationCreateInfo& vma_allocation_create_info) {
  Allocate(buffer_create_info, vma_allocation_create_info);
}

void Buffer::Resize(VkDeviceSize new_size) {
  if (new_size == size_)
    return;
  if (vk_buffer_ != VK_NULL_HANDLE || vma_allocation_ != VK_NULL_HANDLE) {
    vmaDestroyBuffer(Platform::GetVmaAllocator(), vk_buffer_, vma_allocation_);
    vk_buffer_ = VK_NULL_HANDLE;
    vma_allocation_ = VK_NULL_HANDLE;
    vma_allocation_info_ = {};
  }
  VkBufferCreateInfo buffer_create_info{};
  buffer_create_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  buffer_create_info.flags = flags_;
  buffer_create_info.size = new_size;
  buffer_create_info.usage = usage_;
  buffer_create_info.sharingMode = sharing_mode_;
  buffer_create_info.queueFamilyIndexCount = queue_family_indices_.size();
  buffer_create_info.pQueueFamilyIndices = queue_family_indices_.data();
#if ENABLE_EXTERNAL_MEMORY
  VkExternalMemoryBufferCreateInfo vk_external_mem_buffer_create_info = {};
  vk_external_mem_buffer_create_info.sType = VK_STRUCTURE_TYPE_EXTERNAL_MEMORY_BUFFER_CREATE_INFO;
  vk_external_mem_buffer_create_info.pNext = nullptr;
#  ifdef _WIN64
  vk_external_mem_buffer_create_info.handleTypes = VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_WIN32_BIT;
#  else
  vkExternalMemBufferCreateInfo.handleTypes = VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_FD_BIT_KHR;
#  endif

  buffer_create_info.pNext = &vk_external_mem_buffer_create_info;
#endif
  if (vmaCreateBuffer(Platform::GetVmaAllocator(), &buffer_create_info, &vma_allocation_create_info_, &vk_buffer_,
                      &vma_allocation_, &vma_allocation_info_)) {
    throw std::runtime_error("Failed to create buffer!");
  }
  size_ = new_size;
}

Buffer::~Buffer() {
  if (vk_buffer_ != VK_NULL_HANDLE || vma_allocation_ != VK_NULL_HANDLE) {
    vmaDestroyBuffer(Platform::GetVmaAllocator(), vk_buffer_, vma_allocation_);
    vk_buffer_ = VK_NULL_HANDLE;
    vma_allocation_ = VK_NULL_HANDLE;
    vma_allocation_info_ = {};
  }
}

void Buffer::CopyFromBuffer(const Buffer& src_buffer, const VkDeviceSize size, const VkDeviceSize src_offset,
                            const VkDeviceSize dst_offset) {
  Resize(size);
  Platform::ImmediateSubmit([&](const VkCommandBuffer vk_command_buffer) {
    VkBufferCopy copy_region{};
    copy_region.size = size;
    copy_region.srcOffset = src_offset;
    copy_region.dstOffset = dst_offset;
    vkCmdCopyBuffer(vk_command_buffer, src_buffer.GetVkBuffer(), vk_buffer_, 1, &copy_region);
  });
}

void Buffer::CopyFromImage(Image& src_image, const VkBufferImageCopy& image_copy_info) const {
  Platform::ImmediateSubmit([&](const VkCommandBuffer vk_command_buffer) {
    const auto prev_layout = src_image.GetLayout();
    src_image.TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);
    vkCmdCopyImageToBuffer(vk_command_buffer, src_image.GetVkImage(), src_image.GetLayout(), vk_buffer_, 1,
                           &image_copy_info);
    src_image.TransitImageLayout(vk_command_buffer, prev_layout);
  });
}

void Buffer::CopyFromImage(Image& src_image) {
  Resize(src_image.GetExtent().width * src_image.GetExtent().height * sizeof(glm::vec4));
  VkBufferImageCopy image_copy_info{};
  image_copy_info.bufferOffset = 0;
  image_copy_info.bufferRowLength = 0;
  image_copy_info.bufferImageHeight = 0;
  image_copy_info.imageSubresource.layerCount = 1;
  image_copy_info.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
  image_copy_info.imageSubresource.baseArrayLayer = 0;
  image_copy_info.imageSubresource.mipLevel = 0;

  image_copy_info.imageExtent = src_image.GetExtent();
  image_copy_info.imageOffset.x = 0;
  image_copy_info.imageOffset.y = 0;
  image_copy_info.imageOffset.z = 0;
  CopyFromImage(src_image, image_copy_info);
}

const VkBuffer& Buffer::GetVkBuffer() const {
  return vk_buffer_;
}

VmaAllocation Buffer::GetVmaAllocation() const {
  return vma_allocation_;
}

uint32_t Buffer::GetDeviceAddress() const {
  VkBufferDeviceAddressInfo buffer_device_address_info{};
  buffer_device_address_info.sType = VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO;
  buffer_device_address_info.buffer = vk_buffer_;
  return vkGetBufferDeviceAddress(Platform::GetVkDevice(), &buffer_device_address_info);
}

const VmaAllocationInfo& Buffer::GetVmaAllocationInfo() const {
  return vma_allocation_info_;
}

DescriptorSetLayout::~DescriptorSetLayout() {
  if (vk_descriptor_set_layout_ != VK_NULL_HANDLE) {
    vkDestroyDescriptorSetLayout(Platform::GetVkDevice(), vk_descriptor_set_layout_, nullptr);
    vk_descriptor_set_layout_ = VK_NULL_HANDLE;
  }
}

void DescriptorSetLayout::PushDescriptorBinding(uint32_t binding_index, VkDescriptorType type,
                                                VkShaderStageFlags stage_flags, VkDescriptorBindingFlags binding_flags,
                                                const uint32_t descriptor_count) {
  DescriptorBinding binding;
  VkDescriptorSetLayoutBinding binding_info{};
  binding_info.binding = binding_index;
  binding_info.descriptorCount = descriptor_count;
  binding_info.descriptorType = type;
  binding_info.pImmutableSamplers = nullptr;
  binding_info.stageFlags = stage_flags;
  binding.binding = binding_info;
  binding.binding_flags = binding_flags;
  descriptor_set_layout_bindings_[binding_index] = binding;
}

void DescriptorSetLayout::Initialize() {
  if (vk_descriptor_set_layout_ != VK_NULL_HANDLE) {
    vkDestroyDescriptorSetLayout(Platform::GetVkDevice(), vk_descriptor_set_layout_, nullptr);
    vk_descriptor_set_layout_ = VK_NULL_HANDLE;
  }

  std::vector<VkDescriptorSetLayoutBinding> list_of_bindings;
  std::vector<VkDescriptorBindingFlags> list_of_binding_flags;
  for (const auto& binding : descriptor_set_layout_bindings_) {
    list_of_bindings.emplace_back(binding.second.binding);
    list_of_binding_flags.emplace_back(binding.second.binding_flags);
  }

  VkDescriptorSetLayoutBindingFlagsCreateInfoEXT extended_info{
      VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_BINDING_FLAGS_CREATE_INFO_EXT, nullptr};
  extended_info.bindingCount = static_cast<uint32_t>(list_of_binding_flags.size());
  extended_info.pBindingFlags = list_of_binding_flags.data();

  VkDescriptorSetLayoutCreateInfo descriptor_set_layout_create_info{};
  descriptor_set_layout_create_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
  descriptor_set_layout_create_info.bindingCount = static_cast<uint32_t>(list_of_bindings.size());
  descriptor_set_layout_create_info.pBindings = list_of_bindings.data();
  descriptor_set_layout_create_info.pNext = &extended_info;
  Platform::CheckVk(vkCreateDescriptorSetLayout(Platform::GetVkDevice(), &descriptor_set_layout_create_info, nullptr,
                                                &vk_descriptor_set_layout_));
}

const VkDescriptorSet& DescriptorSet::GetVkDescriptorSet() const {
  return descriptor_set_;
}

DescriptorSet::~DescriptorSet() {
  if (descriptor_set_ != VK_NULL_HANDLE) {
    Platform::CheckVk(vkFreeDescriptorSets(Platform::GetVkDevice(),
                                           Platform::GetDescriptorPool()->GetVkDescriptorPool(), 1, &descriptor_set_));
    descriptor_set_ = VK_NULL_HANDLE;
  }
}

DescriptorSet::DescriptorSet(const std::shared_ptr<DescriptorSetLayout>& target_layout) {
  VkDescriptorSetAllocateInfo alloc_info{};
  alloc_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
  alloc_info.descriptorPool = Platform::GetDescriptorPool()->GetVkDescriptorPool();
  alloc_info.descriptorSetCount = 1;
  alloc_info.pSetLayouts = &target_layout->GetVkDescriptorSetLayout();

  if (vkAllocateDescriptorSets(Platform::GetVkDevice(), &alloc_info, &descriptor_set_) != VK_SUCCESS) {
    throw std::runtime_error("failed to allocate descriptor sets!");
  }
  descriptor_set_layout_ = target_layout;
}

void DescriptorSet::UpdateImageDescriptorBinding(const uint32_t binding_index, const VkDescriptorImageInfo& image_info,
                                                 uint32_t array_element) const {
  const auto& descriptor_binding = descriptor_set_layout_->descriptor_set_layout_bindings_[binding_index];
  VkWriteDescriptorSet write_info{};
  write_info.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
  write_info.dstSet = descriptor_set_;
  write_info.dstBinding = binding_index;
  write_info.dstArrayElement = array_element;
  write_info.descriptorType = descriptor_binding.binding.descriptorType;
  write_info.descriptorCount = 1;
  write_info.pImageInfo = &image_info;
  vkUpdateDescriptorSets(Platform::GetVkDevice(), 1, &write_info, 0, nullptr);
}

void DescriptorSet::UpdateBufferDescriptorBinding(const uint32_t binding_index,
                                                  const VkDescriptorBufferInfo& buffer_info,
                                                  uint32_t array_element) const {
  const auto& descriptor_binding = descriptor_set_layout_->descriptor_set_layout_bindings_[binding_index];
  VkWriteDescriptorSet write_info{};
  write_info.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
  write_info.dstSet = descriptor_set_;
  write_info.dstBinding = binding_index;
  write_info.dstArrayElement = array_element;
  write_info.descriptorType = descriptor_binding.binding.descriptorType;
  write_info.descriptorCount = 1;
  write_info.pBufferInfo = &buffer_info;
  vkUpdateDescriptorSets(Platform::GetVkDevice(), 1, &write_info, 0, nullptr);
}

void DescriptorSet::UpdateBufferDescriptorBinding(const uint32_t binding_index, const std::shared_ptr<Buffer>& buffer,
                                                  const uint32_t array_element) const {
  VkDescriptorBufferInfo buffer_info;
  buffer_info.offset = 0;
  buffer_info.range = VK_WHOLE_SIZE;
  buffer_info.buffer = buffer->GetVkBuffer();
  UpdateBufferDescriptorBinding(binding_index, buffer_info, array_element);
}

const VkDescriptorSetLayout& DescriptorSetLayout::GetVkDescriptorSetLayout() const {
  return vk_descriptor_set_layout_;
}

DescriptorPool::DescriptorPool(const VkDescriptorPoolCreateInfo& descriptor_pool_create_info) {
  Platform::CheckVk(
      vkCreateDescriptorPool(Platform::GetVkDevice(), &descriptor_pool_create_info, nullptr, &vk_descriptor_pool_));
}

DescriptorPool::~DescriptorPool() {
  if (vk_descriptor_pool_ != VK_NULL_HANDLE) {
    vkDestroyDescriptorPool(Platform::GetVkDevice(), vk_descriptor_pool_, nullptr);
    vk_descriptor_pool_ = VK_NULL_HANDLE;
  }
}

VkDescriptorPool DescriptorPool::GetVkDescriptorPool() const {
  return vk_descriptor_pool_;
}

ShaderExt::ShaderExt(const VkShaderCreateInfoEXT& shader_create_info_ext) {
  Platform::CheckVk(vkCreateShadersEXT(Platform::GetVkDevice(), 1, &shader_create_info_ext, nullptr, &shader_ext_));
  flags_ = shader_create_info_ext.flags;
  stage_ = shader_create_info_ext.stage;
  next_stage_ = shader_create_info_ext.nextStage;
  code_type_ = shader_create_info_ext.codeType;
  name_ = shader_create_info_ext.pName;
  ApplyVector(set_layouts_, shader_create_info_ext.setLayoutCount, shader_create_info_ext.pSetLayouts);
  ApplyVector(push_constant_ranges_, shader_create_info_ext.pushConstantRangeCount,
              shader_create_info_ext.pPushConstantRanges);
  if (shader_create_info_ext.pSpecializationInfo)
    specialization_info_ = *shader_create_info_ext.pSpecializationInfo;
}

ShaderExt::~ShaderExt() {
  if (shader_ext_ != VK_NULL_HANDLE) {
    vkDestroyShaderEXT(Platform::GetVkDevice(), shader_ext_, nullptr);
    shader_ext_ = VK_NULL_HANDLE;
  }
}

const VkShaderEXT& ShaderExt::GetVkShaderExt() const {
  return shader_ext_;
}

CommandBufferStatus CommandBuffer::GetStatus() const {
  return status_;
}

CommandBuffer::CommandBuffer(const VkCommandBufferLevel& buffer_level) {
  VkCommandBufferAllocateInfo command_buffer_allocate_info = {};
  command_buffer_allocate_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
  command_buffer_allocate_info.commandPool = Platform::GetVkCommandPool();
  command_buffer_allocate_info.level = buffer_level;
  command_buffer_allocate_info.commandBufferCount = 1;
  Platform::CheckVk(
      vkAllocateCommandBuffers(Platform::GetVkDevice(), &command_buffer_allocate_info, &vk_command_buffer_));
  status_ = CommandBufferStatus::Ready;
}

CommandBuffer::~CommandBuffer() {
  if (vk_command_buffer_ != VK_NULL_HANDLE) {
    vkFreeCommandBuffers(Platform::GetVkDevice(), Platform::GetVkCommandPool(), 1, &vk_command_buffer_);
    vk_command_buffer_ = VK_NULL_HANDLE;
  }
  status_ = CommandBufferStatus::Invalid;
}

const VkCommandBuffer& CommandBuffer::GetVkCommandBuffer() const {
  return vk_command_buffer_;
}

void CommandBuffer::Begin(const VkCommandBufferUsageFlags& usage) {
  if (status_ == CommandBufferStatus::Invalid) {
    EVOENGINE_ERROR("Command buffer invalid!")
    return;
  }
  if (status_ != CommandBufferStatus::Ready) {
    EVOENGINE_ERROR("Command buffer not ready!")
    return;
  }
  VkCommandBufferBeginInfo begin_info = {};
  begin_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
  begin_info.flags = usage;
  Platform::CheckVk(vkBeginCommandBuffer(vk_command_buffer_, &begin_info));
  status_ = CommandBufferStatus::Recording;
}

void CommandBuffer::End() {
  if (status_ == CommandBufferStatus::Invalid) {
    EVOENGINE_ERROR("Command buffer invalid!")
    return;
  }
  if (status_ != CommandBufferStatus::Recording) {
    EVOENGINE_ERROR("Command buffer not recording!")
    return;
  }
  Platform::CheckVk(vkEndCommandBuffer(vk_command_buffer_));
  status_ = CommandBufferStatus::Recorded;
}

void CommandBuffer::Record(const std::function<void(VkCommandBuffer vk_command_buffer)>& commands) {
  Begin();
  commands(vk_command_buffer_);
  End();
}

void CommandBuffer::Reset() {
  if (status_ == CommandBufferStatus::Invalid) {
    EVOENGINE_ERROR("Command buffer invalid!");
    return;
  }
  Platform::CheckVk(vkResetCommandBuffer(vk_command_buffer_, 0));
  status_ = CommandBufferStatus::Ready;
}

void CommandQueue::Submit(
    const std::vector<std::shared_ptr<CommandBuffer>>& command_buffers,
    const std::vector<std::pair<std::shared_ptr<Semaphore>, VkPipelineStageFlags>>& wait_semaphores,
    const std::vector<std::shared_ptr<Semaphore>>& signal_semaphores, const std::shared_ptr<Fence>& fence) const {
  VkSubmitInfo submit_info{};
  submit_info.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
  //===========
  submit_info.waitSemaphoreCount = wait_semaphores.size();
  std::vector<VkSemaphore> wait_vk_semaphores(wait_semaphores.size());
  std::vector<VkPipelineStageFlags> wait_vk_wait_stage_flags(wait_semaphores.size());
  for (uint32_t i = 0; i < wait_vk_semaphores.size(); i++) {
    wait_vk_semaphores[i] = wait_semaphores[i].first->GetVkSemaphore();
    wait_vk_wait_stage_flags[i] = wait_semaphores[i].second;
  }
  submit_info.pWaitSemaphores = wait_vk_semaphores.data();
  submit_info.pWaitDstStageMask = wait_vk_wait_stage_flags.data();

  //===========
  submit_info.signalSemaphoreCount = signal_semaphores.size();
  std::vector<VkSemaphore> signal_vk_semaphores(signal_semaphores.size());
  for (uint32_t i = 0; i < signal_vk_semaphores.size(); i++) {
    signal_vk_semaphores[i] = signal_semaphores[i]->GetVkSemaphore();
  }
  submit_info.pSignalSemaphores = signal_vk_semaphores.data();

  //===========
  submit_info.commandBufferCount = command_buffers.size();
  std::vector<VkCommandBuffer> vk_command_buffers(command_buffers.size());
  for (uint32_t i = 0; i < command_buffers.size(); i++) {
    vk_command_buffers[i] = command_buffers[i]->GetVkCommandBuffer();
  }
  submit_info.pCommandBuffers = vk_command_buffers.data();

  if (vkQueueSubmit(vk_queue_, 1, &submit_info, fence->GetVkFence()) != VK_SUCCESS) {
    throw std::runtime_error("Failed to submit command buffer!");
  }
}

void CommandQueue::Submit(
    const std::vector<std::shared_ptr<CommandBuffer>>& command_buffers,
    const std::vector<std::pair<std::shared_ptr<Semaphore>, VkPipelineStageFlags>>& wait_semaphores,
    const std::vector<std::shared_ptr<Semaphore>>& signal_semaphores) const {
  VkSubmitInfo submit_info{};
  submit_info.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
  //===========
  submit_info.waitSemaphoreCount = wait_semaphores.size();
  std::vector<VkSemaphore> wait_vk_semaphores(wait_semaphores.size());
  std::vector<VkPipelineStageFlags> wait_vk_wait_stage_flags(wait_semaphores.size());
  for (uint32_t i = 0; i < wait_vk_semaphores.size(); i++) {
    wait_vk_semaphores[i] = wait_semaphores[i].first->GetVkSemaphore();
    wait_vk_wait_stage_flags[i] = wait_semaphores[i].second;
  }
  submit_info.pWaitSemaphores = wait_vk_semaphores.data();
  submit_info.pWaitDstStageMask = wait_vk_wait_stage_flags.data();

  //===========
  submit_info.signalSemaphoreCount = signal_semaphores.size();
  std::vector<VkSemaphore> signal_vk_semaphores(signal_semaphores.size());
  for (uint32_t i = 0; i < signal_vk_semaphores.size(); i++) {
    signal_vk_semaphores[i] = signal_semaphores[i]->GetVkSemaphore();
  }
  submit_info.pSignalSemaphores = signal_vk_semaphores.data();

  //===========
  submit_info.commandBufferCount = command_buffers.size();
  std::vector<VkCommandBuffer> vk_command_buffers(command_buffers.size());
  for (uint32_t i = 0; i < command_buffers.size(); i++) {
    vk_command_buffers[i] = command_buffers[i]->GetVkCommandBuffer();
  }
  submit_info.pCommandBuffers = vk_command_buffers.data();

  if (vkQueueSubmit(vk_queue_, 1, &submit_info, VK_NULL_HANDLE) != VK_SUCCESS) {
    throw std::runtime_error("Failed to submit command buffer!");
  }
}

void CommandQueue::ImmediateSubmit(
    const std::vector<std::shared_ptr<CommandBuffer>>& command_buffers,
    const std::vector<std::pair<std::shared_ptr<Semaphore>, VkPipelineStageFlags>>& wait_semaphores,
    const std::vector<std::shared_ptr<Semaphore>>& signal_semaphores) const {
  Submit(command_buffers, wait_semaphores, signal_semaphores);
  WaitIdle();
}

void CommandQueue::Present(const std::vector<std::shared_ptr<Semaphore>>& wait_semaphores,
                           const std::vector<std::pair<std::shared_ptr<Swapchain>, uint32_t>>& targets) const {
  VkPresentInfoKHR present_info{};
  present_info.sType = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR;
  std::vector<VkSemaphore> wait_vk_semaphores(wait_semaphores.size());
  for (uint32_t i = 0; i < wait_vk_semaphores.size(); i++) {
    wait_vk_semaphores[i] = wait_semaphores[i]->GetVkSemaphore();
  }
  present_info.waitSemaphoreCount = wait_vk_semaphores.size();
  present_info.pWaitSemaphores = wait_vk_semaphores.data();

  //===========
  present_info.swapchainCount = targets.size();
  std::vector<VkSwapchainKHR> vk_swapchain_khrs(targets.size());
  std::vector<uint32_t> image_indices(targets.size());
  for (uint32_t i = 0; i < targets.size(); i++) {
    vk_swapchain_khrs[i] = targets[i].first->GetVkSwapchain();
    image_indices[i] = targets[i].second;
  }
  present_info.pSwapchains = vk_swapchain_khrs.data();
  present_info.pImageIndices = image_indices.data();

  vkQueuePresentKHR(vk_queue_, &present_info);
}

void CommandQueue::WaitIdle() const {
  vkQueueWaitIdle(vk_queue_);
}

BLAS::BLAS(const std::vector<Vertex>& vertices, const std::vector<glm::uvec3>& triangles) {
  VkBufferCreateInfo buffer_create_info{};
  buffer_create_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  buffer_create_info.usage = VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR |
                             VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
  buffer_create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  buffer_create_info.size = 1;
  VmaAllocationCreateInfo buffer_vma_allocation_create_info{};
  buffer_vma_allocation_create_info.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
  const auto vertex_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  vertex_buffer->Upload(vertices);
  const auto index_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  index_buffer->Upload(triangles);
  const auto transform_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  VkTransformMatrixKHR transform_matrix = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f};
  transform_buffer->Upload(transform_matrix);

  VkDeviceOrHostAddressConstKHR vertex_data_device_address{};
  vertex_data_device_address.deviceAddress = vertex_buffer->GetDeviceAddress();

  VkDeviceOrHostAddressConstKHR indices_data_device_address{};
  indices_data_device_address.deviceAddress = index_buffer->GetDeviceAddress();

  VkDeviceOrHostAddressConstKHR transform_matrix_device_address{};
  transform_matrix_device_address.deviceAddress = transform_buffer->GetDeviceAddress();

  // The bottom level acceleration structure contains one set of triangles as the input geometry
  VkAccelerationStructureGeometryKHR acceleration_structure_geometry{};
  acceleration_structure_geometry.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR;
  acceleration_structure_geometry.pNext = nullptr;
  acceleration_structure_geometry.geometryType = VK_GEOMETRY_TYPE_TRIANGLES_KHR;
  acceleration_structure_geometry.flags = VK_GEOMETRY_OPAQUE_BIT_KHR;
  acceleration_structure_geometry.geometry.triangles = {};
  acceleration_structure_geometry.geometry.triangles.sType =
      VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_TRIANGLES_DATA_KHR;
  acceleration_structure_geometry.geometry.triangles.vertexFormat = VK_FORMAT_R32G32B32_SFLOAT;
  acceleration_structure_geometry.geometry.triangles.vertexData = vertex_data_device_address;
  acceleration_structure_geometry.geometry.triangles.maxVertex = vertices.size();
  acceleration_structure_geometry.geometry.triangles.pNext = nullptr;
  acceleration_structure_geometry.geometry.triangles.vertexStride = sizeof(Vertex);
  acceleration_structure_geometry.geometry.triangles.indexType = VK_INDEX_TYPE_UINT32;
  acceleration_structure_geometry.geometry.triangles.indexData = indices_data_device_address;
  acceleration_structure_geometry.geometry.triangles.transformData = transform_matrix_device_address;

  // Get the size requirements for buffers involved in the acceleration structure build process
  VkAccelerationStructureBuildGeometryInfoKHR acceleration_structure_build_geometry_info{};
  acceleration_structure_build_geometry_info.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR;
  acceleration_structure_build_geometry_info.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
  acceleration_structure_build_geometry_info.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR;
  acceleration_structure_build_geometry_info.geometryCount = 1;
  acceleration_structure_build_geometry_info.pGeometries = &acceleration_structure_geometry;

  VkAccelerationStructureBuildRangeInfoKHR acceleration_structure_build_range_info{};
  acceleration_structure_build_range_info.primitiveCount = triangles.size();
  acceleration_structure_build_range_info.primitiveOffset = 0;
  acceleration_structure_build_range_info.firstVertex = 0;
  acceleration_structure_build_range_info.transformOffset = 0;

  std::vector acceleration_build_structure_range_infos = {&acceleration_structure_build_range_info};
  const std::vector primitive_counts = {static_cast<uint32_t>(triangles.size())};

  VkAccelerationStructureBuildSizesInfoKHR acceleration_structure_build_sizes_info{};
  acceleration_structure_build_sizes_info.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR;
  vkGetAccelerationStructureBuildSizesKHR(Platform::GetVkDevice(), VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR,
                                          &acceleration_structure_build_geometry_info, primitive_counts.data(),
                                          &acceleration_structure_build_sizes_info);

  buffer_create_info.size = acceleration_structure_build_sizes_info.accelerationStructureSize;
  buffer_create_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  buffer_create_info.usage =
      VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_STORAGE_BIT_KHR | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT;

  acceleration_structure_buffer_ = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);

  VkAccelerationStructureCreateInfoKHR acceleration_structure_create_info{};
  acceleration_structure_create_info.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_INFO_KHR;
  acceleration_structure_create_info.buffer = acceleration_structure_buffer_->GetVkBuffer();
  acceleration_structure_create_info.size = acceleration_structure_build_sizes_info.accelerationStructureSize;
  acceleration_structure_create_info.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
  acceleration_structure_create_info.pNext = nullptr;
  vkCreateAccelerationStructureKHR(Platform::GetVkDevice(), &acceleration_structure_create_info, nullptr,
                                   &vk_acceleration_structure_khr_);

  // The actual build process starts here

  // Create a scratch buffer as a temporary storage for the acceleration structure build
  buffer_create_info.size = acceleration_structure_build_sizes_info.buildScratchSize;
  buffer_create_info.usage =VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
      VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT;

  const auto scratch_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  VkDeviceOrHostAddressConstKHR scratch_buffer_device_address{};
  scratch_buffer_device_address.deviceAddress = scratch_buffer->GetDeviceAddress();

  VkAccelerationStructureBuildGeometryInfoKHR acceleration_build_geometry_info{};
  acceleration_build_geometry_info.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR;
  acceleration_build_geometry_info.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
  acceleration_build_geometry_info.flags = acceleration_structure_build_geometry_info.flags;
  acceleration_build_geometry_info.mode = VK_BUILD_ACCELERATION_STRUCTURE_MODE_BUILD_KHR;
  acceleration_build_geometry_info.dstAccelerationStructure = vk_acceleration_structure_khr_;
  acceleration_build_geometry_info.geometryCount = 1;
  acceleration_build_geometry_info.pGeometries = &acceleration_structure_geometry;
  acceleration_build_geometry_info.scratchData.deviceAddress = scratch_buffer_device_address.deviceAddress;

  
  // Build the acceleration structure on the device via a one-time command buffer submission
  // Some implementations may support acceleration structure building on the host
  // (VkPhysicalDeviceAccelerationStructureFeaturesKHR->accelerationStructureHostCommands), but we prefer device builds
  Platform::ImmediateSubmit([&](const VkCommandBuffer vk_command_buffer) {
    vkCmdBuildAccelerationStructuresKHR(vk_command_buffer, 1, &acceleration_build_geometry_info,
                                        acceleration_build_structure_range_infos.data());
  });

  /*
  Platform::CheckVk(vkBuildAccelerationStructuresKHR(Platform::GetVkDevice(), nullptr, 1,
                                                    &acceleration_build_geometry_info,
                                   acceleration_build_structure_range_infos.data()));*/
}

uint64_t BLAS::GetDeviceAddress() const {
  // Get the bottom acceleration structure's handle, which will be used during the top level acceleration build
  VkAccelerationStructureDeviceAddressInfoKHR acceleration_device_address_info{};
  acceleration_device_address_info.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_DEVICE_ADDRESS_INFO_KHR;
  acceleration_device_address_info.accelerationStructure = vk_acceleration_structure_khr_;
  return vkGetAccelerationStructureDeviceAddressKHR(Platform::GetVkDevice(), &acceleration_device_address_info);
}
