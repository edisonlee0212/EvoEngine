#include "RenderPasses/DdgiPassUtilities.hpp"

#include "GraphicsResources.hpp"
#include "Resources.hpp"
#include "Texture2D.hpp"

#include <array>

using namespace evo_engine;

VkDescriptorImageInfo evo_engine::CreateDdgiFallbackImageInfo() {
  VkDescriptorImageInfo image_info{};
  const auto missing_texture = Resources::GetInstance().GetMissingTexture();
  if (missing_texture) {
    image_info.imageLayout = missing_texture->GetLayout();
    image_info.imageView = missing_texture->GetVkImageView();
    image_info.sampler = missing_texture->GetVkSampler();
  }
  return image_info;
}

bool evo_engine::IsValidDescriptorImageInfo(const VkDescriptorImageInfo& image_info) {
  return image_info.imageView != VK_NULL_HANDLE && image_info.sampler != VK_NULL_HANDLE;
}

void evo_engine::ApplyDdgiBufferDependency(const VkCommandBuffer command_buffer, const std::shared_ptr<Buffer>& buffer,
                                           const VkPipelineStageFlags2 source_stages,
                                           const VkAccessFlags2 source_access,
                                           const VkPipelineStageFlags2 destination_stages,
                                           const VkAccessFlags2 destination_access, const VkDeviceSize offset,
                                           const VkDeviceSize size) {
  if (!buffer) {
    return;
  }
  VkBufferMemoryBarrier2 barrier{VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER_2};
  barrier.srcStageMask = source_stages;
  barrier.srcAccessMask = source_access;
  barrier.dstStageMask = destination_stages;
  barrier.dstAccessMask = destination_access;
  barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
  barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
  barrier.buffer = buffer->GetVkBuffer();
  barrier.offset = offset;
  barrier.size = size;
  VkDependencyInfo dependency{VK_STRUCTURE_TYPE_DEPENDENCY_INFO};
  dependency.bufferMemoryBarrierCount = 1;
  dependency.pBufferMemoryBarriers = &barrier;
  vkCmdPipelineBarrier2(command_buffer, &dependency);
}

void evo_engine::ApplyDdgiImageDependency(const VkCommandBuffer command_buffer, const std::shared_ptr<Image>& image,
                                          const VkPipelineStageFlags2 source_stages, const VkAccessFlags2 source_access,
                                          const VkPipelineStageFlags2 destination_stages,
                                          const VkAccessFlags2 destination_access) {
  if (!image) {
    return;
  }
  VkImageMemoryBarrier2 barrier{VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2};
  barrier.srcStageMask = source_stages;
  barrier.srcAccessMask = source_access;
  barrier.dstStageMask = destination_stages;
  barrier.dstAccessMask = destination_access;
  barrier.oldLayout = VK_IMAGE_LAYOUT_GENERAL;
  barrier.newLayout = VK_IMAGE_LAYOUT_GENERAL;
  barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
  barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
  barrier.image = image->GetVkImage();
  barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
  barrier.subresourceRange.levelCount = image->GetMipLevels();
  barrier.subresourceRange.layerCount = 1;
  VkDependencyInfo dependency{VK_STRUCTURE_TYPE_DEPENDENCY_INFO};
  dependency.imageMemoryBarrierCount = 1;
  dependency.pImageMemoryBarriers = &barrier;
  vkCmdPipelineBarrier2(command_buffer, &dependency);
}

void evo_engine::AcquireDdgiFrameResources(const VkCommandBuffer command_buffer,
                                           const std::shared_ptr<Image>& irradiance_atlas,
                                           const std::shared_ptr<Image>& visibility_atlas,
                                           const std::shared_ptr<Buffer>& probe_state,
                                           const std::shared_ptr<Buffer>& probe_metadata,
                                           const std::shared_ptr<Buffer>& selected_ray_diagnostics) {
  constexpr VkPipelineStageFlags2 source_stages =
      VK_PIPELINE_STAGE_2_TRANSFER_BIT | VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT |
      VK_PIPELINE_STAGE_2_VERTEX_SHADER_BIT | VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT |
      VK_PIPELINE_STAGE_2_RAY_TRACING_SHADER_BIT_KHR;
  constexpr VkAccessFlags2 source_access = VK_ACCESS_2_TRANSFER_READ_BIT | VK_ACCESS_2_TRANSFER_WRITE_BIT |
                                           VK_ACCESS_2_SHADER_READ_BIT | VK_ACCESS_2_SHADER_WRITE_BIT;
  constexpr VkPipelineStageFlags2 destination_stages = VK_PIPELINE_STAGE_2_TRANSFER_BIT |
                                                       VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT |
                                                       VK_PIPELINE_STAGE_2_RAY_TRACING_SHADER_BIT_KHR;
  constexpr VkAccessFlags2 destination_access = VK_ACCESS_2_TRANSFER_READ_BIT | VK_ACCESS_2_TRANSFER_WRITE_BIT |
                                                VK_ACCESS_2_SHADER_READ_BIT | VK_ACCESS_2_SHADER_WRITE_BIT;

  std::array<VkImageMemoryBarrier2, 3> image_barriers{};
  uint32_t image_barrier_count = 0u;
  const auto append_image = [&](const std::shared_ptr<Image>& image) {
    if (!image || image->GetLayout() != VK_IMAGE_LAYOUT_GENERAL) {
      return;
    }
    auto& barrier = image_barriers[image_barrier_count++];
    barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2;
    barrier.srcStageMask = source_stages;
    barrier.srcAccessMask = source_access;
    barrier.dstStageMask = destination_stages;
    barrier.dstAccessMask = destination_access;
    barrier.oldLayout = VK_IMAGE_LAYOUT_GENERAL;
    barrier.newLayout = VK_IMAGE_LAYOUT_GENERAL;
    barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrier.image = image->GetVkImage();
    barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    barrier.subresourceRange.levelCount = image->GetMipLevels();
    barrier.subresourceRange.layerCount = 1;
  };
  append_image(irradiance_atlas);
  append_image(visibility_atlas);

  std::array<VkBufferMemoryBarrier2, 3> buffer_barriers{};
  uint32_t buffer_barrier_count = 0u;
  const auto append_buffer = [&](const std::shared_ptr<Buffer>& buffer) {
    if (!buffer) {
      return;
    }
    auto& barrier = buffer_barriers[buffer_barrier_count++];
    barrier.sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER_2;
    barrier.srcStageMask = source_stages;
    barrier.srcAccessMask = source_access;
    barrier.dstStageMask = destination_stages;
    barrier.dstAccessMask = destination_access;
    barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrier.buffer = buffer->GetVkBuffer();
    barrier.size = VK_WHOLE_SIZE;
  };
  append_buffer(probe_state);
  append_buffer(probe_metadata);
  append_buffer(selected_ray_diagnostics);

  if (image_barrier_count == 0u && buffer_barrier_count == 0u) {
    return;
  }
  VkDependencyInfo dependency{VK_STRUCTURE_TYPE_DEPENDENCY_INFO};
  dependency.imageMemoryBarrierCount = image_barrier_count;
  dependency.pImageMemoryBarriers = image_barriers.data();
  dependency.bufferMemoryBarrierCount = buffer_barrier_count;
  dependency.pBufferMemoryBarriers = buffer_barriers.data();
  vkCmdPipelineBarrier2(command_buffer, &dependency);
}

void evo_engine::PublishDdgiFrameResources(const VkCommandBuffer command_buffer,
                                           const std::shared_ptr<Image>& irradiance_atlas,
                                           const std::shared_ptr<Image>& visibility_atlas,
                                           const std::shared_ptr<Buffer>& probe_state,
                                           const std::shared_ptr<Buffer>& probe_metadata,
                                           const std::shared_ptr<Buffer>& selected_ray_diagnostics) {
  std::array<VkImageMemoryBarrier2, 2> image_barriers{};
  uint32_t image_barrier_count = 0u;
  const auto append_image = [&](const std::shared_ptr<Image>& image) {
    if (!image) {
      return;
    }
    auto& barrier = image_barriers[image_barrier_count++];
    barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2;
    barrier.srcStageMask = VK_PIPELINE_STAGE_2_TRANSFER_BIT | VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT;
    barrier.srcAccessMask = VK_ACCESS_2_TRANSFER_WRITE_BIT | VK_ACCESS_2_SHADER_WRITE_BIT;
    barrier.dstStageMask = VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT | VK_PIPELINE_STAGE_2_RAY_TRACING_SHADER_BIT_KHR;
    barrier.dstAccessMask = VK_ACCESS_2_SHADER_READ_BIT;
    barrier.oldLayout = VK_IMAGE_LAYOUT_GENERAL;
    barrier.newLayout = VK_IMAGE_LAYOUT_GENERAL;
    barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrier.image = image->GetVkImage();
    barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    barrier.subresourceRange.levelCount = image->GetMipLevels();
    barrier.subresourceRange.layerCount = 1;
  };
  append_image(irradiance_atlas);
  append_image(visibility_atlas);

  std::array<VkBufferMemoryBarrier2, 3> buffer_barriers{};
  uint32_t buffer_barrier_count = 0u;
  const auto append_buffer = [&](const std::shared_ptr<Buffer>& buffer, const VkPipelineStageFlags2 source_stages,
                                 const VkAccessFlags2 source_access, const VkPipelineStageFlags2 destination_stages) {
    if (!buffer) {
      return;
    }
    auto& barrier = buffer_barriers[buffer_barrier_count++];
    barrier.sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER_2;
    barrier.srcStageMask = source_stages;
    barrier.srcAccessMask = source_access;
    barrier.dstStageMask = destination_stages;
    barrier.dstAccessMask = VK_ACCESS_2_SHADER_READ_BIT;
    barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrier.buffer = buffer->GetVkBuffer();
    barrier.size = VK_WHOLE_SIZE;
  };
  append_buffer(probe_state, VK_PIPELINE_STAGE_2_TRANSFER_BIT | VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                VK_ACCESS_2_TRANSFER_WRITE_BIT | VK_ACCESS_2_SHADER_WRITE_BIT,
                VK_PIPELINE_STAGE_2_VERTEX_SHADER_BIT | VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT |
                    VK_PIPELINE_STAGE_2_RAY_TRACING_SHADER_BIT_KHR);
  append_buffer(probe_metadata, VK_PIPELINE_STAGE_2_TRANSFER_BIT | VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                VK_ACCESS_2_TRANSFER_WRITE_BIT | VK_ACCESS_2_SHADER_WRITE_BIT, VK_PIPELINE_STAGE_2_VERTEX_SHADER_BIT);
  append_buffer(selected_ray_diagnostics, VK_PIPELINE_STAGE_2_RAY_TRACING_SHADER_BIT_KHR, VK_ACCESS_2_SHADER_WRITE_BIT,
                VK_PIPELINE_STAGE_2_VERTEX_SHADER_BIT);

  if (image_barrier_count == 0u && buffer_barrier_count == 0u) {
    return;
  }
  VkDependencyInfo dependency{VK_STRUCTURE_TYPE_DEPENDENCY_INFO};
  dependency.imageMemoryBarrierCount = image_barrier_count;
  dependency.pImageMemoryBarriers = image_barriers.data();
  dependency.bufferMemoryBarrierCount = buffer_barrier_count;
  dependency.pBufferMemoryBarriers = buffer_barriers.data();
  vkCmdPipelineBarrier2(command_buffer, &dependency);
}
