#include "RenderPasses/RenderPassUtilities.hpp"

#include "GraphicsResources.hpp"
#include "Platform.hpp"

using namespace evo_engine;

namespace {
VkImageLayout ToVkImageLayout(const RenderResourceState state) {
  switch (state) {
    case RenderResourceState::ColorAttachment:
    case RenderResourceState::DepthAttachment:
      return VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL;
    case RenderResourceState::ShaderRead:
      return VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    case RenderResourceState::StorageReadWrite:
    case RenderResourceState::AccelerationStructureRead:
    case RenderResourceState::General:
      return VK_IMAGE_LAYOUT_GENERAL;
    case RenderResourceState::Present:
      return VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
    case RenderResourceState::TransferSource:
      return VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
    case RenderResourceState::TransferDestination:
      return VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
    case RenderResourceState::Undefined:
      return VK_IMAGE_LAYOUT_UNDEFINED;
  }
  return VK_IMAGE_LAYOUT_UNDEFINED;
}

uint32_t GetRenderPassQueueFamilyIndex(const RenderPassQueue queue) {
  switch (queue) {
    case RenderPassQueue::Compute:
      return Platform::GetComputeQueueFamilyIndex();
    case RenderPassQueue::Graphics:
    case RenderPassQueue::RayTracing:
      return Platform::GetGraphicsAndComputeQueueFamilyIndex();
  }
  return Platform::GetGraphicsAndComputeQueueFamilyIndex();
}

bool TryGetQueueFamilyOwnershipTransfer(const RenderResourceBarrierPlan& barrier,
                                        const RenderResourceDescriptor& descriptor,
                                        const RenderPassQueue recorded_queue, const bool release_barrier,
                                        uint32_t& src_queue_family_index, uint32_t& dst_queue_family_index) {
  if (!barrier.queue_change || descriptor.managed_by_graph || !Platform::Initialized()) {
    return false;
  }
  const auto expected_queue = release_barrier ? barrier.previous_queue : barrier.next_queue;
  if (recorded_queue != expected_queue) {
    return false;
  }
  try {
    src_queue_family_index = GetRenderPassQueueFamilyIndex(barrier.previous_queue);
    dst_queue_family_index = GetRenderPassQueueFamilyIndex(barrier.next_queue);
  } catch (...) {
    return false;
  }
  return src_queue_family_index != dst_queue_family_index;
}

void ApplyGraphImageBarrier(const VkCommandBuffer vk_command_buffer, const std::shared_ptr<Image>& image,
                            const VkImageLayout target_layout, const bool force_memory_barrier) {
  if (!image || target_layout == VK_IMAGE_LAYOUT_UNDEFINED ||
      (!force_memory_barrier && image->GetLayout() == target_layout)) {
    return;
  }
  image->TransitImageLayout(vk_command_buffer, target_layout);
}

bool ApplyGraphImageQueueOwnershipBarrier(const VkCommandBuffer vk_command_buffer,
                                          const RenderResourceBarrierPlan& barrier,
                                          const RenderResourceDescriptor& descriptor,
                                          const RenderGraphResourceBinding* binding,
                                          const RenderPassQueue recorded_queue, const bool release_barrier) {
  uint32_t src_queue_family_index = VK_QUEUE_FAMILY_IGNORED;
  uint32_t dst_queue_family_index = VK_QUEUE_FAMILY_IGNORED;
  if (!TryGetQueueFamilyOwnershipTransfer(barrier, descriptor, recorded_queue, release_barrier, src_queue_family_index,
                                          dst_queue_family_index)) {
    return false;
  }
  if (!binding || (!binding->image && binding->images.empty())) {
    return false;
  }
  const auto previous_layout = ToVkImageLayout(barrier.previous_state);
  const auto next_layout = ToVkImageLayout(barrier.next_state);
  if (previous_layout == VK_IMAGE_LAYOUT_UNDEFINED || next_layout == VK_IMAGE_LAYOUT_UNDEFINED) {
    return false;
  }
  const auto apply_transfer = [&](const std::shared_ptr<Image>& image) {
    if (image) {
      image->TransitImageLayout(vk_command_buffer, previous_layout, next_layout, src_queue_family_index,
                                dst_queue_family_index, !release_barrier);
    }
  };
  if (!binding->images.empty()) {
    for (const auto& image : binding->images) {
      apply_transfer(image);
    }
    return true;
  }
  apply_transfer(binding->image);
  return true;
}

bool ApplyGraphBufferQueueOwnershipBarrier(const VkCommandBuffer vk_command_buffer,
                                           const RenderResourceBarrierPlan& barrier,
                                           const RenderResourceDescriptor& descriptor,
                                           const RenderGraphResourceBinding* binding,
                                           const RenderPassQueue recorded_queue, const bool release_barrier) {
  uint32_t src_queue_family_index = VK_QUEUE_FAMILY_IGNORED;
  uint32_t dst_queue_family_index = VK_QUEUE_FAMILY_IGNORED;
  if (!TryGetQueueFamilyOwnershipTransfer(barrier, descriptor, recorded_queue, release_barrier, src_queue_family_index,
                                          dst_queue_family_index)) {
    return false;
  }
  if (!binding || !binding->buffer) {
    return false;
  }
  Platform::BufferMemoryBarrier(vk_command_buffer, *binding->buffer, src_queue_family_index, dst_queue_family_index,
                                release_barrier);
  return true;
}

void ApplyGraphResourceBarrier(const VkCommandBuffer vk_command_buffer, const RenderGraphExecutionContext& context,
                               const RenderResourceBarrierPlan& barrier, const RenderPassQueue recorded_queue,
                               const bool release_barrier) {
  const auto* descriptor = context.GetResourceDescriptor(barrier.resource_index);
  if (!descriptor) {
    return;
  }
  const auto* binding = context.GetResourceBinding(descriptor->name);
  switch (barrier.barrier_type) {
    case RenderGraphBarrierType::ImageLayout:
    case RenderGraphBarrierType::ImageMemory: {
      if (ApplyGraphImageQueueOwnershipBarrier(vk_command_buffer, barrier, *descriptor, binding, recorded_queue,
                                               release_barrier) ||
          release_barrier) {
        return;
      }
      if (!binding || (!binding->image && binding->images.empty())) {
        return;
      }
      const auto target_layout = ToVkImageLayout(barrier.next_state);
      const bool force_memory_barrier =
          barrier.barrier_type == RenderGraphBarrierType::ImageMemory || barrier.memory_dependency;
      if (!binding->images.empty()) {
        for (const auto& image : binding->images) {
          ApplyGraphImageBarrier(vk_command_buffer, image, target_layout, force_memory_barrier);
        }
        return;
      }
      ApplyGraphImageBarrier(vk_command_buffer, binding->image, target_layout, force_memory_barrier);
    } break;
    case RenderGraphBarrierType::BufferMemory: {
      if (ApplyGraphBufferQueueOwnershipBarrier(vk_command_buffer, barrier, *descriptor, binding, recorded_queue,
                                                release_barrier) ||
          release_barrier) {
        return;
      }
      if (binding && binding->buffer) {
        Platform::BufferMemoryBarrier(vk_command_buffer, *binding->buffer);
      } else {
        Platform::EverythingBarrier(vk_command_buffer);
      }
    } break;
    case RenderGraphBarrierType::GlobalMemory: {
      if (!release_barrier) {
        Platform::EverythingBarrier(vk_command_buffer);
      }
    } break;
  }
}
}  // namespace

uint32_t evo_engine::CalculateMipDimension(const uint32_t base_dimension, const uint32_t mip_level) {
  return glm::max(1u, base_dimension >> mip_level);
}

std::shared_ptr<ImageView> evo_engine::CreateGraphImageMipView(const std::shared_ptr<Image>& image,
                                                               const uint32_t mip_level) {
  if (!image) {
    return {};
  }
  VkImageViewCreateInfo view_info{};
  view_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
  view_info.image = image->GetVkImage();
  view_info.viewType = VK_IMAGE_VIEW_TYPE_2D;
  view_info.format = image->GetFormat();
  view_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
  view_info.subresourceRange.baseMipLevel = mip_level;
  view_info.subresourceRange.levelCount = 1;
  view_info.subresourceRange.baseArrayLayer = 0;
  view_info.subresourceRange.layerCount = 1;
  return std::make_shared<ImageView>(view_info, image);
}

void evo_engine::ApplyGraphResourceBarriers(const VkCommandBuffer vk_command_buffer,
                                            const RenderGraphExecutionContext& context,
                                            const RenderPassQueue recorded_queue) {
  for (const auto* barrier : context.GetCurrentPassBarriers()) {
    if (!barrier) {
      continue;
    }
    ApplyGraphResourceBarrier(vk_command_buffer, context, *barrier, recorded_queue, false);
  }
}

void evo_engine::ApplyGraphResourceReleaseBarriers(const VkCommandBuffer vk_command_buffer,
                                                   const RenderGraphExecutionContext& context,
                                                   const RenderPassQueue recorded_queue) {
  for (const auto* barrier : context.GetCurrentPassReleaseBarriers()) {
    if (!barrier) {
      continue;
    }
    ApplyGraphResourceBarrier(vk_command_buffer, context, *barrier, recorded_queue, true);
  }
}

void evo_engine::ClearGraphColorImage(const VkCommandBuffer vk_command_buffer, const std::shared_ptr<Image>& image,
                                      const VkClearColorValue& clear_value) {
  if (!image || image->GetMipLevels() == 0) {
    return;
  }
  VkImageSubresourceRange subresource_range{};
  subresource_range.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
  subresource_range.baseMipLevel = 0;
  subresource_range.levelCount = image->GetMipLevels();
  subresource_range.baseArrayLayer = 0;
  subresource_range.layerCount = 1;
  Platform::ClearColorImage(vk_command_buffer, *image, clear_value, 1, &subresource_range);
}
