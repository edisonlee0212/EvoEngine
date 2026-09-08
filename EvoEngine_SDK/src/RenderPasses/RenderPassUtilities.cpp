#include "RenderPasses/RenderPassUtilities.hpp"

#include "GraphicsPipeline.hpp"
#include "GraphicsResources.hpp"
#include "Platform.hpp"
#include "RenderInstanceStorage.hpp"

using namespace evo_engine;

namespace {
const char* GetProfilerGroupName(const RenderPassProfilerGroup group) {
  switch (group) {
    case RenderPassProfilerGroup::FramePreparation:
      return "Frame Preparation";
    case RenderPassProfilerGroup::Shadows:
      return "Shadows";
    case RenderPassProfilerGroup::CameraVisibility:
      return "Camera Visibility";
    case RenderPassProfilerGroup::Geometry:
      return "Geometry";
    case RenderPassProfilerGroup::Lighting:
      return "Lighting";
    case RenderPassProfilerGroup::AmbientOcclusionAndDdgi:
      return "AO / DDGI";
    case RenderPassProfilerGroup::PostProcessing:
      return "Post Processing";
    case RenderPassProfilerGroup::ReflectionProbes:
      return "Reflection Probes";
    case RenderPassProfilerGroup::EditorAndUi:
      return "Editor / UI";
    case RenderPassProfilerGroup::RayTracing:
      return "Ray Tracing";
    case RenderPassProfilerGroup::Other:
      return "Other";
  }
  return "Other";
}

GpuTimestampQueue GetTimestampQueue(const RenderPassQueue queue) {
  if (queue == RenderPassQueue::Compute) {
    return GpuTimestampQueue::Compute;
  }
  return queue == RenderPassQueue::RayTracing ? GpuTimestampQueue::RayTracing : GpuTimestampQueue::Graphics;
}
}  // namespace

GpuTimestampScopeToken evo_engine::BeginRenderPassGpuTimestamp(const VkCommandBuffer vk_command_buffer,
                                                               const RenderGraphExecutionContext& context,
                                                               const uint64_t view_id, const uint64_t instance_id) {
  const auto* descriptor = context.GetCurrentPassDescriptor();
  if (!descriptor) {
    return {};
  }
  return Platform::BeginGpuTimestampScope(
      vk_command_buffer,
      {descriptor->name,
       descriptor->profiler_display_name.empty() ? descriptor->name : descriptor->profiler_display_name,
       GetProfilerGroupName(descriptor->profiler_group), GetTimestampQueue(descriptor->queue), view_id, instance_id});
}

RenderPassGpuTimestampScope::RenderPassGpuTimestampScope(const VkCommandBuffer vk_command_buffer,
                                                         const RenderGraphExecutionContext& context,
                                                         const uint64_t view_id, const uint64_t instance_id)
    : vk_command_buffer_(vk_command_buffer),
      token_(BeginRenderPassGpuTimestamp(vk_command_buffer, context, view_id, instance_id)) {
}

RenderPassGpuTimestampScope::~RenderPassGpuTimestampScope() {
  Platform::EndGpuTimestampScope(vk_command_buffer_, token_);
}

namespace {
VkImageLayout ToVkImageLayout(const RenderResourceState state) {
  switch (state) {
    case RenderResourceState::Undefined:
      return VK_IMAGE_LAYOUT_UNDEFINED;
    case RenderResourceState::Present:
      return VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
    default:
      return VK_IMAGE_LAYOUT_GENERAL;
  }
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

VkPipelineStageFlags2 ShaderStages(const RenderPassQueue queue) {
  switch (queue) {
    case RenderPassQueue::Compute:
      return VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT;
    case RenderPassQueue::RayTracing:
      return VK_PIPELINE_STAGE_2_RAY_TRACING_SHADER_BIT_KHR;
    case RenderPassQueue::Graphics:
      return VK_PIPELINE_STAGE_2_ALL_GRAPHICS_BIT | VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT;
  }
  return VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT;
}

VkPipelineStageFlags2 ResourceStages(const RenderResourceState state, const RenderPassQueue queue) {
  switch (state) {
    case RenderResourceState::Undefined:
    case RenderResourceState::Present:
      return VK_PIPELINE_STAGE_2_NONE;
    case RenderResourceState::ColorAttachment:
      return VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT;
    case RenderResourceState::DepthAttachment:
      return VK_PIPELINE_STAGE_2_EARLY_FRAGMENT_TESTS_BIT | VK_PIPELINE_STAGE_2_LATE_FRAGMENT_TESTS_BIT;
    case RenderResourceState::TransferSource:
    case RenderResourceState::TransferDestination:
    case RenderResourceState::TransferDestinationGeneral:
      return VK_PIPELINE_STAGE_2_TRANSFER_BIT;
    case RenderResourceState::AccelerationStructureRead:
      return VK_PIPELINE_STAGE_2_ACCELERATION_STRUCTURE_BUILD_BIT_KHR | ShaderStages(queue);
    case RenderResourceState::ShaderRead:
    case RenderResourceState::StorageReadWrite:
    case RenderResourceState::General:
      return ShaderStages(queue);
  }
  return VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT;
}

VkAccessFlags2 ResourceAccess(const RenderResourceState state, const RenderResourceUsage usage) {
  const bool read = usage != RenderResourceUsage::Write;
  const bool write = usage != RenderResourceUsage::Read;
  switch (state) {
    case RenderResourceState::Undefined:
    case RenderResourceState::Present:
      return VK_ACCESS_2_NONE;
    case RenderResourceState::ColorAttachment:
      return (read ? VK_ACCESS_2_COLOR_ATTACHMENT_READ_BIT : 0) | (write ? VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT : 0);
    case RenderResourceState::DepthAttachment:
      return (read ? VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_READ_BIT : 0) |
             (write ? VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT : 0);
    case RenderResourceState::ShaderRead:
      return VK_ACCESS_2_SHADER_READ_BIT;
    case RenderResourceState::StorageReadWrite:
    case RenderResourceState::General:
      return (read ? VK_ACCESS_2_SHADER_READ_BIT : 0) | (write ? VK_ACCESS_2_SHADER_WRITE_BIT : 0);
    case RenderResourceState::TransferSource:
      return VK_ACCESS_2_TRANSFER_READ_BIT;
    case RenderResourceState::TransferDestination:
    case RenderResourceState::TransferDestinationGeneral:
      return VK_ACCESS_2_TRANSFER_WRITE_BIT;
    case RenderResourceState::AccelerationStructureRead:
      return VK_ACCESS_2_ACCELERATION_STRUCTURE_READ_BIT_KHR;
  }
  return VK_ACCESS_2_MEMORY_READ_BIT | VK_ACCESS_2_MEMORY_WRITE_BIT;
}

void ApplyGraphImageBarrier(const VkCommandBuffer vk_command_buffer, const std::shared_ptr<Image>& image,
                            const VkImageLayout target_layout, const RenderResourceBarrierPlan& barrier,
                            const bool force_memory_barrier) {
  if (!image || target_layout == VK_IMAGE_LAYOUT_UNDEFINED ||
      (!force_memory_barrier && image->GetLayout() == target_layout)) {
    return;
  }
  if (image->GetLayout() != target_layout) {
    image->TransitImageLayout(vk_command_buffer, target_layout);
  }
  if (!force_memory_barrier) {
    return;
  }
  VkImageMemoryBarrier2 image_barrier{};
  image_barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2;
  image_barrier.srcStageMask = ResourceStages(barrier.previous_state, barrier.previous_queue);
  image_barrier.srcAccessMask = ResourceAccess(barrier.previous_state, barrier.previous_usage);
  image_barrier.dstStageMask = ResourceStages(barrier.next_state, barrier.next_queue);
  image_barrier.dstAccessMask = ResourceAccess(barrier.next_state, barrier.next_usage);
  image_barrier.oldLayout = target_layout;
  image_barrier.newLayout = target_layout;
  image_barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
  image_barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
  image_barrier.image = image->GetVkImage();
  image_barrier.subresourceRange.aspectMask = image->GetFormat() == Platform::Constants::render_texture_depth ||
                                                      image->GetFormat() == Platform::Constants::shadow_map
                                                  ? VK_IMAGE_ASPECT_DEPTH_BIT
                                                  : VK_IMAGE_ASPECT_COLOR_BIT;
  if ((image_barrier.subresourceRange.aspectMask & VK_IMAGE_ASPECT_DEPTH_BIT) != 0 && image->HasStencilComponent()) {
    image_barrier.subresourceRange.aspectMask |= VK_IMAGE_ASPECT_STENCIL_BIT;
  }
  image_barrier.subresourceRange.baseMipLevel = 0;
  image_barrier.subresourceRange.levelCount = VK_REMAINING_MIP_LEVELS;
  image_barrier.subresourceRange.baseArrayLayer = 0;
  image_barrier.subresourceRange.layerCount = VK_REMAINING_ARRAY_LAYERS;
  VkDependencyInfo dependency{};
  dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
  dependency.imageMemoryBarrierCount = 1;
  dependency.pImageMemoryBarriers = &image_barrier;
  vkCmdPipelineBarrier2(vk_command_buffer, &dependency);
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
          ApplyGraphImageBarrier(vk_command_buffer, image, target_layout, barrier, force_memory_barrier);
        }
        return;
      }
      ApplyGraphImageBarrier(vk_command_buffer, binding->image, target_layout, barrier, force_memory_barrier);
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
