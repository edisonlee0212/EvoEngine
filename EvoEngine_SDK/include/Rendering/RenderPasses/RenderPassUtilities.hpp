#pragma once
#include "RenderGraph.hpp"

namespace evo_engine {
class Image;
class ImageView;

[[nodiscard]] uint32_t CalculateMipDimension(uint32_t base_dimension, uint32_t mip_level);
[[nodiscard]] std::shared_ptr<ImageView> CreateGraphImageMipView(const std::shared_ptr<Image>& image,
                                                                 uint32_t mip_level);
void ApplyGraphResourceBarriers(VkCommandBuffer vk_command_buffer, const RenderGraphExecutionContext& context,
                                RenderPassQueue recorded_queue = RenderPassQueue::Graphics);
void ApplyGraphResourceReleaseBarriers(VkCommandBuffer vk_command_buffer, const RenderGraphExecutionContext& context,
                                       RenderPassQueue recorded_queue);
void ClearGraphColorImage(VkCommandBuffer vk_command_buffer, const std::shared_ptr<Image>& image,
                          const VkClearColorValue& clear_value);
}  // namespace evo_engine
