#pragma once
#include "Platform.hpp"
#include "RenderGraph.hpp"

namespace evo_engine {
class GraphicsPipeline;
class Image;
class ImageView;
class RenderInstanceStorage;

[[nodiscard]] GpuTimestampScopeToken BeginRenderPassGpuTimestamp(VkCommandBuffer vk_command_buffer,
                                                                 const RenderGraphExecutionContext& context,
                                                                 uint64_t view_id = 0, uint64_t instance_id = 0);

class RenderPassGpuTimestampScope final {
 public:
  RenderPassGpuTimestampScope(VkCommandBuffer vk_command_buffer, const RenderGraphExecutionContext& context,
                              uint64_t view_id = 0, uint64_t instance_id = 0);
  ~RenderPassGpuTimestampScope();
  RenderPassGpuTimestampScope(const RenderPassGpuTimestampScope&) = delete;
  RenderPassGpuTimestampScope& operator=(const RenderPassGpuTimestampScope&) = delete;

 private:
  VkCommandBuffer vk_command_buffer_ = VK_NULL_HANDLE;
  GpuTimestampScopeToken token_{};
};

[[nodiscard]] uint32_t CalculateMipDimension(uint32_t base_dimension, uint32_t mip_level);
[[nodiscard]] std::shared_ptr<ImageView> CreateGraphImageMipView(const std::shared_ptr<Image>& image,
                                                                 uint32_t mip_level);
void ApplyGraphResourceBarriers(VkCommandBuffer vk_command_buffer, const RenderGraphExecutionContext& context,
                                RenderPassQueue recorded_queue = RenderPassQueue::Graphics);
void ApplyGraphResourceReleaseBarriers(VkCommandBuffer vk_command_buffer, const RenderGraphExecutionContext& context,
                                       RenderPassQueue recorded_queue);
void BindRasterMaterialDescriptorSet(VkCommandBuffer vk_command_buffer,
                                     const std::shared_ptr<GraphicsPipeline>& graphics_pipeline,
                                     const std::shared_ptr<RenderInstanceStorage>& render_instances,
                                     int32_t material_index);
void ClearGraphColorImage(VkCommandBuffer vk_command_buffer, const std::shared_ptr<Image>& image,
                          const VkClearColorValue& clear_value);
}  // namespace evo_engine
