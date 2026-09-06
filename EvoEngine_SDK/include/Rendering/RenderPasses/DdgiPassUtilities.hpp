#pragma once
#include "EvoEngine_SDK_PCH.hpp"

namespace evo_engine {
class EVOENGINE_API Buffer;
class EVOENGINE_API Image;

[[nodiscard]] VkDescriptorImageInfo CreateDdgiFallbackImageInfo();
[[nodiscard]] bool IsValidDescriptorImageInfo(const VkDescriptorImageInfo& image_info);
void ApplyDdgiBufferDependency(VkCommandBuffer command_buffer, const std::shared_ptr<Buffer>& buffer,
                               VkPipelineStageFlags2 source_stages, VkAccessFlags2 source_access,
                               VkPipelineStageFlags2 destination_stages, VkAccessFlags2 destination_access,
                               VkDeviceSize offset = 0, VkDeviceSize size = VK_WHOLE_SIZE);
void ApplyDdgiImageDependency(VkCommandBuffer command_buffer, const std::shared_ptr<Image>& image,
                              VkPipelineStageFlags2 source_stages, VkAccessFlags2 source_access,
                              VkPipelineStageFlags2 destination_stages, VkAccessFlags2 destination_access);
void AcquireDdgiFrameResources(VkCommandBuffer command_buffer, const std::shared_ptr<Image>& irradiance_atlas,
                               const std::shared_ptr<Image>& visibility_atlas,
                               const std::shared_ptr<Buffer>& probe_state,
                               const std::shared_ptr<Buffer>& probe_metadata,
                               const std::shared_ptr<Buffer>& selected_ray_diagnostics);
void PublishDdgiFrameResources(VkCommandBuffer command_buffer, const std::shared_ptr<Image>& irradiance_atlas,
                               const std::shared_ptr<Image>& visibility_atlas,
                               const std::shared_ptr<Buffer>& probe_state,
                               const std::shared_ptr<Buffer>& probe_metadata,
                               const std::shared_ptr<Buffer>& selected_ray_diagnostics);
}  // namespace evo_engine
