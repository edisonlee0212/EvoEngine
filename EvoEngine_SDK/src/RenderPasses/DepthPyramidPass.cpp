#include "RenderPasses/DepthPyramidPass.hpp"

#include "Camera.hpp"
#include "ComputePipeline.hpp"
#include "GraphicsResources.hpp"
#include "Platform.hpp"
#include "RenderPasses/RenderPassUtilities.hpp"
#include "RenderTexture.hpp"

using namespace evo_engine;

RenderPassDescriptor DepthPyramidPass::CreateDescriptor() {
  return {
      RenderPassNames::depth_pyramid,
      RenderPassQueue::Graphics,
      RenderPassScope::Camera,
      {{RenderResourceNames::camera_depth, RenderResourceUsage::Read, RenderResourceState::ShaderRead},
       {RenderResourceNames::camera_depth_pyramid, RenderResourceUsage::Write, RenderResourceState::StorageReadWrite}},
      {RenderPassNames::deferred_geometry}};
}

void DepthPyramidPass::Execute(const RenderGraphExecutionContext& context, const Parameters& parameters) {
  if (!parameters.record_commands) {
    return;
  }
  parameters.record_commands([&](const VkCommandBuffer vk_command_buffer) {
    ApplyGraphResourceBarriers(vk_command_buffer, context);
    const auto* binding = context.GetResourceBinding(RenderResourceNames::camera_depth_pyramid);
    const auto* depth_binding = context.GetResourceBinding(RenderResourceNames::camera_depth);
    if (!binding || !binding->image) {
      return;
    }
    const auto clear_depth_pyramid = [&] {
      VkClearColorValue depth_pyramid_clear{};
      depth_pyramid_clear.float32[0] = 1.0f;
      depth_pyramid_clear.float32[1] = 1.0f;
      depth_pyramid_clear.float32[2] = 1.0f;
      depth_pyramid_clear.float32[3] = 1.0f;
      ClearGraphColorImage(vk_command_buffer, binding->image, depth_pyramid_clear);
    };
    if (!depth_binding || !depth_binding->image || !parameters.camera || !parameters.camera->GetRenderTexture() ||
        !parameters.pipeline || !parameters.pipeline->Initialized() || !parameters.transient_resources) {
      clear_depth_pyramid();
      return;
    }

    const auto depth_pyramid_image = binding->image;
    const auto base_extent = depth_pyramid_image->GetExtent();
    const auto mip_levels = depth_pyramid_image->GetMipLevels();
    if (base_extent.width == 0 || base_extent.height == 0 || mip_levels == 0) {
      return;
    }

    std::vector<std::shared_ptr<ImageView>> depth_pyramid_views;
    depth_pyramid_views.reserve(mip_levels);
    for (uint32_t mip_level = 0; mip_level < mip_levels; ++mip_level) {
      auto image_view = CreateGraphImageMipView(depth_pyramid_image, mip_level);
      parameters.transient_resources->RetainImageView(image_view);
      depth_pyramid_views.emplace_back(std::move(image_view));
    }

    parameters.pipeline->Bind(vk_command_buffer);
    for (uint32_t mip_level = 0; mip_level < mip_levels; ++mip_level) {
      const auto descriptor_set = std::make_shared<DescriptorSet>(parameters.descriptor_set_layout);
      VkDescriptorImageInfo image_info{};
      image_info.imageLayout = mip_level == 0 ? depth_binding->image->GetLayout() : depth_pyramid_image->GetLayout();
      image_info.imageView = mip_level == 0
                                 ? parameters.camera->GetRenderTexture()->GetDepthImageView()->GetVkImageView()
                                 : depth_pyramid_views[mip_level - 1]->GetVkImageView();
      image_info.sampler = parameters.camera->GetRenderTexture()->GetDepthSampler()->GetVkSampler();
      descriptor_set->UpdateImageDescriptorBinding(0, image_info);

      image_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
      image_info.imageView = depth_pyramid_views[mip_level]->GetVkImageView();
      image_info.sampler = VK_NULL_HANDLE;
      descriptor_set->UpdateImageDescriptorBinding(1, image_info);

      const glm::uvec4 push_constant{
          mip_level == 0 ? base_extent.width : CalculateMipDimension(base_extent.width, mip_level - 1),
          mip_level == 0 ? base_extent.height : CalculateMipDimension(base_extent.height, mip_level - 1),
          CalculateMipDimension(base_extent.width, mip_level), CalculateMipDimension(base_extent.height, mip_level)};
      parameters.pipeline->BindDescriptorSet(vk_command_buffer, 0, descriptor_set->GetVkDescriptorSet());
      parameters.pipeline->PushConstant(vk_command_buffer, 0, push_constant);
      parameters.pipeline->Dispatch(vk_command_buffer, Platform::DivUp(push_constant.z, 16),
                                    Platform::DivUp(push_constant.w, 16));
      parameters.transient_resources->RetainDescriptorSet(descriptor_set);
      Platform::EverythingBarrier(vk_command_buffer);
    }
    ApplyGraphResourceReleaseBarriers(vk_command_buffer, context, RenderPassQueue::Graphics);
  });
}
