#include "RenderPasses/DeferredMaterialResolvePass.hpp"

#include "Camera.hpp"
#include "ComputePipeline.hpp"
#include "Console.hpp"
#include "GraphicsResources.hpp"
#include "Platform.hpp"
#include "RenderPasses/RenderPassUtilities.hpp"
#include "RenderTexture.hpp"

using namespace evo_engine;

RenderPassDescriptor DeferredMaterialResolvePass::CreateDescriptor(const bool ambient_occlusion_enabled,
                                                                   const bool depth_pyramid_enabled) {
  const char* dependency = RenderPassNames::deferred_geometry;
  if (ambient_occlusion_enabled) {
    dependency = RenderPassNames::ambient_occlusion;
  } else if (depth_pyramid_enabled) {
    dependency = RenderPassNames::depth_pyramid;
  }
  return {
      RenderPassNames::deferred_material_resolve,
      RenderPassQueue::Graphics,
      RenderPassScope::Camera,
      {{RenderResourceNames::frame_per_frame_descriptor_set, RenderResourceUsage::Read, RenderResourceState::General},
       {RenderResourceNames::camera_depth, RenderResourceUsage::Read, RenderResourceState::ShaderRead},
       {RenderResourceNames::camera_g_buffer, RenderResourceUsage::ReadWrite, RenderResourceState::StorageReadWrite}},
      {dependency},
      RenderPassProfilerGroup::Lighting,
      "Deferred Material Resolve"};
}

void DeferredMaterialResolvePass::Execute(const RenderGraphExecutionContext& context, const Parameters& parameters) {
  if (!parameters.record_commands || !parameters.camera || !parameters.camera->GetRenderTexture() ||
      !parameters.per_frame_descriptor_set || !parameters.pipeline || !parameters.pipeline->Initialized() ||
      !parameters.descriptor_set_layout || !parameters.transient_resources || parameters.camera_index < 0 ||
      !parameters.camera->GetGBufferDescriptorSet()) {
    EVOENGINE_ERROR("Deferred material resolve pass prerequisites are unavailable.");
    return;
  }
  parameters.record_commands([&](const VkCommandBuffer vk_command_buffer) {
    ApplyGraphResourceBarriers(vk_command_buffer, context);
    const RenderPassGpuTimestampScope gpu_timestamp(vk_command_buffer, context,
                                                    parameters.camera->GetHandle().GetValue(),
                                                    static_cast<uint64_t>(parameters.camera_index));
    std::vector<VkRenderingAttachmentInfo> g_buffer_attachments;
    parameters.camera->AppendGBufferColorAttachmentInfos(g_buffer_attachments, VK_ATTACHMENT_LOAD_OP_LOAD,
                                                         VK_ATTACHMENT_STORE_OP_STORE);
    if (g_buffer_attachments.size() != 5u) {
      EVOENGINE_ERROR("Deferred material resolve requires exactly five G-buffer attachments.");
      ApplyGraphResourceReleaseBarriers(vk_command_buffer, context, RenderPassQueue::Graphics);
      return;
    }

    const auto descriptor_set = std::make_shared<DescriptorSet>(parameters.descriptor_set_layout);
    VkDescriptorImageInfo image_info{};
    image_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
    image_info.sampler = VK_NULL_HANDLE;
    for (uint32_t binding = 0; binding < 4; ++binding) {
      image_info.imageView = g_buffer_attachments[binding].imageView;
      descriptor_set->UpdateImageDescriptorBinding(binding, image_info);
    }
    image_info.imageView = g_buffer_attachments[4].imageView;
    descriptor_set->UpdateImageDescriptorBinding(4, image_info);

    parameters.pipeline->Bind(vk_command_buffer);
    parameters.pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                           parameters.per_frame_descriptor_set->GetVkDescriptorSet());
    parameters.pipeline->BindDescriptorSet(vk_command_buffer, 1,
                                           parameters.camera->GetGBufferDescriptorSet()->GetVkDescriptorSet());
    parameters.pipeline->BindDescriptorSet(vk_command_buffer, 2, descriptor_set->GetVkDescriptorSet());
    parameters.pipeline->PushConstant(vk_command_buffer, 0, parameters.camera_index);
    const auto extent = parameters.camera->GetRenderTexture()->GetExtent();
    parameters.pipeline->Dispatch(vk_command_buffer, Platform::DivUp(extent.width, 16),
                                  Platform::DivUp(extent.height, 16));
    parameters.transient_resources->RetainDescriptorSet(descriptor_set);
    ApplyGraphResourceReleaseBarriers(vk_command_buffer, context, RenderPassQueue::Graphics);
  });
}
