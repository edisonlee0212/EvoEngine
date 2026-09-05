#include "RenderPasses/DeferredComputeLightingPass.hpp"

#include "Camera.hpp"
#include "ComputePipeline.hpp"
#include "Console.hpp"
#include "GraphicsResources.hpp"
#include "Platform.hpp"
#include "RenderInstanceStorage.hpp"
#include "RenderPasses/RenderPassUtilities.hpp"
#include "RenderTexture.hpp"
#include "SdfgiResources.hpp"

using namespace evo_engine;

RenderPassDescriptor DeferredComputeLightingPass::CreateDescriptor(const bool ambient_occlusion_enabled,
                                                                   const bool depth_pyramid_enabled) {
  const char* dependency = RenderPassNames::deferred_geometry;
  if (ambient_occlusion_enabled) {
    dependency = RenderPassNames::ambient_occlusion;
  } else if (depth_pyramid_enabled) {
    dependency = RenderPassNames::depth_pyramid;
  }
  RenderPassDescriptor descriptor{
      RenderPassNames::deferred_camera,
      RenderPassQueue::Graphics,
      RenderPassScope::Camera,
      {{RenderResourceNames::frame_per_frame_descriptor_set, RenderResourceUsage::Read, RenderResourceState::General},
       {RenderResourceNames::lighting_directional_shadow_map, RenderResourceUsage::Read,
        RenderResourceState::ShaderRead},
       {RenderResourceNames::camera_depth, RenderResourceUsage::Read, RenderResourceState::ShaderRead},
       {RenderResourceNames::camera_g_buffer, RenderResourceUsage::ReadWrite, RenderResourceState::StorageReadWrite},
       {RenderResourceNames::camera_color, RenderResourceUsage::Write, RenderResourceState::StorageReadWrite}},
      {dependency},
      RenderPassProfilerGroup::Lighting,
      "Deferred Compute Lighting"};
  if (ambient_occlusion_enabled) {
    descriptor.resources.push_back(
        {RenderResourceNames::camera_ambient_occlusion, RenderResourceUsage::Read, RenderResourceState::ShaderRead});
  }
  return descriptor;
}

void DeferredComputeLightingPass::Execute(const RenderGraphExecutionContext& context, const Parameters& parameters) {
  if (!parameters.record_commands || !parameters.camera || !parameters.camera->GetRenderTexture() ||
      !parameters.per_frame_descriptor_set || !parameters.lighting_descriptor_set ||
      !parameters.raster_lighting_texture_descriptor_set || !parameters.pipeline ||
      !parameters.pipeline->Initialized() || !parameters.descriptor_set_layout || !parameters.transient_resources ||
      parameters.camera_index < 0 || !parameters.camera->GetGBufferDescriptorSet()) {
    EVOENGINE_ERROR("Deferred compute lighting pass prerequisites are unavailable.");
    return;
  }
  parameters.record_commands([&](const VkCommandBuffer vk_command_buffer) {
    if (parameters.sdfgi_resources)
      parameters.sdfgi_resources->OrderAccess(vk_command_buffer, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                              VK_ACCESS_2_SHADER_READ_BIT);
    ApplyGraphResourceBarriers(vk_command_buffer, context);
    const RenderPassGpuTimestampScope gpu_timestamp(vk_command_buffer, context,
                                                    parameters.camera->GetHandle().GetValue(),
                                                    static_cast<uint64_t>(parameters.camera_index));
    std::vector<VkRenderingAttachmentInfo> g_buffer_attachments;
    parameters.camera->AppendGBufferColorAttachmentInfos(g_buffer_attachments, VK_ATTACHMENT_LOAD_OP_LOAD,
                                                         VK_ATTACHMENT_STORE_OP_STORE);
    if (g_buffer_attachments.size() != 5u) {
      EVOENGINE_ERROR("Deferred compute lighting requires exactly five G-buffer attachments.");
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
    image_info.imageView = parameters.camera->GetRenderTexture()->GetColorImageView()->GetVkImageView();
    descriptor_set->UpdateImageDescriptorBinding(5, image_info);

    parameters.pipeline->Bind(vk_command_buffer);
    parameters.pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                           parameters.per_frame_descriptor_set->GetVkDescriptorSet());
    parameters.pipeline->BindDescriptorSet(vk_command_buffer, 1,
                                           parameters.camera->GetGBufferDescriptorSet()->GetVkDescriptorSet());
    parameters.pipeline->BindDescriptorSet(vk_command_buffer, 2,
                                           parameters.lighting_descriptor_set->GetVkDescriptorSet());
    parameters.pipeline->BindDescriptorSet(vk_command_buffer, 3,
                                           parameters.raster_lighting_texture_descriptor_set->GetVkDescriptorSet());
    parameters.pipeline->BindDescriptorSet(vk_command_buffer, 4, descriptor_set->GetVkDescriptorSet());
    if (parameters.sdfgi_descriptor_set) {
      parameters.pipeline->BindDescriptorSet(vk_command_buffer, 5,
                                             parameters.sdfgi_descriptor_set->GetVkDescriptorSet());
      parameters.sdfgi_resources->gather_camera_ids.push_back(parameters.camera->GetHandle().GetValue());
    }
    RenderInstancePushConstant push_constant;
    push_constant.instance_index = parameters.reflection_probe_capture ? 2 : 0;
    push_constant.camera_index = parameters.camera_index;
    push_constant.light_split_index =
        parameters.directional_shadow_camera_index >= 0 ? -parameters.directional_shadow_camera_index - 1 : 256;
    push_constant.meshlet_culling_flags = parameters.scene_camera ? 1u : 0u;
    parameters.pipeline->PushConstant(vk_command_buffer, 0, push_constant);
    const auto extent = parameters.camera->GetRenderTexture()->GetExtent();
    parameters.pipeline->Dispatch(vk_command_buffer, Platform::DivUp(extent.width, 16),
                                  Platform::DivUp(extent.height, 16));
    parameters.transient_resources->RetainDescriptorSet(descriptor_set);
    ApplyGraphResourceReleaseBarriers(vk_command_buffer, context, RenderPassQueue::Graphics);
  });
}
