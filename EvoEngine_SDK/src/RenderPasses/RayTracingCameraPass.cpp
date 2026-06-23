#include "RenderPasses/RayTracingCameraPass.hpp"

#include "Camera.hpp"
#include "GraphicsResources.hpp"
#include "Platform.hpp"
#include "RayTracingPipeline.hpp"
#include "RenderInstanceStorage.hpp"
#include "RenderPasses/RenderPassUtilities.hpp"
#include "RenderTexture.hpp"

using namespace evo_engine;

RenderPassDescriptor RayTracingCameraPass::CreateDescriptor() {
  return {
      RenderPassNames::ray_tracing_camera,
      RenderPassQueue::RayTracing,
      RenderPassScope::Camera,
      {{RenderResourceNames::frame_per_frame_descriptor_set, RenderResourceUsage::Read, RenderResourceState::General},
       {RenderResourceNames::frame_ray_tracing_descriptor_set, RenderResourceUsage::Read, RenderResourceState::General},
       {RenderResourceNames::scene_mesh_tlas, RenderResourceUsage::Read,
        RenderResourceState::AccelerationStructureRead},
       {RenderResourceNames::camera_color, RenderResourceUsage::Write, RenderResourceState::StorageReadWrite}}};
}

void RayTracingCameraPass::Execute(const RenderGraphExecutionContext& context, const Parameters& parameters) {
  if (!parameters.record_commands) {
    return;
  }
  parameters.record_commands([&](const VkCommandBuffer vk_command_buffer) {
    const auto render_texture = parameters.camera ? parameters.camera->GetRenderTexture() : nullptr;
    if (!render_texture || !parameters.pipeline || !parameters.per_frame_descriptor_set ||
        !parameters.ray_tracing_descriptor_set || !render_texture->GetStorageDescriptorSet()) {
      return;
    }
    Platform::EverythingBarrier(vk_command_buffer);
    ApplyGraphResourceBarriers(vk_command_buffer, context);
    Platform::EverythingBarrier(vk_command_buffer);
    parameters.pipeline->Bind(vk_command_buffer);
    parameters.pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                           parameters.per_frame_descriptor_set->GetVkDescriptorSet());
    parameters.pipeline->BindDescriptorSet(vk_command_buffer, 1,
                                           parameters.ray_tracing_descriptor_set->GetVkDescriptorSet());
    parameters.pipeline->BindDescriptorSet(vk_command_buffer, 2,
                                           render_texture->GetStorageDescriptorSet()->GetVkDescriptorSet());
    RayTracingCameraPushConstant push_constant;
    push_constant.camera_index = parameters.camera_index;
    push_constant.frame_id = parameters.frame_id;
    parameters.pipeline->PushConstant(vk_command_buffer, 0, push_constant);
    parameters.pipeline->Trace(vk_command_buffer, render_texture->GetExtent().width, render_texture->GetExtent().height,
                               1);
    Platform::EverythingBarrier(vk_command_buffer);
    ApplyGraphResourceReleaseBarriers(vk_command_buffer, context, RenderPassQueue::RayTracing);
  });
}
