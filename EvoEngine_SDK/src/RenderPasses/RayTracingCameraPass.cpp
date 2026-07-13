#include "RenderPasses/RayTracingCameraPass.hpp"

#include "Camera.hpp"
#include "ComputePipeline.hpp"
#include "GraphicsResources.hpp"
#include "Platform.hpp"
#include "RayTracingPipeline.hpp"
#include "RenderInstanceStorage.hpp"
#include "RenderPasses/RenderPassUtilities.hpp"
#include "RenderTexture.hpp"

#include <algorithm>

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
       {RenderResourceNames::camera_ray_hit_distance, RenderResourceUsage::Write,
        RenderResourceState::StorageReadWrite},
       {RenderResourceNames::camera_color, RenderResourceUsage::Write, RenderResourceState::StorageReadWrite}}};
}

RenderPassDescriptor RayQueryCameraPass::CreateDescriptor() {
  return {
      RenderPassNames::ray_query_camera,
      RenderPassQueue::Graphics,
      RenderPassScope::Camera,
      {{RenderResourceNames::frame_per_frame_descriptor_set, RenderResourceUsage::Read, RenderResourceState::General},
       {RenderResourceNames::frame_ray_tracing_descriptor_set, RenderResourceUsage::Read, RenderResourceState::General},
       {RenderResourceNames::scene_mesh_tlas, RenderResourceUsage::Read,
        RenderResourceState::AccelerationStructureRead},
       {RenderResourceNames::camera_ray_hit_distance, RenderResourceUsage::Write,
        RenderResourceState::StorageReadWrite},
       {RenderResourceNames::camera_color, RenderResourceUsage::Write, RenderResourceState::StorageReadWrite}}};
}

void RayTracingCameraPass::Execute(const RenderGraphExecutionContext& context, const Parameters& parameters) {
  if (!parameters.record_commands) {
    return;
  }
  parameters.record_commands([&](const VkCommandBuffer vk_command_buffer) {
    const auto render_texture = parameters.camera ? parameters.camera->GetRenderTexture() : nullptr;
    if (!render_texture || !parameters.pipeline || !parameters.pipeline->Initialized() ||
        !parameters.per_frame_descriptor_set || !parameters.ray_tracing_descriptor_set ||
        !parameters.output_descriptor_set_layout || !parameters.transient_resources || !parameters.history_resources) {
      return;
    }
    auto& history_resources = *parameters.history_resources;
    if (!history_resources.radiance_image || !history_resources.radiance_view || !history_resources.convergence_image ||
        !history_resources.convergence_view) {
      return;
    }
    Platform::EverythingBarrier(vk_command_buffer);
    ApplyGraphResourceBarriers(vk_command_buffer, context);
    Platform::EverythingBarrier(vk_command_buffer);
    const auto* hit_distance_binding = context.GetResourceBinding(RenderResourceNames::camera_ray_hit_distance);
    if (!hit_distance_binding || !hit_distance_binding->image) {
      ApplyGraphResourceReleaseBarriers(vk_command_buffer, context, RenderPassQueue::RayTracing);
      return;
    }
    const auto hit_distance_view = CreateGraphImageMipView(hit_distance_binding->image, 0);
    if (!hit_distance_view) {
      ApplyGraphResourceReleaseBarriers(vk_command_buffer, context, RenderPassQueue::RayTracing);
      return;
    }
    parameters.transient_resources->RetainImageView(hit_distance_view);
    parameters.transient_resources->RetainImageView(history_resources.radiance_view);
    parameters.transient_resources->RetainImageView(history_resources.convergence_view);
    history_resources.radiance_image->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);
    history_resources.convergence_image->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);
    const auto output_descriptor_set = std::make_shared<DescriptorSet>(parameters.output_descriptor_set_layout);
    VkDescriptorImageInfo image_info{};
    image_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
    image_info.imageView = render_texture->GetColorImageView()->GetVkImageView();
    output_descriptor_set->UpdateImageDescriptorBinding(0, image_info);
    image_info.imageView = hit_distance_view->GetVkImageView();
    output_descriptor_set->UpdateImageDescriptorBinding(1, image_info);
    image_info.imageView = history_resources.radiance_view->GetVkImageView();
    output_descriptor_set->UpdateImageDescriptorBinding(2, image_info);
    image_info.imageView = history_resources.convergence_view->GetVkImageView();
    output_descriptor_set->UpdateImageDescriptorBinding(3, image_info);

    parameters.pipeline->Bind(vk_command_buffer);
    parameters.pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                           parameters.per_frame_descriptor_set->GetVkDescriptorSet());
    parameters.pipeline->BindDescriptorSet(vk_command_buffer, 1,
                                           parameters.ray_tracing_descriptor_set->GetVkDescriptorSet());
    parameters.pipeline->BindDescriptorSet(vk_command_buffer, 2, output_descriptor_set->GetVkDescriptorSet());
    RayTracingCameraPushConstant push_constant;
    push_constant.camera_index = parameters.camera_index;
    push_constant.frame_id = history_resources.valid ? history_resources.frame_id : 0u;
    push_constant.frame_samples = static_cast<uint32_t>(std::max(parameters.camera->camera_settings.sample_size, 1));
    push_constant.total_samples = push_constant.frame_id * push_constant.frame_samples;
    push_constant.shader_execution_reordering = Camera::ResolveShaderExecutionReorderingEnabled(
                                                    parameters.camera->camera_settings.shader_execution_reordering_mode)
                                                    ? 1u
                                                    : 0u;
    parameters.pipeline->PushConstant(vk_command_buffer, 0, push_constant);
    const auto gpu_timestamp = Platform::BeginGpuTimestampScope(vk_command_buffer, "Path Trace (RTX)");
    parameters.pipeline->Trace(vk_command_buffer, render_texture->GetExtent().width, render_texture->GetExtent().height,
                               1);
    Platform::EndGpuTimestampScope(vk_command_buffer, gpu_timestamp);
    history_resources.valid = true;
    ++history_resources.frame_id;
    parameters.transient_resources->RetainDescriptorSet(output_descriptor_set);
    Platform::EverythingBarrier(vk_command_buffer);
    ApplyGraphResourceReleaseBarriers(vk_command_buffer, context, RenderPassQueue::RayTracing);
  });
}

void RayQueryCameraPass::Execute(const RenderGraphExecutionContext& context, const Parameters& parameters) {
  if (!parameters.record_commands) {
    return;
  }
  parameters.record_commands([&](const VkCommandBuffer vk_command_buffer) {
    const auto render_texture = parameters.camera ? parameters.camera->GetRenderTexture() : nullptr;
    if (!render_texture || !parameters.pipeline || !parameters.pipeline->Initialized() ||
        !parameters.per_frame_descriptor_set || !parameters.ray_tracing_descriptor_set ||
        !parameters.output_descriptor_set_layout || !parameters.transient_resources || !parameters.history_resources) {
      return;
    }
    auto& history_resources = *parameters.history_resources;
    if (!history_resources.radiance_image || !history_resources.radiance_view || !history_resources.convergence_image ||
        !history_resources.convergence_view) {
      return;
    }
    Platform::EverythingBarrier(vk_command_buffer);
    ApplyGraphResourceBarriers(vk_command_buffer, context);
    Platform::EverythingBarrier(vk_command_buffer);
    const auto* hit_distance_binding = context.GetResourceBinding(RenderResourceNames::camera_ray_hit_distance);
    if (!hit_distance_binding || !hit_distance_binding->image) {
      ApplyGraphResourceReleaseBarriers(vk_command_buffer, context, RenderPassQueue::Graphics);
      return;
    }
    const auto hit_distance_view = CreateGraphImageMipView(hit_distance_binding->image, 0);
    if (!hit_distance_view) {
      ApplyGraphResourceReleaseBarriers(vk_command_buffer, context, RenderPassQueue::Graphics);
      return;
    }
    parameters.transient_resources->RetainImageView(hit_distance_view);
    parameters.transient_resources->RetainImageView(history_resources.radiance_view);
    parameters.transient_resources->RetainImageView(history_resources.convergence_view);
    history_resources.radiance_image->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);
    history_resources.convergence_image->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);
    const auto output_descriptor_set = std::make_shared<DescriptorSet>(parameters.output_descriptor_set_layout);
    VkDescriptorImageInfo image_info{};
    image_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
    image_info.imageView = render_texture->GetColorImageView()->GetVkImageView();
    output_descriptor_set->UpdateImageDescriptorBinding(0, image_info);
    image_info.imageView = hit_distance_view->GetVkImageView();
    output_descriptor_set->UpdateImageDescriptorBinding(1, image_info);
    image_info.imageView = history_resources.radiance_view->GetVkImageView();
    output_descriptor_set->UpdateImageDescriptorBinding(2, image_info);
    image_info.imageView = history_resources.convergence_view->GetVkImageView();
    output_descriptor_set->UpdateImageDescriptorBinding(3, image_info);

    parameters.pipeline->Bind(vk_command_buffer);
    parameters.pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                           parameters.per_frame_descriptor_set->GetVkDescriptorSet());
    parameters.pipeline->BindDescriptorSet(vk_command_buffer, 1,
                                           parameters.ray_tracing_descriptor_set->GetVkDescriptorSet());
    parameters.pipeline->BindDescriptorSet(vk_command_buffer, 2, output_descriptor_set->GetVkDescriptorSet());
    RayTracingCameraPushConstant push_constant;
    push_constant.camera_index = parameters.camera_index;
    push_constant.frame_id = history_resources.valid ? history_resources.frame_id : 0u;
    push_constant.frame_samples = static_cast<uint32_t>(std::max(parameters.camera->camera_settings.sample_size, 1));
    push_constant.total_samples = push_constant.frame_id * push_constant.frame_samples;
    parameters.pipeline->PushConstant(vk_command_buffer, 0, push_constant);
    const auto gpu_timestamp = Platform::BeginGpuTimestampScope(vk_command_buffer, "Path Trace (RQ)");
    parameters.pipeline->Dispatch(vk_command_buffer, Platform::DivUp(render_texture->GetExtent().width, 8),
                                  Platform::DivUp(render_texture->GetExtent().height, 8), 1);
    Platform::EndGpuTimestampScope(vk_command_buffer, gpu_timestamp);
    history_resources.valid = true;
    ++history_resources.frame_id;
    parameters.transient_resources->RetainDescriptorSet(output_descriptor_set);
    Platform::EverythingBarrier(vk_command_buffer);
    ApplyGraphResourceReleaseBarriers(vk_command_buffer, context, RenderPassQueue::Graphics);
  });
}
