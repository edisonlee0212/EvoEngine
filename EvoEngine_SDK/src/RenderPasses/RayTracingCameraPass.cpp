#include "RenderPasses/RayTracingCameraPass.hpp"

#include "Application.hpp"
#include "Camera.hpp"
#include "ComputePipeline.hpp"
#include "GraphicsResources.hpp"
#include "Platform.hpp"
#include "RayTracingPipeline.hpp"
#include "RenderInstanceStorage.hpp"
#include "RenderPasses/RenderPassUtilities.hpp"
#include "RenderTexture.hpp"

#include <algorithm>
#include <array>
#include <utility>

using namespace evo_engine;

namespace {
constexpr uint32_t kRayQueryCameraWorkGroupSize = 8;

void AddRayCameraOptionalOutputAccesses(RenderPassDescriptor& descriptor,
                                        const CameraSettings::RayOutputSettings& outputs) {
  const std::array output_resources{
      std::pair{outputs.albedo, RenderResourceNames::camera_ray_albedo},
      std::pair{outputs.normal, RenderResourceNames::camera_ray_normal},
      std::pair{outputs.ray_count, RenderResourceNames::camera_ray_count},
      std::pair{outputs.path_length, RenderResourceNames::camera_ray_path_length},
      std::pair{outputs.time, RenderResourceNames::camera_ray_time},
      std::pair{outputs.debug, RenderResourceNames::camera_ray_debug},
  };
  for (const auto& [enabled, name] : output_resources) {
    if (enabled) {
      descriptor.resources.push_back({name, RenderResourceUsage::Write, RenderResourceState::StorageReadWrite});
    }
  }
}

void ApplyRayCameraStorageDependencies(const VkCommandBuffer command_buffer, const std::shared_ptr<Image>& color,
                                       RayCameraHistoryResources& history, const VkPipelineStageFlags2 shader_stage) {
  const std::array<std::shared_ptr<Image>, 3> images{color, history.radiance_image, history.convergence_image};
  std::array<VkImageMemoryBarrier2, 3> barriers{};
  uint32_t barrier_count = 0;
  for (const auto& image : images) {
    if (!image) {
      continue;
    }
    if (image->GetLayout() != VK_IMAGE_LAYOUT_GENERAL) {
      image->TransitImageLayout(command_buffer, VK_IMAGE_LAYOUT_GENERAL);
    }
    auto& barrier = barriers[barrier_count++];
    barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2;
    barrier.srcStageMask = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT;
    barrier.srcAccessMask = VK_ACCESS_2_MEMORY_WRITE_BIT;
    barrier.dstStageMask = shader_stage;
    barrier.dstAccessMask = VK_ACCESS_2_SHADER_READ_BIT | VK_ACCESS_2_SHADER_WRITE_BIT;
    barrier.oldLayout = VK_IMAGE_LAYOUT_GENERAL;
    barrier.newLayout = VK_IMAGE_LAYOUT_GENERAL;
    barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrier.image = image->GetVkImage();
    barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    barrier.subresourceRange.baseMipLevel = 0;
    barrier.subresourceRange.levelCount = VK_REMAINING_MIP_LEVELS;
    barrier.subresourceRange.baseArrayLayer = 0;
    barrier.subresourceRange.layerCount = VK_REMAINING_ARRAY_LAYERS;
  }
  VkDependencyInfo dependency{};
  dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
  dependency.imageMemoryBarrierCount = barrier_count;
  dependency.pImageMemoryBarriers = barriers.data();
  vkCmdPipelineBarrier2(command_buffer, &dependency);
}

bool RayCameraOptionalOutputIsUint(const RayCameraOptionalOutput output) {
  return output == RayCameraOptionalOutput::RayCount || output == RayCameraOptionalOutput::PathLength ||
         output == RayCameraOptionalOutput::Time;
}

void RetainRayCameraOptionalOutputViews(RenderGraphTransientResourceStore& transient_resources,
                                        const RayCameraHistoryResources& history) {
  for (uint32_t index = 0u; index < kRayCameraOptionalOutputCount; ++index) {
    if (history.optional_outputs.images[index]) {
      transient_resources.RetainImage(history.optional_outputs.images[index]);
    }
    if (history.optional_outputs.views[index]) {
      transient_resources.RetainImageView(history.optional_outputs.views[index]);
    }
  }
  transient_resources.RetainImage(history.optional_outputs.uint_fallback_image);
  transient_resources.RetainImageView(history.optional_outputs.uint_fallback_view);
}

VkDescriptorImageInfo PrepareRayCameraUintFallback(const VkCommandBuffer command_buffer,
                                                   RayCameraHistoryResources& history) {
  auto& fallback = history.optional_outputs.uint_fallback_image;
  if (fallback && fallback->GetLayout() != VK_IMAGE_LAYOUT_GENERAL) {
    fallback->TransitImageLayout(command_buffer, VK_IMAGE_LAYOUT_GENERAL);
  }
  VkDescriptorImageInfo image_info{};
  image_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
  image_info.imageView = history.optional_outputs.uint_fallback_view->GetVkImageView();
  return image_info;
}

void UpdateRayCameraOptionalOutputDescriptors(const std::shared_ptr<DescriptorSet>& descriptor_set,
                                              const RayCameraHistoryResources& history,
                                              const VkDescriptorImageInfo& fallback_color_image_info,
                                              const VkDescriptorImageInfo& fallback_uint_image_info) {
  for (uint32_t index = 0u; index < kRayCameraOptionalOutputCount; ++index) {
    const auto output = static_cast<RayCameraOptionalOutput>(index);
    auto image_info = RayCameraOptionalOutputIsUint(output) ? fallback_uint_image_info : fallback_color_image_info;
    if (history.optional_outputs.views[index]) {
      image_info.imageView = history.optional_outputs.views[index]->GetVkImageView();
    }
    descriptor_set->UpdateImageDescriptorBinding(kRayCameraOutputDescriptorBaseBindingCount + index, image_info);
  }
}

}  // namespace

RenderPassDescriptor RayTracingCameraPass::CreateDescriptor(const char* pass_name,
                                                            const CameraSettings::RayOutputSettings outputs) {
  RenderPassDescriptor descriptor{
      pass_name,
      RenderPassQueue::RayTracing,
      RenderPassScope::Camera,
      {{RenderResourceNames::frame_per_frame_descriptor_set, RenderResourceUsage::Read, RenderResourceState::General},
       {RenderResourceNames::frame_ray_tracing_descriptor_set, RenderResourceUsage::Read, RenderResourceState::General},
       {RenderResourceNames::scene_mesh_tlas, RenderResourceUsage::Read,
        RenderResourceState::AccelerationStructureRead},
       {RenderResourceNames::camera_ray_hit_distance, RenderResourceUsage::Write,
        RenderResourceState::StorageReadWrite},
       {RenderResourceNames::camera_color, RenderResourceUsage::Write, RenderResourceState::StorageReadWrite}}};
  AddRayCameraOptionalOutputAccesses(descriptor, outputs);
  return descriptor;
}

RenderPassDescriptor RayQueryCameraPass::CreateDescriptor(const CameraSettings::RayOutputSettings outputs) {
  RenderPassDescriptor descriptor{
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
  AddRayCameraOptionalOutputAccesses(descriptor, outputs);
  return descriptor;
}

void RayTracingCameraPass::Execute(const RenderGraphExecutionContext& context, const Parameters& parameters) {
  if (!parameters.record_commands) {
    return;
  }
  parameters.record_commands([&](const VkCommandBuffer vk_command_buffer) {
    const auto render_texture = parameters.camera ? parameters.camera->GetRenderTexture() : nullptr;
    if (!render_texture || !parameters.pipeline || !parameters.pipeline->Initialized() ||
        !parameters.per_frame_descriptor_set || !parameters.ray_tracing_descriptor_set ||
        !parameters.output_descriptor_set || !parameters.transient_resources || !parameters.history_resources) {
      return;
    }
    auto& history_resources = *parameters.history_resources;
    if (!history_resources.radiance_image || !history_resources.radiance_view || !history_resources.convergence_image ||
        !history_resources.convergence_view) {
      return;
    }
    ApplyGraphResourceBarriers(vk_command_buffer, context, RenderPassQueue::RayTracing);
    ApplyRayCameraStorageDependencies(vk_command_buffer, render_texture->GetColorImage(), history_resources,
                                      VK_PIPELINE_STAGE_2_RAY_TRACING_SHADER_BIT_KHR);
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
    RetainRayCameraOptionalOutputViews(*parameters.transient_resources, history_resources);
    parameters.transient_resources->RetainImage(render_texture->GetColorImage());
    parameters.transient_resources->RetainImageView(render_texture->GetColorImageView());
    const auto& output_descriptor_set = parameters.output_descriptor_set;
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
    VkDescriptorImageInfo fallback_color_image_info{};
    fallback_color_image_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
    fallback_color_image_info.imageView = render_texture->GetColorImageView()->GetVkImageView();
    const auto fallback_uint_image_info = PrepareRayCameraUintFallback(vk_command_buffer, history_resources);
    UpdateRayCameraOptionalOutputDescriptors(output_descriptor_set, history_resources, fallback_color_image_info,
                                             fallback_uint_image_info);

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
    push_constant.shader_execution_reordering =
        (!Platform::RayTracingLinearSweptSpheresEnabled() &&
         Camera::ResolveShaderExecutionReorderingEnabled(
             parameters.camera->camera_settings.shader_execution_reordering_mode))
            ? 1u
            : 0u;
    push_constant.max_directional_light_size =
        ApplicationContext::Get().GetApplicationInfo().graphics_settings.max_directional_light_size;
    parameters.pipeline->PushConstant(vk_command_buffer, 0, push_constant);
    const auto gpu_timestamp = Platform::BeginGpuTimestampScope(vk_command_buffer, "Path Trace (RTX)");
    parameters.pipeline->Trace(vk_command_buffer, render_texture->GetExtent().width, render_texture->GetExtent().height,
                               1);
    Platform::EndGpuTimestampScope(vk_command_buffer, gpu_timestamp);
    history_resources.valid = true;
    ++history_resources.frame_id;
    parameters.transient_resources->RetainDescriptorSet(output_descriptor_set);
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
        !parameters.output_descriptor_set || !parameters.transient_resources || !parameters.history_resources) {
      return;
    }
    auto& history_resources = *parameters.history_resources;
    if (!history_resources.radiance_image || !history_resources.radiance_view || !history_resources.convergence_image ||
        !history_resources.convergence_view) {
      return;
    }
    ApplyGraphResourceBarriers(vk_command_buffer, context, RenderPassQueue::Graphics);
    ApplyRayCameraStorageDependencies(vk_command_buffer, render_texture->GetColorImage(), history_resources,
                                      VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT);
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
    RetainRayCameraOptionalOutputViews(*parameters.transient_resources, history_resources);
    parameters.transient_resources->RetainImage(render_texture->GetColorImage());
    parameters.transient_resources->RetainImageView(render_texture->GetColorImageView());
    const auto& output_descriptor_set = parameters.output_descriptor_set;
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
    VkDescriptorImageInfo fallback_color_image_info{};
    fallback_color_image_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
    fallback_color_image_info.imageView = render_texture->GetColorImageView()->GetVkImageView();
    const auto fallback_uint_image_info = PrepareRayCameraUintFallback(vk_command_buffer, history_resources);
    UpdateRayCameraOptionalOutputDescriptors(output_descriptor_set, history_resources, fallback_color_image_info,
                                             fallback_uint_image_info);

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
    push_constant.max_directional_light_size =
        ApplicationContext::Get().GetApplicationInfo().graphics_settings.max_directional_light_size;
    parameters.pipeline->PushConstant(vk_command_buffer, 0, push_constant);
    const auto gpu_timestamp = Platform::BeginGpuTimestampScope(vk_command_buffer, "Path Trace (RQ)");
    parameters.pipeline->Dispatch(vk_command_buffer,
                                  Platform::DivUp(render_texture->GetExtent().width, kRayQueryCameraWorkGroupSize),
                                  Platform::DivUp(render_texture->GetExtent().height, kRayQueryCameraWorkGroupSize), 1);
    Platform::EndGpuTimestampScope(vk_command_buffer, gpu_timestamp);
    history_resources.valid = true;
    ++history_resources.frame_id;
    parameters.transient_resources->RetainDescriptorSet(output_descriptor_set);
    ApplyGraphResourceReleaseBarriers(vk_command_buffer, context, RenderPassQueue::Graphics);
  });
}
