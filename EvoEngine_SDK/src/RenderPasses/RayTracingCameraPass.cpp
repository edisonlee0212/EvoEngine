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
#include <unordered_map>

using namespace evo_engine;

namespace {
struct RayTracingCameraHistoryResources {
  VkExtent3D extent{};
  std::shared_ptr<Image> radiance_image;
  std::shared_ptr<ImageView> radiance_view;
  std::shared_ptr<Image> convergence_image;
  std::shared_ptr<ImageView> convergence_view;
  bool valid = false;
};

std::shared_ptr<Image> CreateRayTracingCameraHistoryImage(const VkExtent3D extent) {
  VkImageCreateInfo image_info{};
  image_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
  image_info.imageType = VK_IMAGE_TYPE_2D;
  image_info.extent = extent;
  image_info.mipLevels = 1;
  image_info.arrayLayers = 1;
  image_info.format = Platform::Constants::render_texture_color;
  image_info.tiling = VK_IMAGE_TILING_OPTIMAL;
  image_info.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
  image_info.usage = VK_IMAGE_USAGE_STORAGE_BIT;
  image_info.samples = VK_SAMPLE_COUNT_1_BIT;
  image_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  return std::make_shared<Image>(image_info);
}

uint64_t MakeRayCameraHistoryKey(const uint64_t camera_handle, const uint32_t technique_index) {
  return (camera_handle << 1u) ^ static_cast<uint64_t>(technique_index & 1u);
}

RayTracingCameraHistoryResources& GetRayTracingCameraHistoryResources(const uint64_t camera_handle,
                                                                      const VkExtent3D extent,
                                                                      const uint32_t technique_index) {
  static std::unordered_map<uint64_t, RayTracingCameraHistoryResources> history_resources;
  auto& resources = history_resources[MakeRayCameraHistoryKey(camera_handle, technique_index)];
  if (resources.extent.width == extent.width && resources.extent.height == extent.height &&
      resources.extent.depth == extent.depth && resources.radiance_image && resources.radiance_view &&
      resources.convergence_image && resources.convergence_view) {
    return resources;
  }

  resources = {};
  resources.extent = extent;
  resources.radiance_image = CreateRayTracingCameraHistoryImage(extent);
  resources.radiance_view = CreateGraphImageMipView(resources.radiance_image, 0);
  resources.convergence_image = CreateRayTracingCameraHistoryImage(extent);
  resources.convergence_view = CreateGraphImageMipView(resources.convergence_image, 0);
  return resources;
}
}  // namespace

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
    if (!render_texture || !parameters.pipeline || !parameters.per_frame_descriptor_set ||
        !parameters.ray_tracing_descriptor_set || !parameters.output_descriptor_set_layout ||
        !parameters.transient_resources) {
      return;
    }
    auto& history_resources =
        GetRayTracingCameraHistoryResources(parameters.camera->GetHandle().GetValue(), render_texture->GetExtent(), 0u);
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
    push_constant.frame_id = history_resources.valid ? parameters.frame_id : 0u;
    push_constant.frame_samples = static_cast<uint32_t>(std::max(parameters.camera->camera_settings.sample_size, 1));
    push_constant.total_samples = push_constant.frame_id * push_constant.frame_samples;
    push_constant.shader_execution_reordering = Camera::ResolveShaderExecutionReorderingEnabled(
                                                    parameters.camera->camera_settings.shader_execution_reordering_mode)
                                                    ? 1u
                                                    : 0u;
    parameters.pipeline->PushConstant(vk_command_buffer, 0, push_constant);
    parameters.pipeline->Trace(vk_command_buffer, render_texture->GetExtent().width, render_texture->GetExtent().height,
                               1);
    history_resources.valid = true;
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
    if (!render_texture || !parameters.pipeline || !parameters.per_frame_descriptor_set ||
        !parameters.ray_tracing_descriptor_set || !parameters.output_descriptor_set_layout ||
        !parameters.transient_resources) {
      return;
    }
    auto& history_resources =
        GetRayTracingCameraHistoryResources(parameters.camera->GetHandle().GetValue(), render_texture->GetExtent(), 1u);
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
    push_constant.frame_id = history_resources.valid ? parameters.frame_id : 0u;
    push_constant.frame_samples = static_cast<uint32_t>(std::max(parameters.camera->camera_settings.sample_size, 1));
    push_constant.total_samples = push_constant.frame_id * push_constant.frame_samples;
    parameters.pipeline->PushConstant(vk_command_buffer, 0, push_constant);
    parameters.pipeline->Dispatch(vk_command_buffer, Platform::DivUp(render_texture->GetExtent().width, 8),
                                  Platform::DivUp(render_texture->GetExtent().height, 8), 1);
    history_resources.valid = true;
    parameters.transient_resources->RetainDescriptorSet(output_descriptor_set);
    Platform::EverythingBarrier(vk_command_buffer);
    ApplyGraphResourceReleaseBarriers(vk_command_buffer, context, RenderPassQueue::Graphics);
  });
}
