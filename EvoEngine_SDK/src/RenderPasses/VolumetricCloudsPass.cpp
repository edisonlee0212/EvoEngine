#include "RenderPasses/VolumetricCloudsPass.hpp"

#include "Camera.hpp"
#include "ComputePipeline.hpp"
#include "GraphicsResources.hpp"
#include "Platform.hpp"
#include "RenderPasses/RenderPassUtilities.hpp"
#include "RenderTexture.hpp"

using namespace evo_engine;

namespace {
VolumetricCloudsPushConstant CreatePushConstant(const VolumetricCloudsPass::Parameters& parameters) {
  VolumetricCloudSettings settings = parameters.settings;
  settings.ClampSettings();

  VolumetricCloudsPushConstant push_constant{};
  push_constant.altitude_coverage_density = {settings.bottom_altitude, settings.top_altitude, settings.coverage,
                                             settings.density};
  push_constant.wind_time = {settings.wind_direction.x, settings.wind_direction.y, settings.wind_speed,
                             parameters.time_seconds};
  const float max_distance =
      parameters.max_distance > 0.0f
          ? parameters.max_distance
          : (parameters.camera ? glm::max(parameters.camera->camera_settings.far_distance, 0.0f) : 0.0f);
  push_constant.lighting_phase_max_distance = {settings.lighting_intensity, settings.ambient_lighting_strength,
                                               settings.phase_anisotropy, max_distance};
  push_constant.camera_frame_steps = {parameters.camera_index, static_cast<int>(parameters.frame_index),
                                      settings.primary_step_count, settings.light_step_count};
  push_constant.flags = {settings.enabled ? 1 : 0, settings.debug_visualization ? 1 : 0, settings.debug_mode,
                         parameters.input_is_ray_hit_distance ? 1 : 0};
  return push_constant;
}

RenderPassDescriptor CreateDescriptor(const char* dependency, const char* depth_or_hit_distance_resource) {
  return {RenderPassNames::volumetric_clouds,
          RenderPassQueue::Graphics,
          RenderPassScope::Camera,
          {{depth_or_hit_distance_resource, RenderResourceUsage::Read, RenderResourceState::ShaderRead},
           {RenderResourceNames::camera_color, RenderResourceUsage::ReadWrite, RenderResourceState::StorageReadWrite},
           {RenderResourceNames::camera_volumetric_cloud_accumulation, RenderResourceUsage::Write,
            RenderResourceState::StorageReadWrite},
           {RenderResourceNames::camera_volumetric_cloud_transmittance, RenderResourceUsage::Write,
            RenderResourceState::StorageReadWrite}},
          {dependency}};
}
}  // namespace

RenderPassDescriptor VolumetricCloudsPass::CreateRasterDescriptor(const char* dependency) {
  return CreateDescriptor(dependency ? dependency : RenderPassNames::deferred_camera,
                          RenderResourceNames::camera_depth);
}

RenderPassDescriptor VolumetricCloudsPass::CreateRayTracingDescriptor(const char* dependency) {
  return CreateDescriptor(dependency ? dependency : RenderPassNames::ray_tracing_camera,
                          RenderResourceNames::camera_ray_hit_distance);
}

void VolumetricCloudsPass::Execute(const RenderGraphExecutionContext& context, const Parameters& parameters) {
  if (!parameters.record_commands) {
    return;
  }
  parameters.record_commands([&](const VkCommandBuffer vk_command_buffer) {
    ApplyGraphResourceBarriers(vk_command_buffer, context);
    if (parameters.pipeline && parameters.pipeline->Initialized() && parameters.composite_pipeline &&
        parameters.composite_pipeline->Initialized() && parameters.per_frame_descriptor_set &&
        parameters.descriptor_set_layout && parameters.transient_resources && parameters.camera &&
        parameters.camera->GetRenderTexture()) {
      const auto render_texture = parameters.camera->GetRenderTexture();
      const auto* depth_binding = context.GetResourceBinding(parameters.input_resource_name);
      const auto* color_binding = context.GetResourceBinding(RenderResourceNames::camera_color);
      const auto* accumulation_binding =
          context.GetResourceBinding(RenderResourceNames::camera_volumetric_cloud_accumulation);
      const auto* transmittance_binding =
          context.GetResourceBinding(RenderResourceNames::camera_volumetric_cloud_transmittance);
      if (depth_binding && depth_binding->image && color_binding && color_binding->image && accumulation_binding &&
          accumulation_binding->image && transmittance_binding && transmittance_binding->image) {
        std::shared_ptr<ImageView> ray_hit_distance_view;
        if (parameters.input_is_ray_hit_distance) {
          ray_hit_distance_view = CreateGraphImageMipView(depth_binding->image, 0);
          if (!ray_hit_distance_view) {
            ApplyGraphResourceReleaseBarriers(vk_command_buffer, context, RenderPassQueue::Graphics);
            return;
          }
          parameters.transient_resources->RetainImageView(ray_hit_distance_view);
        }
        const auto accumulation_view = CreateGraphImageMipView(accumulation_binding->image, 0);
        const auto transmittance_view = CreateGraphImageMipView(transmittance_binding->image, 0);
        if (accumulation_view && transmittance_view) {
          parameters.transient_resources->RetainImageView(accumulation_view);
          parameters.transient_resources->RetainImageView(transmittance_view);

          const auto descriptor_set = std::make_shared<DescriptorSet>(parameters.descriptor_set_layout);
          VkDescriptorImageInfo image_info{};
          image_info.imageLayout = depth_binding->image->GetLayout();
          image_info.imageView = parameters.input_is_ray_hit_distance
                                     ? ray_hit_distance_view->GetVkImageView()
                                     : render_texture->GetDepthImageView()->GetVkImageView();
          image_info.sampler = render_texture->GetDepthSampler()->GetVkSampler();
          descriptor_set->UpdateImageDescriptorBinding(0, image_info);

          image_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
          image_info.imageView = render_texture->GetColorImageView()->GetVkImageView();
          image_info.sampler = VK_NULL_HANDLE;
          descriptor_set->UpdateImageDescriptorBinding(1, image_info);
          image_info.imageView = accumulation_view->GetVkImageView();
          descriptor_set->UpdateImageDescriptorBinding(2, image_info);
          image_info.imageView = transmittance_view->GetVkImageView();
          descriptor_set->UpdateImageDescriptorBinding(3, image_info);

          image_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
          image_info.imageView = accumulation_view->GetVkImageView();
          image_info.sampler = render_texture->GetColorSampler()->GetVkSampler();
          descriptor_set->UpdateImageDescriptorBinding(4, image_info);
          image_info.imageView = transmittance_view->GetVkImageView();
          descriptor_set->UpdateImageDescriptorBinding(5, image_info);

          const auto push_constant = CreatePushConstant(parameters);
          const auto cloud_extent = accumulation_binding->image->GetExtent();
          parameters.pipeline->Bind(vk_command_buffer);
          parameters.pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                                 parameters.per_frame_descriptor_set->GetVkDescriptorSet());
          parameters.pipeline->BindDescriptorSet(vk_command_buffer, 1, descriptor_set->GetVkDescriptorSet());
          parameters.pipeline->PushConstant(vk_command_buffer, 0, push_constant);
          parameters.pipeline->Dispatch(vk_command_buffer, Platform::DivUp(cloud_extent.width, 16),
                                        Platform::DivUp(cloud_extent.height, 16));
          Platform::EverythingBarrier(vk_command_buffer);

          const auto extent = color_binding->image->GetExtent();
          parameters.composite_pipeline->Bind(vk_command_buffer);
          parameters.composite_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                                           parameters.per_frame_descriptor_set->GetVkDescriptorSet());
          parameters.composite_pipeline->BindDescriptorSet(vk_command_buffer, 1, descriptor_set->GetVkDescriptorSet());
          parameters.composite_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
          parameters.composite_pipeline->Dispatch(vk_command_buffer, Platform::DivUp(extent.width, 16),
                                                  Platform::DivUp(extent.height, 16));
          parameters.transient_resources->RetainDescriptorSet(descriptor_set);
          Platform::EverythingBarrier(vk_command_buffer);
        }
      }
    }
    ApplyGraphResourceReleaseBarriers(vk_command_buffer, context, RenderPassQueue::Graphics);
  });
}
