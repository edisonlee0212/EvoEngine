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
#include <string_view>
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
  const std::array images{color, history.radiance_image};
  std::array<VkImageMemoryBarrier2, 2> barriers{};
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
  VkMemoryBarrier2 buffer_barrier{};
  if (history.integrator == CameraSettings::RayIntegrator::RestirPtTemporalOnly ||
      history.integrator == CameraSettings::RayIntegrator::RestirPtSpatialOnly ||
      history.integrator == CameraSettings::RayIntegrator::RestirPtEnhanced) {
    // Order shared scratch writes and two-slot history reads across main-queue submissions.
    buffer_barrier.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER_2;
    buffer_barrier.srcStageMask = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT;
    buffer_barrier.srcAccessMask = VK_ACCESS_2_MEMORY_READ_BIT | VK_ACCESS_2_MEMORY_WRITE_BIT;
    buffer_barrier.dstStageMask = shader_stage;
    buffer_barrier.dstAccessMask = VK_ACCESS_2_SHADER_READ_BIT | VK_ACCESS_2_SHADER_WRITE_BIT;
    dependency.memoryBarrierCount = 1;
    dependency.pMemoryBarriers = &buffer_barrier;
  }
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
  descriptor.profiler_group = RenderPassProfilerGroup::RayTracing;
  descriptor.profiler_display_name = "Path Trace (RTX)";
  return descriptor;
}

RenderPassDescriptor RayQueryCameraPass::CreateDescriptor(const CameraSettings::RayOutputSettings outputs,
                                                          const char* pass_name) {
  RenderPassDescriptor descriptor{
      pass_name,
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
  descriptor.profiler_group = RenderPassProfilerGroup::RayTracing;
  descriptor.profiler_display_name =
      std::string_view(pass_name) == RenderPassNames::ray_query_camera ? "Path Trace (RQ)" : pass_name;
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
    if (!history_resources.radiance_image || !history_resources.radiance_view) {
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
    VkDescriptorImageInfo fallback_color_image_info{};
    fallback_color_image_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
    fallback_color_image_info.imageView = render_texture->GetColorImageView()->GetVkImageView();
    const auto fallback_uint_image_info = PrepareRayCameraUintFallback(vk_command_buffer, history_resources);
    UpdateRayCameraOptionalOutputDescriptors(output_descriptor_set, history_resources, fallback_color_image_info,
                                             fallback_uint_image_info);

#ifdef EVOENGINE_ENABLE_NRD
    if (parameters.write_nrd_signals) {
      if (!parameters.primary_surface_buffer || !history_resources.restir_nrd_diffuse_signal_view ||
          !history_resources.restir_nrd_specular_signal_view || !history_resources.restir_nrd_excluded_signal_view ||
          !history_resources.restir_nrd_diffuse_factor_view || !history_resources.restir_nrd_specular_factor_view) {
        ApplyGraphResourceReleaseBarriers(vk_command_buffer, context, RenderPassQueue::RayTracing);
        return;
      }
      output_descriptor_set->UpdateBufferDescriptorBinding(kRayCameraRestirPrimarySurfaceBinding,
                                                           parameters.primary_surface_buffer);
      parameters.transient_resources->RetainBuffer(parameters.primary_surface_buffer);
      const std::array bindings{
          std::pair{kRayCameraNrdDiffuseSignalBinding, history_resources.restir_nrd_diffuse_signal_view},
          std::pair{kRayCameraNrdSpecularSignalBinding, history_resources.restir_nrd_specular_signal_view},
          std::pair{kRayCameraNrdExcludedSignalBinding, history_resources.restir_nrd_excluded_signal_view},
          std::pair{kRayCameraNrdDiffuseFactorBinding, history_resources.restir_nrd_diffuse_factor_view},
          std::pair{kRayCameraNrdSpecularFactorBinding, history_resources.restir_nrd_specular_factor_view},
      };
      for (const auto& [binding, view] : bindings) {
        image_info.imageView = view->GetVkImageView();
        output_descriptor_set->UpdateImageDescriptorBinding(binding, image_info);
        parameters.transient_resources->RetainImageView(view);
      }
      parameters.transient_resources->RetainImage(history_resources.restir_nrd_diffuse_signal_image);
      parameters.transient_resources->RetainImage(history_resources.restir_nrd_specular_signal_image);
      parameters.transient_resources->RetainImage(history_resources.restir_nrd_excluded_signal_image);
      parameters.transient_resources->RetainImage(history_resources.restir_nrd_diffuse_factor_image);
      parameters.transient_resources->RetainImage(history_resources.restir_nrd_specular_factor_image);
    }
#endif

    parameters.pipeline->Bind(vk_command_buffer);
    parameters.pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                           parameters.per_frame_descriptor_set->GetVkDescriptorSet());
    parameters.pipeline->BindDescriptorSet(vk_command_buffer, 1,
                                           parameters.ray_tracing_descriptor_set->GetVkDescriptorSet());
    parameters.pipeline->BindDescriptorSet(vk_command_buffer, 2, output_descriptor_set->GetVkDescriptorSet());
    RayTracingCameraPushConstant push_constant;
    push_constant.camera_index = parameters.camera_index;
    push_constant.seed_frame_id = static_cast<uint32_t>(Platform::GetFrameCount());
    const uint32_t radiance_frame_id = history_resources.valid ? history_resources.frame_id : 0u;
    const bool restir_pt = history_resources.technique == RayCameraHistoryTechnique::RestirPt;
    push_constant.nrd_accumulated_input = !restir_pt && parameters.camera->camera_settings.accumulate_samples &&
                                          parameters.camera->camera_settings.nrd_denoising;
    if (!restir_pt) {
      push_constant.frame_id = static_cast<uint32_t>(Platform::GetFrameCount());
    } else if (parameters.camera->camera_settings.nrd_denoising && history_resources.restir_nrd_denoiser) {
      push_constant.frame_id = history_resources.restir_nrd_history_frame_count;
    } else if (parameters.camera->camera_settings.ray_integrator == CameraSettings::RayIntegrator::RestirPtEnhanced ||
               parameters.camera->camera_settings.ray_integrator ==
                   CameraSettings::RayIntegrator::RestirPtTemporalOnly) {
      push_constant.frame_id = history_resources.restir_history_frame_id;
    } else {
      push_constant.frame_id = parameters.camera->camera_settings.accumulate_samples
                                   ? radiance_frame_id
                                   : static_cast<uint32_t>(Platform::GetFrameCount());
    }
    push_constant.frame_samples = static_cast<uint32_t>(std::max(parameters.camera->camera_settings.sample_size, 1));
    push_constant.total_samples =
        parameters.camera->camera_settings.accumulate_samples ? radiance_frame_id * push_constant.frame_samples : 0u;
    push_constant.shader_execution_reordering =
        (!Platform::RayTracingLinearSweptSpheresEnabled() &&
         Camera::ResolveShaderExecutionReorderingEnabled(
             parameters.camera->camera_settings.shader_execution_reordering_mode))
            ? 1u
            : 0u;
    push_constant.max_directional_light_size =
        ApplicationContext::Get().GetApplicationInfo().graphics_settings.max_directional_light_size;
    push_constant.nrd_reblur = parameters.camera->camera_settings.nrd_reblur;
    push_constant.nrd_path_tracing = !restir_pt;
    parameters.pipeline->PushConstant(vk_command_buffer, 0, push_constant);
    {
      const RenderPassGpuTimestampScope gpu_timestamp(vk_command_buffer, context,
                                                      parameters.camera->GetHandle().GetValue());
      parameters.pipeline->Trace(vk_command_buffer, render_texture->GetExtent().width,
                                 render_texture->GetExtent().height, 1);
    }
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
    if (!history_resources.radiance_image || !history_resources.radiance_view) {
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
    VkDescriptorImageInfo fallback_color_image_info{};
    fallback_color_image_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
    fallback_color_image_info.imageView = render_texture->GetColorImageView()->GetVkImageView();
    const auto fallback_uint_image_info = PrepareRayCameraUintFallback(vk_command_buffer, history_resources);
    UpdateRayCameraOptionalOutputDescriptors(output_descriptor_set, history_resources, fallback_color_image_info,
                                             fallback_uint_image_info);
#ifdef EVOENGINE_ENABLE_NRD
    if (parameters.bind_nrd_guides) {
      if (!history_resources.restir_nrd_view_z_view || !history_resources.restir_nrd_motion_view ||
          !history_resources.restir_nrd_normal_roughness_view) {
        ApplyGraphResourceReleaseBarriers(vk_command_buffer, context, RenderPassQueue::Graphics);
        return;
      }
      image_info.imageView = history_resources.restir_nrd_view_z_view->GetVkImageView();
      output_descriptor_set->UpdateImageDescriptorBinding(kRayCameraNrdViewZBinding, image_info);
      image_info.imageView = history_resources.restir_nrd_motion_view->GetVkImageView();
      output_descriptor_set->UpdateImageDescriptorBinding(kRayCameraNrdMotionBinding, image_info);
      parameters.transient_resources->RetainImage(history_resources.restir_nrd_view_z_image);
      parameters.transient_resources->RetainImageView(history_resources.restir_nrd_view_z_view);
      parameters.transient_resources->RetainImage(history_resources.restir_nrd_motion_image);
      parameters.transient_resources->RetainImageView(history_resources.restir_nrd_motion_view);
      image_info.imageView = history_resources.restir_nrd_normal_roughness_view->GetVkImageView();
      output_descriptor_set->UpdateImageDescriptorBinding(kRayCameraNrdNormalRoughnessBinding, image_info);
      parameters.transient_resources->RetainImage(history_resources.restir_nrd_normal_roughness_image);
      parameters.transient_resources->RetainImageView(history_resources.restir_nrd_normal_roughness_view);
    }
    if (parameters.write_nrd_signals) {
      if (!history_resources.restir_nrd_diffuse_signal_view || !history_resources.restir_nrd_specular_signal_view ||
          !history_resources.restir_nrd_excluded_signal_view) {
        ApplyGraphResourceReleaseBarriers(vk_command_buffer, context, RenderPassQueue::Graphics);
        return;
      }
      image_info.imageView = history_resources.restir_nrd_diffuse_signal_view->GetVkImageView();
      output_descriptor_set->UpdateImageDescriptorBinding(kRayCameraNrdDiffuseSignalBinding, image_info);
      image_info.imageView = history_resources.restir_nrd_specular_signal_view->GetVkImageView();
      output_descriptor_set->UpdateImageDescriptorBinding(kRayCameraNrdSpecularSignalBinding, image_info);
      image_info.imageView = history_resources.restir_nrd_excluded_signal_view->GetVkImageView();
      output_descriptor_set->UpdateImageDescriptorBinding(kRayCameraNrdExcludedSignalBinding, image_info);
      parameters.transient_resources->RetainImage(history_resources.restir_nrd_diffuse_signal_image);
      parameters.transient_resources->RetainImageView(history_resources.restir_nrd_diffuse_signal_view);
      parameters.transient_resources->RetainImage(history_resources.restir_nrd_specular_signal_image);
      parameters.transient_resources->RetainImageView(history_resources.restir_nrd_specular_signal_view);
      parameters.transient_resources->RetainImage(history_resources.restir_nrd_excluded_signal_image);
      parameters.transient_resources->RetainImageView(history_resources.restir_nrd_excluded_signal_view);
    }
    if (parameters.read_nrd_denoised) {
      if (!history_resources.restir_nrd_denoised_diffuse_view || !history_resources.restir_nrd_denoised_specular_view) {
        ApplyGraphResourceReleaseBarriers(vk_command_buffer, context, RenderPassQueue::Graphics);
        return;
      }
      image_info.imageView = history_resources.restir_nrd_denoised_diffuse_view->GetVkImageView();
      output_descriptor_set->UpdateImageDescriptorBinding(kRayCameraNrdDenoisedDiffuseBinding, image_info);
      image_info.imageView = history_resources.restir_nrd_denoised_specular_view->GetVkImageView();
      output_descriptor_set->UpdateImageDescriptorBinding(kRayCameraNrdDenoisedSpecularBinding, image_info);
      parameters.transient_resources->RetainImage(history_resources.restir_nrd_denoised_diffuse_image);
      parameters.transient_resources->RetainImageView(history_resources.restir_nrd_denoised_diffuse_view);
      parameters.transient_resources->RetainImage(history_resources.restir_nrd_denoised_specular_image);
      parameters.transient_resources->RetainImageView(history_resources.restir_nrd_denoised_specular_view);
    }
    if (parameters.bind_nrd_temporal_aa) {
      const uint32_t slot = parameters.nrd_taa_write_slot;
      const auto bind = [&](uint32_t binding, const std::shared_ptr<Image>& image,
                            const std::shared_ptr<ImageView>& view) {
        image_info.imageView = view->GetVkImageView();
        output_descriptor_set->UpdateImageDescriptorBinding(binding, image_info);
        parameters.transient_resources->RetainImage(image);
        parameters.transient_resources->RetainImageView(view);
      };
      bind(kRayCameraNrdTaaPreviousColorBinding, history_resources.nrd_taa_color_images[slot ^ 1u],
           history_resources.nrd_taa_color_views[slot ^ 1u]);
      bind(kRayCameraNrdTaaPreviousGeometryBinding, history_resources.nrd_taa_geometry_images[slot ^ 1u],
           history_resources.nrd_taa_geometry_views[slot ^ 1u]);
      bind(kRayCameraNrdTaaNextColorBinding, history_resources.nrd_taa_color_images[slot],
           history_resources.nrd_taa_color_views[slot]);
      bind(kRayCameraNrdTaaNextGeometryBinding, history_resources.nrd_taa_geometry_images[slot],
           history_resources.nrd_taa_geometry_views[slot]);
    }
    if (parameters.bind_nrd_lobe_hit_distance) {
      if (!history_resources.restir_nrd_lobe_hit_distance_view) {
        ApplyGraphResourceReleaseBarriers(vk_command_buffer, context, RenderPassQueue::Graphics);
        return;
      }
      image_info.imageView = history_resources.restir_nrd_lobe_hit_distance_view->GetVkImageView();
      output_descriptor_set->UpdateImageDescriptorBinding(kRayCameraNrdLobeHitDistanceBinding, image_info);
      parameters.transient_resources->RetainImage(history_resources.restir_nrd_lobe_hit_distance_image);
      parameters.transient_resources->RetainImageView(history_resources.restir_nrd_lobe_hit_distance_view);
    }
    if (parameters.bind_nrd_material_factors) {
      if (!history_resources.restir_nrd_diffuse_factor_view || !history_resources.restir_nrd_specular_factor_view) {
        ApplyGraphResourceReleaseBarriers(vk_command_buffer, context, RenderPassQueue::Graphics);
        return;
      }
      image_info.imageView = history_resources.restir_nrd_diffuse_factor_view->GetVkImageView();
      output_descriptor_set->UpdateImageDescriptorBinding(kRayCameraNrdDiffuseFactorBinding, image_info);
      image_info.imageView = history_resources.restir_nrd_specular_factor_view->GetVkImageView();
      output_descriptor_set->UpdateImageDescriptorBinding(kRayCameraNrdSpecularFactorBinding, image_info);
      parameters.transient_resources->RetainImage(history_resources.restir_nrd_diffuse_factor_image);
      parameters.transient_resources->RetainImageView(history_resources.restir_nrd_diffuse_factor_view);
      parameters.transient_resources->RetainImage(history_resources.restir_nrd_specular_factor_image);
      parameters.transient_resources->RetainImageView(history_resources.restir_nrd_specular_factor_view);
    }
    if (parameters.bind_nrd_accumulated_input) {
      if (!history_resources.nrd_accumulated_diffuse_view || !history_resources.nrd_accumulated_specular_view) {
        ApplyGraphResourceReleaseBarriers(vk_command_buffer, context, RenderPassQueue::Graphics);
        return;
      }
      image_info.imageView = history_resources.nrd_accumulated_diffuse_view->GetVkImageView();
      output_descriptor_set->UpdateImageDescriptorBinding(kRayCameraNrdAccumulatedDiffuseBinding, image_info);
      image_info.imageView = history_resources.nrd_accumulated_specular_view->GetVkImageView();
      output_descriptor_set->UpdateImageDescriptorBinding(kRayCameraNrdAccumulatedSpecularBinding, image_info);
      parameters.transient_resources->RetainImage(history_resources.nrd_accumulated_diffuse_image);
      parameters.transient_resources->RetainImageView(history_resources.nrd_accumulated_diffuse_view);
      parameters.transient_resources->RetainImage(history_resources.nrd_accumulated_specular_image);
      parameters.transient_resources->RetainImageView(history_resources.nrd_accumulated_specular_view);
    }
#endif
    if (parameters.candidate_buffer) {
      output_descriptor_set->UpdateBufferDescriptorBinding(kRayCameraRestirCandidateBinding,
                                                           parameters.candidate_buffer);
      parameters.transient_resources->RetainBuffer(parameters.candidate_buffer);
    }
    if (parameters.primary_surface_buffer) {
      output_descriptor_set->UpdateBufferDescriptorBinding(kRayCameraRestirPrimarySurfaceBinding,
                                                           parameters.primary_surface_buffer);
      parameters.transient_resources->RetainBuffer(parameters.primary_surface_buffer);
    }
    if (parameters.resolved_buffer) {
      output_descriptor_set->UpdateBufferDescriptorBinding(kRayCameraRestirResolvedBinding, parameters.resolved_buffer);
      parameters.transient_resources->RetainBuffer(parameters.resolved_buffer);
    }
    if (parameters.shift_buffer) {
      output_descriptor_set->UpdateBufferDescriptorBinding(kRayCameraRestirShiftBinding, parameters.shift_buffer);
      parameters.transient_resources->RetainBuffer(parameters.shift_buffer);
    }
    if (parameters.generated_buffer) {
      output_descriptor_set->UpdateBufferDescriptorBinding(kRayCameraRestirGeneratedBinding,
                                                           parameters.generated_buffer);
      parameters.transient_resources->RetainBuffer(parameters.generated_buffer);
    }
    const auto bind_restir_buffer = [&](const uint32_t binding, const std::shared_ptr<Buffer>& buffer) {
      if (buffer) {
        output_descriptor_set->UpdateBufferDescriptorBinding(binding, buffer);
        parameters.transient_resources->RetainBuffer(buffer);
      }
    };
    bind_restir_buffer(kRayCameraRestirPreviousHistoryBinding, parameters.previous_history_buffer);
    bind_restir_buffer(kRayCameraRestirPreviousSurfaceBinding, parameters.previous_surface_buffer);
    bind_restir_buffer(kRayCameraRestirTemporalForwardBinding, parameters.temporal_forward_buffer);
    // SSS references this descriptor statically even when adaptive duplication is disabled.
    bind_restir_buffer(kRayCameraRestirDuplicationBinding, parameters.duplication_buffer
                                                               ? parameters.duplication_buffer
                                                               : parameters.previous_history_buffer);
    bind_restir_buffer(kRayCameraRestirNextHistoryBinding, parameters.next_history_buffer);
    bind_restir_buffer(kRayCameraRestirPairingBinding, parameters.pairing_buffer);
    bind_restir_buffer(kRayCameraRestirPreviousInstanceBinding, parameters.previous_instance_buffer);
    bind_restir_buffer(kRayCameraRestirNrdAccumulationBinding, parameters.nrd_sample_accumulation_buffer);

    parameters.pipeline->Bind(vk_command_buffer);
    parameters.pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                           parameters.per_frame_descriptor_set->GetVkDescriptorSet());
    parameters.pipeline->BindDescriptorSet(vk_command_buffer, 1,
                                           parameters.ray_tracing_descriptor_set->GetVkDescriptorSet());
    parameters.pipeline->BindDescriptorSet(vk_command_buffer, 2, output_descriptor_set->GetVkDescriptorSet());
    RayTracingCameraPushConstant push_constant;
    push_constant.camera_index = parameters.camera_index;
    const uint32_t radiance_frame_id = history_resources.valid ? history_resources.frame_id : 0u;
    const bool restir_pt = history_resources.technique == RayCameraHistoryTechnique::RestirPt;
    push_constant.nrd_accumulated_input = !restir_pt && parameters.camera->camera_settings.accumulate_samples &&
                                          parameters.camera->camera_settings.nrd_denoising;
    const uint32_t samples_per_frame =
        static_cast<uint32_t>(std::max(parameters.camera->camera_settings.sample_size, 1));
    const uint32_t render_frame_id = static_cast<uint32_t>(Platform::GetFrameCount());
    push_constant.seed_frame_id =
        restir_pt ? render_frame_id * samples_per_frame + parameters.restir_sample_index : render_frame_id;
    uint32_t frame_id;
    if (!restir_pt) {
      frame_id = static_cast<uint32_t>(Platform::GetFrameCount());
    } else if (parameters.camera->camera_settings.nrd_denoising && history_resources.restir_nrd_denoiser) {
      frame_id = history_resources.restir_nrd_history_frame_count;
    } else if (parameters.camera->camera_settings.ray_integrator == CameraSettings::RayIntegrator::RestirPtEnhanced ||
               parameters.camera->camera_settings.ray_integrator ==
                   CameraSettings::RayIntegrator::RestirPtTemporalOnly) {
      frame_id = history_resources.restir_history_frame_id;
    } else {
      frame_id = parameters.camera->camera_settings.accumulate_samples
                     ? radiance_frame_id
                     : static_cast<uint32_t>(Platform::GetFrameCount());
    }
    push_constant.frame_id = restir_pt ? frame_id * samples_per_frame + parameters.restir_sample_index : frame_id;
    push_constant.frame_samples = restir_pt ? 1u : samples_per_frame;
    push_constant.total_samples =
        restir_pt
            ? (parameters.camera->camera_settings.accumulate_samples ? radiance_frame_id * samples_per_frame : 0u) +
                  parameters.restir_sample_index
        : parameters.camera->camera_settings.accumulate_samples ? radiance_frame_id * samples_per_frame
                                                                : 0u;
    push_constant.max_directional_light_size =
        ApplicationContext::Get().GetApplicationInfo().graphics_settings.max_directional_light_size;
    push_constant.nrd_reblur = parameters.camera->camera_settings.nrd_reblur;
    push_constant.nrd_path_tracing = !restir_pt;
    if (parameters.bind_nrd_accumulated_input && !history_resources.restir_nrd_history_valid)
      push_constant.total_samples = 0u;
    if (parameters.bind_nrd_temporal_aa)
      push_constant.frame_id =
          history_resources.nrd_taa_history_valid && history_resources.restir_nrd_history_valid ? 1u : 0u;
    if (parameters.sss_temporal) {
      // RayQuery has no SER; its SSS pipeline uses that push word for history validity.
      push_constant.shader_execution_reordering = parameters.sss_history_valid ? 1u : 0u;
      push_constant.frame_id = history_resources.restir_sss_frame_id;
    }
    parameters.pipeline->PushConstant(vk_command_buffer, 0, push_constant);
    {
      const RenderPassGpuTimestampScope gpu_timestamp(vk_command_buffer, context,
                                                      parameters.camera->GetHandle().GetValue());
      parameters.pipeline->Dispatch(
          vk_command_buffer, Platform::DivUp(render_texture->GetExtent().width, kRayQueryCameraWorkGroupSize),
          Platform::DivUp(render_texture->GetExtent().height, kRayQueryCameraWorkGroupSize), 1);
    }
    if (parameters.commit_history) {
      history_resources.valid = true;
      ++history_resources.frame_id;
    }
    if (parameters.commit_nrd_temporal_aa) {
      history_resources.nrd_taa_history_valid = history_resources.restir_nrd_history_valid;
      history_resources.nrd_taa_write_slot = parameters.nrd_taa_write_slot ^ 1u;
    }
    if (parameters.commit_restir_history) {
      history_resources.restir_history_valid = true;
      ++history_resources.restir_history_frame_id;
    }
    if (parameters.sss_temporal && parameters.commit_sss_history) {
      history_resources.restir_sss_history_valid = true;
      ++history_resources.restir_sss_frame_id;
      if (!parameters.sss_spatial_resolve)
        history_resources.restir_sss_write_slot ^= 1u;
    }
    parameters.transient_resources->RetainDescriptorSet(output_descriptor_set);
    ApplyGraphResourceReleaseBarriers(vk_command_buffer, context, RenderPassQueue::Graphics);
  });
}
