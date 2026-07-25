#include "RenderPasses/PostProcessingPass.hpp"

#include "Camera.hpp"
#include "Platform.hpp"
#include "PostProcessingStack.hpp"
#include "RenderPasses/RenderPassUtilities.hpp"

using namespace evo_engine;

RenderPassDescriptor AmbientOcclusionPass::CreateDescriptor() {
  return {RenderPassNames::ambient_occlusion,
          RenderPassQueue::Graphics,
          RenderPassScope::Camera,
          {{RenderResourceNames::camera_depth, RenderResourceUsage::Read, RenderResourceState::ShaderRead},
           {RenderResourceNames::camera_g_buffer, RenderResourceUsage::Read, RenderResourceState::ShaderRead},
           {RenderResourceNames::camera_ambient_occlusion, RenderResourceUsage::ReadWrite,
            RenderResourceState::StorageReadWrite},
           {RenderResourceNames::camera_ambient_occlusion_scratch, RenderResourceUsage::ReadWrite,
            RenderResourceState::StorageReadWrite}},
          {RenderPassNames::depth_pyramid}};
}

void AmbientOcclusionPass::Execute(const RenderGraphExecutionContext& context, const Parameters& parameters) {
  if (!parameters.camera) {
    return;
  }
  const auto post_processing_stack = parameters.camera->post_processing_stack_ref.Get<PostProcessingStack>();
  const auto* ambient_occlusion = context.GetResourceBinding(RenderResourceNames::camera_ambient_occlusion);
  const auto* scratch = context.GetResourceBinding(RenderResourceNames::camera_ambient_occlusion_scratch);
  if (!post_processing_stack || !post_processing_stack->enable_ambient_occlusion || !ambient_occlusion ||
      !ambient_occlusion->image || !scratch || !scratch->image) {
    return;
  }
  const auto ambient_occlusion_view = CreateGraphImageMipView(ambient_occlusion->image, 0);
  const auto scratch_view = CreateGraphImageMipView(scratch->image, 0);
  if (parameters.transient_resources) {
    parameters.transient_resources->RetainAsset(post_processing_stack);
    parameters.transient_resources->RetainImageView(ambient_occlusion_view);
    parameters.transient_resources->RetainImageView(scratch_view);
  }
  post_processing_stack->ProcessAmbientOcclusion(parameters.camera, ambient_occlusion_view, scratch_view,
                                                 [&](const VkCommandBuffer vk_command_buffer) {
                                                   ApplyGraphResourceBarriers(vk_command_buffer, context);
                                                 });
  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    ApplyGraphResourceReleaseBarriers(vk_command_buffer, context, RenderPassQueue::Graphics);
  });
}

RenderPassDescriptor PostProcessingPass::CreateDescriptor(const char* dependency) {
  return {RenderPassNames::post_processing,
          RenderPassQueue::Graphics,
          RenderPassScope::Camera,
          {{RenderResourceNames::camera_depth, RenderResourceUsage::Read, RenderResourceState::ShaderRead},
           {RenderResourceNames::camera_g_buffer, RenderResourceUsage::Read, RenderResourceState::ShaderRead},
           {RenderResourceNames::camera_motion_vectors, RenderResourceUsage::Read, RenderResourceState::ShaderRead},
           {RenderResourceNames::camera_color, RenderResourceUsage::ReadWrite, RenderResourceState::StorageReadWrite}},
          {dependency ? dependency : RenderPassNames::deferred_camera}};
}

RenderPassDescriptor PostProcessingPass::CreateRayTracingDescriptor(const char* dependency) {
  return {RenderPassNames::post_processing,
          RenderPassQueue::Graphics,
          RenderPassScope::Camera,
          {{RenderResourceNames::camera_color, RenderResourceUsage::ReadWrite, RenderResourceState::StorageReadWrite}},
          {dependency ? dependency : RenderPassNames::ray_tracing_camera}};
}

void PostProcessingPass::Execute(const RenderGraphExecutionContext& context, const Parameters& parameters) {
  if (parameters.immediate || !parameters.camera) {
    return;
  }
  if (const auto post_processing_stack = parameters.camera->post_processing_stack_ref.Get<PostProcessingStack>()) {
    if (parameters.transient_resources) {
      parameters.transient_resources->RetainAsset(post_processing_stack);
    }
    if (parameters.ray_camera) {
      post_processing_stack->ProcessRayCamera(parameters.camera, [&](const VkCommandBuffer vk_command_buffer) {
        ApplyGraphResourceBarriers(vk_command_buffer, context);
      });
      if (parameters.transient_resources) {
        parameters.camera->RetainPostProcessingResources(*parameters.transient_resources);
      }
      Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
        ApplyGraphResourceReleaseBarriers(vk_command_buffer, context, RenderPassQueue::Graphics);
      });
      return;
    }
    std::shared_ptr<ImageView> motion_vectors_view;
    if (const auto* motion_binding = context.GetResourceBinding(RenderResourceNames::camera_motion_vectors);
        motion_binding && motion_binding->image) {
      motion_vectors_view = CreateGraphImageMipView(motion_binding->image, 0);
      if (parameters.transient_resources) {
        parameters.transient_resources->RetainImageView(motion_vectors_view);
      }
    }
    post_processing_stack->Process(parameters.camera, std::move(motion_vectors_view),
                                   [&](const VkCommandBuffer vk_command_buffer) {
                                     ApplyGraphResourceBarriers(vk_command_buffer, context);
                                   });
    if (parameters.transient_resources) {
      parameters.camera->RetainPostProcessingResources(*parameters.transient_resources);
    }
    Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
      ApplyGraphResourceReleaseBarriers(vk_command_buffer, context, RenderPassQueue::Graphics);
    });
  }
}
