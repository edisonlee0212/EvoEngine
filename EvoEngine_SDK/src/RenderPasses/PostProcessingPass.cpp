#include "RenderPasses/PostProcessingPass.hpp"

#include "Camera.hpp"
#include "Platform.hpp"
#include "PostProcessingStack.hpp"
#include "RenderPasses/RenderPassUtilities.hpp"

using namespace evo_engine;

namespace {
void RetainPostProcessingRenderTextures(RenderGraphTransientResourceStore* transient_resources,
                                        const std::shared_ptr<PostProcessingStack>& post_processing_stack,
                                        const std::shared_ptr<Camera>& camera) {
  if (!transient_resources) {
    return;
  }
  transient_resources->RetainRenderTextureResources(post_processing_stack->source_color_texture);
  transient_resources->RetainRenderTextureResources(post_processing_stack->result_texture);
  transient_resources->RetainRenderTextureResources(post_processing_stack->swap_texture);
  if (post_processing_stack->anti_aliasing) {
    post_processing_stack->anti_aliasing->RetainRuntimeResources(camera->GetHandle().GetValue(), *transient_resources);
  }
}
}  // namespace

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
    post_processing_stack->motion_vectors_image_view = {};
    if (parameters.ray_camera) {
      post_processing_stack->ProcessRayCamera(parameters.camera, [&](const VkCommandBuffer vk_command_buffer) {
        ApplyGraphResourceBarriers(vk_command_buffer, context);
      });
      RetainPostProcessingRenderTextures(parameters.transient_resources, post_processing_stack, parameters.camera);
      Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
        ApplyGraphResourceReleaseBarriers(vk_command_buffer, context, RenderPassQueue::Graphics);
      });
      return;
    }
    if (parameters.transient_resources) {
      if (const auto* motion_binding = context.GetResourceBinding(RenderResourceNames::camera_motion_vectors);
          motion_binding && motion_binding->image) {
        auto motion_vectors_view = CreateGraphImageMipView(motion_binding->image, 0);
        parameters.transient_resources->RetainImageView(motion_vectors_view);
        post_processing_stack->motion_vectors_image_view = std::move(motion_vectors_view);
      }
    }
    post_processing_stack->Process(parameters.camera, [&](const VkCommandBuffer vk_command_buffer) {
      ApplyGraphResourceBarriers(vk_command_buffer, context);
    });
    RetainPostProcessingRenderTextures(parameters.transient_resources, post_processing_stack, parameters.camera);
    Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
      ApplyGraphResourceReleaseBarriers(vk_command_buffer, context, RenderPassQueue::Graphics);
    });
  }
}
