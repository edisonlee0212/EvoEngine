#include "RenderPasses/PostProcessingPass.hpp"

#include "Camera.hpp"
#include "Platform.hpp"
#include "PostProcessingStack.hpp"
#include "RenderPasses/RenderPassUtilities.hpp"

using namespace evo_engine;

RenderPassDescriptor PostProcessingPass::CreateDescriptor(const char* dependency) {
  return {RenderPassNames::post_processing,
          RenderPassQueue::Graphics,
          RenderPassScope::Camera,
          {{RenderResourceNames::camera_depth, RenderResourceUsage::Read, RenderResourceState::ShaderRead},
           {RenderResourceNames::camera_g_buffer, RenderResourceUsage::Read, RenderResourceState::ShaderRead},
           {RenderResourceNames::camera_color, RenderResourceUsage::ReadWrite, RenderResourceState::StorageReadWrite}},
          {dependency ? dependency : RenderPassNames::deferred_camera}};
}

void PostProcessingPass::Execute(const RenderGraphExecutionContext& context, const Parameters& parameters) {
  if (parameters.immediate || !parameters.camera) {
    return;
  }
  if (const auto post_processing_stack = parameters.camera->post_processing_stack_ref.Get<PostProcessingStack>()) {
    post_processing_stack->Process(parameters.camera, [&](const VkCommandBuffer vk_command_buffer) {
      ApplyGraphResourceBarriers(vk_command_buffer, context);
    });
    Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
      ApplyGraphResourceReleaseBarriers(vk_command_buffer, context, RenderPassQueue::Graphics);
    });
  }
}
