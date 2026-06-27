#include "RenderPasses/GaussianSplatPass.hpp"

#include "Camera.hpp"
#include "RenderInstanceStorage.hpp"
#include "RenderPasses/RenderPassUtilities.hpp"

using namespace evo_engine;

RenderPassDescriptor GaussianSplatPass::CreateDescriptor(const char* dependency) {
  return {
      RenderPassNames::gaussian_splat,
      RenderPassQueue::Graphics,
      RenderPassScope::Camera,
      {{RenderResourceNames::frame_render_instances, RenderResourceUsage::Read, RenderResourceState::ShaderRead},
       {RenderResourceNames::frame_per_frame_descriptor_set, RenderResourceUsage::Read, RenderResourceState::General},
       {RenderResourceNames::camera_depth, RenderResourceUsage::Read, RenderResourceState::DepthAttachment},
       {RenderResourceNames::camera_color, RenderResourceUsage::ReadWrite, RenderResourceState::ColorAttachment}},
      {dependency ? dependency : RenderPassNames::deferred_camera}};
}

void GaussianSplatPass::Execute(const RenderGraphExecutionContext& context, const Parameters& parameters) {
  if (!parameters.record_commands || !parameters.camera || !parameters.camera->GetRenderTexture() ||
      !parameters.render_instances || parameters.render_instances->total_gaussian_splats == 0u ||
      !parameters.render_instances->gaussian_splat_render_instances ||
      parameters.render_instances->gaussian_splat_render_instances->Empty()) {
    return;
  }

  parameters.record_commands([&](const VkCommandBuffer vk_command_buffer) {
    // The raster draw path is added next; this keeps the pass visible to the graph and validates resource barriers.
    ApplyGraphResourceBarriers(vk_command_buffer, context);
    ApplyGraphResourceReleaseBarriers(vk_command_buffer, context, RenderPassQueue::Graphics);
  });
}
