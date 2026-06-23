#include "RenderPasses/VolumetricCloudsPass.hpp"

#include "RenderPasses/RenderPassUtilities.hpp"

using namespace evo_engine;

namespace {
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
    ApplyGraphResourceReleaseBarriers(vk_command_buffer, context, RenderPassQueue::Graphics);
  });
}
