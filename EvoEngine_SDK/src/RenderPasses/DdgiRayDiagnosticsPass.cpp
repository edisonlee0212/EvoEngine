#include "RenderPasses/DdgiRayDiagnosticsPass.hpp"

#include "GraphicsResources.hpp"
#include "Platform.hpp"
#include "RayTracingPipeline.hpp"
#include "RenderPasses/DdgiPassUtilities.hpp"
#include "RenderPasses/RenderPassUtilities.hpp"

#include <chrono>

using namespace evo_engine;

namespace {
using Clock = std::chrono::steady_clock;

float ElapsedMilliseconds(const Clock::time_point start) {
  return std::chrono::duration<float, std::milli>(Clock::now() - start).count();
}

void RecordRayDiagnostics(const VkCommandBuffer vk_command_buffer, const RenderGraphExecutionContext& context,
                          const DdgiRayDiagnosticsPass::Parameters& parameters) {
  if (!parameters.pipeline || !parameters.pipeline->Initialized() || !parameters.per_frame_descriptor_set ||
      !parameters.ray_tracing_descriptor_set || !parameters.ray_output_layout || !parameters.transient_resources) {
    return;
  }
  const auto* binding = context.GetResourceBinding(RenderResourceNames::frame_ddgi_ray_output);
  const auto* update_indices_binding = context.GetResourceBinding(RenderResourceNames::frame_ddgi_probe_update_indices);
  const auto* state_binding = context.GetResourceBinding(RenderResourceNames::frame_ddgi_probe_state);
  const auto* irradiance_binding = context.GetResourceBinding(RenderResourceNames::frame_ddgi_irradiance_atlas);
  const auto* visibility_binding = context.GetResourceBinding(RenderResourceNames::frame_ddgi_visibility_atlas);
  if (!binding || !binding->buffer || !update_indices_binding || !update_indices_binding->buffer || !state_binding ||
      !state_binding->buffer || !irradiance_binding || !irradiance_binding->image || !visibility_binding ||
      !visibility_binding->image) {
    return;
  }
  ApplyGraphResourceBarriers(vk_command_buffer, context, RenderPassQueue::RayTracing);

  const auto fallback_info = CreateDdgiFallbackImageInfo();
  const auto irradiance_view = CreateGraphImageMipView(irradiance_binding->image, 0);
  const auto visibility_view = CreateGraphImageMipView(visibility_binding->image, 0);
  if (!IsValidDescriptorImageInfo(fallback_info) || !irradiance_view || !visibility_view) {
    return;
  }
  parameters.transient_resources->RetainImageView(irradiance_view);
  parameters.transient_resources->RetainImageView(visibility_view);

  const auto ray_output_descriptor_set = std::make_shared<DescriptorSet>(parameters.ray_output_layout);
  ray_output_descriptor_set->UpdateBufferDescriptorBinding(0, binding->buffer);
  ray_output_descriptor_set->UpdateBufferDescriptorBinding(1, state_binding->buffer);
  ray_output_descriptor_set->UpdateBufferDescriptorBinding(2, update_indices_binding->buffer);
  VkDescriptorImageInfo atlas_info{};
  atlas_info.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
  atlas_info.sampler = parameters.atlas_sampler ? parameters.atlas_sampler->GetVkSampler() : fallback_info.sampler;
  atlas_info.imageView = irradiance_view->GetVkImageView();
  ray_output_descriptor_set->UpdateImageDescriptorBinding(17, atlas_info);
  atlas_info.imageView = visibility_view->GetVkImageView();
  ray_output_descriptor_set->UpdateImageDescriptorBinding(18, atlas_info);
  parameters.pipeline->Bind(vk_command_buffer);
  parameters.pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                         parameters.per_frame_descriptor_set->GetVkDescriptorSet());
  parameters.pipeline->BindDescriptorSet(vk_command_buffer, 1,
                                         parameters.ray_tracing_descriptor_set->GetVkDescriptorSet());
  parameters.pipeline->BindDescriptorSet(vk_command_buffer, 2, ray_output_descriptor_set->GetVkDescriptorSet());
  parameters.pipeline->PushConstant(vk_command_buffer, 0, parameters.push_constant);
  const auto sample_count =
      parameters.push_constant.probe_offset_and_update_count.y * parameters.push_constant.probe_counts_and_ray_count.w;
  parameters.pipeline->Trace(vk_command_buffer, sample_count, 1, 1);
  parameters.transient_resources->RetainDescriptorSet(ray_output_descriptor_set);
  Platform::EverythingBarrier(vk_command_buffer);
  ApplyGraphResourceReleaseBarriers(vk_command_buffer, context, RenderPassQueue::RayTracing);
}
}  // namespace

RenderPassDescriptor DdgiRayDiagnosticsPass::CreateDescriptor() {
  return {
      RenderPassNames::ddgi_ray_diagnostics,
      RenderPassQueue::RayTracing,
      RenderPassScope::Frame,
      {{RenderResourceNames::frame_per_frame_descriptor_set, RenderResourceUsage::Read, RenderResourceState::General},
       {RenderResourceNames::frame_ray_tracing_descriptor_set, RenderResourceUsage::Read, RenderResourceState::General},
       {RenderResourceNames::scene_mesh_tlas, RenderResourceUsage::Read,
        RenderResourceState::AccelerationStructureRead},
       {RenderResourceNames::frame_ddgi_probe_update_indices, RenderResourceUsage::Read,
        RenderResourceState::ShaderRead},
       {RenderResourceNames::frame_ddgi_probe_state, RenderResourceUsage::Read, RenderResourceState::ShaderRead},
       {RenderResourceNames::frame_ddgi_irradiance_atlas, RenderResourceUsage::Read, RenderResourceState::ShaderRead},
       {RenderResourceNames::frame_ddgi_visibility_atlas, RenderResourceUsage::Read, RenderResourceState::ShaderRead},
       {RenderResourceNames::frame_ddgi_ray_output, RenderResourceUsage::ReadWrite,
        RenderResourceState::StorageReadWrite}}};
}

void DdgiRayDiagnosticsPass::Execute(const RenderGraphExecutionContext& context, const Parameters& parameters) {
  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    const auto timer = Clock::now();
    RecordRayDiagnostics(vk_command_buffer, context, parameters);
    if (parameters.record_time_ms) {
      *parameters.record_time_ms += ElapsedMilliseconds(timer);
    }
  });
}
