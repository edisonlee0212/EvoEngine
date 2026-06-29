#include "RenderPasses/GaussianSplatPass.hpp"

#include "Camera.hpp"
#include "GaussianSplat.hpp"
#include "GraphicsPipeline.hpp"
#include "GraphicsResources.hpp"
#include "Platform.hpp"
#include "RenderInstanceStorage.hpp"
#include "RenderPasses/RenderPassUtilities.hpp"

#include <algorithm>

using namespace evo_engine;

namespace {
constexpr uint32_t kGaussianSplatSortedIndicesFlag = 1u;

[[nodiscard]] bool IsValidStorageBuffer(const std::shared_ptr<Buffer>& buffer) {
  return buffer && buffer->GetVkBuffer() != VK_NULL_HANDLE && buffer->GetSize() != 0;
}

void ConfigurePremultipliedAlphaBlend(GraphicsPipeline& pipeline) {
  if (pipeline.states.color_blend_attachment_states.empty()) {
    return;
  }
  auto& blend = pipeline.states.color_blend_attachment_states[0];
  blend.blendEnable = VK_TRUE;
  blend.srcColorBlendFactor = VK_BLEND_FACTOR_ONE;
  blend.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
  blend.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
  blend.dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
}

void RecordGaussianSplats(const VkCommandBuffer vk_command_buffer, const RenderGraphExecutionContext& context,
                          const GaussianSplatPass::Parameters& parameters,
                          const std::shared_ptr<RenderInstanceStorage::GaussianSplatRenderInstanceCollection>&
                              gaussian_splat_render_instances,
                          const uint32_t total_gaussian_splats) {
  if (!parameters.pipeline || !parameters.pipeline->Initialized() || !parameters.per_frame_descriptor_set ||
      !parameters.descriptor_set_layout || !parameters.transient_resources || !parameters.camera ||
      !parameters.camera->GetRenderTexture() || total_gaussian_splats == 0u || !gaussian_splat_render_instances ||
      gaussian_splat_render_instances->Empty()) {
    return;
  }

  ApplyGraphResourceBarriers(vk_command_buffer, context);
  std::vector<VkRenderingAttachmentInfo> color_attachment_infos;
  parameters.camera->GetRenderTexture()->AppendColorAttachmentInfos(color_attachment_infos, VK_ATTACHMENT_LOAD_OP_LOAD,
                                                                    VK_ATTACHMENT_STORE_OP_STORE);
  auto depth_attachment = parameters.camera->GetRenderTexture()->GetDepthAttachmentInfo(VK_ATTACHMENT_LOAD_OP_LOAD,
                                                                                        VK_ATTACHMENT_STORE_OP_STORE);

  VkRect2D render_area{};
  render_area.offset = {0, 0};
  render_area.extent.width = parameters.camera->GetSize().x;
  render_area.extent.height = parameters.camera->GetSize().y;
  VkRenderingInfo render_info{};
  render_info.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
  render_info.renderArea = render_area;
  render_info.layerCount = 1;
  render_info.colorAttachmentCount = static_cast<uint32_t>(color_attachment_infos.size());
  render_info.pColorAttachments = color_attachment_infos.data();
  render_info.pDepthAttachment = parameters.use_scene_depth ? &depth_attachment : nullptr;

  Platform::RecordRenderCommands(render_info, vk_command_buffer, [&]() {
    const glm::ivec4 viewport{0, 0, static_cast<int>(parameters.camera->GetSize().x),
                              static_cast<int>(parameters.camera->GetSize().y)};
    parameters.pipeline->states.ResetAllStates(color_attachment_infos.size());
    parameters.pipeline->states.SetViewportScissor(viewport);
    parameters.pipeline->states.depth_write = false;
    parameters.pipeline->states.depth_compare = VK_COMPARE_OP_LESS_OR_EQUAL;
    parameters.pipeline->states.cull_mode = VK_CULL_MODE_NONE;
    ConfigurePremultipliedAlphaBlend(*parameters.pipeline);
    parameters.pipeline->Bind(vk_command_buffer);
    parameters.pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                           parameters.per_frame_descriptor_set->GetVkDescriptorSet());

    gaussian_splat_render_instances->ForEachRenderInstance(
        [&](const std::shared_ptr<RenderInstanceStorage::IRenderInstance>& render_instance) {
          const auto gaussian_instance =
              std::dynamic_pointer_cast<RenderInstanceStorage::GaussianSplatRenderInstance>(render_instance);
          if (!gaussian_instance || !gaussian_instance->gaussian_splat) {
            return;
          }
          const auto splat_count = static_cast<uint32_t>(gaussian_instance->gaussian_splat->GetSplatCount());
          const auto splat_buffer = gaussian_instance->gaussian_splat->GetGpuDataBuffer();
          if (splat_count == 0u || !IsValidStorageBuffer(splat_buffer)) {
            return;
          }

          bool use_sorted_indices = false;
          auto index_buffer = splat_buffer;
          if (gaussian_instance->sort_mode == GaussianSplatSortMode::CpuDepth) {
            const auto* sort_cache = gaussian_instance->gaussian_splat->FindSortCache(
                parameters.camera->GetHandle(), gaussian_instance->renderer_handle);
            if (sort_cache && sort_cache->valid && sort_cache->indices.size() == splat_count &&
                IsValidStorageBuffer(sort_cache->index_buffer)) {
              use_sorted_indices = true;
              index_buffer = sort_cache->index_buffer;
            }
          }

          const auto descriptor_set = std::make_shared<DescriptorSet>(parameters.descriptor_set_layout);
          descriptor_set->UpdateBufferDescriptorBinding(0, splat_buffer);
          descriptor_set->UpdateBufferDescriptorBinding(1, index_buffer);
          const auto requested_sh_degree = static_cast<uint32_t>(std::clamp(gaussian_instance->sh_degree, 0, 3));
          const auto available_sh_degree = gaussian_instance->gaussian_splat->GetSphericalHarmonicsDegree();
          auto effective_sh_degree = std::min(requested_sh_degree, available_sh_degree);
          auto rest_float_count = gaussian_instance->gaussian_splat->GetSphericalHarmonicsRestFloatCount();
          auto rest_buffer = splat_buffer;
          if (effective_sh_degree > 0u) {
            if (const auto candidate = gaussian_instance->gaussian_splat->GetSphericalHarmonicsRestBuffer();
                IsValidStorageBuffer(candidate)) {
              rest_buffer = candidate;
            } else {
              effective_sh_degree = 0u;
              rest_float_count = 0u;
            }
          }
          descriptor_set->UpdateBufferDescriptorBinding(2, rest_buffer);
          parameters.transient_resources->RetainDescriptorSet(descriptor_set);
          parameters.pipeline->BindDescriptorSet(vk_command_buffer, 1, descriptor_set->GetVkDescriptorSet());

          parameters.pipeline->states.depth_test =
              parameters.use_scene_depth && gaussian_instance->depth_mode == GaussianSplatDepthMode::SceneDepth;
          parameters.pipeline->states.ApplyAllStates(vk_command_buffer);

          GaussianSplatPushConstant push_constant{};
          push_constant.camera_instance_count_flags =
              glm::uvec4(parameters.camera_index, static_cast<uint32_t>(gaussian_instance->instance_index), splat_count,
                         use_sorted_indices ? kGaussianSplatSortedIndicesFlag : 0u);
          push_constant.sh_degree_rest_count_reserved = glm::uvec4(effective_sh_degree, rest_float_count, 0u, 0u);
          const float opacity_scale = gaussian_instance->opacity_scale > 0.0f ? gaussian_instance->opacity_scale : 0.0f;
          push_constant.opacity_extent_min_max = glm::vec4(opacity_scale, 2.8284271f, 1.0f, 192.0f);
          parameters.pipeline->PushConstant(vk_command_buffer, 0, push_constant);
          vkCmdDraw(vk_command_buffer, 6u, splat_count, 0u, 0u);
        });
  });
  ApplyGraphResourceReleaseBarriers(vk_command_buffer, context, RenderPassQueue::Graphics);
}
}  // namespace

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

RenderPassDescriptor GaussianSplatPass::CreateOverlayDescriptor(const char* dependency) {
  return {
      RenderPassNames::gaussian_splat,
      RenderPassQueue::Graphics,
      RenderPassScope::Camera,
      {{RenderResourceNames::frame_render_instances, RenderResourceUsage::Read, RenderResourceState::ShaderRead},
       {RenderResourceNames::frame_per_frame_descriptor_set, RenderResourceUsage::Read, RenderResourceState::General},
       {RenderResourceNames::camera_color, RenderResourceUsage::ReadWrite, RenderResourceState::ColorAttachment}},
      {dependency ? dependency : RenderPassNames::ray_tracing_camera}};
}

void GaussianSplatPass::Execute(const RenderGraphExecutionContext& context, const Parameters& parameters) {
  const auto gaussian_splat_render_instances =
      parameters.render_instances ? parameters.render_instances->gaussian_splat_render_instances : nullptr;
  const auto total_gaussian_splats =
      parameters.render_instances ? parameters.render_instances->total_gaussian_splats : 0u;
  if (!parameters.record_commands || !parameters.camera || !parameters.camera->GetRenderTexture() ||
      !parameters.render_instances || total_gaussian_splats == 0u || !gaussian_splat_render_instances ||
      gaussian_splat_render_instances->Empty()) {
    return;
  }

  parameters.record_commands([&](const VkCommandBuffer vk_command_buffer) {
    RecordGaussianSplats(vk_command_buffer, context, parameters, gaussian_splat_render_instances,
                         total_gaussian_splats);
  });
}
