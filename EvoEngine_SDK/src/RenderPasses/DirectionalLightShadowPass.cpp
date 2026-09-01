#include "RenderPasses/DirectionalLightShadowPass.hpp"

#include "GeometryStorage.hpp"
#include "GraphicsPipeline.hpp"
#include "GraphicsResources.hpp"
#include "Platform.hpp"
#include "RenderInstanceStorage.hpp"
#include "RenderPasses/RenderPassUtilities.hpp"

#include <cmath>
#include <initializer_list>
#include <vector>

using namespace evo_engine;

namespace {
void AccountDraws(const bool count_draw_calls, const uint32_t current_frame_index, const size_t prim_count,
                  const RenderDrawCallKind draw_kind = RenderDrawCallKind::Direct,
                  const size_t indirect_draw_commands = 0) {
  if (!count_draw_calls) {
    return;
  }
  Platform::CountRenderPassDraw(RenderPassDrawBucket::DirectionalLightShadow, draw_kind, current_frame_index,
                                prim_count, indirect_draw_commands);
}

bool LightCastsShadow(const glm::vec4& diffuse) {
  return diffuse.w > 0.5f;
}

uint32_t MeshletCullingFlags(const VkCullModeFlags cull_mode) {
  return 259u | (cull_mode == VK_CULL_MODE_BACK_BIT ? 4u : cull_mode == VK_CULL_MODE_FRONT_BIT ? 8u : 0u);
}

bool IsFiniteBound(const Bound& bound) {
  return std::isfinite(bound.min.x) && std::isfinite(bound.min.y) && std::isfinite(bound.min.z) &&
         std::isfinite(bound.max.x) && std::isfinite(bound.max.y) && std::isfinite(bound.max.z) &&
         bound.min.x <= bound.max.x && bound.min.y <= bound.max.y && bound.min.z <= bound.max.z;
}

bool BoundIntersectsClipSpace(const Bound& bound, const glm::mat4& matrix) {
  if (!IsFiniteBound(bound)) {
    return true;
  }
  std::vector<glm::vec3> corners;
  bound.PopulateCorners(corners);
  uint32_t outside_left = 0;
  uint32_t outside_right = 0;
  uint32_t outside_bottom = 0;
  uint32_t outside_top = 0;
  uint32_t outside_near = 0;
  uint32_t outside_far = 0;
  for (const auto& corner : corners) {
    const auto clip = matrix * glm::vec4(corner, 1.0f);
    if (!std::isfinite(clip.x) || !std::isfinite(clip.y) || !std::isfinite(clip.z) || !std::isfinite(clip.w)) {
      return true;
    }
    outside_left += clip.x < -clip.w ? 1 : 0;
    outside_right += clip.x > clip.w ? 1 : 0;
    outside_bottom += clip.y < -clip.w ? 1 : 0;
    outside_top += clip.y > clip.w ? 1 : 0;
    outside_near += clip.z < -clip.w ? 1 : 0;
    outside_far += clip.z > clip.w ? 1 : 0;
  }
  return outside_left != corners.size() && outside_right != corners.size() && outside_bottom != corners.size() &&
         outside_top != corners.size() && outside_near != corners.size() && outside_far != corners.size();
}

bool ShouldRenderShadowInstance(const std::shared_ptr<RenderInstanceStorage::IRenderInstance>& render_instance,
                                const glm::mat4& light_space_matrix) {
  return render_instance->cast_shadow && BoundIntersectsClipSpace(render_instance->world_bound, light_space_matrix);
}

}  // namespace

RenderPassDescriptor DirectionalLightShadowPass::CreateDescriptor() {
  return {
      RenderPassNames::directional_light_shadow,
      RenderPassQueue::Graphics,
      RenderPassScope::Camera,
      {{RenderResourceNames::frame_render_instances, RenderResourceUsage::Read, RenderResourceState::ShaderRead},
       {RenderResourceNames::frame_per_frame_descriptor_set, RenderResourceUsage::Read, RenderResourceState::General},
       {RenderResourceNames::lighting_directional_shadow_map, RenderResourceUsage::Write,
        RenderResourceState::DepthAttachment}},
      {},
      RenderPassProfilerGroup::Shadows,
      "Directional Shadow"};
}

void DirectionalLightShadowPass::Execute(const RenderGraphExecutionContext& context, const Parameters& parameters) {
  if (!parameters.record_commands || !parameters.render_instances || !parameters.per_frame_descriptor_set ||
      !parameters.get_depth_attachment || parameters.shadow_map_extent.width == 0 ||
      parameters.shadow_map_extent.height == 0 || parameters.camera_index < 0 ||
      parameters.max_directional_light_count <= 0 || !parameters.directional_opaque_pipeline) {
    return;
  }

  parameters.record_commands([&](const VkCommandBuffer vk_command_buffer) {
    ApplyGraphResourceBarriers(vk_command_buffer, context);
    VkRect2D render_area;
    render_area.offset = {0, 0};
    render_area.extent = parameters.shadow_map_extent;
    for (uint32_t split = 0; split < 4; split++) {
      const RenderPassGpuTimestampScope gpu_timestamp(vk_command_buffer, context, parameters.camera_index, split);
      const auto depth_attachment =
          parameters.get_depth_attachment(split, VK_ATTACHMENT_LOAD_OP_CLEAR, VK_ATTACHMENT_STORE_OP_STORE);
      VkRenderingInfo render_info{};
      render_info.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
      render_info.renderArea = render_area;
      render_info.layerCount = 1;
      render_info.colorAttachmentCount = 0;
      render_info.pColorAttachments = nullptr;
      render_info.pDepthAttachment = &depth_attachment;
      Platform::RecordRenderCommands(render_info, vk_command_buffer, [&]() {
        for (int i = 0; i < parameters.render_instances->render_info_block.directional_light_size; i++) {
          const auto light_block_index = parameters.camera_index * parameters.max_directional_light_count + i;
          const auto& directional_light_info_block =
              parameters.render_instances->directional_light_info_blocks_[light_block_index];
          if (!LightCastsShadow(directional_light_info_block.diffuse)) {
            continue;
          }
          const auto& light_space_matrix = directional_light_info_block.light_space_matrix[split];
          const auto* shadow_view =
              parameters.render_instances->GetDirectionalShadowView(parameters.camera_index, i, split);
          const bool use_compact_view = shadow_view != nullptr;
          const auto prepare_graphics_pipeline = [&](const std::shared_ptr<GraphicsPipeline>& target_pipeline) {
            if (!target_pipeline || !parameters.per_frame_descriptor_set) {
              return false;
            }
            target_pipeline->states.ResetAllStates(0);
            target_pipeline->Bind(vk_command_buffer);
            target_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                               parameters.per_frame_descriptor_set->GetVkDescriptorSet());
            if (parameters.use_mesh_shader && parameters.meshlet_descriptor_set &&
                (target_pipeline == parameters.directional_opaque_pipeline ||
                 target_pipeline == parameters.directional_masked_pipeline)) {
              target_pipeline->BindDescriptorSet(vk_command_buffer, 1,
                                                 parameters.meshlet_descriptor_set->GetVkDescriptorSet());
            } else if (parameters.strand_meshlet_descriptor_set &&
                       (target_pipeline == parameters.strands_opaque_pipeline ||
                        target_pipeline == parameters.strands_masked_pipeline)) {
              target_pipeline->BindDescriptorSet(vk_command_buffer, 1,
                                                 parameters.strand_meshlet_descriptor_set->GetVkDescriptorSet());
            }
            target_pipeline->states.SetViewportScissor(directional_light_info_block.viewport);
            return true;
          };
          const auto render_bucket = [&](const bool masked) {
            const auto& mesh_instances =
                use_compact_view
                    ? (masked ? shadow_view->deferred_masked_render_instances : shadow_view->deferred_render_instances)
                    : (masked ? parameters.render_instances->deferred_masked_render_instances
                              : parameters.render_instances->deferred_render_instances);
            const auto& instanced_instances =
                use_compact_view ? (masked ? shadow_view->deferred_masked_instanced_render_instances
                                           : shadow_view->deferred_instanced_render_instances)
                                 : (masked ? parameters.render_instances->deferred_masked_instanced_render_instances
                                           : parameters.render_instances->deferred_instanced_render_instances);
            const auto& skinned_instances =
                use_compact_view ? (masked ? shadow_view->deferred_masked_skinned_render_instances
                                           : shadow_view->deferred_skinned_render_instances)
                                 : (masked ? parameters.render_instances->deferred_masked_skinned_render_instances
                                           : parameters.render_instances->deferred_skinned_render_instances);
            const auto& strands_instances =
                use_compact_view ? (masked ? shadow_view->deferred_masked_strands_render_instances
                                           : shadow_view->deferred_strands_render_instances)
                                 : (masked ? parameters.render_instances->deferred_masked_strands_render_instances
                                           : parameters.render_instances->deferred_strands_render_instances);
            const auto& batches =
                use_compact_view
                    ? (masked ? shadow_view->masked_mesh_indirect_batches : shadow_view->opaque_mesh_indirect_batches)
                    : (masked ? parameters.render_instances->masked_shadow_mesh_indirect_batches
                              : parameters.render_instances->opaque_shadow_mesh_indirect_batches);
            const auto& mesh_pipeline =
                masked ? parameters.directional_masked_pipeline : parameters.directional_opaque_pipeline;
            const auto& instanced_pipeline =
                masked ? parameters.instanced_masked_pipeline : parameters.instanced_opaque_pipeline;
            const auto& skinned_pipeline =
                masked ? parameters.skinned_masked_pipeline : parameters.skinned_opaque_pipeline;
            const auto& strands_pipeline =
                masked ? parameters.strands_masked_pipeline : parameters.strands_opaque_pipeline;

            GeometryStorage::BindVertices(vk_command_buffer);
            if (parameters.enable_indirect_rendering && !batches.empty()) {
              const auto& buffer =
                  use_compact_view ? shadow_view->indirect_buffer
                  : parameters.use_mesh_shader
                      ? parameters.render_instances->shadow_mesh_draw_mesh_tasks_indirect_commands_buffer
                      : parameters.render_instances->shadow_mesh_draw_indexed_indirect_commands_buffer;
              if (buffer && prepare_graphics_pipeline(mesh_pipeline)) {
                for (const auto& batch : batches) {
                  if (batch.triangle_count == 0u) {
                    continue;
                  }
                  mesh_pipeline->states.cull_mode = batch.cull_mode;
                  mesh_pipeline->states.polygon_mode = batch.polygon_mode;
                  mesh_pipeline->states.line_width = batch.line_width;
                  RenderInstancePushConstant push_constant{};
                  push_constant.camera_index = light_block_index;
                  push_constant.light_split_index = split;
                  push_constant.instance_index = static_cast<int>(
                      (use_compact_view ? shadow_view->draw_instance_index_offset
                                        : parameters.render_instances->deferred_mesh_draw_instance_index_offset) +
                      batch.first_command);
                  push_constant.meshlet_culling_flags =
                      MeshletCullingFlags(batch.cull_mode) | RenderInstancePushConstant::kRasterDrawInstanceMappingBit;
                  mesh_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
                  mesh_pipeline->states.ApplyAllStates(vk_command_buffer);
                  AccountDraws(parameters.count_draw_calls, parameters.current_frame_index, batch.triangle_count,
                               RenderDrawCallKind::Indirect, batch.command_count);
                  const auto stride = parameters.use_mesh_shader ? sizeof(VkDrawMeshTasksIndirectCommandEXT)
                                                                 : sizeof(VkDrawIndexedIndirectCommand);
                  const VkDeviceSize offset =
                      (use_compact_view ? shadow_view->indirect_buffer_offset : 0) + batch.first_command * stride;
                  if (parameters.use_mesh_shader) {
                    Platform::DrawMeshTasksIndirect(vk_command_buffer, *buffer, offset, batch.command_count,
                                                    sizeof(VkDrawMeshTasksIndirectCommandEXT));
                  } else {
                    Platform::DrawIndexedIndirect(vk_command_buffer, *buffer, offset, batch.command_count,
                                                  sizeof(VkDrawIndexedIndirectCommand));
                  }
                }
              }
            } else if (prepare_graphics_pipeline(mesh_pipeline)) {
              mesh_instances->ForEachMeshRenderInstance([&](const auto& render_instance) {
                if (!use_compact_view && !ShouldRenderShadowInstance(render_instance, light_space_matrix)) {
                  return;
                }
                RenderInstancePushConstant push_constant{};
                push_constant.camera_index = light_block_index;
                push_constant.light_split_index = split;
                push_constant.instance_index = render_instance->instance_index;
                push_constant.meshlet_culling_flags = MeshletCullingFlags(render_instance->cull_mode);
                const auto prim_count = render_instance->Render(vk_command_buffer, push_constant, mesh_pipeline);
                AccountDraws(parameters.count_draw_calls, parameters.current_frame_index, prim_count);
              });
            }
            if (prepare_graphics_pipeline(instanced_pipeline)) {
              instanced_instances->ForEachInstancedRenderInstance([&](const auto& render_instance) {
                if (!use_compact_view && !ShouldRenderShadowInstance(render_instance, light_space_matrix)) {
                  return;
                }
                RenderInstancePushConstant push_constant{};
                push_constant.camera_index = light_block_index;
                push_constant.light_split_index = split;
                push_constant.instance_index = render_instance->instance_index;
                push_constant.meshlet_culling_flags = 257u;
                const auto prim_count = render_instance->Render(vk_command_buffer, push_constant, instanced_pipeline);
                AccountDraws(parameters.count_draw_calls, parameters.current_frame_index, prim_count);
              });
            }
            GeometryStorage::BindSkinnedVertices(vk_command_buffer);
            if (prepare_graphics_pipeline(skinned_pipeline)) {
              skinned_instances->ForEachSkinnedMeshRenderInstance([&](const auto& render_instance) {
                if (!use_compact_view && !ShouldRenderShadowInstance(render_instance, light_space_matrix)) {
                  return;
                }
                RenderInstancePushConstant push_constant{};
                push_constant.camera_index = light_block_index;
                push_constant.light_split_index = split;
                push_constant.instance_index = render_instance->instance_index;
                push_constant.meshlet_culling_flags = 257u;
                const auto prim_count = render_instance->Render(vk_command_buffer, push_constant, skinned_pipeline);
                AccountDraws(parameters.count_draw_calls, parameters.current_frame_index, prim_count);
              });
            }
            if (prepare_graphics_pipeline(strands_pipeline)) {
              strands_instances->ForEachStrandsRenderInstance([&](const auto& render_instance) {
                if (!use_compact_view && !ShouldRenderShadowInstance(render_instance, light_space_matrix)) {
                  return;
                }
                RenderInstancePushConstant push_constant{};
                push_constant.camera_index = light_block_index;
                push_constant.light_split_index = split;
                push_constant.instance_index = render_instance->instance_index;
                push_constant.meshlet_culling_flags = 257u;
                strands_pipeline->states.cull_mode = render_instance->cull_mode;
                const auto prim_count = render_instance->Render(vk_command_buffer, push_constant, strands_pipeline);
                AccountDraws(parameters.count_draw_calls, parameters.current_frame_index, prim_count);
              });
            }
            const auto& external =
                masked ? parameters.external_masked_shadow_rendering : parameters.external_opaque_shadow_rendering;
            if (external && i < parameters.render_instances->directional_light_info_blocks_.size()) {
              external(vk_command_buffer, i, split, directional_light_info_block.viewport, light_space_matrix);
            }
          };
          render_bucket(false);
          render_bucket(true);
        }
      });
    }
    ApplyGraphResourceReleaseBarriers(vk_command_buffer, context, RenderPassQueue::Graphics);
  });
}
