#include "RenderPasses/DeferredGeometryPass.hpp"

#include "Camera.hpp"
#include "GeometryStorage.hpp"
#include "GraphicsPipeline.hpp"
#include "GraphicsResources.hpp"
#include "Platform.hpp"
#include "RenderInstanceStorage.hpp"
#include "RenderPasses/RenderPassUtilities.hpp"

using namespace evo_engine;

namespace {
void AccountDraws(const bool count_draw_calls, const uint32_t current_frame_index, const size_t prim_count,
                  const RenderDrawCallKind kind = RenderDrawCallKind::Direct, const size_t indirect_draw_commands = 0) {
  if (!count_draw_calls) {
    return;
  }
  Platform::CountRenderPassDraw(RenderPassDrawBucket::DeferredGeometry, kind, current_frame_index, prim_count,
                                indirect_draw_commands);
}

VkPolygonMode ResolvePolygonMode(const bool wire_frame, const VkPolygonMode instance_polygon_mode) {
  return wire_frame ? VK_POLYGON_MODE_LINE : instance_polygon_mode;
}

uint32_t MeshletCullingFlags(const VkCullModeFlags cull_mode) {
  return 3u | (cull_mode == VK_CULL_MODE_BACK_BIT ? 4u : cull_mode == VK_CULL_MODE_FRONT_BIT ? 8u : 0u);
}
}  // namespace

RenderPassDescriptor DeferredGeometryPass::CreateDescriptor() {
  return {
      RenderPassNames::deferred_geometry,
      RenderPassQueue::Graphics,
      RenderPassScope::Camera,
      {{RenderResourceNames::frame_render_instances, RenderResourceUsage::Read, RenderResourceState::ShaderRead},
       {RenderResourceNames::frame_per_frame_descriptor_set, RenderResourceUsage::Read, RenderResourceState::General},
       {RenderResourceNames::camera_depth, RenderResourceUsage::Write, RenderResourceState::DepthAttachment},
       {RenderResourceNames::camera_g_buffer, RenderResourceUsage::Write, RenderResourceState::ColorAttachment}},
      {RenderPassNames::directional_light_shadow},
      RenderPassProfilerGroup::Geometry,
      "Deferred Geometry"};
}

void DeferredGeometryPass::Execute(const RenderGraphExecutionContext& context, const Parameters& parameters) {
  if (!parameters.record_commands || !parameters.camera || !parameters.camera->GetRenderTexture() ||
      !parameters.render_instances || !parameters.per_frame_descriptor_set || parameters.camera_index < 0 ||
      !parameters.mesh_pipeline) {
    return;
  }

  parameters.record_commands([&](const VkCommandBuffer vk_command_buffer) {
    VkRect2D render_area;
    render_area.offset = {0, 0};
    render_area.extent.width = parameters.camera->GetSize().x;
    render_area.extent.height = parameters.camera->GetSize().y;
    const glm::ivec4 viewport{0, 0, static_cast<int>(parameters.camera->GetSize().x),
                              static_cast<int>(parameters.camera->GetSize().y)};

    ApplyGraphResourceBarriers(vk_command_buffer, context);
    const RenderPassGpuTimestampScope gpu_timestamp(vk_command_buffer, context,
                                                    parameters.camera->GetHandle().GetValue(),
                                                    static_cast<uint64_t>(parameters.camera_index));

    VkRenderingInfo geometry_pass_render_info{};
    geometry_pass_render_info.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
    geometry_pass_render_info.renderArea = render_area;
    geometry_pass_render_info.layerCount = 1;
    const auto geometry_pass_depth_attachment = parameters.camera->GetRenderTexture()->GetDepthAttachmentInfo(
        VK_ATTACHMENT_LOAD_OP_CLEAR, VK_ATTACHMENT_STORE_OP_STORE);
    geometry_pass_render_info.pDepthAttachment = &geometry_pass_depth_attachment;
    std::vector<VkRenderingAttachmentInfo> geometry_pass_color_attachment_infos;
    parameters.camera->AppendGBufferColorAttachmentInfos(geometry_pass_color_attachment_infos,
                                                         VK_ATTACHMENT_LOAD_OP_CLEAR, VK_ATTACHMENT_STORE_OP_STORE);
    geometry_pass_render_info.colorAttachmentCount = geometry_pass_color_attachment_infos.size();
    geometry_pass_render_info.pColorAttachments = geometry_pass_color_attachment_infos.data();
    Platform::RecordRenderCommands(geometry_pass_render_info, vk_command_buffer, [&]() {
      const auto* camera_visibility = parameters.render_instances->GetCameraRasterVisibility(parameters.camera_index);
      const bool use_camera_visibility = camera_visibility && camera_visibility->enabled;
      const VkDeviceSize indexed_indirect_buffer_offset =
          use_camera_visibility ? camera_visibility->indexed_indirect_buffer_offset : 0;
      const VkDeviceSize mesh_task_indirect_buffer_offset =
          use_camera_visibility ? camera_visibility->mesh_task_indirect_buffer_offset : 0;
      const auto& deferred_mesh_indirect_batches = use_camera_visibility
                                                       ? camera_visibility->deferred_mesh_indirect_batches
                                                       : parameters.render_instances->deferred_mesh_indirect_batches;
      const auto& mesh_draw_indexed_indirect_commands_buffer =
          use_camera_visibility ? camera_visibility->mesh_draw_indexed_indirect_commands_buffer
                                : parameters.render_instances->mesh_draw_indexed_indirect_commands_buffer;
      const auto& mesh_draw_mesh_tasks_indirect_commands_buffer =
          use_camera_visibility ? camera_visibility->mesh_draw_mesh_tasks_indirect_commands_buffer
                                : parameters.render_instances->mesh_draw_mesh_tasks_indirect_commands_buffer;
      const auto& deferred_render_instances = use_camera_visibility
                                                  ? camera_visibility->deferred_render_instances
                                                  : parameters.render_instances->deferred_render_instances;
      const auto& deferred_instanced_render_instances =
          use_camera_visibility ? camera_visibility->deferred_instanced_render_instances
                                : parameters.render_instances->deferred_instanced_render_instances;
      const auto& deferred_skinned_render_instances =
          use_camera_visibility ? camera_visibility->deferred_skinned_render_instances
                                : parameters.render_instances->deferred_skinned_render_instances;
      const auto& deferred_strands_render_instances =
          use_camera_visibility ? camera_visibility->deferred_strands_render_instances
                                : parameters.render_instances->deferred_strands_render_instances;
      GeometryStorage::BindVertices(vk_command_buffer);
      {
        parameters.mesh_pipeline->states.ResetAllStates(geometry_pass_color_attachment_infos.size());
        parameters.mesh_pipeline->states.SetViewportScissor(viewport);
        parameters.mesh_pipeline->states.polygon_mode =
            parameters.wire_frame ? VK_POLYGON_MODE_LINE : VK_POLYGON_MODE_FILL;
        parameters.mesh_pipeline->states.depth_write = true;
        parameters.mesh_pipeline->states.depth_compare = VK_COMPARE_OP_LESS_OR_EQUAL;
        parameters.mesh_pipeline->Bind(vk_command_buffer);
        parameters.mesh_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                                    parameters.per_frame_descriptor_set->GetVkDescriptorSet());
        if (parameters.use_mesh_shader && parameters.meshlet_descriptor_set) {
          parameters.mesh_pipeline->BindDescriptorSet(vk_command_buffer, 1,
                                                      parameters.meshlet_descriptor_set->GetVkDescriptorSet());
        }
        if (parameters.enable_indirect_rendering && !deferred_mesh_indirect_batches.empty()) {
          for (const auto& batch : deferred_mesh_indirect_batches) {
            if (batch.command_count == 0) {
              continue;
            }
            RenderInstancePushConstant push_constant;
            push_constant.camera_index = parameters.camera_index;
            push_constant.meshlet_culling_flags =
                MeshletCullingFlags(batch.cull_mode) | RenderInstancePushConstant::kRasterDrawInstanceMappingBit;
            push_constant.instance_index = static_cast<int>(
                (use_camera_visibility ? camera_visibility->draw_instance_index_offset
                                       : parameters.render_instances->deferred_mesh_draw_instance_index_offset) +
                batch.first_command);
            parameters.mesh_pipeline->states.polygon_mode =
                ResolvePolygonMode(parameters.wire_frame, batch.polygon_mode);
            parameters.mesh_pipeline->states.cull_mode = batch.cull_mode;
            parameters.mesh_pipeline->states.line_width = batch.line_width;
            parameters.mesh_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
            parameters.mesh_pipeline->states.ApplyAllStates(vk_command_buffer);
            AccountDraws(parameters.count_draw_calls, parameters.current_frame_index, batch.triangle_count,
                         RenderDrawCallKind::Indirect, batch.command_count);
            if (parameters.use_mesh_shader) {
              Platform::DrawMeshTasksIndirect(
                  vk_command_buffer, *mesh_draw_mesh_tasks_indirect_commands_buffer,
                  mesh_task_indirect_buffer_offset + batch.first_command * sizeof(VkDrawMeshTasksIndirectCommandEXT),
                  batch.command_count, sizeof(VkDrawMeshTasksIndirectCommandEXT));
            } else {
              Platform::DrawIndexedIndirect(
                  vk_command_buffer, *mesh_draw_indexed_indirect_commands_buffer,
                  indexed_indirect_buffer_offset + batch.first_command * sizeof(VkDrawIndexedIndirectCommand),
                  batch.command_count, sizeof(VkDrawIndexedIndirectCommand));
            }
          }
        } else {
          deferred_render_instances->ForEachMeshRenderInstance([&](const auto& render_instance) {
            RenderInstancePushConstant push_constant;
            push_constant.camera_index = parameters.camera_index;
            push_constant.meshlet_culling_flags = MeshletCullingFlags(render_instance->cull_mode);
            push_constant.instance_index = render_instance->instance_index;
            parameters.mesh_pipeline->states.polygon_mode =
                ResolvePolygonMode(parameters.wire_frame, render_instance->polygon_mode);
            parameters.mesh_pipeline->states.cull_mode = render_instance->cull_mode;
            parameters.mesh_pipeline->states.line_width = render_instance->line_width;
            const auto prim_count = render_instance->Render(vk_command_buffer, push_constant, parameters.mesh_pipeline);
            AccountDraws(parameters.count_draw_calls, parameters.current_frame_index, prim_count);
          });
        }
      }
      {
        if (parameters.instanced_pipeline) {
          parameters.instanced_pipeline->states.ResetAllStates(geometry_pass_color_attachment_infos.size());
          parameters.instanced_pipeline->states.SetViewportScissor(viewport);
          parameters.instanced_pipeline->states.depth_write = true;
          parameters.instanced_pipeline->states.depth_compare = VK_COMPARE_OP_LESS_OR_EQUAL;
          parameters.instanced_pipeline->Bind(vk_command_buffer);
          parameters.instanced_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                                           parameters.per_frame_descriptor_set->GetVkDescriptorSet());
          deferred_instanced_render_instances->ForEachInstancedRenderInstance([&](const auto& render_instance) {
            RenderInstancePushConstant push_constant;
            push_constant.camera_index = parameters.camera_index;
            push_constant.meshlet_culling_flags = 1u;
            push_constant.instance_index = render_instance->instance_index;
            parameters.instanced_pipeline->states.polygon_mode =
                ResolvePolygonMode(parameters.wire_frame, render_instance->polygon_mode);
            parameters.instanced_pipeline->states.cull_mode = render_instance->cull_mode;
            parameters.instanced_pipeline->states.line_width = render_instance->line_width;
            const auto prim_count =
                render_instance->Render(vk_command_buffer, push_constant, parameters.instanced_pipeline);
            AccountDraws(parameters.count_draw_calls, parameters.current_frame_index, prim_count);
          });
        }
      }
      GeometryStorage::BindSkinnedVertices(vk_command_buffer);
      {
        if (parameters.skinned_pipeline) {
          parameters.skinned_pipeline->states.ResetAllStates(geometry_pass_color_attachment_infos.size());
          parameters.skinned_pipeline->states.SetViewportScissor(viewport);
          parameters.skinned_pipeline->states.depth_write = true;
          parameters.skinned_pipeline->states.depth_compare = VK_COMPARE_OP_LESS_OR_EQUAL;
          parameters.skinned_pipeline->Bind(vk_command_buffer);
          parameters.skinned_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                                         parameters.per_frame_descriptor_set->GetVkDescriptorSet());
          deferred_skinned_render_instances->ForEachSkinnedMeshRenderInstance([&](const auto& render_instance) {
            RenderInstancePushConstant push_constant;
            push_constant.camera_index = parameters.camera_index;
            push_constant.meshlet_culling_flags = 1u;
            push_constant.instance_index = render_instance->instance_index;
            parameters.skinned_pipeline->states.polygon_mode =
                ResolvePolygonMode(parameters.wire_frame, render_instance->polygon_mode);
            parameters.skinned_pipeline->states.cull_mode = render_instance->cull_mode;
            parameters.skinned_pipeline->states.line_width = render_instance->line_width;
            const auto prim_count =
                render_instance->Render(vk_command_buffer, push_constant, parameters.skinned_pipeline);
            AccountDraws(parameters.count_draw_calls, parameters.current_frame_index, prim_count);
          });
        }
      }
      {
        if (parameters.strands_pipeline) {
          parameters.strands_pipeline->states.ResetAllStates(geometry_pass_color_attachment_infos.size());
          parameters.strands_pipeline->states.SetViewportScissor(viewport);
          parameters.strands_pipeline->states.depth_write = true;
          parameters.strands_pipeline->states.depth_compare = VK_COMPARE_OP_LESS_OR_EQUAL;
          parameters.strands_pipeline->Bind(vk_command_buffer);
          parameters.strands_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                                         parameters.per_frame_descriptor_set->GetVkDescriptorSet());
          parameters.strands_pipeline->BindDescriptorSet(
              vk_command_buffer, 1, parameters.strand_meshlet_descriptor_set->GetVkDescriptorSet());
          deferred_strands_render_instances->ForEachStrandsRenderInstance([&](const auto& render_instance) {
            RenderInstancePushConstant push_constant;
            push_constant.camera_index = parameters.camera_index;
            push_constant.meshlet_culling_flags = 1u;
            push_constant.instance_index = render_instance->instance_index;
            parameters.strands_pipeline->states.polygon_mode =
                ResolvePolygonMode(parameters.wire_frame, render_instance->polygon_mode);
            parameters.strands_pipeline->states.cull_mode = render_instance->cull_mode;
            parameters.strands_pipeline->states.line_width = render_instance->line_width;
            const auto prim_count =
                render_instance->Render(vk_command_buffer, push_constant, parameters.strands_pipeline);
            AccountDraws(parameters.count_draw_calls, parameters.current_frame_index, prim_count);
          });
        }
      }
      if (parameters.external_deferred_rendering) {
        parameters.external_deferred_rendering(vk_command_buffer, geometry_pass_color_attachment_infos, viewport);
      }
    });
    ApplyGraphResourceReleaseBarriers(vk_command_buffer, context, RenderPassQueue::Graphics);
  });
}
