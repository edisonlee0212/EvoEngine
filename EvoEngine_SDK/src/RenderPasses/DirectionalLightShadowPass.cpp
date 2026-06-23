#include "RenderPasses/DirectionalLightShadowPass.hpp"

#include "GeometryStorage.hpp"
#include "GraphicsPipeline.hpp"
#include "GraphicsResources.hpp"
#include "Platform.hpp"
#include "RenderInstanceStorage.hpp"
#include "RenderPasses/RenderPassUtilities.hpp"

using namespace evo_engine;

namespace {
void AccountDraws(const bool count_draw_calls, const uint32_t current_frame_index, const uint32_t prim_count) {
  if (!count_draw_calls) {
    return;
  }
  auto& platform = Platform::GetInstance();
  platform.draw_call[current_frame_index]++;
  platform.prim_count[current_frame_index] += prim_count;
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
        RenderResourceState::DepthAttachment}}};
}

void DirectionalLightShadowPass::Execute(const RenderGraphExecutionContext& context, const Parameters& parameters) {
  if (!parameters.record_commands || !parameters.render_instances || !parameters.per_frame_descriptor_set ||
      !parameters.get_depth_attachment || parameters.shadow_map_extent.width == 0 ||
      parameters.shadow_map_extent.height == 0 || parameters.camera_index < 0 ||
      parameters.max_directional_light_count <= 0 || !parameters.directional_pipeline) {
    return;
  }

  parameters.record_commands([&](const VkCommandBuffer vk_command_buffer) {
    ApplyGraphResourceBarriers(vk_command_buffer, context);
    VkRect2D render_area;
    render_area.offset = {0, 0};
    render_area.extent = parameters.shadow_map_extent;
    for (uint32_t split = 0; split < 4; split++) {
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
        if (parameters.use_mesh_shader && parameters.meshlet_descriptor_set) {
          parameters.directional_pipeline->BindDescriptorSet(vk_command_buffer, 1,
                                                             parameters.meshlet_descriptor_set->GetVkDescriptorSet());
        }
        for (int i = 0; i < parameters.render_instances->render_info_block.directional_light_size; i++) {
          const auto light_block_index = parameters.camera_index * parameters.max_directional_light_count + i;
          const auto& directional_light_info_block =
              parameters.render_instances->directional_light_info_blocks_[light_block_index];
          const auto prepare_graphics_pipeline = [&](const std::shared_ptr<GraphicsPipeline>& target_pipeline) {
            if (!target_pipeline) {
              return false;
            }
            target_pipeline->states.ResetAllStates(0);
            target_pipeline->Bind(vk_command_buffer);
            target_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                               parameters.per_frame_descriptor_set->GetVkDescriptorSet());
            target_pipeline->states.SetViewportScissor(directional_light_info_block.viewport);
            return true;
          };
          GeometryStorage::BindVertices(vk_command_buffer);
          {
            if (!prepare_graphics_pipeline(parameters.directional_pipeline)) {
              return;
            }
            if (parameters.enable_indirect_rendering &&
                !parameters.render_instances->deferred_render_instances->Empty()) {
              RenderInstancePushConstant push_constant;
              push_constant.camera_index = parameters.camera_index * parameters.max_directional_light_count + i;
              push_constant.light_split_index = split;
              push_constant.instance_index = 0;
              parameters.directional_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
              parameters.directional_pipeline->states.ApplyAllStates(vk_command_buffer);
              AccountDraws(parameters.count_draw_calls, parameters.current_frame_index,
                           parameters.render_instances->total_mesh_triangles);
              if (parameters.use_mesh_shader) {
                Platform::DrawMeshTasksIndirect(
                    vk_command_buffer, *parameters.render_instances->mesh_draw_mesh_tasks_indirect_commands_buffer, 0,
                    parameters.render_instances->mesh_draw_mesh_tasks_indirect_commands.size(),
                    sizeof(VkDrawMeshTasksIndirectCommandEXT));
              } else {
                Platform::DrawIndexedIndirect(
                    vk_command_buffer, *parameters.render_instances->mesh_draw_indexed_indirect_commands_buffer, 0,
                    parameters.render_instances->mesh_draw_indexed_indirect_commands.size(),
                    sizeof(VkDrawIndexedIndirectCommand));
              }
            } else {
              parameters.render_instances->deferred_render_instances->ForEachRenderInstance(
                  [&](const auto& render_instance) {
                    if (!render_instance->cast_shadow) {
                      return;
                    }
                    RenderInstancePushConstant push_constant;
                    push_constant.camera_index = parameters.camera_index * parameters.max_directional_light_count + i;
                    push_constant.light_split_index = split;
                    push_constant.instance_index = render_instance->instance_index;
                    const auto prim_count =
                        render_instance->Render(vk_command_buffer, push_constant, parameters.directional_pipeline);
                    AccountDraws(parameters.count_draw_calls, parameters.current_frame_index, prim_count);
                  });
            }
          }
          {
            if (prepare_graphics_pipeline(parameters.instanced_pipeline)) {
              parameters.render_instances->deferred_instanced_render_instances->ForEachRenderInstance(
                  [&](const auto& render_instance) {
                    if (!render_instance->cast_shadow) {
                      return;
                    }
                    RenderInstancePushConstant push_constant;
                    push_constant.camera_index = parameters.camera_index * parameters.max_directional_light_count + i;
                    push_constant.light_split_index = split;
                    push_constant.instance_index = render_instance->instance_index;
                    const auto prim_count =
                        render_instance->Render(vk_command_buffer, push_constant, parameters.instanced_pipeline);
                    AccountDraws(parameters.count_draw_calls, parameters.current_frame_index, prim_count);
                  });
            }
          }
          GeometryStorage::BindSkinnedVertices(vk_command_buffer);
          {
            if (prepare_graphics_pipeline(parameters.skinned_pipeline)) {
              parameters.render_instances->deferred_skinned_render_instances->ForEachRenderInstance(
                  [&](const auto& render_instance) {
                    if (!render_instance->cast_shadow) {
                      return;
                    }
                    RenderInstancePushConstant push_constant;
                    push_constant.camera_index = parameters.camera_index * parameters.max_directional_light_count + i;
                    push_constant.light_split_index = split;
                    push_constant.instance_index = render_instance->instance_index;
                    const auto prim_count =
                        render_instance->Render(vk_command_buffer, push_constant, parameters.skinned_pipeline);
                    AccountDraws(parameters.count_draw_calls, parameters.current_frame_index, prim_count);
                  });
            }
          }
#ifdef EVOENGINE_WINDOWS
          GeometryStorage::BindStrandPoints(vk_command_buffer);
          {
            if (prepare_graphics_pipeline(parameters.strands_pipeline)) {
              parameters.render_instances->deferred_strands_render_instances->ForEachRenderInstance(
                  [&](const auto& render_instance) {
                    if (!render_instance->cast_shadow) {
                      return;
                    }
                    RenderInstancePushConstant push_constant;
                    push_constant.camera_index = parameters.camera_index * parameters.max_directional_light_count + i;
                    push_constant.light_split_index = split;
                    push_constant.instance_index = render_instance->instance_index;
                    const auto prim_count =
                        render_instance->Render(vk_command_buffer, push_constant, parameters.strands_pipeline);
                    AccountDraws(parameters.count_draw_calls, parameters.current_frame_index, prim_count);
                  });
            }
          }
#endif
          if (parameters.external_shadow_rendering &&
              i < parameters.render_instances->directional_light_info_blocks_.size()) {
            parameters.external_shadow_rendering(
                vk_command_buffer, i, split, parameters.render_instances->directional_light_info_blocks_[i].viewport);
          }
        }
      });
    }
    ApplyGraphResourceReleaseBarriers(vk_command_buffer, context, RenderPassQueue::Graphics);
  });
}
