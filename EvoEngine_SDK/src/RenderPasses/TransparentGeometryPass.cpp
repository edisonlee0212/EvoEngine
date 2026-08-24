#include "RenderPasses/TransparentGeometryPass.hpp"

#include "Camera.hpp"
#include "GeometryStorage.hpp"
#include "GraphicsPipeline.hpp"
#include "GraphicsResources.hpp"
#include "Material.hpp"
#include "Mesh.hpp"
#include "Platform.hpp"
#include "RenderInstanceStorage.hpp"
#include "RenderPasses/RenderPassUtilities.hpp"

#include <algorithm>
#include <vector>

using namespace evo_engine;

namespace {
struct SortedTransparentInstance {
  std::shared_ptr<RenderInstanceStorage::MeshRenderInstance> render_instance;
  float distance_squared = 0.0f;
};

VkPolygonMode ResolvePolygonMode(const bool wire_frame, const VkPolygonMode instance_polygon_mode) {
  return wire_frame ? VK_POLYGON_MODE_LINE : instance_polygon_mode;
}

std::vector<SortedTransparentInstance> CollectSortedTransparentMeshInstances(
    const std::shared_ptr<RenderInstanceStorage::MeshRenderInstanceCollection>& transparent_render_instances,
    const glm::vec3& camera_position) {
  std::vector<SortedTransparentInstance> sorted_instances;
  if (!transparent_render_instances) {
    return sorted_instances;
  }

  transparent_render_instances->ForEachMeshRenderInstance([&](const auto& render_instance) {
    if (!render_instance || !render_instance->mesh || !render_instance->material) {
      return;
    }
    const auto center_to_camera = render_instance->world_bound.Center() - camera_position;
    sorted_instances.push_back({render_instance, glm::dot(center_to_camera, center_to_camera)});
  });

  std::sort(sorted_instances.begin(), sorted_instances.end(), [](const auto& left, const auto& right) {
    return left.distance_squared > right.distance_squared;
  });
  return sorted_instances;
}
}  // namespace

RenderPassDescriptor TransparentGeometryPass::CreateDescriptor(const char* dependency) {
  return {
      RenderPassNames::transparent_geometry,
      RenderPassQueue::Graphics,
      RenderPassScope::Camera,
      {{RenderResourceNames::frame_render_instances, RenderResourceUsage::Read, RenderResourceState::ShaderRead},
       {RenderResourceNames::frame_per_frame_descriptor_set, RenderResourceUsage::Read, RenderResourceState::General},
       {RenderResourceNames::lighting_directional_shadow_map, RenderResourceUsage::Read,
        RenderResourceState::ShaderRead},
       {RenderResourceNames::camera_depth, RenderResourceUsage::Read, RenderResourceState::DepthAttachment},
       {RenderResourceNames::camera_color, RenderResourceUsage::ReadWrite, RenderResourceState::ColorAttachment}},
      {dependency ? dependency : RenderPassNames::deferred_camera},
      RenderPassProfilerGroup::Geometry,
      "Transparent Geometry"};
}

void TransparentGeometryPass::Execute(const RenderGraphExecutionContext& context, const Parameters& parameters) {
  if (!parameters.record_commands || !parameters.camera || !parameters.camera->GetRenderTexture() ||
      !parameters.render_instances || !parameters.mesh_pipeline || !parameters.per_frame_descriptor_set ||
      !parameters.lighting_descriptor_set || !parameters.raster_lighting_texture_descriptor_set ||
      parameters.camera_index < 0) {
    return;
  }
  if (static_cast<size_t>(parameters.camera_index) >= parameters.render_instances->camera_info_blocks_.size()) {
    return;
  }

  const auto camera_position =
      glm::vec3(parameters.render_instances->camera_info_blocks_[parameters.camera_index].inverse_view[3]);
  const auto* camera_visibility = parameters.render_instances->GetCameraRasterVisibility(parameters.camera_index);
  const auto& transparent_render_instances = camera_visibility && camera_visibility->enabled
                                                 ? camera_visibility->transparent_render_instances
                                                 : parameters.render_instances->transparent_render_instances;
  const auto sorted_instances = CollectSortedTransparentMeshInstances(transparent_render_instances, camera_position);
  if (sorted_instances.empty()) {
    return;
  }

  parameters.record_commands([&](const VkCommandBuffer vk_command_buffer) {
    ApplyGraphResourceBarriers(vk_command_buffer, context);
    const RenderPassGpuTimestampScope gpu_timestamp(vk_command_buffer, context,
                                                    parameters.camera->GetHandle().GetValue(),
                                                    static_cast<uint64_t>(parameters.camera_index));

    VkRect2D render_area{};
    render_area.offset = {0, 0};
    render_area.extent.width = parameters.camera->GetSize().x;
    render_area.extent.height = parameters.camera->GetSize().y;
    const glm::ivec4 viewport{0, 0, static_cast<int>(parameters.camera->GetSize().x),
                              static_cast<int>(parameters.camera->GetSize().y)};

    std::vector<VkRenderingAttachmentInfo> color_attachment_infos;
    parameters.camera->GetRenderTexture()->AppendColorAttachmentInfos(
        color_attachment_infos, VK_ATTACHMENT_LOAD_OP_LOAD, VK_ATTACHMENT_STORE_OP_STORE);
    auto depth_attachment = parameters.camera->GetRenderTexture()->GetDepthAttachmentInfo(VK_ATTACHMENT_LOAD_OP_LOAD,
                                                                                          VK_ATTACHMENT_STORE_OP_STORE);

    VkRenderingInfo render_info{};
    render_info.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
    render_info.renderArea = render_area;
    render_info.layerCount = 1;
    render_info.colorAttachmentCount = static_cast<uint32_t>(color_attachment_infos.size());
    render_info.pColorAttachments = color_attachment_infos.data();
    render_info.pDepthAttachment = &depth_attachment;

    Platform::RecordRenderCommands(render_info, vk_command_buffer, [&]() {
      GeometryStorage::BindVertices(vk_command_buffer);
      parameters.mesh_pipeline->states.ResetAllStates(color_attachment_infos.size());
      parameters.mesh_pipeline->states.SetViewportScissor(viewport);
      parameters.mesh_pipeline->states.depth_test = true;
      parameters.mesh_pipeline->states.depth_write = false;
      parameters.mesh_pipeline->states.depth_compare = VK_COMPARE_OP_LESS_OR_EQUAL;

      parameters.mesh_pipeline->Bind(vk_command_buffer);
      parameters.mesh_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                                  parameters.per_frame_descriptor_set->GetVkDescriptorSet());
      parameters.mesh_pipeline->BindDescriptorSet(vk_command_buffer, 2,
                                                  parameters.lighting_descriptor_set->GetVkDescriptorSet());
      parameters.mesh_pipeline->BindDescriptorSet(
          vk_command_buffer, 4, parameters.raster_lighting_texture_descriptor_set->GetVkDescriptorSet());

      for (const auto& sorted_instance : sorted_instances) {
        const auto& render_instance = sorted_instance.render_instance;
        if (!render_instance || !render_instance->mesh || !render_instance->material) {
          continue;
        }
        render_instance->material->draw_settings.ApplySettings(parameters.mesh_pipeline->states);
        parameters.mesh_pipeline->states.polygon_mode =
            ResolvePolygonMode(parameters.wire_frame, render_instance->polygon_mode);
        parameters.mesh_pipeline->states.cull_mode = render_instance->cull_mode;
        parameters.mesh_pipeline->states.line_width = render_instance->line_width;
        parameters.mesh_pipeline->states.depth_test = true;
        parameters.mesh_pipeline->states.depth_write = false;
        parameters.mesh_pipeline->states.depth_compare = VK_COMPARE_OP_LESS_OR_EQUAL;

        RenderInstancePushConstant push_constant;
        push_constant.camera_index = parameters.camera_index;
        push_constant.instance_index = render_instance->instance_index;
        BindRasterMaterialDescriptorSet(vk_command_buffer, parameters.mesh_pipeline, parameters.render_instances,
                                        render_instance->material_index);
        parameters.mesh_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
        render_instance->mesh->DrawIndexed(vk_command_buffer, parameters.mesh_pipeline->states, 1);
        if (parameters.count_draw_calls) {
          Platform::CountRenderPassDraw(RenderPassDrawBucket::TransparentGeometry, RenderDrawCallKind::Direct,
                                        parameters.current_frame_index,
                                        render_instance->mesh->GetTriangleAmount() * 3u);
        }
      }
    });
    ApplyGraphResourceReleaseBarriers(vk_command_buffer, context, RenderPassQueue::Graphics);
  });
}
