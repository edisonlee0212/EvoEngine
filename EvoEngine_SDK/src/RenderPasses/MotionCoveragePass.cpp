#include "RenderPasses/MotionCoveragePass.hpp"

#include "Camera.hpp"
#include "GeometryStorage.hpp"
#include "GraphicsPipeline.hpp"
#include "GraphicsResources.hpp"
#include "Mesh.hpp"
#include "Platform.hpp"
#include "RenderInstanceStorage.hpp"
#include "RenderPasses/RenderPassUtilities.hpp"
#include "RenderTexture.hpp"

#include <algorithm>
#include <vector>

using namespace evo_engine;

namespace {
struct SortedTransparentInstance {
  std::shared_ptr<RenderInstanceStorage::MeshRenderInstance> render_instance;
  float distance_squared = 0.0f;
};

std::vector<SortedTransparentInstance> CollectSortedTransparentMeshInstances(
    const std::shared_ptr<RenderInstanceStorage::MeshRenderInstanceCollection>& transparent_render_instances,
    const glm::vec3& camera_position) {
  std::vector<SortedTransparentInstance> sorted_instances;
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

RenderPassDescriptor MotionCoveragePass::CreateDescriptor() {
  return {
      RenderPassNames::motion_coverage,
      RenderPassQueue::Graphics,
      RenderPassScope::Camera,
      {{RenderResourceNames::frame_render_instances, RenderResourceUsage::Read, RenderResourceState::ShaderRead},
       {RenderResourceNames::frame_per_frame_descriptor_set, RenderResourceUsage::Read, RenderResourceState::General},
       {RenderResourceNames::camera_depth, RenderResourceUsage::Read, RenderResourceState::DepthAttachment},
       {RenderResourceNames::camera_motion_vectors, RenderResourceUsage::ReadWrite,
        RenderResourceState::ColorAttachment}},
      {RenderPassNames::motion_vectors},
      RenderPassProfilerGroup::CameraVisibility,
      "Motion Coverage"};
}

void MotionCoveragePass::Execute(const RenderGraphExecutionContext& context, const Parameters& parameters) {
  if (!parameters.record_commands) {
    return;
  }
  parameters.record_commands([&](const VkCommandBuffer vk_command_buffer) {
    ApplyGraphResourceBarriers(vk_command_buffer, context);
    const RenderPassGpuTimestampScope gpu_timestamp(vk_command_buffer, context,
                                                    parameters.camera ? parameters.camera->GetHandle().GetValue() : 0,
                                                    static_cast<uint64_t>(parameters.camera_index));
    const auto release_barriers = [&] {
      ApplyGraphResourceReleaseBarriers(vk_command_buffer, context, RenderPassQueue::Graphics);
    };
    const auto* motion_binding = context.GetResourceBinding(RenderResourceNames::camera_motion_vectors);
    if (!motion_binding || !motion_binding->image || !parameters.camera || !parameters.camera->GetRenderTexture() ||
        !parameters.render_instances || !parameters.per_frame_descriptor_set || !parameters.transient_resources) {
      release_barriers();
      return;
    }
    const auto* camera_visibility = parameters.render_instances->GetCameraRasterVisibility(parameters.camera_index);
    const bool use_camera_visibility = camera_visibility && camera_visibility->enabled;
    const auto& deferred_skinned_render_instances =
        use_camera_visibility ? camera_visibility->deferred_skinned_render_instances
                              : parameters.render_instances->deferred_skinned_render_instances;
    const auto& transparent_render_instances = use_camera_visibility
                                                   ? camera_visibility->transparent_render_instances
                                                   : parameters.render_instances->transparent_render_instances;
    const bool render_skinned_motion = parameters.skinned_pipeline && parameters.skinned_pipeline->Initialized() &&
                                       parameters.motion_coverage_layout && deferred_skinned_render_instances &&
                                       !deferred_skinned_render_instances->Empty();
    const bool render_transparent_motion =
        parameters.transparent_pipeline && parameters.transparent_pipeline->Initialized() &&
        parameters.motion_coverage_layout && parameters.render_instances->previous_instance_info_descriptor_buffer &&
        parameters.camera_index >= 0 &&
        static_cast<size_t>(parameters.camera_index) < parameters.render_instances->camera_info_blocks_.size() &&
        transparent_render_instances && !transparent_render_instances->Empty();
    if (!render_skinned_motion && !render_transparent_motion) {
      release_barriers();
      return;
    }
    const auto sorted_transparent_instances =
        render_transparent_motion
            ? CollectSortedTransparentMeshInstances(
                  transparent_render_instances,
                  glm::vec3(parameters.render_instances->camera_info_blocks_[parameters.camera_index].inverse_view[3]))
            : std::vector<SortedTransparentInstance>{};

    const auto extent = motion_binding->image->GetExtent();
    auto motion_view = CreateGraphImageMipView(motion_binding->image, 0);
    parameters.transient_resources->RetainImageView(motion_view);

    VkRenderingAttachmentInfo color_attachment{};
    color_attachment.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;
    color_attachment.imageView = motion_view->GetVkImageView();
    color_attachment.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    color_attachment.loadOp = VK_ATTACHMENT_LOAD_OP_LOAD;
    color_attachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    auto depth_attachment = parameters.camera->GetRenderTexture()->GetDepthAttachmentInfo(VK_ATTACHMENT_LOAD_OP_LOAD,
                                                                                          VK_ATTACHMENT_STORE_OP_STORE);
    VkRenderingInfo render_info{};
    render_info.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
    render_info.renderArea.extent = {extent.width, extent.height};
    render_info.layerCount = 1;
    render_info.colorAttachmentCount = 1;
    render_info.pColorAttachments = &color_attachment;
    render_info.pDepthAttachment = &depth_attachment;
    const glm::ivec4 viewport{0, 0, static_cast<int>(extent.width), static_cast<int>(extent.height)};

    Platform::RecordRenderCommands(render_info, vk_command_buffer, [&] {
      if (render_skinned_motion) {
        GeometryStorage::BindSkinnedVertices(vk_command_buffer);
        parameters.skinned_pipeline->states.ResetAllStates(1);
        parameters.skinned_pipeline->states.SetViewportScissor(viewport);
        parameters.skinned_pipeline->states.depth_test = true;
        parameters.skinned_pipeline->states.depth_write = false;
        parameters.skinned_pipeline->states.depth_compare = VK_COMPARE_OP_EQUAL;
        parameters.skinned_pipeline->Bind(vk_command_buffer);
        parameters.skinned_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                                       parameters.per_frame_descriptor_set->GetVkDescriptorSet());
        deferred_skinned_render_instances->ForEachSkinnedMeshRenderInstance([&](const auto& render_instance) {
          if (!render_instance || !render_instance->bone_matrices || !render_instance->skinned_mesh) {
            return;
          }
          const auto pose_descriptor_set = std::make_shared<DescriptorSet>(parameters.motion_coverage_layout);
          pose_descriptor_set->UpdateBufferDescriptorBinding(0,
                                                             render_instance->bone_matrices->GetPreviousBufferInfo());
          pose_descriptor_set->UpdateBufferDescriptorBinding(
              1, parameters.render_instances->previous_instance_info_descriptor_buffer);
          parameters.skinned_pipeline->BindDescriptorSet(vk_command_buffer, 2,
                                                         pose_descriptor_set->GetVkDescriptorSet());
          parameters.skinned_pipeline->states.polygon_mode =
              parameters.wire_frame ? VK_POLYGON_MODE_LINE : render_instance->polygon_mode;
          parameters.skinned_pipeline->states.cull_mode = render_instance->cull_mode;
          parameters.skinned_pipeline->states.line_width = render_instance->line_width;
          RenderInstancePushConstant push_constant{};
          push_constant.instance_index = render_instance->instance_index;
          push_constant.camera_index = parameters.camera_index;
          render_instance->Render(vk_command_buffer, push_constant, parameters.skinned_pipeline);
          parameters.transient_resources->RetainDescriptorSet(pose_descriptor_set);
        });
      }

      if (!sorted_transparent_instances.empty()) {
        const auto motion_descriptor_set = std::make_shared<DescriptorSet>(parameters.motion_coverage_layout);
        motion_descriptor_set->UpdateBufferDescriptorBinding(
            1, parameters.render_instances->previous_instance_info_descriptor_buffer);
        GeometryStorage::BindVertices(vk_command_buffer);
        parameters.transparent_pipeline->states.ResetAllStates(1);
        parameters.transparent_pipeline->states.color_blend_attachment_states[0].colorWriteMask =
            VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT;
        parameters.transparent_pipeline->states.SetViewportScissor(viewport);
        parameters.transparent_pipeline->states.depth_test = true;
        parameters.transparent_pipeline->states.depth_write = false;
        parameters.transparent_pipeline->states.depth_compare = VK_COMPARE_OP_LESS_OR_EQUAL;
        parameters.transparent_pipeline->Bind(vk_command_buffer);
        parameters.transparent_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                                           parameters.per_frame_descriptor_set->GetVkDescriptorSet());
        parameters.transparent_pipeline->BindDescriptorSet(vk_command_buffer, 2,
                                                           motion_descriptor_set->GetVkDescriptorSet());
        for (const auto& sorted_instance : sorted_transparent_instances) {
          const auto& render_instance = sorted_instance.render_instance;
          parameters.transparent_pipeline->states.polygon_mode =
              parameters.wire_frame ? VK_POLYGON_MODE_LINE : render_instance->polygon_mode;
          parameters.transparent_pipeline->states.cull_mode = render_instance->cull_mode;
          parameters.transparent_pipeline->states.line_width = render_instance->line_width;
          RenderInstancePushConstant push_constant{};
          push_constant.instance_index = render_instance->instance_index;
          push_constant.camera_index = parameters.camera_index;
          parameters.transparent_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
          render_instance->mesh->DrawIndexed(vk_command_buffer, parameters.transparent_pipeline->states, 1);
        }
        parameters.transient_resources->RetainDescriptorSet(motion_descriptor_set);
      }
    });
    release_barriers();
  });
}
