#include "RenderPasses/SubsurfaceBackfacePass.hpp"

#include "Camera.hpp"
#include "GeometryStorage.hpp"
#include "GraphicsPipeline.hpp"
#include "GraphicsResources.hpp"
#include "Platform.hpp"
#include "RenderInstanceStorage.hpp"
#include "RenderPasses/RenderPassUtilities.hpp"

using namespace evo_engine;

RenderPassDescriptor SubsurfaceBackfacePass::CreateDescriptor() {
  return {
      "SubsurfaceBackface",
      RenderPassQueue::Graphics,
      RenderPassScope::Camera,
      {{RenderResourceNames::frame_render_instances, RenderResourceUsage::Read, RenderResourceState::ShaderRead},
       {RenderResourceNames::frame_per_frame_descriptor_set, RenderResourceUsage::Read, RenderResourceState::General},
       {RenderResourceNames::camera_depth, RenderResourceUsage::Read, RenderResourceState::ShaderRead},
       {RenderResourceNames::camera_g_buffer, RenderResourceUsage::Read, RenderResourceState::ShaderRead},
       {"Camera.Subsurface.Backface", RenderResourceUsage::Write, RenderResourceState::ColorAttachment},
       {"Camera.Subsurface.BackfaceDepth", RenderResourceUsage::Write, RenderResourceState::DepthAttachment}},
      {RenderPassNames::deferred_geometry},
      RenderPassProfilerGroup::Geometry,
      "Subsurface Backface"};
}

void SubsurfaceBackfacePass::Execute(const RenderGraphExecutionContext& context, const Parameters& parameters) {
  parameters.record_commands([&](const VkCommandBuffer command_buffer) {
    ApplyGraphResourceBarriers(command_buffer, context);
    const RenderPassGpuTimestampScope timestamp(command_buffer, context, parameters.camera->GetHandle().GetValue(),
                                                static_cast<uint64_t>(parameters.camera_index));
    const auto color_image = context.GetResourceBinding("Camera.Subsurface.Backface")->image;
    const auto depth_image = context.GetResourceBinding("Camera.Subsurface.BackfaceDepth")->image;
    const auto color_view = CreateGraphImageMipView(color_image, 0);
    VkImageViewCreateInfo depth_view_info{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
    depth_view_info.image = depth_image->GetVkImage();
    depth_view_info.viewType = VK_IMAGE_VIEW_TYPE_2D;
    depth_view_info.format = depth_image->GetFormat();
    depth_view_info.subresourceRange = {VK_IMAGE_ASPECT_DEPTH_BIT, 0, 1, 0, 1};
    const auto depth_view = std::make_shared<ImageView>(depth_view_info, depth_image);
    parameters.transient_resources->RetainImageView(color_view);
    parameters.transient_resources->RetainImageView(depth_view);
    VkRenderingAttachmentInfo color{VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO};
    color.imageView = color_view->GetVkImageView();
    color.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
    color.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    color.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    VkRenderingAttachmentInfo depth{VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO};
    depth.imageView = depth_view->GetVkImageView();
    depth.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
    depth.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    depth.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    depth.clearValue.depthStencil.depth = 1.0f;
    VkRenderingInfo rendering{VK_STRUCTURE_TYPE_RENDERING_INFO};
    rendering.renderArea.extent = {parameters.camera->GetSize().x, parameters.camera->GetSize().y};
    rendering.layerCount = 1;
    rendering.colorAttachmentCount = 1;
    rendering.pColorAttachments = &color;
    rendering.pDepthAttachment = &depth;
    Platform::RecordRenderCommands(rendering, command_buffer, [&] {
      const auto* visibility = parameters.render_instances->GetCameraRasterVisibility(parameters.camera_index);
      const bool visible = visibility && visibility->enabled;
      const auto& materials = parameters.render_instances->GetGltfShadeMaterials();
      const auto draw = [&](const auto& instance, const std::shared_ptr<GraphicsPipeline>& pipeline) {
        const auto& material = materials[instance->material_index];
        if (material.subsurface_weight <= 0.0f || material.subsurface_mode != 0u)
          return;
        pipeline->states.ResetAllStates(1);
        pipeline->states.SetViewportScissor(
            {0, 0, static_cast<int>(parameters.camera->GetSize().x), static_cast<int>(parameters.camera->GetSize().y)});
        // Fragment facing tests also handle mirrored transforms.
        pipeline->states.cull_mode = VK_CULL_MODE_NONE;
        pipeline->states.depth_write = true;
        pipeline->states.depth_compare = VK_COMPARE_OP_LESS;
        pipeline->Bind(command_buffer);
        pipeline->BindDescriptorSet(command_buffer, 0, parameters.per_frame_descriptor_set->GetVkDescriptorSet());
        pipeline->BindDescriptorSet(command_buffer, 2,
                                    parameters.camera->GetGBufferDescriptorSet()->GetVkDescriptorSet());
        RenderInstancePushConstant constants;
        constants.camera_index = parameters.camera_index;
        constants.instance_index = instance->instance_index;
        instance->Render(command_buffer, constants, pipeline);
      };
      GeometryStorage::BindVertices(command_buffer);
      for (const auto& collection :
           {visible ? visibility->deferred_render_instances : parameters.render_instances->deferred_render_instances,
            visible ? visibility->deferred_masked_render_instances
                    : parameters.render_instances->deferred_masked_render_instances})
        collection->ForEachMeshRenderInstance([&](const auto& instance) {
          draw(instance, parameters.pipelines[0]);
        });
      for (const auto& collection : {visible ? visibility->deferred_instanced_render_instances
                                             : parameters.render_instances->deferred_instanced_render_instances,
                                     visible ? visibility->deferred_masked_instanced_render_instances
                                             : parameters.render_instances->deferred_masked_instanced_render_instances})
        collection->ForEachInstancedRenderInstance([&](const auto& instance) {
          draw(instance, parameters.pipelines[1]);
        });
      GeometryStorage::BindSkinnedVertices(command_buffer);
      for (const auto& collection : {visible ? visibility->deferred_skinned_render_instances
                                             : parameters.render_instances->deferred_skinned_render_instances,
                                     visible ? visibility->deferred_masked_skinned_render_instances
                                             : parameters.render_instances->deferred_masked_skinned_render_instances})
        collection->ForEachSkinnedMeshRenderInstance([&](const auto& instance) {
          draw(instance, parameters.pipelines[2]);
        });
    });
    ApplyGraphResourceReleaseBarriers(command_buffer, context, RenderPassQueue::Graphics);
  });
}
