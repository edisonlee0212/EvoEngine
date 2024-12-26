
#include "Delaunay.hpp"
#include "DsConstraints.hpp"
#include "DsOperators.hpp"
#include "DynamicTreeStrands.hpp"
#include "Shader.hpp"
#include "Tree.hpp"
using namespace eco_sys_lab_plugin;

bool DynamicStrands::FoliageRenderParameters::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  return false;
}

struct FoliageRenderPushConstant {
  union Index1 {
    int instance_index;
    int sub_light_index;
  } index1;
  union Index2 {
    int camera_index;
    int light_index;
  } index2;
  uint32_t leaf_size;
};
void DynamicStrands::BuildFoliageRenderingPipelines() {
  // Descriptor set layout
  point_light_render_pipeline = std::make_shared<GraphicsPipeline>();
  point_light_render_pipeline->task_shader = Shader::CreateTemporary(
      ShaderType::Task, Platform::Constants::shader_global_defines,
      std::filesystem::path("./EcoSysLabResources") / "Shaders/Graphics/Task/DynamicStrandsFoliageRendering.task");
  point_light_render_pipeline->mesh_shader =
      Shader::CreateTemporary(ShaderType::Mesh, Platform::Constants::shader_global_defines,
                              std::filesystem::path("./EcoSysLabResources") /
                                  "Shaders/Graphics/Mesh/Foliage/DynamicStrandsPointLightShadowMap.mesh");
  point_light_render_pipeline->fragment_shader =
      Shader::CreateTemporary(ShaderType::Fragment, Platform::Constants::shader_global_defines,
                              std::filesystem::path("./EcoSysLabResources") / "Shaders/Graphics/Fragment/Empty.frag");
  point_light_render_pipeline->geometry_type = GeometryType::Mesh;
  point_light_render_pipeline->descriptor_set_layouts.emplace_back(RenderLayer::per_frame_layout);
  point_light_render_pipeline->descriptor_set_layouts.emplace_back(strands_layout);
  point_light_render_pipeline->depth_attachment_format = Platform::Constants::shadow_map;
  point_light_render_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
  auto& point_light_push_constant_range = point_light_render_pipeline->push_constant_ranges.emplace_back();
  point_light_push_constant_range.size = sizeof(FoliageRenderPushConstant);
  point_light_push_constant_range.offset = 0;
  point_light_push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
  point_light_render_pipeline->Initialize();
  // Descriptor set layout
  spot_light_render_pipeline = std::make_shared<GraphicsPipeline>();
  spot_light_render_pipeline->task_shader = Shader::CreateTemporary(
      ShaderType::Task, Platform::Constants::shader_global_defines,
      std::filesystem::path("./EcoSysLabResources") / "Shaders/Graphics/Task/DynamicStrandsRendering.task");
  spot_light_render_pipeline->mesh_shader =
      Shader::CreateTemporary(ShaderType::Mesh, Platform::Constants::shader_global_defines,
                              std::filesystem::path("./EcoSysLabResources") /
                                  "Shaders/Graphics/Mesh/Foliage/DynamicStrandsSpotLightShadowMap.mesh");
  spot_light_render_pipeline->fragment_shader =
      Shader::CreateTemporary(ShaderType::Fragment, Platform::Constants::shader_global_defines,
                              std::filesystem::path("./EcoSysLabResources") / "Shaders/Graphics/Fragment/Empty.frag");
  spot_light_render_pipeline->geometry_type = GeometryType::Mesh;
  spot_light_render_pipeline->descriptor_set_layouts.emplace_back(RenderLayer::per_frame_layout);
  spot_light_render_pipeline->descriptor_set_layouts.emplace_back(strands_layout);
  spot_light_render_pipeline->depth_attachment_format = Platform::Constants::shadow_map;
  spot_light_render_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
  auto& spot_light_push_constant_range = spot_light_render_pipeline->push_constant_ranges.emplace_back();
  spot_light_push_constant_range.size = sizeof(FoliageRenderPushConstant);
  spot_light_push_constant_range.offset = 0;
  spot_light_push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
  spot_light_render_pipeline->Initialize();
  // Descriptor set layout
  directional_light_render_pipeline = std::make_shared<GraphicsPipeline>();
  directional_light_render_pipeline->task_shader = Shader::CreateTemporary(
      ShaderType::Task, Platform::Constants::shader_global_defines,
      std::filesystem::path("./EcoSysLabResources") / "Shaders/Graphics/Task/DynamicStrandsRendering.task");
  directional_light_render_pipeline->mesh_shader =
      Shader::CreateTemporary(ShaderType::Mesh, Platform::Constants::shader_global_defines,
                              std::filesystem::path("./EcoSysLabResources") /
                                  "Shaders/Graphics/Mesh/Foliage/DynamicStrandsDirectionalLightShadowMap.mesh");
  directional_light_render_pipeline->fragment_shader =
      Shader::CreateTemporary(ShaderType::Fragment, Platform::Constants::shader_global_defines,
                              std::filesystem::path("./EcoSysLabResources") / "Shaders/Graphics/Fragment/Empty.frag");
  directional_light_render_pipeline->geometry_type = GeometryType::Mesh;
  directional_light_render_pipeline->descriptor_set_layouts.emplace_back(RenderLayer::per_frame_layout);
  directional_light_render_pipeline->descriptor_set_layouts.emplace_back(strands_layout);
  directional_light_render_pipeline->depth_attachment_format = Platform::Constants::shadow_map;
  directional_light_render_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
  auto& directional_light_push_constant_range = directional_light_render_pipeline->push_constant_ranges.emplace_back();
  directional_light_push_constant_range.size = sizeof(FoliageRenderPushConstant);
  directional_light_push_constant_range.offset = 0;
  directional_light_push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
  directional_light_render_pipeline->Initialize();
  // Descriptor set layout
  render_pipeline = std::make_shared<GraphicsPipeline>();
  render_pipeline->task_shader = Shader::CreateTemporary(
      ShaderType::Task, Platform::Constants::shader_global_defines,
      std::filesystem::path("./EcoSysLabResources") / "Shaders/Graphics/Task/DynamicStrandsFoliageRendering.task");
  render_pipeline->mesh_shader = Shader::CreateTemporary(
      ShaderType::Mesh, Platform::Constants::shader_global_defines,
      std::filesystem::path("./EcoSysLabResources") / "Shaders/Graphics/Mesh/DynamicStrandsFoliageRendering.mesh");
  render_pipeline->fragment_shader = Shader::CreateTemporary(
      ShaderType::Fragment, Platform::Constants::shader_global_defines,
      std::filesystem::path("./EcoSysLabResources") / "Shaders/Graphics/Fragment/DynamicStrandsRendering.frag");
  render_pipeline->geometry_type = GeometryType::Mesh;
  render_pipeline->descriptor_set_layouts.emplace_back(RenderLayer::per_frame_layout);
  render_pipeline->descriptor_set_layouts.emplace_back(strands_layout);
  render_pipeline->descriptor_set_layouts.emplace_back(RenderLayer::lighting_layout);
  render_pipeline->depth_attachment_format = Platform::Constants::render_texture_depth;
  render_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
  render_pipeline->color_attachment_formats = {2, Platform::Constants::g_buffer_color};
  auto& push_constant_range = render_pipeline->push_constant_ranges.emplace_back();
  push_constant_range.size = sizeof(FoliageRenderPushConstant);
  push_constant_range.offset = 0;
  push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
  render_pipeline->Initialize();
}
void DynamicStrands::RegisterFoliageShadowMapRendering(const FoliageRenderParameters& render_parameters) const {
  if (!Platform::Constants::support_mesh_shader) {
    EVOENGINE_LOG("Failed to render! Mesh shader unsupported!")
    return;
  }
  const auto render_layer = Application::GetLayer<RenderLayer>();
  if (!render_layer) {
    EVOENGINE_LOG("Failed to render! RenderLayer not present!")
    return;
  }
  if (!point_light_render_pipeline || !point_light_render_pipeline->Initialized()) {
    return;
  }
  if (!spot_light_render_pipeline || !spot_light_render_pipeline->Initialized()) {
    return;
  }
  if (!directional_light_render_pipeline || !directional_light_render_pipeline->Initialized()) {
    return;
  }

  render_layer->RenderToPointLightShadowMap(
      [&](const VkCommandBuffer vk_command_buffer, const RenderLayer::PointLightShadowMapView& view) {
        const uint32_t task_work_group_invocations =
            Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxPreferredTaskWorkGroupInvocations;
        FoliageRenderPushConstant push_constant;
        push_constant.index1.sub_light_index = view.face_index;
        push_constant.index2.light_index = view.light_index;
        push_constant.leaf_size = foliage.size();

        const auto current_frame_index = Platform::GetCurrentFrameIndex();
        point_light_render_pipeline->Bind(vk_command_buffer);
        point_light_render_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                                       RenderLayer::GetPerFrameDescriptorSet()->GetVkDescriptorSet());
        point_light_render_pipeline->BindDescriptorSet(
            vk_command_buffer, 1, strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
        point_light_render_pipeline->states.ResetAllStates(0);
        point_light_render_pipeline->states.SetViewportScissor(view.viewport);
        point_light_render_pipeline->states.ApplyAllStates(vk_command_buffer);

        point_light_render_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
        const uint32_t count = Platform::DivUp(delaunay_tetrahedrons.size(), task_work_group_invocations);
        vkCmdDrawMeshTasksEXT(vk_command_buffer, count, 1, 1);
        return delaunay_tetrahedrons.size();
      });

  render_layer->RenderToSpotLightShadowMap(
      [&](const VkCommandBuffer vk_command_buffer, const RenderLayer::SpotLightShadowMapView& view) {
        const uint32_t task_work_group_invocations =
            Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxPreferredTaskWorkGroupInvocations;
        FoliageRenderPushConstant push_constant;
        push_constant.index1.sub_light_index = 0;
        push_constant.index2.light_index = view.light_index;
        push_constant.leaf_size = foliage.size();
        const auto current_frame_index = Platform::GetCurrentFrameIndex();
        spot_light_render_pipeline->Bind(vk_command_buffer);
        spot_light_render_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                                      RenderLayer::GetPerFrameDescriptorSet()->GetVkDescriptorSet());
        spot_light_render_pipeline->BindDescriptorSet(
            vk_command_buffer, 1, strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
        spot_light_render_pipeline->states.ResetAllStates(0);
        spot_light_render_pipeline->states.SetViewportScissor(view.viewport);
        spot_light_render_pipeline->states.ApplyAllStates(vk_command_buffer);

        spot_light_render_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
        const uint32_t count = Platform::DivUp(delaunay_tetrahedrons.size(), task_work_group_invocations);
        vkCmdDrawMeshTasksEXT(vk_command_buffer, count, 1, 1);
        return delaunay_tetrahedrons.size();
      });

  render_layer->RenderToDirectionalLightShadowMap(
      [&](const VkCommandBuffer vk_command_buffer, const RenderLayer::DirectionalLightShadowMapView& view) {
        const uint32_t task_work_group_invocations =
            Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxPreferredTaskWorkGroupInvocations;
        FoliageRenderPushConstant push_constant;
        push_constant.index1.sub_light_index = view.split_index;
        push_constant.index2.light_index = view.light_index;
        push_constant.leaf_size = foliage.size();
        const auto current_frame_index = Platform::GetCurrentFrameIndex();
        directional_light_render_pipeline->Bind(vk_command_buffer);
        directional_light_render_pipeline->BindDescriptorSet(
            vk_command_buffer, 0, RenderLayer::GetPerFrameDescriptorSet()->GetVkDescriptorSet());
        directional_light_render_pipeline->BindDescriptorSet(
            vk_command_buffer, 1, strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
        directional_light_render_pipeline->states.ResetAllStates(0);
        directional_light_render_pipeline->states.SetViewportScissor(view.viewport);
        directional_light_render_pipeline->states.ApplyAllStates(vk_command_buffer);

        directional_light_render_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
        const uint32_t count = Platform::DivUp(delaunay_tetrahedrons.size(), task_work_group_invocations);
        vkCmdDrawMeshTasksEXT(vk_command_buffer, count, 1, 1);
        return delaunay_tetrahedrons.size();
      });
}

void DynamicStrands::RegisterFoliageRenderFunction(const Handle& renderer_handle,
                                                   const FoliageRenderParameters& render_parameters) const {
  if (!Platform::Constants::support_mesh_shader) {
    EVOENGINE_LOG("Failed to render! Mesh shader unsupported!")
    return;
  }
  if (!render_pipeline || !render_pipeline->Initialized()) {
    return;
  }
  Application::GetLayer<RenderLayer>()->DeferredRenderingAllCameras(
      [&, renderer_handle](const VkCommandBuffer vk_command_buffer,
                           const std::vector<VkRenderingAttachmentInfo>& geometry_pass_color_attachment_infos,
                           const RenderLayer::ForwardRenderingView& view) {
        const auto current_frame_index = Platform::GetCurrentFrameIndex();
        const uint32_t task_work_group_invocations =
            Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxPreferredTaskWorkGroupInvocations;
        FoliageRenderPushConstant push_constant;
        push_constant.index1.instance_index =
            Application::GetLayer<RenderLayer>()->GetCurrentRenderInstanceStorage()->GetRenderInstanceIndex(
                renderer_handle);
        push_constant.index2.camera_index = view.camera_index;
        push_constant.leaf_size = foliage.size();
        render_pipeline->states.ResetAllStates(geometry_pass_color_attachment_infos.size());
        render_pipeline->states.SetViewportScissor(view.viewport);

        render_pipeline->states.ApplyAllStates(vk_command_buffer);

#ifdef USE_RENDERDOC
        if (rdoc_api) {
          rdoc_api->StartFrameCapture(NULL, NULL);
          EVOENGINE_LOG("RDOC API detected!");
        }
#endif  //  USERENDERDOC
        render_pipeline->Bind(vk_command_buffer);
        render_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                           RenderLayer::GetPerFrameDescriptorSet()->GetVkDescriptorSet());
        render_pipeline->BindDescriptorSet(vk_command_buffer, 1,
                                           strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
        render_pipeline->BindDescriptorSet(vk_command_buffer, 2,
                                           RenderLayer::GetLightingDescriptorSet()->GetVkDescriptorSet());

        render_pipeline->PushConstant(vk_command_buffer, 0, push_constant);

        const uint32_t count = Platform::DivUp(delaunay_tetrahedrons.size(), task_work_group_invocations);
        vkCmdDrawMeshTasksEXT(vk_command_buffer, count, 1, 1);
#ifdef USE_RENDERDOC
        if (rdoc_api)
          rdoc_api->EndFrameCapture(NULL, NULL);
#endif
        return segments.size();
      });
}