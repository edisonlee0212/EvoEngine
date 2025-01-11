
#include "Delaunay.hpp"
#include "DsConstraints.hpp"
#include "DsOperators.hpp"
#include "DynamicTreeStrands.hpp"
#include "Shader.hpp"
#include "Tree.hpp"
using namespace eco_sys_lab_plugin;

bool DynamicStrands::FoliageRenderParameters::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;

  if (ImGui::Checkbox("Wireframe", &wireframe)) {
    changed = true;
  }
  return changed;
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
  foliage_point_light_render_pipeline = std::make_shared<GraphicsPipeline>();
  foliage_point_light_render_pipeline->task_shader = Shader::CreateTemporary(
      ShaderType::Task, Platform::Constants::shader_global_defines,
      std::filesystem::path("./EcoSysLabResources") / "Shaders/Graphics/Task/DynamicStrands/Rendering/Foliage.task");
  foliage_point_light_render_pipeline->mesh_shader =
      Shader::CreateTemporary(ShaderType::Mesh, Platform::Constants::shader_global_defines,
                              std::filesystem::path("./EcoSysLabResources") /
                                  "Shaders/Graphics/Mesh/DynamicStrands/Rendering/Foliage/PointLightShadowMap.mesh");
  foliage_point_light_render_pipeline->fragment_shader =
      Shader::CreateTemporary(ShaderType::Fragment, Platform::Constants::shader_global_defines,
                              std::filesystem::path("./EcoSysLabResources") / "Shaders/Graphics/Fragment/Empty.frag");
  foliage_point_light_render_pipeline->geometry_type = GeometryType::Mesh;
  foliage_point_light_render_pipeline->descriptor_set_layouts.emplace_back(RenderLayer::per_frame_layout);
  foliage_point_light_render_pipeline->descriptor_set_layouts.emplace_back(strands_layout);
  foliage_point_light_render_pipeline->depth_attachment_format = Platform::Constants::shadow_map;
  foliage_point_light_render_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
  auto& point_light_push_constant_range = foliage_point_light_render_pipeline->push_constant_ranges.emplace_back();
  point_light_push_constant_range.size = sizeof(FoliageRenderPushConstant);
  point_light_push_constant_range.offset = 0;
  point_light_push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
  foliage_point_light_render_pipeline->Initialize();
  // Descriptor set layout
  foliage_spot_light_render_pipeline = std::make_shared<GraphicsPipeline>();
  foliage_spot_light_render_pipeline->task_shader = Shader::CreateTemporary(
      ShaderType::Task, Platform::Constants::shader_global_defines,
      std::filesystem::path("./EcoSysLabResources") / "Shaders/Graphics/Task/DynamicStrands/Rendering/Foliage.task");
  foliage_spot_light_render_pipeline->mesh_shader =
      Shader::CreateTemporary(ShaderType::Mesh, Platform::Constants::shader_global_defines,
                              std::filesystem::path("./EcoSysLabResources") /
                                  "Shaders/Graphics/Mesh/DynamicStrands/Rendering/Foliage/SpotLightShadowMap.mesh");
  foliage_spot_light_render_pipeline->fragment_shader =
      Shader::CreateTemporary(ShaderType::Fragment, Platform::Constants::shader_global_defines,
                              std::filesystem::path("./EcoSysLabResources") / "Shaders/Graphics/Fragment/Empty.frag");
  foliage_spot_light_render_pipeline->geometry_type = GeometryType::Mesh;
  foliage_spot_light_render_pipeline->descriptor_set_layouts.emplace_back(RenderLayer::per_frame_layout);
  foliage_spot_light_render_pipeline->descriptor_set_layouts.emplace_back(strands_layout);
  foliage_spot_light_render_pipeline->depth_attachment_format = Platform::Constants::shadow_map;
  foliage_spot_light_render_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
  auto& spot_light_push_constant_range = foliage_spot_light_render_pipeline->push_constant_ranges.emplace_back();
  spot_light_push_constant_range.size = sizeof(FoliageRenderPushConstant);
  spot_light_push_constant_range.offset = 0;
  spot_light_push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
  foliage_spot_light_render_pipeline->Initialize();
  // Descriptor set layout
  foliage_directional_light_render_pipeline = std::make_shared<GraphicsPipeline>();
  foliage_directional_light_render_pipeline->task_shader = Shader::CreateTemporary(
      ShaderType::Task, Platform::Constants::shader_global_defines,
      std::filesystem::path("./EcoSysLabResources") / "Shaders/Graphics/Task/DynamicStrands/Rendering/Foliage.task");
  foliage_directional_light_render_pipeline->mesh_shader = Shader::CreateTemporary(
      ShaderType::Mesh, Platform::Constants::shader_global_defines,
      std::filesystem::path("./EcoSysLabResources") /
          "Shaders/Graphics/Mesh/DynamicStrands/Rendering/Foliage/DirectionalLightShadowMap.mesh");
  foliage_directional_light_render_pipeline->fragment_shader =
      Shader::CreateTemporary(ShaderType::Fragment, Platform::Constants::shader_global_defines,
                              std::filesystem::path("./EcoSysLabResources") / "Shaders/Graphics/Fragment/Empty.frag");
  foliage_directional_light_render_pipeline->geometry_type = GeometryType::Mesh;
  foliage_directional_light_render_pipeline->descriptor_set_layouts.emplace_back(RenderLayer::per_frame_layout);
  foliage_directional_light_render_pipeline->descriptor_set_layouts.emplace_back(strands_layout);
  foliage_directional_light_render_pipeline->depth_attachment_format = Platform::Constants::shadow_map;
  foliage_directional_light_render_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
  auto& directional_light_push_constant_range =
      foliage_directional_light_render_pipeline->push_constant_ranges.emplace_back();
  directional_light_push_constant_range.size = sizeof(FoliageRenderPushConstant);
  directional_light_push_constant_range.offset = 0;
  directional_light_push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
  foliage_directional_light_render_pipeline->Initialize();
  // Descriptor set layout
  foliage_render_pipeline = std::make_shared<GraphicsPipeline>();
  foliage_render_pipeline->task_shader = Shader::CreateTemporary(
      ShaderType::Task, Platform::Constants::shader_global_defines,
      std::filesystem::path("./EcoSysLabResources") / "Shaders/Graphics/Task/DynamicStrands/Rendering/Foliage.task");
  foliage_render_pipeline->mesh_shader =
      Shader::CreateTemporary(ShaderType::Mesh, Platform::Constants::shader_global_defines,
                              std::filesystem::path("./EcoSysLabResources") /
                                  "Shaders/Graphics/Mesh/DynamicStrands/Rendering/Foliage/Rendering.mesh");
  foliage_render_pipeline->fragment_shader =
      Shader::CreateTemporary(ShaderType::Fragment, Platform::Constants::shader_global_defines,
                              std::filesystem::path("./EcoSysLabResources") /
                                  "Shaders/Graphics/Fragment/DynamicStrands/Rendering/Foliage.frag");
  foliage_render_pipeline->geometry_type = GeometryType::Mesh;
  foliage_render_pipeline->descriptor_set_layouts.emplace_back(RenderLayer::per_frame_layout);
  foliage_render_pipeline->descriptor_set_layouts.emplace_back(strands_layout);
  foliage_render_pipeline->descriptor_set_layouts.emplace_back(RenderLayer::lighting_layout);
  foliage_render_pipeline->depth_attachment_format = Platform::Constants::render_texture_depth;
  foliage_render_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
  foliage_render_pipeline->color_attachment_formats = {2, Platform::Constants::g_buffer_color};
  auto& push_constant_range = foliage_render_pipeline->push_constant_ranges.emplace_back();
  push_constant_range.size = sizeof(FoliageRenderPushConstant);
  push_constant_range.offset = 0;
  push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
  foliage_render_pipeline->Initialize();
}

uint32_t DynamicStrands::RenderFoliageToPointLightShadowMap(const FoliageRenderParameters& render_parameters,
                                                            const VkCommandBuffer vk_command_buffer,
                                                            const RenderLayer::PointLightShadowMapView& view) const {
  if (!render_parameters.enabled) {
    return 0;
  }
  const uint32_t task_work_group_invocations =
      Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxPreferredTaskWorkGroupInvocations;
  FoliageRenderPushConstant push_constant;
  push_constant.index1.sub_light_index = view.face_index;
  push_constant.index2.light_index = view.light_index;
  push_constant.leaf_size = foliage.size();

  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  foliage_point_light_render_pipeline->Bind(vk_command_buffer);
  foliage_point_light_render_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                                         RenderLayer::GetPerFrameDescriptorSet()->GetVkDescriptorSet());
  foliage_point_light_render_pipeline->BindDescriptorSet(
      vk_command_buffer, 1, strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
  foliage_point_light_render_pipeline->states.ResetAllStates(0);
  foliage_point_light_render_pipeline->states.SetViewportScissor(view.viewport);
  foliage_point_light_render_pipeline->states.ApplyAllStates(vk_command_buffer);

  foliage_point_light_render_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
  const uint32_t count = Platform::DivUp(foliage.size(), task_work_group_invocations);
  vkCmdDrawMeshTasksEXT(vk_command_buffer, count, 1, 1);
  return foliage.size();
}

uint32_t DynamicStrands::RenderFoliageToSpotLightShadowMap(const FoliageRenderParameters& render_parameters,
                                                           VkCommandBuffer vk_command_buffer,
                                                           const RenderLayer::SpotLightShadowMapView& view) const {
  if (!render_parameters.enabled) {
    return 0;
  }
  const uint32_t task_work_group_invocations =
      Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxPreferredTaskWorkGroupInvocations;
  FoliageRenderPushConstant push_constant;
  push_constant.index1.sub_light_index = 0;
  push_constant.index2.light_index = view.light_index;
  push_constant.leaf_size = foliage.size();
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  foliage_spot_light_render_pipeline->Bind(vk_command_buffer);
  foliage_spot_light_render_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                                        RenderLayer::GetPerFrameDescriptorSet()->GetVkDescriptorSet());
  foliage_spot_light_render_pipeline->BindDescriptorSet(
      vk_command_buffer, 1, strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
  foliage_spot_light_render_pipeline->states.ResetAllStates(0);
  foliage_spot_light_render_pipeline->states.SetViewportScissor(view.viewport);
  foliage_spot_light_render_pipeline->states.ApplyAllStates(vk_command_buffer);

  foliage_spot_light_render_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
  const uint32_t count = Platform::DivUp(foliage.size(), task_work_group_invocations);
  vkCmdDrawMeshTasksEXT(vk_command_buffer, count, 1, 1);
  return foliage.size();
}

uint32_t DynamicStrands::RenderFoliageToDirectionalLightShadowMap(
    const FoliageRenderParameters& render_parameters, VkCommandBuffer vk_command_buffer,
    const RenderLayer::DirectionalLightShadowMapView& view) const {
  if (!render_parameters.enabled) {
    return 0;
  }
  const uint32_t task_work_group_invocations =
      Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxPreferredTaskWorkGroupInvocations;
  FoliageRenderPushConstant push_constant;
  push_constant.index1.sub_light_index = view.split_index;
  push_constant.index2.light_index = view.light_index;
  push_constant.leaf_size = foliage.size();
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  foliage_directional_light_render_pipeline->Bind(vk_command_buffer);
  foliage_directional_light_render_pipeline->BindDescriptorSet(
      vk_command_buffer, 0, RenderLayer::GetPerFrameDescriptorSet()->GetVkDescriptorSet());
  foliage_directional_light_render_pipeline->BindDescriptorSet(
      vk_command_buffer, 1, strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
  foliage_directional_light_render_pipeline->states.ResetAllStates(0);
  foliage_directional_light_render_pipeline->states.SetViewportScissor(view.viewport);
  foliage_directional_light_render_pipeline->states.ApplyAllStates(vk_command_buffer);

  foliage_directional_light_render_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
  const uint32_t count = Platform::DivUp(foliage.size(), task_work_group_invocations);
  vkCmdDrawMeshTasksEXT(vk_command_buffer, count, 1, 1);
  return foliage.size();
}

uint32_t DynamicStrands::RenderFoliageToCameraDeferred(
    const Handle& renderer_handle, const FoliageRenderParameters& render_parameters, VkCommandBuffer vk_command_buffer,
    const std::vector<VkRenderingAttachmentInfo>& geometry_pass_color_attachment_infos,
    const RenderLayer::DeferredRenderingView& view) const {
  if (!render_parameters.enabled) {
    return 0;
  }
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  const uint32_t task_work_group_invocations =
      Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxPreferredTaskWorkGroupInvocations;
  FoliageRenderPushConstant push_constant;
  push_constant.index1.instance_index =
      Application::GetLayer<RenderLayer>()->GetCurrentRenderInstanceStorage()->GetRenderInstanceIndex(renderer_handle);
  push_constant.index2.camera_index = view.camera_index;
  push_constant.leaf_size = foliage.size();
  foliage_render_pipeline->states.ResetAllStates(geometry_pass_color_attachment_infos.size());
  foliage_render_pipeline->states.SetViewportScissor(view.viewport);
  foliage_render_pipeline->states.polygon_mode =
      render_parameters.wireframe ? VK_POLYGON_MODE_LINE : VK_POLYGON_MODE_FILL;
  foliage_render_pipeline->states.ApplyAllStates(vk_command_buffer);

#ifdef USE_RENDERDOC
  if (rdoc_api) {
    rdoc_api->StartFrameCapture(NULL, NULL);
    EVOENGINE_LOG("RDOC API detected!");
  }
#endif  //  USERENDERDOC
  foliage_render_pipeline->Bind(vk_command_buffer);
  foliage_render_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                             RenderLayer::GetPerFrameDescriptorSet()->GetVkDescriptorSet());
  foliage_render_pipeline->BindDescriptorSet(vk_command_buffer, 1,
                                             strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
  foliage_render_pipeline->BindDescriptorSet(vk_command_buffer, 2,
                                             RenderLayer::GetLightingDescriptorSet()->GetVkDescriptorSet());
  foliage_render_pipeline->PushConstant(vk_command_buffer, 0, push_constant);

  const uint32_t count = Platform::DivUp(foliage.size(), task_work_group_invocations);
  vkCmdDrawMeshTasksEXT(vk_command_buffer, count, 1, 1);
#ifdef USE_RENDERDOC
  if (rdoc_api)
    rdoc_api->EndFrameCapture(NULL, NULL);
#endif
  return foliage.size();
}