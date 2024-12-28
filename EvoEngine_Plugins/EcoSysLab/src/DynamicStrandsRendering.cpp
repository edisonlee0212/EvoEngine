
#include "Delaunay.hpp"
#include "DsConstraints.hpp"
#include "DsOperators.hpp"
#include "DynamicTreeStrands.hpp"
#include "Shader.hpp"
#include "Tree.hpp"
using namespace eco_sys_lab_plugin;

bool DynamicStrands::RenderParameters::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  ImGui::Checkbox("Render mesh", &render_alpha_shape_mesh);
  ImGui::Checkbox("Render interior complex", &render_complex);
  ImGui::Checkbox("Wireframe", &wireframe);
  ImGui::DragFloat("alpha", &alpha, 0.000001, 0.0f, 1.0f, "%.6f");
  ImGui::DragFloat("bifurcation alpha", &bifurcation_alpha, 0.000001, 0.0f, 1.0f, "%.6f");

  ImGui::Text("Use vertex color for visualization");

  ImGui::RadioButton("Disabled", (int*)&vertex_colors, Default);
  ImGui::RadioButton("Normals", (int*)&vertex_colors, Normals);
  ImGui::RadioButton("Tangents", (int*)&vertex_colors, Tangents);
  ImGui::RadioButton("Texture coordinates", (int*)&vertex_colors, TexCoords);

  return false;
}

struct RenderPushConstant {
  union Index1 {
    int instance_index;
    int sub_light_index;
  } index1;
  union Index2 {
    int camera_index;
    int light_index;
  } index2;
  uint32_t tetrahedrons_size = 0;
  float alpha = 0.0f;
  float bifurcation_alpha = 0.0f;
  int render_complex = 0;
  int vertex_colors = 0;
};
void DynamicStrands::BuildRenderingPipelines() {
  // Descriptor set layout
  point_light_render_pipeline = std::make_shared<GraphicsPipeline>();
  point_light_render_pipeline->task_shader = Shader::CreateTemporary(
      ShaderType::Task, Platform::Constants::shader_global_defines,
      std::filesystem::path("./EcoSysLabResources") / "Shaders/Graphics/Task/DynamicStrandsRendering.task");
  point_light_render_pipeline->mesh_shader = Shader::CreateTemporary(
      ShaderType::Mesh, Platform::Constants::shader_global_defines,
      std::filesystem::path("./EcoSysLabResources") / "Shaders/Graphics/Mesh/DynamicStrandsPointLightShadowMap.mesh");
  point_light_render_pipeline->fragment_shader =
      Shader::CreateTemporary(ShaderType::Fragment, Platform::Constants::shader_global_defines,
                              std::filesystem::path("./EcoSysLabResources") / "Shaders/Graphics/Fragment/Empty.frag");
  point_light_render_pipeline->geometry_type = GeometryType::Mesh;
  point_light_render_pipeline->descriptor_set_layouts.emplace_back(RenderLayer::per_frame_layout);
  point_light_render_pipeline->descriptor_set_layouts.emplace_back(strands_layout);
  point_light_render_pipeline->depth_attachment_format = Platform::Constants::shadow_map;
  point_light_render_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
  auto& point_light_push_constant_range = point_light_render_pipeline->push_constant_ranges.emplace_back();
  point_light_push_constant_range.size = sizeof(RenderPushConstant);
  point_light_push_constant_range.offset = 0;
  point_light_push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
  point_light_render_pipeline->Initialize();
  // Descriptor set layout
  spot_light_render_pipeline = std::make_shared<GraphicsPipeline>();
  spot_light_render_pipeline->task_shader = Shader::CreateTemporary(
      ShaderType::Task, Platform::Constants::shader_global_defines,
      std::filesystem::path("./EcoSysLabResources") / "Shaders/Graphics/Task/DynamicStrandsRendering.task");
  spot_light_render_pipeline->mesh_shader = Shader::CreateTemporary(
      ShaderType::Mesh, Platform::Constants::shader_global_defines,
      std::filesystem::path("./EcoSysLabResources") / "Shaders/Graphics/Mesh/DynamicStrandsSpotLightShadowMap.mesh");
  spot_light_render_pipeline->fragment_shader =
      Shader::CreateTemporary(ShaderType::Fragment, Platform::Constants::shader_global_defines,
                              std::filesystem::path("./EcoSysLabResources") / "Shaders/Graphics/Fragment/Empty.frag");
  spot_light_render_pipeline->geometry_type = GeometryType::Mesh;
  spot_light_render_pipeline->descriptor_set_layouts.emplace_back(RenderLayer::per_frame_layout);
  spot_light_render_pipeline->descriptor_set_layouts.emplace_back(strands_layout);
  spot_light_render_pipeline->depth_attachment_format = Platform::Constants::shadow_map;
  spot_light_render_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
  auto& spot_light_push_constant_range = spot_light_render_pipeline->push_constant_ranges.emplace_back();
  spot_light_push_constant_range.size = sizeof(RenderPushConstant);
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
                                  "Shaders/Graphics/Mesh/DynamicStrandsDirectionalLightShadowMap.mesh");
  directional_light_render_pipeline->fragment_shader =
      Shader::CreateTemporary(ShaderType::Fragment, Platform::Constants::shader_global_defines,
                              std::filesystem::path("./EcoSysLabResources") / "Shaders/Graphics/Fragment/Empty.frag");
  directional_light_render_pipeline->geometry_type = GeometryType::Mesh;
  directional_light_render_pipeline->descriptor_set_layouts.emplace_back(RenderLayer::per_frame_layout);
  directional_light_render_pipeline->descriptor_set_layouts.emplace_back(strands_layout);
  directional_light_render_pipeline->depth_attachment_format = Platform::Constants::shadow_map;
  directional_light_render_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
  auto& directional_light_push_constant_range = directional_light_render_pipeline->push_constant_ranges.emplace_back();
  directional_light_push_constant_range.size = sizeof(RenderPushConstant);
  directional_light_push_constant_range.offset = 0;
  directional_light_push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
  directional_light_render_pipeline->Initialize();
  // Descriptor set layout
  render_pipeline = std::make_shared<GraphicsPipeline>();
  render_pipeline->task_shader = Shader::CreateTemporary(
      ShaderType::Task, Platform::Constants::shader_global_defines,
      std::filesystem::path("./EcoSysLabResources") / "Shaders/Graphics/Task/DynamicStrandsRendering.task");
  render_pipeline->mesh_shader = Shader::CreateTemporary(
      ShaderType::Mesh, Platform::Constants::shader_global_defines,
      std::filesystem::path("./EcoSysLabResources") / "Shaders/Graphics/Mesh/DynamicStrandsRendering.mesh");
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
  push_constant_range.size = sizeof(RenderPushConstant);
  push_constant_range.offset = 0;
  push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
  render_pipeline->Initialize();
}

uint32_t DynamicStrands::RenderToPointLightShadowMap(const RenderParameters& render_parameters,
                                                     const VkCommandBuffer vk_command_buffer,
                                                     const RenderLayer::PointLightShadowMapView& view) const {
  const uint32_t task_work_group_invocations =
      Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxPreferredTaskWorkGroupInvocations;
  RenderPushConstant push_constant;
  push_constant.index1.sub_light_index = view.face_index;
  push_constant.index2.light_index = view.light_index;
  push_constant.tetrahedrons_size = delaunay_tetrahedrons.size();
  push_constant.alpha = render_parameters.alpha;
  push_constant.bifurcation_alpha = render_parameters.bifurcation_alpha;
  push_constant.render_complex = render_parameters.render_complex ? 1 : 0;
  push_constant.vertex_colors = render_parameters.vertex_colors;
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  point_light_render_pipeline->Bind(vk_command_buffer);
  point_light_render_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                                 RenderLayer::GetPerFrameDescriptorSet()->GetVkDescriptorSet());
  point_light_render_pipeline->BindDescriptorSet(vk_command_buffer, 1,
                                                 strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
  point_light_render_pipeline->states.ResetAllStates(0);
  point_light_render_pipeline->states.SetViewportScissor(view.viewport);
  point_light_render_pipeline->states.ApplyAllStates(vk_command_buffer);

  point_light_render_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
  const uint32_t count = Platform::DivUp(delaunay_tetrahedrons.size(), task_work_group_invocations);
  vkCmdDrawMeshTasksEXT(vk_command_buffer, count, 1, 1);
  return delaunay_tetrahedrons.size();
}

uint32_t DynamicStrands::RenderToSpotLightShadowMap(const RenderParameters& render_parameters,
                                                    const VkCommandBuffer vk_command_buffer,
                                                    const RenderLayer::SpotLightShadowMapView& view) const {
  const uint32_t task_work_group_invocations =
      Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxPreferredTaskWorkGroupInvocations;
  RenderPushConstant push_constant;
  push_constant.index1.sub_light_index = 0;
  push_constant.index2.light_index = view.light_index;
  push_constant.tetrahedrons_size = delaunay_tetrahedrons.size();
  push_constant.alpha = render_parameters.alpha;
  push_constant.bifurcation_alpha = render_parameters.bifurcation_alpha;
  push_constant.render_complex = render_parameters.render_complex ? 1 : 0;
  push_constant.vertex_colors = render_parameters.vertex_colors;
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  spot_light_render_pipeline->Bind(vk_command_buffer);
  spot_light_render_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                                RenderLayer::GetPerFrameDescriptorSet()->GetVkDescriptorSet());
  spot_light_render_pipeline->BindDescriptorSet(vk_command_buffer, 1,
                                                strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
  spot_light_render_pipeline->states.ResetAllStates(0);
  spot_light_render_pipeline->states.SetViewportScissor(view.viewport);
  spot_light_render_pipeline->states.ApplyAllStates(vk_command_buffer);

  spot_light_render_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
  const uint32_t count = Platform::DivUp(delaunay_tetrahedrons.size(), task_work_group_invocations);
  vkCmdDrawMeshTasksEXT(vk_command_buffer, count, 1, 1);
  return delaunay_tetrahedrons.size();
}

uint32_t DynamicStrands::RenderToDirectionalLightShadowMap(
    const RenderParameters& render_parameters, const VkCommandBuffer vk_command_buffer,
    const RenderLayer::DirectionalLightShadowMapView& view) const {
  const uint32_t task_work_group_invocations =
      Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxPreferredTaskWorkGroupInvocations;
  RenderPushConstant push_constant;
  push_constant.index1.sub_light_index = view.split_index;
  push_constant.index2.light_index = view.light_index;
  push_constant.tetrahedrons_size = delaunay_tetrahedrons.size();
  push_constant.alpha = render_parameters.alpha;
  push_constant.bifurcation_alpha = render_parameters.bifurcation_alpha;
  push_constant.render_complex = render_parameters.render_complex ? 1 : 0;
  push_constant.vertex_colors = render_parameters.vertex_colors;
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  directional_light_render_pipeline->Bind(vk_command_buffer);
  directional_light_render_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                                       RenderLayer::GetPerFrameDescriptorSet()->GetVkDescriptorSet());
  directional_light_render_pipeline->BindDescriptorSet(
      vk_command_buffer, 1, strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
  directional_light_render_pipeline->states.ResetAllStates(0);
  directional_light_render_pipeline->states.SetViewportScissor(view.viewport);
  directional_light_render_pipeline->states.ApplyAllStates(vk_command_buffer);

  directional_light_render_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
  const uint32_t count = Platform::DivUp(delaunay_tetrahedrons.size(), task_work_group_invocations);
  vkCmdDrawMeshTasksEXT(vk_command_buffer, count, 1, 1);
  return delaunay_tetrahedrons.size();
}

uint32_t DynamicStrands::RenderToCameraDeferred(
    const Handle& renderer_handle, const RenderParameters& render_parameters, const VkCommandBuffer vk_command_buffer,
    const std::vector<VkRenderingAttachmentInfo>& geometry_pass_color_attachment_infos,
    const RenderLayer::DeferredRenderingView& view) const {
  if (!render_parameters.render_alpha_shape_mesh) {
    return 0;
  }
  if (!Platform::Constants::support_mesh_shader) {
    EVOENGINE_LOG("Failed to render! Mesh shader unsupported!")
    return 0;
  }
  if (!render_pipeline || !render_pipeline->Initialized()) {
    return 0;
  }
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  const uint32_t task_work_group_invocations =
      Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxPreferredTaskWorkGroupInvocations;
  RenderPushConstant push_constant;
  push_constant.index1.instance_index =
      Application::GetLayer<RenderLayer>()->GetCurrentRenderInstanceStorage()->GetRenderInstanceIndex(renderer_handle);
  push_constant.index2.camera_index = view.camera_index;
  push_constant.tetrahedrons_size = delaunay_tetrahedrons.size();
  push_constant.alpha = render_parameters.alpha;
  push_constant.bifurcation_alpha = render_parameters.bifurcation_alpha;
  push_constant.render_complex = render_parameters.render_complex ? 1 : 0;
  push_constant.vertex_colors = render_parameters.vertex_colors;

  render_pipeline->states.ResetAllStates(geometry_pass_color_attachment_infos.size());
  render_pipeline->states.SetViewportScissor(view.viewport);
  render_pipeline->states.polygon_mode = render_parameters.wireframe ? VK_POLYGON_MODE_LINE : VK_POLYGON_MODE_FILL;

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
}