#include "DsAlphaShapeMeshing.hpp"

#include "Delaunay.hpp"
#include "DsConstraints.hpp"
#include "DsOperators.hpp"
#include "DynamicTreeStrands.hpp"
#include "RenderParameters.hpp"
#include "Shader.hpp"
#include "Tree.hpp"
using namespace eco_sys_lab_plugin;

bool BranchesRenderParameters::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::Checkbox("Tetrahedron complex", &render_complex))
    changed = true;
  if (ImGui::Checkbox("Solid", &solid))
    changed = true;
  if (ImGui::Checkbox("Wireframe", &wireframe))
    changed = true;
  if (ImGui::DragFloat("Extrusion distance", &global_extrusion_distance, 0.0001f, 0.0f, 0.1f, "%.4f"))
    changed = true;

  if (ImGui::DragFloat("Degenerate triangle threshold 1e-x", &degen_triangle_threshold_logairthmic, 0.01f, 0.0f, 40.0f,
                       "%.6f"))
    changed = true;
  if (ImGui::DragFloat("Break threshold", &break_threshold, 0.0001f, 0.0f, 1.0f, "%.4f"))
    changed = true;
  if (ImGui::Checkbox("Use cubic Hermite spline", &use_cubic_hermite_spline))
    changed = true;

  if (ImGui::TreeNodeEx("Use normal attribute for debugging")) {
    if (ImGui::RadioButton("Disabled", (int*)&vertex_colors, Default))
      changed = true;
    if (ImGui::RadioButton("Absolute Normals", (int*)&vertex_colors, Normals))
      changed = true;
    if (ImGui::RadioButton("Tangents", (int*)&vertex_colors, Tangents))
      changed = true;
    if (ImGui::RadioButton("Groups", (int*)&vertex_colors, Groups))
      changed = true;
    if (ImGui::RadioButton("Degree", (int*)&vertex_colors, Degree))
      changed = true;
    if (ImGui::RadioButton("Bark", (int*)&vertex_colors, Bark))
      changed = true;
    if (ImGui::RadioButton("Normal Quaternion", (int*)&vertex_colors, NormalQuaternion))
      changed = true;
    if (ImGui::RadioButton("Up", (int*)&vertex_colors, Up))
      changed = true;
    if (ImGui::RadioButton("Initial Up", (int*)&vertex_colors, InitUp))
      changed = true;
    if (ImGui::RadioButton("Axis", (int*)&vertex_colors, Axis))
      changed = true;
    if (ImGui::RadioButton("Initial Axis", (int*)&vertex_colors, InitAxis))
      changed = true;
    if (ImGui::RadioButton("Initial Angle", (int*)&vertex_colors, InitAngle))
      changed = true;

    ImGui::TreePop();
  }

  if (ImGui::Checkbox("Use polar coordinates for UV", &use_polar_coordinates_for_uv)) {
    changed = true;

    // reset v_multiplier to default value
    if (use_polar_coordinates_for_uv) {
      u_multiplier = 1.0f;
      v_multiplier = 0.025f;
    } else {
      u_multiplier = 1.0f;
      v_multiplier = 1.0f;
    }
  }

  if (use_polar_coordinates_for_uv) {
    if (ImGui::DragFloat("U-coordinate multiplier", &u_multiplier, 1.f, 1.f, 20))
      changed = true;
  } else {
    if (ImGui::DragFloat("U-coordinate multiplier", &u_multiplier, 0.001f, 0.0f, 100.0f))
      changed = true;
  }

  if (ImGui::DragFloat("V-coordinate multiplier", &v_multiplier, 0.001f, 0.0f, 100.0f))
    changed = true;

  if (ImGui::Checkbox("Persistent damage", &persistent_damage)) {
    changed = true;
  }

  return changed;
}

struct BranchesRenderPushConstant {
  union Index1 {
    int instance_index;
    int sub_light_index;
  } index1;
  union Index2 {
    int camera_index;
    int light_index;
  } index2;

  float u_multiplier = 1.f;
  float v_multiplier = 1.f;

  uint32_t tetrahedrons_size = 0;
  float alpha = 0.0f;
  float bifurcation_alpha = 0.0f;
  float max_dist_squared = 0.0f;

  int render_complex = 0;
  int vertex_colors = 0;
  int inner_wood_material_index = 0;
  int snow_material_index = 0;
  float global_extrusion_distance = 0.0f;
  float break_threshold = 0.01f;
  int use_polar_coordinates_for_uv = 1;
};

void DsAlphaShapeMeshing::BuildBranchesRenderingPipelines() {
  branches_point_light_render_pipeline = std::make_shared<GraphicsPipeline>();
  branches_point_light_render_pipeline->task_shader =
      Shader::CreateTemporary(ShaderType::Task, Platform::GetShaderGlobalDefines(),
                              std::filesystem::path("./EcoSysLabResources") /
                                  "Shaders/Graphics/Task/DynamicStrands/Rendering/AlphaShapeMeshing/Branches.task");
  branches_point_light_render_pipeline->mesh_shader = Shader::CreateTemporary(
      ShaderType::Mesh, Platform::GetShaderGlobalDefines(),
      std::filesystem::path("./EcoSysLabResources") /
          "Shaders/Graphics/Mesh/DynamicStrands/Rendering/AlphaShapeMeshing/Branches/PointLightShadowMap.mesh");
  branches_point_light_render_pipeline->fragment_shader =
      Shader::CreateTemporary(ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
                              std::filesystem::path("./EcoSysLabResources") / "Shaders/Graphics/Fragment/Empty.frag");
  branches_point_light_render_pipeline->geometry_type = GeometryType::Mesh;
  branches_point_light_render_pipeline->descriptor_set_layouts.emplace_back(RenderLayer::per_frame_layout);
  branches_point_light_render_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
  branches_point_light_render_pipeline->depth_attachment_format = Platform::Constants::shadow_map;
  branches_point_light_render_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
  auto& point_light_push_constant_range = branches_point_light_render_pipeline->push_constant_ranges.emplace_back();
  point_light_push_constant_range.size = sizeof(BranchesRenderPushConstant);
  point_light_push_constant_range.offset = 0;
  point_light_push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
  branches_point_light_render_pipeline->Initialize();
  // Descriptor set layout
  branches_spot_light_render_pipeline = std::make_shared<GraphicsPipeline>();
  branches_spot_light_render_pipeline->task_shader =
      Shader::CreateTemporary(ShaderType::Task, Platform::GetShaderGlobalDefines(),
                              std::filesystem::path("./EcoSysLabResources") /
                                  "Shaders/Graphics/Task/DynamicStrands/Rendering/AlphaShapeMeshing/Branches.task");
  branches_spot_light_render_pipeline->mesh_shader = Shader::CreateTemporary(
      ShaderType::Mesh, Platform::GetShaderGlobalDefines(),
      std::filesystem::path("./EcoSysLabResources") /
          "Shaders/Graphics/Mesh/DynamicStrands/Rendering/AlphaShapeMeshing/Branches/SpotLightShadowMap.mesh");
  branches_spot_light_render_pipeline->fragment_shader =
      Shader::CreateTemporary(ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
                              std::filesystem::path("./EcoSysLabResources") / "Shaders/Graphics/Fragment/Empty.frag");
  branches_spot_light_render_pipeline->geometry_type = GeometryType::Mesh;
  branches_spot_light_render_pipeline->descriptor_set_layouts.emplace_back(RenderLayer::per_frame_layout);
  branches_spot_light_render_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
  branches_spot_light_render_pipeline->depth_attachment_format = Platform::Constants::shadow_map;
  branches_spot_light_render_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
  auto& spot_light_push_constant_range = branches_spot_light_render_pipeline->push_constant_ranges.emplace_back();
  spot_light_push_constant_range.size = sizeof(BranchesRenderPushConstant);
  spot_light_push_constant_range.offset = 0;
  spot_light_push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
  branches_spot_light_render_pipeline->Initialize();
  // Descriptor set layout
  branches_directional_light_render_pipeline = std::make_shared<GraphicsPipeline>();
  branches_directional_light_render_pipeline->task_shader =
      Shader::CreateTemporary(ShaderType::Task, Platform::GetShaderGlobalDefines(),
                              std::filesystem::path("./EcoSysLabResources") /
                                  "Shaders/Graphics/Task/DynamicStrands/Rendering/AlphaShapeMeshing/Branches.task");
  branches_directional_light_render_pipeline->mesh_shader = Shader::CreateTemporary(
      ShaderType::Mesh, Platform::GetShaderGlobalDefines(),
      std::filesystem::path("./EcoSysLabResources") /
          "Shaders/Graphics/Mesh/DynamicStrands/Rendering/AlphaShapeMeshing/Branches/DirectionalLightShadowMap.mesh");
  branches_directional_light_render_pipeline->fragment_shader =
      Shader::CreateTemporary(ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
                              std::filesystem::path("./EcoSysLabResources") / "Shaders/Graphics/Fragment/Empty.frag");
  branches_directional_light_render_pipeline->geometry_type = GeometryType::Mesh;
  branches_directional_light_render_pipeline->descriptor_set_layouts.emplace_back(RenderLayer::per_frame_layout);
  branches_directional_light_render_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
  branches_directional_light_render_pipeline->depth_attachment_format = Platform::Constants::shadow_map;
  branches_directional_light_render_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
  auto& directional_light_push_constant_range =
      branches_directional_light_render_pipeline->push_constant_ranges.emplace_back();
  directional_light_push_constant_range.size = sizeof(BranchesRenderPushConstant);
  directional_light_push_constant_range.offset = 0;
  directional_light_push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
  branches_directional_light_render_pipeline->Initialize();
  // Descriptor set layout
  branches_render_pipeline = std::make_shared<GraphicsPipeline>();
  branches_render_pipeline->task_shader =
      Shader::CreateTemporary(ShaderType::Task, Platform::GetShaderGlobalDefines(),
                              std::filesystem::path("./EcoSysLabResources") /
                                  "Shaders/Graphics/Task/DynamicStrands/Rendering/AlphaShapeMeshing/Branches.task");
  branches_render_pipeline->mesh_shader = Shader::CreateTemporary(
      ShaderType::Mesh, Platform::GetShaderGlobalDefines(),
      std::filesystem::path("./EcoSysLabResources") /
          "Shaders/Graphics/Mesh/DynamicStrands/Rendering/AlphaShapeMeshing/Branches/Rendering.mesh");
  branches_render_pipeline->fragment_shader =
      Shader::CreateTemporary(ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
                              std::filesystem::path("./EcoSysLabResources") /
                                  "Shaders/Graphics/Fragment/DynamicStrands/Rendering/Branches.frag");
  branches_render_pipeline->geometry_type = GeometryType::Mesh;
  branches_render_pipeline->descriptor_set_layouts.emplace_back(RenderLayer::per_frame_layout);
  branches_render_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
  branches_render_pipeline->descriptor_set_layouts.emplace_back(RenderLayer::lighting_layout);
  branches_render_pipeline->depth_attachment_format = Platform::Constants::render_texture_depth;
  branches_render_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
  branches_render_pipeline->color_attachment_formats = {2, Platform::Constants::g_buffer_color};
  auto& push_constant_range = branches_render_pipeline->push_constant_ranges.emplace_back();
  push_constant_range.size = sizeof(BranchesRenderPushConstant);
  push_constant_range.offset = 0;
  push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
  branches_render_pipeline->Initialize();
}

uint32_t DsAlphaShapeMeshing::RenderBranchesToPointLightShadowMap(
    const BranchesRenderParameters& render_parameters, const VkCommandBuffer vk_command_buffer,
    const RenderLayer::PointLightShadowMapView& view) const {
  if (!render_parameters.enabled) {
    return 0;
  }
  const uint32_t task_work_group_invocations =
      Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxPreferredTaskWorkGroupInvocations;
  BranchesRenderPushConstant push_constant;
  push_constant.index1.sub_light_index = view.face_index;
  push_constant.index2.light_index = view.light_index;
  push_constant.tetrahedrons_size = delaunay_tetrahedrons.size();
  push_constant.alpha = render_parameters.alpha;
  push_constant.bifurcation_alpha = render_parameters.bifurcation_alpha;
  push_constant.max_dist_squared = render_parameters.max_dist_squared;
  push_constant.render_complex = render_parameters.render_complex ? 1 : 0;
  push_constant.vertex_colors = render_parameters.vertex_colors;
  push_constant.global_extrusion_distance = render_parameters.global_extrusion_distance;
  push_constant.break_threshold = render_parameters.break_threshold;
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  branches_point_light_render_pipeline->Bind(vk_command_buffer);
  branches_point_light_render_pipeline->BindDescriptorSet(
      vk_command_buffer, 0, RenderLayer::GetPerFrameDescriptorSet()->GetVkDescriptorSet());
  branches_point_light_render_pipeline->BindDescriptorSet(
      vk_command_buffer, 1, dynamic_strands->strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
  branches_point_light_render_pipeline->states.ResetAllStates(0);
  branches_point_light_render_pipeline->states.SetViewportScissor(view.viewport);
  branches_point_light_render_pipeline->states.ApplyAllStates(vk_command_buffer);

  branches_point_light_render_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
  const uint32_t count = Platform::DivUp(delaunay_tetrahedrons.size(), task_work_group_invocations);
  vkCmdDrawMeshTasksEXT(vk_command_buffer, count, 1, 1);
  return delaunay_tetrahedrons.size();
}

uint32_t DsAlphaShapeMeshing::RenderBranchesToSpotLightShadowMap(
    const BranchesRenderParameters& render_parameters, const VkCommandBuffer vk_command_buffer,
    const RenderLayer::SpotLightShadowMapView& view) const {
  if (!render_parameters.enabled) {
    return 0;
  }
  const uint32_t task_work_group_invocations =
      Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxPreferredTaskWorkGroupInvocations;
  BranchesRenderPushConstant push_constant;
  push_constant.index1.sub_light_index = 0;
  push_constant.index2.light_index = view.light_index;
  push_constant.tetrahedrons_size = delaunay_tetrahedrons.size();
  push_constant.alpha = render_parameters.alpha;
  push_constant.bifurcation_alpha = render_parameters.bifurcation_alpha;
  push_constant.max_dist_squared = render_parameters.max_dist_squared;
  push_constant.render_complex = render_parameters.render_complex ? 1 : 0;
  push_constant.vertex_colors = render_parameters.vertex_colors;
  push_constant.global_extrusion_distance = render_parameters.global_extrusion_distance;
  push_constant.break_threshold = render_parameters.break_threshold;
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  branches_spot_light_render_pipeline->Bind(vk_command_buffer);
  branches_spot_light_render_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                                         RenderLayer::GetPerFrameDescriptorSet()->GetVkDescriptorSet());
  branches_spot_light_render_pipeline->BindDescriptorSet(
      vk_command_buffer, 1, dynamic_strands->strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
  branches_spot_light_render_pipeline->states.ResetAllStates(0);
  branches_spot_light_render_pipeline->states.SetViewportScissor(view.viewport);
  branches_spot_light_render_pipeline->states.ApplyAllStates(vk_command_buffer);

  branches_spot_light_render_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
  const uint32_t count = Platform::DivUp(delaunay_tetrahedrons.size(), task_work_group_invocations);
  vkCmdDrawMeshTasksEXT(vk_command_buffer, count, 1, 1);
  return delaunay_tetrahedrons.size();
}

uint32_t DsAlphaShapeMeshing::RenderBranchesToDirectionalLightShadowMap(
    const BranchesRenderParameters& render_parameters, const VkCommandBuffer vk_command_buffer,
    const RenderLayer::DirectionalLightShadowMapView& view) const {
  if (!render_parameters.enabled) {
    return 0;
  }
  const uint32_t task_work_group_invocations =
      Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxPreferredTaskWorkGroupInvocations;
  BranchesRenderPushConstant push_constant;
  push_constant.index1.sub_light_index = view.split_index;
  push_constant.index2.light_index = view.light_index;
  push_constant.tetrahedrons_size = delaunay_tetrahedrons.size();
  push_constant.alpha = render_parameters.alpha;
  push_constant.bifurcation_alpha = render_parameters.bifurcation_alpha;
  push_constant.max_dist_squared = render_parameters.max_dist_squared;
  push_constant.render_complex = render_parameters.render_complex ? 1 : 0;
  push_constant.vertex_colors = render_parameters.vertex_colors;
  push_constant.global_extrusion_distance = render_parameters.global_extrusion_distance;
  push_constant.break_threshold = render_parameters.break_threshold;
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  branches_directional_light_render_pipeline->Bind(vk_command_buffer);
  branches_directional_light_render_pipeline->BindDescriptorSet(
      vk_command_buffer, 0, RenderLayer::GetPerFrameDescriptorSet()->GetVkDescriptorSet());
  branches_directional_light_render_pipeline->BindDescriptorSet(
      vk_command_buffer, 1, dynamic_strands->strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
  branches_directional_light_render_pipeline->states.ResetAllStates(0);
  branches_directional_light_render_pipeline->states.SetViewportScissor(view.viewport);
  branches_directional_light_render_pipeline->states.ApplyAllStates(vk_command_buffer);

  branches_directional_light_render_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
  const uint32_t count = Platform::DivUp(delaunay_tetrahedrons.size(), task_work_group_invocations);
  vkCmdDrawMeshTasksEXT(vk_command_buffer, count, 1, 1);
  return delaunay_tetrahedrons.size();
}

uint32_t DsAlphaShapeMeshing::RenderBranchesToCameraDeferred(
    const Handle& renderer_handle, int inner_wood_material_index, int snow_material_index,
    const BranchesRenderParameters& render_parameters, const VkCommandBuffer vk_command_buffer,
    const std::vector<VkRenderingAttachmentInfo>& geometry_pass_color_attachment_infos,
    const RenderLayer::DeferredRenderingView& view, VkPolygonMode polygon_mode) const {
  if (!render_parameters.enabled) {
    return 0;
  }
  if (!Platform::Constants::support_mesh_shader) {
    EVOENGINE_LOG("Failed to render! Mesh shader unsupported!")
    return 0;
  }
  // TODO: this needs to be adaptable to the type of meshing
  if (!DsAlphaShapeMeshing::branches_tetrahedron_filtering_pipeline ||
      !DsAlphaShapeMeshing::branches_triangle_filtering_pipeline || !branches_render_pipeline ||
      !branches_render_pipeline->Initialized()) {
    return 0;
  }
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  const uint32_t task_work_group_invocations =
      Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxPreferredTaskWorkGroupInvocations;

  BranchesRenderPushConstant render_push_constant;
  render_push_constant.index1.instance_index =
      Application::GetLayer<RenderLayer>()->GetCurrentRenderInstanceStorage()->GetRenderInstanceIndex(renderer_handle);
  render_push_constant.index2.camera_index = view.camera_index;
  render_push_constant.tetrahedrons_size = delaunay_tetrahedrons.size();
  render_push_constant.alpha = render_parameters.alpha;
  render_push_constant.bifurcation_alpha = render_parameters.bifurcation_alpha;
  render_push_constant.max_dist_squared = render_parameters.max_dist_squared;
  render_push_constant.render_complex = render_parameters.render_complex ? 1 : 0;
  render_push_constant.vertex_colors = render_parameters.vertex_colors;
  render_push_constant.u_multiplier = render_parameters.u_multiplier;
  render_push_constant.v_multiplier = render_parameters.v_multiplier;
  render_push_constant.inner_wood_material_index = inner_wood_material_index;
  render_push_constant.snow_material_index = snow_material_index;
  render_push_constant.global_extrusion_distance = render_parameters.global_extrusion_distance;
  render_push_constant.break_threshold = render_parameters.break_threshold;
  render_push_constant.use_polar_coordinates_for_uv = render_parameters.use_polar_coordinates_for_uv ? 1 : 0;
  branches_render_pipeline->states.ResetAllStates(geometry_pass_color_attachment_infos.size());
  branches_render_pipeline->states.SetViewportScissor(view.viewport);
  branches_render_pipeline->states.polygon_mode = polygon_mode;
  branches_render_pipeline->states.line_width = 2.0f;
  branches_render_pipeline->states.ApplyAllStates(vk_command_buffer);

#ifdef USE_RENDERDOC
  if (rdoc_api) {
    rdoc_api->StartFrameCapture(NULL, NULL);
    EVOENGINE_LOG("RDOC API detected!");
  }
#endif  //  USERENDERDOC

  branches_render_pipeline->Bind(vk_command_buffer);
  branches_render_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                              RenderLayer::GetPerFrameDescriptorSet()->GetVkDescriptorSet());
  branches_render_pipeline->BindDescriptorSet(
      vk_command_buffer, 1, dynamic_strands->strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
  branches_render_pipeline->BindDescriptorSet(vk_command_buffer, 2,
                                              RenderLayer::GetLightingDescriptorSet()->GetVkDescriptorSet());

  branches_render_pipeline->PushConstant(vk_command_buffer, 0, render_push_constant);

  const uint32_t count = Platform::DivUp(delaunay_tetrahedrons.size(), task_work_group_invocations);
  vkCmdDrawMeshTasksEXT(vk_command_buffer, count, 1, 1);
#ifdef USE_RENDERDOC
  if (rdoc_api)
    rdoc_api->EndFrameCapture(NULL, NULL);
#endif
  return dynamic_strands->segments.size();
}
