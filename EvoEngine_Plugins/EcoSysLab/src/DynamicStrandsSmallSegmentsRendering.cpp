
#include "Delaunay.hpp"
#include "DsConstraints.hpp"
#include "DsOperators.hpp"
#include "DynamicTreeStrands.hpp"
#include "Shader.hpp"
#include "Tree.hpp"
using namespace eco_sys_lab_plugin;

bool DynamicStrands::SmallSegmentsRenderParameters::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::Checkbox("Cast Shadow", &cast_shadow)) {
    changed = true;
  }
  if (ImGui::Checkbox("Wireframe", &wireframe)) {
    changed = true;
  }
  if (ImGui::DragFloat("Thickness multiplier", &thickness_multiplier, 0.1f, 0.1f, 10.f)) {
    changed = true;
  }
  return changed;
}

struct SmallSegmentsRenderPushConstant {
  union Index1 {
    int instance_index;
    int sub_light_index;
  } index1;
  union Index2 {
    int camera_index;
    int light_index;
  } index2;
  uint32_t uniform_particle_size;
  float thickness_multiplier;
};

void DynamicStrands::BuildSmallSegmentsRenderingPipelines() {
  // Descriptor set layout
  small_segments_point_light_render_pipeline = std::make_shared<GraphicsPipeline>();
  small_segments_point_light_render_pipeline->task_shader =
      Shader::CreateTemporary(ShaderType::Task, Platform::Constants::shader_global_defines,
                              std::filesystem::path("./EcoSysLabResources") /
                                  "Shaders/Graphics/Task/DynamicStrands/Rendering/SmallSegments.task");
  small_segments_point_light_render_pipeline->mesh_shader = Shader::CreateTemporary(
      ShaderType::Mesh, Platform::Constants::shader_global_defines,
      std::filesystem::path("./EcoSysLabResources") /
          "Shaders/Graphics/Mesh/DynamicStrands/Rendering/SmallSegments/PointLightShadowMap.mesh");
  small_segments_point_light_render_pipeline->fragment_shader =
      Shader::CreateTemporary(ShaderType::Fragment, Platform::Constants::shader_global_defines,
                              std::filesystem::path("./EcoSysLabResources") / "Shaders/Graphics/Fragment/Empty.frag");
  small_segments_point_light_render_pipeline->geometry_type = GeometryType::Mesh;
  small_segments_point_light_render_pipeline->descriptor_set_layouts.emplace_back(RenderLayer::per_frame_layout);
  small_segments_point_light_render_pipeline->descriptor_set_layouts.emplace_back(strands_layout);
  small_segments_point_light_render_pipeline->depth_attachment_format = Platform::Constants::shadow_map;
  small_segments_point_light_render_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
  auto& point_light_push_constant_range =
      small_segments_point_light_render_pipeline->push_constant_ranges.emplace_back();
  point_light_push_constant_range.size = sizeof(SmallSegmentsRenderPushConstant);
  point_light_push_constant_range.offset = 0;
  point_light_push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
  small_segments_point_light_render_pipeline->Initialize();
  // Descriptor set layout
  small_segments_spot_light_render_pipeline = std::make_shared<GraphicsPipeline>();
  small_segments_spot_light_render_pipeline->task_shader =
      Shader::CreateTemporary(ShaderType::Task, Platform::Constants::shader_global_defines,
                              std::filesystem::path("./EcoSysLabResources") /
                                  "Shaders/Graphics/Task/DynamicStrands/Rendering/SmallSegments.task");
  small_segments_spot_light_render_pipeline->mesh_shader = Shader::CreateTemporary(
      ShaderType::Mesh, Platform::Constants::shader_global_defines,
      std::filesystem::path("./EcoSysLabResources") /
          "Shaders/Graphics/Mesh/DynamicStrands/Rendering/SmallSegments/SpotLightShadowMap.mesh");
  small_segments_spot_light_render_pipeline->fragment_shader =
      Shader::CreateTemporary(ShaderType::Fragment, Platform::Constants::shader_global_defines,
                              std::filesystem::path("./EcoSysLabResources") / "Shaders/Graphics/Fragment/Empty.frag");
  small_segments_spot_light_render_pipeline->geometry_type = GeometryType::Mesh;
  small_segments_spot_light_render_pipeline->descriptor_set_layouts.emplace_back(RenderLayer::per_frame_layout);
  small_segments_spot_light_render_pipeline->descriptor_set_layouts.emplace_back(strands_layout);
  small_segments_spot_light_render_pipeline->depth_attachment_format = Platform::Constants::shadow_map;
  small_segments_spot_light_render_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
  auto& spot_light_push_constant_range = small_segments_spot_light_render_pipeline->push_constant_ranges.emplace_back();
  spot_light_push_constant_range.size = sizeof(SmallSegmentsRenderPushConstant);
  spot_light_push_constant_range.offset = 0;
  spot_light_push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
  small_segments_spot_light_render_pipeline->Initialize();
  // Descriptor set layout
  small_segments_directional_light_render_pipeline = std::make_shared<GraphicsPipeline>();
  small_segments_directional_light_render_pipeline->task_shader =
      Shader::CreateTemporary(ShaderType::Task, Platform::Constants::shader_global_defines,
                              std::filesystem::path("./EcoSysLabResources") /
                                  "Shaders/Graphics/Task/DynamicStrands/Rendering/SmallSegments.task");
  small_segments_directional_light_render_pipeline->mesh_shader = Shader::CreateTemporary(
      ShaderType::Mesh, Platform::Constants::shader_global_defines,
      std::filesystem::path("./EcoSysLabResources") /
          "Shaders/Graphics/Mesh/DynamicStrands/Rendering/SmallSegments/DirectionalLightShadowMap.mesh");
  small_segments_directional_light_render_pipeline->fragment_shader =
      Shader::CreateTemporary(ShaderType::Fragment, Platform::Constants::shader_global_defines,
                              std::filesystem::path("./EcoSysLabResources") / "Shaders/Graphics/Fragment/Empty.frag");
  small_segments_directional_light_render_pipeline->geometry_type = GeometryType::Mesh;
  small_segments_directional_light_render_pipeline->descriptor_set_layouts.emplace_back(RenderLayer::per_frame_layout);
  small_segments_directional_light_render_pipeline->descriptor_set_layouts.emplace_back(strands_layout);
  small_segments_directional_light_render_pipeline->depth_attachment_format = Platform::Constants::shadow_map;
  small_segments_directional_light_render_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
  auto& directional_light_push_constant_range =
      small_segments_directional_light_render_pipeline->push_constant_ranges.emplace_back();
  directional_light_push_constant_range.size = sizeof(SmallSegmentsRenderPushConstant);
  directional_light_push_constant_range.offset = 0;
  directional_light_push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
  small_segments_directional_light_render_pipeline->Initialize();
  // Descriptor set layout
  small_segments_render_pipeline = std::make_shared<GraphicsPipeline>();
  small_segments_render_pipeline->task_shader =
      Shader::CreateTemporary(ShaderType::Task, Platform::Constants::shader_global_defines,
                              std::filesystem::path("./EcoSysLabResources") /
                                  "Shaders/Graphics/Task/DynamicStrands/Rendering/SmallSegments.task");
  small_segments_render_pipeline->mesh_shader =
      Shader::CreateTemporary(ShaderType::Mesh, Platform::Constants::shader_global_defines,
                              std::filesystem::path("./EcoSysLabResources") /
                                  "Shaders/Graphics/Mesh/DynamicStrands/Rendering/SmallSegments/Rendering.mesh");
  small_segments_render_pipeline->fragment_shader =
      Shader::CreateTemporary(ShaderType::Fragment, Platform::Constants::shader_global_defines,
                              std::filesystem::path("./EcoSysLabResources") /
                                  "Shaders/Graphics/Fragment/DynamicStrands/Rendering/SmallSegments.frag");
  small_segments_render_pipeline->geometry_type = GeometryType::Mesh;
  small_segments_render_pipeline->descriptor_set_layouts.emplace_back(RenderLayer::per_frame_layout);
  small_segments_render_pipeline->descriptor_set_layouts.emplace_back(strands_layout);
  small_segments_render_pipeline->descriptor_set_layouts.emplace_back(RenderLayer::lighting_layout);
  small_segments_render_pipeline->depth_attachment_format = Platform::Constants::render_texture_depth;
  small_segments_render_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
  small_segments_render_pipeline->color_attachment_formats = {2, Platform::Constants::g_buffer_color};
  auto& push_constant_range = small_segments_render_pipeline->push_constant_ranges.emplace_back();
  push_constant_range.size = sizeof(SmallSegmentsRenderPushConstant);
  push_constant_range.offset = 0;
  push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
  small_segments_render_pipeline->Initialize();
}

uint32_t DynamicStrands::RenderSmallSegmentsToPointLightShadowMap(
    const SmallSegmentsRenderParameters& render_parameters, const VkCommandBuffer vk_command_buffer,
    const RenderLayer::PointLightShadowMapView& view) const {
  if (!render_parameters.enabled || !render_parameters.cast_shadow) {
    return 0;
  }
  const uint32_t task_work_group_invocations =
      Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxPreferredTaskWorkGroupInvocations;
  SmallSegmentsRenderPushConstant push_constant;
  push_constant.index1.sub_light_index = view.face_index;
  push_constant.index2.light_index = view.light_index;
  push_constant.uniform_particle_size = uniform_particles.size();
  push_constant.thickness_multiplier = render_parameters.thickness_multiplier;

  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  small_segments_point_light_render_pipeline->Bind(vk_command_buffer);
  small_segments_point_light_render_pipeline->BindDescriptorSet(
      vk_command_buffer, 0, RenderLayer::GetPerFrameDescriptorSet()->GetVkDescriptorSet());
  small_segments_point_light_render_pipeline->BindDescriptorSet(
      vk_command_buffer, 1, strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
  small_segments_point_light_render_pipeline->states.ResetAllStates(0);
  small_segments_point_light_render_pipeline->states.SetViewportScissor(view.viewport);
  small_segments_point_light_render_pipeline->states.ApplyAllStates(vk_command_buffer);

  small_segments_point_light_render_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
  const uint32_t count = Platform::DivUp(uniform_particles.size(), task_work_group_invocations);
  vkCmdDrawMeshTasksEXT(vk_command_buffer, count, 1, 1);
  return uniform_particles.size();
}

uint32_t DynamicStrands::RenderSmallSegmentsToSpotLightShadowMap(
    const SmallSegmentsRenderParameters& render_parameters, VkCommandBuffer vk_command_buffer,
    const RenderLayer::SpotLightShadowMapView& view) const {
  if (!render_parameters.enabled || !render_parameters.cast_shadow) {
    return 0;
  }
  const uint32_t task_work_group_invocations =
      Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxPreferredTaskWorkGroupInvocations;
  SmallSegmentsRenderPushConstant push_constant;
  push_constant.index1.sub_light_index = 0;
  push_constant.index2.light_index = view.light_index;
  push_constant.uniform_particle_size = uniform_particles.size();
  push_constant.thickness_multiplier = render_parameters.thickness_multiplier;

  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  small_segments_spot_light_render_pipeline->Bind(vk_command_buffer);
  small_segments_spot_light_render_pipeline->BindDescriptorSet(
      vk_command_buffer, 0, RenderLayer::GetPerFrameDescriptorSet()->GetVkDescriptorSet());
  small_segments_spot_light_render_pipeline->BindDescriptorSet(
      vk_command_buffer, 1, strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
  small_segments_spot_light_render_pipeline->states.ResetAllStates(0);
  small_segments_spot_light_render_pipeline->states.SetViewportScissor(view.viewport);
  small_segments_spot_light_render_pipeline->states.ApplyAllStates(vk_command_buffer);

  small_segments_spot_light_render_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
  const uint32_t count = Platform::DivUp(uniform_particles.size(), task_work_group_invocations);
  vkCmdDrawMeshTasksEXT(vk_command_buffer, count, 1, 1);
  return uniform_particles.size();
}

uint32_t DynamicStrands::RenderSmallSegmentsToDirectionalLightShadowMap(
    const SmallSegmentsRenderParameters& render_parameters, VkCommandBuffer vk_command_buffer,
    const RenderLayer::DirectionalLightShadowMapView& view) const {
  if (!render_parameters.enabled || !render_parameters.cast_shadow) {
    return 0;
  }
  const uint32_t task_work_group_invocations =
      Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxPreferredTaskWorkGroupInvocations;
  SmallSegmentsRenderPushConstant push_constant;
  push_constant.index1.sub_light_index = view.split_index;
  push_constant.index2.light_index = view.light_index;
  push_constant.uniform_particle_size = uniform_particles.size();
  push_constant.thickness_multiplier = render_parameters.thickness_multiplier;

  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  small_segments_directional_light_render_pipeline->Bind(vk_command_buffer);
  small_segments_directional_light_render_pipeline->BindDescriptorSet(
      vk_command_buffer, 0, RenderLayer::GetPerFrameDescriptorSet()->GetVkDescriptorSet());
  small_segments_directional_light_render_pipeline->BindDescriptorSet(
      vk_command_buffer, 1, strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
  small_segments_directional_light_render_pipeline->states.ResetAllStates(0);
  small_segments_directional_light_render_pipeline->states.SetViewportScissor(view.viewport);
  small_segments_directional_light_render_pipeline->states.ApplyAllStates(vk_command_buffer);

  small_segments_directional_light_render_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
  const uint32_t count = Platform::DivUp(uniform_particles.size(), task_work_group_invocations);
  vkCmdDrawMeshTasksEXT(vk_command_buffer, count, 1, 1);
  return uniform_particles.size();
}

uint32_t DynamicStrands::RenderSmallSegmentsToCameraDeferred(
    const Handle& renderer_handle, const SmallSegmentsRenderParameters& render_parameters,
    const VkCommandBuffer vk_command_buffer,
    const std::vector<VkRenderingAttachmentInfo>& geometry_pass_color_attachment_infos,
    const RenderLayer::DeferredRenderingView& view) const {
  if (!render_parameters.enabled) {
    return 0;
  }
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  const uint32_t task_work_group_invocations =
      Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxPreferredTaskWorkGroupInvocations;
  SmallSegmentsRenderPushConstant push_constant;
  push_constant.index1.instance_index =
      Application::GetLayer<RenderLayer>()->GetCurrentRenderInstanceStorage()->GetRenderInstanceIndex(renderer_handle);
  push_constant.index2.camera_index = view.camera_index;
  push_constant.uniform_particle_size = uniform_particles.size();
  push_constant.thickness_multiplier = render_parameters.thickness_multiplier;

  small_segments_render_pipeline->states.ResetAllStates(geometry_pass_color_attachment_infos.size());
  small_segments_render_pipeline->states.SetViewportScissor(view.viewport);
  small_segments_render_pipeline->states.polygon_mode =
      render_parameters.wireframe ? VK_POLYGON_MODE_LINE : VK_POLYGON_MODE_FILL;
  small_segments_render_pipeline->states.ApplyAllStates(vk_command_buffer);

#ifdef USE_RENDERDOC
  if (rdoc_api) {
    rdoc_api->StartFrameCapture(NULL, NULL);
    EVOENGINE_LOG("RDOC API detected!");
  }
#endif  //  USERENDERDOC
  small_segments_render_pipeline->Bind(vk_command_buffer);
  small_segments_render_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                                    RenderLayer::GetPerFrameDescriptorSet()->GetVkDescriptorSet());
  small_segments_render_pipeline->BindDescriptorSet(vk_command_buffer, 1,
                                                    strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
  small_segments_render_pipeline->BindDescriptorSet(vk_command_buffer, 2,
                                                    RenderLayer::GetLightingDescriptorSet()->GetVkDescriptorSet());
  small_segments_render_pipeline->PushConstant(vk_command_buffer, 0, push_constant);

  const uint32_t count = Platform::DivUp(uniform_particles.size(), task_work_group_invocations);
  vkCmdDrawMeshTasksEXT(vk_command_buffer, count, 1, 1);
#ifdef USE_RENDERDOC
  if (rdoc_api)
    rdoc_api->EndFrameCapture(NULL, NULL);
#endif
  return uniform_particles.size();
}