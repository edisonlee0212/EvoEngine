#include "Application.hpp"
#include "Delaunay.hpp"
#include "DsConstraints.hpp"
#include "DsOperators.hpp"
#include "DynamicTreeStrands.hpp"
#include "RenderParameters.hpp"
#include "Shader.hpp"
#include "Tree.hpp"
using namespace eco_sys_lab_package;

namespace {
std::shared_ptr<GraphicsPipeline> CreateMaskedRawPipeline(const std::shared_ptr<GraphicsPipeline>& opaque,
                                                          const std::filesystem::path& fragment_shader_path) {
  auto pipeline = std::make_shared<GraphicsPipeline>();
  pipeline->vertex_shader = opaque->vertex_shader;
  pipeline->task_shader = opaque->task_shader;
  pipeline->mesh_shader = opaque->mesh_shader;
  pipeline->fragment_shader =
      Shader::CreateTemporary(ShaderType::Fragment, Platform::GetShaderGlobalDefines(), fragment_shader_path);
  pipeline->geometry_type = opaque->geometry_type;
  pipeline->vertex_input_attribute_set = opaque->vertex_input_attribute_set;
  pipeline->vertex_input_enabled = opaque->vertex_input_enabled;
  pipeline->primitive_topology = opaque->primitive_topology;
  pipeline->descriptor_set_layouts = opaque->descriptor_set_layouts;
  pipeline->color_attachment_formats = opaque->color_attachment_formats;
  pipeline->depth_attachment_format = opaque->depth_attachment_format;
  pipeline->stencil_attachment_format = opaque->stencil_attachment_format;
  pipeline->push_constant_ranges = opaque->push_constant_ranges;
  pipeline->Initialize();
  return pipeline;
}
}  // namespace

bool FoliageRenderParameters::DrawGui(const std::shared_ptr<EditorLayer>& editor_layer) {
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
      ShaderType::Task, Platform::GetShaderGlobalDefines(),
      std::filesystem::path("./EcoSysLabResources") / "Shaders/Graphics/Task/DynamicStrands/Rendering/Foliage.slang");
  foliage_point_light_render_pipeline->mesh_shader =
      Shader::CreateTemporary(ShaderType::Mesh, Platform::GetShaderGlobalDefines(),
                              std::filesystem::path("./EcoSysLabResources") /
                                  "Shaders/Graphics/Mesh/DynamicStrands/Rendering/Foliage/PointLightShadowMap.slang");
  foliage_point_light_render_pipeline->fragment_shader =
      Shader::CreateTemporary(ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
                              std::filesystem::path("./EcoSysLabResources") / "Shaders/Graphics/Fragment/Empty.slang");
  foliage_point_light_render_pipeline->geometry_type = GeometryType::Mesh;
  foliage_point_light_render_pipeline->descriptor_set_layouts.emplace_back(
      ApplicationContext::Get().GetLayer<RenderLayer>()->GetPerFrameDescriptorSetLayout());
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
      ShaderType::Task, Platform::GetShaderGlobalDefines(),
      std::filesystem::path("./EcoSysLabResources") / "Shaders/Graphics/Task/DynamicStrands/Rendering/Foliage.slang");
  foliage_spot_light_render_pipeline->mesh_shader =
      Shader::CreateTemporary(ShaderType::Mesh, Platform::GetShaderGlobalDefines(),
                              std::filesystem::path("./EcoSysLabResources") /
                                  "Shaders/Graphics/Mesh/DynamicStrands/Rendering/Foliage/SpotLightShadowMap.slang");
  foliage_spot_light_render_pipeline->fragment_shader =
      Shader::CreateTemporary(ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
                              std::filesystem::path("./EcoSysLabResources") / "Shaders/Graphics/Fragment/Empty.slang");
  foliage_spot_light_render_pipeline->geometry_type = GeometryType::Mesh;
  foliage_spot_light_render_pipeline->descriptor_set_layouts.emplace_back(
      ApplicationContext::Get().GetLayer<RenderLayer>()->GetPerFrameDescriptorSetLayout());
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
      ShaderType::Task, Platform::GetShaderGlobalDefines(),
      std::filesystem::path("./EcoSysLabResources") / "Shaders/Graphics/Task/DynamicStrands/Rendering/Foliage.slang");
  foliage_directional_light_render_pipeline->mesh_shader = Shader::CreateTemporary(
      ShaderType::Mesh, Platform::GetShaderGlobalDefines(),
      std::filesystem::path("./EcoSysLabResources") /
          "Shaders/Graphics/Mesh/DynamicStrands/Rendering/Foliage/DirectionalLightShadowMap.slang");
  foliage_directional_light_render_pipeline->fragment_shader =
      Shader::CreateTemporary(ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
                              std::filesystem::path("./EcoSysLabResources") / "Shaders/Graphics/Fragment/Empty.slang");
  foliage_directional_light_render_pipeline->geometry_type = GeometryType::Mesh;
  foliage_directional_light_render_pipeline->descriptor_set_layouts.emplace_back(
      ApplicationContext::Get().GetLayer<RenderLayer>()->GetPerFrameDescriptorSetLayout());
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
      ShaderType::Task, Platform::GetShaderGlobalDefines(),
      std::filesystem::path("./EcoSysLabResources") / "Shaders/Graphics/Task/DynamicStrands/Rendering/Foliage.slang");
  foliage_render_pipeline->mesh_shader =
      Shader::CreateTemporary(ShaderType::Mesh, Platform::GetShaderGlobalDefines(),
                              std::filesystem::path("./EcoSysLabResources") /
                                  "Shaders/Graphics/Mesh/DynamicStrands/Rendering/Foliage/Rendering.slang");
  foliage_render_pipeline->fragment_shader =
      Shader::CreateTemporary(ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
                              std::filesystem::path("./EcoSysLabResources") /
                                  "Shaders/Graphics/Fragment/DynamicStrands/Rendering/Foliage.slang");
  foliage_render_pipeline->geometry_type = GeometryType::Mesh;
  foliage_render_pipeline->descriptor_set_layouts.emplace_back(
      ApplicationContext::Get().GetLayer<RenderLayer>()->GetPerFrameDescriptorSetLayout());
  foliage_render_pipeline->descriptor_set_layouts.emplace_back(strands_layout);
  foliage_render_pipeline->descriptor_set_layouts.emplace_back(
      ApplicationContext::Get().GetLayer<RenderLayer>()->GetLightingDescriptorSetLayout());
  foliage_render_pipeline->depth_attachment_format = Platform::Constants::render_texture_depth;
  foliage_render_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
  foliage_render_pipeline->color_attachment_formats = {
      Platform::Constants::g_buffer_attribute, Platform::Constants::g_buffer_attribute,
      Platform::Constants::g_buffer_attribute, Platform::Constants::g_buffer_attribute,
      Platform::Constants::g_buffer_utility};
  auto& push_constant_range = foliage_render_pipeline->push_constant_ranges.emplace_back();
  push_constant_range.size = sizeof(FoliageRenderPushConstant);
  push_constant_range.offset = 0;
  push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
  foliage_render_pipeline->Initialize();
  foliage_masked_render_pipeline = CreateMaskedRawPipeline(
      foliage_render_pipeline, std::filesystem::path("./EcoSysLabResources") /
                                   "Shaders/Graphics/Fragment/DynamicStrands/Rendering/FoliageMasked.slang");
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
  foliage_point_light_render_pipeline->DrawMeshTasks(vk_command_buffer, count, 1, 1);
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
  foliage_spot_light_render_pipeline->DrawMeshTasks(vk_command_buffer, count, 1, 1);
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
  foliage_directional_light_render_pipeline->DrawMeshTasks(vk_command_buffer, count, 1, 1);
  return foliage.size();
}

uint32_t DynamicStrands::RenderFoliageToCameraDeferred(
    const Handle& renderer_handle, const FoliageRenderParameters& render_parameters,
    const std::shared_ptr<GraphicsPipeline>& pipeline, const VkCullModeFlags cull_mode,
    VkCommandBuffer vk_command_buffer,
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
      ApplicationContext::Get().GetLayer<RenderLayer>()->GetCurrentRenderInstanceStorage()->GetRenderInstanceIndex(
          renderer_handle);
  push_constant.index2.camera_index = view.camera_index;
  push_constant.leaf_size = foliage.size();
  pipeline->states.ResetAllStates(geometry_pass_color_attachment_infos.size());
  pipeline->states.SetViewportScissor(view.viewport);
  pipeline->states.polygon_mode = render_parameters.wireframe ? VK_POLYGON_MODE_LINE : VK_POLYGON_MODE_FILL;
  pipeline->states.cull_mode = cull_mode;
  pipeline->states.ApplyAllStates(vk_command_buffer);

#ifdef USE_RENDERDOC
  if (rdoc_api) {
    rdoc_api->StartFrameCapture(NULL, NULL);
    EVOENGINE_LOG("RDOC API detected!");
  }
#endif  //  USERENDERDOC
  pipeline->Bind(vk_command_buffer);
  pipeline->BindDescriptorSet(vk_command_buffer, 0, RenderLayer::GetPerFrameDescriptorSet()->GetVkDescriptorSet());
  pipeline->BindDescriptorSet(vk_command_buffer, 1, strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
  pipeline->BindDescriptorSet(vk_command_buffer, 2, RenderLayer::GetLightingDescriptorSet()->GetVkDescriptorSet());
  pipeline->PushConstant(vk_command_buffer, 0, push_constant);

  const uint32_t count = Platform::DivUp(foliage.size(), task_work_group_invocations);
  pipeline->DrawMeshTasks(vk_command_buffer, count, 1, 1);
#ifdef USE_RENDERDOC
  if (rdoc_api)
    rdoc_api->EndFrameCapture(NULL, NULL);
#endif
  return foliage.size();
}
