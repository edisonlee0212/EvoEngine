#include "Application.hpp"
#include "Delaunay.hpp"
#include "DsConstraints.hpp"
#include "DsOperators.hpp"
#include "DynamicTreeStrands.hpp"
#include "RenderParameters.hpp"
#include "Shader.hpp"
#include "Tree.hpp"
using namespace eco_sys_lab_package;

struct SegmentPairsVisualizationRenderPushConstant {
  glm::vec4 min_color = glm::vec4(0.2f);
  glm::vec4 max_color = glm::vec4(1.f);
  glm::vec3 position_scale;
  float padding;

  int32_t camera_index = 0;
  uint32_t segment_pair_size = 0;
  uint32_t render_mode = 2;
  int material_index;
  float multiplier = 1.0f;
  float factor = 1.0f;
};

void DynamicStrands::BuildSegmentPairsRenderingPipeline() {
  // Descriptor set layout
  segment_pairs_visualization_render_pipeline = std::make_shared<GraphicsPipeline>();
  segment_pairs_visualization_render_pipeline->task_shader =
      Shader::CreateTemporary(ShaderType::Task, Platform::GetShaderGlobalDefines(),
                              std::filesystem::path("./EcoSysLabResources") /
                                  "Shaders/Graphics/Task/DynamicStrands/Rendering/SegmentPairs.slang");
  segment_pairs_visualization_render_pipeline->mesh_shader =
      Shader::CreateTemporary(ShaderType::Mesh, Platform::GetShaderGlobalDefines(),
                              std::filesystem::path("./EcoSysLabResources") /
                                  "Shaders/Graphics/Mesh/DynamicStrands/Rendering/SegmentPairs/Rendering.slang");
  segment_pairs_visualization_render_pipeline->fragment_shader =
      Shader::CreateTemporary(ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
                              std::filesystem::path("./EcoSysLabResources") /
                                  "Shaders/Graphics/Fragment/DynamicStrands/Rendering/SegmentPairs.slang");
  segment_pairs_visualization_render_pipeline->geometry_type = GeometryType::Mesh;
  segment_pairs_visualization_render_pipeline->descriptor_set_layouts.emplace_back(
      ApplicationContext::Get().GetLayer<RenderLayer>()->GetPerFrameDescriptorSetLayout());
  segment_pairs_visualization_render_pipeline->descriptor_set_layouts.emplace_back(strands_layout);
  segment_pairs_visualization_render_pipeline->descriptor_set_layouts.emplace_back(
      ApplicationContext::Get().GetLayer<RenderLayer>()->GetLightingDescriptorSetLayout());
  segment_pairs_visualization_render_pipeline->depth_attachment_format = Platform::Constants::render_texture_depth;
  segment_pairs_visualization_render_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
  segment_pairs_visualization_render_pipeline->color_attachment_formats = {1,
                                                                           Platform::Constants::render_texture_color};
  auto& segment_pair_push_constant = segment_pairs_visualization_render_pipeline->push_constant_ranges.emplace_back();
  segment_pair_push_constant.size = sizeof(SegmentPairsVisualizationRenderPushConstant);
  segment_pair_push_constant.offset = 0;
  segment_pair_push_constant.stageFlags = VK_SHADER_STAGE_ALL;
  segment_pairs_visualization_render_pipeline->Initialize();
}

bool SegmentPairsRenderParameters::DrawGui(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::DragFloat3("Position scale", &position_scale.x, 0.1f, 0.1f, 100.f)) {
    changed = true;
  }
  if (ImGui::DragFloat("Thickness multiplier", &thickness_multiplier, 0.1f, 0.1f, 10.f)) {
    changed = true;
  }

  return changed;
}

uint32_t DynamicStrands::RenderSegmentPairsToCameraForward(
    int material_index, const DynamicStrandsInitializeParameters& initialize_parameters,
    const SegmentPairsRenderParameters& render_parameters, VkCommandBuffer vk_command_buffer,
    const std::shared_ptr<Camera>& target_camera, const RenderLayer::ForwardRenderingView& view) const {
  if (!render_parameters.enabled) {
    return 0;
  }
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  const uint32_t task_work_group_invocations =
      Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxPreferredTaskWorkGroupInvocations;
  SegmentPairsVisualizationRenderPushConstant segment_pair_push_constant;
  segment_pair_push_constant.render_mode = render_parameters.segment_pair_render_mode;
  segment_pair_push_constant.camera_index = view.camera_index;
  segment_pair_push_constant.min_color = render_parameters.segment_pair_render_mode == 0
                                             ? render_parameters.segment_pair_color_main
                                             : render_parameters.segment_pair_color_min;
  segment_pair_push_constant.max_color = render_parameters.segment_pair_color_max;
  segment_pair_push_constant.multiplier = render_parameters.segment_pair_radius_multiplier;
  segment_pair_push_constant.material_index = material_index;
  segment_pair_push_constant.segment_pair_size = segment_pairs.size();
  segment_pair_push_constant.multiplier = render_parameters.thickness_multiplier;
  segment_pair_push_constant.position_scale = render_parameters.position_scale;

  // Build node graph input (Note: input does not exist here)
  StrengthGraph::Input strength_input;
  BiologicalPropertiesGraph::Input biological_properties_input;

  switch (static_cast<DynamicStrandsVisualizationParameters::SegmentPairRenderMode>(
      render_parameters.segment_pair_render_mode)) {
    case DynamicStrandsVisualizationParameters::SegmentPairRenderMode::BendingLimit: {
      glm::vec2 bending_strength = initialize_parameters.strength_graph.GetBendingStrength(strength_input);
      segment_pair_push_constant.factor =
          initialize_parameters.biological_properties_graph.GetValues(biological_properties_input)
              .trunk_additional_strength_factor +
          glm::max(bending_strength.x, bending_strength.y);
      break;
    }
    case DynamicStrandsVisualizationParameters::SegmentPairRenderMode::TwistLimit: {
      glm::vec2 twisting_strength = initialize_parameters.strength_graph.GetTwistingStrength(strength_input);
      segment_pair_push_constant.factor =
          initialize_parameters.biological_properties_graph.GetValues(biological_properties_input)
              .trunk_additional_strength_factor +
          glm::max(twisting_strength.x, twisting_strength.y);
      break;
    }
    case DynamicStrandsVisualizationParameters::SegmentPairRenderMode::BundleLimit: {
      glm::vec2 bundle_strength = initialize_parameters.strength_graph.GetBundleStrength(strength_input);
      segment_pair_push_constant.factor =
          initialize_parameters.biological_properties_graph.GetValues(biological_properties_input)
              .trunk_additional_strength_factor +
          glm::max(bundle_strength.x, bundle_strength.y);
      break;
    }
    case DynamicStrandsVisualizationParameters::SegmentPairRenderMode::ConnectivityLimit: {
      glm::vec2 connectivity_strength = initialize_parameters.strength_graph.GetConnectivityStrength(strength_input);
      segment_pair_push_constant.factor =
          initialize_parameters.biological_properties_graph.GetValues(biological_properties_input)
              .trunk_additional_strength_factor +
          glm::max(connectivity_strength.x, connectivity_strength.y);
      break;
    }
    default: {
      segment_pair_push_constant.factor = 1.f;
      break;
    }
  }
#pragma region Viewport and scissor
  VkViewport viewport;
  viewport.x = 0.0f;
  viewport.y = 0.0f;
  viewport.width = target_camera->GetSize().x;
  viewport.height = target_camera->GetSize().y;
  viewport.minDepth = 0.0f;
  viewport.maxDepth = 1.0f;
  VkRect2D scissor;
  scissor.offset = {0, 0};
  scissor.extent.width = target_camera->GetSize().x;
  scissor.extent.height = target_camera->GetSize().y;
#pragma endregion
  // 1 here means we only have 1 color attachment. (For deferred shading we will have multiple attachments for
  // GBuffer)

  segment_pairs_visualization_render_pipeline->states.ResetAllStates(1);
  segment_pairs_visualization_render_pipeline->states.view_port = viewport;
  segment_pairs_visualization_render_pipeline->states.scissor = scissor;
  segment_pairs_visualization_render_pipeline->states.polygon_mode = VK_POLYGON_MODE_FILL;
  segment_pairs_visualization_render_pipeline->states.color_blend_attachment_states[0].blendEnable = true;

  segment_pairs_visualization_render_pipeline->states.ApplyAllStates(vk_command_buffer);
  target_camera->GetRenderTexture()->Render(
      vk_command_buffer, VK_ATTACHMENT_LOAD_OP_LOAD, VK_ATTACHMENT_STORE_OP_STORE, [&] {
        segment_pairs_visualization_render_pipeline->Bind(vk_command_buffer);
        segment_pairs_visualization_render_pipeline->BindDescriptorSet(
            vk_command_buffer, 0, RenderLayer::GetPerFrameDescriptorSet()->GetVkDescriptorSet());
        segment_pairs_visualization_render_pipeline->BindDescriptorSet(
            vk_command_buffer, 1, strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
        segment_pairs_visualization_render_pipeline->BindDescriptorSet(
            vk_command_buffer, 2, RenderLayer::GetLightingDescriptorSet()->GetVkDescriptorSet());
        segment_pairs_visualization_render_pipeline->PushConstant(vk_command_buffer, 0, segment_pair_push_constant);
        const uint32_t count = Platform::DivUp(segment_pairs.size(), task_work_group_invocations);
        segment_pairs_visualization_render_pipeline->DrawMeshTasks(vk_command_buffer, count, 1, 1);
      });

  return segment_pairs.size();
}