#include "Application.hpp"
#include "DynamicStrands.hpp"
#include "Shader.hpp"
using namespace eco_sys_lab_package;

void DynamicStrands::Visualize(const std::shared_ptr<Camera>& target_camera,
                               const DynamicStrandsInitializeParameters& initialize_parameters,
                               const DynamicStrandsVisualizationParameters& visualization_parameters) const {
  if (!Platform::GetInstance().GetCapabilities().support_mesh_shader) {
    EVOENGINE_LOG("Failed to render! Mesh shader unsupported!")
    return;
  }
  const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>();
  if (!render_layer) {
    EVOENGINE_LOG("Failed to render! RenderLayer not present!")
    return;
  }
  const auto current_frame_index = Platform::GetCurrentFrameIndex();

  struct SegmentRenderPushConstant {
    glm::vec4 min_color = glm::vec4(0.2f);
    glm::vec4 max_color = glm::vec4(1.f);

    uint32_t camera_index = 0;
    uint32_t strand_segment_size = 0;
    uint32_t render_mode = 2;
    float multiplier = 1.0f;
    float factor = 1.0f;
    float length_multiplier = 1.0f;
  };

  if (!segment_visualization_render_pipeline) {
    // Load shader
    const auto task_shader = std::make_shared<Shader>();
    task_shader->TryCompile(ShaderType::Task, Platform::GetShaderGlobalDefines(),
                            std::filesystem::path("./EcoSysLabResources") /
                                "Shaders/Graphics/Task/DynamicStrands/Visualization/Segments.slang");
    const auto mesh_shader = std::make_shared<Shader>();
    mesh_shader->TryCompile(ShaderType::Mesh, Platform::GetShaderGlobalDefines(),
                            std::filesystem::path("./EcoSysLabResources") /
                                "Shaders/Graphics/Mesh/DynamicStrands/Visualization/Segments.slang");
    const auto frag_shader = std::make_shared<Shader>();
    frag_shader->TryCompile(
        ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./EcoSysLabResources") / "Shaders/Graphics/Fragment/DynamicStrands/Visualization.slang");
    // Descriptor set layout
    segment_visualization_render_pipeline = std::make_shared<GraphicsPipeline>();
    segment_visualization_render_pipeline->task_shader = task_shader;
    segment_visualization_render_pipeline->mesh_shader = mesh_shader;

    segment_visualization_render_pipeline->fragment_shader = frag_shader;
    segment_visualization_render_pipeline->geometry_type = GeometryType::Mesh;

    segment_visualization_render_pipeline->descriptor_set_layouts.emplace_back(
        ApplicationContext::Get().GetLayer<RenderLayer>()->GetPerFrameDescriptorSetLayout());
    segment_visualization_render_pipeline->descriptor_set_layouts.emplace_back(strands_layout);
    segment_visualization_render_pipeline->depth_attachment_format = Platform::Constants::render_texture_depth;
    segment_visualization_render_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    segment_visualization_render_pipeline->color_attachment_formats = {1, Platform::Constants::render_texture_color};

    auto& push_constant_range = segment_visualization_render_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(SegmentRenderPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;

    segment_visualization_render_pipeline->Initialize();
  }
  struct SegmentPairRenderPushConstant {
    glm::vec4 min_color = glm::vec4(0.2f);
    glm::vec4 max_color = glm::vec4(1.f);

    uint32_t camera_index = 0;
    uint32_t strand_segment_pair_size = 0;
    uint32_t render_mode = 2;
    float multiplier = 1.0f;
    float factor = 1.0f;
  };

  if (!segment_pairs_visualization_render_pipeline) {
    // Load shader
    const auto task_shader = std::make_shared<Shader>();
    task_shader->TryCompile(ShaderType::Task, Platform::GetShaderGlobalDefines(),
                            std::filesystem::path("./EcoSysLabResources") /
                                "Shaders/Graphics/Task/DynamicStrands/Visualization/SegmentPairs.slang");
    const auto mesh_shader = std::make_shared<Shader>();
    mesh_shader->TryCompile(ShaderType::Mesh, Platform::GetShaderGlobalDefines(),
                            std::filesystem::path("./EcoSysLabResources") /
                                "Shaders/Graphics/Mesh/DynamicStrands/Visualization/SegmentPairs.slang");
    const auto frag_shader = std::make_shared<Shader>();
    frag_shader->TryCompile(
        ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./EcoSysLabResources") / "Shaders/Graphics/Fragment/DynamicStrands/Visualization.slang");
    // Descriptor set layout
    segment_pairs_visualization_render_pipeline = std::make_shared<GraphicsPipeline>();
    segment_pairs_visualization_render_pipeline->task_shader = task_shader;
    segment_pairs_visualization_render_pipeline->mesh_shader = mesh_shader;

    segment_pairs_visualization_render_pipeline->fragment_shader = frag_shader;
    segment_pairs_visualization_render_pipeline->geometry_type = GeometryType::Mesh;

    segment_pairs_visualization_render_pipeline->descriptor_set_layouts.emplace_back(
        ApplicationContext::Get().GetLayer<RenderLayer>()->GetPerFrameDescriptorSetLayout());
    segment_pairs_visualization_render_pipeline->descriptor_set_layouts.emplace_back(strands_layout);
    segment_pairs_visualization_render_pipeline->depth_attachment_format = Platform::Constants::render_texture_depth;
    segment_pairs_visualization_render_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    segment_pairs_visualization_render_pipeline->color_attachment_formats = {1,
                                                                             Platform::Constants::render_texture_color};

    auto& push_constant_range = segment_pairs_visualization_render_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(SegmentPairRenderPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;

    segment_pairs_visualization_render_pipeline->Initialize();
  }

  struct FoliageRenderPushConstant {
    glm::vec4 min_color = glm::vec4(0.2f);
    glm::vec4 max_color = glm::vec4(1.f);

    uint32_t camera_index = 0;
    uint32_t foliage_size = 0;
    uint32_t render_mode = 0;
  };

  if (!foliage_visualization_render_pipeline) {
    // Load shader
    const auto task_shader = std::make_shared<Shader>();
    task_shader->TryCompile(ShaderType::Task, Platform::GetShaderGlobalDefines(),
                            std::filesystem::path("./EcoSysLabResources") /
                                "Shaders/Graphics/Task/DynamicStrands/Visualization/Foliage.slang");
    const auto mesh_shader = std::make_shared<Shader>();
    mesh_shader->TryCompile(ShaderType::Mesh, Platform::GetShaderGlobalDefines(),
                            std::filesystem::path("./EcoSysLabResources") /
                                "Shaders/Graphics/Mesh/DynamicStrands/Visualization/Foliage.slang");
    const auto frag_shader = std::make_shared<Shader>();
    frag_shader->TryCompile(
        ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./EcoSysLabResources") / "Shaders/Graphics/Fragment/DynamicStrands/Visualization.slang");
    // Descriptor set layout
    foliage_visualization_render_pipeline = std::make_shared<GraphicsPipeline>();
    foliage_visualization_render_pipeline->task_shader = task_shader;
    foliage_visualization_render_pipeline->mesh_shader = mesh_shader;

    foliage_visualization_render_pipeline->fragment_shader = frag_shader;
    foliage_visualization_render_pipeline->geometry_type = GeometryType::Mesh;

    foliage_visualization_render_pipeline->descriptor_set_layouts.emplace_back(
        ApplicationContext::Get().GetLayer<RenderLayer>()->GetPerFrameDescriptorSetLayout());
    foliage_visualization_render_pipeline->descriptor_set_layouts.emplace_back(strands_layout);
    foliage_visualization_render_pipeline->depth_attachment_format = Platform::Constants::render_texture_depth;
    foliage_visualization_render_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    foliage_visualization_render_pipeline->color_attachment_formats = {1, Platform::Constants::render_texture_color};

    auto& push_constant_range = foliage_visualization_render_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(FoliageRenderPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;

    foliage_visualization_render_pipeline->Initialize();
  }

  const uint32_t task_work_group_invocations =
      Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxPreferredTaskWorkGroupInvocations;

  SegmentRenderPushConstant segment_push_constant;
  segment_push_constant.render_mode = visualization_parameters.segment_render_mode;
  segment_push_constant.min_color = visualization_parameters.segment_render_mode == 0
                                        ? visualization_parameters.segment_color_main
                                        : visualization_parameters.segment_color_min;
  segment_push_constant.max_color = visualization_parameters.segment_color_max;
  segment_push_constant.camera_index =
      render_layer->GetCurrentRenderInstanceStorage()->GetCameraIndex(target_camera->GetHandle());
  segment_push_constant.multiplier = visualization_parameters.segment_radius_multiplier;

  // build input for graphs (Note: input does not exist here)
  StrengthGraph::Input strength_input;
  BiologicalPropertiesGraph::Input biological_properties_input;

  segment_push_constant.length_multiplier = visualization_parameters.segment_length_multiplier;
  //(Implement Fungus Propogation)
  switch (static_cast<DynamicStrandsVisualizationParameters::SegmentRenderMode>(
      visualization_parameters.segment_render_mode)) {
    case DynamicStrandsVisualizationParameters::SegmentRenderMode::BoundaryDistance: {
      segment_push_constant.factor = visualization_parameters.segment_boundary_distance_modular;
      break;
    }
    case DynamicStrandsVisualizationParameters::SegmentRenderMode::StretchShearLimit: {
      StrengthGraph::Output::ShearStretchStrengthType shear_stretch_strength =
          initialize_parameters.strength_graph.GetShearStretchStrength(strength_input);
      float trunk_additional_strength_factor =
          initialize_parameters.biological_properties_graph.GetValues(biological_properties_input)
              .trunk_additional_strength_factor;
      segment_push_constant.factor =
          trunk_additional_strength_factor + glm::max(shear_stretch_strength.x, shear_stretch_strength.y);
      break;
    }
    default: {
      segment_push_constant.factor = visualization_parameters.general_factor;
      break;
    }
  }

  segment_push_constant.strand_segment_size = segments.size();

  SegmentPairRenderPushConstant segment_pair_push_constant;
  segment_pair_push_constant.render_mode = visualization_parameters.segment_pair_render_mode;
  segment_pair_push_constant.min_color = visualization_parameters.segment_pair_render_mode == 0
                                             ? visualization_parameters.segment_pair_color_main
                                             : visualization_parameters.segment_pair_color_min;
  segment_pair_push_constant.max_color = visualization_parameters.segment_pair_color_max;
  segment_pair_push_constant.camera_index =
      render_layer->GetCurrentRenderInstanceStorage()->GetCameraIndex(target_camera->GetHandle());
  segment_pair_push_constant.multiplier = visualization_parameters.segment_pair_radius_multiplier;
  segment_pair_push_constant.strand_segment_pair_size = segment_pairs.size();
  float trunk_additional_strength_factor =
      initialize_parameters.biological_properties_graph.GetValues(biological_properties_input)
          .trunk_additional_strength_factor;
  switch (static_cast<DynamicStrandsVisualizationParameters::SegmentPairRenderMode>(
      visualization_parameters.segment_pair_render_mode)) {
    case DynamicStrandsVisualizationParameters::SegmentPairRenderMode::BendingLimit: {
      glm::vec2 bending_strength = initialize_parameters.strength_graph.GetBendingStrength(strength_input);
      segment_pair_push_constant.factor =
          trunk_additional_strength_factor + glm::max(bending_strength.x, bending_strength.y);
      break;
    }
    case DynamicStrandsVisualizationParameters::SegmentPairRenderMode::TwistLimit: {
      glm::vec2 twisting_strength = initialize_parameters.strength_graph.GetTwistingStrength(strength_input);
      segment_pair_push_constant.factor =
          trunk_additional_strength_factor + glm::max(twisting_strength.x, twisting_strength.y);
      break;
    }
    case DynamicStrandsVisualizationParameters::SegmentPairRenderMode::BundleLimit: {
      glm::vec2 bundle_strength = initialize_parameters.strength_graph.GetBundleStrength(strength_input);
      segment_pair_push_constant.factor =
          trunk_additional_strength_factor + glm::max(bundle_strength.x, bundle_strength.y);
      break;
    }
    case DynamicStrandsVisualizationParameters::SegmentPairRenderMode::ConnectivityLimit: {
      glm::vec2 connectivity_strength = initialize_parameters.strength_graph.GetConnectivityStrength(strength_input);
      segment_pair_push_constant.factor =
          trunk_additional_strength_factor + glm::max(connectivity_strength.x, connectivity_strength.y);
      break;
    }
    default: {
      segment_pair_push_constant.factor = 1.f;
      break;
    }
  }

  FoliageRenderPushConstant foliage_push_constant;
  foliage_push_constant.render_mode = visualization_parameters.foliage_render_mode;
  foliage_push_constant.min_color = visualization_parameters.foliage_render_mode == 0
                                        ? visualization_parameters.foliage_color_main
                                        : visualization_parameters.foliage_color_min;
  foliage_push_constant.max_color = visualization_parameters.foliage_color_max;
  foliage_push_constant.camera_index =
      render_layer->GetCurrentRenderInstanceStorage()->GetCameraIndex(target_camera->GetHandle());
  foliage_push_constant.foliage_size = foliage.size();

  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
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

    if (visualization_parameters.render_segment_pairs) {
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
            segment_pairs_visualization_render_pipeline->PushConstant(vk_command_buffer, 0, segment_pair_push_constant);
            const uint32_t count = Platform::DivUp(segment_pairs.size(), task_work_group_invocations);
            segment_pairs_visualization_render_pipeline->DrawMeshTasks(vk_command_buffer, count, 1, 1);
          });
    }

    if (visualization_parameters.render_segments) {
      segment_visualization_render_pipeline->states.ResetAllStates(1);
      segment_visualization_render_pipeline->states.view_port = viewport;
      segment_visualization_render_pipeline->states.scissor = scissor;
      segment_visualization_render_pipeline->states.polygon_mode = VK_POLYGON_MODE_FILL;
      segment_visualization_render_pipeline->states.color_blend_attachment_states[0].blendEnable = true;

      segment_visualization_render_pipeline->states.ApplyAllStates(vk_command_buffer);
      target_camera->GetRenderTexture()->Render(
          vk_command_buffer, VK_ATTACHMENT_LOAD_OP_LOAD, VK_ATTACHMENT_STORE_OP_STORE, [&] {
            segment_visualization_render_pipeline->Bind(vk_command_buffer);
            segment_visualization_render_pipeline->BindDescriptorSet(
                vk_command_buffer, 0, RenderLayer::GetPerFrameDescriptorSet()->GetVkDescriptorSet());
            segment_visualization_render_pipeline->BindDescriptorSet(
                vk_command_buffer, 1, strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
            segment_visualization_render_pipeline->PushConstant(vk_command_buffer, 0, segment_push_constant);
            const uint32_t count = Platform::DivUp(segments.size(), task_work_group_invocations);
            segment_visualization_render_pipeline->DrawMeshTasks(vk_command_buffer, count, 1, 1);
          });
    }

    if (visualization_parameters.render_foliage) {
      foliage_visualization_render_pipeline->states.ResetAllStates(1);
      foliage_visualization_render_pipeline->states.view_port = viewport;
      foliage_visualization_render_pipeline->states.scissor = scissor;
      foliage_visualization_render_pipeline->states.polygon_mode = VK_POLYGON_MODE_FILL;
      foliage_visualization_render_pipeline->states.color_blend_attachment_states[0].blendEnable = true;

      foliage_visualization_render_pipeline->states.ApplyAllStates(vk_command_buffer);
      target_camera->GetRenderTexture()->Render(
          vk_command_buffer, VK_ATTACHMENT_LOAD_OP_LOAD, VK_ATTACHMENT_STORE_OP_STORE, [&] {
            foliage_visualization_render_pipeline->Bind(vk_command_buffer);
            foliage_visualization_render_pipeline->BindDescriptorSet(
                vk_command_buffer, 0, RenderLayer::GetPerFrameDescriptorSet()->GetVkDescriptorSet());
            foliage_visualization_render_pipeline->BindDescriptorSet(
                vk_command_buffer, 1, strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
            foliage_visualization_render_pipeline->PushConstant(vk_command_buffer, 0, foliage_push_constant);
            const uint32_t count = Platform::DivUp(foliage.size(), task_work_group_invocations);
            foliage_visualization_render_pipeline->DrawMeshTasks(vk_command_buffer, count, 1, 1);
          });
    }
  });

  meshing->Visualize(target_camera, initialize_parameters, visualization_parameters);
}
