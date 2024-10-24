#include "DynamicStrands.hpp"
#include "Shader.hpp"
using namespace eco_sys_lab_plugin;

void DynamicStrands::Render(const RenderParameters& render_parameters) const {
  if (!Platform::Constants::support_mesh_shader) {
    EVOENGINE_LOG("Failed to render! Mesh shader unsupported!")
    return;
  }
  const auto render_layer = Application::GetLayer<RenderLayer>();
  if (!render_layer) {
    EVOENGINE_LOG("Failed to render! RenderLayer not present!")
    return;
  }
  const auto current_frame_index = Platform::GetCurrentFrameIndex();

  static std::shared_ptr<GraphicsPipeline> render_pipeline{};
  struct RenderPushConstant {
    uint32_t camera_index = 0;
    uint32_t padding = 0;
    uint32_t strand_segment_size = 0;
    uint32_t render_mode = 2;
    
    glm::vec4 min_color = glm::vec4(0.2f);
    glm::vec4 max_color = glm::vec4(1.f);
  };
  if (!render_pipeline) {
    static std::shared_ptr<Shader> task_shader{};
    static std::shared_ptr<Shader> mesh_shader{};
    static std::shared_ptr<Shader> frag_shader{};
    // Load shader
    task_shader = std::make_shared<Shader>();
    task_shader->Set(
        ShaderType::Task, Platform::Constants::shader_global_defines,
        std::filesystem::path("./EcoSysLabResources") / "Shaders/Graphics/Task/DynamicStrandsRendering.task");

    mesh_shader = std::make_shared<Shader>();
    mesh_shader->Set(
        ShaderType::Mesh, Platform::Constants::shader_global_defines,
        std::filesystem::path("./EcoSysLabResources") / "Shaders/Graphics/Mesh/DynamicStrandsRendering.mesh");

    frag_shader = std::make_shared<Shader>();
    frag_shader->Set(
        ShaderType::Fragment, Platform::Constants::shader_global_defines,
        std::filesystem::path("./EcoSysLabResources") / "Shaders/Graphics/Fragment/DynamicStrandsRendering.frag");
    // Descriptor set layout
    render_pipeline = std::make_shared<GraphicsPipeline>();
    render_pipeline->task_shader = task_shader;
    render_pipeline->mesh_shader = mesh_shader;

    render_pipeline->fragment_shader = frag_shader;
    render_pipeline->geometry_type = GeometryType::Mesh;

    auto per_frame_layout = Platform::GetDescriptorSetLayout("PER_FRAME_LAYOUT");
    render_pipeline->descriptor_set_layouts.emplace_back(per_frame_layout);
    render_pipeline->descriptor_set_layouts.emplace_back(strands_layout);
    render_pipeline->depth_attachment_format = Platform::Constants::render_texture_depth;
    render_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    render_pipeline->color_attachment_formats = {1, Platform::Constants::render_texture_color};

    auto& push_constant_range = render_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RenderPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;

    render_pipeline->Initialize();
  }

  const uint32_t task_work_group_invocations =
      Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxPreferredTaskWorkGroupInvocations;
  RenderPushConstant push_constant;
  push_constant.render_mode = render_parameters.render_mode;
  push_constant.min_color = render_parameters.min_color;
  push_constant.max_color = render_parameters.max_color;
  push_constant.camera_index = render_layer->GetCameraIndex(render_parameters.target_camera->GetHandle());
  //push_constant.multiplier = render_parameters.multiplier;
  push_constant.strand_segment_size = segments.size();

  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
#pragma region Viewport and scissor
    VkViewport viewport;
    viewport.x = 0.0f;
    viewport.y = 0.0f;
    viewport.width = render_parameters.target_camera->GetSize().x;
    viewport.height = render_parameters.target_camera->GetSize().y;
    viewport.minDepth = 0.0f;
    viewport.maxDepth = 1.0f;
    VkRect2D scissor;
    scissor.offset = {0, 0};
    scissor.extent.width = render_parameters.target_camera->GetSize().x;
    scissor.extent.height = render_parameters.target_camera->GetSize().y;
#pragma endregion
    // 1 here means we only have 1 color attachment. (For deferred shading we will have multiple attachments for
    // GBuffer)
    render_pipeline->states.ResetAllStates(1);
    render_pipeline->states.view_port = viewport;
    render_pipeline->states.scissor = scissor;
    render_pipeline->states.polygon_mode = VK_POLYGON_MODE_FILL;
    render_pipeline->states.color_blend_attachment_states[0].blendEnable = true;

    render_pipeline->states.ApplyAllStates(vk_command_buffer);
    render_parameters.target_camera->GetRenderTexture()->Render(
        vk_command_buffer, VK_ATTACHMENT_LOAD_OP_LOAD, VK_ATTACHMENT_STORE_OP_STORE, [&] {
          render_pipeline->Bind(vk_command_buffer);
          render_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                             render_layer->GetPerFrameDescriptorSet()->GetVkDescriptorSet());
          render_pipeline->BindDescriptorSet(vk_command_buffer, 1,
                                             strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
          render_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
          const uint32_t count = Platform::DivUp(segments.size(), task_work_group_invocations);
          vkCmdDrawMeshTasksEXT(vk_command_buffer, count, 1, 1);
        });
  });
}