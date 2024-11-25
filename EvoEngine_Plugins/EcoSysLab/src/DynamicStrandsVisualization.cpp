#include "DynamicStrands.hpp"
#include "Shader.hpp"
using namespace eco_sys_lab_plugin;

bool DynamicStrands::VisualizationParameters::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::Checkbox("Particles", &render_particles))
    changed = true;
  if (render_particles) {
    if (ImGui::Combo("Particle mode", {"Default", "Segment color", "Connectivity strain"}, particle_render_mode))
      changed = true;
    switch (particle_render_mode) {
      case 0: {
        if (ImGui::ColorEdit4("Particle color", &particle_color_main.x))
          changed = true;

        break;
      }
      case 2: {
        if (ImGui::ColorEdit4("Particle min color", &particle_color_min.x))
          changed = true;
        if (ImGui::ColorEdit4("Particle max color", &particle_color_max.x))
          changed = true;
        if (ImGui::DragFloat("Particle multiplier", &particle_multiplier, 0.1f, 0.1f, 1000.f))
          changed = true;
        break;
      }
    }
  }
  if (ImGui::Checkbox("Segments", &render_segments))
    changed = true;
  if (render_segments) {
    if (ImGui::Combo("Segment mode", {"Default", "Segment color", "Stretch/shear strain"}, segment_render_mode))
      changed = true;
    switch (segment_render_mode) {
      case 0: {
        if (ImGui::ColorEdit4("Main segment color", &segment_color_main.x))
          changed = true;
        if (ImGui::ColorEdit4("Sub segment color", &segment_color_sub.x))
          changed = true;
        break;
      }
      case 2: {
        if (ImGui::ColorEdit4("Segment min color", &segment_color_min.x))
          changed = true;
        if (ImGui::ColorEdit4("Segment max color", &segment_color_max.x))
          changed = true;
        if (ImGui::DragFloat("Segment multiplier", &segment_multiplier, 0.1f, 0.1f, 1000.f))
          changed = true;
        break;
      }
    }
  }
  if (ImGui::Checkbox("Connections", &render_connections))
    changed = true;
  if (render_connections) {
    if (ImGui::Combo("Connection mode", {"Default", "Bend/twist strain"}, connection_render_mode))
      changed = true;
    switch (connection_render_mode) {
      case 0: {
        if (ImGui::ColorEdit4("Main connection color", &connection_color_main.x))
          changed = true;
        if (ImGui::ColorEdit4("Sub connection color", &connection_color_sub.x))
          changed = true;
        break;
      }
      case 1: {
        if (ImGui::ColorEdit4("Connection min color", &connection_color_min.x))
          changed = true;
        if (ImGui::ColorEdit4("Connection max color", &connection_color_max.x))
          changed = true;
        if (ImGui::DragFloat("Connection multiplier", &connection_multiplier, 0.1f, 0.1f, 1000.f))
          changed = true;
        break;
      }
    }
  }

  if (ImGui::Checkbox("Uniform Particle", &render_uniform_particles))
    changed = true;
  if (render_uniform_particles) {
    if (ImGui::Combo("Uniform particle mode", {"Default", "Segment color"}, uniform_particle_render_mode))
      changed = true;
    switch (uniform_particle_render_mode) {
      case 0: {
        if (ImGui::ColorEdit4("Uniform particle color", &uniform_particle_main.x))
          changed = true;
        break;
      }
    }
  }
  return changed;
}

void DynamicStrands::Visualize(const std::shared_ptr<Camera>& target_camera,
                               const VisualizationParameters& visualization_parameters) const {
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

  static std::shared_ptr<GraphicsPipeline> particle_render_pipeline{};
  struct ParticleRenderPushConstant {
    glm::vec4 min_color = glm::vec4(0.2f);
    glm::vec4 max_color = glm::vec4(1.f);

    uint32_t camera_index = 0;
    uint32_t strand_particle_size = 0;
    uint32_t render_mode = 2;
    float multiplier = 10.0f;
  };

  if (!particle_render_pipeline) {
    static std::shared_ptr<Shader> task_shader{};
    static std::shared_ptr<Shader> mesh_shader{};
    static std::shared_ptr<Shader> frag_shader{};
    // Load shader
    task_shader = std::make_shared<Shader>();
    task_shader->Set(ShaderType::Task, Platform::Constants::shader_global_defines,
                     std::filesystem::path("./EcoSysLabResources") /
                         "Shaders/Graphics/Task/DynamicStrandParticlesVisualization.task");
    mesh_shader = std::make_shared<Shader>();
    mesh_shader->Set(ShaderType::Mesh, Platform::Constants::shader_global_defines,
                     std::filesystem::path("./EcoSysLabResources") /
                         "Shaders/Graphics/Mesh/DynamicStrandParticlesVisualization.mesh");

    frag_shader = std::make_shared<Shader>();
    frag_shader->Set(
        ShaderType::Fragment, Platform::Constants::shader_global_defines,
        std::filesystem::path("./EcoSysLabResources") / "Shaders/Graphics/Fragment/DynamicStrandsVisualization.frag");
    // Descriptor set layout
    particle_render_pipeline = std::make_shared<GraphicsPipeline>();
    particle_render_pipeline->task_shader = task_shader;
    particle_render_pipeline->mesh_shader = mesh_shader;

    particle_render_pipeline->fragment_shader = frag_shader;
    particle_render_pipeline->geometry_type = GeometryType::Mesh;

    auto per_frame_layout = Platform::GetDescriptorSetLayout("PER_FRAME_LAYOUT");
    particle_render_pipeline->descriptor_set_layouts.emplace_back(per_frame_layout);
    particle_render_pipeline->descriptor_set_layouts.emplace_back(strands_layout);
    particle_render_pipeline->depth_attachment_format = Platform::Constants::render_texture_depth;
    particle_render_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    particle_render_pipeline->color_attachment_formats = {1, Platform::Constants::render_texture_color};

    auto& push_constant_range = particle_render_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(ParticleRenderPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;

    particle_render_pipeline->Initialize();
  }

  static std::shared_ptr<GraphicsPipeline> segment_render_pipeline{};
  struct SegmentRenderPushConstant {
    glm::vec4 min_color = glm::vec4(0.2f);
    glm::vec4 max_color = glm::vec4(1.f);

    uint32_t camera_index = 0;
    uint32_t strand_segment_size = 0;
    uint32_t render_mode = 2;
    float multiplier = 10.0f;
  };
  if (!segment_render_pipeline) {
    static std::shared_ptr<Shader> task_shader{};
    static std::shared_ptr<Shader> mesh_shader{};
    static std::shared_ptr<Shader> frag_shader{};
    // Load shader
    task_shader = std::make_shared<Shader>();
    task_shader->Set(ShaderType::Task, Platform::Constants::shader_global_defines,
                     std::filesystem::path("./EcoSysLabResources") /
                         "Shaders/Graphics/Task/DynamicStrandSegmentsVisualization.task");
    mesh_shader = std::make_shared<Shader>();
    mesh_shader->Set(ShaderType::Mesh, Platform::Constants::shader_global_defines,
                     std::filesystem::path("./EcoSysLabResources") /
                         "Shaders/Graphics/Mesh/DynamicStrandSegmentsVisualization.mesh");
    frag_shader = std::make_shared<Shader>();
    frag_shader->Set(
        ShaderType::Fragment, Platform::Constants::shader_global_defines,
        std::filesystem::path("./EcoSysLabResources") / "Shaders/Graphics/Fragment/DynamicStrandsVisualization.frag");
    // Descriptor set layout
    segment_render_pipeline = std::make_shared<GraphicsPipeline>();
    segment_render_pipeline->task_shader = task_shader;
    segment_render_pipeline->mesh_shader = mesh_shader;

    segment_render_pipeline->fragment_shader = frag_shader;
    segment_render_pipeline->geometry_type = GeometryType::Mesh;

    auto per_frame_layout = Platform::GetDescriptorSetLayout("PER_FRAME_LAYOUT");
    segment_render_pipeline->descriptor_set_layouts.emplace_back(per_frame_layout);
    segment_render_pipeline->descriptor_set_layouts.emplace_back(strands_layout);
    segment_render_pipeline->depth_attachment_format = Platform::Constants::render_texture_depth;
    segment_render_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    segment_render_pipeline->color_attachment_formats = {1, Platform::Constants::render_texture_color};

    auto& push_constant_range = segment_render_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(SegmentRenderPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;

    segment_render_pipeline->Initialize();
  }

  static std::shared_ptr<GraphicsPipeline> connection_render_pipeline{};
  struct ConnectionRenderPushConstant {
    glm::vec4 min_color = glm::vec4(0.2f);
    glm::vec4 max_color = glm::vec4(1.f);

    uint32_t camera_index = 0;
    uint32_t strand_connection_size = 0;
    uint32_t render_mode = 2;
    float multiplier = 10.0f;
  };

  if (!connection_render_pipeline) {
    static std::shared_ptr<Shader> task_shader{};
    static std::shared_ptr<Shader> mesh_shader{};
    static std::shared_ptr<Shader> frag_shader{};
    // Load shader
    task_shader = std::make_shared<Shader>();
    task_shader->Set(ShaderType::Task, Platform::Constants::shader_global_defines,
                     std::filesystem::path("./EcoSysLabResources") /
                         "Shaders/Graphics/Task/DynamicStrandConnectionsVisualization.task");
    mesh_shader = std::make_shared<Shader>();

    mesh_shader->Set(ShaderType::Mesh, Platform::Constants::shader_global_defines,
                     std::filesystem::path("./EcoSysLabResources") /
                         "Shaders/Graphics/Mesh/DynamicStrandConnectionsVisualization.mesh");

    frag_shader = std::make_shared<Shader>();
    frag_shader->Set(
        ShaderType::Fragment, Platform::Constants::shader_global_defines,
        std::filesystem::path("./EcoSysLabResources") / "Shaders/Graphics/Fragment/DynamicStrandsVisualization.frag");
    // Descriptor set layout
    connection_render_pipeline = std::make_shared<GraphicsPipeline>();
    connection_render_pipeline->task_shader = task_shader;
    connection_render_pipeline->mesh_shader = mesh_shader;

    connection_render_pipeline->fragment_shader = frag_shader;
    connection_render_pipeline->geometry_type = GeometryType::Mesh;

    auto per_frame_layout = Platform::GetDescriptorSetLayout("PER_FRAME_LAYOUT");
    connection_render_pipeline->descriptor_set_layouts.emplace_back(per_frame_layout);
    connection_render_pipeline->descriptor_set_layouts.emplace_back(strands_layout);
    connection_render_pipeline->depth_attachment_format = Platform::Constants::render_texture_depth;
    connection_render_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    connection_render_pipeline->color_attachment_formats = {1, Platform::Constants::render_texture_color};

    auto& push_constant_range = connection_render_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(ConnectionRenderPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;

    connection_render_pipeline->Initialize();
  }

  static std::shared_ptr<GraphicsPipeline> uniform_particle_render_pipeline{};
  struct UniformParticleRenderPushConstant {
    glm::vec4 min_color = glm::vec4(0.2f);
    glm::vec4 max_color = glm::vec4(1.f);

    uint32_t camera_index = 0;
    uint32_t strand_uniform_particle_size = 0;
    uint32_t render_mode = 2;
    float multiplier = 10.0f;
  };

  if (!uniform_particle_render_pipeline) {
    static std::shared_ptr<Shader> task_shader{};
    static std::shared_ptr<Shader> mesh_shader{};
    static std::shared_ptr<Shader> frag_shader{};
    // Load shader
    task_shader = std::make_shared<Shader>();
    task_shader->Set(ShaderType::Task, Platform::Constants::shader_global_defines,
                     std::filesystem::path("./EcoSysLabResources") /
                         "Shaders/Graphics/Task/DynamicStrandUniformParticlesVisualization.task");
    mesh_shader = std::make_shared<Shader>();
    mesh_shader->Set(ShaderType::Mesh, Platform::Constants::shader_global_defines,
                     std::filesystem::path("./EcoSysLabResources") /
                         "Shaders/Graphics/Mesh/DynamicStrandUniformParticlesVisualization.mesh");

    frag_shader = std::make_shared<Shader>();
    frag_shader->Set(
        ShaderType::Fragment, Platform::Constants::shader_global_defines,
        std::filesystem::path("./EcoSysLabResources") / "Shaders/Graphics/Fragment/DynamicStrandsVisualization.frag");
    // Descriptor set layout
    uniform_particle_render_pipeline = std::make_shared<GraphicsPipeline>();
    uniform_particle_render_pipeline->task_shader = task_shader;
    uniform_particle_render_pipeline->mesh_shader = mesh_shader;

    uniform_particle_render_pipeline->fragment_shader = frag_shader;
    uniform_particle_render_pipeline->geometry_type = GeometryType::Mesh;

    auto per_frame_layout = Platform::GetDescriptorSetLayout("PER_FRAME_LAYOUT");
    uniform_particle_render_pipeline->descriptor_set_layouts.emplace_back(per_frame_layout);
    uniform_particle_render_pipeline->descriptor_set_layouts.emplace_back(strands_layout);
    uniform_particle_render_pipeline->depth_attachment_format = Platform::Constants::render_texture_depth;
    uniform_particle_render_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    uniform_particle_render_pipeline->color_attachment_formats = {1, Platform::Constants::render_texture_color};

    auto& push_constant_range = uniform_particle_render_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(UniformParticleRenderPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;

    uniform_particle_render_pipeline->Initialize();
  }


  const uint32_t task_work_group_invocations =
      Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxPreferredTaskWorkGroupInvocations;

  ParticleRenderPushConstant particle_push_constant;
  particle_push_constant.render_mode = visualization_parameters.particle_render_mode;
  particle_push_constant.min_color = visualization_parameters.particle_render_mode == 0
                                         ? visualization_parameters.particle_color_main
                                         : visualization_parameters.particle_color_min;
  particle_push_constant.max_color = visualization_parameters.particle_color_max;
  particle_push_constant.camera_index = render_layer->GetCameraIndex(target_camera->GetHandle());
  particle_push_constant.multiplier = visualization_parameters.particle_multiplier;
  particle_push_constant.strand_particle_size = particles.size();

  UniformParticleRenderPushConstant uniform_particle_push_constant;
  uniform_particle_push_constant.render_mode = visualization_parameters.uniform_particle_render_mode;
  uniform_particle_push_constant.min_color = visualization_parameters.uniform_particle_main;
  uniform_particle_push_constant.camera_index = render_layer->GetCameraIndex(target_camera->GetHandle());
  uniform_particle_push_constant.multiplier = 1.f;
  uniform_particle_push_constant.strand_uniform_particle_size = uniform_particles.size();

  SegmentRenderPushConstant segment_push_constant;
  segment_push_constant.render_mode = visualization_parameters.segment_render_mode;
  segment_push_constant.min_color = visualization_parameters.segment_render_mode == 0
                                        ? visualization_parameters.segment_color_main
                                        : visualization_parameters.segment_color_min;
  segment_push_constant.max_color = visualization_parameters.segment_render_mode == 0
                                        ? visualization_parameters.segment_color_sub
                                        : visualization_parameters.segment_color_max;
  segment_push_constant.camera_index = render_layer->GetCameraIndex(target_camera->GetHandle());
  segment_push_constant.multiplier = visualization_parameters.segment_multiplier;
  segment_push_constant.strand_segment_size = segments.size();

  ConnectionRenderPushConstant connection_push_constant;
  connection_push_constant.render_mode = visualization_parameters.connection_render_mode;
  connection_push_constant.min_color = visualization_parameters.connection_render_mode == 0
                                           ? visualization_parameters.connection_color_main
                                           : visualization_parameters.connection_color_min;
  connection_push_constant.max_color = visualization_parameters.connection_render_mode == 0
                                           ? visualization_parameters.connection_color_sub
                                           : visualization_parameters.connection_color_max;
  connection_push_constant.camera_index = render_layer->GetCameraIndex(target_camera->GetHandle());
  connection_push_constant.multiplier = visualization_parameters.connection_multiplier;
  connection_push_constant.strand_connection_size = connections.size();

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
    if (visualization_parameters.render_connections) {
      connection_render_pipeline->states.ResetAllStates(1);
      connection_render_pipeline->states.view_port = viewport;
      connection_render_pipeline->states.scissor = scissor;
      connection_render_pipeline->states.polygon_mode = VK_POLYGON_MODE_FILL;
      connection_render_pipeline->states.color_blend_attachment_states[0].blendEnable = true;

      connection_render_pipeline->states.ApplyAllStates(vk_command_buffer);
      target_camera->GetRenderTexture()->Render(
          vk_command_buffer, VK_ATTACHMENT_LOAD_OP_LOAD, VK_ATTACHMENT_STORE_OP_STORE, [&] {
            connection_render_pipeline->Bind(vk_command_buffer);
            connection_render_pipeline->BindDescriptorSet(
                vk_command_buffer, 0, render_layer->GetPerFrameDescriptorSet()->GetVkDescriptorSet());
            connection_render_pipeline->BindDescriptorSet(
                vk_command_buffer, 1, strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
            connection_render_pipeline->PushConstant(vk_command_buffer, 0, connection_push_constant);
            const uint32_t count = Platform::DivUp(connections.size(), task_work_group_invocations);
            vkCmdDrawMeshTasksEXT(vk_command_buffer, count, 1, 1);
          });
    }
    if (visualization_parameters.render_segments) {
      segment_render_pipeline->states.ResetAllStates(1);
      segment_render_pipeline->states.view_port = viewport;
      segment_render_pipeline->states.scissor = scissor;
      segment_render_pipeline->states.polygon_mode = VK_POLYGON_MODE_FILL;
      segment_render_pipeline->states.color_blend_attachment_states[0].blendEnable = true;

      segment_render_pipeline->states.ApplyAllStates(vk_command_buffer);
      target_camera->GetRenderTexture()->Render(
          vk_command_buffer, VK_ATTACHMENT_LOAD_OP_LOAD, VK_ATTACHMENT_STORE_OP_STORE, [&] {
            segment_render_pipeline->Bind(vk_command_buffer);
            segment_render_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                                       render_layer->GetPerFrameDescriptorSet()->GetVkDescriptorSet());
            segment_render_pipeline->BindDescriptorSet(
                vk_command_buffer, 1, strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
            segment_render_pipeline->PushConstant(vk_command_buffer, 0, segment_push_constant);
            const uint32_t count = Platform::DivUp(segments.size(), task_work_group_invocations);
            vkCmdDrawMeshTasksEXT(vk_command_buffer, count, 1, 1);
          });
    }
    if (visualization_parameters.render_particles) {
      particle_render_pipeline->states.ResetAllStates(1);
      particle_render_pipeline->states.view_port = viewport;
      particle_render_pipeline->states.scissor = scissor;
      particle_render_pipeline->states.polygon_mode = VK_POLYGON_MODE_FILL;
      particle_render_pipeline->states.color_blend_attachment_states[0].blendEnable = true;

      particle_render_pipeline->states.ApplyAllStates(vk_command_buffer);
      target_camera->GetRenderTexture()->Render(
          vk_command_buffer, VK_ATTACHMENT_LOAD_OP_LOAD, VK_ATTACHMENT_STORE_OP_STORE, [&] {
            particle_render_pipeline->Bind(vk_command_buffer);
            particle_render_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                                        render_layer->GetPerFrameDescriptorSet()->GetVkDescriptorSet());
            particle_render_pipeline->BindDescriptorSet(
                vk_command_buffer, 1, strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
            particle_render_pipeline->PushConstant(vk_command_buffer, 0, particle_push_constant);
            const uint32_t count = Platform::DivUp(particles.size(), task_work_group_invocations);
            vkCmdDrawMeshTasksEXT(vk_command_buffer, count, 1, 1);
          });
    }
    if (visualization_parameters.render_uniform_particles) {
      uniform_particle_render_pipeline->states.ResetAllStates(1);
      uniform_particle_render_pipeline->states.view_port = viewport;
      uniform_particle_render_pipeline->states.scissor = scissor;
      uniform_particle_render_pipeline->states.polygon_mode = VK_POLYGON_MODE_FILL;
      uniform_particle_render_pipeline->states.color_blend_attachment_states[0].blendEnable = true;

      uniform_particle_render_pipeline->states.ApplyAllStates(vk_command_buffer);
      target_camera->GetRenderTexture()->Render(
          vk_command_buffer, VK_ATTACHMENT_LOAD_OP_LOAD, VK_ATTACHMENT_STORE_OP_STORE, [&] {
            uniform_particle_render_pipeline->Bind(vk_command_buffer);
            uniform_particle_render_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                                        render_layer->GetPerFrameDescriptorSet()->GetVkDescriptorSet());
            uniform_particle_render_pipeline->BindDescriptorSet(
                vk_command_buffer, 1, strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
            uniform_particle_render_pipeline->PushConstant(vk_command_buffer, 0, uniform_particle_push_constant);
            const uint32_t count = Platform::DivUp(uniform_particles.size(), task_work_group_invocations);
            vkCmdDrawMeshTasksEXT(vk_command_buffer, count, 1, 1);
          });
    }
  });
}
