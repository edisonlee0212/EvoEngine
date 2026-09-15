#include "EditorLayer.hpp"
#include "GeometryStorage.hpp"
#include "GraphicsPipeline.hpp"
#include "RenderLayer.hpp"
#include "Resources.hpp"
#include "Shader.hpp"

using namespace evo_engine;

void EditorLayer::InitializeGizmoPipelines() {
  const auto render_layer = GetApplication().GetLayer<RenderLayer>();
  if (!render_layer)
    return;
  if (!gizmos) {
    gizmos = std::make_shared<GraphicsPipeline>();
    gizmos->vertex_shader =
        Shader::CreateTemporary(ShaderType::Vertex, Platform::GetShaderGlobalDefines(),
                                Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Vertex/Gizmos/Gizmos.slang");
    gizmos->fragment_shader =
        Shader::CreateTemporary(ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
                                Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Fragment/Gizmos/Gizmos.slang");
    gizmos->geometry_type = GeometryType::Mesh;
    gizmos->vertex_input_attribute_set = VertexInputAttributeSet::Position;
    gizmos->depth_attachment_format = Platform::Constants::render_texture_depth;
    gizmos->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    gizmos->color_attachment_formats = {1, Platform::Constants::render_texture_color};
    gizmos->descriptor_set_layouts.emplace_back(render_layer->GetPerFrameDescriptorSetLayout());
    auto& push_constant_range = gizmos->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(GizmosPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;

    gizmos->Initialize();
  }
  if (!gizmos_normal_colored) {
    gizmos_normal_colored = std::make_shared<GraphicsPipeline>();
    gizmos_normal_colored->vertex_shader = Shader::CreateTemporary(
        ShaderType::Vertex, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Vertex/Gizmos/GizmosNormalColored.slang");
    gizmos_normal_colored->fragment_shader = Shader::CreateTemporary(
        ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Fragment/Gizmos/GizmosColored.slang");
    gizmos_normal_colored->geometry_type = GeometryType::Mesh;
    gizmos_normal_colored->vertex_input_attribute_set = VertexInputAttributeSet::PositionNormal;
    gizmos_normal_colored->depth_attachment_format = Platform::Constants::render_texture_depth;
    gizmos_normal_colored->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    gizmos_normal_colored->color_attachment_formats = {1, Platform::Constants::render_texture_color};
    gizmos_normal_colored->descriptor_set_layouts.emplace_back(render_layer->GetPerFrameDescriptorSetLayout());
    gizmos_normal_colored->tessellation_patch_control_points = 4;
    auto& push_constant_range = gizmos_normal_colored->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(GizmosPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    gizmos_normal_colored->Initialize();
  }
  if (!gizmos_vertex_colored) {
    gizmos_vertex_colored = std::make_shared<GraphicsPipeline>();
    gizmos_vertex_colored->vertex_shader = Shader::CreateTemporary(
        ShaderType::Vertex, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Vertex/Gizmos/GizmosVertexColored.slang");
    gizmos_vertex_colored->fragment_shader = Shader::CreateTemporary(
        ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Fragment/Gizmos/GizmosColored.slang");
    gizmos_vertex_colored->geometry_type = GeometryType::Mesh;
    gizmos_vertex_colored->vertex_input_attribute_set = VertexInputAttributeSet::PositionColor;
    gizmos_vertex_colored->depth_attachment_format = Platform::Constants::render_texture_depth;
    gizmos_vertex_colored->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    gizmos_vertex_colored->color_attachment_formats = {1, Platform::Constants::render_texture_color};
    gizmos_vertex_colored->descriptor_set_layouts.emplace_back(render_layer->GetPerFrameDescriptorSetLayout());
    auto& push_constant_range = gizmos_vertex_colored->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(GizmosPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    gizmos_vertex_colored->Initialize();
  }
  if (!gizmos_instanced_colored) {
    gizmos_instanced_colored = std::make_shared<GraphicsPipeline>();
    gizmos_instanced_colored->vertex_shader = Shader::CreateTemporary(
        ShaderType::Vertex, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Vertex/Gizmos/GizmosInstancedColored.slang");
    gizmos_instanced_colored->fragment_shader = Shader::CreateTemporary(
        ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Fragment/Gizmos/GizmosColored.slang");
    gizmos_instanced_colored->geometry_type = GeometryType::Mesh;
    gizmos_instanced_colored->vertex_input_attribute_set = VertexInputAttributeSet::Position;
    gizmos_instanced_colored->depth_attachment_format = Platform::Constants::render_texture_depth;
    gizmos_instanced_colored->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    gizmos_instanced_colored->color_attachment_formats = {1, Platform::Constants::render_texture_color};
    gizmos_instanced_colored->descriptor_set_layouts.emplace_back(render_layer->GetPerFrameDescriptorSetLayout());
    gizmos_instanced_colored->descriptor_set_layouts.emplace_back(
        render_layer->GetParticleInstancedDataDescriptorSetLayout());
    auto& push_constant_range = gizmos_instanced_colored->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(GizmosPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;

    gizmos_instanced_colored->Initialize();
  }
  if (Platform::MeshShaderEnabled()) {
    const auto create_gizmo_strands_pipeline = [&](const std::filesystem::path& fragment_shader_path) {
      auto pipeline = std::make_shared<GraphicsPipeline>();
      pipeline->task_shader = Shader::CreateTemporary(
          ShaderType::Task, Platform::GetShaderGlobalDefines(),
          Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Task/Gizmos/GizmosStrands.slang");
      pipeline->mesh_shader = Shader::CreateTemporary(
          ShaderType::Mesh, Platform::GetShaderGlobalDefines(),
          Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Mesh/Gizmos/GizmosStrands.slang");
      pipeline->fragment_shader =
          Shader::CreateTemporary(ShaderType::Fragment, Platform::GetShaderGlobalDefines(), fragment_shader_path);
      pipeline->vertex_input_enabled = false;
      pipeline->depth_attachment_format = Platform::Constants::render_texture_depth;
      pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
      pipeline->color_attachment_formats = {1, Platform::Constants::render_texture_color};
      pipeline->descriptor_set_layouts = {render_layer->GetPerFrameDescriptorSetLayout(),
                                          render_layer->GetStrandMeshletDescriptorSetLayout()};
      auto& push_constant_range = pipeline->push_constant_ranges.emplace_back();
      push_constant_range.size = sizeof(GizmosPushConstant);
      push_constant_range.offset = 0;
      push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
      pipeline->Initialize();
      return pipeline;
    };
    const auto gizmos_fragment_path =
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Fragment/Gizmos/Gizmos.slang";
    const auto colored_fragment_path =
        Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Fragment/Gizmos/GizmosColored.slang";
    if (!gizmos_strands) {
      gizmos_strands = create_gizmo_strands_pipeline(gizmos_fragment_path);
    }
    if (!gizmos_strands_normal_colored) {
      gizmos_strands_normal_colored = create_gizmo_strands_pipeline(colored_fragment_path);
    }
    if (!gizmos_strands_vertex_colored) {
      gizmos_strands_vertex_colored = create_gizmo_strands_pipeline(colored_fragment_path);
    }
  }
}

void EditorLayer::OnPostRender() {
  if (const auto scene = GetScene(); !scene)
    return;
  if (const auto render_layer = GetApplication().GetLayer<RenderLayer>()) {
    const auto current_frame_index = Platform::GetCurrentFrameIndex();
    const auto current_render_instances = render_layer->GetCurrentRenderInstanceStorage();
    for (const auto& i : gizmo_mesh_tasks_) {
      if (editor_cameras_.find(i.editor_camera_component->GetHandle()) == editor_cameras_.end()) {
        EVOENGINE_ERROR("Target camera not registered in editor!");
        return;
      }
      if (i.editor_camera_component && i.editor_camera_component->IsEnabled()) {
        Platform::RecordCommandsMainQueue(
            [this, i, current_frame_index, current_render_instances](VkCommandBuffer vk_command_buffer) {
              std::shared_ptr<GraphicsPipeline> gizmos_pipeline;
              switch (i.gizmo_settings.color_mode) {
                case GizmoSettings::ColorMode::Default: {
                  gizmos_pipeline = gizmos;
                } break;
                case GizmoSettings::ColorMode::VertexColor: {
                  gizmos_pipeline = gizmos_vertex_colored;
                } break;
                case GizmoSettings::ColorMode::NormalColor: {
                  gizmos_pipeline = gizmos_normal_colored;
                } break;
              }
              i.editor_camera_component->GetRenderTexture()->ApplyGraphicsPipelineStates(gizmos_pipeline->states);
              i.gizmo_settings.ApplySettings(gizmos_pipeline->states);

              gizmos_pipeline->Bind(vk_command_buffer);
              gizmos_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                                 RenderLayer::GetPerFrameDescriptorSet()->GetVkDescriptorSet());

              i.editor_camera_component->GetRenderTexture()->Render(
                  vk_command_buffer, VK_ATTACHMENT_LOAD_OP_LOAD, VK_ATTACHMENT_STORE_OP_STORE, [&]() {
                    GizmosPushConstant push_constant;
                    push_constant.model = i.model;
                    push_constant.color = i.color;
                    push_constant.size = i.size;
                    push_constant.camera_index =
                        current_render_instances->GetCameraIndex(i.editor_camera_component->GetHandle());
                    push_constant.strand_meshlet_offset = 0;
                    push_constant.strand_color_mode = 0;
                    gizmos_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
                    GeometryStorage::BindVertices(vk_command_buffer);
                    i.mesh->DrawIndexed(vk_command_buffer, gizmos_pipeline->states, 1);
                  });
            });
      }
    }
    if (Platform::MeshShaderEnabled()) {
      for (const auto& i : gizmo_strands_tasks_) {
        if (!i.strands || !i.editor_camera_component || !i.editor_camera_component->IsEnabled() ||
            !i.strands->GetStrandMeshletRange() || !i.strands->GetSegmentRange() ||
            i.strands->GetStrandMeshletRange()->prev_frame_range == 0) {
          continue;
        }
        if (editor_cameras_.find(i.editor_camera_component->GetHandle()) == editor_cameras_.end()) {
          EVOENGINE_ERROR("Target camera not registered in editor!");
          return;
        }
        Platform::RecordCommandsMainQueue([this, i, current_frame_index,
                                           current_render_instances](VkCommandBuffer vk_command_buffer) {
          std::shared_ptr<GraphicsPipeline> gizmos_pipeline;
          switch (i.gizmo_settings.color_mode) {
            case GizmoSettings::ColorMode::Default:
              gizmos_pipeline = gizmos_strands;
              break;
            case GizmoSettings::ColorMode::VertexColor:
              gizmos_pipeline = gizmos_strands_vertex_colored;
              break;
            case GizmoSettings::ColorMode::NormalColor:
              gizmos_pipeline = gizmos_strands_normal_colored;
              break;
          }
          i.editor_camera_component->GetRenderTexture()->ApplyGraphicsPipelineStates(gizmos_pipeline->states);
          i.gizmo_settings.ApplySettings(gizmos_pipeline->states);
          gizmos_pipeline->Bind(vk_command_buffer);
          gizmos_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                             RenderLayer::GetPerFrameDescriptorSet()->GetVkDescriptorSet());
          gizmos_pipeline->BindDescriptorSet(
              vk_command_buffer, 1,
              GetApplication().GetLayer<RenderLayer>()->GetStrandMeshletDescriptorSet()->GetVkDescriptorSet());

          i.editor_camera_component->GetRenderTexture()->Render(
              vk_command_buffer, VK_ATTACHMENT_LOAD_OP_LOAD, VK_ATTACHMENT_STORE_OP_STORE, [&]() {
                GizmosPushConstant push_constant;
                push_constant.model = i.model;
                push_constant.color = i.color;
                push_constant.size = i.size;
                push_constant.camera_index =
                    current_render_instances->GetCameraIndex(i.editor_camera_component->GetHandle());
                push_constant.strand_meshlet_offset = i.strands->GetStrandMeshletRange()->prev_frame_offset;
                push_constant.strand_color_mode = static_cast<uint32_t>(i.gizmo_settings.color_mode);
                gizmos_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
                gizmos_pipeline->DrawMeshTasks(vk_command_buffer, i.strands->GetStrandMeshletRange()->prev_frame_range);
                Platform::CountRenderPassDraw(RenderPassDrawBucket::EditorGizmos, RenderDrawCallKind::Direct,
                                              current_frame_index,
                                              i.strands->GetSegmentRange()->prev_frame_index_count);
              });
        });
      }
    }
    for (const auto& i : gizmo_instanced_mesh_tasks_) {
      if (editor_cameras_.find(i.editor_camera_component->GetHandle()) == editor_cameras_.end()) {
        EVOENGINE_ERROR("Target camera not registered in editor!")
        return;
      }
      if (i.editor_camera_component && i.editor_camera_component->IsEnabled()) {
        Platform::RecordCommandsMainQueue([this, i, current_frame_index,
                                           current_render_instances](VkCommandBuffer vk_command_buffer) {
          i.editor_camera_component->GetRenderTexture()->ApplyGraphicsPipelineStates(gizmos_instanced_colored->states);
          i.gizmo_settings.ApplySettings(gizmos_instanced_colored->states);

          gizmos_instanced_colored->Bind(vk_command_buffer);
          gizmos_instanced_colored->BindDescriptorSet(vk_command_buffer, 0,
                                                      RenderLayer::GetPerFrameDescriptorSet()->GetVkDescriptorSet());
          gizmos_instanced_colored->BindDescriptorSet(vk_command_buffer, 1,
                                                      i.particle_info_list->GetDescriptorSet()->GetVkDescriptorSet());

          i.editor_camera_component->GetRenderTexture()->Render(
              vk_command_buffer, VK_ATTACHMENT_LOAD_OP_LOAD, VK_ATTACHMENT_STORE_OP_STORE, [&] {
                GizmosPushConstant push_constant;
                push_constant.model = i.model;
                push_constant.color = glm::vec4(0.0f);
                push_constant.size = i.size;
                push_constant.camera_index =
                    current_render_instances->GetCameraIndex(i.editor_camera_component->GetHandle());
                push_constant.strand_meshlet_offset = 0;
                push_constant.strand_color_mode = 0;
                gizmos_instanced_colored->PushConstant(vk_command_buffer, 0, push_constant);
                GeometryStorage::BindVertices(vk_command_buffer);
                i.mesh->DrawIndexed(vk_command_buffer, gizmos_instanced_colored->states,
                                    i.particle_info_list->PeekParticleInfoList().size());
              });
        });
      }
    }
  }
}
