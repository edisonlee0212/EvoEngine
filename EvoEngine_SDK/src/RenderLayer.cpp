#include "RenderLayer.hpp"
#include "Application.hpp"
#include "EditorLayer.hpp"
#include "GeometryStorage.hpp"
#include "GraphicsPipeline.hpp"
#include "Jobs.hpp"
#include "LodGroup.hpp"
#include "MeshRenderer.hpp"
#include "Particles.hpp"
#include "Platform.hpp"
#include "PostProcessingStack.hpp"
#include "ProjectManager.hpp"
#include "Resources.hpp"
#include "SkinnedMeshRenderer.hpp"
#include "StrandsRenderer.hpp"
#include "TextureStorage.hpp"
#include "Utilities.hpp"
using namespace evo_engine;

void RenderLayer::RenderToPointLightShadowMap(
    std::function<uint32_t(VkCommandBuffer vk_command_buffer, const PointLightShadowMapView& shadow_map_view)>&& func) {
  point_light_shadow_map_external_functions.emplace_back(func);
}

void RenderLayer::RenderToSpotLightShadowMap(
    std::function<uint32_t(VkCommandBuffer vk_command_buffer, const SpotLightShadowMapView& shadow_map_view)>&& func) {
  spot_light_shadow_map_external_functions.emplace_back(func);
}

void RenderLayer::RenderToDirectionalLightShadowMap(
    std::function<uint32_t(VkCommandBuffer vk_command_buffer, const DirectionalLightShadowMapView& shadow_map_view)>&&
        func) {
  directional_light_shadow_map_external_functions.emplace_back(func);
}

void RenderLayer::ForwardRenderingAllCameras(
    std::function<uint32_t(VkCommandBuffer vk_command_buffer, const std::shared_ptr<Camera>& target_camera,
                           const ForwardRenderingView& forward_rendering_view)>&& func) {
  forward_rendering_external_functions.emplace_back(func);
}

void RenderLayer::OnCreate() {
  const auto max_frames_in_flight = Platform::GetMaxFramesInFlight();
  render_instances_list_.resize(max_frames_in_flight);
  for (auto& i : render_instances_list_) {
    i = std::make_shared<RenderInstanceStorage>();
  }
  kernel_descriptor_buffers_.clear();
  VkBufferCreateInfo buffer_create_info{};
  buffer_create_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  buffer_create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  VmaAllocationCreateInfo buffer_vma_allocation_create_info{};
  buffer_vma_allocation_create_info.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
  const auto max_frame_in_flight = Platform::GetMaxFramesInFlight();
  for (size_t i = 0; i < max_frame_in_flight; i++) {
    buffer_create_info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;
    buffer_create_info.size = sizeof(glm::vec4) * Platform::Constants::max_kernel_amount * 2;
    kernel_descriptor_buffers_.emplace_back(
        std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info));
  }
  per_frame_descriptor_sets_.clear();
  for (size_t i = 0; i < max_frames_in_flight; i++) {
    auto descriptor_set = std::make_shared<DescriptorSet>(Platform::GetDescriptorSetLayout("PER_FRAME_LAYOUT"));
    per_frame_descriptor_sets_.emplace_back(descriptor_set);
  }

  meshlet_descriptor_sets_.clear();
  for (size_t i = 0; i < max_frames_in_flight; i++) {
    auto descriptor_set = std::make_shared<DescriptorSet>(Platform::GetDescriptorSetLayout("MESHLET_LAYOUT"));
    meshlet_descriptor_sets_.emplace_back(descriptor_set);
  }

  ray_tracing_descriptor_sets_.clear();
  if (Platform::Constants::support_ray_tracing) {
    for (size_t i = 0; i < max_frames_in_flight; i++) {
      auto descriptor_set = std::make_shared<DescriptorSet>(Platform::GetDescriptorSetLayout("RAY_TRACING_LAYOUT"));
      ray_tracing_descriptor_sets_.emplace_back(descriptor_set);
    }
  }

  std::vector<glm::vec4> kernels;
  for (uint32_t i = 0; i < Platform::Constants::max_kernel_amount; i++) {
    kernels.emplace_back(glm::ballRand(1.0f), 1.0f);
  }
  for (uint32_t i = 0; i < Platform::Constants::max_kernel_amount; i++) {
    kernels.emplace_back(glm::gaussRand(0.0f, 1.0f), glm::gaussRand(0.0f, 1.0f), glm::gaussRand(0.0f, 1.0f),
                         glm::gaussRand(0.0f, 1.0f));
  }
  for (int i = 0; i < Platform::GetMaxFramesInFlight(); i++) {
    kernel_descriptor_buffers_[i]->UploadVector(kernels);
  }
  PrepareEnvironmentalBrdfLut();
  lighting_ = std::make_unique<Lighting>();
  lighting_->Initialize();
}

void RenderLayer::ClearAll() const {
  const auto scene = GetScene();
  if (!scene)
    return;
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  const auto current_render_instances = render_instances_list_[current_frame_index];
  current_render_instances->Clear();

  std::vector<std::pair<GlobalTransform, std::shared_ptr<Camera>>> cameras;
  RenderInstanceStorage::CollectCameras(scene, cameras);

  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    for (const auto& i : cameras) {
      if (const auto render_texture = i.second->GetRenderTexture())
        render_texture->Clear(vk_command_buffer);
    }
  });
}

void RenderLayer::PrepareForRendering() {
  const auto scene = GetScene();
  if (!scene)
    return;
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  auto& graphics = Platform::GetInstance();
  graphics.prim_count[current_frame_index] = 0;
  graphics.draw_call[current_frame_index] = 0;
  const auto current_render_instances = render_instances_list_[current_frame_index];
  ApplyAnimators();
  if (UpdateRenderInstanceStorage(scene, current_frame_index)) {
    per_frame_descriptor_sets_[current_frame_index]->UpdateBufferDescriptorBinding(
        0, current_render_instances->render_info_descriptor_buffer);
    per_frame_descriptor_sets_[current_frame_index]->UpdateBufferDescriptorBinding(
        1, current_render_instances->environment_info_descriptor_buffer);
    per_frame_descriptor_sets_[current_frame_index]->UpdateBufferDescriptorBinding(
        2, current_render_instances->camera_info_descriptor_buffer);
    per_frame_descriptor_sets_[current_frame_index]->UpdateBufferDescriptorBinding(
        3, current_render_instances->material_info_descriptor_buffer);
    per_frame_descriptor_sets_[current_frame_index]->UpdateBufferDescriptorBinding(
        4, current_render_instances->instance_info_descriptor_buffer);
    per_frame_descriptor_sets_[current_frame_index]->UpdateBufferDescriptorBinding(
        5, kernel_descriptor_buffers_[current_frame_index]);
    per_frame_descriptor_sets_[current_frame_index]->UpdateBufferDescriptorBinding(
        6, current_render_instances->directional_light_info_descriptor_buffer);
    per_frame_descriptor_sets_[current_frame_index]->UpdateBufferDescriptorBinding(
        7, current_render_instances->point_light_info_descriptor_buffer);
    per_frame_descriptor_sets_[current_frame_index]->UpdateBufferDescriptorBinding(
        8, current_render_instances->spot_light_info_descriptor_buffer);

    meshlet_descriptor_sets_[current_frame_index]->UpdateBufferDescriptorBinding(0, GeometryStorage::GetVertexBuffer());
    meshlet_descriptor_sets_[current_frame_index]->UpdateBufferDescriptorBinding(1,
                                                                                 GeometryStorage::GetMeshletBuffer());
    if (Platform::Constants::support_ray_tracing && Platform::Settings::use_ray_tracing) {
      current_render_instances->UpdateTopLevelAccelerationStructure(scene);
      if (current_render_instances->mesh_top_level_acceleration_structure) {
        ray_tracing_descriptor_sets_[current_frame_index]->UpdateBufferDescriptorBinding(
            0, GeometryStorage::GetVertexBuffer());
        ray_tracing_descriptor_sets_[current_frame_index]->UpdateBufferDescriptorBinding(
            1, GeometryStorage::GetTriangleBuffer());
        ray_tracing_descriptor_sets_[current_frame_index]->UpdateAccelerationStructureDescriptorBinding(
            2, current_render_instances->mesh_top_level_acceleration_structure);
      }
    }
  }
  TextureStorage::BindTexture2DToDescriptorSet(per_frame_descriptor_sets_[current_frame_index], 9);
  TextureStorage::BindCubemapToDescriptorSet(per_frame_descriptor_sets_[current_frame_index], 10);
}

void RenderLayer::RenderAll() {
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  const auto current_render_instances = render_instances_list_[current_frame_index];
  PreparePointAndSpotLightShadowMap();
  for (const auto& [cameraGlobalTransform, camera] : current_render_instances->cameras) {
    camera->rendered_ = false;
    if (camera->require_rendering_) {
      RenderToCamera(cameraGlobalTransform, camera);
    }
  }

  point_light_shadow_map_external_functions.clear();
  spot_light_shadow_map_external_functions.clear();
  directional_light_shadow_map_external_functions.clear();
  forward_rendering_external_functions.clear();

  if (Platform::Constants::support_ray_tracing && Platform::Settings::use_ray_tracing &&
      current_render_instances->mesh_top_level_acceleration_structure) {
    for (const auto& [cameraGlobalTransform, camera] : current_render_instances->cameras) {
      if (camera->require_rendering_) {
        RenderToCameraRayTracing(cameraGlobalTransform, camera);
      }
    }
  }
}

void RenderLayer::RenderGizmos() const {
  if (const auto scene = GetScene(); !scene)
    return;
  if (const auto editor_layer = Application::GetLayer<EditorLayer>()) {
    const auto current_frame_index = Platform::GetCurrentFrameIndex();
    const auto current_render_instances = render_instances_list_[Platform::GetCurrentFrameIndex()];
    for (const auto& i : editor_layer->gizmo_mesh_tasks_) {
      if (editor_layer->editor_cameras_.find(i.editor_camera_component->GetHandle()) ==
          editor_layer->editor_cameras_.end()) {
        EVOENGINE_ERROR("Target camera not registered in editor!");
        return;
      }
      if (i.editor_camera_component && i.editor_camera_component->IsEnabled()) {
        Platform::RecordCommandsMainQueue([&](VkCommandBuffer vk_command_buffer) {
          std::shared_ptr<GraphicsPipeline> gizmos_pipeline;
          switch (i.gizmo_settings.color_mode) {
            case GizmoSettings::ColorMode::Default: {
              gizmos_pipeline = Platform::GetGraphicsPipeline("GIZMOS");
            } break;
            case GizmoSettings::ColorMode::VertexColor: {
              gizmos_pipeline = Platform::GetGraphicsPipeline("GIZMOS_VERTEX_COLORED");
            } break;
            case GizmoSettings::ColorMode::NormalColor: {
              gizmos_pipeline = Platform::GetGraphicsPipeline("GIZMOS_NORMAL_COLORED");
            } break;
          }
          i.editor_camera_component->GetRenderTexture()->ApplyGraphicsPipelineStates(gizmos_pipeline->states);
          i.gizmo_settings.ApplySettings(gizmos_pipeline->states);

          gizmos_pipeline->Bind(vk_command_buffer);
          gizmos_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                             per_frame_descriptor_sets_[current_frame_index]->GetVkDescriptorSet());

          i.editor_camera_component->GetRenderTexture()->Render(
              vk_command_buffer, VK_ATTACHMENT_LOAD_OP_LOAD, VK_ATTACHMENT_STORE_OP_STORE, [&]() {
                GizmosPushConstant push_constant;
                push_constant.model = i.model;
                push_constant.color = i.color;
                push_constant.size = i.size;
                push_constant.camera_index =
                    current_render_instances->GetCameraIndex(i.editor_camera_component->GetHandle());
                gizmos_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
                GeometryStorage::BindVertices(vk_command_buffer);
                i.mesh->DrawIndexed(vk_command_buffer, gizmos_pipeline->states, 1);
              });
        });
      }
    }
    for (const auto& i : editor_layer->gizmo_instanced_mesh_tasks_) {
      if (editor_layer->editor_cameras_.find(i.editor_camera_component->GetHandle()) ==
          editor_layer->editor_cameras_.end()) {
        EVOENGINE_ERROR("Target camera not registered in editor!")
        return;
      }
      if (i.editor_camera_component && i.editor_camera_component->IsEnabled()) {
        Platform::RecordCommandsMainQueue([&](VkCommandBuffer vk_command_buffer) {
          const auto gizmos_pipeline = Platform::GetGraphicsPipeline("GIZMOS_INSTANCED_COLORED");
          i.editor_camera_component->GetRenderTexture()->ApplyGraphicsPipelineStates(gizmos_pipeline->states);
          i.gizmo_settings.ApplySettings(gizmos_pipeline->states);

          gizmos_pipeline->Bind(vk_command_buffer);
          gizmos_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                             per_frame_descriptor_sets_[current_frame_index]->GetVkDescriptorSet());
          gizmos_pipeline->BindDescriptorSet(vk_command_buffer, 1,
                                             i.instanced_data->GetDescriptorSet()->GetVkDescriptorSet());

          i.editor_camera_component->GetRenderTexture()->Render(
              vk_command_buffer, VK_ATTACHMENT_LOAD_OP_LOAD, VK_ATTACHMENT_STORE_OP_STORE, [&] {
                GizmosPushConstant push_constant;
                push_constant.model = i.model;
                push_constant.color = glm::vec4(0.0f);
                push_constant.size = i.size;
                push_constant.camera_index =
                    current_render_instances->GetCameraIndex(i.editor_camera_component->GetHandle());
                gizmos_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
                GeometryStorage::BindVertices(vk_command_buffer);
                i.mesh->DrawIndexed(vk_command_buffer, gizmos_pipeline->states,
                                    i.instanced_data->PeekParticleInfoList().size());
              });
        });
      }
    }
#ifdef EVOENGINE_WINDOWS
    for (const auto& i : editor_layer->gizmo_strands_tasks_) {
      if (editor_layer->editor_cameras_.find(i.editor_camera_component->GetHandle()) ==
          editor_layer->editor_cameras_.end()) {
        EVOENGINE_ERROR("Target camera not registered in editor!");
        return;
      }
      if (i.editor_camera_component && i.editor_camera_component->IsEnabled()) {
        Platform::RecordCommandsMainQueue([&](VkCommandBuffer vk_command_buffer) {
          std::shared_ptr<GraphicsPipeline> gizmos_pipeline;
          switch (i.gizmo_settings.color_mode) {
            case GizmoSettings::ColorMode::Default: {
              gizmos_pipeline = Platform::GetGraphicsPipeline("GIZMOS_STRANDS");
            } break;
            case GizmoSettings::ColorMode::VertexColor: {
              gizmos_pipeline = Platform::GetGraphicsPipeline("GIZMOS_STRANDS_VERTEX_COLORED");
            } break;
            case GizmoSettings::ColorMode::NormalColor: {
              gizmos_pipeline = Platform::GetGraphicsPipeline("GIZMOS_STRANDS_NORMAL_COLORED");
            } break;
          }
          i.editor_camera_component->GetRenderTexture()->ApplyGraphicsPipelineStates(gizmos_pipeline->states);
          i.gizmo_settings.ApplySettings(gizmos_pipeline->states);

          gizmos_pipeline->Bind(vk_command_buffer);
          gizmos_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                             per_frame_descriptor_sets_[current_frame_index]->GetVkDescriptorSet());

          i.editor_camera_component->GetRenderTexture()->Render(
              vk_command_buffer, VK_ATTACHMENT_LOAD_OP_LOAD, VK_ATTACHMENT_STORE_OP_STORE, [&] {
                GizmosPushConstant push_constant;
                push_constant.model = i.model;
                push_constant.color = i.color;
                push_constant.size = i.m_size;
                push_constant.camera_index =
                    current_render_instances->GetCameraIndex(i.editor_camera_component->GetHandle());
                gizmos_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
                GeometryStorage::BindStrandPoints(vk_command_buffer);
                i.m_strands->DrawIndexed(vk_command_buffer, gizmos_pipeline->states, 1);
              });
        });
      }
    }
#endif
  }
}

void RenderLayer::ForEachCollectedCamera(
    const std::function<void(const std::shared_ptr<Camera>& camera)>& action) const {
  const auto current_render_instances = render_instances_list_[Platform::GetCurrentFrameIndex()];
  for (const auto& camera : current_render_instances->cameras)
    action(camera.second);
}

std::shared_ptr<RenderInstanceStorage> RenderLayer::GetCurrentRenderInstanceStorage() const {
  return render_instances_list_[Platform::GetCurrentFrameIndex()];
}

void RenderLayer::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  if (ImGui::BeginMainMenuBar()) {
    if (ImGui::BeginMenu("View")) {
      ImGui::Checkbox("Render Settings", &enable_render_menu);
      ImGui::EndMenu();
    }
    ImGui::EndMainMenuBar();
  }

  if (enable_render_menu) {
    ImGui::Begin("Render Settings");

    ImGui::Checkbox("Count shadows drawcalls", &count_shadow_rendering_draw_calls);
    ImGui::Checkbox("Wireframe", &wire_frame);
    if (Platform::Constants::support_mesh_shader)
      ImGui::Checkbox("Meshlet", &Platform::Settings::use_mesh_shader);
    ImGui::Checkbox("Indirect Rendering", &enable_indirect_rendering);
    render_settings.OnInspect(editor_layer);
    ImGui::End();
  }
}

void RenderLayer::ApplyAnimators() const {
  const auto scene = GetScene();
  if (const auto* owners = scene->UnsafeGetPrivateComponentOwnersList<Animator>()) {
    Jobs::RunParallelFor(owners->size(), [&](unsigned i) {
      const auto entity = owners->at(i);
      if (!scene->IsEntityEnabled(entity))
        return;
      const auto animator = scene->GetOrSetPrivateComponent<Animator>(owners->at(i)).lock();
      if (!animator->IsEnabled())
        return;
      animator->Apply();
    });
  }
  if (const auto* owners = scene->UnsafeGetPrivateComponentOwnersList<SkinnedMeshRenderer>()) {
    Jobs::RunParallelFor(owners->size(), [&](unsigned i) {
      const auto entity = owners->at(i);
      if (!scene->IsEntityEnabled(entity))
        return;
      const auto skinned_mesh_renderer = scene->GetOrSetPrivateComponent<SkinnedMeshRenderer>(entity).lock();
      if (!skinned_mesh_renderer->IsEnabled())
        return;
      skinned_mesh_renderer->UpdateBoneMatrices();
    });
    for (const auto& i : *owners) {
      if (!scene->IsEntityEnabled(i))
        return;
      const auto skinned_mesh_renderer = scene->GetOrSetPrivateComponent<SkinnedMeshRenderer>(i).lock();
      if (!skinned_mesh_renderer->IsEnabled())
        return;
      skinned_mesh_renderer->UpdateBoneMatrices();
      skinned_mesh_renderer->bone_matrices->UploadData();
    }
  }
}

void RenderLayer::PreparePointAndSpotLightShadowMap() const {
  const bool count_draw_calls = count_shadow_rendering_draw_calls;
  const bool use_mesh_shader = Platform::Constants::support_mesh_shader && Platform::Settings::use_mesh_shader;
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  const auto& point_light_shadow_pipeline = use_mesh_shader
                                                ? Platform::GetGraphicsPipeline("POINT_LIGHT_SHADOW_MAP_MESH")
                                                : Platform::GetGraphicsPipeline("POINT_LIGHT_SHADOW_MAP");
  const auto& spot_light_shadow_pipeline = use_mesh_shader ? Platform::GetGraphicsPipeline("SPOT_LIGHT_SHADOW_MAP_MESH")
                                                           : Platform::GetGraphicsPipeline("SPOT_LIGHT_SHADOW_MAP");

  const auto& point_light_shadow_skinned_pipeline = Platform::GetGraphicsPipeline("POINT_LIGHT_SHADOW_MAP_SKINNED");
  const auto& spot_light_shadow_skinned_pipeline = Platform::GetGraphicsPipeline("SPOT_LIGHT_SHADOW_MAP_SKINNED");

  const auto& point_light_shadow_instanced_pipeline = Platform::GetGraphicsPipeline("POINT_LIGHT_SHADOW_MAP_INSTANCED");
  const auto& spot_light_shadow_instanced_pipeline = Platform::GetGraphicsPipeline("SPOT_LIGHT_SHADOW_MAP_INSTANCED");

  const auto& point_light_shadow_strands_pipeline = Platform::GetGraphicsPipeline("POINT_LIGHT_SHADOW_MAP_STRANDS");
  const auto& spot_light_shadow_strands_pipeline = Platform::GetGraphicsPipeline("SPOT_LIGHT_SHADOW_MAP_STRANDS");
  auto& platform = Platform::GetInstance();
  const auto current_render_instances = render_instances_list_[current_frame_index];
  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    const auto prepare_graphics_pipeline = [&](const std::shared_ptr<GraphicsPipeline>& target_pipeline,
                                               const glm::ivec4& view_port) {
      target_pipeline->states.ResetAllStates(0);
      target_pipeline->Bind(vk_command_buffer);
      target_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                         per_frame_descriptor_sets_[current_frame_index]->GetVkDescriptorSet());
      target_pipeline->states.SetViewportScissor(view_port);
    };

    VkRect2D render_area;
    render_area.offset = {0, 0};
    render_area.extent.width = lighting_->point_light_shadow_map_->GetExtent().width;
    render_area.extent.height = lighting_->point_light_shadow_map_->GetExtent().height;
    lighting_->point_light_shadow_map_->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL);

    for (int face = 0; face < 6; face++) {
      VkRenderingInfo render_info{};
      auto depth_attachment = lighting_->GetLayeredPointLightDepthAttachmentInfo(face, VK_ATTACHMENT_LOAD_OP_CLEAR,
                                                                                 VK_ATTACHMENT_STORE_OP_STORE);
      render_info.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
      render_info.renderArea = render_area;
      render_info.layerCount = 1;
      render_info.colorAttachmentCount = 0;
      render_info.pColorAttachments = nullptr;
      render_info.pDepthAttachment = &depth_attachment;
      Platform::RecordRenderCommands(render_info, vk_command_buffer, [&]() {
        for (int i = 0; i < current_render_instances->point_light_info_blocks_.size(); i++) {
          GeometryStorage::BindVertices(vk_command_buffer);
          {
            prepare_graphics_pipeline(point_light_shadow_pipeline,
                                      current_render_instances->point_light_info_blocks_[i].viewport);
            if (use_mesh_shader) {
              point_light_shadow_pipeline->BindDescriptorSet(
                  vk_command_buffer, 1, meshlet_descriptor_sets_[current_frame_index]->GetVkDescriptorSet());
            }
            if (enable_indirect_rendering &&
                !current_render_instances->deferred_render_instances.render_commands.empty()) {
              RenderInstancePushConstant push_constant;
              push_constant.camera_index = i;
              push_constant.light_split_index = face;
              push_constant.instance_index = 0;
              point_light_shadow_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
              point_light_shadow_pipeline->states.ApplyAllStates(vk_command_buffer);
              if (count_draw_calls)
                platform.draw_call[current_frame_index]++;
              if (count_draw_calls)
                platform.prim_count[current_frame_index] += current_render_instances->total_mesh_triangles;
              if (use_mesh_shader) {
                vkCmdDrawMeshTasksIndirectEXT(
                    vk_command_buffer,
                    current_render_instances->mesh_draw_mesh_tasks_indirect_commands_buffer->GetVkBuffer(), 0,
                    current_render_instances->mesh_draw_mesh_tasks_indirect_commands.size(),
                    sizeof(VkDrawMeshTasksIndirectCommandEXT));
              } else {
                vkCmdDrawIndexedIndirect(
                    vk_command_buffer,
                    current_render_instances->mesh_draw_indexed_indirect_commands_buffer->GetVkBuffer(), 0,
                    current_render_instances->mesh_draw_indexed_indirect_commands.size(),
                    sizeof(VkDrawIndexedIndirectCommand));
              }
            } else {
              for (const auto& render_command : current_render_instances->deferred_render_instances.render_commands) {
                if (!render_command.cast_shadow)
                  continue;
                RenderInstancePushConstant push_constant;
                push_constant.camera_index = i;
                push_constant.light_split_index = face;
                push_constant.instance_index = render_command.instance_index;
                const auto prim_count =
                    render_command.Render(vk_command_buffer, push_constant, point_light_shadow_pipeline);
                if (count_draw_calls) {
                  platform.draw_call[current_frame_index]++;
                  platform.prim_count[current_frame_index] += prim_count;
                }
              }
            }
          }
          {
            prepare_graphics_pipeline(point_light_shadow_instanced_pipeline,
                                      current_render_instances->point_light_info_blocks_[i].viewport);
            for (const auto& render_command :
                 current_render_instances->deferred_instanced_render_instances.render_commands) {
              if (!render_command.cast_shadow)
                continue;
              RenderInstancePushConstant push_constant;
              push_constant.camera_index = i;
              push_constant.light_split_index = face;
              push_constant.instance_index = render_command.instance_index;
              const auto prim_count =
                  render_command.Render(vk_command_buffer, push_constant, point_light_shadow_instanced_pipeline);
              if (count_draw_calls) {
                platform.draw_call[current_frame_index]++;
                platform.prim_count[current_frame_index] += prim_count;
              }
            }
          }
          GeometryStorage::BindSkinnedVertices(vk_command_buffer);
          {
            prepare_graphics_pipeline(point_light_shadow_skinned_pipeline,
                                      current_render_instances->point_light_info_blocks_[i].viewport);
            for (const auto& render_command :
                 current_render_instances->deferred_skinned_render_instances.render_commands) {
              if (!render_command.cast_shadow)
                continue;
              RenderInstancePushConstant push_constant;
              push_constant.camera_index = i;
              push_constant.light_split_index = face;
              push_constant.instance_index = render_command.instance_index;
              const auto prim_count =
                  render_command.Render(vk_command_buffer, push_constant, point_light_shadow_skinned_pipeline);
              if (count_draw_calls) {
                platform.draw_call[current_frame_index]++;
                platform.prim_count[current_frame_index] += prim_count;
              }
            }
          }
#ifdef EVOENGINE_WINDOWS
          GeometryStorage::BindStrandPoints(vk_command_buffer);
          {
            prepare_graphics_pipeline(point_light_shadow_strands_pipeline,
                                      current_render_instances->point_light_info_blocks_[i].viewport);
            for (const auto& render_command :
                 current_render_instances->deferred_strands_render_instances.render_commands) {
              if (!render_command.cast_shadow)
                continue;
              RenderInstancePushConstant push_constant;
              push_constant.camera_index = i;
              push_constant.light_split_index = face;
              push_constant.instance_index = render_command.instance_index;
              const auto prim_count =
                  render_command.Render(vk_command_buffer, push_constant, point_light_shadow_strands_pipeline);
              if (count_draw_calls) {
                platform.draw_call[current_frame_index]++;
                platform.prim_count[current_frame_index] += prim_count;
              }
            }
          }
#endif
          for (const auto& func : point_light_shadow_map_external_functions) {
            const auto prim_count =
                func(vk_command_buffer, {i, face, current_render_instances->point_light_info_blocks_[i].viewport});
            if (count_draw_calls) {
              platform.draw_call[current_frame_index]++;
              platform.prim_count[current_frame_index] += prim_count;
            }
          }
        }
      });
    }
#pragma region Viewport and scissor

    render_area.offset = {0, 0};
    render_area.extent.width = lighting_->spot_light_shadow_map_->GetExtent().width;
    render_area.extent.height = lighting_->spot_light_shadow_map_->GetExtent().height;

#pragma endregion
    lighting_->spot_light_shadow_map_->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL);
    VkRenderingInfo render_info{};
    const auto depth_attachment =
        lighting_->GetSpotLightDepthAttachmentInfo(VK_ATTACHMENT_LOAD_OP_CLEAR, VK_ATTACHMENT_STORE_OP_STORE);
    render_info.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
    render_info.renderArea = render_area;
    render_info.layerCount = 1;
    render_info.colorAttachmentCount = 0;
    render_info.pColorAttachments = nullptr;
    render_info.pDepthAttachment = &depth_attachment;
    Platform::RecordRenderCommands(render_info, vk_command_buffer, [&]() {
      for (int i = 0; i < current_render_instances->spot_light_info_blocks_.size(); i++) {
        GeometryStorage::BindVertices(vk_command_buffer);
        {
          prepare_graphics_pipeline(spot_light_shadow_pipeline,
                                    current_render_instances->spot_light_info_blocks_[i].viewport);
          if (use_mesh_shader) {
            spot_light_shadow_pipeline->BindDescriptorSet(
                vk_command_buffer, 1, meshlet_descriptor_sets_[current_frame_index]->GetVkDescriptorSet());
          }
          if (enable_indirect_rendering &&
              !current_render_instances->deferred_render_instances.render_commands.empty()) {
            RenderInstancePushConstant push_constant;
            push_constant.camera_index = i;
            push_constant.light_split_index = 0;
            push_constant.instance_index = 0;
            spot_light_shadow_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
            spot_light_shadow_pipeline->states.ApplyAllStates(vk_command_buffer);
            if (count_draw_calls)
              platform.draw_call[current_frame_index]++;
            if (count_draw_calls)
              platform.prim_count[current_frame_index] += current_render_instances->total_mesh_triangles;
            if (use_mesh_shader) {
              vkCmdDrawMeshTasksIndirectEXT(
                  vk_command_buffer,
                  current_render_instances->mesh_draw_mesh_tasks_indirect_commands_buffer->GetVkBuffer(), 0,
                  current_render_instances->mesh_draw_mesh_tasks_indirect_commands.size(),
                  sizeof(VkDrawMeshTasksIndirectCommandEXT));
            } else {
              vkCmdDrawIndexedIndirect(
                  vk_command_buffer,
                  current_render_instances->mesh_draw_indexed_indirect_commands_buffer->GetVkBuffer(), 0,
                  current_render_instances->mesh_draw_indexed_indirect_commands.size(),
                  sizeof(VkDrawIndexedIndirectCommand));
            }
          } else {
            for (const auto& render_command : current_render_instances->deferred_render_instances.render_commands) {
              if (!render_command.cast_shadow)
                return;
              RenderInstancePushConstant push_constant;
              push_constant.camera_index = i;
              push_constant.light_split_index = 0;
              push_constant.instance_index = render_command.instance_index;
              const auto prim_count =
                  render_command.Render(vk_command_buffer, push_constant, spot_light_shadow_pipeline);
              if (count_draw_calls) {
                platform.draw_call[current_frame_index]++;
                platform.prim_count[current_frame_index] += prim_count;
              }
            }
          }
        }
        {
          prepare_graphics_pipeline(spot_light_shadow_instanced_pipeline,
                                    current_render_instances->spot_light_info_blocks_[i].viewport);
          for (const auto& render_command :
               current_render_instances->deferred_instanced_render_instances.render_commands) {
            if (!render_command.cast_shadow)
              continue;
            RenderInstancePushConstant push_constant;
            push_constant.camera_index = i;
            push_constant.light_split_index = 0;
            push_constant.instance_index = render_command.instance_index;
            const auto prim_count =
                render_command.Render(vk_command_buffer, push_constant, spot_light_shadow_instanced_pipeline);
            if (count_draw_calls) {
              platform.draw_call[current_frame_index]++;
              platform.prim_count[current_frame_index] += prim_count;
            }
          }
        }
        GeometryStorage::BindSkinnedVertices(vk_command_buffer);
        {
          prepare_graphics_pipeline(spot_light_shadow_skinned_pipeline,
                                    current_render_instances->spot_light_info_blocks_[i].viewport);
          for (const auto& render_command :
               current_render_instances->deferred_skinned_render_instances.render_commands) {
            if (!render_command.cast_shadow)
              return;
            RenderInstancePushConstant push_constant;
            push_constant.camera_index = i;
            push_constant.light_split_index = 0;
            push_constant.instance_index = render_command.instance_index;
            const auto prim_count =
                render_command.Render(vk_command_buffer, push_constant, spot_light_shadow_skinned_pipeline);
            if (count_draw_calls) {
              platform.draw_call[current_frame_index]++;
              platform.prim_count[current_frame_index] += prim_count;
            }
          }
        }
#ifdef EVOENGINE_WINDOWS
        GeometryStorage::BindStrandPoints(vk_command_buffer);
        {
          prepare_graphics_pipeline(spot_light_shadow_strands_pipeline,
                                    current_render_instances->spot_light_info_blocks_[i].viewport);
          for (const auto& render_command :
               current_render_instances->deferred_strands_render_instances.render_commands) {
            if (!render_command.cast_shadow)
              continue;
            RenderInstancePushConstant push_constant;
            push_constant.camera_index = i;
            push_constant.light_split_index = 0;
            push_constant.instance_index = render_command.instance_index;
            const auto prim_count =
                render_command.Render(vk_command_buffer, push_constant, spot_light_shadow_strands_pipeline);
            if (count_draw_calls) {
              platform.draw_call[current_frame_index]++;
              platform.prim_count[current_frame_index] += prim_count;
            }
          }
        }
#endif
        for (const auto& func : spot_light_shadow_map_external_functions) {
          const auto prim_count =
              func(vk_command_buffer, {i, current_render_instances->spot_light_info_blocks_[i].viewport});
          if (count_draw_calls) {
            platform.draw_call[current_frame_index]++;
            platform.prim_count[current_frame_index] += prim_count;
          }
        }
      }
    });
    lighting_->point_light_shadow_map_->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    lighting_->spot_light_shadow_map_->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
  });
}

bool RenderLayer::UpdateRenderInstanceStorage(const std::shared_ptr<Scene>& scene, const uint32_t current_frame_index) {
  auto lod_center = glm::vec3(0.f);
  float lod_max_distance = FLT_MAX;
  bool lod_set = false;
  if (const auto main_camera = scene->main_camera.Get<Camera>()) {
    if (const auto main_camera_owner = main_camera->GetOwner(); scene->IsEntityValid(main_camera_owner)) {
      lod_center = scene->GetDataComponent<GlobalTransform>(main_camera_owner).GetPosition();
      lod_max_distance = main_camera->far_distance;
      lod_set = true;
    }
  }
  if (!lod_set) {
    if (const auto editor_layer = Application::GetLayer<EditorLayer>()) {
      if (const auto scene_camera = editor_layer->GetSceneCamera()) {
        lod_center = editor_layer->GetSceneCameraPosition();
        lod_max_distance = scene_camera->far_distance;
      }
    }
  }
  RenderInstanceStorage::CalculateLodFactor(scene, lod_center, lod_max_distance);
  Bound world_bound{};
  need_fade_ = false;
  const auto current_render_instances = render_instances_list_[current_frame_index];
  current_render_instances->BuildFromScene(render_settings, scene, world_bound);
  const bool render_instance_updated =
      current_render_instances != render_instances_list_[(current_frame_index + Platform::GetMaxFramesInFlight() - 1) %
                                                         Platform::GetMaxFramesInFlight()];
  if (render_instance_updated) {
    current_render_instances->Upload();
  }
  if (const auto editor_layer = Application::GetLayer<EditorLayer>()) {
    if (scene->IsEntityValid(editor_layer->GetSelectedEntity())) {
      for (const auto& i : current_render_instances->instance_info_blocks_) {
        if (i.entity_selected) {
          need_fade_ = true;
        }
      }
    }
    editor_layer->MouseEntitySelection();
  }
  if (render_instance_updated) {
    world_bound.min -= glm::vec3(0.1f);
    world_bound.max += glm::vec3(0.1f);
    scene->SetBound(world_bound);
    current_render_instances->Upload();
  }
  return render_instance_updated;
}

void RenderLayer::PrepareEnvironmentalBrdfLut() {
  environmental_brdf_lut_.reset();
  environmental_brdf_lut_ = ProjectManager::CreateTemporaryAsset<Texture2D>();
  auto& environmental_brdf_lut_texture_storage = environmental_brdf_lut_->RefTexture2DStorage();
  constexpr auto brdf_lut_resolution = 512;
  {
    VkImageCreateInfo image_info{};
    image_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
    image_info.imageType = VK_IMAGE_TYPE_2D;
    image_info.extent.width = brdf_lut_resolution;
    image_info.extent.height = brdf_lut_resolution;
    image_info.extent.depth = 1;
    image_info.mipLevels = 1;
    image_info.arrayLayers = 1;
    image_info.format = VK_FORMAT_R16G16_SFLOAT;
    image_info.tiling = VK_IMAGE_TILING_OPTIMAL;
    image_info.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    image_info.usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
    image_info.samples = VK_SAMPLE_COUNT_1_BIT;
    image_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

    environmental_brdf_lut_texture_storage.image = std::make_unique<Image>(image_info);

    VkImageViewCreateInfo view_info{};
    view_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
    view_info.image = environmental_brdf_lut_->GetVkImage();
    view_info.viewType = VK_IMAGE_VIEW_TYPE_2D;
    view_info.format = VK_FORMAT_R16G16_SFLOAT;
    view_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    view_info.subresourceRange.baseMipLevel = 0;
    view_info.subresourceRange.levelCount = 1;
    view_info.subresourceRange.baseArrayLayer = 0;
    view_info.subresourceRange.layerCount = 1;

    environmental_brdf_lut_texture_storage.image_view = std::make_unique<ImageView>(view_info);

    VkSamplerCreateInfo sampler_info{};
    sampler_info.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
    sampler_info.magFilter = VK_FILTER_LINEAR;
    sampler_info.minFilter = VK_FILTER_LINEAR;
    sampler_info.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    sampler_info.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    sampler_info.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    sampler_info.anisotropyEnable = VK_TRUE;
    sampler_info.maxAnisotropy = Platform::GetSelectedPhysicalDevice()->properties.limits.maxSamplerAnisotropy;
    sampler_info.borderColor = VK_BORDER_COLOR_INT_OPAQUE_BLACK;
    sampler_info.unnormalizedCoordinates = VK_FALSE;
    sampler_info.compareEnable = VK_FALSE;
    sampler_info.compareOp = VK_COMPARE_OP_ALWAYS;
    sampler_info.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;

    environmental_brdf_lut_texture_storage.sampler = std::make_unique<Sampler>(sampler_info);
  }
  const auto environmental_brdf_pipeline = Platform::GetGraphicsPipeline("ENVIRONMENTAL_MAP_BRDF");
  Platform::ImmediateSubmit([&](VkCommandBuffer vk_command_buffer) {
    environmental_brdf_lut_texture_storage.image->TransitImageLayout(vk_command_buffer,
                                                                     VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL);
#pragma region Viewport and scissor
    VkRect2D render_area;
    render_area.offset = {0, 0};
    render_area.extent.width = brdf_lut_resolution;
    render_area.extent.height = brdf_lut_resolution;
    VkViewport viewport;
    viewport.x = 0.0f;
    viewport.y = 0.0f;
    viewport.width = brdf_lut_resolution;
    viewport.height = brdf_lut_resolution;
    viewport.minDepth = 0.0f;
    viewport.maxDepth = 1.0f;

    VkRect2D scissor;
    scissor.offset = {0, 0};
    scissor.extent.width = brdf_lut_resolution;
    scissor.extent.height = brdf_lut_resolution;
    environmental_brdf_pipeline->states.view_port = viewport;
    environmental_brdf_pipeline->states.scissor = scissor;
#pragma endregion
#pragma region Lighting pass
    {
      VkRenderingAttachmentInfo attachment{};
      attachment.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;

      attachment.imageLayout = VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL;
      attachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
      attachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;

      attachment.clearValue = {0, 0, 0, 1};
      attachment.imageView = environmental_brdf_lut_texture_storage.image_view->GetVkImageView();

      VkRenderingInfo render_info{};
      render_info.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
      render_info.renderArea = render_area;
      render_info.layerCount = 1;
      render_info.colorAttachmentCount = 1;
      render_info.pColorAttachments = &attachment;
      environmental_brdf_pipeline->states.depth_test = false;
      environmental_brdf_pipeline->states.color_blend_attachment_states.clear();
      environmental_brdf_pipeline->states.color_blend_attachment_states.resize(1);
      for (auto& i : environmental_brdf_pipeline->states.color_blend_attachment_states) {
        i.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT;
        i.blendEnable = VK_FALSE;
      }
      Platform::RecordRenderCommands(render_info, vk_command_buffer, [&]() {
        environmental_brdf_pipeline->Bind(vk_command_buffer);
        const auto mesh = Resources::GetResource<Mesh>("PRIMITIVE_TEX_PASS_THROUGH");
        GeometryStorage::BindVertices(vk_command_buffer);
        mesh->DrawIndexed(vk_command_buffer, environmental_brdf_pipeline->states, 1);
      });
#pragma endregion
    }
    environmental_brdf_lut_texture_storage.image->TransitImageLayout(vk_command_buffer,
                                                                     VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
  });
}
void RenderLayer::RenderToCamera(const GlobalTransform& camera_global_transform,
                                 const std::shared_ptr<Camera>& camera) const {
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  const auto current_render_instances = render_instances_list_[current_frame_index];
  const int camera_index = current_render_instances->GetCameraIndex(camera->GetHandle());
  const auto scene = Application::GetActiveScene();
  if (camera->camera_render_mode == Camera::CameraRenderMode::Rasterization) {
    const bool count_draw_calls = count_shadow_rendering_draw_calls;
    const bool use_mesh_shader = Platform::Constants::support_mesh_shader && Platform::Settings::use_mesh_shader;
#pragma region Directional Light Shadows
    const auto& directional_light_shadow_pipeline =
        use_mesh_shader ? Platform::GetGraphicsPipeline("DIRECTIONAL_LIGHT_SHADOW_MAP_MESH")
                        : Platform::GetGraphicsPipeline("DIRECTIONAL_LIGHT_SHADOW_MAP");
    const auto& directional_light_shadow_pipeline_skinned =
        Platform::GetGraphicsPipeline("DIRECTIONAL_LIGHT_SHADOW_MAP_SKINNED");
    const auto& directional_light_shadow_pipeline_instanced =
        Platform::GetGraphicsPipeline("DIRECTIONAL_LIGHT_SHADOW_MAP_INSTANCED");
    const auto& directional_light_shadow_pipeline_strands =
        Platform::GetGraphicsPipeline("DIRECTIONAL_LIGHT_SHADOW_MAP_STRANDS");
    auto& platform = Platform::GetInstance();
    Platform::RecordCommandsMainQueue([&](VkCommandBuffer vk_command_buffer) {
#pragma region Viewport and scissor
      VkRect2D render_area;
      render_area.offset = {0, 0};
      render_area.extent.width = lighting_->directional_light_shadow_map_->GetExtent().width;
      render_area.extent.height = lighting_->directional_light_shadow_map_->GetExtent().height;
#pragma endregion
      lighting_->directional_light_shadow_map_->TransitImageLayout(vk_command_buffer,
                                                                   VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL);
      for (int split = 0; split < 4; split++) {
        const auto depth_attachment = lighting_->GetLayeredDirectionalLightDepthAttachmentInfo(
            split, VK_ATTACHMENT_LOAD_OP_CLEAR, VK_ATTACHMENT_STORE_OP_STORE);
        VkRenderingInfo render_info{};
        render_info.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
        render_info.renderArea = render_area;
        render_info.layerCount = 1;
        render_info.colorAttachmentCount = 0;
        render_info.pColorAttachments = nullptr;
        render_info.pDepthAttachment = &depth_attachment;
        Platform::RecordRenderCommands(render_info, vk_command_buffer, [&]() {
          if (use_mesh_shader) {
            directional_light_shadow_pipeline->BindDescriptorSet(
                vk_command_buffer, 1, meshlet_descriptor_sets_[current_frame_index]->GetVkDescriptorSet());
          }
          for (int i = 0; i < current_render_instances->render_info_block.directional_light_size; i++) {
            const auto& directional_light_info_block =
                current_render_instances
                    ->directional_light_info_blocks_[camera_index * Platform::Settings::max_directional_light_size + i];
            const auto prepare_graphics_pipeline = [&](const std::shared_ptr<GraphicsPipeline>& target_pipeline) {
              target_pipeline->states.ResetAllStates(0);
              target_pipeline->Bind(vk_command_buffer);
              target_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                                 per_frame_descriptor_sets_[current_frame_index]->GetVkDescriptorSet());
              target_pipeline->states.SetViewportScissor(directional_light_info_block.viewport);
            };
            GeometryStorage::BindVertices(vk_command_buffer);
            {
              prepare_graphics_pipeline(directional_light_shadow_pipeline);
              if (enable_indirect_rendering &&
                  !current_render_instances->deferred_render_instances.render_commands.empty()) {
                RenderInstancePushConstant push_constant;
                push_constant.camera_index = camera_index * Platform::Settings::max_directional_light_size + i;
                push_constant.light_split_index = split;
                push_constant.instance_index = 0;
                directional_light_shadow_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
                directional_light_shadow_pipeline->states.ApplyAllStates(vk_command_buffer);
                if (count_draw_calls)
                  platform.draw_call[current_frame_index]++;
                if (count_draw_calls)
                  platform.prim_count[current_frame_index] += current_render_instances->total_mesh_triangles;
                if (use_mesh_shader) {
                  vkCmdDrawMeshTasksIndirectEXT(
                      vk_command_buffer,
                      current_render_instances->mesh_draw_mesh_tasks_indirect_commands_buffer->GetVkBuffer(), 0,
                      current_render_instances->mesh_draw_mesh_tasks_indirect_commands.size(),
                      sizeof(VkDrawMeshTasksIndirectCommandEXT));
                } else {
                  vkCmdDrawIndexedIndirect(
                      vk_command_buffer,
                      current_render_instances->mesh_draw_indexed_indirect_commands_buffer->GetVkBuffer(), 0,
                      current_render_instances->mesh_draw_indexed_indirect_commands.size(),
                      sizeof(VkDrawIndexedIndirectCommand));
                }
              } else {
                for (const auto& render_command : current_render_instances->deferred_render_instances.render_commands) {
                  if (!render_command.cast_shadow)
                    continue;
                  RenderInstancePushConstant push_constant;
                  push_constant.camera_index = camera_index * Platform::Settings::max_directional_light_size + i;
                  push_constant.light_split_index = split;
                  push_constant.instance_index = render_command.instance_index;
                  const auto prim_count =
                      render_command.Render(vk_command_buffer, push_constant, directional_light_shadow_pipeline);
                  if (count_draw_calls) {
                    platform.draw_call[current_frame_index]++;
                    platform.prim_count[current_frame_index] += prim_count;
                  }
                }
              }
            }
            {
              prepare_graphics_pipeline(directional_light_shadow_pipeline_instanced);
              for (const auto& render_command :
                   current_render_instances->deferred_instanced_render_instances.render_commands) {
                if (!render_command.cast_shadow)
                  continue;
                RenderInstancePushConstant push_constant;
                push_constant.camera_index = camera_index * Platform::Settings::max_directional_light_size + i;
                push_constant.light_split_index = split;
                push_constant.instance_index = render_command.instance_index;
                const auto prim_count = render_command.Render(vk_command_buffer, push_constant,
                                                              directional_light_shadow_pipeline_instanced);
                if (count_draw_calls) {
                  platform.draw_call[current_frame_index]++;
                  platform.prim_count[current_frame_index] += prim_count;
                }
              }
            }
            GeometryStorage::BindSkinnedVertices(vk_command_buffer);
            {
              prepare_graphics_pipeline(directional_light_shadow_pipeline_skinned);
              for (const auto& render_command :
                   current_render_instances->deferred_skinned_render_instances.render_commands) {
                if (!render_command.cast_shadow)
                  continue;
                RenderInstancePushConstant push_constant;
                push_constant.camera_index = camera_index * Platform::Settings::max_directional_light_size + i;
                push_constant.light_split_index = split;
                push_constant.instance_index = render_command.instance_index;
                const auto prim_count =
                    render_command.Render(vk_command_buffer, push_constant, directional_light_shadow_pipeline_skinned);
                if (count_draw_calls) {
                  platform.draw_call[current_frame_index]++;
                  platform.prim_count[current_frame_index] += prim_count;
                }
              }
            }
#ifdef EVOENGINE_WINDOWS
            GeometryStorage::BindStrandPoints(vk_command_buffer);
            {
              prepare_graphics_pipeline(directional_light_shadow_pipeline_strands);
              for (const auto& render_command :
                   current_render_instances->deferred_strands_render_instances.render_commands) {
                if (!render_command.cast_shadow)
                  continue;
                RenderInstancePushConstant push_constant;
                push_constant.camera_index = camera_index * Platform::Settings::max_directional_light_size + i;
                push_constant.light_split_index = split;
                push_constant.instance_index = render_command.instance_index;
                const auto prim_count =
                    render_command.Render(vk_command_buffer, push_constant, directional_light_shadow_pipeline_strands);
                if (count_draw_calls) {
                  platform.draw_call[current_frame_index]++;
                  platform.prim_count[current_frame_index] += prim_count;
                }
              }
            }
#endif
            for (const auto& func : directional_light_shadow_map_external_functions) {
              const auto prim_count = func(
                  vk_command_buffer, {i, split, current_render_instances->directional_light_info_blocks_[i].viewport});
              if (count_draw_calls) {
                platform.draw_call[current_frame_index]++;
                platform.prim_count[current_frame_index] += prim_count;
              }
            }
          }
        });
      }
    });

#pragma endregion
    const auto editor_layer = Application::GetLayer<EditorLayer>();
    bool is_scene_camera = false;
    bool need_fade = false;
    if (editor_layer) {
      if (camera.get() == editor_layer->GetSceneCamera().get())
        is_scene_camera = true;
      if (need_fade_ && editor_layer->highlight_selection_)
        need_fade = true;
    }

    Platform::RecordCommandsMainQueue([&](VkCommandBuffer vk_command_buffer) {
#pragma region Viewport and scissor
      VkRect2D render_area;
      render_area.offset = {0, 0};
      render_area.extent.width = camera->GetSize().x;
      render_area.extent.height = camera->GetSize().y;
      glm::ivec4 view_port;
      view_port.x = 0.0f;
      view_port.y = 0.0f;
      view_port.z = camera->GetSize().x;
      view_port.w = camera->GetSize().y;

      camera->TransitGBufferImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL);
      camera->render_texture_->GetDepthImage()->TransitImageLayout(vk_command_buffer,
                                                                   VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL);

      VkRenderingInfo geometry_pass_render_info{};
      geometry_pass_render_info.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
      geometry_pass_render_info.renderArea = render_area;
      geometry_pass_render_info.layerCount = 1;
#pragma endregion
#pragma region Deferred Rendering
#pragma region Geometry pass

      const auto geometry_pass_depth_attachment =
          camera->render_texture_->GetDepthAttachmentInfo(VK_ATTACHMENT_LOAD_OP_CLEAR, VK_ATTACHMENT_STORE_OP_STORE);
      geometry_pass_render_info.pDepthAttachment = &geometry_pass_depth_attachment;
      std::vector<VkRenderingAttachmentInfo> geometry_pass_color_attachment_infos;
      camera->AppendGBufferColorAttachmentInfos(geometry_pass_color_attachment_infos, VK_ATTACHMENT_LOAD_OP_CLEAR,
                                                VK_ATTACHMENT_STORE_OP_STORE);
      geometry_pass_render_info.colorAttachmentCount = geometry_pass_color_attachment_infos.size();
      geometry_pass_render_info.pColorAttachments = geometry_pass_color_attachment_infos.data();
      Platform::RecordRenderCommands(geometry_pass_render_info, vk_command_buffer, [&]() {
        GeometryStorage::BindVertices(vk_command_buffer);
        {
          const auto& deferred_prepass_pipeline = use_mesh_shader
                                                      ? Platform::GetGraphicsPipeline("STANDARD_DEFERRED_PREPASS_MESH")
                                                      : Platform::GetGraphicsPipeline("STANDARD_DEFERRED_PREPASS");
          deferred_prepass_pipeline->states.ResetAllStates(geometry_pass_color_attachment_infos.size());
          deferred_prepass_pipeline->states.SetViewportScissor(view_port);
          deferred_prepass_pipeline->states.polygon_mode = wire_frame ? VK_POLYGON_MODE_LINE : VK_POLYGON_MODE_FILL;
          deferred_prepass_pipeline->Bind(vk_command_buffer);
          deferred_prepass_pipeline->BindDescriptorSet(
              vk_command_buffer, 0, per_frame_descriptor_sets_[current_frame_index]->GetVkDescriptorSet());
          if (use_mesh_shader) {
            deferred_prepass_pipeline->BindDescriptorSet(
                vk_command_buffer, 1, meshlet_descriptor_sets_[current_frame_index]->GetVkDescriptorSet());
          }
          if (enable_indirect_rendering &&
              !current_render_instances->deferred_render_instances.render_commands.empty()) {
            RenderInstancePushConstant push_constant;
            push_constant.camera_index = camera_index;
            push_constant.instance_index = 0;
            deferred_prepass_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
            deferred_prepass_pipeline->states.ApplyAllStates(vk_command_buffer);
            if (count_draw_calls)
              platform.draw_call[current_frame_index]++;
            if (count_draw_calls)
              platform.prim_count[current_frame_index] += current_render_instances->total_mesh_triangles;
            if (use_mesh_shader) {
              vkCmdDrawMeshTasksIndirectEXT(
                  vk_command_buffer,
                  current_render_instances->mesh_draw_mesh_tasks_indirect_commands_buffer->GetVkBuffer(), 0,
                  current_render_instances->mesh_draw_mesh_tasks_indirect_commands.size(),
                  sizeof(VkDrawMeshTasksIndirectCommandEXT));
            } else {
              vkCmdDrawIndexedIndirect(
                  vk_command_buffer,
                  current_render_instances->mesh_draw_indexed_indirect_commands_buffer->GetVkBuffer(), 0,
                  current_render_instances->mesh_draw_indexed_indirect_commands.size(),
                  sizeof(VkDrawIndexedIndirectCommand));
            }
          } else {
            for (const auto& render_command : current_render_instances->deferred_render_instances.render_commands) {
              RenderInstancePushConstant push_constant;
              push_constant.camera_index = camera_index;
              push_constant.instance_index = render_command.instance_index;
              deferred_prepass_pipeline->states.polygon_mode =
                  wire_frame ? VK_POLYGON_MODE_LINE : render_command.polygon_mode;
              deferred_prepass_pipeline->states.cull_mode = render_command.cull_mode;
              deferred_prepass_pipeline->states.line_width = render_command.line_width;
              const auto prim_count =
                  render_command.Render(vk_command_buffer, push_constant, deferred_prepass_pipeline);
              if (count_draw_calls) {
                platform.draw_call[current_frame_index]++;
                platform.prim_count[current_frame_index] += prim_count;
              }
            }
          }
        }
        {
          const auto& deferred_instanced_prepass_pipeline =
              Platform::GetGraphicsPipeline("STANDARD_INSTANCED_DEFERRED_PREPASS");
          deferred_instanced_prepass_pipeline->states.ResetAllStates(geometry_pass_color_attachment_infos.size());
          deferred_instanced_prepass_pipeline->states.SetViewportScissor(view_port);
          deferred_instanced_prepass_pipeline->Bind(vk_command_buffer);
          deferred_instanced_prepass_pipeline->BindDescriptorSet(
              vk_command_buffer, 0, per_frame_descriptor_sets_[current_frame_index]->GetVkDescriptorSet());
          for (const auto& render_command :
               current_render_instances->deferred_instanced_render_instances.render_commands) {
            RenderInstancePushConstant push_constant;
            push_constant.camera_index = camera_index;
            push_constant.instance_index = render_command.instance_index;
            deferred_instanced_prepass_pipeline->states.polygon_mode =
                wire_frame ? VK_POLYGON_MODE_LINE : render_command.polygon_mode;
            deferred_instanced_prepass_pipeline->states.cull_mode = render_command.cull_mode;
            deferred_instanced_prepass_pipeline->states.line_width = render_command.line_width;
            const auto prim_count =
                render_command.Render(vk_command_buffer, push_constant, deferred_instanced_prepass_pipeline);
            if (count_draw_calls) {
              platform.draw_call[current_frame_index]++;
              platform.prim_count[current_frame_index] += prim_count;
            }
          }
        }
        GeometryStorage::BindSkinnedVertices(vk_command_buffer);
        {
          const auto& deferred_skinned_prepass_pipeline =
              Platform::GetGraphicsPipeline("STANDARD_SKINNED_DEFERRED_PREPASS");
          deferred_skinned_prepass_pipeline->states.ResetAllStates(geometry_pass_color_attachment_infos.size());
          deferred_skinned_prepass_pipeline->states.SetViewportScissor(view_port);
          deferred_skinned_prepass_pipeline->Bind(vk_command_buffer);
          deferred_skinned_prepass_pipeline->BindDescriptorSet(
              vk_command_buffer, 0, per_frame_descriptor_sets_[current_frame_index]->GetVkDescriptorSet());

          for (const auto& render_command :
               current_render_instances->deferred_skinned_render_instances.render_commands) {
            RenderInstancePushConstant push_constant;
            push_constant.camera_index = camera_index;
            push_constant.instance_index = render_command.instance_index;
            deferred_skinned_prepass_pipeline->states.polygon_mode =
                wire_frame ? VK_POLYGON_MODE_LINE : render_command.polygon_mode;
            deferred_skinned_prepass_pipeline->states.cull_mode = render_command.cull_mode;
            deferred_skinned_prepass_pipeline->states.line_width = render_command.line_width;
            const auto prim_count =
                render_command.Render(vk_command_buffer, push_constant, deferred_skinned_prepass_pipeline);
            if (count_draw_calls) {
              platform.draw_call[current_frame_index]++;
              platform.prim_count[current_frame_index] += prim_count;
            }
          }
        }
#ifdef EVOENGINE_WINDOWS
        GeometryStorage::BindStrandPoints(vk_command_buffer);
        {
          const auto& deferred_strands_prepass_pipeline =
              Platform::GetGraphicsPipeline("STANDARD_STRANDS_DEFERRED_PREPASS");
          deferred_strands_prepass_pipeline->states.ResetAllStates(geometry_pass_color_attachment_infos.size());
          deferred_strands_prepass_pipeline->states.SetViewportScissor(view_port);
          deferred_strands_prepass_pipeline->Bind(vk_command_buffer);
          deferred_strands_prepass_pipeline->BindDescriptorSet(
              vk_command_buffer, 0, per_frame_descriptor_sets_[current_frame_index]->GetVkDescriptorSet());
          for (const auto& render_command :
               current_render_instances->deferred_strands_render_instances.render_commands) {
            RenderInstancePushConstant push_constant;
            push_constant.camera_index = camera_index;
            push_constant.instance_index = render_command.instance_index;
            deferred_strands_prepass_pipeline->states.polygon_mode =
                wire_frame ? VK_POLYGON_MODE_LINE : render_command.polygon_mode;
            deferred_strands_prepass_pipeline->states.cull_mode = render_command.cull_mode;
            deferred_strands_prepass_pipeline->states.line_width = render_command.line_width;
            const auto prim_count =
                render_command.Render(vk_command_buffer, push_constant, deferred_strands_prepass_pipeline);
            if (count_draw_calls) {
              platform.draw_call[current_frame_index]++;
              platform.prim_count[current_frame_index] += prim_count;
            }
          }
        }
#endif
      });

#pragma endregion
#pragma region Lighting pass
      GeometryStorage::BindVertices(vk_command_buffer);
      {
        camera->TransitGBufferImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
        camera->render_texture_->GetDepthImage()->TransitImageLayout(vk_command_buffer,
                                                                     VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
        std::vector<VkRenderingAttachmentInfo> color_attachment_infos;
        camera->GetRenderTexture()->AppendColorAttachmentInfos(color_attachment_infos, VK_ATTACHMENT_LOAD_OP_CLEAR,
                                                               VK_ATTACHMENT_STORE_OP_STORE);
        VkRenderingInfo render_info{};
        render_info.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
        render_info.renderArea = render_area;
        render_info.layerCount = 1;
        render_info.colorAttachmentCount = color_attachment_infos.size();
        render_info.pColorAttachments = color_attachment_infos.data();
        render_info.pDepthAttachment = VK_NULL_HANDLE;
        lighting_->directional_light_shadow_map_->TransitImageLayout(vk_command_buffer,
                                                                     VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
        const auto& deferred_lighting_pipeline =
            is_scene_camera ? Platform::GetGraphicsPipeline("STANDARD_DEFERRED_LIGHTING_SCENE_CAMERA")
                            : Platform::GetGraphicsPipeline("STANDARD_DEFERRED_LIGHTING");
        Platform::RecordRenderCommands(render_info, vk_command_buffer, [&]() {
          deferred_lighting_pipeline->states.ResetAllStates(color_attachment_infos.size());
          deferred_lighting_pipeline->states.depth_test = false;
          deferred_lighting_pipeline->states.SetViewportScissor(view_port);

          deferred_lighting_pipeline->Bind(vk_command_buffer);
          deferred_lighting_pipeline->BindDescriptorSet(
              vk_command_buffer, 0, per_frame_descriptor_sets_[current_frame_index]->GetVkDescriptorSet());
          deferred_lighting_pipeline->BindDescriptorSet(vk_command_buffer, 1,
                                                        camera->g_buffer_descriptor_set_->GetVkDescriptorSet());
          deferred_lighting_pipeline->BindDescriptorSet(vk_command_buffer, 2,
                                                        lighting_->lighting_descriptor_set->GetVkDescriptorSet());
          RenderInstancePushConstant push_constant;
          push_constant.camera_index = camera_index;
          push_constant.light_split_index = need_fade ? glm::max(128, 256 - editor_layer->selection_alpha_) : 256;
          push_constant.instance_index = need_fade ? 1 : 0;
          deferred_lighting_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
          const auto mesh = Resources::GetResource<Mesh>("PRIMITIVE_TEX_PASS_THROUGH");

          mesh->DrawIndexed(vk_command_buffer, deferred_lighting_pipeline->states, 1);
        });
      }
#pragma endregion
#pragma endregion

#pragma region Forward Rendering
      for (const auto& func : forward_rendering_external_functions) {
        const auto prim_count = func(vk_command_buffer, camera, {camera_index, view_port});
        if (count_draw_calls) {
          platform.draw_call[current_frame_index]++;
          platform.prim_count[current_frame_index] += prim_count;
        }
      }
#pragma endregion
    });

    // Post processing
    if (const auto post_processing_stack = camera->post_processing_stack.Get<PostProcessingStack>()) {
      post_processing_stack->Process(camera);
    }
    camera->rendered_ = true;
    camera->require_rendering_ = false;
  }
}

void RenderLayer::RenderToCameraRayTracing(const GlobalTransform& camera_global_transform,
                                           const std::shared_ptr<Camera>& camera) const {
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  const auto current_render_instances = render_instances_list_[current_frame_index];
  const int camera_index = current_render_instances->GetCameraIndex(camera->GetHandle());
  const auto scene = Application::GetActiveScene();
  if (camera->camera_render_mode == Camera::CameraRenderMode::RayTracing) {
    const auto& ray_tracing_pipeline = Platform::GetRayTracingPipeline("RAY_TRACING_CAMERA");
    Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
      camera->GetRenderTexture()->GetColorImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);
      Platform::EverythingBarrier(vk_command_buffer);
      ray_tracing_pipeline->Bind(vk_command_buffer);
      ray_tracing_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                              per_frame_descriptor_sets_[current_frame_index]->GetVkDescriptorSet());
      ray_tracing_pipeline->BindDescriptorSet(vk_command_buffer, 1,
                                              ray_tracing_descriptor_sets_[current_frame_index]->GetVkDescriptorSet());
      ray_tracing_pipeline->BindDescriptorSet(
          vk_command_buffer, 2, camera->GetRenderTexture()->storage_descriptor_set_->GetVkDescriptorSet());

      RayTracingPushConstant push_constant;
      push_constant.camera_index = camera_index;
      push_constant.frame_id = camera->frame_count_;
      ray_tracing_pipeline->PushConstant(vk_command_buffer, 0, push_constant);

      ray_tracing_pipeline->Trace(vk_command_buffer, camera->render_texture_->GetExtent().width,
                                  camera->render_texture_->GetExtent().height, 1);
      Platform::EverythingBarrier(vk_command_buffer);
    });
    camera->rendered_ = true;
    camera->require_rendering_ = false;
  }
}

uint32_t RenderLayer::DrawMesh(const std::shared_ptr<Mesh>& mesh, const std::shared_ptr<Material>& material,
                               const GlobalTransform& global_transform, const bool cast_shadow) const {
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  const auto current_render_instances = render_instances_list_[current_frame_index];
  return current_render_instances->RegisterMeshDrawCommand(mesh, material, global_transform, cast_shadow);
}

const std::shared_ptr<DescriptorSet>& RenderLayer::GetPerFrameDescriptorSet() {
  return Application::GetLayer<RenderLayer>()->per_frame_descriptor_sets_[Platform::GetCurrentFrameIndex()];
}

const std::shared_ptr<DescriptorSet>& RenderLayer::GetLightingDescriptorSet() {
  return Application::GetLayer<RenderLayer>()->lighting_->lighting_descriptor_set;
}
