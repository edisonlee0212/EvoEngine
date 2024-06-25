#include "RenderLayer.hpp"
#include "Application.hpp"
#include "EditorLayer.hpp"
#include "GeometryStorage.hpp"
#include "Graphics.hpp"
#include "GraphicsPipeline.hpp"
#include "Jobs.hpp"
#include "LODGroup.hpp"
#include "MeshRenderer.hpp"
#include "Particles.hpp"
#include "PostProcessingStack.hpp"
#include "ProjectManager.hpp"
#include "SkinnedMeshRenderer.hpp"
#include "StrandsRenderer.hpp"
#include "TextureStorage.hpp"
#include "Utilities.hpp"
using namespace evo_engine;

void RenderInstanceCollection::Dispatch(const std::function<void(const RenderInstance&)>& command_action) const {
  for (const auto& render_command : render_commands) {
    command_action(render_command);
  }
}

void SkinnedRenderInstanceCollection::Dispatch(
    const std::function<void(const SkinnedRenderInstance&)>& command_action) const {
  for (const auto& render_command : render_commands) {
    command_action(render_command);
  }
}

void StrandsRenderInstanceCollection::Dispatch(
    const std::function<void(const StrandsRenderInstance&)>& command_action) const {
  for (const auto& render_command : render_commands) {
    command_action(render_command);
  }
}

void InstancedRenderInstanceCollection::Dispatch(
    const std::function<void(const InstancedRenderInstance&)>& command_action) const {
  for (const auto& render_command : render_commands) {
    command_action(render_command);
  }
}

void RenderLayer::OnCreate() {
  CreateStandardDescriptorBuffers();
  CreatePerFrameDescriptorSets();

  std::vector<glm::vec4> kernels;
  for (uint32_t i = 0; i < Graphics::Constants::max_kernel_amount; i++) {
    kernels.emplace_back(glm::ballRand(1.0f), 1.0f);
  }
  for (uint32_t i = 0; i < Graphics::Constants::max_kernel_amount; i++) {
    kernels.emplace_back(glm::gaussRand(0.0f, 1.0f), glm::gaussRand(0.0f, 1.0f), glm::gaussRand(0.0f, 1.0f),
                         glm::gaussRand(0.0f, 1.0f));
  }
  for (int i = 0; i < Graphics::GetMaxFramesInFlight(); i++) {
    kernel_descriptor_buffers_[i]->UploadVector(kernels);
  }

  PrepareEnvironmentalBrdfLut();

  lighting_ = std::make_unique<Lighting>();
  lighting_->Initialize();
}

void RenderLayer::OnDestroy() {
  render_info_descriptor_buffers_.clear();
  environment_info_descriptor_buffers_.clear();
  camera_info_descriptor_buffers_.clear();
  material_info_descriptor_buffers_.clear();
  instance_info_descriptor_buffers_.clear();
}

void RenderLayer::ClearAllCameras() {
  if (const auto scene = GetScene(); !scene)
    return;
  std::vector<std::pair<GlobalTransform, std::shared_ptr<Camera>>> cameras;
  CollectCameras(cameras);

  Graphics::AppendCommands([&](const VkCommandBuffer command_buffer) {
    for (const auto& i : cameras) {
      if (const auto render_texture = i.second->GetRenderTexture())
        render_texture->Clear(command_buffer);
    }
  });
}

void RenderLayer::RenderAllCameras() {
  const auto scene = GetScene();
  if (!scene)
    return;
  std::vector<std::pair<GlobalTransform, std::shared_ptr<Camera>>> cameras;
  CollectCameras(cameras);
  Bound world_bound{};
  total_mesh_triangles_ = 0;

  glm::vec3 lod_center = glm::vec3(0.f);
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
        lod_set = true;
      }
    }
  }

  CalculateLodFactor(lod_center, lod_max_distance);

  if (CollectRenderInstances(world_bound)) {
    world_bound.min - glm::vec3(0.1f);
    world_bound.max + glm::vec3(0.1f);
    scene->SetBound(world_bound);
  } else {
    world_bound.min = world_bound.max = glm::vec3(0.0f);
    world_bound.min - glm::vec3(1.f);
    world_bound.max + glm::vec3(1.f);
    scene->SetBound(world_bound);
  }

  if (const auto editor_layer = Application::GetLayer<EditorLayer>()) {
    editor_layer->MouseEntitySelection();
  }

  CollectDirectionalLights(cameras);
  CollectPointLights();
  CollectSpotLights();

  const auto current_frame_index = Graphics::GetCurrentFrameIndex();
  auto& graphics = Graphics::GetInstance();
  graphics.triangles[current_frame_index] = 0;
  graphics.strands_segments[current_frame_index] = 0;
  graphics.draw_call[current_frame_index] = 0;

  ApplyAnimator();
  // The following data stays consistent during entire frame.
  {
    for (int split = 0; split < 4; split++) {
      float split_end = max_shadow_distance;
      if (split != 3)
        split_end = max_shadow_distance * shadow_cascade_split[split];
      render_info_block.split_distances[split] = split_end;
    }
    render_info_block.brdflut_texture_index = environmental_brdf_lut_->GetTextureStorageIndex();
    if(enable_debug_visualization) render_info_block.debug_visualization = 1;
    else render_info_block.debug_visualization = 0;
  }

  {
    switch (scene->environment.environment_type) {
      case EnvironmentType::EnvironmentalMap: {
        environment_info_block.background_color.w = 0.0f;
      } break;
      case EnvironmentType::Color: {
        environment_info_block.background_color = glm::vec4(scene->environment.background_color, 1.0f);
      } break;
    }
    environment_info_block.environmental_map_gamma = scene->environment.environment_gamma;
    environment_info_block.environmental_lighting_intensity = scene->environment.ambient_light_intensity;
    environment_info_block.background_intensity = scene->environment.background_intensity;
  }

  render_info_descriptor_buffers_[current_frame_index]->Upload(render_info_block);
  environment_info_descriptor_buffers_[current_frame_index]->Upload(environment_info_block);
  directional_light_info_descriptor_buffers_[current_frame_index]->UploadVector(directional_light_info_blocks_);
  point_light_info_descriptor_buffers_[current_frame_index]->UploadVector(point_light_info_blocks_);
  spot_light_info_descriptor_buffers_[current_frame_index]->UploadVector(spot_light_info_blocks_);

  camera_info_descriptor_buffers_[current_frame_index]->UploadVector(camera_info_blocks_);
  material_info_descriptor_buffers_[current_frame_index]->UploadVector(material_info_blocks_);
  instance_info_descriptor_buffers_[current_frame_index]->UploadVector(instance_info_blocks_);

  mesh_draw_indexed_indirect_commands_buffers_[current_frame_index]->UploadVector(mesh_draw_indexed_indirect_commands_);
  mesh_draw_mesh_tasks_indirect_commands_buffers_[current_frame_index]->UploadVector(
      mesh_draw_mesh_tasks_indirect_commands_);

  VkDescriptorBufferInfo buffer_info;
  buffer_info.offset = 0;
  buffer_info.range = VK_WHOLE_SIZE;

  buffer_info.buffer = render_info_descriptor_buffers_[current_frame_index]->GetVkBuffer();
  per_frame_descriptor_sets_[current_frame_index]->UpdateBufferDescriptorBinding(0, buffer_info);
  buffer_info.buffer = environment_info_descriptor_buffers_[current_frame_index]->GetVkBuffer();
  per_frame_descriptor_sets_[current_frame_index]->UpdateBufferDescriptorBinding(1, buffer_info);
  buffer_info.buffer = camera_info_descriptor_buffers_[current_frame_index]->GetVkBuffer();
  per_frame_descriptor_sets_[current_frame_index]->UpdateBufferDescriptorBinding(2, buffer_info);
  buffer_info.buffer = material_info_descriptor_buffers_[current_frame_index]->GetVkBuffer();
  per_frame_descriptor_sets_[current_frame_index]->UpdateBufferDescriptorBinding(3, buffer_info);
  buffer_info.buffer = instance_info_descriptor_buffers_[current_frame_index]->GetVkBuffer();
  per_frame_descriptor_sets_[current_frame_index]->UpdateBufferDescriptorBinding(4, buffer_info);
  buffer_info.buffer = kernel_descriptor_buffers_[current_frame_index]->GetVkBuffer();
  per_frame_descriptor_sets_[current_frame_index]->UpdateBufferDescriptorBinding(5, buffer_info);
  buffer_info.buffer = directional_light_info_descriptor_buffers_[current_frame_index]->GetVkBuffer();
  per_frame_descriptor_sets_[current_frame_index]->UpdateBufferDescriptorBinding(6, buffer_info);
  buffer_info.buffer = point_light_info_descriptor_buffers_[current_frame_index]->GetVkBuffer();
  per_frame_descriptor_sets_[current_frame_index]->UpdateBufferDescriptorBinding(7, buffer_info);
  buffer_info.buffer = spot_light_info_descriptor_buffers_[current_frame_index]->GetVkBuffer();
  per_frame_descriptor_sets_[current_frame_index]->UpdateBufferDescriptorBinding(8, buffer_info);

  buffer_info.buffer = GeometryStorage::GetVertexBuffer()->GetVkBuffer();
  per_frame_descriptor_sets_[current_frame_index]->UpdateBufferDescriptorBinding(9, buffer_info);
  buffer_info.buffer = GeometryStorage::GetMeshletBuffer()->GetVkBuffer();
  per_frame_descriptor_sets_[current_frame_index]->UpdateBufferDescriptorBinding(10, buffer_info);

  PreparePointAndSpotLightShadowMap();

  for (const auto& [cameraGlobalTransform, camera] : cameras) {
    camera->rendered_ = false;
    if (camera->require_rendering_) {
      RenderToCamera(cameraGlobalTransform, camera);
    }
  }

  deferred_render_instances_.render_commands.clear();
  deferred_skinned_render_instances_.render_commands.clear();
  deferred_instanced_render_instances_.render_commands.clear();
  deferred_strands_render_instances_.render_commands.clear();
  transparent_render_instances_.render_commands.clear();
  transparent_skinned_render_instances_.render_commands.clear();
  transparent_instanced_render_instances_.render_commands.clear();
  transparent_strands_render_instances_.render_commands.clear();

  if (const auto editor_layer = Application::GetLayer<EditorLayer>()) {
    // Gizmos rendering
    for (const auto& i : editor_layer->gizmo_mesh_tasks_) {
      if (editor_layer->editor_cameras_.find(i.editor_camera_component->GetHandle()) ==
          editor_layer->editor_cameras_.end()) {
        EVOENGINE_ERROR("Target camera not registered in editor!");
        return;
      }
      if (i.editor_camera_component && i.editor_camera_component->IsEnabled()) {
        Graphics::AppendCommands([&](const VkCommandBuffer command_buffer) {
          std::shared_ptr<GraphicsPipeline> gizmos_pipeline;
          switch (i.gizmo_settings.color_mode) {
            case GizmoSettings::ColorMode::Default: {
              gizmos_pipeline = Graphics::GetGraphicsPipeline("GIZMOS");
            } break;
            case GizmoSettings::ColorMode::VertexColor: {
              gizmos_pipeline = Graphics::GetGraphicsPipeline("GIZMOS_VERTEX_COLORED");
            } break;
            case GizmoSettings::ColorMode::NormalColor: {
              gizmos_pipeline = Graphics::GetGraphicsPipeline("GIZMOS_NORMAL_COLORED");
            } break;
          }
          i.editor_camera_component->GetRenderTexture()->ApplyGraphicsPipelineStates(gizmos_pipeline->states);
          i.gizmo_settings.ApplySettings(gizmos_pipeline->states);

          gizmos_pipeline->Bind(command_buffer);
          gizmos_pipeline->BindDescriptorSet(command_buffer, 0, GetPerFrameDescriptorSet()->GetVkDescriptorSet());

          i.editor_camera_component->GetRenderTexture()->BeginRendering(command_buffer, VK_ATTACHMENT_LOAD_OP_LOAD,
                                                                        VK_ATTACHMENT_STORE_OP_STORE);
          GizmosPushConstant push_constant;
          push_constant.model = i.model;
          push_constant.color = i.color;
          push_constant.size = i.size;
          push_constant.camera_index = GetCameraIndex(i.editor_camera_component->GetHandle());
          gizmos_pipeline->PushConstant(command_buffer, 0, push_constant);
          GeometryStorage::BindVertices(command_buffer);
          i.mesh->DrawIndexed(command_buffer, gizmos_pipeline->states, 1);
          i.editor_camera_component->GetRenderTexture()->EndRendering(command_buffer);
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
        Graphics::AppendCommands([&](const VkCommandBuffer command_buffer) {
          const auto gizmos_pipeline = Graphics::GetGraphicsPipeline("GIZMOS_INSTANCED_COLORED");
          i.editor_camera_component->GetRenderTexture()->ApplyGraphicsPipelineStates(gizmos_pipeline->states);
          i.gizmo_settings.ApplySettings(gizmos_pipeline->states);

          gizmos_pipeline->Bind(command_buffer);
          gizmos_pipeline->BindDescriptorSet(command_buffer, 0, GetPerFrameDescriptorSet()->GetVkDescriptorSet());
          gizmos_pipeline->BindDescriptorSet(command_buffer, 1,
                                             i.instanced_data->GetDescriptorSet()->GetVkDescriptorSet());

          i.editor_camera_component->GetRenderTexture()->BeginRendering(command_buffer, VK_ATTACHMENT_LOAD_OP_LOAD,
                                                                        VK_ATTACHMENT_STORE_OP_STORE);
          GizmosPushConstant push_constant;
          push_constant.model = i.model;
          push_constant.color = glm::vec4(0.0f);
          push_constant.size = i.size;
          push_constant.camera_index = GetCameraIndex(i.editor_camera_component->GetHandle());
          gizmos_pipeline->PushConstant(command_buffer, 0, push_constant);
          GeometryStorage::BindVertices(command_buffer);
          i.mesh->DrawIndexed(command_buffer, gizmos_pipeline->states, i.instanced_data->PeekParticleInfoList().size());
          i.editor_camera_component->GetRenderTexture()->EndRendering(command_buffer);
        });
      }
    }
    for (const auto& i : editor_layer->gizmo_strands_tasks_) {
      if (editor_layer->editor_cameras_.find(i.editor_camera_component->GetHandle()) ==
          editor_layer->editor_cameras_.end()) {
        EVOENGINE_ERROR("Target camera not registered in editor!");
        return;
      }
      if (i.editor_camera_component && i.editor_camera_component->IsEnabled()) {
        Graphics::AppendCommands([&](const VkCommandBuffer command_buffer) {
          std::shared_ptr<GraphicsPipeline> gizmos_pipeline;
          switch (i.gizmo_settings.color_mode) {
            case GizmoSettings::ColorMode::Default: {
              gizmos_pipeline = Graphics::GetGraphicsPipeline("GIZMOS_STRANDS");
            } break;
            case GizmoSettings::ColorMode::VertexColor: {
              gizmos_pipeline = Graphics::GetGraphicsPipeline("GIZMOS_STRANDS_VERTEX_COLORED");
            } break;
            case GizmoSettings::ColorMode::NormalColor: {
              gizmos_pipeline = Graphics::GetGraphicsPipeline("GIZMOS_STRANDS_NORMAL_COLORED");
            } break;
          }
          i.editor_camera_component->GetRenderTexture()->ApplyGraphicsPipelineStates(gizmos_pipeline->states);
          i.gizmo_settings.ApplySettings(gizmos_pipeline->states);

          gizmos_pipeline->Bind(command_buffer);
          gizmos_pipeline->BindDescriptorSet(command_buffer, 0, GetPerFrameDescriptorSet()->GetVkDescriptorSet());

          i.editor_camera_component->GetRenderTexture()->BeginRendering(command_buffer, VK_ATTACHMENT_LOAD_OP_LOAD,
                                                                        VK_ATTACHMENT_STORE_OP_STORE);
          GizmosPushConstant push_constant;
          push_constant.model = i.model;
          push_constant.color = i.color;
          push_constant.size = i.m_size;
          push_constant.camera_index = GetCameraIndex(i.editor_camera_component->GetHandle());
          gizmos_pipeline->PushConstant(command_buffer, 0, push_constant);
          GeometryStorage::BindStrandPoints(command_buffer);
          i.m_strands->DrawIndexed(command_buffer, gizmos_pipeline->states, 1);
          i.editor_camera_component->GetRenderTexture()->EndRendering(command_buffer);
        });
      }
    }
  }

  material_indices_.clear();
  instance_indices_.clear();
  instance_handles_.clear();

  material_info_blocks_.clear();
  instance_info_blocks_.clear();

  directional_light_info_blocks_.clear();
  point_light_info_blocks_.clear();
  spot_light_info_blocks_.clear();

  mesh_draw_indexed_indirect_commands_.clear();
  mesh_draw_mesh_tasks_indirect_commands_.clear();

  cameras.clear();
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
    ImGui::Checkbox("Enable MeshRenderer", &enable_mesh_renderer);
    ImGui::Checkbox("Enable SkinnedMeshRenderer", &enable_skinned_mesh_renderer);
    ImGui::Checkbox("Enable Particles", &enable_particles);
    ImGui::Checkbox("Enable StrandsRenderer", &enable_strands_renderer);

    ImGui::Checkbox("Count dc for shadows", &count_shadow_rendering_draw_calls);
    ImGui::Checkbox("Wireframe", &wire_frame);
    if (Graphics::Constants::enable_mesh_shader)
      ImGui::Checkbox("Meshlet", &Graphics::Settings::use_mesh_shader);
    if (!Graphics::Settings::use_mesh_shader)
      ImGui::Checkbox("Indirect Rendering", &enable_indirect_rendering);
    if (Graphics::Constants::enable_mesh_shader && Graphics::Settings::use_mesh_shader) {
      ImGui::Checkbox("Show meshlets", &enable_debug_visualization);
    } else {
      ImGui::Checkbox("Show meshes", &enable_debug_visualization);
    }
    ImGui::DragFloat("Gamma", &render_info_block.gamma, 0.01f, 1.0f, 3.0f);
    if (ImGui::CollapsingHeader("Shadow", ImGuiTreeNodeFlags_DefaultOpen)) {
      if (ImGui::TreeNode("Distance")) {
        ImGui::DragFloat("Max shadow distance", &max_shadow_distance, 1.0f, 0.1f);
        ImGui::DragFloat("Split 1", &shadow_cascade_split[0], 0.01f, 0.0f, shadow_cascade_split[1]);
        ImGui::DragFloat("Split 2", &shadow_cascade_split[1], 0.01f, shadow_cascade_split[0], shadow_cascade_split[2]);
        ImGui::DragFloat("Split 3", &shadow_cascade_split[2], 0.01f, shadow_cascade_split[1], shadow_cascade_split[3]);
        ImGui::DragFloat("Split 4", &shadow_cascade_split[3], 0.01f, shadow_cascade_split[2], 1.0f);
        ImGui::TreePop();
      }
      if (ImGui::TreeNode("PCSS")) {
        ImGui::DragInt("Blocker search side amount", &render_info_block.blocker_search_amount, 1, 1, 8);
        ImGui::DragInt("PCF Sample Size", &render_info_block.pcf_sample_amount, 1, 1, 64);
        ImGui::TreePop();
      }
      ImGui::DragFloat("Seam fix ratio", &render_info_block.seam_fix_ratio, 0.001f, 0.0f, 0.1f);
      ImGui::Checkbox("Stable fit", &stable_fit);
    }

    if (ImGui::TreeNodeEx("Strands settings", ImGuiTreeNodeFlags_DefaultOpen)) {
      ImGui::DragFloat("Curve subdivision factor", &render_info_block.strands_subdivision_x_factor, 1.0f, 1.0f,
                       1000.0f);
      ImGui::DragFloat("Ring subdivision factor", &render_info_block.strands_subdivision_y_factor, 1.0f, 1.0f, 1000.0f);
      ImGui::DragInt("Max curve subdivision", &render_info_block.strands_subdivision_max_x, 1, 1, 15);
      ImGui::DragInt("Max ring subdivision", &render_info_block.strands_subdivision_max_y, 1, 1, 15);

      ImGui::TreePop();
    }
    ImGui::End();
  }
}

uint32_t RenderLayer::GetCameraIndex(const Handle& handle) {
  const auto search = camera_indices_.find(handle);
  if (search == camera_indices_.end()) {
    throw std::runtime_error("Unable to find camera!");
  }
  return search->second;
}

uint32_t RenderLayer::GetMaterialIndex(const Handle& handle) {
  const auto search = material_indices_.find(handle);
  if (search == material_indices_.end()) {
    throw std::runtime_error("Unable to find material!");
  }
  return search->second;
}

uint32_t RenderLayer::GetInstanceIndex(const Handle& handle) {
  const auto search = instance_indices_.find(handle);
  if (search == instance_indices_.end()) {
    throw std::runtime_error("Unable to find instance!");
  }
  return search->second;
}

Handle RenderLayer::GetInstanceHandle(uint32_t index) {
  const auto search = instance_handles_.find(index);
  if (search == instance_handles_.end()) {
    return 0;
  }
  return search->second;
}

uint32_t RenderLayer::RegisterCameraIndex(const Handle& handle, const CameraInfoBlock& camera_info_block) {
  const auto search = camera_indices_.find(handle);
  if (search == camera_indices_.end()) {
    const uint32_t index = camera_info_blocks_.size();
    camera_indices_[handle] = index;
    camera_info_blocks_.emplace_back(camera_info_block);
    return index;
  }
  return search->second;
}

uint32_t RenderLayer::RegisterMaterialIndex(const Handle& handle, const MaterialInfoBlock& material_info_block) {
  const auto search = material_indices_.find(handle);
  if (search == material_indices_.end()) {
    const uint32_t index = material_info_blocks_.size();
    material_indices_[handle] = index;
    material_info_blocks_.emplace_back(material_info_block);
    return index;
  }
  return search->second;
}

uint32_t RenderLayer::RegisterInstanceIndex(const Handle& handle, const InstanceInfoBlock& instance_info_block) {
  const auto search = instance_indices_.find(handle);
  if (search == instance_indices_.end()) {
    const uint32_t index = instance_info_blocks_.size();
    instance_indices_[handle] = index;
    instance_handles_[index] = handle;
    instance_info_blocks_.emplace_back(instance_info_block);
    return index;
  }
  return search->second;
}

void RenderLayer::CollectCameras(std::vector<std::pair<GlobalTransform, std::shared_ptr<Camera>>>& cameras) {
  auto scene = GetScene();
  camera_indices_.clear();
  camera_info_blocks_.clear();

  std::vector<std::pair<std::shared_ptr<Camera>, glm::vec3>> camera_pairs;

  if (auto editor_layer = Application::GetLayer<EditorLayer>()) {
    for (const auto& [cameraHandle, editorCamera] : editor_layer->editor_cameras_) {
      if (editorCamera.camera || editorCamera.camera->IsEnabled()) {
        camera_pairs.emplace_back(editorCamera.camera, editorCamera.position);
        CameraInfoBlock camera_info_block;
        GlobalTransform scene_camera_gt;
        scene_camera_gt.SetValue(editorCamera.position, editorCamera.rotation, glm::vec3(1.0f));
        editorCamera.camera->UpdateCameraInfoBlock(camera_info_block, scene_camera_gt);
        const auto index = RegisterCameraIndex(cameraHandle, camera_info_block);

        cameras.emplace_back(scene_camera_gt, editorCamera.camera);
      }
    }
  }
  if (const std::vector<Entity>* camera_entities = scene->UnsafeGetPrivateComponentOwnersList<Camera>()) {
    for (const auto& i : *camera_entities) {
      if (!scene->IsEntityEnabled(i))
        continue;
      assert(scene->HasPrivateComponent<Camera>(i));
      auto camera = scene->GetOrSetPrivateComponent<Camera>(i).lock();
      if (!camera || !camera->IsEnabled())
        continue;
      auto camera_global_transform = scene->GetDataComponent<GlobalTransform>(i);
      camera_pairs.emplace_back(camera, camera_global_transform.GetPosition());
      CameraInfoBlock camera_info_block;
      camera->UpdateCameraInfoBlock(camera_info_block, camera_global_transform);
      const auto index = RegisterCameraIndex(camera->GetHandle(), camera_info_block);

      cameras.emplace_back(camera_global_transform, camera);
    }
  }
}

void RenderLayer::CollectDirectionalLights(
    const std::vector<std::pair<GlobalTransform, std::shared_ptr<Camera>>>& cameras) {
  const auto scene = GetScene();
  auto scene_bound = scene->GetBound();
  auto& min_bound = scene_bound.min;
  auto& max_bound = scene_bound.max;

  const std::vector<Entity>* directional_light_entities =
      scene->UnsafeGetPrivateComponentOwnersList<DirectionalLight>();
  render_info_block.directional_light_size = 0;
  if (directional_light_entities && !directional_light_entities->empty()) {
    directional_light_info_blocks_.resize(Graphics::Settings::max_directional_light_size * cameras.size());
    for (const auto& light_entity : *directional_light_entities) {
      if (!scene->IsEntityEnabled(light_entity))
        continue;
      const auto dlc = scene->GetOrSetPrivateComponent<DirectionalLight>(light_entity).lock();
      if (!dlc->IsEnabled())
        continue;
      render_info_block.directional_light_size++;
    }
    std::vector<glm::uvec3> viewport_results;
    Lighting::AllocateAtlas(render_info_block.directional_light_size,
                            Graphics::Settings::directional_light_shadow_map_resolution, viewport_results);
    for (const auto& [cameraGlobalTransform, camera] : cameras) {
      auto camera_index = GetCameraIndex(camera->GetHandle());
      for (int i = 0; i < render_info_block.directional_light_size; i++) {
        const auto block_index = camera_index * Graphics::Settings::max_directional_light_size + i;
        auto& viewport = directional_light_info_blocks_[block_index].viewport;
        viewport.x = viewport_results[i].x;
        viewport.y = viewport_results[i].y;
        viewport.z = viewport_results[i].z;
        viewport.w = viewport_results[i].z;
      }
    }

    for (const auto& [cameraGlobalTransform, camera] : cameras) {
      size_t directional_light_index = 0;
      auto camera_index = GetCameraIndex(camera->GetHandle());
      glm::vec3 main_camera_pos = cameraGlobalTransform.GetPosition();
      glm::quat main_camera_rot = cameraGlobalTransform.GetRotation();
      for (const auto& light_entity : *directional_light_entities) {
        if (!scene->IsEntityEnabled(light_entity))
          continue;
        const auto dlc = scene->GetOrSetPrivateComponent<DirectionalLight>(light_entity).lock();
        if (!dlc->IsEnabled())
          continue;
        glm::quat rotation = scene->GetDataComponent<GlobalTransform>(light_entity).GetRotation();
        glm::vec3 light_dir = glm::normalize(rotation * glm::vec3(0, 0, 1));
        float plane_distance = 0;
        glm::vec3 center;
        const auto block_index =
            camera_index * Graphics::Settings::max_directional_light_size + directional_light_index;
        directional_light_info_blocks_[block_index].direction = glm::vec4(light_dir, 0.0f);
        directional_light_info_blocks_[block_index].diffuse =
            glm::vec4(dlc->diffuse * dlc->diffuse_brightness, dlc->cast_shadow);
        directional_light_info_blocks_[block_index].m_specular = glm::vec4(0.0f);
        for (int split = 0; split < 4; split++) {
          float split_start = 0;
          float split_end = max_shadow_distance;
          if (split != 0)
            split_start = max_shadow_distance * shadow_cascade_split[split - 1];
          if (split != 4 - 1)
            split_end = max_shadow_distance * shadow_cascade_split[split];
          render_info_block.split_distances[split] = split_end;
          glm::mat4 light_projection, light_view;
          float max = 0;
          glm::vec3 light_pos;
          glm::vec3 corner_points[8];
          Camera::CalculateFrustumPoints(camera, split_start, split_end, main_camera_pos, main_camera_rot,
                                         corner_points);
          glm::vec3 camera_frustum_center =
              (main_camera_rot * glm::vec3(0, 0, -1)) * ((split_end - split_start) / 2.0f + split_start) +
              main_camera_pos;
          if (stable_fit) {
            // Less detail but no shimmering when rotating the camera.
            // max = glm::distance(cornerPoints[4], cameraFrustumCenter);
            max = split_end;
          } else {
            // More detail but cause shimmering when rotating camera.
            max = (glm::max)(
                max, glm::distance(corner_points[0], Ray::ClosestPointOnLine(corner_points[0], camera_frustum_center,
                                                                             camera_frustum_center - light_dir)));
            max = (glm::max)(
                max, glm::distance(corner_points[1], Ray::ClosestPointOnLine(corner_points[1], camera_frustum_center,
                                                                             camera_frustum_center - light_dir)));
            max = (glm::max)(
                max, glm::distance(corner_points[2], Ray::ClosestPointOnLine(corner_points[2], camera_frustum_center,
                                                                             camera_frustum_center - light_dir)));
            max = (glm::max)(
                max, glm::distance(corner_points[3], Ray::ClosestPointOnLine(corner_points[3], camera_frustum_center,
                                                                             camera_frustum_center - light_dir)));
            max = (glm::max)(
                max, glm::distance(corner_points[4], Ray::ClosestPointOnLine(corner_points[4], camera_frustum_center,
                                                                             camera_frustum_center - light_dir)));
            max = (glm::max)(
                max, glm::distance(corner_points[5], Ray::ClosestPointOnLine(corner_points[5], camera_frustum_center,
                                                                             camera_frustum_center - light_dir)));
            max = (glm::max)(
                max, glm::distance(corner_points[6], Ray::ClosestPointOnLine(corner_points[6], camera_frustum_center,
                                                                             camera_frustum_center - light_dir)));
            max = (glm::max)(
                max, glm::distance(corner_points[7], Ray::ClosestPointOnLine(corner_points[7], camera_frustum_center,
                                                                             camera_frustum_center - light_dir)));
          }

          glm::vec3 p0 = Ray::ClosestPointOnLine(glm::vec3(max_bound.x, max_bound.y, max_bound.z),
                                                 camera_frustum_center, camera_frustum_center + light_dir);
          glm::vec3 p7 = Ray::ClosestPointOnLine(glm::vec3(min_bound.x, min_bound.y, min_bound.z),
                                                 camera_frustum_center, camera_frustum_center + light_dir);

          float d0 = glm::distance(p0, p7);

          glm::vec3 p1 = Ray::ClosestPointOnLine(glm::vec3(max_bound.x, max_bound.y, min_bound.z),
                                                 camera_frustum_center, camera_frustum_center + light_dir);
          glm::vec3 p6 = Ray::ClosestPointOnLine(glm::vec3(min_bound.x, min_bound.y, max_bound.z),
                                                 camera_frustum_center, camera_frustum_center + light_dir);

          float d1 = glm::distance(p1, p6);

          glm::vec3 p2 = Ray::ClosestPointOnLine(glm::vec3(max_bound.x, min_bound.y, max_bound.z),
                                                 camera_frustum_center, camera_frustum_center + light_dir);
          glm::vec3 p5 = Ray::ClosestPointOnLine(glm::vec3(min_bound.x, max_bound.y, min_bound.z),
                                                 camera_frustum_center, camera_frustum_center + light_dir);

          float d2 = glm::distance(p2, p5);

          glm::vec3 p3 = Ray::ClosestPointOnLine(glm::vec3(max_bound.x, min_bound.y, min_bound.z),
                                                 camera_frustum_center, camera_frustum_center + light_dir);
          glm::vec3 p4 = Ray::ClosestPointOnLine(glm::vec3(min_bound.x, max_bound.y, max_bound.z),
                                                 camera_frustum_center, camera_frustum_center + light_dir);

          float d3 = glm::distance(p3, p4);

          center =
              Ray::ClosestPointOnLine(scene_bound.Center(), camera_frustum_center, camera_frustum_center + light_dir);
          plane_distance = (glm::max)((glm::max)(d0, d1), (glm::max)(d2, d3));
          light_pos = center - light_dir * plane_distance;
          light_view = glm::lookAt(light_pos, light_pos + light_dir, glm::normalize(rotation * glm::vec3(0, 1, 0)));
          light_projection = glm::ortho(-max, max, -max, max, 0.0f, plane_distance * 2.0f);
#pragma region Fix Shimmering due to the movement of the camera
          glm::mat4 shadow_matrix = light_projection * light_view;
          glm::vec4 shadow_origin = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
          shadow_origin = shadow_matrix * shadow_origin;
          shadow_origin =
              shadow_origin * static_cast<float>(directional_light_info_blocks_[block_index].viewport.z) / 2.0f;
          glm::vec4 rounded_origin = glm::round(shadow_origin);
          glm::vec4 round_offset = rounded_origin - shadow_origin;
          round_offset =
              round_offset * 2.0f / static_cast<float>(directional_light_info_blocks_[block_index].viewport.z);
          round_offset.z = 0.0f;
          round_offset.w = 0.0f;
          glm::mat4 shadow_proj = light_projection;
          shadow_proj[3] += round_offset;
          light_projection = shadow_proj;
#pragma endregion
          directional_light_info_blocks_[block_index].light_space_matrix[split] = light_projection * light_view;
          directional_light_info_blocks_[block_index].light_frustum_width[split] = max;
          directional_light_info_blocks_[block_index].light_frustum_distance[split] = plane_distance;
          if (split == 4 - 1)
            directional_light_info_blocks_[block_index].reserved_parameters =
                glm::vec4(dlc->light_size, 0, dlc->bias, dlc->normal_offset);
        }
        directional_light_index++;
      }
    }
  }
}

void RenderLayer::CollectPointLights() {
  const auto scene = GetScene();
  const auto main_camera = scene->main_camera.Get<Camera>();
  glm::vec3 main_camera_position = {0, 0, 0};
  if (main_camera) {
    if (const auto main_camera_owner = main_camera->GetOwner(); scene->IsEntityValid(main_camera_owner)) {
      main_camera_position = scene->GetDataComponent<GlobalTransform>(main_camera_owner).GetPosition();
    }
  }
  const std::vector<Entity>* point_light_entities = scene->UnsafeGetPrivateComponentOwnersList<PointLight>();
  render_info_block.point_light_size = 0;
  if (point_light_entities && !point_light_entities->empty()) {
    point_light_info_blocks_.resize(point_light_entities->size());
    std::multimap<float, size_t> sorted_point_light_indices;
    for (int i = 0; i < point_light_entities->size(); i++) {
      Entity light_entity = point_light_entities->at(i);
      if (!scene->IsEntityEnabled(light_entity))
        continue;
      const auto plc = scene->GetOrSetPrivateComponent<PointLight>(light_entity).lock();
      if (!plc->IsEnabled())
        continue;
      glm::vec3 position = scene->GetDataComponent<GlobalTransform>(light_entity).value[3];
      point_light_info_blocks_[render_info_block.point_light_size].position = glm::vec4(position, 0);
      point_light_info_blocks_[render_info_block.point_light_size].constant_linear_quad_far_plane.x = plc->constant;
      point_light_info_blocks_[render_info_block.point_light_size].constant_linear_quad_far_plane.y = plc->linear;
      point_light_info_blocks_[render_info_block.point_light_size].constant_linear_quad_far_plane.z = plc->quadratic;
      point_light_info_blocks_[render_info_block.point_light_size].diffuse =
          glm::vec4(plc->diffuse * plc->diffuse_brightness, plc->cast_shadow);
      point_light_info_blocks_[render_info_block.point_light_size].specular = glm::vec4(0);
      point_light_info_blocks_[render_info_block.point_light_size].constant_linear_quad_far_plane.w =
          plc->GetFarPlane();

      glm::mat4 shadow_proj = glm::perspective(
          glm::radians(90.0f), 1.0f, 1.0f,
          point_light_info_blocks_[render_info_block.point_light_size].constant_linear_quad_far_plane.w);
      point_light_info_blocks_[render_info_block.point_light_size].light_space_matrix[0] =
          shadow_proj * glm::lookAt(position, position + glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(0.0f, -1.0f, 0.0f));
      point_light_info_blocks_[render_info_block.point_light_size].light_space_matrix[1] =
          shadow_proj * glm::lookAt(position, position + glm::vec3(-1.0f, 0.0f, 0.0f), glm::vec3(0.0f, -1.0f, 0.0f));
      point_light_info_blocks_[render_info_block.point_light_size].light_space_matrix[2] =
          shadow_proj * glm::lookAt(position, position + glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f));
      point_light_info_blocks_[render_info_block.point_light_size].light_space_matrix[3] =
          shadow_proj * glm::lookAt(position, position + glm::vec3(0.0f, -1.0f, 0.0f), glm::vec3(0.0f, 0.0f, -1.0f));
      point_light_info_blocks_[render_info_block.point_light_size].light_space_matrix[4] =
          shadow_proj * glm::lookAt(position, position + glm::vec3(0.0f, 0.0f, 1.0f), glm::vec3(0.0f, -1.0f, 0.0f));
      point_light_info_blocks_[render_info_block.point_light_size].light_space_matrix[5] =
          shadow_proj * glm::lookAt(position, position + glm::vec3(0.0f, 0.0f, -1.0f), glm::vec3(0.0f, -1.0f, 0.0f));
      point_light_info_blocks_[render_info_block.point_light_size].reserved_parameters =
          glm::vec4(plc->bias, plc->light_size, 0, 0);

      sorted_point_light_indices.insert(
          {glm::distance(main_camera_position, position), render_info_block.point_light_size});
      render_info_block.point_light_size++;
    }
    std::vector<glm::uvec3> view_port_results;
    Lighting::AllocateAtlas(render_info_block.point_light_size, Graphics::Settings::point_light_shadow_map_resolution,
                            view_port_results);
    int allocation_index = 0;
    for (const auto& point_light_index : sorted_point_light_indices) {
      auto& viewport = point_light_info_blocks_[point_light_index.second].viewport;
      viewport.x = view_port_results[allocation_index].x;
      viewport.y = view_port_results[allocation_index].y;
      viewport.z = view_port_results[allocation_index].z;
      viewport.w = view_port_results[allocation_index].z;

      allocation_index++;
    }
  }
  point_light_info_blocks_.resize(render_info_block.point_light_size);
}

void RenderLayer::CollectSpotLights() {
  const auto scene = GetScene();
  const auto main_camera = scene->main_camera.Get<Camera>();
  glm::vec3 main_camera_position = {0, 0, 0};
  if (main_camera) {
    auto main_camera_owner = main_camera->GetOwner();
    if (scene->IsEntityValid(main_camera_owner)) {
      main_camera_position = scene->GetDataComponent<GlobalTransform>(main_camera_owner).GetPosition();
    }
  }

  render_info_block.spot_light_size = 0;
  const std::vector<Entity>* spot_light_entities = scene->UnsafeGetPrivateComponentOwnersList<SpotLight>();
  if (spot_light_entities && !spot_light_entities->empty()) {
    spot_light_info_blocks_.resize(spot_light_entities->size());
    std::multimap<float, size_t> sorted_spot_light_indices;
    for (int i = 0; i < spot_light_entities->size(); i++) {
      Entity light_entity = spot_light_entities->at(i);
      if (!scene->IsEntityEnabled(light_entity))
        continue;
      const auto slc = scene->GetOrSetPrivateComponent<SpotLight>(light_entity).lock();
      if (!slc->IsEnabled())
        continue;
      auto ltw = scene->GetDataComponent<GlobalTransform>(light_entity);
      glm::vec3 position = ltw.value[3];
      glm::vec3 front = ltw.GetRotation() * glm::vec3(0, 0, -1);
      glm::vec3 up = ltw.GetRotation() * glm::vec3(0, 1, 0);
      spot_light_info_blocks_[render_info_block.spot_light_size].position = glm::vec4(position, 0);
      spot_light_info_blocks_[render_info_block.spot_light_size].direction = glm::vec4(front, 0);
      spot_light_info_blocks_[render_info_block.spot_light_size].constant_linear_quad_far_plane.x = slc->constant;
      spot_light_info_blocks_[render_info_block.spot_light_size].constant_linear_quad_far_plane.y = slc->linear;
      spot_light_info_blocks_[render_info_block.spot_light_size].constant_linear_quad_far_plane.z = slc->quadratic;
      spot_light_info_blocks_[render_info_block.spot_light_size].constant_linear_quad_far_plane.w = slc->GetFarPlane();
      spot_light_info_blocks_[render_info_block.spot_light_size].diffuse =
          glm::vec4(slc->diffuse * slc->diffuse_brightness, slc->cast_shadow);
      spot_light_info_blocks_[render_info_block.spot_light_size].specular = glm::vec4(0);

      glm::mat4 shadow_proj =
          glm::perspective(glm::radians(slc->outer_degrees * 2.0f), 1.0f, 1.0f,
                           spot_light_info_blocks_[render_info_block.spot_light_size].constant_linear_quad_far_plane.w);
      spot_light_info_blocks_[render_info_block.spot_light_size].light_space_matrix =
          shadow_proj * glm::lookAt(position, position + front, up);
      spot_light_info_blocks_[render_info_block.spot_light_size].cut_off_outer_cut_off_light_size_bias =
          glm::vec4(glm::cos(glm::radians(slc->inner_degrees)), glm::cos(glm::radians(slc->outer_degrees)),
                    slc->light_size, slc->bias);

      sorted_spot_light_indices.insert(
          {glm::distance(main_camera_position, position), render_info_block.spot_light_size});
      render_info_block.spot_light_size++;
    }
    std::vector<glm::uvec3> view_port_results;
    Lighting::AllocateAtlas(render_info_block.spot_light_size, Graphics::Settings::spot_light_shadow_map_resolution,
                            view_port_results);
    int allocation_index = 0;
    for (const auto& spot_light_index : sorted_spot_light_indices) {
      auto& view_port = spot_light_info_blocks_[spot_light_index.second].viewport;
      view_port.x = view_port_results[allocation_index].x;
      view_port.y = view_port_results[allocation_index].y;
      view_port.z = view_port_results[allocation_index].z;
      view_port.w = view_port_results[allocation_index].z;
      allocation_index++;
    }
  }
  spot_light_info_blocks_.resize(render_info_block.spot_light_size);
}

void RenderLayer::ApplyAnimator() const {
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
  const bool use_mesh_shader = Graphics::Constants::enable_mesh_shader && Graphics::Settings::use_mesh_shader;
  const auto current_frame_index = Graphics::GetCurrentFrameIndex();
  const auto& point_light_shadow_pipeline = use_mesh_shader
                                                ? Graphics::GetGraphicsPipeline("POINT_LIGHT_SHADOW_MAP_MESH")
                                                : Graphics::GetGraphicsPipeline("POINT_LIGHT_SHADOW_MAP");
  const auto& spot_light_shadow_pipeline = use_mesh_shader ? Graphics::GetGraphicsPipeline("SPOT_LIGHT_SHADOW_MAP_MESH")
                                                           : Graphics::GetGraphicsPipeline("SPOT_LIGHT_SHADOW_MAP");

  const auto& point_light_shadow_skinned_pipeline = Graphics::GetGraphicsPipeline("POINT_LIGHT_SHADOW_MAP_SKINNED");
  const auto& spot_light_shadow_skinned_pipeline = Graphics::GetGraphicsPipeline("SPOT_LIGHT_SHADOW_MAP_SKINNED");

  const auto& point_light_shadow_instanced_pipeline = Graphics::GetGraphicsPipeline("POINT_LIGHT_SHADOW_MAP_INSTANCED");
  const auto& spot_light_shadow_instanced_pipeline = Graphics::GetGraphicsPipeline("SPOT_LIGHT_SHADOW_MAP_INSTANCED");

  const auto& point_light_shadow_strands_pipeline = Graphics::GetGraphicsPipeline("POINT_LIGHT_SHADOW_MAP_STRANDS");
  const auto& spot_light_shadow_strands_pipeline = Graphics::GetGraphicsPipeline("SPOT_LIGHT_SHADOW_MAP_STRANDS");
  auto& graphics = Graphics::GetInstance();

  const uint32_t task_work_group_invocations =
      graphics.mesh_shader_properties_ext_.maxPreferredTaskWorkGroupInvocations;

  Graphics::AppendCommands([&](VkCommandBuffer command_buffer) {
#pragma region Viewport and scissor
    VkRect2D render_area;
    render_area.offset = {0, 0};
    render_area.extent.width = lighting_->point_light_shadow_map_->GetExtent().width;
    render_area.extent.height = lighting_->point_light_shadow_map_->GetExtent().height;

    VkViewport viewport;
    viewport.x = 0;
    viewport.y = 0;
    viewport.width = render_area.extent.width;
    viewport.height = render_area.extent.height;
    viewport.minDepth = 0.0f;
    viewport.maxDepth = 1.0f;

    VkRect2D scissor;
    scissor.offset = {0, 0};
    scissor.extent.width = render_area.extent.width;
    scissor.extent.height = render_area.extent.height;

#pragma endregion
    lighting_->point_light_shadow_map_->TransitImageLayout(command_buffer, VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL);

    for (int face = 0; face < 6; face++) {
      GeometryStorage::BindVertices(command_buffer);
      {
        VkRenderingInfo render_info{};
        auto depth_attachment = lighting_->GetLayeredPointLightDepthAttachmentInfo(face, VK_ATTACHMENT_LOAD_OP_CLEAR,
                                                                                   VK_ATTACHMENT_STORE_OP_STORE);
        render_info.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
        render_info.renderArea = render_area;
        render_info.layerCount = 1;
        render_info.colorAttachmentCount = 0;
        render_info.pColorAttachments = nullptr;
        render_info.pDepthAttachment = &depth_attachment;
        point_light_shadow_pipeline->states.ResetAllStates(0);
        point_light_shadow_pipeline->states.view_port = viewport;
        point_light_shadow_pipeline->states.scissor = scissor;

        vkCmdBeginRendering(command_buffer, &render_info);
        point_light_shadow_pipeline->Bind(command_buffer);
        point_light_shadow_pipeline->BindDescriptorSet(
            command_buffer, 0, per_frame_descriptor_sets_[Graphics::GetCurrentFrameIndex()]->GetVkDescriptorSet());
        for (int i = 0; i < point_light_info_blocks_.size(); i++) {
          const auto& point_light_info_block = point_light_info_blocks_[i];
          viewport.x = point_light_info_block.viewport.x;
          viewport.y = point_light_info_block.viewport.y;
          viewport.width = point_light_info_block.viewport.z;
          viewport.height = point_light_info_block.viewport.w;
          point_light_shadow_pipeline->states.view_port = viewport;
          scissor.extent.width = viewport.width;
          scissor.extent.height = viewport.height;
          point_light_shadow_pipeline->states.scissor = scissor;

          if (enable_indirect_rendering && !use_mesh_shader) {
            RenderInstancePushConstant push_constant;
            push_constant.camera_index = i;
            push_constant.light_split_index = face;
            push_constant.instance_index = 0;
            point_light_shadow_pipeline->PushConstant(command_buffer, 0, push_constant);
            point_light_shadow_pipeline->states.ApplyAllStates(command_buffer);
            if (count_draw_calls)
              graphics.draw_call[current_frame_index]++;
            if (count_draw_calls)
              graphics.triangles[current_frame_index] += total_mesh_triangles_;
            vkCmdDrawIndexedIndirect(
                command_buffer, mesh_draw_indexed_indirect_commands_buffers_[current_frame_index]->GetVkBuffer(), 0,
                mesh_draw_indexed_indirect_commands_.size(), sizeof(VkDrawIndexedIndirectCommand));
          } else {
            deferred_render_instances_.Dispatch([&](const RenderInstance& render_command) {
              if (!render_command.cast_shadow)
                return;
              RenderInstancePushConstant push_constant;
              push_constant.camera_index = i;
              push_constant.light_split_index = face;
              push_constant.instance_index = render_command.instance_index;
              point_light_shadow_pipeline->PushConstant(command_buffer, 0, push_constant);
              if (use_mesh_shader) {
                if (count_draw_calls)
                  graphics.draw_call[current_frame_index]++;
                if (count_draw_calls)
                  graphics.triangles[current_frame_index] += render_command.m_mesh->triangles_.size();

                const uint32_t count =
                    (render_command.meshlet_size + task_work_group_invocations - 1) / task_work_group_invocations;
                vkCmdDrawMeshTasksEXT(command_buffer, count, 1, 1);
              } else {
                const auto mesh = render_command.m_mesh;
                if (count_draw_calls)
                  graphics.draw_call[current_frame_index]++;
                if (count_draw_calls)
                  graphics.triangles[current_frame_index] += render_command.m_mesh->triangles_.size();
                mesh->DrawIndexed(command_buffer, point_light_shadow_pipeline->states, 1);
              }
            });
          }
        }
        vkCmdEndRendering(command_buffer);
      }
      {
        VkRenderingInfo render_info{};
        auto depth_attachment = lighting_->GetLayeredPointLightDepthAttachmentInfo(face, VK_ATTACHMENT_LOAD_OP_LOAD,
                                                                                   VK_ATTACHMENT_STORE_OP_STORE);
        render_info.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
        render_info.renderArea = render_area;
        render_info.layerCount = 1;
        render_info.colorAttachmentCount = 0;
        render_info.pColorAttachments = nullptr;
        render_info.pDepthAttachment = &depth_attachment;
        point_light_shadow_instanced_pipeline->states.ResetAllStates(0);
        point_light_shadow_instanced_pipeline->states.view_port = viewport;
        point_light_shadow_instanced_pipeline->states.scissor = scissor;
        vkCmdBeginRendering(command_buffer, &render_info);
        point_light_shadow_instanced_pipeline->Bind(command_buffer);
        point_light_shadow_instanced_pipeline->BindDescriptorSet(
            command_buffer, 0, per_frame_descriptor_sets_[Graphics::GetCurrentFrameIndex()]->GetVkDescriptorSet());
        for (int i = 0; i < point_light_info_blocks_.size(); i++) {
          const auto& point_light_info_block = point_light_info_blocks_[i];
          viewport.x = point_light_info_block.viewport.x;
          viewport.y = point_light_info_block.viewport.y;
          viewport.width = point_light_info_block.viewport.z;
          viewport.height = point_light_info_block.viewport.w;
          scissor.extent.width = viewport.width;
          scissor.extent.height = viewport.height;
          point_light_shadow_instanced_pipeline->states.view_port = viewport;
          point_light_shadow_instanced_pipeline->states.scissor = scissor;
          deferred_instanced_render_instances_.Dispatch([&](const InstancedRenderInstance& render_command) {
            if (!render_command.cast_shadow)
              return;
            point_light_shadow_instanced_pipeline->BindDescriptorSet(
                command_buffer, 1, render_command.particle_infos->GetDescriptorSet()->GetVkDescriptorSet());
            RenderInstancePushConstant push_constant;
            push_constant.camera_index = i;
            push_constant.light_split_index = face;
            push_constant.instance_index = render_command.instance_index;
            point_light_shadow_instanced_pipeline->PushConstant(command_buffer, 0, push_constant);
            const auto mesh = render_command.mesh;
            if (count_draw_calls)
              graphics.draw_call[current_frame_index]++;
            if (count_draw_calls)
              graphics.triangles[current_frame_index] +=
                  render_command.mesh->triangles_.size() * render_command.particle_infos->PeekParticleInfoList().size();
            mesh->DrawIndexed(command_buffer, point_light_shadow_instanced_pipeline->states,
                              render_command.particle_infos->PeekParticleInfoList().size());
          });
        }
        vkCmdEndRendering(command_buffer);
      }
      GeometryStorage::BindSkinnedVertices(command_buffer);
      {
        VkRenderingInfo render_info{};
        auto depth_attachment = lighting_->GetLayeredPointLightDepthAttachmentInfo(face, VK_ATTACHMENT_LOAD_OP_LOAD,
                                                                                   VK_ATTACHMENT_STORE_OP_STORE);
        render_info.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
        render_info.renderArea = render_area;
        render_info.layerCount = 1;
        render_info.colorAttachmentCount = 0;
        render_info.pColorAttachments = nullptr;
        render_info.pDepthAttachment = &depth_attachment;
        point_light_shadow_skinned_pipeline->states.ResetAllStates(0);
        point_light_shadow_skinned_pipeline->states.view_port = viewport;
        point_light_shadow_skinned_pipeline->states.scissor = scissor;
        vkCmdBeginRendering(command_buffer, &render_info);
        point_light_shadow_skinned_pipeline->Bind(command_buffer);
        point_light_shadow_skinned_pipeline->BindDescriptorSet(
            command_buffer, 0, per_frame_descriptor_sets_[Graphics::GetCurrentFrameIndex()]->GetVkDescriptorSet());
        for (int i = 0; i < point_light_info_blocks_.size(); i++) {
          const auto& point_light_info_block = point_light_info_blocks_[i];
          viewport.x = point_light_info_block.viewport.x;
          viewport.y = point_light_info_block.viewport.y;
          viewport.width = point_light_info_block.viewport.z;
          viewport.height = point_light_info_block.viewport.w;
          scissor.extent.width = viewport.width;
          scissor.extent.height = viewport.height;
          point_light_shadow_skinned_pipeline->states.view_port = viewport;
          point_light_shadow_skinned_pipeline->states.scissor = scissor;
          deferred_skinned_render_instances_.Dispatch([&](const SkinnedRenderInstance& render_command) {
            if (!render_command.cast_shadow)
              return;
            point_light_shadow_skinned_pipeline->BindDescriptorSet(
                command_buffer, 1, render_command.bone_matrices->GetDescriptorSet()->GetVkDescriptorSet());
            RenderInstancePushConstant push_constant;
            push_constant.camera_index = i;
            push_constant.light_split_index = face;
            push_constant.instance_index = render_command.instance_index;
            point_light_shadow_skinned_pipeline->PushConstant(command_buffer, 0, push_constant);
            const auto skinned_mesh = render_command.skinned_mesh;
            if (count_draw_calls)
              graphics.draw_call[current_frame_index]++;
            if (count_draw_calls)
              graphics.triangles[current_frame_index] += render_command.skinned_mesh->skinned_triangles_.size();
            skinned_mesh->DrawIndexed(command_buffer, point_light_shadow_skinned_pipeline->states, 1);
          });
        }
        vkCmdEndRendering(command_buffer);
      }
      GeometryStorage::BindStrandPoints(command_buffer);
      {
        VkRenderingInfo render_info{};
        auto depth_attachment = lighting_->GetLayeredPointLightDepthAttachmentInfo(face, VK_ATTACHMENT_LOAD_OP_LOAD,
                                                                                   VK_ATTACHMENT_STORE_OP_STORE);
        render_info.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
        render_info.renderArea = render_area;
        render_info.layerCount = 1;
        render_info.colorAttachmentCount = 0;
        render_info.pColorAttachments = nullptr;
        render_info.pDepthAttachment = &depth_attachment;
        point_light_shadow_strands_pipeline->states.ResetAllStates(0);
        point_light_shadow_strands_pipeline->states.view_port = viewport;
        point_light_shadow_strands_pipeline->states.scissor = scissor;
        vkCmdBeginRendering(command_buffer, &render_info);
        point_light_shadow_strands_pipeline->Bind(command_buffer);
        point_light_shadow_strands_pipeline->BindDescriptorSet(
            command_buffer, 0, per_frame_descriptor_sets_[Graphics::GetCurrentFrameIndex()]->GetVkDescriptorSet());
        for (int i = 0; i < point_light_info_blocks_.size(); i++) {
          const auto& point_light_info_block = point_light_info_blocks_[i];
          viewport.x = point_light_info_block.viewport.x;
          viewport.y = point_light_info_block.viewport.y;
          viewport.width = point_light_info_block.viewport.z;
          viewport.height = point_light_info_block.viewport.w;
          scissor.extent.width = viewport.width;
          scissor.extent.height = viewport.height;
          point_light_shadow_strands_pipeline->states.view_port = viewport;
          point_light_shadow_strands_pipeline->states.scissor = scissor;
          deferred_strands_render_instances_.Dispatch([&](const StrandsRenderInstance& render_command) {
            if (!render_command.cast_shadow)
              return;
            RenderInstancePushConstant push_constant;
            push_constant.camera_index = i;
            push_constant.light_split_index = face;
            push_constant.instance_index = render_command.instance_index;
            point_light_shadow_strands_pipeline->PushConstant(command_buffer, 0, push_constant);
            const auto strands = render_command.m_strands;
            if (count_draw_calls)
              graphics.draw_call[current_frame_index]++;
            if (count_draw_calls)
              graphics.strands_segments[current_frame_index] += render_command.m_strands->segments_.size();
            strands->DrawIndexed(command_buffer, point_light_shadow_strands_pipeline->states, 1);
          });
        }
        vkCmdEndRendering(command_buffer);
      }
    }
#pragma region Viewport and scissor

    render_area.offset = {0, 0};
    render_area.extent.width = lighting_->spot_light_shadow_map_->GetExtent().width;
    render_area.extent.height = lighting_->spot_light_shadow_map_->GetExtent().height;

    viewport.x = 0;
    viewport.y = 0;
    viewport.width = render_area.extent.width;
    viewport.height = render_area.extent.height;
    viewport.minDepth = 0.0f;
    viewport.maxDepth = 1.0f;

    scissor.offset = {0, 0};
    scissor.extent.width = render_area.extent.width;
    scissor.extent.height = render_area.extent.height;

#pragma endregion
    lighting_->spot_light_shadow_map_->TransitImageLayout(command_buffer, VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL);
    GeometryStorage::BindVertices(command_buffer);
    {
      VkRenderingInfo render_info{};
      auto depth_attachment =
          lighting_->GetSpotLightDepthAttachmentInfo(VK_ATTACHMENT_LOAD_OP_CLEAR, VK_ATTACHMENT_STORE_OP_STORE);
      render_info.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
      render_info.renderArea = render_area;
      render_info.layerCount = 1;
      render_info.colorAttachmentCount = 0;
      render_info.pColorAttachments = nullptr;
      render_info.pDepthAttachment = &depth_attachment;
      spot_light_shadow_pipeline->states.ResetAllStates(0);
      spot_light_shadow_pipeline->states.view_port = viewport;
      spot_light_shadow_pipeline->states.scissor = scissor;
      vkCmdBeginRendering(command_buffer, &render_info);
      spot_light_shadow_pipeline->Bind(command_buffer);
      spot_light_shadow_pipeline->BindDescriptorSet(
          command_buffer, 0, per_frame_descriptor_sets_[Graphics::GetCurrentFrameIndex()]->GetVkDescriptorSet());
      for (int i = 0; i < spot_light_info_blocks_.size(); i++) {
        const auto& spot_light_info_block = spot_light_info_blocks_[i];
        viewport.x = spot_light_info_block.viewport.x;
        viewport.y = spot_light_info_block.viewport.y;
        viewport.width = spot_light_info_block.viewport.z;
        viewport.height = spot_light_info_block.viewport.w;
        spot_light_shadow_pipeline->states.view_port = viewport;
        scissor.extent.width = viewport.width;
        scissor.extent.height = viewport.height;
        spot_light_shadow_pipeline->states.scissor = scissor;
        if (enable_indirect_rendering && !use_mesh_shader) {
          RenderInstancePushConstant push_constant;
          push_constant.camera_index = i;
          push_constant.light_split_index = 0;
          push_constant.instance_index = 0;
          spot_light_shadow_pipeline->PushConstant(command_buffer, 0, push_constant);
          spot_light_shadow_pipeline->states.ApplyAllStates(command_buffer);
          if (count_draw_calls)
            graphics.draw_call[current_frame_index]++;
          if (count_draw_calls)
            graphics.triangles[current_frame_index] += total_mesh_triangles_;
          vkCmdDrawIndexedIndirect(command_buffer,
                                   mesh_draw_indexed_indirect_commands_buffers_[current_frame_index]->GetVkBuffer(), 0,
                                   mesh_draw_indexed_indirect_commands_.size(), sizeof(VkDrawIndexedIndirectCommand));
        } else {
          deferred_render_instances_.Dispatch([&](const RenderInstance& render_command) {
            if (!render_command.cast_shadow)
              return;
            RenderInstancePushConstant push_constant;
            push_constant.camera_index = i;
            push_constant.light_split_index = 0;
            push_constant.instance_index = render_command.instance_index;
            spot_light_shadow_pipeline->PushConstant(command_buffer, 0, push_constant);
            if (use_mesh_shader) {
              if (count_draw_calls)
                graphics.draw_call[current_frame_index]++;
              if (count_draw_calls)
                graphics.triangles[current_frame_index] += render_command.m_mesh->triangles_.size();
              const uint32_t count =
                  (render_command.meshlet_size + task_work_group_invocations - 1) / task_work_group_invocations;
              vkCmdDrawMeshTasksEXT(command_buffer, count, 1, 1);
            } else {
              const auto mesh = render_command.m_mesh;
              if (count_draw_calls)
                graphics.draw_call[current_frame_index]++;
              if (count_draw_calls)
                graphics.triangles[current_frame_index] += render_command.m_mesh->triangles_.size();
              mesh->DrawIndexed(command_buffer, spot_light_shadow_pipeline->states, 1);
            }
          });
        }
      }
      vkCmdEndRendering(command_buffer);
    }
    {
      VkRenderingInfo render_info{};
      auto depth_attachment =
          lighting_->GetSpotLightDepthAttachmentInfo(VK_ATTACHMENT_LOAD_OP_LOAD, VK_ATTACHMENT_STORE_OP_STORE);
      render_info.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
      render_info.renderArea = render_area;
      render_info.layerCount = 1;
      render_info.colorAttachmentCount = 0;
      render_info.pColorAttachments = nullptr;
      render_info.pDepthAttachment = &depth_attachment;
      spot_light_shadow_instanced_pipeline->states.ResetAllStates(0);
      spot_light_shadow_instanced_pipeline->states.view_port = viewport;
      spot_light_shadow_instanced_pipeline->states.scissor = scissor;
      vkCmdBeginRendering(command_buffer, &render_info);
      spot_light_shadow_instanced_pipeline->Bind(command_buffer);
      spot_light_shadow_instanced_pipeline->BindDescriptorSet(
          command_buffer, 0, per_frame_descriptor_sets_[Graphics::GetCurrentFrameIndex()]->GetVkDescriptorSet());
      for (int i = 0; i < spot_light_info_blocks_.size(); i++) {
        const auto& spot_light_info_block = spot_light_info_blocks_[i];
        viewport.x = spot_light_info_block.viewport.x;
        viewport.y = spot_light_info_block.viewport.y;
        viewport.width = spot_light_info_block.viewport.z;
        viewport.height = spot_light_info_block.viewport.w;
        spot_light_shadow_instanced_pipeline->states.view_port = viewport;

        deferred_instanced_render_instances_.Dispatch([&](const InstancedRenderInstance& render_command) {
          if (!render_command.cast_shadow)
            return;
          spot_light_shadow_instanced_pipeline->BindDescriptorSet(
              command_buffer, 1, render_command.particle_infos->GetDescriptorSet()->GetVkDescriptorSet());
          RenderInstancePushConstant push_constant;
          push_constant.camera_index = i;
          push_constant.light_split_index = 0;
          push_constant.instance_index = render_command.instance_index;
          spot_light_shadow_instanced_pipeline->PushConstant(command_buffer, 0, push_constant);
          const auto mesh = render_command.mesh;
          if (count_draw_calls)
            graphics.draw_call[current_frame_index]++;
          if (count_draw_calls)
            graphics.triangles[current_frame_index] +=
                render_command.mesh->triangles_.size() * render_command.particle_infos->PeekParticleInfoList().size();
          mesh->DrawIndexed(command_buffer, spot_light_shadow_instanced_pipeline->states,
                            render_command.particle_infos->PeekParticleInfoList().size());
        });
      }
      vkCmdEndRendering(command_buffer);
    }
    GeometryStorage::BindSkinnedVertices(command_buffer);
    {
      VkRenderingInfo render_info{};
      auto depth_attachment =
          lighting_->GetSpotLightDepthAttachmentInfo(VK_ATTACHMENT_LOAD_OP_LOAD, VK_ATTACHMENT_STORE_OP_STORE);
      render_info.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
      render_info.renderArea = render_area;
      render_info.layerCount = 1;
      render_info.colorAttachmentCount = 0;
      render_info.pColorAttachments = nullptr;
      render_info.pDepthAttachment = &depth_attachment;
      spot_light_shadow_skinned_pipeline->states.ResetAllStates(0);
      spot_light_shadow_skinned_pipeline->states.view_port = viewport;
      spot_light_shadow_skinned_pipeline->states.scissor = scissor;
      vkCmdBeginRendering(command_buffer, &render_info);
      spot_light_shadow_skinned_pipeline->Bind(command_buffer);
      spot_light_shadow_skinned_pipeline->BindDescriptorSet(
          command_buffer, 0, per_frame_descriptor_sets_[Graphics::GetCurrentFrameIndex()]->GetVkDescriptorSet());
      for (int i = 0; i < spot_light_info_blocks_.size(); i++) {
        const auto& spot_light_info_block = spot_light_info_blocks_[i];
        viewport.x = spot_light_info_block.viewport.x;
        viewport.y = spot_light_info_block.viewport.y;
        viewport.width = spot_light_info_block.viewport.z;
        viewport.height = spot_light_info_block.viewport.w;
        spot_light_shadow_skinned_pipeline->states.view_port = viewport;

        deferred_skinned_render_instances_.Dispatch([&](const SkinnedRenderInstance& render_command) {
          if (!render_command.cast_shadow)
            return;
          spot_light_shadow_skinned_pipeline->BindDescriptorSet(
              command_buffer, 1, render_command.bone_matrices->GetDescriptorSet()->GetVkDescriptorSet());
          RenderInstancePushConstant push_constant;
          push_constant.camera_index = i;
          push_constant.light_split_index = 0;
          push_constant.instance_index = render_command.instance_index;
          spot_light_shadow_skinned_pipeline->PushConstant(command_buffer, 0, push_constant);
          const auto skinned_mesh = render_command.skinned_mesh;
          if (count_draw_calls)
            graphics.draw_call[current_frame_index]++;
          if (count_draw_calls)
            graphics.triangles[current_frame_index] += render_command.skinned_mesh->skinned_triangles_.size();
          skinned_mesh->DrawIndexed(command_buffer, spot_light_shadow_skinned_pipeline->states, 1);
        });
      }
      vkCmdEndRendering(command_buffer);
    }
    GeometryStorage::BindStrandPoints(command_buffer);
    {
      VkRenderingInfo render_info{};
      auto depth_attachment =
          lighting_->GetSpotLightDepthAttachmentInfo(VK_ATTACHMENT_LOAD_OP_LOAD, VK_ATTACHMENT_STORE_OP_STORE);
      render_info.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
      render_info.renderArea = render_area;
      render_info.layerCount = 1;
      render_info.colorAttachmentCount = 0;
      render_info.pColorAttachments = nullptr;
      render_info.pDepthAttachment = &depth_attachment;
      spot_light_shadow_strands_pipeline->states.ResetAllStates(0);
      spot_light_shadow_strands_pipeline->states.view_port = viewport;
      spot_light_shadow_strands_pipeline->states.scissor = scissor;
      vkCmdBeginRendering(command_buffer, &render_info);
      spot_light_shadow_strands_pipeline->Bind(command_buffer);
      spot_light_shadow_strands_pipeline->BindDescriptorSet(
          command_buffer, 0, per_frame_descriptor_sets_[Graphics::GetCurrentFrameIndex()]->GetVkDescriptorSet());
      for (int i = 0; i < spot_light_info_blocks_.size(); i++) {
        const auto& spot_light_info_block = spot_light_info_blocks_[i];
        viewport.x = spot_light_info_block.viewport.x;
        viewport.y = spot_light_info_block.viewport.y;
        viewport.width = spot_light_info_block.viewport.z;
        viewport.height = spot_light_info_block.viewport.w;
        spot_light_shadow_strands_pipeline->states.view_port = viewport;

        deferred_strands_render_instances_.Dispatch([&](const StrandsRenderInstance& render_command) {
          if (!render_command.cast_shadow)
            return;
          RenderInstancePushConstant push_constant;
          push_constant.camera_index = i;
          push_constant.light_split_index = 0;
          push_constant.instance_index = render_command.instance_index;
          spot_light_shadow_strands_pipeline->PushConstant(command_buffer, 0, push_constant);
          const auto strands = render_command.m_strands;
          if (count_draw_calls)
            graphics.draw_call[current_frame_index]++;
          if (count_draw_calls)
            graphics.strands_segments[current_frame_index] += render_command.m_strands->segments_.size();
          strands->DrawIndexed(command_buffer, spot_light_shadow_strands_pipeline->states, 1);
        });
      }
      vkCmdEndRendering(command_buffer);
    }
    lighting_->point_light_shadow_map_->TransitImageLayout(command_buffer, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    lighting_->spot_light_shadow_map_->TransitImageLayout(command_buffer, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
  });
}

void RenderLayer::CalculateLodFactor(const glm::vec3& center, const float max_distance) const {
  const auto scene = GetScene();
  if (const auto* owners = scene->UnsafeGetPrivateComponentOwnersList<LodGroup>()) {
    for (auto owner : *owners) {
      if (const auto lod_group = scene->GetOrSetPrivateComponent<LodGroup>(owner).lock();
          !lod_group->override_lod_factor) {
        auto gt = scene->GetDataComponent<GlobalTransform>(owner);
        const auto distance = glm::distance(gt.GetPosition(), center);
        const auto distance_factor = glm::clamp(distance / max_distance, 0.f, 1.f);
        lod_group->lod_factor = glm::clamp(distance_factor * distance_factor, 0.f, 1.f);
      }
    }
  }
}

bool RenderLayer::CollectRenderInstances(Bound& world_bound) {
  need_fade_ = false;
  const auto scene = GetScene();
  const auto editor_layer = Application::GetLayer<EditorLayer>();
  const bool enable_selection_high_light = editor_layer && scene->IsEntityValid(editor_layer->selected_entity_);
  auto& min_bound = world_bound.min;
  auto& max_bound = world_bound.max;
  min_bound = glm::vec3(FLT_MAX);
  max_bound = glm::vec3(-FLT_MAX);

  bool has_render_instance = false;
  std::unordered_set<Handle> lod_group_renderers{};
  if (const auto* owners = scene->UnsafeGetPrivateComponentOwnersList<LodGroup>()) {
    for (auto owner : *owners) {
      const auto lod_group = scene->GetOrSetPrivateComponent<LodGroup>(owner).lock();
      for (auto it = lod_group->lods.begin(); it != lod_group->lods.end(); ++it) {
        auto& lod = *it;
        bool render_current_level = true;
        if (lod_group->lod_factor > it->lod_offset) {
          render_current_level = false;
        }
        if (render_current_level && it != lod_group->lods.begin() && lod_group->lod_factor < (it - 1)->lod_offset) {
          render_current_level = false;
        }
        for (auto& renderer : lod.renderers) {
          if (const auto mesh_renderer = renderer.Get<MeshRenderer>()) {
            lod_group_renderers.insert(mesh_renderer->GetHandle());
            if (render_current_level && scene->IsEntityEnabled(owner) &&
                scene->IsEntityEnabled(mesh_renderer->GetOwner()) && mesh_renderer->IsEnabled()) {
              if (TryRegisterRenderer(scene, owner, mesh_renderer, min_bound, max_bound, enable_selection_high_light)) {
                has_render_instance = true;
              }
            }
          } else if (const auto skinned_mesh_renderer = renderer.Get<SkinnedMeshRenderer>()) {
            lod_group_renderers.insert(skinned_mesh_renderer->GetHandle());
            if (render_current_level && scene->IsEntityEnabled(owner) &&
                scene->IsEntityEnabled(skinned_mesh_renderer->GetOwner()) && skinned_mesh_renderer->IsEnabled()) {
              if (TryRegisterRenderer(scene, owner, skinned_mesh_renderer, min_bound, max_bound,
                                      enable_selection_high_light)) {
                has_render_instance = true;
              }
            }
          } else if (const auto particles = renderer.Get<Particles>()) {
            lod_group_renderers.insert(particles->GetHandle());
            if (render_current_level && scene->IsEntityEnabled(owner) &&
                scene->IsEntityEnabled(particles->GetOwner()) && particles->IsEnabled()) {
              if (TryRegisterRenderer(scene, owner, particles, min_bound, max_bound, enable_selection_high_light)) {
                has_render_instance = true;
              }
            }
          } else if (const auto strands_renderer = renderer.Get<StrandsRenderer>()) {
            lod_group_renderers.insert(strands_renderer->GetHandle());
            if (render_current_level && scene->IsEntityEnabled(owner) &&
                scene->IsEntityEnabled(strands_renderer->GetOwner()) && strands_renderer->IsEnabled()) {
              if (TryRegisterRenderer(scene, owner, strands_renderer, min_bound, max_bound,
                                      enable_selection_high_light)) {
                has_render_instance = true;
              }
            }
          }
        }
      }
    }
  }

  if (enable_mesh_renderer) {
    if (const auto* owners = scene->UnsafeGetPrivateComponentOwnersList<MeshRenderer>()) {
      for (auto owner : *owners) {
        if (!scene->IsEntityEnabled(owner))
          continue;
        auto mesh_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(owner).lock();
        if (lod_group_renderers.find(mesh_renderer->GetHandle()) != lod_group_renderers.end())
          continue;
        if (TryRegisterRenderer(scene, owner, mesh_renderer, min_bound, max_bound, enable_selection_high_light)) {
          has_render_instance = true;
        }
      }
    }
  }
  if (enable_skinned_mesh_renderer) {
    if (const auto* owners = scene->UnsafeGetPrivateComponentOwnersList<SkinnedMeshRenderer>()) {
      for (auto owner : *owners) {
        if (!scene->IsEntityEnabled(owner))
          continue;
        auto skinned_mesh_renderer = scene->GetOrSetPrivateComponent<SkinnedMeshRenderer>(owner).lock();
        if (lod_group_renderers.find(skinned_mesh_renderer->GetHandle()) != lod_group_renderers.end())
          continue;
        if (TryRegisterRenderer(scene, owner, skinned_mesh_renderer, min_bound, max_bound,
                                enable_selection_high_light)) {
          has_render_instance = true;
        }
      }
    }
  }
  if (enable_particles) {
    if (const auto* owners = scene->UnsafeGetPrivateComponentOwnersList<Particles>()) {
      for (auto owner : *owners) {
        if (!scene->IsEntityEnabled(owner))
          continue;
        auto particles = scene->GetOrSetPrivateComponent<Particles>(owner).lock();
        if (lod_group_renderers.find(particles->GetHandle()) != lod_group_renderers.end())
          continue;
        if (TryRegisterRenderer(scene, owner, particles, min_bound, max_bound, enable_selection_high_light)) {
          has_render_instance = true;
        }
      }
    }
  }
  if (enable_strands_renderer) {
    if (const auto* owners = scene->UnsafeGetPrivateComponentOwnersList<StrandsRenderer>()) {
      for (auto owner : *owners) {
        if (!scene->IsEntityEnabled(owner))
          continue;
        auto strands_renderer = scene->GetOrSetPrivateComponent<StrandsRenderer>(owner).lock();
        if (lod_group_renderers.find(strands_renderer->GetHandle()) != lod_group_renderers.end())
          continue;
        if (TryRegisterRenderer(scene, owner, strands_renderer, min_bound, max_bound, enable_selection_high_light)) {
          has_render_instance = true;
        }
      }
    }
  }
  return has_render_instance;
}

void RenderLayer::CreateStandardDescriptorBuffers() {
#pragma region Standard Descrioptor Layout
  render_info_descriptor_buffers_.clear();
  environment_info_descriptor_buffers_.clear();
  camera_info_descriptor_buffers_.clear();
  material_info_descriptor_buffers_.clear();
  instance_info_descriptor_buffers_.clear();

  kernel_descriptor_buffers_.clear();
  directional_light_info_descriptor_buffers_.clear();
  point_light_info_descriptor_buffers_.clear();
  spot_light_info_descriptor_buffers_.clear();

  mesh_draw_mesh_tasks_indirect_commands_buffers_.clear();
  mesh_draw_indexed_indirect_commands_buffers_.clear();

  VkBufferCreateInfo buffer_create_info{};
  buffer_create_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;

  buffer_create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  VmaAllocationCreateInfo buffer_vma_allocation_create_info{};
  buffer_vma_allocation_create_info.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
  const auto max_frame_in_flight = Graphics::GetMaxFramesInFlight();
  for (size_t i = 0; i < max_frame_in_flight; i++) {
    buffer_create_info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;
    buffer_create_info.size = sizeof(RenderInfoBlock);
    render_info_descriptor_buffers_.emplace_back(
        std::make_unique<Buffer>(buffer_create_info, buffer_vma_allocation_create_info));
    buffer_create_info.size = sizeof(EnvironmentInfoBlock);
    environment_info_descriptor_buffers_.emplace_back(
        std::make_unique<Buffer>(buffer_create_info, buffer_vma_allocation_create_info));
    buffer_create_info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
    buffer_create_info.size = sizeof(CameraInfoBlock) * Graphics::Constants::initial_camera_size;
    camera_info_descriptor_buffers_.emplace_back(
        std::make_unique<Buffer>(buffer_create_info, buffer_vma_allocation_create_info));
    buffer_create_info.size = sizeof(MaterialInfoBlock) * Graphics::Constants::initial_material_size;
    material_info_descriptor_buffers_.emplace_back(
        std::make_unique<Buffer>(buffer_create_info, buffer_vma_allocation_create_info));
    buffer_create_info.size = sizeof(InstanceInfoBlock) * Graphics::Constants::initial_instance_size;
    instance_info_descriptor_buffers_.emplace_back(
        std::make_unique<Buffer>(buffer_create_info, buffer_vma_allocation_create_info));

    buffer_create_info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;
    buffer_create_info.size = sizeof(glm::vec4) * Graphics::Constants::max_kernel_amount * 2;
    kernel_descriptor_buffers_.emplace_back(
        std::make_unique<Buffer>(buffer_create_info, buffer_vma_allocation_create_info));
    buffer_create_info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
    buffer_create_info.size = sizeof(DirectionalLightInfo) * Graphics::Settings::max_directional_light_size *
                              Graphics::Constants::initial_camera_size;
    directional_light_info_descriptor_buffers_.emplace_back(
        std::make_unique<Buffer>(buffer_create_info, buffer_vma_allocation_create_info));
    buffer_create_info.size = sizeof(PointLightInfo) * Graphics::Settings::max_point_light_size;
    point_light_info_descriptor_buffers_.emplace_back(
        std::make_unique<Buffer>(buffer_create_info, buffer_vma_allocation_create_info));
    buffer_create_info.size = sizeof(SpotLightInfo) * Graphics::Settings::max_spot_light_size;
    spot_light_info_descriptor_buffers_.emplace_back(
        std::make_unique<Buffer>(buffer_create_info, buffer_vma_allocation_create_info));

    buffer_create_info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_INDIRECT_BUFFER_BIT;
    buffer_create_info.size = sizeof(VkDrawMeshTasksIndirectCommandEXT) * Graphics::Constants::initial_render_task_size;
    mesh_draw_mesh_tasks_indirect_commands_buffers_.emplace_back(
        std::make_unique<Buffer>(buffer_create_info, buffer_vma_allocation_create_info));
    buffer_create_info.size = sizeof(VkDrawIndexedIndirectCommand) * Graphics::Constants::initial_render_task_size;
    mesh_draw_indexed_indirect_commands_buffers_.emplace_back(
        std::make_unique<Buffer>(buffer_create_info, buffer_vma_allocation_create_info));
  }
#pragma endregion
}

void RenderLayer::CreatePerFrameDescriptorSets() {
  const auto max_frames_in_flight = Graphics::GetMaxFramesInFlight();

  per_frame_descriptor_sets_.clear();
  for (size_t i = 0; i < max_frames_in_flight; i++) {
    auto descriptor_set = std::make_shared<DescriptorSet>(Graphics::GetDescriptorSetLayout("PER_FRAME_LAYOUT"));
    per_frame_descriptor_sets_.emplace_back(descriptor_set);
  }
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
    sampler_info.maxAnisotropy = Graphics::GetVkPhysicalDeviceProperties().limits.maxSamplerAnisotropy;
    sampler_info.borderColor = VK_BORDER_COLOR_INT_OPAQUE_BLACK;
    sampler_info.unnormalizedCoordinates = VK_FALSE;
    sampler_info.compareEnable = VK_FALSE;
    sampler_info.compareOp = VK_COMPARE_OP_ALWAYS;
    sampler_info.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;

    environmental_brdf_lut_texture_storage.sampler = std::make_unique<Sampler>(sampler_info);
  }
  const auto environmental_brdf_pipeline = Graphics::GetGraphicsPipeline("ENVIRONMENTAL_MAP_BRDF");
  Graphics::ImmediateSubmit([&](const VkCommandBuffer command_buffer) {
    environmental_brdf_lut_texture_storage.image->TransitImageLayout(command_buffer,
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
      vkCmdBeginRendering(command_buffer, &render_info);
      environmental_brdf_pipeline->Bind(command_buffer);
      const auto mesh = Resources::GetResource<Mesh>("PRIMITIVE_TEX_PASS_THROUGH");
      GeometryStorage::BindVertices(command_buffer);
      mesh->DrawIndexed(command_buffer, environmental_brdf_pipeline->states, 1);
      vkCmdEndRendering(command_buffer);
#pragma endregion
    }
    environmental_brdf_lut_texture_storage.image->TransitImageLayout(command_buffer,
                                                                     VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
  });
}

void RenderLayer::RenderToCamera(const GlobalTransform& camera_global_transform,
                                 const std::shared_ptr<Camera>& camera) {
  const bool count_draw_calls = count_shadow_rendering_draw_calls;
  const bool use_mesh_shader = Graphics::Constants::enable_mesh_shader && Graphics::Settings::use_mesh_shader;
  const auto current_frame_index = Graphics::GetCurrentFrameIndex();
  const int camera_index = GetCameraIndex(camera->GetHandle());
  const auto scene = Application::GetActiveScene();
#pragma region Directional Light Shadows
  const auto& directional_light_shadow_pipeline =
      use_mesh_shader ? Graphics::GetGraphicsPipeline("DIRECTIONAL_LIGHT_SHADOW_MAP_MESH")
                      : Graphics::GetGraphicsPipeline("DIRECTIONAL_LIGHT_SHADOW_MAP");
  const auto& directional_light_shadow_pipeline_skinned =
      Graphics::GetGraphicsPipeline("DIRECTIONAL_LIGHT_SHADOW_MAP_SKINNED");
  const auto& directional_light_shadow_pipeline_instanced =
      Graphics::GetGraphicsPipeline("DIRECTIONAL_LIGHT_SHADOW_MAP_INSTANCED");
  const auto& directional_light_shadow_pipeline_strands =
      Graphics::GetGraphicsPipeline("DIRECTIONAL_LIGHT_SHADOW_MAP_STRANDS");
  auto& graphics = Graphics::GetInstance();
  const uint32_t task_work_group_invocations =
      graphics.mesh_shader_properties_ext_.maxPreferredTaskWorkGroupInvocations;
  Graphics::AppendCommands([&](VkCommandBuffer command_buffer) {
#pragma region Viewport and scissor
    VkRect2D render_area;
    render_area.offset = {0, 0};
    render_area.extent.width = lighting_->directional_light_shadow_map_->GetExtent().width;
    render_area.extent.height = lighting_->directional_light_shadow_map_->GetExtent().height;

    VkViewport viewport;
    viewport.x = 0;
    viewport.y = 0;
    viewport.width = render_area.extent.width;
    viewport.height = render_area.extent.height;
    viewport.minDepth = 0.0f;
    viewport.maxDepth = 1.0f;

    VkRect2D scissor;
    scissor.offset = {0, 0};
    scissor.extent.width = render_area.extent.width;
    scissor.extent.height = render_area.extent.height;

#pragma endregion
    lighting_->directional_light_shadow_map_->TransitImageLayout(command_buffer, VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL);

    for (int split = 0; split < 4; split++) {
      {
        const auto depth_attachment = lighting_->GetLayeredDirectionalLightDepthAttachmentInfo(
            split, VK_ATTACHMENT_LOAD_OP_CLEAR, VK_ATTACHMENT_STORE_OP_STORE);
        VkRenderingInfo render_info{};
        render_info.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
        render_info.renderArea = render_area;
        render_info.layerCount = 1;
        render_info.colorAttachmentCount = 0;
        render_info.pColorAttachments = nullptr;
        render_info.pDepthAttachment = &depth_attachment;
        directional_light_shadow_pipeline->states.ResetAllStates(0);
        directional_light_shadow_pipeline->states.scissor = scissor;
        directional_light_shadow_pipeline->states.view_port = viewport;
        directional_light_shadow_pipeline->states.color_blend_attachment_states.clear();

        vkCmdBeginRendering(command_buffer, &render_info);
        directional_light_shadow_pipeline->Bind(command_buffer);
        directional_light_shadow_pipeline->BindDescriptorSet(
            command_buffer, 0, per_frame_descriptor_sets_[current_frame_index]->GetVkDescriptorSet());
        GeometryStorage::BindVertices(command_buffer);
        for (int i = 0; i < render_info_block.directional_light_size; i++) {
          const auto& directional_light_info_block =
              directional_light_info_blocks_[camera_index * Graphics::Settings::max_directional_light_size + i];
          viewport.x = directional_light_info_block.viewport.x;
          viewport.y = directional_light_info_block.viewport.y;
          viewport.width = directional_light_info_block.viewport.z;
          viewport.height = directional_light_info_block.viewport.w;
          scissor.extent.width = directional_light_info_block.viewport.z;
          scissor.extent.height = directional_light_info_block.viewport.w;
          directional_light_shadow_pipeline->states.scissor = scissor;
          directional_light_shadow_pipeline->states.view_port = viewport;
          directional_light_shadow_pipeline->states.ApplyAllStates(command_buffer);
          if (enable_indirect_rendering && !use_mesh_shader) {
            RenderInstancePushConstant push_constant;
            push_constant.camera_index = camera_index * Graphics::Settings::max_directional_light_size + i;
            push_constant.light_split_index = split;
            push_constant.instance_index = 0;
            directional_light_shadow_pipeline->PushConstant(command_buffer, 0, push_constant);
            directional_light_shadow_pipeline->states.ApplyAllStates(command_buffer);
            if (count_draw_calls)
              graphics.draw_call[current_frame_index]++;
            if (count_draw_calls)
              graphics.triangles[current_frame_index] += total_mesh_triangles_;
            vkCmdDrawIndexedIndirect(
                command_buffer, mesh_draw_indexed_indirect_commands_buffers_[current_frame_index]->GetVkBuffer(), 0,
                mesh_draw_indexed_indirect_commands_.size(), sizeof(VkDrawIndexedIndirectCommand));
          } else {
            deferred_render_instances_.Dispatch([&](const RenderInstance& render_command) {
              if (!render_command.cast_shadow)
                return;
              RenderInstancePushConstant push_constant;
              push_constant.camera_index = camera_index * Graphics::Settings::max_directional_light_size + i;
              push_constant.light_split_index = split;
              push_constant.instance_index = render_command.instance_index;
              directional_light_shadow_pipeline->PushConstant(command_buffer, 0, push_constant);
              if (use_mesh_shader) {
                if (count_draw_calls)
                  graphics.draw_call[current_frame_index]++;
                if (count_draw_calls)
                  graphics.triangles[current_frame_index] += render_command.m_mesh->triangles_.size();

                const uint32_t count =
                    (render_command.meshlet_size + task_work_group_invocations - 1) / task_work_group_invocations;
                vkCmdDrawMeshTasksEXT(command_buffer, count, 1, 1);
              } else {
                const auto mesh = render_command.m_mesh;
                GeometryStorage::BindVertices(command_buffer);
                if (count_draw_calls)
                  graphics.draw_call[current_frame_index]++;
                if (count_draw_calls)
                  graphics.triangles[current_frame_index] += render_command.m_mesh->triangles_.size();
                mesh->DrawIndexed(command_buffer, directional_light_shadow_pipeline->states, 1);
              }
            });
          }
        }
        vkCmdEndRendering(command_buffer);
      }
      {
        const auto depth_attachment = lighting_->GetLayeredDirectionalLightDepthAttachmentInfo(
            split, VK_ATTACHMENT_LOAD_OP_LOAD, VK_ATTACHMENT_STORE_OP_STORE);
        VkRenderingInfo render_info{};
        render_info.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
        render_info.renderArea = render_area;
        render_info.layerCount = 1;
        render_info.colorAttachmentCount = 0;
        render_info.pColorAttachments = nullptr;
        render_info.pDepthAttachment = &depth_attachment;
        directional_light_shadow_pipeline_instanced->states.ResetAllStates(0);
        directional_light_shadow_pipeline_instanced->states.scissor = scissor;
        directional_light_shadow_pipeline_instanced->states.view_port = viewport;
        directional_light_shadow_pipeline_instanced->states.color_blend_attachment_states.clear();

        vkCmdBeginRendering(command_buffer, &render_info);
        directional_light_shadow_pipeline_instanced->Bind(command_buffer);
        directional_light_shadow_pipeline_instanced->BindDescriptorSet(
            command_buffer, 0, per_frame_descriptor_sets_[Graphics::GetCurrentFrameIndex()]->GetVkDescriptorSet());
        directional_light_shadow_pipeline_instanced->states.cull_mode = VK_CULL_MODE_NONE;
        GeometryStorage::BindVertices(command_buffer);
        for (int i = 0; i < render_info_block.directional_light_size; i++) {
          const auto& directional_light_info_block =
              directional_light_info_blocks_[camera_index * Graphics::Settings::max_directional_light_size + i];
          viewport.x = directional_light_info_block.viewport.x;
          viewport.y = directional_light_info_block.viewport.y;
          viewport.width = directional_light_info_block.viewport.z;
          viewport.height = directional_light_info_block.viewport.w;
          scissor.extent.width = directional_light_info_block.viewport.z;
          scissor.extent.height = directional_light_info_block.viewport.w;
          directional_light_shadow_pipeline_instanced->states.scissor = scissor;
          directional_light_shadow_pipeline_instanced->states.view_port = viewport;
          directional_light_shadow_pipeline_instanced->states.ApplyAllStates(command_buffer);

          deferred_instanced_render_instances_.Dispatch([&](const InstancedRenderInstance& render_command) {
            if (!render_command.cast_shadow)
              return;
            directional_light_shadow_pipeline_instanced->BindDescriptorSet(
                command_buffer, 1, render_command.particle_infos->GetDescriptorSet()->GetVkDescriptorSet());
            RenderInstancePushConstant push_constant;
            push_constant.camera_index = camera_index * Graphics::Settings::max_directional_light_size + i;
            push_constant.light_split_index = split;
            push_constant.instance_index = render_command.instance_index;
            directional_light_shadow_pipeline_instanced->PushConstant(command_buffer, 0, push_constant);
            const auto mesh = render_command.mesh;
            if (count_draw_calls)
              graphics.draw_call[current_frame_index]++;
            if (count_draw_calls)
              graphics.triangles[current_frame_index] +=
                  render_command.mesh->triangles_.size() * render_command.particle_infos->PeekParticleInfoList().size();
            mesh->DrawIndexed(command_buffer, directional_light_shadow_pipeline_instanced->states,
                              render_command.particle_infos->PeekParticleInfoList().size());
          });
        }
        vkCmdEndRendering(command_buffer);
      }
      GeometryStorage::BindSkinnedVertices(command_buffer);
      {
        const auto depth_attachment = lighting_->GetLayeredDirectionalLightDepthAttachmentInfo(
            split, VK_ATTACHMENT_LOAD_OP_LOAD, VK_ATTACHMENT_STORE_OP_STORE);
        VkRenderingInfo render_info{};
        render_info.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
        render_info.renderArea = render_area;
        render_info.layerCount = 1;
        render_info.colorAttachmentCount = 0;
        render_info.pColorAttachments = nullptr;
        render_info.pDepthAttachment = &depth_attachment;
        directional_light_shadow_pipeline_skinned->states.ResetAllStates(0);
        directional_light_shadow_pipeline_skinned->states.scissor = scissor;
        directional_light_shadow_pipeline_skinned->states.view_port = viewport;
        directional_light_shadow_pipeline_skinned->states.color_blend_attachment_states.clear();

        vkCmdBeginRendering(command_buffer, &render_info);
        directional_light_shadow_pipeline_skinned->Bind(command_buffer);
        directional_light_shadow_pipeline_skinned->BindDescriptorSet(
            command_buffer, 0, per_frame_descriptor_sets_[Graphics::GetCurrentFrameIndex()]->GetVkDescriptorSet());
        directional_light_shadow_pipeline_skinned->states.cull_mode = VK_CULL_MODE_NONE;
        for (int i = 0; i < render_info_block.directional_light_size; i++) {
          const auto& directional_light_info_block =
              directional_light_info_blocks_[camera_index * Graphics::Settings::max_directional_light_size + i];
          viewport.x = directional_light_info_block.viewport.x;
          viewport.y = directional_light_info_block.viewport.y;
          viewport.width = directional_light_info_block.viewport.z;
          viewport.height = directional_light_info_block.viewport.w;
          scissor.extent.width = directional_light_info_block.viewport.z;
          scissor.extent.height = directional_light_info_block.viewport.w;
          directional_light_shadow_pipeline_skinned->states.scissor = scissor;
          directional_light_shadow_pipeline_skinned->states.view_port = viewport;
          directional_light_shadow_pipeline_skinned->states.ApplyAllStates(command_buffer);

          deferred_skinned_render_instances_.Dispatch([&](const SkinnedRenderInstance& render_command) {
            if (!render_command.cast_shadow)
              return;
            directional_light_shadow_pipeline_skinned->BindDescriptorSet(
                command_buffer, 1, render_command.bone_matrices->GetDescriptorSet()->GetVkDescriptorSet());
            RenderInstancePushConstant push_constant;
            push_constant.camera_index = camera_index * Graphics::Settings::max_directional_light_size + i;
            push_constant.light_split_index = split;
            push_constant.instance_index = render_command.instance_index;
            directional_light_shadow_pipeline_skinned->PushConstant(command_buffer, 0, push_constant);
            const auto skinned_mesh = render_command.skinned_mesh;
            if (count_draw_calls)
              graphics.draw_call[current_frame_index]++;
            if (count_draw_calls)
              graphics.triangles[current_frame_index] += render_command.skinned_mesh->skinned_triangles_.size();
            skinned_mesh->DrawIndexed(command_buffer, directional_light_shadow_pipeline_skinned->states, 1);
          });
        }
        vkCmdEndRendering(command_buffer);
      }
      GeometryStorage::BindStrandPoints(command_buffer);
      {
        const auto depth_attachment = lighting_->GetLayeredDirectionalLightDepthAttachmentInfo(
            split, VK_ATTACHMENT_LOAD_OP_LOAD, VK_ATTACHMENT_STORE_OP_STORE);
        VkRenderingInfo render_info{};
        render_info.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
        render_info.renderArea = render_area;
        render_info.layerCount = 1;
        render_info.colorAttachmentCount = 0;
        render_info.pColorAttachments = nullptr;
        render_info.pDepthAttachment = &depth_attachment;
        directional_light_shadow_pipeline_strands->states.ResetAllStates(0);
        directional_light_shadow_pipeline_strands->states.scissor = scissor;
        directional_light_shadow_pipeline_strands->states.view_port = viewport;
        directional_light_shadow_pipeline_strands->states.color_blend_attachment_states.clear();

        vkCmdBeginRendering(command_buffer, &render_info);
        directional_light_shadow_pipeline_strands->Bind(command_buffer);
        directional_light_shadow_pipeline_strands->BindDescriptorSet(
            command_buffer, 0, per_frame_descriptor_sets_[Graphics::GetCurrentFrameIndex()]->GetVkDescriptorSet());
        directional_light_shadow_pipeline_strands->states.cull_mode = VK_CULL_MODE_NONE;
        for (int i = 0; i < render_info_block.directional_light_size; i++) {
          const auto& directional_light_info_block =
              directional_light_info_blocks_[camera_index * Graphics::Settings::max_directional_light_size + i];
          viewport.x = directional_light_info_block.viewport.x;
          viewport.y = directional_light_info_block.viewport.y;
          viewport.width = directional_light_info_block.viewport.z;
          viewport.height = directional_light_info_block.viewport.w;
          scissor.extent.width = directional_light_info_block.viewport.z;
          scissor.extent.height = directional_light_info_block.viewport.w;
          directional_light_shadow_pipeline_strands->states.scissor = scissor;
          directional_light_shadow_pipeline_strands->states.view_port = viewport;
          directional_light_shadow_pipeline_strands->states.ApplyAllStates(command_buffer);

          deferred_strands_render_instances_.Dispatch([&](const StrandsRenderInstance& render_command) {
            if (!render_command.cast_shadow)
              return;
            RenderInstancePushConstant push_constant;
            push_constant.camera_index = camera_index * Graphics::Settings::max_directional_light_size + i;
            push_constant.light_split_index = split;
            push_constant.instance_index = render_command.instance_index;
            directional_light_shadow_pipeline_strands->PushConstant(command_buffer, 0, push_constant);
            const auto strands = render_command.m_strands;
            if (count_draw_calls)
              graphics.draw_call[current_frame_index]++;
            if (count_draw_calls)
              graphics.strands_segments[current_frame_index] += render_command.m_strands->segments_.size();
            strands->DrawIndexed(command_buffer, directional_light_shadow_pipeline_strands->states, 1);
          });
        }
        vkCmdEndRendering(command_buffer);
      }
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

#pragma region Deferred Rendering
  Graphics::AppendCommands([&](VkCommandBuffer command_buffer) {
#pragma region Viewport and scissor
    VkRect2D render_area;
    render_area.offset = {0, 0};
    render_area.extent.width = camera->GetSize().x;
    render_area.extent.height = camera->GetSize().y;
    VkViewport viewport;
    viewport.x = 0.0f;
    viewport.y = 0.0f;
    viewport.width = camera->GetSize().x;
    viewport.height = camera->GetSize().y;
    viewport.minDepth = 0.0f;
    viewport.maxDepth = 1.0f;

    VkRect2D scissor;
    scissor.offset = {0, 0};
    scissor.extent.width = camera->GetSize().x;
    scissor.extent.height = camera->GetSize().y;

    camera->TransitGBufferImageLayout(command_buffer, VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL);
    camera->render_texture_->GetDepthImage()->TransitImageLayout(command_buffer, VK_IMAGE_LAYOUT_ATTACHMENT_OPTIMAL);

    VkRenderingInfo render_info{};
    render_info.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
    render_info.renderArea = render_area;
    render_info.layerCount = 1;
#pragma endregion
#pragma region Geometry pass
    GeometryStorage::BindVertices(command_buffer);
    {
      const auto depth_attachment =
          camera->render_texture_->GetDepthAttachmentInfo(VK_ATTACHMENT_LOAD_OP_CLEAR, VK_ATTACHMENT_STORE_OP_STORE);
      render_info.pDepthAttachment = &depth_attachment;
      std::vector<VkRenderingAttachmentInfo> color_attachment_infos;
      camera->AppendGBufferColorAttachmentInfos(color_attachment_infos, VK_ATTACHMENT_LOAD_OP_CLEAR,
                                                VK_ATTACHMENT_STORE_OP_STORE);
      render_info.colorAttachmentCount = color_attachment_infos.size();
      render_info.pColorAttachments = color_attachment_infos.data();

      const auto& deferred_prepass_pipeline = use_mesh_shader
                                                  ? Graphics::GetGraphicsPipeline("STANDARD_DEFERRED_PREPASS_MESH")
                                                  : Graphics::GetGraphicsPipeline("STANDARD_DEFERRED_PREPASS");
      deferred_prepass_pipeline->states.ResetAllStates(color_attachment_infos.size());
      deferred_prepass_pipeline->states.view_port = viewport;
      deferred_prepass_pipeline->states.scissor = scissor;
      deferred_prepass_pipeline->states.polygon_mode = wire_frame ? VK_POLYGON_MODE_LINE : VK_POLYGON_MODE_FILL;
      vkCmdBeginRendering(command_buffer, &render_info);
      deferred_prepass_pipeline->Bind(command_buffer);
      deferred_prepass_pipeline->BindDescriptorSet(
          command_buffer, 0, per_frame_descriptor_sets_[Graphics::GetCurrentFrameIndex()]->GetVkDescriptorSet());
      if (enable_indirect_rendering && !use_mesh_shader) {
        if (!deferred_render_instances_.render_commands.empty()) {
          RenderInstancePushConstant push_constant;
          push_constant.camera_index = camera_index;
          push_constant.instance_index = 0;
          deferred_prepass_pipeline->PushConstant(command_buffer, 0, push_constant);
          GeometryStorage::BindVertices(command_buffer);
          graphics.draw_call[current_frame_index]++;
          graphics.triangles[current_frame_index] += total_mesh_triangles_;
          vkCmdDrawIndexedIndirect(command_buffer,
                                   mesh_draw_indexed_indirect_commands_buffers_[current_frame_index]->GetVkBuffer(), 0,
                                   mesh_draw_indexed_indirect_commands_.size(), sizeof(VkDrawIndexedIndirectCommand));
        }
      } else {
        deferred_render_instances_.Dispatch([&](const RenderInstance& render_command) {
          RenderInstancePushConstant push_constant;
          push_constant.camera_index = camera_index;
          push_constant.instance_index = render_command.instance_index;
          deferred_prepass_pipeline->states.polygon_mode =
              wire_frame ? VK_POLYGON_MODE_LINE : render_command.polygon_mode;
          deferred_prepass_pipeline->states.cull_mode = render_command.cull_mode;
          deferred_prepass_pipeline->states.line_width = render_command.line_width;
          deferred_prepass_pipeline->states.ApplyAllStates(command_buffer);
          deferred_prepass_pipeline->PushConstant(command_buffer, 0, push_constant);
          if (use_mesh_shader) {
            graphics.draw_call[current_frame_index]++;
            graphics.triangles[current_frame_index] += render_command.m_mesh->triangles_.size();

            const uint32_t count =
                (render_command.meshlet_size + task_work_group_invocations - 1) / task_work_group_invocations;
            vkCmdDrawMeshTasksEXT(command_buffer, count, 1, 1);
          } else {
            const auto mesh = render_command.m_mesh;
            graphics.draw_call[current_frame_index]++;
            graphics.triangles[current_frame_index] += render_command.m_mesh->triangles_.size();
            mesh->DrawIndexed(command_buffer, deferred_prepass_pipeline->states, 1);
          }
        });
      }

      vkCmdEndRendering(command_buffer);
    }
    {
      const auto depth_attachment =
          camera->render_texture_->GetDepthAttachmentInfo(VK_ATTACHMENT_LOAD_OP_LOAD, VK_ATTACHMENT_STORE_OP_STORE);
      render_info.pDepthAttachment = &depth_attachment;
      std::vector<VkRenderingAttachmentInfo> color_attachment_infos;
      camera->AppendGBufferColorAttachmentInfos(color_attachment_infos, VK_ATTACHMENT_LOAD_OP_LOAD,
                                                VK_ATTACHMENT_STORE_OP_STORE);
      render_info.colorAttachmentCount = color_attachment_infos.size();
      render_info.pColorAttachments = color_attachment_infos.data();

      const auto& deferred_instanced_prepass_pipeline =
          Graphics::GetGraphicsPipeline("STANDARD_INSTANCED_DEFERRED_PREPASS");
      deferred_instanced_prepass_pipeline->states.ResetAllStates(color_attachment_infos.size());
      deferred_instanced_prepass_pipeline->states.view_port = viewport;
      deferred_instanced_prepass_pipeline->states.scissor = scissor;
      vkCmdBeginRendering(command_buffer, &render_info);
      deferred_instanced_prepass_pipeline->Bind(command_buffer);
      deferred_instanced_prepass_pipeline->BindDescriptorSet(
          command_buffer, 0, per_frame_descriptor_sets_[Graphics::GetCurrentFrameIndex()]->GetVkDescriptorSet());
      deferred_instanced_render_instances_.Dispatch([&](const InstancedRenderInstance& render_command) {
        deferred_instanced_prepass_pipeline->BindDescriptorSet(
            command_buffer, 1, render_command.particle_infos->GetDescriptorSet()->GetVkDescriptorSet());
        RenderInstancePushConstant push_constant;
        push_constant.camera_index = camera_index;
        push_constant.instance_index = render_command.instance_index;
        deferred_instanced_prepass_pipeline->states.polygon_mode =
            wire_frame ? VK_POLYGON_MODE_LINE : render_command.polygon_mode;
        deferred_instanced_prepass_pipeline->states.cull_mode = render_command.cull_mode;
        deferred_instanced_prepass_pipeline->states.line_width = render_command.line_width;
        deferred_instanced_prepass_pipeline->states.ApplyAllStates(command_buffer);
        deferred_instanced_prepass_pipeline->PushConstant(command_buffer, 0, push_constant);
        const auto mesh = render_command.mesh;
        graphics.draw_call[current_frame_index]++;
        graphics.triangles[current_frame_index] +=
            render_command.mesh->triangles_.size() * render_command.particle_infos->PeekParticleInfoList().size();
        mesh->DrawIndexed(command_buffer, deferred_instanced_prepass_pipeline->states,
                          render_command.particle_infos->PeekParticleInfoList().size());
      });

      vkCmdEndRendering(command_buffer);
    }
    GeometryStorage::BindSkinnedVertices(command_buffer);
    {
      const auto depth_attachment =
          camera->render_texture_->GetDepthAttachmentInfo(VK_ATTACHMENT_LOAD_OP_LOAD, VK_ATTACHMENT_STORE_OP_STORE);
      render_info.pDepthAttachment = &depth_attachment;
      std::vector<VkRenderingAttachmentInfo> color_attachment_infos;
      camera->AppendGBufferColorAttachmentInfos(color_attachment_infos, VK_ATTACHMENT_LOAD_OP_LOAD,
                                                VK_ATTACHMENT_STORE_OP_STORE);
      render_info.colorAttachmentCount = color_attachment_infos.size();
      render_info.pColorAttachments = color_attachment_infos.data();

      const auto& deferred_skinned_prepass_pipeline =
          Graphics::GetGraphicsPipeline("STANDARD_SKINNED_DEFERRED_PREPASS");
      deferred_skinned_prepass_pipeline->states.ResetAllStates(color_attachment_infos.size());
      deferred_skinned_prepass_pipeline->states.view_port = viewport;
      deferred_skinned_prepass_pipeline->states.scissor = scissor;
      vkCmdBeginRendering(command_buffer, &render_info);
      deferred_skinned_prepass_pipeline->Bind(command_buffer);
      deferred_skinned_prepass_pipeline->BindDescriptorSet(
          command_buffer, 0, per_frame_descriptor_sets_[Graphics::GetCurrentFrameIndex()]->GetVkDescriptorSet());

      deferred_skinned_render_instances_.Dispatch([&](const SkinnedRenderInstance& render_command) {
        deferred_skinned_prepass_pipeline->BindDescriptorSet(
            command_buffer, 1, render_command.bone_matrices->GetDescriptorSet()->GetVkDescriptorSet());
        RenderInstancePushConstant push_constant;
        push_constant.camera_index = camera_index;
        push_constant.instance_index = render_command.instance_index;
        deferred_skinned_prepass_pipeline->states.polygon_mode =
            wire_frame ? VK_POLYGON_MODE_LINE : render_command.polygon_mode;
        deferred_skinned_prepass_pipeline->states.cull_mode = render_command.cull_mode;
        deferred_skinned_prepass_pipeline->states.line_width = render_command.line_width;
        deferred_skinned_prepass_pipeline->states.ApplyAllStates(command_buffer);
        deferred_skinned_prepass_pipeline->PushConstant(command_buffer, 0, push_constant);
        const auto skinned_mesh = render_command.skinned_mesh;
        graphics.draw_call[current_frame_index]++;
        graphics.triangles[current_frame_index] += render_command.skinned_mesh->skinned_triangles_.size();
        skinned_mesh->DrawIndexed(command_buffer, deferred_skinned_prepass_pipeline->states, 1);
      });

      vkCmdEndRendering(command_buffer);
    }
    GeometryStorage::BindStrandPoints(command_buffer);
    {
      const auto depth_attachment =
          camera->render_texture_->GetDepthAttachmentInfo(VK_ATTACHMENT_LOAD_OP_LOAD, VK_ATTACHMENT_STORE_OP_STORE);
      render_info.pDepthAttachment = &depth_attachment;
      std::vector<VkRenderingAttachmentInfo> color_attachment_infos;
      camera->AppendGBufferColorAttachmentInfos(color_attachment_infos, VK_ATTACHMENT_LOAD_OP_LOAD,
                                                VK_ATTACHMENT_STORE_OP_STORE);
      render_info.colorAttachmentCount = color_attachment_infos.size();
      render_info.pColorAttachments = color_attachment_infos.data();

      const auto& deferred_strands_prepass_pipeline =
          Graphics::GetGraphicsPipeline("STANDARD_STRANDS_DEFERRED_PREPASS");
      deferred_strands_prepass_pipeline->states.ResetAllStates(color_attachment_infos.size());
      deferred_strands_prepass_pipeline->states.view_port = viewport;
      deferred_strands_prepass_pipeline->states.scissor = scissor;

      vkCmdBeginRendering(command_buffer, &render_info);
      deferred_strands_prepass_pipeline->Bind(command_buffer);
      deferred_strands_prepass_pipeline->BindDescriptorSet(
          command_buffer, 0, per_frame_descriptor_sets_[Graphics::GetCurrentFrameIndex()]->GetVkDescriptorSet());
      deferred_strands_render_instances_.Dispatch([&](const StrandsRenderInstance& render_command) {
        RenderInstancePushConstant push_constant;
        push_constant.camera_index = camera_index;
        push_constant.instance_index = render_command.instance_index;
        deferred_strands_prepass_pipeline->states.polygon_mode =
            wire_frame ? VK_POLYGON_MODE_LINE : render_command.polygon_mode;
        deferred_strands_prepass_pipeline->states.cull_mode = render_command.cull_mode;
        deferred_strands_prepass_pipeline->states.line_width = render_command.line_width;
        deferred_strands_prepass_pipeline->PushConstant(command_buffer, 0, push_constant);
        const auto strands = render_command.m_strands;
        graphics.draw_call[current_frame_index]++;
        graphics.strands_segments[current_frame_index] += render_command.m_strands->segments_.size();
        strands->DrawIndexed(command_buffer, deferred_strands_prepass_pipeline->states, 1);
      });

      vkCmdEndRendering(command_buffer);
    }

#pragma endregion
#pragma region Lighting pass
    GeometryStorage::BindVertices(command_buffer);
    {
      camera->TransitGBufferImageLayout(command_buffer, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
      camera->render_texture_->GetDepthImage()->TransitImageLayout(command_buffer,
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
      lighting_->directional_light_shadow_map_->TransitImageLayout(command_buffer,
                                                                   VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
      const auto& deferred_lighting_pipeline =
          is_scene_camera ? Graphics::GetGraphicsPipeline("STANDARD_DEFERRED_LIGHTING_SCENE_CAMERA")
                          : Graphics::GetGraphicsPipeline("STANDARD_DEFERRED_LIGHTING");
      vkCmdBeginRendering(command_buffer, &render_info);
      deferred_lighting_pipeline->states.ResetAllStates(color_attachment_infos.size());
      deferred_lighting_pipeline->states.depth_test = false;

      deferred_lighting_pipeline->Bind(command_buffer);
      deferred_lighting_pipeline->BindDescriptorSet(
          command_buffer, 0, per_frame_descriptor_sets_[Graphics::GetCurrentFrameIndex()]->GetVkDescriptorSet());
      deferred_lighting_pipeline->BindDescriptorSet(command_buffer, 1,
                                                    camera->g_buffer_descriptor_set_->GetVkDescriptorSet());
      deferred_lighting_pipeline->BindDescriptorSet(command_buffer, 2,
                                                    lighting_->lighting_descriptor_set->GetVkDescriptorSet());
      deferred_lighting_pipeline->states.view_port = viewport;
      deferred_lighting_pipeline->states.scissor = scissor;
      RenderInstancePushConstant push_constant;
      push_constant.camera_index = camera_index;
      push_constant.light_split_index = need_fade ? glm::max(128, 256 - editor_layer->selection_alpha_) : 256;
      push_constant.instance_index = need_fade ? 1 : 0;
      deferred_lighting_pipeline->PushConstant(command_buffer, 0, push_constant);
      const auto mesh = Resources::GetResource<Mesh>("PRIMITIVE_TEX_PASS_THROUGH");

      mesh->DrawIndexed(command_buffer, deferred_lighting_pipeline->states, 1);
      vkCmdEndRendering(command_buffer);
    }
#pragma endregion
  });
#pragma endregion

#pragma region ForwardRendering

#pragma endregion

  // Post processing
  if (const auto post_processing_stack = camera->post_processing_stack.Get<PostProcessingStack>()) {
    post_processing_stack->Process(camera);
  }

  camera->rendered_ = true;
  camera->require_rendering_ = false;
}

bool RenderLayer::TryRegisterRenderer(const std::shared_ptr<Scene>& scene, const Entity& owner,
                                      const std::shared_ptr<MeshRenderer>& mesh_renderer, glm::vec3& min_bound,
                                      glm::vec3& max_bound, const bool enable_selection_highlight) {
  auto material = mesh_renderer->material.Get<Material>();
  auto mesh = mesh_renderer->mesh.Get<Mesh>();
  if (!mesh_renderer->IsEnabled() || !material || !mesh || !mesh->meshlet_range_ || !mesh->triangle_range_)
    return false;
  if (mesh->UnsafeGetVertices().empty() || mesh->UnsafeGetTriangles().empty())
    return false;

  auto gt = scene->GetDataComponent<GlobalTransform>(owner);
  auto ltw = gt.value;
  auto mesh_bound = mesh->GetBound();
  mesh_bound.ApplyTransform(ltw);
  glm::vec3 center = mesh_bound.Center();

  glm::vec3 size = mesh_bound.Size();
  min_bound = glm::vec3((glm::min)(min_bound.x, center.x - size.x), (glm::min)(min_bound.y, center.y - size.y),
                        (glm::min)(min_bound.z, center.z - size.z));
  max_bound = glm::vec3((glm::max)(max_bound.x, center.x + size.x), (glm::max)(max_bound.y, center.y + size.y),
                        (glm::max)(max_bound.z, center.z + size.z));

  MaterialInfoBlock material_info_block;
  material->UpdateMaterialInfoBlock(material_info_block);
  auto material_index = RegisterMaterialIndex(material->GetHandle(), material_info_block);
  InstanceInfoBlock instance_info_block;
  instance_info_block.model = gt;
  instance_info_block.material_index = material_index;
  instance_info_block.entity_selected = enable_selection_highlight && scene->IsEntityAncestorSelected(owner) ? 1 : 0;

  instance_info_block.meshlet_index_offset = mesh->meshlet_range_->offset;
  instance_info_block.meshlet_size = mesh->meshlet_range_->range;

  auto entity_handle = scene->GetEntityHandle(owner);
  auto instance_index = RegisterInstanceIndex(entity_handle, instance_info_block);
  RenderInstance render_instance;
  render_instance.command_type = RenderCommandType::FromRenderer;
  render_instance.m_owner = owner;
  render_instance.m_mesh = mesh;
  render_instance.cast_shadow = mesh_renderer->cast_shadow;
  render_instance.meshlet_size = mesh->meshlet_range_->range;
  render_instance.instance_index = instance_index;

  render_instance.line_width = material->draw_settings.line_width;
  render_instance.cull_mode = material->draw_settings.cull_mode;
  render_instance.polygon_mode = material->draw_settings.polygon_mode;
  if (instance_info_block.entity_selected == 1)
    need_fade_ = true;
  if (material->draw_settings.blending) {
    transparent_render_instances_.render_commands.push_back(render_instance);
  } else {
    deferred_render_instances_.render_commands.push_back(render_instance);
  }

  auto& new_mesh_task = mesh_draw_mesh_tasks_indirect_commands_.emplace_back();
  new_mesh_task.groupCountX = 1;
  new_mesh_task.groupCountY = 1;
  new_mesh_task.groupCountZ = 1;

  auto& new_draw_task = mesh_draw_indexed_indirect_commands_.emplace_back();
  new_draw_task.instanceCount = 1;
  new_draw_task.firstIndex = mesh->triangle_range_->offset * 3;
  new_draw_task.indexCount = static_cast<uint32_t>(mesh->triangles_.size() * 3);
  new_draw_task.vertexOffset = 0;
  new_draw_task.firstInstance = 0;

  total_mesh_triangles_ += mesh->triangles_.size();
  return true;
}

bool RenderLayer::TryRegisterRenderer(const std::shared_ptr<Scene>& scene, const Entity& owner,
                                      const std::shared_ptr<SkinnedMeshRenderer>& skinned_mesh_renderer,
                                      glm::vec3& min_bound, glm::vec3& max_bound, bool enable_selection_highlight) {
  auto material = skinned_mesh_renderer->material.Get<Material>();
  auto skinned_mesh = skinned_mesh_renderer->skinned_mesh.Get<SkinnedMesh>();
  if (!skinned_mesh_renderer->IsEnabled() || !material || !skinned_mesh || !skinned_mesh->skinned_meshlet_range_ ||
      !skinned_mesh->skinned_triangle_range_)
    return false;
  if (skinned_mesh->skinned_vertices_.empty() || skinned_mesh->skinned_triangles_.empty())
    return false;
  GlobalTransform gt;
  if (auto animator = skinned_mesh_renderer->animator.Get<Animator>(); !animator) {
    return false;
  }
  if (!skinned_mesh_renderer->rag_doll_) {
    gt = scene->GetDataComponent<GlobalTransform>(owner);
  }
  auto ltw = gt.value;
  auto mesh_bound = skinned_mesh->GetBound();
  mesh_bound.ApplyTransform(ltw);
  glm::vec3 center = mesh_bound.Center();

  glm::vec3 size = mesh_bound.Size();
  min_bound = glm::vec3((glm::min)(min_bound.x, center.x - size.x), (glm::min)(min_bound.y, center.y - size.y),
                        (glm::min)(min_bound.z, center.z - size.z));
  max_bound = glm::vec3((glm::max)(max_bound.x, center.x + size.x), (glm::max)(max_bound.y, center.y + size.y),
                        (glm::max)(max_bound.z, center.z + size.z));

  MaterialInfoBlock material_info_block;
  material->UpdateMaterialInfoBlock(material_info_block);
  auto material_index = RegisterMaterialIndex(material->GetHandle(), material_info_block);
  InstanceInfoBlock instance_info_block;
  instance_info_block.model = gt;
  instance_info_block.material_index = material_index;
  instance_info_block.entity_selected = enable_selection_highlight && scene->IsEntityAncestorSelected(owner) ? 1 : 0;
  instance_info_block.meshlet_size = skinned_mesh->skinned_meshlet_range_->range;
  auto entity_handle = scene->GetEntityHandle(owner);
  auto instance_index = RegisterInstanceIndex(entity_handle, instance_info_block);

  SkinnedRenderInstance render_instance;
  render_instance.command_type = RenderCommandType::FromRenderer;
  render_instance.m_owner = owner;
  render_instance.skinned_mesh = skinned_mesh;
  render_instance.cast_shadow = skinned_mesh_renderer->cast_shadow;
  render_instance.bone_matrices = skinned_mesh_renderer->bone_matrices;
  render_instance.skinned_meshlet_size = skinned_mesh->skinned_meshlet_range_->range;
  render_instance.instance_index = instance_index;

  render_instance.line_width = material->draw_settings.line_width;
  render_instance.cull_mode = material->draw_settings.cull_mode;
  render_instance.polygon_mode = material->draw_settings.polygon_mode;
  if (instance_info_block.entity_selected == 1)
    need_fade_ = true;

  if (material->draw_settings.blending) {
    transparent_skinned_render_instances_.render_commands.push_back(render_instance);
  } else {
    deferred_skinned_render_instances_.render_commands.push_back(render_instance);
  }
  return true;
}

bool RenderLayer::TryRegisterRenderer(const std::shared_ptr<Scene>& scene, const Entity& owner,
                                      const std::shared_ptr<Particles>& particles, glm::vec3& min_bound,
                                      glm::vec3& max_bound, bool enable_selection_high_light) {
  auto material = particles->material.Get<Material>();
  auto mesh = particles->mesh.Get<Mesh>();
  auto particle_info_list = particles->particle_info_list.Get<ParticleInfoList>();
  if (!particles->IsEnabled() || !material || !mesh || !mesh->meshlet_range_ || !mesh->triangle_range_ ||
      !particle_info_list)
    return false;
  if (particle_info_list->PeekParticleInfoList().empty())
    return false;
  auto gt = scene->GetDataComponent<GlobalTransform>(owner);
  auto ltw = gt.value;
  auto mesh_bound = mesh->GetBound();
  mesh_bound.ApplyTransform(ltw);
  glm::vec3 center = mesh_bound.Center();

  glm::vec3 size = mesh_bound.Size();
  min_bound = glm::vec3((glm::min)(min_bound.x, center.x - size.x), (glm::min)(min_bound.y, center.y - size.y),
                        (glm::min)(min_bound.z, center.z - size.z));

  max_bound = glm::vec3((glm::max)(max_bound.x, center.x + size.x), (glm::max)(max_bound.y, center.y + size.y),
                        (glm::max)(max_bound.z, center.z + size.z));

  MaterialInfoBlock material_info_block;
  material->UpdateMaterialInfoBlock(material_info_block);
  auto material_index = RegisterMaterialIndex(material->GetHandle(), material_info_block);
  InstanceInfoBlock instance_info_block;
  instance_info_block.model = gt;
  instance_info_block.material_index = material_index;
  instance_info_block.entity_selected = enable_selection_high_light && scene->IsEntityAncestorSelected(owner) ? 1 : 0;
  instance_info_block.meshlet_size = mesh->meshlet_range_->range;
  auto entity_handle = scene->GetEntityHandle(owner);
  auto instance_index = RegisterInstanceIndex(entity_handle, instance_info_block);

  InstancedRenderInstance render_instance;
  render_instance.command_type = RenderCommandType::FromRenderer;
  render_instance.owner = owner;
  render_instance.mesh = mesh;
  render_instance.cast_shadow = particles->cast_shadow;
  render_instance.particle_infos = particle_info_list;
  render_instance.meshlet_size = mesh->meshlet_range_->range;

  render_instance.instance_index = instance_index;

  render_instance.line_width = material->draw_settings.line_width;
  render_instance.cull_mode = material->draw_settings.cull_mode;
  render_instance.polygon_mode = material->draw_settings.polygon_mode;

  if (instance_info_block.entity_selected == 1)
    need_fade_ = true;

  if (material->draw_settings.blending) {
    transparent_instanced_render_instances_.render_commands.push_back(render_instance);
  } else {
    deferred_instanced_render_instances_.render_commands.push_back(render_instance);
  }
  return true;
}

void RenderLayer::DrawMesh(const std::shared_ptr<Mesh>& mesh, const std::shared_ptr<Material>& material,
                           glm::mat4 model, bool cast_shadow) {
  auto scene = Application::GetActiveScene();
  MaterialInfoBlock material_info_block;
  material->UpdateMaterialInfoBlock(material_info_block);
  auto material_index = RegisterMaterialIndex(material->GetHandle(), material_info_block);
  InstanceInfoBlock instance_info_block;
  instance_info_block.model.value = model;
  instance_info_block.material_index = material_index;
  instance_info_block.entity_selected = 0;
  instance_info_block.meshlet_index_offset = mesh->meshlet_range_->offset;
  instance_info_block.meshlet_size = mesh->meshlet_range_->range;

  auto entity_handle = Handle();
  auto instance_index = RegisterInstanceIndex(entity_handle, instance_info_block);
  RenderInstance render_instance;
  render_instance.command_type = RenderCommandType::FromApi;
  render_instance.m_owner = Entity();
  render_instance.m_mesh = mesh;
  render_instance.cast_shadow = cast_shadow;
  render_instance.meshlet_size = mesh->meshlet_range_->range;
  render_instance.instance_index = instance_index;
  if (instance_info_block.entity_selected == 1)
    need_fade_ = true;
  if (material->draw_settings.blending) {
    transparent_render_instances_.render_commands.push_back(render_instance);
  } else {
    deferred_render_instances_.render_commands.push_back(render_instance);
  }

  auto& new_mesh_task = mesh_draw_mesh_tasks_indirect_commands_.emplace_back();
  new_mesh_task.groupCountX = 1;
  new_mesh_task.groupCountY = 1;
  new_mesh_task.groupCountZ = 1;

  auto& new_draw_task = mesh_draw_indexed_indirect_commands_.emplace_back();
  new_draw_task.instanceCount = 1;
  new_draw_task.firstIndex = mesh->triangle_range_->offset * 3;
  new_draw_task.indexCount = static_cast<uint32_t>(mesh->triangles_.size() * 3);
  new_draw_task.vertexOffset = 0;
  new_draw_task.firstInstance = 0;

  total_mesh_triangles_ += mesh->triangles_.size();
}

const std::shared_ptr<DescriptorSet>& RenderLayer::GetPerFrameDescriptorSet() const {
  return per_frame_descriptor_sets_[Graphics::GetCurrentFrameIndex()];
}

bool RenderLayer::TryRegisterRenderer(const std::shared_ptr<Scene>& scene, const Entity& owner,
                                      const std::shared_ptr<StrandsRenderer>& strands_renderer, glm::vec3& min_bound,
                                      glm::vec3& max_bound, bool enable_selection_high_light) {
  auto material = strands_renderer->material.Get<Material>();
  auto strands = strands_renderer->strands.Get<Strands>();
  if (!strands_renderer->IsEnabled() || !material || !strands || !strands->strand_meshlet_range_ ||
      !strands->segment_range_)
    return false;
  auto gt = scene->GetDataComponent<GlobalTransform>(owner);
  auto ltw = gt.value;
  auto mesh_bound = strands->bound_;
  mesh_bound.ApplyTransform(ltw);
  glm::vec3 center = mesh_bound.Center();

  glm::vec3 size = mesh_bound.Size();
  min_bound = glm::vec3((glm::min)(min_bound.x, center.x - size.x), (glm::min)(min_bound.y, center.y - size.y),
                        (glm::min)(min_bound.z, center.z - size.z));
  max_bound = glm::vec3((glm::max)(max_bound.x, center.x + size.x), (glm::max)(max_bound.y, center.y + size.y),
                        (glm::max)(max_bound.z, center.z + size.z));

  MaterialInfoBlock material_info_block;
  material->UpdateMaterialInfoBlock(material_info_block);
  auto material_index = RegisterMaterialIndex(material->GetHandle(), material_info_block);
  InstanceInfoBlock instance_info_block;
  instance_info_block.model = gt;
  instance_info_block.material_index = material_index;
  instance_info_block.entity_selected = enable_selection_high_light && scene->IsEntityAncestorSelected(owner) ? 1 : 0;
  instance_info_block.meshlet_size = strands->strand_meshlet_range_->range;
  auto entity_handle = scene->GetEntityHandle(owner);
  auto instance_index = RegisterInstanceIndex(entity_handle, instance_info_block);

  StrandsRenderInstance render_instance;
  render_instance.command_type = RenderCommandType::FromRenderer;
  render_instance.m_owner = owner;
  render_instance.m_strands = strands;
  render_instance.cast_shadow = strands_renderer->cast_shadow;
  render_instance.strand_meshlet_size = strands->strand_meshlet_range_->range;

  render_instance.instance_index = instance_index;

  render_instance.line_width = material->draw_settings.line_width;
  render_instance.cull_mode = material->draw_settings.cull_mode;
  render_instance.polygon_mode = material->draw_settings.polygon_mode;

  if (instance_info_block.entity_selected == 1)
    need_fade_ = true;

  if (material->draw_settings.blending) {
    transparent_strands_render_instances_.render_commands.push_back(render_instance);
  } else {
    deferred_strands_render_instances_.render_commands.push_back(render_instance);
  }
  return true;
}
