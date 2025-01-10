#include "RenderInstanceStorage.hpp"

#include "Application.hpp"
#include "EditorLayer.hpp"
#include "LodGroup.hpp"
#include "RenderLayer.hpp"

using namespace evo_engine;

void RenderSettings::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  ImGui::Checkbox("Show entities", &enable_debug_visualization);
  if (ImGui::CollapsingHeader("Shadow", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (ImGui::TreeNode("Distance")) {
      if (ImGui::DragFloat("Max shadow distance", &max_shadow_distance, 1.0f, 10.f, 1000.f)) {
        max_shadow_distance = glm::clamp(max_shadow_distance, 10.f, 1000.f);
      }
      if (ImGui::DragFloat("Split 1", &shadow_cascade_split[0], 0.01f, 0.0f, shadow_cascade_split[1])) {
        shadow_cascade_split[0] = glm::clamp(shadow_cascade_split[0], 0.f, shadow_cascade_split[1]);
      }
      if (ImGui::DragFloat("Split 2", &shadow_cascade_split[1], 0.01f, shadow_cascade_split[0],
                           shadow_cascade_split[2])) {
        shadow_cascade_split[1] = glm::clamp(shadow_cascade_split[1], shadow_cascade_split[0], shadow_cascade_split[2]);
      }
      if (ImGui::DragFloat("Split 3", &shadow_cascade_split[2], 0.01f, shadow_cascade_split[1],
                           shadow_cascade_split[3])) {
        shadow_cascade_split[2] = glm::clamp(shadow_cascade_split[2], shadow_cascade_split[1], shadow_cascade_split[3]);
      }
      if (ImGui::DragFloat("Split 4", &shadow_cascade_split[3], 0.01f, shadow_cascade_split[2], 1.0f)) {
        shadow_cascade_split[3] = glm::clamp(shadow_cascade_split[3], shadow_cascade_split[2], 1.f);
      }
      ImGui::TreePop();
    }
    if (ImGui::TreeNode("PCSS")) {
      ImGui::DragInt("PCF Sample Size", &pcf_sample_amount, 1, 1, 64);
      ImGui::TreePop();
    }
    ImGui::DragFloat("Seam fix ratio", &seam_fix_ratio, 0.001f, 0.0f, 0.1f);
    ImGui::Checkbox("Stable fit", &stable_fit);
  }
#ifdef EVOENGINE_WINDOWS
  if (ImGui::TreeNodeEx("Strands settings", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::DragFloat("Curve subdivision factor", &strands_subdivision_x_factor, 1.0f, 1.0f, 1000.0f);
    ImGui::DragFloat("Ring subdivision factor", &strands_subdivision_y_factor, 1.0f, 1.0f, 1000.0f);
    ImGui::DragInt("Max curve subdivision", &strands_subdivision_max_x, 1, 1, 15);
    ImGui::DragInt("Max ring subdivision", &strands_subdivision_max_y, 1, 1, 15);

    ImGui::TreePop();
  }
#endif
}

bool RenderInstanceStorage::ExternalRenderInstance::operator!=(const ExternalRenderInstance& other) const {
  if (entity_selected != other.entity_selected)
    return true;
  if (instance_index != other.instance_index)
    return true;
  if (command_type != other.command_type)
    return true;
  if (model.value != other.model.value)
    return true;
  if (owner != other.owner)
    return true;
  if (material != other.material)
    return true;
  if (geometry_version != other.geometry_version)
    return true;
  if (material_version != other.material_version)
    return true;
  if (cast_shadow != other.cast_shadow)
    return true;
  if (line_width != other.line_width)
    return true;
  if (cull_mode != other.cull_mode)
    return true;
  if (polygon_mode != other.polygon_mode)
    return true;
  return false;
}

void RenderInstanceStorage::ExternalRenderInstance::Apply(InstanceInfoBlock& instance_info_block) const {
  instance_info_block.model = model;
  instance_info_block.material_index = material_index;
  instance_info_block.triangle_offset = 0;
  instance_info_block.meshlet_index_offset = 0;
  instance_info_block.meshlet_size = 0;
  instance_info_block.entity_selected = entity_selected;
}

uint32_t RenderInstanceStorage::ExternalRenderInstance::Render(
    VkCommandBuffer vk_command_buffer, const RenderInstancePushConstant& render_instance_push_constant,
    const std::shared_ptr<GraphicsPipeline>& graphics_pipeline) const {
  return 0;
}

bool RenderInstanceStorage::MeshRenderInstance::operator!=(const MeshRenderInstance& other) const {
  if (entity_selected != other.entity_selected)
    return true;
  if (instance_index != other.instance_index)
    return true;
  if (command_type != other.command_type)
    return true;
  if (model.value != other.model.value)
    return true;
  if (owner != other.owner)
    return true;
  if (mesh != other.mesh)
    return true;
  if (material != other.material)
    return true;
  if (geometry_version != other.geometry_version)
    return true;
  if (material_version != other.material_version)
    return true;
  if (cast_shadow != other.cast_shadow)
    return true;
  if (line_width != other.line_width)
    return true;
  if (cull_mode != other.cull_mode)
    return true;
  if (polygon_mode != other.polygon_mode)
    return true;
  return false;
}

void RenderInstanceStorage::MeshRenderInstance::Apply(InstanceInfoBlock& instance_info_block) const {
  instance_info_block.model = model;
  instance_info_block.material_index = material_index;
  instance_info_block.entity_selected = entity_selected;
  instance_info_block.triangle_offset = mesh->triangle_range_->offset;
  instance_info_block.meshlet_index_offset = mesh->meshlet_range_->offset;
  instance_info_block.meshlet_size = mesh->meshlet_range_->range;
}

uint32_t RenderInstanceStorage::MeshRenderInstance::Render(
    const VkCommandBuffer vk_command_buffer, const RenderInstancePushConstant& render_instance_push_constant,
    const std::shared_ptr<GraphicsPipeline>& graphics_pipeline) const {
  const uint32_t task_work_group_invocations =
      Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxPreferredTaskWorkGroupInvocations;
  graphics_pipeline->PushConstant(vk_command_buffer, 0, render_instance_push_constant);
  if (Platform::Settings::use_mesh_shader) {
    graphics_pipeline->states.ApplyAllStates(vk_command_buffer);
    const uint32_t count =
        (mesh->meshlet_range_->range + task_work_group_invocations - 1) / task_work_group_invocations;
    vkCmdDrawMeshTasksEXT(vk_command_buffer, count, 1, 1);
  } else {
    mesh->DrawIndexed(vk_command_buffer, graphics_pipeline->states, 1);
  }
  return mesh->GetTriangleAmount();
}

bool RenderInstanceStorage::SkinnedMeshRenderInstance::operator!=(const SkinnedMeshRenderInstance& other) const {
  if (entity_selected != other.entity_selected)
    return true;
  if (instance_index != other.instance_index)
    return true;
  if (command_type != other.command_type)
    return true;
  if (model.value != other.model.value)
    return true;
  if (owner != other.owner)
    return true;
  if (material != other.material)
    return true;
  if (skinned_mesh != other.skinned_mesh)
    return true;
  if (bone_matrices != other.bone_matrices)
    return true;
  if (geometry_version != other.geometry_version)
    return true;
  if (material_version != other.material_version)
    return true;
  if (bone_matrices_version != other.bone_matrices_version)
    return true;
  if (cast_shadow != other.cast_shadow)
    return true;
  if (line_width != other.line_width)
    return true;
  if (cull_mode != other.cull_mode)
    return true;
  if (polygon_mode != other.polygon_mode)
    return true;
  return false;
}

void RenderInstanceStorage::SkinnedMeshRenderInstance::Apply(InstanceInfoBlock& instance_info_block) const {
  instance_info_block.model = model;
  instance_info_block.material_index = material_index;
  instance_info_block.entity_selected = entity_selected;
  instance_info_block.triangle_offset = skinned_mesh->skinned_triangle_range_->offset;
  instance_info_block.meshlet_index_offset = skinned_mesh->skinned_meshlet_range_->offset;
  instance_info_block.meshlet_size = skinned_mesh->skinned_meshlet_range_->range;
}

uint32_t RenderInstanceStorage::SkinnedMeshRenderInstance::Render(
    VkCommandBuffer vk_command_buffer, const RenderInstancePushConstant& render_instance_push_constant,
    const std::shared_ptr<GraphicsPipeline>& graphics_pipeline) const {
  graphics_pipeline->BindDescriptorSet(vk_command_buffer, 1, bone_matrices->GetDescriptorSet()->GetVkDescriptorSet());
  graphics_pipeline->PushConstant(vk_command_buffer, 0, render_instance_push_constant);
  skinned_mesh->DrawIndexed(vk_command_buffer, graphics_pipeline->states, 1);
  return skinned_mesh->GetTriangleAmount();
}

bool RenderInstanceStorage::InstancedRenderInstance::operator!=(const InstancedRenderInstance& other) const {
  if (entity_selected != other.entity_selected)
    return true;
  if (instance_index != other.instance_index)
    return true;
  if (command_type != other.command_type)
    return true;
  if (model.value != other.model.value)
    return true;
  if (owner != other.owner)
    return true;
  if (material != other.material)
    return true;
  if (mesh != other.mesh)
    return true;
  if (particle_infos != other.particle_infos)
    return true;
  if (geometry_version != other.geometry_version)
    return true;
  if (material_version != other.material_version)
    return true;
  if (particle_info_list_version != other.particle_info_list_version)
    return true;
  if (cast_shadow != other.cast_shadow)
    return true;
  if (line_width != other.line_width)
    return true;
  if (cull_mode != other.cull_mode)
    return true;
  if (polygon_mode != other.polygon_mode)
    return true;
  return false;
}

void RenderInstanceStorage::InstancedRenderInstance::Apply(InstanceInfoBlock& instance_info_block) const {
  instance_info_block.model = model;
  instance_info_block.material_index = material_index;
  instance_info_block.entity_selected = entity_selected;
  instance_info_block.triangle_offset = mesh->triangle_range_->offset;
  instance_info_block.meshlet_index_offset = mesh->meshlet_range_->offset;
  instance_info_block.meshlet_size = mesh->meshlet_range_->range;
}

uint32_t RenderInstanceStorage::InstancedRenderInstance::Render(
    const VkCommandBuffer vk_command_buffer, const RenderInstancePushConstant& render_instance_push_constant,
    const std::shared_ptr<GraphicsPipeline>& graphics_pipeline) const {
  graphics_pipeline->BindDescriptorSet(vk_command_buffer, 1, particle_infos->GetDescriptorSet()->GetVkDescriptorSet());
  graphics_pipeline->PushConstant(vk_command_buffer, 0, render_instance_push_constant);
  mesh->DrawIndexed(vk_command_buffer, graphics_pipeline->states, particle_infos->PeekParticleInfoList().size());
  return mesh->UnsafeGetTriangles().size() * particle_infos->PeekParticleInfoList().size();
}

bool RenderInstanceStorage::StrandsRenderInstance::operator!=(const StrandsRenderInstance& other) const {
  if (entity_selected != other.entity_selected)
    return true;
  if (instance_index != other.instance_index)
    return true;
  if (command_type != other.command_type)
    return true;
  if (model.value != other.model.value)
    return true;
  if (owner != other.owner)
    return true;
  if (material != other.material)
    return true;
  if (strands != other.strands)
    return true;

  if (geometry_version != other.geometry_version)
    return true;
  if (material_version != other.material_version)
    return true;

  if (cast_shadow != other.cast_shadow)
    return true;
  if (line_width != other.line_width)
    return true;
  if (cull_mode != other.cull_mode)
    return true;
  if (polygon_mode != other.polygon_mode)
    return true;
  return false;
}

void RenderInstanceStorage::StrandsRenderInstance::Apply(InstanceInfoBlock& instance_info_block) const {
  instance_info_block.model = model;
  instance_info_block.material_index = material_index;
  instance_info_block.entity_selected = entity_selected;
  instance_info_block.triangle_offset = strands->segment_range_->offset;
  instance_info_block.meshlet_index_offset = strands->strand_meshlet_range_->offset;
  instance_info_block.meshlet_size = strands->strand_meshlet_range_->range;
}

uint32_t RenderInstanceStorage::StrandsRenderInstance::Render(
    const VkCommandBuffer vk_command_buffer, const RenderInstancePushConstant& render_instance_push_constant,
    const std::shared_ptr<GraphicsPipeline>& graphics_pipeline) const {
  graphics_pipeline->PushConstant(vk_command_buffer, 0, render_instance_push_constant);
  strands->DrawIndexed(vk_command_buffer, graphics_pipeline->states, 1);
  return strands->GetSegmentAmount();
}

bool RenderInstanceStorage::ExternalRenderInstanceCollection::operator!=(
    const ExternalRenderInstanceCollection& other) const {
  if (render_commands.size() != other.render_commands.size())
    return true;
  for (uint32_t i = 0; i < render_commands.size(); i++) {
    if (render_commands[i] != other.render_commands[i])
      return true;
  }
  return false;
}

bool RenderInstanceStorage::ExternalRenderInstanceCollection::Empty() const {
  return render_commands.empty();
}

void RenderInstanceStorage::ExternalRenderInstanceCollection::Register(
    const std::shared_ptr<IRenderInstance>& render_instance) {
  render_commands.emplace_back(std::dynamic_pointer_cast<ExternalRenderInstance>(render_instance));
}

void RenderInstanceStorage::ExternalRenderInstanceCollection::ForEachRenderInstance(
    const std::function<void(const std::shared_ptr<IRenderInstance>&)>& action) {
  for (const auto& i : render_commands) {
    action(i);
  }
}

bool RenderInstanceStorage::MeshRenderInstanceCollection::operator!=(const MeshRenderInstanceCollection& other) const {
  if (render_commands.size() != other.render_commands.size())
    return true;
  for (uint32_t i = 0; i < render_commands.size(); i++) {
    if (render_commands[i] != other.render_commands[i])
      return true;
  }
  return false;
}

bool RenderInstanceStorage::MeshRenderInstanceCollection::Empty() const {
  return render_commands.empty();
}

void RenderInstanceStorage::MeshRenderInstanceCollection::Register(
    const std::shared_ptr<IRenderInstance>& render_instance) {
  render_commands.emplace_back(std::dynamic_pointer_cast<MeshRenderInstance>(render_instance));
}

void RenderInstanceStorage::MeshRenderInstanceCollection::ForEachRenderInstance(
    const std::function<void(const std::shared_ptr<IRenderInstance>&)>& action) {
  for (const auto& i : render_commands) {
    action(i);
  }
}

bool RenderInstanceStorage::SkinnedMeshRenderInstanceCollection::Empty() const {
  return render_commands.empty();
}

void RenderInstanceStorage::SkinnedMeshRenderInstanceCollection::Register(
    const std::shared_ptr<IRenderInstance>& render_instance) {
  render_commands.emplace_back(std::dynamic_pointer_cast<SkinnedMeshRenderInstance>(render_instance));
}

bool RenderInstanceStorage::SkinnedMeshRenderInstanceCollection::operator!=(
    const SkinnedMeshRenderInstanceCollection& other) const {
  if (render_commands.size() != other.render_commands.size())
    return true;
  for (uint32_t i = 0; i < render_commands.size(); i++) {
    if (render_commands[i] != other.render_commands[i])
      return true;
  }
  return false;
}

void RenderInstanceStorage::SkinnedMeshRenderInstanceCollection::ForEachRenderInstance(
    const std::function<void(const std::shared_ptr<IRenderInstance>&)>& action) {
  for (const auto& i : render_commands) {
    action(i);
  }
}

bool RenderInstanceStorage::StrandsRenderInstanceCollection::Empty() const {
  return render_commands.empty();
}

void RenderInstanceStorage::StrandsRenderInstanceCollection::Register(
    const std::shared_ptr<IRenderInstance>& render_instance) {
  render_commands.emplace_back(std::dynamic_pointer_cast<StrandsRenderInstance>(render_instance));
}

bool RenderInstanceStorage::StrandsRenderInstanceCollection::operator!=(
    const StrandsRenderInstanceCollection& other) const {
  if (render_commands.size() != other.render_commands.size())
    return true;
  for (uint32_t i = 0; i < render_commands.size(); i++) {
    if (render_commands[i] != other.render_commands[i])
      return true;
  }
  return false;
}

void RenderInstanceStorage::StrandsRenderInstanceCollection::ForEachRenderInstance(
    const std::function<void(const std::shared_ptr<IRenderInstance>&)>& action) {
  for (const auto& i : render_commands) {
    action(i);
  }
}

bool RenderInstanceStorage::InstancedRenderInstanceCollection::Empty() const {
  return render_commands.empty();
}

void RenderInstanceStorage::InstancedRenderInstanceCollection::Register(
    const std::shared_ptr<IRenderInstance>& render_instance) {
  render_commands.emplace_back(std::dynamic_pointer_cast<InstancedRenderInstance>(render_instance));
}

bool RenderInstanceStorage::InstancedRenderInstanceCollection::operator!=(
    const InstancedRenderInstanceCollection& other) const {
  if (render_commands.size() != other.render_commands.size())
    return true;
  for (uint32_t i = 0; i < render_commands.size(); i++) {
    if (render_commands[i] != other.render_commands[i])
      return true;
  }
  return false;
}

void RenderInstanceStorage::InstancedRenderInstanceCollection::ForEachRenderInstance(
    const std::function<void(const std::shared_ptr<IRenderInstance>&)>& action) {
  for (const auto& i : render_commands) {
    action(i);
  }
}

void RenderInstanceStorage::RenderInfoBlock::Apply(const RenderSettings& target_render_settings) {
  for (int split = 0; split < 4; split++) {
    float split_end = target_render_settings.max_shadow_distance;
    if (split != 3)
      split_end = target_render_settings.max_shadow_distance * target_render_settings.shadow_cascade_split[split];
    split_distances[split] = split_end;
  }
  if (const auto render_layer = Application::GetLayer<RenderLayer>()) {
    brdflut_texture_index = render_layer->environmental_brdf_lut_->GetTextureStorageIndex();
  }
  if (target_render_settings.enable_debug_visualization)
    debug_visualization = 1;
  else
    debug_visualization = 0;

  pcf_sample_amount = target_render_settings.pcf_sample_amount;
  seam_fix_ratio = target_render_settings.seam_fix_ratio;
  strands_subdivision_x_factor = target_render_settings.strands_subdivision_x_factor;
  strands_subdivision_y_factor = target_render_settings.strands_subdivision_y_factor;
  strands_subdivision_max_x = target_render_settings.strands_subdivision_max_x;
  strands_subdivision_max_y = target_render_settings.strands_subdivision_max_y;
}

bool RenderInstanceStorage::RenderInfoBlock::operator!=(const RenderInfoBlock& other) const {
  if (split_distances != other.split_distances)
    return true;

  if (pcf_sample_amount != other.pcf_sample_amount)
    return true;

  if (seam_fix_ratio != other.seam_fix_ratio)
    return true;

  if (strands_subdivision_x_factor != other.strands_subdivision_x_factor)
    return true;
  if (strands_subdivision_y_factor != other.strands_subdivision_y_factor)
    return true;
  if (strands_subdivision_max_x != other.strands_subdivision_max_x)
    return true;
  if (strands_subdivision_max_y != other.strands_subdivision_max_y)
    return true;

  if (directional_light_size != other.directional_light_size)
    return true;
  if (point_light_size != other.point_light_size)
    return true;
  if (spot_light_size != other.spot_light_size)
    return true;
  if (brdflut_texture_index != other.brdflut_texture_index)
    return true;

  if (debug_visualization != other.debug_visualization)
    return true;

  return false;
}

bool RenderInstanceStorage::EnvironmentInfoBlock::operator!=(const EnvironmentInfoBlock& other) const {
  if (background_color != other.background_color)
    return true;
  if (environmental_map_gamma != other.environmental_map_gamma)
    return true;
  if (environmental_lighting_intensity != other.environmental_lighting_intensity)
    return true;
  if (background_intensity != other.background_intensity)
    return true;
  if (environmental_padding2 != other.environmental_padding2)
    return true;

  return false;
}

bool RenderInstanceStorage::InstanceInfoBlock::operator!=(const InstanceInfoBlock& other) const {
  if (model != other.model)
    return true;
  if (material_index != other.material_index)
    return true;
  if (triangle_offset != other.triangle_offset)
    return true;
  if (meshlet_index_offset != other.meshlet_index_offset)
    return true;
  if (meshlet_size != other.meshlet_size)
    return true;
  if (entity_selected != other.entity_selected)
    return true;
  return false;
}

void RenderInstanceStorage::MaterialInfoBlock::Apply(const std::shared_ptr<Material>& target_material) {
  if (const auto albedo_texture = target_material->GetAlbedoTexture();
      albedo_texture && albedo_texture->GetVkSampler()) {
    albedo_texture_index = albedo_texture->GetTextureStorageIndex();
  } else {
    albedo_texture_index = -1;
  }
  if (const auto normal_texture = target_material->GetNormalTexture();
      normal_texture && normal_texture->GetVkSampler()) {
    normal_texture_index = normal_texture->GetTextureStorageIndex();
  } else {
    normal_texture_index = -1;
  }
  if (const auto metallic_texture = target_material->GetMetallicTexture();
      metallic_texture && metallic_texture->GetVkSampler()) {
    metallic_texture_index = metallic_texture->GetTextureStorageIndex();
  } else {
    metallic_texture_index = -1;
  }
  if (const auto roughness_texture = target_material->GetRoughnessTexture();
      roughness_texture && roughness_texture->GetVkSampler()) {
    roughness_texture_index = roughness_texture->GetTextureStorageIndex();
  } else {
    roughness_texture_index = -1;
  }
  if (const auto ao_texture = target_material->GetAoTexture(); ao_texture && ao_texture->GetVkSampler()) {
    ao_texture_index = ao_texture->GetTextureStorageIndex();
  } else {
    ao_texture_index = -1;
  }
  cast_shadow = true;
  subsurface_color = {target_material->material_properties.subsurface_color, 0.0f};
  subsurface_radius = {target_material->material_properties.subsurface_radius, 0.0f};
  albedo_color_val = glm::vec4(
      target_material->material_properties.albedo_color,
      target_material->draw_settings.blending ? (1.0f - target_material->material_properties.transmission) : 1.0f);
  metallic_val = target_material->material_properties.metallic;
  roughness_val = target_material->material_properties.roughness;
  ao_val = 1.0f;
  emission_val = target_material->material_properties.emission;
}

bool RenderInstanceStorage::MaterialInfoBlock::operator!=(const MaterialInfoBlock& other) const {
  if (albedo_texture_index != other.albedo_texture_index)
    return true;
  if (normal_texture_index != other.normal_texture_index)
    return true;
  if (metallic_texture_index != other.metallic_texture_index)
    return true;
  if (roughness_texture_index != other.roughness_texture_index)
    return true;

  if (ao_texture_index != other.ao_texture_index)
    return true;
  if (cast_shadow != other.cast_shadow)
    return true;
  if (receive_shadow != other.receive_shadow)
    return true;
  if (enable_shadow != other.enable_shadow)
    return true;

  if (albedo_color_val != other.albedo_color_val)
    return true;
  if (subsurface_color != other.subsurface_color)
    return true;
  if (subsurface_radius != other.subsurface_radius)
    return true;

  if (metallic_val != other.metallic_val)
    return true;
  if (roughness_val != other.roughness_val)
    return true;
  if (ao_val != other.ao_val)
    return true;
  if (emission_val != other.emission_val)
    return true;
  return false;
}

void RenderInstanceStorage::CollectEntityRenderers(const std::shared_ptr<Scene>& target_scene, Bound& world_bound) {
  auto& min_bound = world_bound.min;
  auto& max_bound = world_bound.max;
  geometry_storage_version = GeometryStorage::GetVersion();
  texture_storage_version = TextureStorage::GetVersion();
  bool has_render_instance = false;
  std::unordered_set<Handle> lod_group_renderers{};
  if (const auto* owners = target_scene->UnsafeGetPrivateComponentOwnersList<LodGroup>()) {
    for (auto owner : *owners) {
      const auto lod_group = target_scene->GetOrSetPrivateComponent<LodGroup>(owner).lock();
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
            if (render_current_level && target_scene->IsEntityEnabled(owner) &&
                target_scene->IsEntityEnabled(mesh_renderer->GetOwner()) && mesh_renderer->IsEnabled()) {
              if (RegisterEntity(target_scene, owner, mesh_renderer, min_bound, max_bound)) {
                has_render_instance = true;
              }
            }
          } else if (const auto skinned_mesh_renderer = renderer.Get<SkinnedMeshRenderer>()) {
            lod_group_renderers.insert(skinned_mesh_renderer->GetHandle());
            if (render_current_level && target_scene->IsEntityEnabled(owner) &&
                target_scene->IsEntityEnabled(skinned_mesh_renderer->GetOwner()) &&
                skinned_mesh_renderer->IsEnabled()) {
              if (RegisterEntity(target_scene, owner, skinned_mesh_renderer, min_bound, max_bound)) {
                has_render_instance = true;
              }
            }
          } else if (const auto particles = renderer.Get<Particles>()) {
            lod_group_renderers.insert(particles->GetHandle());
            if (render_current_level && target_scene->IsEntityEnabled(owner) &&
                target_scene->IsEntityEnabled(particles->GetOwner()) && particles->IsEnabled()) {
              if (RegisterEntity(target_scene, owner, particles, min_bound, max_bound)) {
                has_render_instance = true;
              }
            }
          } else if (const auto strands_renderer = renderer.Get<StrandsRenderer>()) {
            lod_group_renderers.insert(strands_renderer->GetHandle());
            if (render_current_level && target_scene->IsEntityEnabled(owner) &&
                target_scene->IsEntityEnabled(strands_renderer->GetOwner()) && strands_renderer->IsEnabled()) {
              if (RegisterEntity(target_scene, owner, strands_renderer, min_bound, max_bound)) {
                has_render_instance = true;
              }
            }
          }
        }
      }
    }
  }

  if (const auto* owners = target_scene->UnsafeGetPrivateComponentOwnersList<MeshRenderer>()) {
    for (auto owner : *owners) {
      if (!target_scene->IsEntityEnabled(owner))
        continue;
      auto mesh_renderer = target_scene->GetOrSetPrivateComponent<MeshRenderer>(owner).lock();
      if (lod_group_renderers.find(mesh_renderer->GetHandle()) != lod_group_renderers.end())
        continue;
      if (RegisterEntity(target_scene, owner, mesh_renderer, min_bound, max_bound)) {
        has_render_instance = true;
      }
    }
  }

  if (const auto* owners = target_scene->UnsafeGetPrivateComponentOwnersList<SkinnedMeshRenderer>()) {
    for (auto owner : *owners) {
      if (!target_scene->IsEntityEnabled(owner))
        continue;
      auto skinned_mesh_renderer = target_scene->GetOrSetPrivateComponent<SkinnedMeshRenderer>(owner).lock();
      if (lod_group_renderers.find(skinned_mesh_renderer->GetHandle()) != lod_group_renderers.end())
        continue;
      if (RegisterEntity(target_scene, owner, skinned_mesh_renderer, min_bound, max_bound)) {
        has_render_instance = true;
      }
    }
  }

  if (const auto* owners = target_scene->UnsafeGetPrivateComponentOwnersList<Particles>()) {
    for (auto owner : *owners) {
      if (!target_scene->IsEntityEnabled(owner))
        continue;
      auto particles = target_scene->GetOrSetPrivateComponent<Particles>(owner).lock();
      if (lod_group_renderers.find(particles->GetHandle()) != lod_group_renderers.end())
        continue;
      if (RegisterEntity(target_scene, owner, particles, min_bound, max_bound)) {
        has_render_instance = true;
      }
    }
  }

  if (const auto* owners = target_scene->UnsafeGetPrivateComponentOwnersList<StrandsRenderer>()) {
    for (auto owner : *owners) {
      if (!target_scene->IsEntityEnabled(owner))
        continue;
      auto strands_renderer = target_scene->GetOrSetPrivateComponent<StrandsRenderer>(owner).lock();
      if (lod_group_renderers.find(strands_renderer->GetHandle()) != lod_group_renderers.end())
        continue;
      if (RegisterEntity(target_scene, owner, strands_renderer, min_bound, max_bound)) {
        has_render_instance = true;
      }
    }
  }

  if (!has_render_instance) {
    min_bound = max_bound = glm::vec3(0.0f);
  }
}

void RenderInstanceStorage::BuildRenderInstanceBlocks() {
  const auto register_render_instance = [&](const std::shared_ptr<IRenderInstance>& render_instance) {
    render_instance->instance_index = instance_info_blocks_.size();
    if (render_instance->entity_handle != 0)
      instance_entity_handles_[render_instance->instance_index] = render_instance->entity_handle;
    if (render_instance->renderer_handle != 0) {
      instance_renderer_handles_[render_instance->instance_index] = render_instance->renderer_handle;
      renderer_indices_[render_instance->renderer_handle] = render_instance->instance_index;
    }
    auto& render_instance_block = instance_info_blocks_.emplace_back();
    render_instance->Apply(render_instance_block);
  };
  deferred_render_instances->ForEachRenderInstance([&](const auto& render_instance) {
    register_render_instance(render_instance);
  });
  deferred_skinned_render_instances->ForEachRenderInstance([&](const auto& render_instance) {
    register_render_instance(render_instance);
  });
  deferred_instanced_render_instances->ForEachRenderInstance([&](const auto& render_instance) {
    register_render_instance(render_instance);
  });
  deferred_strands_render_instances->ForEachRenderInstance([&](const auto& render_instance) {
    register_render_instance(render_instance);
  });

  forward_render_instances->ForEachRenderInstance([&](const auto& render_instance) {
    register_render_instance(render_instance);
  });
  forward_skinned_render_instances->ForEachRenderInstance([&](const auto& render_instance) {
    register_render_instance(render_instance);
  });
  forward_instanced_render_instances->ForEachRenderInstance([&](const auto& render_instance) {
    register_render_instance(render_instance);
  });
  forward_strands_render_instances->ForEachRenderInstance([&](const auto& render_instance) {
    register_render_instance(render_instance);
  });

  transparent_render_instances->ForEachRenderInstance([&](const auto& render_instance) {
    register_render_instance(render_instance);
  });
  transparent_skinned_render_instances->ForEachRenderInstance([&](const auto& render_instance) {
    register_render_instance(render_instance);
  });
  transparent_instanced_render_instances->ForEachRenderInstance([&](const auto& render_instance) {
    register_render_instance(render_instance);
  });
  transparent_strands_render_instances->ForEachRenderInstance([&](const auto& render_instance) {
    register_render_instance(render_instance);
  });

  external_render_instances->ForEachRenderInstance([&](const auto& render_instance) {
    register_render_instance(render_instance);
  });
}

void RenderInstanceStorage::CollectLights(const std::shared_ptr<Scene>& target_scene, const Bound& world_bound) {
  auto& min_bound = world_bound.min;
  auto& max_bound = world_bound.max;
#pragma region Directional Light
  const std::vector<Entity>* directional_light_entities =
      target_scene->UnsafeGetPrivateComponentOwnersList<DirectionalLight>();
  render_info_block.directional_light_size = 0;
  if (directional_light_entities && !directional_light_entities->empty()) {
    directional_light_info_blocks_.resize(Platform::Settings::max_directional_light_size * cameras.size());
    for (const auto& light_entity : *directional_light_entities) {
      if (!target_scene->IsEntityEnabled(light_entity))
        continue;
      const auto dlc = target_scene->GetOrSetPrivateComponent<DirectionalLight>(light_entity).lock();
      if (!dlc->IsEnabled())
        continue;
      render_info_block.directional_light_size++;
    }
    std::vector<glm::uvec3> viewport_results;
    Lighting::AllocateAtlas(render_info_block.directional_light_size,
                            Platform::Settings::directional_light_shadow_map_resolution, viewport_results);
    for (const auto& [cameraGlobalTransform, camera] : cameras) {
      auto camera_index = GetCameraIndex(camera->GetHandle());
      for (int i = 0; i < render_info_block.directional_light_size; i++) {
        const auto block_index = camera_index * Platform::Settings::max_directional_light_size + i;
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
        if (!target_scene->IsEntityEnabled(light_entity))
          continue;
        const auto dlc = target_scene->GetOrSetPrivateComponent<DirectionalLight>(light_entity).lock();
        if (!dlc->IsEnabled())
          continue;
        glm::quat rotation = target_scene->GetDataComponent<GlobalTransform>(light_entity).GetRotation();
        glm::vec3 light_dir = glm::normalize(rotation * glm::vec3(0, 0, 1));
        float plane_distance = 0;
        glm::vec3 center;
        const auto block_index =
            camera_index * Platform::Settings::max_directional_light_size + directional_light_index;
        directional_light_info_blocks_[block_index].direction = glm::vec4(light_dir, 0.0f);
        directional_light_info_blocks_[block_index].diffuse =
            glm::vec4(dlc->diffuse * dlc->diffuse_brightness, dlc->cast_shadow);
        directional_light_info_blocks_[block_index].specular = glm::vec4(0.0f);
        for (int split = 0; split < 4; split++) {
          float split_start = 0;
          float split_end = render_settings.max_shadow_distance;
          if (split != 0)
            split_start = render_settings.max_shadow_distance * render_settings.shadow_cascade_split[split - 1];
          if (split != 4 - 1)
            split_end = render_settings.max_shadow_distance * render_settings.shadow_cascade_split[split];
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
          if (render_settings.stable_fit) {
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
              Ray::ClosestPointOnLine(world_bound.Center(), camera_frustum_center, camera_frustum_center + light_dir);
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
#pragma endregion

  const auto main_camera = target_scene->main_camera.Get<Camera>();
  GlobalTransform main_camera_global_transform{};
  if (main_camera) {
    if (const auto main_camera_owner = main_camera->GetOwner(); target_scene->IsEntityValid(main_camera_owner)) {
      main_camera_global_transform = target_scene->GetDataComponent<GlobalTransform>(main_camera_owner);
    }
  }
  const glm::vec3 main_camera_position = main_camera_global_transform.GetPosition();
  const std::vector<Entity>* point_light_entities = target_scene->UnsafeGetPrivateComponentOwnersList<PointLight>();
  render_info_block.point_light_size = 0;
  if (point_light_entities && !point_light_entities->empty()) {
    point_light_info_blocks_.resize(point_light_entities->size());
    std::multimap<float, size_t> sorted_point_light_indices;
    for (int i = 0; i < point_light_entities->size(); i++) {
      Entity light_entity = point_light_entities->at(i);
      if (!target_scene->IsEntityEnabled(light_entity))
        continue;
      const auto plc = target_scene->GetOrSetPrivateComponent<PointLight>(light_entity).lock();
      if (!plc->IsEnabled())
        continue;
      glm::vec3 position = target_scene->GetDataComponent<GlobalTransform>(light_entity).value[3];
      point_light_info_blocks_[render_info_block.point_light_size].position = glm::vec4(position, 0);
      point_light_info_blocks_[render_info_block.point_light_size].constant_linear_quad_far_plane.x = plc->constant;
      point_light_info_blocks_[render_info_block.point_light_size].constant_linear_quad_far_plane.y = plc->linear;
      point_light_info_blocks_[render_info_block.point_light_size].constant_linear_quad_far_plane.z = plc->quadratic;
      point_light_info_blocks_[render_info_block.point_light_size].diffuse =
          glm::vec4(plc->diffuse * plc->diffuse_brightness, plc->cast_shadow);
      point_light_info_blocks_[render_info_block.point_light_size].specular = glm::vec4(0);
      point_light_info_blocks_[render_info_block.point_light_size].constant_linear_quad_far_plane.w =
          plc->GetFarPlane();

      glm::mat4 shadow_proj =
          glm::perspective(glm::radians(90.0f), 1.0f, plc->shadow_distance / 1000.f, plc->shadow_distance);
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
    Lighting::AllocateAtlas(render_info_block.point_light_size, Platform::Settings::point_light_shadow_map_resolution,
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

  render_info_block.spot_light_size = 0;
  const std::vector<Entity>* spot_light_entities = target_scene->UnsafeGetPrivateComponentOwnersList<SpotLight>();
  if (spot_light_entities && !spot_light_entities->empty()) {
    spot_light_info_blocks_.resize(spot_light_entities->size());
    std::multimap<float, size_t> sorted_spot_light_indices;
    for (auto light_entity : *spot_light_entities) {
      if (!target_scene->IsEntityEnabled(light_entity))
        continue;
      const auto slc = target_scene->GetOrSetPrivateComponent<SpotLight>(light_entity).lock();
      if (!slc->IsEnabled())
        continue;
      auto ltw = target_scene->GetDataComponent<GlobalTransform>(light_entity);
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

      glm::mat4 shadow_proj = glm::perspective(glm::radians(slc->outer_degrees * 2.0f), 1.0f,
                                               slc->shadow_distance / 1000.f, slc->shadow_distance);
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
    Lighting::AllocateAtlas(render_info_block.spot_light_size, Platform::Settings::spot_light_shadow_map_resolution,
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

void RenderInstanceStorage::CollectEnvironment(const std::shared_ptr<Scene>& target_scene) {
  switch (target_scene->environment.environment_type) {
    case EnvironmentType::EnvironmentalMap: {
      environment_info_block.background_color.w = 0.0f;
    } break;
    case EnvironmentType::Color: {
      environment_info_block.background_color = glm::vec4(target_scene->environment.background_color, 1.0f);
    } break;
  }
  environment_info_block.environmental_map_gamma = target_scene->environment.environment_gamma;
  environment_info_block.environmental_lighting_intensity = target_scene->environment.ambient_light_intensity;
  environment_info_block.background_intensity = target_scene->environment.background_intensity;
}

void RenderInstanceStorage::CollectEditorCameras(
    const std::shared_ptr<Scene>& target_scene,
    std::vector<std::pair<GlobalTransform, std::shared_ptr<Camera>>>& cameras) {
  if (const auto editor_layer = Application::GetLayer<EditorLayer>()) {
    for (const auto& [cameraHandle, editorCamera] : editor_layer->editor_cameras_) {
      if (editorCamera.camera || editorCamera.camera->IsEnabled()) {
        GlobalTransform scene_camera_gt;
        scene_camera_gt.SetValue(editorCamera.position, editorCamera.rotation, glm::vec3(1.0f));
        cameras.emplace_back(scene_camera_gt, editorCamera.camera);
      }
    }
  }
}

void RenderInstanceStorage::CollectCameras(const std::shared_ptr<Scene>& target_scene,
                                           std::vector<std::pair<GlobalTransform, std::shared_ptr<Camera>>>& cameras) {
  if (const std::vector<Entity>* camera_entities = target_scene->UnsafeGetPrivateComponentOwnersList<Camera>()) {
    for (const auto& i : *camera_entities) {
      if (!target_scene->IsEntityEnabled(i))
        continue;
      assert(target_scene->HasPrivateComponent<Camera>(i));
      auto camera = target_scene->GetOrSetPrivateComponent<Camera>(i).lock();
      if (!camera || !camera->IsEnabled())
        continue;
      auto camera_global_transform = target_scene->GetDataComponent<GlobalTransform>(i);
      cameras.emplace_back(camera_global_transform, camera);
    }
  }
}

RenderInstanceStorage::RenderInstanceStorage() {
  VkBufferCreateInfo buffer_create_info{};
  buffer_create_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  buffer_create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  VmaAllocationCreateInfo buffer_vma_allocation_create_info{};
  buffer_vma_allocation_create_info.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;

  buffer_create_info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;
  buffer_create_info.size = sizeof(RenderInfoBlock);
  render_info_descriptor_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  buffer_create_info.size = sizeof(EnvironmentInfoBlock);
  environment_info_descriptor_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);

  buffer_create_info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
  buffer_create_info.size = sizeof(CameraInfoBlock) * Platform::Constants::initial_camera_size;
  camera_info_descriptor_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  buffer_create_info.size = sizeof(DirectionalLightInfoBlock) * Platform::Settings::max_directional_light_size *
                            Platform::Constants::initial_camera_size;
  directional_light_info_descriptor_buffer =
      std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  buffer_create_info.size = sizeof(PointLightInfoBlock) * Platform::Settings::max_point_light_size;
  point_light_info_descriptor_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  buffer_create_info.size = sizeof(SpotLightInfoBlock) * Platform::Settings::max_spot_light_size;
  spot_light_info_descriptor_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  buffer_create_info.size =
      glm::max(static_cast<size_t>(1), sizeof(MaterialInfoBlock) * Platform::Constants::initial_material_size);
  material_info_descriptor_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  buffer_create_info.size =
      glm::max(static_cast<size_t>(1), sizeof(InstanceInfoBlock) * Platform::Constants::initial_instance_size);
  instance_info_descriptor_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  buffer_create_info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_INDIRECT_BUFFER_BIT;
  buffer_create_info.size = glm::max(static_cast<size_t>(1),
                                     sizeof(VkDrawIndexedIndirectCommand) * mesh_draw_indexed_indirect_commands.size());
  mesh_draw_indexed_indirect_commands_buffer =
      std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);

  buffer_create_info.size = glm::max(static_cast<size_t>(1), sizeof(VkDrawMeshTasksIndirectCommandEXT) *
                                                                 mesh_draw_mesh_tasks_indirect_commands.size());
  mesh_draw_mesh_tasks_indirect_commands_buffer =
      std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);

  deferred_render_instances = std::make_shared<MeshRenderInstanceCollection>();
  deferred_skinned_render_instances = std::make_shared<SkinnedMeshRenderInstanceCollection>();
  deferred_instanced_render_instances = std::make_shared<InstancedRenderInstanceCollection>();
  deferred_strands_render_instances = std::make_shared<StrandsRenderInstanceCollection>();

  forward_render_instances = std::make_shared<MeshRenderInstanceCollection>();
  forward_skinned_render_instances = std::make_shared<SkinnedMeshRenderInstanceCollection>();
  forward_instanced_render_instances = std::make_shared<InstancedRenderInstanceCollection>();
  forward_strands_render_instances = std::make_shared<StrandsRenderInstanceCollection>();

  transparent_render_instances = std::make_shared<MeshRenderInstanceCollection>();
  transparent_skinned_render_instances = std::make_shared<SkinnedMeshRenderInstanceCollection>();
  transparent_instanced_render_instances = std::make_shared<InstancedRenderInstanceCollection>();
  transparent_strands_render_instances = std::make_shared<StrandsRenderInstanceCollection>();

  external_render_instances = std::make_shared<ExternalRenderInstanceCollection>();
}

void RenderInstanceStorage::Clear() {
  total_mesh_triangles = 0;
  total_skinned_mesh_triangles = 0;
  total_instanced_mesh_triangles = 0;
  total_strands_segments = 0;

  deferred_render_instances = std::make_shared<MeshRenderInstanceCollection>();
  deferred_skinned_render_instances = std::make_shared<SkinnedMeshRenderInstanceCollection>();
  deferred_instanced_render_instances = std::make_shared<InstancedRenderInstanceCollection>();
  deferred_strands_render_instances = std::make_shared<StrandsRenderInstanceCollection>();

  forward_render_instances = std::make_shared<MeshRenderInstanceCollection>();
  forward_skinned_render_instances = std::make_shared<SkinnedMeshRenderInstanceCollection>();
  forward_instanced_render_instances = std::make_shared<InstancedRenderInstanceCollection>();
  forward_strands_render_instances = std::make_shared<StrandsRenderInstanceCollection>();

  transparent_render_instances = std::make_shared<MeshRenderInstanceCollection>();
  transparent_skinned_render_instances = std::make_shared<SkinnedMeshRenderInstanceCollection>();
  transparent_instanced_render_instances = std::make_shared<InstancedRenderInstanceCollection>();
  transparent_strands_render_instances = std::make_shared<StrandsRenderInstanceCollection>();

  external_render_instances = std::make_shared<ExternalRenderInstanceCollection>();

  instance_entity_handles_.clear();
  instance_renderer_handles_.clear();

  renderer_indices_.clear();
  camera_indices_.clear();
  material_indices_.clear();
  render_settings = {};

  camera_info_blocks_.clear();
  material_info_blocks_.clear();
  instance_info_blocks_.clear();
  directional_light_info_blocks_.clear();
  point_light_info_blocks_.clear();
  spot_light_info_blocks_.clear();
  render_info_block = {};

  cameras.clear();

  mesh_draw_indexed_indirect_commands.clear();
  mesh_draw_mesh_tasks_indirect_commands.clear();
}

void RenderInstanceStorage::Upload() const {
  if (!Platform::Initialized())
    return;
  camera_info_descriptor_buffer->UploadVector(camera_info_blocks_);
  material_info_descriptor_buffer->UploadVector(material_info_blocks_);
  instance_info_descriptor_buffer->UploadVector(instance_info_blocks_);
  render_info_descriptor_buffer->Upload(render_info_block);
  directional_light_info_descriptor_buffer->UploadVector(directional_light_info_blocks_);
  point_light_info_descriptor_buffer->UploadVector(point_light_info_blocks_);
  spot_light_info_descriptor_buffer->UploadVector(spot_light_info_blocks_);

  mesh_draw_indexed_indirect_commands_buffer->UploadVector(mesh_draw_indexed_indirect_commands);
  mesh_draw_mesh_tasks_indirect_commands_buffer->UploadVector(mesh_draw_mesh_tasks_indirect_commands);

  environment_info_descriptor_buffer->Upload(environment_info_block);
}

const std::vector<RenderInstanceStorage::MaterialInfoBlock>& RenderInstanceStorage::GetMaterialInfoBlocks() const {
  return material_info_blocks_;
}

const std::vector<RenderInstanceStorage::InstanceInfoBlock>& RenderInstanceStorage::GetInstanceInfoBlocks() const {
  return instance_info_blocks_;
}

void RenderInstanceStorage::CalculateLodFactor(const std::shared_ptr<Scene>& scene, const glm::vec3& view_position,
                                               const float max_distance) {
  if (const auto* owners = scene->UnsafeGetPrivateComponentOwnersList<LodGroup>()) {
    for (auto owner : *owners) {
      if (const auto lod_group = scene->GetOrSetPrivateComponent<LodGroup>(owner).lock();
          !lod_group->override_lod_factor) {
        auto gt = scene->GetDataComponent<GlobalTransform>(owner);
        const auto distance = glm::distance(gt.GetPosition(), view_position);
        const auto distance_factor = glm::clamp(distance / max_distance, 0.f, 1.f);
        lod_group->lod_factor = glm::clamp(distance_factor * distance_factor, 0.f, 1.f);
      }
    }
  }
}
bool RenderInstanceStorage::operator!=(const RenderInstanceStorage& other) const {
  if (render_info_block != other.render_info_block)
    return true;

  if (environment_info_block != other.environment_info_block)
    return true;

  if (instance_info_blocks_.size() != other.instance_info_blocks_.size())
    return true;
  for (uint32_t i = 0; i < instance_info_blocks_.size(); i++) {
    if (instance_info_blocks_[i] != other.instance_info_blocks_[i])
      return true;
  }

  if (directional_light_info_blocks_.size() != other.directional_light_info_blocks_.size())
    return true;
  for (uint32_t i = 0; i < directional_light_info_blocks_.size(); i++) {
    if (directional_light_info_blocks_[i] != other.directional_light_info_blocks_[i])
      return true;
  }

  if (point_light_info_blocks_.size() != other.point_light_info_blocks_.size())
    return true;
  for (uint32_t i = 0; i < point_light_info_blocks_.size(); i++) {
    if (point_light_info_blocks_[i] != other.point_light_info_blocks_[i])
      return true;
  }

  if (spot_light_info_blocks_.size() != other.spot_light_info_blocks_.size())
    return true;
  for (uint32_t i = 0; i < spot_light_info_blocks_.size(); i++) {
    if (spot_light_info_blocks_[i] != other.spot_light_info_blocks_[i])
      return true;
  }

  if (material_info_blocks_.size() != other.material_info_blocks_.size())
    return true;
  for (uint32_t i = 0; i < material_info_blocks_.size(); i++) {
    if (material_info_blocks_[i] != other.material_info_blocks_[i])
      return true;
  }

  if (external_render_instances != other.external_render_instances)
    return true;

  if (geometry_storage_version != other.geometry_storage_version)
    return true;
  if (texture_storage_version != other.texture_storage_version)
    return true;

  return false;
}

bool RenderInstanceStorage::RegisterMeshDrawCommand(const std::shared_ptr<Mesh>& mesh,
                                                    const std::shared_ptr<Material>& material,
                                                    const GlobalTransform& model, bool cast_shadow) {
  if (!material || !mesh || !mesh->meshlet_range_ || !mesh->triangle_range_)
    return false;
  if (mesh->UnsafeGetVertices().empty() || mesh->UnsafeGetTriangles().empty())
    return false;
  MaterialInfoBlock material_info_block;
  material_info_block.Apply(material);
  const auto render_instance = std::make_shared<MeshRenderInstance>();
  render_instance->command_type = RenderInstanceType::FromApi;
  render_instance->owner = Entity();
  render_instance->mesh = mesh;
  render_instance->entity_handle = 0;
  render_instance->renderer_handle = 0;
  render_instance->material = material;
  render_instance->model = model;
  render_instance->renderer_handle = 0;
  render_instance->cast_shadow = cast_shadow;
  render_instance->geometry_version = mesh->GetVersion();
  render_instance->material_version = material->GetVersion();
  render_instance->material_index = RegisterMaterial(material->GetHandle(), material_info_block);
  render_instance->line_width = material->draw_settings.line_width;
  render_instance->cull_mode = material->draw_settings.cull_mode;
  render_instance->polygon_mode = material->draw_settings.polygon_mode;
  render_instance->entity_selected = false;
  if (material->draw_settings.blending) {
    transparent_render_instances->Register(render_instance);
  } else {
    deferred_render_instances->Register(render_instance);
  }

  auto& new_mesh_task = mesh_draw_mesh_tasks_indirect_commands.emplace_back();
  new_mesh_task.groupCountX = 1;
  new_mesh_task.groupCountY = 1;
  new_mesh_task.groupCountZ = 1;

  auto& new_draw_task = mesh_draw_indexed_indirect_commands.emplace_back();
  new_draw_task.instanceCount = 1;
  new_draw_task.firstIndex = mesh->triangle_range_->offset * 3;
  new_draw_task.indexCount = static_cast<uint32_t>(mesh->triangles_.size() * 3);
  new_draw_task.vertexOffset = 0;
  new_draw_task.firstInstance = 0;

  total_mesh_triangles += mesh->triangles_.size();

  return true;
}

bool RenderInstanceStorage::RegisterMeshDrawInstancedCommand(
    const std::shared_ptr<Mesh>& mesh, const std::shared_ptr<Material>& material, const GlobalTransform& model,
    const std::shared_ptr<ParticleInfoList>& particle_info_list, const bool cast_shadow) {
  if (!material || !mesh || !mesh->meshlet_range_ || !mesh->triangle_range_)
    return false;
  if (mesh->UnsafeGetVertices().empty() || mesh->UnsafeGetTriangles().empty())
    return false;
  MaterialInfoBlock material_info_block;
  material_info_block.Apply(material);
  const auto render_instance = std::make_shared<InstancedRenderInstance>();
  render_instance->command_type = RenderInstanceType::FromApi;
  render_instance->owner = Entity();
  render_instance->mesh = mesh;
  render_instance->entity_handle = 0;
  render_instance->renderer_handle = 0;
  render_instance->material = material;
  render_instance->model = model;
  render_instance->particle_infos = particle_info_list;
  render_instance->particle_info_list_version = particle_info_list->GetVersion();
  render_instance->renderer_handle = 0;
  render_instance->cast_shadow = cast_shadow;
  render_instance->geometry_version = mesh->GetVersion();
  render_instance->material_version = material->GetVersion();
  render_instance->material_index = RegisterMaterial(material->GetHandle(), material_info_block);
  render_instance->line_width = material->draw_settings.line_width;
  render_instance->cull_mode = material->draw_settings.cull_mode;
  render_instance->polygon_mode = material->draw_settings.polygon_mode;
  render_instance->entity_selected = false;
  if (material->draw_settings.blending) {
    transparent_instanced_render_instances->Register(render_instance);
  } else {
    deferred_instanced_render_instances->Register(render_instance);
  }

  total_mesh_triangles += mesh->triangles_.size() * particle_info_list->PeekParticleInfoList().size();

  return true;
}

bool RenderInstanceStorage::RegisterRenderInstance(const std::shared_ptr<Scene>& target_scene, const Entity& entity,
                                                   const Handle& renderer_handle,
                                                   const std::shared_ptr<Material>& material) {
  if (!material)
    return false;
  const auto gt = target_scene->GetDataComponent<GlobalTransform>(entity);
  MaterialInfoBlock material_info_block;
  material_info_block.Apply(material);
  const auto render_instance = std::make_shared<ExternalRenderInstance>();
  render_instance->command_type = RenderInstanceType::Unknown;
  render_instance->owner = entity;
  render_instance->model = gt;
  render_instance->entity_handle = target_scene->GetEntityHandle(entity);
  render_instance->renderer_handle = renderer_handle;
  render_instance->material = material;
  render_instance->cast_shadow = false;
  // No need to update this render instance because we do not use this for ray tracer.
  render_instance->geometry_version = 0;
  render_instance->material_version = material->GetVersion();
  render_instance->material_index = RegisterMaterial(material->GetHandle(), material_info_block);
  render_instance->entity_selected = target_scene->IsEntityAncestorSelected(entity);
  render_instance->line_width = material->draw_settings.line_width;
  render_instance->cull_mode = material->draw_settings.cull_mode;
  render_instance->polygon_mode = material->draw_settings.polygon_mode;

  external_render_instances->Register(render_instance);

  return true;
}

int RenderInstanceStorage::RegisterMaterial(const std::shared_ptr<Material>& material) {
  if (!material)
    return -1;
  MaterialInfoBlock material_info_block;
  material_info_block.Apply(material);
  return RegisterMaterial(material->GetHandle(), material_info_block);
}

void RenderInstanceStorage::BuildFromScene(const RenderSettings& render_settings, const std::shared_ptr<Scene>& scene,
                                           Bound& world_bound) {
  this->render_settings = render_settings;
  render_info_block.Apply(this->render_settings);
  CollectEnvironment(scene);
  CollectEditorCameras(scene, cameras);
  CollectCameras(scene, cameras);
  for (const auto& camera_info : cameras) {
    CameraInfoBlock camera_info_block;
    camera_info.second->UpdateCameraInfoBlock(camera_info_block, camera_info.first);
    const auto index = RegisterCamera(camera_info.second->GetHandle(), camera_info_block);
  }
  CollectEntityRenderers(scene, world_bound);
  BuildRenderInstanceBlocks();
  CollectLights(scene, world_bound);
}

void RenderInstanceStorage::UpdateTopLevelAccelerationStructure(const std::shared_ptr<Scene>& scene) {
  // mesh_top_level_acceleration_structure = std::make_shared<TopLevelAccelerationStructure>(scene, *this);
}

bool RenderInstanceStorage::RegisterEntity(const std::shared_ptr<Scene>& target_scene, const Entity& owner,
                                           const std::shared_ptr<StrandsRenderer>& strands_renderer,
                                           glm::vec3& min_bound, glm::vec3& max_bound) {
  auto material = strands_renderer->material.Get<Material>();
  auto strands = strands_renderer->strands.Get<Strands>();
  if (!strands_renderer->IsEnabled() || !material || !strands || !strands->strand_meshlet_range_ ||
      !strands->segment_range_)
    return false;
  auto gt = target_scene->GetDataComponent<GlobalTransform>(owner);
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
  material_info_block.Apply(material);

  const auto render_instance = std::make_shared<StrandsRenderInstance>();
  render_instance->command_type = RenderInstanceType::FromRenderer;
  render_instance->owner = owner;
  render_instance->entity_handle = target_scene->GetEntityHandle(owner);
  render_instance->renderer_handle = strands_renderer->GetHandle();
  render_instance->model = gt;
  render_instance->strands = strands;
  render_instance->material = material;
  render_instance->cast_shadow = strands_renderer->cast_shadow;
  render_instance->geometry_version = strands->GetVersion();
  render_instance->material_version = material->GetVersion();
  render_instance->material_index = RegisterMaterial(material->GetHandle(), material_info_block);
  render_instance->entity_selected = target_scene->IsEntityAncestorSelected(owner);
  render_instance->line_width = material->draw_settings.line_width;
  render_instance->cull_mode = material->draw_settings.cull_mode;
  render_instance->polygon_mode = material->draw_settings.polygon_mode;

  if (material->draw_settings.blending) {
    transparent_strands_render_instances->Register(render_instance);
  } else {
    deferred_strands_render_instances->Register(render_instance);
  }

  total_strands_segments += strands->segments_.size();
  return true;
}
bool RenderInstanceStorage::RegisterEntity(const std::shared_ptr<Scene>& target_scene, const Entity& owner,
                                           const std::shared_ptr<MeshRenderer>& mesh_renderer, glm::vec3& min_bound,
                                           glm::vec3& max_bound) {
  auto material = mesh_renderer->material.Get<Material>();
  auto mesh = mesh_renderer->mesh.Get<Mesh>();
  if (!mesh_renderer->IsEnabled() || !material || !mesh || !mesh->meshlet_range_ || !mesh->triangle_range_)
    return false;
  if (mesh->UnsafeGetVertices().empty() || mesh->UnsafeGetTriangles().empty())
    return false;

  auto gt = target_scene->GetDataComponent<GlobalTransform>(owner);
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
  material_info_block.Apply(material);
  const auto render_instance = std::make_shared<MeshRenderInstance>();
  render_instance->command_type = RenderInstanceType::FromRenderer;
  render_instance->owner = owner;
  render_instance->mesh = mesh;
  render_instance->material = material;
  render_instance->model = gt;
  render_instance->entity_handle = target_scene->GetEntityHandle(owner);
  render_instance->renderer_handle = mesh_renderer->GetHandle();
  render_instance->cast_shadow = mesh_renderer->cast_shadow;
  render_instance->geometry_version = mesh->GetVersion();
  render_instance->material_version = material->GetVersion();
  render_instance->material_index = RegisterMaterial(material->GetHandle(), material_info_block);
  render_instance->line_width = material->draw_settings.line_width;
  render_instance->cull_mode = material->draw_settings.cull_mode;
  render_instance->polygon_mode = material->draw_settings.polygon_mode;
  render_instance->entity_selected = target_scene->IsEntityAncestorSelected(owner);
  if (material->draw_settings.blending) {
    transparent_render_instances->Register(render_instance);
  } else {
    deferred_render_instances->Register(render_instance);
  }
  if (const auto render_layer = Application::GetLayer<RenderLayer>()) {
    const uint32_t task_work_group_invocations =
        Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxPreferredTaskWorkGroupInvocations;
    auto& new_mesh_task = mesh_draw_mesh_tasks_indirect_commands.emplace_back();
    const uint32_t count =
        (mesh->meshlet_range_->range + task_work_group_invocations - 1) / task_work_group_invocations;
    new_mesh_task.groupCountX = count;
    new_mesh_task.groupCountY = 1;
    new_mesh_task.groupCountZ = 1;

    auto& new_draw_task = mesh_draw_indexed_indirect_commands.emplace_back();
    new_draw_task.instanceCount = 1;
    new_draw_task.firstIndex = mesh->triangle_range_->offset * 3;
    new_draw_task.indexCount = static_cast<uint32_t>(mesh->triangles_.size() * 3);
    new_draw_task.vertexOffset = 0;
    new_draw_task.firstInstance = 0;
  }
  total_mesh_triangles += mesh->triangles_.size();
  return true;
}

bool RenderInstanceStorage::RegisterEntity(const std::shared_ptr<Scene>& target_scene, const Entity& owner,
                                           const std::shared_ptr<SkinnedMeshRenderer>& skinned_mesh_renderer,
                                           glm::vec3& min_bound, glm::vec3& max_bound) {
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
    gt = target_scene->GetDataComponent<GlobalTransform>(owner);
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
  material_info_block.Apply(material);
  const auto render_instance = std::make_shared<SkinnedMeshRenderInstance>();
  render_instance->command_type = RenderInstanceType::FromRenderer;
  render_instance->owner = owner;
  render_instance->entity_handle = target_scene->GetEntityHandle(owner);
  render_instance->renderer_handle = skinned_mesh_renderer->GetHandle();
  render_instance->model = gt;
  render_instance->skinned_mesh = skinned_mesh;
  render_instance->material = material;
  render_instance->cast_shadow = skinned_mesh_renderer->cast_shadow;
  render_instance->bone_matrices = skinned_mesh_renderer->bone_matrices;
  render_instance->geometry_version = skinned_mesh->GetVersion();
  render_instance->material_version = material->GetVersion();
  render_instance->material_index = RegisterMaterial(material->GetHandle(), material_info_block);
  render_instance->bone_matrices_version = skinned_mesh_renderer->bone_matrices->GetVersion();
  render_instance->entity_selected = target_scene->IsEntityAncestorSelected(owner);
  render_instance->line_width = material->draw_settings.line_width;
  render_instance->cull_mode = material->draw_settings.cull_mode;
  render_instance->polygon_mode = material->draw_settings.polygon_mode;

  if (material->draw_settings.blending) {
    transparent_skinned_render_instances->Register(render_instance);
  } else {
    deferred_skinned_render_instances->Register(render_instance);
  }

  total_skinned_mesh_triangles += skinned_mesh->skinned_triangles_.size();
  return true;
}

bool RenderInstanceStorage::RegisterEntity(const std::shared_ptr<Scene>& target_scene, const Entity& owner,
                                           const std::shared_ptr<Particles>& particles, glm::vec3& min_bound,
                                           glm::vec3& max_bound) {
  auto material = particles->material.Get<Material>();
  auto mesh = particles->mesh.Get<Mesh>();
  auto particle_info_list = particles->particle_info_list.Get<ParticleInfoList>();
  if (!particles->IsEnabled() || !material || !mesh || !mesh->meshlet_range_ || !mesh->triangle_range_ ||
      !particle_info_list)
    return false;
  if (particle_info_list->PeekParticleInfoList().empty())
    return false;
  auto gt = target_scene->GetDataComponent<GlobalTransform>(owner);
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
  material_info_block.Apply(material);

  const auto render_instance = std::make_shared<InstancedRenderInstance>();
  render_instance->command_type = RenderInstanceType::FromRenderer;
  render_instance->model = gt;
  render_instance->owner = owner;
  render_instance->entity_handle = target_scene->GetEntityHandle(owner);
  render_instance->renderer_handle = particles->GetHandle();
  render_instance->mesh = mesh;
  render_instance->material = material;
  render_instance->cast_shadow = particles->cast_shadow;
  render_instance->particle_infos = particle_info_list;
  render_instance->geometry_version = mesh->GetVersion();
  render_instance->material_version = material->GetVersion();
  render_instance->material_index = RegisterMaterial(material->GetHandle(), material_info_block);
  render_instance->particle_info_list_version = particle_info_list->GetVersion();
  render_instance->entity_selected = target_scene->IsEntityAncestorSelected(owner);
  render_instance->line_width = material->draw_settings.line_width;
  render_instance->cull_mode = material->draw_settings.cull_mode;
  render_instance->polygon_mode = material->draw_settings.polygon_mode;

  if (material->draw_settings.blending) {
    transparent_instanced_render_instances->Register(render_instance);
  } else {
    deferred_instanced_render_instances->Register(render_instance);
  }

  total_instanced_mesh_triangles += mesh->triangles_.size() * particle_info_list->PeekParticleInfoList().size();
  return true;
}

int RenderInstanceStorage::RegisterMaterial(const Handle& handle, const MaterialInfoBlock& material_info_block) {
  const auto search = material_indices_.find(handle);
  if (search == material_indices_.end()) {
    const int index = material_info_blocks_.size();
    material_indices_[handle] = index;
    material_info_blocks_.emplace_back(material_info_block);
    return index;
  }
  return search->second;
}

int RenderInstanceStorage::RegisterCamera(const Handle& handle, const CameraInfoBlock& camera_info_block) {
  const auto search = camera_indices_.find(handle);
  if (search == camera_indices_.end()) {
    const int index = camera_info_blocks_.size();
    camera_indices_[handle] = index;
    camera_info_blocks_.emplace_back(camera_info_block);
    return index;
  }
  return search->second;
}

int RenderInstanceStorage::GetMaterialIndex(const Handle& material_handle) {
  const auto search = material_indices_.find(material_handle);
  if (search == material_indices_.end()) {
    throw std::runtime_error("Unable to find material!");
  }
  return search->second;
}

int RenderInstanceStorage::GetRenderInstanceIndex(const Handle& renderer_handle) {
  const auto search = renderer_indices_.find(renderer_handle);
  if (search == renderer_indices_.end()) {
    throw std::runtime_error("Unable to find renderer!");
  }
  return search->second;
}

int RenderInstanceStorage::GetCameraIndex(const Handle& camera_handle) {
  const auto search = camera_indices_.find(camera_handle);
  if (search == camera_indices_.end()) {
    throw std::runtime_error("Unable to find camera!");
  }
  return search->second;
}

Handle RenderInstanceStorage::GetInstanceEntityHandle(const int render_instance_index) {
  const auto search = instance_entity_handles_.find(render_instance_index);
  if (search == instance_entity_handles_.end()) {
    return 0;
  }
  return search->second;
}

Handle RenderInstanceStorage::GetInstanceRendererHandle(const int render_instance_index) {
  const auto search = instance_renderer_handles_.find(render_instance_index);
  if (search == instance_renderer_handles_.end()) {
    return 0;
  }
  return search->second;
}
