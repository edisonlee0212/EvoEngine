#include "RenderInstances.hpp"
#include "LODGroup.hpp"

using namespace evo_engine;

bool MeshRenderInstance::operator!=(const MeshRenderInstance& other) const {
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
  if (mesh_version != other.mesh_version)
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

bool SkinnedMeshRenderInstance::operator!=(const SkinnedMeshRenderInstance& other) const {
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
  if (skinned_mesh_version != other.skinned_mesh_version)
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

bool InstancedRenderInstance::operator!=(const InstancedRenderInstance& other) const {
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
  if (mesh_version != other.mesh_version)
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

bool StrandsRenderInstance::operator!=(const StrandsRenderInstance& other) const {
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

  if (strands_version != other.strands_version)
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

bool MeshRenderInstanceCollection::operator!=(const MeshRenderInstanceCollection& other) const {
  if (render_commands.size() != other.render_commands.size())
    return true;
  for (uint32_t i = 0; i < render_commands.size(); i++) {
    if (render_commands[i] != other.render_commands[i])
      return true;
  }
  return false;
}

bool SkinnedMeshRenderInstanceCollection::operator!=(const SkinnedMeshRenderInstanceCollection& other) const {
  if (render_commands.size() != other.render_commands.size())
    return true;
  for (uint32_t i = 0; i < render_commands.size(); i++) {
    if (render_commands[i] != other.render_commands[i])
      return true;
  }
  return false;
}

bool StrandsRenderInstanceCollection::operator!=(const StrandsRenderInstanceCollection& other) const {
  if (render_commands.size() != other.render_commands.size())
    return true;
  for (uint32_t i = 0; i < render_commands.size(); i++) {
    if (render_commands[i] != other.render_commands[i])
      return true;
  }
  return false;
}

bool InstancedRenderInstanceCollection::operator!=(const InstancedRenderInstanceCollection& other) const {
  if (render_commands.size() != other.render_commands.size())
    return true;
  for (uint32_t i = 0; i < render_commands.size(); i++) {
    if (render_commands[i] != other.render_commands[i])
      return true;
  }
  return false;
}

void RenderInstances::Collect(Bound& world_bound) {
  auto& min_bound = world_bound.min;
  auto& max_bound = world_bound.max;
  min_bound = glm::vec3(FLT_MAX);
  max_bound = glm::vec3(-FLT_MAX);

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
              if (TryRegisterRenderer(owner, mesh_renderer, min_bound, max_bound)) {
                has_render_instance = true;
              }
            }
          } else if (const auto skinned_mesh_renderer = renderer.Get<SkinnedMeshRenderer>()) {
            lod_group_renderers.insert(skinned_mesh_renderer->GetHandle());
            if (render_current_level && target_scene->IsEntityEnabled(owner) &&
                target_scene->IsEntityEnabled(skinned_mesh_renderer->GetOwner()) &&
                skinned_mesh_renderer->IsEnabled()) {
              if (TryRegisterRenderer(owner, skinned_mesh_renderer, min_bound, max_bound)) {
                has_render_instance = true;
              }
            }
          } else if (const auto particles = renderer.Get<Particles>()) {
            lod_group_renderers.insert(particles->GetHandle());
            if (render_current_level && target_scene->IsEntityEnabled(owner) &&
                target_scene->IsEntityEnabled(particles->GetOwner()) && particles->IsEnabled()) {
              if (TryRegisterRenderer(owner, particles, min_bound, max_bound)) {
                has_render_instance = true;
              }
            }
          } else if (const auto strands_renderer = renderer.Get<StrandsRenderer>()) {
            lod_group_renderers.insert(strands_renderer->GetHandle());
            if (render_current_level && target_scene->IsEntityEnabled(owner) &&
                target_scene->IsEntityEnabled(strands_renderer->GetOwner()) && strands_renderer->IsEnabled()) {
              if (TryRegisterRenderer(owner, strands_renderer, min_bound, max_bound)) {
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
      if (TryRegisterRenderer(owner, mesh_renderer, min_bound, max_bound)) {
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
      if (TryRegisterRenderer(owner, skinned_mesh_renderer, min_bound, max_bound)) {
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
      if (TryRegisterRenderer(owner, particles, min_bound, max_bound)) {
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
      if (TryRegisterRenderer(owner, strands_renderer, min_bound, max_bound)) {
        has_render_instance = true;
      }
    }
  }
  if (!has_render_instance) {
    min_bound = max_bound = glm::vec3(0.0f);
  }
}

RenderInstances::RenderInstances() {
  VkBufferCreateInfo buffer_create_info{};
  buffer_create_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  buffer_create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  VmaAllocationCreateInfo buffer_vma_allocation_create_info{};
  buffer_vma_allocation_create_info.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
  buffer_create_info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
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
}

bool RenderInstances::MeshInstancesUpdated(const RenderInstances& other) const {
  if (deferred_render_instances.render_commands.size() != other.deferred_render_instances.render_commands.size())
    return true;
  if (transparent_render_instances.render_commands.size() != other.transparent_render_instances.render_commands.size())
    return true;
  if (deferred_render_instances != other.deferred_render_instances)
    return true;
  if (transparent_render_instances != other.transparent_render_instances)
    return true;
  return false;
}

bool RenderInstances::SkinnedMeshInstancesUpdated(const RenderInstances& other) const {
  if (deferred_skinned_render_instances.render_commands.size() !=
      other.deferred_skinned_render_instances.render_commands.size())
    return true;
  if (transparent_skinned_render_instances.render_commands.size() !=
      other.transparent_skinned_render_instances.render_commands.size())
    return true;
  if (deferred_skinned_render_instances != other.deferred_skinned_render_instances)
    return true;
  if (transparent_skinned_render_instances != other.transparent_skinned_render_instances)
    return true;
  return false;
}

bool RenderInstances::InstancedMeshInstancesUpdated(const RenderInstances& other) const {
  if (deferred_instanced_render_instances.render_commands.size() !=
      other.deferred_instanced_render_instances.render_commands.size())
    return true;
  if (transparent_instanced_render_instances.render_commands.size() !=
      other.transparent_instanced_render_instances.render_commands.size())
    return true;
  if (deferred_instanced_render_instances != other.deferred_instanced_render_instances)
    return true;
  if (transparent_instanced_render_instances != other.transparent_instanced_render_instances)
    return true;
  return false;
}

bool RenderInstances::StrandsInstancesUpdated(const RenderInstances& other) const {
  if (deferred_strands_render_instances.render_commands.size() !=
      other.deferred_strands_render_instances.render_commands.size())
    return true;
  if (transparent_strands_render_instances.render_commands.size() !=
      other.transparent_strands_render_instances.render_commands.size())
    return true;
  if (deferred_strands_render_instances != other.deferred_strands_render_instances)
    return true;
  if (transparent_strands_render_instances != other.transparent_strands_render_instances)
    return true;
  return false;
}

void RenderInstances::Clear() {
  total_mesh_triangles = 0;
  total_skinned_mesh_triangles = 0;
  total_instanced_mesh_triangles = 0;
  total_strands_segments = 0;

  deferred_render_instances.render_commands.clear();
  deferred_skinned_render_instances.render_commands.clear();
  deferred_instanced_render_instances.render_commands.clear();
  deferred_strands_render_instances.render_commands.clear();
  transparent_render_instances.render_commands.clear();
  transparent_skinned_render_instances.render_commands.clear();
  transparent_instanced_render_instances.render_commands.clear();
  transparent_strands_render_instances.render_commands.clear();

  material_indices_.clear();
  instance_indices_.clear();
  instance_handles_.clear();

  material_info_blocks_.clear();
  instance_info_blocks_.clear();

  target_scene.reset();

  mesh_draw_indexed_indirect_commands.clear();
  mesh_draw_mesh_tasks_indirect_commands.clear();
}

void RenderInstances::Upload() const {
  material_info_descriptor_buffer->UploadVector(material_info_blocks_);
  instance_info_descriptor_buffer->UploadVector(instance_info_blocks_);

  mesh_draw_indexed_indirect_commands_buffer->UploadVector(mesh_draw_indexed_indirect_commands);
  mesh_draw_mesh_tasks_indirect_commands_buffer->UploadVector(mesh_draw_mesh_tasks_indirect_commands);
}

const std::vector<MaterialInfoBlock>& RenderInstances::GetMaterialInfoBlocks() const {
  return material_info_blocks_;
}

const std::vector<InstanceInfoBlock>& RenderInstances::GetInstanceInfoBlocks() const {
  return instance_info_blocks_;
}

void RenderInstances::CalculateLodFactor(const std::shared_ptr<Scene>& scene, const glm::vec3& view_position,
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

bool RenderInstances::UpdateRenderInstances(const std::shared_ptr<Scene>& scene, Bound& world_bound) {
  const RenderInstances old_render_instances = *this;
  Clear();
  target_scene = scene;
  Collect(world_bound);

  const bool mesh_updated = MeshInstancesUpdated(old_render_instances);
  const bool skinned_mesh_updated = SkinnedMeshInstancesUpdated(old_render_instances);
  const bool instanced_mesh_updated = InstancedMeshInstancesUpdated(old_render_instances);
  const bool strands_updated = StrandsInstancesUpdated(old_render_instances);

  if (mesh_updated || skinned_mesh_updated || instanced_mesh_updated || strands_updated) {
    if (mesh_updated) {
      if (Platform::Constants::support_ray_tracing && Platform::Settings::use_ray_tracing) {
        UpdateTopLevelAccelerationStructure(scene);
#ifndef NDEBUG
        EVOENGINE_LOG("TLAS updated!");
#endif
      }
    }
    return true;
  }
  return false;
}

void RenderInstances::UpdateTopLevelAccelerationStructure(const std::shared_ptr<Scene>& scene) {
  mesh_top_level_acceleration_structure =
      std::make_shared<TopLevelAccelerationStructure>(scene, deferred_render_instances.render_commands);
}

bool RenderInstances::TryRegisterRenderer(const Entity& owner, const std::shared_ptr<StrandsRenderer>& strands_renderer,
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
  material->UpdateMaterialInfoBlock(material_info_block);
  auto material_index = RegisterMaterialIndex(material->GetHandle(), material_info_block);
  InstanceInfoBlock instance_info_block;
  instance_info_block.model = gt;
  instance_info_block.material_index = material_index;
  instance_info_block.entity_selected = target_scene->IsEntityAncestorSelected(owner) ? 1 : 0;
  instance_info_block.meshlet_size = strands->strand_meshlet_range_->range;
  auto entity_handle = target_scene->GetEntityHandle(owner);
  auto instance_index = RegisterInstanceIndex(entity_handle, instance_info_block);

  StrandsRenderInstance render_instance;
  render_instance.command_type = RenderCommandType::FromRenderer;
  render_instance.owner = owner;
  render_instance.model = gt;
  render_instance.strands = strands;
  render_instance.material = material;
  render_instance.cast_shadow = strands_renderer->cast_shadow;
  render_instance.strand_meshlet_size = strands->strand_meshlet_range_->range;
  render_instance.strands_version = strands->GetVersion();
  render_instance.material_version = material->GetVersion();

  render_instance.instance_index = instance_index;

  render_instance.line_width = material->draw_settings.line_width;
  render_instance.cull_mode = material->draw_settings.cull_mode;
  render_instance.polygon_mode = material->draw_settings.polygon_mode;

  if (material->draw_settings.blending) {
    transparent_strands_render_instances.render_commands.push_back(render_instance);
  } else {
    deferred_strands_render_instances.render_commands.push_back(render_instance);
  }

  total_strands_segments += strands->segments_.size();
  return true;
}
bool RenderInstances::TryRegisterRenderer(const Entity& owner, const std::shared_ptr<MeshRenderer>& mesh_renderer,
                                          glm::vec3& min_bound, glm::vec3& max_bound) {
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
  material->UpdateMaterialInfoBlock(material_info_block);
  auto material_index = RegisterMaterialIndex(material->GetHandle(), material_info_block);
  InstanceInfoBlock instance_info_block;
  instance_info_block.model = gt;
  instance_info_block.material_index = material_index;
  instance_info_block.triangle_offset = mesh->GetTriangleRange()->offset;
  instance_info_block.entity_selected = target_scene->IsEntityAncestorSelected(owner) ? 1 : 0;

  instance_info_block.meshlet_index_offset = mesh->meshlet_range_->offset;
  instance_info_block.meshlet_size = mesh->meshlet_range_->range;

  auto entity_handle = target_scene->GetEntityHandle(owner);
  auto instance_index = RegisterInstanceIndex(entity_handle, instance_info_block);
  MeshRenderInstance render_instance;
  render_instance.command_type = RenderCommandType::FromRenderer;
  render_instance.owner = owner;
  render_instance.mesh = mesh;
  render_instance.material = material;
  render_instance.model = gt;

  render_instance.cast_shadow = mesh_renderer->cast_shadow;
  render_instance.meshlet_size = mesh->meshlet_range_->range;
  render_instance.instance_index = instance_index;
  render_instance.mesh_version = mesh->GetVersion();
  render_instance.material_version = material->GetVersion();

  render_instance.line_width = material->draw_settings.line_width;
  render_instance.cull_mode = material->draw_settings.cull_mode;
  render_instance.polygon_mode = material->draw_settings.polygon_mode;

  if (material->draw_settings.blending) {
    transparent_render_instances.render_commands.push_back(render_instance);
  } else {
    deferred_render_instances.render_commands.push_back(render_instance);
  }
  const uint32_t task_work_group_invocations =
      Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxPreferredTaskWorkGroupInvocations;
  auto& new_mesh_task = mesh_draw_mesh_tasks_indirect_commands.emplace_back();
  const uint32_t count = (render_instance.meshlet_size + task_work_group_invocations - 1) / task_work_group_invocations;
  new_mesh_task.groupCountX = count;
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

bool RenderInstances::TryRegisterRenderer(const Entity& owner,
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
  material->UpdateMaterialInfoBlock(material_info_block);
  auto material_index = RegisterMaterialIndex(material->GetHandle(), material_info_block);
  InstanceInfoBlock instance_info_block;
  instance_info_block.model = gt;
  instance_info_block.material_index = material_index;
  instance_info_block.entity_selected = target_scene->IsEntityAncestorSelected(owner) ? 1 : 0;
  instance_info_block.meshlet_size = skinned_mesh->skinned_meshlet_range_->range;
  auto entity_handle = target_scene->GetEntityHandle(owner);
  auto instance_index = RegisterInstanceIndex(entity_handle, instance_info_block);

  SkinnedMeshRenderInstance render_instance;
  render_instance.command_type = RenderCommandType::FromRenderer;
  render_instance.owner = owner;
  render_instance.model = gt;
  render_instance.skinned_mesh = skinned_mesh;
  render_instance.material = material;
  render_instance.cast_shadow = skinned_mesh_renderer->cast_shadow;
  render_instance.bone_matrices = skinned_mesh_renderer->bone_matrices;
  render_instance.skinned_meshlet_size = skinned_mesh->skinned_meshlet_range_->range;
  render_instance.instance_index = instance_index;
  render_instance.skinned_mesh_version = skinned_mesh->GetVersion();
  render_instance.material_version = material->GetVersion();
  render_instance.bone_matrices_version = skinned_mesh_renderer->bone_matrices->GetVersion();

  render_instance.line_width = material->draw_settings.line_width;
  render_instance.cull_mode = material->draw_settings.cull_mode;
  render_instance.polygon_mode = material->draw_settings.polygon_mode;

  if (material->draw_settings.blending) {
    transparent_skinned_render_instances.render_commands.push_back(render_instance);
  } else {
    deferred_skinned_render_instances.render_commands.push_back(render_instance);
  }

  total_skinned_mesh_triangles += skinned_mesh->skinned_triangles_.size();
  return true;
}

bool RenderInstances::TryRegisterRenderer(const Entity& owner, const std::shared_ptr<Particles>& particles,
                                          glm::vec3& min_bound, glm::vec3& max_bound) {
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
  material->UpdateMaterialInfoBlock(material_info_block);
  auto material_index = RegisterMaterialIndex(material->GetHandle(), material_info_block);
  InstanceInfoBlock instance_info_block;
  instance_info_block.model = gt;
  instance_info_block.material_index = material_index;
  instance_info_block.entity_selected = target_scene->IsEntityAncestorSelected(owner) ? 1 : 0;
  instance_info_block.meshlet_size = mesh->meshlet_range_->range;
  auto entity_handle = target_scene->GetEntityHandle(owner);
  auto instance_index = RegisterInstanceIndex(entity_handle, instance_info_block);

  InstancedRenderInstance render_instance;
  render_instance.command_type = RenderCommandType::FromRenderer;
  render_instance.model = gt;
  render_instance.owner = owner;
  render_instance.mesh = mesh;
  render_instance.material = material;
  render_instance.cast_shadow = particles->cast_shadow;
  render_instance.particle_infos = particle_info_list;
  render_instance.meshlet_size = mesh->meshlet_range_->range;
  render_instance.mesh_version = mesh->GetVersion();
  render_instance.material_version = material->GetVersion();
  render_instance.particle_info_list_version = particle_info_list->GetVersion();

  render_instance.instance_index = instance_index;

  render_instance.line_width = material->draw_settings.line_width;
  render_instance.cull_mode = material->draw_settings.cull_mode;
  render_instance.polygon_mode = material->draw_settings.polygon_mode;

  if (material->draw_settings.blending) {
    transparent_instanced_render_instances.render_commands.push_back(render_instance);
  } else {
    deferred_instanced_render_instances.render_commands.push_back(render_instance);
  }

  total_instanced_mesh_triangles += mesh->triangles_.size() * particle_info_list->PeekParticleInfoList().size();
  return true;
}

uint32_t RenderInstances::RegisterMaterialIndex(const Handle& handle, const MaterialInfoBlock& material_info_block) {
  const auto search = material_indices_.find(handle);
  if (search == material_indices_.end()) {
    const uint32_t index = material_info_blocks_.size();
    material_indices_[handle] = index;
    material_info_blocks_.emplace_back(material_info_block);
    return index;
  }
  return search->second;
}

uint32_t RenderInstances::RegisterInstanceIndex(const Handle& handle, const InstanceInfoBlock& instance_info_block) {
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

uint32_t RenderInstances::GetMaterialIndex(const Handle& handle) {
  const auto search = material_indices_.find(handle);
  if (search == material_indices_.end()) {
    throw std::runtime_error("Unable to find material!");
  }
  return search->second;
}

uint32_t RenderInstances::GetInstanceIndex(const Handle& handle) {
  const auto search = instance_indices_.find(handle);
  if (search == instance_indices_.end()) {
    throw std::runtime_error("Unable to find instance!");
  }
  return search->second;
}

Handle RenderInstances::GetInstanceHandle(uint32_t index) {
  const auto search = instance_handles_.find(index);
  if (search == instance_handles_.end()) {
    return 0;
  }
  return search->second;
}