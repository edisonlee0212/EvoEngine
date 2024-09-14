#include "RenderInstances.hpp"
#include "LODGroup.hpp"

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

void RenderInstances::Clear() {
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
}

RenderInstances::RenderInstances(const uint32_t max_frame_in_flight) {
  VkBufferCreateInfo buffer_create_info{};
  buffer_create_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  buffer_create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  VmaAllocationCreateInfo buffer_vma_allocation_create_info{};
  buffer_vma_allocation_create_info.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
  for (size_t i = 0; i < max_frame_in_flight; i++) {
    buffer_create_info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
    buffer_create_info.size = sizeof(MaterialInfoBlock) * Platform::Constants::initial_material_size;
    material_info_descriptor_buffers.emplace_back(
        std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info));
    buffer_create_info.size = sizeof(InstanceInfoBlock) * Platform::Constants::initial_instance_size;
    instance_info_descriptor_buffers.emplace_back(
        std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info));
  }
}

void RenderInstances::Upload(const uint32_t current_frame_index) const {
  material_info_descriptor_buffers[current_frame_index]->UploadVector(material_info_blocks_);
  instance_info_descriptor_buffers[current_frame_index]->UploadVector(instance_info_blocks_);
}

const std::vector<MaterialInfoBlock>& RenderInstances::GetMaterialInfoBlocks() const {
  return material_info_blocks_;
}

const std::vector<InstanceInfoBlock>& RenderInstances::GetInstanceInfoBlocks() const {
  return instance_info_blocks_;
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
  render_instance.m_owner = owner;
  render_instance.m_strands = strands;
  render_instance.cast_shadow = strands_renderer->cast_shadow;
  render_instance.strand_meshlet_size = strands->strand_meshlet_range_->range;

  render_instance.instance_index = instance_index;

  render_instance.line_width = material->draw_settings.line_width;
  render_instance.cull_mode = material->draw_settings.cull_mode;
  render_instance.polygon_mode = material->draw_settings.polygon_mode;

  if (material->draw_settings.blending) {
    transparent_strands_render_instances.render_commands.push_back(render_instance);
  } else {
    deferred_strands_render_instances.render_commands.push_back(render_instance);
  }
  return true;
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
  Clear();
  target_scene = scene;
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
  return has_render_instance;
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
  instance_info_block.entity_selected = target_scene->IsEntityAncestorSelected(owner) ? 1 : 0;

  instance_info_block.meshlet_index_offset = mesh->meshlet_range_->offset;
  instance_info_block.meshlet_size = mesh->meshlet_range_->range;

  auto entity_handle = target_scene->GetEntityHandle(owner);
  auto instance_index = RegisterInstanceIndex(entity_handle, instance_info_block);
  RenderInstance render_instance;
  render_instance.command_type = RenderCommandType::FromRenderer;
  render_instance.owner = owner;
  render_instance.mesh = mesh;
  render_instance.cast_shadow = mesh_renderer->cast_shadow;
  render_instance.meshlet_size = mesh->meshlet_range_->range;
  render_instance.instance_index = instance_index;

  render_instance.line_width = material->draw_settings.line_width;
  render_instance.cull_mode = material->draw_settings.cull_mode;
  render_instance.polygon_mode = material->draw_settings.polygon_mode;

  if (material->draw_settings.blending) {
    transparent_render_instances.render_commands.push_back(render_instance);
  } else {
    deferred_render_instances.render_commands.push_back(render_instance);
  }
  /*
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

  total_mesh_triangles_ += mesh->triangles_.size();*/
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

  if (material->draw_settings.blending) {
    transparent_skinned_render_instances.render_commands.push_back(render_instance);
  } else {
    deferred_skinned_render_instances.render_commands.push_back(render_instance);
  }
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
  render_instance.owner = owner;
  render_instance.mesh = mesh;
  render_instance.cast_shadow = particles->cast_shadow;
  render_instance.particle_infos = particle_info_list;
  render_instance.meshlet_size = mesh->meshlet_range_->range;

  render_instance.instance_index = instance_index;

  render_instance.line_width = material->draw_settings.line_width;
  render_instance.cull_mode = material->draw_settings.cull_mode;
  render_instance.polygon_mode = material->draw_settings.polygon_mode;

  if (material->draw_settings.blending) {
    transparent_instanced_render_instances.render_commands.push_back(render_instance);
  } else {
    deferred_instanced_render_instances.render_commands.push_back(render_instance);
  }
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