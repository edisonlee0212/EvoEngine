#include "RenderInstanceStorage.hpp"

#include "Application.hpp"
#include "EditorLayer.hpp"
#include "EnvironmentalMap.hpp"
#include "LodGroup.hpp"
#include "Platform.hpp"
#include "RenderLayer.hpp"
#include "Texture2D.hpp"

#include <cmath>

using namespace evo_engine;

namespace {
bool RequiresAlphaTestedShadow(const GltfShadeMaterial& material) {
  if (material.alpha_mode != static_cast<int32_t>(GltfAlphaMode::Opaque)) {
    return true;
  }
  if (material.pbr_base_color_texture != 0 || material.pbr_diffuse_texture != 0) {
    return true;
  }
  return material.pbr_base_color_factor.a <= material.alpha_cutoff;
}

bool UsesTransparentRasterPass(const Material& material, const GltfShadeMaterial& shade_material) {
  return material.draw_settings.blending || GltfMaterialRequiresTransparentPass(shade_material);
}

VkDrawMeshTasksIndirectCommandEXT CreateMeshTaskCommand(const uint32_t meshlet_range) {
  VkDrawMeshTasksIndirectCommandEXT command{};
  const uint32_t task_work_group_invocations =
      Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxPreferredTaskWorkGroupInvocations;
  command.groupCountX = (meshlet_range + task_work_group_invocations - 1) / task_work_group_invocations;
  command.groupCountY = 1;
  command.groupCountZ = 1;
  return command;
}

VkDrawIndexedIndirectCommand CreateIndexedCommand(const uint32_t triangle_offset, const uint32_t triangle_index_count) {
  VkDrawIndexedIndirectCommand command{};
  command.instanceCount = 1;
  command.firstIndex = triangle_offset * 3;
  command.indexCount = triangle_index_count * 3;
  command.vertexOffset = 0;
  command.firstInstance = 0;
  return command;
}

void AppendMeshIndirectCommands(std::vector<VkDrawIndexedIndirectCommand>& indexed_commands,
                                std::vector<VkDrawMeshTasksIndirectCommandEXT>& mesh_task_commands,
                                const uint32_t triangle_offset, const uint32_t triangle_index_count,
                                const uint32_t meshlet_range) {
  mesh_task_commands.emplace_back(CreateMeshTaskCommand(meshlet_range));
  indexed_commands.emplace_back(CreateIndexedCommand(triangle_offset, triangle_index_count));
}

size_t CountRenderInstances(const std::shared_ptr<RenderInstanceStorage::IRenderInstanceCollection>& render_instances) {
  size_t count = 0;
  render_instances->ForEachRenderInstance([&](const auto&) {
    ++count;
  });
  return count;
}

void ValidateDeferredMeshIndirectCommandCount(
    const std::shared_ptr<RenderInstanceStorage::IRenderInstanceCollection>& deferred_render_instances,
    const std::vector<VkDrawIndexedIndirectCommand>& indexed_commands,
    const std::vector<VkDrawMeshTasksIndirectCommandEXT>& mesh_task_commands) {
  if (!ApplicationContext::Get().GetLayer<RenderLayer>()) {
    return;
  }
  const auto deferred_count = CountRenderInstances(deferred_render_instances);
  const auto indexed_count = indexed_commands.size();
  const auto mesh_task_count = mesh_task_commands.size();
  assert(indexed_count == deferred_count);
  assert(mesh_task_count == deferred_count);
  if (indexed_count != deferred_count || mesh_task_count != deferred_count) {
    EVOENGINE_ERROR("Deferred mesh indirect command count mismatch: deferred_instances=" +
                    std::to_string(deferred_count) + ", indexed_commands=" + std::to_string(indexed_count) +
                    ", mesh_task_commands=" + std::to_string(mesh_task_count))
  }
}

bool GaussianSplatGpuRadixSortSupported() {
  return Platform::Initialized() && Platform::GetInstance().GetCapabilities().subgroup_size >= 32u;
}

}  // namespace

float RenderSettings::GetShadowCascadeSplit(const int split, const float near_distance) const {
  const auto clamped_split = glm::clamp(split, 0, 3);
  if (clamped_split == 3) {
    return 1.0f;
  }
  const auto far_distance = glm::max(max_shadow_distance, 0.001f);
  const auto near_clip_distance = glm::clamp(near_distance, 0.001f, far_distance);
  const auto split_ratio = static_cast<float>(clamped_split + 1) / 4.0f;
  const auto uniform_split = split_ratio;
  const auto logarithmic_split =
      near_clip_distance * std::pow(far_distance / near_clip_distance, split_ratio) / far_distance;
  return glm::clamp(glm::mix(uniform_split, logarithmic_split, glm::clamp(shadow_cascade_split_lambda, 0.0f, 1.0f)),
                    0.0f, 1.0f);
}

float RenderSettings::GetShadowCascadeSplitDistance(const int split, const float near_distance) const {
  return max_shadow_distance * GetShadowCascadeSplit(split, near_distance);
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
  if (alpha_tested_shadow != other.alpha_tested_shadow)
    return true;
  if (line_width != other.line_width)
    return true;
  if (cull_mode != other.cull_mode)
    return true;
  if (polygon_mode != other.polygon_mode)
    return true;
  if (ddgi_geometry.bottom_level_acceleration_structure != other.ddgi_geometry.bottom_level_acceleration_structure)
    return true;
  if (ddgi_geometry.triangle_offset != other.ddgi_geometry.triangle_offset)
    return true;
  return false;
}

bool RenderInstanceStorage::ExternalRenderInstance::HasDdgiRayTracingGeometry() const {
  return ddgi_geometry.IsValid();
}

void RenderInstanceStorage::ExternalRenderInstance::Apply(InstanceInfoBlock& instance_info_block) const {
  instance_info_block.model = model;
  instance_info_block.material_index = material_index;
  instance_info_block.triangle_offset = HasDdgiRayTracingGeometry() ? ddgi_geometry.triangle_offset : 0;
  instance_info_block.meshlet_index_offset = 0;
  instance_info_block.meshlet_size = 0;
  instance_info_block.info_index = entity_selected ? 1 : 0;
  instance_info_block.entity_index = owner.GetIndex();
  instance_info_block.renderer_handle = renderer_handle;
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
  if (alpha_tested_shadow != other.alpha_tested_shadow)
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
  instance_info_block.info_index = entity_selected ? 1 : 0;
  instance_info_block.triangle_offset = mesh->triangle_range_->prev_frame_offset;
  instance_info_block.meshlet_index_offset = mesh->meshlet_range_->prev_frame_offset;
  instance_info_block.meshlet_size = mesh->meshlet_range_->prev_frame_range;
  instance_info_block.entity_index = owner.GetIndex();
  instance_info_block.renderer_handle = renderer_handle;
}

uint32_t RenderInstanceStorage::MeshRenderInstance::Render(
    const VkCommandBuffer vk_command_buffer, const RenderInstancePushConstant& render_instance_push_constant,
    const std::shared_ptr<GraphicsPipeline>& graphics_pipeline) const {
  const uint32_t task_work_group_invocations =
      Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxPreferredTaskWorkGroupInvocations;
  graphics_pipeline->PushConstant(vk_command_buffer, 0, render_instance_push_constant);

  if (Platform::MeshShaderEnabled()) {
    graphics_pipeline->states.ApplyAllStates(vk_command_buffer);
    const uint32_t count =
        (mesh->meshlet_range_->prev_frame_range + task_work_group_invocations - 1) / task_work_group_invocations;
    graphics_pipeline->DrawMeshTasks(vk_command_buffer, count);
  } else {
    mesh->DrawIndexed(vk_command_buffer, graphics_pipeline->states, 1);
  }
  return mesh->triangle_range_->prev_frame_index_count;
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
  if (ray_tracing_triangle_range != other.ray_tracing_triangle_range)
    return true;
  if (ray_tracing_blas != other.ray_tracing_blas)
    return true;
  if (geometry_version != other.geometry_version)
    return true;
  if (ray_tracing_geometry_version != other.ray_tracing_geometry_version)
    return true;
  if (material_version != other.material_version)
    return true;
  if (bone_matrices_version != other.bone_matrices_version)
    return true;
  if (cast_shadow != other.cast_shadow)
    return true;
  if (alpha_tested_shadow != other.alpha_tested_shadow)
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
  instance_info_block.info_index = entity_selected ? 1 : 0;
  if (ray_tracing_triangle_range && ray_tracing_triangle_range->prev_frame_index_count != 0) {
    instance_info_block.triangle_offset = ray_tracing_triangle_range->prev_frame_offset;
  } else if (skinned_mesh->ray_tracing_triangle_range_ &&
             skinned_mesh->ray_tracing_triangle_range_->prev_frame_index_count != 0) {
    instance_info_block.triangle_offset = skinned_mesh->ray_tracing_triangle_range_->prev_frame_offset;
  } else {
    instance_info_block.triangle_offset = skinned_mesh->skinned_triangle_range_->prev_frame_offset;
  }
  instance_info_block.meshlet_index_offset = skinned_mesh->skinned_meshlet_range_->prev_frame_offset;
  instance_info_block.meshlet_size = skinned_mesh->skinned_meshlet_range_->prev_frame_range;
  instance_info_block.entity_index = owner.GetIndex();
  instance_info_block.renderer_handle = renderer_handle;
}

uint32_t RenderInstanceStorage::SkinnedMeshRenderInstance::Render(
    VkCommandBuffer vk_command_buffer, const RenderInstancePushConstant& render_instance_push_constant,
    const std::shared_ptr<GraphicsPipeline>& graphics_pipeline) const {
  graphics_pipeline->BindDescriptorSet(vk_command_buffer, 1, bone_matrices->GetDescriptorSet()->GetVkDescriptorSet());
  graphics_pipeline->PushConstant(vk_command_buffer, 0, render_instance_push_constant);
  skinned_mesh->DrawIndexed(vk_command_buffer, graphics_pipeline->states, 1);
  return skinned_mesh->skinned_triangle_range_->prev_frame_index_count;
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
  if (alpha_tested_shadow != other.alpha_tested_shadow)
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
  instance_info_block.info_index = entity_selected ? 1 : 0;
  instance_info_block.triangle_offset = mesh->triangle_range_->prev_frame_offset;
  instance_info_block.meshlet_index_offset = mesh->meshlet_range_->prev_frame_offset;
  instance_info_block.meshlet_size = mesh->meshlet_range_->prev_frame_range;
  instance_info_block.entity_index = owner.GetIndex();
  instance_info_block.renderer_handle = renderer_handle;
}

uint32_t RenderInstanceStorage::InstancedRenderInstance::Render(
    const VkCommandBuffer vk_command_buffer, const RenderInstancePushConstant& render_instance_push_constant,
    const std::shared_ptr<GraphicsPipeline>& graphics_pipeline) const {
  graphics_pipeline->BindDescriptorSet(vk_command_buffer, 1, particle_infos->GetDescriptorSet()->GetVkDescriptorSet());
  graphics_pipeline->PushConstant(vk_command_buffer, 0, render_instance_push_constant);
  mesh->DrawIndexed(vk_command_buffer, graphics_pipeline->states, particle_infos->PeekParticleInfoList().size());
  return mesh->triangle_range_->prev_frame_index_count * particle_infos->PeekParticleInfoList().size();
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
  if (alpha_tested_shadow != other.alpha_tested_shadow)
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
  instance_info_block.info_index = entity_selected ? 1 : 0;
  instance_info_block.triangle_offset = strands->segment_range_->prev_frame_offset;
  instance_info_block.meshlet_index_offset = strands->strand_meshlet_range_->prev_frame_offset;
  instance_info_block.meshlet_size = strands->strand_meshlet_range_->prev_frame_range;
  instance_info_block.entity_index = owner.GetIndex();
  instance_info_block.renderer_handle = renderer_handle;
}

uint32_t RenderInstanceStorage::StrandsRenderInstance::Render(
    const VkCommandBuffer vk_command_buffer, const RenderInstancePushConstant& render_instance_push_constant,
    const std::shared_ptr<GraphicsPipeline>& graphics_pipeline) const {
  graphics_pipeline->PushConstant(vk_command_buffer, 0, render_instance_push_constant);
  strands->DrawIndexed(vk_command_buffer, graphics_pipeline->states, 1);
  return strands->segment_range_->prev_frame_index_count;
}

bool RenderInstanceStorage::GaussianSplatRenderInstance::operator!=(const GaussianSplatRenderInstance& other) const {
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
  if (gaussian_splat != other.gaussian_splat)
    return true;
  if (geometry_version != other.geometry_version)
    return true;
  if (opacity_scale != other.opacity_scale)
    return true;
  if (sh_degree != other.sh_degree)
    return true;
  if (sort_mode != other.sort_mode)
    return true;
  if (depth_mode != other.depth_mode)
    return true;
  if (raster_mode != other.raster_mode)
    return true;
  return false;
}

void RenderInstanceStorage::GaussianSplatRenderInstance::Apply(InstanceInfoBlock& instance_info_block) const {
  instance_info_block.model = model;
  instance_info_block.material_index = -1;
  instance_info_block.triangle_offset = 0;
  instance_info_block.meshlet_index_offset = 0;
  instance_info_block.meshlet_size = 0;
  instance_info_block.info_index = entity_selected ? 1 : 0;
  instance_info_block.entity_index = owner.GetIndex();
  instance_info_block.renderer_handle = renderer_handle;
}

uint32_t RenderInstanceStorage::GaussianSplatRenderInstance::Render(VkCommandBuffer, const RenderInstancePushConstant&,
                                                                    const std::shared_ptr<GraphicsPipeline>&) const {
  return 0;
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
  ForEachExternalRenderInstance([&](const auto& render_instance) {
    action(render_instance);
  });
}

void RenderInstanceStorage::ExternalRenderInstanceCollection::ForEachExternalRenderInstance(
    const std::function<void(const std::shared_ptr<ExternalRenderInstance>&)>& action) const {
  for (const auto& i : render_commands) {
    action(i);
  }
}

bool RenderInstanceStorage::ExternalRenderInstanceCollection::HasDdgiRayTracingGeometry() const {
  for (const auto& render_command : render_commands) {
    if (render_command && render_command->HasDdgiRayTracingGeometry()) {
      return true;
    }
  }
  return false;
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
  ForEachMeshRenderInstance([&](const auto& render_instance) {
    action(render_instance);
  });
}

void RenderInstanceStorage::MeshRenderInstanceCollection::ForEachMeshRenderInstance(
    const std::function<void(const std::shared_ptr<MeshRenderInstance>&)>& action) const {
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
  ForEachSkinnedMeshRenderInstance([&](const auto& render_instance) {
    action(render_instance);
  });
}

void RenderInstanceStorage::SkinnedMeshRenderInstanceCollection::ForEachSkinnedMeshRenderInstance(
    const std::function<void(const std::shared_ptr<SkinnedMeshRenderInstance>&)>& action) const {
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
  ForEachStrandsRenderInstance([&](const auto& render_instance) {
    action(render_instance);
  });
}

void RenderInstanceStorage::StrandsRenderInstanceCollection::ForEachStrandsRenderInstance(
    const std::function<void(const std::shared_ptr<StrandsRenderInstance>&)>& action) const {
  for (const auto& i : render_commands) {
    action(i);
  }
}

bool RenderInstanceStorage::GaussianSplatRenderInstanceCollection::Empty() const {
  return render_commands.empty();
}

void RenderInstanceStorage::GaussianSplatRenderInstanceCollection::Register(
    const std::shared_ptr<IRenderInstance>& render_instance) {
  render_commands.emplace_back(std::dynamic_pointer_cast<GaussianSplatRenderInstance>(render_instance));
}

bool RenderInstanceStorage::GaussianSplatRenderInstanceCollection::operator!=(
    const GaussianSplatRenderInstanceCollection& other) const {
  if (render_commands.size() != other.render_commands.size())
    return true;
  for (uint32_t i = 0; i < render_commands.size(); i++) {
    if (render_commands[i] && other.render_commands[i]) {
      if (*render_commands[i] != *other.render_commands[i])
        return true;
    } else if (render_commands[i] != other.render_commands[i]) {
      return true;
    }
  }
  return false;
}

void RenderInstanceStorage::GaussianSplatRenderInstanceCollection::ForEachRenderInstance(
    const std::function<void(const std::shared_ptr<IRenderInstance>&)>& action) {
  ForEachGaussianSplatRenderInstance([&](const auto& render_instance) {
    action(render_instance);
  });
}

void RenderInstanceStorage::GaussianSplatRenderInstanceCollection::ForEachGaussianSplatRenderInstance(
    const std::function<void(const std::shared_ptr<GaussianSplatRenderInstance>&)>& action) const {
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
  ForEachInstancedRenderInstance([&](const auto& render_instance) {
    action(render_instance);
  });
}

void RenderInstanceStorage::InstancedRenderInstanceCollection::ForEachInstancedRenderInstance(
    const std::function<void(const std::shared_ptr<InstancedRenderInstance>&)>& action) const {
  for (const auto& i : render_commands) {
    action(i);
  }
}

void RenderInstanceStorage::RenderInfoBlock::Apply(const RenderSettings& target_render_settings) {
  for (int split = 0; split < 4; split++) {
    split_distances[split] = target_render_settings.GetShadowCascadeSplitDistance(split);
  }
  if (const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>()) {
    brdflut_texture_index = render_layer->environmental_brdf_lut_->GetTextureStorageIndex();
  }
  if (target_render_settings.enable_debug_visualization)
    debug_visualization = 1;
  else
    debug_visualization = 0;

  pcf_sample_amount = target_render_settings.pcf_sample_amount;
  shadow_cascade_transition_width = glm::max(target_render_settings.shadow_cascade_transition_width, 0.0f);
  shadow_debug_parameters = glm::ivec4(glm::clamp(target_render_settings.shadow_debug_mode, 0, 5),
                                       glm::clamp(target_render_settings.shadow_debug_selected_cascade, 0, 3),
                                       glm::max(target_render_settings.shadow_debug_selected_light, 0), 0);
  shadow_fade_parameters = glm::vec4(glm::clamp(target_render_settings.shadow_distance_fade, 0.0f,
                                                glm::max(target_render_settings.max_shadow_distance, 0.0f)),
                                     0.0f, 0.0f, 0.0f);
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

  if (shadow_cascade_transition_width != other.shadow_cascade_transition_width)
    return true;
  if (ddgi_indirect_intensity != other.ddgi_indirect_intensity)
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
  if (ddgi_first_probe != other.ddgi_first_probe)
    return true;
  if (ddgi_probe_step_x != other.ddgi_probe_step_x)
    return true;
  if (ddgi_probe_step_y != other.ddgi_probe_step_y)
    return true;
  if (ddgi_probe_step_z != other.ddgi_probe_step_z)
    return true;
  if (ddgi_probe_counts != other.ddgi_probe_counts)
    return true;
  if (ddgi_probe_scroll_offset != other.ddgi_probe_scroll_offset)
    return true;
  if (ddgi_atlas_parameters != other.ddgi_atlas_parameters)
    return true;
  if (ddgi_volume_parameters != other.ddgi_volume_parameters)
    return true;
  if (ddgi_sampling_parameters != other.ddgi_sampling_parameters)
    return true;
  if (shadow_debug_parameters != other.shadow_debug_parameters)
    return true;
  if (shadow_fade_parameters != other.shadow_fade_parameters)
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
  if (environment_type != other.environment_type)
    return true;
  if (environment_pdf_texture_index != other.environment_pdf_texture_index)
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
  if (info_index != other.info_index)
    return true;
  if (entity_index != other.entity_index)
    return true;
  if (renderer_handle != other.renderer_handle)
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
          } else if (const auto gaussian_splat_renderer = renderer.Get<GaussianSplatRenderer>()) {
            lod_group_renderers.insert(gaussian_splat_renderer->GetHandle());
            if (render_current_level && target_scene->IsEntityEnabled(owner) &&
                target_scene->IsEntityEnabled(gaussian_splat_renderer->GetOwner()) &&
                gaussian_splat_renderer->IsEnabled()) {
              if (RegisterEntity(target_scene, owner, gaussian_splat_renderer, min_bound, max_bound)) {
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

  if (const auto* owners = target_scene->UnsafeGetPrivateComponentOwnersList<GaussianSplatRenderer>()) {
    for (auto owner : *owners) {
      if (!target_scene->IsEntityEnabled(owner))
        continue;
      auto gaussian_splat_renderer = target_scene->GetOrSetPrivateComponent<GaussianSplatRenderer>(owner).lock();
      if (lod_group_renderers.find(gaussian_splat_renderer->GetHandle()) != lod_group_renderers.end())
        continue;
      if (RegisterEntity(target_scene, owner, gaussian_splat_renderer, min_bound, max_bound)) {
        has_render_instance = true;
      }
    }
  }

  if (!has_render_instance) {
    min_bound = max_bound = glm::vec3(0.0f);
  }
}

void RenderInstanceStorage::BuildRenderInstanceBlocks() {
  total_opaque_shadow_mesh_triangles = 0;
  total_alpha_tested_shadow_mesh_triangles = 0;
  opaque_shadow_mesh_draw_indexed_indirect_commands.clear();
  opaque_shadow_mesh_draw_mesh_tasks_indirect_commands.clear();
  alpha_tested_shadow_mesh_draw_indexed_indirect_commands.clear();
  alpha_tested_shadow_mesh_draw_mesh_tasks_indirect_commands.clear();

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
  const auto prepare_gaussian_splat_render_instance =
      [&](const std::shared_ptr<GaussianSplatRenderInstance>& render_instance) {
        if (!render_instance || !render_instance->gaussian_splat) {
          return;
        }
        render_instance->geometry_version = render_instance->gaussian_splat->GetGpuDataRevision();
        const auto camera_count = std::min(cameras.size(), camera_info_blocks_.size());
        for (size_t camera_index = 0; camera_index < camera_count; ++camera_index) {
          const auto& camera = cameras[camera_index].second;
          if (!camera) {
            continue;
          }
          (void)render_instance->gaussian_splat->EnsureGpuPrepassCache(camera->GetHandle(),
                                                                       render_instance->renderer_handle);
          if (render_instance->sort_mode == GaussianSplatSortMode::CpuDepth ||
              (render_instance->sort_mode == GaussianSplatSortMode::GpuRadix &&
               !GaussianSplatGpuRadixSortSupported())) {
            (void)render_instance->gaussian_splat->EnsureSortedIndices(
                camera->GetHandle(), render_instance->renderer_handle, render_instance->model.value,
                camera_info_blocks_[camera_index].view);
          }
        }
      };
  const auto register_shadow_mesh_indirect_command = [&](const std::shared_ptr<MeshRenderInstance>& render_instance) {
    VkDrawIndexedIndirectCommand opaque_draw{};
    VkDrawMeshTasksIndirectCommandEXT opaque_mesh_task{};
    VkDrawIndexedIndirectCommand alpha_tested_draw{};
    VkDrawMeshTasksIndirectCommandEXT alpha_tested_mesh_task{};

    if (render_instance && render_instance->cast_shadow && render_instance->mesh) {
      const auto triangle_offset = render_instance->mesh->triangle_range_->prev_frame_offset;
      const auto triangle_index_count = render_instance->mesh->triangle_range_->prev_frame_index_count;
      const auto meshlet_range = render_instance->mesh->meshlet_range_->prev_frame_range;
      if (render_instance->alpha_tested_shadow) {
        alpha_tested_draw = CreateIndexedCommand(triangle_offset, triangle_index_count);
        alpha_tested_mesh_task = CreateMeshTaskCommand(meshlet_range);
        total_alpha_tested_shadow_mesh_triangles += triangle_index_count;
      } else {
        opaque_draw = CreateIndexedCommand(triangle_offset, triangle_index_count);
        opaque_mesh_task = CreateMeshTaskCommand(meshlet_range);
        total_opaque_shadow_mesh_triangles += triangle_index_count;
      }
    }

    opaque_shadow_mesh_draw_indexed_indirect_commands.emplace_back(opaque_draw);
    opaque_shadow_mesh_draw_mesh_tasks_indirect_commands.emplace_back(opaque_mesh_task);
    alpha_tested_shadow_mesh_draw_indexed_indirect_commands.emplace_back(alpha_tested_draw);
    alpha_tested_shadow_mesh_draw_mesh_tasks_indirect_commands.emplace_back(alpha_tested_mesh_task);
  };
  deferred_render_instances->ForEachMeshRenderInstance([&](const auto& render_instance) {
    register_render_instance(render_instance);
    register_shadow_mesh_indirect_command(render_instance);
  });
  ValidateDeferredMeshIndirectCommandCount(deferred_render_instances, mesh_draw_indexed_indirect_commands,
                                           mesh_draw_mesh_tasks_indirect_commands);
  deferred_skinned_render_instances->ForEachSkinnedMeshRenderInstance([&](const auto& render_instance) {
    register_render_instance(render_instance);
  });
  deferred_instanced_render_instances->ForEachInstancedRenderInstance([&](const auto& render_instance) {
    register_render_instance(render_instance);
  });
  deferred_strands_render_instances->ForEachStrandsRenderInstance([&](const auto& render_instance) {
    register_render_instance(render_instance);
  });

  forward_render_instances->ForEachMeshRenderInstance([&](const auto& render_instance) {
    register_render_instance(render_instance);
  });
  forward_skinned_render_instances->ForEachSkinnedMeshRenderInstance([&](const auto& render_instance) {
    register_render_instance(render_instance);
  });
  forward_instanced_render_instances->ForEachInstancedRenderInstance([&](const auto& render_instance) {
    register_render_instance(render_instance);
  });
  forward_strands_render_instances->ForEachStrandsRenderInstance([&](const auto& render_instance) {
    register_render_instance(render_instance);
  });

  transparent_render_instances->ForEachMeshRenderInstance([&](const auto& render_instance) {
    register_render_instance(render_instance);
  });
  transparent_skinned_render_instances->ForEachSkinnedMeshRenderInstance([&](const auto& render_instance) {
    register_render_instance(render_instance);
  });
  transparent_instanced_render_instances->ForEachInstancedRenderInstance([&](const auto& render_instance) {
    register_render_instance(render_instance);
  });
  transparent_strands_render_instances->ForEachStrandsRenderInstance([&](const auto& render_instance) {
    register_render_instance(render_instance);
  });

  gaussian_splat_render_instances->ForEachGaussianSplatRenderInstance([&](const auto& render_instance) {
    prepare_gaussian_splat_render_instance(render_instance);
    register_render_instance(render_instance);
  });

  external_render_instances->ForEachExternalRenderInstance([&](const auto& render_instance) {
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
  const auto& graphics_settings = ApplicationContext::Get().GetApplicationInfo().graphics_settings;
  const auto max_directional_light_size = graphics_settings.max_directional_light_size;

  if (directional_light_entities && !directional_light_entities->empty() && max_directional_light_size > 0) {
    directional_light_info_blocks_.resize(max_directional_light_size * cameras.size());
    uint32_t directional_light_size = 0;
    uint32_t directional_shadow_light_size = 0;
    for (const auto& light_entity : *directional_light_entities) {
      if (!target_scene->IsEntityEnabled(light_entity))
        continue;
      const auto dlc = target_scene->GetOrSetPrivateComponent<DirectionalLight>(light_entity).lock();
      if (!dlc->IsEnabled())
        continue;
      if (directional_light_size >= max_directional_light_size)
        break;
      directional_light_size++;
      if (dlc->cast_shadow) {
        directional_shadow_light_size++;
      }
    }
    render_info_block.directional_light_size = static_cast<int>(directional_light_size);
    std::vector<glm::uvec3> viewport_results;
    Lighting::AllocateAtlas(directional_shadow_light_size, graphics_settings.directional_light_shadow_map_resolution,
                            viewport_results);
    for (const auto& [cameraGlobalTransform, camera] : cameras) {
      auto camera_index = GetCameraIndex(camera->GetHandle());
      size_t directional_light_index = 0;
      size_t directional_shadow_light_index = 0;
      for (const auto& light_entity : *directional_light_entities) {
        if (!target_scene->IsEntityEnabled(light_entity))
          continue;
        const auto dlc = target_scene->GetOrSetPrivateComponent<DirectionalLight>(light_entity).lock();
        if (!dlc->IsEnabled())
          continue;
        if (directional_light_index >= max_directional_light_size)
          break;
        const auto block_index = camera_index * max_directional_light_size + directional_light_index;
        auto& viewport = directional_light_info_blocks_[block_index].viewport;
        viewport = glm::ivec4(0);
        if (dlc->cast_shadow && directional_shadow_light_index < viewport_results.size()) {
          viewport.x = viewport_results[directional_shadow_light_index].x;
          viewport.y = viewport_results[directional_shadow_light_index].y;
          viewport.z = viewport_results[directional_shadow_light_index].z;
          viewport.w = viewport_results[directional_shadow_light_index].z;
          directional_shadow_light_index++;
        }
        directional_light_index++;
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
        if (directional_light_index >= max_directional_light_size)
          break;
        glm::quat rotation = target_scene->GetDataComponent<GlobalTransform>(light_entity).GetRotation();
        glm::vec3 light_dir = glm::normalize(rotation * glm::vec3(0, 0, 1));
        float plane_distance = 0;
        glm::vec3 center;
        const auto block_index = camera_index * max_directional_light_size + directional_light_index;
        directional_light_info_blocks_[block_index].direction = glm::vec4(light_dir, 0.0f);
        directional_light_info_blocks_[block_index].diffuse =
            glm::vec4(dlc->diffuse * dlc->diffuse_brightness, dlc->cast_shadow);
        directional_light_info_blocks_[block_index].specular = glm::vec4(0.0f);
        const auto camera_near_distance = glm::max(camera->camera_settings.near_distance, 0.001f);
        for (int split = 0; split < 4; split++) {
          float split_start = 0;
          float split_end = render_settings.GetShadowCascadeSplitDistance(split, camera_near_distance);
          if (split != 0)
            split_start = render_settings.GetShadowCascadeSplitDistance(split - 1, camera_near_distance);
          render_info_block.split_distances[split] = split_end;
          glm::mat4 light_projection, light_view;
          float max_distance = split_end;
          glm::vec3 light_pos;
          glm::vec3 camera_frustum_center =
              (main_camera_rot * glm::vec3(0, 0, -1)) * ((split_end - split_start) / 2.0f + split_start) +
              main_camera_pos;

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
          plane_distance = glm::max(glm::max(d0, d1), glm::max(d2, d3));
          light_pos = center - light_dir * plane_distance;
          light_view = glm::lookAt(light_pos, light_pos + light_dir, glm::normalize(rotation * glm::vec3(0, 1, 0)));
          light_projection =
              glm::ortho(-max_distance, max_distance, -max_distance, max_distance, 0.0f, plane_distance * 2.0f);
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
          directional_light_info_blocks_[block_index].light_frustum_width[split] = max_distance;
          directional_light_info_blocks_[block_index].light_frustum_distance[split] = plane_distance;
          if (split == 4 - 1)
            directional_light_info_blocks_[block_index].reserved_parameters =
                glm::vec4(dlc->light_size, dlc->slope_bias, dlc->bias, dlc->normal_offset);
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
    std::multimap<float, size_t> sorted_point_shadow_light_indices;
    uint32_t point_shadow_light_size = 0;
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
      point_light_info_blocks_[render_info_block.point_light_size].viewport = glm::ivec4(0);
      point_light_info_blocks_[render_info_block.point_light_size].constant_linear_quad_far_plane.w =
          plc->range > 0.0f ? plc->range : plc->GetFarPlane();

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

      if (plc->cast_shadow) {
        sorted_point_shadow_light_indices.insert(
            {glm::distance(main_camera_position, position), render_info_block.point_light_size});
        point_shadow_light_size++;
      }
      render_info_block.point_light_size++;
    }
    std::vector<glm::uvec3> view_port_results;
    Lighting::AllocateAtlas(point_shadow_light_size, graphics_settings.point_light_shadow_map_resolution,
                            view_port_results);
    int allocation_index = 0;
    for (const auto& point_light_index : sorted_point_shadow_light_indices) {
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
    std::multimap<float, size_t> sorted_spot_shadow_light_indices;
    uint32_t spot_shadow_light_size = 0;
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
      spot_light_info_blocks_[render_info_block.spot_light_size].constant_linear_quad_far_plane.w =
          slc->range > 0.0f ? slc->range : slc->GetFarPlane();
      spot_light_info_blocks_[render_info_block.spot_light_size].diffuse =
          glm::vec4(slc->diffuse * slc->diffuse_brightness, slc->cast_shadow);
      spot_light_info_blocks_[render_info_block.spot_light_size].specular = glm::vec4(0);
      spot_light_info_blocks_[render_info_block.spot_light_size].viewport = glm::ivec4(0);

      glm::mat4 shadow_proj = glm::perspective(glm::radians(slc->outer_degrees * 2.0f), 1.0f,
                                               slc->shadow_distance / 1000.f, slc->shadow_distance);
      spot_light_info_blocks_[render_info_block.spot_light_size].light_space_matrix =
          shadow_proj * glm::lookAt(position, position + front, up);
      spot_light_info_blocks_[render_info_block.spot_light_size].cut_off_outer_cut_off_light_size_bias =
          glm::vec4(glm::cos(glm::radians(slc->inner_degrees)), glm::cos(glm::radians(slc->outer_degrees)),
                    slc->light_size, slc->bias);

      if (slc->cast_shadow) {
        sorted_spot_shadow_light_indices.insert(
            {glm::distance(main_camera_position, position), render_info_block.spot_light_size});
        spot_shadow_light_size++;
      }
      render_info_block.spot_light_size++;
    }
    std::vector<glm::uvec3> view_port_results;
    Lighting::AllocateAtlas(spot_shadow_light_size, graphics_settings.spot_light_shadow_map_resolution,
                            view_port_results);
    int allocation_index = 0;
    for (const auto& spot_light_index : sorted_spot_shadow_light_indices) {
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
  environment_info_block.environment_pdf_texture_index = -1.0f;
  switch (target_scene->environment.environment_type) {
    case Scene::EnvironmentType::EnvironmentalMap: {
      environment_info_block.background_color.w = 0.0f;
      environment_info_block.environment_type = 0.0f;
      if (const auto environmental_map = target_scene->environment.environmental_map.Get<EnvironmentalMap>()) {
        if (const auto pdf_texture = environmental_map->environment_pdf_texture.Get<Texture2D>()) {
          environment_info_block.environment_pdf_texture_index =
              static_cast<float>(pdf_texture->GetTextureStorageIndex());
        }
      }
    } break;
    case Scene::EnvironmentType::Color: {
      environment_info_block.background_color = glm::vec4(target_scene->environment.background_color, 1.0f);
      environment_info_block.environment_type = 1.0f;
    } break;
  }
  environment_info_block.environmental_map_gamma = target_scene->environment.environment_gamma;
  environment_info_block.environmental_lighting_intensity = target_scene->environment.ambient_light_intensity;
  environment_info_block.background_intensity = target_scene->environment.background_intensity;
}

void RenderInstanceStorage::CollectEditorCameras(
    const std::shared_ptr<Scene>& target_scene,
    std::vector<std::pair<GlobalTransform, std::shared_ptr<Camera>>>& cameras) {
  if (const auto editor_layer = ApplicationContext::Get().GetLayer<EditorLayer>()) {
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

  const auto& graphics_settings = ApplicationContext::Get().GetApplicationInfo().graphics_settings;

  buffer_create_info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
  buffer_create_info.size = sizeof(CameraInfoBlock) * Platform::Constants::initial_camera_size;
  camera_info_descriptor_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  buffer_create_info.size = sizeof(DirectionalLightInfoBlock) * graphics_settings.max_directional_light_size *
                            Platform::Constants::initial_camera_size;
  directional_light_info_descriptor_buffer =
      std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  buffer_create_info.size = sizeof(PointLightInfoBlock) * graphics_settings.max_point_light_size;
  point_light_info_descriptor_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  buffer_create_info.size = sizeof(SpotLightInfoBlock) * graphics_settings.max_spot_light_size;
  spot_light_info_descriptor_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  buffer_create_info.size =
      glm::max(static_cast<size_t>(1), sizeof(GltfShadeMaterial) * Platform::Constants::initial_material_size);
  gltf_material_descriptor_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  buffer_create_info.size =
      glm::max(static_cast<size_t>(1), sizeof(GltfTextureInfo) * Platform::Constants::initial_material_size);
  gltf_texture_info_descriptor_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
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

  buffer_create_info.size =
      glm::max(static_cast<size_t>(1),
               sizeof(VkDrawIndexedIndirectCommand) * opaque_shadow_mesh_draw_indexed_indirect_commands.size());
  opaque_shadow_mesh_draw_indexed_indirect_commands_buffer =
      std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);

  buffer_create_info.size =
      glm::max(static_cast<size_t>(1),
               sizeof(VkDrawMeshTasksIndirectCommandEXT) * opaque_shadow_mesh_draw_mesh_tasks_indirect_commands.size());
  opaque_shadow_mesh_draw_mesh_tasks_indirect_commands_buffer =
      std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);

  buffer_create_info.size =
      glm::max(static_cast<size_t>(1),
               sizeof(VkDrawIndexedIndirectCommand) * alpha_tested_shadow_mesh_draw_indexed_indirect_commands.size());
  alpha_tested_shadow_mesh_draw_indexed_indirect_commands_buffer =
      std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);

  buffer_create_info.size =
      glm::max(static_cast<size_t>(1), sizeof(VkDrawMeshTasksIndirectCommandEXT) *
                                           alpha_tested_shadow_mesh_draw_mesh_tasks_indirect_commands.size());
  alpha_tested_shadow_mesh_draw_mesh_tasks_indirect_commands_buffer =
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

  gaussian_splat_render_instances = std::make_shared<GaussianSplatRenderInstanceCollection>();
  external_render_instances = std::make_shared<ExternalRenderInstanceCollection>();
}

void RenderInstanceStorage::Clear() {
  total_mesh_triangles = 0;
  total_opaque_shadow_mesh_triangles = 0;
  total_alpha_tested_shadow_mesh_triangles = 0;
  total_skinned_mesh_triangles = 0;
  total_instanced_mesh_triangles = 0;
  total_strands_segments = 0;
  total_gaussian_splats = 0;

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

  gaussian_splat_render_instances = std::make_shared<GaussianSplatRenderInstanceCollection>();
  external_render_instances = std::make_shared<ExternalRenderInstanceCollection>();

  instance_entity_handles_.clear();
  instance_renderer_handles_.clear();

  renderer_indices_.clear();
  camera_indices_.clear();
  material_indices_.clear();
  render_settings = {};

  camera_info_blocks_.clear();
  gltf_material_cache_.Clear();
  instance_info_blocks_.clear();
  directional_light_info_blocks_.clear();
  point_light_info_blocks_.clear();
  spot_light_info_blocks_.clear();
  render_info_block = {};

  cameras.clear();

  mesh_draw_indexed_indirect_commands.clear();
  mesh_draw_mesh_tasks_indirect_commands.clear();
  opaque_shadow_mesh_draw_indexed_indirect_commands.clear();
  opaque_shadow_mesh_draw_mesh_tasks_indirect_commands.clear();
  alpha_tested_shadow_mesh_draw_indexed_indirect_commands.clear();
  alpha_tested_shadow_mesh_draw_mesh_tasks_indirect_commands.clear();
}

void RenderInstanceStorage::Upload() const {
  if (!Platform::Initialized())
    return;
  camera_info_descriptor_buffer->UploadVector(camera_info_blocks_);
  gltf_material_descriptor_buffer->UploadVector(gltf_material_cache_.GetShadeMaterials());
  gltf_texture_info_descriptor_buffer->UploadVector(gltf_material_cache_.GetTextureInfos());
  instance_info_descriptor_buffer->UploadVector(instance_info_blocks_);
  render_info_descriptor_buffer->Upload(render_info_block);
  directional_light_info_descriptor_buffer->UploadVector(directional_light_info_blocks_);
  point_light_info_descriptor_buffer->UploadVector(point_light_info_blocks_);
  spot_light_info_descriptor_buffer->UploadVector(spot_light_info_blocks_);

  mesh_draw_indexed_indirect_commands_buffer->UploadVector(mesh_draw_indexed_indirect_commands);
  mesh_draw_mesh_tasks_indirect_commands_buffer->UploadVector(mesh_draw_mesh_tasks_indirect_commands);
  opaque_shadow_mesh_draw_indexed_indirect_commands_buffer->UploadVector(
      opaque_shadow_mesh_draw_indexed_indirect_commands);
  opaque_shadow_mesh_draw_mesh_tasks_indirect_commands_buffer->UploadVector(
      opaque_shadow_mesh_draw_mesh_tasks_indirect_commands);
  alpha_tested_shadow_mesh_draw_indexed_indirect_commands_buffer->UploadVector(
      alpha_tested_shadow_mesh_draw_indexed_indirect_commands);
  alpha_tested_shadow_mesh_draw_mesh_tasks_indirect_commands_buffer->UploadVector(
      alpha_tested_shadow_mesh_draw_mesh_tasks_indirect_commands);

  environment_info_descriptor_buffer->Upload(environment_info_block);
}

const std::vector<GltfShadeMaterial>& RenderInstanceStorage::GetGltfShadeMaterials() const {
  return gltf_material_cache_.GetShadeMaterials();
}

const std::vector<GltfTextureInfo>& RenderInstanceStorage::GetGltfTextureInfos() const {
  return gltf_material_cache_.GetTextureInfos();
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

  if (GetGltfShadeMaterials() != other.GetGltfShadeMaterials())
    return true;
  if (GetGltfTextureInfos() != other.GetGltfTextureInfos())
    return true;

  for (uint32_t i = 0; i < camera_info_blocks_.size(); i++) {
    if (camera_info_blocks_[i] != other.camera_info_blocks_[i])
      return true;
  }

  if (*gaussian_splat_render_instances != *other.gaussian_splat_render_instances)
    return true;

  if (*external_render_instances != *other.external_render_instances)
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
  if (mesh->triangle_range_->prev_frame_index_count == 0 || mesh->meshlet_range_->prev_frame_range == 0)
    return false;
  auto mesh_bound = mesh->GetBound();
  mesh_bound.ApplyTransform(model.value);
  const auto material_data = BuildMaterialGltfData(*material);
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
  render_instance->alpha_tested_shadow = RequiresAlphaTestedShadow(material_data.shade_material);
  render_instance->world_bound = mesh_bound;
  render_instance->geometry_version = mesh->GetVersion();
  render_instance->material_version = material->GetVersion();
  render_instance->material_index = RegisterMaterial(material, material_data);
  render_instance->line_width = material->draw_settings.line_width;
  render_instance->cull_mode = material->draw_settings.cull_mode;
  render_instance->polygon_mode = material->draw_settings.polygon_mode;
  render_instance->entity_selected = false;
  if (UsesTransparentRasterPass(*material, material_data.shade_material)) {
    transparent_render_instances->Register(render_instance);
  } else {
    deferred_render_instances->Register(render_instance);
    AppendMeshIndirectCommands(mesh_draw_indexed_indirect_commands, mesh_draw_mesh_tasks_indirect_commands,
                               mesh->triangle_range_->prev_frame_offset, mesh->triangle_range_->prev_frame_index_count,
                               mesh->meshlet_range_->prev_frame_range);
    total_mesh_triangles += mesh->triangle_range_->prev_frame_index_count;
  }

  return true;
}

bool RenderInstanceStorage::RegisterMeshDrawInstancedCommand(
    const std::shared_ptr<Mesh>& mesh, const std::shared_ptr<Material>& material, const GlobalTransform& model,
    const std::shared_ptr<ParticleInfoList>& particle_info_list, const bool cast_shadow) {
  if (!material || !mesh || !mesh->meshlet_range_ || !mesh->triangle_range_)
    return false;
  if (mesh->UnsafeGetVertices().empty() || mesh->UnsafeGetTriangles().empty())
    return false;
  if (mesh->triangle_range_->prev_frame_index_count == 0 || mesh->meshlet_range_->prev_frame_range == 0)
    return false;
  const auto material_data = BuildMaterialGltfData(*material);
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
  render_instance->alpha_tested_shadow = RequiresAlphaTestedShadow(material_data.shade_material);
  render_instance->geometry_version = mesh->GetVersion();
  render_instance->material_version = material->GetVersion();
  render_instance->material_index = RegisterMaterial(material, material_data);
  render_instance->line_width = material->draw_settings.line_width;
  render_instance->cull_mode = material->draw_settings.cull_mode;
  render_instance->polygon_mode = material->draw_settings.polygon_mode;
  render_instance->entity_selected = false;
  if (UsesTransparentRasterPass(*material, material_data.shade_material)) {
    transparent_instanced_render_instances->Register(render_instance);
  } else {
    deferred_instanced_render_instances->Register(render_instance);
  }

  total_mesh_triangles +=
      mesh->triangle_range_->prev_frame_index_count * particle_info_list->PeekParticleInfoList().size();

  return true;
}

bool RenderInstanceStorage::RegisterRenderInstance(const std::shared_ptr<Scene>& target_scene, const Entity& entity,
                                                   const Handle& renderer_handle,
                                                   const std::shared_ptr<Material>& material, int* out_material_index) {
  return RegisterRenderInstance(target_scene, entity, renderer_handle, material, {}, out_material_index);
}

bool RenderInstanceStorage::RegisterRenderInstance(const std::shared_ptr<Scene>& target_scene, const Entity& entity,
                                                   const Handle& renderer_handle,
                                                   const std::shared_ptr<Material>& material,
                                                   const DdgiExternalGeometry& ddgi_geometry, int* out_material_index) {
  if (!material)
    return false;
  const auto gt = target_scene->GetDataComponent<GlobalTransform>(entity);
  const auto material_data = BuildMaterialGltfData(*material);
  const auto render_instance = std::make_shared<ExternalRenderInstance>();
  render_instance->command_type = RenderInstanceType::Unknown;
  render_instance->owner = entity;
  render_instance->model = gt;
  render_instance->entity_handle = target_scene->GetEntityHandle(entity);
  render_instance->renderer_handle = renderer_handle;
  render_instance->material = material;
  render_instance->cast_shadow = false;
  render_instance->alpha_tested_shadow = RequiresAlphaTestedShadow(material_data.shade_material);
  render_instance->ddgi_geometry = ddgi_geometry;
  render_instance->geometry_version = ddgi_geometry.IsValid() ? ddgi_geometry.geometry_version : 0;
  render_instance->material_version = material->GetVersion();
  render_instance->material_index = RegisterMaterial(material, material_data);
  render_instance->entity_selected = target_scene->IsEntityAncestorSelected(entity);
  render_instance->line_width = material->draw_settings.line_width;
  render_instance->cull_mode = material->draw_settings.cull_mode;
  render_instance->polygon_mode = material->draw_settings.polygon_mode;

  if (out_material_index) {
    *out_material_index = render_instance->material_index;
  }
  external_render_instances->Register(render_instance);

  return true;
}

int RenderInstanceStorage::RegisterMaterial(const std::shared_ptr<Material>& material) {
  if (!material)
    return -1;
  return RegisterMaterial(material, BuildMaterialGltfData(*material));
}

void RenderInstanceStorage::BuildFromScene(const RenderSettings& render_settings, const std::shared_ptr<Scene>& scene,
                                           Bound& world_bound, const bool include_editor_cameras) {
  this->render_settings = render_settings;
  render_info_block.Apply(this->render_settings);
  CollectEnvironment(scene);
  if (include_editor_cameras) {
    CollectEditorCameras(scene, cameras);
  }
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
  mesh_top_level_acceleration_structure.reset();
  if (!deferred_render_instances->Empty() || !deferred_instanced_render_instances->Empty() ||
      !deferred_skinned_render_instances->Empty() || !forward_render_instances->Empty() ||
      !forward_instanced_render_instances->Empty() || !forward_skinned_render_instances->Empty() ||
      !transparent_render_instances->Empty() || !transparent_instanced_render_instances->Empty() ||
      !transparent_skinned_render_instances->Empty() || external_render_instances->HasDdgiRayTracingGeometry()) {
    auto acceleration_structure = std::make_shared<TopLevelAccelerationStructure>(scene, *this);
    if (acceleration_structure->GetVkAccelerationStructure() != VK_NULL_HANDLE) {
      mesh_top_level_acceleration_structure = acceleration_structure;
    }
  }
}

bool RenderInstanceStorage::RegisterEntity(const std::shared_ptr<Scene>& target_scene, const Entity& owner,
                                           const std::shared_ptr<StrandsRenderer>& strands_renderer,
                                           glm::vec3& min_bound, glm::vec3& max_bound) {
  auto material = strands_renderer->material.Get<Material>();
  auto strands = strands_renderer->strands.Get<Strands>();
  if (!strands_renderer->IsEnabled() || !material || !strands || !strands->strand_meshlet_range_ ||
      !strands->segment_range_)
    return false;
  if (strands->segment_range_->prev_frame_index_count == 0 || strands->strand_meshlet_range_->prev_frame_range == 0)
    return false;
  auto gt = target_scene->GetDataComponent<GlobalTransform>(owner);
  auto ltw = gt.value;
  auto mesh_bound = strands->bound_;
  mesh_bound.ApplyTransform(ltw);
  glm::vec3 center = mesh_bound.Center();

  glm::vec3 size = mesh_bound.Size();
  min_bound = glm::vec3((glm::min)(min_bound.x, center.x - size.x), (glm::min)(min_bound.y, center.y - size.y),
                        (glm::min)(min_bound.z, center.z - size.z));
  max_bound = glm::vec3(glm::max(max_bound.x, center.x + size.x), glm::max(max_bound.y, center.y + size.y),
                        glm::max(max_bound.z, center.z + size.z));

  const auto material_data = BuildMaterialGltfData(*material);

  const auto render_instance = std::make_shared<StrandsRenderInstance>();
  render_instance->command_type = RenderInstanceType::FromRenderer;
  render_instance->owner = owner;
  render_instance->entity_handle = target_scene->GetEntityHandle(owner);
  render_instance->renderer_handle = strands_renderer->GetHandle();
  render_instance->model = gt;
  render_instance->strands = strands;
  render_instance->material = material;
  render_instance->cast_shadow = strands_renderer->cast_shadow;
  render_instance->alpha_tested_shadow = RequiresAlphaTestedShadow(material_data.shade_material);
  render_instance->world_bound = mesh_bound;
  render_instance->geometry_version = strands->GetVersion();
  render_instance->material_version = material->GetVersion();
  render_instance->material_index = RegisterMaterial(material, material_data);
  render_instance->entity_selected = target_scene->IsEntityAncestorSelected(owner);
  render_instance->line_width = material->draw_settings.line_width;
  render_instance->cull_mode = material->draw_settings.cull_mode;
  render_instance->polygon_mode = material->draw_settings.polygon_mode;

  if (UsesTransparentRasterPass(*material, material_data.shade_material)) {
    transparent_strands_render_instances->Register(render_instance);
  } else {
    deferred_strands_render_instances->Register(render_instance);
  }

  total_strands_segments += strands->segment_range_->prev_frame_index_count;
  return true;
}

bool RenderInstanceStorage::RegisterEntity(const std::shared_ptr<Scene>& target_scene, const Entity& owner,
                                           const std::shared_ptr<GaussianSplatRenderer>& gaussian_splat_renderer,
                                           glm::vec3& min_bound, glm::vec3& max_bound) {
  auto gaussian_splat = gaussian_splat_renderer->gaussian_splat.Get<GaussianSplat>();
  if (!gaussian_splat_renderer->IsEnabled() || !gaussian_splat || gaussian_splat->Empty())
    return false;

  auto gt = target_scene->GetDataComponent<GlobalTransform>(owner);
  auto mesh_bound = Bound();
  mesh_bound.min = gaussian_splat->GetMinBound();
  mesh_bound.max = gaussian_splat->GetMaxBound();
  mesh_bound.ApplyTransform(gt.value);
  glm::vec3 center = mesh_bound.Center();

  glm::vec3 size = mesh_bound.Size();
  min_bound = glm::vec3((glm::min)(min_bound.x, center.x - size.x), (glm::min)(min_bound.y, center.y - size.y),
                        (glm::min)(min_bound.z, center.z - size.z));
  max_bound = glm::vec3(glm::max(max_bound.x, center.x + size.x), glm::max(max_bound.y, center.y + size.y),
                        glm::max(max_bound.z, center.z + size.z));

  const auto render_instance = std::make_shared<GaussianSplatRenderInstance>();
  (void)gaussian_splat->EnsureGpuData();
  render_instance->command_type = RenderInstanceType::FromRenderer;
  render_instance->owner = owner;
  render_instance->entity_handle = target_scene->GetEntityHandle(owner);
  render_instance->renderer_handle = gaussian_splat_renderer->GetHandle();
  render_instance->model = gt;
  render_instance->gaussian_splat = gaussian_splat;
  render_instance->world_bound = mesh_bound;
  render_instance->geometry_version = gaussian_splat->GetGpuDataRevision();
  render_instance->entity_selected = target_scene->IsEntityAncestorSelected(owner);
  render_instance->opacity_scale = gaussian_splat_renderer->opacity_scale;
  render_instance->sh_degree = gaussian_splat_renderer->sh_degree;
  render_instance->sort_mode = gaussian_splat_renderer->sort_mode;
  render_instance->depth_mode = gaussian_splat_renderer->depth_mode;
  render_instance->raster_mode = gaussian_splat_renderer->raster_mode;

  gaussian_splat_render_instances->Register(render_instance);
  total_gaussian_splats += gaussian_splat->GetSplatCount();
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
  if (mesh->triangle_range_->prev_frame_index_count == 0 || mesh->meshlet_range_->prev_frame_range == 0)
    return false;

  auto gt = target_scene->GetDataComponent<GlobalTransform>(owner);
  auto ltw = gt.value;
  auto mesh_bound = mesh->GetBound();
  mesh_bound.ApplyTransform(ltw);
  glm::vec3 center = mesh_bound.Center();

  glm::vec3 size = mesh_bound.Size();
  min_bound = glm::vec3((glm::min)(min_bound.x, center.x - size.x), (glm::min)(min_bound.y, center.y - size.y),
                        (glm::min)(min_bound.z, center.z - size.z));
  max_bound = glm::vec3(glm::max(max_bound.x, center.x + size.x), glm::max(max_bound.y, center.y + size.y),
                        glm::max(max_bound.z, center.z + size.z));

  const auto material_data = BuildMaterialGltfData(*material);
  const auto render_instance = std::make_shared<MeshRenderInstance>();
  render_instance->command_type = RenderInstanceType::FromRenderer;
  render_instance->owner = owner;
  render_instance->mesh = mesh;
  render_instance->material = material;
  render_instance->model = gt;
  render_instance->entity_handle = target_scene->GetEntityHandle(owner);
  render_instance->renderer_handle = mesh_renderer->GetHandle();
  render_instance->cast_shadow = mesh_renderer->cast_shadow;
  render_instance->alpha_tested_shadow = RequiresAlphaTestedShadow(material_data.shade_material);
  render_instance->world_bound = mesh_bound;
  render_instance->geometry_version = mesh->GetVersion();
  render_instance->material_version = material->GetVersion();
  render_instance->material_index = RegisterMaterial(material, material_data);
  render_instance->line_width = material->draw_settings.line_width;
  render_instance->cull_mode = material->draw_settings.cull_mode;
  render_instance->polygon_mode = material->draw_settings.polygon_mode;
  render_instance->entity_selected = target_scene->IsEntityAncestorSelected(owner);
  if (UsesTransparentRasterPass(*material, material_data.shade_material)) {
    transparent_render_instances->Register(render_instance);
  } else {
    deferred_render_instances->Register(render_instance);
    if (ApplicationContext::Get().GetLayer<RenderLayer>()) {
      AppendMeshIndirectCommands(mesh_draw_indexed_indirect_commands, mesh_draw_mesh_tasks_indirect_commands,
                                 mesh->triangle_range_->prev_frame_offset,
                                 mesh->triangle_range_->prev_frame_index_count, mesh->meshlet_range_->prev_frame_range);
    }
    total_mesh_triangles += mesh->triangle_range_->prev_frame_index_count;
  }
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
  if (skinned_mesh->skinned_triangle_range_->prev_frame_index_count == 0 ||
      skinned_mesh->skinned_meshlet_range_->prev_frame_range == 0)
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
  max_bound = glm::vec3(glm::max(max_bound.x, center.x + size.x), glm::max(max_bound.y, center.y + size.y),
                        glm::max(max_bound.z, center.z + size.z));

  const auto material_data = BuildMaterialGltfData(*material);
  const auto render_instance = std::make_shared<SkinnedMeshRenderInstance>();
  render_instance->command_type = RenderInstanceType::FromRenderer;
  render_instance->owner = owner;
  render_instance->entity_handle = target_scene->GetEntityHandle(owner);
  render_instance->renderer_handle = skinned_mesh_renderer->GetHandle();
  render_instance->model = gt;
  render_instance->skinned_mesh = skinned_mesh;
  render_instance->material = material;
  render_instance->cast_shadow = skinned_mesh_renderer->cast_shadow;
  render_instance->alpha_tested_shadow = RequiresAlphaTestedShadow(material_data.shade_material);
  render_instance->world_bound = mesh_bound;
  render_instance->bone_matrices = skinned_mesh_renderer->bone_matrices;
  render_instance->geometry_version = skinned_mesh->GetVersion();
  render_instance->ray_tracing_geometry_version = skinned_mesh_renderer->ray_tracing_geometry_version_;
  render_instance->ray_tracing_triangle_range = skinned_mesh_renderer->ray_tracing_triangle_range_;
  render_instance->ray_tracing_blas = skinned_mesh_renderer->ray_tracing_blas_;
  render_instance->material_version = material->GetVersion();
  render_instance->material_index = RegisterMaterial(material, material_data);
  render_instance->bone_matrices_version = skinned_mesh_renderer->bone_matrices->GetVersion();
  render_instance->entity_selected = target_scene->IsEntityAncestorSelected(owner);
  render_instance->line_width = material->draw_settings.line_width;
  render_instance->cull_mode = material->draw_settings.cull_mode;
  render_instance->polygon_mode = material->draw_settings.polygon_mode;

  if (UsesTransparentRasterPass(*material, material_data.shade_material)) {
    transparent_skinned_render_instances->Register(render_instance);
  } else {
    deferred_skinned_render_instances->Register(render_instance);
  }

  total_skinned_mesh_triangles += skinned_mesh->skinned_triangle_range_->prev_frame_index_count;
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
  if (mesh->triangle_range_->prev_frame_index_count == 0 || mesh->meshlet_range_->prev_frame_range == 0)
    return false;
  auto gt = target_scene->GetDataComponent<GlobalTransform>(owner);
  auto ltw = gt.value;
  auto mesh_bound = mesh->GetBound();
  mesh_bound.ApplyTransform(ltw);
  glm::vec3 center = mesh_bound.Center();

  glm::vec3 size = mesh_bound.Size();
  min_bound = glm::vec3((glm::min)(min_bound.x, center.x - size.x), (glm::min)(min_bound.y, center.y - size.y),
                        (glm::min)(min_bound.z, center.z - size.z));

  max_bound = glm::vec3(glm::max(max_bound.x, center.x + size.x), glm::max(max_bound.y, center.y + size.y),
                        glm::max(max_bound.z, center.z + size.z));

  const auto material_data = BuildMaterialGltfData(*material);

  const auto render_instance = std::make_shared<InstancedRenderInstance>();
  render_instance->command_type = RenderInstanceType::FromRenderer;
  render_instance->model = gt;
  render_instance->owner = owner;
  render_instance->entity_handle = target_scene->GetEntityHandle(owner);
  render_instance->renderer_handle = particles->GetHandle();
  render_instance->mesh = mesh;
  render_instance->material = material;
  render_instance->cast_shadow = particles->cast_shadow;
  render_instance->alpha_tested_shadow = RequiresAlphaTestedShadow(material_data.shade_material);
  render_instance->particle_infos = particle_info_list;
  render_instance->geometry_version = mesh->GetVersion();
  render_instance->material_version = material->GetVersion();
  render_instance->material_index = RegisterMaterial(material, material_data);
  render_instance->particle_info_list_version = particle_info_list->GetVersion();
  render_instance->entity_selected = target_scene->IsEntityAncestorSelected(owner);
  render_instance->line_width = material->draw_settings.line_width;
  render_instance->cull_mode = material->draw_settings.cull_mode;
  render_instance->polygon_mode = material->draw_settings.polygon_mode;

  if (UsesTransparentRasterPass(*material, material_data.shade_material)) {
    transparent_instanced_render_instances->Register(render_instance);
  } else {
    deferred_instanced_render_instances->Register(render_instance);
  }

  total_instanced_mesh_triangles +=
      mesh->triangle_range_->prev_frame_index_count * particle_info_list->PeekParticleInfoList().size();
  return true;
}

int RenderInstanceStorage::RegisterMaterial(const std::shared_ptr<Material>& material,
                                            const GltfMaterialData& material_data) {
  if (!material)
    return -1;
  const auto handle = material->GetHandle();
  const auto search = material_indices_.find(handle);
  if (search == material_indices_.end()) {
    const int index = static_cast<int>(gltf_material_cache_.GetShadeMaterials().size());
    material_indices_[handle] = index;
    const auto gltf_material_index = gltf_material_cache_.Append(material_data);
    if (gltf_material_index != static_cast<uint32_t>(index)) {
      throw std::runtime_error("glTF material cache drifted from render material indices.");
    }
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
