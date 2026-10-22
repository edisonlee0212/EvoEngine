#include "GpuTreeMeshRenderer.hpp"

#include "Application.hpp"
#include "BasicBarkDescriptor.hpp"
#include "BasicFoliageDescriptor.hpp"
#include "BranchRenderer.hpp"
#include "DynamicTreeSkeleton.hpp"
#include "FoliageRenderer.hpp"
#include "GraphicsPipeline.hpp"
#include "Material.hpp"
#include "Platform/Platform.hpp"
#include "Shader.hpp"
#include "Tree.hpp"

#include <cstring>
#include <unordered_set>

using namespace eco_sys_lab_package;

namespace {
struct TreeMeshPushConstant {
  uint32_t node_base = 0;
  uint32_t node_count = 0;
  uint32_t camera_index = 0;
  uint32_t pass_index = 0;
  uint32_t light_index = 0;
  uint32_t light_face = 0;
  uint32_t padding0 = 0;
  uint32_t geometry_pass = 0;
};

std::shared_ptr<GraphicsPipeline> MakePipeline(const std::shared_ptr<Shader>& task, const std::shared_ptr<Shader>& mesh,
                                               bool shadow, const std::shared_ptr<DescriptorSetLayout>& layout,
                                               const char* fragment_name) {
  auto pipeline = std::make_shared<GraphicsPipeline>();
  pipeline->task_shader = task;
  pipeline->mesh_shader = mesh;
  pipeline->fragment_shader =
      Shader::CreateTemporary(ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
                              std::filesystem::path("./EcoSysLabResources/Shaders/Graphics/Fragment") / fragment_name);
  pipeline->geometry_type = GeometryType::Mesh;
  const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>();
  pipeline->descriptor_set_layouts = {
      render_layer->GetPerFrameDescriptorSetLayout(), render_layer->GetEmptyDescriptorSetLayout(),
      shadow ? render_layer->GetEmptyDescriptorSetLayout() : render_layer->GetLightingDescriptorSetLayout(), layout};
  pipeline->depth_attachment_format =
      shadow ? Platform::Constants::shadow_map : Platform::Constants::render_texture_depth;
  pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
  if (!shadow) {
    pipeline->color_attachment_formats = {
        Platform::Constants::g_buffer_attribute, Platform::Constants::g_buffer_attribute,
        Platform::Constants::g_buffer_attribute, Platform::Constants::g_buffer_attribute,
        Platform::Constants::g_buffer_utility};
  }
  auto& push_range = pipeline->push_constant_ranges.emplace_back();
  push_range.size = sizeof(TreeMeshPushConstant);
  push_range.offset = 0;
  push_range.stageFlags = VK_SHADER_STAGE_ALL;
  pipeline->Initialize();
  return pipeline;
}
}  // namespace

void GpuTreeMeshRenderer::Initialize() {
  if (layout_ || !Platform::MeshShaderEnabled())
    return;
  layout_ = std::make_shared<DescriptorSetLayout>();
  layout_->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                                 VK_SHADER_STAGE_TASK_BIT_EXT | VK_SHADER_STAGE_MESH_BIT_EXT, 0);
  layout_->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_MESH_BIT_EXT, 0);
  layout_->PushDescriptorBinding(2, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                                 VK_SHADER_STAGE_TASK_BIT_EXT | VK_SHADER_STAGE_MESH_BIT_EXT, 0);
  layout_->Initialize();
  VkBufferCreateInfo buffer_info{};
  buffer_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  buffer_info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
  buffer_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  buffer_info.size = sizeof(GpuTreeNode);
  VmaAllocationCreateInfo allocation_info{};
  allocation_info.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
  VmaAllocationCreateInfo instance_allocation_info{};
  instance_allocation_info.usage = VMA_MEMORY_USAGE_AUTO_PREFER_HOST;
  instance_allocation_info.flags =
      VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT | VMA_ALLOCATION_CREATE_MAPPED_BIT;
  instance_allocation_info.requiredFlags = VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
  for (uint32_t i = 0; i < Platform::GetMaxFramesInFlight(); ++i) {
    node_buffers_.emplace_back(std::make_shared<Buffer>(buffer_info, allocation_info));
    buffer_info.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
    buffer_info.size = sizeof(GpuTreeInstanceIndices);
    instance_index_buffers_.emplace_back(std::make_shared<Buffer>(buffer_info, instance_allocation_info));
    buffer_info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
    buffer_info.size = sizeof(GpuTreeFoliageParams);
    foliage_params_buffers_.emplace_back(std::make_shared<Buffer>(buffer_info, allocation_info));
    buffer_info.size = sizeof(GpuTreeNode);
    descriptor_sets_.emplace_back(std::make_shared<DescriptorSet>(layout_));
    tree_handles_.emplace_back();
    instance_indices_ready_.push_back(false);
  }
  const auto task = Shader::CreateTemporary(ShaderType::Task, Platform::GetShaderGlobalDefines(),
                                            "./EcoSysLabResources/Shaders/Graphics/Task/TreeMesh.slang");
  const auto mesh = Shader::CreateTemporary(ShaderType::Mesh, Platform::GetShaderGlobalDefines(),
                                            "./EcoSysLabResources/Shaders/Graphics/Mesh/TreeMesh.slang");
  camera_pipeline_ = MakePipeline(task, mesh, false, layout_, "TreeMesh.slang");
  shadow_pipeline_ = MakePipeline(task, mesh, true, layout_, "Empty.slang");
  const auto foliage_task = Shader::CreateTemporary(ShaderType::Task, Platform::GetShaderGlobalDefines(),
                                                    "./EcoSysLabResources/Shaders/Graphics/Task/TreeFoliage.slang");
  const auto foliage_mesh = Shader::CreateTemporary(ShaderType::Mesh, Platform::GetShaderGlobalDefines(),
                                                    "./EcoSysLabResources/Shaders/Graphics/Mesh/TreeFoliage.slang");
  foliage_opaque_pipeline_ = MakePipeline(foliage_task, foliage_mesh, false, layout_, "TreeFoliage.slang");
  foliage_masked_pipeline_ = MakePipeline(foliage_task, foliage_mesh, false, layout_, "TreeFoliageMasked.slang");
  foliage_opaque_shadow_pipeline_ = MakePipeline(foliage_task, foliage_mesh, true, layout_, "Empty.slang");
  foliage_masked_shadow_pipeline_ =
      MakePipeline(foliage_task, foliage_mesh, true, layout_, "TreeFoliageMaskedShadow.slang");

  default_foliage_profile_ = AssetManager::CreateTemporaryAsset<SubsurfaceProfile>();
  default_foliage_profile_->albedo = {0.75f, 0.95f, 0.45f};
  default_foliage_profile_->scattering_distance = {0.002f, 0.004f, 0.001f};
  default_foliage_material_ = AssetManager::CreateTemporaryAsset<Material>();
  auto& shade = default_foliage_material_->material_data.shade_material;
  shade.pbr_base_color_factor = {152.0f / 255.0f, 203.0f / 255.0f, 0.0f, 1.0f};
  shade.pbr_roughness_factor = 0.9f;
  shade.pbr_metallic_factor = 0.0f;
  shade.subsurface_weight = 0.5f;
  shade.subsurface_thickness = 0.0005f;
  shade.subsurface_mode = static_cast<uint32_t>(SubsurfaceMode::ThinSurface);
  shade.double_sided = 1;
  default_foliage_material_->subsurface_profile = default_foliage_profile_;
  default_foliage_material_->MarkDirty();
}

bool GpuTreeMeshRenderer::Prepare() {
  Initialize();
  return camera_pipeline_ && camera_pipeline_->Initialized() && shadow_pipeline_ && shadow_pipeline_->Initialized() &&
         foliage_opaque_pipeline_ && foliage_opaque_pipeline_->Initialized() && foliage_masked_pipeline_ &&
         foliage_masked_pipeline_->Initialized() && foliage_opaque_shadow_pipeline_ &&
         foliage_opaque_shadow_pipeline_->Initialized() && foliage_masked_shadow_pipeline_ &&
         foliage_masked_shadow_pipeline_->Initialized();
}

void GpuTreeMeshRenderer::Register(const std::shared_ptr<Scene>& scene) {
  const auto branch_entities = scene->UnsafeGetPrivateComponentOwnersList<BranchRenderer>();
  const auto foliage_entities = scene->UnsafeGetPrivateComponentOwnersList<FoliageRenderer>();
  if ((!branch_entities || branch_entities->empty()) && (!foliage_entities || foliage_entities->empty()))
    return;
  if (!Prepare())
    return;
  std::unordered_set<Entity, Entity> tree_entities;
  if (branch_entities)
    tree_entities.insert(branch_entities->begin(), branch_entities->end());
  if (foliage_entities)
    tree_entities.insert(foliage_entities->begin(), foliage_entities->end());
  const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>();
  const auto storage = render_layer->GetCurrentRenderInstanceStorage();
  std::vector<GpuTreeNode> nodes;
  std::vector<GpuTreeFoliageParams> foliage_params;
  bool any_branch = false;
  bool any_foliage_opaque = false;
  bool any_foliage_masked = false;
  const auto frame = Platform::GetCurrentFrameIndex();
  auto& handles = tree_handles_[frame];
  handles.clear();
  instance_indices_ready_[frame] = false;
  auto scene_bound = scene->GetBound();
  for (const auto& entity : tree_entities) {
    if (!scene->IsEntityEnabled(entity) || !scene->HasPrivateComponent<Tree>(entity))
      continue;
    const auto tree = scene->GetOrSetPrivateComponent<Tree>(entity).lock();
    if (!tree || !tree->IsEnabled())
      continue;
    const auto branch_renderer = scene->HasPrivateComponent<BranchRenderer>(entity)
                                     ? scene->GetOrSetPrivateComponent<BranchRenderer>(entity).lock()
                                     : nullptr;
    const auto foliage_renderer = scene->HasPrivateComponent<FoliageRenderer>(entity)
                                      ? scene->GetOrSetPrivateComponent<FoliageRenderer>(entity).lock()
                                      : nullptr;
    bool branch_enabled = branch_renderer && branch_renderer->IsEnabled();
    bool foliage_enabled = foliage_renderer && foliage_renderer->IsEnabled();
    if (!branch_enabled && !foliage_enabled)
      continue;
    const auto& settings = tree->tree_mesh_generator_settings;
    const auto& shoots = tree->shoot_model.PeekShootSkeleton();
    const auto& roots = tree->root_model.PeekRootSkeleton();
    if (shoots.PeekSortedNodeList().empty() && (!branch_enabled || roots.PeekSortedNodeList().empty()))
      continue;
    std::shared_ptr<BasicBarkDescriptor> bark;
    std::shared_ptr<BasicFoliageDescriptor> foliage_descriptor;
    if (const auto descriptor = tree->tree_descriptor_ref.Get<TreeDescriptor>()) {
      bark = descriptor->bark_descriptor.Get<BasicBarkDescriptor>();
      foliage_descriptor = descriptor->foliage_descriptor.Get<BasicFoliageDescriptor>();
    }
    const auto branch_material = branch_enabled ? branch_renderer->material_ref.Get<Material>() : nullptr;
    branch_enabled = branch_material && ClassifyGltfRasterMaterial(
                                            branch_material->BuildGltfMaterialData().shade_material,
                                            branch_material->draw_settings.blending) == GltfRasterMaterialClass::Opaque;
    auto foliage_material = foliage_enabled ? foliage_renderer->material_ref.Get<Material>() : nullptr;
    if (foliage_enabled && !foliage_material && foliage_descriptor)
      foliage_material = foliage_descriptor->leaf_material_ref.Get<Material>();
    if (foliage_enabled && !foliage_material)
      foliage_material = default_foliage_material_;
    GltfRasterMaterialClass foliage_class = GltfRasterMaterialClass::Forward;
    if (foliage_enabled) {
      foliage_class = ClassifyGltfRasterMaterial(foliage_material->BuildGltfMaterialData().shade_material,
                                                 foliage_material->draw_settings.blending);
      foliage_enabled = foliage_class != GltfRasterMaterialClass::Forward;
    }
    if (!branch_enabled && !foliage_enabled)
      continue;
    TreeHandles tree_handles;
    int branch_material_index = -1;
    int foliage_material_index = -1;
    if (branch_enabled) {
      tree_handles.branch = branch_renderer->GetHandle();
      tree_handles.has_branch =
          storage->RegisterRenderInstance(scene, entity, tree_handles.branch, branch_material, &branch_material_index);
    }
    if (foliage_enabled) {
      tree_handles.foliage = foliage_renderer->GetHandle();
      tree_handles.has_foliage = storage->RegisterRenderInstance(scene, entity, tree_handles.foliage, foliage_material,
                                                                 &foliage_material_index);
    }
    if (!tree_handles.has_branch && !tree_handles.has_foliage)
      continue;
    const size_t base = nodes.size();
    const auto tree_index = static_cast<uint32_t>(handles.size());
    const auto physics = scene->HasPrivateComponent<DynamicTreeSkeleton>(entity)
                             ? scene->GetOrSetPrivateComponent<DynamicTreeSkeleton>(entity).lock()
                             : nullptr;
    const auto* physics_pose =
        physics && physics->IsEnabled() && physics->HasPhysics(*tree) ? &tree->shoot_model.PeekPhysicsPose() : nullptr;
    if (!shoots.PeekSortedNodeList().empty())
      AppendGpuTreeNodes(shoots, settings, bark.get(), tree_index, true, nodes, physics_pose);
    if (tree_handles.has_branch && !roots.PeekSortedNodeList().empty())
      AppendGpuTreeNodes(roots, settings, bark.get(), tree_index, false, nodes);
    handles.push_back(tree_handles);
    const BasicFoliageDescriptor fallback_descriptor;
    const auto& foliage = foliage_descriptor ? *foliage_descriptor : fallback_descriptor;
    float tree_size = glm::length(shoots.max - shoots.min);
    if (physics_pose && !shoots.PeekSortedNodeList().empty()) {
      glm::vec3 minimum = glm::vec3(nodes[base].position_thickness);
      glm::vec3 maximum = minimum;
      for (size_t i = base; i < base + shoots.PeekSortedNodeList().size(); ++i) {
        const glm::vec3 start = glm::vec3(nodes[i].position_thickness);
        const glm::vec3 end = start + glm::vec3(nodes[i].direction_length) * nodes[i].direction_length.w;
        minimum = glm::min(minimum, glm::min(start, end));
        maximum = glm::max(maximum, glm::max(start, end));
      }
      tree_size = glm::length(maximum - minimum);
    }
    GpuTreeFoliageParams params{};
    params.size_count = {foliage.leaf_size.x, foliage.leaf_size.y, tree_size, static_cast<float>(foliage.leaf_count)};
    params.filters = {foliage.max_node_thickness, foliage.min_root_distance, foliage.max_end_distance,
                      foliage.rotation_variance};
    params.distributions = {foliage.stem_length.mean, foliage.stem_length.deviation, foliage.branching_angle.mean,
                            foliage.branching_angle.deviation};
    params.tropism = {foliage.horizontal_tropism, foliage.gravitropism, 0.0f, 0.0f};
    params.flags.x = tree_handles.has_foliage ? (foliage_class == GltfRasterMaterialClass::Masked ? 2u : 1u) : 0u;
    foliage_params.push_back(params);
    any_branch |= tree_handles.has_branch;
    any_foliage_opaque |= tree_handles.has_foliage && params.flags.x == 1u;
    any_foliage_masked |= tree_handles.has_foliage && params.flags.x == 2u;
    const auto model = scene->GetDataComponent<GlobalTransform>(entity).value;
    const float max_scale = glm::max(glm::length(glm::vec3(model[0])),
                                     glm::max(glm::length(glm::vec3(model[1])), glm::length(glm::vec3(model[2]))));
    for (size_t i = base; i < nodes.size(); ++i) {
      nodes[i].foliage_meta.z = entity.GetIndex() ^ (entity.GetVersion() * 0x9e3779b9u);
      if (tree_handles.has_branch)
        nodes[i].links.z = static_cast<uint32_t>(branch_material_index);
      if (tree_handles.has_foliage && nodes[i].foliage_meta.y)
        nodes[i].links.w = static_cast<uint32_t>(foliage_material_index);
      const auto& node = nodes[i];
      const glm::vec3 start = glm::vec3(model * glm::vec4(glm::vec3(node.position_thickness), 1.0f));
      const glm::vec3 end = glm::vec3(model * glm::vec4(glm::vec3(node.position_thickness) +
                                                            glm::vec3(node.direction_length) * node.direction_length.w,
                                                        1.0f));
      const glm::vec3 padding(
          (node.position_thickness.w +
           (tree_handles.has_foliage ? glm::max(params.size_count.x, params.size_count.y) * tree_size * 0.1f : 0.0f)) *
          max_scale);
      scene_bound.min = glm::min(scene_bound.min, glm::min(start, end) - padding);
      scene_bound.max = glm::max(scene_bound.max, glm::max(start, end) + padding);
    }
  }
  scene->SetBound(scene_bound);
  node_count_ = static_cast<uint32_t>(nodes.size());
  if (!node_count_)
    return;
  BufferUploadBatch upload;
  upload.AddVector(node_buffers_[frame], nodes,
                   {BufferUploadUsage::StorageRead, BufferUploadCapacityPolicy::GrowIfNeeded});
  upload.AddVector(foliage_params_buffers_[frame], foliage_params,
                   {BufferUploadUsage::StorageRead, BufferUploadCapacityPolicy::GrowIfNeeded});
  upload.Record(upload_arena_);
  descriptor_sets_[frame]->UpdateBufferDescriptorBinding(0, node_buffers_[frame], 0);
  instance_index_buffers_[frame]->Resize(
      std::max(sizeof(GpuTreeInstanceIndices), handles.size() * sizeof(GpuTreeInstanceIndices)));
  descriptor_sets_[frame]->UpdateBufferDescriptorBinding(1, instance_index_buffers_[frame], 0);
  descriptor_sets_[frame]->UpdateBufferDescriptorBinding(2, foliage_params_buffers_[frame], 0);
  const auto self = shared_from_this();
  if (any_branch)
    render_layer->RawOpaqueRenderingAllCameras([self](VkCommandBuffer command_buffer, const auto& attachments,
                                                      const RenderLayer::DeferredRenderingView& view) {
      return self->Draw(command_buffer, self->camera_pipeline_, view.viewport, 0, 0, view.camera_index, 0, 0,
                        VK_CULL_MODE_NONE);
    });
  if (any_branch)
    render_layer->RenderOpaqueToPointLightShadowMap([self](VkCommandBuffer command_buffer, const auto& view) {
      return self->Draw(command_buffer, self->shadow_pipeline_, view.viewport, 0, 1, 0, view.light_index,
                        view.face_index, VK_CULL_MODE_NONE);
    });
  if (any_branch)
    render_layer->RenderOpaqueToSpotLightShadowMap([self](VkCommandBuffer command_buffer, const auto& view) {
      return self->Draw(command_buffer, self->shadow_pipeline_, view.viewport, 0, 2, 0, view.light_index, 0,
                        VK_CULL_MODE_NONE);
    });
  if (any_branch)
    render_layer->RenderOpaqueToDirectionalLightShadowMap([self](VkCommandBuffer command_buffer, const auto& view) {
      return self->Draw(command_buffer, self->shadow_pipeline_, view.viewport, 0, 3, 0, view.light_index,
                        view.split_index, VK_CULL_MODE_NONE);
    });
  if (any_foliage_opaque) {
    render_layer->RawOpaqueRenderingAllCameras([self](VkCommandBuffer command_buffer, const auto&, const auto& view) {
      return self->Draw(command_buffer, self->foliage_opaque_pipeline_, view.viewport, 1, 0, view.camera_index, 0, 0,
                        VK_CULL_MODE_NONE);
    });
    render_layer->RenderOpaqueToPointLightShadowMap([self](VkCommandBuffer cb, const auto& view) {
      return self->Draw(cb, self->foliage_opaque_shadow_pipeline_, view.viewport, 1, 1, 0, view.light_index,
                        view.face_index, VK_CULL_MODE_NONE);
    });
    render_layer->RenderOpaqueToSpotLightShadowMap([self](VkCommandBuffer cb, const auto& view) {
      return self->Draw(cb, self->foliage_opaque_shadow_pipeline_, view.viewport, 1, 2, 0, view.light_index, 0,
                        VK_CULL_MODE_NONE);
    });
    render_layer->RenderOpaqueToDirectionalLightShadowMap([self](VkCommandBuffer cb, const auto& view) {
      return self->Draw(cb, self->foliage_opaque_shadow_pipeline_, view.viewport, 1, 3, 0, view.light_index,
                        view.split_index, VK_CULL_MODE_NONE);
    });
  }
  if (any_foliage_masked) {
    render_layer->AlphaMaskedRenderingAllCameras([self](VkCommandBuffer cb, const auto&, const auto& view) {
      return self->Draw(cb, self->foliage_masked_pipeline_, view.viewport, 2, 0, view.camera_index, 0, 0,
                        VK_CULL_MODE_NONE);
    });
    render_layer->RenderAlphaMaskedToPointLightShadowMap([self](VkCommandBuffer cb, const auto& view) {
      return self->Draw(cb, self->foliage_masked_shadow_pipeline_, view.viewport, 2, 1, 0, view.light_index,
                        view.face_index, VK_CULL_MODE_NONE);
    });
    render_layer->RenderAlphaMaskedToSpotLightShadowMap([self](VkCommandBuffer cb, const auto& view) {
      return self->Draw(cb, self->foliage_masked_shadow_pipeline_, view.viewport, 2, 2, 0, view.light_index, 0,
                        VK_CULL_MODE_NONE);
    });
    render_layer->RenderAlphaMaskedToDirectionalLightShadowMap([self](VkCommandBuffer cb, const auto& view) {
      return self->Draw(cb, self->foliage_masked_shadow_pipeline_, view.viewport, 2, 3, 0, view.light_index,
                        view.split_index, VK_CULL_MODE_NONE);
    });
  }
}

uint32_t GpuTreeMeshRenderer::Draw(VkCommandBuffer command_buffer, const std::shared_ptr<GraphicsPipeline>& pipeline,
                                   const glm::ivec4& viewport, uint32_t geometry_pass, uint32_t pass, uint32_t camera,
                                   uint32_t light, uint32_t face, VkCullModeFlags cull_mode) {
  if (!node_count_)
    return 0;
  const auto frame = Platform::GetCurrentFrameIndex();
  if (!instance_indices_ready_[frame]) {
    const auto storage = ApplicationContext::Get().GetLayer<RenderLayer>()->GetCurrentRenderInstanceStorage();
    std::vector<GpuTreeInstanceIndices> indices;
    indices.reserve(tree_handles_[frame].size());
    for (const auto& handles : tree_handles_[frame])
      indices.push_back(
          {handles.has_branch ? static_cast<uint32_t>(storage->GetRenderInstanceIndex(handles.branch)) : 0u,
           handles.has_foliage ? static_cast<uint32_t>(storage->GetRenderInstanceIndex(handles.foliage)) : 0u});
    std::memcpy(instance_index_buffers_[frame]->GetVmaAllocationInfo().pMappedData, indices.data(),
                indices.size() * sizeof(GpuTreeInstanceIndices));
    instance_indices_ready_[frame] = true;
  }
  pipeline->Bind(command_buffer);
  pipeline->BindDescriptorSet(command_buffer, 0, RenderLayer::GetPerFrameDescriptorSet()->GetVkDescriptorSet());
  pipeline->BindDescriptorSet(command_buffer, 3,
                              descriptor_sets_[Platform::GetCurrentFrameIndex()]->GetVkDescriptorSet());
  pipeline->states.ResetAllStates(pass == 0 ? 5 : 0);
  pipeline->states.SetViewportScissor(viewport);
  pipeline->states.cull_mode = cull_mode;
  pipeline->states.ApplyAllStates(command_buffer);
  const uint32_t limit = Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxTaskWorkGroupCount[0];
  for (uint32_t base = 0; base < node_count_; base += limit) {
    TreeMeshPushConstant push{};
    push.node_base = base;
    push.node_count = node_count_;
    push.camera_index = camera;
    push.pass_index = pass;
    push.light_index = light;
    push.light_face = face;
    push.geometry_pass = geometry_pass;
    pipeline->PushConstant(command_buffer, 0, push);
    pipeline->DrawMeshTasks(command_buffer, std::min(limit, node_count_ - base), 1, 1);
  }
  return node_count_;
}
