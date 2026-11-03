#include "GpuTreeMeshRenderer.hpp"

#include <limits>
#include "Application.hpp"
#include "BasicBarkDescriptor.hpp"
#include "BasicFoliageDescriptor.hpp"
#include "BranchRenderer.hpp"
#include "DynamicTreeSkeleton.hpp"
#include "FoliageRenderer.hpp"
#include "GpuTreePhysics.hpp"
#include "GraphicsPipeline.hpp"
#include "Material.hpp"
#include "Platform/Platform.hpp"
#include "Profiler.hpp"
#include "Shader.hpp"
#include "Tree.hpp"

#include <cstdlib>
#include <cstring>
#include <unordered_set>

using namespace eco_sys_lab_package;

namespace {
using TreeMeshPushConstant = GpuTreeDrawParameters;

using DeformPushConstant = GpuTreeDeformParameters;

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
                                 VK_SHADER_STAGE_TASK_BIT_EXT | VK_SHADER_STAGE_MESH_BIT_EXT |
                                     VK_SHADER_STAGE_FRAGMENT_BIT | VK_SHADER_STAGE_COMPUTE_BIT,
                                 0);
  layout_->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                                 VK_SHADER_STAGE_TASK_BIT_EXT | VK_SHADER_STAGE_MESH_BIT_EXT |
                                     VK_SHADER_STAGE_FRAGMENT_BIT | VK_SHADER_STAGE_COMPUTE_BIT,
                                 0);
  layout_->PushDescriptorBinding(2, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                                 VK_SHADER_STAGE_TASK_BIT_EXT | VK_SHADER_STAGE_MESH_BIT_EXT |
                                     VK_SHADER_STAGE_FRAGMENT_BIT | VK_SHADER_STAGE_COMPUTE_BIT,
                                 0);
  for (uint32_t i = 5; i <= 8; ++i)
    layout_->PushDescriptorBinding(i, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_ALL, 0);
  layout_->Initialize();
  deform_layout_ = std::make_shared<DescriptorSetLayout>();
  for (uint32_t i = 0; i < 5; ++i)
    deform_layout_->PushDescriptorBinding(i, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  deform_layout_->Initialize();
  deform_pipeline_ = std::make_shared<ComputePipeline>();
  deform_pipeline_->compute_shader =
      Shader::CreateTemporary(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                              "./EcoSysLabResources/Shaders/Compute/DeformTreeNodes.slang");
  deform_pipeline_->descriptor_set_layouts = {deform_layout_};
  deform_pipeline_->push_constant_ranges.push_back({VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(DeformPushConstant)});
  deform_pipeline_->Initialize();
  canopy_pipeline_ = std::make_shared<ComputePipeline>();
  canopy_pipeline_->compute_shader =
      Shader::CreateTemporary(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                              "./EcoSysLabResources/Shaders/Compute/TreeCanopyBounds.slang");
  const auto empty = ApplicationContext::Get().GetLayer<RenderLayer>()->GetEmptyDescriptorSetLayout();
  canopy_pipeline_->descriptor_set_layouts = {empty, empty, empty, layout_};
  canopy_pipeline_->Initialize();
  const auto task = Shader::CreateTemporary(ShaderType::Task, Platform::GetShaderGlobalDefines(),
                                            "./EcoSysLabResources/Shaders/Graphics/Task/TreeMesh.slang");
  const auto mesh = Shader::CreateTemporary(ShaderType::Mesh, Platform::GetShaderGlobalDefines(),
                                            "./EcoSysLabResources/Shaders/Graphics/Mesh/TreeMesh.slang");
  const auto& limits = Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext;
  if (std::getenv("EVOENGINE_TREE_GEOMETRY_REFERENCE") ||
      std::min(limits.maxMeshWorkGroupCount[0], limits.maxMeshWorkGroupTotalCount) < 3u * 65535u) {
    camera_pipeline_ = MakePipeline(task, mesh, false, layout_, "TreeMesh.slang");
  } else {
    camera_batch_ = std::max(
        1u,
        std::min(32u, std::min(limits.maxMeshWorkGroupCount[0], limits.maxMeshWorkGroupTotalCount) / (3u * 65535u)));
    const auto batch_task = Shader::CreateTemporary(ShaderType::Task, Platform::GetShaderGlobalDefines(),
                                                    "./EcoSysLabResources/Shaders/Graphics/Task/TreeMeshBatched.slang");
    const auto indexed_mesh =
        Shader::CreateTemporary(ShaderType::Mesh, Platform::GetShaderGlobalDefines(),
                                "./EcoSysLabResources/Shaders/Graphics/Mesh/TreeMeshIndexed.slang");
    camera_pipeline_ = MakePipeline(batch_task, indexed_mesh, false, layout_, "TreeMesh.slang");
    impostor_pipeline_ =
        MakePipeline(batch_task,
                     Shader::CreateTemporary(ShaderType::Mesh, Platform::GetShaderGlobalDefines(),
                                             "./EcoSysLabResources/Shaders/Graphics/Mesh/TreeImpostor.slang"),
                     false, layout_, "TreeImpostor.slang");
  }
  shadow_pipeline_ = MakePipeline(task, mesh, true, layout_, "Empty.slang");
  shadow_reference_ = std::getenv("EVOENGINE_TREE_SHADOW_REFERENCE") != nullptr;
  const auto shadow_defines =
      Platform::GetShaderGlobalDefines() + (shadow_reference_ ? "\n#define EE_TREE_SHADOW_REFERENCE 1\n" : "");
  const auto shadow_task = Shader::CreateTemporary(ShaderType::Task, shadow_defines,
                                                   "./EcoSysLabResources/Shaders/Graphics/Task/TreeShadow.slang");
  const auto shadow_mesh = Shader::CreateTemporary(ShaderType::Mesh, Platform::GetShaderGlobalDefines(),
                                                   "./EcoSysLabResources/Shaders/Graphics/Mesh/TreeShadow.slang");
  directional_shadow_pipeline_ = MakePipeline(shadow_task, shadow_mesh, true, layout_, "TreeShadow.slang");
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
  return canopy_pipeline_ && canopy_pipeline_->Initialized() &&
         (!impostor_pipeline_ || impostor_pipeline_->Initialized()) && deform_pipeline_ &&
         deform_pipeline_->Initialized() && camera_pipeline_ && camera_pipeline_->Initialized() && shadow_pipeline_ &&
         shadow_pipeline_->Initialized() && directional_shadow_pipeline_ &&
         directional_shadow_pipeline_->Initialized() && foliage_opaque_pipeline_ &&
         foliage_opaque_pipeline_->Initialized() && foliage_masked_pipeline_ &&
         foliage_masked_pipeline_->Initialized() && foliage_opaque_shadow_pipeline_ &&
         foliage_opaque_shadow_pipeline_->Initialized() && foliage_masked_shadow_pipeline_ &&
         foliage_masked_shadow_pipeline_->Initialized();
}

void GpuTreeMeshRenderer::Register(const std::shared_ptr<Scene>& scene) {
  const ProfilerScope total_scope("EcoSysLab / Tree render registration", "EcoSysLab");
  const auto frame_number = Platform::GetFrameCount();
  retired_trees_.erase(std::remove_if(retired_trees_.begin(), retired_trees_.end(),
                                      [&](const auto& entry) {
                                        return frame_number - entry.first > Platform::GetMaxFramesInFlight() + 1;
                                      }),
                       retired_trees_.end());
  if (cached_scene_.lock() != scene) {
    for (auto& entry : tree_node_cache_)
      retired_trees_.emplace_back(frame_number, std::move(entry.second));
    tree_node_cache_.clear();
    cached_scene_ = scene;
  }
  const auto branches = scene->UnsafeGetPrivateComponentOwnersList<BranchRenderer>();
  const auto leaves = scene->UnsafeGetPrivateComponentOwnersList<FoliageRenderer>();
  std::unordered_set<Entity, Entity> entities;
  if (branches)
    entities.insert(branches->begin(), branches->end());
  if (leaves)
    entities.insert(leaves->begin(), leaves->end());
  if (!Prepare())
    return;
  const auto render = ApplicationContext::Get().GetLayer<RenderLayer>();
  const auto storage = render->GetCurrentRenderInstanceStorage();
  const auto index = Platform::GetCurrentFrameIndex();
  auto scene_bound = scene->GetBound();
  const auto self = shared_from_this();
  for (const auto entity : entities) {
    const ProfilerScope tree_scope("EcoSysLab / Prepare render tree", "EcoSysLab");
    if (!scene->IsEntityEnabled(entity) || !scene->HasPrivateComponent<Tree>(entity))
      continue;
    const auto owner = scene->GetOrSetPrivateComponent<Tree>(entity).lock();
    if (!owner || !owner->IsEnabled())
      continue;
    const auto branch = scene->HasPrivateComponent<BranchRenderer>(entity)
                            ? scene->GetOrSetPrivateComponent<BranchRenderer>(entity).lock()
                            : nullptr;
    const auto leaf = scene->HasPrivateComponent<FoliageRenderer>(entity)
                          ? scene->GetOrSetPrivateComponent<FoliageRenderer>(entity).lock()
                          : nullptr;
    bool branch_enabled = branch && branch->IsEnabled(), leaf_enabled = leaf && leaf->IsEnabled();
    if (!branch_enabled && !leaf_enabled)
      continue;
    const auto descriptor = owner->tree_descriptor_ref.Get<TreeDescriptor>();
    const auto bark = descriptor ? descriptor->bark_descriptor.Get<BasicBarkDescriptor>() : nullptr;
    const auto foliage = descriptor ? descriptor->foliage_descriptor.Get<BasicFoliageDescriptor>() : nullptr;
    const auto branch_material = branch_enabled ? branch->material_ref.Get<Material>() : nullptr;
    branch_enabled = branch_material && ClassifyGltfRasterMaterial(
                                            branch_material->BuildGltfMaterialData().shade_material,
                                            branch_material->draw_settings.blending) == GltfRasterMaterialClass::Opaque;
    auto leaf_material = leaf_enabled ? leaf->material_ref.Get<Material>() : nullptr;
    if (leaf_enabled && !leaf_material && foliage)
      leaf_material = foliage->leaf_material_ref.Get<Material>();
    if (leaf_enabled && !leaf_material)
      leaf_material = default_foliage_material_;
    auto leaf_class = GltfRasterMaterialClass::Forward;
    if (leaf_enabled) {
      leaf_class = ClassifyGltfRasterMaterial(leaf_material->BuildGltfMaterialData().shade_material,
                                              leaf_material->draw_settings.blending);
      leaf_enabled = leaf_class != GltfRasterMaterialClass::Forward;
    }
    if (!branch_enabled && !leaf_enabled)
      continue;
    TreeHandles handles;
    int branch_index = -1, leaf_index = -1;
    if (branch_enabled) {
      handles.branch = branch->GetHandle();
      handles.has_branch =
          storage->RegisterRenderInstance(scene, entity, handles.branch, branch_material, &branch_index);
    }
    if (leaf_enabled) {
      handles.foliage = leaf->GetHandle();
      handles.has_foliage = storage->RegisterRenderInstance(scene, entity, handles.foliage, leaf_material, &leaf_index);
    }
    if (!handles.has_branch && !handles.has_foliage)
      continue;
    const auto physics = scene->HasPrivateComponent<DynamicTreeSkeleton>(entity)
                             ? scene->GetOrSetPrivateComponent<DynamicTreeSkeleton>(entity).lock()
                             : nullptr;
    const auto state = physics && physics->IsEnabled() ? physics->GetGpuPhysicsState() : nullptr;
    auto& cached = tree_node_cache_[entity];
    if (!cached)
      cached = std::make_shared<TreeNodeCache>();
    auto& cache = *cached;
    const auto& settings = owner->tree_mesh_generator_settings;
    const auto parameters = GpuTreeMeshParameters(settings, bark.get());
    if (cache.tree.lock() != owner || cache.parameters != parameters || cache.physics.lock() != state ||
        cache.shoot_revision != owner->GetShootModelRevision().content ||
        cache.root_revision != owner->GetRootModelRevision().content) {
      const ProfilerScope build_scope("EcoSysLab / Build tree render cache", "EcoSysLab");
      cache.nodes.clear();
      AppendGpuTreeNodes(owner->shoot_model.PeekShootSkeleton(), settings, bark.get(), 0, true, cache.nodes);
      cache.shoot_count = static_cast<uint32_t>(cache.nodes.size());
      if (state)
        for (auto& node : cache.nodes) {
          if (!physics->HasPhysicsRodForNode(static_cast<SkeletonNodeHandle>(node.foliage_meta.x)))
            continue;
          while (node.links.x != UINT32_MAX && !physics->HasPhysicsRodForNode(static_cast<SkeletonNodeHandle>(
                                                   cache.nodes[node.links.x].foliage_meta.x)))
            node.links.x = cache.nodes[node.links.x].links.x;
        }
      AppendGpuTreeNodes(owner->root_model.PeekRootSkeleton(), settings, bark.get(), 0, false, cache.nodes);
      for (size_t i = 0; i < cache.nodes.size(); ++i) {
        auto& node = cache.nodes[i];
        node.foliage_meta.z = entity.GetIndex() ^ (entity.GetVersion() * 0x9e3779b9u);
        if (state && i < cache.shoot_count &&
            !physics->HasPhysicsRodForNode(static_cast<SkeletonNodeHandle>(node.foliage_meta.x)))
          node.links.z = UINT32_MAX;
        else
          node.links.z = 0;
      }
      std::vector<bool> apical;
      for (const auto handle : owner->shoot_model.PeekShootSkeleton().PeekSortedNodeList())
        apical.push_back(owner->shoot_model.PeekShootSkeleton().PeekNode(handle).IsApical());
      cache.shadow_groups = BuildTreeShadowGroups(cache.nodes, apical);
      cache.tree = owner;
      cache.physics = state;
      cache.parameters = parameters;
      cache.shoot_revision = owner->GetShootModelRevision().content;
      cache.root_revision = owner->GetRootModelRevision().content;
      cache.foliage_padding = -1;
      ++cache.generation;
    }
    if (cache.nodes.empty())
      continue;
    GpuTreeFoliageParams params{};
    if (foliage) {
      params.size_count = {foliage->leaf_size.x, foliage->leaf_size.y, 0, static_cast<float>(foliage->leaf_count)};
      params.filters = {foliage->max_node_thickness, foliage->min_root_distance, foliage->max_end_distance,
                        foliage->rotation_variance};
      params.distributions = {foliage->stem_length.mean, foliage->stem_length.deviation, foliage->branching_angle.mean,
                              foliage->branching_angle.deviation};
      params.tropism = {foliage->horizontal_tropism, foliage->gravitropism, 0, 0};
    } else if (handles.has_foliage) {
      const BasicFoliageDescriptor fallback;
      params.size_count = {fallback.leaf_size.x, fallback.leaf_size.y, 0, static_cast<float>(fallback.leaf_count)};
      params.filters = {fallback.max_node_thickness, fallback.min_root_distance, fallback.max_end_distance,
                        fallback.rotation_variance};
      params.distributions = {fallback.stem_length.mean, fallback.stem_length.deviation, fallback.branching_angle.mean,
                              fallback.branching_angle.deviation};
      params.tropism = {fallback.horizontal_tropism, fallback.gravitropism, 0, 0};
    }
    params.flags.x = handles.has_foliage ? (leaf_class == GltfRasterMaterialClass::Masked ? 2u : 1u) : 0u;
    const float leaf_padding = handles.has_foliage ? 2 * glm::max(params.size_count.x, params.size_count.y) : 0;
    if (cache.foliage_padding != leaf_padding) {
      cache.local_bound.min = glm::vec3(std::numeric_limits<float>::max());
      cache.local_bound.max = glm::vec3(std::numeric_limits<float>::lowest());
      for (const auto& node : cache.nodes) {
        const glm::vec3 a(node.position_thickness), b = a + glm::vec3(node.direction_length) * node.direction_length.w;
        const glm::vec3 padding(node.position_thickness.w + leaf_padding);
        cache.local_bound.min = glm::min(cache.local_bound.min, glm::min(a, b) - padding);
        cache.local_bound.max = glm::max(cache.local_bound.max, glm::max(a, b) + padding);
      }
      cache.foliage_padding = leaf_padding;
    }
    const auto transform = scene->GetDataComponent<GlobalTransform>(entity);
    for (uint32_t corner = 0; corner < 8; ++corner) {
      const auto& a = cache.local_bound.min;
      const auto& b = cache.local_bound.max;
      const glm::vec3 point(transform.value *
                            glm::vec4(corner & 1 ? b.x : a.x, corner & 2 ? b.y : a.y, corner & 4 ? b.z : a.z, 1));
      scene_bound.min = glm::min(scene_bound.min, point);
      scene_bound.max = glm::max(scene_bound.max, point);
    }
    if (cache.frames.empty()) {
      VkBufferCreateInfo info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
      info.size = sizeof(GpuTreeNode);
      info.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
      VmaAllocationCreateInfo device{};
      device.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
      auto host = device;
      host.usage = VMA_MEMORY_USAGE_AUTO_PREFER_HOST;
      host.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT | VMA_ALLOCATION_CREATE_MAPPED_BIT;
      host.requiredFlags = VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
      cache.frames.resize(Platform::GetMaxFramesInFlight());
      for (auto& frame : cache.frames) {
        frame.nodes = std::make_shared<Buffer>(info, device);
        frame.rest = std::make_shared<Buffer>(info, device);
        frame.shadow_groups = std::make_shared<Buffer>(info, device);
        frame.shadow_ranges = std::make_shared<Buffer>(info, device);
        frame.shadow_members = std::make_shared<Buffer>(info, device);
        frame.canopy_bounds = std::make_shared<Buffer>(info, device);
        info.size = sizeof(GpuTreeFoliageParams);
        frame.foliage = std::make_shared<Buffer>(info, device);
        info.size = sizeof(GpuTreeInstanceIndices);
        frame.instances = std::make_shared<Buffer>(info, host);
        info.size = sizeof(GpuTreeNode);
        frame.draw_set = std::make_shared<DescriptorSet>(layout_);
        frame.deform_set = std::make_shared<DescriptorSet>(deform_layout_);
      }
    }
    auto& frame = cache.frames[index];
    frame.handles = handles;
    frame.tessellation = branch && branch->screen_space_tessellation && !std::getenv("EVOENGINE_TREE_FULL_TESSELLATION")
                             ? glm::vec2(glm::clamp(branch->pixel_error, 0.1f, 2.0f),
                                         glm::clamp(branch->segment_length_pixels, 2.0f, 64.0f))
                             : glm::vec2(0);
    frame.impostor_diameter =
        branch && branch->camera_impostors ? glm::clamp(branch->impostor_diameter_pixels, 1.0f, 8.0f) : 0.0f;
    if (const auto value = std::getenv("EVOENGINE_TREE_IMPOSTOR_PIXELS"))
      frame.impostor_diameter = glm::clamp(static_cast<float>(std::atof(value)), 0.0f, 8.0f);
    frame.canopy_texels = branch && branch->canopy_shadows ? glm::clamp(branch->canopy_shadow_texels, 4.0f, 512.0f) : 0;
    if (const auto value = std::getenv("EVOENGINE_TREE_CANOPY_TEXELS"))
      frame.canopy_texels = glm::clamp(static_cast<float>(std::atof(value)), 0.0f, 512.0f);
    frame.indices_ready = false;
    frame.physics = state;
    frame.node_count = static_cast<uint32_t>(cache.nodes.size());
    BufferUploadBatch upload;
    std::vector<GpuTreeNode> packed;
    std::vector<glm::vec4> empty_bounds;
    std::vector<GpuTreeFoliageParams> foliage_upload;
    const int branch_slot = handles.has_branch ? branch_index : -1, leaf_slot = handles.has_foliage ? leaf_index : -1;
    if (frame.generation != cache.generation || frame.branch_material != branch_slot ||
        frame.foliage_material != leaf_slot) {
      upload.AddVector(frame.shadow_groups, cache.shadow_groups.node_groups,
                       {BufferUploadUsage::StorageRead, BufferUploadCapacityPolicy::GrowIfNeeded});
      if (!cache.shadow_groups.groups.empty()) {
        upload.AddVector(frame.shadow_ranges, cache.shadow_groups.groups,
                         {BufferUploadUsage::StorageRead, BufferUploadCapacityPolicy::GrowIfNeeded});
        upload.AddVector(frame.shadow_members, cache.shadow_groups.members,
                         {BufferUploadUsage::StorageRead, BufferUploadCapacityPolicy::GrowIfNeeded});
        empty_bounds.resize(cache.shadow_groups.groups.size() * 2);
        upload.AddVector(frame.canopy_bounds, empty_bounds,
                         {BufferUploadUsage::StorageReadWrite, BufferUploadCapacityPolicy::GrowIfNeeded});
      }
      packed = cache.nodes;
      for (auto& node : packed) {
        const bool valid = node.links.z != UINT32_MAX;
        node.links.z = valid && handles.has_branch ? static_cast<uint32_t>(branch_slot) : UINT32_MAX;
        node.links.w =
            valid && handles.has_foliage && node.foliage_meta.y ? static_cast<uint32_t>(leaf_slot) : UINT32_MAX;
      }
      upload.AddVector(frame.nodes, packed,
                       {state ? BufferUploadUsage::StorageReadWrite : BufferUploadUsage::StorageRead,
                        BufferUploadCapacityPolicy::GrowIfNeeded});
      if (state)
        upload.AddVector(frame.rest, packed,
                         {BufferUploadUsage::StorageRead, BufferUploadCapacityPolicy::GrowIfNeeded});
      frame.generation = cache.generation;
      frame.branch_material = branch_slot;
      frame.foliage_material = leaf_slot;
      Profiler::GetInstance().RecordCounter(
          "Tree node upload bytes", packed.size() * sizeof(GpuTreeNode) * (state ? 2 : 1), "EcoSysLab", "bytes");
    }
    if (!frame.params_ready || std::memcmp(&frame.params, &params, sizeof(params))) {
      foliage_upload.push_back(params);
      upload.AddVector(frame.foliage, foliage_upload,
                       {BufferUploadUsage::StorageRead, BufferUploadCapacityPolicy::GrowIfNeeded});
      frame.params = params;
      frame.params_ready = true;
    }
    upload.Record(upload_arena_);
    frame.draw_set->UpdateBufferDescriptorBinding(0, frame.nodes);
    frame.draw_set->UpdateBufferDescriptorBinding(1, frame.instances);
    frame.draw_set->UpdateBufferDescriptorBinding(2, frame.foliage);
    frame.draw_set->UpdateBufferDescriptorBinding(5, frame.shadow_groups);
    frame.draw_set->UpdateBufferDescriptorBinding(6, frame.shadow_ranges);
    frame.draw_set->UpdateBufferDescriptorBinding(7, frame.shadow_members);
    frame.draw_set->UpdateBufferDescriptorBinding(8, frame.canopy_bounds);
    if (state && cache.shoot_count) {
      frame.deform_set->UpdateBufferDescriptorBinding(0, frame.nodes);
      frame.deform_set->UpdateBufferDescriptorBinding(1, state->vertices[state->vertex_index]);
      frame.deform_set->UpdateBufferDescriptorBinding(2, state->rods[state->rod_index]);
      frame.deform_set->UpdateBufferDescriptorBinding(3, state->node_rods);
      frame.deform_set->UpdateBufferDescriptorBinding(4, frame.rest);
      const auto rotation = glm::inverse(transform.GetRotation());
      const DeformPushConstant push{0,
                                    cache.shoot_count,
                                    0,
                                    state->attribute_arrays ? 1u : 0u,
                                    glm::vec4(transform.GetPosition(), 0),
                                    glm::vec4(transform.GetScale(), 0),
                                    {rotation.x, rotation.y, rotation.z, rotation.w}};
      Platform::RecordCommandsMainQueue([&](VkCommandBuffer command) {
        deform_pipeline_->Bind(command);
        deform_pipeline_->BindDescriptorSet(command, 0, frame.deform_set->GetVkDescriptorSet());
        deform_pipeline_->PushConstant(command, 0, push);
        deform_pipeline_->Dispatch(command, (cache.shoot_count + 63) / 64, 1, 1);
        Platform::EverythingBarrier(command);
      });
    }
    if (handles.has_branch && directional_shadow_impostors && !shadow_reference_ && frame.canopy_texels > 0 &&
        !cache.shadow_groups.groups.empty()) {
      Platform::RecordCommandsMainQueue([&](VkCommandBuffer command) {
        const auto timing = Platform::BeginGpuTimestampScope(
            command, {"EcoSysLabCanopyBounds", "EcoSysLab Canopy Bounds", "EcoSysLab", GpuTimestampQueue::Compute});
        canopy_pipeline_->Bind(command);
        canopy_pipeline_->BindDescriptorSet(command, 3, frame.draw_set->GetVkDescriptorSet());
        canopy_pipeline_->Dispatch(command, static_cast<uint32_t>(cache.shadow_groups.groups.size()), 1, 1);
        Platform::EverythingBarrier(command);
        Platform::EndGpuTimestampScope(command, timing);
      });
    }
    if (handles.has_branch) {
      render->RawOpaqueRenderingAllCameras(
          [self, cached](VkCommandBuffer command, const auto&, const RenderLayer::DeferredRenderingView& view) {
            const auto count = self->Draw(cached, command, self->camera_pipeline_, view.viewport, 0, 0,
                                          view.camera_index, 0, 0, VK_CULL_MODE_NONE);
            if (self->impostor_pipeline_ && cached->frames[Platform::GetCurrentFrameIndex()].impostor_diameter > 0)
              self->Draw(cached, command, self->impostor_pipeline_, view.viewport, 0, 0, view.camera_index, 0, 0,
                         VK_CULL_MODE_NONE);
            return count;
          });
      render->RenderOpaqueToPointLightShadowMap([self, cached](VkCommandBuffer command, const auto& view) {
        return self->Draw(cached, command, self->shadow_pipeline_, view.viewport, 0, 1, 0, view.light_index,
                          view.face_index, VK_CULL_MODE_NONE);
      });
      render->RenderOpaqueToSpotLightShadowMap([self, cached](VkCommandBuffer command, const auto& view) {
        return self->Draw(cached, command, self->shadow_pipeline_, view.viewport, 0, 2, 0, view.light_index, 0,
                          VK_CULL_MODE_NONE);
      });
      render->RenderOpaqueToDirectionalLightShadowMap([self, cached](VkCommandBuffer command, const auto& view) {
        return self->Draw(
            cached, command,
            self->directional_shadow_impostors ? self->directional_shadow_pipeline_ : self->shadow_pipeline_,
            view.viewport, 0, 3, 0, view.light_index, view.split_index, VK_CULL_MODE_NONE);
      });
    }
    if (handles.has_foliage) {
      if (leaf_class == GltfRasterMaterialClass::Opaque) {
        render->RawOpaqueRenderingAllCameras(
            [self, cached](VkCommandBuffer command, const auto&, const RenderLayer::DeferredRenderingView& view) {
              return self->Draw(cached, command, self->foliage_opaque_pipeline_, view.viewport, 1, 0, view.camera_index,
                                0, 0, VK_CULL_MODE_NONE);
            });
        render->RenderOpaqueToPointLightShadowMap([self, cached](VkCommandBuffer command, const auto& view) {
          return self->Draw(cached, command, self->foliage_opaque_shadow_pipeline_, view.viewport, 1, 1, 0,
                            view.light_index, view.face_index, VK_CULL_MODE_NONE);
        });
        render->RenderOpaqueToSpotLightShadowMap([self, cached](VkCommandBuffer command, const auto& view) {
          return self->Draw(cached, command, self->foliage_opaque_shadow_pipeline_, view.viewport, 1, 2, 0,
                            view.light_index, 0, VK_CULL_MODE_NONE);
        });
        render->RenderOpaqueToDirectionalLightShadowMap([self, cached](VkCommandBuffer command, const auto& view) {
          return self->Draw(cached, command, self->foliage_opaque_shadow_pipeline_, view.viewport, 1, 3, 0,
                            view.light_index, view.split_index, VK_CULL_MODE_NONE);
        });
      } else {
        render->AlphaMaskedRenderingAllCameras(
            [self, cached](VkCommandBuffer command, const auto&, const RenderLayer::DeferredRenderingView& view) {
              return self->Draw(cached, command, self->foliage_masked_pipeline_, view.viewport, 2, 0, view.camera_index,
                                0, 0, VK_CULL_MODE_NONE);
            });
        render->RenderAlphaMaskedToPointLightShadowMap([self, cached](VkCommandBuffer command, const auto& view) {
          return self->Draw(cached, command, self->foliage_masked_shadow_pipeline_, view.viewport, 2, 1, 0,
                            view.light_index, view.face_index, VK_CULL_MODE_NONE);
        });
        render->RenderAlphaMaskedToSpotLightShadowMap([self, cached](VkCommandBuffer command, const auto& view) {
          return self->Draw(cached, command, self->foliage_masked_shadow_pipeline_, view.viewport, 2, 2, 0,
                            view.light_index, 0, VK_CULL_MODE_NONE);
        });
        render->RenderAlphaMaskedToDirectionalLightShadowMap([self, cached](VkCommandBuffer command, const auto& view) {
          return self->Draw(cached, command, self->foliage_masked_shadow_pipeline_, view.viewport, 2, 3, 0,
                            view.light_index, view.split_index, VK_CULL_MODE_NONE);
        });
      }
    }
  }
  for (auto it = tree_node_cache_.begin(); it != tree_node_cache_.end();) {
    if (entities.find(it->first) != entities.end() && !it->second->tree.expired()) {
      ++it;
      continue;
    }
    retired_trees_.emplace_back(frame_number, std::move(it->second));
    it = tree_node_cache_.erase(it);
  }
  scene->SetBound(scene_bound);
}

uint32_t GpuTreeMeshRenderer::Draw(const std::shared_ptr<TreeNodeCache>& tree, VkCommandBuffer command,
                                   const std::shared_ptr<GraphicsPipeline>& pipeline, const glm::ivec4& viewport,
                                   uint32_t geometry, uint32_t pass, uint32_t camera, uint32_t light, uint32_t face,
                                   VkCullModeFlags cull_mode) {
  auto& frame = tree->frames[Platform::GetCurrentFrameIndex()];
  const auto count = geometry ? tree->shoot_count : frame.node_count;
  if (!count)
    return 0;
  if (!frame.indices_ready) {
    const auto storage = ApplicationContext::Get().GetLayer<RenderLayer>()->GetCurrentRenderInstanceStorage();
    const GpuTreeInstanceIndices indices{
        frame.handles.has_branch ? static_cast<uint32_t>(storage->GetRenderInstanceIndex(frame.handles.branch)) : 0u,
        frame.handles.has_foliage ? static_cast<uint32_t>(storage->GetRenderInstanceIndex(frame.handles.foliage)) : 0u};
    std::memcpy(frame.instances->GetVmaAllocationInfo().pMappedData, &indices, sizeof(indices));
    frame.indices_ready = true;
  }
  pipeline->Bind(command);
  pipeline->BindDescriptorSet(command, 0, RenderLayer::GetPerFrameDescriptorSet()->GetVkDescriptorSet());
  pipeline->BindDescriptorSet(command, 3, frame.draw_set->GetVkDescriptorSet());
  pipeline->states.ResetAllStates(pass == 0 ? 5 : 0);
  pipeline->states.SetViewportScissor(viewport);
  pipeline->states.cull_mode = cull_mode;
  pipeline->states.ApplyAllStates(command);
  const bool shadow = geometry == 0 && pass == 3;
  const auto timing =
      shadow && std::getenv("EVOENGINE_TREE_SHADOW_PROFILE")
          ? Platform::BeginGpuTimestampScope(command, {"EcoSysLabBranchShadow" + std::to_string(face),
                                                       "EcoSysLab Branch Shadow Cascade " + std::to_string(face),
                                                       "EcoSysLab", GpuTimestampQueue::Graphics})
          : GpuTimestampScopeToken{};
  const auto geometry_timing =
      geometry == 0 && pass == 0 && std::getenv("EVOENGINE_TREE_GEOMETRY_PROFILE")
          ? Platform::BeginGpuTimestampScope(command, {"EcoSysLabBranchGeometry", "EcoSysLab Branch Geometry",
                                                       "EcoSysLab", GpuTimestampQueue::Graphics})
          : GpuTimestampScopeToken{};
  const uint32_t batch = (pipeline == camera_pipeline_ || pipeline == impostor_pipeline_) ? camera_batch_
                         : pipeline == directional_shadow_pipeline_ && !shadow_reference_ ? 32
                                                                                          : 1;
  const auto limit = Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxTaskWorkGroupCount[0];
  for (uint32_t base = 0; base < count; base += limit * batch) {
    TreeMeshPushConstant push{};
    push.node_base = base;
    push.node_count = count;
    push.padding0 = batch;
    push.camera_index = camera;
    push.pass_index = pass;
    push.light_index = light;
    push.light_face = face;
    push.geometry_pass = geometry;
    if (geometry == 0 && pass == 0 && impostor_pipeline_)
      push.impostor = {frame.impostor_diameter, pipeline == impostor_pipeline_ ? 1.0f : 0.0f, 0.0f, 0.0f};
    if (geometry == 0 && pass == 0)
      push.tessellation = {static_cast<float>(viewport.z), static_cast<float>(viewport.w), frame.tessellation.x,
                           frame.tessellation.y};
    if (pipeline == directional_shadow_pipeline_ && !shadow_reference_)
      push.impostor = {frame.canopy_texels, static_cast<float>(viewport.z), static_cast<float>(viewport.w), 0};
    pipeline->PushConstant(command, 0, push);
    pipeline->DrawMeshTasks(command, std::min(limit, (count - base + batch - 1) / batch), 1, 1);
  }
  Platform::EndGpuTimestampScope(command, geometry_timing);
  Platform::EndGpuTimestampScope(command, timing);
  return count;
}
