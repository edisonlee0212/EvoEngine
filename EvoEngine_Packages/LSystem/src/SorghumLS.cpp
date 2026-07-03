#include "SorghumLS.hpp"

#include "LSystemInspectionAdapters.hpp"
#include "LSystemSerializationAdapters.hpp"
#include "SorghumLSDescriptor.hpp"

#include <AssetManager.hpp>
#include <EditorLayer.hpp>
#include <Material.hpp>
#include <Mesh.hpp>
#include <MeshRenderer.hpp>
#include <Particles.hpp>
#include <Scene.hpp>
#include <Times.hpp>
#include <Texture2D.hpp>
#include <Transform.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>

using namespace l_system_package;
using namespace evo_engine;

namespace {

SorghumLS::ColorMode g_sorghum_ls_color_mode = SorghumLS::ColorMode::Shaded;

glm::vec4 HashToColor(const uint32_t id) {
  const float hue = static_cast<float>((id * 2654435761u) & 1023u) / 1024.0f;
  const float s = 0.72f;
  const float v = 0.92f;
  const float h6 = hue * 6.0f;
  const int sector = static_cast<int>(h6);
  const float f = h6 - static_cast<float>(sector);
  const float p = v * (1.0f - s);
  const float q = v * (1.0f - s * f);
  const float t = v * (1.0f - s * (1.0f - f));

  glm::vec3 rgb(v, t, p);
  switch (sector % 6) {
    case 0: rgb = glm::vec3(v, t, p); break;
    case 1: rgb = glm::vec3(q, v, p); break;
    case 2: rgb = glm::vec3(p, v, t); break;
    case 3: rgb = glm::vec3(p, q, v); break;
    case 4: rgb = glm::vec3(t, p, v); break;
    default: rgb = glm::vec3(v, p, q); break;
  }
  return glm::vec4(rgb, 1.0f);
}

bool IsFiniteVec3(const glm::vec3& v) {
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

bool IsFiniteQuat(const glm::quat& q) {
  return std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w);
}

bool IsFiniteMat4(const glm::mat4& m) {
  for (int c = 0; c < 4; c++) {
    for (int r = 0; r < 4; r++) {
      if (!std::isfinite(m[c][r])) {
        return false;
      }
    }
  }
  return true;
}

glm::vec4 EvaluateInternodeGreenPalette(const int rank,
                                        const int max_rank,
                                        const int order) {
  const float rank_t = (max_rank > 0)
      ? std::clamp(static_cast<float>(rank) / static_cast<float>(max_rank), 0.0f, 1.0f)
      : 0.0f;

  const glm::vec3 dark_green(0.12f, 0.33f, 0.11f);
  const glm::vec3 mid_green(0.24f, 0.56f, 0.18f);
  const glm::vec3 light_green(0.46f, 0.77f, 0.34f);

  glm::vec3 color = (rank_t <= 0.5f)
      ? glm::mix(dark_green, mid_green, rank_t * 2.0f)
      : glm::mix(mid_green, light_green, (rank_t - 0.5f) * 2.0f);

  // Slightly darken higher-order axes so tillers remain separable but in-palette.
  const float order_t = std::clamp(static_cast<float>(order), 0.0f, 3.0f) / 3.0f;
  color *= (1.0f - 0.08f * order_t);

  return glm::vec4(color, 1.0f);
}

void GenerateUnitCylinderMesh(std::vector<Vertex>& vertices,
                              std::vector<unsigned int>& indices,
                              int segments = 8) {
  vertices.clear();
  indices.clear();

  const glm::vec4 stem_color(0.36f, 0.58f, 0.24f, 1.0f);
  const float angle_step = glm::two_pi<float>() / static_cast<float>(segments);

  for (int ring = 0; ring <= 1; ring++) {
    const float y = static_cast<float>(ring);
    for (int s = 0; s < segments; s++) {
      const float angle = angle_step * static_cast<float>(s);
      const float cx = std::cos(angle);
      const float cz = std::sin(angle);
      Vertex v;
      v.position = glm::vec3(cx, y, cz);
      v.normal = glm::normalize(glm::vec3(cx, 0.0f, cz));
      v.color = stem_color;
      v.tex_coord = glm::vec2(static_cast<float>(s) / static_cast<float>(segments), y);
      vertices.push_back(v);
    }
  }

  for (int s = 0; s < segments; s++) {
    const unsigned int s0 = static_cast<unsigned int>(s);
    const unsigned int s1 = static_cast<unsigned int>((s + 1) % segments);
    const unsigned int e0 = s0 + static_cast<unsigned int>(segments);
    const unsigned int e1 = s1 + static_cast<unsigned int>(segments);
    indices.push_back(s0);
    indices.push_back(e0);
    indices.push_back(s1);
    indices.push_back(s1);
    indices.push_back(e0);
    indices.push_back(e1);
  }
}

bool BuildStemContextForLeafNode(const SorghumGraph& graph,
                                 const LNodeHandle leaf_handle,
                                 StemContext& out_stem_ctx) {
  out_stem_ctx.segments.clear();
  if (leaf_handle < 0) return false;

  const auto& leaf_node = graph.PeekNode(leaf_handle);
  if (!leaf_node.data.Is<SorghumLeaf>()) {
    return false;
  }
  const int leaf_order = leaf_node.data.Get<SorghumLeaf>().order;

  std::vector<LNodeHandle> chain;
  auto cursor = leaf_node.GetParentHandle();
  while (cursor >= 0) {
    const auto& node = graph.PeekNode(cursor);
    if (node.data.Is<SorghumInternode>()) {
      const auto& internode = node.data.Get<SorghumInternode>();
      if (internode.order == leaf_order) {
        chain.push_back(cursor);
      }
    }
    cursor = node.GetParentHandle();
  }
  if (chain.empty()) {
    return false;
  }

  std::reverse(chain.begin(), chain.end());

  for (const auto node_handle : chain) {
    const auto& node = graph.PeekNode(node_handle);
    const auto& internode = node.data.Get<SorghumInternode>();

    StemContext::Segment seg;
    seg.position = node.info.global_position;
    seg.front = glm::normalize(node.info.global_rotation * glm::vec3(0, 0, -1));
    seg.up = glm::normalize(node.info.global_rotation * glm::vec3(0, 1, 0));
    seg.radius = std::max(0.0002f, internode.thickness * 0.5f);
    seg.theta = 180.0f;
    out_stem_ctx.segments.push_back(seg);
  }

  // Add an explicit endpoint segment at the distal end of the host axis.
  const auto& tip_node = graph.PeekNode(chain.back());
  const auto& tip_internode = tip_node.data.Get<SorghumInternode>();
  StemContext::Segment tip_seg;
  tip_seg.position = tip_node.info.GetGlobalEndPosition();
  tip_seg.front = glm::normalize(tip_node.info.global_rotation * glm::vec3(0, 0, -1));
  tip_seg.up = glm::normalize(tip_node.info.global_rotation * glm::vec3(0, 1, 0));
  tip_seg.radius = std::max(0.0002f, tip_internode.thickness * 0.5f);
  tip_seg.theta = 180.0f;
  out_stem_ctx.segments.push_back(tip_seg);

  return out_stem_ctx.segments.size() >= 2;
}

bool HasSorghumGeometryChildren(const std::shared_ptr<Scene>& scene,
                               const Entity owner) {
  if (!scene || !scene->IsEntityValid(owner)) {
    return false;
  }

  bool has_internodes = false;
  bool has_leaves = false;
  for (const auto& child : scene->GetChildren(owner)) {
    if (!scene->IsEntityValid(child)) {
      continue;
    }
    const auto name = scene->GetEntityName(child);
    if (name == "Sorghum Internodes") {
      has_internodes = true;
    } else if (name == "Sorghum Leaves") {
      has_leaves = true;
    }

    if (has_internodes && has_leaves) {
      return true;
    }
  }

  return false;
}

}  // namespace

void SorghumLS::SetGlobalColorMode(const ColorMode mode) {
  g_sorghum_ls_color_mode = mode;
}

SorghumLS::ColorMode SorghumLS::GetGlobalColorMode() {
  return g_sorghum_ls_color_mode;
}

void SorghumLS::ClearGeometryEntities() const {
  const auto scene = GetScene();
  const auto self = GetOwner();
  const auto children = scene->GetChildren(self);
  for (const auto& child : children) {
    scene->DeleteEntity(child);
  }
}

void SorghumLS::GenerateGeometryEntities(const bool uncapped_growth) {
  ClearGeometryEntities();
  growth_model.Reset();
  GrowToTargetGDD(uncapped_growth);
}

void SorghumLS::GeneratePreviewGeometryEntities(const float preview_target_gdd,
                                                const uint32_t preview_max_growth_steps) {
  // Preview always replays from a clean state so topology-affecting edits are visible.
  ClearGeometryEntities();
  growth_model.Reset();

  const double grow_start = GetApplication().GetTimes().Now();
  auto descriptor = descriptor_ref.Get<SorghumLSDescriptor>();
  if (!descriptor) {
    last_grow_seconds = 0.0;
    return;
  }

  growth_model.Initialize(*descriptor, seed);

  const float clamped_target_gdd = std::min(target_gdd, std::max(0.0f, preview_target_gdd));
  const uint32_t step_cap = std::max(1u, preview_max_growth_steps);
  growth_model.GrowToGDD(clamped_target_gdd, step_cap);

  last_grow_seconds = GetApplication().GetTimes().Now() - grow_start;
  RebuildGeometry();
}

void SorghumLS::GrowToTargetGDD(const bool uncapped_growth, const uint32_t max_growth_steps) {
  const double grow_start = GetApplication().GetTimes().Now();
  last_rebuild_seconds = 0.0;
  last_rebuild_internode_seconds = 0.0;
  last_leaf_spline_seconds = 0.0;
  last_leaf_mesh_seconds = 0.0;
  last_mesh_upload_seconds = 0.0;
  auto descriptor = descriptor_ref.Get<SorghumLSDescriptor>();
  if (!descriptor) {
    last_grow_seconds = 0.0;
    return;
  }

  if (!growth_model.IsInitialized()) {
    growth_model.Initialize(*descriptor, seed);
  }

  bool reinitialized = false;
  const float gdd_step = std::max(1.0e-5f, growth_model.gdd_per_growth_step);
  if (target_gdd + gdd_step < growth_model.accumulated_gdd) {
    growth_model.Initialize(*descriptor, seed);
    reinitialized = true;
  }

  growth_model.GrowToGDD(target_gdd, uncapped_growth ? 0u : max_growth_steps);
  last_grow_seconds = GetApplication().GetTimes().Now() - grow_start;

  const bool no_growth_step = !reinitialized && growth_model.last_growth_steps == 0;
  // After Ctrl+W reset, geometry children may be missing while growth can still
  // report zero new steps. Force one rebuild to restore child entities.
  const bool missing_geometry_children =
      !HasSorghumGeometryChildren(GetScene(), GetOwner());
  if (no_growth_step && !missing_geometry_children) {
    return;
  }

  RebuildGeometry();
}

void SorghumLS::SetSeasonalChronologicalMode(
    const bool enable_independent_chronological_clock) {
  growth_model.SetChronologicalCoupledToThermal(
      !enable_independent_chronological_clock);
}

bool SorghumLS::AdvanceChronologicalAging(const float delta_years) {
  if (!std::isfinite(delta_years) || delta_years <= 0.0f) {
    return false;
  }

  auto descriptor = descriptor_ref.Get<SorghumLSDescriptor>();
  if (!descriptor) {
    return false;
  }

  if (!growth_model.IsInitialized()) {
    growth_model.Initialize(*descriptor, seed);
  }

  growth_model.AdvanceChronologicalYears(delta_years);
  const bool changed = growth_model.AgeOnlyStep();
  if (changed) {
    RebuildGeometry();
  }
  return changed;
}

void SorghumLS::RebuildGeometry() {
  auto& times = GetApplication().GetTimes();
  const double rebuild_start = times.Now();
  last_invalid_instance_count = 0;
  last_node_count = 0;
  last_internode_count = 0;
  last_leaf_count = 0;
  last_live_leaf_count = 0;
  last_rebuild_internode_seconds = 0.0;
  last_leaf_spline_seconds = 0.0;
  last_leaf_mesh_seconds = 0.0;
  last_mesh_upload_seconds = 0.0;

  if (!growth_model.IsInitialized()) {
    last_rebuild_seconds = 0.0;
    return;
  }

  const auto scene = GetScene();
  const auto owner = GetOwner();
  const auto color_mode = GetGlobalColorMode();
  const glm::vec4 instance_color = HashToColor(owner.GetIndex());
  const auto descriptor = descriptor_ref.Get<SorghumLSDescriptor>();
  SorghumLeafAtlasLayout base_leaf_atlas_layout;
  if (descriptor) {
    base_leaf_atlas_layout.variant_columns = descriptor->leaf_atlas_variant_columns;
    base_leaf_atlas_layout.variant_rows = descriptor->leaf_atlas_variant_rows;
    base_leaf_atlas_layout.variant_count = descriptor->leaf_atlas_variant_count;
    base_leaf_atlas_layout.tile_uv_inset = descriptor->leaf_atlas_tile_uv_inset;
    base_leaf_atlas_layout.distal_region_uses_top_half = descriptor->leaf_atlas_distal_region_uses_top_half;
  }
  base_leaf_atlas_layout = NormalizeSorghumLeafAtlasLayout(base_leaf_atlas_layout);

  Entity internode_entity;
  Entity leaf_entity;
  for (const auto& child : scene->GetChildren(owner)) {
    const auto name = scene->GetEntityName(child);
    if (name == "Sorghum Internodes") {
      internode_entity = child;
    } else if (name == "Sorghum Leaves") {
      leaf_entity = child;
    }
  }

  // -------------------------------------------------------------------------
  // Internodes as instanced cylinders via Particles.
  // -------------------------------------------------------------------------
  {
    const double internode_start = times.Now();
    static thread_local std::vector<ParticleInfo> internode_infos_cache;
    auto& infos = internode_infos_cache;
    infos.clear();

    const auto& sorted = growth_model.graph.PeekSortedNodeList();
    last_node_count = static_cast<uint32_t>(sorted.size());

    int max_internode_rank = 0;
    for (const auto handle : sorted) {
      const auto& node = growth_model.graph.PeekNode(handle);
      if (!node.data.Is<SorghumInternode>()) {
        continue;
      }
      const auto& internode = node.data.Get<SorghumInternode>();
      max_internode_rank = std::max(max_internode_rank, std::max(0, internode.rank));
    }

    const glm::quat cylinder_axis_fix = glm::angleAxis(
      -glm::half_pi<float>(), glm::vec3(1.0f, 0.0f, 0.0f));

    for (const auto handle : sorted) {
      const auto& node = growth_model.graph.PeekNode(handle);
      if (!node.data.Is<SorghumInternode>()) {
        continue;
      }
      const auto& internode = node.data.Get<SorghumInternode>();
      if (node.info.length <= 0.0f || internode.thickness <= 0.0f) {
        continue;
      }

      if (!IsFiniteVec3(node.info.global_position) || !std::isfinite(node.info.length) ||
          !std::isfinite(internode.thickness)) {
        last_invalid_instance_count++;
        continue;
      }

      glm::quat instance_rotation = glm::normalize(node.info.global_rotation * cylinder_axis_fix);
      if (!IsFiniteQuat(instance_rotation)) {
        instance_rotation = glm::quat(1, 0, 0, 0);
        last_invalid_instance_count++;
      }

      const float half_thick = std::max(0.0001f, internode.thickness * 0.5f);
      ParticleInfo pi;
      const glm::mat4 model =
          glm::translate(node.info.global_position) *
          glm::mat4_cast(instance_rotation) *
          glm::scale(glm::vec3(half_thick, node.info.length, half_thick));
      if (!IsFiniteMat4(model)) {
        last_invalid_instance_count++;
        continue;
      }

      pi.instance_matrix.value = model;
      pi.instance_color = EvaluateInternodeGreenPalette(
          internode.rank,
          max_internode_rank,
          internode.order);
      infos.push_back(pi);

    }

    last_internode_count = static_cast<uint32_t>(infos.size());

    if (!scene->IsEntityValid(internode_entity)) {
      internode_entity = scene->CreateEntity("Sorghum Internodes");
      scene->SetParent(internode_entity, owner);
    }
    scene->SetEntitySerializable(internode_entity, false);

    const auto particles = scene->GetOrSetPrivateComponent<Particles>(internode_entity).lock();
    if (!particles->mesh.Get<Mesh>()) {
      auto mesh = AssetManager::CreateTemporaryAsset<Mesh>();
      std::vector<Vertex> cyl_verts;
      std::vector<unsigned int> cyl_indices;
      GenerateUnitCylinderMesh(cyl_verts, cyl_indices, 8);
      VertexAttributes attrs{};
      attrs.normal = true;
      attrs.tex_coord = true;
      attrs.color = true;
      mesh->SetVertices(attrs, cyl_verts, cyl_indices);
      particles->mesh = mesh;
    }
    if (!particles->material.Get<Material>()) {
      particles->material = AssetManager::CreateTemporaryAsset<Material>();
    }

    auto info_list = particles->particle_info_list.Get<ParticleInfoList>();
    if (!info_list) {
      particles->particle_info_list = info_list = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
    }
    info_list->SetParticleInfos(infos);
    last_rebuild_internode_seconds = times.Now() - internode_start;
  }

  // -------------------------------------------------------------------------
  // Leaves as one procedural mesh (all live leaves aggregated).
  // -------------------------------------------------------------------------
  {
    static thread_local std::vector<Vertex> leaf_vertices_cache;
    static thread_local std::vector<glm::uvec3> leaf_indices_cache;
    auto& leaf_vertices = leaf_vertices_cache;
    auto& leaf_indices = leaf_indices_cache;
    leaf_vertices.clear();
    leaf_indices.clear();
    leaf_vertices.reserve(16384);
    leaf_indices.reserve(16384);
    static thread_local StemContext stem_ctx;
    static thread_local SorghumSpline leaf_spline;

    const auto& sorted = growth_model.graph.PeekSortedNodeList();
    for (const auto handle : sorted) {
      const auto& node = growth_model.graph.PeekNode(handle);
      if (!node.data.Is<SorghumLeaf>()) {
        continue;
      }

      const auto& leaf = node.data.Get<SorghumLeaf>();
      last_leaf_count++;
      if (!leaf.alive) {
        continue;
      }
      last_live_leaf_count++;

      if (!BuildStemContextForLeafNode(growth_model.graph, handle, stem_ctx)) {
        continue;
      }

      const double spline_start = times.Now();
      BuildLeafSplineFromState(
          leaf, stem_ctx, growth_model.sampled, leaf_mesh_settings, leaf_spline);
      last_leaf_spline_seconds += times.Now() - spline_start;

      const double mesh_start = times.Now();
      const size_t vertex_start = leaf_vertices.size();
      auto leaf_atlas_layout = base_leaf_atlas_layout;
      leaf_atlas_layout.variant_index = ComputeSorghumLeafAtlasVariant(
          seed, static_cast<uint32_t>(node.GetIndex()), leaf.node_random, leaf_atlas_layout.variant_count);
      GenerateBladeGeometry(
          leaf_spline,
          leaf,
          growth_model.sampled,
          leaf_mesh_settings,
          leaf_vertices,
          leaf_indices,
          false,
          static_cast<uint32_t>(node.GetIndex()),
          leaf_atlas_layout);
      if (leaf_bottom_face) {
        GenerateBladeGeometry(
            leaf_spline,
            leaf,
            growth_model.sampled,
            leaf_mesh_settings,
            leaf_vertices,
            leaf_indices,
          true,
          static_cast<uint32_t>(node.GetIndex()),
          leaf_atlas_layout);
      }

      // Optional color overrides for debug modes.
      // Shaded and ByNode keep stage-shaded per-node colors generated by SorghumLeafMesh.
      if (color_mode == ColorMode::ByType || color_mode == ColorMode::ByInstance) {
        glm::vec4 override_color(0.22f, 0.56f, 0.20f, 1.0f);
        if (color_mode == ColorMode::ByType) {
          override_color = glm::vec4(0.18f, 0.72f, 0.24f, 1.0f);
        } else if (color_mode == ColorMode::ByInstance) {
          override_color = instance_color;
        }
        for (size_t i = vertex_start; i < leaf_vertices.size(); ++i) {
          leaf_vertices[i].color = override_color;
        }
      }
      last_leaf_mesh_seconds += times.Now() - mesh_start;
    }

    if (!scene->IsEntityValid(leaf_entity)) {
      leaf_entity = scene->CreateEntity("Sorghum Leaves");
      scene->SetParent(leaf_entity, owner);
    }
    scene->SetEntitySerializable(leaf_entity, false);

    const auto mesh_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(leaf_entity).lock();
    if (!mesh_renderer->mesh.Get<Mesh>()) {
      mesh_renderer->mesh = AssetManager::CreateTemporaryAsset<Mesh>();
    }
    if (!mesh_renderer->material.Get<Material>()) {
      mesh_renderer->material = AssetManager::CreateTemporaryAsset<Material>();
    }

    if (const auto material = mesh_renderer->material.Get<Material>()) {
      const auto albedo = descriptor ? descriptor->leaf_atlas_albedo_texture.Get<Texture2D>() : nullptr;
      if (albedo) {
        material->vertex_color_only = false;
        material->SetAlbedoTexture(albedo);
        material->SetNormalTexture(descriptor->leaf_atlas_normal_texture.Get<Texture2D>());
        material->SetRoughnessTexture(descriptor->leaf_atlas_roughness_texture.Get<Texture2D>());
        material->SetMetallicTexture(descriptor->leaf_atlas_metallic_texture.Get<Texture2D>());
        material->SetAoTexture(descriptor->leaf_atlas_ao_texture.Get<Texture2D>());
        material->material_properties.albedo_color = glm::vec3(1.0f);
        material->material_properties.metallic = 0.0f;
        material->material_properties.roughness = 0.8f;
      } else {
        material->SetAlbedoTexture(nullptr);
        material->SetNormalTexture(nullptr);
        material->SetRoughnessTexture(nullptr);
        material->SetMetallicTexture(nullptr);
        material->SetAoTexture(nullptr);
        material->vertex_color_only = true;
        material->material_properties.metallic = 0.0f;
        material->material_properties.roughness = 1.0f;
      }
    }

    const auto mesh = mesh_renderer->mesh.Get<Mesh>();
    if (!leaf_vertices.empty() && !leaf_indices.empty()) {
      const double upload_start = times.Now();
      mesh->ray_tracing_acceleration_enabled = false;
      mesh->compact_storage_on_update = false;
      mesh->optimize_meshlet_layout = false;
      VertexAttributes attributes{};
      attributes.normal = true;
      attributes.tangent = true;
      attributes.color = true;
      attributes.tex_coord = true;
      mesh->SetVertices(attributes, leaf_vertices, leaf_indices);
      last_mesh_upload_seconds = times.Now() - upload_start;
    }
  }

  last_rebuild_seconds = times.Now() - rebuild_start;
}

void SorghumLS::ExportObj(const std::filesystem::path& path) const {
  const auto scene = GetScene();
  const auto owner = GetOwner();

  for (const auto& child : scene->GetChildren(owner)) {
    if (scene->GetEntityName(child) != "Sorghum Leaves") {
      continue;
    }
    if (const auto renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(child).lock()) {
      if (const auto mesh = renderer->mesh.Get<Mesh>()) {
        if (!mesh->Export(path)) {
          EVOENGINE_ERROR("Sorghum mesh export failed.");
        }
        return;
      }
    }
  }

  EVOENGINE_ERROR("Sorghum mesh export failed: no Sorghum Leaves mesh entity.");
}

void SorghumLS::ExportFlowGraph(YAML::Emitter& out) {
  out << YAML::Key << "Flows" << YAML::Value << YAML::BeginSeq;

  if (!growth_model.IsInitialized()) {
    out << YAML::EndSeq;
    return;
  }

  auto& graph = growth_model.graph;
  graph.SortLists();
  graph.CalculateFlows();

  for (const auto flow_handle : graph.PeekSortedFlowList()) {
    const auto& flow = graph.PeekFlow(flow_handle);
    out << YAML::BeginMap;
    out << YAML::Key << "I" << YAML::Value << flow_handle;
    out << YAML::Key << "PI" << YAML::Value << flow.GetParentHandle();
    out << YAML::Key << "SP" << YAML::Value << flow.info.global_start_position;
    out << YAML::Key << "EP" << YAML::Value << flow.info.global_end_position;
    out << YAML::Key << "ST" << YAML::Value << flow.info.start_thickness;
    out << YAML::Key << "ET" << YAML::Value << flow.info.end_thickness;
    out << YAML::EndMap;
  }

  out << YAML::EndSeq;
}

void SorghumLS::ExportFlowGraph(const std::filesystem::path& path) {
  try {
    YAML::Emitter out;
    out << YAML::BeginMap;
    ExportFlowGraph(out);
    out << YAML::EndMap;

    std::ofstream output_file(path.string());
    output_file << out.c_str();
    output_file.flush();
  } catch (const std::exception& e) {
    EVOENGINE_ERROR(std::string("Failed to save: ") + e.what());
  }
}

void SorghumLS::ExportNodeGraph(YAML::Emitter& out) {
  out << YAML::Key << "Nodes" << YAML::Value << YAML::BeginSeq;

  if (!growth_model.IsInitialized()) {
    out << YAML::EndSeq;
    return;
  }

  auto& graph = growth_model.graph;
  graph.SortLists();
  graph.CalculateFlows();

  for (const auto node_handle : graph.PeekSortedNodeList()) {
    const auto& node = graph.PeekNode(node_handle);
    out << YAML::BeginMap;
    out << YAML::Key << "I" << YAML::Value << node_handle;
    out << YAML::Key << "PI" << YAML::Value << node.GetParentHandle();
    out << YAML::Key << "FI" << YAML::Value << node.GetFlowHandle();
    out << YAML::Key << "S" << YAML::Value << node.symbol_id;
    out << YAML::Key << "SP" << YAML::Value << node.info.global_position;
    out << YAML::Key << "EP" << YAML::Value << node.info.GetGlobalEndPosition();
    out << YAML::Key << "D" << YAML::Value << node.info.GetGlobalDirection();
    out << YAML::Key << "T" << YAML::Value << node.info.thickness;
    out << YAML::EndMap;
  }

  out << YAML::EndSeq;
}

void SorghumLS::ExportNodeGraph(const std::filesystem::path& path) {
  try {
    YAML::Emitter out;
    out << YAML::BeginMap;
    ExportNodeGraph(out);
    out << YAML::EndMap;

    std::ofstream output_file(path.string());
    output_file << out.c_str();
    output_file.flush();
  } catch (const std::exception& e) {
    EVOENGINE_ERROR(std::string("Failed to save: ") + e.what());
  }
}

void SorghumLS::Start() {
  if (!growth_model.IsInitialized() && target_gdd > 0.0f && descriptor_ref.Get<SorghumLSDescriptor>()) {
    GenerateGeometryEntities();
  }
}

void SorghumLS::OnDestroy() {
  ClearGeometryEntities();
}

bool l_system_package::InspectSorghumLS(InspectorContext& context, SorghumLS& sorghum) {
  const auto& editor_layer = context.editor_layer;
  auto& descriptor_ref = sorghum.descriptor_ref;
  auto& seed = sorghum.seed;
  auto& target_gdd = sorghum.target_gdd;
  auto& leaf_mesh_settings = sorghum.leaf_mesh_settings;
  auto& leaf_bottom_face = sorghum.leaf_bottom_face;
  auto& growth_model = sorghum.growth_model;
  const auto last_node_count = sorghum.last_node_count;
  const auto last_internode_count = sorghum.last_internode_count;
  const auto last_leaf_count = sorghum.last_leaf_count;
  const auto last_live_leaf_count = sorghum.last_live_leaf_count;
  bool changed = false;

  if (editor_layer->DragAndDropButton<SorghumLSDescriptor>(descriptor_ref, "Descriptor")) {
    changed = true;
  }

  int seed_int = static_cast<int>(seed);
  if (ImGui::DragInt("Seed", &seed_int, 1, 0, 999999)) {
    seed = static_cast<unsigned int>(seed_int);
    changed = true;
  }

  if (ImGui::DragFloat("Target GDD", &target_gdd, 1.0f, 0.0f, 10000.0f)) {
    target_gdd = std::max(0.0f, target_gdd);
    changed = true;
  }

  if (ImGui::TreeNode("Leaf meshing")) {
    if (ImGui::DragFloat("Vertical subdivision", &leaf_mesh_settings.vertical_subdivision_length,
                         0.001f, 0.0001f, 1.0f, "%.4f")) {
      leaf_mesh_settings.vertical_subdivision_length =
          std::max(0.0001f, leaf_mesh_settings.vertical_subdivision_length);
      changed = true;
    }
    if (ImGui::DragInt("Horizontal subdivision", &leaf_mesh_settings.horizontal_subdivision_step,
                       1, 2, 128)) {
      leaf_mesh_settings.horizontal_subdivision_step =
          std::max(2, leaf_mesh_settings.horizontal_subdivision_step);
      changed = true;
    }
    if (ImGui::Checkbox("Leaf sheath", &leaf_mesh_settings.enable_leaf_sheath)) {
      changed = true;
    }
    if (ImGui::DragFloat("Leaf thickness", &leaf_mesh_settings.leaf_thickness,
                         0.0001f, 0.0f, 0.1f, "%.4f")) {
      leaf_mesh_settings.leaf_thickness = std::max(0.0f, leaf_mesh_settings.leaf_thickness);
      changed = true;
    }
    if (ImGui::Checkbox("Leaf bottom face", &leaf_bottom_face)) {
      changed = true;
    }
    ImGui::TreePop();
  }

  if (ImGui::Button("Generate")) {
    sorghum.GenerateGeometryEntities();
    changed = true;
  }
  ImGui::SameLine();
  if (ImGui::Button("Clear")) {
    sorghum.ClearGeometryEntities();
    changed = true;
  }

  if (growth_model.IsInitialized()) {
    ImGui::Separator();
    ImGui::Text("GDD: %.1f", growth_model.accumulated_gdd);
    ImGui::Text("Topology: %s", growth_model.IsTopologyComplete() ? "Complete" : "Pending");
    ImGui::Text("Nodes: %u", last_node_count);
    ImGui::Text("Internodes: %u", last_internode_count);
    ImGui::Text("Leaves: %u (live %u)", last_leaf_count, last_live_leaf_count);
  }

  return changed;
}

void l_system_package::SerializeSorghumLS(YAML::Emitter& out, const SorghumLS& target) {
  const auto& descriptor_ref = target.descriptor_ref;
  const auto seed = target.seed;
  const auto target_gdd = target.target_gdd;
  const auto leaf_bottom_face = target.leaf_bottom_face;
  const auto& leaf_mesh_settings = target.leaf_mesh_settings;
  descriptor_ref.Save("descriptor_ref", out);
  out << YAML::Key << "seed" << YAML::Value << seed;
  out << YAML::Key << "target_gdd" << YAML::Value << target_gdd;
  out << YAML::Key << "leaf_bottom_face" << YAML::Value << leaf_bottom_face;
  out << YAML::Key << "leaf_vertical_subdivision_length" << YAML::Value
      << leaf_mesh_settings.vertical_subdivision_length;
  out << YAML::Key << "leaf_horizontal_subdivision_step" << YAML::Value
      << leaf_mesh_settings.horizontal_subdivision_step;
  out << YAML::Key << "leaf_enable_sheath" << YAML::Value
      << leaf_mesh_settings.enable_leaf_sheath;
  out << YAML::Key << "leaf_thickness" << YAML::Value
      << leaf_mesh_settings.leaf_thickness;
}

void l_system_package::DeserializeSorghumLS(const YAML::Node& in, SorghumLS& target) {
  auto& descriptor_ref = target.descriptor_ref;
  auto& seed = target.seed;
  auto& target_gdd = target.target_gdd;
  auto& leaf_bottom_face = target.leaf_bottom_face;
  auto& leaf_mesh_settings = target.leaf_mesh_settings;
  descriptor_ref.Load("descriptor_ref", in);
  if (in["seed"]) seed = in["seed"].as<unsigned int>();
  if (in["target_gdd"]) target_gdd = std::max(0.0f, in["target_gdd"].as<float>());
  if (in["leaf_bottom_face"]) leaf_bottom_face = in["leaf_bottom_face"].as<bool>();
  if (in["leaf_vertical_subdivision_length"]) {
    leaf_mesh_settings.vertical_subdivision_length = std::max(
        0.0001f, in["leaf_vertical_subdivision_length"].as<float>());
  }
  if (in["leaf_horizontal_subdivision_step"]) {
    leaf_mesh_settings.horizontal_subdivision_step = std::max(
        2, in["leaf_horizontal_subdivision_step"].as<int>());
  }
  if (in["leaf_enable_sheath"]) {
    leaf_mesh_settings.enable_leaf_sheath = in["leaf_enable_sheath"].as<bool>();
  }
  if (in["leaf_thickness"]) {
    leaf_mesh_settings.leaf_thickness = std::max(0.0f, in["leaf_thickness"].as<float>());
  }
}

void SorghumLS::CollectAssetRef(std::vector<AssetRef>& list) {
  list.push_back(descriptor_ref);
}
