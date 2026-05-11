#include "MaizeTassel.hpp"
#include "MaizeTasselDescriptor.hpp"

#include <AssetManager.hpp>
#include <EditorLayer.hpp>
#include <Material.hpp>
#include <Mesh.hpp>
#include <Particles.hpp>
#include <Scene.hpp>
#include <Times.hpp>
#include <Transform.hpp>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <fstream>
#include <unordered_set>

#ifdef LSYSTEM_GPU_PIPELINE
#include "gpu/LSystemGPUEngine.hpp"
#include "gpu/TasselInstancePacker.hpp"
#include "gpu/TasselGrowthPacker.hpp"
#endif

using namespace l_system_plugin;
using namespace evo_engine;

namespace {
MaizeTassel::ColorMode g_maize_tassel_color_mode = MaizeTassel::ColorMode::Shaded;
std::atomic<bool> g_force_cpu_particles_path{false};

bool IsFiniteQuat(const glm::quat& q) {
  return std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w);
}

bool IsFiniteVec3(const glm::vec3& v) {
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
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

bool IsInternodeNode(const TasselNode& node) {
  return node.data.Is<TasselInternode>();
}

LNodeHandle FindParentInternodeNodeHandle(const TasselGraph& graph, const LNodeHandle node_handle) {
  auto parent_handle = graph.PeekNode(node_handle).GetParentHandle();
  while (parent_handle >= 0) {
    const auto& parent = graph.PeekNode(parent_handle);
    if (IsInternodeNode(parent)) {
      return parent_handle;
    }
    parent_handle = parent.GetParentHandle();
  }
  return -1;
}

std::unordered_set<LFlowHandle> CollectInternodeFlowHandles(const TasselGraph& graph) {
  std::unordered_set<LFlowHandle> retained_flow_handles;
  for (const auto flow_handle : graph.PeekSortedFlowList()) {
    const auto& flow = graph.PeekFlow(flow_handle);
    const auto& node_handles = flow.PeekNodeHandles();
    if (node_handles.empty()) {
      continue;
    }

    bool has_internode = false;
    for (const auto node_handle : node_handles) {
      if (IsInternodeNode(graph.PeekNode(node_handle))) {
        has_internode = true;
      }
    }

    if (has_internode) {
      retained_flow_handles.emplace(flow_handle);
    }
  }
  return retained_flow_handles;
}

LFlowHandle FindParentInternodeFlowHandle(const TasselGraph& graph, const LFlowHandle flow_handle,
                                          const std::unordered_set<LFlowHandle>& retained_flow_handles) {
  auto parent_flow_handle = graph.PeekFlow(flow_handle).GetParentHandle();
  while (parent_flow_handle >= 0) {
    if (retained_flow_handles.find(parent_flow_handle) != retained_flow_handles.end()) {
      return parent_flow_handle;
    }
    parent_flow_handle = graph.PeekFlow(parent_flow_handle).GetParentHandle();
  }
  return -1;
}

void AppendParticlesToMesh(const std::shared_ptr<Scene>& scene, const Entity& entity,
                           std::vector<Vertex>& out_vertices, std::vector<glm::uvec3>& out_triangles) {
  if (!scene->IsEntityValid(entity) || !scene->HasPrivateComponent<Particles>(entity)) {
    return;
  }

  const auto particles = scene->GetOrSetPrivateComponent<Particles>(entity).lock();
  if (!particles) {
    return;
  }

  const auto mesh = particles->mesh.Get<Mesh>();
  const auto particle_info_list = particles->particle_info_list.Get<ParticleInfoList>();
  if (!mesh || !particle_info_list) {
    return;
  }

  const auto& source_vertices = mesh->UnsafeGetVertices();
  const auto& source_triangles = mesh->UnsafeGetTriangles();
  const auto& instances = particle_info_list->PeekParticleInfoList();
  if (source_vertices.empty() || source_triangles.empty() || instances.empty()) {
    return;
  }

  const auto entity_global_transform = scene->GetDataComponent<GlobalTransform>(entity);
  for (const auto& instance : instances) {
    const glm::mat4 world_transform = entity_global_transform.value * instance.instance_matrix.value;
    if (!IsFiniteMat4(world_transform)) {
      continue;
    }

    const glm::mat3 world_3x3(world_transform);
    glm::mat3 normal_transform(1.0f);
    const float determinant = glm::determinant(world_3x3);
    if (std::isfinite(determinant) && std::abs(determinant) > 1e-8f) {
      normal_transform = glm::transpose(glm::inverse(world_3x3));
    }

    const auto vertex_offset = static_cast<uint32_t>(out_vertices.size());
    out_vertices.reserve(out_vertices.size() + source_vertices.size());
    out_triangles.reserve(out_triangles.size() + source_triangles.size());

    for (const auto& source_vertex : source_vertices) {
      Vertex vertex = source_vertex;
      vertex.position = glm::vec3(world_transform * glm::vec4(source_vertex.position, 1.0f));

      const glm::vec3 transformed_normal = normal_transform * source_vertex.normal;
      if (IsFiniteVec3(transformed_normal) && glm::length(transformed_normal) > 1e-8f) {
        vertex.normal = glm::normalize(transformed_normal);
      }

      const glm::vec3 transformed_tangent = normal_transform * source_vertex.tangent;
      if (IsFiniteVec3(transformed_tangent) && glm::length(transformed_tangent) > 1e-8f) {
        vertex.tangent = glm::normalize(transformed_tangent);
      }

      vertex.color = instance.instance_color;
      out_vertices.emplace_back(vertex);
    }

    for (const auto& source_triangle : source_triangles) {
      out_triangles.emplace_back(vertex_offset + source_triangle.x,
                                 vertex_offset + source_triangle.y,
                                 vertex_offset + source_triangle.z);
    }
  }
}
}  // namespace

// ---------------------------------------------------------------------------

void MaizeTassel::SetGlobalColorMode(const ColorMode mode) {
  g_maize_tassel_color_mode = mode;
}

MaizeTassel::ColorMode MaizeTassel::GetGlobalColorMode() {
  return g_maize_tassel_color_mode;
}

void MaizeTassel::SetForceCpuParticlesPath(const bool force) {
  g_force_cpu_particles_path.store(force, std::memory_order_relaxed);
}

bool MaizeTassel::IsForceCpuParticlesPath() {
  return g_force_cpu_particles_path.load(std::memory_order_relaxed);
}

// ---------------------------------------------------------------------------

void MaizeTassel::ClearGeometryEntities() const {
  const auto scene = GetScene();
  const auto self = GetOwner();
  const auto children = scene->GetChildren(self);
  for (const auto& child : children) {
    const auto name = scene->GetEntityName(child);
    if (name == "Tassel Internodes" || name == "Tassel Spikelets" ||
        name == "Tassel Stem Mesh") {
      scene->DeleteEntity(child);
    }
  }
}

// ---------------------------------------------------------------------------
// Spikelet mesh generation (elongated ellipsoid)
// ---------------------------------------------------------------------------

static void GenerateSpikeletMesh(std::vector<Vertex>& vertices,
                                  std::vector<unsigned int>& indices,
                                  int stacks, int slices) {
  vertices.clear();
  indices.clear();

  const glm::vec4 spikelet_color(0.85f, 0.78f, 0.35f, 1.0f);

  for (int i = 0; i <= stacks; i++) {
    const float phi = glm::pi<float>() * static_cast<float>(i) / static_cast<float>(stacks);
    const float sin_phi = std::sin(phi);
    const float cos_phi = std::cos(phi);
    for (int j = 0; j <= slices; j++) {
      const float theta = glm::two_pi<float>() * static_cast<float>(j) / static_cast<float>(slices);
      const float sin_theta = std::sin(theta);
      const float cos_theta = std::cos(theta);

      Vertex v;
      v.position = glm::vec3(0.5f * sin_phi * cos_theta,
                              1.0f * cos_phi,
                              0.5f * sin_phi * sin_theta);
      v.normal = glm::normalize(glm::vec3(v.position.x / (0.5f * 0.5f),
                                           v.position.y / (1.0f * 1.0f),
                                           v.position.z / (0.5f * 0.5f)));
      v.color = spikelet_color;
      v.tex_coord = glm::vec2(static_cast<float>(j) / static_cast<float>(slices),
                               static_cast<float>(i) / static_cast<float>(stacks));
      vertices.push_back(v);
    }
  }

  for (int i = 0; i < stacks; i++) {
    for (int j = 0; j < slices; j++) {
      const unsigned int a = static_cast<unsigned int>(i * (slices + 1) + j);
      const unsigned int b = a + static_cast<unsigned int>(slices + 1);
      indices.push_back(a);
      indices.push_back(b);
      indices.push_back(a + 1);
      indices.push_back(a + 1);
      indices.push_back(b);
      indices.push_back(b + 1);
    }
  }
}

// ---------------------------------------------------------------------------
// Unit cylinder mesh (radius=1, height=1, along +Y, centered at origin)
// ---------------------------------------------------------------------------

static void GenerateUnitCylinderMesh(std::vector<Vertex>& vertices,
                                     std::vector<unsigned int>& indices,
                                     int segments = 6) {
  vertices.clear();
  indices.clear();

  const glm::vec4 stem_color(0.45f, 0.55f, 0.2f, 1.0f);
  const float angle_step = glm::two_pi<float>() / static_cast<float>(segments);

  // Two rings: bottom (y=0) and top (y=1).
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

  // Triangles between the two rings.
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

// ---------------------------------------------------------------------------
// Collect spikelet-pair geometry from graph
// ---------------------------------------------------------------------------

struct PairEllipsoidInstance {
  glm::vec3 position = glm::vec3(0.0f);
  glm::quat rotation = glm::quat(1, 0, 0, 0);
  glm::vec3 scale = glm::vec3(0.0f);
  uint32_t node_index = 0;
  bool distal = false;
};

struct PairInternodeInstance {
  glm::mat4 model = glm::mat4(1.0f);
  uint32_t node_index = 0;
};

static glm::quat MakeYToDirectionRotation(const glm::vec3& direction) {
  const glm::vec3 y_axis(0.0f, 1.0f, 0.0f);
  const glm::vec3 dir = glm::normalize(direction);
  const float dot = std::clamp(glm::dot(y_axis, dir), -1.0f, 1.0f);
  if (dot > 0.9999f) {
    return glm::quat(1, 0, 0, 0);
  }
  if (dot < -0.9999f) {
    return glm::angleAxis(glm::pi<float>(), glm::vec3(1.0f, 0.0f, 0.0f));
  }
  const glm::vec3 axis = glm::normalize(glm::cross(y_axis, dir));
  const float angle = std::acos(dot);
  return glm::normalize(glm::angleAxis(angle, axis));
}

static void CollectSpikeletPairsFromGraph(
    const TasselGraph& graph,
    std::vector<PairEllipsoidInstance>& ellipsoids,
    std::vector<PairInternodeInstance>& pair_internodes) {
  ellipsoids.clear();
  pair_internodes.clear();

  const auto& sorted = graph.PeekSortedNodeList();
  for (const auto handle : sorted) {
    const auto& node = graph.PeekNode(handle);
    if (!node.data.Is<TasselSpikeletPair>()) {
      continue;
    }
    const auto& pair = node.data.Get<TasselSpikeletPair>();

    const glm::vec3 base_pos = node.info.global_position;
    const glm::vec3 axis_dir = glm::normalize(node.info.global_rotation * glm::vec3(0, 0, -1));
    glm::vec3 radial_dir = node.info.global_rotation * glm::vec3(1, 0, 0);
    radial_dir -= axis_dir * glm::dot(radial_dir, axis_dir);
    float radial_len = glm::length(radial_dir);
    if (radial_len <= 1e-5f) {
      radial_dir = glm::cross(axis_dir, glm::vec3(0, 1, 0));
      radial_len = glm::length(radial_dir);
      if (radial_len <= 1e-5f) {
        radial_dir = glm::cross(axis_dir, glm::vec3(1, 0, 0));
        radial_len = glm::length(radial_dir);
      }
    }
    if (radial_len > 1e-5f) {
      radial_dir /= radial_len;
    } else {
      radial_dir = glm::vec3(1.0f, 0.0f, 0.0f);
    }

    const float proximal_angle_rad = glm::radians(pair.proximal_outward_angle);
    const glm::vec3 proximal_dir = glm::normalize(axis_dir * std::cos(proximal_angle_rad) +
                                                  radial_dir * std::sin(proximal_angle_rad));
    const float proximal_half_height = std::max(0.0f, pair.proximal_scale.y);
    const glm::vec3 proximal_bottom = base_pos;
    const glm::vec3 proximal_pos = proximal_bottom + proximal_dir * proximal_half_height;

    const float internode_angle_rad = glm::radians(pair.pair_internode_angle);
    const glm::vec3 internode_dir = glm::normalize(proximal_dir * std::cos(internode_angle_rad) +
                                                   radial_dir * std::sin(internode_angle_rad));
    const glm::vec3 internode_start = base_pos;
    const glm::vec3 internode_end = internode_start + internode_dir * std::max(0.0f, pair.pair_internode_length);

    const float distal_angle_rad = glm::radians(pair.distal_outward_angle);
    const glm::vec3 distal_dir = glm::normalize(internode_dir * std::cos(distal_angle_rad) +
                                                radial_dir * std::sin(distal_angle_rad));
    const float distal_half_height = std::max(0.0f, pair.distal_scale.y);
    const glm::vec3 distal_bottom = internode_end;
    const glm::vec3 distal_pos = distal_bottom + distal_dir * distal_half_height;

    PairEllipsoidInstance proximal{};
    proximal.position = proximal_pos;
    proximal.rotation = MakeYToDirectionRotation(proximal_dir);
    proximal.scale = pair.proximal_scale;
    proximal.node_index = static_cast<uint32_t>(node.GetIndex());
    proximal.distal = false;
    ellipsoids.push_back(proximal);

    PairEllipsoidInstance distal{};
    distal.position = distal_pos;
    distal.rotation = MakeYToDirectionRotation(distal_dir);
    distal.scale = pair.distal_scale;
    distal.node_index = static_cast<uint32_t>(node.GetIndex());
    distal.distal = true;
    ellipsoids.push_back(distal);

    if (pair.pair_internode_length > 0.0f && pair.pair_internode_thickness > 0.0f) {
      PairInternodeInstance pair_internode{};
      const glm::quat rot = MakeYToDirectionRotation(internode_dir);
      pair_internode.model = glm::translate(internode_start) * glm::mat4_cast(rot) *
                             glm::scale(glm::vec3(pair.pair_internode_thickness * 0.5f,
                                                  pair.pair_internode_length,
                                                  pair.pair_internode_thickness * 0.5f));
      pair_internode.node_index = static_cast<uint32_t>(node.GetIndex());
      pair_internodes.push_back(pair_internode);
    }
  }
}

// ---------------------------------------------------------------------------
// Generate geometry entities
// ---------------------------------------------------------------------------

void MaizeTassel::GenerateGeometryEntities(const bool uncapped_growth) {
  // Explicitly clear before full reset so RebuildGeometry creates entities fresh.
  ClearGeometryEntities();
  growth_model.Reset();
  GrowToTargetGDD(uncapped_growth);
}

void MaizeTassel::GeneratePreviewGeometryEntities(const float preview_target_gdd,
                                                  const uint32_t preview_max_growth_steps) {
  // Preview always replays from a clean state so topology-affecting edits are visible.
  ClearGeometryEntities();
  growth_model.Reset();

  const double grow_start = Times::Now();
  auto descriptor = descriptor_ref.Get<MaizeTasselDescriptor>();
  if (!descriptor) {
    last_grow_seconds = 0.0;
    return;
  }

  growth_model.Initialize(*descriptor, seed);

  const float clamped_target_gdd = std::min(target_gdd, std::max(0.0f, preview_target_gdd));
  const uint32_t step_cap = std::max(1u, preview_max_growth_steps);
  growth_model.GrowToGDD(clamped_target_gdd, step_cap);

  last_grow_seconds = Times::Now() - grow_start;
  RebuildGeometry();
}

// ---------------------------------------------------------------------------
// Grow to target GDD (lazy init, handles backward scrubbing)
// ---------------------------------------------------------------------------

void MaizeTassel::GrowToTargetGDD(const bool uncapped_growth) {
  const double grow_start = Times::Now();
  auto descriptor = descriptor_ref.Get<MaizeTasselDescriptor>();
  if (!descriptor) {
    last_grow_seconds = 0.0;
    return;
  }

  // Lazy init.
  if (!growth_model.IsInitialized()) {
    growth_model.Initialize(*descriptor, seed);
  }

  // Backward scrubbing: re-init and re-grow from zero.
  // The growth model advances in fixed GDD steps, so accumulated_gdd can be
  // up to one step ahead of target_gdd due to quantization. Treat only larger
  // backward moves as true scrubbing to avoid periodic full re-initialization.
  const float gdd_step = std::max(1e-5f, growth_model.gdd_per_growth_step);
  if (target_gdd + gdd_step < growth_model.accumulated_gdd) {
    growth_model.Initialize(*descriptor, seed);
  }

  growth_model.GrowToGDD(target_gdd, uncapped_growth ? 0u : max_growth_steps_per_frame);
  last_grow_seconds = Times::Now() - grow_start;
  RebuildGeometry();
}

void MaizeTassel::SetSeasonalChronologicalMode(
    const bool enable_independent_chronological_clock) {
  growth_model.SetChronologicalCoupledToThermal(
      !enable_independent_chronological_clock);
}

bool MaizeTassel::AdvanceChronologicalAging(const float delta_years) {
  if (!std::isfinite(delta_years) || delta_years <= 0.0f) {
    return false;
  }

  auto descriptor = descriptor_ref.Get<MaizeTasselDescriptor>();
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

// ---------------------------------------------------------------------------
// Rebuild geometry from current growth model state (no re-initialization)
// ---------------------------------------------------------------------------

void MaizeTassel::RebuildGeometry() {
  const double rebuild_start = Times::Now();
  last_rebuild_internode_collect_seconds = 0.0;
  last_rebuild_internode_upload_seconds = 0.0;
  last_rebuild_spikelet_collect_seconds = 0.0;
  last_rebuild_spikelet_upload_seconds = 0.0;
  last_invalid_instance_count = 0;
  last_internode_count = 0;
  last_spikelet_count = 0;
  last_node_count = 0;

  // NOTE: Do NOT call ClearGeometryEntities() here.
  // We reuse existing child entities so their GlobalTransforms remain stable
  // across CTRL+F growth frames. ClearGeometryEntities() is only called on
  // explicit full resets (GenerateGeometryEntities / OnDestroy).

  if (!growth_model.IsInitialized()) {
    last_rebuild_seconds = 0.0;
    return;
  }

  const auto scene = GetScene();
  const auto owner = GetOwner();
  const auto color_mode = GetGlobalColorMode();
  const glm::vec4 instance_color = HashToColor(owner.GetIndex());

  // Reuse temporary buffers across rebuild calls to reduce allocator churn.
  static thread_local std::vector<PairEllipsoidInstance> pair_ellipsoid_instances_cache;
  static thread_local std::vector<PairInternodeInstance> pair_internode_instances_cache;
  static thread_local std::vector<ParticleInfo> internode_infos_cache;
  static thread_local std::vector<ParticleInfo> spikelet_infos_cache;

  // --- Find existing geometry child entities ---
  Entity internode_entity, spikelet_entity;
  for (const auto& child : scene->GetChildren(owner)) {
    const auto name = scene->GetEntityName(child);
    if (name == "Tassel Stem Mesh") {
      // Remove stale legacy mesh entity so only particle-based tassel geometry is shown.
      scene->DeleteEntity(child);
      continue;
    }
    if (name == "Tassel Internodes")
      internode_entity = child;
    else if (name == "Tassel Spikelets")
      spikelet_entity = child;
  }

  auto& pair_ellipsoid_instances = pair_ellipsoid_instances_cache;
  auto& pair_internode_instances = pair_internode_instances_cache;
  CollectSpikeletPairsFromGraph(growth_model.graph, pair_ellipsoid_instances, pair_internode_instances);

  // --- Internodes (Particles — one unit cylinder instance per internode) ---
  {
    const double collect_start = Times::Now();
    const auto& sorted = growth_model.graph.PeekSortedNodeList();
    last_node_count = static_cast<uint32_t>(sorted.size());
    auto& infos = internode_infos_cache;
    infos.clear();
    infos.reserve(sorted.size() + pair_internode_instances.size());
    // Tassel node rotations align local +Z to growth direction. Our unit cylinder
    // is authored along local +Y, so rotate +Y -> -Z before applying node rotation.
    const glm::quat cylinder_axis_fix = glm::angleAxis(
      -glm::half_pi<float>(), glm::vec3(1.0f, 0.0f, 0.0f));

    for (const auto handle : sorted) {
      const auto& node = growth_model.graph.PeekNode(handle);
      if (!node.data.Is<TasselInternode>())
        continue;
      if (node.info.length <= 0.0f)
        continue;
      if (!IsFiniteVec3(node.info.global_position) || !std::isfinite(node.info.length) ||
          !std::isfinite(node.info.thickness)) {
        last_invalid_instance_count++;
        continue;
      }

      const float half_thick = node.info.thickness * 0.5f;
      if (half_thick <= 0.0f) {
        continue;
      }

      glm::quat instance_rotation =
          glm::normalize(node.info.global_rotation * cylinder_axis_fix);
      if (!IsFiniteQuat(instance_rotation)) {
        instance_rotation = glm::quat(1, 0, 0, 0);
        last_invalid_instance_count++;
      }
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
      if (color_mode == ColorMode::ByNode) {
        pi.instance_color = HashToColor(static_cast<uint32_t>(node.GetIndex()));
      } else if (color_mode == ColorMode::ByInstance) {
        pi.instance_color = instance_color;
      } else if (color_mode == ColorMode::ByType) {
        pi.instance_color = glm::vec4(0.22f, 0.82f, 0.33f, 1.0f);
      } else {
        pi.instance_color = glm::vec4(0.45f, 0.55f, 0.2f, 1.0f);
      }
      infos.push_back(pi);
    }

    for (const auto& pair_internode : pair_internode_instances) {
      if (!IsFiniteMat4(pair_internode.model)) {
        last_invalid_instance_count++;
        continue;
      }
      ParticleInfo pi;
      pi.instance_matrix.value = pair_internode.model;
      if (color_mode == ColorMode::ByNode) {
        pi.instance_color = HashToColor(pair_internode.node_index);
      } else if (color_mode == ColorMode::ByInstance) {
        pi.instance_color = instance_color;
      } else if (color_mode == ColorMode::ByType) {
        pi.instance_color = glm::vec4(0.30f, 0.70f, 0.25f, 1.0f);
      } else {
        pi.instance_color = glm::vec4(0.38f, 0.50f, 0.22f, 1.0f);
      }
      infos.push_back(pi);
    }

    last_rebuild_internode_collect_seconds = Times::Now() - collect_start;

    last_internode_count = static_cast<uint32_t>(infos.size());

    // Scanner-compat runtime branch. When IsForceCpuParticlesPath() is true
    // (set by headless DatasetGenerator::GenerateDataForTassel), fall through
    // to the legacy CPU `Particles` "Tassel Internodes" entity so that
    // TasselPointCloudScanner / RenderInstanceStorage can see the geometry.
    // Otherwise (interactive editing default), use the Phase 2b.3 GPU path.
#ifdef LSYSTEM_GPU_PIPELINE
    const bool use_gpu_internodes = !IsForceCpuParticlesPath();
#else
    constexpr bool use_gpu_internodes = false;
#endif

    if (use_gpu_internodes) {
#ifdef LSYSTEM_GPU_PIPELINE
    // ---- Phase 2b.3: GPU pack path ----------------------------------------
    // Replaces the per-frame CPU walk in PackTasselInternodes with a GPU
    // compute dispatch. Flow per frame:
    //   1. CPU `PackTasselGrowth` flattens the live TasselGraph into the
    //      growth SoA (one walk; cheap O(N)). global_position /
    //      global_rotation / length_thickness are seeded from
    //      `node.info.*` so the GPU pack reads CPU-truth values without
    //      needing to run grow.comp / propagate.comp.
    //   2. `UploadTasselGrowth` pushes the SoA into per-instance SSBOs
    //      (the engine's growth_layout descriptor set).
    //   3. `DispatchPackInternodesGpu` runs tassel_pack_internodes.comp
    //      to write the existing TasselInternodeInstance SSBO that the
    //      mesh shader already binds. Bit-equivalent to the CPU pack
    //      output for the current runtime path.
    //
    // grow.comp / propagate.comp are intentionally NOT dispatched yet —
    // the CPU growth_model remains the source of truth for length /
    // pose. Phase 3+ will flip the dependency direction (GPU drives,
    // CPU shadows for diagnostics).
    //
    // The CPU `Particles` entity is NOT created in this path — the
    // deferred-rendering callback registered from LSystemLayer::Update()
    // draws directly from the SSBO via tassel_internode.{task,mesh,frag}.
    // Any pre-existing CPU-path child entity is deleted to avoid
    // double-rendering.
    if (scene->IsEntityValid(internode_entity)) {
      scene->DeleteEntity(internode_entity);
      internode_entity = {};
    }
    {
      static thread_local gpu::TasselGrowthSoA gpu_growth_cache;
      auto& soa = gpu_growth_cache;

      gpu::PackGrowthOptions growth_opts;
      growth_opts.instance_id = gpu_instance_id;
      // Pack-only path doesn't read local_rotation (propagate.comp isn't
      // dispatched), but the packer requires a non-null function. Pass
      // identity; if a future revision dispatches propagate, swap this
      // for the real per-type rotation function from GeometryPass.
      growth_opts.local_rotation_fn = [](const auto&, const auto&) {
        return glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
      };
      growth_opts.reject_non_finite = false;
      const double pack_start = Times::Now();
      const gpu::PackGrowthResult growth_result =
          gpu::PackTasselGrowth(growth_model.graph, growth_opts, soa);
      (void)growth_result;
      last_rebuild_internode_collect_seconds += (Times::Now() - pack_start);

      if (gpu_instance_id == 0u) {
        gpu_instance_id = gpu::LSystemGPUEngine::Get().CreateInstance("MaizeTassel", seed);
      }

      const double upload_start = Times::Now();
      auto& engine = gpu::LSystemGPUEngine::Get();
      engine.UploadTasselGrowth(gpu_instance_id, soa);
      const float ic[4] = {instance_color.r, instance_color.g, instance_color.b, instance_color.a};
      const bool ok = engine.DispatchPackInternodesGpu(
          gpu_instance_id,
          static_cast<uint32_t>(color_mode),
          ic);
      // Fallback: if the GPU pack pipeline failed to initialize (e.g.
      // shader compile error at startup), drop to the legacy CPU pack
      // so the visualization still renders. This keeps headless
      // runs and broken-driver environments alive.
      if (!ok) {
        static thread_local std::vector<gpu::TasselInternodeInstance> gpu_pack_cache;
        auto& packed = gpu_pack_cache;
        packed.clear();
        gpu::PackInternodesOptions opts;
        opts.color_mode = static_cast<gpu::ColorMode>(static_cast<int>(color_mode));
        opts.instance_color = instance_color;
        opts.include_pair_internodes = true;
        gpu::PackTasselInternodes(growth_model.graph, opts, packed);
        engine.UploadTasselInternodes(gpu_instance_id, packed.data(),
                                      static_cast<uint32_t>(packed.size()));
      }
      last_rebuild_internode_upload_seconds = Times::Now() - upload_start;
    }
#endif  // LSYSTEM_GPU_PIPELINE (GPU branch body)
    } else {
#ifdef LSYSTEM_GPU_PIPELINE
    // Scanner-compat CPU path. If the GPU SSBO was previously populated for
    // this instance, zero it so the deferred mesh-shader callback draws
    // nothing and we don't double-render alongside the CPU Particles entity.
    if (gpu_instance_id != 0u) {
      gpu::LSystemGPUEngine::Get().UploadTasselInternodes(gpu_instance_id, nullptr, 0);
    }
#endif
    if (!infos.empty()) {
      std::shared_ptr<Particles> particles;
      std::shared_ptr<ParticleInfoList> particle_info_list;

      if (!scene->IsEntityValid(internode_entity)) {
        internode_entity = scene->CreateEntity("Tassel Internodes");
        particles = scene->GetOrSetPrivateComponent<Particles>(internode_entity).lock();

        const auto mesh = AssetManager::CreateTemporaryAsset<Mesh>();
        std::vector<Vertex> cyl_verts;
        std::vector<unsigned int> cyl_idx;
        GenerateUnitCylinderMesh(cyl_verts, cyl_idx);
        VertexAttributes attrs{};
        attrs.normal = true;
        attrs.color = true;
        attrs.tex_coord = true;
        mesh->SetVertices(attrs, cyl_verts, cyl_idx);

        const auto material = AssetManager::CreateTemporaryAsset<Material>();
        material->vertex_color_only = true;

        particle_info_list = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
        particles->mesh = mesh;
        particles->material = material;
        particles->particle_info_list = particle_info_list;
        scene->SetParent(internode_entity, owner);
      } else {
        particles = scene->GetOrSetPrivateComponent<Particles>(internode_entity).lock();
        particle_info_list = particles->particle_info_list.Get<ParticleInfoList>();
        if (!particle_info_list) {
          particle_info_list = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
          particles->particle_info_list = particle_info_list;
        }
      }
      const double upload_start = Times::Now();
      particle_info_list->SetParticleInfos(infos);
      last_rebuild_internode_upload_seconds = Times::Now() - upload_start;
    } else {
      if (scene->IsEntityValid(internode_entity))
        scene->DeleteEntity(internode_entity);
    }
    }  // end runtime CPU branch
  }

  // --- Spikelets (Particles) ---
  {
    const double collect_start = Times::Now();
    last_spikelet_count = static_cast<uint32_t>(pair_ellipsoid_instances.size());

    if (!pair_ellipsoid_instances.empty()) {
      std::shared_ptr<Particles> particles;
      std::shared_ptr<ParticleInfoList> particle_info_list;

      auto ensure_spikelet_material = [](const std::shared_ptr<Particles>& p) {
        auto material = p->material.Get<Material>();
        if (!material) {
          material = AssetManager::CreateTemporaryAsset<Material>();
          p->material = material;
        }
        material->vertex_color_only = true;
        material->material_properties.albedo_color = glm::vec3(1.0f);
        material->material_properties.metallic = 0.0f;
        material->material_properties.roughness = 1.0f;
      };

      if (!scene->IsEntityValid(spikelet_entity)) {
        // First time: create entity and permanent assets.
        spikelet_entity = scene->CreateEntity("Tassel Spikelets");
        particles = scene->GetOrSetPrivateComponent<Particles>(spikelet_entity).lock();

        const auto mesh = AssetManager::CreateTemporaryAsset<Mesh>();
        std::vector<Vertex> spk_vertices;
        std::vector<unsigned int> spk_indices;
        GenerateSpikeletMesh(spk_vertices, spk_indices, 6, 8);
        VertexAttributes spk_attrs{};
        spk_attrs.normal = true;
        spk_attrs.color = true;
        spk_attrs.tex_coord = true;
        mesh->SetVertices(spk_attrs, spk_vertices, spk_indices);

        const auto material = AssetManager::CreateTemporaryAsset<Material>();
        material->vertex_color_only = true;
        material->material_properties.albedo_color = glm::vec3(1.0f);
        material->material_properties.metallic = 0.0f;
        material->material_properties.roughness = 1.0f;

        particle_info_list = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
        particles->mesh = mesh;
        particles->material = material;
        particles->particle_info_list = particle_info_list;
        scene->SetParent(spikelet_entity, owner);
      } else {
        // Reuse existing entity — just update the particle data.
        particles = scene->GetOrSetPrivateComponent<Particles>(spikelet_entity).lock();
        particle_info_list = particles->particle_info_list.Get<ParticleInfoList>();
        if (!particle_info_list) {
          particle_info_list = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
          particles->particle_info_list = particle_info_list;
        }
      }

      ensure_spikelet_material(particles);

      auto& infos = spikelet_infos_cache;
      infos.clear();
      infos.resize(pair_ellipsoid_instances.size());
      for (size_t i = 0; i < pair_ellipsoid_instances.size(); i++) {
        const auto& pair_ellipsoid = pair_ellipsoid_instances[i];
        const glm::quat safe_rotation = IsFiniteQuat(pair_ellipsoid.rotation)
            ? glm::normalize(pair_ellipsoid.rotation)
            : glm::quat(1, 0, 0, 0);
        const glm::mat4 model = glm::translate(pair_ellipsoid.position) *
                                glm::mat4_cast(safe_rotation) *
                                glm::scale(pair_ellipsoid.scale);
        if (!IsFiniteMat4(model)) {
          last_invalid_instance_count++;
          infos[i].instance_matrix.value = glm::mat4(1.0f);
        } else {
          infos[i].instance_matrix.value = model;
        }
        if (color_mode == ColorMode::ByNode) {
          infos[i].instance_color = HashToColor(pair_ellipsoid.node_index);
        } else if (color_mode == ColorMode::ByInstance) {
          infos[i].instance_color = instance_color;
        } else if (color_mode == ColorMode::ByType) {
          infos[i].instance_color = glm::vec4(0.95f, 0.61f, 0.12f, 1.0f);
        } else {
          infos[i].instance_color = pair_ellipsoid.distal
              ? glm::vec4(0.80f, 0.75f, 0.30f, 1.0f)
              : glm::vec4(0.85f, 0.78f, 0.35f, 1.0f);
        }
      }
      last_rebuild_spikelet_collect_seconds = Times::Now() - collect_start;
      const double upload_start = Times::Now();
      particle_info_list->SetParticleInfos(infos);
      last_rebuild_spikelet_upload_seconds = Times::Now() - upload_start;
    } else {
      last_rebuild_spikelet_collect_seconds = Times::Now() - collect_start;
      // No spikelets — remove the entity if present.
      if (scene->IsEntityValid(spikelet_entity))
        scene->DeleteEntity(spikelet_entity);
    }
  }

  last_rebuild_seconds = Times::Now() - rebuild_start;
}

// ---------------------------------------------------------------------------

void MaizeTassel::ExportObj(const std::filesystem::path& path) const {
  const auto scene = GetScene();
  if (!scene) {
    return;
  }

  std::vector<Vertex> vertices;
  std::vector<glm::uvec3> triangles;

  const auto owner = GetOwner();
  for (const auto& child : scene->GetChildren(owner)) {
    const auto name = scene->GetEntityName(child);
    if (name == "Tassel Internodes" || name == "Tassel Spikelets") {
      AppendParticlesToMesh(scene, child, vertices, triangles);
    }
  }

  if (vertices.empty() || triangles.empty()) {
    EVOENGINE_ERROR("Tassel mesh export failed: no tassel particle geometry available.");
    return;
  }

  const auto mesh = AssetManager::CreateTemporaryAsset<Mesh>();
  VertexAttributes vertex_attributes{};
  vertex_attributes.normal = true;
  vertex_attributes.tangent = true;
  vertex_attributes.color = true;
  vertex_attributes.tex_coord = true;
  mesh->SetVertices(vertex_attributes, vertices, triangles);

  if (!mesh->Export(path)) {
    EVOENGINE_ERROR("Tassel mesh export failed!");
  }
}

void MaizeTassel::ExportFlowGraph(YAML::Emitter& out) {
  out << YAML::Key << "Flows" << YAML::Value << YAML::BeginSeq;

  if (!growth_model.IsInitialized()) {
    out << YAML::EndSeq;
    return;
  }

  auto& graph = growth_model.graph;
  graph.SortLists();
  graph.CalculateFlows();

  const auto retained_flow_handles = CollectInternodeFlowHandles(graph);
  for (const auto flow_handle : graph.PeekSortedFlowList()) {
    if (retained_flow_handles.find(flow_handle) == retained_flow_handles.end()) {
      continue;
    }

    const auto& flow = graph.PeekFlow(flow_handle);
    const auto parent_flow_handle =
        FindParentInternodeFlowHandle(graph, flow_handle, retained_flow_handles);

    out << YAML::BeginMap;
    out << YAML::Key << "I" << YAML::Value << flow_handle;
    out << YAML::Key << "PI" << YAML::Value << parent_flow_handle;
    out << YAML::Key << "SP" << YAML::Value << flow.info.global_start_position;
    out << YAML::Key << "SD" << YAML::Value << flow.info.global_start_rotation * glm::vec3(0, 0, -1);
    out << YAML::Key << "ST" << YAML::Value << flow.info.start_thickness;
    out << YAML::Key << "EP" << YAML::Value << flow.info.global_end_position;
    out << YAML::Key << "ED" << YAML::Value << flow.info.global_end_rotation * glm::vec3(0, 0, -1);
    out << YAML::Key << "ET" << YAML::Value << flow.info.end_thickness;
    out << YAML::EndMap;
  }

  out << YAML::EndSeq;
}

void MaizeTassel::ExportFlowGraph(const std::filesystem::path& path) {
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

void MaizeTassel::ExportNodeGraph(YAML::Emitter& out) {
  out << YAML::Key << "Nodes" << YAML::Value << YAML::BeginSeq;

  if (!growth_model.IsInitialized()) {
    out << YAML::EndSeq;
    return;
  }

  auto& graph = growth_model.graph;
  graph.SortLists();
  graph.CalculateFlows();

  const auto retained_flow_handles = CollectInternodeFlowHandles(graph);
  for (const auto node_handle : graph.PeekSortedNodeList()) {
    const auto& node = graph.PeekNode(node_handle);
    if (!IsInternodeNode(node)) {
      continue;
    }

    const auto parent_node_handle = FindParentInternodeNodeHandle(graph, node_handle);
    auto flow_handle = node.GetFlowHandle();
    while (flow_handle >= 0 && retained_flow_handles.find(flow_handle) == retained_flow_handles.end()) {
      flow_handle = graph.PeekFlow(flow_handle).GetParentHandle();
    }

    out << YAML::BeginMap;
    out << YAML::Key << "I" << YAML::Value << node_handle;
    out << YAML::Key << "PI" << YAML::Value << parent_node_handle;
    out << YAML::Key << "FI" << YAML::Value << flow_handle;
    out << YAML::Key << "SP" << YAML::Value << node.info.global_position;
    out << YAML::Key << "EP" << YAML::Value << node.info.GetGlobalEndPosition();
    out << YAML::Key << "D" << YAML::Value << node.info.GetGlobalDirection();
    out << YAML::Key << "T" << YAML::Value << node.info.thickness;
    out << YAML::EndMap;
  }

  out << YAML::EndSeq;
}

void MaizeTassel::ExportNodeGraph(const std::filesystem::path& path) {
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

// ---------------------------------------------------------------------------

void MaizeTassel::OnDestroy() {
  ClearGeometryEntities();
#ifdef LSYSTEM_GPU_PIPELINE
  if (gpu_instance_id != 0u) {
    gpu::LSystemGPUEngine::Get().DestroyInstance(gpu_instance_id);
    gpu_instance_id = 0u;
  }
#endif
}

// ---------------------------------------------------------------------------

bool MaizeTassel::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;

  if (editor_layer->DragAndDropButton<MaizeTasselDescriptor>(descriptor_ref, "Descriptor"))
    changed = true;

  int seed_int = static_cast<int>(seed);
  if (ImGui::DragInt("Seed", &seed_int, 1, 0, 999999)) {
    seed = static_cast<unsigned int>(seed_int);
    changed = true;
  }

  if (ImGui::DragFloat("Target GDD", &target_gdd, 1.0f, 0.0f, 2000.0f))
    changed = true;

  if (ImGui::Button("Generate")) {
    GenerateGeometryEntities();
    changed = true;
  }
  ImGui::SameLine();
  if (ImGui::Button("Clear")) {
    ClearGeometryEntities();
    changed = true;
  }

  // Show growth model state.
  if (growth_model.IsInitialized()) {
    ImGui::Separator();
    ImGui::Text("GDD: %.1f", growth_model.accumulated_gdd);
    ImGui::Text("Topology: %s", growth_model.IsTopologyComplete() ? "Complete" : "Pending");
    const auto& sorted = growth_model.graph.PeekSortedNodeList();
    ImGui::Text("Nodes: %d", static_cast<int>(sorted.size()));
  }

  return changed;
}

// ---------------------------------------------------------------------------

void MaizeTassel::Serialize(YAML::Emitter& out) const {
  descriptor_ref.Save("descriptor_ref", out);
  out << YAML::Key << "seed" << YAML::Value << seed;
  out << YAML::Key << "target_gdd" << YAML::Value << target_gdd;
}

void MaizeTassel::Deserialize(const YAML::Node& in) {
  descriptor_ref.Load("descriptor_ref", in);
  if (in["seed"]) seed = in["seed"].as<unsigned int>();
  if (in["target_gdd"]) target_gdd = in["target_gdd"].as<float>();
}

void MaizeTassel::CollectAssetRef(std::vector<AssetRef>& list) {
  list.push_back(descriptor_ref);
}
