#include "ScotsPine.hpp"
#include "LSystemInspectionAdapters.hpp"
#include "LSystemSerializationAdapters.hpp"
#include "ScotsPineDescriptor.hpp"
#include "ScotsPineModules.hpp"

// Phase 1 biologically-emergent organ geometry primitives.
#include "CrossSectionProfile.hpp"
#include "ElasticaSolver.hpp"
#include "GeneralizedCylinderMesher.hpp"
#include "GrowthField.hpp"
#include "MaterialProfile.hpp"
#include "OrganCenterline.hpp"
#include "OrganMeshPly.hpp"

#include <AssetManager.hpp>
#include <EditorLayer.hpp>
#include <Material.hpp>
#include <Mesh.hpp>
#include <MeshRenderer.hpp>
#include <Particles.hpp>
#include <Scene.hpp>
#include <Times.hpp>
#include <Transform.hpp>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <fstream>
#include <glm/gtx/quaternion.hpp>
#include <limits>
#include <mutex>
#include <unordered_set>

using namespace l_system_package;
using namespace evo_engine;

namespace {

ScotsPine::ColorMode g_scots_pine_color_mode = ScotsPine::ColorMode::Shaded;
std::atomic<bool> g_force_cpu_particles_path{false};

// Phase 1: when true, replace the per-cluster octahedron marker (legacy,
// kept under [deprecated] block below) with explicit swept generalized-
// cylinder needle geometry. Defaults true; set to false to compare against
// the legacy visual baseline.
std::atomic<bool> g_use_generalized_cylinder_needles{true};

// Visualization-only knobs (see ScotsPine.hpp for contract notes).
// All three default to neutral values that reproduce existing behaviour
// byte-identically when no caller opts in.
std::atomic<float> g_internode_visual_radius_multiplier{1.0f};
std::atomic<bool> g_render_needles_enabled{true};
std::atomic<bool> g_generate_needle_topology_enabled{true};
// Leader debug colour stored as four floats; protected by a coarse mutex
// because atomic<glm::vec4> is not portable. Reads happen once per rebuild
// in the same thread that calls into RebuildGeometry; contention is nil.
std::mutex g_leader_internode_debug_color_mutex;
glm::vec4 g_leader_internode_debug_color{0.0f, 0.0f, 0.0f, 0.0f};

bool IsFiniteQuat(const glm::quat& q) {
  return std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w);
}

bool IsFiniteVec3(const glm::vec3& v) {
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

bool IsFiniteVec2(const glm::vec2& v) {
  return std::isfinite(v.x) && std::isfinite(v.y);
}

bool IsFiniteVec4(const glm::vec4& v) {
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z) && std::isfinite(v.w);
}

glm::vec4 SanitizeFiniteColor(const glm::vec4& value, const glm::vec4& fallback) {
  if (!IsFiniteVec4(value))
    return fallback;
  return glm::clamp(value, glm::vec4(0.0f), glm::vec4(1.0f));
}

bool IsFiniteMat4(const glm::mat4& m) {
  for (int c = 0; c < 4; c++) {
    for (int r = 0; r < 4; r++) {
      if (!std::isfinite(m[c][r]))
        return false;
    }
  }
  return true;
}

bool IsNeedleAggregateMeshValid(const std::vector<Vertex>& vertices, const std::vector<glm::uvec3>& triangles) {
  if (vertices.empty() || triangles.empty())
    return true;

  for (const auto& v : vertices) {
    if (!IsFiniteVec3(v.position)) {
      return false;
    }
  }

  const uint32_t vertex_count = static_cast<uint32_t>(vertices.size());
  for (const auto& tri : triangles) {
    if (tri.x >= vertex_count || tri.y >= vertex_count || tri.z >= vertex_count) {
      return false;
    }
  }

  return true;
}

void SanitizeNeedleAggregateMeshVertices(std::vector<Vertex>& vertices) {
  for (auto& v : vertices) {
    if (!IsFiniteVec3(v.normal) || glm::length(v.normal) <= 1.0e-8f) {
      v.normal = glm::vec3(0.0f, 1.0f, 0.0f);
    } else {
      v.normal = glm::normalize(v.normal);
    }

    if (!IsFiniteVec3(v.tangent) || glm::length(v.tangent) <= 1.0e-8f) {
      // Keep tangent orthogonal to the sanitized normal to avoid NaN TBN in shading.
      glm::vec3 ref(1.0f, 0.0f, 0.0f);
      if (std::abs(glm::dot(ref, v.normal)) > 0.95f) {
        ref = glm::vec3(0.0f, 0.0f, 1.0f);
      }
      v.tangent = glm::normalize(ref - v.normal * glm::dot(ref, v.normal));
    } else {
      v.tangent = glm::normalize(v.tangent);
    }

    if (!IsFiniteVec2(v.tex_coord)) {
      v.tex_coord = glm::vec2(0.0f);
    }

    if (!IsFiniteVec4(v.color)) {
      v.color = glm::vec4(1.0f);
    } else {
      v.color = glm::clamp(v.color, glm::vec4(0.0f), glm::vec4(1.0f));
    }
  }
}

bool HasNeedleSenescenceOrAbscission(const PineGraph& graph, const std::vector<LNodeHandle>& sorted_nodes) {
  for (const auto handle : sorted_nodes) {
    const auto& node = graph.PeekNode(handle);
    if (!node.data.template Is<PineNeedleCluster>())
      continue;
    const auto& cluster = node.data.template Get<PineNeedleCluster>();
    if (!cluster.alive || cluster.senescence_phase > 0.0f) {
      return true;
    }
  }
  return false;
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
    case 0:
      rgb = glm::vec3(v, t, p);
      break;
    case 1:
      rgb = glm::vec3(q, v, p);
      break;
    case 2:
      rgb = glm::vec3(p, v, t);
      break;
    case 3:
      rgb = glm::vec3(p, q, v);
      break;
    case 4:
      rgb = glm::vec3(t, p, v);
      break;
    default:
      rgb = glm::vec3(v, p, q);
      break;
  }
  return glm::vec4(rgb, 1.0f);
}

uint32_t MixBits(const uint32_t value) {
  uint32_t x = value;
  x ^= x >> 16;
  x *= 0x7feb352du;
  x ^= x >> 15;
  x *= 0x846ca68bu;
  x ^= x >> 16;
  return x;
}

float HashToUnitOpen01(const uint32_t hash) {
  constexpr float kDenominator = 16777217.0f;
  const float u = static_cast<float>(hash & 0x00ffffffu) / kDenominator;
  return std::clamp(u + (1.0f / kDenominator), 1.0e-6f, 1.0f - 1.0e-6f);
}

float DeterministicNormalFromNodeRandom(const float node_random, const uint32_t salt) {
  constexpr float kTwoPi = 6.28318530717958647692f;
  const float clamped = std::clamp(node_random, 0.0f, 1.0f);
  const uint32_t base = static_cast<uint32_t>(std::round(clamped * 16777215.0f));
  const uint32_t h1 = MixBits(base ^ salt ^ 0x9e3779b9u);
  const uint32_t h2 = MixBits(base ^ salt ^ 0x85ebca6bu);
  const float u1 = HashToUnitOpen01(h1);
  const float u2 = HashToUnitOpen01(h2);
  const float radius = std::sqrt(-2.0f * std::log(u1));
  return radius * std::cos(kTwoPi * u2);
}

float EvaluatePositionalMultiplier(const evo_engine::PlottedDistribution<float>& distribution, const float s_norm,
                                   const float node_random, const uint32_t salt) {
  const float x = std::clamp(s_norm, 0.0f, 1.0f);
  const float mean_value = distribution.mean.GetValue(x);
  const float deviation_value = std::max(0.0f, distribution.deviation.GetValue(x));
  if (!(deviation_value > 0.0f)) {
    return std::max(0.0f, mean_value);
  }
  const float z = DeterministicNormalFromNodeRandom(node_random, salt);
  return std::max(0.0f, mean_value + deviation_value * z);
}

constexpr float kNeedleCrossSectionTemporalWindowYears = 2.0f;

float EvaluateTemporalCrossSectionMultiplier(const evo_engine::PlottedDistribution<float>& distribution,
                                             const float cluster_age_years, const float node_random,
                                             const uint32_t salt) {
  // Curve x-domain is [0,1]; map 0..2 years of cluster age onto that axis.
  const float age_norm =
      std::clamp(cluster_age_years / std::max(1.0e-6f, kNeedleCrossSectionTemporalWindowYears), 0.0f, 1.0f);
  return EvaluatePositionalMultiplier(distribution, age_norm, node_random, salt);
}

bool IsInternodeNode(const PineNode& node) {
  return node.data.template Is<PineInternode>();
}

LNodeHandle FindParentInternodeNodeHandle(const PineGraph& graph, const LNodeHandle node_handle) {
  auto parent_handle = graph.PeekNode(node_handle).GetParentHandle();
  while (parent_handle >= 0) {
    const auto& parent = graph.PeekNode(parent_handle);
    if (IsInternodeNode(parent))
      return parent_handle;
    parent_handle = parent.GetParentHandle();
  }
  return -1;
}

std::unordered_set<LFlowHandle> CollectInternodeFlowHandles(const PineGraph& graph) {
  std::unordered_set<LFlowHandle> retained;
  for (const auto flow_handle : graph.PeekSortedFlowList()) {
    const auto& flow = graph.PeekFlow(flow_handle);
    const auto& node_handles = flow.PeekNodeHandles();
    if (node_handles.empty())
      continue;
    bool has_internode = false;
    for (const auto h : node_handles) {
      if (IsInternodeNode(graph.PeekNode(h))) {
        has_internode = true;
        break;
      }
    }
    if (has_internode)
      retained.emplace(flow_handle);
  }
  return retained;
}

LFlowHandle FindParentInternodeFlowHandle(const PineGraph& graph, const LFlowHandle flow_handle,
                                          const std::unordered_set<LFlowHandle>& retained) {
  auto parent_flow_handle = graph.PeekFlow(flow_handle).GetParentHandle();
  while (parent_flow_handle >= 0) {
    if (retained.find(parent_flow_handle) != retained.end())
      return parent_flow_handle;
    parent_flow_handle = graph.PeekFlow(parent_flow_handle).GetParentHandle();
  }
  return -1;
}

void AppendParticlesToMesh(const std::shared_ptr<Scene>& scene, const Entity& entity, std::vector<Vertex>& out_vertices,
                           std::vector<glm::uvec3>& out_triangles) {
  if (!scene->IsEntityValid(entity) || !scene->HasPrivateComponent<Particles>(entity))
    return;
  const auto particles = scene->GetOrSetPrivateComponent<Particles>(entity).lock();
  if (!particles)
    return;
  const auto mesh = particles->mesh.Get<Mesh>();
  const auto particle_info_list = particles->particle_info_list.Get<ParticleInfoList>();
  if (!mesh || !particle_info_list)
    return;
  const auto& source_vertices = mesh->UnsafeGetVertices();
  const auto& source_triangles = mesh->UnsafeGetTriangles();
  const auto& instances = particle_info_list->PeekParticleInfoList();
  if (source_vertices.empty() || source_triangles.empty() || instances.empty())
    return;

  const auto entity_global_transform = scene->GetDataComponent<GlobalTransform>(entity);
  for (const auto& instance : instances) {
    const glm::mat4 world_transform = entity_global_transform.value * instance.instance_matrix.value;
    if (!IsFiniteMat4(world_transform))
      continue;
    const glm::mat3 world_3x3(world_transform);
    glm::mat3 normal_transform(1.0f);
    const float det = glm::determinant(world_3x3);
    if (std::isfinite(det) && std::abs(det) > 1e-8f) {
      normal_transform = glm::transpose(glm::inverse(world_3x3));
    }
    const auto vertex_offset = static_cast<uint32_t>(out_vertices.size());
    out_vertices.reserve(out_vertices.size() + source_vertices.size());
    out_triangles.reserve(out_triangles.size() + source_triangles.size());
    for (const auto& sv : source_vertices) {
      Vertex v = sv;
      v.position = glm::vec3(world_transform * glm::vec4(sv.position, 1.0f));
      const glm::vec3 tn = normal_transform * sv.normal;
      if (IsFiniteVec3(tn) && glm::length(tn) > 1e-8f)
        v.normal = glm::normalize(tn);
      const glm::vec3 tt = normal_transform * sv.tangent;
      if (IsFiniteVec3(tt) && glm::length(tt) > 1e-8f)
        v.tangent = glm::normalize(tt);
      v.color = instance.instance_color;
      out_vertices.emplace_back(v);
    }
    for (const auto& st : source_triangles) {
      out_triangles.emplace_back(vertex_offset + st.x, vertex_offset + st.y, vertex_offset + st.z);
    }
  }
}

void AppendMeshRendererToMesh(const std::shared_ptr<Scene>& scene, const Entity& entity,
                              std::vector<Vertex>& out_vertices, std::vector<glm::uvec3>& out_triangles) {
  if (!scene->IsEntityValid(entity) || !scene->HasPrivateComponent<MeshRenderer>(entity))
    return;
  const auto mesh_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(entity).lock();
  if (!mesh_renderer)
    return;
  const auto mesh = mesh_renderer->mesh.Get<Mesh>();
  if (!mesh)
    return;

  const auto& source_vertices = mesh->UnsafeGetVertices();
  const auto& source_triangles = mesh->UnsafeGetTriangles();
  if (source_vertices.empty() || source_triangles.empty())
    return;

  const auto entity_global_transform = scene->GetDataComponent<GlobalTransform>(entity);
  const glm::mat4 world_transform = entity_global_transform.value;
  if (!IsFiniteMat4(world_transform))
    return;

  const glm::mat3 world_3x3(world_transform);
  glm::mat3 normal_transform(1.0f);
  const float det = glm::determinant(world_3x3);
  if (std::isfinite(det) && std::abs(det) > 1e-8f) {
    normal_transform = glm::transpose(glm::inverse(world_3x3));
  }

  const auto vertex_offset = static_cast<uint32_t>(out_vertices.size());
  out_vertices.reserve(out_vertices.size() + source_vertices.size());
  out_triangles.reserve(out_triangles.size() + source_triangles.size());
  for (const auto& sv : source_vertices) {
    Vertex v = sv;
    v.position = glm::vec3(world_transform * glm::vec4(sv.position, 1.0f));
    const glm::vec3 tn = normal_transform * sv.normal;
    if (IsFiniteVec3(tn) && glm::length(tn) > 1e-8f)
      v.normal = glm::normalize(tn);
    const glm::vec3 tt = normal_transform * sv.tangent;
    if (IsFiniteVec3(tt) && glm::length(tt) > 1e-8f)
      v.tangent = glm::normalize(tt);
    out_vertices.emplace_back(v);
  }
  for (const auto& st : source_triangles) {
    out_triangles.emplace_back(vertex_offset + st.x, vertex_offset + st.y, vertex_offset + st.z);
  }
}

// Unit cylinder: radius=1, height=1, along +Y, base at y=0.
void GenerateUnitCylinderMesh(std::vector<Vertex>& vertices, std::vector<unsigned int>& indices,
                              const glm::vec4& bark_color, int segments = 6) {
  vertices.clear();
  indices.clear();
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
      v.color = bark_color;
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

// Unit octahedron centered at origin, radius=1 along each axis.
void GenerateUnitOctahedronMesh(std::vector<Vertex>& vertices, std::vector<unsigned int>& indices,
                                const glm::vec4& needle_color) {
  vertices.clear();
  indices.clear();
  const std::array<glm::vec3, 6> positions = {glm::vec3(1, 0, 0),  glm::vec3(-1, 0, 0), glm::vec3(0, 1, 0),
                                              glm::vec3(0, -1, 0), glm::vec3(0, 0, 1),  glm::vec3(0, 0, -1)};
  for (const auto& p : positions) {
    Vertex v;
    v.position = p;
    v.normal = glm::normalize(p);
    v.color = needle_color;
    v.tex_coord = glm::vec2(0.5f, 0.5f);
    vertices.push_back(v);
  }
  // 8 triangular faces.
  const std::array<glm::uvec3, 8> tris = {glm::uvec3(0, 2, 4), glm::uvec3(2, 1, 4), glm::uvec3(1, 3, 4),
                                          glm::uvec3(3, 0, 4), glm::uvec3(2, 0, 5), glm::uvec3(1, 2, 5),
                                          glm::uvec3(3, 1, 5), glm::uvec3(0, 3, 5)};
  for (const auto& t : tris) {
    indices.push_back(t.x);
    indices.push_back(t.y);
    indices.push_back(t.z);
  }
}

// ---------------------------------------------------------------------------
// Phase 1: Build a single aggregate mesh containing every needle in the
// tree, swept as a generalized cylinder along a (currently straight)
// per-needle OrganCenterline using an EllipticProfile with independent
// width/thickness curves. Anchor = parent internode's distal tangent.
//
// This is the visual seam that Phases 2-5 will animate by mutating each
// needle's centerline control points (differential growth + elastica).
// All other geometry stays where it is.
// ---------------------------------------------------------------------------
struct NeedleAnchor {
  glm::vec3 base_position;       ///< World-space attachment point.
  glm::quat orientation;         ///< Rotates +Z (needle local forward) to outward direction.
  glm::vec3 base_adaxial_world;  ///< World-space adaxial reference at the base.
};

inline NeedleAnchor ComputeFascicleNeedleAnchor(const PineNode& /*cluster_node*/, const PineNode& parent_internode_node,
                                                float s_along_parent_norm, float roll_offset_deg,
                                                float branching_angle_deg, int needle_index_in_cluster,
                                                int needle_count_in_cluster, float cluster_random) {
  // Parent internode tangent and rolled radial basis in world space.
  // Using the internode's world rotation preserves phyllotactic roll.
  glm::vec3 parent_dir = parent_internode_node.info.GetGlobalDirection();
  if (!IsFiniteVec3(parent_dir) || glm::length(parent_dir) <= 1e-8f) {
    parent_dir = glm::vec3(0.0f, 0.0f, -1.0f);
  } else {
    parent_dir = glm::normalize(parent_dir);
  }

  glm::vec3 perp = parent_internode_node.info.global_rotation * glm::vec3(1.0f, 0.0f, 0.0f);
  if (!IsFiniteVec3(perp) || glm::length(perp) <= 1e-8f) {
    // Fallback keeps anchors valid even if an upstream rotation is degenerate.
    glm::vec3 ref(0.0f, 1.0f, 0.0f);
    if (std::abs(glm::dot(ref, parent_dir)) > 0.95f)
      ref = glm::vec3(1.0f, 0.0f, 0.0f);
    perp = ref - parent_dir * glm::dot(ref, parent_dir);
  }
  const float plen = glm::length(perp);
  perp = (plen > 1e-8f) ? (perp / plen) : glm::vec3(1.0f, 0.0f, 0.0f);

  // Apply per-cluster azimuthal phyllotactic roll around the parent axis so
  // sibling clusters on the same shoot fan out by ~137.5 degrees.
  const glm::quat roll_q = glm::angleAxis(glm::radians(roll_offset_deg), parent_dir);
  perp = glm::normalize(roll_q * perp);

  // Upward fan around the parent axis. For the common 2-needle fascicle, keep
  // both needles in a narrow V instead of placing them 180 degrees apart
  // (which frequently sends one needle downward and reads as "spaghetti").
  const float kFanHalfAngleRad = glm::radians(12.0f);
  const float kFanJitterRad = glm::radians(4.0f);
  const float fan_jitter = (cluster_random - 0.5f) * 2.0f * kFanJitterRad;
  float fan_t = 0.0f;
  if (needle_count_in_cluster > 1) {
    fan_t = static_cast<float>(needle_index_in_cluster) / static_cast<float>(needle_count_in_cluster - 1);
    fan_t = fan_t * 2.0f - 1.0f;
  }
  const float angle = fan_t * kFanHalfAngleRad + fan_jitter;
  const glm::quat about_axis = glm::angleAxis(angle, parent_dir);
  const glm::vec3 radial = about_axis * perp;

  // Branching angle is measured from the apical axis (parent_dir). 0 deg
  // means fully apical; increasing angle opens the fascicle toward radial.
  const float branching_angle_rad = glm::radians(std::clamp(branching_angle_deg, 0.0f, 89.5f));
  const glm::vec3 needle_dir =
      glm::normalize(std::cos(branching_angle_rad) * parent_dir + std::sin(branching_angle_rad) * radial);

  // Build a stable local frame instead of a shortest-arc quaternion so the
  // local x-z bending plane is locked to the stem-facing radial plane.
  // +Z = needle forward, +X = adaxial->abaxial direction (outward from stem),
  // +Y = completes a right-handed basis.
  const glm::vec3 z_axis = needle_dir;
  glm::vec3 x_axis = radial - z_axis * glm::dot(radial, z_axis);
  if (!IsFiniteVec3(x_axis) || glm::length(x_axis) <= 1e-8f) {
    glm::vec3 ref(0.0f, 1.0f, 0.0f);
    if (std::abs(glm::dot(ref, z_axis)) > 0.95f)
      ref = glm::vec3(1.0f, 0.0f, 0.0f);
    x_axis = ref - z_axis * glm::dot(ref, z_axis);
  }
  x_axis = glm::normalize(x_axis);
  glm::vec3 y_axis = glm::cross(z_axis, x_axis);
  if (!IsFiniteVec3(y_axis) || glm::length(y_axis) <= 1e-8f) {
    y_axis = glm::vec3(0.0f, 1.0f, 0.0f);
  } else {
    y_axis = glm::normalize(y_axis);
  }
  x_axis = glm::normalize(glm::cross(y_axis, z_axis));
  const glm::mat3 basis_world_from_local(x_axis, y_axis, z_axis);
  const glm::quat orientation = glm::normalize(glm::quat_cast(basis_world_from_local));

  // Use -X as the adaxial reference direction for the ellipsoid profile.
  const glm::vec3 adaxial_world = glm::normalize(orientation * glm::vec3(-1, 0, 0));

  // Anchor at fraction `s_along_parent_norm` along the parent shoot's length.
  // Parent's `global_position` is the proximal end; segment extends along
  // parent_dir for `parent.info.length` units.
  const float s_clamped = std::clamp(s_along_parent_norm, 0.0f, 1.0f);
  const float parent_length = std::max(0.0f, parent_internode_node.info.length);
  const glm::vec3 anchor_pos = parent_internode_node.info.global_position + parent_dir * (s_clamped * parent_length);

  NeedleAnchor anchor;
  anchor.base_position = anchor_pos;
  anchor.orientation = orientation;
  anchor.base_adaxial_world = adaxial_world;
  return anchor;
}

/// Build a single aggregate triangle mesh containing every alive needle in
/// the tree. Vertices are emitted in world space (anchor transforms baked
/// in) so the consuming `Particles` instance uses an identity matrix.
inline void BuildPineNeedleAggregateMesh(
    const PineGraph& graph, const std::vector<LNodeHandle>& sorted_nodes, const ScotsPine::ColorMode color_mode,
    const glm::vec4& needle_young_color, const glm::vec4& needle_old_color, const float needle_axial_age_span,
    const float needle_axial_age_exponent, const PlottedDistribution<float>& needle_cross_section_width_profile,
    const PlottedDistribution<float>& needle_cross_section_thickness_profile,
    const PlottedDistribution<float>& needle_cross_section_temporal_maturity_curve,
    const int needle_fascicular_start_year, const float needle_lignification_factor_year1,
    const float needle_lignification_factor_year2plus, const float needle_stomatal_strip_density_year1,
    const float needle_stomatal_strip_density_year2plus, const float needle_basal_taper_ratio_year1,
    const float needle_basal_taper_ratio_year2plus, const float needle_fascicle_sheath_budget_years,
    const float needle_specularity_plasticity_year1, const float needle_specularity_plasticity_year2plus,
    std::vector<Vertex>& out_vertices, std::vector<glm::uvec3>& out_triangles, const int station_count,
    const int perimeter_count, std::vector<ScotsPine::NeedleSkeletonLine>* out_needle_skeleton_lines) {
  out_vertices.clear();
  out_triangles.clear();
  if (out_needle_skeleton_lines) {
    out_needle_skeleton_lines->clear();
  }

  // Geometry parameters (Phase 1 defaults). station/perimeter are caller-
  // supplied so auto-grow can use a cheaper tessellation while growth is
  // advancing, then recover high quality when growth pauses.
  const int kStations = std::max(4, station_count);
  const int kPerimeter = std::max(3, perimeter_count);
  constexpr float kNeedleDefaultWidthRadiusM = 0.0007f;       // ~0.7 mm half-width.
  constexpr float kNeedleDefaultThicknessRadiusM = 0.00045f;  // ~0.45 mm half-thickness.

  EllipticProfile ellipsoid_profile(/*aspect_ratio=*/1.0f, /*twist_radians=*/0.0f);

  for (const auto handle : sorted_nodes) {
    const auto& node = graph.PeekNode(handle);
    if (!node.data.template Is<PineNeedleCluster>())
      continue;
    const auto& cluster = node.data.template Get<PineNeedleCluster>();
    if (!cluster.alive)
      continue;

    const LNodeHandle parent_handle = FindParentInternodeNodeHandle(graph, handle);
    if (parent_handle < 0)
      continue;
    const auto& parent_node = graph.PeekNode(parent_handle);

    const float sen = std::clamp(cluster.senescence_phase, 0.0f, 1.0f);
    const float cluster_age_years =
        std::max(0.0f, graph.data.clock.NowYears() - cluster.continuous_growth.t_init_years);
    const float cluster_lifespan_years = std::max(0.25f, static_cast<float>(cluster.lifespan_years));
    const float cluster_age_norm = std::clamp(cluster_age_years / cluster_lifespan_years, 0.0f, 1.0f);
    const float cluster_branching_angle_deg = std::clamp(cluster.branching_angle_deg, 0.0f, 89.5f);
    const float clamped_axial_exponent = std::max(0.1f, needle_axial_age_exponent);
    const float length_mult = glm::mix(1.0f, 0.82f, sen);
    const bool year2plus_cohort = cluster.initiation_year_index >= std::max(0, needle_fascicular_start_year);
    const float cohort_lignification_factor = year2plus_cohort
                                                  ? std::clamp(needle_lignification_factor_year2plus, 0.0f, 2.0f)
                                                  : std::clamp(needle_lignification_factor_year1, 0.0f, 2.0f);
    const float cohort_stomatal_strip_density = year2plus_cohort
                                                    ? std::clamp(needle_stomatal_strip_density_year2plus, 0.0f, 1.0f)
                                                    : std::clamp(needle_stomatal_strip_density_year1, 0.0f, 1.0f);
    const float cohort_basal_taper_ratio = year2plus_cohort ? std::clamp(needle_basal_taper_ratio_year2plus, 0.6f, 1.2f)
                                                            : std::clamp(needle_basal_taper_ratio_year1, 0.6f, 1.2f);
    const float cohort_specularity_plasticity = year2plus_cohort
                                                    ? std::clamp(needle_specularity_plasticity_year2plus, 0.0f, 1.0f)
                                                    : std::clamp(needle_specularity_plasticity_year1, 0.0f, 1.0f);
    const float sheath_budget_years = std::max(0.0f, needle_fascicle_sheath_budget_years);

    // Phase 6 seedling realism pass: drive rendered needle thickness from
    // material profile radii when available, with Phase 1 taper as a fallback.
    const float t_now_years = graph.data.clock.NowYears();
    const float maturation_multiplier =
        (cluster.continuous_growth.maturation_years > 0.0f) ? cluster.continuous_growth.Multiplier(t_now_years) : 1.0f;

    // Phase 4: world-frame gravity vector (-Y world-up convention).
    const float gravity_mag = graph.data.gravity_m_s2;
    const glm::vec3 gravity_world(0.0f, -gravity_mag, 0.0f);

    for (int n = 0; n < std::max(1, cluster.count); ++n) {
      const PineNeedleInstanceProfile* needle_profile = (static_cast<size_t>(n) < cluster.per_needle_profiles.size())
                                                            ? &cluster.per_needle_profiles[static_cast<size_t>(n)]
                                                            : nullptr;
      const BilateralGrowthField1D& needle_growth_field =
          needle_profile ? needle_profile->growth_field : cluster.growth_field;
      const MaterialProfile1D& needle_material_profile =
          needle_profile ? needle_profile->material_profile : cluster.material_profile;
      MaterialProfile1D matured_material_profile = needle_material_profile;
      const float needle_length_scale = needle_profile ? std::clamp(needle_profile->length_scale, 0.05f, 3.0f) : 1.0f;
      const float needle_radius_scale = needle_profile ? std::clamp(needle_profile->radius_scale, 0.05f, 3.0f) : 1.0f;
      const float needle_wave_amplitude_deg = needle_profile
                                                  ? std::clamp(needle_profile->sinusoidal_amplitude_deg, 0.0f, 45.0f)
                                                  : std::clamp(cluster.sinusoidal_amplitude_deg, 0.0f, 45.0f);
      const float needle_wave_frequency_cycles =
          needle_profile ? std::clamp(needle_profile->sinusoidal_frequency_cycles, 0.0f, 12.0f)
                         : std::clamp(cluster.sinusoidal_frequency_cycles, 0.0f, 12.0f);
      const float needle_wave_phase_rad =
          needle_profile ? needle_profile->sinusoidal_phase_rad : cluster.sinusoidal_phase_rad;
      // Use chronological age so fascicle opening continues through dormant season.
      const float needle_relax_years = needle_profile ? std::max(0.0f, needle_profile->branching_relax_years)
                                                      : std::max(0.0f, cluster.branching_relax_years);
      const float needle_relax_progress =
          (needle_relax_years <= 1e-5f) ? 1.0f : std::clamp(cluster_age_years / needle_relax_years, 0.0f, 1.0f);
      const float active_branching_angle_deg = cluster_branching_angle_deg * needle_relax_progress;

      const bool has_profile_radii =
          std::isfinite(matured_material_profile.base_radius_m) &&
          std::isfinite(matured_material_profile.tip_radius_m) &&
          (matured_material_profile.base_radius_m > 0.0f || matured_material_profile.tip_radius_m > 0.0f);
      const float fallback_width_radius_m =
          has_profile_radii ? std::max(matured_material_profile.base_radius_m, 0.0f) : kNeedleDefaultWidthRadiusM;
      const float fallback_thickness_radius_m =
          has_profile_radii ? std::max(matured_material_profile.tip_radius_m, 0.0f) : kNeedleDefaultThicknessRadiusM;
      const float raw_cluster_width_radius_m = (cluster.cross_section_width_radius_m > 0.0f)
                                                   ? cluster.cross_section_width_radius_m
                                                   : fallback_width_radius_m;
      const float raw_cluster_thickness_radius_m = (cluster.cross_section_thickness_radius_m > 0.0f)
                                                       ? cluster.cross_section_thickness_radius_m
                                                       : fallback_thickness_radius_m;
      const float clamped_cluster_width_radius_m = std::max(raw_cluster_width_radius_m, 0.00002f);
      const float clamped_cluster_thickness_radius_m = std::max(raw_cluster_thickness_radius_m, 0.00002f);
      std::vector<float> width_radius_table;
      std::vector<float> thickness_radius_table;
      width_radius_table.reserve(static_cast<size_t>(kStations));
      thickness_radius_table.reserve(static_cast<size_t>(kStations));
      const float radius_vigor_scale = std::clamp(cluster.render_radius_scale * needle_radius_scale, 0.10f, 4.0f);
      const uint32_t node_hash = static_cast<uint32_t>(node.GetIndex());
      const uint32_t needle_hash = static_cast<uint32_t>(n);
      const uint32_t width_seed = node_hash ^ (needle_hash * 0x9e3779b9u) ^ 0x2d9c8f13u;
      const uint32_t thickness_seed = node_hash ^ (needle_hash * 0x85ebca6bu) ^ 0xa5b35705u;
      const uint32_t temporal_seed = node_hash ^ (needle_hash * 0xc2b2ae35u) ^ 0x4f1bbcdcu;
      const float temporal_cross_section_multiplier = EvaluateTemporalCrossSectionMultiplier(
          needle_cross_section_temporal_maturity_curve, cluster_age_years, cluster.node_random, temporal_seed);
      for (int i = 0; i < kStations; ++i) {
        const float s_norm = static_cast<float>(i) / static_cast<float>(kStations - 1);
        const float width_profile_multiplier = EvaluatePositionalMultiplier(
            needle_cross_section_width_profile, s_norm, cluster.node_random, width_seed ^ static_cast<uint32_t>(i));
        const float thickness_profile_multiplier =
            EvaluatePositionalMultiplier(needle_cross_section_thickness_profile, s_norm, cluster.node_random,
                                         thickness_seed ^ static_cast<uint32_t>(i));
        const float raw_width_radius =
            clamped_cluster_width_radius_m * width_profile_multiplier * temporal_cross_section_multiplier;
        const float raw_thickness_radius =
            clamped_cluster_thickness_radius_m * thickness_profile_multiplier * temporal_cross_section_multiplier;
        const float scaled_width_radius = raw_width_radius * radius_vigor_scale;
        const float scaled_thickness_radius = raw_thickness_radius * radius_vigor_scale;
        const float safe_width_radius = std::max(scaled_width_radius, 0.00002f);
        const float safe_thickness_radius = std::max(scaled_thickness_radius, 0.00002f);
        const float basal_taper_multiplier = glm::mix(cohort_basal_taper_ratio, 1.0f, s_norm);
        const float tapered_width_radius = std::max(safe_width_radius * basal_taper_multiplier, 0.00002f);
        const float tapered_thickness_radius = std::max(safe_thickness_radius * basal_taper_multiplier, 0.00002f);
        width_radius_table.push_back(tapered_width_radius);
        thickness_radius_table.push_back(tapered_thickness_radius);
      }
      const float base_radius_m = width_radius_table.empty() ? kNeedleDefaultWidthRadiusM : width_radius_table.front();
      const float secondary_base_radius_m = thickness_radius_table.empty()
                                                ? kNeedleDefaultThicknessRadiusM
                                                : std::max(thickness_radius_table.front(), 0.00002f);

      // Phase 3: bent centerline driven by a per-needle bilateral growth
      // field, ramped by the cluster's continuous-growth multiplier.
      const float length = std::max(0.001f, cluster.length * length_mult * needle_length_scale);
      OrganCenterline intrinsic_centerline =
          BuildBentNeedleCenterline(length, /*segments=*/kStations - 1, needle_growth_field, maturation_multiplier,
                                    needle_wave_amplitude_deg, needle_wave_frequency_cycles, needle_wave_phase_rad);

      const bool mechanics_active = matured_material_profile.IsActive() && gravity_mag > 0.0f;
      const NeedleAnchor anchor =
          ComputeFascicleNeedleAnchor(node, parent_node, cluster.s_along_parent_norm, cluster.roll_offset_deg,
                                      active_branching_angle_deg, n, cluster.count, cluster.node_random);

      // Phase 4: per-needle elastica solve. The intrinsic centerline gives
      // kappa_intrinsic(s) and the deflection plane (local x-z); gravity is
      // projected from world into needle-local coordinates and decomposed
      // onto that plane. When the material profile is inert, this whole
      // block is skipped and we fall back to the Phase 3 intrinsic shape.
      OrganCenterline centerline = intrinsic_centerline;
      if (mechanics_active) {
        // Project gravity into the needle's local frame.
        const glm::quat q_inv = glm::conjugate(anchor.orientation);
        const glm::vec3 g_local = q_inv * gravity_world;
        // Use only the in-plane (x, z) components - the cross-plane Y
        // component would induce twist; Phase 4 intentionally restricts
        // bending to the same plane the intrinsic curvature lives in.
        const glm::vec2 g_xz(g_local.x, g_local.z);

        // Sample intrinsic curvature from the un-deflected centerline by
        // numerical differentiation of its tangent angle. (Closed-form
        // would require exposing the field math here; this is cheap.)
        std::vector<float> kappa_table(kStations, 0.0f);
        {
          const float ds = length / static_cast<float>(kStations - 1);
          float prev_theta = 0.0f;
          for (int i = 0; i < kStations; ++i) {
            const float s_i = static_cast<float>(i) * ds;
            const auto sample = intrinsic_centerline.Sample(s_i);
            const float theta_i = std::atan2(sample.tangent.x, sample.tangent.z);
            if (i == 0) {
              kappa_table[0] = 0.0f;
            } else {
              kappa_table[i] = (theta_i - prev_theta) / ds;
            }
            prev_theta = theta_i;
          }
        }

        PlanarElasticaInput esi;
        esi.length_m = length;
        esi.station_count = kStations;
        esi.intrinsic_curvature_per_m = [&kappa_table](float s_norm) {
          const int N = static_cast<int>(kappa_table.size());
          const float idx_f = std::clamp(s_norm, 0.0f, 1.0f) * static_cast<float>(N - 1);
          const int i0 = std::clamp(static_cast<int>(std::floor(idx_f)), 0, N - 1);
          const int i1 = std::min(i0 + 1, N - 1);
          const float u = idx_f - static_cast<float>(i0);
          return kappa_table[i0] * (1.0f - u) + kappa_table[i1] * u;
        };
        esi.bending_stiffness_Pa_m4 = [&matured_material_profile, t_now_years](float s_norm) {
          return matured_material_profile.BendingStiffness_Pa_m4(s_norm, t_now_years);
        };
        esi.mass_per_length_kg_m = [&matured_material_profile](float s_norm) {
          return matured_material_profile.MassPerLength_kg_m(s_norm);
        };
        esi.gravity_acceleration_xz_m_s2 = g_xz;
        esi.relaxation = 0.6f;
        esi.tolerance_rad = 1e-5f;
        esi.max_iterations = 24;
        const PlanarElasticaOutput eso = SolvePlanarElastica(esi);

        // Convert solver positions back into a centerline (planar, x-z).
        OrganCenterline deflected;
        auto& cps = deflected.MutableControlPoints();
        cps.resize(static_cast<size_t>(kStations));
        for (int i = 0; i < kStations; ++i) {
          cps[static_cast<size_t>(i)] = glm::vec3(eso.positions_xz[i].x, 0.0f, eso.positions_xz[i].y);
        }
        deflected.Invalidate();
        centerline = std::move(deflected);
      }

      if (out_needle_skeleton_lines) {
        ScotsPine::NeedleSkeletonLine line;
        line.cluster_node_handle = static_cast<int>(handle);
        line.parent_node_handle = static_cast<int>(parent_handle);
        line.needle_index = n;

        const float centerline_length = std::max(0.0f, centerline.TotalLength());
        const int station_n = std::max(2, kStations);
        line.points_world.reserve(static_cast<size_t>(station_n));

        for (int si = 0; si < station_n; ++si) {
          const float s_norm = static_cast<float>(si) / static_cast<float>(station_n - 1);
          const float s = centerline_length * s_norm;
          const auto sample = centerline.Sample(s);
          const glm::vec3 world_pos = anchor.base_position + anchor.orientation * sample.position;
          line.points_world.emplace_back(world_pos);
        }

        out_needle_skeleton_lines->emplace_back(std::move(line));
      }

      GeneralizedCylinderMesherConfig cfg;
      cfg.station_count = kStations;
      cfg.perimeter_count = kPerimeter;
      std::vector<float> station_lignification;
      std::vector<float> station_stripe_proxy;
      std::vector<float> station_stripe_scale;
      std::vector<float> station_sheath_visibility;
      station_lignification.reserve(static_cast<size_t>(kStations));
      station_stripe_proxy.reserve(static_cast<size_t>(kStations));
      station_stripe_scale.reserve(static_cast<size_t>(kStations));
      station_sheath_visibility.reserve(static_cast<size_t>(kStations));

      const float stripe_strength = cohort_stomatal_strip_density * cohort_specularity_plasticity * 0.08f;
      for (int i = 0; i < kStations; ++i) {
        const float s_norm = static_cast<float>(i) / static_cast<float>(kStations - 1);
        const float axial = std::pow(s_norm, clamped_axial_exponent);
        const float axial_shift = (axial - 0.5f) * 2.0f;
        const float segment_age_norm = std::clamp(cluster_age_norm + needle_axial_age_span * axial_shift, 0.0f, 1.0f);
        float segment_oldness = std::clamp(std::max(segment_age_norm, sen) * cohort_lignification_factor, 0.0f, 1.0f);
        float sheath_visibility = 0.0f;
        if (year2plus_cohort && sheath_budget_years > 0.0f) {
          const float sheath_progress = std::clamp(cluster_age_years / sheath_budget_years, 0.0f, 1.0f);
          const float sheath_mask = std::clamp(1.0f - s_norm * 3.5f, 0.0f, 1.0f);
          sheath_visibility = std::clamp(sheath_progress * sheath_mask, 0.0f, 1.0f);
          segment_oldness = std::clamp(std::max(segment_oldness, sheath_visibility), 0.0f, 1.0f);
        }
        const float stripe_phase = (s_norm * 48.0f + cluster.node_random * 17.0f) * glm::two_pi<float>();
        const float stripe_wave = 0.5f + 0.5f * std::sin(stripe_phase);
        const float stripe_scale = 1.0f - stripe_strength + stripe_strength * stripe_wave;
        const float stripe_proxy =
            std::clamp(cohort_stomatal_strip_density * cohort_specularity_plasticity * stripe_wave, 0.0f, 1.0f);

        station_lignification.emplace_back(segment_oldness);
        station_stripe_proxy.emplace_back(stripe_proxy);
        station_stripe_scale.emplace_back(stripe_scale);
        station_sheath_visibility.emplace_back(sheath_visibility);
      }

      if (color_mode == ScotsPine::ColorMode::ByNode) {
        const glm::vec4 node_color = HashToColor(static_cast<uint32_t>(node.GetIndex()));
        cfg.vertex_color = node_color;
        cfg.station_color_table.assign(static_cast<size_t>(kStations), node_color);
      } else if (color_mode == ScotsPine::ColorMode::ByInstance) {
        // ByInstance is driven by particle instance tint; keep vertex colors neutral.
        cfg.vertex_color = glm::vec4(1.0f);
        cfg.station_color_table.clear();
      } else {
        cfg.station_color_table.clear();
        cfg.station_color_table.reserve(static_cast<size_t>(kStations));
        for (int i = 0; i < kStations; ++i) {
          const float segment_oldness = station_lignification[static_cast<size_t>(i)];
          const float stripe_proxy = station_stripe_proxy[static_cast<size_t>(i)];
          const float stripe_scale = station_stripe_scale[static_cast<size_t>(i)];
          const float sheath_visibility = station_sheath_visibility[static_cast<size_t>(i)];
          glm::vec4 segment_color;
          if (color_mode == ScotsPine::ColorMode::NeedleLignification) {
            const glm::vec3 low(0.05f, 0.28f, 0.08f);
            const glm::vec3 high(0.75f, 0.42f, 0.10f);
            segment_color = glm::vec4(glm::mix(low, high, segment_oldness), 1.0f);
          } else if (color_mode == ScotsPine::ColorMode::NeedleStripeProxy) {
            const glm::vec3 low(0.04f, 0.08f, 0.20f);
            const glm::vec3 high(0.80f, 0.95f, 1.00f);
            segment_color = glm::vec4(glm::mix(low, high, stripe_proxy), 1.0f);
          } else if (color_mode == ScotsPine::ColorMode::NeedleSheath) {
            const glm::vec3 low(0.06f, 0.06f, 0.06f);
            const glm::vec3 high(0.95f, 0.75f, 0.20f);
            segment_color = glm::vec4(glm::mix(low, high, sheath_visibility), 1.0f);
          } else {
            segment_color = glm::mix(needle_young_color, needle_old_color, segment_oldness);
            segment_color.r *= stripe_scale;
            segment_color.g *= stripe_scale;
            segment_color.b *= stripe_scale;
          }
          cfg.station_color_table.emplace_back(segment_color);
        }
        cfg.vertex_color = cfg.station_color_table.front();
      }
      cfg.anchor_position = anchor.base_position;
      cfg.anchor_rotation = anchor.orientation;
      cfg.base_radius = base_radius_m;
      cfg.radius_table = width_radius_table;
      cfg.secondary_base_radius = secondary_base_radius_m;
      cfg.secondary_radius_table = thickness_radius_table;
      cfg.base_adaxial = glm::vec3(1, 0, 0);  // +X in needle-local frame.

      const std::size_t vertex_start = out_vertices.size();
      SweepGeneralizedCylinder<Vertex>(centerline, ellipsoid_profile, cfg, out_vertices, out_triangles);
      const std::size_t appended_vertices = out_vertices.size() - vertex_start;
      const std::size_t expected_vertices = static_cast<std::size_t>(kStations) * static_cast<std::size_t>(kPerimeter);
      const std::size_t metadata_vertices = std::min(appended_vertices, expected_vertices);
      // Pack station signals for future shader-side parity checks and overlays.
      for (std::size_t local_index = 0; local_index < metadata_vertices; ++local_index) {
        const std::size_t station_index =
            std::min(static_cast<std::size_t>(kStations - 1), local_index / static_cast<std::size_t>(kPerimeter));
        auto& vertex = out_vertices[vertex_start + local_index];
        vertex.vertex_info1 = station_lignification[station_index];
        vertex.vertex_info2 = station_stripe_proxy[station_index];
        vertex.vertex_info3 = station_sheath_visibility[station_index];
        vertex.vertex_info4.x = cohort_stomatal_strip_density;
        vertex.vertex_info4.y = cohort_specularity_plasticity;
      }
    }
  }
}

}  // namespace

// ===========================================================================

float ScotsPine::GetInfancyTargetGDD() const {
  float descriptor_tgt = 18000.0f;
  if (auto desc = const_cast<AssetRef&>(descriptor_ref).Get<ScotsPineDescriptor>()) {
    std::mt19937 rng(seed);
    descriptor_tgt = std::max(0.0f, SampleDistribution(desc->target_gdd, rng));
  }
  if (descriptor_tgt <= 0.0f) {
    return 0.0f;
  }
  constexpr float kWarmStartRatio = 0.20f;
  constexpr float kWarmStartMinGdd = 250.0f;
  constexpr float kWarmStartMaxGdd = 900.0f;
  const float warm_start = std::clamp(descriptor_tgt * kWarmStartRatio, kWarmStartMinGdd, kWarmStartMaxGdd);
  return std::min(warm_start, descriptor_tgt);
}

void ScotsPine::SetGlobalColorMode(const ColorMode mode) {
  g_scots_pine_color_mode = mode;
}
ScotsPine::ColorMode ScotsPine::GetGlobalColorMode() {
  return g_scots_pine_color_mode;
}
void ScotsPine::SetForceCpuParticlesPath(const bool force) {
  g_force_cpu_particles_path.store(force, std::memory_order_relaxed);
}
bool ScotsPine::IsForceCpuParticlesPath() {
  return g_force_cpu_particles_path.load(std::memory_order_relaxed);
}

void ScotsPine::SetInternodeVisualRadiusMultiplier(const float multiplier) {
  // Clamp to a sensible non-negative range. Caller may pass 0 to make the
  // visualised trunk vanish; negative values are nonsensical here.
  const float safe = std::isfinite(multiplier) ? std::max(multiplier, 0.0f) : 1.0f;
  g_internode_visual_radius_multiplier.store(safe, std::memory_order_relaxed);
}
float ScotsPine::GetInternodeVisualRadiusMultiplier() {
  return g_internode_visual_radius_multiplier.load(std::memory_order_relaxed);
}

void ScotsPine::SetRenderNeedlesEnabled(const bool enabled) {
  g_render_needles_enabled.store(enabled, std::memory_order_relaxed);
}
bool ScotsPine::IsRenderNeedlesEnabled() {
  return g_render_needles_enabled.load(std::memory_order_relaxed);
}

void ScotsPine::SetGenerateNeedleTopologyEnabled(const bool enabled) {
  g_generate_needle_topology_enabled.store(enabled, std::memory_order_relaxed);
}
bool ScotsPine::IsGenerateNeedleTopologyEnabled() {
  return g_generate_needle_topology_enabled.load(std::memory_order_relaxed);
}

void ScotsPine::SetLeaderInternodeDebugColor(const glm::vec4& color) {
  std::lock_guard<std::mutex> lock(g_leader_internode_debug_color_mutex);
  g_leader_internode_debug_color = color;
}
glm::vec4 ScotsPine::GetLeaderInternodeDebugColor() {
  std::lock_guard<std::mutex> lock(g_leader_internode_debug_color_mutex);
  return g_leader_internode_debug_color;
}

namespace {
std::shared_ptr<ScotsPineDescriptor> ResolvePostRepotDescriptorForGrowth(ScotsPine& pine) {
  if (!pine.enable_repot_profile_switch) {
    return nullptr;
  }
  return pine.post_repot_descriptor_ref.Get<ScotsPineDescriptor>();
}

float ResolveRepotSwitchGddForGrowth(const ScotsPine& pine,
                                     const std::shared_ptr<ScotsPineDescriptor>& post_descriptor) {
  if (!post_descriptor) {
    return -1.0f;
  }
  return std::max(0.0f, pine.repot_switch_gdd);
}

bool ResolveNeedleTopologyEnabledForGrowth() {
  return g_generate_needle_topology_enabled.load(std::memory_order_relaxed);
}
}  // namespace

// ===========================================================================

void ScotsPine::ClearGeometryEntities() const {
  if (render_target_) {
    render_target_->ClearAllChannels();
  }
  last_needle_skeleton_lines.clear();

  // One-time migration cleanup for legacy child entities from pre-channel builds.
  const auto scene = GetScene();
  if (!scene) {
    return;
  }
  const auto self = GetOwner();
  if (!scene->IsEntityValid(self)) {
    return;
  }

  const auto children = scene->GetChildren(self);
  for (const auto& child : children) {
    const auto name = scene->GetEntityName(child);
    if (name == "Pine Stem Container" || name == "Pine Needles Container" || name == "Pine Internodes" ||
        name == "Pine Needles" || name == "Pine Needles Geometry") {
      scene->DeleteEntity(child);
    }
  }
}

// ===========================================================================
// Generate / Preview / Grow
// ===========================================================================

void ScotsPine::GenerateGeometryEntities(const bool uncapped_growth) {
  ClearGeometryEntities();
  growth_model.Reset();
  GrowToTargetGDD(uncapped_growth);
}

void ScotsPine::GeneratePreviewGeometryEntities(const float preview_target_gdd,
                                                const uint32_t preview_max_growth_steps) {
  ClearGeometryEntities();
  growth_model.Reset();

  const auto& times = GetApplication().GetTimes();
  const double grow_start = times.Now();
  auto descriptor = descriptor_ref.Get<ScotsPineDescriptor>();
  if (!descriptor) {
    last_grow_seconds = 0.0;
    return;
  }
  const auto post_descriptor = ResolvePostRepotDescriptorForGrowth(*this);
  const bool enable_needle_topology = ResolveNeedleTopologyEnabledForGrowth();
  growth_model.Initialize(*descriptor, seed, glm::vec3(0), kDefaultRootRotation, post_descriptor.get(),
                          ResolveRepotSwitchGddForGrowth(*this, post_descriptor), enable_needle_topology);
  const float clamped_target_gdd = std::min(target_gdd, std::max(0.0f, preview_target_gdd));
  const uint32_t step_cap = std::max(1u, preview_max_growth_steps);
  growth_model.GrowToGDDWithProfileSwitch(clamped_target_gdd, step_cap);

  last_grow_seconds = times.Now() - grow_start;
  RebuildGeometry();
}

void ScotsPine::GrowToTargetGDD(const bool uncapped_growth) {
  (void)uncapped_growth;
  const auto& times = GetApplication().GetTimes();
  const double grow_start = times.Now();
  last_rebuild_seconds = 0.0;
  last_rebuild_internode_seconds = 0.0;
  last_needle_mesh_seconds = 0.0;
  last_mesh_upload_seconds = 0.0;
  auto descriptor = descriptor_ref.Get<ScotsPineDescriptor>();
  if (!descriptor) {
    last_grow_seconds = 0.0;
    return;
  }
  const auto post_descriptor = ResolvePostRepotDescriptorForGrowth(*this);
  const float repot_switch_gdd_value = ResolveRepotSwitchGddForGrowth(*this, post_descriptor);
  const bool enable_needle_topology = ResolveNeedleTopologyEnabledForGrowth();
  bool reinitialized = false;
  if (!growth_model.IsInitialized()) {
    growth_model.Initialize(*descriptor, seed, glm::vec3(0), kDefaultRootRotation, post_descriptor.get(),
                            repot_switch_gdd_value, enable_needle_topology);
    reinitialized = true;
  }
  const bool topology_policy_changed = growth_model.IsNeedleTopologyEnabled() != enable_needle_topology;
  // Backward-scrubbing: if the user has dragged target_gdd backward beyond a step,
  // re-init from scratch so geometry shrinks instead of being stuck at the high-water mark.
  const float gdd_step = std::max(1e-5f, growth_model.gdd_per_growth_step);
  if (topology_policy_changed || target_gdd + gdd_step < growth_model.accumulated_gdd) {
    growth_model.Initialize(*descriptor, seed, glm::vec3(0), kDefaultRootRotation, post_descriptor.get(),
                            repot_switch_gdd_value, enable_needle_topology);
    reinitialized = true;
  }
  growth_model.GrowToGDDWithProfileSwitch(target_gdd);
  last_grow_seconds = times.Now() - grow_start;

  const float current_internode_visual_radius_multiplier =
      std::max(0.0f, g_internode_visual_radius_multiplier.load(std::memory_order_relaxed));
  const bool current_render_needles_enabled = g_render_needles_enabled.load(std::memory_order_relaxed);
  const glm::vec4 current_leader_debug_color = []() {
    std::lock_guard<std::mutex> lock(g_leader_internode_debug_color_mutex);
    return g_leader_internode_debug_color;
  }();
  const int current_color_mode = static_cast<int>(GetGlobalColorMode());
  const auto approx_equal = [](const float a, const float b) {
    return std::isfinite(a) && std::isfinite(b) && std::abs(a - b) <= 1.0e-6f;
  };
  const auto vec4_equal = [&](const glm::vec4& a, const glm::vec4& b) {
    return approx_equal(a.x, b.x) && approx_equal(a.y, b.y) && approx_equal(a.z, b.z) && approx_equal(a.w, b.w);
  };
  const bool visual_settings_changed =
      !approx_equal(last_applied_internode_visual_radius_multiplier, current_internode_visual_radius_multiplier) ||
      last_applied_render_needles_enabled != current_render_needles_enabled ||
      !vec4_equal(last_applied_leader_debug_color, current_leader_debug_color) ||
      last_applied_color_mode != current_color_mode;

  // Auto-grow calls this every frame; when no growth step was taken, a full
  // mesh rebuild is wasted work. Preserve exact behavior on explicit
  // reinitialization (backward scrub), where geometry must always be refreshed.
  // But loaded scenes may deserialize stale particle buffers; if any visual-only
  // knob changed, force one rebuild even when growth itself did not advance.
  if (!reinitialized && growth_model.last_growth_steps == 0 && !visual_settings_changed) {
    return;
  }
  RebuildGeometry();
}

void ScotsPine::SetSeasonalChronologicalMode(const bool enable_independent_chronological_clock) {
  growth_model.SetChronologicalCoupledToThermal(!enable_independent_chronological_clock);
}

bool ScotsPine::AdvanceChronologicalAging(const float delta_years) {
  if (!std::isfinite(delta_years) || delta_years <= 0.0f) {
    return false;
  }

  auto descriptor = descriptor_ref.Get<ScotsPineDescriptor>();
  if (!descriptor) {
    return false;
  }

  if (!growth_model.IsInitialized()) {
    const auto post_descriptor = ResolvePostRepotDescriptorForGrowth(*this);
    const bool enable_needle_topology = ResolveNeedleTopologyEnabledForGrowth();
    growth_model.Initialize(*descriptor, seed, glm::vec3(0), kDefaultRootRotation, post_descriptor.get(),
                            ResolveRepotSwitchGddForGrowth(*this, post_descriptor), enable_needle_topology);
  }

  growth_model.AdvanceChronologicalYears(delta_years);
  const bool changed = growth_model.AgeOnlyStep();
  if (changed) {
    RebuildGeometry();
  }
  return changed;
}

// ===========================================================================
// Rebuild geometry from current growth model state
// ===========================================================================

void ScotsPine::RebuildGeometry() {
  const auto& times = GetApplication().GetTimes();
  const double rebuild_start = times.Now();
  last_invalid_instance_count = 0;
  last_internode_count = 0;
  last_needle_count = 0;
  last_node_count = 0;
  last_rebuild_internode_seconds = 0.0;
  last_needle_mesh_seconds = 0.0;
  last_mesh_upload_seconds = 0.0;

  if (!growth_model.IsInitialized()) {
    last_rebuild_seconds = 0.0;
    return;
  }

  const auto scene = GetScene();
  const auto owner = GetOwner();
  if (!scene || !scene->IsEntityValid(owner)) {
    last_rebuild_seconds = 0.0;
    return;
  }

  if (!render_target_ || render_target_->GetRootEntity() != owner) {
    render_target_ = std::make_unique<PlantRenderTarget>(scene, owner);
  }

  const auto color_mode = GetGlobalColorMode();
  const glm::vec4 instance_color = HashToColor(owner.GetIndex());
  const glm::vec4 kDefaultNeedleColor(0.16f, 0.45f, 0.18f, 1.0f);
  const glm::vec4 kDefaultNeedleOldColor(0.42f, 0.27f, 0.10f, 1.0f);
  const glm::vec4 kDefaultStemColor(0.83f, 0.72f, 0.50f, 1.0f);
  const glm::vec4 kDefaultStemOldColor(0.45f, 0.30f, 0.20f, 1.0f);
  constexpr float kDefaultNeedleAxialAgeSpan = 0.35f;
  constexpr float kDefaultNeedleAxialAgeExponent = 1.0f;
  constexpr float kDefaultInternodeAgeExponent = 1.0f;

  glm::vec4 needle_base_color = kDefaultNeedleColor;
  glm::vec4 needle_old_color = kDefaultNeedleOldColor;
  glm::vec4 stem_base_color = kDefaultStemColor;
  glm::vec4 stem_old_color = kDefaultStemOldColor;
  float needle_axial_age_span = kDefaultNeedleAxialAgeSpan;
  float needle_axial_age_exponent = kDefaultNeedleAxialAgeExponent;
  float internode_age_exponent = kDefaultInternodeAgeExponent;
  if (const auto descriptor = descriptor_ref.Get<ScotsPineDescriptor>()) {
    needle_base_color = descriptor->needle_color_rgba;
    needle_old_color = descriptor->needle_old_color_rgba;
    stem_base_color = descriptor->main_stem_color_rgba;
    stem_old_color = descriptor->main_stem_old_color_rgba;
    needle_axial_age_span = descriptor->needle_axial_age_span;
    needle_axial_age_exponent = descriptor->needle_axial_age_exponent;
    internode_age_exponent = descriptor->internode_age_exponent;
  }
  needle_base_color = SanitizeFiniteColor(needle_base_color, kDefaultNeedleColor);
  needle_old_color = SanitizeFiniteColor(needle_old_color, kDefaultNeedleOldColor);
  stem_base_color = SanitizeFiniteColor(stem_base_color, kDefaultStemColor);
  stem_old_color = SanitizeFiniteColor(stem_old_color, kDefaultStemOldColor);

  const float needle_color_energy = std::max(needle_base_color.r, std::max(needle_base_color.g, needle_base_color.b));
  const float needle_old_color_energy = std::max(needle_old_color.r, std::max(needle_old_color.g, needle_old_color.b));
  if (needle_color_energy <= 1.0e-4f && needle_old_color_energy <= 1.0e-4f) {
    needle_base_color = kDefaultNeedleColor;
    needle_old_color = kDefaultNeedleOldColor;
  }
  const float stem_color_energy = std::max(stem_base_color.r, std::max(stem_base_color.g, stem_base_color.b));
  const float stem_old_color_energy = std::max(stem_old_color.r, std::max(stem_old_color.g, stem_old_color.b));
  if (stem_color_energy <= 1.0e-4f && stem_old_color_energy <= 1.0e-4f) {
    stem_base_color = kDefaultStemColor;
    stem_old_color = kDefaultStemOldColor;
  }

  const float t_now_years = growth_model.graph.data.clock.NowYears();
  const float max_internode_age_years = std::max(1.0f, static_cast<float>(growth_model.sampled.needle_lifespan_years));

  static thread_local std::vector<ParticleInfo> internode_infos_cache;
  static thread_local std::vector<ParticleInfo> needle_infos_cache;

  const auto& sorted = growth_model.graph.PeekSortedNodeList();
  last_node_count = static_cast<uint32_t>(sorted.size());

  // -- Internodes (instance channel) --
  {
    const double internode_start = times.Now();
    auto& infos = internode_infos_cache;
    infos.clear();
    infos.reserve(sorted.size());
    const glm::quat cylinder_axis_fix = glm::angleAxis(-glm::half_pi<float>(), glm::vec3(1.0f, 0.0f, 0.0f));

    const float internode_visual_radius_multiplier =
        std::max(0.0f, g_internode_visual_radius_multiplier.load(std::memory_order_relaxed));
    const glm::vec4 leader_debug_color = []() {
      std::lock_guard<std::mutex> lock(g_leader_internode_debug_color_mutex);
      return g_leader_internode_debug_color;
    }();
    const bool leader_debug_color_active = leader_debug_color.a > 0.0f;

    for (const auto handle : sorted) {
      const auto& node = growth_model.graph.PeekNode(handle);
      if (!node.data.template Is<PineInternode>())
        continue;
      const auto& internode = node.data.template Get<PineInternode>();
      if (node.info.length <= 0.0f)
        continue;
      if (!IsFiniteVec3(node.info.global_position) || !std::isfinite(node.info.length) ||
          !std::isfinite(node.info.thickness)) {
        last_invalid_instance_count++;
        continue;
      }
      const float per_node_visual_multiplier = node.info.order == 0 ? internode_visual_radius_multiplier : 1.0f;
      const float half_thick = node.info.thickness * 0.5f * per_node_visual_multiplier;
      if (half_thick <= 0.0f)
        continue;

      const float internode_age_years = std::max(0.0f, t_now_years - internode.continuous_growth.t_init_years);
      float internode_age_norm = std::clamp(internode_age_years / max_internode_age_years, 0.0f, 1.0f);
      internode_age_norm = std::pow(internode_age_norm, std::max(0.1f, internode_age_exponent));
      glm::vec4 stem_age_color = glm::mix(stem_base_color, stem_old_color, internode_age_norm);
      stem_age_color.a = 1.0f;
      const glm::vec4 stem_type_color(stem_base_color.r, stem_base_color.g, stem_base_color.b, 1.0f);

      glm::quat instance_rotation = glm::normalize(node.info.global_rotation * cylinder_axis_fix);
      if (!IsFiniteQuat(instance_rotation)) {
        instance_rotation = glm::quat(1, 0, 0, 0);
        last_invalid_instance_count++;
      }

      const glm::mat4 model = glm::translate(node.info.global_position) * glm::mat4_cast(instance_rotation) *
                              glm::scale(glm::vec3(half_thick, node.info.length, half_thick));
      if (!IsFiniteMat4(model)) {
        last_invalid_instance_count++;
        continue;
      }

      ParticleInfo pi;
      pi.instance_matrix.value = model;
      if (color_mode == ColorMode::ByNode) {
        pi.instance_color = HashToColor(static_cast<uint32_t>(node.GetIndex()));
      } else if (color_mode == ColorMode::ByInstance) {
        pi.instance_color = instance_color;
      } else if (color_mode == ColorMode::ByType) {
        pi.instance_color = stem_type_color;
      } else {
        pi.instance_color = stem_age_color;
      }
      if (leader_debug_color_active && node.info.order == 0) {
        pi.instance_color = leader_debug_color;
      }
      pi.instance_color.a = 1.0f;
      infos.push_back(pi);
    }

    last_internode_count = static_cast<uint32_t>(infos.size());
    if (infos.empty()) {
      render_target_->RemoveInstanceChannel(kChannelInternodes);
    } else {
      std::shared_ptr<Mesh> internode_mesh;
      std::shared_ptr<Material> internode_material;
      const auto& channels = render_target_->GetInstanceChannels();
      if (const auto it = channels.find(kChannelInternodes); it != channels.end()) {
        internode_mesh = it->second->GetInstanceMesh();
        internode_material = it->second->GetInstanceMaterial();
      }
      if (!internode_mesh) {
        internode_mesh = AssetManager::CreateTemporaryAsset<Mesh>();
      }
      if (!internode_material) {
        internode_material = AssetManager::CreateTemporaryAsset<Material>();
      }

      bool needs_mesh_rebuild = true;
      if (internode_mesh) {
        const auto& vertices = internode_mesh->UnsafeGetVertices();
        const auto& triangles = internode_mesh->UnsafeGetTriangles();
        needs_mesh_rebuild = vertices.empty() || triangles.empty();
        if (!needs_mesh_rebuild) {
          for (const auto& vertex : vertices) {
            const auto color = vertex.color;
            const bool near_white = std::abs(color.r - 1.0f) <= 1.0e-3f && std::abs(color.g - 1.0f) <= 1.0e-3f &&
                                    std::abs(color.b - 1.0f) <= 1.0e-3f;
            if (!IsFiniteVec4(color) || !near_white) {
              needs_mesh_rebuild = true;
              break;
            }
          }
        }
      }

      if (needs_mesh_rebuild) {
        std::vector<Vertex> cyl_verts;
        std::vector<unsigned int> cyl_idx;
        GenerateUnitCylinderMesh(cyl_verts, cyl_idx, glm::vec4(1.0f));
        VertexAttributes attrs{};
        attrs.normal = true;
        attrs.color = true;
        attrs.tex_coord = true;
        internode_mesh->SetVertices(attrs, cyl_verts, cyl_idx);
      }

      internode_material->vertex_color_only = true;
      internode_material->SetAlbedoTexture(nullptr);
      internode_material->material_properties.albedo_color = glm::vec3(1.0f);
      internode_material->draw_settings.blending = false;
      internode_material->material_properties.metallic = 0.0f;
      internode_material->material_properties.specular = 0.12f;
      internode_material->material_properties.specular_tint = 0.05f;
      internode_material->material_properties.roughness = 0.86f;
      internode_material->material_properties.transmission = 0.0f;
      internode_material->material_properties.subsurface_factor = 0.0f;
      internode_material->material_properties.clear_coat = 0.0f;

      if (auto* channel = render_target_->GetOrCreateInstanceChannel(kChannelInternodes, "Pine Internodes",
                                                                     internode_mesh, internode_material)) {
        channel->Stage(std::move(infos));
      }
    }
    last_rebuild_internode_seconds = times.Now() - internode_start;
  }

  // -- Needles (mesh channel by default, instance channel legacy fallback) --
  const bool use_new_needle_geometry = g_use_generalized_cylinder_needles.load(std::memory_order_relaxed);
  const bool render_needles_enabled = g_render_needles_enabled.load(std::memory_order_relaxed);

  if (!render_needles_enabled) {
    render_target_->RemoveMeshChannel(kChannelNeedles);
    render_target_->RemoveInstanceChannel(kChannelNeedles);
    last_needle_skeleton_lines.clear();
    last_needle_count = 0;
  } else if (use_new_needle_geometry) {
    static thread_local std::vector<Vertex> needle_geom_vertices;
    static thread_local std::vector<glm::uvec3> needle_geom_triangles;

    const bool senescence_active = HasNeedleSenescenceOrAbscission(growth_model.graph, sorted);
    const bool growth_active_lod = growth_model.last_growth_steps > 0 && !senescence_active;
    const int needle_station_count = std::max(4, growth_model.sampled.needle_segment_count + 1);
    const int needle_perimeter_count = growth_active_lod ? 5 : 8;

    {
      const double needle_mesh_start = times.Now();
      BuildPineNeedleAggregateMesh(
          growth_model.graph, sorted, color_mode, needle_base_color, needle_old_color, needle_axial_age_span,
          needle_axial_age_exponent, growth_model.sampled.distributions.needle_cross_section_width_profile,
          growth_model.sampled.distributions.needle_cross_section_thickness_profile,
          growth_model.sampled.distributions.needle_cross_section_temporal_maturity_curve,
          growth_model.sampled.needle_fascicular_start_year, growth_model.sampled.needle_lignification_factor_year1,
          growth_model.sampled.needle_lignification_factor_year2plus,
          growth_model.sampled.needle_stomatal_strip_density_year1,
          growth_model.sampled.needle_stomatal_strip_density_year2plus,
          growth_model.sampled.needle_basal_taper_ratio_year1, growth_model.sampled.needle_basal_taper_ratio_year2plus,
          growth_model.sampled.needle_fascicle_sheath_budget_years,
          growth_model.sampled.needle_specularity_plasticity_year1,
          growth_model.sampled.needle_specularity_plasticity_year2plus, needle_geom_vertices, needle_geom_triangles,
          needle_station_count, needle_perimeter_count, &last_needle_skeleton_lines);

      SanitizeNeedleAggregateMeshVertices(needle_geom_vertices);
      last_needle_count =
          needle_geom_triangles.empty()
              ? 0u
              : static_cast<uint32_t>(needle_geom_vertices.size() /
                                      static_cast<size_t>(needle_station_count * needle_perimeter_count));
      last_needle_mesh_seconds = times.Now() - needle_mesh_start;
    }

    const bool needle_mesh_valid = IsNeedleAggregateMeshValid(needle_geom_vertices, needle_geom_triangles);
    if (!needle_mesh_valid) {
      last_invalid_instance_count++;
    }

    render_target_->RemoveInstanceChannel(kChannelNeedles);
    if (needle_mesh_valid && !needle_geom_triangles.empty()) {
      const glm::vec3 by_instance_tint = glm::clamp(glm::vec3(instance_color), glm::vec3(0.0f), glm::vec3(1.0f));

      if (auto* mesh_channel = render_target_->GetOrCreateMeshChannel(kChannelNeedles, "Pine Needles Geometry")) {
        if (const auto needle_material = mesh_channel->GetMaterial()) {
          const float needle_specularity =
              std::clamp(0.5f * (growth_model.sampled.needle_specularity_plasticity_year1 +
                                 growth_model.sampled.needle_specularity_plasticity_year2plus),
                         0.0f, 1.0f);
          const float stomatal_density =
              std::clamp(0.5f * (growth_model.sampled.needle_stomatal_strip_density_year1 +
                                 growth_model.sampled.needle_stomatal_strip_density_year2plus),
                         0.0f, 1.0f);
          const float needle_lignification =
              std::clamp(0.5f * (growth_model.sampled.needle_lignification_factor_year1 +
                                 growth_model.sampled.needle_lignification_factor_year2plus),
                         0.0f, 2.0f);

          needle_material->vertex_color_only = true;
          needle_material->SetAlbedoTexture(nullptr);
          needle_material->material_properties.albedo_color =
              color_mode == ColorMode::ByInstance ? by_instance_tint : glm::vec3(1.0f);
          needle_material->draw_settings.blending = false;
          needle_material->draw_settings.cull_mode = VK_CULL_MODE_NONE;
          needle_material->material_properties.metallic = 0.0f;
          needle_material->material_properties.specular = std::clamp(0.04f + 0.08f * needle_specularity, 0.04f, 0.12f);
          needle_material->material_properties.specular_tint =
              std::clamp(0.02f + 0.05f * stomatal_density, 0.0f, 0.10f);
          needle_material->material_properties.roughness =
              std::clamp(0.90f + 0.06f * needle_lignification, 0.85f, 0.98f);
          needle_material->material_properties.subsurface_factor = 0.0f;
          needle_material->material_properties.ior = 1.33f;
          needle_material->material_properties.transmission = 0.0f;
          needle_material->material_properties.transmission_roughness = 1.0f;
          needle_material->material_properties.clear_coat = 0.0f;
          needle_material->material_properties.clear_coat_roughness = 1.0f;
          needle_material->material_properties.emission = 0.0f;
        }

        VertexAttributes attrs{};
        attrs.normal = true;
        attrs.tangent = true;
        attrs.color = true;
        attrs.tex_coord = true;
        mesh_channel->Stage(attrs, std::move(needle_geom_vertices), std::move(needle_geom_triangles));
      }
    } else {
      render_target_->RemoveMeshChannel(kChannelNeedles);
    }
  } else {
    last_needle_skeleton_lines.clear();
    auto& infos = needle_infos_cache;
    infos.clear();

    for (const auto handle : sorted) {
      const auto& node = growth_model.graph.PeekNode(handle);
      if (!node.data.template Is<PineNeedleCluster>())
        continue;
      const auto& cluster = node.data.template Get<PineNeedleCluster>();
      const glm::vec3 pos = node.info.global_position;
      if (!IsFiniteVec3(pos)) {
        last_invalid_instance_count++;
        continue;
      }
      const float size = std::max(0.005f, cluster.target_length * 0.05f);
      ParticleInfo pi;
      const glm::mat4 model = glm::translate(pos) * glm::scale(glm::vec3(size));
      if (!IsFiniteMat4(model)) {
        last_invalid_instance_count++;
        continue;
      }
      pi.instance_matrix.value = model;
      if (color_mode == ColorMode::ByNode) {
        pi.instance_color = HashToColor(static_cast<uint32_t>(node.GetIndex()));
      } else if (color_mode == ColorMode::ByInstance) {
        pi.instance_color = instance_color;
      } else {
        pi.instance_color = needle_base_color;
      }
      infos.push_back(pi);
    }

    last_needle_count = static_cast<uint32_t>(infos.size());
    render_target_->RemoveMeshChannel(kChannelNeedles);
    if (infos.empty()) {
      render_target_->RemoveInstanceChannel(kChannelNeedles);
    } else {
      std::shared_ptr<Mesh> needle_mesh;
      std::shared_ptr<Material> needle_material;
      const auto& channels = render_target_->GetInstanceChannels();
      if (const auto it = channels.find(kChannelNeedles); it != channels.end()) {
        needle_mesh = it->second->GetInstanceMesh();
        needle_material = it->second->GetInstanceMaterial();
      }
      if (!needle_mesh) {
        needle_mesh = AssetManager::CreateTemporaryAsset<Mesh>();
      }
      if (!needle_material) {
        needle_material = AssetManager::CreateTemporaryAsset<Material>();
      }

      std::vector<Vertex> oct_verts;
      std::vector<unsigned int> oct_idx;
      GenerateUnitOctahedronMesh(oct_verts, oct_idx, glm::vec4(1.0f));
      VertexAttributes attrs{};
      attrs.normal = true;
      attrs.color = true;
      attrs.tex_coord = true;
      needle_mesh->SetVertices(attrs, oct_verts, oct_idx);

      needle_material->vertex_color_only = false;
      needle_material->SetAlbedoTexture(nullptr);
      needle_material->material_properties.albedo_color = glm::vec3(1.0f);
      needle_material->draw_settings.blending = false;

      if (auto* channel = render_target_->GetOrCreateInstanceChannel(kChannelNeedles, "Pine Needles", needle_mesh,
                                                                     needle_material)) {
        channel->Stage(std::move(infos));
      }
    }
  }

  if (render_target_) {
    const double upload_start = times.Now();
    render_target_->FlushPending();
    last_mesh_upload_seconds = times.Now() - upload_start;
  }

  last_applied_internode_visual_radius_multiplier =
      std::max(0.0f, g_internode_visual_radius_multiplier.load(std::memory_order_relaxed));
  last_applied_render_needles_enabled = g_render_needles_enabled.load(std::memory_order_relaxed);
  {
    std::lock_guard<std::mutex> lock(g_leader_internode_debug_color_mutex);
    last_applied_leader_debug_color = g_leader_internode_debug_color;
  }
  last_applied_color_mode = static_cast<int>(GetGlobalColorMode());
  last_rebuild_seconds = times.Now() - rebuild_start;
}

// ===========================================================================
// Export
// ===========================================================================

void ScotsPine::ExportObj(const std::filesystem::path& path) const {
  const auto scene = GetScene();
  if (!scene)
    return;

  if (!render_target_) {
    EVOENGINE_ERROR("Pine mesh export failed: no render channels available.");
    return;
  }

  std::vector<Vertex> vertices;
  std::vector<glm::uvec3> triangles;

  for (const auto& [channel_id, channel] : render_target_->GetInstanceChannels()) {
    (void)channel_id;
    if (!channel) {
      continue;
    }
    const auto entity = channel->GetEntity();
    if (scene->IsEntityValid(entity)) {
      AppendParticlesToMesh(scene, entity, vertices, triangles);
    }
  }
  for (const auto& [channel_id, channel] : render_target_->GetMeshChannels()) {
    (void)channel_id;
    if (!channel) {
      continue;
    }
    const auto entity = channel->GetEntity();
    if (scene->IsEntityValid(entity)) {
      AppendMeshRendererToMesh(scene, entity, vertices, triangles);
    }
  }

  if (vertices.empty() || triangles.empty()) {
    EVOENGINE_ERROR("Pine mesh export failed: no pine channel geometry available.");
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
    EVOENGINE_ERROR("Pine mesh export failed!");
  }
}

void ScotsPine::ExportFlowGraph(YAML::Emitter& out) {
  out << YAML::Key << "Flows" << YAML::Value << YAML::BeginSeq;
  if (!growth_model.IsInitialized()) {
    out << YAML::EndSeq;
    return;
  }
  auto& graph = growth_model.graph;
  graph.SortLists();
  graph.CalculateFlows();
  const auto retained = CollectInternodeFlowHandles(graph);
  for (const auto flow_handle : graph.PeekSortedFlowList()) {
    if (retained.find(flow_handle) == retained.end())
      continue;
    const auto& flow = graph.PeekFlow(flow_handle);
    const auto parent_flow_handle = FindParentInternodeFlowHandle(graph, flow_handle, retained);
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

void ScotsPine::ExportFlowGraph(const std::filesystem::path& path) {
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

void ScotsPine::ExportNodeGraph(YAML::Emitter& out) {
  out << YAML::Key << "Nodes" << YAML::Value << YAML::BeginSeq;
  if (!growth_model.IsInitialized()) {
    out << YAML::EndSeq;
    return;
  }
  auto& graph = growth_model.graph;
  graph.SortLists();
  graph.CalculateFlows();
  const auto retained = CollectInternodeFlowHandles(graph);
  for (const auto node_handle : graph.PeekSortedNodeList()) {
    const auto& node = graph.PeekNode(node_handle);
    if (!IsInternodeNode(node))
      continue;
    const auto parent_node_handle = FindParentInternodeNodeHandle(graph, node_handle);
    auto flow_handle = node.GetFlowHandle();
    while (flow_handle >= 0 && retained.find(flow_handle) == retained.end()) {
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

void ScotsPine::ExportNodeGraph(const std::filesystem::path& path) {
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

void ScotsPine::ExportNeedleSkeleton(YAML::Emitter& out) {
  const int segment_count = growth_model.IsInitialized() ? std::max(1, growth_model.sampled.needle_segment_count) : 1;

  out << YAML::Key << "NeedleSegmentCount" << YAML::Value << segment_count;
  out << YAML::Key << "NeedleStationCount" << YAML::Value << (segment_count + 1);
  out << YAML::Key << "Needles" << YAML::Value << YAML::BeginSeq;
  for (const auto& line : last_needle_skeleton_lines) {
    out << YAML::BeginMap;
    out << YAML::Key << "CI" << YAML::Value << line.cluster_node_handle;
    out << YAML::Key << "PI" << YAML::Value << line.parent_node_handle;
    out << YAML::Key << "NI" << YAML::Value << line.needle_index;
    out << YAML::Key << "P" << YAML::Value << YAML::BeginSeq;
    for (const auto& point : line.points_world) {
      out << point;
    }
    out << YAML::EndSeq;
    out << YAML::EndMap;
  }
  out << YAML::EndSeq;
}

void ScotsPine::ExportNeedleSkeleton(const std::filesystem::path& path) {
  try {
    YAML::Emitter out;
    out << YAML::BeginMap;
    ExportNeedleSkeleton(out);
    out << YAML::EndMap;
    std::ofstream output_file(path.string());
    output_file << out.c_str();
    output_file.flush();
  } catch (const std::exception& e) {
    EVOENGINE_ERROR(std::string("Failed to save: ") + e.what());
  }
}

// ===========================================================================
// Component lifecycle / inspector
// ===========================================================================

void ScotsPine::OnDestroy() {
  ClearGeometryEntities();
  render_target_.reset();
}

bool l_system_package::InspectScotsPine(InspectorContext& context, ScotsPine& pine) {
  const auto& editor_layer = context.editor_layer;
  bool changed = false;

  if (editor_layer->DragAndDropButton<ScotsPineDescriptor>(pine.descriptor_ref, "Descriptor"))
    changed = true;

  if (editor_layer->DragAndDropButton<ScotsPineDescriptor>(pine.post_repot_descriptor_ref, "Post-Repot Descriptor")) {
    changed = true;
  }

  if (ImGui::Checkbox("Enable Repot Profile Switch", &pine.enable_repot_profile_switch)) {
    changed = true;
  }
  if (pine.enable_repot_profile_switch) {
    if (ImGui::DragFloat("Repot Switch GDD", &pine.repot_switch_gdd, 10.0f, 0.0f, 200000.0f, "%.1f")) {
      pine.repot_switch_gdd = std::max(0.0f, pine.repot_switch_gdd);
      changed = true;
    }
  }

  int seed_int = static_cast<int>(pine.seed);
  if (ImGui::DragInt("Seed", &seed_int, 1, 0, 999999)) {
    pine.seed = static_cast<unsigned int>(seed_int);
    changed = true;
  }

  if (ImGui::DragFloat("Target GDD", &pine.target_gdd, 1.0f, 0.0f, 200000.0f, "%.1f"))
    changed = true;

  if (ImGui::Button("Generate")) {
    pine.GenerateGeometryEntities();
    changed = true;
  }
  ImGui::SameLine();
  if (ImGui::Button("Clear")) {
    pine.ClearGeometryEntities();
    changed = true;
  }

  if (pine.growth_model.IsInitialized()) {
    ImGui::Separator();
    ImGui::Text("GDD: %.1f", pine.growth_model.accumulated_gdd);
    ImGui::Text("Topology: %s", pine.growth_model.IsTopologyComplete() ? "Complete" : "Pending");
    const auto& sorted = pine.growth_model.graph.PeekSortedNodeList();
    ImGui::Text("Nodes: %d", static_cast<int>(sorted.size()));
    ImGui::Text("Internodes: %u   Needles: %u", pine.last_internode_count, pine.last_needle_count);
  }

  return changed;
}

void l_system_package::SerializeScotsPine(YAML::Emitter& out, const ScotsPine& target) {
  target.descriptor_ref.Save("descriptor_ref", out);
  target.post_repot_descriptor_ref.Save("post_repot_descriptor_ref", out);
  out << YAML::Key << "seed" << YAML::Value << target.seed;
  out << YAML::Key << "target_gdd" << YAML::Value << target.target_gdd;
  out << YAML::Key << "enable_repot_profile_switch" << YAML::Value << target.enable_repot_profile_switch;
  out << YAML::Key << "repot_switch_gdd" << YAML::Value << target.repot_switch_gdd;
}

void l_system_package::DeserializeScotsPine(const YAML::Node& in, ScotsPine& target) {
  target.descriptor_ref.Load("descriptor_ref", in);
  target.post_repot_descriptor_ref.Load("post_repot_descriptor_ref", in);
  if (in["seed"])
    target.seed = in["seed"].as<unsigned int>();
  if (in["target_gdd"]) {
    target.target_gdd = in["target_gdd"].as<float>();
  } else if (in["target_year"]) {
    target.target_gdd = static_cast<float>(in["target_year"].as<int>()) * kPineGddPerYear;
  }
  if (in["enable_repot_profile_switch"]) {
    target.enable_repot_profile_switch = in["enable_repot_profile_switch"].as<bool>();
  }
  if (in["repot_switch_gdd"]) {
    target.repot_switch_gdd = std::max(0.0f, in["repot_switch_gdd"].as<float>());
  }
}

void ScotsPine::CollectAssetRef(std::vector<AssetRef>& list) {
  list.push_back(descriptor_ref);
  list.push_back(post_repot_descriptor_ref);
}
