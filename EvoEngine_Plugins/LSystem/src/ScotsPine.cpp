#include "ScotsPine.hpp"
#include "ScotsPineDescriptor.hpp"
#include "ScotsPineModules.hpp"

// Phase 1 biologically-emergent organ geometry primitives.
#include "CrossSectionProfile.hpp"
#include "GeneralizedCylinderMesher.hpp"
#include "OrganCenterline.hpp"
#include "GrowthField.hpp"
#include "ElasticaSolver.hpp"
#include "MaterialProfile.hpp"
#include "OrganMeshPly.hpp"

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
#include <glm/gtx/quaternion.hpp>
#include <limits>
#include <unordered_set>

using namespace l_system_plugin;
using namespace evo_engine;

namespace {

ScotsPine::ColorMode g_scots_pine_color_mode = ScotsPine::ColorMode::Shaded;
std::atomic<bool> g_force_cpu_particles_path{false};

// Phase 1: when true, replace the per-cluster octahedron marker (legacy,
// kept under [deprecated] block below) with explicit swept generalized-
// cylinder needle geometry. Defaults true; set to false to compare against
// the legacy visual baseline.
std::atomic<bool> g_use_generalized_cylinder_needles{true};

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
  return std::isfinite(v.x) && std::isfinite(v.y) &&
         std::isfinite(v.z) && std::isfinite(v.w);
}

glm::vec4 SanitizeFiniteColor(const glm::vec4& value,
                              const glm::vec4& fallback) {
  if (!IsFiniteVec4(value)) return fallback;
  return glm::clamp(value, glm::vec4(0.0f), glm::vec4(1.0f));
}

bool IsFiniteMat4(const glm::mat4& m) {
  for (int c = 0; c < 4; c++) {
    for (int r = 0; r < 4; r++) {
      if (!std::isfinite(m[c][r])) return false;
    }
  }
  return true;
}

bool IsNeedleAggregateMeshValid(const std::vector<Vertex>& vertices,
                                const std::vector<glm::uvec3>& triangles) {
  if (vertices.empty() || triangles.empty()) return true;

  for (const auto& v : vertices) {
    if (!IsFiniteVec3(v.position) ||
        !IsFiniteVec3(v.normal) ||
        !IsFiniteVec3(v.tangent) ||
        !IsFiniteVec2(v.tex_coord) ||
        !IsFiniteVec4(v.color)) {
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

bool HasNeedleSenescenceOrAbscission(const PineGraph& graph,
                                     const std::vector<LNodeHandle>& sorted_nodes) {
  for (const auto handle : sorted_nodes) {
    const auto& node = graph.PeekNode(handle);
    if (!node.data.template Is<PineNeedleCluster>()) continue;
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
    case 0: rgb = glm::vec3(v, t, p); break;
    case 1: rgb = glm::vec3(q, v, p); break;
    case 2: rgb = glm::vec3(p, v, t); break;
    case 3: rgb = glm::vec3(p, q, v); break;
    case 4: rgb = glm::vec3(t, p, v); break;
    default: rgb = glm::vec3(v, p, q); break;
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

float EvaluatePositionalMultiplier(const evo_engine::PlottedDistribution<float>& distribution,
                                   const float s_norm,
                                   const float node_random,
                                   const uint32_t salt) {
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

float EvaluateTemporalCrossSectionMultiplier(
    const evo_engine::PlottedDistribution<float>& distribution,
    const float cluster_age_years,
    const float node_random,
    const uint32_t salt) {
  // Curve x-domain is [0,1]; map 0..2 years of cluster age onto that axis.
  const float age_norm = std::clamp(
      cluster_age_years / std::max(1.0e-6f, kNeedleCrossSectionTemporalWindowYears),
      0.0f, 1.0f);
  return EvaluatePositionalMultiplier(distribution, age_norm, node_random, salt);
}

bool IsInternodeNode(const PineNode& node) {
  return node.data.template Is<PineInternode>();
}

LNodeHandle FindParentInternodeNodeHandle(const PineGraph& graph, const LNodeHandle node_handle) {
  auto parent_handle = graph.PeekNode(node_handle).GetParentHandle();
  while (parent_handle >= 0) {
    const auto& parent = graph.PeekNode(parent_handle);
    if (IsInternodeNode(parent)) return parent_handle;
    parent_handle = parent.GetParentHandle();
  }
  return -1;
}

std::unordered_set<LFlowHandle> CollectInternodeFlowHandles(const PineGraph& graph) {
  std::unordered_set<LFlowHandle> retained;
  for (const auto flow_handle : graph.PeekSortedFlowList()) {
    const auto& flow = graph.PeekFlow(flow_handle);
    const auto& node_handles = flow.PeekNodeHandles();
    if (node_handles.empty()) continue;
    bool has_internode = false;
    for (const auto h : node_handles) {
      if (IsInternodeNode(graph.PeekNode(h))) {
        has_internode = true;
        break;
      }
    }
    if (has_internode) retained.emplace(flow_handle);
  }
  return retained;
}

LFlowHandle FindParentInternodeFlowHandle(const PineGraph& graph, const LFlowHandle flow_handle,
                                          const std::unordered_set<LFlowHandle>& retained) {
  auto parent_flow_handle = graph.PeekFlow(flow_handle).GetParentHandle();
  while (parent_flow_handle >= 0) {
    if (retained.find(parent_flow_handle) != retained.end()) return parent_flow_handle;
    parent_flow_handle = graph.PeekFlow(parent_flow_handle).GetParentHandle();
  }
  return -1;
}

void AppendParticlesToMesh(const std::shared_ptr<Scene>& scene, const Entity& entity,
                           std::vector<Vertex>& out_vertices, std::vector<glm::uvec3>& out_triangles) {
  if (!scene->IsEntityValid(entity) || !scene->HasPrivateComponent<Particles>(entity)) return;
  const auto particles = scene->GetOrSetPrivateComponent<Particles>(entity).lock();
  if (!particles) return;
  const auto mesh = particles->mesh.Get<Mesh>();
  const auto particle_info_list = particles->particle_info_list.Get<ParticleInfoList>();
  if (!mesh || !particle_info_list) return;
  const auto& source_vertices = mesh->UnsafeGetVertices();
  const auto& source_triangles = mesh->UnsafeGetTriangles();
  const auto& instances = particle_info_list->PeekParticleInfoList();
  if (source_vertices.empty() || source_triangles.empty() || instances.empty()) return;

  const auto entity_global_transform = scene->GetDataComponent<GlobalTransform>(entity);
  for (const auto& instance : instances) {
    const glm::mat4 world_transform = entity_global_transform.value * instance.instance_matrix.value;
    if (!IsFiniteMat4(world_transform)) continue;
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
      if (IsFiniteVec3(tn) && glm::length(tn) > 1e-8f) v.normal = glm::normalize(tn);
      const glm::vec3 tt = normal_transform * sv.tangent;
      if (IsFiniteVec3(tt) && glm::length(tt) > 1e-8f) v.tangent = glm::normalize(tt);
      v.color = instance.instance_color;
      out_vertices.emplace_back(v);
    }
    for (const auto& st : source_triangles) {
      out_triangles.emplace_back(vertex_offset + st.x, vertex_offset + st.y, vertex_offset + st.z);
    }
  }
}

// Unit cylinder: radius=1, height=1, along +Y, base at y=0.
void GenerateUnitCylinderMesh(std::vector<Vertex>& vertices, std::vector<unsigned int>& indices,
                              const glm::vec4& bark_color,
                              int segments = 6) {
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
    indices.push_back(s0); indices.push_back(e0); indices.push_back(s1);
    indices.push_back(s1); indices.push_back(e0); indices.push_back(e1);
  }
}

// Unit octahedron centered at origin, radius=1 along each axis.
void GenerateUnitOctahedronMesh(std::vector<Vertex>& vertices,
                                std::vector<unsigned int>& indices,
                                const glm::vec4& needle_color) {
  vertices.clear();
  indices.clear();
  const std::array<glm::vec3, 6> positions = {
      glm::vec3( 1, 0, 0), glm::vec3(-1, 0, 0),
      glm::vec3( 0, 1, 0), glm::vec3( 0,-1, 0),
      glm::vec3( 0, 0, 1), glm::vec3( 0, 0,-1)};
  for (const auto& p : positions) {
    Vertex v;
    v.position = p;
    v.normal = glm::normalize(p);
    v.color = needle_color;
    v.tex_coord = glm::vec2(0.5f, 0.5f);
    vertices.push_back(v);
  }
  // 8 triangular faces.
  const std::array<glm::uvec3, 8> tris = {
      glm::uvec3(0, 2, 4), glm::uvec3(2, 1, 4), glm::uvec3(1, 3, 4), glm::uvec3(3, 0, 4),
      glm::uvec3(2, 0, 5), glm::uvec3(1, 2, 5), glm::uvec3(3, 1, 5), glm::uvec3(0, 3, 5)};
  for (const auto& t : tris) {
    indices.push_back(t.x); indices.push_back(t.y); indices.push_back(t.z);
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
  glm::vec3 base_position;        ///< World-space attachment point.
  glm::quat orientation;          ///< Rotates +Z (needle local forward) to outward direction.
  glm::vec3 base_adaxial_world;   ///< World-space adaxial reference at the base.
};

inline NeedleAnchor ComputeFascicleNeedleAnchor(const PineNode& /*cluster_node*/,
                                                 const PineNode& parent_internode_node,
                                                 float s_along_parent_norm,
                                                 float roll_offset_deg,
                                                 float branching_angle_deg,
                                                 int needle_index_in_cluster,
                                                 int needle_count_in_cluster,
                                                 float cluster_random) {
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
    if (std::abs(glm::dot(ref, parent_dir)) > 0.95f) ref = glm::vec3(1.0f, 0.0f, 0.0f);
    perp = ref - parent_dir * glm::dot(ref, parent_dir);
  }
  const float plen = glm::length(perp);
  perp = (plen > 1e-8f) ? (perp / plen) : glm::vec3(1.0f, 0.0f, 0.0f);

  // Apply per-cluster azimuthal phyllotactic roll around the parent axis so
  // sibling clusters on the same shoot fan out by ~137.5 degrees.
  const glm::quat roll_q =
      glm::angleAxis(glm::radians(roll_offset_deg), parent_dir);
  perp = glm::normalize(roll_q * perp);

  // Upward fan around the parent axis. For the common 2-needle fascicle, keep
  // both needles in a narrow V instead of placing them 180 degrees apart
  // (which frequently sends one needle downward and reads as "spaghetti").
  const float kFanHalfAngleRad = glm::radians(12.0f);
  const float kFanJitterRad = glm::radians(4.0f);
  const float fan_jitter = (cluster_random - 0.5f) * 2.0f * kFanJitterRad;
  float fan_t = 0.0f;
  if (needle_count_in_cluster > 1) {
    fan_t = static_cast<float>(needle_index_in_cluster) /
            static_cast<float>(needle_count_in_cluster - 1);
    fan_t = fan_t * 2.0f - 1.0f;
  }
  const float angle = fan_t * kFanHalfAngleRad + fan_jitter;
  const glm::quat about_axis = glm::angleAxis(angle, parent_dir);
  const glm::vec3 radial = about_axis * perp;

    // Branching angle is measured from the apical axis (parent_dir). 0 deg
    // means fully apical; increasing angle opens the fascicle toward radial.
    const float branching_angle_rad = glm::radians(std::clamp(branching_angle_deg, 0.0f, 89.5f));
  const glm::vec3 needle_dir = glm::normalize(
      std::cos(branching_angle_rad) * parent_dir +
      std::sin(branching_angle_rad) * radial);

  // Build a stable local frame instead of a shortest-arc quaternion so the
  // local x-z bending plane is locked to the stem-facing radial plane.
  // +Z = needle forward, +X = adaxial->abaxial direction (outward from stem),
  // +Y = completes a right-handed basis.
  const glm::vec3 z_axis = needle_dir;
  glm::vec3 x_axis = radial - z_axis * glm::dot(radial, z_axis);
  if (!IsFiniteVec3(x_axis) || glm::length(x_axis) <= 1e-8f) {
    glm::vec3 ref(0.0f, 1.0f, 0.0f);
    if (std::abs(glm::dot(ref, z_axis)) > 0.95f) ref = glm::vec3(1.0f, 0.0f, 0.0f);
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
  const float parent_length =
      std::max(0.0f, parent_internode_node.info.length);
  const glm::vec3 anchor_pos = parent_internode_node.info.global_position +
                               parent_dir * (s_clamped * parent_length);

  NeedleAnchor anchor;
  anchor.base_position = anchor_pos;
  anchor.orientation = orientation;
  anchor.base_adaxial_world = adaxial_world;
  return anchor;
}

/// Build a single aggregate triangle mesh containing every alive needle in
/// the tree. Vertices are emitted in world space (anchor transforms baked
/// in) so the consuming `Particles` instance uses an identity matrix.
inline void BuildPineNeedleAggregateMesh(const PineGraph& graph,
                                          const std::vector<LNodeHandle>& sorted_nodes,
                                          const ScotsPine::ColorMode color_mode,
                                          const glm::vec4& needle_young_color,
                                          const glm::vec4& needle_old_color,
                                          const float needle_axial_age_span,
                                          const float needle_axial_age_exponent,
                                          const PlottedDistribution<float>& needle_cross_section_width_profile,
                                          const PlottedDistribution<float>& needle_cross_section_thickness_profile,
                                          const PlottedDistribution<float>& needle_cross_section_temporal_maturity_curve,
                                          const int needle_fascicular_start_year,
                                          const float needle_lignification_factor_year1,
                                          const float needle_lignification_factor_year2plus,
                                          const float needle_stomatal_strip_density_year1,
                                          const float needle_stomatal_strip_density_year2plus,
                                          const float needle_basal_taper_ratio_year1,
                                          const float needle_basal_taper_ratio_year2plus,
                                          const float needle_fascicle_sheath_budget_years,
                                          const float needle_specularity_plasticity_year1,
                                          const float needle_specularity_plasticity_year2plus,
                                          std::vector<Vertex>& out_vertices,
                                          std::vector<glm::uvec3>& out_triangles,
                                          const int station_count,
                                          const int perimeter_count) {
  out_vertices.clear();
  out_triangles.clear();

  // Geometry parameters (Phase 1 defaults). station/perimeter are caller-
  // supplied so auto-grow can use a cheaper tessellation while growth is
  // advancing, then recover high quality when growth pauses.
  const int kStations = std::max(4, station_count);
  const int kPerimeter = std::max(3, perimeter_count);
  constexpr float kNeedleDefaultWidthRadiusM = 0.0007f;      // ~0.7 mm half-width.
  constexpr float kNeedleDefaultThicknessRadiusM = 0.00045f; // ~0.45 mm half-thickness.

  EllipticProfile ellipsoid_profile(/*aspect_ratio=*/1.0f, /*twist_radians=*/0.0f);

  for (const auto handle : sorted_nodes) {
    const auto& node = graph.PeekNode(handle);
    if (!node.data.template Is<PineNeedleCluster>()) continue;
    const auto& cluster = node.data.template Get<PineNeedleCluster>();
    if (!cluster.alive) continue;

    const LNodeHandle parent_handle = FindParentInternodeNodeHandle(graph, handle);
    if (parent_handle < 0) continue;
    const auto& parent_node = graph.PeekNode(parent_handle);

    const float sen = std::clamp(cluster.senescence_phase, 0.0f, 1.0f);
    const float cluster_age_years =
      std::max(0.0f, graph.data.clock.NowYears() - cluster.continuous_growth.t_init_years);
    const float cluster_lifespan_years = std::max(0.25f, static_cast<float>(cluster.lifespan_years));
    const float cluster_age_norm = std::clamp(cluster_age_years / cluster_lifespan_years, 0.0f, 1.0f);
    const float cluster_branching_angle_deg = std::clamp(cluster.branching_angle_deg, 0.0f, 89.5f);
    const float clamped_axial_exponent = std::max(0.1f, needle_axial_age_exponent);
    const float length_mult = glm::mix(1.0f, 0.82f, sen);
    const bool year2plus_cohort =
      cluster.initiation_year_index >= std::max(0, needle_fascicular_start_year);
    const float cohort_lignification_factor = year2plus_cohort
      ? std::clamp(needle_lignification_factor_year2plus, 0.0f, 2.0f)
      : std::clamp(needle_lignification_factor_year1, 0.0f, 2.0f);
    const float cohort_stomatal_strip_density = year2plus_cohort
      ? std::clamp(needle_stomatal_strip_density_year2plus, 0.0f, 1.0f)
      : std::clamp(needle_stomatal_strip_density_year1, 0.0f, 1.0f);
    const float cohort_basal_taper_ratio = year2plus_cohort
      ? std::clamp(needle_basal_taper_ratio_year2plus, 0.6f, 1.2f)
      : std::clamp(needle_basal_taper_ratio_year1, 0.6f, 1.2f);
    const float cohort_specularity_plasticity = year2plus_cohort
      ? std::clamp(needle_specularity_plasticity_year2plus, 0.0f, 1.0f)
      : std::clamp(needle_specularity_plasticity_year1, 0.0f, 1.0f);
    const float sheath_budget_years = std::max(0.0f, needle_fascicle_sheath_budget_years);

    // Phase 6 seedling realism pass: drive rendered needle thickness from
    // material profile radii when available, with Phase 1 taper as a fallback.
    const float t_now_years = graph.data.clock.NowYears();
    const float maturation_multiplier =
        (cluster.continuous_growth.maturation_years > 0.0f)
            ? cluster.continuous_growth.Multiplier(t_now_years)
            : 1.0f;

    // Phase 4: world-frame gravity vector (-Y world-up convention).
    const float gravity_mag = graph.data.gravity_m_s2;
    const glm::vec3 gravity_world(0.0f, -gravity_mag, 0.0f);

    for (int n = 0; n < std::max(1, cluster.count); ++n) {
      const PineNeedleInstanceProfile* needle_profile =
        (static_cast<size_t>(n) < cluster.per_needle_profiles.size())
          ? &cluster.per_needle_profiles[static_cast<size_t>(n)]
          : nullptr;
      const BilateralGrowthField1D& needle_growth_field =
        needle_profile ? needle_profile->growth_field : cluster.growth_field;
      const MaterialProfile1D& needle_material_profile =
        needle_profile ? needle_profile->material_profile : cluster.material_profile;
      MaterialProfile1D matured_material_profile = needle_material_profile;
      const float needle_length_scale = needle_profile
        ? std::clamp(needle_profile->length_scale, 0.05f, 3.0f)
        : 1.0f;
      const float needle_radius_scale = needle_profile
        ? std::clamp(needle_profile->radius_scale, 0.05f, 3.0f)
        : 1.0f;
      const float needle_wave_amplitude_deg = needle_profile
        ? std::clamp(needle_profile->sinusoidal_amplitude_deg, 0.0f, 45.0f)
        : std::clamp(cluster.sinusoidal_amplitude_deg, 0.0f, 45.0f);
      const float needle_wave_frequency_cycles = needle_profile
        ? std::clamp(needle_profile->sinusoidal_frequency_cycles, 0.0f, 12.0f)
        : std::clamp(cluster.sinusoidal_frequency_cycles, 0.0f, 12.0f);
      const float needle_wave_phase_rad = needle_profile
        ? needle_profile->sinusoidal_phase_rad
        : cluster.sinusoidal_phase_rad;
      // Use chronological age so fascicle opening continues through dormant season.
      const float needle_relax_years = needle_profile
        ? std::max(0.0f, needle_profile->branching_relax_years)
        : std::max(0.0f, cluster.branching_relax_years);
      const float needle_relax_progress = (needle_relax_years <= 1e-5f)
        ? 1.0f
        : std::clamp(cluster_age_years / needle_relax_years, 0.0f, 1.0f);
      const float active_branching_angle_deg =
        cluster_branching_angle_deg * needle_relax_progress;

      const bool has_profile_radii =
        std::isfinite(matured_material_profile.base_radius_m) &&
        std::isfinite(matured_material_profile.tip_radius_m) &&
        (matured_material_profile.base_radius_m > 0.0f ||
         matured_material_profile.tip_radius_m > 0.0f);
      const float fallback_width_radius_m = has_profile_radii
        ? std::max(matured_material_profile.base_radius_m, 0.0f)
        : kNeedleDefaultWidthRadiusM;
      const float fallback_thickness_radius_m = has_profile_radii
        ? std::max(matured_material_profile.tip_radius_m, 0.0f)
        : kNeedleDefaultThicknessRadiusM;
      const float raw_cluster_width_radius_m = (cluster.cross_section_width_radius_m > 0.0f)
        ? cluster.cross_section_width_radius_m
        : fallback_width_radius_m;
      const float raw_cluster_thickness_radius_m = (cluster.cross_section_thickness_radius_m > 0.0f)
        ? cluster.cross_section_thickness_radius_m
        : fallback_thickness_radius_m;
        const float clamped_cluster_width_radius_m =
          std::max(raw_cluster_width_radius_m, 0.00002f);
        const float clamped_cluster_thickness_radius_m =
          std::max(raw_cluster_thickness_radius_m, 0.00002f);
      std::vector<float> width_radius_table;
      std::vector<float> thickness_radius_table;
      width_radius_table.reserve(static_cast<size_t>(kStations));
      thickness_radius_table.reserve(static_cast<size_t>(kStations));
      const float radius_vigor_scale = std::clamp(
        cluster.render_radius_scale * needle_radius_scale, 0.10f, 4.0f);
      const uint32_t node_hash = static_cast<uint32_t>(node.GetIndex());
      const uint32_t needle_hash = static_cast<uint32_t>(n);
      const uint32_t width_seed = node_hash ^ (needle_hash * 0x9e3779b9u) ^ 0x2d9c8f13u;
      const uint32_t thickness_seed = node_hash ^ (needle_hash * 0x85ebca6bu) ^ 0xa5b35705u;
      const uint32_t temporal_seed = node_hash ^ (needle_hash * 0xc2b2ae35u) ^ 0x4f1bbcdcu;
      const float temporal_cross_section_multiplier =
        EvaluateTemporalCrossSectionMultiplier(
            needle_cross_section_temporal_maturity_curve,
            cluster_age_years,
            cluster.node_random,
            temporal_seed);
      for (int i = 0; i < kStations; ++i) {
      const float s_norm = static_cast<float>(i) / static_cast<float>(kStations - 1);
      const float width_profile_multiplier = EvaluatePositionalMultiplier(
          needle_cross_section_width_profile, s_norm, cluster.node_random,
          width_seed ^ static_cast<uint32_t>(i));
      const float thickness_profile_multiplier = EvaluatePositionalMultiplier(
          needle_cross_section_thickness_profile, s_norm, cluster.node_random,
          thickness_seed ^ static_cast<uint32_t>(i));
      const float raw_width_radius =
          clamped_cluster_width_radius_m * width_profile_multiplier * temporal_cross_section_multiplier;
      const float raw_thickness_radius =
          clamped_cluster_thickness_radius_m * thickness_profile_multiplier * temporal_cross_section_multiplier;
      const float scaled_width_radius = raw_width_radius * radius_vigor_scale;
      const float scaled_thickness_radius = raw_thickness_radius * radius_vigor_scale;
      const float safe_width_radius =
          std::max(scaled_width_radius, 0.00002f);
      const float safe_thickness_radius =
          std::max(scaled_thickness_radius, 0.00002f);
      const float basal_taper_multiplier =
        glm::mix(cohort_basal_taper_ratio, 1.0f, s_norm);
      const float tapered_width_radius = std::max(
        safe_width_radius * basal_taper_multiplier, 0.00002f);
      const float tapered_thickness_radius = std::max(
        safe_thickness_radius * basal_taper_multiplier, 0.00002f);
      width_radius_table.push_back(tapered_width_radius);
      thickness_radius_table.push_back(tapered_thickness_radius);
      }
      const float base_radius_m =
        width_radius_table.empty() ? kNeedleDefaultWidthRadiusM
                   : width_radius_table.front();
      const float secondary_base_radius_m =
        thickness_radius_table.empty() ? kNeedleDefaultThicknessRadiusM
                   : std::max(thickness_radius_table.front(), 0.00002f);

      // Phase 3: bent centerline driven by a per-needle bilateral growth
      // field, ramped by the cluster's continuous-growth multiplier.
      const float length = std::max(
        0.001f, cluster.length * length_mult * needle_length_scale);
      OrganCenterline intrinsic_centerline = BuildBentNeedleCenterline(
        length, /*segments=*/kStations - 1,
        needle_growth_field, maturation_multiplier,
        needle_wave_amplitude_deg, needle_wave_frequency_cycles,
        needle_wave_phase_rad);

      const bool mechanics_active =
        matured_material_profile.IsActive() && gravity_mag > 0.0f;
      const NeedleAnchor anchor = ComputeFascicleNeedleAnchor(
          node, parent_node, cluster.s_along_parent_norm, cluster.roll_offset_deg,
          active_branching_angle_deg,
          n, cluster.count, cluster.node_random);

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
        // Use only the in-plane (x, z) components — the cross-plane Y
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
          cps[static_cast<size_t>(i)] =
              glm::vec3(eso.positions_xz[i].x, 0.0f, eso.positions_xz[i].y);
        }
        deflected.Invalidate();
        centerline = std::move(deflected);
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

      const float stripe_strength =
        cohort_stomatal_strip_density * cohort_specularity_plasticity * 0.08f;
      for (int i = 0; i < kStations; ++i) {
        const float s_norm = static_cast<float>(i) / static_cast<float>(kStations - 1);
        const float axial = std::pow(s_norm, clamped_axial_exponent);
        const float axial_shift = (axial - 0.5f) * 2.0f;
        const float segment_age_norm =
            std::clamp(cluster_age_norm + needle_axial_age_span * axial_shift, 0.0f, 1.0f);
        float segment_oldness = std::clamp(
          std::max(segment_age_norm, sen) * cohort_lignification_factor, 0.0f, 1.0f);
        float sheath_visibility = 0.0f;
        if (year2plus_cohort && sheath_budget_years > 0.0f) {
          const float sheath_progress =
            std::clamp(cluster_age_years / sheath_budget_years, 0.0f, 1.0f);
          const float sheath_mask = std::clamp(1.0f - s_norm * 3.5f, 0.0f, 1.0f);
          sheath_visibility = std::clamp(sheath_progress * sheath_mask, 0.0f, 1.0f);
          segment_oldness = std::clamp(
            std::max(segment_oldness, sheath_visibility), 0.0f, 1.0f);
        }
        const float stripe_phase =
          (s_norm * 48.0f + cluster.node_random * 17.0f) * glm::two_pi<float>();
        const float stripe_wave = 0.5f + 0.5f * std::sin(stripe_phase);
        const float stripe_scale = 1.0f - stripe_strength + stripe_strength * stripe_wave;
        const float stripe_proxy = std::clamp(
          cohort_stomatal_strip_density * cohort_specularity_plasticity * stripe_wave,
          0.0f, 1.0f);

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
      SweepGeneralizedCylinder<Vertex>(centerline, ellipsoid_profile, cfg,
                                       out_vertices, out_triangles);
      const std::size_t appended_vertices = out_vertices.size() - vertex_start;
      const std::size_t expected_vertices =
        static_cast<std::size_t>(kStations) * static_cast<std::size_t>(kPerimeter);
      const std::size_t metadata_vertices = std::min(appended_vertices, expected_vertices);
      // Pack station signals for future shader-side parity checks and overlays.
      for (std::size_t local_index = 0; local_index < metadata_vertices; ++local_index) {
        const std::size_t station_index = std::min(
          static_cast<std::size_t>(kStations - 1),
          local_index / static_cast<std::size_t>(kPerimeter));
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

void ScotsPine::SetGlobalColorMode(const ColorMode mode) { g_scots_pine_color_mode = mode; }
ScotsPine::ColorMode ScotsPine::GetGlobalColorMode() { return g_scots_pine_color_mode; }
void ScotsPine::SetForceCpuParticlesPath(const bool force) {
  g_force_cpu_particles_path.store(force, std::memory_order_relaxed);
}
bool ScotsPine::IsForceCpuParticlesPath() {
  return g_force_cpu_particles_path.load(std::memory_order_relaxed);
}

namespace {
std::shared_ptr<ScotsPineDescriptor> ResolvePostRepotDescriptorForGrowth(
    const ScotsPine& pine) {
  if (!pine.enable_repot_profile_switch) {
    return nullptr;
  }
  return pine.post_repot_descriptor_ref.Get<ScotsPineDescriptor>();
}

float ResolveRepotSwitchGddForGrowth(const ScotsPine& pine,
                                     const std::shared_ptr<ScotsPineDescriptor>&
                                         post_descriptor) {
  if (!post_descriptor) {
    return -1.0f;
  }
  return std::max(0.0f, pine.repot_switch_gdd);
}
}  // namespace

// ===========================================================================

void ScotsPine::ClearGeometryEntities() const {
  const auto scene = GetScene();
  const auto self = GetOwner();
  const auto children = scene->GetChildren(self);
  for (const auto& child : children) {
    const auto name = scene->GetEntityName(child);
    if (name == "Pine Internodes" || name == "Pine Needles" ||
        name == "Pine Needles Geometry") {
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

  const double grow_start = Times::Now();
  auto descriptor = descriptor_ref.Get<ScotsPineDescriptor>();
  if (!descriptor) {
    last_grow_seconds = 0.0;
    return;
  }
  const auto post_descriptor = ResolvePostRepotDescriptorForGrowth(*this);
  growth_model.Initialize(*descriptor, seed, glm::vec3(0), kDefaultRootRotation,
                          post_descriptor.get(),
                          ResolveRepotSwitchGddForGrowth(*this, post_descriptor));
  const float clamped_target_gdd = std::min(target_gdd, std::max(0.0f, preview_target_gdd));
  const uint32_t step_cap = std::max(1u, preview_max_growth_steps);
  growth_model.GrowToGDDWithProfileSwitch(clamped_target_gdd, step_cap);

  last_grow_seconds = Times::Now() - grow_start;
  RebuildGeometry();
}

void ScotsPine::GrowToTargetGDD(const bool uncapped_growth) {
  const double grow_start = Times::Now();
  auto descriptor = descriptor_ref.Get<ScotsPineDescriptor>();
  if (!descriptor) {
    last_grow_seconds = 0.0;
    return;
  }
  const auto post_descriptor = ResolvePostRepotDescriptorForGrowth(*this);
  const float repot_switch_gdd_value =
      ResolveRepotSwitchGddForGrowth(*this, post_descriptor);
  if (!growth_model.IsInitialized()) {
    growth_model.Initialize(*descriptor, seed, glm::vec3(0), kDefaultRootRotation,
                            post_descriptor.get(), repot_switch_gdd_value);
  }
  bool reinitialized = false;
  // Backward-scrubbing: if the user has dragged target_gdd backward beyond a step,
  // re-init from scratch so geometry shrinks instead of being stuck at the high-water mark.
  const float gdd_step = std::max(1e-5f, growth_model.gdd_per_growth_step);
  if (target_gdd + gdd_step < growth_model.accumulated_gdd) {
    growth_model.Initialize(*descriptor, seed, glm::vec3(0), kDefaultRootRotation,
                            post_descriptor.get(), repot_switch_gdd_value);
    reinitialized = true;
  }
  growth_model.GrowToGDDWithProfileSwitch(target_gdd,
                                          uncapped_growth ? 0u : max_growth_steps_per_frame);
  last_grow_seconds = Times::Now() - grow_start;

  // Auto-grow calls this every frame; when no growth step was taken, a full
  // mesh rebuild is wasted work. Preserve exact behavior on explicit
  // reinitialization (backward scrub), where geometry must always be refreshed.
  if (!reinitialized && growth_model.last_growth_steps == 0) {
    return;
  }
  RebuildGeometry();
}

void ScotsPine::SetSeasonalChronologicalMode(
    const bool enable_independent_chronological_clock) {
  growth_model.SetChronologicalCoupledToThermal(
      !enable_independent_chronological_clock);
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
    growth_model.Initialize(*descriptor, seed, glm::vec3(0), kDefaultRootRotation,
                            post_descriptor.get(),
                            ResolveRepotSwitchGddForGrowth(*this, post_descriptor));
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
  const double rebuild_start = Times::Now();
  last_invalid_instance_count = 0;
  last_internode_count = 0;
  last_needle_count = 0;
  last_node_count = 0;

  if (!growth_model.IsInitialized()) {
    last_rebuild_seconds = 0.0;
    return;
  }

  const auto scene = GetScene();
  const auto owner = GetOwner();
  const auto color_mode = GetGlobalColorMode();
  const glm::vec4 instance_color = HashToColor(owner.GetIndex());
  const glm::vec4 kDefaultNeedleColor(0.16f, 0.45f, 0.18f, 1.0f);
  const glm::vec4 kDefaultNeedleOldColor(0.42f, 0.27f, 0.10f, 1.0f);
  const glm::vec4 kDefaultStemColor(0.60f, 0.78f, 0.50f, 1.0f);
  const glm::vec4 kDefaultStemOldColor(0.45f, 0.34f, 0.22f, 1.0f);
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

  const float needle_color_energy =
      std::max(needle_base_color.r, std::max(needle_base_color.g, needle_base_color.b));
  const float needle_old_color_energy =
      std::max(needle_old_color.r, std::max(needle_old_color.g, needle_old_color.b));
  if (needle_color_energy <= 1.0e-4f && needle_old_color_energy <= 1.0e-4f) {
    // Guard against corrupted descriptor color fields that zero out all needles.
    needle_base_color = kDefaultNeedleColor;
    needle_old_color = kDefaultNeedleOldColor;
  }
  const float t_now_years = growth_model.graph.data.clock.NowYears();
  // Stem-age color denominator: use needle lifespan as a coarse browning
  // horizon. Phytomer model has no descriptor-level "max age" anymore.
  const float max_internode_age_years =
      std::max(1.0f, static_cast<float>(growth_model.sampled.needle_lifespan_years));

  static thread_local std::vector<ParticleInfo> internode_infos_cache;
  static thread_local std::vector<ParticleInfo> needle_infos_cache;

  // -- Find existing geometry child entities (reuse for stable transforms) --
  // TEMPORARY WORKAROUND for the stem/needles cross-render bug:
  // Stem and needles are placed under SEPARATE container entities (siblings
  // under `owner`) so they never share an immediate parent. Empirically,
  // when both Particles components were direct children of `owner`, enabling
  // the stem caused the needles to render black (and vice versa after Ctrl+W).
  // The two-container split decouples whatever per-parent state was
  // pollinating across the two renderables.
  Entity stem_container, needles_container;
  for (const auto& child : scene->GetChildren(owner)) {
    const auto name = scene->GetEntityName(child);
    if (name == "Pine Stem Container")
      stem_container = child;
    else if (name == "Pine Needles Container")
      needles_container = child;
  }
  if (!scene->IsEntityValid(stem_container)) {
    stem_container = scene->CreateEntity("Pine Stem Container");
    scene->SetParent(stem_container, owner);
  }
  if (!scene->IsEntityValid(needles_container)) {
    needles_container = scene->CreateEntity("Pine Needles Container");
    scene->SetParent(needles_container, owner);
  }
  Entity internode_entity, needle_entity, needle_geom_entity;
  for (const auto& child : scene->GetChildren(stem_container)) {
    const auto name = scene->GetEntityName(child);
    if (name == "Pine Internodes")
      internode_entity = child;
  }
  for (const auto& child : scene->GetChildren(needles_container)) {
    const auto name = scene->GetEntityName(child);
    if (name == "Pine Needles")
      needle_entity = child;
    else if (name == "Pine Needles Geometry")
      needle_geom_entity = child;
  }
  // Migration: if a previous build placed these directly under `owner`,
  // delete the orphans so a fresh pair is created in the new containers.
  for (const auto& child : scene->GetChildren(owner)) {
    const auto name = scene->GetEntityName(child);
    if (name == "Pine Internodes" || name == "Pine Needles" ||
        name == "Pine Needles Geometry") {
      scene->DeleteEntity(child);
    }
  }

  const auto& sorted = growth_model.graph.PeekSortedNodeList();
  last_node_count = static_cast<uint32_t>(sorted.size());

  // -- Internodes (unit cylinders authored along +Y; rotate +Y -> -Z) --
  {
    auto& infos = internode_infos_cache;
    infos.clear();
    infos.reserve(sorted.size());
    const glm::quat cylinder_axis_fix =
        glm::angleAxis(-glm::half_pi<float>(), glm::vec3(1.0f, 0.0f, 0.0f));

    for (const auto handle : sorted) {
      const auto& node = growth_model.graph.PeekNode(handle);
      if (!node.data.template Is<PineInternode>()) continue;
      const auto& internode = node.data.template Get<PineInternode>();
      if (node.info.length <= 0.0f) continue;
      if (!IsFiniteVec3(node.info.global_position) || !std::isfinite(node.info.length) ||
          !std::isfinite(node.info.thickness)) {
        last_invalid_instance_count++;
        continue;
      }
      const float half_thick = node.info.thickness * 0.5f;
      if (half_thick <= 0.0f) continue;
        const float internode_age_years =
          std::max(0.0f, t_now_years - internode.continuous_growth.t_init_years);
        float internode_age_norm = std::clamp(internode_age_years / max_internode_age_years, 0.0f, 1.0f);
        internode_age_norm = std::pow(internode_age_norm, std::max(0.1f, internode_age_exponent));
        const glm::vec4 internode_age_color =
          glm::mix(stem_base_color, stem_old_color, internode_age_norm);

      glm::quat instance_rotation =
          glm::normalize(node.info.global_rotation * cylinder_axis_fix);
      if (!IsFiniteQuat(instance_rotation)) {
        instance_rotation = glm::quat(1, 0, 0, 0);
        last_invalid_instance_count++;
      }
      ParticleInfo pi;
      const glm::mat4 model = glm::translate(node.info.global_position) *
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
        pi.instance_color = internode_age_color;
      } else {
        pi.instance_color = internode_age_color;
      }
      infos.push_back(pi);
    }
    last_internode_count = static_cast<uint32_t>(infos.size());

    if (!infos.empty()) {
      std::shared_ptr<Particles> particles;
      std::shared_ptr<ParticleInfoList> particle_info_list;

      if (!scene->IsEntityValid(internode_entity)) {
        internode_entity = scene->CreateEntity("Pine Internodes");
        particles = scene->GetOrSetPrivateComponent<Particles>(internode_entity).lock();

        const auto mesh = AssetManager::CreateTemporaryAsset<Mesh>();
        std::vector<Vertex> cyl_verts;
        std::vector<unsigned int> cyl_idx;
        // Internode material uses vertex_color_only=true with the
        // "vertex.color * instance.color" tint policy. The static cylinder
        // mesh therefore needs a NEUTRAL vertex color so per-instance
        // ColorMode tints (ByNode/ByInstance/ByType/age-shading) dominate.
        GenerateUnitCylinderMesh(cyl_verts, cyl_idx, glm::vec4(1.0f));
        VertexAttributes attrs{};
        attrs.normal = true;
        attrs.color = true;
        attrs.tex_coord = true;
        mesh->SetVertices(attrs, cyl_verts, cyl_idx);

        const auto material = AssetManager::CreateTemporaryAsset<Material>();
        // Internode mesh has uniform vertex.color (1,1,1); per-instance
        // ColorMode tint is delivered through ParticleInfo::instance_color.
        // The legacy `has_instance_tint` path in StandardDeferredInstanced.frag
        // multiplies albedo by instanceColor, which is exactly what we want.
        // Keep vertex_color_only=false so that path is taken even if a saved
        // scene's mesh asset still carries old baked vertex colors.
        material->vertex_color_only = false;

        // Wire the freshly created assets onto Particles. Without these
        // assignments, particle_info_list would remain a null shared_ptr and
        // the SetParticleInfos() call below would deref null (manifesting as
        // an assertion deep in GeometryStorage::UpdateParticleInfo because
        // the default RangeDescriptor is unregistered).
        particle_info_list = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
        particles->mesh = mesh;
        particles->material = material;
        particles->particle_info_list = particle_info_list;
        scene->SetParent(internode_entity, stem_container);
      } else {
        particles = scene->GetOrSetPrivateComponent<Particles>(internode_entity).lock();
        particle_info_list = particles->particle_info_list.Get<ParticleInfoList>();
        if (!particle_info_list) {
          particle_info_list = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
          particles->particle_info_list = particle_info_list;
        }
      }

      if (const auto internode_material = particles->material.Get<Material>()) {
        // See note in the creation block above: stem cylinder uses
        // ParticleInfo instance_color exclusively, not per-vertex tint.
        internode_material->vertex_color_only = false;
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
      }

      particle_info_list->SetParticleInfos(infos);
    } else if (scene->IsEntityValid(internode_entity)) {
      scene->DeleteEntity(internode_entity);
    }
  }

  // -- Needles --
  // Phase 1: route needle geometry through the new generalized-cylinder path
  // (one aggregate mesh containing every needle, anchored at its parent
  // internode, swept along a SemiCircularProfile). Legacy octahedron-marker
  // path retained behind `g_use_generalized_cylinder_needles` for visual A/B.
  const bool use_new_needle_geometry =
      g_use_generalized_cylinder_needles.load(std::memory_order_relaxed);

  if (use_new_needle_geometry) {
    // Aggregate-mesh path. World-space vertices baked in; instance is identity.
    static thread_local std::vector<Vertex> needle_geom_vertices;
    static thread_local std::vector<glm::uvec3> needle_geom_triangles;
    static thread_local std::vector<unsigned int> needle_geom_flat_indices;
    glm::vec4 needle_instance_tint(1.0f, 1.0f, 1.0f, 0.0f);
    if (color_mode == ColorMode::ByInstance) {
      needle_instance_tint = instance_color;
      needle_instance_tint.a = 1.0f;
    }
    // In aggregate mode, all non-ByInstance modes are vertex-colored
    // (including pine-only diagnostic overlays) while ByInstance stays
    // instance-tinted for consistency with the debug palette.

    // Avoid LOD topology switching once senescence/abscission starts.
    // Shrinking/vanishing needle cohorts already change geometry; keeping
    // dynamic tessellation fixed past this point avoids additional visual
    // instability.
    const bool senescence_active =
      HasNeedleSenescenceOrAbscission(growth_model.graph, sorted);
    const bool growth_active_lod =
      growth_model.last_growth_steps > 0 && !senescence_active;
    const int needle_station_count =
      std::max(4, growth_model.sampled.needle_segment_count + 1);
    const int needle_perimeter_count = growth_active_lod ? 5 : 8;

    BuildPineNeedleAggregateMesh(growth_model.graph,
             sorted,
             color_mode,
             needle_base_color,
             needle_old_color,
             needle_axial_age_span,
             needle_axial_age_exponent,
         growth_model.sampled.distributions.needle_cross_section_width_profile,
         growth_model.sampled.distributions.needle_cross_section_thickness_profile,
           growth_model.sampled.distributions.needle_cross_section_temporal_maturity_curve,
          growth_model.sampled.needle_fascicular_start_year,
          growth_model.sampled.needle_lignification_factor_year1,
          growth_model.sampled.needle_lignification_factor_year2plus,
          growth_model.sampled.needle_stomatal_strip_density_year1,
          growth_model.sampled.needle_stomatal_strip_density_year2plus,
          growth_model.sampled.needle_basal_taper_ratio_year1,
          growth_model.sampled.needle_basal_taper_ratio_year2plus,
          growth_model.sampled.needle_fascicle_sheath_budget_years,
          growth_model.sampled.needle_specularity_plasticity_year1,
          growth_model.sampled.needle_specularity_plasticity_year2plus,
                   needle_geom_vertices, needle_geom_triangles,
                   needle_station_count, needle_perimeter_count);

    // Count needles for telemetry: vertices / (kStations * kPerimeter).
    last_needle_count = needle_geom_triangles.empty() ? 0u :
      static_cast<uint32_t>(needle_geom_vertices.size() /
                  static_cast<size_t>(needle_station_count * needle_perimeter_count));

    const bool needle_mesh_valid =
        IsNeedleAggregateMeshValid(needle_geom_vertices, needle_geom_triangles);
    if (!needle_mesh_valid) {
      // Keep the previously uploaded mesh to avoid rendering corruption from
      // transient invalid vertices/indices.
      last_invalid_instance_count++;
    }

    // Tear down legacy octahedron entity if it still exists from a prior toggle.
    if (scene->IsEntityValid(needle_entity)) scene->DeleteEntity(needle_entity);

    if (needle_mesh_valid && !needle_geom_triangles.empty()) {
      std::shared_ptr<Particles> particles;
      std::shared_ptr<ParticleInfoList> particle_info_list;
      std::shared_ptr<Mesh> mesh;
      ParticleInfo identity;
      identity.instance_matrix.value = glm::mat4(1.0f);
      // Alpha < 0.5 selects vertex-color tint path in StandardDeferredInstanced.
      identity.instance_color = needle_instance_tint;

      if (!scene->IsEntityValid(needle_geom_entity)) {
        needle_geom_entity = scene->CreateEntity("Pine Needles Geometry");
        particles = scene->GetOrSetPrivateComponent<Particles>(needle_geom_entity).lock();

        mesh = AssetManager::CreateTemporaryAsset<Mesh>();

        const auto material = AssetManager::CreateTemporaryAsset<Material>();
        material->vertex_color_only = true;
        material->material_properties.albedo_color = glm::vec3(1.0f);
        material->material_properties.metallic = 0.0f;
        material->material_properties.roughness = 1.0f;

        particle_info_list = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
        particle_info_list->SetParticleInfos({identity});

        particles->mesh = mesh;
        particles->material = material;
        particles->particle_info_list = particle_info_list;
        scene->SetParent(needle_geom_entity, needles_container);
      } else {
        particles = scene->GetOrSetPrivateComponent<Particles>(needle_geom_entity).lock();
        mesh = particles->mesh.Get<Mesh>();
        if (!mesh) {
          mesh = AssetManager::CreateTemporaryAsset<Mesh>();
          particles->mesh = mesh;
        }
        particle_info_list = particles->particle_info_list.Get<ParticleInfoList>();
        if (!particle_info_list) {
          particle_info_list = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
          particle_info_list->SetParticleInfos({identity});
          particles->particle_info_list = particle_info_list;
        } else {
          particle_info_list->SetParticleInfos({identity});
        }
      }

      if (const auto needle_material = particles->material.Get<Material>()) {
        const float needle_specularity = std::clamp(
          0.5f * (growth_model.sampled.needle_specularity_plasticity_year1 +
                  growth_model.sampled.needle_specularity_plasticity_year2plus),
          0.0f, 1.0f);
        const float stomatal_density = std::clamp(
          0.5f * (growth_model.sampled.needle_stomatal_strip_density_year1 +
                  growth_model.sampled.needle_stomatal_strip_density_year2plus),
          0.0f, 1.0f);
        const float needle_lignification = std::clamp(
          0.5f * (growth_model.sampled.needle_lignification_factor_year1 +
                  growth_model.sampled.needle_lignification_factor_year2plus),
          0.0f, 2.0f);

        needle_material->vertex_color_only = true;
        needle_material->SetAlbedoTexture(nullptr);
        needle_material->material_properties.albedo_color = glm::vec3(1.0f);
        needle_material->draw_settings.blending = false;
        needle_material->material_properties.metallic = 0.0f;
        needle_material->material_properties.specular =
          std::clamp(0.08f + 0.35f * needle_specularity, 0.05f, 0.50f);
        needle_material->material_properties.specular_tint =
          std::clamp(0.10f + 0.25f * stomatal_density, 0.0f, 0.50f);
        needle_material->material_properties.roughness =
          std::clamp(0.88f - 0.42f * needle_specularity, 0.35f, 0.95f);
        needle_material->material_properties.subsurface_factor =
          std::clamp(0.05f + 0.20f * (1.0f - 0.5f * needle_lignification), 0.0f, 0.45f);
        needle_material->material_properties.ior = 1.36f;
        needle_material->material_properties.transmission =
          std::clamp(0.18f + 0.35f * stomatal_density, 0.0f, 0.75f);
        needle_material->material_properties.transmission_roughness =
          std::clamp(0.45f + 0.35f * needle_lignification, 0.0f, 1.0f);
        needle_material->material_properties.clear_coat =
          std::clamp(0.05f + 0.20f * needle_specularity, 0.0f, 0.35f);
        needle_material->material_properties.clear_coat_roughness =
          std::clamp(0.70f - 0.45f * needle_specularity, 0.05f, 1.0f);
      }

      VertexAttributes attrs{};
      attrs.normal = true;
      attrs.tangent = true;
      attrs.color = true;
      attrs.tex_coord = true;
      // Mesh::SetVertices takes flat unsigned int indices. Pack uvec3 -> uint.
      needle_geom_flat_indices.clear();
      needle_geom_flat_indices.reserve(needle_geom_triangles.size() * 3);
      for (const auto& t : needle_geom_triangles) {
        needle_geom_flat_indices.push_back(t.x);
        needle_geom_flat_indices.push_back(t.y);
        needle_geom_flat_indices.push_back(t.z);
      }
      mesh->SetVertices(attrs, needle_geom_vertices, needle_geom_flat_indices);
    } else if (needle_mesh_valid && scene->IsEntityValid(needle_geom_entity)) {
      scene->DeleteEntity(needle_geom_entity);
    }
  } else {
    // [deprecated] Phase 1 baseline: per-cluster octahedron marker. Kept so an
    // operator can A/B the new geometry; once Phase 6 ships the elastica
    // solver this entire block is slated for removal.
    if (scene->IsEntityValid(needle_geom_entity)) scene->DeleteEntity(needle_geom_entity);

    auto& infos = needle_infos_cache;
    infos.clear();

    for (const auto handle : sorted) {
      const auto& node = growth_model.graph.PeekNode(handle);
      if (!node.data.template Is<PineNeedleCluster>()) continue;
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
      } else if (color_mode == ColorMode::ByType) {
        pi.instance_color = needle_base_color;
      } else {
        pi.instance_color = needle_base_color;
      }
      infos.push_back(pi);
    }
    last_needle_count = static_cast<uint32_t>(infos.size());

    if (!infos.empty()) {
      std::shared_ptr<Particles> particles;
      std::shared_ptr<ParticleInfoList> particle_info_list;

      if (!scene->IsEntityValid(needle_entity)) {
        needle_entity = scene->CreateEntity("Pine Needles");
        particles = scene->GetOrSetPrivateComponent<Particles>(needle_entity).lock();

        const auto mesh = AssetManager::CreateTemporaryAsset<Mesh>();
        std::vector<Vertex> oct_verts;
        std::vector<unsigned int> oct_idx;
        // Same rationale as the internode cylinder: keep vertex color
        // neutral so per-instance mode tint (set via ParticleInfo) wins
        // when the material uses vertex_color_only with multiply policy.
        GenerateUnitOctahedronMesh(oct_verts, oct_idx, glm::vec4(1.0f));
        VertexAttributes attrs{};
        attrs.normal = true;
        attrs.color = true;
        attrs.tex_coord = true;
        mesh->SetVertices(attrs, oct_verts, oct_idx);

        const auto material = AssetManager::CreateTemporaryAsset<Material>();
        material->vertex_color_only = true;
        material->material_properties.albedo_color = glm::vec3(1.0f);
        material->material_properties.metallic = 0.0f;
        material->material_properties.roughness = 1.0f;

        particle_info_list = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
        particles->mesh = mesh;
        particles->material = material;
        particles->particle_info_list = particle_info_list;
        scene->SetParent(needle_entity, needles_container);
      } else {
        particles = scene->GetOrSetPrivateComponent<Particles>(needle_entity).lock();
        particle_info_list = particles->particle_info_list.Get<ParticleInfoList>();
        if (!particle_info_list) {
          particle_info_list = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
          particles->particle_info_list = particle_info_list;
        }
      }
      if (const auto needle_material = particles->material.Get<Material>()) {
        needle_material->vertex_color_only = false;
        needle_material->SetAlbedoTexture(nullptr);
        needle_material->material_properties.albedo_color = glm::vec3(1.0f);
        needle_material->draw_settings.blending = false;
      }
      particle_info_list->SetParticleInfos(infos);
    } else if (scene->IsEntityValid(needle_entity)) {
      scene->DeleteEntity(needle_entity);
    }
  }

  last_rebuild_seconds = Times::Now() - rebuild_start;
}

// ===========================================================================
// Export
// ===========================================================================

void ScotsPine::ExportObj(const std::filesystem::path& path) const {
  const auto scene = GetScene();
  if (!scene) return;

  std::vector<Vertex> vertices;
  std::vector<glm::uvec3> triangles;

  const auto owner = GetOwner();
  for (const auto& child : scene->GetChildren(owner)) {
    const auto name = scene->GetEntityName(child);
    if (name == "Pine Internodes" || name == "Pine Needles" ||
        name == "Pine Needles Geometry") {
      AppendParticlesToMesh(scene, child, vertices, triangles);
    }
  }

  if (vertices.empty() || triangles.empty()) {
    EVOENGINE_ERROR("Pine mesh export failed: no pine particle geometry available.");
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
    if (retained.find(flow_handle) == retained.end()) continue;
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
    if (!IsInternodeNode(node)) continue;
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

// ===========================================================================
// Component lifecycle / inspector
// ===========================================================================

void ScotsPine::OnDestroy() {
  ClearGeometryEntities();
}

bool ScotsPine::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;

  if (editor_layer->DragAndDropButton<ScotsPineDescriptor>(descriptor_ref, "Descriptor"))
    changed = true;

  if (editor_layer->DragAndDropButton<ScotsPineDescriptor>(
          post_repot_descriptor_ref, "Post-Repot Descriptor")) {
    changed = true;
  }

  if (ImGui::Checkbox("Enable Repot Profile Switch", &enable_repot_profile_switch)) {
    changed = true;
  }
  if (enable_repot_profile_switch) {
    if (ImGui::DragFloat("Repot Switch GDD", &repot_switch_gdd,
                         10.0f, 0.0f, 200000.0f, "%.1f")) {
      repot_switch_gdd = std::max(0.0f, repot_switch_gdd);
      changed = true;
    }
  }

  int seed_int = static_cast<int>(seed);
  if (ImGui::DragInt("Seed", &seed_int, 1, 0, 999999)) {
    seed = static_cast<unsigned int>(seed_int);
    changed = true;
  }

  if (ImGui::DragFloat("Target GDD", &target_gdd, 1.0f, 0.0f, 200000.0f, "%.1f"))
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

  if (growth_model.IsInitialized()) {
    ImGui::Separator();
    ImGui::Text("GDD: %.1f", growth_model.accumulated_gdd);
    ImGui::Text("Topology: %s", growth_model.IsTopologyComplete() ? "Complete" : "Pending");
    const auto& sorted = growth_model.graph.PeekSortedNodeList();
    ImGui::Text("Nodes: %d", static_cast<int>(sorted.size()));
    ImGui::Text("Internodes: %u   Needles: %u", last_internode_count, last_needle_count);
  }

  return changed;
}

void ScotsPine::Serialize(YAML::Emitter& out) const {
  descriptor_ref.Save("descriptor_ref", out);
  post_repot_descriptor_ref.Save("post_repot_descriptor_ref", out);
  out << YAML::Key << "seed" << YAML::Value << seed;
  out << YAML::Key << "target_gdd" << YAML::Value << target_gdd;
  out << YAML::Key << "enable_repot_profile_switch" << YAML::Value
      << enable_repot_profile_switch;
  out << YAML::Key << "repot_switch_gdd" << YAML::Value << repot_switch_gdd;
}

void ScotsPine::Deserialize(const YAML::Node& in) {
  descriptor_ref.Load("descriptor_ref", in);
  post_repot_descriptor_ref.Load("post_repot_descriptor_ref", in);
  if (in["seed"]) seed = in["seed"].as<unsigned int>();
  if (in["target_gdd"]) {
    target_gdd = in["target_gdd"].as<float>();
  } else if (in["target_year"]) {
    target_gdd = static_cast<float>(in["target_year"].as<int>()) * kPineGddPerYear;
  }
  if (in["enable_repot_profile_switch"]) {
    enable_repot_profile_switch = in["enable_repot_profile_switch"].as<bool>();
  }
  if (in["repot_switch_gdd"]) {
    repot_switch_gdd = std::max(0.0f, in["repot_switch_gdd"].as<float>());
  }
}

void ScotsPine::CollectAssetRef(std::vector<AssetRef>& list) {
  list.push_back(descriptor_ref);
  list.push_back(post_repot_descriptor_ref);
}
