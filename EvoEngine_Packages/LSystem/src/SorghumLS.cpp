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
#include <Texture2D.hpp>
#include <Times.hpp>
#include <Transform.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <map>
#include <stdexcept>

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

glm::vec4 EvaluateInternodeGreenPalette(const int rank, const int max_rank, const int order) {
  const float rank_t =
      (max_rank > 0) ? std::clamp(static_cast<float>(rank) / static_cast<float>(max_rank), 0.0f, 1.0f) : 0.0f;

  const glm::vec3 dark_green(0.12f, 0.33f, 0.11f);
  const glm::vec3 mid_green(0.24f, 0.56f, 0.18f);
  const glm::vec3 light_green(0.46f, 0.77f, 0.34f);

  glm::vec3 color = (rank_t <= 0.5f) ? glm::mix(dark_green, mid_green, rank_t * 2.0f)
                                     : glm::mix(mid_green, light_green, (rank_t - 0.5f) * 2.0f);

  // Slightly darken higher-order axes so tillers remain separable but in-palette.
  const float order_t = std::clamp(static_cast<float>(order), 0.0f, 3.0f) / 3.0f;
  color *= (1.0f - 0.08f * order_t);

  return glm::vec4(color, 1.0f);
}

glm::vec4 EvaluatePanicleColor(const SorghumLSDescriptor* descriptor, const float progress) {
  const glm::vec3 immature = descriptor ? descriptor->panicle_immature_color : glm::vec3(0.32f, 0.56f, 0.16f);
  const glm::vec3 mature = descriptor ? descriptor->panicle_mature_color : glm::vec3(0.48f, 0.16f, 0.07f);
  return glm::vec4(glm::mix(immature, mature, std::clamp(progress, 0.0f, 1.0f)), 1.0f);
}

void AppendPanicleSegment(std::vector<Vertex>& vertices, std::vector<glm::uvec3>& triangles, const glm::vec3& start,
                          const glm::vec3& end, const float start_radius, const float end_radius,
                          const glm::vec4& color) {
  const glm::vec3 delta = end - start;
  const float length = glm::length(delta);
  if (length <= 1.0e-6f) {
    return;
  }
  constexpr uint32_t kSides = 6u;
  const glm::vec3 direction = delta / length;
  const glm::vec3 reference = std::abs(direction.y) > 0.95f ? glm::vec3(1.0f, 0.0f, 0.0f) : glm::vec3(0.0f, 1.0f, 0.0f);
  const glm::vec3 right = glm::normalize(glm::cross(reference, direction));
  const glm::vec3 up = glm::normalize(glm::cross(direction, right));
  const uint32_t base = static_cast<uint32_t>(vertices.size());
  for (uint32_t ring = 0; ring < 2u; ++ring) {
    const float t = static_cast<float>(ring);
    const glm::vec3 center = glm::mix(start, end, t);
    const float radius = glm::mix(std::max(0.00005f, start_radius), std::max(0.00005f, end_radius), t);
    for (uint32_t side = 0; side <= kSides; ++side) {
      const float u = static_cast<float>(side) / static_cast<float>(kSides);
      const float angle = glm::two_pi<float>() * u;
      const glm::vec3 radial = right * std::cos(angle) + up * std::sin(angle);
      Vertex vertex{};
      vertex.position = center + radial * radius;
      vertex.normal = radial;
      vertex.tangent = glm::normalize(-right * std::sin(angle) + up * std::cos(angle));
      vertex.color = color;
      vertex.tex_coord = glm::vec2(u, t);
      vertices.emplace_back(vertex);
    }
  }
  const uint32_t stride = kSides + 1u;
  for (uint32_t side = 0; side < kSides; ++side) {
    const uint32_t a = base + side;
    const uint32_t b = a + 1u;
    const uint32_t c = a + stride;
    const uint32_t d = c + 1u;
    // The radial vertex normal points outwards.  Keep the geometric winding
    // consistent with it so the one-sided OptiX material retains the visible
    // exterior of the rachis, branches, and spikelets.  The former reversed
    // winding made the native panicle mesh invisible from the exterior even
    // though its L-system graph and metadata were complete.
    triangles.emplace_back(a, b, c);
    triangles.emplace_back(b, d, c);
  }
}

void GeneratePanicleMesh(const SorghumGraph& graph, const SorghumLSDescriptor* descriptor,
                         std::vector<Vertex>& vertices, std::vector<glm::uvec3>& triangles,
                         std::vector<SorghumOrganGeometryRange>& ranges, uint32_t& branch_count,
                         uint32_t& spikelet_count) {
  vertices.clear();
  triangles.clear();
  branch_count = 0;
  spikelet_count = 0;
  for (const auto handle : graph.PeekSortedNodeList()) {
    const auto& node = graph.PeekNode(handle);
    const size_t vertex_start = vertices.size();
    const size_t triangle_start = triangles.size();
    SorghumOrganGeometryRange range;
    range.node_id = static_cast<int>(node.GetIndex());
    if (node.data.Is<SorghumPanicleRachis>()) {
      const auto& rachis = node.data.Get<SorghumPanicleRachis>();
      AppendPanicleSegment(vertices, triangles, node.info.global_position, node.info.GetGlobalEndPosition(),
                           rachis.thickness * 0.5f, rachis.thickness * 0.38f,
                           EvaluatePanicleColor(descriptor, rachis.growth_progress));
      range.kind = SorghumOrganGeometryKind::PanicleRachis;
    } else if (node.data.Is<SorghumPanicleBranch>()) {
      const auto& branch = node.data.Get<SorghumPanicleBranch>();
      AppendPanicleSegment(vertices, triangles, node.info.global_position, node.info.GetGlobalEndPosition(),
                           branch.thickness * 0.5f, branch.thickness * 0.26f,
                           EvaluatePanicleColor(descriptor, branch.growth_progress));
      range.kind = SorghumOrganGeometryKind::PanicleBranch;
      range.rank = branch.rank;
      ++branch_count;
    } else if (node.data.Is<SorghumPanicleSpikelet>()) {
      const auto& spikelet = node.data.Get<SorghumPanicleSpikelet>();
      const glm::vec3 start = node.info.global_position;
      const glm::vec3 end = node.info.GetGlobalEndPosition();
      const float total_length = spikelet.pedicel_length + spikelet.spikelet_length;
      const float pedicel_fraction = total_length > 1.0e-6f ? spikelet.pedicel_length / total_length : 0.0f;
      const glm::vec3 seed_start = glm::mix(start, end, pedicel_fraction);
      const glm::vec3 seed_middle = glm::mix(seed_start, end, 0.62f);
      const glm::vec4 color = EvaluatePanicleColor(descriptor, spikelet.growth_progress);
      if (spikelet.pedicel_length > 1.0e-6f) {
        AppendPanicleSegment(vertices, triangles, start, seed_start, spikelet.radius * 0.22f, spikelet.radius * 0.30f,
                             color);
      }
      AppendPanicleSegment(vertices, triangles, seed_start, seed_middle, spikelet.radius * 0.40f, spikelet.radius,
                           color);
      AppendPanicleSegment(vertices, triangles, seed_middle, end, spikelet.radius, spikelet.radius * 0.16f, color);
      range.kind = SorghumOrganGeometryKind::PanicleSpikelet;
      range.rank = spikelet.branch_rank;
      ++spikelet_count;
    } else {
      continue;
    }
    range.vertex_offset = static_cast<uint32_t>(vertex_start);
    range.vertex_count = static_cast<uint32_t>(vertices.size() - vertex_start);
    range.triangle_offset = static_cast<uint32_t>(triangle_start);
    range.triangle_count = static_cast<uint32_t>(triangles.size() - triangle_start);
    ranges.emplace_back(range);
  }
}

void GenerateUnitCylinderMesh(std::vector<Vertex>& vertices, std::vector<unsigned int>& indices, int segments = 12) {
  vertices.clear();
  indices.clear();

  const glm::vec4 stem_color(0.36f, 0.58f, 0.24f, 1.0f);
  const float angle_step = glm::two_pi<float>() / static_cast<float>(segments);
  constexpr std::array<float, 4> kRingY{0.0f, 0.06f, 0.94f, 1.0f};
  constexpr std::array<float, 4> kRingRadius{1.06f, 1.0f, 1.0f, 1.06f};

  for (size_t ring = 0; ring < kRingY.size(); ++ring) {
    for (int s = 0; s <= segments; ++s) {
      const float angle = angle_step * static_cast<float>(s);
      const float cx = std::cos(angle);
      const float cz = std::sin(angle);
      Vertex v;
      v.position = glm::vec3(cx * kRingRadius[ring], kRingY[ring], cz * kRingRadius[ring]);
      v.normal = glm::normalize(glm::vec3(cx, 0.0f, cz));
      v.tangent = glm::normalize(glm::vec3(-cz, 0.0f, cx));
      v.color = stem_color;
      v.tex_coord = glm::vec2(static_cast<float>(s) / static_cast<float>(segments), kRingY[ring]);
      vertices.push_back(v);
    }
  }

  const unsigned int ring_stride = static_cast<unsigned int>(segments + 1);
  for (unsigned int ring = 0; ring + 1 < kRingY.size(); ++ring) {
    for (int s = 0; s < segments; ++s) {
      const unsigned int s0 = ring * ring_stride + static_cast<unsigned int>(s);
      const unsigned int s1 = s0 + 1u;
      const unsigned int e0 = s0 + ring_stride;
      const unsigned int e1 = e0 + 1u;
      indices.insert(indices.end(), {s0, e0, s1, s1, e0, e1});
    }
  }

  for (int cap = 0; cap < 2; ++cap) {
    const float y = static_cast<float>(cap);
    const float normal_y = cap == 0 ? -1.0f : 1.0f;
    const unsigned int center = static_cast<unsigned int>(vertices.size());
    Vertex center_vertex;
    center_vertex.position = glm::vec3(0.0f, y, 0.0f);
    center_vertex.normal = glm::vec3(0.0f, normal_y, 0.0f);
    center_vertex.tangent = glm::vec3(1.0f, 0.0f, 0.0f);
    center_vertex.color = stem_color;
    center_vertex.tex_coord = glm::vec2(0.5f);
    vertices.push_back(center_vertex);
    const unsigned int rim_start = static_cast<unsigned int>(vertices.size());
    for (int s = 0; s <= segments; ++s) {
      const float angle = angle_step * static_cast<float>(s);
      Vertex v = center_vertex;
      v.position = glm::vec3(std::cos(angle) * kRingRadius[cap == 0 ? 0 : 3], y,
                             std::sin(angle) * kRingRadius[cap == 0 ? 0 : 3]);
      v.tex_coord = glm::vec2(0.5f + 0.5f * std::cos(angle), 0.5f + 0.5f * std::sin(angle));
      vertices.push_back(v);
    }
    for (int s = 0; s < segments; ++s) {
      const unsigned int a = rim_start + static_cast<unsigned int>(s);
      const unsigned int b = a + 1u;
      if (cap == 0)
        indices.insert(indices.end(), {center, b, a});
      else
        indices.insert(indices.end(), {center, a, b});
    }
  }
}

void GenerateContinuousCulmMesh(const SorghumGraph& graph, const SorghumLSDescriptor* descriptor,
                                std::vector<Vertex>& vertices, std::vector<glm::uvec3>& triangles,
                                std::vector<SorghumOrganGeometryRange>& organ_ranges) {
  vertices.clear();
  triangles.clear();
  std::map<int, std::vector<LNodeHandle>> axes;
  for (const auto handle : graph.PeekSortedNodeList()) {
    const auto& node = graph.PeekNode(handle);
    if (node.data.Is<SorghumInternode>() && node.info.length > 0.0f) {
      axes[node.data.Get<SorghumInternode>().axis_id].push_back(handle);
    }
  }

  const uint32_t sides = descriptor ? descriptor->culm_radial_segments : 24u;
  const float node_scale = descriptor ? descriptor->culm_node_radius_scale : 1.08f;
  const float repeat_m = descriptor ? descriptor->culm_texture_repeat_m : 0.25f;
  for (auto& [axis_id, handles] : axes) {
    const uint32_t range_vertex_start = static_cast<uint32_t>(vertices.size());
    const uint32_t range_triangle_start = static_cast<uint32_t>(triangles.size());
    std::sort(handles.begin(), handles.end(), [&](const auto a, const auto b) {
      return graph.PeekNode(a).data.Get<SorghumInternode>().rank < graph.PeekNode(b).data.Get<SorghumInternode>().rank;
    });
    if (handles.empty())
      continue;

    std::vector<glm::vec3> centers;
    std::vector<float> radii;
    std::vector<int> ranks;
    centers.reserve(handles.size() * 3 + 1);
    radii.reserve(handles.size() * 3 + 1);
    ranks.reserve(handles.size() * 3 + 1);
    for (size_t i = 0; i < handles.size(); ++i) {
      const auto& node = graph.PeekNode(handles[i]);
      const auto& internode = node.data.Get<SorghumInternode>();
      const glm::vec3 start = node.info.global_position;
      const glm::vec3 end = node.info.GetGlobalEndPosition();
      const float radius = std::max(0.0001f, internode.thickness * 0.5f);
      if (i == 0) {
        centers.push_back(start);
        radii.push_back(radius);
        ranks.push_back(internode.rank);
      }
      centers.push_back(glm::mix(start, end, 0.06f));
      radii.push_back(radius);
      ranks.push_back(internode.rank);
      centers.push_back(glm::mix(start, end, 0.94f));
      radii.push_back(radius);
      ranks.push_back(internode.rank);
      const bool has_next = i + 1 < handles.size();
      float end_radius = radius;
      if (has_next) {
        const auto& next = graph.PeekNode(handles[i + 1]).data.Get<SorghumInternode>();
        end_radius = 0.5f * (radius + std::max(0.0001f, next.thickness * 0.5f)) * node_scale;
      }
      centers.push_back(end);
      radii.push_back(end_radius);
      ranks.push_back(internode.rank);
    }

    float distance = 0.0f;
    const uint32_t vertex_base = static_cast<uint32_t>(vertices.size());
    for (size_t ring = 0; ring < centers.size(); ++ring) {
      if (ring > 0)
        distance += glm::distance(centers[ring - 1], centers[ring]);
      const glm::vec3 before = ring > 0 ? centers[ring] - centers[ring - 1] : centers[1] - centers[0];
      const glm::vec3 after = ring + 1 < centers.size() ? centers[ring + 1] - centers[ring] : before;
      const glm::vec3 direction = glm::normalize(before + after);
      const glm::vec3 reference = std::abs(direction.y) > 0.95f ? glm::vec3(1, 0, 0) : glm::vec3(0, 1, 0);
      const glm::vec3 right = glm::normalize(glm::cross(reference, direction));
      const glm::vec3 up = glm::normalize(glm::cross(direction, right));
      for (uint32_t side = 0; side <= sides; ++side) {
        const float u = static_cast<float>(side) / static_cast<float>(sides);
        const float angle = glm::two_pi<float>() * u;
        const glm::vec3 radial = right * std::cos(angle) + up * std::sin(angle);
        Vertex vertex{};
        vertex.position = centers[ring] + radial * radii[ring];
        vertex.normal = radial;
        vertex.tangent = glm::normalize(-right * std::sin(angle) + up * std::cos(angle));
        vertex.tex_coord = glm::vec2(u, distance / std::max(0.01f, repeat_m));
        vertex.color = g_sorghum_ls_color_mode == SorghumLS::ColorMode::Shaded
                           ? glm::vec4(1.0f)
                           : EvaluateInternodeGreenPalette(ranks[ring], static_cast<int>(handles.size()) - 1,
                                                           axis_id == 0 ? 0 : 1);
        vertices.push_back(vertex);
      }
    }
    const uint32_t stride = sides + 1u;
    for (uint32_t ring = 0; ring + 1u < centers.size(); ++ring) {
      for (uint32_t side = 0; side < sides; ++side) {
        const uint32_t a = vertex_base + ring * stride + side;
        const uint32_t b = a + 1u;
        const uint32_t c = a + stride;
        const uint32_t d = c + 1u;
        triangles.emplace_back(a, c, b);
        triangles.emplace_back(b, c, d);
      }
    }
    for (uint32_t cap = 0; cap < 2u; ++cap) {
      const uint32_t ring = cap == 0u ? 0u : static_cast<uint32_t>(centers.size() - 1u);
      const glm::vec3 axis = cap == 0u ? glm::normalize(centers[1] - centers[0])
                                       : glm::normalize(centers.back() - centers[centers.size() - 2u]);
      const glm::vec3 normal = cap == 0u ? -axis : axis;
      const uint32_t center_index = static_cast<uint32_t>(vertices.size());
      Vertex center{};
      center.position = centers[ring];
      center.normal = normal;
      center.tangent = vertices[vertex_base + ring * stride].tangent;
      center.tex_coord = glm::vec2(0.5f);
      center.color = vertices[vertex_base + ring * stride].color;
      vertices.push_back(center);
      const uint32_t rim_start = static_cast<uint32_t>(vertices.size());
      for (uint32_t side = 0; side <= sides; ++side) {
        const float angle = glm::two_pi<float>() * static_cast<float>(side) / static_cast<float>(sides);
        Vertex rim = vertices[vertex_base + ring * stride + side];
        rim.normal = normal;
        rim.tex_coord = glm::vec2(0.5f + 0.5f * std::cos(angle), 0.5f + 0.5f * std::sin(angle));
        vertices.push_back(rim);
      }
      for (uint32_t side = 0; side < sides; ++side) {
        const uint32_t a = rim_start + side;
        const uint32_t b = a + 1u;
        if (cap == 0u)
          triangles.emplace_back(center_index, b, a);
        else
          triangles.emplace_back(center_index, a, b);
      }
    }
    SorghumOrganGeometryRange range;
    range.kind = SorghumOrganGeometryKind::Culm;
    range.axis_id = axis_id;
    range.vertex_offset = range_vertex_start;
    range.vertex_count = static_cast<uint32_t>(vertices.size()) - range_vertex_start;
    range.triangle_offset = range_triangle_start;
    range.triangle_count = static_cast<uint32_t>(triangles.size()) - range_triangle_start;
    organ_ranges.emplace_back(range);
  }
}

bool BuildStemContextForLeafNode(const SorghumGraph& graph, const LNodeHandle leaf_handle, StemContext& out_stem_ctx) {
  out_stem_ctx.segments.clear();
  if (leaf_handle < 0)
    return false;

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

bool HasSorghumGeometryChildren(const std::shared_ptr<Scene>& scene, const Entity owner) {
  if (!scene || !scene->IsEntityValid(owner)) {
    return false;
  }

  bool has_internodes = false;
  bool has_leaves = false;
  bool has_panicle = false;
  for (const auto& child : scene->GetChildren(owner)) {
    if (!scene->IsEntityValid(child)) {
      continue;
    }
    const auto name = scene->GetEntityName(child);
    if (name == "Sorghum Internodes") {
      has_internodes = true;
    } else if (name == "Sorghum Leaves") {
      has_leaves = true;
    } else if (name == "Sorghum Panicle") {
      has_panicle = true;
    }

    if (has_internodes && has_leaves && has_panicle) {
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
  if (render_target_) {
    render_target_->ClearAllChannels();
  }

  const auto scene = GetScene();
  if (!scene) {
    geometry_snapshot_.reset();
    return;
  }
  const auto self = GetOwner();
  if (!scene->IsEntityValid(self)) {
    geometry_snapshot_.reset();
    return;
  }

  // One-time cleanup for generated children created by older Sorghum builds.
  // Authored children are deliberately left alone.
  const auto children = scene->GetChildren(self);
  for (const auto& child : children) {
    const auto name = scene->GetEntityName(child);
    if (name == "Sorghum Internodes" || name == "Sorghum Leaves" || name == "Sorghum Panicle") {
      scene->DeleteEntity(child);
    }
  }
  geometry_snapshot_.reset();
}

const std::shared_ptr<const SorghumGeometrySnapshot>& SorghumLS::GetGeometrySnapshot() const {
  return geometry_snapshot_;
}

void SorghumLS::GenerateGeometryEntities(const bool uncapped_growth, const bool reuse_geometry_entities) {
  if (!reuse_geometry_entities) {
    ClearGeometryEntities();
  }
  growth_model.Reset();
  GrowToTargetGDD(uncapped_growth);
}

std::shared_ptr<const SorghumGeometrySnapshot> SorghumLS::GenerateGeometrySnapshot(const bool uncapped_growth,
                                                                                   const uint32_t max_growth_steps) {
  growth_model.Reset();
  const double grow_start = GetApplication().GetTimes().Now();
  const auto descriptor = descriptor_ref.Get<SorghumLSDescriptor>();
  if (!descriptor) {
    last_grow_seconds = 0.0;
    return {};
  }

  growth_model.Initialize(*descriptor, seed);
  growth_model.GrowToGDD(target_gdd, uncapped_growth ? 0u : max_growth_steps);
  if (descriptor->finalize_snapshot_morphology) {
    growth_model.FinalizeSnapshotMorphology();
  }
  last_grow_seconds = GetApplication().GetTimes().Now() - grow_start;
  return BuildGeometrySnapshot();
}

std::shared_ptr<const SorghumGeometrySnapshot> SorghumLS::AdvanceGeometrySnapshot(const bool uncapped_growth,
                                                                                  const uint32_t max_growth_steps) {
  const double grow_start = GetApplication().GetTimes().Now();
  const auto descriptor = descriptor_ref.Get<SorghumLSDescriptor>();
  if (!descriptor) {
    last_grow_seconds = 0.0;
    return {};
  }

  bool reinitialized = false;
  if (!growth_model.IsInitialized()) {
    growth_model.Initialize(*descriptor, seed);
    reinitialized = true;
  }
  const float gdd_step = std::max(1.0e-5f, growth_model.gdd_per_growth_step);
  if (target_gdd + gdd_step < growth_model.accumulated_gdd) {
    growth_model.Initialize(*descriptor, seed);
    reinitialized = true;
  }

  growth_model.GrowToGDD(target_gdd, uncapped_growth ? 0u : max_growth_steps);
  if (descriptor->finalize_snapshot_morphology) {
    growth_model.FinalizeSnapshotMorphology();
  }
  last_grow_seconds = GetApplication().GetTimes().Now() - grow_start;

  // A sub-step GDD change has no new L-system state.  Preserve the existing
  // snapshot so ordered animation capture can reuse its exact geometry.
  if (!reinitialized && growth_model.last_growth_steps == 0 && geometry_snapshot_) {
    return geometry_snapshot_;
  }
  return BuildGeometrySnapshot();
}

std::shared_ptr<const SorghumGeometrySnapshot> SorghumLS::GeneratePreviewGeometrySnapshot(
    const float preview_target_gdd, const uint32_t preview_max_growth_steps) {
  growth_model.Reset();
  const double grow_start = GetApplication().GetTimes().Now();
  const auto descriptor = descriptor_ref.Get<SorghumLSDescriptor>();
  if (!descriptor) {
    last_grow_seconds = 0.0;
    return {};
  }

  growth_model.Initialize(*descriptor, seed);
  growth_model.GrowToGDD(std::min(target_gdd, std::max(0.0f, preview_target_gdd)),
                         std::max(1u, preview_max_growth_steps));
  last_grow_seconds = GetApplication().GetTimes().Now() - grow_start;

  const auto full_quality_settings = leaf_mesh_settings;
  leaf_mesh_settings.vertical_subdivision_length =
      std::max(0.025f, full_quality_settings.vertical_subdivision_length * 2.5f);
  leaf_mesh_settings.horizontal_subdivision_step = std::min(3, full_quality_settings.horizontal_subdivision_step);
  auto snapshot = BuildGeometrySnapshot();
  leaf_mesh_settings = full_quality_settings;
  return snapshot;
}

void SorghumLS::GeneratePreviewGeometryEntities(const float preview_target_gdd,
                                                const uint32_t preview_max_growth_steps) {
  PublishGeometrySnapshot(GeneratePreviewGeometrySnapshot(preview_target_gdd, preview_max_growth_steps));
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
  if (descriptor->finalize_snapshot_morphology) {
    growth_model.FinalizeSnapshotMorphology();
  }
  last_grow_seconds = GetApplication().GetTimes().Now() - grow_start;

  const bool no_growth_step = !reinitialized && growth_model.last_growth_steps == 0;
  // After Ctrl+W reset, geometry children may be missing while growth can still
  // report zero new steps. Force one rebuild to restore child entities.
  const bool missing_geometry_children = !HasSorghumGeometryChildren(GetScene(), GetOwner());
  if (no_growth_step && !missing_geometry_children) {
    return;
  }

  RebuildGeometry();
}

void SorghumLS::SetSeasonalChronologicalMode(const bool enable_independent_chronological_clock) {
  growth_model.SetChronologicalCoupledToThermal(!enable_independent_chronological_clock);
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

std::shared_ptr<const SorghumGeometrySnapshot> SorghumLS::BuildGeometrySnapshot() {
  auto& times = GetApplication().GetTimes();
  const double rebuild_start = times.Now();
  last_invalid_instance_count = 0;
  last_node_count = 0;
  last_internode_count = 0;
  last_leaf_count = 0;
  last_live_leaf_count = 0;
  last_panicle_branch_count = 0;
  last_panicle_spikelet_count = 0;
  last_rebuild_internode_seconds = 0.0;
  last_leaf_spline_seconds = 0.0;
  last_leaf_mesh_seconds = 0.0;
  last_mesh_upload_seconds = 0.0;

  if (!growth_model.IsInitialized()) {
    last_rebuild_seconds = 0.0;
    return {};
  }

  const auto scene = GetScene();
  const auto owner = GetOwner();
  const auto color_mode = GetGlobalColorMode();
  const glm::vec4 instance_color = HashToColor(owner.GetIndex());
  const auto descriptor = descriptor_ref.Get<SorghumLSDescriptor>();
  auto snapshot = std::make_shared<SorghumGeometrySnapshot>();
  snapshot->geometry_version = ++geometry_version_;
  snapshot->seed = seed;
  snapshot->target_gdd = target_gdd;
  if (geometry_snapshot_) {
    snapshot->internode_instances.reserve(geometry_snapshot_->internode_instances.size());
    snapshot->culm_vertices.reserve(geometry_snapshot_->culm_vertices.size());
    snapshot->culm_triangles.reserve(geometry_snapshot_->culm_triangles.size());
    snapshot->leaf_vertices.reserve(geometry_snapshot_->leaf_vertices.size());
    snapshot->leaf_triangles.reserve(geometry_snapshot_->leaf_triangles.size());
    snapshot->panicle_vertices.reserve(geometry_snapshot_->panicle_vertices.size());
    snapshot->panicle_triangles.reserve(geometry_snapshot_->panicle_triangles.size());
    snapshot->organ_ranges.reserve(geometry_snapshot_->organ_ranges.size());
  }
  auto current_leaf_mesh_settings = leaf_mesh_settings;
  current_leaf_mesh_settings.gravity_local_m_s2 =
      glm::conjugate(scene->GetDataComponent<GlobalTransform>(owner).GetRotation()) * glm::vec3(0.0f, -9.80665f, 0.0f);
  SorghumLeafAtlasLayout base_leaf_atlas_layout;
  if (descriptor) {
    base_leaf_atlas_layout.variant_columns = descriptor->leaf_atlas_variant_columns;
    base_leaf_atlas_layout.variant_rows = descriptor->leaf_atlas_variant_rows;
    base_leaf_atlas_layout.variant_count = descriptor->leaf_atlas_variant_count;
    base_leaf_atlas_layout.tile_uv_inset = descriptor->leaf_atlas_tile_uv_inset;
    base_leaf_atlas_layout.distal_region_uses_top_half = descriptor->leaf_atlas_distal_region_uses_top_half;
    base_leaf_atlas_layout.semantic_quadrants = descriptor->leaf_atlas_semantic_quadrants;
  }
  base_leaf_atlas_layout = NormalizeSorghumLeafAtlasLayout(base_leaf_atlas_layout);

  // -------------------------------------------------------------------------
  // Internodes as instanced cylinders via Particles.
  // -------------------------------------------------------------------------
  {
    const double internode_start = times.Now();
    auto& infos = snapshot->internode_instances;

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

    const glm::quat cylinder_axis_fix = glm::angleAxis(-glm::half_pi<float>(), glm::vec3(1.0f, 0.0f, 0.0f));

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
      const glm::mat4 model = glm::translate(node.info.global_position) * glm::mat4_cast(instance_rotation) *
                              glm::scale(glm::vec3(half_thick, node.info.length, half_thick));
      if (!IsFiniteMat4(model)) {
        last_invalid_instance_count++;
        continue;
      }

      pi.instance_matrix.value = model;
      pi.instance_color = EvaluateInternodeGreenPalette(internode.rank, max_internode_rank, internode.order);
      const uint32_t instance_offset = static_cast<uint32_t>(infos.size());
      infos.push_back(pi);
      SorghumOrganGeometryRange range;
      range.kind = SorghumOrganGeometryKind::InternodeInstance;
      range.axis_id = internode.axis_id;
      range.rank = internode.rank;
      range.node_id = static_cast<int>(node.GetIndex());
      range.instance_offset = instance_offset;
      range.instance_count = 1;
      snapshot->organ_ranges.emplace_back(range);
    }

    last_internode_count = static_cast<uint32_t>(infos.size());

    auto& culm_vertices = snapshot->culm_vertices;
    auto& culm_triangles = snapshot->culm_triangles;
    GenerateContinuousCulmMesh(growth_model.graph, descriptor.get(), culm_vertices, culm_triangles,
                               snapshot->organ_ranges);
    last_rebuild_internode_seconds = times.Now() - internode_start;
  }

  // -------------------------------------------------------------------------
  // Leaves as one procedural mesh (all live leaves aggregated).
  // -------------------------------------------------------------------------
  {
    auto& leaf_vertices = snapshot->leaf_vertices;
    auto& leaf_indices = snapshot->leaf_triangles;
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
      BuildLeafSplineFromState(leaf, stem_ctx, growth_model.sampled, current_leaf_mesh_settings, leaf_spline);
      last_leaf_spline_seconds += times.Now() - spline_start;

      const double mesh_start = times.Now();
      const size_t vertex_start = leaf_vertices.size();
      const size_t triangle_start = leaf_indices.size();
      auto leaf_atlas_layout = base_leaf_atlas_layout;
      leaf_atlas_layout.variant_index = ComputeSorghumLeafAtlasVariant(
          seed, static_cast<uint32_t>(node.GetIndex()), leaf.node_random, leaf_atlas_layout.variant_count);
      GenerateBladeGeometry(leaf_spline, leaf, growth_model.sampled, leaf_mesh_settings, leaf_vertices, leaf_indices,
                            false, static_cast<uint32_t>(node.GetIndex()), leaf_atlas_layout);
      if (leaf_bottom_face) {
        GenerateBladeGeometry(leaf_spline, leaf, growth_model.sampled, leaf_mesh_settings, leaf_vertices, leaf_indices,
                              true, static_cast<uint32_t>(node.GetIndex()), leaf_atlas_layout);
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
      SorghumOrganGeometryRange range;
      range.kind = SorghumOrganGeometryKind::Leaf;
      range.axis_id = leaf.axis_id;
      range.rank = leaf.rank;
      range.node_id = static_cast<int>(node.GetIndex());
      range.vertex_offset = static_cast<uint32_t>(vertex_start);
      range.vertex_count = static_cast<uint32_t>(leaf_vertices.size() - vertex_start);
      range.triangle_offset = static_cast<uint32_t>(triangle_start);
      range.triangle_count = static_cast<uint32_t>(leaf_indices.size() - triangle_start);
      snapshot->organ_ranges.emplace_back(range);
      last_leaf_mesh_seconds += times.Now() - mesh_start;
    }
  }

  GeneratePanicleMesh(growth_model.graph, descriptor.get(), snapshot->panicle_vertices, snapshot->panicle_triangles,
                      snapshot->organ_ranges, last_panicle_branch_count, last_panicle_spikelet_count);

  snapshot->node_count = last_node_count;
  snapshot->internode_count = last_internode_count;
  snapshot->leaf_count = last_leaf_count;
  snapshot->live_leaf_count = last_live_leaf_count;
  snapshot->panicle_branch_count = last_panicle_branch_count;
  snapshot->panicle_spikelet_count = last_panicle_spikelet_count;
  snapshot->invalid_instance_count = last_invalid_instance_count;
  last_rebuild_seconds = times.Now() - rebuild_start;
  return snapshot;
}

void SorghumLS::PublishGeometrySnapshot(const std::shared_ptr<const SorghumGeometrySnapshot>& snapshot,
                                        const bool update_render_geometry, const bool preserve_existing_geometry) {
  if (!snapshot) {
    return;
  }

  geometry_snapshot_ = snapshot;
  if (!update_render_geometry) {
    return;
  }

  auto& times = GetApplication().GetTimes();
  const double publish_start = times.Now();
  const auto scene = GetScene();
  const auto owner = GetOwner();
  if (!scene || !scene->IsEntityValid(owner)) {
    return;
  }

  if (!render_target_ || render_target_->GetRootEntity() != owner) {
    render_target_ = std::make_unique<PlantRenderTarget>(scene, owner);
  }

  const auto descriptor = descriptor_ref.Get<SorghumLSDescriptor>();

  VertexAttributes attributes{};
  attributes.normal = true;
  attributes.tangent = true;
  attributes.tex_coord = true;
  attributes.color = true;

  if (snapshot->culm_triangles.empty()) {
    if (!preserve_existing_geometry) {
      render_target_->RemoveMeshChannel(kChannelCulm);
    }
  } else if (auto* culm_channel = render_target_->GetOrCreateMeshChannel(kChannelCulm, "Sorghum Internodes")) {
    if (const auto material = culm_channel->GetMaterial()) {
      const auto albedo = descriptor ? descriptor->stem_albedo_texture.Get<Texture2D>() : nullptr;
      material->SetTexture(&GltfShadeMaterial::pbr_base_color_texture, albedo);
      material->SetTexture(&GltfShadeMaterial::normal_texture,
                           descriptor ? descriptor->stem_normal_texture.Get<Texture2D>() : nullptr);
      material->SetTexture(&GltfShadeMaterial::pbr_metallic_roughness_texture, nullptr);
      material->SetTexture(&GltfShadeMaterial::occlusion_texture,
                           descriptor ? descriptor->stem_ao_texture.Get<Texture2D>() : nullptr);
      material->vertex_color_only = false;
      auto& shade = material->material_data.shade_material;
      shade.pbr_base_color_factor =
          glm::vec4(albedo || !descriptor ? glm::vec3(1.0f) : descriptor->stem_material_albedo_color, 1.0f);
      shade.pbr_roughness_factor = descriptor ? descriptor->stem_material_roughness : 0.74f;
      shade.pbr_metallic_factor = descriptor ? descriptor->stem_material_metallic : 0.0f;
      shade.specular_factor = descriptor ? descriptor->stem_material_specular : 0.4f;
      shade.emissive_factor = glm::vec3(0.0f);
      material->MarkDirty();
    }
    culm_channel->Stage(attributes, snapshot->culm_vertices, snapshot->culm_triangles);
  }

  if (snapshot->leaf_triangles.empty()) {
    if (!preserve_existing_geometry) {
      render_target_->RemoveMeshChannel(kChannelLeaves);
    }
  } else if (auto* leaf_channel = render_target_->GetOrCreateMeshChannel(kChannelLeaves, "Sorghum Leaves")) {
    if (const auto material = leaf_channel->GetMaterial()) {
      const auto albedo = descriptor ? descriptor->leaf_atlas_albedo_texture.Get<Texture2D>() : nullptr;
      if (albedo) {
        material->vertex_color_only = false;
        material->SetTexture(&GltfShadeMaterial::pbr_base_color_texture, albedo);
        material->SetTexture(&GltfShadeMaterial::normal_texture,
                             descriptor->leaf_atlas_normal_texture.Get<Texture2D>());
        material->SetTexture(&GltfShadeMaterial::occlusion_texture, descriptor->leaf_atlas_ao_texture.Get<Texture2D>());
      } else {
        material->SetTexture(&GltfShadeMaterial::pbr_base_color_texture, nullptr);
        material->SetTexture(&GltfShadeMaterial::normal_texture, nullptr);
        material->SetTexture(&GltfShadeMaterial::occlusion_texture, nullptr);
        material->vertex_color_only = false;
      }
      material->SetTexture(&GltfShadeMaterial::pbr_metallic_roughness_texture, nullptr);
      auto& shade = material->material_data.shade_material;
      const auto leaf_color = descriptor ? descriptor->leaf_material_albedo_color : glm::vec3(0.26f, 0.52f, 0.18f);
      const auto transmission_color =
          descriptor ? descriptor->leaf_material_subsurface_color : glm::vec3(0.26f, 0.52f, 0.18f);
      const float transmission = descriptor ? descriptor->leaf_material_subsurface_factor : 0.0f;
      shade.pbr_base_color_factor = glm::vec4(albedo ? glm::vec3(1.0f) : leaf_color, 1.0f);
      shade.pbr_metallic_factor = descriptor ? descriptor->leaf_material_metallic : 0.0f;
      shade.pbr_roughness_factor = descriptor ? descriptor->leaf_material_roughness : 0.72f;
      shade.specular_factor = descriptor ? descriptor->leaf_material_specular : 0.45f;
      shade.double_sided = 1;
      shade.diffuse_transmission_factor = glm::clamp(transmission, 0.0f, 1.0f);
      shade.diffuse_transmission_color = transmission_color;
      shade.multiscatter_color_factor = transmission_color * glm::clamp(transmission, 0.0f, 1.0f);
      shade.emissive_factor = glm::vec3(0.0f);
      material->MarkDirty();
    }
    leaf_channel->Stage(attributes, snapshot->leaf_vertices, snapshot->leaf_triangles);
  }

  if (snapshot->panicle_triangles.empty()) {
    if (!preserve_existing_geometry) {
      render_target_->RemoveMeshChannel(kChannelPanicle);
    }
  } else if (auto* panicle_channel = render_target_->GetOrCreateMeshChannel(kChannelPanicle, "Sorghum Panicle")) {
    if (const auto material = panicle_channel->GetMaterial()) {
      material->vertex_color_only = true;
      material->SetTexture(&GltfShadeMaterial::pbr_base_color_texture, nullptr);
      material->SetTexture(&GltfShadeMaterial::normal_texture, nullptr);
      material->SetTexture(&GltfShadeMaterial::pbr_metallic_roughness_texture, nullptr);
      material->SetTexture(&GltfShadeMaterial::occlusion_texture, nullptr);
      auto& shade = material->material_data.shade_material;
      shade.pbr_base_color_factor = glm::vec4(1.0f);
      shade.pbr_roughness_factor = descriptor ? descriptor->panicle_material_roughness : 0.78f;
      shade.pbr_metallic_factor = 0.0f;
      shade.specular_factor = 0.35f;
      shade.emissive_factor = glm::vec3(0.0f);
      material->MarkDirty();
    }
    panicle_channel->Stage(attributes, snapshot->panicle_vertices, snapshot->panicle_triangles);
  }

  const double upload_start = times.Now();
  render_target_->FlushPending();
  last_mesh_upload_seconds = times.Now() - upload_start;
  last_rebuild_seconds += times.Now() - publish_start;
}

std::size_t SorghumLS::FlushGeometryUpdates() const {
  return render_target_ ? render_target_->FlushPending() : 0;
}

void SorghumLS::RebuildGeometry() {
  PublishGeometrySnapshot(BuildGeometrySnapshot());
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
  render_target_.reset();
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
    if (ImGui::DragFloat("Vertical subdivision", &leaf_mesh_settings.vertical_subdivision_length, 0.001f, 0.0001f, 1.0f,
                         "%.4f")) {
      leaf_mesh_settings.vertical_subdivision_length =
          std::max(0.0001f, leaf_mesh_settings.vertical_subdivision_length);
      changed = true;
    }
    if (ImGui::DragInt("Horizontal subdivision", &leaf_mesh_settings.horizontal_subdivision_step, 1, 2, 128)) {
      leaf_mesh_settings.horizontal_subdivision_step = std::max(2, leaf_mesh_settings.horizontal_subdivision_step);
      changed = true;
    }
    if (ImGui::Checkbox("Leaf sheath", &leaf_mesh_settings.enable_leaf_sheath)) {
      changed = true;
    }
    if (ImGui::DragFloat("Leaf width scale", &leaf_mesh_settings.leaf_width_scale, 0.01f, 0.0f, 10.0f, "%.3f")) {
      leaf_mesh_settings.leaf_width_scale = std::max(0.0f, leaf_mesh_settings.leaf_width_scale);
      changed = true;
    }
    if (ImGui::DragFloat("Leaf thickness", &leaf_mesh_settings.leaf_thickness, 0.0001f, 0.0f, 0.1f, "%.4f")) {
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
    ImGui::Text("Panicle: %u branches, %u spikelets", sorghum.last_panicle_branch_count,
                sorghum.last_panicle_spikelet_count);
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
  out << YAML::Key << "leaf_enable_sheath" << YAML::Value << leaf_mesh_settings.enable_leaf_sheath;
  out << YAML::Key << "leaf_width_scale" << YAML::Value << leaf_mesh_settings.leaf_width_scale;
}

void l_system_package::DeserializeSorghumLS(const YAML::Node& in, SorghumLS& target) {
  auto& descriptor_ref = target.descriptor_ref;
  auto& seed = target.seed;
  auto& target_gdd = target.target_gdd;
  auto& leaf_bottom_face = target.leaf_bottom_face;
  auto& leaf_mesh_settings = target.leaf_mesh_settings;
  descriptor_ref.Load("descriptor_ref", in);
  if (in["seed"])
    seed = in["seed"].as<unsigned int>();
  if (in["target_gdd"])
    target_gdd = std::max(0.0f, in["target_gdd"].as<float>());
  if (in["leaf_bottom_face"])
    leaf_bottom_face = in["leaf_bottom_face"].as<bool>();
  if (in["leaf_vertical_subdivision_length"]) {
    leaf_mesh_settings.vertical_subdivision_length =
        std::max(0.0001f, in["leaf_vertical_subdivision_length"].as<float>());
  }
  if (in["leaf_horizontal_subdivision_step"]) {
    leaf_mesh_settings.horizontal_subdivision_step = std::max(2, in["leaf_horizontal_subdivision_step"].as<int>());
  }
  if (in["leaf_enable_sheath"]) {
    leaf_mesh_settings.enable_leaf_sheath = in["leaf_enable_sheath"].as<bool>();
  }
  if (in["leaf_width_scale"]) {
    leaf_mesh_settings.leaf_width_scale = std::max(0.0f, in["leaf_width_scale"].as<float>());
  }
  if (in["leaf_thickness"]) {
    throw std::runtime_error("Legacy scene leaf_thickness is unsupported; regenerate with descriptor-owned thickness");
  }
}

void SorghumLS::CollectAssetRef(std::vector<AssetRef>& list) {
  list.push_back(descriptor_ref);
}
