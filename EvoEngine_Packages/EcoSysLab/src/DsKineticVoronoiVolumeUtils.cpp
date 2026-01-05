#include "DsKineticVoronoiVolumeUtils.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_set>

namespace eco_sys_lab_package {
namespace {

constexpr double kDegenerateAreaEpsilon = 1e-18;
constexpr double kDegenerateVolumeEpsilon = 1e-18;

struct UndirectedEdge {
  uint32_t a = 0;
  uint32_t b = 0;

  bool operator==(const UndirectedEdge& other) const {
    return a == other.a && b == other.b;
  }
};

struct UndirectedEdgeHash {
  size_t operator()(const UndirectedEdge& edge) const noexcept {
    return (static_cast<size_t>(edge.a) << 32) ^ static_cast<size_t>(edge.b);
  }
};

UndirectedEdge MakeEdge(const uint32_t i0, const uint32_t i1) {
  return i0 < i1 ? UndirectedEdge{i0, i1} : UndirectedEdge{i1, i0};
}

const glm::vec3& VertexPosition(const DsKineticVoronoiMeshing::GpuSegmentMeshletVertex& vertex,
                                const bool use_current_x) {
  return use_current_x ? vertex.x : vertex.x0;
}

double TriangleSignedVolumeContribution(const glm::vec3& p0, const glm::vec3& p1, const glm::vec3& p2) {
  // (1/6) p0 · (p1 × p2) — divergence theorem for a closed oriented surface.
  return static_cast<double>(glm::dot(p0, glm::cross(p1, p2))) / 6.0;
}

double AbsoluteTetVolume(const glm::vec3& origin, const glm::vec3& p0, const glm::vec3& p1, const glm::vec3& p2) {
  const glm::dvec3 a = glm::dvec3(p0) - glm::dvec3(origin);
  const glm::dvec3 b = glm::dvec3(p1) - glm::dvec3(origin);
  const glm::dvec3 c = glm::dvec3(p2) - glm::dvec3(origin);
  const double triple = glm::dot(a, glm::cross(b, c));
  return std::abs(triple) / 6.0;
}

bool TriangleIsDegenerate(const glm::vec3& p0, const glm::vec3& p1, const glm::vec3& p2) {
  const glm::dvec3 cross = glm::cross(glm::dvec3(p1) - glm::dvec3(p0), glm::dvec3(p2) - glm::dvec3(p0));
  return glm::dot(cross, cross) <= kDegenerateAreaEpsilon;
}

}  // namespace

std::unordered_map<unsigned int, std::vector<size_t>> DsKineticVoronoiVolumeUtils::TriangleIndicesBySegment(
    const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletVertex>& vertices,
    const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletTriangle>& triangles) {
  std::unordered_map<unsigned int, std::vector<size_t>> by_segment;
  by_segment.reserve(triangles.size() / 4 + 1);
  for (size_t tri_index = 0; tri_index < triangles.size(); ++tri_index) {
    const auto& tri = triangles[tri_index];
    if (tri.vertex_index0 >= vertices.size() || tri.vertex_index1 >= vertices.size() ||
        tri.vertex_index2 >= vertices.size()) {
      continue;
    }
    const unsigned int segment_index = vertices[tri.vertex_index0].segment_index;
    by_segment[segment_index].push_back(tri_index);
  }
  return by_segment;
}

DsKineticVoronoiVolumeUtils::MeshClosedness DsKineticVoronoiVolumeUtils::DiagnoseClosedness(
    const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletVertex>& vertices,
    const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletTriangle>& triangles,
    const std::vector<size_t>& triangle_indices) {
  MeshClosedness result;
  std::unordered_map<UndirectedEdge, size_t, UndirectedEdgeHash> edge_counts;
  edge_counts.reserve(triangle_indices.size() * 2);

  for (const size_t tri_index : triangle_indices) {
    if (tri_index >= triangles.size()) {
      continue;
    }
    const auto& tri = triangles[tri_index];
    if (tri.vertex_index0 >= vertices.size() || tri.vertex_index1 >= vertices.size() ||
        tri.vertex_index2 >= vertices.size()) {
      continue;
    }
    ++edge_counts[MakeEdge(tri.vertex_index0, tri.vertex_index1)];
    ++edge_counts[MakeEdge(tri.vertex_index1, tri.vertex_index2)];
    ++edge_counts[MakeEdge(tri.vertex_index2, tri.vertex_index0)];
  }

  for (const auto& [edge, count] : edge_counts) {
    (void)edge;
    if (count == 2) {
      continue;
    }
    if (count > 2) {
      ++result.non_manifold_edge_count;
    }
    ++result.boundary_edge_count;
  }

  result.is_closed = result.boundary_edge_count == 0 && !triangle_indices.empty();
  return result;
}

double DsKineticVoronoiVolumeUtils::ComputeMeshletVolume(
    const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletVertex>& vertices,
    const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletTriangle>& triangles,
    const std::vector<size_t>& triangle_indices, const bool use_current_x, bool* used_fallback,
    MeshClosedness* closedness_out) {
  if (used_fallback) {
    *used_fallback = false;
  }
  if (triangle_indices.empty()) {
    if (closedness_out) {
      *closedness_out = {};
    }
    return 0.0;
  }

  const MeshClosedness closedness = DiagnoseClosedness(vertices, triangles, triangle_indices);
  if (closedness_out) {
    *closedness_out = closedness;
  }

  if (closedness.is_closed) {
    // Precise (non-robust) divergence formula for a closed oriented triangle mesh.
    double signed_volume = 0.0;
    for (const size_t tri_index : triangle_indices) {
      if (tri_index >= triangles.size()) {
        continue;
      }
      const auto& tri = triangles[tri_index];
      if (tri.vertex_index0 >= vertices.size() || tri.vertex_index1 >= vertices.size() ||
          tri.vertex_index2 >= vertices.size()) {
        continue;
      }
      const glm::vec3& p0 = VertexPosition(vertices[tri.vertex_index0], use_current_x);
      const glm::vec3& p1 = VertexPosition(vertices[tri.vertex_index1], use_current_x);
      const glm::vec3& p2 = VertexPosition(vertices[tri.vertex_index2], use_current_x);
      if (TriangleIsDegenerate(p0, p1, p2)) {
        continue;
      }
      signed_volume += TriangleSignedVolumeContribution(p0, p1, p2);
    }
    const double volume = std::abs(signed_volume);
    return volume < kDegenerateVolumeEpsilon ? 0.0 : volume;
  }

  // Robust fallback: fan from the vertex centroid. Abs tet volumes tolerate flipped /
  // open / mildly non-manifold skins and give a good estimate for star-shaped meshlets.
  if (used_fallback) {
    *used_fallback = true;
  }

  std::unordered_set<uint32_t> unique_vertex_indices;
  unique_vertex_indices.reserve(triangle_indices.size());
  for (const size_t tri_index : triangle_indices) {
    if (tri_index >= triangles.size()) {
      continue;
    }
    const auto& tri = triangles[tri_index];
    if (tri.vertex_index0 < vertices.size()) {
      unique_vertex_indices.insert(tri.vertex_index0);
    }
    if (tri.vertex_index1 < vertices.size()) {
      unique_vertex_indices.insert(tri.vertex_index1);
    }
    if (tri.vertex_index2 < vertices.size()) {
      unique_vertex_indices.insert(tri.vertex_index2);
    }
  }
  if (unique_vertex_indices.size() < 4) {
    return 0.0;
  }

  glm::dvec3 centroid(0.0);
  for (const uint32_t index : unique_vertex_indices) {
    centroid += glm::dvec3(VertexPosition(vertices[index], use_current_x));
  }
  centroid /= static_cast<double>(unique_vertex_indices.size());
  const glm::vec3 origin(static_cast<float>(centroid.x), static_cast<float>(centroid.y),
                         static_cast<float>(centroid.z));

  double volume = 0.0;
  for (const size_t tri_index : triangle_indices) {
    if (tri_index >= triangles.size()) {
      continue;
    }
    const auto& tri = triangles[tri_index];
    if (tri.vertex_index0 >= vertices.size() || tri.vertex_index1 >= vertices.size() ||
        tri.vertex_index2 >= vertices.size()) {
      continue;
    }
    const glm::vec3& p0 = VertexPosition(vertices[tri.vertex_index0], use_current_x);
    const glm::vec3& p1 = VertexPosition(vertices[tri.vertex_index1], use_current_x);
    const glm::vec3& p2 = VertexPosition(vertices[tri.vertex_index2], use_current_x);
    if (TriangleIsDegenerate(p0, p1, p2)) {
      continue;
    }
    volume += AbsoluteTetVolume(origin, p0, p1, p2);
  }
  return volume < kDegenerateVolumeEpsilon ? 0.0 : volume;
}

DsKineticVoronoiVolumeUtils::VolumeResult DsKineticVoronoiVolumeUtils::ComputeAllMeshletVolumes(
    const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletVertex>& vertices,
    const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletTriangle>& triangles, const bool use_current_x) {
  VolumeResult result;
  const auto by_segment = TriangleIndicesBySegment(vertices, triangles);
  result.meshlets.reserve(by_segment.size());

  std::vector<unsigned int> segment_indices;
  segment_indices.reserve(by_segment.size());
  for (const auto& [segment_index, _] : by_segment) {
    segment_indices.push_back(segment_index);
  }
  std::sort(segment_indices.begin(), segment_indices.end());

  for (const unsigned int segment_index : segment_indices) {
    const auto& tri_indices = by_segment.at(segment_index);
    MeshletVolume entry;
    entry.segment_index = segment_index;
    MeshClosedness closedness;
    bool used_fallback = false;
    entry.volume = ComputeMeshletVolume(vertices, triangles, tri_indices, use_current_x, &used_fallback, &closedness);
    entry.is_closed = closedness.is_closed;
    entry.used_fallback = used_fallback;
    result.cumulative_volume += entry.volume;
    if (entry.is_closed) {
      ++result.closed_meshlet_count;
    } else {
      ++result.open_meshlet_count;
    }
    result.meshlets.push_back(entry);
  }
  return result;
}

}  // namespace eco_sys_lab_package
