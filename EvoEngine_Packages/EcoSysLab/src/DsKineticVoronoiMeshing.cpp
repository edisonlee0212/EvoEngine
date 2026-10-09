#include "DsKineticVoronoiMeshing.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <exception>
#include <filesystem>
#include <fstream>
#include <functional>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_inverse.hpp>  // for inverse()
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/norm.hpp>  // for length2()
#include <iomanip>
#include <limits>
#include <numeric>
#include <optional>
#include <queue>
#include <sstream>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>
#include "Application.hpp"
#include "BufferExporter.hpp"
#include "ComputePipeline.hpp"
#include "DsConstraints.hpp"
#include "DsIntersectionBoundaryMesh.hpp"
#include "DsIntersectionBoundaryMeshGroup.hpp"
#include "DsKineticVoronoiVolumeUtils.hpp"
#include "DynamicStrands.hpp"
#include "DynamicStrandsDemo.hpp"
#include "DynamicStrandsInitializationParameters.hpp"
#include "DynamicTreeStrands.hpp"
#include "EcoSysLabPaths.hpp"
#include "EditorDialogBridge.hpp"
#include "Jobs.hpp"
#include "KineticMeshCachePipeline.hpp"
#include "MeshRenderer.hpp"
#include "Platform/Platform.hpp"
#include "ProgressBar.hpp"
#include "ProjectManager.hpp"
#include "Shader.hpp"
#include "StrandModelMeshGenerator.hpp"
#include "Transform.hpp"
#include "Vertex.hpp"
#include "imgui.h"
#include "kinDS/kinDS/KineticDelaunay.hpp"
#include "kinDS/kinDS/MeshIntersection.hpp"
#include "kinDS/kinDS/MeshingBuffer.hpp"
#include "kinDS/kinDS/ObjExporter.hpp"
#include "kinDS/kinDS/Polynomial.hpp"
#include "kinDS/kinDS/SegmentBuilder.hpp"
#include "kinDS/kinDS/Statistics.hpp"
#include "kinDS/kinDS/StrandTree.hpp"

using namespace eco_sys_lab_package;

namespace {
std::shared_ptr<GraphicsPipeline> CreateMaskedRawPipeline(const std::shared_ptr<GraphicsPipeline>& opaque,
                                                          const std::filesystem::path& fragment_shader_path) {
  auto pipeline = std::make_shared<GraphicsPipeline>();
  pipeline->vertex_shader = opaque->vertex_shader;
  pipeline->task_shader = opaque->task_shader;
  pipeline->mesh_shader = opaque->mesh_shader;
  pipeline->fragment_shader =
      Shader::CreateTemporary(ShaderType::Fragment, Platform::GetShaderGlobalDefines(), fragment_shader_path);
  pipeline->geometry_type = opaque->geometry_type;
  pipeline->vertex_input_attribute_set = opaque->vertex_input_attribute_set;
  pipeline->vertex_input_enabled = opaque->vertex_input_enabled;
  pipeline->primitive_topology = opaque->primitive_topology;
  pipeline->descriptor_set_layouts = opaque->descriptor_set_layouts;
  pipeline->color_attachment_formats = opaque->color_attachment_formats;
  pipeline->depth_attachment_format = opaque->depth_attachment_format;
  pipeline->stencil_attachment_format = opaque->stencil_attachment_format;
  pipeline->push_constant_ranges = opaque->push_constant_ranges;
  pipeline->Initialize();
  return pipeline;
}
}  // namespace

namespace {

using GpuMeshletVertex = DsKineticVoronoiMeshing::GpuSegmentMeshletVertex;
using GpuMeshletTriangle = DsKineticVoronoiMeshing::GpuSegmentMeshletTriangle;

constexpr uint32_t kMeshingBufferVersion = kinDS::meshingBufferVersion();

constexpr int kColorNeighborConnectivity = 4;

std::function<void(size_t, std::function<void(size_t)>)> MakeKineticParallelFor() {
  const size_t workers = DsKineticVoronoiMeshing::meshing_settings.kinds_parallel_for_workers;
  return [workers](size_t count, std::function<void(size_t)> func) {
    if (workers == 1) {
      for (size_t i = 0; i < count; ++i) {
        func(i);
      }
      return;
    }
    Jobs::RunParallelFor(
        count,
        [&](size_t i) {
          func(i);
        },
        workers);
  };
}

int EffectiveSegmentMeshletColorMode(const DsKineticVoronoiMeshing::SegmentMeshletsRenderParameters& parameters) {
  if (parameters.debug_neighbor_connectivity) {
    return kColorNeighborConnectivity;
  }
  return parameters.color_mode;
}

/// Force neighbor == -2 on faces whose VoronoiMesh material is bark (brown / light_blue / bark).
/// Used after cache load so remeshed bark tags apply even when GPU neighbors were baked as -1.
void RepairBarkNeighborTagsFromMaterials(std::vector<kinDS::VoronoiMesh>& meshlets,
                                         std::vector<std::vector<int>>& neighbors) {
  const size_t count = std::min(meshlets.size(), neighbors.size());
  for (size_t mesh_index = 0; mesh_index < count; ++mesh_index) {
    const auto& mesh = meshlets[mesh_index];
    const auto& material_ids = mesh.getMaterialIDs();
    const auto& material_names = mesh.getMaterialNames();
    auto& face_neighbors = neighbors[mesh_index];
    const size_t tri_count = mesh.getTriangleCount();
    if (face_neighbors.size() < tri_count) {
      face_neighbors.resize(tri_count, -1);
    }
    for (size_t tri = 0; tri < tri_count && tri < material_ids.size(); ++tri) {
      const int material_id = material_ids[tri];
      bool is_bark = false;
      if (material_id >= 0 && static_cast<size_t>(material_id) < material_names.size()) {
        const std::string& name = material_names[static_cast<size_t>(material_id)];
        is_bark = (name == "brown" || name == "light_blue" || name == "bark");
      } else if (material_id == kinDS::SegmentBuilder::BoundaryIntervalMeshletMaterialId ||
                 material_id == kinDS::SegmentBuilder::PendingSplitFallbackMeshletMaterialId) {
        is_bark = true;
      }
      if (is_bark) {
        face_neighbors[tri] = -2;
      }
    }
  }
}

/// Per-meshlet bark flag from face neighbor tags (-2 = bark / exterior).
std::vector<uint8_t> BuildMeshletHasBarkFlags(const std::vector<std::vector<int>>& meshing_neighbor_indices) {
  std::vector<uint8_t> has_bark(meshing_neighbor_indices.size(), 0);
  for (size_t meshlet_id = 0; meshlet_id < meshing_neighbor_indices.size(); ++meshlet_id) {
    for (const int neighbor : meshing_neighbor_indices[meshlet_id]) {
      if (neighbor == -2) {
        has_bark[meshlet_id] = 1;
        break;
      }
    }
  }
  return has_bark;
}

struct ExactPositionKey {
  uint64_t x = 0;
  uint64_t y = 0;
  uint64_t z = 0;

  static ExactPositionKey From(const glm::dvec3& v) {
    ExactPositionKey key;
    std::memcpy(&key.x, &v.x, sizeof(double));
    std::memcpy(&key.y, &v.y, sizeof(double));
    std::memcpy(&key.z, &v.z, sizeof(double));
    return key;
  }

  bool operator==(const ExactPositionKey& other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct ExactPositionKeyHash {
  size_t operator()(const ExactPositionKey& key) const {
    size_t h = static_cast<size_t>(key.x);
    h ^= static_cast<size_t>(key.y) + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2);
    h ^= static_cast<size_t>(key.z) + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2);
    return h;
  }
};

class BarkSeamUnionFind {
 public:
  explicit BarkSeamUnionFind(const size_t count) : parent_(count), rank_(count, 0) {
    std::iota(parent_.begin(), parent_.end(), size_t{0});
  }

  size_t Find(size_t i) {
    while (parent_[i] != i) {
      parent_[i] = parent_[parent_[i]];
      i = parent_[i];
    }
    return i;
  }

  void Unite(size_t a, size_t b) {
    a = Find(a);
    b = Find(b);
    if (a == b) {
      return;
    }
    if (rank_[a] < rank_[b]) {
      std::swap(a, b);
    }
    parent_[b] = a;
    if (rank_[a] == rank_[b]) {
      ++rank_[a];
    }
  }

 private:
  std::vector<size_t> parent_;
  std::vector<size_t> rank_;
};

bool ArePhysicsSegmentsConnected(const int segment_a, const int segment_b,
                                 const std::vector<DynamicStrands::GpuSegment>& segments,
                                 const std::vector<DynamicStrands::GpuSegmentPair>& segment_pairs,
                                 const std::vector<DynamicStrands::GpuSegmentData>& segment_data_list) {
  if (segment_a == segment_b) {
    return true;
  }
  if (segment_a < 0 || segment_b < 0 || static_cast<size_t>(segment_a) >= segments.size() ||
      static_cast<size_t>(segment_b) >= segments.size()) {
    return false;
  }

  const auto& seg_a = segments[static_cast<size_t>(segment_a)];
  const auto& seg_b = segments[static_cast<size_t>(segment_b)];
  const bool strand_adjacent = seg_a.prev_handle == segment_b || seg_a.next_handle == segment_b ||
                               seg_b.prev_handle == segment_a || seg_b.next_handle == segment_a;

  // Strand neighbors are always treated as connected for bark-seam averaging at construction time.
  if (strand_adjacent) {
    return true;
  }

  if (static_cast<size_t>(segment_a) >= segment_data_list.size() ||
      static_cast<size_t>(segment_b) >= segment_data_list.size()) {
    return false;
  }

  const auto& data_a = segment_data_list[static_cast<size_t>(segment_a)];
  for (const int pair_handle : data_a.pair_handles) {
    if (pair_handle < 0 || static_cast<size_t>(pair_handle) >= segment_pairs.size()) {
      continue;
    }
    const auto& pair = segment_pairs[static_cast<size_t>(pair_handle)];
    const int other = (pair.segment0_handle == segment_a)   ? pair.segment1_handle
                      : (pair.segment1_handle == segment_a) ? pair.segment0_handle
                                                            : -1;
    if (other != segment_b) {
      continue;
    }
    return pair.bend_twist_bundle_integrity > 0.f;
  }
  return false;
}

/// True when a bark face is a Delaunay triangle-cap fan (light_blue / pending-split fallback material).
bool IsTriangleCapBarkFace(const kinDS::VoronoiMesh& mesh, size_t triangle_index) {
  const auto& material_ids = mesh.getMaterialIDs();
  if (triangle_index >= material_ids.size()) {
    return false;
  }
  const int material_id = material_ids[triangle_index];
  const auto& names = mesh.getMaterialNames();
  if (material_id >= 0 && static_cast<size_t>(material_id) < names.size()) {
    return names[static_cast<size_t>(material_id)] == "light_blue";
  }
  return material_id == kinDS::SegmentBuilder::PendingSplitFallbackMeshletMaterialId;
}

/// Average bark corner normals at vertices shared across neighboring segment meshlets, and at
/// multi-face vertices within a single meshlet (bark often comes from several glued intersection pieces).
/// Uses exact position + intact segment-pair / strand links for cross-segment groups. Triangle-cap
/// (light_blue) corners on boundary verts (shared with regular bark) contribute to the average and
/// receive it; cap-only interior corners are left alone here and filled by barycentric interpolation.
/// Runs on CPU meshlets before GPU upload.
void AverageSharedBarkSeamNormals(std::vector<kinDS::VoronoiMesh>& meshes,
                                  const std::vector<std::vector<int>>& meshing_neighbor_indices,
                                  const std::vector<uint8_t>& meshlet_has_bark,
                                  const std::vector<size_t>& meshing_to_physics_segment_indices,
                                  const std::vector<DynamicStrands::GpuSegment>& segments,
                                  const std::vector<DynamicStrands::GpuSegmentPair>& segment_pairs,
                                  const std::vector<DynamicStrands::GpuSegmentData>& segment_data_list) {
  struct BarkCornerRef {
    size_t meshlet_id = 0;
    size_t triangle_index = 0;
    int corner = 0;
    bool from_triangle_cap = false;
  };

  // One entry per (meshlet, local vertex) that participates in at least one bark corner.
  struct BarkVertex {
    size_t meshlet_id = 0;
    size_t local_vertex = 0;
    int physics_segment = -1;
    ExactPositionKey position{};
    std::vector<BarkCornerRef> corners;
    bool on_regular_bark = false;
    bool on_triangle_cap = false;
  };

  size_t bark_meshlet_count = 0;
  for (const uint8_t flag : meshlet_has_bark) {
    bark_meshlet_count += flag ? 1 : 0;
  }

  std::vector<BarkVertex> bark_vertices;
  bark_vertices.reserve(meshes.size() * 8);

  std::unordered_map<uint64_t, size_t> bark_vertex_index;
  const auto pack_key = [](size_t meshlet_id, size_t local_vertex) -> uint64_t {
    return (static_cast<uint64_t>(meshlet_id) << 32) | static_cast<uint64_t>(local_vertex);
  };

  for (size_t meshlet_id = 0; meshlet_id < meshes.size(); ++meshlet_id) {
    if (meshlet_id >= meshlet_has_bark.size() || !meshlet_has_bark[meshlet_id]) {
      continue;
    }
    auto& mesh = meshes[meshlet_id];
    if (mesh.getNormalMode() != kinDS::NormalMode::PerTriangleCorner) {
      continue;
    }
    if (mesh.getNormals().size() != mesh.getTriangles().size()) {
      EVOENGINE_WARNING("Bark seam normals: meshlet " << meshlet_id << " has " << mesh.getNormals().size()
                                                      << " normals for " << mesh.getTriangles().size()
                                                      << " corners; skipping.");
      continue;
    }
    if (meshlet_id >= meshing_neighbor_indices.size()) {
      continue;
    }
    const int physics_segment = meshlet_id < meshing_to_physics_segment_indices.size()
                                    ? static_cast<int>(meshing_to_physics_segment_indices[meshlet_id])
                                    : -1;
    if (physics_segment < 0) {
      continue;
    }

    const auto& face_neighbors = meshing_neighbor_indices[meshlet_id];
    const auto& triangles = mesh.getTriangles();
    const auto& vertices = mesh.getVertices();
    const size_t tri_count = mesh.getTriangleCount();
    for (size_t tri = 0; tri < tri_count; ++tri) {
      if (tri >= face_neighbors.size() || face_neighbors[tri] != -2) {
        continue;
      }
      const bool is_cap = IsTriangleCapBarkFace(mesh, tri);
      for (int corner = 0; corner < 3; ++corner) {
        const size_t local_vertex = triangles[3 * tri + static_cast<size_t>(corner)];
        if (local_vertex >= vertices.size()) {
          continue;
        }
        const uint64_t key = pack_key(meshlet_id, local_vertex);
        auto it = bark_vertex_index.find(key);
        if (it == bark_vertex_index.end()) {
          BarkVertex bv;
          bv.meshlet_id = meshlet_id;
          bv.local_vertex = local_vertex;
          bv.physics_segment = physics_segment;
          bv.position = ExactPositionKey::From(vertices[local_vertex]);
          bv.corners.push_back(BarkCornerRef{meshlet_id, tri, corner, is_cap});
          bv.on_regular_bark = !is_cap;
          bv.on_triangle_cap = is_cap;
          bark_vertex_index.emplace(key, bark_vertices.size());
          bark_vertices.push_back(std::move(bv));
        } else {
          BarkVertex& bv = bark_vertices[it->second];
          bv.corners.push_back(BarkCornerRef{meshlet_id, tri, corner, is_cap});
          bv.on_regular_bark = bv.on_regular_bark || !is_cap;
          bv.on_triangle_cap = bv.on_triangle_cap || is_cap;
        }
      }
    }
  }

  // Positions that appear on regular (non-cap) bark — used to spot cap-only interior verts.
  std::unordered_set<ExactPositionKey, ExactPositionKeyHash> regular_bark_positions;
  regular_bark_positions.reserve(bark_vertices.size());
  for (const BarkVertex& bv : bark_vertices) {
    if (bv.on_regular_bark) {
      regular_bark_positions.insert(bv.position);
    }
  }

  size_t co_located_position_groups = 0;
  size_t shared_bark_vertices = 0;
  size_t shared_seam_groups = 0;
  size_t corners_updated = 0;
  size_t corners_failed = 0;
  size_t regular_source_corners = 0;
  size_t cap_corners_overwritten = 0;
  size_t cap_interior_corners_interpolated = 0;

  if (!bark_vertices.empty()) {
    std::unordered_map<ExactPositionKey, std::vector<size_t>, ExactPositionKeyHash> groups_by_position;
    groups_by_position.reserve(bark_vertices.size());
    for (size_t i = 0; i < bark_vertices.size(); ++i) {
      groups_by_position[bark_vertices[i].position].push_back(i);
    }

    auto average_and_write_component = [&](const std::vector<size_t>& component) {
      bool has_regular_source = false;
      for (const size_t bark_vertex_index : component) {
        has_regular_source = has_regular_source || bark_vertices[bark_vertex_index].on_regular_bark;
      }
      // Cap-only coincidences (no regular bark) do not define a boundary seam average; interiors
      // keep their face normals until the barycentric pass.
      if (!has_regular_source) {
        return;
      }

      glm::dvec3 sum_normal(0.0);
      size_t corner_count = 0;
      for (const size_t bark_vertex_index : component) {
        for (const BarkCornerRef& ref : bark_vertices[bark_vertex_index].corners) {
          // Boundary verts: include both regular bark and triangle-cap corners in the average.
          const size_t corner_index = 3 * ref.triangle_index + static_cast<size_t>(ref.corner);
          sum_normal += meshes[ref.meshlet_id].getNormals()[corner_index];
          ++corner_count;
          ++regular_source_corners;
        }
      }
      if (corner_count == 0) {
        return;
      }
      if (corner_count < 2) {
        return;
      }
      const double length = glm::length(sum_normal);
      if (length <= 1.0e-16) {
        return;
      }
      const glm::dvec3 averaged = sum_normal / length;

      ++shared_seam_groups;
      shared_bark_vertices += component.size();

      // Write the averaged normal to every bark face corner at these boundary verts (regular + cap).
      // Cap-only interior verts are not in this component and keep their normals for interpolation.
      for (const size_t bark_vertex_index : component) {
        const BarkVertex& bark_vertex = bark_vertices[bark_vertex_index];
        if (bark_vertex.meshlet_id >= meshes.size() || bark_vertex.meshlet_id >= meshing_neighbor_indices.size()) {
          ++corners_failed;
          continue;
        }
        kinDS::VoronoiMesh& mesh = meshes[bark_vertex.meshlet_id];
        const auto& face_neighbors = meshing_neighbor_indices[bark_vertex.meshlet_id];
        const auto& triangles = mesh.getTriangles();
        if (mesh.getNormalMode() != kinDS::NormalMode::PerTriangleCorner ||
            mesh.getNormals().size() != triangles.size()) {
          ++corners_failed;
          continue;
        }
        const size_t tri_count = mesh.getTriangleCount();
        for (size_t tri = 0; tri < tri_count; ++tri) {
          if (tri >= face_neighbors.size() || face_neighbors[tri] != -2) {
            continue;
          }
          const bool is_cap = IsTriangleCapBarkFace(mesh, tri);
          for (int corner = 0; corner < 3; ++corner) {
            const size_t corner_index = 3 * tri + static_cast<size_t>(corner);
            if (triangles[corner_index] != bark_vertex.local_vertex) {
              continue;
            }
            if (corner_index >= mesh.getNormals().size()) {
              ++corners_failed;
              continue;
            }
            mesh.setNormal(averaged, corner_index);
            ++corners_updated;
            if (is_cap) {
              ++cap_corners_overwritten;
            }
          }
        }
      }
    };

    for (auto& [position_key, member_indices] : groups_by_position) {
      (void)position_key;
      const size_t member_count = member_indices.size();
      if (member_count == 0) {
        continue;
      }
      ++co_located_position_groups;

      // Single (meshlet, local vertex) with multiple bark faces — common after gluing several
      // intersection pieces into one segment meshlet. No cross-segment requirement.
      if (member_count == 1) {
        average_and_write_component(member_indices);
        continue;
      }

      BarkSeamUnionFind uf(member_count);
      for (size_t i = 0; i < member_count; ++i) {
        const int segment_i = bark_vertices[member_indices[i]].physics_segment;
        for (size_t j = i + 1; j < member_count; ++j) {
          const int segment_j = bark_vertices[member_indices[j]].physics_segment;
          // Same meshlet/segment: always unite (intra-meshlet welded verts).
          // Distinct segments: unite only when physics connectivity is intact.
          if (segment_i == segment_j ||
              ArePhysicsSegmentsConnected(segment_i, segment_j, segments, segment_pairs, segment_data_list)) {
            uf.Unite(i, j);
          }
        }
      }

      std::unordered_map<size_t, std::vector<size_t>> components;
      components.reserve(member_count);
      for (size_t i = 0; i < member_count; ++i) {
        components[uf.Find(i)].push_back(member_indices[i]);
      }

      for (const auto& [root, component] : components) {
        (void)root;
        average_and_write_component(component);
      }
    }
  }

  // Cap-only interior corners: barycentric blend of the triangle's three corner normals after seam writes.
  for (size_t meshlet_id = 0; meshlet_id < meshes.size(); ++meshlet_id) {
    if (meshlet_id >= meshlet_has_bark.size() || !meshlet_has_bark[meshlet_id]) {
      continue;
    }
    if (meshlet_id >= meshing_neighbor_indices.size()) {
      continue;
    }
    kinDS::VoronoiMesh& mesh = meshes[meshlet_id];
    if (mesh.getNormalMode() != kinDS::NormalMode::PerTriangleCorner ||
        mesh.getNormals().size() != mesh.getTriangles().size()) {
      continue;
    }
    const auto& face_neighbors = meshing_neighbor_indices[meshlet_id];
    const auto& triangles = mesh.getTriangles();
    const auto& vertices = mesh.getVertices();
    const size_t tri_count = mesh.getTriangleCount();
    for (size_t tri = 0; tri < tri_count; ++tri) {
      if (tri >= face_neighbors.size() || face_neighbors[tri] != -2) {
        continue;
      }
      if (!IsTriangleCapBarkFace(mesh, tri)) {
        continue;
      }

      size_t corner_vids[3];
      glm::dvec3 corner_normals[3];
      bool corner_is_interior[3] = {false, false, false};
      bool any_interior = false;
      bool valid = true;
      for (int corner = 0; corner < 3; ++corner) {
        const size_t corner_index = 3 * tri + static_cast<size_t>(corner);
        const size_t local_vertex = triangles[corner_index];
        if (local_vertex >= vertices.size()) {
          valid = false;
          break;
        }
        corner_vids[corner] = local_vertex;
        corner_normals[corner] = mesh.getNormals()[corner_index];
        const ExactPositionKey pos = ExactPositionKey::From(vertices[local_vertex]);
        // Interior to the cap: not shared with any regular bark face (even on another meshlet).
        corner_is_interior[corner] = regular_bark_positions.find(pos) == regular_bark_positions.end();
        any_interior = any_interior || corner_is_interior[corner];
      }
      if (!valid || !any_interior) {
        continue;
      }

      const glm::dvec3& p0 = vertices[corner_vids[0]];
      const glm::dvec3& p1 = vertices[corner_vids[1]];
      const glm::dvec3& p2 = vertices[corner_vids[2]];
      const glm::dvec3 e1 = p1 - p0;
      const glm::dvec3 e2 = p2 - p0;
      const double area2 = glm::length(glm::cross(e1, e2));
      if (area2 <= 1.0e-16) {
        continue;
      }

      // Control normals: rim (shared with regular bark) keep post-average values. Interior control
      // slots temporarily use the average of rim controls so a barycentric eval at an interior
      // corner mixes the updated rim normals instead of returning the stale face normal.
      glm::dvec3 rim_sum(0.0);
      size_t rim_count = 0;
      for (int corner = 0; corner < 3; ++corner) {
        if (!corner_is_interior[corner]) {
          rim_sum += corner_normals[corner];
          ++rim_count;
        }
      }
      glm::dvec3 face_n = glm::cross(e1, e2);
      const double face_len = glm::length(face_n);
      if (face_len > 1.0e-16) {
        face_n /= face_len;
      } else {
        face_n = glm::dvec3(0.0, 1.0, 0.0);
      }
      const glm::dvec3 rim_avg = rim_count > 0 ? (rim_sum / static_cast<double>(rim_count)) : face_n;

      glm::dvec3 control[3];
      for (int corner = 0; corner < 3; ++corner) {
        control[corner] = corner_is_interior[corner] ? rim_avg : corner_normals[corner];
      }

      for (int corner = 0; corner < 3; ++corner) {
        if (!corner_is_interior[corner]) {
          continue;
        }
        const glm::dvec3& pi = vertices[corner_vids[corner]];
        // Areal barycentric weights of pi in triangle (p0,p1,p2).
        const double w0 = glm::length(glm::cross(p1 - pi, p2 - pi)) / area2;
        const double w1 = glm::length(glm::cross(p2 - pi, p0 - pi)) / area2;
        const double w2 = glm::length(glm::cross(p0 - pi, p1 - pi)) / area2;
        glm::dvec3 blended = w0 * control[0] + w1 * control[1] + w2 * control[2];
        const double blended_len = glm::length(blended);
        if (blended_len > 1.0e-16) {
          blended /= blended_len;
        } else {
          blended = face_n;
        }
        mesh.setNormal(blended, 3 * tri + static_cast<size_t>(corner));
        ++cap_interior_corners_interpolated;
      }
    }
  }

  EVOENGINE_LOG("Bark seam normals: bark_meshlets="
                << bark_meshlet_count << ", bark_vertices=" << bark_vertices.size()
                << ", co_located_groups=" << co_located_position_groups << ", shared_seam_groups=" << shared_seam_groups
                << ", shared_bark_vertices=" << shared_bark_vertices << ", corners_updated=" << corners_updated
                << ", corners_failed=" << corners_failed << ", regular_source_corners=" << regular_source_corners
                << ", cap_corners_overwritten=" << cap_corners_overwritten
                << ", cap_interior_corners_interpolated=" << cap_interior_corners_interpolated
                << ", segments=" << segments.size() << ", pairs=" << segment_pairs.size());
}

/// Reduce stretch on Delaunay triangle-cap bark UVs (light_blue): among each cap triangle's three
/// corners, the median-circumference vertex gets height += 1 (per-corner, so regular bark is unchanged).
/// Cap-only interior corners then get areal barycentric blends of those (adjusted) corner UVs.
void FixTriangleCapBarkUVs(std::vector<kinDS::VoronoiMesh>& meshes,
                           const std::vector<std::vector<int>>& meshing_neighbor_indices,
                           const std::vector<uint8_t>& meshlet_has_bark) {
  // Positions on regular (non-cap) bark — rim vs interior classification, same as normals.
  std::unordered_set<ExactPositionKey, ExactPositionKeyHash> regular_bark_positions;
  for (size_t meshlet_id = 0; meshlet_id < meshes.size(); ++meshlet_id) {
    if (meshlet_id >= meshlet_has_bark.size() || !meshlet_has_bark[meshlet_id]) {
      continue;
    }
    if (meshlet_id >= meshing_neighbor_indices.size()) {
      continue;
    }
    const auto& mesh = meshes[meshlet_id];
    const auto& face_neighbors = meshing_neighbor_indices[meshlet_id];
    const auto& triangles = mesh.getTriangles();
    const auto& vertices = mesh.getVertices();
    const size_t tri_count = mesh.getTriangleCount();
    for (size_t tri = 0; tri < tri_count; ++tri) {
      if (tri >= face_neighbors.size() || face_neighbors[tri] != -2) {
        continue;
      }
      if (IsTriangleCapBarkFace(mesh, tri)) {
        continue;
      }
      for (int corner = 0; corner < 3; ++corner) {
        const size_t local_vertex = triangles[3 * tri + static_cast<size_t>(corner)];
        if (local_vertex < vertices.size()) {
          regular_bark_positions.insert(ExactPositionKey::From(vertices[local_vertex]));
        }
      }
    }
  }

  size_t cap_triangles_adjusted = 0;
  size_t middle_height_bumps = 0;
  size_t interior_uvs_interpolated = 0;
  size_t corners_failed = 0;

  for (size_t meshlet_id = 0; meshlet_id < meshes.size(); ++meshlet_id) {
    if (meshlet_id >= meshlet_has_bark.size() || !meshlet_has_bark[meshlet_id]) {
      continue;
    }
    if (meshlet_id >= meshing_neighbor_indices.size()) {
      continue;
    }
    kinDS::VoronoiMesh& mesh = meshes[meshlet_id];
    const auto& face_neighbors = meshing_neighbor_indices[meshlet_id];
    const auto& triangles = mesh.getTriangles();
    const auto& vertices = mesh.getVertices();
    const size_t tri_count = mesh.getTriangleCount();
    for (size_t tri = 0; tri < tri_count; ++tri) {
      if (tri >= face_neighbors.size() || face_neighbors[tri] != -2) {
        continue;
      }
      if (!IsTriangleCapBarkFace(mesh, tri)) {
        continue;
      }

      size_t corner_vids[3];
      glm::dvec3 corner_uvs[3];
      bool corner_is_interior[3] = {false, false, false};
      bool valid = true;
      for (int corner = 0; corner < 3; ++corner) {
        const size_t corner_index = 3 * tri + static_cast<size_t>(corner);
        if (!mesh.hasValidUVIndex(corner_index)) {
          valid = false;
          break;
        }
        const size_t local_vertex = triangles[corner_index];
        if (local_vertex >= vertices.size()) {
          valid = false;
          break;
        }
        corner_vids[corner] = local_vertex;
        corner_uvs[corner] = mesh.getUV(corner_index);
        const ExactPositionKey pos = ExactPositionKey::From(vertices[local_vertex]);
        corner_is_interior[corner] = regular_bark_positions.find(pos) == regular_bark_positions.end();
      }
      if (!valid) {
        ++corners_failed;
        continue;
      }

      // Median circumference among the three corners (UV.x = circum, UV.y = height).
      int middle = 0;
      const double c0 = corner_uvs[0].x;
      const double c1 = corner_uvs[1].x;
      const double c2 = corner_uvs[2].x;
      if ((c0 <= c1 && c0 >= c2) || (c0 >= c1 && c0 <= c2)) {
        middle = 0;
      } else if ((c1 <= c0 && c1 >= c2) || (c1 >= c0 && c1 <= c2)) {
        middle = 1;
      } else {
        middle = 2;
      }
      corner_uvs[middle].y += 1.0;
      ++middle_height_bumps;

      const glm::dvec3& p0 = vertices[corner_vids[0]];
      const glm::dvec3& p1 = vertices[corner_vids[1]];
      const glm::dvec3& p2 = vertices[corner_vids[2]];
      const glm::dvec3 e1 = p1 - p0;
      const glm::dvec3 e2 = p2 - p0;
      const double area2 = glm::length(glm::cross(e1, e2));

      for (int corner = 0; corner < 3; ++corner) {
        const size_t corner_index = 3 * tri + static_cast<size_t>(corner);
        if (!corner_is_interior[corner]) {
          // Rim (and the middle-circum vertex): write adjusted corner UV on this cap face only.
          mesh.setUV(corner_uvs[corner], corner_index);
          continue;
        }
        if (area2 <= 1.0e-16) {
          mesh.setUV(corner_uvs[corner], corner_index);
          continue;
        }
        const glm::dvec3& pi = vertices[corner_vids[corner]];
        const double w0 = glm::length(glm::cross(p1 - pi, p2 - pi)) / area2;
        const double w1 = glm::length(glm::cross(p2 - pi, p0 - pi)) / area2;
        const double w2 = glm::length(glm::cross(p0 - pi, p1 - pi)) / area2;
        mesh.setUV(w0 * corner_uvs[0] + w1 * corner_uvs[1] + w2 * corner_uvs[2], corner_index);
        ++interior_uvs_interpolated;
      }
      ++cap_triangles_adjusted;
    }
  }

  EVOENGINE_LOG("Triangle-cap bark UVs: cap_tris_adjusted="
                << cap_triangles_adjusted << ", middle_height_bumps=" << middle_height_bumps
                << ", interior_uvs_interpolated=" << interior_uvs_interpolated
                << ", corners_failed=" << corners_failed);
}

/// Optional 1->4 bark subdivision during prepare: each bark triangle (neighbor == -2) is replaced by
/// four children using edge midpoints. Runs whenever enabled, including when bark smooth iterations
/// are 0. Every bark edge midpoint is also inserted into all adjacent non-bark triangles that still
/// own that undirected edge (typically one interior face on the same meshlet; at segment interfaces,
/// also the neighboring meshlet's interior face).
///
/// Acceleration: one pass builds a geometric edge -> incident-face map (and per-meshlet position ->
/// vertex maps). Propagation then touches only the 0-2 non-bark faces stored for each bark edge
/// instead of scanning every triangle in every meshlet.
void SubdivideBarkTrianglesOnce(std::vector<kinDS::VoronoiMesh>& meshes,
                                std::vector<std::vector<int>>& meshing_neighbor_indices) {
  if (meshes.empty() || meshing_neighbor_indices.size() != meshes.size()) {
    return;
  }

  struct BarkFace {
    size_t meshlet_id = 0;
    size_t tri = 0;
    size_t v[3] = {};
  };
  struct FaceEdgeRef {
    size_t meshlet_id = 0;
    size_t tri = 0;
    size_t v0 = 0;
    size_t v1 = 0;
    bool is_bark = false;
  };
  struct EdgeKey {
    ExactPositionKey a{};
    ExactPositionKey b{};
    bool operator==(const EdgeKey& o) const {
      return a == o.a && b == o.b;
    }
  };
  struct EdgeKeyHash {
    size_t operator()(const EdgeKey& e) const {
      ExactPositionKeyHash h;
      size_t r = h(e.a);
      r ^= h(e.b) + 0x9e3779b97f4a7c15ull + (r << 6) + (r >> 2);
      return r;
    }
  };
  auto make_edge_key = [](const ExactPositionKey& p0, const ExactPositionKey& p1) -> EdgeKey {
    if (p0.x < p1.x || (p0.x == p1.x && (p0.y < p1.y || (p0.y == p1.y && p0.z < p1.z)))) {
      return EdgeKey{p0, p1};
    }
    return EdgeKey{p1, p0};
  };

  std::vector<BarkFace> bark_faces;
  std::unordered_map<EdgeKey, glm::dvec3, EdgeKeyHash> bark_edge_midpoints;
  std::unordered_map<EdgeKey, std::vector<FaceEdgeRef>, EdgeKeyHash> edge_to_faces;
  bark_faces.reserve(1024);
  bark_edge_midpoints.reserve(2048);
  edge_to_faces.reserve(4096);

  // Per-meshlet position -> local vertex (kept up to date while creating midpoints).
  std::vector<std::unordered_map<ExactPositionKey, size_t, ExactPositionKeyHash>> pos_to_vid(meshes.size());

  for (size_t meshlet_id = 0; meshlet_id < meshes.size(); ++meshlet_id) {
    const auto& neighbors = meshing_neighbor_indices[meshlet_id];
    const auto& mesh = meshes[meshlet_id];
    const auto& tris = mesh.getTriangles();
    const auto& verts = mesh.getVertices();
    auto& pos_map = pos_to_vid[meshlet_id];
    pos_map.reserve(verts.size() * 2);
    for (size_t vi = 0; vi < verts.size(); ++vi) {
      pos_map.emplace(ExactPositionKey::From(verts[vi]), vi);
    }

    const size_t tri_count = mesh.getTriangleCount();
    for (size_t tri = 0; tri < tri_count; ++tri) {
      const bool is_bark = tri < neighbors.size() && neighbors[tri] == -2;
      size_t corner[3];
      bool valid = true;
      for (int c = 0; c < 3; ++c) {
        corner[c] = tris[3 * tri + static_cast<size_t>(c)];
        if (corner[c] >= verts.size()) {
          valid = false;
          break;
        }
      }
      if (!valid) {
        continue;
      }
      if (is_bark) {
        BarkFace face;
        face.meshlet_id = meshlet_id;
        face.tri = tri;
        face.v[0] = corner[0];
        face.v[1] = corner[1];
        face.v[2] = corner[2];
        bark_faces.push_back(face);
      }
      for (int e = 0; e < 3; ++e) {
        const size_t a = corner[e];
        const size_t b = corner[(e + 1) % 3];
        const EdgeKey key = make_edge_key(ExactPositionKey::From(verts[a]), ExactPositionKey::From(verts[b]));
        edge_to_faces[key].push_back(FaceEdgeRef{meshlet_id, tri, a, b, is_bark});
        if (is_bark) {
          bark_edge_midpoints.emplace(key, 0.5 * (verts[a] + verts[b]));
        }
      }
    }
  }

  if (bark_faces.empty() || bark_edge_midpoints.empty()) {
    return;
  }

  // Propagate each bark-edge midpoint only to the non-bark faces already indexed on that edge.
  size_t propagated_splits = 0;
  for (const auto& [edge, midpoint] : bark_edge_midpoints) {
    const auto it = edge_to_faces.find(edge);
    if (it == edge_to_faces.end()) {
      continue;
    }
    for (const FaceEdgeRef& ref : it->second) {
      if (ref.is_bark) {
        continue;
      }
      auto& mesh = meshes[ref.meshlet_id];
      auto& neighbors = meshing_neighbor_indices[ref.meshlet_id];
      const size_t c0 = mesh.triangleCornerIndex(ref.tri, ref.v0);
      const size_t c1 = mesh.triangleCornerIndex(ref.tri, ref.v1);
      if (c0 == static_cast<size_t>(-1) || c1 == static_cast<size_t>(-1)) {
        continue;  // edge already gone (face split by another bark edge)
      }
      const int neighbor_tag = (ref.tri < neighbors.size()) ? neighbors[ref.tri] : -1;
      const auto [new_vid, new_tri] = mesh.splitTriangle(c0, c1, midpoint, "{}");
      if (new_vid == static_cast<size_t>(-1) || new_tri == static_cast<size_t>(-1)) {
        continue;
      }
      pos_to_vid[ref.meshlet_id].emplace(ExactPositionKey::From(midpoint), new_vid);
      if (neighbors.size() < mesh.getTriangleCount()) {
        neighbors.resize(mesh.getTriangleCount(), -1);
      }
      if (new_tri < neighbors.size()) {
        neighbors[new_tri] = neighbor_tag;
      }
      ++propagated_splits;
    }
  }

  auto get_or_create_midpoint = [&](size_t meshlet_id, size_t va, size_t vb, const glm::dvec3& midpoint) -> size_t {
    auto& mesh = meshes[meshlet_id];
    auto& pos_map = pos_to_vid[meshlet_id];
    const ExactPositionKey mid_key = ExactPositionKey::From(midpoint);
    if (const auto found = pos_map.find(mid_key); found != pos_map.end()) {
      return found->second;
    }
    const size_t mid = mesh.addVertex(midpoint, "{}");
    pos_map.emplace(mid_key, mid);
    const double t0 = mesh.vertexKineticTime(va);
    const double t1 = mesh.vertexKineticTime(vb);
    if (std::isfinite(t0) && std::isfinite(t1)) {
      mesh.setVertexKineticTime(mid, 0.5 * (t0 + t1));
    }
    if (const auto uv0 = mesh.vertexSemanticUv(va); uv0.has_value()) {
      if (const auto uv1 = mesh.vertexSemanticUv(vb); uv1.has_value()) {
        mesh.setVertexSemanticUv(mid, 0.5 * (uv0.value() + uv1.value()));
      }
    }
    if (mesh.isVertexFlexible(va) || mesh.isVertexFlexible(vb)) {
      mesh.setVertexFlexible(mid, true);
    }
    return mid;
  };

  // 1->4 rewrite each original bark face.
  size_t subdivided = 0;
  for (const BarkFace& face : bark_faces) {
    auto& mesh = meshes[face.meshlet_id];
    auto& neighbors = meshing_neighbor_indices[face.meshlet_id];
    auto& tris = mesh.getTriangles();
    const auto& verts = mesh.getVertices();
    if (3 * face.tri + 2 >= tris.size()) {
      continue;
    }
    if (tris[3 * face.tri] != face.v[0] || tris[3 * face.tri + 1] != face.v[1] || tris[3 * face.tri + 2] != face.v[2]) {
      continue;
    }
    if (face.v[0] >= verts.size() || face.v[1] >= verts.size() || face.v[2] >= verts.size()) {
      continue;
    }

    const size_t va = face.v[0];
    const size_t vb = face.v[1];
    const size_t vc = face.v[2];
    const glm::dvec3 mid_ab = 0.5 * (verts[va] + verts[vb]);
    const glm::dvec3 mid_bc = 0.5 * (verts[vb] + verts[vc]);
    const glm::dvec3 mid_ca = 0.5 * (verts[vc] + verts[va]);
    const size_t m_ab = get_or_create_midpoint(face.meshlet_id, va, vb, mid_ab);
    const size_t m_bc = get_or_create_midpoint(face.meshlet_id, vb, vc, mid_bc);
    const size_t m_ca = get_or_create_midpoint(face.meshlet_id, vc, va, mid_ca);

    const int material_id = (face.tri < mesh.getMaterialIDs().size()) ? mesh.getMaterialIDs()[face.tri] : -1;
    const std::string face_meta = (mesh.storeMetadata() && face.tri < mesh.getFaceMetadata().size())
                                      ? mesh.getFaceMetadata()[face.tri]
                                      : std::string("{}");

    size_t uv_a = std::numeric_limits<size_t>::max();
    size_t uv_b = std::numeric_limits<size_t>::max();
    size_t uv_c = std::numeric_limits<size_t>::max();
    size_t uv_ab = std::numeric_limits<size_t>::max();
    size_t uv_bc = std::numeric_limits<size_t>::max();
    size_t uv_ca = std::numeric_limits<size_t>::max();
    auto& uv_indices = mesh.getUVIndices();
    const bool has_uvs = uv_indices.size() == tris.size();
    if (has_uvs) {
      uv_a = uv_indices[3 * face.tri];
      uv_b = uv_indices[3 * face.tri + 1];
      uv_c = uv_indices[3 * face.tri + 2];
      const auto& uvs = mesh.getUVs();
      auto mid_uv = [&](size_t i0, size_t i1) -> size_t {
        if (i0 >= uvs.size() || i1 >= uvs.size()) {
          return std::numeric_limits<size_t>::max();
        }
        return mesh.addUV(0.5 * (uvs[i0] + uvs[i1]));
      };
      uv_ab = mid_uv(uv_a, uv_b);
      uv_bc = mid_uv(uv_b, uv_c);
      uv_ca = mid_uv(uv_c, uv_a);
    }

    tris[3 * face.tri] = va;
    tris[3 * face.tri + 1] = m_ab;
    tris[3 * face.tri + 2] = m_ca;
    if (has_uvs) {
      uv_indices[3 * face.tri] = uv_a;
      uv_indices[3 * face.tri + 1] = uv_ab;
      uv_indices[3 * face.tri + 2] = uv_ca;
    }

    const size_t t_b = mesh.addTriangle(m_ab, vb, m_bc, uv_ab, uv_b, uv_bc, material_id, face_meta);
    const size_t t_c = mesh.addTriangle(m_ca, m_bc, vc, uv_ca, uv_bc, uv_c, material_id, face_meta);
    const size_t t_m = mesh.addTriangle(m_ab, m_bc, m_ca, uv_ab, uv_bc, uv_ca, material_id, face_meta);

    if (neighbors.size() < mesh.getTriangleCount()) {
      neighbors.resize(mesh.getTriangleCount(), -1);
    }
    neighbors[face.tri] = -2;
    if (t_b < neighbors.size()) {
      neighbors[t_b] = -2;
    }
    if (t_c < neighbors.size()) {
      neighbors[t_c] = -2;
    }
    if (t_m < neighbors.size()) {
      neighbors[t_m] = -2;
    }
    ++subdivided;
  }

  for (size_t meshlet_id = 0; meshlet_id < meshes.size(); ++meshlet_id) {
    meshes[meshlet_id].mergeDuplicateVertices(0.0);
    if (meshing_neighbor_indices[meshlet_id].size() != meshes[meshlet_id].getTriangleCount()) {
      meshing_neighbor_indices[meshlet_id].resize(meshes[meshlet_id].getTriangleCount(), -1);
    }
    if (meshes[meshlet_id].getNormalMode() != kinDS::NormalMode::NoNormals) {
      meshes[meshlet_id].computeNormals(meshes[meshlet_id].getNormalMode());
    }
  }

  if (subdivided > 0) {
    EVOENGINE_LOG("Bark subdivision: 1->4 split " << subdivided << " bark triangle(s), propagated " << propagated_splits
                                                  << " interior edge split(s) across " << meshes.size()
                                                  << " meshlet(s).");
  }
}

/// Lift circumferential UV.x into a chart continuous with @p reference (period 1.0 wrap from
/// @c adjustedBoundaryTriangleUvs). Height (y) and z are unchanged.
glm::dvec3 LiftBarkUvCircumContinuity(const glm::dvec3& uv, const glm::dvec3& reference, const double period = 1.0) {
  glm::dvec3 lifted = uv;
  if (period > 0.0 && std::isfinite(uv.x) && std::isfinite(reference.x)) {
    lifted.x = uv.x - std::round((uv.x - reference.x) / period) * period;
  }
  return lifted;
}

/// Re-unwrap already-smoothed bark triangle UVs so corners spanning the cut stay locally continuous
/// (same rule as kinDS::adjustedBoundaryTriangleUvs on raw angles, with period 1.0).
void UnwrapBarkTriangleCornerUvs(glm::dvec3& u, glm::dvec3& v, glm::dvec3& w) {
  const double base = u.x;
  v.x -= std::round(v.x - base);
  w.x -= std::round(w.x - base);
}

/// Collect bark triangles across meshlets, Laplacian-smooth with
/// @ref StrandModelMeshGenerator::MeshSmoothing (no ground lock), then write positions back to every
/// meshlet vertex that shares that position (interior faces sharing verts move with bark).
/// Optionally co-smooth bark corner UVs with the same iterations/strength (wrap-aware circum lift).
///
/// Hole / open-boundary notes (same as marching-cubes MeshSmoothing):
/// - Connectivity is undirected 1-ring from bark triangles only; hole rims remain valid if degree >= 1.
/// Uniform Laplacian on the welded bark surface (neighbor == -2 faces).
/// Exact-position welding across meshlets; interior vertices that share those positions move with bark.
/// - Boundary vertices are pulled toward remaining neighbors (rim tends to shrink; holes are not filled).
/// - Degree-0 vertices keep their position (no divide-by-zero).
/// - Unwelded near-duplicates across meshlets will not connect — exact position welding is required.
void SmoothBarkMeshPositions(std::vector<kinDS::VoronoiMesh>& meshes,
                             const std::vector<std::vector<int>>& meshing_neighbor_indices,
                             const std::vector<uint8_t>& meshlet_has_bark, const int iterations, const float strength,
                             const bool lock_boundary, const bool smooth_uvs) {
  if (iterations <= 0 || meshes.empty() || strength <= 0.0f) {
    return;
  }

  struct MeshletVertexRef {
    size_t meshlet_id = 0;
    size_t local_vertex = 0;
  };
  struct BarkFaceRef {
    size_t meshlet_id = 0;
    size_t tri = 0;
    unsigned corner_unified[3] = {0, 0, 0};
    bool has_uvs = false;
  };

  std::vector<glm::dvec3> unified_positions;
  std::vector<std::vector<MeshletVertexRef>> refs_by_unified;
  std::unordered_map<ExactPositionKey, unsigned, ExactPositionKeyHash> position_to_unified;
  std::vector<unsigned> bark_indices;
  std::vector<BarkFaceRef> bark_faces;
  std::vector<uint8_t> meshlet_touched(meshes.size(), 0);

  // Per-unified-vertex UV accumulation (wrap-lifted to the first corner that seeds each vertex).
  std::vector<glm::dvec3> unified_uv_sum;
  std::vector<glm::dvec3> unified_uv_seed;
  std::vector<int> unified_uv_count;

  size_t bark_triangle_count = 0;
  for (size_t meshlet_id = 0; meshlet_id < meshes.size(); ++meshlet_id) {
    if (meshlet_id >= meshlet_has_bark.size() || !meshlet_has_bark[meshlet_id]) {
      continue;
    }
    if (meshlet_id >= meshing_neighbor_indices.size()) {
      continue;
    }
    const auto& mesh = meshes[meshlet_id];
    const auto& face_neighbors = meshing_neighbor_indices[meshlet_id];
    const auto& triangles = mesh.getTriangles();
    const auto& vertices = mesh.getVertices();
    const size_t tri_count = mesh.getTriangleCount();
    for (size_t tri = 0; tri < tri_count; ++tri) {
      if (tri >= face_neighbors.size() || face_neighbors[tri] != -2) {
        continue;
      }
      ++bark_triangle_count;
      unsigned corner_unified[3];
      bool valid = true;
      for (int corner = 0; corner < 3; ++corner) {
        const size_t local_vertex = triangles[3 * tri + static_cast<size_t>(corner)];
        if (local_vertex >= vertices.size()) {
          valid = false;
          break;
        }
        const ExactPositionKey key = ExactPositionKey::From(vertices[local_vertex]);
        auto it = position_to_unified.find(key);
        if (it == position_to_unified.end()) {
          const unsigned unified = static_cast<unsigned>(unified_positions.size());
          position_to_unified.emplace(key, unified);
          unified_positions.push_back(vertices[local_vertex]);
          refs_by_unified.push_back({MeshletVertexRef{meshlet_id, local_vertex}});
          unified_uv_sum.emplace_back(0.0);
          unified_uv_seed.emplace_back(0.0);
          unified_uv_count.push_back(0);
          corner_unified[corner] = unified;
        } else {
          corner_unified[corner] = it->second;
          auto& refs = refs_by_unified[it->second];
          bool already = false;
          for (const MeshletVertexRef& ref : refs) {
            if (ref.meshlet_id == meshlet_id && ref.local_vertex == local_vertex) {
              already = true;
              break;
            }
          }
          if (!already) {
            refs.push_back(MeshletVertexRef{meshlet_id, local_vertex});
          }
        }
      }
      if (!valid) {
        continue;
      }
      bark_indices.push_back(corner_unified[0]);
      bark_indices.push_back(corner_unified[1]);
      bark_indices.push_back(corner_unified[2]);

      BarkFaceRef face;
      face.meshlet_id = meshlet_id;
      face.tri = tri;
      face.corner_unified[0] = corner_unified[0];
      face.corner_unified[1] = corner_unified[1];
      face.corner_unified[2] = corner_unified[2];
      face.has_uvs =
          mesh.hasValidUVIndex(3 * tri) && mesh.hasValidUVIndex(3 * tri + 1) && mesh.hasValidUVIndex(3 * tri + 2);
      if (face.has_uvs && smooth_uvs) {
        for (int corner = 0; corner < 3; ++corner) {
          const unsigned unified = corner_unified[corner];
          const glm::dvec3 corner_uv = mesh.getUV(3 * tri + static_cast<size_t>(corner));
          if (unified_uv_count[unified] == 0) {
            unified_uv_seed[unified] = corner_uv;
            unified_uv_sum[unified] = corner_uv;
            unified_uv_count[unified] = 1;
          } else {
            unified_uv_sum[unified] += LiftBarkUvCircumContinuity(corner_uv, unified_uv_seed[unified]);
            ++unified_uv_count[unified];
          }
        }
      }
      bark_faces.push_back(face);
    }
  }

  if (unified_positions.empty() || bark_indices.size() < 3) {
    EVOENGINE_LOG("Bark mesh smooth: no bark triangles to smooth.");
    return;
  }

  // Bark-manifold boundary: undirected edges that appear in exactly one bark triangle.
  std::vector<uint8_t> is_boundary_vertex(unified_positions.size(), 0);
  size_t boundary_vertex_count = 0;
  if (lock_boundary) {
    struct UndirectedEdge {
      unsigned a = 0;
      unsigned b = 0;
      bool operator==(const UndirectedEdge& o) const {
        return a == o.a && b == o.b;
      }
    };
    struct UndirectedEdgeHash {
      size_t operator()(const UndirectedEdge& e) const {
        size_t h = static_cast<size_t>(e.a);
        h ^= static_cast<size_t>(e.b) + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2);
        return h;
      }
    };
    std::unordered_map<UndirectedEdge, int, UndirectedEdgeHash> edge_face_count;
    edge_face_count.reserve(bark_indices.size());
    for (size_t i = 0; i + 2 < bark_indices.size(); i += 3) {
      const unsigned c0 = bark_indices[i];
      const unsigned c1 = bark_indices[i + 1];
      const unsigned c2 = bark_indices[i + 2];
      auto add_edge = [&](unsigned u, unsigned v) {
        if (u > v) {
          std::swap(u, v);
        }
        ++edge_face_count[UndirectedEdge{u, v}];
      };
      add_edge(c0, c1);
      add_edge(c1, c2);
      add_edge(c2, c0);
    }
    for (const auto& [edge, count] : edge_face_count) {
      if (count != 1) {
        continue;
      }
      for (const unsigned vid : {edge.a, edge.b}) {
        if (vid < is_boundary_vertex.size() && !is_boundary_vertex[vid]) {
          is_boundary_vertex[vid] = 1;
          ++boundary_vertex_count;
        }
      }
    }
  }

  // Also register any non-bark meshlet vertices that share a welded bark position so interior
  // faces sharing those verts are dragged along when we write back.
  for (size_t meshlet_id = 0; meshlet_id < meshes.size(); ++meshlet_id) {
    const auto& vertices = meshes[meshlet_id].getVertices();
    for (size_t local_vertex = 0; local_vertex < vertices.size(); ++local_vertex) {
      const ExactPositionKey key = ExactPositionKey::From(vertices[local_vertex]);
      auto it = position_to_unified.find(key);
      if (it == position_to_unified.end()) {
        continue;
      }
      auto& refs = refs_by_unified[it->second];
      bool already = false;
      for (const MeshletVertexRef& ref : refs) {
        if (ref.meshlet_id == meshlet_id && ref.local_vertex == local_vertex) {
          already = true;
          break;
        }
      }
      if (!already) {
        refs.push_back(MeshletVertexRef{meshlet_id, local_vertex});
      }
    }
  }

  std::vector<Vertex> smooth_vertices(unified_positions.size());
  for (size_t i = 0; i < unified_positions.size(); ++i) {
    smooth_vertices[i].position = glm::vec3(unified_positions[i]);
  }
  std::vector<glm::vec3> locked_positions;
  if (lock_boundary) {
    locked_positions.resize(smooth_vertices.size());
    for (size_t i = 0; i < smooth_vertices.size(); ++i) {
      locked_positions[i] = smooth_vertices[i].position;
    }
  }

  std::vector<glm::dvec3> smooth_uvs_attr(unified_positions.size(), glm::dvec3(0.0));
  std::vector<uint8_t> has_smooth_uv(unified_positions.size(), 0);
  std::vector<glm::dvec3> locked_uvs;
  std::vector<std::vector<unsigned>> uv_connectivity;
  if (smooth_uvs) {
    for (size_t i = 0; i < unified_positions.size(); ++i) {
      if (unified_uv_count[i] > 0) {
        smooth_uvs_attr[i] = unified_uv_sum[i] / static_cast<double>(unified_uv_count[i]);
        has_smooth_uv[i] = 1;
      }
    }
    if (lock_boundary) {
      locked_uvs = smooth_uvs_attr;
    }
    // Same undirected 1-ring as MeshSmoothing (built once for UV iterations).
    uv_connectivity.resize(unified_positions.size());
    for (size_t i = 0; i + 2 < bark_indices.size(); i += 3) {
      const unsigned a = bark_indices[i];
      const unsigned b = bark_indices[i + 1];
      const unsigned c = bark_indices[i + 2];
      auto link = [&](unsigned from, unsigned to) {
        auto& n = uv_connectivity[from];
        for (const unsigned existing : n) {
          if (existing == to) {
            return;
          }
        }
        n.push_back(to);
      };
      link(a, b);
      link(a, c);
      link(b, a);
      link(b, c);
      link(c, a);
      link(c, b);
    }
  }

  const float blend = glm::clamp(strength, 0.0f, 1.0f);
  for (int iter = 0; iter < iterations; ++iter) {
    StrandModelMeshGenerator::MeshSmoothing(smooth_vertices, bark_indices, /*lock_ground_plane=*/false, blend);
    if (lock_boundary) {
      for (size_t i = 0; i < smooth_vertices.size(); ++i) {
        if (is_boundary_vertex[i]) {
          smooth_vertices[i].position = locked_positions[i];
        }
      }
    }

    if (smooth_uvs) {
      std::vector<glm::dvec3> new_uvs = smooth_uvs_attr;
      for (size_t i = 0; i < smooth_uvs_attr.size(); ++i) {
        if (!has_smooth_uv[i] || uv_connectivity[i].empty()) {
          continue;
        }
        if (lock_boundary && is_boundary_vertex[i]) {
          continue;
        }
        glm::dvec3 sum(0.0);
        int count = 0;
        for (const unsigned neighbor : uv_connectivity[i]) {
          if (!has_smooth_uv[neighbor]) {
            continue;
          }
          sum += LiftBarkUvCircumContinuity(smooth_uvs_attr[neighbor], smooth_uvs_attr[i]);
          ++count;
        }
        if (count > 0) {
          const glm::dvec3 avg = sum / static_cast<double>(count);
          new_uvs[i] = glm::mix(smooth_uvs_attr[i], avg, static_cast<double>(blend));
        }
      }
      for (size_t i = 0; i < smooth_uvs_attr.size(); ++i) {
        if (lock_boundary && is_boundary_vertex[i]) {
          smooth_uvs_attr[i] = locked_uvs[i];
        } else if (has_smooth_uv[i]) {
          smooth_uvs_attr[i] = new_uvs[i];
        }
      }
    }
  }

  size_t meshlet_vertices_updated = 0;
  size_t locked_skips = 0;
  for (size_t unified = 0; unified < smooth_vertices.size(); ++unified) {
    if (lock_boundary && is_boundary_vertex[unified]) {
      ++locked_skips;
      continue;  // leave original meshlet positions unchanged
    }
    const glm::dvec3 new_pos(smooth_vertices[unified].position.x, smooth_vertices[unified].position.y,
                             smooth_vertices[unified].position.z);
    for (const MeshletVertexRef& ref : refs_by_unified[unified]) {
      meshes[ref.meshlet_id].replaceVertex(ref.local_vertex, new_pos);
      meshlet_touched[ref.meshlet_id] = 1;
      ++meshlet_vertices_updated;
    }
  }

  size_t bark_uv_faces_written = 0;
  if (smooth_uvs) {
    for (const BarkFaceRef& face : bark_faces) {
      if (!face.has_uvs) {
        continue;
      }
      if (!has_smooth_uv[face.corner_unified[0]] || !has_smooth_uv[face.corner_unified[1]] ||
          !has_smooth_uv[face.corner_unified[2]]) {
        continue;
      }
      glm::dvec3 u = smooth_uvs_attr[face.corner_unified[0]];
      glm::dvec3 v = smooth_uvs_attr[face.corner_unified[1]];
      glm::dvec3 w = smooth_uvs_attr[face.corner_unified[2]];
      UnwrapBarkTriangleCornerUvs(u, v, w);
      auto& mesh = meshes[face.meshlet_id];
      mesh.setUV(u, 3 * face.tri);
      mesh.setUV(v, 3 * face.tri + 1);
      mesh.setUV(w, 3 * face.tri + 2);
      ++bark_uv_faces_written;
    }
  }

  size_t meshlets_recomputed = 0;
  for (size_t meshlet_id = 0; meshlet_id < meshes.size(); ++meshlet_id) {
    if (!meshlet_touched[meshlet_id]) {
      continue;
    }
    if (meshes[meshlet_id].getNormalMode() == kinDS::NormalMode::NoNormals) {
      continue;
    }
    meshes[meshlet_id].computeNormals(kinDS::NormalMode::PerTriangleCorner);
    ++meshlets_recomputed;
  }

  EVOENGINE_LOG("Bark mesh smooth: iterations="
                << iterations << ", strength=" << blend << ", bark_tris=" << bark_triangle_count << ", welded_verts="
                << unified_positions.size() << ", boundary_locked=" << (lock_boundary ? boundary_vertex_count : 0)
                << ", locked_skips=" << locked_skips << ", smooth_uvs=" << (smooth_uvs ? 1 : 0) << ", uv_faces_written="
                << bark_uv_faces_written << ", meshlet_vert_writes=" << meshlet_vertices_updated
                << ", meshlets_recomputed_normals=" << meshlets_recomputed);
}

/// Build a welded bark-only mesh (positions + per-corner normals) in @p root_transform / GPU frame.
kinDS::VoronoiMesh BuildBarkDebugMesh(const std::vector<kinDS::VoronoiMesh>& meshes,
                                      const std::vector<std::vector<int>>& meshing_neighbor_indices,
                                      const std::vector<uint8_t>& meshlet_has_bark,
                                      const GlobalTransform& root_transform) {
  kinDS::VoronoiMesh bark_mesh({"bark"}, kinDS::NormalMode::PerTriangleCorner);
  bark_mesh.setStoreMetadata(false);

  std::unordered_map<ExactPositionKey, size_t, ExactPositionKeyHash> position_to_vertex;
  size_t triangle_count = 0;

  for (size_t meshlet_id = 0; meshlet_id < meshes.size(); ++meshlet_id) {
    if (meshlet_id >= meshlet_has_bark.size() || !meshlet_has_bark[meshlet_id]) {
      continue;
    }
    if (meshlet_id >= meshing_neighbor_indices.size()) {
      continue;
    }
    const auto& mesh = meshes[meshlet_id];
    if (mesh.getNormalMode() != kinDS::NormalMode::PerTriangleCorner ||
        mesh.getNormals().size() != mesh.getTriangles().size()) {
      continue;
    }
    const auto& face_neighbors = meshing_neighbor_indices[meshlet_id];
    const auto& triangles = mesh.getTriangles();
    const auto& vertices = mesh.getVertices();
    const size_t tri_count = mesh.getTriangleCount();
    for (size_t tri = 0; tri < tri_count; ++tri) {
      if (tri >= face_neighbors.size() || face_neighbors[tri] != -2) {
        continue;
      }
      size_t corner_vids[3];
      size_t corner_uvs[3] = {std::numeric_limits<size_t>::max(), std::numeric_limits<size_t>::max(),
                              std::numeric_limits<size_t>::max()};
      glm::dvec3 corner_normals[3];
      bool valid = true;
      for (int corner = 0; corner < 3; ++corner) {
        const size_t corner_index = 3 * tri + static_cast<size_t>(corner);
        const size_t local_vertex = triangles[corner_index];
        if (local_vertex >= vertices.size()) {
          valid = false;
          break;
        }
        const glm::vec3 world_pos = root_transform.TransformPoint(
            glm::vec3(vertices[local_vertex].x, vertices[local_vertex].y, vertices[local_vertex].z));
        const ExactPositionKey key = ExactPositionKey::From(glm::dvec3(world_pos.x, world_pos.y, world_pos.z));
        auto it = position_to_vertex.find(key);
        if (it == position_to_vertex.end()) {
          corner_vids[corner] = bark_mesh.addVertex(glm::dvec3(world_pos.x, world_pos.y, world_pos.z));
          position_to_vertex.emplace(key, corner_vids[corner]);
        } else {
          corner_vids[corner] = it->second;
        }

        const glm::dvec3 local_n = mesh.getNormals()[corner_index];
        const glm::vec3 world_n = root_transform.TransformVector(glm::vec3(local_n.x, local_n.y, local_n.z));
        corner_normals[corner] = glm::dvec3(world_n.x, world_n.y, world_n.z);
        const double n_len = glm::length(corner_normals[corner]);
        if (n_len > 1.0e-16) {
          corner_normals[corner] /= n_len;
        }

        if (mesh.hasValidUVIndex(corner_index)) {
          corner_uvs[corner] = bark_mesh.addUV(mesh.getUV(corner_index));
        }
      }
      if (!valid) {
        continue;
      }
      bark_mesh.addTriangle(corner_vids[0], corner_vids[1], corner_vids[2], corner_uvs[0], corner_uvs[1], corner_uvs[2],
                            /*material_id=*/0);
      bark_mesh.addNormal(corner_normals[0]);
      bark_mesh.addNormal(corner_normals[1]);
      bark_mesh.addNormal(corner_normals[2]);
      ++triangle_count;
    }
  }

  EVOENGINE_LOG("Bark debug mesh: " << triangle_count << " triangles, " << bark_mesh.getVertexCount()
                                    << " welded vertices.");
  return bark_mesh;
}

bool SegmentPairNeedsMaterialInit(const DynamicStrands::GpuSegmentPair& pair) {
  return pair.max_bending_modulus <= 0.f || pair.max_torsion_modulus <= 0.f;
}

void CopySegmentPairMaterialProperties(DynamicStrands::GpuSegmentPair& dst, const DynamicStrands::GpuSegmentPair& src) {
  dst.bending_alpha = src.bending_alpha;
  dst.torsion_alpha = src.torsion_alpha;
  dst.max_bending_modulus = src.max_bending_modulus;
  dst.max_torsion_modulus = src.max_torsion_modulus;
  dst.max_bending_twist_bundle_strain = src.max_bending_twist_bundle_strain;
  dst.bending_twist_bundle_strain_limit = src.bending_twist_bundle_strain_limit;
  dst.max_connectivity_strain = src.max_connectivity_strain;
  dst.connectivity_strain_limit = src.connectivity_strain_limit;
}

void InitializeCompactedSegmentPair(DynamicStrands::GpuSegmentPair& pair, DynamicStrands::GpuSegment& segment0,
                                    DynamicStrands::GpuSegment& segment1, const bool direct_connection,
                                    const DynamicStrandsInitializeParameters& initialize_parameters,
                                    const DynamicStrands::GpuSegmentPair* material_template = nullptr) {
  const glm::vec3 segment0_center = (segment0.particle0.x + segment0.particle1.x) * 0.5f;
  const glm::vec3 segment1_center = (segment1.particle0.x + segment1.particle1.x) * 0.5f;
  pair.segment0_offset = glm::vec4(glm::inverse(segment1.q) * (segment0_center - segment1_center), 0.0f);
  pair.segment1_offset = glm::vec4(glm::inverse(segment0.q) * (segment1_center - segment0_center), 0.0f);
  pair.rest_darboux_vector = glm::conjugate(segment0.q) * segment1.q;

  pair.bending_twist_bundle_strain = glm::vec3(0.f);
  pair.connectivity_strain = 0.f;
  pair.bend_twist_bundle_integrity = 1.f;
  pair.connectivity_integrity = direct_connection ? 1.f : 0.f;
  pair.compression_lock = pair.positional_lock = pair.rotational_lock = pair.tensile_lock = 0;

  if (!SegmentPairNeedsMaterialInit(pair)) {
    return;
  }
  if (material_template != nullptr && !SegmentPairNeedsMaterialInit(*material_template)) {
    CopySegmentPairMaterialProperties(pair, *material_template);
    return;
  }

  const float root_distance = (segment0.particle0.root_distance + segment0.particle1.root_distance +
                               segment1.particle0.root_distance + segment1.particle1.root_distance) *
                              0.25f;
  const float distance_to_boundary = (segment0.boundary_distance + segment1.boundary_distance) * 0.5f;

  BiologicalPropertiesGraph::Input biological_properties_input;
  biological_properties_input.root_distance = root_distance;
  biological_properties_input.polar_distance =
      (segment0.profile_polar_coordinate.x + segment1.profile_polar_coordinate.x) * 0.5f;
  biological_properties_input.polar_angle =
      (segment0.profile_polar_coordinate.y + segment1.profile_polar_coordinate.y) * 0.5f;
  biological_properties_input.profile_boundary_distance = distance_to_boundary;
  const BiologicalPropertiesGraph::Output biological_properties =
      initialize_parameters.biological_properties_graph.GetValues(biological_properties_input);
  const float trunk_strength_factor =
      initialize_parameters.trunk_additional_strength
          ? ActivationFunction::Sigmoid(biological_properties.trunk_additional_strength_factor, 0.f,
                                        biological_properties.trunk_offset,
                                        1.f / biological_properties.trunk_transition, root_distance)
          : 0.f;

  ModulusGraph::Input modulus_graph_input;
  modulus_graph_input.root_distance = root_distance;
  modulus_graph_input.polar_distance = biological_properties_input.polar_distance;
  modulus_graph_input.polar_angle = biological_properties_input.polar_angle;
  modulus_graph_input.profile_boundary_distance = distance_to_boundary;

  const glm::vec2 max_bending_modulus = initialize_parameters.modulus_graph.GetBendingModulus(modulus_graph_input);
  pair.max_bending_modulus =
      glm::max(1e-9f, ActivationFunction::Sigmoid(max_bending_modulus.x, max_bending_modulus.y,
                                                  initialize_parameters.sapwood_offset,
                                                  1.f / initialize_parameters.wood_transition, distance_to_boundary)) *
      1e9f;

  const glm::vec2 max_twisting_modulus = initialize_parameters.modulus_graph.GetTwistingModulus(modulus_graph_input);
  pair.max_torsion_modulus =
      glm::max(1e-9f, ActivationFunction::Sigmoid(max_twisting_modulus.x, max_twisting_modulus.y,
                                                  initialize_parameters.sapwood_offset,
                                                  1.f / initialize_parameters.wood_transition, distance_to_boundary)) *
      1e9f;

  const float segment_radius = (segment0.radius + segment1.radius) * 0.5f;
  const float segment_length = (segment0.rest_length + segment1.rest_length) * 0.5f;
  const float second_moment_of_area = glm::pi<float>() * std::pow(segment_radius, 4.f) * 0.25f;
  const float polar_moment_of_inertia = glm::pi<float>() * std::pow(segment_radius, 4.f) * 0.5f;
  pair.bending_alpha = 1.f / (pair.max_bending_modulus * second_moment_of_area / glm::pow(segment_length, 3.f));
  pair.torsion_alpha = 1.f / (pair.max_torsion_modulus * polar_moment_of_inertia / segment_length);

  StrengthGraph::Input strength_graph_input;
  strength_graph_input.root_distance = root_distance;
  strength_graph_input.polar_distance = biological_properties_input.polar_distance;
  strength_graph_input.polar_angle = biological_properties_input.polar_angle;
  strength_graph_input.profile_boundary_distance = distance_to_boundary;

  const glm::vec2 bending_strength = initialize_parameters.strength_graph.GetBendingStrength(strength_graph_input);
  const float max_bending_strain = glm::max(
      0.001f, trunk_strength_factor + ActivationFunction::Sigmoid(
                                          bending_strength.x, bending_strength.y, initialize_parameters.sapwood_offset,
                                          1.f / initialize_parameters.wood_transition, distance_to_boundary));
  const glm::vec2 twisting_strength = initialize_parameters.strength_graph.GetTwistingStrength(strength_graph_input);
  const float max_twisting_strain =
      glm::max(0.001f, trunk_strength_factor + ActivationFunction::Sigmoid(twisting_strength.x, twisting_strength.y,
                                                                           initialize_parameters.sapwood_offset,
                                                                           1.f / initialize_parameters.wood_transition,
                                                                           distance_to_boundary));
  const glm::vec2 bundle_strength = initialize_parameters.strength_graph.GetBundleStrength(strength_graph_input);
  const float max_bundle_strain = glm::max(
      0.001f, trunk_strength_factor + ActivationFunction::Sigmoid(
                                          bundle_strength.x, bundle_strength.y, initialize_parameters.sapwood_offset,
                                          1.f / initialize_parameters.wood_transition, distance_to_boundary));
  const glm::vec2 connectivity_strength =
      initialize_parameters.strength_graph.GetConnectivityStrength(strength_graph_input);
  const float max_connectivity_strain =
      glm::max(0.001f, trunk_strength_factor + ActivationFunction::Sigmoid(
                                                   connectivity_strength.x, connectivity_strength.y,
                                                   initialize_parameters.sapwood_offset,
                                                   1.f / initialize_parameters.wood_transition, distance_to_boundary));
  pair.max_bending_twist_bundle_strain = pair.bending_twist_bundle_strain_limit =
      glm::vec3(max_bending_strain, max_twisting_strain, max_bundle_strain);
  pair.max_connectivity_strain = pair.connectivity_strain_limit = max_connectivity_strain;
}

void RecountSegmentPairHandles(std::vector<DynamicStrands::GpuSegment>& segments,
                               std::vector<DynamicStrands::GpuSegmentData>& segment_data_list) {
  for (auto& segment : segments) {
    segment.pairs_count = 0;
  }
  for (size_t segment_index = 0; segment_index < segment_data_list.size(); ++segment_index) {
    int pair_count = 0;
    for (const int pair_handle : segment_data_list[segment_index].pair_handles) {
      if (pair_handle >= 0) {
        ++pair_count;
      }
    }
    segments[segment_index].pairs_count = pair_count;
  }
}

kinDS::MeshingBufferHashSettings CurrentMeshingBufferHashSettings() {
  const auto& settings = DsKineticVoronoiMeshing::meshing_settings;
  kinDS::MeshingBufferHashSettings out;
  out.store_mesh_metadata = settings.store_mesh_metadata;
  out.spline_tension = settings.spline_tension;
  out.bark_smooth_iterations = settings.bark_smooth_iterations;
  out.bark_smooth_strength = settings.bark_smooth_strength;
  out.bark_smooth_lock_boundary = settings.bark_smooth_lock_boundary;
  out.bark_smooth_uvs = settings.bark_smooth_uvs;
  out.bark_subdivide = settings.bark_subdivide;
  out.alpha_cutoff = settings.alpha_cutoff;
  out.branch_alpha_cutoff = settings.branch_alpha_cutoff;
  out.look_ahead = settings.look_ahead;
  out.hinge_only_profile_plane_mix = settings.hinge_only_profile_plane_mix;
  return out;
}

using MeshingInputHashStats = kinDS::MeshingBufferHashStats;

std::filesystem::path MeshingBufferDirectory() {
  const auto project_path = ProjectManager::GetProjectPath();
  if (project_path.empty()) {
    return std::filesystem::path("MeshBuffers");
  }
  return project_path.parent_path() / "MeshBuffers";
}

std::string CurrentUtcTimestamp() {
  const std::time_t time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  std::tm utc{};
#ifdef _WIN32
  gmtime_s(&utc, &time);
#else
  gmtime_r(&time, &utc);
#endif
  std::ostringstream timestamp;
  timestamp << std::put_time(&utc, "%Y-%m-%dT%H:%M:%SZ");
  return timestamp.str();
}

MeshingInputHashStats ComputeMeshingInputHashStats(
    const std::vector<std::vector<glm::dvec2>>& support_points,
    const std::vector<std::vector<double>>& subdivisions_by_strand,
    const std::vector<std::vector<int>>& physics_strand_to_segment_indices,
    const std::vector<std::vector<glm::dmat4>>& transforms_by_height_and_branch, const GlobalTransform& root_transform,
    const std::vector<std::vector<size_t>>& branch_indices,
    const std::vector<std::vector<std::vector<size_t>>>& strands_by_branch_id, const bool include_alpha_cutoff = true) {
  return kinDS::computeMeshingInputHash(support_points, subdivisions_by_strand, physics_strand_to_segment_indices,
                                        transforms_by_height_and_branch, root_transform.value, branch_indices,
                                        strands_by_branch_id, CurrentMeshingBufferHashSettings(), include_alpha_cutoff);
}

/// Default TreeMesher alpha_cutoff; caches hashed before alpha_cutoff entered the key used this value implicitly.
constexpr double kLegacyDefaultAlphaCutoff = 10.0;

void LogMeshingInputHashStats(const MeshingInputHashStats& stats) {
  EVOENGINE_LOG("Meshing buffer hash "
                << stats.input_hash << " | settings(v=" << kMeshingBufferVersion
                << ", store_meta=" << (DsKineticVoronoiMeshing::meshing_settings.store_mesh_metadata ? 1 : 0)
                << ", spline_tension=" << DsKineticVoronoiMeshing::meshing_settings.spline_tension
                << ", alpha_cutoff=" << DsKineticVoronoiMeshing::meshing_settings.alpha_cutoff
                << ", branch_alpha_cutoff=" << DsKineticVoronoiMeshing::meshing_settings.branch_alpha_cutoff
                << ", look_ahead=" << DsKineticVoronoiMeshing::meshing_settings.look_ahead
                << ", cap_start=1, xform_at_construction=1)=" << stats.settings_hash << " root=" << stats.root_hash
                << " [" << stats.root_transform_summary << "]"
                << " support(strands=" << stats.support_strand_count << ", pts=" << stats.support_point_count
                << ")=" << stats.support_hash << " subdiv(strands=" << stats.subdiv_strand_count
                << ", vals=" << stats.subdiv_value_count << ", min_len=" << stats.min_segment_length
                << ", max_len=" << stats.max_segment_length << ")=" << stats.subdiv_hash
                << " physics(strands=" << stats.physics_strand_count << ", segs=" << stats.physics_segment_count
                << ")=" << stats.physics_hash << " transforms(heights=" << stats.transform_height_count
                << ", mats=" << stats.transform_matrix_count << ")=" << stats.transforms_hash
                << " branches(heights=" << stats.branch_height_count << ", ids=" << stats.branch_id_count
                << ")=" << stats.branch_hash << " strands_by_branch(outer=" << stats.strands_by_branch_outer_count
                << ", ids=" << stats.strands_by_branch_id_count << ")=" << stats.strands_by_branch_hash);
}

YAML::Node BuildMeshingBufferStatisticsNode(const MeshingInputHashStats& stats) {
  YAML::Node statistics;
  statistics["input_hash"] = stats.input_hash;
  statistics["recorded_utc"] = CurrentUtcTimestamp();
  statistics["hash_inputs"]["settings"]["schema"] = "DsKineticVoronoiMeshing.v1";
  statistics["hash_inputs"]["settings"]["buffer_version"] = kMeshingBufferVersion;
  statistics["hash_inputs"]["settings"]["store_mesh_metadata"] =
      DsKineticVoronoiMeshing::meshing_settings.store_mesh_metadata;
  statistics["hash_inputs"]["settings"]["spline_tension"] = DsKineticVoronoiMeshing::meshing_settings.spline_tension;
  statistics["hash_inputs"]["settings"]["alpha_cutoff"] = DsKineticVoronoiMeshing::meshing_settings.alpha_cutoff;
  statistics["hash_inputs"]["settings"]["branch_alpha_cutoff"] =
      DsKineticVoronoiMeshing::meshing_settings.branch_alpha_cutoff;
  statistics["hash_inputs"]["settings"]["look_ahead"] = DsKineticVoronoiMeshing::meshing_settings.look_ahead;
  statistics["hash_inputs"]["settings"]["mesh_cap_at_start"] = true;
  statistics["hash_inputs"]["settings"]["transform_mesh_at_construction"] = true;
  statistics["hash_inputs"]["settings"]["hash"] = stats.settings_hash;
  statistics["hash_inputs"]["root_transform"]["summary"] = stats.root_transform_summary;
  statistics["hash_inputs"]["root_transform"]["hash"] = stats.root_hash;
  statistics["hash_inputs"]["support_points"]["strand_count"] = stats.support_strand_count;
  statistics["hash_inputs"]["support_points"]["point_count"] = stats.support_point_count;
  statistics["hash_inputs"]["support_points"]["hash"] = stats.support_hash;
  statistics["hash_inputs"]["subdivisions"]["min_segment_length"] = stats.min_segment_length;
  statistics["hash_inputs"]["subdivisions"]["max_segment_length"] = stats.max_segment_length;
  statistics["hash_inputs"]["subdivisions"]["strand_count"] = stats.subdiv_strand_count;
  statistics["hash_inputs"]["subdivisions"]["value_count"] = stats.subdiv_value_count;
  statistics["hash_inputs"]["subdivisions"]["hash"] = stats.subdiv_hash;
  statistics["hash_inputs"]["physics_segments"]["strand_count"] = stats.physics_strand_count;
  statistics["hash_inputs"]["physics_segments"]["segment_count"] = stats.physics_segment_count;
  statistics["hash_inputs"]["physics_segments"]["hash"] = stats.physics_hash;
  statistics["hash_inputs"]["transforms"]["height_count"] = stats.transform_height_count;
  statistics["hash_inputs"]["transforms"]["matrix_count"] = stats.transform_matrix_count;
  statistics["hash_inputs"]["transforms"]["hash"] = stats.transforms_hash;
  statistics["hash_inputs"]["branches"]["height_count"] = stats.branch_height_count;
  statistics["hash_inputs"]["branches"]["id_count"] = stats.branch_id_count;
  statistics["hash_inputs"]["branches"]["hash"] = stats.branch_hash;
  statistics["hash_inputs"]["strands_by_branch"]["outer_count"] = stats.strands_by_branch_outer_count;
  statistics["hash_inputs"]["strands_by_branch"]["id_count"] = stats.strands_by_branch_id_count;
  statistics["hash_inputs"]["strands_by_branch"]["hash"] = stats.strands_by_branch_hash;
  return statistics;
}

// Emit top-level keys in a stable order so description always appears below statistics.
bool WriteMeshingBufferYml(const std::filesystem::path& yml_path, const YAML::Node& root) {
  static constexpr const char* kOrderedKeys[] = {
      "format",
      "version",
      "hash",
      "created_utc",
      "gpu_vertex_count",
      "gpu_triangle_count",
      "meshlet_count",
      "vertex_stride",
      "triangle_stride",
      "spline_tension",
      "alpha_cutoff",
      "branch_alpha_cutoff",
      "look_ahead",
      "store_mesh_metadata",
      "mesh_cap_at_start",
      "transform_mesh_at_construction",
      "statistics",
      "description",
  };

  YAML::Emitter out;
  out << YAML::BeginMap;
  std::unordered_set<std::string> emitted;
  for (const char* key : kOrderedKeys) {
    if (root[key]) {
      out << YAML::Key << key << YAML::Value << root[key];
      emitted.insert(key);
    }
  }
  for (auto it = root.begin(); it != root.end(); ++it) {
    const std::string key = it->first.as<std::string>();
    if (emitted.count(key) != 0) {
      continue;
    }
    out << YAML::Key << key << YAML::Value << it->second;
  }
  out << YAML::EndMap;

  std::ofstream yaml_out(yml_path);
  yaml_out << out.c_str();
  return static_cast<bool>(yaml_out);
}

bool UpdateMeshingBufferYmlOnCacheHit(const std::filesystem::path& yml_path, const MeshingInputHashStats& stats,
                                      const std::string& description) {
  try {
    YAML::Node root;
    if (std::filesystem::exists(yml_path)) {
      root = YAML::LoadFile(yml_path.string());
    } else {
      root["format"] = "KVMG";
      root["version"] = kMeshingBufferVersion;
      root["hash"] = stats.input_hash;
    }
    if (!root["statistics"] || !root["statistics"]["input_hash"]) {
      root["statistics"] = BuildMeshingBufferStatisticsNode(stats);
    } else {
      root["statistics"]["hash_inputs"]["subdivisions"]["min_segment_length"] = stats.min_segment_length;
      root["statistics"]["hash_inputs"]["subdivisions"]["max_segment_length"] = stats.max_segment_length;
    }
    root["description"] = description;
    if (!WriteMeshingBufferYml(yml_path, root)) {
      EVOENGINE_WARNING("Failed to update meshing buffer metadata " << yml_path.string());
      return false;
    }
    return true;
  } catch (const std::exception& exception) {
    EVOENGINE_WARNING("Failed to update meshing buffer metadata " << yml_path.string() << ": " << exception.what());
    return false;
  }
}

bool TryMigrateMeshingBufferCacheFiles(const std::filesystem::path& legacy_bin, const std::filesystem::path& legacy_yml,
                                       const std::filesystem::path& new_bin, const std::filesystem::path& new_yml,
                                       const MeshingInputHashStats& new_stats) {
  std::error_code ec;
  if (std::filesystem::exists(new_bin, ec)) {
    EVOENGINE_WARNING("Legacy meshing buffer migrate skipped: target already exists " << new_bin.string());
    return false;
  }

  std::filesystem::rename(legacy_bin, new_bin, ec);
  if (ec) {
    EVOENGINE_WARNING("Failed to rename meshing buffer " << legacy_bin.string() << " -> " << new_bin.string() << ": "
                                                         << ec.message());
    return false;
  }

  if (std::filesystem::exists(legacy_yml, ec)) {
    if (std::filesystem::exists(new_yml, ec)) {
      std::filesystem::remove(legacy_yml, ec);
    } else {
      std::filesystem::rename(legacy_yml, new_yml, ec);
      if (ec) {
        EVOENGINE_WARNING("Renamed meshing .bin but failed to rename .yml "
                          << legacy_yml.string() << " -> " << new_yml.string() << ": " << ec.message());
      }
    }
  }

  try {
    YAML::Node root;
    if (std::filesystem::exists(new_yml)) {
      root = YAML::LoadFile(new_yml.string());
    } else {
      root["format"] = "KVMG";
      root["version"] = kMeshingBufferVersion;
    }
    root["hash"] = new_stats.input_hash;
    root["alpha_cutoff"] = DsKineticVoronoiMeshing::meshing_settings.alpha_cutoff;
    root["branch_alpha_cutoff"] = DsKineticVoronoiMeshing::meshing_settings.branch_alpha_cutoff;
    root["look_ahead"] = DsKineticVoronoiMeshing::meshing_settings.look_ahead;
    root["spline_tension"] = DsKineticVoronoiMeshing::meshing_settings.spline_tension;
    root["statistics"] = BuildMeshingBufferStatisticsNode(new_stats);
    root["description"] = DsKineticVoronoiMeshing::meshing_settings.meshing_buffer_description;
    WriteMeshingBufferYml(new_yml, root);
  } catch (const std::exception& exception) {
    EVOENGINE_WARNING("Migrated meshing buffer binary but failed to refresh metadata " << new_yml.string() << ": "
                                                                                       << exception.what());
  }

  EVOENGINE_LOG("Migrated legacy meshing buffer cache " << legacy_bin.filename().string() << " -> "
                                                        << new_bin.filename().string()
                                                        << " (added alpha_cutoff to hash).");
  return true;
}

template <typename T>
void PackGpuBlob(const std::vector<T>& values, std::vector<std::byte>& out_bytes) {
  out_bytes.resize(values.size() * sizeof(T));
  if (!values.empty()) {
    std::memcpy(out_bytes.data(), values.data(), out_bytes.size());
  }
}

template <typename T>
bool UnpackGpuBlob(const std::vector<std::byte>& bytes, uint32_t stride, std::vector<T>& out_values) {
  if (stride != sizeof(T)) {
    return false;
  }
  if (bytes.size() % sizeof(T) != 0) {
    return false;
  }
  out_values.resize(bytes.size() / sizeof(T));
  if (!out_values.empty()) {
    std::memcpy(out_values.data(), bytes.data(), bytes.size());
  }
  return true;
}

bool SaveMeshingBuffer(const std::filesystem::path& bin_path, const std::filesystem::path& yml_path,
                       const MeshingInputHashStats& hash_stats, const GlobalTransform& root_transform,
                       const std::vector<GpuMeshletVertex>& gpu_vertices,
                       const std::vector<GpuMeshletTriangle>& gpu_triangles,
                       const std::vector<kinDS::VoronoiMesh>& meshlets, const std::vector<std::vector<int>>& neighbors,
                       const std::vector<size_t>& meshing_to_physics,
                       const std::vector<std::vector<size_t>>& strand_to_segment) {
  kinDS::MeshingBufferPayload payload;
  payload.root_transform = root_transform.value;
  payload.meshlets = meshlets;
  payload.neighbors = neighbors;
  payload.meshing_to_physics = meshing_to_physics;
  payload.strand_to_segment = strand_to_segment;
  payload.gpu_vertex_stride = static_cast<uint32_t>(sizeof(GpuMeshletVertex));
  payload.gpu_triangle_stride = static_cast<uint32_t>(sizeof(GpuMeshletTriangle));
  PackGpuBlob(gpu_vertices, payload.gpu_vertices);
  PackGpuBlob(gpu_triangles, payload.gpu_triangles);

  kinDS::MeshingBufferMetadata metadata;
  metadata.input_hash = hash_stats.input_hash;
  metadata.description = DsKineticVoronoiMeshing::meshing_settings.meshing_buffer_description;
  metadata.alpha_cutoff = DsKineticVoronoiMeshing::meshing_settings.alpha_cutoff;
  metadata.branch_alpha_cutoff = DsKineticVoronoiMeshing::meshing_settings.branch_alpha_cutoff;
  metadata.look_ahead = DsKineticVoronoiMeshing::meshing_settings.look_ahead;
  metadata.spline_tension = DsKineticVoronoiMeshing::meshing_settings.spline_tension;
  metadata.store_mesh_metadata = DsKineticVoronoiMeshing::meshing_settings.store_mesh_metadata;

  if (!kinDS::saveMeshingBuffer(bin_path, payload, metadata)) {
    EVOENGINE_WARNING("Failed to write meshing buffer " << bin_path.string());
    return false;
  }

  YAML::Node root;
  root["format"] = "KVMG";
  root["version"] = kMeshingBufferVersion;
  root["hash"] = hash_stats.input_hash;
  root["created_utc"] = CurrentUtcTimestamp();
  root["gpu_vertex_count"] = gpu_vertices.size();
  root["gpu_triangle_count"] = gpu_triangles.size();
  root["meshlet_count"] = meshlets.size();
  root["vertex_stride"] = sizeof(GpuMeshletVertex);
  root["triangle_stride"] = sizeof(GpuMeshletTriangle);
  root["spline_tension"] = DsKineticVoronoiMeshing::meshing_settings.spline_tension;
  root["alpha_cutoff"] = DsKineticVoronoiMeshing::meshing_settings.alpha_cutoff;
  root["branch_alpha_cutoff"] = DsKineticVoronoiMeshing::meshing_settings.branch_alpha_cutoff;
  root["look_ahead"] = DsKineticVoronoiMeshing::meshing_settings.look_ahead;
  root["store_mesh_metadata"] = DsKineticVoronoiMeshing::meshing_settings.store_mesh_metadata;
  root["mesh_cap_at_start"] = true;
  root["transform_mesh_at_construction"] = true;
  root["statistics"] = BuildMeshingBufferStatisticsNode(hash_stats);
  root["description"] = DsKineticVoronoiMeshing::meshing_settings.meshing_buffer_description;

  if (!WriteMeshingBufferYml(yml_path, root)) {
    EVOENGINE_WARNING("Wrote meshing buffer binary but failed to write metadata " << yml_path.string());
  }
  return true;
}

bool LoadMeshingBuffer(const std::filesystem::path& bin_path, const GlobalTransform& root_transform,
                       std::vector<GpuMeshletVertex>& gpu_vertices, std::vector<GpuMeshletTriangle>& gpu_triangles,
                       std::vector<kinDS::VoronoiMesh>& meshlets, std::vector<std::vector<int>>& neighbors,
                       std::vector<size_t>& meshing_to_physics, std::vector<std::vector<size_t>>& strand_to_segment) {
  kinDS::MeshingBufferPayload payload;
  if (!kinDS::loadMeshingBuffer(bin_path, root_transform.value, payload, true)) {
    return false;
  }

  gpu_vertices.clear();
  gpu_triangles.clear();
  if (payload.gpu_vertex_stride != 0 || payload.gpu_triangle_stride != 0) {
    if (!UnpackGpuBlob(payload.gpu_vertices, payload.gpu_vertex_stride, gpu_vertices) ||
        !UnpackGpuBlob(payload.gpu_triangles, payload.gpu_triangle_stride, gpu_triangles)) {
      EVOENGINE_WARNING("Meshing buffer "
                        << bin_path.string()
                        << " has incompatible GPU strides; will rebuild GPU from meshlets if present.");
      gpu_vertices.clear();
      gpu_triangles.clear();
    }
  }

  meshlets = std::move(payload.meshlets);
  neighbors = std::move(payload.neighbors);
  meshing_to_physics = std::move(payload.meshing_to_physics);
  strand_to_segment = std::move(payload.strand_to_segment);
  return true;
}

/// Debug tag for how a profile plane was produced (group-name suffix + vertex/face metadata).
enum class ProfilePlaneDebugKind { Original, Parallel, Rotated };

const char* ProfilePlaneDebugKindSuffix(ProfilePlaneDebugKind kind) {
  switch (kind) {
    case ProfilePlaneDebugKind::Original:
      return "original";
    case ProfilePlaneDebugKind::Parallel:
      return "parallel";
    case ProfilePlaneDebugKind::Rotated:
      return "rotated";
  }
  return "unknown";
}

constexpr double kProfilePlaneNearParallelAngle = 1e-3;
constexpr double kProfilePlaneHingeEps = 1e-10;

/// Hinge between two profile planes for EcoSysLab mix (independent of kinDS::PlaneProjector).
struct ProfilePlaneHinge {
  glm::dvec3 axis{0.0, 1.0, 0.0};
  glm::dvec3 point{0.0};
  double angle = 0.0;
  glm::dvec3 nA{0.0, 1.0, 0.0};
  glm::dvec3 nB{0.0, 1.0, 0.0};
};

/// Geometric normal from profile span columns (col0 × col2), matching PlaneProjector's spanning-vector sense.
glm::dvec3 ProfilePlaneGeometricNormal(const glm::dmat4& transform) {
  const glm::dvec3 u(transform[0]);
  const glm::dvec3 v(transform[2]);
  const glm::dvec3 n = glm::cross(u, v);
  const double len = glm::length(n);
  if (len < kProfilePlaneHingeEps) {
    const glm::dvec3 front(transform[1]);
    const double fl = glm::length(front);
    return (fl > kProfilePlaneHingeEps) ? (front / fl) : glm::dvec3(0.0, 1.0, 0.0);
  }
  return n / len;
}

/**
 * Build the rotation hinge that maps plane A onto plane B.
 * Intersection point is the particular solution in span{nA, nB} (closest to the origin on the line),
 * satisfying nA·x + dA = 0 and nB·x + dB = 0 with d = -n·origin.
 *
 * NOTE: kinDS::PlaneProjector uses a different (incorrect) intersection formula; do not reuse it here.
 * @return false when planes are parallel / anti-parallel within @c kProfilePlaneHingeEps.
 */
bool TryBuildProfilePlaneHinge(const glm::dmat4& plane_a, const glm::dmat4& plane_b, ProfilePlaneHinge& out) {
  const glm::dvec3 oA(plane_a[3]);
  const glm::dvec3 oB(plane_b[3]);
  out.nA = ProfilePlaneGeometricNormal(plane_a);
  out.nB = ProfilePlaneGeometricNormal(plane_b);

  glm::dvec3 axis_raw = glm::cross(out.nA, out.nB);
  const double axis_len = glm::length(axis_raw);
  if (axis_len < kProfilePlaneHingeEps) {
    return false;
  }
  out.axis = axis_raw / axis_len;

  const double cos_theta = glm::clamp(glm::dot(out.nA, out.nB), -1.0, 1.0);
  out.angle = std::acos(cos_theta);

  // Planes: n·x + d = 0, d = -n·o. Particular point on the line in span{nA, nB}:
  //   λ nA + μ nB with [1, c; c, 1][λ; μ] = [-dA; -dB], c = nA·nB.
  const double dA = -glm::dot(out.nA, oA);
  const double dB = -glm::dot(out.nB, oB);
  const double c = cos_theta;
  const double det = 1.0 - c * c;  // sin^2(theta) = |nA × nB|^2 for unit normals
  if (std::abs(det) < kProfilePlaneHingeEps) {
    return false;
  }
  const double lambda = (-dA + c * dB) / det;
  const double mu = (-dB + c * dA) / det;
  out.point = lambda * out.nA + mu * out.nB;
  return true;
}

/// Same parallel / near-parallel decision as @ref MixProfilePlaneTransforms (keep thresholds in sync).
bool ProfilePlaneMixUsesParallelFallback(const glm::dmat4& lower, const glm::dmat4& upper) {
  ProfilePlaneHinge hinge;
  if (!TryBuildProfilePlaneHinge(lower, upper, hinge)) {
    return true;
  }
  const glm::dvec3 oA(lower[3]);
  const glm::dvec3 oB(upper[3]);
  const double len_uA = glm::length(glm::dvec3(lower[0]));
  const double len_vA = glm::length(glm::dvec3(lower[2]));
  const double len_uB = glm::length(glm::dvec3(upper[0]));
  const double len_vB = glm::length(glm::dvec3(upper[2]));
  const double span = std::max({len_uA, len_vA, len_uB, len_vB, glm::length(oB - oA), 1.0});
  const double hinge_radius = std::max(glm::length(oA - hinge.point), glm::length(oB - hinge.point));
  return hinge.angle < kProfilePlaneNearParallelAngle || hinge_radius > 1e4 * span;
}

/// Debug mesh: one axis-aligned quad per (height, branch) profile plane from StrandTree input only.
/// Quad extents are the min/max of that branch's site support points at that height, transformed by
/// the stored profile→object matrix (local convention (u, 0, v)).
/// Group names are suffixed with original / parallel / rotated; matching JSON is stored as metadata.
kinDS::VoronoiMesh BuildProfilePlanesDebugMesh(const kinDS::StrandTree& tree, int uniform_subdivision) {
  kinDS::VoronoiMesh mesh({}, kinDS::PerTriangleCorner);
  mesh.setStoreMetadata(true);
  const auto& support_points = tree.getSupportPoints();
  const auto& transforms = tree.getTransformsByHeightAndBranch();
  const auto& strands_by_branch_id = tree.getStrandsByBranchId();
  const size_t subdiv = static_cast<size_t>(std::max(1, uniform_subdivision));

  size_t plane_count = 0;
  for (size_t height = 0; height < transforms.size(); ++height) {
    if (height >= strands_by_branch_id.size()) {
      break;
    }
    for (size_t branch_id = 0; branch_id < transforms[height].size(); ++branch_id) {
      if (branch_id >= strands_by_branch_id[height].size()) {
        continue;
      }
      const auto& strand_ids = strands_by_branch_id[height][branch_id];
      if (strand_ids.empty()) {
        continue;
      }

      double min_u = std::numeric_limits<double>::infinity();
      double max_u = -std::numeric_limits<double>::infinity();
      double min_v = std::numeric_limits<double>::infinity();
      double max_v = -std::numeric_limits<double>::infinity();
      size_t site_count = 0;
      for (const size_t strand_id : strand_ids) {
        if (strand_id >= support_points.size() || height >= support_points[strand_id].size()) {
          continue;
        }
        const glm::dvec2& p = support_points[strand_id][height];
        min_u = std::min(min_u, p.x);
        max_u = std::max(max_u, p.x);
        min_v = std::min(min_v, p.y);
        max_v = std::max(max_v, p.y);
        ++site_count;
      }
      if (site_count == 0) {
        continue;
      }

      constexpr double kMinExtent = 1e-4;
      if (max_u - min_u < kMinExtent) {
        const double mid = 0.5 * (min_u + max_u);
        min_u = mid - 0.5 * kMinExtent;
        max_u = mid + 0.5 * kMinExtent;
      }
      if (max_v - min_v < kMinExtent) {
        const double mid = 0.5 * (min_v + max_v);
        min_v = mid - 0.5 * kMinExtent;
        max_v = mid + 0.5 * kMinExtent;
      }

      ProfilePlaneDebugKind kind = ProfilePlaneDebugKind::Original;
      if (height % subdiv != 0) {
        const size_t lower_h = (height / subdiv) * subdiv;
        const size_t upper_h = std::min(lower_h + subdiv, transforms.size() - 1);
        const size_t strand_id = strand_ids.front();
        const size_t lower_b = tree.getBranchIndex(strand_id, lower_h);
        const size_t upper_b = tree.getBranchIndex(strand_id, upper_h);
        if (lower_h < transforms.size() && upper_h < transforms.size() && lower_b < transforms[lower_h].size() &&
            upper_b < transforms[upper_h].size()) {
          kind = ProfilePlaneMixUsesParallelFallback(transforms[lower_h][lower_b], transforms[upper_h][upper_b])
                     ? ProfilePlaneDebugKind::Parallel
                     : ProfilePlaneDebugKind::Rotated;
        } else {
          // Missing bracketing originals — still mark as interpolated via hinge path unknown; use rotated label.
          kind = ProfilePlaneDebugKind::Rotated;
        }
      }

      const char* kind_suffix = ProfilePlaneDebugKindSuffix(kind);
      std::ostringstream group_name;
      group_name << "h" << height << "_b" << branch_id << "_n" << site_count << "_" << kind_suffix;
      mesh.startNewGroup(group_name.str());

      std::ostringstream material_name;
      material_name << "branch_" << branch_id << "_" << kind_suffix;
      const int material_id = mesh.ensureMaterialName(material_name.str());

      std::ostringstream meta;
      meta << "{\"plane_kind\":\"" << kind_suffix << "\",\"height\":" << height << ",\"branch\":" << branch_id
           << ",\"sites\":" << site_count << "}";
      const std::string metadata = meta.str();

      const glm::dmat4& profile_to_object = transforms[height][branch_id];
      const auto to_object = [&](double u, double v) -> glm::dvec3 {
        const glm::dvec4 world = profile_to_object * glm::dvec4(u, 0.0, v, 1.0);
        return glm::dvec3(world.x, world.y, world.z);
      };

      const size_t v0 = mesh.addVertex(to_object(min_u, min_v), metadata);
      const size_t v1 = mesh.addVertex(to_object(max_u, min_v), metadata);
      const size_t v2 = mesh.addVertex(to_object(max_u, max_v), metadata);
      const size_t v3 = mesh.addVertex(to_object(min_u, max_v), metadata);
      mesh.addTriangle(v0, v1, v2, material_id, metadata);
      mesh.addTriangle(v0, v2, v3, material_id, metadata);
      ++plane_count;
    }
  }

  if (plane_count > 0) {
    mesh.computeNormals(kinDS::PerTriangleCorner);
  }
  return mesh;
}

}  // namespace

// helper functions

/**
 * @brief Compute a 3D affine transformation that maps three coplanar source
 *        points to three coplanar target points, assuming an affine
 *        transformation that preserves the normalized plane normal direction.
 *
 * Given three non-collinear source points (p0, p1, p2) and their corresponding
 * non-collinear target points (q0, q1, q2), this function constructs the unique
 * affine transform T that satisfies:
 *
 *     T * vec4(p0, 1) = vec4(q0, 1)
 *     T * vec4(p1, 1) = vec4(q1, 1)
 *     T * vec4(p2, 1) = vec4(q2, 1)
 *
 * as well as:
 *
 *     T * n  = n'
 *
 * where n and n' are the normalized plane normals of the source and target
 * triangles, respectively. The normal direction is enforced to avoid the
 * underdetermined case that arises when all points lie in a plane.
 *
 * @note The returned transform maps points **from the source frame to the
 *       target frame**, i.e.:
 *
 *           T * vec4(p, 1) = vec4(q, 1)
 *
 *       for any point p lying in the same plane as (p0,p1,p2).
 *
 * @param p0 First source point in 3D.
 * @param p1 Second source point in 3D.
 * @param p2 Third source point in 3D.
 * @param q0 Corresponding target point to p0.
 * @param q1 Corresponding target point to p1.
 * @param q2 Corresponding target point to p2.
 *
 * @return glm::dmat4 The affine transformation matrix T such that T * p = q.
 *
 * @throws Undefined behavior if the three source or target points are collinear
 *         (i.e., they do not span a plane).
 */
glm::dmat4 ComputeAffineFromCoplanarPoints(const glm::vec3& p0, const glm::vec3& p1, const glm::vec3& p2,
                                           const glm::vec3& q0, const glm::vec3& q1, const glm::vec3& q2) {
  // --- Source basis ---
  glm::dvec3 u = p1 - p0;
  glm::dvec3 v = p2 - p0;
  glm::dvec3 n = glm::normalize(glm::cross(u, v));

  // --- Target basis ---
  glm::dvec3 up = q1 - q0;
  glm::dvec3 vp = q2 - q0;
  glm::dvec3 np = glm::normalize(glm::cross(up, vp));

  // (Optional) ensure consistent orientation.
  // If dot(n, np) < 0, flip np.
  if (glm::dot(n, np) < 0.0f)
    np = -np;

  // Build basis matrices B and B'
  glm::dmat3 B;
  B[0] = u;  // column 0
  B[1] = v;  // column 1
  B[2] = n;  // column 2

  glm::dmat3 Bp;
  Bp[0] = up;
  Bp[1] = vp;
  Bp[2] = np;

  // Linear part: A = B' * inverse(B)
  glm::mat3 A = Bp * glm::inverse(B);

  // Translation: t = q0 - A * p0
  glm::vec3 t = q0 - A * p0;

  // Assemble full 4x4 affine transform
  glm::dmat4 T(1.0f);
  T[0][0] = A[0][0];
  T[1][0] = A[1][0];
  T[2][0] = A[2][0];
  T[0][1] = A[0][1];
  T[1][1] = A[1][1];
  T[2][1] = A[2][1];
  T[0][2] = A[0][2];
  T[1][2] = A[1][2];
  T[2][2] = A[2][2];

  T[3] = glm::vec4(t, 1.0f);

  return T;
}

std::optional<std::array<size_t, 3>> FindNonCollinearTriple(std::function<glm::vec3(size_t)> get_point, size_t size,
                                                            float eps = 1e-6f) {
  if (size < 3)
    return std::optional<std::array<size_t, 3>>();

  // Step 1: choose p0
  size_t i0 = 0;
  size_t i1 = -1;
  size_t i2 = -1;
  // Step 2: choose p1 - must be distinct from p0

  for (int j = 1; j < size; ++j) {
    if (glm::length(get_point(j) - get_point(i0)) > eps) {
      i1 = j;
      break;
    }
  }
  if (i1 == -1)
    return std::optional<std::array<size_t, 3>>();  // all points identical

  // Step 3: find p2 that makes area > 0
  for (int k = i1 + 1; k < size; ++k) {
    glm::vec3 u = get_point(i1) - get_point(i0);
    glm::vec3 v = get_point(k) - get_point(i0);
    float area2 = glm::length(glm::cross(u, v));
    if (area2 > eps) {
      i2 = k;
      return std::optional<std::array<size_t, 3>>({i0, i1, i2});  // non-collinear triple found
    }
  }

  return std::optional<std::array<size_t, 3>>();  // all points collinear
}

std::optional<std::array<size_t, 2>> FindNonIdenticalPair(std::function<glm::vec3(size_t)> get_point, size_t size,
                                                          float eps = 1e-6f) {
  if (size < 2)
    return std::optional<std::array<size_t, 2>>();

  size_t i0 = 0;
  size_t i1 = -1;
  for (int j = 1; j < size; ++j) {
    if (glm::length(get_point(j) - get_point(i0)) > eps) {
      i1 = j;
      return std::optional<std::array<size_t, 2>>({i0, i1});
    }
  }
  return std::optional<std::array<size_t, 2>>();  // all points identical
}

glm::vec3 ProfileToModelCoordinates(const std::vector<std::vector<glm::dmat4>>& profile_to_model_transforms,
                                    glm::dvec3 point, float t, const std::vector<size_t>& branch_indices,
                                    float w = 1.0f) {
  size_t lower_section_index = static_cast<size_t>(std::max(0.0f, glm::floor(t)));

  size_t upper_section_index = std::min(profile_to_model_transforms.size() - 1, static_cast<size_t>(glm::ceil(t)));

  // check range
  auto coord_str = std::to_string(t);
  if (lower_section_index >= profile_to_model_transforms.size()) {
    std::cout << ("ProfileToModelCoordinates: lower bound of point z-coordinate out of range: " + coord_str).c_str()
              << std::endl;
  }
  if (upper_section_index >= profile_to_model_transforms.size()) {
    std::cout << ("ProfileToModelCoordinates: upper bound of point z-coordinate out of range: " + coord_str).c_str()
              << std::endl;
  }

  // only set second coordinate to 0 for points, not for normal vectors
  // TODO: I actually wanted to get rid of this coordinate swap at some point
  glm::vec4 local_pos(point[0], (1.0f - w) * point[2], point[1], w);
  size_t lower_branch_index = branch_indices[lower_section_index];
  glm::vec4 global_pos = profile_to_model_transforms[lower_section_index][lower_branch_index] * local_pos;

  if (upper_section_index != lower_section_index) {
    size_t upper_branch_index = branch_indices[upper_section_index];
    glm::vec4 upper_global_pos = profile_to_model_transforms[upper_section_index][upper_branch_index] * local_pos;
    float frac = static_cast<float>(t - static_cast<double>(lower_section_index));
    global_pos = glm::mix(global_pos, upper_global_pos, frac);
  }

  if (w == 0.0f) {
    global_pos = glm::normalize(global_pos);
  }

  return glm::vec3(global_pos);
}

glm::vec3 ToVec3(const glm::dvec3& a) {
  return glm::vec3(static_cast<float>(a[0]), static_cast<float>(a[1]), static_cast<float>(a[2]));
}

struct StrandCrossSectionGuidePoint {
  glm::dvec2 profile_position;
  SkeletonNodeHandle node_handle;
  StrandSegmentHandle segment_handle = -1;
  double root_distance = 0.0;
};

namespace {

/// Profile-plane to model-space transform for an internode cross-section at a given origin.
glm::dmat4 BuildInternodeProfileTransformAtOrigin(const StrandModelSkeleton& skeleton, SkeletonNodeHandle node_handle,
                                                  const glm::dvec3& origin) {
  if (node_handle < 0 || node_handle >= static_cast<SkeletonNodeHandle>(skeleton.PeekRawNodes().size())) {
    return glm::dmat4(1.0);
  }

  const auto& node = skeleton.PeekNode(node_handle);
  const glm::vec3 left_f = node.info.regulated_global_rotation * glm::vec3(1.0f, 0.0f, 0.0f);
  const glm::vec3 up_f = node.info.regulated_global_rotation * glm::vec3(0.0f, 1.0f, 0.0f);
  const glm::vec3 front_f = node.info.regulated_global_rotation * glm::vec3(0.0f, 0.0f, -1.0f);
  const glm::dvec3 left(left_f);
  const glm::dvec3 up(up_f);
  const glm::dvec3 front(front_f);
  const double radius = node.data.strand_radius;

  glm::dmat4 transform(1.0);
  transform[0] = glm::dvec4(left * radius, 0.0);
  transform[1] = glm::dvec4(front, 0.0);
  transform[2] = glm::dvec4(up * radius, 0.0);
  transform[3] = glm::dvec4(origin, 1.0);
  return transform;
}

/// Distal internode cross-section (segment ends), matching @ref StrandModel::ApplyProfile strand segment ends.
glm::dmat4 BuildInternodeProfileTransformAtEnd(const StrandModelSkeleton& skeleton, SkeletonNodeHandle node_handle) {
  if (node_handle < 0 || node_handle >= static_cast<SkeletonNodeHandle>(skeleton.PeekRawNodes().size())) {
    return glm::dmat4(1.0);
  }

  const auto& node = skeleton.PeekNode(node_handle);
  return BuildInternodeProfileTransformAtOrigin(skeleton, node_handle, glm::dvec3(node.info.GetGlobalEndPosition()));
}

/// Proximal internode cross-section (strand roots), matching @ref StrandModel::ApplyProfile for the first segment.
glm::dmat4 BuildInternodeProfileTransformAtStart(const StrandModelSkeleton& skeleton, SkeletonNodeHandle node_handle) {
  if (node_handle < 0 || node_handle >= static_cast<SkeletonNodeHandle>(skeleton.PeekRawNodes().size())) {
    return glm::dmat4(1.0);
  }

  const auto& node = skeleton.PeekNode(node_handle);
  return BuildInternodeProfileTransformAtOrigin(skeleton, node_handle, glm::dvec3(node.info.global_position));
}

/// Signed in-plane angle from @p from to @p to about unit normal @p normal (radians).
double SignedAngleAboutNormal(const glm::dvec3& from, const glm::dvec3& to, const glm::dvec3& normal) {
  constexpr double kEps = 1e-12;
  glm::dvec3 a = from - normal * glm::dot(from, normal);
  glm::dvec3 b = to - normal * glm::dot(to, normal);
  const double la = glm::length(a);
  const double lb = glm::length(b);
  if (la < kEps || lb < kEps) {
    return 0.0;
  }
  a /= la;
  b /= lb;
  return std::atan2(glm::dot(normal, glm::cross(a, b)), glm::dot(a, b));
}

/// Geometric profile-plane normal from span columns (matches @ref kinDS::PlaneProjector).
/// Note: transform column 1 is @c front = -cross(left, up), so do not use column 1 here.
glm::dvec3 GeometricProfileNormal(const glm::dvec3& u, const glm::dvec3& v, const glm::dvec3& fallback) {
  constexpr double kEps = 1e-12;
  const glm::dvec3 n = glm::cross(u, v);
  const double len = glm::length(n);
  if (len < kEps) {
    const double fl = glm::length(fallback);
    return (fl > kEps) ? (fallback / fl) : glm::dvec3(0.0, 1.0, 0.0);
  }
  return n / len;
}

/**
 * Interpolate profile-plane transforms between two original internode frames.
 *
 * Non-parallel: hinge about the plane intersection line, then in-plane rotation about the (hinged)
 * origin, then in-plane origin shift — each scaled by @p fraction. The shift is expressed in the
 * intermediate hinged frame (not as a fixed upper-plane world vector).
 * When @ref DsKineticVoronoiMeshing::MeshingSettings::hinge_only_profile_plane_mix is set, only the
 * hinge is applied (debug).
 * Parallel / near-parallel fallback: translation plus rotation about the geometric plane normal.
 * Span/normal column lengths are lerped separately.
 */
glm::dmat4 MixProfilePlaneTransforms(const glm::dmat4& lower, const glm::dmat4& upper, double fraction) {
  const double f = glm::clamp(fraction, 0.0, 1.0);
  if (f <= 0.0) {
    return lower;
  }
  if (f >= 1.0) {
    return upper;
  }

  constexpr double kEps = 1e-8;

  const glm::dvec3 oA(lower[3]);
  const glm::dvec3 uA(lower[0]);
  const glm::dvec3 nA_col(lower[1]);  // front; opposite geometric normal
  const glm::dvec3 vA(lower[2]);
  const glm::dvec3 oB(upper[3]);
  const glm::dvec3 uB(upper[0]);
  const glm::dvec3 nB_col(upper[1]);
  const glm::dvec3 vB(upper[2]);

  const double len_uA = glm::length(uA);
  const double len_vA = glm::length(vA);
  const double len_uB = glm::length(uB);
  const double len_vB = glm::length(vB);
  const double len_nA = glm::length(nA_col);
  const double len_nB = glm::length(nB_col);

  // Same spanning-vector normals as PlaneProjector (not the front column).
  const glm::dvec3 nA_geo = GeometricProfileNormal(uA, vA, nA_col);
  glm::dvec3 nB_geo = GeometricProfileNormal(uB, vB, nB_col);

  glm::dvec3 o_f = oA;
  glm::dvec3 u_f = uA;
  glm::dvec3 v_f = vA;
  glm::dvec3 n_f = nA_col;

  // Keep in sync with ProfilePlaneMixUsesParallelFallback (profile-plane debug tags).
  // Hinge geometry comes from TryBuildProfilePlaneHinge — not PlaneProjector (known bad m_p0).
  const bool use_parallel = ProfilePlaneMixUsesParallelFallback(lower, upper);

  if (use_parallel) {
    // Parallel: translation + rotation about the geometric plane normal (matches PlaneProjector sense).
    if (glm::dot(nA_geo, nB_geo) < 0.0) {
      nB_geo = -nB_geo;
    }
    const glm::dvec3 translation = oB - oA;
    const double phi = SignedAngleAboutNormal(uA, uB, nA_geo);

    const glm::dmat3 R = glm::dmat3(glm::rotate(glm::dmat4(1.0), f * phi, nA_geo));
    // Rotate spans about the lower origin, then translate (same as translating first for pure axes).
    u_f = R * uA;
    v_f = R * vA;
    n_f = R * ((len_nA > kEps) ? nA_col : nA_geo);
    o_f = oA + f * translation;
  } else {
    // Non-parallel: hinge about intersection line → (optional) in-plane rot → (optional) in-plane shift.
    ProfilePlaneHinge hinge;
    if (!TryBuildProfilePlaneHinge(lower, upper, hinge)) {
      // Should be unreachable when use_parallel is false; fall back to parallel mix.
      if (glm::dot(nA_geo, nB_geo) < 0.0) {
        nB_geo = -nB_geo;
      }
      const glm::dvec3 translation = oB - oA;
      const double phi = SignedAngleAboutNormal(uA, uB, nA_geo);
      const glm::dmat3 R = glm::dmat3(glm::rotate(glm::dmat4(1.0), f * phi, nA_geo));
      u_f = R * uA;
      v_f = R * vA;
      n_f = R * ((len_nA > kEps) ? nA_col : nA_geo);
      o_f = oA + f * translation;
    } else {
      const glm::dvec3& axis = hinge.axis;
      const glm::dvec3& p0 = hinge.point;
      const double theta = hinge.angle;
      // Use hinge normals (same sense as spanning-vector cross products).
      nB_geo = hinge.nB;
      const glm::dvec3 nA_hinge = hinge.nA;

      const glm::dmat3 R_h_f = glm::dmat3(glm::rotate(glm::dmat4(1.0), f * theta, axis));
      o_f = p0 + R_h_f * (oA - p0);
      u_f = R_h_f * uA;
      v_f = R_h_f * vA;
      n_f = R_h_f * ((len_nA > kEps) ? nA_col : nA_hinge);

      // Debug: hinge only — skip in-plane origin shift and rotation about the plane normal.
      if (!DsKineticVoronoiMeshing::meshing_settings.hinge_only_profile_plane_mix) {
        const glm::dmat3 R_full = glm::dmat3(glm::rotate(glm::dmat4(1.0), theta, axis));
        const glm::dmat3 R_full_inv = glm::transpose(R_full);  // pure rotation
        const glm::dvec3 oA_h = p0 + R_full * (oA - p0);
        const glm::dvec3 uA_h = R_full * uA;

        // Residual after full hinge, parallel to the upper geometric plane.
        glm::dvec3 shift_B = oB - oA_h;
        shift_B -= nB_geo * glm::dot(shift_B, nB_geo);

        // In-plane twist about geometric normal after full hinge.
        const double phi = SignedAngleAboutNormal(uA_h, uB, nB_geo);

        glm::dvec3 n_mid = glm::normalize(R_h_f * nA_hinge);
        if (glm::length2(n_mid) < kEps * kEps) {
          n_mid = nB_geo;
        }
        // In-plane rotation about the hinged support origin (before shifting it).
        const glm::dmat3 R_n_f = glm::dmat3(glm::rotate(glm::dmat4(1.0), f * phi, n_mid));
        u_f = R_n_f * u_f;
        v_f = R_n_f * v_f;
        n_f = R_n_f * n_f;

        // Carry the residual shift in the intermediate hinged orientation (not as a fixed B-world vector).
        o_f += f * (R_h_f * (R_full_inv * shift_B));
      }
    }
  }

  const double su = glm::mix(len_uA, len_uB, f);
  const double sv = glm::mix(len_vA, len_vB, f);
  const double sn = glm::mix(len_nA > kEps ? len_nA : 1.0, len_nB > kEps ? len_nB : 1.0, f);

  if (glm::length(u_f) > kEps) {
    u_f = glm::normalize(u_f) * su;
  }
  if (glm::length(v_f) > kEps) {
    v_f = glm::normalize(v_f) * sv;
  }
  if (glm::length(n_f) > kEps) {
    n_f = glm::normalize(n_f) * sn;
  } else {
    // Prefer front-aligned geometric normal (opposite cross(u,v) in our frame convention).
    n_f = -GeometricProfileNormal(u_f, v_f, n_f) * sn;
  }

  glm::dmat4 result(1.0);
  result[0] = glm::dvec4(u_f, 0.0);
  result[1] = glm::dvec4(n_f, 0.0);
  result[2] = glm::dvec4(v_f, 0.0);
  result[3] = glm::dvec4(o_f, 1.0);
  return result;
}

glm::dmat4 BuildInterpolatedInternodeTransformAtHeight(const StrandModelSkeleton& skeleton,
                                                       const std::vector<StrandCrossSectionGuidePoint>& guide_points,
                                                       size_t height) {
  if (guide_points.empty()) {
    return glm::dmat4(1.0);
  }

  const size_t clamped_height = std::min(height, guide_points.size() - 1);
  const SkeletonNodeHandle current_internode = guide_points[clamped_height].node_handle;

  size_t internode_run_start = clamped_height;
  while (internode_run_start > 0 && guide_points[internode_run_start - 1].node_handle == current_internode) {
    --internode_run_start;
  }

  size_t internode_run_end = clamped_height;
  while (internode_run_end + 1 < guide_points.size() &&
         guide_points[internode_run_end + 1].node_handle == current_internode) {
    ++internode_run_end;
  }

  const glm::dmat4 current_internode_end_transform = BuildInternodeProfileTransformAtEnd(skeleton, current_internode);

  const double end_distance = guide_points[internode_run_end].root_distance;

  glm::dmat4 lower_transform;
  glm::dmat4 upper_transform = current_internode_end_transform;
  double start_distance = 0.0;

  if (internode_run_start == 0) {
    // Strand roots are placed at the internode base (global_position), not at the parent's distal end.
    // See StrandModel::ApplyProfile when prev_segment_handle == -1.
    lower_transform = BuildInternodeProfileTransformAtStart(skeleton, current_internode);
    start_distance = guide_points[0].root_distance;
  } else {
    const SkeletonNodeHandle previous_internode = guide_points[internode_run_start - 1].node_handle;
    lower_transform = BuildInternodeProfileTransformAtEnd(skeleton, previous_internode);
    start_distance = guide_points[internode_run_start - 1].root_distance;
  }

  if (end_distance <= start_distance + glm::epsilon<double>()) {
    return current_internode_end_transform;
  }

  const double fraction =
      (guide_points[clamped_height].root_distance - start_distance) / (end_distance - start_distance);
  return MixProfilePlaneTransforms(lower_transform, upper_transform, fraction);
}

struct ProfilePlane {
  glm::dvec3 origin{0.0};
  glm::dvec3 normal{0.0, 1.0, 0.0};
};

struct CubicPlaneHit {
  double t = 0.0;
  glm::dvec3 point{0.0};
  int segment_index = 0;
};

constexpr double kPlaneSplineRootMargin = 1e-6;
constexpr double kPlaneSplineImagEps = 1e-8;
constexpr double kPlaneSplineResidualEps = 1e-4;
constexpr double kPlaneSplineDegenerateEps = 1e-12;

ProfilePlane ExtractProfilePlane(const glm::dmat4& transform) {
  ProfilePlane plane;
  plane.origin = glm::dvec3(transform[3]);
  const glm::dvec3 front = glm::dvec3(transform[1]);
  const double front_length = glm::length(front);
  plane.normal = front_length > kPlaneSplineDegenerateEps ? front / front_length : glm::dvec3(0.0, 1.0, 0.0);
  return plane;
}

glm::dvec2 WorldToProfile(const glm::dmat4& transform, const glm::dvec3& hit) {
  const glm::dvec4 local = glm::inverse(transform) * glm::dvec4(hit, 1.0);
  // ProfileToModelCoordinates uses local (x, 0, y) with the y/z swap convention.
  return glm::dvec2(local.x, local.z);
}

void StrandCubicPowerCoeffs(const glm::dvec3& v0, const glm::dvec3& v1, const glm::dvec3& v2, const glm::dvec3& v3,
                            glm::dvec3& c0, glm::dvec3& c1, glm::dvec3& c2, glm::dvec3& c3, double tension = 0.0) {
  // Meshing-only blend between:
  //   tension 0 → Strands::CubicInterpolation (does not pass through knots)
  //   tension 1 → Catmull-Rom Hermite (passes through v1 and v2)
  const double t = glm::clamp(tension, 0.0, 1.0);

  // Strands::CubicInterpolation power expansion.
  const glm::dvec3 p0 = (v2 + v0) / 6.0 + v1 * (4.0 / 6.0);
  const glm::dvec3 p1 = v2 - v0;
  const glm::dvec3 p2 = v2 - v1;
  const glm::dvec3 p3 = v3 - v1;
  const glm::dvec3 strands_c0 = p0;
  const glm::dvec3 strands_c1 = 0.5 * p1;
  const glm::dvec3 strands_c2 = -0.5 * p1 + p2;
  const glm::dvec3 strands_c3 = (1.0 / 6.0) * p1 - (2.0 / 3.0) * p2 + (1.0 / 6.0) * p3;

  // Catmull-Rom as Hermite through v1 → v2 with tangents 0.5*(v2-v0), 0.5*(v3-v1).
  const glm::dvec3 m0 = 0.5 * (v2 - v0);
  const glm::dvec3 m1 = 0.5 * (v3 - v1);
  const glm::dvec3 catmull_c0 = v1;
  const glm::dvec3 catmull_c1 = m0;
  const glm::dvec3 catmull_c2 = -3.0 * v1 - 2.0 * m0 + 3.0 * v2 - m1;
  const glm::dvec3 catmull_c3 = 2.0 * v1 + m0 - 2.0 * v2 + m1;

  c0 = glm::mix(strands_c0, catmull_c0, t);
  c1 = glm::mix(strands_c1, catmull_c1, t);
  c2 = glm::mix(strands_c2, catmull_c2, t);
  c3 = glm::mix(strands_c3, catmull_c3, t);
}

glm::dvec3 EvalStrandCubic(const glm::dvec3& c0, const glm::dvec3& c1, const glm::dvec3& c2, const glm::dvec3& c3,
                           double t) {
  return c0 + t * (c1 + t * (c2 + t * c3));
}

glm::dvec3 EvalStrandCubicDerivative(const glm::dvec3& c1, const glm::dvec3& c2, const glm::dvec3& c3, double t) {
  return c1 + t * (2.0 * c2 + t * 3.0 * c3);
}

double PlaneResidual(const glm::dvec3& point, const ProfilePlane& plane) {
  return glm::dot(plane.normal, point - plane.origin);
}

std::vector<double> CollectRealRootsIn01(const kinDS::Polynomial& poly) {
  std::vector<double> roots_in_01;
  if (poly.degree() <= 0) {
    return roots_in_01;
  }

  const Eigen::VectorXcd complex_roots = poly.roots();
  for (int i = 0; i < complex_roots.size(); ++i) {
    if (std::abs(complex_roots[i].imag()) > kPlaneSplineImagEps) {
      continue;
    }
    const double root = complex_roots[i].real();
    if (root >= -kPlaneSplineRootMargin && root <= 1.0 + kPlaneSplineRootMargin) {
      roots_in_01.push_back(glm::clamp(root, 0.0, 1.0));
    }
  }
  return roots_in_01;
}

double BisectPlaneRoot(const glm::dvec3& c0, const glm::dvec3& c1, const glm::dvec3& c2, const glm::dvec3& c3,
                       const ProfilePlane& plane, double t_min, double t_max, int iterations = 40) {
  double f_min = PlaneResidual(EvalStrandCubic(c0, c1, c2, c3, t_min), plane);
  double f_max = PlaneResidual(EvalStrandCubic(c0, c1, c2, c3, t_max), plane);
  if (f_min * f_max > 0.0) {
    return 0.5 * (t_min + t_max);
  }

  for (int i = 0; i < iterations; ++i) {
    const double t_mid = 0.5 * (t_min + t_max);
    const double f_mid = PlaneResidual(EvalStrandCubic(c0, c1, c2, c3, t_mid), plane);
    if (f_min * f_mid <= 0.0) {
      t_max = t_mid;
      f_max = f_mid;
    } else {
      t_min = t_mid;
      f_min = f_mid;
    }
  }
  return 0.5 * (t_min + t_max);
}

double NewtonPolishPlaneRoot(const glm::dvec3& c0, const glm::dvec3& c1, const glm::dvec3& c2, const glm::dvec3& c3,
                             const ProfilePlane& plane, double t, int iterations = 4) {
  for (int i = 0; i < iterations; ++i) {
    const glm::dvec3 point = EvalStrandCubic(c0, c1, c2, c3, t);
    const double f = PlaneResidual(point, plane);
    const double fp = glm::dot(plane.normal, EvalStrandCubicDerivative(c1, c2, c3, t));
    if (std::abs(fp) < kPlaneSplineDegenerateEps) {
      break;
    }
    t = glm::clamp(t - f / fp, 0.0, 1.0);
  }
  return t;
}

std::optional<CubicPlaneHit> IntersectCubicSegmentWithPlane(const glm::dvec3& v0, const glm::dvec3& v1,
                                                            const glm::dvec3& v2, const glm::dvec3& v3,
                                                            const ProfilePlane& plane, int segment_index,
                                                            double preferred_t = -1.0, double tension = 0.0) {
  glm::dvec3 c0, c1, c2, c3;
  StrandCubicPowerCoeffs(v0, v1, v2, v3, c0, c1, c2, c3, tension);

  const double f0 = PlaneResidual(EvalStrandCubic(c0, c1, c2, c3, 0.0), plane);
  const double f1 = PlaneResidual(EvalStrandCubic(c0, c1, c2, c3, 1.0), plane);

  Eigen::VectorXd coeffs(4);
  coeffs << (glm::dot(plane.normal, c0) - glm::dot(plane.normal, plane.origin)), glm::dot(plane.normal, c1),
      glm::dot(plane.normal, c2), glm::dot(plane.normal, c3);
  kinDS::Polynomial residual_poly(coeffs);

  std::vector<double> candidate_ts = CollectRealRootsIn01(residual_poly);

  // Degenerate / missed-root fallback: endpoints straddle the plane.
  if (candidate_ts.empty() && f0 * f1 <= 0.0) {
    candidate_ts.push_back(BisectPlaneRoot(c0, c1, c2, c3, plane, 0.0, 1.0));
  }

  // Near-coplanar segment: keep endpoint with smaller residual.
  if (candidate_ts.empty()) {
    if (std::abs(f0) <= kPlaneSplineResidualEps) {
      candidate_ts.push_back(0.0);
    } else if (std::abs(f1) <= kPlaneSplineResidualEps) {
      candidate_ts.push_back(1.0);
    } else {
      return std::nullopt;
    }
  }

  for (double& t : candidate_ts) {
    t = NewtonPolishPlaneRoot(c0, c1, c2, c3, plane, t);
  }

  auto score = [&](double t) {
    const double residual = std::abs(PlaneResidual(EvalStrandCubic(c0, c1, c2, c3, t), plane));
    const double preference = preferred_t >= 0.0 ? std::abs(t - preferred_t) : 0.0;
    return residual + 1e-3 * preference;
  };

  double best_t = candidate_ts.front();
  double best_score = score(best_t);
  for (size_t i = 1; i < candidate_ts.size(); ++i) {
    const double candidate_score = score(candidate_ts[i]);
    if (candidate_score < best_score) {
      best_score = candidate_score;
      best_t = candidate_ts[i];
    }
  }

  CubicPlaneHit hit;
  hit.t = best_t;
  hit.point = EvalStrandCubic(c0, c1, c2, c3, best_t);
  hit.segment_index = segment_index;
  if (std::abs(PlaneResidual(hit.point, plane)) > 10.0 * kPlaneSplineResidualEps && f0 * f1 > 0.0) {
    return std::nullopt;
  }
  return hit;
}

std::optional<CubicPlaneHit> IntersectStrandWithPlane(const StrandModelStrandGroup& strand_group,
                                                      StrandHandle strand_handle, const ProfilePlane& plane,
                                                      int hint_segment_index, double preferred_t = -1.0,
                                                      double tension = 0.0) {
  const auto& strand = strand_group.PeekStrand(strand_handle);
  const auto& segment_handles = strand.PeekStrandSegmentHandles();
  if (segment_handles.empty()) {
    return std::nullopt;
  }

  const int segment_count = static_cast<int>(segment_handles.size());
  const int clamped_hint = glm::clamp(hint_segment_index, 0, segment_count - 1);

  auto try_segment = [&](int segment_index) -> std::optional<CubicPlaneHit> {
    glm::vec3 p0, p1, p2, p3;
    strand_group.GetPositionControlPoints(segment_handles[segment_index], p0, p1, p2, p3);
    return IntersectCubicSegmentWithPlane(glm::dvec3(p0), glm::dvec3(p1), glm::dvec3(p2), glm::dvec3(p3), plane,
                                          segment_index, preferred_t, tension);
  };

  // Search outward from the height hint so successive samples advance monotonically.
  if (auto hit = try_segment(clamped_hint)) {
    return hit;
  }

  for (int radius = 1; radius < segment_count; ++radius) {
    const int forward = clamped_hint + radius;
    if (forward < segment_count) {
      if (auto hit = try_segment(forward)) {
        return hit;
      }
    }
    const int backward = clamped_hint - radius;
    if (backward >= 0) {
      if (auto hit = try_segment(backward)) {
        return hit;
      }
    }
  }
  return std::nullopt;
}

glm::dvec2 SampleStrandProfileAtPlane(const StrandModelStrandGroup& strand_group, StrandHandle strand_handle,
                                      const glm::dmat4& transform, int& hint_segment_index,
                                      const glm::dvec2& fallback_profile, double preferred_t = -1.0,
                                      double tension = 0.0) {
  const ProfilePlane plane = ExtractProfilePlane(transform);
  const auto hit =
      IntersectStrandWithPlane(strand_group, strand_handle, plane, hint_segment_index, preferred_t, tension);
  if (!hit.has_value()) {
    return fallback_profile;
  }

  hint_segment_index = hit->segment_index;
  const glm::dvec2 profile = WorldToProfile(transform, hit->point);

#ifndef NDEBUG
  const glm::dvec4 reconstructed = transform * glm::dvec4(profile.x, 0.0, profile.y, 1.0);
  const double round_trip = std::abs(PlaneResidual(glm::dvec3(reconstructed), plane));
  if (round_trip > kPlaneSplineResidualEps) {
    EVOENGINE_WARNING("Plane-spline profile round-trip residual " << round_trip << " exceeds tolerance on strand "
                                                                  << strand_handle << " segment "
                                                                  << hit->segment_index);
  }
#endif

  return profile;
}

// Legacy affine-fit helpers (retained for comparison; no longer used in InitData).
[[maybe_unused]] glm::dmat4 FitGlobalProfileToModelTransformAtHeight(
    int h, const std::vector<std::vector<size_t>>& sorted_segments,
    const DtsStrandGroup& uniformly_subdivided_strand_group) {
  const auto& segments = sorted_segments[h == 0 ? 0 : (h - 1)];

  std::function<glm::vec3(size_t)> get_point = [&](size_t idx) {
    size_t segment_handle = segments[idx];
    const auto& segment_data = uniformly_subdivided_strand_group.PeekStrandSegmentData(segment_handle);
    return glm::vec3(segment_data.profile_position.x, 0.0f, segment_data.profile_position.y);
  };

  std::optional<std::array<size_t, 3>> triple_opt = FindNonCollinearTriple(get_point, segments.size());

  glm::vec3 p0_profile, p1_profile, p2_profile;
  glm::vec3 p0_global, p1_global, p2_global;

  if (!triple_opt.has_value()) {
    std::optional<std::array<size_t, 2>> pair_opt = FindNonIdenticalPair(get_point, segments.size());

    StrandSegmentHandle first_segment_handle = segments[0];
    const auto& first_segment_data = uniformly_subdivided_strand_group.PeekStrandSegmentData(first_segment_handle);
    const auto& first_segment = uniformly_subdivided_strand_group.PeekStrandSegment(first_segment_handle);
    const auto& strand = uniformly_subdivided_strand_group.PeekStrand(first_segment.GetStrandHandle());
    StrandSegmentHandle next_segment_handle = first_segment.GetNextHandle();

    float normal_sign = 1.0f;
    glm::vec3 normal_global;

    if (h != 0) {
      if (next_segment_handle == -1) {
        next_segment_handle = first_segment.GetPrevHandle();
        normal_sign = -1.0f;
      }

      const auto& next_segment = uniformly_subdivided_strand_group.PeekStrandSegment(next_segment_handle);
      normal_global = normal_sign * glm::normalize(next_segment.end_position - first_segment.end_position);
    } else {
      normal_global = normal_sign * glm::normalize(strand.start_position - first_segment.end_position);
    }

    glm::vec3 u_global;
    glm::vec3 v_global;
    glm::vec3 u_profile;
    glm::vec3 v_profile;

    if (!pair_opt.has_value()) {
      u_global = glm::normalize(glm::cross(normal_global, glm::vec3(1.0f, 0.0f, 0.0f)));
      if (glm::length(u_global) < glm::epsilon<float>()) {
        u_global = glm::normalize(glm::cross(normal_global, glm::vec3(0.0f, 0.0f, 1.0f)));
      }
      v_global = glm::normalize(glm::cross(normal_global, u_global));

      if (h != 0) {
        p0_global = first_segment.end_position;
      } else {
        p0_global = strand.start_position;
      }
      p1_global = p0_global + u_global;
      p0_profile = glm::vec3(first_segment_data.profile_position.x, 0.0f, first_segment_data.profile_position.y);
      p1_profile = p0_profile + glm::vec3(1.0f, 0.0f, 0.0f);
    } else {
      const auto& pair = pair_opt.value();
      size_t p0_idx = pair[0];
      size_t p1_idx = pair[1];

      const glm::vec2& p0_profile_2d =
          uniformly_subdivided_strand_group.PeekStrandSegmentData(segments[p0_idx]).profile_position;
      p0_profile = glm::vec3(p0_profile_2d.x, 0.0f, p0_profile_2d.y);

      const glm::vec2& p1_profile_2d =
          uniformly_subdivided_strand_group.PeekStrandSegmentData(segments[p1_idx]).profile_position;
      p1_profile = glm::vec3(p1_profile_2d.x, 0.0f, p1_profile_2d.y);

      if (h != 0) {
        p0_global = uniformly_subdivided_strand_group.PeekStrandSegment(segments[p0_idx]).end_position;
        p1_global = uniformly_subdivided_strand_group.PeekStrandSegment(segments[p1_idx]).end_position;
      } else {
        p0_global =
            uniformly_subdivided_strand_group
                .PeekStrand(uniformly_subdivided_strand_group.PeekStrandSegment(segments[p0_idx]).GetStrandHandle())
                .start_position;
        p1_global =
            uniformly_subdivided_strand_group
                .PeekStrand(uniformly_subdivided_strand_group.PeekStrandSegment(segments[p1_idx]).GetStrandHandle())
                .start_position;
      }

      u_profile = glm::normalize(p1_profile - p0_profile);
      u_global = glm::normalize(p1_global - p0_global);
      v_profile = glm::normalize(glm::cross(normal_global, u_profile));
      v_global = glm::normalize(glm::cross(normal_global, u_global));
    }

    p2_global = p0_global + v_global;
    p2_profile = p0_profile + glm::vec3(0.0f, 0.0f, 1.0f);
  } else {
    const auto& triple = triple_opt.value();

    const glm::vec2 p0_profile_2d =
        uniformly_subdivided_strand_group.PeekStrandSegmentData(segments[triple[0]]).profile_position;
    p0_profile = glm::vec3(p0_profile_2d.x, 0.0f, p0_profile_2d.y);

    const glm::vec2& p1_profile_2d =
        uniformly_subdivided_strand_group.PeekStrandSegmentData(segments[triple[1]]).profile_position;
    p1_profile = glm::vec3(p1_profile_2d.x, 0.0f, p1_profile_2d.y);

    const glm::vec2& p2_profile_2d =
        uniformly_subdivided_strand_group.PeekStrandSegmentData(segments[triple[2]]).profile_position;
    p2_profile = glm::vec3(p2_profile_2d.x, 0.0f, p2_profile_2d.y);

    if (h != 0) {
      p0_global = uniformly_subdivided_strand_group.PeekStrandSegment(segments[triple[0]]).end_position;
      p1_global = uniformly_subdivided_strand_group.PeekStrandSegment(segments[triple[1]]).end_position;
      p2_global = uniformly_subdivided_strand_group.PeekStrandSegment(segments[triple[2]]).end_position;
    } else {
      p0_global =
          uniformly_subdivided_strand_group
              .PeekStrand(uniformly_subdivided_strand_group.PeekStrandSegment(segments[triple[0]]).GetStrandHandle())
              .start_position;
      p1_global =
          uniformly_subdivided_strand_group
              .PeekStrand(uniformly_subdivided_strand_group.PeekStrandSegment(segments[triple[1]]).GetStrandHandle())
              .start_position;
      p2_global =
          uniformly_subdivided_strand_group
              .PeekStrand(uniformly_subdivided_strand_group.PeekStrandSegment(segments[triple[2]]).GetStrandHandle())
              .start_position;
    }
  }

  return ComputeAffineFromCoplanarPoints(p0_profile, p1_profile, p2_profile, p0_global, p1_global, p2_global);
}

[[maybe_unused]] glm::dmat4 FitBranchProfileToModelTransformAtHeight(
    int h, size_t branch_index, const std::vector<std::vector<std::vector<size_t>>>& strands_by_branch_id,
    const std::vector<std::vector<StrandCrossSectionGuidePoint>>& strand_guide_points,
    const DtsStrandGroup& uniformly_subdivided_strand_group) {
  const auto& strand_ids = strands_by_branch_id[h][branch_index];
  if (strand_ids.empty()) {
    return glm::dmat4(1.0);
  }

  std::function<glm::vec3(size_t)> get_point = [&](size_t idx) {
    size_t strand_id = strand_ids[idx];
    const auto& segment_data =
        uniformly_subdivided_strand_group.PeekStrandSegmentData(strand_guide_points[strand_id][h].segment_handle);
    return glm::vec3(segment_data.profile_position.x, 0.0f, segment_data.profile_position.y);
  };

  std::optional<std::array<size_t, 3>> triple_opt = FindNonCollinearTriple(get_point, strand_ids.size());

  glm::vec3 p0_profile, p1_profile, p2_profile;
  glm::vec3 p0_global, p1_global, p2_global;

  if (!triple_opt.has_value()) {
    std::optional<std::array<size_t, 2>> pair_opt = FindNonIdenticalPair(get_point, strand_ids.size());

    StrandSegmentHandle first_segment_handle = strand_guide_points[strand_ids[0]][h].segment_handle;
    const auto& first_segment_data = uniformly_subdivided_strand_group.PeekStrandSegmentData(first_segment_handle);
    const auto& first_segment = uniformly_subdivided_strand_group.PeekStrandSegment(first_segment_handle);
    const auto& strand = uniformly_subdivided_strand_group.PeekStrand(first_segment.GetStrandHandle());
    StrandSegmentHandle next_segment_handle = first_segment.GetNextHandle();

    float normal_sign = 1.0f;
    glm::vec3 normal_global;

    if (h != 0) {
      if (next_segment_handle == -1) {
        next_segment_handle = first_segment.GetPrevHandle();
        normal_sign = -1.0f;
      }

      const auto& next_segment = uniformly_subdivided_strand_group.PeekStrandSegment(next_segment_handle);
      normal_global = normal_sign * glm::normalize(next_segment.end_position - first_segment.end_position);
    } else {
      normal_global = normal_sign * glm::normalize(strand.start_position - first_segment.end_position);
    }

    glm::vec3 u_global;
    glm::vec3 v_global;

    if (!pair_opt.has_value()) {
      u_global = glm::normalize(glm::cross(normal_global, glm::vec3(1.0f, 0.0f, 0.0f)));
      if (glm::length(u_global) < glm::epsilon<float>()) {
        u_global = glm::normalize(glm::cross(normal_global, glm::vec3(0.0f, 0.0f, 1.0f)));
      }
      v_global = glm::normalize(glm::cross(normal_global, u_global));

      if (h != 0) {
        p0_global = first_segment.end_position;
      } else {
        p0_global = strand.start_position;
      }
      p1_global = p0_global + u_global;
      p0_profile = glm::vec3(first_segment_data.profile_position.x, 0.0f, first_segment_data.profile_position.y);
      p1_profile = p0_profile + glm::vec3(1.0f, 0.0f, 0.0f);
    } else {
      const auto& pair = pair_opt.value();
      const glm::vec2& p0_profile_2d =
          uniformly_subdivided_strand_group
              .PeekStrandSegmentData(strand_guide_points[strand_ids[pair[0]]][h].segment_handle)
              .profile_position;
      p0_profile = glm::vec3(p0_profile_2d.x, 0.0f, p0_profile_2d.y);

      const glm::vec2& p1_profile_2d =
          uniformly_subdivided_strand_group
              .PeekStrandSegmentData(strand_guide_points[strand_ids[pair[1]]][h].segment_handle)
              .profile_position;
      p1_profile = glm::vec3(p1_profile_2d.x, 0.0f, p1_profile_2d.y);

      if (h != 0) {
        p0_global = uniformly_subdivided_strand_group
                        .PeekStrandSegment(strand_guide_points[strand_ids[pair[0]]][h].segment_handle)
                        .end_position;
        p1_global = uniformly_subdivided_strand_group
                        .PeekStrandSegment(strand_guide_points[strand_ids[pair[1]]][h].segment_handle)
                        .end_position;
      } else {
        p0_global = uniformly_subdivided_strand_group
                        .PeekStrand(uniformly_subdivided_strand_group
                                        .PeekStrandSegment(strand_guide_points[strand_ids[pair[0]]][h].segment_handle)
                                        .GetStrandHandle())
                        .start_position;
        p1_global = uniformly_subdivided_strand_group
                        .PeekStrand(uniformly_subdivided_strand_group
                                        .PeekStrandSegment(strand_guide_points[strand_ids[pair[1]]][h].segment_handle)
                                        .GetStrandHandle())
                        .start_position;
      }

      glm::vec3 u_profile = glm::normalize(p1_profile - p0_profile);
      (void)u_profile;
      glm::vec3 u_global_dir = glm::normalize(p1_global - p0_global);
      v_global = glm::normalize(glm::cross(normal_global, u_global_dir));
    }

    p2_global = p0_global + v_global;
    p2_profile = p0_profile + glm::vec3(0.0f, 0.0f, 1.0f);
  } else {
    const auto& triple = triple_opt.value();

    const glm::vec2 p0_profile_2d =
        uniformly_subdivided_strand_group
            .PeekStrandSegmentData(strand_guide_points[strand_ids[triple[0]]][h].segment_handle)
            .profile_position;
    p0_profile = glm::vec3(p0_profile_2d.x, 0.0f, p0_profile_2d.y);

    const glm::vec2& p1_profile_2d =
        uniformly_subdivided_strand_group
            .PeekStrandSegmentData(strand_guide_points[strand_ids[triple[1]]][h].segment_handle)
            .profile_position;
    p1_profile = glm::vec3(p1_profile_2d.x, 0.0f, p1_profile_2d.y);

    const glm::vec2& p2_profile_2d =
        uniformly_subdivided_strand_group
            .PeekStrandSegmentData(strand_guide_points[strand_ids[triple[2]]][h].segment_handle)
            .profile_position;
    p2_profile = glm::vec3(p2_profile_2d.x, 0.0f, p2_profile_2d.y);

    if (h != 0) {
      p0_global = uniformly_subdivided_strand_group
                      .PeekStrandSegment(strand_guide_points[strand_ids[triple[0]]][h].segment_handle)
                      .end_position;
      p1_global = uniformly_subdivided_strand_group
                      .PeekStrandSegment(strand_guide_points[strand_ids[triple[1]]][h].segment_handle)
                      .end_position;
      p2_global = uniformly_subdivided_strand_group
                      .PeekStrandSegment(strand_guide_points[strand_ids[triple[2]]][h].segment_handle)
                      .end_position;
    } else {
      p0_global = uniformly_subdivided_strand_group
                      .PeekStrand(uniformly_subdivided_strand_group
                                      .PeekStrandSegment(strand_guide_points[strand_ids[triple[0]]][h].segment_handle)
                                      .GetStrandHandle())
                      .start_position;
      p1_global = uniformly_subdivided_strand_group
                      .PeekStrand(uniformly_subdivided_strand_group
                                      .PeekStrandSegment(strand_guide_points[strand_ids[triple[1]]][h].segment_handle)
                                      .GetStrandHandle())
                      .start_position;
      p2_global = uniformly_subdivided_strand_group
                      .PeekStrand(uniformly_subdivided_strand_group
                                      .PeekStrandSegment(strand_guide_points[strand_ids[triple[2]]][h].segment_handle)
                                      .GetStrandHandle())
                      .start_position;
    }
  }

  return ComputeAffineFromCoplanarPoints(p0_profile, p1_profile, p2_profile, p0_global, p1_global, p2_global);
}

}  // namespace

// kinDS::VoronoiMesh DsKineticVoronoiMeshing::TransformBoundaryMesh(
//     const kinDS::VoronoiMesh& boundary_mesh,
//     const std::vector<std::vector<glm::dmat4>>& transforms_by_height_and_branch,
//     const std::vector<std::vector<glm::dmat4>>& normal_transforms_by_height_and_branch,
//     const GlobalTransform& root_transform, const std::vector<std::vector<size_t>>& branch_indices,
//     const std::vector<size_t>& boundary_vertex_to_strand_id) {
//   // Boundary remap export is intentionally disabled for now.
//   return {};
// }

void DsKineticVoronoiMeshing::WriteIntersectionStatisticsCsv(
    const std::filesystem::path& base_csv_path, const std::vector<std::pair<std::string, IntersectionRunStats>>& rows) {
  if (rows.empty()) {
    return;
  }
  const std::filesystem::path csv_path = kinDS::Statistics::timestampedCsvPath(base_csv_path);
  std::ofstream out(csv_path);
  if (!out) {
    EVOENGINE_ERROR("Failed to write intersection statistics CSV " << csv_path.string());
    return;
  }
  out << "name,inside_meshlets,intersecting_meshlets,outside_meshlets,input_poly_count,runtime_s\n";
  out << std::setprecision(std::numeric_limits<double>::max_digits10);
  size_t total_polys = 0;
  double total_runtime = 0.0;
  for (const auto& [name, stats] : rows) {
    out << name << ',' << stats.inside_meshlets << ',' << stats.intersecting_meshlets << ',' << stats.outside_meshlets
        << ',' << stats.input_poly_count << ',' << stats.runtime_seconds << '\n';
    total_polys += stats.input_poly_count;
    total_runtime += stats.runtime_seconds;
  }
  if (rows.size() > 1) {
    out << "total,,,," << total_polys << ',' << total_runtime << '\n';
  }
  EVOENGINE_LOG("Wrote intersection statistics CSV to " << csv_path.string());
}

bool DsKineticVoronoiMeshing::HasMeshedSegmentMeshlets() const {
  if (!tree_mesher_) {
    return false;
  }
  return !segment_meshlets_.empty() || !tree_mesher_->getSegmentMeshlets().empty();
}

bool DsKineticVoronoiMeshing::EnsureCpuMeshletsFromGpu() {
  if (!segment_meshlets_.empty()) {
    if (tree_mesher_ && tree_mesher_->getSegmentMeshlets().empty()) {
      tree_mesher_->getSegmentMeshlets() = segment_meshlets_;
      tree_mesher_->getMeshingNeighborIndices() = meshing_neighbor_indices_;
    }
    return tree_mesher_ != nullptr;
  }
  if (tree_mesher_ && !tree_mesher_->getSegmentMeshlets().empty()) {
    segment_meshlets_ = tree_mesher_->getSegmentMeshlets();
    if (meshing_neighbor_indices_.empty()) {
      meshing_neighbor_indices_ = tree_mesher_->getMeshingNeighborIndices();
    }
    return true;
  }
  if (segment_meshlet_vertices.empty() || segment_meshlet_triangles.empty()) {
    return false;
  }
  if (!tree_mesher_) {
    if (!strand_tree) {
      return false;
    }
    tree_mesher_ = std::make_shared<kinDS::TreeMesher>(*strand_tree, MakeKineticParallelFor());
    tree_mesher_->getSettings().transform_mesh_at_construction = true;
    tree_mesher_->getSettings().mesh_cap_at_start = true;
    tree_mesher_->getSettings().meshing_statistics_csv_path = EcoSysLabMetadataPath("meshing_statistics.csv");
  }

  GlobalTransform inv_root;
  inv_root.value = glm::inverse(meshlets_root_transform_.value);

  const auto& meshing_to_physics = tree_mesher_->getMeshingToPhysicsSegmentIndices();
  int max_physics = -1;
  for (const auto& vertex : segment_meshlet_vertices) {
    max_physics = glm::max(max_physics, static_cast<int>(vertex.segment_index));
  }
  for (const size_t physics_id : meshing_to_physics) {
    if (physics_id != static_cast<size_t>(-1)) {
      max_physics = glm::max(max_physics, static_cast<int>(physics_id));
    }
  }
  if (max_physics < 0) {
    return false;
  }

  std::vector<int> physics_to_meshing(static_cast<size_t>(max_physics) + 1, -1);
  if (!meshing_to_physics.empty()) {
    segment_meshlets_.assign(meshing_to_physics.size(), kinDS::VoronoiMesh({}, kinDS::PerTriangleCorner));
    meshing_neighbor_indices_.assign(meshing_to_physics.size(), {});
    for (size_t meshing_id = 0; meshing_id < meshing_to_physics.size(); ++meshing_id) {
      const size_t physics_id = meshing_to_physics[meshing_id];
      if (physics_id != static_cast<size_t>(-1) && physics_id < physics_to_meshing.size()) {
        physics_to_meshing[physics_id] = static_cast<int>(meshing_id);
      }
    }
  } else {
    segment_meshlets_.assign(static_cast<size_t>(max_physics) + 1, kinDS::VoronoiMesh({}, kinDS::PerTriangleCorner));
    meshing_neighbor_indices_.assign(static_cast<size_t>(max_physics) + 1, {});
    std::vector<size_t> identity(static_cast<size_t>(max_physics) + 1);
    for (int physics_id = 0; physics_id <= max_physics; ++physics_id) {
      physics_to_meshing[static_cast<size_t>(physics_id)] = physics_id;
      identity[static_cast<size_t>(physics_id)] = static_cast<size_t>(physics_id);
    }
    tree_mesher_->setMeshingToPhysicsSegmentIndices(std::move(identity));
  }

  auto meshlet_index_for_physics = [&](const int physics_id) -> int {
    if (physics_id < 0 || static_cast<size_t>(physics_id) >= physics_to_meshing.size()) {
      return physics_id;
    }
    return physics_to_meshing[static_cast<size_t>(physics_id)];
  };

  std::vector<size_t> global_to_local(segment_meshlet_vertices.size(), static_cast<size_t>(-1));
  std::vector<std::vector<glm::dvec3>> corner_normals(segment_meshlets_.size());
  const bool restore_metadata = meshing_settings.store_mesh_metadata &&
                                segment_meshlet_vertex_metadata.size() == segment_meshlet_vertices.size() &&
                                segment_meshlet_face_metadata.size() == segment_meshlet_triangles.size();
  for (auto& mesh : segment_meshlets_) {
    mesh.setStoreMetadata(restore_metadata);
  }

  for (size_t vertex_index = 0; vertex_index < segment_meshlet_vertices.size(); ++vertex_index) {
    const int meshlet_id =
        meshlet_index_for_physics(static_cast<int>(segment_meshlet_vertices[vertex_index].segment_index));
    if (meshlet_id < 0 || static_cast<size_t>(meshlet_id) >= segment_meshlets_.size()) {
      continue;
    }
    const glm::vec3 local = inv_root.TransformPoint(segment_meshlet_vertices[vertex_index].x0);
    const std::string& vertex_meta = restore_metadata ? segment_meshlet_vertex_metadata[vertex_index] : std::string{};
    global_to_local[vertex_index] = segment_meshlets_[static_cast<size_t>(meshlet_id)].addVertex(
        glm::dvec3(local.x, local.y, local.z), vertex_meta);
  }

  size_t triangle_index = 0;
  for (const auto& triangle : segment_meshlet_triangles) {
    if (triangle.vertex_index0 >= segment_meshlet_vertices.size() ||
        triangle.vertex_index1 >= segment_meshlet_vertices.size() ||
        triangle.vertex_index2 >= segment_meshlet_vertices.size()) {
      ++triangle_index;
      continue;
    }
    const int meshlet_id =
        meshlet_index_for_physics(static_cast<int>(segment_meshlet_vertices[triangle.vertex_index0].segment_index));
    if (meshlet_id < 0 || static_cast<size_t>(meshlet_id) >= segment_meshlets_.size()) {
      ++triangle_index;
      continue;
    }
    const size_t i0 = global_to_local[triangle.vertex_index0];
    const size_t i1 = global_to_local[triangle.vertex_index1];
    const size_t i2 = global_to_local[triangle.vertex_index2];
    if (i0 == static_cast<size_t>(-1) || i1 == static_cast<size_t>(-1) || i2 == static_cast<size_t>(-1)) {
      ++triangle_index;
      continue;
    }
    auto& mesh = segment_meshlets_[static_cast<size_t>(meshlet_id)];
    const size_t uv0 = mesh.addUV(glm::dvec3(triangle.uv[0].x, triangle.uv[0].y, triangle.uv[0].z));
    const size_t uv1 = mesh.addUV(glm::dvec3(triangle.uv[1].x, triangle.uv[1].y, triangle.uv[1].z));
    const size_t uv2 = mesh.addUV(glm::dvec3(triangle.uv[2].x, triangle.uv[2].y, triangle.uv[2].z));
    const std::string& face_meta = restore_metadata ? segment_meshlet_face_metadata[triangle_index] : std::string{};
    mesh.addTriangle(i0, i1, i2, uv0, uv1, uv2, -1, face_meta);
    auto& normals = corner_normals[static_cast<size_t>(meshlet_id)];
    for (int corner = 0; corner < 3; ++corner) {
      const glm::vec3 local_n = inv_root.TransformVector(glm::vec3(triangle.normal[corner]));
      normals.emplace_back(local_n.x, local_n.y, local_n.z);
    }
    const int neighbor = meshlet_index_for_physics(triangle.neighbor_segment_index);
    meshing_neighbor_indices_[static_cast<size_t>(meshlet_id)].push_back(neighbor);
    ++triangle_index;
  }

  for (size_t meshlet_id = 0; meshlet_id < segment_meshlets_.size(); ++meshlet_id) {
    if (!corner_normals[meshlet_id].empty()) {
      segment_meshlets_[meshlet_id].setCornerNormals(std::move(corner_normals[meshlet_id]));
    }
  }

  bool have_strand_map = false;
  try {
    have_strand_map = !tree_mesher_->getMeshingStrandToSegmentIndices().empty();
  } catch (const std::exception&) {
    have_strand_map = false;
  }
  if (strand_tree && !have_strand_map) {
    const auto& physics_strands = strand_tree->getPhysicsStrandToSegmentIndices();
    std::vector<std::vector<size_t>> meshing_strands(physics_strands.size());
    for (size_t strand_id = 0; strand_id < physics_strands.size(); ++strand_id) {
      for (const int physics_id : physics_strands[strand_id]) {
        const int meshlet_id = meshlet_index_for_physics(physics_id);
        if (meshlet_id >= 0) {
          meshing_strands[strand_id].push_back(static_cast<size_t>(meshlet_id));
        }
      }
    }
    tree_mesher_->setMeshingStrandToSegmentIndices(std::move(meshing_strands));
  }

  tree_mesher_->getSegmentMeshlets() = segment_meshlets_;
  tree_mesher_->getMeshingNeighborIndices() = meshing_neighbor_indices_;
  EVOENGINE_LOG("Rebuilt CPU meshlets from GPU buffers (" << segment_meshlets_.size() << " meshlets, "
                                                          << segment_meshlet_vertices.size() << " vertices).");
  return !segment_meshlets_.empty();
}

DsKineticVoronoiMeshing::OwnerMeshing DsKineticVoronoiMeshing::FindForEntity(const std::shared_ptr<Scene>& scene,
                                                                             const Entity& entity) {
  OwnerMeshing result;
  if (!scene || !scene->IsEntityValid(entity)) {
    return result;
  }

  const auto try_entity = [&](const Entity& candidate) -> OwnerMeshing {
    OwnerMeshing found;
    if (!scene->IsEntityValid(candidate) || !scene->HasPrivateComponent<DynamicTreeStrands>(candidate)) {
      return found;
    }
    const auto dts = scene->GetOrSetPrivateComponent<DynamicTreeStrands>(candidate).lock();
    if (!dts || !dts->dynamic_strands || !dts->dynamic_strands->GetKineticVoronoiMeshing()) {
      return found;
    }
    found.meshing = dts->dynamic_strands->GetKineticVoronoiMeshing();
    if (found.meshing) {
      found.dts_owner = candidate;
    }
    return found;
  };

  // Prefer a meshed KineticVoronoi DTS. TreeDescriptor demos mesh into PhysicsDemo's DTS; still
  // walk ancestors / scene owners so Intersection Meshes parented under related entities resolve.
  OwnerMeshing first_ancestor;
  Entity current = entity;
  while (scene->IsEntityValid(current)) {
    OwnerMeshing found = try_entity(current);
    if (found.meshing) {
      found.meshing->EnsureCpuMeshletsFromGpu();
      if (found.meshing->HasMeshedSegmentMeshlets()) {
        return found;
      }
      if (!first_ancestor.meshing) {
        first_ancestor = found;
      }
    }
    current = scene->GetParent(current);
  }

  OwnerMeshing first_global;
  const auto* dts_owners = scene->UnsafeGetPrivateComponentOwnersList<DynamicTreeStrands>();
  if (dts_owners) {
    for (const auto& owner : *dts_owners) {
      OwnerMeshing found = try_entity(owner);
      if (!found.meshing) {
        continue;
      }
      found.meshing->EnsureCpuMeshletsFromGpu();
      if (found.meshing->HasMeshedSegmentMeshlets()) {
        return found;
      }
      if (!first_global.meshing) {
        first_global = found;
      }
    }
  }

  if (first_ancestor.meshing) {
    return first_ancestor;
  }
  return first_global;
}

bool DsKineticVoronoiMeshing::LoadIntersectionSetup(const std::shared_ptr<Scene>& scene, const Entity& owner,
                                                    const std::filesystem::path& yaml_path) {
  if (!scene || !scene->IsEntityValid(owner)) {
    EVOENGINE_ERROR("Load intersection setup: invalid owner entity.");
    return false;
  }
  if (!std::filesystem::exists(yaml_path)) {
    EVOENGINE_ERROR("Load intersection setup: file not found: " << yaml_path.string());
    return false;
  }

  const auto create_group_entity = [&](const std::shared_ptr<Scene>& s, const Entity& o) -> Entity {
    const Entity group = s->CreateEntity("Intersection Meshes (" + yaml_path.stem().string() + ")");
    s->SetParent(group, o);
    GlobalTransform group_gt{};
    group_gt.value = glm::mat4(1.0f);
    s->SetDataComponent(group, group_gt);
    s->GetOrSetPrivateComponent<DsIntersectionBoundaryMeshGroup>(group);
    return group;
  };
  const auto resolve_obj_path = [&](const std::filesystem::path& obj_path) -> std::filesystem::path {
    if (obj_path.is_absolute() && std::filesystem::exists(obj_path)) {
      return obj_path;
    }
    const auto from_assets = ProjectManager::GetAssetsFolderPath() / obj_path;
    if (std::filesystem::exists(from_assets)) {
      return from_assets;
    }
    const auto from_yaml_dir = yaml_path.parent_path() / obj_path;
    if (std::filesystem::exists(from_yaml_dir)) {
      return from_yaml_dir;
    }
    return obj_path;
  };

  try {
    const YAML::Node root = YAML::Load(FileUtils::LoadFileAsString(yaml_path));
    if (!root["intersection_meshes"]) {
      EVOENGINE_ERROR("Load intersection setup: no 'intersection_meshes' key in file.");
      return false;
    }
    const Entity group = create_group_entity(scene, owner);
    if (root["group_transform"]) {
      GlobalTransform group_gt{};
      group_gt.value = root["group_transform"].as<glm::mat4>();
      scene->SetDataComponent(group, group_gt);
    }
    size_t loaded_count = 0;
    for (const auto& entry : root["intersection_meshes"]) {
      if (!entry["obj_path"] || !entry["transform"]) {
        continue;
      }
      const std::filesystem::path obj_path = resolve_obj_path(entry["obj_path"].as<std::string>());
      const glm::mat4 transform_value = entry["transform"].as<glm::mat4>();
      try {
        kinDS::VoronoiMesh loaded_mesh = kinDS::ObjExporter::readMesh(obj_path);
        const auto child = scene->CreateEntity("Intersection Mesh (" + obj_path.stem().string() + ")");
        scene->SetParent(child, group);
        GlobalTransform child_gt{};
        child_gt.value = transform_value;
        scene->SetDataComponent(child, child_gt);
        const auto ibm = scene->GetOrSetPrivateComponent<DsIntersectionBoundaryMesh>(child).lock();
        if (ibm) {
          ibm->LoadMesh(std::move(loaded_mesh), obj_path);
          ++loaded_count;
        }
      } catch (const std::exception& ex) {
        EVOENGINE_ERROR("Failed to load OBJ '" << obj_path.string() << "': " << ex.what());
      }
    }
    EVOENGINE_LOG("Loaded intersection setup from " << yaml_path.string() << " (" << loaded_count << " mesh(es)).");
    return loaded_count > 0;
  } catch (const std::exception& ex) {
    EVOENGINE_ERROR("Failed to parse intersection setup YAML: " << ex.what());
    return false;
  }
}

bool DsKineticVoronoiMeshing::IntersectMeshletsWithBoundary(const kinDS::VoronoiMesh& raw_mesh,
                                                            const GlobalTransform& boundary_world_transform,
                                                            const GlobalTransform& tree_world_transform,
                                                            IntersectionRunStats* stats,
                                                            const bool apply_to_simulation) {
  if (!HasMeshedSegmentMeshlets() && !EnsureCpuMeshletsFromGpu()) {
    EVOENGINE_ERROR("Intersect: no meshed segment meshlets available. Run meshing first.");
    return false;
  }
  if (raw_mesh.getTriangleCount() == 0) {
    EVOENGINE_ERROR("Intersect: intersection boundary mesh is empty.");
    return false;
  }
  if (!strand_tree) {
    EVOENGINE_ERROR("Intersect: strand tree is missing.");
    return false;
  }

  // Meshlets live in tree-local space (the kinDS algorithm produces geometry relative to the tree
  // origin, before any world transform is applied). The boundary entity's GlobalTransform is in world
  // space. Convert the boundary into tree-local space using the tree entity's *current* world transform.
  kinDS::VoronoiMesh boundary_mesh = raw_mesh;
  const glm::dmat4 clip_transform =
      glm::inverse(glm::dmat4(tree_world_transform.value)) * glm::dmat4(boundary_world_transform.value);
  boundary_mesh.applyTransform(clip_transform);

  // Restore pristine meshlets so Intersect can be re-run after moving the boundary.
  if (segment_meshlets_.empty() && tree_mesher_ && !tree_mesher_->getSegmentMeshlets().empty()) {
    segment_meshlets_ = tree_mesher_->getSegmentMeshlets();
    meshing_neighbor_indices_ = tree_mesher_->getMeshingNeighborIndices();
  }
  tree_mesher_->getSegmentMeshlets() = segment_meshlets_;
  tree_mesher_->getMeshingNeighborIndices() = meshing_neighbor_indices_;

  const bool previous_fix_missing_meshes = tree_mesher_->getSettings().fix_missing_meshes;
  const bool previous_keep_original_on_failure = tree_mesher_->getSettings().keep_original_on_intersection_failure;
  const bool previous_prefer_meshlet_uv_on_seam = tree_mesher_->getSettings().intersection_prefer_meshlet_uv_on_seam;
  const bool previous_boundary_faces_interior_uv = tree_mesher_->getSettings().intersection_boundary_faces_interior_uv;
  tree_mesher_->getSettings().fix_missing_meshes = meshing_settings.intersection_boundary_fix_missing_meshes;
  tree_mesher_->getSettings().keep_original_on_intersection_failure =
      meshing_settings.intersection_keep_original_on_failure;
  tree_mesher_->getSettings().intersection_prefer_meshlet_uv_on_seam =
      meshing_settings.intersection_prefer_meshlet_uv_on_seam;
  tree_mesher_->getSettings().intersection_boundary_faces_interior_uv =
      meshing_settings.intersection_boundary_faces_interior_uv;
  tree_mesher_->getSettings().export_separate_contributor_objects =
      meshing_settings.export_separate_contributor_objects;
  EVOENGINE_LOG("Intersecting meshlets with boundary (" << boundary_mesh.getTriangleCount() << " triangles)...");
  const auto intersection_started = std::chrono::steady_clock::now();
  const kinDS::TreeMesher::BoundaryTruncateResult truncate_result = tree_mesher_->truncateToBoundary(boundary_mesh);
  const double intersection_seconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - intersection_started).count();
  if (stats) {
    stats->inside_meshlets = truncate_result.inside_count;
    stats->intersecting_meshlets = truncate_result.intersecting_count;
    stats->outside_meshlets = truncate_result.outside_count;
    stats->input_poly_count = boundary_mesh.getTriangleCount();
    stats->runtime_seconds = intersection_seconds;
  }
  tree_mesher_->getSettings().fix_missing_meshes = previous_fix_missing_meshes;
  tree_mesher_->getSettings().keep_original_on_intersection_failure = previous_keep_original_on_failure;
  tree_mesher_->getSettings().intersection_prefer_meshlet_uv_on_seam = previous_prefer_meshlet_uv_on_seam;
  tree_mesher_->getSettings().intersection_boundary_faces_interior_uv = previous_boundary_faces_interior_uv;

  segment_meshlet_vertices.clear();
  segment_meshlet_triangles.clear();
  if (!apply_to_simulation) {
    PrepareAndPopulateGpuMeshletBuffers(
        tree_mesher_->getSegmentMeshlets(), strand_tree->getPhysicsStrandToSegmentIndices(),
        tree_mesher_->getMeshingStrandToSegmentIndices(), tree_mesher_->getMeshingNeighborIndices(),
        tree_mesher_->getMeshingToPhysicsSegmentIndices(), meshlets_root_transform_);
    EVOENGINE_LOG("Intersection complete (export-only). " << segment_meshlet_vertices.size() << " vertices, "
                                                          << segment_meshlet_triangles.size() << " triangles.");
    return true;
  }

  DownloadPhysicsSegmentsAndPairs();
  if (meshing_settings.recompute_segment_pairs) {
    SplitIntersectingMeshletsByConnectedComponents(truncate_result.intersecting_meshlet_indices);
  }

  PrepareAndPopulateGpuMeshletBuffers(
      tree_mesher_->getSegmentMeshlets(), strand_tree->getPhysicsStrandToSegmentIndices(),
      tree_mesher_->getMeshingStrandToSegmentIndices(), tree_mesher_->getMeshingNeighborIndices(),
      tree_mesher_->getMeshingToPhysicsSegmentIndices(), meshlets_root_transform_);
  Upload();
  CompactSurvivingPhysicsSegments(truncate_result.outside_meshlet_indices,
                                  /*rebuild_pairs=*/!meshing_settings.recompute_segment_pairs);
  if (meshing_settings.recompute_segment_pairs) {
    RecomputeSegmentPairs(*tree_mesher_);
    FinalizeStrandConnectivityAndPairMaterials();
    // Rebuild GPU meshlets so segment_pair_index matches the recomputed pairs.
    segment_meshlet_vertices.clear();
    segment_meshlet_triangles.clear();
    PrepareAndPopulateGpuMeshletBuffers(
        tree_mesher_->getSegmentMeshlets(), strand_tree->getPhysicsStrandToSegmentIndices(),
        tree_mesher_->getMeshingStrandToSegmentIndices(), tree_mesher_->getMeshingNeighborIndices(),
        tree_mesher_->getMeshingToPhysicsSegmentIndices(), meshlets_root_transform_);
  }
  Upload();  // remapped / rebuilt meshlet segment / neighbor / pair indices
  UploadPhysicsSegmentsAndPairs();
  UpdateBindings();
  EVOENGINE_LOG("Intersection complete. GPU meshlet buffers updated ("
                << segment_meshlet_vertices.size() << " vertices, " << segment_meshlet_triangles.size()
                << " triangles). Physics segments after compact: " << dynamic_strands->segments.size()
                << ", pairs: " << dynamic_strands->segment_pairs.size() << ".");
  return true;
}

bool DsKineticVoronoiMeshing::ResetMeshletsToGpu() {
  if (!HasMeshedSegmentMeshlets() && !EnsureCpuMeshletsFromGpu()) {
    EVOENGINE_ERROR("Reset meshlets: no meshed segment meshlets available. Run meshing first.");
    return false;
  }
  if (!strand_tree) {
    EVOENGINE_ERROR("Reset meshlets: strand tree is missing.");
    return false;
  }

  if (segment_meshlets_.empty() && tree_mesher_ && !tree_mesher_->getSegmentMeshlets().empty()) {
    segment_meshlets_ = tree_mesher_->getSegmentMeshlets();
    meshing_neighbor_indices_ = tree_mesher_->getMeshingNeighborIndices();
  }
  tree_mesher_->getSegmentMeshlets() = segment_meshlets_;
  tree_mesher_->getMeshingNeighborIndices() = meshing_neighbor_indices_;

  segment_meshlet_vertices.clear();
  segment_meshlet_triangles.clear();
  PrepareAndPopulateGpuMeshletBuffers(segment_meshlets_, strand_tree->getPhysicsStrandToSegmentIndices(),
                                      tree_mesher_->getMeshingStrandToSegmentIndices(), meshing_neighbor_indices_,
                                      tree_mesher_->getMeshingToPhysicsSegmentIndices(), meshlets_root_transform_);
  Upload();
  UpdateBindings();
  EVOENGINE_LOG("Reset meshlets to GPU (pristine visuals for remaining physics segments; "
                << "OUTSIDE segments removed by intersection are not restored). " << segment_meshlet_vertices.size()
                << " vertices, " << segment_meshlet_triangles.size() << " triangles.");
  return true;
}

void DsKineticVoronoiMeshing::DownloadPhysicsSegmentsAndPairs() {
  if (!dynamic_strands) {
    return;
  }
  if (!dynamic_strands->segments.empty()) {
    dynamic_strands->device_segments_buffer->DownloadVector(dynamic_strands->segments,
                                                            dynamic_strands->segments.size());
  }
  if (!dynamic_strands->segment_pairs.empty()) {
    dynamic_strands->device_segment_pairs_buffer->DownloadVector(dynamic_strands->segment_pairs,
                                                                 dynamic_strands->segment_pairs.size());
  }
  if (!dynamic_strands->segment_data_list.empty()) {
    dynamic_strands->device_segment_data_list_buffer->DownloadVector(dynamic_strands->segment_data_list,
                                                                     dynamic_strands->segment_data_list.size());
  }
  if (!dynamic_strands->strands.empty()) {
    dynamic_strands->device_strands_buffer->DownloadVector(dynamic_strands->strands, dynamic_strands->strands.size());
  }
  if (!dynamic_strands->foliage.empty()) {
    dynamic_strands->device_foliage_buffer->DownloadVector(dynamic_strands->foliage, dynamic_strands->foliage.size());
  }
}

void DsKineticVoronoiMeshing::UploadPhysicsSegmentsAndPairs() {
  if (!dynamic_strands) {
    return;
  }
  dynamic_strands->device_segments_buffer->UploadVector(dynamic_strands->segments);
  dynamic_strands->device_segments_buffer->SetDebugName("Segments Buffer");
  dynamic_strands->device_segment_pairs_buffer->UploadVector(dynamic_strands->segment_pairs);
  dynamic_strands->device_segment_pairs_buffer->SetDebugName("Segment Pairs Buffer");
  dynamic_strands->device_segment_data_list_buffer->UploadVector(dynamic_strands->segment_data_list);
  dynamic_strands->device_segment_data_list_buffer->SetDebugName("Segment Data List Buffer");
  dynamic_strands->device_strands_buffer->UploadVector(dynamic_strands->strands);
  dynamic_strands->device_strands_buffer->SetDebugName("Strands Buffer");
  dynamic_strands->device_foliage_buffer->UploadVector(dynamic_strands->foliage);
  dynamic_strands->device_foliage_buffer->SetDebugName("Foliage Buffer");
}

void DsKineticVoronoiMeshing::FinalizeStrandConnectivityAndPairMaterials() {
  if (!dynamic_strands || !strand_tree) {
    return;
  }
  auto& segments = dynamic_strands->segments;
  auto& segment_data_list = dynamic_strands->segment_data_list;
  auto& segment_pairs = dynamic_strands->segment_pairs;
  auto& strands = dynamic_strands->strands;
  const auto& physics_strand_map = strand_tree->getPhysicsStrandToSegmentIndices();

  // Refresh prev/next and strand endpoints from physics strand maps.
  for (auto& segment : segments) {
    segment.prev_handle = -1;
    segment.next_handle = -1;
  }
  for (size_t strand_id = 0; strand_id < physics_strand_map.size(); ++strand_id) {
    if (strand_id >= strands.size()) {
      continue;
    }
    auto& strand = strands[strand_id];
    const auto& slots = physics_strand_map[strand_id];
    if (slots.empty()) {
      strand.begin_segment_handle = -1;
      strand.end_segment_handle = -1;
      strand.front_propagate_begin_segment_handle = -1;
      strand.back_propagate_begin_segment_handle = -1;
      strand.alternative_front_propagate_begin_segment_handle = -1;
      strand.alternative_back_propagate_begin_segment_handle = -1;
      continue;
    }
    strand.begin_segment_handle = slots.front();
    strand.end_segment_handle = slots.back();
    strand.front_propagate_begin_segment_handle = strand.begin_segment_handle;
    strand.back_propagate_begin_segment_handle = strand.end_segment_handle;
    strand.alternative_front_propagate_begin_segment_handle = strand.begin_segment_handle;
    strand.alternative_back_propagate_begin_segment_handle = strand.end_segment_handle;
    for (size_t slot = 0; slot < slots.size(); ++slot) {
      const int id = slots[slot];
      if (id < 0 || static_cast<size_t>(id) >= segments.size()) {
        continue;
      }
      if (slot > 0) {
        const int prev = slots[slot - 1];
        if (prev >= 0 && static_cast<size_t>(prev) < segments.size()) {
          segments[static_cast<size_t>(id)].prev_handle = prev;
          segments[static_cast<size_t>(prev)].next_handle = id;
        }
      }
    }
  }

  // Initialize pair materials / rest pose for all recomputed pairs.
  const uint32_t vertical_count = dynamic_strands->connection_segment_pair_size;
  for (size_t pair_index = 0; pair_index < segment_pairs.size(); ++pair_index) {
    auto& pair = segment_pairs[pair_index];
    if (pair.segment0_handle < 0 || pair.segment1_handle < 0 ||
        static_cast<size_t>(pair.segment0_handle) >= segments.size() ||
        static_cast<size_t>(pair.segment1_handle) >= segments.size()) {
      continue;
    }
    const bool direct_connection = pair_index < vertical_count;
    InitializeCompactedSegmentPair(pair, segments[static_cast<size_t>(pair.segment0_handle)],
                                   segments[static_cast<size_t>(pair.segment1_handle)], direct_connection,
                                   initialize_parameters_, nullptr);
  }
  RecountSegmentPairHandles(segments, segment_data_list);
}

namespace {

using QuantizedVec3 = std::tuple<int64_t, int64_t, int64_t>;
using QuantizedEdge = std::pair<QuantizedVec3, QuantizedVec3>;

QuantizedVec3 QuantizePosition(const glm::dvec3& p) {
  constexpr double kScale = 1.0e5;
  return {static_cast<int64_t>(std::llround(p.x * kScale)), static_cast<int64_t>(std::llround(p.y * kScale)),
          static_cast<int64_t>(std::llround(p.z * kScale))};
}

QuantizedEdge MakeQuantizedEdge(const glm::dvec3& a, const glm::dvec3& b) {
  QuantizedVec3 qa = QuantizePosition(a);
  QuantizedVec3 qb = QuantizePosition(b);
  if (qb < qa) {
    std::swap(qa, qb);
  }
  return {qa, qb};
}

void FitSegmentLengthToMeshOnParentAxis(DynamicStrands::GpuSegment& segment, const kinDS::VoronoiMesh& mesh,
                                        const GlobalTransform& root_transform) {
  const glm::vec3 p0 = segment.particle0.x0;
  const glm::vec3 p1 = segment.particle1.x0;
  glm::vec3 axis = p1 - p0;
  const float axis_len = glm::length(axis);
  if (axis_len < 1e-8f || mesh.getVertexCount() == 0) {
    return;
  }
  axis /= axis_len;

  float t_min = std::numeric_limits<float>::max();
  float t_max = std::numeric_limits<float>::lowest();
  for (const auto& v : mesh.getVertices()) {
    const glm::vec3 world = root_transform.TransformPoint(
        glm::vec3(static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z)));
    const float t = glm::dot(world - p0, axis);
    t_min = std::min(t_min, t);
    t_max = std::max(t_max, t);
  }
  t_min = glm::clamp(t_min, 0.f, axis_len);
  t_max = glm::clamp(t_max, 0.f, axis_len);
  if (t_max - t_min < 1e-5f) {
    const float mid = 0.5f * (t_min + t_max);
    t_min = glm::max(0.f, mid - 5e-5f);
    t_max = glm::min(axis_len, mid + 5e-5f);
  }

  const glm::vec3 new_p0 = p0 + axis * t_min;
  const glm::vec3 new_p1 = p0 + axis * t_max;
  segment.particle0.x0 = segment.particle0.x = segment.particle0.last_x = new_p0;
  segment.particle1.x0 = segment.particle1.x = segment.particle1.last_x = new_p1;
  segment.rest_length = glm::length(new_p1 - new_p0);
}

float MeshAxialCenterOnParentAxis(const kinDS::VoronoiMesh& mesh, const glm::vec3& p0, const glm::vec3& axis,
                                  const GlobalTransform& root_transform) {
  if (mesh.getVertexCount() == 0) {
    return 0.f;
  }
  double sum = 0.0;
  for (const auto& v : mesh.getVertices()) {
    const glm::vec3 world = root_transform.TransformPoint(
        glm::vec3(static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z)));
    sum += static_cast<double>(glm::dot(world - p0, axis));
  }
  return static_cast<float>(sum / static_cast<double>(mesh.getVertexCount()));
}

}  // namespace

size_t DsKineticVoronoiMeshing::SplitIntersectingMeshletsByConnectedComponents(
    const std::vector<size_t>& intersecting_meshing_indices) {
  if (!dynamic_strands || !tree_mesher_ || !strand_tree || intersecting_meshing_indices.empty()) {
    return 0;
  }

  auto& meshes = tree_mesher_->getSegmentMeshlets();
  auto& neighbor_lists = tree_mesher_->getMeshingNeighborIndices();
  auto meshing_to_physics = tree_mesher_->getMeshingToPhysicsSegmentIndices();
  auto meshing_strand_map = tree_mesher_->getMeshingStrandToSegmentIndices();
  auto& physics_strand_map = strand_tree->getPhysicsStrandToSegmentIndices();
  auto& segments = dynamic_strands->segments;
  auto& segment_data_list = dynamic_strands->segment_data_list;

  size_t meshlets_split = 0;
  size_t extra_segments = 0;

  struct EdgeHash {
    size_t operator()(const QuantizedEdge& e) const noexcept {
      size_t h = 0;
      const auto mix = [&](const QuantizedVec3& v) {
        h ^= std::hash<int64_t>{}(std::get<0>(v)) + 0x9e3779b9 + (h << 6) + (h >> 2);
        h ^= std::hash<int64_t>{}(std::get<1>(v)) + 0x9e3779b9 + (h << 6) + (h >> 2);
        h ^= std::hash<int64_t>{}(std::get<2>(v)) + 0x9e3779b9 + (h << 6) + (h >> 2);
      };
      mix(e.first);
      mix(e.second);
      return h;
    }
  };

  for (const size_t meshing_id : intersecting_meshing_indices) {
    if (meshing_id >= meshes.size() || meshing_id >= neighbor_lists.size()) {
      continue;
    }
    if (meshes[meshing_id].getTriangleCount() == 0) {
      continue;
    }

    auto split = kinDS::VoronoiMesh::splitIntoConnectedComponents(meshes[meshing_id], neighbor_lists[meshing_id]);
    if (split.meshes.size() <= 1) {
      continue;
    }

    if (meshing_id >= meshing_to_physics.size()) {
      continue;
    }
    const size_t parent_physics_id = meshing_to_physics[meshing_id];
    if (parent_physics_id == static_cast<size_t>(-1) || parent_physics_id >= segments.size()) {
      continue;
    }

    // Find strand + slot for this meshing id.
    size_t strand_id = static_cast<size_t>(-1);
    size_t slot_index = static_cast<size_t>(-1);
    for (size_t s = 0; s < meshing_strand_map.size(); ++s) {
      for (size_t slot = 0; slot < meshing_strand_map[s].size(); ++slot) {
        if (meshing_strand_map[s][slot] == meshing_id) {
          strand_id = s;
          slot_index = slot;
          break;
        }
      }
      if (strand_id != static_cast<size_t>(-1)) {
        break;
      }
    }
    if (strand_id == static_cast<size_t>(-1) || strand_id >= physics_strand_map.size() ||
        slot_index >= physics_strand_map[strand_id].size()) {
      EVOENGINE_WARNING("SplitIntersectingMeshlets: could not find strand slot for meshing id " << meshing_id);
      continue;
    }

    const DynamicStrands::GpuSegment parent_segment = segments[parent_physics_id];
    const DynamicStrands::GpuSegmentData parent_data = segment_data_list[parent_physics_id];
    const glm::vec3 axis_origin = parent_segment.particle0.x0;
    glm::vec3 axis = parent_segment.particle1.x0 - parent_segment.particle0.x0;
    const float axis_len = glm::length(axis);
    if (axis_len > 1e-8f) {
      axis /= axis_len;
    } else {
      axis = glm::vec3(1.f, 0.f, 0.f);
    }

    // Sort components along the parent axis.
    std::vector<size_t> order(split.meshes.size());
    std::iota(order.begin(), order.end(), 0);
    std::stable_sort(order.begin(), order.end(), [&](size_t a, size_t b) {
      return MeshAxialCenterOnParentAxis(split.meshes[a], axis_origin, axis, meshlets_root_transform_) <
             MeshAxialCenterOnParentAxis(split.meshes[b], axis_origin, axis, meshlets_root_transform_);
    });

    std::vector<size_t> child_meshing_ids(order.size());
    std::vector<size_t> child_physics_ids(order.size());
    std::vector<std::unordered_set<QuantizedEdge, EdgeHash>> child_edges(order.size());

    // Component 0 (sorted) replaces the original meshlet/physics segment.
    {
      const size_t src = order[0];
      meshes[meshing_id] = std::move(split.meshes[src]);
      neighbor_lists[meshing_id] = std::move(split.face_neighbors[src]);
      child_meshing_ids[0] = meshing_id;
      child_physics_ids[0] = parent_physics_id;
      FitSegmentLengthToMeshOnParentAxis(segments[parent_physics_id], meshes[meshing_id], meshlets_root_transform_);
      child_edges[0] = [&]() {
        std::unordered_set<QuantizedEdge, EdgeHash> edges;
        const auto& verts = meshes[meshing_id].getVertices();
        const auto& tris = meshes[meshing_id].getTriangles();
        for (size_t i = 0; i + 2 < tris.size(); i += 3) {
          edges.insert(MakeQuantizedEdge(verts[tris[i]], verts[tris[i + 1]]));
          edges.insert(MakeQuantizedEdge(verts[tris[i + 1]], verts[tris[i + 2]]));
          edges.insert(MakeQuantizedEdge(verts[tris[i + 2]], verts[tris[i]]));
        }
        return edges;
      }();
    }

    for (size_t ci = 1; ci < order.size(); ++ci) {
      const size_t src = order[ci];
      const size_t new_meshing_id = meshes.size();
      meshes.push_back(std::move(split.meshes[src]));
      neighbor_lists.push_back(std::move(split.face_neighbors[src]));

      DynamicStrands::GpuSegment child_segment = parent_segment;
      FitSegmentLengthToMeshOnParentAxis(child_segment, meshes.back(), meshlets_root_transform_);
      child_segment.prev_handle = -1;
      child_segment.next_handle = -1;
      const size_t new_physics_id = segments.size();
      segments.push_back(child_segment);
      DynamicStrands::GpuSegmentData child_data = parent_data;
      for (int& handle : child_data.pair_handles) {
        handle = -1;
      }
      segment_data_list.push_back(child_data);

      meshing_to_physics.push_back(new_physics_id);
      child_meshing_ids[ci] = new_meshing_id;
      child_physics_ids[ci] = new_physics_id;
      {
        std::unordered_set<QuantizedEdge, EdgeHash> edges;
        const auto& verts = meshes.back().getVertices();
        const auto& tris = meshes.back().getTriangles();
        for (size_t i = 0; i + 2 < tris.size(); i += 3) {
          edges.insert(MakeQuantizedEdge(verts[tris[i]], verts[tris[i + 1]]));
          edges.insert(MakeQuantizedEdge(verts[tris[i + 1]], verts[tris[i + 2]]));
          edges.insert(MakeQuantizedEdge(verts[tris[i + 2]], verts[tris[i]]));
        }
        child_edges[ci] = std::move(edges);
      }
      ++extra_segments;
    }

    // Insert sibling slots into strand maps (replace parent slot with sorted children).
    {
      auto& physics_slots = physics_strand_map[strand_id];
      auto& meshing_slots = meshing_strand_map[strand_id];
      std::vector<int> new_physics_slots;
      std::vector<size_t> new_meshing_slots;
      new_physics_slots.reserve(physics_slots.size() + child_meshing_ids.size());
      new_meshing_slots.reserve(meshing_slots.size() + child_meshing_ids.size());
      for (size_t slot = 0; slot < physics_slots.size(); ++slot) {
        if (slot == slot_index) {
          for (size_t ci = 0; ci < child_meshing_ids.size(); ++ci) {
            new_physics_slots.push_back(static_cast<int>(child_physics_ids[ci]));
            new_meshing_slots.push_back(child_meshing_ids[ci]);
          }
        } else {
          new_physics_slots.push_back(physics_slots[slot]);
          if (slot < meshing_slots.size()) {
            new_meshing_slots.push_back(meshing_slots[slot]);
          }
        }
      }
      physics_slots = std::move(new_physics_slots);
      meshing_slots = std::move(new_meshing_slots);
    }

    // Retarget reverse neighbor tags that still point at the original meshing id.
    for (size_t other = 0; other < neighbor_lists.size(); ++other) {
      if (std::find(child_meshing_ids.begin(), child_meshing_ids.end(), other) != child_meshing_ids.end()) {
        continue;
      }
      if (other >= meshes.size()) {
        continue;
      }
      auto& other_neighbors = neighbor_lists[other];
      const auto& other_mesh = meshes[other];
      const auto& other_verts = other_mesh.getVertices();
      const auto& other_tris = other_mesh.getTriangles();
      for (size_t face = 0; face < other_neighbors.size(); ++face) {
        if (other_neighbors[face] != static_cast<int>(meshing_id)) {
          continue;
        }
        if (face * 3 + 2 >= other_tris.size()) {
          continue;
        }
        const QuantizedEdge e0 =
            MakeQuantizedEdge(other_verts[other_tris[face * 3]], other_verts[other_tris[face * 3 + 1]]);
        const QuantizedEdge e1 =
            MakeQuantizedEdge(other_verts[other_tris[face * 3 + 1]], other_verts[other_tris[face * 3 + 2]]);
        const QuantizedEdge e2 =
            MakeQuantizedEdge(other_verts[other_tris[face * 3 + 2]], other_verts[other_tris[face * 3]]);

        int best_ci = 0;
        int best_hits = -1;
        for (size_t ci = 0; ci < child_edges.size(); ++ci) {
          int hits = 0;
          if (child_edges[ci].count(e0))
            ++hits;
          if (child_edges[ci].count(e1))
            ++hits;
          if (child_edges[ci].count(e2))
            ++hits;
          if (hits > best_hits) {
            best_hits = hits;
            best_ci = static_cast<int>(ci);
          }
        }
        other_neighbors[face] = static_cast<int>(child_meshing_ids[static_cast<size_t>(best_ci)]);
      }
    }

    ++meshlets_split;
  }

  tree_mesher_->setMeshingToPhysicsSegmentIndices(std::move(meshing_to_physics));
  tree_mesher_->setMeshingStrandToSegmentIndices(std::move(meshing_strand_map));

  if (meshlets_split > 0) {
    EVOENGINE_LOG("SplitIntersectingMeshlets: split " << meshlets_split << " INTERSECT meshlet(s), created "
                                                      << extra_segments << " extra segment(s).");
  }
  return extra_segments;
}

void DsKineticVoronoiMeshing::CompactSurvivingPhysicsSegments(const std::vector<size_t>& outside_meshing_indices,
                                                              const bool rebuild_pairs) {
  if (!dynamic_strands || !tree_mesher_ || !strand_tree) {
    return;
  }

  auto& segments = dynamic_strands->segments;
  auto& segment_data_list = dynamic_strands->segment_data_list;
  auto& segment_pairs = dynamic_strands->segment_pairs;
  auto& strands = dynamic_strands->strands;
  auto& foliage = dynamic_strands->foliage;

  if (segments.empty() || segment_data_list.size() != segments.size()) {
    EVOENGINE_ERROR("CompactSurvivingPhysicsSegments: segments / segment_data_list size mismatch or empty.");
    return;
  }

  const size_t old_segment_count = segments.size();
  std::vector<int> old_to_new(old_segment_count, -1);

  // Mark OUTSIDE physics IDs for removal.
  std::vector<char> remove(old_segment_count, 0);
  const auto& meshing_to_physics = tree_mesher_->getMeshingToPhysicsSegmentIndices();
  size_t outside_physics_count = 0;
  for (const size_t meshing_index : outside_meshing_indices) {
    if (meshing_index >= meshing_to_physics.size()) {
      continue;
    }
    const size_t physics_id = meshing_to_physics[meshing_index];
    if (physics_id == static_cast<size_t>(-1) || physics_id >= old_segment_count) {
      continue;
    }
    if (!remove[physics_id]) {
      remove[physics_id] = 1;
      ++outside_physics_count;
    }
  }

  if (outside_physics_count == 0) {
    EVOENGINE_LOG("CompactSurvivingPhysicsSegments: no OUTSIDE physics segments to remove.");
    return;
  }

  // Build dense remap for survivors.
  int next_new = 0;
  for (size_t old_id = 0; old_id < old_segment_count; ++old_id) {
    if (!remove[old_id]) {
      old_to_new[old_id] = next_new++;
    }
  }
  const size_t new_segment_count = static_cast<size_t>(next_new);

  auto remap_segment = [&](int handle) -> int {
    if (handle < 0 || static_cast<size_t>(handle) >= old_to_new.size()) {
      return -1;
    }
    return old_to_new[static_cast<size_t>(handle)];
  };

  // Compact segments + segment_data in order.
  std::vector<DynamicStrands::GpuSegment> new_segments;
  std::vector<DynamicStrands::GpuSegmentData> new_segment_data;
  new_segments.reserve(new_segment_count);
  new_segment_data.reserve(new_segment_count);
  for (size_t old_id = 0; old_id < old_segment_count; ++old_id) {
    if (remove[old_id]) {
      continue;
    }
    new_segments.push_back(segments[old_id]);
    new_segment_data.push_back(segment_data_list[old_id]);
  }

  // Splice prev/next around removed neighbors, then remap.
  for (size_t new_id = 0; new_id < new_segments.size(); ++new_id) {
    auto& seg = new_segments[new_id];
    // Walk prev until a survivor (or none).
    int prev = seg.prev_handle;
    while (prev >= 0 && static_cast<size_t>(prev) < remove.size() && remove[static_cast<size_t>(prev)]) {
      prev = segments[static_cast<size_t>(prev)].prev_handle;
    }
    int next = seg.next_handle;
    while (next >= 0 && static_cast<size_t>(next) < remove.size() && remove[static_cast<size_t>(next)]) {
      next = segments[static_cast<size_t>(next)].next_handle;
    }
    seg.prev_handle = remap_segment(prev);
    seg.next_handle = remap_segment(next);
  }

  // Compact pairs: keep only pairs whose both endpoints survive; preserve integrity/strain.
  // Skipped when rebuild_pairs is false — @ref RecomputeSegmentPairs will rebuild from mesh.
  std::vector<DynamicStrands::GpuSegmentPair> new_pairs;
  if (rebuild_pairs) {
    new_pairs.reserve(segment_pairs.size());

    for (size_t old_pair = 0; old_pair < segment_pairs.size(); ++old_pair) {
      const auto& pair = segment_pairs[old_pair];
      const int s0 = remap_segment(pair.segment0_handle);
      const int s1 = remap_segment(pair.segment1_handle);
      if (s0 < 0 || s1 < 0) {
        continue;
      }
      DynamicStrands::GpuSegmentPair kept = pair;
      kept.segment0_handle = s0;
      kept.segment1_handle = s1;
      new_pairs.push_back(kept);
    }
  }

  // Rebuild pair_handles from strand order for [0]/[1], then pack laterals into [2+].
  // connection_segment_pair_size becomes the vertical prefix count after rebuild.

  // Update strand segment maps: drop OUTSIDE slots, remap physics IDs.
  auto& physics_strand_map = strand_tree->getPhysicsStrandToSegmentIndices();
  auto meshing_strand_map = tree_mesher_->getMeshingStrandToSegmentIndices();
  for (size_t strand_id = 0; strand_id < physics_strand_map.size(); ++strand_id) {
    std::vector<int> new_physics_slots;
    std::vector<size_t> new_meshing_slots;
    const auto& old_physics_slots = physics_strand_map[strand_id];
    const auto& old_meshing_slots =
        strand_id < meshing_strand_map.size() ? meshing_strand_map[strand_id] : std::vector<size_t>{};
    const size_t slot_count = std::min(old_physics_slots.size(), old_meshing_slots.size());
    new_physics_slots.reserve(slot_count);
    new_meshing_slots.reserve(slot_count);
    for (size_t slot = 0; slot < slot_count; ++slot) {
      const int old_physics = old_physics_slots[slot];
      if (old_physics < 0 || static_cast<size_t>(old_physics) >= remove.size() ||
          remove[static_cast<size_t>(old_physics)]) {
        continue;
      }
      new_physics_slots.push_back(old_to_new[static_cast<size_t>(old_physics)]);
      new_meshing_slots.push_back(old_meshing_slots[slot]);
    }
    // Also keep any trailing physics-only slots that survived (should be rare).
    for (size_t slot = slot_count; slot < old_physics_slots.size(); ++slot) {
      const int old_physics = old_physics_slots[slot];
      if (old_physics < 0 || static_cast<size_t>(old_physics) >= remove.size() ||
          remove[static_cast<size_t>(old_physics)]) {
        continue;
      }
      new_physics_slots.push_back(old_to_new[static_cast<size_t>(old_physics)]);
    }
    physics_strand_map[strand_id] = std::move(new_physics_slots);
    if (strand_id < meshing_strand_map.size()) {
      meshing_strand_map[strand_id] = std::move(new_meshing_slots);
    }
  }
  tree_mesher_->setMeshingStrandToSegmentIndices(std::move(meshing_strand_map));

  // Remap meshing_to_physics; OUTSIDE -> -1.
  auto new_meshing_to_physics = tree_mesher_->getMeshingToPhysicsSegmentIndices();
  for (size_t meshing_id = 0; meshing_id < new_meshing_to_physics.size(); ++meshing_id) {
    const size_t old_physics = new_meshing_to_physics[meshing_id];
    if (old_physics == static_cast<size_t>(-1) || old_physics >= old_to_new.size()) {
      new_meshing_to_physics[meshing_id] = static_cast<size_t>(-1);
      continue;
    }
    const int mapped = old_to_new[old_physics];
    new_meshing_to_physics[meshing_id] = mapped < 0 ? static_cast<size_t>(-1) : static_cast<size_t>(mapped);
  }
  tree_mesher_->setMeshingToPhysicsSegmentIndices(std::move(new_meshing_to_physics));

  // Clear / rebuild pair_handles.
  for (auto& data : new_segment_data) {
    data.particle0_position_correction = glm::vec3(0.f);
    data.particle1_position_correction = glm::vec3(0.f);
    data.q_correction = glm::quat(1.f, 0.f, 0.f, 0.f);
    for (int& handle : data.pair_handles) {
      handle = -1;
    }
  }

  std::vector<DynamicStrands::GpuSegmentPair> ordered_pairs;
  uint32_t vertical_pair_count = 0;

  // Always refresh strand begin/end segment handles from remapped maps.
  const auto& remapped_physics_strands = strand_tree->getPhysicsStrandToSegmentIndices();
  for (size_t strand_id = 0; strand_id < remapped_physics_strands.size(); ++strand_id) {
    if (strand_id >= strands.size()) {
      continue;
    }
    auto& strand = strands[strand_id];
    strand.begin_segment_pair_handle = -1;
    strand.end_segment_pair_handle = -1;
    strand.front_propagate_begin_segment_pair_handle = -1;
    strand.back_propagate_begin_segment_pair_handle = -1;
    strand.alternative_front_propagate_begin_segment_pair_handle = -1;
    strand.alternative_back_propagate_begin_segment_pair_handle = -1;

    const auto& physics_slots = remapped_physics_strands[strand_id];
    if (physics_slots.empty()) {
      strand.begin_segment_handle = -1;
      strand.end_segment_handle = -1;
      strand.front_propagate_begin_segment_handle = -1;
      strand.back_propagate_begin_segment_handle = -1;
      strand.alternative_front_propagate_begin_segment_handle = -1;
      strand.alternative_back_propagate_begin_segment_handle = -1;
      continue;
    }

    strand.begin_segment_handle = physics_slots.front();
    strand.end_segment_handle = physics_slots.back();
    strand.front_propagate_begin_segment_handle = strand.begin_segment_handle;
    strand.back_propagate_begin_segment_handle = strand.end_segment_handle;
    strand.alternative_front_propagate_begin_segment_handle = strand.begin_segment_handle;
    strand.alternative_back_propagate_begin_segment_handle = strand.end_segment_handle;
  }

  if (rebuild_pairs) {
    ordered_pairs.reserve(new_pairs.size());

    // Pass 1: vertical pairs from remapped strand maps.
    for (size_t strand_id = 0; strand_id < remapped_physics_strands.size(); ++strand_id) {
      if (strand_id >= strands.size()) {
        continue;
      }
      auto& strand = strands[strand_id];
      const auto& physics_slots = remapped_physics_strands[strand_id];
      if (physics_slots.empty()) {
        continue;
      }

      for (size_t slot = 0; slot + 1 < physics_slots.size(); ++slot) {
        const int below = physics_slots[slot];
        const int above = physics_slots[slot + 1];
        if (below < 0 || above < 0 || static_cast<size_t>(below) >= new_segment_data.size() ||
            static_cast<size_t>(above) >= new_segment_data.size()) {
          continue;
        }
        // Prefer an existing surviving pair with matching endpoints (preserve integrity).
        int found_old_pair = -1;
        for (size_t p = 0; p < new_pairs.size(); ++p) {
          const auto& cand = new_pairs[p];
          if ((cand.segment0_handle == below && cand.segment1_handle == above) ||
              (cand.segment0_handle == above && cand.segment1_handle == below)) {
            found_old_pair = static_cast<int>(p);
            break;
          }
        }
        const bool direct_connection = found_old_pair >= 0;
        DynamicStrands::GpuSegmentPair vertical{};
        const DynamicStrands::GpuSegmentPair* material_template = nullptr;
        if (found_old_pair >= 0) {
          vertical = new_pairs[static_cast<size_t>(found_old_pair)];
          // Mark consumed so lateral pass can skip.
          new_pairs[static_cast<size_t>(found_old_pair)].segment0_handle = -1;
          new_pairs[static_cast<size_t>(found_old_pair)].segment1_handle = -1;
        } else {
          for (const auto& cand : new_pairs) {
            if (cand.segment0_handle < 0 || cand.segment1_handle < 0) {
              continue;
            }
            if (cand.segment0_handle == below || cand.segment1_handle == below || cand.segment0_handle == above ||
                cand.segment1_handle == above) {
              if (!SegmentPairNeedsMaterialInit(cand)) {
                material_template = &cand;
                break;
              }
            }
          }
        }
        vertical.segment0_handle = below;
        vertical.segment1_handle = above;
        InitializeCompactedSegmentPair(vertical, new_segments[static_cast<size_t>(below)],
                                       new_segments[static_cast<size_t>(above)], direct_connection,
                                       initialize_parameters_, material_template);
        const int pair_handle = static_cast<int>(ordered_pairs.size());
        ordered_pairs.push_back(vertical);
        new_segment_data[static_cast<size_t>(below)].pair_handles[1] = pair_handle;
        new_segment_data[static_cast<size_t>(above)].pair_handles[0] = pair_handle;
        if (strand.begin_segment_pair_handle == -1) {
          strand.begin_segment_pair_handle = pair_handle;
        }
        strand.end_segment_pair_handle = pair_handle;
        ++vertical_pair_count;
      }

      // Propagate pair bookkeeping (same as RecomputeSegmentPairs).
      strand.front_propagate_begin_segment_pair_handle = -1;
      strand.back_propagate_begin_segment_pair_handle = -1;
      strand.alternative_front_propagate_begin_segment_pair_handle = -1;
      strand.alternative_back_propagate_begin_segment_pair_handle = -1;
      if (strand.begin_segment_handle == -1 || strand.begin_segment_pair_handle == -1) {
        continue;
      }
      strand.front_propagate_begin_segment_pair_handle = strand.begin_segment_pair_handle;
      if (strand.begin_segment_pair_handle == strand.end_segment_pair_handle) {
        strand.alternative_front_propagate_begin_segment_pair_handle = strand.begin_segment_pair_handle;
        continue;
      }
      strand.alternative_front_propagate_begin_segment_pair_handle = strand.begin_segment_pair_handle + 1;
      const int connection_size = strand.end_segment_pair_handle - strand.begin_segment_pair_handle + 1;
      strand.back_propagate_begin_segment_pair_handle =
          connection_size % 2 == 0 ? strand.end_segment_pair_handle : strand.end_segment_pair_handle - 1;
      strand.alternative_back_propagate_begin_segment_pair_handle =
          connection_size % 2 == 0 ? strand.end_segment_pair_handle - 1 : strand.end_segment_pair_handle;
    }

    // Pass 2: remaining pairs as laterals into slots >= 2.
    std::vector<uint32_t> pair_slot_offsets(new_segment_data.size(), 2);
    for (const auto& cand : new_pairs) {
      if (cand.segment0_handle < 0 || cand.segment1_handle < 0) {
        continue;  // consumed as vertical
      }
      const int a = cand.segment0_handle;
      const int b = cand.segment1_handle;
      if (static_cast<size_t>(a) >= new_segment_data.size() || static_cast<size_t>(b) >= new_segment_data.size()) {
        continue;
      }
      auto& first_slot = pair_slot_offsets[static_cast<size_t>(a)];
      auto& second_slot = pair_slot_offsets[static_cast<size_t>(b)];
      if (first_slot >= BUNDLE_MAX_CONNECTION || second_slot >= BUNDLE_MAX_CONNECTION) {
        continue;
      }
      const int pair_handle = static_cast<int>(ordered_pairs.size());
      ordered_pairs.push_back(cand);
      new_segment_data[static_cast<size_t>(a)].pair_handles[first_slot] = pair_handle;
      new_segment_data[static_cast<size_t>(b)].pair_handles[second_slot] = pair_handle;
      ++first_slot;
      ++second_slot;
    }

    // Refresh lateral-pair rest pose / zero strains; material props were copied with the pair.
    for (size_t pair_index = vertical_pair_count; pair_index < ordered_pairs.size(); ++pair_index) {
      auto& pair = ordered_pairs[pair_index];
      if (pair.segment0_handle < 0 || pair.segment1_handle < 0) {
        continue;
      }
      InitializeCompactedSegmentPair(pair, new_segments[static_cast<size_t>(pair.segment0_handle)],
                                     new_segments[static_cast<size_t>(pair.segment1_handle)], false,
                                     initialize_parameters_, nullptr);
    }

    RecountSegmentPairHandles(new_segments, new_segment_data);
  } else {
    ordered_pairs.clear();
    segment_pairs.clear();
  }

  for (auto& segment : new_segments) {
    segment.shear_stretch_strain = 0.f;
    segment.torque = glm::vec3(0.f);
    segment.angular_v = glm::vec3(0.f);
    segment.particle0.v = glm::vec3(0.f);
    segment.particle1.v = glm::vec3(0.f);
    segment.particle0.acceleration = glm::vec3(0.f);
    segment.particle1.acceleration = glm::vec3(0.f);
  }

  // Remap foliage.
  std::vector<DynamicStrands::GpuLeaf> new_foliage;
  new_foliage.reserve(foliage.size());
  for (auto leaf : foliage) {
    const int mapped = remap_segment(leaf.segment_handle);
    if (mapped < 0) {
      continue;
    }
    leaf.segment_handle = mapped;
    new_foliage.push_back(leaf);
  }

  // Remap meshlet GPU buffers that still reference old physics IDs.
  for (auto& vertex : segment_meshlet_vertices) {
    const int mapped = remap_segment(static_cast<int>(vertex.segment_index));
    vertex.segment_index = mapped < 0 ? 0u : static_cast<unsigned int>(mapped);
  }
  for (auto& triangle : segment_meshlet_triangles) {
    if (triangle.neighbor_segment_index >= 0) {
      const int mapped = remap_segment(triangle.neighbor_segment_index);
      triangle.neighbor_segment_index = mapped < 0 ? -3 : mapped;
    }
    triangle.segment_pair_index = -1;
    if (rebuild_pairs && triangle.neighbor_segment_index >= 0) {
      // Find owning segment from first vertex.
      if (triangle.vertex_index0 < segment_meshlet_vertices.size()) {
        const int owner = static_cast<int>(segment_meshlet_vertices[triangle.vertex_index0].segment_index);
        if (owner >= 0 && static_cast<size_t>(owner) < new_segment_data.size()) {
          for (const int pair_handle : new_segment_data[static_cast<size_t>(owner)].pair_handles) {
            if (pair_handle < 0 || static_cast<size_t>(pair_handle) >= ordered_pairs.size()) {
              continue;
            }
            const auto& pair = ordered_pairs[static_cast<size_t>(pair_handle)];
            if (pair.segment0_handle == triangle.neighbor_segment_index ||
                pair.segment1_handle == triangle.neighbor_segment_index) {
              triangle.segment_pair_index = pair_handle;
              break;
            }
          }
        }
      }
    }
  }

  segments = std::move(new_segments);
  segment_data_list = std::move(new_segment_data);
  if (rebuild_pairs) {
    segment_pairs = std::move(ordered_pairs);
  } else {
    segment_pairs.clear();
  }
  foliage = std::move(new_foliage);
  dynamic_strands->connection_segment_pair_size = vertical_pair_count;

  // Pivot constraints cache segment indices from Initialize; remap or invalidate after compact.
  for (const auto& constraint : dynamic_strands->constraints) {
    if (const auto pivot_transform = std::dynamic_pointer_cast<DsPivotTransform>(constraint)) {
      pivot_transform->RemapSegmentIndices(old_to_new);
    } else if (const auto pivot_axis = std::dynamic_pointer_cast<DsPivotAxis>(constraint)) {
      pivot_axis->RemapSegmentIndices(old_to_new);
    } else if (const auto pivot_point = std::dynamic_pointer_cast<DsPivotPoint>(constraint)) {
      pivot_point->RemapSegmentIndices(old_to_new);
    }
  }

  EVOENGINE_LOG("CompactSurvivingPhysicsSegments: removed " << outside_physics_count << " OUTSIDE segment(s); now "
                                                            << segments.size() << " segments, " << segment_pairs.size()
                                                            << " pairs (" << vertical_pair_count << " vertical).");
}

void DsKineticVoronoiMeshing::RecomputeSegmentPairs(const kinDS::TreeMesher& tree_mesher) {
  const auto& meshes = tree_mesher.getSegmentMeshlets();
  const auto& meshing_neighbor_indices = tree_mesher.getMeshingNeighborIndices();
  const auto& meshing_to_physics_segment_indices = tree_mesher.getMeshingToPhysicsSegmentIndices();
  const auto& meshing_strand_to_segment_indices = tree_mesher.getMeshingStrandToSegmentIndices();
  const auto& physics_strand_to_segment_indices = strand_tree->getPhysicsStrandToSegmentIndices();

  auto& segment_data_list = dynamic_strands->segment_data_list;
  auto& segment_pairs = dynamic_strands->segment_pairs;
  auto& strands = dynamic_strands->strands;

  segment_pairs.clear();
  for (auto& segment_data : segment_data_list) {
    for (int& pair_handle : segment_data.pair_handles) {
      pair_handle = -1;
    }
  }

  // Collect physics-segment neighbors implied by meshlet face adjacency.
  const auto collect_physics_neighbors = [&](const size_t meshing_segment_id) {
    std::unordered_set<int> neighbors;
    if (meshing_segment_id >= meshes.size() || meshing_segment_id >= meshing_neighbor_indices.size()) {
      return neighbors;
    }
    const auto& triangles = meshes[meshing_segment_id].getTriangles();
    const auto& face_neighbors = meshing_neighbor_indices[meshing_segment_id];
    for (size_t triangle_vertex_index = 0; triangle_vertex_index < triangles.size(); triangle_vertex_index += 3) {
      const size_t face_index = triangle_vertex_index / 3;
      if (face_index >= face_neighbors.size()) {
        continue;
      }
      const int meshing_neighbor_segment_index = face_neighbors[face_index];
      if (meshing_neighbor_segment_index < 0 ||
          static_cast<size_t>(meshing_neighbor_segment_index) >= meshing_to_physics_segment_indices.size()) {
        continue;
      }
      const int physics_neighbor =
          static_cast<int>(meshing_to_physics_segment_indices[static_cast<size_t>(meshing_neighbor_segment_index)]);
      if (physics_neighbor >= 0) {
        neighbors.insert(physics_neighbor);
      }
    }
    return neighbors;
  };

  // Pass 1: same-strand vertical pairs.
  // Convention (matches DynamicStrandsInitialize / StiffRod):
  //   pair_handles[0] = below (prev / proximal)
  //   pair_handles[1] = above (next / distal)
  //   GpuSegmentPair.segment0 = below, segment1 = above
  for (size_t strand_id = 0; strand_id < physics_strand_to_segment_indices.size(); ++strand_id) {
    if (strand_id >= strands.size() || strand_id >= meshing_strand_to_segment_indices.size()) {
      continue;
    }
    auto& strand = strands[strand_id];
    strand.begin_segment_pair_handle = -1;
    strand.end_segment_pair_handle = -1;

    const auto& physics_segments = physics_strand_to_segment_indices[strand_id];
    const auto& meshing_segments = meshing_strand_to_segment_indices[strand_id];
    const size_t segment_count = std::min(physics_segments.size(), meshing_segments.size());
    if (segment_count < 2) {
      continue;
    }

    for (size_t segment_no = 0; segment_no + 1 < segment_count; ++segment_no) {
      const int below_physics_id = physics_segments[segment_no];
      const int above_physics_id = physics_segments[segment_no + 1];
      const size_t below_meshing_id = meshing_segments[segment_no];
      const size_t above_meshing_id = meshing_segments[segment_no + 1];
      if (below_physics_id < 0 || above_physics_id < 0 ||
          static_cast<size_t>(below_physics_id) >= segment_data_list.size() ||
          static_cast<size_t>(above_physics_id) >= segment_data_list.size()) {
        continue;
      }

      const auto below_neighbors = collect_physics_neighbors(below_meshing_id);
      const auto above_neighbors = collect_physics_neighbors(above_meshing_id);
      const bool adjacent = below_neighbors.count(above_physics_id) > 0 || above_neighbors.count(below_physics_id) > 0;
      if (!adjacent) {
        continue;
      }

      const int pair_handle = static_cast<int>(segment_pairs.size());
      DynamicStrands::GpuSegmentPair segment_pair{};
      segment_pair.segment0_handle = below_physics_id;
      segment_pair.segment1_handle = above_physics_id;
      segment_pairs.emplace_back(segment_pair);

      segment_data_list[below_physics_id].pair_handles[1] = pair_handle;
      segment_data_list[above_physics_id].pair_handles[0] = pair_handle;

      if (strand.begin_segment_pair_handle == -1) {
        strand.begin_segment_pair_handle = pair_handle;
      }
      strand.end_segment_pair_handle = pair_handle;
    }
  }

  dynamic_strands->connection_segment_pair_size = static_cast<uint32_t>(segment_pairs.size());

  // Mirror initialize: refresh strand pair-propagation bookkeeping from the new vertical range.
  for (auto& strand : strands) {
    strand.front_propagate_begin_segment_pair_handle = -1;
    strand.back_propagate_begin_segment_pair_handle = -1;
    strand.alternative_front_propagate_begin_segment_pair_handle = -1;
    strand.alternative_back_propagate_begin_segment_pair_handle = -1;
    if (strand.begin_segment_handle == -1 || strand.begin_segment_pair_handle == -1) {
      continue;
    }
    strand.front_propagate_begin_segment_pair_handle = strand.begin_segment_pair_handle;
    if (strand.begin_segment_pair_handle == strand.end_segment_pair_handle) {
      strand.alternative_front_propagate_begin_segment_pair_handle = strand.begin_segment_pair_handle;
      continue;
    }
    strand.alternative_front_propagate_begin_segment_pair_handle = strand.begin_segment_pair_handle + 1;

    const int connection_size = strand.end_segment_pair_handle - strand.begin_segment_pair_handle + 1;
    strand.back_propagate_begin_segment_pair_handle =
        connection_size % 2 == 0 ? strand.end_segment_pair_handle : strand.end_segment_pair_handle - 1;
    strand.alternative_back_propagate_begin_segment_pair_handle =
        connection_size % 2 == 0 ? strand.end_segment_pair_handle - 1 : strand.end_segment_pair_handle;
  }

  // Pass 2: remaining mesh neighbors as lateral pairs starting at slot 2.
  std::vector<uint32_t> pair_slot_offsets(segment_data_list.size(), 2);
  for (size_t strand_id = 0; strand_id < physics_strand_to_segment_indices.size(); ++strand_id) {
    if (strand_id >= meshing_strand_to_segment_indices.size()) {
      continue;
    }
    const auto& physics_segments = physics_strand_to_segment_indices[strand_id];
    const auto& meshing_segments = meshing_strand_to_segment_indices[strand_id];
    const size_t segment_count = std::min(physics_segments.size(), meshing_segments.size());
    for (size_t segment_no = 0; segment_no < segment_count; ++segment_no) {
      const int physics_segment_id = physics_segments[segment_no];
      if (physics_segment_id < 0 || static_cast<size_t>(physics_segment_id) >= segment_data_list.size()) {
        continue;
      }
      const int below_physics_id = segment_no > 0 ? physics_segments[segment_no - 1] : -1;
      const int above_physics_id = segment_no + 1 < segment_count ? physics_segments[segment_no + 1] : -1;

      for (const int physics_neighbor_id : collect_physics_neighbors(meshing_segments[segment_no])) {
        if (physics_neighbor_id == physics_segment_id || physics_neighbor_id == below_physics_id ||
            physics_neighbor_id == above_physics_id) {
          continue;
        }
        if (physics_neighbor_id < 0 || static_cast<size_t>(physics_neighbor_id) >= segment_data_list.size()) {
          continue;
        }
        // Create each undirected pair once.
        if (physics_neighbor_id <= physics_segment_id) {
          continue;
        }

        auto& first_slot = pair_slot_offsets[static_cast<size_t>(physics_segment_id)];
        auto& second_slot = pair_slot_offsets[static_cast<size_t>(physics_neighbor_id)];
        if (first_slot >= BUNDLE_MAX_CONNECTION || second_slot >= BUNDLE_MAX_CONNECTION) {
          continue;
        }

        const int pair_handle = static_cast<int>(segment_pairs.size());
        DynamicStrands::GpuSegmentPair segment_pair{};
        segment_pair.segment0_handle = physics_segment_id;
        segment_pair.segment1_handle = physics_neighbor_id;
        segment_pairs.emplace_back(segment_pair);

        segment_data_list[physics_segment_id].pair_handles[first_slot] = pair_handle;
        segment_data_list[physics_neighbor_id].pair_handles[second_slot] = pair_handle;
        ++first_slot;
        ++second_slot;
      }
    }
  }

  EVOENGINE_LOG("Recomputed segment pairs from mesh: "
                << dynamic_strands->connection_segment_pair_size << " vertical, "
                << (segment_pairs.size() - dynamic_strands->connection_segment_pair_size) << " lateral (total "
                << segment_pairs.size() << ").");
}

void DsKineticVoronoiMeshing::WriteMeshingFailureStatistics(const std::string& failure_message) {
  if (!meshing_settings.collect_meshing_statistics) {
    return;
  }

  const double cutoff = meshing_settings.alpha_cutoff;
  const double alpha = cutoff * cutoff;

  std::string experiment_tag = meshing_settings.meshing_statistics_experiment_name;
  if (experiment_tag.empty()) {
    experiment_tag = "Unknown";
    if (const auto scene = ApplicationContext::Get().GetActiveScene()) {
      if (const auto* demo_owners = scene->UnsafeGetPrivateComponentOwnersList<DynamicStrandsDemo>()) {
        for (const Entity& entity : *demo_owners) {
          if (const auto demo = scene->GetOrSetPrivateComponent<DynamicStrandsDemo>(entity).lock()) {
            experiment_tag = DynamicStrandsDemo::DemoTypeExportFolderName(demo->demo_type);
            break;
          }
        }
      }
    }
  }
  std::replace(experiment_tag.begin(), experiment_tag.end(), ' ', '_');
  for (char& c : experiment_tag) {
    if (c == '+' || c == '/' || c == '\\' || c == ':') {
      c = '_';
    }
  }

  // Prefer whatever stats were collected before the failure (still on the current tree_mesher_).
  if (tree_mesher_) {
    if (kinDS::Statistics* stats = tree_mesher_->getMeshingStatistics()) {
      // Close the open section so the in-progress section still contributes a row.
      if (stats->isCollecting()) {
        stats->endRun();
      }
      if (!stats->empty() || !stats->eventList().empty()) {
        stats->setTotalsAlpha(alpha);
        stats->setTotalsFailure(failure_message);
        stats->setFilenameExperimentTag(experiment_tag);
        tree_mesher_->writeCollectedMeshingStatistics();
        meshing_settings.meshing_statistics_experiment_name.clear();
        return;
      }
    }
  }

  // No collected events — still emit a total-only failure CSV with the same naming scheme.
  kinDS::Statistics failure_stats;
  failure_stats.setTotalsAlpha(alpha);
  failure_stats.setTotalsFailure(failure_message);
  failure_stats.setFilenameExperimentTag(std::move(experiment_tag));
  failure_stats.writeCsv(tree_mesher_ ? tree_mesher_->getSettings().meshing_statistics_csv_path
                                      : EcoSysLabMetadataPath("meshing_statistics.csv"));
  meshing_settings.meshing_statistics_experiment_name.clear();
}

bool DsKineticVoronoiMeshing::RunMeshingAlgorithm(
    const std::vector<std::vector<glm::dvec2>>& support_points,
    std::vector<std::vector<double>>& subdivisions_by_strand,
    std::vector<std::vector<int>>& physics_strand_to_segment_indices,
    const std::vector<std::vector<glm::dmat4>>& transforms_by_height_and_branch, const GlobalTransform& root_transform,
    const std::vector<std::vector<size_t>>& branch_indices,
    std::vector<std::vector<std::vector<size_t>>>& strands_by_branch_id, const float min_segment_length,
    const float max_segment_length) {
  last_meshing_succeeded_ = false;
  std::vector<float> bottom_boundary_distances_by_strand_id(physics_strand_to_segment_indices.size());
  std::vector<float> top_boundary_distances_by_strand_id(physics_strand_to_segment_indices.size());

  for (size_t strand_id = 0; strand_id < physics_strand_to_segment_indices.size(); strand_id++) {
    int bottom_segment_id = physics_strand_to_segment_indices[strand_id].front();
    int top_segment_id = physics_strand_to_segment_indices[strand_id].back();

    bottom_boundary_distances_by_strand_id[strand_id] = dynamic_strands->segments[bottom_segment_id].boundary_distance;
    top_boundary_distances_by_strand_id[strand_id] = dynamic_strands->segments[top_segment_id].boundary_distance;
  }

  EVOENGINE_LOG("Starting Kinetic Delaunay Voronoi Meshing...");

  for (size_t strand_id = 0; strand_id < subdivisions_by_strand.size(); ++strand_id) {
    size_t non_positive_count = 0;
    double example_t = 0.0;
    for (const double t : subdivisions_by_strand[strand_id]) {
      if (t <= 0.0) {
        if (non_positive_count == 0) {
          example_t = t;
        }
        ++non_positive_count;
      }
    }
    if (non_positive_count > 0) {
      EVOENGINE_WARNING("Subdivision parameter list for strand "
                        << strand_id << " contains " << non_positive_count
                        << " value(s) with t<=0 (e.g. t=" << example_t
                        << ") before meshing; these schedule a subdiv at bootstrap and yield zero-length meshlets.");
    }
  }

  // Preserve prior successful mesh so a TreeMesher failure leaves GPU/CPU buffers unchanged.
  const auto previous_strand_tree = strand_tree;
  const auto previous_tree_mesher = tree_mesher_;
  const auto previous_vertices = segment_meshlet_vertices;
  const auto previous_triangles = segment_meshlet_triangles;
  const auto previous_vertex_metadata = segment_meshlet_vertex_metadata;
  const auto previous_face_metadata = segment_meshlet_face_metadata;
  const auto previous_meshlets = segment_meshlets_;
  const auto previous_neighbors = meshing_neighbor_indices_;
  const auto previous_boundary_distances = boundary_distances_by_vertex;
  const auto previous_root_transform = meshlets_root_transform_;

  std::vector<DynamicStrands::GpuSegmentPair> previous_segment_pairs;
  std::vector<std::array<int, BUNDLE_MAX_CONNECTION>> previous_pair_handles;
  std::vector<std::pair<int, int>> previous_strand_pair_ends;
  uint32_t previous_connection_segment_pair_size = 0;
  const bool snapshot_pairs = meshing_settings.recompute_segment_pairs && dynamic_strands != nullptr;
  if (snapshot_pairs) {
    previous_segment_pairs = dynamic_strands->segment_pairs;
    previous_connection_segment_pair_size = dynamic_strands->connection_segment_pair_size;
    previous_pair_handles.resize(dynamic_strands->segment_data_list.size());
    for (size_t i = 0; i < dynamic_strands->segment_data_list.size(); ++i) {
      std::copy_n(dynamic_strands->segment_data_list[i].pair_handles, BUNDLE_MAX_CONNECTION,
                  previous_pair_handles[i].begin());
    }
    previous_strand_pair_ends.resize(dynamic_strands->strands.size());
    for (size_t i = 0; i < dynamic_strands->strands.size(); ++i) {
      previous_strand_pair_ends[i] = {dynamic_strands->strands[i].begin_segment_pair_handle,
                                      dynamic_strands->strands[i].end_segment_pair_handle};
    }
  }

  const auto restore_previous_meshing_state = [&]() {
    strand_tree = previous_strand_tree;
    tree_mesher_ = previous_tree_mesher;
    segment_meshlet_vertices = previous_vertices;
    segment_meshlet_triangles = previous_triangles;
    segment_meshlet_vertex_metadata = previous_vertex_metadata;
    segment_meshlet_face_metadata = previous_face_metadata;
    segment_meshlets_ = previous_meshlets;
    meshing_neighbor_indices_ = previous_neighbors;
    boundary_distances_by_vertex = previous_boundary_distances;
    meshlets_root_transform_ = previous_root_transform;
    if (snapshot_pairs) {
      dynamic_strands->segment_pairs = previous_segment_pairs;
      dynamic_strands->connection_segment_pair_size = previous_connection_segment_pair_size;
      for (size_t i = 0; i < previous_pair_handles.size() && i < dynamic_strands->segment_data_list.size(); ++i) {
        std::copy_n(previous_pair_handles[i].begin(), BUNDLE_MAX_CONNECTION,
                    dynamic_strands->segment_data_list[i].pair_handles);
      }
      for (size_t i = 0; i < previous_strand_pair_ends.size() && i < dynamic_strands->strands.size(); ++i) {
        dynamic_strands->strands[i].begin_segment_pair_handle = previous_strand_pair_ends[i].first;
        dynamic_strands->strands[i].end_segment_pair_handle = previous_strand_pair_ends[i].second;
      }
    }
  };

  try {
    strand_tree =
        std::make_shared<kinDS::StrandTree>(support_points, subdivisions_by_strand, physics_strand_to_segment_indices,
                                            transforms_by_height_and_branch, branch_indices, strands_by_branch_id);

    MeshingInputHashStats hash_stats = ComputeMeshingInputHashStats(
        support_points, subdivisions_by_strand, physics_strand_to_segment_indices, transforms_by_height_and_branch,
        root_transform, branch_indices, strands_by_branch_id);
    hash_stats.min_segment_length = min_segment_length;
    hash_stats.max_segment_length = max_segment_length;
    last_meshing_input_hash_ = hash_stats.input_hash;
    last_meshing_root_transform_ = root_transform.value;

    if (meshing_settings.dry_run_strand_tree_only) {
      LogMeshingInputHashStats(hash_stats);
      EVOENGINE_LOG("Dry run: strand tree prepared, skipping meshing algorithm.");
      last_meshing_succeeded_ = true;
      return true;
    }

    tree_mesher_ = std::make_shared<kinDS::TreeMesher>(*strand_tree, MakeKineticParallelFor());
    tree_mesher_->getSettings().transform_mesh_at_construction = true;
    tree_mesher_->getSettings().mesh_cap_at_start = true;
    tree_mesher_->getSettings().alpha_cutoff = meshing_settings.alpha_cutoff;
    tree_mesher_->getSettings().branch_alpha_cutoff = meshing_settings.branch_alpha_cutoff;
    tree_mesher_->getSettings().look_ahead = meshing_settings.look_ahead;
    tree_mesher_->getSettings().collect_meshing_statistics = meshing_settings.collect_meshing_statistics;
    // Defer CSV write until after T-junction closure so totals can include meshlet tri/vert counts.
    tree_mesher_->getSettings().defer_meshing_statistics_write = meshing_settings.collect_meshing_statistics;
    tree_mesher_->getSettings().flush_meshing_statistics_each_section =
        meshing_settings.collect_meshing_statistics && meshing_settings.flush_meshing_statistics_each_section;
    tree_mesher_->getSettings().meshing_statistics_csv_path = EcoSysLabMetadataPath("meshing_statistics.csv");
    {
      std::string experiment_tag = meshing_settings.meshing_statistics_experiment_name;
      std::replace(experiment_tag.begin(), experiment_tag.end(), ' ', '_');
      for (char& c : experiment_tag) {
        if (c == '+' || c == '/' || c == '\\' || c == ':') {
          c = '_';
        }
      }
      tree_mesher_->getSettings().meshing_statistics_experiment_tag = std::move(experiment_tag);
    }
    tree_mesher_->getSettings().store_mesh_metadata = meshing_settings.store_mesh_metadata;
    tree_mesher_->getSettings().export_separate_contributor_objects =
        meshing_settings.export_separate_contributor_objects;

    LogMeshingInputHashStats(hash_stats);
    const std::string& input_hash = hash_stats.input_hash;
    const std::filesystem::path buffer_dir = MeshingBufferDirectory();
    std::filesystem::path bin_path = buffer_dir / (input_hash + ".bin");
    std::filesystem::path yml_path = buffer_dir / (input_hash + ".yml");

    const std::filesystem::path force_bin = meshing_settings.force_load_meshing_buffer_bin;
    const std::string expected_hash = meshing_settings.force_load_expected_hash.empty()
                                          ? (force_bin.empty() ? std::string{} : force_bin.stem().string())
                                          : meshing_settings.force_load_expected_hash;
    if (!force_bin.empty()) {
      if (expected_hash.empty() || expected_hash == input_hash) {
        EVOENGINE_LOG("Force-load MeshBuffers: recomputed hash " << input_hash << " matches expected"
                                                                 << (expected_hash.empty() ? "" : (" " + expected_hash))
                                                                 << ".");
      } else {
        EVOENGINE_WARNING("Force-load MeshBuffers: recomputed hash " << input_hash << " differs from expected "
                                                                     << expected_hash
                                                                     << " (subdivision text precision / growth drift). "
                                                                        "Loading stored buffer anyway.");
      }
    }
    // One-shot force-load flags (cleared so subsequent remeshes use normal cache lookup).
    meshing_settings.force_load_meshing_buffer_bin.clear();
    meshing_settings.force_load_expected_hash.clear();

    const auto warn_segment_count_mismatch =
        [&](const std::vector<std::vector<size_t>>& meshing_strand_to_segment_indices) {
          for (size_t strand_id = 0; strand_id < physics_strand_to_segment_indices.size(); ++strand_id) {
            if (strand_id >= meshing_strand_to_segment_indices.size()) {
              continue;
            }
            if (meshing_strand_to_segment_indices[strand_id].size() !=
                physics_strand_to_segment_indices[strand_id].size()) {
              EVOENGINE_WARNING("Meshing algorithm resulted in "
                                << meshing_strand_to_segment_indices[strand_id].size() << " segments for strand "
                                << strand_id << ", but the physics simulation has "
                                << physics_strand_to_segment_indices[strand_id].size() << ". There are "
                                << subdivisions_by_strand[strand_id].size() << " subdivision parameters in range ["
                                << subdivisions_by_strand[strand_id].front() << ", "
                                << subdivisions_by_strand[strand_id].back() << "].");
            }
          }
        };

    const auto debug_export_meshes = [&]() {
      if (!meshing_settings.debug_export_meshes) {
        return;
      }
      EVOENGINE_LOG("Exporting Kinetic Delaunay Voronoi Meshes for Debugging...");
      tree_mesher_->exportMeshlets(kinDS::MeshletExportMode::PerSegment, "meshlets");
      tree_mesher_->exportMeshlets(kinDS::MeshletExportMode::Combined, "combined_mesh.obj");
      EVOENGINE_LOG("Kinetic Delaunay Voronoi Meshes exported.");
    };

    bool loaded_from_cache = false;
    bool rebuild_gpu_from_meshlets = false;
    const auto try_load_cache_at = [&](const std::filesystem::path& load_bin_path,
                                       const std::filesystem::path& load_yml_path) -> bool {
      std::vector<GpuMeshletVertex> gpu_vertices;
      std::vector<GpuMeshletTriangle> gpu_triangles;
      std::vector<kinDS::VoronoiMesh> meshlets;
      std::vector<std::vector<int>> neighbors;
      std::vector<size_t> meshing_to_physics;
      std::vector<std::vector<size_t>> strand_to_segment;
      if (!LoadMeshingBuffer(load_bin_path, root_transform, gpu_vertices, gpu_triangles, meshlets, neighbors,
                             meshing_to_physics, strand_to_segment)) {
        EVOENGINE_WARNING("Meshing buffer " << load_bin_path.string()
                                            << " exists but could not be loaded; remeshing (cache miss).");
        return false;
      }
      meshlets_root_transform_ = root_transform;
      segment_meshlet_vertices = std::move(gpu_vertices);
      segment_meshlet_triangles = std::move(gpu_triangles);
      tree_mesher_->setMeshingToPhysicsSegmentIndices(std::move(meshing_to_physics));
      tree_mesher_->setMeshingStrandToSegmentIndices(std::move(strand_to_segment));
      if (!meshlets.empty()) {
        segment_meshlets_ = std::move(meshlets);
        meshing_neighbor_indices_ = std::move(neighbors);
        RepairBarkNeighborTagsFromMaterials(segment_meshlets_, meshing_neighbor_indices_);
        tree_mesher_->getSegmentMeshlets() = segment_meshlets_;
        tree_mesher_->getMeshingNeighborIndices() = meshing_neighbor_indices_;
        rebuild_gpu_from_meshlets = true;
      } else {
        meshing_neighbor_indices_ = std::move(neighbors);
        tree_mesher_->getMeshingNeighborIndices() = meshing_neighbor_indices_;
        EnsureCpuMeshletsFromGpu();
      }
      if (!HasMeshedSegmentMeshlets()) {
        EVOENGINE_WARNING("Meshing buffer " << load_bin_path.string()
                                            << " loaded GPU data but CPU meshlets could not be restored; remeshing.");
        return false;
      }
      warn_segment_count_mismatch(tree_mesher_->getMeshingStrandToSegmentIndices());
      EVOENGINE_LOG("Meshing buffer cache hit " << load_bin_path.stem().string() << " ("
                                                << segment_meshlet_vertices.size() << " vertices, "
                                                << segment_meshlet_triangles.size() << " triangles).");
      if (UpdateMeshingBufferYmlOnCacheHit(load_yml_path, hash_stats, meshing_settings.meshing_buffer_description)) {
        EVOENGINE_LOG("Updated meshing buffer metadata for " << load_yml_path.stem().string() << ".");
      }
      return true;
    };

    if (!force_bin.empty() && std::filesystem::exists(force_bin)) {
      const std::filesystem::path force_yml = force_bin.parent_path() / (force_bin.stem().string() + ".yml");
      EVOENGINE_LOG("Force-loading MeshBuffers from " << force_bin.string() << " (recomputed hash " << input_hash
                                                      << ").");
      loaded_from_cache = try_load_cache_at(force_bin, force_yml);
      if (!loaded_from_cache) {
        EVOENGINE_ERROR("Force-load failed for " << force_bin.string() << "; falling back to hash lookup / remesh.");
      }
    } else if (!force_bin.empty()) {
      EVOENGINE_ERROR("Force-load MeshBuffers path missing: " << force_bin.string()
                                                              << "; falling back to hash lookup / remesh.");
    }

    if (!loaded_from_cache && !meshing_settings.override_meshing_buffer && std::filesystem::exists(bin_path)) {
      loaded_from_cache = try_load_cache_at(bin_path, yml_path);
    } else if (!loaded_from_cache && !meshing_settings.override_meshing_buffer &&
               meshing_settings.alpha_cutoff == kLegacyDefaultAlphaCutoff) {
      // Pre-alpha_cutoff hashes omitted the cutoff (always 10). Fall back and migrate to the new name.
      MeshingInputHashStats legacy_stats = ComputeMeshingInputHashStats(
          support_points, subdivisions_by_strand, physics_strand_to_segment_indices, transforms_by_height_and_branch,
          root_transform, branch_indices, strands_by_branch_id, /*include_alpha_cutoff=*/false);
      legacy_stats.min_segment_length = min_segment_length;
      legacy_stats.max_segment_length = max_segment_length;
      const std::filesystem::path legacy_bin = buffer_dir / (legacy_stats.input_hash + ".bin");
      const std::filesystem::path legacy_yml = buffer_dir / (legacy_stats.input_hash + ".yml");
      if (legacy_stats.input_hash != input_hash && std::filesystem::exists(legacy_bin)) {
        EVOENGINE_LOG("Meshing buffer cache miss for " << input_hash << "; trying legacy hash "
                                                       << legacy_stats.input_hash << " (alpha_cutoff=10).");
        if (try_load_cache_at(legacy_bin, legacy_yml)) {
          loaded_from_cache = true;
          if (TryMigrateMeshingBufferCacheFiles(legacy_bin, legacy_yml, bin_path, yml_path, hash_stats)) {
            // Paths now point at the migrated files for any later metadata updates.
          } else {
            // Still usable from the legacy path if rename failed.
            bin_path = legacy_bin;
            yml_path = legacy_yml;
          }
        }
      } else if (!loaded_from_cache) {
        EVOENGINE_LOG("Meshing buffer cache miss " << input_hash << " (no file at " << bin_path.string() << ").");
      }
    } else if (!loaded_from_cache && meshing_settings.override_meshing_buffer) {
      EVOENGINE_LOG("Meshing buffer override enabled; remeshing for hash " << input_hash << ".");
    } else if (!loaded_from_cache) {
      EVOENGINE_LOG("Meshing buffer cache miss " << input_hash << " (no file at " << bin_path.string() << ").");
    }

    if (!loaded_from_cache) {
      auto& meshes = tree_mesher_->runMeshingAlgorithm(meshing_settings.debug_svg);

      // Keep pristine copies for later Intersect button runs (no clipping during meshing).
      segment_meshlets_ = meshes;
      meshing_neighbor_indices_ = tree_mesher_->getMeshingNeighborIndices();
      RepairBarkNeighborTagsFromMaterials(segment_meshlets_, meshing_neighbor_indices_);
      tree_mesher_->getSegmentMeshlets() = segment_meshlets_;
      tree_mesher_->getMeshingNeighborIndices() = meshing_neighbor_indices_;
      meshlets_root_transform_ = root_transform;
      warn_segment_count_mismatch(tree_mesher_->getMeshingStrandToSegmentIndices());
    }

    const auto& meshing_to_physics_segment_indices = tree_mesher_->getMeshingToPhysicsSegmentIndices();
    const auto& meshing_strand_to_segment_indices = tree_mesher_->getMeshingStrandToSegmentIndices();

    if (meshing_settings.recompute_segment_pairs) {
      RecomputeSegmentPairs(*tree_mesher_);
    }

    // Prepare CPU meshlets (T-junctions + bark) before GPU upload / buffer save so the cached
    // CPU meshlets match the GPU buffers.
    if (!loaded_from_cache || meshing_settings.recompute_segment_pairs || rebuild_gpu_from_meshlets) {
      PrepareSegmentMeshletsForGpu(segment_meshlets_, meshing_neighbor_indices_, meshing_to_physics_segment_indices,
                                   root_transform);
      tree_mesher_->getSegmentMeshlets() = segment_meshlets_;
      tree_mesher_->getMeshingNeighborIndices() = meshing_neighbor_indices_;

      segment_meshlet_vertices.clear();
      segment_meshlet_triangles.clear();
      PopulateGpuMeshletBuffers(segment_meshlets_, physics_strand_to_segment_indices, meshing_strand_to_segment_indices,
                                meshing_neighbor_indices_, meshing_to_physics_segment_indices, root_transform);
    }

    if (!loaded_from_cache) {
      auto& boundary_mesh = tree_mesher_->getBoundaryMesh();
      auto& boundary_vertex_to_strand_id = tree_mesher_->getBoundaryVertexToStrandId();

      boundary_distances_by_vertex.resize(boundary_mesh.getVertexCount(), 0.0f);
      for (size_t i = 0; i < boundary_mesh.getVertexCount(); i++) {
        size_t strand_id = boundary_vertex_to_strand_id[i];

        // as a heuristic, just use the bottom boundary distance if height is <= 0, otherwise use the top boundary
        // distance
        float height = boundary_mesh.getVertices()[i][2];
        if (height <= 0.0f) {
          boundary_distances_by_vertex[i] = bottom_boundary_distances_by_strand_id[strand_id];
        } else {
          boundary_distances_by_vertex[i] = top_boundary_distances_by_strand_id[strand_id];
        }
      }

      if (SaveMeshingBuffer(bin_path, yml_path, hash_stats, root_transform, segment_meshlet_vertices,
                            segment_meshlet_triangles, segment_meshlets_, meshing_neighbor_indices_,
                            meshing_to_physics_segment_indices, meshing_strand_to_segment_indices)) {
        EVOENGINE_LOG("Saved Kinetic Voronoi mesh buffer " << input_hash << " to " << bin_path.string());
      }

      EVOENGINE_LOG("Kinetic Delaunay Voronoi Meshing completed.");
    }

    try {
      debug_export_meshes();
    } catch (const std::exception& exception) {
      EVOENGINE_WARNING("Meshing succeeded but debug mesh export failed: " << exception.what());
    } catch (...) {
      EVOENGINE_WARNING("Meshing succeeded but debug mesh export failed with an unknown error.");
    }
    last_meshing_succeeded_ = true;
    CaptureInitialMeshletVolumes();
    return true;
  } catch (const std::exception& exception) {
    EVOENGINE_ERROR("Kinetic Voronoi meshing failed: " << exception.what()
                                                       << "; cancelling update, previous mesh buffers kept.");
    WriteMeshingFailureStatistics(exception.what());
    restore_previous_meshing_state();
    last_meshing_succeeded_ = false;
    return false;
  } catch (...) {
    EVOENGINE_ERROR(
        "Kinetic Voronoi meshing failed with an unknown error; cancelling update, previous mesh buffers "
        "kept.");
    WriteMeshingFailureStatistics("unknown error");
    restore_previous_meshing_state();
    last_meshing_succeeded_ = false;
    return false;
  }
}

void DsKineticVoronoiMeshing::PrepareSegmentMeshletsForGpu(
    std::vector<kinDS::VoronoiMesh>& meshes, std::vector<std::vector<int>>& meshing_neighbor_indices,
    const std::vector<size_t>& meshing_to_physics_segment_indices, const GlobalTransform& root_transform) {
  // Glue / extraction can leave T-junctions across and inside segment meshlets; close them before bark.
  kinDS::closeCrossMeshletTJunctions(meshes, meshing_neighbor_indices);
  kinDS::closeIntraMeshletTJunctions(meshes, meshing_neighbor_indices);

  // Write deferred meshing statistics with mesh counts taken after T-junction fixes (before bark edits).
  if (meshing_settings.collect_meshing_statistics && tree_mesher_) {
    if (kinDS::Statistics* stats = tree_mesher_->getMeshingStatistics()) {
      if (!stats->empty() || !stats->eventList().empty()) {
        size_t triangle_count = 0;
        size_t vertex_count = 0;
        for (const kinDS::VoronoiMesh& mesh : meshes) {
          triangle_count += mesh.getTriangleCount();
          vertex_count += mesh.getVertexCount();
        }
        const double cutoff = meshing_settings.alpha_cutoff;
        stats->setTotalsAlpha(cutoff * cutoff);
        stats->setTotalsMeshCounts(triangle_count, vertex_count);

        std::string experiment_tag = meshing_settings.meshing_statistics_experiment_name;
        if (experiment_tag.empty()) {
          experiment_tag = "Unknown";
          if (const auto scene = ApplicationContext::Get().GetActiveScene()) {
            if (const auto* demo_owners = scene->UnsafeGetPrivateComponentOwnersList<DynamicStrandsDemo>()) {
              for (const Entity& entity : *demo_owners) {
                if (const auto demo = scene->GetOrSetPrivateComponent<DynamicStrandsDemo>(entity).lock()) {
                  experiment_tag = DynamicStrandsDemo::DemoTypeExportFolderName(demo->demo_type);
                  break;
                }
              }
            }
          }
        }
        std::replace(experiment_tag.begin(), experiment_tag.end(), ' ', '_');
        for (char& c : experiment_tag) {
          if (c == '+' || c == '/' || c == '\\' || c == ':') {
            c = '_';
          }
        }
        stats->setFilenameExperimentTag(std::move(experiment_tag));
        tree_mesher_->writeCollectedMeshingStatistics();
        meshing_settings.meshing_statistics_experiment_name.clear();
      }
    }
  }

  if (meshing_settings.bark_subdivide) {
    SubdivideBarkTrianglesOnce(meshes, meshing_neighbor_indices);
  }

  const std::vector<uint8_t> meshlet_has_bark = BuildMeshletHasBarkFlags(meshing_neighbor_indices);
  SmoothBarkMeshPositions(meshes, meshing_neighbor_indices, meshlet_has_bark, meshing_settings.bark_smooth_iterations,
                          meshing_settings.bark_smooth_strength, meshing_settings.bark_smooth_lock_boundary,
                          meshing_settings.bark_smooth_uvs);
  AverageSharedBarkSeamNormals(meshes, meshing_neighbor_indices, meshlet_has_bark, meshing_to_physics_segment_indices,
                               dynamic_strands->segments, dynamic_strands->segment_pairs,
                               dynamic_strands->segment_data_list);
  FixTriangleCapBarkUVs(meshes, meshing_neighbor_indices, meshlet_has_bark);
  bark_debug_mesh_ = BuildBarkDebugMesh(meshes, meshing_neighbor_indices, meshlet_has_bark, root_transform);
  has_bark_debug_mesh_ = bark_debug_mesh_.getTriangleCount() > 0;
}

void DsKineticVoronoiMeshing::PopulateGpuMeshletBuffers(
    const std::vector<kinDS::VoronoiMesh>& meshes,
    const std::vector<std::vector<int>>& physics_strand_to_segment_indices,
    const std::vector<std::vector<size_t>>& meshing_strand_to_segment_indices,
    const std::vector<std::vector<int>>& meshing_neighbor_indices,
    const std::vector<size_t>& meshing_to_physics_segment_indices, const GlobalTransform& root_transform) {
  // CPU-only metadata parallel to the GPU buffers (never uploaded).
  segment_meshlet_vertex_metadata.clear();
  segment_meshlet_face_metadata.clear();
  const bool keep_metadata = meshing_settings.store_mesh_metadata;
  if (keep_metadata) {
    segment_meshlet_vertex_metadata.reserve(segment_meshlet_vertices.capacity());
    segment_meshlet_face_metadata.reserve(segment_meshlet_triangles.capacity());
  }

  for (size_t strand_id = 0; strand_id < physics_strand_to_segment_indices.size(); ++strand_id) {
    for (size_t segment_no = 0; segment_no < meshing_strand_to_segment_indices[strand_id].size(); ++segment_no) {
      const size_t meshing_segment_id = meshing_strand_to_segment_indices[strand_id][segment_no];
      if (meshing_segment_id >= meshes.size()) {
        EVOENGINE_ERROR("meshing_segment_id out of bounds: " << meshing_segment_id
                                                             << "; upper bound is: " << meshes.size())
        continue;
      }

      const auto& mesh = meshes[meshing_segment_id];
      const int physics_segment_id = physics_strand_to_segment_indices[strand_id][segment_no];

      if (mesh.getNormalMode() != kinDS::NormalMode::PerTriangleCorner ||
          mesh.getNormals().size() != mesh.getTriangles().size()) {
        EVOENGINE_ERROR("PopulateGpuMeshletBuffers: meshlet "
                        << meshing_segment_id << " is not PerTriangleCorner (normals=" << mesh.getNormals().size()
                        << ", corners=" << mesh.getTriangles().size() << "); refusing to upload conflated indices.");
        continue;
      }

      const size_t vertex_offset = segment_meshlet_vertices.size();
      const auto& mesh_vertex_metadata = mesh.getVertexMetadata();
      for (size_t local_vi = 0; local_vi < mesh.getVertices().size(); ++local_vi) {
        const auto& v = mesh.getVertices()[local_vi];
        GpuSegmentMeshletVertex vertex;
        vertex.x0 = root_transform.TransformPoint(glm::vec3(v[0], v[1], v[2]));
        vertex.x = vertex.x0;
        vertex.segment_index = physics_segment_id;
        segment_meshlet_vertices.push_back(vertex);
        if (keep_metadata) {
          segment_meshlet_vertex_metadata.push_back(
              local_vi < mesh_vertex_metadata.size() ? mesh_vertex_metadata[local_vi] : std::string("{}"));
        }
      }

      const auto& triangles = mesh.getTriangles();
      const auto& mesh_face_metadata = mesh.getFaceMetadata();
      for (size_t triangle_vertex_index = 0; triangle_vertex_index < triangles.size(); triangle_vertex_index += 3) {
        GpuSegmentMeshletTriangle triangle;
        triangle.vertex_index0 = static_cast<unsigned int>(triangles[triangle_vertex_index] + vertex_offset);
        triangle.vertex_index1 = static_cast<unsigned int>(triangles[triangle_vertex_index + 1] + vertex_offset);
        triangle.vertex_index2 = static_cast<unsigned int>(triangles[triangle_vertex_index + 2] + vertex_offset);

        const int meshing_neighbor_segment_index =
            meshing_neighbor_indices[meshing_segment_id][triangle_vertex_index / 3];
        if (meshing_neighbor_segment_index >= static_cast<long>(meshing_to_physics_segment_indices.size())) {
          EVOENGINE_ERROR("meshing_neighbor_segment_index out of bounds: " << meshing_neighbor_segment_index
                                                                           << "; upper bound is: "
                                                                           << meshing_to_physics_segment_indices.size())
        } else if (meshing_neighbor_segment_index >= 0) {
          triangle.neighbor_segment_index = meshing_to_physics_segment_indices[meshing_neighbor_segment_index];
        } else {
          triangle.neighbor_segment_index = meshing_neighbor_segment_index;
        }

        for (size_t j = 0; j < 3; j++) {
          // Corner index into PerTriangleCorner normals — never triangles[corner] (vertex id).
          const size_t corner_index = triangle_vertex_index + j;
          const glm::dvec3 local_n = mesh.getNormals()[corner_index];
          glm::vec3 world_n = root_transform.TransformVector(glm::vec3(local_n.x, local_n.y, local_n.z));
          const float n_len = glm::length(world_n);
          if (n_len > 1.0e-16f) {
            world_n /= n_len;
          }
          // normal0 = rest-pose (GPU prediction rotates into normal each frame).
          triangle.normal0[j] = triangle.normal[j] = glm::vec4(world_n, 0.0f);

          if (mesh.hasValidUVIndex(corner_index)) {
            triangle.uv[j] = glm::vec4(ToVec3(mesh.getUV(corner_index)), 0.0);
          } else {
            triangle.uv[j] = glm::vec4(0.0f, 0.0f, 0.0f, 0.0f);
          }
        }

        triangle.segment_pair_index = -1;
        if (triangle.neighbor_segment_index >= 0) {
          for (int pair_handle : dynamic_strands->segment_data_list[physics_segment_id].pair_handles) {
            if (pair_handle == -1) {
              continue;
            }
            if (dynamic_strands->segment_pairs[pair_handle].segment0_handle != physics_segment_id &&
                dynamic_strands->segment_pairs[pair_handle].segment1_handle != physics_segment_id) {
              EVOENGINE_ERROR("Segment pair incorrectly referenced!");
            }
            if (dynamic_strands->segment_pairs[pair_handle].segment0_handle == triangle.neighbor_segment_index ||
                dynamic_strands->segment_pairs[pair_handle].segment1_handle == triangle.neighbor_segment_index) {
              triangle.segment_pair_index = static_cast<int>(pair_handle);
              break;
            }
          }
        }

        segment_meshlet_triangles.push_back(triangle);
        if (keep_metadata) {
          const size_t face_index = triangle_vertex_index / 3;
          segment_meshlet_face_metadata.push_back(
              face_index < mesh_face_metadata.size() ? mesh_face_metadata[face_index] : std::string("{}"));
        }
      }
    }
  }
}

void DsKineticVoronoiMeshing::PrepareAndPopulateGpuMeshletBuffers(
    std::vector<kinDS::VoronoiMesh>& meshes, const std::vector<std::vector<int>>& physics_strand_to_segment_indices,
    const std::vector<std::vector<size_t>>& meshing_strand_to_segment_indices,
    std::vector<std::vector<int>>& meshing_neighbor_indices,
    const std::vector<size_t>& meshing_to_physics_segment_indices, const GlobalTransform& root_transform) {
  PrepareSegmentMeshletsForGpu(meshes, meshing_neighbor_indices, meshing_to_physics_segment_indices, root_transform);
  if (&meshes != &segment_meshlets_) {
    segment_meshlets_ = meshes;
  }
  if (&meshing_neighbor_indices != &meshing_neighbor_indices_) {
    meshing_neighbor_indices_ = meshing_neighbor_indices;
  }
  PopulateGpuMeshletBuffers(meshes, physics_strand_to_segment_indices, meshing_strand_to_segment_indices,
                            meshing_neighbor_indices, meshing_to_physics_segment_indices, root_transform);
}

// DsKineticVoronoiMeshing implementation
DsKineticVoronoiMeshing::RenderSettings DsKineticVoronoiMeshing::render_settings = {};
DsKineticVoronoiMeshing::MeshingSettings DsKineticVoronoiMeshing::meshing_settings = {};

DsKineticVoronoiMeshing::DsKineticVoronoiMeshing() {
}

DsKineticVoronoiMeshing::~DsKineticVoronoiMeshing() {
  tree_mesher_.reset();
}

void eco_sys_lab_package::DsKineticVoronoiMeshing::InitBuffer(
    VkBufferCreateInfo& buffer_create_info, VmaAllocationCreateInfo& buffer_vma_allocation_create_info) {
  device_segment_meshlet_triangles_buffer =
      std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  device_segment_meshlet_vertices_buffer =
      std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  device_segment_signed_volumes_buffer =
      std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  device_segment_initial_volumes_buffer =
      std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  device_volume_result_buffers.clear();
  const auto frames = Platform::GetMaxFramesInFlight();
  device_volume_result_buffers.reserve(frames);
  for (uint32_t i = 0; i < frames; ++i) {
    device_volume_result_buffers.push_back(
        std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info));
  }
}

bool eco_sys_lab_package::BuildKineticMeshingInputs(const DynamicStrandsInitializeParameters& initialize_parameters,
                                                    const StrandModelSkeleton& strand_model_skeleton,
                                                    const StrandModelStrandGroup& strand_model_strand_group,
                                                    DtsStrandGroup& randomly_subdivided_strand_group,
                                                    DtsStrandGroup& uniformly_subdivided_strand_group,
                                                    KineticMeshingInputs& out_inputs) {
  const auto& randomly_subdivided_strands = randomly_subdivided_strand_group.PeekStrands();
  const auto& randomly_subdivided_strand_segments = randomly_subdivided_strand_group.PeekStrandSegments();
  if (randomly_subdivided_strands.empty()) {
    EVOENGINE_LOG("Strand Group is empty!");
    return false;
  }
  strand_model_strand_group.UniformlySubdivide<DtsStrandGroupData, DtsStrandData, DtsStrandSegmentData>(
      uniformly_subdivided_strand_group, initialize_parameters.uniform_subdivision,
      [&](const StrandHandle src_handle, DtsStrandData& strand_data) {
      },
      [&](const float start_root_distance, const float end_root_distance, const StrandSegmentHandle src_handle,
          const uint32_t original_segment_index, const float segment_t, DtsStrandSegmentData& segment_data,
          const uint32_t sub_segment_index) {
        const auto& src_segment_data = strand_model_strand_group.PeekStrandSegmentData(src_handle);
        segment_data.node_handle = src_segment_data.node_handle;
        segment_data.original_segment_handle = src_handle;
        segment_data.original_segment_index = original_segment_index;
        segment_data.segment_index = sub_segment_index;
        segment_data.original_segment_t = segment_t;
        segment_data.start_root_distance = start_root_distance;
        segment_data.end_root_distance = end_root_distance;
        const auto& strand_segment = strand_model_strand_group.PeekStrandSegment(src_handle);
        const auto& strand = strand_model_strand_group.PeekStrand(strand_segment.GetStrandHandle());
        const auto& strand_segment_handles = strand.PeekStrandSegmentHandles();

        glm::vec2 p0, p1, p3;
        const glm::vec2 p2 = src_segment_data.profile_position;
        float d0, d1, d3;
        const float d2 = src_segment_data.initial_distance_to_boundary;
        if (src_handle == strand_segment_handles.front()) {
          d1 = d2;
          d0 = d1 * 2.0f - d2;

          p1 = p2;
          p0 = p1 * 2.0f - p2;
        } else if (strand_segment.GetPrevHandle() == strand_segment_handles.front()) {
          const auto& prev_segment_data =
              strand_model_strand_group.PeekStrandSegmentData(strand_segment.GetPrevHandle());
          d0 = d2;
          d1 = prev_segment_data.initial_distance_to_boundary;

          p0 = p2;
          p1 = prev_segment_data.profile_position;
        } else {
          const auto& prev_segment = strand_model_strand_group.PeekStrandSegment(strand_segment.GetPrevHandle());
          const auto& prev_segment_data =
              strand_model_strand_group.PeekStrandSegmentData(strand_segment.GetPrevHandle());
          const auto& prev_prev_segment_data =
              strand_model_strand_group.PeekStrandSegmentData(prev_segment.GetPrevHandle());
          d0 = prev_prev_segment_data.initial_distance_to_boundary;
          d1 = prev_segment_data.initial_distance_to_boundary;

          p0 = prev_prev_segment_data.profile_position;
          p1 = prev_segment_data.profile_position;
        }
        if (src_handle == strand_segment_handles.back()) {
          d3 = d2 * 2.0f - d1;

          p3 = p2 * 2.0f - p1;
        } else {
          const auto& next_segment_data =
              strand_model_strand_group.PeekStrandSegmentData(strand_segment.GetNextHandle());
          d3 = next_segment_data.initial_distance_to_boundary;

          p3 = next_segment_data.profile_position;
        }
        segment_data.initial_distance_to_boundary = Strands::CubicInterpolation(d0, d1, d2, d3, segment_t);
        segment_data.profile_position = Strands::CubicInterpolation(p0, p1, p2, p3, segment_t);

        const auto calculate_polar_coordinates = [](const glm::vec2& profile_position) {
          const auto r = glm::length(profile_position);
          if (r <= glm::epsilon<float>()) {
            return glm::vec2(0.0f);
          }
          if (profile_position.y >= 0)
            return glm::vec2(r, glm::acos(profile_position.x / r));
          return glm::vec2(r, -glm::acos(profile_position.x / r));
        };

        segment_data.profile_polar_coordinate = calculate_polar_coordinates(segment_data.profile_position);
      },
      (initialize_parameters.min_segment_length + initialize_parameters.max_segment_length) * .5f * .01f);

  std::vector<std::vector<StrandCrossSectionGuidePoint>> strand_guide_points(randomly_subdivided_strands.size());
  std::vector<std::vector<int>> randomly_subdivided_segment_handles(randomly_subdivided_strands.size());
  std::vector<std::vector<double>> random_subdivisions_by_strand(randomly_subdivided_strands.size());

  int maxSegmentCount = std::numeric_limits<int>::min();
  std::mutex m;

  std::function<void(int&, int)> updateMax = [&](int& cur_max, int candidate) {
    std::lock_guard<std::mutex> lock(m);
    cur_max = std::max(cur_max, candidate);
  };

  Jobs::RunParallelFor(randomly_subdivided_strands.size(), [&](const size_t strand_index) {
    auto& random_subdivided_strand = randomly_subdivided_strands[strand_index];
    auto& uniformly_subdivided_strand = uniformly_subdivided_strand_group.PeekStrand(strand_index);

    size_t first_segment_handle = uniformly_subdivided_strand.PeekStrandSegmentHandles()[0];
    const auto& first_uniform_segment_data =
        uniformly_subdivided_strand_group.PeekStrandSegmentData(first_segment_handle);

    StrandCrossSectionGuidePoint first_guide_point;
    first_guide_point.profile_position =
        glm::dvec2(first_uniform_segment_data.profile_position.x, first_uniform_segment_data.profile_position.y);
    first_guide_point.node_handle = first_uniform_segment_data.node_handle;
    first_guide_point.segment_handle = first_segment_handle;
    first_guide_point.root_distance = first_uniform_segment_data.start_root_distance;
    strand_guide_points[strand_index].push_back(first_guide_point);

    updateMax(maxSegmentCount, uniformly_subdivided_strand.PeekStrandSegmentHandles().size());
    for (int uniform_segment_index = 0;
         uniform_segment_index < uniformly_subdivided_strand.PeekStrandSegmentHandles().size();
         uniform_segment_index++) {
      size_t segment_handle = uniformly_subdivided_strand.PeekStrandSegmentHandles()[uniform_segment_index];
      const auto& uniform_segment_data = uniformly_subdivided_strand_group.PeekStrandSegmentData(segment_handle);

      StrandCrossSectionGuidePoint guide_point;
      guide_point.profile_position =
          glm::dvec2(uniform_segment_data.profile_position.x, uniform_segment_data.profile_position.y);
      guide_point.node_handle = uniform_segment_data.node_handle;
      guide_point.segment_handle = segment_handle;
      guide_point.root_distance = uniform_segment_data.end_root_distance;
      strand_guide_points[strand_index].push_back(guide_point);
    }

    for (int random_segment_index = 0;
         random_segment_index < random_subdivided_strand.PeekStrandSegmentHandles().size(); random_segment_index++) {
      size_t segment_handle = random_subdivided_strand.PeekStrandSegmentHandles()[random_segment_index];
      const auto& segment = randomly_subdivided_strand_segments[segment_handle];
      const auto& random_segment_data = randomly_subdivided_strand_group.PeekStrandSegmentData(
          random_subdivided_strand.PeekStrandSegmentHandles()[random_segment_index]);

      randomly_subdivided_segment_handles[strand_index].push_back(static_cast<int>(segment_handle));

      if (!std::isnan(segment.end_t)) {
        random_subdivisions_by_strand[strand_index].push_back(
            initialize_parameters.uniform_subdivision * (segment.end_t + random_segment_data.original_segment_index));
      }
    }
  });

  // Create a branch index lookup using [strand_id][h]
  std::vector<std::vector<size_t>> branch_indices(strand_guide_points.size());
  // Maintain the branches as [h][branch_id][strand_no]
  std::vector<std::vector<std::vector<size_t>>> strands_by_branch_id(maxSegmentCount + 1);

  std::map<SkeletonNodeHandle, size_t> node_to_branch_map;
  for (size_t strand_id = 0; strand_id < strand_guide_points.size(); strand_id++) {
    auto& guide_points = strand_guide_points[strand_id];
    auto node_handle = guide_points.front().node_handle;
    auto it = node_to_branch_map.find(node_handle);
    if (it != node_to_branch_map.end()) {
      size_t branch_index = it->second;
      branch_indices[strand_id].push_back(branch_index);
      strands_by_branch_id[0][branch_index].push_back(strand_id);
    } else {
      size_t branch_index = strands_by_branch_id[0].size();
      node_to_branch_map[node_handle] = branch_index;
      strands_by_branch_id[0].push_back({strand_id});
      branch_indices[strand_id].push_back(branch_index);
    }
  }

  for (size_t h = 1; h < maxSegmentCount + 1; h++) {
    // Now iterate over each branch and check if we need to split it
    strands_by_branch_id[h].resize(strands_by_branch_id[h - 1].size());

    // EVOENGINE_LOG("------------------ Height: " << h)

    for (size_t branch_id = 0; branch_id < strands_by_branch_id[h - 1].size(); branch_id++) {
      auto& branch_strands = strands_by_branch_id[h - 1][branch_id];
      // branch might end early:
      if (branch_strands.empty()) {
        // EVOENGINE_LOG("Branch with id " << branch_id << " ended at height " << h);
        continue;
      }

      SkeletonNodeHandle branch_node = strand_guide_points[branch_strands.front()][h].node_handle;
      std::map<SkeletonNodeHandle, size_t> node_to_branch_map;

      node_to_branch_map[branch_node] = branch_id;

      for (size_t& strand_id : branch_strands) {
        const auto& guide_points = strand_guide_points[strand_id];

        if (h >= guide_points.size()) {
          // EVOENGINE_LOG("Strand " << strand_id << " ended early at height " << h);
          continue;  // strand ends here
        }

        auto node_handle = guide_points[h].node_handle;

        auto it = node_to_branch_map.find(node_handle);
        if (it != node_to_branch_map.end()) {
          size_t branch_index = it->second;
          // EVOENGINE_LOG("strand " << strand_id << " belongs to already found branch " << branch_index);
          branch_indices[strand_id].push_back(branch_index);
          strands_by_branch_id[h][branch_index].push_back(strand_id);
        } else {
          size_t branch_index = strands_by_branch_id[h].size();
          // EVOENGINE_LOG("strand " << strand_id << " belongs to newly discovered branch " << branch_index);
          node_to_branch_map[node_handle] = branch_index;
          strands_by_branch_id[h].push_back({strand_id});
          branch_indices[strand_id].push_back(branch_index);
        }
      }
    }
  }

  // For debugging, output the node index for each height:
  /*for (size_t h = 0; h < maxSegmentCount + 1; h++) {
    std::cout << "Node handles at height " << h << ": ";

    // collect in a set to not list duplicates
    std::set<SkeletonNodeHandle> handles;
    for (size_t strand_id = 0; strand_id < strand_guide_points.size(); strand_id++) {
      auto& guide_points = strand_guide_points[strand_id];
      if (h < guide_points.size()) {
        handles.insert(guide_points[h].node_handle);
      }
    }

    for (auto& handle : handles) {
      std::cout << handle << ", ";
    }
    std::cout << std::endl;
  }

  for (size_t h = 0; h < maxSegmentCount + 1; h++) {
    std::cout << "Branch indices at height " << h << ": ";
    std::set<size_t> index_set;
    for (size_t strand_id = 0; strand_id < strand_guide_points.size(); strand_id++) {
      if (h < branch_indices[strand_id].size()) {
        index_set.insert(branch_indices[strand_id][h]);
      }
    }

    for (auto& index : index_set) {
      std::cout << index << ", ";
    }
    std::cout << std::endl;
  }*/

  std::vector<std::vector<glm::dmat4>> transforms_by_height_and_branch(maxSegmentCount + 1);
  Jobs::RunParallelFor(maxSegmentCount + 1, [&](const size_t h) {
    transforms_by_height_and_branch[h].resize(strands_by_branch_id[h].size());
    for (size_t branch_index = 0; branch_index < transforms_by_height_and_branch[h].size(); branch_index++) {
      const auto& strand_ids = strands_by_branch_id[h][branch_index];
      if (strand_ids.empty()) {
        continue;
      }

      transforms_by_height_and_branch[h][branch_index] = BuildInterpolatedInternodeTransformAtHeight(
          strand_model_skeleton, strand_guide_points[strand_ids.front()], h);
    }
  });

  // Replace parametric 2D profile samples with 3D cubic-strand ∩ profile-plane samples.
  Jobs::RunParallelFor(strand_guide_points.size(), [&](const size_t strand_index) {
    auto& guide_points = strand_guide_points[strand_index];
    if (guide_points.empty() || strand_index >= branch_indices.size()) {
      return;
    }

    int hint_segment_index = 0;
    double preferred_t = 0.0;
    for (size_t h = 0; h < guide_points.size(); ++h) {
      if (h >= branch_indices[strand_index].size()) {
        break;
      }

      const size_t branch_index = branch_indices[strand_index][h];
      if (h >= transforms_by_height_and_branch.size() || branch_index >= transforms_by_height_and_branch[h].size()) {
        break;
      }

      if (guide_points[h].segment_handle >= 0) {
        hint_segment_index =
            static_cast<int>(uniformly_subdivided_strand_group.PeekStrandSegmentData(guide_points[h].segment_handle)
                                 .original_segment_index);
        if (h == 0) {
          preferred_t = 0.0;
        } else {
          preferred_t = uniformly_subdivided_strand_group.PeekStrandSegmentData(guide_points[h].segment_handle)
                            .original_segment_t;
        }
      }

      const glm::dmat4& transform = transforms_by_height_and_branch[h][branch_index];
      const glm::dvec2 fallback = guide_points[h].profile_position;
      guide_points[h].profile_position = SampleStrandProfileAtPlane(
          strand_model_strand_group, static_cast<StrandHandle>(strand_index), transform, hint_segment_index, fallback,
          preferred_t, DsKineticVoronoiMeshing::meshing_settings.spline_tension);
    }
  });

#ifndef NDEBUG
  {
    size_t residual_failures = 0;
    double max_residual = 0.0;
    for (size_t strand_index = 0; strand_index < strand_guide_points.size(); ++strand_index) {
      const auto& guide_points = strand_guide_points[strand_index];
      for (size_t h = 0; h < guide_points.size(); ++h) {
        if (h >= branch_indices[strand_index].size()) {
          break;
        }
        const size_t branch_index = branch_indices[strand_index][h];
        if (h >= transforms_by_height_and_branch.size() || branch_index >= transforms_by_height_and_branch[h].size()) {
          break;
        }
        const glm::dmat4& transform = transforms_by_height_and_branch[h][branch_index];
        const ProfilePlane plane = ExtractProfilePlane(transform);
        const glm::dvec2& profile = guide_points[h].profile_position;
        const glm::dvec3 reconstructed = glm::dvec3(transform * glm::dvec4(profile.x, 0.0, profile.y, 1.0));
        const double residual = std::abs(PlaneResidual(reconstructed, plane));
        max_residual = std::max(max_residual, residual);
        if (residual > kPlaneSplineResidualEps) {
          ++residual_failures;
        }
      }
    }
    if (residual_failures > 0) {
      EVOENGINE_WARNING("Plane-spline profile sampling: "
                        << residual_failures << " samples exceed residual tolerance; max residual = " << max_residual);
    }
  }
#endif

  std::vector<std::vector<glm::dvec2>> strand_splines;
  strand_splines.reserve(strand_guide_points.size());
  for (const auto& guide_points : strand_guide_points) {
    std::vector<glm::dvec2> support_points;
    support_points.reserve(guide_points.size());
    for (const auto& guide_point : guide_points) {
      support_points.emplace_back(guide_point.profile_position);
    }
    strand_splines.push_back(std::move(support_points));
  }

  out_inputs.support_points = std::move(strand_splines);
  out_inputs.subdivisions_by_strand = std::move(random_subdivisions_by_strand);
  out_inputs.physics_strand_to_segment_indices = std::move(randomly_subdivided_segment_handles);
  out_inputs.transforms_by_height_and_branch = std::move(transforms_by_height_and_branch);
  out_inputs.root_transform = initialize_parameters.root_transform;
  out_inputs.branch_indices = std::move(branch_indices);
  out_inputs.strands_by_branch_id = std::move(strands_by_branch_id);
  out_inputs.min_segment_length = initialize_parameters.min_segment_length;
  out_inputs.max_segment_length = initialize_parameters.max_segment_length;
  return true;
}

void eco_sys_lab_package::DsKineticVoronoiMeshing::InitData(
    const DynamicStrandsInitializeParameters& initialize_parameters, const StrandModelSkeleton& strand_model_skeleton,
    const StrandModelStrandGroup& strand_model_strand_group, DtsStrandGroup& randomly_subdivided_strand_group,
    DtsStrandGroup& uniformly_subdivided_strand_group) {
  initialize_parameters_ = initialize_parameters;
  KineticMeshingInputs inputs;
  if (!BuildKineticMeshingInputs(initialize_parameters, strand_model_skeleton, strand_model_strand_group,
                                 randomly_subdivided_strand_group, uniformly_subdivided_strand_group, inputs)) {
    return;
  }
  kinDS::logger.setLogLevel(kinDS::LogLevel::Debug, false);
  last_cache_result_ = static_cast<int>(RunKineticMeshingCache(*this, inputs));
}

KineticMeshCacheResult eco_sys_lab_package::RunKineticMeshingCache(DsKineticVoronoiMeshing& meshing,
                                                                   KineticMeshingInputs& inputs) {
  const bool skip_load = DsKineticVoronoiMeshing::meshing_settings.cache_hit_skip_load;
  const bool override_buffer = DsKineticVoronoiMeshing::meshing_settings.override_meshing_buffer;

  MeshingInputHashStats hash_stats =
      ComputeMeshingInputHashStats(inputs.support_points, inputs.subdivisions_by_strand,
                                   inputs.physics_strand_to_segment_indices, inputs.transforms_by_height_and_branch,
                                   inputs.root_transform, inputs.branch_indices, inputs.strands_by_branch_id);
  hash_stats.min_segment_length = inputs.min_segment_length;
  hash_stats.max_segment_length = inputs.max_segment_length;
  const std::filesystem::path bin_path = MeshingBufferDirectory() / (hash_stats.input_hash + ".bin");
  const bool bin_exists = std::filesystem::exists(bin_path);
  const bool would_hit = !override_buffer && bin_exists;

  if (would_hit && skip_load) {
    LogMeshingInputHashStats(hash_stats);
    EVOENGINE_LOG("Meshing buffer cache hit (precompute skip load) " << hash_stats.input_hash << " at "
                                                                     << bin_path.string());
    meshing.last_meshing_succeeded_ = true;
    return KineticMeshCacheResult::CacheHit;
  }

  const bool ok = meshing.RunMeshingAlgorithm(
      inputs.support_points, inputs.subdivisions_by_strand, inputs.physics_strand_to_segment_indices,
      inputs.transforms_by_height_and_branch, inputs.root_transform, inputs.branch_indices, inputs.strands_by_branch_id,
      inputs.min_segment_length, inputs.max_segment_length);
  if (!ok) {
    return KineticMeshCacheResult::Failed;
  }
  if (would_hit) {
    return KineticMeshCacheResult::CacheHit;
  }
  return KineticMeshCacheResult::Saved;
}

void eco_sys_lab_package::DsKineticVoronoiMeshing::InitializationGraphicsPipeline(
    const DynamicStrandsInitializeParameters& initialize_parameters) {
  // Don't need this for now
}

struct VertexPredictionPushConstant {
  uint32_t vertex_count = 0;
  int padding0;
  int padding1;
  int padding2;
};

struct TrianglePredictionPushConstant {
  uint32_t triangle_count = 0;
  int padding0;
  int padding1;
  int padding2;
};

void eco_sys_lab_package::DsKineticVoronoiMeshing::BuildRenderComputePipelines() {
  static std::shared_ptr<Shader> shader{};
  shader = std::make_shared<Shader>();
  shader->TryCompile(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                     std::filesystem::path("./EcoSysLabResources") /
                         "Shaders/Compute/DynamicStrands/Prediction/KineticVoronoiMeshing/Vertex.slang");

  branches_vertex_update_pipeline = std::make_shared<ComputePipeline>();
  branches_vertex_update_pipeline->compute_shader = shader;
  branches_vertex_update_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
  branches_vertex_update_pipeline->descriptor_set_layouts.emplace_back(geometry_descriptor_set_layout);

  auto& push_constant_range = branches_vertex_update_pipeline->push_constant_ranges.emplace_back();
  push_constant_range.size = sizeof(VertexPredictionPushConstant);
  push_constant_range.offset = 0;
  push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

  branches_vertex_update_pipeline->Initialize();

  // Triangles
  branches_triangle_update_pipeline = std::make_shared<ComputePipeline>();
  branches_triangle_update_pipeline->compute_shader =
      Shader::CreateTemporary(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                              std::filesystem::path("./EcoSysLabResources") /
                                  "Shaders/Compute/DynamicStrands/Prediction/KineticVoronoiMeshing/Triangle.slang");
  branches_triangle_update_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
  branches_triangle_update_pipeline->descriptor_set_layouts.emplace_back(geometry_descriptor_set_layout);

  auto& triangle_prediction_push_constant_range =
      branches_triangle_update_pipeline->push_constant_ranges.emplace_back();
  triangle_prediction_push_constant_range.size = sizeof(TrianglePredictionPushConstant);
  triangle_prediction_push_constant_range.offset = 0;
  triangle_prediction_push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

  branches_triangle_update_pipeline->Initialize();

  volume_measure_pipeline = std::make_shared<ComputePipeline>();
  volume_measure_pipeline->compute_shader = Shader::CreateTemporary(
      ShaderType::Compute, Platform::GetShaderGlobalDefines(),
      std::filesystem::path("./EcoSysLabResources") /
          "Shaders/Compute/DynamicStrands/VolumeMeasure/KineticVoronoiMeshing/VolumeMeasure.slang");
  volume_measure_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
  volume_measure_pipeline->descriptor_set_layouts.emplace_back(geometry_descriptor_set_layout);
  auto& volume_measure_push = volume_measure_pipeline->push_constant_ranges.emplace_back();
  volume_measure_push.size = sizeof(uint32_t) * 4;
  volume_measure_push.offset = 0;
  volume_measure_push.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
  volume_measure_pipeline->Initialize();

  volume_finalize_pipeline = std::make_shared<ComputePipeline>();
  volume_finalize_pipeline->compute_shader = Shader::CreateTemporary(
      ShaderType::Compute, Platform::GetShaderGlobalDefines(),
      std::filesystem::path("./EcoSysLabResources") /
          "Shaders/Compute/DynamicStrands/VolumeMeasure/KineticVoronoiMeshing/VolumeFinalize.slang");
  volume_finalize_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
  volume_finalize_pipeline->descriptor_set_layouts.emplace_back(geometry_descriptor_set_layout);
  auto& volume_finalize_push = volume_finalize_pipeline->push_constant_ranges.emplace_back();
  volume_finalize_push.size = sizeof(uint32_t) * 4;
  volume_finalize_push.offset = 0;
  volume_finalize_push.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
  volume_finalize_pipeline->Initialize();
}

void eco_sys_lab_package::DsKineticVoronoiMeshing::RenderCompute(const bool physics_simulation_active) const {
  if (dynamic_strands->segments.empty())
    return;
  const uint32_t work_group_invocations = Platform::GetInstance().GetCapabilities().compute_work_group_invocations;
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  const bool volume_heatmap_active = render_settings.segment_meshlet_render_parameters.color_mode ==
                                     SegmentMeshletsRenderParameters::VolumeChangeHeatmap;
  const bool run_volume_measure =
      physics_simulation_active && (render_settings.enable_volume_measure || volume_heatmap_active);

  // Kinetic skinned meshlet volume is nearly rigid on the GPU; meaningful change is measured on CPU
  // after download + bark smooth (throttled). Do this before recording this frame's skinning so Download
  // sees the previous frame's completed vertex update.
  if (run_volume_measure) {
    MaybeMeasureSmoothedVolumeCpu();
  }

  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    // Vertices
    VertexPredictionPushConstant vertex_push_constant;
    vertex_push_constant.vertex_count = segment_meshlet_vertices.size();
    branches_vertex_update_pipeline->Bind(vk_command_buffer);
    branches_vertex_update_pipeline->BindDescriptorSet(
        vk_command_buffer, 0, dynamic_strands->strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
    branches_vertex_update_pipeline->BindDescriptorSet(
        vk_command_buffer, 1, geometry_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
    branches_vertex_update_pipeline->PushConstant(vk_command_buffer, 0, vertex_push_constant);
    branches_vertex_update_pipeline->Dispatch(
        vk_command_buffer, Platform::DivUp(vertex_push_constant.vertex_count, work_group_invocations), 1, 1);
    Platform::EverythingBarrier(vk_command_buffer);

    // Triangles
    TrianglePredictionPushConstant triangle_push_constant;
    triangle_push_constant.triangle_count = segment_meshlet_triangles.size();
    branches_triangle_update_pipeline->Bind(vk_command_buffer);
    branches_triangle_update_pipeline->BindDescriptorSet(
        vk_command_buffer, 0, dynamic_strands->strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
    branches_triangle_update_pipeline->BindDescriptorSet(
        vk_command_buffer, 1, geometry_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
    branches_triangle_update_pipeline->PushConstant(vk_command_buffer, 0, triangle_push_constant);
    branches_triangle_update_pipeline->Dispatch(
        vk_command_buffer, Platform::DivUp(triangle_push_constant.triangle_count, work_group_invocations), 1, 1);
    Platform::EverythingBarrier(vk_command_buffer);
  });
}

void eco_sys_lab_package::DsKineticVoronoiMeshing::BuildRenderingPipelines() {
  BuildSegmentMeshletsRenderingPipelines();
}

void eco_sys_lab_package::DsKineticVoronoiMeshing::Download() {
  if (!segment_meshlet_vertices.empty()) {
    device_segment_meshlet_vertices_buffer->DownloadVector(segment_meshlet_vertices, segment_meshlet_vertices.size());
  }
  if (!segment_meshlet_triangles.empty()) {
    device_segment_meshlet_triangles_buffer->DownloadVector(segment_meshlet_triangles,
                                                            segment_meshlet_triangles.size());
  }
  // Metadata is CPU-only (never on GPU). The parallel caches filled at populate stay valid across download
  // as long as topology is unchanged; export/rebuild reattaches them to the downloaded geometry.
}

void eco_sys_lab_package::DsKineticVoronoiMeshing::Upload() {
  device_segment_meshlet_vertices_buffer->UploadVector(segment_meshlet_vertices);
  device_segment_meshlet_vertices_buffer->SetDebugName("Segment Meshlet Vertices Buffer");
  device_segment_meshlet_triangles_buffer->UploadVector(segment_meshlet_triangles);
  device_segment_meshlet_triangles_buffer->SetDebugName("Segment Meshlet Triangles Buffer");

  if (dynamic_strands) {
    const size_t segment_count = std::max<size_t>(1, dynamic_strands->segments.size());
    std::vector<float> initials(segment_count, 0.0f);
    if (has_initial_meshlet_volumes_) {
      for (const auto& [segment_index, volume] : initial_meshlet_volumes_by_segment_) {
        if (segment_index < initials.size()) {
          initials[segment_index] = static_cast<float>(volume);
        }
      }
    }
    if (device_segment_signed_volumes_buffer) {
      // Mirror baseline into the "current" buffer until a measure pass overwrites it.
      device_segment_signed_volumes_buffer->UploadVector(initials);
      device_segment_signed_volumes_buffer->SetDebugName("Kinetic Segment Signed Volumes");
    }
    if (device_segment_initial_volumes_buffer) {
      device_segment_initial_volumes_buffer->UploadVector(initials);
      device_segment_initial_volumes_buffer->SetDebugName("Kinetic Segment Initial Volumes");
    }
  }
}

void eco_sys_lab_package::DsKineticVoronoiMeshing::Clear() {
  segment_meshlet_vertices.clear();
  segment_meshlet_triangles.clear();
  segment_meshlet_vertex_metadata.clear();
  segment_meshlet_face_metadata.clear();
  segment_meshlets_.clear();
  meshing_neighbor_indices_.clear();
  bark_debug_mesh_ = kinDS::VoronoiMesh{};
  has_bark_debug_mesh_ = false;
  boundary_distances_by_vertex.clear();
  strand_tree.reset();
  tree_mesher_.reset();
  meshlets_root_transform_ = {};
  initial_meshlet_volumes_by_segment_.clear();
  initial_meshlet_cumulative_volume_ = 0.0;
  has_initial_meshlet_volumes_ = false;
  volume_measure_csv_.Close();
  volume_measure_frame_counter_ = 0;
}

void DsKineticVoronoiMeshing::CaptureInitialMeshletVolumes() {
  initial_meshlet_volumes_by_segment_.clear();
  initial_meshlet_cumulative_volume_ = 0.0;
  has_initial_meshlet_volumes_ = false;
  if (segment_meshlet_vertices.empty() || segment_meshlet_triangles.empty()) {
    return;
  }
  // Rest pose (`x0`) at the end of meshing / cache load — before intersection / physics deformation.
  const auto volumes =
      DsKineticVoronoiVolumeUtils::ComputeAllMeshletVolumes(segment_meshlet_vertices, segment_meshlet_triangles, false);
  initial_meshlet_volumes_by_segment_.reserve(volumes.meshlets.size());
  for (const auto& entry : volumes.meshlets) {
    initial_meshlet_volumes_by_segment_[entry.segment_index] = entry.volume;
  }
  initial_meshlet_cumulative_volume_ = volumes.cumulative_volume;
  has_initial_meshlet_volumes_ = true;
  EVOENGINE_LOG("Captured initial Kinetic meshlet volumes: " << volumes.meshlets.size() << " meshlets, cumulative="
                                                             << initial_meshlet_cumulative_volume_);

  if (device_segment_signed_volumes_buffer && dynamic_strands) {
    // Keep current == initial at capture so heatmap is white before the first measure pass.
    std::vector<float> initials(std::max<size_t>(1, dynamic_strands->segments.size()), 0.0f);
    for (const auto& [segment_index, volume] : initial_meshlet_volumes_by_segment_) {
      if (segment_index < initials.size()) {
        initials[segment_index] = static_cast<float>(volume);
      }
    }
    device_segment_signed_volumes_buffer->UploadVector(initials);
    device_segment_signed_volumes_buffer->SetDebugName("Kinetic Segment Signed Volumes");
  }
  if (device_segment_initial_volumes_buffer && dynamic_strands) {
    std::vector<float> initials(std::max<size_t>(1, dynamic_strands->segments.size()), 0.0f);
    for (const auto& [segment_index, volume] : initial_meshlet_volumes_by_segment_) {
      if (segment_index < initials.size()) {
        initials[segment_index] = static_cast<float>(volume);
      }
    }
    device_segment_initial_volumes_buffer->UploadVector(initials);
    device_segment_initial_volumes_buffer->SetDebugName("Kinetic Segment Initial Volumes");
  }
}

bool DsKineticVoronoiMeshing::BuildSmoothedCurrentMeshletVertices(std::vector<GpuSegmentMeshletVertex>& out_vertices) {
  out_vertices.clear();
  if (segment_meshlet_vertices.empty() || segment_meshlet_triangles.empty()) {
    return false;
  }
  if (!EnsureCpuMeshletsFromGpu() || segment_meshlets_.empty() || !strand_tree || !tree_mesher_) {
    return false;
  }

  const auto& physics_strand_to_segment_indices = strand_tree->getPhysicsStrandToSegmentIndices();
  const auto& meshing_strand_to_segment_indices = tree_mesher_->getMeshingStrandToSegmentIndices();
  if (physics_strand_to_segment_indices.size() != meshing_strand_to_segment_indices.size()) {
    EVOENGINE_ERROR("BuildSmoothedCurrentMeshletVertices: physics/meshing strand map size mismatch.");
    return false;
  }

  // Working copy so rest-pose @ref segment_meshlets_ topology is not permanently deformed.
  std::vector<kinDS::VoronoiMesh> working = segment_meshlets_;
  GlobalTransform inv_root;
  inv_root.value = glm::inverse(meshlets_root_transform_.value);

  size_t gpu_vi = 0;
  for (size_t strand_id = 0; strand_id < physics_strand_to_segment_indices.size(); ++strand_id) {
    if (meshing_strand_to_segment_indices[strand_id].size() != physics_strand_to_segment_indices[strand_id].size()) {
      EVOENGINE_ERROR("BuildSmoothedCurrentMeshletVertices: strand " << strand_id << " segment count mismatch.");
      return false;
    }
    for (size_t segment_no = 0; segment_no < meshing_strand_to_segment_indices[strand_id].size(); ++segment_no) {
      const size_t meshing_segment_id = meshing_strand_to_segment_indices[strand_id][segment_no];
      if (meshing_segment_id >= working.size()) {
        EVOENGINE_ERROR("BuildSmoothedCurrentMeshletVertices: meshing segment id out of range.");
        return false;
      }
      auto& mesh_vertices = working[meshing_segment_id].getVertices();
      for (size_t local_vi = 0; local_vi < mesh_vertices.size(); ++local_vi) {
        if (gpu_vi >= segment_meshlet_vertices.size()) {
          EVOENGINE_ERROR("BuildSmoothedCurrentMeshletVertices: GPU vertex buffer shorter than CPU meshlets.");
          return false;
        }
        const glm::vec3 world = segment_meshlet_vertices[gpu_vi].x;
        const glm::vec3 local = inv_root.TransformPoint(world);
        mesh_vertices[local_vi] = glm::dvec3(local.x, local.y, local.z);
        ++gpu_vi;
      }
    }
  }
  if (gpu_vi != segment_meshlet_vertices.size()) {
    EVOENGINE_ERROR("BuildSmoothedCurrentMeshletVertices: GPU vertex count ("
                    << segment_meshlet_vertices.size() << ") != CPU meshlet vertices (" << gpu_vi << ").");
    return false;
  }

  const std::vector<uint8_t> meshlet_has_bark = BuildMeshletHasBarkFlags(meshing_neighbor_indices_);
  SmoothBarkMeshPositions(working, meshing_neighbor_indices_, meshlet_has_bark, meshing_settings.bark_smooth_iterations,
                          meshing_settings.bark_smooth_strength, meshing_settings.bark_smooth_lock_boundary,
                          meshing_settings.bark_smooth_uvs);

  out_vertices = segment_meshlet_vertices;
  gpu_vi = 0;
  for (size_t strand_id = 0; strand_id < physics_strand_to_segment_indices.size(); ++strand_id) {
    for (size_t segment_no = 0; segment_no < meshing_strand_to_segment_indices[strand_id].size(); ++segment_no) {
      const size_t meshing_segment_id = meshing_strand_to_segment_indices[strand_id][segment_no];
      const auto& mesh_vertices = working[meshing_segment_id].getVertices();
      for (size_t local_vi = 0; local_vi < mesh_vertices.size(); ++local_vi) {
        const glm::dvec3& local = mesh_vertices[local_vi];
        out_vertices[gpu_vi].x = meshlets_root_transform_.TransformPoint(glm::vec3(local.x, local.y, local.z));
        ++gpu_vi;
      }
    }
  }
  return true;
}

void DsKineticVoronoiMeshing::MaybeMeasureSmoothedVolumeCpu() const {
  const int interval = glm::max(1, render_settings.volume_measure_interval_frames);
  const uint32_t frame_index = volume_measure_frame_counter_;
  ++volume_measure_frame_counter_;
  if ((frame_index % static_cast<uint32_t>(interval)) != 0) {
    return;
  }

  auto* self = const_cast<DsKineticVoronoiMeshing*>(this);
  self->Download();

  std::vector<GpuSegmentMeshletVertex> smoothed_vertices;
  if (!self->BuildSmoothedCurrentMeshletVertices(smoothed_vertices)) {
    EVOENGINE_WARNING("Kinetic volume measure: download+smooth failed; skipping frame " << frame_index);
    return;
  }

  const auto volumes =
      DsKineticVoronoiVolumeUtils::ComputeAllMeshletVolumes(smoothed_vertices, segment_meshlet_triangles, true);

  if (render_settings.enable_volume_measure) {
    if (!volume_measure_csv_.IsOpen()) {
      volume_measure_csv_.Open(MakeTimestampedVolumeMeasureCsvPath("kinetic_volume_measure"), "kinetic");
    }
    volume_measure_csv_.Append(frame_index, volumes.cumulative_volume, initial_meshlet_cumulative_volume_);
  }

  if (device_segment_signed_volumes_buffer && dynamic_strands) {
    std::vector<float> currents(std::max<size_t>(1, dynamic_strands->segments.size()), 0.0f);
    for (const auto& entry : volumes.meshlets) {
      if (entry.segment_index < currents.size()) {
        currents[entry.segment_index] = static_cast<float>(entry.volume);
      }
    }
    device_segment_signed_volumes_buffer->UploadVector(currents);
    device_segment_signed_volumes_buffer->SetDebugName("Kinetic Segment Signed Volumes");
  }
}

void eco_sys_lab_package::DsKineticVoronoiMeshing::UpdateBindings() const {
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  // Legacy strands bindings 8–9 kept valid; Slang meshlet shaders use geometry set 1.
  dynamic_strands->strands_descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(
      8, device_segment_meshlet_vertices_buffer, 0);
  dynamic_strands->strands_descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(
      9, device_segment_meshlet_triangles_buffer, 0);
  if (!geometry_descriptor_sets.empty()) {
    geometry_descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(
        0, device_segment_meshlet_vertices_buffer, 0);
    geometry_descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(
        1, device_segment_meshlet_triangles_buffer, 0);
  }
  if (device_segment_signed_volumes_buffer) {
    dynamic_strands->strands_descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(
        14, device_segment_signed_volumes_buffer, 0);
  }
  if (!device_volume_result_buffers.empty()) {
    const auto& result_buffer = device_volume_result_buffers[current_frame_index % device_volume_result_buffers.size()];
    dynamic_strands->strands_descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(15, result_buffer, 0);
  }
  if (device_segment_initial_volumes_buffer) {
    dynamic_strands->strands_descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(
        18, device_segment_initial_volumes_buffer, 0);
  }
}

void DsKineticVoronoiMeshing::DispatchVolumeMeasure(const VkCommandBuffer vk_command_buffer) const {
  if (!volume_measure_pipeline || !volume_measure_pipeline->Initialized() || !volume_finalize_pipeline ||
      !volume_finalize_pipeline->Initialized() || segment_meshlet_triangles.empty() || !dynamic_strands) {
    return;
  }
  const uint32_t work_group_invocations = Platform::GetInstance().GetCapabilities().compute_work_group_invocations;
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  const uint32_t segment_size = static_cast<uint32_t>(std::max<size_t>(1, dynamic_strands->segments.size()));

  struct MeasurePush {
    uint32_t triangle_size = 0;
    uint32_t segment_size = 0;
    uint32_t frame_index = 0;
    int padding0 = 0;
  } measure_push{};
  measure_push.triangle_size = static_cast<uint32_t>(segment_meshlet_triangles.size());
  measure_push.segment_size = segment_size;
  measure_push.frame_index = volume_measure_frame_counter_;

  volume_measure_pipeline->Bind(vk_command_buffer);
  volume_measure_pipeline->BindDescriptorSet(
      vk_command_buffer, 0, dynamic_strands->strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
  volume_measure_pipeline->BindDescriptorSet(vk_command_buffer, 1,
                                             geometry_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
  volume_measure_pipeline->PushConstant(vk_command_buffer, 0, measure_push);
  vkCmdDispatch(vk_command_buffer, Platform::DivUp(measure_push.triangle_size, work_group_invocations), 1, 1);
  Platform::EverythingBarrier(vk_command_buffer);

  struct FinalizePush {
    uint32_t segment_size = 0;
    uint32_t frame_index = 0;
    int padding0 = 0;
    int padding1 = 0;
  } finalize_push{};
  finalize_push.segment_size = segment_size;
  finalize_push.frame_index = volume_measure_frame_counter_;

  volume_finalize_pipeline->Bind(vk_command_buffer);
  volume_finalize_pipeline->BindDescriptorSet(
      vk_command_buffer, 0, dynamic_strands->strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
  volume_finalize_pipeline->BindDescriptorSet(vk_command_buffer, 1,
                                              geometry_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
  volume_finalize_pipeline->PushConstant(vk_command_buffer, 0, finalize_push);
  vkCmdDispatch(vk_command_buffer, Platform::DivUp(finalize_push.segment_size, work_group_invocations), 1, 1);
  Platform::EverythingBarrier(vk_command_buffer);
}

void DsKineticVoronoiMeshing::ReadbackVolumeMeasureToCsv() const {
  if (device_volume_result_buffers.empty()) {
    return;
  }
  if (!volume_measure_csv_.IsOpen()) {
    volume_measure_csv_.Open(MakeTimestampedVolumeMeasureCsvPath("kinetic_volume_measure"), "kinetic");
  }
  const auto max_frames = static_cast<uint32_t>(device_volume_result_buffers.size());
  if (volume_measure_frame_counter_ < max_frames) {
    ++volume_measure_frame_counter_;
    return;
  }
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  const auto read_index = (current_frame_index + 1) % max_frames;
  GpuVolumeMeasureResult result{};
  device_volume_result_buffers[read_index]->Download(result);
  volume_measure_csv_.Append(result.frame_index, static_cast<double>(result.cumulative_volume),
                             initial_meshlet_cumulative_volume_ > 0.0
                                 ? initial_meshlet_cumulative_volume_
                                 : static_cast<double>(result.initial_cumulative_volume));
  ++volume_measure_frame_counter_;
}

void eco_sys_lab_package::DsKineticVoronoiMeshing::InspectSharedMeshingSettings(
    const std::shared_ptr<EditorLayer>& editor_layer) {
  (void)editor_layer;
  ImGui::Checkbox("Dry run (strand tree only)", &meshing_settings.dry_run_strand_tree_only);
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip("Prepare the strand tree during initialization but skip the meshing algorithm.");
  }
  ImGui::Checkbox("Override buffer", &meshing_settings.override_meshing_buffer);
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip(
        "When enabled, skip loading a cached meshing buffer and overwrite it with newly computed mesh data.");
  }
  ImGui::Checkbox("Recompute segment pairs from mesh", &meshing_settings.recompute_segment_pairs);
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip(
        "Rebuild the physics segment-pair graph from meshlet adjacency. pair_handles[0]/[1] are reserved for "
        "same-strand below/above neighbors (-1 if missing); other neighbors start at index 2. "
        "Also enables post-intersection splitting of INTERSECT meshlets that have multiple connected components "
        "(length-fitted; radius unchanged).");
  }
  ImGui::Checkbox("Debug SVG", &meshing_settings.debug_svg);
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip("Export kinDS segment-builder debug SVGs during meshing.");
  }
  ImGui::Checkbox("Collect meshing statistics", &meshing_settings.collect_meshing_statistics);
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip(
        "Enable kinDS runtime/event statistics CSV and a companion event-list CSV after meshing "
        "(written under the project Metadata/ folder; filenames include a timestamp so previous runs are kept). "
        "The event list has one row per "
        "kinetic event including whether each radius event used a boundary-transition shift. "
        "Also writes a per-mesh intersection CSV for Intersect and for Intersect and export all on an "
        "Intersection Meshes group.");
  }
  if (meshing_settings.collect_meshing_statistics) {
    ImGui::Checkbox("Flush statistics each section", &meshing_settings.flush_meshing_statistics_each_section);
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip(
          "Append a row to an incremental *_partial_*.csv whenever a kinetic section finishes, "
          "so completed-section stats survive a mid-run crash or failure.");
    }
  }
  ImGui::Checkbox("Debug export meshes", &meshing_settings.debug_export_meshes);
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip("After meshing, export per-segment meshlets and a combined OBJ for debugging.");
  }
  ImGui::Checkbox("Separate interior/boundary OBJ objects", &meshing_settings.export_separate_contributor_objects);
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip(
        "When enabled, debug and failed-meshlet OBJs contain one object (o) per interior/boundary contributor "
        "(iN / bN). Disable to write a single object per file.");
  }
  ImGui::Checkbox("Store mesh metadata", &meshing_settings.store_mesh_metadata);
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip("Store JSON vertex/face metadata on meshlets (TreeMesher store_mesh_metadata).");
  }
  ImGui::Checkbox("Hinge-only profile plane mix", &meshing_settings.hinge_only_profile_plane_mix);
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip(
        "Debug: for non-parallel (rotated) profile-plane interpolation, apply only the hinge about the "
        "intersection line — skip in-plane origin shift and rotation about the plane normal. "
        "Parallel mixes are unchanged. Affects meshing buffer hash.");
  }
  if (ImGui::DragFloat("Spline tension", &meshing_settings.spline_tension, 0.01f, 0.0f, 1.0f)) {
    meshing_settings.spline_tension = glm::clamp(meshing_settings.spline_tension, 0.0f, 1.0f);
  }
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip(
        "Meshing-only blend for plane-spline sampling. 0 = Strands cubic (away from knots), 1 = Catmull-Rom (through "
        "knots).");
  }
  if (ImGui::DragInt("Bark smooth iterations", &meshing_settings.bark_smooth_iterations, 1, 0, 50)) {
    meshing_settings.bark_smooth_iterations = glm::max(0, meshing_settings.bark_smooth_iterations);
  }
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip(
        "Laplacian smooth welded bark triangles (same method as marching-cubes MeshSmoothing, without ground "
        "lock). Interior faces that share bark vertices move with them. 0 = off. Affects meshing buffer hash.");
  }
  ImGui::Checkbox("Subdivide bark", &meshing_settings.bark_subdivide);
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip(
        "1→4-subdivide each bark triangle via edge midpoints during prepare (independent of bark smooth "
        "iterations — still runs when smooth is 0). Each bark edge split is propagated to adjacent "
        "interior faces (same meshlet, and the neighboring meshlet at segment interfaces). Affects "
        "meshing buffer hash when enabled.");
  }
  if (ImGui::DragFloat("Bark smooth strength", &meshing_settings.bark_smooth_strength, 0.01f, 0.0f, 1.0f)) {
    meshing_settings.bark_smooth_strength = glm::clamp(meshing_settings.bark_smooth_strength, 0.0f, 1.0f);
  }
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip(
        "Per-iteration blend toward the neighbor average. 0 = no move, 1 = full Laplacian step (previous behavior). "
        "Try ~0.2–0.5 if one iteration is too strong. Affects meshing buffer hash when iterations > 0.");
  }
  ImGui::Checkbox("Lock bark boundary", &meshing_settings.bark_smooth_lock_boundary);
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip(
        "Keep vertices on the bark manifold boundary fixed during Laplacian smooth (edges that belong to only one "
        "bark triangle). Prevents bark edges from pulling inward. Affects meshing buffer hash when iterations > 0 "
        "and this is unchecked.");
  }
  ImGui::Checkbox("Smooth bark UVs", &meshing_settings.bark_smooth_uvs);
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip(
        "Co-smooth bark corner UVs with the same iterations and strength as positions. Circumferential wrap "
        "discontinuities are lifted into a continuous chart before averaging, then each bark triangle is "
        "re-unwrapped. Affects meshing buffer hash when iterations > 0 and this is unchecked.");
  }
  {
    float alpha_cutoff = static_cast<float>(meshing_settings.alpha_cutoff);
    if (ImGui::DragFloat("Alpha cutoff", &alpha_cutoff, 0.1f, 0.0f, 1e6f, "%.3f")) {
      meshing_settings.alpha_cutoff = static_cast<double>(glm::max(0.0f, alpha_cutoff));
    }
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip(
          "kinDS alpha / radius cutoff for inside-outside classification (TreeMesher alpha_cutoff). "
          "Affects radius events and boundary meshing.");
    }
  }
  {
    float branch_alpha_cutoff = static_cast<float>(meshing_settings.branch_alpha_cutoff);
    if (ImGui::DragFloat("Branch alpha cutoff", &branch_alpha_cutoff, 0.1f, 0.0f, 1e6f, "%.3f")) {
      meshing_settings.branch_alpha_cutoff = static_cast<double>(glm::max(0.0f, branch_alpha_cutoff));
    }
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip(
          "kinDS radius cutoff for Delaunay triangles whose three strands are not on the same input branch "
          "(TreeMesher branch_alpha_cutoff). Disabled when equal to Alpha cutoff.");
    }
  }
  {
    int look_ahead = static_cast<int>(meshing_settings.look_ahead);
    if (ImGui::DragInt("Look ahead", &look_ahead, 1, 0, 1024)) {
      meshing_settings.look_ahead = static_cast<size_t>(glm::max(0, look_ahead));
    }
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip(
          "Extra sections above floor(t)+1 when deciding whether a triangle's strands share an input branch "
          "for Branch alpha cutoff (0 = default). Out-of-range heights clamp to the last valid index.");
    }
  }

  ImGui::DragFloat("UV height factor", &render_settings.segment_meshlet_render_parameters.uv_height_factor, 0.001f,
                   0.001f, 1.0f);
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip("Scales bark UV height (kinetic time / along-strand). Used for viewport shading and OBJ export.");
  }
  ImGui::DragFloat("UV circum factor", &render_settings.segment_meshlet_render_parameters.uv_circum_factor, 1.0f, 1.0f,
                   50.0f, "%.0f");
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip("Scales bark UV circumference (polar angle). Used for viewport shading and OBJ export.");
  }

  ImGui::Checkbox("Fix missing meshlets after intersection",
                  &meshing_settings.intersection_boundary_fix_missing_meshes);
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip("Attempt to repair empty meshlets after boundary intersection using neighbor triangles.");
  }
  ImGui::Checkbox("Keep original meshlet on intersection failure",
                  &meshing_settings.intersection_keep_original_on_failure);
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip(
        "If intersection fails (e.g. non-manifold), keep the uncut meshlet. Disable to replace it with an empty mesh.");
  }
  ImGui::Checkbox("Prefer meshlet UVs on intersection seam", &meshing_settings.intersection_prefer_meshlet_uv_on_seam);
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip(
        "Vertices lying on the original meshlet surface receive segment-meshlet UVs at the clip seam, even on "
        "boundary-origin faces.");
  }
  ImGui::Checkbox("Interior UVs on clip-boundary faces", &meshing_settings.intersection_boundary_faces_interior_uv);
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip(
        "Faces originating from the clip boundary use interior-style (a,b,h) UVs. Bark polar distance is treated as "
        "r=1 when converting.");
  }
}

bool eco_sys_lab_package::DsKineticVoronoiMeshing::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  InspectSharedMeshingSettings(editor_layer);

  // --- Helpers used by Add / Save / Load / Export all ---
  // Returns the DynamicTreeStrands owner entity for this meshing instance.
  const auto find_owner_entity = [&]() -> Entity {
    const auto scene = ApplicationContext::Get().GetActiveScene();
    if (!scene || !dynamic_strands) {
      return Entity{};
    }
    const auto dts_owners = scene->UnsafeGetPrivateComponentOwnersList<DynamicTreeStrands>();
    if (!dts_owners) {
      return Entity{};
    }
    Entity this_owner{};
    for (const auto& e : *dts_owners) {
      const auto dts = scene->GetOrSetPrivateComponent<DynamicTreeStrands>(e).lock();
      if (dts && dts->dynamic_strands && dts->dynamic_strands->GetKineticVoronoiMeshing() == this) {
        return e;
      }
    }
    return Entity{};
  };

  // Create a new Intersection Meshes group that contains only the selected OBJ.
  eco_sys_lab_package::OpenEditorFile(
      "Add Intersection Boundary Mesh", "OBJ", {".obj"},
      [&](const std::filesystem::path& path) {
        if (!dynamic_strands) {
          return;
        }
        const auto scene = ApplicationContext::Get().GetActiveScene();
        if (!scene) {
          return;
        }
        const Entity owner = find_owner_entity();
        if (!scene->IsEntityValid(owner)) {
          return;
        }
        try {
          kinDS::VoronoiMesh loaded_mesh = kinDS::ObjExporter::readMesh(path);
          const Entity group = scene->CreateEntity("Intersection Meshes (" + path.stem().string() + ")");
          scene->SetParent(group, owner);
          GlobalTransform group_gt{};
          group_gt.value = glm::mat4(1.0f);
          scene->SetDataComponent(group, group_gt);
          scene->GetOrSetPrivateComponent<DsIntersectionBoundaryMeshGroup>(group);

          const auto child = scene->CreateEntity("Intersection Mesh (" + path.stem().string() + ")");
          scene->SetParent(child, group);
          GlobalTransform child_gt{};
          child_gt.value = glm::mat4(1.0f);
          scene->SetDataComponent(child, child_gt);
          const auto ibm = scene->GetOrSetPrivateComponent<DsIntersectionBoundaryMesh>(child).lock();
          if (ibm) {
            ibm->LoadMesh(std::move(loaded_mesh), path);
          }
          EVOENGINE_LOG("Created Intersection Meshes group with boundary mesh from " << path.string() << ".");
        } catch (const std::exception& ex) {
          EVOENGINE_ERROR("Failed to load intersection boundary OBJ: " << ex.what());
        }
      },
      false);
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip(
        "Open a file dialog to pick an OBJ, then create a new Intersection Meshes group containing only that mesh. "
        "To add more meshes to an existing group, use Add intersection boundary mesh on the group entity.");
  }

  ImGui::SameLine();
  eco_sys_lab_package::OpenEditorFile(
      "Load intersection setup", "YAML", {".yml"},
      [&](const std::filesystem::path& load_path) {
        const auto scene = ApplicationContext::Get().GetActiveScene();
        const Entity owner = find_owner_entity();
        if (!scene || !scene->IsEntityValid(owner)) {
          EVOENGINE_ERROR("Load intersection setup: could not find owner entity.");
          return;
        }
        LoadIntersectionSetup(scene, owner, load_path);
      },
      false);
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip(
        "Load an intersection setup YAML into a newly created Intersection Meshes group "
        "(existing groups are left untouched).");
  }

  const bool can_reset_meshlets = HasMeshedSegmentMeshlets();
  if (!can_reset_meshlets) {
    ImGui::BeginDisabled();
  }
  if (ImGui::Button("Reset meshlets")) {
    ResetMeshletsToGpu();
  }
  if (!can_reset_meshlets) {
    ImGui::EndDisabled();
  }
  if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
    ImGui::SetTooltip(
        "Reload pristine meshlets (from the last meshing run) into GPU buffers without intersection. "
        "Requires a completed meshing run.");
  }

  eco_sys_lab_package::SaveEditorFile(
      "Download and export Kinetic meshlets (PLY)", "PLY", {".ply"},
      [&](const std::filesystem::path& path) {
        dynamic_strands->Download();
        EVOENGINE_LOG("Downloaded data from GPU");
        PlyExporter::ExportAscii(path, segment_meshlet_vertices, segment_meshlet_triangles,
                                 render_settings.segment_meshlet_render_parameters.uv_height_factor,
                                 render_settings.segment_meshlet_render_parameters.uv_circum_factor);
      },
      false);
  ImGui::SameLine();
  eco_sys_lab_package::SaveEditorFile(
      "Export Kinetic meshlets (PLY)", "PLY", {".ply"},
      [&](const std::filesystem::path& path) {
        PlyExporter::ExportAscii(path, segment_meshlet_vertices, segment_meshlet_triangles,
                                 render_settings.segment_meshlet_render_parameters.uv_height_factor,
                                 render_settings.segment_meshlet_render_parameters.uv_circum_factor);
      },
      false);

  eco_sys_lab_package::SaveEditorFile(
      "Download and export Kinetic meshlets (OBJ)", "OBJ", {".obj"},
      [&](const std::filesystem::path& path) {
        dynamic_strands->Download();
        EVOENGINE_LOG("Downloaded data from GPU");
        MeshletObjExport::ExportObj(
            path, segment_meshlet_vertices, segment_meshlet_triangles, dynamic_strands->segments,
            render_settings.segment_meshlet_render_parameters.uv_height_factor,
            render_settings.segment_meshlet_render_parameters.uv_circum_factor,
            render_settings.segment_meshlet_render_parameters.fracture_distance, dynamic_strands->segment_pairs,
            dynamic_strands->segment_data_list, segment_meshlet_vertex_metadata, segment_meshlet_face_metadata);
      },
      false);
  ImGui::SameLine();
  eco_sys_lab_package::SaveEditorFile(
      "Export Kinetic meshlets (OBJ)", "OBJ", {".obj"},
      [&](const std::filesystem::path& path) {
        MeshletObjExport::ExportObj(
            path, segment_meshlet_vertices, segment_meshlet_triangles, dynamic_strands->segments,
            render_settings.segment_meshlet_render_parameters.uv_height_factor,
            render_settings.segment_meshlet_render_parameters.uv_circum_factor,
            render_settings.segment_meshlet_render_parameters.fracture_distance, dynamic_strands->segment_pairs,
            dynamic_strands->segment_data_list, segment_meshlet_vertex_metadata, segment_meshlet_face_metadata);
      },
      false);
  ImGui::SameLine();
  eco_sys_lab_package::SaveEditorFile(
      "Export volume change heatmap (OBJ)", "OBJ", {".obj"},
      [&](const std::filesystem::path& path) {
        if (!has_initial_meshlet_volumes_) {
          EVOENGINE_ERROR("Volume change heatmap: no initial meshlet volumes. Complete Kinetic meshing first.");
          return;
        }
        try {
          dynamic_strands->Download();
          std::vector<GpuSegmentMeshletVertex> smoothed_vertices;
          if (!BuildSmoothedCurrentMeshletVertices(smoothed_vertices)) {
            EVOENGINE_ERROR("Volume change heatmap: download+bark-smooth failed.");
            return;
          }
          VolumeChangeHeatmapExport::ExportKineticMeshlets(path, smoothed_vertices, segment_meshlet_triangles,
                                                           initial_meshlet_volumes_by_segment_,
                                                           initial_meshlet_cumulative_volume_);
          EVOENGINE_LOG("Exported Kinetic volume change heatmap to " + path.string());
        } catch (const std::exception& e) {
          EVOENGINE_ERROR(std::string("Volume change heatmap export failed: ") + e.what());
        }
      },
      false);
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip(
        "Per-meshlet %% volume change vs rest-pose baseline at meshing time, after download + bark smooth. "
        "White=0%%, red=loss, blue=gain (clamped to +-30%% for materials). "
        "Respects Per-meshlet objects (object names include meshlet id and %% change). "
        "For a shared Kinetic+Alpha color scale, use Export both volume change heatmaps under Both meshing.");
  }
  ImGui::SameLine();
  if (strand_tree) {
    eco_sys_lab_package::SaveEditorFile(
        "Export Profile Planes", "OBJ", {".obj"},
        [&](const std::filesystem::path& path) {
          const kinDS::VoronoiMesh planes_mesh =
              BuildProfilePlanesDebugMesh(*strand_tree, initialize_parameters_.uniform_subdivision);
          if (planes_mesh.getTriangles().empty()) {
            EVOENGINE_WARNING("Export Profile Planes: no (height, branch) planes with sites in StrandTree.");
            return;
          }
          kinDS::ObjExporter::writeMesh(planes_mesh, path, 1.0, 1.0, {}, /*include_metadata=*/true);
          EVOENGINE_LOG("Exported profile-plane debug mesh (" + std::to_string(planes_mesh.getTriangles().size() / 3) +
                        " triangles) to " + path.string());
        },
        false);
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip(
          "OBJ of one quad per StrandTree profile plane (height × branch). "
          "Group suffix original/parallel/rotated; JSON metadata on vertices/faces. "
          "Extents from min/max site support points; placement from stored profile transforms.");
    }
  }
  ImGui::SameLine();
  if (has_bark_debug_mesh_) {
    eco_sys_lab_package::SaveEditorFile(
        "Export Bark Debug OBJ", "OBJ", {".obj"},
        [&](const std::filesystem::path& path) {
          // Same framework OBJ writer as GPU mesh export (per-corner vt/vn, geometric normals).
          kinDS::ObjWriteOptions options;
          options.uv_height_factor = render_settings.segment_meshlet_render_parameters.uv_height_factor;
          options.uv_circum_factor = render_settings.segment_meshlet_render_parameters.uv_circum_factor;
          options.framework_compatible = true;
          options.write_obj_groups = false;
          kinDS::ObjExporter::writeMesh(bark_debug_mesh_, path, options);
          EVOENGINE_LOG("Exported bark debug mesh (" + std::to_string(bark_debug_mesh_.getTriangleCount()) +
                        " triangles, " + std::to_string(bark_debug_mesh_.getVertexCount()) + " verts) to " +
                        path.string());
        },
        false);
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip(
          "Welded bark-only mesh after bark smooth + shared seam normal averaging (same PerTriangleCorner "
          "normals uploaded to GPU). Built at last GPU meshlet populate; same OBJ writer as Download+Export.");
    }
  } else {
    ImGui::BeginDisabled();
    ImGui::Button("Export Bark Debug OBJ");
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
      ImGui::SetTooltip("No bark debug mesh yet — run Kinetic Voronoi meshing / populate meshlets first.");
    }
  }

  if (ImGui::TreeNodeEx("Visualization Export", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::TextUnformatted("Color");
    ImGui::RadioButton("Strands##viz_color", reinterpret_cast<int*>(&MeshletObjExport::visualization_color_mode),
                       static_cast<int>(MeshletObjExport::VisualizationColorMode::Strands));
    ImGui::SameLine();
    ImGui::RadioButton("Segments##viz_color", reinterpret_cast<int*>(&MeshletObjExport::visualization_color_mode),
                       static_cast<int>(MeshletObjExport::VisualizationColorMode::Segments));
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip(
          "Solid materials match Visualization Segment mode (Strand color / Segment color); "
          "duplicate colors share one material.");
    }

    ImGui::TextUnformatted("Object grouping");
    ImGui::RadioButton("None##viz_grouping", reinterpret_cast<int*>(&MeshletObjExport::visualization_object_grouping),
                       static_cast<int>(MeshletObjExport::VisualizationObjectGrouping::Combined));
    ImGui::SameLine();
    ImGui::RadioButton("By highlight##viz_grouping",
                       reinterpret_cast<int*>(&MeshletObjExport::visualization_object_grouping),
                       static_cast<int>(MeshletObjExport::VisualizationObjectGrouping::ByHighlight));
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip(
          "None: one combined OBJ object (faces still colored by Color above).\n"
          "By highlight: one object per strand or segment matching Color.");
    }

    ImGui::PushID("visualization_export_obj");
    eco_sys_lab_package::SaveEditorFile(
        "Export OBJ", "OBJ", {".obj"},
        [&](const std::filesystem::path& path) {
          dynamic_strands->Download();
          EVOENGINE_LOG("Downloaded data from GPU");
          MeshletObjExport::ExportVisualizationObj(
              path, segment_meshlet_vertices, segment_meshlet_triangles, dynamic_strands->segments,
              MeshletObjExport::visualization_color_mode, MeshletObjExport::visualization_object_grouping,
              render_settings.segment_meshlet_render_parameters.uv_height_factor,
              render_settings.segment_meshlet_render_parameters.uv_circum_factor,
              render_settings.segment_meshlet_render_parameters.fracture_distance, dynamic_strands->segment_pairs,
              dynamic_strands->segment_data_list);
          EVOENGINE_LOG("Exported visualization OBJ to " + path.string());
        },
        false);
    ImGui::PopID();
    ImGui::TreePop();
  }

  ImGui::Checkbox("Export smoothing", &MeshletObjExport::enable_smoothing);
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip(
        "When enabled, ApplySmoothing runs before OBJ write using downloaded segment pairs / segment data "
        "connections.");
  }
  ImGui::Checkbox("Per-meshlet objects", &MeshletObjExport::per_meshlet_objects);
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip(
        "When enabled, OBJ export writes one object (o) per segment meshlet instead of a single combined "
        "object.");
  }
  ImGui::Checkbox("Separate bark OBJ group", &MeshletObjExport::separate_bark_obj_group);
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip(
        "On combined GPU mesh OBJ export, write bark faces as object `bark` and remaining faces as `interior` "
        "(helps isolate bark lighting in Blender). Ignored when Per-meshlet objects is on.");
  }

  // eco_sys_lab_package::SaveEditorFile(
  //     "Export Boundary OBJ", "OBJ", {".obj"},
  //     [&](const std::filesystem::path& path) {
  //       kinDS::ObjExporter::writeMesh(
  //           transformed_boundary_mesh, path, render_settings.segment_meshlet_render_parameters.uv_height_factor,
  //           render_settings.segment_meshlet_render_parameters.uv_circum_factor, boundary_distances_by_vertex);
  //     },
  //     false);

  if (strand_tree) {
    eco_sys_lab_package::SaveEditorFile(
        "Export Strand Tree", "TXT", {".txt"},
        [&](const std::filesystem::path& path) {
          strand_tree->saveToFile(path);
        },
        false);
  }
  return false;
}

void DsKineticVoronoiMeshing::Stats(const std::shared_ptr<EditorLayer>& editor_layer) {
  ImGui::Text((std::string("Segment Meshlets Vertices: ") + std::to_string(segment_meshlet_vertices.size())).c_str());
  ImGui::Text((std::string("Segment Meshlets Triangles: ") + std::to_string(segment_meshlet_triangles.size())).c_str());
}

void DsKineticVoronoiMeshing::OnInspectRenderSettings(const std::shared_ptr<EditorLayer>& editor_layer) {
  ImGui::Checkbox("Render Segment Meshlets", &render_settings.segment_meshlet_render_parameters.enabled);
  ImGui::Checkbox("Volume measure → CSV (download+smooth)", &render_settings.enable_volume_measure);
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip(
        "While Playing: every N frames, download meshlets, re-apply bark smooth, measure cumulative volume on CPU, "
        "and append absolute + %% of initial to Metadata/kinetic_volume_measure_<timestamp>.csv. "
        "GPU skinned volume is nearly rigid; this path captures bark-smooth volume change. "
        "Paused/stopped skips measure and CSV.");
  }
  if (ImGui::DragInt("Volume measure interval (frames)", &render_settings.volume_measure_interval_frames, 1, 1, 1000)) {
    render_settings.volume_measure_interval_frames = glm::max(1, render_settings.volume_measure_interval_frames);
  }
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip(
        "Download + bark smooth + CPU volume measure every N physics frames (default 10). "
        "Also drives volume-change heatmap buffer uploads while that color mode is active.");
  }
  if (render_settings.segment_meshlet_render_parameters.enabled) {
    if (ImGui::Button("Rebuild segment meshlet pipelines")) {
      BuildSegmentMeshletsRenderingPipelines();
    }

    const char* color_modes[] = {
        "Standard", "Normals", "UVs", "Pair", "Neighbor connectivity", "Neighbor tags", "Volume change heatmap"};
    ImGui::Combo("Color mode", &render_settings.segment_meshlet_render_parameters.color_mode, color_modes,
                 IM_ARRAYSIZE(color_modes));
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip(
          "Neighbor tags: brown = -2 (bark), blue = -1 (interior/open), green = >=0 (lateral), "
          "magenta = <-2 (out of range).\n"
          "Volume change heatmap: white = 0%%, red = loss, blue = gain (clamped to +-30%%); "
          "CPU volumes after download+smooth, updated every Volume measure interval frames while Playing.");
    }

    ImGui::Checkbox("Debug neighbor connectivity",
                    &render_settings.segment_meshlet_render_parameters.debug_neighbor_connectivity);
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip(
          "Color meshlet faces by lateral-neighbor state (viewport + OBJ export): grey = never had a neighbor or "
          "neighbor removed by compact, brown = bark, red = pair disconnected, green = pair still connected.");
    }

    // uv factors
    ImGui::DragFloat("UV height factor", &render_settings.segment_meshlet_render_parameters.uv_height_factor, 0.001f,
                     0.001f, 1.0f);
    ImGui::DragFloat("UV circum factor", &render_settings.segment_meshlet_render_parameters.uv_circum_factor, 1.0f,
                     1.0f, 50.0f, "%.0f");

    ImGui::DragFloat("Fracture distance", &render_settings.segment_meshlet_render_parameters.fracture_distance, 0.0001f,
                     0.0f, 2.0f, "%.4f");
  }
}

void eco_sys_lab_package::DsKineticVoronoiMeshing::RegisterRenderInstances(Handle& rendering_instance_handle,
                                                                           std::shared_ptr<Scene> scene,
                                                                           Entity& owner) {
  RegisterSegmentMeshletsRenderInstance(rendering_instance_handle, scene, owner);
}

void DsKineticVoronoiMeshing::RegisterSegmentMeshletsRenderInstance(Handle& rendering_instance_handle,
                                                                    std::shared_ptr<Scene> scene, Entity& owner) {
  const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>();
  if (!render_layer) {
    EVOENGINE_LOG("Failed to render! RenderLayer not present!")
    return;
  }
  const auto inner_wood_material = dynamic_strands->materials.inner_wood_material_ref.Get<Material>();
  const auto snow_material = dynamic_strands->materials.snow_material_ref.Get<Material>();
  if (const auto bark_material = dynamic_strands->materials.bark_material_ref.Get<Material>();
      bark_material && inner_wood_material && snow_material) {
    if (!dynamic_strands->segments.empty()) {
      if (segment_meshlet_point_light_render_pipeline && segment_meshlet_point_light_render_pipeline->Initialized()) {
        render_layer->RenderOpaqueToPointLightShadowMap([=](VkCommandBuffer vk_command_buffer, const auto& view) {
          return RenderSegmentMeshletsToPointLightShadowMap(render_settings.segment_meshlet_render_parameters,
                                                            vk_command_buffer, view);
        });
      }
      if (segment_meshlet_spot_light_render_pipeline && segment_meshlet_spot_light_render_pipeline->Initialized()) {
        render_layer->RenderOpaqueToSpotLightShadowMap([=](VkCommandBuffer vk_command_buffer, const auto& view) {
          return RenderSegmentMeshletsToSpotLightShadowMap(render_settings.segment_meshlet_render_parameters,
                                                           vk_command_buffer, view);
        });
      }
      if (segment_meshlet_directional_light_render_pipeline &&
          segment_meshlet_directional_light_render_pipeline->Initialized()) {
        render_layer->RenderOpaqueToDirectionalLightShadowMap([=](VkCommandBuffer vk_command_buffer, const auto& view) {
          return RenderSegmentMeshletsToDirectionalLightShadowMap(render_settings.segment_meshlet_render_parameters,
                                                                  vk_command_buffer, view);
        });
      }
      if (segment_meshlet_render_pipeline && segment_meshlet_masked_render_pipeline &&
          segment_meshlet_render_pipeline->Initialized() && segment_meshlet_masked_render_pipeline->Initialized()) {
        const auto current_render_storage =
            ApplicationContext::Get().GetLayer<RenderLayer>()->GetCurrentRenderInstanceStorage();
        const auto renderer_handle = rendering_instance_handle;
        int bark_material_index = -1;
        current_render_storage->RegisterRenderInstance(scene, owner, renderer_handle, bark_material,
                                                       &bark_material_index);
        const auto inner_material_index = current_render_storage->RegisterMaterial(inner_wood_material);
        const auto snow_material_index = current_render_storage->RegisterMaterial(snow_material);
        const auto register_material = [&](const std::shared_ptr<Material>& material, const int material_index) {
          const auto material_data = material->BuildGltfMaterialData();
          const auto material_class =
              ClassifyGltfRasterMaterial(material_data.shade_material, material->draw_settings.blending);
          const auto pipeline = material_class == GltfRasterMaterialClass::Masked
                                    ? segment_meshlet_masked_render_pipeline
                                    : segment_meshlet_render_pipeline;
          auto render = [=](const VkCommandBuffer vk_command_buffer,
                            const std::vector<VkRenderingAttachmentInfo>& geometry_pass_color_attachment_infos,
                            const RenderLayer::DeferredRenderingView& view) {
            return RenderSegmentMeshletsToCameraDeferred(
                renderer_handle, bark_material_index, inner_material_index, snow_material_index, material_index,
                pipeline, material->draw_settings.cull_mode, render_settings.segment_meshlet_render_parameters,
                vk_command_buffer, geometry_pass_color_attachment_infos, view, VK_POLYGON_MODE_FILL);
          };
          if (material_class == GltfRasterMaterialClass::Opaque) {
            render_layer->RawOpaqueRenderingAllCameras(std::move(render));
          } else if (material_class == GltfRasterMaterialClass::Masked) {
            render_layer->AlphaMaskedRenderingAllCameras(std::move(render));
          } else {
            EVOENGINE_ERROR(
                "Kinetic Voronoi material requires forward rendering, which this procedural renderer "
                "does not support.");
          }
        };
        register_material(bark_material, bark_material_index);
        if (inner_material_index != bark_material_index) {
          register_material(inner_wood_material, inner_material_index);
        }
      }
    }
  }
}

void eco_sys_lab_package::DsKineticVoronoiMeshing::Visualize(
    const std::shared_ptr<Camera>& target_camera, const DynamicStrandsInitializeParameters& initialize_parameters,
    const DynamicStrandsVisualizationParameters& visualization_parameters) {
  // TODO
}

void DsKineticVoronoiMeshing::BuildSegmentMeshletsRenderingPipelines() {
  segment_meshlet_point_light_render_pipeline = std::make_shared<GraphicsPipeline>();
  segment_meshlet_point_light_render_pipeline->task_shader = Shader::CreateTemporary(
      ShaderType::Task, Platform::GetShaderGlobalDefines(),
      std::filesystem::path("./EcoSysLabResources") /
          "Shaders/Graphics/Task/DynamicStrands/Rendering/KineticVoronoiMeshing/SegmentMeshlet.slang");
  segment_meshlet_point_light_render_pipeline->mesh_shader =
      Shader::CreateTemporary(ShaderType::Mesh, Platform::GetShaderGlobalDefines(),
                              std::filesystem::path("./EcoSysLabResources") /
                                  "Shaders/Graphics/Mesh/DynamicStrands/Rendering/KineticVoronoiMeshing/SegmentMeshlet/"
                                  "PointLightShadowMap.slang");
  segment_meshlet_point_light_render_pipeline->fragment_shader =
      Shader::CreateTemporary(ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
                              std::filesystem::path("./EcoSysLabResources") / "Shaders/Graphics/Fragment/Empty.slang");
  segment_meshlet_point_light_render_pipeline->geometry_type = GeometryType::Mesh;
  segment_meshlet_point_light_render_pipeline->descriptor_set_layouts.emplace_back(
      ApplicationContext::Get().GetLayer<RenderLayer>()->GetPerFrameDescriptorSetLayout());
  segment_meshlet_point_light_render_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
  segment_meshlet_point_light_render_pipeline->depth_attachment_format = Platform::Constants::shadow_map;
  segment_meshlet_point_light_render_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
  auto& point_light_push_constant_range =
      segment_meshlet_point_light_render_pipeline->push_constant_ranges.emplace_back();
  point_light_push_constant_range.size = sizeof(SegmentMeshletPushConstant);
  point_light_push_constant_range.offset = 0;
  point_light_push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
  CompleteGraphicsDescriptorLayouts(segment_meshlet_point_light_render_pipeline);
  segment_meshlet_point_light_render_pipeline->Initialize();
  // Descriptor set layout
  segment_meshlet_spot_light_render_pipeline = std::make_shared<GraphicsPipeline>();
  segment_meshlet_spot_light_render_pipeline->task_shader = Shader::CreateTemporary(
      ShaderType::Task, Platform::GetShaderGlobalDefines(),
      std::filesystem::path("./EcoSysLabResources") /
          "Shaders/Graphics/Task/DynamicStrands/Rendering/KineticVoronoiMeshing/SegmentMeshlet.slang");

  segment_meshlet_spot_light_render_pipeline->mesh_shader =
      Shader::CreateTemporary(ShaderType::Mesh, Platform::GetShaderGlobalDefines(),
                              std::filesystem::path("./EcoSysLabResources") /
                                  "Shaders/Graphics/Mesh/DynamicStrands/Rendering/KineticVoronoiMeshing/SegmentMeshlet/"
                                  "SpotLightShadowMap.slang");
  segment_meshlet_spot_light_render_pipeline->fragment_shader =
      Shader::CreateTemporary(ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
                              std::filesystem::path("./EcoSysLabResources") / "Shaders/Graphics/Fragment/Empty.slang");
  segment_meshlet_spot_light_render_pipeline->geometry_type = GeometryType::Mesh;
  segment_meshlet_spot_light_render_pipeline->descriptor_set_layouts.emplace_back(
      ApplicationContext::Get().GetLayer<RenderLayer>()->GetPerFrameDescriptorSetLayout());
  segment_meshlet_spot_light_render_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
  segment_meshlet_spot_light_render_pipeline->depth_attachment_format = Platform::Constants::shadow_map;
  segment_meshlet_spot_light_render_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
  auto& spot_light_push_constant_range =
      segment_meshlet_spot_light_render_pipeline->push_constant_ranges.emplace_back();
  spot_light_push_constant_range.size = sizeof(SegmentMeshletPushConstant);
  spot_light_push_constant_range.offset = 0;
  spot_light_push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
  CompleteGraphicsDescriptorLayouts(segment_meshlet_spot_light_render_pipeline);
  segment_meshlet_spot_light_render_pipeline->Initialize();
  // Descriptor set layout
  segment_meshlet_directional_light_render_pipeline = std::make_shared<GraphicsPipeline>();
  segment_meshlet_directional_light_render_pipeline->task_shader = Shader::CreateTemporary(
      ShaderType::Task, Platform::GetShaderGlobalDefines(),
      std::filesystem::path("./EcoSysLabResources") /
          "Shaders/Graphics/Task/DynamicStrands/Rendering/KineticVoronoiMeshing/SegmentMeshlet.slang");

  segment_meshlet_directional_light_render_pipeline->mesh_shader =
      Shader::CreateTemporary(ShaderType::Mesh, Platform::GetShaderGlobalDefines(),
                              std::filesystem::path("./EcoSysLabResources") /
                                  "Shaders/Graphics/Mesh/DynamicStrands/Rendering/KineticVoronoiMeshing/SegmentMeshlet/"
                                  "DirectionalLightShadowMap.slang");
  segment_meshlet_directional_light_render_pipeline->fragment_shader =
      Shader::CreateTemporary(ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
                              std::filesystem::path("./EcoSysLabResources") / "Shaders/Graphics/Fragment/Empty.slang");
  segment_meshlet_directional_light_render_pipeline->geometry_type = GeometryType::Mesh;
  segment_meshlet_directional_light_render_pipeline->descriptor_set_layouts.emplace_back(
      ApplicationContext::Get().GetLayer<RenderLayer>()->GetPerFrameDescriptorSetLayout());
  segment_meshlet_directional_light_render_pipeline->descriptor_set_layouts.emplace_back(
      DynamicStrands::strands_layout);
  segment_meshlet_directional_light_render_pipeline->depth_attachment_format = Platform::Constants::shadow_map;
  segment_meshlet_directional_light_render_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
  auto& directional_light_push_constant_range =
      segment_meshlet_directional_light_render_pipeline->push_constant_ranges.emplace_back();
  directional_light_push_constant_range.size = sizeof(SegmentMeshletPushConstant);
  directional_light_push_constant_range.offset = 0;
  directional_light_push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
  CompleteGraphicsDescriptorLayouts(segment_meshlet_directional_light_render_pipeline);
  segment_meshlet_directional_light_render_pipeline->Initialize();
  // Descriptor set layout
  segment_meshlet_render_pipeline = std::make_shared<GraphicsPipeline>();
  segment_meshlet_render_pipeline->task_shader = Shader::CreateTemporary(
      ShaderType::Task, Platform::GetShaderGlobalDefines(),
      std::filesystem::path("./EcoSysLabResources") /
          "Shaders/Graphics/Task/DynamicStrands/Rendering/KineticVoronoiMeshing/SegmentMeshlet.slang");
  segment_meshlet_render_pipeline->mesh_shader = Shader::CreateTemporary(
      ShaderType::Mesh, Platform::GetShaderGlobalDefines(),
      std::filesystem::path("./EcoSysLabResources") /
          "Shaders/Graphics/Mesh/DynamicStrands/Rendering/KineticVoronoiMeshing/SegmentMeshlet/Rendering.slang");
  segment_meshlet_render_pipeline->fragment_shader = Shader::CreateTemporary(
      ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
      std::filesystem::path("./EcoSysLabResources") /
          "Shaders/Graphics/Fragment/DynamicStrands/Rendering/KineticVoronoiMeshing/Branches.slang");
  segment_meshlet_render_pipeline->geometry_type = GeometryType::Mesh;
  segment_meshlet_render_pipeline->descriptor_set_layouts.emplace_back(
      ApplicationContext::Get().GetLayer<RenderLayer>()->GetPerFrameDescriptorSetLayout());
  segment_meshlet_render_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
  segment_meshlet_render_pipeline->descriptor_set_layouts.emplace_back(
      ApplicationContext::Get().GetLayer<RenderLayer>()->GetLightingDescriptorSetLayout());
  segment_meshlet_render_pipeline->depth_attachment_format = Platform::Constants::render_texture_depth;
  segment_meshlet_render_pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
  segment_meshlet_render_pipeline->color_attachment_formats = {
      Platform::Constants::g_buffer_attribute, Platform::Constants::g_buffer_attribute,
      Platform::Constants::g_buffer_attribute, Platform::Constants::g_buffer_attribute,
      Platform::Constants::g_buffer_utility};
  auto& push_constant_range = segment_meshlet_render_pipeline->push_constant_ranges.emplace_back();
  push_constant_range.size = sizeof(SegmentMeshletPushConstant);
  push_constant_range.offset = 0;
  push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
  CompleteGraphicsDescriptorLayouts(segment_meshlet_render_pipeline);
  segment_meshlet_render_pipeline->Initialize();
  segment_meshlet_masked_render_pipeline =
      CreateMaskedRawPipeline(segment_meshlet_render_pipeline, std::filesystem::path("./EcoSysLabResources") /
                                                                   "Shaders/Graphics/Fragment/DynamicStrands/Rendering/"
                                                                   "KineticVoronoiMeshing/BranchesMasked.slang");
}

uint32_t DsKineticVoronoiMeshing::RenderSegmentMeshletsToPointLightShadowMap(
    const SegmentMeshletsRenderParameters& render_parameters, const VkCommandBuffer vk_command_buffer,
    const RenderLayer::PointLightShadowMapView& view) const {
  if (!render_parameters.enabled) {
    return 0;
  }
  const uint32_t task_work_group_invocations =
      Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxPreferredTaskWorkGroupInvocations;
  SegmentMeshletPushConstant push_constant;
  push_constant.index1.sub_light_index = view.face_index;
  push_constant.index2.light_index = view.light_index;
  push_constant.vertex_count = segment_meshlet_vertices.size();
  push_constant.triangle_count = segment_meshlet_triangles.size();
  push_constant.color_mode = render_settings.segment_meshlet_render_parameters.color_mode;
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  segment_meshlet_point_light_render_pipeline->Bind(vk_command_buffer);
  segment_meshlet_point_light_render_pipeline->BindDescriptorSet(
      vk_command_buffer, 0, RenderLayer::GetPerFrameDescriptorSet()->GetVkDescriptorSet());
  segment_meshlet_point_light_render_pipeline->BindDescriptorSet(
      vk_command_buffer, 1, dynamic_strands->strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
  segment_meshlet_point_light_render_pipeline->BindDescriptorSet(
      vk_command_buffer, 3, geometry_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
  segment_meshlet_point_light_render_pipeline->states.ResetAllStates(0);
  segment_meshlet_point_light_render_pipeline->states.SetViewportScissor(view.viewport);
  segment_meshlet_point_light_render_pipeline->states.ApplyAllStates(vk_command_buffer);

  segment_meshlet_point_light_render_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
  const uint32_t count = Platform::DivUp(segment_meshlet_triangles.size(), task_work_group_invocations);
  segment_meshlet_point_light_render_pipeline->DrawMeshTasks(vk_command_buffer, count, 1, 1);
  return segment_meshlet_triangles.size();
}

uint32_t DsKineticVoronoiMeshing::RenderSegmentMeshletsToSpotLightShadowMap(
    const SegmentMeshletsRenderParameters& render_parameters, VkCommandBuffer vk_command_buffer,
    const RenderLayer::SpotLightShadowMapView& view) const {
  if (!render_parameters.enabled) {
    return 0;
  }
  const uint32_t task_work_group_invocations =
      Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxPreferredTaskWorkGroupInvocations;
  SegmentMeshletPushConstant push_constant;
  push_constant.index1.sub_light_index = 0;
  push_constant.index2.light_index = view.light_index;
  push_constant.vertex_count = segment_meshlet_vertices.size();
  push_constant.triangle_count = segment_meshlet_triangles.size();
  push_constant.color_mode = render_settings.segment_meshlet_render_parameters.color_mode;
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  segment_meshlet_spot_light_render_pipeline->Bind(vk_command_buffer);
  segment_meshlet_spot_light_render_pipeline->BindDescriptorSet(
      vk_command_buffer, 0, RenderLayer::GetPerFrameDescriptorSet()->GetVkDescriptorSet());
  segment_meshlet_spot_light_render_pipeline->BindDescriptorSet(
      vk_command_buffer, 1, dynamic_strands->strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
  segment_meshlet_spot_light_render_pipeline->BindDescriptorSet(
      vk_command_buffer, 3, geometry_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
  segment_meshlet_spot_light_render_pipeline->states.ResetAllStates(0);
  segment_meshlet_spot_light_render_pipeline->states.SetViewportScissor(view.viewport);
  segment_meshlet_spot_light_render_pipeline->states.ApplyAllStates(vk_command_buffer);

  segment_meshlet_spot_light_render_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
  const uint32_t count = Platform::DivUp(segment_meshlet_triangles.size(), task_work_group_invocations);
  segment_meshlet_spot_light_render_pipeline->DrawMeshTasks(vk_command_buffer, count, 1, 1);
  return segment_meshlet_triangles.size();
}

uint32_t DsKineticVoronoiMeshing::RenderSegmentMeshletsToDirectionalLightShadowMap(
    const SegmentMeshletsRenderParameters& render_parameters, VkCommandBuffer vk_command_buffer,
    const RenderLayer::DirectionalLightShadowMapView& view) const {
  if (!render_parameters.enabled) {
    return 0;
  }
  const uint32_t task_work_group_invocations =
      Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxPreferredTaskWorkGroupInvocations;
  SegmentMeshletPushConstant push_constant;
  push_constant.index1.sub_light_index = view.split_index;
  push_constant.index2.light_index = view.light_index;
  push_constant.vertex_count = segment_meshlet_vertices.size();
  push_constant.triangle_count = segment_meshlet_triangles.size();
  push_constant.color_mode = render_settings.segment_meshlet_render_parameters.color_mode;
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  segment_meshlet_directional_light_render_pipeline->Bind(vk_command_buffer);
  segment_meshlet_directional_light_render_pipeline->BindDescriptorSet(
      vk_command_buffer, 0, RenderLayer::GetPerFrameDescriptorSet()->GetVkDescriptorSet());
  segment_meshlet_directional_light_render_pipeline->BindDescriptorSet(
      vk_command_buffer, 1, dynamic_strands->strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
  segment_meshlet_directional_light_render_pipeline->BindDescriptorSet(
      vk_command_buffer, 3, geometry_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
  segment_meshlet_directional_light_render_pipeline->states.ResetAllStates(0);
  segment_meshlet_directional_light_render_pipeline->states.SetViewportScissor(view.viewport);
  segment_meshlet_directional_light_render_pipeline->states.ApplyAllStates(vk_command_buffer);

  segment_meshlet_directional_light_render_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
  const uint32_t count = Platform::DivUp(segment_meshlet_triangles.size(), task_work_group_invocations);
  segment_meshlet_directional_light_render_pipeline->DrawMeshTasks(vk_command_buffer, count, 1, 1);
  return segment_meshlet_triangles.size();
}

uint32_t DsKineticVoronoiMeshing::RenderSegmentMeshletsToCameraDeferred(
    const Handle& renderer_handle, int bark_material_index, int inner_wood_material_index, int snow_material_index,
    const int render_material_index, const std::shared_ptr<GraphicsPipeline>& pipeline, const VkCullModeFlags cull_mode,
    const SegmentMeshletsRenderParameters& render_parameters, VkCommandBuffer vk_command_buffer,
    const std::vector<VkRenderingAttachmentInfo>& geometry_pass_color_attachment_infos,
    const RenderLayer::DeferredRenderingView& view, VkPolygonMode polygon_mode) const {
  if (!render_parameters.enabled) {
    return 0;
  }
  if (!Platform::GetInstance().GetCapabilities().support_mesh_shader) {
    EVOENGINE_LOG("Failed to render! Mesh shader unsupported!")
    return 0;
  }

  // TODO: If we add any compute shaders, also check them here
  if (!pipeline || !pipeline->Initialized()) {
    return 0;
  }
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  const uint32_t task_work_group_invocations =
      Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxPreferredTaskWorkGroupInvocations;

  SegmentMeshletPushConstant render_push_constant;
  render_push_constant.index1.instance_index =
      ApplicationContext::Get().GetLayer<RenderLayer>()->GetCurrentRenderInstanceStorage()->GetRenderInstanceIndex(
          renderer_handle);
  render_push_constant.index2.camera_index = view.camera_index;
  render_push_constant.vertex_count = segment_meshlet_vertices.size();
  render_push_constant.triangle_count = segment_meshlet_triangles.size();
  render_push_constant.color_mode = EffectiveSegmentMeshletColorMode(render_parameters);
  render_push_constant.inner_wood_material_index = inner_wood_material_index;
  render_push_constant.bark_material_index = bark_material_index;
  render_push_constant.uv_height_factor = render_settings.segment_meshlet_render_parameters.uv_height_factor;
  render_push_constant.uv_circum_factor = render_settings.segment_meshlet_render_parameters.uv_circum_factor;
  render_push_constant.fracture_distance = render_settings.segment_meshlet_render_parameters.fracture_distance;
  render_push_constant.render_material_index = render_material_index;

  pipeline->states.ResetAllStates(geometry_pass_color_attachment_infos.size());
  pipeline->states.SetViewportScissor(view.viewport);
  pipeline->states.polygon_mode = polygon_mode;
  pipeline->states.line_width = 2.0f;
  pipeline->states.cull_mode = cull_mode;
  pipeline->states.ApplyAllStates(vk_command_buffer);

#ifdef USE_RENDERDOC
  if (rdoc_api) {
    rdoc_api->StartFrameCapture(NULL, NULL);
    EVOENGINE_LOG("RDOC API detected!");
  }
#endif  //  USERENDERDOC

  pipeline->Bind(vk_command_buffer);
  pipeline->BindDescriptorSet(vk_command_buffer, 0, RenderLayer::GetPerFrameDescriptorSet()->GetVkDescriptorSet());
  pipeline->BindDescriptorSet(vk_command_buffer, 1,
                              dynamic_strands->strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
  pipeline->BindDescriptorSet(vk_command_buffer, 3,
                              geometry_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
  pipeline->BindDescriptorSet(vk_command_buffer, 2, RenderLayer::GetLightingDescriptorSet()->GetVkDescriptorSet());

  pipeline->PushConstant(vk_command_buffer, 0, render_push_constant);

  const uint32_t count = Platform::DivUp(segment_meshlet_triangles.size(), task_work_group_invocations);
  pipeline->DrawMeshTasks(vk_command_buffer, count, 1, 1);
#ifdef USE_RENDERDOC
  if (rdoc_api)
    rdoc_api->EndFrameCapture(NULL, NULL);
#endif
  return dynamic_strands->segments.size();
}

/* uint32_t DsKineticVoronoiMeshing::RenderSegmentMeshletVisualizationToCameraDeferred(
    const Handle& renderer_handle, const DynamicStrandsInitializeParameters& initialize_parameters,
    const SmallSegmentsVisualizationRenderParameters& render_parameters, VkCommandBuffer vk_command_buffer,
    const std::vector<VkRenderingAttachmentInfo>& geometry_pass_color_attachment_infos,
    const RenderLayer::DeferredRenderingView& view) const {
}*/

DsKineticVoronoiMeshing::RenderSettings& DsKineticVoronoiMeshing::RefRenderSettings() {
  return render_settings;
}
