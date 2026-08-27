#include "BufferExporter.hpp"

#include <array>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <unordered_map>
#include <vector>

using namespace eco_sys_lab_plugin;

namespace {

struct ExactX0Key {
  uint32_t x = 0;
  uint32_t y = 0;
  uint32_t z = 0;

  static ExactX0Key From(const glm::vec3& v) {
    ExactX0Key key;
    std::memcpy(&key.x, &v.x, sizeof(float));
    std::memcpy(&key.y, &v.y, sizeof(float));
    std::memcpy(&key.z, &v.z, sizeof(float));
    return key;
  }

  bool operator==(const ExactX0Key& other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct ExactX0KeyHash {
  size_t operator()(const ExactX0Key& key) const {
    size_t h = static_cast<size_t>(key.x);
    h ^= static_cast<size_t>(key.y) + 0x9e3779b9 + (h << 6) + (h >> 2);
    h ^= static_cast<size_t>(key.z) + 0x9e3779b9 + (h << 6) + (h >> 2);
    return h;
  }
};

class UnionFind {
 public:
  explicit UnionFind(const size_t n) : parent_(n), rank_(n, 0) {
    for (size_t i = 0; i < n; ++i) {
      parent_[i] = i;
    }
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

bool AreSegmentsStillConnected(const int segment_a, const int segment_b,
                               const std::vector<DynamicStrands::GpuSegment>& segments,
                               const std::vector<DynamicStrands::GpuSegmentPair>& segment_pairs,
                               const std::vector<DynamicStrands::GpuSegmentData>& segment_data_list) {
  if (segment_a == segment_b) {
    return true;
  }
  if (segment_a < 0 || segment_b < 0 || static_cast<size_t>(segment_a) >= segment_data_list.size() ||
      static_cast<size_t>(segment_b) >= segment_data_list.size() ||
      static_cast<size_t>(segment_a) >= segments.size() || static_cast<size_t>(segment_b) >= segments.size()) {
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

    // Neighbor in the rod-element graph. Strand (prev/next) pairs use connectivity_integrity;
    // lateral bundle pairs use bend_twist_bundle_integrity.
    const auto& seg_a = segments[static_cast<size_t>(segment_a)];
    const auto& seg_b = segments[static_cast<size_t>(segment_b)];
    const bool strand_adjacent = seg_a.prev_handle == segment_b || seg_a.next_handle == segment_b ||
                                 seg_b.prev_handle == segment_a || seg_b.next_handle == segment_a;
    if (strand_adjacent) {
      return pair.connectivity_integrity > 0.f;
    }
    return pair.bend_twist_bundle_integrity > 0.f;
  }
  return false;
}

}  // namespace

bool MeshletObjExport::enable_smoothing = false;

void PlyExporter::ExportAscii(const std::filesystem::path& path,
                              const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletVertex>& vertices,
                              const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletTriangle>& triangles,
                              double uv_height_factor, double uv_circum_factor) {
  std::ofstream file(path);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open PLY file for writing");
  }

  WriteHeader(file, vertices.size(), triangles.size());
  WriteVertices(file, vertices);
  WriteFaces(file, triangles, uv_height_factor, uv_circum_factor);

  file.close();
}

void PlyExporter::WriteHeader(std::ofstream& file, size_t vertex_count, size_t face_count) {
  file << "ply\n";
  file << "format ascii 1.0\n";

  // Material convention
  file << "comment material 0 bark\n";
  file << "comment material 1 interior\n";

  // Vertices
  file << "element vertex " << vertex_count << "\n";
  file << "property float x\n";
  file << "property float y\n";
  file << "property float z\n";

  // Faces
  file << "element face " << face_count << "\n";
  file << "property list uchar int vertex_indices\n";
  file << "property list uchar float corner_normals\n";
  file << "property list uchar float corner_uvs\n";
  file << "property int material_id\n";

  file << "end_header\n";
}

void PlyExporter::WriteVertices(std::ofstream& file,
                                const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletVertex>& vertices) {
  for (const auto& v : vertices) {
    file << v.x.x << " " << v.x.y << " " << v.x.z << "\n";
  }
}

void PlyExporter::WriteFaces(std::ofstream& file,
                             const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletTriangle>& triangles,
                             double uv_height_factor, double uv_circum_factor) {
  for (const auto& t : triangles) {
    const int material_id = (t.neighbor_segment_index == -2) ? 0 : 1;

    // Vertex indices
    file << "3 " << t.vertex_index0 << " " << t.vertex_index1 << " " << t.vertex_index2 << " ";

    // Corner normals (3 * vec3)
    file << "9 ";
    for (int i = 0; i < 3; ++i) {
      file << t.normal[i].x << " " << t.normal[i].y << " " << t.normal[i].z << " ";
    }

    // Corner UVs (3 * vec3)
    file << "6 ";
    for (int i = 0; i < 3; ++i) {
      glm::vec4 uv = t.uv[i];

      if (material_id == 0) {
        uv.x *= uv_circum_factor;
        uv.y *= uv_height_factor;
      } else {
        uv.z *= uv_height_factor;
      }

      file << uv.x << " " << uv.y << " " << uv.z << " ";
    }

    // Material
    file << material_id << "\n";
  }
}

kinDS::VoronoiMesh MeshletObjExport::ToVoronoiMesh(
    const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletVertex>& vertices,
    const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletTriangle>& triangles, float fracture_distance) {
  kinDS::VoronoiMesh mesh(std::vector<std::string>{"bark", "interior"}, kinDS::PerTriangleCorner);

  for (const auto& v : vertices) {
    const glm::vec3 p = v.x - fracture_distance * v.shift;
    mesh.addVertex(glm::dvec3(p.x, p.y, p.z));
  }

  for (const auto& t : triangles) {
    const int material_id = (t.neighbor_segment_index == -2) ? 0 : 1;
    const size_t uv0 = mesh.addUV(glm::dvec3(t.uv[0].x, t.uv[0].y, t.uv[0].z));
    const size_t uv1 = mesh.addUV(glm::dvec3(t.uv[1].x, t.uv[1].y, t.uv[1].z));
    const size_t uv2 = mesh.addUV(glm::dvec3(t.uv[2].x, t.uv[2].y, t.uv[2].z));
    mesh.addTriangle(t.vertex_index0, t.vertex_index1, t.vertex_index2, uv0, uv1, uv2, material_id);
    for (int i = 0; i < 3; ++i) {
      mesh.addNormal(glm::dvec3(t.normal[i].x, t.normal[i].y, t.normal[i].z));
    }
  }

  return mesh;
}

kinDS::ObjExportGpuAttributes MeshletObjExport::BuildGpuAttributes(
    const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletVertex>& vertices,
    const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletTriangle>& triangles,
    const std::vector<DynamicStrands::GpuSegment>& segments, double uv_height_factor) {
  kinDS::ObjExportGpuAttributes attrs;
  const size_t n = vertices.size();
  attrs.color.resize(n);
  attrs.boundary_distance.resize(n);
  attrs.profile_position.resize(n);
  attrs.profile_polar_coordinate.resize(n);
  attrs.HC.resize(n);
  attrs.HL.resize(n);
  attrs.RW.resize(n);
  attrs.RB.resize(n);
  attrs.moisture.resize(n);
  attrs.position0.resize(n);
  attrs.direction0.resize(n);
  attrs.root_distance.resize(n);
  attrs.uv_3.assign(n, 0.0);
  attrs.has_neighbor.resize(triangles.size());

  for (size_t i = 0; i < n; ++i) {
    const unsigned int segment_index = vertices[i].segment_index;
    if (segment_index >= segments.size()) {
      continue;
    }
    const auto& seg = segments[segment_index];
    attrs.color[i] = glm::dvec4(seg.color.x, seg.color.y, seg.color.z, seg.color.w);
    attrs.boundary_distance[i] = seg.boundary_distance;
    attrs.profile_position[i] = glm::dvec2(seg.profile_position.x, seg.profile_position.y);
    attrs.profile_polar_coordinate[i] = glm::dvec2(seg.profile_polar_coordinate.x, seg.profile_polar_coordinate.y);
    attrs.HC[i] = seg.HC;
    attrs.HL[i] = seg.HL;
    attrs.RW[i] = seg.RW;
    attrs.RB[i] = seg.RB;
    attrs.moisture[i] = seg.moisture;

    const glm::vec3 p0 = seg.particle0.x0;
    const glm::vec3 p1 = seg.particle1.x0;
    const glm::vec3 avg = 0.5f * (p0 + p1);
    attrs.position0[i] = glm::dvec3(avg.x, avg.y, avg.z);
    const glm::vec3 direction = glm::normalize(p1 - p0);
    attrs.direction0[i] = glm::dvec3(direction.x, direction.y, direction.z);
    attrs.root_distance[i] = 0.5 * (seg.particle0.root_distance + seg.particle1.root_distance);
  }

  for (size_t tri_index = 0; tri_index < triangles.size(); ++tri_index) {
    const auto& tri = triangles[tri_index];
    attrs.has_neighbor[tri_index] = tri.neighbor_segment_index >= 0;
    const bool is_bark = (tri.neighbor_segment_index == -2);
    const std::array<unsigned int, 3> v_idxs = {tri.vertex_index0, tri.vertex_index1, tri.vertex_index2};
    for (int c = 0; c < 3; ++c) {
      const glm::vec4& uv = tri.uv[c];
      const double value =
          is_bark ? static_cast<double>(uv.y) * uv_height_factor : static_cast<double>(uv.z) * uv_height_factor;
      if (v_idxs[c] < attrs.uv_3.size()) {
        attrs.uv_3[v_idxs[c]] = value;
      }
    }
  }

  return attrs;
}

void MeshletObjExport::AppendGpuAttributes(kinDS::ObjExportGpuAttributes& dst,
                                           const kinDS::ObjExportGpuAttributes& src) {
  auto append = [](auto& d, const auto& s) {
    d.insert(d.end(), s.begin(), s.end());
  };
  append(dst.color, src.color);
  append(dst.boundary_distance, src.boundary_distance);
  append(dst.profile_position, src.profile_position);
  append(dst.profile_polar_coordinate, src.profile_polar_coordinate);
  append(dst.HC, src.HC);
  append(dst.HL, src.HL);
  append(dst.RW, src.RW);
  append(dst.RB, src.RB);
  append(dst.moisture, src.moisture);
  append(dst.position0, src.position0);
  append(dst.direction0, src.direction0);
  append(dst.root_distance, src.root_distance);
  append(dst.uv_3, src.uv_3);
  append(dst.has_neighbor, src.has_neighbor);
}

void MeshletObjExport::ApplySmoothing(
    std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletVertex>& vertices,
    std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletTriangle>& /*triangles*/,
    const std::vector<DynamicStrands::GpuSegment>& segments,
    const std::vector<DynamicStrands::GpuSegmentPair>& segment_pairs,
    const std::vector<DynamicStrands::GpuSegmentData>& segment_data_list) {
  if (vertices.size() < 2) {
    return;
  }

  std::unordered_map<ExactX0Key, std::vector<size_t>, ExactX0KeyHash> groups_by_x0;
  groups_by_x0.reserve(vertices.size());
  for (size_t i = 0; i < vertices.size(); ++i) {
    groups_by_x0[ExactX0Key::From(vertices[i].x0)].push_back(i);
  }

  for (auto& [x0_key, member_indices] : groups_by_x0) {
    (void)x0_key;
    const size_t member_count = member_indices.size();
    if (member_count < 2) {
      continue;
    }

    UnionFind uf(member_count);
    for (size_t i = 0; i < member_count; ++i) {
      const int segment_i = static_cast<int>(vertices[member_indices[i]].segment_index);
      for (size_t j = i + 1; j < member_count; ++j) {
        const int segment_j = static_cast<int>(vertices[member_indices[j]].segment_index);
        if (AreSegmentsStillConnected(segment_i, segment_j, segments, segment_pairs, segment_data_list)) {
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
      if (component.size() < 2) {
        continue;
      }
      glm::vec3 mean_x(0.f);
      for (const size_t vertex_index : component) {
        mean_x += vertices[vertex_index].x;
      }
      const float inv = 1.f / static_cast<float>(component.size());
      mean_x *= inv;
      for (const size_t vertex_index : component) {
        vertices[vertex_index].x = mean_x;
      }
    }
  }
}

void MeshletObjExport::ExportObj(const std::filesystem::path& path,
                                 const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletVertex>& vertices,
                                 const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletTriangle>& triangles,
                                 const std::vector<DynamicStrands::GpuSegment>& segments, double uv_height_factor,
                                 double uv_circum_factor, float fracture_distance,
                                 const std::vector<DynamicStrands::GpuSegmentPair>& segment_pairs,
                                 const std::vector<DynamicStrands::GpuSegmentData>& segment_data_list) {
  std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletVertex> export_vertices = vertices;
  std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletTriangle> export_triangles = triangles;
  if (enable_smoothing) {
    ApplySmoothing(export_vertices, export_triangles, segments, segment_pairs, segment_data_list);
  }

  kinDS::VoronoiMesh mesh = ToVoronoiMesh(export_vertices, export_triangles, fracture_distance);
  kinDS::ObjWriteOptions options;
  options.uv_height_factor = uv_height_factor;
  options.uv_circum_factor = uv_circum_factor;
  options.framework_compatible = true;
  options.write_obj_groups = false;
  options.gpu_attributes = BuildGpuAttributes(export_vertices, export_triangles, segments, uv_height_factor);
  kinDS::ObjExporter::writeMesh(mesh, path, options);
}

void MeshletObjExport::ExportObjCombined(const std::filesystem::path& path, const std::vector<MeshGroup>& groups,
                                         const std::vector<DynamicStrands::GpuSegment>& segments,
                                         double uv_height_factor, double uv_circum_factor, float fracture_distance,
                                         const std::vector<DynamicStrands::GpuSegmentPair>& segment_pairs,
                                         const std::vector<DynamicStrands::GpuSegmentData>& segment_data_list) {
  if (groups.empty()) {
    throw std::runtime_error("ExportObjCombined: no mesh groups to export");
  }

  kinDS::VoronoiMesh combined;
  kinDS::ObjExportGpuAttributes combined_attrs;
  bool initialized = false;

  for (const auto& group : groups) {
    std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletVertex> export_vertices = group.vertices;
    std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletTriangle> export_triangles = group.triangles;
    if (enable_smoothing) {
      ApplySmoothing(export_vertices, export_triangles, segments, segment_pairs, segment_data_list);
    }

    kinDS::VoronoiMesh part = ToVoronoiMesh(export_vertices, export_triangles, fracture_distance);
    kinDS::ObjExportGpuAttributes part_attrs =
        BuildGpuAttributes(export_vertices, export_triangles, segments, uv_height_factor);

    if (!initialized) {
      combined = std::move(part);
      combined.setGroupOffsets({0});
      combined.setGroupNames({group.name});
      combined_attrs = std::move(part_attrs);
      initialized = true;
    } else {
      combined.startNewGroup(group.name);
      combined += part;
      AppendGpuAttributes(combined_attrs, part_attrs);
    }
  }

  kinDS::ObjWriteOptions options;
  options.uv_height_factor = uv_height_factor;
  options.uv_circum_factor = uv_circum_factor;
  options.framework_compatible = true;
  options.write_obj_groups = true;
  options.gpu_attributes = std::move(combined_attrs);
  kinDS::ObjExporter::writeMesh(combined, path, options);
}
