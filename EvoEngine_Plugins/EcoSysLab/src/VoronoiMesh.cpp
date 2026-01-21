#include "VoronoiMesh.hpp"
#include <unordered_map>

// resolve a macro conflict with /usr/include/X11/X.h:350:21
#pragma push_macro("Success")
#undef Success

#include "Dense"

#pragma pop_macro("Success")

#ifdef USE_CGAL
#  include <CGAL/Polygon_mesh_processing/triangulate_hole.h>
#  include <CGAL/Simple_cartesian.h>
#  include <CGAL/Surface_mesh.h>

#  include <vector>

namespace PMP = CGAL::Polygon_mesh_processing;

typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_3 Point_3;
typedef CGAL::Surface_mesh<Point_3> Surface_mesh;
#endif

using namespace kinDS;

std::array<double, 3> barycentricCoordinates(const VoronoiPoint<3>& A, const VoronoiPoint<3>& B,
                                             const VoronoiPoint<3>& C, const VoronoiPoint<3>& P) {
  // Vectors
  const VoronoiVector<3> v0 = B - A;
  const VoronoiVector<3> v1 = C - A;
  const VoronoiVector<3> v2 = P - A;

  // Dot products
  const double d00 = v0 * v0;
  const double d01 = v0 * v1;
  const double d11 = v1 * v1;
  const double d20 = v2 * v0;
  const double d21 = v2 * v1;

  // Compute barycentric coordinates
  const double denom = d00 * d11 - d01 * d01;

  // Degenerate triangle check
  if (std::abs(denom) < 1e-15) {
    // Fallback: put everything on A
    return {1.0, 0.0, 0.0};
  }

  const double v = (d11 * d20 - d01 * d21) / denom;
  const double w = (d00 * d21 - d01 * d20) / denom;
  const double u = 1.0 - v - w;

  return {u, v, w};
}

size_t VoronoiMesh::addVertex(double x, double y, double z) {
  size_t index = vertices.size();
  vertices.emplace_back(VoronoiPoint<3>{x, y, z});
  return index;
}

size_t VoronoiMesh::addVertex(const VoronoiPoint<3>& p) {
  size_t index = vertices.size();
  vertices.emplace_back(p);
  return index;
}

size_t VoronoiMesh::addTriangle(size_t v1, size_t v2, size_t v3, int material_id) {
  return addTriangle(v1, v2, v3, std::numeric_limits<size_t>::max(), std::numeric_limits<size_t>::max(),
                     std::numeric_limits<size_t>::max(), material_id);
}

size_t VoronoiMesh::addTriangle(size_t v1, size_t v2, size_t v3, size_t uv1, size_t uv2, size_t uv3, int material_id) {
  size_t index = triangles.size() / 3;

  // Check vertex indices
  if (v1 >= vertices.size() || v2 >= vertices.size() || v3 >= vertices.size()) {
    throw std::out_of_range("Vertex index out of range when adding triangle.");
  }

  triangles.push_back(v1);
  triangles.push_back(v2);
  triangles.push_back(v3);
  uv_indices.push_back(uv1);
  uv_indices.push_back(uv2);
  uv_indices.push_back(uv3);

  material_ids.push_back(material_id);

  return index;
}

size_t VoronoiMesh::addNormal(double nx, double ny, double nz) {
  return addNormal(VoronoiVector<3>{nx, ny, nz});
}

size_t VoronoiMesh::addNormal(const VoronoiVector<3>& n) {
  size_t index = normals.size();
  normals.emplace_back(n);
  return index;
}

size_t VoronoiMesh::addUV(double u, double v, double w) {
  return addUV(VoronoiVector<3>{u, v, w});
}

size_t VoronoiMesh::addUV(VoronoiVector<3> uv) {
  size_t index = uvs.size();
  uvs.emplace_back(uv);
  return index;
}

void VoronoiMesh::startNewGroup() {
  group_offsets.push_back(triangles.size());  // Store the current vertex index count as a new group offset
}

void VoronoiMesh::setGroupOffsets(const std::vector<size_t>& offsets) {
  group_offsets = offsets;
}

VoronoiMesh& VoronoiMesh::operator+=(const VoronoiMesh& other) {
  if (normal_mode != other.normal_mode) {
    // Maybe we should implement auto-conversion at some point, but for now just throw an error
    throw std::runtime_error("Meshes don't use the same normal mode.");
  }

  size_t old_vertices_size = vertices.size();
  vertices.insert(vertices.end(), other.vertices.begin(), other.vertices.end());

  size_t old_vertex_indices_size = triangles.size();
  triangles.insert(triangles.end(), other.triangles.begin(), other.triangles.end());

  std::transform(triangles.begin() + old_vertex_indices_size, triangles.end(),
                 triangles.begin() + old_vertex_indices_size, [&](size_t index) {
                   return index + old_vertices_size;
                 });

  normals.insert(normals.end(), other.normals.begin(), other.normals.end());

  size_t old_uvs_size = uvs.size();
  uvs.insert(uvs.end(), other.uvs.begin(), other.uvs.end());

  size_t old_uv_indices_size = uv_indices.size();
  uv_indices.insert(uv_indices.end(), other.uv_indices.begin(), other.uv_indices.end());

  std::transform(uv_indices.begin() + old_uv_indices_size, uv_indices.end(), uv_indices.begin() + old_uv_indices_size,
                 [&](size_t index) {
                   return index + old_uvs_size;
                 });

  size_t old_group_count = group_offsets.size();
  group_offsets.insert(group_offsets.end(), other.group_offsets.begin(), other.group_offsets.end());

  std::transform(group_offsets.begin() + old_group_count, group_offsets.end(), group_offsets.begin() + old_group_count,
                 [&](size_t offset) {
                   return offset + old_vertex_indices_size;
                 });

  return *this;
}

void VoronoiMesh::flipOrientation() {
  // Flip the orientation of each triangle by swapping the second and third vertex indices
  for (size_t i = 0; i < triangles.size(); i += 3) {
    std::swap(triangles[i + 1], triangles[i + 2]);
    std::swap(uv_indices[i + 1], uv_indices[i + 2]);
  }

  // flip all normals
  for (size_t i = 0; i < normals.size(); i++) {
    normals[i] = -normals[i];
  }
}

void VoronoiMesh::mergeDuplicateVertices(double epsilon) {
  const double inv_eps = (epsilon > 0.0) ? 1.0 / epsilon : 0.0;
  std::unordered_map<VoronoiPoint<3>, size_t, VoronoiMesh::Vec3iHash> grid;
  std::vector<VoronoiPoint<3>> newVerts;
  newVerts.reserve(vertices.size());

  std::vector<size_t> remap(vertices.size(), size_t(-1));

  for (size_t i = 0; i < vertices.size(); ++i) {
    const auto& v = vertices[i];

    // Quantize vertex for approximate matching
    VoronoiPoint<3> key;
    if (epsilon > 0.0) {
      key[0] = static_cast<int>(std::llround(v[0] * inv_eps));
      key[1] = static_cast<int>(std::llround(v[1] * inv_eps));
      key[2] = static_cast<int>(std::llround(v[2] * inv_eps));
    } else {
      key[0] = static_cast<int>(std::hash<double>{}(v[0]) & 0x7FFFFFFF);
      key[1] = static_cast<int>(std::hash<double>{}(v[1]) & 0x7FFFFFFF);
      key[2] = static_cast<int>(std::hash<double>{}(v[2]) & 0x7FFFFFFF);
    }

    auto it = grid.find(key);
    if (it == grid.end()) {
      size_t newIndex = newVerts.size();
      grid[key] = newIndex;
      newVerts.push_back(v);
      remap[i] = newIndex;
    } else {
      remap[i] = it->second;
    }
  }

  // Remap triangle indices
  for (size_t& idx : triangles) {
    idx = remap[idx];
  }

  vertices.swap(newVerts);
}

void VoronoiMesh::patchHoles(std::function<void(size_t)> tri_callback,
                             std::function<void(size_t, size_t)> vertex_callback, int material_id) {
#ifdef USE_CGAL
  Surface_mesh mesh;

  // --- 1. Copy vertices (1:1 mapping) ---
  std::vector<Surface_mesh::Vertex_index> vmap(vertices.size());

  for (size_t i = 0; i < vertices.size(); ++i) {
    const auto& v = vertices[i];
    vmap[i] = mesh.add_vertex(Point_3(v[0], v[1], v[2]));
  }

  // --- 2. Copy faces ---
  const size_t original_face_count = triangles.size() / 3;

  for (size_t i = 0; i < triangles.size(); i += 3) {
    mesh.add_face(vmap[triangles[i + 0]], vmap[triangles[i + 1]], vmap[triangles[i + 2]]);
  }

  // --- 3. Fill holes ---
  for (auto h : mesh.halfedges()) {
    if (mesh.is_border(h)) {
      PMP::triangulate_hole(mesh, h, PMP::parameters::use_delaunay_triangulation(true));
    }
  }

  // --- 4. Append ONLY new triangles ---
  size_t face_index = 0;
  for (auto f : mesh.faces()) {
    if (face_index++ < original_face_count)
      continue;

    auto h = mesh.halfedge(f);
    size_t corner_index = triangles.size();
    size_t tri_index = corner_index / 3;
    std::array<size_t, 3> v_indices;

    for (int i = 0; i < 3; ++i) {
      v_indices[i] = mesh.target(h).idx();

      if (normal_mode == PerTriangleCorner) {
        normals.push_back(VoronoiVector<3>{0.0, 0.0, 0.0});
      }
      h = mesh.next(h);
    }
    addTriangle(v_indices[0], v_indices[1], v_indices[2], material_id);

    tri_callback(tri_index);
    for (int i = 0; i < 3; ++i) {
      vertex_callback(v_indices[i], corner_index + i);
    }
  }
#endif
}

std::vector<VoronoiVector<3>> kinDS::VoronoiMesh::computeVertexNormals() {
  std::vector<VoronoiVector<3>> vertex_normals(vertices.size(), VoronoiVector<3>{0.0, 0.0, 0.0});
  // Accumulate triangle normals into vertex normals
  for (size_t i = 0; i + 2 < triangles.size(); i += 3) {
    size_t i0 = triangles[i];
    size_t i1 = triangles[i + 1];
    size_t i2 = triangles[i + 2];

    const VoronoiPoint<3>& p0 = vertices[i0];
    const VoronoiPoint<3>& p1 = vertices[i1];
    const VoronoiPoint<3>& p2 = vertices[i2];

    VoronoiVector<3> e1 = p1 - p0;
    VoronoiVector<3> e2 = p2 - p0;

    // Unnormalized triangle normal (area-weighted)
    VoronoiVector<3> triNormal = e1 % e2;

    vertex_normals[i0] += triNormal;
    vertex_normals[i1] += triNormal;
    vertex_normals[i2] += triNormal;
  }

  // Normalize the accumulated vertex normals
  for (VoronoiVector<3>& n : vertex_normals) {
    if (n.len_sqr() > 0.0) {
      n = n.normalized();
    }
  }

  return vertex_normals;
}

void kinDS::VoronoiMesh::computeNormals(NormalMode normal_mode) {
  // Ensure normals has the correct size and is zero-initialized
  this->normal_mode = normal_mode;

  if (normal_mode == PerVertex) {
    normals = computeVertexNormals();
  } else if (normal_mode == PerTriangleCorner) {
    std::vector<VoronoiVector<3>> vertex_normals = computeVertexNormals();
    normals.resize(triangles.size(), VoronoiVector<3>{0.0, 0.0, 0.0});

    for (size_t i = 0; i < triangles.size(); i++) {
      normals[i] = vertex_normals[triangles[i]];
    }
  }
}

std::array<double, 3> kinDS::VoronoiMesh::computeBarycentricCoordinates(size_t triangle_index,
                                                                        VoronoiPoint<3>& point) const {
  return barycentricCoordinates(vertices[triangles[3 * triangle_index]], vertices[triangles[3 * triangle_index + 1]],
                                vertices[triangles[3 * triangle_index + 2]], point);
}

const std::vector<kinDS::VoronoiPoint<3>>& VoronoiMesh::getVertices() const {
  return vertices;
}

std::vector<VoronoiPoint<3>>& kinDS::VoronoiMesh::getVertices() {
  return vertices;
}

const std::vector<size_t>& VoronoiMesh::getTriangles() const {
  return triangles;
}

std::vector<size_t>& kinDS::VoronoiMesh::getTriangles() {
  return triangles;
}

const std::vector<VoronoiVector<3>>& VoronoiMesh::getNormals() const {
  return normals;
}

std::vector<VoronoiVector<3>>& VoronoiMesh::getNormals() {
  return normals;
}

const std::vector<VoronoiVector<3>>& VoronoiMesh::getUVs() const {
  return uvs;
}

const std::vector<size_t>& VoronoiMesh::getUVIndices() const {
  return uv_indices;
}

void VoronoiMesh::printStatistics() const {
  EVOENGINE_LOG("\nuv_indices.size(): " << uv_indices.size() << "\ntriangles.size(): " << triangles.size()
                                        << "\nuvs.size(): " << uvs.size() << "\nvertices.size():" << vertices.size());
}

bool kinDS::VoronoiMesh::hasValidUVIndex(size_t triangle_vertex_index) const {
  if (triangle_vertex_index >= uv_indices.size()) {
    EVOENGINE_ERROR("triangle_vertex_index: " << triangle_vertex_index << "\nuv_indices.size(): " << uv_indices.size()
                                              << "\ntriangles.size(): " << triangles.size() << "\nuvs.size(): "
                                              << uvs.size() << "\nvertices.size():" << vertices.size());
    throw std::out_of_range("Triangle vertex index out of range when checking UV index.");
  }
  return uv_indices[triangle_vertex_index] < uvs.size();
}

std::vector<size_t> kinDS::VoronoiMesh::findTriangleCorners(size_t vertex_index, bool stop_early) const {
  std::vector<size_t> corner_indices;

  for (size_t i = 0; i < triangles.size(); i++) {
    if (triangles[i] == vertex_index) {
      corner_indices.push_back(i);
      if (stop_early) {
      }
    }
  }

  return corner_indices;
}

std::vector<size_t>& VoronoiMesh::getUVIndices() {
  return uv_indices;
}

void VoronoiMesh::removeIsolatedVertices() {
  const size_t n_vertices = vertices.size();

  // 1. Mark used vertices
  std::vector<bool> used(n_vertices, false);
  for (size_t idx : triangles) {
    if (idx < n_vertices) {
      used[idx] = true;
    }
  }

  // 2. Build old -> new index map
  std::vector<size_t> remap(n_vertices, size_t(-1));
  size_t new_count = 0;

  for (size_t i = 0; i < n_vertices; ++i) {
    if (used[i]) {
      remap[i] = new_count++;
    } else {
      // EVOENGINE_LOG("Found unused vertex at index: " << i);
    }
  }

  // Early out: nothing to remove
  if (new_count == n_vertices) {
    return;
  }

  // 3. Compact vertex data
  std::vector<VoronoiPoint<3>> new_vertices;
  new_vertices.reserve(new_count);

  for (size_t i = 0; i < n_vertices; ++i) {
    if (used[i]) {
      new_vertices.push_back(vertices[i]);
    }
  }
  vertices.swap(new_vertices);

  // 4. Compact per-vertex normals if needed
  if (normal_mode == PerVertex) {
    std::vector<VoronoiVector<3>> new_normals;
    new_normals.reserve(new_count);

    for (size_t i = 0; i < n_vertices; ++i) {
      if (used[i]) {
        new_normals.push_back(normals[i]);
      }
    }
    normals.swap(new_normals);
  }

  // 5. Remap triangle vertex indices
  for (size_t& idx : triangles) {
    idx = remap[idx];
  }
}

void VoronoiMesh::removeDegenerateTriangles() {
  const size_t n_triangles = triangles.size() / 3;
  std::vector<size_t> new_triangles;
  std::vector<size_t> new_uv_indices;
  std::vector<int> new_material_ids;

  new_triangles.reserve(triangles.size());
  if (!uv_indices.empty()) {
    new_uv_indices.reserve(uv_indices.size());
  }
  new_material_ids.reserve(material_ids.size());

  for (size_t t = 0; t < n_triangles; ++t) {
    size_t i0 = triangles[3 * t + 0];
    size_t i1 = triangles[3 * t + 1];
    size_t i2 = triangles[3 * t + 2];

    // Check for duplicate vertices
    if (i0 != i1 && i0 != i2 && i1 != i2) {
      new_triangles.push_back(i0);
      new_triangles.push_back(i1);
      new_triangles.push_back(i2);

      // Keep uv_indices in sync if present
      if (!uv_indices.empty()) {
        new_uv_indices.push_back(uv_indices[3 * t + 0]);
        new_uv_indices.push_back(uv_indices[3 * t + 1]);
        new_uv_indices.push_back(uv_indices[3 * t + 2]);
      }

      new_material_ids.push_back(material_ids[t]);
    }
  }

  triangles.swap(new_triangles);
  if (!uv_indices.empty()) {
    uv_indices.swap(new_uv_indices);
  }
  material_ids.swap(new_material_ids);

  // Optionally: warn if triangles were removed
  size_t removed = n_triangles - (triangles.size() / 3);
  if (removed > 0) {
    // std::cerr << "Removed " << removed << " degenerate triangles.\n";
  }
}

const VoronoiVector<3>& kinDS::VoronoiMesh::getNormal(size_t triangle_vertex_index) const {
  if (normal_mode == PerTriangleCorner) {
    return normals[triangle_vertex_index];
  } else {
    return normals[triangles[triangle_vertex_index]];
  }
}

const VoronoiVector<3>& kinDS::VoronoiMesh::getUV(size_t triangle_vertex_index) const {
  return uvs[uv_indices[triangle_vertex_index]];
}

void kinDS::VoronoiMesh::setNormal(const VoronoiVector<3>& normal, size_t triangle_vertex_index) {
  if (normal_mode == PerTriangleCorner) {
    if (triangle_vertex_index >= normals.size()) {
      throw std::out_of_range("Triangle vertex index out of range when setting normal.");
    }
    normals[triangle_vertex_index] = normal;
  } else {
    size_t vertex_index = triangles[triangle_vertex_index];
    if (vertex_index >= normals.size()) {
      throw std::out_of_range("Vertex index out of range when setting normal.");
    }
    normals[vertex_index] = normal;
  }
}

void kinDS::VoronoiMesh::setUV(const VoronoiVector<3>& uv, size_t triangle_vertex_index) {
  size_t uv_index = uv_indices[triangle_vertex_index];
  if (uv_index >= uvs.size()) {
    uv_index = uvs.size();
    uvs.emplace_back(uv);
    uv_indices[triangle_vertex_index] = uv_index;
    return;
  }
  uvs[uv_index] = uv;
}

NormalMode kinDS::VoronoiMesh::getNormalMode() const {
  return normal_mode;
}

void kinDS::VoronoiMesh::checkForDegenerateTriangles() const {
  size_t degenerate_faces = 0;
  for (size_t i = 0; i < triangles.size(); i += 3) {
    const auto& p0 = vertices[triangles[i]];
    const auto& p1 = vertices[triangles[i + 1]];
    const auto& p2 = vertices[triangles[i + 2]];

    // compute squared area via cross product
    Eigen::Vector3d v0(p0[0], p0[1], p0[2]);
    Eigen::Vector3d v1(p1[0], p1[1], p1[2]);
    Eigen::Vector3d v2(p2[0], p2[1], p2[2]);
    double area2 = ((v1 - v0).cross(v2 - v0)).squaredNorm();

    if (area2 < 1e-20) {
      degenerate_faces++;
    }
  }
  if (degenerate_faces != 0)
    std::cerr << "Degenerate triangles: " << degenerate_faces << "/" << (triangles.size() / 3) << "\n";
}