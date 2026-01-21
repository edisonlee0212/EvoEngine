#pragma once
#include <algorithm>
#include <vector>
#include "VoronoiPoint.hpp"

namespace kinDS {

enum NormalMode { PerVertex, PerTriangleCorner };

class VoronoiMesh {
 private:
  std::vector<VoronoiPoint<3>> vertices;    // Stores vertex coordinates
  std::vector<size_t> triangles;            // Stores indices of vertices forming triangles
  std::vector<VoronoiVector<3>> normals;    // Stores normal vectors for each triangle or vertex depending on node
  std::vector<VoronoiVector<3>> uvs;        // Stores texture coordinates
  std::vector<size_t> uv_indices;           // Stores indices of texture coordinates for face corners
  std::vector<size_t> group_offsets;        // Offsets for groups of triangles, if needed
  std::vector<std::string> material_names;  // Stores material names - needed for exporting
  std::vector<int> material_ids;            // Stores material IDs per triangle

  NormalMode normal_mode;

  struct Vec3iHash {
    std::size_t operator()(const VoronoiPoint<3>& v) const noexcept {
      std::size_t h1 = std::hash<int>{}(v[0]);
      std::size_t h2 = std::hash<int>{}(v[1]);
      std::size_t h3 = std::hash<int>{}(v[2]);
      return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
  };

 public:
  VoronoiMesh(std::vector<std::string> material_names = {}, NormalMode normal_mode = PerTriangleCorner)
      : material_names(std::move(material_names)), normal_mode(normal_mode) {};

  VoronoiMesh(std::vector<VoronoiPoint<3>> vertices, std::vector<size_t> triangles,
              std::vector<std::string> material_names = {}, std::vector<VoronoiVector<3>> normals = {},
              std::vector<VoronoiVector<3>> uvs = {}, std::vector<size_t> uv_indices = {},
              NormalMode normal_mode = PerTriangleCorner)
      : vertices(std::move(vertices)),
        triangles(std::move(triangles)),
        material_names(std::move(material_names)),
        normals(std::move(normals)),
        uvs(std::move(uvs)),
        uv_indices(std::move(uv_indices)),
        group_offsets(std::vector<size_t>(1, 0)),  // Initialize with one group offset at 0
        normal_mode(normal_mode) {
    if (this->uv_indices.empty()) {
      // If no UV indices are provided, set the same length as vertex indices
      this->uv_indices.resize(this->triangles.size(), std::numeric_limits<size_t>::max());
    }
  }

  ~VoronoiMesh() = default;

  // methods to manipulate the mesh, such as adding vertices, triangles, normals, and UVs
  size_t addVertex(double x, double y, double z);
  size_t addVertex(const VoronoiPoint<3>& p);
  size_t addTriangle(size_t v1, size_t v2, size_t v3, int material_id = -1);
  size_t addTriangle(size_t v1, size_t v2, size_t v3, size_t uv1, size_t uv2, size_t uv3, int material_id = -1);
  size_t addNormal(double nx, double ny, double nz);
  size_t addNormal(const VoronoiVector<3>& n);
  size_t addUV(double u, double v, double w);
  size_t addUV(VoronoiVector<3> uv);
  void startNewGroup();
  void setGroupOffsets(const std::vector<size_t>& offsets);
  VoronoiMesh& operator+=(const VoronoiMesh& other);
  void flipOrientation();

  // Merge duplicate vertices (within epsilon) and update triangle indices
  void mergeDuplicateVertices(double epsilon = 0.0);
  void patchHoles(std::function<void(size_t)> tri_callback, std::function<void(size_t, size_t)> vertex_callback,
                  int material_id = -1);

  // compute normals
  void computeNormals(NormalMode normal_mode = PerVertex);

  std::array<double, 3> computeBarycentricCoordinates(size_t triangle_index, VoronoiPoint<3>& point) const;

  // Methods to retrieve mesh data
  const std::vector<VoronoiPoint<3>>& getVertices() const;
  std::vector<VoronoiPoint<3>>& getVertices();
  const std::vector<size_t>& getTriangles() const;
  std::vector<size_t>& getTriangles();
  const std::vector<VoronoiVector<3>>& getNormals() const;
  std::vector<VoronoiVector<3>>& getNormals();
  const std::vector<VoronoiVector<3>>& getUVs() const;
  const std::vector<size_t>& getUVIndices() const;
  void printStatistics() const;
  bool hasValidUVIndex(size_t triangle_vertex_index) const;
  std::vector<size_t> findTriangleCorners(size_t vertex_index, bool stop_early = false) const;

  std::vector<size_t>& getUVIndices();

  const std::vector<int>& getMaterialIDs() const {
    return material_ids;
  }

  const std::vector<std::string>& getMaterialNames() const {
    return material_names;
  }

  void removeIsolatedVertices();

  void removeDegenerateTriangles();

  /**
   * Provides a mode-independent way to get the normal from a triangle vertex
   * @param triangle_vertex_index index corresponding to the indices in the triangle buffer
   * @return
   */
  const VoronoiVector<3>& getNormal(size_t triangle_vertex_index) const;
  const VoronoiVector<3>& getUV(size_t triangle_vertex_index) const;

  void setNormal(const VoronoiVector<3>& normal, size_t triangle_vertex_index);
  void setUV(const VoronoiVector<3>& uv, size_t triangle_vertex_index);

  NormalMode getNormalMode() const;

  const std::vector<size_t>& getGroupOffsets() const {
    return group_offsets;
  }

  const size_t getTriangleCount() const {
    return triangles.size() / 3;
  }

  const size_t getVertexCount() const {
    return vertices.size();
  }

  // debug methods
  void checkForDegenerateTriangles() const;

 private:
  std::vector<VoronoiVector<3>> computeVertexNormals();
};
}  // namespace kinDS