#pragma once
#include <algorithm>
#include <vector>
#include "Point.hpp"

namespace kinDS {

enum NormalMode { PerVertex, PerTriangleCorner };

class VoronoiMesh {
 private:
  std::vector<Point<3>> vertices;     // Stores vertex coordinates
  std::vector<size_t> triangles;      // Stores indices of vertices forming triangles
  std::vector<Vector<3>> normals;     // Stores normal vectors for each vertex
  std::vector<Vector<2>> uvs;         // Stores texture coordinates for each vertex
  std::vector<size_t> uv_indices;     // Stores indices of texture coordinates for faces
  std::vector<size_t> group_offsets;  // Offsets for groups of triangles, if needed

  NormalMode normal_mode;

  struct Vec3iHash {
    std::size_t operator()(const Point<3>& v) const noexcept {
      std::size_t h1 = std::hash<int>{}(v[0]);
      std::size_t h2 = std::hash<int>{}(v[1]);
      std::size_t h3 = std::hash<int>{}(v[2]);
      return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
  };

 public:
  VoronoiMesh(NormalMode normal_mode = PerTriangleCorner) : normal_mode(normal_mode) {};

  VoronoiMesh(std::vector<Point<3>> vertices, std::vector<size_t> triangles, std::vector<Vector<3>> normals = {},

              std::vector<Vector<2>> uvs = {}, std::vector<size_t> uv_indices = {})
      : vertices(std::move(vertices)),
        triangles(std::move(triangles)),
        normals(std::move(normals)),
        uvs(std::move(uvs)),
        uv_indices(std::move(uv_indices)),
        group_offsets(std::vector<size_t>(1, 0))  // Initialize with one group offset at 0
  {
    if (this->uv_indices.empty()) {
      // If no UV indices are provided, set the same length as vertex indices
      this->uv_indices.resize(this->triangles.size(), std::numeric_limits<size_t>::max());
    }
  }

  ~VoronoiMesh() = default;

  // methods to manipulate the mesh, such as adding vertices, triangles, normals, and UVs
  size_t addVertex(double x, double y, double z);
  size_t addVertex(const Point<3>& p);
  size_t addTriangle(size_t v1, size_t v2, size_t v3);
  size_t addTriangle(size_t v1, size_t v2, size_t v3, size_t uv1, size_t uv2, size_t uv3);
  size_t addNormal(double nx, double ny, double nz);
  size_t addNormal(const Vector<3>& n);
  size_t addUV(double u, double v);
  size_t addUV(Vector<2> uv);
  void startNewGroup();
  void setGroupOffsets(const std::vector<size_t>& offsets);
  VoronoiMesh& operator+=(const VoronoiMesh& other);
  void flipOrientation();

  // Merge duplicate vertices (within epsilon) and update triangle indices
  void mergeDuplicateVertices(double epsilon = 0.0);

  // compute normals
  void computeNormals(NormalMode normal_mode = PerVertex);

  std::array<double, 3> computeBarycentricCoordinates(size_t triangle_index, Point<3>& point) const;

  // Methods to retrieve mesh data
  const std::vector<Point<3>>& getVertices() const;
  std::vector<Point<3>>& getVertices();
  const std::vector<size_t>& getTriangles() const;
  std::vector<size_t>& getTriangles();
  const std::vector<Vector<3>>& getNormals() const;
  std::vector<Vector<3>>& getNormals();
  const std::vector<Vector<2>>& getUVs() const;
  const std::vector<size_t>& getUVIndices() const;
  bool hasValidUVIndex(size_t triangle_vertex_index) const;

  std::vector<size_t>& getUVIndices();

  /**
   * Provides a mode-independent way to get the normal from a triangle vertex
   * @param triangle_vertex_index index corresponding to the indices in the triangle buffer
   * @return
   */
  const Vector<3>& getNormal(size_t triangle_vertex_index) const;
  const Vector<2>& getUV(size_t triangle_vertex_index) const;

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
  std::vector<Vector<3>> computeVertexNormals();
};
}  // namespace kinDS