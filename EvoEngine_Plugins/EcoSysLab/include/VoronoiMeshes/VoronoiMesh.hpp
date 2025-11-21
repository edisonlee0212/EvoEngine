#pragma once
#include <algorithm>
#include <vector>
#include "Point.hpp"

namespace kinDS {

class Mesh {
 private:
  std::vector<Point<3>> vertices;      // Stores vertex coordinates
  std::vector<size_t> vertex_indices;  // Stores indices of vertices forming triangles
  std::vector<Vector<3>> normals;      // Stores normal vectors for each vertex
  std::vector<Vector<2>> uvs;          // Stores texture coordinates for each vertex
  std::vector<size_t> uv_indices;      // Stores indices of texture coordinates for faces
  std::vector<size_t> group_offsets;   // Offsets for groups of triangles, if needed

  struct Vec3iHash {
    std::size_t operator()(const Point<3>& v) const noexcept {
      std::size_t h1 = std::hash<int>{}(v[0]);
      std::size_t h2 = std::hash<int>{}(v[1]);
      std::size_t h3 = std::hash<int>{}(v[2]);
      return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
  };

 public:
  Mesh() = default;
  Mesh(std::vector<Point<3>> vertices, std::vector<size_t> vertex_indices, std::vector<Vector<3>> normals = {},

       std::vector<Vector<2>> uvs = {}, std::vector<size_t> uv_indices = {})
      : vertices(std::move(vertices)),
        vertex_indices(std::move(vertex_indices)),
        normals(std::move(normals)),
        uvs(std::move(uvs)),
        uv_indices(std::move(uv_indices)),
        group_offsets(std::vector<size_t>(1, 0))  // Initialize with one group offset at 0
  {
    if (this->uv_indices.empty()) {
      // If no UV indices are provided, set the same length as vertex indices
      this->uv_indices.resize(this->vertex_indices.size(), std::numeric_limits<size_t>::max());
    }
  }

  ~Mesh() = default;

  // Add methods to manipulate the mesh, such as adding vertices, triangles, normals, and UVs
  size_t addVertex(double x, double y, double z);
  void addTriangle(size_t v1, size_t v2, size_t v3);
  void addTriangle(size_t v1, size_t v2, size_t v3, size_t uv1, size_t uv2, size_t uv3);
  void addNormal(double nx, double ny, double nz);
  void addUV(double u, double v);
  void startNewGroup();
  void setGroupOffsets(const std::vector<size_t>& offsets);
  Mesh& operator+=(const Mesh& other);
  void flipOrientation();

  // Merge duplicate vertices (within epsilon) and update triangle indices
  void mergeDuplicateVertices(double epsilon = 0.0);

  // Methods to retrieve mesh data
  const std::vector<Point<3>>& getVertices() const;
  std::vector<Point<3>>& getVertices();
  const std::vector<size_t>& getVertexIndices() const;
  std::vector<size_t>& getVertexIndices();
  const std::vector<Vector<3>>& getNormals() const;
  const std::vector<Vector<2>>& getUVs() const;
  const std::vector<size_t>& getUVIndices() const;
  const std::vector<size_t>& getGroupOffsets() const {
    return group_offsets;
  }

  // debug methods
  void checkForDegenerateTriangles() const;
};
}  // namespace kinDS
