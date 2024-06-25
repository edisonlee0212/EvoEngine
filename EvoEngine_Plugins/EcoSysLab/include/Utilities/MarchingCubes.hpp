/*
    Tables and conventions from
    http://paulbourke.net/geometry/polygonise/
*/

#pragma once

#include <Vertex.hpp>

using namespace evo_engine;
namespace eco_sys_lab {
struct TestingCell {
  glm::vec3 m_position;
};

struct MarchingCubeCell {
  glm::vec3 m_vertex[8];
  float m_value[8];
};

class MarchingCubes {
 public:
  /// m_edgeToVertices[i] = {a, b} => edge i joins vertices a and b
  static std::vector<std::pair<int, int>> m_edgeToVertices;

  /// m_edgeTable[i] is a 12 bit number; i is a cubeIndex
  /// m_edgeTable[i][j] = 1 if isosurface intersects edge j for cubeIndex i
  static int m_edgeTable[256];

  /// m_triangleTable[i] is a list of edges forming triangles for cubeIndex i
  static int m_triangleTable[256][16];
  /// Get triangles of a single cell
  static void TriangulateCell(MarchingCubeCell& cell, float isovalue, std::vector<Vertex>& vertices);

  /// Triangulate a scalar field represented by `scalarFunction`. `isovalue` should be used for isovalue computation
  static void TriangulateField(const glm::vec3& center,
                               const std::function<float(const glm::vec3& samplePoint)>& sampleFunction, float isovalue,
                               float cellSize, const std::vector<TestingCell>& testingCells,
                               std::vector<Vertex>& vertices, std::vector<unsigned>& indices, bool removeDuplicate);
};
}  // namespace eco_sys_lab