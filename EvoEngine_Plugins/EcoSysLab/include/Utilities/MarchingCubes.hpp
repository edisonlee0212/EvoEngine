
/*
    Tables and conventions from
    http://paulbourke.net/geometry/polygonise/
*/

#pragma once

#include <Vertex.hpp>

using namespace evo_engine;

namespace eco_sys_lab_plugin {

/**
 * @struct TestingCell
 * @brief Represents a simple testing cell with a position in 3D space.
 */
struct TestingCell {
  glm::vec3 m_position;  ///< The position of the testing cell.
};

/**
 * @struct MarchingCubeCell
 * @brief Represents a cell for the Marching Cubes algorithm.
 */
struct MarchingCubeCell {
  glm::vec3 m_vertex[8];  ///< The eight vertices of the cube.
  float m_value[8];       ///< The scalar values at each vertex of the cube.
};

/**
 * @class MarchingCubes
 * @brief Implements the Marching Cubes algorithm for isosurface extraction.
 */
class MarchingCubes {
 public:
  /// @brief Maps edges to their corresponding vertex indices.
  /// @details m_edgeToVertices[i] = {a, b} means that edge `i` connects vertices `a` and `b`.
  static std::vector<std::pair<int, int>> m_edgeToVertices;

  /// @brief Lookup table used to determine which edges are intersected by the isosurface.
  /// @details m_edgeTable[i] is a 12-bit number where each bit represents whether an edge is intersected.
  static int m_edgeTable[256];

  /// @brief Lookup table that defines the triangulation for each possible cube configuration.
  /// @details m_triangleTable[i] is a list of edges that form triangles for cube index `i`.
  static int m_triangleTable[256][16];

  /**
   * @brief Extracts triangles from a single cell based on a given isovalue.
   * @param cell The MarchingCubeCell to process.
   * @param isovalue The threshold value used for the isosurface extraction.
   * @param vertices The output vector that stores the generated vertices.
   */
  static void TriangulateCell(MarchingCubeCell& cell, float isovalue, std::vector<Vertex>& vertices);

  /**
   * @brief Triangulates a scalar field using the given sample function and isovalue.
   * @param center The center of the field.
   * @param sampleFunction A function that returns scalar field values at given sample points.
   * @param isovalue The threshold value used for the isosurface extraction.
   * @param cellSize The size of each cell in the field.
   * @param testingCells A list of testing cells to be considered for triangulation.
   * @param vertices The output vector storing the generated vertices.
   * @param indices The output vector storing the indices of the generated triangles.
   * @param removeDuplicate A flag indicating whether duplicate vertices should be removed.
   */
  static void TriangulateField(const glm::vec3& center,
                               const std::function<float(const glm::vec3& samplePoint)>& sampleFunction, float isovalue,
                               float cellSize, const std::vector<TestingCell>& testingCells,
                               std::vector<Vertex>& vertices, std::vector<unsigned>& indices, bool removeDuplicate);
};

}  // namespace eco_sys_lab_plugin
