#pragma once

#include "StrandModel.hpp"
#include "StrandModelData.hpp"
#include "StrandModelMeshGenerator.hpp"
#include "Vertex.hpp"

namespace eco_sys_lab_package {
using namespace evo_engine;

/**
 * @class AlphaShapeMeshGenerator
 * @brief A class responsible for generating meshes using the Alpha Shape algorithm.
 */
class AlphaShapeMeshGenerator {
 public:
  /**
   * @brief Generates a mesh representation of the given strand model.
   *
   * @param strand_model The strand model used as input.
   * @param vertices A vector to store the generated vertices.
   * @param indices A vector to store the generated indices.
   * @param settings Mesh generation settings.
   */
  static void Generate(const StrandModel& strand_model, std::vector<Vertex>& vertices,
                       std::vector<unsigned int>& indices, const StrandModelMeshGeneratorSettings& settings);

  /**
   * @brief Generates a mesh representation of the given strand model with texture coordinates and index pairs.
   *
   * @param strand_model The strand model used as input.
   * @param vertices A vector to store the generated vertices.
   * @param tex_coords A vector to store the generated texture coordinates.
   * @param index_pairs A vector to store the generated index pairs.
   * @param settings Mesh generation settings.
   */
  static void Generate(const StrandModel& strand_model, std::vector<Vertex>& vertices,
                       std::vector<glm::vec2>& tex_coords,
                       std::vector<std::pair<unsigned int, unsigned int>>& index_pairs,
                       const StrandModelMeshGeneratorSettings& settings);

 private:
  /**
   * @brief Computes the Alpha Shape of a given set of points.
   *
   * @param points The input points used for the Alpha Shape computation.
   * @param vertices A vector to store the generated vertices.
   * @param indices A vector to store the generated indices.
   * @param alpha The Alpha value used to control shape tightness.
   */
  static void ComputeAlphaShape(std::vector<glm::vec3> points, std::vector<Vertex>& vertices,
                                std::vector<unsigned int>& indices, double alpha);
};
}  // namespace eco_sys_lab_package