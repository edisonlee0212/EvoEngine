
#pragma once

#include "StrandModel.hpp"
#include "StrandModelData.hpp"
#include "StrandModelMeshGenerator.hpp"
#include "Vertex.hpp"

namespace eco_sys_lab_plugin {
using namespace evo_engine;

/**
 * @class IterativeSlicingMeshGenerator
 * @brief A class responsible for generating mesh representations of strand models
 *        using an iterative slicing approach.
 */
class IterativeSlicingMeshGenerator {
 public:
  /**
   * @brief Generates mesh data from a given strand model.
   *
   * @param strand_model The strand model to generate the mesh from.
   * @param vertices A vector to store the generated mesh vertices.
   * @param indices A vector to store the generated mesh indices.
   * @param settings The settings that define the mesh generation parameters.
   */
  static void Generate(const StrandModel& strand_model, std::vector<Vertex>& vertices,
                       std::vector<unsigned int>& indices, const StrandModelMeshGeneratorSettings& settings);

  /**
   * @brief Generates mesh data from a given strand model, including texture coordinates.
   *
   * @param strand_model The strand model to generate the mesh from.
   * @param vertices A vector to store the generated mesh vertices.
   * @param tex_coords A vector to store the generated texture coordinates.
   * @param index_pairs A vector to store index pairs that represent mesh connectivity.
   * @param settings The settings that define the mesh generation parameters.
   */
  static void Generate(const StrandModel& strand_model, std::vector<Vertex>& vertices,
                       std::vector<glm::vec2>& tex_coords,
                       std::vector<std::pair<unsigned int, unsigned int>>& index_pairs,
                       const StrandModelMeshGeneratorSettings& settings);
};

}  // namespace eco_sys_lab_plugin
