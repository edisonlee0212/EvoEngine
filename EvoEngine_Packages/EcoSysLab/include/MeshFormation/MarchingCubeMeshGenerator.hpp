#pragma once

#include "StrandModel.hpp"
#include "StrandModelData.hpp"
#include "StrandModelMeshGenerator.hpp"
#include "Vertex.hpp"

namespace eco_sys_lab_package {
using namespace evo_engine;

/**
 * @class MarchingCubeMeshGenerator
 * @brief A utility class for generating meshes using the marching cubes algorithm.
 *
 * This class provides static methods for generating 3D mesh representations
 * from strand models. The generated meshes can be used for visualization
 * and simulation within the EcoSysLab package.
 */
class MarchingCubeMeshGenerator {
 public:
  /**
   * @brief Generates a mesh from the given strand model.
   *
   * This method creates a 3D mesh representation of the input strand model,
   * storing the results in the provided vertices and indices vectors.
   *
   * @param strand_model The input strand model to generate the mesh from.
   * @param vertices A reference to the vector that will store the generated vertices.
   * @param indices A reference to the vector that will store the generated indices.
   * @param settings The settings used for mesh generation.
   */
  static void Generate(const StrandModel& strand_model, std::vector<Vertex>& vertices,
                       std::vector<unsigned int>& indices, const StrandModelMeshGeneratorSettings& settings);

  /**
   * @brief Generates a mesh with texture coordinates from the given strand model.
   *
   * In addition to generating vertices and indices, this method also computes texture coordinates
   * and index pairs for further processing.
   *
   * @param strand_model The input strand model to generate the mesh from.
   * @param vertices A reference to the vector that will store the generated vertices.
   * @param tex_coords A reference to the vector that will store the texture coordinates.
   * @param index_pairs A reference to the vector that will store index pairs.
   * @param settings The settings used for mesh generation.
   */
  static void Generate(const StrandModel& strand_model, std::vector<Vertex>& vertices,
                       std::vector<glm::vec2>& tex_coords,
                       std::vector<std::pair<unsigned int, unsigned int>>& index_pairs,
                       const StrandModelMeshGeneratorSettings& settings);

 private:
  /**
   * @brief Converts a line segment into a voxelized representation.
   *
   * This method divides the given line segment into smaller voxel-sized
   * segments based on the specified voxel side length.
   *
   * @param start The starting point of the line segment.
   * @param end The ending point of the line segment.
   * @param voxel_side_length The side length of each voxel.
   * @return A vector of voxel positions represented as integer coordinate triplets.
   */
  static std::vector<glm::ivec3> VoxelizeLineSeg(glm::vec3 start, glm::vec3 end, float voxel_side_length);

  /**
   * @brief Implements the marching cubes algorithm for mesh generation.
   *
   * This method applies the marching cubes algorithm to the strand model, generating
   * a 3D mesh representation based on the given settings.
   *
   * @param strand_model The input strand model.
   * @param vertices A reference to the vector storing the generated vertices.
   * @param indices A reference to the vector storing the generated indices.
   * @param settings The settings used for mesh generation.
   */
  static void MarchingCube(const StrandModel& strand_model, std::vector<Vertex>& vertices,
                           std::vector<unsigned>& indices, const StrandModelMeshGeneratorSettings& settings);
};
}  // namespace eco_sys_lab_package
