#pragma once

#include "StrandModel.hpp"
#include "StrandModelData.hpp"
#include "StrandModelMeshGenerator.hpp"
#include "Vertex.hpp"

// TODO: should this inherit from StrandModelMeshGenerator?
using namespace evo_engine;
namespace eco_sys_lab_plugin {
class MarchingCubeMeshGenerator {
 public:
  static void Generate(const StrandModel& strand_model, std::vector<Vertex>& vertices, std::vector<unsigned int>& indices,
                       const StrandModelMeshGeneratorSettings& settings);
  static void Generate(const StrandModel& strand_model, std::vector<Vertex>& vertices, std::vector<glm::vec2>& tex_coords,
                       std::vector<std::pair<unsigned int, unsigned int>>& index_pairs,
                       const StrandModelMeshGeneratorSettings& settings);

 private:
  static std::vector<glm::ivec3> MarchingCubeMeshGenerator::VoxelizeLineSeg(glm::vec3 start, glm::vec3 end,
                                                                            float voxel_side_length);
  static void MarchingCube(const StrandModel& strand_model, std::vector<Vertex>& vertices,
                           std::vector<unsigned>& indices, const StrandModelMeshGeneratorSettings& settings);
};
}  // namespace eco_sys_lab_plugin