#pragma once

#include "StrandModelMeshGenerator.hpp"
#include "StrandModel.hpp"
#include "StrandModelData.hpp"
#include "Vertex.hpp"

// TODO: should this inherit from StrandModelMeshGenerator?
using namespace evo_engine;
namespace eco_sys_lab_plugin {
class AlphaShapeMeshGenerator {
public:
  static void Generate(const StrandModel& strand_model, std::vector<Vertex>& vertices, std::vector<unsigned int>& indices,
                       const StrandModelMeshGeneratorSettings& settings);
  static void Generate(const StrandModel& strand_model, std::vector<Vertex>& vertices, std::vector<glm::vec2>& tex_coords,
                       std::vector<std::pair<unsigned int, unsigned int>>& index_pairs,
                       const StrandModelMeshGeneratorSettings& settings);

private:
  static void ComputeAlphaShape(std::vector<glm::vec3> points, std::vector<Vertex>& vertices,
                                std::vector<unsigned int>& indices, double alpha);

};
}  // namespace eco_sys_lab_plugin