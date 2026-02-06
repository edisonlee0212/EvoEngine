#include "VoronoiMeshGenerator.hpp"

using namespace evo_engine;

namespace eco_sys_lab_plugin {

void VoronoiMeshGenerator::Generate(const StrandModel& strand_model, std::vector<Vertex>& vertices,
                                    std::vector<unsigned int>& indices,
                                    const StrandModelMeshGeneratorSettings& settings) {
  // Implementation of the Voronoi mesh generation algorithm
  // just use the index pair version and discard the tex coords
  std::vector<glm::vec2> tex_coords;
  std::vector<std::pair<unsigned int, unsigned int>> index_pairs;
  Generate(strand_model, vertices, tex_coords, index_pairs, settings);

  for (const auto& pair : index_pairs) {
    indices.emplace_back(pair.first);
  }
}

void VoronoiMeshGenerator::Generate(const StrandModel& strand_model, std::vector<Vertex>& vertices,
                                    std::vector<glm::vec2>& tex_coords,
                                    std::vector<std::pair<unsigned int, unsigned int>>& index_pairs,
                                    const StrandModelMeshGeneratorSettings& settings) {
}

void VoronoiMeshGenerator::Generate(const DtsStrandGroup& randomly_subdivided_strands, const StrandModel& strand_model,
                                    std::vector<Vertex>& vertices, std::vector<glm::vec2>& tex_coords,
                                    std::vector<std::pair<unsigned int, unsigned int>>& index_pairs,
                                    const StrandModelMeshGeneratorSettings& settings) {
}
}  // namespace eco_sys_lab_plugin