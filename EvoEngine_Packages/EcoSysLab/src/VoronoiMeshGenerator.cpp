#include "VoronoiMeshGenerator.hpp"

using namespace evo_engine;

namespace eco_sys_lab_package {
void VoronoiMeshGenerator::Generate(const StrandModel&, std::vector<Vertex>&, std::vector<unsigned int>&,
                                    const StrandModelMeshGeneratorSettings&) {
  EVOENGINE_WARNING("VoronoiMeshGenerator is retired. Use DynamicStrands kinetic meshing.");
}

void VoronoiMeshGenerator::Generate(const StrandModel&, std::vector<Vertex>&, std::vector<glm::vec2>&,
                                    std::vector<std::pair<unsigned int, unsigned int>>&,
                                    const StrandModelMeshGeneratorSettings&) {
  EVOENGINE_WARNING("VoronoiMeshGenerator is retired. Use DynamicStrands kinetic meshing.");
}

void VoronoiMeshGenerator::Generate(const DtsStrandGroup&, const StrandModel&, std::vector<Vertex>&,
                                    std::vector<glm::vec2>&, std::vector<std::pair<unsigned int, unsigned int>>&,
                                    const StrandModelMeshGeneratorSettings&) {
  EVOENGINE_WARNING("VoronoiMeshGenerator is retired. Use DynamicStrands kinetic meshing.");
}
}  // namespace eco_sys_lab_package
