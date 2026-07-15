#pragma once

#include "Vertex.hpp"

#include <vector>

namespace evo_engine {

void GenerateMikkTangents(std::vector<Vertex>& vertices, std::vector<glm::uvec3>& triangles, int tex_coord,
                          std::vector<uint32_t>* source_vertex_indices = nullptr);
void GenerateMikkTangents(std::vector<SkinnedVertex>& vertices, std::vector<glm::uvec3>& triangles, int tex_coord,
                          std::vector<uint32_t>* source_vertex_indices = nullptr);

}  // namespace evo_engine
