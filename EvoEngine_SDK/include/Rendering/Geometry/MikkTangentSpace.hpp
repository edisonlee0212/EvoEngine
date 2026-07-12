#pragma once

#include "Vertex.hpp"

#include <vector>

namespace evo_engine {

void GenerateMikkTangents(std::vector<Vertex>& vertices, std::vector<glm::uvec3>& triangles, int tex_coord);
void GenerateMikkTangents(std::vector<SkinnedVertex>& vertices, std::vector<glm::uvec3>& triangles, int tex_coord);

}  // namespace evo_engine
