#pragma once

#include "InspectorRegistry.hpp"

namespace mesh_repair_package {
class MeshColoring;

bool InspectMeshColoring(evo_engine::InspectorContext& context, MeshColoring& mesh_coloring);
}  // namespace mesh_repair_package
