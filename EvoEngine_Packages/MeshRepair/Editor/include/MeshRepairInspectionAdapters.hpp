#pragma once

#include "AssetRef.hpp"
#include "InspectorRegistry.hpp"
#include "PrivateComponentRef.hpp"

namespace mesh_repair_package {
class MeshColoring;

struct MeshRepairInspectorState {
  evo_engine::AssetRef visibility_test_prefab_ref;
  evo_engine::AssetRef visibility_test_mesh_ref;
  std::vector<float> triangle_errors{};
  evo_engine::AssetRef triangle_error_prefab_ref;
  float error_threshold = 0.0f;
  evo_engine::PrivateComponentRef mesh_entity_ref;
};

bool InspectMeshColoring(evo_engine::InspectorContext& context, MeshColoring& mesh_coloring,
                         MeshRepairInspectorState& state);
}  // namespace mesh_repair_package
