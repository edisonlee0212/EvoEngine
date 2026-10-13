#pragma once
#include "DsAlphaShapeMeshing.hpp"
#include "DsKineticVoronoiMeshing.hpp"
#include "InspectorRegistry.hpp"
namespace eco_sys_lab_package {
struct DynamicStrandsMeshingInspector {
  static bool Inspect(evo_engine::InspectorContext& context, DsAlphaShapeMeshing& target);
  static bool Inspect(evo_engine::InspectorContext& context, DsKineticVoronoiMeshing& target);
  static bool Inspect(evo_engine::InspectorContext& context, DsAlphaShapeVisualizationParameters& target);
  static void DrawStats(const DsAlphaShapeMeshing& target);
  static void DrawStats(const DsKineticVoronoiMeshing& target);
  static void DrawStats(const DsMeshing& target);
  /// @param lock_primary_enable When true, the primary mesh enable checkbox is read-only (driven by the
  ///        Active mesh visualization radio when both Kinetic and Alpha backends are present).
  static void DrawDsAlphaShapeMeshingSettings(const std::shared_ptr<evo_engine::EditorLayer>& editor_layer,
                                              bool lock_primary_enable = false);
  static void DrawDsKineticVoronoiMeshingSettings(const std::shared_ptr<evo_engine::EditorLayer>& editor_layer,
                                                   bool lock_primary_enable = false);
};
}  // namespace eco_sys_lab_package
