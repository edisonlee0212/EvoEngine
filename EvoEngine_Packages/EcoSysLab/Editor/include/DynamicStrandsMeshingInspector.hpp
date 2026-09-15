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
  static void DrawDsAlphaShapeMeshingSettings(const std::shared_ptr<evo_engine::EditorLayer>& editor_layer);
  static void DrawDsKineticVoronoiMeshingSettings(const std::shared_ptr<evo_engine::EditorLayer>& editor_layer);
};
}  // namespace eco_sys_lab_package
