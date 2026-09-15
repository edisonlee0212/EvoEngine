#include "Application.hpp"
#include "Delaunay.hpp"
#include "DsAlphaShapeMeshing.hpp"
#include "DsConstraints.hpp"
#include "DsOperators.hpp"
#include "DynamicTreeStrands.hpp"
#include "EcoSysLabSettingsEditor.hpp"
#include "EditorLayer.hpp"
#include "EditorWidgets.hpp"
#include "RenderParameters.hpp"
#include "Shader.hpp"
#include "Tree.hpp"

using namespace eco_sys_lab_package;

bool eco_sys_lab_package::InspectSettings(SmallSegmentsRenderParameters& target,
                                          const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::Checkbox("Cast Shadow", &target.cast_shadow)) {
    changed = true;
  }
  if (ImGui::DragFloat3("Position scale", &target.position_scale.x, 0.1f, 0.1f, 100.f)) {
    changed = true;
  }
  if (ImGui::Checkbox("Wireframe", &target.wireframe)) {
    changed = true;
  }
  if (ImGui::DragFloat("Thickness multiplier", &target.thickness_multiplier, 0.1f, 0.1f, 10.f)) {
    changed = true;
  }

  return changed;
}

bool eco_sys_lab_package::InspectSettings(SmallSegmentsVisualizationRenderParameters& target,
                                          const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::DragFloat("Thickness multiplier", &target.thickness_multiplier, 0.1f, 0.1f, 10.f)) {
    changed = true;
  }

  if (ImGui::DragFloat3("Position scale", &target.position_scale.x, 0.1f, 0.1f, 100.f)) {
    changed = true;
  }

  if (ImGui::Combo("Segment mode",
                   {"Default", "Node color", "Group index", "Boundary distance", "Strength", "Shear/Strain strain",
                    "Shear/Stretch limit", "Segment color", "Strand color"},
                   target.segment_render_mode))
    changed = true;
  switch (target.segment_render_mode) {
    case 0: {
      if (ImGui::ColorEdit4("Segment color", &target.segment_color_main.x))
        changed = true;
      break;
    }
    case 3: {
      if (ImGui::ColorEdit4("Segment min color", &target.segment_color_min.x))
        changed = true;
      if (ImGui::ColorEdit4("Segment max color", &target.segment_color_max.x))
        changed = true;
      if (ImGui::DragFloat("Segment boundary distance modular", &target.segment_boundary_distance_modular, 0.001f,
                           0.001f, 1.f))
        changed = true;
      break;
    }
    case 4:
    case 5:
    case 6: {
      if (ImGui::ColorEdit4("Segment min color", &target.segment_color_min.x))
        changed = true;
      if (ImGui::ColorEdit4("Segment max color", &target.segment_color_max.x))
        changed = true;
      break;
    }
    default:
      break;
  }

  return changed;
}
