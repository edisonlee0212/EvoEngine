#include "Application.hpp"
#include "Delaunay.hpp"
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

bool eco_sys_lab_package::InspectSettings(SegmentPairsRenderParameters& target,
                                          const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::DragFloat3("Position scale", &target.position_scale.x, 0.1f, 0.1f, 100.f)) {
    changed = true;
  }
  if (ImGui::DragFloat("Thickness multiplier", &target.thickness_multiplier, 0.1f, 0.1f, 10.f)) {
    changed = true;
  }

  return changed;
}
