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

bool eco_sys_lab_package::InspectSettings(FoliageRenderParameters& target,
                                          const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;

  if (ImGui::Checkbox("Wireframe", &target.wireframe)) {
    changed = true;
  }
  return changed;
}
