#include "TreeAssetInspectors.hpp"
#include "imgui.h"
using namespace eco_sys_lab_package;
using namespace evo_engine;
bool eco_sys_lab_package::InspectLSystemString(InspectorContext& context, LSystemString& target) {
  bool changed = false;
  ImGui::Text(("Command Size: " + std::to_string(target.m_commands.size())).c_str());
  if (ImGui::DragFloat("Internode Length", &target.m_internodeLength))
    changed = true;
  if (ImGui::DragFloat("Thickness Factor", &target.m_thicknessFactor))
    changed = true;
  if (ImGui::DragFloat("End node thickness", &target.m_endNodeThickness))
    changed = true;
  return changed;
}
bool eco_sys_lab_package::InspectTreeGraph(InspectorContext& context, TreeGraph& target) {
  bool changed = false;
  ImGui::Checkbox("Length limit", &target.enable_instantiate_length_limit);
  if (target.enable_instantiate_length_limit)
    ImGui::DragFloat("Length limit", &target.instantiate_length_limit, 0.1f);

  return false;
}
bool eco_sys_lab_package::InspectTreeGraphV2(InspectorContext& context, TreeGraphV2& target) {
  bool changed = false;
  ImGui::Checkbox("Length limit", &target.enable_instantiate_length_limit);
  ImGui::DragFloat("Length limit", &target.instantiate_length_limit, 0.1f);
  return changed;
}