#include "EcoSysLabSettingsEditor.hpp"
#include "EditorLayer.hpp"
#include "EditorWidgets.hpp"
#include "SkeletalGraphSettings.hpp"

using namespace eco_sys_lab_package;

bool eco_sys_lab_package::InspectSettings(SkeletalGraphSettings& target,
                                          const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  ImGui::DragFloat("Line thickness", &target.line_thickness, 0.001f, 0.0f, 1.0f);
  ImGui::DragFloat("Fixed line thickness", &target.fixed_line_thickness, 0.001f, 0.0f, 1.0f);
  ImGui::DragFloat("Branch point size", &target.branch_point_size, 0.01f, 0.0f, 1.0f);
  ImGui::DragFloat("Junction point size", &target.junction_point_size, 0.01f, 0.0f, 1.0f);

  ImGui::Checkbox("Fixed point size", &target.fixed_point_size);
  if (target.fixed_point_size) {
    ImGui::DragFloat("Fixed point size multiplier", &target.fixed_point_size_factor, 0.001f, 0.0f, 1.0f);
  }

  ImGui::ColorEdit4("Line color", &target.line_color.x);
  ImGui::ColorEdit4("Branch point color", &target.branch_point_color.x);
  ImGui::ColorEdit4("Junction point color", &target.junction_point_color.x);

  return changed;
}
