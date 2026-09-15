#include "BasicFoliageDescriptor.hpp"
#include "CurveEditors.hpp"
#include "DynamicsSettingsEditor.hpp"
#include "EcoSysLabGraphEditors.hpp"
#include "EcoSysLabSettingsEditor.hpp"
#include "EditorLayer.hpp"
#include "EditorWidgets.hpp"
#include "SDKInspectionAdapters.hpp"
using namespace evo_engine;
using namespace eco_sys_lab_package;
bool eco_sys_lab_package::InspectSettings(DynamicTreeStrands::BoardExperimentSetupSettings& target,
                                          const std::shared_ptr<EditorLayer>& editor_layer) {
  ImGui::DragFloat("Rod length", &target.segment_length, 0.01f, 0.01f, 10.0f);
  ImGui::DragFloat("Rod radius", &target.radius, 0.001f, 0.001f, 1.0f);
  ImGui::DragInt3("Rod dimension (3D)", &target.rod_dimension.x, 1, 1, 1000);

  ImGui::DragFloat("Center damage", &target.center_damage, 0.01f, 0.01f, 1.0f);
  ImGui::DragFloat("Center damage offset", &target.center_distance_offset, 0.01f, 0.01f, 1.0f);
  ImGui::DragFloat("Center damage transition", &target.center_damage_transition, 0.001f, 0.001f, 1.0f);

  ImGui::Combo("Left Pivot Type", {"Empty", "Point", "Axis", "Transform"}, target.left_pivot_type);
  ImGui::Combo("Right Pivot Type", {"Empty", "Point", "Axis", "Transform"}, target.right_pivot_type);

  ImGui::DragFloat3("Initial velocity", &target.initial_velocity.x, 0.01f, 0.0f, 1.0f);
  ImGui::DragFloat3("Initial angular velocity", &target.initial_angular_velocity.x, 0.01f, 0.0f, 1.0f);
  return false;
}
bool eco_sys_lab_package::InspectSettings(DynamicTreeStrands::LogExperimentSetupSettings& target,
                                          const std::shared_ptr<EditorLayer>& editor_layer) {
  ImGui::DragFloat("Rod length", &target.segment_length, 0.01f, 0.01f, 10.0f);
  ImGui::DragFloat("Rod radius", &target.radius, 0.001f, 0.001f, 1.0f);
  ImGui::DragInt("Rod size", &target.rod_size, 1, 1, 1000);
  ImGui::DragInt("Rod segment size", &target.rod_segment_count, 1, 1, 1000);

  ImGui::DragFloat("Center damage", &target.center_damage, 0.01f, 0.01f, 1.0f);
  ImGui::DragFloat("Center damage offset", &target.center_distance_offset, 0.01f, 0.01f, 1.0f);
  ImGui::DragFloat("Center damage transition", &target.center_damage_transition, 0.001f, 0.001f, 1.0f);

  ImGui::Combo("Left Pivot Type", {"Empty", "Point", "Axis", "Transform"}, target.left_pivot_type);
  ImGui::Combo("Right Pivot Type", {"Empty", "Point", "Axis", "Transform"}, target.right_pivot_type);

  ImGui::DragFloat3("Initial velocity", &target.initial_velocity.x, 0.01f, 0.0f, 1.0f);
  ImGui::DragFloat3("Initial angular velocity", &target.initial_angular_velocity.x, 0.01f, 0.0f, 1.0f);
  return false;
}
