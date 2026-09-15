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
bool eco_sys_lab_package::InspectSettings(DynamicSkeleton::InitializeParameters& target,
                                          const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;

  if (ImGui::TreeNode("Material Properties")) {
    if (editor_widgets::Draw(target.wood_density, "Wood Density"))
      changed = true;
    if (editor_widgets::Draw(target.max_youngs_modulus, "Wood Young's modulus"))
      changed = true;
    if (editor_widgets::Draw(target.max_shear_modulus, "Wood Shear modulus"))
      changed = true;
    if (editor_widgets::Draw(target.max_bending_modulus, "Wood Bending modulus"))
      changed = true;
    if (editor_widgets::Draw(target.max_twisting_modulus, "Wood Torsion modulus"))
      changed = true;
    ImGui::TreePop();
  }

  return changed;
}
bool eco_sys_lab_package::InspectSettings(DynamicSkeleton::PhysicsParameters& target,
                                          const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;

  if (ImGui::DragFloat("Time step", &target.time_step, 0.001f, 0.001f, 1.0f))
    changed = true;
  if (ImGui::DragInt("Sub step", &target.sub_step, 1, 1, 100)) {
    changed = true;
  }
  if (ImGui::Checkbox("Breaking", &target.enable_breaking)) {
    changed = true;
  }
  if (ImGui::Checkbox("Disconnection", &target.enable_disconnection)) {
    changed = true;
  }
  if (ImGui::DragInt("Constraint Iteration", &target.constraint_iteration, 1, 1, 500))
    changed = true;
  if (ImGui::DragFloat("Velocity damping", &target.velocity_damping, 0.01f, 0.01f, 1.0f))
    changed = true;
  if (ImGui::DragFloat("Angular velocity damping", &target.angular_velocity_damping, 0.00001f, 0.0f, 1.0f, "%.5f"))
    changed = true;

  return changed;
}
bool eco_sys_lab_package::InspectSettings(DynamicSkeleton::VisualizationParameters& target,
                                          const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  return changed;
}
