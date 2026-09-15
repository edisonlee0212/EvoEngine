#include "BasicFoliageDescriptor.hpp"
#include "CurveEditors.hpp"
#include "DynamicStrandsInitializationParameters.hpp"
#include "EcoSysLabSettingsEditor.hpp"
#include "EditorLayer.hpp"
#include "EditorWidgets.hpp"
#include "SDKInspectionAdapters.hpp"

using namespace eco_sys_lab_package;

bool eco_sys_lab_package::InspectSettings(BundleSolverSettings& target) {
  bool changed = false;
  auto mode_index = static_cast<unsigned>(target.mode);
  if (ImGui::Combo("Mode (reinitialize)", {"Legacy", "Coupled XPBD", "Hybrid"}, mode_index)) {
    target.mode = static_cast<BundleSolverMode>(mode_index);
    changed = true;
  }
  changed = ImGui::DragInt("Legacy iterations", &target.legacy_iterations, 1, 1, 100) || changed;
  changed = ImGui::DragInt("Pair iterations", &target.pair_iterations, 1, 1, 100) || changed;
  changed = ImGui::DragInt("Coarse iterations", &target.coarse_iterations, 1, 1, 100) || changed;
  changed =
      ImGui::DragFloat("Position compliance scale", &target.position_compliance_scale, 0.01f, 0.f, 100.f) || changed;
  changed =
      ImGui::DragFloat("Bending compliance scale", &target.bending_compliance_scale, 0.01f, 0.f, 100.f) || changed;
  changed =
      ImGui::DragFloat("Torsion compliance scale", &target.torsion_compliance_scale, 0.01f, 0.f, 100.f) || changed;
  changed = ImGui::SliderFloat("Shape matching strength", &target.shape_matching_strength, 0.f, 1.f) || changed;
  changed = ImGui::DragFloat("Slice spacing factor (reinitialize)", &target.slice_spacing_factor, 0.05f, 0.1f, 10.f) ||
            changed;
  changed =
      ImGui::DragInt("Minimum slice members (reinitialize)", &target.minimum_slice_members, 1, 1, 1024) || changed;
  return changed;
}
