#include "BasicPruningDescriptor.hpp"
#include "EcoSysLabDescriptorInspectors.hpp"
#include "EcoSysLabSerializationAdapters.hpp"
#include "EditorLayer.hpp"

using namespace eco_sys_lab_package;

bool eco_sys_lab_package::InspectBasicPruningDescriptor(InspectorContext& context, BasicPruningDescriptor& target) {
  const auto& editor_layer = context.editor_layer;
  bool changed = false;
  if (ImGui::TreeNodeEx("Pruning", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (ImGui::DragFloat("Low Branch Pruning", &target.low_branch_pruning, 0.01f, 0.0f, 1.f))
      changed = true;
    changed = ImGui::Checkbox("Trunk Protection", &target.trunk_protection) || changed;
    changed = ImGui::DragInt("Max chain length", &target.max_flow_length, 1) || changed;
    changed = ImGui::DragFloat("Light pruning threshold", &target.light_pruning_factor, 0.01f) || changed;

    changed = ImGui::DragFloat("Branch strength", &target.branch_strength, 0.01f, 0.0f) || changed;
    changed =
        ImGui::DragFloat("Branch strength thickness factor", &target.branch_strength_thickness_factor, 0.01f, 0.0f) ||
        changed;
    changed = ImGui::DragFloat("Branch strength lighting threshold", &target.branch_strength_lighting_threshold, 0.01f,
                               0.0f, 1.0f) ||
              changed;
    changed =
        ImGui::DragFloat("Branch strength lighting loss", &target.branch_strength_lighting_loss, 0.01f, 0.0f, 1.0f) ||
        changed;
    changed = ImGui::DragFloat("Branch breaking multiplier", &target.branch_breaking_multiplier, 0.01f, 0.01f, 10.0f) ||
              changed;

    changed =
        ImGui::DragFloat("Branch breaking factor", &target.branch_breaking_factor, 0.01f, 0.01f, 10.0f) || changed;

    ImGui::TreePop();
  }

  return changed;
}
