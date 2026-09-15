#include "BasicRootDescriptor.hpp"
#include "EcoSysLabDescriptorInspectors.hpp"
#include "EcoSysLabSerializationAdapters.hpp"
#include "EditorLayer.hpp"
#include "SDKInspectionAdapters.hpp"
#include "ShootModel.hpp"

using namespace eco_sys_lab_package;

bool eco_sys_lab_package::InspectBasicRootDescriptor(InspectorContext& context, BasicRootDescriptor& target,
                                                     GrowthDescriptorInspectorState& state) {
  const auto& editor_layer = context.editor_layer;
  bool changed = false;
  changed = ImGui::DragFloat("Growth rate", &target.growth_rate, 0.01f, 0.0f, 10.0f) || changed;
  changed = ImGui::DragFloat("Straight Tap Root", &target.straight_tap_root, 0.1f, 0.0f, 100.f) || changed;
  if (ImGui::TreeNodeEx("Root node", ImGuiTreeNodeFlags_DefaultOpen)) {
    changed = ImGui::DragInt("Base node count", &target.base_root_node_count, 1, 0, 3) || changed;
    changed =
        ImGui::DragFloat("Lateral node flushing prob", &target.lateral_node_flushing_probability, 0.01f, 0.01f, 1.0f) ||
        changed;
    ImGui::Checkbox("Show branching angle graph", &state.show_branching_angle_graph);
    ImGui::Checkbox("Show roll angle graph", &state.show_roll_angle_graph);
    ImGui::Checkbox("Show apical angle graph", &state.show_apical_angle_graph);
    if (state.show_branching_angle_graph) {
      changed =
          evo_engine::DrawProceduralNoiseGraph(target.branching_angle_graph, "Branching Angle Graph", editor_layer) ||
          changed;
    }
    if (state.show_roll_angle_graph) {
      changed =
          evo_engine::DrawProceduralNoiseGraph(target.roll_angle_graph, "Roll Angle Graph", editor_layer) || changed;
    }
    if (state.show_apical_angle_graph) {
      changed = evo_engine::DrawProceduralNoiseGraph(target.apical_angle_graph, "Apical Angle Graph", editor_layer) ||
                changed;
    }

    changed = ImGui::DragFloat("Root node length", &target.root_node_length, 0.001f) || changed;
    changed = ImGui::DragFloat("Root node length thickness factor", &target.root_node_length_thickness_factor, 0.0001f,
                               0.0f, 1.0f) ||
              changed;
    changed = ImGui::DragFloat3("Thickness min/factor/age", &target.end_node_thickness, 0.0001f, 0.0f, 1.0f, "%.6f") ||
              changed;
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Root Shape Control", ImGuiTreeNodeFlags_DefaultOpen)) {
    changed = ImGui::DragFloat("Apical control", &target.apical_control, 0.01f) || changed;
    changed = ImGui::DragFloat2("Inhibitor val/loss", &target.apical_dominance, 0.01f) || changed;
    changed = ImGui::DragFloat("Root distance control", &target.root_distance_control, 0.01f) || changed;
    changed = ImGui::DragFloat2("Soil Friction/Speed", &target.soil_density_friction.x, 0.01f) || changed;
    changed = ImGui::DragFloat("Tropism intensity", &target.tropism_intensity, 0.01f) || changed;
    changed = ImGui::DragFloat2("Tropism switch prob/dist", &target.tropism_switch_probability, 0.01f) || changed;
    ImGui::TreePop();
  }

  return changed;
}
