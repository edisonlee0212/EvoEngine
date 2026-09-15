#include "BasicShootDescriptor.hpp"
#include "EcoSysLabDescriptorInspectors.hpp"
#include "EcoSysLabSerializationAdapters.hpp"
#include "EditorLayer.hpp"
#include "SDKInspectionAdapters.hpp"
#include "ShootModel.hpp"

using namespace eco_sys_lab_package;

bool eco_sys_lab_package::InspectBasicShootDescriptor(InspectorContext& context, BasicShootDescriptor& target,
                                                      GrowthDescriptorInspectorState& state) {
  const auto& editor_layer = context.editor_layer;
  bool changed = false;
  changed = ImGui::DragFloat("Growth rate", &target.growth_rate, 0.01f, 0.0f, 10.0f) || changed;
  changed = ImGui::DragFloat("Straight Trunk", &target.straight_trunk, 0.1f, 0.0f, 100.f) || changed;
  if (ImGui::TreeNodeEx("Internode", ImGuiTreeNodeFlags_DefaultOpen)) {
    changed = ImGui::DragInt("Base node count", &target.base_internode_count, 1, 0, 3) || changed;
    changed = ImGui::DragInt("Lateral bud count", &target.lateral_bud_count, 1, 0, 3) || changed;
    changed = ImGui::DragInt("Max Order", &target.max_order, 1, -1, 100) || changed;

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

    changed = ImGui::DragFloat("Internode length", &target.internode_length, 0.001f) || changed;
    changed = ImGui::DragFloat("Internode length thickness factor", &target.internode_length_thickness_factor, 0.0001f,
                               0.0f, 1.0f) ||
              changed;
    changed = ImGui::DragFloat3("Thickness min/factor/age", &target.end_node_thickness, 0.0001f, 0.0f, 1.0f, "%.6f") ||
              changed;

    changed =
        ImGui::DragFloat("Bending strength", &target.gravity_bending_strength, 0.01f, 0.0f, 1.0f, "%.3f") || changed;
    changed = ImGui::DragFloat("Bending thickness factor", &target.gravity_bending_thickness_factor, 0.1f, 0.0f, 10.f,
                               "%.3f") ||
              changed;
    changed =
        ImGui::DragFloat("Bending angle factor", &target.gravity_bending_max, 0.01f, 0.0f, 1.0f, "%.3f") || changed;

    changed =
        ImGui::DragFloat("Internode shadow factor", &target.internode_shadow_factor, 0.001f, 0.0f, 1.0f) || changed;

    ImGui::TreePop();
  }
  if (ImGui::TreeNodeEx("Bud fate", ImGuiTreeNodeFlags_DefaultOpen)) {
    changed = ImGui::DragFloat("Gravitropism", &target.gravitropism, 0.01f) || changed;
    changed = ImGui::DragFloat("Phototropism", &target.phototropism, 0.01f) || changed;
    changed = ImGui::DragFloat("Horizontal Tropism", &target.horizontal_tropism, 0.01f) || changed;

    changed =
        ImGui::DragFloat("Apical bud extinction rate", &target.apical_bud_extinction_rate, 0.01f, 0.0f, 1.0f, "%.5f") ||
        changed;
    changed =
        ImGui::DragFloat("Lateral bud flushing rate", &target.lateral_bud_flushing_rate, 0.01f, 0.0f, 1.0f, "%.5f") ||
        changed;

    changed = ImGui::DragFloat2("Inhibitor val/loss", &target.apical_dominance, 0.01f) || changed;
    ImGui::TreePop();
  }
  if (ImGui::TreeNodeEx("Tree Shape Control", ImGuiTreeNodeFlags_DefaultOpen)) {
    changed = ImGui::DragFloat("Apical control", &target.apical_control, 0.01f) || changed;
    changed = ImGui::DragFloat("Root distance control", &target.root_distance_control, 0.01f) || changed;
    changed = ImGui::DragFloat("Height control", &target.height_control, 0.01f) || changed;

    ImGui::TreePop();
  }

  return changed;
}
