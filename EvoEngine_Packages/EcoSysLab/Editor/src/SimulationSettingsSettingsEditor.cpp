#include "EcoSysLabSettingsEditor.hpp"
#include "EditorLayer.hpp"
#include "EditorWidgets.hpp"
#include "SimulationSettings.hpp"

using namespace eco_sys_lab_package;

bool eco_sys_lab_package::InspectSettings(SimulationSettings& target,
                                          const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::DragInt("Max node count", &target.max_node_count, 500, 0, INT_MAX)) {
    changed = true;
  }
  if (ImGui::DragInt("Max flow count", &target.max_flow_count, 500, 0, INT_MAX)) {
    changed = true;
  }
  if (ImGui::Button("Grow daily")) {
    target.delta_time = 1.f;
    changed = true;
  }
  ImGui::SameLine();
  if (ImGui::Button("Grow weekly")) {
    target.delta_time = 7.f;
    changed = true;
  }
  ImGui::SameLine();
  if (ImGui::Button("Grow monthly")) {
    target.delta_time = 30.f;
    changed = true;
  }

  if (ImGui::DragFloat("Delta time", &target.delta_time, 0.1f, 0, 30, "%.1f"))
    changed = true;
  if (ImGui::Checkbox("Auto clear fruit and leaves", &target.auto_clear_fruit_and_leaves))
    changed = true;
  if (ImGui::DragFloat("Crown shyness", &target.crown_shyness_distance, 0.01f, 0.0f, 1.0f))
    changed = true;
  if (ImGui::Checkbox("Simulate soil", &target.soil_simulation))
    changed = true;
  if (ImGui::TreeNode("Lighting Estimation Settings")) {
    changed = ImGui::DragFloat("Skylight Intensity", &target.skylight_intensity, 0.01f, 0.0f, 10.0f) || changed;
    changed =
        ImGui::DragFloat("Environmental Intensity", &target.environment_light_intensity, 0.01f, 0.0f, 10.0f) || changed;
    changed = ImGui::DragFloat("Shadow distance loss", &target.shadow_distance_loss, 0.01f, 0.0f, 10.0f) || changed;
    changed = ImGui::DragFloat("Detection radius", &target.detection_radius, 0.001f, 0.0f, 1.0f) || changed;
    changed = ImGui::DragInt("Blur iteration", &target.blur_iteration, 1, 0, 10) || changed;

    ImGui::TreePop();
  }
  return changed;
}

bool eco_sys_lab_package::InspectSettings(SimulationStats& target, const std::shared_ptr<EditorLayer>& editor_layer) {
  if (ImGui::TreeNodeEx("Stats")) {
    ImGui::Text("Growth time: %.4f", target.last_used_time);
    ImGui::Text("Total time: %.4f", target.total_time);
    ImGui::Text("Total internode size: %d", target.internode_size);
    ImGui::Text("Total shoot branch size: %d", target.shoot_stem_size);
    ImGui::Text("Total fruit size: %d", target.fruit_size);
    ImGui::Text("Total leaf size: %d", target.leaf_size);
    ImGui::Text("Total root node size: %d", target.root_node_size);
    ImGui::Text("Total root branch size: %d", target.root_stem_size);

    ImGui::TreePop();
  }
  return false;
}
