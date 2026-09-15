#include "CurveEditors.hpp"
#include "EcoSysLabSettingsEditor.hpp"
#include "EditorLayer.hpp"
#include "EditorWidgets.hpp"
#include "StrandModelParameters.hpp"

using namespace eco_sys_lab_package;

bool eco_sys_lab_package::InspectSettings(StrandModelParameters& target,
                                          const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::TreeNodeEx("Profile settings", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (ImGui::Button("Oak Trunk")) {
      target.end_node_strands = 3200;
      target.strand_radius_distribution.mean.max_value = 0.003f;
      target.strand_radius_distribution.mean.curve = Curve2D(1.0f, 0.8f, {0, 0}, {1, 1});
    }
    if (ImGui::TreeNode("Physics settings")) {
      if (ImGui::DragFloat("Physics damping", &target.profile_physics_settings.damping, 0.01f, 0.0f, 1.0f))
        changed = true;
      if (ImGui::DragFloat("Physics max speed", &target.profile_physics_settings.max_speed, 0.01f, 0.0f, 100.0f))
        changed = true;
      if (ImGui::DragFloat("Physics particle softness", &target.profile_physics_settings.particle_softness, 0.01f, 0.0f,
                           1.0f))
        changed = true;
      ImGui::TreePop();
    }
    if (ImGui::DragFloat("Center attraction strength", &target.center_attraction_strength, 100.f, 0.0f, 10000.0f))
      changed = true;
    if (ImGui::DragInt("Max iteration cell factor", &target.max_simulation_iteration_cell_factor, 1, 0, 500))
      changed = true;
    if (ImGui::DragInt("Branch Packing Timeout", &target.branch_profile_packing_max_iteration, 1, 0, 10000))
      changed = true;
    if (ImGui::DragInt("Junction Packing Timeout", &target.junction_profile_packing_max_iteration, 1, 20, 10000))
      changed = true;
    if (ImGui::DragInt("Modified Packing Timeout", &target.modified_profile_packing_max_iteration, 1, 20, 10000))
      changed = true;
    if (ImGui::DragInt("Timeout with boundaries)", &target.modified_profile_packing_max_iteration, 1, 20, 10000))
      changed = true;
    ImGui::TreePop();
  }
  if (ImGui::DragFloat("Overlap threshold", &target.overlap_threshold, 0.01f, 0.0f, 1.0f))
    changed = true;
  if (ImGui::DragInt("Initial branch strand count", &target.strands_along_branch, 1, 0, 50))
    changed = true;
  if (ImGui::DragInt("Initial end node strand count", &target.end_node_strands, 20, 1, 3200))
    changed = true;

  if (ImGui::Checkbox("Pre-merge", &target.pre_merge))
    changed = true;

  static PlottedDistributionSettings plotted_distribution_settings = {
      0.001f, {0.001f, true, true, ""}, {0.001f, true, true, ""}, ""};
  if (editor_widgets::Draw(target.branch_twist_distribution, "Branch Twist", plotted_distribution_settings))
    changed = true;
  if (editor_widgets::Draw(target.junction_twist_distribution, "Junction Twist", plotted_distribution_settings))
    changed = true;
  if (editor_widgets::Draw(target.strand_radius_distribution, "Strand Thickness", plotted_distribution_settings))
    changed = true;

  if (ImGui::DragFloat("Cladoptosis Range", &target.cladoptosis_range, 0.01f, 0.0f, 50.f))
    changed = true;
  if (editor_widgets::Draw(target.cladoptosis_distribution, "Cladoptosis", plotted_distribution_settings))
    changed = true;

  if (ImGui::TreeNodeEx("Graph Adjustment settings", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (ImGui::DragFloat("Side factor", &target.side_push_factor, 0.01f, 0.0f, 2.0f))
      changed = true;
    if (ImGui::DragFloat("Apical Side factor", &target.apical_side_push_factor, 0.01f, 0.0f, 2.0f))
      changed = true;
    if (ImGui::DragFloat("Rotation factor", &target.rotation_push_factor, 0.01f, 0.0f, 2.0f))
      changed = true;
    if (ImGui::DragFloat("Apical Rotation factor", &target.apical_branch_rotation_push_factor, 0.01f, 0.0f, 2.0f))
      changed = true;
    ImGui::TreePop();
  }
  if (ImGui::DragInt("Max node count", &target.node_max_count, 1, -1, 999))
    changed = true;
  if (ImGui::DragInt("Boundary point distance", &target.boundary_point_distance, 1, 3, 30))
    changed = true;
  if (ImGui::ColorEdit4("Boundary color", &target.boundary_point_color.x))
    changed = true;
  if (ImGui::ColorEdit4("Content color", &target.content_point_color.x))
    changed = true;
  return changed;
}
