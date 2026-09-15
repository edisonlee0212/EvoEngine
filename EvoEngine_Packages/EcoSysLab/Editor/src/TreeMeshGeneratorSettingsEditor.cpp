#include "EcoSysLabSettingsEditor.hpp"
#include "EditorLayer.hpp"
#include "EditorWidgets.hpp"
#include "Tree.hpp"
#include "TreeMeshGenerator.hpp"

using namespace eco_sys_lab_package;

void eco_sys_lab_package::InspectSettings(TreeMeshGeneratorSettings& target,
                                          const std::shared_ptr<EditorLayer>& editor_layer) {
  if (ImGui::TreeNodeEx("Mesh Generator settings")) {
    ImGui::Checkbox("Shoot Branch", &target.enable_shoot_branch);
    ImGui::Checkbox("Root Branch", &target.enable_root_branch);
    ImGui::Checkbox("Fruit", &target.enable_fruit);
    ImGui::Checkbox("Foliage", &target.enable_foliage);
    ImGui::Checkbox("Fine root", &target.enable_fine_root);
    ImGui::Checkbox("Foliage instancing", &target.foliage_instancing);
    ImGui::Combo("Branch mesh mode", {"Cylindrical", "Marching cubes"}, target.branch_mesh_type);

    ImGui::Combo("Branch color mode", {"Internode Color", "Junction"}, target.vertex_color_mode);

    if (ImGui::TreeNode("Cylindrical mesh settings")) {
      ImGui::Checkbox("Stitch all children", &target.stitch_all_children);
      ImGui::DragFloat("Trunk Thickness Threshold", &target.trunk_thickness, 1.0f, 0.0f, 16.0f);
      ImGui::DragFloat("X Step", &target.x_subdivision, 0.00001f, 0.00001f, 1.0f, "%.5f");
      ImGui::DragFloat("Trunk Y Step", &target.trunk_y_subdivision, 0.00001f, 0.00001f, 1.0f, "%.5f");
      ImGui::DragFloat("Branch Y Step", &target.branch_y_subdivision, 0.00001f, 0.00001f, 1.0f, "%.5f");

      ImGui::Checkbox("Smoothness", &target.smoothness);
      if (target.smoothness) {
        ImGui::DragFloat("Base control point ratio", &target.base_control_point_ratio, 0.001f, 0.0f, 1.0f);
        ImGui::DragFloat("Branch control point ratio", &target.branch_control_point_ratio, 0.001f, 0.0f, 1.0f);
      }
      ImGui::Checkbox("Override radius", &target.override_radius);
      if (target.override_radius)
        ImGui::DragFloat("Radius", &target.radius);
      ImGui::DragFloat("Radius multiplier", &target.radius_multiplier, 0.01f, 0.01f, 100.f);
      ImGui::DragFloat("Tree Part Base Distance", &target.tree_part_base_distance, 1, 0, 10);
      ImGui::DragFloat("Tree Part End Distance", &target.tree_part_end_distance, 1, 0, 10);
      ImGui::TreePop();
    }
    if (ImGui::TreeNode("Marching cubes settings")) {
      ImGui::Checkbox("Auto set level", &target.auto_level);
      if (!target.auto_level)
        ImGui::DragInt("Voxel subdivision level", &target.voxel_subdivision_level, 1, 5, 16);
      else
        ImGui::DragFloat("Min Cube size", &target.marching_cube_radius, 0.0001, 0.001f, 1.0f);
      ImGui::DragInt("Smooth iteration", &target.voxel_smooth_iteration, 0, 0, 10);
      if (target.voxel_smooth_iteration == 0)
        ImGui::Checkbox("Remove duplicate", &target.remove_duplicate);
      ImGui::TreePop();
    }
    if (target.enable_shoot_branch && ImGui::TreeNode("Branch settings")) {
      ImGui::TreePop();
    }
    if (target.enable_foliage && ImGui::TreeNode("Foliage settings")) {
      ImGui::TreePop();
    }

    ImGui::Checkbox("Mesh Override", &target.presentation_override);
    if (target.presentation_override && ImGui::TreeNodeEx("Override settings")) {
      ImGui::DragFloat("Max thickness", &target.presentation_override_settings.max_thickness, 0.01f, 0.0f, 1.0f);

      ImGui::TreePop();
    }
    ImGui::TreePop();
  }
}
