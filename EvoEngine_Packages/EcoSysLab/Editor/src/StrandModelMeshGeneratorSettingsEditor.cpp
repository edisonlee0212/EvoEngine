#include <glm/gtx/intersect.hpp>
#include <glm/gtx/io.hpp>
#include <queue>
#include "AlphaShapeMeshGenerator.hpp"
#include "EcoSysLabLayer.hpp"
#include "EcoSysLabSettingsEditor.hpp"
#include "EditorLayer.hpp"
#include "EditorWidgets.hpp"
#include "IterativeSlicingMeshGenerator.hpp"
#include "Jobs.hpp"
#include "MarchingCubeMeshGenerator.hpp"
#include "MeshGenUtils.hpp"
#include "Octree.hpp"
#include "StrandModelMeshGenerator.hpp"
#include "TreeMeshGenerator.hpp"
#include "VoronoiMeshGenerator.hpp"

using namespace eco_sys_lab_package;

void eco_sys_lab_package::InspectSettings(StrandModelMeshGeneratorSettings& target,
                                          const std::shared_ptr<EditorLayer>& editor_layer) {
  ImGui::Combo("Mode", {"Iterative Slicing", "Marching Cube", "Alpha Shape", "Kinetic Voronoi"}, target.generator_type);
  if (target.generator_type == 0 && ImGui::TreeNode("Iterative Slicing settings")) {
    ImGui::DragInt("Steps per segment", &target.steps_per_segment, 1.0f, 1, 99);

    // ImGui::Checkbox("[DEBUG] Limit Profile Iterations", &m_limitProfileIterations);
    // ImGui::DragInt("[DEBUG] Limit", &m_maxProfileIterations);

    ImGui::DragFloat("[DEBUG] MaxParam", &target.max_param);
    // ImGui::Checkbox("Compute branch joints", &branch_connections);
    ImGui::DragInt("uCoord multiplier", &target.u_multiplier, 1, 1);
    ImGui::DragFloat("vCoord multiplier", &target.v_multiplier, 0.1f);
    ImGui::DragFloat("cluster distance factor", &target.cluster_distance, 0.1f, 1.0f, 10.0f);
    ImGui::TreePop();
  }

  if (target.generator_type == 1 && ImGui::TreeNode("Marching Cube settings")) {
    ImGui::Checkbox("Auto set level", &target.auto_level);
    if (!target.auto_level)
      ImGui::DragInt("Voxel subdivision level", &target.voxel_subdivision_level, 1, 5, 16);
    else
      ImGui::DragFloat("Min Cube size", &target.marching_cube_radius, 0.0001f, 0.001f, 1.0f);
    if (target.smooth_iteration == 0)
      ImGui::Checkbox("Remove duplicate", &target.remove_duplicate);
    ImGui::ColorEdit4("Marching cube color", &target.marching_cube_color.x);
    ImGui::ColorEdit4("Cylindrical color", &target.cylindrical_color.x);
    ImGui::DragInt("uCoord multiplier", &target.root_distance_multiplier, 1, 1, 100);
    ImGui::DragFloat("vCoord multiplier", &target.circle_multiplier, 0.1f);
    ImGui::TreePop();
  }

  if (target.generator_type == 2 && ImGui::TreeNode("Alpha Shape settings")) {
    ImGui::DragInt("Steps per segment", &target.steps_per_segment, 1.0f, 1, 99);
    ImGui::DragFloat("alpha factor", &target.cluster_distance, 0.1f, 1.0f, 10.0f);
    ImGui::DragInt("uCoord multiplier", &target.u_multiplier, 1, 1);
    ImGui::DragFloat("vCoord multiplier", &target.v_multiplier, 0.1f);
    ImGui::TreePop();
  }

  ImGui::DragInt("Major branch cell min", &target.min_cell_count_for_major_branches, 1, 0, 1000);
  ImGui::DragInt("Minor branch cell max", &target.max_cell_count_for_minor_branches, 1, 0, 1000);

  ImGui::Checkbox("Recalculate UV", &target.recalculate_uv);
  ImGui::Checkbox("Fast UV", &target.fast_uv);
  ImGui::DragInt("Smooth iteration", &target.smooth_iteration, 0, 0, 10);
  ImGui::Checkbox("Branch", &target.enable_branch);
  ImGui::Checkbox("Foliage", &target.enable_foliage);
}
