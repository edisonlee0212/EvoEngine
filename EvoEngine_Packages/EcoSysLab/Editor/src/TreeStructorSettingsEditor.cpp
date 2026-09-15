#include <unordered_set>
#include "BasicFoliageDescriptor.hpp"
#include "EcoSysLabLayer.hpp"
#include "EcoSysLabSerializationAdapters.hpp"
#include "EcoSysLabSettingsEditor.hpp"
#include "EditorFileDialogs.hpp"
#include "EditorLayer.hpp"
#include "EditorWidgets.hpp"
#include "Platform.hpp"
#include "SDKInspectionAdapters.hpp"
#include "TreeStructor.hpp"
#include "rapidcsv.h"

using namespace eco_sys_lab_package;

void eco_sys_lab_package::InspectSettings(ConnectivityGraphSettings& target) {
  // ImGui::Checkbox("Allow Reverse connections", &reverse_connection);
  if (ImGui::Button("Load reduced connection settings")) {
    target.point_point_connection_detection_radius = 0.05f;
    target.point_branch_connection_detection_radius = 0.1f;
    target.branch_branch_connection_max_length_range = 15.0f;
    target.direction_connection_angle_limit = 30.0f;
    target.indirect_connection_angle_limit = 15.0f;
    target.max_scatter_point_connection_height = 1.5f;
    target.parallel_shift_check = true;
  }
  if (ImGui::Button("Load default connection settings")) {
    target = ConnectivityGraphSettings();
  }
  if (ImGui::Button("Load max connection settings")) {
    target.point_point_connection_detection_radius = 0.05f;
    target.point_branch_connection_detection_radius = 0.1f;
    target.branch_branch_connection_max_length_range = 10.0f;
    target.direction_connection_angle_limit = 90.0f;
    target.indirect_connection_angle_limit = 90.0f;
    target.max_scatter_point_connection_height = 10.f;
    target.parallel_shift_check = false;
  }

  ImGui::DragFloat("Point-point connection max height", &target.max_scatter_point_connection_height, 0.01f, 0.01f,
                   3.0f);
  ImGui::DragFloat("Point-point detection radius", &target.point_point_connection_detection_radius, 0.01f, 0.01f, 1.0f);
  ImGui::DragFloat("Point-branch detection radius", &target.point_branch_connection_detection_radius, 0.01f, 0.01f,
                   2.0f);
  ImGui::DragFloat("Branch-branch detection range", &target.branch_branch_connection_max_length_range, 0.01f, 0.01f,
                   2.0f);
  ImGui::DragFloat("Direct connection angle limit", &target.direction_connection_angle_limit, 0.01f, 0.0f, 180.0f);
  ImGui::DragFloat("Indirect connection angle limit", &target.indirect_connection_angle_limit, 0.01f, 0.0f, 180.0f);

  ImGui::Checkbox("Zigzag check", &target.zigzag_check);
  if (target.zigzag_check) {
    ImGui::DragFloat("Zigzag branch shortening", &target.zigzag_branch_shortening, 0.01f, 0.0f, 0.5f);
  }
  ImGui::Checkbox("Parallel shift check", &target.parallel_shift_check);
  if (target.parallel_shift_check)
    ImGui::DragFloat("Parallel Shift range limit", &target.parallel_shift_limit_range, 0.01f, 0.0f, 1.0f);

  ImGui::Checkbox("Point existence check", &target.point_existence_check);
  if (target.point_existence_check)
    ImGui::DragFloat("Point existence check radius", &target.point_existence_check_radius, 0.01f, 0.0f, 1.0f);
}

void eco_sys_lab_package::InspectSettings(ReconstructionSettings& target) {
  ImGui::DragFloat("Internode length", &target.internode_length, 0.01f, 0.01f, 1.0f);
  ImGui::DragFloat("Root node max height", &target.min_height, 0.01f, 0.01f, 1.0f);
  ImGui::DragFloat("Tree distance limit", &target.minimum_tree_distance, 0.01f, 0.01f, 1.0f);
  ImGui::DragFloat("Branch shortening", &target.branch_shortening, 0.01f, 0.01f, 0.4f);
  ImGui::DragInt("Max parent candidate size", &target.max_parent_candidate_size, 1, 2, 10);
  ImGui::DragInt("Max child size", &target.max_child_size, 1, 2, 10);

  ImGui::DragFloat("Override thickness root distance", &target.override_thickness_root_distance, 0.01f, 0.01f, 0.5f);
  ImGui::DragFloat("Space colonization factor", &target.space_colonization_factor, 0.01f, 0.f, 1.0f);
  if (target.space_colonization_factor > 0.0f) {
    ImGui::DragInt("Space colonization timeout", &target.space_colonization_timeout, 1, 0, 500);
    ImGui::DragFloat("Space colonization removal distance", &target.space_colonization_removal_distance_factor, 0.1f,
                     0.f, 10.0f);
    ImGui::DragFloat("Space colonization detection distance", &target.space_colonization_detection_distance_factor,
                     0.1f, 0.f, 20.0f);
    ImGui::DragFloat("Space colonization perception theta", &target.space_colonization_theta, 0.1f, 0.f, 90.0f);
  }
  ImGui::DragFloat("End node thickness", &target.end_node_thickness, 0.001f, 0.001f, 1.0f);
  ImGui::DragFloat("Thickness sum factor", &target.thickness_sum_factor, 0.01f, 0.0f, 2.0f);
  ImGui::DragFloat("Thickness accumulation factor", &target.thickness_accumulation_factor, 0.00001f, 0.0f, 1.0f,
                   "%.5f");
  ImGui::Checkbox("Use imported root thickness", &target.apply_root_thickness);
  ImGui::Checkbox("Limit parent thickness", &target.limit_parent_thickness);
  ImGui::DragFloat("Minimum root thickness", &target.minimum_root_thickness, 0.001f, 0.0f, 1.0f, "%.3f");
  ImGui::DragInt("Minimum node count", &target.minimum_node_count, 1, 0, 100);

  ImGui::DragInt("Node back track limit", &target.node_back_track_limit, 1, 0, 100);
  ImGui::DragInt("Branch back track limit", &target.branch_back_track_limit, 1, 0, 10);

  ImGui::Checkbox("Use root distance", &target.use_root_distance);

  ImGui::DragInt("Optimization timeout", &target.optimization_timeout, 1, 0, 100);

  ImGui::DragFloat("Direction smoothing", &target.direction_smoothing, 0.01f, 0.0f, 1.0f);
  ImGui::DragFloat("Position smoothing", &target.position_smoothing, 0.01f, 0.0f, 1.0f);
  ImGui::DragInt("Smoothing iteration", &target.smooth_iteration, 1, 0, 100);

  ImGui::Checkbox("Use foliage", &target.use_foliage);
  /*
  ImGui::Checkbox("Candidate Search", &m_candidateSearch);
  if (m_candidateSearch) ImGui::DragInt("Candidate Search limit", &m_candidateSearchLimit, 1, 0, 10);
  ImGui::Checkbox("Force connect all branches", &m_forceConnectAllBranches);
  */
}
