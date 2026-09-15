#pragma once
#include "CubeVolume.hpp"
#include "EditorLayer.hpp"
#include "InspectorRegistry.hpp"
#include "RadialBoundingVolume.hpp"
#include "Soil.hpp"
#include "TreeStructor.hpp"
namespace eco_sys_lab_package {
struct CubeVolumeInspector {
  PrivateComponentRef privateComponentRef{};
  bool Inspect(evo_engine::InspectorContext& context, CubeVolume& target);
};
struct RadialBoundingVolumeInspector {
  float augmentation = 1.0f;
  AssetRef pointCloud;
  bool displayBound = false;
  bool Inspect(evo_engine::InspectorContext& context, RadialBoundingVolume& target);
};
struct SoilInspector {
  float x_depth = 1;
  float z_depth = 1;
  float water_factor = 20.f;
  float nutrient_factor = 1.f;
  bool ground_surface = false;
  AssetRef soil_albedo_texture;
  AssetRef soil_normal_texture;
  AssetRef soil_roughness_texture;
  AssetRef soil_height_texture;
  AssetRef soil_metallic_texture;
  bool Inspect(evo_engine::InspectorContext& context, Soil& target);
};
struct TreeStructorInspector {
  Handle previous_handle = 0;
  std::vector<glm::vec3> scattered_point_connections_starts;
  std::vector<glm::vec3> scattered_point_connections_ends;
  std::vector<glm::vec4> scattered_point_connection_colors;
  std::vector<glm::vec3> candidate_branch_connection_starts;
  std::vector<glm::vec3> candidate_branch_connection_ends;
  std::vector<glm::vec4> candidate_branch_connection_colors;
  std::vector<glm::vec3> reversed_candidate_branch_connection_starts;
  std::vector<glm::vec3> reversed_candidate_branch_connection_ends;
  std::vector<glm::vec4> reversed_candidate_branch_connection_colors;
  std::vector<glm::vec3> filtered_branch_connection_starts;
  std::vector<glm::vec3> filtered_branch_connection_ends;
  std::vector<glm::vec4> filtered_branch_connection_colors;
  std::vector<glm::vec3> selected_branch_connection_starts;
  std::vector<glm::vec3> selected_branch_connection_ends;
  std::vector<glm::vec4> selected_branch_connection_colors;
  std::vector<glm::vec3> scatter_point_to_branch_connection_starts;
  std::vector<glm::vec3> scatter_point_to_branch_connection_ends;
  std::vector<glm::vec4> scatter_point_to_branch_connection_colors;
  std::vector<glm::vec3> predicted_branch_starts;
  std::vector<glm::vec3> predicted_branch_ends;
  std::vector<glm::vec4> predicted_branch_colors;
  std::vector<float> predicted_branch_widths;
  std::vector<ParticleInfo> allocated_point_matrices;
  std::vector<ParticleInfo> scatter_point_matrices;
  bool enable_debug_rendering = true;
  bool use_real_branch_width = true;
  float predicted_branch_width = 0.005f;
  float connection_width = 0.001f;
  float point_size = 1.f;
  int color_mode = 0;
  float import_scale = 0.1f;
  GizmoSettings gizmo_settings{};
  std::shared_ptr<ParticleInfoList> allocated_point_info_list;
  std::shared_ptr<ParticleInfoList> scattered_point_info_list;
  std::shared_ptr<ParticleInfoList> scattered_point_connection_info_list;
  std::shared_ptr<ParticleInfoList> candidate_branch_connection_info_list;
  std::shared_ptr<ParticleInfoList> reversed_candidate_branch_connection_info_list;
  std::shared_ptr<ParticleInfoList> filtered_branch_connection_info_list;
  std::shared_ptr<ParticleInfoList> selected_branch_connection_info_list;
  std::shared_ptr<ParticleInfoList> scatter_point_to_branch_connection_info_list;
  std::shared_ptr<ParticleInfoList> selected_branch_info_list;
  glm::vec4 scatter_point_to_branch_connection_color = glm::vec4(1, 0, 1, 1);
  glm::vec4 allocated_point_color = glm::vec4(0, 0.5, 0.25, 1);
  glm::vec4 scatter_point_color = glm::vec4(0.25, 0.5, 0, 1);
  glm::vec4 scattered_point_connection_color = glm::vec4(0, 0, 0, 1);
  glm::vec4 candidate_branch_connection_color = glm::vec4(1, 1, 0, 1);
  glm::vec4 reversed_candidate_branch_connection_color = glm::vec4(0, 1, 1, 1);
  glm::vec4 filtered_branch_connection_color = glm::vec4(0, 0, 1, 1);
  glm::vec4 selected_branch_connection_color = glm::vec4(0.3, 0, 0, 1);
  glm::vec4 selected_branch_color = glm::vec4(0.6, 0.3, 0.0, 1.0f);
  bool enable_allocated_points = false;
  bool enable_scattered_points = false;
  bool enable_scattered_point_connections = false;
  bool enable_scatter_point_to_branch_connections = false;
  bool enable_candidate_branch_connections = false;
  bool enable_reversed_candidate_branch_connections = false;
  bool enable_filtered_branch_connections = false;
  bool enable_selected_branch_connections = true;
  bool enable_selected_branches = true;
  bool debug_allocated_points = true;
  bool debug_scattered_points = true;
  bool debug_scattered_point_connections = false;
  bool debug_scatter_point_to_branch_connections = false;
  bool debug_candidate_connections = false;
  bool debug_reversed_candidate_connections = false;
  bool debug_filtered_connections = false;
  bool debug_selected_branch_connections = true;
  bool debug_selected_branches = true;
  bool Inspect(evo_engine::InspectorContext& context, TreeStructor& target);
  void FormInfoEntities(const TreeStructor& target) const;
};
}  // namespace eco_sys_lab_package
