#pragma once
#include <Plot2D.hpp>

#include "StrandModelData.hpp"
#include "StrandModelProfile.hpp"
using namespace evo_engine;
namespace eco_sys_lab {
struct StrandModelParameters {
  float center_attraction_strength = 40000;

  int max_simulation_iteration_cell_factor = 5;
  int branch_profile_packing_max_iteration = 200;
  int junction_profile_packing_max_iteration = 500;
  int modified_profile_packing_max_iteration = 1500;

  float overlap_threshold = 0.1f;
  int end_node_strands = 1;
  int strands_along_branch = 0;
  bool pre_merge = false;

  int node_max_count = -1;

  int boundary_point_distance = 6;

  glm::vec4 boundary_point_color = glm::vec4(0.6f, 0.3f, 0, 1);
  glm::vec4 content_point_color = glm::vec4(0, 0.3, 0.0f, 1);

  float side_push_factor = 1.0f;
  float apical_side_push_factor = 1.f;
  float rotation_push_factor = 1.f;
  float apical_branch_rotation_push_factor = 1.f;

  PlottedDistribution<float> branch_twist_distribution{};
  PlottedDistribution<float> junction_twist_distribution{};
  PlottedDistribution<float> strand_radius_distribution{};
  float cladoptosis_range = 10.0f;
  PlottedDistribution<float> cladoptosis_distribution{};

  ParticlePhysicsSettings profile_physics_settings{};
};
}  // namespace eco_sys_lab