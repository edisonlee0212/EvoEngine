#pragma once
#include <Plot2D.hpp>

#include "StrandModelData.hpp"
#include "StrandModelProfile.hpp"

namespace eco_sys_lab_plugin {
using namespace evo_engine;

/**
 * @struct StrandModelParameters
 * @brief Holds parameters for configuring the strand-based procedural tree model.
 */
struct StrandModelParameters {
  /**
   * @brief Strength of attraction force pulling strands towards the center.
   */
  float center_attraction_strength = 40000;

  /**
   * @brief Factor determining the maximum number of simulation iterations per cell.
   */
  int max_simulation_iteration_cell_factor = 5;

  /**
   * @brief Maximum number of iterations allowed for branch profile packing.
   */
  int branch_profile_packing_max_iteration = 200;

  /**
   * @brief Maximum number of iterations allowed for junction profile packing.
   */
  int junction_profile_packing_max_iteration = 500;

  /**
   * @brief Maximum number of iterations allowed for modified profile packing.
   */
  int modified_profile_packing_max_iteration = 1500;

  /**
   * @brief Threshold for detecting strand overlap.
   */
  float overlap_threshold = 0.1f;

  /**
   * @brief Number of strands assigned to end nodes.
   */
  int end_node_strands = 1;

  /**
   * @brief Number of strands along branches.
   */
  int strands_along_branch = 0;

  /**
   * @brief Flag indicating whether pre-merging of strands is performed.
   */
  bool pre_merge = false;

  /**
   * @brief Maximum number of nodes allowed in the structure, -1 for no limit.
   */
  int node_max_count = -1;

  /**
   * @brief Distance between boundary points.
   */
  int boundary_point_distance = 6;

  /**
   * @brief Color used to represent boundary points.
   */
  glm::vec4 boundary_point_color = glm::vec4(0.6f, 0.3f, 0, 1);

  /**
   * @brief Color used to represent content points.
   */
  glm::vec4 content_point_color = glm::vec4(0, 0.3, 0.0f, 1);

  /**
   * @brief Factor controlling the lateral push exerted on strands.
   */
  float side_push_factor = 1.0f;

  /**
   * @brief Factor controlling the lateral push exerted on apical strands.
   */
  float apical_side_push_factor = 1.f;

  /**
   * @brief Factor controlling rotation forces applied to strands.
   */
  float rotation_push_factor = 1.f;

  /**
   * @brief Factor controlling rotation forces on apical branches.
   */
  float apical_branch_rotation_push_factor = 1.f;

  /**
   * @brief Distribution controlling the twist along branches.
   */
  PlottedDistribution<float> branch_twist_distribution{};

  /**
   * @brief Distribution controlling the twist at junctions.
   */
  PlottedDistribution<float> junction_twist_distribution{};

  /**
   * @brief Distribution defining the radius variation of strands.
   */
  PlottedDistribution<float> strand_radius_distribution{};

  /**
   * @brief Range of cladoptosis (self-pruning) for branches.
   */
  float cladoptosis_range = 10.0f;

  /**
   * @brief Distribution governing the likelihood of cladoptosis.
   */
  PlottedDistribution<float> cladoptosis_distribution{};

  /**
   * @brief Physics settings applied to strand profiles.
   */
  ParticlePhysicsSettings profile_physics_settings{};

  /**
   * @brief Inspects in an editor.
   * @param editor_layer The editor layer managing inspection.
   * @return True if data was not modified during inspection.
   */
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);

  /**
   * @brief Saves to a YAML emitter.
   * @param name The name of the settings entry.
   * @param out The YAML emitter to serialize data into.
   */
  void Save(const std::string& name, YAML::Emitter& out) const;

  /**
   * @brief Loads from a YAML node.
   * @param name The name of the settings entry.
   * @param in The YAML node containing serialized data.
   */
  void Load(const std::string& name, const YAML::Node& in);
};
}  // namespace eco_sys_lab_plugin