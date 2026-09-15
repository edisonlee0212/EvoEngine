#include "StrandModelParameters.hpp"

using namespace eco_sys_lab_package;

void StrandModelParameters::Save(const std::string& name, YAML::Emitter& out) const {
  out << YAML::Key << name << YAML::BeginMap;
  out << YAML::Key << "center_attraction_strength" << YAML::Value << center_attraction_strength;
  out << YAML::Key << "max_simulation_iteration_cell_factor" << YAML::Value << max_simulation_iteration_cell_factor;
  out << YAML::Key << "branch_profile_packing_max_iteration" << YAML::Value << branch_profile_packing_max_iteration;
  out << YAML::Key << "junction_profile_packing_max_iteration" << YAML::Value << junction_profile_packing_max_iteration;
  out << YAML::Key << "modified_profile_packing_max_iteration" << YAML::Value << modified_profile_packing_max_iteration;
  out << YAML::Key << "overlap_threshold" << YAML::Value << overlap_threshold;
  out << YAML::Key << "end_node_strands" << YAML::Value << end_node_strands;
  out << YAML::Key << "strands_along_branch" << YAML::Value << strands_along_branch;
  out << YAML::Key << "pre_merge" << YAML::Value << pre_merge;
  out << YAML::Key << "node_max_count" << YAML::Value << node_max_count;
  out << YAML::Key << "boundary_point_distance" << YAML::Value << boundary_point_distance;
  out << YAML::Key << "boundary_point_color" << YAML::Value << boundary_point_color;
  out << YAML::Key << "content_point_color" << YAML::Value << content_point_color;
  out << YAML::Key << "side_push_factor" << YAML::Value << side_push_factor;
  out << YAML::Key << "apical_side_push_factor" << YAML::Value << apical_side_push_factor;
  out << YAML::Key << "rotation_push_factor" << YAML::Value << rotation_push_factor;
  out << YAML::Key << "apical_branch_rotation_push_factor" << YAML::Value << apical_branch_rotation_push_factor;
  out << YAML::Key << "cladoptosis_range" << YAML::Value << cladoptosis_range;
  branch_twist_distribution.Save("branch_twist_distribution", out);
  junction_twist_distribution.Save("junction_twist_distribution", out);
  strand_radius_distribution.Save("strand_radius_distribution", out);
  cladoptosis_distribution.Save("cladoptosis_distribution", out);
  profile_physics_settings.Save("profile_physics_settings", out);

  out << YAML::EndMap;
}

void StrandModelParameters::Load(const std::string& name, const YAML::Node& in) {
  if (in[name]) {
    const auto& in_settings = in[name];
    if (in_settings["center_attraction_strength"])
      center_attraction_strength = in_settings["center_attraction_strength"].as<float>();
    if (in_settings["max_simulation_iteration_cell_factor"])
      max_simulation_iteration_cell_factor = in_settings["max_simulation_iteration_cell_factor"].as<int>();
    if (in_settings["branch_profile_packing_max_iteration"])
      branch_profile_packing_max_iteration = in_settings["branch_profile_packing_max_iteration"].as<int>();
    if (in_settings["junction_profile_packing_max_iteration"])
      junction_profile_packing_max_iteration = in_settings["junction_profile_packing_max_iteration"].as<int>();
    if (in_settings["modified_profile_packing_max_iteration"])
      modified_profile_packing_max_iteration = in_settings["modified_profile_packing_max_iteration"].as<int>();
    if (in_settings["overlap_threshold"])
      overlap_threshold = in_settings["overlap_threshold"].as<float>();
    if (in_settings["end_node_strands"])
      end_node_strands = in_settings["end_node_strands"].as<int>();
    if (in_settings["strands_along_branch"])
      strands_along_branch = in_settings["strands_along_branch"].as<int>();
    if (in_settings["pre_merge"])
      pre_merge = in_settings["pre_merge"].as<bool>();
    if (in_settings["node_max_count"])
      node_max_count = in_settings["node_max_count"].as<int>();
    if (in_settings["boundary_point_distance"])
      boundary_point_distance = in_settings["boundary_point_distance"].as<int>();
    if (in_settings["boundary_point_color"])
      boundary_point_color = in_settings["boundary_point_color"].as<glm::vec4>();
    if (in_settings["content_point_color"])
      content_point_color = in_settings["content_point_color"].as<glm::vec4>();
    if (in_settings["side_push_factor"])
      side_push_factor = in_settings["side_push_factor"].as<float>();
    if (in_settings["apical_side_push_factor"])
      apical_side_push_factor = in_settings["apical_side_push_factor"].as<float>();
    if (in_settings["rotation_push_factor"])
      rotation_push_factor = in_settings["rotation_push_factor"].as<float>();
    if (in_settings["apical_branch_rotation_push_factor"])
      apical_branch_rotation_push_factor = in_settings["apical_branch_rotation_push_factor"].as<float>();
    if (in_settings["cladoptosis_range"])
      cladoptosis_range = in_settings["cladoptosis_range"].as<float>();

    branch_twist_distribution.Load("branch_twist_distribution", in_settings);
    junction_twist_distribution.Load("junction_twist_distribution", in_settings);
    strand_radius_distribution.Load("strand_radius_distribution", in_settings);
    cladoptosis_distribution.Load("cladoptosis_distribution", in_settings);
    profile_physics_settings.Load("profile_physics_settings", in_settings);
  }
}
