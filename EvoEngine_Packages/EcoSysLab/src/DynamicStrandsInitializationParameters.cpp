#include "DynamicStrandsInitializationParameters.hpp"

#include "BasicFoliageDescriptor.hpp"

using namespace eco_sys_lab_package;

void BundleSolverSettings::Save(const std::string& name, YAML::Emitter& out) const {
  out << YAML::Key << name << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "mode" << YAML::Value << static_cast<int>(mode);
  out << YAML::Key << "legacy_iterations" << YAML::Value << legacy_iterations;
  out << YAML::Key << "pair_iterations" << YAML::Value << pair_iterations;
  out << YAML::Key << "coarse_iterations" << YAML::Value << coarse_iterations;
  out << YAML::Key << "position_compliance_scale" << YAML::Value << position_compliance_scale;
  out << YAML::Key << "bending_compliance_scale" << YAML::Value << bending_compliance_scale;
  out << YAML::Key << "torsion_compliance_scale" << YAML::Value << torsion_compliance_scale;
  out << YAML::Key << "shape_matching_strength" << YAML::Value << shape_matching_strength;
  out << YAML::Key << "slice_spacing_factor" << YAML::Value << slice_spacing_factor;
  out << YAML::Key << "minimum_slice_members" << YAML::Value << minimum_slice_members;
  out << YAML::EndMap;
}

void BundleSolverSettings::Load(const std::string& name, const YAML::Node& in) {
  if (!in[name])
    return;
  const auto& settings = in[name];
  if (settings["mode"]) {
    const auto value = settings["mode"].as<int>();
    mode = value >= static_cast<int>(BundleSolverMode::Legacy) && value <= static_cast<int>(BundleSolverMode::Hybrid)
               ? static_cast<BundleSolverMode>(value)
               : BundleSolverMode::Legacy;
  }
  if (settings["legacy_iterations"])
    legacy_iterations = glm::max(1, settings["legacy_iterations"].as<int>());
  if (settings["pair_iterations"])
    pair_iterations = glm::max(1, settings["pair_iterations"].as<int>());
  if (settings["coarse_iterations"])
    coarse_iterations = glm::max(1, settings["coarse_iterations"].as<int>());
  if (settings["position_compliance_scale"])
    position_compliance_scale = glm::max(0.f, settings["position_compliance_scale"].as<float>());
  if (settings["bending_compliance_scale"])
    bending_compliance_scale = glm::max(0.f, settings["bending_compliance_scale"].as<float>());
  if (settings["torsion_compliance_scale"])
    torsion_compliance_scale = glm::max(0.f, settings["torsion_compliance_scale"].as<float>());
  if (settings["shape_matching_strength"])
    shape_matching_strength = glm::clamp(settings["shape_matching_strength"].as<float>(), 0.f, 1.f);
  if (settings["slice_spacing_factor"])
    slice_spacing_factor = glm::max(0.1f, settings["slice_spacing_factor"].as<float>());
  if (settings["minimum_slice_members"])
    minimum_slice_members = glm::max(1, settings["minimum_slice_members"].as<int>());
}

void DynamicStrandsInitializeParameters::Save(const std::string& name, YAML::Emitter& out) const {
  out << YAML::Key << name << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "min_segment_length" << YAML::Value << min_segment_length;
  out << YAML::Key << "max_segment_length" << YAML::Value << max_segment_length;
  out << YAML::Key << "uniform_subdivision" << YAML::Value << uniform_subdivision;

  damage_graph.Save("damage_graph", out);

  out << YAML::Key << "damage_scale_factor" << YAML::Value << damage_scale_factor;
  out << YAML::Key << "neighbor_vertical_range" << YAML::Value << neighbor_vertical_range;
  out << YAML::Key << "neighbor_horizontal_range" << YAML::Value << neighbor_horizontal_range;

  out << YAML::Key << "sapwood_offset" << YAML::Value << sapwood_offset;
  out << YAML::Key << "wood_transition" << YAML::Value << wood_transition;

  modulus_graph.Save("modulus_graph", out);

  strength_graph.Save("strength_graph", out);

  out << YAML::Key << "trunk_additional_strength" << YAML::Value << trunk_additional_strength;
  biological_properties_graph.Save("biological_properties_graph", out);

  leaf_position_alpha.Save("leaf_position_alpha", out);
  leaf_rotation_alpha.Save("leaf_rotation_alpha", out);
  max_leaf_position_strain.Save("max_leaf_position_strain", out);
  max_leaf_rotation_strain.Save("max_leaf_rotation_strain", out);

  out << YAML::Key << "root_transform" << YAML::Value << root_transform.value;
  out << YAML::Key << "use_cgal" << YAML::Value << use_cgal;
  out << YAML::Key << "triangulate_per_bundle" << YAML::Value << triangulate_per_bundle;

  out << YAML::Key << "alpha" << YAML::Value << alpha;
  out << YAML::Key << "bifurcation_alpha" << YAML::Value << bifurcation_alpha;
  out << YAML::Key << "max_dist_squared" << YAML::Value << max_dist_squared;
  out << YAML::Key << "use_cubic_hermite_spline" << YAML::Value << use_cubic_hermite_spline;
  out << YAML::Key << "min_bundle_size" << YAML::Value << min_bundle_size;

  bundle_solver.Save("bundle_solver", out);

  foliage_descriptor.Save("foliage_descriptor", out);

  out << YAML::EndMap;
}

void DynamicStrandsInitializeParameters::Load(const std::string& name, const YAML::Node& in) {
  if (in[name]) {
    const auto& in_parameters = in[name];

    if (in_parameters["min_segment_length"])
      min_segment_length = in_parameters["min_segment_length"].as<float>();
    if (in_parameters["max_segment_length"])
      max_segment_length = in_parameters["max_segment_length"].as<float>();

    if (in_parameters["uniform_subdivision"])
      uniform_subdivision = in_parameters["uniform_subdivision"].as<int>();

    damage_graph.Load("damage_graph", in_parameters);
    if (in_parameters["damage_scale_factor"])
      damage_scale_factor = in_parameters["damage_scale_factor"].as<glm::vec3>();
    if (in_parameters["neighbor_vertical_range"])
      neighbor_vertical_range = in_parameters["neighbor_vertical_range"].as<float>();
    if (in_parameters["neighbor_horizontal_range"])
      neighbor_horizontal_range = in_parameters["neighbor_horizontal_range"].as<float>();

    if (in_parameters["sapwood_offset"])
      sapwood_offset = in_parameters["sapwood_offset"].as<float>();
    if (in_parameters["wood_transition"])
      wood_transition = in_parameters["wood_transition"].as<float>();
    modulus_graph.Load("modulus_graph", in_parameters);

    strength_graph.Load("strength_graph", in_parameters);

    if (in_parameters["trunk_additional_strength"])
      trunk_additional_strength = in_parameters["trunk_additional_strength"].as<bool>();
    biological_properties_graph.Load("biological_properties_graph", in_parameters);

    leaf_position_alpha.Load("leaf_position_alpha", in_parameters);
    leaf_rotation_alpha.Load("leaf_rotation_alpha", in_parameters);
    max_leaf_position_strain.Load("max_leaf_position_strain", in_parameters);
    max_leaf_rotation_strain.Load("max_leaf_rotation_strain", in_parameters);

    if (in_parameters["root_transform"])
      root_transform.value = in_parameters["root_transform"].as<glm::mat4>();

    if (in_parameters["use_cgal"])
      use_cgal = in_parameters["use_cgal"].as<bool>();
    if (in_parameters["triangulate_per_bundle"])
      triangulate_per_bundle = in_parameters["triangulate_per_bundle"].as<bool>();

    if (in_parameters["alpha"])
      alpha = in_parameters["alpha"].as<float>();
    if (in_parameters["bifurcation_alpha"])
      bifurcation_alpha = in_parameters["bifurcation_alpha"].as<float>();
    if (in_parameters["max_dist_squared"])
      max_dist_squared = in_parameters["max_dist_squared"].as<float>();
    if (in_parameters["use_cubic_hermite_spline"])
      use_cubic_hermite_spline = in_parameters["use_cubic_hermite_spline"].as<bool>();
    if (in_parameters["min_bundle_size"])
      min_bundle_size = in_parameters["min_bundle_size"].as<int>();

    bundle_solver.Load("bundle_solver", in_parameters);

    foliage_descriptor.Load("foliage_descriptor", in_parameters);
  }
}
