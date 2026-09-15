#include "SimulationSettings.hpp"

using namespace eco_sys_lab_package;

void SimulationSettings::Save(const std::string& name, YAML::Emitter& out) const {
  out << YAML::Key << name << YAML::Value << YAML::BeginMap;
  Serialize(out);
  out << YAML::EndMap;
}

void SimulationSettings::Load(const std::string& name, const YAML::Node& in) {
  if (in[name])
    Deserialize(in[name]);
}

void SimulationSettings::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "delta_time" << YAML::Value << delta_time;
  out << YAML::Key << "soil_simulation" << YAML::Value << soil_simulation;
  out << YAML::Key << "auto_clear_fruit_and_leaves" << YAML::Value << auto_clear_fruit_and_leaves;
  out << YAML::Key << "crown_shyness_distance" << YAML::Value << crown_shyness_distance;
  out << YAML::Key << "max_node_count" << YAML::Value << max_node_count;
  out << YAML::Key << "max_flow_count" << YAML::Value << max_flow_count;

  out << YAML::Key << "skylight_intensity" << YAML::Value << skylight_intensity;
  out << YAML::Key << "shadow_distance_loss" << YAML::Value << shadow_distance_loss;
  out << YAML::Key << "detection_radius" << YAML::Value << detection_radius;
  out << YAML::Key << "environment_light_intensity" << YAML::Value << environment_light_intensity;
  out << YAML::Key << "blur_iteration" << YAML::Value << blur_iteration;
}

void SimulationSettings::Deserialize(const YAML::Node& in) {
  if (in["delta_time"])
    delta_time = in["delta_time"].as<float>();
  if (in["soil_simulation"])
    soil_simulation = in["soil_simulation"].as<bool>();
  if (in["auto_clear_fruit_and_leaves"])
    auto_clear_fruit_and_leaves = in["auto_clear_fruit_and_leaves"].as<bool>();
  if (in["crown_shyness_distance"])
    crown_shyness_distance = in["crown_shyness_distance"].as<float>();
  if (in["max_node_count"])
    max_node_count = in["max_node_count"].as<int>();
  if (in["max_flow_count"])
    max_flow_count = in["max_flow_count"].as<int>();

  if (in["skylight_intensity"])
    skylight_intensity = in["skylight_intensity"].as<float>();
  if (in["shadow_distance_loss"])
    shadow_distance_loss = in["shadow_distance_loss"].as<float>();
  if (in["detection_radius"])
    detection_radius = in["detection_radius"].as<float>();
  if (in["environment_light_intensity"])
    environment_light_intensity = in["environment_light_intensity"].as<float>();

  if (in["blur_iteration"])
    blur_iteration = in["blur_iteration"].as<int>();
}
