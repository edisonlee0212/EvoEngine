//
// Created by lllll on 10/21/2022.
//
#include "TreeGrowthSettings.hpp"
using namespace eco_sys_lab_package;

bool TreeGrowthSettings::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::Checkbox("Enable space colonization", &use_space_colonization))
    changed = true;
  if (use_space_colonization) {
    if (ImGui::Checkbox("Space colonization auto resize", &space_colonization_auto_resize))
      changed = true;
  }
  return changed;
}

void TreeGrowthSettings::Save(const std::string& name, YAML::Emitter& out) const {
  out << YAML::Key << name << YAML::BeginMap;
  out << YAML::Key << "node_developmental_vigor_filling_rate" << YAML::Value << node_developmental_vigor_filling_rate;

  out << YAML::Key << "use_space_colonization" << YAML::Value << use_space_colonization;
  out << YAML::Key << "space_colonization_auto_resize" << YAML::Value << space_colonization_auto_resize;
  out << YAML::Key << "space_colonization_removal_distance_factor" << YAML::Value
      << space_colonization_removal_distance_factor;
  out << YAML::Key << "space_colonization_detection_distance_factor" << YAML::Value
      << space_colonization_detection_distance_factor;
  out << YAML::Key << "space_colonization_theta" << YAML::Value << space_colonization_theta;
  out << YAML::EndMap;
}

void TreeGrowthSettings::Load(const std::string& name, const YAML::Node& in) {
  if (in["name"]) {
    const auto& in_settings = in["name"];
    if (in_settings["node_developmental_vigor_filling_rate"])
      node_developmental_vigor_filling_rate = in_settings["node_developmental_vigor_filling_rate"].as<float>();
    if (in_settings["use_space_colonization"])
      use_space_colonization = in_settings["use_space_colonization"].as<bool>();
    if (in_settings["space_colonization_auto_resize"])
      space_colonization_auto_resize = in_settings["space_colonization_auto_resize"].as<bool>();
    if (in_settings["space_colonization_removal_distance_factor"])
      space_colonization_removal_distance_factor =
          in_settings["space_colonization_removal_distance_factor"].as<float>();
    if (in_settings["space_colonization_detection_distance_factor"])
      space_colonization_detection_distance_factor =
          in_settings["space_colonization_detection_distance_factor"].as<float>();
    if (in_settings["space_colonization_theta"])
      space_colonization_theta = in_settings["space_colonization_theta"].as<float>();
  }
}