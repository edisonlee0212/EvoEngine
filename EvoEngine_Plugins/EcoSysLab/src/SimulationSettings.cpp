#include "SimulationSettings.hpp"

using namespace eco_sys_lab;

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

bool SimulationSettings::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::Button("Grow weekly")) {
    delta_time = 0.01918f;
    changed = true;
  }
  ImGui::SameLine();
  if (ImGui::Button("Grow monthly")) {
    delta_time = 0.0822f;
    changed = true;
  }
  ImGui::SameLine();
  if (ImGui::DragFloat("Delta time", &delta_time, 0.00001f, 0, 1, "%.5f"))
    changed = true;
  if (ImGui::Checkbox("Auto clear fruit and leaves", &auto_clear_fruit_and_leaves))
    changed = true;
  if (ImGui::DragFloat("Crown shyness", &crown_shyness_distance, 0.01f, 0.0f, 1.0f))
    changed = true;
  if (ImGui::Checkbox("Simulate soil", &soil_simulation))
    changed = true;
  if (ImGui::TreeNode("Lighting Estimation Settings")) {
    changed = ImGui::DragFloat("Skylight Intensity", &skylight_intensity, 0.01f, 0.0f, 10.0f) || changed;
    changed = ImGui::DragFloat("Environmental Intensity", &environment_light_intensity, 0.01f, 0.0f, 10.0f) || changed;
    changed = ImGui::DragFloat("Shadow distance loss", &shadow_distance_loss, 0.01f, 0.0f, 10.0f) || changed;
    changed = ImGui::DragFloat("Detection radius", &detection_radius, 0.001f, 0.0f, 1.0f) || changed;
    changed = ImGui::DragInt("Blur iteration", &blur_iteration, 1, 0, 10) || changed;

    ImGui::TreePop();
  }
  return changed;
}
