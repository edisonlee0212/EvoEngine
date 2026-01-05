#include "SkeletalGraphSettings.hpp"

using namespace eco_sys_lab_plugin;

bool SkeletalGraphSettings::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  ImGui::DragFloat("Line thickness", &line_thickness, 0.001f, 0.0f, 1.0f);
  ImGui::DragFloat("Fixed line thickness", &fixed_line_thickness, 0.001f, 0.0f, 1.0f);
  ImGui::DragFloat("Branch point size", &branch_point_size, 0.01f, 0.0f, 1.0f);
  ImGui::DragFloat("Junction point size", &junction_point_size, 0.01f, 0.0f, 1.0f);

  ImGui::Checkbox("Fixed point size", &fixed_point_size);
  if (fixed_point_size) {
    ImGui::DragFloat("Fixed point size multiplier", &fixed_point_size_factor, 0.001f, 0.0f, 1.0f);
  }

  ImGui::ColorEdit4("Line color", &line_color.x);
  ImGui::ColorEdit4("Branch point color", &branch_point_color.x);
  ImGui::ColorEdit4("Junction point color", &junction_point_color.x);

  return changed;
}

void SkeletalGraphSettings::Save(const std::string& name, YAML::Emitter& out) const {
  out << YAML::Key << name << YAML::BeginMap;

  out << YAML::Key << "line_thickness" << YAML::Value << line_thickness;
  out << YAML::Key << "fixed_line_thickness" << YAML::Value << fixed_line_thickness;
  out << YAML::Key << "branch_point_size" << YAML::Value << branch_point_size;
  out << YAML::Key << "junction_point_size" << YAML::Value << junction_point_size;

  out << YAML::Key << "fixed_point_size" << YAML::Value << fixed_point_size;
  out << YAML::Key << "fixed_point_size_factor" << YAML::Value << fixed_point_size_factor;

  out << YAML::Key << "line_color" << YAML::Value << line_color;
  out << YAML::Key << "branch_point_color" << YAML::Value << branch_point_color;
  out << YAML::Key << "junction_point_color" << YAML::Value << junction_point_color;

  out << YAML::Key << "line_focus_color" << YAML::Value << line_focus_color;
  out << YAML::Key << "branch_focus_color" << YAML::Value << branch_focus_color;

  out << YAML::EndMap;
}

void SkeletalGraphSettings::Load(const std::string& name, const YAML::Node& in) {
  if (in[name]) {
    const auto& in_settings = in[name];
    if (in_settings["line_thickness"])
      line_thickness = in_settings["line_thickness"].as<float>();
    if (in_settings["fixed_line_thickness"])
      fixed_line_thickness = in_settings["fixed_line_thickness"].as<float>();
    if (in_settings["branch_point_size"])
      branch_point_size = in_settings["branch_point_size"].as<float>();
    if (in_settings["junction_point_size"])
      junction_point_size = in_settings["junction_point_size"].as<float>();

    if (in_settings["fixed_point_size"])
      fixed_point_size = in_settings["fixed_point_size"].as<bool>();
    if (in_settings["fixed_point_size_factor"])
      fixed_point_size_factor = in_settings["fixed_point_size_factor"].as<float>();

    if (in_settings["line_color"])
      line_color = in_settings["line_color"].as<glm::vec4>();
    if (in_settings["branch_point_color"])
      branch_point_color = in_settings["branch_point_color"].as<glm::vec4>();
    if (in_settings["junction_point_color"])
      junction_point_color = in_settings["junction_point_color"].as<glm::vec4>();

    if (in_settings["line_focus_color"])
      line_focus_color = in_settings["line_focus_color"].as<glm::vec4>();
    if (in_settings["branch_focus_color"])
      branch_focus_color = in_settings["branch_focus_color"].as<glm::vec4>();
  }
}