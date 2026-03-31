#include "LSystemDescriptor.hpp"
#include <EditorLayer.hpp>

using namespace l_system_plugin;
using namespace evo_engine;

bool LSystemDescriptor::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;

  if (ImGui::DragInt("Derivation Steps", &derivation_steps, 1, 0, 100))
    changed = true;

  int seed_int = static_cast<int>(seed);
  if (ImGui::DragInt("Seed", &seed_int, 1, 0, 999999)) {
    seed = static_cast<unsigned int>(seed_int);
    changed = true;
  }

  if (ImGui::DragFloat3("Root Position", &root_position.x, 0.1f))
    changed = true;

  glm::vec3 euler = glm::degrees(glm::eulerAngles(root_rotation));
  if (ImGui::DragFloat3("Root Rotation (deg)", &euler.x, 1.0f)) {
    root_rotation = glm::quat(glm::radians(euler));
    changed = true;
  }

  if (ImGui::DragFloat("Default Length", &default_length, 0.01f, 0.001f, 100.0f))
    changed = true;

  if (ImGui::DragFloat("Default Thickness", &default_thickness, 0.001f, 0.001f, 10.0f))
    changed = true;

  if (ImGui::Checkbox("Auto Derive on Change", &auto_derive_on_change))
    changed = true;

  return changed;
}

void LSystemDescriptor::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "derivation_steps" << YAML::Value << derivation_steps;
  out << YAML::Key << "seed" << YAML::Value << seed;
  out << YAML::Key << "root_position" << YAML::Value << root_position;
  out << YAML::Key << "root_rotation" << YAML::Value << root_rotation;
  out << YAML::Key << "default_length" << YAML::Value << default_length;
  out << YAML::Key << "default_thickness" << YAML::Value << default_thickness;
  out << YAML::Key << "auto_derive_on_change" << YAML::Value << auto_derive_on_change;
}

void LSystemDescriptor::Deserialize(const YAML::Node& in) {
  if (in["derivation_steps"])
    derivation_steps = in["derivation_steps"].as<int>();
  if (in["seed"])
    seed = in["seed"].as<unsigned int>();
  if (in["root_position"])
    root_position = in["root_position"].as<glm::vec3>();
  if (in["root_rotation"])
    root_rotation = in["root_rotation"].as<glm::quat>();
  if (in["default_length"])
    default_length = in["default_length"].as<float>();
  if (in["default_thickness"])
    default_thickness = in["default_thickness"].as<float>();
  if (in["auto_derive_on_change"])
    auto_derive_on_change = in["auto_derive_on_change"].as<bool>();
}
