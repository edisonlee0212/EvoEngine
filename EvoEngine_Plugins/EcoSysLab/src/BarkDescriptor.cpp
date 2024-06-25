#include "BarkDescriptor.hpp"

using namespace eco_sys_lab;

bool BarkDescriptor::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) {
  bool changed = false;
  if (ImGui::DragFloat("Bark X Frequency", &bark_x_frequency, 0.1f, 0.0f, 100.0f))
    changed = true;
  if (ImGui::DragFloat("Bark Y Frequency", &bark_y_frequency, 0.1f, 0.0f, 100.0f))
    changed = true;
  if (ImGui::DragFloat("Bark Depth", &bark_depth, 0.01f, 0.0f, 1.0f))
    changed = true;

  if (ImGui::DragFloat("Base Frequency", &base_frequency, 0.01f, 0.0f, 1.0f))
    changed = true;
  if (ImGui::DragFloat("Base Max Distance", &base_max_distance, 0.01f, 0.0f, 1.0f))
    changed = true;
  if (ImGui::DragFloat("Base Distance Decrease Factor", &base_distance_decrease_factor, 0.01f, 0.0f, 1.0f))
    changed = true;
  if (ImGui::DragFloat("Base Depth", &base_depth, 0.01f, 0.0f, 1.0f))
    changed = true;

  return changed;
}

float BarkDescriptor::GetValue(const float xFactor, const float distanceToRoot) {
  float bark = bark_depth * glm::perlin(glm::vec3(bark_x_frequency * glm::sin(xFactor * 2.0f * glm::pi<float>()),
                                                  bark_x_frequency * glm::cos(xFactor * 2.0f * glm::pi<float>()),
                                                  bark_y_frequency * distanceToRoot));

  float base = base_depth +
               base_depth * glm::perlin(glm::vec3(base_frequency * glm::sin(xFactor * 2.0f * glm::pi<float>()),
                                                  base_frequency * glm::cos(xFactor * 2.0f * glm::pi<float>()), 0.0f));

  base *=
      glm::pow(glm::max(0.0f, (base_max_distance - distanceToRoot) / base_max_distance), base_distance_decrease_factor);

  return bark + base;
}

void BarkDescriptor::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "bark_x_frequency" << YAML::Value << bark_x_frequency;
  out << YAML::Key << "bark_y_frequency" << YAML::Value << bark_y_frequency;
  out << YAML::Key << "bark_depth" << YAML::Value << bark_depth;
  out << YAML::Key << "base_frequency" << YAML::Value << base_frequency;
  out << YAML::Key << "base_max_distance" << YAML::Value << base_max_distance;
  out << YAML::Key << "base_distance_decrease_factor" << YAML::Value << base_distance_decrease_factor;
  out << YAML::Key << "base_depth" << YAML::Value << base_depth;
}

void BarkDescriptor::Deserialize(const YAML::Node& in) {
  if (in["bark_x_frequency"])
    bark_x_frequency = in["bark_x_frequency"].as<float>();
  if (in["bark_y_frequency"])
    bark_y_frequency = in["bark_y_frequency"].as<float>();
  if (in["bark_depth"])
    bark_depth = in["bark_depth"].as<float>();
  if (in["base_frequency"])
    base_frequency = in["base_frequency"].as<float>();
  if (in["base_max_distance"])
    base_max_distance = in["base_max_distance"].as<float>();
  if (in["base_distance_decrease_factor"])
    base_distance_decrease_factor = in["base_distance_decrease_factor"].as<float>();
  if (in["base_depth"])
    base_depth = in["base_depth"].as<float>();
}
