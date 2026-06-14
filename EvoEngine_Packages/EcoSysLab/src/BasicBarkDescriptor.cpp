#include "BasicBarkDescriptor.hpp"
#include "EcoSysLabSerializationAdapters.hpp"

using namespace eco_sys_lab_package;

bool BasicBarkDescriptor::DrawGui(const std::shared_ptr<EditorLayer>& editor_layer) {
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
  if (editor_layer->DragAndDropButton<Material>(bark_material_ref, "Bark Material"))
    changed = true;
  return changed;
}

float BasicBarkDescriptor::GetValue(const float x_factor, const float distance_to_root) const {
  const float bark = bark_depth * glm::perlin(glm::vec3(bark_x_frequency * glm::sin(x_factor * 2.0f * glm::pi<float>()),
                                                        bark_x_frequency * glm::cos(x_factor * 2.0f * glm::pi<float>()),
                                                        bark_y_frequency * distance_to_root));

  float base = base_depth +
               base_depth * glm::perlin(glm::vec3(base_frequency * glm::sin(x_factor * 2.0f * glm::pi<float>()),
                                                  base_frequency * glm::cos(x_factor * 2.0f * glm::pi<float>()), 0.0f));

  base *= glm::pow(glm::max(0.0f, (base_max_distance - distance_to_root) / base_max_distance),
                   base_distance_decrease_factor);

  return bark + base;
}

void eco_sys_lab_package::SerializeBasicBarkDescriptor(YAML::Emitter& out, const BasicBarkDescriptor& target) {
  out << YAML::Key << "bark_x_frequency" << YAML::Value << target.bark_x_frequency;
  out << YAML::Key << "bark_y_frequency" << YAML::Value << target.bark_y_frequency;
  out << YAML::Key << "bark_depth" << YAML::Value << target.bark_depth;
  out << YAML::Key << "base_frequency" << YAML::Value << target.base_frequency;
  out << YAML::Key << "base_max_distance" << YAML::Value << target.base_max_distance;
  out << YAML::Key << "base_distance_decrease_factor" << YAML::Value << target.base_distance_decrease_factor;
  out << YAML::Key << "base_depth" << YAML::Value << target.base_depth;
  target.bark_material_ref.Save("bark_material_ref", out);
}

void eco_sys_lab_package::DeserializeBasicBarkDescriptor(const YAML::Node& in, BasicBarkDescriptor& target) {
  if (in["bark_x_frequency"])
    target.bark_x_frequency = in["bark_x_frequency"].as<float>();
  if (in["bark_y_frequency"])
    target.bark_y_frequency = in["bark_y_frequency"].as<float>();
  if (in["bark_depth"])
    target.bark_depth = in["bark_depth"].as<float>();
  if (in["base_frequency"])
    target.base_frequency = in["base_frequency"].as<float>();
  if (in["base_max_distance"])
    target.base_max_distance = in["base_max_distance"].as<float>();
  if (in["base_distance_decrease_factor"])
    target.base_distance_decrease_factor = in["base_distance_decrease_factor"].as<float>();
  if (in["base_depth"])
    target.base_depth = in["base_depth"].as<float>();
  target.bark_material_ref.Load("bark_material_ref", in);
}

void BasicBarkDescriptor::CollectAssetRef(std::vector<AssetRef>& list) {
  if (bark_material_ref.Get<Material>())
    list.push_back(bark_material_ref);
}
