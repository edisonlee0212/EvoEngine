#include "TreeController.hpp"

using namespace eco_sys_lab_plugin;
bool TreePruningSettings::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::DragFloat("Low Branch Pruning", &low_branch_pruning, 0.01f, 0.0f, 1.f))
    changed = true;
  return changed;
}

void TreePruningSettings::Save(const std::string& name, YAML::Emitter& out) const {
  out << YAML::Key << name << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "low_branch_pruning" << YAML::Value << low_branch_pruning;
  out << YAML::EndMap;
}

void TreePruningSettings::Load(const std::string& name, const YAML::Node& in) {
  if (in[name]) {
    const auto& cd = in[name];
    if (cd["low_branch_pruning"]) {
      low_branch_pruning = cd["low_branch_pruning"].as<float>();
    }
  }
}
