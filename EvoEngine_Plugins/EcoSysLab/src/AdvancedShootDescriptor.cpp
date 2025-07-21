#include "AdvancedShootDescriptor.hpp"

using namespace eco_sys_lab_plugin;
void AdvancedShootDescriptor::PrepareController(ShootGrowthController& shoot_growth_controller) const {
}
void AdvancedShootDescriptor::Serialize(YAML::Emitter& out) const {
}
void AdvancedShootDescriptor::Deserialize(const YAML::Node& in) {
}
bool AdvancedShootDescriptor::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  return changed;
}