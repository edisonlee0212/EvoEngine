#include "EcoSysLabSerializationAdapters.hpp"

using namespace eco_sys_lab_package;

void AdvancedShootDescriptor::PrepareController(ShootGrowthController& shoot_growth_controller) const {
}

void eco_sys_lab_package::SerializeAdvancedShootDescriptor(YAML::Emitter& out, const AdvancedShootDescriptor& target) {
  (void)out;
  (void)target;
}

void eco_sys_lab_package::DeserializeAdvancedShootDescriptor(const YAML::Node& in, AdvancedShootDescriptor& target) {
  (void)in;
  (void)target;
}

bool AdvancedShootDescriptor::DrawGui(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  return changed;
}
