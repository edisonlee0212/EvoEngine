#include "BasicRootDescriptor.hpp"

#include "ShootModel.hpp"

using namespace eco_sys_lab_plugin;

void BasicRootDescriptor::PrepareController(RootGrowthController& root_growth_controller) const {
}
void BasicRootDescriptor::Serialize(YAML::Emitter& out) const {
}
void BasicRootDescriptor::Deserialize(const YAML::Node& in) {
}
bool BasicRootDescriptor::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  return false;
}