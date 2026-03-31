#include "LSystemLayer.hpp"
#include "ClassRegistry.hpp"
#include "LSystemDescriptor.hpp"

using namespace l_system_plugin;
using namespace evo_engine;

void LSystemLayer::OnCreate() {
  ClassRegistry::RegisterAsset<LSystemDescriptor>("LSystemDescriptor", {".lsys"});
}

void LSystemLayer::OnDestroy() {
}

void LSystemLayer::Update() {
}

void LSystemLayer::OnInspect(const std::shared_ptr<evo_engine::EditorLayer>& editor_layer) {
}
