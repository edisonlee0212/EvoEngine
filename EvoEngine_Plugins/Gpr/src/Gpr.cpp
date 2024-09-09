#include "Gpr.hpp"

using namespace evo_engine;
using namespace gpr_plugin;
bool Gpr::SaveInternal(const std::filesystem::path& path) const {
  return false;
}

bool Gpr::LoadInternal(const std::filesystem::path& path) {
  return false;
}

void Gpr::OnCreate() {
  
}

bool Gpr::OnInspect(const std::shared_ptr<evo_engine::EditorLayer>& editor_layer) {
  return false;
}
