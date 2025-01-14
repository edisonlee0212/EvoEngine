#include "DynamicStrandsDemo.hpp"

using namespace eco_sys_lab_plugin;

bool DynamicStrandsDemo::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::Button("Dry break")) {
    
  }
  if (ImGui::Button("Squishy break")) {
  }
  return changed;
}

void DynamicStrandsDemo::Update() {
  
}

void DynamicStrandsDemo::LateUpdate() {
  
}
