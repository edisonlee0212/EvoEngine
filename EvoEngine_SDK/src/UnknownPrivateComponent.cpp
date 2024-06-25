//
// Created by lllll on 8/23/2021.
//

#include "UnknownPrivateComponent.hpp"
using namespace evo_engine;
bool UnknownPrivateComponent::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  ImGui::Text(std::string("Type: " + original_type_name_).c_str());

  return false;
}