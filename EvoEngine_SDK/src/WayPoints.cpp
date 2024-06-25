#include "WayPoints.hpp"

#include "EditorLayer.hpp"
using namespace evo_engine;

bool WayPoints::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  const auto scene = GetScene();
  bool changed = false;
  if (EntityRef temp_entity_holder; editor_layer->DragAndDropButton(temp_entity_holder, "Drop new SoilLayerDescriptor here...")) {
    if (auto entity = temp_entity_holder.Get(); scene->IsEntityValid(entity)) {
      entities.emplace_back(entity);
      changed = true;
    }
    temp_entity_holder.Clear();
  }
  for (int i = 0; i < entities.size(); i++) {
    auto entity = entities[i].Get();
    if (scene->IsEntityValid(entity)) {
      if (ImGui::TreeNodeEx(("No." + std::to_string(i + 1)).c_str(), ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Text(("Name: " + scene->GetEntityName(entity)).c_str());

        if (ImGui::Button("Remove")) {
          entities.erase(entities.begin() + i);
          changed = true;
          ImGui::TreePop();
          continue;
        }
        if (i < entities.size() - 1) {
          ImGui::SameLine();
          if (ImGui::Button("Move down")) {
            changed = true;
            std::swap(entities[i + 1], entities[i]);
          }
        }
        if (i > 0) {
          ImGui::SameLine();
          if (ImGui::Button("Move up")) {
            changed = true;
            std::swap(entities[i - 1], entities[i]);
          }
        }
        ImGui::TreePop();
      }
    } else {
      entities.erase(entities.begin() + i);
      i--;
    }
  }
  return changed;
}

void WayPoints::OnCreate() {
}

void WayPoints::OnDestroy() {
  entities.clear();
  speed = 1.0f;
}

void WayPoints::Update() {
}
