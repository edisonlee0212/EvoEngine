#include "EditorWidgets.hpp"
#include "imgui_internal.h"

bool ImGui::Splitter(bool split_vertically, float thickness, float& size1, float& size2, float min_size1,
                     float min_size2, float splitter_long_axis_size) {
  ImGuiContext& g = *GImGui;
  ImGuiWindow* window = g.CurrentWindow;
  ImGuiID id = window->GetID("##Splitter");
  ImRect bb;
  bb.Min = window->DC.CursorPos + (split_vertically ? ImVec2(size1, 0.0f) : ImVec2(0.0f, size1));
  bb.Max = bb.Min + CalcItemSize(split_vertically ? ImVec2(thickness, splitter_long_axis_size)
                                                  : ImVec2(splitter_long_axis_size, thickness),
                                 0.0f, 0.0f);
  return SplitterBehavior(bb, id, split_vertically ? ImGuiAxis_X : ImGuiAxis_Y, &size1, &size2, min_size1, min_size2,
                          0.0f);
}

bool ImGui::Combo(const std::string& label, const std::vector<std::string>& items, unsigned& current_selection,
                  ImGuiComboFlags flags) {
  bool modified = false;
  current_selection = glm::clamp(current_selection, 0u, static_cast<unsigned>(items.size()));
  if (ImGui::BeginCombo(label.c_str(), items[current_selection].c_str(),
                        flags))  // The second parameter is the label previewed before opening the combo.
  {
    for (unsigned i = 0; i < items.size(); i++) {
      const bool selected = current_selection == i;
      if (ImGui::Selectable(items[i].c_str(), selected)) {
        current_selection = i;
        modified = true;
      }
      if (selected) {
        ImGui::SetItemDefaultFocus();  // You may set the initial focus when opening the combo (scrolling
                                       // + for keyboard navigation support)
      }
    }
    ImGui::EndCombo();
  }
  return modified;
}

bool ImGui::Combo(const std::string& label, const std::vector<std::string>& items, int& current_selection,
                  ImGuiComboFlags flags) {
  bool modified = false;
  current_selection = glm::clamp(current_selection, 0, static_cast<int>(items.size()));
  if (ImGui::BeginCombo(label.c_str(), items[current_selection].c_str(),
                        flags))  // The second parameter is the label previewed before opening the combo.
  {
    for (int i = 0; i < items.size(); i++) {
      const bool selected = current_selection == i;
      if (ImGui::Selectable(items[i].c_str(), selected)) {
        current_selection = i;
        modified = true;
      }
      if (selected) {
        ImGui::SetItemDefaultFocus();  // You may set the initial focus when opening the combo (scrolling
                                       // + for keyboard navigation support)
      }
    }
    ImGui::EndCombo();
  }
  return modified;
}
