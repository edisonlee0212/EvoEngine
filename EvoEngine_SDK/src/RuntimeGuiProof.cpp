#include "RuntimeGuiProof.hpp"

using namespace evo_engine;

void RuntimeGuiProof::Draw(const ImVec2 origin, const ImVec2 size) {
  BeginView(origin, size);
  ImGui::SetNextWindowBgAlpha(0.65f);
  if (context_.BeginWindow("Runtime GUI proof")) {
    ImGui::TextUnformatted("One ImGui context / separate overlay");
    ImGui::SliderFloat("Slider", &value_, 0, 1);
    ImGui::InputText("Text", text_, sizeof(text_));
    ImGui::BeginChild("Child", {0, 45}, ImGuiChildFlags_Borders);
    ImGui::TextUnformatted("Child window");
    ImGui::EndChild();
    if (ImGui::Button("Popup"))
      context_.OpenPopup("Proof popup");
    if (context_.BeginPopup("Proof popup")) {
      ImGui::TextUnformatted("Owned popup");
      if (ImGui::Button("Close"))
        ImGui::CloseCurrentPopup();
      context_.EndPopup();
    }
    ImGui::SameLine();
    if (ImGui::Button("Modal"))
      context_.OpenPopup("Proof modal");
    if (context_.BeginPopupModal("Proof modal", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
      ImGui::TextUnformatted("Shared modal blocks the editor too");
      if (ImGui::Button("Close modal"))
        ImGui::CloseCurrentPopup();
      context_.EndPopup();
    }
    ImGui::SameLine();
    ImGui::TextUnformatted("Hover me");
    if (ImGui::IsItemHovered()) {
      if (context_.BeginTooltip()) {
        ImGui::TextUnformatted("Owned runtime tooltip");
        context_.EndTooltip();
      }
    }
  }
  context_.EndWindow();
  context_.FinishView();
}
