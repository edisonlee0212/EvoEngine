#include "Application.hpp"
#include "DigitalAgricultureInspectionAdapters.hpp"
#include "DigitalAgricultureInspectorStates.hpp"
#include "DigitalAgricultureSerializationAdapters.hpp"
#include "EditorFileDialogs.hpp"
#include "EditorLayer.hpp"
#include "Scene.hpp"
#include "Sorghum.hpp"
#include "SorghumGrowthStages.hpp"
#include "SorghumLayer.hpp"
#include "Times.hpp"
#include "Utilities.hpp"
#include "rapidcsv.h"
using namespace digital_agriculture_package;
bool SorghumGrowthStagesInspector::Inspect(InspectorContext& context, SorghumGrowthStages& growth_stages) {
  (void)context;
  auto& sorghum_growth_stages = growth_stages.sorghum_growth_stages;
  auto& state_mode = growth_stages.state_mode;
  if (ImGui::Button("Instantiate")) {
    auto entity = growth_stages.CreateEntity();
  }
  auto& auto_save = ui_auto_save;
  ImGui::Checkbox("Auto save", &auto_save);
  if (!auto_save) {
    ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(255, 0, 0, 255));
    ImGui::Text("[Auto save disabled!]");
    ImGui::PopStyleColor();
  } else {
    auto& last_auto_save_time = ui_last_auto_save_time;
    auto& auto_save_interval = ui_auto_save_interval;
    if (ImGui::TreeNodeEx("Auto save settings")) {
      if (ImGui::DragFloat("Time interval", &auto_save_interval, 1.0f, 2.0f, 300.0f)) {
        auto_save_interval = glm::clamp(auto_save_interval, 5.0f, 300.0f);
      }
      ImGui::TreePop();
    }
    if (last_auto_save_time == 0) {
      last_auto_save_time = ApplicationContext::Get().GetTimes().Now();
    } else if (last_auto_save_time + auto_save_interval < ApplicationContext::Get().GetTimes().Now()) {
      last_auto_save_time = ApplicationContext::Get().GetTimes().Now();
      if (!growth_stages.Saved()) {
        growth_stages.Save();
        EVOENGINE_LOG(growth_stages.GetTypeName() + " autosaved!");
      }
    }
  }
  if (!growth_stages.Saved()) {
    ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(255, 0, 0, 255));
    ImGui::Text("[Changed unsaved!]");
    ImGui::PopStyleColor();
  }
  bool changed = false;
  EditorFileDialogs::OpenFile(
      "Import CSV", "CSV", {".csv", ".CSV"},
      [&growth_stages, &changed](const std::filesystem::path& path) {
        changed = growth_stages.ImportCsv(path);
      },
      false);
  auto& state_modes = ui_state_modes;
  if (ImGui::Combo("Mode", &state_mode, state_modes, IM_ARRAYSIZE(state_modes))) {
    changed = false;
  }
  if (ImGui::TreeNodeEx("States", ImGuiTreeNodeFlags_DefaultOpen)) {
    const float start_time = sorghum_growth_stages.empty() ? 1.0f : sorghum_growth_stages.begin()->first;
    if (start_time >= 0.01f) {
      if (ImGui::Button("New start state")) {
        changed = true;
        if (sorghum_growth_stages.empty()) {
          growth_stages.Add(0.0f, SorghumState());
        } else {
          growth_stages.Add(0.0f, sorghum_growth_stages.begin()->second);
        }
      }
    }

    float previous_time = 0.0f;
    int state_index = 1;
    for (auto it = sorghum_growth_stages.begin(); it != sorghum_growth_stages.end(); ++it) {
      if (ImGui::TreeNodeEx(("State " + std::to_string(state_index) + ": " + it->second.name).c_str())) {
        const std::string tag = "##SorghumState" + std::to_string(state_index);
        if (ImGui::BeginPopupContextItem(tag.c_str())) {
          if (ImGui::BeginMenu(("Rename" + tag).c_str())) {
            auto& new_name = ui_new_name;
            ImGui::InputText(("New name" + tag).c_str(), new_name, 256);
            if (ImGui::Button(("Confirm" + tag).c_str())) {
              it->second.name = new_name;
              memset(new_name, 0, 256);
            }
            ImGui::EndMenu();
          }
          ImGui::EndPopup();
        }
        if (state_index != 1) {
          if (ImGui::Button("Copy prev leaves shape")) {
            for (int i = 0; i < (it - 1)->second.leaves.size() && i < it->second.leaves.size(); i++) {
              it->second.leaves[i].CopyShape((it - 1)->second.leaves[i]);
              it->second.saved = false;
              changed = true;
            }
          }
          ImGui::SameLine();
          if (ImGui::Button("Duplicate prev")) {
            it->second = (it - 1)->second;
            it->second.saved = false;
            for (auto& leaf_state : it->second.leaves)
              leaf_state.saved = false;
            it->second.panicle.saved = false;
            it->second.stem.saved = false;
            changed = true;
          }
        }
        if (it != (--sorghum_growth_stages.end())) {
          auto tit = it;
          ++tit;
          const float next_time = tit->first - 0.01f;
          float current_time = it->first;
          if (ImGui::InputFloat("Time", &current_time)) {
            it->first = glm::clamp(current_time, previous_time, next_time);
            changed = true;
          }

        } else {
          float current_time = it->first;
          if (ImGui::InputFloat("Time", &current_time)) {
            it->first = glm::clamp(current_time, previous_time, 99999.0f);
            changed = true;
          }
        }

        if (DrawSorghumStateGui(it->second, state_mode)) {
          changed = true;
        }

        ImGui::TreePop();
      }
      previous_time = it->first + 0.01f;
      state_index++;
    }

    if (!sorghum_growth_stages.empty()) {
      if (ImGui::Button("New end state")) {
        changed = true;
        const float end_time = (--sorghum_growth_stages.end())->first;
        growth_stages.Add(end_time + 0.01f, (--sorghum_growth_stages.end())->second);
      }
      ImGui::SameLine();
      if (ImGui::Button("Remove end state")) {
        changed = true;
        sorghum_growth_stages.erase(--sorghum_growth_stages.end());
      }
    }
    ImGui::TreePop();
  }
  /*
  if (ImGui::TreeNode("Import state...")) {
          auto& seed = ui_seed;
          ImGui::DragInt("Using seed", &seed);
          auto& descriptor = ui_descriptor;
          editorLayer->DragAndDropButton<SorghumDescriptorGenerator>(
                  descriptor, "Drag SPD here to add end state");
          auto temp = descriptor.Get<SorghumDescriptorGenerator>();
          if (temp) {
                  float endTime =
                          sorghum_growth_stages.empty() ? -0.01f : (--sorghum_growth_stages.end())->first;
                  SorghumState stage;
                  temp->Apply(stage, seed);
                  Add(endTime + 0.01f, stage);
                  descriptor.Clear();
                  changed = true;
          }
          ImGui::TreePop();
  }
  */
  return changed;
}
