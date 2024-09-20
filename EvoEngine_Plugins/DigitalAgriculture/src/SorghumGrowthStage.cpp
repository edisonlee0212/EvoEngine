//
// Created by lllll on 1/8/2022.
//
#include "Application.hpp"
#include "EditorLayer.hpp"
#include "Scene.hpp"
#include "Sorghum.hpp"
#include "SorghumGrowthStages.hpp"
#include "SorghumLayer.hpp"
#include "SorghumDescriptorGenerator.hpp"
#include "Times.hpp"
#include "Utilities.hpp"
#include "rapidcsv.h"
using namespace digital_agriculture_plugin;

bool SorghumGrowthStages::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  if (ImGui::Button("Instantiate")) {
    auto entity = CreateEntity();
  }
  static bool auto_save = false;
  ImGui::Checkbox("Auto save", &auto_save);
  if (!auto_save) {
    ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(255, 0, 0, 255));
    ImGui::Text("[Auto save disabled!]");
    ImGui::PopStyleColor();
  } else {
    static double last_auto_save_time = 0;
    static float auto_save_interval = 10;
    if (ImGui::TreeNodeEx("Auto save settings")) {
      if (ImGui::DragFloat("Time interval", &auto_save_interval, 1.0f, 2.0f, 300.0f)) {
        auto_save_interval = glm::clamp(auto_save_interval, 5.0f, 300.0f);
      }
      ImGui::TreePop();
    }
    if (last_auto_save_time == 0) {
      last_auto_save_time = Times::Now();
    } else if (last_auto_save_time + auto_save_interval < Times::Now()) {
      last_auto_save_time = Times::Now();
      if (!saved_) {
        Save();
        EVOENGINE_LOG(GetTypeName() + " autosaved!");
      }
    }
  }
  if (!saved_) {
    ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(255, 0, 0, 255));
    ImGui::Text("[Changed unsaved!]");
    ImGui::PopStyleColor();
  }
  bool changed = false;
  FileUtils::OpenFile(
      "Import CSV", "CSV", {".csv", ".CSV"},
      [&](const std::filesystem::path& path) {
        changed = ImportCsv(path);
      },
      false);
  static const char* state_modes[]{"Default", "Cubic-Bezier"};
  if (ImGui::Combo("Mode", &state_mode, state_modes, IM_ARRAYSIZE(state_modes))) {
    changed = false;
  }
  if (ImGui::TreeNodeEx("States", ImGuiTreeNodeFlags_DefaultOpen)) {
    const float start_time = sorghum_growth_stages.empty() ? 1.0f : sorghum_growth_stages.begin()->first;
    if (start_time >= 0.01f) {
      if (ImGui::Button("New start state")) {
        changed = true;
        if (sorghum_growth_stages.empty()) {
          Add(0.0f, SorghumGrowthStage());
        } else {
          Add(0.0f, sorghum_growth_stages.begin()->second);
        }
      }
    }

    float previous_time = 0.0f;
    int state_index = 1;
    for (auto it = sorghum_growth_stages.begin(); it != sorghum_growth_stages.end(); ++it) {
      if (ImGui::TreeNodeEx(("State " + std::to_string(state_index) + ": " + it->second.name).c_str())) {
        const std::string tag = "##SorghumGrowthStage" + std::to_string(state_index);
        if (ImGui::BeginPopupContextItem(tag.c_str())) {
          if (ImGui::BeginMenu(("Rename" + tag).c_str())) {
            static char new_name[256];
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

        if (it->second.OnInspect(state_mode)) {
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
        Add(end_time + 0.01f, (--sorghum_growth_stages.end())->second);
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
          static int seed = 0;
          ImGui::DragInt("Using seed", &seed);
          static AssetRef descriptor;
          editorLayer->DragAndDropButton<SorghumDescriptorGenerator>(
                  descriptor, "Drag SPD here to add end state");
          auto temp = descriptor.Get<SorghumDescriptorGenerator>();
          if (temp) {
                  float endTime =
                          sorghum_growth_stages.empty() ? -0.01f : (--sorghum_growth_stages.end())->first;
                  SorghumGrowthStage stage;
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

void SorghumGrowthStages::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "state_mode" << YAML::Value << state_mode;
  out << YAML::Key << "sorghum_growth_stages" << YAML::Value << YAML::BeginSeq;
  for (auto& state : sorghum_growth_stages) {
    out << YAML::BeginMap;
    out << YAML::Key << "Time" << YAML::Value << state.first;
    state.second.Serialize(out);
    out << YAML::EndMap;
  }
  out << YAML::EndSeq;
}

void SorghumGrowthStages::Deserialize(const YAML::Node& in) {
  if (in["state_mode"])
    state_mode = in["state_mode"].as<int>();
  if (in["sorghum_growth_stages"]) {
    sorghum_growth_stages.clear();
    for (const auto& in_state : in["sorghum_growth_stages"]) {
      SorghumGrowthStage state;
      state.Deserialize(in_state);
      sorghum_growth_stages.emplace_back(in_state["Time"].as<float>(), state);
    }
  }
}

Entity SorghumGrowthStages::CreateEntity(const float time) const {
  const auto scene = Application::GetActiveScene();
  const auto entity = scene->CreateEntity(GetTitle());
  const auto sorghum = scene->GetOrSetPrivateComponent<Sorghum>(entity).lock();
  const auto sorghum_state = ProjectManager::CreateTemporaryAsset<SorghumDescriptor>();
  Apply(sorghum_state, time);
  sorghum->sorghum_descriptor = sorghum_state;
  sorghum->sorghum_growth_stages = GetSelf();
  sorghum->GenerateGeometryEntities(SorghumMeshGeneratorSettings{});
  return entity;
}

void SorghumGrowthStages::Add(float time, const SorghumGrowthStage& state) {
  for (auto it = sorghum_growth_stages.begin(); it != sorghum_growth_stages.end(); ++it) {
    if (it->first == time) {
      it->second = state;
      return;
    }
    if (it->first > time) {
      sorghum_growth_stages.insert(it, {time, state});
      return;
    }
  }
  sorghum_growth_stages.emplace_back(time, state);
  sorghum_growth_stages.back().second.name = "Unnamed";
}

void SorghumGrowthStages::ResetTime(float previous_time, float new_time) {
  for (auto& i : sorghum_growth_stages) {
    if (i.first == previous_time) {
      i.first = new_time;
      return;
    }
  }
  EVOENGINE_ERROR("Failed: State at previous time not exists!");
}
void SorghumGrowthStages::Remove(float time) {
  for (auto it = sorghum_growth_stages.begin(); it != sorghum_growth_stages.end(); ++it) {
    if (it->first == time) {
      sorghum_growth_stages.erase(it);
      return;
    }
  }
}
float SorghumGrowthStages::GetCurrentStartTime() const {
  if (sorghum_growth_stages.empty()) {
    return 0.0f;
  }
  return sorghum_growth_stages.begin()->first;
}
float SorghumGrowthStages::GetCurrentEndTime() const {
  if (sorghum_growth_stages.empty()) {
    return 0.0f;
  }
  return (--sorghum_growth_stages.end())->first;
}

bool SorghumGrowthStages::ImportCsv(const std::filesystem::path& file_path) {
  try {
    rapidcsv::Document doc(file_path.string());
    std::vector<std::string> time_points = doc.GetColumn<std::string>("Time Point");
    std::vector<float> stem_heights = doc.GetColumn<float>("Stem Height");
    std::vector<float> stem_width = doc.GetColumn<float>("Stem Width");
    std::vector<float> leaf_index = doc.GetColumn<float>("Leaf Number");
    std::vector<float> leaf_length = doc.GetColumn<float>("Leaf Length");
    std::vector<float> leaf_width = doc.GetColumn<float>("Leaf Width");
    std::vector<float> leaf_height = doc.GetColumn<float>("Leaf Height");
    std::vector<float> starting_point = doc.GetColumn<float>("Start Point");
    std::vector<float> branching_angle = doc.GetColumn<float>("Branching Angle");
    std::vector<float> panicle_length = doc.GetColumn<float>("Panicle Height");
    std::vector<float> panicle_width = doc.GetColumn<float>("Panicle Width");

    sorghum_growth_stages.clear();

    std::map<std::string, std::pair<int, int>> column_indices;
    int current_index = 0;
    for (int row = 0; row < time_points.size(); row++) {
      auto& time_point = time_points[row];
      if (column_indices.find(time_point) == column_indices.end()) {
        column_indices[time_point].first = current_index;
        current_index++;
      }
      if (column_indices[time_point].second < leaf_index[row])
        column_indices[time_point].second = leaf_index[row];
    }

    sorghum_growth_stages.resize(current_index);
    for (int row = 0; row < time_points.size(); row++) {
      int state_index = column_indices.at(time_points[row]).first;
      auto& state_pair = sorghum_growth_stages[state_index];
      auto& state = state_pair.second;
      if (state.leaves.empty()) {
        state_pair.first = state_index;
        state.name = time_points[row];
        state.leaves.resize(column_indices.at(time_points[row]).second);
        for (auto& leaf : state.leaves)
          leaf.dead = true;
        state.stem.length = stem_heights[row] / 100.0f;
        state.stem.width_along_stem.min_value = 0.0f;
        state.stem.width_along_stem.max_value = stem_width[row] * 2.0f;
        state.panicle.panicle_size.x = state.panicle.panicle_size.z = panicle_width[row] / 100.0f;
        state.panicle.panicle_size.y = panicle_length[row] / 100.0f;
        state.panicle.seed_amount =
            state.panicle.panicle_size.x * state.panicle.panicle_size.y * state.panicle.panicle_size.z / 0.001f;
      }
      auto& leaf = state.leaves[leaf_index[row] - 1];
      leaf.index = leaf_index[row] - 1;
      leaf.length = leaf_length[row] / 100.0f;
      if (leaf.length == 0)
        leaf.dead = true;
      else {
        leaf.dead = false;
        leaf.roll_angle = leaf.index % 2 * 180.0f;
        leaf.width_along_leaf.max_value = leaf_width[row] / 100.0f;
        leaf.starting_point = leaf_height[row] / stem_heights[row];
        leaf.branching_angle = branching_angle[row];
      }
    }

    for (auto& sorghum_state : sorghum_growth_stages) {
      sorghum_state.second.saved = false;
      int current_leaf_index = 0;
      for (auto& leaf_state : sorghum_state.second.leaves) {
        leaf_state.saved = false;
        leaf_state.index = current_leaf_index;
        current_leaf_index++;
      }
      sorghum_state.second.stem.saved = false;
      sorghum_state.second.panicle.saved = false;
    }
    saved_ = false;
  } catch (const std::exception& e) {
    return false;
  }
  return true;
}

glm::vec3 SorghumStemGrowthStage::GetPoint(float point) const {
  return direction * point * length;
}