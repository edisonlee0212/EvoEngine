//
// Created by lllll on 1/8/2022.
//
#include "SorghumGrowthStages.hpp"
#include "Application.hpp"
#include "EditorLayer.hpp"
#include "Scene.hpp"
#include "Sorghum.hpp"
#include "SorghumGrowthStages.hpp"
#include "SorghumLayer.hpp"
#include "Times.hpp"
#include "Utilities.hpp"
#include "rapidcsv.h"
using namespace digital_agriculture_plugin;

void SorghumGrowthStages::Apply(const std::shared_ptr<SorghumDescriptor>& target_sorghum_descriptor,
                                const float time) const {
  if (sorghum_growth_stages.empty())
    return;
  const auto actual_time = glm::clamp(time, 0.0f, 99999.0f);
  float previous_time = sorghum_growth_stages.begin()->first;
  SorghumGrowthStagePair state_pair;
  state_pair.left_stage = sorghum_growth_stages.begin()->second;
  state_pair.right_stage = state_pair.left_stage;

  if (actual_time < previous_time) {
    // Get from zero state to first state.
    state_pair.Apply(target_sorghum_descriptor, 0.0f);
    return;
  }

  float a = 0.0f;
  for (auto it = (++sorghum_growth_stages.begin()); it != sorghum_growth_stages.end(); ++it) {
    state_pair.left_stage = state_pair.right_stage;
    state_pair.right_stage = it->second;
    if (it->first > actual_time) {
      a = (actual_time - previous_time) / (it->first - previous_time);
      break;
    }
    previous_time = it->first;
  }
  state_pair.Apply(target_sorghum_descriptor, a);
}

void SorghumGrowthStagePair::Apply(const std::shared_ptr<SorghumDescriptor>& target_sorghum_descriptor,
                                   const float a) const {
  ApplyPanicle(target_sorghum_descriptor, a);
  ApplyStem(target_sorghum_descriptor, a);
  ApplyLeaves(target_sorghum_descriptor, a);
}

void SorghumGrowthStagePair::ApplyLeaves(const std::shared_ptr<SorghumDescriptor>& target_descriptor,
                                         const float a) const {
  const auto leaf_size = GetLeafSize(a);
  target_descriptor->leaves.resize(leaf_size);
  for (int i = 0; i < leaf_size; i++) {
    ApplyLeaf(target_descriptor, a, i);
  }
}

void SorghumGrowthStagePair::ApplyLeaf(const std::shared_ptr<SorghumDescriptor>& target_descriptor, float a,
                                       int leaf_index) const {
  constexpr auto up_direction = glm::vec3(0, 1, 0);
  auto front_direction = glm::vec3(0, 0, -1);
  front_direction = glm::rotate(front_direction, glm::radians(glm::linearRand(0.0f, 360.0f)), up_direction);
  glm::vec3 stem_front = GetStemDirection(a);
  const float stem_length = GetStemLength(a);
  const auto sorghum_layer = ApplicationContext::Get().GetLayer<SorghumLayer>();

  const float preserved_a = a;
  SorghumLeafState actual_left, actual_right;
  LeafStateHelper(actual_left, actual_right, a, leaf_index);

  float stem_width = glm::mix(left_stage.stem.width_along_stem.GetValue(actual_left.starting_point),
                              right_stage.stem.width_along_stem.GetValue(actual_right.starting_point), preserved_a);

  auto& leaf_state = target_descriptor->leaves[leaf_index];
  leaf_state.spline.segments.clear();
  leaf_state.index = leaf_index;

  float starting_point_ratio = glm::mix(actual_left.starting_point, actual_right.starting_point, a);
  float leaf_length = glm::mix(actual_left.length, actual_right.length, a);
  if (leaf_length == 0.0f)
    return;

  float branching_angle = glm::mix(actual_left.branching_angle, actual_right.branching_angle, a);
  float roll_angle = glm::mod(glm::mix(actual_left.roll_angle, actual_right.roll_angle, a), 360.0f);

  // Build nodes...

  float back_track_ratio = 0.05f;
  if (starting_point_ratio < back_track_ratio)
    back_track_ratio = starting_point_ratio;

  glm::vec3 leaf_left = glm::normalize(glm::rotate(glm::vec3(0, 0, -1), glm::radians(roll_angle), glm::vec3(0, 1, 0)));
  auto leaf_up = glm::normalize(glm::cross(stem_front, leaf_left));
  glm::vec3 stem_offset = stem_width * -leaf_up;

  auto direction = glm::rotate(glm::vec3(0, 1, 0), glm::radians(branching_angle), leaf_left);
  float sheath_ratio = starting_point_ratio - back_track_ratio;

  if (sheath_ratio > 0) {
    int root_to_sheath_node_count =
        static_cast<int>(glm::min(2.0f, stem_length * sheath_ratio / sorghum_layer->vertical_subdivision_length));
    for (int i = 0; i < root_to_sheath_node_count; i++) {
      float factor = static_cast<float>(i) / root_to_sheath_node_count;
      float current_root_to_sheath_point = glm::mix(0.f, sheath_ratio, factor);

      const auto up = glm::normalize(glm::cross(stem_front, leaf_left));
      leaf_state.spline.segments.emplace_back(
          glm::normalize(stem_front) * current_root_to_sheath_point * stem_length + stem_offset, up, stem_front,
          stem_width, 180.f, 0.0f, 0.0f);
    }
  }

  int sheath_node_count =
      static_cast<int>(glm::max(2.0f, stem_length * back_track_ratio / sorghum_layer->vertical_subdivision_length));
  for (int i = 0; i <= sheath_node_count; i++) {
    float factor = static_cast<float>(i) / sheath_node_count;
    float current_sheath_point =
        glm::mix(sheath_ratio, starting_point_ratio,
                 factor);  // sheathRatio + static_cast<float>(i) / sheathNodeCount * backTrackRatio;
    glm::vec3 actual_direction = glm::normalize(glm::mix(stem_front, direction, factor));

    const auto up = glm::normalize(glm::cross(actual_direction, leaf_left));
    leaf_state.spline.segments.emplace_back(
        glm::normalize(stem_front) * current_sheath_point * stem_length + stem_offset, up, actual_direction,
        stem_width + 0.002f * static_cast<float>(i) / sheath_node_count,
        180.0f - 90.0f * static_cast<float>(i) / sheath_node_count, 0.0f, 0.0f);
  }

  int node_amount = static_cast<int>(glm::max(4.0f, leaf_length / sorghum_layer->vertical_subdivision_length));
  float unit_length = leaf_length / node_amount;

  int node_to_full_expand = static_cast<int>(0.1f * leaf_length / sorghum_layer->vertical_subdivision_length);

  auto period_start = glm::mix(actual_left.waviness_period_start, actual_right.waviness_period_start, a);
  const float waviness_frequency = glm::mix(actual_left.waviness_frequency, actual_right.waviness_frequency, a);
  glm::vec3 node_position = stem_front * starting_point_ratio * stem_length + stem_offset;
  for (int i = 1; i <= node_amount; i++) {
    const float factor = static_cast<float>(i) / node_amount;
    glm::vec3 current_direction;

    float rotate_angle =
        glm::mix(actual_left.bending_along_leaf.GetValue(factor), actual_right.bending_along_leaf.GetValue(factor), a);
    current_direction = glm::rotate(direction, glm::radians(rotate_angle), leaf_left);
    node_position += current_direction * unit_length;

    float expand_angle =
        glm::mix(actual_left.curling_along_leaf.GetValue(factor), actual_right.curling_along_leaf.GetValue(factor), a);

    float collar_factor = glm::min(1.0f, static_cast<float>(i) / node_to_full_expand);

    float waviness = glm::mix(actual_left.waviness_along_leaf.GetValue(factor),
                              actual_right.waviness_along_leaf.GetValue(factor), a);
    period_start += glm::vec2(waviness_frequency);

    float width = glm::mix(
        stem_width + 0.002f,
        glm::mix(actual_left.width_along_leaf.GetValue(factor), actual_right.width_along_leaf.GetValue(factor), a),
        collar_factor);
    float angle = 90.0f - (90.0f - expand_angle) * glm::pow(collar_factor, 2.0f);

    const auto up = glm::normalize(glm::cross(current_direction, leaf_left));
    leaf_state.spline.segments.emplace_back(node_position, up, current_direction, width, angle,
                                            waviness * glm::simplex(glm::vec2(period_start.x, 0.f)),
                                            waviness * glm::simplex(glm::vec2(0.f, period_start.y)));
  }
}

void SorghumGrowthStagePair::LeafStateHelper(SorghumLeafState& left, SorghumLeafState& right, float& a,
                                             const int leaf_index) const {
  const int previous_leaf_size = left_stage.leaves.size();
  const int next_leaf_size = right_stage.leaves.size();
  if (leaf_index < previous_leaf_size) {
    left = left_stage.leaves[leaf_index];
    if (left.dead)
      left.length = 0;
    if (leaf_index < next_leaf_size) {
      if (right_stage.leaves[leaf_index].dead || right_stage.leaves[leaf_index].length == 0)
        right = left;
      else {
        right = right_stage.leaves[leaf_index];
      }
    } else {
      right = left_stage.leaves[leaf_index];
    }
    return;
  }

  const int completed_leaf_size =
      static_cast<int>(left_stage.leaves.size()) +
      static_cast<int>(glm::floor(static_cast<float>(right_stage.leaves.size() - left_stage.leaves.size()) * a));
  a = glm::clamp(a * (next_leaf_size - previous_leaf_size) - (completed_leaf_size - previous_leaf_size), 0.0f, 1.0f);
  left = right = right_stage.leaves[leaf_index];
  if (leaf_index >= completed_leaf_size) {
    left.length = 0.0f;
    left.width_along_leaf.min_value = left.width_along_leaf.max_value = 0.0f;
    left.waviness_along_leaf.min_value = left.waviness_along_leaf.max_value = 0.0f;
    for (auto& i : left.spline.curves) {
      i.p0 = i.p1 = i.p2 = i.p3 = right.spline.EvaluatePointFromCurves(0.0f);
    }
  } else {
    left = right;
  }
}

int SorghumGrowthStagePair::GetLeafSize(const float a) const {
  if (left_stage.leaves.size() <= right_stage.leaves.size()) {
    return static_cast<int>(left_stage.leaves.size()) +
           static_cast<int>(glm::ceil(static_cast<float>(right_stage.leaves.size() - left_stage.leaves.size()) * a));
  }
  return left_stage.leaves.size();
}
float SorghumGrowthStagePair::GetStemLength(const float a) const {
  float left_length, right_length;
  switch (static_cast<StateMode>(state_mode)) {
    case StateMode::Default:
      left_length = left_stage.stem.length;
      right_length = right_stage.stem.length;
      break;
    case StateMode::CubicBezier:
      if (!left_stage.stem.spline.curves.empty()) {
        left_length = glm::distance(left_stage.stem.spline.curves.front().p0, left_stage.stem.spline.curves.back().p3);
      } else {
        left_length = 0.0f;
      }
      if (!right_stage.stem.spline.curves.empty()) {
        right_length =
            glm::distance(right_stage.stem.spline.curves.front().p0, right_stage.stem.spline.curves.back().p3);
      } else {
        right_length = 0.0f;
      }
      break;
  }
  return glm::mix(left_length, right_length, a);
}
glm::vec3 SorghumGrowthStagePair::GetStemDirection(const float a) const {
  glm::vec3 left_dir, right_dir;
  switch (static_cast<StateMode>(state_mode)) {
    case StateMode::Default:
      left_dir = glm::normalize(left_stage.stem.direction);
      right_dir = glm::normalize(right_stage.stem.direction);
      break;
    case StateMode::CubicBezier:
      if (!left_stage.stem.spline.curves.empty()) {
        left_dir = glm::vec3(0.0f, 1.0f, 0.0f);
      } else {
        left_dir = glm::vec3(0.0f, 1.0f, 0.0f);
      }
      if (!right_stage.stem.spline.curves.empty()) {
        right_dir = glm::vec3(0.0f, 1.0f, 0.0f);
      } else {
        right_dir = glm::vec3(0.0f, 1.0f, 0.0f);
      }
      break;
  }

  return glm::normalize(glm::mix(left_dir, right_dir, a));
}
glm::vec3 SorghumGrowthStagePair::GetStemPoint(const float a, const float point) const {
  glm::vec3 left_point, right_point;
  switch (static_cast<StateMode>(state_mode)) {
    case StateMode::Default:
      left_point = glm::normalize(left_stage.stem.direction) * point * left_stage.stem.length;
      right_point = glm::normalize(right_stage.stem.direction) * point * right_stage.stem.length;
      break;
    case StateMode::CubicBezier:
      if (!left_stage.stem.spline.curves.empty()) {
        left_point = left_stage.stem.spline.EvaluatePointFromCurves(point);
      } else {
        left_point = glm::vec3(0.0f, 0.0f, 0.0f);
      }
      if (!right_stage.stem.spline.curves.empty()) {
        right_point = right_stage.stem.spline.EvaluatePointFromCurves(point);
      } else {
        right_point = glm::vec3(0.0f, 0.0f, 0.0f);
      }
      break;
  }

  return glm::mix(left_point, right_point, a);
}

void SorghumGrowthStagePair::ApplyPanicle(const std::shared_ptr<SorghumDescriptor>& target_state, const float a) const {
  target_state->panicle.panicle_size = glm::mix(left_stage.panicle.panicle_size, right_stage.panicle.panicle_size, a);
  target_state->panicle.seed_amount = glm::mix(left_stage.panicle.seed_amount, right_stage.panicle.seed_amount, a);
  target_state->panicle.seed_radius = glm::mix(left_stage.panicle.seed_radius, right_stage.panicle.seed_radius, a);
}

void SorghumGrowthStagePair::ApplyStem(const std::shared_ptr<SorghumDescriptor>& target_state, const float a) const {
  constexpr auto up_direction = glm::vec3(0, 1, 0);
  auto front_direction = glm::vec3(0, 0, -1);
  front_direction = glm::rotate(front_direction, glm::radians(glm::linearRand(0.0f, 360.0f)), up_direction);
  glm::vec3 stem_front = GetStemDirection(a);
  const float stem_length = GetStemLength(a);
  const auto sorghum_layer = ApplicationContext::Get().GetLayer<SorghumLayer>();
  const int stem_node_amount =
      static_cast<int>(glm::max(4.0f, stem_length / sorghum_layer->vertical_subdivision_length));
  const float stem_unit_length = stem_length / stem_node_amount;
  target_state->stem.spline.segments.clear();
  const glm::vec3 stem_left =
      glm::normalize(glm::rotate(glm::vec3(1, 0, 0), glm::radians(glm::linearRand(0.0f, 0.0f)), stem_front));
  for (int i = 0; i <= stem_node_amount; i++) {
    float stem_width =
        glm::mix(left_stage.stem.width_along_stem.GetValue(static_cast<float>(i) / stem_node_amount),
                 right_stage.stem.width_along_stem.GetValue(static_cast<float>(i) / stem_node_amount), a);
    glm::vec3 stem_node_position;
    stem_node_position = stem_front * stem_unit_length * static_cast<float>(i);

    const auto up = glm::normalize(glm::cross(stem_front, stem_left));
    target_state->stem.spline.segments.emplace_back(stem_node_position, up, stem_front, stem_width, 180.f, 0.0f, 0.0f);
  }
}

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
      last_auto_save_time = ApplicationContext::Get().GetTimes().Now();
    } else if (last_auto_save_time + auto_save_interval < ApplicationContext::Get().GetTimes().Now()) {
      last_auto_save_time = ApplicationContext::Get().GetTimes().Now();
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
          Add(0.0f, SorghumState());
        } else {
          Add(0.0f, sorghum_growth_stages.begin()->second);
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

        if (it->second.OnInspectImpl(state_mode)) {
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
      SorghumState state;
      state.Deserialize(in_state);
      sorghum_growth_stages.emplace_back(in_state["Time"].as<float>(), state);
    }
  }
}

std::shared_ptr<Texture2D> SorghumGrowthStages::GenerateThumbnailTexture() {
  static std::shared_ptr<Texture2D> thumbnail;
  if (!thumbnail) {
    thumbnail = AssetManager::CreateTemporaryAsset<Texture2D>();
    thumbnail->Import(std::filesystem::absolute(std::filesystem::path("./DigitalAgricultureResources") /
                                                "Icons/SorghumGrowthStages.png"));
  }
  return thumbnail;
}

Entity SorghumGrowthStages::CreateEntity(const float time) const {
  const auto scene = ApplicationContext::Get().GetActiveScene();
  const auto entity = scene->CreateEntity(GetTitle());
  const auto sorghum = scene->GetOrSetPrivateComponent<Sorghum>(entity).lock();
  const auto sorghum_state = AssetManager::CreateTemporaryAsset<SorghumDescriptor>();
  Apply(sorghum_state, time);
  sorghum->sorghum_descriptor = sorghum_state;
  sorghum->sorghum_growth_stages = GetSelf();
  sorghum->GenerateGeometryEntities(SorghumMeshGeneratorSettings{});
  return entity;
}

void SorghumGrowthStages::Add(float time, const SorghumState& state) {
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

void SorghumGrowthStages::ResetTime(const float previous_time, const float new_time) {
  for (auto& i : sorghum_growth_stages) {
    if (i.first == previous_time) {
      i.first = new_time;
      return;
    }
  }
  EVOENGINE_ERROR("Failed: State at previous time not exists!");
}
void SorghumGrowthStages::Remove(const float time) {
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
      const auto current_leaf_index = static_cast<int>(leaf_index[row]);
      if (column_indices[time_point].second < current_leaf_index)
        column_indices[time_point].second = current_leaf_index;
    }

    sorghum_growth_stages.resize(current_index);
    for (int row = 0; row < time_points.size(); row++) {
      int state_index = column_indices.at(time_points[row]).first;
      auto& state_pair = sorghum_growth_stages[state_index];
      auto& state = state_pair.second;
      if (state.leaves.empty()) {
        state_pair.first = static_cast<float>(state_index);
        state.name = time_points[row];
        state.leaves.resize(column_indices.at(time_points[row]).second);
        for (auto& leaf : state.leaves)
          leaf.dead = true;
        state.stem.length = stem_heights[row] / 100.0f;
        state.stem.width_along_stem.min_value = 0.0f;
        state.stem.width_along_stem.max_value = stem_width[row] * 2.0f;
        state.panicle.panicle_size.x = state.panicle.panicle_size.z = panicle_width[row] / 100.0f;
        state.panicle.panicle_size.y = panicle_length[row] / 100.0f;
        state.panicle.seed_amount = static_cast<int>(state.panicle.panicle_size.x * state.panicle.panicle_size.y *
                                                     state.panicle.panicle_size.z / 0.001f);
      }
      const auto current_leaf_index = static_cast<int>(leaf_index[row]);
      auto& leaf = state.leaves[current_leaf_index - 1];
      leaf.index = current_leaf_index - 1;
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
