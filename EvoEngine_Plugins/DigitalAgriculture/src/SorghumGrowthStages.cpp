//
// Created by lllll on 1/8/2022.
//
#include "SorghumGrowthStages.hpp"
#include "Application.hpp"
#include "EditorLayer.hpp"
#include "Scene.hpp"
#include "SorghumGrowthStages.hpp"
#include "SorghumLayer.hpp"
#include "Times.hpp"
#include "Utilities.hpp"
#include "rapidcsv.h"
using namespace digital_agriculture_plugin;

void SorghumGrowthStages::Apply(const std::shared_ptr<SorghumDescriptor>& target_state, const float time) const {
  if (sorghum_growth_stages.empty())
    return;
  const auto actual_time = glm::clamp(time, 0.0f, 99999.0f);
  float previous_time = sorghum_growth_stages.begin()->first;
  SorghumGrowthStagePair state_pair;
  state_pair.left_stage = sorghum_growth_stages.begin()->second;
  state_pair.right_stage = state_pair.left_stage;

  if (actual_time < previous_time) {
    // Get from zero state to first state.
    state_pair.Apply(target_state, 0.0f);
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
  state_pair.Apply(target_state, a);
}
void SorghumGrowthStagePair::Apply(const std::shared_ptr<SorghumDescriptor>& target_state, const float a) const {
  ApplyPanicle(target_state, a);
  ApplyStem(target_state, a);
  ApplyLeaves(target_state, a);
}

void SorghumGrowthStagePair::ApplyLeaves(const std::shared_ptr<SorghumDescriptor>& target_state, const float a) const {
  const auto leaf_size = GetLeafSize(a);
  target_state->leaves.resize(leaf_size);
  for (int i = 0; i < leaf_size; i++) {
    ApplyLeaf(target_state, a, i);
  }
}

void SorghumGrowthStagePair::ApplyLeaf(const std::shared_ptr<SorghumDescriptor>& target_state, float a,
                                       int leaf_index) const {
  constexpr auto up_direction = glm::vec3(0, 1, 0);
  auto front_direction = glm::vec3(0, 0, -1);
  front_direction = glm::rotate(front_direction, glm::radians(glm::linearRand(0.0f, 360.0f)), up_direction);
  glm::vec3 stem_front = GetStemDirection(a);
  const float stem_length = GetStemLength(a);
  const auto sorghum_layer = Application::GetLayer<SorghumLayer>();

  const float preserved_a = a;
  SorghumLeafGrowthStage actual_left, actual_right;
  LeafStateHelper(actual_left, actual_right, a, leaf_index);

  float stem_width = glm::mix(left_stage.stem.width_along_stem.GetValue(actual_left.starting_point),
                              right_stage.stem.width_along_stem.GetValue(actual_right.starting_point), preserved_a);

  auto& leaf_state = target_state->leaves[leaf_index];
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
        glm::min(2.0f, stem_length * sheath_ratio / sorghum_layer->vertical_subdivision_length);
    for (int i = 0; i < root_to_sheath_node_count; i++) {
      float factor = static_cast<float>(i) / root_to_sheath_node_count;
      float current_root_to_sheath_point = glm::mix(0.f, sheath_ratio, factor);

      const auto up = glm::normalize(glm::cross(stem_front, leaf_left));
      leaf_state.spline.segments.emplace_back(
          glm::normalize(stem_front) * current_root_to_sheath_point * stem_length + stem_offset, up, stem_front,
          stem_width, 180.f, 0, 0);
    }
  }

  int sheath_node_count = glm::max(2.0f, stem_length * back_track_ratio / sorghum_layer->vertical_subdivision_length);
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
        180.0f - 90.0f * static_cast<float>(i) / sheath_node_count, 0, 0);
  }

  int node_amount = glm::max(4.0f, leaf_length / sorghum_layer->vertical_subdivision_length);
  float unit_length = leaf_length / node_amount;

  int node_to_full_expand = 0.1f * leaf_length / sorghum_layer->vertical_subdivision_length;

  float height_offset = glm::linearRand(0.f, 100.f);
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
    height_offset += waviness_frequency;

    float width = glm::mix(
        stem_width + 0.002f,
        glm::mix(actual_left.width_along_leaf.GetValue(factor), actual_right.width_along_leaf.GetValue(factor), a),
        collar_factor);
    float angle = 90.0f - (90.0f - expand_angle) * glm::pow(collar_factor, 2.0f);

    const auto up = glm::normalize(glm::cross(current_direction, leaf_left));
    leaf_state.spline.segments.emplace_back(node_position, up, current_direction, width, angle,
                                              waviness * glm::simplex(glm::vec2(height_offset, 0.f)),
                                              waviness * glm::simplex(glm::vec2(0.f, height_offset)));
  }
}

void SorghumGrowthStagePair::LeafStateHelper(SorghumLeafGrowthStage& left, SorghumLeafGrowthStage& right, float& a,
                                             int leaf_index) const {
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
      left_stage.leaves.size() + glm::floor((right_stage.leaves.size() - left_stage.leaves.size()) * a);
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
    return left_stage.leaves.size() + glm::ceil((right_stage.leaves.size() - left_stage.leaves.size()) * a);
  }
  return left_stage.leaves.size();
}
float SorghumGrowthStagePair::GetStemLength(float a) const {
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
glm::vec3 SorghumGrowthStagePair::GetStemDirection(float a) const {
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
glm::vec3 SorghumGrowthStagePair::GetStemPoint(float a, float point) const {
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
  target_state->panicle.panicle_size =
      glm::mix(left_stage.panicle.panicle_size, right_stage.panicle.panicle_size, a);
  target_state->panicle.seed_amount = glm::mix(left_stage.panicle.seed_amount, right_stage.panicle.seed_amount, a);
  target_state->panicle.seed_radius = glm::mix(left_stage.panicle.seed_radius, right_stage.panicle.seed_radius, a);
}

void SorghumGrowthStagePair::ApplyStem(const std::shared_ptr<SorghumDescriptor>& target_state, const float a) const {
  constexpr auto up_direction = glm::vec3(0, 1, 0);
  auto front_direction = glm::vec3(0, 0, -1);
  front_direction = glm::rotate(front_direction, glm::radians(glm::linearRand(0.0f, 360.0f)), up_direction);
  glm::vec3 stem_front = GetStemDirection(a);
  const float stem_length = GetStemLength(a);
  const auto sorghum_layer = Application::GetLayer<SorghumLayer>();
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
    target_state->stem.spline.segments.emplace_back(stem_node_position, up, stem_front, stem_width, 180.f, 0, 0);
  }
}

bool SorghumPanicleGrowthStage::OnInspect() {
  bool changed = false;
  if (ImGui::DragFloat("Panicle width", &panicle_size.x, 0.001f)) {
    changed = true;
    panicle_size.z = panicle_size.x;
  }
  if (ImGui::DragFloat("Panicle height", &panicle_size.y, 0.001f))
    changed = true;
  if (ImGui::DragInt("Num of seeds", &seed_amount, 1.0f))
    changed = true;
  if (ImGui::DragFloat("Seed radius", &seed_radius, 0.0001f))
    changed = true;
  if (changed)
    saved = false;
  return changed;
}
void SorghumPanicleGrowthStage::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "panicle_size" << YAML::Value << panicle_size;
  out << YAML::Key << "seed_amount" << YAML::Value << seed_amount;
  out << YAML::Key << "seed_radius" << YAML::Value << seed_radius;
}
void SorghumPanicleGrowthStage::Deserialize(const YAML::Node& in) {
  if (in["panicle_size"])
    panicle_size = in["panicle_size"].as<glm::vec3>();
  if (in["seed_amount"])
    seed_amount = in["seed_amount"].as<int>();
  if (in["seed_radius"])
    seed_radius = in["seed_radius"].as<float>();
  saved = true;
}

SorghumPanicleGrowthStage::SorghumPanicleGrowthStage() {
  panicle_size = glm::vec3(0, 0, 0);
  seed_amount = 0;
  seed_radius = 0.002f;
  saved = false;
}
void SorghumStemGrowthStage::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "direction" << YAML::Value << direction;
  width_along_stem.Save("width_along_stem", out);
  out << YAML::Key << "length" << YAML::Value << length;
  out << YAML::Key << "spline" << YAML::Value << YAML::BeginMap;
  spline.Serialize(out);
  out << YAML::EndMap;
}
void SorghumStemGrowthStage::Deserialize(const YAML::Node& in) {
  if (in["spline"]) {
    spline.Deserialize(in["spline"]);
  }

  if (in["direction"])
    direction = in["direction"].as<glm::vec3>();
  if (in["length"])
    length = in["length"].as<float>();
  width_along_stem.Load("width_along_stem", in);

  saved = true;
}
bool SorghumStemGrowthStage::OnInspect(int mode) {
  bool changed = false;
  switch ((StateMode)mode) {
    case StateMode::Default:
      // ImGui::DragFloat3("Direction", &direction.x, 0.01f);
      if (ImGui::DragFloat("Length", &length, 0.01f))
        changed = true;
      break;
    case StateMode::CubicBezier:
      if (ImGui::TreeNode("Spline")) {
        spline.OnInspect();
        ImGui::TreePop();
      }
      break;
  }
  if (width_along_stem.OnInspect("Width along stem"))
    changed = true;

  if (changed)
    saved = false;
  return changed;
}
bool SorghumLeafGrowthStage::OnInspect(int mode) {
  bool changed = false;
  if (ImGui::Checkbox("Dead", &dead)) {
    changed = true;
    if (!dead && length == 0.0f)
      length = 0.35f;
  }
  if (!dead) {
    if (ImGui::InputFloat("Starting point", &starting_point)) {
      starting_point = glm::clamp(starting_point, 0.0f, 1.0f);
      changed = true;
    }
    switch ((StateMode)mode) {
      case StateMode::Default:
        if (ImGui::TreeNodeEx("Geometric", ImGuiTreeNodeFlags_DefaultOpen)) {
          if (ImGui::DragFloat("Length", &length, 0.01f, 0.0f, 999.0f))
            changed = true;
          if (ImGui::TreeNodeEx("Angles", ImGuiTreeNodeFlags_DefaultOpen)) {
            if (ImGui::DragFloat("Roll angle", &roll_angle, 1.0f, -999.0f, 999.0f))
              changed = true;
            if (ImGui::InputFloat("Branching angle", &branching_angle)) {
              branching_angle = glm::clamp(branching_angle, 0.0f, 180.0f);
              changed = true;
            }
            ImGui::TreePop();
          }
          ImGui::TreePop();
        }
        break;
      case StateMode::CubicBezier:
        if (ImGui::TreeNodeEx("Geometric", ImGuiTreeNodeFlags_DefaultOpen)) {
          spline.OnInspect();
          ImGui::TreePop();
        }
        break;
    }

    if (ImGui::TreeNodeEx("Others")) {
      if (width_along_leaf.OnInspect("Width"))
        changed = true;
      if (curling_along_leaf.OnInspect("Rolling"))
        changed = true;

      static CurveDescriptorSettings leaf_bending = {1.0f, false, true,
                                                     "The bending of the leaf, controls how leaves bend because of "
                                                     "gravity. Positive value results in leaf bending towards the "
                                                     "ground, negative value results in leaf bend towards the sky"};

      if (bending_along_leaf.OnInspect("Bending along leaf", leaf_bending)) {
        changed = true;
        bending_along_leaf.curve.UnsafeGetValues()[1].y = 0.5f;
      }
      if (waviness_along_leaf.OnInspect("Waviness along leaf"))
        changed = true;

      if (ImGui::DragFloat("Waviness frequency", &waviness_frequency, 0.01f, 0.0f, 999.0f))
        changed = true;
      if (ImGui::DragFloat2("Waviness start period", &waviness_period_start.x, 0.01f, 0.0f, 999.0f))
        changed = true;
      ImGui::TreePop();
    }
  }
  if (changed)
    saved = false;
  return changed;
}
void SorghumLeafGrowthStage::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "dead" << YAML::Value << dead;
  out << YAML::Key << "index" << YAML::Value << index;
  if (!dead) {
    out << YAML::Key << "spline" << YAML::Value << YAML::BeginMap;
    spline.Serialize(out);
    out << YAML::EndMap;

    out << YAML::Key << "starting_point" << YAML::Value << starting_point;
    out << YAML::Key << "length" << YAML::Value << length;
    curling_along_leaf.Save("curling_along_leaf", out);
    width_along_leaf.Save("width_along_leaf", out);
    out << YAML::Key << "roll_angle" << YAML::Value << roll_angle;
    out << YAML::Key << "branching_angle" << YAML::Value << branching_angle;
    bending_along_leaf.Save("bending_along_leaf", out);
    waviness_along_leaf.Save("waviness_along_leaf", out);
    out << YAML::Key << "waviness_frequency" << YAML::Value << waviness_frequency;
    out << YAML::Key << "waviness_period_start" << YAML::Value << waviness_period_start;
  }
}

void SorghumLeafGrowthStage::Deserialize(const YAML::Node& in) {
  if (in["index"])
    index = in["index"].as<int>();
  if (in["dead"])
    dead = in["dead"].as<bool>();
  if (!dead) {
    if (in["spline"]) {
      spline.Deserialize(in["spline"]);
    }

    if (in["starting_point"])
      starting_point = in["starting_point"].as<float>();
    if (in["length"])
      length = in["length"].as<float>();
    if (in["roll_angle"])
      roll_angle = in["roll_angle"].as<float>();
    if (in["branching_angle"])
      branching_angle = in["branching_angle"].as<float>();
    if (in["waviness_frequency"])
      waviness_frequency = in["waviness_frequency"].as<float>();
    if (in["waviness_period_start"])
      waviness_period_start = in["waviness_period_start"].as<glm::vec2>();

    curling_along_leaf.Load("curling_along_leaf", in);
    bending_along_leaf.Load("bending_along_leaf", in);
    width_along_leaf.Load("width_along_leaf", in);
    waviness_along_leaf.Load("waviness_along_leaf", in);
  }

  saved = true;
}
SorghumStemGrowthStage::SorghumStemGrowthStage() {
  length = 0.35f;
  width_along_stem = {0.0f, 0.015f, {0.6f, 0.4f, {0, 0}, {1, 1}}};

  saved = false;
}

SorghumLeafGrowthStage::SorghumLeafGrowthStage() {
  dead = false;
  waviness_along_leaf = {0.0f, 5.0f, {0.0f, 0.5f, {0, 0}, {1, 1}}};
  waviness_frequency = 0.03f;
  waviness_period_start = {0.0f, 0.0f};
  width_along_leaf = {0.0f, 0.02f, {0.5f, 0.1f, {0, 0}, {1, 1}}};
  auto& pairs = width_along_leaf.curve.UnsafeGetValues();
  pairs.clear();
  pairs.emplace_back(-0.1, 0.0f);
  pairs.emplace_back(0, 0.5);
  pairs.emplace_back(0.11196319, 0.111996889);

  pairs.emplace_back(-0.0687116608, 0);
  pairs.emplace_back(0.268404901, 0.92331290);
  pairs.emplace_back(0.100000001, 0.0f);

  pairs.emplace_back(-0.100000001, 0);
  pairs.emplace_back(0.519368708, 1);
  pairs.emplace_back(0.100000001, 0);

  pairs.emplace_back(-0.100000001, 0.0f);
  pairs.emplace_back(1, 0.1);
  pairs.emplace_back(0.1, 0.0f);

  bending_along_leaf = {-180.0f, 180.0f, {0.5f, 0.5, {0, 0}, {1, 1}}};
  curling_along_leaf = {0.0f, 90.0f, {0.3f, 0.3f, {0, 0}, {1, 1}}};
  length = 0.35f;
  branching_angle = 30.0f;

  saved = false;
}
void SorghumLeafGrowthStage::CopyShape(const SorghumLeafGrowthStage& another) {
  spline = another.spline;
  width_along_leaf.curve = another.width_along_leaf.curve;
  curling_along_leaf = another.curling_along_leaf;
  bending_along_leaf = another.bending_along_leaf;
  waviness_along_leaf = another.waviness_along_leaf;
  waviness_period_start = another.waviness_period_start;
  waviness_frequency = another.waviness_frequency;

  saved = false;
}

bool SorghumGrowthStage::OnInspect(int mode) {
  bool changed = false;
  if (ImGui::TreeNodeEx((std::string("Stem")).c_str())) {
    if (stem.OnInspect(mode))
      changed = true;
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Leaves")) {
    int leaf_size = leaves.size();
    if (ImGui::InputInt("Number of leaves", &leaf_size)) {
      changed = true;
      leaf_size = glm::clamp(leaf_size, 0, 999);
      const auto previous_size = leaves.size();
      leaves.resize(leaf_size);
      for (int i = 0; i < leaf_size; i++) {
        if (i >= previous_size) {
          if (i - 1 >= 0) {
            leaves[i] = leaves[i - 1];
            leaves[i].roll_angle = glm::mod(leaves[i - 1].roll_angle + 180.0f, 360.0f);
            leaves[i].starting_point = leaves[i - 1].starting_point + 0.1f;
          } else {
            leaves[i] = SorghumLeafGrowthStage();
            leaves[i].roll_angle = 0;
            leaves[i].starting_point = 0.1f;
          }
        }
        leaves[i].index = i;
      }
    }
    for (auto& leaf : leaves) {
      if (ImGui::TreeNode(
              ("Leaf No." + std::to_string(leaf.index + 1) + (leaf.length == 0.0f || leaf.dead ? " (Dead)" : ""))
                  .c_str())) {
        if (leaf.OnInspect(mode))
          changed = true;
        ImGui::TreePop();
      }
    }
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx((std::string("Panicle")).c_str())) {
    if (panicle.OnInspect())
      changed = true;
    ImGui::TreePop();
  }
  if (mode == static_cast<int>(StateMode::CubicBezier)) {
    FileUtils::OpenFile(
        "Import...", "TXT", {".txt"},
        [&](const std::filesystem::path& path) {
          std::ifstream file(path, std::fstream::in);
          if (!file.is_open()) {
            EVOENGINE_LOG("Failed to open file!");
            return;
          }
          changed = true;
          // Number of leaves in the file
          int leaf_count;
          file >> leaf_count;
          stem = SorghumStemGrowthStage();
          stem.spline.Import(file);
          /*
          // Recenter plant:
          glm::vec3 posSum = stem.spline.curves.front().p0;
          for (auto &curve : stem.spline.curves) {
            curve.p0 -= posSum;
            curve.m_p1 -= posSum;
            curve.m_p2 -= posSum;
            curve.m_p3 -= posSum;
          }
          */
          leaves.resize(leaf_count);
          for (int i = 0; i < leaf_count; i++) {
            float starting_point;
            file >> starting_point;
            leaves[i] = SorghumLeafGrowthStage();
            leaves[i].starting_point = starting_point;
            leaves[i].spline.Import(file);
            leaves[i].spline.curves[0].p0 = stem.spline.EvaluatePointFromCurves(starting_point);
          }

          for (int i = 0; i < leaf_count; i++) {
            leaves[i].index = i;
          }
        },
        false);
  }
  if (changed)
    saved = false;
  return changed;
}

void SorghumGrowthStage::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "version_" << YAML::Value << version_;
  out << YAML::Key << "name" << YAML::Value << name;
  out << YAML::Key << "panicle" << YAML::Value << YAML::BeginMap;
  panicle.Serialize(out);
  out << YAML::EndMap;
  out << YAML::Key << "stem" << YAML::Value << YAML::BeginMap;
  stem.Serialize(out);
  out << YAML::EndMap;

  if (!leaves.empty()) {
    out << YAML::Key << "leaves" << YAML::Value << YAML::BeginSeq;
    for (auto& i : leaves) {
      out << YAML::BeginMap;
      i.Serialize(out);
      out << YAML::EndMap;
    }
    out << YAML::EndSeq;
  }
}

void SorghumGrowthStage::Deserialize(const YAML::Node& in) {
  if (in["version_"])
    version_ = in["version_"].as<unsigned>();
  if (in["name"])
    name = in["name"].as<std::string>();
  if (in["panicle"])
    panicle.Deserialize(in["panicle"]);

  if (in["stem"])
    stem.Deserialize(in["stem"]);

  if (in["leaves"]) {
    for (const auto& i : in["leaves"]) {
      SorghumLeafGrowthStage leaf;
      leaf.Deserialize(i);
      leaves.push_back(leaf);
    }
  }
}
SorghumGrowthStage::SorghumGrowthStage() {
  saved = false;
  name = "Unnamed";
}
