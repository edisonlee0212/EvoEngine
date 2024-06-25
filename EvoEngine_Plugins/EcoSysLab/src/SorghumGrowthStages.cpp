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
using namespace eco_sys_lab;

void SorghumGrowthStages::Apply(const std::shared_ptr<SorghumState>& targetState, float time) const {
  if (m_sorghumGrowthStages.empty())
    return;
  const auto actualTime = glm::clamp(time, 0.0f, 99999.0f);
  float previousTime = m_sorghumGrowthStages.begin()->first;
  SorghumGrowthStagePair statePair;
  statePair.m_left = m_sorghumGrowthStages.begin()->second;
  statePair.m_right = statePair.m_left;

  if (actualTime < previousTime) {
    // Get from zero state to first state.
    statePair.Apply(targetState, 0.0f);
    return;
  }

  float a = 0.0f;
  for (auto it = (++m_sorghumGrowthStages.begin()); it != m_sorghumGrowthStages.end(); ++it) {
    statePair.m_left = statePair.m_right;
    statePair.m_right = it->second;
    if (it->first > actualTime) {
      a = (actualTime - previousTime) / (it->first - previousTime);
      break;
    }
    previousTime = it->first;
  }
  statePair.Apply(targetState, a);
}
void SorghumGrowthStagePair::Apply(const std::shared_ptr<SorghumState>& targetState, float a) {
  ApplyPanicle(targetState, a);
  ApplyStem(targetState, a);
  ApplyLeaves(targetState, a);
}

void SorghumGrowthStagePair::ApplyLeaves(const std::shared_ptr<SorghumState>& targetState, float a) {
  const auto leafSize = GetLeafSize(a);
  targetState->m_leaves.resize(leafSize);
  for (int i = 0; i < leafSize; i++) {
    ApplyLeaf(targetState, a, i);
  }
}

void SorghumGrowthStagePair::ApplyLeaf(const std::shared_ptr<SorghumState>& targetState, float a, int leafIndex) {
  constexpr auto upDirection = glm::vec3(0, 1, 0);
  auto frontDirection = glm::vec3(0, 0, -1);
  frontDirection = glm::rotate(frontDirection, glm::radians(glm::linearRand(0.0f, 360.0f)), upDirection);
  glm::vec3 stemFront = GetStemDirection(a);
  const float stemLength = GetStemLength(a);
  const auto sorghumLayer = Application::GetLayer<SorghumLayer>();

  const float preservedA = a;
  SorghumLeafGrowthStage actualLeft, actualRight;
  LeafStateHelper(actualLeft, actualRight, a, leafIndex);

  float stemWidth = glm::mix(m_left.m_stem.m_widthAlongStem.GetValue(actualLeft.m_startingPoint),
                             m_right.m_stem.m_widthAlongStem.GetValue(actualRight.m_startingPoint), preservedA);

  auto& leafState = targetState->m_leaves[leafIndex];
  leafState.m_spline.m_segments.clear();
  leafState.m_index = leafIndex;

  float startingPointRatio = glm::mix(actualLeft.m_startingPoint, actualRight.m_startingPoint, a);
  float leafLength = glm::mix(actualLeft.m_length, actualRight.m_length, a);
  if (leafLength == 0.0f)
    return;

  float branchingAngle = glm::mix(actualLeft.m_branchingAngle, actualRight.m_branchingAngle, a);
  float rollAngle = glm::mod(glm::mix(actualLeft.m_rollAngle, actualRight.m_rollAngle, a), 360.0f);

  // Build nodes...

  float backTrackRatio = 0.05f;
  if (startingPointRatio < backTrackRatio)
    backTrackRatio = startingPointRatio;

  glm::vec3 leafLeft = glm::normalize(glm::rotate(glm::vec3(0, 0, -1), glm::radians(rollAngle), glm::vec3(0, 1, 0)));
  auto leafUp = glm::normalize(glm::cross(stemFront, leafLeft));
  glm::vec3 stemOffset = stemWidth * -leafUp;

  auto direction = glm::rotate(glm::vec3(0, 1, 0), glm::radians(branchingAngle), leafLeft);
  float sheathRatio = startingPointRatio - backTrackRatio;

  if (sheathRatio > 0) {
    int rootToSheathNodeCount = glm::min(2.0f, stemLength * sheathRatio / sorghumLayer->vertical_subdivision_length);
    for (int i = 0; i < rootToSheathNodeCount; i++) {
      float factor = static_cast<float>(i) / rootToSheathNodeCount;
      float currentRootToSheathPoint = glm::mix(0.f, sheathRatio, factor);

      const auto up = glm::normalize(glm::cross(stemFront, leafLeft));
      leafState.m_spline.m_segments.emplace_back(
          glm::normalize(stemFront) * currentRootToSheathPoint * stemLength + stemOffset, up, stemFront, stemWidth,
          180.f, 0, 0);
    }
  }

  int sheathNodeCount = glm::max(2.0f, stemLength * backTrackRatio / sorghumLayer->vertical_subdivision_length);
  for (int i = 0; i <= sheathNodeCount; i++) {
    float factor = static_cast<float>(i) / sheathNodeCount;
    float currentSheathPoint =
        glm::mix(sheathRatio, startingPointRatio,
                 factor);  // sheathRatio + static_cast<float>(i) / sheathNodeCount * backTrackRatio;
    glm::vec3 actualDirection = glm::normalize(glm::mix(stemFront, direction, factor));

    const auto up = glm::normalize(glm::cross(actualDirection, leafLeft));
    leafState.m_spline.m_segments.emplace_back(glm::normalize(stemFront) * currentSheathPoint * stemLength + stemOffset,
                                               up, actualDirection,
                                               stemWidth + 0.002f * static_cast<float>(i) / sheathNodeCount,
                                               180.0f - 90.0f * static_cast<float>(i) / sheathNodeCount, 0, 0);
  }

  int nodeAmount = glm::max(4.0f, leafLength / sorghumLayer->vertical_subdivision_length);
  float unitLength = leafLength / nodeAmount;

  int nodeToFullExpand = 0.1f * leafLength / sorghumLayer->vertical_subdivision_length;

  float heightOffset = glm::linearRand(0.f, 100.f);
  const float wavinessFrequency = glm::mix(actualLeft.m_wavinessFrequency, actualRight.m_wavinessFrequency, a);
  glm::vec3 nodePosition = stemFront * startingPointRatio * stemLength + stemOffset;
  for (int i = 1; i <= nodeAmount; i++) {
    const float factor = static_cast<float>(i) / nodeAmount;
    glm::vec3 currentDirection;

    float rotateAngle =
        glm::mix(actualLeft.m_bendingAlongLeaf.GetValue(factor), actualRight.m_bendingAlongLeaf.GetValue(factor), a);
    currentDirection = glm::rotate(direction, glm::radians(rotateAngle), leafLeft);
    nodePosition += currentDirection * unitLength;

    float expandAngle =
        glm::mix(actualLeft.m_curlingAlongLeaf.GetValue(factor), actualRight.m_curlingAlongLeaf.GetValue(factor), a);

    float collarFactor = glm::min(1.0f, static_cast<float>(i) / nodeToFullExpand);

    float waviness =
        glm::mix(actualLeft.m_wavinessAlongLeaf.GetValue(factor), actualRight.m_wavinessAlongLeaf.GetValue(factor), a);
    heightOffset += wavinessFrequency;

    float width = glm::mix(
        stemWidth + 0.002f,
        glm::mix(actualLeft.m_widthAlongLeaf.GetValue(factor), actualRight.m_widthAlongLeaf.GetValue(factor), a),
        collarFactor);
    float angle = 90.0f - (90.0f - expandAngle) * glm::pow(collarFactor, 2.0f);

    const auto up = glm::normalize(glm::cross(currentDirection, leafLeft));
    leafState.m_spline.m_segments.emplace_back(nodePosition, up, currentDirection, width, angle,
                                               waviness * glm::simplex(glm::vec2(heightOffset, 0.f)),
                                               waviness * glm::simplex(glm::vec2(0.f, heightOffset)));
  }
}

void SorghumGrowthStagePair::LeafStateHelper(SorghumLeafGrowthStage& left, SorghumLeafGrowthStage& right, float& a,
                                             int leafIndex) const {
  const int previousLeafSize = m_left.m_leaves.size();
  const int nextLeafSize = m_right.m_leaves.size();
  if (leafIndex < previousLeafSize) {
    left = m_left.m_leaves[leafIndex];
    if (left.m_dead)
      left.m_length = 0;
    if (leafIndex < nextLeafSize) {
      if (m_right.m_leaves[leafIndex].m_dead || m_right.m_leaves[leafIndex].m_length == 0)
        right = left;
      else {
        right = m_right.m_leaves[leafIndex];
      }
    } else {
      right = m_left.m_leaves[leafIndex];
    }
    return;
  }

  const int completedLeafSize =
      m_left.m_leaves.size() + glm::floor((m_right.m_leaves.size() - m_left.m_leaves.size()) * a);
  a = glm::clamp(a * (nextLeafSize - previousLeafSize) - (completedLeafSize - previousLeafSize), 0.0f, 1.0f);
  left = right = m_right.m_leaves[leafIndex];
  if (leafIndex >= completedLeafSize) {
    left.m_length = 0.0f;
    left.m_widthAlongLeaf.min_value = left.m_widthAlongLeaf.max_value = 0.0f;
    left.m_wavinessAlongLeaf.min_value = left.m_wavinessAlongLeaf.max_value = 0.0f;
    for (auto& i : left.m_spline.curves) {
      i.p0 = i.p1 = i.p2 = i.p3 = right.m_spline.EvaluatePointFromCurves(0.0f);
    }
  } else {
    left = right;
  }
}

int SorghumGrowthStagePair::GetLeafSize(float a) const {
  if (m_left.m_leaves.size() <= m_right.m_leaves.size()) {
    return m_left.m_leaves.size() + glm::ceil((m_right.m_leaves.size() - m_left.m_leaves.size()) * a);
  }
  return m_left.m_leaves.size();
}
float SorghumGrowthStagePair::GetStemLength(float a) const {
  float leftLength, rightLength;
  switch ((StateMode)m_mode) {
    case StateMode::Default:
      leftLength = m_left.m_stem.m_length;
      rightLength = m_right.m_stem.m_length;
      break;
    case StateMode::CubicBezier:
      if (!m_left.m_stem.m_spline.curves.empty()) {
        leftLength = glm::distance(m_left.m_stem.m_spline.curves.front().p0, m_left.m_stem.m_spline.curves.back().p3);
      } else {
        leftLength = 0.0f;
      }
      if (!m_right.m_stem.m_spline.curves.empty()) {
        rightLength =
            glm::distance(m_right.m_stem.m_spline.curves.front().p0, m_right.m_stem.m_spline.curves.back().p3);
      } else {
        rightLength = 0.0f;
      }
      break;
  }
  return glm::mix(leftLength, rightLength, a);
}
glm::vec3 SorghumGrowthStagePair::GetStemDirection(float a) const {
  glm::vec3 leftDir, rightDir;
  switch ((StateMode)m_mode) {
    case StateMode::Default:
      leftDir = glm::normalize(m_left.m_stem.m_direction);
      rightDir = glm::normalize(m_right.m_stem.m_direction);
      break;
    case StateMode::CubicBezier:
      if (!m_left.m_stem.m_spline.curves.empty()) {
        leftDir = glm::vec3(0.0f, 1.0f, 0.0f);
      } else {
        leftDir = glm::vec3(0.0f, 1.0f, 0.0f);
      }
      if (!m_right.m_stem.m_spline.curves.empty()) {
        rightDir = glm::vec3(0.0f, 1.0f, 0.0f);
      } else {
        rightDir = glm::vec3(0.0f, 1.0f, 0.0f);
      }
      break;
  }

  return glm::normalize(glm::mix(leftDir, rightDir, a));
}
glm::vec3 SorghumGrowthStagePair::GetStemPoint(float a, float point) const {
  glm::vec3 leftPoint, rightPoint;
  switch ((StateMode)m_mode) {
    case StateMode::Default:
      leftPoint = glm::normalize(m_left.m_stem.m_direction) * point * m_left.m_stem.m_length;
      rightPoint = glm::normalize(m_right.m_stem.m_direction) * point * m_right.m_stem.m_length;
      break;
    case StateMode::CubicBezier:
      if (!m_left.m_stem.m_spline.curves.empty()) {
        leftPoint = m_left.m_stem.m_spline.EvaluatePointFromCurves(point);
      } else {
        leftPoint = glm::vec3(0.0f, 0.0f, 0.0f);
      }
      if (!m_right.m_stem.m_spline.curves.empty()) {
        rightPoint = m_right.m_stem.m_spline.EvaluatePointFromCurves(point);
      } else {
        rightPoint = glm::vec3(0.0f, 0.0f, 0.0f);
      }
      break;
  }

  return glm::mix(leftPoint, rightPoint, a);
}

void SorghumGrowthStagePair::ApplyPanicle(const std::shared_ptr<SorghumState>& targetState, const float a) const {
  targetState->m_panicle.m_panicleSize = glm::mix(m_left.m_panicle.m_panicleSize, m_right.m_panicle.m_panicleSize, a);
  targetState->m_panicle.m_seedAmount = glm::mix(m_left.m_panicle.m_seedAmount, m_right.m_panicle.m_seedAmount, a);
  targetState->m_panicle.m_seedRadius = glm::mix(m_left.m_panicle.m_seedRadius, m_right.m_panicle.m_seedRadius, a);
}

void SorghumGrowthStagePair::ApplyStem(const std::shared_ptr<SorghumState>& targetState, float a) const {
  constexpr auto upDirection = glm::vec3(0, 1, 0);
  auto frontDirection = glm::vec3(0, 0, -1);
  frontDirection = glm::rotate(frontDirection, glm::radians(glm::linearRand(0.0f, 360.0f)), upDirection);
  glm::vec3 stemFront = GetStemDirection(a);
  const float stemLength = GetStemLength(a);
  const auto sorghumLayer = Application::GetLayer<SorghumLayer>();
  const int stemNodeAmount = static_cast<int>(glm::max(4.0f, stemLength / sorghumLayer->vertical_subdivision_length));
  const float stemUnitLength = stemLength / stemNodeAmount;
  targetState->m_stem.m_spline.m_segments.clear();
  const glm::vec3 stemLeft =
      glm::normalize(glm::rotate(glm::vec3(1, 0, 0), glm::radians(glm::linearRand(0.0f, 0.0f)), stemFront));
  for (int i = 0; i <= stemNodeAmount; i++) {
    float stemWidth = glm::mix(m_left.m_stem.m_widthAlongStem.GetValue(static_cast<float>(i) / stemNodeAmount),
                               m_right.m_stem.m_widthAlongStem.GetValue(static_cast<float>(i) / stemNodeAmount), a);
    glm::vec3 stemNodePosition;
    stemNodePosition = stemFront * stemUnitLength * static_cast<float>(i);

    const auto up = glm::normalize(glm::cross(stemFront, stemLeft));
    targetState->m_stem.m_spline.m_segments.emplace_back(stemNodePosition, up, stemFront, stemWidth, 180.f, 0, 0);
  }
}

bool SorghumPanicleGrowthStage::OnInspect() {
  bool changed = false;
  if (ImGui::DragFloat("Panicle width", &m_panicleSize.x, 0.001f)) {
    changed = true;
    m_panicleSize.z = m_panicleSize.x;
  }
  if (ImGui::DragFloat("Panicle height", &m_panicleSize.y, 0.001f))
    changed = true;
  if (ImGui::DragInt("Num of seeds", &m_seedAmount, 1.0f))
    changed = true;
  if (ImGui::DragFloat("Seed radius", &m_seedRadius, 0.0001f))
    changed = true;
  if (changed)
    m_saved = false;
  return changed;
}
void SorghumPanicleGrowthStage::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "m_panicleSize" << YAML::Value << m_panicleSize;
  out << YAML::Key << "m_seedAmount" << YAML::Value << m_seedAmount;
  out << YAML::Key << "m_seedRadius" << YAML::Value << m_seedRadius;
}
void SorghumPanicleGrowthStage::Deserialize(const YAML::Node& in) {
  if (in["m_panicleSize"])
    m_panicleSize = in["m_panicleSize"].as<glm::vec3>();
  if (in["m_seedAmount"])
    m_seedAmount = in["m_seedAmount"].as<int>();
  if (in["m_seedRadius"])
    m_seedRadius = in["m_seedRadius"].as<float>();
  m_saved = true;
}

SorghumPanicleGrowthStage::SorghumPanicleGrowthStage() {
  m_panicleSize = glm::vec3(0, 0, 0);
  m_seedAmount = 0;
  m_seedRadius = 0.002f;
  m_saved = false;
}
void SorghumStemGrowthStage::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "m_direction" << YAML::Value << m_direction;
  m_widthAlongStem.Save("m_widthAlongStem", out);
  out << YAML::Key << "m_length" << YAML::Value << m_length;
  out << YAML::Key << "m_spline" << YAML::Value << YAML::BeginMap;
  m_spline.Serialize(out);
  out << YAML::EndMap;
}
void SorghumStemGrowthStage::Deserialize(const YAML::Node& in) {
  if (in["m_spline"]) {
    m_spline.Deserialize(in["m_spline"]);
  }

  if (in["m_direction"])
    m_direction = in["m_direction"].as<glm::vec3>();
  if (in["m_length"])
    m_length = in["m_length"].as<float>();
  m_widthAlongStem.Load("m_widthAlongStem", in);

  m_saved = true;
}
bool SorghumStemGrowthStage::OnInspect(int mode) {
  bool changed = false;
  switch ((StateMode)mode) {
    case StateMode::Default:
      // ImGui::DragFloat3("Direction", &m_direction.x, 0.01f);
      if (ImGui::DragFloat("Length", &m_length, 0.01f))
        changed = true;
      break;
    case StateMode::CubicBezier:
      if (ImGui::TreeNode("Spline")) {
        m_spline.OnInspect();
        ImGui::TreePop();
      }
      break;
  }
  if (m_widthAlongStem.OnInspect("Width along stem"))
    changed = true;

  if (changed)
    m_saved = false;
  return changed;
}
bool SorghumLeafGrowthStage::OnInspect(int mode) {
  bool changed = false;
  if (ImGui::Checkbox("Dead", &m_dead)) {
    changed = true;
    if (!m_dead && m_length == 0.0f)
      m_length = 0.35f;
  }
  if (!m_dead) {
    if (ImGui::InputFloat("Starting point", &m_startingPoint)) {
      m_startingPoint = glm::clamp(m_startingPoint, 0.0f, 1.0f);
      changed = true;
    }
    switch ((StateMode)mode) {
      case StateMode::Default:
        if (ImGui::TreeNodeEx("Geometric", ImGuiTreeNodeFlags_DefaultOpen)) {
          if (ImGui::DragFloat("Length", &m_length, 0.01f, 0.0f, 999.0f))
            changed = true;
          if (ImGui::TreeNodeEx("Angles", ImGuiTreeNodeFlags_DefaultOpen)) {
            if (ImGui::DragFloat("Roll angle", &m_rollAngle, 1.0f, -999.0f, 999.0f))
              changed = true;
            if (ImGui::InputFloat("Branching angle", &m_branchingAngle)) {
              m_branchingAngle = glm::clamp(m_branchingAngle, 0.0f, 180.0f);
              changed = true;
            }
            ImGui::TreePop();
          }
          ImGui::TreePop();
        }
        break;
      case StateMode::CubicBezier:
        if (ImGui::TreeNodeEx("Geometric", ImGuiTreeNodeFlags_DefaultOpen)) {
          m_spline.OnInspect();
          ImGui::TreePop();
        }
        break;
    }

    if (ImGui::TreeNodeEx("Others")) {
      if (m_widthAlongLeaf.OnInspect("Width"))
        changed = true;
      if (m_curlingAlongLeaf.OnInspect("Rolling"))
        changed = true;

      static CurveDescriptorSettings leafBending = {1.0f, false, true,
                                                    "The bending of the leaf, controls how leaves bend because of "
                                                    "gravity. Positive value results in leaf bending towards the "
                                                    "ground, negative value results in leaf bend towards the sky"};

      if (m_bendingAlongLeaf.OnInspect("Bending along leaf", leafBending)) {
        changed = true;
        m_bendingAlongLeaf.curve.UnsafeGetValues()[1].y = 0.5f;
      }
      if (m_wavinessAlongLeaf.OnInspect("Waviness along leaf"))
        changed = true;

      if (ImGui::DragFloat("Waviness frequency", &m_wavinessFrequency, 0.01f, 0.0f, 999.0f))
        changed = true;
      if (ImGui::DragFloat2("Waviness start period", &m_wavinessPeriodStart.x, 0.01f, 0.0f, 999.0f))
        changed = true;
      ImGui::TreePop();
    }
  }
  if (changed)
    m_saved = false;
  return changed;
}
void SorghumLeafGrowthStage::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "m_dead" << YAML::Value << m_dead;
  out << YAML::Key << "m_index" << YAML::Value << m_index;
  if (!m_dead) {
    out << YAML::Key << "m_spline" << YAML::Value << YAML::BeginMap;
    m_spline.Serialize(out);
    out << YAML::EndMap;

    out << YAML::Key << "m_startingPoint" << YAML::Value << m_startingPoint;
    out << YAML::Key << "m_length" << YAML::Value << m_length;
    m_curlingAlongLeaf.Save("m_curlingAlongLeaf", out);
    m_widthAlongLeaf.Save("m_widthAlongLeaf", out);
    out << YAML::Key << "m_rollAngle" << YAML::Value << m_rollAngle;
    out << YAML::Key << "m_branchingAngle" << YAML::Value << m_branchingAngle;
    m_bendingAlongLeaf.Save("m_bendingAlongLeaf", out);
    m_wavinessAlongLeaf.Save("m_wavinessAlongLeaf", out);
    out << YAML::Key << "m_wavinessFrequency" << YAML::Value << m_wavinessFrequency;
    out << YAML::Key << "m_wavinessPeriodStart" << YAML::Value << m_wavinessPeriodStart;
  }
}

void SorghumLeafGrowthStage::Deserialize(const YAML::Node& in) {
  if (in["m_index"])
    m_index = in["m_index"].as<int>();
  if (in["m_dead"])
    m_dead = in["m_dead"].as<bool>();
  if (!m_dead) {
    if (in["m_spline"]) {
      m_spline.Deserialize(in["m_spline"]);
    }

    if (in["m_startingPoint"])
      m_startingPoint = in["m_startingPoint"].as<float>();
    if (in["m_length"])
      m_length = in["m_length"].as<float>();
    if (in["m_rollAngle"])
      m_rollAngle = in["m_rollAngle"].as<float>();
    if (in["m_branchingAngle"])
      m_branchingAngle = in["m_branchingAngle"].as<float>();
    if (in["m_wavinessFrequency"])
      m_wavinessFrequency = in["m_wavinessFrequency"].as<float>();
    if (in["m_wavinessPeriodStart"])
      m_wavinessPeriodStart = in["m_wavinessPeriodStart"].as<glm::vec2>();

    m_curlingAlongLeaf.Load("m_curlingAlongLeaf", in);
    m_bendingAlongLeaf.Load("m_bendingAlongLeaf", in);
    m_widthAlongLeaf.Load("m_widthAlongLeaf", in);
    m_wavinessAlongLeaf.Load("m_wavinessAlongLeaf", in);
  }

  m_saved = true;
}
SorghumStemGrowthStage::SorghumStemGrowthStage() {
  m_length = 0.35f;
  m_widthAlongStem = {0.0f, 0.015f, {0.6f, 0.4f, {0, 0}, {1, 1}}};

  m_saved = false;
}

SorghumLeafGrowthStage::SorghumLeafGrowthStage() {
  m_dead = false;
  m_wavinessAlongLeaf = {0.0f, 5.0f, {0.0f, 0.5f, {0, 0}, {1, 1}}};
  m_wavinessFrequency = 0.03f;
  m_wavinessPeriodStart = {0.0f, 0.0f};
  m_widthAlongLeaf = {0.0f, 0.02f, {0.5f, 0.1f, {0, 0}, {1, 1}}};
  auto& pairs = m_widthAlongLeaf.curve.UnsafeGetValues();
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

  m_bendingAlongLeaf = {-180.0f, 180.0f, {0.5f, 0.5, {0, 0}, {1, 1}}};
  m_curlingAlongLeaf = {0.0f, 90.0f, {0.3f, 0.3f, {0, 0}, {1, 1}}};
  m_length = 0.35f;
  m_branchingAngle = 30.0f;

  m_saved = false;
}
void SorghumLeafGrowthStage::CopyShape(const SorghumLeafGrowthStage& another) {
  m_spline = another.m_spline;
  m_widthAlongLeaf.curve = another.m_widthAlongLeaf.curve;
  m_curlingAlongLeaf = another.m_curlingAlongLeaf;
  m_bendingAlongLeaf = another.m_bendingAlongLeaf;
  m_wavinessAlongLeaf = another.m_wavinessAlongLeaf;
  m_wavinessPeriodStart = another.m_wavinessPeriodStart;
  m_wavinessFrequency = another.m_wavinessFrequency;

  m_saved = false;
}

bool SorghumGrowthStage::OnInspect(int mode) {
  bool changed = false;
  if (ImGui::TreeNodeEx((std::string("Stem")).c_str())) {
    if (m_stem.OnInspect(mode))
      changed = true;
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Leaves")) {
    int leafSize = m_leaves.size();
    if (ImGui::InputInt("Number of leaves", &leafSize)) {
      changed = true;
      leafSize = glm::clamp(leafSize, 0, 999);
      auto previousSize = m_leaves.size();
      m_leaves.resize(leafSize);
      for (int i = 0; i < leafSize; i++) {
        if (i >= previousSize) {
          if (i - 1 >= 0) {
            m_leaves[i] = m_leaves[i - 1];
            m_leaves[i].m_rollAngle = glm::mod(m_leaves[i - 1].m_rollAngle + 180.0f, 360.0f);
            m_leaves[i].m_startingPoint = m_leaves[i - 1].m_startingPoint + 0.1f;
          } else {
            m_leaves[i] = SorghumLeafGrowthStage();
            m_leaves[i].m_rollAngle = 0;
            m_leaves[i].m_startingPoint = 0.1f;
          }
        }
        m_leaves[i].m_index = i;
      }
    }
    for (auto& leaf : m_leaves) {
      if (ImGui::TreeNode(
              ("Leaf No." + std::to_string(leaf.m_index + 1) + (leaf.m_length == 0.0f || leaf.m_dead ? " (Dead)" : ""))
                  .c_str())) {
        if (leaf.OnInspect(mode))
          changed = true;
        ImGui::TreePop();
      }
    }
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx((std::string("Panicle")).c_str())) {
    if (m_panicle.OnInspect())
      changed = true;
    ImGui::TreePop();
  }
  if (mode == (int)StateMode::CubicBezier) {
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
          int leafCount;
          file >> leafCount;
          m_stem = SorghumStemGrowthStage();
          m_stem.m_spline.Import(file);
          /*
          // Recenter plant:
          glm::vec3 posSum = m_stem.m_spline.curves.front().p0;
          for (auto &curve : m_stem.m_spline.curves) {
            curve.p0 -= posSum;
            curve.m_p1 -= posSum;
            curve.m_p2 -= posSum;
            curve.m_p3 -= posSum;
          }
          */
          m_leaves.resize(leafCount);
          for (int i = 0; i < leafCount; i++) {
            float startingPoint;
            file >> startingPoint;
            m_leaves[i] = SorghumLeafGrowthStage();
            m_leaves[i].m_startingPoint = startingPoint;
            m_leaves[i].m_spline.Import(file);
            m_leaves[i].m_spline.curves[0].p0 = m_stem.m_spline.EvaluatePointFromCurves(startingPoint);
          }

          for (int i = 0; i < leafCount; i++) {
            m_leaves[i].m_index = i;
          }
        },
        false);
  }
  if (changed)
    m_saved = false;
  return changed;
}

void SorghumGrowthStage::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "m_version" << YAML::Value << m_version;
  out << YAML::Key << "m_name" << YAML::Value << m_name;
  out << YAML::Key << "m_panicle" << YAML::Value << YAML::BeginMap;
  m_panicle.Serialize(out);
  out << YAML::EndMap;
  out << YAML::Key << "m_stem" << YAML::Value << YAML::BeginMap;
  m_stem.Serialize(out);
  out << YAML::EndMap;

  if (!m_leaves.empty()) {
    out << YAML::Key << "m_leaves" << YAML::Value << YAML::BeginSeq;
    for (auto& i : m_leaves) {
      out << YAML::BeginMap;
      i.Serialize(out);
      out << YAML::EndMap;
    }
    out << YAML::EndSeq;
  }
}

void SorghumGrowthStage::Deserialize(const YAML::Node& in) {
  if (in["m_version"])
    m_version = in["m_version"].as<unsigned>();
  if (in["m_name"])
    m_name = in["m_name"].as<std::string>();
  if (in["m_panicle"])
    m_panicle.Deserialize(in["m_panicle"]);

  if (in["m_stem"])
    m_stem.Deserialize(in["m_stem"]);

  if (in["m_leaves"]) {
    for (const auto& i : in["m_leaves"]) {
      SorghumLeafGrowthStage leaf;
      leaf.Deserialize(i);
      m_leaves.push_back(leaf);
    }
  }
}
SorghumGrowthStage::SorghumGrowthStage() {
  m_saved = false;
  m_name = "Unnamed";
}
