#include "SorghumStateGenerator.hpp"
#include "ProjectManager.hpp"
#include "SorghumLayer.hpp"

#include "Times.hpp"

#include "Scene.hpp"
#include "Sorghum.hpp"
#include "SorghumSpline.hpp"

using namespace eco_sys_lab;

void TipMenu(const std::string& content) {
  if (ImGui::IsItemHovered()) {
    ImGui::BeginTooltip();
    ImGui::TextUnformatted(content.c_str());
    ImGui::EndTooltip();
  }
}

bool SorghumStateGenerator::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  if (ImGui::Button("Instantiate")) {
    auto entity = CreateEntity();
  }
  static bool auto_save = true;
  ImGui::Checkbox("Auto save", &auto_save);
  static bool intro = true;
  ImGui::Checkbox("Introduction", &intro);
  if (intro) {
    ImGui::TextWrapped(
        "This is the introduction of the parameter setting interface. "
        "\nFor each parameter, you are allowed to set average and "
        "variance value. \nInstantiate a new sorghum in the scene so you "
        "can preview the changes in real time. \nThe curve editors are "
        "provided for stem/leaf details to allow you have control of "
        "geometric properties along the stem/leaf. It's also provided "
        "for leaf settings to allow you control the distribution of "
        "different leaves from the bottom to top.\nMake sure you Save the "
        "parameters!\nValues are in meters or degrees.");
  }
  bool changed = false;
  if (ImGui::TreeNodeEx("Panicle settings", ImGuiTreeNodeFlags_DefaultOpen)) {
    TipMenu(
        "The settings for panicle. The panicle will always be placed "
        "at the tip of the stem.");
    if (m_panicleSize.OnInspect("Size", 0.001f, "The size of panicle")) {
      changed = true;
    }
    if (m_panicleSeedAmount.OnInspect("Seed amount", 1.0f, "The amount of seeds in the panicle"))
      changed = true;
    if (m_panicleSeedRadius.OnInspect("Seed radius", 0.001f, "The size of the seed in the panicle"))
      changed = true;
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Stem settings", ImGuiTreeNodeFlags_DefaultOpen)) {
    TipMenu("The settings for stem.");
    if (m_stemTiltAngle.OnInspect("Stem tilt angle", 0.001f, "The tilt angle for stem")) {
      changed = true;
    }
    if (m_internodeLength.OnInspect("Length", 0.01f,
                                    "The length of the stem, use Ending Point in leaf settings to make "
                                    "stem taller than top leaf for panicle"))
      changed = true;
    if (m_stemWidth.OnInspect("Width", 0.001f,
                              "The overall width of the stem, adjust the width "
                              "along stem in Stem Details"))
      changed = true;
    if (ImGui::TreeNode("Stem Details")) {
      TipMenu("The detailed settings for stem.");
      if (m_widthAlongStem.OnInspect("Width along stem"))
        changed = true;
      ImGui::TreePop();
    }
    ImGui::TreePop();
  }
  if (ImGui::TreeNodeEx("Leaves settings", ImGuiTreeNodeFlags_DefaultOpen)) {
    TipMenu("The settings for leaves.");
    if (m_leafAmount.OnInspect("Num of leaves", 1.0f, "The total amount of leaves"))
      changed = true;

    static PlottedDistributionSettings leaf_starting_point = {
        0.01f,
        {0.01f, false, true, ""},
        {0.01f, false, false, ""},
        "The starting point of each leaf along stem. Default each leaf "
        "located uniformly on stem."};

    if (m_leafStartingPoint.OnInspect("Starting point along stem", leaf_starting_point)) {
      changed = true;
    }

    static PlottedDistributionSettings leaf_curling = {
        0.01f, {0.01f, false, true, ""}, {0.01f, false, false, ""}, "The leaf curling."};

    if (m_leafCurling.OnInspect("Leaf curling", leaf_curling)) {
      changed = true;
    }

    static PlottedDistributionSettings leaf_roll_angle = {0.01f,
                                                        {},
                                                        {},
                                                        "The polar angle of leaf. Normally you should only change the "
                                                        "deviation. Values are in degrees"};
    if (m_leafRollAngle.OnInspect("Roll angle", leaf_roll_angle))
      changed = true;

    static PlottedDistributionSettings leaf_branching_angle = {
        0.01f, {}, {}, "The branching angle of the leaf. Values are in degrees"};
    if (m_leafBranchingAngle.OnInspect("Branching angle", leaf_branching_angle))
      changed = true;

    static PlottedDistributionSettings leaf_bending = {1.0f,
                                                      {1.0f, false, true, ""},
                                                      {},
                                                      "The bending of the leaf, controls how leaves bend because of "
                                                      "gravity. Positive value results in leaf bending towards the "
                                                      "ground, negative value results in leaf bend towards the sky"};
    if (m_leafBending.OnInspect("Bending", leaf_bending))
      changed = true;

    static PlottedDistributionSettings leaf_bending_acceleration = {
        0.01f, {0.01f, false, true, ""}, {}, "The changes of bending along the leaf."};

    if (m_leafBendingAcceleration.OnInspect("Bending acceleration", leaf_bending_acceleration))
      changed = true;

    static PlottedDistributionSettings leaf_bending_smoothness = {
        0.01f, {0.01f, false, true, ""}, {}, "The smoothness of bending along the leaf."};

    if (m_leafBendingSmoothness.OnInspect("Bending smoothness", leaf_bending_smoothness))
      changed = true;

    if (m_leafWaviness.OnInspect("Waviness"))
      changed = true;
    if (m_leafWavinessFrequency.OnInspect("Waviness Frequency"))
      changed = true;

    if (m_leafLength.OnInspect("Length"))
      changed = true;
    if (m_leafWidth.OnInspect("Width"))
      changed = true;

    if (ImGui::TreeNode("Leaf Details")) {
      if (m_widthAlongLeaf.OnInspect("Width along leaf"))
        changed = true;
      if (m_wavinessAlongLeaf.OnInspect("Waviness along leaf"))
        changed = true;
      ImGui::TreePop();
    }
    ImGui::TreePop();
  }

  static double last_auto_save_time = 0;
  static float auto_save_interval = 5;

  if (auto_save) {
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
  } else {
    if (!saved_) {
      ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(255, 0, 0, 255));
      ImGui::Text("[Changed unsaved!]");
      ImGui::PopStyleColor();
    }
  }

  return changed;
}
void SorghumStateGenerator::Serialize(YAML::Emitter& out) const {
  m_panicleSize.Save("m_panicleSize", out);
  m_panicleSeedAmount.Save("m_panicleSeedAmount", out);
  m_panicleSeedRadius.Save("m_panicleSeedRadius", out);

  m_stemTiltAngle.Save("m_stemTiltAngle", out);
  m_internodeLength.Save("m_internodeLength", out);
  m_stemWidth.Save("m_stemWidth", out);

  m_leafAmount.Save("m_leafAmount", out);
  m_leafStartingPoint.Save("m_leafStartingPoint", out);
  m_leafCurling.Save("m_leafCurling", out);
  m_leafRollAngle.Save("m_leafRollAngle", out);
  m_leafBranchingAngle.Save("m_leafBranchingAngle", out);
  m_leafBending.Save("m_leafBending", out);
  m_leafBendingAcceleration.Save("m_leafBendingAcceleration", out);
  m_leafBendingSmoothness.Save("m_leafBendingSmoothness", out);
  m_leafWaviness.Save("m_leafWaviness", out);
  m_leafWavinessFrequency.Save("m_leafWavinessFrequency", out);
  m_leafLength.Save("m_leafLength", out);
  m_leafWidth.Save("m_leafWidth", out);

  m_widthAlongStem.Save("m_widthAlongStem", out);
  m_widthAlongLeaf.Save("m_widthAlongLeaf", out);
  m_wavinessAlongLeaf.Save("m_wavinessAlongLeaf", out);
}
void SorghumStateGenerator::Deserialize(const YAML::Node& in) {
  m_panicleSize.Load("m_panicleSize", in);
  m_panicleSeedAmount.Load("m_panicleSeedAmount", in);
  m_panicleSeedRadius.Load("m_panicleSeedRadius", in);

  m_stemTiltAngle.Load("m_stemTiltAngle", in);
  m_internodeLength.Load("m_internodeLength", in);
  m_stemWidth.Load("m_stemWidth", in);

  m_leafAmount.Load("m_leafAmount", in);
  m_leafStartingPoint.Load("m_leafStartingPoint", in);
  m_leafCurling.Load("m_leafCurling", in);

  m_leafRollAngle.Load("m_leafRollAngle", in);
  m_leafBranchingAngle.Load("m_leafBranchingAngle", in);

  m_leafBending.Load("m_leafBending", in);
  m_leafBendingAcceleration.Load("m_leafBendingAcceleration", in);
  m_leafBendingSmoothness.Load("m_leafBendingSmoothness", in);
  m_leafWaviness.Load("m_leafWaviness", in);
  m_leafWavinessFrequency.Load("m_leafWavinessFrequency", in);
  m_leafLength.Load("m_leafLength", in);
  m_leafWidth.Load("m_leafWidth", in);

  m_widthAlongStem.Load("m_widthAlongStem", in);
  m_widthAlongLeaf.Load("m_widthAlongLeaf", in);
  m_wavinessAlongLeaf.Load("m_wavinessAlongLeaf", in);
}

Entity SorghumStateGenerator::CreateEntity(const unsigned int seed) const {
  const auto scene = Application::GetActiveScene();
  const auto entity = scene->CreateEntity(GetTitle());
  const auto sorghum = scene->GetOrSetPrivateComponent<Sorghum>(entity).lock();
  const auto sorghum_state = ProjectManager::CreateTemporaryAsset<SorghumState>();
  Apply(sorghum_state, seed);
  sorghum->m_sorghumState = sorghum_state;
  sorghum->m_sorghumDescriptor = GetSelf();
  sorghum->GenerateGeometryEntities(SorghumMeshGeneratorSettings{});
  return entity;
}

void SorghumStateGenerator::Apply(const std::shared_ptr<SorghumState>& target_state, const unsigned int seed) const {
  if (seed > 0)
    srand(seed);

  constexpr auto up_direction = glm::vec3(0, 1, 0);
  auto front_direction = glm::vec3(0, 0, -1);
  front_direction = glm::rotate(front_direction, glm::radians(glm::linearRand(0.0f, 360.0f)), up_direction);

  glm::vec3 stem_front = glm::normalize(glm::rotate(
      up_direction, glm::radians(glm::gaussRand(m_stemTiltAngle.mean, m_stemTiltAngle.deviation)), front_direction));
  const int leaf_size = glm::clamp(m_leafAmount.GetValue(), 2.0f, 128.0f);
  float stem_length = m_internodeLength.GetValue() * leaf_size / (1.f - m_leafStartingPoint.GetValue(0));
  Plot2D<float> width_along_stem = {0.0f, m_stemWidth.GetValue(), m_widthAlongStem};
  const auto sorghum_layer = Application::GetLayer<SorghumLayer>();
  // Build stem...
  target_state->m_stem.m_spline.m_segments.clear();
  int stem_node_amount = static_cast<int>(glm::max(4.0f, stem_length / sorghum_layer->vertical_subdivision_length));
  float stem_unit_length = stem_length / stem_node_amount;
  glm::vec3 stem_left =
      glm::normalize(glm::rotate(glm::vec3(1, 0, 0), glm::radians(glm::linearRand(0.0f, 0.0f)), stem_front));
  for (int i = 0; i <= stem_node_amount; i++) {
    float stem_width = width_along_stem.GetValue(static_cast<float>(i) / stem_node_amount);
    glm::vec3 stem_node_position;
    stem_node_position = stem_front * stem_unit_length * static_cast<float>(i);

    const auto up = glm::normalize(glm::cross(stem_front, stem_left));
    target_state->m_stem.m_spline.m_segments.emplace_back(stem_node_position, up, stem_front, stem_width, 180.f, 0, 0);
  }

  target_state->m_leaves.resize(leaf_size);
  for (int leaf_index = 0; leaf_index < leaf_size; leaf_index++) {
    const float step = static_cast<float>(leaf_index) / (static_cast<float>(leaf_size) - 1.0f);
    auto& leaf_state = target_state->m_leaves[leaf_index];
    leaf_state.m_spline.m_segments.clear();
    leaf_state.m_index = leaf_index;

    float starting_point_ratio = m_leafStartingPoint.GetValue(step);
    float leaf_length = m_leafLength.GetValue(step);
    if (leaf_length == 0.0f)
      return;

    Plot2D waviness_along_leaf = {0.0f, m_leafWaviness.GetValue(step) * 2.0f, m_wavinessAlongLeaf};
    Plot2D width_along_leaf = {0.0f, m_leafWidth.GetValue(step) * 2.0f, m_widthAlongLeaf};
    auto curling = glm::clamp(m_leafCurling.GetValue(step), 0.0f, 90.0f) / 90.0f;
    Plot2D curling_along_leaf = {0.0f, 90.0f, {curling, curling}};
    // auto curling = glm::clamp(m_leafCurling.GetValue(step), 0.0f, 90.0f) / 90.0f;
    // Plot2D curlingAlongLeaf = {0.0f, curling * 90.0f, m_curlingAlongLeaf };
    float branching_angle = m_leafBranchingAngle.GetValue(step);
    float roll_angle = glm::mod((leaf_index % 2) * 180.0f + m_leafRollAngle.GetValue(step), 360.0f);
    auto bending = m_leafBending.GetValue(step);
    bending = (bending + 180) / 360.0f;
    const auto bending_acceleration = m_leafBendingAcceleration.GetValue(step);
    const auto bending_smoothness = m_leafBendingSmoothness.GetValue(step);

    Plot2D bending_along_leaf = {-180.0f, 180.0f, {0.5f, bending}};
    const glm::vec2 middle = glm::mix(glm::vec2(0, bending), glm::vec2(1, 0.5f), bending_acceleration);
    auto& bending_along_leaf_curve = bending_along_leaf.curve.UnsafeGetValues();
    bending_along_leaf_curve.clear();
    bending_along_leaf_curve.emplace_back(-0.1, 0.0f);
    bending_along_leaf_curve.emplace_back(0, 0.5f);
    glm::vec2 left_delta = {middle.x, middle.y - 0.5f};
    bending_along_leaf_curve.push_back(left_delta * (1.0f - bending_smoothness));
    glm::vec2 right_delta = {middle.x - 1.0f, bending - middle.y};
    bending_along_leaf_curve.push_back(right_delta * (1.0f - bending_smoothness));
    bending_along_leaf_curve.emplace_back(1.0, bending);
    bending_along_leaf_curve.emplace_back(0.1, 0.0f);

    // Build nodes...
    float stem_width = width_along_stem.GetValue(starting_point_ratio);
    float back_track_ratio = 0.05f;
    if (starting_point_ratio < back_track_ratio)
      back_track_ratio = starting_point_ratio;

    glm::vec3 leaf_left = glm::normalize(glm::rotate(glm::vec3(0, 0, -1), glm::radians(roll_angle), glm::vec3(0, 1, 0)));
    auto leaf_up = glm::normalize(glm::cross(stem_front, leaf_left));
    glm::vec3 stem_offset = stem_width * -leaf_up;

    auto direction = glm::rotate(glm::vec3(0, 1, 0), glm::radians(branching_angle), leaf_left);
    float sheath_ratio = starting_point_ratio - back_track_ratio;

    if (sheath_ratio > 0) {
      int root_to_sheath_node_count = glm::min(2.0f, stem_length * sheath_ratio / sorghum_layer->vertical_subdivision_length);
      for (int i = 0; i < root_to_sheath_node_count; i++) {
        float factor = static_cast<float>(i) / root_to_sheath_node_count;
        float current_root_to_sheath_point = glm::mix(0.f, sheath_ratio, factor);

        const auto up = glm::normalize(glm::cross(stem_front, leaf_left));
        leaf_state.m_spline.m_segments.emplace_back(
            glm::normalize(stem_front) * current_root_to_sheath_point * stem_length + stem_offset, up, stem_front, stem_width,
            180.f, 0, 0);
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
      leaf_state.m_spline.m_segments.emplace_back(
          glm::normalize(stem_front) * current_sheath_point * stem_length + stem_offset, up, actual_direction,
          stem_width + 0.002f * static_cast<float>(i) / sheath_node_count,
          180.0f - 90.0f * static_cast<float>(i) / sheath_node_count, 0, 0);
    }

    int node_amount = glm::max(4.0f, leaf_length / sorghum_layer->vertical_subdivision_length);
    float unit_length = leaf_length / node_amount;

    int node_to_full_expand = 0.1f * leaf_length / sorghum_layer->vertical_subdivision_length;

    float height_offset = glm::linearRand(0.f, 100.f);
    const float waviness_frequency = m_leafWavinessFrequency.GetValue(step);
    glm::vec3 node_position = stem_front * starting_point_ratio * stem_length + stem_offset;
    for (int i = 1; i <= node_amount; i++) {
      const float factor = static_cast<float>(i) / node_amount;
      glm::vec3 current_direction;

      float rotate_angle = bending_along_leaf.GetValue(factor);
      current_direction = glm::rotate(direction, glm::radians(rotate_angle), leaf_left);
      node_position += current_direction * unit_length;

      float expand_angle = curling_along_leaf.GetValue(factor);

      float collar_factor = glm::min(1.0f, static_cast<float>(i) / node_to_full_expand);

      float waviness = waviness_along_leaf.GetValue(factor);
      height_offset += waviness_frequency;

      float width = glm::mix(stem_width + 0.002f, width_along_leaf.GetValue(factor), collar_factor);
      float angle = 90.0f - (90.0f - expand_angle) * glm::pow(collar_factor, 2.0f);

      const auto up = glm::normalize(glm::cross(current_direction, leaf_left));
      leaf_state.m_spline.m_segments.emplace_back(node_position, up, current_direction, width, angle,
                                                 waviness * glm::simplex(glm::vec2(height_offset, 0.f)),
                                                 waviness * glm::simplex(glm::vec2(0.f, height_offset)));
    }
  }

  target_state->m_panicle.m_seedAmount = m_panicleSeedAmount.GetValue();
  const auto panicle_size = m_panicleSize.GetValue();
  target_state->m_panicle.m_panicleSize = glm::vec3(panicle_size.x, panicle_size.y, panicle_size.x);
  target_state->m_panicle.m_seedRadius = m_panicleSeedRadius.GetValue();
}

void SorghumStateGenerator::OnCreate() {
  m_panicleSize.mean = glm::vec3(0.0, 0.0, 0.0);
  m_panicleSeedAmount.mean = 0;
  m_panicleSeedRadius.mean = 0.002f;

  m_stemTiltAngle.mean = 0.0f;
  m_stemTiltAngle.deviation = 0.0f;
  m_internodeLength.mean = 0.449999988f;
  m_internodeLength.deviation = 0.150000006f;
  m_stemWidth.mean = 0.0140000004;
  m_stemWidth.deviation = 0.0f;

  m_leafAmount.mean = 9.0f;
  m_leafAmount.deviation = 1.0f;

  m_leafStartingPoint.mean = {0.0f, 1.0f, Curve2D(0.1f, 1.0f)};
  m_leafStartingPoint.deviation = {0.0f, 1.0f, Curve2D(0.0f, 0.0f)};

  m_leafCurling.mean = {0.0f, 90.0f, Curve2D(0.3f, 0.7f)};
  m_leafCurling.deviation = {0.0f, 1.0f, Curve2D(0.0f, 0.0f)};
  m_leafRollAngle.mean = {-1.0f, 1.0f, Curve2D(0.5f, 0.5f)};
  m_leafRollAngle.deviation = {0.0f, 6.0f, Curve2D(0.3f, 1.0f)};

  m_leafBranchingAngle.mean = {0.0f, 55.0f, Curve2D(0.5f, 0.2f)};
  m_leafBranchingAngle.deviation = {0.0f, 3.0f, Curve2D(0.67f, 0.225f)};

  m_leafBending.mean = {-180.0f, 180.0f, Curve2D(0.5f, 0.5f)};
  m_leafBending.deviation = {0.0f, 0.0f, Curve2D(0.5f, 0.5f)};

  m_leafBendingAcceleration.mean = {0.0f, 1.0f, Curve2D(0.5f, 0.5f)};
  m_leafBendingSmoothness.mean = {0.0f, 1.0f, Curve2D(0.5f, 0.5f)};
  m_leafBendingAcceleration.deviation = {0.0f, 0.0f, Curve2D(0.5f, 0.5f)};

  m_leafWaviness.mean = {0.0f, 20.0f, Curve2D(0.5f, 0.5f)};
  m_leafWaviness.deviation = {0.0f, 0.0f, Curve2D(0.5f, 0.5f)};

  m_leafWavinessFrequency.mean = {0.0f, 1.0f, Curve2D(0.5f, 0.5f)};
  m_leafWavinessFrequency.deviation = {0.0f, 0.0f, Curve2D(0.5f, 0.5f)};

  m_leafLength.mean = {0.0f, 2.5f, Curve2D(0.165, 0.247)};
  m_leafLength.deviation = {0.0f, 0.0f, Curve2D(0.5f, 0.5f)};

  m_leafWidth.mean = {0.0f, 0.075f, Curve2D(0.5f, 0.5f)};
  m_leafWidth.deviation = {0.0f, 0.0f, Curve2D(0.5f, 0.5f)};

  m_widthAlongStem = Curve2D(1.0f, 0.1f);
  m_widthAlongLeaf = Curve2D(0.5f, 0.1f);
  m_wavinessAlongLeaf = Curve2D(0.0f, 0.5f);
}
