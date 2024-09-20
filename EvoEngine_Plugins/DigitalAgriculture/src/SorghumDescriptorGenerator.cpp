#include "SorghumDescriptorGenerator.hpp"
#include "ProjectManager.hpp"
#include "SorghumLayer.hpp"

#include "Plot2D.hpp"
#include "Scene.hpp"
#include "Sorghum.hpp"
#include "SorghumSpline.hpp"
#include "Times.hpp"

using namespace digital_agriculture_plugin;

void TipMenu(const std::string& content) {
  if (ImGui::IsItemHovered()) {
    ImGui::BeginTooltip();
    ImGui::TextUnformatted(content.c_str());
    ImGui::EndTooltip();
  }
}

bool SorghumDescriptorGenerator::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
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
    if (panicle_size.OnInspect("Size", 0.001f, "The size of panicle")) {
      changed = true;
    }
    if (panicle_seed_amount.OnInspect("Seed amount", 1.0f, "The amount of seeds in the panicle"))
      changed = true;
    if (panicle_seed_radius.OnInspect("Seed radius", 0.001f, "The size of the seed in the panicle"))
      changed = true;
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Stem settings", ImGuiTreeNodeFlags_DefaultOpen)) {
    TipMenu("The settings for stem.");
    if (stem_tilt_angle.OnInspect("Stem tilt angle", 0.001f, "The tilt angle for stem")) {
      changed = true;
    }
    if (internode_length.OnInspect("Length", 0.01f,
                                   "The length of the stem, use Ending Point in leaf settings to make "
                                   "stem taller than top leaf for panicle"))
      changed = true;
    if (stem_width.OnInspect("Width", 0.001f,
                             "The overall width of the stem, adjust the width "
                             "along stem in Stem Details"))
      changed = true;
    if (ImGui::TreeNode("Stem Details")) {
      TipMenu("The detailed settings for stem.");
      if (width_along_stem.OnInspect("Width along stem"))
        changed = true;
      ImGui::TreePop();
    }
    ImGui::TreePop();
  }
  if (ImGui::TreeNodeEx("Leaves settings", ImGuiTreeNodeFlags_DefaultOpen)) {
    TipMenu("The settings for leaves.");
    if (leaf_amount.OnInspect("Num of leaves", 1.0f, "The total amount of leaves"))
      changed = true;

    static PlottedDistributionSettings leaf_starting_point = {
        0.01f,
        {0.01f, false, true, ""},
        {0.01f, false, false, ""},
        "The starting point of each leaf along stem. Default each leaf "
        "located uniformly on stem."};

    if (this->leaf_starting_point.OnInspect("Starting point along stem", leaf_starting_point)) {
      changed = true;
    }

    static PlottedDistributionSettings leaf_curling = {
        0.01f, {0.01f, false, true, ""}, {0.01f, false, false, ""}, "The leaf curling."};

    if (this->leaf_curling.OnInspect("Leaf curling", leaf_curling)) {
      changed = true;
    }

    static PlottedDistributionSettings leaf_roll_angle = {
        0.01f,
        {},
        {},
        "The polar angle of leaf. Normally you should only change the "
        "deviation. Values are in degrees"};
    if (this->leaf_roll_angle.OnInspect("Roll angle", leaf_roll_angle))
      changed = true;

    static PlottedDistributionSettings leaf_branching_angle = {
        0.01f, {}, {}, "The branching angle of the leaf. Values are in degrees"};
    if (this->leaf_branching_angle.OnInspect("Branching angle", leaf_branching_angle))
      changed = true;

    static PlottedDistributionSettings leaf_bending = {1.0f,
                                                       {1.0f, false, true, ""},
                                                       {},
                                                       "The bending of the leaf, controls how leaves bend because of "
                                                       "gravity. Positive value results in leaf bending towards the "
                                                       "ground, negative value results in leaf bend towards the sky"};
    if (this->leaf_bending.OnInspect("Bending", leaf_bending))
      changed = true;

    static PlottedDistributionSettings leaf_bending_acceleration = {
        0.01f, {0.01f, false, true, ""}, {}, "The changes of bending along the leaf."};

    if (this->leaf_bending_acceleration.OnInspect("Bending acceleration", leaf_bending_acceleration))
      changed = true;

    static PlottedDistributionSettings leaf_bending_smoothness = {
        0.01f, {0.01f, false, true, ""}, {}, "The smoothness of bending along the leaf."};

    if (this->leaf_bending_smoothness.OnInspect("Bending smoothness", leaf_bending_smoothness))
      changed = true;

    if (leaf_waviness.OnInspect("Waviness"))
      changed = true;
    if (leaf_waviness_frequency.OnInspect("Waviness Frequency"))
      changed = true;

    if (leaf_length.OnInspect("Length"))
      changed = true;
    if (leaf_width.OnInspect("Width"))
      changed = true;

    if (ImGui::TreeNode("Leaf Details")) {
      if (width_along_leaf.OnInspect("Width along leaf"))
        changed = true;
      if (waviness_along_leaf.OnInspect("Waviness along leaf"))
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
void SorghumDescriptorGenerator::Serialize(YAML::Emitter& out) const {
  panicle_size.Save("panicle_size", out);
  panicle_seed_amount.Save("panicle_seed_amount", out);
  panicle_seed_radius.Save("panicle_seed_radius", out);

  stem_tilt_angle.Save("stem_tilt_angle", out);
  internode_length.Save("internode_length", out);
  stem_width.Save("stem_width", out);

  leaf_amount.Save("leaf_amount", out);
  leaf_starting_point.Save("leaf_starting_point", out);
  leaf_curling.Save("leaf_curling", out);

  leaf_roll_angle.Save("leaf_roll_angle", out);
  leaf_branching_angle.Save("leaf_branching_angle", out);

  leaf_bending.Save("leaf_bending", out);
  leaf_bending_acceleration.Save("leaf_bending_acceleration", out);
  leaf_bending_smoothness.Save("leaf_bending_smoothness", out);
  leaf_waviness.Save("leaf_waviness", out);
  leaf_waviness_frequency.Save("leaf_waviness_frequency", out);
  leaf_length.Save("leaf_length", out);
  leaf_width.Save("leaf_width", out);

  width_along_stem.Save("width_along_stem", out);
  width_along_leaf.Save("width_along_leaf", out);
  waviness_along_leaf.Save("waviness_along_leaf", out);
}
void SorghumDescriptorGenerator::Deserialize(const YAML::Node& in) {
  panicle_size.Load("panicle_size", in);
  panicle_seed_amount.Load("panicle_seed_amount", in);
  panicle_seed_radius.Load("panicle_seed_radius", in);

  stem_tilt_angle.Load("stem_tilt_angle", in);
  internode_length.Load("internode_length", in);
  stem_width.Load("stem_width", in);

  leaf_amount.Load("leaf_amount", in);
  leaf_starting_point.Load("leaf_starting_point", in);
  leaf_curling.Load("leaf_curling", in);

  leaf_roll_angle.Load("leaf_roll_angle", in);
  leaf_branching_angle.Load("leaf_branching_angle", in);

  leaf_bending.Load("leaf_bending", in);
  leaf_bending_acceleration.Load("leaf_bending_acceleration", in);
  leaf_bending_smoothness.Load("leaf_bending_smoothness", in);
  leaf_waviness.Load("leaf_waviness", in);
  leaf_waviness_frequency.Load("leaf_waviness_frequency", in);
  leaf_length.Load("leaf_length", in);
  leaf_width.Load("leaf_width", in);

  width_along_stem.Load("width_along_stem", in);
  width_along_leaf.Load("width_along_leaf", in);
  waviness_along_leaf.Load("waviness_along_leaf", in);
}

Entity SorghumDescriptorGenerator::CreateEntity(const unsigned int seed) const {
  const auto scene = Application::GetActiveScene();
  const auto entity = scene->CreateEntity(GetTitle());
  const auto sorghum = scene->GetOrSetPrivateComponent<Sorghum>(entity).lock();
  const auto sorghum_state = ProjectManager::CreateTemporaryAsset<SorghumDescriptor>();
  Apply(sorghum_state, seed);
  sorghum->sorghum_descriptor = sorghum_state;
  sorghum->sorghum_state_generator = GetSelf();
  sorghum->GenerateGeometryEntities(SorghumMeshGeneratorSettings{});
  return entity;
}

void SorghumDescriptorGenerator::Apply(const std::shared_ptr<SorghumDescriptor>& target_state, const unsigned int seed) const {
  if (seed > 0)
    srand(seed);

  constexpr auto up_direction = glm::vec3(0, 1, 0);
  auto front_direction = glm::vec3(0, 0, -1);
  front_direction = glm::rotate(front_direction, glm::radians(glm::linearRand(0.0f, 360.0f)), up_direction);

  glm::vec3 stem_front = glm::normalize(glm::rotate(
      up_direction, glm::radians(glm::gaussRand(stem_tilt_angle.mean, stem_tilt_angle.deviation)), front_direction));
  const int leaf_size = glm::clamp(leaf_amount.GetValue(), 2.0f, 128.0f);
  float stem_length = internode_length.GetValue() * leaf_size / (1.f - leaf_starting_point.GetValue(0));
  Plot2D width_along_stem = {0.0f, stem_width.GetValue(), this->width_along_stem};
  const auto sorghum_layer = Application::GetLayer<SorghumLayer>();
  // Build stem...
  target_state->stem.spline.segments.clear();
  int stem_node_amount = static_cast<int>(glm::max(4.0f, stem_length / sorghum_layer->vertical_subdivision_length));
  float stem_unit_length = stem_length / stem_node_amount;
  glm::vec3 stem_left =
      glm::normalize(glm::rotate(glm::vec3(1, 0, 0), glm::radians(glm::linearRand(0.0f, 0.0f)), stem_front));
  for (int i = 0; i <= stem_node_amount; i++) {
    float stem_width = width_along_stem.GetValue(static_cast<float>(i) / stem_node_amount);
    glm::vec3 stem_node_position;
    stem_node_position = stem_front * stem_unit_length * static_cast<float>(i);

    const auto up = glm::normalize(glm::cross(stem_front, stem_left));
    target_state->stem.spline.segments.emplace_back(stem_node_position, up, stem_front, stem_width, 180.f, 0, 0);
  }

  target_state->leaves.resize(leaf_size);
  for (int leaf_index = 0; leaf_index < leaf_size; leaf_index++) {
    const float step = static_cast<float>(leaf_index) / (static_cast<float>(leaf_size) - 1.0f);
    auto& leaf_state = target_state->leaves[leaf_index];
    leaf_state.spline.segments.clear();
    leaf_state.index = leaf_index;

    float starting_point_ratio = leaf_starting_point.GetValue(step);
    float leaf_length = this->leaf_length.GetValue(step);
    if (leaf_length == 0.0f)
      return;

    Plot2D waviness_along_leaf = {0.0f, leaf_waviness.GetValue(step) * 2.0f, this->waviness_along_leaf};
    Plot2D width_along_leaf = {0.0f, leaf_width.GetValue(step) * 2.0f, this->width_along_leaf};
    auto curling = glm::clamp(leaf_curling.GetValue(step), 0.0f, 90.0f) / 90.0f;
    Plot2D curling_along_leaf = {0.0f, 90.0f, {curling, curling}};
    // auto curling = glm::clamp(leaf_curling.GetValue(step), 0.0f, 90.0f) / 90.0f;
    // Plot2D curlingAlongLeaf = {0.0f, curling * 90.0f, curling_along_leaf };
    float branching_angle = leaf_branching_angle.GetValue(step);
    float roll_angle = glm::mod((leaf_index % 2) * 180.0f + leaf_roll_angle.GetValue(step), 360.0f);
    auto bending = leaf_bending.GetValue(step);
    bending = (bending + 180) / 360.0f;
    const auto bending_acceleration = leaf_bending_acceleration.GetValue(step);
    const auto bending_smoothness = leaf_bending_smoothness.GetValue(step);

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

    glm::vec3 leaf_left =
        glm::normalize(glm::rotate(glm::vec3(0, 0, -1), glm::radians(roll_angle), glm::vec3(0, 1, 0)));
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
    const float waviness_frequency = leaf_waviness_frequency.GetValue(step);
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
      leaf_state.spline.segments.emplace_back(node_position, up, current_direction, width, angle,
                                                  waviness * glm::simplex(glm::vec2(height_offset, 0.f)),
                                                  waviness * glm::simplex(glm::vec2(0.f, height_offset)));
    }
  }

  target_state->panicle.seed_amount = panicle_seed_amount.GetValue();
  const auto panicle_size = this->panicle_size.GetValue();
  target_state->panicle.panicle_size = glm::vec3(panicle_size.x, panicle_size.y, panicle_size.x);
  target_state->panicle.seed_radius = panicle_seed_radius.GetValue();
}

void SorghumDescriptorGenerator::OnCreate() {
  panicle_size.mean = glm::vec3(0.0, 0.0, 0.0);
  panicle_seed_amount.mean = 0;
  panicle_seed_radius.mean = 0.002f;

  stem_tilt_angle.mean = 0.0f;
  stem_tilt_angle.deviation = 0.0f;
  internode_length.mean = 0.449999988f;
  internode_length.deviation = 0.150000006f;
  stem_width.mean = 0.0140000004;
  stem_width.deviation = 0.0f;

  leaf_amount.mean = 9.0f;
  leaf_amount.deviation = 1.0f;

  leaf_starting_point.mean = {0.0f, 1.0f, Curve2D(0.1f, 1.0f)};
  leaf_starting_point.deviation = {0.0f, 1.0f, Curve2D(0.0f, 0.0f)};

  leaf_curling.mean = {0.0f, 90.0f, Curve2D(0.3f, 0.7f)};
  leaf_curling.deviation = {0.0f, 1.0f, Curve2D(0.0f, 0.0f)};
  leaf_roll_angle.mean = {-1.0f, 1.0f, Curve2D(0.5f, 0.5f)};
  leaf_roll_angle.deviation = {0.0f, 6.0f, Curve2D(0.3f, 1.0f)};

  leaf_branching_angle.mean = {0.0f, 55.0f, Curve2D(0.5f, 0.2f)};
  leaf_branching_angle.deviation = {0.0f, 3.0f, Curve2D(0.67f, 0.225f)};

  leaf_bending.mean = {-180.0f, 180.0f, Curve2D(0.5f, 0.5f)};
  leaf_bending.deviation = {0.0f, 0.0f, Curve2D(0.5f, 0.5f)};

  leaf_bending_acceleration.mean = {0.0f, 1.0f, Curve2D(0.5f, 0.5f)};
  leaf_bending_smoothness.mean = {0.0f, 1.0f, Curve2D(0.5f, 0.5f)};
  leaf_bending_acceleration.deviation = {0.0f, 0.0f, Curve2D(0.5f, 0.5f)};

  leaf_waviness.mean = {0.0f, 20.0f, Curve2D(0.5f, 0.5f)};
  leaf_waviness.deviation = {0.0f, 0.0f, Curve2D(0.5f, 0.5f)};

  leaf_waviness_frequency.mean = {0.0f, 1.0f, Curve2D(0.5f, 0.5f)};
  leaf_waviness_frequency.deviation = {0.0f, 0.0f, Curve2D(0.5f, 0.5f)};

  leaf_length.mean = {0.0f, 2.5f, Curve2D(0.165, 0.247)};
  leaf_length.deviation = {0.0f, 0.0f, Curve2D(0.5f, 0.5f)};

  leaf_width.mean = {0.0f, 0.075f, Curve2D(0.5f, 0.5f)};
  leaf_width.deviation = {0.0f, 0.0f, Curve2D(0.5f, 0.5f)};

  width_along_stem = Curve2D(1.0f, 0.1f);
  width_along_leaf = Curve2D(0.5f, 0.1f);
  waviness_along_leaf = Curve2D(0.0f, 0.5f);
}
