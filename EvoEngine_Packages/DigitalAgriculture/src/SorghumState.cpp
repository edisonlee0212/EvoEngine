//
// Created by lllll on 1/8/2022.
//
#include "Application.hpp"
#include "EditorLayer.hpp"
#include "Scene.hpp"

#include "Sorghum.hpp"
#include "SorghumGenerator.hpp"
#include "SorghumGrowthStages.hpp"
#include "SorghumLayer.hpp"
#include "Times.hpp"
#include "Utilities.hpp"
#include "rapidcsv.h"
using namespace digital_agriculture_package;

bool SorghumPanicleState::OnInspectImpl() {
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

void SorghumPanicleState::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "panicle_size" << YAML::Value << panicle_size;
  out << YAML::Key << "seed_amount" << YAML::Value << seed_amount;
  out << YAML::Key << "seed_radius" << YAML::Value << seed_radius;
}

void SorghumPanicleState::Deserialize(const YAML::Node& in) {
  if (in["panicle_size"])
    panicle_size = in["panicle_size"].as<glm::vec3>();
  if (in["seed_amount"])
    seed_amount = in["seed_amount"].as<int>();
  if (in["seed_radius"])
    seed_radius = in["seed_radius"].as<float>();
  saved = true;
}

void SorghumPanicleState::Apply(SorghumPanicleDescriptor& target_sorghum_panicle_descriptor) const {
  target_sorghum_panicle_descriptor.panicle_size = panicle_size;
  target_sorghum_panicle_descriptor.seed_amount = seed_amount;
  target_sorghum_panicle_descriptor.seed_radius = seed_radius;
}

SorghumPanicleState::SorghumPanicleState() {
  panicle_size = glm::vec3(0, 0, 0);
  seed_amount = 0;
  seed_radius = 0.002f;
  saved = false;
}

glm::vec3 SorghumStemState::GetPoint(const float point) const {
  return direction * point * length;
}

void SorghumStemState::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "direction" << YAML::Value << direction;
  width_along_stem.Save("width_along_stem", out);
  out << YAML::Key << "length" << YAML::Value << length;
  out << YAML::Key << "spline" << YAML::Value << YAML::BeginMap;
  spline.Serialize(out);
  out << YAML::EndMap;
}

void SorghumStemState::Deserialize(const YAML::Node& in) {
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

bool SorghumStemState::OnInspectImpl(int mode) {
  bool changed = false;
  switch (static_cast<StateMode>(mode)) {
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

void SorghumStemState::Apply(SorghumStemDescriptor& target_sorghum_stem_descriptor) const {
  target_sorghum_stem_descriptor.spline.segments.clear();
  const auto sorghum_layer = ApplicationContext::Get().GetLayer<SorghumLayer>();
  const int stem_node_amount = static_cast<int>(glm::max(4.0f, length / sorghum_layer->vertical_subdivision_length));
  const float stem_unit_length = length / static_cast<float>(stem_node_amount);
  const glm::vec3 stem_left =
      glm::normalize(glm::rotate(glm::vec3(1, 0, 0), glm::radians(glm::linearRand(0.0f, 0.0f)), direction));
  for (int i = 0; i <= stem_node_amount; i++) {
    float stem_width = width_along_stem.GetValue(static_cast<float>(i) / static_cast<float>(stem_node_amount));
    glm::vec3 stem_node_position;
    stem_node_position = direction * stem_unit_length * static_cast<float>(i);

    const auto up = glm::normalize(glm::cross(direction, stem_left));
    target_sorghum_stem_descriptor.spline.segments.emplace_back(stem_node_position, up, direction, stem_width, 180.f, 0,
                                                                0);
  }
}

bool SorghumLeafState::OnInspectImpl(int mode) {
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
    switch (static_cast<StateMode>(mode)) {
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

void SorghumLeafState::Apply(const SorghumStemState& stem_state,
                             SorghumLeafDescriptor& target_sorghum_leaf_descriptor) const {
  target_sorghum_leaf_descriptor.spline.segments.clear();
  target_sorghum_leaf_descriptor.index = index;
  float stem_width = stem_state.width_along_stem.GetValue(starting_point);
  float back_track_ratio = 0.05f;
  if (starting_point < back_track_ratio)
    back_track_ratio = starting_point;
  const auto sorghum_layer = ApplicationContext::Get().GetLayer<SorghumLayer>();
  glm::vec3 leaf_left = glm::normalize(glm::rotate(glm::vec3(0, 0, -1), glm::radians(roll_angle), glm::vec3(0, 1, 0)));
  auto leaf_up = glm::normalize(glm::cross(stem_state.direction, leaf_left));
  glm::vec3 stem_offset = stem_width * -leaf_up;

  auto direction = glm::rotate(glm::vec3(0, 1, 0), glm::radians(branching_angle), leaf_left);
  float sheath_ratio = starting_point - back_track_ratio;

  if (sheath_ratio > 0) {
    int root_to_sheath_node_count =
        static_cast<int>(glm::min(2.0f, stem_state.length * sheath_ratio / sorghum_layer->vertical_subdivision_length));
    for (int i = 0; i < root_to_sheath_node_count; i++) {
      float factor = static_cast<float>(i) / static_cast<float>(root_to_sheath_node_count);
      float current_root_to_sheath_point = glm::mix(0.f, sheath_ratio, factor);

      const auto up = glm::normalize(glm::cross(stem_state.direction, leaf_left));
      target_sorghum_leaf_descriptor.spline.segments.emplace_back(
          glm::normalize(stem_state.direction) * current_root_to_sheath_point * stem_state.length + stem_offset, up,
          stem_state.direction, stem_width, 180.f, 0, 0);
    }
  }
  /*
  int sheath_node_count =
      static_cast<int>(glm::max(2.0f, stem_state.length * back_track_ratio /
  sorghum_layer->vertical_subdivision_length));

  for (int i = 0; i <= sheath_node_count; i++) {
    float factor = static_cast<float>(i) / static_cast<float>(sheath_node_count);
    float current_sheath_point =
        glm::mix(sheath_ratio, starting_point,
                 factor);  // sheathRatio + static_cast<float>(i) / sheathNodeCount * backTrackRatio;
    glm::vec3 actual_direction = glm::normalize(glm::mix(stem_state.direction, direction, factor));

    const auto up = glm::normalize(glm::cross(actual_direction, leaf_left));
    target_sorghum_leaf_descriptor.spline.segments.emplace_back(
        glm::normalize(stem_state.direction) * current_sheath_point * stem_state.length + stem_offset, up,
  actual_direction, stem_width + 0.002f * static_cast<float>(i) / static_cast<float>(sheath_node_count), 180.0f - 90.0f
  * static_cast<float>(i) / static_cast<float>(sheath_node_count), 0, 0);
  }
  */
  int node_amount = static_cast<int>(glm::max(4.0f, length / sorghum_layer->vertical_subdivision_length));
  float unit_length = length / static_cast<float>(node_amount);

  int node_to_full_expand = static_cast<int>(0.1f * length / sorghum_layer->vertical_subdivision_length);

  auto current_period = waviness_period_start;

  glm::vec3 node_position = stem_state.direction * starting_point * stem_state.length + stem_offset;
  for (int i = 1; i <= node_amount; i++) {
    const float factor = static_cast<float>(i) / static_cast<float>(node_amount);
    glm::vec3 current_direction;

    float rotate_angle = bending_along_leaf.GetValue(factor);
    current_direction = glm::rotate(direction, glm::radians(rotate_angle), leaf_left);
    node_position += current_direction * unit_length;

    float expand_angle = curling_along_leaf.GetValue(factor);

    float collar_factor = glm::min(1.0f, static_cast<float>(i) / static_cast<float>(node_to_full_expand));

    float waviness = waviness_along_leaf.GetValue(factor);
    current_period += glm::vec2(waviness_frequency, waviness_frequency);

    float width = glm::mix(stem_width + 0.002f, width_along_leaf.GetValue(factor), collar_factor);
    float angle = 90.0f - (90.0f - expand_angle) * glm::pow(collar_factor, 2.0f);

    const auto up = glm::normalize(glm::cross(current_direction, leaf_left));
    target_sorghum_leaf_descriptor.spline.segments.emplace_back(
        node_position, up, current_direction, width, angle, waviness * glm::simplex(glm::vec2(current_period.x, 0.f)),
        waviness * glm::simplex(glm::vec2(0.f, current_period.y)));
  }
}

void SorghumLeafState::Serialize(YAML::Emitter& out) const {
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

void SorghumLeafState::Deserialize(const YAML::Node& in) {
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

SorghumStemState::SorghumStemState() {
  length = 0.35f;
  width_along_stem = {0.0f, 0.015f, {0.6f, 0.4f, {0, 0}, {1, 1}}};

  saved = false;
}

SorghumLeafState::SorghumLeafState() {
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

void SorghumLeafState::CopyShape(const SorghumLeafState& another) {
  spline = another.spline;
  width_along_leaf.curve = another.width_along_leaf.curve;
  curling_along_leaf = another.curling_along_leaf;
  bending_along_leaf = another.bending_along_leaf;
  waviness_along_leaf = another.waviness_along_leaf;
  waviness_period_start = another.waviness_period_start;
  waviness_frequency = another.waviness_frequency;

  saved = false;
}

bool SorghumState::OnInspectImpl(int mode) {
  bool changed = false;
  if (ImGui::TreeNodeEx((std::string("Stem")).c_str())) {
    if (stem.OnInspectImpl(mode))
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
            leaves[i] = SorghumLeafState();
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
        if (leaf.OnInspectImpl(mode))
          changed = true;
        ImGui::TreePop();
      }
    }
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx((std::string("Panicle")).c_str())) {
    if (panicle.OnInspectImpl())
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
          stem = SorghumStemState();
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
            leaves[i] = SorghumLeafState();
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

bool SorghumState::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::Button("Instantiate")) {
    const auto new_entity = CreateEntity("New Sorghum");
  }

  static float target_waviness_factor = 1.0f;
  ImGui::DragFloat("Target leaf waviness", &target_waviness_factor, 0.01f, 0.01f, 3.0f);
  if (ImGui::Button("Create sorghum with changed waviness")) {
    const auto scene = ApplicationContext::Get().GetActiveScene();
    const auto sorghum_entity = scene->CreateEntity(GetTitle());
    const auto sorghum = scene->GetOrSetPrivateComponent<Sorghum>(sorghum_entity).lock();
    const auto new_sorghum_state = AssetManager::CreateTemporaryAsset<SorghumState>();
    SorghumMeshGeneratorSettings settings{};
    settings.enable_leaf_sheath = false;
    settings.bottom_face = false;
    ChangeWaviness(target_waviness_factor, settings, *new_sorghum_state);

    sorghum->sorghum_state = new_sorghum_state;
    if (const auto sorghum_layer = ApplicationContext::Get().GetLayer<SorghumLayer>()) {
      sorghum->GenerateGeometryEntities(sorghum_layer->sorghum_mesh_generator_settings);
    } else {
      sorghum->GenerateGeometryEntities({});
    }
  }

  static int state_mode = static_cast<int>(StateMode::Default);
  static const char* state_modes[]{"Default", "Cubic-Bezier"};
  if (ImGui::Combo("Mode", &state_mode, state_modes, IM_ARRAYSIZE(state_modes))) {
    changed = false;
  }
  OnInspectImpl(state_mode);
  return changed;
}

void SorghumState::Apply(const std::shared_ptr<SorghumDescriptor>& target_sorghum_descriptor) const {
  panicle.Apply(target_sorghum_descriptor->panicle);
  stem.Apply(target_sorghum_descriptor->stem);
  target_sorghum_descriptor->leaves.resize(leaves.size());
  for (int leaf_index = 0; leaf_index < leaves.size(); leaf_index++) {
    leaves[leaf_index].Apply(stem, target_sorghum_descriptor->leaves[leaf_index]);
  }
}

void SorghumState::Serialize(YAML::Emitter& out) const {
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

void SorghumState::Deserialize(const YAML::Node& in) {
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
      SorghumLeafState leaf;
      leaf.Deserialize(i);
      leaves.push_back(leaf);
    }
  }
}

Entity SorghumState::CreateEntity(const std::string& name) const {
  const auto scene = ApplicationContext::Get().GetActiveScene();
  const auto sorghum_entity = scene->CreateEntity(name);
  const auto sorghum = scene->GetOrSetPrivateComponent<Sorghum>(sorghum_entity).lock();
  sorghum->sorghum_state = GetSelf();
  if (const auto sorghum_layer = ApplicationContext::Get().GetLayer<SorghumLayer>()) {
    sorghum->GenerateGeometryEntities(sorghum_layer->sorghum_mesh_generator_settings);
  } else {
    sorghum->GenerateGeometryEntities({});
  }
  return sorghum_entity;
}

std::shared_ptr<Texture2D> SorghumState::GenerateThumbnailTexture() {
  static std::shared_ptr<Texture2D> thumbnail;
  if (!thumbnail) {
    thumbnail = AssetManager::CreateTemporaryAsset<Texture2D>();
    thumbnail->Import(std::filesystem::absolute(std::filesystem::path("./DigitalAgricultureResources") /
                                                "Icons/SorghumDescriptor.png"));
  }
  return thumbnail;
}

void SorghumState::ChangeWaviness(const float factor, const SorghumMeshGeneratorSettings& mesh_generator_settings,
                                  SorghumState& target_sorghum_state) const {
  target_sorghum_state = *this;
  for (int leaf_index = 0; leaf_index < target_sorghum_state.leaves.size(); leaf_index++) {
    leaves[leaf_index].ChangeWaviness(factor, stem, mesh_generator_settings, target_sorghum_state.leaves[leaf_index]);
  }
}

void SorghumLeafState::ChangeWaviness(const float factor, const SorghumStemState& stem_state,
                                      const SorghumMeshGeneratorSettings& mesh_generator_settings,
                                      SorghumLeafState& target_leaf_state) const {
  std::vector<Vertex> vertices;
  std::vector<unsigned int> indices;
  const auto calculate_total_area = [&]() {
    float ret_val = 0.0f;
    for (int i = 0; i < indices.size() / 3; i++) {
      const auto& v0 = vertices[indices[i * 3]];
      const auto& v1 = vertices[indices[i * 3 + 1]];
      const auto& v2 = vertices[indices[i * 3 + 2]];

      const float a = glm::distance(v0.position, v1.position);
      const float b = glm::distance(v1.position, v2.position);
      const float c = glm::distance(v2.position, v0.position);
      const float p = (a + b + c) * 0.5f;
      ret_val += glm::sqrt(p * (p - a) * (p - b) * (p - c));
    }
    return ret_val;
  };

  SorghumLeafDescriptor sorghum_leaf_descriptor;

  Apply(stem_state, sorghum_leaf_descriptor);
  sorghum_leaf_descriptor.GenerateGeometry(vertices, indices, mesh_generator_settings);

  const float target_total_area = calculate_total_area();

  float width_factor_upper_bound = 2.f;
  float width_factor_lower_bound = 0.f;

  for (int iteration = 0; iteration < 16; iteration++) {
    const float mid_width_factor = (width_factor_lower_bound + width_factor_upper_bound) * .5f;
    target_leaf_state = *this;
    target_leaf_state.waviness_along_leaf.max_value = waviness_along_leaf.max_value * factor;
    target_leaf_state.waviness_along_leaf.min_value = waviness_along_leaf.min_value * factor;
    target_leaf_state.width_along_leaf.max_value = width_along_leaf.max_value * mid_width_factor;
    target_leaf_state.width_along_leaf.min_value = width_along_leaf.min_value * mid_width_factor;
    target_leaf_state.Apply(stem_state, sorghum_leaf_descriptor);

    vertices.clear();
    indices.clear();
    sorghum_leaf_descriptor.GenerateGeometry(vertices, indices, mesh_generator_settings);

    if (calculate_total_area() >= target_total_area) {
      width_factor_upper_bound = mid_width_factor;
    } else {
      width_factor_lower_bound = mid_width_factor;
    }
  }
}

SorghumState::SorghumState() {
  saved = false;
  name = "Unnamed";
}
