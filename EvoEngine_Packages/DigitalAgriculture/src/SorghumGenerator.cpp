#include "SorghumGenerator.hpp"
#include "Application.hpp"
#include "ProjectManager.hpp"
#include "SorghumLayer.hpp"

#include "Plot2D.hpp"
#include "Scene.hpp"
#include "Sorghum.hpp"
#include "SorghumSpline.hpp"
#include "Times.hpp"

using namespace digital_agriculture_package;

void TipMenu(const std::string& content) {
  if (ImGui::IsItemHovered()) {
    ImGui::BeginTooltip();
    ImGui::TextUnformatted(content.c_str());
    ImGui::EndTooltip();
  }
}

bool SorghumGenerator::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
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

    if (ImGui::TreeNode("Per leaf settings")) {
      if (ImGui::TreeNode("Width along leaf")) {
        if (width_along_leaf.OnInspect("Width along leaf"))
          changed = true;
        ImGui::TreePop();
      }
      if (ImGui::TreeNode("Waviness along leaf")) {
        if (waviness_along_leaf.OnInspect("Waviness along leaf"))
          changed = true;
        ImGui::TreePop();
      }
      if (ImGui::TreeNode("Curling along leaf")) {
        if (curling_along_leaf.OnInspect("Curling along leaf"))
          changed = true;
        ImGui::TreePop();
      }
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
      last_auto_save_time = ApplicationContext::Get().GetTimes().Now();
    } else if (last_auto_save_time + auto_save_interval < ApplicationContext::Get().GetTimes().Now()) {
      last_auto_save_time = ApplicationContext::Get().GetTimes().Now();
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
void SorghumGenerator::Serialize(YAML::Emitter& out) const {
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
  curling_along_leaf.Save("curling_along_leaf", out);
}
void SorghumGenerator::Deserialize(const YAML::Node& in) {
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
  curling_along_leaf.Load("curling_along_leaf", in);
}

std::shared_ptr<Texture2D> SorghumGenerator::GenerateThumbnailTexture() {
  static std::shared_ptr<Texture2D> thumbnail;
  if (!thumbnail) {
    thumbnail = AssetManager::CreateTemporaryAsset<Texture2D>();
    thumbnail->Import(std::filesystem::absolute(std::filesystem::path("./DigitalAgricultureResources") /
                                                "Icons/SorghumGenerator.png"));
  }
  return thumbnail;
}

Entity SorghumGenerator::CreateEntity(const unsigned int seed) const {
  const auto scene = ApplicationContext::Get().GetActiveScene();
  const auto entity = scene->CreateEntity(GetTitle());
  const auto sorghum = scene->GetOrSetPrivateComponent<Sorghum>(entity).lock();
  const auto sorghum_state = AssetManager::CreateTemporaryAsset<SorghumState>();
  Apply(sorghum_state, seed);
  sorghum->sorghum_state = sorghum_state;
  sorghum->sorghum_generator = GetSelf();
  sorghum->GenerateGeometryEntities(SorghumMeshGeneratorSettings{});
  return entity;
}

void SorghumGenerator::Apply(const std::shared_ptr<SorghumDescriptor>& target_sorghum_descriptor,
                             const unsigned int seed) const {
  const auto sorghum_state = AssetManager::CreateTemporaryAsset<SorghumState>();
  Apply(sorghum_state, seed);
  sorghum_state->Apply(target_sorghum_descriptor);
}

void SorghumGenerator::Apply(const std::shared_ptr<SorghumState>& target_sorghum_state, const unsigned seed) const {
  if (seed > 0)
    srand(seed);
  // Panicle
  target_sorghum_state->panicle.seed_amount = static_cast<int>(panicle_seed_amount.GetValue());
  const auto current_panicle_size = this->panicle_size.GetValue();
  target_sorghum_state->panicle.panicle_size =
      glm::vec3(current_panicle_size.x, current_panicle_size.y, current_panicle_size.x);
  target_sorghum_state->panicle.seed_radius = panicle_seed_radius.GetValue();
  // Stem
  constexpr auto up_direction = glm::vec3(0, 1, 0);
  auto front_direction = glm::vec3(0, 0, -1);
  front_direction = glm::rotate(front_direction, glm::radians(glm::linearRand(0.0f, 360.0f)), up_direction);

  target_sorghum_state->stem.direction = glm::normalize(glm::rotate(
      up_direction, glm::radians(glm::gaussRand(stem_tilt_angle.mean, stem_tilt_angle.deviation)), front_direction));
  const int leaf_size = static_cast<int>(glm::clamp(leaf_amount.GetValue(), 2.0f, 128.0f));
  target_sorghum_state->stem.length =
      internode_length.GetValue() * static_cast<float>(leaf_size) / (1.f - leaf_starting_point.GetValue(0));
  target_sorghum_state->stem.width_along_stem = {0.0f, stem_width.GetValue(), width_along_stem};
  // Leaves
  target_sorghum_state->leaves.resize(leaf_size);
  for (int leaf_index = 0; leaf_index < leaf_size; leaf_index++) {
    const float step = static_cast<float>(leaf_index) / (static_cast<float>(leaf_size) - 1.0f);
    auto& leaf_state = target_sorghum_state->leaves[leaf_index];
    leaf_state.index = leaf_index;
    leaf_state.starting_point = leaf_starting_point.GetValue(step);
    leaf_state.length = leaf_length.GetValue(step);
    if (leaf_state.length == 0.0f)
      continue;

    leaf_state.waviness_along_leaf = {0.0f, leaf_waviness.GetValue(step) * 2.0f, waviness_along_leaf};
    leaf_state.width_along_leaf = {0.0f, leaf_width.GetValue(step) * 2.0f, width_along_leaf};
    const auto curling = glm::clamp(leaf_curling.GetValue(step), 0.0f, 90.0f) / 90.0f;
    // leaf_state.curling_along_leaf = {0.0f, curling * 90.0f, curling_along_leaf};
    leaf_state.curling_along_leaf = {0.0f, curling * 90.0f, {1.f, 1.f}};
    leaf_state.branching_angle = leaf_branching_angle.GetValue(step);
    leaf_state.roll_angle = glm::mod((leaf_index % 2) * 180.0f + leaf_roll_angle.GetValue(step), 360.0f);
    auto bending = leaf_bending.GetValue(step);
    bending = (bending + 180) / 360.0f;
    const auto bending_acceleration = leaf_bending_acceleration.GetValue(step);
    const auto bending_smoothness = leaf_bending_smoothness.GetValue(step);

    leaf_state.bending_along_leaf = {-180.0f, 180.0f, {0.5f, bending}};
    const glm::vec2 middle = glm::mix(glm::vec2(0, bending), glm::vec2(1, 0.5f), bending_acceleration);
    auto& bending_along_leaf_curve = leaf_state.bending_along_leaf.curve.UnsafeGetValues();
    bending_along_leaf_curve.clear();
    bending_along_leaf_curve.emplace_back(-0.1, 0.0f);
    bending_along_leaf_curve.emplace_back(0, 0.5f);
    glm::vec2 left_delta = {middle.x, middle.y - 0.5f};
    bending_along_leaf_curve.push_back(left_delta * (1.0f - bending_smoothness));
    glm::vec2 right_delta = {middle.x - 1.0f, bending - middle.y};
    bending_along_leaf_curve.push_back(right_delta * (1.0f - bending_smoothness));
    bending_along_leaf_curve.emplace_back(1.0, bending);
    bending_along_leaf_curve.emplace_back(0.1, 0.0f);

    leaf_state.waviness_frequency = leaf_waviness_frequency.GetValue(step);
    leaf_state.waviness_period_start = glm::vec2(glm::linearRand(0.f, 100.f), glm::linearRand(0.f, 100.f));
  }
}

void SorghumGenerator::OnCreate() {
  panicle_size.mean = glm::vec3(0.0, 0.0, 0.0);
  panicle_seed_amount.mean = 0;
  panicle_seed_radius.mean = 0.002f;

  stem_tilt_angle.mean = 0.0f;
  stem_tilt_angle.deviation = 0.0f;
  internode_length.mean = 0.449999988f;
  internode_length.deviation = 0.150000006f;
  stem_width.mean = 0.0140000004f;
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

  leaf_length.mean = {0.0f, 2.5f, Curve2D(0.165f, 0.247f)};
  leaf_length.deviation = {0.0f, 0.0f, Curve2D(0.5f, 0.5f)};

  leaf_width.mean = {0.0f, 0.075f, Curve2D(0.5f, 0.5f)};
  leaf_width.deviation = {0.0f, 0.0f, Curve2D(0.5f, 0.5f)};

  width_along_stem = Curve2D(1.0f, 0.1f);
  width_along_leaf = Curve2D(0.5f, 0.1f);
  waviness_along_leaf = Curve2D(0.0f, 0.5f);
}
