#include "Application.hpp"
#include "CurveEditors.hpp"
#include "DigitalAgricultureInspectionAdapters.hpp"
#include "DigitalAgricultureInspectorStates.hpp"
#include "DigitalAgricultureSerializationAdapters.hpp"
#include "Plot2D.hpp"
#include "ProjectManager.hpp"
#include "Scene.hpp"
#include "Sorghum.hpp"
#include "SorghumLayer.hpp"
#include "SorghumSpline.hpp"
#include "Times.hpp"
using namespace digital_agriculture_package;
namespace {
void TipMenu(const std::string& content) {
  if (ImGui::IsItemHovered()) {
    ImGui::BeginTooltip();
    ImGui::TextUnformatted(content.c_str());
    ImGui::EndTooltip();
  }
}
}  // namespace
bool SorghumGeneratorInspector::Inspect(InspectorContext& context, SorghumGenerator& generator) {
  (void)context;
  auto& panicle_size = generator.panicle_size;
  auto& panicle_seed_amount = generator.panicle_seed_amount;
  auto& panicle_seed_radius = generator.panicle_seed_radius;
  auto& stem_tilt_angle = generator.stem_tilt_angle;
  auto& internode_length = generator.internode_length;
  auto& stem_width = generator.stem_width;
  auto& leaf_amount = generator.leaf_amount;
  auto& leaf_starting_point = generator.leaf_starting_point;
  auto& leaf_curling = generator.leaf_curling;
  auto& leaf_roll_angle = generator.leaf_roll_angle;
  auto& leaf_branching_angle = generator.leaf_branching_angle;
  auto& leaf_bending = generator.leaf_bending;
  auto& leaf_bending_acceleration = generator.leaf_bending_acceleration;
  auto& leaf_bending_smoothness = generator.leaf_bending_smoothness;
  auto& leaf_waviness = generator.leaf_waviness;
  auto& leaf_waviness_frequency = generator.leaf_waviness_frequency;
  auto& leaf_length = generator.leaf_length;
  auto& leaf_width = generator.leaf_width;
  auto& width_along_stem = generator.width_along_stem;
  auto& curling_along_leaf = generator.curling_along_leaf;
  auto& width_along_leaf = generator.width_along_leaf;
  auto& waviness_along_leaf = generator.waviness_along_leaf;
  if (ImGui::Button("Instantiate")) {
    auto entity = generator.CreateEntity();
  }
  auto& auto_save = ui_auto_save;
  ImGui::Checkbox("Auto save", &auto_save);
  auto& intro = ui_intro;
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
    if (editor_widgets::Draw(panicle_size, "Size", 0.001f, "The size of panicle")) {
      changed = true;
    }
    if (editor_widgets::Draw(panicle_seed_amount, "Seed amount", 1.0f, "The amount of seeds in the panicle"))
      changed = true;
    if (editor_widgets::Draw(panicle_seed_radius, "Seed radius", 0.001f, "The size of the seed in the panicle"))
      changed = true;
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Stem settings", ImGuiTreeNodeFlags_DefaultOpen)) {
    TipMenu("The settings for stem.");
    if (editor_widgets::Draw(stem_tilt_angle, "Stem tilt angle", 0.001f, "The tilt angle for stem")) {
      changed = true;
    }
    if (editor_widgets::Draw(internode_length, "Length", 0.01f,
                             "The length of the stem, use Ending Point in leaf settings to make "
                             "stem taller than top leaf for panicle"))
      changed = true;
    if (editor_widgets::Draw(stem_width, "Width", 0.001f,
                             "The overall width of the stem, adjust the width "
                             "along stem in Stem Details"))
      changed = true;
    if (ImGui::TreeNode("Stem Details")) {
      TipMenu("The detailed settings for stem.");
      if (editor_widgets::Draw(width_along_stem, "Width along stem"))
        changed = true;
      ImGui::TreePop();
    }
    ImGui::TreePop();
  }
  if (ImGui::TreeNodeEx("Leaves settings", ImGuiTreeNodeFlags_DefaultOpen)) {
    TipMenu("The settings for leaves.");
    if (editor_widgets::Draw(leaf_amount, "Num of leaves", 1.0f, "The total amount of leaves"))
      changed = true;

    auto& leaf_starting_point = ui_leaf_starting_point;

    if (editor_widgets::Draw(generator.leaf_starting_point, "Starting point along stem", leaf_starting_point)) {
      changed = true;
    }

    auto& leaf_curling = ui_leaf_curling;

    if (editor_widgets::Draw(generator.leaf_curling, "Leaf curling", leaf_curling)) {
      changed = true;
    }

    auto& leaf_roll_angle = ui_leaf_roll_angle;
    if (editor_widgets::Draw(generator.leaf_roll_angle, "Roll angle", leaf_roll_angle))
      changed = true;

    auto& leaf_branching_angle = ui_leaf_branching_angle;
    if (editor_widgets::Draw(generator.leaf_branching_angle, "Branching angle", leaf_branching_angle))
      changed = true;

    auto& leaf_bending = ui_leaf_bending;
    if (editor_widgets::Draw(generator.leaf_bending, "Bending", leaf_bending))
      changed = true;

    auto& leaf_bending_acceleration = ui_leaf_bending_acceleration;

    if (editor_widgets::Draw(generator.leaf_bending_acceleration, "Bending acceleration", leaf_bending_acceleration))
      changed = true;

    auto& leaf_bending_smoothness = ui_leaf_bending_smoothness;

    if (editor_widgets::Draw(generator.leaf_bending_smoothness, "Bending smoothness", leaf_bending_smoothness))
      changed = true;

    if (editor_widgets::Draw(leaf_waviness, "Waviness"))
      changed = true;
    if (editor_widgets::Draw(leaf_waviness_frequency, "Waviness Frequency"))
      changed = true;

    if (editor_widgets::Draw(leaf_length, "Length"))
      changed = true;
    if (editor_widgets::Draw(leaf_width, "Width"))
      changed = true;

    if (ImGui::TreeNode("Per leaf settings")) {
      if (ImGui::TreeNode("Width along leaf")) {
        if (editor_widgets::Draw(width_along_leaf, "Width along leaf"))
          changed = true;
        ImGui::TreePop();
      }
      if (ImGui::TreeNode("Waviness along leaf")) {
        if (editor_widgets::Draw(waviness_along_leaf, "Waviness along leaf"))
          changed = true;
        ImGui::TreePop();
      }
      if (ImGui::TreeNode("Curling along leaf")) {
        if (editor_widgets::Draw(curling_along_leaf, "Curling along leaf"))
          changed = true;
        ImGui::TreePop();
      }
      ImGui::TreePop();
    }
    ImGui::TreePop();
  }

  auto& last_auto_save_time = ui_last_auto_save_time;
  auto& auto_save_interval = ui_auto_save_interval;

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
      if (!generator.Saved()) {
        generator.Save();
        EVOENGINE_LOG(generator.GetTypeName() + " autosaved!");
      }
    }
  } else {
    if (!generator.Saved()) {
      ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(255, 0, 0, 255));
      ImGui::Text("[Changed unsaved!]");
      ImGui::PopStyleColor();
    }
  }

  return changed;
}
