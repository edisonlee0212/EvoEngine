#include "Application.hpp"
#include "CurveEditors.hpp"
#include "DigitalAgricultureInspectionAdapters.hpp"
#include "DigitalAgricultureInspectorStates.hpp"
#include "DigitalAgricultureSerializationAdapters.hpp"
#include "EditorFileDialogs.hpp"
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
bool digital_agriculture_package::DrawSorghumPanicleStateGui(SorghumPanicleState& state) {
  bool changed = false;
  if (ImGui::DragFloat("Panicle width", &state.panicle_size.x, 0.001f)) {
    changed = true;
    state.panicle_size.z = state.panicle_size.x;
  }
  if (ImGui::DragFloat("Panicle height", &state.panicle_size.y, 0.001f))
    changed = true;
  if (ImGui::DragInt("Num of seeds", &state.seed_amount, 1.0f))
    changed = true;
  if (ImGui::DragFloat("Seed radius", &state.seed_radius, 0.0001f))
    changed = true;
  if (changed)
    state.saved = false;
  return changed;
}
bool digital_agriculture_package::DrawSorghumStemStateGui(SorghumStemState& state, int mode) {
  bool changed = false;
  switch (static_cast<StateMode>(mode)) {
    case StateMode::Default:
      // ImGui::DragFloat3("Direction", &direction.x, 0.01f);
      if (ImGui::DragFloat("Length", &state.length, 0.01f))
        changed = true;
      break;
    case StateMode::CubicBezier:
      if (ImGui::TreeNode("Spline")) {
        editor_widgets::Draw(state.spline);
        ImGui::TreePop();
      }
      break;
  }
  if (editor_widgets::Draw(state.width_along_stem, "Width along stem"))
    changed = true;

  if (changed)
    state.saved = false;
  return changed;
}
bool digital_agriculture_package::DrawSorghumLeafStateGui(SorghumLeafState& state, int mode) {
  bool changed = false;
  if (ImGui::Checkbox("Dead", &state.dead)) {
    changed = true;
    if (!state.dead && state.length == 0.0f)
      state.length = 0.35f;
  }
  if (!state.dead) {
    if (ImGui::InputFloat("Starting point", &state.starting_point)) {
      state.starting_point = glm::clamp(state.starting_point, 0.0f, 1.0f);
      changed = true;
    }
    switch (static_cast<StateMode>(mode)) {
      case StateMode::Default:
        if (ImGui::TreeNodeEx("Geometric", ImGuiTreeNodeFlags_DefaultOpen)) {
          if (ImGui::DragFloat("Length", &state.length, 0.01f, 0.0f, 999.0f))
            changed = true;
          if (ImGui::TreeNodeEx("Angles", ImGuiTreeNodeFlags_DefaultOpen)) {
            if (ImGui::DragFloat("Roll angle", &state.roll_angle, 1.0f, -999.0f, 999.0f))
              changed = true;
            if (ImGui::InputFloat("Branching angle", &state.branching_angle)) {
              state.branching_angle = glm::clamp(state.branching_angle, 0.0f, 180.0f);
              changed = true;
            }
            ImGui::TreePop();
          }
          ImGui::TreePop();
        }
        break;
      case StateMode::CubicBezier:
        if (ImGui::TreeNodeEx("Geometric", ImGuiTreeNodeFlags_DefaultOpen)) {
          editor_widgets::Draw(state.spline);
          ImGui::TreePop();
        }
        break;
    }

    if (ImGui::TreeNodeEx("Others")) {
      if (editor_widgets::Draw(state.width_along_leaf, "Width"))
        changed = true;
      if (editor_widgets::Draw(state.curling_along_leaf, "Rolling"))
        changed = true;

      const CurveDescriptorSettings leaf_bending = {1.0f, false, true,
                                                    "The bending of the leaf, controls how leaves bend because of "
                                                    "gravity. Positive value results in leaf bending towards the "
                                                    "ground, negative value results in leaf bend towards the sky"};

      if (editor_widgets::Draw(state.bending_along_leaf, "Bending along leaf", leaf_bending)) {
        changed = true;
        state.bending_along_leaf.curve.UnsafeGetValues()[1].y = 0.5f;
      }
      if (editor_widgets::Draw(state.waviness_along_leaf, "Waviness along leaf"))
        changed = true;

      if (ImGui::DragFloat("Waviness frequency", &state.waviness_frequency, 0.01f, 0.0f, 999.0f))
        changed = true;
      if (ImGui::DragFloat2("Waviness start period", &state.waviness_period_start.x, 0.01f, 0.0f, 999.0f))
        changed = true;
      ImGui::TreePop();
    }
  }
  if (changed)
    state.saved = false;
  return changed;
}
bool digital_agriculture_package::DrawSorghumStateGui(SorghumState& state, int mode) {
  bool changed = false;
  if (ImGui::TreeNodeEx((std::string("Stem")).c_str())) {
    if (DrawSorghumStemStateGui(state.stem, mode))
      changed = true;
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Leaves")) {
    int leaf_size = state.leaves.size();
    if (ImGui::InputInt("Number of leaves", &leaf_size)) {
      changed = true;
      leaf_size = glm::clamp(leaf_size, 0, 999);
      const auto previous_size = state.leaves.size();
      state.leaves.resize(leaf_size);
      for (int i = 0; i < leaf_size; i++) {
        if (i >= previous_size) {
          if (i - 1 >= 0) {
            state.leaves[i] = state.leaves[i - 1];
            state.leaves[i].roll_angle = glm::mod(state.leaves[i - 1].roll_angle + 180.0f, 360.0f);
            state.leaves[i].starting_point = state.leaves[i - 1].starting_point + 0.1f;
          } else {
            state.leaves[i] = SorghumLeafState();
            state.leaves[i].roll_angle = 0;
            state.leaves[i].starting_point = 0.1f;
          }
        }
        state.leaves[i].index = i;
      }
    }
    for (auto& leaf : state.leaves) {
      if (ImGui::TreeNode(
              ("Leaf No." + std::to_string(leaf.index + 1) + (leaf.length == 0.0f || leaf.dead ? " (Dead)" : ""))
                  .c_str())) {
        if (DrawSorghumLeafStateGui(leaf, mode))
          changed = true;
        ImGui::TreePop();
      }
    }
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx((std::string("Panicle")).c_str())) {
    if (DrawSorghumPanicleStateGui(state.panicle))
      changed = true;
    ImGui::TreePop();
  }
  if (mode == static_cast<int>(StateMode::CubicBezier)) {
    EditorFileDialogs::OpenFile(
        "Import...", "TXT", {".txt"},
        [&state, &changed](const std::filesystem::path& path) {
          std::ifstream file(path, std::fstream::in);
          if (!file.is_open()) {
            EVOENGINE_LOG("Failed to open file!");
            return;
          }
          changed = true;
          // Number of leaves in the file
          int leaf_count;
          file >> leaf_count;
          state.stem = SorghumStemState();
          state.stem.spline.Import(file);
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
          state.leaves.resize(leaf_count);
          for (int i = 0; i < leaf_count; i++) {
            float starting_point;
            file >> starting_point;
            state.leaves[i] = SorghumLeafState();
            state.leaves[i].starting_point = starting_point;
            state.leaves[i].spline.Import(file);
            state.leaves[i].spline.curves[0].p0 = state.stem.spline.EvaluatePointFromCurves(starting_point);
          }

          for (int i = 0; i < leaf_count; i++) {
            state.leaves[i].index = i;
          }
        },
        false);
  }
  if (changed)
    state.saved = false;
  return changed;
}
bool SorghumStateInspector::Inspect(InspectorContext& context, SorghumState& state) {
  (void)context;
  bool changed = false;
  if (ImGui::Button("Instantiate")) {
    const auto new_entity = state.CreateEntity("New Sorghum");
  }

  auto& target_waviness_factor = ui_target_waviness_factor;
  ImGui::DragFloat("Target leaf waviness", &target_waviness_factor, 0.01f, 0.01f, 3.0f);
  if (ImGui::Button("Create sorghum with changed waviness")) {
    const auto scene = ApplicationContext::Get().GetActiveScene();
    const auto sorghum_entity = scene->CreateEntity(state.GetTitle());
    const auto sorghum = scene->GetOrSetPrivateComponent<Sorghum>(sorghum_entity).lock();
    const auto new_sorghum_state = AssetManager::CreateTemporaryAsset<SorghumState>();
    SorghumMeshGeneratorSettings settings{};
    settings.enable_leaf_sheath = false;
    settings.bottom_face = false;
    state.ChangeWaviness(target_waviness_factor, settings, *new_sorghum_state);

    sorghum->sorghum_state = new_sorghum_state;
    if (const auto sorghum_layer = ApplicationContext::Get().GetLayer<SorghumLayer>()) {
      sorghum->GenerateGeometryEntities(sorghum_layer->sorghum_mesh_generator_settings);
    } else {
      sorghum->GenerateGeometryEntities({});
    }
  }

  auto& state_mode = ui_state_mode;
  auto& state_modes = ui_state_modes;
  if (ImGui::Combo("Mode", &state_mode, state_modes, IM_ARRAYSIZE(state_modes))) {
    changed = false;
  }
  DrawSorghumStateGui(state, state_mode);
  return changed;
}
