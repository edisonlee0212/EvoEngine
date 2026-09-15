#include "BasicFoliageDescriptor.hpp"
#include "CurveEditors.hpp"
#include "DynamicsSettingsEditor.hpp"
#include "EcoSysLabGraphEditors.hpp"
#include "EcoSysLabSettingsEditor.hpp"
#include "EditorLayer.hpp"
#include "EditorWidgets.hpp"
#include "SDKInspectionAdapters.hpp"
using namespace evo_engine;
using namespace eco_sys_lab_package;
bool DynamicStrandsInitializationInspector::Inspect(InspectorContext& context,
                                                    DynamicStrandsInitializeParameters& target) {
  const auto& editor_layer = context.editor_layer;
  bool changed = false;
  if (ImGui::DragFloat("Min segment length", &target.min_segment_length, 0.001f, 0.001f, target.max_segment_length))
    changed = true;
  if (ImGui::DragFloat("Max segment length", &target.max_segment_length, 0.001f, target.min_segment_length, 1.0f))
    changed = true;
  if (ImGui::DragInt("Uniform subdivision", &target.uniform_subdivision, 1, 1, 16)) {
    target.uniform_subdivision = glm::clamp(target.uniform_subdivision, 1, 16);
    changed = true;
  }
  if (ImGui::DragFloat("Neighbor vertical range", &target.neighbor_vertical_range, 0.01f, 0.01f, 10.0f))
    changed = true;
  if (ImGui::DragFloat("Neighbor horizontal range", &target.neighbor_horizontal_range, 0.01f, 0.01f, 10.0f))
    changed = true;
  if (ImGui::TreeNodeEx("Physical properties", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::Checkbox("Show damage graph", &show_damage_graph);
    if (show_damage_graph) {
      changed = evo_engine::DrawProceduralNoiseGraph(target.damage_graph, "Damage graph", editor_layer) || changed;
    }

    if (ImGui::DragFloat3("Damage scale factor", &target.damage_scale_factor.x, 0.001f, 0.f, 1.f)) {
      changed = true;
    }

    if (ImGui::DragFloat("Sapwood offset", &target.sapwood_offset, 0.01f, 0.0f, 1.0f)) {
      changed = true;
    }
    if (ImGui::DragFloat("Sapwood transition", &target.wood_transition, 0.001f, 0.001f, 1.f)) {
      target.wood_transition = glm::clamp(target.wood_transition, 0.001f, 1.f);
      changed = true;
    }
    if (ImGui::TreeNode("Wood material")) {
      ImGui::Checkbox("Show modulus graph", &show_modulus_graph);
      if (show_modulus_graph) {
        changed = DrawStrandGraph(target.modulus_graph, "modulus graph", editor_layer) || changed;
      }
      ImGui::TreePop();
    }

    ImGui::Checkbox("Show strength graph", &show_strength_graph);
    if (show_strength_graph) {
      changed = DrawStrandGraph(target.strength_graph, "Strength", editor_layer) || changed;
    }
    if (ImGui::Checkbox("Trunk", &target.trunk_additional_strength)) {
      changed = true;
    }

    if (target.trunk_additional_strength) {
      ImGui::Checkbox("Show trunk biological properties graph", &show_biological_properties_graph);
      if (show_biological_properties_graph) {
        changed = DrawStrandGraph(target.biological_properties_graph, "biological properties", editor_layer) || changed;
      }
    }

    if (ImGui::TreeNode("Foliage attachments")) {
      if (editor_widgets::Draw(target.leaf_position_alpha, "Leaf position alpha"))
        changed = true;

      if (editor_widgets::Draw(target.leaf_rotation_alpha, "Leaf rotation alpha"))
        changed = true;

      if (editor_widgets::Draw(target.max_leaf_position_strain, "Max leaf position strain"))
        changed = true;

      if (editor_widgets::Draw(target.max_leaf_rotation_strain, "Max leaf rotation strain"))
        changed = true;

      ImGui::TreePop();
    }

    ImGui::TreePop();
  }

  editor_layer->DragAndDropButton<BasicFoliageDescriptor>(target.foliage_descriptor, "Foliage Descriptor");

  if (ImGui::TreeNode("Meshing Properties")) {
#ifdef USE_CGAL
    if (ImGui::Checkbox("Use CGAL", &target.use_cgal))
      changed = true;
#endif  // USE_CGAL
    if (ImGui::Checkbox("Triangulate per bundle", &target.triangulate_per_bundle))
      changed = true;
    if (ImGui::DragFloat("Alpha", &target.alpha, 0.000001f, 0.0f, 1.0f, "%.6f"))
      changed = true;
    if (ImGui::DragFloat("Bifurcation Alpha", &target.bifurcation_alpha, 0.000001f, 0.0f, 1.0f, "%.6f"))
      changed = true;
    if (ImGui::DragFloat("Max Distance Squared", &target.max_dist_squared, 0.000001f, 0.0f, 1.0f, "%.6f"))
      changed = true;
    if (ImGui::Checkbox("Use cubic Hermite spline", &target.use_cubic_hermite_spline))
      changed = true;
    if (ImGui::DragInt("Min bundle size", &target.min_bundle_size, 1, 1, 100))
      changed = true;

    ImGui::TreePop();
  }

  if (ImGui::TreeNode("Bundle solver")) {
    changed = InspectSettings(target.bundle_solver) || changed;
    ImGui::TreePop();
  }

  return changed;
}
