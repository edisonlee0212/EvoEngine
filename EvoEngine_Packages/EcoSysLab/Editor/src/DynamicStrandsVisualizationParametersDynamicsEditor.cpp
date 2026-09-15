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
bool eco_sys_lab_package::InspectSettings(DynamicStrandsVisualizationParameters& target,
                                          const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::Checkbox("Segments", &target.render_segments))
    changed = true;
  if (target.render_segments) {
    if (ImGui::Combo(
            "Segment mode",
            {"Default", "Node color", "Group index", "Boundary distance", "Strength", "Shear/Strain strain",
             "Shear/Stretch limit", "Segment color", "Strand color", "Fungus density", "Health", "Screen depth"},
            target.segment_render_mode))
      changed = true;

    switch (target.segment_render_mode) {
      case 0: {
        if (ImGui::ColorEdit4("Segment color", &target.segment_color_main.x))
          changed = true;
        break;
      }
      case 3: {
        if (ImGui::ColorEdit4("Segment min color", &target.segment_color_min.x))
          changed = true;
        if (ImGui::ColorEdit4("Segment max color", &target.segment_color_max.x))
          changed = true;
        if (ImGui::DragFloat("Segment boundary distance modular", &target.segment_boundary_distance_modular, 0.001f,
                             0.001f, 1.f))
          changed = true;
        break;
      }
      case 4:
      case 5:
      case 6: {
        if (ImGui::ColorEdit4("Segment min color", &target.segment_color_min.x))
          changed = true;
        if (ImGui::ColorEdit4("Segment max color", &target.segment_color_max.x))
          changed = true;
        break;
      }
      default:
        break;
    }
    if (ImGui::DragFloat("Segment radius multiplier", &target.segment_radius_multiplier, 0.1f, 0.1f, 1000.f))
      changed = true;
    if (ImGui::DragFloat("Segment length multiplier", &target.segment_length_multiplier, 0.05f, 0.2f, 1.0f))
      changed = true;
    if (ImGui::DragFloat("General factor", &target.general_factor, 0.5f, 1.0f, 20.0f))
      changed = true;
  }

  if (ImGui::Checkbox("Segment Pair", &target.render_segment_pairs))
    changed = true;
  if (target.render_segment_pairs) {
    if (ImGui::Combo(
            "Segment Pair mode",
            {"Default", "Bending strain", "Twisting strain", "Bundle strain", "Combined strain", "Connectivity strain",
             "Bending Limit", "Twisting limit", "Bundle limit", "Connectivity limit", "Segment color"},
            target.segment_pair_render_mode))
      changed = true;
    switch (target.segment_pair_render_mode) {
      case 0: {
        if (ImGui::ColorEdit4("Segment pair color", &target.segment_pair_color_main.x))
          changed = true;
        break;
      }
      case 1:
      case 2:
      case 3:
      case 4:
      case 5:
      case 6:
      case 7:
      case 8:
      case 9:
      case 10: {
        if (ImGui::ColorEdit4("Segment pair min color", &target.segment_pair_color_min.x))
          changed = true;
        if (ImGui::ColorEdit4("Segment pair max color", &target.segment_pair_color_max.x))
          changed = true;
        break;
      }
    }
    if (ImGui::DragFloat("Segment pair radius multiplier", &target.segment_pair_radius_multiplier, 0.1f, 0.1f, 10.f))
      changed = true;
  }

  if (ImGui::Checkbox("Foliage", &target.render_foliage))
    changed = true;
  if (target.render_foliage) {
    if (ImGui::Combo("Foliage mode", {"Default"}, target.foliage_render_mode))
      changed = true;
    switch (target.foliage_render_mode) {
      case 0: {
        if (ImGui::ColorEdit4("Foliage color", &target.foliage_color_main.x))
          changed = true;
        break;
      }
    }
  }
  return changed;
}
