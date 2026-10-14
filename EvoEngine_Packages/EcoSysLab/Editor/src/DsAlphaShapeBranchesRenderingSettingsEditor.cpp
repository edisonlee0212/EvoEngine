#include "Application.hpp"
#include "Delaunay.hpp"
#include "DsAlphaShapeMeshing.hpp"
#include "DsConstraints.hpp"
#include "DsOperators.hpp"
#include "DynamicTreeStrands.hpp"
#include "EcoSysLabSettingsEditor.hpp"
#include "EditorLayer.hpp"
#include "EditorWidgets.hpp"
#include "RenderParameters.hpp"
#include "Shader.hpp"
#include "Tree.hpp"

using namespace eco_sys_lab_package;

bool eco_sys_lab_package::InspectSettings(BranchesRenderParameters& target,
                                          const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::Checkbox("Tetrahedron complex", &target.render_complex))
    changed = true;
  if (ImGui::Checkbox("Solid", &target.solid))
    changed = true;
  if (ImGui::Checkbox("Wireframe", &target.wireframe))
    changed = true;
  if (ImGui::DragFloat("Extrusion distance", &target.global_extrusion_distance, 0.0001f, 0.0f, 0.1f, "%.4f"))
    changed = true;

  if (ImGui::DragFloat("Degenerate triangle threshold 1e-x", &target.degen_triangle_threshold_logairthmic, 0.01f, 0.0f,
                       40.0f, "%.6f"))
    changed = true;
  if (ImGui::DragFloat("Break threshold", &target.break_threshold, 0.0001f, 0.0f, 1.0f, "%.4f"))
    changed = true;
  if (ImGui::Checkbox("Use cubic Hermite spline", &target.use_cubic_hermite_spline))
    changed = true;

  // Must match BranchesRenderParameters::VertexColors order.
  int color_mode = static_cast<int>(target.vertex_colors);
  if (ImGui::Combo("Color mode",
                   {"Disabled", "Absolute Normals", "Tangents", "Groups", "Degree", "Bark", "Normal Quaternion", "Up",
                    "Initial Up", "Axis", "Initial Axis", "Initial Angle", "Volume change heatmap"},
                   color_mode)) {
    target.vertex_colors = static_cast<BranchesRenderParameters::VertexColors>(color_mode);
    changed = true;
  }

  if (ImGui::Checkbox("Use polar coordinates for UV", &target.use_polar_coordinates_for_uv)) {
    changed = true;

    // reset v_multiplier to default value
    if (target.use_polar_coordinates_for_uv) {
      target.u_multiplier = 1.0f;
      target.v_multiplier = 0.025f;
    } else {
      target.u_multiplier = 1.0f;
      target.v_multiplier = 1.0f;
    }
  }

  if (target.use_polar_coordinates_for_uv) {
    if (ImGui::DragFloat("U-coordinate multiplier", &target.u_multiplier, 1.f, 1.f, 20))
      changed = true;
  } else {
    if (ImGui::DragFloat("U-coordinate multiplier", &target.u_multiplier, 0.001f, 0.0f, 100.0f))
      changed = true;
  }

  if (ImGui::DragFloat("V-coordinate multiplier", &target.v_multiplier, 0.001f, 0.0f, 100.0f))
    changed = true;
  if (ImGui::DragFloat("Interior texture diameter", &target.texture_diameter, 0.01f, 0.1f, 2.0f))
    changed = true;

  if (ImGui::Checkbox("Persistent damage", &target.persistent_damage)) {
    changed = true;
  }

  return changed;
}
