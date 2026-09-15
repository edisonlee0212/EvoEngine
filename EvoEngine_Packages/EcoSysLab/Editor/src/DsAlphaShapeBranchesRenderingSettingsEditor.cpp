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

  if (ImGui::TreeNodeEx("Use normal attribute for debugging")) {
    if (ImGui::RadioButton("Disabled", (int*)&target.vertex_colors, BranchesRenderParameters::Default))
      changed = true;
    if (ImGui::RadioButton("Absolute Normals", (int*)&target.vertex_colors, BranchesRenderParameters::Normals))
      changed = true;
    if (ImGui::RadioButton("Tangents", (int*)&target.vertex_colors, BranchesRenderParameters::Tangents))
      changed = true;
    if (ImGui::RadioButton("Groups", (int*)&target.vertex_colors, BranchesRenderParameters::Groups))
      changed = true;
    if (ImGui::RadioButton("Degree", (int*)&target.vertex_colors, BranchesRenderParameters::Degree))
      changed = true;
    if (ImGui::RadioButton("Bark", (int*)&target.vertex_colors, BranchesRenderParameters::Bark))
      changed = true;
    if (ImGui::RadioButton("Normal Quaternion", (int*)&target.vertex_colors,
                           BranchesRenderParameters::NormalQuaternion))
      changed = true;
    if (ImGui::RadioButton("Up", (int*)&target.vertex_colors, BranchesRenderParameters::Up))
      changed = true;
    if (ImGui::RadioButton("Initial Up", (int*)&target.vertex_colors, BranchesRenderParameters::InitUp))
      changed = true;
    if (ImGui::RadioButton("Axis", (int*)&target.vertex_colors, BranchesRenderParameters::Axis))
      changed = true;
    if (ImGui::RadioButton("Initial Axis", (int*)&target.vertex_colors, BranchesRenderParameters::InitAxis))
      changed = true;
    if (ImGui::RadioButton("Initial Angle", (int*)&target.vertex_colors, BranchesRenderParameters::InitAngle))
      changed = true;

    ImGui::TreePop();
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

  if (ImGui::Checkbox("Persistent damage", &target.persistent_damage)) {
    changed = true;
  }

  return changed;
}
