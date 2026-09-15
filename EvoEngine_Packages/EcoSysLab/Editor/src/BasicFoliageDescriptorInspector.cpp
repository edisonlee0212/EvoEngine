#include "CurveEditors.hpp"
#include "EcoSysLabDescriptorInspectors.hpp"
#include "EcoSysLabSerializationAdapters.hpp"
#include "EditorLayer.hpp"
#include "ShootModel.hpp"

using namespace eco_sys_lab_package;

bool eco_sys_lab_package::InspectBasicFoliageDescriptor(InspectorContext& context, BasicFoliageDescriptor& target) {
  const auto& editor_layer = context.editor_layer;
  bool changed = false;

  changed = editor_widgets::Draw(target.activation_temperature, "Activation temperature") | changed;
  changed = editor_widgets::Draw(target.activation_light_intensity, "Activation light intensity") | changed;
  changed = editor_widgets::Draw(target.growth_rate, "Growth rate") | changed;
  changed = editor_widgets::Draw(target.damage_temperature, "Damage temperature") | changed;
  changed = editor_widgets::Draw(target.damage_rate, "Damage rate") | changed;
  changed = editor_widgets::Draw(target.hang_time, "Hang time") | changed;

  if (ImGui::DragFloat2("Leaf size", &target.leaf_size.x, 0.001f, 0.0f, 1.0f))
    changed = true;
  if (ImGui::DragInt("Leaf per node", &target.leaf_count, 1, 0, 50))
    changed = true;
  changed = editor_widgets::Draw(target.stem_length, "Stem length") | changed;
  if (ImGui::DragFloat("Rotation variance", &target.rotation_variance, 0.01f, 0.0f, 1.0f))
    changed = true;
  changed = editor_widgets::Draw(target.branching_angle, "Branching angle") | changed;
  if (ImGui::DragFloat("Max node thickness", &target.max_node_thickness, 0.001f, 0.0f, 5.0f))
    changed = true;
  if (ImGui::DragFloat("Min root distance", &target.min_root_distance, 0.01f, 0.0f, 10.0f))
    changed = true;
  if (ImGui::DragFloat("Max end distance", &target.max_end_distance, 0.01f, 0.0f, 10.0f))
    changed = true;

  changed = ImGui::DragFloat("Horizontal Tropism", &target.horizontal_tropism, 0.001f, 0.0f, 1.0f) || changed;
  changed = ImGui::DragFloat("Gravitropism", &target.gravitropism, 0.001f, 0.0f, 1.0f) || changed;
  if (editor_layer->DragAndDropButton<Material>(target.leaf_material_ref, "Leaf Material"))
    changed = true;
  return changed;
}
