#include "BasicBarkDescriptor.hpp"
#include "EcoSysLabDescriptorInspectors.hpp"
#include "EcoSysLabSerializationAdapters.hpp"
#include "EditorLayer.hpp"

using namespace eco_sys_lab_package;

bool eco_sys_lab_package::InspectBasicBarkDescriptor(InspectorContext& context, BasicBarkDescriptor& target) {
  const auto& editor_layer = context.editor_layer;
  bool changed = false;
  if (ImGui::DragFloat("Bark X Frequency", &target.bark_x_frequency, 0.1f, 0.0f, 100.0f))
    changed = true;
  if (ImGui::DragFloat("Bark Y Frequency", &target.bark_y_frequency, 0.1f, 0.0f, 100.0f))
    changed = true;
  if (ImGui::DragFloat("Bark Depth", &target.bark_depth, 0.01f, 0.0f, 1.0f))
    changed = true;

  if (ImGui::DragFloat("Base Frequency", &target.base_frequency, 0.01f, 0.0f, 1.0f))
    changed = true;
  if (ImGui::DragFloat("Base Max Distance", &target.base_max_distance, 0.01f, 0.0f, 1.0f))
    changed = true;
  if (ImGui::DragFloat("Base Distance Decrease Factor", &target.base_distance_decrease_factor, 0.01f, 0.0f, 1.0f))
    changed = true;
  if (ImGui::DragFloat("Base Depth", &target.base_depth, 0.01f, 0.0f, 1.0f))
    changed = true;
  if (editor_layer->DragAndDropButton<Material>(target.bark_material_ref, "Bark Material"))
    changed = true;
  return changed;
}
