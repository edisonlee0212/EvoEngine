#include "DigitalAgricultureInspectionAdapters.hpp"
#include "DigitalAgricultureInspectorStates.hpp"
#include "DigitalAgricultureSerializationAdapters.hpp"
#include "EcoSysLabLayer.hpp"
#include "EditorLayer.hpp"
#include "Scene.hpp"
#include "Soil.hpp"
#include "Sorghum.hpp"
#include "SorghumCoordinates.hpp"
#include "SorghumGenerator.hpp"
#include "SorghumLayer.hpp"
#include "TransformGraph.hpp"
using namespace digital_agriculture_package;
using namespace eco_sys_lab_package;
bool SorghumFieldInspector::Inspect(InspectorContext& context, SorghumField& field) {
  const auto& editor_layer = context.editor_layer;
  bool changed = false;
  if (ImGui::DragInt("Size limit", &field.size_limit, 1, 0, 10000))
    changed = false;
  if (ImGui::DragFloat("Sorghum size", &field.sorghum_size, 0.01f, 0, 10))
    changed = false;
  if (ImGui::Button("Instantiate")) {
    field.InstantiateField();
  }

  auto& index = ui_index;
  auto& radius = ui_radius;
  ImGui::DragInt("Index", &index);
  ImGui::DragFloat("Radius", &radius);
  auto& temp_coordinates = ui_temp_coordinates;
  if (editor_layer->DragAndDropButton<SorghumCoordinates>(temp_coordinates, "Apply from sorghum coordinates")) {
    if (const auto coordinates = temp_coordinates.Get<SorghumCoordinates>()) {
      glm::dvec2 offset;
      coordinates->Apply(field, offset, index, radius);
      temp_coordinates.Clear();
    }
  }
  ImGui::Text("Matrices count: %d", (int)field.matrices.size());

  return changed;
}
