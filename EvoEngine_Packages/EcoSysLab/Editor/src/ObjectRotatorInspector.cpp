#include "EcoSysLabObjectInspectors.hpp"
#include "EcoSysLabSerializationAdapters.hpp"
#include "EditorLayer.hpp"
#include "Scene.hpp"
#include "Times.hpp"
#include "Transform.hpp"

using namespace evo_engine;
using namespace eco_sys_lab_package;

bool ObjectRotatorInspector::Inspect(InspectorContext& context, ObjectRotator& target) {
  const auto& editor_layer = context.editor_layer;
  ImGui::DragFloat("Speed", &target.rotate_speed);
  ImGui::DragFloat3("Rotation", &target.rotation.x);
  return false;
}
