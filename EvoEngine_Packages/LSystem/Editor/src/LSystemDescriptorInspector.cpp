#include "LSystemDescriptor.hpp"
#include "LSystemInspectionAdapters.hpp"

using namespace evo_engine;
using namespace l_system_package;
bool l_system_package::InspectLSystemDescriptor(InspectorContext& context, LSystemDescriptor& descriptor) {
  (void)context;
  bool changed = false;

  if (ImGui::DragInt("Derivation Steps", &descriptor.derivation_steps, 1, 0, 100))
    changed = true;

  int seed_int = static_cast<int>(descriptor.seed);
  if (ImGui::DragInt("Seed", &seed_int, 1, 0, 999999)) {
    descriptor.seed = static_cast<unsigned int>(seed_int);
    changed = true;
  }

  if (ImGui::DragFloat3("Root Position", &descriptor.root_position.x, 0.1f))
    changed = true;

  glm::vec3 euler = glm::degrees(glm::eulerAngles(descriptor.root_rotation));
  if (ImGui::DragFloat3("Root Rotation (deg)", &euler.x, 1.0f)) {
    descriptor.root_rotation = glm::quat(glm::radians(euler));
    changed = true;
  }

  if (ImGui::DragFloat("Default Length", &descriptor.default_length, 0.01f, 0.001f, 100.0f))
    changed = true;

  if (ImGui::DragFloat("Default Thickness", &descriptor.default_thickness, 0.001f, 0.001f, 10.0f))
    changed = true;

  if (ImGui::Checkbox("Auto Derive on Change", &descriptor.auto_derive_on_change))
    changed = true;

  return changed;
}
