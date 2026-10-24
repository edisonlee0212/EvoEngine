#include "DynamicTreeSkeleton.hpp"
#include "EcoSysLabObjectInspectors.hpp"
#include "Tree.hpp"

using namespace evo_engine;
using namespace eco_sys_lab_package;

bool DynamicTreeSkeletonInspector::Inspect(InspectorContext&, DynamicTreeSkeleton& target) {
  bool changed = false;
  const auto target_scene = target.GetScene();
  const auto tree = target_scene->HasPrivateComponent<Tree>(target.GetOwner())
                        ? target_scene->GetOrSetPrivateComponent<Tree>(target.GetOwner()).lock()
                        : nullptr;
  if (tree) {
    if (ImGui::Button(target.gpu_physics_enabled ? "Reset GPU pose to growth" : "Initialize from growth pose"))
      target.InitializePhysics(*tree);
    if (target.gpu_physics_enabled) {
      ImGui::TextUnformatted("GPU physics runs whenever growth is idle.");
    } else {
      ImGui::SameLine();
      if (ImGui::Button(target.physics_running ? "Pause physics" : "Run physics") && target.HasPhysics(*tree))
        target.physics_running = !target.physics_running;
      ImGui::SameLine();
      if (ImGui::Button("Clear physics"))
        target.ResetPhysics(*tree);
    }
    if (ImGui::Checkbox("GPU physics (experimental)", &target.gpu_physics_enabled)) {
      target.InitializePhysics(*tree);
      changed = true;
    }
    ImGui::TextDisabled("GPU mode currently supports fixed roots and rod motion; fracture and contacts use CPU mode.");
    auto& physics = target.physics_settings;
    if (ImGui::Button("Reset physics settings to defaults")) {
      physics = DynamicTreeSkeleton::PhysicsSettings{};
      target.InitializePhysics(*tree);
      changed = true;
    }
    changed |= ImGui::DragFloat("Simulated time per fixed update", &physics.time_step, 0.0001f, 0.00001f, 0.1f);
    changed |= ImGui::DragInt("Physics substeps", &physics.substeps, 1, 1, 64);
    changed |= ImGui::DragInt("Physics iterations", &physics.iterations, 1, 1, 64);
    changed |= ImGui::DragFloat("Stretch stiffness", &physics.stretch_stiffness, 10.0f, 0.0f, 1000000.0f);
    changed |= ImGui::DragFloat("Bend stiffness", &physics.bend_stiffness, 1.0f, 0.0f, 100000.0f);
    changed |= ImGui::DragFloat("Wood density (reinitialize)", &physics.density, 1.0f, 0.001f, 2000.0f);
    changed |= ImGui::DragFloat("Physics damping", &physics.damping, 0.001f, 0.0f, 10.0f);
    changed |= ImGui::DragFloat("Contact friction", &physics.contact_friction, 0.01f, 0.0f, 2.0f);
    changed |= ImGui::DragFloat3("Gravity", &physics.gravity.x, 0.1f);
    changed |= ImGui::DragFloat3("Anchor offset", &physics.anchor_offset.x, 0.01f);
    changed |= ImGui::Checkbox("Enable branch fracture", &physics.fracture_enabled);
    changed |=
        ImGui::DragFloat("Reference fracture angle (degrees)", &physics.fracture_angle_degrees, 0.5f, 0.0f, 180.0f);
    ImGui::TextDisabled("At 1 m length and 0.05 m radius; adjusted by each junction's length and radius.");
    changed |= ImGui::DragFloat("Fracture exposure (seconds)", &physics.fracture_duration, 0.001f, 0.0f, 10.0f);
  } else {
    ImGui::TextUnformatted("Attach a Tree to this entity to simulate its shoot skeleton.");
  }
  return changed;
}
