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
    if (ImGui::Button("Reset GPU pose to growth"))
      target.InitializePhysics(*tree);
    ImGui::TextUnformatted("GPU physics runs whenever growth is idle.");
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
    changed |= ImGui::DragFloat("Wind drag", &physics.wind_drag, 0.1f, 0.0f, 100.0f);
    changed |= ImGui::DragFloat3("Gravity", &physics.gravity.x, 0.1f);
    changed |= ImGui::DragFloat3("Anchor offset", &physics.anchor_offset.x, 0.01f);
    ImGui::TextDisabled("Branch fracture and collider contact await GPU support.");
  } else {
    ImGui::TextUnformatted("Attach a Tree to this entity to simulate its shoot skeleton.");
  }
  return changed;
}
