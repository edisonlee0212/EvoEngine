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
    if (ImGui::Button("Initialize from growth pose"))
      target.InitializePhysics(*tree);
    ImGui::SameLine();
    if (ImGui::Button(target.physics_running ? "Pause physics" : "Run physics") && target.HasPhysics(*tree))
      target.physics_running = !target.physics_running;
    ImGui::SameLine();
    if (ImGui::Button("Clear physics"))
      target.ResetPhysics(*tree);
    ImGui::TextUnformatted(target.HasPhysics(*tree) ? "Growth is frozen while physics is initialized."
                                                    : "Initialize after growth finishes.");
    auto& physics = target.physics_settings;
    changed |= ImGui::DragFloat("Simulated time per fixed update", &physics.time_step, 0.0001f, 0.00001f, 0.1f);
    changed |= ImGui::DragInt("Physics substeps", &physics.substeps, 1, 1, 64);
    changed |= ImGui::DragInt("Physics iterations", &physics.iterations, 1, 1, 64);
    changed |= ImGui::DragFloat("Stretch stiffness", &physics.stretch_stiffness, 10.0f, 0.0f, 1000000.0f);
    changed |= ImGui::DragFloat("Bend stiffness", &physics.bend_stiffness, 1.0f, 0.0f, 100000.0f);
    changed |= ImGui::DragFloat("Wood density (reinitialize)", &physics.density, 1.0f, 0.001f, 2000.0f);
    changed |= ImGui::DragFloat("Physics damping", &physics.damping, 0.001f, 0.0f, 10.0f);
    changed |= ImGui::DragFloat3("Gravity", &physics.gravity.x, 0.1f);
    changed |= ImGui::DragFloat3("Anchor offset", &physics.anchor_offset.x, 0.01f);
    changed |= ImGui::DragInt("Force internode", &physics.force_node);
    changed |= ImGui::DragFloat3("Tip force", &physics.tip_force.x, 0.1f);
  } else {
    ImGui::TextUnformatted("Attach a Tree to this entity to simulate its shoot skeleton.");
  }
  return changed;
}
