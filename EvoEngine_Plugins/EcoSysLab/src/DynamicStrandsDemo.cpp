#include "DynamicStrandsDemo.hpp"

#include "DynamicTreeStrands.hpp"

using namespace eco_sys_lab_plugin;

bool DynamicStrandsDemo::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  if (demo_type != DemoType::Empty) {
    ImGui::Text("Demo started");
    ImGui::Text(("Simulated time: " + std::to_string(simulated_time)).c_str());
    ImGui::Text(("Target simulation time: " + std::to_string(target_simulation_time)).c_str());
    if (ImGui::Button("Force stop")) {
      demo_type = DemoType::Empty;
      simulated_time = target_simulation_time;
    }
    return false;
  }
  bool changed = false;
  const auto owner = GetOwner();
  const auto scene = GetScene();
  const auto dts = scene->GetOrSetPrivateComponent<DynamicTreeStrands>(owner).lock();
  dts->enable_physics = false;

  if (ImGui::TreeNode("Physics Parameters")) {
    physics_parameters.OnInspect(editor_layer);
    ImGui::TreePop();
  }

  ImGui::DragFloat("Target simulation time", &target_simulation_time, 0.1f, 0.1f, 100.f);
  ImGui::DragFloat("Target factor 0", &target_factor0, 0.01f, 0.0f, 1.f);
  ImGui::DragFloat("Target factor 1", &target_factor1, 0.01f, 0.0f, 1.f);
  if (ImGui::TreeNode("Dry break")) {
    log_experiment_setup_settings.OnInspect(editor_layer);
    if (ImGui::Button("Start")) {
      simulated_time = 0.f;
      demo_type = DemoType::DryBreak;
      log_experiment_setup_settings.left_pivot_type = static_cast<unsigned>(DynamicTreeStrands::PivotType::Transform);
      log_experiment_setup_settings.right_pivot_type = static_cast<unsigned>(DynamicTreeStrands::PivotType::Transform);
      dts->LogExperimentSetup(log_experiment_setup_settings);
    }
    ImGui::TreePop();
  }

  return changed;
}

void DynamicStrandsDemo::Update() {
  if (demo_type != DemoType::Empty && simulated_time >= target_simulation_time) {
    const auto owner = GetOwner();
    const auto scene = GetScene();
    demo_type = DemoType::Empty;
    const auto children = scene->GetChildren(owner);
    for (const auto& child : children) {
      scene->DeleteEntity(child);
    }
  }
  if (demo_type == DemoType::Empty)
    return;
  const auto owner = GetOwner();
  const auto scene = GetScene();
  const auto dts = scene->GetOrSetPrivateComponent<DynamicTreeStrands>(owner).lock();
  const auto children = scene->GetChildren(owner);

  Entity left_pivot, right_pivot;
  for (const auto& child : children) {
    if (scene->GetEntityName(child) == "Left Pivot") {
      left_pivot = child;
    } else if (scene->GetEntityName(child) == "Right Pivot") {
      right_pivot = child;
    }
  }

  const float progress = simulated_time / target_simulation_time;

  switch (demo_type) {
    case DemoType::DryBreak: {
      const float log_distance = static_cast<float>(log_experiment_setup_settings.rod_segment_count) *
                                 log_experiment_setup_settings.segment_length;
      const float left_distance = log_distance * 0.5f * progress * target_factor0;
      const float right_distance = log_distance * (1.f - 0.5f * progress * target_factor0);

      auto left_operator_root_transform = GlobalTransform();
      left_operator_root_transform.SetPosition(
          dts->initialize_parameters.root_transform.TransformPoint(glm::vec3(left_distance, 0, 0)));
      auto right_operator_root_transform = GlobalTransform();
      right_operator_root_transform.SetPosition(
          dts->initialize_parameters.root_transform.TransformPoint(glm::vec3(right_distance, 0, 0)));
      const float angle = glm::acos(1.f - progress * target_factor1);
      left_operator_root_transform.SetEulerRotation(glm::vec3(0, 0, -angle));
      right_operator_root_transform.SetEulerRotation(glm::vec3(0, 0, angle));
      scene->SetDataComponent(left_pivot, left_operator_root_transform);
      scene->SetDataComponent(right_pivot, right_operator_root_transform);
    } break;
    case DemoType::SquishyBreak: {
    } break;
    default:
      break;
  }

  dts->PhysicsStep(physics_parameters);
  simulated_time += physics_parameters.time_step;
}

void DynamicStrandsDemo::LateUpdate() {
}
