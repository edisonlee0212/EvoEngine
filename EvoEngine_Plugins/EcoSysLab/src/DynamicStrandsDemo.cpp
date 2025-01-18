#include "DynamicStrandsDemo.hpp"

#include "DynamicTreeStrands.hpp"

using namespace eco_sys_lab_plugin;

bool DynamicStrandsDemo::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  if (ImGui::TreeNode("Physics Parameters")) {
    physics_parameters.OnInspect(editor_layer);
    ImGui::TreePop();
  }

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
  if (ImGui::TreeNode("Initialize Parameters")) {
    dts->initialize_parameters.OnInspect(editor_layer);
    ImGui::TreePop();
  }
  ImGui::DragFloat("Target simulation time", &target_simulation_time, 0.1f, 0.1f, 100.f);
  ImGui::DragFloat("Target factor 0", &target_factor0, 0.01f, 0.0f, 1.f);
  ImGui::DragFloat("Target factor 1", &target_factor1, 0.01f, 0.0f, 1.f);
  if (ImGui::TreeNode("Rod settings")) {
    log_experiment_setup_settings.OnInspect(editor_layer);
    ImGui::TreePop();
  }
  if (ImGui::TreeNode("Board settings")) {
    board_experiment_setup_settings.OnInspect(editor_layer);
    ImGui::TreePop();
  }
  if (ImGui::Button("Dry break (Rod)")) {
    simulated_time = 0.f;
    demo_type = DemoType::DryBreakRod;
    log_experiment_setup_settings.left_pivot_type = static_cast<unsigned>(DynamicTreeStrands::PivotType::Transform);
    log_experiment_setup_settings.right_pivot_type = static_cast<unsigned>(DynamicTreeStrands::PivotType::Transform);
    dts->LogExperimentSetup(log_experiment_setup_settings);
    dts->enable_physics = false;
  }
  if (ImGui::Button("Board break [L]")) {
    simulated_time = 0.f;
    demo_type = DemoType::BreakBoardLow;
    board_experiment_setup_settings.center_damage = 0.f;
    board_experiment_setup_settings.left_pivot_type = static_cast<unsigned>(DynamicTreeStrands::PivotType::Transform);
    board_experiment_setup_settings.right_pivot_type = static_cast<unsigned>(DynamicTreeStrands::PivotType::Transform);
    dts->BoardExperimentSetup(board_experiment_setup_settings);
    dts->initialize_parameters.shear_stretch_strength = {0.2f, 0.1f};
    dts->initialize_parameters.bending_strength = {0.2f, 0.1f};
    dts->initialize_parameters.twisting_strength = {0.2f, 0.1f};
    dts->initialize_parameters.bundle_strength = {0.02f, 0.02f};
    dts->initialize_parameters.connectivity_strength = {0.1f, 0.1f};
    dts->enable_physics = false;
  }
  if (ImGui::Button("Board break [M]")) {
    simulated_time = 0.f;
    demo_type = DemoType::BreakBoardMed;
    board_experiment_setup_settings.center_damage = 0.f;
    board_experiment_setup_settings.left_pivot_type = static_cast<unsigned>(DynamicTreeStrands::PivotType::Transform);
    board_experiment_setup_settings.right_pivot_type = static_cast<unsigned>(DynamicTreeStrands::PivotType::Transform);
    dts->BoardExperimentSetup(board_experiment_setup_settings);
    dts->initialize_parameters.shear_stretch_strength = {0.2f, 0.1f};
    dts->initialize_parameters.bending_strength = {0.2f, 0.1f};
    dts->initialize_parameters.twisting_strength = {0.2f, 0.1f};
    dts->initialize_parameters.bundle_strength = {0.04f, 0.04f};
    dts->initialize_parameters.connectivity_strength = {0.1f, 0.1f};
    dts->enable_physics = false;
  }
  if (ImGui::Button("Board break [H]")) {
    simulated_time = 0.f;
    demo_type = DemoType::BreakBoardHigh;
    board_experiment_setup_settings.center_damage = 0.f;
    board_experiment_setup_settings.left_pivot_type = static_cast<unsigned>(DynamicTreeStrands::PivotType::Transform);
    board_experiment_setup_settings.right_pivot_type = static_cast<unsigned>(DynamicTreeStrands::PivotType::Transform);
    dts->initialize_parameters.shear_stretch_strength = {0.2f, 0.1f};
    dts->initialize_parameters.bending_strength = {0.2f, 0.1f};
    dts->initialize_parameters.twisting_strength = {0.2f, 0.1f};
    dts->initialize_parameters.bundle_strength = {0.07f, 0.07f};
    dts->initialize_parameters.connectivity_strength = {0.1f, 0.1f};
    dts->BoardExperimentSetup(board_experiment_setup_settings);
    dts->enable_physics = false;
  }

  if (ImGui::Button("Twisting break")) {
    simulated_time = 0.f;
    demo_type = DemoType::TwistingBreak;
    target_factor0 = 0.f;
    dts->initialize_parameters.shear_stretch_strength = glm::vec2(0.1f);
    dts->initialize_parameters.bending_strength = glm::vec2(0.1f);
    dts->initialize_parameters.twisting_strength = glm::vec2(0.1f);
    dts->initialize_parameters.bundle_strength = glm::vec2(0.1f);
    dts->initialize_parameters.connectivity_strength = glm::vec2(0.1f);

    dts->initialize_parameters.damage.noise_descriptors.clear();
    auto& noise = dts->initialize_parameters.damage.noise_descriptors.emplace_back();
    noise.type = static_cast<unsigned>(NoiseType::Perlin);

    board_experiment_setup_settings.left_pivot_type = static_cast<unsigned>(DynamicTreeStrands::PivotType::Transform);
    board_experiment_setup_settings.right_pivot_type = static_cast<unsigned>(DynamicTreeStrands::PivotType::Transform);
    board_experiment_setup_settings.rod_dimension = {160, 10, 20};
    dts->BoardExperimentSetup(board_experiment_setup_settings);
    dts->enable_physics = false;
  }
  if (ImGui::Button("Bending break")) {
    simulated_time = 0.f;
    target_factor0 = 0.f;
    demo_type = DemoType::BendingBreak;

    dts->initialize_parameters.shear_stretch_strength = glm::vec2(0.1f);
    dts->initialize_parameters.bending_strength = glm::vec2(0.1f);
    dts->initialize_parameters.twisting_strength = glm::vec2(0.1f);
    dts->initialize_parameters.bundle_strength = glm::vec2(0.1f);
    dts->initialize_parameters.connectivity_strength = glm::vec2(0.1f);

    dts->initialize_parameters.damage.noise_descriptors.clear();
    auto& noise = dts->initialize_parameters.damage.noise_descriptors.emplace_back();
    noise.type = static_cast<unsigned>(NoiseType::Perlin);

    board_experiment_setup_settings.left_pivot_type = static_cast<unsigned>(DynamicTreeStrands::PivotType::Transform);
    board_experiment_setup_settings.right_pivot_type = static_cast<unsigned>(DynamicTreeStrands::PivotType::Transform);
    board_experiment_setup_settings.rod_dimension = {160, 10, 20};
    dts->BoardExperimentSetup(board_experiment_setup_settings);
    dts->enable_physics = false;
  }
  if (ImGui::Button("Shearing break")) {
    simulated_time = 0.f;
    target_factor0 = 0.f;
    demo_type = DemoType::ShearingBreak;

    dts->initialize_parameters.shear_stretch_strength = glm::vec2(0.1f);
    dts->initialize_parameters.bending_strength = glm::vec2(0.1f);
    dts->initialize_parameters.twisting_strength = glm::vec2(0.1f);
    dts->initialize_parameters.bundle_strength = glm::vec2(0.1f);
    dts->initialize_parameters.connectivity_strength = glm::vec2(0.1f);

    dts->initialize_parameters.damage.noise_descriptors.clear();
    auto& noise = dts->initialize_parameters.damage.noise_descriptors.emplace_back();
    noise.type = static_cast<unsigned>(NoiseType::Perlin);

    board_experiment_setup_settings.left_pivot_type = static_cast<unsigned>(DynamicTreeStrands::PivotType::Transform);
    board_experiment_setup_settings.right_pivot_type = static_cast<unsigned>(DynamicTreeStrands::PivotType::Transform);
    board_experiment_setup_settings.rod_dimension = {160, 10, 20};
    dts->BoardExperimentSetup(board_experiment_setup_settings);
    dts->enable_physics = false;
  }
  if (ImGui::Button("Stretching break")) {
    simulated_time = 0.f;
    target_factor0 = 0.f;
    demo_type = DemoType::StretchingBreak;

    dts->initialize_parameters.shear_stretch_strength = glm::vec2(0.1f);
    dts->initialize_parameters.bending_strength = glm::vec2(0.1f);
    dts->initialize_parameters.twisting_strength = glm::vec2(0.1f);
    dts->initialize_parameters.bundle_strength = glm::vec2(0.1f);
    dts->initialize_parameters.connectivity_strength = glm::vec2(0.1f);

    dts->initialize_parameters.damage.noise_descriptors.clear();
    auto& noise = dts->initialize_parameters.damage.noise_descriptors.emplace_back();
    noise.type = static_cast<unsigned>(NoiseType::Perlin);

    board_experiment_setup_settings.left_pivot_type = static_cast<unsigned>(DynamicTreeStrands::PivotType::Transform);
    board_experiment_setup_settings.right_pivot_type = static_cast<unsigned>(DynamicTreeStrands::PivotType::Transform);
    board_experiment_setup_settings.rod_dimension = {160, 10, 20};
    dts->BoardExperimentSetup(board_experiment_setup_settings);
    dts->enable_physics = false;
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
  const auto owner_gt = scene->GetDataComponent<GlobalTransform>(owner);
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
    case DemoType::DryBreakRod: {
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
      left_operator_root_transform.SetRotation(owner_gt.GetRotation() * glm::quat(glm::vec3(0, 0, -angle)));
      right_operator_root_transform.SetRotation(owner_gt.GetRotation() * glm::quat(glm::vec3(0, 0, angle)));
      scene->SetDataComponent(left_pivot, left_operator_root_transform);
      scene->SetDataComponent(right_pivot, right_operator_root_transform);
    } break;
    case DemoType::BreakBoardLow:
    case DemoType::BreakBoardMed:
    case DemoType::BreakBoardHigh: {
      const float board_distance = static_cast<float>(board_experiment_setup_settings.rod_dimension.z) *
                                   board_experiment_setup_settings.segment_length;
      const float left_distance = board_distance * 0.5f * progress * target_factor0;
      const float right_distance = board_distance * (1.f - 0.5f * progress * target_factor0);

      auto left_operator_root_transform = GlobalTransform();
      left_operator_root_transform.SetPosition(
          dts->initialize_parameters.root_transform.TransformPoint(glm::vec3(left_distance, 0, 0)));
      auto right_operator_root_transform = GlobalTransform();
      right_operator_root_transform.SetPosition(
          dts->initialize_parameters.root_transform.TransformPoint(glm::vec3(right_distance, 0, 0)));
      const float angle = glm::acos(1.f - progress * target_factor1);
      left_operator_root_transform.SetRotation(owner_gt.GetRotation() * glm::quat(glm::vec3(0, 0, -angle)));
      right_operator_root_transform.SetRotation(owner_gt.GetRotation() * glm::quat(glm::vec3(0, 0, angle)));
      scene->SetDataComponent(left_pivot, left_operator_root_transform);
      scene->SetDataComponent(right_pivot, right_operator_root_transform);
    } break;
    case DemoType::TwistingBreak: {
      const float board_distance = static_cast<float>(board_experiment_setup_settings.rod_dimension.z) *
                                   board_experiment_setup_settings.segment_length;
      const float left_distance = board_distance * 0.5f * progress * target_factor0;
      const float right_distance = board_distance * (1.f - 0.5f * progress * target_factor0);

      auto left_operator_root_transform = GlobalTransform();
      left_operator_root_transform.SetPosition(
          dts->initialize_parameters.root_transform.TransformPoint(glm::vec3(left_distance, 0, 0)));
      auto right_operator_root_transform = GlobalTransform();
      right_operator_root_transform.SetPosition(
          dts->initialize_parameters.root_transform.TransformPoint(glm::vec3(right_distance, 0, 0)));
      const float angle = glm::acos(1.f - progress * target_factor1);
      left_operator_root_transform.SetRotation(owner_gt.GetRotation() * glm::quat(glm::vec3(-angle, 0, 0)));
      right_operator_root_transform.SetRotation(owner_gt.GetRotation() * glm::quat(glm::vec3(angle, 0, 0)));
      scene->SetDataComponent(left_pivot, left_operator_root_transform);
      scene->SetDataComponent(right_pivot, right_operator_root_transform);
    } break;
    case DemoType::BendingBreak: {
      const float board_distance = static_cast<float>(board_experiment_setup_settings.rod_dimension.z) *
                                   board_experiment_setup_settings.segment_length;
      const float left_distance = board_distance * 0.5f * progress * target_factor0;
      const float right_distance = board_distance * (1.f - 0.5f * progress * target_factor0);

      auto left_operator_root_transform = GlobalTransform();
      left_operator_root_transform.SetPosition(
          dts->initialize_parameters.root_transform.TransformPoint(glm::vec3(left_distance, 0, 0)));
      auto right_operator_root_transform = GlobalTransform();
      right_operator_root_transform.SetPosition(
          dts->initialize_parameters.root_transform.TransformPoint(glm::vec3(right_distance, 0, 0)));
      const float angle = glm::acos(1.f - progress * target_factor1);
      left_operator_root_transform.SetRotation(owner_gt.GetRotation() * glm::quat(glm::vec3(0, 0, -angle)));
      right_operator_root_transform.SetRotation(owner_gt.GetRotation() * glm::quat(glm::vec3(0, 0, angle)));
      scene->SetDataComponent(left_pivot, left_operator_root_transform);
      scene->SetDataComponent(right_pivot, right_operator_root_transform);
    } break;
    case DemoType::ShearingBreak: {
      const float board_distance = static_cast<float>(board_experiment_setup_settings.rod_dimension.z) *
                                   board_experiment_setup_settings.segment_length;
      const float left_distance = board_distance * 0.5f * progress * target_factor0;
      const float right_distance = board_distance * (1.f - 0.5f * progress * target_factor0);

      auto left_operator_root_transform = GlobalTransform();
      left_operator_root_transform.SetPosition(dts->initialize_parameters.root_transform.TransformPoint(
          glm::vec3(left_distance, -board_distance * 0.5f * progress * target_factor1, 0)));
      auto right_operator_root_transform = GlobalTransform();
      right_operator_root_transform.SetPosition(dts->initialize_parameters.root_transform.TransformPoint(
          glm::vec3(right_distance, board_distance * 0.5f * progress * target_factor1, 0)));
      scene->SetDataComponent(left_pivot, left_operator_root_transform);
      scene->SetDataComponent(right_pivot, right_operator_root_transform);
    } break;
    case DemoType::StretchingBreak: {
      const float board_distance = static_cast<float>(board_experiment_setup_settings.rod_dimension.z) *
                                   board_experiment_setup_settings.segment_length;
      const float left_distance = board_distance * 0.5f * progress * target_factor0;
      const float right_distance = board_distance * (1.f - 0.5f * progress * target_factor0);

      auto left_operator_root_transform = GlobalTransform();
      left_operator_root_transform.SetPosition(dts->initialize_parameters.root_transform.TransformPoint(
          glm::vec3(left_distance - board_distance * 0.5f * progress * target_factor1, 0.f, 0.f)));
      auto right_operator_root_transform = GlobalTransform();
      right_operator_root_transform.SetPosition(dts->initialize_parameters.root_transform.TransformPoint(
          glm::vec3(right_distance + board_distance * 0.5f * progress * target_factor1, 0.f, 0.f)));
      scene->SetDataComponent(left_pivot, left_operator_root_transform);
      scene->SetDataComponent(right_pivot, right_operator_root_transform);
    } break;
    default:
      break;
  }

  dts->PhysicsStep(physics_parameters);
  simulated_time += physics_parameters.time_step;
}

void DynamicStrandsDemo::LateUpdate() {
}
