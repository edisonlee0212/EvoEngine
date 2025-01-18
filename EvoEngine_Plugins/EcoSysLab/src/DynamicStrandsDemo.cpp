#include "DynamicStrandsDemo.hpp"

#include "DsColliders.hpp"
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

  if (ImGui::Button("Board break [L]")) {
    simulated_time = 0.f;
    physics_parameters = {};
    demo_type = DemoType::BreakBoardLow;
    board_experiment_setup_settings.center_damage = 0.f;
    board_experiment_setup_settings.left_pivot_type = static_cast<unsigned>(DynamicTreeStrands::PivotType::Transform);
    board_experiment_setup_settings.right_pivot_type = static_cast<unsigned>(DynamicTreeStrands::PivotType::Transform);
    dts->BoardExperimentSetup(board_experiment_setup_settings);
    dts->initialize_parameters.shear_stretch_strength = {500.f, 250.f};
    dts->initialize_parameters.bending_strength = {500.f, 250.f};
    dts->initialize_parameters.twisting_strength = {500.f, 250.f};
    dts->initialize_parameters.bundle_strength = {50.f, 50.f};
    dts->initialize_parameters.connectivity_strength = {250.f, 250.f};
    dts->initialize_parameters.max_segment_length = 0.06f;
    dts->initialize_parameters.min_segment_length = 0.03f;
    dts->enable_physics = false;
  }
  if (ImGui::Button("Board break [M]")) {
    simulated_time = 0.f;
    physics_parameters = {};
    demo_type = DemoType::BreakBoardMed;
    board_experiment_setup_settings.center_damage = 0.f;
    board_experiment_setup_settings.left_pivot_type = static_cast<unsigned>(DynamicTreeStrands::PivotType::Transform);
    board_experiment_setup_settings.right_pivot_type = static_cast<unsigned>(DynamicTreeStrands::PivotType::Transform);
    dts->BoardExperimentSetup(board_experiment_setup_settings);
    dts->initialize_parameters.shear_stretch_strength = {500.f, 250.f};
    dts->initialize_parameters.bending_strength = {500.f, 250.f};
    dts->initialize_parameters.twisting_strength = {500.f, 250.f};
    dts->initialize_parameters.bundle_strength = {100.f, 100.f};
    dts->initialize_parameters.connectivity_strength = {250.f, 250.f};
    dts->initialize_parameters.max_segment_length = 0.06f;
    dts->initialize_parameters.min_segment_length = 0.03f;
    dts->enable_physics = false;
  }
  if (ImGui::Button("Board break [H]")) {
    simulated_time = 0.f;
    physics_parameters = {};
    demo_type = DemoType::BreakBoardHigh;
    board_experiment_setup_settings.center_damage = 0.f;
    board_experiment_setup_settings.left_pivot_type = static_cast<unsigned>(DynamicTreeStrands::PivotType::Transform);
    board_experiment_setup_settings.right_pivot_type = static_cast<unsigned>(DynamicTreeStrands::PivotType::Transform);
    dts->initialize_parameters.shear_stretch_strength = {500.f, 250.f};
    dts->initialize_parameters.bending_strength = {500.f, 250.f};
    dts->initialize_parameters.twisting_strength = {500.f, 250.f};
    dts->initialize_parameters.bundle_strength = {175.f, 175.f};
    dts->initialize_parameters.connectivity_strength = {250.f, 250.f};
    dts->initialize_parameters.max_segment_length = 0.06f;
    dts->initialize_parameters.min_segment_length = 0.03f;
    dts->BoardExperimentSetup(board_experiment_setup_settings);
    dts->enable_physics = false;
  }

  if (ImGui::Button("Twisting break")) {
    simulated_time = 0.f;
    physics_parameters = {};
    demo_type = DemoType::TwistingBreak;
    target_factor0 = 0.f;
    dts->initialize_parameters.shear_stretch_strength = glm::vec2(250.f);
    dts->initialize_parameters.bending_strength = glm::vec2(250.f);
    dts->initialize_parameters.twisting_strength = glm::vec2(250.f);
    dts->initialize_parameters.bundle_strength = glm::vec2(250.f);
    dts->initialize_parameters.connectivity_strength = glm::vec2(250.f);
    dts->initialize_parameters.max_segment_length = 0.06f;
    dts->initialize_parameters.min_segment_length = 0.03f;
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
    physics_parameters = {};
    target_factor0 = 0.f;
    demo_type = DemoType::BendingBreak;

    dts->initialize_parameters.shear_stretch_strength = glm::vec2(250.f);
    dts->initialize_parameters.bending_strength = glm::vec2(250.f);
    dts->initialize_parameters.twisting_strength = glm::vec2(250.f);
    dts->initialize_parameters.bundle_strength = glm::vec2(250.f);
    dts->initialize_parameters.connectivity_strength = glm::vec2(250.f);
    dts->initialize_parameters.max_segment_length = 0.06f;
    dts->initialize_parameters.min_segment_length = 0.03f;
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
    physics_parameters = {};
    target_factor0 = 0.f;
    demo_type = DemoType::ShearingBreak;

    dts->initialize_parameters.shear_stretch_strength = glm::vec2(250.f);
    dts->initialize_parameters.bending_strength = glm::vec2(250.f);
    dts->initialize_parameters.twisting_strength = glm::vec2(250.f);
    dts->initialize_parameters.bundle_strength = glm::vec2(250.f);
    dts->initialize_parameters.connectivity_strength = glm::vec2(250.f);
    dts->initialize_parameters.max_segment_length = 0.06f;
    dts->initialize_parameters.min_segment_length = 0.03f;
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
    physics_parameters = {};
    target_factor0 = 0.f;
    demo_type = DemoType::StretchingBreak;

    dts->initialize_parameters.shear_stretch_strength = glm::vec2(250.f);
    dts->initialize_parameters.bending_strength = glm::vec2(250.f);
    dts->initialize_parameters.twisting_strength = glm::vec2(250.f);
    dts->initialize_parameters.bundle_strength = glm::vec2(250.f);
    dts->initialize_parameters.connectivity_strength = glm::vec2(250.f);
    dts->initialize_parameters.max_segment_length = 0.06f;
    dts->initialize_parameters.min_segment_length = 0.03f;
    dts->initialize_parameters.damage.noise_descriptors.clear();
    auto& noise = dts->initialize_parameters.damage.noise_descriptors.emplace_back();
    noise.type = static_cast<unsigned>(NoiseType::Perlin);

    board_experiment_setup_settings.left_pivot_type = static_cast<unsigned>(DynamicTreeStrands::PivotType::Transform);
    board_experiment_setup_settings.right_pivot_type = static_cast<unsigned>(DynamicTreeStrands::PivotType::Transform);
    board_experiment_setup_settings.rod_dimension = {160, 10, 20};
    dts->BoardExperimentSetup(board_experiment_setup_settings);
    dts->enable_physics = false;
  }

  if (ImGui::Button("Sap/Heart Increase")) {
    simulated_time = 0.f;
    physics_parameters = {};
    target_factor0 = 1.5f;
    demo_type = DemoType::SapHeartIncrease;
    log_experiment_setup_settings.left_pivot_type = static_cast<unsigned>(DynamicTreeStrands::PivotType::Transform);
    log_experiment_setup_settings.right_pivot_type = static_cast<unsigned>(DynamicTreeStrands::PivotType::Transform);
    log_experiment_setup_settings.rod_segment_count = 10;
    log_experiment_setup_settings.rod_size = 3200;

    dts->initialize_parameters.damage.noise_descriptors.clear();
    auto& noise = dts->initialize_parameters.damage.noise_descriptors.emplace_back();
    noise.type = static_cast<unsigned>(NoiseType::Perlin);
    dts->initialize_parameters.shear_stretch_strength = glm::vec2(750.f, 50.f);
    dts->initialize_parameters.bending_strength = glm::vec2(750.f, 50.f);
    dts->initialize_parameters.twisting_strength = glm::vec2(750.f, 50.f);
    dts->initialize_parameters.bundle_strength = glm::vec2(750.f, 50.f);
    dts->initialize_parameters.connectivity_strength = glm::vec2(750.f, 50.f);
    dts->initialize_parameters.max_segment_length = 0.06f;
    dts->initialize_parameters.min_segment_length = 0.03f;
    // dts->initialize_parameters.wood_transition = 0.001f;
    dts->LogExperimentSetup(log_experiment_setup_settings);
    dts->enable_physics = false;
  }
  if (ImGui::Button("Sap/Heart Equal")) {
    simulated_time = 0.f;
    physics_parameters = {};
    target_factor0 = 1.5f;
    demo_type = DemoType::SapHeartEqual;
    log_experiment_setup_settings.left_pivot_type = static_cast<unsigned>(DynamicTreeStrands::PivotType::Transform);
    log_experiment_setup_settings.right_pivot_type = static_cast<unsigned>(DynamicTreeStrands::PivotType::Transform);
    log_experiment_setup_settings.rod_segment_count = 10;
    log_experiment_setup_settings.rod_size = 3200;

    dts->initialize_parameters.damage.noise_descriptors.clear();
    auto& noise = dts->initialize_parameters.damage.noise_descriptors.emplace_back();
    noise.type = static_cast<unsigned>(NoiseType::Perlin);
    dts->initialize_parameters.shear_stretch_strength = glm::vec2(500.f);
    dts->initialize_parameters.bending_strength = glm::vec2(500.f);
    dts->initialize_parameters.twisting_strength = glm::vec2(500.f);
    dts->initialize_parameters.bundle_strength = glm::vec2(500.f);
    dts->initialize_parameters.connectivity_strength = glm::vec2(500.f);
    dts->initialize_parameters.max_segment_length = 0.06f;
    dts->initialize_parameters.min_segment_length = 0.03f;
    // dts->initialize_parameters.wood_transition = 0.001f;
    dts->LogExperimentSetup(log_experiment_setup_settings);
    dts->enable_physics = false;
  }

  if (ImGui::Button("Sap/Heart Decrease")) {
    simulated_time = 0.f;
    physics_parameters = {};
    target_factor0 = 1.5f;
    demo_type = DemoType::SapHeartDecrease;
    log_experiment_setup_settings.left_pivot_type = static_cast<unsigned>(DynamicTreeStrands::PivotType::Transform);
    log_experiment_setup_settings.right_pivot_type = static_cast<unsigned>(DynamicTreeStrands::PivotType::Transform);
    log_experiment_setup_settings.rod_segment_count = 10;
    log_experiment_setup_settings.rod_size = 3200;

    dts->initialize_parameters.damage.noise_descriptors.clear();
    auto& noise = dts->initialize_parameters.damage.noise_descriptors.emplace_back();
    noise.type = static_cast<unsigned>(NoiseType::Perlin);
    dts->initialize_parameters.shear_stretch_strength = glm::vec2(50.f, 750.f);
    dts->initialize_parameters.bending_strength = glm::vec2(50.f, 750.f);
    dts->initialize_parameters.twisting_strength = glm::vec2(50.f, 750.f);
    dts->initialize_parameters.bundle_strength = glm::vec2(50.f, 750.f);
    dts->initialize_parameters.connectivity_strength = glm::vec2(50.f, 750.f);
    dts->initialize_parameters.max_segment_length = 0.06f;
    dts->initialize_parameters.min_segment_length = 0.03f;
    // dts->initialize_parameters.wood_transition = 0.001f;
    dts->LogExperimentSetup(log_experiment_setup_settings);
    dts->enable_physics = false;
  }

  if (ImGui::Button("Short Rod sphere Collision")) {
    simulated_time = 0.f;
    physics_parameters = {};
    physics_parameters.time_step = 0.01f;
    target_factor0 = 0.f;
    demo_type = DemoType::ShortRodSphereCollision;

    dts->initialize_parameters.shear_stretch_strength = glm::vec2(100.f);
    dts->initialize_parameters.bending_strength = glm::vec2(100.f);
    dts->initialize_parameters.twisting_strength = glm::vec2(100.f);
    dts->initialize_parameters.bundle_strength = glm::vec2(100.f);
    dts->initialize_parameters.connectivity_strength = glm::vec2(100.f);
    dts->initialize_parameters.max_segment_length = 0.03f;
    dts->initialize_parameters.min_segment_length = 0.015f;
    dts->initialize_parameters.damage.noise_descriptors.clear();
    auto& noise = dts->initialize_parameters.damage.noise_descriptors.emplace_back();
    noise.type = static_cast<unsigned>(NoiseType::Perlin);
    noise.multiplier = 0.95f;
    noise.shift = glm::vec3(1000.f);
    board_experiment_setup_settings.center_damage = 0.7f;
    dts->initialize_parameters.damage_scale_factor = glm::vec3(0.01f);
    board_experiment_setup_settings.left_pivot_type = static_cast<unsigned>(DynamicTreeStrands::PivotType::Transform);
    board_experiment_setup_settings.right_pivot_type = static_cast<unsigned>(DynamicTreeStrands::PivotType::Transform);
    board_experiment_setup_settings.rod_dimension = {160, 10, 20};
    
    const auto sphere_entity = scene->CreateEntity("Sphere");
    scene->GetOrSetPrivateComponent<DsSphereCollider>(sphere_entity);
    const auto mmr = scene->GetOrSetPrivateComponent<MeshRenderer>(sphere_entity).lock();
    mmr->mesh = Resources::TryGetResource<Mesh>("PRIMITIVE_SPHERE");
    mmr->material = ProjectManager::CreateTemporaryAsset<Material>();
    object_initial_pose.SetPosition(glm::vec3(.5f, 1.3f, 0));
    object_initial_pose.SetScale(glm::vec3(0.3f));
    scene->SetDataComponent(sphere_entity, object_initial_pose);

    const auto temp_entity = temp_entity_ref.Get();
    if (scene->IsEntityValid(temp_entity)) {
      scene->DeleteEntity(temp_entity);
    }
    temp_entity_ref = sphere_entity;
    dts->BoardExperimentSetup(board_experiment_setup_settings);
    dts->enable_physics = false;
  }
  if (ImGui::Button("Long Rod sphere Collision")) {
    simulated_time = 0.f;
    physics_parameters = {};
    physics_parameters.time_step = 0.01f;
    target_factor0 = 0.f;
    demo_type = DemoType::LongRodSphereCollision;

    dts->initialize_parameters.shear_stretch_strength = glm::vec2(100.f);
    dts->initialize_parameters.bending_strength = glm::vec2(100.f);
    dts->initialize_parameters.twisting_strength = glm::vec2(100.f);
    dts->initialize_parameters.bundle_strength = glm::vec2(100.f);
    dts->initialize_parameters.connectivity_strength = glm::vec2(100.f);
    dts->initialize_parameters.max_segment_length = 0.06f;
    dts->initialize_parameters.min_segment_length = 0.03f;
    dts->initialize_parameters.damage.noise_descriptors.clear();
    auto& noise = dts->initialize_parameters.damage.noise_descriptors.emplace_back();
    noise.type = static_cast<unsigned>(NoiseType::Perlin);
    noise.multiplier = 0.95f;
    noise.shift = glm::vec3(1000.f);
    board_experiment_setup_settings.center_damage = 0.7f;
    dts->initialize_parameters.damage_scale_factor = glm::vec3(0.01f);
    board_experiment_setup_settings.left_pivot_type = static_cast<unsigned>(DynamicTreeStrands::PivotType::Transform);
    board_experiment_setup_settings.right_pivot_type = static_cast<unsigned>(DynamicTreeStrands::PivotType::Transform);
    board_experiment_setup_settings.rod_dimension = {160, 10, 20};

    const auto sphere_entity = scene->CreateEntity("Sphere");
    scene->GetOrSetPrivateComponent<DsSphereCollider>(sphere_entity);
    const auto mmr = scene->GetOrSetPrivateComponent<MeshRenderer>(sphere_entity).lock();
    mmr->mesh = Resources::TryGetResource<Mesh>("PRIMITIVE_SPHERE");
    mmr->material = ProjectManager::CreateTemporaryAsset<Material>();
    object_initial_pose.SetPosition(glm::vec3(0.5f, 1.3f, 0));
    object_initial_pose.SetScale(glm::vec3(0.3f));
    scene->SetDataComponent(sphere_entity, object_initial_pose);

    const auto temp_entity = temp_entity_ref.Get();
    if (scene->IsEntityValid(temp_entity)) {
      scene->DeleteEntity(temp_entity);
    }
    temp_entity_ref = sphere_entity;
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
    case DemoType::SapHeartIncrease:
    case DemoType::SapHeartEqual:
    case DemoType::SapHeartDecrease: {
      const float log_distance = static_cast<float>(log_experiment_setup_settings.rod_segment_count) *
                                 log_experiment_setup_settings.segment_length;
      const float left_distance = -log_distance * 0.5f * progress * target_factor0;
      auto leaf_operator_root_transform = GlobalTransform();
      leaf_operator_root_transform.SetPosition(
          dts->initialize_parameters.root_transform.TransformPoint(glm::vec3(left_distance, 0, 0)));
      const float angle = glm::acos(1.f - progress * target_factor1);
      leaf_operator_root_transform.SetRotation(owner_gt.GetRotation() * glm::quat(glm::vec3(angle, 0, 0)));
      scene->SetDataComponent(left_pivot, leaf_operator_root_transform);
      break;
    }
    case DemoType::ShortRodSphereCollision: {
      GlobalTransform gt = object_initial_pose;
      const auto temp_entity = temp_entity_ref.Get();
      gt.SetPosition(object_initial_pose.GetPosition() + glm::vec3(0, -50, 0) * progress);
      scene->SetDataComponent(temp_entity, gt);
    }
    case DemoType::LongRodSphereCollision: {
      GlobalTransform gt = object_initial_pose;
      const auto temp_entity = temp_entity_ref.Get();
      gt.SetPosition(object_initial_pose.GetPosition() + glm::vec3(0, -50, 0) * progress);
      scene->SetDataComponent(temp_entity, gt);
    }
    default:
      break;
  }

  dts->PhysicsStep(physics_parameters);
  simulated_time += physics_parameters.time_step;
}

void DynamicStrandsDemo::LateUpdate() {
}
