#include "DynamicStrandsDemo.hpp"

#include "DsColliders.hpp"
#include "DynamicTreeStrands.hpp"
#include "EcoSysLabLayer.hpp"

using namespace eco_sys_lab_plugin;

void DynamicStrandsDemo::ResetEnvironment(const std::shared_ptr<EditorLayer>& editor_layer) {
  const auto owner = GetOwner();
  const auto scene = GetScene();
  const auto children = scene->GetChildren(owner);
  if (scene->HasPrivateComponent<DynamicTreeStrands>(owner)) {
    scene->RemovePrivateComponent<DynamicTreeStrands>(owner);
  }
  const auto dts = scene->GetOrSetPrivateComponent<DynamicTreeStrands>(owner).lock();

  target_simulation_time = 10.f;
  simulated_time = 0.f;
  //target_factor0 = 1.f;
  //target_factor1 = 1.f;
  //physics_parameters = {};
  physics_parameters.time_step = 0.01f;

  board_experiment_setup_settings.center_damage = 0.f;
  board_experiment_setup_settings.left_pivot_type = static_cast<unsigned>(DynamicTreeStrands::PivotType::Transform);
  board_experiment_setup_settings.right_pivot_type = static_cast<unsigned>(DynamicTreeStrands::PivotType::Transform);
  log_experiment_setup_settings.left_pivot_type = static_cast<unsigned>(DynamicTreeStrands::PivotType::Transform);
  log_experiment_setup_settings.right_pivot_type = static_cast<unsigned>(DynamicTreeStrands::PivotType::Transform);
  log_experiment_setup_settings.lock_upper = false;
  log_experiment_setup_settings.t_cut = false;
  log_experiment_setup_settings.cut_width = 0.7f;
  log_experiment_setup_settings.sweep_angle = 0.f;
  board_experiment_setup_settings.rod_dimension = {20, 40, 20};
  dts->initialize_parameters.shear_stretch_strength = {500.f, 250.f};
  dts->initialize_parameters.bending_strength = {500.f, 250.f};
  dts->initialize_parameters.twisting_strength = {500.f, 250.f};
  dts->initialize_parameters.bundle_strength = {500.f, 250.f};
  dts->initialize_parameters.connectivity_strength = {250.f, 125.f};
  dts->initialize_parameters.max_segment_length = 0.06f;
  dts->initialize_parameters.min_segment_length = 0.03f;
  dts->initialize_parameters.damage_scale_factor = glm::vec3(0.01f);
  dts->initialize_parameters.damage.noise_descriptors.clear();
  dts->enable_physics = false;
  object_initial_pose = {};
  tree_initial_pose = {};
  tree_initial_pose.SetPosition(glm::vec3(0, -0.05, 0));
  camera_pose = {};
  camera_pose.SetPosition(glm::vec3(0, 1, 4.5));
  camera_pose.SetEulerRotation(glm::radians(glm::vec3(0, 0, 0)));
  const auto scene_camera = editor_layer->GetSceneCamera();
  scene_camera->camera_settings.fov = 90.f;
  scene_camera->camera_settings.far_distance = 150.f;
  scene_camera->camera_settings.fade_ratio = 0.01f;
  for (const auto& child : children) {
    scene->DeleteEntity(child);
  }
  const auto temp_entity = temp_entity1_ref.Get();
  if (scene->IsEntityValid(temp_entity)) {
    scene->DeleteEntity(temp_entity);
  }
  const auto tree_entity = tree_entity_ref.Get();
  if (scene->IsEntityValid(tree_entity)) {
    scene->DeleteEntity(tree_entity);
  }
  const auto eco_sys_lab_layer = Application::GetLayer<EcoSysLabLayer>();
  const std::vector<Entity>* tree_entities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
  eco_sys_lab_layer->ResetAllTrees(tree_entities);
  physics_parameters.enable_structural_damage = true;
  physics_parameters.enable_segment_compression_disconnection = true;
  physics_parameters.segment_velocity_damping = 1.f;
  physics_parameters.segment_angular_velocity_damping = 1.f;
}

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
      demo_status = DemoStatus::Idle;
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
  if (ImGui::Button("Tension/Compression")) {
    ResetEnvironment(editor_layer);
    camera_pose.SetPosition(glm::vec3(0.5, 0.7, 1));
    camera_pose.SetEulerRotation(glm::radians(glm::vec3(10, 0, 0)));
    demo_type = DemoType::TensionCompression;
    demo_status = DemoStatus::Simulation;
    log_experiment_setup_settings.center_damage = 0.95f;
    log_experiment_setup_settings.center_distance_offset = 0.01f;
    log_experiment_setup_settings.center_damage_transition = 0.02f;
    log_experiment_setup_settings.sweep_angle = -60.f;
    dts->initialize_parameters.bundle_strength = {500.f, 50.f};
    dts->initialize_parameters.connectivity_strength = {100.f, 100.f};
    log_experiment_setup_settings.i_cut = true;
    log_experiment_setup_settings.cut_width = 0.7f;

    physics_parameters.enable_segment_disconnection = false;

    // target_factor0 = 2.f; // distance
    // target_factor1 = 1.f; // angle

    dts->LogExperimentSetup(log_experiment_setup_settings);
    editor_layer->SetSceneCameraRotation(camera_pose.GetRotation());
    editor_layer->SetSceneCameraPosition(camera_pose.GetPosition());
  }
  if (ImGui::Button("Log break [Diffuse]")) {
    ResetEnvironment(editor_layer);
    camera_pose.SetPosition(glm::vec3(0.5, 0.7, 1));
    camera_pose.SetEulerRotation(glm::radians(glm::vec3(10, 0, 0)));
    demo_type = DemoType::LogBreak;
    demo_status = DemoStatus::Simulation;
    log_experiment_setup_settings.center_damage = 0.95f;
    log_experiment_setup_settings.center_distance_offset = 0.01f;
    log_experiment_setup_settings.center_damage_transition = 0.02f;
    dts->initialize_parameters.bundle_strength = {500.f, 50.f};
    dts->initialize_parameters.connectivity_strength = {100.f, 100.f};
    log_experiment_setup_settings.t_cut = true;
    log_experiment_setup_settings.cut_width = 0.f;
    target_factor0 = 2.f;
    target_factor1 = 1.f;

    dts->LogExperimentSetup(log_experiment_setup_settings);
    editor_layer->SetSceneCameraRotation(camera_pose.GetRotation());
    editor_layer->SetSceneCameraPosition(camera_pose.GetPosition());
  }

  if (ImGui::Button("Log break [Clean]")) {
    ResetEnvironment(editor_layer);
    camera_pose.SetPosition(glm::vec3(0.5, 0.7, 1));
    camera_pose.SetEulerRotation(glm::radians(glm::vec3(10, 0, 0)));
    demo_type = DemoType::LogBreak;
    demo_status = DemoStatus::Simulation;
    log_experiment_setup_settings.center_damage = 1.f;
    log_experiment_setup_settings.center_distance_offset = 0.01f;
    log_experiment_setup_settings.center_damage_transition = 0.02f;
    dts->initialize_parameters.bundle_strength = {500.f, 50.f};
    dts->initialize_parameters.connectivity_strength = {250.f, 250.f};
    log_experiment_setup_settings.lock_upper = true;
    log_experiment_setup_settings.t_cut = true;
    target_factor0 = 0.3f;
    target_factor1 = 1.f;

    physics_parameters.enable_segment_compression_disconnection = false;
    dts->LogExperimentSetup(log_experiment_setup_settings);
    editor_layer->SetSceneCameraRotation(camera_pose.GetRotation());
    editor_layer->SetSceneCameraPosition(camera_pose.GetPosition());
  }
  if (ImGui::Button("Log break [Transverse buckling]")) {
    ResetEnvironment(editor_layer);
    camera_pose.SetPosition(glm::vec3(0.5, 0.7, 1));
    camera_pose.SetEulerRotation(glm::radians(glm::vec3(10, 0, 0)));
    demo_type = DemoType::LogBreak;
    demo_status = DemoStatus::Simulation;
    log_experiment_setup_settings.center_damage = 0.95f;
    log_experiment_setup_settings.center_distance_offset = 0.01f;
    log_experiment_setup_settings.center_damage_transition = 0.02f;
    dts->initialize_parameters.bundle_strength = {500.f, 50.f};
    dts->initialize_parameters.connectivity_strength = {250.f, 250.f};

    // target_factor0 = 0.3f;
    target_factor1 = 1.f;
    physics_parameters.enable_positional_breaking = false;
    physics_parameters.enable_segment_disconnection = false;
    dts->LogExperimentSetup(log_experiment_setup_settings);
    editor_layer->SetSceneCameraRotation(camera_pose.GetRotation());
    editor_layer->SetSceneCameraPosition(camera_pose.GetPosition());
  }

  if (ImGui::Button("Board break [Low]")) {
    ResetEnvironment(editor_layer);
    camera_pose.SetPosition(glm::vec3(-0.5, 1.5, 1));
    camera_pose.SetEulerRotation(glm::radians(glm::vec3(-20, -45, 0)));
    demo_type = DemoType::BoardBreak;
    demo_status = DemoStatus::Simulation;
    board_experiment_setup_settings.center_damage = 0.f;
    dts->initialize_parameters.bundle_strength = {500.f, 50.f};
    dts->initialize_parameters.connectivity_strength = {250.f, 250.f};
    dts->BoardExperimentSetup(board_experiment_setup_settings);
    editor_layer->SetSceneCameraRotation(camera_pose.GetRotation());
    editor_layer->SetSceneCameraPosition(camera_pose.GetPosition());
  }
  if (ImGui::Button("Board break [Medium]")) {
    ResetEnvironment(editor_layer);
    camera_pose.SetPosition(glm::vec3(-0.5, 1.5, 1));
    camera_pose.SetEulerRotation(glm::radians(glm::vec3(-20, -45, 0)));
    demo_type = DemoType::BoardBreak;
    demo_status = DemoStatus::Simulation;
    board_experiment_setup_settings.center_damage = 0.f;
    dts->initialize_parameters.bundle_strength = {100.f, 100.f};
    dts->initialize_parameters.connectivity_strength = {250.f, 250.f};
    dts->BoardExperimentSetup(board_experiment_setup_settings);
    editor_layer->SetSceneCameraRotation(camera_pose.GetRotation());
    editor_layer->SetSceneCameraPosition(camera_pose.GetPosition());
  }
  if (ImGui::Button("Board break [High]")) {
    ResetEnvironment(editor_layer);
    camera_pose.SetPosition(glm::vec3(-0.5, 1.5, 1));
    camera_pose.SetEulerRotation(glm::radians(glm::vec3(-20, -45, 0)));
    demo_type = DemoType::BoardBreak;
    demo_status = DemoStatus::Simulation;
    dts->initialize_parameters.bundle_strength = {175.f, 175.f};
    dts->initialize_parameters.connectivity_strength = {250.f, 250.f};
    dts->BoardExperimentSetup(board_experiment_setup_settings);
    editor_layer->SetSceneCameraRotation(camera_pose.GetRotation());
    editor_layer->SetSceneCameraPosition(camera_pose.GetPosition());
  }

  if (ImGui::Button("Twisting break")) {
    ResetEnvironment(editor_layer);
    camera_pose.SetPosition(glm::vec3(-0.5, 2.2, 1));
    camera_pose.SetEulerRotation(glm::radians(glm::vec3(-40, -45, 0)));
    demo_type = DemoType::TwistingBreak;
    demo_status = DemoStatus::Simulation;
    target_factor0 = 0.f;
    dts->initialize_parameters.shear_stretch_strength = glm::vec2(250.f);
    dts->initialize_parameters.bending_strength = glm::vec2(250.f);
    dts->initialize_parameters.twisting_strength = glm::vec2(250.f);
    dts->initialize_parameters.bundle_strength = glm::vec2(250.f);
    dts->initialize_parameters.connectivity_strength = glm::vec2(250.f);

    auto& noise = dts->initialize_parameters.damage.noise_descriptors.emplace_back();
    noise.type = static_cast<unsigned>(NoiseType::Perlin);

    board_experiment_setup_settings.rod_dimension = {160, 10, 20};
    dts->BoardExperimentSetup(board_experiment_setup_settings);
    editor_layer->SetSceneCameraRotation(camera_pose.GetRotation());
    editor_layer->SetSceneCameraPosition(camera_pose.GetPosition());
  }
  if (ImGui::Button("Bending break")) {
    ResetEnvironment(editor_layer);
    camera_pose.SetPosition(glm::vec3(-0.5, 2.2, 1));
    camera_pose.SetEulerRotation(glm::radians(glm::vec3(-40, -45, 0)));
    target_factor0 = 0.f;
    demo_type = DemoType::BendingBreak;
    demo_status = DemoStatus::Simulation;
    dts->initialize_parameters.shear_stretch_strength = glm::vec2(250.f);
    dts->initialize_parameters.bending_strength = glm::vec2(250.f);
    dts->initialize_parameters.twisting_strength = glm::vec2(250.f);
    dts->initialize_parameters.bundle_strength = glm::vec2(250.f);
    dts->initialize_parameters.connectivity_strength = glm::vec2(250.f);

    auto& noise = dts->initialize_parameters.damage.noise_descriptors.emplace_back();
    noise.type = static_cast<unsigned>(NoiseType::Perlin);

    board_experiment_setup_settings.left_pivot_type = static_cast<unsigned>(DynamicTreeStrands::PivotType::Transform);
    board_experiment_setup_settings.right_pivot_type = static_cast<unsigned>(DynamicTreeStrands::PivotType::Transform);
    board_experiment_setup_settings.rod_dimension = {160, 10, 20};
    dts->BoardExperimentSetup(board_experiment_setup_settings);
    editor_layer->SetSceneCameraRotation(camera_pose.GetRotation());
    editor_layer->SetSceneCameraPosition(camera_pose.GetPosition());
  }
  if (ImGui::Button("Shearing break")) {
    ResetEnvironment(editor_layer);
    camera_pose.SetPosition(glm::vec3(-0.5, 2.2, 1));
    camera_pose.SetEulerRotation(glm::radians(glm::vec3(-40, -45, 0)));
    target_factor0 = 0.f;
    demo_type = DemoType::ShearingBreak;
    demo_status = DemoStatus::Simulation;
    dts->initialize_parameters.shear_stretch_strength = glm::vec2(250.f);
    dts->initialize_parameters.bending_strength = glm::vec2(250.f);
    dts->initialize_parameters.twisting_strength = glm::vec2(250.f);
    dts->initialize_parameters.bundle_strength = glm::vec2(250.f);
    dts->initialize_parameters.connectivity_strength = glm::vec2(250.f);
    auto& noise = dts->initialize_parameters.damage.noise_descriptors.emplace_back();
    noise.type = static_cast<unsigned>(NoiseType::Perlin);

    board_experiment_setup_settings.rod_dimension = {160, 10, 20};
    dts->BoardExperimentSetup(board_experiment_setup_settings);
    editor_layer->SetSceneCameraRotation(camera_pose.GetRotation());
    editor_layer->SetSceneCameraPosition(camera_pose.GetPosition());
  }
  if (ImGui::Button("Stretching break")) {
    ResetEnvironment(editor_layer);
    camera_pose.SetPosition(glm::vec3(-0.5, 2.2, 1));
    camera_pose.SetEulerRotation(glm::radians(glm::vec3(-40, -45, 0)));
    target_factor0 = 0.f;
    demo_type = DemoType::StretchingBreak;
    demo_status = DemoStatus::Simulation;
    dts->initialize_parameters.shear_stretch_strength = glm::vec2(250.f);
    dts->initialize_parameters.bending_strength = glm::vec2(250.f);
    dts->initialize_parameters.twisting_strength = glm::vec2(250.f);
    dts->initialize_parameters.bundle_strength = glm::vec2(250.f);
    dts->initialize_parameters.connectivity_strength = glm::vec2(250.f);
    auto& noise = dts->initialize_parameters.damage.noise_descriptors.emplace_back();
    noise.type = static_cast<unsigned>(NoiseType::Perlin);

    board_experiment_setup_settings.rod_dimension = {160, 10, 20};
    dts->BoardExperimentSetup(board_experiment_setup_settings);
    editor_layer->SetSceneCameraRotation(camera_pose.GetRotation());
    editor_layer->SetSceneCameraPosition(camera_pose.GetPosition());
  }

  if (ImGui::Button("Sap/Heart Increase")) {
    ResetEnvironment(editor_layer);
    camera_pose.SetPosition(glm::vec3(-0.3, 1.3, 0.2));
    camera_pose.SetEulerRotation(glm::radians(glm::vec3(-30, -60, 0)));
    target_factor0 = 5.f;
    demo_type = DemoType::SapHeart;
    demo_status = DemoStatus::Simulation;
    log_experiment_setup_settings.rod_segment_count = 10;
    log_experiment_setup_settings.rod_size = 3200;

    auto& noise = dts->initialize_parameters.damage.noise_descriptors.emplace_back();
    noise.type = static_cast<unsigned>(NoiseType::Perlin);
    dts->initialize_parameters.shear_stretch_strength = glm::vec2(750.f, 50.f);
    dts->initialize_parameters.bending_strength = glm::vec2(750.f, 50.f);
    dts->initialize_parameters.twisting_strength = glm::vec2(750.f, 50.f);
    dts->initialize_parameters.bundle_strength = glm::vec2(750.f, 50.f);
    dts->initialize_parameters.connectivity_strength = glm::vec2(750.f, 50.f);
    dts->LogExperimentSetup(log_experiment_setup_settings);
    editor_layer->SetSceneCameraRotation(camera_pose.GetRotation());
    editor_layer->SetSceneCameraPosition(camera_pose.GetPosition());
  }
  if (ImGui::Button("Sap/Heart Equal")) {
    ResetEnvironment(editor_layer);
    camera_pose.SetPosition(glm::vec3(-0.3, 1.3, 0.2));
    camera_pose.SetEulerRotation(glm::radians(glm::vec3(-30, -60, 0)));
    target_factor0 = 5.f;
    demo_type = DemoType::SapHeart;
    demo_status = DemoStatus::Simulation;
    log_experiment_setup_settings.rod_segment_count = 10;
    log_experiment_setup_settings.rod_size = 3200;

    auto& noise = dts->initialize_parameters.damage.noise_descriptors.emplace_back();
    noise.type = static_cast<unsigned>(NoiseType::Perlin);
    dts->initialize_parameters.shear_stretch_strength = glm::vec2(500.f);
    dts->initialize_parameters.bending_strength = glm::vec2(500.f);
    dts->initialize_parameters.twisting_strength = glm::vec2(500.f);
    dts->initialize_parameters.bundle_strength = glm::vec2(500.f);
    dts->initialize_parameters.connectivity_strength = glm::vec2(500.f);
    dts->LogExperimentSetup(log_experiment_setup_settings);
    editor_layer->SetSceneCameraRotation(camera_pose.GetRotation());
    editor_layer->SetSceneCameraPosition(camera_pose.GetPosition());
  }
  if (ImGui::Button("Sap/Heart Decrease")) {
    ResetEnvironment(editor_layer);
    camera_pose.SetPosition(glm::vec3(-0.3, 1.3, 0.2));
    camera_pose.SetEulerRotation(glm::radians(glm::vec3(-30, -60, 0)));
    target_factor0 = 5.f;
    demo_type = DemoType::SapHeart;
    demo_status = DemoStatus::Simulation;
    log_experiment_setup_settings.rod_segment_count = 10;
    log_experiment_setup_settings.rod_size = 3200;

    auto& noise = dts->initialize_parameters.damage.noise_descriptors.emplace_back();
    noise.type = static_cast<unsigned>(NoiseType::Perlin);
    dts->initialize_parameters.shear_stretch_strength = glm::vec2(50.f, 750.f);
    dts->initialize_parameters.bending_strength = glm::vec2(50.f, 750.f);
    dts->initialize_parameters.twisting_strength = glm::vec2(50.f, 750.f);
    dts->initialize_parameters.bundle_strength = glm::vec2(50.f, 750.f);
    dts->initialize_parameters.connectivity_strength = glm::vec2(50.f, 750.f);
    dts->LogExperimentSetup(log_experiment_setup_settings);
    editor_layer->SetSceneCameraRotation(camera_pose.GetRotation());
    editor_layer->SetSceneCameraPosition(camera_pose.GetPosition());
  }

  if (ImGui::Button("Short rod sphere Collision")) {
    ResetEnvironment(editor_layer);
    camera_pose.SetPosition(glm::vec3(-0.5, 2.2, 1));
    camera_pose.SetEulerRotation(glm::radians(glm::vec3(-40, -45, 0)));
    target_factor0 = 0.f;
    demo_type = DemoType::BoardCollision;
    demo_status = DemoStatus::Simulation;
    dts->initialize_parameters.shear_stretch_strength = glm::vec2(100.f);
    dts->initialize_parameters.bending_strength = glm::vec2(100.f);
    dts->initialize_parameters.twisting_strength = glm::vec2(100.f);
    dts->initialize_parameters.bundle_strength = glm::vec2(100.f);
    dts->initialize_parameters.connectivity_strength = glm::vec2(100.f);
    dts->initialize_parameters.max_segment_length = 0.03f;
    dts->initialize_parameters.min_segment_length = 0.015f;

    auto& noise = dts->initialize_parameters.damage.noise_descriptors.emplace_back();
    noise.type = static_cast<unsigned>(NoiseType::Perlin);
    noise.multiplier = 0.95f;
    noise.shift = glm::vec3(1000.f);

    board_experiment_setup_settings.center_damage = 0.7f;
    board_experiment_setup_settings.rod_dimension = {160, 10, 20};

    const auto sphere_entity = scene->CreateEntity("Sphere");
    scene->GetOrSetPrivateComponent<DsSphereCollider>(sphere_entity);
    const auto mmr = scene->GetOrSetPrivateComponent<MeshRenderer>(sphere_entity).lock();
    mmr->mesh = Resources::TryGetResource<Mesh>("PRIMITIVE_SPHERE");
    mmr->material = ProjectManager::CreateTemporaryAsset<Material>();
    object_initial_pose.SetPosition(glm::vec3(.5f, 1.3f, 0));
    object_initial_pose.SetScale(glm::vec3(0.3f));
    scene->SetDataComponent(sphere_entity, object_initial_pose);

    const auto temp_entity = temp_entity1_ref.Get();
    if (scene->IsEntityValid(temp_entity)) {
      scene->DeleteEntity(temp_entity);
    }
    temp_entity1_ref = sphere_entity;
    dts->BoardExperimentSetup(board_experiment_setup_settings);
    editor_layer->SetSceneCameraRotation(camera_pose.GetRotation());
    editor_layer->SetSceneCameraPosition(camera_pose.GetPosition());
  }
  if (ImGui::Button("Long rod sphere Collision")) {
    ResetEnvironment(editor_layer);
    camera_pose.SetPosition(glm::vec3(-0.5, 2.2, 1));
    camera_pose.SetEulerRotation(glm::radians(glm::vec3(-40, -45, 0)));
    target_factor0 = 0.f;
    demo_type = DemoType::BoardCollision;
    demo_status = DemoStatus::Simulation;
    dts->initialize_parameters.shear_stretch_strength = glm::vec2(100.f);
    dts->initialize_parameters.bending_strength = glm::vec2(100.f);
    dts->initialize_parameters.twisting_strength = glm::vec2(100.f);
    dts->initialize_parameters.bundle_strength = glm::vec2(100.f);
    dts->initialize_parameters.connectivity_strength = glm::vec2(100.f);
    auto& noise = dts->initialize_parameters.damage.noise_descriptors.emplace_back();
    noise.type = static_cast<unsigned>(NoiseType::Perlin);
    noise.multiplier = 0.95f;
    noise.shift = glm::vec3(1000.f);
    board_experiment_setup_settings.center_damage = 0.7f;
    dts->initialize_parameters.damage_scale_factor = glm::vec3(0.01f);
    board_experiment_setup_settings.rod_dimension = {160, 10, 20};

    const auto sphere_entity = scene->CreateEntity("Sphere");
    scene->GetOrSetPrivateComponent<DsSphereCollider>(sphere_entity);
    const auto mmr = scene->GetOrSetPrivateComponent<MeshRenderer>(sphere_entity).lock();
    mmr->mesh = Resources::TryGetResource<Mesh>("PRIMITIVE_SPHERE");
    mmr->material = ProjectManager::CreateTemporaryAsset<Material>();
    object_initial_pose.SetPosition(glm::vec3(0.5f, 1.3f, 0));
    object_initial_pose.SetScale(glm::vec3(0.3f));
    scene->SetDataComponent(sphere_entity, object_initial_pose);
    temp_entity1_ref = sphere_entity;
    dts->BoardExperimentSetup(board_experiment_setup_settings);
    editor_layer->SetSceneCameraRotation(camera_pose.GetRotation());
    editor_layer->SetSceneCameraPosition(camera_pose.GetPosition());
  }

  if (ImGui::Button("Small cylinder Collision")) {
    ResetEnvironment(editor_layer);
    camera_pose.SetPosition(glm::vec3(-0.5, 2.2, 1));
    camera_pose.SetEulerRotation(glm::radians(glm::vec3(-40, -45, 0)));
    target_factor0 = 0.f;
    demo_type = DemoType::BoardCollision;
    demo_status = DemoStatus::Simulation;
    dts->initialize_parameters.shear_stretch_strength = glm::vec2(100.f);
    dts->initialize_parameters.bending_strength = glm::vec2(100.f);
    dts->initialize_parameters.twisting_strength = glm::vec2(100.f);
    dts->initialize_parameters.bundle_strength = glm::vec2(100.f);
    dts->initialize_parameters.connectivity_strength = glm::vec2(100.f);
    auto& noise = dts->initialize_parameters.damage.noise_descriptors.emplace_back();
    noise.type = static_cast<unsigned>(NoiseType::Perlin);
    noise.multiplier = 0.99f;
    board_experiment_setup_settings.center_damage = 0.0f;
    dts->initialize_parameters.damage_scale_factor = glm::vec3(0.005f, 0.1f, 0.05f);
    board_experiment_setup_settings.rod_dimension = {160, 10, 20};

    const auto cylinder_entity = scene->CreateEntity("Cylinder");
    scene->GetOrSetPrivateComponent<DsCylinderCollider>(cylinder_entity);
    const auto mmr = scene->GetOrSetPrivateComponent<MeshRenderer>(cylinder_entity).lock();
    mmr->mesh = Resources::TryGetResource<Mesh>("PRIMITIVE_CYLINDER");
    mmr->material = ProjectManager::CreateTemporaryAsset<Material>();

    object_initial_pose.SetPosition(glm::vec3(0.5f, 1.3f, 0));
    object_initial_pose.SetEulerRotation(glm::radians(glm::vec3(90.f, 0, 0)));
    object_initial_pose.SetScale(glm::vec3(0.2f));
    scene->SetDataComponent(cylinder_entity, object_initial_pose);

    temp_entity1_ref = cylinder_entity;
    dts->BoardExperimentSetup(board_experiment_setup_settings);
    editor_layer->SetSceneCameraRotation(camera_pose.GetRotation());
    editor_layer->SetSceneCameraPosition(camera_pose.GetPosition());
  }
  if (ImGui::Button("Big cylinder Collision")) {
    ResetEnvironment(editor_layer);
    camera_pose.SetPosition(glm::vec3(-0.5, 2.2, 1));
    camera_pose.SetEulerRotation(glm::radians(glm::vec3(-40, -45, 0)));
    target_factor0 = 0.f;
    demo_type = DemoType::BoardCollision;
    demo_status = DemoStatus::Simulation;
    dts->initialize_parameters.shear_stretch_strength = glm::vec2(100.f);
    dts->initialize_parameters.bending_strength = glm::vec2(100.f);
    dts->initialize_parameters.twisting_strength = glm::vec2(100.f);
    dts->initialize_parameters.bundle_strength = glm::vec2(100.f);
    dts->initialize_parameters.connectivity_strength = glm::vec2(100.f);
    auto& noise = dts->initialize_parameters.damage.noise_descriptors.emplace_back();
    noise.type = static_cast<unsigned>(NoiseType::Perlin);
    noise.multiplier = 0.99f;
    board_experiment_setup_settings.center_damage = 0.0f;
    dts->initialize_parameters.damage_scale_factor = glm::vec3(0.005f, 0.1f, 0.05f);
    board_experiment_setup_settings.rod_dimension = {160, 10, 20};

    const auto cylinder_entity = scene->CreateEntity("Cylinder");
    scene->GetOrSetPrivateComponent<DsCylinderCollider>(cylinder_entity);
    const auto mmr = scene->GetOrSetPrivateComponent<MeshRenderer>(cylinder_entity).lock();
    mmr->mesh = Resources::TryGetResource<Mesh>("PRIMITIVE_CYLINDER");
    mmr->material = ProjectManager::CreateTemporaryAsset<Material>();
    object_initial_pose.SetPosition(glm::vec3(0.5f, 1.3f, 0));
    object_initial_pose.SetEulerRotation(glm::radians(glm::vec3(90.f, 0, 0)));
    object_initial_pose.SetScale(glm::vec3(0.4f));
    scene->SetDataComponent(cylinder_entity, object_initial_pose);

    temp_entity1_ref = cylinder_entity;
    dts->BoardExperimentSetup(board_experiment_setup_settings);
    editor_layer->SetSceneCameraRotation(camera_pose.GetRotation());
    editor_layer->SetSceneCameraPosition(camera_pose.GetPosition());
  }

  if (ImGui::Button("Trunk Strength [Low]")) {
    ResetEnvironment(editor_layer);
    demo_type = DemoType::TrunkStrength;
    demo_status = DemoStatus::TreeGrowth;
    const auto tree_entity = scene->CreateEntity("Tree");
    tree_entity_ref = tree_entity;
    const auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock();
    tree->tree_model.seed = 10;
    // tree.
    scene->SetDataComponent(tree_entity, tree_initial_pose);
    target_growth_time = 8.f;
    tree->tree_descriptor_ref = ProjectManager::GetOrCreateAsset("./TreeDescriptors/Acacia.tree");
    const auto tree_dts = scene->GetOrSetPrivateComponent<DynamicTreeStrands>(tree_entity).lock();
    tree_dts->initialize_parameters.trunk_additional_strength_factor = 500.f;
    tree_dts->enable_physics = false;
    editor_layer->SetSceneCameraRotation(camera_pose.GetRotation());
    editor_layer->SetSceneCameraPosition(camera_pose.GetPosition());
  }
  if (ImGui::Button("Trunk Strength [High]")) {
    ResetEnvironment(editor_layer);
    demo_type = DemoType::TrunkStrength;
    demo_status = DemoStatus::TreeGrowth;
    const auto tree_entity = scene->CreateEntity("Tree");
    tree_entity_ref = tree_entity;
    const auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock();
    tree->tree_model.seed = 10;
    // tree.
    scene->SetDataComponent(tree_entity, tree_initial_pose);
    target_growth_time = 8.f;
    tree->tree_descriptor_ref = ProjectManager::GetOrCreateAsset("./TreeDescriptors/Acacia.tree");
    const auto tree_dts = scene->GetOrSetPrivateComponent<DynamicTreeStrands>(tree_entity).lock();
    tree_dts->enable_physics = false;
    tree_dts->initialize_parameters.trunk_additional_strength_factor = 1250.f;
    editor_layer->SetSceneCameraRotation(camera_pose.GetRotation());
    editor_layer->SetSceneCameraPosition(camera_pose.GetPosition());
  }

  if (ImGui::Button("Wind [Low]")) {
    ResetEnvironment(editor_layer);
    demo_type = DemoType::Wind;
    demo_status = DemoStatus::TreeGrowth;
    const auto tree_entity = scene->CreateEntity("Tree");
    tree_entity_ref = tree_entity;
    const auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock();
    // tree.
    scene->SetDataComponent(tree_entity, tree_initial_pose);
    target_growth_time = 8.f;
    tree->tree_descriptor_ref = ProjectManager::GetOrCreateAsset("./TreeDescriptors/Oak.tree");
    const auto tree_dts = scene->GetOrSetPrivateComponent<DynamicTreeStrands>(tree_entity).lock();
    tree_dts->enable_physics = false;
    target_factor0 = 0.05f;
    physics_parameters.segment_velocity_damping = 10.f;
    physics_parameters.segment_angular_velocity_damping = 10.f;
    editor_layer->SetSceneCameraRotation(camera_pose.GetRotation());
    editor_layer->SetSceneCameraPosition(camera_pose.GetPosition());
  }
  if (ImGui::Button("Wind [High]")) {
    ResetEnvironment(editor_layer);
    demo_type = DemoType::Wind;
    demo_status = DemoStatus::TreeGrowth;
    const auto tree_entity = scene->CreateEntity("Tree");
    tree_entity_ref = tree_entity;
    const auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock();
    // tree.
    scene->SetDataComponent(tree_entity, tree_initial_pose);
    target_growth_time = 8.f;
    tree->tree_descriptor_ref = ProjectManager::GetOrCreateAsset("./TreeDescriptors/Oak.tree");
    const auto tree_dts = scene->GetOrSetPrivateComponent<DynamicTreeStrands>(tree_entity).lock();
    tree_dts->enable_physics = false;
    target_factor0 = 0.16f;
    physics_parameters.segment_velocity_damping = 10.f;
    physics_parameters.segment_angular_velocity_damping = 10.f;
    editor_layer->SetSceneCameraRotation(camera_pose.GetRotation());
    editor_layer->SetSceneCameraPosition(camera_pose.GetPosition());
  }

  if (ImGui::Button("Tree Collision")) {
    ResetEnvironment(editor_layer);
    demo_type = DemoType::TreeCollision;
    demo_status = DemoStatus::TreeGrowth;
    const auto tree_entity = scene->CreateEntity("Tree");
    tree_entity_ref = tree_entity;
    const auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock();
    // tree.
    scene->SetDataComponent(tree_entity, tree_initial_pose);
    target_growth_time = 8.f;
    tree->tree_descriptor_ref = ProjectManager::GetOrCreateAsset("./TreeDescriptors/Acacia.tree");
    const auto tree_dts = scene->GetOrSetPrivateComponent<DynamicTreeStrands>(tree_entity).lock();
    tree_dts->enable_physics = false;
    target_factor0 = 0.03f;
    tree->tree_model.seed = 8;
    const auto cylinder_entity = scene->CreateEntity("Cylinder");
    scene->GetOrSetPrivateComponent<DsCylinderCollider>(cylinder_entity);
    const auto mmr = scene->GetOrSetPrivateComponent<MeshRenderer>(cylinder_entity).lock();
    mmr->mesh = Resources::TryGetResource<Mesh>("PRIMITIVE_CYLINDER");
    mmr->material = ProjectManager::CreateTemporaryAsset<Material>();
    object_initial_pose.SetPosition(glm::vec3(-1.f, .5f, 0));
    object_initial_pose.SetEulerRotation(glm::radians(glm::vec3(90.f, 0, 0)));
    object_initial_pose.SetScale(glm::vec3(0.2f, 2.f, 0.2f));
    scene->SetDataComponent(cylinder_entity, object_initial_pose);

    temp_entity1_ref = cylinder_entity;
    editor_layer->SetSceneCameraRotation(camera_pose.GetRotation());
    editor_layer->SetSceneCameraPosition(camera_pose.GetPosition());
  }

  if (ImGui::Button("Tree Break")) {
    ResetEnvironment(editor_layer);
    demo_type = DemoType::TreeBreak;
    demo_status = DemoStatus::TreeGrowth;
    const auto tree_entity = scene->CreateEntity("Tree");
    tree_entity_ref = tree_entity;
    const auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock();
    // tree.
    scene->SetDataComponent(tree_entity, tree_initial_pose);
    target_growth_time = 12.f;
    tree->tree_descriptor_ref = ProjectManager::GetOrCreateAsset("./TreeDescriptors/Demo.tree");
    const auto tree_dts = scene->GetOrSetPrivateComponent<DynamicTreeStrands>(tree_entity).lock();
    tree_dts->enable_physics = false;
    tree->tree_model.seed = 3;
    tree->strand_model_parameters.strand_radius_distribution.mean.min_value = 0.f;
    tree->strand_model_parameters.strand_radius_distribution.mean.max_value = 0.002f;
    auto& curve_values = tree->strand_model_parameters.strand_radius_distribution.mean.curve.UnsafeGetValues();
    curve_values.clear();
    curve_values.emplace_back(-0.1, 0);
    curve_values.emplace_back(0, 1);
    curve_values.emplace_back(0.1, 0);

    curve_values.emplace_back(-0.4, 0);
    curve_values.emplace_back(0.5, 0.5);
    curve_values.emplace_back(0.1, 0);

    curve_values.emplace_back(-0.1, 0);
    curve_values.emplace_back(1, 0.5);
    curve_values.emplace_back(0.1, 0);
  }


  if (ImGui::Button("Hardwood")) {
    ResetEnvironment(editor_layer);
    camera_pose.SetPosition(glm::vec3(0, 1, 5));
    camera_pose.SetEulerRotation(glm::radians(glm::vec3(0, 0, 0)));
    const auto scene_camera = editor_layer->GetSceneCamera();

    scene_camera->camera_settings.fov = 90;

    demo_type = DemoType::WoodType;
    demo_status = DemoStatus::TreeGrowth;
    const auto tree_entity = scene->CreateEntity("Tree");
    tree_entity_ref = tree_entity;
    const auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock();
    // tree.
    tree_initial_pose.SetEulerRotation(glm::radians(glm::vec3(0, 0, 45)));
    scene->SetDataComponent(tree_entity, tree_initial_pose);
    target_growth_time = 8.f;
    tree->tree_descriptor_ref = ProjectManager::GetOrCreateAsset("./TreeDescriptors/Demo.tree");
    tree->pruning_settings.low_branch_pruning = 0.3f;
    const auto tree_dts = scene->GetOrSetPrivateComponent<DynamicTreeStrands>(tree_entity).lock();
    tree_dts->enable_physics = false;
    tree->tree_model.seed = 0;

    tree->strand_model_parameters.strand_radius_distribution.mean.min_value = 0.f;
    tree->strand_model_parameters.strand_radius_distribution.mean.max_value = 0.0015f;
    auto& curve_values = tree->strand_model_parameters.strand_radius_distribution.mean.curve.UnsafeGetValues();
    curve_values.clear();
    curve_values.emplace_back(-0.1, 0);
    curve_values.emplace_back(0, 0.75);
    curve_values.emplace_back(0.1, 0);

    curve_values.emplace_back(-0.4, 0);
    curve_values.emplace_back(0.5, 0.75);
    curve_values.emplace_back(0.1, 0);

    curve_values.emplace_back(-0.1, 0);
    curve_values.emplace_back(1, 0.5);
    curve_values.emplace_back(0.1, 0);

    editor_layer->SetSceneCameraRotation(camera_pose.GetRotation());
    editor_layer->SetSceneCameraPosition(camera_pose.GetPosition());
  }
  if (ImGui::Button("Softwood")) {
    ResetEnvironment(editor_layer);
    camera_pose.SetPosition(glm::vec3(0, 1.5, 7.5));
    camera_pose.SetEulerRotation(glm::radians(glm::vec3(0, 0, 0)));
    editor_layer->GetSceneCamera()->camera_settings.fov = 90;
    demo_type = DemoType::WoodType;
    demo_status = DemoStatus::TreeGrowth;
    const auto tree_entity = scene->CreateEntity("Tree");
    tree_entity_ref = tree_entity;
    const auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock();
    // tree.
    tree_initial_pose.SetEulerRotation(glm::radians(glm::vec3(0, 0, 45)));
    scene->SetDataComponent(tree_entity, tree_initial_pose);
    target_growth_time = 8.f;
    tree->pruning_settings.low_branch_pruning = 0.3f;
    tree->tree_descriptor_ref = ProjectManager::GetOrCreateAsset("./TreeDescriptors/Demo.tree");
    const auto tree_dts = scene->GetOrSetPrivateComponent<DynamicTreeStrands>(tree_entity).lock();
    tree_dts->enable_physics = false;
    tree->tree_model.seed = 0;

    tree->strand_model_parameters.strand_radius_distribution.mean.min_value = 0.f;
    tree->strand_model_parameters.strand_radius_distribution.mean.max_value = 0.002f;
    auto& curve_values = tree->strand_model_parameters.strand_radius_distribution.mean.curve.UnsafeGetValues();
    curve_values.clear();
    curve_values.emplace_back(-0.1, 0);
    curve_values.emplace_back(0, 1);
    curve_values.emplace_back(0.1, 0);

    curve_values.emplace_back(-0.4, 0);
    curve_values.emplace_back(0.5, 0.75);
    curve_values.emplace_back(0.1, 0);

    curve_values.emplace_back(-0.1, 0);
    curve_values.emplace_back(1, 0.5);
    curve_values.emplace_back(0.1, 0);

    editor_layer->SetSceneCameraRotation(camera_pose.GetRotation());
    editor_layer->SetSceneCameraPosition(camera_pose.GetPosition());
  }

  return changed;
}

void DynamicStrandsDemo::Update() {
  if (demo_status == DemoStatus::Idle)
    return;
  if (demo_status == DemoStatus::Simulation && simulated_time >= target_simulation_time) {
    demo_type = DemoType::Empty;
    demo_status = DemoStatus::Idle;
    return;
  }
  const auto owner = GetOwner();
  const auto scene = GetScene();
  const auto dts = scene->GetOrSetPrivateComponent<DynamicTreeStrands>(owner).lock();

  if (demo_status == DemoStatus::TreeGrowth) {
    const auto eco_sys_lab_layer = Application::GetLayer<EcoSysLabLayer>();
    eco_sys_lab_layer->Simulate(simulation_settings, simulation_stats);
    if (eco_sys_lab_layer->GetSimulatedTime() >= target_growth_time) {
      const auto tree_entity = tree_entity_ref.Get();
      if (scene->IsEntityValid(tree_entity)) {
        const auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock();
        const auto tree_dts = scene->GetOrSetPrivateComponent<DynamicTreeStrands>(tree_entity).lock();
        tree_dts->InitializeFromTree(tree);
      } else {
        demo_type = DemoType::Empty;
        demo_status = DemoStatus::Idle;
      }
      demo_status = DemoStatus::Simulation;
    }
    return;
  }

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
    case DemoType::TensionCompression: {
      const float board_distance = static_cast<float>(log_experiment_setup_settings.rod_segment_count) *
                                   log_experiment_setup_settings.segment_length;
      const float left_distance = board_distance * -0.5f * progress * target_factor0;
      const float right_distance = board_distance * (1.f + 0.5f * progress * target_factor0);

      auto left_operator_root_transform = GlobalTransform();
      left_operator_root_transform.SetPosition(
          dts->initialize_parameters.root_transform.TransformPoint(glm::vec3(left_distance, 0, 0)));
      auto right_operator_root_transform = GlobalTransform();
      right_operator_root_transform.SetPosition(
          dts->initialize_parameters.root_transform.TransformPoint(glm::vec3(right_distance, 0, 0)));
      const float angle = glm::acos(1.f - progress * target_factor1);
      left_operator_root_transform.SetRotation(owner_gt.GetRotation() * glm::quat(glm::vec3(0, 0, angle)));
      right_operator_root_transform.SetRotation(owner_gt.GetRotation() * glm::quat(glm::vec3(0, 0, -angle)));
      scene->SetDataComponent(left_pivot, left_operator_root_transform);
      scene->SetDataComponent(right_pivot, right_operator_root_transform);
      dts->PhysicsStep(physics_parameters);
    } break;
    case DemoType::LogBreak: {
      const float board_distance = static_cast<float>(log_experiment_setup_settings.rod_segment_count) *
                                   log_experiment_setup_settings.segment_length;
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
      dts->PhysicsStep(physics_parameters);
    } break;
    case DemoType::BoardBreak: {
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
      dts->PhysicsStep(physics_parameters);
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
      dts->PhysicsStep(physics_parameters);
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
      dts->PhysicsStep(physics_parameters);
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
      dts->PhysicsStep(physics_parameters);
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
      dts->PhysicsStep(physics_parameters);
    } break;
    case DemoType::SapHeart: {
      const float log_distance = static_cast<float>(log_experiment_setup_settings.rod_segment_count) *
                                 log_experiment_setup_settings.segment_length;
      const float left_distance = -log_distance * 0.5f * progress * target_factor0;
      auto leaf_operator_root_transform = GlobalTransform();
      leaf_operator_root_transform.SetPosition(
          dts->initialize_parameters.root_transform.TransformPoint(glm::vec3(left_distance, 0, 0)));
      const float angle = glm::acos(1.f - progress * target_factor1);
      leaf_operator_root_transform.SetRotation(owner_gt.GetRotation() * glm::quat(glm::vec3(angle, 0, 0)));
      scene->SetDataComponent(left_pivot, leaf_operator_root_transform);
      dts->PhysicsStep(physics_parameters);
      break;
    }
    case DemoType::BoardCollision: {
      GlobalTransform gt = object_initial_pose;
      const auto temp_entity = temp_entity1_ref.Get();
      gt.SetPosition(object_initial_pose.GetPosition() + glm::vec3(0, -50, 0) * progress);
      scene->SetDataComponent(temp_entity, gt);
      dts->PhysicsStep(physics_parameters);
      break;
    }
    case DemoType::TrunkStrength: {
      GlobalTransform gt = tree_initial_pose;
      const auto tree_entity = tree_entity_ref.Get();
      if (scene->IsEntityValid(tree_entity)) {
        const float real_progress = glm::clamp(simulated_time / (target_simulation_time * .005f), 0.f, 1.f);
        gt.SetEulerRotation(glm::radians(glm::vec3(0, glm::pow(real_progress, 2.f) * 180.f, 0)));
        scene->SetDataComponent(tree_entity, gt);
        const auto tree_dts = scene->GetOrSetPrivateComponent<DynamicTreeStrands>(tree_entity).lock();
        tree_dts->PhysicsStep(physics_parameters);
      }
      break;
    }
    case DemoType::Wind: {
      const auto tree_entity = tree_entity_ref.Get();
      if (scene->IsEntityValid(tree_entity)) {
        const float real_progress = glm::clamp(simulated_time / (target_simulation_time * 0.05f), 0.f, 1.f);
        const auto tree_dts = scene->GetOrSetPrivateComponent<DynamicTreeStrands>(tree_entity).lock();
        tree_dts->wind->enabled = true;
        tree_dts->wind->main_force = glm::vec3((simulated_time > 1.f ? 0.f : -target_factor0 * real_progress), 0, 0);
        tree_dts->PhysicsStep(physics_parameters);
      }
      break;
    }
    case DemoType::TreeCollision: {
      GlobalTransform gt = object_initial_pose;
      const auto temp_entity = temp_entity1_ref.Get();
      const float real_progress = glm::clamp((simulated_time - .5f) / (target_simulation_time * 0.02f), 0.f, 1.f);
      gt.SetPosition(object_initial_pose.GetPosition() + glm::vec3(2, 0, 0) * real_progress);
      scene->SetDataComponent(temp_entity, gt);
      const auto tree_entity = tree_entity_ref.Get();
      if (scene->IsEntityValid(tree_entity)) {
        const auto tree_dts = scene->GetOrSetPrivateComponent<DynamicTreeStrands>(tree_entity).lock();
        tree_dts->PhysicsStep(physics_parameters);
      }
      break;
    }
    default:
      break;
  }

  simulated_time += physics_parameters.time_step;
}
