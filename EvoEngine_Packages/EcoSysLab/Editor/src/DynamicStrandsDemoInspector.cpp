#include "DsColliders.hpp"
#include "DynamicStrandsComponentInspectors.hpp"
#include "DynamicStrandsDemo.hpp"
#include "DynamicTreeStrands.hpp"
#include "DynamicsSettingsEditor.hpp"
#include "EcoSysLabLayer.hpp"
#include "EditorLayer.hpp"
#include "EditorWidgets.hpp"
#include "InspectorRegistry.hpp"

using namespace evo_engine;
using namespace eco_sys_lab_package;
void DynamicStrandsDemoInspector::ResetEnvironment(DynamicStrandsDemo& target,
                                                   const std::shared_ptr<EditorLayer>& editor_layer) {
  const auto owner = target.GetOwner();
  const auto scene = target.GetScene();
  const auto children = scene->GetChildren(owner);
  if (scene->HasPrivateComponent<DynamicTreeStrands>(owner)) {
    scene->RemovePrivateComponent<DynamicTreeStrands>(owner);
  }
  const auto dts = scene->GetOrSetPrivateComponent<DynamicTreeStrands>(owner).lock();

  target.target_simulation_time = 100.f;
  target.simulated_time = 0.f;
  // target_factor0 = 1.f;
  target.target_factor1 = 1.f;
  target.physics_parameters = {};
  target.physics_parameters.time_step = 0.005f;
  target.physics_parameters.enable_segment_collision = false;

  target.board_experiment_setup_settings.center_damage = 0.f;
  target.board_experiment_setup_settings.left_pivot_type =
      static_cast<unsigned>(DynamicTreeStrands::PivotType::Transform);
  target.board_experiment_setup_settings.right_pivot_type =
      static_cast<unsigned>(DynamicTreeStrands::PivotType::Transform);
  target.log_experiment_setup_settings.left_pivot_type =
      static_cast<unsigned>(DynamicTreeStrands::PivotType::Transform);
  target.log_experiment_setup_settings.right_pivot_type =
      static_cast<unsigned>(DynamicTreeStrands::PivotType::Transform);
  target.log_experiment_setup_settings.lock_upper = false;
  target.log_experiment_setup_settings.t_cut = false;
  target.log_experiment_setup_settings.t_cut_width = 0.7f;
  target.board_experiment_setup_settings.rod_dimension = {20, 40, 20};

  dts->initialize_parameters.strength_graph.SetShearStretchStrength({500.f, 250.f});
  dts->initialize_parameters.strength_graph.SetBendingStrength({500.f, 250.f});
  dts->initialize_parameters.strength_graph.SetTwistingStrength({500.f, 250.f});
  dts->initialize_parameters.strength_graph.SetBundleStrength({500.f, 250.f});
  dts->initialize_parameters.strength_graph.SetConnectivityStrength({250.f, 125.f});

  dts->initialize_parameters.max_segment_length = 0.06f;
  dts->initialize_parameters.min_segment_length = 0.03f;
  dts->initialize_parameters.damage_scale_factor = glm::vec3(0.01f);
  dts->initialize_parameters.damage_graph.Reset();
  dts->enable_physics = false;
  target.object_initial_pose = {};
  target.tree_initial_pose = {};
  target.tree_initial_pose.SetPosition(glm::vec3(0, -0.05, 0));
  camera_pose = {};
  camera_pose.SetPosition(glm::vec3(0, 1, 4.5));
  camera_pose.SetEulerRotation(glm::radians(glm::vec3(0, 0, 0)));
  editor_layer->GetSceneCamera()->camera_settings.fov = 120;
  for (const auto& child : children) {
    scene->DeleteEntity(child);
  }
  const auto temp_entity = target.temp_entity1_ref.Get();
  if (scene->IsEntityValid(temp_entity)) {
    scene->DeleteEntity(temp_entity);
  }
  const auto tree_entity = target.tree_entity_ref.Get();
  if (scene->IsEntityValid(tree_entity)) {
    scene->DeleteEntity(tree_entity);
  }
  const auto eco_sys_lab_layer = ApplicationContext::Get().GetLayer<EcoSysLabLayer>();
  const std::vector<Entity>* tree_entities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
  eco_sys_lab_layer->ResetAllTrees(tree_entities);
  target.physics_parameters.enable_structural_damage = true;
  target.physics_parameters.enable_segment_compression_disconnection = true;
  target.physics_parameters.segment_velocity_damping = 1.f;
  target.physics_parameters.segment_angular_velocity_damping = 1.f;
}
bool DynamicStrandsDemoInspector::Inspect(InspectorContext& context, DynamicStrandsDemo& target) {
  const auto& editor_layer = context.editor_layer;
  auto& inspector_context = context;
  if (ImGui::TreeNode("Physics Parameters")) {
    InspectSettings(target.physics_parameters, editor_layer);
    ImGui::TreePop();
  }

  if (target.demo_type != DynamicStrandsDemo::DemoType::Empty) {
    ImGui::Text("Demo started");
    ImGui::Text(("Simulated time: " + std::to_string(target.simulated_time)).c_str());
    ImGui::Text(("Target simulation time: " + std::to_string(target.target_simulation_time)).c_str());
    if (ImGui::Button("Force stop")) {
      target.demo_type = DynamicStrandsDemo::DemoType::Empty;
      target.demo_status = DynamicStrandsDemo::DemoStatus::Idle;
    }
    return false;
  }
  bool changed = false;
  const auto owner = target.GetOwner();
  const auto scene = target.GetScene();
  const auto dts = scene->GetOrSetPrivateComponent<DynamicTreeStrands>(owner).lock();
  if (ImGui::TreeNode("Initialize Parameters")) {
    InspectorRegistry::GetInstance().InspectValue(inspector_context, dts->initialize_parameters);
    ImGui::TreePop();
  }
  ImGui::DragFloat("Target simulation time", &target.target_simulation_time, 0.1f, 0.1f, 100.f);
  ImGui::DragFloat("Target factor 0", &target.target_factor0, 0.01f, 0.0f, 1.f);
  ImGui::DragFloat("Target factor 1", &target.target_factor1, 0.01f, 0.0f, 1.f);
  if (ImGui::TreeNode("Rod settings")) {
    InspectSettings(target.log_experiment_setup_settings, editor_layer);
    ImGui::TreePop();
  }
  if (ImGui::TreeNode("Board settings")) {
    InspectSettings(target.board_experiment_setup_settings, editor_layer);
    ImGui::TreePop();
  }

  if (ImGui::Button("Log break [Diffuse]")) {
    ResetEnvironment(target, editor_layer);
    camera_pose.SetPosition(glm::vec3(0.5, 0.7, 1));
    camera_pose.SetEulerRotation(glm::radians(glm::vec3(10, 0, 0)));
    target.demo_type = DynamicStrandsDemo::DemoType::LogBreak;
    target.demo_status = DynamicStrandsDemo::DemoStatus::Simulation;
    target.log_experiment_setup_settings.center_damage = 0.95f;
    target.log_experiment_setup_settings.center_distance_offset = 0.01f;
    target.log_experiment_setup_settings.center_damage_transition = 0.02f;
    target.log_experiment_setup_settings.fungus_test = false;

    // TODO
    dts->initialize_parameters.strength_graph.SetBundleStrength({500.f, 50.f});
    dts->initialize_parameters.strength_graph.SetConnectivityStrength({100.f, 100.f});

    target.log_experiment_setup_settings.t_cut = true;
    target.log_experiment_setup_settings.t_cut_width = 0.f;
    target.target_factor0 = 2.f;
    target.target_factor1 = 1.f;

    dts->LogExperimentSetup(target.log_experiment_setup_settings);
    editor_layer->SetSceneCameraRotation(camera_pose.GetRotation());
    editor_layer->SetSceneCameraPosition(camera_pose.GetPosition());
  }
  if (ImGui::Button("Fungus [Competition-Equal]")) {
    ResetEnvironment(target, editor_layer);
    camera_pose.SetPosition(glm::vec3(0.25, 0.9, 0.6));
    camera_pose.SetEulerRotation(glm::radians(glm::vec3(-30, 0, 0)));
    target.target_factor0 = 1.5f;
    target.demo_type = DynamicStrandsDemo::DemoType::Fungus;
    target.demo_status = DynamicStrandsDemo::DemoStatus::Simulation;
    target.log_experiment_setup_settings.rod_segment_count = 10;
    target.log_experiment_setup_settings.rod_segment_count = 20;
    target.log_experiment_setup_settings.rod_size = 3200;
    target.log_experiment_setup_settings.segment_length = 0.025f;
    target.log_experiment_setup_settings.fungus_test = true;
    target.log_experiment_setup_settings.competition_setting = true;
    target.log_experiment_setup_settings.cube_pattern = true;
    target.log_experiment_setup_settings.internal_pattern = false;
    target.log_experiment_setup_settings.right_pivot_type =
        static_cast<unsigned>(DynamicTreeStrands::PivotType::Partial_Transform);
    target.log_experiment_setup_settings.left_pivot_type =
        static_cast<unsigned>(DynamicTreeStrands::PivotType::Partial_Transform);
    target.physics_parameters.enable_fungus = true;
    target.physics_parameters.enable_segment_collision = false;
    target.physics_parameters.bo = 0.0f;
    target.physics_parameters.be = 0.0f;
    target.physics_parameters.HL_threshold = -1.0f;
    target.physics_parameters.HC_threshold = -1.0f;  // Disable breakage for competition
    target.physics_parameters.k = 0.4f;
    target.physics_parameters.ycb = 2.0f;
    target.physics_parameters.kc = 0.1f;
    target.physics_parameters.ycw = 0.f;

    // physics_parameters.HL_threshold = 0.5f;
    // physics_parameters.HC_threshold = 0.5f;

    target.physics_parameters.bd_offset = 100.f;  // disable fast internal decay

    target.physics_parameters.matrixAw = glm::mat3(1.f, 0.0f, 0.0f, 0.0f, 0.1f, 0.0f, 0.0f, 0.0f, 10.0f);
    target.physics_parameters.matrixAb = glm::mat3(1.f, 0.0f, 0.0f, 0.0f, 0.1f, 0.0f, 0.0f, 0.0f, 10.0f);  //(R,T,L)
    dts->initialize_parameters.max_segment_length = 0.01f;
    dts->initialize_parameters.min_segment_length = 0.005f;

    dts->LogExperimentSetup(target.log_experiment_setup_settings);
    editor_layer->SetSceneCameraRotation(camera_pose.GetRotation());
    editor_layer->SetSceneCameraPosition(camera_pose.GetPosition());
  }

  if (ImGui::Button("Fungus [Competition-Brown]")) {
    ResetEnvironment(target, editor_layer);
    camera_pose.SetPosition(glm::vec3(0.25, 0.9, 0.6));
    camera_pose.SetEulerRotation(glm::radians(glm::vec3(-30, 0, 0)));
    target.target_factor0 = 1.5f;
    target.demo_type = DynamicStrandsDemo::DemoType::Fungus;
    target.demo_status = DynamicStrandsDemo::DemoStatus::Simulation;
    target.log_experiment_setup_settings.rod_segment_count = 10;
    target.log_experiment_setup_settings.rod_segment_count = 20;
    target.log_experiment_setup_settings.rod_size = 3200;
    target.log_experiment_setup_settings.segment_length = 0.025f;
    target.log_experiment_setup_settings.fungus_test = true;
    target.log_experiment_setup_settings.competition_setting = true;
    target.log_experiment_setup_settings.cube_pattern = true;
    target.log_experiment_setup_settings.internal_pattern = false;
    target.log_experiment_setup_settings.right_pivot_type =
        static_cast<unsigned>(DynamicTreeStrands::PivotType::Partial_Transform);
    target.log_experiment_setup_settings.left_pivot_type =
        static_cast<unsigned>(DynamicTreeStrands::PivotType::Partial_Transform);
    target.physics_parameters.enable_fungus = true;
    target.physics_parameters.enable_segment_collision = false;
    target.physics_parameters.bo = 0.0f;
    target.physics_parameters.be = 0.0f;
    target.physics_parameters.HL_threshold = -1.0f;
    target.physics_parameters.HC_threshold = -1.0f;  // Disable breakage for competition
    target.physics_parameters.k = 0.4f;
    target.physics_parameters.ycb = 2.0f;
    target.physics_parameters.kc = 0.1f;
    target.physics_parameters.ycw = 0.f;

    target.physics_parameters.bw = 24.0f;
    target.physics_parameters.cpw = 0.0f;
    // physics_parameters.HL_threshold = 0.3f;
    // physics_parameters.HC_threshold = 0.3f;  // Brown rot beat white rot

    target.physics_parameters.bd_offset = 100.f;  // disable fast internal decay

    target.physics_parameters.matrixAw = glm::mat3(1.f, 0.0f, 0.0f, 0.0f, 0.1f, 0.0f, 0.0f, 0.0f, 10.0f);
    target.physics_parameters.matrixAb = glm::mat3(1.f, 0.0f, 0.0f, 0.0f, 0.1f, 0.0f, 0.0f, 0.0f, 10.0f);  //(R,T,L)
    dts->initialize_parameters.max_segment_length = 0.01f;
    dts->initialize_parameters.min_segment_length = 0.005f;

    dts->LogExperimentSetup(target.log_experiment_setup_settings);
    editor_layer->SetSceneCameraRotation(camera_pose.GetRotation());
    editor_layer->SetSceneCameraPosition(camera_pose.GetPosition());
  }
  if (ImGui::Button("Fungus [Competition-White]")) {
    ResetEnvironment(target, editor_layer);
    camera_pose.SetPosition(glm::vec3(0.25, 0.9, 0.6));
    camera_pose.SetEulerRotation(glm::radians(glm::vec3(-30, 0, 0)));
    target.target_factor0 = 1.5f;
    target.demo_type = DynamicStrandsDemo::DemoType::Fungus;
    target.demo_status = DynamicStrandsDemo::DemoStatus::Simulation;
    target.log_experiment_setup_settings.rod_segment_count = 10;
    target.log_experiment_setup_settings.rod_segment_count = 20;
    target.log_experiment_setup_settings.rod_size = 3200;
    target.log_experiment_setup_settings.segment_length = 0.025f;
    target.log_experiment_setup_settings.fungus_test = true;
    target.log_experiment_setup_settings.competition_setting = true;
    target.log_experiment_setup_settings.cube_pattern = true;
    target.log_experiment_setup_settings.internal_pattern = false;
    target.log_experiment_setup_settings.right_pivot_type =
        static_cast<unsigned>(DynamicTreeStrands::PivotType::Partial_Transform);
    target.log_experiment_setup_settings.left_pivot_type =
        static_cast<unsigned>(DynamicTreeStrands::PivotType::Partial_Transform);
    target.physics_parameters.enable_fungus = true;
    target.physics_parameters.enable_segment_collision = false;
    target.physics_parameters.bo = 0.0f;
    target.physics_parameters.be = 0.0f;
    target.physics_parameters.HL_threshold = -1.0f;
    target.physics_parameters.HC_threshold = -1.0f;  // Disable breakage for competition
    target.physics_parameters.k = 0.4f;
    target.physics_parameters.ycb = 2.0f;
    target.physics_parameters.kc = 0.1f;
    target.physics_parameters.ycw = 0.f;

    target.physics_parameters.bb = 24.0f;
    target.physics_parameters.cpb = 0.0f;
    // physics_parameters.HL_threshold = 0.2f;
    // physics_parameters.HC_threshold = 0.2f;  // White rot beat brown rot

    target.physics_parameters.bd_offset = 100.f;  // disable fast internal decay

    target.physics_parameters.matrixAw = glm::mat3(1.f, 0.0f, 0.0f, 0.0f, 0.1f, 0.0f, 0.0f, 0.0f, 10.0f);
    target.physics_parameters.matrixAb = glm::mat3(1.f, 0.0f, 0.0f, 0.0f, 0.1f, 0.0f, 0.0f, 0.0f, 10.0f);  //(R,T,L)
    dts->initialize_parameters.max_segment_length = 0.01f;
    dts->initialize_parameters.min_segment_length = 0.005f;

    dts->LogExperimentSetup(target.log_experiment_setup_settings);
    editor_layer->SetSceneCameraRotation(camera_pose.GetRotation());
    editor_layer->SetSceneCameraPosition(camera_pose.GetPosition());
  }

  if (ImGui::Button("Fungus [Internal]")) {
    ResetEnvironment(target, editor_layer);
    camera_pose.SetPosition(glm::vec3(-0.3, 1.3, 0.2));
    camera_pose.SetEulerRotation(glm::radians(glm::vec3(-30, -60, 0)));
    target.target_factor0 = 1.5f;
    target.demo_type = DynamicStrandsDemo::DemoType::Fungus;
    target.demo_status = DynamicStrandsDemo::DemoStatus::Simulation;
    target.log_experiment_setup_settings.rod_segment_count = 10;
    target.log_experiment_setup_settings.rod_segment_count = 20;
    target.log_experiment_setup_settings.rod_size = 3200;
    target.log_experiment_setup_settings.segment_length = 0.025f;
    target.log_experiment_setup_settings.fungus_test = true;
    target.log_experiment_setup_settings.internal_pattern = true;
    target.log_experiment_setup_settings.cube_pattern = false;
    target.physics_parameters.enable_fungus = false;
    target.physics_parameters.pivot_ring_radius = 0.05f;

    target.log_experiment_setup_settings.right_pivot_type =
        static_cast<unsigned>(DynamicTreeStrands::PivotType::Partial_Transform);
    target.log_experiment_setup_settings.left_pivot_type =
        static_cast<unsigned>(DynamicTreeStrands::PivotType::Partial_Transform);

    target.physics_parameters.gravity = glm::vec3(0.f, 0.f, 0.f);
    dts->initialize_parameters.max_segment_length = 0.01f;
    dts->initialize_parameters.min_segment_length = 0.005f;

    dts->LogExperimentSetup(target.log_experiment_setup_settings);
    editor_layer->SetSceneCameraRotation(camera_pose.GetRotation());
    editor_layer->SetSceneCameraPosition(camera_pose.GetPosition());
  }

  if (ImGui::Button("Fungus [Cubical]")) {
    ResetEnvironment(target, editor_layer);
    camera_pose.SetPosition(glm::vec3(-0.3, 1.3, 0.2));
    camera_pose.SetEulerRotation(glm::radians(glm::vec3(-30, -60, 0)));
    target.target_factor0 = 1.5f;
    target.demo_type = DynamicStrandsDemo::DemoType::Fungus;
    target.demo_status = DynamicStrandsDemo::DemoStatus::Simulation;
    target.log_experiment_setup_settings.rod_segment_count = 10;
    target.log_experiment_setup_settings.rod_segment_count = 20;
    target.log_experiment_setup_settings.rod_size = 3200;
    target.log_experiment_setup_settings.segment_length = 0.025f;
    target.log_experiment_setup_settings.fungus_test = true;
    target.log_experiment_setup_settings.cube_pattern = true;
    target.log_experiment_setup_settings.internal_pattern = false;
    target.log_experiment_setup_settings.right_pivot_type =
        static_cast<unsigned>(DynamicTreeStrands::PivotType::Partial_Transform);
    target.log_experiment_setup_settings.left_pivot_type =
        static_cast<unsigned>(DynamicTreeStrands::PivotType::Partial_Transform);
    target.physics_parameters.enable_fungus = true;
    target.physics_parameters.enable_segment_collision = true;
    target.physics_parameters.bo = 0.0f;
    target.physics_parameters.be = 0.0f;

    target.physics_parameters.ycw = 0.1f;
    target.physics_parameters.ycb = 2.0f;
    target.physics_parameters.aw = 3.5f;
    target.physics_parameters.bw = 4.0f;
    target.physics_parameters.k = 0.2f;

    target.physics_parameters.bd_offset = 100.f;  // disable fast internal decay

    // physics_parameters.matrixAw = glm::mat3(0.5f, 0.0f, 0.0f, 0.0f, 0.5f, 0.0f, 0.0f, 0.0f, 0.5f);
    // physics_parameters.matrixAb = glm::mat3(50.f, 0.0f, 0.0f, 0.0f, 0.2f, 0.0f, 0.0f, 0.0f, 0.2f);
    dts->initialize_parameters.max_segment_length = 0.01f;
    dts->initialize_parameters.min_segment_length = 0.005f;

    dts->LogExperimentSetup(target.log_experiment_setup_settings);
    editor_layer->SetSceneCameraRotation(camera_pose.GetRotation());
    editor_layer->SetSceneCameraPosition(camera_pose.GetPosition());
  }

  if (ImGui::Button("Fungus [Non-Cubical]")) {
    ResetEnvironment(target, editor_layer);
    camera_pose.SetPosition(glm::vec3(-0.3, 1.3, 0.2));
    camera_pose.SetEulerRotation(glm::radians(glm::vec3(-30, -60, 0)));
    target.target_factor0 = 1.5f;
    target.demo_type = DynamicStrandsDemo::DemoType::Fungus;
    target.demo_status = DynamicStrandsDemo::DemoStatus::Simulation;
    target.log_experiment_setup_settings.rod_segment_count = 10;
    target.log_experiment_setup_settings.rod_segment_count = 20;
    target.log_experiment_setup_settings.rod_size = 3200;
    target.log_experiment_setup_settings.segment_length = 0.025f;
    target.log_experiment_setup_settings.fungus_test = true;
    target.log_experiment_setup_settings.cube_pattern = false;
    target.log_experiment_setup_settings.internal_pattern = false;
    target.physics_parameters.enable_fungus = true;
    target.physics_parameters.enable_segment_collision = true;
    target.physics_parameters.bo = 0.0f;
    target.physics_parameters.be = 0.0f;
    target.physics_parameters.ycb = 0.1f;
    target.physics_parameters.ylw = 3.0f;
    target.physics_parameters.ab = 3.5f;
    target.physics_parameters.k = 0.2f;

    target.physics_parameters.bd_offset = 100.f;  // disable fast internal decay

    // physics_parameters.matrixAw = glm::mat3(0.5f, 0.0f, 0.0f, 0.0f, 0.5f, 0.0f, 0.0f, 0.0f, 0.5f);
    // physics_parameters.matrixAb = glm::mat3(0.5f, 0.0f, 0.0f, 0.0f, 0.5f, 0.0f, 0.0f, 0.0f, 0.5f);
    // physics_parameters.pl = 0.01f; //anisotropic case
    dts->initialize_parameters.max_segment_length = 0.01f;
    dts->initialize_parameters.min_segment_length = 0.005f;

    dts->LogExperimentSetup(target.log_experiment_setup_settings);
    editor_layer->SetSceneCameraRotation(camera_pose.GetRotation());
    editor_layer->SetSceneCameraPosition(camera_pose.GetPosition());
  }

  if (ImGui::Button("Board Fungus")) {
    ResetEnvironment(target, editor_layer);
    camera_pose.SetPosition(glm::vec3(-0.5, 2.2, 1));
    camera_pose.SetEulerRotation(glm::radians(glm::vec3(-40, -45, 0)));
    target.demo_type = DynamicStrandsDemo::DemoType::Fungus;
    target.demo_status = DynamicStrandsDemo::DemoStatus::Simulation;

    dts->initialize_parameters.strength_graph.SetShearStretchStrength({100.f, 100.f});
    dts->initialize_parameters.strength_graph.SetBendingStrength({100.f, 100.f});
    dts->initialize_parameters.strength_graph.SetTwistingStrength({100.f, 100.f});
    dts->initialize_parameters.strength_graph.SetBundleStrength({100.f, 100.f});
    dts->initialize_parameters.strength_graph.SetConnectivityStrength({100.f, 100.f});

    dts->initialize_parameters.max_segment_length = 0.03f;
    dts->initialize_parameters.min_segment_length = 0.015f;

    target.physics_parameters.enable_fungus = true;
    target.physics_parameters.treespace = false;
    target.physics_parameters.matrixAw = glm::mat3(200.0f, 0.0f, 0.0f, 0.0f, 200.0f, 0.0f, 0.0f, 0.0f, 200.0f);
    // physics_parameters.matrixAb = glm::mat3(2.f, 0.0f, 0.0f, 0.0f, 1000.f, 0.0f, 0.0f, 0.0f, 0.2f);  // slow y
    target.physics_parameters.matrixAb = glm::mat3(1000.f, 0.0f, 0.0f, 0.0f, 0.2f, 0.0f, 0.0f, 0.0f, 0.2f);  // slow x
    target.physics_parameters.brb = 0.75f;
    target.physics_parameters.pc = 0.05f;
    target.physics_parameters.k = 1.0f;

    target.physics_parameters.bd_offset = 100.f;  // disable boundary decay

    target.board_experiment_setup_settings.center_damage = 0.7f;
    target.board_experiment_setup_settings.rod_dimension = {160, 10, 20};
    target.board_experiment_setup_settings.fungus_test = true;

    dts->BoardExperimentSetup(target.board_experiment_setup_settings);
    editor_layer->SetSceneCameraRotation(camera_pose.GetRotation());
    editor_layer->SetSceneCameraPosition(camera_pose.GetPosition());
  }
  if (ImGui::Button("Log break [Clean]")) {
    ResetEnvironment(target, editor_layer);
    camera_pose.SetPosition(glm::vec3(0.5, 0.7, 1));
    camera_pose.SetEulerRotation(glm::radians(glm::vec3(10, 0, 0)));
    target.demo_type = DynamicStrandsDemo::DemoType::LogBreak;
    target.demo_status = DynamicStrandsDemo::DemoStatus::Simulation;
    target.log_experiment_setup_settings.center_damage = 1.f;
    target.log_experiment_setup_settings.center_distance_offset = 0.01f;
    target.log_experiment_setup_settings.center_damage_transition = 0.02f;
    target.log_experiment_setup_settings.fungus_test = false;

    dts->initialize_parameters.strength_graph.SetBundleStrength({500.f, 50.f});
    dts->initialize_parameters.strength_graph.SetConnectivityStrength({250.f, 250.f});

    target.log_experiment_setup_settings.lock_upper = true;
    target.log_experiment_setup_settings.t_cut = true;
    target.target_factor0 = 0.3f;
    target.target_factor1 = 1.f;

    target.physics_parameters.enable_segment_compression_disconnection = false;
    dts->LogExperimentSetup(target.log_experiment_setup_settings);
    editor_layer->SetSceneCameraRotation(camera_pose.GetRotation());
    editor_layer->SetSceneCameraPosition(camera_pose.GetPosition());
  }
  if (ImGui::Button("Log break [Transverse buckling]")) {
    ResetEnvironment(target, editor_layer);
    camera_pose.SetPosition(glm::vec3(0.5, 0.7, 1));
    camera_pose.SetEulerRotation(glm::radians(glm::vec3(10, 0, 0)));
    target.demo_type = DynamicStrandsDemo::DemoType::LogBreak;
    target.demo_status = DynamicStrandsDemo::DemoStatus::Simulation;
    target.log_experiment_setup_settings.center_damage = 0.95f;
    target.log_experiment_setup_settings.center_distance_offset = 0.01f;
    target.log_experiment_setup_settings.center_damage_transition = 0.02f;
    target.log_experiment_setup_settings.fungus_test = false;

    dts->initialize_parameters.strength_graph.SetBundleStrength({500.f, 50.f});
    dts->initialize_parameters.strength_graph.SetConnectivityStrength({250.f, 250.f});

    // target_factor0 = 0.3f;
    target.target_factor1 = 1.f;
    target.physics_parameters.enable_positional_breaking = false;
    target.physics_parameters.enable_segment_disconnection = false;
    dts->LogExperimentSetup(target.log_experiment_setup_settings);
    editor_layer->SetSceneCameraRotation(camera_pose.GetRotation());
    editor_layer->SetSceneCameraPosition(camera_pose.GetPosition());
  }

  if (ImGui::Button("Board break [Low]")) {
    ResetEnvironment(target, editor_layer);
    camera_pose.SetPosition(glm::vec3(-0.5, 1.5, 1));
    camera_pose.SetEulerRotation(glm::radians(glm::vec3(-20, -45, 0)));
    target.demo_type = DynamicStrandsDemo::DemoType::BoardBreak;
    target.demo_status = DynamicStrandsDemo::DemoStatus::Simulation;
    target.board_experiment_setup_settings.center_damage = 0.f;

    dts->initialize_parameters.strength_graph.SetBundleStrength({500.f, 50.f});
    dts->initialize_parameters.strength_graph.SetConnectivityStrength({250.f, 250.f});

    dts->BoardExperimentSetup(target.board_experiment_setup_settings);
    editor_layer->SetSceneCameraRotation(camera_pose.GetRotation());
    editor_layer->SetSceneCameraPosition(camera_pose.GetPosition());
  }
  if (ImGui::Button("Board break [Medium]")) {
    ResetEnvironment(target, editor_layer);
    camera_pose.SetPosition(glm::vec3(-0.5, 1.5, 1));
    camera_pose.SetEulerRotation(glm::radians(glm::vec3(-20, -45, 0)));
    target.demo_type = DynamicStrandsDemo::DemoType::BoardBreak;
    target.demo_status = DynamicStrandsDemo::DemoStatus::Simulation;
    target.board_experiment_setup_settings.center_damage = 0.f;

    dts->initialize_parameters.strength_graph.SetBundleStrength({100.f, 100.f});
    dts->initialize_parameters.strength_graph.SetConnectivityStrength({250.f, 250.f});

    dts->BoardExperimentSetup(target.board_experiment_setup_settings);
    editor_layer->SetSceneCameraRotation(camera_pose.GetRotation());
    editor_layer->SetSceneCameraPosition(camera_pose.GetPosition());
  }
  if (ImGui::Button("Board break [High]")) {
    ResetEnvironment(target, editor_layer);
    camera_pose.SetPosition(glm::vec3(-0.5, 1.5, 1));
    camera_pose.SetEulerRotation(glm::radians(glm::vec3(-20, -45, 0)));
    target.demo_type = DynamicStrandsDemo::DemoType::BoardBreak;
    target.demo_status = DynamicStrandsDemo::DemoStatus::Simulation;

    dts->initialize_parameters.strength_graph.SetBundleStrength({175.f, 175.f});
    dts->initialize_parameters.strength_graph.SetConnectivityStrength({250.f, 250.f});

    dts->BoardExperimentSetup(target.board_experiment_setup_settings);
    editor_layer->SetSceneCameraRotation(camera_pose.GetRotation());
    editor_layer->SetSceneCameraPosition(camera_pose.GetPosition());
  }

  if (ImGui::Button("Twisting break")) {
    ResetEnvironment(target, editor_layer);
    camera_pose.SetPosition(glm::vec3(-0.5, 2.2, 1));
    camera_pose.SetEulerRotation(glm::radians(glm::vec3(-40, -45, 0)));
    target.demo_type = DynamicStrandsDemo::DemoType::TwistingBreak;
    target.demo_status = DynamicStrandsDemo::DemoStatus::Simulation;
    target.target_factor0 = 0.f;
    dts->initialize_parameters.strength_graph.SetShearStretchStrength({250.f, 250.f});
    dts->initialize_parameters.strength_graph.SetBendingStrength({250.f, 250.f});
    dts->initialize_parameters.strength_graph.SetTwistingStrength({250.f, 250.f});
    dts->initialize_parameters.strength_graph.SetBundleStrength({250.f, 250.f});
    dts->initialize_parameters.strength_graph.SetConnectivityStrength({250.f, 250.f});

    // auto& noise = dts->initialize_parameters.damage_graph.noise_descriptors.emplace_back();
    // noise.type = static_cast<unsigned>(NoiseType::Perlin);

    target.board_experiment_setup_settings.rod_dimension = {160, 10, 20};
    dts->BoardExperimentSetup(target.board_experiment_setup_settings);
    editor_layer->SetSceneCameraRotation(camera_pose.GetRotation());
    editor_layer->SetSceneCameraPosition(camera_pose.GetPosition());
  }
  if (ImGui::Button("Bending break")) {
    ResetEnvironment(target, editor_layer);
    camera_pose.SetPosition(glm::vec3(-0.5, 2.2, 1));
    camera_pose.SetEulerRotation(glm::radians(glm::vec3(-40, -45, 0)));
    target.target_factor0 = 0.f;
    target.demo_type = DynamicStrandsDemo::DemoType::BendingBreak;
    target.demo_status = DynamicStrandsDemo::DemoStatus::Simulation;

    dts->initialize_parameters.strength_graph.SetShearStretchStrength({250.f, 250.f});
    dts->initialize_parameters.strength_graph.SetBendingStrength({250.f, 250.f});
    dts->initialize_parameters.strength_graph.SetTwistingStrength({250.f, 250.f});
    dts->initialize_parameters.strength_graph.SetBundleStrength({250.f, 250.f});
    dts->initialize_parameters.strength_graph.SetConnectivityStrength({250.f, 250.f});

    // auto& noise = dts->initialize_parameters.damage_graph.noise_descriptors.emplace_back();
    // noise.type = static_cast<unsigned>(NoiseType::Perlin);

    target.board_experiment_setup_settings.left_pivot_type =
        static_cast<unsigned>(DynamicTreeStrands::PivotType::Transform);
    target.board_experiment_setup_settings.right_pivot_type =
        static_cast<unsigned>(DynamicTreeStrands::PivotType::Transform);
    target.board_experiment_setup_settings.rod_dimension = {160, 10, 20};
    dts->BoardExperimentSetup(target.board_experiment_setup_settings);
    editor_layer->SetSceneCameraRotation(camera_pose.GetRotation());
    editor_layer->SetSceneCameraPosition(camera_pose.GetPosition());
  }
  if (ImGui::Button("Shearing break")) {
    ResetEnvironment(target, editor_layer);
    camera_pose.SetPosition(glm::vec3(-0.5, 2.2, 1));
    camera_pose.SetEulerRotation(glm::radians(glm::vec3(-40, -45, 0)));
    target.target_factor0 = 0.f;
    target.demo_type = DynamicStrandsDemo::DemoType::ShearingBreak;
    target.demo_status = DynamicStrandsDemo::DemoStatus::Simulation;

    dts->initialize_parameters.strength_graph.SetShearStretchStrength({250.f, 250.f});
    dts->initialize_parameters.strength_graph.SetBendingStrength({250.f, 250.f});
    dts->initialize_parameters.strength_graph.SetTwistingStrength({250.f, 250.f});
    dts->initialize_parameters.strength_graph.SetBundleStrength({250.f, 250.f});
    dts->initialize_parameters.strength_graph.SetConnectivityStrength({250.f, 250.f});
    // auto& noise = dts->initialize_parameters.damage_graph.noise_descriptors.emplace_back();
    // noise.type = static_cast<unsigned>(NoiseType::Perlin);

    target.board_experiment_setup_settings.rod_dimension = {160, 10, 20};
    dts->BoardExperimentSetup(target.board_experiment_setup_settings);
    editor_layer->SetSceneCameraRotation(camera_pose.GetRotation());
    editor_layer->SetSceneCameraPosition(camera_pose.GetPosition());
  }
  if (ImGui::Button("Stretching break")) {
    ResetEnvironment(target, editor_layer);
    camera_pose.SetPosition(glm::vec3(-0.5, 2.2, 1));
    camera_pose.SetEulerRotation(glm::radians(glm::vec3(-40, -45, 0)));
    target.target_factor0 = 0.f;
    target.demo_type = DynamicStrandsDemo::DemoType::StretchingBreak;
    target.demo_status = DynamicStrandsDemo::DemoStatus::Simulation;

    dts->initialize_parameters.strength_graph.SetShearStretchStrength({250.f, 250.f});
    dts->initialize_parameters.strength_graph.SetBendingStrength({250.f, 250.f});
    dts->initialize_parameters.strength_graph.SetTwistingStrength({250.f, 250.f});
    dts->initialize_parameters.strength_graph.SetBundleStrength({250.f, 250.f});
    dts->initialize_parameters.strength_graph.SetConnectivityStrength({250.f, 250.f});
    // auto& noise = dts->initialize_parameters.damage_graph.noise_descriptors.emplace_back();
    // noise.type = static_cast<unsigned>(NoiseType::Perlin);

    target.board_experiment_setup_settings.rod_dimension = {160, 10, 20};
    dts->BoardExperimentSetup(target.board_experiment_setup_settings);
    editor_layer->SetSceneCameraRotation(camera_pose.GetRotation());
    editor_layer->SetSceneCameraPosition(camera_pose.GetPosition());
  }

  if (ImGui::Button("Sap/Heart Increase")) {
    ResetEnvironment(target, editor_layer);
    camera_pose.SetPosition(glm::vec3(-0.3, 1.3, 0.2));
    camera_pose.SetEulerRotation(glm::radians(glm::vec3(-30, -60, 0)));
    target.target_factor0 = 1.5f;
    target.demo_type = DynamicStrandsDemo::DemoType::SapHeart;
    target.demo_status = DynamicStrandsDemo::DemoStatus::Simulation;
    target.log_experiment_setup_settings.rod_segment_count = 10;
    target.log_experiment_setup_settings.rod_size = 3200;
    target.log_experiment_setup_settings.fungus_test = false;

    // auto& noise = dts->initialize_parameters.damage_graph.noise_descriptors.emplace_back();
    // noise.type = static_cast<unsigned>(NoiseType::Perlin);

    dts->initialize_parameters.strength_graph.SetShearStretchStrength({750.f, 50.f});
    dts->initialize_parameters.strength_graph.SetBendingStrength({750.f, 50.f});
    dts->initialize_parameters.strength_graph.SetTwistingStrength({750.f, 50.f});
    dts->initialize_parameters.strength_graph.SetBundleStrength({750.f, 50.f});
    dts->initialize_parameters.strength_graph.SetConnectivityStrength({750.f, 50.f});
    dts->LogExperimentSetup(target.log_experiment_setup_settings);
    editor_layer->SetSceneCameraRotation(camera_pose.GetRotation());
    editor_layer->SetSceneCameraPosition(camera_pose.GetPosition());
  }
  if (ImGui::Button("Sap/Heart Equal")) {
    ResetEnvironment(target, editor_layer);
    camera_pose.SetPosition(glm::vec3(-0.3, 1.3, 0.2));
    camera_pose.SetEulerRotation(glm::radians(glm::vec3(-30, -60, 0)));
    target.target_factor0 = 1.5f;
    target.demo_type = DynamicStrandsDemo::DemoType::SapHeart;
    target.demo_status = DynamicStrandsDemo::DemoStatus::Simulation;
    target.log_experiment_setup_settings.rod_segment_count = 10;
    target.log_experiment_setup_settings.rod_size = 3200;
    target.log_experiment_setup_settings.fungus_test = false;

    // auto& noise = dts->initialize_parameters.damage_graph.noise_descriptors.emplace_back();
    // noise.type = static_cast<unsigned>(NoiseType::Perlin);
    dts->initialize_parameters.strength_graph.SetShearStretchStrength({500.f, 500.f});
    dts->initialize_parameters.strength_graph.SetBendingStrength({500.f, 500.f});
    dts->initialize_parameters.strength_graph.SetTwistingStrength({500.f, 500.f});
    dts->initialize_parameters.strength_graph.SetBundleStrength({500.f, 500.f});
    dts->initialize_parameters.strength_graph.SetConnectivityStrength({500.f, 500.f});
    dts->LogExperimentSetup(target.log_experiment_setup_settings);
    editor_layer->SetSceneCameraRotation(camera_pose.GetRotation());
    editor_layer->SetSceneCameraPosition(camera_pose.GetPosition());
  }
  if (ImGui::Button("Sap/Heart Decrease")) {
    ResetEnvironment(target, editor_layer);
    camera_pose.SetPosition(glm::vec3(-0.3, 1.3, 0.2));
    camera_pose.SetEulerRotation(glm::radians(glm::vec3(-30, -60, 0)));
    target.target_factor0 = 1.5f;
    target.demo_type = DynamicStrandsDemo::DemoType::SapHeart;
    target.demo_status = DynamicStrandsDemo::DemoStatus::Simulation;
    target.log_experiment_setup_settings.rod_segment_count = 10;
    target.log_experiment_setup_settings.rod_size = 3200;
    target.log_experiment_setup_settings.fungus_test = false;

    // auto& noise = dts->initialize_parameters.damage_graph.noise_descriptors.emplace_back();
    // noise.type = static_cast<unsigned>(NoiseType::Perlin);
    dts->initialize_parameters.strength_graph.SetShearStretchStrength({50.f, 750.f});
    dts->initialize_parameters.strength_graph.SetBendingStrength({50.f, 750.f});
    dts->initialize_parameters.strength_graph.SetTwistingStrength({50.f, 750.f});
    dts->initialize_parameters.strength_graph.SetBundleStrength({50.f, 750.f});
    dts->initialize_parameters.strength_graph.SetConnectivityStrength({50.f, 750.f});
    dts->LogExperimentSetup(target.log_experiment_setup_settings);
    editor_layer->SetSceneCameraRotation(camera_pose.GetRotation());
    editor_layer->SetSceneCameraPosition(camera_pose.GetPosition());
  }

  if (ImGui::Button("Short rod sphere Collision")) {
    ResetEnvironment(target, editor_layer);
    camera_pose.SetPosition(glm::vec3(-0.5, 2.2, 1));
    camera_pose.SetEulerRotation(glm::radians(glm::vec3(-40, -45, 0)));
    target.target_factor0 = 0.f;
    target.demo_type = DynamicStrandsDemo::DemoType::BoardCollision;
    target.demo_status = DynamicStrandsDemo::DemoStatus::Simulation;

    dts->initialize_parameters.strength_graph.SetShearStretchStrength({100.f, 100.f});
    dts->initialize_parameters.strength_graph.SetBendingStrength({100.f, 100.f});
    dts->initialize_parameters.strength_graph.SetTwistingStrength({100.f, 100.f});
    dts->initialize_parameters.strength_graph.SetBundleStrength({100.f, 100.f});
    dts->initialize_parameters.strength_graph.SetConnectivityStrength({100.f, 100.f});

    dts->initialize_parameters.max_segment_length = 0.03f;
    dts->initialize_parameters.min_segment_length = 0.015f;

    // auto& noise = dts->initialize_parameters.damage_graph.noise_descriptors.emplace_back();
    // noise.type = static_cast<unsigned>(NoiseType::Perlin);
    // noise.multiplier = 0.95f;
    // noise.shift = glm::vec3(1000.f);

    target.board_experiment_setup_settings.center_damage = 0.7f;
    target.board_experiment_setup_settings.rod_dimension = {160, 10, 20};

    const auto sphere_entity = scene->CreateEntity("Sphere");
    scene->GetOrSetPrivateComponent<DsSphereCollider>(sphere_entity);
    const auto mmr = scene->GetOrSetPrivateComponent<MeshRenderer>(sphere_entity).lock();
    mmr->mesh = Resources::GetInstance().GetPrimitives().sphere;
    mmr->material = AssetManager::CreateTemporaryAsset<Material>();
    target.object_initial_pose.SetPosition(glm::vec3(.5f, 1.3f, 0));
    target.object_initial_pose.SetScale(glm::vec3(0.3f));
    scene->SetDataComponent(sphere_entity, target.object_initial_pose);

    const auto temp_entity = target.temp_entity1_ref.Get();
    if (scene->IsEntityValid(temp_entity)) {
      scene->DeleteEntity(temp_entity);
    }
    target.temp_entity1_ref = sphere_entity;
    dts->BoardExperimentSetup(target.board_experiment_setup_settings);
    editor_layer->SetSceneCameraRotation(camera_pose.GetRotation());
    editor_layer->SetSceneCameraPosition(camera_pose.GetPosition());
  }
  if (ImGui::Button("Long rod sphere Collision")) {
    ResetEnvironment(target, editor_layer);
    camera_pose.SetPosition(glm::vec3(-0.5, 2.2, 1));
    camera_pose.SetEulerRotation(glm::radians(glm::vec3(-40, -45, 0)));
    target.target_factor0 = 0.f;
    target.demo_type = DynamicStrandsDemo::DemoType::BoardCollision;
    target.demo_status = DynamicStrandsDemo::DemoStatus::Simulation;

    dts->initialize_parameters.strength_graph.SetShearStretchStrength({100.f, 100.f});
    dts->initialize_parameters.strength_graph.SetBendingStrength({100.f, 100.f});
    dts->initialize_parameters.strength_graph.SetTwistingStrength({100.f, 100.f});
    dts->initialize_parameters.strength_graph.SetBundleStrength({100.f, 100.f});
    dts->initialize_parameters.strength_graph.SetConnectivityStrength({100.f, 100.f});

    // auto& noise = dts->initialize_parameters.damage_graph.noise_descriptors.emplace_back();
    // noise.type = static_cast<unsigned>(NoiseType::Perlin);
    // noise.multiplier = 0.95f;
    // noise.shift = glm::vec3(1000.f);
    target.board_experiment_setup_settings.center_damage = 0.7f;
    dts->initialize_parameters.damage_scale_factor = glm::vec3(0.01f);
    target.board_experiment_setup_settings.rod_dimension = {160, 10, 20};

    const auto sphere_entity = scene->CreateEntity("Sphere");
    scene->GetOrSetPrivateComponent<DsSphereCollider>(sphere_entity);
    const auto mmr = scene->GetOrSetPrivateComponent<MeshRenderer>(sphere_entity).lock();
    mmr->mesh = Resources::GetInstance().GetPrimitives().sphere;
    mmr->material = AssetManager::CreateTemporaryAsset<Material>();
    target.object_initial_pose.SetPosition(glm::vec3(0.5f, 1.3f, 0));
    target.object_initial_pose.SetScale(glm::vec3(0.3f));
    scene->SetDataComponent(sphere_entity, target.object_initial_pose);
    target.temp_entity1_ref = sphere_entity;
    dts->BoardExperimentSetup(target.board_experiment_setup_settings);
    editor_layer->SetSceneCameraRotation(camera_pose.GetRotation());
    editor_layer->SetSceneCameraPosition(camera_pose.GetPosition());
  }

  if (ImGui::Button("Small cylinder Collision")) {
    ResetEnvironment(target, editor_layer);
    camera_pose.SetPosition(glm::vec3(-0.5, 2.2, 1));
    camera_pose.SetEulerRotation(glm::radians(glm::vec3(-40, -45, 0)));
    target.target_factor0 = 0.f;
    target.demo_type = DynamicStrandsDemo::DemoType::BoardCollision;
    target.demo_status = DynamicStrandsDemo::DemoStatus::Simulation;

    dts->initialize_parameters.strength_graph.SetShearStretchStrength({100.f, 100.f});
    dts->initialize_parameters.strength_graph.SetBendingStrength({100.f, 100.f});
    dts->initialize_parameters.strength_graph.SetTwistingStrength({100.f, 100.f});
    dts->initialize_parameters.strength_graph.SetBundleStrength({100.f, 100.f});
    dts->initialize_parameters.strength_graph.SetConnectivityStrength({100.f, 100.f});
    // auto& noise = dts->initialize_parameters.damage_graph.noise_descriptors.emplace_back();
    // noise.type = static_cast<unsigned>(NoiseType::Perlin);
    // noise.multiplier = 0.99f;
    target.board_experiment_setup_settings.center_damage = 0.0f;
    dts->initialize_parameters.damage_scale_factor = glm::vec3(0.005f, 0.1f, 0.05f);
    target.board_experiment_setup_settings.rod_dimension = {160, 10, 20};

    const auto cylinder_entity = scene->CreateEntity("Cylinder");
    scene->GetOrSetPrivateComponent<DsCylinderCollider>(cylinder_entity);
    const auto mmr = scene->GetOrSetPrivateComponent<MeshRenderer>(cylinder_entity).lock();
    mmr->mesh = Resources::GetInstance().GetPrimitives().cylinder;
    mmr->material = AssetManager::CreateTemporaryAsset<Material>();

    target.object_initial_pose.SetPosition(glm::vec3(0.5f, 1.3f, 0));
    target.object_initial_pose.SetEulerRotation(glm::radians(glm::vec3(90.f, 0, 0)));
    target.object_initial_pose.SetScale(glm::vec3(0.2f));
    scene->SetDataComponent(cylinder_entity, target.object_initial_pose);

    target.temp_entity1_ref = cylinder_entity;
    dts->BoardExperimentSetup(target.board_experiment_setup_settings);
    editor_layer->SetSceneCameraRotation(camera_pose.GetRotation());
    editor_layer->SetSceneCameraPosition(camera_pose.GetPosition());
  }
  if (ImGui::Button("Big cylinder Collision")) {
    ResetEnvironment(target, editor_layer);
    camera_pose.SetPosition(glm::vec3(-0.5, 2.2, 1));
    camera_pose.SetEulerRotation(glm::radians(glm::vec3(-40, -45, 0)));
    target.target_factor0 = 0.f;
    target.demo_type = DynamicStrandsDemo::DemoType::BoardCollision;
    target.demo_status = DynamicStrandsDemo::DemoStatus::Simulation;

    dts->initialize_parameters.strength_graph.SetShearStretchStrength({100.f, 100.f});
    dts->initialize_parameters.strength_graph.SetBendingStrength({100.f, 100.f});
    dts->initialize_parameters.strength_graph.SetTwistingStrength({100.f, 100.f});
    dts->initialize_parameters.strength_graph.SetBundleStrength({100.f, 100.f});
    dts->initialize_parameters.strength_graph.SetConnectivityStrength({100.f, 100.f});

    // auto& noise = dts->initialize_parameters.damage_graph.noise_descriptors.emplace_back();
    // noise.type = static_cast<unsigned>(NoiseType::Perlin);
    // noise.multiplier = 0.99f;
    target.board_experiment_setup_settings.center_damage = 0.0f;
    dts->initialize_parameters.damage_scale_factor = glm::vec3(0.005f, 0.1f, 0.05f);
    target.board_experiment_setup_settings.rod_dimension = {160, 10, 20};

    const auto cylinder_entity = scene->CreateEntity("Cylinder");
    scene->GetOrSetPrivateComponent<DsCylinderCollider>(cylinder_entity);
    const auto mmr = scene->GetOrSetPrivateComponent<MeshRenderer>(cylinder_entity).lock();
    mmr->mesh = Resources::GetInstance().GetPrimitives().cylinder;
    mmr->material = AssetManager::CreateTemporaryAsset<Material>();
    target.object_initial_pose.SetPosition(glm::vec3(0.5f, 1.3f, 0));
    target.object_initial_pose.SetEulerRotation(glm::radians(glm::vec3(90.f, 0, 0)));
    target.object_initial_pose.SetScale(glm::vec3(0.4f));
    scene->SetDataComponent(cylinder_entity, target.object_initial_pose);

    target.temp_entity1_ref = cylinder_entity;
    dts->BoardExperimentSetup(target.board_experiment_setup_settings);
    editor_layer->SetSceneCameraRotation(camera_pose.GetRotation());
    editor_layer->SetSceneCameraPosition(camera_pose.GetPosition());
  }

  if (ImGui::Button("Trunk Strength [Low]")) {
    ResetEnvironment(target, editor_layer);
    target.demo_type = DynamicStrandsDemo::DemoType::TrunkStrength;
    target.demo_status = DynamicStrandsDemo::DemoStatus::TreeGrowth;
    const auto tree_entity = scene->CreateEntity("Tree");
    target.tree_entity_ref = tree_entity;
    const auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock();
    tree->shoot_model.seed = 10;
    // tree.
    scene->SetDataComponent(tree_entity, target.tree_initial_pose);
    target.target_growth_time = 8.f;
    tree->tree_descriptor_ref = ProjectManager::GetOrCreateAsset("./TreeDescriptors/Acacia.tree");
    const auto tree_dts = scene->GetOrSetPrivateComponent<DynamicTreeStrands>(tree_entity).lock();

    BiologicalPropertiesGraph::Output biological_properties_output;
    biological_properties_output.trunk_additional_strength_factor = 500.f;
    tree_dts->initialize_parameters.biological_properties_graph.SetValues(biological_properties_output);

    tree_dts->enable_physics = false;
    editor_layer->SetSceneCameraRotation(camera_pose.GetRotation());
    editor_layer->SetSceneCameraPosition(camera_pose.GetPosition());
  }
  if (ImGui::Button("Trunk Strength [High]")) {
    ResetEnvironment(target, editor_layer);
    target.demo_type = DynamicStrandsDemo::DemoType::TrunkStrength;
    target.demo_status = DynamicStrandsDemo::DemoStatus::TreeGrowth;
    const auto tree_entity = scene->CreateEntity("Tree");
    target.tree_entity_ref = tree_entity;
    const auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock();
    tree->shoot_model.seed = 10;
    // tree.
    scene->SetDataComponent(tree_entity, target.tree_initial_pose);
    target.target_growth_time = 8.f;
    tree->tree_descriptor_ref = ProjectManager::GetOrCreateAsset("./TreeDescriptors/Acacia.tree");
    const auto tree_dts = scene->GetOrSetPrivateComponent<DynamicTreeStrands>(tree_entity).lock();
    tree_dts->enable_physics = false;

    BiologicalPropertiesGraph::Output biological_properties_output;
    biological_properties_output.trunk_additional_strength_factor = 1250.f;
    tree_dts->initialize_parameters.biological_properties_graph.SetValues(biological_properties_output);

    editor_layer->SetSceneCameraRotation(camera_pose.GetRotation());
    editor_layer->SetSceneCameraPosition(camera_pose.GetPosition());
  }

  if (ImGui::Button("Wind [Low]")) {
    ResetEnvironment(target, editor_layer);
    target.demo_type = DynamicStrandsDemo::DemoType::Wind;
    target.demo_status = DynamicStrandsDemo::DemoStatus::TreeGrowth;
    const auto tree_entity = scene->CreateEntity("Tree");
    target.tree_entity_ref = tree_entity;
    const auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock();
    // tree.
    scene->SetDataComponent(tree_entity, target.tree_initial_pose);
    target.target_growth_time = 8.f;
    tree->tree_descriptor_ref = ProjectManager::GetOrCreateAsset("./TreeDescriptors/Oak.tree");
    const auto tree_dts = scene->GetOrSetPrivateComponent<DynamicTreeStrands>(tree_entity).lock();
    tree_dts->enable_physics = false;
    target.target_factor0 = 0.05f;
    target.physics_parameters.segment_velocity_damping = 10.f;
    target.physics_parameters.segment_angular_velocity_damping = 10.f;
    editor_layer->SetSceneCameraRotation(camera_pose.GetRotation());
    editor_layer->SetSceneCameraPosition(camera_pose.GetPosition());
  }
  if (ImGui::Button("Wind [High]")) {
    ResetEnvironment(target, editor_layer);
    target.demo_type = DynamicStrandsDemo::DemoType::Wind;
    target.demo_status = DynamicStrandsDemo::DemoStatus::TreeGrowth;
    const auto tree_entity = scene->CreateEntity("Tree");
    target.tree_entity_ref = tree_entity;
    const auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock();
    // tree.
    scene->SetDataComponent(tree_entity, target.tree_initial_pose);
    target.target_growth_time = 8.f;
    tree->tree_descriptor_ref = ProjectManager::GetOrCreateAsset("./TreeDescriptors/Oak.tree");
    const auto tree_dts = scene->GetOrSetPrivateComponent<DynamicTreeStrands>(tree_entity).lock();
    tree_dts->enable_physics = false;
    target.target_factor0 = 0.16f;
    target.physics_parameters.segment_velocity_damping = 10.f;
    target.physics_parameters.segment_angular_velocity_damping = 10.f;
    editor_layer->SetSceneCameraRotation(camera_pose.GetRotation());
    editor_layer->SetSceneCameraPosition(camera_pose.GetPosition());
  }

  if (ImGui::Button("Tree Collision")) {
    ResetEnvironment(target, editor_layer);
    target.demo_type = DynamicStrandsDemo::DemoType::TreeCollision;
    target.demo_status = DynamicStrandsDemo::DemoStatus::TreeGrowth;
    const auto tree_entity = scene->CreateEntity("Tree");
    target.tree_entity_ref = tree_entity;
    const auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock();
    // tree.
    scene->SetDataComponent(tree_entity, target.tree_initial_pose);
    target.target_growth_time = 8.f;
    tree->tree_descriptor_ref = ProjectManager::GetOrCreateAsset("./TreeDescriptors/Acacia.tree");
    const auto tree_dts = scene->GetOrSetPrivateComponent<DynamicTreeStrands>(tree_entity).lock();
    tree_dts->enable_physics = false;
    target.target_factor0 = 0.03f;
    tree->shoot_model.seed = 8;
    const auto cylinder_entity = scene->CreateEntity("Cylinder");
    scene->GetOrSetPrivateComponent<DsCylinderCollider>(cylinder_entity);
    const auto mmr = scene->GetOrSetPrivateComponent<MeshRenderer>(cylinder_entity).lock();
    mmr->mesh = Resources::GetInstance().GetPrimitives().cylinder;
    mmr->material = AssetManager::CreateTemporaryAsset<Material>();
    target.object_initial_pose.SetPosition(glm::vec3(-1.f, .5f, 0));
    target.object_initial_pose.SetEulerRotation(glm::radians(glm::vec3(90.f, 0, 0)));
    target.object_initial_pose.SetScale(glm::vec3(0.2f, 2.f, 0.2f));
    scene->SetDataComponent(cylinder_entity, target.object_initial_pose);

    target.temp_entity1_ref = cylinder_entity;
    editor_layer->SetSceneCameraRotation(camera_pose.GetRotation());
    editor_layer->SetSceneCameraPosition(camera_pose.GetPosition());
  }

  if (ImGui::Button("Tree Break")) {
    ResetEnvironment(target, editor_layer);
    target.demo_type = DynamicStrandsDemo::DemoType::TreeBreak;
    target.demo_status = DynamicStrandsDemo::DemoStatus::TreeGrowth;
    const auto tree_entity = scene->CreateEntity("Tree");
    target.tree_entity_ref = tree_entity;
    const auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock();
    // tree.
    scene->SetDataComponent(tree_entity, target.tree_initial_pose);
    target.target_growth_time = 12.f;
    tree->tree_descriptor_ref = ProjectManager::GetOrCreateAsset("./TreeDescriptors/Demo.tree");
    const auto tree_dts = scene->GetOrSetPrivateComponent<DynamicTreeStrands>(tree_entity).lock();
    tree_dts->enable_physics = false;
    tree->shoot_model.seed = 3;
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

  return changed;
}
