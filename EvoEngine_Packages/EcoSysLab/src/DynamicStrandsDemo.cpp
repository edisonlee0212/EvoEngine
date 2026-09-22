#include "DynamicStrandsDemo.hpp"

#include "DsColliders.hpp"
#include "DynamicTreeStrands.hpp"
#include "EcoSysLabLayer.hpp"

using namespace eco_sys_lab_package;

bool DynamicStrandsDemo::ControlsStrands(Entity entity) {
  if (demo_status != DemoStatus::Simulation)
    return false;
  if (entity == GetOwner())
    return true;
  return (demo_type == DemoType::TrunkStrength || demo_type == DemoType::Wind ||
          demo_type == DemoType::TreeCollision) &&
         entity == tree_entity_ref.Get();
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
    dts->dynamic_strands->UpdateBindings();
    const auto eco_sys_lab_layer = ApplicationContext::Get().GetLayer<EcoSysLabLayer>();
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

  const auto eco_sys_lab_layer = ApplicationContext::Get().GetLayer<EcoSysLabLayer>();
  const bool run_physics = eco_sys_lab_layer->IsDynamicStrandsPhysicsRunning();
  const bool run_fungus = physics_parameters.enable_fungus && eco_sys_lab_layer->IsDynamicStrandsFungusRunning();
  const int fungus_steps = eco_sys_lab_layer->GetDynamicStrandsFungusStepsPerFrame();
  if (!run_physics) {
    if (run_fungus) {
      auto fungus_target = dts;
      if (demo_type == DemoType::TrunkStrength || demo_type == DemoType::Wind || demo_type == DemoType::TreeCollision) {
        const auto tree_entity = tree_entity_ref.Get();
        if (scene->IsEntityValid(tree_entity))
          fungus_target = scene->GetOrSetPrivateComponent<DynamicTreeStrands>(tree_entity).lock();
      }
      fungus_target->dynamic_strands->UpdateBindings();
      fungus_target->FungusStep(physics_parameters, fungus_steps);
    }
    return;
  }
  const auto step_physics = [&](const std::shared_ptr<DynamicTreeStrands>& target) {
    target->dynamic_strands->UpdateBindings();
    target->PhysicsStep(physics_parameters, run_fungus ? fungus_steps : 0);
  };

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
      step_physics(dts);
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
      step_physics(dts);
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
      step_physics(dts);
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
      step_physics(dts);
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
      step_physics(dts);
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
      step_physics(dts);
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
      step_physics(dts);
      break;
    }
    case DemoType::BoardCollision: {
      GlobalTransform gt = object_initial_pose;
      const auto temp_entity = temp_entity1_ref.Get();
      gt.SetPosition(object_initial_pose.GetPosition() + glm::vec3(0, -50, 0) * progress);
      scene->SetDataComponent(temp_entity, gt);
      step_physics(dts);
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
        step_physics(tree_dts);
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
        step_physics(tree_dts);
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
        step_physics(tree_dts);
      }
      break;
    }
    case DemoType::Fungus: {
      step_physics(dts);
      break;
    }
    default:
      break;
  }

  simulated_time += physics_parameters.time_step;
}
