#pragma once
#include "DynamicStrands.hpp"
#include "DynamicTreeSkeleton.hpp"
#include "DynamicTreeStrands.hpp"
#include "SimulationSettings.hpp"
#include "Tree.hpp"

namespace eco_sys_lab_plugin {
class DynamicStrandsDemo : public IPrivateComponent {
  float target_simulation_time = 10.f;
  float simulated_time = 0.f;

  float target_factor0 = 1.f;
  float target_factor1 = 1.f;

  float target_growth_time = 0.f;
  EntityRef temp_entity1_ref;
  EntityRef tree_entity_ref;
  GlobalTransform object_initial_pose{};
  GlobalTransform tree_initial_pose{};

  GlobalTransform camera_pose;
  void ResetEnvironment(const std::shared_ptr<EditorLayer>& editor_layer);

 public:
  enum class DemoType {
    Empty,
    LogBreak,
    BoardBreak,
    TwistingBreak,
    BendingBreak,
    ShearingBreak,
    StretchingBreak,
    SapHeart,
    BoardCollision,
    TrunkStrength,
    Wind,
    TreeCollision
  };
  enum class DemoStatus { Idle, TreeGrowth, Simulation };

  SimulationSettings simulation_settings{};
  SimulationStats simulation_stats{};

  DynamicStrands::PhysicsParameters physics_parameters{};
  DynamicTreeStrands::LogExperimentSetupSettings log_experiment_setup_settings{};
  DynamicTreeStrands::BoardExperimentSetupSettings board_experiment_setup_settings{};

  DemoType demo_type = DemoType::Empty;
  DemoStatus demo_status = DemoStatus::Idle;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void Update() override;
};

}  // namespace eco_sys_lab_plugin