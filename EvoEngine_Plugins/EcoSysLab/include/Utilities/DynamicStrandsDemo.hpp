#pragma once
#include "DynamicStrands.hpp"
#include "DynamicTreeSkeleton.hpp"
#include "DynamicTreeStrands.hpp"
#include "Tree.hpp"

namespace eco_sys_lab_plugin {
class DynamicStrandsDemo : public IPrivateComponent {
  float target_simulation_time = 10.f;
  float simulated_time = 0.f;

  float target_factor0 = 1.f;
  float target_factor1 = 1.f;

 public:
  enum class DemoType {
    Empty,
    DryBreakRod,
    BreakBoardLow,
    BreakBoardMed,
    BreakBoardHigh,
    TwistingBreak,
    BendingBreak,
    ShearingBreak,
    StretchingBreak
  };
  DynamicStrands::PhysicsParameters physics_parameters{};
  DynamicTreeStrands::LogExperimentSetupSettings log_experiment_setup_settings{};
  DynamicTreeStrands::BoardExperimentSetupSettings board_experiment_setup_settings{};

  DemoType demo_type = DemoType::Empty;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void Update() override;
  void LateUpdate() override;
};

}  // namespace eco_sys_lab_plugin