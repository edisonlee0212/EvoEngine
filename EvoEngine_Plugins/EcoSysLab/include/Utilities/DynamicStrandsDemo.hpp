#pragma once
#include "DynamicStrands.hpp"
#include "DynamicTreeSkeleton.hpp"
#include "DynamicTreeStrands.hpp"
#include "SimulationSettings.hpp"
#include "Tree.hpp"

namespace eco_sys_lab_plugin {
/**
 * @class DynamicStrandsDemo
 * @brief A demonstration class for simulating dynamic strands in a tree model.
 */
class DynamicStrandsDemo : public IPrivateComponent {
  /// @brief Target simulation time in seconds.
  float target_simulation_time = 100.f;

  /// @brief Currently simulated time in seconds.
  float simulated_time = 0.f;

  /// @brief Target factor for simulation parameter 0.
  float target_factor0 = 1.f;

  /// @brief Target factor for simulation parameter 1.
  float target_factor1 = 1.f;

  /// @brief Target growth time for tree-growth demos, in years.
  float target_growth_time = 0.f;

  /// @brief Reference to a temporary entity.
  EntityRef temp_entity1_ref;

  /// @brief Reference to a tree entity in the simulation.
  EntityRef tree_entity_ref;

  /// @brief Initial pose of the object.
  GlobalTransform object_initial_pose{};

  /// @brief Initial pose of the tree.
  GlobalTransform tree_initial_pose{};

  /// @brief Camera pose in the scene.
  GlobalTransform camera_pose;

  /**
   * @brief Resets the environment including entities and their transformations.
   * @param editor_layer Shared pointer to the editor layer handling the environment.
   */
  void ResetEnvironment(const std::shared_ptr<EditorLayer>& editor_layer);

 public:
  /**
   * @enum DemoType
   * @brief Defines different types of demo scenarios for strand physics.
   */
  enum class DemoType {
    Empty,            ///< No demo scenario selected.
    LogBreak,         ///< Demonstrates log breaking physics.
    BoardBreak,       ///< Simulates board breaking physics.
    TwistingBreak,    ///< Demonstrates twisting break effects.
    BendingBreak,     ///< Simulates bending-induced breaking.
    ShearingBreak,    ///< Demonstrates shearing-based breaking.
    StretchingBreak,  ///< Demonstrates stretching physics effects.
    SapHeart,         ///< Simulates sap heart wood differences.
    BoardCollision,   ///< Demonstrates board collision physics.
    TrunkStrength,    ///< Demonstrates different tree trunk strength.
    Wind,             ///< Simulates tree reaction to wind.
    TreeCollision,    ///< Demonstrates tree collisions.
    TreeBreak,        ///< Demonstrates tree breaking physics.
    Fungus,           ///< Fungus / Woodstock demos.
    SmallTrunk,       ///< Grow Oak_trunk (4 years) then run volumetric meshing.
    NormalTrunk,      ///< Grow Oak_trunk (8 years) then run volumetric meshing.
    LogCut,           ///< Volumetric log cut + board-style pivot break simulation.
    LogSpoon          ///< Volumetric log spoon cut + board-style pivot break simulation.
  };

  /**
   * @enum DemoStatus
   * @brief Defines the status of the demo simulation.
   */
  enum class DemoStatus {
    Idle,        ///< No active simulation.
    TreeGrowth,  ///< The tree is growing.
    Simulation   ///< The physics simulation is running.
  };

  /// @brief Settings for the physics simulation.
  SimulationSettings simulation_settings{};

  /// @brief Statistics for the ongoing simulation.
  SimulationStats simulation_stats{};

  /// @brief Physics parameters used in the dynamic strands.
  DynamicStrands::PhysicsParameters physics_parameters{};

  /// @brief Settings for log experiment setup.
  DynamicTreeStrands::LogExperimentSetupSettings log_experiment_setup_settings{};

  /// @brief Settings for board experiment setup.
  DynamicTreeStrands::BoardExperimentSetupSettings board_experiment_setup_settings{};

  /// @brief Description applied when tree-growth demos call InitializeFromTree.
  std::string pending_meshing_buffer_description;

  /// @brief True after EcoSysLab auto-grow has been requested for the current tree-growth demo.
  bool tree_auto_grow_started_ = false;

  /// @brief When true, download GPU meshlets and export OBJ at scheduled simulation times (no file dialog).
  bool automated_export = false;
  /// @brief Inclusive lower simulation time (seconds) for the first automated export.
  float automated_export_lower = 0.f;
  /// @brief Inclusive upper simulation time (seconds) for the last automated export.
  float automated_export_upper = 10.f;
  /// @brief Time step between automated exports (seconds).
  float automated_export_stepsize = 0.5f;

  /// @brief Current demo type being simulated.
  DemoType demo_type = DemoType::Empty;

  /// @brief Current status of the demo simulation.
  DemoStatus demo_status = DemoStatus::Idle;

  /**
   * @brief Inspects and modifies properties of the demo in the editor interface.
   * @param editor_layer Shared pointer to the editor layer for UI rendering.
   * @return True if asset content is unmodified, otherwise false.
   */
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;

  /**
   * @brief Updates the physics simulation and demo state.
   */
  void Update() override;

 private:
  /// Next simulation time at which automated export should fire.
  float next_automated_export_time_ = 0.f;

  /// Start EcoSysLab auto-grow for @ref target_growth_time years (async; advances in EcoSysLabLayer::Update).
  void BeginTreeAutoGrow();
  /// If auto-grow has finished, build strands/mesh and enter physics Simulation. Safe to call from OnInspect.
  void TryFinishTreeGrowthAndStartMeshing();
  /// Reset the automated-export schedule to @ref automated_export_lower.
  void ResetAutomatedExportSchedule();
  /// Folder name under PhysicsDemoExports for the current @ref demo_type.
  [[nodiscard]] static const char* DemoTypeExportFolderName(DemoType type);
  /// DynamicTreeStrands that owns the active simulation mesh (owner or tree entity).
  [[nodiscard]] std::shared_ptr<DynamicTreeStrands> GetActiveDynamicTreeStrands();
  /// If due, download meshlets and write OBJs for all scheduled times <= @p time.
  void TryAutomatedExportsUpTo(float time);
};
}  // namespace eco_sys_lab_plugin