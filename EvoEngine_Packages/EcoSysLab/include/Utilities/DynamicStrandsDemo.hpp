#pragma once
#include <atomic>
#include <filesystem>
#include <memory>
#include <mutex>
#include <vector>
#include "DynamicStrands.hpp"
#include "DynamicTreeSkeleton.hpp"
#include "DynamicTreeStrands.hpp"
#include "KineticMeshCachePipeline.hpp"
#include "SimulationSettings.hpp"
#include "Tree.hpp"
#include "VolumetricTreeExperiment.hpp"

namespace eco_sys_lab_package {
void ApplyVolumetricTreePhysicsPreset(DynamicStrands::PhysicsParameters& physics_parameters);
/**
 * @class DynamicStrandsDemo
 * @brief A demonstration class for simulating dynamic strands in a tree model.
 */
class DynamicStrandsDemo : public IPrivateComponent {
  friend struct DynamicStrandsDemoInspector;
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

  /// @brief Target growth iterations (Simulate steps). When > 0, used instead of @ref target_growth_time.
  int target_growth_iterations = 0;

  /// @brief Reference to a temporary entity.
  EntityRef temp_entity1_ref;

  /// @brief Reference to a tree entity in the simulation.
  EntityRef tree_entity_ref;

  /// @brief Initial pose of the object.
  GlobalTransform object_initial_pose{};

  /// @brief Initial pose of the tree.
  GlobalTransform tree_initial_pose{};
  GlobalTransform camera_pose{};

 public:
  /**
   * @enum DemoType
   * @brief Defines different types of demo scenarios for strand physics.
   */
  enum class DemoType {
    Empty,               ///< No demo scenario selected.
    LogBreak,            ///< Demonstrates log breaking physics.
    BoardBreak,          ///< Simulates board breaking physics.
    TwistingBreak,       ///< Demonstrates twisting break effects.
    BendingBreak,        ///< Simulates bending-induced breaking.
    ShearingBreak,       ///< Demonstrates shearing-based breaking.
    StretchingBreak,     ///< Demonstrates stretching physics effects.
    SapHeart,            ///< Simulates sap heart wood differences.
    BoardCollision,      ///< Demonstrates board collision physics.
    TrunkStrength,       ///< Demonstrates different tree trunk strength.
    Wind,                ///< Simulates tree reaction to wind.
    TreeCollision,       ///< Demonstrates tree collisions.
    TreeBreak,           ///< Demonstrates tree breaking physics.
    Fungus,              ///< Fungus / Woodstock demos.
    SmallTrunk,          ///< Grow Oak_trunk (4 years) then run volumetric meshing.
    NormalTrunk,         ///< Grow Oak_trunk (8 years) then run volumetric meshing.
    StockyTrunk,         ///< Grow Oak_trunk_stocky (seed 42) then run volumetric meshing.
    OakThickStump,       ///< Grow Oak (15 iterations, default seed); 100 end strands, alpha cutoffs 5, tension 1.
    OakThickStump200,    ///< Same as OakThickStump but 200 end strands/branch.
    OakThickStump300,    ///< Same as OakThickStump but 300 end strands/branch.
    OakThickStump400,    ///< Same as OakThickStump but 400 end strands/branch.
    OakTwoYearSparse,    ///< Grow Oak for 2 years; 10 end strands/branch, alpha cutoffs 5, tension 1.
    OakTwoYear20,        ///< Grow Oak for 2 years; 20 end strands/branch.
    OakTwoYear50,        ///< Grow Oak for 2 years; 50 end strands/branch.
    OakTwoYear100,       ///< Grow Oak for 2 years; 100 end strands/branch.
    OakThreeYear4,       ///< Grow Oak for 3 years; 4 end strands/branch.
    OakThreeYear10,      ///< Grow Oak for 3 years; 10 end strands/branch.
    OakThreeYear20,      ///< Grow Oak for 3 years; 20 end strands/branch.
    OakThreeYear50,      ///< Grow Oak for 3 years; 50 end strands/branch.
    OakFourYearSparse,   ///< Grow Oak for 4 years; 4 end strands/branch, alpha cutoffs 5, tension 1.
    OakSixYearSparse,    ///< Grow Oak for 6 years; 4 end strands/branch, alpha cutoffs 5, tension 1.
    OakEightYearSparse,  ///< Grow Oak for 8 years; 4 end strands/branch, alpha cutoffs 5, tension 1.
    LogCut,              ///< Volumetric log cut + board-style pivot break simulation.
    LogBreakComparison,  ///< Log-cut style volumetric log without intersection meshes; 1600 strands for Log-break
                         ///< comparison.
    LogBreakComparisonHalfCutoffDoubleAlpha,  ///< Same as LogBreakComparison with Kinetic cutoff ×½ and Alpha α
                                              ///< = 1.3e-4.
    LogSpoon,                                 ///< Volumetric log spoon cut + board-style pivot break simulation.
    LogCutUprightBunny                        ///< Upright half-length log + bunny boundary (bottom fixed, top rotates).
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

  /// @brief When true, @ref override_min_segment_length and @ref override_max_segment_length replace per-experiment
  /// values.
  bool override_experiment_segment_subdivision = false;
  /// @brief Minimum segment length for random subdivision when override is enabled.
  float override_min_segment_length = 0.02f;
  /// @brief Maximum segment length for random subdivision when override is enabled.
  float override_max_segment_length = 0.04f;

  /// @brief Description applied when tree-growth demos call InitializeFromTree.
  std::string pending_meshing_buffer_description;
  /// @brief Experiment started by @ref StartVolumetricTreeExperiment (avoids DemoType collisions).
  VolumetricTreeExperimentId pending_volumetric_experiment_id_ = VolumetricTreeExperimentId::Count;

  /// @brief True after EcoSysLab auto-grow has been requested for the current tree-growth demo.
  bool tree_auto_grow_started_ = false;

  /// @brief When true, download GPU meshlets and export at scheduled simulation times (no file dialog).
  bool automated_export = false;
  /// @brief Automated export: write regular mesh OBJs (kinetic_meshlets_*/alpha_tetrahedra_*).
  bool automated_export_meshes = true;
  /// @brief Automated export: write volume-change heatmap OBJs.
  bool automated_export_heatmaps = true;
  /// @brief Automated export: write volume measure / per-element CSVs.
  bool automated_export_csvs = true;
  /// @brief Inclusive lower simulation time (seconds) for the first automated export.
  float automated_export_lower = 0.f;
  /// @brief Inclusive upper simulation time (seconds) for the last automated export.
  float automated_export_upper = 25.f;
  /// @brief Time step between automated exports (seconds).
  float automated_export_stepsize = 5.f;

  /// @brief Current demo type being simulated.
  DemoType demo_type = DemoType::Empty;

  /// @brief Current status of the demo simulation.
  DemoStatus demo_status = DemoStatus::Idle;

  /// Folder name under PhysicsDemoExports for the current @ref demo_type.
  [[nodiscard]] static const char* DemoTypeExportFolderName(DemoType type);

  /**
   * @brief Updates the physics simulation and demo state.
   */
  void Update() override;
  [[nodiscard]] bool ControlsStrands(Entity entity);
  void ResetEnvironment();
  bool DrawVolumetricMeshingUi();

 private:
  enum class PrecomputeJobStatus { Queued, Growing, Exporting, ReadyToMesh, Running, CacheHit, Done, Failed, Crashed };

  struct PrecomputeJob {
    std::string experiment_cli_id;
    std::string display_name;
    PrecomputeJobStatus status = PrecomputeJobStatus::Queued;
    std::filesystem::path log_path;
    std::filesystem::path job_path;
    std::filesystem::path strand_tree_path;
#ifdef _WIN32
    void* process_handle = nullptr;  // HANDLE
    unsigned long process_id = 0;
#else
    int process_pid = -1;
#endif
  };

  struct PrecomputeExportResult {
    size_t job_index = static_cast<size_t>(-1);
    bool ok = false;
  };

  /// Non-copyable sync state lives behind shared_ptr so IPrivateComponent clone assignment stays valid.
  struct PrecomputeExportSync {
    std::mutex mutex;
    std::vector<PrecomputeExportResult> results;
    std::atomic<int> exporting_count{0};
  };

  std::vector<char> precompute_selected_;  // parallel to GetVolumetricTreeExperiments()
  /// Mesh-worker concurrency (growth in the parent is always one-at-a-time).
  int precompute_max_concurrent_ = 24;
  bool precompute_cancel_requested_ = false;
  bool precompute_active_ = false;
  std::vector<PrecomputeJob> precompute_jobs_;
  /// Index into @ref precompute_jobs_ while parent-side growth is running; npos when idle.
  size_t precompute_growing_job_index_ = static_cast<size_t>(-1);
  /// StrandTree/job export runs off the main thread so EcoSysLab stays responsive.
  std::shared_ptr<PrecomputeExportSync> precompute_export_sync_ = std::make_shared<PrecomputeExportSync>();

  void EnsurePrecomputeSelectionSize();
  void DrawVolumetricPrecomputeUi();
  void StartPrecomputeQueue();
  void PollPrecomputeJobs();
  void PumpPrecomputeQueue();
  void StartPrecomputeGrowth(size_t job_index);
  void FinishPrecomputeGrowthAndSpawnWorker();
  [[nodiscard]] bool SpawnPrecomputeWorker(PrecomputeJob& job);
  [[nodiscard]] static const char* PrecomputeJobStatusLabel(PrecomputeJobStatus status);

  /// Next simulation time at which automated export should fire.
  float next_automated_export_time_ = 0.f;
  /// Output folder for the current automated-export run (includes a timestamp subfolder).
  std::filesystem::path automated_export_folder_;
  /// Column order for per-meshlet / per-tet absolute-volume CSVs (set on first write).
  std::vector<unsigned int> automated_kinetic_meshlet_columns_;
  std::vector<size_t> automated_alpha_tet_columns_;
  /// Snapshot of export settings; used to detect mid-run edits without replaying past times.
  bool automated_export_snapshot_ = false;
  bool automated_export_meshes_snapshot_ = true;
  bool automated_export_heatmaps_snapshot_ = true;
  bool automated_export_csvs_snapshot_ = true;
  float automated_export_lower_snapshot_ = 0.f;
  float automated_export_upper_snapshot_ = 25.f;
  float automated_export_stepsize_snapshot_ = 5.f;

  /// Start EcoSysLab auto-grow for @ref target_growth_iterations Simulate steps when > 0,
  /// otherwise @ref target_growth_time years (async; advances in EcoSysLabLayer::Update).
  void BeginTreeAutoGrow();
  /// If auto-grow has finished, build strands/mesh and enter physics Simulation. Safe to call from OnInspect.
  void TryFinishTreeGrowthAndStartMeshing();
  /// Reset the automated-export schedule to @ref automated_export_lower.
  void ResetAutomatedExportSchedule();
  /// Advance @ref next_automated_export_time_ to the next slot after @p current_time (skip past times).
  void AdvanceAutomatedExportScheduleFrom(float current_time);
  void SnapshotAutomatedExportSettings();
  [[nodiscard]] bool AutomatedExportSettingsChanged() const;
  /// DynamicTreeStrands on the PhysicsDemo owner (including TreeDescriptor demos).
  [[nodiscard]] std::shared_ptr<DynamicTreeStrands> GetActiveDynamicTreeStrands();
  /// If due, download meshes and write OBJs/heatmaps/volume CSVs for all scheduled times <= @p time.
  void TryAutomatedExportsUpTo(float time);
  void ApplySegmentSubdivisionOverride(DynamicStrandsInitializeParameters& initialize_parameters) const;
  void RunLogExperimentSetup(const std::shared_ptr<DynamicTreeStrands>& dts);
  void RunBoardExperimentSetup(const std::shared_ptr<DynamicTreeStrands>& dts);
  void StartVolumetricTreeExperiment(const VolumetricTreeExperiment& experiment);
  void StartPrecomputedExperiment(const PrecomputedExperimentAssets& assets);
  void DrawPrecomputedExperimentsUi();

  /// When set, after growth load StrandTree subdivisions + force-load MeshBuffers from Assets.
  std::string pending_precomputed_cli_id_;
  std::filesystem::path pending_precomputed_strand_tree_;
  std::filesystem::path pending_precomputed_mesh_bin_;
  std::string pending_precomputed_hash_;

  /// Pending alpha (= cutoff^2) values for a parameter sweep after tree growth (empty when inactive).
  std::vector<double> alpha_sweep_;
  /// Filename / folder tag for the active sweep (e.g. "Small_Trunk", "Oak_thick_stump").
  std::string alpha_sweep_experiment_name_;
  /// When true, skip meshing and run 2D alpha-hull fractal analysis for @ref alpha_hull_height_min_..@ref
  /// alpha_hull_height_max_.
  bool alpha_hull_fractal_pending_ = false;
  /// Inclusive section-index range for alpha-hull fractal analysis (default 0..25).
  size_t alpha_hull_height_min_ = 0;
  size_t alpha_hull_height_max_ = 25;
  /// After tree growth, mesh once per pending alpha (forces remesh so statistics are written).
  void RunAlphaSweepMeshing(const std::shared_ptr<Tree>& tree, const std::shared_ptr<DynamicTreeStrands>& dts);
  /// Dry-run StrandTree, Delaunay + alpha hulls for each section in range, fractal dimensions (no meshing).
  void RunAlphaHullFractalExperiment(const std::shared_ptr<Tree>& tree, const std::shared_ptr<DynamicTreeStrands>& dts);
};
}  // namespace eco_sys_lab_package
