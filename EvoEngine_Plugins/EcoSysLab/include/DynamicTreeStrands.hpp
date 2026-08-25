#pragma once
#include "DsConstraints.hpp"
#include "DsMaterials.hpp"
#include "DsOperators.hpp"
#include "DynamicStrands.hpp"
#include "DynamicStrandsVisualizationParameters.hpp"
#include "ShootGrowthData.hpp"
#include "StrandModelData.hpp"
#include "Tree.hpp"

namespace eco_sys_lab_plugin {
using namespace evo_engine;

/**
 * @class DynamicTreeStrands
 * @brief Handles the dynamic strand physics simulation for tree models.
 */
class DynamicTreeStrands : public IPrivateComponent {
  Handle foliage_rendering_instance_handle;  ///< Handle for foliage rendering instance.

 public:
  DynamicTreeStrands() {
    EVOENGINE_LOG("DynamicTreeStrands component created.");
  }

  /**
   * @brief Initializes dynamic tree strands based on a tree structure.
   * @param tree Shared pointer to the tree structure.
   * @param meshing_buffer_description Optional free-form note for mesh-buffer YML metadata.
   *        When empty, uses a generic InitializeFromTree description.
   */
  void InitializeFromTree(const std::shared_ptr<Tree>& tree, const std::string& meshing_buffer_description = {});

  int seed = 0;                ///< Seed for procedural generation (used when fixed_subdivision_seed is true).
  /// When true, random strand subdivisions use @ref seed so meshing-buffer hashes stay stable across attempts.
  /// When false, a fresh non-deterministic seed is drawn each subdivide.
  bool fixed_subdivision_seed = true;
  StrandModel strand_model{};  ///< Strand model.

  DynamicStrandsInitializeParameters initialize_parameters{};  ///< Initialization parameters for DynamicStrands.
  bool enable_physics = true;                                  ///< Flag to enable or disable physics simulation.
  bool initialized_from_tree = false;
  bool limit_strand_length = false;  ///< Flag to limit strand length.
  float max_strand_length = 1.f;     ///< Maximum allowable strand length.
  DsMaterials materials;

  std::shared_ptr<DynamicStrands> dynamic_strands{};  ///< Shared pointer to DynamicStrands instance.

  /**
   * @struct PivotTransform
   * @brief Defines a transform pivot point for strands.
   */
  struct PivotTransform {
    Entity target_entity;                                  ///< Target entity associated with this pivot.
    std::shared_ptr<DsPivotTransform> ds_pivot_transform;  ///< Transform pivot.
  };

  /**
   * @struct PivotAxis
   * @brief Defines an axis pivot point for strands.
   */
  struct PivotAxis {
    Entity target_entity;                        ///< Target entity associated with this pivot.
    std::shared_ptr<DsPivotAxis> ds_pivot_axis;  ///< Axis pivot.
  };

  /**
   * @struct PivotPoint
   * @brief Defines a point pivot point for strands.
   */
  struct PivotPoint {
    Entity target_entity;                          ///< Target entity associated with this pivot.
    std::shared_ptr<DsPivotPoint> ds_pivot_point;  ///< Point pivot.
  };

  std::vector<PivotPoint> point_pivots;          ///< List of point pivots.
  std::vector<PivotAxis> axis_pivots;            ///< List of axis pivots.
  std::vector<PivotTransform> transform_pivots;  ///< List of transform pivots.

  std::shared_ptr<DsBoxSelection> box_selection_operator;        ///< Operator for box selection.
  std::shared_ptr<DsLineCut> line_cut_operator;                  ///< Operator for line cutting.
  std::shared_ptr<DsPointCut> point_cut_operator;                ///< Operator for point cutting.
  std::shared_ptr<DsSaw> saw_operator;                           ///< Operator for saw operation.
  std::shared_ptr<DsDrag> drag_operator;                         ///< Operator for dragging operation.
  std::shared_ptr<DsLeafDrop> leaf_drop;                         ///< Operator for simulating leaf drop.
  std::shared_ptr<DsSnow> snow;                                  ///< Operator for simulating snow effects.
  std::shared_ptr<DsWind> wind;                                  ///< Operator for wind simulation.
  std::shared_ptr<DsStopAll> stop_all;                           ///< Operator to stop all physics interactions.
  std::shared_ptr<DsFungusInjection> fungus_injection_operator;  ///< Operator for fungus injection.

  /**
   * @brief Updates the dynamic strands simulation.
   */
  void UpdateDynamicStrands(DtsStrandGroup& randomly_subdivided_strand_group,
                            DtsStrandGroup& uniformly_subdivided_strand_group);

  /**
   * @brief Creates a static root structure for the tree strands.
   */
  void CreateStaticRoot();

  /**
   * @brief Serializes the object data to YAML format.
   * @param out YAML emitter to store serialized data.
   */
  void Serialize(YAML::Emitter& out) const override;

  /**
   * @brief Deserializes the object data from YAML format.
   * @param in YAML node containing serialized data.
   */
  void Deserialize(const YAML::Node& in) override;

  /**
   * @brief Inspects and modifies the object in the editor.
   * @param editor_layer Shared pointer to editor layer.
   * @return True if the content is not modified, false otherwise.
   */
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;

  /**
   * @brief Called upon creation of the component.
   */
  void OnCreate() override;

  /**
   * @brief Called upon destruction of the component.
   */
  void OnDestroy() override;

  /**
   * @brief Collects references to all asset resources.
   * @param list Vector to store collected asset references.
   */
  void CollectAssetRef(std::vector<AssetRef>& list) override;

  /**
   * @enum PivotType
   * @brief Types of pivot points for dynamic strands.
   */
  enum class PivotType { Empty, Point, Axis, Transform, Partial_Transform };

  /**
   * @struct BoardExperimentSetupSettings
   * @brief Settings for board experiment setup.
   */
  struct BoardExperimentSetupSettings {
    float segment_length = 0.05f;                                            ///< Length of each segment.
    float radius = 0.002f;                                                   ///< Radius of each strand.
    glm::ivec3 rod_dimension = {20, 40, 20};                                 ///< Dimensions of the rod structure.
    unsigned left_pivot_type = static_cast<unsigned>(PivotType::Transform);  ///< Type of left pivot.
    unsigned right_pivot_type = static_cast<unsigned>(PivotType::Empty);     ///< Type of right pivot.
    float center_damage = 0.5f;                                              ///< Damage factor at the center.
    float center_distance_offset = 0.1f;                                     ///< Offset distance from the center.
    float center_damage_transition = 0.1f;                                   ///< Damage transition factor.
    bool fungus_test = false;
    glm::vec3 initial_velocity = glm::vec3(0.f);          ///< Initial velocity of the structure.
    glm::vec3 initial_angular_velocity = glm::vec3(0.f);  ///< Initial angular velocity of the structure.
    /// Free-form note written into mesh-buffer YML metadata (not hashed).
    std::string meshing_buffer_description = "created from DynamicTreeStrands BoardExperimentSetup";

    /**
     * @brief Inspects board experiment settings in the editor.
     * @param editor_layer Shared pointer to editor layer.
     * @return True if the content is not modified, false otherwise.
     */
    bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);
  };

  /**
   * @struct LogExperimentSetupSettings
   * @brief Settings for log experiment setup.
   */
  struct LogExperimentSetupSettings {
    float segment_length = 0.05f;              ///< Length of each segment.
    float radius = 0.002f;                     ///< Radius of each strand.
    int rod_size = 800;                        ///< Size of the rod structure.
    int rod_segment_count = 20;                ///< Number of segments in the rod.
    float center_attraction_strength = 40000;  ///< Strength of the center attraction force.
    float center_damage = 0.5f;                ///< Damage factor at the center.
    float center_distance_offset = 0.1f;       ///< Offset distance from the center.
    float center_damage_transition = 0.1f;     ///< Damage transition factor.
    unsigned left_pivot_type = static_cast<unsigned>(PivotType::Transform);  ///< Type of left pivot.
    unsigned right_pivot_type = static_cast<unsigned>(PivotType::Empty);     ///< Type of right pivot.
    glm::vec3 initial_velocity = glm::vec3(0.f);                             ///< Initial velocity of the structure.
    glm::vec3 initial_angular_velocity = glm::vec3(0.f);  ///< Initial angular velocity of the structure.
    bool lock_upper = false;                              ///< Flag to lock the upper part.
    bool t_cut = false;                                   ///< Flag to enable T-cut operation.
    bool fungus_test = false;
    bool cube_pattern = false;
    bool internal_pattern = false;
    bool competition_setting = false;
    float t_cut_width = 0.7f;  ///< Width of the T-cut.
    /// Free-form note written into mesh-buffer YML metadata (not hashed).
    std::string meshing_buffer_description = "created from DynamicTreeStrands LogExperimentSetup";

    /**
     * @brief Inspects log experiment settings in the editor.
     * @param editor_layer Shared pointer to editor layer.
     * @return True if the content is not modified, false otherwise.
     */
    bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);
  };

  /**
   * @brief Sets up a board experiment with the given settings.
   * @param settings Configuration settings for the board experiment.
   */
  void BoardExperimentSetup(const BoardExperimentSetupSettings& settings);

  /**
   * @brief Sets up a log experiment with the given settings.
   * @param settings Configuration settings for the log experiment.
   */
  void LogExperimentSetup(const LogExperimentSetupSettings& settings);

  /**
   * @brief Initializes strand particles for the given strand group.
   * @param target_strand_group Strand group to initialize particles for.
   */
  void InitializeStrandParticles(const DtsStrandGroup& target_strand_group) const;

  /**
   * @brief Clears existing strand particles.
   */
  void ClearStrandParticles() const;

  /**
   * @brief Clears strand/mesh GPU+CPU data, pivots, particles, and related child entities
   *        so the component can be re-initialized without restarting the application.
   *        Preserves materials and initialization/editor settings.
   */
  void Reset();

  /**
   * @brief Advances the interaction step in the simulation.
   */
  void InteractionStep() const;

  /**
   * @brief Advances the physics simulation step.
   * @param physics_parameters Parameters for the physics simulation step.
   */
  void PhysicsStep(const DynamicStrands::PhysicsParameters& physics_parameters) const;

  /**
   * @brief Renders visualizations for dynamic strands.
   * @param target_camera Camera used for visualization.
   * @param visualization_parameters Parameters for visualization rendering.
   */
  void Visualization(const std::shared_ptr<Camera>& target_camera,
                     const DynamicStrandsVisualizationParameters& visualization_parameters) const;

  /**
   * @brief Registers rendering instance for foliage visualization.
   * @param render_parameters Parameters for foliage rendering.
   */
  void RegisterFoliageRenderInstance(const FoliageRenderParameters& render_parameters);

  /**
   * @brief Registers rendering instance for segment pair visualization.
   * @param render_parameters Parameters for segment pair rendering.
   */
  void RegisterSegmentPairRenderInstance(const SegmentPairsRenderParameters& render_parameters);

  struct Region {
    float r_min, r_max;
    float phi_min, phi_max;
    float x_min, x_max;
  };

  enum Axis3 { AX_R, AX_PHI, AX_X, AX_NONE };

  struct Node {
    Axis3 axis;
    float coord;
    Node* left;
    Node* right;
    int leaf_id;

    Node(int id) : axis(AX_NONE), coord(0.0f), left(nullptr), right(nullptr), leaf_id(id) {
    }
  };

  void split_one(const Region& c, float p_min, float p_max, float max_ratio, int max_tries, Region& c1, Region& c2,
                 Axis3& out_axis, float& out_coord, float r_scale_coef, float p_scale_coef);

  Node* build_bsp(const Region& init, int N, float p_half, float max_ratio, int max_tries,
                  std::vector<Region>& out_regions, float r_scale_coef = 150.f, float p_scale_coef = 300.f);

  int classify_point(const std::array<float, 3>& pt, Node* node);

  struct Node_tilt {
    Axis3 axis = AX_NONE;
    float coord = 0.0f;
    Node_tilt* left = nullptr;
    Node_tilt* right = nullptr;
    int leaf_id = -1;
    glm::vec3 p0_n = glm::vec3(0.0f);
    glm::vec3 n_tilt = glm::vec3(0.0f);
    bool has_tilt = false;

    Node_tilt() = default;

    explicit Node_tilt(int id) : leaf_id(id) {
    }
  };

  Node_tilt* build_bsp_tilt(const Region& init, int N, float p_half, float max_ratio, int max_tries, float tilt_eps,
                            bool enable_tilt, std::vector<Region>& out_regions, float r_scale_coef = 150.f,
                            float p_scale_coef = 300.f);

  int classify_point_tilt(const std::array<float, 3>& pt, const Node_tilt* node);

  void delete_tree(Node_tilt* n);

  int classify_point_jitter_axis(const std::array<float, 3>& pt, const Node_tilt* node, float eps_norm,
                                 uint32_t base_seed, bool wrap = true);

  float normalize_coord(Axis3 axis, float coord);
};
}  // namespace eco_sys_lab_plugin