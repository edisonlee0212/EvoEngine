#pragma once
#include "EnvironmentGrid.hpp"
#include "Octree.hpp"
#include "ProfileConstraints.hpp"
#include "Skeleton.hpp"
#include "StrandModelParameters.hpp"
#include "TreeOccupancyGrid.hpp"

namespace eco_sys_lab_plugin {
using namespace evo_engine;

#pragma region Utilities

/**
 * @brief Enumeration of different types of buds.
 */
enum class BudType {
  Apical,   ///< Represents an apical bud.
  Lateral,  ///< Represents a lateral bud.
};

/**
 * @brief Enumeration of bud statuses.
 */
enum class OrganStatus {
  Dormant,  ///< The organ is inactive.
  Flushed,  ///< The organ is active.
  Inactive  ///< The organ is dead.
};

/**
 * @brief Represents a module responsible for reproductive processes in tree structures.
 */
struct ShootOrgan {
  float maturity = 0.0f;                 ///< The maturity level of the module.
  float health = 1.0f;                   ///< The health state of the module.
  glm::quat rotation = glm::vec3(0.0f);  ///< The transformation matrix.
  glm::vec3 position = glm::vec3(0.0f);

  glm::vec3 scale = glm::vec3(1.0f);
  OrganStatus status = OrganStatus::Inactive;

  float carbohydrate_source = 0.0f;
  float carbohydrate_sink = 0.0f;
  float carbohydrate_storage = 0.0f;
  // Persistent sink override used for long-term pruning sink dominance.
  bool force_sink = false;
  float force_sink_multiplier = 1.0f;

  /**
   * @brief Resets the reproductive module to its initial state.
   */
  virtual void Reset();
};

struct Leaf : ShootOrgan {
  float activation_temperature = 0;
  float activation_light_intensity = 0;
  float hang_time = 0;
  float growth_rate = 0;
  float leaf_stem_length = 0.f;
  float damage_rate = 0;
  float damage_temperature = 0;
  glm::vec3 position_offset = glm::vec3(0.0f);

  /// Senescence progress [0,1]: 0 = healthy green, 1.0 = yellow.
  /// Driven by climate (low temp / short daylight) and used for visual tinting.
  float senescence = 0.0f;

  /// Index into BasicFoliageDescriptor::leaf_material_variants. Assigned at formulation, persistent for lifetime.
  uint32_t mesh_index = 0;

  /// Index of this leaf among siblings on the same internode. Set during formulation.
  uint32_t leaf_index = 0;
  /// Total number of sibling leaves on the same internode. Set during formulation.
  uint32_t sibling_count = 1;
};

struct Flower : ShootOrgan {
  float activation_temperature = 0;
  float pollination_time = 0;
  float hang_time = 0;
  float growth_rate = 0;
  uint32_t fruit_index;

  float stem_length = 0.f;
  glm::vec3 position_offset = glm::vec3(0.0f);
};

struct Fruit : ShootOrgan {
  float activation_temperature = 0;
  float growth_rate = 0;
  float hang_time = 0;
  float stem_length = 0.f;
  float damage_rate = 0;
  float damage_temperature = 0;
  glm::vec3 position_offset = glm::vec3(0.0f);
};

/**
 * @brief Represents a bud in a procedural tree simulation.
 */
class Bud {
 public:
  BudType type = BudType::Apical;             ///< Type of the bud.
  OrganStatus status = OrganStatus::Dormant;  ///< Current status of the bud.
  int index = 0;
  glm::quat local_rotation = glm::vec3(0.0f);  ///< Local rotation of the bud.

  glm::vec3 marker_direction = glm::vec3(0.0f);  ///< Direction marker for growth simulation (not serialized).
  size_t marker_count = 0;                       ///< Marker count used for tracking growth (not serialized).
  float shoot_flux = 0.0f;                       ///< Value representing the shoot flux (not serialized).
};

/**
 * @brief Structure representing shoot flux values.
 */
struct Vigor {
  float value = 0.0f;  ///< The flux value.
};

/**
 * @brief Structure representing voxel data in tree modeling.
 */
struct TreeVoxelData {
  SkeletonNodeHandle node_handle = -1;  ///< Node handle used in skeleton structure.
  SkeletonNodeHandle flow_handle = -1;  ///< Flow handle for simulation.
  unsigned reference_count = 0;         ///< Reference counter for voxel management.
};

#pragma endregion

/**
 * @brief Data structure representing internode growth properties.
 */
struct InternodeGrowthData {
  float internode_length = 0.0f;  ///< Length of the internode.
  float internode_thickness = 0.0f;
  float start_age = 0;      ///< Age at which growth starts.
  float finish_age = 0.0f;  ///< Age at which growth finishes.

  glm::quat desired_local_rotation = glm::vec3(0.0f);   ///< Desired local rotation.
  glm::quat desired_global_rotation = glm::vec3(0.0f);  ///< Desired global rotation.
  glm::vec3 desired_global_position = glm::vec3(0.0f);  ///< Desired global position.

  float sagging_stress = 0;    ///< Stress due to sagging.
  float sagging_force = 0.0f;  ///< Force influencing sagging.
  float sagging = 0;           ///< Degree of sagging.

  float extra_mass = 0.0f;   ///< Extra mass contribution.
  float density = 1.0f;      ///< Density of the internode.
  float strength = 1.0f;     ///< Strength parameter.
  float shadow_size = 0.0f;  ///< How much shadow does this internode casts.

  /**
   * @brief List of buds associated with this internode.
   *
   * The first bud in the list will always be the apical bud pointing forward.
   */
  std::vector<Bud> buds;

  std::vector<Leaf> leaves;
  std::vector<Flower> flowers;
  std::vector<Fruit> fruits;

  float descendant_total_biomass_factor = 0;  ///< Total biomass factor of descendants.
  float biomass_factor = 0;                   ///< Biomass factor of this internode.

  glm::vec3 desired_descendant_weight_center = glm::vec3(0.0f);  ///< Desired weight center of descendants.
  glm::vec3 descendant_weight_center = glm::vec3(0.0f);          ///< Actual weight center of descendants.

  float temperature = 0.0f;                    ///< Temperature affecting growth (not serialized).
  float inhibitor_sink = 0.0f;                 ///< Inhibitor sink value (not serialized).
  float light_intake = 1.0f;                   ///< Light intake factor (not serialized).
  float descendant_total_light_intake = 0.0f;  ///< Total light intake of descendants (not serialized).

  glm::vec3 light_direction = glm::vec3(0, 1, 0);  ///< Direction of light (not serialized).
  float growth_potential = 0.0f;                   ///< Growth potential factor (not serialized).
  float desired_growth_rate = 0.0f;                ///< Desired rate of growth (not serialized).
  float growth_rate = 0.0f;                        ///< Actual growth rate (not serialized).
  float space_occupancy = 0.0f;                    ///< Space occupied by this internode.

  float carbohydrate_source = 0.0f;
  float carbohydrate_sink = 0.0f;
  float carbohydrate_storage = 0.0f;
  bool force_sink = false;
  float force_sink_multiplier = 1.0f;

  float max_carbohydrate_mass = 0.0f;
  /**
   * @brief The carbohydrate "pressure" or "voltage" at this node.
   * This is the value that will be solved iteratively.
   */
  float carbohydrate_mass = 0.0f;

  /**
   * @brief A temporary buffer to hold the concentration for the next iteration.
   * This is essential for a stable numerical solver.
   */
  float next_concentration = 0.0f;

  /**
   * @brief The transport efficiency (1 / resistance) of this internode.
   * Calculated based on length and thickness (Area / Length).
   */
  float conductance = 0.0f;

  /**
   * @brief The final net gain or loss of carbohydrates from flow.
   * This will be calculated by the solver and used for growth.
   */
  float net_flow_balance = 0.0f;

  // Visualization flag: set true when overflow clamp triggers in pressure-flow solver.
  bool is_overflown = false;
};

/**
 * @brief Represents growth data for an individual shoot stem.
 */
struct ShootStemGrowthData {};

/**
 * @brief Contains growth data for a shoot.
 */
struct ShootGrowthData {
  Octree<TreeVoxelData> octree = {};  ///< Octree structure for voxel data.
  size_t max_marker_count = 0;        ///< Maximum marker count tracked.

  std::vector<Leaf> dropped_leaves;     ///< List of dropped leaves.
  std::vector<Flower> dropped_flowers;  ///< List of dropped fruits.
  std::vector<Fruit> dropped_fruits;    ///< List of dropped fruits.

  glm::vec3 desired_min = glm::vec3(FLT_MAX);  ///< Minimum desired bounds.
  glm::vec3 desired_max = glm::vec3(FLT_MIN);  ///< Maximum desired bounds.

  unsigned entity_index = 0;                          ///< Index used for identification.
  glm::vec3 gravity_direction = glm::vec3(0, -1, 0);  ///< Current direction of gravity;
  float age = 0;                                      ///< Age of the tree in years.
};

/**
 * @brief A skeleton representing the structure of a shoot.
 */
typedef Skeleton<ShootGrowthData, ShootStemGrowthData, InternodeGrowthData> ShootSkeleton;

/**
 * @brief Data representing a node in the strand model simulation.
 */
struct StrandModelNodeData {
  StrandModelProfile<CellParticlePhysicsData> profile{};            ///< Profile defining particle physics properties.
  std::unordered_map<StrandHandle, ParticleHandle> particle_map{};  ///< Mapping of strand handles to particles.
  bool boundaries_updated = false;           ///< Flag indicating if boundaries have been updated.
  ProfileConstraints profile_constraints{};  ///< Profile constraints applied to the strand model.

  float front_control_point_distance = 0.0f;  ///< Distance to the front control point.
  float back_control_point_distance = 0.0f;   ///< Distance to the back control point.

  float center_direction_radius = 0.0f;  ///< Center direction radius for strand positioning.

  glm::vec2 offset = glm::vec2(0.0f);  ///< Offset in the strand model.
  float twist_angle = 0.0f;            ///< Twist angle applied to strands.
  int packing_iteration = 0;           ///< Number of packing iterations performed.
  bool split = false;                  ///< Whether the strand has been split.
  bool is_wound_profile = false;  ///< True if this node is a wound site from pruning (not a natural growth tip).

  StrandHandle wound_cap_strand_handle = -1;                  ///< Handle to the boundary ring strand at wound profiles (-1 = none).
  std::vector<ParticleHandle> wound_cap_ring_particle_handles; ///< Ordered boundary particle handles forming the wound ring.

  float strand_radius = 0.002f;  ///< Radius of the strands.
  int strand_count = 0;          ///< Number of strands.

  JobHandle job = {};  ///< Job handle for multi-threaded simulation.
};

/**
 * @brief Represents flow-related data in the strand model simulation.
 */
struct StrandModelFlowData {};

/**
 * @brief Contains skeleton data for the strand model.
 */
struct StrandModelSkeletonData {
  StrandModelStrandGroup strand_group{};  ///< The strand group associated with the model.
  int num_of_particles = 0;               ///< Number of particles in the strand model.
};

/**
 * @brief A skeleton structure used for strand model simulation.
 */
typedef Skeleton<StrandModelSkeletonData, StrandModelFlowData, StrandModelNodeData> StrandModelSkeleton;
}  // namespace eco_sys_lab_plugin