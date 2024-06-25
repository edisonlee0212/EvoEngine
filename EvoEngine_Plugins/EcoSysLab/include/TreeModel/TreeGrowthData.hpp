#pragma once
#include "EnvironmentGrid.hpp"
#include "Octree.hpp"
#include "ProfileConstraints.hpp"
#include "Skeleton.hpp"
#include "StrandModelParameters.hpp"
#include "TreeOccupancyGrid.hpp"
using namespace evo_engine;
namespace eco_sys_lab {
#pragma region Utilities
enum class BudType { Apical, Lateral, Leaf, Fruit };

enum class BudStatus {
  Dormant,
  Died,
};

struct ReproductiveModule {
  float maturity = 0.0f;
  float health = 1.0f;
  glm::mat4 transform = glm::mat4(0.0f);
  void Reset();
};

class Bud {
 public:
  float flushing_rate;    // No Serialize
  float extinction_rate;  // No Serialize

  BudType type = BudType::Apical;
  BudStatus status = BudStatus::Dormant;

  glm::quat local_rotation = glm::vec3(0.0f);

  //-1.0 means the no fruit.
  ReproductiveModule reproductive_module;
  glm::vec3 marker_direction = glm::vec3(0.0f);  // No Serialize
  size_t marker_count = 0;                       // No Serialize
  float shoot_flux = 0.0f;                       // No Serialize
};

struct ShootFlux {
  float value = 0.0f;
};

struct RootFlux {
  float value = 0.0f;
};

struct TreeVoxelData {
  SkeletonNodeHandle node_handle = -1;
  SkeletonNodeHandle flow_handle = -1;
  unsigned reference_count = 0;
};

#pragma endregion

struct InternodeGrowthData {
  float internode_length = 0.0f;
  int index_of_parent_bud = 0;
  float start_age = 0;
  float finish_age = 0.0f;

  glm::quat desired_local_rotation = glm::vec3(0.0f);
  glm::quat desired_global_rotation = glm::vec3(0.0f);
  glm::vec3 desired_global_position = glm::vec3(0.0f);

  float sagging_stress = 0;
  float sagging_force = 0.f;
  float sagging = 0;
  int order = 0;
  float extra_mass = 0.0f;
  float density = 1.f;
  float strength = 1.f;

  /**
   * List of buds, first one will always be the apical bud which points forward.
   */
  std::vector<Bud> buds;
  std::vector<glm::mat4> leaves;
  std::vector<glm::mat4> fruits;

  int level = 0;                     // No Serialize
  bool max_child = false;             // No Serialize
  float descendant_total_biomass = 0;  // No Serialize
  float biomass = 0;                 // No Serialize
  glm::vec3 desired_descendant_weight_center = glm::vec3(0.f);
  glm::vec3 descendant_weight_center = glm::vec3(0.f);
  float temperature = 0.0f;                       // No Serialize
  float inhibitor_sink = 0;                        // No Serialize
  float light_intensity = 1.0f;                    // No Serialize
  float max_descendant_light_intensity = 0.f;        // No Serialize
  glm::vec3 light_direction = glm::vec3(0, 1, 0);  // No Serialize
  float growth_potential = 0.0f;                   // No Serialize
  float desired_growth_rate = 0.0f;                 // No Serialize
  float growth_rate = 0.0f;                        // No Serialize
  float space_occupancy = 0.0f;
};

struct ShootStemGrowthData {
  int order = 0;
};

struct ShootGrowthData {
  Octree<TreeVoxelData> octree = {};

  size_t max_marker_count = 0;

  std::vector<ReproductiveModule> dropped_leaves;
  std::vector<ReproductiveModule> dropped_fruits;

  glm::vec3 desired_min = glm::vec3(FLT_MAX);
  glm::vec3 desired_max = glm::vec3(FLT_MIN);

  int max_level = 0;
  int max_order = 0;

  unsigned index = 0;
};

typedef Skeleton<ShootGrowthData, ShootStemGrowthData, InternodeGrowthData> ShootSkeleton;

struct StrandModelNodeData {
  StrandModelProfile<CellParticlePhysicsData> profile{};
  std::unordered_map<StrandHandle, ParticleHandle> particle_map{};
  bool boundaries_updated = false;
  ProfileConstraints profile_constraints{};

  float front_control_point_distance = 0.0f;
  float back_control_point_distance = 0.0f;

  float center_direction_radius = 0.0f;

  glm::vec2 offset = glm::vec2(0.0f);
  float twist_angle = 0.0f;
  int packing_iteration = 0;
  bool split = false;

  float strand_radius = 0.002f;
  int strand_count = 0;

  JobHandle job = {};
};

struct StrandModelFlowData {};

struct StrandModelSkeletonData {
  StrandModelStrandGroup strand_group{};
  int num_of_particles = 0;
};

typedef Skeleton<StrandModelSkeletonData, StrandModelFlowData, StrandModelNodeData> StrandModelSkeleton;
}  // namespace eco_sys_lab