#pragma once
#include "EnvironmentGrid.hpp"
#include "Octree.hpp"
#include "Skeleton.hpp"

namespace eco_sys_lab_package {
using namespace evo_engine;

/**
 * @brief A skeleton representing the structure of a root.
 */
struct RootGrowthData {
  glm::vec3 desired_min = glm::vec3(FLT_MAX);  ///< Minimum desired bounds.
  glm::vec3 desired_max = glm::vec3(FLT_MIN);  ///< Maximum desired bounds.

  unsigned entity_index = 0;  ///< Index used for identification.
  float age = 0;              ///< Age of the tree in years.

  glm::vec3 gravity_direction = glm::vec3(0, -1, 0);
};

struct RootStemGrowthData {};

struct RootNodeGrowthData {
  glm::quat desired_local_rotation = glm::vec3(0.0f);   ///< Desired local rotation.
  glm::quat desired_global_rotation = glm::vec3(0.0f);  ///< Desired global rotation.
  glm::vec3 desired_global_position = glm::vec3(0.0f);  ///< Desired global position.

  float growth_potential = 0.0f;     ///< Growth potential factor (not serialized).
  float desired_growth_rate = 0.0f;  ///< Desired rate of growth (not serialized).
  float growth_rate = 0.0f;          ///< Actual growth rate (not serialized).
  float space_occupancy = 0.0f;      ///< Space occupied by this internode.

  float water = 0.f;
  float nutrient = 0.f;
  float soil_density = 0.f;
  float node_thickness = 0.f;
  float node_length = 0.f;
  float start_age = 0;          ///< Age at which growth starts.
  float finish_age = 0.0f;      ///< Age at which growth finishes.
  float inhibitor_sink = 0.0f;  ///< Inhibitor sink value (not serialized).

  float horizontal_tropism;
  float vertical_tropism;
};

/**
 * @brief A skeleton representing the structure of a root.
 */
typedef Skeleton<RootGrowthData, RootStemGrowthData, RootNodeGrowthData> RootSkeleton;
}  // namespace eco_sys_lab_package