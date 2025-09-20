#pragma once
#include "EnvironmentGrid.hpp"
#include "Octree.hpp"
#include "Skeleton.hpp"
using namespace evo_engine;
namespace eco_sys_lab_plugin {

/**
 * @brief A skeleton representing the structure of a root.
 */
struct RootGrowthData {};
struct RootStemGrowthData {};
struct RootNodeGrowthData {};
/**
 * @brief A skeleton representing the structure of a root.
 */
typedef Skeleton<RootGrowthData, RootStemGrowthData, RootNodeGrowthData> RootSkeleton;
}  // namespace eco_sys_lab_plugin
