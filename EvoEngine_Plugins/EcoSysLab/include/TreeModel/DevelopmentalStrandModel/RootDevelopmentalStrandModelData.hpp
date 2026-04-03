#pragma once
#include "RootGrowthData.hpp"

#include <optional>

namespace eco_sys_lab_plugin {
using namespace evo_engine;

/**
 * @brief Unified node data combining root node growth data with optional strand model data.
 *
 * Mirrors DevelopmentalStrandModelNodeData but for the root skeleton.
 */
struct RootDevelopmentalStrandModelNodeData {
  RootNodeGrowthData root_node_data{};
  std::optional<StrandModelNodeData> strand_data{};

  void InitializeStrandData() {
    if (!strand_data.has_value()) {
      strand_data.emplace();
    }
  }

  [[nodiscard]] bool HasStrandData() const {
    return strand_data.has_value();
  }
};

/**
 * @brief Unified flow data combining root stem growth data with optional strand model flow data.
 */
struct RootDevelopmentalStrandModelFlowData {
  RootStemGrowthData stem_data{};
  std::optional<StrandModelFlowData> strand_data{};
};

/**
 * @brief Unified skeleton data combining root growth data with optional strand model skeleton data.
 */
struct RootDevelopmentalStrandModelSkeletonData {
  RootGrowthData root_data{};
  std::optional<StrandModelSkeletonData> strand_data{};

  void InitializeStrandData() {
    if (!strand_data.has_value()) {
      strand_data.emplace();
    }
  }

  [[nodiscard]] bool HasStrandData() const {
    return strand_data.has_value();
  }
};

/**
 * @brief A unified skeleton for root developmental strand model co-evolution.
 */
typedef Skeleton<RootDevelopmentalStrandModelSkeletonData, RootDevelopmentalStrandModelFlowData,
                 RootDevelopmentalStrandModelNodeData>
    RootDevelopmentalStrandModelSkeleton;

}  // namespace eco_sys_lab_plugin
