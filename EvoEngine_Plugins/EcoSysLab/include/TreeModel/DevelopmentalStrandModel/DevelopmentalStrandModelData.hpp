#pragma once
#include "ShootGrowthData.hpp"

#include <optional>

namespace eco_sys_lab_plugin {
using namespace evo_engine;

/**
 * @brief Unified node data combining internode growth data with optional strand model data.
 *
 * Uses composition: InternodeGrowthData is always present (drives skeleton growth),
 * while StrandModelNodeData is lazily initialized only when the procedural strand model
 * is enabled. This avoids sync issues between two separate skeletons and enables
 * co-evolution of growth and strand topology.
 */
struct DevelopmentalStrandModelNodeData {
  InternodeGrowthData internode_data{};
  std::optional<StrandModelNodeData> strand_data{};

  /**
   * @brief Initializes strand data if not already present.
   */
  void InitializeStrandData() {
    if (!strand_data.has_value()) {
      strand_data.emplace();
    }
  }

  /**
   * @brief Returns true if strand data has been initialized.
   */
  [[nodiscard]] bool HasStrandData() const {
    return strand_data.has_value();
  }
};

/**
 * @brief Unified flow data combining shoot stem growth data with optional strand model flow data.
 */
struct DevelopmentalStrandModelFlowData {
  ShootStemGrowthData stem_data{};
  std::optional<StrandModelFlowData> strand_data{};
};

/**
 * @brief Unified skeleton data combining shoot growth data with optional strand model skeleton data.
 *
 * ShootGrowthData (octree, markers, dropped organs, etc.) is always present.
 * StrandModelSkeletonData (strand group, particle count) is lazily initialized
 * when the procedural strand model is enabled.
 */
struct DevelopmentalStrandModelSkeletonData {
  ShootGrowthData shoot_data{};
  std::optional<StrandModelSkeletonData> strand_data{};

  /**
   * @brief Initializes strand skeleton data if not already present.
   */
  void InitializeStrandData() {
    if (!strand_data.has_value()) {
      strand_data.emplace();
    }
  }

  /**
   * @brief Returns true if strand skeleton data has been initialized.
   */
  [[nodiscard]] bool HasStrandData() const {
    return strand_data.has_value();
  }
};

/**
 * @brief A unified skeleton that can drive both shoot growth and strand model co-evolution.
 *
 * When the procedural strand model is disabled, this behaves identically to a ShootSkeleton —
 * strand fields remain uninitialized (std::nullopt) with zero overhead beyond the optional wrappers.
 * When enabled, strand data is lazily initialized on nodes as they are created or when
 * incremental strand growth begins.
 */
typedef Skeleton<DevelopmentalStrandModelSkeletonData, DevelopmentalStrandModelFlowData, DevelopmentalStrandModelNodeData>
    DevelopmentalStrandModelSkeleton;

}  // namespace eco_sys_lab_plugin
