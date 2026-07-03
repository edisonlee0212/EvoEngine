#pragma once

#include "LSystemGraph.hpp"           // kDefaultRootRotation
#include "LSystemGrowthModelBase.hpp"
#include "SorghumModules.hpp"
#include "SorghumRules.hpp"
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

namespace l_system_package {

class SorghumLSDescriptor;

/**
 * @brief Vegetative-stage growth model for one sorghum culm.
 *
 * Mirrors PineGrowthModel exactly:
 *   - GDD-driven topology + organ growth via SorghumRules.
 *   - Chronological clock advanced from accumulated_gdd / kSorghumGddPerYear.
 *   - Leaf maturity stamp + senescence handled in CRTP hooks
 *     (UpdateNodeInfoImpl, UpdateNodeAgingOnlyImpl) using helpers that
 *     mirror UpdateNeedleMaturityState / UpdateNeedleChronologicalAging.
 *
 * Initialize() creates a persistent SorghumRoot, then attaches the main apex
 * and every tiller bud directly under that root. The topology rules only grow
 * existing apices/buds; they do not create new node-origin tillers.
 */
class SorghumGrowthModel
    : public LSystemGrowthModelBase<SorghumGrowthModel, SorghumGraph, SorghumEngine> {
 public:
  using Base = LSystemGrowthModelBase<SorghumGrowthModel, SorghumGraph, SorghumEngine>;

  /// Sample the descriptor, seed the root apex, install rule lists.
  void Initialize(const SorghumLSDescriptor& descriptor,
                  unsigned int seed,
                  const glm::vec3& root_position = glm::vec3(0.0f),
                  const glm::quat& root_rotation = kDefaultRootRotation);

  /// Reset all base + plant-specific state to uninitialized.
  void Reset();

  // ===== CRTP hooks invoked by LSystemGrowthModelBase =====

  void UpdateNodeInfoImpl(LGraphNode<SorghumModuleData>& node);

  [[nodiscard]] bool IsDevelopmentalSymbolImpl(
      const LGraphNode<SorghumModuleData>& node) const;

  [[nodiscard]] glm::quat ComputeChildLocalRotationImpl(
      const LGraphNode<SorghumModuleData>& node,
      const LGraphNode<SorghumModuleData>& parent) const;

  [[nodiscard]] glm::vec3 ComputeChildGlobalPositionImpl(
      const LGraphNode<SorghumModuleData>& node,
      const LGraphNode<SorghumModuleData>& parent) const;

  [[nodiscard]] float ChronologicalGddPerYearImpl() const;

  bool UpdateNodeAgingOnlyImpl(LGraphNode<SorghumModuleData>& node);

  // -- Plant-specific public state --
  SampledSorghumParams sampled;
};

}  // namespace l_system_package
