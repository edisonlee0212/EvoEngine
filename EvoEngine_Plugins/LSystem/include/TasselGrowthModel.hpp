#pragma once

#include "LSystemGrowthModelBase.hpp"
#include "MaizeTasselModules.hpp"
#include "MaizeTasselRules.hpp"
#include <glm/glm.hpp>
#include <cstdint>

namespace l_system_plugin {

class MaizeTasselDescriptor;

/**
 * @brief GDD-driven growth model for a single maize tassel instance.
 *
 * Manages the full lifecycle: sampling genotype distributions, topology
 * derivation, incremental GDD-driven growth, and geometry propagation.
 *
 * Owned by MaizeTassel (not an IAsset).
 *
 * Inherits the generic algorithm skeleton (DeriveTopology, GrowStep,
 * GrowToGDD, PropagateGeometry, profile aggregation, base-state reset) from
 * LSystemGrowthModelBase. Plant-specific behaviour lives in the four hook
 * methods invoked by the base via CRTP plus the plant-specific Initialize.
 */
class TasselGrowthModel
    : public LSystemGrowthModelBase<TasselGrowthModel, TasselGraph, TasselEngine> {
 public:
  using Base = LSystemGrowthModelBase<TasselGrowthModel, TasselGraph, TasselEngine>;

  /**
   * @brief Initialize the model from a genotype and seed.
   * Samples all distributions, sets up the axiom, creates rules.
   */
  void Initialize(const MaizeTasselDescriptor& descriptor, unsigned int seed,
                  const glm::vec3& root_position = glm::vec3(0),
                  const glm::quat& root_rotation = kDefaultRootRotation);

  /**
   * @brief Reset the model to uninitialized state.
   */
  void Reset();

  // ===== CRTP hooks invoked by LSystemGrowthModelBase =====

  /// Sync per-node info (length, thickness) from module data. For tassels,
  /// only TasselInternode contributes; everything else collapses to zero.
  void UpdateNodeInfoImpl(LGraphNode<TasselModuleData>& node);

  /// True iff the module is a still-developing symbol (apex/lateral/spike apex).
  /// Used by the base to decide when topology becomes complete.
  [[nodiscard]] bool IsDevelopmentalSymbolImpl(const LGraphNode<TasselModuleData>& node) const;

  /// Per-node local rotation for the GeometryPass walk.
  [[nodiscard]] glm::quat ComputeChildLocalRotationImpl(
      const LGraphNode<TasselModuleData>& node,
      const LGraphNode<TasselModuleData>& parent) const;

    [[nodiscard]] float ChronologicalGddPerYearImpl() const;

    bool UpdateNodeAgingOnlyImpl(LGraphNode<TasselModuleData>& node);

  // -- Plant-specific public state --
  SampledTasselParams sampled;
};

}  // namespace l_system_plugin
