#pragma once

#include "IPrivateComponent.hpp"
#include <cstdint>

namespace l_system_plugin {
using namespace evo_engine;

/**
 * @brief Base class for L-System components, providing shared state and infancy reset.
 */
template <typename TDerived>
class LSystemComponentBase : public IPrivateComponent {
 public:
  /// Reference to the genotype descriptor asset.
  AssetRef descriptor_ref;

  /// Seed for deterministic generation.
  unsigned int seed = 42;

  /// Target GDD used by GrowToTargetGDD.
  float target_gdd = 0.0f;

  /**
   * @brief Resets the component to its infancy stage and clears geometry.
   *
   * Re-seeds if a new seed is provided, assigns the infancy GDD bound,
   * and clears the geometry until explicitly generated.
   */
  void ResetToInfancy(unsigned int optional_new_seed) {
    seed = optional_new_seed;
    target_gdd = static_cast<TDerived*>(this)->GetInfancyTargetGDD();
    static_cast<TDerived*>(this)->ClearGeometryEntities();
  }

  /**
   * @brief Resets the component to its infancy stage without altering the seed and clears geometry.
   */
  void ResetToInfancy() {
    target_gdd = static_cast<TDerived*>(this)->GetInfancyTargetGDD();
    static_cast<TDerived*>(this)->ClearGeometryEntities();
  }
};

}  // namespace l_system_plugin