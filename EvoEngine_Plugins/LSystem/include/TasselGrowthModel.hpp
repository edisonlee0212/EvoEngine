#pragma once

#include "MaizeTasselModules.hpp"
#include "MaizeTasselRules.hpp"
#include "GeometryPass.hpp"
#include <glm/glm.hpp>
#include <cstdint>
#include <random>

namespace l_system_plugin {

class MaizeTasselDescriptor;

/**
 * @brief GDD-driven growth model for a single maize tassel instance.
 *
 * Manages the full lifecycle: sampling genotype distributions, topology
 * derivation, incremental GDD-driven growth, and geometry propagation.
 *
 * Owned by MaizeTassel (not an IAsset).
 */
class TasselGrowthModel {
 public:
  struct GrowthStepProfile {
    double total_seconds = 0.0;
    double apply_growth_rules_seconds = 0.0;
    double apply_topology_rules_seconds = 0.0;
    double sort_lists_seconds = 0.0;
    double update_node_info_seconds = 0.0;
    double propagate_geometry_seconds = 0.0;
    double topology_scan_seconds = 0.0;
  };

  /**
   * @brief Initialize the model from a genotype and seed.
   * Samples all distributions, sets up the axiom, creates rules.
   */
  void Initialize(const MaizeTasselDescriptor& descriptor, unsigned int seed,
                  const glm::vec3& root_position = glm::vec3(0),
                  const glm::quat& root_rotation = kDefaultRootRotation);

  /**
    * @brief Legacy helper: apply topology rules until no structural change occurs.
    * Prefer GrowStep/GrowToGDD for age-driven incremental development.
   */
  void DeriveTopology();

  /**
    * @brief Apply one thermal growth step (age updates + topology + geometry).
   */
  void GrowStep();

  /**
   * @brief Grow until the accumulated GDD reaches target_gdd.
   * @param target_gdd The target GDD value.
    * @param max_growth_steps Optional per-call step budget (0 = unlimited).
   */
    void GrowToGDD(float target_gdd, uint32_t max_growth_steps = 0);

  /**
   * @brief Propagate geometry transforms top-down.
   * Must be called after topology and/or growth changes for valid positions.
   */
  void PropagateGeometry();

  /**
   * @brief Reset the model to uninitialized state.
   */
  void Reset();

  [[nodiscard]] bool IsInitialized() const { return initialized_; }
  /// True when no pending Apex/Lateral symbols remain.
  [[nodiscard]] bool IsTopologyComplete() const { return topology_complete_; }

  // -- Public state --
  float accumulated_gdd = 0.0f;
  float gdd_per_growth_step = 1.0f;
  uint32_t last_growth_steps = 0;
  GrowthStepProfile last_growth_step_profile{};
  GrowthStepProfile last_grow_to_gdd_profile{};
  TasselGraph graph{1};
  SampledTasselParams sampled;

 private:
  std::mt19937 rng_;
  TasselEngine engine_;
  bool initialized_ = false;
  bool topology_complete_ = false;
  int max_topology_steps_ = 0;
  glm::vec3 root_position_{0.0f};
  glm::quat root_rotation_{1, 0, 0, 0};
};

}  // namespace l_system_plugin
