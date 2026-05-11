#pragma once

#include "LSystemGrowthModelBase.hpp"
#include "ScotsPineModules.hpp"
#include "ScotsPineRules.hpp"
#include <glm/glm.hpp>
#include <vector>

namespace l_system_plugin {

// kPineGddPerYear is defined in ScotsPineDescriptor.hpp (included transitively
// via ScotsPineRules.hpp). Both Rules and GrowthModel need it.

class ScotsPineDescriptor;

/**
 * @brief Annual-cycle growth model for a single Scots pine instance.
 *
 * Time-stepped version: topology and continuous organ growth advance together
 * in GrowStep()/GrowToGDD(). Leader/lateral child emission can be gated by
 * parent internode maturation so internodes visibly elongate before emitting
 * descendants. Inherits the generic algorithm skeleton from
 * LSystemGrowthModelBase via CRTP.
 *
 * Owned by ScotsPine (or by a smoke-test driver in tools/).
 */
class PineGrowthModel
    : public LSystemGrowthModelBase<PineGrowthModel, PineGraph, PineEngine> {
 public:
  using Base = LSystemGrowthModelBase<PineGrowthModel, PineGraph, PineEngine>;

  /**
   * @brief Initialize the model from a genotype and seed.
   * Samples all distributions, sets up the leader axiom, creates rules.
   */
  void Initialize(const ScotsPineDescriptor& descriptor, unsigned int seed,
                  const glm::vec3& root_position = glm::vec3(0),
                                    const glm::quat& root_rotation = kDefaultRootRotation,
                                    const ScotsPineDescriptor* post_repot_descriptor = nullptr,
                                    float repot_switch_gdd = -1.0f);

    /**
     * @brief Grow toward target GDD with optional one-time pre->post profile switch.
     */
    void GrowToGDDWithProfileSwitch(float target_gdd, uint32_t max_growth_steps = 0);

    [[nodiscard]] bool IsPostRepotProfileActive() const {
        return post_repot_profile_active_;
    }

    [[nodiscard]] bool HasPostRepotProfile() const {
        return has_post_repot_profile_;
    }

    [[nodiscard]] float GetRepotSwitchGdd() const {
        return repot_switch_gdd_;
    }

  /**
   * @brief Reset the model to uninitialized state.
   */
  void Reset();

  // ===== CRTP hooks invoked by LSystemGrowthModelBase =====

  /// Sync per-node info (length, thickness) from module data. Only PineInternode
  /// contributes; everything else collapses to zero so geometry pass skips them.
  void UpdateNodeInfoImpl(LGraphNode<PineModuleData>& node);

  /// True iff the module is a still-developing symbol (apex). Used by the base
  /// to decide when topology has converged.
  [[nodiscard]] bool IsDevelopmentalSymbolImpl(const LGraphNode<PineModuleData>& node) const;

  /// Per-node local rotation for the GeometryPass walk.
  [[nodiscard]] glm::quat ComputeChildLocalRotationImpl(
      const LGraphNode<PineModuleData>& node,
      const LGraphNode<PineModuleData>& parent) const;

  [[nodiscard]] float ChronologicalGddPerYearImpl() const;

  bool UpdateNodeAgingOnlyImpl(LGraphNode<PineModuleData>& node);

  // -- Plant-specific public state --
  SampledPineParams sampled;

 private:
    void ApplyActiveSampledProfile(const SampledPineParams& profile_sampled);
    void RefreshEngineRulesForActiveProfile();
    void ActivatePostRepotProfile();

  void RebuildStemLoadCacheIfNeeded();

    SampledPineParams pre_repot_sampled_;
    SampledPineParams post_repot_sampled_;
    bool has_post_repot_profile_ = false;
    bool post_repot_profile_active_ = false;
    float repot_switch_gdd_ = -1.0f;

  int stem_load_cache_graph_version_ = -1;
  std::vector<float> stem_load_cache_;
};

}  // namespace l_system_plugin
