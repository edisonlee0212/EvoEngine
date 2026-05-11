#pragma once

// Generic CRTP base for L-system growth models. Owns the algorithm skeleton
// (DeriveTopology, GrowStep, GrowToGDD, PropagateGeometry, profile aggregation,
// reset of base state) so per-species growth models only implement the
// plant-specific hooks listed below.
//
// Phase 2.2 of the L-system consolidation work; see docs/seam_inventory.md
// for the broader plan.
//
// CRTP usage:
//
//     class TasselGrowthModel
//         : public LSystemGrowthModelBase<TasselGrowthModel, TasselGraph, TasselEngine> {
//      public:
//       // Plant-specific axiom + sampled-state setup. Calls ResetBase() then
//       // populates `graph`, `sampled`, and any plant-only fields. Drives the
//       // descriptor.Sample(rng_) and root-apex creation steps.
//       void Initialize(const MaizeTasselDescriptor& descriptor, ...);
//
//       // Hooks invoked by the base via CRTP:
//       void UpdateNodeInfoImpl(LGraphNode<MD>& node);
//       bool IsDevelopmentalSymbolImpl(const LGraphNode<MD>& node) const;
//       glm::quat ComputeChildLocalRotationImpl(
//           const LGraphNode<MD>& node, const LGraphNode<MD>& parent) const;
//       void ResetExtensionImpl();   // reset plant-specific fields
//     };
//
// Bit-exact-parity contract: the call order in DeriveTopology / GrowStep /
// GrowToGDD / PropagateGeometry below MUST match the original
// TasselGrowthModel::* bodies character-for-character (RNG draws, sort
// invocations, profile timestamps). Validated by tools/lsystem_parity_check.py.

#include "GeometryPass.hpp"
#include <algorithm>
#include <Times.hpp>
#include <cmath>
#include <cstdint>
#include <functional>
#include <glm/gtc/quaternion.hpp>
#include <random>

namespace l_system_plugin {

template <typename Derived, typename TGraph, typename TEngine>
class LSystemGrowthModelBase {
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

  // -- Public state --
  float accumulated_gdd = 0.0f;
  float gdd_per_growth_step = 1.0f;
  uint32_t last_growth_steps = 0;
  GrowthStepProfile last_growth_step_profile{};
  GrowthStepProfile last_grow_to_gdd_profile{};
  TGraph graph{1};

  [[nodiscard]] bool IsInitialized() const { return initialized_; }
  [[nodiscard]] bool IsTopologyComplete() const { return topology_complete_; }

  /// Apply topology rules until no structural change occurs. Generic.
  void DeriveTopology();

  /// Apply one thermal growth step (age updates + topology + geometry). Generic.
  void GrowStep();

  /// Grow until the accumulated GDD reaches target_gdd. Generic loop with
  /// per-step profile aggregation.
  void GrowToGDD(float target_gdd, uint32_t max_growth_steps = 0);

  /// Run age-only updates using chronological time without applying topology
  /// or growth rules. Returns true if any node changed visible age state.
  bool AgeOnlyStep();

  void SetChronologicalCoupledToThermal(const bool coupled) {
    chronological_coupled_to_thermal_ = coupled;
  }

  [[nodiscard]] bool IsChronologicalCoupledToThermal() const {
    return chronological_coupled_to_thermal_;
  }

  [[nodiscard]] float GetChronologicalYears() const {
    return chronological_years_;
  }

  void SetChronologicalYears(const float years) {
    if (!std::isfinite(years)) return;
    chronological_years_ = std::max(0.0f, years);
  }

  void AdvanceChronologicalYears(const float dt_years) {
    if (!std::isfinite(dt_years) || dt_years <= 0.0f) return;
    chronological_years_ += dt_years;
  }

  /// Propagate geometry transforms top-down using the Derived hook for the
  /// per-node local rotation.
  void PropagateGeometry();

 protected:
  TEngine engine_{};
  std::mt19937 rng_;
  bool initialized_ = false;
  bool topology_complete_ = false;
  int max_topology_steps_ = 0;
  glm::vec3 root_position_{0.0f};
  glm::quat root_rotation_{1, 0, 0, 0};

  /// Reset the base-class state. Derived classes call this from their own
  /// Reset() and then reset their plant-specific fields.
  void ResetBase() {
    graph = TGraph(1);
    engine_ = TEngine();
    accumulated_gdd = 0.0f;
    initialized_ = false;
    topology_complete_ = false;
    max_topology_steps_ = 0;
    chronological_years_ = 0.0f;
    chronological_coupled_to_thermal_ = true;
  }

 private:
  template <typename TNode>
  void RefreshNodeTemporalState(TNode& node) {
    auto& temporal = node.info.temporal;
    if (!temporal.initialized) {
      temporal.initialized = true;
      temporal.birth_thermal_gdd = accumulated_gdd;
      temporal.birth_absolute_years = chronological_years_;
    }
    temporal.age_thermal_gdd = std::max(0.0f, accumulated_gdd - temporal.birth_thermal_gdd);
    temporal.age_absolute_years =
        std::max(0.0f, chronological_years_ - temporal.birth_absolute_years);
  }

  Derived* AsDerived() { return static_cast<Derived*>(this); }
  const Derived* AsDerived() const { return static_cast<const Derived*>(this); }

  float chronological_years_ = 0.0f;
  bool chronological_coupled_to_thermal_ = true;
};

// ---------------------------------------------------------------------------
// DeriveTopology
// ---------------------------------------------------------------------------

template <typename Derived, typename TGraph, typename TEngine>
void LSystemGrowthModelBase<Derived, TGraph, TEngine>::DeriveTopology() {
  if (!initialized_)
    return;

  // Keep deriving until no more topology changes occur.
  for (int i = 0; i < max_topology_steps_; i++) {
    bool changed = engine_.ApplyTopologyRules(graph, rng_);
    if (!changed)
      break;
  }

  // Assign lengths/thicknesses from module data before geometry pass.
  graph.SortLists();
  for (const auto handle : graph.PeekSortedNodeList()) {
    auto& node = graph.RefNode(handle);
    RefreshNodeTemporalState(node);
    AsDerived()->UpdateNodeInfoImpl(node);
  }

  PropagateGeometry();
  topology_complete_ = true;
}

// ---------------------------------------------------------------------------
// GrowStep
// ---------------------------------------------------------------------------

template <typename Derived, typename TGraph, typename TEngine>
void LSystemGrowthModelBase<Derived, TGraph, TEngine>::GrowStep() {
  if (!initialized_)
    return;

  const double step_start = evo_engine::Times::Now();
  last_growth_step_profile = {};
  bool topology_phase_ran = false;
  bool topology_changed = false;

  // Age and interpolate existing modules first.
  double phase_start = evo_engine::Times::Now();
  engine_.ApplyGrowthRules(graph, rng_);
  last_growth_step_profile.apply_growth_rules_seconds = evo_engine::Times::Now() - phase_start;

  // Once topology is complete, growth rules can no longer create topology symbols.
  if (!topology_complete_) {
    topology_phase_ran = true;
    phase_start = evo_engine::Times::Now();
    topology_changed = engine_.ApplyTopologyRules(graph, rng_);
    last_growth_step_profile.apply_topology_rules_seconds = evo_engine::Times::Now() - phase_start;

    phase_start = evo_engine::Times::Now();
    if (topology_changed) {
      graph.SortLists();
    }
    last_growth_step_profile.sort_lists_seconds += evo_engine::Times::Now() - phase_start;
  }

  accumulated_gdd += gdd_per_growth_step;
  if (chronological_coupled_to_thermal_) {
    const float gdd_per_year = std::max(1.0f, AsDerived()->ChronologicalGddPerYearImpl());
    AdvanceChronologicalYears(gdd_per_growth_step / gdd_per_year);
  }

  const auto& sorted_nodes = graph.PeekSortedNodeList();
  phase_start = evo_engine::Times::Now();
  for (const auto handle : sorted_nodes) {
    auto& node = graph.RefNode(handle);
    RefreshNodeTemporalState(node);
    AsDerived()->UpdateNodeInfoImpl(node);
  }
  last_growth_step_profile.update_node_info_seconds = evo_engine::Times::Now() - phase_start;

  phase_start = evo_engine::Times::Now();
  PropagateGeometry();
  last_growth_step_profile.propagate_geometry_seconds = evo_engine::Times::Now() - phase_start;

  if (topology_phase_ran) {
    // Topology is complete when no pending developmental symbols remain.
    phase_start = evo_engine::Times::Now();
    topology_complete_ = true;
    for (const auto handle : sorted_nodes) {
      const auto& node = graph.PeekNode(handle);
      if (AsDerived()->IsDevelopmentalSymbolImpl(node)) {
        topology_complete_ = false;
        break;
      }
    }
    last_growth_step_profile.topology_scan_seconds = evo_engine::Times::Now() - phase_start;
  }

  last_growth_step_profile.total_seconds = evo_engine::Times::Now() - step_start;
}

// ---------------------------------------------------------------------------
// GrowToGDD
// ---------------------------------------------------------------------------

template <typename Derived, typename TGraph, typename TEngine>
void LSystemGrowthModelBase<Derived, TGraph, TEngine>::GrowToGDD(
    const float target_gdd, const uint32_t max_growth_steps) {
  last_growth_steps = 0;
  last_grow_to_gdd_profile = {};
  while (accumulated_gdd < target_gdd &&
         (max_growth_steps == 0 || last_growth_steps < max_growth_steps)) {
    GrowStep();
    last_growth_steps++;

    last_grow_to_gdd_profile.total_seconds += last_growth_step_profile.total_seconds;
    last_grow_to_gdd_profile.apply_growth_rules_seconds +=
        last_growth_step_profile.apply_growth_rules_seconds;
    last_grow_to_gdd_profile.apply_topology_rules_seconds +=
        last_growth_step_profile.apply_topology_rules_seconds;
    last_grow_to_gdd_profile.sort_lists_seconds +=
        last_growth_step_profile.sort_lists_seconds;
    last_grow_to_gdd_profile.update_node_info_seconds +=
        last_growth_step_profile.update_node_info_seconds;
    last_grow_to_gdd_profile.propagate_geometry_seconds +=
        last_growth_step_profile.propagate_geometry_seconds;
    last_grow_to_gdd_profile.topology_scan_seconds +=
        last_growth_step_profile.topology_scan_seconds;
  }
  if (last_growth_steps == 0) {
    last_growth_step_profile = {};
    last_grow_to_gdd_profile = {};
  }
}

// ---------------------------------------------------------------------------
// AgeOnlyStep
// ---------------------------------------------------------------------------

template <typename Derived, typename TGraph, typename TEngine>
bool LSystemGrowthModelBase<Derived, TGraph, TEngine>::AgeOnlyStep() {
  if (!initialized_)
    return false;

  graph.SortLists();
  bool changed = false;
  for (const auto handle : graph.PeekSortedNodeList()) {
    auto& node = graph.RefNode(handle);
    RefreshNodeTemporalState(node);
    changed = AsDerived()->UpdateNodeAgingOnlyImpl(node) || changed;
  }
  return changed;
}

// ---------------------------------------------------------------------------
// PropagateGeometry
// ---------------------------------------------------------------------------

template <typename Derived, typename TGraph, typename TEngine>
void LSystemGrowthModelBase<Derived, TGraph, TEngine>::PropagateGeometry() {
  using NodeT = typename std::decay<decltype(graph.RefNode(0))>::type;
  GeometryPass::Execute(graph, root_position_, root_rotation_,
      std::function<glm::quat(const NodeT&, const NodeT&)>(
        [this](const NodeT& node, const NodeT& parent) -> glm::quat {
          return AsDerived()->ComputeChildLocalRotationImpl(node, parent);
        }));
}

}  // namespace l_system_plugin
