#include "SorghumGrowthModel.hpp"
#include "SorghumLSDescriptor.hpp"
#include <algorithm>
#include <cmath>
#include <glm/gtc/quaternion.hpp>

using namespace l_system_package;

namespace {

// -----------------------------------------------------------------------------
// Leaf maturity stamp — verbatim port of UpdateNeedleMaturityState from
// PineGrowthModel.cpp, retargeted to SorghumLeaf.
// -----------------------------------------------------------------------------
void UpdateLeafMaturityState(SorghumLeaf& leaf,
                             const float thermal_now_years,
                             const float absolute_age_years) {
  if (leaf.maturity_reached) {
    return;
  }

  const float clamped_chrono_age = std::max(0.0f, absolute_age_years);
  if (leaf.continuous_growth.maturation_years <= 0.0f) {
    leaf.maturity_reached = true;
    leaf.chronological_age_at_maturity_years = clamped_chrono_age;
    return;
  }

  constexpr float kMaturityEpsilon = 1.0e-4f;
  const float normalized_age = leaf.continuous_growth.NormalizedAge(thermal_now_years);
  if (normalized_age >= 1.0f - kMaturityEpsilon) {
    leaf.maturity_reached = true;
    leaf.chronological_age_at_maturity_years = clamped_chrono_age;
  }
}

// -----------------------------------------------------------------------------
// Leaf chronological senescence — verbatim port of
// UpdateNeedleChronologicalAging, retargeted to SorghumLeaf
// (browning_years -> wilting_years).
//
// Rank-driven proximal-first turnover analog: in pine the proximal cohorts
// (low s_along_parent_norm) senesce first. For sorghum we mirror exactly
// the same formula — leaves anchored low on the internode (proximal) drop
// first; in our grammar all leaves use s_along_parent_norm = 1.0 by default,
// so proximality contribution is currently 0 and lifespan is governed by
// `lifespan_years` + `node_random` jitter alone. Future placement variants
// can move s_along_parent_norm without changing this code.
// -----------------------------------------------------------------------------
bool UpdateLeafChronologicalAging(SorghumLeaf& leaf,
                                  const float absolute_age_years) {
  const int previous_age_years = leaf.age_years;
  const float previous_senescence = leaf.senescence_phase;
  const bool previous_alive = leaf.alive;

  const float clamped_age_years = std::max(0.0f, absolute_age_years);
  leaf.age_years = std::max(0, static_cast<int>(std::floor(clamped_age_years)));

  const float s_norm = std::clamp(leaf.s_along_parent_norm, 0.0f, 1.0f);
  const float proximality = 1.0f - s_norm;
  const float lifespan_years_f = static_cast<float>(leaf.lifespan_years);
  constexpr float kMaxProximalLifespanReductionYears = 2.0f;
  constexpr float kNodeRandomJitterYears = 0.35f;
  const float lifespan_jitter =
      (leaf.node_random - 0.5f) * 2.0f * kNodeRandomJitterYears;
  const float effective_lifespan_years = std::max(
      0.25f,
      lifespan_years_f - proximality * kMaxProximalLifespanReductionYears +
          lifespan_jitter);

  const float senescence_age_years = leaf.maturity_reached
      ? std::max(
            0.0f,
            clamped_age_years - std::max(0.0f, leaf.chronological_age_at_maturity_years))
      : 0.0f;

  if (leaf.maturity_reached && senescence_age_years > effective_lifespan_years) {
    const float wilting_years = std::max(0.0f, leaf.wilting_years);
    const float wilting_scale = 1.0f - 0.35f * proximality;
    const float effective_wilting_years =
        std::max(0.05f, wilting_years * wilting_scale);
    const float phase =
        (wilting_years > 0.0f)
            ? std::clamp((senescence_age_years - effective_lifespan_years) /
                             effective_wilting_years,
                         0.0f, 1.0f)
            : 1.0f;
    leaf.senescence_phase = phase;
    if (phase >= 1.0f) leaf.alive = false;
  } else {
    leaf.senescence_phase = 0.0f;
  }

  return leaf.age_years != previous_age_years ||
         std::abs(leaf.senescence_phase - previous_senescence) > 1.0e-5f ||
         leaf.alive != previous_alive;
}

// Effective plastochron threshold for an apex of `order`. Duplicated here
// (and in SorghumRules.cpp) so neither TU has to expose internal helpers.
inline float ComputePlastochronGdd(const SampledSorghumParams& params, const int order) {
  const float base = std::max(1.0f, params.plastochron_gdd);
  const float axis_scale = (order >= 1)
      ? std::max(0.1f, params.lateral_axis_plastochron_scale)
      : std::max(0.1f, params.main_axis_plastochron_scale);
  const float ref_maturity = std::max(1.0f, params.reference_maturity_gdd);
  const float maturity_ratio = std::max(0.1f, params.maturity_gdd / ref_maturity);
  const float coupling = std::max(0.0f, params.maturity_initiation_coupling);
  const float maturity_scale = std::max(0.1f, 1.0f + coupling * (maturity_ratio - 1.0f));
  return std::max(1.0f, base * axis_scale * maturity_scale);
}

}  // namespace

// ---------------------------------------------------------------------------
// Initialize
// ---------------------------------------------------------------------------

void SorghumGrowthModel::Initialize(const SorghumLSDescriptor& descriptor,
                                    const unsigned int seed,
                                    const glm::vec3& root_position,
                                    const glm::quat& root_rotation) {
  Reset();

  rng_.seed(seed);
  root_position_ = root_position;
  root_rotation_ = root_rotation;

  // Sample all distributions deterministically against the seeded RNG.
  sampled = descriptor.Sample(rng_);

  // Seed a persistent root node. The main apex and all tiller buds are direct
  // children of this root, so every tiller originates from the plant base.
  graph = SorghumGraph(1);
  const float root_random = SampleUnit01(rng_);
  {
    auto& root = graph.RefNode(0);
    root.symbol_id = SorghumSymbol::Root;
    SorghumRoot root_data;
    root_data.node_random = root_random;
    root.data.Set<SorghumRoot>(root_data);
    root.info.global_position = root_position;
    root.info.global_rotation = root_rotation_;
    root.info.length = 0.0f;
    root.info.thickness = 0.0f;
  }

  SorghumApex apex;
  apex.order = 0;
  apex.vigor = std::max(1, sampled.total_phytomer_count);
  apex.phytomer_count = 0;
  apex.node_random = SampleUnit01(rng_);
  // Phyllotaxis phase carried across phytomers; randomised per plant so
  // identical seeds reproduce identical leaf azimuth offsets.
  apex.phyllotaxis_phase = std::fmod(apex.node_random * 360.0f, 360.0f);
  apex.age_gdd = 0.0f;
  apex.sampled_plastochron_gdd = ComputePlastochronGdd(sampled, /*order=*/0);
  const auto main_apex_handle = graph.Extend(0, false);
  {
    auto& main_apex = graph.RefNode(main_apex_handle);
    main_apex.symbol_id = SorghumSymbol::Apex;
    main_apex.data.Set<SorghumApex>(apex);
    main_apex.info.global_position = root_position;
    main_apex.info.global_rotation = root_rotation_;
    main_apex.info.length = 0.0f;
    main_apex.info.thickness = 0.0f;
  }

  auto tiller_rng = MakeNodeRng(apex.node_random, 0xC8013EA4u);
  const int seeded_tiller_count = std::max(0, sampled.tiller_count);
  const int lateral_phytomers = std::max(1, static_cast<int>(std::round(
      static_cast<float>(sampled.total_phytomer_count) * sampled.tiller_phytomer_count_scale)));
  for (int i = 0; i < seeded_tiller_count; ++i) {
    SorghumTillerBud bud;
    bud.insertion_angle = sampled.tiller_insertion_angle;
    bud.azimuth_offset = ComputeSorghumTillerBudAzimuth(i, sampled.branch_azimuth_offset);
    const float t_bud = (seeded_tiller_count > 1)
        ? static_cast<float>(i) / static_cast<float>(seeded_tiller_count - 1)
        : 0.0f;
    bud.initial_dormancy_gdd = std::max(
        0.0f, SamplePlotted(sampled.tiller_initiation_delay_gdd, t_bud, tiller_rng));
    bud.dormancy_gdd_remaining = bud.initial_dormancy_gdd;
    bud.lateral_phytomer_count = lateral_phytomers;
    bud.lateral_thickness_ratio = sampled.tiller_thickness_ratio;
    bud.node_random = SampleUnit01(tiller_rng);

    const auto bud_handle = graph.Extend(0, true);
    auto& bud_node = graph.RefNode(bud_handle);
    bud_node.symbol_id = SorghumSymbol::TillerBud;
    bud_node.data.Set<SorghumTillerBud>(bud);
    bud_node.info.global_position = root_position;
    bud_node.info.global_rotation = root_rotation_;
    bud_node.info.length = 0.0f;
    bud_node.info.thickness = 0.0f;
  }

  // Install rule lists.
  engine_ = SorghumEngine();
  engine_.topology_rules = CreateSorghumTopologyRules(sampled);
  engine_.growth_rules = CreateSorghumGrowthRules(sampled);

  // Topology budget upper bound: one main culm phytomer cycle + each tiller
  // running its own (scaled) cycle. A wide slack covers leaves + buds +
  // termination passes.
  const int main_phytomers = std::max(1, sampled.total_phytomer_count);
  const int tiller_count = std::max(0, sampled.tiller_count);
  const int tiller_phytomers = std::max(1, static_cast<int>(std::round(
      static_cast<float>(main_phytomers) * sampled.tiller_phytomer_count_scale)));
  max_topology_steps_ =
      std::max(64, (main_phytomers + tiller_count * tiller_phytomers) * 4 + 64);

  initialized_ = true;
  topology_complete_ = false;
}

// ---------------------------------------------------------------------------
// CRTP hooks
// ---------------------------------------------------------------------------

void SorghumGrowthModel::UpdateNodeInfoImpl(LGraphNode<SorghumModuleData>& node) {
  // Keep the SimulationClock (used by ContinuousGrowthState::NormalizedAge)
  // in sync with the thermal year derived from accumulated_gdd.
  const float thermal_now_years = accumulated_gdd / kSorghumGddPerYear;
  if (graph.data.clock.NowYears() != thermal_now_years) {
    graph.data.clock.SetYears(thermal_now_years);
  }

  if (node.data.Is<SorghumInternode>()) {
    auto& internode = node.data.Get<SorghumInternode>();
    // Sync GeometryPass-visible dimensions from G-Internode's interpolated
    // state. G-Internode itself already wrote length/thickness; we just
    // mirror them onto node.info so the geometry pass picks them up.
    node.info.length = internode.length;
    node.info.thickness = std::max(0.00002f, internode.thickness);
    return;
  }

  if (node.data.Is<SorghumLeaf>()) {
    auto& leaf = node.data.Get<SorghumLeaf>();
    const float absolute_age_years =
        std::max(0.0f, node.info.temporal.age_absolute_years);

    // Thermal maturity stamp + chronological senescence — exact parity
    // with PineGrowthModel for needle clusters.
    UpdateLeafMaturityState(leaf, thermal_now_years, absolute_age_years);
    UpdateLeafChronologicalAging(leaf, absolute_age_years);

    // Leaves do not contribute length/thickness to the GeometryPass walk;
    // their geometry is built out-of-band by SorghumLS / SorghumLeafMesh.
    node.info.length = 0.0f;
    node.info.thickness = 0.0f;
    return;
  }

  // Root / Apex / TillerBud / PanicleBud carry no axis geometry.
  node.info.length = 0.0f;
  node.info.thickness = 0.0f;
}

bool SorghumGrowthModel::IsDevelopmentalSymbolImpl(
    const LGraphNode<SorghumModuleData>& node) const {
  // Apex is developmental until vigor is exhausted (and then converted to
  // PanicleBud). TillerBud is developmental until dormancy expires and it
  // becomes an order-1 Apex. PanicleBud is a terminal placeholder.
  return node.data.Is<SorghumApex>() || node.data.Is<SorghumTillerBud>();
}

glm::quat SorghumGrowthModel::ComputeChildLocalRotationImpl(
    const LGraphNode<SorghumModuleData>& node,
    const LGraphNode<SorghumModuleData>& /*parent*/) const {
  if (node.data.Is<SorghumInternode>()) {
    const auto& internode = node.data.Get<SorghumInternode>();

    auto finite_quat = [](const glm::quat& q) {
      return std::isfinite(q.x) && std::isfinite(q.y) &&
             std::isfinite(q.z) && std::isfinite(q.w);
    };

    glm::vec3 bend_axis = internode.bend_axis_local;
    if (!std::isfinite(bend_axis.x) || !std::isfinite(bend_axis.y) ||
        !std::isfinite(bend_axis.z) || glm::dot(bend_axis, bend_axis) < 1e-8f) {
      bend_axis = glm::vec3(1.0f, 0.0f, 0.0f);
    } else {
      bend_axis = glm::normalize(bend_axis);
    }

    float safe_curvature = std::clamp(internode.curvature, -89.0f, 89.0f);
    if (!std::isfinite(safe_curvature)) {
      safe_curvature = 0.0f;
    }

    // roll about parent axis -> pitch (branch_angle) -> bend (curvature).
    // Identical convention to PineGrowthModel.
    glm::quat roll = glm::angleAxis(glm::radians(internode.roll_angle), glm::vec3(0, 0, -1));
    glm::quat pitch = glm::angleAxis(glm::radians(internode.branch_angle), glm::vec3(1, 0, 0));
    glm::quat bend = glm::angleAxis(glm::radians(safe_curvature), bend_axis);
    glm::quat local = glm::normalize(roll * pitch * bend);
    if (!finite_quat(local)) {
      return glm::quat(1, 0, 0, 0);
    }
    return local;
  }
  // Leaves, buds, and apices contribute no axis change.
  return glm::quat(1, 0, 0, 0);
}

glm::vec3 SorghumGrowthModel::ComputeChildGlobalPositionImpl(
    const LGraphNode<SorghumModuleData>& node,
    const LGraphNode<SorghumModuleData>& parent) const {
  return ComputeSorghumChildGlobalPosition(node, parent);
}

float SorghumGrowthModel::ChronologicalGddPerYearImpl() const {
  return kSorghumGddPerYear;
}

bool SorghumGrowthModel::UpdateNodeAgingOnlyImpl(LGraphNode<SorghumModuleData>& node) {
  if (node.data.Is<SorghumLeaf>()) {
    auto& leaf = node.data.Get<SorghumLeaf>();
    const float thermal_now_years = accumulated_gdd / kSorghumGddPerYear;
    UpdateLeafMaturityState(
        leaf, thermal_now_years, node.info.temporal.age_absolute_years);
    return UpdateLeafChronologicalAging(
        leaf, node.info.temporal.age_absolute_years);
  }
  return false;
}

// ---------------------------------------------------------------------------
// Reset
// ---------------------------------------------------------------------------

void SorghumGrowthModel::Reset() {
  ResetBase();
  sampled = SampledSorghumParams();
}
