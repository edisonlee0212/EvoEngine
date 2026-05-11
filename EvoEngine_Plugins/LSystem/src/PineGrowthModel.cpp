#include "PineGrowthModel.hpp"
#include "ScotsPineDescriptor.hpp"
#include <algorithm>
#include <cstdint>
#include <cmath>
#include <glm/gtc/quaternion.hpp>

using namespace l_system_plugin;

namespace {
void UpdateNeedleMaturityState(PineNeedleCluster& cluster,
                               const float thermal_now_years,
                               const float absolute_age_years) {
  if (cluster.maturity_reached) {
    return;
  }

  const float clamped_chrono_age = std::max(0.0f, absolute_age_years);
  if (cluster.continuous_growth.maturation_years <= 0.0f) {
    cluster.maturity_reached = true;
    cluster.chronological_age_at_maturity_years = clamped_chrono_age;
    return;
  }

  constexpr float kMaturityEpsilon = 1.0e-4f;
  const float normalized_age = cluster.continuous_growth.NormalizedAge(thermal_now_years);
  if (normalized_age >= 1.0f - kMaturityEpsilon) {
    cluster.maturity_reached = true;
    cluster.chronological_age_at_maturity_years = clamped_chrono_age;
  }
}

bool UpdateWhorlBudChronologicalDormancy(PineWhorlBud& bud,
                     const float absolute_age_years,
                     const int initial_dormancy_years) {
  const int previous_remaining = bud.dormancy_years_remaining;
  const int completed_years = std::max(
    0, static_cast<int>(std::floor(std::max(0.0f, absolute_age_years))));
  const int target_remaining = std::max(0, initial_dormancy_years - completed_years);

  // Keep monotonic countdown even under coarse timestep jumps.
  bud.dormancy_years_remaining =
    std::min(bud.dormancy_years_remaining, target_remaining);

  return bud.dormancy_years_remaining != previous_remaining;
}

bool UpdateNeedleChronologicalAging(PineNeedleCluster& cluster,
                  const float absolute_age_years) {
  const int previous_age_years = cluster.age_years;
  const float previous_senescence = cluster.senescence_phase;
  const bool previous_alive = cluster.alive;

  const float clamped_age_years = std::max(0.0f, absolute_age_years);
  cluster.age_years = std::max(0, static_cast<int>(std::floor(clamped_age_years)));

  // Inner cohorts senesce earlier than distal cohorts, approximating
  // proximal-first canopy turnover in young pines.
  const float s_norm = std::clamp(cluster.s_along_parent_norm, 0.0f, 1.0f);
  const float proximality = 1.0f - s_norm;
  const float lifespan_years_f = static_cast<float>(cluster.lifespan_years);
  constexpr float kMaxProximalLifespanReductionYears = 2.0f;
  constexpr float kNodeRandomJitterYears = 0.35f;
  const float lifespan_jitter =
    (cluster.node_random - 0.5f) * 2.0f * kNodeRandomJitterYears;
  const float effective_lifespan_years = std::max(
    0.25f,
    lifespan_years_f - proximality * kMaxProximalLifespanReductionYears +
      lifespan_jitter);

  const float senescence_age_years = cluster.maturity_reached
      ? std::max(
            0.0f,
            clamped_age_years - std::max(0.0f, cluster.chronological_age_at_maturity_years))
      : 0.0f;

  if (cluster.maturity_reached && senescence_age_years > effective_lifespan_years) {
    const float browning_years = std::max(0.0f, cluster.browning_years);
    const float browning_scale = 1.0f - 0.35f * proximality;
    const float effective_browning_years =
      std::max(0.05f, browning_years * browning_scale);
    const float phase =
      (browning_years > 0.0f)
        ? std::clamp((senescence_age_years - effective_lifespan_years) /
                 effective_browning_years,
               0.0f, 1.0f)
        : 1.0f;
    cluster.senescence_phase = phase;
    if (phase >= 1.0f) cluster.alive = false;
  } else {
    cluster.senescence_phase = 0.0f;
  }

  return cluster.age_years != previous_age_years ||
     std::abs(cluster.senescence_phase - previous_senescence) > 1.0e-5f ||
     cluster.alive != previous_alive;
}

float HashToUnitOpen01(const uint32_t hash) {
  constexpr float kDenominator = 16777217.0f;
  const float u = static_cast<float>(hash & 0x00ffffffu) / kDenominator;
  return std::clamp(u + (1.0f / kDenominator), 1.0e-6f, 1.0f - 1.0e-6f);
}

float DeterministicNormalFromNodeRandom(const float node_random, const uint32_t salt) {
  constexpr float kTwoPi = 6.28318530717958647692f;
  const uint32_t h1 = HashNodeSeed(node_random, salt ^ 0x9e3779b9u);
  const uint32_t h2 = HashNodeSeed(node_random, salt ^ 0x85ebca6bu);
  const float u1 = HashToUnitOpen01(h1);
  const float u2 = HashToUnitOpen01(h2);
  const float radius = std::sqrt(-2.0f * std::log(u1));
  return radius * std::cos(kTwoPi * u2);
}

float EvaluateMaturityMultiplier(const evo_engine::PlottedDistribution<float>& distribution,
                                 const float maturity_age_t,
                                 const float node_random,
                                 const uint32_t salt) {
  const float x = std::clamp(maturity_age_t, 0.0f, 1.0f);
  const float mean_value = distribution.mean.GetValue(x);
  const float deviation_value = std::max(0.0f, distribution.deviation.GetValue(x));
  if (!(deviation_value > 0.0f)) {
    return std::clamp(mean_value, 0.0f, 1.0f);
  }
  const float z = DeterministicNormalFromNodeRandom(node_random, salt);
  return std::clamp(mean_value + deviation_value * z, 0.0f, 1.0f);
}
}  // namespace

void PineGrowthModel::ApplyActiveSampledProfile(
    const SampledPineParams& profile_sampled) {
  sampled = profile_sampled;
  graph.data.gravity_m_s2 = sampled.gravity_m_s2;
  graph.data.needle_radius_to_stem_thickness_max_ratio =
      sampled.needle_radius_to_stem_thickness_max_ratio;
}

void PineGrowthModel::RefreshEngineRulesForActiveProfile() {
  engine_.topology_rules = CreatePineTopologyRules(sampled);
  engine_.growth_rules = CreatePineGrowthRules(sampled);
}

void PineGrowthModel::ActivatePostRepotProfile() {
  if (!has_post_repot_profile_ || post_repot_profile_active_) {
    return;
  }
  ApplyActiveSampledProfile(post_repot_sampled_);
  RefreshEngineRulesForActiveProfile();
  post_repot_profile_active_ = true;
}

void PineGrowthModel::GrowToGDDWithProfileSwitch(const float target_gdd,
                                                 const uint32_t max_growth_steps) {
  if (!has_post_repot_profile_ || repot_switch_gdd_ < 0.0f) {
    Base::GrowToGDD(target_gdd, max_growth_steps);
    return;
  }

  auto accumulate_profile = [](Base::GrowthStepProfile& dst,
                               const Base::GrowthStepProfile& src) {
    dst.total_seconds += src.total_seconds;
    dst.apply_growth_rules_seconds += src.apply_growth_rules_seconds;
    dst.apply_topology_rules_seconds += src.apply_topology_rules_seconds;
    dst.sort_lists_seconds += src.sort_lists_seconds;
    dst.update_node_info_seconds += src.update_node_info_seconds;
    dst.propagate_geometry_seconds += src.propagate_geometry_seconds;
    dst.topology_scan_seconds += src.topology_scan_seconds;
  };

  uint32_t total_steps = 0;
  Base::GrowthStepProfile total_profile{};

  auto run_segment = [&](const float segment_target,
                         const uint32_t segment_cap) {
    Base::GrowToGDD(segment_target, segment_cap);
    total_steps += last_growth_steps;
    accumulate_profile(total_profile, last_grow_to_gdd_profile);
  };

  if (!post_repot_profile_active_ && accumulated_gdd >= repot_switch_gdd_) {
    ActivatePostRepotProfile();
  }

  uint32_t remaining_steps = max_growth_steps;
  if (!post_repot_profile_active_ &&
      accumulated_gdd < repot_switch_gdd_ &&
      target_gdd > repot_switch_gdd_) {
    const uint32_t pre_cap = (max_growth_steps == 0) ? 0u : remaining_steps;
    run_segment(repot_switch_gdd_, pre_cap);

    if (max_growth_steps != 0) {
      if (total_steps >= max_growth_steps) {
        last_growth_steps = total_steps;
        last_grow_to_gdd_profile = total_profile;
        return;
      }
      remaining_steps = max_growth_steps - total_steps;
    }

    if (!post_repot_profile_active_ && accumulated_gdd >= repot_switch_gdd_) {
      ActivatePostRepotProfile();
    }
  }

  const uint32_t final_cap = (max_growth_steps == 0) ? 0u : remaining_steps;
  run_segment(target_gdd, final_cap);

  last_growth_steps = total_steps;
  last_grow_to_gdd_profile = total_profile;
  if (last_growth_steps == 0) {
    last_growth_step_profile = {};
    last_grow_to_gdd_profile = {};
  }
}

void PineGrowthModel::RebuildStemLoadCacheIfNeeded() {
  const int graph_version = graph.GetVersion();
  const auto& raw_nodes = graph.PeekRawNodes();
  if (stem_load_cache_graph_version_ == graph_version &&
      stem_load_cache_.size() == raw_nodes.size()) {
    return;
  }

  stem_load_cache_graph_version_ = graph_version;
  stem_load_cache_.assign(raw_nodes.size(), 0.0f);

  const auto& sorted_nodes = graph.PeekSortedNodeList();
  for (auto it = sorted_nodes.rbegin(); it != sorted_nodes.rend(); ++it) {
    const auto node_handle = *it;
    const auto& node = graph.PeekNode(node_handle);

    // Count structural internodes in the descendant subtree as a lightweight
    // child-load proxy for cambial thickening.
    float subtree_load = node.data.Is<PineInternode>() ? 1.0f : 0.0f;
    for (const auto child_handle : node.PeekChildHandles()) {
      if (child_handle >= 0 &&
          child_handle < static_cast<LNodeHandle>(stem_load_cache_.size())) {
        subtree_load += stem_load_cache_[child_handle];
      }
    }
    stem_load_cache_[node_handle] = subtree_load;
  }
}

// ---------------------------------------------------------------------------
// Initialize
// ---------------------------------------------------------------------------

void PineGrowthModel::Initialize(const ScotsPineDescriptor& descriptor,
                                 const unsigned int seed,
                                 const glm::vec3& root_position,
                                 const glm::quat& root_rotation,
                                 const ScotsPineDescriptor* post_repot_descriptor,
                                 const float repot_switch_gdd) {
  Reset();

  rng_.seed(seed);
  root_position_ = root_position;
  root_rotation_ = root_rotation;

  // Sample all distributions from the descriptor (deterministic, fixed RNG order).
  pre_repot_sampled_ = descriptor.Sample(rng_);
  sampled = pre_repot_sampled_;

  if (post_repot_descriptor && repot_switch_gdd >= 0.0f) {
    std::mt19937 post_rng(seed ^ 0x9e3779b9u);
    post_repot_sampled_ = post_repot_descriptor->Sample(post_rng);
    has_post_repot_profile_ = true;
    post_repot_profile_active_ = false;
    repot_switch_gdd_ = repot_switch_gdd;
  }

  // Sampled once per plant: random initial orientation around +Y.
  const glm::quat sampled_root_yaw = glm::angleAxis(
      glm::radians(sampled.initial_orientation_yaw_deg), glm::vec3(0.0f, 1.0f, 0.0f));
  root_rotation_ = glm::normalize(root_rotation * sampled_root_yaw);

  // Set up the single leader apex.
  graph = PineGraph(1);
  // Phase 4: copy plant-wide gravity into PineGraphData so the mesh builder
  // can apply body forces without having to re-resolve the descriptor.
  ApplyActiveSampledProfile(sampled);
  auto& root = graph.RefNode(0);
  root.symbol_id = PineSymbol::Apex;
  PineApex apex;
  apex.order = 0;
  apex.node_random = SampleUnit01(rng_);
  apex.phyllotaxis_phase = std::fmod(apex.node_random * 360.0f, 360.0f);
  apex.year_index = 0;
  apex.phytomers_this_year = 0;
  // Stamp per-apex resampled scheduling fields used by R0/R1 condition+
  // produce so condition+produce see the same value within a year.
  {
    auto root_node_rng = MakeNodeRng(apex.node_random, 0xA1F00D00u);
    apex.sampled_plastochron_years =
        pine_detail::SamplePlastochronYears(sampled, root_node_rng);
    apex.sampled_max_phytomers_per_seasonal_growth =
        pine_detail::SampleMaxPhytomersPerSeasonalGrowth(sampled, root_node_rng);
    // Year-1 bare-zone count (R0 never fires in year 1 because the leader
    // is constructed with year_index == 0 and the clock starts at year 0).
    apex.bare_phytomers_this_year =
        pine_detail::SampleBarePhytomersThisYear(
            sampled,
            apex.sampled_max_phytomers_per_seasonal_growth,
            root_node_rng);
    apex.previous_season_completion_ratio = 1.0f;
    apex.previous_season_vigor = 1.0f;
  }
  // Allow the leader's first phytomer to emit immediately at t=0; subsequent
  // emissions are plastochron-gated in R1 via this timestamp.
  apex.t_last_emission_years = -std::max(0.0f, apex.sampled_plastochron_years);
  root.data.Set<PineApex>(apex);
  root.info.global_position = root_position;
  root.info.global_rotation = root_rotation_;
  root.info.length = 0.0f;
  root.info.thickness = std::max(0.00005f, sampled.leader_internode_thickness_m);

  // Build the derivation engine with the sampled rule sets.
  engine_ = PineEngine();
  RefreshEngineRulesForActiveProfile();

  // Topology budget for the phytomer grammar:
  //   - leader emits one phytomer per plastochron, capped at
  //     `max_phytomers_per_seasonal_growth` per year.
  //   - each whorl bud (when enabled) activates after `whorl_dormancy_years`
  //     and spawns N lateral apices, which run their own seasonal cycle.
  // Upper bound estimate scales with the per-year cap and a generous slack
  // for needle-cluster successors and chronological dormancy transitions.
  const int branches = std::max(1, sampled.branches_per_whorl);
  const int phytomers_per_year = std::max(1, sampled.max_phytomers_per_seasonal_growth);
  max_topology_steps_ =
      std::max(64, phytomers_per_year * 32 * (1 + branches * 16) + 512);

  initialized_ = true;
  topology_complete_ = false;
}

// ---------------------------------------------------------------------------
// CRTP hooks invoked by LSystemGrowthModelBase
// ---------------------------------------------------------------------------

void PineGrowthModel::UpdateNodeInfoImpl(LGraphNode<PineModuleData>& node) {
  // Keep the thermal (GDD-derived) physiological clock for topology and
  // continuous-growth curves. Chronological age is tracked separately in
  // node.info.temporal and is used for aging/senescence.
  const float thermal_now_years = accumulated_gdd / kPineGddPerYear;
  if (graph.data.clock.NowYears() != thermal_now_years) {
    graph.data.clock.SetYears(thermal_now_years);
  }

  if (node.data.Is<PineInternode>()) {
    RebuildStemLoadCacheIfNeeded();

    auto& internode = node.data.Get<PineInternode>();
    const float absolute_age_years =
        std::max(0.0f, node.info.temporal.age_absolute_years);
    internode.age_years =
        std::max(0, static_cast<int>(std::floor(absolute_age_years)));

    float stem_load_scale = 1.0f;
    if (internode.order == 0) {
      const auto node_handle = node.GetHandle();
      if (node_handle >= 0 &&
          node_handle < static_cast<LNodeHandle>(stem_load_cache_.size())) {
        const float subtree_internode_count =
            std::max(1.0f, stem_load_cache_[node_handle]);
        // Area ~ load surrogate => diameter ~ load^0.5; use a softer
        // exponent to avoid over-thickening in dense canopies.
        stem_load_scale = std::clamp(std::pow(subtree_internode_count, 0.20f), 1.0f, 4.0f);
      }
    }

    // Thermal maturity age remains GDD-driven, but organ size multipliers are
    // now sampled from maturity plot2D distributions (mean/deviation) with a
    // fixed per-organ realization.
    const float internode_maturity_age_t = std::clamp(
        internode.continuous_growth.NormalizedAge(thermal_now_years), 0.0f, 1.0f);
    const float internode_length_maturity = EvaluateMaturityMultiplier(
        sampled.distributions.internode_length_maturity_curve,
        internode_maturity_age_t,
        internode.node_random,
        0x31A1C001u);
    const float internode_width_maturity = EvaluateMaturityMultiplier(
        sampled.distributions.internode_width_maturity_curve,
        internode_maturity_age_t,
        internode.node_random,
        0x31A1C002u);

    if (internode.continuous_growth.maturation_years > 0.0f) {
      internode.length = internode.target_length * internode_length_maturity;
      internode.thickness = internode.target_thickness * internode_width_maturity * stem_load_scale;
      internode.growth_progress = std::clamp(internode_length_maturity, 0.0f, 1.0f);
    } else {
      internode.length = internode.target_length;
      internode.thickness = internode.target_thickness * stem_load_scale;
      internode.growth_progress = 1.0f;
    }
    internode.thickness = std::max(0.00002f, internode.thickness);
    node.info.length = internode.length;
    node.info.thickness = internode.thickness;
  } else if (node.data.Is<PineNeedleCluster>()) {
    auto& cluster = node.data.Get<PineNeedleCluster>();
    const float absolute_age_years =
        std::max(0.0f, node.info.temporal.age_absolute_years);
    float needle_length_maturity = 1.0f;
    if (cluster.continuous_growth.maturation_years > 0.0f) {
      const float needle_maturity_age_t = std::clamp(
        cluster.continuous_growth.NormalizedAge(thermal_now_years), 0.0f, 1.0f);
      needle_length_maturity = EvaluateMaturityMultiplier(
        sampled.distributions.needle_length_maturity_curve,
        needle_maturity_age_t,
        cluster.node_random,
        0x31A1C101u);
    }
    cluster.length = cluster.target_length * needle_length_maturity;

    // Development to maturity is thermal; once mature, needles are determinate.
    UpdateNeedleMaturityState(cluster, thermal_now_years, absolute_age_years);
    if (cluster.maturity_reached) {
      cluster.length = cluster.target_length;
    }

    UpdateNeedleChronologicalAging(cluster, absolute_age_years);

    // Needle clusters do not contribute to flow length / thickness in the
    // GeometryPass walk; their geometry is built out-of-band by ScotsPine.
    node.info.length = 0.0f;
    node.info.thickness = 0.0f;
  } else if (node.data.Is<PineWhorlBud>()) {
    auto& bud = node.data.Get<PineWhorlBud>();
    UpdateWhorlBudChronologicalDormancy(
        bud,
        std::max(0.0f, node.info.temporal.age_absolute_years),
      std::max(0, bud.initial_dormancy_years));
    node.info.length = 0.0f;
    node.info.thickness = 0.0f;
  } else {
    node.info.length = 0.0f;
    node.info.thickness = 0.0f;
  }
}

bool PineGrowthModel::IsDevelopmentalSymbolImpl(const LGraphNode<PineModuleData>& node) const {
  // Apex symbols (leader + laterals) are still developing until they exhaust
  // vigor or reach max_age. WhorlBud is unused in Phase 3 but counted here
  // for forward compatibility.
  return node.data.Is<PineApex>() || node.data.Is<PineWhorlBud>();
}

glm::quat PineGrowthModel::ComputeChildLocalRotationImpl(
    const LGraphNode<PineModuleData>& node,
    const LGraphNode<PineModuleData>& /*parent*/) const {
  if (node.data.Is<PineInternode>()) {
    const auto& internode = node.data.Get<PineInternode>();

    auto finite_quat = [](const glm::quat& q) {
      return std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w);
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

    glm::quat roll = glm::angleAxis(glm::radians(internode.roll_angle), glm::vec3(0, 0, -1));
    glm::quat pitch = glm::angleAxis(glm::radians(internode.branch_angle), glm::vec3(1, 0, 0));
    glm::quat bend = glm::angleAxis(glm::radians(safe_curvature), bend_axis);
    glm::quat local = glm::normalize(roll * pitch * bend);
    if (!finite_quat(local)) {
      return glm::quat(1, 0, 0, 0);
    }
    return local;
  }
  // Needle clusters, whorl buds, and apices contribute no axis change.
  return glm::quat(1, 0, 0, 0);
}

float PineGrowthModel::ChronologicalGddPerYearImpl() const {
  return kPineGddPerYear;
}

bool PineGrowthModel::UpdateNodeAgingOnlyImpl(LGraphNode<PineModuleData>& node) {
  if (node.data.Is<PineInternode>()) {
    auto& internode = node.data.Get<PineInternode>();
    const int previous_age_years = internode.age_years;
    internode.age_years = std::max(
        0, static_cast<int>(std::floor(node.info.temporal.age_absolute_years)));
    return internode.age_years != previous_age_years;
  }

  if (node.data.Is<PineNeedleCluster>()) {
    auto& cluster = node.data.Get<PineNeedleCluster>();
    const float thermal_now_years = accumulated_gdd / kPineGddPerYear;
    UpdateNeedleMaturityState(
        cluster, thermal_now_years, node.info.temporal.age_absolute_years);
    return UpdateNeedleChronologicalAging(
        cluster, node.info.temporal.age_absolute_years);
  }

  if (node.data.Is<PineWhorlBud>()) {
    auto& bud = node.data.Get<PineWhorlBud>();
    return UpdateWhorlBudChronologicalDormancy(
      bud,
      node.info.temporal.age_absolute_years,
      std::max(0, bud.initial_dormancy_years));
  }

  return false;
}

// ---------------------------------------------------------------------------
// Reset
// ---------------------------------------------------------------------------

void PineGrowthModel::Reset() {
  ResetBase();
  sampled = SampledPineParams();
  pre_repot_sampled_ = SampledPineParams();
  post_repot_sampled_ = SampledPineParams();
  has_post_repot_profile_ = false;
  post_repot_profile_active_ = false;
  repot_switch_gdd_ = -1.0f;
  stem_load_cache_graph_version_ = -1;
  stem_load_cache_.clear();
}
