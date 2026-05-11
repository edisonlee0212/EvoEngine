#pragma once

#include "DerivationEngine.hpp"
#include "LSystemRuleHelpers.hpp"  // SampleDistribution, SamplePlotted, SampleUnit01,
                                   // HashNodeSeed, MakeNodeRng, TropismEntry, SampledTropism.
#include "MaizeTasselModules.hpp"
#include <Plot2D.hpp>
#include <glm/gtc/quaternion.hpp>
#include <algorithm>
#include <cmath>
#include <random>

namespace l_system_plugin {

// Generic helpers (SampleDistribution, SamplePlotted, HashNodeSeed,
// MakeNodeRng, SampleUnit01, TropismEntry, SampledTropism) live in
// LSystemRuleHelpers.hpp so additional species can reuse them. Forward
// declarations for the maize-specific timing helpers stay here because they
// depend on SampledTasselParams, which is defined below.

struct SampledTasselParams;
float ComputeMaturityInitiationScale(const SampledTasselParams& params);
float ComputeInitiationPlastochronGdd(const SampledTasselParams& params,
                                      int order,
                                      bool is_lateral_bud);

// ---------------------------------------------------------------------------
// SampledTasselParams — two-zone architecture.
//
// Branch zone (lower): rachis apex produces internodes + lateral buds.
// Central spike (upper): spike apex produces internodes + spikelet pairs only.
// Lateral branches: apex order>=1 produces spikelets on branch arms.
//
// PlottedDistributions are COPIED from the descriptor and evaluated at runtime.
// ---------------------------------------------------------------------------

struct SampledTasselParams {
  // ===== Branch Zone (lower rachis) =====
  int branch_node_count = 5;                                          ///< Number of rachis nodes that bear laterals.
  evo_engine::PlottedDistribution<float> branch_internode_length;     ///< Rachis internode length in branch zone.
  evo_engine::PlottedDistribution<float> branch_internode_thickness;  ///< Rachis internode thickness in branch zone.
  evo_engine::PlottedDistribution<float> lateral_insertion_angle;     ///< Angle at which laterals depart rachis.
  evo_engine::PlottedDistribution<float> lateral_internode_length;    ///< Internode length on lateral branches.
  evo_engine::PlottedDistribution<float> lateral_node_count;          ///< Number of nodes per lateral branch.
  evo_engine::PlottedDistribution<float> peduncle_branch_probability; ///< Probability of producing laterals on peduncle zone.

  // ===== Central Spike (upper rachis) =====
  int spike_node_count = 8;                                           ///< Number of rachis nodes in the spike.
  evo_engine::PlottedDistribution<float> spike_internode_length;      ///< Internode length in spike zone.
  evo_engine::PlottedDistribution<float> spike_internode_thickness;   ///< Internode thickness in spike zone.
  evo_engine::PlottedDistribution<float> spike_zone_branch_probability; ///< Probability of lateral branching in spike zone.

  // ===== Main-rachis spikelet pair morphology =====
  evo_engine::PlottedDistribution<float> main_pair_proximal_scale_x;
  evo_engine::PlottedDistribution<float> main_pair_proximal_scale_y;
  evo_engine::PlottedDistribution<float> main_pair_proximal_scale_z;
  evo_engine::PlottedDistribution<float> main_pair_proximal_angle;
  evo_engine::PlottedDistribution<float> main_pair_internode_length;
  evo_engine::PlottedDistribution<float> main_pair_internode_thickness;
  evo_engine::PlottedDistribution<float> main_pair_internode_angle;
  evo_engine::PlottedDistribution<float> main_pair_distal_scale_x;
  evo_engine::PlottedDistribution<float> main_pair_distal_scale_y;
  evo_engine::PlottedDistribution<float> main_pair_distal_scale_z;
  evo_engine::PlottedDistribution<float> main_pair_distal_angle;

  // ===== Non-main-axis (peduncle branch) spikelet pair morphology =====
  evo_engine::PlottedDistribution<float> branch_pair_proximal_scale_x;
  evo_engine::PlottedDistribution<float> branch_pair_proximal_scale_y;
  evo_engine::PlottedDistribution<float> branch_pair_proximal_scale_z;
  evo_engine::PlottedDistribution<float> branch_pair_proximal_angle;
  evo_engine::PlottedDistribution<float> branch_pair_internode_length;
  evo_engine::PlottedDistribution<float> branch_pair_internode_thickness;
  evo_engine::PlottedDistribution<float> branch_pair_internode_angle;
  evo_engine::PlottedDistribution<float> branch_pair_distal_scale_x;
  evo_engine::PlottedDistribution<float> branch_pair_distal_scale_y;
  evo_engine::PlottedDistribution<float> branch_pair_distal_scale_z;
  evo_engine::PlottedDistribution<float> branch_pair_distal_angle;

  // ===== Shared =====
  float phyllotaxis_angle = 180.0f;
  evo_engine::SingleDistribution<float> branch_azimuth_offset{0.0f};
  evo_engine::SingleDistribution<float> final_age_gdd{400.0f};

  // Position-dependent thermal modifiers.
  evo_engine::PlottedDistribution<float> lateral_initiation_delay_gdd;
  evo_engine::PlottedDistribution<float> spike_anthesis_offset_gdd;

  // Secondary branch topology controls.
  evo_engine::PlottedDistribution<float> primary_lateral_branch_probability;
  evo_engine::PlottedDistribution<float> secondary_lateral_branch_probability;
  float lateral_thickness_ratio = 0.6f;
  float secondary_insertion_angle = 30.0f;
  float secondary_internode_length = 2.0f;
  float secondary_internode_thickness = 0.1f;
  int secondary_node_count = 1;

  // Growth curves.
  evo_engine::Curve2D rachis_elongation_curve;    ///< For both zones' rachis internodes.
  evo_engine::Curve2D rachis_thickness_curve;     ///< Thickness progression for order-0 internodes.
  evo_engine::Curve2D lateral_elongation_curve;   ///< For lateral branch internodes.
  evo_engine::Curve2D lateral_thickness_curve;    ///< Thickness progression for order>=1 internodes.
  evo_engine::Curve2D lateral_angle_development_curve; ///< Interpolates branch angle over organ age.
  evo_engine::Curve2D pair_proximal_scale_curve;   ///< Controls proximal ellipsoid scale progression.
  evo_engine::Curve2D pair_proximal_angle_curve;   ///< Controls proximal ellipsoid branching angle progression.
  evo_engine::Curve2D pair_internode_length_curve; ///< Controls pair internode length progression.
  evo_engine::Curve2D pair_internode_thickness_curve; ///< Controls pair internode thickness progression.
  evo_engine::Curve2D pair_internode_angle_curve;  ///< Controls pair internode branching angle progression.
  evo_engine::Curve2D pair_distal_scale_curve;     ///< Controls distal ellipsoid scale progression.
  evo_engine::Curve2D pair_distal_angle_curve;     ///< Controls distal ellipsoid branching angle progression.

  // Tropisms.
  std::vector<SampledTropism> tropisms;

  // Thermal timing.
  float base_temperature = 10.0f;
  float plastochron_gdd = 30.0f;
  float anthesis_gdd = 200.0f;
  float maturity_gdd = 400.0f;
  float main_axis_plastochron_scale = 1.0f;
  float lateral_axis_plastochron_scale = 1.0f;
  float lateral_bud_plastochron_scale = 1.0f;
  float maturity_initiation_coupling = 0.0f;
  float reference_maturity_gdd = 400.0f;
  float branch_angle_relaxation = 0.08f;
  float pair_angle_relaxation = 1.0f;
  float stage_1_end_t = 0.167f;
  float stage_2_end_t = 0.50f;
  float stage_3_end_t = 0.85f;
  float secondary_ramp_start_t = 0.167f;
  float secondary_ramp_end_t = 0.50f;
  float mature_droop_start_t = 0.85f;
  float mature_droop_strength = 0.0f;
  float gdd_step = 1.0f;
};

inline float ComputeMaturityInitiationScale(const SampledTasselParams& params) {
  const float ref_maturity = std::max(1.0f, params.reference_maturity_gdd);
  const float maturity_ratio = std::max(0.1f, params.maturity_gdd / ref_maturity);
  const float coupling = std::max(0.0f, params.maturity_initiation_coupling);
  return std::max(0.1f, 1.0f + coupling * (maturity_ratio - 1.0f));
}

inline float ComputeInitiationPlastochronGdd(const SampledTasselParams& params,
                                             const int order,
                                             const bool is_lateral_bud) {
  const float base_plastochron = std::max(1.0f, params.plastochron_gdd);

  float axis_scale = std::max(0.1f, params.main_axis_plastochron_scale);
  if (is_lateral_bud) {
    axis_scale = std::max(0.1f, params.lateral_bud_plastochron_scale);
  } else if (order >= 1) {
    axis_scale = std::max(0.1f, params.lateral_axis_plastochron_scale);
  }

  const float maturity_scale = ComputeMaturityInitiationScale(params);
  return std::max(1.0f, base_plastochron * axis_scale * maturity_scale);
}

inline float Smoothstep01(const float x) {
  const float t = std::clamp(x, 0.0f, 1.0f);
  return t * t * (3.0f - 2.0f * t);
}

inline void ResolveStageThresholds(const SampledTasselParams& params,
                                   float& stage_1_end_t,
                                   float& stage_2_end_t,
                                   float& stage_3_end_t) {
  stage_1_end_t = std::clamp(params.stage_1_end_t, 0.0f, 1.0f);
  stage_2_end_t = std::clamp(params.stage_2_end_t, std::min(1.0f, stage_1_end_t + 0.02f), 1.0f);
  stage_3_end_t = std::clamp(params.stage_3_end_t, std::min(1.0f, stage_2_end_t + 0.02f), 1.0f);
}

inline float EvaluateSecondaryStageRamp(const SampledTasselParams& params, const float age_t) {
  const float ramp_start_t = std::clamp(params.secondary_ramp_start_t, 0.0f, 1.0f);
  const float ramp_end_t = std::clamp(params.secondary_ramp_end_t,
                                      std::min(1.0f, ramp_start_t + 0.02f),
                                      1.0f);
  if (age_t <= ramp_start_t) {
    return 0.0f;
  }
  if (age_t >= ramp_end_t) {
    return 1.0f;
  }
  return Smoothstep01((age_t - ramp_start_t) / std::max(1e-4f, ramp_end_t - ramp_start_t));
}

// ---------------------------------------------------------------------------
// Rule factory
// ---------------------------------------------------------------------------

using TasselRule = ProductionRule<TasselGraph, TasselModuleData>;
using TasselEngine = DerivationEngine<TasselGraphData, TasselFlowData, TasselModuleData>;

/**
 * @brief Create topology rules for the two-zone tassel architecture.
 *
 * Rule 1 — Continuous main-axis extension (Apex order=0, vigor>0):
 *   Apex → Internode + {optional Main-Axis Pair} + [optional Lateral] + Apex(vigor-1)
 *
 * Rule 2 — Main-axis exhaustion (Apex order=0, vigor<=0):
 *   Apex → (removed)
 *
 * Rule 3 — [deprecated] Legacy central-spike extension (SpikeApex, vigor>0):
 *   SpikeApex → Internode + [Sessile] + [Pedicellate] + SpikeApex(vigor-1)
 *
 * Rule 4 — [deprecated] Legacy central-spike exhaustion (SpikeApex, vigor<=0):
 *   SpikeApex → [Sessile] + [Pedicellate]
 *
 * Rule 5 — Lateral/secondary extension (Apex order>=1, vigor>0):
 *   Apex → Internode + [Sessile] + [Pedicellate] + {optional Secondary} + Apex(vigor-1)
 *
 * Rule 6 — Lateral apex exhaustion (Apex order>=1, vigor<=0):
 *   Apex → [Sessile] + [Pedicellate]
 *
 * Rule 7 — Lateral bud expansion (end-node Lateral):
 *   Lateral → Internode + Apex(order, vigor)
 */
inline std::vector<TasselRule> CreateTasselTopologyRules(const SampledTasselParams& params) {
  std::vector<TasselRule> rules;

  auto smoothstep01 = [](float x) {
    const float t = std::clamp(x, 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
  };

  auto lerp = [](const float a, const float b, const float t) {
    const float w = std::clamp(t, 0.0f, 1.0f);
    return a + (b - a) * w;
  };

  auto normalize_local_position = [](const int age, const float vigor) {
    const int estimated_nodes = std::max(1, static_cast<int>(std::round(static_cast<float>(age) + vigor)));
    return static_cast<float>(age) / static_cast<float>(std::max(1, estimated_nodes - 1));
  };

  auto normalize_degrees = [](const float degrees) {
    float wrapped = std::fmod(degrees, 360.0f);
    if (wrapped < 0.0f) {
      wrapped += 360.0f;
    }
    return wrapped;
  };

  auto compute_pair_azimuth = [normalize_degrees, &params](const float axis_phase, const int pair_ordinal) {
    const float base = normalize_degrees(axis_phase);
    const float progression = static_cast<float>(std::max(0, pair_ordinal)) * params.phyllotaxis_angle;
    return normalize_degrees(base + progression);
  };

  auto init_pair = [&params](TasselSpikeletPair& pair,
                             const bool main_rachis_pair,
                             const float t_pos,
                             const float anthesis_offset_gdd,
                             const int pair_ordinal,
                             const float phyllotaxis_azimuth,
                             std::mt19937& rng) {
    const auto& proximal_x = main_rachis_pair ? params.main_pair_proximal_scale_x : params.branch_pair_proximal_scale_x;
    const auto& proximal_y = main_rachis_pair ? params.main_pair_proximal_scale_y : params.branch_pair_proximal_scale_y;
    const auto& proximal_z = main_rachis_pair ? params.main_pair_proximal_scale_z : params.branch_pair_proximal_scale_z;
    const auto& proximal_angle = main_rachis_pair ? params.main_pair_proximal_angle : params.branch_pair_proximal_angle;
    const auto& internode_length = main_rachis_pair ? params.main_pair_internode_length : params.branch_pair_internode_length;
    const auto& internode_thickness =
        main_rachis_pair ? params.main_pair_internode_thickness : params.branch_pair_internode_thickness;
    const auto& internode_angle = main_rachis_pair ? params.main_pair_internode_angle : params.branch_pair_internode_angle;
    const auto& distal_x = main_rachis_pair ? params.main_pair_distal_scale_x : params.branch_pair_distal_scale_x;
    const auto& distal_y = main_rachis_pair ? params.main_pair_distal_scale_y : params.branch_pair_distal_scale_y;
    const auto& distal_z = main_rachis_pair ? params.main_pair_distal_scale_z : params.branch_pair_distal_scale_z;
    const auto& distal_angle = main_rachis_pair ? params.main_pair_distal_angle : params.branch_pair_distal_angle;

    pair.proximal_scale = glm::vec3(0.0f);
    pair.proximal_target_scale = glm::vec3(
        std::max(0.01f, SamplePlotted(proximal_x, t_pos, rng)),
        std::max(0.01f, SamplePlotted(proximal_y, t_pos, rng)),
        std::max(0.01f, SamplePlotted(proximal_z, t_pos, rng)));
    pair.proximal_outward_angle = 0.0f;
    pair.proximal_target_outward_angle = SamplePlotted(proximal_angle, t_pos, rng);

    pair.pair_internode_length = 0.0f;
    pair.pair_internode_target_length = std::max(0.01f, SamplePlotted(internode_length, t_pos, rng));
    pair.pair_internode_thickness = 0.0f;
    pair.pair_internode_target_thickness = std::max(0.005f, SamplePlotted(internode_thickness, t_pos, rng));
    pair.pair_internode_angle = 0.0f;
    pair.pair_internode_target_angle = SamplePlotted(internode_angle, t_pos, rng);

    pair.distal_scale = glm::vec3(0.0f);
    pair.distal_target_scale = glm::vec3(
        std::max(0.01f, SamplePlotted(distal_x, t_pos, rng)),
        std::max(0.01f, SamplePlotted(distal_y, t_pos, rng)),
        std::max(0.01f, SamplePlotted(distal_z, t_pos, rng)));
    pair.distal_outward_angle = 0.0f;
    pair.distal_target_outward_angle = SamplePlotted(distal_angle, t_pos, rng);

    pair.phase = SpikeletPhase::Emerging;
    pair.main_rachis_pair = main_rachis_pair;
    pair.final_age_gdd = std::max(1.0f, SampleDistribution(params.final_age_gdd, rng));
    pair.anthesis_offset_gdd = anthesis_offset_gdd;
    pair.age_gdd = 0.0f;
    pair.pair_ordinal = std::max(0, pair_ordinal);
    pair.phyllotaxis_azimuth = phyllotaxis_azimuth;
    pair.node_random = SampleUnit01(rng);
  };

  // Rule 1: Continuous order-0 main axis with zone-blended geometry and events.
  {
    TasselRule rule;
    rule.predecessor_symbol = TasselSymbol::Apex;
    rule.priority = 0;
    rule.condition = [params](const RuleContext<TasselGraph>& ctx) -> bool {
      const auto& apex = ctx.self.data.Get<TasselApex>();
      const float plastochron_threshold = ComputeInitiationPlastochronGdd(params, apex.order, false);
      return apex.order == 0 && apex.vigor > 0.0f && apex.age_gdd >= plastochron_threshold;
    };
    rule.produce = [params, smoothstep01, lerp, init_pair, compute_pair_azimuth](RuleContext<TasselGraph>& ctx)
        -> ProductionResult<TasselModuleData> {
      const auto& apex = ctx.self.data.Get<TasselApex>();
      auto node_rng = MakeNodeRng(apex.node_random, 0xC8013EA4u);

      const int branch_nodes = std::max(0, params.branch_node_count);
      const int spike_nodes = std::max(0, params.spike_node_count);
      const int total_main_nodes = std::max(1, branch_nodes + spike_nodes);
      const int main_node_index = std::max(0, apex.age);

      const float axis_t = static_cast<float>(main_node_index) /
                           static_cast<float>(std::max(1, total_main_nodes - 1));
      const float branch_t = static_cast<float>(std::min(main_node_index, std::max(0, branch_nodes - 1))) /
                             static_cast<float>(std::max(1, branch_nodes - 1));
      const int spike_node_index = std::max(0, main_node_index - branch_nodes);
      const float spike_t = static_cast<float>(spike_node_index) /
                            static_cast<float>(std::max(1, spike_nodes - 1));

      const float transition_center = static_cast<float>(std::max(0, branch_nodes - 1)) /
                                      static_cast<float>(std::max(1, total_main_nodes - 1));
      const float transition_half_width = std::max(
          0.5f / static_cast<float>(std::max(1, total_main_nodes - 1)),
          1.5f / static_cast<float>(std::max(2, total_main_nodes)));
      const float transition_start = transition_center - transition_half_width;
      const float transition_span = std::max(1e-4f, 2.0f * transition_half_width);
      const float spike_weight = smoothstep01((axis_t - transition_start) / transition_span);

      const float branch_length = std::max(0.01f, SamplePlotted(params.branch_internode_length, branch_t, node_rng));
      const float spike_length = std::max(0.01f, SamplePlotted(params.spike_internode_length, spike_t, node_rng));
      const float main_target_length = std::max(0.01f, lerp(branch_length, spike_length, spike_weight));

      const float branch_thickness =
          std::max(0.01f, SamplePlotted(params.branch_internode_thickness, branch_t, node_rng));
      const float spike_thickness =
          std::max(0.01f, SamplePlotted(params.spike_internode_thickness, spike_t, node_rng));
      const float main_target_thickness = std::max(0.01f, lerp(branch_thickness, spike_thickness, spike_weight));

      const float peduncle_branch_prob =
          std::clamp(SamplePlotted(params.peduncle_branch_probability, branch_t, node_rng), 0.0f, 1.0f);
      const float spike_zone_branch_prob =
          std::clamp(SamplePlotted(params.spike_zone_branch_probability, spike_t, node_rng), 0.0f, 1.0f);
      const float branch_prob = std::clamp(lerp(peduncle_branch_prob, spike_zone_branch_prob, spike_weight), 0.0f, 1.0f);

      ProductionResult<TasselModuleData> result;

      {
        Successor<TasselModuleData> s;
        s.is_branch = false;
        TasselInternode internode;
        internode.target_length = main_target_length;
        internode.target_thickness = main_target_thickness;
        internode.length = 0.0f;
        internode.thickness = 0.0f;
        internode.branch_angle = 0.0f;
        internode.target_branch_angle = 0.0f;
        internode.roll_angle = params.phyllotaxis_angle;
        internode.bend_axis_local = glm::vec3(1.0f, 0.0f, 0.0f);
        internode.order = 0;
        internode.is_spike = spike_weight >= 0.5f;
        internode.node_random = SampleUnit01(node_rng);
        s.data.Set<TasselInternode>(internode);
        s.symbol_id = TasselSymbol::Internode;
        result.successors.push_back(std::move(s));
      }

      const bool emit_main_pair = spike_nodes > 0 && main_node_index >= std::max(0, branch_nodes);
      if (emit_main_pair) {
        Successor<TasselModuleData> s;
        s.is_branch = true;
        TasselSpikeletPair pair;
        const float anthesis_offset = SamplePlotted(params.spike_anthesis_offset_gdd, spike_t, node_rng);
        const int pair_ordinal = std::max(0, spike_node_index);
        const float pair_azimuth = compute_pair_azimuth(apex.phyllotaxis_phase, pair_ordinal);
        init_pair(pair, true, spike_t, anthesis_offset, pair_ordinal, pair_azimuth, node_rng);
        s.data.Set<TasselSpikeletPair>(pair);
        s.symbol_id = TasselSymbol::SpikeletPair;
        result.successors.push_back(std::move(s));
      }

      if (SampleUnit01(node_rng) < branch_prob) {
        const float lateral_vigor = std::max(0.0f, SamplePlotted(params.lateral_node_count, axis_t, node_rng));
        if (lateral_vigor > 0.0f) {
          Successor<TasselModuleData> s;
          s.is_branch = true;
          TasselLateral lateral;
          const float lateral_delay = std::max(0.0f, SamplePlotted(params.lateral_initiation_delay_gdd, axis_t, node_rng));
          lateral.insertion_angle = SamplePlotted(params.lateral_insertion_angle, axis_t, node_rng);
          lateral.azimuth_offset = SampleDistribution(params.branch_azimuth_offset, node_rng);
          lateral.target_length = std::max(0.01f, SamplePlotted(params.lateral_internode_length, axis_t, node_rng));
          lateral.target_thickness =
              std::max(0.01f, main_target_thickness * std::max(0.01f, params.lateral_thickness_ratio));
          lateral.order = 1;
          lateral.lateral_vigor = lateral_vigor;
          lateral.age_gdd = -lateral_delay;
          lateral.node_random = SampleUnit01(node_rng);
          s.data.Set<TasselLateral>(lateral);
          s.symbol_id = TasselSymbol::Lateral;
          result.successors.push_back(std::move(s));
        }
      }

      {
        Successor<TasselModuleData> s;
        s.is_branch = false;
        TasselApex new_apex;
        const float plastochron_threshold = ComputeInitiationPlastochronGdd(params, apex.order, false);
        new_apex.vigor = apex.vigor - 1.0f;
        new_apex.age = apex.age + 1;
        new_apex.order = 0;
        new_apex.age_gdd = std::max(0.0f, apex.age_gdd - plastochron_threshold);
        new_apex.phyllotaxis_phase = apex.phyllotaxis_phase;
        new_apex.node_random = SampleUnit01(node_rng);
        s.data.Set<TasselApex>(new_apex);
        s.symbol_id = TasselSymbol::Apex;
        result.successors.push_back(std::move(s));
      }

      return result;
    };
    rules.push_back(std::move(rule));
  }

  // Rule 2: Main-axis apex exhaustion cleanup.
  {
    TasselRule rule;
    rule.predecessor_symbol = TasselSymbol::Apex;
    rule.priority = 1;
    rule.condition = [params](const RuleContext<TasselGraph>& ctx) -> bool {
      const auto& apex = ctx.self.data.Get<TasselApex>();
      const float plastochron_threshold = ComputeInitiationPlastochronGdd(params, apex.order, false);
      return apex.order == 0 && apex.vigor <= 0.0f && apex.age_gdd >= plastochron_threshold;
    };
    rule.produce = [](RuleContext<TasselGraph>&) -> ProductionResult<TasselModuleData> {
      // Exhausted order-0 apices simply disappear.
      return {};
    };
    rules.push_back(std::move(rule));
  }

  // Rule 3: Spike-zone extension.
  {
    TasselRule rule;
    rule.predecessor_symbol = TasselSymbol::SpikeApex;
    rule.priority = 0;
    rule.condition = [params](const RuleContext<TasselGraph>& ctx) -> bool {
      const auto& apex = ctx.self.data.Get<TasselSpikeApex>();
      const float plastochron_threshold = ComputeInitiationPlastochronGdd(params, 0, false);
      return apex.vigor > 0.0f && apex.age_gdd >= plastochron_threshold;
    };
    rule.produce = [params, init_pair, compute_pair_azimuth](RuleContext<TasselGraph>& ctx) -> ProductionResult<TasselModuleData> {
      const auto& apex = ctx.self.data.Get<TasselSpikeApex>();
      auto node_rng = MakeNodeRng(apex.node_random, 0xAD90777Du);
      const float t_pos = static_cast<float>(apex.age) / static_cast<float>(std::max(1, params.spike_node_count - 1));
      const float branch_prob =
          std::clamp(SamplePlotted(params.spike_zone_branch_probability, t_pos, node_rng), 0.0f, 1.0f);

      ProductionResult<TasselModuleData> result;

      {
        Successor<TasselModuleData> s;
        s.is_branch = false;
        TasselInternode internode;
        internode.target_length = std::max(0.01f, SamplePlotted(params.spike_internode_length, t_pos, node_rng));
        internode.target_thickness = std::max(0.01f, SamplePlotted(params.spike_internode_thickness, t_pos, node_rng));
        internode.length = 0.0f;
        internode.thickness = 0.0f;
        internode.branch_angle = 0.0f;
        internode.target_branch_angle = 0.0f;
        internode.roll_angle = params.phyllotaxis_angle;
        internode.bend_axis_local = glm::vec3(1.0f, 0.0f, 0.0f);
        internode.order = 0;
        internode.is_spike = true;
        internode.node_random = SampleUnit01(node_rng);
        s.data.Set<TasselInternode>(internode);
        s.symbol_id = TasselSymbol::Internode;
        result.successors.push_back(std::move(s));
      }

      {
        Successor<TasselModuleData> s;
        s.is_branch = true;
        TasselSpikeletPair pair;
        const float anthesis_offset = SamplePlotted(params.spike_anthesis_offset_gdd, t_pos, node_rng);
        const int pair_ordinal = std::max(0, apex.age);
        const float pair_azimuth = compute_pair_azimuth(apex.phyllotaxis_phase, pair_ordinal);
        init_pair(pair, true, t_pos, anthesis_offset, pair_ordinal, pair_azimuth, node_rng);
        s.data.Set<TasselSpikeletPair>(pair);
        s.symbol_id = TasselSymbol::SpikeletPair;
        result.successors.push_back(std::move(s));
      }

      if (SampleUnit01(node_rng) < branch_prob) {
        const float lateral_vigor = std::max(0.0f, SamplePlotted(params.lateral_node_count, t_pos, node_rng));
        if (lateral_vigor > 0.0f) {
        Successor<TasselModuleData> s;
        s.is_branch = true;
        TasselLateral lateral;
        const float lateral_delay = std::max(0.0f, SamplePlotted(params.lateral_initiation_delay_gdd, t_pos, node_rng));
        lateral.insertion_angle = SamplePlotted(params.lateral_insertion_angle, t_pos, node_rng);
        lateral.azimuth_offset = SampleDistribution(params.branch_azimuth_offset, node_rng);
        lateral.target_length = std::max(0.01f, SamplePlotted(params.lateral_internode_length, t_pos, node_rng));
        lateral.target_thickness = std::max(
            0.01f,
            SamplePlotted(params.spike_internode_thickness, t_pos, node_rng) * std::max(0.01f, params.lateral_thickness_ratio));
        lateral.order = 1;
        lateral.lateral_vigor = lateral_vigor;
        lateral.age_gdd = -lateral_delay;
        lateral.node_random = SampleUnit01(node_rng);
        s.data.Set<TasselLateral>(lateral);
        s.symbol_id = TasselSymbol::Lateral;
        result.successors.push_back(std::move(s));
        }
      }

      {
        Successor<TasselModuleData> s;
        s.is_branch = false;
        TasselSpikeApex new_apex;
        const float plastochron_threshold = ComputeInitiationPlastochronGdd(params, 0, false);
        new_apex.vigor = apex.vigor - 1.0f;
        new_apex.age = apex.age + 1;
        new_apex.age_gdd = std::max(0.0f, apex.age_gdd - plastochron_threshold);
        new_apex.phyllotaxis_phase = apex.phyllotaxis_phase;
        new_apex.node_random = SampleUnit01(node_rng);
        s.data.Set<TasselSpikeApex>(new_apex);
        s.symbol_id = TasselSymbol::SpikeApex;
        result.successors.push_back(std::move(s));
      }

      return result;
    };
    rules.push_back(std::move(rule));
  }

  // Rule 4: Spike-zone terminal pair.
  {
    TasselRule rule;
    rule.predecessor_symbol = TasselSymbol::SpikeApex;
    rule.priority = 1;
    rule.condition = [params](const RuleContext<TasselGraph>& ctx) -> bool {
      const auto& apex = ctx.self.data.Get<TasselSpikeApex>();
      const float plastochron_threshold = ComputeInitiationPlastochronGdd(params, 0, false);
      return apex.vigor <= 0.0f && apex.age_gdd >= plastochron_threshold;
    };
    rule.produce = [params, init_pair, compute_pair_azimuth](RuleContext<TasselGraph>& ctx) -> ProductionResult<TasselModuleData> {
      ProductionResult<TasselModuleData> result;
      const auto& apex = ctx.self.data.Get<TasselSpikeApex>();
      if (apex.age <= 0) {
        return result;
      }
      auto node_rng = MakeNodeRng(apex.node_random, 0x417B8EF9u);

      Successor<TasselModuleData> s;
      s.is_branch = false;
      TasselSpikeletPair pair;
      const float anthesis_offset = SamplePlotted(params.spike_anthesis_offset_gdd, 1.0f, node_rng);
      const int pair_ordinal = std::max(0, apex.age);
      const float pair_azimuth = compute_pair_azimuth(apex.phyllotaxis_phase, pair_ordinal);
      init_pair(pair, true, 1.0f, anthesis_offset, pair_ordinal, pair_azimuth, node_rng);
      s.data.Set<TasselSpikeletPair>(pair);
      s.symbol_id = TasselSymbol::SpikeletPair;
      result.successors.push_back(std::move(s));
      return result;
    };
    rules.push_back(std::move(rule));
  }

  // Rule 5: Lateral/secondary extension with pair production.
  {
    TasselRule rule;
    rule.predecessor_symbol = TasselSymbol::Apex;
    rule.priority = 0;
    rule.condition = [params](const RuleContext<TasselGraph>& ctx) -> bool {
      const auto& apex = ctx.self.data.Get<TasselApex>();
      const float plastochron_threshold = ComputeInitiationPlastochronGdd(params, apex.order, false);
      return apex.order >= 1 && apex.vigor > 0.0f && apex.age_gdd >= plastochron_threshold;
    };
    rule.produce = [params, normalize_local_position, init_pair, compute_pair_azimuth](RuleContext<TasselGraph>& ctx) -> ProductionResult<TasselModuleData> {
      const auto& apex = ctx.self.data.Get<TasselApex>();
      auto node_rng = MakeNodeRng(apex.node_random, 0xD4E12C77u);
      const bool is_primary = (apex.order == 1);
      const float t_pos = normalize_local_position(apex.age, apex.vigor);

      const float in_length = is_primary
                                  ? std::max(0.01f, SamplePlotted(params.lateral_internode_length, t_pos, node_rng))
                                  : std::max(0.01f, params.secondary_internode_length);
      const float in_thickness = is_primary
                                     ? std::max(0.01f, SamplePlotted(params.branch_internode_thickness, t_pos, node_rng) *
                                                            std::max(0.01f, params.lateral_thickness_ratio))
                                     : std::max(0.01f, params.secondary_internode_thickness);

      ProductionResult<TasselModuleData> result;

      {
        Successor<TasselModuleData> s;
        s.is_branch = false;
        TasselInternode internode;
        internode.target_length = in_length;
        internode.target_thickness = in_thickness;
        internode.length = 0.0f;
        internode.thickness = 0.0f;
        internode.branch_angle = 0.0f;
        internode.target_branch_angle = 0.0f;
        internode.roll_angle = 0.0f;
        internode.bend_axis_local = glm::vec3(1.0f, 0.0f, 0.0f);
        internode.order = apex.order;
        internode.is_spike = false;
        internode.node_random = SampleUnit01(node_rng);
        s.data.Set<TasselInternode>(internode);
        s.symbol_id = TasselSymbol::Internode;
        result.successors.push_back(std::move(s));
      }

      {
        Successor<TasselModuleData> s;
        s.is_branch = true;
        TasselSpikeletPair pair;
        const int pair_ordinal = std::max(0, apex.age);
        const float pair_azimuth = compute_pair_azimuth(apex.phyllotaxis_phase, pair_ordinal);
        init_pair(pair, false, t_pos, 0.0f, pair_ordinal, pair_azimuth, node_rng);
        s.data.Set<TasselSpikeletPair>(pair);
        s.symbol_id = TasselSymbol::SpikeletPair;
        result.successors.push_back(std::move(s));
      }

      const float branch_prob_base =
          is_primary
              ? std::clamp(SamplePlotted(params.primary_lateral_branch_probability, t_pos, node_rng), 0.0f, 1.0f)
              : std::clamp(SamplePlotted(params.secondary_lateral_branch_probability, t_pos, node_rng), 0.0f, 1.0f);
      const float apex_age_t = std::clamp(apex.age_gdd / std::max(1.0f, params.maturity_gdd), 0.0f, 1.0f);
      const float secondary_stage_ramp = EvaluateSecondaryStageRamp(params, apex_age_t);
      const float branch_prob = std::clamp(branch_prob_base * secondary_stage_ramp, 0.0f, 1.0f);
      if (SampleUnit01(node_rng) < branch_prob && params.secondary_node_count > 0) {
        Successor<TasselModuleData> s;
        s.is_branch = true;
        TasselLateral lateral;
        lateral.insertion_angle = params.secondary_insertion_angle;
        lateral.azimuth_offset = SampleDistribution(params.branch_azimuth_offset, node_rng);
        lateral.target_length = params.secondary_internode_length;
        lateral.target_thickness = params.secondary_internode_thickness;
        lateral.order = 2;
        lateral.lateral_vigor = static_cast<float>(params.secondary_node_count);
        lateral.node_random = SampleUnit01(node_rng);
        s.data.Set<TasselLateral>(lateral);
        s.symbol_id = TasselSymbol::Lateral;
        result.successors.push_back(std::move(s));
      }

      {
        Successor<TasselModuleData> s;
        s.is_branch = false;
        TasselApex new_apex;
        const float plastochron_threshold = ComputeInitiationPlastochronGdd(params, apex.order, false);
        new_apex.vigor = apex.vigor - 1.0f;
        new_apex.age = apex.age + 1;
        new_apex.order = apex.order;
        new_apex.age_gdd = std::max(0.0f, apex.age_gdd - plastochron_threshold);
        new_apex.phyllotaxis_phase = apex.phyllotaxis_phase;
        new_apex.node_random = SampleUnit01(node_rng);
        s.data.Set<TasselApex>(new_apex);
        s.symbol_id = TasselSymbol::Apex;
        result.successors.push_back(std::move(s));
      }

      return result;
    };
    rules.push_back(std::move(rule));
  }

  // Rule 6: Lateral terminal pair.
  {
    TasselRule rule;
    rule.predecessor_symbol = TasselSymbol::Apex;
    rule.priority = 1;
    rule.condition = [params](const RuleContext<TasselGraph>& ctx) -> bool {
      const auto& apex = ctx.self.data.Get<TasselApex>();
      const float plastochron_threshold = ComputeInitiationPlastochronGdd(params, apex.order, false);
      return apex.order >= 1 && apex.vigor <= 0.0f && apex.age_gdd >= plastochron_threshold;
    };
    rule.produce = [params, init_pair, compute_pair_azimuth](RuleContext<TasselGraph>& ctx) -> ProductionResult<TasselModuleData> {
      ProductionResult<TasselModuleData> result;
      const auto& apex = ctx.self.data.Get<TasselApex>();
      if (apex.age <= 0) {
        return result;
      }
      auto node_rng = MakeNodeRng(apex.node_random, 0x9E3779B1u);

      Successor<TasselModuleData> s;
      s.is_branch = false;
      TasselSpikeletPair pair;
      const int pair_ordinal = std::max(0, apex.age);
      const float pair_azimuth = compute_pair_azimuth(apex.phyllotaxis_phase, pair_ordinal);
      init_pair(pair, false, 1.0f, 0.0f, pair_ordinal, pair_azimuth, node_rng);
      s.data.Set<TasselSpikeletPair>(pair);
      s.symbol_id = TasselSymbol::SpikeletPair;
      result.successors.push_back(std::move(s));
      return result;
    };
    rules.push_back(std::move(rule));
  }

  // Rule 7: Lateral bud expansion.
  {
    TasselRule rule;
    rule.predecessor_symbol = TasselSymbol::Lateral;
    rule.priority = 0;
    rule.condition = [params](const RuleContext<TasselGraph>& ctx) -> bool {
      const auto& lateral = ctx.self.data.Get<TasselLateral>();
      const float plastochron_threshold = ComputeInitiationPlastochronGdd(params, lateral.order, true);
      return ctx.self.IsEndNode() && lateral.age_gdd >= plastochron_threshold;
    };
    rule.produce = [params, normalize_degrees](RuleContext<TasselGraph>& ctx) -> ProductionResult<TasselModuleData> {
      const auto& lateral = ctx.self.data.Get<TasselLateral>();
      auto node_rng = MakeNodeRng(lateral.node_random, 0xB5297A4Du);

      ProductionResult<TasselModuleData> result;
      if (lateral.lateral_vigor <= 0.0f) {
        return result;
      }

      {
        Successor<TasselModuleData> s;
        s.is_branch = false;
        TasselInternode internode;
        internode.target_length = lateral.target_length;
        internode.target_thickness = lateral.target_thickness;
        internode.length = 0.0f;
        internode.thickness = 0.0f;
        internode.branch_angle = 0.0f;
        internode.target_branch_angle = lateral.insertion_angle;
        internode.roll_angle = lateral.azimuth_offset;
        internode.bend_axis_local = glm::vec3(1.0f, 0.0f, 0.0f);
        internode.order = lateral.order;
        internode.is_spike = false;
        internode.node_random = SampleUnit01(node_rng);
        s.data.Set<TasselInternode>(internode);
        s.symbol_id = TasselSymbol::Internode;
        result.successors.push_back(std::move(s));
      }

      {
        Successor<TasselModuleData> s;
        s.is_branch = false;
        TasselApex new_apex;
        const float plastochron_threshold = ComputeInitiationPlastochronGdd(params, lateral.order, true);
        new_apex.vigor = lateral.lateral_vigor;
        new_apex.age = 0;
        new_apex.order = std::clamp(lateral.order, 1, 2);
        new_apex.age_gdd = std::max(0.0f, lateral.age_gdd - plastochron_threshold);
        new_apex.phyllotaxis_phase = normalize_degrees(lateral.azimuth_offset);
        new_apex.node_random = SampleUnit01(node_rng);
        s.data.Set<TasselApex>(new_apex);
        s.symbol_id = TasselSymbol::Apex;
        result.successors.push_back(std::move(s));
      }

      return result;
    };
    rules.push_back(std::move(rule));
  }

  return rules;
}

/**
 * @brief Create growth rules for tassel maturation.
 */
inline std::vector<TasselRule> CreateTasselGrowthRules(const SampledTasselParams& params) {
  std::vector<TasselRule> rules;

  // Apex thermal aging.
  {
    TasselRule rule;
    rule.predecessor_symbol = TasselSymbol::Apex;
    rule.priority = 0;
    rule.produce = [params](RuleContext<TasselGraph>& ctx) -> ProductionResult<TasselModuleData> {
      ProductionResult<TasselModuleData> result;
      Successor<TasselModuleData> s;
      s.is_branch = false;
      s.symbol_id = TasselSymbol::Apex;
      auto apex = ctx.self.data.Get<TasselApex>();
      apex.age_gdd += params.gdd_step;
      s.data.Set<TasselApex>(apex);
      result.successors.push_back(std::move(s));
      return result;
    };
    rules.push_back(std::move(rule));
  }

  // SpikeApex thermal aging.
  {
    TasselRule rule;
    rule.predecessor_symbol = TasselSymbol::SpikeApex;
    rule.priority = 0;
    rule.produce = [params](RuleContext<TasselGraph>& ctx) -> ProductionResult<TasselModuleData> {
      ProductionResult<TasselModuleData> result;
      Successor<TasselModuleData> s;
      s.is_branch = false;
      s.symbol_id = TasselSymbol::SpikeApex;
      auto apex = ctx.self.data.Get<TasselSpikeApex>();
      apex.age_gdd += params.gdd_step;
      s.data.Set<TasselSpikeApex>(apex);
      result.successors.push_back(std::move(s));
      return result;
    };
    rules.push_back(std::move(rule));
  }

  // Lateral thermal aging.
  {
    TasselRule rule;
    rule.predecessor_symbol = TasselSymbol::Lateral;
    rule.priority = 0;
    rule.produce = [params](RuleContext<TasselGraph>& ctx) -> ProductionResult<TasselModuleData> {
      ProductionResult<TasselModuleData> result;
      Successor<TasselModuleData> s;
      s.is_branch = false;
      s.symbol_id = TasselSymbol::Lateral;
      auto lateral = ctx.self.data.Get<TasselLateral>();
      lateral.age_gdd += params.gdd_step;
      s.data.Set<TasselLateral>(lateral);
      result.successors.push_back(std::move(s));
      return result;
    };
    rules.push_back(std::move(rule));
  }

  // Internode growth: elongation + tropism.
  {
    TasselRule rule;
    rule.predecessor_symbol = TasselSymbol::Internode;
    rule.priority = 0;
    rule.produce = [params](RuleContext<TasselGraph>& ctx) -> ProductionResult<TasselModuleData> {
      ProductionResult<TasselModuleData> result;
      Successor<TasselModuleData> s;
      s.is_branch = false;
      s.symbol_id = TasselSymbol::Internode;

      auto internode = ctx.self.data.Get<TasselInternode>();
      internode.age_gdd += params.gdd_step;

      const float maturity_gdd = std::max(1.0f, params.maturity_gdd);
      const float age_t = std::clamp(internode.age_gdd / maturity_gdd, 0.0f, 1.0f);

      // Use rachis curve for order=0, lateral curve for branches.
      const auto& elongation_curve = (internode.order == 0) ? params.rachis_elongation_curve
                                                            : params.lateral_elongation_curve;
      const auto& thickness_curve = (internode.order == 0) ? params.rachis_thickness_curve
                                                           : params.lateral_thickness_curve;
      internode.growth_progress = std::clamp(elongation_curve.GetValue(age_t), 0.0f, 1.0f);
      internode.length = internode.target_length * internode.growth_progress;
      internode.thickness = internode.target_thickness * std::clamp(thickness_curve.GetValue(age_t), 0.0f, 1.0f);
      if (internode.order >= 1 && !internode.is_spike) {
        const float curve_angle_progress = std::clamp(params.lateral_angle_development_curve.GetValue(age_t), 0.0f, 1.0f);
        float stage_1_end_t = 0.0f;
        float stage_2_end_t = 0.0f;
        float stage_3_end_t = 0.0f;
        ResolveStageThresholds(params, stage_1_end_t, stage_2_end_t, stage_3_end_t);

        float stage_open_limit = 1.0f;
        if (age_t <= stage_1_end_t) {
          stage_open_limit = 0.0f;
        } else if (age_t <= stage_2_end_t) {
          const float span = std::max(1e-4f, stage_2_end_t - stage_1_end_t);
          const float stage_t = (age_t - stage_1_end_t) / span;
          stage_open_limit = 0.08f + 0.12f * Smoothstep01(stage_t);
        } else if (age_t <= stage_3_end_t) {
          const float span = std::max(1e-4f, stage_3_end_t - stage_2_end_t);
          const float stage_t = (age_t - stage_2_end_t) / span;
          stage_open_limit = 0.20f + 0.80f * Smoothstep01(stage_t);
        }

        const float angle_progress = std::min(curve_angle_progress, stage_open_limit);
        const float target_branch_angle = internode.target_branch_angle * angle_progress;

        // Developmental angle follows a damped first-order response to avoid unrealistically fast swing-out.
        const float base_angle_relaxation = std::clamp(params.branch_angle_relaxation, 0.001f, 1.0f);
        const float order_damping = 1.0f / (1.0f + 0.35f * static_cast<float>(std::max(0, internode.order - 1)));
        const float growth_gate = 0.35f + 0.65f * internode.growth_progress;
        float stage_relaxation_multiplier = 0.70f;
        if (age_t <= stage_1_end_t) {
          stage_relaxation_multiplier = 0.35f;
        } else if (age_t <= stage_2_end_t) {
          stage_relaxation_multiplier = 2.20f;
        } else if (age_t <= stage_3_end_t) {
          stage_relaxation_multiplier = 1.00f;
        }
        const float angle_relaxation = std::clamp(
            base_angle_relaxation * order_damping * growth_gate * stage_relaxation_multiplier,
            0.003f,
            0.35f);
        internode.branch_angle = glm::mix(internode.branch_angle, target_branch_angle, angle_relaxation);
      }

      // Tropism bending.
      const float max_order = 2.0f;
      const float order_t = std::clamp(static_cast<float>(internode.order) / max_order, 0.0f, 1.0f);

      // Quasi-static compliance proxy: slender/long internodes bend more than short/thick internodes.
      // This approximates M/(EI) scaling without solving a full beam equilibrium each step.
      const float effective_length = std::max(0.01f, std::max(internode.length, internode.target_length));
      const float effective_radius =
          std::max(0.005f, 0.5f * std::max(internode.thickness, internode.target_thickness));
      const float flexural_rigidity_proxy = std::max(1e-4f, std::pow(effective_radius, 4.0f));
      const float load_compliance =
          std::clamp(0.01f * (effective_length * effective_length * effective_length) / flexural_rigidity_proxy,
                     0.05f,
                     3.5f);

      glm::vec3 growth_dir = ctx.self.info.global_rotation * glm::vec3(0, 0, -1);
      if (!std::isfinite(growth_dir.x) || !std::isfinite(growth_dir.y) ||
          !std::isfinite(growth_dir.z) || glm::dot(growth_dir, growth_dir) < 1e-8f) {
        growth_dir = glm::vec3(0.0f, 0.0f, -1.0f);
      } else {
        growth_dir = glm::normalize(growth_dir);
      }

      auto stable_perpendicular = [](const glm::vec3& direction) {
        const glm::vec3 abs_dir = glm::abs(direction);
        glm::vec3 reference = (abs_dir.x <= abs_dir.y && abs_dir.x <= abs_dir.z)
                                  ? glm::vec3(1.0f, 0.0f, 0.0f)
                                  : ((abs_dir.y <= abs_dir.z)
                                         ? glm::vec3(0.0f, 1.0f, 0.0f)
                                         : glm::vec3(0.0f, 0.0f, 1.0f));
        glm::vec3 axis = glm::cross(direction, reference);
        float axis_len = glm::length(axis);
        if (axis_len < 1e-5f) {
          reference = glm::vec3(1.0f, 0.0f, 0.0f);
          axis = glm::cross(direction, reference);
          axis_len = glm::length(axis);
          if (axis_len < 1e-5f) {
            reference = glm::vec3(0.0f, 1.0f, 0.0f);
            axis = glm::cross(direction, reference);
            axis_len = glm::length(axis);
          }
        }
        return axis_len < 1e-5f ? glm::vec3(1.0f, 0.0f, 0.0f) : axis / axis_len;
      };

      glm::vec3 total_bend_world(0.0f);
      for (const auto& tropism : params.tropisms) {
        const float order_mult = tropism.order_response.mean.GetValue(order_t);
        const float effective_strength = tropism.strength * order_mult;
        if (std::abs(effective_strength) < 0.001f)
          continue;

        const float alignment = std::clamp(glm::dot(growth_dir, tropism.direction), -1.0f, 1.0f);

        glm::vec3 bend_axis_world = glm::cross(growth_dir, tropism.direction);
        float axis_len = glm::length(bend_axis_world);
        if (axis_len < 1e-5f) {
          glm::vec3 previous_axis_world = ctx.self.info.global_rotation * internode.bend_axis_local;
          if (std::isfinite(previous_axis_world.x) && std::isfinite(previous_axis_world.y) &&
              std::isfinite(previous_axis_world.z)) {
            previous_axis_world -= growth_dir * glm::dot(previous_axis_world, growth_dir);
            axis_len = glm::length(previous_axis_world);
            if (axis_len > 1e-5f) {
              bend_axis_world = previous_axis_world / axis_len;
            }
          }
          if (axis_len < 1e-5f) {
            bend_axis_world = stable_perpendicular(growth_dir);
          }
        } else {
          bend_axis_world /= axis_len;
        }

        if (!std::isfinite(bend_axis_world.x) || !std::isfinite(bend_axis_world.y) ||
            !std::isfinite(bend_axis_world.z)) {
          continue;
        }

        const float bend_magnitude = effective_strength * (1.0f - alignment) * load_compliance;
        total_bend_world += bend_axis_world * bend_magnitude;
      }

      if (internode.order >= 1 && !internode.is_spike) {
        const float droop_strength_base = std::max(0.0f, params.mature_droop_strength);
        if (droop_strength_base > 0.0f) {
          const float droop_start_t = std::clamp(params.mature_droop_start_t, 0.0f, 1.0f);
          const float droop_span = std::max(1e-4f, 1.0f - droop_start_t);
          const float droop_t = std::clamp((age_t - droop_start_t) / droop_span, 0.0f, 1.0f);
          if (droop_t > 0.0f) {
            const float droop_progress = Smoothstep01(droop_t);
            const float order_damping =
                1.0f / (1.0f + 0.40f * static_cast<float>(std::max(0, internode.order - 1)));
            const float effective_strength = droop_strength_base * droop_progress * order_damping;

            const glm::vec3 downward(0.0f, -1.0f, 0.0f);
            const float alignment = std::clamp(glm::dot(growth_dir, downward), -1.0f, 1.0f);

            glm::vec3 bend_axis_world = glm::cross(growth_dir, downward);
            float axis_len = glm::length(bend_axis_world);
            if (axis_len < 1e-5f) {
              bend_axis_world = stable_perpendicular(growth_dir);
            } else {
              bend_axis_world /= axis_len;
            }

            if (std::isfinite(bend_axis_world.x) && std::isfinite(bend_axis_world.y) &&
                std::isfinite(bend_axis_world.z)) {
              const float bend_magnitude = effective_strength * (1.0f - alignment) * load_compliance;
              total_bend_world += bend_axis_world * bend_magnitude;
            }
          }
        }
      }

      const float total_curvature = glm::length(total_bend_world);
      constexpr float kMaxStableCurvature = 85.0f;
      const float curvature_relaxation = std::clamp(0.08f * (0.4f + 0.6f * internode.growth_progress), 0.02f, 0.20f);
      const float target_curvature = std::clamp(total_curvature, 0.0f, kMaxStableCurvature);
      internode.curvature = glm::mix(internode.curvature, target_curvature, curvature_relaxation);

      if (total_curvature > 1e-5f) {
        const glm::vec3 bend_axis_world = total_bend_world / total_curvature;
        const glm::vec3 bend_axis_local =
            glm::normalize(glm::conjugate(ctx.self.info.global_rotation) * bend_axis_world);
        if (std::isfinite(bend_axis_local.x) && std::isfinite(bend_axis_local.y) &&
            std::isfinite(bend_axis_local.z) && glm::dot(bend_axis_local, bend_axis_local) > 1e-8f) {
          const glm::vec3 blended_axis = glm::mix(internode.bend_axis_local, bend_axis_local, curvature_relaxation);
          if (std::isfinite(blended_axis.x) && std::isfinite(blended_axis.y) &&
              std::isfinite(blended_axis.z) && glm::dot(blended_axis, blended_axis) > 1e-8f) {
            internode.bend_axis_local = glm::normalize(blended_axis);
          }
        }
      }

      s.data.Set<TasselInternode>(internode);
      result.successors.push_back(std::move(s));
      return result;
    };
    rules.push_back(std::move(rule));
  }

  // Spikelet-pair maturation.
  {
    TasselRule rule;
    rule.predecessor_symbol = TasselSymbol::SpikeletPair;
    rule.priority = 0;
    rule.produce = [params](RuleContext<TasselGraph>& ctx) -> ProductionResult<TasselModuleData> {
      ProductionResult<TasselModuleData> result;
      Successor<TasselModuleData> s;
      s.is_branch = false;
      s.symbol_id = TasselSymbol::SpikeletPair;

      auto pair = ctx.self.data.Get<TasselSpikeletPair>();
      pair.age_gdd += params.gdd_step;

      const float maturity_gdd = std::max(1.0f, pair.final_age_gdd);
      const float anthesis_gdd = std::clamp(
          params.anthesis_gdd + pair.anthesis_offset_gdd,
          0.0f,
          std::max(0.0f, maturity_gdd - 0.001f));
      const float age_t = std::clamp(pair.age_gdd / maturity_gdd, 0.0f, 1.0f);

      const float proximal_scale_progress = std::clamp(params.pair_proximal_scale_curve.GetValue(age_t), 0.0f, 1.0f);
      const float proximal_angle_progress = std::clamp(params.pair_proximal_angle_curve.GetValue(age_t), 0.0f, 1.0f);
      const float internode_length_progress = std::clamp(params.pair_internode_length_curve.GetValue(age_t), 0.0f, 1.0f);
      const float internode_thickness_progress =
          std::clamp(params.pair_internode_thickness_curve.GetValue(age_t), 0.0f, 1.0f);
      const float internode_angle_progress = std::clamp(params.pair_internode_angle_curve.GetValue(age_t), 0.0f, 1.0f);
      const float distal_scale_progress = std::clamp(params.pair_distal_scale_curve.GetValue(age_t), 0.0f, 1.0f);
      const float distal_angle_progress = std::clamp(params.pair_distal_angle_curve.GetValue(age_t), 0.0f, 1.0f);
      const float pair_angle_relaxation = std::clamp(params.pair_angle_relaxation, 0.001f, 1.0f);

      pair.proximal_scale = pair.proximal_target_scale * proximal_scale_progress;
      const float proximal_target_angle = pair.proximal_target_outward_angle * proximal_angle_progress;
      pair.proximal_outward_angle = glm::mix(pair.proximal_outward_angle, proximal_target_angle, pair_angle_relaxation);

      pair.pair_internode_length = pair.pair_internode_target_length * internode_length_progress;
      pair.pair_internode_thickness = pair.pair_internode_target_thickness * internode_thickness_progress;
      const float internode_target_angle = pair.pair_internode_target_angle * internode_angle_progress;
      pair.pair_internode_angle = glm::mix(pair.pair_internode_angle, internode_target_angle, pair_angle_relaxation);

      pair.distal_scale = pair.distal_target_scale * distal_scale_progress;
      const float distal_target_angle = pair.distal_target_outward_angle * distal_angle_progress;
      pair.distal_outward_angle = glm::mix(pair.distal_outward_angle, distal_target_angle, pair_angle_relaxation);

      if (pair.age_gdd >= maturity_gdd) {
        pair.phase = SpikeletPhase::Mature;
      } else if (pair.age_gdd >= anthesis_gdd) {
        pair.phase = SpikeletPhase::Anthesis;
      } else {
        pair.phase = SpikeletPhase::Emerging;
      }

      s.data.Set<TasselSpikeletPair>(pair);
      result.successors.push_back(std::move(s));
      return result;
    };
    rules.push_back(std::move(rule));
  }

  return rules;
}

}  // namespace l_system_plugin
