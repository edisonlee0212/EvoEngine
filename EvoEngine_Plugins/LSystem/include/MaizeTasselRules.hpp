#pragma once

#include "DerivationEngine.hpp"
#include "MaizeTasselModules.hpp"
#include <Plot2D.hpp>
#include <glm/gtc/quaternion.hpp>
#include <algorithm>
#include <cmath>
#include <random>

namespace l_system_plugin {

// ---------------------------------------------------------------------------
// Helper: deterministic sampling from a SingleDistribution using a seeded RNG.
// ---------------------------------------------------------------------------

template <typename T>
T SampleDistribution(const evo_engine::SingleDistribution<T>& dist, std::mt19937& rng) {
  if (dist.deviation <= 0.0f)
    return dist.mean;
  std::normal_distribution<float> normal(0.0f, 1.0f);
  return dist.mean + T(dist.deviation * normal(rng));
}

// ---------------------------------------------------------------------------
// Helper: deterministic sampling from a PlottedDistribution using a seeded RNG.
// ---------------------------------------------------------------------------

inline float SamplePlotted(const evo_engine::PlottedDistribution<float>& pd, float t, std::mt19937& rng) {
  const float mean_val = pd.mean.GetValue(t);
  const float dev_val = pd.deviation.GetValue(t);
  if (dev_val <= 0.0f)
    return mean_val;
  std::normal_distribution<float> dist(mean_val, dev_val);
  return dist(rng);
}

inline uint32_t HashNodeSeed(const float node_random, const uint32_t salt) {
  const float clamped = std::clamp(node_random, 0.0f, 1.0f);
  uint32_t x = static_cast<uint32_t>(clamped * 4294967295.0f) ^ (salt + 0x9e3779b9u);
  x ^= x >> 16;
  x *= 0x7feb352du;
  x ^= x >> 15;
  x *= 0x846ca68bu;
  x ^= x >> 16;
  return x;
}

inline std::mt19937 MakeNodeRng(const float node_random, const uint32_t salt) {
  return std::mt19937(HashNodeSeed(node_random, salt));
}

inline float SampleUnit01(std::mt19937& rng) {
  std::uniform_real_distribution<float> dist(0.0f, 1.0f);
  return dist(rng);
}

// ---------------------------------------------------------------------------
// TropismEntry — user-facing tropism descriptor (one per dynamic list entry).
// ---------------------------------------------------------------------------

struct TropismEntry {
  evo_engine::SingleDistribution<float> direction_x{0.0f};
  evo_engine::SingleDistribution<float> direction_y{-1.0f};
  evo_engine::SingleDistribution<float> direction_z{0.0f};
  evo_engine::SingleDistribution<float> strength{0.0f};
  float usage_chance_percent = 100.0f;  ///< Per-plant activation chance in [0, 100].

  /// Curve: x = normalized branching order (0=rachis..1=max order), y = response multiplier.
  evo_engine::PlottedDistribution<float> order_response;
};

// ---------------------------------------------------------------------------
// SampledTropism — concrete sampled tropism values for one instance.
// ---------------------------------------------------------------------------

struct SampledTropism {
  glm::vec3 direction{0.0f, -1.0f, 0.0f};
  float strength = 0.0f;
  evo_engine::PlottedDistribution<float> order_response;
};

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
  float plastochron_gdd = 30.0f;
  float anthesis_gdd = 200.0f;
  float maturity_gdd = 400.0f;
  float gdd_step = 1.0f;
};

// ---------------------------------------------------------------------------
// Rule factory
// ---------------------------------------------------------------------------

using TasselRule = ProductionRule<TasselGraph, TasselModuleData>;
using TasselEngine = DerivationEngine<TasselGraphData, TasselFlowData, TasselModuleData>;

/**
 * @brief Create topology rules for the two-zone tassel architecture.
 *
 * Rule 1 — Branch zone extension (Apex order=0, vigor>0):
 *   Apex → Internode + [Lateral] + Apex(vigor-1)
 *
 * Rule 2 — Branch zone exhaustion (Apex order=0, vigor<=0):
 *   Apex → (removed)
 *
 * Rule 3 — Central spike extension (SpikeApex, vigor>0):
 *   SpikeApex → Internode + [Sessile] + [Pedicellate] + SpikeApex(vigor-1)
 *
 * Rule 4 — Central spike exhaustion (SpikeApex, vigor<=0):
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
    pair.proximal_target_outward_angle = std::max(0.0f, SamplePlotted(proximal_angle, t_pos, rng));

    pair.pair_internode_length = 0.0f;
    pair.pair_internode_target_length = std::max(0.01f, SamplePlotted(internode_length, t_pos, rng));
    pair.pair_internode_thickness = 0.0f;
    pair.pair_internode_target_thickness = std::max(0.005f, SamplePlotted(internode_thickness, t_pos, rng));
    pair.pair_internode_angle = 0.0f;
    pair.pair_internode_target_angle = std::max(0.0f, SamplePlotted(internode_angle, t_pos, rng));

    pair.distal_scale = glm::vec3(0.0f);
    pair.distal_target_scale = glm::vec3(
        std::max(0.01f, SamplePlotted(distal_x, t_pos, rng)),
        std::max(0.01f, SamplePlotted(distal_y, t_pos, rng)),
        std::max(0.01f, SamplePlotted(distal_z, t_pos, rng)));
    pair.distal_outward_angle = 0.0f;
    pair.distal_target_outward_angle = std::max(0.0f, SamplePlotted(distal_angle, t_pos, rng));

    pair.phase = SpikeletPhase::Emerging;
    pair.main_rachis_pair = main_rachis_pair;
    pair.final_age_gdd = std::max(1.0f, SampleDistribution(params.final_age_gdd, rng));
    pair.anthesis_offset_gdd = anthesis_offset_gdd;
    pair.age_gdd = 0.0f;
    pair.pair_ordinal = std::max(0, pair_ordinal);
    pair.phyllotaxis_azimuth = phyllotaxis_azimuth;
    pair.node_random = SampleUnit01(rng);
  };

  // Rule 1: Peduncle extension.
  {
    TasselRule rule;
    rule.predecessor_symbol = TasselSymbol::Apex;
    rule.priority = 0;
    rule.condition = [params](const RuleContext<TasselGraph>& ctx) -> bool {
      const auto& apex = ctx.self.data.Get<TasselApex>();
      return apex.order == 0 && apex.vigor > 0.0f && apex.age_gdd >= params.plastochron_gdd;
    };
    rule.produce = [params](RuleContext<TasselGraph>& ctx) -> ProductionResult<TasselModuleData> {
      const auto& apex = ctx.self.data.Get<TasselApex>();
      auto node_rng = MakeNodeRng(apex.node_random, 0xC8013EA4u);
      const float t_pos = static_cast<float>(apex.age) / static_cast<float>(std::max(1, params.branch_node_count - 1));
      const float branch_prob =
          std::clamp(SamplePlotted(params.peduncle_branch_probability, t_pos, node_rng), 0.0f, 1.0f);

      ProductionResult<TasselModuleData> result;

      {
        Successor<TasselModuleData> s;
        s.is_branch = false;
        TasselInternode internode;
        internode.target_length = std::max(0.01f, SamplePlotted(params.branch_internode_length, t_pos, node_rng));
        internode.target_thickness = std::max(0.01f, SamplePlotted(params.branch_internode_thickness, t_pos, node_rng));
        internode.length = 0.0f;
        internode.thickness = 0.0f;
        internode.branch_angle = 0.0f;
        internode.target_branch_angle = 0.0f;
        internode.roll_angle = params.phyllotaxis_angle;
        internode.bend_axis_local = glm::vec3(1.0f, 0.0f, 0.0f);
        internode.order = 0;
        internode.is_spike = false;
        internode.node_random = SampleUnit01(node_rng);
        s.data.Set<TasselInternode>(internode);
        s.symbol_id = TasselSymbol::Internode;
        result.successors.push_back(std::move(s));
      }

      if (SampleUnit01(node_rng) < branch_prob) {
        const float lateral_vigor = std::max(0.0f, SamplePlotted(params.lateral_node_count, t_pos, node_rng));
        if (lateral_vigor > 0.0f) {
        Successor<TasselModuleData> s;
        s.is_branch = true;
        TasselLateral lateral;
        const float lateral_delay =
            std::max(0.0f, SamplePlotted(params.lateral_initiation_delay_gdd, t_pos, node_rng));
        lateral.insertion_angle = std::max(1.0f, SamplePlotted(params.lateral_insertion_angle, t_pos, node_rng));
        lateral.azimuth_offset = SampleDistribution(params.branch_azimuth_offset, node_rng);
        lateral.target_length = std::max(0.01f, SamplePlotted(params.lateral_internode_length, t_pos, node_rng));
        lateral.target_thickness = std::max(
            0.01f,
            SamplePlotted(params.branch_internode_thickness, t_pos, node_rng) * std::max(0.01f, params.lateral_thickness_ratio));
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
        new_apex.vigor = apex.vigor - 1.0f;
        new_apex.age = apex.age + 1;
        new_apex.order = 0;
        new_apex.age_gdd = std::max(0.0f, apex.age_gdd - params.plastochron_gdd);
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

  // Rule 2: Peduncle apex exhaustion cleanup.
  {
    TasselRule rule;
    rule.predecessor_symbol = TasselSymbol::Apex;
    rule.priority = 1;
    rule.condition = [params](const RuleContext<TasselGraph>& ctx) -> bool {
      const auto& apex = ctx.self.data.Get<TasselApex>();
      return apex.order == 0 && apex.vigor <= 0.0f && apex.age_gdd >= params.plastochron_gdd;
    };
    rule.produce = [](RuleContext<TasselGraph>&) -> ProductionResult<TasselModuleData> {
      // Spike apex is initialized in the axiom, so exhausted peduncle apices simply disappear.
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
      return apex.vigor > 0.0f && apex.age_gdd >= params.plastochron_gdd;
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
        lateral.insertion_angle = std::max(1.0f, SamplePlotted(params.lateral_insertion_angle, t_pos, node_rng));
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
        new_apex.vigor = apex.vigor - 1.0f;
        new_apex.age = apex.age + 1;
        new_apex.age_gdd = std::max(0.0f, apex.age_gdd - params.plastochron_gdd);
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
      return apex.vigor <= 0.0f && apex.age_gdd >= params.plastochron_gdd;
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
      return apex.order >= 1 && apex.vigor > 0.0f && apex.age_gdd >= params.plastochron_gdd;
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

      const float branch_prob = is_primary
                                    ? std::clamp(SamplePlotted(params.primary_lateral_branch_probability, t_pos, node_rng), 0.0f, 1.0f)
                                    : std::clamp(SamplePlotted(params.secondary_lateral_branch_probability, t_pos, node_rng), 0.0f, 1.0f);
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
        new_apex.vigor = apex.vigor - 1.0f;
        new_apex.age = apex.age + 1;
        new_apex.order = apex.order;
        new_apex.age_gdd = std::max(0.0f, apex.age_gdd - params.plastochron_gdd);
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
      return apex.order >= 1 && apex.vigor <= 0.0f && apex.age_gdd >= params.plastochron_gdd;
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
      return ctx.self.IsEndNode() && lateral.age_gdd >= params.plastochron_gdd;
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
        new_apex.vigor = lateral.lateral_vigor;
        new_apex.age = 0;
        new_apex.order = std::clamp(lateral.order, 1, 2);
        new_apex.age_gdd = std::max(0.0f, lateral.age_gdd - params.plastochron_gdd);
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
        const float angle_progress = std::clamp(params.lateral_angle_development_curve.GetValue(age_t), 0.0f, 1.0f);
        internode.branch_angle = internode.target_branch_angle * angle_progress;
      }

      // Tropism bending.
      const float max_order = 2.0f;
      const float order_t = std::clamp(static_cast<float>(internode.order) / max_order, 0.0f, 1.0f);

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

        const float bend_magnitude = effective_strength * (1.0f - alignment);
        total_bend_world += bend_axis_world * bend_magnitude;
      }

      const float total_curvature = glm::length(total_bend_world);
      constexpr float kMaxStableCurvature = 85.0f;
      constexpr float kTropismSmoothing = 0.25f;
      const float target_curvature = std::clamp(total_curvature, 0.0f, kMaxStableCurvature);
      internode.curvature = glm::mix(internode.curvature, target_curvature, kTropismSmoothing);

      if (total_curvature > 1e-5f) {
        const glm::vec3 bend_axis_world = total_bend_world / total_curvature;
        const glm::vec3 bend_axis_local =
            glm::normalize(glm::conjugate(ctx.self.info.global_rotation) * bend_axis_world);
        if (std::isfinite(bend_axis_local.x) && std::isfinite(bend_axis_local.y) &&
            std::isfinite(bend_axis_local.z) && glm::dot(bend_axis_local, bend_axis_local) > 1e-8f) {
          const glm::vec3 blended_axis = glm::mix(internode.bend_axis_local, bend_axis_local, kTropismSmoothing);
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

      pair.proximal_scale = pair.proximal_target_scale * proximal_scale_progress;
      pair.proximal_outward_angle = pair.proximal_target_outward_angle * proximal_angle_progress;

      pair.pair_internode_length = pair.pair_internode_target_length * internode_length_progress;
      pair.pair_internode_thickness = pair.pair_internode_target_thickness * internode_thickness_progress;
      pair.pair_internode_angle = pair.pair_internode_target_angle * internode_angle_progress;

      pair.distal_scale = pair.distal_target_scale * distal_scale_progress;
      pair.distal_outward_angle = pair.distal_target_outward_angle * distal_angle_progress;

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
