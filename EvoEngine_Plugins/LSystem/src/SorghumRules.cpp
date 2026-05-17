#include "SorghumRules.hpp"
#include <glm/glm.hpp>
#include <algorithm>
#include <cmath>

// ---------------------------------------------------------------------------
// SorghumRules.cpp — Phase C
//
// Production rules for the sorghum L-system. Tiller bud seeding happens in
// SorghumGrowthModel::Initialize() (not as an R-Init-Tillers rule), and leaf
// maturity + chronological senescence are handled in the growth model's CRTP
// hooks (UpdateNodeInfoImpl / UpdateNodeAgingOnlyImpl) — exactly as
// PineGrowthModel does. This keeps the per-symbol rules below short and
// focused on the topology grammar + simple per-step thermal updates.
// ---------------------------------------------------------------------------

namespace l_system_plugin {

namespace {

constexpr uint32_t kInternodeElongationSalt = 0x9142A3C1u;
constexpr uint32_t kInternodeThicknessSalt = 0x7F6B92D5u;
constexpr uint32_t kLeafBladeGrowthSalt = 0xA511C3E9u;
constexpr uint32_t kLeafWidthGrowthSalt = 0xC5D12FA7u;
constexpr uint32_t kLeafAngleDevelopmentSalt = 0xB2F84A31u;
constexpr uint32_t kLeafCurlingDevelopmentSalt = 0xD31A8C73u;
constexpr uint32_t kLeafBendingDevelopmentSalt = 0xE1456BF9u;

inline float NormalizeDegrees(float degrees) {
  float w = std::fmod(degrees, 360.0f);
  if (w < 0.0f) w += 360.0f;
  return w;
}

inline float RankToUnitPosition(const int rank, const int total_nodes) {
  if (total_nodes <= 1) {
    // Single-phytomer axes are both basal and apical; sample midpoint so
    // default rank curves do not collapse organs to near-zero size.
    return 0.5f;
  }
  return std::clamp(
      static_cast<float>(rank) / static_cast<float>(total_nodes - 1), 0.0f, 1.0f);
}

// Effective plastochron threshold for an apex of `order`. Mirrors
// ComputeInitiationPlastochronGdd in MaizeTasselRules but inlined here so
// SorghumRules has no dependency on tassel headers.
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
// Topology rules
// ---------------------------------------------------------------------------

std::vector<SorghumRule> CreateSorghumTopologyRules(const SampledSorghumParams& params) {
  std::vector<SorghumRule> rules;

  // -------------------------------------------------------------------------
  // R-Apex-Phytomer-Order0  (main culm phytomer emission)
  //   Apex(order=0, vigor>0, age_gdd >= plastochron) ->
  //       Internode(rank=k) + [Leaf(rank=k)] + Apex(order=0, vigor-1)
  //
  //   Distichous phyllotaxis: leaf.roll = phyllotaxis_phase + 180*(rank%2).
  //   Per-axis sampled_plastochron_gdd was stamped at apex creation time
  //   (Initialize / R-TillerBud-Activate) so condition + produce see the
  //   same value across passes (parity with PineApex rationale).
  // -------------------------------------------------------------------------
  {
    SorghumRule rule;
    rule.predecessor_symbol = SorghumSymbol::Apex;
    rule.priority = 0;
    rule.condition = [](const RuleContext<SorghumGraph>& ctx) -> bool {
      const auto& apex = ctx.self.data.Get<SorghumApex>();
      return apex.order == 0 && apex.vigor > 0 && apex.age_gdd >= apex.sampled_plastochron_gdd;
    };
    rule.produce = [params](RuleContext<SorghumGraph>& ctx) -> ProductionResult<SorghumModuleData> {
      const auto& apex = ctx.self.data.Get<SorghumApex>();
      auto node_rng = MakeNodeRng(apex.node_random, 0xC8013EA4u);

      const int rank = std::max(0, apex.phytomer_count);
      const int total_nodes = std::max(1, params.total_phytomer_count);
      const float t_pos = RankToUnitPosition(rank, total_nodes);

      ProductionResult<SorghumModuleData> result;

      // -- Internode (prolong same flow) --
      {
        Successor<SorghumModuleData> s;
        s.is_branch = false;
        SorghumInternode internode;
        internode.target_length = std::max(0.001f, SamplePlotted(params.internode_length, t_pos, node_rng));
        internode.target_thickness = std::max(0.0005f, SamplePlotted(params.internode_thickness, t_pos, node_rng));
        internode.length = 0.0f;
        internode.thickness = 0.0f;
        internode.branch_angle = 0.0f;          // main culm is upright
        internode.roll_angle = params.phyllotaxis_angle;  // carry distichous twist along axis
        internode.bend_axis_local = glm::vec3(1.0f, 0.0f, 0.0f);
        internode.curvature = 0.0f;
        internode.rank = rank;
        internode.order = 0;
        internode.node_random = SampleUnit01(node_rng);
        s.data.Set<SorghumInternode>(internode);
        s.symbol_id = SorghumSymbol::Internode;
        result.successors.push_back(std::move(s));
      }

      // -- Leaf (branch off the internode) --
      {
        Successor<SorghumModuleData> s;
        s.is_branch = true;
        SorghumLeaf leaf;
        leaf.rank = rank;
        leaf.s_along_parent_norm = 1.0f;  // anchored at distal end of internode (sheath wraps node)
        leaf.order = 0;
        leaf.node_random = SampleUnit01(node_rng);

        const float distichous = (rank % 2 == 0) ? 0.0f : 180.0f;
        const float extra_roll = SamplePlotted(params.leaf_roll_angle, t_pos, node_rng);
        leaf.roll_angle_deg = NormalizeDegrees(apex.phyllotaxis_phase + distichous + extra_roll);

        leaf.target_blade_length = std::max(0.001f, SamplePlotted(params.leaf_blade_length, t_pos, node_rng));
        leaf.target_blade_max_width = std::max(0.0005f, SamplePlotted(params.leaf_blade_max_width, t_pos, node_rng));
        leaf.target_sheath_length = std::max(0.0f, SamplePlotted(params.leaf_sheath_length, t_pos, node_rng));
        leaf.target_insertion_angle_deg = SamplePlotted(params.leaf_insertion_angle, t_pos, node_rng);
        leaf.target_curling = SamplePlotted(params.leaf_curling, t_pos, node_rng);
        leaf.target_bending = SamplePlotted(params.leaf_bending, t_pos, node_rng);
        leaf.target_waviness = std::max(0.0f, SamplePlotted(params.leaf_waviness, t_pos, node_rng));
        leaf.waviness_frequency = std::max(0.0f, params.leaf_waviness_frequency);

        // Lifecycle scalars sampled per-leaf with node_random as RNG seed —
        // independent draws per cohort, deterministic for a given plant seed.
        auto leaf_rng = MakeNodeRng(leaf.node_random, 0x71F00D11u);
        leaf.lifespan_years = std::max(0.1f, SampleDistribution(params.leaf_lifespan_years, leaf_rng));
        leaf.wilting_years = std::max(0.05f, SampleDistribution(params.leaf_wilting_years, leaf_rng));

        // Continuous-growth window over thermal years (parity with PineNeedleCluster).
        leaf.continuous_growth.t_init_years = ctx.graph.data.clock.NowYears();
        leaf.continuous_growth.maturation_years =
            std::max(0.0f, params.maturity_gdd / kSorghumGddPerYear);

        s.data.Set<SorghumLeaf>(leaf);
        s.symbol_id = SorghumSymbol::Leaf;
        result.successors.push_back(std::move(s));
      }

      // -- Tiller buds (basal node only, main culm only) --
      //
      // The L-system axiom is just the root Apex (Pine-style); we cannot seed
      // sibling nodes from Initialize(). Instead, the very first phytomer of
      // the main culm spawns `tiller_count` SorghumTillerBud branch successors
      // attached to its basal internode. Each bud sleeps on its own GDD
      // dormancy clock and later activates into an order-1 Apex via
      // R-TillerBud-Activate.
      if (rank == 0 && apex.order == 0) {
        const int n = std::max(0, params.tiller_count);
        const float spacing = (n > 0) ? 360.0f / static_cast<float>(n) : 0.0f;
        const int lateral_phytomers = std::max(1, static_cast<int>(std::round(
            static_cast<float>(params.total_phytomer_count) * params.tiller_phytomer_count_scale)));
        for (int i = 0; i < n; ++i) {
          Successor<SorghumModuleData> s;
          s.is_branch = true;
          SorghumTillerBud bud;
          bud.insertion_angle = params.tiller_insertion_angle;
          bud.azimuth_offset = NormalizeDegrees(
              params.branch_azimuth_offset + spacing * static_cast<float>(i));
          const float t_bud = (n > 1)
              ? static_cast<float>(i) / static_cast<float>(n - 1)
              : 0.0f;
          bud.initial_dormancy_gdd = std::max(0.0f,
              SamplePlotted(params.tiller_initiation_delay_gdd, t_bud, node_rng));
          bud.dormancy_gdd_remaining = bud.initial_dormancy_gdd;
          bud.lateral_phytomer_count = lateral_phytomers;
          bud.lateral_thickness_ratio = params.tiller_thickness_ratio;
          bud.node_random = SampleUnit01(node_rng);
          s.data.Set<SorghumTillerBud>(bud);
          s.symbol_id = SorghumSymbol::TillerBud;
          result.successors.push_back(std::move(s));
        }
      }

      // -- Continued apex (prolong same flow, carries phyllotaxis state) --
      {
        Successor<SorghumModuleData> s;
        s.is_branch = false;
        SorghumApex new_apex = apex;
        new_apex.vigor = apex.vigor - 1;
        new_apex.phytomer_count = apex.phytomer_count + 1;
        new_apex.age_gdd = std::max(0.0f, apex.age_gdd - apex.sampled_plastochron_gdd);
        new_apex.node_random = SampleUnit01(node_rng);
        s.data.Set<SorghumApex>(new_apex);
        s.symbol_id = SorghumSymbol::Apex;
        result.successors.push_back(std::move(s));
      }

      return result;
    };
    rules.push_back(std::move(rule));
  }

  // -------------------------------------------------------------------------
  // R-Apex-Terminate (main culm)
  //   Apex(order=0, vigor<=0) -> PanicleBud{}
  //
  //   Placeholder symbol; future panicle grammar plugs in here.
  // -------------------------------------------------------------------------
  {
    SorghumRule rule;
    rule.predecessor_symbol = SorghumSymbol::Apex;
    rule.priority = 1;
    rule.condition = [](const RuleContext<SorghumGraph>& ctx) -> bool {
      const auto& apex = ctx.self.data.Get<SorghumApex>();
      return apex.order == 0 && apex.vigor <= 0;
    };
    rule.produce = [](RuleContext<SorghumGraph>& ctx) -> ProductionResult<SorghumModuleData> {
      const auto& apex = ctx.self.data.Get<SorghumApex>();
      ProductionResult<SorghumModuleData> result;
      Successor<SorghumModuleData> s;
      s.is_branch = false;
      SorghumPanicleBud pb;
      pb.node_random = apex.node_random;
      s.data.Set<SorghumPanicleBud>(pb);
      s.symbol_id = SorghumSymbol::PanicleBud;
      result.successors.push_back(std::move(s));
      return result;
    };
    rules.push_back(std::move(rule));
  }

  // -------------------------------------------------------------------------
  // R-Apex-Phytomer-Order1 (tiller phytomer emission)
  //   Apex(order>=1, vigor>0, age_gdd >= plastochron) ->
  //       Internode(rank=k, order=1) + [Leaf(rank=k, order=1)] + Apex(continue)
  //
  //   First internode (rank 0) on a tiller carries the insertion angle so
  //   the lateral departs from the basal node; subsequent internodes are
  //   straight (branch_angle = 0). Leaf morphology shares the main-culm
  //   curves; tiller-specific geometry differences (smaller, shorter) come
  //   from `tiller_thickness_ratio` and `tiller_phytomer_count_scale`.
  // -------------------------------------------------------------------------
  {
    SorghumRule rule;
    rule.predecessor_symbol = SorghumSymbol::Apex;
    rule.priority = 0;
    rule.condition = [](const RuleContext<SorghumGraph>& ctx) -> bool {
      const auto& apex = ctx.self.data.Get<SorghumApex>();
      return apex.order >= 1 && apex.vigor > 0 && apex.age_gdd >= apex.sampled_plastochron_gdd;
    };
    rule.produce = [params](RuleContext<SorghumGraph>& ctx) -> ProductionResult<SorghumModuleData> {
      const auto& apex = ctx.self.data.Get<SorghumApex>();
      auto node_rng = MakeNodeRng(apex.node_random, 0xB5297A4Du);

      const int rank = std::max(0, apex.phytomer_count);
      const int total_nodes = std::max(1, static_cast<int>(std::round(
          static_cast<float>(params.total_phytomer_count) * params.tiller_phytomer_count_scale)));
      const float t_pos = RankToUnitPosition(rank, total_nodes);

      ProductionResult<SorghumModuleData> result;

      // -- Internode (lateral) --
      {
        Successor<SorghumModuleData> s;
        s.is_branch = false;
        SorghumInternode internode;
        const float length_scale = params.tiller_phytomer_count_scale;
        internode.target_length = std::max(0.001f,
            SamplePlotted(params.internode_length, t_pos, node_rng) * length_scale);
        internode.target_thickness = std::max(0.0005f,
            SamplePlotted(params.internode_thickness, t_pos, node_rng) * params.tiller_thickness_ratio);
        internode.length = 0.0f;
        internode.thickness = 0.0f;
        internode.branch_angle = (rank == 0) ? params.tiller_insertion_angle : 0.0f;
        internode.roll_angle = (rank == 0) ? 0.0f : params.phyllotaxis_angle;
        internode.bend_axis_local = glm::vec3(1.0f, 0.0f, 0.0f);
        internode.curvature = 0.0f;
        internode.rank = rank;
        internode.order = apex.order;
        internode.node_random = SampleUnit01(node_rng);
        s.data.Set<SorghumInternode>(internode);
        s.symbol_id = SorghumSymbol::Internode;
        result.successors.push_back(std::move(s));
      }

      // -- Leaf --
      {
        Successor<SorghumModuleData> s;
        s.is_branch = true;
        SorghumLeaf leaf;
        leaf.rank = rank;
        leaf.s_along_parent_norm = 1.0f;
        leaf.order = apex.order;
        leaf.node_random = SampleUnit01(node_rng);

        const float distichous = (rank % 2 == 0) ? 0.0f : 180.0f;
        const float extra_roll = SamplePlotted(params.leaf_roll_angle, t_pos, node_rng);
        leaf.roll_angle_deg = NormalizeDegrees(apex.phyllotaxis_phase + distichous + extra_roll);

        const float blade_scale = params.tiller_phytomer_count_scale;
        leaf.target_blade_length = std::max(0.001f,
            SamplePlotted(params.leaf_blade_length, t_pos, node_rng) * blade_scale);
        leaf.target_blade_max_width = std::max(0.0005f,
            SamplePlotted(params.leaf_blade_max_width, t_pos, node_rng) * blade_scale);
        leaf.target_sheath_length = std::max(0.0f,
            SamplePlotted(params.leaf_sheath_length, t_pos, node_rng) * blade_scale);
        leaf.target_insertion_angle_deg = SamplePlotted(params.leaf_insertion_angle, t_pos, node_rng);
        leaf.target_curling = SamplePlotted(params.leaf_curling, t_pos, node_rng);
        leaf.target_bending = SamplePlotted(params.leaf_bending, t_pos, node_rng);
        leaf.target_waviness = std::max(0.0f, SamplePlotted(params.leaf_waviness, t_pos, node_rng));
        leaf.waviness_frequency = std::max(0.0f, params.leaf_waviness_frequency);

        auto leaf_rng = MakeNodeRng(leaf.node_random, 0x71F00D22u);
        leaf.lifespan_years = std::max(0.1f, SampleDistribution(params.leaf_lifespan_years, leaf_rng));
        leaf.wilting_years = std::max(0.05f, SampleDistribution(params.leaf_wilting_years, leaf_rng));

        leaf.continuous_growth.t_init_years = ctx.graph.data.clock.NowYears();
        leaf.continuous_growth.maturation_years =
            std::max(0.0f, params.maturity_gdd / kSorghumGddPerYear);

        s.data.Set<SorghumLeaf>(leaf);
        s.symbol_id = SorghumSymbol::Leaf;
        result.successors.push_back(std::move(s));
      }

      // -- Continued apex --
      {
        Successor<SorghumModuleData> s;
        s.is_branch = false;
        SorghumApex new_apex = apex;
        new_apex.vigor = apex.vigor - 1;
        new_apex.phytomer_count = apex.phytomer_count + 1;
        new_apex.age_gdd = std::max(0.0f, apex.age_gdd - apex.sampled_plastochron_gdd);
        new_apex.node_random = SampleUnit01(node_rng);
        s.data.Set<SorghumApex>(new_apex);
        s.symbol_id = SorghumSymbol::Apex;
        result.successors.push_back(std::move(s));
      }

      return result;
    };
    rules.push_back(std::move(rule));
  }

  // -------------------------------------------------------------------------
  // R-Apex-Terminate-Lateral (tiller exhaustion)
  //   Apex(order>=1, vigor<=0) -> empty (death)
  //
  //   Tillers do not produce a panicle in the foundation grammar. The apex
  //   simply disappears once vigor is exhausted.
  // -------------------------------------------------------------------------
  {
    SorghumRule rule;
    rule.predecessor_symbol = SorghumSymbol::Apex;
    rule.priority = 1;
    rule.condition = [](const RuleContext<SorghumGraph>& ctx) -> bool {
      const auto& apex = ctx.self.data.Get<SorghumApex>();
      return apex.order >= 1 && apex.vigor <= 0;
    };
    rule.produce = [](RuleContext<SorghumGraph>&) -> ProductionResult<SorghumModuleData> {
      return {};
    };
    rules.push_back(std::move(rule));
  }

  // -------------------------------------------------------------------------
  // R-TillerBud-Activate
  //   TillerBud(dormancy_gdd_remaining<=0) -> Apex(order=1, vigor=lateral_phytomer_count)
  //
  //   Bud was seeded as a basal branch off the root apex by
  //   SorghumGrowthModel::Initialize(). Activation replaces the bud node
  //   with an order-1 apex that inherits the bud's azimuth_offset as its
  //   phyllotaxis_phase, so the first internode departs in the right
  //   direction.
  // -------------------------------------------------------------------------
  {
    SorghumRule rule;
    rule.predecessor_symbol = SorghumSymbol::TillerBud;
    rule.priority = 0;
    rule.condition = [](const RuleContext<SorghumGraph>& ctx) -> bool {
      const auto& bud = ctx.self.data.Get<SorghumTillerBud>();
      return bud.dormancy_gdd_remaining <= 0.0f;
    };
    rule.produce = [params](RuleContext<SorghumGraph>& ctx) -> ProductionResult<SorghumModuleData> {
      const auto& bud = ctx.self.data.Get<SorghumTillerBud>();
      auto node_rng = MakeNodeRng(bud.node_random, 0xA1F00D33u);

      ProductionResult<SorghumModuleData> result;
      Successor<SorghumModuleData> s;
      s.is_branch = false;
      SorghumApex apex;
      apex.order = 1;
      apex.vigor = std::max(1, bud.lateral_phytomer_count);
      apex.phytomer_count = 0;
      apex.phyllotaxis_phase = NormalizeDegrees(bud.azimuth_offset);
      apex.node_random = SampleUnit01(node_rng);
      apex.age_gdd = 0.0f;
      apex.sampled_plastochron_gdd = ComputePlastochronGdd(params, /*order=*/1);
      s.data.Set<SorghumApex>(apex);
      s.symbol_id = SorghumSymbol::Apex;
      result.successors.push_back(std::move(s));
      return result;
    };
    rules.push_back(std::move(rule));
  }

  return rules;
}

// ---------------------------------------------------------------------------
// Growth rules (in-place per-step updates)
// ---------------------------------------------------------------------------

std::vector<SorghumRule> CreateSorghumGrowthRules(const SampledSorghumParams& params) {
  std::vector<SorghumRule> rules;

  // -------------------------------------------------------------------------
  // G-Apex-Age — apex.age_gdd += gdd_step
  // -------------------------------------------------------------------------
  {
    SorghumRule rule;
    rule.predecessor_symbol = SorghumSymbol::Apex;
    rule.priority = 0;
    rule.produce = [params](RuleContext<SorghumGraph>& ctx) -> ProductionResult<SorghumModuleData> {
      ProductionResult<SorghumModuleData> result;
      Successor<SorghumModuleData> s;
      s.is_branch = false;
      s.symbol_id = SorghumSymbol::Apex;
      auto apex = ctx.self.data.Get<SorghumApex>();
      apex.age_gdd += params.gdd_step;
      s.data.Set<SorghumApex>(apex);
      result.successors.push_back(std::move(s));
      return result;
    };
    rules.push_back(std::move(rule));
  }

  // -------------------------------------------------------------------------
  // G-TillerBud-Age — bud.dormancy_gdd_remaining = max(0, ... - gdd_step)
  // -------------------------------------------------------------------------
  {
    SorghumRule rule;
    rule.predecessor_symbol = SorghumSymbol::TillerBud;
    rule.priority = 0;
    rule.produce = [params](RuleContext<SorghumGraph>& ctx) -> ProductionResult<SorghumModuleData> {
      ProductionResult<SorghumModuleData> result;
      Successor<SorghumModuleData> s;
      s.is_branch = false;
      s.symbol_id = SorghumSymbol::TillerBud;
      auto bud = ctx.self.data.Get<SorghumTillerBud>();
      bud.dormancy_gdd_remaining = std::max(0.0f, bud.dormancy_gdd_remaining - params.gdd_step);
      s.data.Set<SorghumTillerBud>(bud);
      result.successors.push_back(std::move(s));
      return result;
    };
    rules.push_back(std::move(rule));
  }

  // -------------------------------------------------------------------------
  // G-Internode — interpolate length/thickness toward target via curves.
  //   Pre-maturity growth only; CRTP UpdateNodeInfoImpl in SorghumGrowthModel
  //   handles continuous_growth-driven maturity (parity with Pine).
  // -------------------------------------------------------------------------
  {
    SorghumRule rule;
    rule.predecessor_symbol = SorghumSymbol::Internode;
    rule.priority = 0;
    rule.produce = [params](RuleContext<SorghumGraph>& ctx) -> ProductionResult<SorghumModuleData> {
      ProductionResult<SorghumModuleData> result;
      Successor<SorghumModuleData> s;
      s.is_branch = false;
      s.symbol_id = SorghumSymbol::Internode;

      auto internode = ctx.self.data.Get<SorghumInternode>();
      internode.age_gdd += params.gdd_step;

      const float maturity_gdd = std::max(1.0f, params.maturity_gdd);
      const float age_t = std::clamp(internode.age_gdd / maturity_gdd, 0.0f, 1.0f);

        const float length_progress = EvaluatePlottedDeterministic(
          params.internode_elongation_curve,
          age_t,
          internode.node_random,
          kInternodeElongationSalt,
          0.0f,
          1.0f);
        const float thickness_progress = EvaluatePlottedDeterministic(
          params.internode_thickness_curve,
          age_t,
          internode.node_random,
          kInternodeThicknessSalt,
          0.0f,
          1.0f);

      internode.growth_progress = length_progress;
      internode.length = internode.target_length * length_progress;
      internode.thickness = std::max(0.00002f, internode.target_thickness * thickness_progress);

      s.data.Set<SorghumInternode>(internode);
      result.successors.push_back(std::move(s));
      return result;
    };
    rules.push_back(std::move(rule));
  }

  // -------------------------------------------------------------------------
  // G-Leaf-Thermal — pre-maturity GDD-driven growth.
  //   Interpolates blade/sheath/angle/curling/bending/waviness toward their
  //   stamped targets. Maturity stamp + chronological senescence are done
  //   in SorghumGrowthModel CRTP hooks (parity with PineNeedleCluster).
  //   Post-maturity, this rule is a no-op (current values already match
  //   target and curves saturate at 1.0).
  // -------------------------------------------------------------------------
  {
    SorghumRule rule;
    rule.predecessor_symbol = SorghumSymbol::Leaf;
    rule.priority = 0;
    rule.produce = [params](RuleContext<SorghumGraph>& ctx) -> ProductionResult<SorghumModuleData> {
      ProductionResult<SorghumModuleData> result;
      Successor<SorghumModuleData> s;
      s.is_branch = false;
      s.symbol_id = SorghumSymbol::Leaf;

      auto leaf = ctx.self.data.Get<SorghumLeaf>();
      if (!leaf.maturity_reached) {
        leaf.age_gdd += params.gdd_step;
      }

      const float maturity_gdd = std::max(1.0f, params.maturity_gdd);
      const float age_t = std::clamp(leaf.age_gdd / maturity_gdd, 0.0f, 1.0f);

        const float blade_t = EvaluatePlottedDeterministic(
          params.leaf_blade_growth_curve,
          age_t,
          leaf.node_random,
          kLeafBladeGrowthSalt,
          0.0f,
          1.0f);
        const float width_t = EvaluatePlottedDeterministic(
          params.leaf_width_growth_curve,
          age_t,
          leaf.node_random,
          kLeafWidthGrowthSalt,
          0.0f,
          1.0f);
        const float angle_t = EvaluatePlottedDeterministic(
          params.leaf_angle_development_curve,
          age_t,
          leaf.node_random,
          kLeafAngleDevelopmentSalt,
          0.0f,
          1.0f);
        const float curl_t = EvaluatePlottedDeterministic(
          params.leaf_curling_development_curve,
          age_t,
          leaf.node_random,
          kLeafCurlingDevelopmentSalt,
          0.0f,
          1.0f);
        const float bend_t = EvaluatePlottedDeterministic(
          params.leaf_bending_development_curve,
          age_t,
          leaf.node_random,
          kLeafBendingDevelopmentSalt,
          0.0f,
          1.0f);

      leaf.growth_progress = blade_t;
      leaf.blade_length = leaf.target_blade_length * blade_t;
      leaf.blade_max_width = leaf.target_blade_max_width * width_t;
      leaf.sheath_length = leaf.target_sheath_length * blade_t;  // sheath tracks blade elongation
      leaf.insertion_angle_deg = leaf.target_insertion_angle_deg * angle_t;
      leaf.curling = leaf.target_curling * curl_t;
      leaf.bending = leaf.target_bending * bend_t;
      leaf.waviness = leaf.target_waviness * blade_t;  // amplitude tracks blade growth

      s.data.Set<SorghumLeaf>(leaf);
      result.successors.push_back(std::move(s));
      return result;
    };
    rules.push_back(std::move(rule));
  }

  return rules;
}

}  // namespace l_system_plugin
