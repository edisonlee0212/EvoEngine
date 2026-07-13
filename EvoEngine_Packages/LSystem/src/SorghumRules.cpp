#include "SorghumRules.hpp"
#include <algorithm>
#include <cmath>
#include <glm/glm.hpp>

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

namespace l_system_package {

namespace {

constexpr uint32_t kInternodeElongationSalt = 0x9142A3C1u;
constexpr uint32_t kInternodeThicknessSalt = 0x7F6B92D5u;
constexpr uint32_t kLeafSheathLengthGrowthSalt = 0x9A73C1D2u;
constexpr uint32_t kLeafNeckLengthGrowthSalt = 0xAF15D84Cu;
constexpr uint32_t kLeafBladeGrowthSalt = 0xA511C3E9u;
constexpr uint32_t kLeafSheathWidthGrowthSalt = 0xB44E97A1u;
constexpr uint32_t kLeafNeckWidthGrowthSalt = 0xC2F1AE65u;
constexpr uint32_t kLeafWidthGrowthSalt = 0xC5D12FA7u;
constexpr uint32_t kLeafAngleDevelopmentSalt = 0xB2F84A31u;
constexpr uint32_t kLeafCurlingDevelopmentSalt = 0xD31A8C73u;
constexpr uint32_t kLeafBendingDevelopmentSalt = 0xE1456BF9u;

inline float NormalizeDegrees(float degrees) {
  float w = std::fmod(degrees, 360.0f);
  if (w < 0.0f)
    w += 360.0f;
  return w;
}

inline float RankToUnitPosition(const int rank, const int total_nodes) {
  return ComputeSorghumAxisRankPosition(rank, total_nodes);
}

inline float AxisRankPosition(const SorghumApex& apex, const int local_rank) {
  return RankToUnitPosition(local_rank, std::max(1, apex.axis_phytomer_count));
}

inline float SampleMainInternodeProfile(const std::vector<float>& profile, const float normalized_rank) {
  if (profile.empty())
    return 0.0f;
  if (profile.size() == 1)
    return profile.front();
  const float position = std::clamp(normalized_rank, 0.0f, 1.0f) * static_cast<float>(profile.size() - 1);
  const size_t lower = static_cast<size_t>(std::floor(position));
  const size_t upper = std::min(lower + 1, profile.size() - 1);
  return glm::mix(profile[lower], profile[upper], position - static_cast<float>(lower));
}

inline float ComputeTillerAxisLengthScale(const SampledSorghumParams& params, const std::vector<float>& main_profile,
                                          const int tiller_count, const float insertion_angle,
                                          const float final_lean_angle, const float target_height_ratio) {
  const int main_count = std::max(1, params.total_phytomer_count);
  float main_path_length = 0.0f;
  for (int rank = 0; rank < main_count; ++rank) {
    const float length = std::max(
        0.0f, main_profile.empty() ? params.internode_length.mean.GetValue(RankToUnitPosition(rank, main_count))
                                   : main_profile[static_cast<size_t>(rank)]);
    main_path_length += length;
  }

  float tiller_path_length = 0.0f;
  float tiller_vertical_rise = 0.0f;
  float cumulative_tilt_degrees = 0.0f;
  for (int rank = 0; rank < std::max(1, tiller_count); ++rank) {
    const float normalized_rank = RankToUnitPosition(rank, std::max(1, tiller_count));
    const float length =
        std::max(0.0f, main_profile.empty() ? params.internode_length.mean.GetValue(normalized_rank)
                                            : SampleMainInternodeProfile(main_profile, normalized_rank));
    cumulative_tilt_degrees += ComputeSorghumTillerInternodeBranchAngle(
        rank, tiller_count, insertion_angle, final_lean_angle, params.tiller_recovery_axis_fraction);
    tiller_path_length += length;
    tiller_vertical_rise += length * std::cos(glm::radians(cumulative_tilt_degrees));
  }
  if (tiller_path_length <= 1.0e-6f || tiller_vertical_rise <= 1.0e-6f)
    return 1.0f;

  const float target_tip_height = std::clamp(target_height_ratio, 0.75f, 1.05f) * main_path_length;
  const float height_scale = target_tip_height / tiller_vertical_rise;
  const float path_cap_scale = params.tiller_max_axis_length_ratio * main_path_length / tiller_path_length;
  return std::clamp(std::min(height_scale, path_cap_scale), 0.5f, 1.5f);
}

// Effective plastochron threshold for an apex of `order`. Mirrors
// ComputeInitiationPlastochronGdd in MaizeTasselRules but inlined here so
// SorghumRules has no dependency on tassel headers.
inline float ComputePlastochronGdd(const SampledSorghumParams& params, const int order) {
  const float base = std::max(1.0f, params.plastochron_gdd);
  const float axis_scale = (order >= 1) ? std::max(0.1f, params.lateral_axis_plastochron_scale)
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
  //   Phyllotaxis: leaf.roll = phyllotaxis_phase; the continued apex advances
  //   that phase by the descriptor's phyllotaxis angle after each phytomer.
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
      const int origin_rank = rank + 1;
      const int tiller_selection_index = ComputeSorghumTillerSelectionIndex(params, origin_rank);

      ProductionResult<SorghumModuleData> result;
      // -- Internode (prolong same flow) --
      {
        Successor<SorghumModuleData> s;
        s.is_branch = false;
        SorghumInternode internode;
        const auto& main_profile = ctx.graph.data.main_internode_target_lengths;
        internode.target_length = std::max(0.001f, main_profile.size() > static_cast<size_t>(rank)
                                                       ? main_profile[static_cast<size_t>(rank)]
                                                       : SamplePlotted(params.internode_length, t_pos, node_rng));
        internode.target_thickness = std::max(0.0005f, SamplePlotted(params.internode_thickness, t_pos, node_rng));
        internode.length = 0.0f;
        internode.thickness = 0.0f;
        internode.branch_angle = 0.0f;  // main culm is upright
        internode.roll_angle = 0.0f;
        internode.bend_axis_local = glm::vec3(1.0f, 0.0f, 0.0f);
        internode.curvature = 0.0f;
        internode.rank = rank;
        internode.order = 0;
        internode.axis_id = 0;
        internode.origin_rank = 0;
        internode.axis_phytomer_count = total_nodes;
        internode.development_rate_scale = 1.0f;
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
        leaf.axis_id = 0;
        leaf.origin_rank = 0;
        leaf.axis_phytomer_count = total_nodes;
        leaf.development_rate_scale = 1.0f;
        leaf.node_random = SampleUnit01(node_rng);

        const float extra_roll = SamplePlotted(params.leaf_roll_angle, t_pos, node_rng);
        leaf.roll_angle_deg = NormalizeDegrees(apex.phyllotaxis_phase + extra_roll);
        leaf.target_blade_length = std::max(0.001f, SamplePlotted(params.leaf_blade_length, t_pos, node_rng));
        leaf.target_blade_max_width = std::max(0.0005f, SamplePlotted(params.leaf_blade_max_width, t_pos, node_rng));
        leaf.target_blade_thickness = std::max(0.00001f, SamplePlotted(params.leaf_blade_thickness, t_pos, node_rng));
        leaf.target_sheath_thickness = std::max(0.00001f, SamplePlotted(params.leaf_sheath_thickness, t_pos, node_rng));
        leaf.target_sheath_length = std::max(0.0f, SamplePlotted(params.leaf_sheath_length, t_pos, node_rng));
        leaf.target_neck_length = std::max(0.0f, SamplePlotted(params.leaf_neck_length, t_pos, node_rng));
        leaf.target_sheath_end_width_ratio =
            std::max(0.05f, SamplePlotted(params.leaf_sheath_end_width_ratio, t_pos, node_rng));
        leaf.target_neck_end_width_ratio =
            std::max(0.05f, SamplePlotted(params.leaf_neck_end_width_ratio, t_pos, node_rng));
        leaf.target_blade_end_width_ratio =
            std::max(0.05f, SamplePlotted(params.leaf_blade_end_width_ratio, t_pos, node_rng));
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
        leaf.continuous_growth.maturation_years = std::max(0.0f, params.maturity_gdd / kSorghumGddPerYear);

        s.data.Set<SorghumLeaf>(leaf);
        s.symbol_id = SorghumSymbol::Leaf;
        result.successors.push_back(std::move(s));
      }

      // -- Primary tiller bud (branch from this internode's leaf axil) --
      if (tiller_selection_index >= 0) {
        Successor<SorghumModuleData> s;
        s.is_branch = true;
        s.parent_override = 0;
        SorghumTillerBud bud;
        bud.axis_id = tiller_selection_index + 1;
        bud.origin_rank = origin_rank;
        bud.reference_main_phytomer_count = total_nodes;
        bud.target_leaf_count_ratio =
            std::clamp(SampleDistribution(params.tiller_leaf_count_ratio, node_rng), 0.75f, 1.05f);
        bud.target_height_ratio = std::clamp(SampleDistribution(params.tiller_height_ratio, node_rng), 0.75f, 1.05f);
        bud.lateral_phytomer_count = ComputeSorghumTillerLeafBudget(total_nodes, bud.target_leaf_count_ratio);
        bud.activation_main_leaf_stage =
            params.tiller_emergence_main_leaf_stages[static_cast<size_t>(std::clamp(origin_rank - 1, 0, 5))];
        bud.insertion_angle = std::max(0.0f, SampleDistribution(params.tiller_insertion_angle, node_rng));
        bud.final_lean_angle = SampleDistribution(params.tiller_final_lean_angle, node_rng);
        bud.azimuth_offset = NormalizeDegrees(params.branch_azimuth_offset +
                                              360.0f * static_cast<float>(tiller_selection_index) /
                                                  static_cast<float>(std::max(1, params.tiller_count)) +
                                              SampleDistribution(params.tiller_azimuth_jitter, node_rng));
        const float origin_t = static_cast<float>(origin_rank - 1) / 5.0f;
        const float leaf_area_ratio =
            std::clamp(SamplePlotted(params.tiller_leaf_area_ratio_by_origin, origin_t, node_rng), 0.1f, 1.2f);
        bud.leaf_linear_scale = std::sqrt(leaf_area_ratio);
        bud.lateral_thickness_ratio =
            std::clamp(SampleDistribution(params.tiller_thickness_ratio, node_rng), 0.05f, 1.2f);
        bud.axis_length_scale = ComputeTillerAxisLengthScale(params, ctx.graph.data.main_internode_target_lengths,
                                                             bud.lateral_phytomer_count, bud.insertion_angle,
                                                             bud.final_lean_angle, bud.target_height_ratio);
        bud.node_random = SampleUnit01(node_rng);
        s.data.Set<SorghumTillerBud>(bud);
        s.symbol_id = SorghumSymbol::TillerBud;
        result.successors.push_back(std::move(s));
      }

      // -- Continued apex (prolong same flow, carries phyllotaxis state) --
      {
        Successor<SorghumModuleData> s;
        s.is_branch = false;
        SorghumApex new_apex = apex;
        new_apex.vigor = apex.vigor - 1;
        new_apex.phytomer_count = apex.phytomer_count + 1;
        new_apex.phyllotaxis_phase = NormalizeDegrees(apex.phyllotaxis_phase + params.phyllotaxis_angle);
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
  //   The first internode departs from the basal node at the bud azimuth.
  //   The configured recovery sequence splits the inverse bend so the tiller
  //   smoothly recovers from its lateral departure.
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
      const float t_pos = AxisRankPosition(apex, rank);

      ProductionResult<SorghumModuleData> result;

      // -- Internode (lateral) --
      {
        Successor<SorghumModuleData> s;
        s.is_branch = false;
        SorghumInternode internode;
        internode.target_length =
            std::max(0.001f, (ctx.graph.data.main_internode_target_lengths.empty()
                                  ? SamplePlotted(params.internode_length, t_pos, node_rng)
                                  : SampleMainInternodeProfile(ctx.graph.data.main_internode_target_lengths, t_pos)) *
                                 apex.axis_length_scale);
        internode.target_thickness =
            std::max(0.0005f, SamplePlotted(params.internode_thickness, t_pos, node_rng) * apex.thickness_ratio);
        internode.length = 0.0f;
        internode.thickness = 0.0f;
        const bool first_lateral_internode = rank == 0;
        internode.branch_angle =
            ComputeSorghumTillerInternodeBranchAngle(rank, apex.axis_phytomer_count, apex.insertion_angle,
                                                     apex.final_lean_angle, params.tiller_recovery_axis_fraction);
        internode.roll_angle = first_lateral_internode ? apex.phyllotaxis_phase : 0.0f;
        internode.bend_axis_local = glm::vec3(1.0f, 0.0f, 0.0f);
        internode.curvature = 0.0f;
        internode.rank = rank;
        internode.order = apex.order;
        internode.axis_id = apex.axis_id;
        internode.origin_rank = apex.origin_rank;
        internode.axis_phytomer_count = apex.axis_phytomer_count;
        internode.development_rate_scale = apex.development_rate_scale;
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
        leaf.axis_id = apex.axis_id;
        leaf.origin_rank = apex.origin_rank;
        leaf.axis_phytomer_count = apex.axis_phytomer_count;
        leaf.development_rate_scale = apex.development_rate_scale;
        leaf.width_scale = apex.leaf_linear_scale;
        leaf.node_random = SampleUnit01(node_rng);

        const float extra_roll = SamplePlotted(params.leaf_roll_angle, t_pos, node_rng);
        leaf.roll_angle_deg = NormalizeDegrees(apex.phyllotaxis_phase + extra_roll);

        leaf.target_blade_length =
            std::max(0.001f, SamplePlotted(params.leaf_blade_length, t_pos, node_rng) * apex.leaf_linear_scale);
        leaf.target_blade_max_width =
            std::max(0.0005f, SamplePlotted(params.leaf_blade_max_width, t_pos, node_rng) * apex.leaf_linear_scale);
        leaf.target_blade_thickness = std::max(0.00001f, SamplePlotted(params.leaf_blade_thickness, t_pos, node_rng));
        leaf.target_sheath_thickness = std::max(0.00001f, SamplePlotted(params.leaf_sheath_thickness, t_pos, node_rng));
        leaf.target_sheath_length = std::max(0.0f, SamplePlotted(params.leaf_sheath_length, t_pos, node_rng));
        leaf.target_neck_length = std::max(0.0f, SamplePlotted(params.leaf_neck_length, t_pos, node_rng));
        leaf.target_sheath_end_width_ratio =
            std::max(0.05f, SamplePlotted(params.leaf_sheath_end_width_ratio, t_pos, node_rng));
        leaf.target_neck_end_width_ratio =
            std::max(0.05f, SamplePlotted(params.leaf_neck_end_width_ratio, t_pos, node_rng));
        leaf.target_blade_end_width_ratio =
            std::max(0.05f, SamplePlotted(params.leaf_blade_end_width_ratio, t_pos, node_rng));
        leaf.target_insertion_angle_deg = SamplePlotted(params.leaf_insertion_angle, t_pos, node_rng);
        leaf.target_curling = SamplePlotted(params.leaf_curling, t_pos, node_rng);
        leaf.target_bending = SamplePlotted(params.leaf_bending, t_pos, node_rng);
        leaf.target_waviness = std::max(0.0f, SamplePlotted(params.leaf_waviness, t_pos, node_rng));
        leaf.waviness_frequency = std::max(0.0f, params.leaf_waviness_frequency);

        auto leaf_rng = MakeNodeRng(leaf.node_random, 0x71F00D22u);
        leaf.lifespan_years = std::max(0.1f, SampleDistribution(params.leaf_lifespan_years, leaf_rng));
        leaf.wilting_years = std::max(0.05f, SampleDistribution(params.leaf_wilting_years, leaf_rng));

        leaf.continuous_growth.t_init_years = ctx.graph.data.clock.NowYears();
        leaf.continuous_growth.maturation_years = std::max(0.0f, params.maturity_gdd / kSorghumGddPerYear);

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
        new_apex.phyllotaxis_phase = NormalizeDegrees(apex.phyllotaxis_phase + params.phyllotaxis_angle);
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
  //   TillerBud(main expanded leaves >= activation stage) -> lateral Apex
  //
  //   The bud is attached to its selected main-culm leaf axil. Activation
  //   replaces it with an order-1 apex carrying the sampled origin, insertion,
  //   recovery, and relative-size morphology.
  // -------------------------------------------------------------------------
  {
    SorghumRule rule;
    rule.predecessor_symbol = SorghumSymbol::TillerBud;
    rule.priority = 0;
    rule.condition = [](const RuleContext<SorghumGraph>& ctx) -> bool {
      const auto& bud = ctx.self.data.Get<SorghumTillerBud>();
      return ctx.graph.data.main_expanded_leaf_count >= bud.activation_main_leaf_stage;
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
      apex.sampled_plastochron_gdd =
          ComputeSorghumTillerCatchUpPlastochron(ComputePlastochronGdd(params, /*order=*/0),
                                                 bud.reference_main_phytomer_count - bud.activation_main_leaf_stage,
                                                 bud.lateral_phytomer_count, params.lateral_axis_plastochron_scale);
      apex.development_rate_scale =
          std::max(1.0f, ComputePlastochronGdd(params, /*order=*/0) / apex.sampled_plastochron_gdd);
      apex.age_gdd = apex.sampled_plastochron_gdd;
      apex.axis_id = bud.axis_id;
      apex.origin_rank = bud.origin_rank;
      apex.reference_main_phytomer_count = bud.reference_main_phytomer_count;
      apex.axis_phytomer_count = bud.lateral_phytomer_count;
      apex.insertion_angle = bud.insertion_angle;
      apex.final_lean_angle = bud.final_lean_angle;
      apex.leaf_linear_scale = bud.leaf_linear_scale;
      apex.thickness_ratio = bud.lateral_thickness_ratio;
      apex.axis_length_scale = bud.axis_length_scale;
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
      const float age_t =
          std::clamp(internode.age_gdd * std::max(1.0f, internode.development_rate_scale) / maturity_gdd, 0.0f, 1.0f);

      const float length_progress = EvaluatePlottedDeterministic(
          params.internode_elongation_curve, age_t, internode.node_random, kInternodeElongationSalt, 0.0f, 1.0f);
      const float thickness_progress = EvaluatePlottedDeterministic(
          params.internode_thickness_curve, age_t, internode.node_random, kInternodeThicknessSalt, 0.0f, 1.0f);

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
      const float age_t =
          std::clamp(leaf.age_gdd * std::max(1.0f, leaf.development_rate_scale) / maturity_gdd, 0.0f, 1.0f);

      const float sheath_length_t = EvaluatePlottedDeterministic(
          params.leaf_sheath_length_growth_curve, age_t, leaf.node_random, kLeafSheathLengthGrowthSalt, 0.0f, 1.0f);
      const float neck_length_t = EvaluatePlottedDeterministic(params.leaf_neck_length_growth_curve, age_t,
                                                               leaf.node_random, kLeafNeckLengthGrowthSalt, 0.0f, 1.0f);
      const float blade_length_t = EvaluatePlottedDeterministic(params.leaf_blade_growth_curve, age_t, leaf.node_random,
                                                                kLeafBladeGrowthSalt, 0.0f, 1.0f);
      const float sheath_width_t = EvaluatePlottedDeterministic(
          params.leaf_sheath_width_growth_curve, age_t, leaf.node_random, kLeafSheathWidthGrowthSalt, 0.0f, 1.0f);
      const float neck_width_t = EvaluatePlottedDeterministic(params.leaf_neck_width_growth_curve, age_t,
                                                              leaf.node_random, kLeafNeckWidthGrowthSalt, 0.0f, 1.0f);
      const float blade_width_t = EvaluatePlottedDeterministic(params.leaf_width_growth_curve, age_t, leaf.node_random,
                                                               kLeafWidthGrowthSalt, 0.0f, 1.0f);
      const float angle_t = EvaluatePlottedDeterministic(params.leaf_angle_development_curve, age_t, leaf.node_random,
                                                         kLeafAngleDevelopmentSalt, 0.0f, 1.0f);
      const float curl_t = EvaluatePlottedDeterministic(params.leaf_curling_development_curve, age_t, leaf.node_random,
                                                        kLeafCurlingDevelopmentSalt, 0.0f, 1.0f);
      const float bend_t = EvaluatePlottedDeterministic(params.leaf_bending_development_curve, age_t, leaf.node_random,
                                                        kLeafBendingDevelopmentSalt, 0.0f, 1.0f);

      leaf.growth_progress = blade_length_t;
      leaf.sheath_length = leaf.target_sheath_length * sheath_length_t;
      leaf.neck_length = leaf.target_neck_length * neck_length_t;
      leaf.blade_length = leaf.target_blade_length * blade_length_t;
      leaf.sheath_end_width_ratio =
          std::max(0.05f, 1.0f + (leaf.target_sheath_end_width_ratio - 1.0f) * sheath_width_t);
      leaf.neck_end_width_ratio = std::max(0.05f, 1.0f + (leaf.target_neck_end_width_ratio - 1.0f) * neck_width_t);
      leaf.blade_end_width_ratio = std::max(0.05f, 1.0f + (leaf.target_blade_end_width_ratio - 1.0f) * blade_width_t);
      leaf.blade_max_width = leaf.target_blade_max_width * blade_width_t;
      leaf.blade_thickness = leaf.target_blade_thickness * blade_width_t;
      leaf.sheath_thickness = leaf.target_sheath_thickness * sheath_width_t;
      leaf.insertion_angle_deg = leaf.target_insertion_angle_deg * angle_t;
      leaf.curling = leaf.target_curling * curl_t;
      leaf.bending = leaf.target_bending * bend_t;
      leaf.waviness = leaf.target_waviness * blade_length_t;  // amplitude tracks blade growth

      s.data.Set<SorghumLeaf>(leaf);
      result.successors.push_back(std::move(s));
      return result;
    };
    rules.push_back(std::move(rule));
  }

  return rules;
}

}  // namespace l_system_package
