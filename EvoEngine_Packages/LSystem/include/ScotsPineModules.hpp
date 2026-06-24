#pragma once

#include <cstdint>
#include <glm/glm.hpp>
#include <type_traits>
#include <vector>
#include "GrowthField.hpp"
#include "GrowthFunction.hpp"
#include "LSystemGraph.hpp"
#include "MaterialProfile.hpp"
#include "ModuleTypes.hpp"
#include "SimulationClock.hpp"

namespace l_system_package {

// ---------------------------------------------------------------------------
// Pinus sylvestris module set (simple monopodial phytomer model).
//
// Topology:
//   - The PineApex (immortal) emits ONE phytomer per plastochron during its
//     active growth season. A phytomer is exactly:
//         PineInternode  +  (optionally) one PineNeedleSheath.
//   - During the proximal `bare_zone_fraction` of each year's seasonal
//     growth (temporal fraction of the active season), the apex emits an
//     internode-only phytomer (no needle fascicle). After that, every
//     phytomer carries one fascicle sheath.
//   - Once the apex has emitted `max_phytomers_per_seasonal_growth`
//     phytomers in the current year it stops until the season rolls over
//     to the next year (LSystemLayer detects the dormant->active edge and
//     bumps the SimulationClock's year index).
//   - PineWhorlBud / lateral spawning rules remain in place but are inert
//     while the descriptor's max_branching_order is 0.
//
// Symbols:
//   0 PineApex          - immortal terminal meristem; emits phytomers.
//   1 PineInternode     - one phytomer's internode segment + thickness.
//   2 PineWhorlBud      - overwintering bud; activates -> N lateral apices
//                          (currently disabled by default).
//   3 PineNeedleSheath - persistent basal fascicle sheath on a dwarf shoot.
//   4 PineNeedle       - one needle emerging from a parent sheath.
// ---------------------------------------------------------------------------

enum class PhenologyState { Dormant, Flush, Elongate };

struct PineApex {
  int order = 0;                   ///< 0 = leader, 1+ = laterals.
  float phyllotaxis_phase = 0.0f;  ///< Azimuth carried across phytomers (deg).
  float node_random = 0.5f;        ///< Per-node random scalar in [0,1].
  PhenologyState phenology_state = PhenologyState::Dormant;

  /// Year index this apex last advanced into. R0 (year-rollover) compares
  /// `clock.YearIndex()` against this and resets `phytomers_this_year` when
  /// they differ, so the apex starts a fresh seasonal-growth budget.
  int year_index = 0;
  /// Phytomers emitted so far in the current year. R1 stops emitting once
  /// this reaches `max_phytomers_per_seasonal_growth`.
  int phytomers_this_year = 0;
  /// Physiological time (years) of the last phytomer emission. R1 enforces
  /// `plastochron_years` spacing using this anchor.
  float t_last_emission_years = 0.0f;

  /// Per-apex stamped sample of the descriptor's `plastochron_gdd` distribution
  /// (converted to years). Stamped once at apex creation so the R1 condition
  /// predicate and the R1 production lambda observe the same value across
  /// derivation passes (avoids gate/produce drift when deviation > 0). Also
  /// used by R0 to seed `t_last_emission_years` on year rollover.
  float sampled_plastochron_years = 1.0f;
  /// Per-apex stamped sample of `max_phytomers_per_seasonal_growth`. Same
  /// stability rationale: R1's "stop after this year's cap" check must read
  /// a value that is stable across condition/produce calls for one apex.
  int sampled_max_phytomers_per_seasonal_growth = 12;
  /// Per-apex stamped count of how many phytomers at the start of the
  /// current year are emitted *without* a needle fascicle. Stamped at year
  /// rollover (R0) by drawing the descriptor's `bare_zone_fraction`
  /// distribution exactly once and rounding to the nearest integer
  /// `bare_zone_fraction * sampled_max_phytomers_per_seasonal_growth`.
  /// Replaces the previous time-based gate (which compared
  /// `temporal_progress_in_active_season` to `bare_zone_fraction`); a
  /// count-based gate is robust to year-to-year differences in active-season
  /// length and to plastochron resampling.
  int bare_phytomers_this_year = 0;
  /// Completion ratio from the immediately preceding year
  /// `phytomers_this_year / sampled_max_phytomers_per_seasonal_growth`.
  /// Computed in R0 and carried into the next year's emissions.
  float previous_season_completion_ratio = 1.0f;
  /// Clamped completion-derived vigor proxy carried over at year rollover.
  /// Used by initiation-time needle capacity mapping.
  float previous_season_vigor = 1.0f;
  /// One branch pseudo-whorl site stamped for this annual shoot. `-1` means
  /// this axis produces no branch whorl this year.
  int whorl_phytomer_this_year = -1;
};

struct PineInternode {
  float length = 0.0f;            ///< Current shoot length (m).
  float thickness = 0.0f;         ///< Current shoot thickness/diameter (m).
  float target_length = 0.0f;     ///< Final mature length (m).
  float target_thickness = 0.0f;  ///< Final mature thickness (m).
  float branch_angle = 0.0f;      ///< Deflection from parent axis (deg).
  float roll_angle = 0.0f;        ///< Roll about parent axis (deg).
  glm::vec3 bend_axis_local = glm::vec3(1.0f, 0.0f, 0.0f);
  float curvature = 0.0f;        ///< Local bending from gravitropism (deg).
  float growth_progress = 0.0f;  ///< 0 = just emerged, 1 = mature.
  int year_produced = 0;         ///< Year cohort.
  int age_years = 0;             ///< Module-local age in whole years.
  int order = 0;                 ///< Branching order of this axis.
  float node_random = 0.5f;
  ContinuousGrowthState continuous_growth{};
};

struct PineWhorlBud {
  float insertion_angle = 60.0f;     ///< Lateral departure angle (deg).
  int lateral_count = 5;             ///< Apices spawned at activation.
  int dormancy_years_remaining = 0;  ///< 0 = activate this step.
  int initial_dormancy_years = 0;    ///< Creation-time dormancy budget used for chronological countdown.
  int order = 1;                     ///< Order of laterals to spawn.
  float phyllotaxis_phase = 0.0f;    ///< First lateral azimuth (deg).
  float lateral_azimuth_step_deg = 137.5f;  ///< Coherent azimuth step for this pseudo-whorl.
  float node_random = 0.5f;
};

struct PineNeedle {
  int needle_index = 0;           ///< Index inside the parent sheath fascicle.
  int needle_count = 2;           ///< Parent sheath fascicle count.
  float length = 0.0f;            ///< Current needle length (m).
  float target_length = 0.025f;   ///< Mature needle length (m).
  int age_years = 0;              ///< Years since needle initiation.
  int lifespan_years = 4;         ///< Years before browning starts.
  float browning_years = 0.8f;    ///< Years from senescence onset to abscission.
  bool alive = true;              ///< False = abscised; mesher skips.
  bool maturity_reached = false;  ///< True once thermal maturation reaches 1.0.
  float chronological_age_at_maturity_years = 0.0f;

  // Visible browning progress: 0 = fully green, 1 = fully brown.
  // Driven by PineGrowthModel::UpdateNodeInfoImpl from chronological age
  // elapsed after thermal maturity.
  float senescence_phase = 0.0f;

  // Creation-time vigor scaling for geometric needle thickness.
  float render_radius_scale = 1.0f;

  // Initiation-time capacity diagnostics.
  int initiation_year_index = 0;
  int initiation_phytomer_index = 0;
  float s_along_parent_norm = 0.5f;
  float intra_year_capacity_weight = 1.0f;
  float inter_year_capacity_weight = 1.0f;
  float bud_storage_vigor_weight = 1.0f;

  // Ellipsoid cross-section maxima (semi-axis radii, metres) before applying
  // per-position profile multipliers in the mesher.
  float cross_section_width_radius_m = 0.0009f;
  float cross_section_thickness_radius_m = 0.00055f;

  // Years required for this needle to follow its parent sheath's full
  // branching angle. Kept per needle so siblings can relax at slightly
  // different rates inside the same fascicle.
  float branching_relax_years = 220.0f / 1500.0f;

  float node_random = 0.5f;
  ContinuousGrowthState continuous_growth{};
  float sinusoidal_amplitude_deg = 0.0f;
  float sinusoidal_frequency_cycles = 0.0f;
  float sinusoidal_phase_rad = 0.0f;
  BilateralGrowthField1D growth_field{};
  MaterialProfile1D material_profile{};
};

struct PineNeedleSheath {
  int count = 2;                  ///< Pinus sylvestris fascicle = 2.
  float length = 0.0f;            ///< Current sheath cylinder length (m).
  float target_length = 0.006f;   ///< Mature sheath length (m).
  float width = 0.0f;             ///< Current sheath cylinder diameter (m).
  float target_width = 0.0012f;   ///< Mature sheath cylinder diameter (m).
  int age_years = 0;              ///< Years since sheath initiation.
  bool needles_spawned = false;   ///< True once child PineNeedle modules have been emitted.

  // Anchor on the parent internode's centerline.
  float s_along_parent_norm = 0.5f;  ///< Fractional position [0,1] along parent shoot.
  float roll_offset_deg = 0.0f;      ///< Azimuthal offset around parent axis (deg).

  // Fascicle opening behavior: starts apical (0 deg) and relaxes toward this
  // branching angle as chronological age approaches each needle's
  // branching_relax_years.
  float branching_angle_deg = 72.0f;
  float branching_relax_years = 220.0f / 1500.0f;
  float node_random = 0.5f;

  // Initiation-time capacity diagnostics.
  int initiation_year_index = 0;
  int initiation_phytomer_index = 0;
  float intra_year_capacity_weight = 1.0f;
  float inter_year_capacity_weight = 1.0f;
  float bud_storage_vigor_weight = 1.0f;

  ContinuousGrowthState continuous_growth{};
  std::vector<PineNeedle> pending_needles{};
};

/**
 * @brief Render-role taxonomy for Scots pine module types.
 *
 * This is the module-level render policy used by the render backbone. Modules
 * with role None remain in the biological graph but are excluded from render
 * snapshots.
 */
enum class PineModuleRenderRole : std::uint8_t {
  None = 0,
  StemInstance = 1,
  NeedleStrands = 2,
  SheathInstance = 3,
};

template <typename ModuleT>
struct PineModuleRenderTrait {
  static constexpr PineModuleRenderRole role = PineModuleRenderRole::None;
};

template <>
struct PineModuleRenderTrait<PineInternode> {
  static constexpr PineModuleRenderRole role = PineModuleRenderRole::StemInstance;
};

template <>
struct PineModuleRenderTrait<PineNeedle> {
  static constexpr PineModuleRenderRole role = PineModuleRenderRole::NeedleStrands;
};

template <>
struct PineModuleRenderTrait<PineNeedleSheath> {
  static constexpr PineModuleRenderRole role = PineModuleRenderRole::SheathInstance;
};

// ---------------------------------------------------------------------------
// Type aliases
// ---------------------------------------------------------------------------

using PineModuleData = ModuleVariant<PineApex, PineInternode, PineWhorlBud, PineNeedleSheath, PineNeedle>;

struct PineSymbol {
  static constexpr int Apex =
      ModuleIndex<PineApex, PineApex, PineInternode, PineWhorlBud, PineNeedleSheath, PineNeedle>::value;  // 0
  static constexpr int Internode =
      ModuleIndex<PineInternode, PineApex, PineInternode, PineWhorlBud, PineNeedleSheath, PineNeedle>::value;  // 1
  static constexpr int WhorlBud =
      ModuleIndex<PineWhorlBud, PineApex, PineInternode, PineWhorlBud, PineNeedleSheath, PineNeedle>::value;  // 2
  static constexpr int NeedleSheath =
      ModuleIndex<PineNeedleSheath, PineApex, PineInternode, PineWhorlBud, PineNeedleSheath, PineNeedle>::value;  // 3
  static constexpr int Needle =
      ModuleIndex<PineNeedle, PineApex, PineInternode, PineWhorlBud, PineNeedleSheath, PineNeedle>::value;  // 4
};

// ---------------------------------------------------------------------------
// Graph / flow data
// ---------------------------------------------------------------------------

struct PineGraphData {
  int current_year = 0;
  int total_derivation_steps = 0;
  SimulationClock clock{};
  /// World-frame gravity magnitude (m/s^2). 0 = no body force on needles.
  float gravity_m_s2 = 0.0f;
};

struct PineFlowData {};

using PineGraph = LSystemGraph<PineGraphData, PineFlowData, PineModuleData>;
using PineNode = LGraphNode<PineModuleData>;
using PineFlow = LGraphFlow<PineFlowData>;

inline PineModuleRenderRole ResolvePineModuleRenderRole(const PineModuleData& module_data) {
  return module_data.Visit([](const auto& module) {
    using ModuleType = std::decay_t<decltype(module)>;
    return PineModuleRenderTrait<ModuleType>::role;
  });
}

}  // namespace l_system_package
