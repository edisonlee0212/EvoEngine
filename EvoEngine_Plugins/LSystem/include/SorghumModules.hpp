#pragma once

#include "GrowthFunction.hpp"
#include "LSystemGraph.hpp"
#include "ModuleTypes.hpp"
#include "SimulationClock.hpp"
#include <glm/glm.hpp>
#include <vector>

namespace l_system_plugin {

// ---------------------------------------------------------------------------
// Sorghum bicolor module set (vegetative culm only; panicle reserved).
//
// Topology:
//   - The SorghumApex (immortal until vigor is exhausted) emits ONE phytomer
//     per plastochron on the GDD clock. A phytomer is exactly:
//         SorghumInternode  +  SorghumLeaf  + (apex passes through).
//     Phyllotaxis is distichous (180 deg by default, parameterizable so
//     cultivar drift is exposable).
//   - At t=0 a single-shot rule emits `tiller_count` SorghumTillerBud
//     siblings at the basal node. Each bud activates after a per-bud
//     `dormancy_gdd_remaining` countdown elapses, becoming an order-1 Apex.
//   - When the main apex's vigor reaches zero it terminates with a
//     SorghumPanicleBud placeholder so a future panicle grammar can plug
//     in without symbol-id churn.
//
// Symbols:
//   0 SorghumApex       — culm meristem; emits phytomers and terminates.
//   1 SorghumInternode  — one phytomer's stem segment + thickness.
//   2 SorghumLeaf       — distichous blade + sheath anchored on parent
//                         internode; lifecycle mirrors PineNeedleCluster
//                         (GDD growth -> maturity stamp -> chronological
//                          senescence -> abscission).
//   3 SorghumTillerBud  — basal lateral bud; activates into order-1 apex.
//   4 SorghumPanicleBud — placeholder for future panicle grammar.
// ---------------------------------------------------------------------------

struct SorghumApex {
  int order = 0;                  ///< 0 = main culm, 1 = tiller.
  int vigor = 0;                  ///< Remaining phytomer emission budget.
  int phytomer_count = 0;         ///< Phytomers emitted so far on this axis.
  float phyllotaxis_phase = 0.0f; ///< Azimuth carried across phytomers (deg).
  float node_random = 0.5f;       ///< Per-node random scalar in [0,1].
  float age_gdd = 0.0f;           ///< GDD accumulated since last emission.

  /// Per-apex stamped sample of the descriptor's `plastochron_gdd`. Stamped
  /// once at apex creation so the R-Apex-Phytomer condition predicate and
  /// the produce lambda observe the same value across derivation passes
  /// (parity with PineApex::sampled_plastochron_years rationale).
  float sampled_plastochron_gdd = 50.0f;
};

struct SorghumInternode {
  float length = 0.0f;            ///< Current shoot length (m).
  float thickness = 0.0f;         ///< Current shoot thickness/diameter (m).
  float target_length = 0.0f;     ///< Final mature length (m).
  float target_thickness = 0.0f;  ///< Final mature thickness (m).
  float branch_angle = 0.0f;      ///< Deflection from parent axis (deg).
  float roll_angle = 0.0f;        ///< Roll about parent axis (deg).
  glm::vec3 bend_axis_local = glm::vec3(1.0f, 0.0f, 0.0f);
  float curvature = 0.0f;         ///< Local bending (deg).
  float growth_progress = 0.0f;   ///< 0 = just emerged, 1 = mature.
  float age_gdd = 0.0f;           ///< Module-local thermal age (GDD).
  int rank = 0;                   ///< Phytomer rank along host axis.
  int order = 0;                  ///< Branching order of host axis.
  float node_random = 0.5f;
  ContinuousGrowthState continuous_growth{};
};

// ---------------------------------------------------------------------------
// SorghumLeaf — distichous blade + sheath anchored on parent internode.
//
// Lifecycle mirrors PineNeedleCluster exactly:
//   1. GDD-driven growth (until `maturity_reached`):
//      blade length/width, sheath length, insertion angle, curling, bending,
//      and waviness all interpolate from current toward target via the
//      descriptor's growth curves keyed on age_gdd / maturity_gdd.
//   2. Maturity stamp: when continuous_growth.NormalizedAge >= 1 - eps,
//      `maturity_reached = true` and `chronological_age_at_maturity_years`
//      is stamped from the node's absolute chronological age.
//   3. Chronological senescence (post-maturity): senescence_phase ramps
//      0..1 over `wilting_years` after `lifespan_years` elapse. Once
//      senescence_phase >= 1 the leaf is `alive=false` and the mesher
//      skips it. node_random adds +/-0.35y jitter to lifespan
//      (kNodeRandomJitterYears parity).
// ---------------------------------------------------------------------------
struct SorghumLeaf {
  // -- Topology / placement --
  int rank = 0;                       ///< Phytomer rank along parent culm.
  float s_along_parent_norm = 1.0f;   ///< Fractional anchor on parent internode.
  float roll_angle_deg = 0.0f;        ///< Distichous azimuth (parent.phyllotaxis + 180*(rank%2)).
  int order = 0;                      ///< Branching order of host axis.
  float node_random = 0.5f;

  // -- GDD-driven growth (until maturity) --
  float age_gdd = 0.0f;
  float blade_length = 0.0f;
  float target_blade_length = 0.0f;       ///< (m)
  float blade_max_width = 0.0f;
  float target_blade_max_width = 0.0f;    ///< (m)
  float sheath_length = 0.0f;
  float target_sheath_length = 0.0f;      ///< (m)
  float insertion_angle_deg = 0.0f;
  float target_insertion_angle_deg = 60.0f;
  float curling = 0.0f;
  float target_curling = 0.0f;            ///< (deg) per SorghumLeafState
  float bending = 0.0f;
  float target_bending = 0.0f;            ///< (deg)
  float waviness = 0.0f;
  float target_waviness = 0.0f;           ///< amplitude
  float waviness_frequency = 0.0f;        ///< constant; not interpolated
  float growth_progress = 0.0f;           ///< 0..1; mirrors PineInternode::growth_progress
  ContinuousGrowthState continuous_growth{}; ///< thermal maturation state (parity with PineNeedleCluster)

  // -- Maturity stamp --
  bool maturity_reached = false;
  float chronological_age_at_maturity_years = 0.0f;

  // -- Chronological senescence (post-maturity, mirrors PineNeedleCluster) --
  int age_years = 0;
  float lifespan_years = 2.5f;        ///< Sampled at initiation (~2-3y typical).
  float wilting_years = 0.5f;         ///< Senescence-onset -> fully-wilted window.
  float senescence_phase = 0.0f;      ///< 0 = green, 1 = fully wilted.
  bool alive = true;                  ///< False = abscised; mesher skips.
};

struct SorghumTillerBud {
  float insertion_angle = 30.0f;          ///< Lateral departure angle (deg).
  float azimuth_offset = 0.0f;            ///< Roll about basal axis (deg).
  float dormancy_gdd_remaining = 0.0f;    ///< 0 = activate next step.
  float initial_dormancy_gdd = 0.0f;      ///< Creation-time dormancy budget.
  int lateral_phytomer_count = 0;         ///< Vigor seeded into the activated apex.
  float lateral_thickness_ratio = 0.7f;   ///< Tiller stem thickness vs main culm.
  float node_random = 0.5f;
};

struct SorghumPanicleBud {
  float node_random = 0.5f;
};

// ---------------------------------------------------------------------------
// Type aliases
// ---------------------------------------------------------------------------

using SorghumModuleData = ModuleVariant<SorghumApex,
                                        SorghumInternode,
                                        SorghumLeaf,
                                        SorghumTillerBud,
                                        SorghumPanicleBud>;

struct SorghumSymbol {
  static constexpr int Apex =
      ModuleIndex<SorghumApex, SorghumApex, SorghumInternode, SorghumLeaf,
                  SorghumTillerBud, SorghumPanicleBud>::value; // 0
  static constexpr int Internode =
      ModuleIndex<SorghumInternode, SorghumApex, SorghumInternode, SorghumLeaf,
                  SorghumTillerBud, SorghumPanicleBud>::value; // 1
  static constexpr int Leaf =
      ModuleIndex<SorghumLeaf, SorghumApex, SorghumInternode, SorghumLeaf,
                  SorghumTillerBud, SorghumPanicleBud>::value; // 2
  static constexpr int TillerBud =
      ModuleIndex<SorghumTillerBud, SorghumApex, SorghumInternode, SorghumLeaf,
                  SorghumTillerBud, SorghumPanicleBud>::value; // 3
  static constexpr int PanicleBud =
      ModuleIndex<SorghumPanicleBud, SorghumApex, SorghumInternode, SorghumLeaf,
                  SorghumTillerBud, SorghumPanicleBud>::value; // 4
};

// ---------------------------------------------------------------------------
// Graph / flow data
// ---------------------------------------------------------------------------

struct SorghumGraphData {
  int total_derivation_steps = 0;
  SimulationClock clock{};
};

struct SorghumFlowData {};

using SorghumGraph = LSystemGraph<SorghumGraphData, SorghumFlowData, SorghumModuleData>;
using SorghumNode = LGraphNode<SorghumModuleData>;
using SorghumFlow = LGraphFlow<SorghumFlowData>;

}  // namespace l_system_plugin
