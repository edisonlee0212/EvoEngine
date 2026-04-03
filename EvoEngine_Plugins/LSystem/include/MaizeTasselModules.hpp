#pragma once

#include "ModuleTypes.hpp"
#include "LSystemGraph.hpp"
#include <glm/glm.hpp>

namespace l_system_plugin {

// ---------------------------------------------------------------------------
// Module types for a monopodial maize tassel with two-zone architecture.
//
// Symbol  0: TasselApex        — branch-zone meristem (order=0: rachis producing laterals,
//                                 order>=1: lateral branch producing spikelet pairs)
// Symbol  1: TasselInternode   — stem segment (rachis or lateral)
// Symbol  2: TasselLateral     — marker for a lateral branch bud
// Symbol  3: TasselSpikeletPair — paired reproductive unit (proximal ellipsoid, pair internode, distal ellipsoid)
// Symbol  4: TasselSpikeApex   — central-spike meristem (produces spikelet pairs)
// ---------------------------------------------------------------------------

struct TasselApex {
  float vigor = 1.0f;   ///< Remaining capacity to produce internodes.
  int age = 0;           ///< Derivation steps since creation.
  int order = 0;         ///< Branching order: 0=rachis, 1=primary, 2=secondary.
  float age_gdd = 0.0f;  ///< Module-local thermal age used for timed child creation.
  float phyllotaxis_phase = 0.0f; ///< Per-axis azimuth phase anchor in degrees.
  float node_random = 0.5f; ///< Per-node random scalar in [0,1] driving deterministic local randomness.
};

struct TasselInternode {
  float length = 0.0f;          ///< Current segment length (cm).
  float thickness = 0.0f;       ///< Current segment thickness (cm).
  float target_length = 0.0f;   ///< Final target length when fully grown (cm).
  float target_thickness = 0.0f;///< Final target thickness when fully grown (cm).
  float branch_angle = 0.0f;    ///< Deflection angle from parent axis (degrees).
  float target_branch_angle = 0.0f; ///< Final branch deflection angle used by angle-development curve.
  float roll_angle = 0.0f;      ///< Roll around parent axis (phyllotaxis, degrees).
  glm::vec3 bend_axis_local = glm::vec3(1.0f, 0.0f, 0.0f); ///< Local bend axis used for directional tropism steering.
  float curvature = 0.0f;       ///< Local bending from tropism (degrees).
  float growth_progress = 0.0f; ///< 0.0 = just initiated, 1.0 = fully grown.
  float age_gdd = 0.0f;         ///< Module-local thermal age.
  int order = 0;                 ///< Branching order: 0=rachis, 1=primary, 2=secondary.
  bool is_spike = false;         ///< True if this internode belongs to the central spike zone.
  float node_random = 0.5f;      ///< Per-node random scalar in [0,1] driving deterministic local randomness.
};

struct TasselLateral {
  float insertion_angle = 30.0f; ///< Angle at which lateral departs parent (degrees).
  float azimuth_offset = 0.0f;   ///< Random roll offset around parent axis (degrees), independent per branch.
  float target_length = 0.0f;   ///< Target internode length for the lateral branch (cm).
  float target_thickness = 0.0f;///< Target thickness for the lateral branch (cm).
  int order = 1;                 ///< Branching order that the lateral creates.
  float lateral_vigor = 1.0f;   ///< Number of internodes the lateral will produce.
  float age_gdd = 0.0f;         ///< Module-local thermal age before expansion.
  float node_random = 0.5f;     ///< Per-node random scalar in [0,1] driving deterministic local randomness.
};

/// Spikelet maturation phases.
enum class SpikeletPhase : int {
  Emerging = 0,
  Anthesis = 1,
  Mature = 2,
};

struct TasselSpikeletPair {
  // Proximal ellipsoid (attached near the parent axis).
  glm::vec3 proximal_scale = glm::vec3(0.0f);
  glm::vec3 proximal_target_scale = glm::vec3(0.25f, 0.45f, 0.25f);
  float proximal_outward_angle = 0.0f;
  float proximal_target_outward_angle = 0.0f;

  // Pair internode (short branch connecting proximal and distal ellipsoids).
  float pair_internode_length = 0.0f;
  float pair_internode_target_length = 0.2f;
  float pair_internode_thickness = 0.0f;
  float pair_internode_target_thickness = 0.04f;
  float pair_internode_angle = 0.0f;
  float pair_internode_target_angle = 0.0f;

  // Distal ellipsoid (at the end of the pair internode).
  glm::vec3 distal_scale = glm::vec3(0.0f);
  glm::vec3 distal_target_scale = glm::vec3(0.22f, 0.40f, 0.22f);
  float distal_outward_angle = 0.0f;
  float distal_target_outward_angle = 0.0f;

  // Development timing.
  SpikeletPhase phase = SpikeletPhase::Emerging;
  float final_age_gdd = 0.0f;       ///< Absolute maturity age from module birth.
  float anthesis_offset_gdd = 0.0f; ///< Position-dependent flowering shift.
  float age_gdd = 0.0f;
  int pair_ordinal = 0;             ///< Insertion index along the local axis.
  float phyllotaxis_azimuth = 0.0f; ///< Explicit roll angle around parent axis in degrees.

  // Context and stochastic state.
  bool main_rachis_pair = false;  ///< True for pairs on main rachis spike zone; false for non-main-axis branches.
  float node_random = 0.5f;
};

/// Central spike apex — only produces spikelet pairs, never lateral branches.
struct TasselSpikeApex {
  float vigor = 1.0f;   ///< Remaining capacity to produce internodes.
  int age = 0;           ///< Derivation steps since creation.
  float age_gdd = 0.0f;  ///< Module-local thermal age.
  float phyllotaxis_phase = 0.0f; ///< Per-axis azimuth phase anchor in degrees.
  float node_random = 0.5f; ///< Per-node random scalar in [0,1] driving deterministic local randomness.
};

// ---------------------------------------------------------------------------
// Convenience type aliases
// ---------------------------------------------------------------------------

/// Module data variant — stores one of the five tassel module types.
using TasselModuleData =
  ModuleVariant<TasselApex, TasselInternode, TasselLateral, TasselSpikeletPair, TasselSpikeApex>;

/// Symbol IDs (match variant index order).
struct TasselSymbol {
  static constexpr int Apex =
    ModuleIndex<TasselApex, TasselApex, TasselInternode, TasselLateral, TasselSpikeletPair, TasselSpikeApex>::value;  // 0
  static constexpr int Internode =
    ModuleIndex<TasselInternode, TasselApex, TasselInternode, TasselLateral, TasselSpikeletPair, TasselSpikeApex>::value;  // 1
  static constexpr int Lateral =
    ModuleIndex<TasselLateral, TasselApex, TasselInternode, TasselLateral, TasselSpikeletPair, TasselSpikeApex>::value;  // 2
  static constexpr int SpikeletPair =
    ModuleIndex<TasselSpikeletPair, TasselApex, TasselInternode, TasselLateral, TasselSpikeletPair, TasselSpikeApex>::value;  // 3
  static constexpr int SpikeApex =
    ModuleIndex<TasselSpikeApex, TasselApex, TasselInternode, TasselLateral, TasselSpikeletPair, TasselSpikeApex>::value;  // 4
};

// ---------------------------------------------------------------------------
// Graph / flow data (plant-wide and per-flow state)
// ---------------------------------------------------------------------------

struct TasselGraphData {
  int total_derivation_steps = 0;
};

struct TasselFlowData {};  // No per-flow state needed yet.

// ---------------------------------------------------------------------------
// Fully instantiated graph and engine types
// ---------------------------------------------------------------------------

using TasselGraph = LSystemGraph<TasselGraphData, TasselFlowData, TasselModuleData>;
using TasselNode = LGraphNode<TasselModuleData>;
using TasselFlow = LGraphFlow<TasselFlowData>;

}  // namespace l_system_plugin
