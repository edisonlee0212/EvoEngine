#pragma once

#include <cstdint>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#ifdef ECOSYSLAB_PLUGIN
#  include "Skeleton.hpp"
#endif

namespace digital_agriculture_plugin {

// ============================================================================
// Phytomer organ types for grass-like crops (maize, sorghum)
// ============================================================================

/**
 * @brief Development phase of a single phytomer.
 *
 * In grass crops, each phytomer progresses linearly through these phases.
 * The transitions are driven by accumulated thermal time (GDD).
 */
enum class PhytomerPhase : uint8_t {
  Emerging,    ///< Leaf primordium is forming inside the whorl; not yet visible.
  Growing,     ///< Leaf has emerged from the whorl and is actively elongating.
  Mature,      ///< Leaf has reached its final dimensions; fully expanded (ligule visible).
  Senescent,   ///< Leaf is senescing (losing chlorophyll, reduced photosynthesis).
  Dead         ///< Leaf is fully dead (brown, no photosynthetic contribution).
};

/**
 * @brief Per-leaf growth data within a phytomer.
 *
 * Tracks the development of a single leaf blade attached to the phytomer's node.
 * "Length" and "width" are the biologically meaningful dimensions that the FSPM
 * colleague's model produces or that we grow internally from carbon allocation.
 *
 * These values are the *target* or *current* dimensions used to generate geometry
 * (either mesh or strand-ribbon). They map directly to the existing
 * SorghumLeafState / MaizeLeafState shape parameters.
 */
struct CropLeafData {
  // --- Dimensions (meters) ---
  float length = 0.0f;             ///< Current blade length from ligule to tip.
  float max_length = 0.0f;         ///< Final (genetically determined) blade length.
  float width = 0.0f;              ///< Current maximum blade width (at widest point).
  float max_width = 0.0f;          ///< Final maximum blade width.
  float sheath_length = 0.0f;      ///< Current length of the leaf sheath wrapping the stem.

  // --- Shape parameters (reuse existing DigitalAgriculture conventions) ---
  float roll_angle = 0.0f;         ///< Phyllotactic roll angle around the stem (degrees).
  float branching_angle = 0.0f;    ///< Current insertion angle from the stem axis (degrees). Animated from 0 to max.
  float max_branching_angle = 0.0f;///< Final (genotype) insertion angle (degrees).
  float curling = 0.0f;            ///< Transverse curling (degrees, 0-90).
  float bending = 0.0f;            ///< Gravity-induced bending along the midrib (degrees, -180 to 180).
  float bending_acceleration = 0.5f; ///< Bending curvature acceleration (0-1).
  float bending_smoothness = 0.5f;   ///< Bending curvature smoothness (0-1).
  float waviness = 0.0f;           ///< Amplitude of waviness along the blade.
  float waviness_frequency = 0.0f; ///< Spatial frequency of waviness.

  // --- Physiology ---
  float leaf_area = 0.0f;                ///< Current one-sided leaf area (m^2), derived from length * width * form factor.
  float specific_leaf_area = 0.0f;       ///< SLA (m^2 / g dry mass), used to convert area <-> mass.
  float par_intercepted = 0.0f;          ///< PAR intercepted this timestep (micromol m^-2 s^-1), filled by light model.
  float nitrogen_content = 0.0f;         ///< Leaf nitrogen content (g N / m^2), affects photosynthetic capacity.
  float chlorophyll_fraction = 1.0f;     ///< Fraction of max chlorophyll remaining (1 = healthy green, 0 = fully senescent).

  // --- Carbon balance (per-leaf sink/source) ---
  float carbon_demand = 0.0f;      ///< Carbon sink demand for this leaf's growth this timestep (g C).
  float carbon_allocated = 0.0f;   ///< Carbon actually allocated to this leaf this timestep (g C).

  // --- Status ---
  PhytomerPhase phase = PhytomerPhase::Emerging;
  float thermal_time_since_emergence = 0.0f;  ///< GDD accumulated since this leaf became visible.
};

/**
 * @brief Per-internode growth data within a phytomer.
 *
 * In grass crops the internode only elongates after the stem elongation phase
 * begins (roughly when the plant transitions from vegetative to reproductive).
 * Before that, internodes are effectively zero-length and the plant is a rosette.
 */
struct CropInternodeData {
  float length = 0.0f;            ///< Current internode length (meters).
  float max_length = 0.0f;        ///< Final internode length.
  float diameter = 0.0f;          ///< Current internode diameter (meters).
  float max_diameter = 0.0f;      ///< Final internode diameter.

  float dry_mass = 0.0f;          ///< Structural dry mass of the internode (g).
  float carbon_demand = 0.0f;     ///< Carbon sink demand for elongation this timestep (g C).
  float carbon_allocated = 0.0f;  ///< Carbon actually allocated this timestep (g C).
};

/**
 * @brief Per-phytomer growth data — the node-level data for the crop skeleton.
 *
 * Each node in the crop skeleton represents one phytomer: the fundamental
 * repeating unit of a grass shoot consisting of {node, internode, leaf, axillary bud}.
 *
 * This struct is designed to be the NodeData template parameter for
 * eco_sys_lab_plugin::Skeleton<CropSkeletonData, CropFlowData, CropPhytomerData>.
 *
 * It carries both the developmental state needed by the growth model and the
 * geometric shape data needed by the renderer.
 */
struct CropPhytomerData {
  // --- Identity ---
  int phytomer_index = 0;       ///< 0-based index from the base (oldest) phytomer upward.
  int rank = 0;                 ///< Rank on the culm (0 = main culm, >0 = tiller order).

  // --- Thermal time tracking ---
  float emerged_at_gdd = 0.0f;  ///< Cumulative GDD at which this phytomer was initiated.
  float thermal_age = 0.0f;     ///< GDD accumulated since this phytomer was initiated.

  // --- Organs ---
  CropLeafData leaf;             ///< The leaf blade + sheath attached at this node.
  CropInternodeData internode;   ///< The internode below this node.

  // --- Axillary bud (for tillers / ear / panicle) ---
  bool has_axillary_bud = true;          ///< Whether an axillary bud is present.
  bool axillary_bud_active = false;      ///< Whether the axillary bud has broken dormancy.
  bool is_ear_bearing = false;           ///< True if this phytomer bears the ear (maize) or panicle (sorghum).

  // --- Carbon transport (for source-sink solver) ---
  float carbohydrate_mass = 0.0f;        ///< Current carbohydrate content at this node (g C).
  float max_carbohydrate_mass = 0.0f;    ///< Storage capacity at this node (g C).
  float carbohydrate_source = 0.0f;      ///< Source strength (photosynthesis output) this timestep.
  float carbohydrate_sink = 0.0f;        ///< Total sink demand (leaf + internode + reproductive) this timestep.
  float conductance = 0.0f;              ///< Transport conductance to parent node.
  float net_flow_balance = 0.0f;         ///< Net carbon gain/loss from transport this timestep.

  // --- Geometry (stem segment, used by renderer) ---
  glm::quat desired_local_rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);  ///< Target rotation relative to parent.
  float sagging = 0.0f;                  ///< Gravity-induced droop of the stem at this node.

  // --- Light environment ---
  float light_exposure = 0.0f;           ///< Aggregate light reaching this phytomer's neighborhood (0-1 normalized).
};

// ============================================================================
// Skeleton-level and flow-level data
// ============================================================================

/**
 * @brief Skeleton-level data for a crop plant.
 *
 * Holds plant-wide state that isn't per-phytomer: total carbon pool,
 * phenological stage, cumulative thermal time, etc.
 */
struct CropSkeletonData {
  // --- Phenology ---
  float cumulative_gdd = 0.0f;     ///< Total growing degree-days since planting.
  float base_temperature = 8.0f;   ///< Base temperature for GDD accumulation (°C). Maize ~8-10, sorghum ~8.
  float daily_temperature = 25.0f; ///< Current daily mean temperature (°C), set by environment.

  int total_phytomers_initiated = 0;  ///< How many phytomers have been initiated so far.
  int final_leaf_number = 0;          ///< Genetically determined final leaf number (set by descriptor).
  float plastochron_gdd = 0.0f;       ///< GDD interval between successive phytomer initiations.

  // --- Developmental phases ---
  bool stem_elongation_started = false;  ///< True once the plant transitions to stem elongation.
  float stem_elongation_start_gdd = 0.0f;///< Cumulative GDD at which stem elongation began.
  bool flowering_started = false;        ///< True once reproductive development begins.
  bool grain_filling_started = false;    ///< True once grain fill begins (for carbon partitioning).

  // --- Whole-plant carbon ---
  float total_carbon_source = 0.0f;     ///< Total photosynthetic output this timestep (g C).
  float total_carbon_sink = 0.0f;       ///< Total sink demand this timestep (g C).
  float carbon_reserve = 0.0f;          ///< Non-structural carbohydrate reserve in the stem (g C).
  float max_carbon_reserve = 0.0f;      ///< Maximum stem reserve capacity (g C).

  // --- Plant geometry summary ---
  float plant_height = 0.0f;            ///< Current total plant height (meters).
  float total_leaf_area = 0.0f;         ///< Sum of all leaf areas (m^2), i.e. LAI contribution.
};

/**
 * @brief Flow-level data for the crop skeleton.
 *
 * In EcoSysLab's Skeleton, a "flow" groups a contiguous chain of nodes.
 * For a single-culm grass crop this is minimal — mainly useful if we later
 * support tillers (each tiller = a separate flow branching from the base).
 */
struct CropFlowData {
  int tiller_order = 0;   ///< 0 = main culm, 1+ = tiller generation.
};

// ============================================================================
// Skeleton typedef
// ============================================================================

#ifdef ECOSYSLAB_PLUGIN
/**
 * @brief The crop skeleton type — parameterized with crop-specific data.
 *
 * Uses EcoSysLab's Skeleton template with:
 *  - CropSkeletonData: plant-wide state
 *  - CropFlowData: per-culm/tiller state
 *  - CropPhytomerData: per-phytomer (per-node) state
 */
using CropSkeleton = eco_sys_lab_plugin::Skeleton<CropSkeletonData, CropFlowData, CropPhytomerData>;
#endif

}  // namespace digital_agriculture_plugin
