#pragma once

#include "CropGrowthData.hpp"
#include "Plot2D.hpp"

namespace digital_agriculture_package {
using namespace evo_engine;

class SorghumGenerator;  // forward declaration

// ============================================================================
// CropDescriptor — genotype-level parameters for a grass crop
// ============================================================================

/**
 * @brief Genotype/cultivar descriptor for a grass crop (maize, sorghum, etc.).
 *
 * Holds the genetically determined parameters that, together with the
 * environment, drive a CropShootModel. This replaces the role of
 * SorghumGenerator's distributions with concrete per-rank parameter tables
 * while keeping the along-leaf/stem shape curves as fixed genotype profiles.
 *
 * The CropShootModel reads from this descriptor; the bridge utilities use
 * the shape curves when converting a CropSkeleton to legacy SorghumState.
 */
class CropDescriptor : public IAsset {
 public:
  // ------------------------------------------------------------------
  // Phenology
  // ------------------------------------------------------------------
  float base_temperature = 8.0f;          ///< Base temperature for GDD (°C).
  float plastochron_gdd = 40.0f;          ///< GDD between successive phytomer initiations.
  int final_leaf_number = 16;             ///< Total number of leaves the plant will produce.

  float stem_elongation_gdd = 40.f;     ///< Cumulative GDD at which stem elongation begins.
  float flowering_gdd = 800.0f;           ///< Cumulative GDD at which flowering begins. //Alex-note: Unused
  float grain_filling_gdd = 1000.0f;      ///< Cumulative GDD at which grain filling begins.//Alex-note: Unused
  float maturity_gdd = 1500.0f;           ///< Cumulative GDD at which the plant is physiologically mature.//Alex-note: UnuseD

  float leaf_growth_duration_gdd = 120.0f;  ///< GDD from emergence to full expansion per leaf.
  float senescence_onset_gdd = 100.0f;      ///< GDD after maturity before a leaf starts senescing.

  // ------------------------------------------------------------------
  // Per-rank parameter tables (index 0 = basal leaf, index N-1 = flag leaf)
  //
  // These are PlottedDistribution<float>: they store a mean curve + deviation
  // curve parameterized on t = leaf_rank / (final_leaf_number - 1).
  // The CropShootModel evaluates them at the normalized rank of each
  // phytomer to get genotype max values.
  // ------------------------------------------------------------------

  /// @name Leaf geometry targets
  /// @{
  PlottedDistribution<float> max_leaf_length;       ///< Max blade length per rank (m).
  PlottedDistribution<float> max_leaf_width;         ///< Max blade width per rank (m).
  PlottedDistribution<float> leaf_sheath_length;     ///< Sheath length per rank (m).
  /// @}

  /// @name Leaf shape parameters
  /// @{
  PlottedDistribution<float> leaf_roll_angle;        ///< Phyllotactic roll angle per rank (rad).
  PlottedDistribution<float> leaf_branching_angle;   ///< Insertion angle per rank (rad).
  PlottedDistribution<float> leaf_curling;           ///< Transverse curling per rank (0-1).
  PlottedDistribution<float> leaf_bending;           ///< Gravity bending per rank.
  PlottedDistribution<float> leaf_bending_acceleration;
  PlottedDistribution<float> leaf_bending_smoothness;
  PlottedDistribution<float> leaf_waviness;          ///< Waviness amplitude per rank.
  PlottedDistribution<float> leaf_waviness_frequency; ///< Waviness frequency per rank.
  /// @}

  /// @name Internode geometry targets
  /// @{
  PlottedDistribution<float> max_internode_length;   ///< Max internode length per rank (m).
  PlottedDistribution<float> max_internode_diameter;  ///< Max internode diameter per rank (m).
  /// @}

  /// @name Stem properties
  /// @{
  SingleDistribution<float> stem_tilt_angle;   ///< Initial tilt of the stem from vertical (rad).
  /// @}

  // ------------------------------------------------------------------
  // Along-organ shape curves (genotype profiles)
  //
  // These are Curve2D objects that define the normalized shape profile.
  // x ∈ [0,1] is the position along the organ, y is the parameter value
  // normalized to [0,1] (scaled by the per-leaf scalar at render time).
  // ------------------------------------------------------------------

  Curve2D width_along_stem;     ///< Stem width variation from base to top.
  Curve2D width_along_leaf;     ///< Leaf width from base to tip (scaled by max_width).
  Curve2D curling_along_leaf;   ///< Curling profile along the leaf.
  Curve2D waviness_along_leaf;  ///< Waviness intensity along the leaf.

  // ------------------------------------------------------------------
  // Panicle / ear (optional — not all crops have a panicle) //Alex-note: unused. 
  // ------------------------------------------------------------------
  SingleDistribution<glm::vec2> panicle_size;
  SingleDistribution<float> panicle_seed_amount;
  SingleDistribution<float> panicle_seed_radius;

  // ------------------------------------------------------------------
  // Carbon model parameters (initial/default values) 
  // // Alex-note: unused. In the future, carbon-based FSPM could be implemented with these notions.
  // ------------------------------------------------------------------
  float specific_leaf_area = 20.0f;         ///< SLA (m^2 / kg dry mass).
  float max_stem_reserve_fraction = 0.3f;   ///< Max fraction of stem mass that can be reserve carbohydrate.

  // ------------------------------------------------------------------
  // IAsset interface
  // ------------------------------------------------------------------
  void OnCreate() override;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;

  // ------------------------------------------------------------------
  // Import from SorghumGenerator
  // ------------------------------------------------------------------
  /// @brief Copies all geometric distributions and shape curves from a
  /// SorghumGenerator into this CropDescriptor. Phenology GDD values are
  /// kept at their current values; only leaf/internode/stem/panicle
  /// parameters are overwritten.
  void InitFromSorghumGenerator(const SorghumGenerator& sg);
};

}  // namespace digital_agriculture_package
