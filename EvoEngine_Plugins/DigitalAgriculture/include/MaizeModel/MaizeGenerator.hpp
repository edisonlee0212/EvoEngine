#pragma once

#include "Plot2D.hpp"
#include "MaizeDescriptor.hpp"
#include "MaizeGrowthStages.hpp"

namespace digital_agriculture_plugin {
using namespace evo_engine;

/**
 * @class MaizeGenerator
 * @brief Generates procedural models for 3D maize plant representations with continuous growth.
 *
 * This class mirrors SorghumGenerator's morphological parameters but adds continuous
 * procedural growth driven by a single plant_age parameter (0.0 to 1.0).
 *
 * Growth rules:
 * - Leaf emergence follows a linear plastochron.
 * - Each leaf takes exactly two plastochrons to reach its final curve-evaluated length and branching angle.
 * - Stem elongation is completely linear.
 * - Gravity bending remains static and does not change over time.
 * - Panicle/tassel geometry is omitted.
 */
class MaizeGenerator : public IAsset {
 public:
  /// @name Stem Properties
  /// @{
  SingleDistribution<float> stem_tilt_angle;   ///< Tilt angle of the maize stem.
  SingleDistribution<float> internode_length;  ///< Length of internodes in the maize stem.
  SingleDistribution<float> stem_width;        ///< Width of the maize stem.
  /// @}

  /// @name Leaf Properties
  /// @{
  SingleDistribution<float> leaf_amount;  ///< Number of leaves present at full maturity.

  PlottedDistribution<float> leaf_starting_point;   ///< Starting point of leaf emergence.
  PlottedDistribution<float> leaf_curling;          ///< Curling behavior of the leaves.
  PlottedDistribution<float> leaf_roll_angle;       ///< Roll angle of the leaves.
  PlottedDistribution<float> leaf_branching_angle;  ///< Angle at which leaves branch from stem.

  PlottedDistribution<float> leaf_bending;               ///< Bending of leaves along their length.
  PlottedDistribution<float> leaf_bending_acceleration;  ///< Acceleration of bending curvature.
  PlottedDistribution<float> leaf_bending_smoothness;    ///< Smoothness factor of leaf bending.

  PlottedDistribution<float> leaf_waviness;            ///< Waviness exhibited by leaf shape.
  PlottedDistribution<float> leaf_waviness_frequency;  ///< Frequency of leaf waviness pattern.
  PlottedDistribution<float> leaf_length;              ///< Length of individual leaves.
  PlottedDistribution<float> leaf_width;               ///< Width of individual leaves.
  /// @}

  /// @name Finer Control
  /// @{
  Curve2D width_along_stem;     ///< Defines the width variations along the stem.
  Curve2D curling_along_leaf;   ///< Defines the curling profile along the leaf.
  Curve2D width_along_leaf;     ///< Defines leaf width variations from base to tip.
  Curve2D waviness_along_leaf;  ///< Defines the waviness intensity along the leaf.
  /// @}

  /**
   * @brief Initializes the maize generator asset with default sorghum-like parameters.
   */
  void OnCreate() override;

  /**
   * @brief Handles the inspection of this asset in the editor, including the Plant Age slider.
   *
   * @param editor_layer A shared pointer to the editor layer interacting with this asset.
   * @return `true` if the asset content was modified, `false` otherwise.
   */
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;

  /**
   * @brief Serializes the maize generator's parameters to a YAML emitter.
   *
   * @param out YAML emitter where the serialized data is stored.
   */
  void Serialize(YAML::Emitter& out) const override;

  /**
   * @brief Deserializes the maize generator's parameters from a YAML node.
   *
   * @param in YAML node containing the serialized data.
   */
  void Deserialize(const YAML::Node& in) override;

  /**
   * @brief Generates a thumbnail texture representing the maize model.
   *
   * @return A shared pointer to the generated thumbnail texture.
   */
  [[nodiscard]] std::shared_ptr<Texture2D> GenerateThumbnailTexture() override;

  /**
   * @brief Creates a new maize entity using the generator's parameters at current plant_age.
   *
   * @param seed Random seed for procedural generation.
   * @return A new entity representing the generated maize.
   */
  [[nodiscard]] Entity CreateEntity(unsigned int seed = 0);

  /**
   * @brief Applies the generator's parameters to a maize state (used as base data),
   * evaluating all curves at the fully-mature configuration.
   *
   * @param target_maize_state Target maize state to populate.
   * @param seed Random seed for procedural generation.
   */
  void Apply(const std::shared_ptr<MaizeState>& target_maize_state, unsigned int seed = 0) const;

  /**
   * @brief Applies continuous procedural growth to a fully-evaluated MaizeState.
   *
   * This modifies the state in-place based on plant_age:
   * - Determines how many leaves have emerged (linear plastochron).
   * - Scales each visible leaf's length and branching angle based on its individual growth progress.
   * - Linearly interpolates stem length.
   * - Keeps gravity bending static.
   * - Zeroes out panicle geometry.
   *
   * @param target_maize_state The fully-evaluated state to modify with growth.
   * @param age The age of the plant (0.0 to 1.0).
   */
  void ApplyGrowth(const std::shared_ptr<MaizeState>& target_maize_state, float age) const;

 private:
  /// @brief Cached entity for live preview in the editor.
  Entity preview_entity_{};
  /// @brief Cached maize state for live preview.
  std::shared_ptr<MaizeState> cached_state_;
};

}  // namespace digital_agriculture_plugin
