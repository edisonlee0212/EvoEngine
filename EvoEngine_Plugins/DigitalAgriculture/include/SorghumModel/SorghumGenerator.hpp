
#pragma once

#include "Plot2D.hpp"
#include "SorghumDescriptor.hpp"
#include "SorghumGrowthStages.hpp"

namespace digital_agriculture_plugin {
using namespace evo_engine;

/**
 * @class SorghumGenerator
 * @brief Generates procedural models for 3D sorghum representations.
 *
 * This class provides various distributions and curves that define the structure and appearance
 * of a procedurally generated sorghum model, allowing fine-tuned control over its morphology.
 */
class SorghumGenerator : public IAsset {
 public:
  /// @name Panicle Properties
  /// @{
  SingleDistribution<glm::vec2> panicle_size;     ///< Distribution defining the size of the sorghum panicle.
  SingleDistribution<float> panicle_seed_amount;  ///< Number of seeds per panicle.
  SingleDistribution<float> panicle_seed_radius;  ///< Radius of individual seeds in the panicle.
  /// @}

  /// @name Stem Properties
  /// @{
  SingleDistribution<float> stem_tilt_angle;   ///< Tilt angle of the sorghum stem.
  SingleDistribution<float> internode_length;  ///< Length of internodes in the sorghum stem.
  SingleDistribution<float> stem_width;        ///< Width of the sorghum stem.
  /// @}

  /// @name Leaf Properties
  /// @{
  SingleDistribution<float> leaf_amount;  ///< Number of leaves present in the plant.

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
   * @brief Initializes the sorghum generator asset.
   */
  void OnCreate() override;

  /**
   * @brief Handles the inspection of this asset in the editor.
   *
   * @param editor_layer A shared pointer to the editor layer interacting with this asset.
   * @return `true` if the asset content remains unmodified, `false` otherwise.
   */
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;

  /**
   * @brief Serializes the sorghum generator's parameters to a YAML emitter.
   *
   * @param out YAML emitter where the serialized data is stored.
   */
  void Serialize(YAML::Emitter& out) const override;

  /**
   * @brief Deserializes the sorghum generator's parameters from a YAML node.
   *
   * @param in YAML node containing the serialized data.
   */
  void Deserialize(const YAML::Node& in) override;

  /**
   * @brief Generates a thumbnail texture representing the sorghum model.
   *
   * @return A shared pointer to the generated thumbnail texture.
   */
  [[nodiscard]] std::shared_ptr<Texture2D> GenerateThumbnailTexture() override;

  /**
   * @brief Creates a new sorghum entity using the generator's parameters.
   *
   * @param seed Random seed for procedural generation.
   * @return A new entity representing the generated sorghum.
   */
  [[nodiscard]] Entity CreateEntity(unsigned int seed = 0) const;

  /**
   * @brief Applies the generator's parameters to a sorghum descriptor.
   *
   * @param target_sorghum_descriptor Target sorghum descriptor to update.
   * @param seed Random seed for procedural generation.
   */
  void Apply(const std::shared_ptr<SorghumDescriptor>& target_sorghum_descriptor, unsigned int seed = 0) const;

  /**
   * @brief Applies the generator's parameters to a sorghum growth state.
   *
   * @param target_sorghum_state Target sorghum state to update.
   * @param seed Random seed for procedural generation.
   */
  void Apply(const std::shared_ptr<SorghumState>& target_sorghum_state, unsigned int seed = 0) const;
};

}  // namespace digital_agriculture_plugin
