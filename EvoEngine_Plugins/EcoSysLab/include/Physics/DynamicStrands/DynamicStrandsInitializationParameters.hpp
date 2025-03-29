#pragma once
#include "Noises.hpp"
#include "Plot2D.hpp"
namespace eco_sys_lab_plugin {
/**
 * \brief Parameters used during the initialization of the dynamic strand model.
 */
struct DynamicStrandsInitializeParameters {
  float min_segment_length = 0.03f;                  ///< The minimum length of a segment.
  float max_segment_length = 0.06f;                  ///< The maximum length of a segment.
  int uniform_subdivision = 5;                       ///< The number of uniform subdivisions per segment for strands.
  Noise3D damage{};                                  ///< Noise parameter for simulating structural damage.
  glm::vec3 damage_scale_factor = glm::vec3(0.01f);  ///< Scale factor for damage effects.
  float neighbor_vertical_range = 3.0f;              ///< Vertical range for finding neighboring segments.
  float neighbor_horizontal_range = 3.0f;            ///< Horizontal range for finding neighboring segments.

  float sapwood_offset = 0.05f;    ///< Offset for simulating sapwood in the model.
  float wood_transition = 0.005f;  ///< Transition factor between different wood types.

  glm::vec2 density = {600.f, 700.f};                   ///< Density range (min, max) for materials.
  glm::vec2 max_stretch_shear_modulus = {9.5f, 13.5f};  ///< Maximum shear modulus range.
  glm::vec2 max_bending_modulus = {0.15f, 2.f};         ///< Maximum bending modulus range.
  glm::vec2 max_twisting_modulus = {0.15f, 2.f};        ///< Maximum twisting modulus range.

  glm::vec2 shear_stretch_strength = {500.f, 250.f};  ///< Strength settings for shear stretch constraints.
  glm::vec2 bending_strength = {500.f, 250.f};        ///< Strength settings for bending constraints.
  glm::vec2 twisting_strength = {500.f, 250.f};       ///< Strength settings for twisting constraints.
  glm::vec2 bundle_strength = {500.f, 250.f};         ///< Strength settings for bundle constraints.
  glm::vec2 connectivity_strength = {250, 125.f};     ///< Strength settings for connectivity constraints.

  bool trunk_additional_strength = true;            ///< Whether the model contains a trunk structure.
  float trunk_offset = 0.3f;                        ///< Offset distance for trunk-based calculations.
  float trunk_transition = 0.1f;                    ///< Transition factor for trunk segmentation.
  float trunk_additional_strength_factor = 1250.f;  ///< Additional strength factor applied to trunks.

  SingleDistribution<float> leaf_position_alpha = {0.01f, 0.1f};      ///< Alpha parameter for leaf positioning.
  SingleDistribution<float> leaf_rotation_alpha = {0.01f, 0.1f};      ///< Alpha parameter for leaf rotation.
  SingleDistribution<float> max_leaf_position_strain = {300.f, 5.f};  ///< Maximum strain for leaf positioning.
  SingleDistribution<float> max_leaf_rotation_strain = {300.f, 5.f};  ///< Maximum strain for leaf rotation.

  GlobalTransform root_transform{};  ///< Transformation applied to the root of the strand model.

  bool use_cgal = false;                ///< Whether to use CGAL for geometric computations.
  bool triangulate_per_bundle = false;  ///< Whether triangulation should be performed per bundle.

  float alpha = 0.00005f;                 ///< Alpha parameter for high precision calculations.
  float bifurcation_alpha = 0.00005f;     ///< Alpha parameter for bifurcation computations.
  float max_dist_squared = 1.0f;          ///< Maximum squared distance considered in calculations.
  bool use_cubic_hermite_spline = false;  ///< Whether to use cubic Hermite splines for interpolation, else use linear.
  int min_bundle_size = 3;                ///< Minimum size of a bundle, i.e. particle count in that branch.

  AssetRef foliage_descriptor;  ///< Descriptor reference for foliage data.

  /**
   * \brief Editor inspection function.
   * \param editor_layer The editor layer to be modified.
   * \return True if contents are unchanged during inspection.
   */
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);

  /**
   * @brief Saves to a YAML emitter.
   * @param name The name of the settings entry.
   * @param out The YAML emitter to serialize data into.
   */
  void Save(const std::string& name, YAML::Emitter& out) const;

  /**
   * @brief Loads from a YAML node.
   * @param name The name of the settings entry.
   * @param in The YAML node containing serialized data.
   */
  void Load(const std::string& name, const YAML::Node& in);
};
}  // namespace eco_sys_lab_plugin