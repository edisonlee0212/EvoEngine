#pragma once
#include "DynamicTreeStrandGraph.hpp"
#include "Plot2D.hpp"

namespace eco_sys_lab_plugin {
using namespace evo_engine;

enum class MeshingType { AlphaShape, KineticVoronoi };

/**
 * \brief Parameters used during the initialization of the dynamic strand model.
 */
struct DynamicStrandsInitializeParameters {
  //MeshingType meshing_type = MeshingType::KineticVoronoi;  ///< The type of meshing algorithm to use.
  MeshingType meshing_type = MeshingType::AlphaShape;  ///< The type of meshing algorithm to use.
  float min_segment_length = 0.03f;                        ///< The minimum length of a segment.
  float max_segment_length = 0.06f;                        ///< The maximum length of a segment.
  int uniform_subdivision = 5;                         ///< The number of uniform subdivisions per segment for strands.
  procedural_noise::ProceduralNoise3D damage_graph{};  ///< Noise parameter for simulating structural damage.
  glm::vec3 damage_scale_factor = glm::vec3(0.01f);    ///< Scale factor for damage effects.
  float neighbor_vertical_range = 3.0f;                ///< Vertical range for finding neighboring segments.
  float neighbor_horizontal_range = 3.0f;              ///< Horizontal range for finding neighboring segments.

  float sapwood_offset = 0.05f;    ///< Offset for simulating sapwood in the model.
  float wood_transition = 0.005f;  ///< Transition factor between different wood types.

  ModulusGraph modulus_graph{ModulusGraph::Output()};  ///< Graph for offset distance for trunk-based calculations.
  bool show_modulus_graph = false;                     ///< Whether to show the trunk offset graph.

  StrengthGraph strength_graph{StrengthGraph::Output()};  ///< Graph for strength calculations.
  bool show_strength_graph = false;                       ///< Whether to show the strength graph.
  bool trunk_additional_strength = true;                  ///< Whether the model contains a trunk structure.

  BiologicalPropertiesGraph biological_properties_graph{
      BiologicalPropertiesGraph::Output()};       ///< Graph for biological properties calculations.
  bool show_biological_properties_graph = false;  ///< Whether to show the biological properties graph.

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