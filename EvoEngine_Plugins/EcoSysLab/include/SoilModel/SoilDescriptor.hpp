#pragma once

#include "HeightField.hpp"
#include "VoxelSoilModel.hpp"

namespace eco_sys_lab_plugin {
using namespace evo_engine;

/**
 * \brief Enum representing different types of soil materials.
 */
enum class SoilMaterialType { Clay, SiltyClay, Loam, Sand, LoamySand, Air };

/**
 * \brief Represents the descriptor for a soil layer, containing texture and procedural noise parameters.
 */
class SoilLayerDescriptor : public IAsset {
 public:
  AssetRef albedo_texture;     ///< Reference to the albedo texture.
  AssetRef roughness_texture;  ///< Reference to the roughness texture.
  AssetRef metallic_texture;   ///< Reference to the metallic texture.
  AssetRef normal_texture;     ///< Reference to the normal texture.
  AssetRef height_texture;     ///< Reference to the height texture.

  procedural_noise::ProceduralNoise3D capacity_graph{};           ///< 3D noise representing capacity.
  procedural_noise::ProceduralNoise3D permeability_graph{};       ///< 3D noise representing permeability.
  procedural_noise::ProceduralNoise3D density_graph{};            ///< 3D noise representing density.
  procedural_noise::ProceduralNoise3D initial_nutrients_graph{};  ///< 3D noise representing initial nutrients.
  procedural_noise::ProceduralNoise3D initial_water_graph{};      ///< 3D noise representing initial water.
  procedural_noise::ProceduralNoise2D thickness_graph{};          ///< 2D noise representing thickness.

  /**
   * \brief Handles inspection logic for the soil layer descriptor.
   * \param editor_layer A shared pointer to the editor layer.
   * \return True if content is not modified; otherwise, false.
   */
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;

  /**
   * \brief Serializes the soil layer descriptor to a YAML emitter.
   * \param out The YAML emitter to write the serialized data.
   */
  void Serialize(YAML::Emitter& out) const override;

  /**
   * \brief Deserializes the soil layer descriptor from a YAML node.
   * \param in The YAML node containing the serialized data.
   */
  void Deserialize(const YAML::Node& in) override;

  /**
   * \brief Collects asset references from the descriptor.
   * \param list A vector to store the collected asset references.
   */
  void CollectAssetRef(std::vector<AssetRef>& list) override;
};

/**
 * \brief The soil descriptor contains the procedural parameters for the soil model.
 * It allows users to control menu options and handle serialization for the soil model.
 */
class SoilDescriptor : public IAsset {
 public:
  SoilParameters soil_parameters;                ///< Parameters defining soil behavior.
  glm::ivec2 texture_resolution = {512, 512};    ///< Resolution of the texture.
  std::vector<AssetRef> soil_layer_descriptors;  ///< References to soil layer descriptors.
  AssetRef height_field;                         ///< Reference to the height field asset.

  /**
   * \brief Generates a thumbnail texture for visualization.
   * \return A shared pointer to the generated Texture2D object.
   */
  [[nodiscard]] std::shared_ptr<Texture2D> GenerateThumbnailTexture() override;

  /**
   * \brief Handles inspection logic for the soil descriptor.
   * \param editor_layer A shared pointer to the editor layer.
   * \return True if content is not modified; otherwise, false.
   */
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;

  /**
   * \brief Applies a random offset to the soil descriptor.
   * \param min The minimum value of the offset.
   * \param max The maximum value of the offset.
   */
  void RandomOffset(float min, float max);

  /**
   * \brief Serializes the soil descriptor to a YAML emitter.
   * \param out The YAML emitter to write the serialized data.
   */
  void Serialize(YAML::Emitter& out) const override;

  /**
   * \brief Deserializes the soil descriptor from a YAML node.
   * \param in The YAML node containing the serialized data.
   */
  void Deserialize(const YAML::Node& in) override;

  /**
   * \brief Collects asset references from the descriptor.
   * \param list A vector to store the collected asset references.
   */
  void CollectAssetRef(std::vector<AssetRef>& list) override;
};

}  // namespace eco_sys_lab_plugin
