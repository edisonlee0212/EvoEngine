#pragma once

#include "HeightField.hpp"
#include "VoxelSoilModel.hpp"
using namespace evo_engine;
namespace eco_sys_lab {
enum class SoilMaterialType { Clay, SiltyClay, Loam, Sand, LoamySand, Air };

class SoilLayerDescriptor : public IAsset {
 public:
  AssetRef albedo_texture;
  AssetRef roughness_texture;
  AssetRef metallic_texture;
  AssetRef normal_texture;
  AssetRef height_texture;

  Noise3D capacity;
  Noise3D permeability;
  Noise3D density;
  Noise3D initial_nutrients;
  Noise3D initial_water;
  Noise2D thickness;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
  void CollectAssetRef(std::vector<AssetRef>& list) override;
};

/**
 * \brief The soil descriptor contains the procedural parameters for soil model.
 * It helps provide the user's control menu and serialization outside the portable soil model
 */
class SoilDescriptor : public IAsset {
 public:
  SoilParameters soil_parameters;
  glm::ivec2 texture_resolution = {512, 512};
  std::vector<AssetRef> soil_layer_descriptors;
  AssetRef height_field;
  /**ImGui menu goes to here. Also you can take care you visualization with Gizmos here.
   * Note that the visualization will only be activated while you are inspecting the soil private component in the
   * entity inspector.
   */
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void RandomOffset(float min, float max);
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
  void CollectAssetRef(std::vector<AssetRef>& list) override;
};
enum class SoilProperty {
  Blank,

  WaterDensity,
  WaterDensityGradient,
  DiffusionDivergence,
  GravityDivergence,

  NutrientDensity,

  SoilDensity,

  SoilLayer
};
/**
 * \brief The soil is designed to be a private component of an entity.
 * It holds the soil model and can be referenced by multiple trees.
 * The soil will also take care of visualization and menu for soil model.
 */
class Soil : public IPrivateComponent {
 public:
  VoxelSoilModel soil_model;
  AssetRef soil_descriptor;
  /**ImGui menu goes to here.Also you can take care you visualization with Gizmos here.
   * Note that the visualization will only be activated while you are inspecting the soil private component in the
   * entity inspector.
   */
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void RandomOffset(float min, float max);
  void Serialize(YAML::Emitter& out) const override;

  void Deserialize(const YAML::Node& in) override;

  void CollectAssetRef(std::vector<AssetRef>& list) override;

  Entity GenerateMesh(float x_depth = 0.0f, float z_depth = 0.0f);

  void InitializeSoilModel();

  void SplitRootTestSetup();

  void FixedUpdate() override;
  Entity GenerateSurfaceQuadX(bool back_facing, float depth, const glm::vec2& min_xy, const glm::vec2 max_xy,
                              float water_factor, float nutrient_factor);
  Entity GenerateSurfaceQuadZ(bool back_facing, float depth, const glm::vec2& min_xy, const glm::vec2 max_xy,
                              float water_factor, float nutrient_factor);

  Entity GenerateCutOut(float x_depth, float z_depth, float water_factor, float nutrient_factor, bool ground_surface);
  Entity GenerateFullBox(float water_factor, float nutrient_factor, bool ground_surface);

 private:
  // member variables to avoid static variables (in case of multiple Soil instances?)
  bool auto_step_ = false;
  bool irrigation_ = true;
  float temporal_progression_progress_ = 0;
  bool temporal_progression_ = false;
  // for user specified sources:
  glm::vec3 source_position_ = glm::vec3(0, 0, 0);
  float source_amount_ = 50.f;
  float source_width_ = 1.0f;
};
}  // namespace eco_sys_lab