#pragma once

#include "HeightField.hpp"
#include "SoilDescriptor.hpp"
#include "VoxelSoilModel.hpp"

namespace eco_sys_lab_package {
using namespace evo_engine;

/**
 * \brief Enum representing different soil properties that can be visualized or simulated.
 */
enum class SoilProperty {
  Blank,                 ///< No property.
  WaterDensity,          ///< Represents the density of water in soil.
  WaterDensityGradient,  ///< Represents the gradient of water density.
  DiffusionDivergence,   ///< Represents divergence of diffusion.
  GravityDivergence,     ///< Represents divergence caused by gravity.
  NutrientDensity,       ///< Represents the density of nutrients in soil.
  SoilDensity,           ///< Represents soil density.
  SoilLayer              ///< Represents different soil layers.
};

/**
 * \brief The soil is designed to be a private component of an entity.
 * It holds the soil model and can be referenced by multiple trees.
 * The soil will also handle visualization and menu control for the soil model.
 */
class Soil : public IPrivateComponent {
  // member variables to avoid static variables (in case of multiple Soil instances?)
  bool auto_step_ = false;
  bool irrigation_ = true;
  float temporal_progression_progress_ = 0;
  bool temporal_progression_ = false;
  // for user specified sources:
  glm::vec3 source_position_ = glm::vec3(0, 0, 0);
  float source_amount_ = 50.f;
  float source_width_ = 1.0f;

 public:
  VoxelSoilModel soil_model;     ///< The voxel-based soil model.
  AssetRef soil_descriptor_ref;  ///< Reference to the associated soil descriptor.

  /**
   * \brief Handles the inspection logic for the soil component.
   * \param editor_layer A shared pointer to the editor layer.
   * \return True if content is not modified; otherwise, false.
   */
  bool DrawGui(const std::shared_ptr<EditorLayer>& editor_layer);

  /**
   * \brief Applies a random offset to the soil component.
   * \param min The minimum value of the offset.
   * \param max The maximum value of the offset.
   */
  void RandomOffset(float min, float max);

  /**
   * \brief Collects asset references from the soil component.
   * \param list A vector to store the collected asset references.
   */
  void CollectAssetRef(std::vector<AssetRef>& list);

  /**
   * \brief Generates a mesh representation of the soil.
   * \param x_depth Depth along the X-axis.
   * \param z_depth Depth along the Z-axis.
   * \return An Entity representing the generated mesh.
   */
  Entity GenerateMesh(float x_depth = 0.0f, float z_depth = 0.0f);

  /**
   * \brief Initializes the soil model.
   */
  void InitializeSoilModel();

  /**
   * \brief Sets up a test environment for root splitting.
   */
  void SplitRootTestSetup();

  /**
   * \brief Executes fixed updates for the soil component.
   */
  void FixedUpdate() override;

  /**
   * \brief Generates a quad surface aligned along the X-axis.
   * \param back_facing Determines if the quad is back-facing.
   * \param depth Depth of the quad.
   * \param min_xy Minimum XY coordinates for the quad.
   * \param max_xy Maximum XY coordinates for the quad.
   * \param water_factor Factor influencing water visualization.
   * \param nutrient_factor Factor influencing nutrient visualization.
   * \return An Entity representing the generated surface.
   */
  Entity GenerateSurfaceQuadX(bool back_facing, float depth, const glm::vec2& min_xy, const glm::vec2 max_xy,
                              float water_factor, float nutrient_factor);

  /**
   * \brief Generates a quad surface aligned along the Z-axis.
   * \param back_facing Determines if the quad is back-facing.
   * \param depth Depth of the quad.
   * \param min_xy Minimum XY coordinates for the quad.
   * \param max_xy Maximum XY coordinates for the quad.
   * \param water_factor Factor influencing water visualization.
   * \param nutrient_factor Factor influencing nutrient visualization.
   * \return An Entity representing the generated surface.
   */
  Entity GenerateSurfaceQuadZ(bool back_facing, float depth, const glm::vec2& min_xy, const glm::vec2 max_xy,
                              float water_factor, float nutrient_factor);

  /**
   * \brief Generates a cut-out region in the soil.
   * \param x_depth Depth along the X-axis.
   * \param z_depth Depth along the Z-axis.
   * \param water_factor Factor influencing water visualization.
   * \param nutrient_factor Factor influencing nutrient visualization.
   * \param ground_surface Whether the cut-out represents the ground surface.
   * \return An Entity representing the cut-out.
   */
  Entity GenerateCutOut(float x_depth, float z_depth, float water_factor, float nutrient_factor, bool ground_surface);

  /**
   * \brief Generates a full box representation of the soil.
   * \param water_factor Factor influencing water visualization.
   * \param nutrient_factor Factor influencing nutrient visualization.
   * \param ground_surface Whether the box represents the ground surface.
   * \return An Entity representing the generated box.
   */
  Entity GenerateFullBox(float water_factor, float nutrient_factor, bool ground_surface);
};
}  // namespace eco_sys_lab_package
