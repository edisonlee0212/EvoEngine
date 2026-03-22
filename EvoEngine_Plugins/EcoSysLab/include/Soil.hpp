#pragma once

#include "HeightField.hpp"
#include "SoilDescriptor.hpp"
#include "VoxelSoilModel.hpp"
#include "GraphicsPipeline.hpp"
#include "Platform.hpp"

namespace eco_sys_lab_plugin {
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
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;

  /**
   * \brief Applies a random offset to the soil component.
   * \param min The minimum value of the offset.
   * \param max The maximum value of the offset.
   */
  void RandomOffset(float min, float max);

  /**
   * \brief Serializes the soil component to a YAML emitter.
   * \param out The YAML emitter to write the serialized data.
   */
  void Serialize(YAML::Emitter& out) const override;

  /**
   * \brief Deserializes the soil component from a YAML node.
   * \param in The YAML node containing the serialized data.
   */
  void Deserialize(const YAML::Node& in) override;

  /**
   * \brief Collects asset references from the soil component.
   * \param list A vector to store the collected asset references.
   */
  void CollectAssetRef(std::vector<AssetRef>& list) override;

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

  // ── Terrain tessellation rendering ──

  /// Static terrain tessellation pipeline (shared across all Soil instances).
  static std::shared_ptr<GraphicsPipeline> terrain_tessellation_pipeline;

  /// Initializes the terrain tessellation pipeline. Call once at startup (e.g. from EcoSysLabLayer::OnCreate).
  static void InitializeTerrainPipeline();

  /**
   * \brief Generates a tessellated terrain mesh from the height field.
   *
   * Creates a quad-patch grid for hardware tessellation with displacement mapping.
   * The generated mesh entity uses the deferred rendering hook for tessellated rendering.
   * \param displacement_texture Optional high-res displacement texture.
   * \param displacement_intensity Scale factor for displacement (default 0.1).
   * \return An Entity representing the tessellated terrain.
   */
  Entity GenerateTerrainMesh(const std::shared_ptr<Texture2D>& displacement_texture = nullptr,
                             float displacement_intensity = 0.1f);

  /**
   * \brief Registers the terrain for tessellated deferred rendering this frame.
   *
   * Call each frame (e.g. in LateUpdate) to inject the tessellated terrain into
   * the deferred pass through RenderLayer::DeferredRenderingAllCameras.
   */
  void RegisterTerrainRenderInstance();

 private:
  /// Custom vertex buffer for terrain quad patches.
  std::shared_ptr<Buffer> terrain_vertex_buffer_;
  /// Custom index buffer for terrain quad patches (4 indices per patch).
  std::shared_ptr<Buffer> terrain_index_buffer_;
  /// Number of quad-patch indices (= num_quads * 4).
  uint32_t terrain_index_count_ = 0;
  /// Number of vertices in the terrain mesh.
  uint32_t terrain_vertex_count_ = 0;
  /// Material for the tessellated terrain.
  std::shared_ptr<Material> terrain_material_;
  /// Whether terrain mesh has been generated.
  bool terrain_mesh_ready_ = false;
};
}  // namespace eco_sys_lab_plugin