
#pragma once
#ifdef CUDA_MODULE_SERVICE
#  include <CUDAModule.hpp>
#endif
#include "ILayer.hpp"
#include "PointCloud.hpp"
#include "SorghumDescriptor.hpp"
#include "SorghumField.hpp"

namespace digital_agriculture_package {
using namespace evo_engine;

/**
 * @class SorghumLayer
 * @brief A layer responsible for handling and generating 3D sorghum models in the DigitalAgriculture package.
 */
class SorghumLayer : public ILayer {
  /**
   * @brief Helper function for exporting an object to an OBJ file.
   * @param position The position of the object.
   * @param mesh The mesh of the object.
   * @param of Output file stream to write OBJ data.
   * @param start_index The starting index for writing vertex data.
   */
  static void ObjExportHelper(glm::vec3 position, const std::shared_ptr<Mesh>& mesh, std::ofstream& of,
                              unsigned& start_index);

 public:
  /** @brief Flag to enable or disable compressed BTF (Bidirectional Texture Function). */
  bool enable_compressed_btf = false;

#ifdef CUDA_MODULE_SERVICE
#  pragma region Illumination
  /** @brief Random seed for illumination calculations. */
  int m_seed = 0;

  /** @brief Distance used in push operations for light interaction. */
  float push_distance = 0.001f;

  /** @brief Properties defining ray tracing behavior. */
  RayProperties ray_properties{4, 4};

  /** @brief List of entities currently being processed for illumination. */
  std::vector<Entity> processing_entities;

  /** @brief Index of the entity currently being processed. */
  int processing_index;

  /** @brief Flag indicating whether illumination calculations are in progress. */
  bool processing = false;

  /** @brief Size of light probes used in calculations. */
  float light_probe_size = 0.05f;

  /** @brief Time taken per plant for illumination calculations. */
  float per_plant_calculation_time = 0.0f;

  /** @brief Performs illumination calculations in a frame-by-frame manner. */
  void CalculateIlluminationFrameByFrame();

  /** @brief Computes illumination effects on the models. */
  void CalculateIllumination();

#  pragma endregion
#endif

  /** @brief Settings for the sorghum mesh generation process. */
  SorghumMeshGeneratorSettings sorghum_mesh_generator_settings;

  /** @brief Reference to the panicle material used for rendering. */
  AssetRef panicle_material;

  /** @brief Reference to the material used for the underside of leaves. */
  AssetRef leaf_bottom_face_material;

  /** @brief Reference to the material used for leaves. */
  AssetRef leaf_material;

  /** @brief Reference to the CBTf (Compressed Bidirectional Texture Function) group for leaves. */
  AssetRef leaf_cbtf_group;

  /** @brief Reference to the leaf albedo texture. */
  AssetRef leaf_albedo_texture;

  /** @brief Reference to the leaf normal map texture. */
  AssetRef leaf_normal_texture;

  /** @brief Array of segmented materials for leaves, supporting up to 25 segments. */
  AssetRef segmented_leaf_materials[25];

  /** @brief Length of vertical subdivisions in the generated mesh. */
  float vertical_subdivision_length = 0.01f;

  /** @brief Number of horizontal subdivision steps for mesh generation. */
  int horizontal_subdivision_step = 4;

  /** @brief Width of the skeleton structure supporting the sorghum model. */
  float skeleton_width = 0.0025f;

  /** @brief Color of the skeleton structure. */
  glm::vec3 skeleton_color = glm::vec3(0);

  /** @brief Whether Ctrl+F auto-growth is enabled for crop-driven sorghums. */
  bool auto_increase_crop_target_gdd_ = false;
  /** @brief Auto-growth speed in GDD per second. At 300 GDD/s the full 600 GDD lifecycle takes ~2 s. */
  float crop_target_gdd_increase_speed_ = 300.0f;
  /** @brief Daily temperature (C) used while auto-growing crop models. */
  float crop_growth_daily_temperature_ = 25.0f;
  /** @brief Minimum interval (seconds) between mesh rebuilds during auto-growth.
   *  The growth model advances every frame, but geometry is only regenerated at this rate. */
  float mesh_regen_interval_ = 0.1f;
  /** @brief Tracks whether the growth model has been updated since the last mesh rebuild. */
  bool crop_growth_dirty_ = false;
  /** @brief Wall-clock time of the last mesh rebuild during auto-growth. */
  float last_mesh_regen_time_ = 0.0f;

  /**
   * @brief Called when the layer is created.
   */
  void RegisterTypes(Application& application) override;
  void OnCreate() override;

  /**
   * @brief Generates meshes for all sorghum entities based on the specified settings.
   * @param sorghum_mesh_generator_settings The settings to use for generating the meshes.
   */
  void GenerateMeshForAllSorghums(const SorghumMeshGeneratorSettings& sorghum_mesh_generator_settings) const;

  /**
   * @brief Renders the inspection UI for this layer in the editor.
   * @param editor_layer Shared pointer to the editor layer.
   * @return True if the asset's content is not modified during inspection.
   */
  void OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;

  /**
   * @brief Called every frame to update the layer.
   */
  void Update() override;

  /**
   * @brief Exports a single sorghum entity to an external file.
   * @param sorghum The entity representing the sorghum.
   * @param of Output file stream to write data.
   * @param start_index The starting index for writing vertex data.
   */
  static void ExportSorghum(const Entity& sorghum, std::ofstream& of, unsigned& start_index);

  /**
   * @brief Exports all sorghum models to an external file.
   * @param filename The name of the output file.
   */
  void ExportAllSorghumsModel(const std::string& filename) const;
};

}  // namespace digital_agriculture_package
