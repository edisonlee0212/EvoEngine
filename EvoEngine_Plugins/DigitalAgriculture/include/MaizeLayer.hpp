#pragma once
#ifdef CUDA_MODULE_PLUGIN
#  include <CUDAModule.hpp>
#endif
#include "ILayer.hpp"
#include "PointCloud.hpp"
#include "MaizeDescriptor.hpp"

namespace digital_agriculture_plugin {
using namespace evo_engine;

/**
 * @class MaizeLayer
 * @brief A layer responsible for handling and generating 3D maize models in the DigitalAgriculture plugin.
 */
class MaizeLayer : public ILayer {
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

#ifdef CUDA_MODULE_PLUGIN
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

  /** @brief Settings for the maize mesh generation process. */
  MaizeMeshGeneratorSettings maize_mesh_generator_settings;

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

  /** @brief Width of the skeleton structure supporting the maize model. */
  float skeleton_width = 0.0025f;

  /** @brief Color of the skeleton structure. */
  glm::vec3 skeleton_color = glm::vec3(0);

  /**
   * @brief Called when the layer is created.
   */
  void OnCreate() override;

  /**
   * @brief Generates meshes for all maize entities based on the specified settings.
   * @param maize_mesh_generator_settings The settings to use for generating the meshes.
   */
  void GenerateMeshForAllMaizes(const MaizeMeshGeneratorSettings& maize_mesh_generator_settings) const;

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
   * @brief Exports a single maize entity to an external file.
   * @param maize The entity representing the maize.
   * @param of Output file stream to write data.
   * @param start_index The starting index for writing vertex data.
   */
  static void ExportMaize(const Entity& maize, std::ofstream& of, unsigned& start_index);

  /**
   * @brief Exports all maize models to an external file.
   * @param filename The name of the output file.
   */
  void ExportAllMaizesModel(const std::string& filename) const;
};

}  // namespace digital_agriculture_plugin
