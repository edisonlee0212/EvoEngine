
#pragma once

#include "StrandModel.hpp"
#include "StrandModelData.hpp"
#include "TreeMeshGenerator.hpp"
#include "Vertex.hpp"

using namespace evo_engine;

namespace eco_sys_lab_plugin {

/**
 * @brief Enumeration for different strand model mesh generation techniques.
 */
enum class StrandModelMeshGeneratorType {
  RecursiveSlicing,  ///< Uses recursive slicing for mesh generation.
  MarchingCube,      ///< Uses marching cubes for mesh generation.
  AlphaShape         ///< Uses alpha shapes for mesh generation.
};

/**
 * @brief Settings for configuring the strand model mesh generator.
 */
struct StrandModelMeshGeneratorSettings {
  /**
   * @brief Type of mesh generator to be used.
   * @details Defaults to Recursive Slicing.
   */
  unsigned generator_type = static_cast<unsigned>(StrandModelMeshGeneratorType::RecursiveSlicing);

#pragma region Recursive Slicing
  int steps_per_segment = 16;  ///< Number of slicing steps per segment.

  float max_param = std::numeric_limits<float>::infinity();  ///< Maximum parameter value for slicing.
  bool branch_connections = true;                            ///< Whether to connect branch structures.
  int u_multiplier = 2;                                      ///< Multiplier for U-axis resolution.
  float v_multiplier = 0.25;                                 ///< Multiplier for V-axis scaling.
  float cluster_distance = 1.0f;                             ///< Minimum required distance for clustering branches.
#pragma endregion

#pragma region Hybrid MarchingCube
  bool remove_duplicate = true;         ///< Determines if duplicate vertices should be removed.
  bool auto_level = true;               ///< Enables automatic level determination.
  int voxel_subdivision_level = 10;     ///< Level of voxel subdivision for marching cubes.
  float marching_cube_radius = 0.002f;  ///< Radius used in marching cube calculations.
  float x_subdivision = 0.03f;          ///< Subdivision factor along the X-axis.
  float y_subdivision = 0.03f;          ///< Subdivision factor along the Y-axis.
  glm::vec4 marching_cube_color = glm::vec4(0.6, 0.3, 0.0f, 1.0f);  ///< Color assigned to marching cube elements.
  glm::vec4 cylindrical_color = glm::vec4(0.1, 0.9, 0.0f, 1.0f);    ///< Color assigned to cylindrical elements.

  int root_distance_multiplier = 10;  ///< Multiplication factor for root distance.
  float circle_multiplier = 1.f;      ///< Circle rendering multiplier.
#pragma endregion

  bool recalculate_uv = false;                 ///< Determines if UV coordinates should be recalculated.
  bool fast_uv = true;                         ///< Enables fast UV calculation.
  int smooth_iteration = 0;                    ///< Number of smoothing iterations applied to the mesh.
  int min_cell_count_for_major_branches = 5;   ///< Minimum number of cells required for generating major branches.
  int max_cell_count_for_minor_branches = 10;  ///< Maximum number of cells considered for minor branches.
  bool enable_branch = true;                   ///< Enables or disables branch generation.
  bool enable_foliage = true;                  ///< Enables or disables foliage generation.

  /**
   * @brief Inspects and modifies settings within the editor.
   * @param editor_layer The shared pointer to the editor layer interface.
   */
  void OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);
};

/**
 * @brief Class responsible for generating a mesh from a strand model.
 */
class StrandModelMeshGenerator {
  /**
   * @brief Generates a cylindrical mesh from the given strand model.
   * @param strand_model The input strand model.
   * @param vertices Output vector storing the generated vertices.
   * @param indices Output vector storing the generated indices.
   * @param settings Configuration settings for the generator.
   */
  static void CylindricalMeshing(const StrandModel& strand_model, std::vector<Vertex>& vertices,
                                 std::vector<unsigned int>& indices, const StrandModelMeshGeneratorSettings& settings);

  /**
   * @brief Performs mesh smoothing on the generated mesh.
   * @param vertices The vertex list of the mesh.
   * @param indices The index list of the mesh.
   */
  static void MeshSmoothing(std::vector<Vertex>& vertices, std::vector<unsigned int>& indices);

  /**
   * @brief Performs mesh smoothing using index pairs.
   * @param vertices The vertex list of the mesh.
   * @param indices The index pairs representing connectivity.
   */
  static void MeshSmoothing(std::vector<Vertex>& vertices, std::vector<std::pair<unsigned int, unsigned int>>& indices);

  /**
   * @brief Calculates normals for the given vertex and index data.
   * @param vertices The vertex list to update.
   * @param indices The index list defining mesh connectivity.
   */
  static void CalculateNormal(std::vector<Vertex>& vertices, const std::vector<unsigned int>& indices);

  /**
   * @brief Calculates normals for an indexed set of vertex pairs.
   * @param vertices The vertex list to update.
   * @param indices The index list using paired connectivity representation.
   */
  static void CalculateNormal(std::vector<Vertex>& vertices,
                              const std::vector<std::pair<unsigned int, unsigned int>>& indices);

  /**
   * @brief Computes UV coordinates based on the strand model.
   * @param strand_model The strand model providing mesh data.
   * @param vertices The output vertex list including UV coordinates.
   * @param settings UV computation settings.
   */
  static void CalculateUv(const StrandModel& strand_model, std::vector<Vertex>& vertices,
                          const StrandModelMeshGeneratorSettings& settings);

 public:
  /**
   * @brief Generates a mesh from a strand model using the specified settings.
   * @param strand_model The input strand model.
   * @param vertices Output vector storing the generated vertices.
   * @param indices Output vector storing the generated indices.
   * @param settings Configuration settings for mesh generation.
   */
  static void Generate(const StrandModel& strand_model, std::vector<Vertex>& vertices,
                       std::vector<unsigned int>& indices, const StrandModelMeshGeneratorSettings& settings);

  /**
   * @brief Generates a mesh including texture coordinates for the strand model.
   * @param strand_model The input strand model.
   * @param vertices Output vector storing the generated vertices.
   * @param tex_coords Output vector storing the generated texture coordinates.
   * @param index_pairs Output vector storing connectivity as indexed pairs.
   * @param settings Configuration settings for mesh generation.
   */
  static void Generate(const StrandModel& strand_model, std::vector<Vertex>& vertices,
                       std::vector<glm::vec2>& tex_coords,
                       std::vector<std::pair<unsigned int, unsigned int>>& index_pairs,
                       const StrandModelMeshGeneratorSettings& settings);
};

}  // namespace eco_sys_lab_plugin
