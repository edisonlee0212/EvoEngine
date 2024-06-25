#pragma once

#include "StrandModel.hpp"
#include "StrandModelData.hpp"
#include "TreeMeshGenerator.hpp"
#include "Vertex.hpp"

using namespace evo_engine;
namespace eco_sys_lab {
enum class StrandModelMeshGeneratorType { RecursiveSlicing, MarchingCube };

struct StrandModelMeshGeneratorSettings {
  unsigned generator_type = static_cast<unsigned>(StrandModelMeshGeneratorType::RecursiveSlicing);
#pragma region Recursive Slicing
  int steps_per_segment = 4;
  // this is for debugging purposes only and should not be used to obtain a proper mesh
  // bool m_limitProfileIterations = false;
  // int m_maxProfileIterations = 20;
  float max_param = std::numeric_limits<float>::infinity();
  bool branch_connections = true;
  int u_multiplier = 2;
  float v_multiplier = 0.25;
  float cluster_distance = 1.0f;
#pragma endregion

#pragma region Hybrid MarchingCube
  bool remove_duplicate = true;

  bool auto_level = true;
  int voxel_subdivision_level = 10;
  float marching_cube_radius = 0.002f;
  float x_subdivision = 0.03f;
  float y_subdivision = 0.03f;
  glm::vec4 marching_cube_color = glm::vec4(0.6, 0.3, 0.0f, 1.0f);
  glm::vec4 cylindrical_color = glm::vec4(0.1, 0.9, 0.0f, 1.0f);

  int root_distance_multiplier = 10;
  float circle_multiplier = 1.f;
#pragma endregion

  bool recalculate_uv = false;
  bool fast_uv = true;
  int smooth_iteration = 0;
  int min_cell_count_for_major_branches = 5;
  int max_cell_count_for_minor_branches = 10;
  bool enable_branch = true;
  bool enable_foliage = true;
  void OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);
};
class StrandModelMeshGenerator {
  static void RecursiveSlicing(const StrandModel& strand_model, std::vector<Vertex>& vertices,
                               std::vector<unsigned int>& indices, const StrandModelMeshGeneratorSettings& settings);

  static void RecursiveSlicing(const StrandModel& strand_model, std::vector<Vertex>& vertices,
                               std::vector<glm::vec2>& tex_coords,
                               std::vector<std::pair<unsigned int, unsigned int>>& indices,
                               const StrandModelMeshGeneratorSettings& settings);

  static void MarchingCube(const StrandModel& strand_model, std::vector<Vertex>& vertices,
                           std::vector<unsigned int>& indices, const StrandModelMeshGeneratorSettings& settings);

  static void CylindricalMeshing(const StrandModel& strand_model, std::vector<Vertex>& vertices,
                                 std::vector<unsigned int>& indices, const StrandModelMeshGeneratorSettings& settings);

  static void MeshSmoothing(std::vector<Vertex>& vertices, std::vector<unsigned int>& indices);

  static void MeshSmoothing(std::vector<Vertex>& vertices, std::vector<std::pair<unsigned int, unsigned int>>& indices);

  static void CalculateNormal(std::vector<Vertex>& vertices, const std::vector<unsigned int>& indices);

  static void CalculateNormal(std::vector<Vertex>& vertices,
                              const std::vector<std::pair<unsigned int, unsigned int>>& indices);

  static void CalculateUv(const StrandModel& strand_model, std::vector<Vertex>& vertices,
                          const StrandModelMeshGeneratorSettings& settings);

 public:
  static void Generate(const StrandModel& strand_model, std::vector<Vertex>& vertices,
                       std::vector<unsigned int>& indices, const StrandModelMeshGeneratorSettings& settings);
  static void Generate(const StrandModel& strand_model, std::vector<Vertex>& vertices, std::vector<glm::vec2>& tex_coords,
                       std::vector<std::pair<unsigned int, unsigned int>>& indices,
                       const StrandModelMeshGeneratorSettings& settings);
};
}  // namespace eco_sys_lab