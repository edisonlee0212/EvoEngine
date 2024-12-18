#pragma once
#include "ForestDescriptor.hpp"
#include "SorghumDescriptor.hpp"
#include "SorghumField.hpp"
#include "SorghumGenerator.hpp"
#include "SorghumPointCloudScanner.hpp"
#include "TreeMeshGenerator.hpp"
#include "TreePointCloudScanner.hpp"
using namespace evo_engine;
using namespace eco_sys_lab_plugin;
using namespace digital_agriculture_plugin;
namespace dataset_generation_plugin {
class DatasetGenerator {
 public:
  struct TreeGrowthLimitation {
    int max_iteration = -1;
    int max_node_count = -1;
    int max_flow_count = -1;
    float low_branch_pruning = 0.f;
  };

  static void GenerateTreeTrunkMesh(const std::string& tree_parameters_path, float delta_time, int max_iterations,
                                    int max_tree_node_count, const TreeMeshGeneratorSettings& mesh_generator_settings,
                                    const std::string& tree_mesh_output_path, const std::string& tree_trunk_output_path,
                                    const std::string& tree_info_path);

  static void GenerateTreeMesh(const std::filesystem::path& tree_parameters_path, float low_branch_pruning,
                               float delta_time, int max_iterations, const std::vector<int>& target_tree_node_count,
                               const TreeMeshGeneratorSettings& mesh_generator_settings,
                               const std::string& tree_mesh_output_path);

  static void GenerateDataForTree(const TreePointCloudPointSettings& point_settings,
                                  const std::shared_ptr<PointCloudCaptureSettings>& capture_settings,
                                  const std::filesystem::path& tree_parameters_path, float delta_time,
                                  const TreeGrowthLimitation& tree_growth_limitation,
                                  const TreeMeshGeneratorSettings& mesh_generator_settings, bool export_point_cloud,
                                  const std::string& point_cloud_output_path, bool export_mesh,
                                  const std::string& mesh_output_path, bool export_skeleton,
                                  const std::string& skeleton_output_path);
  static void GeneratePointCloudForForest(int grid_size, float grid_distance, float random_shift,
                                          const TreePointCloudPointSettings& point_settings,
                                          const std::shared_ptr<PointCloudCaptureSettings>& capture_settings,
                                          const std::string& tree_parameters_folder_path, float delta_time,
                                          int max_iterations, int max_tree_node_count,
                                          const TreeMeshGeneratorSettings& mesh_generator_settings,
                                          const std::string& point_cloud_output_path);
  static void GeneratePointCloudForForestPatch(const glm::ivec2& grid_size,
                                               const TreePointCloudPointSettings& point_settings,
                                               const std::shared_ptr<PointCloudCaptureSettings>& capture_settings,
                                               const std::shared_ptr<ForestPatch>& forest_patch,
                                               const TreeMeshGeneratorSettings& mesh_generator_settings,
                                               const std::string& point_cloud_output_path);
  static void GeneratePointCloudForForestPatchJoinedSpecies(
      const glm::ivec2& grid_size, const TreePointCloudPointSettings& point_settings,
      const std::shared_ptr<PointCloudCaptureSettings>& capture_settings,
      const std::shared_ptr<ForestPatch>& forest_patch, const std::string& species_folder_path,
      const TreeMeshGeneratorSettings& mesh_generator_settings, const std::string& point_cloud_output_path);

  static void GeneratePointCloudForSorghum(const std::shared_ptr<SorghumDescriptor>& sorghum_descriptor,
                                           const SorghumPointCloudPointSettings& point_settings,
                                           const std::shared_ptr<PointCloudCaptureSettings>& capture_settings,
                                           const SorghumMeshGeneratorSettings& sorghum_mesh_generator_settings,
                                           bool avoid_occlusion, bool generate_ground,
                                           const std::filesystem::path& point_cloud_output_path);

  static void GeneratePointCloudForSorghum(const std::shared_ptr<SorghumState>& sorghum_state,
                                           const SorghumPointCloudPointSettings& point_settings,
                                           const std::shared_ptr<PointCloudCaptureSettings>& capture_settings,
                                           const SorghumMeshGeneratorSettings& sorghum_mesh_generator_settings,
                                           bool avoid_occlusion, bool generate_ground,
                                           const std::filesystem::path& point_cloud_output_path);

  static void GenerateMeshAndPointCloudForSorghum(const std::shared_ptr<SorghumDescriptor>& sorghum_descriptor,
                                                  const SorghumPointCloudPointSettings& point_settings,
                                                  const std::shared_ptr<PointCloudCaptureSettings>& capture_settings,
                                                  const SorghumMeshGeneratorSettings& sorghum_mesh_generator_settings,
                                                  bool avoid_occlusion, bool generate_ground,
                                                  const std::filesystem::path& mesh_output_path,
                                                  const std::filesystem::path& point_cloud_output_path);

  static void GenerateMeshAndPointCloudForSorghum(const std::shared_ptr<SorghumState>& sorghum_state,
                                                  const SorghumPointCloudPointSettings& point_settings,
                                                  const std::shared_ptr<PointCloudCaptureSettings>& capture_settings,
                                                  const SorghumMeshGeneratorSettings& sorghum_mesh_generator_settings,
                                                  bool avoid_occlusion, bool generate_ground,
                                                  const std::filesystem::path& mesh_output_path,
                                                  const std::filesystem::path& point_cloud_output_path);

  static void GenerateMeshForSorghum(const std::shared_ptr<SorghumState>& sorghum_state,
                                     const SorghumMeshGeneratorSettings& sorghum_mesh_generator_settings,
                                     const std::filesystem::path& mesh_output_path);

  static void GenerateMeshForSorghum(const std::shared_ptr<SorghumDescriptor>& sorghum_descriptor,
                                     const SorghumMeshGeneratorSettings& sorghum_mesh_generator_settings,
                                     const std::filesystem::path& mesh_output_path);

  static void GeneratePointCloudForSorghumPatch(const SorghumFieldPatch& pattern,
                                                const std::shared_ptr<SorghumGenerator>& sorghum_descriptor,
                                                const SorghumPointCloudPointSettings& point_settings,
                                                const std::shared_ptr<PointCloudCaptureSettings>& capture_settings,
                                                const SorghumMeshGeneratorSettings& sorghum_mesh_generator_settings,
                                                const std::filesystem::path& point_cloud_output_path);
};

}  // namespace dataset_generation_plugin