#pragma once
#include "ForestDescriptor.hpp"
#include "SorghumDescriptor.hpp"
#include "SorghumDescriptorGenerator.hpp"
#include "SorghumField.hpp"
#include "SorghumPointCloudScanner.hpp"
#include "TreeMeshGenerator.hpp"
#include "TreePointCloudScanner.hpp"
using namespace evo_engine;
using namespace eco_sys_lab_plugin;
using namespace digital_agriculture_plugin;
namespace dataset_generation_plugin {
class DatasetGenerator {
 public:
  static void GenerateTreeTrunkMesh(const std::string& tree_parameters_path, float delta_time, int max_iterations,
                                    int max_tree_node_count, const TreeMeshGeneratorSettings& mesh_generator_settings,
                                    const std::string& tree_mesh_output_path, const std::string& tree_trunk_output_path,
                                    const std::string& tree_info_path);

  static auto GenerateTreeMesh(const std::string& tree_parameters_path, float delta_time, int max_iterations,
                               int max_tree_node_count, const TreeMeshGeneratorSettings& mesh_generator_settings,
                               const std::string& tree_mesh_output_path) -> void;

  static void GenerateTreeMesh(const std::string& tree_parameters_path, float delta_time, int max_iterations,
                               std::vector<int> target_tree_node_count,
                               const TreeMeshGeneratorSettings& mesh_generator_settings,
                               const std::string& tree_mesh_output_path);

  static void GeneratePointCloudForTree(const TreePointCloudPointSettings& point_settings,
                                        const std::shared_ptr<PointCloudCaptureSettings>& capture_settings,
                                        const std::string& tree_parameters_path, float delta_time, int max_iterations,
                                        int max_tree_node_count, const TreeMeshGeneratorSettings& mesh_generator_settings,
                                        const std::string& point_cloud_output_path, bool export_tree_mesh,
                                        const std::string& tree_mesh_output_path);
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
                                           bool avoid_occlusion, const std::filesystem::path& point_cloud_output_path);

  static void GeneratePointCloudForSorghumPatch(const SorghumFieldPatch& pattern,
                                                const std::shared_ptr<SorghumDescriptorGenerator>& sorghum_descriptor,
                                                const SorghumPointCloudPointSettings& point_settings,
                                                const std::shared_ptr<PointCloudCaptureSettings>& capture_settings,
                                                const SorghumMeshGeneratorSettings& sorghum_mesh_generator_settings,
                                                const std::filesystem::path& point_cloud_output_path);
};

}  // namespace dataset_generation_plugin