#pragma once
#include "ForestDescriptor.hpp"
#include "SorghumField.hpp"
#include "SorghumPointCloudScanner.hpp"
#include "SorghumDescriptor.hpp"
#include "SorghumDescriptorGenerator.hpp"
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

  static void GenerateTreeMesh(const std::string& treeParametersPath, float deltaTime, int maxIterations,
                               int maxTreeNodeCount, const TreeMeshGeneratorSettings& meshGeneratorSettings,
                               const std::string& treeMeshOutputPath);

  static void GenerateTreeMesh(const std::string& treeParametersPath, float deltaTime, int maxIterations,
                               std::vector<int> targetTreeNodeCount,
                               const TreeMeshGeneratorSettings& meshGeneratorSettings,
                               const std::string& treeMeshOutputPath);

  static void GeneratePointCloudForTree(const TreePointCloudPointSettings& pointSettings,
                                        const std::shared_ptr<PointCloudCaptureSettings>& captureSettings,
                                        const std::string& treeParametersPath, float deltaTime, int maxIterations,
                                        int maxTreeNodeCount, const TreeMeshGeneratorSettings& meshGeneratorSettings,
                                        const std::string& pointCloudOutputPath, bool exportTreeMesh,
                                        const std::string& treeMeshOutputPath);
  static void GeneratePointCloudForForest(int gridSize, float gridDistance, float randomShift,
                                          const TreePointCloudPointSettings& pointSettings,
                                          const std::shared_ptr<PointCloudCaptureSettings>& captureSettings,
                                          const std::string& treeParametersFolderPath, float deltaTime,
                                          int maxIterations, int maxTreeNodeCount,
                                          const TreeMeshGeneratorSettings& meshGeneratorSettings,
                                          const std::string& pointCloudOutputPath);
  static void GeneratePointCloudForForestPatch(const glm::ivec2& gridSize,
                                               const TreePointCloudPointSettings& pointSettings,
                                               const std::shared_ptr<PointCloudCaptureSettings>& captureSettings,
                                               const std::shared_ptr<ForestPatch>& forestPatch,
                                               const TreeMeshGeneratorSettings& meshGeneratorSettings,
                                               const std::string& pointCloudOutputPath);
  static void GeneratePointCloudForForestPatchJoinedSpecies(
      const glm::ivec2& grid_size, const TreePointCloudPointSettings& point_settings,
      const std::shared_ptr<PointCloudCaptureSettings>& capture_settings,
      const std::shared_ptr<ForestPatch>& forest_patch, const std::string& species_folder_path,
      const TreeMeshGeneratorSettings& mesh_generator_settings, const std::string& point_cloud_output_path);
  static void GeneratePointCloudForSorghumPatch(const SorghumFieldPatch& pattern,
                                                const std::shared_ptr<SorghumDescriptorGenerator>& sorghumDescriptor,
                                                const SorghumPointCloudPointSettings& point_settings,
                                                const std::shared_ptr<PointCloudCaptureSettings>& capture_settings,
                                                const SorghumMeshGeneratorSettings& sorghum_mesh_generator_settings,
                                                const std::string& point_cloud_output_path);
};

}  // namespace eco_sys_lab_plugin