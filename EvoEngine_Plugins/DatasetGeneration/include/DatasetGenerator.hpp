#pragma once
#include "ForestDescriptor.hpp"
#include "SorghumDescriptor.hpp"
#include "SorghumField.hpp"
#include "SorghumGenerator.hpp"
#include "SorghumPointCloudScanner.hpp"
#include "TreeMeshGenerator.hpp"
#include "TreeModel.hpp"
#include "TreePointCloudScanner.hpp"
using namespace evo_engine;
using namespace eco_sys_lab_plugin;
using namespace digital_agriculture_plugin;
namespace dataset_generation_plugin {
class DatasetGenerator {
 public:
  struct CameraCaptureSettings {
    GlobalTransform global_transform{};
    CameraSettings camera_settings{};
    glm::uvec2 render_resolution = {2048, 2048};
    glm::uvec2 output_resolution = {1024, 1024};
  };

  struct TreeDataGenerationParameters {
    // Parameters
    std::filesystem::path tree_descriptor_path;
    std::filesystem::path foliage_descriptor_path;
    std::filesystem::path bark_descriptor_path;
    // Growth control
    SimulationSettings simulation_settings{};
    TreeGrowthSettings tree_growth_settings{};
    Tree::PruningSettings pruning_settings{};

    // Stop condition
    int max_iteration = -1;
    bool use_node_growth_capture = false;
    std::vector<int> growth_capture{};

    bool export_point_cloud = false;
    bool export_mesh = false;
    bool export_skeleton = false;
    bool export_rendering = false;
    bool export_depth = false;

    // Data generation
    bool generate_ground_mesh = false;
    TreePointCloudPointSettings tree_point_cloud_point_settings{};
    TreeMeshGeneratorSettings tree_mesh_generator_settings{};
    std::vector<CameraCaptureSettings> camera_capture_settings{};
    float max_depth = 20.f;
    // Export path
    std::filesystem::path output_folder;
    std::string output_file_prefix;
  };

  static void GenerateDataForTree(const TreeDataGenerationParameters& data_generation_parameters,
                                  const std::shared_ptr<PointCloudCaptureSettings>& capture_settings);

  static void GenerateDataForForest(int grid_size, float grid_distance, float random_shift,
                                    const TreeDataGenerationParameters& data_generation_parameters,
                                    const std::filesystem::path& species_folder_path,
                                    const std::shared_ptr<PointCloudCaptureSettings>& capture_settings);

  static void GeneratePointCloudForForestPatch(const glm::ivec2& grid_size,
                                               const std::shared_ptr<ForestPatch>& forest_patch,
                                               const TreeDataGenerationParameters& data_generation_parameters,
                                               const std::shared_ptr<PointCloudCaptureSettings>& capture_settings);

  static void GeneratePointCloudForForestPatchJoinedSpecies(
      const glm::ivec2& grid_size, const std::shared_ptr<ForestPatch>& forest_patch,
      const std::filesystem::path& species_folder_path, const TreeDataGenerationParameters& data_generation_parameters,
      const std::shared_ptr<PointCloudCaptureSettings>& capture_settings);

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