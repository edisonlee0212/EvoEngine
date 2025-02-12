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
    glm::vec3 position;
    glm::vec3 euler_rotation;

    CameraSettings camera_settings{};
    glm::uvec2 render_resolution = {2048, 2048};
    glm::uvec2 output_resolution = {1024, 1024};
  };

  struct TreeDataGenerationParameters {
    // Parameters
    std::filesystem::path tree_descriptor_path{};
    std::filesystem::path foliage_descriptor_path{};
    std::filesystem::path bark_descriptor_path{};
    // Growth control
    SimulationSettings simulation_settings{};
    TreeGrowthSettings tree_growth_settings{};
    Tree::PruningSettings pruning_settings{};

    // Stop condition
    int max_iteration = -1;
    bool use_node_growth_capture = false;
    std::vector<int> growth_capture{};

    // Export types
    bool export_point_cloud = false;
    bool export_mesh = false;
    bool export_skeleton = false;
    bool export_rendering = false;
    bool export_depth = false;
    bool export_statistics = false;

    // Data generation
    bool generate_ground_mesh = false;
    TreePointCloudPointSettings tree_point_cloud_point_settings{};
    TreeMeshGeneratorSettings tree_mesh_generator_settings{};
    std::vector<CameraCaptureSettings> camera_capture_settings{};
    std::shared_ptr<PointCloudCaptureSettings> point_cloud_capture_settings{};
    int seed = 0;
    float max_depth = 20.f;
    // Export path
    std::filesystem::path output_folder{};
    std::string output_file_name{};
  };

  static void GenerateDataForTree(const TreeDataGenerationParameters& data_generation_parameters);

  static void GenerateDataForForest(int grid_size, float grid_distance, float random_shift,
                                    const TreeDataGenerationParameters& data_generation_parameters,
                                    const std::filesystem::path& species_folder_path);

  static void GeneratePointCloudForForestPatch(const glm::ivec2& grid_size,
                                               const std::shared_ptr<ForestPatch>& forest_patch,
                                               const TreeDataGenerationParameters& data_generation_parameters);

  static void GeneratePointCloudForForestPatchJoinedSpecies(
      const glm::ivec2& grid_size, const std::shared_ptr<ForestPatch>& forest_patch,
      const std::filesystem::path& species_folder_path, const TreeDataGenerationParameters& data_generation_parameters);

  struct SorghumDataGenerationParameters {
    // Export types
    bool export_point_cloud = false;
    bool export_mesh = false;

    // Data generation
    bool generate_ground_mesh = false;
    bool avoid_occlusion = false;
    SorghumPointCloudPointSettings sorghum_point_cloud_point_settings{};
    SorghumMeshGeneratorSettings sorghum_mesh_generator_settings{};
    std::shared_ptr<PointCloudCaptureSettings> point_cloud_capture_settings{};
    // Export path
    std::filesystem::path output_folder{};
    std::string output_file_name{};
  };

  static Entity CreateSorghumEntity(const std::filesystem::path& sorghum_path, int seed = 0);

  static Entity CreateSorghumEntity(const std::shared_ptr<IAsset>& sorghum_asset, int seed = 0);

  static void ApplySorghumGrid(const std::shared_ptr<IAsset>& target_sorghum_field,
                               const std::filesystem::path& sorghum_path, const SorghumGrid& sorghum_grid);

  static void ApplySorghumGrid(const std::shared_ptr<IAsset>& target_sorghum_field,
                               const std::shared_ptr<IAsset>& sorghum_generator, const SorghumGrid& sorghum_grid);

  static void GenerateDataForSorghum(const Entity& sorghum_entity,
                                     const SorghumDataGenerationParameters& data_generation_parameters);

  static void GenerateDataForAllSorghums(const SorghumDataGenerationParameters& data_generation_parameters);
};

}  // namespace dataset_generation_plugin