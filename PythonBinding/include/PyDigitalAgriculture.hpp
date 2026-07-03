#pragma once

#ifdef DIGITAL_AGRICULTURE_PACKAGE

//#  include "AnimationPlayer.hpp"
//#  include "Application.hpp"
//#  include "ClassRegistry.hpp"
//#  include "Climate.hpp"
//#  include "EditorLayer.hpp"
//#  include "HeightField.hpp"
//#  include "MeshRenderer.hpp"
#  include "EditorLayer.hpp"
#  include "IPrivateComponent.hpp"
#  include "ObjectRotator.hpp"
#  include "PlayerController.hpp"
#  include "PostProcessingStack.hpp"
#  include "Prefab.hpp"
#  include "ProjectManager.hpp"
#  include "RadialBoundingVolume.hpp"
#  include "RenderLayer.hpp"
#  include "Scene.hpp"
#  include "ShootModel.hpp"
#  include "Soil.hpp"
#  include "SorghumLayer.hpp"
#  include "Times.hpp"
#  include "Tree.hpp"
#  include "TreeStructor.hpp"
#  include "WindowLayer.hpp"
#  include "pybind11/pybind11.h"
#  include "pybind11/stl.h"
#  include "pybind11/stl/filesystem.h"
#  include <map>

#  if DATASET_GENERATION_PACKAGE
#    include <SorghumPointCloudScanner.hpp>
#    include "DatasetGenerator.hpp"
#  endif
#  include "PyEvoEngine.hpp"

namespace py_digital_agriculture_package {
#  if DATASET_GENERATION_PACKAGE
using namespace dataset_generation_package;
#  endif

using namespace evo_engine;
using namespace py_evo_engine;
using namespace digital_agriculture_package;

struct ParbarProbeRecord {
  std::string cultivar;
  std::string model;
  std::string sensor_bar_level;
  std::string height_rule;
  uint32_t row = 0;
  uint32_t column = 0;
  uint32_t represented_plant_count = 0;
  glm::vec3 position = glm::vec3(0.0f);
  glm::vec3 normal = glm::vec3(0.0f, 1.0f, 0.0f);
  glm::vec3 energy = glm::vec3(0.0f);
  glm::vec3 direction = glm::vec3(0.0f);
  float average_represented_root_elevation_m = 0.0f;
  float average_represented_plant_height_m = 0.0f;
  float sensor_top_elevation_m = 0.0f;
  float height_fraction_of_average_height = 0.0f;
  float scalar = 0.0f;
  float normalized = 0.0f;
};

struct LSystemGridIlluminationRecord {
  std::string name;
  std::string cultivar;
  uint32_t row = 0;
  uint32_t column = 0;
  glm::vec3 position = glm::vec3(0.0f);
  uint32_t triangle_count = 0;
  uint32_t leaf_triangle_count = 0;
  uint32_t stem_triangle_count = 0;
  float area = 0.0f;
  float leaf_area = 0.0f;
  float stem_area = 0.0f;
  float plant_height_m = 0.0f;
  glm::vec3 total_flux = glm::vec3(0.0f);
  glm::vec3 average_flux = glm::vec3(0.0f);
  float scalar = 0.0f;
  glm::vec3 isolated_total_flux = glm::vec3(0.0f);
  glm::vec3 isolated_average_flux = glm::vec3(0.0f);
  float isolated_scalar = 0.0f;
  float retention_ratio = 1.0f;
  float shadow_loss = 0.0f;
  float normalized = 0.0f;
};

struct LSystemPlantHeightFitRecord {
  std::string date;
  std::string cultivar;
  std::string plant_name;
  std::string base_plant_name;
  std::string scene_asset_path;
  std::string descriptor_asset_path;
  uint32_t cluster_index = 0;
  uint32_t cluster_size = 1;
  float cluster_offset_x_m = 0.0f;
  float cluster_offset_z_m = 0.0f;
  float cluster_offset_radius_m = 0.0f;
  float clump_mean_height_m = 0.0f;
  float target_height_m = 0.0f;
  float pre_fit_height_m = 0.0f;
  float final_height_m = 0.0f;
  float optimized_descriptor_scale = 1.0f;
  float per_plant_scale = 1.0f;
  float leaf_modules_mean = 0.0f;
  float leaf_modules_deviation = 0.0f;
  float leaf_thickness_m = 0.001f;
  uint32_t leaf_count = 0;
  float middle_parbar_top_elevation_m = 0.0f;
};

class PyDigitalAgriculture {
 public:
  /**
   * @brief Add SorghumLayer to the framework.
   */
  static void PushSorghumLayer();
  static void RegisterClasses();
  static Entity CreateEntityFromSorghumState(const Handle& sorghum_handle);
  static Entity CreateEntityFromSorghumDescriptor(const Handle& sorghum_handle);
  static Entity CreateEntityFromSorghumGenerator(const Handle& sorghum_generator_handle, int seed);
  static Entity CreateEntityFromSorghumField(const Handle& sorghum_generator_handle, int seed);
  static void ApplySorghumGrid(const Handle& sorghum_field_handle, const Handle& sorghum_generator_handle,
                               const SorghumGrid& sorghum_grid);

  /**
   * @brief Register for python binding.
   * @param m The target python binding module to register functions and classes.
   */
  static void Initialize(pybind11::module& m);

  static void EnableBTF();

  static void SetCBTFGroup(const Handle& cbtf_group_handle);

  static bool CheckBTFComponentsExist();

  static void SetSkyDome();

  static void PushRayTracerLayer();

  static void SetSunDirection(glm::vec3 angles);

  static void IlluminationEstimationOnSorghum();

  static void CheckTriangleEstimator(const Entity& sorghum_entity);

  static Entity InstantiateSorghumField(const Handle& sorghum_field_handle, const Handle& sorghum_coordinates,
                                       const int seed, const int index=200, const float radius=2000.0f);

  static std::vector<std::vector<glm::vec3>> GetAllIlluminationEstimationResultsOnSorghum();

  static Handle SetPARSensors(const Entity& sorghum_field);

  static void IlluminationEstimationOnSensors(const Handle& sensor_group_handle);

  static std::vector<std::vector<glm::vec3>> GetAllIlluminationEstimationResultsFromSensors(
      const Handle& sensor_group_handle);

  static void SetIlluminationSamples(int samples, int bounces);

  static bool RunLSystemSorghumProject(const std::filesystem::path& project_path,
                                       const std::filesystem::path& runtime_package_path = {},
                                       const std::filesystem::path& start_scene_path = {},
                                       bool load_project_assets = false);

  static bool WaitForProjectIdle(int max_frames = 30000);

  static void LoopFrames(int frames);

  static size_t GrowSorghumLsPlantsToAdulthood();

  static size_t SetSorghumLsLeafThickness(float leaf_thickness_m, bool regenerate_geometry = true);

  static size_t SetSorghumLsGridSpacing(float spacing_x, float spacing_z);

  static size_t MoveParbarMiddlePanelsToPlantHeightFraction(float height_fraction = 2.0f / 3.0f);

  static Handle CreateParbarTopFaceSensorGroup(uint32_t samples_per_panel = 100);

  static void EstimatePARSensors(const Handle& sensor_group_handle, int samples = 64, int bounces = 4,
                                 float push_normal_distance = 0.001f, int seed = 0);

  static std::vector<ParbarProbeRecord> GetParbarTopFaceSensorResults(const Handle& sensor_group_handle,
                                                                      uint32_t samples_per_panel = 100);

  static std::vector<LSystemGridIlluminationRecord> EstimateSorghumLsGridIllumination(
      int samples = 64, int bounces = 4, int max_triangles_per_plant = 0,
      float push_normal_distance = 0.001f, int seed = 0);

  static std::vector<LSystemPlantHeightFitRecord> FitSorghumLsDateHeightScene(
      const std::string& date, const std::map<std::string, float>& target_heights_m,
      float leaf_modules_mean, float leaf_modules_deviation, const std::filesystem::path& descriptor_folder,
      const std::filesystem::path& scene_asset_path, int optimizer_sample_count = 240,
      float tolerance_m = 0.005f, int max_fit_iterations = 6, float leaf_thickness_m = 0.001f,
      int cluster_min_count = 1, int cluster_max_count = 1, float cluster_radius_m = 0.0f);

  static bool SaveActiveSceneAsProjectAsset(const std::filesystem::path& scene_asset_path);

  static Entity CreateEntityFromPrefab(const Handle& prefab_handle, const glm::vec3& position,
                                       const glm::vec3& euler_rotation, const glm::vec3& scale);
};

}  // namespace py_digital_agriculture_plugin

#endif
