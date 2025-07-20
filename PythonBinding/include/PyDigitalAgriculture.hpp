#pragma once

#ifdef DIGITAL_AGRICULTURE_PACKAGE

#  include "AnimationPlayer.hpp"
#  include "Application.hpp"
#  include "ClassRegistry.hpp"
#  include "Climate.hpp"
#  include "EditorLayer.hpp"
#  include "HeightField.hpp"
#  include "MeshRenderer.hpp"
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
#  include "pybind11/stl/filesystem.h"

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
                                       const int seed, const int index=200, const float radius=2.5f);

  static std::vector<std::vector<glm::vec3>> GetAllIlluminationEstimationResultsOnSorghum();

  static Handle SetPARSensors(const Entity& sorghum_field);

  static void IlluminationEstimationOnSensors(const Handle& sensor_group_handle);

  static std::vector<std::vector<glm::vec3>> GetAllIlluminationEstimationResultsFromSensors(
      const Handle& sensor_group_handle);
};

}  // namespace py_digital_agriculture_plugin

#endif
