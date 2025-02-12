#pragma once

#ifdef DIGITAL_AGRICULTURE_PLUGIN

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
#  include "Soil.hpp"
#  include "SorghumLayer.hpp"
#  include "Times.hpp"
#  include "Tree.hpp"
#  include "TreeModel.hpp"
#  include "TreeStructor.hpp"
#  include "WindowLayer.hpp"
#  include "pybind11/pybind11.h"
#  include "pybind11/stl/filesystem.h"
#  ifdef CUDA_MODULE_PLUGIN
#    include <CUDAModule.hpp>
#    include <RayTracerLayer.hpp>
#  endif

#  if DATASET_GENERATION_PLUGIN
#    include <SorghumPointCloudScanner.hpp>
#    include "DatasetGenerator.hpp"
using namespace dataset_generation_plugin;
#  endif
#  include "PyEvoEngine.hpp"

using namespace evo_engine;
using namespace py_evo_engine;
using namespace digital_agriculture_plugin;

namespace py_digital_agriculture_plugin {
class PyDigitalAgriculture {
  EVOENGINE_SINGLETON_INSTANCE(PyDigitalAgriculture)
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
};
}  // namespace py_digital_agriculture_plugin

#endif