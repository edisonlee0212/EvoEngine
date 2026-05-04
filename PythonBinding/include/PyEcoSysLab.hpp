#pragma once

#ifdef ECOSYSLAB_PLUGIN
#  include "AnimationPlayer.hpp"
#  include "Application.hpp"
#  include "AssetManager.hpp"
#  include "ClassRegistry.hpp"
#  include "EditorLayer.hpp"
#  include "MeshRenderer.hpp"
#  include "PlayerController.hpp"
#  include "PostProcessingStack.hpp"
#  include "Prefab.hpp"
#  include "ProjectManager.hpp"

#  include "RenderLayer.hpp"
#  include "Scene.hpp"

#  include <pybind11/stl_bind.h>
#  include "Times.hpp"
#  include "WindowLayer.hpp"
#  include "pybind11/pybind11.h"
#  include "pybind11/stl/filesystem.h"

#  if DATASET_GENERATION_PLUGIN
#    include <TreePointCloudScanner.hpp>
#    include "DatasetGenerator.hpp"
#  endif

#  include "Climate.hpp"
#  include "EcoSysLabLayer.hpp"
#  include "HeightField.hpp"
#  include "ObjectRotator.hpp"

#  include "BasicFoliageDescriptor.hpp"
#  include "ParticlePhysics2DDemo.hpp"
#  include "Physics2DDemo.hpp"
#  include "RadialBoundingVolume.hpp"
#  include "ShootModel.hpp"
#  include "Soil.hpp"
#  include "Tree.hpp"
#  include "TreeStructor.hpp"

#  include "PyEvoEngine.hpp"

namespace py_eco_sys_lab_plugin {
#  if DATASET_GENERATION_PLUGIN
using namespace dataset_generation_plugin;
#  endif
using namespace evo_engine;
using namespace py_evo_engine;
using namespace eco_sys_lab_plugin;

class PyEcoSysLab {
 public:
  /**
   * @brief Add EcoSysLabLayer to the framework.
   */
  static void PushEcoSysLabLayer();
  static void RegisterClasses();
  /**
   * @brief Register for python binding.
   * @param m The target python binding module to register functions and classes.
   */
  static void Initialize(pybind11::module& m);
};
}  // namespace py_eco_sys_lab_plugin

#endif
