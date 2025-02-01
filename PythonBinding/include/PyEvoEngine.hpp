#pragma once
#include "AnimationPlayer.hpp"
#include "Application.hpp"
#include "ClassRegistry.hpp"
#include "EditorLayer.hpp"
#include "MeshRenderer.hpp"
#include "PlayerController.hpp"
#include "PostProcessingStack.hpp"
#include "Prefab.hpp"
#include "ProjectManager.hpp"
#include "RenderLayer.hpp"
#include "Scene.hpp"
#include "Times.hpp"
#include "WindowLayer.hpp"
#include "pybind11/pybind11.h"
#include "pybind11/stl/filesystem.h"

using namespace evo_engine;
namespace py_evo_engine {
class PyEvoEngine {
 public:
  static void Initialize();
};
}  // namespace py_evo_engine