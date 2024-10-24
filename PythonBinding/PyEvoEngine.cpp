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

namespace py = pybind11;

void push_layers(const bool enable_window_layer, const bool enable_editor_layer) {
  if (enable_window_layer)
    Application::PushLayer<WindowLayer>();
  if (enable_window_layer && enable_editor_layer)
    Application::PushLayer<EditorLayer>();
  Application::PushLayer<RenderLayer>();
}

void engine_run_windowless(const std::filesystem::path& project_path) {
  if (std::filesystem::path(project_path).extension().string() != ".eveproj") {
    EVOENGINE_ERROR("Project path doesn't point to a EvoEngine project!");
    return;
  }
  push_layers(false, false);
  ApplicationInfo application_info{};
  application_info.project_path = project_path;
  Application::Initialize(application_info);
  Application::Start();
}

void engine_run(const std::filesystem::path& project_path) {
  if (!project_path.empty()) {
    if (std::filesystem::path(project_path).extension().string() != ".eveproj") {
      EVOENGINE_ERROR("Project path doesn't point to a EvoEngine project!");
      return;
    }
  }
  push_layers(true, false);
  ApplicationInfo application_info{};
  application_info.project_path = project_path;
  Application::Initialize(application_info);
  Application::Start();
}

void engine_run_with_editor(const std::filesystem::path& project_path) {
  if (!project_path.empty()) {
    if (std::filesystem::path(project_path).extension().string() != ".eveproj") {
      EVOENGINE_ERROR("Project path doesn't point to a EvoEngine project!");
      return;
    }
  }
  push_layers(true, true);
  ApplicationInfo application_info{};
  application_info.project_path = project_path;
  Application::Initialize(application_info);
  Application::Start();
}

void engine_loop() {
  Application::Loop();
}

void engine_terminate() {
  Application::Terminate();
}

void capture_current_scene(const int resolution_x, const int resolution_y, const std::string& output_path) {
  if (resolution_x <= 0 || resolution_y <= 0) {
    EVOENGINE_ERROR("Resolution error!");
    return;
  }

  const auto scene = Application::GetActiveScene();
  if (!scene) {
    EVOENGINE_ERROR("No active scene!");
    return;
  }
  const auto main_camera = scene->main_camera.Get<Camera>();
  if (!main_camera) {
    EVOENGINE_ERROR("No main camera in scene!");
    return;
  }
  main_camera->Resize({resolution_x, resolution_y});
  Application::Loop();
  main_camera->GetRenderTexture()->StoreToPng(output_path);
  EVOENGINE_LOG("Exported image to " + output_path);
}


PYBIND11_MODULE(PyEvoEngine, m) {
  m.doc() = "PyEvoEngine";  // optional module docstring
  m.def("engine_run_windowless", &engine_run_windowless, "Start Project (Windowless)");
  m.def("engine_run", &engine_run, "Start Project (No Editor)");
  m.def("engine_run_with_editor", &engine_run_with_editor, "Start Project (with Editor)");
  m.def("engine_loop", &engine_loop, "Loop Application");
  m.def("engine_terminate", &engine_terminate, "Terminate Application");

  m.def("capture_current_scene", &capture_current_scene, "Capture current scene");
}