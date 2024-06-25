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

#pragma region Helpers
void PushWindowLayer() {
  Application::PushLayer<WindowLayer>();
}
void PushEditorLayer() {
  Application::PushLayer<EditorLayer>();
}
void PushRenderLayer() {
  Application::PushLayer<RenderLayer>();
}

Entity CreateDynamicCube(const float& mass, const glm::vec3& color, const glm::vec3& position,
                         const glm::vec3& rotation, const glm::vec3& scale, const std::string& name);

Entity CreateSolidCube(const float& mass, const glm::vec3& color, const glm::vec3& position, const glm::vec3& rotation,
                       const glm::vec3& scale, const std::string& name);

Entity CreateCube(const glm::vec3& color, const glm::vec3& position, const glm::vec3& rotation, const glm::vec3& scale,
                  const std::string& name);

Entity CreateDynamicSphere(const float& mass, const glm::vec3& color, const glm::vec3& position,
                           const glm::vec3& rotation, const float& scale, const std::string& name);

Entity CreateSolidSphere(const float& mass, const glm::vec3& color, const glm::vec3& position,
                         const glm::vec3& rotation, const float& scale, const std::string& name);

Entity CreateSphere(const glm::vec3& color, const glm::vec3& position, const glm::vec3& rotation, const float& scale,
                    const std::string& name);

enum class DemoSetup { Empty, Rendering, Galaxy, Planets };
Entity LoadScene(const std::shared_ptr<Scene>& scene, const std::string& base_entity_name, bool add_spheres);
void SetupDemo(DemoSetup demo_setup);
void SetupDemoScene(DemoSetup demo_setup, ApplicationInfo& application_info, bool enable_physics);
#pragma endregion

void CreateRenderingDemo() {
  DemoSetup demo_setup = DemoSetup::Rendering;
  SetupDemo(demo_setup);
}

void InitializePlanetsDemo() {
  DemoSetup demo_setup = DemoSetup::Planets;
  SetupDemo(demo_setup);
}

void RegisterLayers(const bool enable_window_layer, const bool enable_editor_layer) {
  if (enable_window_layer)
    Application::PushLayer<WindowLayer>();
  if (enable_window_layer && enable_editor_layer)
    Application::PushLayer<EditorLayer>();
  Application::PushLayer<RenderLayer>();
}

void StartProjectWindowless(const std::filesystem::path& project_path) {
  if (std::filesystem::path(project_path).extension().string() != ".eveproj") {
    EVOENGINE_ERROR("Project path doesn't point to a evo_engine project!");
    return;
  }
  RegisterLayers(false, false);
  ApplicationInfo application_info{};
  application_info.project_path = project_path;
  Application::Initialize(application_info);
  Application::Start();
}

void StartProjectWithEditor(const std::filesystem::path& project_path) {
  if (!project_path.empty()) {
    if (std::filesystem::path(project_path).extension().string() != ".eveproj") {
      EVOENGINE_ERROR("Project path doesn't point to a evo_engine project!");
      return;
    }
  }
  RegisterLayers(false, false);
  ApplicationInfo application_info{};
  application_info.project_path = project_path;
  Application::Initialize(application_info);
  Application::Start();
  Application::Run();
}

void CaptureActiveScene(const int resolution_x, const int resolution_y, const std::string& output_path) {
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

PYBIND11_MODULE(pyevoengine, m) {
  py::class_<Entity>(m, "Entity").def("get_index", &Entity::GetIndex).def("get_version", &Entity::GetVersion);

  py::class_<Scene>(m, "Scene")
      .def("create_entity", static_cast<Entity (Scene::*)(const std::string&)>(&Scene::CreateEntity))
      .def("delete_entity", &Scene::DeleteEntity)
      .def("save", &Scene::Save);
  py::class_<ApplicationInfo>(m, "ApplicationInfo")
      .def(py::init<>())
      .def_readwrite("project_path_", &ApplicationInfo::project_path)
      .def_readwrite("application_name", &ApplicationInfo::application_name)
      .def_readwrite("enable_docking", &ApplicationInfo::enable_docking)
      .def_readwrite("enable_viewport", &ApplicationInfo::enable_viewport)
      .def_readwrite("full_screen", &ApplicationInfo::full_screen);

  py::class_<Application>(m, "Application")
      .def_static("initialize", &Application::Initialize)
      .def_static("start", &Application::Start)
      .def_static("run", &Application::Run)
      .def_static("loop", &Application::Loop)
      .def_static("terminate", &Application::Terminate)
      .def_static("get_active_scene", &Application::GetActiveScene);

  py::class_<ProjectManager>(m, "ProjectManager")
      .def_static("GetOrCreateProject", &ProjectManager::GetOrCreateProject)
      .def_static("SaveProject", &ProjectManager::SaveProject);
  m.doc() = "evo_engine";  // optional module docstring

  m.def("create_rendering_demo", &CreateRenderingDemo, "Create Rendering Demo");

  m.def("register_layers", &RegisterLayers, "RegisterLayers");
  m.def("start_project_windowless", &StartProjectWindowless, "StartProjectWindowless");
  m.def("start_project_with_editor", &StartProjectWithEditor, "StartProjectWithEditor");

  m.def("capture_active_scene", &CaptureActiveScene, "CaptureActiveScene");
}

void SetupDemo(DemoSetup demo_setup) {
  Application::PushLayer<RenderLayer>();
  const ApplicationInfo application_info{};
  Application::Initialize(application_info);
  Application::Start();
}
