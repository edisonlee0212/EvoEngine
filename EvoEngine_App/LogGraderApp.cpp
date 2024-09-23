// PlantFactory.cpp : This file contains the 'main' function. Program execution
// begins and ends there.
//
#include <Application.hpp>
#include "ClassRegistry.hpp"

#include "ProjectManager.hpp"

#include "WindowLayer.hpp"
#ifdef ECOSYSLAB_PLUGIN
#  include "BarkDescriptor.hpp"
#  include "HeightField.hpp"
#  include "Tree.hpp"
using namespace eco_sys_lab_plugin;
#endif
#ifdef LOG_SCANNING_PLUGIN
#  include "JoeScanScanner.hpp"
using namespace log_scanning_plugin;
#endif
#ifdef LOG_GRADING_PLUGIN
#  include "LogGrader.hpp"
using namespace log_grading_plugin;
#endif

#include "EditorLayer.hpp"
using namespace evo_engine;

void EngineSetup();

int main() {
  EngineSetup();

  Application::PushLayer<WindowLayer>();
  Application::PushLayer<EditorLayer>();
  Application::PushLayer<RenderLayer>();
#ifdef LOG_GRADING_PLUGIN
  PrivateComponentRegistration<LogGrader>("LogGrader");
#endif
#ifdef ECOSYSLAB_PLUGIN
  AssetRegistration<BarkDescriptor>("BarkDescriptor", {".bs"});
#endif

#ifdef LOG_SCANNING_PLUGIN
  AssetRegistration<JoeScan>("JoeScan", {".jscan"});
  PrivateComponentRegistration<JoeScanScanner>("JoeScanScanner");
#endif
  ApplicationInfo application_configs;
  application_configs.application_name = "Log Grader";
  std::filesystem::create_directory(std::filesystem::path(".") / "LogGraderProject");
  application_configs.project_path =
      std::filesystem::absolute(std::filesystem::path(".") / "LogGraderProject" / "Default.eveproj");
  Application::Initialize(application_configs);

  // adjust default camera speed
  const auto editor_layer = Application::GetLayer<EditorLayer>();
  editor_layer->velocity = 2.f;
  editor_layer->default_scene_camera_position = glm::vec3(1.124, 0.218, 14.089);
  // override default scene camera position etc.
  editor_layer->default_scene_camera_position = glm::vec3(0, 2.5, 6);
  editor_layer->SetCameraPosition(editor_layer->GetSceneCamera(), editor_layer->default_scene_camera_position);
  editor_layer->GetSceneCamera()->clear_color = glm::vec3(1.f);
  const auto render_layer = Application::GetLayer<RenderLayer>();

  ProjectManager::GetInstance().show_project_window = false;
#pragma region Engine Loop
  Application::Start();
  Application::Run();
#pragma endregion
  Application::Terminate();
}

void EngineSetup() {
  ProjectManager::SetActionAfterSceneLoad([=](const std::shared_ptr<Scene>& scene) {
#ifdef LOG_GRADING_PLUGIN
#  pragma region Engine Setup
#  pragma endregion
    std::vector<Entity> entities;
    scene->GetAllEntities(entities);
    bool found = false;
    for (const auto& entity : entities) {
      if (scene->HasPrivateComponent<LogGrader>(entity)) {
        const auto editor_layer = Application::GetLayer<EditorLayer>();
        editor_layer->SetSelectedEntity(entity);
        editor_layer->SetLockEntitySelection(true);
        found = true;
        break;
      }
    }
    if (!found) {
      const auto entity = scene->CreateEntity("LogGrader");
      scene->GetOrSetPrivateComponent<LogGrader>(entity);
      const auto editor_layer = Application::GetLayer<EditorLayer>();
      editor_layer->SetSelectedEntity(entity);
      editor_layer->SetLockEntitySelection(true);
    }
  });
  ProjectManager::SetActionAfterNewScene([=](const std::shared_ptr<Scene>& scene) {
#  pragma region Engine Setup
#  pragma region Preparations
    Times::SetTimeStep(0.016f);
#  pragma endregion
#  pragma endregion
    const auto entity = scene->CreateEntity("LogGrader");
    scene->GetOrSetPrivateComponent<LogGrader>(entity);
    const auto editor_layer = Application::GetLayer<EditorLayer>();
    editor_layer->SetSelectedEntity(entity);
    editor_layer->SetLockEntitySelection(true);
#endif
  });
}
