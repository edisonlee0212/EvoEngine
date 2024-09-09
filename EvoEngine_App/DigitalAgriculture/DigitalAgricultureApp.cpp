// PlantFactory.cpp : This file contains the 'main' function. Program execution
// begins and ends there.
//
#include <Application.hpp>

#ifdef OPTIX_RAY_TRACER_PLUGIN
#  include <CUDAModule.hpp>
#  include <RayTracerLayer.hpp>
#endif
#include "Times.hpp"
#include "ClassRegistry.hpp"
#include "ObjectRotator.hpp"
#include "ProjectManager.hpp"
#include "SorghumLayer.hpp"
#include "WindowLayer.hpp"
using namespace digital_agriculture_plugin;

void EngineSetup();

int main() {
  std::filesystem::path resourceFolderPath("../../../Resources");
  if (!std::filesystem::exists(resourceFolderPath)) {
    resourceFolderPath = "../../Resources";
  }
  if (!std::filesystem::exists(resourceFolderPath)) {
    resourceFolderPath = "../Resources";
  }
  if (std::filesystem::exists(resourceFolderPath)) {
    for (auto i : std::filesystem::recursive_directory_iterator(resourceFolderPath)) {
      if (i.is_directory())
        continue;
      auto oldPath = i.path();
      auto newPath = i.path();
      bool remove = false;
      if (i.path().extension().string() == ".uescene") {
        newPath.replace_extension(".evescene");
        remove = true;
      }
      if (i.path().extension().string() == ".umeta") {
        newPath.replace_extension(".evefilemeta");
        remove = true;
      }
      if (i.path().extension().string() == ".ueproj") {
        newPath.replace_extension(".eveproj");
        remove = true;
      }
      if (i.path().extension().string() == ".ufmeta") {
        newPath.replace_extension(".evefoldermeta");
        remove = true;
      }
      if (remove) {
        std::filesystem::copy(oldPath, newPath);
        std::filesystem::remove(oldPath);
      }
    }
  }

  EngineSetup();

  Application::PushLayer<WindowLayer>();
  Application::PushLayer<EditorLayer>();
  Application::PushLayer<RenderLayer>();
#ifdef OPTIX_RAY_TRACER_PLUGIN
  Application::PushLayer<RayTracerLayer>();
#endif
#ifdef DIGITAL_AGRICULTURE_PLUGIN
  Application::PushLayer<SorghumLayer>();
#endif
  PrivateComponentRegistration<eco_sys_lab_plugin::ObjectRotator>("ObjectRotator");
  ApplicationInfo applicationConfigs;
  applicationConfigs.application_name = "DigitalAgriculture";
  applicationConfigs.project_path = std::filesystem::absolute(resourceFolderPath / "DigitalAgricultureProject" / "test.eveproj");
  Application::Initialize(applicationConfigs);

#ifdef OPTIX_RAY_TRACER_PLUGIN

  auto rayTracerLayer = Application::GetLayer<RayTracerLayer>();
#endif

  // adjust default camera speed
  const auto editorLayer = Application::GetLayer<EditorLayer>();
  editorLayer->velocity = 2.f;
  editorLayer->default_scene_camera_position = glm::vec3(1.124, 0.218, 14.089);
  // override default scene camera position etc.
  editorLayer->show_camera_window = false;
  editorLayer->show_scene_window = true;
  editorLayer->show_entity_explorer_window = true;
  editorLayer->show_entity_inspector_window = true;
  const auto renderLayer = Application::GetLayer<RenderLayer>();
#pragma region Engine Loop
  Application::Start();
  Application::Run();
#pragma endregion
  Application::Terminate();
}

void EngineSetup() {
  ProjectManager::SetActionAfterNewScene([=](const std::shared_ptr<Scene>& scene) {
#pragma region Engine Setup
    Transform transform;
    transform.SetEulerRotation(glm::radians(glm::vec3(150, 30, 0)));
#pragma region Preparations
    Times::SetTimeStep(0.016f);
    transform = Transform();
    transform.SetPosition(glm::vec3(0, 2, 35));
    transform.SetEulerRotation(glm::radians(glm::vec3(15, 0, 0)));
    if (auto mainCamera = Application::GetActiveScene()->main_camera.Get<Camera>()) {
      scene->SetDataComponent(mainCamera->GetOwner(), transform);
      mainCamera->use_clear_color = true;
      mainCamera->clear_color = glm::vec3(0.5f);
    }
#pragma endregion
#pragma endregion
  });
}
