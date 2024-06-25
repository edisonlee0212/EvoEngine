// PlantFactory.cpp : This file contains the 'main' function. Program execution
// begins and ends there.
//
#include <Application.hpp>

#ifdef BUILD_WITH_RAYTRACER
#  include <CUDAModule.hpp>
#  include <RayTracerLayer.hpp>
#endif

#include "Times.hpp"

#include "ClassRegistry.hpp"
#include "Climate.hpp"
#include "EcoSysLabLayer.hpp"
#include "ForestDescriptor.hpp"
#include "HeightField.hpp"
#include "ObjectRotator.hpp"
#include "ParticlePhysics2DDemo.hpp"
#include "Physics2DDemo.hpp"
#include "ProjectManager.hpp"
#include "RadialBoundingVolume.hpp"
#include "Soil.hpp"
#include "SorghumLayer.hpp"
#include "Tree.hpp"
#include "TreeModel.hpp"
#include "TreePointCloudScanner.hpp"
#include "TreeStructor.hpp"
#include "WindowLayer.hpp"
using namespace eco_sys_lab;

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
#ifdef BUILD_WITH_RAYTRACER
  Application::PushLayer<RayTracerLayer>();
#endif
  Application::PushLayer<EcoSysLabLayer>();
  Application::PushLayer<SorghumLayer>();
  ClassRegistry::RegisterPrivateComponent<ObjectRotator>("ObjectRotator");
  ClassRegistry::RegisterPrivateComponent<Physics2DDemo>("Physics2DDemo");
  ClassRegistry::RegisterPrivateComponent<ParticlePhysics2DDemo>("ParticlePhysics2DDemo");

  ApplicationInfo applicationConfigs;
  applicationConfigs.application_name = "SorghumFactory";
  applicationConfigs.project_path = std::filesystem::absolute(resourceFolderPath / "SorghumProject" / "test.eveproj");
  Application::Initialize(applicationConfigs);

#ifdef BUILD_WITH_RAYTRACER

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
    auto mainCamera = Application::GetActiveScene()->main_camera.Get<Camera>();
    if (mainCamera) {
      scene->SetDataComponent(mainCamera->GetOwner(), transform);
      mainCamera->use_clear_color = true;
      mainCamera->clear_color = glm::vec3(0.5f);
    }
#pragma endregion
#pragma endregion
  });
}
