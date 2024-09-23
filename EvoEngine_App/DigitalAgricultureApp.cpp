// PlantFactory.cpp : This file contains the 'main' function. Program execution
// begins and ends there.
//
#include <Application.hpp>

#ifdef OPTIX_RAY_TRACER_PLUGIN
#  include <CUDAModule.hpp>
#  include <RayTracerLayer.hpp>
#endif
#include "ClassRegistry.hpp"
#include "Times.hpp"
#ifdef ECOSYSLAB_PLUGIN
#  include "ObjectRotator.hpp"
using namespace eco_sys_lab_plugin;
#endif

#include "ProjectManager.hpp"

#ifdef DIGITAL_AGRICULTURE_PLUGIN
#  include "SorghumLayer.hpp"
using namespace digital_agriculture_plugin;
#endif
#include "EditorLayer.hpp"
#include "WindowLayer.hpp"
using namespace evo_engine;
void EngineSetup();

int main() {
  std::filesystem::path resource_folder_path("../../../../../Resources");
  if (!std::filesystem::exists(resource_folder_path)) {
    resource_folder_path = "../../../../Resources";
  }
  if (!std::filesystem::exists(resource_folder_path)) {
    resource_folder_path = "../../../Resources";
  }
  if (!std::filesystem::exists(resource_folder_path)) {
    resource_folder_path = "../../Resources";
  }
  if (!std::filesystem::exists(resource_folder_path)) {
    resource_folder_path = "../Resources";
  }
  if (std::filesystem::exists(resource_folder_path)) {
    for (auto i : std::filesystem::recursive_directory_iterator(resource_folder_path)) {
      if (i.is_directory())
        continue;
      const auto& old_path = i.path();
      auto new_path = i.path();
      bool remove = false;
      if (i.path().extension().string() == ".uescene") {
        new_path.replace_extension(".evescene");
        remove = true;
      }
      if (i.path().extension().string() == ".umeta") {
        new_path.replace_extension(".evefilemeta");
        remove = true;
      }
      if (i.path().extension().string() == ".ueproj") {
        new_path.replace_extension(".eveproj");
        remove = true;
      }
      if (i.path().extension().string() == ".ufmeta") {
        new_path.replace_extension(".evefoldermeta");
        remove = true;
      }
      if (remove) {
        std::filesystem::copy(old_path, new_path);
        std::filesystem::remove(old_path);
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
#ifdef ECOSYSLAB_PLUGIN
  PrivateComponentRegistration<ObjectRotator>("ObjectRotator");
#endif
  ApplicationInfo application_configs;
  application_configs.application_name = "DigitalAgriculture";
  application_configs.project_path =
      std::filesystem::absolute(resource_folder_path / "DigitalAgricultureProject" / "test.eveproj");
  Application::Initialize(application_configs);

#ifdef OPTIX_RAY_TRACER_PLUGIN

  auto ray_tracer_layer = Application::GetLayer<RayTracerLayer>();
#endif

  // adjust default camera speed
  const auto editor_layer = Application::GetLayer<EditorLayer>();
  editor_layer->velocity = 2.f;
  editor_layer->default_scene_camera_position = glm::vec3(1.124, 0.218, 14.089);
  // override default scene camera position etc.
  editor_layer->show_camera_window = false;
  editor_layer->show_scene_window = true;
  editor_layer->show_entity_explorer_window = true;
  editor_layer->show_entity_inspector_window = true;
  const auto render_layer = Application::GetLayer<RenderLayer>();
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
    if (const auto main_camera = Application::GetActiveScene()->main_camera.Get<Camera>()) {
      scene->SetDataComponent(main_camera->GetOwner(), transform);
      main_camera->use_clear_color = true;
      main_camera->clear_color = glm::vec3(0.5f);
    }
#pragma endregion
#pragma endregion
  });
}
