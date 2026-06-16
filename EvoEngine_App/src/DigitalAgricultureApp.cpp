// PlantFactory.cpp : This file contains the 'main' function. Program execution
// begins and ends there.
//
#include <Application.hpp>

#ifdef CUDA_MODULE_SERVICE
#  include <CUDAModule.hpp>
#  include <RayTracerLayer.hpp>
#endif
#include "ClassRegistry.hpp"
#include "PathUtils.hpp"
#include "Times.hpp"

#include "ProjectManager.hpp"

#include "EditorLayer.hpp"
#include "ImGuiLayer.hpp"
#include "RenderLayer.hpp"
#include "WindowLayer.hpp"
using namespace evo_engine;
void EngineSetup();

int main() {
  Application application;
  auto resource_folder_path = path_utils::FindAncestorChildPath("Resources", std::filesystem::current_path(), 8);
  if (resource_folder_path.empty()) {
    resource_folder_path = path_utils::NormalizeAbsolutePath("Resources");
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

  ApplicationContext::Get().PushLayer<RenderLayer>("Render Layer");
#ifdef CUDA_MODULE_SERVICE
  ApplicationContext::Get().PushLayer<RayTracerLayer>("Ray Tracer Layer");
#endif
  ApplicationContext::Get().PushLayer<WindowLayer>("Window Layer");
  ApplicationContext::Get().PushLayer<ImGuiLayer>("ImGui Layer");
  ApplicationContext::Get().PushLayer<EditorLayer>("Editor Layer");

  ApplicationInitializationSettings application_configs;
  application_configs.application_name = "DigitalAgriculture";
  application_configs.project_path =
      std::filesystem::absolute(resource_folder_path / "DigitalAgricultureProject" / "test.eveproj");
  application_configs.enable_runtime_packages = true;
  application_configs.use_custom_title_bar = true;
  application_configs.startup_runtime_packages = {"DigitalAgriculture"};
  ApplicationContext::Get().Initialize(application_configs);

#ifdef CUDA_MODULE_SERVICE

  auto ray_tracer_layer = ApplicationContext::Get().GetLayer<RayTracerLayer>();
#endif

  // adjust default camera speed
  const auto editor_layer = ApplicationContext::Get().GetLayer<EditorLayer>();
  editor_layer->velocity = 2.f;
  editor_layer->default_scene_camera_position = glm::vec3(1.124, 0.218, 14.089);
  const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>();
#pragma region Engine Loop
  ApplicationContext::Get().Start();
  ApplicationContext::Get().Run();
#pragma endregion
  ApplicationContext::Get().Terminate();
}

void EngineSetup() {
  ProjectManager::SetActionAfterNewScene([=](const std::shared_ptr<Scene>& scene) {
#pragma region Engine Setup
    Transform transform;
    transform.SetEulerRotation(glm::radians(glm::vec3(150, 30, 0)));
#pragma region Preparations
    ApplicationContext::Get().GetTimes().SetTimeStep(0.016f);
    transform = Transform();
    transform.SetPosition(glm::vec3(0, 2, 35));
    transform.SetEulerRotation(glm::radians(glm::vec3(15, 0, 0)));
    if (const auto main_camera = ApplicationContext::Get().GetActiveScene()->main_camera.Get<Camera>()) {
      scene->SetDataComponent(main_camera->GetOwner(), transform);
      main_camera->camera_settings.use_clear_color = true;
      main_camera->camera_settings.clear_color = glm::vec4(0.5f, 0.5f, 0.5f, 1.f);
    }
#pragma endregion
#pragma endregion
  });
}
