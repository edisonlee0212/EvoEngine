// PlantFactory.cpp : This file contains the 'main' function. Program execution
// begins and ends there.
//
#include <Application.hpp>
#include "ClassRegistry.hpp"

#include "EditorLayer.hpp"
#include "RenderLayer.hpp"
#include "WindowLayer.hpp"
using namespace evo_engine;

int main() {
  Application application;
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

  ApplicationContext::Get().PushLayer<RenderLayer>("Render Layer");
  ApplicationContext::Get().PushLayer<WindowLayer>("Window Layer");
  ApplicationContext::Get().PushLayer<EditorLayer>("Editor Layer");
  ApplicationInitializationSettings application_configs;
  application_configs.application_name = "Log Grader";
  application_configs.project_path =
      std::filesystem::absolute(resource_folder_path / "LogGradingProject" / "Default.eveproj");
  application_configs.enable_runtime_packages = true;
  application_configs.startup_runtime_packages = {"LogGrading"};
  ApplicationContext::Get().Initialize(application_configs);

  // adjust default camera speed
  const auto editor_layer = ApplicationContext::Get().GetLayer<EditorLayer>();
  editor_layer->velocity = 2.f;
  editor_layer->default_scene_camera_position = glm::vec3(1.124, 0.218, 14.089);
  // override default scene camera position etc.
  editor_layer->default_scene_camera_position = glm::vec3(0, 2.5, 6);
  editor_layer->SetSceneCameraPosition(editor_layer->default_scene_camera_position);
  editor_layer->GetSceneCamera()->camera_settings.clear_color = glm::vec4(1.f);
  const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>();

#pragma region Engine Loop
  ApplicationContext::Get().Start();
  ApplicationContext::Get().Run();
#pragma endregion
  ApplicationContext::Get().Terminate();
}
