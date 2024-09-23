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
  application_configs.project_path =
      std::filesystem::absolute(resource_folder_path / "LogGraderProject" / "Default.eveproj");
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

#pragma region Engine Loop
  Application::Start();
  Application::Run();
#pragma endregion
  Application::Terminate();
}
