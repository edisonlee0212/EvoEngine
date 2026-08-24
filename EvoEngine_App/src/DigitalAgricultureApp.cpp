// PlantFactory.cpp : This file contains the 'main' function. Program execution
// begins and ends there.
//
#include <Application.hpp>

#ifdef CUDA_MODULE_SERVICE
#  include <CUDAModule.hpp>
#  include <RayTracerLayer.hpp>
#endif
#include "AppBootstrap.hpp"
#include "ClassRegistry.hpp"
#include "PathUtils.hpp"
#include "Times.hpp"

#include "ProjectManager.hpp"

using namespace evo_engine;
void EngineSetup();

namespace {
EditorLayoutSettings CreateSorghumEditorLayout() {
  EditorLayoutSettings settings;
  settings.panels.scene = true;
  settings.panels.camera = false;
  settings.panels.scene_info = false;
  settings.panels.camera_info = false;
  settings.panels.entity_explorer = true;
  settings.panels.entity_inspector = false;
  settings.panels.console = false;
  settings.panels.project = false;
  settings.panels.resources = false;
  settings.panels.profiler = false;
  settings.panels.runtime_package_manager = false;
  settings.panels.render_layer_inspection = false;

  EditorDockLayoutSettings dock_layout;
  dock_layout.left_fraction = 0.18f;
  dock_layout.right_fraction = 0.30f;
  dock_layout.bottom_fraction = 0.05f;
  settings.dock_layout = dock_layout;
  return settings;
}

void UseRayTracing(const std::shared_ptr<Scene>& scene) {
  if (scene) {
    if (const auto camera = scene->main_camera.Get<Camera>()) {
      camera->camera_render_mode = Camera::CameraRenderMode::RayTracing;
    }
  }
}

void ConfigureSorghumEditor() {
  const auto editor_layer = ApplicationContext::Get().GetLayer<EditorLayer>();
  if (!editor_layer) {
    return;
  }
  editor_layer->velocity = 2.f;
  editor_layer->default_scene_camera_position = glm::vec3(1.124, 0.218, 14.089);
  editor_layer->GetSceneCamera()->camera_render_mode = Camera::CameraRenderMode::RayTracing;
  UseRayTracing(ApplicationContext::Get().GetActiveScene());
  for (const auto& layer : ApplicationContext::Get().GetLayers()) {
    layer->enable_inspection = layer->GetLayerName() == "LSystem Layer";
  }
  editor_layer->RequestEditorLayout(CreateSorghumEditorLayout());
}
}  // namespace

int main(const int argc, char** argv) {
  Application application;
  const auto application_mode = ParseApplicationModeArguments(argc, argv);
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
  ApplicationContext::Get().RegisterPostAttachSceneFunction(UseRayTracing);

  if (application_mode != ApplicationMode::Headless) {
    ApplicationContext::Get().PushLayer<RenderLayer>("Render Layer");
#ifdef CUDA_MODULE_SERVICE
    ApplicationContext::Get().PushLayer<RayTracerLayer>("Ray Tracer Layer");
#endif
    PushWindowAndUiLayers(application_mode);
  }

  ApplicationInitializationSettings application_configs;
  application_configs.application_mode = application_mode;
  application_configs.application_name = "DigitalAgriculture";
  application_configs.project_path = std::filesystem::absolute(resource_folder_path / "DigitalAgricultureProject" /
                                                               "test_lsystem_sorghum_genotype_c_aug11_2x10.eveproj");
  application_configs.enable_runtime_packages = true;
  application_configs.use_custom_title_bar = true;
  application_configs.startup_runtime_packages = {"DigitalAgriculture", "LSystem"};
  // The experiment scene already references every asset needed by the editor.
  // Avoid decoding the rest of the research archive on startup.
  application_configs.load_project_assets = false;
  ApplyApplicationModeDefaults(application_configs);
  ApplicationContext::Get().Initialize(application_configs);
  ApplicationContext::Get().RemoveLayersOwnedByPackage("DigitalAgriculture");
  ConfigureSorghumEditor();

#pragma region Engine Loop
  ApplicationContext::Get().Start();
  while (!ProjectManager::IsProjectIdle()) {
    if (!ApplicationContext::Get().Loop()) {
      ApplicationContext::Get().Terminate();
      return 0;
    }
  }
  ConfigureSorghumEditor();
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
      main_camera->camera_settings.background_source = Camera::BackgroundSource::ClearColor;
      main_camera->camera_settings.clear_color = glm::vec4(0.5f, 0.5f, 0.5f, 1.f);
    }
#pragma endregion
#pragma endregion
  });
}
