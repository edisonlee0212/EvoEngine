// PlantFactory.cpp : This file contains the 'main' function. Program execution
// begins and ends there.
//
#include <Application.hpp>

#ifdef CUDA_MODULE_SERVICE
#  include <CUDAModule.hpp>
#  include <RayTracerCamera.hpp>
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
  settings.panels.entity_inspector = true;
  settings.panels.console = false;
  settings.panels.project = true;
  settings.panels.resources = false;
  settings.panels.profiler = false;
  settings.panels.runtime_package_manager = false;
  settings.panels.render_layer_inspection = false;

  EditorDockLayoutSettings dock_layout;
  dock_layout.left_fraction = 0.18f;
  dock_layout.right_fraction = 0.30f;
  dock_layout.bottom_fraction = 0.25f;
  dock_layout.asset_inspector_fraction = 0.40f;
  settings.dock_layout = dock_layout;
  return settings;
}

struct DigitalAgricultureArguments {
  ApplicationMode application_mode = ApplicationMode::Editor;
  std::filesystem::path startup_scene;
};

DigitalAgricultureArguments ParseArguments(const int argc, char** argv) {
  constexpr auto default_scene = "GeneratedAssets/Scenes/Sorghum_4x10_GrowthStage03.evescene";
  DigitalAgricultureArguments arguments{ApplicationMode::Editor, default_scene};
  for (int index = 1; index < argc; ++index) {
    if (ConsumeApplicationModeArgument(argc, argv, index, arguments.application_mode)) {
      continue;
    }
    if (std::string_view(argv[index]) == "--startup-scene") {
      if (index + 1 >= argc) {
        throw std::invalid_argument("--startup-scene requires an Assets-relative .evescene path.");
      }
      arguments.startup_scene = argv[++index];
      continue;
    }
    throw std::invalid_argument("Unknown application argument: " + std::string(argv[index] ? argv[index] : ""));
  }
  return arguments;
}

void ConfigureScenePresentation(const std::shared_ptr<Scene>& scene) {
  const auto editor_layer = ApplicationContext::Get().GetLayer<EditorLayer>();
  const auto main_camera = scene ? scene->main_camera.Get<Camera>() : nullptr;
  if (!scene || !editor_layer || !main_camera || !scene->IsEntityValid(main_camera->GetOwner())) {
    return;
  }

  // The Vulkan viewport should remain the fast, color-accurate preview for field editing.
  main_camera->camera_render_mode = Camera::CameraRenderMode::Rasterization;
  main_camera->ResetFrameCount();
  const auto editor_camera = editor_layer->GetSceneCamera();
  editor_camera->camera_settings = main_camera->camera_settings;
  editor_camera->camera_render_mode = Camera::CameraRenderMode::Rasterization;
  editor_camera->ResetFrameCount();

  // Both editor viewports use the editor camera transform, so seed it from the scene's saved main camera.
  const auto main_camera_transform = scene->GetDataComponent<GlobalTransform>(main_camera->GetOwner()).value;
  const glm::vec3 camera_position = main_camera_transform[3];
  const glm::quat camera_rotation = glm::quat_cast(main_camera_transform);
  editor_layer->default_scene_camera_position = camera_position;
  editor_layer->default_scene_camera_rotation = camera_rotation;
  editor_layer->SetSceneCameraPosition(camera_position);
  editor_layer->SetSceneCameraRotation(camera_rotation);
#ifdef CUDA_MODULE_SERVICE
  if (const auto ray_tracer_layer = ApplicationContext::Get().GetLayer<RayTracerLayer>()) {
    ray_tracer_layer->scene_camera->ApplyCameraSettings(main_camera->camera_settings);
  }
#endif
}

void ConfigureSorghumEditor() {
  const auto editor_layer = ApplicationContext::Get().GetLayer<EditorLayer>();
  if (!editor_layer) {
    return;
  }
  editor_layer->ClearAssetInspectors();
  editor_layer->velocity = 2.f;
  ConfigureScenePresentation(ApplicationContext::Get().GetActiveScene());
  for (const auto& layer : ApplicationContext::Get().GetLayers()) {
    const auto& name = layer->GetLayerName();
    layer->enable_inspection = name == "Sorghum Layer" || name == "LSystem Layer" || name == "Ray Tracer Layer";
  }
#ifdef CUDA_MODULE_SERVICE
  if (const auto ray_tracer_layer = ApplicationContext::Get().GetLayer<RayTracerLayer>()) {
    ray_tracer_layer->show_scene_window = true;
    ray_tracer_layer->focus_scene_window = true;
  }
#endif
  editor_layer->RequestEditorLayout(CreateSorghumEditorLayout());
}
}  // namespace

int main(const int argc, char** argv) {
  Application application;
  const auto arguments = ParseArguments(argc, argv);
  const auto application_mode = arguments.application_mode;
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
  ApplicationContext::Get().RegisterPostAttachSceneFunction(ConfigureScenePresentation);
  // Project editor state is restored after Attach(), so repeat camera synchronization once loading is complete.
  ProjectManager::SetActionAfterSceneLoad(ConfigureScenePresentation);

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
  application_configs.startup_scene_path = arguments.startup_scene;
  application_configs.enable_runtime_packages = true;
  application_configs.use_custom_title_bar = true;
  application_configs.startup_runtime_packages = {"DigitalAgriculture", "LSystem"};
  // The experiment scene already references every asset needed by the editor.
  // Avoid decoding the rest of the research archive on startup.
  application_configs.load_project_assets = false;
  ApplyApplicationModeDefaults(application_configs);
  ApplicationContext::Get().Initialize(application_configs);
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
