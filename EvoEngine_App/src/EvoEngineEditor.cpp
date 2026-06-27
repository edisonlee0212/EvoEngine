#include "AppBootstrap.hpp"
#include "Application.hpp"
#include "Camera.hpp"
#include "DemoProfiles.hpp"
#include "DemoScene.hpp"
#include "EditorLayer.hpp"
#include "PathUtils.hpp"
#include "Platform.hpp"
#include "PostProcessingStack.hpp"
#include "ProjectManager.hpp"
#include "RenderLayer.hpp"
#include "Times.hpp"
#include "WindowLayer.hpp"

#include <algorithm>
#include <optional>
#include <stdexcept>

#ifdef EVOENGINE_WINDOWS
#  ifndef NOMINMAX
#    define NOMINMAX
#  endif
#  include <Windows.h>
#endif

using namespace evo_engine;

namespace {
struct EditorCommandLine {
  std::optional<std::filesystem::path> project_path;
  std::optional<DemoProfileId> demo_profile_id;
  std::optional<std::filesystem::path> demo_preview_capture_path;
  ApplicationMode application_mode = ApplicationMode::Editor;
  bool application_mode_explicit = false;
  int preview_capture_width = 1280;
  int preview_capture_height = 720;
  size_t preview_capture_warmup_frames = 8;
  std::optional<Camera::CameraRenderMode> preview_capture_render_mode;
};

Camera::CameraRenderMode ParsePreviewRenderMode(const std::string& value) {
  if (value == "rasterization" || value == "raster" || value == "Rasterization") {
    return Camera::CameraRenderMode::Rasterization;
  }
  if (value == "raytracing" || value == "ray-tracing" || value == "RayTracing") {
    return Camera::CameraRenderMode::RayTracing;
  }
  throw std::invalid_argument("Unknown preview render mode: " + value);
}

EditorCommandLine ParseCommandLine(const int argc, char** argv) {
  EditorCommandLine command_line;
  for (int arg_index = 1; arg_index < argc; ++arg_index) {
    const std::string argument = argv[arg_index] ? argv[arg_index] : "";
    if (argument == "--project" || argument == "-p") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument(argument + " requires a project path.");
      }
      command_line.project_path = std::filesystem::absolute(argv[++arg_index]);
    } else if (argument == "--demo") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument("--demo requires a profile id.");
      }
      const std::string profile_id = argv[++arg_index] ? argv[arg_index] : "";
      const auto* profile = FindDemoProfile(profile_id);
      if (!profile) {
        throw std::invalid_argument("Unknown EvoEngineEditor demo profile: " + profile_id);
      }
      command_line.demo_profile_id = profile->id;
    } else if (argument == "--capture-demo-preview") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument("--capture-demo-preview requires an output PNG path.");
      }
      command_line.demo_preview_capture_path = std::filesystem::absolute(argv[++arg_index]);
    } else if (argument == "--preview-width") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument("--preview-width requires a positive integer.");
      }
      command_line.preview_capture_width = std::max(1, std::stoi(argv[++arg_index]));
    } else if (argument == "--preview-height") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument("--preview-height requires a positive integer.");
      }
      command_line.preview_capture_height = std::max(1, std::stoi(argv[++arg_index]));
    } else if (argument == "--preview-warmup-frames") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument("--preview-warmup-frames requires a non-negative integer.");
      }
      command_line.preview_capture_warmup_frames = static_cast<size_t>(std::max(0, std::stoi(argv[++arg_index])));
    } else if (argument == "--preview-render-mode") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument("--preview-render-mode requires rasterization or raytracing.");
      }
      command_line.preview_capture_render_mode = ParsePreviewRenderMode(argv[++arg_index] ? argv[arg_index] : "");
    } else {
      auto application_mode = command_line.application_mode;
      if (!ConsumeApplicationModeArgument(argc, argv, arg_index, application_mode)) {
        if (!command_line.project_path) {
          command_line.project_path = std::filesystem::absolute(argument);
        } else {
          throw std::invalid_argument("Unknown EvoEngineEditor argument: " + argument);
        }
      } else {
        command_line.application_mode = application_mode;
        command_line.application_mode_explicit = true;
      }
      continue;
    }
  }
  if (command_line.demo_profile_id && command_line.project_path) {
    throw std::invalid_argument("EvoEngineEditor --demo cannot be combined with --project.");
  }
  if (command_line.demo_preview_capture_path && !command_line.demo_profile_id) {
    throw std::invalid_argument("--capture-demo-preview requires --demo <profile-id>.");
  }
  if (command_line.demo_profile_id) {
    const auto& profile = GetDemoProfile(*command_line.demo_profile_id);
    if (!command_line.application_mode_explicit) {
      command_line.application_mode = profile.default_application_mode;
    }
    if (command_line.demo_preview_capture_path) {
      if (command_line.application_mode_explicit && command_line.application_mode != ApplicationMode::Editor) {
        throw std::invalid_argument("--capture-demo-preview requires editor mode.");
      }
      command_line.application_mode = ApplicationMode::Editor;
    } else if (!IsDemoProfileApplicationModeSupported(profile.id, command_line.application_mode)) {
      throw std::invalid_argument("EvoEngineEditor --demo " + std::string(profile.id_name) + " does not support " +
                                  GetApplicationModeName(command_line.application_mode) + " mode.");
    }
  }
  return command_line;
}

std::filesystem::path CurrentExecutablePath() {
#ifdef EVOENGINE_WINDOWS
  return path_utils::CurrentExecutablePath("EvoEngineEditor.exe");
#else
  return path_utils::CurrentExecutablePath("EvoEngineEditor");
#endif
}

std::filesystem::path LauncherExecutablePath() {
#ifdef EVOENGINE_WINDOWS
  return CurrentExecutablePath().parent_path() / "EvoEngineLauncher.exe";
#else
  return CurrentExecutablePath().parent_path() / "EvoEngineLauncher";
#endif
}

bool LaunchLauncherProcess(std::string& error) {
  const auto launcher_path = LauncherExecutablePath();
  if (!std::filesystem::exists(launcher_path)) {
    error = "Could not find EvoEngineLauncher next to EvoEngineEditor.";
    return false;
  }

#ifdef EVOENGINE_WINDOWS
  std::wstring command_line = L"\"" + launcher_path.wstring() + L"\"";
  STARTUPINFOW startup_info{};
  startup_info.cb = sizeof(startup_info);
  PROCESS_INFORMATION process_info{};
  const auto working_directory = launcher_path.parent_path().wstring();
  if (!CreateProcessW(nullptr, command_line.data(), nullptr, nullptr, FALSE, 0, nullptr, working_directory.c_str(),
                      &startup_info, &process_info)) {
    error = "Failed to launch EvoEngineLauncher.";
    return false;
  }
  CloseHandle(process_info.hProcess);
  CloseHandle(process_info.hThread);
  return true;
#else
  const auto command = "\"" + launcher_path.string() + "\" &";
  if (std::system(command.c_str()) != 0) {
    error = "Failed to launch EvoEngineLauncher.";
    return false;
  }
  return true;
#endif
}

void ConfigurePackageDemoNewSceneDefaults() {
  ProjectManager::SetActionAfterNewScene([](const std::shared_ptr<Scene>& scene) {
    ApplicationContext::Get().GetTimes().SetTimeStep(0.016f);
    Transform transform;
    transform.SetPosition(glm::vec3(0, 2, 35));
    transform.SetEulerRotation(glm::radians(glm::vec3(15, 0, 0)));
    if (const auto main_camera = scene->main_camera.Get<Camera>()) {
      scene->SetDataComponent(main_camera->GetOwner(), transform);
      main_camera->camera_settings.use_clear_color = true;
      main_camera->camera_settings.clear_color = glm::vec4(0.5f, 0.5f, 0.5f, 1.f);
    }
  });
}

void ConfigureDemoProfile(const DemoProfileId profile_id, const ApplicationMode application_mode,
                          ApplicationInitializationSettings& application_info) {
  const auto missing_resources = MissingDemoProfileResourceRequirements(profile_id);
  if (!missing_resources.empty()) {
    std::string message = "Demo profile '" + std::string(GetDemoProfileIdName(profile_id)) + "' is missing ";
    for (size_t i = 0; i < missing_resources.size(); ++i) {
      if (i > 0) {
        message += ", ";
      }
      message += missing_resources[i];
    }
    throw std::runtime_error(message + ".");
  }

  application_info.application_mode = application_mode;
  application_info.use_custom_title_bar = true;
  const auto resource_root = FindDemoProfileResourcesRoot();
  switch (profile_id) {
    case DemoProfileId::Rendering:
      SetupDemoScene(DemoSetup::Rendering, application_info, resource_root);
      break;
    case DemoProfileId::Ddgi:
      SetupDemoScene(DemoSetup::CornellBox, application_info, resource_root);
      ConfigureDdgiCornellBoxApplication(application_info, application_mode);
      break;
    case DemoProfileId::ProceduralGalaxy:
      SetupDemoScene(DemoSetup::ProceduralGalaxy, application_info, resource_root);
      break;
    case DemoProfileId::GaussianSplat:
      SetupDemoScene(DemoSetup::GaussianSplat, application_info, resource_root, false);
      break;
    case DemoProfileId::LSystem: {
      const auto& profile = GetDemoProfile(profile_id);
      application_info.application_name = profile.title;
      application_info.project_path = ResolveDemoProfileProjectPath(profile_id, resource_root);
      application_info.enable_runtime_packages = true;
      application_info.startup_runtime_packages = profile.startup_runtime_packages;
      break;
    }
    case DemoProfileId::EcoSysLab:
    case DemoProfileId::DigitalAgriculture: {
      NormalizeLegacyResourceExtensions(resource_root);
      ConfigurePackageDemoNewSceneDefaults();
      const auto& profile = GetDemoProfile(profile_id);
      application_info.application_name = profile.title;
      application_info.project_path = ResolveDemoProfileProjectPath(profile_id, resource_root);
      application_info.enable_runtime_packages = true;
      application_info.startup_runtime_packages = profile.startup_runtime_packages;
      break;
    }
  }
}

void ApplyDemoEditorDefaults(const DemoProfileId profile_id) {
  const auto editor_layer = ApplicationContext::Get().GetLayer<EditorLayer>();
  if (!editor_layer) {
    return;
  }
  switch (profile_id) {
    case DemoProfileId::EcoSysLab: {
      editor_layer->velocity = 2.f;
      const auto scene_camera = editor_layer->GetSceneCamera();
      if (!scene_camera) {
        return;
      }
      auto& camera_settings = scene_camera->camera_settings;
      camera_settings.use_clear_color = true;
      camera_settings.clear_color = glm::vec4(1.f);
      camera_settings.background_intensity = 3.f;
      const auto post_processing_stack = scene_camera->post_processing_stack_ref.Get<PostProcessingStack>();
      if (post_processing_stack) {
        post_processing_stack->enable_bloom = false;
      }
      break;
    }
    case DemoProfileId::DigitalAgriculture:
      editor_layer->velocity = 2.f;
      editor_layer->default_scene_camera_position = glm::vec3(1.124f, 0.218f, 14.089f);
      editor_layer->SetSceneCameraPosition(editor_layer->default_scene_camera_position);
      break;
    case DemoProfileId::LSystem:
      editor_layer->velocity = 2.f;
      editor_layer->default_scene_camera_position = glm::vec3(0.0f, 1.0f, 5.0f);
      editor_layer->SetSceneCameraPosition(editor_layer->default_scene_camera_position);
      break;
    case DemoProfileId::ProceduralGalaxy:
      editor_layer->velocity = 50.f;
      editor_layer->default_scene_camera_position = glm::vec3(0.0f, 100.0f, 100.0f);
      editor_layer->SetSceneCameraPosition(editor_layer->default_scene_camera_position);
      editor_layer->SetSceneCameraRotation(glm::quat(glm::radians(glm::vec3(-50.0f, 0.0f, 0.0f))));
      break;
    case DemoProfileId::GaussianSplat:
      editor_layer->velocity = 1.0f;
      editor_layer->default_scene_camera_position = glm::vec3(0.0f, 0.0f, 3.0f);
      editor_layer->SetSceneCameraPosition(editor_layer->default_scene_camera_position);
      break;
    case DemoProfileId::Rendering:
    case DemoProfileId::Ddgi:
      break;
  }
}

void WaitForDemoProfileProjectIdle() {
  constexpr size_t max_load_frames = 30000;
  size_t load_frame_count = 0;
  while (!ProjectManager::IsProjectIdle()) {
    if (!ApplicationContext::Get().Loop()) {
      throw std::runtime_error("Application ended before demo profile project load completed.");
    }
    ++load_frame_count;
    if (load_frame_count >= max_load_frames) {
      throw std::runtime_error("Demo profile project load timed out.");
    }
  }
}

void ApplyDemoProfilePostLoadSetup(const DemoProfileId profile_id, const ApplicationMode application_mode) {
  switch (profile_id) {
    case DemoProfileId::Rendering:
      if (application_mode == ApplicationMode::Editor) {
        ApplyRenderingDemoEditorSetup();
      }
      break;
    case DemoProfileId::Ddgi:
      ConfigureDdgiCornellBoxScene(ApplicationContext::Get().GetActiveScene());
      if (const auto editor_layer = ApplicationContext::Get().GetLayer<EditorLayer>()) {
        if (const auto scene_camera = editor_layer->GetSceneCamera()) {
          scene_camera->skybox.Clear();
          scene_camera->camera_settings.use_clear_color = true;
          scene_camera->camera_settings.clear_color = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
          scene_camera->camera_settings.background_intensity = 0.0f;
          scene_camera->ResetFrameCount();
        }
      }
      if (application_mode == ApplicationMode::Player) {
        ApplicationContext::Get().Play();
      }
      break;
    case DemoProfileId::EcoSysLab:
    case DemoProfileId::DigitalAgriculture:
    case DemoProfileId::LSystem:
    case DemoProfileId::ProceduralGalaxy:
      break;
    case DemoProfileId::GaussianSplat:
      ConfigureGaussianSplatDemoScene(ApplicationContext::Get().GetActiveScene());
      break;
  }
}

void CaptureDemoPreview(const std::filesystem::path& output_path, const int width, const int height,
                        const size_t warmup_frames,
                        const std::optional<Camera::CameraRenderMode>& preview_render_mode) {
  const glm::uvec2 preview_resolution(static_cast<uint32_t>(width), static_cast<uint32_t>(height));
  if (const auto window_layer = ApplicationContext::Get().GetLayer<WindowLayer>()) {
    window_layer->ResizeWindow(width, height);
    window_layer->CenterWindow();
  }
  const auto editor_layer = ApplicationContext::Get().GetLayer<EditorLayer>();
  if (!editor_layer) {
    throw std::runtime_error("Demo preview capture requires EditorLayer.");
  }
  editor_layer->RequestSceneCameraPreviewWindow(preview_resolution);
  const auto scene_camera = editor_layer->GetSceneCamera();
  if (!scene_camera) {
    throw std::runtime_error("Demo preview capture requires a scene camera.");
  }
  if (preview_render_mode) {
    scene_camera->camera_render_mode = *preview_render_mode;
    scene_camera->ResetFrameCount();
  }
  scene_camera->Resize(preview_resolution);
  for (size_t frame_index = 0; frame_index < warmup_frames; ++frame_index) {
    if (!ApplicationContext::Get().Loop()) {
      throw std::runtime_error("Application ended before demo preview capture completed.");
    }
  }
  const auto render_texture = scene_camera->GetRenderTexture();
  if (!render_texture) {
    throw std::runtime_error("Demo preview capture scene camera has no render texture.");
  }
  if (const auto parent_path = output_path.parent_path(); !parent_path.empty()) {
    std::filesystem::create_directories(parent_path);
  }
  render_texture->StoreToPng(output_path, width, height);
}
}  // namespace

int main(const int argc, char** argv) {
  Application application;
  bool initialized = false;
  try {
    const auto command_line = ParseCommandLine(argc, argv);
    const auto& project_path = command_line.project_path;
    if (!project_path) {
      if (command_line.demo_profile_id) {
        PushStandardApplicationLayers(command_line.application_mode);

        ApplicationInitializationSettings application_info{};
        ConfigureDemoProfile(*command_line.demo_profile_id, command_line.application_mode, application_info);
        ApplyApplicationModeDefaults(application_info);
        ApplicationContext::Get().Initialize(application_info);
        initialized = true;
        if (command_line.application_mode == ApplicationMode::Editor) {
          ApplyDemoEditorDefaults(*command_line.demo_profile_id);
        }

        ApplicationContext::Get().Start(false);
        WaitForDemoProfileProjectIdle();
        ApplyDemoProfilePostLoadSetup(*command_line.demo_profile_id, command_line.application_mode);
        if (command_line.demo_preview_capture_path) {
          CaptureDemoPreview(*command_line.demo_preview_capture_path, command_line.preview_capture_width,
                             command_line.preview_capture_height, command_line.preview_capture_warmup_frames,
                             command_line.preview_capture_render_mode);
          ApplicationContext::Get().Terminate();
          return 0;
        }
        ApplicationContext::Get().Run();
        ApplicationContext::Get().Terminate();
        return 0;
      }
      std::string error;
      if (!LaunchLauncherProcess(error)) {
        EVOENGINE_ERROR(error)
        return 1;
      }
      return 0;
    }
    if (project_path->extension() != ".eveproj") {
      EVOENGINE_ERROR("EvoEngineEditor requires --project <path-to-.eveproj>.")
      return 1;
    }

    PushStandardApplicationLayers(command_line.application_mode);

    ApplicationInitializationSettings application_info{};
    application_info.application_mode = command_line.application_mode;
    const auto launch_metadata = ProjectManager::LoadProjectLaunchMetadata(*project_path);
    application_info.application_name = launch_metadata.application_name;
    application_info.project_path = *project_path;
    application_info.use_custom_title_bar = true;
    application_info.startup_runtime_packages = launch_metadata.startup_runtime_packages;
    application_info.enable_runtime_packages = !application_info.startup_runtime_packages.empty();
    ApplyApplicationModeDefaults(application_info);
    ApplicationContext::Get().Initialize(application_info);
    initialized = true;

    ApplicationContext::Get().Start();
    ApplicationContext::Get().Run();
    ApplicationContext::Get().Terminate();
    return 0;
  } catch (const std::exception& error) {
    EVOENGINE_ERROR(error.what())
    if (initialized) {
      ApplicationContext::Get().Terminate();
    }
    return 1;
  }
}
