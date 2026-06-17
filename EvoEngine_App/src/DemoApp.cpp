#include "AppBootstrap.hpp"
#include "Application.hpp"
#include "AssetManager.hpp"
#include "Camera.hpp"
#include "DemoScene.hpp"
#include "EditorLayer.hpp"
#include "ImGuiLayer.hpp"
#include "Material.hpp"
#include "Mesh.hpp"
#include "MeshRenderer.hpp"
#include "Platform.hpp"
#include "ProjectManager.hpp"
#include "RenderLayer.hpp"
#include "SkinnedMesh.hpp"
#include "SkinnedMeshRenderer.hpp"
#include "WindowLayer.hpp"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <fstream>

#ifdef PHYSX_PHYSICS_SERVICE
#  include "PhysicsLayer.hpp"
#endif

using namespace evo_engine;

namespace {
enum class DemoAppRunMode { Normal, SmokeTest, EditorScreenshot };

struct DemoAppRuntimeConfig {
  DemoAppRunMode mode = DemoAppRunMode::Normal;
  ApplicationMode application_mode = ApplicationMode::Editor;
  DemoSetup demo_setup = DemoSetup::Rendering;
  size_t frames_after_play = 100;
  size_t warmup_frames = 30;
  size_t max_load_frames = 30000;
  size_t max_play_frames = 1000;
  int screenshot_width = 1920;
  int screenshot_height = 1080;
  bool exit_on_complete = true;
  std::filesystem::path ready_file;
  std::filesystem::path done_file;
};

struct DemoAppCommandLine {
  std::optional<std::filesystem::path> run_config_path;
  std::optional<ApplicationMode> application_mode;
};

DemoAppCommandLine ParseCommandLine(const int argc, char** argv) {
  DemoAppCommandLine command_line;
  for (int arg_index = 1; arg_index < argc; ++arg_index) {
    const std::string argument = argv[arg_index] ? argv[arg_index] : "";
    if (argument == "--run-config") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument("--run-config requires a path.");
      }
      command_line.run_config_path = std::filesystem::absolute(argv[++arg_index]);
    } else {
      auto mode = command_line.application_mode.value_or(ApplicationMode::Editor);
      if (!ConsumeApplicationModeArgument(argc, argv, arg_index, mode)) {
        throw std::invalid_argument("Unknown DemoApp argument: " + argument);
      }
      command_line.application_mode = mode;
    }
  }
  return command_line;
}

std::optional<std::filesystem::path> FindRunConfigPath(const DemoAppCommandLine& command_line, const int argc,
                                                       char** argv) {
  if (command_line.run_config_path) {
    return command_line.run_config_path;
  }
  std::vector<std::filesystem::path> candidates;
  if (argc > 0 && argv[0]) {
    candidates.emplace_back(std::filesystem::absolute(argv[0]).parent_path() / "DemoApp.run.yaml");
  }
  candidates.emplace_back(std::filesystem::current_path() / "DemoApp.run.yaml");
  for (const auto& candidate : candidates) {
    if (std::filesystem::exists(candidate)) {
      return candidate;
    }
  }
  return {};
}

DemoSetup ParseDemoSetup(const std::string& value) {
  if (value == "Empty") {
    return DemoSetup::Empty;
  }
  if (value == "Rendering") {
    return DemoSetup::Rendering;
  }
  if (value == "Universe") {
    return DemoSetup::Universe;
  }
  throw std::invalid_argument("Unknown demo_setup value: " + value);
}

DemoAppRuntimeConfig LoadRunConfig(const std::filesystem::path& path) {
  const YAML::Node root = YAML::LoadFile(path.string());
  if (!root || !root.IsMap()) {
    throw std::runtime_error("DemoApp run config must be a YAML map.");
  }

  DemoAppRuntimeConfig config;
  if (const auto mode = root["mode"]) {
    const auto mode_name = mode.as<std::string>();
    if (mode_name == "normal") {
      config.mode = DemoAppRunMode::Normal;
    } else if (mode_name == "smoke_test") {
      config.mode = DemoAppRunMode::SmokeTest;
    } else if (mode_name == "editor_screenshot") {
      config.mode = DemoAppRunMode::EditorScreenshot;
    } else {
      throw std::invalid_argument("Unknown mode value: " + mode_name);
    }
  }
  if (const auto application_mode = root["application_mode"]) {
    config.application_mode = ParseApplicationModeName(application_mode.as<std::string>());
  }
  if (const auto demo_setup = root["demo_setup"]) {
    config.demo_setup = ParseDemoSetup(demo_setup.as<std::string>());
  }
  if (const auto frames_after_play = root["frames_after_play"]) {
    config.frames_after_play = frames_after_play.as<size_t>();
  }
  if (const auto warmup_frames = root["warmup_frames"]) {
    config.warmup_frames = warmup_frames.as<size_t>();
  }
  if (const auto max_load_frames = root["max_load_frames"]) {
    config.max_load_frames = max_load_frames.as<size_t>();
  }
  if (const auto max_play_frames = root["max_play_frames"]) {
    config.max_play_frames = max_play_frames.as<size_t>();
  }
  if (const auto screenshot_width = root["screenshot_width"]) {
    config.screenshot_width = screenshot_width.as<int>();
  }
  if (const auto screenshot_height = root["screenshot_height"]) {
    config.screenshot_height = screenshot_height.as<int>();
  }
  if (const auto exit_on_complete = root["exit_on_complete"]) {
    config.exit_on_complete = exit_on_complete.as<bool>();
  }
  if (const auto ready_file = root["ready_file"]) {
    config.ready_file = std::filesystem::absolute(ready_file.as<std::string>());
  }
  if (const auto done_file = root["done_file"]) {
    config.done_file = std::filesystem::absolute(done_file.as<std::string>());
  }
  return config;
}

int FailSmokeTest(Application& application, const std::string& reason) {
  if (application.IsPlaying()) {
    application.Stop();
  }
  application.End();
  std::cerr << "EVOENGINE_APP_TEST_RESULT failed reason=\"" << reason << "\"" << std::endl;
  return 1;
}

int RunSmokeTest(const DemoAppRuntimeConfig& config) {
  auto& application = ApplicationContext::Get();
  const bool expect_player_autoplay = config.application_mode == ApplicationMode::Player;
  application.Start(expect_player_autoplay);

  size_t load_frame_count = 0;
  while (!ProjectManager::IsProjectIdle()) {
    if (!application.Loop()) {
      return FailSmokeTest(application, "application ended before project load completed");
    }
    ++load_frame_count;
    if (load_frame_count >= config.max_load_frames) {
      return FailSmokeTest(application, "project load timed out");
    }
  }

  if (!ProjectManager::GetStartScene().lock()) {
    return FailSmokeTest(application, "start scene is missing after project load");
  }
  if (!application.GetActiveScene()) {
    return FailSmokeTest(application, "active scene is missing after project load");
  }

  const auto loaded_frame = Platform::GetFrameCount();
  if (expect_player_autoplay) {
    if (!application.IsPlaying()) {
      return FailSmokeTest(application, "player mode did not enter play mode automatically");
    }
  } else {
    application.Play();
    if (!application.IsPlaying()) {
      return FailSmokeTest(application, "application did not enter play mode");
    }
  }

  const auto play_start_frame = Platform::GetFrameCount();
  size_t play_loop_count = 0;
  while (Platform::GetFrameCount() - play_start_frame < config.frames_after_play) {
    if (!application.Loop()) {
      return FailSmokeTest(application, "application ended before requested play frames completed");
    }
    ++play_loop_count;
    if (play_loop_count >= config.max_play_frames) {
      return FailSmokeTest(application, "play frame wait timed out");
    }
  }

  std::cout << "EVOENGINE_APP_TEST_RESULT passed"
            << " loaded_frame=" << loaded_frame << " play_start_frame=" << play_start_frame
            << " final_frame=" << Platform::GetFrameCount() << std::endl;
  if (!config.exit_on_complete) {
    application.Run();
  } else {
    application.Stop();
    application.End();
  }
  return 0;
}

void WriteMarkerFile(const std::filesystem::path& path) {
  if (path.empty()) {
    throw std::invalid_argument("Marker file path is empty.");
  }
  if (const auto parent = path.parent_path(); !parent.empty()) {
    std::filesystem::create_directories(parent);
  }
  std::ofstream marker(path);
  if (!marker) {
    throw std::runtime_error("Failed to write marker file: " + path.string());
  }
  marker << "ready" << std::endl;
}

std::string Lowercase(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(), [](const unsigned char character) {
    return static_cast<char>(std::tolower(character));
  });
  return value;
}

struct ShowcaseInspectorTarget {
  Entity entity;
  std::shared_ptr<IAsset> mesh;
  std::shared_ptr<IAsset> material;
  int score = -1;
};

int ScoreAssetTitle(const Handle& asset_handle, const std::string& needle, const int weight) {
  if (asset_handle.GetValue() == 0) {
    return 0;
  }
  const auto asset = AssetManager::GetAsset(asset_handle);
  if (!asset) {
    return 0;
  }
  return Lowercase(asset->GetTitle()).find(needle) != std::string::npos ? weight : 0;
}

int ScoreMaterialTextures(const std::shared_ptr<Material>& material) {
  if (!material) {
    return 0;
  }
  int score = 0;
  score += ScoreAssetTitle(material->PeekAlbedoTextureRef().GetAssetHandle(), "curtain", 5);
  score += ScoreAssetTitle(material->PeekNormalTextureRef().GetAssetHandle(), "curtain", 3);
  score += ScoreAssetTitle(material->PeekAlbedoTextureRef().GetAssetHandle(), "blue", 1);
  score += ScoreAssetTitle(material->PeekAlbedoTextureRef().GetAssetHandle(), "green", 1);
  return score;
}

void ConsiderShowcaseTarget(const std::shared_ptr<Scene>& scene, const Entity& entity,
                            const std::shared_ptr<IAsset>& mesh, const std::shared_ptr<Material>& material,
                            ShowcaseInspectorTarget& target) {
  if (!mesh || !material) {
    return;
  }
  const auto entity_name = Lowercase(scene->GetEntityName(entity));
  const auto mesh_title = Lowercase(mesh->GetTitle());
  const auto material_title = Lowercase(material->GetTitle());
  int score = 0;
  if (entity_name.find("curtain") != std::string::npos) {
    score += 8;
  }
  if (mesh_title.find("curtain") != std::string::npos) {
    score += 4;
  }
  if (material_title.find("curtain") != std::string::npos) {
    score += 4;
  }
  score += ScoreMaterialTextures(material);
  if (entity_name.find("blue") != std::string::npos || mesh_title.find("blue") != std::string::npos ||
      material_title.find("blue") != std::string::npos) {
    score += 1;
  }
  if (score > target.score) {
    target = {entity, mesh, material, score};
  }
}

void FrameSceneCameraOnTarget(const std::shared_ptr<EditorLayer>& editor_layer, const std::shared_ptr<Scene>& scene,
                              const Entity& entity) {
  if (!scene->IsEntityValid(entity)) {
    return;
  }
  editor_layer->SetSceneCameraPosition(glm::vec3(0.0f, 0.0f, 3.0f));
  editor_layer->SetSceneCameraRotation(glm::quat(glm::vec3(0.0f)));
}

void PrepareReadmeScreenshotShowcase(const std::shared_ptr<EditorLayer>& editor_layer) {
  const auto scene = ApplicationContext::Get().GetActiveScene();
  if (!scene || !editor_layer) {
    return;
  }

  if (const auto main_camera = scene->main_camera.Get<Camera>()) {
    main_camera->camera_render_mode = Camera::CameraRenderMode::RayTracing;
    main_camera->ResetFrameCount();
  }

  ShowcaseInspectorTarget target;
  for (const auto& entity : scene->UnsafeGetAllEntities()) {
    if (!scene->IsEntityValid(entity)) {
      continue;
    }
    if (scene->HasPrivateComponent<MeshRenderer>(entity)) {
      const auto mesh_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(entity).lock();
      ConsiderShowcaseTarget(scene, entity, mesh_renderer->mesh.Get<Mesh>(), mesh_renderer->material.Get<Material>(),
                             target);
    }
    if (scene->HasPrivateComponent<SkinnedMeshRenderer>(entity)) {
      const auto skinned_mesh_renderer = scene->GetOrSetPrivateComponent<SkinnedMeshRenderer>(entity).lock();
      ConsiderShowcaseTarget(scene, entity, skinned_mesh_renderer->skinned_mesh.Get<SkinnedMesh>(),
                             skinned_mesh_renderer->material.Get<Material>(), target);
    }
  }

  if (target.score >= 0) {
    if (target.score >= 4) {
      scene->SetEntityName(target.entity, "Blur Curtain");
    }
    editor_layer->SetSelectedEntity(target.entity, false);
    FrameSceneCameraOnTarget(editor_layer, scene, target.entity);
    editor_layer->ClearAssetInspectors();
    editor_layer->OpenAssetInspector(target.material);
  }
}

EditorLayoutSettings CreateReadmeScreenshotEditorLayout() {
  EditorLayoutSettings settings;
  settings.panels.scene = true;
  settings.panels.camera = true;
  settings.panels.scene_camera_debug = false;
  settings.panels.scene_info = false;
  settings.panels.camera_info = true;
  settings.panels.entity_explorer = true;
  settings.panels.entity_inspector = true;
  settings.panels.console = true;
  settings.panels.project = true;
  settings.panels.resources = false;
  settings.panels.runtime_package_manager = true;

  EditorDockLayoutSettings dock_layout;
  dock_layout.left_fraction = 0.15f;
  dock_layout.right_fraction = 0.19f;
  dock_layout.bottom_fraction = 0.28f;
  dock_layout.camera_fraction = 0.50f;
  settings.dock_layout = dock_layout;

  EditorFloatingWindowLayout asset_inspector_window;
  asset_inspector_window.anchor = EditorFloatingWindowLayout::Anchor::LowerLeft;
  asset_inspector_window.size = {292.0f, 530.0f};
  asset_inspector_window.margin = {24.0f, 24.0f};
  settings.asset_inspector_window = asset_inspector_window;

  EditorRuntimePackageManagerLayoutSettings package_manager;
  package_manager.floating_window.anchor = EditorFloatingWindowLayout::Anchor::LowerRight;
  package_manager.floating_window.size = {600.0f, 530.0f};
  package_manager.floating_window.margin = {24.0f, 24.0f};
  package_manager.list_width_fraction = 0.30f;
  package_manager.list_width_min = 220.0f;
  package_manager.list_width_max = 320.0f;
  settings.runtime_package_manager = package_manager;

  EditorProjectBrowserLayoutSettings project_browser;
  project_browser.hierarchy_width = 320.0f;
  project_browser.reveal_folder = std::filesystem::path("Models") / "Sponza" / "textures";
  settings.project_browser = project_browser;

  return settings;
}

int FailEditorScreenshot(Application& application, const std::string& reason) {
  application.End();
  std::cerr << "EVOENGINE_EDITOR_SCREENSHOT_RESULT failed reason=\"" << reason << "\"" << std::endl;
  return 1;
}

int RunEditorScreenshot(const DemoAppRuntimeConfig& config) {
  if (config.ready_file.empty()) {
    throw std::invalid_argument("editor_screenshot mode requires ready_file.");
  }
  if (config.done_file.empty()) {
    throw std::invalid_argument("editor_screenshot mode requires done_file.");
  }

  auto& application = ApplicationContext::Get();
  application.Start(false);

  size_t load_frame_count = 0;
  while (!ProjectManager::IsProjectIdle()) {
    if (!application.Loop()) {
      return FailEditorScreenshot(application, "application ended before project load completed");
    }
    ++load_frame_count;
    if (load_frame_count >= config.max_load_frames) {
      return FailEditorScreenshot(application, "project load timed out");
    }
  }

  const auto editor_layer = application.GetLayer<EditorLayer>();
  if (!editor_layer) {
    return FailEditorScreenshot(application, "editor layer is missing");
  }
  editor_layer->RequestEditorLayout(CreateReadmeScreenshotEditorLayout());
  PrepareReadmeScreenshotShowcase(editor_layer);

  for (size_t frame_index = 0; frame_index < config.warmup_frames; ++frame_index) {
    if (!application.Loop()) {
      return FailEditorScreenshot(application, "application ended before screenshot warmup completed");
    }
  }

  WriteMarkerFile(config.ready_file);
  std::cout << "EVOENGINE_EDITOR_SCREENSHOT_READY ready_file=\"" << config.ready_file.string() << "\"" << std::endl;

  while (!std::filesystem::exists(config.done_file)) {
    if (!application.Loop()) {
      return FailEditorScreenshot(application, "application ended before screenshot capture completed");
    }
  }

  application.End();
  std::cout << "EVOENGINE_EDITOR_SCREENSHOT_RESULT passed" << std::endl;
  return 0;
}
}  // namespace

int main(const int argc, char** argv) {
  Application application;
  bool initialized = false;
  bool automated_run = false;
  try {
    const auto command_line = ParseCommandLine(argc, argv);
    const auto run_config_path = FindRunConfigPath(command_line, argc, argv);
    std::optional<DemoAppRuntimeConfig> runtime_config;
    if (run_config_path) {
      runtime_config = LoadRunConfig(*run_config_path);
    }
    if (runtime_config && command_line.application_mode) {
      runtime_config->application_mode = *command_line.application_mode;
    }
    const auto demo_setup = runtime_config ? runtime_config->demo_setup : DemoSetup::Rendering;
    auto application_mode = command_line.application_mode.value_or(ApplicationMode::Editor);
    if (runtime_config) {
      application_mode = runtime_config->mode == DemoAppRunMode::EditorScreenshot ? ApplicationMode::Editor
                                                                                  : runtime_config->application_mode;
    }

    PushStandardApplicationLayers(application_mode);
#ifdef PHYSX_PHYSICS_SERVICE
    ApplicationContext::Get().PushLayer<PhysicsLayer>();
#endif

    ApplicationInitializationSettings application_info;
    SetupDemoScene(demo_setup, application_info);
    application_info.application_mode = application_mode;
    if (runtime_config && runtime_config->mode == DemoAppRunMode::EditorScreenshot) {
      application_info.default_window_size = {runtime_config->screenshot_width, runtime_config->screenshot_height};
    }
    application_info.use_custom_title_bar = true;
    ApplyApplicationModeDefaults(application_info);

    ApplicationContext::Get().Initialize(application_info);
    initialized = true;

    int exit_code = 0;
    automated_run = runtime_config && (runtime_config->mode == DemoAppRunMode::SmokeTest ||
                                       runtime_config->mode == DemoAppRunMode::EditorScreenshot);
    if (runtime_config && runtime_config->mode == DemoAppRunMode::SmokeTest) {
      exit_code = RunSmokeTest(*runtime_config);
    } else if (runtime_config && runtime_config->mode == DemoAppRunMode::EditorScreenshot) {
      exit_code = RunEditorScreenshot(*runtime_config);
    } else {
      ApplicationContext::Get().Start();
      ApplicationContext::Get().Run();
    }
    ApplicationContext::Get().Terminate();
    if (automated_run) {
      std::cout.flush();
      std::cerr.flush();
      std::_Exit(exit_code);
    }
    return exit_code;
  } catch (const std::exception& e) {
    std::cerr << "EVOENGINE_APP_TEST_RESULT failed reason=\"" << e.what() << "\"" << std::endl;
    if (initialized) {
      ApplicationContext::Get().Terminate();
    }
    if (automated_run) {
      std::cout.flush();
      std::cerr.flush();
      std::_Exit(1);
    }
    return 1;
  }
}
