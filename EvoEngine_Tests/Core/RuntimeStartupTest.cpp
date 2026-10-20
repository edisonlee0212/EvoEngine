#include <gtest/gtest.h>
#include <chrono>
#include <cstdlib>
#include <thread>
#include "Application.hpp"
#include "AssetManager.hpp"
#include "Camera.hpp"
#include "Platform.hpp"
#include "ProjectManager.hpp"
#include "RenderLayer.hpp"
#include "Scene.hpp"
#include "WindowLayer.hpp"

using namespace evo_engine;

TEST(RuntimeStartup, PreparesAFrameWithoutShowingTheWindowOrStartingPlayback) {
  if (!std::getenv("EVOENGINE_RUNTIME_GUI_PROOF"))
    GTEST_SKIP() << "Set EVOENGINE_RUNTIME_GUI_PROOF=1 to run the graphics fixture.";
  Application app;
  ApplicationContextScope scope(app);
  app.PushLayer<RenderLayer>("Rendering");
  const auto window = app.PushLayer<WindowLayer>("Window");
  ApplicationInitializationSettings settings;
  settings.application_mode = ApplicationMode::Player;
  settings.allow_empty_project = true;
  settings.default_window_size = {320, 240};
  settings.load_project_assets = settings.load_project_start_scene = false;
  settings.redirect_standard_streams_to_console = false;
  settings.graphics_settings.use_ray_tracing = settings.graphics_settings.use_mesh_shader = false;
  settings.defer_window_show = true;
  settings.window_mode = WindowDisplayMode::Windowed;
  bool reported_before_graphics = false;
  settings.startup_status = [&](std::string_view) {
    reported_before_graphics |= !Platform::Initialized();
  };
  app.Initialize(settings);
  EXPECT_TRUE(reported_before_graphics);
  EXPECT_FALSE(glfwGetWindowAttrib(window->GetGlfwWindow(), GLFW_VISIBLE));
  window->SetDisplayMode(WindowDisplayMode::BorderlessWindowed);
  EXPECT_FALSE(glfwGetWindowAttrib(window->GetGlfwWindow(), GLFW_VISIBLE));
  {
    const auto scene = AssetManager::CreateTemporaryAsset<Scene>();
    const auto camera = scene->GetOrSetPrivateComponent<Camera>(scene->CreateEntity("Main camera")).lock();
    scene->main_camera = camera;
    ProjectManager::SetStartScene(scene);
    app.Attach(scene);
    for (int frame = 0; frame < 3; ++frame)
      EXPECT_TRUE(app.Loop());
    EXPECT_EQ(app.GetApplicationStatus(), Application::ExecutionStatus::NotPlaying);
    EXPECT_GT(camera->GetFrameCount(), 0u);
    EXPECT_FALSE(glfwGetWindowAttrib(window->GetGlfwWindow(), GLFW_VISIBLE));
    window->ShowWindow();
    EXPECT_TRUE(glfwGetWindowAttrib(window->GetGlfwWindow(), GLFW_VISIBLE));
  }
  app.Terminate();
}

TEST(RuntimeStartup, PreparesOnlyRequestedRayModesAndWaitsForModeChanges) {
  if (!std::getenv("EVOENGINE_RUNTIME_GUI_PROOF"))
    GTEST_SKIP() << "Set EVOENGINE_RUNTIME_GUI_PROOF=1 to run the graphics fixture.";
  Application app;
  ApplicationContextScope scope(app);
  auto renderer = app.PushLayer<RenderLayer>("Rendering");
  app.PushLayer<WindowLayer>("Window");
  ApplicationInitializationSettings settings;
  settings.application_mode = ApplicationMode::Player;
  settings.allow_empty_project = true;
  settings.default_window_size = {320, 240};
  settings.load_project_assets = settings.load_project_start_scene = false;
  settings.redirect_standard_streams_to_console = false;
  settings.graphics_settings.use_ray_tracing = true;
  settings.graphics_settings.use_mesh_shader = false;
  settings.defer_window_show = true;
  settings.window_mode = WindowDisplayMode::Windowed;
  app.Initialize(settings);
  if (!Platform::RayQueryEnabled()) {
    renderer.reset();
    app.Terminate();
    GTEST_SKIP() << "Ray query is not available.";
  }
  {
    const auto scene = AssetManager::CreateTemporaryAsset<Scene>();
    const auto camera = scene->main_camera.Get<Camera>();
    camera->camera_render_mode = Camera::CameraRenderMode::Rasterization;
    scene->main_camera = camera;
    ProjectManager::SetStartScene(scene);
    app.Attach(scene);
    for (int frame = 0; frame < 4; ++frame)
      EXPECT_TRUE(app.Loop());
    EXPECT_EQ(renderer->GetShaderPreparationProgress().total, 0u);
    EXPECT_TRUE(renderer->IsSceneShaderPrewarmComplete(scene));

    camera->camera_render_mode = Camera::CameraRenderMode::RayQuery;
    camera->camera_settings.ray_integrator = CameraSettings::RayIntegrator::PathTracing;
    camera->camera_settings.nrd_denoising = false;
    camera->camera_settings.pt_display_firefly_strength = 0;
    const auto prepare = [&] {
      EXPECT_TRUE(app.Loop());
      EXPECT_FALSE(renderer->IsSceneShaderPrewarmComplete(scene));
      EXPECT_FALSE(renderer->GetShaderPreparationProgress().current.empty());
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(180);
      while (!renderer->IsSceneShaderPrewarmComplete(scene) && renderer->GetSceneShaderPrewarmError(scene).empty() &&
             std::chrono::steady_clock::now() < deadline) {
        if (!app.Loop()) {
          ADD_FAILURE() << "Application stopped during shader preparation.";
          break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      EXPECT_TRUE(renderer->IsSceneShaderPrewarmComplete(scene));
      EXPECT_TRUE(renderer->GetSceneShaderPrewarmError(scene).empty());
      const auto progress = renderer->GetShaderPreparationProgress();
      EXPECT_EQ(progress.completed, progress.total);
      return progress.total;
    };
    const auto beauty_requests = prepare();
    EXPECT_GT(beauty_requests, 0u);
    camera->camera_settings.ray_debug_view = CameraSettings::RayDebugView::BaseColor;
    EXPECT_GT(prepare(), beauty_requests);
    const auto requested = renderer->GetShaderPreparationProgress().total;
    camera->camera_render_mode = Camera::CameraRenderMode::Rasterization;
    for (int frame = 0; frame < 4; ++frame)
      EXPECT_TRUE(app.Loop());
    EXPECT_EQ(renderer->GetShaderPreparationProgress().total, requested);
  }
  renderer.reset();
  app.Terminate();
}
