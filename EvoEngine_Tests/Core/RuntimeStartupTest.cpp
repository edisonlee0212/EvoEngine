#include <gtest/gtest.h>
#include <cstdlib>
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
