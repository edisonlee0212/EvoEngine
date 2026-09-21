#include <gtest/gtest.h>
#include <imgui_internal.h>
#include "Application.hpp"
#include "AssetManager.hpp"
#include "Camera.hpp"
#include "EditorLayer.hpp"
#include "ImGuiLayer.hpp"
#include "Platform.hpp"
#include "ProjectManager.hpp"
#include "RenderLayer.hpp"
#include "RenderTexture.hpp"
#include "RuntimeDebugGui.hpp"
#include "RuntimeGui.hpp"
#include "RuntimeGuiLayer.hpp"
#include "RuntimeGuiProof.hpp"
#include "Scene.hpp"
#include "Texture2D.hpp"
#include "WindowLayer.hpp"

using namespace evo_engine;

namespace evo_engine {
struct RuntimeInputTestAccess {
  static void Key(GLFWwindow* window, int action) {
    Input::KeyCallBack(window, GLFW_KEY_W, 0, action, 0);
  }
  static Input::KeyActionType RawKey() {
    return Input::GetKey(GLFW_KEY_W);
  }
};
}  // namespace evo_engine

TEST(RuntimeGuiProof, CameraOverlaySurvivesFramesResizeAndPause) {
  if (!std::getenv("EVOENGINE_RUNTIME_GUI_PROOF"))
    GTEST_SKIP() << "Set EVOENGINE_RUNTIME_GUI_PROOF=1 to run the graphics fixture.";
  Application app;
  ApplicationContextScope scope(app);
  auto window = app.PushLayer<WindowLayer>("Window");
  app.PushLayer<RenderLayer>("Rendering");
  auto gui = app.PushLayer<ImGuiLayer>("GUI");
  auto editor = app.PushLayer<EditorLayer>("Editor");
  ApplicationInitializationSettings settings;
  settings.allow_empty_project = true;
  settings.default_window_size = {800, 600};
  settings.load_project_assets = false;
  settings.load_project_start_scene = false;
  settings.redirect_standard_streams_to_console = false;
  settings.graphics_settings.use_ray_tracing = false;
  settings.graphics_settings.use_mesh_shader = false;
  app.Initialize(settings);
  auto scene = AssetManager::CreateTemporaryAsset<Scene>();
  auto camera = scene->GetOrSetPrivateComponent<Camera>(scene->CreateEntity("Main camera")).lock();
  scene->main_camera = camera;
  ProjectManager::SetStartScene(scene);
  app.Attach(scene);
  editor->show_scene_window = false;
  editor->show_camera_info = false;
  editor->main_camera_allow_auto_resize = false;
  editor->main_camera_resolution_x = 320;
  editor->main_camera_resolution_y = 240;
  ImGui::LoadIniSettingsFromMemory("[Window][Camera]\nPos=10,30\nSize=700,500\nCollapsed=0\n");
  app.Start();
  app.Play();
  auto playing_camera = app.GetActiveScene()->main_camera.Get<Camera>();
  playing_camera->Resize({320, 240});
  for (int frame = 0; frame < 12; ++frame)
    ASSERT_TRUE(app.Loop());
  ASSERT_NE(gui->GetRuntimeGuiProof(), nullptr);
  auto overlay = gui->GetRuntimeGuiProof()->GetOverlay();
  ASSERT_NE(overlay, nullptr);
  const auto extent = overlay->GetExtent();
  std::vector<uint32_t> overlay_pixels;
  {
    Platform::WaitForFrameSubmissions("Runtime GUI proof readback");
    Buffer readback(extent.width * extent.height * sizeof(uint32_t));
    readback.CopyFromImage(*overlay->GetColorImage());
    readback.DownloadVector(overlay_pixels, extent.width * extent.height);
  }
  EXPECT_TRUE(std::any_of(overlay_pixels.begin(), overlay_pixels.end(), [](const auto& p) {
    return (p >> 24) > 25;
  }));
  EXPECT_TRUE(std::any_of(overlay_pixels.begin(), overlay_pixels.end(), [](const auto& p) {
    return (p >> 24) == 0;
  }));
  std::vector<glm::vec4> camera_before;
  playing_camera->GetRenderTexture()->GetRgbaChannelData(camera_before);
  playing_camera->SetEnabled(false);
  app.Pause();
  for (int frame = 0; frame < 4; ++frame)
    ASSERT_TRUE(app.Loop());
  if (const char* output = std::getenv("EVOENGINE_RUNTIME_GUI_PROOF_OUTPUT")) {
    std::filesystem::create_directories(output);
    std::vector<float> rgba(overlay_pixels.size() * 4);
    for (uint32_t y = 0; y < extent.height; ++y)
      for (uint32_t x = 0; x < extent.width; ++x) {
        const auto pixel = overlay_pixels[y * extent.width + x];
        const auto index = ((extent.height - 1 - y) * extent.width + x) * 4;
        rgba[index] = ((pixel >> 16) & 255) / 255.0f;
        rgba[index + 1] = ((pixel >> 8) & 255) / 255.0f;
        rgba[index + 2] = (pixel & 255) / 255.0f;
        rgba[index + 3] = (pixel >> 24) / 255.0f;
      }
    Texture2D::StoreToPng(std::filesystem::path(output) / "overlay.png", rgba, extent.width, extent.height, 4, 4);
    playing_camera->GetRenderTexture()->StoreToPng(std::filesystem::path(output) / "camera.png");
    window->RequestScreenshot(std::filesystem::path(output) / "presentation.png");
    for (int frame = 0; frame < 4; ++frame)
      ASSERT_TRUE(app.Loop());
    std::string error;
    EXPECT_TRUE(window->StoreCompletedScreenshot(error)) << error;
  }
  ImGui::SetWindowSize("Camera", {500, 380});
  for (int frame = 0; frame < 8; ++frame)
    ASSERT_TRUE(app.Loop());
  EXPECT_NE(gui->GetRuntimeGuiProof()->GetOverlay(), overlay);
  std::vector<glm::vec4> camera_after;
  playing_camera->GetRenderTexture()->GetRgbaChannelData(camera_after);
  ASSERT_EQ(camera_before.size(), camera_after.size());
  EXPECT_EQ(camera_before, camera_after);
  const auto* main_viewport = ImGui::GetMainViewport();
  ImGui::SetWindowPos("Camera", {main_viewport->Pos.x + main_viewport->Size.x + 50, main_viewport->Pos.y + 20});
  for (int frame = 0; frame < 16; ++frame)
    ASSERT_TRUE(app.Loop());
  EXPECT_NE(ImGui::FindWindowByName("Camera")->Viewport, main_viewport);
  EXPECT_EQ(ImGui::FindWindowByName("Runtime GUI proof###RuntimeGui/1/Runtime GUI proof")->Viewport,
            ImGui::FindWindowByName("Camera")->Viewport);
  ImGui::SetWindowPos("Camera", {main_viewport->Pos.x + 10, main_viewport->Pos.y + 30});
  for (int frame = 0; frame < 16; ++frame)
    ASSERT_TRUE(app.Loop());
  EXPECT_EQ(ImGui::FindWindowByName("Camera")->Viewport, main_viewport);
  auto* camera_host = ImGui::FindWindowByName("Camera")->DC.ChildWindows.front();
  auto* runtime_window = ImGui::FindWindowByName("Runtime GUI proof###RuntimeGui/1/Runtime GUI proof");
  ImGui::FocusWindow(camera_host);
  RuntimeInputTestAccess::Key(window->GetGlfwWindow(), GLFW_PRESS);
  ASSERT_TRUE(app.Loop());
  EXPECT_NE(app.GetActiveScene()->GetKey(GLFW_KEY_W), Input::KeyActionType::Release);
  ImGui::FocusWindow(runtime_window);
  ASSERT_TRUE(app.Loop());
  EXPECT_EQ(app.GetActiveScene()->GetKey(GLFW_KEY_W), Input::KeyActionType::Release);
  EXPECT_EQ(RuntimeInputTestAccess::RawKey(), Input::KeyActionType::Hold);
  RuntimeInputTestAccess::Key(window->GetGlfwWindow(), GLFW_RELEASE);
  RuntimeInputTestAccess::Key(window->GetGlfwWindow(), GLFW_PRESS);
  ASSERT_TRUE(app.Loop());
  EXPECT_EQ(app.GetActiveScene()->GetKey(GLFW_KEY_W), Input::KeyActionType::Release);
  RuntimeInputTestAccess::Key(window->GetGlfwWindow(), GLFW_RELEASE);
  ASSERT_TRUE(app.Loop());
  EXPECT_EQ(RuntimeInputTestAccess::RawKey(), Input::KeyActionType::Release);
  if (const char* seconds = std::getenv("EVOENGINE_RUNTIME_GUI_PROOF_SECONDS")) {
    const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(std::atoi(seconds));
    while (std::chrono::steady_clock::now() < until && app.Loop()) {
    }
  }
  app.Stop();
  overlay.reset();
  playing_camera.reset();
  camera.reset();
  scene.reset();
  editor.reset();
  gui.reset();
  window.reset();
  std::weak_ptr<RenderInstanceStorage> instances = app.GetLayer<RenderLayer>()->GetCurrentRenderInstanceStorage();
  app.Terminate();
  EXPECT_TRUE(instances.expired());
  EXPECT_EQ(ImGui::GetCurrentContext(), nullptr);
}

TEST(RuntimeGuiProof, StoppedLayoutsPersistAndPlayChangesRemainTemporary) {
  if (!std::getenv("EVOENGINE_RUNTIME_GUI_PROOF"))
    GTEST_SKIP();
  Application app;
  ApplicationContextScope scope(app);
  app.PushLayer<WindowLayer>("Window");
  app.PushLayer<RenderLayer>("Rendering");
  app.PushLayer<ImGuiLayer>("GUI");
  app.PushLayer<RuntimeGuiLayer>("Runtime GUI");
  auto editor = app.PushLayer<EditorLayer>("Editor");
  ApplicationInitializationSettings settings;
  settings.allow_empty_project = true;
  settings.default_window_size = {800, 600};
  settings.load_project_assets = settings.load_project_start_scene = false;
  settings.redirect_standard_streams_to_console = false;
  settings.graphics_settings.use_ray_tracing = settings.graphics_settings.use_mesh_shader = false;
  app.Initialize(settings);
  auto source = AssetManager::CreateTemporaryAsset<Scene>();
  auto camera = source->GetOrSetPrivateComponent<Camera>(source->CreateEntity("Main camera")).lock();
  source->main_camera = camera;
  auto component = source->GetOrSetPrivateComponent<RuntimeGui>(source->CreateEntity("GUI")).lock();
  component->camera = camera;
  auto asset = AssetManager::CreateTemporaryAsset<RuntimeDebugGui>();
  component->AddGuiAsset(asset);
  ProjectManager::SetStartScene(source);
  app.Attach(source);
  editor->show_scene_window = false;
  editor->show_camera_info = false;
  ImGui::LoadIniSettingsFromMemory("[Window][Camera]\nPos=10,30\nSize=700,500\nCollapsed=0\n");
  app.Start(false);
  for (int frame = 0; frame < 5; ++frame)
    ASSERT_TRUE(app.Loop());
  ASSERT_FALSE(component->GetLayout().empty());
  EXPECT_NE(component->GetLayout().find("IsChild=1"), std::string::npos);
  const auto find_window = [](const std::shared_ptr<Scene>& scene) -> ImGuiWindow* {
    const auto prefix = "Runtime debug###RuntimeGui/" + std::to_string(scene->GetHandle()) + "/";
    for (auto* window : GImGui->Windows)
      if (std::string(window->Name).rfind(prefix, 0) == 0)
        return window;
    return nullptr;
  };
  auto* authored_window = find_window(source);
  ASSERT_NE(authored_window, nullptr);
  ImGui::SetWindowPos(authored_window, {authored_window->Pos.x + 20, authored_window->Pos.y + 10});
  ImGui::SetWindowSize(authored_window, {300, 260});
  for (int frame = 0; frame < 3; ++frame)
    ASSERT_TRUE(app.Loop());
  const auto saved = component->GetLayout();
  EXPECT_FALSE(source->Saved());
  app.Play();
  auto playing = app.GetActiveScene();
  auto live = playing->GetOrSetPrivateComponent<RuntimeGui>(component->GetOwner()).lock();
  EXPECT_EQ(live->GetLayout(), saved);
  live->draw_order = 99;
  for (int frame = 0; frame < 5; ++frame)
    ASSERT_TRUE(app.Loop());
  auto* playing_window = find_window(playing);
  ASSERT_NE(playing_window, nullptr);
  ImGui::SetWindowSize(playing_window, {340, 290});
  for (int frame = 0; frame < 3; ++frame)
    ASSERT_TRUE(app.Loop());
  EXPECT_NE(live->GetLayout(), saved);
  EXPECT_EQ(component->GetLayout(), saved);
  EXPECT_EQ(component->draw_order, 0);
  auto dynamic = playing->GetOrSetPrivateComponent<RuntimeGui>(playing->CreateEntity("Dynamic GUI")).lock();
  dynamic->camera = playing->main_camera;
  dynamic->AddGuiAsset(asset);
  ASSERT_TRUE(app.Loop());
  EXPECT_EQ(source->GetPrivateComponentOwnersList<RuntimeGui>().size(), 1);
  app.Pause();
  ASSERT_TRUE(app.Loop());
  EXPECT_EQ(component->GetLayout(), saved);
  app.Stop();
  EXPECT_EQ(app.GetActiveScene(), source);
  for (int frame = 0; frame < 3; ++frame)
    ASSERT_TRUE(app.Loop());
  EXPECT_EQ(component->GetLayout(), saved);
  app.Play();
  const auto owner = component->GetOwner();
  source->RemovePrivateComponent<RuntimeGui>(owner);
  auto replacement = source->GetOrSetPrivateComponent<RuntimeGui>(owner).lock();
  replacement->SetLayout("replacement layout must not be overwritten");
  for (int frame = 0; frame < 3; ++frame)
    ASSERT_TRUE(app.Loop());
  EXPECT_EQ(replacement->GetLayout(), "replacement layout must not be overwritten");
  source->DeleteEntity(owner);
  ASSERT_TRUE(app.Loop());
  EXPECT_FALSE(source->IsEntityValid(owner));
  app.Stop();
  EXPECT_TRUE(source->GetPrivateComponentOwnersList<RuntimeGui>().empty());
  replacement.reset();
  dynamic.reset();
  live.reset();
  playing.reset();
  component.reset();
  asset.reset();
  camera.reset();
  source.reset();
  app.Terminate();
}
