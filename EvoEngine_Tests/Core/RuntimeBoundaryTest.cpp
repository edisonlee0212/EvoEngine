#include "Application.hpp"
#include "AssetManager.hpp"
#include "Camera.hpp"
#include "ClassRegistry.hpp"
#include "PackageManager.hpp"
#include "Platform.hpp"
#include "ProjectManager.hpp"
#include "RenderLayer.hpp"
#include "Scene.hpp"
#include "WindowLayer.hpp"

#include <Windows.h>
#include <gtest/gtest.h>

using namespace evo_engine;

namespace evo_engine {
struct RuntimeInputTestAccess {
  static void AltEnter(GLFWwindow* window, const int action = GLFW_PRESS) {
    Input::KeyCallBack(window, GLFW_KEY_ENTER, 0, action, GLFW_MOD_ALT);
  }
  static void KeyCallback(GLFWwindow* window, const int action) {
    Input::KeyCallBack(window, GLFW_KEY_A, 0, action, 0);
  }
};
}  // namespace evo_engine

namespace {
class RuntimeComponent final : public IPrivateComponent {};

class RuntimeBoundaryTest : public testing::Test {
 protected:
  Application application_;
  ApplicationContextScope scope_{application_};

  void SetUp() override {
    ApplicationInitializationSettings settings;
    settings.application_mode = ApplicationMode::Headless;
    settings.allow_empty_project = true;
    settings.load_default_resources = false;
    settings.load_project_assets = false;
    settings.load_project_start_scene = false;
    application_.Initialize(settings);
  }

  void TearDown() override {
    application_.Terminate();
  }
};

class RuntimeWindowTest : public RuntimeBoundaryTest {
 protected:
  virtual void ConfigureWindow(ApplicationInitializationSettings&) {
  }
  void SetUp() override {
    application_.PushLayer<WindowLayer>("Runtime window");
    application_.PushLayer<RenderLayer>("Runtime rendering");
    ApplicationInitializationSettings settings;
    settings.application_mode = ApplicationMode::Player;
    settings.application_name = "Runtime boundary smoke test";
    settings.default_window_size = {320, 240};
    settings.allow_empty_project = true;
    settings.load_project_assets = false;
    settings.load_project_start_scene = false;
    settings.hide_console_window = false;
    settings.redirect_standard_streams_to_console = false;
    settings.graphics_settings.use_ray_tracing = false;
    settings.graphics_settings.use_mesh_shader = false;
    ConfigureWindow(settings);
    application_.Initialize(settings);
  }
};
class RuntimeDisplayTest : public RuntimeWindowTest, public testing::WithParamInterface<WindowDisplayMode> {
 protected:
  void ConfigureWindow(ApplicationInitializationSettings& settings) override {
    settings.window_mode = GetParam();
  }
};

class RuntimeDisplayPermissionsTest : public RuntimeWindowTest {
 protected:
  void ConfigureWindow(ApplicationInitializationSettings& settings) override {
    settings.window_mode = WindowDisplayMode::Windowed;
    settings.window_resizable = false;
    settings.allow_resolution_change = false;
  }
};
}  // namespace

TEST_F(RuntimeBoundaryTest, RegistersAndCreatesComponentsWithoutInspectorRegistry) {
  application_.RegisterPrivateComponent<RuntimeComponent>("Runtime boundary component");
  const auto scene = AssetManager::CreateTemporaryAsset<Scene>();
  const auto entity = scene->CreateEntity("Runtime entity");
  EXPECT_NE(scene->GetOrSetPrivateComponent<RuntimeComponent>(entity).lock(), nullptr);
}

TEST_F(RuntimeBoundaryTest, ClonesSceneDataWithoutEditorLayers) {
  const auto source = AssetManager::CreateTemporaryAsset<Scene>();
  const auto entity = source->CreateEntity("Runtime entity");
  Transform transform;
  transform.SetPosition({1.0f, 2.0f, 3.0f});
  source->SetDataComponent(entity, transform);
  const auto clone = AssetManager::CreateTemporaryAsset<Scene>();
  Scene::Clone(source, clone);
  EXPECT_EQ(clone->GetDataComponent<Transform>(entity).GetPosition(), transform.GetPosition());
}

TEST(RuntimeLifecycle, HostControlsAutoplayAndBusyOperationsWithoutEditorDependencies) {
  class HostLayer final : public ILayer {
   public:
    int starts = 0;
    bool AllowsAutoplay() const override {
      return false;
    }
    void OnRuntimeStart() override {
      ++starts;
    }
  };
  Application application;
  ApplicationContextScope scope(application);
  const auto host = application.PushLayer<HostLayer>("Host");
  ApplicationInitializationSettings settings;
  settings.application_mode = ApplicationMode::Headless;
  settings.allow_empty_project = true;
  settings.load_default_resources = false;
  settings.load_project_assets = false;
  settings.load_project_start_scene = false;
  application.Initialize(settings);
  const auto scene = AssetManager::CreateTemporaryAsset<Scene>();
  ProjectManager::SetStartScene(scene);
  application.Attach(scene);
  application.Start(true);
  EXPECT_EQ(application.GetApplicationStatus(), Application::ExecutionStatus::NotPlaying);
  bool busy = true;
  application.SetRuntimeOperationBusyPredicate([&] {
    return busy;
  });
  EXPECT_FALSE(PackageManager::CanModifyPackages());
  application.Play();
  application.Step();
  EXPECT_EQ(host->starts, 0);
  EXPECT_EQ(application.GetApplicationStatus(), Application::ExecutionStatus::NotPlaying);
  {
    ApplicationContextScope restore_scope(application);
    Application other;
    ApplicationContextScope other_scope(other);
    EXPECT_FALSE(other.RuntimeOperationBusy());
    EXPECT_TRUE(PackageManager::CanModifyPackages());
  }
  busy = false;
  EXPECT_TRUE(PackageManager::CanModifyPackages());
  application.Play();
  EXPECT_EQ(host->starts, 1);
  application.Pause();
  application.Play();
  EXPECT_EQ(host->starts, 1);
  application.Stop();
  application.Step();
  EXPECT_EQ(host->starts, 2);
  application.Stop();
  busy = true;
  application.SetRuntimeOperationBusyPredicate({});
  EXPECT_FALSE(application.RuntimeOperationBusy());
  application.Terminate();
}

TEST_F(RuntimeWindowTest, RendersMainCameraAndDispatchesSceneInputWithoutEditor) {
  const auto scene = AssetManager::CreateTemporaryAsset<Scene>();
  const auto camera = scene->GetOrSetPrivateComponent<Camera>(scene->CreateEntity("Main camera")).lock();
  scene->main_camera = camera;
  ProjectManager::SetStartScene(scene);
  application_.Attach(scene);
  application_.Start(true);
  for (int frame = 0; frame < 3; ++frame) {
    ASSERT_TRUE(application_.Loop());
  }
  const auto playing_scene = application_.GetActiveScene();
  const auto playing_camera = playing_scene->main_camera.Get<Camera>();
  ASSERT_NE(playing_camera, nullptr);
  EXPECT_GT(playing_camera->GetFrameCount(), 0u);

  const auto window = application_.GetLayer<WindowLayer>()->GetGlfwWindow();
  const auto extent = Platform::GetSwapchain()->GetImageExtent();
  EXPECT_EQ(playing_camera->GetSize(), glm::uvec2(extent.width, extent.height));
  const auto window_layer = application_.GetLayer<WindowLayer>();
  window_layer->SetAutoRenderMainCamera(false);
  playing_camera->Resize({160, 120});
  ASSERT_TRUE(application_.Loop());
  EXPECT_EQ(playing_camera->GetSize(), glm::uvec2(160, 120));
  window_layer->SetAutoRenderMainCamera(true);
  ASSERT_TRUE(application_.Loop());
  EXPECT_EQ(playing_camera->GetSize(), glm::uvec2(extent.width, extent.height));
  RuntimeInputTestAccess::KeyCallback(window, GLFW_PRESS);
  EXPECT_EQ(playing_scene->GetKey(GLFW_KEY_A), Input::KeyActionType::Press);
  RuntimeInputTestAccess::KeyCallback(window, GLFW_RELEASE);
  EXPECT_EQ(playing_scene->GetKey(GLFW_KEY_A), Input::KeyActionType::Release);
}

TEST_P(RuntimeDisplayTest, MainCameraTracksWindowResolutionAcrossDisplayModes) {
  const auto scene = AssetManager::CreateTemporaryAsset<Scene>();
  const auto camera = scene->GetOrSetPrivateComponent<Camera>(scene->CreateEntity("Main camera")).lock();
  camera->Resize({160, 120});
  scene->main_camera = camera;
  ProjectManager::SetStartScene(scene);
  application_.Attach(scene);
  application_.Start(true);
  const auto layer = application_.GetLayer<WindowLayer>();
  const auto settle = [&] {
    for (int frame = 0; frame < 3; ++frame) {
      EXPECT_TRUE(application_.Loop());
    }
  };
  const auto check = [&](WindowDisplayMode requested) {
    settle();
    const auto mode = layer->GetDisplayMode();
    if (requested == WindowDisplayMode::ExclusiveFullscreen) {
      EXPECT_TRUE(mode == requested || mode == WindowDisplayMode::Windowed);
    } else {
      EXPECT_EQ(mode, requested);
    }
    const auto hwnd = FindWindowW(nullptr, L"Runtime boundary smoke test");
    ASSERT_NE(hwnd, nullptr);
    EXPECT_EQ((GetWindowLongPtrW(hwnd, GWL_STYLE) & WS_CAPTION) != 0, mode == WindowDisplayMode::Windowed);
    if (WindowModePolicy::IsFullscreen(mode)) {
      DEVMODEW display{};
      display.dmSize = sizeof(display);
      ASSERT_TRUE(EnumDisplaySettingsW(nullptr, ENUM_CURRENT_SETTINGS, &display));
      EXPECT_EQ(layer->GetFramebufferSize(), glm::ivec2(display.dmPelsWidth, display.dmPelsHeight));
      RECT rect{};
      ASSERT_TRUE(GetWindowRect(hwnd, &rect));
      EXPECT_EQ(rect.left, 0);
      EXPECT_EQ(rect.top, 0);
      EXPECT_FALSE(layer->SetResolution(640, 480));
    }
    const auto active_camera = application_.GetActiveScene()->main_camera.Get<Camera>();
    EXPECT_EQ(active_camera->GetSize(), glm::uvec2(layer->GetFramebufferSize()));
    EXPECT_GT(active_camera->GetFrameCount(), 0u);
  };
  check(GetParam());
  for (auto mode : {WindowDisplayMode::Windowed, WindowDisplayMode::BorderlessWindowed,
                    WindowDisplayMode::BorderlessFullscreen, WindowDisplayMode::ExclusiveFullscreen}) {
    ASSERT_TRUE(layer->SetDisplayMode(mode));
    check(mode);
  }
  ASSERT_TRUE(layer->SetDisplayMode(WindowDisplayMode::BorderlessWindowed));
  ASSERT_TRUE(layer->SetResolution(400, 300));
  settle();
  ASSERT_TRUE(layer->SetDisplayMode(WindowDisplayMode::BorderlessFullscreen));
  settle();
  RuntimeInputTestAccess::AltEnter(layer->GetGlfwWindow());
  check(WindowDisplayMode::BorderlessWindowed);
  EXPECT_EQ(layer->GetFramebufferSize(), glm::ivec2(400, 300));
  RuntimeInputTestAccess::AltEnter(layer->GetGlfwWindow(), GLFW_REPEAT);
  EXPECT_EQ(layer->GetDisplayMode(), WindowDisplayMode::BorderlessWindowed);
  RuntimeInputTestAccess::AltEnter(layer->GetGlfwWindow());
  check(WindowDisplayMode::BorderlessFullscreen);
}

INSTANTIATE_TEST_SUITE_P(RuntimeModes, RuntimeDisplayTest,
                         testing::Values(WindowDisplayMode::Windowed, WindowDisplayMode::BorderlessWindowed,
                                         WindowDisplayMode::BorderlessFullscreen,
                                         WindowDisplayMode::ExclusiveFullscreen));

TEST_F(RuntimeDisplayPermissionsTest, ModeSwitchingRemainsAvailableWithBothPermissionsDisabled) {
  const auto layer = application_.GetLayer<WindowLayer>();
  EXPECT_FALSE(layer->SetResolution(600, 400));
  const auto hwnd = FindWindowW(nullptr, L"Runtime boundary smoke test");
  ASSERT_NE(hwnd, nullptr);
  EXPECT_EQ(GetWindowLongPtrW(hwnd, GWL_STYLE) & WS_THICKFRAME, 0);
  RuntimeInputTestAccess::AltEnter(layer->GetGlfwWindow());
  for (int frame = 0; frame < 3; ++frame)
    ASSERT_TRUE(application_.Loop());
  EXPECT_EQ(layer->GetDisplayMode(), WindowDisplayMode::BorderlessFullscreen);
  RuntimeInputTestAccess::AltEnter(layer->GetGlfwWindow());
  for (int frame = 0; frame < 3; ++frame)
    ASSERT_TRUE(application_.Loop());
  EXPECT_EQ(layer->GetDisplayMode(), WindowDisplayMode::Windowed);
  EXPECT_EQ(layer->GetFramebufferSize(), glm::ivec2(320, 240));
}

TEST(RuntimeInput, LayerCanConsumeInputBeforeItReachesTheScene) {
  class InputLayer final : public ILayer {
   public:
    bool consume = true;
    int received = 0;
    bool OnInputEvent(const Input::InputEvent&) override {
      ++received;
      return consume;
    }
  };
  Application application;
  ApplicationContextScope scope(application);
  const auto layer = application.PushLayer<InputLayer>("Input consumer");
  ASSERT_TRUE(layer);
  ApplicationInitializationSettings settings;
  settings.application_mode = ApplicationMode::Headless;
  settings.allow_empty_project = true;
  settings.load_default_resources = false;
  settings.load_project_assets = false;
  settings.load_project_start_scene = false;
  application.Initialize(settings);
  const auto scene = AssetManager::CreateTemporaryAsset<Scene>();
  application.Attach(scene);
  RuntimeInputTestAccess::KeyCallback(nullptr, GLFW_PRESS);
  EXPECT_EQ(layer->received, 1);
  EXPECT_EQ(scene->GetKey(GLFW_KEY_A), Input::KeyActionType::Release);
  layer->consume = false;
  RuntimeInputTestAccess::KeyCallback(nullptr, GLFW_PRESS);
  EXPECT_EQ(layer->received, 2);
  EXPECT_EQ(scene->GetKey(GLFW_KEY_A), Input::KeyActionType::Press);
  RuntimeInputTestAccess::KeyCallback(nullptr, GLFW_RELEASE);
  EXPECT_EQ(scene->GetKey(GLFW_KEY_A), Input::KeyActionType::Release);
  application.Terminate();
}

TEST(RuntimeConsole, RetainsBoundedMessagesAndDrainsWithoutAnEditor) {
  static_cast<void>(Console::DrainPendingMessages());
  for (int index = 0; index < 10005; ++index)
    Console::Log(std::to_string(index));
  const auto messages = Console::DrainPendingMessages();
  ASSERT_EQ(messages.size(), 10000u);
  EXPECT_EQ(messages.front().m_value, "5");
  EXPECT_EQ(messages.back().m_value, "10004");
  EXPECT_EQ(messages.front().m_type, ConsoleMessageType::Log);
  EXPECT_GT(messages.front().m_timestamp, 0);
  EXPECT_TRUE(Console::DrainPendingMessages().empty());
}
