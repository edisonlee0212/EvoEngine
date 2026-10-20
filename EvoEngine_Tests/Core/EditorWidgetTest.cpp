#include <gtest/gtest.h>
#include <imgui_internal.h>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <thread>
#include "Application.hpp"
#include "AssetManager.hpp"
#include "Camera.hpp"
#include "CurveEditors.hpp"
#include "EditorLayer.hpp"
#include "EditorTextureRegistry.hpp"
#include "EditorTheme.hpp"
#include "EvoEngine_SDK_PCH.hpp"
#include "ImGuiLayer.hpp"
#include "MeshRenderer.hpp"
#include "OffscreenPreviewRenderer.hpp"
#include "RenderLayer.hpp"
#include "Resources.hpp"
#include "Scene.hpp"
#include "Shader.hpp"
#include "Texture2D.hpp"
#include "TextureStorage.hpp"
#include "WindowLayer.hpp"

using namespace evo_engine;

namespace evo_engine {
struct EditorStatusBarTestAccess {
  static void Draw(EditorLayer& editor, const float top_offset = 0) {
    const auto height = editor.DrawStatusBar();
    editor.DrawDockspace(top_offset, height);
  }
  static void Poll(EditorLayer& editor) {
    editor.PollConsoleMessages();
  }
  static void Clear(EditorLayer& editor) {
    editor.ClearConsoleMessages();
  }
  static std::pair<size_t, size_t> Counts(const EditorLayer& editor) {
    return {editor.status_warning_count_, editor.status_error_count_};
  }
  static std::array<bool, 3> Filters(const EditorLayer& editor) {
    return {editor.enable_console_logs_, editor.enable_console_warnings_, editor.enable_console_errors_};
  }
};
}  // namespace evo_engine

namespace {
class StartupWindowProbe final : public ILayer {
 public:
  bool observed = false;
  std::filesystem::path capture_path;

  void OnCreate() override {
    const auto window = GetApplication().GetLayer<WindowLayer>()->GetGlfwWindow();
    int x, y, width, height, work_x, work_y, work_width, work_height;
    glfwGetWindowPos(window, &x, &y);
    glfwGetWindowSize(window, &width, &height);
    glfwGetMonitorWorkarea(glfwGetPrimaryMonitor(), &work_x, &work_y, &work_width, &work_height);
    const auto startup = ImGui::FindWindowByName("Starting EvoEngine");
    ASSERT_NE(startup, nullptr);
    const auto padding = ImGui::GetStyle().WindowPadding;
    EXPECT_NEAR(width, startup->DC.CursorMaxPos.x - startup->Pos.x + padding.x, 1);
    EXPECT_NEAR(height, startup->DC.CursorMaxPos.y - startup->Pos.y + padding.y, 1);
    EXPECT_LT(width, 640);
    EXPECT_LT(height, 240);
    EXPECT_EQ(x, work_x + std::max(0, (work_width - width) / 2));
    EXPECT_EQ(y, work_y + std::max(0, (work_height - height) / 2));
    EXPECT_EQ(glfwGetWindowAttrib(window, GLFW_MAXIMIZED), GLFW_FALSE);
    observed = true;
    if (!capture_path.empty()) {
      const auto layer = GetApplication().GetLayer<WindowLayer>();
      const auto current = Shader::GetCurrentPreparation();
      const auto redraw = [&] {
        return Shader::RunPreparationTask(current, [] {
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
        });
      };
      ASSERT_TRUE(redraw());
      layer->RequestScreenshot(capture_path);
      ASSERT_TRUE(redraw());
      std::string error;
      ASSERT_TRUE(layer->StoreCompletedScreenshot(error)) << error;
    }
  }
};
}  // namespace

TEST(ImGuiStartup, CompactWindowIsCenteredAndRestoresConfiguredDisplay) {
  for (int mode = 0; mode < 4; ++mode) {
    SCOPED_TRACE(mode);
    Application app;
    ApplicationContextScope scope(app);
    auto window = app.PushLayer<WindowLayer>("Window");
    app.PushLayer<RenderLayer>("Rendering");
    app.PushLayer<ImGuiLayer>("GUI");
    const auto probe = app.PushLayer<StartupWindowProbe>("Startup probe");
    if (const auto directory = std::getenv("EVOENGINE_EDITOR_STARTUP_CAPTURE")) {
      std::filesystem::create_directories(directory);
      probe->capture_path = std::filesystem::path(directory) / ("startup-" + std::to_string(mode) + ".png");
    }
    ApplicationInitializationSettings settings;
    settings.allow_empty_project = true;
    settings.default_window_size = {800, 600};
    settings.full_screen = mode == 1;
    if (mode == 2)
      settings.window_mode = WindowDisplayMode::Windowed;
    settings.use_custom_title_bar = mode == 3;
    settings.load_project_assets = settings.load_project_start_scene = false;
    settings.redirect_standard_streams_to_console = false;
    settings.graphics_settings.use_ray_tracing = settings.graphics_settings.use_mesh_shader = false;
    app.Initialize(settings);
    glfwPollEvents();
    EXPECT_TRUE(probe->observed);
    EXPECT_EQ(window->IsWindowMaximized(), mode == 1);
    if (mode != 1) {
      int width, height;
      glfwGetWindowSize(window->GetGlfwWindow(), &width, &height);
      EXPECT_EQ(width, 800);
      EXPECT_EQ(height, 600);
    }
    app.Terminate();
  }
}

TEST(ImGuiStartup, LauncherThemesWorkWithoutNodeEditorContext) {
  ImGui::CreateContext();
  EXPECT_EQ(ImNodes::GetCurrentContext(), nullptr);
  editor_theme::ApplyDefault();
  EXPECT_EQ(editor_theme::GetCurrentTheme(), editor_theme::Theme::Dark);
  const auto dark = ImGui::GetStyle().Colors[ImGuiCol_WindowBg];
  editor_theme::Apply(editor_theme::Theme::Light);
  EXPECT_EQ(editor_theme::GetCurrentTheme(), editor_theme::Theme::Light);
  EXPECT_GT(ImGui::GetStyle().Colors[ImGuiCol_WindowBg].x, dark.x);
  EXPECT_EQ(ImNodes::GetCurrentContext(), nullptr);
  editor_theme::ApplyDefault();
  ImGui::DestroyContext();
}

TEST(ImGuiStartup, CameraInfoSurvivesClearingAndRestoringMainCamera) {
  Application app;
  ApplicationContextScope scope(app);
  app.PushLayer<WindowLayer>("Window");
  app.PushLayer<RenderLayer>("Rendering");
  app.PushLayer<ImGuiLayer>("GUI");
  auto editor = app.PushLayer<EditorLayer>("Editor");
  ApplicationInitializationSettings settings;
  settings.allow_empty_project = true;
  settings.default_window_size = {640, 480};
  settings.load_project_assets = settings.load_project_start_scene = false;
  settings.redirect_standard_streams_to_console = false;
  settings.graphics_settings.use_ray_tracing = settings.graphics_settings.use_mesh_shader = false;
  app.Initialize(settings);
  auto scene = AssetManager::CreateTemporaryAsset<Scene>();
  auto original = scene->GetOrSetPrivateComponent<Camera>(scene->CreateEntity("Original camera")).lock();
  auto replacement = scene->GetOrSetPrivateComponent<Camera>(scene->CreateEntity("New camera")).lock();
  scene->main_camera = original;
  app.Attach(scene);
  editor->show_scene_window = false;
  editor->show_camera_window = editor->show_camera_info = true;
  ImGui::LoadIniSettingsFromMemory("[Window][Camera]\nPos=10,30\nSize=600,400\nCollapsed=0\n");
  app.Start(false);
  for (int frame = 0; frame < 3; ++frame)
    ASSERT_TRUE(app.Loop());
  EXPECT_TRUE(original->Rendered());
  scene->main_camera = replacement;
  for (int frame = 0; frame < 3; ++frame)
    ASSERT_TRUE(app.Loop());
  EXPECT_TRUE(replacement->Rendered());
  scene->main_camera.Clear();
  for (int frame = 0; frame < 3; ++frame)
    ASSERT_TRUE(app.Loop());
  EXPECT_EQ(scene->main_camera.Get<Camera>(), nullptr);
  scene->main_camera = original;
  for (int frame = 0; frame < 3; ++frame)
    ASSERT_TRUE(app.Loop());
  EXPECT_TRUE(original->Rendered());
  original.reset();
  replacement.reset();
  scene.reset();
  editor.reset();
  app.Terminate();
}

TEST(ImGuiStartup, EditorDrawsGizmosAfterSceneRendering) {
  Application app;
  ApplicationContextScope scope(app);
  app.PushLayer<WindowLayer>("Window");
  app.PushLayer<RenderLayer>("Rendering");
  app.PushLayer<ImGuiLayer>("GUI");
  auto editor = app.PushLayer<EditorLayer>("Editor");
  ApplicationInitializationSettings settings;
  settings.allow_empty_project = true;
  settings.default_window_size = {640, 480};
  settings.load_project_assets = false;
  settings.load_project_start_scene = false;
  settings.hide_console_window = false;
  settings.redirect_standard_streams_to_console = false;
  settings.graphics_settings.use_ray_tracing = false;
  settings.graphics_settings.use_mesh_shader = false;
  app.Initialize(settings);
  auto scene = AssetManager::CreateTemporaryAsset<Scene>();
  app.Attach(scene);
  const auto selected = scene->CreateEntity("Selected cube");
  {
    const auto renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(selected).lock();
    renderer->mesh.Set(Resources::GetInstance().GetPrimitives().cube);
    renderer->material.Set(AssetManager::CreateTemporaryAsset<Material>());
  }
  editor->SetSceneCameraResolutionOverride(glm::uvec2(128, 128));
  editor->SetSceneCameraPosition({0, 0, 5});
  editor->SetSceneCameraRotation(glm::quat(1, 0, 0, 0));
  {
    const auto render = app.GetLayer<RenderLayer>();
    std::vector<RenderLayer::CameraView> views;
    render->CollectAuxiliaryCameras(views);
    ASSERT_EQ(views.size(), 1u);
    EXPECT_EQ(views.front().second, editor->GetSceneCamera());
    EXPECT_EQ(render->GetPrimaryAuxiliaryCamera().first.GetPosition(), glm::vec3(0, 0, 5));
    views.clear();
    editor->GetSceneCamera()->SetEnabled(false);
    render->CollectAuxiliaryCameras(views);
    EXPECT_TRUE(views.empty());
    editor->GetSceneCamera()->SetEnabled(true);
  }
  app.RegisterUpdateFunction([&] {
    editor->GetSceneCamera()->SetRequireRendering(true);
    GizmoSettings gizmo;
    gizmo.draw_settings.polygon_mode = VK_POLYGON_MODE_FILL;
    gizmo.draw_settings.blending = false;
    editor->DrawGizmoCube({1, 0, 0, 1}, glm::mat4(1), 1, gizmo);
  });
  app.Start();
  for (int frame = 0; frame < 4; ++frame)
    ASSERT_TRUE(app.Loop());
  EXPECT_TRUE(editor->GetSceneCamera()->Rendered());
  std::vector<glm::vec4> pixels;
  editor->GetSceneCamera()->GetRenderTexture()->GetRgbaChannelData(pixels);
  ASSERT_EQ(pixels.size(), 128u * 128u);
  EXPECT_TRUE(std::any_of(pixels.begin(), pixels.end(), [](const auto& pixel) {
    return pixel.r > 0.9f && pixel.g < 0.1f && pixel.b < 0.1f;
  }));
  editor->SetSelectedEntity(selected, false);
  for (int frame = 0; frame < 120; ++frame) {
    ASSERT_TRUE(app.Loop());
    if (editor->GetEntitySelectionHighlightSnapshot().fade_progress >= 1.0f)
      break;
  }
  EXPECT_TRUE(editor->GetEntitySelectionHighlightSnapshot().active);
  editor->GetSceneCamera()->GetRenderTexture()->GetRgbaChannelData(pixels);
  EXPECT_TRUE(std::any_of(pixels.begin(), pixels.end(), [](const auto& pixel) {
    return pixel.r > 0.8f && pixel.g > 0.4f && pixel.b < 0.15f;
  }));
  scene.reset();
  app.Terminate();
  editor.reset();
  EXPECT_EQ(ImGui::GetCurrentContext(), nullptr);
}

namespace {
class EditorWidgetTest : public testing::Test {
 protected:
  ImGuiContext* previous_ = nullptr;
  ImGuiContext* context_ = nullptr;
  void SetUp() override {
    previous_ = ImGui::GetCurrentContext();
    context_ = ImGui::CreateContext();
    ImGui::SetCurrentContext(context_);
    auto& io = ImGui::GetIO();
    io.IniFilename = nullptr;
    io.DisplaySize = {1600, 1200};
    io.DeltaTime = 1.0f / 60.0f;
    unsigned char* pixels;
    int width, height;
    io.Fonts->GetTexDataAsRGBA32(&pixels, &width, &height);
  }
  void TearDown() override {
    ImGui::DestroyContext(context_);
    ImGui::SetCurrentContext(previous_);
  }
  void Begin() {
    ImGui::NewFrame();
    ImGui::SetNextWindowPos({40, 40}, ImGuiCond_Always);
    ImGui::SetNextWindowSize({1500, 1100}, ImGuiCond_Always);
    ImGui::Begin("Curve controls");
    ImGui::PushItemWidth(100);
  }
  void End() {
    ImGui::PopItemWidth();
    ImGui::End();
    ImGui::Render();
  }
};
}  // namespace

TEST_F(EditorWidgetTest, StatusBarReservesSpaceWithBothTitleBarsAndNarrowWindows) {
  Application app;
  ApplicationContextScope scope(app);
  const auto editor = app.PushLayer<EditorLayer>("Editor");
  ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_DockingEnable;
  for (const float width : {1200.0f, 360.0f}) {
    for (const float top_offset : {0.0f, 42.0f}) {
      ImGui::GetIO().DisplaySize = {width, 800};
      ImGui::NewFrame();
      EditorStatusBarTestAccess::Draw(*editor, top_offset);
      ImGui::Render();
      const auto bar = ImGui::FindWindowByName("##EditorStatusBar");
      const auto dock = ImGui::FindWindowByName("Root DockSpace");
      ASSERT_NE(bar, nullptr);
      ASSERT_NE(dock, nullptr);
      EXPECT_FLOAT_EQ(bar->Size.x, width);
      EXPECT_FLOAT_EQ(bar->Size.y, ImGui::GetFrameHeight());
      EXPECT_FLOAT_EQ(bar->Pos.y + bar->Size.y, 800);
      EXPECT_LE(dock->Pos.y + dock->Size.y, bar->Pos.y);
      EXPECT_TRUE(bar->Flags & ImGuiWindowFlags_NoDocking);
      EXPECT_TRUE(bar->Flags & ImGuiWindowFlags_NoSavedSettings);
      EXPECT_LE(bar->DC.CursorMaxPos.x, bar->InnerRect.Max.x + 1.0f);
    }
  }
}

TEST_F(EditorWidgetTest, StatusBarCountsConsoleMessagesOpensErrorFilterAndClearsCounts) {
  Application app;
  ApplicationContextScope scope(app);
  const auto editor = app.PushLayer<EditorLayer>("Editor");
  editor->show_console_window = false;
  EditorStatusBarTestAccess::Clear(*editor);
  Console::Warning("First warning");
  Console::Warning("Second warning");
  Console::Error("An error");
  EditorStatusBarTestAccess::Poll(*editor);
  const auto draw = [&] {
    ImGui::NewFrame();
    EditorStatusBarTestAccess::Draw(*editor);
    ImGui::Render();
  };
  draw();
  EXPECT_EQ(EditorStatusBarTestAccess::Counts(*editor), (std::pair<size_t, size_t>{2, 1}));
  const auto bar = ImGui::FindWindowByName("##EditorStatusBar");
  ASSERT_NE(bar, nullptr);
  ImGui::GetIO().AddMousePosEvent(bar->Pos.x + bar->Size.x - 20, bar->Pos.y + bar->Size.y / 2);
  draw();
  ImGui::GetIO().AddMouseButtonEvent(0, true);
  draw();
  ImGui::GetIO().AddMouseButtonEvent(0, false);
  draw();
  EXPECT_TRUE(editor->show_console_window);
  EXPECT_EQ(EditorStatusBarTestAccess::Filters(*editor), (std::array<bool, 3>{false, false, true}));
  EditorStatusBarTestAccess::Clear(*editor);
  draw();
  EXPECT_EQ(EditorStatusBarTestAccess::Counts(*editor), (std::pair<size_t, size_t>{0, 0}));
}

TEST_F(EditorWidgetTest, StatusBarActivityOpensDetails) {
  Application app;
  ApplicationContextScope scope(app);
  const auto editor = app.PushLayer<EditorLayer>("Editor");
  const auto draw = [&] {
    ImGui::NewFrame();
    EditorStatusBarTestAccess::Draw(*editor);
    ImGui::Render();
  };
  draw();
  const auto bar = ImGui::FindWindowByName("##EditorStatusBar");
  ASSERT_NE(bar, nullptr);
  ImGui::ActivateItemByID(bar->GetID("###StatusActivity"));
  draw();
  EXPECT_TRUE(ImGui::IsPopupOpen(bar->GetID("Background activity"), ImGuiPopupFlags_None));
}

TEST_F(EditorWidgetTest, CurveClearEditsTheRuntimeModel) {
  Curve2D curve(0.2f, 0.8f, {0, 0}, {1, 1}, false);
  curve.UnsafeGetValues().insert(curve.UnsafeGetValues().begin() + 1, {0.5f, 0.9f});
  Curve2D cleared = curve;
  cleared.Clear();
  ImVec2 button;
  bool changed = false;
  for (int frame = 0; frame < 4; ++frame) {
    if (frame > 0) {
      ImGui::GetIO().AddMousePosEvent(button.x, button.y);
      if (frame > 1)
        ImGui::GetIO().AddMouseButtonEvent(0, frame == 2);
    }
    Begin();
    button = ImGui::GetCursorScreenPos();
    button.x += 8;
    button.y += 8;
    changed |= editor_widgets::Draw(curve, "Curve", {160, 100});
    End();
  }
  EXPECT_TRUE(changed);
  EXPECT_EQ(curve.UnsafeGetValues(), cleared.UnsafeGetValues());
  YAML::Emitter out;
  out << YAML::BeginMap;
  curve.Save("curve", out);
  out << YAML::EndMap;
  Curve2D restored;
  restored.Load("curve", YAML::Load(out.c_str()));
  EXPECT_EQ(restored.UnsafeGetValues(), cleared.UnsafeGetValues());
}

TEST_F(EditorWidgetTest, ScalarAndVectorControlsPreserveValuesWithoutInput) {
  Plot2D<float> scalar;
  Plot2D<glm::vec2> vector2;
  Plot2D<glm::vec3> vector3;
  SingleDistribution<float> single{2.0f, 0.1f};
  SingleDistribution<glm::vec2> single2{glm::vec2(3), 0.2f};
  SingleDistribution<glm::vec3> single3{glm::vec3(4), 0.3f};
  PlottedDistribution<glm::vec3> distribution;
  for (int frame = 0; frame < 2; ++frame) {
    Begin();
    const auto draw = [](auto& value, const char* label) {
      ImGui::SetNextItemOpen(true, ImGuiCond_Always);
      EXPECT_FALSE(editor_widgets::Draw(value, label));
    };
    draw(scalar, "Scalar");
    draw(vector2, "Vector2");
    draw(vector3, "Vector3");
    draw(single, "Single");
    draw(single2, "Single2");
    draw(single3, "Single3");
    draw(distribution, "Distribution");
    End();
  }
  EXPECT_EQ(scalar.min_value, 0.0f);
  EXPECT_EQ(vector2.max_value, glm::vec2(1));
  EXPECT_EQ(vector3.max_value, glm::vec3(1));
  EXPECT_EQ(single.mean, 2.0f);
  EXPECT_EQ(single2.mean, glm::vec2(3));
  EXPECT_EQ(single3.mean, glm::vec3(4));
}

TEST(ImGuiStartup, DefaultTexturesAndFrameRenderingUseInitializedBackend) {
  Application app;
  ApplicationContextScope scope(app);
  app.PushLayer<WindowLayer>("Window");
  app.PushLayer<RenderLayer>("Rendering");
  app.PushLayer<ImGuiLayer>("GUI");
  ApplicationInitializationSettings settings;
  settings.allow_empty_project = true;
  settings.default_window_size = {320, 240};
  settings.load_project_assets = false;
  settings.load_project_start_scene = false;
  settings.hide_console_window = false;
  settings.redirect_standard_streams_to_console = false;
  settings.graphics_settings.use_ray_tracing = false;
  settings.graphics_settings.use_mesh_shader = false;
  app.Initialize(settings);
  ASSERT_NE(ImGui::GetCurrentContext(), nullptr);
  ASSERT_NE(ImGui::GetIO().BackendRendererUserData, nullptr);
  ASSERT_NE(Resources::GetInstance().GetMissingTexture(), nullptr);
  EXPECT_NE(EditorTextureRegistry::GetTextureId(*Resources::GetInstance().GetMissingTexture()), 0);
  auto scene = AssetManager::CreateTemporaryAsset<Scene>();
  auto camera = scene->GetOrSetPrivateComponent<Camera>(scene->CreateEntity("Main camera")).lock();
  scene->main_camera = camera;
  app.Attach(scene);
  app.Start();
  auto displayed_texture = AssetManager::CreateTemporaryAsset<Texture2D>();
  displayed_texture->SetRgbaChannelData(std::vector<glm::vec4>(16, glm::vec4(1)), {4, 4});
  displayed_texture->UnsafeUploadDataImmediately();
  std::weak_ptr<ImageView> retained_view;
  int displayed_frames = 0;
  app.RegisterUpdateFunction([&] {
    ImGui::Begin("Camera attachments");
    if (displayed_frames == 1)
      camera->Resize({160, 120});
    for (const auto& attachment :
         {camera->GetGBufferBaseColorAoResources(), camera->GetGBufferNormalRoughnessResources(),
          camera->GetGBufferPbrFlagsResources(), camera->GetGBufferEmissiveResources()}) {
      const auto id = EditorTextureRegistry::GetTextureId(attachment);
      EXPECT_NE(id, 0);
      ImGui::Image(id, {24, 24});
    }
    ImGui::End();
    if (displayed_frames++ != 0)
      return;
    ImGui::Begin("Texture lifetime");
    const auto id = EditorTextureRegistry::GetTextureId(*displayed_texture);
    EXPECT_NE(id, 0);
    EXPECT_EQ(EditorTextureRegistry::GetTextureId(*displayed_texture), id);
    ImGui::Image(id, {32, 32});
    auto& storage = displayed_texture->RefTexture2DStorage();
    retained_view = storage.image_view;
    storage.Clear();
    EXPECT_FALSE(retained_view.expired());
    ImGui::End();
  });
  for (int frame = 0; frame < 3; ++frame)
    ASSERT_TRUE(app.Loop());
  Platform::DrainGpuResourceWork();
  EditorTextureRegistry::CollectGarbage();
  EXPECT_TRUE(retained_view.expired());
  {
    OffscreenPreviewSettings preview_settings;
    preview_settings.resolution = {32, 32};
    const auto thumbnail =
        OffscreenPreviewRenderer::RenderMesh(Resources::GetInstance().GetPrimitives().sphere, {}, preview_settings);
    ASSERT_NE(thumbnail, nullptr);
    std::vector<glm::vec4> pixels;
    thumbnail->GetRgbaChannelData(pixels);
    ASSERT_EQ(pixels.size(), 32u * 32u);
    EXPECT_TRUE(std::all_of(pixels.begin(), pixels.end(), [](const auto& pixel) {
      return std::isfinite(pixel.r) && std::isfinite(pixel.g) && std::isfinite(pixel.b) && std::isfinite(pixel.a);
    }));
    EXPECT_TRUE(std::any_of(pixels.begin(), pixels.end(), [](const auto& pixel) {
      return pixel.r > 0.01f || pixel.g > 0.01f || pixel.b > 0.01f;
    }));
  }
  displayed_texture.reset();
  camera.reset();
  scene.reset();
  app.Terminate();
  EXPECT_EQ(ImGui::GetCurrentContext(), nullptr);
}
