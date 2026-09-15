#include <gtest/gtest.h>
#include "Application.hpp"
#include "AssetManager.hpp"
#include "Camera.hpp"
#include "CurveEditors.hpp"
#include "EditorLayer.hpp"
#include "EditorTextureRegistry.hpp"
#include "EvoEngine_SDK_PCH.hpp"
#include "ImGuiLayer.hpp"
#include "MeshRenderer.hpp"
#include "OffscreenPreviewRenderer.hpp"
#include "RenderLayer.hpp"
#include "Resources.hpp"
#include "Scene.hpp"
#include "Texture2D.hpp"
#include "TextureStorage.hpp"
#include "WindowLayer.hpp"

using namespace evo_engine;

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
