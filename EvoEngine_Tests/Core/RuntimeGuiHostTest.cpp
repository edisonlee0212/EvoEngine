#include <gtest/gtest.h>
#include <imgui.h>
#include "Application.hpp"
#include "AssetManager.hpp"
#include "Camera.hpp"
#include "ImGuiLayer.hpp"
#include "Prefab.hpp"
#include "ProjectManager.hpp"
#include "RenderLayer.hpp"
#include "RuntimeDebugGui.hpp"
#include "RuntimeGui.hpp"
#include "RuntimeGuiLayer.hpp"
#include "RuntimeGuiProof.hpp"
#include "Scene.hpp"
#include "WindowLayer.hpp"

using namespace evo_engine;

TEST(RuntimeGuiPersistence, SceneAndPrefabRoundTripRelinkCameraAndKeepIndependentLayouts) {
  Application app;
  ApplicationContextScope scope(app);
  ApplicationInitializationSettings settings;
  settings.application_mode = ApplicationMode::Headless;
  settings.allow_empty_project = true;
  settings.load_default_resources = settings.load_project_assets = settings.load_project_start_scene = false;
  app.Initialize(settings);
  const auto directory =
      std::filesystem::temp_directory_path() / ("EvoEngineRuntimeGui-" + std::to_string(Handle().GetValue()));
  std::filesystem::create_directories(directory);
  auto scene = AssetManager::CreateTemporaryAsset<Scene>();
  app.Attach(scene);
  const auto entity = scene->CreateEntity("GUI camera prefab");
  const auto entity_handle = scene->GetEntityHandle(entity);
  auto camera = scene->GetOrSetPrivateComponent<Camera>(entity).lock();
  auto gui = scene->GetOrSetPrivateComponent<RuntimeGui>(entity).lock();
  auto asset = AssetManager::CreateTemporaryAsset<RuntimeDebugGui>();
  gui->camera = camera;
  gui->draw_order = 7;
  ASSERT_TRUE(gui->AddGuiAsset(asset));
  const auto layout = "[Window][" + std::to_string(asset->GetHandle()) +
                      "/Debug]\nPos=25,30\nSize=280,240\nCollapsed=0\n\n[Window][" +
                      std::to_string(asset->GetHandle()) + "/Debug/Child/Images]\nIsChild=1\nSize=250,80\n";
  gui->SetLayout(layout);
  scene->main_camera = camera;
  ASSERT_TRUE(scene->Export(directory / "Scene.evescene"));
  auto restored = AssetManager::CreateTemporaryAsset<Scene>();
  ASSERT_TRUE(restored->Import(directory / "Scene.evescene"));
  const auto restored_entity = restored->GetEntity(entity_handle);
  auto restored_gui = restored->GetOrSetPrivateComponent<RuntimeGui>(restored_entity).lock();
  EXPECT_EQ(restored_gui->GetLayout(), layout);
  EXPECT_EQ(restored_gui->draw_order, 7);
  EXPECT_EQ(restored_gui->camera.Get<Camera>(), restored->GetOrSetPrivateComponent<Camera>(restored_entity).lock());
  EXPECT_EQ(restored_gui->camera.Get<Camera>(), restored->main_camera.Get<Camera>());
  ASSERT_EQ(restored_gui->GetGuiAssets().size(), 1);
  EXPECT_EQ(restored_gui->GetGuiAssets().front().GetAssetHandle(), asset->GetHandle());

  auto prefab = AssetManager::CreateTemporaryAsset<Prefab>();
  prefab->FromEntity(entity);
  ASSERT_TRUE(prefab->Export(directory / "Gui.eveprefab"));
  auto loaded_prefab = AssetManager::CreateTemporaryAsset<Prefab>();
  ASSERT_TRUE(loaded_prefab->Import(directory / "Gui.eveprefab"));
  const auto first = loaded_prefab->ToEntity(scene);
  const auto second = loaded_prefab->ToEntity(scene);
  auto first_gui = scene->GetOrSetPrivateComponent<RuntimeGui>(first).lock();
  auto second_gui = scene->GetOrSetPrivateComponent<RuntimeGui>(second).lock();
  EXPECT_NE(first_gui->GetHandle(), second_gui->GetHandle());
  EXPECT_EQ(first_gui->GetLayout(), layout);
  EXPECT_EQ(second_gui->GetLayout(), layout);
  EXPECT_EQ(first_gui->camera.Get<Camera>(), scene->GetOrSetPrivateComponent<Camera>(first).lock());
  EXPECT_EQ(second_gui->camera.Get<Camera>(), scene->GetOrSetPrivateComponent<Camera>(second).lock());
  ASSERT_EQ(first_gui->GetGuiAssets().size(), 1);
  ASSERT_EQ(second_gui->GetGuiAssets().size(), 1);
  EXPECT_EQ(first_gui->GetGuiAssets().front().GetAssetHandle(), second_gui->GetGuiAssets().front().GetAssetHandle());
  first_gui->SetLayout("independent edited layout");
  EXPECT_EQ(second_gui->GetLayout(), layout);
  EXPECT_EQ(gui->GetLayout(), layout);
  second_gui.reset();
  first_gui.reset();
  loaded_prefab.reset();
  prefab.reset();
  restored_gui.reset();
  restored.reset();
  asset.reset();
  gui.reset();
  camera.reset();
  scene.reset();
  app.Terminate();
  std::filesystem::remove_all(directory);
}

namespace {
class RuntimeInputObserver : public ILayer {
 public:
  int presses = 0;
  bool OnInputEvent(const Input::InputEvent& event) override {
    if (event.key == GLFW_KEY_W && event.key_action == Input::KeyActionType::Press)
      ++presses;
    return ILayer::OnInputEvent(event);
  }
};
}  // namespace

TEST(RuntimeGuiHost, PlayerRendersWithoutEditorLayers) {
  if (!std::getenv("EVOENGINE_RUNTIME_GUI_PROOF"))
    GTEST_SKIP() << "Set EVOENGINE_RUNTIME_GUI_PROOF=1 to run the graphics fixture.";
  Application app;
  ApplicationContextScope scope(app);
  app.PushLayer<RenderLayer>("Rendering");
  app.PushLayer<WindowLayer>("Window");
  app.PushLayer<ImGuiLayer>("GUI");
  auto observer = app.PushLayer<RuntimeInputObserver>("Gameplay observer");
  ApplicationInitializationSettings settings;
  settings.application_mode = ApplicationMode::Player;
  settings.allow_empty_project = true;
  settings.default_window_size = {640, 480};
  settings.load_project_assets = settings.load_project_start_scene = false;
  settings.redirect_standard_streams_to_console = false;
  settings.graphics_settings.use_ray_tracing = settings.graphics_settings.use_mesh_shader = false;
  app.Initialize(settings);
  auto scene = AssetManager::CreateTemporaryAsset<Scene>();
  auto camera = scene->GetOrSetPrivateComponent<Camera>(scene->CreateEntity("Main camera")).lock();
  scene->main_camera = camera;
  app.Attach(scene);
  app.Start(false);
  for (int frame = 0; frame < 12; ++frame)
    ASSERT_TRUE(app.Loop());
  EXPECT_NE(app.GetLayer<ImGuiLayer>()->GetRuntimeGuiProof()->GetOverlay(), nullptr);
  EXPECT_TRUE(camera->Rendered());
  EXPECT_EQ(app.GetLayers().size(), 4);
  const auto window = app.GetLayer<WindowLayer>()->GetGlfwWindow();
  glfwIconifyWindow(window);
  for (int frame = 0; frame < 3; ++frame)
    ASSERT_TRUE(app.Loop());
  glfwRestoreWindow(window);
  for (int frame = 0; frame < 8; ++frame)
    ASSERT_TRUE(app.Loop());
  ImGui::SetWindowFocus(nullptr);
  const auto key_callback = glfwSetKeyCallback(window, nullptr);
  glfwSetKeyCallback(window, key_callback);
  ASSERT_NE(key_callback, nullptr);
  key_callback(window, GLFW_KEY_W, 0, GLFW_PRESS, 0);
  ASSERT_TRUE(app.Loop());
  EXPECT_EQ(observer->presses, 1);
  EXPECT_EQ(scene->GetKey(GLFW_KEY_W), Input::KeyActionType::Press);
  key_callback(window, GLFW_KEY_W, 0, GLFW_RELEASE, 0);
  ASSERT_TRUE(app.Loop());
  EXPECT_EQ(scene->GetKey(GLFW_KEY_W), Input::KeyActionType::Release);
  observer.reset();
  camera.reset();
  scene.reset();
  app.Terminate();
  EXPECT_EQ(ImGui::GetCurrentContext(), nullptr);
}

namespace {
class CountingRuntimeGui : public IRuntimeGui {
 public:
  std::vector<uint64_t> calls;
  std::function<void()> after_draw;
  void OnGui(RuntimeGuiContext& context) override {
    const auto component = context.GetComponent();
    EXPECT_NE(component, nullptr);
    EXPECT_EQ(context.GetScene(), component->GetScene());
    calls.push_back(component->GetScene()->GetEntityHandle(component->GetOwner()).GetValue());
    if (context.BeginWindow("Same title")) {
      context.BeginChild("Resizable", {120, 70}, ImGuiChildFlags_ResizeY);
      ImGui::TextUnformatted("Shared asset, independent child layout");
      context.EndChild();
    }
    context.EndWindow();
    if (const auto callback = after_draw)
      callback();
  }
};
}  // namespace

TEST(RuntimeGuiHost, ComponentsTargetCameraAndShareAssetsWithIndependentLayouts) {
  if (!std::getenv("EVOENGINE_RUNTIME_GUI_PROOF"))
    GTEST_SKIP();
  Application app;
  ApplicationContextScope scope(app);
  app.RegisterAsset<CountingRuntimeGui>("CountingRuntimeGui", {".testgui"});
  app.PushLayer<RenderLayer>("Rendering");
  app.PushLayer<WindowLayer>("Window");
  app.PushLayer<ImGuiLayer>("GUI");
  app.PushLayer<RuntimeGuiLayer>("Runtime GUI");
  ApplicationInitializationSettings settings;
  settings.application_mode = ApplicationMode::Player;
  settings.allow_empty_project = true;
  settings.default_window_size = {640, 480};
  settings.load_project_assets = settings.load_project_start_scene = false;
  settings.redirect_standard_streams_to_console = false;
  settings.graphics_settings.use_ray_tracing = settings.graphics_settings.use_mesh_shader = false;
  app.Initialize(settings);
  auto scene = AssetManager::CreateTemporaryAsset<Scene>();
  auto camera = scene->GetOrSetPrivateComponent<Camera>(scene->CreateEntity("Main camera")).lock();
  scene->main_camera = camera;
  auto asset = AssetManager::CreateTemporaryAsset<CountingRuntimeGui>();
  auto example = AssetManager::CreateTemporaryAsset<RuntimeDebugGui>();
  auto first = scene->GetOrSetPrivateComponent<RuntimeGui>(scene->CreateEntity("First GUI")).lock();
  auto second = scene->GetOrSetPrivateComponent<RuntimeGui>(scene->CreateEntity("Second GUI")).lock();
  first->camera = camera;
  second->camera = camera;
  first->draw_order = 10;
  ASSERT_TRUE(first->AddGuiAsset(asset));
  ASSERT_FALSE(first->AddGuiAsset(asset));
  ASSERT_TRUE(second->AddGuiAsset(asset));
  ASSERT_TRUE(first->AddGuiAsset(example));
  ASSERT_TRUE(second->AddGuiAsset(example));
  std::vector<AssetRef> dependencies;
  first->CollectAssetRef(dependencies);
  EXPECT_EQ(dependencies.size(), 2);
  YAML::Emitter serialized;
  serialized << YAML::BeginMap;
  Serialization::SerializeObject(serialized, *first);
  serialized << YAML::EndMap;
  const auto serialized_gui = YAML::Load(serialized.c_str());
  ASSERT_TRUE(serialized_gui["gui_assets"].IsSequence());
  ASSERT_EQ(serialized_gui["gui_assets"].size(), 2);
  first->camera.Clear();
  first->RemoveGuiAsset(0);
  first->RemoveGuiAsset(0);
  Serialization::DeserializeObject(serialized_gui, *first);
  EXPECT_EQ(first->camera.Get<Camera>(), camera);
  EXPECT_EQ(first->GetGuiAssets().size(), 2);
  ProjectManager::SetStartScene(scene);
  app.Attach(scene);
  app.Start(false);
  app.Play();
  auto playing = app.GetActiveScene();
  auto a = playing->GetOrSetPrivateComponent<RuntimeGui>(first->GetOwner()).lock();
  auto b = playing->GetOrSetPrivateComponent<RuntimeGui>(second->GetOwner()).lock();
  EXPECT_NE(a, first);
  EXPECT_EQ(a->camera.Get<Camera>(), playing->main_camera.Get<Camera>());
  for (int frame = 0; frame < 4; ++frame) {
    asset->calls.clear();
    ASSERT_TRUE(app.Loop());
    ASSERT_EQ(asset->calls.size(), 2);
    EXPECT_EQ(asset->calls[0], playing->GetEntityHandle(b->GetOwner()).GetValue());
    EXPECT_EQ(asset->calls[1], playing->GetEntityHandle(a->GetOwner()).GetValue());
  }
  EXPECT_NE(a->GetLayout().find("IsChild=1"), std::string::npos);
  EXPECT_EQ(first->GetLayout(), "");  // Runtime never dirties the exported source scene.
  a->SetEnabled(false);
  asset->calls.clear();
  ASSERT_TRUE(app.Loop());
  ASSERT_EQ(asset->calls.size(), 1);
  b->camera.Clear();
  asset->calls.clear();
  ASSERT_TRUE(app.Loop());
  EXPECT_TRUE(asset->calls.empty());
  a->SetEnabled(true);
  app.Pause();
  asset->calls.clear();
  ASSERT_TRUE(app.Loop());
  EXPECT_EQ(asset->calls.size(), 1);
  playing->main_camera.Clear();
  asset->calls.clear();
  ASSERT_TRUE(app.Loop());
  EXPECT_TRUE(asset->calls.empty());
  playing->main_camera = a->camera;
  b->camera = a->camera;
  asset->after_draw = [&] {
    app.Attach(playing);
  };
  asset->calls.clear();
  ASSERT_TRUE(app.Loop());
  EXPECT_EQ(asset->calls.size(), 1);
  asset->after_draw = {};
  a.reset();
  b.reset();
  playing.reset();
  first.reset();
  second.reset();
  asset.reset();
  example.reset();
  camera.reset();
  scene.reset();
  app.Terminate();
  EXPECT_EQ(ImGui::GetCurrentContext(), nullptr);
}
