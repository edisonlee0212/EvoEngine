#include "PyEvoEngine.hpp"
#include "EnvironmentalLightingResolver.hpp"
#include "GeometryStorage.hpp"
#include "ImGuiLayer.hpp"
#include "TextureStorage.hpp"
#ifdef CUDA_MODULE_SERVICE
#  include "RayTracerLayer.hpp"
#endif
using namespace py_evo_engine;
namespace py = pybind11;

PyEvoEngine::PyEvoEngine() = default;

PyEvoEngine::~PyEvoEngine() = default;

PyEvoEngine& PyEvoEngine::GetRuntime() {
  static PyEvoEngine runtime;
  ApplicationContext::Set(&runtime.application);
  return runtime;
}

Application& PyEvoEngine::GetApplication() {
  return application;
}

namespace {
DemoSetup ParseDemoSetupName(const std::string& demo_setup_name) {
  if (demo_setup_name == "Rendering") {
    return DemoSetup::Rendering;
  }
  if (demo_setup_name == "Universe") {
    return DemoSetup::Universe;
  }
  if (demo_setup_name == "Empty") {
    return DemoSetup::Empty;
  }
  EVOENGINE_ERROR("Unsupported demo setup: " + demo_setup_name)
  return DemoSetup::Empty;
}

void EnsureRenderLayer() {
  if (!ApplicationContext::Get().GetLayer<RenderLayer>()) {
    ApplicationContext::Get().PushLayer<RenderLayer>("Render Layer");
  }
}
}  // namespace

bool PyEvoEngine::CaptureCurrentScene(const int resolution_x, const int resolution_y,
                                      const std::filesystem::path& output_path, const int warmup_frames) {
  if (resolution_x <= 0 || resolution_y <= 0) {
    EVOENGINE_ERROR("Resolution error!");
    return false;
  }

  constexpr int max_readiness_frames = 300;
  int readiness_frames = 0;
  auto& application = ApplicationContext::Get();
  const auto is_scene_ready = []() {
    return ProjectManager::IsProjectIdle() && !GeometryStorage::HasPendingUploads() &&
           !TextureStorage::HasPendingUploads();
  };
  while (!is_scene_ready() && readiness_frames < max_readiness_frames) {
    application.Loop();
    readiness_frames++;
  }
  if (!is_scene_ready()) {
    const auto snapshot = AssetManager::GetAssetLoadSnapshot();
    EVOENGINE_ERROR("Scene is not ready for capture! Frames: " + std::to_string(readiness_frames) +
                    ", project idle: " + std::to_string(ProjectManager::IsProjectIdle()) +
                    ", geometry version: " + std::to_string(GeometryStorage::GetVersion()) +
                    ", geometry pending: " + std::to_string(GeometryStorage::HasPendingUploads()) +
                    ", texture pending: " + std::to_string(TextureStorage::HasPendingUploads()) + ", asset queued: " +
                    std::to_string(snapshot.queued) + ", asset loading CPU: " + std::to_string(snapshot.loading_cpu) +
                    ", asset waiting finalize: " + std::to_string(snapshot.waiting_for_finalize) +
                    ", asset GPU pending: " + std::to_string(snapshot.gpu_pending))
    return false;
  }

  const auto scene = application.GetActiveScene();
  if (!scene) {
    EVOENGINE_ERROR("No active scene!");
    return false;
  }
  const auto main_camera = scene->main_camera.Get<Camera>();
  if (!main_camera) {
    EVOENGINE_ERROR("No main camera in scene!");
    return false;
  }
  main_camera->Resize({resolution_x, resolution_y});
  const auto loop_count = std::max(1, warmup_frames);
  for (int i = 0; i < loop_count; i++) {
    application.Loop();
  }
  if (const auto parent_path = output_path.parent_path(); !parent_path.empty()) {
    std::filesystem::create_directories(parent_path);
  }
  main_camera->GetRenderTexture()->StoreToPng(output_path);
  const bool success = std::filesystem::exists(output_path) && std::filesystem::file_size(output_path) > 0;
  if (success) {
    EVOENGINE_LOG("Exported image to " + output_path.string());
  } else {
    EVOENGINE_ERROR("Failed to export image to " + output_path.string())
  }
  return success;
}

Handle PyEvoEngine::CreateRuntimeAsset(const std::string& asset_type) {
  auto& py_evo_engine = GetRuntime();
  const auto new_asset = AssetManager::CreateTemporaryAsset(asset_type);
  const auto ret_val = new_asset->GetHandle();
  py_evo_engine.runtime_assets.insert({ret_val, new_asset});
  return ret_val;
}

void PyEvoEngine::DeleteRuntimeAsset(const Handle& asset_handle) {
  auto& py_evo_engine = GetRuntime();
  if (const auto search = py_evo_engine.runtime_assets.find(asset_handle);
      search != py_evo_engine.runtime_assets.end()) {
    py_evo_engine.runtime_assets.erase(asset_handle);
  } else {
    EVOENGINE_ERROR("DeleteRuntimeAsset failed: Asset not found!")
  }
}

std::shared_ptr<IAsset> PyEvoEngine::GetAsset(const Handle& asset_handle) {
  auto& py_evo_engine = GetRuntime();
  if (const auto search = py_evo_engine.runtime_assets.find(asset_handle);
      search != py_evo_engine.runtime_assets.end()) {
    return search->second;
  }
  if (const auto asset = AssetManager::GetAsset(asset_handle)) {
    return asset;
  }
  EVOENGINE_ERROR("GetRuntimeAsset failed: Asset not found!")
  return {};
}

bool PyEvoEngine::IsRuntimeAsset(const Handle& asset_handle) {
  auto& py_evo_engine = GetRuntime();
  return py_evo_engine.runtime_assets.find(asset_handle) != py_evo_engine.runtime_assets.end();
}

Handle PyEvoEngine::GetAssetHandle(const std::filesystem::path& asset_relative_path) {
  if (!asset_relative_path.is_relative()) {
    EVOENGINE_ERROR("GetAsset failed: Not a relative path!")
    return 0;
  }
  if (!std::filesystem::exists(ProjectManager::GetAssetsFolderPath() / asset_relative_path)) {
    EVOENGINE_ERROR("GetAsset failed: File not exist!")
    return 0;
  }
  return ProjectManager::GetOrCreateAsset(asset_relative_path)->GetHandle();
}
Handle PyEvoEngine::ImportRuntimeAsset(const std::string& asset_type,
                                       const std::filesystem::path& asset_absolute_path) {
  auto& py_evo_engine = GetRuntime();

  if (!asset_absolute_path.is_absolute()) {
    EVOENGINE_ERROR("ImportRuntimeAsset failed: Not a absolute path!")
    return 0;
  }
  if (ProjectManager::IsInAssetsFolder(asset_absolute_path)) {
    EVOENGINE_ERROR("ImportRuntimeAsset failed: File is inside asset folder!")
    return 0;
  }
  if (!std::filesystem::exists(asset_absolute_path)) {
    EVOENGINE_ERROR("ImportRuntimeAsset failed: File not exist!")
    return 0;
  }
  const auto new_asset = AssetManager::CreateTemporaryAsset(asset_type);
  new_asset->Import(asset_absolute_path);
  const auto ret_val = new_asset->GetHandle();
  py_evo_engine.runtime_assets.insert({ret_val, new_asset});
  return ret_val;
}

bool PyEvoEngine::ExportAsset(const Handle& asset_handle, const std::filesystem::path& asset_absolute_path) {
  if (!asset_absolute_path.is_absolute()) {
    EVOENGINE_ERROR("ImportRuntimeAsset failed: Not a absolute path!")
    return false;
  }
  if (ProjectManager::IsInAssetsFolder(asset_absolute_path)) {
    EVOENGINE_ERROR("ImportRuntimeAsset failed: File is inside asset folder!")
    return false;
  }
  const auto asset = GetAsset(asset_handle);
  return asset->Export(asset_absolute_path);
}

bool PyEvoEngine::AssetSave(const Handle& asset_handle) {
  if (IsRuntimeAsset(asset_handle)) {
    EVOENGINE_ERROR("AssetSave failed: asset is runtime asset!")
    return false;
  }
  const auto asset = AssetManager::GetAsset(asset_handle);
  if (!asset) {
    EVOENGINE_ERROR("AssetSave failed: asset not found!")
    return false;
  }
  return asset->Save();
}

bool PyEvoEngine::AssetLoad(const Handle& asset_handle) {
  if (IsRuntimeAsset(asset_handle)) {
    EVOENGINE_ERROR("AssetLoad failed: asset is runtime asset!")
    return false;
  }
  const auto asset = AssetManager::GetAsset(asset_handle);
  if (!asset) {
    EVOENGINE_ERROR("AssetLoad failed: asset not found!")
    return false;
  }
  return asset->Load();
}

void PyEvoEngine::Initialize(pybind11::module& m) {
  auto& py_evo_engine = GetRuntime();
  py_evo_engine.runtime_assets.clear();

  m.def("PushRenderLayer", &PushRenderLayer);
  m.def("PushWindowLayer", &PushWindowLayer);
  m.def("PushEditorLayer", &PushEditorLayer);
  m.def("PushRayTracerLayer", &PushRayTracerLayer);

  m.def("RunWindowless", &RunWindowless);
  m.def("RunDemoWindowless", &RunDemoWindowless, py::arg("demo_setup_name"), py::arg("resource_folder_path"),
        py::arg("clear_generated_project_files") = true);
  m.def("CaptureCurrentScene", &CaptureCurrentScene, py::arg("resolution_x"), py::arg("resolution_y"),
        py::arg("output_path"), py::arg("warmup_frames") = 1);
  m.def("IsCurrentSceneDdgiEnabled", &IsCurrentSceneDdgiEnabled);
  m.def("Run", &Run);
  m.def("RunWithScene", &RunWithScene);
  m.def("Loop", &Loop);
  m.def("Terminate", &Terminate);

  py::class_<Handle>(m, "Handle", py::module_local()).def(py::init<>()).def("GetValue", &Handle::GetValue);
  m.def("CreateRuntimeAsset", &CreateRuntimeAsset);
  m.def("DeleteRuntimeAsset", &DeleteRuntimeAsset);
  m.def("GetAssetHandle", &GetAssetHandle);
  m.def("ImportRuntimeAsset", &ImportRuntimeAsset);
  m.def("ExportAsset", &ExportAsset);
  m.def("AssetSave", &AssetSave);
  m.def("AssetLoad", &AssetLoad);

  py::class_<glm::vec2>(m, "Vec2", py::module_local())
      .def(py::init<>())
      .def_readwrite("x", &glm::vec2::x)
      .def_readwrite("y", &glm::vec2::y);

  py::class_<glm::vec3>(m, "Vec3", py::module_local())
      .def(py::init<>())
      .def_readwrite("x", &glm::vec3::x)
      .def_readwrite("y", &glm::vec3::y)
      .def_readwrite("z", &glm::vec3::z);

  py::class_<glm::vec4>(m, "Vec4", py::module_local())
      .def(py::init<>())
      .def_readwrite("x", &glm::vec4::x)
      .def_readwrite("y", &glm::vec4::y)
      .def_readwrite("z", &glm::vec4::z)
      .def_readwrite("w", &glm::vec4::w);

  py::class_<glm::uvec2>(m, "UVec2", py::module_local())
      .def(py::init<>())
      .def_readwrite("x", &glm::uvec2::x)
      .def_readwrite("y", &glm::uvec2::y);

  py::class_<glm::uvec3>(m, "UVec3", py::module_local())
      .def(py::init<>())
      .def_readwrite("x", &glm::uvec3::x)
      .def_readwrite("y", &glm::uvec3::y)
      .def_readwrite("z", &glm::uvec3::z);

  py::class_<glm::uvec4>(m, "UVec4", py::module_local())
      .def(py::init<>())
      .def_readwrite("x", &glm::uvec4::x)
      .def_readwrite("y", &glm::uvec4::y)
      .def_readwrite("z", &glm::uvec4::z)
      .def_readwrite("w", &glm::uvec4::w);

  py::class_<glm::ivec2>(m, "IVec2", py::module_local())
      .def(py::init<>())
      .def_readwrite("x", &glm::ivec2::x)
      .def_readwrite("y", &glm::ivec2::y);

  py::class_<glm::ivec3>(m, "IVec3", py::module_local())
      .def(py::init<>())
      .def_readwrite("x", &glm::ivec3::x)
      .def_readwrite("y", &glm::ivec3::y)
      .def_readwrite("z", &glm::ivec3::z);

  py::class_<glm::ivec4>(m, "IVec4", py::module_local())
      .def(py::init<>())
      .def_readwrite("x", &glm::ivec4::x)
      .def_readwrite("y", &glm::ivec4::y)
      .def_readwrite("z", &glm::ivec4::z)
      .def_readwrite("w", &glm::ivec4::w);

  py::class_<Entity>(m, "Entity", py::module_local())
      .def("GetIndex", &Entity::GetIndex)
      .def("GetVersion", &Entity::GetVersion);

  m.def("CreateEntity", &CreateEntity);
  m.def("DeleteEntity", &DeleteEntity);
  m.def("IsEntityValid", &IsEntityValid);
}
void PyEvoEngine::PushRenderLayer() {
  EnsureRenderLayer();
}
void PyEvoEngine::PushWindowLayer() {
  if (!ApplicationContext::Get().GetLayer<WindowLayer>()) {
    ApplicationContext::Get().PushLayer<WindowLayer>("Window Layer");
  }
}
void PyEvoEngine::PushEditorLayer() {
  if (!ApplicationContext::Get().GetLayer<ImGuiLayer>()) {
    ApplicationContext::Get().PushLayer<ImGuiLayer>("ImGui Layer");
  }
  if (!ApplicationContext::Get().GetLayer<EditorLayer>()) {
    ApplicationContext::Get().PushLayer<EditorLayer>("Editor Layer");
  }
}
void PyEvoEngine::PushRayTracerLayer() {
#ifdef CUDA_MODULE_SERVICE
  ApplicationContext::Get().PushLayer<RayTracerLayer>("Ray Tracer Layer");
#endif
}

bool PyEvoEngine::RunWindowless(const std::filesystem::path& project_path) {
  if (std::filesystem::path(project_path).extension().string() != ".eveproj") {
    EVOENGINE_ERROR("Project path doesn't point to a EvoEngine project!");
    return false;
  }
  EnsureRenderLayer();
  ApplicationInitializationSettings application_info{};
  application_info.project_path = project_path;
  ApplicationContext::Get().Initialize(application_info);
  ApplicationContext::Get().Start();
  return true;
}

bool PyEvoEngine::RunDemoWindowless(const std::string& demo_setup_name,
                                    const std::filesystem::path& resource_folder_path,
                                    const bool clear_generated_project_files) {
  const auto demo_setup = ParseDemoSetupName(demo_setup_name);
  if (demo_setup == DemoSetup::Empty && demo_setup_name != "Empty") {
    return false;
  }

  EnsureRenderLayer();
  ApplicationInitializationSettings application_info{};
  SetupDemoScene(demo_setup, application_info, resource_folder_path, clear_generated_project_files);
  if (application_info.project_path.empty()) {
    EVOENGINE_ERROR("Demo setup did not provide a project path!");
    return false;
  }
  ApplicationContext::Get().Initialize(application_info);
  ApplicationContext::Get().Start();
  return true;
}

bool PyEvoEngine::IsCurrentSceneDdgiEnabled() {
  auto& application = ApplicationContext::Get();
  const auto render_layer = application.GetLayer<RenderLayer>();
  const auto scene = application.GetActiveScene();
  if (!render_layer || !scene || !render_layer->enable_indirect_rendering) {
    return false;
  }

  const auto resolved_lighting = ResolveEnvironmentalLighting(scene);
  const auto& settings = resolved_lighting.ddgi_settings;
  if (!settings.runtime.enabled || settings.runtime.pause_updates) {
    return false;
  }

  return !resolved_lighting.ddgi_volumes.empty();
}

void PyEvoEngine::Run(const std::filesystem::path& project_path) {
  if (std::filesystem::path(project_path).extension().string() != ".eveproj") {
    EVOENGINE_ERROR("Project path doesn't point to a EvoEngine project!");
    return;
  }
  ApplicationInitializationSettings application_info{};
  application_info.project_path = project_path;
  ApplicationContext::Get().Initialize(application_info);
  ApplicationContext::Get().Start();
}

void PyEvoEngine::RunWithScene(const std::filesystem::path& project_path,
                               const std::filesystem::path& project_relative_path) {
  if (std::filesystem::path(project_path).extension().string() != ".eveproj") {
    EVOENGINE_ERROR("Project path doesn't point to a EvoEngine project!");
    return;
  }
  ApplicationInitializationSettings application_info{};
  application_info.project_path = project_path;
  ApplicationContext::Get().Initialize(application_info);
  const auto new_scene = std::dynamic_pointer_cast<Scene>(ProjectManager::GetOrCreateAsset(project_relative_path));
  ProjectManager::SetStartScene(new_scene);
  ApplicationContext::Get().Start();
}

bool PyEvoEngine::Loop() {
  return ApplicationContext::Get().Loop();
}
void PyEvoEngine::Terminate() {
  ApplicationContext::Get().Terminate();
}
Entity PyEvoEngine::CreateEntity(const std::string& name) {
  const auto scene = ApplicationContext::Get().GetActiveScene();
  return scene->CreateEntity(name);
}
void PyEvoEngine::DeleteEntity(const Entity& entity) {
  const auto scene = ApplicationContext::Get().GetActiveScene();
  return scene->DeleteEntity(entity);
}
bool PyEvoEngine::IsEntityValid(const Entity& entity) {
  const auto scene = ApplicationContext::Get().GetActiveScene();
  return scene->IsEntityValid(entity);
}
