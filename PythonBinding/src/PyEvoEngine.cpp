#include "PyEvoEngine.hpp"
#include "EnvironmentalLighting.hpp"
#include "EnvironmentalLightingResolver.hpp"
#include "GeometryStorage.hpp"
#include "ImGuiLayer.hpp"
#include "Lights.hpp"
#include "TextureStorage.hpp"
#include "TransformGraph.hpp"
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

bool IsCurrentSceneCaptureReady() {
  return ProjectManager::IsProjectIdle() && !GeometryStorage::HasPendingUploads() &&
         !TextureStorage::HasPendingUploads();
}
}  // namespace

bool PyEvoEngine::ConfigureCurrentSceneCameraForCapture(const std::string& render_mode, const int samples_per_frame,
                                                         const int bounces) {
  if (samples_per_frame <= 0 || bounces < 0) {
    EVOENGINE_ERROR("Invalid capture camera sample or bounce settings.")
    return false;
  }

  const auto scene = ApplicationContext::Get().GetActiveScene();
  const auto main_camera = scene ? scene->main_camera.Get<Camera>() : nullptr;
  if (!main_camera) {
    EVOENGINE_ERROR("No main camera in scene!")
    return false;
  }

  const auto& render_mode_names = Camera::GetCameraRenderModeNames();
  const auto mode_it = std::find(render_mode_names.begin(), render_mode_names.end(), render_mode);
  if (mode_it == render_mode_names.end()) {
    EVOENGINE_ERROR("Unsupported capture camera render mode: " + render_mode)
    return false;
  }
  const auto requested_mode = static_cast<Camera::CameraRenderMode>(std::distance(render_mode_names.begin(), mode_it));
  const auto resolved_mode = Camera::ResolveCameraRenderMode(requested_mode);
  if (resolved_mode != requested_mode) {
    EVOENGINE_ERROR("Capture camera render mode " + render_mode + " is unavailable; refusing fallback to " +
                    Camera::GetCameraRenderModeName(resolved_mode) + ".")
    return false;
  }

  main_camera->camera_render_mode = requested_mode;
  main_camera->camera_settings.sample_size = samples_per_frame;
  main_camera->camera_settings.bounce = bounces;
  main_camera->camera_settings.auto_spp_enabled = false;
  main_camera->ResetFrameCount();
  return true;
}

bool PyEvoEngine::ConfigureCurrentSceneOutdoorLightingForCapture(
    const glm::vec3& sun_euler_degrees, const float sun_angular_diameter_radians, const float sun_intensity,
    const glm::vec3& sun_color, const float sky_light_intensity, const float ambient_light_intensity,
    const glm::vec3& background_color_linear, const float gamma) {
  const auto scene = ApplicationContext::Get().GetActiveScene();
  const auto main_camera = scene ? scene->main_camera.Get<Camera>() : nullptr;
  if (!scene || !main_camera || !scene->IsEntityValid(main_camera->GetOwner())) {
    EVOENGINE_ERROR("No valid active scene camera for outdoor capture lighting.")
    return false;
  }
  if (sun_angular_diameter_radians < 0.0f || sun_intensity < 0.0f || sky_light_intensity < 0.0f ||
      ambient_light_intensity < 0.0f || gamma <= 0.0f || glm::any(glm::lessThan(sun_color, glm::vec3(0.0f)))) {
    EVOENGINE_ERROR("Invalid outdoor capture lighting parameter.")
    return false;
  }

  auto lighting = scene->environmental_lighting.Get<EnvironmentalLighting>();
  if (!lighting || !lighting->IsTemporary()) {
    lighting = AssetManager::CreateTemporaryAsset<EnvironmentalLighting>();
    scene->environmental_lighting = lighting;
  }
  lighting->indirect_environment_source.kind = EnvironmentalLighting::IndirectEnvironmentSourceKind::EngineDefault;
  lighting->environment_lighting_intensity = sky_light_intensity;
  lighting->diffuse_fallback_intensity = ambient_light_intensity;
  lighting->specular_fallback_intensity = sky_light_intensity;

  auto directional_light_entities = scene->GetPrivateComponentOwnersList<DirectionalLight>();
  if (directional_light_entities.empty()) {
    const auto light_entity = scene->CreateEntity("Capture Directional Light");
    scene->GetOrSetPrivateComponent<DirectionalLight>(light_entity);
    directional_light_entities.emplace_back(light_entity);
  }
  const auto sun_rotation = glm::quat(glm::radians(sun_euler_degrees));
  for (const auto& light_entity : directional_light_entities) {
    const auto directional_light = scene->GetOrSetPrivateComponent<DirectionalLight>(light_entity).lock();
    if (!directional_light) {
      continue;
    }
    directional_light->cast_shadow = true;
    directional_light->diffuse = sun_color;
    directional_light->diffuse_brightness = sun_intensity;
    directional_light->light_size = sun_angular_diameter_radians;
    auto transform = scene->GetDataComponent<GlobalTransform>(light_entity);
    transform.SetValue(transform.GetPosition(), sun_rotation, transform.GetScale());
    scene->SetDataComponent(light_entity, transform);
  }

  main_camera->camera_settings.background_source = CameraSettings::BackgroundSource::ClearColor;
  main_camera->camera_settings.clear_color = glm::vec4(glm::max(background_color_linear, glm::vec3(0.0f)), 1.0f);
  main_camera->camera_settings.background_intensity = 1.0f;
  main_camera->camera_settings.gamma = gamma;
  main_camera->ResetFrameCount();
  TransformGraph::CalculateTransformGraphs(scene, false);
  return !directional_light_entities.empty();
}

bool PyEvoEngine::SetMainCameraLookAt(const glm::vec3& position, const glm::vec3& target, const glm::vec3& up,
                                      const float fov_degrees) {
  const auto scene = ApplicationContext::Get().GetActiveScene();
  const auto main_camera = scene ? scene->main_camera.Get<Camera>() : nullptr;
  if (!main_camera || !scene->IsEntityValid(main_camera->GetOwner())) {
    EVOENGINE_ERROR("No valid main camera in scene!")
    return false;
  }

  const auto target_direction = target - position;
  if (glm::length(target_direction) <= 1e-6f || glm::length(up) <= 1e-6f) {
    EVOENGINE_ERROR("Invalid look-at camera direction or up vector.")
    return false;
  }

  const auto front = glm::normalize(target_direction);
  const auto normalized_up = glm::normalize(up);
  const auto right = glm::cross(front, normalized_up);
  if (glm::length(right) <= 1e-6f) {
    EVOENGINE_ERROR("Look-at camera direction and up vector are parallel.")
    return false;
  }

  auto transform = scene->GetDataComponent<GlobalTransform>(main_camera->GetOwner());
  const auto camera_up = glm::normalize(glm::cross(glm::normalize(right), front));
  transform.SetValue(position, glm::quatLookAt(front, camera_up), glm::vec3(1.0f));
  scene->SetDataComponent(main_camera->GetOwner(), transform);
  main_camera->camera_settings.fov = glm::clamp(fov_degrees, 1.0f, 179.0f) * 2.0f;
  main_camera->ResetFrameCount();
  TransformGraph::CalculateTransformGraphs(scene, false);
  return true;
}

bool PyEvoEngine::WaitForCurrentSceneReady(const int maximum_frames) {
  if (maximum_frames <= 0) {
    EVOENGINE_ERROR("Scene readiness maximum frame count must be positive.")
    return false;
  }
  auto& application = ApplicationContext::Get();
  int frame_count = 0;
  while (!IsCurrentSceneCaptureReady() && frame_count < maximum_frames) {
    if (!application.Loop()) {
      EVOENGINE_ERROR("Application ended before the scene became ready for capture.")
      return false;
    }
    ++frame_count;
    if (frame_count % 25 == 0) {
      const auto snapshot = AssetManager::GetAssetLoadSnapshot();
      EVOENGINE_LOG("Scene readiness progress: frames=" + std::to_string(frame_count) +
                    ", project idle=" + std::to_string(ProjectManager::IsProjectIdle()) +
                    ", geometry pending=" + std::to_string(GeometryStorage::HasPendingUploads()) +
                    ", texture pending=" + std::to_string(TextureStorage::HasPendingUploads()) +
                    ", asset queued=" + std::to_string(snapshot.queued) +
                    ", asset loading CPU=" + std::to_string(snapshot.loading_cpu) +
                    ", asset waiting finalize=" + std::to_string(snapshot.waiting_for_finalize) +
                    ", asset GPU pending=" + std::to_string(snapshot.gpu_pending))
    }
  }
  if (IsCurrentSceneCaptureReady()) {
    EVOENGINE_LOG("Scene ready for capture after " + std::to_string(frame_count) + " frame(s).")
    return true;
  }
  const auto snapshot = AssetManager::GetAssetLoadSnapshot();
  EVOENGINE_ERROR("Scene did not become ready for capture. Frames: " + std::to_string(frame_count) +
                  ", project idle: " + std::to_string(ProjectManager::IsProjectIdle()) +
                  ", geometry pending: " + std::to_string(GeometryStorage::HasPendingUploads()) +
                  ", texture pending: " + std::to_string(TextureStorage::HasPendingUploads()) +
                  ", asset queued: " + std::to_string(snapshot.queued) +
                  ", asset loading CPU: " + std::to_string(snapshot.loading_cpu) +
                  ", asset waiting finalize: " + std::to_string(snapshot.waiting_for_finalize) +
                  ", asset GPU pending: " + std::to_string(snapshot.gpu_pending))
  return false;
}

bool PyEvoEngine::CaptureCurrentScene(const int resolution_x, const int resolution_y,
                                      const std::filesystem::path& output_path, const int warmup_frames,
                                      const bool require_accumulated_frames) {
  if (resolution_x <= 0 || resolution_y <= 0 || warmup_frames < 0) {
    EVOENGINE_ERROR("Invalid capture resolution or frame count!")
    return false;
  }

  constexpr int max_readiness_frames = 300;
  int readiness_frames = 0;
  auto& application = ApplicationContext::Get();
  while (!IsCurrentSceneCaptureReady() && readiness_frames < max_readiness_frames) {
    application.Loop();
    readiness_frames++;
  }
  if (!IsCurrentSceneCaptureReady()) {
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
  if (require_accumulated_frames) {
    const auto target_frame_count = static_cast<uint32_t>(std::max(1, warmup_frames));
    const auto maximum_loop_count = target_frame_count + 600u;
    main_camera->ResetFrameCount();
    uint32_t loop_count = 0;
    while (main_camera->GetFrameCount() < target_frame_count && loop_count < maximum_loop_count) {
      if (!application.Loop()) {
        EVOENGINE_ERROR("Application ended before the capture accumulation target was reached.")
        return false;
      }
      ++loop_count;
    }
    if (main_camera->GetFrameCount() < target_frame_count) {
      EVOENGINE_ERROR(
          "Capture accumulation target was not reached. Requested frames: " + std::to_string(target_frame_count) +
          ", accumulated frames: " + std::to_string(main_camera->GetFrameCount()) + ".")
      return false;
    }
    const auto samples_per_frame = static_cast<uint64_t>(std::max(main_camera->camera_settings.sample_size, 1));
    EVOENGINE_LOG(
        "Capture accumulation: mode=" + std::string(Camera::GetCameraRenderModeName(main_camera->camera_render_mode)) +
        ", frames=" + std::to_string(main_camera->GetFrameCount()) +
        ", samples_per_frame=" + std::to_string(samples_per_frame) +
        ", total_spp=" + std::to_string(main_camera->GetFrameCount() * samples_per_frame))
  } else {
    const auto loop_count = std::max(1, warmup_frames);
    for (int i = 0; i < loop_count; i++) {
      application.Loop();
    }
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
  m.def("ConfigureCurrentSceneCameraForCapture", &ConfigureCurrentSceneCameraForCapture, py::arg("render_mode"),
        py::arg("samples_per_frame"), py::arg("bounces"));
  m.def("ConfigureCurrentSceneOutdoorLightingForCapture", &ConfigureCurrentSceneOutdoorLightingForCapture,
        py::arg("sun_euler_degrees"), py::arg("sun_angular_diameter_radians"), py::arg("sun_intensity"),
        py::arg("sun_color"), py::arg("sky_light_intensity"), py::arg("ambient_light_intensity"),
        py::arg("background_color_linear"), py::arg("gamma"));
  m.def("SetMainCameraLookAt", &SetMainCameraLookAt, py::arg("position"), py::arg("target"), py::arg("up"),
        py::arg("fov_degrees"));
  m.def("WaitForCurrentSceneReady", &WaitForCurrentSceneReady, py::arg("maximum_frames") = 3000);
  m.def("CaptureCurrentScene", &CaptureCurrentScene, py::arg("resolution_x"), py::arg("resolution_y"),
        py::arg("output_path"), py::arg("warmup_frames") = 1, py::arg("require_accumulated_frames") = false);
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
  // Windowless batch jobs only need the start scene and its dependency graph.
  // Loading every asset discovered in a research project makes capture startup
  // scale with unrelated historical data; the native non-editor app uses the
  // same lazy project-asset policy.
  application_info.load_project_assets = false;
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
  if (!settings.runtime.enabled || render_layer->GetDdgiSessionState().pause_updates) {
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
