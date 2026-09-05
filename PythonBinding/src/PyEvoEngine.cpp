#include "PyEvoEngine.hpp"
#include "Cubemap.hpp"
#include "EnvironmentalLightingResolver.hpp"
#include "GeometryStorage.hpp"
#include "ImGuiLayer.hpp"
#include "Platform.hpp"
#include "Profiler.hpp"
#include "SdfgiCapabilities.hpp"
#include "Texture2D.hpp"
#include "TextureStorage.hpp"
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
std::string FormatRevisions(const std::vector<uint64_t>& revisions) {
  std::string result;
  for (const auto revision : revisions) {
    if (!result.empty()) {
      result += ',';
    }
    result += std::to_string(revision);
  }
  return result;
}

void LogSampledTextureDiagnostics(const std::shared_ptr<RenderLayer>& render_layer) {
  const auto texture_2d = TextureStorage::GetTexture2DArrayDiagnostics();
  const auto cubemap = TextureStorage::GetCubemapArrayDiagnostics();
  const auto frame_count = std::max(Platform::GetFrameCount(), 1u);
  const auto log_array = [frame_count](const char* type, const SampledTextureArrayDiagnostics& diagnostics) {
    const auto rebuilds_per_frame = static_cast<double>(diagnostics.full_rebuilds) / frame_count;
    const auto descriptors_per_rebuild =
        diagnostics.full_rebuilds == 0
            ? 0.0
            : static_cast<double>(diagnostics.descriptors_written) / static_cast<double>(diagnostics.full_rebuilds);
    EVOENGINE_LOG(
        "EVOENGINE_SAMPLED_TEXTURE_ARRAY type=" + std::string(type) +
        " capacity=" + std::to_string(diagnostics.capacity) + " occupancy=" + std::to_string(diagnostics.occupancy) +
        " high_water=" + std::to_string(diagnostics.high_water_mark) + " pending=" +
        std::to_string(diagnostics.pending_count) + " retiring=" + std::to_string(diagnostics.retiring_count) +
        " reusable=" + std::to_string(diagnostics.reusable_count) +
        " descriptor_revision=" + std::to_string(diagnostics.descriptor_revision) + " registration_revision=" +
        std::to_string(diagnostics.registration_revision) + " rebuilds=" + std::to_string(diagnostics.full_rebuilds) +
        " rebuilds_per_frame=" + std::to_string(rebuilds_per_frame) + " descriptors_per_rebuild=" +
        std::to_string(descriptors_per_rebuild) + " overflow_attempts=" + std::to_string(diagnostics.overflow_attempts))
  };
  log_array("2d", texture_2d);
  log_array("cubemap", cubemap);
  EVOENGINE_LOG("EVOENGINE_SAMPLED_TEXTURE_MIRRORS type=2d applied_revisions=" +
                FormatRevisions(render_layer->GetPerFrameTexture2DAppliedRevisions()))
  EVOENGINE_LOG("EVOENGINE_SAMPLED_TEXTURE_MIRRORS type=cubemap applied_revisions=" +
                FormatRevisions(render_layer->GetPerFrameCubemapAppliedRevisions()))

  double command_recording_cpu_ms = 0.0;
  const auto cpu_frame = Profiler::GetInstance().GetLatestFrameStatsSnapshot();
  for (const auto& event : cpu_frame.named_event_totals) {
    if (event.name == "RenderLayer::RenderAll") {
      command_recording_cpu_ms = event.total_ms;
      break;
    }
  }
  double gpu_frame_ms = 0.0;
  const auto gpu_frames = Platform::GetGpuTimestampFrameHistory();
  const auto gpu_frame = std::find_if(gpu_frames.rbegin(), gpu_frames.rend(), [](const auto& frame) {
    return frame.results_available;
  });
  if (gpu_frame != gpu_frames.rend()) {
    gpu_frame_ms = gpu_frame->span_milliseconds;
  }
  size_t draw_calls = 0;
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  const auto& platform = Platform::GetInstance();
  if (current_frame_index < platform.render_pass_draw_stats.size()) {
    for (const auto& stats : platform.render_pass_draw_stats[current_frame_index]) {
      draw_calls += stats.TotalDrawCalls();
    }
  }
  const auto descriptor_update_cpu_ms =
      static_cast<double>(texture_2d.descriptor_update_cpu_nanoseconds + cubemap.descriptor_update_cpu_nanoseconds) /
      1.0e6;
  const auto descriptor_metadata_bytes =
      (texture_2d.descriptor_metadata_bytes_per_mirror + cubemap.descriptor_metadata_bytes_per_mirror) *
      render_layer->GetPerFrameTexture2DAppliedRevisions().size();
  EVOENGINE_LOG("EVOENGINE_BINDLESS_PERFORMANCE frames=" + std::to_string(frame_count) +
                " descriptor_update_cpu_ms=" + std::to_string(descriptor_update_cpu_ms) +
                " command_recording_cpu_ms=" + std::to_string(command_recording_cpu_ms) +
                " material_fixed_descriptor_binds=0 draw_calls=" + std::to_string(draw_calls) +
                " gpu_frame_ms=" + std::to_string(gpu_frame_ms) +
                " descriptor_metadata_bytes=" + std::to_string(descriptor_metadata_bytes))
}

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

bool PyEvoEngine::ConfigureSecondarySceneCameraForCapture(const int resolution_x, const int resolution_y) {
  if (resolution_x <= 0 || resolution_y <= 0) {
    EVOENGINE_ERROR("Invalid secondary capture camera resolution.")
    return false;
  }
  const auto scene = ApplicationContext::Get().GetActiveScene();
  const auto main_camera = scene ? scene->main_camera.Get<Camera>() : nullptr;
  if (!main_camera || !scene->IsEntityValid(main_camera->GetOwner())) {
    EVOENGINE_ERROR("No main camera in scene!")
    return false;
  }

  const auto entity = scene->CreateEntity("Secondary Capture Camera");
  scene->SetDataComponent(entity, scene->GetDataComponent<Transform>(main_camera->GetOwner()));
  scene->SetDataComponent(entity, scene->GetDataComponent<GlobalTransform>(main_camera->GetOwner()));
  const auto camera = scene->GetOrSetPrivateComponent<Camera>(entity).lock();
  if (!camera) {
    EVOENGINE_ERROR("Failed to create secondary capture camera.")
    return false;
  }
  camera->camera_render_mode = main_camera->camera_render_mode;
  camera->camera_settings = main_camera->camera_settings;
  camera->skybox = main_camera->skybox;
  camera->background_environment = main_camera->background_environment;
  camera->post_processing_stack_ref = main_camera->post_processing_stack_ref;
  camera->Resize({resolution_x, resolution_y});
  camera->SetRequireRendering(true);
  GetRuntime().capture_secondary_camera = camera;
  return true;
}

bool PyEvoEngine::ConfigureRasterPathForCapture(const bool meshlet_enabled, const bool indirect_enabled) {
  const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>();
  if (!render_layer) {
    EVOENGINE_ERROR("Raster capture requires RenderLayer.")
    return false;
  }
  if (meshlet_enabled && !Platform::MeshShaderEnabled()) {
    EVOENGINE_ERROR("Raster capture requested meshlets, but the mesh-shader path is unavailable.")
    return false;
  }
  render_layer->enable_meshlet = meshlet_enabled;
  render_layer->enable_indirect_rendering = indirect_enabled;
  return true;
}

bool PyEvoEngine::CaptureCurrentScene(const int resolution_x, const int resolution_y,
                                      const std::filesystem::path& output_path, const int warmup_frames,
                                      const bool require_accumulated_frames,
                                      const bool require_stable_texture_registrations) {
  if (resolution_x <= 0 || resolution_y <= 0 || warmup_frames < 0) {
    EVOENGINE_ERROR("Invalid capture resolution or frame count!")
    return false;
  }

  constexpr int max_readiness_frames = 300;
  int readiness_frames = 0;
  auto& application = ApplicationContext::Get();
  const auto loop = [&]() {
    if (const auto secondary_camera = GetRuntime().capture_secondary_camera.lock()) {
      secondary_camera->SetRequireRendering(true);
    }
    return application.Loop();
  };
  const auto is_scene_ready = []() {
    return ProjectManager::IsProjectIdle() && !GeometryStorage::HasPendingUploads() &&
           !TextureStorage::HasPendingUploads();
  };
  while (!is_scene_ready() && readiness_frames < max_readiness_frames) {
    loop();
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
  uint64_t texture_2d_registration_revision = 0;
  uint64_t cubemap_registration_revision = 0;
  if (require_stable_texture_registrations) {
    Platform::SetGpuTimestampCaptureEnabled(true);
    Profiler::GetInstance().SetEnabled(true);
    Profiler::GetInstance().ClearFrameHistory();
    TextureStorage::ResetDescriptorUpdateStats();
    loop();
    while (TextureStorage::HasPendingUploads() && readiness_frames < max_readiness_frames) {
      loop();
      ++readiness_frames;
    }
    texture_2d_registration_revision = TextureStorage::GetTexture2DRegistrationRevision();
    cubemap_registration_revision = TextureStorage::GetCubemapRegistrationRevision();
  }
  if (require_accumulated_frames) {
    const auto target_frame_count = static_cast<uint32_t>(std::max(1, warmup_frames));
    const auto maximum_loop_count = target_frame_count + 600u;
    main_camera->ResetFrameCount();
    uint32_t loop_count = 0;
    while (main_camera->GetFrameCount() < target_frame_count && loop_count < maximum_loop_count) {
      if (!loop()) {
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
      loop();
    }
  }
  const auto final_texture_2d_registration_revision = TextureStorage::GetTexture2DRegistrationRevision();
  const auto final_cubemap_registration_revision = TextureStorage::GetCubemapRegistrationRevision();
  if (require_stable_texture_registrations &&
      (texture_2d_registration_revision != final_texture_2d_registration_revision ||
       cubemap_registration_revision != final_cubemap_registration_revision)) {
    EVOENGINE_ERROR("Transient capture resources changed persistent sampled-texture registrations: 2D " +
                    std::to_string(texture_2d_registration_revision) + " -> " +
                    std::to_string(final_texture_2d_registration_revision) + ", cubemap " +
                    std::to_string(cubemap_registration_revision) + " -> " +
                    std::to_string(final_cubemap_registration_revision) + ".")
    return false;
  }
  if (require_stable_texture_registrations) {
    EVOENGINE_LOG("EVOENGINE_TRANSIENT_TEXTURE_REGISTRATIONS_STABLE texture_2d=" +
                  std::to_string(texture_2d_registration_revision) +
                  ", cubemap=" + std::to_string(cubemap_registration_revision))
    LogSampledTextureDiagnostics(ApplicationContext::Get().GetLayer<RenderLayer>());
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

bool PyEvoEngine::CaptureSecondarySceneCamera(const std::filesystem::path& output_path) {
  const auto camera = GetRuntime().capture_secondary_camera.lock();
  if (!camera || !camera->GetRenderTexture()) {
    EVOENGINE_ERROR("Secondary capture camera has no render texture.")
    return false;
  }
  if (const auto parent_path = output_path.parent_path(); !parent_path.empty()) {
    std::filesystem::create_directories(parent_path);
  }
  camera->GetRenderTexture()->StoreToPng(output_path);
  return std::filesystem::exists(output_path) && std::filesystem::file_size(output_path) > 0;
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

  m.def("RunWindowless", &RunWindowless);
  m.def("RunDemoWindowless", &RunDemoWindowless, py::arg("demo_setup_name"), py::arg("resource_folder_path"),
        py::arg("clear_generated_project_files") = true, py::arg("enable_ray_features") = true);
  m.def("ExerciseTextureLifecycleForCapture", &ExerciseTextureLifecycleForCapture);
  m.def("ConfigureCurrentSceneCameraForCapture", &ConfigureCurrentSceneCameraForCapture, py::arg("render_mode"),
        py::arg("samples_per_frame"), py::arg("bounces"));
  m.def("ConfigureSecondarySceneCameraForCapture", &ConfigureSecondarySceneCameraForCapture, py::arg("resolution_x"),
        py::arg("resolution_y"));
  m.def("ConfigureRasterPathForCapture", &ConfigureRasterPathForCapture, py::arg("meshlet_enabled"),
        py::arg("indirect_enabled"));
  m.def("CaptureCurrentScene", &CaptureCurrentScene, py::arg("resolution_x"), py::arg("resolution_y"),
        py::arg("output_path"), py::arg("warmup_frames") = 1, py::arg("require_accumulated_frames") = false,
        py::arg("require_stable_texture_registrations") = false);
  m.def("CaptureSecondarySceneCamera", &CaptureSecondarySceneCamera, py::arg("output_path"));
  m.def("IsCurrentSceneDdgiEnabled", &IsCurrentSceneDdgiEnabled);
  m.def("SharedTextureDescriptorArraysEnabled", []() {
    const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>();
    return render_layer && render_layer->SharedTextureDescriptorArraysEnabled();
  });
  m.def("RayTracingEnabled", &Platform::RayTracingEnabled);
  m.def("RayQueryEnabled", &Platform::RayQueryEnabled);
  m.def("RayAccelerationStructureEnabled", &Platform::RayAccelerationStructureEnabled);
  m.def(
      "SdfgiCapabilityReport",
      [](const uint32_t cascade_count, const uint32_t history_size) {
        const auto report = QuerySdfgiCapabilities(cascade_count, history_size);
        py::dict result;
        result["supported"] = report.Supported();
        result["device_name"] = report.device_name;
        result["driver_version"] = report.driver_version;
        result["reference_commit"] = kSdfgiReferenceCommit;
        result["ray_tracing_enabled"] = report.ray_tracing_enabled;
        result["ray_query_enabled"] = report.ray_query_enabled;
        result["blas_enabled"] = report.acceleration_structures_enabled;
        result["tlas_enabled"] = report.acceleration_structures_enabled;
        result["summary"] = report.ToString();
        py::dict checks;
        for (const auto& check : report.checks) {
          checks[py::str(check.name)] = check.supported;
        }
        result["checks"] = checks;
        return result;
      },
      py::arg("cascade_count") = 4, py::arg("history_size") = 30);
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
                                    const bool clear_generated_project_files, const bool enable_ray_features) {
  const auto demo_setup = ParseDemoSetupName(demo_setup_name);
  if (demo_setup == DemoSetup::Empty && demo_setup_name != "Empty") {
    return false;
  }

  EnsureRenderLayer();
  ApplicationInitializationSettings application_info{};
  SetupDemoScene(demo_setup, application_info, resource_folder_path, clear_generated_project_files);
  application_info.graphics_settings.use_ray_tracing = enable_ray_features;
  if (application_info.project_path.empty()) {
    EVOENGINE_ERROR("Demo setup did not provide a project path!");
    return false;
  }
  ApplicationContext::Get().Initialize(application_info);
  ApplicationContext::Get().Start();
  return true;
}

bool PyEvoEngine::ExerciseTextureLifecycleForCapture() {
  if (!Platform::Initialized()) {
    return false;
  }
  auto texture_a = std::make_shared<Texture2D>();
  auto texture_retired = std::make_shared<Texture2D>();
  auto texture_tail = std::make_shared<Texture2D>();
  const auto texture_a_index = texture_a->GetTextureStorageIndex();
  const auto texture_retired_index = texture_retired->GetTextureStorageIndex();
  const auto texture_tail_index = texture_tail->GetTextureStorageIndex();
  texture_retired.reset();
  TextureStorage::DeviceSync();
  auto texture_replacement = std::make_shared<Texture2D>();
  const bool texture_indices_stable = texture_a->GetTextureStorageIndex() == texture_a_index &&
                                      texture_tail->GetTextureStorageIndex() == texture_tail_index &&
                                      texture_replacement->GetTextureStorageIndex() == texture_retired_index;

  auto cubemap_a = std::make_shared<Cubemap>();
  auto cubemap_retired = std::make_shared<Cubemap>();
  auto cubemap_tail = std::make_shared<Cubemap>();
  const auto cubemap_a_index = cubemap_a->GetTextureStorageIndex();
  const auto cubemap_retired_index = cubemap_retired->GetTextureStorageIndex();
  const auto cubemap_tail_index = cubemap_tail->GetTextureStorageIndex();
  cubemap_retired.reset();
  TextureStorage::DeviceSync();
  auto cubemap_replacement = std::make_shared<Cubemap>();
  const bool cubemap_indices_stable = cubemap_a->GetTextureStorageIndex() == cubemap_a_index &&
                                      cubemap_tail->GetTextureStorageIndex() == cubemap_tail_index &&
                                      cubemap_replacement->GetTextureStorageIndex() == cubemap_retired_index;

  texture_a.reset();
  texture_tail.reset();
  texture_replacement.reset();
  cubemap_a.reset();
  cubemap_tail.reset();
  cubemap_replacement.reset();
  TextureStorage::DeviceSync();
  return texture_indices_stable && cubemap_indices_stable;
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
