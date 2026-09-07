#include "PyEvoEngine.hpp"
#include "Cubemap.hpp"
#include "EnvironmentalLightingResolver.hpp"
#include "GeometryStorage.hpp"
#include "ImGuiLayer.hpp"
#include "Lights.hpp"
#include "MeshRenderer.hpp"
#include "Platform.hpp"
#include "Profiler.hpp"
#include "SdfgiCapabilities.hpp"
#include "SdfgiDebug.hpp"
#include "SdfgiGather.hpp"
#include "SdfgiLight.hpp"
#include "SdfgiPreprocess.hpp"
#include "SdfgiProbe.hpp"
#include "SdfgiResources.hpp"
#include "SdfgiRuntime.hpp"
#include "SdfgiVoxelizer.hpp"
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
  std::vector<glm::vec4> pixels;
  main_camera->GetRenderTexture()->GetRgbaChannelData(pixels);
  if (pixels.empty() || std::any_of(pixels.begin(), pixels.end(), [](const auto& pixel) {
        return !std::isfinite(pixel.x) || !std::isfinite(pixel.y) || !std::isfinite(pixel.z) || !std::isfinite(pixel.w);
      })) {
    EVOENGINE_ERROR("Capture contains empty or nonfinite output.")
    return false;
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
  m.def("ResizeCurrentSceneCameraForCapture", [](const uint32_t width, const uint32_t height) {
    const auto scene = ApplicationContext::Get().GetActiveScene();
    const auto camera = scene ? scene->main_camera.Get<Camera>() : nullptr;
    if (!camera || width == 0 || height == 0)
      throw py::value_error("A main camera and positive capture resolution are required");
    camera->Resize({width, height});
  });
  m.def("IsCurrentSceneReadyForCapture", []() {
    return ProjectManager::IsProjectIdle() && !GeometryStorage::HasPendingUploads() &&
           !TextureStorage::HasPendingUploads();
  });
  m.def("GetCurrentSceneCameraPositionForCapture", []() {
    const auto scene = ApplicationContext::Get().GetActiveScene();
    const auto camera = scene ? scene->main_camera.Get<Camera>() : nullptr;
    if (!camera)
      throw py::value_error("A main camera is required");
    const auto position = scene->GetDataComponent<GlobalTransform>(camera->GetOwner()).GetPosition();
    return py::make_tuple(position.x, position.y, position.z);
  });
  m.def("SetCurrentSceneCameraPositionForCapture", [](const float x, const float y, const float z) {
    const auto scene = ApplicationContext::Get().GetActiveScene();
    const auto camera = scene ? scene->main_camera.Get<Camera>() : nullptr;
    if (!camera || !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
      throw py::value_error("A main camera and finite position are required");
    auto transform = scene->GetDataComponent<GlobalTransform>(camera->GetOwner());
    transform.SetPosition({x, y, z});
    scene->SetDataComponent(camera->GetOwner(), transform);
  });
  m.def("IsCurrentSceneDdgiEnabled", &IsCurrentSceneDdgiEnabled);
  m.def("ScaleCurrentSceneStaticMaterialsForCapture", [](const float color_scale, const float emission_scale) {
    const auto scene = ApplicationContext::Get().GetActiveScene();
    if (!scene || !std::isfinite(color_scale) || !std::isfinite(emission_scale) || color_scale < 0 ||
        emission_scale < 0)
      throw py::value_error("An active scene and finite nonnegative scales are required");
    std::set<uint64_t> edited;
    if (const auto owners = scene->UnsafeGetPrivateComponentOwnersList<MeshRenderer>())
      for (const auto entity : *owners) {
        const auto renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(entity).lock();
        const auto material = renderer->material.Get<Material>();
        if (!scene->IsEntityStatic(entity) || !material || !edited.insert(material->GetHandle().GetValue()).second)
          continue;
        auto& data = material->material_data.shade_material;
        data.pbr_base_color_factor *= glm::vec4(color_scale, color_scale, color_scale, 1);
        data.emissive_factor *= emission_scale;
        material->SetUnsaved();
      }
    return edited.size();
  });
  m.def("ScaleCurrentSceneDirectionalLightsForCapture", [](const float scale) {
    const auto scene = ApplicationContext::Get().GetActiveScene();
    if (!scene || !std::isfinite(scale) || scale < 0)
      throw py::value_error("An active scene and finite nonnegative scale are required");
    uint32_t count = 0;
    if (const auto owners = scene->UnsafeGetPrivateComponentOwnersList<DirectionalLight>())
      for (const auto entity : *owners) {
        scene->GetOrSetPrivateComponent<DirectionalLight>(entity).lock()->diffuse_brightness *= scale;
        ++count;
      }
    return count;
  });
  m.def("SharedTextureDescriptorArraysEnabled", []() {
    const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>();
    return render_layer && render_layer->SharedTextureDescriptorArraysEnabled();
  });
  m.def("RayTracingEnabled", &Platform::RayTracingEnabled);
  m.def("RayQueryEnabled", &Platform::RayQueryEnabled);
  m.def("RayAccelerationStructureEnabled", &Platform::RayAccelerationStructureEnabled);
  py::enum_<IndirectGiProvider>(m, "IndirectGiProvider")
      .value("Environment", IndirectGiProvider::Environment)
      .value("AuthoredDdgi", IndirectGiProvider::AuthoredDdgi)
      .value("AutomaticSdfgi", IndirectGiProvider::AutomaticSdfgi);
  py::enum_<SdfgiSettings::VerticalScale>(m, "SdfgiVerticalScale")
      .value("Percent50", SdfgiSettings::VerticalScale::Percent50)
      .value("Percent75", SdfgiSettings::VerticalScale::Percent75)
      .value("Percent100", SdfgiSettings::VerticalScale::Percent100);
  py::class_<SdfgiSettings>(m, "SdfgiSettings")
      .def(py::init<>())
      .def_readwrite("cascade_count", &SdfgiSettings::cascade_count)
      .def_readwrite("positional_light_cascade_count", &SdfgiSettings::positional_light_cascade_count)
      .def_readwrite("voxel_count_x", &SdfgiSettings::voxel_count_x)
      .def_readwrite("probe_spacing_cells", &SdfgiSettings::probe_spacing_cells)
      .def_readwrite("voxel_count_y", &SdfgiSettings::voxel_count_y)
      .def_readwrite("min_cell_size", &SdfgiSettings::min_cell_size)
      .def_property("cascade0_distance", &SdfgiSettings::GetCascade0Distance, &SdfgiSettings::SetCascade0Distance)
      .def_property("max_distance", &SdfgiSettings::GetMaxDistance, &SdfgiSettings::SetMaxDistance)
      .def_readwrite("vertical_scale", &SdfgiSettings::vertical_scale)
      .def_readwrite("use_occlusion", &SdfgiSettings::use_occlusion)
      .def_readwrite("probe_relocation", &SdfgiSettings::probe_relocation)
      .def_readwrite("static_entities_only", &SdfgiSettings::static_entities_only)
      .def_readwrite("ray_count", &SdfgiSettings::ray_count)
      .def_readwrite("history_size", &SdfgiSettings::history_size)
      .def_readwrite("light_update_frames", &SdfgiSettings::light_update_frames)
      .def_readwrite("bounce_feedback", &SdfgiSettings::bounce_feedback)
      .def_readwrite("read_sky_light", &SdfgiSettings::read_sky_light)
      .def_readwrite("energy", &SdfgiSettings::energy)
      .def_readwrite("normal_bias", &SdfgiSettings::normal_bias)
      .def_readwrite("probe_bias", &SdfgiSettings::probe_bias)
      .def_readwrite("anchor_camera_entity", &SdfgiSettings::anchor_camera_entity);
  m.def("SetCurrentSceneGiProvider", [](const IndirectGiProvider provider) {
    const auto scene = ApplicationContext::Get().GetActiveScene();
    const auto lighting = scene ? scene->environmental_lighting.Get<EnvironmentalLighting>() : nullptr;
    if (!lighting)
      throw py::value_error("The active scene has no EnvironmentalLighting asset");
    lighting->indirect_gi_provider = provider;
    if (provider == IndirectGiProvider::AuthoredDdgi)
      lighting->ddgi_settings.runtime.enabled = true;
    lighting->SetUnsaved();
  });
  m.def("SetCurrentSceneSdfgiSettings", [](const SdfgiSettings& settings) {
    if (const auto error = settings.Validate(); !error.empty())
      throw py::value_error(error);
    const auto render = ApplicationContext::Get().GetLayer<RenderLayer>();
    const auto report = QuerySdfgiCapabilities(settings.cascade_count, settings.history_size, settings.voxel_count_x,
                                               settings.voxel_count_y, settings.probe_spacing_cells,
                                               render ? render->GetDdgiHistoryAllocationBytes() : 0);
    if (!report.Supported())
      throw py::value_error(report.ToString());
    const auto scene = ApplicationContext::Get().GetActiveScene();
    const auto lighting = scene ? scene->environmental_lighting.Get<EnvironmentalLighting>() : nullptr;
    if (!lighting)
      throw py::value_error("The active scene has no EnvironmentalLighting asset");
    lighting->sdfgi_settings = settings;
    lighting->SetUnsaved();
  });
  m.def("GetCurrentSceneDdgiHistoryCount", [] {
    return ResolveEnvironmentalLighting(ApplicationContext::Get().GetActiveScene()).ddgi_settings.runtime.history_count;
  });
  m.def("GetCurrentSceneDdgiHistoryStatus", [] {
    py::list volumes;
    if (const auto render = ApplicationContext::Get().GetLayer<RenderLayer>()) {
      const auto snapshot = render->GetDdgiInspectorSnapshot();
      for (const auto& volume : snapshot.volumes) {
        py::dict item;
        item["volume_id"] = volume.stable_entity_id;
        item["phase"] = volume.history_phase;
        item["count"] = volume.history_count;
        item["completed_updates"] = volume.history_completed_updates;
        item["ready"] = volume.sampling_complete;
        item["relocation_warmup"] = volume.warmup_active;
        item["resources_ready"] = volume.resources_ready;
        volumes.append(item);
      }
    }
    return volumes;
  });
  m.def("SetCurrentSceneDdgiHistoryCount", [](const int count) {
    const auto render = ApplicationContext::Get().GetLayer<RenderLayer>();
    std::string error;
    if (!render || !render->SetDdgiHistoryCount(ApplicationContext::Get().GetActiveScene(), count, error))
      throw py::value_error(error.empty() ? "RenderLayer is unavailable" : error);
  });
  m.def(
      "RequestCurrentSceneSdfgiVoxelDebug",
      [](const uint32_t cascade, uint32_t slice) {
        const auto scene = ApplicationContext::Get().GetActiveScene();
        const auto runtime = scene ? scene->GetSdfgiRuntime() : nullptr;
        if (!runtime || !runtime->resources)
          throw py::value_error("Automatic SDFGI resources are not available");
        const uint32_t slice_count = std::min(runtime->settings.voxel_count_x, runtime->settings.voxel_count_y);
        if (slice == UINT32_MAX)
          slice = slice_count / 2;
        if (cascade >= runtime->settings.cascade_count || slice >= slice_count)
          throw py::value_error("SDFGI cascade or slice is out of range");
        runtime->resources->voxel_debug_request = glm::uvec2(cascade, slice);
      },
      py::arg("cascade") = 0, py::arg("slice") = UINT32_MAX);
  m.def("CaptureCurrentSceneSdfgiVoxelDebug", [](const std::filesystem::path& path) {
    const auto scene = ApplicationContext::Get().GetActiveScene();
    const auto runtime = scene ? scene->GetSdfgiRuntime() : nullptr;
    const auto resources = runtime ? runtime->resources : nullptr;
    if (!resources || resources->voxel_debug_request || !resources->voxel_debug)
      throw py::value_error("Request a voxel diagnostic and render a frame before capture");
    resources->voxel_debug->StoreToPng(path);
  });
  m.def(
      "RequestCurrentSceneSdfgiPreprocessDebug",
      [](const uint32_t cascade, uint32_t slice) {
        const auto scene = ApplicationContext::Get().GetActiveScene();
        const auto runtime = scene ? scene->GetSdfgiRuntime() : nullptr;
        if (!runtime || !runtime->resources)
          throw py::value_error("Automatic SDFGI resources are not available");
        const uint32_t slice_count = std::min(runtime->settings.voxel_count_x, runtime->settings.voxel_count_y);
        if (slice == UINT32_MAX)
          slice = slice_count / 2;
        if (cascade >= runtime->settings.cascade_count || slice >= slice_count)
          throw py::value_error("SDFGI cascade or slice is out of range");
        runtime->resources->preprocess_debug_request = glm::uvec2(cascade, slice);
      },
      py::arg("cascade") = 0, py::arg("slice") = UINT32_MAX);
  m.def("CaptureCurrentSceneSdfgiPreprocessDebug", [](const std::filesystem::path& path) {
    const auto scene = ApplicationContext::Get().GetActiveScene();
    const auto runtime = scene ? scene->GetSdfgiRuntime() : nullptr;
    const auto resources = runtime ? runtime->resources : nullptr;
    if (!resources || resources->preprocess_debug_request || !resources->preprocess_debug)
      throw py::value_error("Request a preprocessing diagnostic and render a frame before capture");
    resources->preprocess_debug->StoreToPng(path);
    for (const auto& frame : resources->voxel_frames)
      if (frame && frame->preprocess_readback)
        frame->preprocess_readback->ReadAfterFrameFence(*resources);
  });
  m.def(
      "RequestCurrentSceneSdfgiLightDebug",
      [](const uint32_t cascade, uint32_t slice) {
        const auto scene = ApplicationContext::Get().GetActiveScene();
        const auto runtime = scene ? scene->GetSdfgiRuntime() : nullptr;
        if (!runtime || !runtime->resources)
          throw py::value_error("Automatic SDFGI resources are not available");
        const uint32_t slice_count = std::min(runtime->settings.voxel_count_x, runtime->settings.voxel_count_y);
        if (slice == UINT32_MAX)
          slice = slice_count / 2;
        if (cascade >= runtime->settings.cascade_count || slice >= slice_count)
          throw py::value_error("SDFGI cascade or slice is out of range");
        runtime->resources->light_debug_request = glm::uvec2(cascade, slice);
      },
      py::arg("cascade") = 0, py::arg("slice") = UINT32_MAX);
  m.def("CaptureCurrentSceneSdfgiLightDebug", [](const std::filesystem::path& path) {
    const auto scene = ApplicationContext::Get().GetActiveScene();
    const auto runtime = scene ? scene->GetSdfgiRuntime() : nullptr;
    const auto resources = runtime ? runtime->resources : nullptr;
    if (!resources || resources->light_debug_request || !resources->light_debug)
      throw py::value_error("Request a lighting diagnostic and render a frame before capture");
    resources->light_debug->StoreToPng(path);
  });
  m.def("SetGpuTimingCaptureEnabled", &Platform::SetGpuTimestampCaptureEnabled);
  m.def("ReadCurrentSceneSdfgiFieldStatus", []() {
    const auto scene = ApplicationContext::Get().GetActiveScene();
    const auto runtime = scene ? scene->GetSdfgiRuntime() : nullptr;
    if (!runtime || !runtime->resources)
      throw py::value_error("Automatic SDFGI resources are unavailable");
    Platform::WaitForFrameSubmissions("SDFGI explicit field-status readback");
    SdfgiFieldStatus status;
    runtime->resources->buffers.at("Status").buffer->Download(status);
    py::dict result;
    result["generation"] = status.generation;
    result["ready"] = status.ready;
    result["failure_flags"] = status.failure_flags;
    return result;
  });
  m.def(
      "RequestCurrentSceneSdfgiProbeDebug",
      [](const uint32_t cascade, uint32_t probe) {
        const auto scene = ApplicationContext::Get().GetActiveScene();
        const auto runtime = scene ? scene->GetSdfgiRuntime() : nullptr;
        if (!runtime || !runtime->resources)
          throw py::value_error("Automatic SDFGI resources are not available");
        const auto size = runtime->settings.ProbeSize();
        if (probe == UINT32_MAX)
          probe = size.x / 2 + size.z / 2 * size.x + size.y / 2 * size.x * size.z;
        if (cascade >= runtime->settings.cascade_count || probe >= uint32_t(size.x * size.y * size.z))
          throw py::value_error("SDFGI cascade or probe is out of range");
        runtime->resources->probe_debug_request = glm::uvec2(cascade, probe);
      },
      py::arg("cascade") = 0, py::arg("probe") = UINT32_MAX);
  m.def("CaptureCurrentSceneSdfgiProbeDebug", [](const std::filesystem::path& path) {
    const auto scene = ApplicationContext::Get().GetActiveScene();
    const auto runtime = scene ? scene->GetSdfgiRuntime() : nullptr;
    const auto resources = runtime ? runtime->resources : nullptr;
    if (!resources || resources->probe_debug_request || !resources->probe_debug)
      throw py::value_error("Request a probe diagnostic and render a frame before capture");
    resources->probe_debug->StoreToPng(path);
    const auto status = resources->probe_debug->ReadStatus();
    py::dict result;
    result["generation"] = status.generation;
    result["ready"] = status.ready;
    result["failure_flags"] = status.failure_flags;
    return result;
  });
  py::enum_<SdfgiDebugView>(m, "SdfgiDebugView")
      .value("Beauty", SdfgiDebugView::None)
      .value("Cascades", SdfgiDebugView::Cascades)
      .value("Sdf", SdfgiDebugView::Sdf)
      .value("Probes", SdfgiDebugView::Probes)
      .value("Visibility", SdfgiDebugView::Visibility)
      .value("DirtyRegions", SdfgiDebugView::DirtyRegions)
      .value("DistanceSlice", SdfgiDebugView::DistanceSlice)
      .value("Diffuse", SdfgiDebugView::Diffuse)
      .value("Specular", SdfgiDebugView::Specular)
      .value("Fallback", SdfgiDebugView::Fallback)
      .value("Contributors", SdfgiDebugView::Contributors);
  py::class_<SdfgiDebugState, std::shared_ptr<SdfgiDebugState>>(m, "SdfgiDebugState")
      .def_readwrite("enabled", &SdfgiDebugState::enabled)
      .def_readwrite("frozen", &SdfgiDebugState::frozen)
      .def_readwrite("single_step", &SdfgiDebugState::single_step)
      .def_readwrite("full_redraw", &SdfgiDebugState::full_redraw)
      .def_readwrite("reset_history", &SdfgiDebugState::reset_history)
      .def_readwrite("view", &SdfgiDebugState::view)
      .def_readwrite("cascade", &SdfgiDebugState::cascade)
      .def_readwrite("probe", &SdfgiDebugState::probe)
      .def_readwrite("slice", &SdfgiDebugState::slice)
      .def_readwrite("depth_test", &SdfgiDebugState::depth_test)
      .def_readwrite("camera_id", &SdfgiDebugState::camera_id)
      .def_property(
          "seed",
          [](const SdfgiDebugState& state) {
            return state.seed;
          },
          [](SdfgiDebugState& state, uint32_t seed) {
            if (state.seed != seed) {
              state.seed = seed;
              state.reset_history = true;
            }
          });
  const auto debug_runtime = []() {
    const auto scene = ApplicationContext::Get().GetActiveScene();
    const auto runtime = scene ? scene->GetSdfgiRuntime() : nullptr;
    if (!runtime)
      throw py::value_error("Automatic SDFGI runtime unavailable");
    return runtime;
  };
  m.def("GetCurrentSceneSdfgiDebug", [debug_runtime]() {
    return debug_runtime()->debug;
  });
  m.def("SelectMainCameraForSdfgiDebug", [debug_runtime]() {
    const auto camera = ApplicationContext::Get().GetActiveScene()->main_camera.Get<Camera>();
    if (!camera)
      throw py::value_error("Main camera unavailable");
    debug_runtime()->debug->camera_id = camera->GetHandle().GetValue();
  });
  m.def("GetCurrentSceneSdfgiSnapshot", [debug_runtime]() {
    return BuildSdfgiDebugSnapshot(*debug_runtime());
  });
  m.def("CaptureCurrentSceneSdfgiDebug", [debug_runtime](const std::filesystem::path& path) {
    CaptureSdfgiDebugImage(*debug_runtime(), path);
  });
  m.def("GetCurrentSceneGiStatus", []() {
    const auto scene = ApplicationContext::Get().GetActiveScene();
    const auto lighting = ResolveEnvironmentalLighting(scene);
    const auto runtime = scene ? scene->GetSdfgiRuntime() : nullptr;
    py::dict result;
    result["requested_provider"] = GetIndirectGiProviderName(lighting.indirect_gi_provider);
    auto effective = IndirectGiProvider::Environment;
    if (lighting.indirect_gi_provider == IndirectGiProvider::AutomaticSdfgi && runtime && runtime->published)
      effective = IndirectGiProvider::AutomaticSdfgi;
    if (lighting.indirect_gi_provider == IndirectGiProvider::AuthoredDdgi) {
      if (const auto render = ApplicationContext::Get().GetLayer<RenderLayer>()) {
        const auto snapshot = render->GetDdgiInspectorSnapshot();
        if (snapshot.enabled && snapshot.aggregate.active_probe_count && snapshot.aggregate.lighting_descriptors_bound)
          effective = IndirectGiProvider::AuthoredDdgi;
      }
    }
    result["effective_provider"] = GetIndirectGiProviderName(effective);
    result["sdfgi_state_active"] = runtime != nullptr;
    const auto resources = runtime ? runtime->resources : nullptr;
    result["gpu_resources_allocated"] = resources != nullptr;
    result["gpu_initialization_recorded"] = resources && resources->initialization_recorded;
    py::dict memory;
    const char* memory_names[]{"field", "scratch", "upload", "diagnostic"};
    for (size_t i = 0; i < 4; ++i)
      memory[memory_names[i]] = resources ? resources->GetAllocationBytes(static_cast<SdfgiMemoryClass>(i)) : 0;
    result["active_allocation_bytes"] = memory;
    result["voxelization_recorded"] = resources && resources->voxelization_recorded;
    result["voxel_failure"] = resources ? resources->voxel_failure : std::string{};
    result["voxel_debug_recorded"] = resources && resources->voxel_debug && resources->voxel_debug->recorded;
    result["preprocessed_cascades"] = resources ? resources->preprocessed_cascades : 0;
    result["geometry_update_count"] = resources ? resources->geometry_update_count : 0;
    result["payload_update_count"] = resources ? resources->payload_update_count : 0;
    result["invalidation_reason"] = runtime ? runtime->invalidation_reason : std::string{};
    py::list pending_changes;
    if (runtime)
      for (const auto flags : runtime->pending_changes)
        pending_changes.append(flags);
    result["pending_changes"] = pending_changes;
    result["payload_cascades"] = runtime ? runtime->payload_cascades : 0;
    result["preprocess_status_available"] = resources && resources->preprocess_status_available;
    result["preprocess_failure_flags"] = resources ? resources->preprocess_status.failure_flags : 0;
    result["preprocess_failure"] = resources ? resources->preprocess_failure : std::string{};
    result["preprocess_debug_recorded"] =
        resources && resources->preprocess_debug && resources->preprocess_debug->recorded;
    result["preprocess_debug_failure"] = resources ? resources->preprocess_debug_failure : std::string{};
    py::list solid_cells;
    if (resources)
      for (const auto& dispatch : resources->solid_cell_dispatch) {
        py::dict counts;
        counts["total"] = dispatch.total_count;
        counts["groups"] = py::make_tuple(dispatch.x, dispatch.y, dispatch.z);
        solid_cells.append(counts);
      }
    result["solid_cells"] = solid_cells;
    result["lighting_recorded"] = resources && resources->lighting_recorded;
    result["light_failure"] = resources ? resources->light_failure : std::string{};
    result["light_debug_recorded"] = resources && resources->light_debug && resources->light_debug->recorded;
    result["light_debug_failure"] = resources ? resources->light_debug_failure : std::string{};
    result["transport_recorded"] = resources && resources->transport_recorded;
    result["transport_pass"] = resources ? resources->transport_pass : 0;
    result["published_generation"] =
        resources && resources->publication ? resources->publication->metadata.generation : 0;
    py::list sdfgi_camera_ids;
    if (resources)
      for (const auto id : resources->gather_camera_ids)
        sdfgi_camera_ids.append(id);
    result["sdfgi_camera_ids"] = sdfgi_camera_ids;
    result["transport_failure"] = resources ? resources->transport_failure : std::string{};
    result["probe_debug_recorded"] = resources && resources->probe_debug && resources->probe_debug->recorded;
    result["probe_debug_failure"] = resources ? resources->probe_debug_failure : std::string{};
    py::dict transport;
    if (resources && resources->last_transport_frame != UINT32_MAX)
      for (const auto& frame : resources->probe_frames)
        if (frame && frame->scene_frame == resources->last_transport_frame && !frame->constants.empty()) {
          const auto& params = frame->constants[0];
          transport["history_index"] = params.history_index;
          transport["history_size"] = params.history_size;
          transport["ray_count"] = params.ray_count;
          transport["sky_flags"] = params.sky_flags;
          transport["sky_lod"] = params.sky_lod_inverse_gamma[0];
          transport["sky_inverse_gamma"] = params.sky_lod_inverse_gamma[1];
          transport["sky_energy"] = params.sky_energy;
          transport["bounce_feedback"] = resources->settings.bounce_feedback;
        }
    result["transport"] = transport;
    py::list timings;
    for (const auto& frame : Platform::GetGpuTimestampFrameHistory())
      if (frame.results_available) {
        py::dict stages;
        for (const auto& sample : frame.samples)
          if (sample.metadata.group == "SDFGI")
            stages[py::str(sample.metadata.stable_pass_id)] = sample.duration_milliseconds;
        if (stages.size()) {
          py::dict value;
          value["application_frame"] = frame.application_frame_index;
          value["stages_ms"] = stages;
          timings.append(value);
        }
      }
    result["sdfgi_gpu_timings"] = timings;
    py::list light_counts;
    if (resources && resources->last_light_frame != UINT32_MAX)
      for (const auto& frame : resources->light_frames)
        if (frame && frame->scene_frame == resources->last_light_frame)
          for (const auto& lights : frame->lights) {
            py::dict counts;
            counts["static"] = lights.data[0].size();
            counts["dynamic"] = lights.data[1].size();
            counts["static_overflow"] = lights.overflow[0];
            counts["dynamic_overflow"] = lights.overflow[1];
            light_counts.append(counts);
          }
    result["cascade_lights"] = light_counts;
    result["maintenance_count"] = runtime ? runtime->maintenance_count : 0;
    result["published"] = runtime && runtime->published;
    result["missing_anchor"] = !runtime || runtime->missing_anchor;
    result["anchor_camera_id"] = runtime ? runtime->anchor.camera_id : 0;
    result["anchor_source"] = runtime ? static_cast<uint32_t>(runtime->anchor.source) : 0;
    result["anchor_override_fell_back"] = runtime && runtime->anchor.override_fell_back;
    result["anchor_replaced"] = runtime && runtime->anchor_replaced;
    result["accepted_contributor_count"] = runtime ? runtime->contributors.entries.size() : 0;
    result["contributor_change_count"] = runtime ? runtime->contributors.changes.size() : 0;
    py::dict exclusions;
    py::list cascades;
    py::list pending_regions;
    py::dict lights;
    uint32_t static_lights = 0, dynamic_lights = 0;
    if (runtime) {
      py::list changes;
      for (const auto flags : runtime->contributors.AffectedCascades(
               runtime->cascades, SdfgiYMultiplier(runtime->settings.vertical_scale)))
        changes.append(flags);
      result["cascade_input_changes"] = changes;
      for (const auto& [reason, count] : runtime->scene_snapshot.excluded)
        exclusions[GetSdfgiExclusionName(reason)] = count;
      for (const auto& light : runtime->scene_snapshot.lights)
        light.dynamic ? ++dynamic_lights : ++static_lights;
      for (const auto& cascade : runtime->cascades) {
        py::dict value;
        value["cell_size"] = cascade.cell_size;
        value["position"] = py::make_tuple(cascade.position.x, cascade.position.y, cascade.position.z);
        value["dirty_regions"] =
            py::make_tuple(cascade.dirty_regions.x, cascade.dirty_regions.y, cascade.dirty_regions.z);
        value["full_redraw"] = cascade.full_redraw;
        cascades.append(value);
      }
      for (const auto& region : runtime->pending_regions) {
        py::dict value;
        value["cascade"] = region.cascade;
        value["offset"] = py::make_tuple(region.offset.x, region.offset.y, region.offset.z);
        value["size"] = py::make_tuple(region.size.x, region.size.y, region.size.z);
        pending_regions.append(value);
      }
    }
    lights["static"] = static_lights;
    lights["dynamic"] = dynamic_lights;
    result["excluded_contributors"] = exclusions;
    result["lights"] = lights;
    result["cascades"] = cascades;
    result["pending_regions"] = pending_regions;
    result["fallback_reason"] = runtime ? runtime->fallback_reason : std::string{};
    return result;
  });
  m.def(
      "SdfgiCapabilityReport",
      [](const uint32_t cascade_count, const uint32_t history_size, const uint32_t voxel_count_x,
         const uint32_t voxel_count_y, const uint32_t probe_spacing_cells) {
        const auto render = ApplicationContext::Get().GetLayer<RenderLayer>();
        const auto report =
            QuerySdfgiCapabilities(cascade_count, history_size, voxel_count_x, voxel_count_y, probe_spacing_cells,
                                   render ? render->GetDdgiHistoryAllocationBytes() : 0);
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
      py::arg("cascade_count") = 4, py::arg("history_size") = 30, py::arg("voxel_count_x") = 128,
      py::arg("voxel_count_y") = 64, py::arg("probe_spacing_cells") = 4);
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
