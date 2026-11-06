#include "PyEvoEngine.hpp"
#include <cmath>
#include "EnvironmentalLighting.hpp"
#include "EnvironmentalLightingResolver.hpp"
#include "EnvironmentalMap.hpp"
#include "GeometryStorage.hpp"
#include "ImGuiLayer.hpp"
#include "Lights.hpp"
#include "TextureStorage.hpp"
#include "TransformGraph.hpp"
#include "pybind11/numpy.h"
#ifdef CUDA_MODULE_SERVICE
#  include "CUDAModule.hpp"
#  include "PrefabFieldIllumination.hpp"
#  include "RayTracerCamera.hpp"
#  include "RayTracerLayer.hpp"
#  include "VolumetricCloudPresets.hpp"
#  include "VoxelIlluminationEstimator.hpp"
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
#ifdef CUDA_MODULE_SERVICE
struct PythonCloudSettings : VolumetricCloudSettings {
  bool affect_illumination_probes = false;
};

std::shared_ptr<RayTracerLayer> GetCloudLayer() {
  const auto layer = ApplicationContext::Get().GetLayer<RayTracerLayer>();
  if (!layer || !layer->scene_camera)
    throw std::runtime_error("Start the engine with a RayTracerLayer before accessing cloud settings.");
  return layer;
}

void RegisterCloudBindings(py::module& m) {
  py::enum_<CloudFormation>(m, "CloudFormation", py::module_local())
      .value("Scattered", CloudFormation::Scattered)
      .value("BroadBank", CloudFormation::BroadBank);
  py::class_<PythonCloudSettings> settings(m, "RayTracerCloudSettings", py::module_local());
  settings.def(py::init<>())
      .def("ApplyPreset", [](PythonCloudSettings& s, CloudFormation preset) { ApplyCloudFormation(s, preset); },
           py::arg("preset"))
      .def_readwrite("affect_illumination_probes", &PythonCloudSettings::affect_illumination_probes)
      .def_readwrite("enabled", &VolumetricCloudSettings::enabled)
      .def_readwrite("primary_step_count", &VolumetricCloudSettings::primary_step_count)
      .def_readwrite("light_step_count", &VolumetricCloudSettings::light_step_count)
      .def_readwrite("resolution_divisor", &VolumetricCloudSettings::resolution_divisor)
      .def_readwrite("use_spherical_atmosphere", &VolumetricCloudSettings::use_spherical_atmosphere)
      .def_readwrite("empty_step_fallback_count", &VolumetricCloudSettings::empty_step_fallback_count)
      .def_readwrite("enable_temporal_reprojection", &VolumetricCloudSettings::enable_temporal_reprojection)
      .def_readwrite("enable_cloud_shadows", &VolumetricCloudSettings::enable_cloud_shadows)
      .def_readwrite("cloud_shadow_step_count", &VolumetricCloudSettings::cloud_shadow_step_count)
      .def_readwrite("debug_visualization", &VolumetricCloudSettings::debug_visualization)
      .def_readwrite("debug_mode", &VolumetricCloudSettings::debug_mode)
      .def_property("wind_direction", [](const PythonCloudSettings& s) {
        return py::make_tuple(s.wind_direction.x, s.wind_direction.y);
      }, [](PythonCloudSettings& s, const std::array<float, 2>& direction) {
        if (!std::isfinite(direction[0]) || !std::isfinite(direction[1]))
          throw py::value_error("wind_direction must be finite.");
        s.wind_direction = {direction[0], direction[1]};
      });

  const auto float_field = [&](const char* name, float VolumetricCloudSettings::*member) {
    settings.def_property(name, [member](const PythonCloudSettings& s) { return s.*member; },
                          [member, name](PythonCloudSettings& s, float value) {
                            if (!std::isfinite(value))
                              throw py::value_error(std::string(name) + " must be finite.");
                            s.*member = value;
                          });
  };
  float_field("coverage", &VolumetricCloudSettings::coverage);
  float_field("density", &VolumetricCloudSettings::density);
  float_field("bottom_altitude", &VolumetricCloudSettings::bottom_altitude);
  float_field("top_altitude", &VolumetricCloudSettings::top_altitude);
  float_field("max_march_distance", &VolumetricCloudSettings::max_march_distance);
  float_field("wind_speed", &VolumetricCloudSettings::wind_speed);
  float_field("lighting_intensity", &VolumetricCloudSettings::lighting_intensity);
  float_field("ambient_lighting_strength", &VolumetricCloudSettings::ambient_lighting_strength);
  float_field("phase_anisotropy", &VolumetricCloudSettings::phase_anisotropy);
  float_field("base_noise_scale", &VolumetricCloudSettings::base_noise_scale);
  float_field("detail_noise_scale", &VolumetricCloudSettings::detail_noise_scale);
  float_field("extinction_scale", &VolumetricCloudSettings::extinction_scale);
  float_field("atmosphere_radius", &VolumetricCloudSettings::atmosphere_radius);
  float_field("cloud_type", &VolumetricCloudSettings::cloud_type);
  float_field("curl_strength", &VolumetricCloudSettings::curl_strength);
  float_field("coarse_step_fraction", &VolumetricCloudSettings::coarse_step_fraction);
  float_field("fine_step_scale", &VolumetricCloudSettings::fine_step_scale);
  float_field("temporal_blend_factor", &VolumetricCloudSettings::temporal_blend_factor);
  float_field("cloud_shadow_strength", &VolumetricCloudSettings::cloud_shadow_strength);

  m.def("GetRayTracerCloudSettings", [] {
    const auto layer = GetCloudLayer();
    return PythonCloudSettings{layer->viewport_clouds, layer->clouds_affect_illumination};
  }, "Return a detached copy of the active ray tracer layer's cloud settings.");
  m.def("SetRayTracerCloudSettings", [](const PythonCloudSettings& s) {
    GetCloudLayer()->SetCloudSettings(s, s.affect_illumination_probes);
  }, py::arg("settings"), "Clamp and apply cloud settings to the active ray tracer layer.");
}
#endif

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

bool PyEvoEngine::ConfigureCurrentSceneCudaOutdoorLightingForCapture(
    const glm::vec3& sun_euler_degrees, const float sun_angular_diameter_radians, const float sun_intensity,
    const glm::vec3& sun_color, const float sky_light_intensity, const float ambient_light_intensity,
    const float gamma) {
#ifdef CUDA_MODULE_SERVICE
  const auto layer = ApplicationContext::Get().GetLayer<RayTracerLayer>();
  if (!layer || !std::isfinite(sun_angular_diameter_radians) || !std::isfinite(sun_intensity) ||
      !std::isfinite(sky_light_intensity) || !std::isfinite(ambient_light_intensity) || !std::isfinite(gamma)) {
    return false;
  }
  auto& environment = layer->environment_properties;
  environment.environmental_lighting_type = EnvironmentalLightingType::Skydome;
  environment.sun_direction = glm::quat(glm::radians(sun_euler_degrees)) * glm::vec3(0.0f, 0.0f, -1.0f);
  environment.sun_angular_diameter_radians = glm::clamp(sun_angular_diameter_radians, 0.0f, glm::pi<float>());
  environment.sun_intensity = std::max(0.0f, sun_intensity);
  environment.sun_color = glm::max(sun_color, glm::vec3(0.0f));
  environment.skylight_intensity = std::max(0.0f, sky_light_intensity);
  environment.ambient_light_intensity = std::max(0.0f, ambient_light_intensity);
  environment.gamma = std::max(0.01f, gamma);
  return true;
#else
  return false;
#endif
}

bool PyEvoEngine::ConfigureCurrentSceneCudaEnvironmentMapForCapture(
    const std::filesystem::path& hdr_path, const float intensity, const float rotation_degrees, const float gamma) {
#ifdef CUDA_MODULE_SERVICE
  auto& application = ApplicationContext::Get();
  const auto scene = application.GetActiveScene();
  const auto layer = application.GetLayer<RayTracerLayer>();
  if (!scene || !layer || hdr_path.extension() != ".hdr" || !std::isfinite(intensity) || intensity < 0.0f ||
      !std::isfinite(rotation_degrees) || !std::isfinite(gamma) || gamma <= 0.0f) {
    return false;
  }
  const auto texture = AssetManager::CreateTemporaryAsset<Texture2D>();
  if (!texture->Import(std::filesystem::absolute(hdr_path))) return false;
  const auto map = AssetManager::CreateTemporaryAsset<EnvironmentalMap>();
  map->ConstructFromTexture2D(texture);
  if (!map->environment_cubemap.Get<Cubemap>()) return false;

  // Keep the source separate from baked reflection probes; the photo already contains the sun.
  const auto lighting = AssetManager::CreateTemporaryAsset<EnvironmentalLighting>();
  lighting->indirect_environment_source.kind = EnvironmentalLighting::IndirectEnvironmentSourceKind::EnvironmentalMap;
  lighting->indirect_environment_source.environmental_map = map;
  lighting->indirect_environment_source.rotation = glm::radians(rotation_degrees);
  lighting->indirect_environment_source.gamma = gamma;
  lighting->environment_lighting_intensity = intensity;
  lighting->diffuse_fallback_intensity = 0.0f;
  scene->environmental_lighting = lighting;
  layer->environment_properties.environmental_lighting_type = EnvironmentalLightingType::Scene;
  layer->environment_properties.sun_intensity = 0.0f;
  if (const auto camera = scene->main_camera.Get<Camera>()) {
    camera->camera_settings.background_source = CameraSettings::BackgroundSource::InheritEnvironmentalLighting;
    camera->camera_settings.gamma = gamma;
    camera->ResetFrameCount();
  }
  layer->UpdateScene(scene);
  return layer->environment_properties.use_environmental_map && layer->environment_properties.environmental_map != 0;
#else
  return false;
#endif
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
                    ", texture pending=" + std::to_string(TextureStorage::HasPendingUploads()) + ", asset queued=" +
                    std::to_string(snapshot.queued) + ", asset loading CPU=" + std::to_string(snapshot.loading_cpu) +
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
                  ", texture pending: " + std::to_string(TextureStorage::HasPendingUploads()) + ", asset queued: " +
                  std::to_string(snapshot.queued) + ", asset loading CPU: " + std::to_string(snapshot.loading_cpu) +
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

bool PyEvoEngine::CaptureCurrentSceneCuda(const int resolution_x, const int resolution_y,
                                          const std::filesystem::path& output_path, const int samples,
                                          const int bounces, const float gamma, const float denoiser_strength,
                                          const bool use_layer_clouds) {
#ifdef CUDA_MODULE_SERVICE
  if (resolution_x <= 0 || resolution_y <= 0 || samples <= 0 || bounces < 0 || gamma <= 0.0f ||
      !std::isfinite(denoiser_strength) || denoiser_strength < 0.0f || denoiser_strength > 1.0f) {
    return false;
  }
  auto& application = ApplicationContext::Get();
  const auto scene = application.GetActiveScene();
  const auto layer = application.GetLayer<RayTracerLayer>();
  const auto main_camera = scene ? scene->main_camera.Get<Camera>() : nullptr;
  if (!scene || !layer || !main_camera || !scene->IsEntityValid(main_camera->GetOwner())) {
    return false;
  }

  layer->UpdateScene(scene);
  if (CudaModule::GetRayTracer()->instances.empty()) {
    return false;
  }

  const Entity camera_entity = scene->CreateEntity("__PythonCudaCaptureCamera");
  scene->SetDataComponent(camera_entity, scene->GetDataComponent<GlobalTransform>(main_camera->GetOwner()));
  auto camera = scene->GetOrSetPrivateComponent<RayTracerCamera>(camera_entity).lock();
  if (!camera) {
    scene->DeleteEntity(camera_entity);
    return false;
  }
  camera->allow_auto_resize = false;
  camera->frame_size = glm::uvec2(resolution_x, resolution_y);
  camera->ApplyCameraSettings(main_camera->camera_settings);
  camera->ray_properties.samples = samples;
  camera->ray_properties.bounces = bounces;
  camera->SetGamma(gamma);
  camera->SetDenoiserStrength(denoiser_strength);
  camera->Render(camera->ray_properties, layer->environment_properties,
                 use_layer_clouds ? layer->viewport_clouds : VolumetricCloudSettings{});

  if (const auto parent = output_path.parent_path(); !parent.empty()) {
    std::filesystem::create_directories(parent);
  }
  camera->render_texture->StoreToPng(output_path);
  const bool success = std::filesystem::exists(output_path) && std::filesystem::file_size(output_path) > 0;
  camera.reset();
  scene->DeleteEntity(camera_entity);
  return success;
#else
  return false;
#endif
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
#ifdef CUDA_MODULE_SERVICE
  RegisterCloudBindings(m);
#endif

  m.def("RunWindowless", &RunWindowless);
  m.def("RunDemoWindowless", &RunDemoWindowless, py::arg("demo_setup_name"), py::arg("resource_folder_path"),
        py::arg("clear_generated_project_files") = true);
  m.def("ConfigureCurrentSceneCameraForCapture", &ConfigureCurrentSceneCameraForCapture, py::arg("render_mode"),
        py::arg("samples_per_frame"), py::arg("bounces"));
  m.def("ConfigureCurrentSceneOutdoorLightingForCapture", &ConfigureCurrentSceneOutdoorLightingForCapture,
        py::arg("sun_euler_degrees"), py::arg("sun_angular_diameter_radians"), py::arg("sun_intensity"),
        py::arg("sun_color"), py::arg("sky_light_intensity"), py::arg("ambient_light_intensity"),
        py::arg("background_color_linear"), py::arg("gamma"));
  m.def("ConfigureCurrentSceneCudaOutdoorLightingForCapture", &ConfigureCurrentSceneCudaOutdoorLightingForCapture,
        py::arg("sun_euler_degrees"), py::arg("sun_angular_diameter_radians"), py::arg("sun_intensity"),
        py::arg("sun_color"), py::arg("sky_light_intensity"), py::arg("ambient_light_intensity"), py::arg("gamma"));
  m.def("SetMainCameraLookAt", &SetMainCameraLookAt, py::arg("position"), py::arg("target"), py::arg("up"),
        py::arg("fov_degrees"));
  m.def("ConfigureCurrentSceneCudaEnvironmentMapForCapture", &ConfigureCurrentSceneCudaEnvironmentMapForCapture,
        py::arg("hdr_path"), py::arg("intensity") = 1.0f, py::arg("rotation_degrees") = 0.0f, py::arg("gamma") = 2.2f);
  m.def("WaitForCurrentSceneReady", &WaitForCurrentSceneReady, py::arg("maximum_frames") = 3000);
  m.def("CaptureCurrentScene", &CaptureCurrentScene, py::arg("resolution_x"), py::arg("resolution_y"),
        py::arg("output_path"), py::arg("warmup_frames") = 1, py::arg("require_accumulated_frames") = false);
  m.def("CaptureCurrentSceneCuda", &CaptureCurrentSceneCuda, py::arg("resolution_x"), py::arg("resolution_y"),
        py::arg("output_path"), py::arg("samples") = 64, py::arg("bounces") = 4, py::arg("gamma") = 2.2f,
        py::arg("denoiser_strength") = 0.0f, py::kw_only(), py::arg("use_layer_clouds") = false);
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
  m.def("DeleteCurrentSceneEntitiesByName", &DeleteCurrentSceneEntitiesByName);
  m.def("GetCurrentSceneEntityCountByName", &GetCurrentSceneEntityCountByName);
  m.def("SaveCurrentSceneAs", &SaveCurrentSceneAs);
}

void PyEvoEngine::RegisterFieldIlluminationBindings(pybind11::module& m) {
  py::class_<PrefabFieldMeasurementRegion>(m, "PrefabFieldMeasurementRegion", py::module_local())
      .def_readonly("row_start", &PrefabFieldMeasurementRegion::row_start)
      .def_readonly("column_start", &PrefabFieldMeasurementRegion::column_start)
      .def_readonly("rows", &PrefabFieldMeasurementRegion::rows)
      .def_readonly("columns", &PrefabFieldMeasurementRegion::columns);

  py::class_<SurfaceIlluminationResult>(m, "SurfaceIlluminationResult", py::module_local())
      .def_readonly("total_area", &SurfaceIlluminationResult::total_area)
      .def_readonly("total_flux", &SurfaceIlluminationResult::total_flux)
      .def_readonly("average_flux", &SurfaceIlluminationResult::average_flux);

  py::class_<PrefabFieldIlluminationResult>(m, "PrefabFieldIlluminationResult", py::module_local())
      .def_readonly("model_index", &PrefabFieldIlluminationResult::model_index)
      .def_readonly("asset_index", &PrefabFieldIlluminationResult::asset_index)
      .def_readonly("row", &PrefabFieldIlluminationResult::row)
      .def_readonly("column", &PrefabFieldIlluminationResult::column)
      .def_readonly("total_area", &PrefabFieldIlluminationResult::total_area)
      .def_readonly("total_flux", &PrefabFieldIlluminationResult::total_flux)
      .def_readonly("average_flux", &PrefabFieldIlluminationResult::average_flux)
      .def_readonly("organs", &PrefabFieldIlluminationResult::organs);

  py::class_<PrefabFieldIlluminationSummary>(m, "PrefabFieldIlluminationSummary", py::module_local())
      .def_readonly("models", &PrefabFieldIlluminationSummary::models)
      .def_readonly("total_area", &PrefabFieldIlluminationSummary::total_area)
      .def_readonly("total_flux", &PrefabFieldIlluminationSummary::total_flux)
      .def_readonly("area_weighted_average_flux", &PrefabFieldIlluminationSummary::area_weighted_average_flux)
      .def_readonly("mean_model_average_flux", &PrefabFieldIlluminationSummary::mean_model_average_flux)
      .def_readonly("organs", &PrefabFieldIlluminationSummary::organs);

  m.def("CreatePrefabField", &CreatePrefabField, py::arg("asset_folder"), py::arg("rows"), py::arg("columns"),
        py::arg("column_spacing"), py::arg("row_spacing"), py::arg("seed"), py::arg("rotation_x_degrees"),
        py::arg("rotation_y_degrees"), py::arg("rotation_z_degrees"), py::arg("uniform_scale"),
        py::arg("random_yaw_degrees") = 0.0f, py::arg("recursive") = false, py::arg("roi") = 1.0f);
  m.def("ConfigurePrefabField", &ConfigurePrefabField, py::arg("field_entity"), py::arg("asset_folder"),
        py::arg("rows"), py::arg("columns"), py::arg("column_spacing"), py::arg("row_spacing"), py::arg("seed"),
        py::arg("rotation_x_degrees"), py::arg("rotation_y_degrees"), py::arg("rotation_z_degrees"),
        py::arg("uniform_scale"), py::arg("random_yaw_degrees") = 0.0f, py::arg("recursive") = false,
        py::arg("roi") = 1.0f);
  m.def("RegeneratePrefabField", &RegeneratePrefabField);
  m.def("GetPrefabFieldModelCount", &GetPrefabFieldModelCount);
  m.def("GetPrefabFieldMeasurementRegion", &GetPrefabFieldMeasurementRegion);
  m.def("ClearPrefabField", &ClearPrefabField);
  m.def("CreateVoxelIlluminationEstimator", &CreateVoxelIlluminationEstimator, py::arg("name") = "Voxel illumination",
        py::arg("center") = std::array<float, 3>{0, 0, 0}, py::arg("rotation_degrees") = std::array<float, 3>{0, 0, 0});
  m.def("EstimateVoxelIllumination", &EstimateVoxelIllumination, py::arg("entity"), py::arg("volume_size"),
        py::arg("resolution"), py::arg("samples_per_voxel") = 4, py::arg("rays_per_point") = 64, py::arg("bounces") = 4,
        py::arg("seed") = 0, py::arg("push_normal_distance") = 0.0f);
  m.def("EstimatePrefabFieldIllumination", &EstimatePrefabFieldIllumination, py::arg("field_entity"),
        py::arg("samples") = 64, py::arg("bounces") = 4, py::arg("seed") = 0,
        py::arg("push_normal_distance") = 0.0001f);
  m.def("EstimatePrefabFieldIlluminationSummary", &EstimatePrefabFieldIlluminationSummary, py::arg("field_entity"),
        py::arg("samples") = 64, py::arg("bounces") = 4, py::arg("seed") = 0,
        py::arg("push_normal_distance") = 0.0001f);
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
  // Headless scene jobs should load the requested scene dependency graph, not unrelated project history.
  application_info.startup_scene_path = project_relative_path;
  application_info.load_project_assets = false;
  ApplicationContext::Get().Initialize(application_info);
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

size_t PyEvoEngine::DeleteCurrentSceneEntitiesByName(const std::string& name) {
  const auto scene = ApplicationContext::Get().GetActiveScene();
  if (!scene) {
    return 0;
  }
  std::vector<Entity> entities;
  scene->GetAllEntities(entities);
  std::vector<Entity> matches;
  for (const auto entity : entities) {
    if (scene->IsEntityValid(entity) && scene->GetEntityName(entity) == name) {
      matches.emplace_back(entity);
    }
  }
  for (const auto entity : matches) {
    if (scene->IsEntityValid(entity)) {
      scene->DeleteEntity(entity);
    }
  }
  return matches.size();
}

size_t PyEvoEngine::GetCurrentSceneEntityCountByName(const std::string& name) {
  const auto scene = ApplicationContext::Get().GetActiveScene();
  if (!scene) {
    return 0;
  }
  std::vector<Entity> entities;
  scene->GetAllEntities(entities);
  return std::count_if(entities.begin(), entities.end(), [&](const Entity entity) {
    return scene->IsEntityValid(entity) && scene->GetEntityName(entity) == name;
  });
}

size_t PyEvoEngine::ConfigurePrefabField(const Entity& field_entity, const std::filesystem::path& asset_folder,
                                         const int rows, const int columns, const float column_spacing,
                                         const float row_spacing, const uint32_t seed, const float rotation_x_degrees,
                                         const float rotation_y_degrees, const float rotation_z_degrees,
                                         const float uniform_scale, const float random_yaw_degrees,
                                         const bool recursive, const float roi) {
  const auto scene = ApplicationContext::Get().GetActiveScene();
  if (!scene || !scene->IsEntityValid(field_entity) || asset_folder.empty() || asset_folder.is_absolute() ||
      rows <= 0 || columns <= 0 || column_spacing < 0.0f || row_spacing < 0.0f || uniform_scale <= 0.0f ||
      random_yaw_degrees < 0.0f || !std::isfinite(roi) || roi <= 0.0f || roi > 1.0f) {
    return 0;
  }
  const auto field = scene->GetOrSetPrivateComponent<PrefabField>(field_entity).lock();
  field->asset_folder = asset_folder;
  field->rows = rows;
  field->columns = columns;
  field->spacing = glm::vec2(column_spacing, row_spacing);
  field->seed = seed;
  field->model_rotation_degrees = glm::vec3(rotation_x_degrees, rotation_y_degrees, rotation_z_degrees);
  field->model_scale = glm::vec3(uniform_scale);
  field->random_yaw_degrees = random_yaw_degrees;
  field->recursive = recursive;
  field->illumination_roi = roi;
  return field->GenerateField().size();
}

Entity PyEvoEngine::CreatePrefabField(const std::filesystem::path& asset_folder, const int rows, const int columns,
                                      const float column_spacing, const float row_spacing, const uint32_t seed,
                                      const float rotation_x_degrees, const float rotation_y_degrees,
                                      const float rotation_z_degrees, const float uniform_scale,
                                      const float random_yaw_degrees, const bool recursive, const float roi) {
  const auto scene = ApplicationContext::Get().GetActiveScene();
  if (!scene) {
    return {};
  }
  const auto entity = scene->CreateEntity("Prefab Field");
  if (ConfigurePrefabField(entity, asset_folder, rows, columns, column_spacing, row_spacing, seed, rotation_x_degrees,
                           rotation_y_degrees, rotation_z_degrees, uniform_scale, random_yaw_degrees, recursive,
                           roi) == 0) {
    scene->DeleteEntity(entity);
    return {};
  }
  return entity;
}

size_t PyEvoEngine::RegeneratePrefabField(const Entity& field_entity) {
  const auto scene = ApplicationContext::Get().GetActiveScene();
  if (!scene || !scene->IsEntityValid(field_entity) || !scene->HasPrivateComponent<PrefabField>(field_entity)) {
    return 0;
  }
  return scene->GetOrSetPrivateComponent<PrefabField>(field_entity).lock()->GenerateField().size();
}

size_t PyEvoEngine::GetPrefabFieldModelCount(const Entity& field_entity) {
  const auto scene = ApplicationContext::Get().GetActiveScene();
  if (!scene || !scene->IsEntityValid(field_entity) || !scene->HasPrivateComponent<PrefabField>(field_entity)) {
    return 0;
  }
  return scene->GetOrSetPrivateComponent<PrefabField>(field_entity).lock()->GetGeneratedEntities().size();
}

PrefabFieldMeasurementRegion PyEvoEngine::GetPrefabFieldMeasurementRegion(const Entity& field_entity) {
  const auto scene = ApplicationContext::Get().GetActiveScene();
  if (!scene || !scene->IsEntityValid(field_entity) || !scene->HasPrivateComponent<PrefabField>(field_entity))
    return {};
  return scene->GetOrSetPrivateComponent<PrefabField>(field_entity).lock()->GetMeasurementRegion();
}

void PyEvoEngine::ClearPrefabField(const Entity& field_entity) {
  const auto scene = ApplicationContext::Get().GetActiveScene();
  if (scene && scene->IsEntityValid(field_entity) && scene->HasPrivateComponent<PrefabField>(field_entity)) {
    scene->GetOrSetPrivateComponent<PrefabField>(field_entity).lock()->ClearField();
  }
}

Entity PyEvoEngine::CreateVoxelIlluminationEstimator(const std::string& name, const std::array<float, 3>& center,
                                                     const std::array<float, 3>& rotation_degrees) {
#ifdef CUDA_MODULE_SERVICE
  const auto scene = ApplicationContext::Get().GetActiveScene();
  if (!scene || !ApplicationContext::Get().GetLayer<RayTracerLayer>())
    throw std::runtime_error("Load a scene with PushRayTracerLayer() before creating a voxel estimator.");
  for (int i = 0; i < 3; ++i)
    if (!std::isfinite(center[i]) || !std::isfinite(rotation_degrees[i]))
      throw py::value_error("Center and rotation must be finite.");
  const auto entity = scene->CreateEntity(name);
  Transform transform;
  transform.SetValue(glm::vec3(center[0], center[1], center[2]),
                     glm::quat(glm::radians(glm::vec3(rotation_degrees[0], rotation_degrees[1], rotation_degrees[2]))),
                     glm::vec3(1.0f));
  scene->SetDataComponent(entity, transform);
  scene->GetOrSetPrivateComponent<VoxelIlluminationEstimator>(entity);
  TransformGraph::CalculateTransformGraphs(scene, false);
  return entity;
#else
  throw std::runtime_error("This Python module was built without CUDA support.");
#endif
}

py::dict PyEvoEngine::EstimateVoxelIllumination(const Entity& entity, const std::array<float, 3>& volume_size,
                                                const std::array<int, 3>& resolution, const int samples_per_voxel,
                                                const int rays_per_point, const int bounces, const int seed,
                                                const float push_normal_distance) {
#ifdef CUDA_MODULE_SERVICE
  const auto scene = ApplicationContext::Get().GetActiveScene();
  const auto layer = ApplicationContext::Get().GetLayer<RayTracerLayer>();
  if (!scene || !layer || !scene->IsEntityValid(entity) ||
      !scene->HasPrivateComponent<VoxelIlluminationEstimator>(entity))
    throw py::value_error("A valid voxel estimator and RayTracerLayer are required.");
  if (samples_per_voxel <= 0 || rays_per_point <= 0 || bounces < 0 || !std::isfinite(push_normal_distance))
    throw py::value_error("Sample counts must be positive, bounces non-negative, and normal distance finite.");
  size_t point_count = samples_per_voxel;
  for (int i = 0; i < 3; ++i) {
    if (!std::isfinite(volume_size[i]) || volume_size[i] <= 0 || resolution[i] <= 0)
      throw py::value_error("Volume size must be finite and positive; resolution must be positive.");
    if (point_count > 1000000 / static_cast<size_t>(resolution[i]))
      throw py::value_error("The voxel estimator supports at most 1,000,000 virtual PAR points.");
    point_count *= resolution[i];
  }
  const auto estimator = scene->GetOrSetPrivateComponent<VoxelIlluminationEstimator>(entity).lock();
  estimator->volume_size = glm::vec3(volume_size[0], volume_size[1], volume_size[2]);
  estimator->voxel_resolution = glm::ivec3(resolution[0], resolution[1], resolution[2]);
  estimator->samples_per_voxel = samples_per_voxel;
  estimator->ray_properties = {bounces, rays_per_point};
  estimator->seed = seed;
  estimator->push_normal_distance = push_normal_distance;
  TransformGraph::CalculateTransformGraphs(scene, false);
  layer->UpdateScene(scene);
  estimator->Estimate();
  const auto& voxels = estimator->PeekVoxels();
  if (voxels.size() != point_count / samples_per_voxel)
    throw std::runtime_error("Voxel estimation did not return the requested grid.");

  // Own all memory: later estimates, entity deletion and engine teardown must not invalidate Python results.
  const std::vector<py::ssize_t> shape{resolution[0], resolution[1], resolution[2]};
  const std::vector<py::ssize_t> vector_shape{resolution[0], resolution[1], resolution[2], 3};
  py::array_t<float> energy(vector_shape), centers(vector_shape), intensity(shape);
  py::array_t<uint32_t> counts(shape);
  for (size_t i = 0; i < voxels.size(); ++i) {
    for (int axis = 0; axis < 3; ++axis) {
      energy.mutable_data()[i * 3 + axis] = voxels[i].energy[axis];
      centers.mutable_data()[i * 3 + axis] = voxels[i].center[axis];
    }
    intensity.mutable_data()[i] = voxels[i].GetIntensity();
    counts.mutable_data()[i] = voxels[i].sample_count;
  }
  const auto transform = scene->GetDataComponent<GlobalTransform>(entity);
  py::array_t<float> matrix(std::vector<py::ssize_t>{4, 4});
  auto matrix_values = matrix.mutable_unchecked<2>();
  for (int row = 0; row < 4; ++row)
    for (int col = 0; col < 4; ++col)
      matrix_values(row, col) = transform.value[col][row];
  const auto vector = [](const glm::vec3& v) {
    return std::array<float, 3>{v.x, v.y, v.z};
  };
  const auto& env = layer->environment_properties;
  py::dict lighting;
  lighting["mode"] = static_cast<int>(env.environmental_lighting_type);
  lighting["use_environmental_map"] = env.use_environmental_map;
  lighting["skylight_intensity"] = env.skylight_intensity;
  lighting["ambient_light_intensity"] = env.ambient_light_intensity;
  lighting["environment_rotation_radians"] = env.environment_rotation;
  lighting["gamma"] = env.gamma;
  lighting["color"] = vector(env.color);
  lighting["sun_direction"] = vector(env.sun_direction);
  lighting["sun_color"] = vector(env.sun_color);
  lighting["sun_intensity"] = env.sun_intensity;
  lighting["sun_angular_diameter_radians"] = env.sun_angular_diameter_radians;
  py::dict result;
  result["energy"] = energy;
  result["intensity"] = intensity;
  result["centers_local"] = centers;
  result["sample_count"] = counts;
  result["local_to_world"] = matrix;
  result["resolution"] = resolution;
  result["volume_size"] = volume_size;
  result["sensor_normal_world"] = vector(glm::normalize(transform.GetRotation() * glm::vec3(0, 1, 0)));
  result["samples_per_voxel"] = samples_per_voxel;
  result["rays_per_point"] = rays_per_point;
  result["bounces"] = bounces;
  result["seed"] = seed;
  result["push_normal_distance"] = push_normal_distance;
  result["lighting"] = lighting;
  result["axis_order"] = "xyz; C order; z fastest";
  result["intensity_definition"] = "length(mean RGB energy); uncalibrated engine units";
  return result;
#else
  throw std::runtime_error("This Python module was built without CUDA support.");
#endif
}

std::vector<PrefabFieldIlluminationResult> PyEvoEngine::EstimatePrefabFieldIllumination(
    const Entity& field_entity, const int samples, const int bounces, const int seed,
    const float push_normal_distance) {
  return EstimatePrefabFieldIlluminationSummary(field_entity, samples, bounces, seed, push_normal_distance).models;
}

PrefabFieldIlluminationSummary PyEvoEngine::EstimatePrefabFieldIlluminationSummary(const Entity& field_entity,
                                                                                   const int samples, const int bounces,
                                                                                   const int seed,
                                                                                   const float push_normal_distance) {
#ifdef CUDA_MODULE_SERVICE
  const auto scene = ApplicationContext::Get().GetActiveScene();
  if (!scene || !scene->IsEntityValid(field_entity) || samples <= 0 || bounces < 0 || push_normal_distance < 0.0f ||
      !scene->HasPrivateComponent<PrefabField>(field_entity)) {
    return {};
  }
  const auto field = scene->GetOrSetPrivateComponent<PrefabField>(field_entity).lock();
  return evo_engine::EstimatePrefabFieldIllumination(
      *field, PrefabFieldIlluminationSettings{samples, bounces, seed, push_normal_distance});
#else
  return {};
#endif
}

bool PyEvoEngine::SaveCurrentSceneAs(const std::filesystem::path& project_relative_path) {
  const auto scene = ApplicationContext::Get().GetActiveScene();
  if (!scene || project_relative_path.empty() || project_relative_path.is_absolute() ||
      project_relative_path.extension() != ".evescene") {
    return false;
  }
  if (std::filesystem::exists(ProjectManager::GetAssetsFolderPath() / project_relative_path)) {
    const auto destination = std::dynamic_pointer_cast<Scene>(ProjectManager::GetOrCreateAsset(project_relative_path));
    if (!destination) {
      return false;
    }
    if (destination != scene) {
      Scene::Clone(scene, destination);
    }
    return destination->Save();
  }
  const auto copy = AssetManager::CreateTemporaryAsset<Scene>();
  Scene::Clone(scene, copy);
  ProjectManager::GetOrCreateFolder(project_relative_path.parent_path());
  return copy->SetPathAndSave(project_relative_path);
}
