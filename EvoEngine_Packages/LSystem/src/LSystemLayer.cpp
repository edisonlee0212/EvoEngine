#include "LSystemLayer.hpp"

#include "Application.hpp"
#include "AssetManager.hpp"
#include "Camera.hpp"
#include "EditorLayer.hpp"
#include "LSystemInspectionAdapters.hpp"
#include "LSystemSerializationAdapters.hpp"
#include "PathUtils.hpp"
#include "Platform.hpp"
#include "ProjectManager.hpp"
#include "Scene.hpp"
#include "ScotsPine.hpp"
#include "ScotsPineBatchContract.hpp"
#include "ScotsPineDescriptor.hpp"
#include "ScotsPineTemporalGrowth.hpp"
#include "Serialization.hpp"
#include "Texture2D.hpp"
#include "Times.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <future>
#include <iomanip>
#include <sstream>
#include <system_error>

#if defined(_WIN32)
#  include <Windows.h>
#  include <Shellapi.h>
#endif

using namespace l_system_package;
using namespace evo_engine;

namespace {

constexpr float kCalendarDaysPerYear = 365.0f;
constexpr int kFullFidelityWidth = 1372;
constexpr int kFullFidelityHeight = 1040;
constexpr int kFullFidelityJpgQuality = 95;
constexpr int kFullFidelityPngCompressionLevel = 1;
constexpr int kFullFidelityWriterThreads = 2;
constexpr int kFullFidelityProfileWarmupDatapoints = 1;
constexpr float kFullFidelityPlantBlurRadiusPx = 1.25f;
constexpr float kFullFidelityMaxTargetGdd = 4500.0f;
constexpr const char* kFullFidelityOutputName = "editor_preview";
constexpr const char* kFullFidelityRenderMode = "rasterization";
constexpr const char* kFullFidelityRgbFormat = "jpg";

template <typename Selector>
double AverageProfileMetric(const std::vector<LSystemLayer::ProfileFrame>& frames, Selector selector) {
  if (frames.empty()) {
    return 0.0;
  }
  double sum = 0.0;
  for (const auto& frame : frames) {
    sum += selector(frame);
  }
  return sum / static_cast<double>(frames.size());
}

template <typename Selector>
double MaxProfileMetric(const std::vector<LSystemLayer::ProfileFrame>& frames, Selector selector) {
  double max_value = 0.0;
  for (const auto& frame : frames) {
    max_value = std::max(max_value, selector(frame));
  }
  return max_value;
}

int ClampColorModeIndex(const int mode) {
  return std::clamp(mode, 0, 7);
}

int ResolveEffectiveColorMode(const int selected_mode, const bool scene_plant_view_tint_enabled) {
  return scene_plant_view_tint_enabled ? ClampColorModeIndex(selected_mode) : 0;
}

void ApplyGlobalPlantColorMode(const int selected_mode, const bool scene_plant_view_tint_enabled) {
  const int effective_mode = ResolveEffectiveColorMode(selected_mode, scene_plant_view_tint_enabled);
  ScotsPine::SetGlobalColorMode(static_cast<ScotsPine::ColorMode>(effective_mode));
}

float ClampDayOfYear(const float day) {
  if (!std::isfinite(day)) {
    return 0.0f;
  }
  return std::clamp(day, 0.0f, kCalendarDaysPerYear - 0.001f);
}

double CalendarAbsoluteDay(const int year, const float day_of_year) {
  return static_cast<double>(std::max(0, year)) * static_cast<double>(kCalendarDaysPerYear) +
         static_cast<double>(ClampDayOfYear(day_of_year));
}

void SetCalendarFromAbsoluteDay(LSystemLayer& layer, double absolute_day) {
  absolute_day = std::max(0.0, absolute_day);
  layer.simulation_year = static_cast<int>(std::floor(absolute_day / static_cast<double>(kCalendarDaysPerYear)));
  const double day = absolute_day - static_cast<double>(layer.simulation_year) * static_cast<double>(kCalendarDaysPerYear);
  layer.simulation_day_of_year = ClampDayOfYear(static_cast<float>(day));
}

bool IsCalendarAtEnd(const LSystemLayer& layer) {
  return CalendarAbsoluteDay(layer.simulation_year, layer.simulation_day_of_year) >=
         CalendarAbsoluteDay(layer.calendar_end_year, layer.calendar_end_day_of_year) - 1.0e-5;
}

void ClampCalendarToEnd(LSystemLayer& layer) {
  const double current = CalendarAbsoluteDay(layer.simulation_year, layer.simulation_day_of_year);
  const double end = CalendarAbsoluteDay(layer.calendar_end_year, layer.calendar_end_day_of_year);
  if (current > end) {
    SetCalendarFromAbsoluteDay(layer, end);
  }
}

float AdvanceLayerCalendar(LSystemLayer& layer, const float requested_delta_days, bool& reached_end) {
  reached_end = false;
  const float safe_delta = std::max(0.0f, requested_delta_days);
  if (safe_delta <= 0.0f) {
    ClampCalendarToEnd(layer);
    reached_end = IsCalendarAtEnd(layer);
    return 0.0f;
  }

  const double current = CalendarAbsoluteDay(layer.simulation_year, layer.simulation_day_of_year);
  double target = current + static_cast<double>(safe_delta);
  const double end = CalendarAbsoluteDay(layer.calendar_end_year, layer.calendar_end_day_of_year);
  if (current >= end) {
    target = end;
    reached_end = true;
  } else if (target >= end) {
    target = end;
    reached_end = true;
  }
  SetCalendarFromAbsoluteDay(layer, target);
  return static_cast<float>(std::max(0.0, target - current));
}

glm::vec4 ComputeSeasonalTintColor(const float day_of_year, const std::array<glm::vec4, 4>& colors) {
  const float phase = ClampDayOfYear(day_of_year) / kCalendarDaysPerYear * 4.0f;
  const int lower_index = static_cast<int>(std::floor(phase)) % 4;
  const int upper_index = (lower_index + 1) % 4;
  const float t = phase - static_cast<float>(lower_index);
  return glm::mix(colors[static_cast<size_t>(lower_index)], colors[static_cast<size_t>(upper_index)], t);
}

void ApplyGlobalSeasonalPlantTint(const LSystemLayer& layer) {
  ScotsPine::SeasonalColorTint tint{};
  tint.enabled = layer.seasonal_plant_tint_enabled;
  tint.color = ComputeSeasonalTintColor(layer.simulation_day_of_year, layer.seasonal_tint_colors);
  tint.strength = layer.seasonal_plant_tint_strength;
  ScotsPine::SetGlobalSeasonalColorTint(tint);
}

void RebuildAllPlantGeometry(const std::shared_ptr<Scene>& scene) {
  if (!scene) {
    return;
  }

  if (const auto* pine_entities_ptr = scene->UnsafeGetPrivateComponentOwnersList<ScotsPine>()) {
    const std::vector<Entity> pine_entities = *pine_entities_ptr;
    for (const auto& entity : pine_entities) {
      if (!scene->IsEntityValid(entity)) {
        continue;
      }
      if (const auto pine = scene->GetOrSetPrivateComponent<ScotsPine>(entity).lock()) {
        if (pine->growth_model.IsInitialized()) {
          pine->RebuildGeometry();
        } else {
          pine->GenerateGeometryEntities();
        }
      }
    }
  }
}

std::string EscapeProfileJson(const std::string& value) {
  std::string escaped;
  escaped.reserve(value.size());
  for (const char ch : value) {
    switch (ch) {
      case '\\':
        escaped += "\\\\";
        break;
      case '"':
        escaped += "\\\"";
        break;
      case '\n':
        escaped += "\\n";
        break;
      case '\r':
        escaped += "\\r";
        break;
      case '\t':
        escaped += "\\t";
        break;
      default:
        escaped += ch;
        break;
    }
  }
  return escaped;
}

std::string BuildProfileStamp() {
  const auto stamp =
      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch())
          .count();
  return std::to_string(stamp);
}

std::string BuildPreviewStamp() {
  const std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  std::tm local_time{};
#if defined(_WIN32)
  localtime_s(&local_time, &now);
#else
  localtime_r(&now, &local_time);
#endif
  std::ostringstream stamp;
  stamp << std::put_time(&local_time, "%Y%m%d_%H%M%S");
  return stamp.str();
}

template <size_t BufferSize>
void CopyStringToBuffer(const std::string& value, std::array<char, BufferSize>& buffer) {
  buffer.fill('\0');
  if constexpr (BufferSize > 0) {
    std::strncpy(buffer.data(), value.c_str(), BufferSize - 1);
    buffer[BufferSize - 1] = '\0';
  }
}

template <size_t BufferSize>
std::string BufferToString(const std::array<char, BufferSize>& buffer) {
  return std::string(buffer.data());
}

const char* BoolString(const bool value) {
  return value ? "true" : "false";
}

std::filesystem::path ResolveCurrentExecutablePath() {
#if defined(_WIN32)
  std::vector<wchar_t> buffer(MAX_PATH);
  while (true) {
    const DWORD copied = GetModuleFileNameW(nullptr, buffer.data(), static_cast<DWORD>(buffer.size()));
    if (copied == 0) {
      return {};
    }
    if (copied < buffer.size() - 1) {
      return std::filesystem::path(std::wstring(buffer.data(), copied));
    }
    buffer.resize(buffer.size() * 2);
  }
#else
  std::error_code ec;
  const auto path = std::filesystem::read_symlink("/proc/self/exe", ec);
  return ec ? std::filesystem::path{} : path;
#endif
}

std::filesystem::path FindCheckoutRootFrom(const std::filesystem::path& start_path) {
  if (start_path.empty()) {
    return {};
  }
  const auto package_marker = path_utils::FindAncestorChildPath("EvoEngine_Packages", start_path, 12);
  if (package_marker.empty()) {
    return {};
  }
  const auto root = package_marker.parent_path();
  std::error_code error;
  return std::filesystem::exists(root / "Resources", error) ? root : std::filesystem::path{};
}

std::filesystem::path ResolveEvoEngineCheckoutRoot() {
  std::vector<std::filesystem::path> candidates;
  const auto project_path = ProjectManager::GetProjectPath();
  if (!project_path.empty()) {
    path_utils::AddUniqueNormalizedPath(candidates, project_path.parent_path());
  }
  const auto assets_path = ProjectManager::GetAssetsFolderPath();
  if (!assets_path.empty()) {
    path_utils::AddUniqueNormalizedPath(candidates, assets_path);
  }
  const auto executable_path = ResolveCurrentExecutablePath();
  if (!executable_path.empty()) {
    path_utils::AddUniqueNormalizedPath(candidates, executable_path.parent_path());
  }
  path_utils::AddUniqueNormalizedPath(candidates, std::filesystem::current_path());

  for (const auto& candidate : candidates) {
    if (const auto root = FindCheckoutRootFrom(candidate); !root.empty()) {
      return root;
    }
  }
  return path_utils::NormalizeAbsolutePath(std::filesystem::current_path());
}

std::filesystem::path DefaultFullFidelityOutputRoot() {
  return ResolveEvoEngineCheckoutRoot() / "output" / "editor_previews";
}

std::filesystem::path FullFidelityBackgroundDirectory() {
  return ResolveEvoEngineCheckoutRoot() / "Resources" / "ScotsPineSynthetic" / "Backgrounds";
}

std::filesystem::path FullFidelityBackgroundImage() {
  return FullFidelityBackgroundDirectory() / "extracted_side_view.png";
}

std::filesystem::path ResolveScotsPineRendererExecutable() {
  const auto current_executable = ResolveCurrentExecutablePath();
  if (current_executable.empty()) {
    return {};
  }

  if (current_executable.stem().string().find("ScotsPineDataGeneratorApp") != std::string::npos) {
    return current_executable;
  }

#if defined(_WIN32)
  const auto sibling = current_executable.parent_path() / "ScotsPineDataGeneratorApp.exe";
#else
  const auto sibling = current_executable.parent_path() / "ScotsPineDataGeneratorApp";
#endif
  std::error_code ec;
  return std::filesystem::exists(sibling, ec) ? sibling : current_executable;
}

bool OpenFolderInExplorer(const std::filesystem::path& folder_path) {
  if (folder_path.empty()) {
    return false;
  }
#if defined(_WIN32)
  const auto normalized = path_utils::NormalizeAbsolutePath(folder_path).wstring();
  const auto result = ShellExecuteW(nullptr, L"open", normalized.c_str(), nullptr, nullptr, SW_SHOWNORMAL);
  return reinterpret_cast<std::intptr_t>(result) > 32;
#else
  return false;
#endif
}

std::string QuoteCommandArgument(const std::filesystem::path& path) {
  std::string text = path.string();
  std::string quoted;
  quoted.reserve(text.size() + 2);
  quoted.push_back('"');
  for (const char ch : text) {
    if (ch == '"') {
      quoted += "\\\"";
    } else {
      quoted.push_back(ch);
    }
  }
  quoted.push_back('"');
  return quoted;
}

std::string BuildSyntheticRendererCommand(const std::filesystem::path& renderer_executable,
                                          const std::filesystem::path& config_path) {
  const auto renderer_directory = renderer_executable.parent_path();
#if defined(_WIN32)
  return "cd /d " + QuoteCommandArgument(renderer_directory) + " && " + QuoteCommandArgument(renderer_executable) +
         " --synthetic-config " + QuoteCommandArgument(config_path);
#else
  return "cd " + QuoteCommandArgument(renderer_directory) + " && " + QuoteCommandArgument(renderer_executable) +
         " --synthetic-config " + QuoteCommandArgument(config_path);
#endif
}

std::filesystem::path ResolveActiveSceneAssetPath(const std::shared_ptr<Scene>& scene) {
  if (!scene || scene->IsTemporary()) {
    return {};
  }
  return scene->GetAssetsFolderRelativePath();
}

std::filesystem::path ResolveFirstScotsPineDescriptorPath(const std::shared_ptr<Scene>& scene) {
  if (!scene) {
    return {};
  }
  const auto* pine_entities_ptr = scene->UnsafeGetPrivateComponentOwnersList<ScotsPine>();
  if (!pine_entities_ptr) {
    return {};
  }
  for (const auto& entity : *pine_entities_ptr) {
    if (!scene->IsEntityValid(entity)) {
      continue;
    }
    const auto pine = scene->GetOrSetPrivateComponent<ScotsPine>(entity).lock();
    if (!pine) {
      continue;
    }
    const auto descriptor = pine->descriptor_ref.Get<ScotsPineDescriptor>();
    if (descriptor && !descriptor->IsTemporary()) {
      const auto path = descriptor->GetAssetsFolderRelativePath();
      if (!path.empty()) {
        return path;
      }
    }
  }
  return {};
}

bool WriteSyntheticOptionsConfig(const ScotsPineSyntheticRenderOptions& options, const std::filesystem::path& config_path,
                                 std::string& error) {
  if (!config_path.parent_path().empty()) {
    std::filesystem::create_directories(config_path.parent_path());
  }

  std::ofstream out(config_path, std::ios::trunc);
  if (!out.is_open()) {
    error = "Failed to create synthetic config: " + config_path.string();
    return false;
  }

  out << "project_path=" << options.project_path.string() << '\n';
  out << "scene_path=" << options.scene_path.string() << '\n';
  out << "descriptor_path=" << options.descriptor_path.string() << '\n';
  out << "output_root=" << options.output_root.string() << '\n';
  out << "output_name=" << options.output_name << '\n';
  out << "background_image=" << options.background_image.string() << '\n';
  out << "background_dir=" << options.background_dir.string() << '\n';
  out << "scene_pine_growth_mode=" << options.scene_pine_growth_mode << '\n';
  out << "render_mode=" << options.render_mode << '\n';
  out << "rgb_output_format=" << options.rgb_output_format << '\n';
  out << "base_seed=" << options.base_seed << '\n';
  out << "seed_a=" << options.seed_a << '\n';
  out << "seed_b=" << options.seed_b << '\n';
  out << "seed_c=" << options.seed_c << '\n';
  out << "sample_count=" << options.sample_count << '\n';
  out << "start_index=" << options.start_index << '\n';
  out << "worker_id=" << options.worker_id << '\n';
  out << "worker_count=" << options.worker_count << '\n';
  out << "frame_count=" << options.frame_count << '\n';
  out << "width=" << options.width << '\n';
  out << "height=" << options.height << '\n';
  out << "profile_warmup_datapoints=" << options.profile_warmup_datapoints << '\n';
  out << "writer_threads=" << options.writer_threads << '\n';
  out << "jpg_quality=" << options.jpg_quality << '\n';
  out << "png_compression_level=" << options.png_compression_level << '\n';
  out << "plant_blur_radius_px=" << options.plant_blur_radius_px << '\n';
  if (options.override_max_target_gdd) {
    out << "max_target_gdd=" << options.max_target_gdd << '\n';
  }
  out << "calendar_step_days=" << options.calendar_step_days << '\n';
  out << "ray_trace_samples=" << options.ray_trace_samples << '\n';
  out << "ray_trace_bounces=" << options.ray_trace_bounces << '\n';
  out << "transparent_bg=" << BoolString(options.transparent_bg) << '\n';
  out << "load_scene=" << BoolString(options.load_scene) << '\n';
  out << "use_scene_main_camera=" << BoolString(options.use_scene_main_camera) << '\n';
  out << "use_scene_pine_transforms=" << BoolString(options.use_scene_pine_transforms) << '\n';
  out << "use_scene_pine_growth=" << BoolString(options.use_scene_pine_growth) << '\n';
  out << "render_needles=" << BoolString(options.render_needles) << '\n';
  out << "preserve_scene_pine_seed=" << BoolString(options.preserve_scene_pine_seed) << '\n';
  out << "calendar_start_from_reset=" << BoolString(options.calendar_start_from_reset) << '\n';
  out << "strict_parity=" << BoolString(options.strict_parity) << '\n';
  out << "batch_output_subdirs=" << BoolString(options.batch_output_subdirs) << '\n';
  out << "keep_going=" << BoolString(options.keep_going) << '\n';
  out << "composite_background=" << BoolString(options.composite_background) << '\n';
  out << "write_raw_rgba=" << BoolString(options.write_raw_rgba) << '\n';
  out << "write_foreground_mask=" << BoolString(options.write_foreground_mask) << '\n';
  out << "export_depth=" << BoolString(options.export_depth) << '\n';
  out << "export_instance_mask=" << BoolString(options.export_instance_mask) << '\n';
  out << "export_synthetic_labels=" << BoolString(options.export_synthetic_labels) << '\n';
  out << "export_flow_graph=" << BoolString(options.export_flow_graph) << '\n';
  out << "export_node_graph=" << BoolString(options.export_node_graph) << '\n';
  out << "export_needle_skeleton=" << BoolString(options.export_needle_skeleton) << '\n';
  out << "export_annotation_skeleton=" << BoolString(options.export_annotation_skeleton) << '\n';
  out << "export_annotation_overlay=" << BoolString(options.export_annotation_overlay) << '\n';
  out << "uncapped_growth=" << BoolString(options.uncapped_growth) << '\n';
  return true;
}

}  // namespace

void LSystemLayer::OnCreate() {
  calendar_start_year = std::max(0, calendar_start_year);
  calendar_start_day_of_year = ClampDayOfYear(calendar_start_day_of_year);
  simulation_year = std::max(0, simulation_year);
  simulation_day_of_year = ClampDayOfYear(simulation_day_of_year);
  calendar_end_enabled = true;
  calendar_end_year = std::max(0, calendar_end_year);
  calendar_end_day_of_year = ClampDayOfYear(calendar_end_day_of_year);
  ClampCalendarToEnd(*this);
  InitializeFullFidelitySyntheticExportDefaults();
  ApplyGlobalPlantColorMode(tassel_color_mode, scene_plant_view_tint_enabled);
  ApplyGlobalSeasonalPlantTint(*this);
}

void LSystemLayer::OnDestroy() {
  RestoreSeasonalSceneTint(GetApplication().GetLayer<EditorLayer>());
  ScotsPine::SetGlobalSeasonalColorTint({});
}

bool LSystemLayer::SupportsProjectStateSerialization() const {
  return true;
}

void LSystemLayer::SerializeProjectState(YAML::Emitter& out) const {
  SerializeLSystemLayer(out, *this);
}

void LSystemLayer::DeserializeProjectState(const YAML::Node& in) {
  DeserializeLSystemLayer(in, *this);
}

void LSystemLayer::EnsureLoadedSceneScotsPinesGenerated() {
  const auto scene = GetScene();
  if (!scene) {
    auto_regenerated_scene_.reset();
    return;
  }
  if (auto_regenerated_scene_.lock() == scene) {
    return;
  }
  if (!ProjectManager::IsProjectIdle()) {
    return;
  }

  const auto* pine_entities_ptr = scene->UnsafeGetPrivateComponentOwnersList<ScotsPine>();
  if (!pine_entities_ptr) {
    auto_regenerated_scene_ = scene;
    return;
  }

  bool waiting_for_descriptor = false;
  const std::vector<Entity> pine_entities = *pine_entities_ptr;
  for (const auto& entity : pine_entities) {
    if (!scene->IsEntityValid(entity)) {
      continue;
    }
    const auto pine = scene->GetOrSetPrivateComponent<ScotsPine>(entity).lock();
    if (!pine) {
      continue;
    }

    if (!pine->descriptor_ref.Get<ScotsPineDescriptor>()) {
      waiting_for_descriptor = true;
      continue;
    }

    // Saved ScotsPine child render entities are only a derived cache. Regenerate
    // them from the serialized descriptor, seed, and target GDD so the editor
    // viewport and synthetic renderer start from the same authoritative state.
    const float requested_target_gdd = std::max(0.0f, pine->target_gdd);
    pine->ClearGeometryEntities();
    pine->growth_model.Reset();
    pine->target_gdd = requested_target_gdd;
    if (requested_target_gdd > 0.0f) {
      pine->GenerateGeometryEntities();
    }
  }

  if (!waiting_for_descriptor) {
    auto_regenerated_scene_ = scene;
  }
}

void LSystemLayer::ApplySeasonalSceneTint(const std::shared_ptr<EditorLayer>& editor_layer) {
  if (!editor_layer) {
    return;
  }
  const auto scene_camera = editor_layer->GetSceneCamera();
  if (!scene_camera) {
    return;
  }
  if (!seasonal_sky_tint_enabled) {
    RestoreSeasonalSceneTint(editor_layer);
    return;
  }
  if (!seasonal_scene_camera_override_active_) {
    seasonal_scene_camera_prev_use_clear_color_ = scene_camera->camera_settings.use_clear_color;
    seasonal_scene_camera_prev_clear_color_ = scene_camera->camera_settings.clear_color;
    seasonal_scene_camera_override_active_ = true;
  }
  scene_camera->camera_settings.use_clear_color = true;
  scene_camera->camera_settings.clear_color = ComputeSeasonalTintColor(simulation_day_of_year, seasonal_tint_colors);
}

void LSystemLayer::RestoreSeasonalSceneTint(const std::shared_ptr<EditorLayer>& editor_layer) {
  if (!seasonal_scene_camera_override_active_ || !editor_layer) {
    return;
  }
  const auto scene_camera = editor_layer->GetSceneCamera();
  if (!scene_camera) {
    return;
  }
  scene_camera->camera_settings.use_clear_color = seasonal_scene_camera_prev_use_clear_color_;
  scene_camera->camera_settings.clear_color = seasonal_scene_camera_prev_clear_color_;
  seasonal_scene_camera_override_active_ = false;
}

void LSystemLayer::PushProfileFrame(const ProfileFrame& frame) {
  if (profiling_history_size <= 0) {
    return;
  }
  if (profiling_history.size() >= static_cast<size_t>(profiling_history_size)) {
    profiling_history.erase(profiling_history.begin());
  }
  profiling_history.push_back(frame);
}

void LSystemLayer::ExportProfileCsv(const std::string& path) const {
  std::ofstream out(path, std::ios::trunc);
  if (!out.is_open()) {
    return;
  }

  out << "frame,update_ms,grow_ms,rebuild_ms,pines,growth_steps,nodes,internodes,needles,invalid_instances\n";
  for (size_t i = 0; i < profiling_history.size(); i++) {
    const auto& f = profiling_history[i];
    out << i << "," << std::fixed << std::setprecision(4) << f.update_ms << "," << f.grow_ms << "," << f.rebuild_ms
        << "," << f.pine_count << "," << f.growth_steps << "," << f.node_count << "," << f.internode_count << ","
        << f.needle_count << "," << f.invalid_instance_count << "\n";
  }
}

void LSystemLayer::RunSyntheticRenderProfile() {
  synthetic_profile_last_total_ms = 0.0;
  synthetic_profile_samples_per_hour = 0.0;
  synthetic_profile_last_output.clear();

  const auto scene = GetScene();
  if (!scene) {
    synthetic_profile_last_status = "No active scene.";
    return;
  }

  const auto camera = scene->main_camera.Get<Camera>();
  if (!camera || !scene->IsEntityValid(camera->GetOwner())) {
    synthetic_profile_last_status = "Active scene has no valid main camera.";
    return;
  }

  const bool ray_tracing = synthetic_profile_render_mode == 1;
  if (ray_tracing && !Platform::RayTracingEnabled()) {
    synthetic_profile_last_status = "Ray tracing requested, but this device has ray tracing disabled.";
    return;
  }

  const auto output_dir =
      std::filesystem::path(synthetic_profile_output_root) / ("profile_" + BuildProfileStamp());
  std::filesystem::create_directories(output_dir);
  const auto image_path = output_dir / "active_scene_rgba.png";
  const auto json_path = output_dir / "render_profile.json";
  const auto csv_path = output_dir / "render_profile.csv";

  const auto old_mode = camera->camera_render_mode;
  const int old_samples = camera->camera_settings.sample_size;
  const int old_bounces = camera->camera_settings.bounce;

  const auto start_time = std::chrono::steady_clock::now();
  camera->camera_render_mode = ray_tracing ? Camera::CameraRenderMode::RayTracing : Camera::CameraRenderMode::Rasterization;
  camera->camera_settings.sample_size = std::max(1, synthetic_profile_ray_trace_samples);
  camera->camera_settings.bounce = std::max(1, synthetic_profile_ray_trace_bounces);
  camera->ResetFrameCount();
  camera->SetRequireRendering(true);
  GetApplication().Loop();
  GetApplication().Loop();
  const double render_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start_time).count();

  const auto write_start_time = std::chrono::steady_clock::now();
  const auto size = camera->GetSize();
  camera->GetRenderTexture()->StoreToPng(image_path, static_cast<int>(size.x), static_cast<int>(size.y));
  const double write_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - write_start_time).count();
  synthetic_profile_last_total_ms = render_ms + write_ms;
  synthetic_profile_samples_per_hour =
      synthetic_profile_last_total_ms > 0.0 ? 3600000.0 / synthetic_profile_last_total_ms : 0.0;

  std::ofstream json(json_path, std::ios::trunc);
  if (json.is_open()) {
    json << std::fixed << std::setprecision(4);
    json << "{\n";
    json << "  \"run\": {\n";
    json << "    \"source\": \"editor_active_scene\",\n";
    json << "    \"render_mode\": \"" << (ray_tracing ? "ray_tracing" : "rasterization") << "\",\n";
    json << "    \"ray_trace_samples\": " << synthetic_profile_ray_trace_samples << ",\n";
    json << "    \"ray_trace_bounces\": " << synthetic_profile_ray_trace_bounces << "\n";
    json << "  },\n";
    json << "  \"summary\": {\n";
    json << "    \"datapoints\": 1,\n";
    json << "    \"mean_ms\": " << synthetic_profile_last_total_ms << ",\n";
    json << "    \"samples_per_hour\": " << synthetic_profile_samples_per_hour << "\n";
    json << "  },\n";
    json << "  \"datapoints\": [\n";
    json << "    {\n";
    json << "      \"datapoint_index\": 0,\n";
    json << "      \"warmup\": false,\n";
    json << "      \"success\": true,\n";
    json << "      \"resolution\": [" << size.x << ", " << size.y << "],\n";
    json << "      \"rgb_path\": \"" << EscapeProfileJson(image_path.string()) << "\",\n";
    json << "      \"phases_ms\": {\"rgb_render\": " << render_ms << ", \"rgb_write\": " << write_ms
         << ", \"total_engine\": " << synthetic_profile_last_total_ms << "},\n";
    json << "      \"error\": \"\"\n";
    json << "    }\n";
    json << "  ]\n";
    json << "}\n";
  }

  std::ofstream csv(csv_path, std::ios::trunc);
  if (csv.is_open()) {
    csv << "datapoint_index,warmup,success,width,height,total_engine_ms,rgb_render_ms,rgb_write_ms,rgb_path,error\n";
    csv << std::fixed << std::setprecision(4);
    csv << "0,0,1," << size.x << "," << size.y << "," << synthetic_profile_last_total_ms << "," << render_ms << ","
        << write_ms << ",\"" << EscapeProfileJson(image_path.string()) << "\",\"\"\n";
  }

  camera->camera_render_mode = old_mode;
  camera->camera_settings.sample_size = old_samples;
  camera->camera_settings.bounce = old_bounces;
  camera->ResetFrameCount();

  synthetic_profile_last_output = output_dir.string();
  std::ostringstream status;
  status << "Profiled active scene in " << std::fixed << std::setprecision(2) << synthetic_profile_last_total_ms
         << " ms.";
  synthetic_profile_last_status = status.str();
}

void LSystemLayer::InitializeFullFidelitySyntheticExportDefaults() {
  const auto output_root_text = BufferToString(full_fidelity_output_root);
  if (output_root_text.empty() || !std::filesystem::path(output_root_text).is_absolute()) {
    CopyStringToBuffer(DefaultFullFidelityOutputRoot().string(), full_fidelity_output_root);
  }
  if (BufferToString(full_fidelity_output_name).empty()) {
    CopyStringToBuffer(kFullFidelityOutputName, full_fidelity_output_name);
  }
}

void LSystemLayer::RequestFullFidelitySyntheticExport() {
  InitializeFullFidelitySyntheticExportDefaults();
  if (full_fidelity_export_future_.valid()) {
    full_fidelity_last_status = "Export is already running.";
    return;
  }

  auto scene = GetScene();
  if (!scene) {
    full_fidelity_last_status = "No active scene.";
    return;
  }

  if (!scene->IsTemporary()) {
    scene->Save();
  }
  ProjectManager::SaveProject();
  for (int flush_attempt = 0; flush_attempt < 8 && !ProjectManager::IsProjectIdle(); ++flush_attempt) {
    GetApplication().Loop();
  }
  if (!ProjectManager::IsProjectIdle()) {
    full_fidelity_last_status = "Project save is still flushing; try the export again once the project is idle.";
    return;
  }

  ScotsPineSyntheticRenderOptions options;
  options.project_path = ProjectManager::GetProjectPath();
  if (options.project_path.empty()) {
    full_fidelity_last_status = "No project path is available.";
    return;
  }

  if (const auto active_scene_path = ResolveActiveSceneAssetPath(scene); !active_scene_path.empty()) {
    options.scene_path = active_scene_path;
  }
  if (const auto descriptor_path = ResolveFirstScotsPineDescriptorPath(scene); !descriptor_path.empty()) {
    options.descriptor_path = descriptor_path;
  }

  const auto output_root_text = BufferToString(full_fidelity_output_root);
  const auto output_name_text = BufferToString(full_fidelity_output_name);
  const auto output_base = path_utils::NormalizeAbsolutePath(output_root_text.empty() ? DefaultFullFidelityOutputRoot()
                                                                                     : std::filesystem::path(output_root_text));
  options.output_root = output_base / ("preview_" + BuildPreviewStamp());
  options.output_name = output_name_text.empty() ? kFullFidelityOutputName : output_name_text;
  options.background_image = FullFidelityBackgroundImage();
  options.background_dir = FullFidelityBackgroundDirectory();
  options.render_mode = kFullFidelityRenderMode;
  options.rgb_output_format = kFullFidelityRgbFormat;
  options.width = kFullFidelityWidth;
  options.height = kFullFidelityHeight;
  options.sample_count = 1;
  options.start_index = 0;
  options.worker_id = 0;
  options.worker_count = 1;
  options.frame_count = 1;
  options.profile_warmup_datapoints = kFullFidelityProfileWarmupDatapoints;
  options.writer_threads = kFullFidelityWriterThreads;
  options.jpg_quality = kFullFidelityJpgQuality;
  options.png_compression_level = kFullFidelityPngCompressionLevel;
  options.plant_blur_radius_px = kFullFidelityPlantBlurRadiusPx;
  options.max_target_gdd = kFullFidelityMaxTargetGdd;
  options.calendar_step_days = 1.0f;
  options.override_max_target_gdd = true;
  options.transparent_bg = true;
  options.load_scene = true;
  options.use_scene_main_camera = true;
  options.use_scene_pine_transforms = true;
  options.use_scene_pine_growth = true;
  options.preserve_scene_pine_seed = false;
  options.calendar_start_from_reset = true;
  options.strict_parity = false;
  options.batch_output_subdirs = true;
  options.keep_going = false;
  options.composite_background = true;
  options.write_raw_rgba = false;
  options.write_foreground_mask = true;
  options.export_depth = false;
  options.export_instance_mask = false;
  options.export_synthetic_labels = false;
  options.export_flow_graph = false;
  options.export_node_graph = false;
  options.export_needle_skeleton = false;
  options.export_annotation_skeleton = true;
  options.export_annotation_overlay = true;

  const auto renderer_executable = ResolveScotsPineRendererExecutable();
  if (renderer_executable.empty() || !std::filesystem::exists(renderer_executable)) {
    full_fidelity_last_status = "Could not locate ScotsPineDataGeneratorApp executable.";
    return;
  }

  const auto config_path =
      std::filesystem::temp_directory_path() / ("evo_scots_pine_full_fidelity_" + BuildProfileStamp() + ".cfg");
  std::string config_error;
  if (!WriteSyntheticOptionsConfig(options, config_path, config_error)) {
    full_fidelity_last_status = config_error;
    return;
  }

  const auto rgb_path = ScotsPineSyntheticExpectedFirstRgbPath(options);
  const std::string command = BuildSyntheticRendererCommand(renderer_executable, config_path);

  full_fidelity_preview_texture_.reset();
  full_fidelity_last_config_path = config_path.string();
  full_fidelity_last_output = options.output_root.string();
  full_fidelity_last_rgb_path = rgb_path.string();
  full_fidelity_last_status = "Full-fidelity export running.";

  full_fidelity_export_future_ = std::async(std::launch::async, [command, config_path, output_root = options.output_root,
                                                                 rgb_path]() {
    FullFidelityExportResult result;
    result.config_path = config_path;
    result.output_root = output_root;
    result.rgb_path = rgb_path;
    const auto start_time = std::chrono::steady_clock::now();
    result.exit_code = std::system(command.c_str());
    result.elapsed_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start_time).count();
    if (result.exit_code != 0) {
      result.error = "Renderer exited with code " + std::to_string(result.exit_code) + ".";
    }
    return result;
  });
}

void LSystemLayer::PollFullFidelitySyntheticExport() {
  if (!full_fidelity_export_future_.valid()) {
    return;
  }
  if (full_fidelity_export_future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
    return;
  }

  FullFidelityExportResult result;
  try {
    result = full_fidelity_export_future_.get();
  } catch (const std::exception& e) {
    full_fidelity_last_status = std::string("Full-fidelity export failed: ") + e.what();
    return;
  }

  full_fidelity_last_output = result.output_root.string();
  full_fidelity_last_rgb_path = result.rgb_path.string();
  full_fidelity_last_config_path = result.config_path.string();

  if (!result.error.empty()) {
    full_fidelity_last_status = result.error;
    return;
  }

  std::error_code exists_error;
  if (!std::filesystem::exists(result.rgb_path, exists_error)) {
    full_fidelity_last_status = "Export finished, but the expected composited RGB file was not found.";
    return;
  }

  LoadFullFidelityPreviewTexture(result.rgb_path);
  std::ostringstream status;
  status << "Full-fidelity export finished in " << std::fixed << std::setprecision(2) << result.elapsed_ms << " ms.";
  full_fidelity_last_status = status.str();
}

void LSystemLayer::LoadFullFidelityPreviewTexture(const std::filesystem::path& path) {
  auto texture = AssetManager::CreateTemporaryAsset<Texture2D>();
  if (texture && texture->Import(path)) {
    full_fidelity_preview_texture_ = texture;
  } else {
    full_fidelity_preview_texture_.reset();
  }
}

void LSystemLayer::DrawFullFidelitySyntheticExportUi() {
  InitializeFullFidelitySyntheticExportDefaults();

  if (!ImGui::CollapsingHeader("Full-Fidelity Synthetic Export", ImGuiTreeNodeFlags_DefaultOpen)) {
    return;
  }
  ImGui::InputText("Output Root", full_fidelity_output_root.data(), full_fidelity_output_root.size());
  ImGui::InputText("Output Name", full_fidelity_output_name.data(), full_fidelity_output_name.size());

  ImGui::TextDisabled("Fixed production settings");
  ImGui::Text("Resolution: %dx%d", kFullFidelityWidth, kFullFidelityHeight);
  ImGui::Text("Renderer: %s", kFullFidelityRenderMode);
  ImGui::Text("RGB: %s, quality %d", kFullFidelityRgbFormat, kFullFidelityJpgQuality);
  ImGui::Text("Final output: composited JPG with plant blur %.2f px", kFullFidelityPlantBlurRadiusPx);
  ImGui::Text("Annotations: skeleton JSON and red-dot overlay PNG");
  ImGui::Text("Scene camera, transforms, growth, and foreground mask: on");
  ImGui::Text("Raw RGBA, depth, instance masks, labels, and graphs: off");

  const bool export_running = full_fidelity_export_future_.valid();
  if (!export_running) {
    if (ImGui::Button("Run Production Preview")) {
      full_fidelity_export_requested_ = true;
    }
  } else {
    ImGui::Text("Export running...");
  }

  ImGui::TextWrapped("%s", full_fidelity_last_status.c_str());
  if (!full_fidelity_last_output.empty()) {
    ImGui::TextWrapped("Latest Output: %s", full_fidelity_last_output.c_str());
    if (ImGui::Button("Copy Output Path")) {
      ImGui::SetClipboardText(full_fidelity_last_output.c_str());
    }
    ImGui::SameLine();
    if (ImGui::Button("Open Output Folder")) {
      if (!OpenFolderInExplorer(full_fidelity_last_output)) {
        full_fidelity_last_status = "Could not open the output folder.";
      }
    }
  } else {
    ImGui::BeginDisabled();
    ImGui::Button("Copy Output Path");
    ImGui::SameLine();
    ImGui::Button("Open Output Folder");
    ImGui::EndDisabled();
  }
  if (!full_fidelity_last_rgb_path.empty()) {
    ImGui::TextWrapped("Preview JPG: %s", full_fidelity_last_rgb_path.c_str());
  }
  if (!full_fidelity_last_config_path.empty()) {
    ImGui::TextWrapped("Config: %s", full_fidelity_last_config_path.c_str());
  }

  if (full_fidelity_preview_texture_) {
    const auto resolution = full_fidelity_preview_texture_->GetResolution();
    if (resolution.x > 0 && resolution.y > 0) {
      const float max_width = std::max(160.0f, ImGui::GetContentRegionAvail().x);
      const float preview_width = std::min(max_width, static_cast<float>(resolution.x));
      const float preview_height = preview_width * static_cast<float>(resolution.y) / static_cast<float>(resolution.x);
      ImGui::Image(full_fidelity_preview_texture_->GetImTextureId(), ImVec2(preview_width, preview_height), ImVec2(0, 1),
                   ImVec2(1, 0));
    }
  }
}

void LSystemLayer::Update() {
  PollFullFidelitySyntheticExport();
  if (full_fidelity_export_requested_) {
    full_fidelity_export_requested_ = false;
    RequestFullFidelitySyntheticExport();
  }

  if (synthetic_profile_requested_) {
    synthetic_profile_requested_ = false;
    RunSyntheticRenderProfile();
  }

  EnsureLoadedSceneScotsPinesGenerated();

  auto& times = GetApplication().GetTimes();
  const double update_start = times.Now();
  ProfileFrame frame{};

  if (!auto_grow) {
    return;
  }

  const auto scene = GetScene();
  if (!scene) {
    return;
  }

  const float dt = static_cast<float>(times.DeltaTime());
  if (dt > 0.0f) {
    const float fps = 1.0f / dt;
    if (fps < kAutoGrowFailsafeMinFps) {
      auto_grow = false;
      fps_failsafe_tripped_ = true;
      last_failsafe_fps_ = fps;
      return;
    }
  }

  const float requested_delta_days = std::max(0.0f, chronological_days_per_second) * std::max(0.0f, dt);
  bool calendar_end_reached = false;
  const float delta_days = AdvanceLayerCalendar(*this, requested_delta_days, calendar_end_reached);
  if (calendar_end_reached && stop_auto_grow_at_calendar_end) {
    auto_grow = false;
  }
  ApplyGlobalSeasonalPlantTint(*this);

  ScotsPineCalendarSettings calendar_settings;
  calendar_settings.simulation_day_of_year = simulation_day_of_year;
  calendar_settings.simulation_year = simulation_year;
  calendar_settings.calendar_end_enabled = calendar_end_enabled;
  calendar_settings.calendar_end_year = calendar_end_year;
  calendar_settings.calendar_end_day_of_year = calendar_end_day_of_year;

  if (const auto* pine_entities_ptr = scene->UnsafeGetPrivateComponentOwnersList<ScotsPine>()) {
    const std::vector<Entity> pine_entities = *pine_entities_ptr;
    for (const auto& entity : pine_entities) {
      if (!scene->IsEntityValid(entity)) {
        continue;
      }

      auto pine = scene->GetOrSetPrivateComponent<ScotsPine>(entity).lock();
      if (!pine) {
        continue;
      }
      frame.pine_count++;

      const auto step_result = AdvanceScotsPineCalendarStep(*pine, calendar_settings, delta_days);

      if (profiling_enabled) {
        frame.grow_ms += pine->last_grow_seconds * 1000.0;
        frame.rebuild_ms += pine->last_rebuild_seconds * 1000.0;
        frame.growth_steps += step_result.growth_steps;
        frame.node_count += pine->last_node_count;
        frame.internode_count += pine->last_internode_count;
        frame.needle_count += pine->last_needle_count;
        frame.invalid_instance_count += pine->last_invalid_instance_count;
      }
    }
  }

  if (profiling_enabled) {
    frame.update_ms = (times.Now() - update_start) * 1000.0;
    last_profile_frame = frame;
    PushProfileFrame(frame);
  }
}

bool l_system_package::InspectLSystemLayer(InspectorContext& context, LSystemLayer& layer) {
  layer.ApplySeasonalSceneTint(context.editor_layer);
  auto& auto_grow = layer.auto_grow;
  auto& reseed_on_reset = layer.reseed_on_reset;
  auto& calendar_start_year = layer.calendar_start_year;
  auto& calendar_start_day_of_year = layer.calendar_start_day_of_year;
  auto& chronological_days_per_second = layer.chronological_days_per_second;
  auto& simulation_year = layer.simulation_year;
  auto& simulation_day_of_year = layer.simulation_day_of_year;
  auto& calendar_end_enabled = layer.calendar_end_enabled;
  auto& calendar_end_year = layer.calendar_end_year;
  auto& calendar_end_day_of_year = layer.calendar_end_day_of_year;
  auto& stop_auto_grow_at_calendar_end = layer.stop_auto_grow_at_calendar_end;
  auto& tassel_color_mode = layer.tassel_color_mode;
  auto& scene_plant_view_tint_enabled = layer.scene_plant_view_tint_enabled;
  auto& seasonal_sky_tint_enabled = layer.seasonal_sky_tint_enabled;
  auto& seasonal_plant_tint_enabled = layer.seasonal_plant_tint_enabled;
  auto& seasonal_plant_tint_strength = layer.seasonal_plant_tint_strength;
  auto& seasonal_tint_colors = layer.seasonal_tint_colors;
  auto& profiling_enabled = layer.profiling_enabled;
  auto& profiling_history_size = layer.profiling_history_size;
  auto& profiling_export_path = layer.profiling_export_path;
  auto& last_profile_frame = layer.last_profile_frame;
  auto& profiling_history = layer.profiling_history;
  auto& fps_failsafe_tripped_ = layer.fps_failsafe_tripped_;
  auto& last_failsafe_fps_ = layer.last_failsafe_fps_;

  auto reset_all_lsystems = [&]() {
    const auto scene = layer.GetScene();
    if (!scene) {
      return;
    }

    unsigned int base_seed = 0u;
    if (reseed_on_reset) {
      base_seed = static_cast<unsigned int>(std::chrono::steady_clock::now().time_since_epoch().count() & 0xffffffffu);
    }
    unsigned int seed_offset = 0u;

    if (const auto* pine_entities_ptr = scene->UnsafeGetPrivateComponentOwnersList<ScotsPine>()) {
      const std::vector<Entity> pine_entities = *pine_entities_ptr;
      for (const auto& entity : pine_entities) {
        if (!scene->IsEntityValid(entity)) {
          continue;
        }
        auto pine = scene->GetOrSetPrivateComponent<ScotsPine>(entity).lock();
        if (!pine) {
          continue;
        }

        if (reseed_on_reset) {
          pine->seed = base_seed + seed_offset++;
        }
        // Hard reset for Ctrl+W: clear all generated geometry and leave the
        // plant at zero thermal target (no warm-start regrowth).
        pine->target_gdd = 0.0f;
        pine->growth_model.Reset();
        pine->ClearGeometryEntities();
      }
    }

    simulation_year = std::max(0, calendar_start_year);
    simulation_day_of_year = ClampDayOfYear(calendar_start_day_of_year);
    ClampCalendarToEnd(layer);
    ApplyGlobalSeasonalPlantTint(layer);
  };

  const auto left_ctrl_state = EditorLayer::GetKey(GLFW_KEY_LEFT_CONTROL);
  const auto right_ctrl_state = EditorLayer::GetKey(GLFW_KEY_RIGHT_CONTROL);
  const bool ctrl_down =
      left_ctrl_state == Input::KeyActionType::Hold || left_ctrl_state == Input::KeyActionType::Press ||
      right_ctrl_state == Input::KeyActionType::Hold || right_ctrl_state == Input::KeyActionType::Press;

  if (ctrl_down) {
    if (EditorLayer::GetKey(GLFW_KEY_F) == Input::KeyActionType::Press) {
      auto_grow = !auto_grow;
      if (auto_grow) {
        fps_failsafe_tripped_ = false;
        last_failsafe_fps_ = 0.0f;
      }
    }
    if (EditorLayer::GetKey(GLFW_KEY_W) == Input::KeyActionType::Press) {
      auto& app = layer.GetApplication();
      if (app.IsPlaying()) {
        app.Stop();
      }
      auto_grow = false;
      reset_all_lsystems();
    }
  }

  const auto window_title = layer.GetLayerName();
  bool open = layer.enable_inspection;
  if (!ImGui::Begin(window_title.c_str(), &open)) {
    ImGui::End();
    layer.enable_inspection = open;
    return false;
  }
  if (ImGui::CollapsingHeader("Growth", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (ImGui::Checkbox("Auto-Grow (Ctrl+F)", &auto_grow) && auto_grow) {
      fps_failsafe_tripped_ = false;
      last_failsafe_fps_ = 0.0f;
    }
    ImGui::TextDisabled("Thermal rates are descriptor-owned and sampled per pine.");

    ImGui::SeparatorText("Calendar");
    calendar_end_enabled = true;
    if (ImGui::DragInt("Calendar Start Year", &calendar_start_year, 1.0f, 0, 10000)) {
      calendar_start_year = std::max(0, calendar_start_year);
    }
    if (ImGui::DragFloat("Calendar Start Day Of Year", &calendar_start_day_of_year, 0.25f, 0.0f, 364.999f, "%.2f")) {
      calendar_start_day_of_year = ClampDayOfYear(calendar_start_day_of_year);
    }
    if (ImGui::Button("Reset Calendar To Start")) {
      simulation_year = std::max(0, calendar_start_year);
      simulation_day_of_year = ClampDayOfYear(calendar_start_day_of_year);
      ClampCalendarToEnd(layer);
      ApplyGlobalSeasonalPlantTint(layer);
      RebuildAllPlantGeometry(layer.GetScene());
    }
    ImGui::DragFloat("Calendar Days/sec (Chronology)", &chronological_days_per_second, 0.25f, 0.0f, 365.0f, "%.2f");
    ImGui::Text("Simulation Year: %d", simulation_year);
    ImGui::Text("Simulation Day Of Year: %.2f", simulation_day_of_year);
    ImGui::TextDisabled("Calendar end is always enabled.");
    if (ImGui::DragInt("End Year", &calendar_end_year, 1.0f, 0, 10000)) {
      calendar_end_year = std::max(0, calendar_end_year);
      ClampCalendarToEnd(layer);
      ApplyGlobalSeasonalPlantTint(layer);
      RebuildAllPlantGeometry(layer.GetScene());
    }
    if (ImGui::DragFloat("End Day Of Year", &calendar_end_day_of_year, 0.25f, 0.0f, 364.999f, "%.2f")) {
      calendar_end_day_of_year = ClampDayOfYear(calendar_end_day_of_year);
      ClampCalendarToEnd(layer);
      ApplyGlobalSeasonalPlantTint(layer);
      RebuildAllPlantGeometry(layer.GetScene());
    }
    ImGui::Checkbox("Stop Auto-Grow At Calendar End", &stop_auto_grow_at_calendar_end);
    if (IsCalendarAtEnd(layer)) {
      ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.2f, 1.0f), "Calendar end reached.");
    }
    ImGui::Text("Calendar: Year %d, Day %.2f", simulation_year, simulation_day_of_year);
    if (const auto scene = layer.GetScene()) {
      if (const auto* pine_entities_ptr = scene->UnsafeGetPrivateComponentOwnersList<ScotsPine>()) {
        uint32_t pine_count = 0;
        float target_gdd_sum = 0.0f;
        float accumulated_gdd_sum = 0.0f;
        uint32_t node_count = 0;
        uint32_t internode_count = 0;
        uint32_t needle_count = 0;
        for (const auto& entity : *pine_entities_ptr) {
          if (!scene->IsEntityValid(entity)) {
            continue;
          }
          const auto pine = scene->GetOrSetPrivateComponent<ScotsPine>(entity).lock();
          if (!pine) {
            continue;
          }
          pine_count++;
          target_gdd_sum += std::max(0.0f, pine->target_gdd);
          accumulated_gdd_sum += std::max(0.0f, pine->growth_model.accumulated_gdd);
          node_count += pine->last_node_count;
          internode_count += pine->last_internode_count;
          needle_count += pine->last_needle_count;
        }
        if (pine_count > 0) {
          const float inv_count = 1.0f / static_cast<float>(pine_count);
          ImGui::Text("Scots pine growth: %u pines, mean target %.1f GDD, mean accumulated %.1f GDD", pine_count,
                      target_gdd_sum * inv_count, accumulated_gdd_sum * inv_count);
          ImGui::Text("Current nodes/internodes/needles: %u / %u / %u", node_count, internode_count, needle_count);
        }
      }
    }

    ImGui::Checkbox("Reseed on Reset (Ctrl+W when stopped)", &reseed_on_reset);
    if (fps_failsafe_tripped_) {
      ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.2f, 1.0f), "Auto-grow stopped by 1 FPS failsafe (last: %.2f FPS).",
                         last_failsafe_fps_);
      ImGui::TextDisabled("Re-enable Auto-Grow to resume growth.");
    }
    if (ImGui::Button("Reset All LSystems (Ctrl+W)")) {
      reset_all_lsystems();
    }
  }

  if (ImGui::CollapsingHeader("Profiling", profiling_enabled ? ImGuiTreeNodeFlags_DefaultOpen : 0)) {
    ImGui::Checkbox("Enable LSystem Profiling", &profiling_enabled);
    ImGui::DragInt("Profile History Size", &profiling_history_size, 1.0f, 30, 4000);

    static char export_path_buffer[260] = "lsystem_profile.csv";
    static std::string last_loaded_export_path;
    if (last_loaded_export_path != profiling_export_path && profiling_export_path.size() < sizeof(export_path_buffer)) {
      std::snprintf(export_path_buffer, sizeof(export_path_buffer), "%s", profiling_export_path.c_str());
      last_loaded_export_path = profiling_export_path;
    }
    if (ImGui::InputText("Profile CSV Path", export_path_buffer, sizeof(export_path_buffer))) {
      profiling_export_path = export_path_buffer;
      last_loaded_export_path = profiling_export_path;
    }

    if (ImGui::Button("Export Profile CSV")) {
      layer.ExportProfileCsv(profiling_export_path);
    }
    ImGui::SameLine();
    if (ImGui::Button("Clear Profile History")) {
      profiling_history.clear();
    }

    if (profiling_enabled) {
      const double avg_update_ms = AverageProfileMetric(profiling_history, [](const LSystemLayer::ProfileFrame& f) {
        return f.update_ms;
      });
      const double avg_grow_ms = AverageProfileMetric(profiling_history, [](const LSystemLayer::ProfileFrame& f) {
        return f.grow_ms;
      });
      const double avg_rebuild_ms = AverageProfileMetric(profiling_history, [](const LSystemLayer::ProfileFrame& f) {
        return f.rebuild_ms;
      });

      const double max_update_ms = MaxProfileMetric(profiling_history, [](const LSystemLayer::ProfileFrame& f) {
        return f.update_ms;
      });
      const double max_grow_ms = MaxProfileMetric(profiling_history, [](const LSystemLayer::ProfileFrame& f) {
        return f.grow_ms;
      });
      const double max_rebuild_ms = MaxProfileMetric(profiling_history, [](const LSystemLayer::ProfileFrame& f) {
        return f.rebuild_ms;
      });

      ImGui::SeparatorText("Rolling Stats");
      ImGui::Text("History Frames: %d", static_cast<int>(profiling_history.size()));
      ImGui::Text("Last Update: %.3f ms", last_profile_frame.update_ms);
      ImGui::Text("Last Grow: %.3f ms", last_profile_frame.grow_ms);
      ImGui::Text("Last Rebuild: %.3f ms", last_profile_frame.rebuild_ms);
      ImGui::Text("Last Pines: %u", last_profile_frame.pine_count);
      ImGui::Text("Last Growth Steps: %u", last_profile_frame.growth_steps);
      ImGui::Text("Last Nodes/Internodes/Needles: %u / %u / %u", last_profile_frame.node_count,
                  last_profile_frame.internode_count, last_profile_frame.needle_count);
      ImGui::Text("Last Invalid Instances: %u", last_profile_frame.invalid_instance_count);

      ImGui::SeparatorText("Averages / Maxima");
      ImGui::Text("Update ms avg/max: %.3f / %.3f", avg_update_ms, max_update_ms);
      ImGui::Text("Grow ms avg/max: %.3f / %.3f", avg_grow_ms, max_grow_ms);
      ImGui::Text("Rebuild ms avg/max: %.3f / %.3f", avg_rebuild_ms, max_rebuild_ms);
    }
  }

  if (ImGui::CollapsingHeader("Render", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (ImGui::Checkbox("Enable Scene/Plant View Tint", &scene_plant_view_tint_enabled)) {
      ApplyGlobalPlantColorMode(tassel_color_mode, scene_plant_view_tint_enabled);
      RebuildAllPlantGeometry(layer.GetScene());
    }
    if (!scene_plant_view_tint_enabled) {
      ImGui::TextDisabled("Tint disabled: effective mode forced to Shaded.");
    }

    {
      const char* color_mode_items[] = {
          "Shaded", "By Type", "By Instance", "By Node", "Needle Lignification", "Needle Stripe Proxy", "Needle Sheath"};
      if (ImGui::Combo("Plant Color Mode", &tassel_color_mode, color_mode_items, IM_ARRAYSIZE(color_mode_items))) {
        tassel_color_mode = ClampColorModeIndex(tassel_color_mode);
        ApplyGlobalPlantColorMode(tassel_color_mode, scene_plant_view_tint_enabled);
        RebuildAllPlantGeometry(layer.GetScene());
      }
    }

    if (ImGui::Button("Reset Plant Color View (Shaded)")) {
      tassel_color_mode = 0;
      scene_plant_view_tint_enabled = false;
      seasonal_plant_tint_enabled = false;
      ApplyGlobalPlantColorMode(tassel_color_mode, scene_plant_view_tint_enabled);
      ApplyGlobalSeasonalPlantTint(layer);
      RebuildAllPlantGeometry(layer.GetScene());
    }

    ImGui::SeparatorText("Seasonal Tint");
    const glm::vec4 seasonal_color = ComputeSeasonalTintColor(simulation_day_of_year, seasonal_tint_colors);
    ImGui::ColorButton("Current Seasonal Tint", ImVec4(seasonal_color.r, seasonal_color.g, seasonal_color.b,
                                                       seasonal_color.a));
    ImGui::SameLine();
    ImGui::Text("Day %.1f", simulation_day_of_year);
    if (ImGui::Checkbox("Seasonal Sky Tint", &seasonal_sky_tint_enabled)) {
      layer.ApplySeasonalSceneTint(context.editor_layer);
    }
    if (ImGui::Checkbox("Seasonal Plant Tint", &seasonal_plant_tint_enabled)) {
      ApplyGlobalSeasonalPlantTint(layer);
      RebuildAllPlantGeometry(layer.GetScene());
    }
    if (seasonal_plant_tint_enabled &&
        ImGui::DragFloat("Plant Tint Strength", &seasonal_plant_tint_strength, 0.01f, 0.0f, 1.0f, "%.2f")) {
      seasonal_plant_tint_strength = std::clamp(seasonal_plant_tint_strength, 0.0f, 1.0f);
      ApplyGlobalSeasonalPlantTint(layer);
      RebuildAllPlantGeometry(layer.GetScene());
    }
    const char* season_names[] = {"Spring", "Summer", "Fall", "Winter"};
    bool seasonal_palette_changed = false;
    for (size_t i = 0; i < seasonal_tint_colors.size(); i++) {
      seasonal_palette_changed =
          ImGui::ColorEdit4(season_names[i], &seasonal_tint_colors[i].x) || seasonal_palette_changed;
    }
    if (seasonal_palette_changed) {
      ApplyGlobalSeasonalPlantTint(layer);
      layer.ApplySeasonalSceneTint(context.editor_layer);
      RebuildAllPlantGeometry(layer.GetScene());
    }

    layer.DrawFullFidelitySyntheticExportUi();
  }
  ImGui::End();
  layer.enable_inspection = open;
  return false;
}

void l_system_package::SerializeLSystemLayer(YAML::Emitter& out, const LSystemLayer& target) {
  out << YAML::Key << "auto_grow" << YAML::Value << target.auto_grow;
  out << YAML::Key << "calendar_start_year" << YAML::Value << std::max(0, target.calendar_start_year);
  out << YAML::Key << "calendar_start_day_of_year" << YAML::Value
      << ClampDayOfYear(target.calendar_start_day_of_year);
  out << YAML::Key << "chronological_days_per_second" << YAML::Value << target.chronological_days_per_second;
  out << YAML::Key << "simulation_year" << YAML::Value << std::max(0, target.simulation_year);
  out << YAML::Key << "simulation_day_of_year" << YAML::Value
      << NormalizeScotsPineDayOfYear(target.simulation_day_of_year);
  out << YAML::Key << "calendar_end_enabled" << YAML::Value << true;
  out << YAML::Key << "calendar_end_year" << YAML::Value << std::max(0, target.calendar_end_year);
  out << YAML::Key << "calendar_end_day_of_year" << YAML::Value
      << ClampDayOfYear(target.calendar_end_day_of_year);
  out << YAML::Key << "stop_auto_grow_at_calendar_end" << YAML::Value << target.stop_auto_grow_at_calendar_end;
  out << YAML::Key << "reseed_on_reset" << YAML::Value << target.reseed_on_reset;
  out << YAML::Key << "tassel_color_mode" << YAML::Value << target.tassel_color_mode;
  out << YAML::Key << "scene_plant_view_tint_enabled" << YAML::Value << target.scene_plant_view_tint_enabled;
  out << YAML::Key << "seasonal_sky_tint_enabled" << YAML::Value << target.seasonal_sky_tint_enabled;
  out << YAML::Key << "seasonal_plant_tint_enabled" << YAML::Value << target.seasonal_plant_tint_enabled;
  out << YAML::Key << "seasonal_plant_tint_strength" << YAML::Value << target.seasonal_plant_tint_strength;
  out << YAML::Key << "seasonal_tint_colors" << YAML::Value << YAML::BeginSeq;
  for (const auto& color : target.seasonal_tint_colors) {
    out << color;
  }
  out << YAML::EndSeq;
  out << YAML::Key << "profiling_enabled" << YAML::Value << target.profiling_enabled;
  out << YAML::Key << "profiling_history_size" << YAML::Value << target.profiling_history_size;
  out << YAML::Key << "profiling_export_path" << YAML::Value << target.profiling_export_path;
  out << YAML::Key << "synthetic_profile_render_mode" << YAML::Value << target.synthetic_profile_render_mode;
  out << YAML::Key << "synthetic_profile_ray_trace_samples" << YAML::Value
      << target.synthetic_profile_ray_trace_samples;
  out << YAML::Key << "synthetic_profile_ray_trace_bounces" << YAML::Value
      << target.synthetic_profile_ray_trace_bounces;
  out << YAML::Key << "synthetic_profile_output_root" << YAML::Value << target.synthetic_profile_output_root;
  out << YAML::Key << "full_fidelity_output_root" << YAML::Value << BufferToString(target.full_fidelity_output_root);
  out << YAML::Key << "full_fidelity_output_name" << YAML::Value << BufferToString(target.full_fidelity_output_name);
}

void l_system_package::DeserializeLSystemLayer(const YAML::Node& in, LSystemLayer& target) {
  if (in["auto_grow"]) {
    target.auto_grow = in["auto_grow"].as<bool>();
  }
  if (in["calendar_start_year"]) {
    target.calendar_start_year = std::max(0, in["calendar_start_year"].as<int>());
  }
  if (in["calendar_start_day_of_year"]) {
    target.calendar_start_day_of_year = ClampDayOfYear(in["calendar_start_day_of_year"].as<float>());
  } else if (in["season_start_day"]) {
    target.calendar_start_day_of_year = ClampDayOfYear(static_cast<float>(in["season_start_day"].as<int>()));
  }
  if (in["season_end_day"] && !in["calendar_end_day_of_year"]) {
    target.calendar_end_day_of_year = ClampDayOfYear(static_cast<float>(in["season_end_day"].as<int>()));
  }
  if (in["chronological_days_per_second"]) {
    target.chronological_days_per_second = std::max(0.0f, in["chronological_days_per_second"].as<float>());
  }
  if (in["simulation_year"]) {
    target.simulation_year = std::max(0, in["simulation_year"].as<int>());
  }
  target.simulation_day_of_year = ClampDayOfYear(target.calendar_start_day_of_year);
  if (in["simulation_day_of_year"]) {
    target.simulation_day_of_year = NormalizeScotsPineDayOfYear(in["simulation_day_of_year"].as<float>());
  }
  target.calendar_end_enabled = true;
  if (in["calendar_end_year"]) {
    target.calendar_end_year = std::max(0, in["calendar_end_year"].as<int>());
  } else if (in["calendar_cap_year"]) {
    target.calendar_end_year = std::max(0, in["calendar_cap_year"].as<int>());
  }
  if (in["calendar_end_day_of_year"]) {
    target.calendar_end_day_of_year = ClampDayOfYear(in["calendar_end_day_of_year"].as<float>());
  } else if (in["calendar_cap_day_of_year"]) {
    target.calendar_end_day_of_year = ClampDayOfYear(in["calendar_cap_day_of_year"].as<float>());
  }
  if (in["stop_auto_grow_at_calendar_end"]) {
    target.stop_auto_grow_at_calendar_end = in["stop_auto_grow_at_calendar_end"].as<bool>();
  } else if (in["stop_auto_grow_at_calendar_cap"]) {
    target.stop_auto_grow_at_calendar_end = in["stop_auto_grow_at_calendar_cap"].as<bool>();
  }
  ClampCalendarToEnd(target);
  if (in["reseed_on_reset"]) {
    target.reseed_on_reset = in["reseed_on_reset"].as<bool>();
  }
  if (in["tassel_color_mode"]) {
    target.tassel_color_mode = ClampColorModeIndex(in["tassel_color_mode"].as<int>());
  }
  if (in["scene_plant_view_tint_enabled"]) {
    target.scene_plant_view_tint_enabled = in["scene_plant_view_tint_enabled"].as<bool>();
  }
  if (in["seasonal_sky_tint_enabled"]) {
    target.seasonal_sky_tint_enabled = in["seasonal_sky_tint_enabled"].as<bool>();
  }
  if (in["seasonal_plant_tint_enabled"]) {
    target.seasonal_plant_tint_enabled = in["seasonal_plant_tint_enabled"].as<bool>();
  }
  if (in["seasonal_plant_tint_strength"]) {
    target.seasonal_plant_tint_strength = std::clamp(in["seasonal_plant_tint_strength"].as<float>(), 0.0f, 1.0f);
  }
  if (in["seasonal_tint_colors"] && in["seasonal_tint_colors"].IsSequence()) {
    const auto colors = in["seasonal_tint_colors"];
    for (size_t i = 0; i < target.seasonal_tint_colors.size() && i < colors.size(); i++) {
      target.seasonal_tint_colors[i] = colors[i].as<glm::vec4>();
    }
  }
  if (in["profiling_enabled"]) {
    target.profiling_enabled = in["profiling_enabled"].as<bool>();
  }
  if (in["profiling_history_size"]) {
    target.profiling_history_size = std::max(30, in["profiling_history_size"].as<int>());
  }
  if (in["profiling_export_path"]) {
    target.profiling_export_path = in["profiling_export_path"].as<std::string>();
  }
  if (in["synthetic_profile_render_mode"]) {
    target.synthetic_profile_render_mode = std::clamp(in["synthetic_profile_render_mode"].as<int>(), 0, 1);
  }
  if (in["synthetic_profile_ray_trace_samples"]) {
    target.synthetic_profile_ray_trace_samples = std::max(1, in["synthetic_profile_ray_trace_samples"].as<int>());
  }
  if (in["synthetic_profile_ray_trace_bounces"]) {
    target.synthetic_profile_ray_trace_bounces = std::max(1, in["synthetic_profile_ray_trace_bounces"].as<int>());
  }
  if (in["synthetic_profile_output_root"]) {
    target.synthetic_profile_output_root = in["synthetic_profile_output_root"].as<std::string>();
  }
  if (in["full_fidelity_output_root"]) {
    CopyStringToBuffer(in["full_fidelity_output_root"].as<std::string>(), target.full_fidelity_output_root);
  }
  if (in["full_fidelity_output_name"]) {
    CopyStringToBuffer(in["full_fidelity_output_name"].as<std::string>(), target.full_fidelity_output_name);
  }
  target.InitializeFullFidelitySyntheticExportDefaults();

  ApplyGlobalPlantColorMode(target.tassel_color_mode, target.scene_plant_view_tint_enabled);
  ApplyGlobalSeasonalPlantTint(target);
  RebuildAllPlantGeometry(target.GetScene());
}
