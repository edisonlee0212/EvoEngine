#include "Application.hpp"
#include "ClassRegistry.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <limits>
#include <optional>
#include <random>
#include <string>

#include "EditorLayer.hpp"

#include "RenderLayer.hpp"

#ifdef DIGITAL_AGRICULTURE_PLUGIN

#  include "MaizeLayer.hpp"
using namespace digital_agriculture_plugin;
#endif
#include "WindowLayer.hpp"

#ifdef DATASET_GENERATION_PLUGIN
#  include <TasselPointCloudScanner.hpp>
#  include "DatasetGenerator.hpp"
using namespace dataset_generation_plugin;
#endif

#ifdef LSYSTEM_PLUGIN
#  include "MaizeTassel.hpp"
#  include "MaizeTasselDescriptor.hpp"
#endif

#ifdef ECOSYSLAB_PLUGIN
#  include "Soil.hpp"
#endif

using namespace evo_engine;

namespace {
struct GeneratorRunOptions {
  uint32_t output_count = 8u;
  uint32_t sample_index_start = 0u;
  uint32_t dataset_seed = 42u;
  bool use_hashed_seed = true;
  bool force_target_points = true;
  uint32_t target_points = 8000u;
  // TODO(perf): Default capture mode flipped to CPU for headless reliability.
  // GPU PT capture (PointCloud::SampleCurrentScene) requires a live RenderLayer
  // with valid swapchain-equivalent resources; whether it initializes correctly
  // in fully windowless mode has not been validated. CPU CpuRayTracer is
  // deterministic and runs without RenderLayer at all (Cpu branch of
  // run_windowless skips RenderLayer push). Re-evaluate after Phase 4+ when
  // the GPU pipeline is mature enough to validate windowless GPU PT capture.
  bool use_gpu_capture = false;
  bool realistic_noise = true;
  bool apply_per_seed_scanner_jitter = true;
  bool export_mesh = true;
  bool export_point_cloud = true;
  bool export_flow_graph = true;
  bool export_node_graph = true;
  bool open_output_folder = true;
  std::filesystem::path output_folder_override{};
};

bool ParseUintArg(const std::string& text, uint32_t& value) {
  try {
    size_t consumed = 0;
    const auto parsed = std::stoull(text, &consumed, 10);
    if (consumed != text.size()) {
      return false;
    }
    if (parsed > std::numeric_limits<uint32_t>::max()) {
      return false;
    }
    value = static_cast<uint32_t>(parsed);
    return true;
  } catch (const std::exception&) {
    return false;
  }
}

void PrintUsage() {
  EVOENGINE_LOG("MaizeDataGeneratorApp options:\n"
                "  --count <N>                 Number of Tassel_* samples to generate.\n"
                "  --index-start <N>           Starting sample index for naming (Tassel_<N>).\n"
                "  --seed <N>                  Dataset seed base.\n"
                "  --seed-hashed               Use hash-mixed per-sample seeds (default).\n"
                "  --seed-incremental          Use incremental per-sample seeds (seed + sample_index).\n"
                "  --target-points <N>         Override capture target points mean (default: 8000).\n"
                "  --no-target-points-override Disable target points override and use scene/descriptor value.\n"
                "  --output-folder <path>      Override output folder (default: .../SyntheticTassels).\n"
                "  --cpu | --gpu               Capture backend (default: --cpu).\n"
                "  --realistic-noise | --no-realistic-noise\n"
                "                              Toggle stochastic scanner noise profile.\n"
                "  --jitter | --no-jitter      Toggle per-sample scanner/capture jitter.\n"
                "  --export-mesh | --no-export-mesh\n"
                "  --export-point-cloud | --no-export-point-cloud\n"
                "  --export-flow-graph | --no-export-flow-graph\n"
                "  --export-node-graph | --no-export-node-graph\n"
                "  --open-output-folder | --no-open-output-folder\n"
                "  --help                      Print this help message.");
}

bool ParseGeneratorRunOptions(const int argc,
                              char** argv,
                              GeneratorRunOptions& options,
                              bool& show_help,
                              std::string& error_message) {
  show_help = false;
  for (int i = 1; i < argc; i++) {
    const std::string arg = argv[i];
    const auto require_value = [&](const char* flag) -> const char* {
      if (i + 1 >= argc) {
        error_message = std::string("Missing value for ") + flag;
        return nullptr;
      }
      i++;
      return argv[i];
    };

    if (arg == "--help" || arg == "-h") {
      show_help = true;
      return true;
    }

    if (arg == "--count") {
      const char* value = require_value("--count");
      if (!value) {
        return false;
      }
      if (!ParseUintArg(value, options.output_count)) {
        error_message = "Invalid value for --count: " + std::string(value);
        return false;
      }
      continue;
    }

    if (arg == "--index-start") {
      const char* value = require_value("--index-start");
      if (!value) {
        return false;
      }
      if (!ParseUintArg(value, options.sample_index_start)) {
        error_message = "Invalid value for --index-start: " + std::string(value);
        return false;
      }
      continue;
    }

    if (arg == "--seed") {
      const char* value = require_value("--seed");
      if (!value) {
        return false;
      }
      if (!ParseUintArg(value, options.dataset_seed)) {
        error_message = "Invalid value for --seed: " + std::string(value);
        return false;
      }
      continue;
    }

    if (arg == "--seed-hashed") {
      options.use_hashed_seed = true;
      continue;
    }
    if (arg == "--seed-incremental") {
      options.use_hashed_seed = false;
      continue;
    }

    if (arg == "--target-points") {
      const char* value = require_value("--target-points");
      if (!value) {
        return false;
      }
      if (!ParseUintArg(value, options.target_points)) {
        error_message = "Invalid value for --target-points: " + std::string(value);
        return false;
      }
      options.force_target_points = true;
      continue;
    }
    if (arg == "--no-target-points-override") {
      options.force_target_points = false;
      continue;
    }

    if (arg == "--output-folder") {
      const char* value = require_value("--output-folder");
      if (!value) {
        return false;
      }
      options.output_folder_override = std::filesystem::path(value);
      continue;
    }

    if (arg == "--cpu") {
      options.use_gpu_capture = false;
      continue;
    }
    if (arg == "--gpu") {
      options.use_gpu_capture = true;
      continue;
    }

    if (arg == "--realistic-noise") {
      options.realistic_noise = true;
      continue;
    }
    if (arg == "--no-realistic-noise") {
      options.realistic_noise = false;
      continue;
    }

    if (arg == "--jitter") {
      options.apply_per_seed_scanner_jitter = true;
      continue;
    }
    if (arg == "--no-jitter") {
      options.apply_per_seed_scanner_jitter = false;
      continue;
    }

    if (arg == "--export-mesh") {
      options.export_mesh = true;
      continue;
    }
    if (arg == "--no-export-mesh") {
      options.export_mesh = false;
      continue;
    }

    if (arg == "--export-point-cloud") {
      options.export_point_cloud = true;
      continue;
    }
    if (arg == "--no-export-point-cloud") {
      options.export_point_cloud = false;
      continue;
    }

    if (arg == "--export-flow-graph") {
      options.export_flow_graph = true;
      continue;
    }
    if (arg == "--no-export-flow-graph") {
      options.export_flow_graph = false;
      continue;
    }

    if (arg == "--export-node-graph") {
      options.export_node_graph = true;
      continue;
    }
    if (arg == "--no-export-node-graph") {
      options.export_node_graph = false;
      continue;
    }

    if (arg == "--open-output-folder") {
      options.open_output_folder = true;
      continue;
    }
    if (arg == "--no-open-output-folder") {
      options.open_output_folder = false;
      continue;
    }

    error_message = "Unknown option: " + arg;
    return false;
  }

  if (options.output_count == 0) {
    error_message = "--count must be greater than 0.";
    return false;
  }

  if (!options.export_point_cloud &&
      !options.export_flow_graph &&
      !options.export_node_graph &&
      !options.export_mesh) {
    error_message = "At least one export output must be enabled.";
    return false;
  }

  if (options.force_target_points && options.target_points == 0) {
    error_message = "--target-points must be greater than 0 when override is enabled.";
    return false;
  }

  return true;
}
}  // namespace

void register_classes() {
#ifdef DATASET_GENERATION_PLUGIN
  AssetRegistration<TasselPointCloudScannerDescriptor>("TasselPointCloudScannerDescriptor", {".tscan"});
  PrivateComponentRegistration<TasselPointCloudScanner>("TasselPointCloudScanner");
#endif

#ifdef LSYSTEM_PLUGIN
  AssetRegistration<l_system_plugin::MaizeTasselDescriptor>("MaizeTasselDescriptor", {".mtassel"});
  PrivateComponentRegistration<l_system_plugin::MaizeTassel>("MaizeTassel");
#endif
}

std::shared_ptr<Scene> ResolveGeneratorSceneAsset() {
  constexpr auto baseline_scene_name = "DigitalAgriculture.evescene";
  constexpr auto generator_scene_name = "DigitalAgriculture_MaizeGenerator.evescene";

  const std::filesystem::path generator_scene_relative_path = generator_scene_name;
  const std::filesystem::path generator_scene_absolute_path =
      ProjectManager::GetAssetsFolderPath() / generator_scene_relative_path;

  if (!std::filesystem::exists(generator_scene_absolute_path)) {
    const auto baseline_scene =
        std::dynamic_pointer_cast<Scene>(ProjectManager::GetOrCreateAsset(baseline_scene_name));
    if (!baseline_scene) {
      EVOENGINE_ERROR("Unable to resolve baseline scene asset: " + std::string(baseline_scene_name));
      return nullptr;
    }

    const auto cloned_scene = AssetManager::CreateTemporaryAsset<Scene>();
    Scene::Clone(baseline_scene, cloned_scene);

    if (!cloned_scene->SetPathAndSave(generator_scene_relative_path)) {
      EVOENGINE_ERROR("Failed to create isolated generator scene; falling back to baseline scene.");
      return baseline_scene;
    }

    EVOENGINE_LOG("Created isolated generator scene asset: " + std::string(generator_scene_name));
  }

  const auto generator_scene =
      std::dynamic_pointer_cast<Scene>(ProjectManager::GetOrCreateAsset(generator_scene_relative_path));
  if (!generator_scene) {
    EVOENGINE_ERROR("Unable to resolve isolated generator scene asset: " + std::string(generator_scene_name));
  }
  return generator_scene;
}

void run_with_editor(const std::filesystem::path& project_path) {
  if (std::filesystem::path(project_path).extension().string() != ".eveproj") {
    EVOENGINE_ERROR("Project path doesn't point to a EvoEngine project!");
    return;
  }
  register_classes();

  Application::PushLayer<RenderLayer>("Render Layer");
  Application::PushLayer<WindowLayer>("Window Layer");

  Application::PushLayer<EditorLayer>("Editor Layer");
#ifdef DIGITAL_AGRICULTURE_PLUGIN
  Application::PushLayer<MaizeLayer>("Maize Layer");
#endif

  ApplicationInitializationSettings application_info{};
  application_info.application_name = "MaizeDataGenerator";
  application_info.project_path = project_path;
  Application::Initialize(application_info);
  const auto new_scene = ResolveGeneratorSceneAsset();
  if (!new_scene) {
    EVOENGINE_ERROR("Failed to resolve scene for MaizeDataGenerator.");
    return;
  }
  Application::Attach(new_scene);
  Application::Start();
}

void run_windowless(const PointCloudCaptureSettings::CaptureMode capture_mode,
                    const std::filesystem::path& project_path) {
  if (std::filesystem::path(project_path).extension().string() != ".eveproj") {
    EVOENGINE_ERROR("Project path doesn't point to a EvoEngine project!");
    return;
  }
  register_classes();
  switch (capture_mode) {
    case PointCloudCaptureSettings::CaptureMode::Cpu: {
#ifdef DIGITAL_AGRICULTURE_PLUGIN
      Application::PushLayer<MaizeLayer>("Maize Layer");
#endif
    } break;
    case PointCloudCaptureSettings::CaptureMode::Gpu:
      Application::PushLayer<RenderLayer>("Render Layer");
#ifdef DIGITAL_AGRICULTURE_PLUGIN
      Application::PushLayer<MaizeLayer>("Maize Layer");
#endif
      break;
  }
  ApplicationInitializationSettings application_info{};
  application_info.application_name = "MaizeDataGenerator";
  application_info.project_path = project_path;
  Application::Initialize(application_info);
  const auto new_scene = ResolveGeneratorSceneAsset();
  if (!new_scene) {
    EVOENGINE_ERROR("Failed to resolve scene for MaizeDataGenerator.");
    return;
  }
  Application::Attach(new_scene);
  Application::Start();
}

#ifdef LSYSTEM_PLUGIN
// [deprecated] Legacy preset randomization path retained for quick A/B comparison.
enum class TasselSamplingQualityPreset {
  Medium,
  High,
  Ultra,
};

// [deprecated] Legacy descriptor discovery retained for fallback workflows.
std::filesystem::path ResolveTasselDescriptorPath(const std::filesystem::path& project_assets_folder) {
  const std::array<std::filesystem::path, 3> descriptor_candidates = {
      std::filesystem::path("LSystem") / "TasselSample.mtassel",
      std::filesystem::path("LSystem") / "New MaizeTasselDescriptor.mtassel",
      "New MaizeTasselDescriptor.mtassel"};

  for (const auto& relative_candidate : descriptor_candidates) {
    if (std::filesystem::exists(project_assets_folder / relative_candidate)) {
      return relative_candidate;
    }
  }

  return {};
}

uint32_t MixSeed(const int seed, const uint32_t salt) {
  uint32_t x = static_cast<uint32_t>(seed) ^ salt;
  x ^= x >> 16;
  x *= 0x7feb352du;
  x ^= x >> 15;
  x *= 0x846ca68bu;
  x ^= x >> 16;
  return x;
}

float SampleUniform(std::mt19937& rng, const float min_v, const float max_v) {
  std::uniform_real_distribution<float> dist(min_v, max_v);
  return dist(rng);
}

int SampleUniformInt(std::mt19937& rng, const int min_v, const int max_v) {
  std::uniform_int_distribution<int> dist(min_v, max_v);
  return dist(rng);
}

std::string ToString(const TasselScanMode mode) {
  switch (mode) {
    case TasselScanMode::Hemisphere:
      return "Hemisphere";
    case TasselScanMode::Gantry:
      return "Gantry";
    case TasselScanMode::Circular:
      return "Circular";
  }
  return "Unknown";
}

constexpr float kModerateJitterPercent = 0.10f;
constexpr int kModerateIntegerJitter = 2;

float JitterByPercent(std::mt19937& rng, const float baseline, const float percent,
                      const float min_v, const float max_v = std::numeric_limits<float>::max()) {
  const float jitter_radius = std::max(1e-4f, std::abs(baseline) * percent);
  const float jittered = SampleUniform(rng, baseline - jitter_radius, baseline + jitter_radius);
  return std::clamp(jittered, min_v, max_v);
}

int JitterByIntegerDelta(std::mt19937& rng, const int baseline, const int jitter_delta,
                         const int min_v, const int max_v) {
  const int jittered = SampleUniformInt(rng, baseline - jitter_delta, baseline + jitter_delta);
  return std::clamp(jittered, min_v, max_v);
}

template <typename T>
std::optional<Entity> FindFirstEnabledOwner(const std::shared_ptr<Scene>& scene,
                                            const std::string& component_label,
                                            const bool emit_error = true) {
  const auto* owners = scene->UnsafeGetPrivateComponentOwnersList<T>();
  if (!owners || owners->empty()) {
    if (emit_error) {
      EVOENGINE_ERROR("No " + component_label + " entities found in scene.");
    }
    return std::nullopt;
  }

  size_t valid_count = 0;
  size_t enabled_count = 0;
  std::optional<Entity> selected{};

  for (const auto& owner : *owners) {
    if (!scene->IsEntityValid(owner)) {
      continue;
    }
    valid_count++;
    if (scene->IsEntityEnabled(owner)) {
      enabled_count++;
      if (!selected.has_value()) {
        selected = owner;
      }
    }
  }

  if (!selected.has_value()) {
    if (emit_error) {
      EVOENGINE_ERROR("Unable to select an enabled " + component_label + " owner.");
    }
    return std::nullopt;
  }

  if (valid_count > 1) {
    EVOENGINE_LOG("Found " + std::to_string(valid_count) + " valid " + component_label +
                  " owners (enabled=" + std::to_string(enabled_count) + ") and selected entity index=" +
                  std::to_string(selected->GetIndex()) + ".");
  }

  return selected;
}

struct SceneTasselGenerationContext {
  Entity scanner_entity{};
  Entity tassel_entity{};
  TasselPointCloudPointSettings scanner_point_baseline{};
  TasselPointCloudGridCaptureSettings scanner_capture_baseline{};
};

enum class TasselGenerationContextMode {
  Auto,
  SceneRequired,
  DescriptorOnly,
};

const char* ToString(const TasselGenerationContextMode mode) {
  switch (mode) {
    case TasselGenerationContextMode::Auto:
      return "Auto";
    case TasselGenerationContextMode::SceneRequired:
      return "SceneRequired";
    case TasselGenerationContextMode::DescriptorOnly:
      return "DescriptorOnly";
  }
  return "Unknown";
}

SceneTasselGenerationContext BuildDefaultTasselGenerationContext(
    const PointCloudCaptureSettings::CaptureMode capture_mode) {
  SceneTasselGenerationContext context{};
  context.scanner_point_baseline = TasselPointCloudPointSettings{};
  context.scanner_capture_baseline = TasselPointCloudGridCaptureSettings{};
  context.scanner_capture_baseline.capture_mode = capture_mode;
  return context;
}

std::optional<SceneTasselGenerationContext> ResolveSceneTasselGenerationContext(
    const PointCloudCaptureSettings::CaptureMode capture_mode,
    const bool emit_error = true) {
  const auto scene = Application::GetActiveScene();
  if (!scene) {
    if (emit_error) {
      EVOENGINE_ERROR("No active scene available for tassel generation context resolution.");
    }
    return std::nullopt;
  }

  const auto scanner_entity = FindFirstEnabledOwner<TasselPointCloudScanner>(scene, "TasselPointCloudScanner",
                                                                              emit_error);
  if (!scanner_entity.has_value()) {
    return std::nullopt;
  }

  const auto tassel_entity = FindFirstEnabledOwner<l_system_plugin::MaizeTassel>(scene, "MaizeTassel",
                                                                                   emit_error);
  if (!tassel_entity.has_value()) {
    return std::nullopt;
  }

  const auto scanner = scene->GetOrSetPrivateComponent<TasselPointCloudScanner>(scanner_entity.value()).lock();
  if (!scanner) {
    if (emit_error) {
      EVOENGINE_ERROR("Unable to access selected in-scene TasselPointCloudScanner component.");
    }
    return std::nullopt;
  }

  SceneTasselGenerationContext context{};
  context.scanner_entity = scanner_entity.value();
  context.tassel_entity = tassel_entity.value();
  if (!scanner->TryGetActiveSettings(context.scanner_point_baseline, context.scanner_capture_baseline)) {
    if (emit_error) {
      EVOENGINE_ERROR("Failed to extract baseline settings from in-scene TasselPointCloudScanner.");
    }
    return std::nullopt;
  }

  context.scanner_capture_baseline.capture_mode = capture_mode;
  EVOENGINE_LOG("Using scene scanner baseline: scan_mode=" + ToString(context.scanner_capture_baseline.scan_mode) +
                ", step=" + std::to_string(context.scanner_capture_baseline.step) +
                ", scan_resolution=" + std::to_string(context.scanner_capture_baseline.scan_resolution) +
                ", dropout=" + std::to_string(context.scanner_point_baseline.dropout_probability));
  return context;
}

void ApplyPerSeedBaselineScannerJitter(
    DatasetGenerator::TasselDataGenerationParameters& data_generation_parameters,
    const TasselPointCloudPointSettings& baseline_point_settings,
    const TasselPointCloudGridCaptureSettings& baseline_capture_settings,
    const int sample_seed,
    const bool realistic_noise,
    const PointCloudCaptureSettings::CaptureMode capture_mode) {
  auto jittered_capture_settings =
      std::make_shared<TasselPointCloudGridCaptureSettings>(baseline_capture_settings);
  jittered_capture_settings->capture_mode = capture_mode;

  std::mt19937 capture_rng(MixSeed(sample_seed, 0xA11CE5EDu));
  jittered_capture_settings->bounding_box_size =
      JitterByPercent(capture_rng, baseline_capture_settings.bounding_box_size, kModerateJitterPercent, 0.05f);
  jittered_capture_settings->grid_size.x =
      JitterByIntegerDelta(capture_rng, baseline_capture_settings.grid_size.x, kModerateIntegerJitter, 1, 256);
  jittered_capture_settings->grid_size.y =
      JitterByIntegerDelta(capture_rng, baseline_capture_settings.grid_size.y, kModerateIntegerJitter, 1, 256);
  jittered_capture_settings->grid_distance =
      JitterByPercent(capture_rng, baseline_capture_settings.grid_distance, kModerateJitterPercent, 0.001f);
  jittered_capture_settings->step =
      JitterByPercent(capture_rng, baseline_capture_settings.step, kModerateJitterPercent, 0.0001f);
  jittered_capture_settings->samples_per_step =
      JitterByIntegerDelta(capture_rng, baseline_capture_settings.samples_per_step, kModerateIntegerJitter, 1,
                           1 << 20);
  jittered_capture_settings->sample_height =
      JitterByPercent(capture_rng, baseline_capture_settings.sample_height, kModerateJitterPercent, 0.001f);
  jittered_capture_settings->scan_center = baseline_capture_settings.scan_center;
  jittered_capture_settings->look_target_height =
      JitterByPercent(capture_rng, baseline_capture_settings.look_target_height, kModerateJitterPercent,
                      -10000.0f, 10000.0f);
  jittered_capture_settings->scan_mode = baseline_capture_settings.scan_mode;

  jittered_capture_settings->scanner_angles.clear();
  jittered_capture_settings->scanner_angles.reserve(baseline_capture_settings.scanner_angles.size());
  for (const auto baseline_angle : baseline_capture_settings.scanner_angles) {
    jittered_capture_settings->scanner_angles.emplace_back(
        JitterByPercent(capture_rng, baseline_angle, kModerateJitterPercent, -89.0f, 89.0f));
  }
  if (jittered_capture_settings->scanner_angles.empty()) {
    jittered_capture_settings->scanner_angles.emplace_back(30.0f);
  }

  jittered_capture_settings->scanner_distance =
      JitterByPercent(capture_rng, baseline_capture_settings.scanner_distance, kModerateJitterPercent, 0.01f);
  jittered_capture_settings->pitch_angle_start =
      JitterByIntegerDelta(capture_rng, baseline_capture_settings.pitch_angle_start, kModerateIntegerJitter, -89,
                           89);
  jittered_capture_settings->pitch_angle_end =
      JitterByIntegerDelta(capture_rng, baseline_capture_settings.pitch_angle_end, kModerateIntegerJitter, -89, 89);
  if (jittered_capture_settings->pitch_angle_end <= jittered_capture_settings->pitch_angle_start) {
    jittered_capture_settings->pitch_angle_end = std::min(89, jittered_capture_settings->pitch_angle_start + 1);
  }
  jittered_capture_settings->pitch_angle_step =
      JitterByIntegerDelta(capture_rng, baseline_capture_settings.pitch_angle_step, kModerateIntegerJitter, 1, 360);
  jittered_capture_settings->turn_angle_start =
      JitterByIntegerDelta(capture_rng, baseline_capture_settings.turn_angle_start, kModerateIntegerJitter, 0, 360);
  jittered_capture_settings->turn_angle_end =
      JitterByIntegerDelta(capture_rng, baseline_capture_settings.turn_angle_end, kModerateIntegerJitter, 0, 360);
  if (jittered_capture_settings->turn_angle_end <= jittered_capture_settings->turn_angle_start) {
    jittered_capture_settings->turn_angle_end = std::min(360, jittered_capture_settings->turn_angle_start + 1);
  }
  jittered_capture_settings->turn_angle_step =
      JitterByIntegerDelta(capture_rng, baseline_capture_settings.turn_angle_step, kModerateIntegerJitter, 1, 360);
  jittered_capture_settings->fov =
      JitterByPercent(capture_rng, baseline_capture_settings.fov, kModerateJitterPercent, 10.0f, 179.0f);
  jittered_capture_settings->scan_resolution =
      JitterByIntegerDelta(capture_rng, baseline_capture_settings.scan_resolution, kModerateIntegerJitter, 1,
                           1 << 15);

  auto& point_settings = data_generation_parameters.tassel_point_cloud_point_settings;
  point_settings = baseline_point_settings;
  std::mt19937 point_rng(MixSeed(sample_seed, 0x51DE5EEDu));

    point_settings.distance_sigma_scale =
      JitterByPercent(point_rng, baseline_point_settings.distance_sigma_scale, kModerateJitterPercent, 0.0f);
    point_settings.hit_ball_jitter_radius =
      JitterByPercent(point_rng, baseline_point_settings.hit_ball_jitter_radius, kModerateJitterPercent, 0.0f);
  if (realistic_noise) {
    point_settings.range_noise_base_sigma =
        JitterByPercent(point_rng, baseline_point_settings.range_noise_base_sigma, kModerateJitterPercent, 0.0f);
    point_settings.range_noise_scale =
        JitterByPercent(point_rng, baseline_point_settings.range_noise_scale, kModerateJitterPercent, 0.0f);
    point_settings.dropout_probability =
        JitterByPercent(point_rng, baseline_point_settings.dropout_probability, kModerateJitterPercent, 0.0f,
                        0.999f);
    point_settings.angular_noise_sigma =
        JitterByPercent(point_rng, baseline_point_settings.angular_noise_sigma, kModerateJitterPercent, 0.0f);
  } else {
    point_settings.range_noise_base_sigma = 0.0f;
    point_settings.range_noise_scale = 0.0f;
    point_settings.dropout_probability = 0.0f;
    point_settings.angular_noise_sigma = 0.0f;
  }

  point_settings.min_range =
      JitterByPercent(point_rng, baseline_point_settings.min_range, kModerateJitterPercent, 0.0f);
  if (baseline_point_settings.max_range > 0.0f) {
    point_settings.max_range =
        JitterByPercent(point_rng, baseline_point_settings.max_range, kModerateJitterPercent,
                        point_settings.min_range + 1e-5f);
  } else {
    point_settings.max_range = 0.0f;
  }
  point_settings.bounding_box_limit =
      JitterByPercent(point_rng, baseline_point_settings.bounding_box_limit, kModerateJitterPercent, 0.01f);
  point_settings.color_output = baseline_point_settings.color_output;
  point_settings.type_index = baseline_point_settings.type_index;
  point_settings.instance_index = baseline_point_settings.instance_index;

  data_generation_parameters.point_cloud_capture_settings = jittered_capture_settings;
}

void ApplyBaselineScannerSettings(
    DatasetGenerator::TasselDataGenerationParameters& data_generation_parameters,
    const TasselPointCloudPointSettings& baseline_point_settings,
    const TasselPointCloudGridCaptureSettings& baseline_capture_settings,
    const bool realistic_noise,
    const PointCloudCaptureSettings::CaptureMode capture_mode) {
  auto capture_settings = std::make_shared<TasselPointCloudGridCaptureSettings>(baseline_capture_settings);
  capture_settings->capture_mode = capture_mode;
  data_generation_parameters.point_cloud_capture_settings = capture_settings;

  auto& point_settings = data_generation_parameters.tassel_point_cloud_point_settings;
  point_settings = baseline_point_settings;

  if (!realistic_noise) {
    point_settings.range_noise_base_sigma = 0.0f;
    point_settings.range_noise_scale = 0.0f;
    point_settings.dropout_probability = 0.0f;
    point_settings.angular_noise_sigma = 0.0f;
  }
}

// [deprecated] Legacy quality presets retained for fallback workflows.
void ApplyTasselCapturePreset(
  const std::shared_ptr<TasselPointCloudGridCaptureSettings>& capture_settings,
  const TasselSamplingQualityPreset preset,
  const PointCloudCaptureSettings::CaptureMode capture_mode) {
  capture_settings->capture_mode = capture_mode;
  capture_settings->grid_distance = 0.5f;
  capture_settings->sample_height = 1.0f;
  capture_settings->scanner_angles = {0.0f, 15.0f, 30.0f, 45.0f, 60.0f};

  switch (preset) {
    case TasselSamplingQualityPreset::Medium: {
      capture_settings->scan_mode = TasselScanMode::Hemisphere;
      capture_settings->grid_size = {1, 1};
      capture_settings->step = 0.004f;
      capture_settings->samples_per_step = 384;
      capture_settings->bounding_box_size = 2.0f;
      capture_settings->scan_resolution = 128;
    } break;
    case TasselSamplingQualityPreset::High: {
      capture_settings->scan_mode = TasselScanMode::Hemisphere;
      capture_settings->grid_size = {2, 2};
      capture_settings->step = 0.0025f;
      capture_settings->samples_per_step = 768;
      capture_settings->bounding_box_size = 2.5f;
      capture_settings->scan_resolution = 160;
    } break;
    case TasselSamplingQualityPreset::Ultra: {
      capture_settings->scan_mode = TasselScanMode::Circular;
      capture_settings->grid_size = {1, 1};
      capture_settings->step = 0.0020f;
      capture_settings->samples_per_step = 1024;
      capture_settings->bounding_box_size = 3.0f;
      capture_settings->scanner_distance = 0.8f;
      capture_settings->pitch_angle_start = -45;
      capture_settings->pitch_angle_end = 75;
      capture_settings->pitch_angle_step = 5;
      capture_settings->turn_angle_start = 0;
      capture_settings->turn_angle_end = 360;
      capture_settings->turn_angle_step = 5;
      capture_settings->fov = 50.0f;
      capture_settings->scan_resolution = 192;
    } break;
  }
}

// [deprecated] Legacy fixed noise profile retained for fallback workflows.
void ApplyTasselPointCloudNoiseProfile(
  DatasetGenerator::TasselDataGenerationParameters& data_generation_parameters,
  const bool realistic_noise) {
  auto& settings = data_generation_parameters.tassel_point_cloud_point_settings;
  settings.type_index = true;
  settings.instance_index = true;
  settings.color_output = true;
  settings.bounding_box_limit = 10.0f;

  // Legacy noise is disabled for consistent geometric labels.
  settings.hit_ball_jitter_radius = 0.0f;
  settings.distance_sigma_scale = 0.0f;

  if (realistic_noise) {
    settings.range_noise_base_sigma = 0.001f;
    settings.range_noise_scale = 0.0005f;
    settings.dropout_probability = 0.05f;
    settings.angular_noise_sigma = 0.0002f;
  } else {
    settings.range_noise_base_sigma = 0.0f;
    settings.range_noise_scale = 0.0f;
    settings.dropout_probability = 0.0f;
    settings.angular_noise_sigma = 0.0f;
  }

  settings.min_range = 0.0f;
  settings.max_range = 0.0f;
}

// [deprecated] Legacy broad photowalk randomization retained for fallback workflows.
void ApplyPerSeedPhotowalkCaptureSettings(
  const std::shared_ptr<TasselPointCloudGridCaptureSettings>& capture_settings,
  const int sample_seed,
  const PointCloudCaptureSettings::CaptureMode capture_mode) {
  std::mt19937 rng(MixSeed(sample_seed, 0xA11CE5EDu));
  capture_settings->capture_mode = capture_mode;

  // Mostly circular trajectories (walking around plants), occasionally gantry-like strips.
  const bool use_circular = SampleUniform(rng, 0.0f, 1.0f) < 0.85f;

  if (use_circular) {
    capture_settings->scan_mode = TasselScanMode::Circular;
    capture_settings->grid_size = {1, 1};
    capture_settings->grid_distance = 0.5f;
    capture_settings->step = SampleUniform(rng, 0.003f, 0.010f);
    capture_settings->samples_per_step = SampleUniformInt(rng, 128, 512);
    capture_settings->sample_height = SampleUniform(rng, 0.3f, 1.2f);

    capture_settings->scanner_distance = SampleUniform(rng, 0.6f, 1.8f);
    capture_settings->pitch_angle_start = SampleUniformInt(rng, -20, 5);
    capture_settings->pitch_angle_end = SampleUniformInt(rng, 20, 55);
    capture_settings->pitch_angle_end = std::max(capture_settings->pitch_angle_end,
                                                 capture_settings->pitch_angle_start + 10);
    capture_settings->pitch_angle_step = SampleUniformInt(rng, 8, 20);

    const int turn_span = SampleUniformInt(rng, 140, 320);
    capture_settings->turn_angle_start = SampleUniformInt(rng, 0, 360 - turn_span);
    capture_settings->turn_angle_end = capture_settings->turn_angle_start + turn_span;
    capture_settings->turn_angle_step = SampleUniformInt(rng, 6, 20);

    capture_settings->fov = SampleUniform(rng, 55.0f, 95.0f);
    capture_settings->scan_resolution = SampleUniformInt(rng, 48, 140);

    // Keep bound broad enough to avoid clipping large tassels while still limiting far outliers.
    const float radial_extent = capture_settings->scanner_distance + capture_settings->sample_height + 1.5f;
    capture_settings->bounding_box_size = std::clamp(radial_extent, 2.5f, 8.0f);
  } else {
    capture_settings->scan_mode = TasselScanMode::Gantry;
    capture_settings->grid_size = {SampleUniformInt(rng, 1, 2), SampleUniformInt(rng, 1, 2)};
    capture_settings->grid_distance = SampleUniform(rng, 0.4f, 0.9f);
    capture_settings->step = SampleUniform(rng, 0.004f, 0.012f);
    capture_settings->samples_per_step = SampleUniformInt(rng, 96, 512);
    capture_settings->sample_height = SampleUniform(rng, 0.6f, 1.8f);
    capture_settings->bounding_box_size = SampleUniform(rng, 4.0f, 8.0f);

    const int angle_count = SampleUniformInt(rng, 2, 5);
    const float min_angle = SampleUniform(rng, 15.0f, 35.0f);
    float max_angle = SampleUniform(rng, 45.0f, 75.0f);
    max_angle = std::max(max_angle, min_angle + 5.0f);
    capture_settings->scanner_angles.clear();
    if (angle_count == 1) {
      capture_settings->scanner_angles.emplace_back(min_angle);
    } else {
      for (int i = 0; i < angle_count; i++) {
        const float t = static_cast<float>(i) / static_cast<float>(angle_count - 1);
        capture_settings->scanner_angles.emplace_back(min_angle + t * (max_angle - min_angle));
      }
    }

    capture_settings->scanner_distance = 0.8f;
    capture_settings->pitch_angle_start = -30;
    capture_settings->pitch_angle_end = 60;
    capture_settings->pitch_angle_step = 10;
    capture_settings->turn_angle_start = 0;
    capture_settings->turn_angle_end = 360;
    capture_settings->turn_angle_step = 10;
    capture_settings->fov = 60.0f;
    capture_settings->scan_resolution = SampleUniformInt(rng, 64, 160);
  }
}

// [deprecated] Legacy broad noise randomization retained for fallback workflows.
void ApplyPerSeedTasselPointCloudNoiseProfile(
    DatasetGenerator::TasselDataGenerationParameters& data_generation_parameters,
    const bool realistic_noise,
    const int sample_seed) {
  auto& settings = data_generation_parameters.tassel_point_cloud_point_settings;
  settings.type_index = true;
  settings.instance_index = true;
  settings.color_output = true;

  // Legacy noise is disabled for consistent geometric labels.
  settings.hit_ball_jitter_radius = 0.0f;
  settings.distance_sigma_scale = 0.0f;

  if (realistic_noise) {
    std::mt19937 rng(MixSeed(sample_seed, 0x51DE5EEDu));
    settings.range_noise_base_sigma = SampleUniform(rng, 0.0012f, 0.0120f);
    settings.range_noise_scale = SampleUniform(rng, 0.0006f, 0.0060f);
    settings.dropout_probability = SampleUniform(rng, 0.15f, 0.60f);
    settings.angular_noise_sigma = SampleUniform(rng, 0.0002f, 0.0035f);
    settings.min_range = SampleUniform(rng, 0.0f, 0.35f);
    settings.max_range = 0.0f;
    settings.bounding_box_limit = SampleUniform(rng, 3.0f, 12.0f);
  } else {
    settings.range_noise_base_sigma = 0.0f;
    settings.range_noise_scale = 0.0f;
    settings.dropout_probability = 0.0f;
    settings.angular_noise_sigma = 0.0f;
    settings.min_range = 0.0f;
    settings.max_range = 0.0f;
    settings.bounding_box_limit = 4.0f;
  }
}

void tassel_mesh_point_cloud_skeleton(const uint32_t output_size,
                                      const uint32_t sample_index_start,
                                      const SceneTasselGenerationContext& scene_context,
                                      const bool use_existing_scene_entities,
                                      const PointCloudCaptureSettings::CaptureMode capture_mode,
                                      const std::filesystem::path& output_folder,
                                      const bool realistic_noise = false,
                                      const uint32_t dataset_seed = 42u,
                                      const bool use_hashed_seed = true,
                                      const bool apply_per_seed_scanner_jitter = true,
                                      const bool force_target_points = true,
                                      const uint32_t target_points = 8000u,
                                      const bool export_mesh = true,
                                      const bool export_point_cloud = true,
                                      const bool export_flow_graph = true,
                                      const bool export_node_graph = true) {
#ifndef DATASET_GENERATION_PLUGIN
  throw std::runtime_error("DatasetGeneration plugin missing!");
#endif

  std::filesystem::create_directories(output_folder);

  DatasetGenerator::TasselDataGenerationParameters data_generation_parameters{};
  data_generation_parameters.export_mesh = export_mesh;
  data_generation_parameters.export_point_cloud = export_point_cloud;
  data_generation_parameters.export_flow_graph = export_flow_graph;
  data_generation_parameters.export_node_graph = export_node_graph;
  data_generation_parameters.max_growth_steps_per_frame = 0;
  data_generation_parameters.uncapped_growth = true;
  data_generation_parameters.use_existing_scene_entities = use_existing_scene_entities;
  if (use_existing_scene_entities) {
    data_generation_parameters.scene_tassel_entity = scene_context.tassel_entity;
    data_generation_parameters.scene_scanner_entity = scene_context.scanner_entity;
  }
  data_generation_parameters.output_folder = output_folder;

  constexpr float kMinTargetGdd = 500.0f;
  constexpr float kMaxTargetGdd = 3000.0f;
  constexpr float kTargetGddBiasPower = 2.0f;

  const auto sample_biased_target_gdd = [&](const uint32_t sample_index) {
    if (output_size <= 1) {
      return kMinTargetGdd;
    }

    const float t = static_cast<float>(sample_index) / static_cast<float>(output_size - 1);
    const float biased_t = std::pow(t, kTargetGddBiasPower);
    return kMinTargetGdd + (kMaxTargetGdd - kMinTargetGdd) * biased_t;
  };

  for (uint32_t i = 0; i < output_size; i++) {
    const uint32_t sample_index = sample_index_start + i;
    const uint32_t sampled_seed = use_hashed_seed
                                      ? MixSeed(static_cast<int>(sample_index), dataset_seed)
                                      : (dataset_seed + sample_index);
    const int sample_seed = static_cast<int>(sampled_seed & 0x7FFFFFFFu);
    data_generation_parameters.seed = sample_seed;

    data_generation_parameters.target_gdd = sample_biased_target_gdd(i);

    if (apply_per_seed_scanner_jitter) {
      ApplyPerSeedBaselineScannerJitter(
        data_generation_parameters,
        scene_context.scanner_point_baseline,
        scene_context.scanner_capture_baseline,
        sample_seed,
        realistic_noise,
        capture_mode);
    } else {
      ApplyBaselineScannerSettings(
        data_generation_parameters,
        scene_context.scanner_point_baseline,
        scene_context.scanner_capture_baseline,
        realistic_noise,
        capture_mode);
    }

    auto sample_capture_settings = std::dynamic_pointer_cast<TasselPointCloudGridCaptureSettings>(
      data_generation_parameters.point_cloud_capture_settings);
    if (!sample_capture_settings) {
      EVOENGINE_ERROR("Failed to build per-seed TasselPointCloudGridCaptureSettings from scanner baseline.");
      return;
    }

    if (force_target_points) {
      // The scanner's point-budget enforcement is gated on Circular scan_mode
      // (see TasselPointCloudScanner.cpp budget_state setup). Hemisphere/Gantry
      // ignore target_points entirely, producing wildly over-budget PLYs
      // (10k-500k points). Force Circular here so --target-points actually
      // binds. Per-seed jitter still randomizes pitch/turn sweeps within
      // Circular for view-diversity.
      sample_capture_settings->scan_mode = TasselScanMode::Circular;
      sample_capture_settings->point_budget_enabled = true;
      sample_capture_settings->target_points = std::max<uint32_t>(1u, target_points);
      sample_capture_settings->target_points_deviation = 0;
    }

    data_generation_parameters.output_file_name = "Tassel_" + std::to_string(sample_index);
    EVOENGINE_LOG("Generating " + data_generation_parameters.output_file_name +
                  " index=" + std::to_string(sample_index) +
                  " seed=" + std::to_string(sample_seed) +
                  " gdd=" + std::to_string(data_generation_parameters.target_gdd) +
                  " scan_mode=" + ToString(sample_capture_settings->scan_mode) +
                  " step=" + std::to_string(sample_capture_settings->step) +
                  " scan_resolution=" + std::to_string(sample_capture_settings->scan_resolution) +
                  " dropout=" + std::to_string(data_generation_parameters.tassel_point_cloud_point_settings.dropout_probability));
    DatasetGenerator::GenerateDataForTassel(data_generation_parameters);
  }
}
#endif

int main(int argc, char** argv) {
  GeneratorRunOptions run_options{};
  bool show_help = false;
  std::string parse_error;
  if (!ParseGeneratorRunOptions(argc, argv, run_options, show_help, parse_error)) {
    EVOENGINE_ERROR(parse_error);
    PrintUsage();
    return 1;
  }
  if (show_help) {
    PrintUsage();
    return 0;
  }

  std::filesystem::path resource_folder_path("../../../../../Resources");
  if (!std::filesystem::exists(resource_folder_path)) {
    resource_folder_path = "../../../../Resources";
  }
  if (!std::filesystem::exists(resource_folder_path)) {
    resource_folder_path = "../../../Resources";
  }
  if (!std::filesystem::exists(resource_folder_path)) {
    resource_folder_path = "../../Resources";
  }
  if (!std::filesystem::exists(resource_folder_path)) {
    resource_folder_path = "../Resources";
  }
  resource_folder_path = std::filesystem::absolute(resource_folder_path);

  // Keep generator/editor layout state isolated in a maize-specific project.
  const std::filesystem::path project_path =
      std::filesystem::absolute(resource_folder_path / "DigitalAgricultureProject" / "maize_data_generator.eveproj");

  const auto capture_mode =
      run_options.use_gpu_capture ? PointCloudCaptureSettings::CaptureMode::Gpu
                                  : PointCloudCaptureSettings::CaptureMode::Cpu;

  run_windowless(capture_mode, project_path);
  const std::filesystem::path synthetic_dataset_root = resource_folder_path.parent_path() / "MaizeTasselSyntheticPointCloud";
  const std::filesystem::path output_folder_path =
      run_options.output_folder_override.empty()
          ? (synthetic_dataset_root / "SyntheticTassels")
          : std::filesystem::absolute(run_options.output_folder_override);

#ifdef LSYSTEM_PLUGIN
  constexpr TasselGenerationContextMode kTasselGenerationContextMode = TasselGenerationContextMode::Auto;

  bool use_existing_scene_entities = false;
  auto scene_tassel_context = BuildDefaultTasselGenerationContext(capture_mode);

  switch (kTasselGenerationContextMode) {
    case TasselGenerationContextMode::DescriptorOnly: {
      EVOENGINE_LOG("Tassel generation context mode: DescriptorOnly. "
                    "Using descriptor-based generation with default scanner baseline settings.");
    } break;
    case TasselGenerationContextMode::SceneRequired: {
      const auto resolved_context = ResolveSceneTasselGenerationContext(capture_mode, true);
      if (!resolved_context.has_value()) {
        EVOENGINE_ERROR("Failed to resolve in-scene scanner/tassel context in SceneRequired mode.");
        Application::Terminate();
        return 1;
      }
      scene_tassel_context = resolved_context.value();
      use_existing_scene_entities = true;
      EVOENGINE_LOG("Tassel generation context mode: SceneRequired. Using in-scene scanner/tassel entities.");
    } break;
    case TasselGenerationContextMode::Auto:
    default: {
      const auto resolved_context = ResolveSceneTasselGenerationContext(capture_mode, false);
      if (resolved_context.has_value()) {
        scene_tassel_context = resolved_context.value();
        use_existing_scene_entities = true;
        EVOENGINE_LOG("Tassel generation context mode: Auto. Using in-scene scanner/tassel entities.");
      } else {
        EVOENGINE_WARNING("Tassel generation context mode: Auto. "
                          "No in-scene scanner/tassel context found; falling back to descriptor-based generation with default scanner baselines.");
      }
    } break;
  }

  EVOENGINE_LOG("Resolved tassel generation mode=" + std::string(ToString(kTasselGenerationContextMode)) +
                ", use_existing_scene_entities=" + (use_existing_scene_entities ? "true" : "false"));

  const auto tassel_output_folder = output_folder_path;
  EVOENGINE_LOG("Synthetic tassel export folder: " + tassel_output_folder.string());
  EVOENGINE_LOG("Synthetic tassel sample count: " + std::to_string(run_options.output_count));
  EVOENGINE_LOG("Synthetic tassel sample index start: " + std::to_string(run_options.sample_index_start));
  EVOENGINE_LOG("Synthetic tassel dataset seed base: " + std::to_string(run_options.dataset_seed));
  EVOENGINE_LOG("Synthetic tassel seed mode: " +
                std::string(run_options.use_hashed_seed ? "hashed" : "incremental"));
  EVOENGINE_LOG("Target points override: " +
                std::string(run_options.force_target_points ? "enabled" : "disabled") +
                (run_options.force_target_points
                     ? (", target_points=" + std::to_string(run_options.target_points))
                     : ""));
  EVOENGINE_LOG("Scanner capture jitter: " + std::string(run_options.apply_per_seed_scanner_jitter ? "enabled" : "disabled"));
  EVOENGINE_LOG("Exports mesh/point/flow/node: " +
                std::string(run_options.export_mesh ? "1" : "0") + "/" +
                std::string(run_options.export_point_cloud ? "1" : "0") + "/" +
                std::string(run_options.export_flow_graph ? "1" : "0") + "/" +
                std::string(run_options.export_node_graph ? "1" : "0"));

  tassel_mesh_point_cloud_skeleton(run_options.output_count,
                                   run_options.sample_index_start,
                                   scene_tassel_context,
                                   use_existing_scene_entities,
                                   capture_mode,
                                   tassel_output_folder,
                                   run_options.realistic_noise,
                                   run_options.dataset_seed,
                                   run_options.use_hashed_seed,
                                   run_options.apply_per_seed_scanner_jitter,
                                   run_options.force_target_points,
                                   run_options.target_points,
                                   run_options.export_mesh,
                                   run_options.export_point_cloud,
                                   run_options.export_flow_graph,
                                   run_options.export_node_graph);
#endif

  EVOENGINE_LOG("Generation Finished!")

  // Open File Explorer for generated files.
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
  if (run_options.open_output_folder) {
    const auto folder_path = output_folder_path.string();
    ShellExecuteA(nullptr, "open", folder_path.c_str(), nullptr, nullptr, SW_SHOWDEFAULT);
  }
#endif
  Application::Terminate();
  return 0;
}
