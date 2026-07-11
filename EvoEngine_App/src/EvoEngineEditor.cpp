#include "AppBootstrap.hpp"
#include "Application.hpp"
#include "AssetManager.hpp"
#include "Camera.hpp"
#include "DemoProfiles.hpp"
#include "DemoScene.hpp"
#include "EditorLayer.hpp"
#include "PathUtils.hpp"
#include "Platform.hpp"
#include "PostProcessingStack.hpp"
#include "ProjectManager.hpp"
#include "RenderLayer.hpp"
#include "TextureStorage.hpp"
#include "Times.hpp"
#include "WindowLayer.hpp"
#include "nlohmann/json.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>

#ifdef EVOENGINE_WINDOWS
#  ifndef NOMINMAX
#    define NOMINMAX
#  endif
#  include <Windows.h>
#endif

using namespace evo_engine;

namespace {
struct EditorCommandLine {
  std::optional<std::filesystem::path> project_path;
  std::optional<DemoProfileId> demo_profile_id;
  std::optional<std::filesystem::path> demo_preview_capture_path;
  std::optional<std::filesystem::path> preview_capture_metrics_path;
  std::optional<GraphicsInitializationSettings::ShadowMapResolutionQuality> shadow_map_resolution_quality;
  ApplicationMode application_mode = ApplicationMode::Editor;
  bool application_mode_explicit = false;
  int preview_capture_width = 1280;
  int preview_capture_height = 720;
  size_t preview_capture_warmup_frames = 8;
  std::optional<Camera::CameraRenderMode> preview_capture_render_mode;
  std::optional<CameraSettings::ShaderExecutionReorderingMode> preview_capture_ser_mode;
  std::optional<bool> preview_capture_firefly_clamp_enabled;
  std::optional<float> preview_capture_firefly_clamp_threshold;
  std::optional<bool> preview_capture_emissive_triangle_nee_enabled;
  std::optional<bool> preview_capture_auto_spp_enabled;
  std::optional<int> preview_capture_auto_spp_min_samples;
  std::optional<int> preview_capture_auto_spp_max_samples;
  std::optional<float> preview_capture_auto_spp_convergence_threshold;
  std::optional<int> preview_capture_sample_size;
  std::optional<glm::vec3> preview_capture_camera_position;
  std::optional<glm::vec3> preview_capture_camera_look_at;
  std::optional<bool> preview_ambient_occlusion_enabled;
  std::optional<AmbientOcclusion::Algorithm> preview_ambient_occlusion_algorithm;
  std::optional<bool> preview_anti_aliasing_enabled;
  std::optional<AntiAliasing::Algorithm> preview_anti_aliasing_algorithm;
  std::optional<AntiAliasing::TaaPreset> preview_taa_preset;
  std::optional<AntiAliasing::SmaaPreset> preview_smaa_preset;
  std::optional<bool> preview_anti_aliasing_tgsm;
  std::optional<bool> preview_anti_aliasing_fp16;
  std::optional<bool> preview_anti_aliasing_motion_sequence;
  std::optional<AntiAliasing::TaaDebugMode> preview_taa_debug_mode;
  std::optional<AntiAliasing::SmaaDebugMode> preview_smaa_debug_mode;
  bool preview_anti_aliasing_debug_disabled = false;
  std::optional<float> preview_shadow_split_lambda;
  std::optional<float> preview_shadow_cascade_transition_width;
  std::optional<float> preview_shadow_distance_fade;
  std::optional<int> preview_shadow_debug_mode;
  std::optional<int> preview_shadow_debug_cascade;
  std::optional<int> preview_shadow_debug_light;
  bool preview_capture_deterministic = false;
  bool preview_capture_bistro_ddgi = false;
  bool disable_ray_tracing_pipeline = false;
};

Camera::CameraRenderMode ParsePreviewRenderMode(const std::string& value) {
  const auto raster_fallback = Camera::ParseCameraRenderMode(value, Camera::CameraRenderMode::Rasterization);
  const auto ray_query_fallback = Camera::ParseCameraRenderMode(value, Camera::CameraRenderMode::RayQuery);
  if (raster_fallback != Camera::CameraRenderMode::Rasterization ||
      ray_query_fallback != Camera::CameraRenderMode::RayQuery) {
    return raster_fallback;
  }
  throw std::invalid_argument("Unknown preview render mode: " + value);
}

CameraSettings::ShaderExecutionReorderingMode ParsePreviewShaderExecutionReorderingMode(const std::string& value) {
  const auto disabled_fallback =
      Camera::ParseShaderExecutionReorderingMode(value, CameraSettings::ShaderExecutionReorderingMode::Disabled);
  const auto enabled_fallback =
      Camera::ParseShaderExecutionReorderingMode(value, CameraSettings::ShaderExecutionReorderingMode::Enabled);
  if (disabled_fallback != CameraSettings::ShaderExecutionReorderingMode::Disabled ||
      enabled_fallback != CameraSettings::ShaderExecutionReorderingMode::Enabled) {
    return disabled_fallback;
  }
  throw std::invalid_argument("Unknown preview SER mode: " + value);
}

bool ParsePreviewBool(const std::string& value, const std::string& argument) {
  auto normalized = value;
  std::transform(normalized.begin(), normalized.end(), normalized.begin(), [](const char character) {
    return static_cast<char>(std::tolower(static_cast<unsigned char>(character)));
  });
  if (normalized == "1" || normalized == "on" || normalized == "true" || normalized == "enabled" ||
      normalized == "enable") {
    return true;
  }
  if (normalized == "0" || normalized == "off" || normalized == "false" || normalized == "disabled" ||
      normalized == "disable") {
    return false;
  }
  throw std::invalid_argument(argument + " requires enabled or disabled.");
}

std::optional<AmbientOcclusion::Algorithm> ParsePreviewAmbientOcclusionAlgorithm(const std::string& value,
                                                                                 bool& enabled) {
  auto normalized = value;
  std::transform(normalized.begin(), normalized.end(), normalized.begin(), [](const char character) {
    return static_cast<char>(std::tolower(static_cast<unsigned char>(character)));
  });
  if (normalized == "ssao") {
    enabled = true;
    return AmbientOcclusion::Algorithm::Ssao;
  }
  if (normalized == "gtao") {
    enabled = true;
    return AmbientOcclusion::Algorithm::Gtao;
  }
  if (normalized == "0" || normalized == "off" || normalized == "false" || normalized == "disabled" ||
      normalized == "disable" || normalized == "none") {
    enabled = false;
    return {};
  }
  throw std::invalid_argument("--preview-ao requires ssao, gtao, or disabled.");
}

std::optional<AntiAliasing::Algorithm> ParsePreviewAntiAliasingAlgorithm(const std::string& value, bool& enabled) {
  auto normalized = value;
  std::transform(normalized.begin(), normalized.end(), normalized.begin(), [](const char character) {
    return static_cast<char>(std::tolower(static_cast<unsigned char>(character)));
  });
  if (normalized == "taa") {
    enabled = true;
    return AntiAliasing::Algorithm::Taa;
  }
  if (normalized == "smaa") {
    enabled = true;
    return AntiAliasing::Algorithm::Smaa;
  }
  if (normalized == "disabled") {
    enabled = false;
    return {};
  }
  throw std::invalid_argument("--preview-aa requires disabled, taa, or smaa.");
}

int ParsePreviewShadowDebugMode(const std::string& value) {
  auto normalized = value;
  std::transform(normalized.begin(), normalized.end(), normalized.begin(), [](const char character) {
    return static_cast<char>(std::tolower(static_cast<unsigned char>(character)));
  });
  if (normalized == "off" || normalized == "disabled" || normalized == "none") {
    return 0;
  }
  if (normalized == "cascade" || normalized == "cascade-index" || normalized == "split" ||
      normalized == "split-index") {
    return 1;
  }
  if (normalized == "light-uv" || normalized == "uv") {
    return 2;
  }
  if (normalized == "light-depth" || normalized == "depth") {
    return 3;
  }
  if (normalized == "atlas-uv" || normalized == "atlas") {
    return 4;
  }
  if (normalized == "texel-density" || normalized == "density") {
    return 5;
  }
  throw std::invalid_argument("Unknown preview shadow debug mode: " + value);
}

void ParsePreviewDebugMode(const std::string& value, EditorCommandLine& command_line) {
  auto normalized = value;
  std::transform(normalized.begin(), normalized.end(), normalized.begin(), [](const char character) {
    return static_cast<char>(std::tolower(static_cast<unsigned char>(character)));
  });
  if (normalized == "off" || normalized == "disabled" || normalized == "none") {
    command_line.preview_taa_debug_mode.reset();
    command_line.preview_smaa_debug_mode.reset();
    command_line.preview_anti_aliasing_debug_disabled = true;
    return;
  }
  command_line.preview_anti_aliasing_debug_disabled = false;
  if (normalized == "taa-motion" || normalized == "motion") {
    command_line.preview_smaa_debug_mode.reset();
    command_line.preview_taa_debug_mode = AntiAliasing::TaaDebugMode::Motion;
    return;
  }
  if (normalized == "taa-depth-confidence" || normalized == "depth-confidence") {
    command_line.preview_smaa_debug_mode.reset();
    command_line.preview_taa_debug_mode = AntiAliasing::TaaDebugMode::DepthConfidence;
    return;
  }
  if (normalized == "taa-history-confidence" || normalized == "history-confidence") {
    command_line.preview_smaa_debug_mode.reset();
    command_line.preview_taa_debug_mode = AntiAliasing::TaaDebugMode::HistoryConfidence;
    return;
  }
  if (normalized == "taa-no-history" || normalized == "no-history") {
    command_line.preview_smaa_debug_mode.reset();
    command_line.preview_taa_debug_mode = AntiAliasing::TaaDebugMode::NoHistory;
    return;
  }
  if (normalized == "smaa-edges") {
    command_line.preview_taa_debug_mode.reset();
    command_line.preview_smaa_debug_mode = AntiAliasing::SmaaDebugMode::Edges;
    return;
  }
  if (normalized == "smaa-weights") {
    command_line.preview_taa_debug_mode.reset();
    command_line.preview_smaa_debug_mode = AntiAliasing::SmaaDebugMode::BlendWeights;
    return;
  }
  throw std::invalid_argument("Unknown preview debug mode: " + value);
}

void ParsePreviewAntiAliasingPreset(const std::string& value, EditorCommandLine& command_line) {
  auto normalized = value;
  std::transform(normalized.begin(), normalized.end(), normalized.begin(), [](const char character) {
    return static_cast<char>(std::tolower(static_cast<unsigned char>(character)));
  });
  if (normalized == "best" || normalized == "best-quality") {
    command_line.preview_smaa_preset.reset();
    command_line.preview_taa_preset = AntiAliasing::TaaPreset::BestQuality;
    return;
  }
  if (normalized == "high-quality") {
    command_line.preview_smaa_preset.reset();
    command_line.preview_taa_preset = AntiAliasing::TaaPreset::HighQuality;
    return;
  }
  if (normalized == "performance") {
    command_line.preview_smaa_preset.reset();
    command_line.preview_taa_preset = AntiAliasing::TaaPreset::Performance;
    return;
  }
  if (normalized == "low") {
    command_line.preview_taa_preset.reset();
    command_line.preview_smaa_preset = AntiAliasing::SmaaPreset::Low;
    return;
  }
  if (normalized == "medium") {
    command_line.preview_taa_preset.reset();
    command_line.preview_smaa_preset = AntiAliasing::SmaaPreset::Medium;
    return;
  }
  if (normalized == "high") {
    command_line.preview_taa_preset.reset();
    command_line.preview_smaa_preset = AntiAliasing::SmaaPreset::High;
    return;
  }
  if (normalized == "ultra") {
    command_line.preview_taa_preset.reset();
    command_line.preview_smaa_preset = AntiAliasing::SmaaPreset::Ultra;
    return;
  }
  throw std::invalid_argument(
      "--preview-aa-preset requires best-quality, high-quality, performance, low, medium, high, or ultra.");
}

glm::vec3 ParseVec3Argument(const int argc, char** argv, int& arg_index, const std::string& argument) {
  if (arg_index + 1 >= argc) {
    throw std::invalid_argument(argument + " requires x,y,z.");
  }
  std::string value = argv[++arg_index] ? argv[arg_index] : "";
  std::replace(value.begin(), value.end(), ',', ' ');
  std::stringstream stream(value);
  glm::vec3 result{};
  if (!(stream >> result.x >> result.y >> result.z)) {
    throw std::invalid_argument(argument + " requires x,y,z.");
  }
  std::string trailing;
  if (stream >> trailing) {
    throw std::invalid_argument(argument + " requires exactly three numeric values.");
  }
  return result;
}

EditorCommandLine ParseCommandLine(const int argc, char** argv) {
  EditorCommandLine command_line;
  for (int arg_index = 1; arg_index < argc; ++arg_index) {
    const std::string argument = argv[arg_index] ? argv[arg_index] : "";
    if (argument == "--project" || argument == "-p") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument(argument + " requires a project path.");
      }
      command_line.project_path = std::filesystem::absolute(argv[++arg_index]);
    } else if (argument == "--demo") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument("--demo requires a profile id.");
      }
      const std::string profile_id = argv[++arg_index] ? argv[arg_index] : "";
      const auto* profile = FindDemoProfile(profile_id);
      if (!profile) {
        throw std::invalid_argument("Unknown EvoEngineEditor demo profile: " + profile_id);
      }
      command_line.demo_profile_id = profile->id;
    } else if (argument == "--capture-demo-preview") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument("--capture-demo-preview requires an output PNG or HDR path.");
      }
      command_line.demo_preview_capture_path = std::filesystem::absolute(argv[++arg_index]);
    } else if (argument == "--preview-metrics-json") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument("--preview-metrics-json requires an output JSON path.");
      }
      command_line.preview_capture_metrics_path = std::filesystem::absolute(argv[++arg_index]);
    } else if (argument == "--preview-width") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument("--preview-width requires a positive integer.");
      }
      command_line.preview_capture_width = std::max(1, std::stoi(argv[++arg_index]));
    } else if (argument == "--preview-height") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument("--preview-height requires a positive integer.");
      }
      command_line.preview_capture_height = std::max(1, std::stoi(argv[++arg_index]));
    } else if (argument == "--preview-warmup-frames") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument("--preview-warmup-frames requires a non-negative integer.");
      }
      command_line.preview_capture_warmup_frames = static_cast<size_t>(std::max(0, std::stoi(argv[++arg_index])));
    } else if (argument == "--preview-render-mode") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument("--preview-render-mode requires rasterization, raytracing, or rayquery.");
      }
      command_line.preview_capture_render_mode = ParsePreviewRenderMode(argv[++arg_index] ? argv[arg_index] : "");
    } else if (argument == "--disable-ray-tracing-pipeline") {
      command_line.disable_ray_tracing_pipeline = true;
    } else if (argument == "--preview-ser") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument("--preview-ser requires disabled, automatic, or enabled.");
      }
      command_line.preview_capture_ser_mode =
          ParsePreviewShaderExecutionReorderingMode(argv[++arg_index] ? argv[arg_index] : "");
    } else if (argument == "--preview-firefly-clamp") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument("--preview-firefly-clamp requires enabled or disabled.");
      }
      command_line.preview_capture_firefly_clamp_enabled =
          ParsePreviewBool(argv[++arg_index] ? argv[arg_index] : "", argument);
    } else if (argument == "--preview-firefly-clamp-threshold") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument("--preview-firefly-clamp-threshold requires a non-negative number.");
      }
      command_line.preview_capture_firefly_clamp_threshold = std::max(0.0f, std::stof(argv[++arg_index]));
    } else if (argument == "--preview-emissive-nee") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument("--preview-emissive-nee requires enabled or disabled.");
      }
      command_line.preview_capture_emissive_triangle_nee_enabled =
          ParsePreviewBool(argv[++arg_index] ? argv[arg_index] : "", argument);
    } else if (argument == "--preview-auto-spp") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument("--preview-auto-spp requires enabled or disabled.");
      }
      command_line.preview_capture_auto_spp_enabled =
          ParsePreviewBool(argv[++arg_index] ? argv[arg_index] : "", argument);
    } else if (argument == "--preview-auto-spp-min-samples") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument("--preview-auto-spp-min-samples requires a positive integer.");
      }
      command_line.preview_capture_auto_spp_min_samples = std::max(1, std::stoi(argv[++arg_index]));
    } else if (argument == "--preview-auto-spp-max-samples") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument("--preview-auto-spp-max-samples requires a positive integer.");
      }
      command_line.preview_capture_auto_spp_max_samples = std::max(1, std::stoi(argv[++arg_index]));
    } else if (argument == "--preview-auto-spp-threshold") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument("--preview-auto-spp-threshold requires a non-negative number.");
      }
      command_line.preview_capture_auto_spp_convergence_threshold = std::max(0.0f, std::stof(argv[++arg_index]));
    } else if (argument == "--preview-sample-size") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument("--preview-sample-size requires a positive integer.");
      }
      command_line.preview_capture_sample_size = std::max(1, std::stoi(argv[++arg_index]));
    } else if (argument == "--preview-camera-position") {
      command_line.preview_capture_camera_position = ParseVec3Argument(argc, argv, arg_index, argument);
    } else if (argument == "--preview-camera-look-at") {
      command_line.preview_capture_camera_look_at = ParseVec3Argument(argc, argv, arg_index, argument);
    } else if (argument == "--preview-ao") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument("--preview-ao requires ssao, gtao, or disabled.");
      }
      bool preview_ambient_occlusion_enabled = false;
      command_line.preview_ambient_occlusion_algorithm = ParsePreviewAmbientOcclusionAlgorithm(
          argv[++arg_index] ? argv[arg_index] : "", preview_ambient_occlusion_enabled);
      command_line.preview_ambient_occlusion_enabled = preview_ambient_occlusion_enabled;
    } else if (argument == "--preview-aa") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument("--preview-aa requires disabled, taa, or smaa.");
      }
      bool preview_anti_aliasing_enabled = false;
      command_line.preview_anti_aliasing_algorithm =
          ParsePreviewAntiAliasingAlgorithm(argv[++arg_index] ? argv[arg_index] : "", preview_anti_aliasing_enabled);
      command_line.preview_anti_aliasing_enabled = preview_anti_aliasing_enabled;
    } else if (argument == "--preview-aa-preset") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument("--preview-aa-preset requires a preset.");
      }
      ParsePreviewAntiAliasingPreset(argv[++arg_index] ? argv[arg_index] : "", command_line);
    } else if (argument == "--preview-aa-tgsm") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument("--preview-aa-tgsm requires enabled or disabled.");
      }
      command_line.preview_anti_aliasing_tgsm = ParsePreviewBool(argv[++arg_index] ? argv[arg_index] : "", argument);
    } else if (argument == "--preview-aa-fp16") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument("--preview-aa-fp16 requires enabled or disabled.");
      }
      command_line.preview_anti_aliasing_fp16 = ParsePreviewBool(argv[++arg_index] ? argv[arg_index] : "", argument);
    } else if (argument == "--preview-aa-motion-sequence") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument("--preview-aa-motion-sequence requires enabled or disabled.");
      }
      command_line.preview_anti_aliasing_motion_sequence =
          ParsePreviewBool(argv[++arg_index] ? argv[arg_index] : "", argument);
    } else if (argument == "--preview-debug") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument("--preview-debug requires a mode.");
      }
      ParsePreviewDebugMode(argv[++arg_index] ? argv[arg_index] : "", command_line);
    } else if (argument == "--preview-shadow-split-lambda") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument("--preview-shadow-split-lambda requires a value between 0 and 1.");
      }
      command_line.preview_shadow_split_lambda = std::clamp(std::stof(argv[++arg_index]), 0.0f, 1.0f);
    } else if (argument == "--preview-shadow-cascade-transition-width") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument("--preview-shadow-cascade-transition-width requires a non-negative width.");
      }
      command_line.preview_shadow_cascade_transition_width = std::max(0.0f, std::stof(argv[++arg_index]));
    } else if (argument == "--preview-shadow-distance-fade") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument("--preview-shadow-distance-fade requires a non-negative width.");
      }
      command_line.preview_shadow_distance_fade = std::max(0.0f, std::stof(argv[++arg_index]));
    } else if (argument == "--preview-shadow-debug") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument("--preview-shadow-debug requires a mode.");
      }
      command_line.preview_shadow_debug_mode = ParsePreviewShadowDebugMode(argv[++arg_index] ? argv[arg_index] : "");
    } else if (argument == "--preview-shadow-debug-cascade") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument("--preview-shadow-debug-cascade requires a cascade index.");
      }
      command_line.preview_shadow_debug_cascade = std::clamp(std::stoi(argv[++arg_index]), 0, 3);
    } else if (argument == "--preview-shadow-debug-light") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument("--preview-shadow-debug-light requires a directional light index.");
      }
      command_line.preview_shadow_debug_light = std::max(0, std::stoi(argv[++arg_index]));
    } else if (argument == "--shadow-map-resolution" || argument == "--shadow-resolution") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument(argument + " requires low, medium, high, or very-high.");
      }
      command_line.shadow_map_resolution_quality =
          ParseShadowMapResolutionQualityName(argv[++arg_index] ? argv[arg_index] : "");
    } else if (argument == "--preview-deterministic") {
      command_line.preview_capture_deterministic = true;
    } else if (argument == "--preview-bistro-ddgi") {
      command_line.preview_capture_bistro_ddgi = true;
    } else {
      auto application_mode = command_line.application_mode;
      if (!ConsumeApplicationModeArgument(argc, argv, arg_index, application_mode)) {
        if (!command_line.project_path) {
          command_line.project_path = std::filesystem::absolute(argument);
        } else {
          throw std::invalid_argument("Unknown EvoEngineEditor argument: " + argument);
        }
      } else {
        command_line.application_mode = application_mode;
        command_line.application_mode_explicit = true;
      }
      continue;
    }
  }
  if (command_line.demo_profile_id && command_line.project_path) {
    throw std::invalid_argument("EvoEngineEditor --demo cannot be combined with --project.");
  }
  if (command_line.demo_preview_capture_path && !command_line.demo_profile_id) {
    throw std::invalid_argument("--capture-demo-preview requires --demo <profile-id>.");
  }
  if (command_line.preview_capture_metrics_path && !command_line.demo_preview_capture_path) {
    throw std::invalid_argument("--preview-metrics-json requires --capture-demo-preview.");
  }
  if (command_line.demo_preview_capture_path) {
    auto extension = command_line.demo_preview_capture_path->extension().string();
    std::transform(extension.begin(), extension.end(), extension.begin(), [](const char character) {
      return static_cast<char>(std::tolower(static_cast<unsigned char>(character)));
    });
    if (extension != ".png" && extension != ".hdr") {
      throw std::invalid_argument("--capture-demo-preview output must use .png or .hdr.");
    }
  }
  if (command_line.preview_capture_camera_position.has_value() !=
      command_line.preview_capture_camera_look_at.has_value()) {
    throw std::invalid_argument("--preview-camera-position and --preview-camera-look-at must be provided together.");
  }
  if ((command_line.preview_capture_camera_position || command_line.preview_capture_camera_look_at) &&
      !command_line.demo_preview_capture_path) {
    throw std::invalid_argument("--preview-camera-position requires --capture-demo-preview.");
  }
  if ((command_line.preview_ambient_occlusion_enabled || command_line.preview_ambient_occlusion_algorithm ||
       command_line.preview_anti_aliasing_enabled || command_line.preview_anti_aliasing_algorithm ||
       command_line.preview_taa_preset || command_line.preview_smaa_preset || command_line.preview_anti_aliasing_tgsm ||
       command_line.preview_anti_aliasing_fp16 || command_line.preview_anti_aliasing_motion_sequence ||
       command_line.preview_taa_debug_mode || command_line.preview_smaa_debug_mode ||
       command_line.preview_anti_aliasing_debug_disabled) &&
      !command_line.demo_preview_capture_path) {
    throw std::invalid_argument("Preview post-processing overrides require --capture-demo-preview.");
  }
  if (command_line.preview_capture_bistro_ddgi && !command_line.demo_preview_capture_path) {
    throw std::invalid_argument("--preview-bistro-ddgi requires --capture-demo-preview.");
  }
  if ((command_line.preview_shadow_debug_mode || command_line.preview_shadow_debug_cascade ||
       command_line.preview_shadow_debug_light) &&
      !command_line.demo_preview_capture_path) {
    throw std::invalid_argument("--preview-shadow-debug requires --capture-demo-preview.");
  }
  if ((command_line.preview_shadow_split_lambda || command_line.preview_shadow_cascade_transition_width ||
       command_line.preview_shadow_distance_fade) &&
      !command_line.demo_preview_capture_path) {
    throw std::invalid_argument("Preview shadow overrides require --capture-demo-preview.");
  }
  if (command_line.preview_capture_bistro_ddgi && command_line.demo_profile_id != DemoProfileId::Bistro) {
    throw std::invalid_argument("--preview-bistro-ddgi requires --demo bistro.");
  }
  if (command_line.preview_anti_aliasing_motion_sequence &&
      command_line.demo_profile_id != DemoProfileId::RenderingRegression) {
    throw std::invalid_argument("--preview-aa-motion-sequence requires --demo rendering-regression.");
  }
  const auto preview_anti_aliasing_algorithm =
      command_line.preview_anti_aliasing_algorithm.value_or(AntiAliasing::Algorithm::Smaa);
  const bool preview_anti_aliasing_disabled =
      command_line.preview_anti_aliasing_enabled && !*command_line.preview_anti_aliasing_enabled;
  const bool has_technique_specific_anti_aliasing_options =
      command_line.preview_taa_preset || command_line.preview_smaa_preset || command_line.preview_anti_aliasing_tgsm ||
      command_line.preview_anti_aliasing_fp16 || command_line.preview_anti_aliasing_motion_sequence ||
      command_line.preview_taa_debug_mode || command_line.preview_smaa_debug_mode;
  if (preview_anti_aliasing_disabled && has_technique_specific_anti_aliasing_options) {
    throw std::invalid_argument("--preview-aa disabled cannot be combined with AA presets, controls, or debug modes.");
  }
  if (!command_line.preview_anti_aliasing_algorithm && has_technique_specific_anti_aliasing_options) {
    throw std::invalid_argument("AA presets, controls, and debug modes require --preview-aa taa or --preview-aa smaa.");
  }
  if (preview_anti_aliasing_algorithm != AntiAliasing::Algorithm::Taa &&
      (command_line.preview_taa_preset || command_line.preview_anti_aliasing_tgsm ||
       command_line.preview_anti_aliasing_fp16 || command_line.preview_anti_aliasing_motion_sequence ||
       command_line.preview_taa_debug_mode)) {
    throw std::invalid_argument("TAA presets, controls, and debug modes require --preview-aa taa.");
  }
  if (preview_anti_aliasing_algorithm != AntiAliasing::Algorithm::Smaa &&
      (command_line.preview_smaa_preset || command_line.preview_smaa_debug_mode)) {
    throw std::invalid_argument("SMAA presets and debug modes require --preview-aa smaa.");
  }
  if (command_line.demo_profile_id) {
    const auto& profile = GetDemoProfile(*command_line.demo_profile_id);
    if (!command_line.application_mode_explicit) {
      command_line.application_mode = profile.default_application_mode;
    }
    if (command_line.demo_preview_capture_path) {
      if (command_line.application_mode_explicit && command_line.application_mode != ApplicationMode::Editor) {
        throw std::invalid_argument("--capture-demo-preview requires editor mode.");
      }
      command_line.application_mode = ApplicationMode::Editor;
    } else if (!IsDemoProfileApplicationModeSupported(profile.id, command_line.application_mode)) {
      throw std::invalid_argument("EvoEngineEditor --demo " + std::string(profile.id_name) + " does not support " +
                                  GetApplicationModeName(command_line.application_mode) + " mode.");
    }
  }
  return command_line;
}

void ApplyGraphicsCommandLineOverrides(const EditorCommandLine& command_line,
                                       ApplicationInitializationSettings& application_info) {
  if (command_line.shadow_map_resolution_quality) {
    application_info.graphics_settings.SetShadowMapResolutionQuality(*command_line.shadow_map_resolution_quality);
  }
  if (command_line.disable_ray_tracing_pipeline) {
    Platform::GetInstance().GetCapabilities().support_ray_tracing = false;
  }
}

void ApplyPreviewCameraOverride(const std::shared_ptr<EditorLayer>& editor_layer,
                                const std::optional<glm::vec3>& camera_position,
                                const std::optional<glm::vec3>& camera_look_at) {
  if (!camera_position || !camera_look_at) {
    return;
  }
  const auto front = *camera_look_at - *camera_position;
  if (glm::length(front) <= 0.0001f) {
    throw std::invalid_argument("--preview-camera-position and --preview-camera-look-at must be different.");
  }
  const auto normalized_front = glm::normalize(front);
  auto up = glm::vec3(0.0f, 1.0f, 0.0f);
  if (glm::abs(glm::dot(normalized_front, up)) > 0.98f) {
    up = glm::vec3(0.0f, 0.0f, 1.0f);
  }
  editor_layer->SetSceneCameraPosition(*camera_position);
  editor_layer->SetSceneCameraRotation(glm::quatLookAt(normalized_front, up));
  if (const auto scene_camera = editor_layer->GetSceneCamera()) {
    scene_camera->ResetFrameCount();
  }
}

std::filesystem::path CurrentExecutablePath() {
#ifdef EVOENGINE_WINDOWS
  return path_utils::CurrentExecutablePath("EvoEngineEditor.exe");
#else
  return path_utils::CurrentExecutablePath("EvoEngineEditor");
#endif
}

std::filesystem::path LauncherExecutablePath() {
#ifdef EVOENGINE_WINDOWS
  return CurrentExecutablePath().parent_path() / "EvoEngineLauncher.exe";
#else
  return CurrentExecutablePath().parent_path() / "EvoEngineLauncher";
#endif
}

bool LaunchLauncherProcess(std::string& error) {
  const auto launcher_path = LauncherExecutablePath();
  if (!std::filesystem::exists(launcher_path)) {
    error = "Could not find EvoEngineLauncher next to EvoEngineEditor.";
    return false;
  }

#ifdef EVOENGINE_WINDOWS
  std::wstring command_line = L"\"" + launcher_path.wstring() + L"\"";
  STARTUPINFOW startup_info{};
  startup_info.cb = sizeof(startup_info);
  PROCESS_INFORMATION process_info{};
  const auto working_directory = launcher_path.parent_path().wstring();
  if (!CreateProcessW(nullptr, command_line.data(), nullptr, nullptr, FALSE, 0, nullptr, working_directory.c_str(),
                      &startup_info, &process_info)) {
    error = "Failed to launch EvoEngineLauncher.";
    return false;
  }
  CloseHandle(process_info.hProcess);
  CloseHandle(process_info.hThread);
  return true;
#else
  const auto command = "\"" + launcher_path.string() + "\" &";
  if (std::system(command.c_str()) != 0) {
    error = "Failed to launch EvoEngineLauncher.";
    return false;
  }
  return true;
#endif
}

void ConfigurePackageDemoNewSceneDefaults() {
  ProjectManager::SetActionAfterNewScene([](const std::shared_ptr<Scene>& scene) {
    ApplicationContext::Get().GetTimes().SetTimeStep(0.016f);
    Transform transform;
    transform.SetPosition(glm::vec3(0, 2, 35));
    transform.SetEulerRotation(glm::radians(glm::vec3(15, 0, 0)));
    if (const auto main_camera = scene->main_camera.Get<Camera>()) {
      scene->SetDataComponent(main_camera->GetOwner(), transform);
      main_camera->camera_settings.use_clear_color = true;
      main_camera->camera_settings.clear_color = glm::vec4(0.5f, 0.5f, 0.5f, 1.f);
    }
  });
}

void ApplyBistroDemoEditorCameraDefaults() {
  const auto editor_layer = ApplicationContext::Get().GetLayer<EditorLayer>();
  if (!editor_layer) {
    return;
  }
  editor_layer->velocity = 15.0f;
  editor_layer->enable_gizmos = false;
  editor_layer->show_scene_info = true;
  editor_layer->SetSelectedEntity({});
  editor_layer->default_scene_camera_position = glm::vec3(3.0f, 25.0f, 150.0f);
  editor_layer->default_scene_camera_rotation = glm::quat(glm::radians(glm::vec3(-5.0f, 0.0f, 0.0f)));
  editor_layer->SetSceneCameraPosition(editor_layer->default_scene_camera_position);
  editor_layer->SetSceneCameraRotation(editor_layer->default_scene_camera_rotation);
  if (const auto scene_camera = editor_layer->GetSceneCamera()) {
    scene_camera->camera_settings.fov = 60.0f;
    scene_camera->camera_settings.far_distance = 500.0f;
    scene_camera->ResetFrameCount();
  }
}

void ConfigureDemoProfile(const DemoProfileId profile_id, const ApplicationMode application_mode,
                          ApplicationInitializationSettings& application_info) {
  const auto missing_resources = MissingDemoProfileResourceRequirements(profile_id);
  if (!missing_resources.empty()) {
    std::string message = "Demo profile '" + std::string(GetDemoProfileIdName(profile_id)) + "' is missing ";
    for (size_t i = 0; i < missing_resources.size(); ++i) {
      if (i > 0) {
        message += ", ";
      }
      message += missing_resources[i];
    }
    throw std::runtime_error(message + ".");
  }

  application_info.application_mode = application_mode;
  application_info.use_custom_title_bar = true;
  const auto resource_root = FindDemoProfileResourcesRoot();
  switch (profile_id) {
    case DemoProfileId::Rendering:
      SetupDemoScene(DemoSetup::Rendering, application_info, resource_root);
      break;
    case DemoProfileId::RenderingRegression:
      SetupDemoScene(DemoSetup::RenderingRegression, application_info, resource_root, false);
      break;
    case DemoProfileId::Ddgi:
      SetupDemoScene(DemoSetup::CornellBox, application_info, resource_root);
      ConfigureDdgiCornellBoxApplication(application_info, application_mode);
      break;
    case DemoProfileId::ProceduralGalaxy:
      SetupDemoScene(DemoSetup::ProceduralGalaxy, application_info, resource_root);
      break;
    case DemoProfileId::GaussianSplat:
      SetupDemoScene(DemoSetup::GaussianSplat, application_info, resource_root, false);
      break;
    case DemoProfileId::Bicycle:
      SetupDemoScene(DemoSetup::Bicycle, application_info, resource_root, false);
      break;
    case DemoProfileId::Bistro:
      SetupDemoScene(DemoSetup::Bistro, application_info, resource_root, false);
      break;
    case DemoProfileId::LSystem: {
      const auto& profile = GetDemoProfile(profile_id);
      application_info.application_name = profile.title;
      application_info.project_path = ResolveDemoProfileProjectPath(profile_id, resource_root);
      application_info.enable_runtime_packages = true;
      application_info.startup_runtime_packages = profile.startup_runtime_packages;
      break;
    }
    case DemoProfileId::EcoSysLab:
    case DemoProfileId::DigitalAgriculture: {
      NormalizeLegacyResourceExtensions(resource_root);
      ConfigurePackageDemoNewSceneDefaults();
      const auto& profile = GetDemoProfile(profile_id);
      application_info.application_name = profile.title;
      application_info.project_path = ResolveDemoProfileProjectPath(profile_id, resource_root);
      application_info.enable_runtime_packages = true;
      application_info.startup_runtime_packages = profile.startup_runtime_packages;
      break;
    }
  }
}

void ApplyDemoEditorDefaults(const DemoProfileId profile_id) {
  const auto editor_layer = ApplicationContext::Get().GetLayer<EditorLayer>();
  if (!editor_layer) {
    return;
  }
  switch (profile_id) {
    case DemoProfileId::EcoSysLab: {
      editor_layer->velocity = 2.f;
      const auto scene_camera = editor_layer->GetSceneCamera();
      if (!scene_camera) {
        return;
      }
      auto& camera_settings = scene_camera->camera_settings;
      camera_settings.use_clear_color = true;
      camera_settings.clear_color = glm::vec4(1.f);
      camera_settings.background_intensity = 3.f;
      const auto post_processing_stack = scene_camera->post_processing_stack_ref.Get<PostProcessingStack>();
      if (post_processing_stack) {
        post_processing_stack->enable_bloom = false;
      }
      break;
    }
    case DemoProfileId::DigitalAgriculture:
      editor_layer->velocity = 2.f;
      editor_layer->default_scene_camera_position = glm::vec3(1.124f, 0.218f, 14.089f);
      editor_layer->SetSceneCameraPosition(editor_layer->default_scene_camera_position);
      break;
    case DemoProfileId::LSystem:
      editor_layer->velocity = 2.f;
      editor_layer->default_scene_camera_position = glm::vec3(0.0f, 1.0f, 5.0f);
      editor_layer->SetSceneCameraPosition(editor_layer->default_scene_camera_position);
      break;
    case DemoProfileId::ProceduralGalaxy:
      editor_layer->velocity = 50.f;
      editor_layer->default_scene_camera_position = glm::vec3(0.0f, 100.0f, 100.0f);
      editor_layer->SetSceneCameraPosition(editor_layer->default_scene_camera_position);
      editor_layer->SetSceneCameraRotation(glm::quat(glm::radians(glm::vec3(-50.0f, 0.0f, 0.0f))));
      break;
    case DemoProfileId::GaussianSplat:
    case DemoProfileId::Bicycle:
      editor_layer->velocity = 1.0f;
      editor_layer->default_scene_camera_position = glm::vec3(0.0f, 0.0f, 3.0f);
      editor_layer->SetSceneCameraPosition(editor_layer->default_scene_camera_position);
      break;
    case DemoProfileId::Bistro:
      ApplyBistroDemoEditorCameraDefaults();
      break;
    case DemoProfileId::RenderingRegression:
    case DemoProfileId::Rendering:
    case DemoProfileId::Ddgi:
      break;
  }
}

void WaitForDemoProfileProjectIdle() {
  constexpr size_t max_load_frames = 30000;
  size_t load_frame_count = 0;
  while (!ProjectManager::IsProjectIdle()) {
    if (!ApplicationContext::Get().Loop()) {
      throw std::runtime_error("Application ended before demo profile project load completed.");
    }
    ++load_frame_count;
    if (load_frame_count >= max_load_frames) {
      throw std::runtime_error("Demo profile project load timed out.");
    }
  }
}

bool DemoPreviewSceneInputsReady() {
  return ProjectManager::IsProjectIdle() && !AssetManager::GetAssetLoadSnapshot().Active() &&
         !TextureStorage::HasPendingUploads();
}

void WaitForDemoPreviewSceneInputsReady() {
  constexpr size_t max_load_frames = 30000;
  constexpr size_t settled_frame_count = 4;
  size_t load_frame_count = 0;
  size_t stable_frame_count = 0;
  while (stable_frame_count < settled_frame_count) {
    if (DemoPreviewSceneInputsReady()) {
      ++stable_frame_count;
    } else {
      stable_frame_count = 0;
    }
    if (stable_frame_count >= settled_frame_count) {
      break;
    }
    if (!ApplicationContext::Get().Loop()) {
      throw std::runtime_error("Application ended before demo preview scene inputs were ready.");
    }
    ++load_frame_count;
    if (load_frame_count >= max_load_frames) {
      throw std::runtime_error("Demo preview scene input readiness timed out.");
    }
  }
}

void ApplyDemoProfilePostLoadSetup(const DemoProfileId profile_id, const ApplicationMode application_mode) {
  switch (profile_id) {
    case DemoProfileId::Rendering:
      if (application_mode == ApplicationMode::Editor) {
        ApplyRenderingDemoEditorSetup();
      }
      break;
    case DemoProfileId::RenderingRegression:
      ConfigureRenderingRegressionDemoScene(ApplicationContext::Get().GetActiveScene());
      break;
    case DemoProfileId::Ddgi:
      ConfigureDdgiCornellBoxScene(ApplicationContext::Get().GetActiveScene());
      if (const auto editor_layer = ApplicationContext::Get().GetLayer<EditorLayer>()) {
        if (const auto scene_camera = editor_layer->GetSceneCamera()) {
          scene_camera->skybox.Clear();
          scene_camera->camera_settings.use_clear_color = true;
          scene_camera->camera_settings.clear_color = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
          scene_camera->camera_settings.background_intensity = 0.0f;
          scene_camera->ResetFrameCount();
        }
      }
      if (application_mode == ApplicationMode::Player) {
        ApplicationContext::Get().Play();
      }
      break;
    case DemoProfileId::EcoSysLab:
    case DemoProfileId::DigitalAgriculture:
    case DemoProfileId::LSystem:
    case DemoProfileId::ProceduralGalaxy:
      break;
    case DemoProfileId::GaussianSplat:
      ConfigureGaussianSplatDemoScene(ApplicationContext::Get().GetActiveScene());
      break;
    case DemoProfileId::Bicycle:
      ConfigureBicycleDemoScene(ApplicationContext::Get().GetActiveScene());
      break;
    case DemoProfileId::Bistro:
      ConfigureBistroDemoScene(ApplicationContext::Get().GetActiveScene());
      break;
  }
}

void CaptureDemoPreview(
    const std::filesystem::path& output_path, const std::optional<std::filesystem::path>& metrics_path, const int width,
    const int height, const size_t warmup_frames, const std::optional<DemoProfileId> demo_profile_id,
    const std::optional<Camera::CameraRenderMode>& preview_render_mode,
    const std::optional<CameraSettings::ShaderExecutionReorderingMode>& preview_ser_mode,
    const std::optional<bool>& preview_firefly_clamp_enabled,
    const std::optional<float>& preview_firefly_clamp_threshold,
    const std::optional<bool>& preview_emissive_triangle_nee_enabled,
    const std::optional<bool>& preview_auto_spp_enabled, const std::optional<int>& preview_auto_spp_min_samples,
    const std::optional<int>& preview_auto_spp_max_samples,
    const std::optional<float>& preview_auto_spp_convergence_threshold, const std::optional<int> preview_sample_size,
    const std::optional<glm::vec3>& preview_camera_position, const std::optional<glm::vec3>& preview_camera_look_at,
    const std::optional<bool>& preview_ambient_occlusion_enabled,
    const std::optional<AmbientOcclusion::Algorithm>& preview_ambient_occlusion_algorithm,
    const std::optional<bool>& preview_anti_aliasing_enabled,
    const std::optional<AntiAliasing::Algorithm>& preview_anti_aliasing_algorithm,
    const std::optional<AntiAliasing::TaaPreset>& preview_taa_preset,
    const std::optional<AntiAliasing::SmaaPreset>& preview_smaa_preset,
    const std::optional<bool>& preview_anti_aliasing_tgsm, const std::optional<bool>& preview_anti_aliasing_fp16,
    const std::optional<bool>& preview_anti_aliasing_motion_sequence,
    const std::optional<AntiAliasing::TaaDebugMode>& preview_taa_debug_mode,
    const std::optional<AntiAliasing::SmaaDebugMode>& preview_smaa_debug_mode,
    const bool preview_anti_aliasing_debug_disabled, const std::optional<float>& preview_shadow_split_lambda,
    const std::optional<float>& preview_shadow_cascade_transition_width,
    const std::optional<float>& preview_shadow_distance_fade, const std::optional<int>& preview_shadow_debug_mode,
    const std::optional<int>& preview_shadow_debug_cascade, const std::optional<int>& preview_shadow_debug_light,
    const bool deterministic_capture, const bool preview_bistro_ddgi) {
  auto output_extension = output_path.extension().string();
  std::transform(output_extension.begin(), output_extension.end(), output_extension.begin(), [](const char character) {
    return static_cast<char>(std::tolower(static_cast<unsigned char>(character)));
  });
  const bool linear_hdr_output = output_extension == ".hdr";
  const glm::uvec2 preview_resolution(static_cast<uint32_t>(width), static_cast<uint32_t>(height));
  if (const auto window_layer = ApplicationContext::Get().GetLayer<WindowLayer>()) {
    window_layer->ResizeWindow(width, height);
    window_layer->CenterWindow();
  }
  const auto editor_layer = ApplicationContext::Get().GetLayer<EditorLayer>();
  if (!editor_layer) {
    throw std::runtime_error("Demo preview capture requires EditorLayer.");
  }
  editor_layer->show_camera_window = false;
  editor_layer->RequestSceneCameraPreviewWindow(preview_resolution);
  editor_layer->SetSceneCameraResolutionOverride(preview_resolution);
  const auto scene_camera = editor_layer->GetSceneCamera();
  if (!scene_camera) {
    throw std::runtime_error("Demo preview capture requires a scene camera.");
  }
  ApplyPreviewCameraOverride(editor_layer, preview_camera_position, preview_camera_look_at);
  if (preview_render_mode) {
    scene_camera->camera_render_mode = *preview_render_mode;
    scene_camera->ResetFrameCount();
  }
  if (preview_ser_mode) {
    scene_camera->camera_settings.shader_execution_reordering_mode = *preview_ser_mode;
    scene_camera->ResetFrameCount();
  }
  if (preview_firefly_clamp_enabled) {
    scene_camera->camera_settings.firefly_clamp_enabled = *preview_firefly_clamp_enabled;
    scene_camera->ResetFrameCount();
  }
  if (preview_firefly_clamp_threshold) {
    scene_camera->camera_settings.firefly_clamp_threshold = *preview_firefly_clamp_threshold;
    scene_camera->ResetFrameCount();
  }
  if (preview_emissive_triangle_nee_enabled) {
    scene_camera->camera_settings.emissive_triangle_nee_enabled = *preview_emissive_triangle_nee_enabled;
    scene_camera->ResetFrameCount();
  }
  if (preview_auto_spp_enabled) {
    scene_camera->camera_settings.auto_spp_enabled = *preview_auto_spp_enabled;
    scene_camera->ResetFrameCount();
  }
  if (preview_auto_spp_min_samples) {
    scene_camera->camera_settings.auto_spp_min_samples = std::max(1, *preview_auto_spp_min_samples);
    scene_camera->camera_settings.auto_spp_max_samples = std::max(scene_camera->camera_settings.auto_spp_max_samples,
                                                                  scene_camera->camera_settings.auto_spp_min_samples);
    scene_camera->ResetFrameCount();
  }
  if (preview_auto_spp_max_samples) {
    scene_camera->camera_settings.auto_spp_max_samples =
        std::max(scene_camera->camera_settings.auto_spp_min_samples, *preview_auto_spp_max_samples);
    scene_camera->ResetFrameCount();
  }
  if (preview_auto_spp_convergence_threshold) {
    scene_camera->camera_settings.auto_spp_convergence_threshold =
        std::max(0.0f, *preview_auto_spp_convergence_threshold);
    scene_camera->ResetFrameCount();
  }
  if (deterministic_capture || preview_sample_size) {
    scene_camera->camera_settings.sample_size =
        deterministic_capture ? preview_sample_size.value_or(1) : preview_sample_size.value();
    scene_camera->ResetFrameCount();
  }
  if (deterministic_capture) {
    if (const auto post_processing_stack = scene_camera->post_processing_stack_ref.Get<PostProcessingStack>()) {
      post_processing_stack->enable_bloom = false;
      post_processing_stack->enable_ambient_occlusion = false;
      post_processing_stack->enable_screen_space_reflection = false;
      post_processing_stack->enable_anti_aliasing = false;
      if (post_processing_stack->tone_mapping) {
        post_processing_stack->tone_mapping->auto_exposure = false;
        post_processing_stack->tone_mapping->dither = false;
      }
    }
  }
  if (preview_ambient_occlusion_enabled || preview_ambient_occlusion_algorithm || preview_anti_aliasing_enabled ||
      preview_anti_aliasing_algorithm || preview_taa_preset || preview_smaa_preset || preview_anti_aliasing_tgsm ||
      preview_anti_aliasing_fp16 || preview_taa_debug_mode || preview_smaa_debug_mode ||
      preview_anti_aliasing_debug_disabled) {
    if (const auto post_processing_stack = scene_camera->post_processing_stack_ref.Get<PostProcessingStack>()) {
      if (preview_ambient_occlusion_enabled) {
        post_processing_stack->enable_ambient_occlusion = *preview_ambient_occlusion_enabled;
      }
      if (preview_ambient_occlusion_algorithm) {
        post_processing_stack->enable_ambient_occlusion = true;
        if (post_processing_stack->ambient_occlusion) {
          post_processing_stack->ambient_occlusion->algorithm = *preview_ambient_occlusion_algorithm;
        }
      }
      if (preview_anti_aliasing_enabled) {
        post_processing_stack->enable_anti_aliasing = *preview_anti_aliasing_enabled;
      }
      if (post_processing_stack->anti_aliasing) {
        const auto& anti_aliasing = post_processing_stack->anti_aliasing;
        if (preview_anti_aliasing_algorithm) {
          anti_aliasing->algorithm = *preview_anti_aliasing_algorithm;
          post_processing_stack->enable_anti_aliasing = true;
        }
        if (preview_taa_preset) {
          post_processing_stack->enable_anti_aliasing = true;
          anti_aliasing->ApplyTaaPreset(*preview_taa_preset);
        }
        if (preview_smaa_preset) {
          post_processing_stack->enable_anti_aliasing = true;
          anti_aliasing->smaa.preset = *preview_smaa_preset;
        }
        if (preview_anti_aliasing_tgsm) {
          post_processing_stack->enable_anti_aliasing = true;
          anti_aliasing->taa.use_tgsm = *preview_anti_aliasing_tgsm;
          anti_aliasing->taa.preset = AntiAliasing::TaaPreset::Custom;
        }
        if (preview_anti_aliasing_fp16) {
          post_processing_stack->enable_anti_aliasing = true;
          anti_aliasing->taa.use_fp16 = *preview_anti_aliasing_fp16;
          anti_aliasing->taa.preset = AntiAliasing::TaaPreset::Custom;
        }
        if (preview_anti_aliasing_debug_disabled) {
          anti_aliasing->taa.debug_mode = AntiAliasing::TaaDebugMode::None;
          anti_aliasing->smaa.debug_mode = AntiAliasing::SmaaDebugMode::None;
        }
        if (preview_taa_debug_mode) {
          post_processing_stack->enable_anti_aliasing = true;
          anti_aliasing->taa.debug_mode = *preview_taa_debug_mode;
        }
        if (preview_smaa_debug_mode) {
          post_processing_stack->enable_anti_aliasing = true;
          anti_aliasing->smaa.debug_mode = *preview_smaa_debug_mode;
        }
        anti_aliasing->NormalizeSettings();
        anti_aliasing->ResetHistory(scene_camera);
      }
      scene_camera->ResetFrameCount();
    }
  }
  if (preview_shadow_split_lambda || preview_shadow_cascade_transition_width || preview_shadow_distance_fade ||
      preview_shadow_debug_mode || preview_shadow_debug_cascade || preview_shadow_debug_light) {
    const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>();
    if (!render_layer) {
      throw std::runtime_error("Preview shadow overrides require RenderLayer.");
    }
    if (preview_shadow_split_lambda) {
      render_layer->render_settings.shadow_cascade_split_lambda = std::clamp(*preview_shadow_split_lambda, 0.0f, 1.0f);
    }
    if (preview_shadow_cascade_transition_width) {
      render_layer->render_settings.shadow_cascade_transition_width =
          std::max(0.0f, *preview_shadow_cascade_transition_width);
    }
    if (preview_shadow_distance_fade) {
      render_layer->render_settings.shadow_distance_fade = std::max(0.0f, *preview_shadow_distance_fade);
    }
    if (preview_shadow_debug_mode) {
      render_layer->render_settings.shadow_debug_mode = std::clamp(*preview_shadow_debug_mode, 0, 5);
    }
    if (preview_shadow_debug_cascade) {
      render_layer->render_settings.shadow_debug_selected_cascade = std::clamp(*preview_shadow_debug_cascade, 0, 3);
    }
    if (preview_shadow_debug_light) {
      render_layer->render_settings.shadow_debug_selected_light = std::max(0, *preview_shadow_debug_light);
    }
  }
  const auto resolved_render_mode = Camera::ResolveCameraRenderMode(scene_camera->camera_render_mode);
  if (linear_hdr_output && !Camera::IsRayCameraRenderMode(resolved_render_mode)) {
    throw std::invalid_argument("Linear HDR preview capture requires raytracing or rayquery mode.");
  }
  if (demo_profile_id == DemoProfileId::Bistro && Camera::IsRayCameraRenderMode(resolved_render_mode)) {
    ConfigureBistroRayTracingPostProcessing(scene_camera);
    if (deterministic_capture) {
      if (const auto post_processing_stack = scene_camera->post_processing_stack_ref.Get<PostProcessingStack>();
          post_processing_stack && post_processing_stack->tone_mapping) {
        post_processing_stack->tone_mapping->auto_exposure = true;
        post_processing_stack->tone_mapping->auto_exposure_delta_time_override = 1.0f / 60.0f;
        post_processing_stack->tone_mapping->dither = false;
      }
    }
  }
  if (demo_profile_id == DemoProfileId::Bistro && !preview_bistro_ddgi) {
    const auto active_scene = ApplicationContext::Get().GetActiveScene();
    ConfigureBistroParityCapture(active_scene, scene_camera);
  }
  if (linear_hdr_output) {
    if (const auto post_processing_stack = scene_camera->post_processing_stack_ref.Get<PostProcessingStack>()) {
      post_processing_stack->enable_ambient_occlusion = false;
      post_processing_stack->enable_bloom = false;
      post_processing_stack->enable_screen_space_reflection = false;
      post_processing_stack->enable_anti_aliasing = false;
      post_processing_stack->enable_tone_mapping = false;
      scene_camera->ResetFrameCount();
    }
  }
  if (demo_profile_id == DemoProfileId::Bistro && !preview_bistro_ddgi) {
    const auto active_scene = ApplicationContext::Get().GetActiveScene();
    LogBistroParityCaptureState(active_scene, scene_camera, width, height,
                                Camera::GetCameraRenderModeName(resolved_render_mode), output_path);
  }
  if (demo_profile_id == DemoProfileId::Bistro && deterministic_capture &&
      Camera::IsRayCameraRenderMode(resolved_render_mode)) {
    if (const auto post_processing_stack = scene_camera->post_processing_stack_ref.Get<PostProcessingStack>();
        post_processing_stack && post_processing_stack->tone_mapping) {
      post_processing_stack->tone_mapping->auto_exposure = true;
      post_processing_stack->tone_mapping->auto_exposure_delta_time_override = 1.0f / 60.0f;
      post_processing_stack->tone_mapping->dither = false;
    }
  }
  scene_camera->Resize(preview_resolution);
  WaitForDemoPreviewSceneInputsReady();
  if (const auto active_scene = ApplicationContext::Get().GetActiveScene()) {
    if (const auto main_camera = active_scene->main_camera.Get<Camera>(); main_camera && main_camera != scene_camera) {
      main_camera->SetRequireRendering(false);
    }
  }
  if (demo_profile_id == DemoProfileId::RenderingRegression && preview_anti_aliasing_motion_sequence) {
    SetRenderingRegressionTemporalMotionEnabled(*preview_anti_aliasing_motion_sequence);
  }
  scene_camera->ResetFrameCount();
  const auto startup_gpu_timestamp_stats = Platform::GetGpuTimestampStats();
  Platform::SetGpuTimestampCaptureEnabled(true);
  const auto capture_start_time = std::chrono::steady_clock::now();
  const bool temporal_motion_capture =
      demo_profile_id == DemoProfileId::RenderingRegression && preview_anti_aliasing_motion_sequence.value_or(false);
  const bool wait_for_ray_accumulation =
      Camera::IsRayCameraRenderMode(resolved_render_mode) && !temporal_motion_capture;
  constexpr size_t max_capture_frame_slack = 30000;
  const size_t max_capture_frames = warmup_frames + max_capture_frame_slack;
  size_t capture_frame_count = 0;
  while ((wait_for_ray_accumulation && scene_camera->GetFrameCount() < warmup_frames) ||
         (!wait_for_ray_accumulation && capture_frame_count < warmup_frames)) {
    if (!ApplicationContext::Get().Loop()) {
      throw std::runtime_error("Application ended before demo preview capture completed.");
    }
    ++capture_frame_count;
    if (capture_frame_count >= max_capture_frames) {
      throw std::runtime_error(wait_for_ray_accumulation
                                   ? "Demo preview capture timed out before accumulating requested ray-tracing frames."
                                   : "Demo preview capture timed out.");
    }
  }
  const auto capture_elapsed_seconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - capture_start_time).count();
  const auto capture_frames_per_second =
      capture_elapsed_seconds > 0.0 ? static_cast<double>(capture_frame_count) / capture_elapsed_seconds : 0.0;
  std::cout << "Demo preview capture timing: output=\"" << output_path.string()
            << "\" render_mode=" << Camera::GetCameraRenderModeName(resolved_render_mode) << " ser_mode="
            << Camera::GetShaderExecutionReorderingModeName(
                   scene_camera->camera_settings.shader_execution_reordering_mode)
            << " warmup_frames=" << warmup_frames << " rendered_frames=" << capture_frame_count
            << " camera_frames=" << scene_camera->GetFrameCount() << " elapsed_seconds=" << capture_elapsed_seconds
            << " frames_per_second=" << capture_frames_per_second << std::endl;
  const auto render_texture = scene_camera->GetRenderTexture();
  if (!render_texture) {
    throw std::runtime_error("Demo preview capture scene camera has no render texture.");
  }
  const auto render_extent = render_texture->GetExtent();
  if (render_extent.width != preview_resolution.x || render_extent.height != preview_resolution.y) {
    throw std::runtime_error("Demo preview capture render texture resolution changed before saving.");
  }
  if (const auto parent_path = output_path.parent_path(); !parent_path.empty()) {
    std::filesystem::create_directories(parent_path);
  }
  if (!render_texture->Save(output_path)) {
    throw std::runtime_error("Demo preview capture failed to save output image.");
  }

  nlohmann::ordered_json metrics;
  metrics["schema"] = 1;
  metrics["type"] = "evoengine_ray_capture";
  metrics["renderer"] = "EvoEngine";
  metrics["demo_profile"] = demo_profile_id ? GetDemoProfileIdName(*demo_profile_id) : "";
  metrics["render_mode"] = Camera::GetCameraRenderModeName(resolved_render_mode);
  metrics["output_path"] = output_path.string();
  metrics["output_format"] = linear_hdr_output ? "radiance_hdr_linear" : "png_display";
  metrics["width"] = width;
  metrics["height"] = height;
  metrics["requested_frames"] = warmup_frames;
  metrics["rendered_frames"] = capture_frame_count;
  metrics["camera_frames"] = scene_camera->GetFrameCount();
  metrics["samples_per_frame"] = scene_camera->camera_settings.sample_size;
  const uint64_t effective_spp =
      static_cast<uint64_t>(scene_camera->GetFrameCount()) * scene_camera->camera_settings.sample_size;
  metrics["effective_spp"] = effective_spp;
  metrics["bounce_depth"] = scene_camera->camera_settings.bounce;
  metrics["firefly_clamp_enabled"] = scene_camera->camera_settings.firefly_clamp_enabled;
  metrics["firefly_clamp_threshold"] = scene_camera->camera_settings.firefly_clamp_threshold;
  metrics["emissive_triangle_nee_enabled"] = scene_camera->camera_settings.emissive_triangle_nee_enabled;
  metrics["auto_spp_enabled"] = scene_camera->camera_settings.auto_spp_enabled;
  metrics["temporal_motion_capture"] = temporal_motion_capture;
  metrics["camera_position_override"] = nullptr;
  metrics["camera_look_at_override"] = nullptr;
  if (preview_camera_position && preview_camera_look_at) {
    metrics["camera_position_override"] = {preview_camera_position->x, preview_camera_position->y,
                                           preview_camera_position->z};
    metrics["camera_look_at_override"] = {preview_camera_look_at->x, preview_camera_look_at->y,
                                          preview_camera_look_at->z};
  }
  metrics["ser_mode_requested"] =
      Camera::GetShaderExecutionReorderingModeName(scene_camera->camera_settings.shader_execution_reordering_mode);
  metrics["ser_supported"] = Platform::GetInstance().GetCapabilities().support_shader_execution_reordering;
  metrics["ser_enabled"] =
      resolved_render_mode == Camera::CameraRenderMode::RayTracing &&
      Camera::ResolveShaderExecutionReorderingEnabled(scene_camera->camera_settings.shader_execution_reordering_mode);
  const auto& capabilities = Platform::GetInstance().GetCapabilities();
  metrics["capabilities"] = {{"acceleration_structure", capabilities.support_acceleration_structure},
                             {"ray_tracing_pipeline", capabilities.support_ray_tracing},
                             {"ray_query", capabilities.support_ray_query},
                             {"shader_execution_reordering", capabilities.support_shader_execution_reordering}};
  metrics["deterministic"] = deterministic_capture;
  metrics["accumulation_wall_seconds"] = capture_elapsed_seconds;
  metrics["frames_per_second"] = capture_frames_per_second;
  const auto effective_samples =
      static_cast<double>(width) * static_cast<double>(height) * static_cast<double>(effective_spp);
  metrics["wall_throughput_msamples_per_second"] =
      capture_elapsed_seconds > 0.0 ? effective_samples / capture_elapsed_seconds / 1.0e6 : 0.0;
  metrics["gpu_timestamps_available"] = Platform::GpuTimestampCaptureAvailable();
  const auto& physical_device = Platform::GetSelectedPhysicalDevice()->properties;
  metrics["gpu"] = {{"name", physical_device.deviceName},
                    {"vendor_id", physical_device.vendorID},
                    {"device_id", physical_device.deviceID},
                    {"driver_version", physical_device.driverVersion},
                    {"api_version", physical_device.apiVersion}};
  metrics["startup_gpu_sections"] = nlohmann::ordered_json::array();
  for (const auto& stats : startup_gpu_timestamp_stats) {
    metrics["startup_gpu_sections"].push_back({{"name", stats.name},
                                               {"last_ms", stats.last_milliseconds},
                                               {"average_ms", stats.AverageMilliseconds()},
                                               {"minimum_ms", stats.minimum_milliseconds},
                                               {"maximum_ms", stats.maximum_milliseconds},
                                               {"sample_count", stats.sample_count}});
  }
  metrics["gpu_sections"] = nlohmann::ordered_json::array();
  for (const auto& stats : Platform::GetGpuTimestampStats()) {
    metrics["gpu_sections"].push_back({{"name", stats.name},
                                       {"last_ms", stats.last_milliseconds},
                                       {"average_ms", stats.AverageMilliseconds()},
                                       {"minimum_ms", stats.minimum_milliseconds},
                                       {"maximum_ms", stats.maximum_milliseconds},
                                       {"sample_count", stats.sample_count}});
  }
  std::cout << "RAY_CAPTURE_JSON " << metrics.dump() << std::endl;
  if (metrics_path) {
    if (const auto parent_path = metrics_path->parent_path(); !parent_path.empty()) {
      std::filesystem::create_directories(parent_path);
    }
    std::ofstream metrics_file(*metrics_path);
    if (!metrics_file) {
      throw std::runtime_error("Demo preview capture failed to open metrics JSON output.");
    }
    metrics_file << metrics.dump(2) << '\n';
  }
  Platform::SetGpuTimestampCaptureEnabled(false);
  editor_layer->SetSceneCameraResolutionOverride(std::nullopt);
}
}  // namespace

int main(const int argc, char** argv) {
  Application application;
  bool initialized = false;
  bool automated_capture = false;
  try {
    const auto command_line = ParseCommandLine(argc, argv);
    automated_capture = command_line.demo_preview_capture_path.has_value();
    const auto& project_path = command_line.project_path;
    if (!project_path) {
      if (command_line.demo_profile_id) {
        PushStandardApplicationLayers(command_line.application_mode);

        ApplicationInitializationSettings application_info{};
        ConfigureDemoProfile(*command_line.demo_profile_id, command_line.application_mode, application_info);
        ApplyApplicationModeDefaults(application_info);
        ApplyGraphicsCommandLineOverrides(command_line, application_info);
        ApplicationContext::Get().Initialize(application_info);
        initialized = true;
        if (command_line.demo_preview_capture_path) {
          Platform::SetGpuTimestampCaptureEnabled(true);
        }
        if (command_line.application_mode == ApplicationMode::Editor) {
          ApplyDemoEditorDefaults(*command_line.demo_profile_id);
        }

        ApplicationContext::Get().Start(false);
        WaitForDemoProfileProjectIdle();
        ApplyDemoProfilePostLoadSetup(*command_line.demo_profile_id, command_line.application_mode);
        if (command_line.demo_preview_capture_path) {
          CaptureDemoPreview(
              *command_line.demo_preview_capture_path, command_line.preview_capture_metrics_path,
              command_line.preview_capture_width, command_line.preview_capture_height,
              command_line.preview_capture_warmup_frames, command_line.demo_profile_id,
              command_line.preview_capture_render_mode, command_line.preview_capture_ser_mode,
              command_line.preview_capture_firefly_clamp_enabled, command_line.preview_capture_firefly_clamp_threshold,
              command_line.preview_capture_emissive_triangle_nee_enabled, command_line.preview_capture_auto_spp_enabled,
              command_line.preview_capture_auto_spp_min_samples, command_line.preview_capture_auto_spp_max_samples,
              command_line.preview_capture_auto_spp_convergence_threshold, command_line.preview_capture_sample_size,
              command_line.preview_capture_camera_position, command_line.preview_capture_camera_look_at,
              command_line.preview_ambient_occlusion_enabled, command_line.preview_ambient_occlusion_algorithm,
              command_line.preview_anti_aliasing_enabled, command_line.preview_anti_aliasing_algorithm,
              command_line.preview_taa_preset, command_line.preview_smaa_preset,
              command_line.preview_anti_aliasing_tgsm, command_line.preview_anti_aliasing_fp16,
              command_line.preview_anti_aliasing_motion_sequence, command_line.preview_taa_debug_mode,
              command_line.preview_smaa_debug_mode, command_line.preview_anti_aliasing_debug_disabled,
              command_line.preview_shadow_split_lambda, command_line.preview_shadow_cascade_transition_width,
              command_line.preview_shadow_distance_fade, command_line.preview_shadow_debug_mode,
              command_line.preview_shadow_debug_cascade, command_line.preview_shadow_debug_light,
              command_line.preview_capture_deterministic, command_line.preview_capture_bistro_ddgi);
          ApplicationContext::Get().Terminate();
          std::cout.flush();
          std::cerr.flush();
          std::_Exit(0);
        }
        ApplicationContext::Get().Run();
        ApplicationContext::Get().Terminate();
        return 0;
      }
      std::string error;
      if (!LaunchLauncherProcess(error)) {
        EVOENGINE_ERROR(error)
        return 1;
      }
      return 0;
    }
    if (project_path->extension() != ".eveproj") {
      EVOENGINE_ERROR("EvoEngineEditor requires --project <path-to-.eveproj>.")
      return 1;
    }

    PushStandardApplicationLayers(command_line.application_mode);

    ApplicationInitializationSettings application_info{};
    application_info.application_mode = command_line.application_mode;
    const auto launch_metadata = ProjectManager::LoadProjectLaunchMetadata(*project_path);
    application_info.application_name = launch_metadata.application_name;
    application_info.project_path = *project_path;
    application_info.use_custom_title_bar = true;
    application_info.startup_runtime_packages = launch_metadata.startup_runtime_packages;
    application_info.enable_runtime_packages = !application_info.startup_runtime_packages.empty();
    ApplyApplicationModeDefaults(application_info);
    ApplyGraphicsCommandLineOverrides(command_line, application_info);
    ApplicationContext::Get().Initialize(application_info);
    initialized = true;

    ApplicationContext::Get().Start();
    ApplicationContext::Get().Run();
    ApplicationContext::Get().Terminate();
    return 0;
  } catch (const std::exception& error) {
    EVOENGINE_ERROR(error.what())
    if (initialized) {
      ApplicationContext::Get().Terminate();
    }
    if (automated_capture) {
      std::cout.flush();
      std::cerr.flush();
      std::_Exit(1);
    }
    return 1;
  }
}
