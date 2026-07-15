#include "AppBootstrap.hpp"
#include "Application.hpp"
#include "AssetManager.hpp"
#include "Camera.hpp"
#include "DemoProfiles.hpp"
#include "DemoScene.hpp"
#include "EditorLayer.hpp"
#include "GeometryStorage.hpp"
#include "GraphicsPipeline.hpp"
#include "Lights.hpp"
#include "Mesh.hpp"
#include "PathUtils.hpp"
#include "Platform.hpp"
#include "PostProcessingStack.hpp"
#include "ProjectManager.hpp"
#include "RenderLayer.hpp"
#include "Resources.hpp"
#include "Scene.hpp"
#include "Shader.hpp"
#include "StrandsRenderer.hpp"
#include "TextureStorage.hpp"
#include "Times.hpp"
#include "WindowLayer.hpp"
#include "nlohmann/json.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <initializer_list>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <unordered_set>
#include <vector>

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
  size_t preview_capture_timing_warmup_frames = 0;
  std::optional<Camera::CameraRenderMode> preview_capture_render_mode;
  std::optional<CameraSettings::RayDebugView> preview_capture_ray_debug_view;
  std::optional<CameraSettings::ShaderExecutionReorderingMode> preview_capture_ser_mode;
  std::optional<bool> preview_capture_firefly_clamp_enabled;
  std::optional<float> preview_capture_firefly_clamp_threshold;
  std::optional<bool> preview_capture_emissive_triangle_nee_enabled;
  bool preview_force_full_ray_shader_variant = false;
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
  std::optional<RenderSettings::ShadowCascadeFitMode> preview_shadow_fit_mode;
  std::optional<int> preview_shadow_pcf_samples;
  std::optional<int> preview_shadow_debug_mode;
  std::optional<int> preview_shadow_debug_cascade;
  std::optional<int> preview_shadow_debug_light;
  std::optional<int> preview_shadow_light_count;
  bool preview_shadow_caster_fixture = false;
  bool preview_strand_fixture = false;
  bool preview_capture_deterministic = false;
  bool preview_capture_bistro_ddgi = false;
  bool preview_capture_m10_ray_transport = false;
  bool preview_post_processing_stress = false;
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

CameraSettings::RayDebugView ParsePreviewRayDebugView(const std::string& value) {
  const auto beauty_fallback = Camera::ParseRayDebugView(value, CameraSettings::RayDebugView::Beauty);
  const auto atlas_fallback = Camera::ParseRayDebugView(value, CameraSettings::RayDebugView::ValidationAtlas);
  if (beauty_fallback != CameraSettings::RayDebugView::Beauty ||
      atlas_fallback != CameraSettings::RayDebugView::ValidationAtlas) {
    return beauty_fallback;
  }
  throw std::invalid_argument("Unknown preview ray debug view: " + value);
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
    } else if (argument == "--preview-timing-warmup-frames") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument("--preview-timing-warmup-frames requires a non-negative integer.");
      }
      command_line.preview_capture_timing_warmup_frames =
          static_cast<size_t>(std::max(0, std::stoi(argv[++arg_index])));
    } else if (argument == "--preview-render-mode") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument("--preview-render-mode requires rasterization, raytracing, or rayquery.");
      }
      command_line.preview_capture_render_mode = ParsePreviewRenderMode(argv[++arg_index] ? argv[arg_index] : "");
    } else if (argument == "--preview-ray-debug" || argument == "--preview-ray-debug-view") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument(argument + " requires a ray debug view.");
      }
      command_line.preview_capture_ray_debug_view = ParsePreviewRayDebugView(argv[++arg_index] ? argv[arg_index] : "");
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
    } else if (argument == "--preview-ray-shader-variant") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument("--preview-ray-shader-variant requires auto or full.");
      }
      const std::string value = argv[++arg_index] ? argv[arg_index] : "";
      if (value == "full") {
        command_line.preview_force_full_ray_shader_variant = true;
      } else if (value != "auto") {
        throw std::invalid_argument("--preview-ray-shader-variant requires auto or full.");
      }
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
    } else if (argument == "--preview-shadow-fit") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument("--preview-shadow-fit requires stable-sphere or tight-aabb.");
      }
      command_line.preview_shadow_fit_mode = ParseShadowCascadeFitModeName(argv[++arg_index] ? argv[arg_index] : "");
    } else if (argument == "--preview-shadow-pcf-samples") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument("--preview-shadow-pcf-samples requires a value between 1 and 64.");
      }
      command_line.preview_shadow_pcf_samples = std::clamp(std::stoi(argv[++arg_index]), 1, 64);
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
    } else if (argument == "--preview-shadow-light-count") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument("--preview-shadow-light-count requires 1, 2, or 4.");
      }
      const auto count = std::stoi(argv[++arg_index]);
      if (count != 1 && count != 2 && count != 4) {
        throw std::invalid_argument("--preview-shadow-light-count requires 1, 2, or 4.");
      }
      command_line.preview_shadow_light_count = count;
    } else if (argument == "--preview-shadow-caster-fixture") {
      command_line.preview_shadow_caster_fixture = true;
    } else if (argument == "--preview-strand-fixture") {
      command_line.preview_strand_fixture = true;
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
    } else if (argument == "--preview-m10-ray-transport") {
      command_line.preview_capture_m10_ray_transport = true;
    } else if (argument == "--preview-post-processing-stress") {
      command_line.preview_post_processing_stress = true;
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
  if (command_line.preview_capture_ray_debug_view && !command_line.demo_preview_capture_path) {
    throw std::invalid_argument("--preview-ray-debug requires --capture-demo-preview.");
  }
  if (command_line.preview_capture_ray_debug_view &&
      (!command_line.preview_capture_render_mode ||
       !Camera::IsRayCameraRenderMode(*command_line.preview_capture_render_mode))) {
    throw std::invalid_argument("--preview-ray-debug requires --preview-render-mode raytracing or rayquery.");
  }
  if (command_line.demo_preview_capture_path) {
    auto extension = command_line.demo_preview_capture_path->extension().string();
    std::transform(extension.begin(), extension.end(), extension.begin(), [](const char character) {
      return static_cast<char>(std::tolower(static_cast<unsigned char>(character)));
    });
    if (extension != ".png" && extension != ".hdr") {
      throw std::invalid_argument("--capture-demo-preview output must use .png or .hdr.");
    }
    if (command_line.preview_capture_warmup_frames != 0 &&
        command_line.preview_capture_timing_warmup_frames >= command_line.preview_capture_warmup_frames) {
      throw std::invalid_argument("--preview-timing-warmup-frames must be smaller than --preview-warmup-frames.");
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
  if (command_line.preview_capture_m10_ray_transport && !command_line.demo_preview_capture_path) {
    throw std::invalid_argument("--preview-m10-ray-transport requires --capture-demo-preview.");
  }
  if (command_line.preview_post_processing_stress &&
      (!command_line.demo_preview_capture_path || !command_line.preview_capture_metrics_path)) {
    throw std::invalid_argument(
        "--preview-post-processing-stress requires --capture-demo-preview and --preview-metrics-json.");
  }
  if ((command_line.preview_shadow_fit_mode || command_line.preview_shadow_pcf_samples ||
       command_line.preview_shadow_debug_mode || command_line.preview_shadow_debug_cascade ||
       command_line.preview_shadow_debug_light || command_line.preview_shadow_light_count ||
       command_line.preview_shadow_caster_fixture) &&
      !command_line.demo_preview_capture_path) {
    throw std::invalid_argument("Preview shadow controls require --capture-demo-preview.");
  }
  if (command_line.preview_strand_fixture && !command_line.demo_preview_capture_path) {
    throw std::invalid_argument("--preview-strand-fixture requires --capture-demo-preview.");
  }
  if (command_line.preview_strand_fixture && command_line.demo_profile_id != DemoProfileId::RenderingRegression) {
    throw std::invalid_argument("--preview-strand-fixture requires --demo rendering-regression.");
  }
  if (command_line.preview_strand_fixture && command_line.preview_shadow_caster_fixture) {
    throw std::invalid_argument("--preview-strand-fixture cannot be combined with --preview-shadow-caster-fixture.");
  }
  if ((command_line.preview_shadow_split_lambda || command_line.preview_shadow_cascade_transition_width ||
       command_line.preview_shadow_distance_fade) &&
      !command_line.demo_preview_capture_path) {
    throw std::invalid_argument("Preview shadow overrides require --capture-demo-preview.");
  }
  if (command_line.preview_capture_bistro_ddgi && command_line.demo_profile_id != DemoProfileId::Bistro) {
    throw std::invalid_argument("--preview-bistro-ddgi requires --demo bistro.");
  }
  if (command_line.preview_capture_m10_ray_transport &&
      command_line.demo_profile_id != DemoProfileId::RenderingRegression) {
    throw std::invalid_argument("--preview-m10-ray-transport requires --demo rendering-regression.");
  }
  if (command_line.preview_post_processing_stress &&
      command_line.demo_profile_id != DemoProfileId::RenderingRegression) {
    throw std::invalid_argument("--preview-post-processing-stress requires --demo rendering-regression.");
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
         !TextureStorage::HasPendingUploads() && !GeometryStorage::HasPendingUploads() &&
         !BottomLevelAccelerationStructure::HasPendingStaticBuilds();
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

nlohmann::ordered_json TimingStatsJson(const std::vector<GpuTimestampStats>& timing_stats) {
  auto result = nlohmann::ordered_json::array();
  for (const auto& stats : timing_stats) {
    result.push_back({{"name", stats.name},
                      {"last_ms", stats.last_milliseconds},
                      {"average_ms", stats.AverageMilliseconds()},
                      {"median_ms", stats.MedianMilliseconds()},
                      {"p95_ms", stats.PercentileMilliseconds(0.95)},
                      {"minimum_ms", stats.minimum_milliseconds},
                      {"maximum_ms", stats.maximum_milliseconds},
                      {"total_ms", stats.total_milliseconds},
                      {"sample_count", stats.sample_count}});
  }
  return result;
}

nlohmann::ordered_json WaitReasonJson(const std::vector<GpuTimestampStats>& timing_stats,
                                      const std::initializer_list<std::string_view> names) {
  std::vector<double> samples;
  double total_milliseconds = 0.0;
  uint64_t sample_count = 0;
  for (const auto& stats : timing_stats) {
    if (std::find(names.begin(), names.end(), stats.name) == names.end()) {
      continue;
    }
    samples.insert(samples.end(), stats.samples_milliseconds.begin(), stats.samples_milliseconds.end());
    total_milliseconds += stats.total_milliseconds;
    sample_count += stats.sample_count;
  }
  std::sort(samples.begin(), samples.end());
  double median_milliseconds = 0.0;
  if (!samples.empty()) {
    const auto middle = samples.size() / 2;
    median_milliseconds = samples.size() % 2 == 0 ? (samples[middle - 1] + samples[middle]) * 0.5 : samples[middle];
  }
  return {{"count", sample_count}, {"total_ms", total_milliseconds}, {"median_ms", median_milliseconds}};
}

nlohmann::ordered_json SynchronizationWaitsJson(const std::vector<GpuTimestampStats>& timing_stats) {
  return {{"frame_slot_reuse", WaitReasonJson(timing_stats, {"Recycled Frame Fence Wait"})},
          {"capture_flush", WaitReasonJson(timing_stats, {"Capture Completion Fence Wait"})},
          {"required_geometry_upload",
           WaitReasonJson(timing_stats, {"Required Geometry Upload Fence Wait", "Required Geometry Upload Wait"})},
          {"required_camera_resize", WaitReasonJson(timing_stats, {"Required Camera Resize Fence Wait"})},
          {"required_texture_storage", WaitReasonJson(timing_stats, {"Required Texture Storage Fence Wait"})},
          {"just_submitted_frame", WaitReasonJson(timing_stats, {"Submitted Frame Fence Wait"})},
          {"redundant", WaitReasonJson(timing_stats, {"Redundant Fence Wait"})}};
}

nlohmann::ordered_json TlasUploadTelemetryJson(const TopLevelAccelerationStructure::UploadTelemetry& telemetry) {
  return {{"source_bytes", telemetry.source_bytes},
          {"uploaded_bytes", telemetry.uploaded_bytes},
          {"upload_ratio", telemetry.source_bytes == 0 ? 0.0
                                                       : static_cast<double>(telemetry.uploaded_bytes) /
                                                             static_cast<double>(telemetry.source_bytes)},
          {"range_count", telemetry.range_count},
          {"operation_count", telemetry.operation_count},
          {"build_count", telemetry.build_count},
          {"update_count", telemetry.update_count},
          {"no_op_count", telemetry.no_op_count},
          {"full_upload_count", telemetry.full_upload_count},
          {"zero_instance_upload_update_count", telemetry.zero_instance_upload_update_count}};
}

GpuMemorySnapshot MaxGpuMemorySnapshot(const GpuMemorySnapshot& left, const GpuMemorySnapshot& right) {
  auto result = left;
  result.block_count = std::max(left.block_count, right.block_count);
  result.allocation_count = std::max(left.allocation_count, right.allocation_count);
  result.block_bytes = std::max(left.block_bytes, right.block_bytes);
  result.allocation_bytes = std::max(left.allocation_bytes, right.allocation_bytes);
  if (result.heaps.size() < right.heaps.size()) {
    result.heaps.resize(right.heaps.size());
  }
  for (size_t heap_index = 0; heap_index < right.heaps.size(); ++heap_index) {
    auto& target = result.heaps[heap_index];
    const auto& source = right.heaps[heap_index];
    target.heap_index = source.heap_index;
    target.device_local = source.device_local;
    target.heap_size_bytes = source.heap_size_bytes;
    target.block_count = std::max(target.block_count, source.block_count);
    target.allocation_count = std::max(target.allocation_count, source.allocation_count);
    target.block_bytes = std::max(target.block_bytes, source.block_bytes);
    target.allocation_bytes = std::max(target.allocation_bytes, source.allocation_bytes);
    target.driver_usage_bytes = std::max(target.driver_usage_bytes, source.driver_usage_bytes);
    target.driver_budget_bytes = source.driver_budget_bytes;
  }
  return result;
}

nlohmann::ordered_json GpuMemorySnapshotJson(const GpuMemorySnapshot& snapshot) {
  nlohmann::ordered_json heaps = nlohmann::ordered_json::array();
  for (const auto& heap : snapshot.heaps) {
    heaps.push_back({{"heap_index", heap.heap_index},
                     {"category", heap.device_local ? "device_local" : "host"},
                     {"heap_size_bytes", heap.heap_size_bytes},
                     {"block_count", heap.block_count},
                     {"allocation_count", heap.allocation_count},
                     {"block_bytes", heap.block_bytes},
                     {"allocation_bytes", heap.allocation_bytes},
                     {"driver_usage_bytes", heap.driver_usage_bytes},
                     {"driver_budget_bytes", heap.driver_budget_bytes}});
  }
  return {{"block_count", snapshot.block_count},
          {"allocation_count", snapshot.allocation_count},
          {"block_bytes", snapshot.block_bytes},
          {"allocation_bytes", snapshot.allocation_bytes},
          {"heaps", std::move(heaps)}};
}

nlohmann::ordered_json RayCameraHistoryStatsJson(const RayCameraHistoryStats& stats) {
  return {{"live_camera_count", stats.live_camera_count},
          {"live_history_count", stats.live_history_count},
          {"live_ray_tracing_history_count", stats.live_ray_tracing_history_count},
          {"live_ray_query_history_count", stats.live_ray_query_history_count},
          {"valid_history_count", stats.valid_history_count},
          {"radiance_image_count", stats.radiance_image_count},
          {"convergence_image_count", stats.convergence_image_count},
          {"radiance_view_count", stats.radiance_view_count},
          {"convergence_view_count", stats.convergence_view_count},
          {"live_byte_size", stats.live_byte_size},
          {"peak_live_history_count", stats.peak_live_history_count},
          {"peak_live_byte_size", stats.peak_live_byte_size},
          {"creation_count", stats.creation_count},
          {"reuse_count", stats.reuse_count},
          {"invalidation_count", stats.invalidation_count},
          {"retirement_count", stats.retirement_count},
          {"live_output_descriptor_count", stats.live_output_descriptor_count},
          {"peak_live_output_descriptor_count", stats.peak_live_output_descriptor_count},
          {"output_descriptor_creation_count", stats.output_descriptor_creation_count},
          {"output_descriptor_reuse_count", stats.output_descriptor_reuse_count}};
}

nlohmann::ordered_json RayCameraFramePathStatsJson(const RenderLayer::RayCameraFramePathStats& stats) {
  const auto& cache = stats.render_graph_plan_cache;
  return {{"render_graph_plan_cache",
           {{"entry_count", cache.entry_count},
            {"capacity", cache.capacity},
            {"hit_count", cache.hit_count},
            {"miss_count", cache.miss_count},
            {"eviction_count", cache.eviction_count},
            {"compilation_count", cache.compilation_count},
            {"compilation_ms", cache.compilation_milliseconds}}},
          {"live_output_descriptor_count", stats.live_output_descriptor_count},
          {"peak_live_output_descriptor_count", stats.peak_live_output_descriptor_count},
          {"output_descriptor_creation_count", stats.output_descriptor_creation_count},
          {"output_descriptor_reuse_count", stats.output_descriptor_reuse_count},
          {"retained_frame_slot_count", stats.retained_frame_slot_count}};
}

nlohmann::ordered_json DescriptorSetLifetimeStatsJson(const DescriptorSet::LifetimeStats& stats) {
  return {{"live_count", stats.live_count},
          {"peak_live_count", stats.peak_live_count},
          {"creation_count", stats.creation_count}};
}

nlohmann::ordered_json RayTracingPipelineLifetimeStatsJson(const RayTracingPipeline::LifetimeStats& stats) {
  return {{"live_pipeline_count", stats.live_pipeline_count},
          {"peak_live_pipeline_count", stats.peak_live_pipeline_count},
          {"pipeline_creation_count", stats.pipeline_creation_count},
          {"live_shader_binding_table_count", stats.live_shader_binding_table_count},
          {"peak_live_shader_binding_table_count", stats.peak_live_shader_binding_table_count},
          {"shader_binding_table_creation_count", stats.shader_binding_table_creation_count}};
}

nlohmann::ordered_json RenderPassDrawStatsJson(const RenderPassDrawStats& stats) {
  const auto caster_draws = [&](const DirectionalShadowCasterKind kind) {
    return stats.directional_shadow_caster_draw_calls[static_cast<size_t>(kind)];
  };
  return {{"direct_draw_calls", stats.direct_draw_calls},
          {"indirect_draw_calls", stats.indirect_draw_calls},
          {"indirect_draw_commands", stats.indirect_draw_commands},
          {"primitive_count", stats.prim_count},
          {"total_draw_calls", stats.TotalDrawCalls()},
          {"directional_shadow_casters",
           {{"regular", caster_draws(DirectionalShadowCasterKind::Regular)},
            {"mesh_shader", caster_draws(DirectionalShadowCasterKind::MeshShader)},
            {"instanced", caster_draws(DirectionalShadowCasterKind::Instanced)},
            {"skinned", caster_draws(DirectionalShadowCasterKind::Skinned)},
            {"strands", caster_draws(DirectionalShadowCasterKind::Strands)},
            {"external", caster_draws(DirectionalShadowCasterKind::External)}}}};
}

nlohmann::ordered_json RenderPassDrawStatsJson(const RenderPassDrawBucket bucket) {
  RenderPassDrawStats result{};
  const auto bucket_index = static_cast<size_t>(bucket);
  for (const auto& frame_stats : Platform::GetInstance().render_pass_draw_stats) {
    if (frame_stats[bucket_index].TotalDrawCalls() > result.TotalDrawCalls()) {
      result = frame_stats[bucket_index];
    }
  }
  return RenderPassDrawStatsJson(result);
}

nlohmann::ordered_json DirectionalShadowDrawStatsJson() {
  return RenderPassDrawStatsJson(RenderPassDrawBucket::DirectionalLightShadow);
}

nlohmann::ordered_json StrandFixtureTelemetryJson(const std::shared_ptr<RenderLayer>& render_layer,
                                                  const std::shared_ptr<Scene>& scene) {
  size_t renderer_count = 0;
  size_t cast_shadow_count = 0;
  size_t configured_segment_count = 0;
  auto geometry_versions = nlohmann::ordered_json::array();
  const auto render_instances = render_layer ? render_layer->GetCurrentRenderInstanceStorage() : nullptr;
  if (scene) {
    if (const auto* owners = scene->UnsafeGetPrivateComponentOwnersList<StrandsRenderer>()) {
      for (const auto& owner : *owners) {
        const auto renderer = scene->GetOrSetPrivateComponent<StrandsRenderer>(owner).lock();
        const auto strands = renderer ? renderer->strands.Get<Strands>() : nullptr;
        if (!renderer || !renderer->IsEnabled() || !strands) {
          continue;
        }
        ++renderer_count;
        cast_shadow_count += renderer->cast_shadow ? 1 : 0;
        configured_segment_count += strands->GetSegmentAmount();
        geometry_versions.push_back(strands->GetVersion());
      }
    }
  }
  bool geometry_updated = renderer_count == 2;
  bool has_reuploaded_geometry = false;
  for (const auto& version : geometry_versions) {
    const auto value = version.get<uint32_t>();
    geometry_updated &= value >= 1;
    has_reuploaded_geometry |= value >= 2;
  }
  geometry_updated &= has_reuploaded_geometry;
  const auto registered_renderer_count =
      render_instances && render_instances->total_strands_segments == configured_segment_count ? renderer_count : 0;
  return {{"mesh_shader_supported", Platform::MeshShaderEnabled()},
          {"mesh_shader_enabled", Platform::MeshShaderEnabled() && render_layer && render_layer->enable_meshlet},
          {"registered_renderer_count", registered_renderer_count},
          {"cast_shadow_true_count", cast_shadow_count},
          {"cast_shadow_false_count", renderer_count - cast_shadow_count},
          {"segment_count", render_instances ? render_instances->total_strands_segments : 0},
          {"meshlet_count", render_instances ? render_instances->total_strand_meshlets : 0},
          {"geometry_versions", std::move(geometry_versions)},
          {"geometry_updated", geometry_updated},
          {"geometry_uploads_pending", GeometryStorage::HasPendingMeshUploads()},
          {"deferred_draws", RenderPassDrawStatsJson(RenderPassDrawBucket::DeferredGeometry)}};
}

void ConfigureDirectionalShadowLightCount(const std::shared_ptr<Scene>& scene, const int light_count) {
  if (!scene) {
    throw std::runtime_error("Directional-shadow validation requires an active scene.");
  }

  std::shared_ptr<DirectionalLight> source;
  if (const auto* owners = scene->UnsafeGetPrivateComponentOwnersList<DirectionalLight>()) {
    for (const auto& owner : *owners) {
      const auto light = scene->GetOrSetPrivateComponent<DirectionalLight>(owner).lock();
      if (!light) {
        continue;
      }
      if (!source && scene->IsEntityEnabled(owner) && light->IsEnabled()) {
        source = light;
      }
      light->SetEnabled(false);
    }
  }
  if (!source) {
    const auto entity = scene->CreateEntity("CSM Validation Directional Light 0");
    source = scene->GetOrSetPrivateComponent<DirectionalLight>(entity).lock();
  }
  if (!source) {
    throw std::runtime_error("Directional-shadow validation failed to create its source light.");
  }

  const auto diffuse = source->diffuse;
  const auto diffuse_brightness = source->diffuse_brightness;
  const auto bias = source->bias;
  const auto slope_bias = source->slope_bias;
  const auto normal_offset = source->normal_offset;
  const auto light_size = source->light_size;
  const auto configure = [&](const std::shared_ptr<DirectionalLight>& light) {
    light->SetEnabled(true);
    light->cast_shadow = true;
    light->diffuse = diffuse;
    light->diffuse_brightness = diffuse_brightness / static_cast<float>(light_count);
    light->bias = bias;
    light->slope_bias = slope_bias;
    light->normal_offset = normal_offset;
    light->light_size = light_size;
  };
  configure(source);

  for (int light_index = 1; light_index < light_count; ++light_index) {
    const auto entity = scene->CreateEntity("CSM Validation Directional Light " + std::to_string(light_index));
    const auto light = scene->GetOrSetPrivateComponent<DirectionalLight>(entity).lock();
    if (!light) {
      throw std::runtime_error("Directional-shadow validation failed to create a packed light.");
    }
    configure(light);
    Transform transform;
    transform.SetEulerRotation(glm::radians(
        glm::vec3(35.0f + 20.0f * static_cast<float>(light_index), 45.0f * static_cast<float>(light_index), 0.0f)));
    scene->SetDataComponent(entity, transform);
  }
  std::cout << "CSM validation directional shadow lights: " << light_count << std::endl;
}

nlohmann::ordered_json DirectionalShadowTelemetryJson(const std::shared_ptr<RenderLayer>& render_layer,
                                                      const std::shared_ptr<Camera>& camera) {
  nlohmann::ordered_json result;
  result["draws"] = DirectionalShadowDrawStatsJson();
  result["fit_policy"] =
      RenderSettings::GetShadowCascadeFitModeName(render_layer->render_settings.shadow_cascade_fit_mode);
  result["pcf_sample_count"] = render_layer->render_settings.directional_pcf_sample_amount;
  result["indirect_rendering_enabled"] = render_layer->enable_indirect_rendering;
  result["mesh_shader_enabled"] = Platform::MeshShaderEnabled() && render_layer->enable_meshlet;
  result["lights"] = nlohmann::ordered_json::array();

  const auto render_instances = render_layer->GetPreviousRenderInstanceStorage();
  if (!render_instances || !camera) {
    return result;
  }
  const auto telemetry = render_instances->GetDirectionalShadowTelemetry(camera->GetHandle());
  result["pcf_sample_count"] = telemetry.pcf_sample_amount;
  for (size_t local_light_index = 0; local_light_index < telemetry.lights.size(); ++local_light_index) {
    const auto& light = telemetry.lights[local_light_index];
    if (light.diffuse.w <= 0.5f || light.viewport.z <= 0 || light.viewport.w <= 0) {
      continue;
    }
    nlohmann::ordered_json cascades = nlohmann::ordered_json::array();
    for (int cascade = 0; cascade < 4; ++cascade) {
      const auto half_extent_x = light.light_frustum_width[cascade];
      const auto half_extent_y = light.light_frustum_height[cascade];
      const auto first_split_start = glm::max(camera->camera_settings.near_distance, 0.001f);
      cascades.push_back({{"index", cascade},
                          {"split_start", cascade == 0 ? first_split_start : telemetry.split_distances[cascade - 1]},
                          {"split_end", telemetry.split_distances[cascade]},
                          {"orthographic_half_extent", half_extent_x},
                          {"orthographic_half_extent_y", half_extent_y},
                          {"light_space_depth_span", light.light_frustum_distance[cascade] * 2.0f},
                          {"viewport", {light.viewport.x, light.viewport.y, light.viewport.z, light.viewport.w}},
                          {"world_units_per_texel", 2.0f * half_extent_x / static_cast<float>(light.viewport.z)},
                          {"world_units_per_texel_y", 2.0f * half_extent_y / static_cast<float>(light.viewport.w)},
                          {"pcf_radius_texels_x",
                           light.reserved_parameters.x * static_cast<float>(light.viewport.z) / (2.0f * half_extent_x)},
                          {"pcf_radius_texels_y", light.reserved_parameters.x * static_cast<float>(light.viewport.w) /
                                                      (2.0f * half_extent_y)}});
    }
    result["lights"].push_back({{"index", local_light_index},
                                {"pcf_radius_world", light.reserved_parameters.x},
                                {"bias", light.reserved_parameters.z},
                                {"slope_bias", light.reserved_parameters.y},
                                {"normal_offset", light.reserved_parameters.w},
                                {"cascades", cascades}});
  }
  return result;
}

std::shared_ptr<GraphicsPipeline> CreateCsmExternalShadowValidationPipeline() {
  auto pipeline = std::make_shared<GraphicsPipeline>();
  pipeline->vertex_shader = Shader::CreateTemporary(
      ShaderType::Vertex, Platform::GetShaderGlobalDefines(),
      Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Vertex/Lighting/DirectionalShadowValidation.vert");
  pipeline->fragment_shader =
      Shader::CreateTemporary(ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
                              Resources::GetDefaultResourcesPath() / "Shaders/Graphics/Fragment/Empty.frag");
  pipeline->geometry_type = GeometryType::Mesh;
  pipeline->depth_attachment_format = Platform::Constants::shadow_map;
  pipeline->stencil_attachment_format = VK_FORMAT_UNDEFINED;
  auto& push_constant_range = pipeline->push_constant_ranges.emplace_back();
  push_constant_range.size = sizeof(glm::mat4);
  push_constant_range.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
  pipeline->Initialize();
  if (!pipeline->Initialized()) {
    throw std::runtime_error("Failed to initialize the CSM external-caster validation pipeline.");
  }
  return pipeline;
}

nlohmann::ordered_json PostProcessingRuntimeStatsJson(const PostProcessingRuntimeStats& stats) {
  return {{"scratch_size", {stats.scratch_size.x, stats.scratch_size.y}},
          {"stack_handle", stats.stack_handle},
          {"stack_version", stats.stack_version},
          {"render_technique", stats.render_technique},
          {"scratch_generation", stats.scratch_generation},
          {"source_texture", stats.source_texture},
          {"result_texture", stats.result_texture},
          {"swap_texture", stats.swap_texture},
          {"taa_color_textures", {stats.taa_color_textures[0], stats.taa_color_textures[1]}},
          {"taa_depth_textures", {stats.taa_depth_textures[0], stats.taa_depth_textures[1]}},
          {"smaa_edges_texture", stats.smaa_edges_texture},
          {"smaa_blend_texture", stats.smaa_blend_texture},
          {"histogram_buffer", stats.histogram_buffer},
          {"luminance_buffer", stats.luminance_buffer},
          {"taa_frame_index", stats.taa_frame_index},
          {"taa_last_processed_frame", stats.taa_last_processed_frame},
          {"taa_history_valid", stats.taa_history_valid},
          {"auto_exposure_time_initialized", stats.auto_exposure_time_initialized},
          {"luminance_reset_pending", stats.luminance_reset_pending},
          {"auto_exposure_process_count", stats.auto_exposure_process_count},
          {"auto_exposure_reset_count", stats.auto_exposure_reset_count},
          {"temporal_reset_count", stats.temporal_reset_count},
          {"version_reset_count", stats.version_reset_count},
          {"resolution_reset_count", stats.resolution_reset_count},
          {"technique_reset_count", stats.technique_reset_count},
          {"descriptor_sets", stats.descriptor_sets}};
}

void RequirePostProcessingStress(const bool condition, const std::string& message) {
  if (!condition) {
    throw std::runtime_error("M16b post-processing stress failed: " + message);
  }
}

void CheckPostProcessingStress(std::vector<std::string>& failures, const bool condition, const std::string& message) {
  if (!condition) {
    failures.emplace_back(message);
  }
}

void CheckDistinctPostProcessingResources(std::vector<std::string>& failures, const std::vector<uint64_t>& first,
                                          const std::vector<uint64_t>& second, const std::string& label) {
  std::unordered_set<uint64_t> first_resources;
  for (const auto resource : first) {
    CheckPostProcessingStress(failures, resource != 0, label + " contains a null first-camera resource");
    first_resources.emplace(resource);
  }
  for (const auto resource : second) {
    CheckPostProcessingStress(failures, resource != 0, label + " contains a null second-camera resource");
    CheckPostProcessingStress(failures, first_resources.find(resource) == first_resources.end(),
                              label + " aliases between cameras");
  }
}

struct PostProcessingStressCamera {
  Entity entity;
  std::shared_ptr<Camera> camera;
  uint64_t handle = 0;
};

nlohmann::ordered_json PostProcessingStressCameraJson(const PostProcessingStressCamera& stress_camera) {
  return {{"camera_handle", stress_camera.handle},
          {"size", {stress_camera.camera->GetSize().x, stress_camera.camera->GetSize().y}},
          {"frame_count", stress_camera.camera->GetFrameCount()},
          {"rendered", stress_camera.camera->Rendered()},
          {"runtime", PostProcessingRuntimeStatsJson(stress_camera.camera->GetPostProcessingRuntimeStats())}};
}

nlohmann::ordered_json RunPostProcessingStress() {
  const auto scene = ApplicationContext::Get().GetActiveScene();
  RequirePostProcessingStress(static_cast<bool>(scene), "active scene is unavailable");

  const auto stack = AssetManager::CreateTemporaryAsset<PostProcessingStack>();
  RequirePostProcessingStress(static_cast<bool>(stack), "shared stack creation failed");
  stack->enable_ambient_occlusion = false;
  stack->enable_screen_space_reflection = false;
  stack->enable_bloom = false;
  stack->enable_anti_aliasing = true;
  stack->enable_tone_mapping = true;
  stack->anti_aliasing->algorithm = AntiAliasing::Algorithm::Taa;
  stack->anti_aliasing->ApplyTaaPreset(AntiAliasing::TaaPreset::BestQuality);
  stack->tone_mapping->auto_exposure = true;
  stack->tone_mapping->auto_exposure_delta_time_override = 1.0f / 60.0f;
  stack->tone_mapping->dither = false;
  stack->SetUnsaved();

  const auto create_camera = [&](const std::string& name, const glm::uvec2 size) {
    PostProcessingStressCamera result;
    result.entity = scene->CreateEntity(name);
    result.handle = scene->GetEntityHandle(result.entity).GetValue();
    result.camera = scene->GetOrSetPrivateComponent<Camera>(result.entity).lock();
    RequirePostProcessingStress(static_cast<bool>(result.camera), "camera creation failed");
    result.camera->camera_render_mode = Camera::CameraRenderMode::Rasterization;
    result.camera->camera_settings.use_clear_color = true;
    result.camera->post_processing_stack_ref = stack;
    result.camera->Resize(size);
    return result;
  };
  const auto render = [&](const std::vector<std::shared_ptr<Camera>>& cameras, const size_t frame_count) {
    size_t rendered_frames = 0;
    size_t attempts = 0;
    while (rendered_frames < frame_count) {
      for (const auto& camera : cameras) {
        camera->SetRequireRendering(true);
      }
      RequirePostProcessingStress(ApplicationContext::Get().Loop(), "application ended during camera rendering");
      const bool all_rendered = std::all_of(cameras.begin(), cameras.end(), [](const auto& camera) {
        return camera->Rendered();
      });
      if (all_rendered) {
        ++rendered_frames;
      }
      RequirePostProcessingStress(++attempts <= frame_count + 16, "temporary cameras were not rendered");
    }
  };
  const auto settle_deleted_cameras = [&](PostProcessingStressCamera& first, PostProcessingStressCamera& second) {
    scene->DeleteEntity(first.entity);
    scene->DeleteEntity(second.entity);
    first.camera.reset();
    second.camera.reset();
    for (int frame = 0; frame < Platform::GetMaxFramesInFlight() + 2; ++frame) {
      RequirePostProcessingStress(ApplicationContext::Get().Loop(),
                                  "application ended while retiring camera resources");
    }
    Platform::WaitForFrameSubmissions("M16b Post-Processing Stress Cleanup Fence Wait");
  };

  auto first = create_camera("M16b Shared Stack Camera A", {640, 360});
  auto second = create_camera("M16b Shared Stack Camera B", {320, 180});
  render({first.camera, second.camera}, 8);
  const auto taa_first = first.camera->GetPostProcessingRuntimeStats();
  const auto taa_second = second.camera->GetPostProcessingRuntimeStats();
  std::vector<std::string> failures;
  nlohmann::ordered_json result;
  CheckPostProcessingStress(failures, taa_first.scratch_size == glm::uvec2(640, 360), "camera A scratch size is wrong");
  CheckPostProcessingStress(failures, taa_second.scratch_size == glm::uvec2(320, 180),
                            "camera B scratch size is wrong");
  CheckPostProcessingStress(failures, taa_first.stack_handle != 0 && taa_first.stack_handle == taa_second.stack_handle,
                            "cameras did not observe one shared stack");
  CheckPostProcessingStress(failures, taa_first.taa_history_valid && taa_second.taa_history_valid,
                            "TAA histories did not initialize");
  CheckPostProcessingStress(failures,
                            taa_first.auto_exposure_process_count > 0 && taa_second.auto_exposure_process_count > 0 &&
                                !taa_first.luminance_reset_pending && !taa_second.luminance_reset_pending,
                            "auto exposure histories did not initialize");
  CheckPostProcessingStress(failures, taa_first.taa_last_processed_frame == taa_second.taa_last_processed_frame,
                            "cameras were not processed in the same frame");
  CheckDistinctPostProcessingResources(
      failures, {taa_first.source_texture, taa_first.result_texture, taa_first.swap_texture},
      {taa_second.source_texture, taa_second.result_texture, taa_second.swap_texture}, "scratch textures");
  CheckDistinctPostProcessingResources(failures,
                                       {taa_first.taa_color_textures[0], taa_first.taa_color_textures[1],
                                        taa_first.taa_depth_textures[0], taa_first.taa_depth_textures[1]},
                                       {taa_second.taa_color_textures[0], taa_second.taa_color_textures[1],
                                        taa_second.taa_depth_textures[0], taa_second.taa_depth_textures[1]},
                                       "TAA histories");
  CheckDistinctPostProcessingResources(failures, {taa_first.histogram_buffer, taa_first.luminance_buffer},
                                       {taa_second.histogram_buffer, taa_second.luminance_buffer},
                                       "auto-exposure buffers");
  CheckDistinctPostProcessingResources(failures, taa_first.descriptor_sets, taa_second.descriptor_sets,
                                       "descriptor sets");
  result["taa_baseline"] = {{"a", PostProcessingStressCameraJson(first)},
                            {"b", PostProcessingStressCameraJson(second)}};
  const auto second_before_a_only = PostProcessingRuntimeStatsJson(taa_second);
  const auto second_frame_before_a_only = second.camera->GetFrameCount();
  render({first.camera}, 1);
  const auto after_a_only_first = first.camera->GetPostProcessingRuntimeStats();
  const auto after_a_only_second = second.camera->GetPostProcessingRuntimeStats();
  CheckPostProcessingStress(failures, after_a_only_first.taa_last_processed_frame > taa_first.taa_last_processed_frame,
                            "camera A TAA history did not advance independently");
  CheckPostProcessingStress(failures,
                            after_a_only_first.auto_exposure_process_count > taa_first.auto_exposure_process_count,
                            "camera A exposure history did not advance independently");
  CheckPostProcessingStress(failures,
                            PostProcessingRuntimeStatsJson(after_a_only_second) == second_before_a_only &&
                                second.camera->GetFrameCount() == second_frame_before_a_only &&
                                !second.camera->Rendered(),
                            "camera B changed while only camera A rendered");
  result["after_a_only"] = {{"a", PostProcessingStressCameraJson(first)},
                            {"b", PostProcessingStressCameraJson(second)}};

  const auto old_version = stack->GetVersion();
  stack->anti_aliasing->algorithm = AntiAliasing::Algorithm::Smaa;
  stack->SetUnsaved();
  CheckPostProcessingStress(failures, stack->GetVersion() == old_version + 1,
                            "settings mutation did not advance version once");
  render({first.camera}, 1);
  const auto version_first = first.camera->GetPostProcessingRuntimeStats();
  const auto version_second_pending = second.camera->GetPostProcessingRuntimeStats();
  CheckPostProcessingStress(
      failures,
      version_first.stack_version == stack->GetVersion() && version_second_pending.stack_version == old_version,
      "stack version was not observed lazily per camera");
  CheckPostProcessingStress(failures, version_first.version_reset_count == after_a_only_first.version_reset_count + 1,
                            "camera A version reset count is wrong");
  CheckPostProcessingStress(failures,
                            version_first.auto_exposure_reset_count == after_a_only_first.auto_exposure_reset_count + 1,
                            "camera A exposure history was not reset by version");
  CheckPostProcessingStress(failures, version_first.temporal_reset_count == after_a_only_first.temporal_reset_count + 1,
                            "camera A temporal state was not reset exactly once by version");
  CheckPostProcessingStress(failures,
                            version_first.scratch_generation == after_a_only_first.scratch_generation &&
                                version_first.source_texture == after_a_only_first.source_texture &&
                                version_first.result_texture == after_a_only_first.result_texture &&
                                version_first.swap_texture == after_a_only_first.swap_texture,
                            "camera A compatible scratch was replaced by version invalidation");
  result["after_version_a"] = {{"a", PostProcessingStressCameraJson(first)},
                               {"b", PostProcessingStressCameraJson(second)}};

  render({second.camera}, 1);
  const auto version_second = second.camera->GetPostProcessingRuntimeStats();
  CheckPostProcessingStress(failures,
                            version_second.stack_version == stack->GetVersion() &&
                                version_second.version_reset_count == after_a_only_second.version_reset_count + 1,
                            "camera B did not observe its own version reset");
  CheckPostProcessingStress(
      failures,
      version_second.auto_exposure_reset_count == after_a_only_second.auto_exposure_reset_count + 1 &&
          version_second.temporal_reset_count == after_a_only_second.temporal_reset_count + 1,
      "camera B temporal subsystems were not reset exactly once by version");
  CheckPostProcessingStress(failures,
                            version_second.scratch_generation == after_a_only_second.scratch_generation &&
                                version_second.source_texture == after_a_only_second.source_texture &&
                                version_second.result_texture == after_a_only_second.result_texture &&
                                version_second.swap_texture == after_a_only_second.swap_texture,
                            "camera B compatible scratch was replaced by version invalidation");
  CheckDistinctPostProcessingResources(failures, {version_first.smaa_edges_texture, version_first.smaa_blend_texture},
                                       {version_second.smaa_edges_texture, version_second.smaa_blend_texture},
                                       "SMAA targets");
  result["after_version_b"] = {{"a", PostProcessingStressCameraJson(first)},
                               {"b", PostProcessingStressCameraJson(second)}};

  const auto second_before_resize = PostProcessingRuntimeStatsJson(version_second);
  first.camera->Resize({480, 270});
  render({first.camera}, 1);
  const auto resized_first = first.camera->GetPostProcessingRuntimeStats();
  CheckPostProcessingStress(failures,
                            resized_first.scratch_size == glm::uvec2(480, 270) &&
                                resized_first.scratch_generation == version_first.scratch_generation + 1 &&
                                resized_first.resolution_reset_count == version_first.resolution_reset_count + 1,
                            "camera A resolution invalidation is wrong");
  CheckPostProcessingStress(failures,
                            resized_first.version_reset_count == version_first.version_reset_count &&
                                resized_first.technique_reset_count == version_first.technique_reset_count,
                            "camera A resize affected another invalidation key");
  CheckPostProcessingStress(failures,
                            resized_first.temporal_reset_count == version_first.temporal_reset_count + 1 &&
                                resized_first.auto_exposure_reset_count == version_first.auto_exposure_reset_count + 1,
                            "camera A resize did not reset every temporal subsystem exactly once");
  CheckPostProcessingStress(
      failures, PostProcessingRuntimeStatsJson(second.camera->GetPostProcessingRuntimeStats()) == second_before_resize,
      "camera A resize changed camera B");
  result["after_resize_a"] = {{"a", PostProcessingStressCameraJson(first)},
                              {"b", PostProcessingStressCameraJson(second)}};

  const auto first_before_technique = PostProcessingRuntimeStatsJson(resized_first);
  const auto technique_second_before = second.camera->GetPostProcessingRuntimeStats();
  CameraInfoBlock camera_info_block{};
  second.camera->camera_render_mode = Camera::CameraRenderMode::RayQuery;
  second.camera->SetRequireRendering(true);
  second.camera->UpdateCameraInfoBlock(camera_info_block, GlobalTransform{});
  second.camera->ResetRenderState();
  const auto ray_query_second = second.camera->GetPostProcessingRuntimeStats();
  CheckPostProcessingStress(
      failures,
      ray_query_second.render_technique == static_cast<uint32_t>(Camera::CameraRenderMode::RayQuery) &&
          ray_query_second.technique_reset_count == technique_second_before.technique_reset_count + 1 &&
          ray_query_second.resolution_reset_count == technique_second_before.resolution_reset_count &&
          ray_query_second.version_reset_count == technique_second_before.version_reset_count &&
          ray_query_second.scratch_generation == technique_second_before.scratch_generation &&
          ray_query_second.temporal_reset_count == technique_second_before.temporal_reset_count + 1 &&
          ray_query_second.auto_exposure_reset_count == technique_second_before.auto_exposure_reset_count + 1,
      "RayQuery technique invalidation affected another key");
  CheckPostProcessingStress(
      failures, PostProcessingRuntimeStatsJson(first.camera->GetPostProcessingRuntimeStats()) == first_before_technique,
      "camera B technique switch changed camera A");
  result["after_technique_b_ray_query"] = {{"a", PostProcessingStressCameraJson(first)},
                                           {"b", PostProcessingStressCameraJson(second)}};
  second.camera->camera_render_mode = Camera::CameraRenderMode::Rasterization;
  second.camera->SetRequireRendering(true);
  second.camera->UpdateCameraInfoBlock(camera_info_block, GlobalTransform{});
  second.camera->ResetRenderState();
  const auto raster_second = second.camera->GetPostProcessingRuntimeStats();
  CheckPostProcessingStress(
      failures,
      raster_second.render_technique == static_cast<uint32_t>(Camera::CameraRenderMode::Rasterization) &&
          raster_second.technique_reset_count == ray_query_second.technique_reset_count + 1 &&
          raster_second.resolution_reset_count == ray_query_second.resolution_reset_count &&
          raster_second.version_reset_count == ray_query_second.version_reset_count &&
          raster_second.scratch_generation == ray_query_second.scratch_generation &&
          raster_second.temporal_reset_count == ray_query_second.temporal_reset_count + 1 &&
          raster_second.auto_exposure_reset_count == ray_query_second.auto_exposure_reset_count + 1,
      "raster technique restoration did not use the independent path");
  result["after_technique_b_rasterization"] = {{"a", PostProcessingStressCameraJson(first)},
                                               {"b", PostProcessingStressCameraJson(second)}};

  settle_deleted_cameras(first, second);
  const auto churn_baseline_memory = Platform::GetGpuMemorySnapshot();
  const auto churn_baseline_descriptors = DescriptorSet::GetLifetimeStats();
  uint64_t first_active_allocation_count = 0;
  uint64_t first_active_allocation_bytes = 0;
  uint64_t first_active_descriptor_count = 0;
  auto churn_cycles = nlohmann::ordered_json::array();
  for (size_t iteration = 0; iteration < 4; ++iteration) {
    auto churn_first = create_camera("M16b Churn Camera A", {480, 270});
    auto churn_second = create_camera("M16b Churn Camera B", {320, 180});
    render({churn_first.camera, churn_second.camera}, 1);
    const auto active_memory = Platform::GetGpuMemorySnapshot();
    const auto active_descriptors = DescriptorSet::GetLifetimeStats();
    churn_cycles.push_back({{"iteration", iteration},
                            {"memory", GpuMemorySnapshotJson(active_memory)},
                            {"descriptor_sets", DescriptorSetLifetimeStatsJson(active_descriptors)}});
    if (iteration == 0) {
      first_active_allocation_count = active_memory.allocation_count;
      first_active_allocation_bytes = active_memory.allocation_bytes;
      first_active_descriptor_count = active_descriptors.live_count;
    } else {
      CheckPostProcessingStress(failures,
                                active_memory.allocation_count <= first_active_allocation_count &&
                                    active_memory.allocation_bytes <= first_active_allocation_bytes &&
                                    active_descriptors.live_count <= first_active_descriptor_count,
                                "multi-camera churn exceeded the first active-cycle bound");
    }
    settle_deleted_cameras(churn_first, churn_second);
  }
  const auto churn_final_memory = Platform::GetGpuMemorySnapshot();
  const auto churn_final_descriptors = DescriptorSet::GetLifetimeStats();
  CheckPostProcessingStress(failures,
                            churn_final_memory.allocation_count == churn_baseline_memory.allocation_count &&
                                churn_final_memory.allocation_bytes == churn_baseline_memory.allocation_bytes &&
                                churn_final_descriptors.live_count == churn_baseline_descriptors.live_count,
                            "multi-camera churn did not return allocations to baseline");
  result["churn"] = {
      {"iterations", 4},
      {"baseline_memory", GpuMemorySnapshotJson(churn_baseline_memory)},
      {"baseline_descriptor_sets", DescriptorSetLifetimeStatsJson(churn_baseline_descriptors)},
      {"first_active_memory",
       {{"allocation_count", first_active_allocation_count}, {"allocation_bytes", first_active_allocation_bytes}}},
      {"first_active_descriptor_count", first_active_descriptor_count},
      {"cycles", std::move(churn_cycles)},
      {"final_memory", GpuMemorySnapshotJson(churn_final_memory)},
      {"final_descriptor_sets", DescriptorSetLifetimeStatsJson(churn_final_descriptors)}};
  result["stack_version_before_mutation"] = old_version;
  result["stack_version_after_mutation"] = stack->GetVersion();
  result["failures"] = std::move(failures);
  result["pass"] = result["failures"].empty();
  return result;
}

void CaptureDemoPreview(
    const std::filesystem::path& output_path, const std::optional<std::filesystem::path>& metrics_path, const int width,
    const int height, const size_t warmup_frames, const size_t timing_warmup_frames,
    const std::optional<DemoProfileId> demo_profile_id,
    const std::optional<Camera::CameraRenderMode>& preview_render_mode,
    const std::optional<CameraSettings::RayDebugView>& preview_ray_debug_view,
    const std::optional<CameraSettings::ShaderExecutionReorderingMode>& preview_ser_mode,
    const std::optional<bool>& preview_firefly_clamp_enabled,
    const std::optional<float>& preview_firefly_clamp_threshold,
    const std::optional<bool>& preview_emissive_triangle_nee_enabled, const bool force_full_ray_shader_variant,
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
    const std::optional<float>& preview_shadow_distance_fade,
    const std::optional<RenderSettings::ShadowCascadeFitMode>& preview_shadow_fit_mode,
    const std::optional<int>& preview_shadow_pcf_samples, const std::optional<int>& preview_shadow_debug_mode,
    const std::optional<int>& preview_shadow_debug_cascade, const std::optional<int>& preview_shadow_debug_light,
    const std::optional<int>& preview_shadow_light_count, const bool preview_shadow_caster_fixture,
    const bool preview_strand_fixture, const bool deterministic_capture, const bool preview_bistro_ddgi,
    const bool preview_m10_ray_transport, const bool preview_post_processing_stress) {
  auto output_extension = output_path.extension().string();
  std::transform(output_extension.begin(), output_extension.end(), output_extension.begin(), [](const char character) {
    return static_cast<char>(std::tolower(static_cast<unsigned char>(character)));
  });
  const bool linear_hdr_output = output_extension == ".hdr";
  const glm::uvec2 preview_resolution(static_cast<uint32_t>(width), static_cast<uint32_t>(height));
  if (preview_strand_fixture) {
    if (!Platform::MeshShaderEnabled()) {
      throw std::runtime_error("Strand validation requires mesh-shader support.");
    }
    ConfigureStrandMeshShaderValidation(ApplicationContext::Get().GetActiveScene());
  }
  if (preview_m10_ray_transport) {
    ConfigureM10RayTransportValidation(ApplicationContext::Get().GetActiveScene());
  }
  if (preview_shadow_light_count) {
    ConfigureDirectionalShadowLightCount(ApplicationContext::Get().GetActiveScene(), *preview_shadow_light_count);
  }
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
  auto resolved_camera_position = preview_camera_position;
  auto resolved_camera_look_at = preview_camera_look_at;
  if (preview_strand_fixture && !resolved_camera_position) {
    resolved_camera_position = glm::vec3(0.0f, 0.45f, 4.5f);
    resolved_camera_look_at = glm::vec3(0.0f, 0.0f, -2.5f);
  }
  ApplyPreviewCameraOverride(editor_layer, resolved_camera_position, resolved_camera_look_at);
  if (preview_render_mode) {
    scene_camera->camera_render_mode = *preview_render_mode;
    scene_camera->ResetFrameCount();
  }
  if (preview_ray_debug_view) {
    scene_camera->camera_settings.ray_debug_view = *preview_ray_debug_view;
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
      }
      scene_camera->ResetFrameCount();
    }
  }
  if (preview_shadow_split_lambda || preview_shadow_cascade_transition_width || preview_shadow_distance_fade ||
      preview_shadow_fit_mode || preview_shadow_pcf_samples || preview_shadow_debug_mode ||
      preview_shadow_debug_cascade || preview_shadow_debug_light) {
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
    if (preview_shadow_fit_mode) {
      render_layer->render_settings.shadow_cascade_fit_mode = *preview_shadow_fit_mode;
    }
    if (preview_shadow_pcf_samples) {
      render_layer->render_settings.directional_pcf_sample_amount = std::clamp(*preview_shadow_pcf_samples, 1, 64);
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
  const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>();
  if (preview_strand_fixture) {
    if (!render_layer || resolved_render_mode != Camera::CameraRenderMode::Rasterization) {
      throw std::runtime_error("Strand validation requires the raster RenderLayer path.");
    }
    render_layer->enable_meshlet = true;
  }
  std::shared_ptr<GraphicsPipeline> external_shadow_pipeline;
  std::shared_ptr<Mesh> external_shadow_mesh;
  glm::mat4 external_shadow_model(1.0f);
  const auto register_external_shadow_caster = [&]() {
    if (!preview_shadow_caster_fixture) {
      return;
    }
    render_layer->RenderToDirectionalLightShadowMap(
        [external_shadow_pipeline, external_shadow_mesh, external_shadow_model](
            const VkCommandBuffer command_buffer, const RenderLayer::DirectionalLightShadowMapView& view) {
          external_shadow_pipeline->states.ResetAllStates(0);
          external_shadow_pipeline->states.SetViewportScissor(view.viewport);
          external_shadow_pipeline->Bind(command_buffer);
          external_shadow_pipeline->PushConstant(command_buffer, 0, view.light_space_matrix * external_shadow_model);
          GeometryStorage::BindVertices(command_buffer);
          external_shadow_mesh->DrawIndexed(command_buffer, external_shadow_pipeline->states, 1);
          return external_shadow_mesh->GetTriangleAmount();
        });
  };
  if (Camera::IsRayCameraRenderMode(resolved_render_mode)) {
    if (!render_layer) {
      throw std::runtime_error("Ray preview capture requires RenderLayer.");
    }
    render_layer->force_full_ray_camera_shader_variant = force_full_ray_shader_variant;
  }
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
  if (preview_strand_fixture) {
    UpdateStrandMeshShaderValidationGeometry(ApplicationContext::Get().GetActiveScene());
    WaitForDemoPreviewSceneInputsReady();
  }
  if (preview_shadow_caster_fixture) {
    if (!render_layer || resolved_render_mode != Camera::CameraRenderMode::Rasterization) {
      throw std::runtime_error("CSM caster validation requires the raster RenderLayer path.");
    }
    ConfigureCsmCasterValidation(ApplicationContext::Get().GetActiveScene());
    WaitForDemoPreviewSceneInputsReady();
    EnableCsmCasterValidation(ApplicationContext::Get().GetActiveScene());
    render_layer->enable_meshlet = false;
    external_shadow_pipeline = CreateCsmExternalShadowValidationPipeline();
    external_shadow_mesh = Resources::GetInstance().GetPrimitives().cube;
    external_shadow_model =
        glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, -1.0f, -4.0f)) * glm::scale(glm::mat4(1.0f), glm::vec3(0.45f));
  }
  nlohmann::ordered_json post_processing_stress = nullptr;
  if (preview_post_processing_stress) {
    try {
      RequirePostProcessingStress(!linear_hdr_output && resolved_render_mode == Camera::CameraRenderMode::Rasterization,
                                  "stress capture must use rasterization and PNG output");
      post_processing_stress = RunPostProcessingStress();
    } catch (...) {
      Platform::DrainGpuResourceWork();
      throw;
    }
  }
  if (Camera::IsRayCameraRenderMode(resolved_render_mode)) {
    const auto technique = resolved_render_mode == Camera::CameraRenderMode::RayQuery
                               ? RayCameraShaderTechnique::RayQuery
                               : RayCameraShaderTechnique::RayTracing;
    constexpr size_t max_variant_wait_frames = 30000;
    size_t wait_frames = 0;
    while (!render_layer->IsRayCameraShaderVariantReady(technique)) {
      const auto stats = render_layer->GetRayCameraShaderVariantStats(technique);
      if (stats.failed) {
        throw std::runtime_error("Ray camera shader variant failed: " + stats.last_error);
      }
      if (!ApplicationContext::Get().Loop()) {
        throw std::runtime_error("Application ended before the ray camera shader variant became ready.");
      }
      if (++wait_frames >= max_variant_wait_frames) {
        throw std::runtime_error("Timed out waiting for the ray camera shader variant.");
      }
    }
  }
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
  const auto startup_cpu_timing_stats = Platform::GetCpuTimingStats();
  const auto startup_tlas_upload = render_layer->GetTlasUploadTelemetry();
  auto capture_tlas_upload_baseline = startup_tlas_upload;
  const auto startup_gpu_memory = Platform::GetGpuMemorySnapshot();
  auto peak_gpu_memory = startup_gpu_memory;
  auto final_gpu_memory = startup_gpu_memory;
  const auto startup_ray_camera_history = render_layer->GetRayCameraHistoryStats();
  auto measurement_ray_camera_history = startup_ray_camera_history;
  auto final_ray_camera_history = startup_ray_camera_history;
  uint64_t minimum_live_ray_camera_histories = startup_ray_camera_history.live_history_count;
  uint64_t maximum_live_ray_camera_histories = startup_ray_camera_history.live_history_count;
  const auto startup_ray_camera_frame_path = render_layer->GetRayCameraFramePathStats();
  auto measurement_ray_camera_frame_path = startup_ray_camera_frame_path;
  auto final_ray_camera_frame_path = startup_ray_camera_frame_path;
  const auto startup_descriptor_sets = DescriptorSet::GetLifetimeStats();
  auto measurement_descriptor_sets = startup_descriptor_sets;
  auto final_descriptor_sets = startup_descriptor_sets;
  uint64_t minimum_live_descriptor_sets = startup_descriptor_sets.live_count;
  uint64_t maximum_live_descriptor_sets = startup_descriptor_sets.live_count;
  const auto startup_ray_tracing_pipelines = RayTracingPipeline::GetLifetimeStats();
  auto measurement_ray_tracing_pipelines = startup_ray_tracing_pipelines;
  auto final_ray_tracing_pipelines = startup_ray_tracing_pipelines;
  uint64_t minimum_live_ray_tracing_pipelines = startup_ray_tracing_pipelines.live_pipeline_count;
  uint64_t maximum_live_ray_tracing_pipelines = startup_ray_tracing_pipelines.live_pipeline_count;
  uint64_t minimum_live_shader_binding_tables = startup_ray_tracing_pipelines.live_shader_binding_table_count;
  uint64_t maximum_live_shader_binding_tables = startup_ray_tracing_pipelines.live_shader_binding_table_count;
  double memory_telemetry_seconds = 0.0;
  Platform::SetGpuTimestampCaptureEnabled(true);
  auto measurement_start_time = std::chrono::steady_clock::now();
  const bool temporal_motion_capture =
      demo_profile_id == DemoProfileId::RenderingRegression && preview_anti_aliasing_motion_sequence.value_or(false);
  const bool wait_for_ray_accumulation =
      Camera::IsRayCameraRenderMode(resolved_render_mode) && !temporal_motion_capture;
  constexpr size_t max_capture_frame_slack = 30000;
  const size_t max_capture_frames = warmup_frames + max_capture_frame_slack;
  size_t capture_frame_count = 0;
  while ((wait_for_ray_accumulation && scene_camera->GetFrameCount() < warmup_frames) ||
         (!wait_for_ray_accumulation && capture_frame_count < warmup_frames)) {
    register_external_shadow_caster();
    if (!ApplicationContext::Get().Loop()) {
      throw std::runtime_error("Application ended before demo preview capture completed.");
    }
    const auto memory_telemetry_start = std::chrono::steady_clock::now();
    final_gpu_memory = Platform::GetGpuMemorySnapshot();
    peak_gpu_memory = MaxGpuMemorySnapshot(peak_gpu_memory, final_gpu_memory);
    final_ray_camera_history = render_layer->GetRayCameraHistoryStats();
    final_ray_camera_frame_path = render_layer->GetRayCameraFramePathStats();
    minimum_live_ray_camera_histories =
        std::min(minimum_live_ray_camera_histories, final_ray_camera_history.live_history_count);
    maximum_live_ray_camera_histories =
        std::max(maximum_live_ray_camera_histories, final_ray_camera_history.live_history_count);
    final_descriptor_sets = DescriptorSet::GetLifetimeStats();
    minimum_live_descriptor_sets = std::min(minimum_live_descriptor_sets, final_descriptor_sets.live_count);
    maximum_live_descriptor_sets = std::max(maximum_live_descriptor_sets, final_descriptor_sets.live_count);
    final_ray_tracing_pipelines = RayTracingPipeline::GetLifetimeStats();
    minimum_live_ray_tracing_pipelines =
        std::min(minimum_live_ray_tracing_pipelines, final_ray_tracing_pipelines.live_pipeline_count);
    maximum_live_ray_tracing_pipelines =
        std::max(maximum_live_ray_tracing_pipelines, final_ray_tracing_pipelines.live_pipeline_count);
    minimum_live_shader_binding_tables =
        std::min(minimum_live_shader_binding_tables, final_ray_tracing_pipelines.live_shader_binding_table_count);
    maximum_live_shader_binding_tables =
        std::max(maximum_live_shader_binding_tables, final_ray_tracing_pipelines.live_shader_binding_table_count);
    memory_telemetry_seconds +=
        std::chrono::duration<double>(std::chrono::steady_clock::now() - memory_telemetry_start).count();
    ++capture_frame_count;
    if (capture_frame_count == std::min(timing_warmup_frames, warmup_frames)) {
      Platform::WaitForFrameSubmissions("Capture Warmup Fence Wait");
      Platform::ResetGpuTimestampStats();
      capture_tlas_upload_baseline = render_layer->GetTlasUploadTelemetry();
      measurement_ray_camera_history = final_ray_camera_history;
      measurement_ray_camera_frame_path = final_ray_camera_frame_path;
      minimum_live_ray_camera_histories = final_ray_camera_history.live_history_count;
      maximum_live_ray_camera_histories = final_ray_camera_history.live_history_count;
      measurement_descriptor_sets = final_descriptor_sets;
      minimum_live_descriptor_sets = final_descriptor_sets.live_count;
      maximum_live_descriptor_sets = final_descriptor_sets.live_count;
      measurement_ray_tracing_pipelines = final_ray_tracing_pipelines;
      minimum_live_ray_tracing_pipelines = final_ray_tracing_pipelines.live_pipeline_count;
      maximum_live_ray_tracing_pipelines = final_ray_tracing_pipelines.live_pipeline_count;
      minimum_live_shader_binding_tables = final_ray_tracing_pipelines.live_shader_binding_table_count;
      maximum_live_shader_binding_tables = final_ray_tracing_pipelines.live_shader_binding_table_count;
      memory_telemetry_seconds = 0.0;
      measurement_start_time = std::chrono::steady_clock::now();
    }
    if (capture_frame_count >= max_capture_frames) {
      throw std::runtime_error(wait_for_ray_accumulation
                                   ? "Demo preview capture timed out before accumulating requested ray-tracing frames."
                                   : "Demo preview capture timed out.");
    }
  }
  Platform::WaitForFrameSubmissions("Capture Completion Fence Wait");
  const auto effective_timing_warmup_frames = std::min(timing_warmup_frames, capture_frame_count);
  const auto measured_frame_count = capture_frame_count - effective_timing_warmup_frames;
  const auto capture_tlas_upload = render_layer->GetTlasUploadTelemetry().DeltaFrom(capture_tlas_upload_baseline);
  const auto capture_elapsed_seconds =
      std::max(0.0, std::chrono::duration<double>(std::chrono::steady_clock::now() - measurement_start_time).count() -
                        memory_telemetry_seconds);
  const auto capture_frames_per_second =
      capture_elapsed_seconds > 0.0 ? static_cast<double>(measured_frame_count) / capture_elapsed_seconds : 0.0;
  std::cout << "Demo preview capture timing: output=\"" << output_path.string()
            << "\" render_mode=" << Camera::GetCameraRenderModeName(resolved_render_mode) << " ser_mode="
            << Camera::GetShaderExecutionReorderingModeName(
                   scene_camera->camera_settings.shader_execution_reordering_mode)
            << " requested_frames=" << warmup_frames << " timing_warmup_frames=" << effective_timing_warmup_frames
            << " measured_frames=" << measured_frame_count << " rendered_frames=" << capture_frame_count
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
  metrics["schema"] = 2;
  metrics["type"] = "evoengine_ray_capture";
  metrics["renderer"] = "EvoEngine";
  metrics["demo_profile"] = demo_profile_id ? GetDemoProfileIdName(*demo_profile_id) : "";
  metrics["render_mode"] = Camera::GetCameraRenderModeName(resolved_render_mode);
  metrics["ray_debug_view"] = Camera::GetRayDebugViewName(scene_camera->camera_settings.ray_debug_view);
  metrics["output_path"] = output_path.string();
  metrics["output_format"] = linear_hdr_output ? "radiance_hdr_linear" : "png_display";
  metrics["width"] = width;
  metrics["height"] = height;
  metrics["requested_frames"] = warmup_frames;
  metrics["timing_warmup_frames"] = effective_timing_warmup_frames;
  metrics["measured_frames"] = measured_frame_count;
  metrics["rendered_frames"] = capture_frame_count;
  metrics["camera_frames"] = scene_camera->GetFrameCount();
  metrics["samples_per_frame"] = scene_camera->camera_settings.sample_size;
  const uint64_t effective_spp =
      static_cast<uint64_t>(scene_camera->GetFrameCount()) * scene_camera->camera_settings.sample_size;
  metrics["effective_spp"] = effective_spp;
  const uint64_t measured_spp = static_cast<uint64_t>(measured_frame_count) * scene_camera->camera_settings.sample_size;
  metrics["measured_spp"] = measured_spp;
  metrics["bounce_depth"] = scene_camera->camera_settings.bounce;
  metrics["firefly_clamp_enabled"] = scene_camera->camera_settings.firefly_clamp_enabled;
  metrics["firefly_clamp_threshold"] = scene_camera->camera_settings.firefly_clamp_threshold;
  metrics["emissive_triangle_nee_enabled"] = scene_camera->camera_settings.emissive_triangle_nee_enabled;
  metrics["auto_spp_enabled"] = scene_camera->camera_settings.auto_spp_enabled;
  metrics["m10_ray_transport"] = preview_m10_ray_transport;
  metrics["csm_caster_fixture"] = preview_shadow_caster_fixture;
  metrics["strand_fixture"] = preview_strand_fixture;
  metrics["strand_validation"] =
      preview_strand_fixture ? StrandFixtureTelemetryJson(render_layer, ApplicationContext::Get().GetActiveScene())
                             : nullptr;
  metrics["post_processing_stress"] = post_processing_stress;
  if (preview_m10_ray_transport) {
    metrics["m10_fixture_version"] = "isolated-v2";
  } else {
    metrics["m10_fixture_version"] = nullptr;
  }
  metrics["temporal_motion_capture"] = temporal_motion_capture;
  metrics["camera_position_override"] = nullptr;
  metrics["camera_look_at_override"] = nullptr;
  if (resolved_camera_position && resolved_camera_look_at) {
    metrics["camera_position_override"] = {resolved_camera_position->x, resolved_camera_position->y,
                                           resolved_camera_position->z};
    metrics["camera_look_at_override"] = {resolved_camera_look_at->x, resolved_camera_look_at->y,
                                          resolved_camera_look_at->z};
  }
  const auto camera_position = editor_layer->GetSceneCameraPosition();
  const auto camera_front = editor_layer->GetSceneCameraRotation() * glm::vec3(0.0f, 0.0f, -1.0f);
  const auto camera_look_at = camera_position + camera_front;
  metrics["camera_position"] = {camera_position.x, camera_position.y, camera_position.z};
  metrics["camera_look_at"] = {camera_look_at.x, camera_look_at.y, camera_look_at.z};
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
                             {"shader_execution_reordering", capabilities.support_shader_execution_reordering},
                             {"pipeline_creation_feedback", capabilities.support_pipeline_creation_feedback}};
  const auto pipeline_cache = Platform::GetPipelineCacheStats();
  metrics["pipeline_cache"] = {
      {"initialized", pipeline_cache.initialized},
      {"path", pipeline_cache.path},
      {"load_source", pipeline_cache.load_source},
      {"initial_bytes", pipeline_cache.initial_bytes},
      {"persisted_bytes", pipeline_cache.persisted_bytes},
      {"creation_count", pipeline_cache.creation_count},
      {"creation_failures", pipeline_cache.creation_failures},
      {"valid_feedback_count", pipeline_cache.valid_feedback_count},
      {"application_cache_hit_count", pipeline_cache.application_cache_hit_count},
      {"deferred_creation_count", pipeline_cache.deferred_creation_count},
      {"synchronous_fallback_count", pipeline_cache.synchronous_fallback_count},
      {"feedback_supported", pipeline_cache.feedback_supported},
      {"deferred_host_operations_supported", pipeline_cache.deferred_host_operations_supported}};
  metrics["query_only"] =
      resolved_render_mode == Camera::CameraRenderMode::RayQuery && !capabilities.support_ray_tracing;
  metrics["active_ray_backend"] = nullptr;
  if (resolved_render_mode == Camera::CameraRenderMode::RayTracing) {
    metrics["active_ray_backend"] = "ray-tracing-pipeline";
  } else if (resolved_render_mode == Camera::CameraRenderMode::RayQuery) {
    metrics["active_ray_backend"] = "ray-query-compute";
  }
  metrics["ray_pipeline_max_recursion_depth"] = nullptr;
  metrics["device_ray_tracing_max_recursion_depth"] = nullptr;
  if (capabilities.support_ray_tracing) {
    metrics["device_ray_tracing_max_recursion_depth"] =
        Platform::GetSelectedPhysicalDevice()->ray_tracing_properties_ext.maxRayRecursionDepth;
    if (resolved_render_mode == Camera::CameraRenderMode::RayTracing) {
      if (const auto recursion_depth = render_layer->GetRayTracingCameraMaxRecursionDepth()) {
        metrics["ray_pipeline_max_recursion_depth"] = *recursion_depth;
      }
    }
  }
  metrics["ray_shader_variant"] = nullptr;
  if (Camera::IsRayCameraRenderMode(resolved_render_mode)) {
    const auto technique = resolved_render_mode == Camera::CameraRenderMode::RayQuery
                               ? RayCameraShaderTechnique::RayQuery
                               : RayCameraShaderTechnique::RayTracing;
    const auto variant = render_layer->GetRayCameraShaderVariantStats(technique);
    metrics["ray_shader_variant"] = {
        {"technique", technique == RayCameraShaderTechnique::RayQuery ? "RayQuery" : "RayTracing"},
        {"selection_mode", force_full_ray_shader_variant ? "full" : "auto"},
        {"requested_mask", variant.requested_mask},
        {"active_mask", variant.active_mask},
        {"requested_key", variant.requested_key},
        {"active_key", variant.active_key},
        {"cache_source", variant.cache_source},
        {"pending", variant.pending},
        {"ready", variant.ready},
        {"fallback_active", variant.fallback_active},
        {"failed", variant.failed},
        {"last_error", variant.last_error},
        {"build_count", variant.build_count},
        {"activation_count", variant.activation_count},
        {"accumulation_reset_count", variant.accumulation_reset_count},
        {"fallback_frame_count", variant.fallback_frame_count},
        {"eviction_count", variant.eviction_count},
        {"resident_variant_count", variant.resident_variant_count},
        {"pending_build_count", variant.pending_build_count},
        {"failed_entry_count", variant.failed_entry_count},
        {"retained_submission_count", variant.retained_submission_count},
        {"variant_capacity", variant.variant_capacity},
        {"build_ms", variant.build_milliseconds},
        {"request_to_ready_ms", variant.request_to_ready_milliseconds},
        {"fallback_build_ms", variant.fallback_build_milliseconds},
        {"shader_cache",
         {{"memory_hits", variant.shader_cache.memory_hits},
          {"disk_hits", variant.shader_cache.disk_hits},
          {"disk_misses", variant.shader_cache.disk_misses},
          {"compilations", variant.shader_cache.compilations},
          {"coalesced_waits", variant.shader_cache.coalesced_waits},
          {"corrupt_entries", variant.shader_cache.corrupt_entries},
          {"failures", variant.shader_cache.failures}}},
        {"pipeline_creation",
         {{"result", variant.pipeline_creation.result},
          {"feedback_supported", variant.pipeline_creation.feedback_supported},
          {"feedback_valid", variant.pipeline_creation.feedback_valid},
          {"application_cache_hit", variant.pipeline_creation.application_cache_hit},
          {"duration_ns", variant.pipeline_creation.duration_nanoseconds},
          {"wall_ms", variant.pipeline_creation.wall_milliseconds},
          {"deferred_requested", variant.pipeline_creation.deferred_requested},
          {"deferred_used", variant.pipeline_creation.deferred_used},
          {"synchronous_fallback", variant.pipeline_creation.synchronous_fallback},
          {"fallback_reason", variant.pipeline_creation.fallback_reason}}}};
  }
  metrics["deterministic"] = deterministic_capture;
  metrics["accumulation_wall_seconds"] = capture_elapsed_seconds;
  metrics["memory_telemetry_seconds_excluded"] = memory_telemetry_seconds;
  metrics["frames_per_second"] = capture_frames_per_second;
  const auto effective_samples =
      static_cast<double>(width) * static_cast<double>(height) * static_cast<double>(measured_spp);
  metrics["wall_throughput_msamples_per_second"] =
      capture_elapsed_seconds > 0.0 ? effective_samples / capture_elapsed_seconds / 1.0e6 : 0.0;
  metrics["gpu_timestamps_available"] = Platform::GpuTimestampCaptureAvailable();
  const auto& physical_device = Platform::GetSelectedPhysicalDevice()->properties;
  metrics["gpu"] = {{"name", physical_device.deviceName},
                    {"vendor_id", physical_device.vendorID},
                    {"device_id", physical_device.deviceID},
                    {"driver_version", physical_device.driverVersion},
                    {"api_version", physical_device.apiVersion}};
  metrics["startup_gpu_sections"] = TimingStatsJson(startup_gpu_timestamp_stats);
  metrics["gpu_sections"] = TimingStatsJson(Platform::GetGpuTimestampStats());
  metrics["directional_shadow"] = DirectionalShadowTelemetryJson(render_layer, scene_camera);
  metrics["startup_cpu_sections"] = TimingStatsJson(startup_cpu_timing_stats);
  const auto capture_cpu_timing_stats = Platform::GetCpuTimingStats();
  metrics["cpu_sections"] = TimingStatsJson(capture_cpu_timing_stats);
  metrics["synchronization_waits"] = SynchronizationWaitsJson(capture_cpu_timing_stats);
  metrics["frame_synchronization"] = {
      {"policy", "wait-on-frame-slot-reuse"},
      {"max_frames_in_flight", Platform::GetMaxFramesInFlight()},
      {"pending_submissions_after_capture_flush", Platform::GetPendingFrameSubmissionCount()},
      {"validation_layers_enabled", Platform::ValidationLayersEnabled()}};
  metrics["startup_tlas_upload"] = TlasUploadTelemetryJson(startup_tlas_upload);
  metrics["tlas_upload"] = TlasUploadTelemetryJson(capture_tlas_upload);
  const auto blas_builder = BottomLevelAccelerationStructure::GetStaticBuildTelemetry();
  auto blas_passes = nlohmann::ordered_json::array();
  for (const auto& pass : blas_builder.passes) {
    blas_passes.push_back({{"begin", pass.begin},
                           {"count", pass.count},
                           {"destination_bytes", pass.destination_size},
                           {"scratch_bytes", pass.scratch_size},
                           {"scratch_wave_count", pass.scratch_wave_count},
                           {"oversized_singleton", pass.oversized_singleton}});
  }
  const auto compact_ratio = blas_builder.eligible_static_uncompacted_bytes == 0
                                 ? 0.0
                                 : static_cast<double>(blas_builder.eligible_static_compacted_bytes) /
                                       static_cast<double>(blas_builder.eligible_static_uncompacted_bytes);
  metrics["blas_builder"] = {
      {"fixed_hint_bytes", blas_builder.fixed_hint_bytes},
      {"complete", blas_builder.complete},
      {"pending_count", blas_builder.pending_count},
      {"total_blas_count", blas_builder.total_blas_count},
      {"static_eligible_count", blas_builder.static_eligible_count},
      {"updateable_count", blas_builder.updateable_count},
      {"shared_input_count", blas_builder.shared_input_count},
      {"private_input_count", blas_builder.private_input_count},
      {"private_input_bytes", blas_builder.private_input_bytes},
      {"cumulative_built_static_count", blas_builder.cumulative_built_static_count},
      {"cumulative_uncompacted_bytes", blas_builder.cumulative_uncompacted_bytes},
      {"cumulative_compacted_bytes", blas_builder.cumulative_compacted_bytes},
      {"pass_count", blas_builder.pass_count},
      {"scratch_wave_count", blas_builder.scratch_wave_count},
      {"scratch_peak_bytes", blas_builder.scratch_peak_bytes},
      {"eligible_static_uncompacted_bytes", blas_builder.eligible_static_uncompacted_bytes},
      {"eligible_static_compacted_bytes", blas_builder.eligible_static_compacted_bytes},
      {"eligible_static_compaction_ratio", compact_ratio},
      {"final_compacted_storage_bytes", blas_builder.final_compacted_storage_bytes},
      {"transient_peak_bytes", blas_builder.transient_peak_bytes},
      {"transient_scope", "builder scratch plus live original and compacted static BLAS allocations"},
      {"wall_milliseconds", blas_builder.wall_milliseconds},
      {"passes", std::move(blas_passes)}};
  metrics["ray_camera_history"] = {
      {"ownership", "camera-owned-single-slot"},
      {"maximum_histories_per_camera", 1},
      {"bytes_per_pixel_per_history", sizeof(glm::vec4) * 2u},
      {"startup", RayCameraHistoryStatsJson(startup_ray_camera_history)},
      {"measurement_baseline", RayCameraHistoryStatsJson(measurement_ray_camera_history)},
      {"final", RayCameraHistoryStatsJson(final_ray_camera_history)},
      {"capture_minimum_live_history_count", minimum_live_ray_camera_histories},
      {"capture_maximum_live_history_count", maximum_live_ray_camera_histories},
      {"capture_creation_count",
       final_ray_camera_history.creation_count >= measurement_ray_camera_history.creation_count
           ? final_ray_camera_history.creation_count - measurement_ray_camera_history.creation_count
           : 0u}};
  const auto& measurement_plan_cache = measurement_ray_camera_frame_path.render_graph_plan_cache;
  const auto& final_plan_cache = final_ray_camera_frame_path.render_graph_plan_cache;
  metrics["ray_camera_frame_path"] = {
      {"startup", RayCameraFramePathStatsJson(startup_ray_camera_frame_path)},
      {"measurement_baseline", RayCameraFramePathStatsJson(measurement_ray_camera_frame_path)},
      {"final", RayCameraFramePathStatsJson(final_ray_camera_frame_path)},
      {"capture",
       {{"plan_cache_hits", final_plan_cache.hit_count - measurement_plan_cache.hit_count},
        {"plan_cache_misses", final_plan_cache.miss_count - measurement_plan_cache.miss_count},
        {"plan_compilations", final_plan_cache.compilation_count - measurement_plan_cache.compilation_count},
        {"plan_evictions", final_plan_cache.eviction_count - measurement_plan_cache.eviction_count},
        {"plan_compilation_ms",
         final_plan_cache.compilation_milliseconds - measurement_plan_cache.compilation_milliseconds},
        {"output_descriptor_creations", final_ray_camera_frame_path.output_descriptor_creation_count -
                                            measurement_ray_camera_frame_path.output_descriptor_creation_count},
        {"output_descriptor_reuses", final_ray_camera_frame_path.output_descriptor_reuse_count -
                                         measurement_ray_camera_frame_path.output_descriptor_reuse_count}}}};
  metrics["resource_lifetime"] = {
      {"descriptor_sets",
       {{"startup", DescriptorSetLifetimeStatsJson(startup_descriptor_sets)},
        {"measurement_baseline", DescriptorSetLifetimeStatsJson(measurement_descriptor_sets)},
        {"final", DescriptorSetLifetimeStatsJson(final_descriptor_sets)},
        {"capture_minimum_live_count", minimum_live_descriptor_sets},
        {"capture_maximum_live_count", maximum_live_descriptor_sets},
        {"capture_creation_count", final_descriptor_sets.creation_count - measurement_descriptor_sets.creation_count}}},
      {"ray_tracing_pipelines",
       {{"startup", RayTracingPipelineLifetimeStatsJson(startup_ray_tracing_pipelines)},
        {"measurement_baseline", RayTracingPipelineLifetimeStatsJson(measurement_ray_tracing_pipelines)},
        {"final", RayTracingPipelineLifetimeStatsJson(final_ray_tracing_pipelines)},
        {"capture_minimum_live_pipeline_count", minimum_live_ray_tracing_pipelines},
        {"capture_maximum_live_pipeline_count", maximum_live_ray_tracing_pipelines},
        {"capture_minimum_live_shader_binding_table_count", minimum_live_shader_binding_tables},
        {"capture_maximum_live_shader_binding_table_count", maximum_live_shader_binding_tables},
        {"capture_pipeline_creation_count", final_ray_tracing_pipelines.pipeline_creation_count -
                                                measurement_ray_tracing_pipelines.pipeline_creation_count},
        {"capture_shader_binding_table_creation_count",
         final_ray_tracing_pipelines.shader_binding_table_creation_count -
             measurement_ray_tracing_pipelines.shader_binding_table_creation_count}}}};
  metrics["gpu_memory"] = {{"scope", "startup-ready plus capture-window samples; pre-ready transient peaks excluded"},
                           {"startup_ready", GpuMemorySnapshotJson(startup_gpu_memory)},
                           {"peak", GpuMemorySnapshotJson(peak_gpu_memory)},
                           {"final", GpuMemorySnapshotJson(final_gpu_memory)}};
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
        application_info.enable_gpu_timestamp_capture = automated_capture;
        ApplicationContext::Get().Initialize(application_info);
        initialized = true;
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
              command_line.preview_capture_warmup_frames, command_line.preview_capture_timing_warmup_frames,
              command_line.demo_profile_id, command_line.preview_capture_render_mode,
              command_line.preview_capture_ray_debug_view, command_line.preview_capture_ser_mode,
              command_line.preview_capture_firefly_clamp_enabled, command_line.preview_capture_firefly_clamp_threshold,
              command_line.preview_capture_emissive_triangle_nee_enabled,
              command_line.preview_force_full_ray_shader_variant, command_line.preview_capture_auto_spp_enabled,
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
              command_line.preview_shadow_distance_fade, command_line.preview_shadow_fit_mode,
              command_line.preview_shadow_pcf_samples, command_line.preview_shadow_debug_mode,
              command_line.preview_shadow_debug_cascade, command_line.preview_shadow_debug_light,
              command_line.preview_shadow_light_count, command_line.preview_shadow_caster_fixture,
              command_line.preview_strand_fixture, command_line.preview_capture_deterministic,
              command_line.preview_capture_bistro_ddgi, command_line.preview_capture_m10_ray_transport,
              command_line.preview_post_processing_stress);
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
