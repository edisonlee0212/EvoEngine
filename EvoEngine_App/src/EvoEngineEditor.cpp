#include "AppBootstrap.hpp"
#include "Application.hpp"
#include "AssetManager.hpp"
#include "Camera.hpp"
#include "DemoProfiles.hpp"
#include "DemoScene.hpp"
#include "EditorLayer.hpp"
#include "EnvironmentalLighting.hpp"
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

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <vector>

#include <glm/gtc/packing.hpp>

#ifdef EVOENGINE_WINDOWS
#  ifndef NOMINMAX
#    define NOMINMAX
#  endif
#  include <Windows.h>
#endif

using namespace evo_engine;

namespace {
// DDGI_VALIDATION_CAPTURE_PROTOCOL_BEGIN
struct EditorCommandLine {
  std::optional<std::filesystem::path> project_path;
  std::optional<DemoProfileId> demo_profile_id;
  std::optional<std::filesystem::path> demo_preview_capture_path;
  std::optional<GraphicsInitializationSettings::ShadowMapResolutionQuality> shadow_map_resolution_quality;
  ApplicationMode application_mode = ApplicationMode::Editor;
  bool application_mode_explicit = false;
  int preview_capture_width = 1280;
  int preview_capture_height = 720;
  size_t preview_capture_warmup_frames = 8;
  std::optional<Camera::CameraRenderMode> preview_capture_render_mode;
  std::optional<int> preview_capture_ray_bounces;
  std::optional<CameraSettings::RayDebugView> preview_capture_ray_debug_view;
  std::optional<CameraSettings::RayOutputSettings> preview_capture_ray_outputs;
  std::optional<CameraSettings::ShaderExecutionReorderingMode> preview_capture_ser_mode;
  std::optional<float> preview_capture_firefly_clamp_threshold;
  std::optional<bool> preview_capture_auto_spp_enabled;
  std::optional<int> preview_capture_auto_spp_min_samples;
  std::optional<int> preview_capture_auto_spp_max_samples;
  std::optional<float> preview_capture_auto_spp_convergence_threshold;
  std::optional<int> preview_capture_sample_size;
  std::optional<std::filesystem::path> preview_ray_profile_report_path;
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
  bool preview_strand_fixture = false;
  bool preview_strand_punctual_fixture = false;
  bool preview_strand_gizmo_fixture = false;
  bool preview_capture_deterministic = false;
  bool preview_capture_bistro_ddgi = false;
  std::optional<std::string> preview_ddgi_fixture;
  std::optional<std::filesystem::path> preview_ddgi_report_path;
  uint32_t preview_ddgi_seed = 0x6d2b79f5u;
  std::optional<int> preview_ddgi_uniform_ray_count;
  std::optional<int> preview_ddgi_emissive_ray_count;
  size_t preview_ddgi_measure_frames = 120;
  size_t preview_ddgi_response_frames = 0;
  bool preview_ddgi_continuous_updates = false;
  bool preview_ddgi_disabled = false;
  bool preview_ddgi_reference = false;
  std::string preview_ddgi_phase = "ad-hoc";
  size_t preview_ddgi_run_index = 0;
  bool bistro_smoke = false;
};

std::string NormalizePreviewChoice(std::string value) {
  value.erase(std::remove_if(value.begin(), value.end(),
                             [](const char character) {
                               return character == '-' || character == '_' || character == '/' ||
                                      std::isspace(static_cast<unsigned char>(character));
                             }),
              value.end());
  std::transform(value.begin(), value.end(), value.begin(), [](const char character) {
    return static_cast<char>(std::tolower(static_cast<unsigned char>(character)));
  });
  return value;
}

Camera::CameraRenderMode ParsePreviewRenderMode(const std::string& value) {
  const auto normalized = NormalizePreviewChoice(value);
  if (normalized == "raster" || normalized == "rasterization") {
    return Camera::CameraRenderMode::Rasterization;
  }
  if (normalized == "raytracing" || normalized == "pathtracing" || normalized == "pathtrace") {
    return Camera::CameraRenderMode::RayTracing;
  }
  if (normalized == "rayquery") {
    return Camera::CameraRenderMode::RayQuery;
  }
  throw std::invalid_argument("Unknown preview render mode: " + value);
}

CameraSettings::ShaderExecutionReorderingMode ParsePreviewShaderExecutionReorderingMode(const std::string& value) {
  const auto normalized = NormalizePreviewChoice(value);
  if (normalized == "off" || normalized == "disabled" || normalized == "disable") {
    return CameraSettings::ShaderExecutionReorderingMode::Disabled;
  }
  if (normalized == "auto" || normalized == "automatic") {
    return CameraSettings::ShaderExecutionReorderingMode::Automatic;
  }
  if (normalized == "on" || normalized == "enabled" || normalized == "enable") {
    return CameraSettings::ShaderExecutionReorderingMode::Enabled;
  }
  throw std::invalid_argument("Unknown preview SER mode: " + value);
}

CameraSettings::RayDebugView ParsePreviewRayDebugView(const std::string& value) {
  const auto normalized = NormalizePreviewChoice(value);
  static const std::vector<std::string> names{
      "beauty",           "materialid", "basecolor",      "geometricnormal",   "shadingnormal",
      "roughness",        "metallic",   "specularf0",     "alphacoverage",     "transmission",
      "iridescence",      "emission",   "directpunctual", "directenvironment", "directemissive",
      "indirectradiance", "pathdepth",  "bsdfpdf",        "lightpdf",          "emissivepdf"};
  const auto match = std::find(names.begin(), names.end(), normalized);
  if (match != names.end()) {
    return static_cast<CameraSettings::RayDebugView>(std::distance(names.begin(), match));
  }
  if (normalized == "none" || normalized == "off" || normalized == "disabled") {
    return CameraSettings::RayDebugView::Beauty;
  }
  if (normalized == "material") {
    return CameraSettings::RayDebugView::MaterialId;
  }
  if (normalized == "alpha" || normalized == "opacity" || normalized == "coverage") {
    return CameraSettings::RayDebugView::AlphaCoverage;
  }
  if (normalized == "f0") {
    return CameraSettings::RayDebugView::SpecularF0;
  }
  throw std::invalid_argument("Unknown preview ray debug view: " + value);
}

CameraSettings::RayOutputSettings ParsePreviewRayOutputs(const std::string& value) {
  CameraSettings::RayOutputSettings outputs;
  std::stringstream stream(value);
  std::string token;
  while (std::getline(stream, token, ',')) {
    const auto normalized = NormalizePreviewChoice(token);
    if (normalized.empty() || normalized == "none" || normalized == "off" || normalized == "disabled") {
      continue;
    }
    if (normalized == "all") {
      outputs.albedo = true;
      outputs.normal = true;
      outputs.ray_count = true;
      outputs.path_length = true;
      outputs.time = true;
      outputs.debug = true;
      continue;
    }
    if (normalized == "albedo") {
      outputs.albedo = true;
    } else if (normalized == "normal" || normalized == "normals") {
      outputs.normal = true;
    } else if (normalized == "raycount") {
      outputs.ray_count = true;
    } else if (normalized == "pathlength") {
      outputs.path_length = true;
    } else if (normalized == "time") {
      outputs.time = true;
    } else if (normalized == "debug") {
      outputs.debug = true;
    } else {
      throw std::invalid_argument("Unknown preview ray output: " + token);
    }
  }
  return outputs;
}

const char* GetPreviewRayOutputName(const RayCameraOptionalOutput output) {
  switch (output) {
    case RayCameraOptionalOutput::Albedo:
      return "albedo";
    case RayCameraOptionalOutput::Normal:
      return "normal";
    case RayCameraOptionalOutput::RayCount:
      return "ray-count";
    case RayCameraOptionalOutput::PathLength:
      return "path-length";
    case RayCameraOptionalOutput::Time:
      return "time";
    case RayCameraOptionalOutput::Debug:
      return "debug";
    case RayCameraOptionalOutput::Count:
      break;
  }
  return "unknown";
}

uint32_t PreviewRayOutputBit(const RayCameraOptionalOutput output) {
  return 1u << static_cast<uint32_t>(output);
}

bool PreviewRayOutputIsScalar(const RayCameraOptionalOutput output) {
  return output == RayCameraOptionalOutput::RayCount || output == RayCameraOptionalOutput::PathLength ||
         output == RayCameraOptionalOutput::Time;
}

struct PreviewRayOutputSummary {
  const char* name = "";
  bool scalar = false;
  uint32_t width = 0;
  uint32_t height = 0;
  uint64_t value_count = 0;
  uint64_t finite_value_count = 0;
  uint64_t nonzero_value_count = 0;
  double minimum = 0.0;
  double maximum = 0.0;
  double absolute_sum = 0.0;
};

void AccumulatePreviewRayOutputValue(PreviewRayOutputSummary& summary, const double value) {
  if (!std::isfinite(value)) {
    return;
  }
  ++summary.finite_value_count;
  if (value != 0.0) {
    ++summary.nonzero_value_count;
  }
  summary.minimum = std::min(summary.minimum, value);
  summary.maximum = std::max(summary.maximum, value);
  summary.absolute_sum += std::abs(value);
}

PreviewRayOutputSummary SummarizePreviewRayOutput(const RayCameraOptionalOutput output, Image& image) {
  PreviewRayOutputSummary summary;
  summary.name = GetPreviewRayOutputName(output);
  summary.scalar = PreviewRayOutputIsScalar(output);
  summary.width = image.GetExtent().width;
  summary.height = image.GetExtent().height;
  summary.minimum = std::numeric_limits<double>::infinity();
  summary.maximum = -std::numeric_limits<double>::infinity();
  const auto pixel_count = static_cast<size_t>(summary.width) * summary.height;
  if (image.GetFormat() == VK_FORMAT_R32_UINT) {
    std::vector<uint32_t> pixels;
    Buffer image_buffer(sizeof(uint32_t) * pixel_count);
    image_buffer.CopyFromImage(image, sizeof(uint32_t));
    image_buffer.DownloadVector(pixels, pixel_count);
    summary.value_count = pixels.size();
    for (const uint32_t value : pixels) {
      AccumulatePreviewRayOutputValue(summary, static_cast<double>(value));
    }
  } else if (image.GetFormat() == VK_FORMAT_R8G8B8A8_UNORM) {
    std::vector<uint8_t> pixels;
    Buffer image_buffer(4u * pixel_count);
    image_buffer.CopyFromImage(image, 4u);
    image_buffer.DownloadVector(pixels, 4u * pixel_count);
    summary.value_count = pixels.size();
    for (const uint8_t value : pixels) {
      AccumulatePreviewRayOutputValue(summary, static_cast<double>(value) / 255.0);
    }
  } else if (image.GetFormat() == VK_FORMAT_R16G16B16A16_SFLOAT) {
    std::vector<uint16_t> pixels;
    Buffer image_buffer(8u * pixel_count);
    image_buffer.CopyFromImage(image, 8u);
    image_buffer.DownloadVector(pixels, 4u * pixel_count);
    summary.value_count = pixels.size();
    for (const uint16_t value : pixels) {
      AccumulatePreviewRayOutputValue(summary, glm::unpackHalf1x16(value));
    }
  } else if (image.GetFormat() == VK_FORMAT_R32G32B32A32_SFLOAT) {
    std::vector<glm::vec4> pixels;
    Buffer image_buffer(sizeof(glm::vec4) * pixel_count);
    image_buffer.CopyFromImage(image, sizeof(glm::vec4));
    image_buffer.DownloadVector(pixels, pixel_count);
    summary.value_count = pixels.size() * 4u;
    for (const auto& pixel : pixels) {
      AccumulatePreviewRayOutputValue(summary, pixel.x);
      AccumulatePreviewRayOutputValue(summary, pixel.y);
      AccumulatePreviewRayOutputValue(summary, pixel.z);
      AccumulatePreviewRayOutputValue(summary, pixel.w);
    }
  } else {
    throw std::runtime_error(std::string("Preview ray output has unsupported format: ") + summary.name);
  }
  if (summary.finite_value_count == 0u) {
    summary.minimum = 0.0;
    summary.maximum = 0.0;
  }
  return summary;
}

std::filesystem::path PreviewRayOutputReportPath(std::filesystem::path output_path) {
  output_path.replace_extension(".ray-outputs.json");
  return output_path;
}

void WritePreviewRayOutputReport(const std::filesystem::path& output_path, Camera& camera) {
  const auto& resources = camera.GetRayCameraOptionalOutputResources();
  if (resources.enabled_mask == 0u) {
    return;
  }
  std::vector<PreviewRayOutputSummary> summaries;
  summaries.reserve(kRayCameraOptionalOutputCount);
  for (uint32_t index = 0u; index < kRayCameraOptionalOutputCount; ++index) {
    const auto output = static_cast<RayCameraOptionalOutput>(index);
    if ((resources.enabled_mask & PreviewRayOutputBit(output)) == 0u) {
      continue;
    }
    const auto& image = resources.images[index];
    if (!image) {
      throw std::runtime_error(std::string("Enabled preview ray output is missing an image: ") +
                               GetPreviewRayOutputName(output));
    }
    summaries.emplace_back(SummarizePreviewRayOutput(output, *image));
    const auto& summary = summaries.back();
    if (summary.width == 0u || summary.height == 0u || summary.value_count == 0u ||
        summary.finite_value_count != summary.value_count) {
      throw std::runtime_error(std::string("Preview ray output readback is invalid: ") + summary.name);
    }
  }
  const auto report_path = PreviewRayOutputReportPath(output_path);
  if (const auto parent_path = report_path.parent_path(); !parent_path.empty()) {
    std::filesystem::create_directories(parent_path);
  }
  std::ofstream output(report_path);
  output << "{\n";
  output << "  \"enabled_mask\": " << resources.enabled_mask << ",\n";
  output << "  \"outputs\": [\n";
  for (size_t index = 0; index < summaries.size(); ++index) {
    const auto& summary = summaries[index];
    output << "    {\"name\": \"" << summary.name << "\", \"scalar\": " << (summary.scalar ? "true" : "false")
           << ", \"width\": " << summary.width << ", \"height\": " << summary.height
           << ", \"value_count\": " << summary.value_count << ", \"finite_value_count\": " << summary.finite_value_count
           << ", \"nonzero_value_count\": " << summary.nonzero_value_count << ", \"minimum\": " << summary.minimum
           << ", \"maximum\": " << summary.maximum << ", \"absolute_sum\": " << summary.absolute_sum << "}"
           << (index + 1 == summaries.size() ? "\n" : ",\n");
  }
  output << "  ]\n";
  output << "}\n";
  if (!output) {
    throw std::runtime_error("Failed to write preview ray output report: " + report_path.string());
  }
  std::cout << "EVOENGINE_RAY_OUTPUT_REPORT path=\"" << report_path.string() << "\" outputs=" << summaries.size()
            << " enabled_mask=" << resources.enabled_mask << std::endl;
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
    } else if (argument == "--preview-bounces") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument("--preview-bounces requires a non-negative integer.");
      }
      command_line.preview_capture_ray_bounces = std::max(0, std::stoi(argv[++arg_index]));
    } else if (argument == "--preview-ray-debug") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument(argument + " requires a ray debug view.");
      }
      command_line.preview_capture_ray_debug_view = ParsePreviewRayDebugView(argv[++arg_index] ? argv[arg_index] : "");
    } else if (argument == "--preview-ray-outputs") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument(argument + " requires a comma-separated ray output list or all.");
      }
      command_line.preview_capture_ray_outputs = ParsePreviewRayOutputs(argv[++arg_index] ? argv[arg_index] : "");
    } else if (argument == "--preview-ser") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument("--preview-ser requires disabled, automatic, or enabled.");
      }
      command_line.preview_capture_ser_mode =
          ParsePreviewShaderExecutionReorderingMode(argv[++arg_index] ? argv[arg_index] : "");
    } else if (argument == "--preview-firefly-clamp-threshold") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument("--preview-firefly-clamp-threshold requires a non-negative number.");
      }
      command_line.preview_capture_firefly_clamp_threshold = std::max(0.0f, std::stof(argv[++arg_index]));
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
    } else if (argument == "--preview-ray-profile-report") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument("--preview-ray-profile-report requires an output JSON path.");
      }
      command_line.preview_ray_profile_report_path = std::filesystem::absolute(argv[++arg_index]);
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
    } else if (argument == "--preview-strand-fixture") {
      command_line.preview_strand_fixture = true;
    } else if (argument == "--preview-strand-punctual-fixture") {
      command_line.preview_strand_punctual_fixture = true;
    } else if (argument == "--preview-strand-gizmo-fixture") {
      command_line.preview_strand_gizmo_fixture = true;
    } else if (argument == "--shadow-map-resolution" || argument == "--shadow-resolution") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument(argument + " requires low, medium, high, or very-high.");
      }
      command_line.shadow_map_resolution_quality =
          ParseShadowMapResolutionQualityName(argv[++arg_index] ? argv[arg_index] : "");
    } else if (argument == "--preview-deterministic") {
      command_line.preview_capture_deterministic = true;
    } else if (argument == "--preview-ddgi-fixture") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument("--preview-ddgi-fixture requires a fixture id.");
      }
      command_line.preview_ddgi_fixture = argv[++arg_index] ? argv[arg_index] : "";
    } else if (argument == "--preview-ddgi-report") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument("--preview-ddgi-report requires an output JSON path.");
      }
      command_line.preview_ddgi_report_path = std::filesystem::absolute(argv[++arg_index]);
    } else if (argument == "--preview-ddgi-seed") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument("--preview-ddgi-seed requires an unsigned integer.");
      }
      command_line.preview_ddgi_seed = static_cast<uint32_t>(std::stoul(argv[++arg_index], nullptr, 0));
    } else if (argument == "--preview-ddgi-emissive-rays" || argument == "--preview-ddgi-guided-rays") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument("--preview-ddgi-emissive-rays requires a value between 0 and 4096.");
      }
      const int emissive_ray_count = std::stoi(argv[++arg_index]);
      if (emissive_ray_count < 0 || emissive_ray_count > 4096) {
        throw std::invalid_argument("--preview-ddgi-emissive-rays requires a value between 0 and 4096.");
      }
      command_line.preview_ddgi_emissive_ray_count = emissive_ray_count;
    } else if (argument == "--preview-ddgi-uniform-rays") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument("--preview-ddgi-uniform-rays requires a value between 1 and 4096.");
      }
      const int uniform_ray_count = std::stoi(argv[++arg_index]);
      if (uniform_ray_count < 1 || uniform_ray_count > 4096) {
        throw std::invalid_argument("--preview-ddgi-uniform-rays requires a value between 1 and 4096.");
      }
      command_line.preview_ddgi_uniform_ray_count = uniform_ray_count;
    } else if (argument == "--preview-ddgi-continuous-updates") {
      command_line.preview_ddgi_continuous_updates = true;
    } else if (argument == "--preview-ddgi-guided-emitters") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument("--preview-ddgi-guided-emitters requires a value between 1 and 8.");
      }
      const int legacy_emitter_limit = std::stoi(argv[++arg_index]);
      if (legacy_emitter_limit < 1 || legacy_emitter_limit > 8) {
        throw std::invalid_argument("--preview-ddgi-guided-emitters requires a value between 1 and 8.");
      }
    } else if (argument == "--preview-ddgi-measure-frames") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument("--preview-ddgi-measure-frames requires a positive integer.");
      }
      command_line.preview_ddgi_measure_frames = static_cast<size_t>(std::max(1, std::stoi(argv[++arg_index])));
    } else if (argument == "--preview-ddgi-response-frames") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument("--preview-ddgi-response-frames requires a positive integer.");
      }
      command_line.preview_ddgi_response_frames = static_cast<size_t>(std::max(1, std::stoi(argv[++arg_index])));
    } else if (argument == "--preview-ddgi-disabled") {
      command_line.preview_ddgi_disabled = true;
    } else if (argument == "--preview-ddgi-reference") {
      command_line.preview_ddgi_reference = true;
    } else if (argument == "--preview-ddgi-phase") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument("--preview-ddgi-phase requires calibration or holdout.");
      }
      command_line.preview_ddgi_phase = argv[++arg_index] ? argv[arg_index] : "";
      if (command_line.preview_ddgi_phase != "calibration" && command_line.preview_ddgi_phase != "holdout") {
        throw std::invalid_argument("--preview-ddgi-phase requires calibration or holdout.");
      }
    } else if (argument == "--preview-ddgi-run-index") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument("--preview-ddgi-run-index requires a non-negative integer.");
      }
      command_line.preview_ddgi_run_index = static_cast<size_t>(std::max(0, std::stoi(argv[++arg_index])));
    } else if (argument == "--preview-bistro-ddgi") {
      command_line.preview_capture_bistro_ddgi = true;
    } else if (argument == "--bistro-smoke") {
      command_line.bistro_smoke = true;
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
  if (command_line.preview_capture_ray_debug_view && !command_line.demo_preview_capture_path) {
    throw std::invalid_argument("--preview-ray-debug requires --capture-demo-preview.");
  }
  if (command_line.preview_capture_ray_outputs && !command_line.demo_preview_capture_path) {
    throw std::invalid_argument("--preview-ray-outputs requires --capture-demo-preview.");
  }
  if (command_line.preview_ray_profile_report_path &&
      (!command_line.demo_preview_capture_path ||
       command_line.preview_ray_profile_report_path->extension() != ".json" ||
       !command_line.preview_capture_render_mode ||
       !Camera::IsRayCameraRenderMode(*command_line.preview_capture_render_mode))) {
    throw std::invalid_argument(
        "--preview-ray-profile-report requires a JSON demo capture in raytracing or rayquery mode.");
  }
  if (command_line.preview_capture_ray_debug_view &&
      (!command_line.preview_capture_render_mode ||
       !Camera::IsRayCameraRenderMode(*command_line.preview_capture_render_mode))) {
    throw std::invalid_argument("--preview-ray-debug requires --preview-render-mode raytracing or rayquery.");
  }
  if (command_line.preview_capture_ray_outputs &&
      (!command_line.preview_capture_render_mode ||
       !Camera::IsRayCameraRenderMode(*command_line.preview_capture_render_mode))) {
    throw std::invalid_argument("--preview-ray-outputs requires --preview-render-mode raytracing or rayquery.");
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
  if (command_line.preview_ddgi_fixture) {
    const std::set<std::string> fixture_ids = {"cornell",
                                               "sponza",
                                               "furnace",
                                               "alpha-tested",
                                               "scrolling",
                                               "emissive-small",
                                               "emissive-small-equal-radiance",
                                               "emissive-small-equal-power",
                                               "emissive-large",
                                               "emissive-textured-uv0",
                                               "emissive-textured-uv1",
                                               "emissive-textured-uv2",
                                               "emissive-textured-uv3",
                                               "emissive-one-sided",
                                               "emissive-double-sided",
                                               "emissive-alpha-cutout",
                                               "emissive-multi",
                                               "emissive-moving-rigid",
                                               "emissive-enable",
                                               "emissive-disable",
                                               "emissive-enable-hdr",
                                               "geometry-moving",
                                               "analytic-light",
                                               "emissive-direct-hit",
                                               "emissive-empty"};
    if (fixture_ids.find(*command_line.preview_ddgi_fixture) == fixture_ids.end()) {
      throw std::invalid_argument("Unknown DDGI validation fixture: " + *command_line.preview_ddgi_fixture);
    }
    if (!command_line.demo_preview_capture_path) {
      throw std::invalid_argument("--preview-ddgi-fixture requires --capture-demo-preview.");
    }
    const auto required_profile =
        *command_line.preview_ddgi_fixture == "cornell" ? DemoProfileId::Ddgi : DemoProfileId::RenderingRegression;
    if (command_line.demo_profile_id != required_profile) {
      throw std::invalid_argument("DDGI validation fixture requires its canonical demo profile.");
    }
    const bool small_emitter_baseline_fixture = *command_line.preview_ddgi_fixture == "emissive-small" ||
                                                *command_line.preview_ddgi_fixture == "emissive-small-equal-radiance" ||
                                                *command_line.preview_ddgi_fixture == "emissive-small-equal-power" ||
                                                *command_line.preview_ddgi_fixture == "emissive-large";
    const bool temporal_response_fixture = *command_line.preview_ddgi_fixture == "emissive-enable" ||
                                           *command_line.preview_ddgi_fixture == "emissive-disable" ||
                                           *command_line.preview_ddgi_fixture == "emissive-enable-hdr" ||
                                           *command_line.preview_ddgi_fixture == "emissive-empty";
    const auto expected_image_extension = *command_line.preview_ddgi_fixture == "furnace" ||
                                                  *command_line.preview_ddgi_fixture == "analytic-light" ||
                                                  small_emitter_baseline_fixture || temporal_response_fixture
                                              ? ".hdr"
                                              : ".png";
    if (!command_line.preview_capture_deterministic || command_line.preview_capture_width != 1920 ||
        command_line.preview_capture_height != 1080 ||
        command_line.demo_preview_capture_path->extension() != expected_image_extension) {
      throw std::invalid_argument("DDGI validation requires its canonical deterministic 1920x1080 image format.");
    }
    if (command_line.preview_ddgi_reference) {
      if (command_line.preview_ddgi_report_path || command_line.preview_ddgi_disabled ||
          command_line.preview_capture_render_mode != Camera::CameraRenderMode::RayTracing ||
          command_line.preview_capture_ray_debug_view != CameraSettings::RayDebugView::Beauty ||
          command_line.preview_capture_ser_mode != CameraSettings::ShaderExecutionReorderingMode::Disabled ||
          command_line.preview_capture_auto_spp_enabled != false || command_line.preview_capture_sample_size != 4 ||
          command_line.preview_capture_warmup_frames != 64) {
        throw std::invalid_argument(
            "DDGI quality references require a fixed 64-frame, 4-SPP RT-pipeline beauty capture.");
      }
    } else if (!command_line.preview_ddgi_report_path ||
               command_line.preview_ddgi_report_path->extension() != ".json" ||
               command_line.preview_capture_render_mode != Camera::CameraRenderMode::Rasterization) {
      throw std::invalid_argument(
          "DDGI measurements require deterministic 1920x1080 rasterization image and JSON report outputs.");
    }
  } else if (command_line.preview_ddgi_report_path || command_line.preview_ddgi_disabled ||
             command_line.preview_ddgi_reference || command_line.preview_ddgi_phase != "ad-hoc" ||
             command_line.preview_ddgi_run_index != 0) {
    throw std::invalid_argument("DDGI validation options require --preview-ddgi-fixture.");
  }
  if (command_line.bistro_smoke && command_line.demo_profile_id != DemoProfileId::Bistro) {
    throw std::invalid_argument("--bistro-smoke requires --demo bistro.");
  }
  if (command_line.bistro_smoke && command_line.application_mode != ApplicationMode::Editor) {
    throw std::invalid_argument("--bistro-smoke requires --editor.");
  }
  if (command_line.bistro_smoke && command_line.demo_preview_capture_path) {
    throw std::invalid_argument("--bistro-smoke cannot be combined with --capture-demo-preview.");
  }
  if (command_line.bistro_smoke &&
      (command_line.preview_capture_width != 1920 || command_line.preview_capture_height != 1080)) {
    throw std::invalid_argument("--bistro-smoke requires --preview-width 1920 --preview-height 1080.");
  }
  if ((command_line.preview_shadow_fit_mode || command_line.preview_shadow_pcf_samples ||
       command_line.preview_shadow_debug_mode || command_line.preview_shadow_debug_cascade ||
       command_line.preview_shadow_debug_light) &&
      !command_line.demo_preview_capture_path) {
    throw std::invalid_argument("Preview shadow controls require --capture-demo-preview.");
  }
  if (command_line.preview_strand_fixture && !command_line.demo_preview_capture_path) {
    throw std::invalid_argument("--preview-strand-fixture requires --capture-demo-preview.");
  }
  if (command_line.preview_strand_fixture && command_line.demo_profile_id != DemoProfileId::RenderingRegression) {
    throw std::invalid_argument("--preview-strand-fixture requires --demo rendering-regression.");
  }
  if (command_line.preview_strand_punctual_fixture && !command_line.demo_preview_capture_path) {
    throw std::invalid_argument("--preview-strand-punctual-fixture requires --capture-demo-preview.");
  }
  if (command_line.preview_strand_punctual_fixture &&
      command_line.demo_profile_id != DemoProfileId::RenderingRegression) {
    throw std::invalid_argument("--preview-strand-punctual-fixture requires --demo rendering-regression.");
  }
  if (command_line.preview_strand_punctual_fixture && command_line.preview_strand_fixture) {
    throw std::invalid_argument("--preview-strand-punctual-fixture cannot be combined with another shadow fixture.");
  }
  if (command_line.preview_strand_gizmo_fixture && !command_line.demo_preview_capture_path) {
    throw std::invalid_argument("--preview-strand-gizmo-fixture requires --capture-demo-preview.");
  }
  if (command_line.preview_strand_gizmo_fixture && command_line.demo_profile_id != DemoProfileId::RenderingRegression) {
    throw std::invalid_argument("--preview-strand-gizmo-fixture requires --demo rendering-regression.");
  }
  if (command_line.preview_strand_gizmo_fixture &&
      (command_line.preview_strand_fixture || command_line.preview_strand_punctual_fixture)) {
    throw std::invalid_argument("--preview-strand-gizmo-fixture cannot be combined with another validation fixture.");
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
      main_camera->camera_settings.background_source = Camera::BackgroundSource::ClearColor;
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
  constexpr bool clear_generated_project_files = true;
  switch (profile_id) {
    case DemoProfileId::Rendering:
      SetupDemoScene(DemoSetup::Rendering, application_info, resource_root, clear_generated_project_files);
      break;
    case DemoProfileId::RenderingRegression:
      SetupDemoScene(DemoSetup::RenderingRegression, application_info, resource_root, clear_generated_project_files);
      break;
    case DemoProfileId::Ddgi:
      SetupDemoScene(DemoSetup::CornellBox, application_info, resource_root, clear_generated_project_files);
      ConfigureDdgiCornellBoxApplication(application_info, application_mode);
      break;
    case DemoProfileId::ProceduralGalaxy:
      SetupDemoScene(DemoSetup::ProceduralGalaxy, application_info, resource_root, clear_generated_project_files);
      break;
    case DemoProfileId::GaussianSplat:
      SetupDemoScene(DemoSetup::GaussianSplat, application_info, resource_root, clear_generated_project_files);
      break;
    case DemoProfileId::Bicycle:
      SetupDemoScene(DemoSetup::Bicycle, application_info, resource_root, clear_generated_project_files);
      break;
    case DemoProfileId::Bistro:
      SetupDemoScene(DemoSetup::Bistro, application_info, resource_root, clear_generated_project_files);
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
      camera_settings.background_source = Camera::BackgroundSource::ClearColor;
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
          scene_camera->camera_settings.background_source = Camera::BackgroundSource::ClearColor;
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

std::string JsonEscape(const std::string& value) {
  std::ostringstream stream;
  for (const unsigned char character : value) {
    switch (character) {
      case '"':
        stream << "\\\"";
        break;
      case '\\':
        stream << "\\\\";
        break;
      case '\n':
        stream << "\\n";
        break;
      case '\r':
        stream << "\\r";
        break;
      case '\t':
        stream << "\\t";
        break;
      default:
        if (character < 0x20u) {
          stream << "\\u" << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(character) << std::dec;
        } else {
          stream << static_cast<char>(character);
        }
        break;
    }
  }
  return stream.str();
}

template <size_t Size>
std::string HexBytes(const std::array<uint8_t, Size>& bytes) {
  std::ostringstream stream;
  stream << std::hex << std::setfill('0');
  for (const auto byte : bytes) {
    stream << std::setw(2) << static_cast<unsigned>(byte);
  }
  return stream.str();
}

struct DdgiHysteresisBoostFrame {
  float hysteresis = 0.0f;
  uint32_t update_reasons = DdgiUpdateReasonNone;
  uint32_t updated_probe_count = 0;
  uint32_t boosted_volume_count = 0;
  uint32_t restoring_volume_count = 0;
};

void WriteDdgiValidationReport(const std::filesystem::path& report_path, const std::filesystem::path& image_path,
                               const std::string& fixture_id, const uint32_t seed, const size_t measure_frames,
                               const size_t response_frames, const size_t warmup_frames, const glm::uvec2 resolution,
                               const Camera::CameraRenderMode render_mode, const std::string& phase,
                               const size_t run_index, const size_t convergence_frames, const bool convergence_observed,
                               const uint32_t history_reset_reasons_after_transition,
                               const uint32_t transition_update_reasons, const float transition_update_hysteresis,
                               const bool transition_response_observed, const bool transition_warmup_active,
                               const std::vector<DdgiHysteresisBoostFrame>& hysteresis_boost_frames,
                               const bool paused_change_latched, const bool paused_hysteresis_frozen,
                               const bool ddgi_enabled) {
  const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>();
  if (!render_layer) {
    throw std::runtime_error("DDGI validation report requires RenderLayer.");
  }
  const auto active_scene = ApplicationContext::Get().GetActiveScene();
  if (!active_scene) {
    throw std::runtime_error("DDGI validation report requires an active scene.");
  }
  const auto validation_lighting = active_scene->environmental_lighting.Get<EnvironmentalLighting>();
  if (!validation_lighting) {
    throw std::runtime_error("DDGI validation report requires asset-owned EnvironmentalLighting.");
  }
  const auto& validation_volume_defaults = validation_lighting->ddgi_settings.volume_defaults;
  const auto validation_ddgi_pack = validation_lighting->GetOrCreateDdgiVolumePack();
  const bool pause_updates_after_convergence =
      validation_ddgi_pack->volumes.empty()
          ? validation_volume_defaults.pause_probe_updates_after_convergence
          : validation_ddgi_pack->volumes.front().pause_probe_updates_after_convergence;
  const auto fingerprint = Platform::GetGpuDeviceFingerprint();
  const auto memory = Platform::GetGpuMemorySnapshot();
  (void)render_layer->RefreshDdgiProbeDebugData();
  const auto performance = render_layer->GetDdgiInspectorSnapshot().aggregate;
  const auto gpu_timestamps = Platform::GetGpuTimestampStats();
  auto render_mode_name = std::string(Camera::GetCameraRenderModeName(render_mode));
  std::transform(render_mode_name.begin(), render_mode_name.end(), render_mode_name.begin(), [](const char character) {
    return static_cast<char>(std::tolower(static_cast<unsigned char>(character)));
  });
  if (const auto parent = report_path.parent_path(); !parent.empty()) {
    std::filesystem::create_directories(parent);
  }
  std::ofstream output(report_path, std::ios::trunc);
  if (!output) {
    throw std::runtime_error("Failed to open DDGI validation report: " + report_path.string());
  }
  const bool dynamic_fixture = fixture_id == "scrolling" || fixture_id == "emissive-moving-rigid" ||
                               fixture_id == "emissive-enable" || fixture_id == "emissive-disable" ||
                               fixture_id == "emissive-enable-hdr" || fixture_id == "geometry-moving";
  const auto transition_kind = fixture_id == "emissive-enable" || fixture_id == "emissive-enable-hdr"
                                   ? "enable-emission"
                               : fixture_id == "emissive-disable" ? "disable-emission"
                               : fixture_id == "geometry-moving"  ? "translate-occluder-x"
                                                                  : "translate-x-one-spacing";
  output << std::setprecision(17);
  output << "{\n  \"schema_version\": 7,\n"
         << "  \"fixture_id\": \"" << JsonEscape(fixture_id) << "\",\n"
         << "  \"fixture_definition\": {\"version\": 6, \"sha256\": "
            "\"0f1012c5000072ad7c6ecaf74727f3e34fc0bbec3cf5d69e0ba8ffb8705239ea\"},\n"
         << "  \"image_file\": \"" << JsonEscape(image_path.filename().string()) << "\",\n"
         << "  \"capture\": {\"phase\": \"" << JsonEscape(phase) << "\", \"run_index\": " << run_index
         << ", \"output_encoding\": \"" << (image_path.extension() == ".hdr" ? "linear-rgbe-hdr" : "srgb8-png")
         << "\", \"transition_kind\": \"" << (dynamic_fixture ? transition_kind : "none")
         << "\", \"transition_applied\": " << (dynamic_fixture ? "true" : "false")
         << ", \"history_reset_after_transition\": "
         << (history_reset_reasons_after_transition != 0u ? "true" : "false")
         << ", \"history_reset_reasons_after_transition\": " << history_reset_reasons_after_transition
         << ", \"transition_response_observed\": " << (transition_response_observed ? "true" : "false")
         << ", \"transition_update_reasons\": " << transition_update_reasons
         << ", \"transition_update_hysteresis\": " << transition_update_hysteresis
         << ", \"transition_warmup_active\": " << (transition_warmup_active ? "true" : "false")
         << ", \"paused_change_latched\": " << (paused_change_latched ? "true" : "false")
         << ", \"paused_hysteresis_frozen\": " << (paused_hysteresis_frozen ? "true" : "false")
         << ", \"response_frames\": " << response_frames << ", \"hysteresis_boost_frames\": [";
  for (size_t index = 0; index < hysteresis_boost_frames.size(); ++index) {
    if (index != 0u) {
      output << ", ";
    }
    const auto& frame = hysteresis_boost_frames[index];
    output << "{\"hysteresis\": " << frame.hysteresis << ", \"update_reasons\": " << frame.update_reasons
           << ", \"updated_probes\": " << frame.updated_probe_count
           << ", \"boosted_volumes\": " << frame.boosted_volume_count
           << ", \"restoring_volumes\": " << frame.restoring_volume_count << "}";
  }
  output << "]},\n"
         << "  \"build\": {\"configuration\": \"" << EVOENGINE_BUILD_CONFIGURATION << "\"},\n"
         << "  \"contract\": {\"resolution\": [" << resolution.x << ", " << resolution.y << "], \"render_mode\": \""
         << render_mode_name
         << "\", "
            "\"vulkan_rt_pipeline\": "
         << (Platform::RayTracingEnabled() ? "true" : "false")
         << ", \"graphics_validation\": " << (Platform::GraphicsValidationEnabled() ? "true" : "false")
         << ", \"gpu_timestamps\": " << (Platform::GpuTimestampCaptureEnabled() ? "true" : "false")
         << ", \"deterministic_seed_enabled\": true, \"deterministic_seed\": " << seed
         << ", \"seed_sequence\": \"logical-ddgi-update-v1\", \"measure_frames\": " << measure_frames
         << ", \"warmup_frames\": " << warmup_frames
         << ", \"probe_variability_threshold\": " << validation_volume_defaults.probe_variability_threshold
         << ", \"probe_variability_min_samples\": " << validation_volume_defaults.probe_variability_min_samples
         << ", \"pause_updates_after_convergence\": " << (pause_updates_after_convergence ? "true" : "false")
         << ", \"ddgi_enabled\": " << (ddgi_enabled ? "true" : "false") << "},\n"
         << "  \"hardware\": {\"device_name\": \"" << JsonEscape(fingerprint.device_name)
         << "\", \"vendor_id\": " << fingerprint.vendor_id << ", \"device_id\": " << fingerprint.device_id
         << ", \"device_type\": " << fingerprint.device_type << ", \"driver_version\": " << fingerprint.driver_version
         << ", \"api_version\": " << fingerprint.api_version << ", \"driver_id\": " << fingerprint.driver_id
         << ", \"driver_name\": \"" << JsonEscape(fingerprint.driver_name) << "\", \"driver_info\": \""
         << JsonEscape(fingerprint.driver_info) << "\", \"pipeline_cache_uuid\": \""
         << HexBytes(fingerprint.pipeline_cache_uuid) << "\", \"device_uuid\": \"" << HexBytes(fingerprint.device_uuid)
         << "\", \"driver_uuid\": \"" << HexBytes(fingerprint.driver_uuid) << "\", \"conformance_version\": ["
         << static_cast<unsigned>(fingerprint.conformance_version[0]) << ", "
         << static_cast<unsigned>(fingerprint.conformance_version[1]) << ", "
         << static_cast<unsigned>(fingerprint.conformance_version[2]) << ", "
         << static_cast<unsigned>(fingerprint.conformance_version[3]) << "]},\n"
         << "  \"convergence\": {\"observed\": " << (convergence_observed ? "true" : "false")
         << ", \"frames\": " << convergence_frames << ", \"variability\": " << performance.probe_variability_average
         << ", \"variability_maximum\": " << performance.probe_variability_maximum
         << ", \"unstable_fraction\": " << performance.probe_variability_unstable_fraction << "},\n"
         << "  \"ddgi\": {\"active_probes\": " << performance.active_probe_count
         << ", \"storage_probes\": " << performance.storage_probe_count
         << ", \"updated_probes\": " << performance.updated_probe_count
         << ", \"rays_per_probe\": " << performance.ray_count + performance.emissive_ray_count
         << ", \"uniform_rays_per_probe\": " << performance.ray_count
         << ", \"emissive_rays_per_probe\": " << performance.emissive_ray_count
         << ", \"recorded_rays\": " << performance.recorded_ray_sample_count
         << ", \"update_hysteresis\": " << performance.probe_update_hysteresis
         << ", \"boosted_volumes\": " << performance.hysteresis_boosted_volume_count
         << ", \"restoring_volumes\": " << performance.hysteresis_restoring_volume_count
         << ", \"lighting_descriptors_bound\": " << (performance.lighting_descriptors_bound ? "true" : "false")
         << "},\n"
         << "  \"emissive_sampling\": {\"stats_available\": "
         << (performance.emissive_sampling_stats_available ? "true" : "false")
         << ", \"inventory_triangles\": " << performance.emissive_triangle_count
         << ", \"eligible_instances\": " << performance.emissive_eligible_instance_count
         << ", \"excluded_emissive_instances\": " << performance.emissive_excluded_instance_count
         << ", \"unrepresentable_probabilities\": " << performance.emissive_unrepresentable_probability_count
         << ", \"estimated_emitted_power\": " << performance.emissive_estimated_power
         << ", \"candidate_rays_upper_bound\": " << performance.emissive_sampling_candidate_ray_count
         << ", \"nee_attempts\": " << performance.emissive_nee_attempt_count
         << ", \"zero_pdf_rejects\": " << performance.emissive_zero_pdf_reject_count
         << ", \"emitter_backface_rejects\": " << performance.emissive_emitter_backface_reject_count
         << ", \"alpha_mask_rejects\": " << performance.emissive_alpha_mask_reject_count
         << ", \"invalid_sample_rejects\": " << performance.emissive_invalid_sample_reject_count
         << ", \"receiver_backface_rejects\": " << performance.emissive_receiver_backface_reject_count
         << ", \"shadowed_samples\": " << performance.emissive_shadowed_sample_count
         << ", \"zero_radiance_samples\": " << performance.emissive_zero_radiance_sample_count
         << ", \"nonzero_contributions\": " << performance.emissive_nonzero_contribution_count << "},\n"
         << "  \"logical_memory\": {\"scope\": \"validation-volume-logical-resources-v1\", "
            "\"probe_metadata_bytes\": "
         << performance.probe_metadata_byte_size << ", \"probe_state_bytes\": " << performance.probe_state_byte_size
         << ", \"probe_update_index_bytes\": 0"
         << ", \"ray_output_bytes\": " << performance.ray_output_byte_size
         << ", \"ray_sample_info_bytes\": " << performance.ray_sample_info_byte_size
         << ", \"irradiance_atlas_bytes\": " << performance.irradiance_atlas_byte_size
         << ", \"visibility_atlas_bytes\": " << performance.visibility_atlas_byte_size
         << ", \"variability_atlas_bytes\": " << performance.variability_atlas_byte_size
         << ", \"variability_reduction_bytes\": " << performance.variability_reduction_byte_size
         << ", \"persistent_bytes\": " << performance.persistent_byte_size
         << ", \"per_frame_transient_bytes\": " << performance.per_frame_transient_byte_size
         << ", \"peak_resident_bytes\": " << performance.peak_resident_byte_size << "},\n"
         << "  \"vma_memory\": {\"block_count\": " << memory.block_count
         << ", \"allocation_count\": " << memory.allocation_count << ", \"block_bytes\": " << memory.block_bytes
         << ", \"allocation_bytes\": " << memory.allocation_bytes << ", \"heaps\": [";
  for (size_t index = 0; index < memory.heaps.size(); ++index) {
    const auto& heap = memory.heaps[index];
    if (index != 0) {
      output << ", ";
    }
    output << "{\"index\": " << heap.heap_index << ", \"device_local\": " << (heap.device_local ? "true" : "false")
           << ", \"size_bytes\": " << heap.heap_size_bytes << ", \"allocation_bytes\": " << heap.allocation_bytes
           << ", \"driver_usage_bytes\": " << heap.driver_usage_bytes
           << ", \"driver_budget_bytes\": " << heap.driver_budget_bytes << "}";
  }
  output << "]},\n  \"gpu_timestamps\": {\n";
  for (size_t stat_index = 0; stat_index < gpu_timestamps.size(); ++stat_index) {
    const auto& stats = gpu_timestamps[stat_index];
    output << "    \"" << JsonEscape(stats.name) << "\": {\"sample_count\": " << stats.sample_count
           << ", \"minimum_ms\": " << stats.minimum_milliseconds << ", \"median_ms\": " << stats.MedianMilliseconds()
           << ", \"p95_ms\": " << stats.PercentileMilliseconds(0.95)
           << ", \"maximum_ms\": " << stats.maximum_milliseconds << ", \"samples_ms\": [";
    for (size_t sample_index = 0; sample_index < stats.samples_milliseconds.size(); ++sample_index) {
      if (sample_index != 0) {
        output << ", ";
      }
      output << stats.samples_milliseconds[sample_index];
    }
    output << "]}" << (stat_index + 1 == gpu_timestamps.size() ? "\n" : ",\n");
  }
  output << "  }\n}\n";
  if (!output) {
    throw std::runtime_error("Failed to write DDGI validation report: " + report_path.string());
  }
  std::cout << "EVOENGINE_DDGI_VALIDATION_REPORT path=\"" << report_path.string() << "\"" << std::endl;
}

void WriteRayCameraProfileReport(const std::filesystem::path& report_path, const std::filesystem::path& image_path,
                                 const Camera& camera, const Camera::CameraRenderMode render_mode,
                                 const size_t rendered_frames, const double elapsed_seconds,
                                 const double readiness_wait_milliseconds,
                                 const ShaderCompileCacheStats& readiness_shader_stats_before,
                                 const ShaderCompileCacheStats& readiness_shader_stats_after) {
  const auto history = camera.GetRayCameraHistoryStats();
  const auto memory = Platform::GetGpuMemorySnapshot();
  const auto fingerprint = Platform::GetGpuDeviceFingerprint();
  const auto timestamps = Platform::GetGpuTimestampStats();
  double ray_camera_gpu_average_ms = 0.0;
  for (const auto& timestamp : timestamps) {
    if (timestamp.name.rfind("Path Trace ", 0) == 0) {
      ray_camera_gpu_average_ms += timestamp.AverageMilliseconds();
    }
  }
  if (const auto parent = report_path.parent_path(); !parent.empty()) {
    std::filesystem::create_directories(parent);
  }
  std::ofstream output(report_path, std::ios::trunc);
  if (!output) {
    throw std::runtime_error("Failed to open ray camera profile report: " + report_path.string());
  }
  output << std::setprecision(17);
  output << "{\n  \"schema_version\": 1,\n"
         << "  \"image_file\": \"" << JsonEscape(image_path.filename().string()) << "\",\n"
         << "  \"hardware\": {\"device_name\": \"" << JsonEscape(fingerprint.device_name)
         << "\", \"vendor_id\": " << fingerprint.vendor_id << ", \"device_id\": " << fingerprint.device_id
         << ", \"driver_version\": " << fingerprint.driver_version << "},\n"
         << "  \"capture\": {\"render_mode\": \"" << JsonEscape(Camera::GetCameraRenderModeName(render_mode))
         << "\", \"rendered_frames\": " << rendered_frames << ", \"elapsed_seconds\": " << elapsed_seconds
         << ", \"frames_per_second\": "
         << (elapsed_seconds > 0.0 ? static_cast<double>(rendered_frames) / elapsed_seconds : 0.0)
         << ", \"gpu_average_ms\": " << ray_camera_gpu_average_ms << "},\n"
         << "  \"variant_readiness\": {\"wait_ms\": " << readiness_wait_milliseconds
         << ", \"shader_cache_delta\": {\"memory_hits\": "
         << readiness_shader_stats_after.memory_hits - readiness_shader_stats_before.memory_hits
         << ", \"disk_hits\": " << readiness_shader_stats_after.disk_hits - readiness_shader_stats_before.disk_hits
         << ", \"disk_misses\": "
         << readiness_shader_stats_after.disk_misses - readiness_shader_stats_before.disk_misses
         << ", \"compilations\": "
         << readiness_shader_stats_after.compilations - readiness_shader_stats_before.compilations
         << ", \"coalesced_waits\": "
         << readiness_shader_stats_after.coalesced_waits - readiness_shader_stats_before.coalesced_waits
         << ", \"failures\": " << readiness_shader_stats_after.failures - readiness_shader_stats_before.failures
         << ", \"native_slang_frontend\": "
         << readiness_shader_stats_after.native_slang_frontend_invocations -
                readiness_shader_stats_before.native_slang_frontend_invocations
         << ", \"compatibility_slang_frontend\": "
         << readiness_shader_stats_after.compatibility_slang_frontend_invocations -
                readiness_shader_stats_before.compatibility_slang_frontend_invocations
         << ", \"glslang_frontend\": "
         << readiness_shader_stats_after.glslang_frontend_invocations -
                readiness_shader_stats_before.glslang_frontend_invocations
         << "}},\n"
         << "  \"ray_history_memory\": {\"live_bytes\": " << history.live_byte_size
         << ", \"peak_live_bytes\": " << history.peak_live_byte_size << "},\n"
         << "  \"vma_memory\": {\"block_count\": " << memory.block_count
         << ", \"allocation_count\": " << memory.allocation_count << ", \"block_bytes\": " << memory.block_bytes
         << ", \"allocation_bytes\": " << memory.allocation_bytes << "},\n"
         << "  \"gpu_timestamps\": {\n";
  for (size_t index = 0; index < timestamps.size(); ++index) {
    const auto& stats = timestamps[index];
    output << "    \"" << JsonEscape(stats.name) << "\": {\"sample_count\": " << stats.sample_count
           << ", \"average_ms\": " << stats.AverageMilliseconds() << ", \"median_ms\": " << stats.MedianMilliseconds()
           << ", \"p95_ms\": " << stats.PercentileMilliseconds(0.95)
           << ", \"minimum_ms\": " << stats.minimum_milliseconds << ", \"maximum_ms\": " << stats.maximum_milliseconds
           << "}" << (index + 1 == timestamps.size() ? "\n" : ",\n");
  }
  output << "  }\n}\n";
  if (!output) {
    throw std::runtime_error("Failed to write ray camera profile report: " + report_path.string());
  }
  std::cout << "EVOENGINE_RAY_CAMERA_PROFILE_REPORT path=\"" << report_path.string() << "\"" << std::endl;
}

void CaptureDemoPreview(
    const std::filesystem::path& output_path, const int width, const int height, const size_t warmup_frames,
    const std::optional<DemoProfileId> demo_profile_id,
    const std::optional<Camera::CameraRenderMode>& preview_render_mode, const std::optional<int>& preview_ray_bounces,
    const std::optional<CameraSettings::RayDebugView>& preview_ray_debug_view,
    const std::optional<CameraSettings::RayOutputSettings>& preview_ray_outputs,
    const std::optional<CameraSettings::ShaderExecutionReorderingMode>& preview_ser_mode,
    const std::optional<float>& preview_firefly_clamp_threshold, const std::optional<bool>& preview_auto_spp_enabled,
    const std::optional<int>& preview_auto_spp_min_samples, const std::optional<int>& preview_auto_spp_max_samples,
    const std::optional<float>& preview_auto_spp_convergence_threshold, const std::optional<int> preview_sample_size,
    const std::optional<std::filesystem::path>& preview_ray_profile_report_path,
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
    const bool preview_strand_fixture, const bool preview_strand_punctual_fixture,
    const bool preview_strand_gizmo_fixture, const bool deterministic_capture, const bool preview_bistro_ddgi,
    const std::optional<std::string>& preview_ddgi_fixture,
    const std::optional<std::filesystem::path>& preview_ddgi_report_path, const uint32_t preview_ddgi_seed,
    const size_t preview_ddgi_measure_frames, const size_t preview_ddgi_response_frames,
    const bool preview_ddgi_disabled, const bool preview_ddgi_reference, const std::string& preview_ddgi_phase,
    const size_t preview_ddgi_run_index) {
  auto output_extension = output_path.extension().string();
  std::transform(output_extension.begin(), output_extension.end(), output_extension.begin(), [](const char character) {
    return static_cast<char>(std::tolower(static_cast<unsigned char>(character)));
  });
  const bool linear_hdr_output = output_extension == ".hdr";
  const glm::uvec2 preview_resolution(static_cast<uint32_t>(width), static_cast<uint32_t>(height));
  if (preview_strand_fixture || preview_strand_punctual_fixture || preview_strand_gizmo_fixture) {
    if (preview_strand_fixture) {
      ConfigureStrandMeshShaderValidation(ApplicationContext::Get().GetActiveScene());
    } else if (preview_strand_punctual_fixture) {
      ConfigureStrandPunctualShadowValidation(ApplicationContext::Get().GetActiveScene());
    } else {
      ConfigureStrandGizmoValidation(ApplicationContext::Get().GetActiveScene());
    }
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
  if ((preview_strand_fixture || preview_strand_gizmo_fixture) && !resolved_camera_position) {
    resolved_camera_position = glm::vec3(0.0f, 0.45f, 4.5f);
    resolved_camera_look_at = glm::vec3(0.0f, 0.0f, -2.5f);
  } else if (preview_strand_punctual_fixture && !resolved_camera_position) {
    resolved_camera_position = glm::vec3(0.0f, 3.0f, 8.0f);
    resolved_camera_look_at = glm::vec3(0.0f, 1.0f, -12.0f);
  }
  ApplyPreviewCameraOverride(editor_layer, resolved_camera_position, resolved_camera_look_at);
  if (preview_render_mode) {
    scene_camera->camera_render_mode = *preview_render_mode;
    scene_camera->ResetFrameCount();
  }
  if (preview_ddgi_reference) {
    scene_camera->camera_settings.bounce = 4;
    scene_camera->ResetFrameCount();
  }
  if (preview_ray_debug_view) {
    scene_camera->camera_settings.ray_debug_view = *preview_ray_debug_view;
    scene_camera->ResetFrameCount();
  }
  if (preview_ray_outputs) {
    scene_camera->camera_settings.ray_outputs = *preview_ray_outputs;
    scene_camera->ResetFrameCount();
  }
  if (preview_ser_mode) {
    scene_camera->camera_settings.shader_execution_reordering_mode = *preview_ser_mode;
    scene_camera->ResetFrameCount();
  }
  if (preview_firefly_clamp_threshold) {
    scene_camera->camera_settings.firefly_clamp_threshold = *preview_firefly_clamp_threshold;
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
  if (preview_strand_fixture || preview_strand_punctual_fixture || preview_strand_gizmo_fixture) {
    if (!render_layer) {
      throw std::runtime_error("Strand validation requires RenderLayer.");
    }
    const bool ray_strand_fixture = preview_strand_fixture && Camera::IsRayCameraRenderMode(resolved_render_mode);
    if (!ray_strand_fixture) {
      if (resolved_render_mode != Camera::CameraRenderMode::Rasterization || !Platform::MeshShaderEnabled()) {
        throw std::runtime_error("Raster strand validation requires mesh-shader support.");
      }
      render_layer->enable_meshlet = true;
    }
  }
  if (Camera::IsRayCameraRenderMode(resolved_render_mode)) {
    if (!render_layer) {
      throw std::runtime_error("Ray preview capture requires RenderLayer.");
    }
  }
  if (preview_ray_profile_report_path) {
    if (!Platform::GpuTimestampCaptureAvailable() || !Platform::GpuTimestampCaptureEnabled()) {
      throw std::runtime_error("Ray camera profiling requires available and enabled GPU timestamp capture.");
    }
    Platform::WaitForFrameSubmissions("Ray Camera Profile Warmup Fence Wait");
    Platform::ResetGpuTimestampStats();
  }
  if (linear_hdr_output && !Camera::IsRayCameraRenderMode(resolved_render_mode) &&
      !(preview_ddgi_fixture && preview_ddgi_report_path)) {
    throw std::invalid_argument("Linear HDR preview capture requires raytracing or rayquery mode.");
  }
  if (preview_ddgi_reference && resolved_render_mode != Camera::CameraRenderMode::RayTracing) {
    throw std::runtime_error("DDGI quality references require the Vulkan ray-tracing pipeline without fallback.");
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
  if (preview_ray_bounces) {
    scene_camera->camera_settings.bounce = std::max(0, *preview_ray_bounces);
    scene_camera->ResetFrameCount();
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
  const auto ray_camera_readiness_start = std::chrono::steady_clock::now();
  const auto ray_camera_readiness_shader_stats_before = Shader::GetCompileCacheStats();
  WaitForDemoPreviewSceneInputsReady();
  if (preview_strand_fixture) {
    UpdateStrandMeshShaderValidationGeometry(ApplicationContext::Get().GetActiveScene());
    WaitForDemoPreviewSceneInputsReady();
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
  const auto ray_camera_readiness_shader_stats_after = Shader::GetCompileCacheStats();
  const double ray_camera_readiness_wait_milliseconds =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - ray_camera_readiness_start).count();
  if (demo_profile_id == DemoProfileId::Bistro && !preview_bistro_ddgi) {
    const auto active_scene = ApplicationContext::Get().GetActiveScene();
    LogBistroParityCaptureState(active_scene, scene_camera, width, height,
                                Camera::GetCameraRenderModeName(resolved_render_mode), output_path);
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
  const bool temporal_motion_capture =
      demo_profile_id == DemoProfileId::RenderingRegression && preview_anti_aliasing_motion_sequence.value_or(false);
  const bool wait_for_ray_accumulation =
      Camera::IsRayCameraRenderMode(resolved_render_mode) && !temporal_motion_capture;
  constexpr size_t max_capture_frame_slack = 30000;
  const size_t max_capture_frames = warmup_frames + max_capture_frame_slack;
  size_t capture_frame_count = 0;
  const auto capture_start_time = std::chrono::steady_clock::now();
  while ((wait_for_ray_accumulation && scene_camera->GetFrameCount() < warmup_frames) ||
         (!wait_for_ray_accumulation && capture_frame_count < warmup_frames)) {
    if (!ApplicationContext::Get().Loop()) {
      throw std::runtime_error("Application ended before demo preview capture completed.");
    }
    if (++capture_frame_count >= max_capture_frames) {
      throw std::runtime_error(wait_for_ray_accumulation
                                   ? "Demo preview capture timed out before accumulating requested ray-tracing frames."
                                   : "Demo preview capture timed out.");
    }
  }
  size_t ddgi_convergence_frames = 0;
  bool ddgi_convergence_observed = false;
  uint32_t ddgi_history_reset_reasons_after_transition = DdgiUpdateReasonNone;
  uint32_t ddgi_transition_update_reasons = DdgiUpdateReasonNone;
  float ddgi_transition_update_hysteresis = 0.0f;
  bool ddgi_transition_response_observed = false;
  bool ddgi_transition_warmup_active = false;
  std::vector<DdgiHysteresisBoostFrame> ddgi_hysteresis_boost_frames;
  bool ddgi_paused_change_latched = false;
  bool ddgi_paused_hysteresis_frozen = false;
  if (preview_ddgi_fixture && preview_ddgi_report_path) {
    if (!render_layer || !Platform::GpuTimestampCaptureAvailable() || !Platform::GpuTimestampCaptureEnabled()) {
      throw std::runtime_error("DDGI validation requires available and enabled GPU timestamp capture.");
    }
    const auto active_scene = ApplicationContext::Get().GetActiveScene();
    if (!active_scene) {
      throw std::runtime_error("DDGI validation requires an active scene.");
    }
    const bool dynamic_fixture =
        *preview_ddgi_fixture == "scrolling" || *preview_ddgi_fixture == "emissive-moving-rigid" ||
        *preview_ddgi_fixture == "emissive-enable" || *preview_ddgi_fixture == "emissive-disable" ||
        *preview_ddgi_fixture == "emissive-enable-hdr" || *preview_ddgi_fixture == "geometry-moving";
    constexpr uint32_t history_reset_reason_mask = DdgiUpdateReasonSource | DdgiUpdateReasonManualReset;
    const auto record_transition_response = [&]() {
      const auto snapshot = render_layer->GetDdgiInspectorSnapshot();
      if (dynamic_fixture && snapshot.last_probe_history_cleared) {
        ddgi_history_reset_reasons_after_transition |= snapshot.last_probe_update_reasons & history_reset_reason_mask;
      }
      const bool scene_change_frame =
          dynamic_fixture && (snapshot.last_probe_update_reasons & DdgiUpdateReasonSceneChange) != 0u;
      const bool hysteresis_restore_frame =
          dynamic_fixture && (snapshot.last_probe_update_reasons & DdgiUpdateReasonHysteresisRestore) != 0u;
      if (scene_change_frame && !ddgi_transition_response_observed) {
        ddgi_transition_response_observed = true;
        ddgi_transition_update_hysteresis = snapshot.aggregate.probe_update_hysteresis;
      }
      if (scene_change_frame || hysteresis_restore_frame) {
        ddgi_transition_update_reasons |= snapshot.last_probe_update_reasons;
        ddgi_transition_warmup_active = ddgi_transition_warmup_active || snapshot.aggregate.probe_warmup_active;
        ddgi_hysteresis_boost_frames.push_back(
            {snapshot.aggregate.probe_update_hysteresis, snapshot.last_probe_update_reasons,
             snapshot.aggregate.updated_probe_count, snapshot.aggregate.hysteresis_boosted_volume_count,
             snapshot.aggregate.hysteresis_restoring_volume_count});
      }
    };
    const auto hysteresis_before_transition = [&] {
      const auto snapshot = render_layer->GetDdgiInspectorSnapshot();
      return snapshot.volumes.empty() ? render_layer->render_settings.ddgi_hysteresis
                                      : snapshot.volumes.front().current_hysteresis;
    }();
    if (dynamic_fixture && !AdvanceDdgiValidationFixture(active_scene, *preview_ddgi_fixture)) {
      throw std::runtime_error("DDGI validation fixture could not advance its dynamic target.");
    }
    if (dynamic_fixture && !preview_ddgi_disabled && preview_ddgi_response_frames == 0) {
      auto& session = render_layer->GetDdgiSessionState();
      session.pause_updates = true;
      for (size_t frame = 0; frame < 3; ++frame) {
        if (!ApplicationContext::Get().Loop()) {
          throw std::runtime_error("Application ended during paused DDGI scene-change validation.");
        }
      }
      const auto paused_snapshot = render_layer->GetDdgiInspectorSnapshot();
      ddgi_paused_change_latched =
          std::any_of(paused_snapshot.volumes.begin(), paused_snapshot.volumes.end(), [](const auto& volume) {
            return volume.pending_scene_changes;
          });
      ddgi_paused_hysteresis_frozen =
          !paused_snapshot.volumes.empty() &&
          std::all_of(paused_snapshot.volumes.begin(), paused_snapshot.volumes.end(), [&](const auto& volume) {
            return std::abs(volume.current_hysteresis - hysteresis_before_transition) <= 1.0e-6f &&
                   !volume.hysteresis_boost_active && !volume.hysteresis_boost_restoring;
          });
      session.pause_updates = false;
    }
    if (!preview_ddgi_disabled && preview_ddgi_response_frames == 0) {
      if (!dynamic_fixture) {
        render_layer->RequestDdgiHistoryReset();
      }
      constexpr size_t max_convergence_frames = 1024;
      while (ddgi_convergence_frames < max_convergence_frames) {
        if (!ApplicationContext::Get().Loop()) {
          throw std::runtime_error("Application ended during DDGI convergence measurement.");
        }
        record_transition_response();
        ++ddgi_convergence_frames;
        if (render_layer->GetDdgiInspectorSnapshot().aggregate.probe_variability_converged) {
          ddgi_convergence_observed = true;
          break;
        }
      }
      if (!ddgi_convergence_observed) {
        const auto& performance = render_layer->GetDdgiInspectorSnapshot().aggregate;
        std::ostringstream message;
        message << "DDGI validation did not converge within " << max_convergence_frames
                << " frames: variability=" << performance.probe_variability_average
                << ", maximum=" << performance.probe_variability_maximum
                << ", unstable_fraction=" << performance.probe_variability_unstable_fraction
                << ", samples=" << performance.probe_variability_sample_count
                << ", warmup=" << performance.probe_warmup_frame_index << "/" << performance.probe_warmup_frame_count
                << ", updated_probes=" << performance.updated_probe_count
                << ", update_reasons=" << render_layer->GetDdgiInspectorSnapshot().last_probe_update_reasons << ".";
        throw std::runtime_error(message.str());
      }
      auto& ddgi_settings =
          render_layer->GetScene()->environmental_lighting.Get<EnvironmentalLighting>()->ddgi_settings;
      ddgi_settings.volume_defaults.pause_probe_updates_after_convergence = false;
      if (const auto lighting = active_scene->environmental_lighting.Get<EnvironmentalLighting>()) {
        for (auto& volume : lighting->GetOrCreateDdgiVolumePack()->volumes) {
          volume.pause_probe_updates_after_convergence = false;
        }
      } else {
        throw std::runtime_error("DDGI validation timing requires asset-owned EnvironmentalLighting volumes.");
      }
      for (size_t frame = 0; frame < 8; ++frame) {
        if (!ApplicationContext::Get().Loop()) {
          throw std::runtime_error("Application ended during DDGI timing preparation.");
        }
        record_transition_response();
      }
    }
    Platform::WaitForFrameSubmissions("DDGI Validation Warmup Fence Wait");
    Platform::ResetGpuTimestampStats();
    const auto measurement_frame_count =
        preview_ddgi_response_frames > 0 ? preview_ddgi_response_frames : preview_ddgi_measure_frames;
    for (size_t frame = 0; frame < measurement_frame_count; ++frame) {
      if (!ApplicationContext::Get().Loop()) {
        throw std::runtime_error("Application ended during DDGI GPU timing measurement.");
      }
      record_transition_response();
      ++capture_frame_count;
    }
  }
  Platform::WaitForFrameSubmissions("Capture Completion Fence Wait");
  const auto capture_elapsed_seconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - capture_start_time).count();
  const auto capture_frames_per_second =
      capture_elapsed_seconds > 0.0 ? static_cast<double>(capture_frame_count) / capture_elapsed_seconds : 0.0;
  std::cout << "Demo preview capture: output=\"" << output_path.string()
            << "\" render_mode=" << Camera::GetCameraRenderModeName(resolved_render_mode)
            << " rendered_frames=" << capture_frame_count << " camera_frames=" << scene_camera->GetFrameCount()
            << " elapsed_seconds=" << capture_elapsed_seconds << " frames_per_second=" << capture_frames_per_second
            << " samples_per_frame=" << scene_camera->camera_settings.sample_size
            << " ray_bounces=" << scene_camera->camera_settings.bounce << std::endl;
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
  if (preview_ray_outputs && preview_ray_outputs->AnyEnabled()) {
    WritePreviewRayOutputReport(output_path, *scene_camera);
  }
  if (preview_ray_profile_report_path) {
    WriteRayCameraProfileReport(*preview_ray_profile_report_path, output_path, *scene_camera, resolved_render_mode,
                                capture_frame_count, capture_elapsed_seconds, ray_camera_readiness_wait_milliseconds,
                                ray_camera_readiness_shader_stats_before, ray_camera_readiness_shader_stats_after);
  }
  if (preview_ddgi_reference) {
    std::cout << "EVOENGINE_DDGI_REFERENCE fixture=" << *preview_ddgi_fixture
              << " render_mode=" << Camera::GetCameraRenderModeName(resolved_render_mode)
              << " resolution=" << render_extent.width << "x" << render_extent.height
              << " frames=" << scene_camera->GetFrameCount()
              << " spp_per_frame=4 total_spp=" << scene_camera->GetFrameCount() * 4 << " output=" << output_extension
              << std::endl;
  }
  if (preview_ddgi_fixture && preview_ddgi_report_path) {
    WriteDdgiValidationReport(
        *preview_ddgi_report_path, output_path, *preview_ddgi_fixture, preview_ddgi_seed,
        preview_ddgi_response_frames > 0 ? preview_ddgi_response_frames : preview_ddgi_measure_frames,
        preview_ddgi_response_frames, warmup_frames, glm::uvec2(render_extent.width, render_extent.height),
        resolved_render_mode, preview_ddgi_phase, preview_ddgi_run_index, ddgi_convergence_frames,
        ddgi_convergence_observed, ddgi_history_reset_reasons_after_transition, ddgi_transition_update_reasons,
        ddgi_transition_update_hysteresis, ddgi_transition_response_observed, ddgi_transition_warmup_active,
        ddgi_hysteresis_boost_frames, ddgi_paused_change_latched, ddgi_paused_hysteresis_frozen,
        !preview_ddgi_disabled);
  }

  editor_layer->SetSceneCameraResolutionOverride(std::nullopt);
}

void RunBistroSmoke(const int width, const int height) {
  const glm::uvec2 resolution(static_cast<uint32_t>(width), static_cast<uint32_t>(height));
  const auto editor_layer = ApplicationContext::Get().GetLayer<EditorLayer>();
  const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>();
  if (!editor_layer || !render_layer || !ApplicationContext::Get().GetActiveScene()) {
    throw std::runtime_error("Bistro smoke requires the editor, render layer, and an active scene.");
  }
  if (const auto window_layer = ApplicationContext::Get().GetLayer<WindowLayer>()) {
    window_layer->ResizeWindow(width, height);
    window_layer->CenterWindow();
  }
  editor_layer->show_camera_window = false;
  editor_layer->RequestSceneCameraPreviewWindow(resolution);
  editor_layer->SetSceneCameraResolutionOverride(resolution);
  const auto scene_camera = editor_layer->GetSceneCamera();
  if (!scene_camera) {
    throw std::runtime_error("Bistro smoke requires a scene camera.");
  }
  if (const auto main_camera = ApplicationContext::Get().GetActiveScene()->main_camera.Get<Camera>();
      main_camera && main_camera != scene_camera) {
    main_camera->SetEnabled(false);
  }
  scene_camera->SetRequireRendering(true);
  scene_camera->ResetFrameCount();
  WaitForDemoPreviewSceneInputsReady();

  constexpr size_t max_readiness_frames = 30000;
  bool observed_ddgi_execution = false;
  bool ddgi_ready = false;
  bool frame_ready = false;
  for (size_t frame = 0; frame < max_readiness_frames && (!ddgi_ready || !frame_ready); ++frame) {
    if (!ApplicationContext::Get().Loop()) {
      throw std::runtime_error("Application ended before Bistro smoke readiness.");
    }
    const auto stats = render_layer->GetDdgiInspectorSnapshot().aggregate;
    observed_ddgi_execution |= stats.recorded_probe_update_count > 0 && stats.recorded_ray_sample_count > 0 &&
                               stats.lighting_descriptors_bound;
    if (!ddgi_ready &&
        render_layer->GetScene()->environmental_lighting.Get<EnvironmentalLighting>()->ddgi_settings.runtime.enabled &&
        observed_ddgi_execution && stats.active_probe_count > 0 &&
        stats.storage_probe_count >= stats.active_probe_count && stats.irradiance_atlas_extent.x > 0 &&
        stats.irradiance_atlas_extent.y > 0 && stats.visibility_atlas_extent.x > 0 &&
        stats.visibility_atlas_extent.y > 0) {
      ddgi_ready = true;
      std::cout << "EVOENGINE_BISTRO_DDGI_READY active_probes=" << stats.active_probe_count
                << " storage_probes=" << stats.storage_probe_count
                << " recorded_updated_probes=" << stats.recorded_probe_update_count
                << " recorded_ray_samples=" << stats.recorded_ray_sample_count
                << " lighting_descriptors_bound=" << stats.lighting_descriptors_bound
                << " irradiance_atlas=" << stats.irradiance_atlas_extent.x << "x" << stats.irradiance_atlas_extent.y
                << " visibility_atlas=" << stats.visibility_atlas_extent.x << "x" << stats.visibility_atlas_extent.y
                << std::endl;
    }
    const auto render_texture = scene_camera->GetRenderTexture();
    if (!frame_ready && ddgi_ready && scene_camera->Rendered() && render_texture) {
      const auto extent = render_texture->GetExtent();
      if (extent.width == resolution.x && extent.height == resolution.y) {
        std::vector<glm::vec4> pixels;
        render_texture->GetRgbaChannelData(pixels);
        const auto expected_pixel_count = static_cast<size_t>(resolution.x) * resolution.y;
        if (pixels.size() != expected_pixel_count) {
          throw std::runtime_error("Bistro smoke render readback size does not match 1920x1080.");
        }
        double luminance_sum = 0.0;
        for (const auto& pixel : pixels) {
          if (!std::isfinite(pixel.x) || !std::isfinite(pixel.y) || !std::isfinite(pixel.z) ||
              !std::isfinite(pixel.w)) {
            throw std::runtime_error("Bistro smoke render readback contains non-finite pixels.");
          }
          luminance_sum +=
              std::max(0.0f, pixel.x) * 0.2126 + std::max(0.0f, pixel.y) * 0.7152 + std::max(0.0f, pixel.z) * 0.0722;
        }
        if (luminance_sum <= 1e-6) {
          throw std::runtime_error("Bistro smoke render readback is black.");
        }
        frame_ready = true;
        std::cout << "EVOENGINE_BISTRO_FRAME_READY resolution=" << extent.width << "x" << extent.height
                  << " finite_pixels=" << pixels.size() << " luminance_sum=" << luminance_sum << std::endl;
      }
    }
  }
  if (!ddgi_ready || !frame_ready) {
    throw std::runtime_error("Bistro smoke readiness timed out.");
  }

  const auto ready_time = std::chrono::steady_clock::now();
  auto last_heartbeat_time = ready_time - std::chrono::seconds(1);
  while (ApplicationContext::Get().Loop()) {
    const auto now = std::chrono::steady_clock::now();
    if (now - last_heartbeat_time >= std::chrono::seconds(1)) {
      const auto elapsed_seconds = std::chrono::duration<double>(now - ready_time).count();
      const auto stats = render_layer->GetDdgiInspectorSnapshot().aggregate;
      std::cout << "EVOENGINE_BISTRO_SMOKE_HEARTBEAT elapsed_seconds=" << elapsed_seconds
                << " active_probes=" << stats.active_probe_count << std::endl;
      last_heartbeat_time = now;
    }
  }
  editor_layer->SetSceneCameraResolutionOverride(std::nullopt);
}
}  // namespace

int main(const int argc, char** argv) {
  Application application;
  bool initialized = false;
  bool automated_capture = false;
  try {
    const auto command_line = ParseCommandLine(argc, argv);
    automated_capture = command_line.demo_preview_capture_path.has_value() || command_line.bistro_smoke;
    const auto& project_path = command_line.project_path;
    if (!project_path) {
      if (command_line.demo_profile_id) {
        PushStandardApplicationLayers(command_line.application_mode);

        ApplicationInitializationSettings application_info{};
        ConfigureDemoProfile(*command_line.demo_profile_id, command_line.application_mode, application_info);
        ApplyApplicationModeDefaults(application_info);
        ApplyGraphicsCommandLineOverrides(command_line, application_info);
        application_info.enable_gpu_timestamp_capture = command_line.preview_ddgi_report_path.has_value() ||
                                                        command_line.preview_ray_profile_report_path.has_value();
        ApplicationContext::Get().Initialize(application_info);
        initialized = true;
        if (command_line.application_mode == ApplicationMode::Editor) {
          ApplyDemoEditorDefaults(*command_line.demo_profile_id);
        }

        ApplicationContext::Get().Start(false);
        WaitForDemoProfileProjectIdle();
        ApplyDemoProfilePostLoadSetup(*command_line.demo_profile_id, command_line.application_mode);
        try {
          if (RunRenderingSponzaProbeAuthoringFromEnvironment()) {
            ApplicationContext::Get().Terminate();
            std::cout << "EVOENGINE_SPONZA_PROBE_AUTHORING_SHUTDOWN_COMPLETE" << std::endl;
            std::cout.flush();
            std::cerr.flush();
            std::_Exit(0);
          }
        } catch (const std::exception& error) {
          ApplicationContext::Get().Terminate();
          std::cerr << "EVOENGINE_SPONZA_PROBE_AUTHORING_ERROR " << error.what() << std::endl;
          std::cout.flush();
          std::cerr.flush();
          std::_Exit(1);
        }
        if (command_line.demo_preview_capture_path) {
          if (command_line.preview_ddgi_fixture) {
            const auto scene = ApplicationContext::Get().GetActiveScene();
            ConfigureDdgiValidationFixture(scene, *command_line.preview_ddgi_fixture);
            const auto lighting = scene ? scene->environmental_lighting.Get<EnvironmentalLighting>() : nullptr;
            if (!lighting) {
              throw std::runtime_error("DDGI preview fixture requires asset-owned EnvironmentalLighting settings.");
            }
            auto& ddgi = lighting->ddgi_settings;
            ddgi.runtime.deterministic_ray_seed_enabled = true;
            ddgi.runtime.deterministic_ray_seed = command_line.preview_ddgi_seed;
            if (command_line.preview_ddgi_uniform_ray_count) {
              ddgi.runtime.ray_count = *command_line.preview_ddgi_uniform_ray_count;
            }
            if (command_line.preview_ddgi_emissive_ray_count) {
              ddgi.runtime.emissive_ray_count = *command_line.preview_ddgi_emissive_ray_count;
            }
            if (command_line.preview_ddgi_continuous_updates) {
              ddgi.volume_defaults.pause_probe_updates_after_convergence = false;
              for (auto& volume : lighting->GetOrCreateDdgiVolumePack()->volumes) {
                volume.pause_probe_updates_after_convergence = false;
              }
            }
            ddgi.runtime.enabled = !command_line.preview_ddgi_disabled && !command_line.preview_ddgi_reference;
            if (const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>();
                render_layer && command_line.preview_ddgi_report_path.has_value() &&
                !command_line.preview_ddgi_disabled && !command_line.preview_ddgi_reference) {
              render_layer->RequestDdgiEmissiveSamplingCapture();
              render_layer->RequestDdgiGatherTimingCapture();
            }
            if (command_line.preview_ddgi_reference &&
                (*command_line.preview_ddgi_fixture == "scrolling" ||
                 *command_line.preview_ddgi_fixture == "emissive-moving-rigid" ||
                 *command_line.preview_ddgi_fixture == "emissive-enable" ||
                 *command_line.preview_ddgi_fixture == "emissive-disable" ||
                 *command_line.preview_ddgi_fixture == "emissive-enable-hdr" ||
                 *command_line.preview_ddgi_fixture == "geometry-moving") &&
                !AdvanceDdgiValidationFixture(scene, *command_line.preview_ddgi_fixture)) {
              throw std::runtime_error("DDGI reference fixture could not advance its dynamic target.");
            }
          }
          CaptureDemoPreview(
              *command_line.demo_preview_capture_path, command_line.preview_capture_width,
              command_line.preview_capture_height, command_line.preview_capture_warmup_frames,
              command_line.demo_profile_id, command_line.preview_capture_render_mode,
              command_line.preview_capture_ray_bounces, command_line.preview_capture_ray_debug_view,
              command_line.preview_capture_ray_outputs, command_line.preview_capture_ser_mode,
              command_line.preview_capture_firefly_clamp_threshold, command_line.preview_capture_auto_spp_enabled,
              command_line.preview_capture_auto_spp_min_samples, command_line.preview_capture_auto_spp_max_samples,
              command_line.preview_capture_auto_spp_convergence_threshold, command_line.preview_capture_sample_size,
              command_line.preview_ray_profile_report_path, command_line.preview_capture_camera_position,
              command_line.preview_capture_camera_look_at, command_line.preview_ambient_occlusion_enabled,
              command_line.preview_ambient_occlusion_algorithm, command_line.preview_anti_aliasing_enabled,
              command_line.preview_anti_aliasing_algorithm, command_line.preview_taa_preset,
              command_line.preview_smaa_preset, command_line.preview_anti_aliasing_tgsm,
              command_line.preview_anti_aliasing_fp16, command_line.preview_anti_aliasing_motion_sequence,
              command_line.preview_taa_debug_mode, command_line.preview_smaa_debug_mode,
              command_line.preview_anti_aliasing_debug_disabled, command_line.preview_shadow_split_lambda,
              command_line.preview_shadow_cascade_transition_width, command_line.preview_shadow_distance_fade,
              command_line.preview_shadow_fit_mode, command_line.preview_shadow_pcf_samples,
              command_line.preview_shadow_debug_mode, command_line.preview_shadow_debug_cascade,
              command_line.preview_shadow_debug_light, command_line.preview_strand_fixture,
              command_line.preview_strand_punctual_fixture, command_line.preview_strand_gizmo_fixture,
              command_line.preview_capture_deterministic, command_line.preview_capture_bistro_ddgi,
              command_line.preview_ddgi_fixture, command_line.preview_ddgi_report_path, command_line.preview_ddgi_seed,
              command_line.preview_ddgi_measure_frames, command_line.preview_ddgi_response_frames,
              command_line.preview_ddgi_disabled, command_line.preview_ddgi_reference, command_line.preview_ddgi_phase,
              command_line.preview_ddgi_run_index);
          ApplicationContext::Get().Terminate();
          std::cout.flush();
          std::cerr.flush();
          std::_Exit(0);
        }
        // DDGI_VALIDATION_CAPTURE_PROTOCOL_END
        try {
          if (RunReflectionProbeValidationFromEnvironment(command_line.preview_capture_width,
                                                          command_line.preview_capture_height)) {
            ApplicationContext::Get().Terminate();
            std::cout << "EVOENGINE_REFLECTION_PROBE_SHUTDOWN_COMPLETE" << std::endl;
            std::cout.flush();
            std::cerr.flush();
            std::_Exit(0);
          }
        } catch (const std::exception& error) {
          ApplicationContext::Get().Terminate();
          std::cerr << "EVOENGINE_REFLECTION_PROBE_ERROR " << error.what() << std::endl;
          std::cout.flush();
          std::cerr.flush();
          std::_Exit(1);
        }
        try {
          if (RunEnvironmentLightingValidationFromEnvironment(command_line.preview_capture_width,
                                                              command_line.preview_capture_height)) {
            ApplicationContext::Get().Terminate();
            std::cout << "EVOENGINE_ENVIRONMENT_LIGHTING_SHUTDOWN_COMPLETE" << std::endl;
            std::cout.flush();
            std::cerr.flush();
            std::_Exit(0);
          }
        } catch (const std::exception& error) {
          ApplicationContext::Get().Terminate();
          std::cerr << "EVOENGINE_ENVIRONMENT_LIGHTING_ERROR " << error.what() << std::endl;
          std::cout.flush();
          std::cerr.flush();
          std::_Exit(1);
        }
        try {
          if (RunDdgiEmissiveValidationFromEnvironment(command_line.preview_capture_width,
                                                       command_line.preview_capture_height)) {
            ApplicationContext::Get().Terminate();
            std::cout << "EVOENGINE_DDGI_EMISSIVE_SHUTDOWN_COMPLETE" << std::endl;
            std::cout.flush();
            std::cerr.flush();
            std::_Exit(0);
          }
        } catch (const std::exception& error) {
          ApplicationContext::Get().Terminate();
          std::cerr << "EVOENGINE_DDGI_EMISSIVE_ERROR " << error.what() << std::endl;
          std::cout.flush();
          std::cerr.flush();
          std::_Exit(1);
        }
        try {
          if (RunDdgiMultiVolumeValidationFromEnvironment(command_line.preview_capture_width,
                                                          command_line.preview_capture_height)) {
            ApplicationContext::Get().Terminate();
            std::cout << "EVOENGINE_DDGI_MULTI_VOLUME_SHUTDOWN_COMPLETE" << std::endl;
            std::cout.flush();
            std::cerr.flush();
            std::_Exit(0);
          }
        } catch (const std::exception& error) {
          ApplicationContext::Get().Terminate();
          std::cerr << "EVOENGINE_DDGI_MULTI_VOLUME_ERROR " << error.what() << std::endl;
          std::cout.flush();
          std::cerr.flush();
          std::_Exit(1);
        }
        if (command_line.bistro_smoke) {
          RunBistroSmoke(command_line.preview_capture_width, command_line.preview_capture_height);
          ApplicationContext::Get().Terminate();
          std::cout << "EVOENGINE_BISTRO_SMOKE_SHUTDOWN_COMPLETE" << std::endl;
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
