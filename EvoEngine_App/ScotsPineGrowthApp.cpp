#include "Application.hpp"
#include "AssetManager.hpp"
#include "Camera.hpp"
#include "ClassRegistry.hpp"
#include "Lights.hpp"
#include "ProjectManager.hpp"
#include "RenderLayer.hpp"
#include "Scene.hpp"

#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#ifdef CUDA_MODULE_PLUGIN
#  include "RayTracerLayer.hpp"
#endif

#ifdef LSYSTEM_PLUGIN
#  include "LSystemLayer.hpp"
#  include "ScotsPine.hpp"
#  include "ScotsPineDescriptor.hpp"
using namespace l_system_plugin;
#endif

using namespace evo_engine;

namespace {

struct CameraRigView {
  std::string label = "view";
  glm::vec3 position = glm::vec3(0.0f);
  float fx = 0.0f;
  float fy = 0.0f;
  float cx = 0.0f;
  float cy = 0.0f;
  uint32_t width = 1024u;
  uint32_t height = 1024u;
  float near_distance = 0.1f;
  float far_distance = 200.0f;
  float fov_deg = 60.0f;
  glm::vec3 forward = glm::vec3(0.0f, 0.0f, -1.0f);
  glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f);
};

struct Options {
  std::filesystem::path output_root{};
  std::string output_name = "scotspine";
  std::filesystem::path project_path{};
  std::filesystem::path scene_path = "DigitalAgriculture.evescene";
  std::filesystem::path descriptor_path = "New ScotsPineDescriptor.spine";
  std::filesystem::path post_repot_descriptor_path{};
  std::filesystem::path camera_rig_file{};
  bool use_empty_scene = true;

  uint32_t seed = 42u;
  std::optional<uint32_t> seed_a = std::nullopt;
  std::optional<uint32_t> seed_b = std::nullopt;
  std::optional<uint32_t> seed_c = std::nullopt;

  uint32_t frame_count = 10u;
  float max_target_gdd = 18000.0f;
  float repot_switch_gdd = 6000.0f;
  glm::uvec2 render_resolution = {1024u, 1024u};

  bool use_gpu = false;
  bool uncapped_growth = true;
  bool transparent_bg = false;

  bool export_mesh_per_frame = false;
  bool export_depth_per_view = false;
  bool export_instance_mask_per_view = false;
  bool export_node_graph_per_frame = false;
  bool export_flow_graph_per_frame = false;
  bool export_needle_skeleton_per_frame = false;

  float ambient_light = 0.30f;
  float directional_light = 7.0f;
  float visual_scale = 20.0f;

  float triangle_side_length = 3.0f;
  float triangle_offset_x = 0.0f;
  float triangle_offset_z = 0.0f;
  float triangle_yaw_deg = 0.0f;
};

struct PineInstance {
  Entity entity{};
  std::shared_ptr<ScotsPine> pine;
  uint32_t seed = 0u;
};

std::filesystem::path ResolveResourceFolderPath() {
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
  return std::filesystem::absolute(resource_folder_path);
}

std::string Trim(const std::string& value) {
  size_t begin = 0;
  while (begin < value.size() && std::isspace(static_cast<unsigned char>(value[begin])) != 0) {
    begin++;
  }
  size_t end = value.size();
  while (end > begin && std::isspace(static_cast<unsigned char>(value[end - 1])) != 0) {
    end--;
  }
  return value.substr(begin, end - begin);
}

std::string JsonEscape(const std::string& value) {
  std::ostringstream out;
  for (const char ch : value) {
    switch (ch) {
      case '"': out << "\\\""; break;
      case '\\': out << "\\\\"; break;
      case '\n': out << "\\n"; break;
      case '\r': out << "\\r"; break;
      case '\t': out << "\\t"; break;
      default:
        if (static_cast<unsigned char>(ch) < 0x20u) {
          out << "\\u" << std::hex << std::setw(4) << std::setfill('0')
              << static_cast<int>(static_cast<unsigned char>(ch))
              << std::dec << std::setfill(' ');
        } else {
          out << ch;
        }
        break;
    }
  }
  return out.str();
}

std::string SanitizeLabelForFilename(const std::string& value) {
  std::string out;
  out.reserve(value.size());
  for (const char ch : value) {
    const bool ok = (std::isalnum(static_cast<unsigned char>(ch)) != 0) || ch == '_' || ch == '-';
    out.push_back(ok ? ch : '_');
  }
  if (out.empty()) {
    return "view";
  }
  return out;
}

void PrintUsage() {
  EVOENGINE_LOG(
      "ScotsPineGrowthApp options:\n"
      "  --output-root <path>         Folder for generated outputs (required).\n"
      "  --output-name <name>         File prefix (default: scotspine).\n"
      "  --project-path <path>        .eveproj path (default: DigitalAgricultureProject/test.eveproj).\n"
      "  --scene-path <path>          Scene asset path (default: DigitalAgriculture.evescene).\n"
      "  --empty-scene                Ignore scene asset and render in a temporary empty scene (default).\n"
      "  --load-scene                 Load the scene asset specified by --scene-path.\n"
      "  --descriptor-path <path>     Scots pine descriptor path (default: New ScotsPineDescriptor.spine).\n"
      "  --post-repot-descriptor-path <path> Optional post-repot descriptor for profile switching.\n"
      "  --camera-rig-file <path>     Optional camera rig text file (one line per view).\n"
      "  --seed <N>                   Base deterministic seed (default: 42).\n"
      "  --seed-a <N>                 Seed for tree A (default: --seed).\n"
      "  --seed-b <N>                 Seed for tree B (default: --seed + 1).\n"
      "  --seed-c <N>                 Seed for tree C (default: --seed + 2).\n"
      "  --frame-count <N>            Number of growth frames (default: 10).\n"
      "  --max-target-gdd <float>     Final target GDD at last frame (default: 18000).\n"
      "  --repot-switch-gdd <float>   GDD trigger for post-repot descriptor (default: 6000).\n"
      "  --render-resolution WxH      Fallback render resolution (default: 1024x1024).\n"
      "  --triangle-side-length <m>   Side length of 3-tree equilateral triangle (default: 3.0).\n"
      "  --triangle-offset-x <m>      Triangle center X offset (default: 0).\n"
      "  --triangle-offset-z <m>      Triangle center Z offset (default: 0).\n"
      "  --triangle-yaw-deg <deg>     Triangle yaw around +Y (default: 0).\n"
      "  --ambient-light <float>      Ambient light intensity (default: 0.30).\n"
      "  --directional-light <float>  Directional light intensity (default: 7.0).\n"
      "  --visual-scale <float>       Uniform scale for each tree (default: 20.0).\n"
      "  --gpu | --cpu                Toggle ray tracer layer (default: --cpu).\n"
      "  --transparent-bg             Clear background to transparent RGBA.\n"
      "  --export-mesh                Export OBJ per frame per tree.\n"
      "  --export-depth               Export linear depth PNG per frame/view.\n"
      "  --export-instance-mask       Export instance-color mask per frame/view.\n"
      "  --export-node-graph          Export node graph YAML per frame/tree.\n"
      "  --export-flow-graph          Export flow graph YAML per frame/tree.\n"
      "  --export-needle-skeleton     Export per-needle centerline YAML per frame/tree.\n"
      "  --uncapped-growth            Use uncapped growth solve (default).\n"
      "  --capped-growth              Use capped growth solve.\n"
      "  --help                       Print this help message.\n"
      "\n"
      "Camera rig line format:\n"
      "  label px py pz fx fy cx cy width height near far fov forward_x forward_y forward_z up_x up_y up_z\n");
}

bool ParseResolution(const std::string& value, glm::uvec2& out_resolution) {
  const auto separator = value.find('x');
  if (separator == std::string::npos || separator == 0 || separator >= value.size() - 1) {
    return false;
  }
  try {
    const auto width = static_cast<uint32_t>(std::stoul(value.substr(0, separator)));
    const auto height = static_cast<uint32_t>(std::stoul(value.substr(separator + 1)));
    if (width == 0 || height == 0) {
      return false;
    }
    out_resolution = {width, height};
    return true;
  } catch (const std::exception&) {
    return false;
  }
}

bool ParseU32(const std::string& value, uint32_t& out_value) {
  try {
    size_t consumed = 0;
    const auto parsed = std::stoull(value, &consumed, 10);
    if (consumed != value.size() || parsed > std::numeric_limits<uint32_t>::max()) {
      return false;
    }
    out_value = static_cast<uint32_t>(parsed);
    return true;
  } catch (const std::exception&) {
    return false;
  }
}

bool ParseF32(const std::string& value, float& out_value) {
  try {
    size_t consumed = 0;
    const auto parsed = std::stof(value, &consumed);
    if (consumed != value.size()) {
      return false;
    }
    out_value = parsed;
    return true;
  } catch (const std::exception&) {
    return false;
  }
}

bool ParseOptions(const int argc, char** argv, Options& options, bool& show_help, std::string& error_message) {
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

    if (arg == "--output-root") {
      const auto* value = require_value("--output-root");
      if (!value) {
        return false;
      }
      options.output_root = std::filesystem::path(value);
      continue;
    }

    if (arg == "--output-name") {
      const auto* value = require_value("--output-name");
      if (!value) {
        return false;
      }
      options.output_name = value;
      continue;
    }

    if (arg == "--project-path") {
      const auto* value = require_value("--project-path");
      if (!value) {
        return false;
      }
      options.project_path = std::filesystem::path(value);
      continue;
    }

    if (arg == "--scene-path") {
      const auto* value = require_value("--scene-path");
      if (!value) {
        return false;
      }
      options.scene_path = std::filesystem::path(value);
      continue;
    }

    if (arg == "--descriptor-path") {
      const auto* value = require_value("--descriptor-path");
      if (!value) {
        return false;
      }
      options.descriptor_path = std::filesystem::path(value);
      continue;
    }

    if (arg == "--post-repot-descriptor-path") {
      const auto* value = require_value("--post-repot-descriptor-path");
      if (!value) {
        return false;
      }
      options.post_repot_descriptor_path = std::filesystem::path(value);
      continue;
    }

    if (arg == "--camera-rig-file") {
      const auto* value = require_value("--camera-rig-file");
      if (!value) {
        return false;
      }
      options.camera_rig_file = std::filesystem::path(value);
      continue;
    }

    if (arg == "--empty-scene") {
      options.use_empty_scene = true;
      continue;
    }

    if (arg == "--load-scene") {
      options.use_empty_scene = false;
      continue;
    }

    if (arg == "--seed") {
      const auto* value = require_value("--seed");
      if (!value) {
        return false;
      }
      if (!ParseU32(value, options.seed)) {
        error_message = "Invalid value for --seed: " + std::string(value);
        return false;
      }
      continue;
    }

    if (arg == "--seed-a") {
      const auto* value = require_value("--seed-a");
      if (!value) {
        return false;
      }
      uint32_t parsed = 0u;
      if (!ParseU32(value, parsed)) {
        error_message = "Invalid value for --seed-a: " + std::string(value);
        return false;
      }
      options.seed_a = parsed;
      continue;
    }

    if (arg == "--seed-b") {
      const auto* value = require_value("--seed-b");
      if (!value) {
        return false;
      }
      uint32_t parsed = 0u;
      if (!ParseU32(value, parsed)) {
        error_message = "Invalid value for --seed-b: " + std::string(value);
        return false;
      }
      options.seed_b = parsed;
      continue;
    }

    if (arg == "--seed-c") {
      const auto* value = require_value("--seed-c");
      if (!value) {
        return false;
      }
      uint32_t parsed = 0u;
      if (!ParseU32(value, parsed)) {
        error_message = "Invalid value for --seed-c: " + std::string(value);
        return false;
      }
      options.seed_c = parsed;
      continue;
    }

    if (arg == "--frame-count") {
      const auto* value = require_value("--frame-count");
      if (!value) {
        return false;
      }
      if (!ParseU32(value, options.frame_count) || options.frame_count == 0) {
        error_message = "Invalid value for --frame-count: " + std::string(value);
        return false;
      }
      continue;
    }

    if (arg == "--max-target-gdd") {
      const auto* value = require_value("--max-target-gdd");
      if (!value) {
        return false;
      }
      if (!ParseF32(value, options.max_target_gdd) || options.max_target_gdd < 0.0f) {
        error_message = "Invalid value for --max-target-gdd: " + std::string(value);
        return false;
      }
      continue;
    }

    if (arg == "--repot-switch-gdd") {
      const auto* value = require_value("--repot-switch-gdd");
      if (!value) {
        return false;
      }
      if (!ParseF32(value, options.repot_switch_gdd) || options.repot_switch_gdd < 0.0f) {
        error_message = "Invalid value for --repot-switch-gdd: " + std::string(value);
        return false;
      }
      continue;
    }

    if (arg == "--render-resolution") {
      const auto* value = require_value("--render-resolution");
      if (!value) {
        return false;
      }
      if (!ParseResolution(value, options.render_resolution)) {
        error_message = "Invalid value for --render-resolution: " + std::string(value);
        return false;
      }
      continue;
    }

    if (arg == "--triangle-side-length") {
      const auto* value = require_value("--triangle-side-length");
      if (!value) {
        return false;
      }
      if (!ParseF32(value, options.triangle_side_length) || options.triangle_side_length <= 0.0f) {
        error_message = "Invalid value for --triangle-side-length: " + std::string(value);
        return false;
      }
      continue;
    }

    if (arg == "--triangle-offset-x") {
      const auto* value = require_value("--triangle-offset-x");
      if (!value) {
        return false;
      }
      if (!ParseF32(value, options.triangle_offset_x)) {
        error_message = "Invalid value for --triangle-offset-x: " + std::string(value);
        return false;
      }
      continue;
    }

    if (arg == "--triangle-offset-z") {
      const auto* value = require_value("--triangle-offset-z");
      if (!value) {
        return false;
      }
      if (!ParseF32(value, options.triangle_offset_z)) {
        error_message = "Invalid value for --triangle-offset-z: " + std::string(value);
        return false;
      }
      continue;
    }

    if (arg == "--triangle-yaw-deg") {
      const auto* value = require_value("--triangle-yaw-deg");
      if (!value) {
        return false;
      }
      if (!ParseF32(value, options.triangle_yaw_deg)) {
        error_message = "Invalid value for --triangle-yaw-deg: " + std::string(value);
        return false;
      }
      continue;
    }

    if (arg == "--ambient-light") {
      const auto* value = require_value("--ambient-light");
      if (!value) {
        return false;
      }
      if (!ParseF32(value, options.ambient_light) || options.ambient_light < 0.0f) {
        error_message = "Invalid value for --ambient-light: " + std::string(value);
        return false;
      }
      continue;
    }

    if (arg == "--directional-light") {
      const auto* value = require_value("--directional-light");
      if (!value) {
        return false;
      }
      if (!ParseF32(value, options.directional_light) || options.directional_light < 0.0f) {
        error_message = "Invalid value for --directional-light: " + std::string(value);
        return false;
      }
      continue;
    }

    if (arg == "--visual-scale") {
      const auto* value = require_value("--visual-scale");
      if (!value) {
        return false;
      }
      if (!ParseF32(value, options.visual_scale) || options.visual_scale <= 0.0f) {
        error_message = "Invalid value for --visual-scale: " + std::string(value);
        return false;
      }
      continue;
    }

    if (arg == "--gpu") {
      options.use_gpu = true;
      continue;
    }

    if (arg == "--cpu") {
      options.use_gpu = false;
      continue;
    }

    if (arg == "--transparent-bg") {
      options.transparent_bg = true;
      continue;
    }

    if (arg == "--export-mesh") {
      options.export_mesh_per_frame = true;
      continue;
    }

    if (arg == "--export-depth") {
      options.export_depth_per_view = true;
      continue;
    }

    if (arg == "--export-instance-mask") {
      options.export_instance_mask_per_view = true;
      continue;
    }

    if (arg == "--export-node-graph") {
      options.export_node_graph_per_frame = true;
      continue;
    }

    if (arg == "--export-flow-graph") {
      options.export_flow_graph_per_frame = true;
      continue;
    }

    if (arg == "--export-needle-skeleton") {
      options.export_needle_skeleton_per_frame = true;
      continue;
    }

    if (arg == "--uncapped-growth") {
      options.uncapped_growth = true;
      continue;
    }

    if (arg == "--capped-growth") {
      options.uncapped_growth = false;
      continue;
    }

    error_message = "Unknown option: " + arg;
    return false;
  }

  if (options.output_root.empty()) {
    error_message = "--output-root is required.";
    return false;
  }

  if (options.output_name.empty()) {
    error_message = "--output-name cannot be empty.";
    return false;
  }

  if (!options.seed_a.has_value()) {
    options.seed_a = options.seed;
  }
  if (!options.seed_b.has_value()) {
    options.seed_b = options.seed + 1u;
  }
  if (!options.seed_c.has_value()) {
    options.seed_c = options.seed + 2u;
  }

  return true;
}

glm::vec3 SafeNormalize(const glm::vec3& value, const glm::vec3& fallback) {
  const float len = glm::length(value);
  if (!std::isfinite(len) || len <= 1.0e-6f) {
    return fallback;
  }
  return value / len;
}

glm::quat BuildCameraRotation(const glm::vec3& forward, const glm::vec3& up) {
  const glm::vec3 front = SafeNormalize(forward, glm::vec3(0.0f, 0.0f, -1.0f));
  glm::vec3 up_vec = SafeNormalize(up, glm::vec3(0.0f, 1.0f, 0.0f));

  glm::vec3 right = glm::cross(front, up_vec);
  right = SafeNormalize(right, glm::vec3(1.0f, 0.0f, 0.0f));

  up_vec = glm::cross(right, front);
  up_vec = SafeNormalize(up_vec, glm::vec3(0.0f, 1.0f, 0.0f));

  // Camera uses local -Z as look direction.
  glm::mat3 world_from_local(1.0f);
  world_from_local[0] = right;
  world_from_local[1] = up_vec;
  world_from_local[2] = -front;
  return glm::normalize(glm::quat_cast(world_from_local));
}

CameraRigView MakeDefaultCameraView(const Options& options) {
  CameraRigView view;
  view.label = "default";
  view.position = glm::vec3(0.0f, 0.8f, 7.5f);

  const glm::quat rotation = glm::quat(glm::radians(glm::vec3(15.0f, 0.0f, 0.0f)));
  view.forward = rotation * glm::vec3(0.0f, 0.0f, -1.0f);
  view.up = rotation * glm::vec3(0.0f, 1.0f, 0.0f);

  view.width = options.render_resolution.x;
  view.height = options.render_resolution.y;
  view.near_distance = 0.1f;
  view.far_distance = 200.0f;
  view.fov_deg = 120.0f;

  const float half_w = static_cast<float>(view.width) * 0.5f;
  const float half_h = static_cast<float>(view.height) * 0.5f;
  const float fy = half_h / std::tan(glm::radians(view.fov_deg * 0.5f));
  view.fx = fy;
  view.fy = fy;
  view.cx = half_w;
  view.cy = half_h;

  return view;
}

bool LoadCameraRigFromFile(const std::filesystem::path& path,
                           std::vector<CameraRigView>& out_views,
                           std::string& error_message) {
  std::ifstream in(path.string());
  if (!in) {
    error_message = "Failed to open camera rig file: " + path.string();
    return false;
  }

  out_views.clear();
  std::string line;
  size_t line_number = 0;
  while (std::getline(in, line)) {
    line_number++;
    const std::string trimmed = Trim(line);
    if (trimmed.empty() || trimmed[0] == '#') {
      continue;
    }

    std::istringstream iss(trimmed);
    CameraRigView view;
    uint32_t width = 0u;
    uint32_t height = 0u;

    if (!(iss >> view.label
              >> view.position.x >> view.position.y >> view.position.z
              >> view.fx >> view.fy >> view.cx >> view.cy
              >> width >> height
              >> view.near_distance >> view.far_distance >> view.fov_deg
              >> view.forward.x >> view.forward.y >> view.forward.z
              >> view.up.x >> view.up.y >> view.up.z)) {
      error_message = "Malformed camera rig line " + std::to_string(line_number) + " in " + path.string();
      return false;
    }

    view.width = width;
    view.height = height;
    if (view.width == 0 || view.height == 0) {
      error_message = "Invalid camera resolution on line " + std::to_string(line_number);
      return false;
    }

    view.near_distance = std::max(1.0e-4f, view.near_distance);
    view.far_distance = std::max(view.near_distance + 1.0e-4f, view.far_distance);
    view.fov_deg = std::clamp(view.fov_deg, 5.0f, 175.0f);
    view.forward = SafeNormalize(view.forward, glm::vec3(0.0f, 0.0f, -1.0f));
    view.up = SafeNormalize(view.up, glm::vec3(0.0f, 1.0f, 0.0f));
    out_views.push_back(view);
  }

  if (out_views.empty()) {
    error_message = "Camera rig file has no usable camera lines: " + path.string();
    return false;
  }

  return true;
}

void WriteCameraMetadataJson(const std::filesystem::path& path,
                             const std::vector<CameraRigView>& views) {
  if (path.empty()) {
    return;
  }
  if (!path.parent_path().empty()) {
    std::filesystem::create_directories(path.parent_path());
  }
  std::ofstream out(path.string(), std::ios::trunc);
  if (!out) {
    EVOENGINE_WARNING("Failed to write camera metadata JSON: " + path.string());
    return;
  }

  out << std::fixed << std::setprecision(9);
  out << "{\n";
  out << "  \"views\": [\n";
  for (size_t i = 0; i < views.size(); i++) {
    const auto& view = views[i];
    out << "    {\n";
    out << "      \"index\": " << i << ",\n";
    out << "      \"label\": \"" << JsonEscape(view.label) << "\",\n";
    out << "      \"position\": [" << view.position.x << ", " << view.position.y << ", " << view.position.z << "],\n";
    out << "      \"forward\": [" << view.forward.x << ", " << view.forward.y << ", " << view.forward.z << "],\n";
    out << "      \"up\": [" << view.up.x << ", " << view.up.y << ", " << view.up.z << "],\n";
    out << "      \"intrinsic\": {\"fx\": " << view.fx << ", \"fy\": " << view.fy
        << ", \"cx\": " << view.cx << ", \"cy\": " << view.cy << "},\n";
    out << "      \"resolution\": [" << view.width << ", " << view.height << "],\n";
    out << "      \"near\": " << view.near_distance << ",\n";
    out << "      \"far\": " << view.far_distance << ",\n";
    out << "      \"fov_deg\": " << view.fov_deg << "\n";
    out << "    }";
    if (i + 1 < views.size()) {
      out << ",";
    }
    out << "\n";
  }
  out << "  ]\n";
  out << "}\n";
}

std::array<glm::vec3, 3> BuildTrianglePositions(const Options& options) {
  const float side = options.triangle_side_length;
  const float half_side = side * 0.5f;
  const float tri_h = side * std::sqrt(3.0f) * 0.5f;
  const float centroid_to_apex = (2.0f / 3.0f) * tri_h;
  const float centroid_to_base = (1.0f / 3.0f) * tri_h;

  std::array<glm::vec3, 3> points = {
      glm::vec3(0.0f, 0.0f, centroid_to_apex),
      glm::vec3(-half_side, 0.0f, -centroid_to_base),
      glm::vec3(half_side, 0.0f, -centroid_to_base),
  };

  const float yaw = glm::radians(options.triangle_yaw_deg);
  const float c = std::cos(yaw);
  const float s = std::sin(yaw);
  for (auto& p : points) {
    const float x = c * p.x + s * p.z;
    const float z = -s * p.x + c * p.z;
    p.x = x + options.triangle_offset_x;
    p.z = z + options.triangle_offset_z;
  }
  return points;
}

#ifdef LSYSTEM_PLUGIN
std::shared_ptr<ScotsPineDescriptor> ResolveDescriptor(const std::filesystem::path& descriptor_path) {
  if (descriptor_path.empty()) {
    return AssetManager::CreateTemporaryAsset<ScotsPineDescriptor>();
  }

  if (descriptor_path.is_relative()) {
    const auto assets_candidate = ProjectManager::GetAssetsFolderPath() / descriptor_path;
    if (std::filesystem::exists(assets_candidate)) {
      auto descriptor = std::dynamic_pointer_cast<ScotsPineDescriptor>(ProjectManager::GetOrCreateAsset(descriptor_path));
      if (descriptor) {
        return descriptor;
      }
      descriptor = AssetManager::CreateTemporaryAsset<ScotsPineDescriptor>();
      descriptor->Import(assets_candidate);
      return descriptor;
    }

    if (std::filesystem::exists(descriptor_path)) {
      auto descriptor = AssetManager::CreateTemporaryAsset<ScotsPineDescriptor>();
      descriptor->Import(std::filesystem::absolute(descriptor_path));
      return descriptor;
    }
  } else {
    if (ProjectManager::IsInAssetsFolder(descriptor_path)) {
      const auto relative_path = ProjectManager::GetAssetsRelativePath(descriptor_path);
      auto descriptor = std::dynamic_pointer_cast<ScotsPineDescriptor>(ProjectManager::GetOrCreateAsset(relative_path));
      if (descriptor) {
        return descriptor;
      }
    }

    if (std::filesystem::exists(descriptor_path)) {
      auto descriptor = AssetManager::CreateTemporaryAsset<ScotsPineDescriptor>();
      descriptor->Import(descriptor_path);
      return descriptor;
    }
  }

  EVOENGINE_WARNING("Failed to resolve Scots pine descriptor path; using default descriptor values.");
  return AssetManager::CreateTemporaryAsset<ScotsPineDescriptor>();
}
#endif

}  // namespace

int main(int argc, char** argv) {
#ifndef LSYSTEM_PLUGIN
  EVOENGINE_ERROR("ScotsPineGrowthApp requires LSYSTEM_PLUGIN.");
  return 1;
#endif

  Options options;
  bool show_help = false;
  std::string error_message;

  const auto resource_folder_path = ResolveResourceFolderPath();
  options.project_path = resource_folder_path / "DigitalAgricultureProject" / "test.eveproj";

  if (!ParseOptions(argc, argv, options, show_help, error_message)) {
    EVOENGINE_ERROR(error_message);
    PrintUsage();
    return 2;
  }

  if (show_help) {
    PrintUsage();
    return 0;
  }

  if (std::filesystem::path(options.project_path).extension().string() != ".eveproj") {
    EVOENGINE_ERROR("Project path doesn't point to a valid .eveproj file.");
    return 3;
  }

  if (!std::filesystem::exists(options.project_path)) {
    EVOENGINE_ERROR("Project path does not exist: " + options.project_path.string());
    return 3;
  }

  std::filesystem::create_directories(options.output_root);

  Application::PushLayer<RenderLayer>("Render Layer");
#ifdef CUDA_MODULE_PLUGIN
  if (options.use_gpu) {
    Application::PushLayer<RayTracerLayer>("Ray Tracer Layer");
  }
#endif
  Application::PushLayer<LSystemLayer>("LSystem Layer");

  ApplicationInitializationSettings application_info{};
  application_info.application_name = "ScotsPineGrowthApp";
  application_info.project_path = options.project_path;
  Application::Initialize(application_info);

  std::shared_ptr<Scene> target_scene;
  if (options.use_empty_scene) {
    target_scene = AssetManager::CreateTemporaryAsset<Scene>();
  } else {
    target_scene = std::dynamic_pointer_cast<Scene>(ProjectManager::GetOrCreateAsset(options.scene_path));
  }
  if (!target_scene) {
    const std::string scene_error = options.use_empty_scene
                                        ? "Failed to create temporary empty scene."
                                        : ("Failed to load scene asset: " + options.scene_path.string());
    EVOENGINE_ERROR(scene_error);
    Application::Terminate();
    return 4;
  }
  Application::Attach(target_scene);
  Application::Start();

  const auto scene = Application::GetActiveScene();
  if (!scene) {
    EVOENGINE_ERROR("No active scene after initialization.");
    Application::Terminate();
    return 5;
  }

  scene->environment.ambient_light_intensity = options.ambient_light;
  if (const auto* directional_light_entities = scene->UnsafeGetPrivateComponentOwnersList<DirectionalLight>()) {
    for (const auto& light_entity : *directional_light_entities) {
      const auto light = scene->GetOrSetPrivateComponent<DirectionalLight>(light_entity).lock();
      if (light) {
        light->diffuse_brightness = options.directional_light;
      }
    }
    if (directional_light_entities->empty()) {
      const auto light_entity = scene->CreateEntity("Directional Light");
      const auto light = scene->GetOrSetPrivateComponent<DirectionalLight>(light_entity).lock();
      if (light) {
        light->diffuse_brightness = options.directional_light;
      }
      auto light_transform = scene->GetDataComponent<GlobalTransform>(light_entity);
      light_transform.SetEulerRotation(glm::radians(glm::vec3(-45.0f, 35.0f, 0.0f)));
      scene->SetDataComponent(light_entity, light_transform);
    }
  } else {
    const auto light_entity = scene->CreateEntity("Directional Light");
    const auto light = scene->GetOrSetPrivateComponent<DirectionalLight>(light_entity).lock();
    if (light) {
      light->diffuse_brightness = options.directional_light;
    }
    auto light_transform = scene->GetDataComponent<GlobalTransform>(light_entity);
    light_transform.SetEulerRotation(glm::radians(glm::vec3(-45.0f, 35.0f, 0.0f)));
    scene->SetDataComponent(light_entity, light_transform);
  }

  const auto descriptor = ResolveDescriptor(options.descriptor_path);
  if (!descriptor) {
    EVOENGINE_ERROR("Unable to create Scots pine descriptor.");
    Application::Terminate();
    return 6;
  }

  std::shared_ptr<ScotsPineDescriptor> post_repot_descriptor = nullptr;
  if (!options.post_repot_descriptor_path.empty()) {
    post_repot_descriptor = ResolveDescriptor(options.post_repot_descriptor_path);
    if (!post_repot_descriptor) {
      EVOENGINE_ERROR("Unable to create post-repot Scots pine descriptor.");
      Application::Terminate();
      return 6;
    }
  }

  const auto positions = BuildTrianglePositions(options);
  std::array<uint32_t, 3> tree_seeds = {
      options.seed_a.value_or(options.seed),
      options.seed_b.value_or(options.seed + 1u),
      options.seed_c.value_or(options.seed + 2u),
  };

  std::vector<PineInstance> pines;
  pines.reserve(3);
  for (size_t i = 0; i < 3; i++) {
    const auto entity = scene->CreateEntity("Scots Pine " + std::to_string(i));
    const auto pine = scene->GetOrSetPrivateComponent<ScotsPine>(entity).lock();
    if (!pine) {
      EVOENGINE_ERROR("Unable to create ScotsPine component.");
      Application::Terminate();
      return 7;
    }

    pine->descriptor_ref = descriptor;
    pine->post_repot_descriptor_ref = post_repot_descriptor;
    pine->enable_repot_profile_switch = static_cast<bool>(post_repot_descriptor);
    pine->repot_switch_gdd = std::max(0.0f, options.repot_switch_gdd);

    auto pine_transform = scene->GetDataComponent<GlobalTransform>(entity);
    pine_transform.SetScale(glm::vec3(options.visual_scale));
    pine_transform.SetPosition(positions[i]);
    scene->SetDataComponent(entity, pine_transform);

    pines.push_back(PineInstance{entity, pine, tree_seeds[i]});
  }

  auto camera = scene->main_camera.Get<Camera>();
  Entity camera_entity{};
  bool created_camera = false;
  if (!camera) {
    camera_entity = scene->CreateEntity("Main Camera");
    camera = scene->GetOrSetPrivateComponent<Camera>(camera_entity).lock();
    scene->main_camera = camera;
    created_camera = true;
  } else {
    camera_entity = camera->GetOwner();
  }

  if (!camera) {
    EVOENGINE_ERROR("Unable to resolve camera for rendering.");
    Application::Terminate();
    return 8;
  }

  std::vector<CameraRigView> camera_views;
  if (!options.camera_rig_file.empty()) {
    std::string load_error;
    if (!LoadCameraRigFromFile(options.camera_rig_file, camera_views, load_error)) {
      EVOENGINE_ERROR(load_error);
      Application::Terminate();
      return 9;
    }
  }
  if (camera_views.empty()) {
    camera_views.push_back(MakeDefaultCameraView(options));
  }

  camera->camera_render_mode = options.use_gpu ? Camera::CameraRenderMode::RayTracing
                                                : Camera::CameraRenderMode::Rasterization;

  const auto camera_json_path = options.output_root / (options.output_name + std::string("_camera_views.json"));
  WriteCameraMetadataJson(camera_json_path, camera_views);

  EVOENGINE_LOG("ScotsPineGrowthApp started: output_root=" + options.output_root.string() +
                ", frame_count=" + std::to_string(options.frame_count) +
                ", views=" + std::to_string(camera_views.size()));

  for (uint32_t frame_index = 0; frame_index < options.frame_count; frame_index++) {
    const float t = options.frame_count <= 1u
                        ? 1.0f
                        : static_cast<float>(frame_index) / static_cast<float>(options.frame_count - 1u);

    for (size_t tree_index = 0; tree_index < pines.size(); tree_index++) {
      auto& instance = pines[tree_index];
      instance.pine->seed = instance.seed;
      instance.pine->target_gdd = std::max(0.0f, options.max_target_gdd * t);
      instance.pine->GenerateGeometryEntities(options.uncapped_growth);

      std::ostringstream tree_stem;
      tree_stem << options.output_name << "_f" << std::setw(4) << std::setfill('0') << frame_index
                << "_tree" << tree_index;
      const auto tree_prefix = options.output_root / tree_stem.str();

      if (options.export_mesh_per_frame) {
        instance.pine->ExportObj(tree_prefix.string() + ".obj");
      }
      if (options.export_node_graph_per_frame) {
        instance.pine->ExportNodeGraph(tree_prefix.string() + "_node.yaml");
      }
      if (options.export_flow_graph_per_frame) {
        instance.pine->ExportFlowGraph(tree_prefix.string() + "_flow.yaml");
      }
      if (options.export_needle_skeleton_per_frame) {
        instance.pine->ExportNeedleSkeleton(tree_prefix.string() + "_needle_skeleton.yaml");
      }
    }

    for (size_t view_index = 0; view_index < camera_views.size(); view_index++) {
      const auto& view = camera_views[view_index];

      auto camera_transform = scene->GetDataComponent<GlobalTransform>(camera_entity);
      camera_transform.SetPosition(view.position);
      camera_transform.SetRotation(BuildCameraRotation(view.forward, view.up));
      scene->SetDataComponent(camera_entity, camera_transform);

      camera->camera_settings.near_distance = std::max(1.0e-4f, view.near_distance);
      camera->camera_settings.far_distance = std::max(camera->camera_settings.near_distance + 1.0e-4f, view.far_distance);
      camera->camera_settings.fov = std::clamp(view.fov_deg, 5.0f, 175.0f);
      camera->camera_settings.use_clear_color = true;
      camera->camera_settings.clear_color = options.transparent_bg
                                                ? glm::vec4(0.0f, 0.0f, 0.0f, 0.0f)
                                                : glm::vec4(0.97f, 0.97f, 0.97f, 1.0f);
      camera->Resize(glm::uvec2(view.width, view.height));

      ScotsPine::SetGlobalColorMode(ScotsPine::ColorMode::Shaded);
      Application::Loop();
      Application::Loop();

      const std::string label = SanitizeLabelForFilename(view.label);
      std::ostringstream view_stem;
      view_stem << options.output_name << "_f" << std::setw(4) << std::setfill('0') << frame_index
                << "_v" << std::setw(2) << std::setfill('0') << view_index << "_" << label;
      const auto prefix = options.output_root / view_stem.str();

      camera->GetRenderTexture()->StoreToPng(prefix.string() + "_rgba.png");

      if (options.export_depth_per_view) {
        camera->GetRenderTexture()->StoreLinearDepthToPng(
            prefix.string() + "_depth.png",
            camera->camera_settings.near_distance,
            camera->camera_settings.far_distance,
            camera->camera_settings.far_distance);
      }

      if (options.export_instance_mask_per_view) {
        camera->camera_settings.clear_color = glm::vec4(0.0f, 0.0f, 0.0f, 0.0f);
        ScotsPine::SetGlobalColorMode(ScotsPine::ColorMode::ByInstance);
        Application::Loop();
        Application::Loop();
        camera->GetRenderTexture()->StoreToPng(prefix.string() + "_instance_mask.png");
        ScotsPine::SetGlobalColorMode(ScotsPine::ColorMode::Shaded);
      }
    }

    EVOENGINE_LOG("frame " + std::to_string(frame_index + 1u) + "/" + std::to_string(options.frame_count));
  }

  if (created_camera && scene->IsEntityValid(camera_entity)) {
    scene->DeleteEntity(camera_entity);
  }

  for (const auto& instance : pines) {
    if (scene->IsEntityValid(instance.entity)) {
      scene->DeleteEntity(instance.entity);
    }
  }

  Application::Terminate();
  EVOENGINE_LOG("ScotsPineGrowthApp finished.");
  return 0;
}
