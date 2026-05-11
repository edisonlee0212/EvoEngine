#include "Application.hpp"
#include "Camera.hpp"
#include "ClassRegistry.hpp"
#include "Lights.hpp"
#include "ProjectManager.hpp"
#include "RenderLayer.hpp"
#include "Scene.hpp"

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <optional>
#include <sstream>
#include <string>

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

struct Options {
  std::filesystem::path output_root{};
  std::string output_name = "scotspine";
  std::filesystem::path project_path{};
  std::filesystem::path scene_path = "DigitalAgriculture.evescene";
  std::filesystem::path descriptor_path = "New ScotsPineDescriptor.spine";
  std::filesystem::path post_repot_descriptor_path{};
  uint32_t seed = 42u;
  uint32_t frame_count = 10u;
  float max_target_gdd = 18000.0f;
  float repot_switch_gdd = 6000.0f;
  glm::uvec2 render_resolution = {1024u, 1024u};
  bool use_gpu = false;
  bool export_mesh_per_frame = false;
  bool uncapped_growth = true;
  float ambient_light = 0.30f;
  float directional_light = 7.0f;
  float visual_scale = 20.0f;
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

void PrintUsage() {
  EVOENGINE_LOG("ScotsPineGrowthApp options:\n"
                "  --output-root <path>       Folder for generated PNG frames (required).\n"
                "  --output-name <name>       Frame file prefix (default: scotspine).\n"
                "  --project-path <path>      .eveproj path (default: DigitalAgricultureProject/test.eveproj).\n"
                "  --scene-path <path>        Scene asset path (default: DigitalAgriculture.evescene).\n"
                "  --descriptor-path <path>   Scots pine descriptor path (default: New ScotsPineDescriptor.spine).\n"
                "  --post-repot-descriptor-path <path> Optional post-repot descriptor for profile switching.\n"
                "  --seed <N>                 Deterministic seed (default: 42).\n"
                "  --frame-count <N>          Number of growth frames (default: 10).\n"
                "  --max-target-gdd <float>   Final target GDD at last frame (default: 18000).\n"
                "  --repot-switch-gdd <float> GDD trigger for switching to post-repot descriptor (default: 6000).\n"
                "  --render-resolution WxH    PNG resolution (default: 1024x1024).\n"
                "  --ambient-light <float>    Ambient light intensity (default: 0.30).\n"
                "  --directional-light <float> Directional light intensity (default: 7.0).\n"
                "  --visual-scale <float>     Uniform scene scale applied to generated pine (default: 20.0).\n"
                "  --gpu | --cpu              Toggle ray tracer layer (default: --cpu).\n"
                "  --export-mesh              Export per-frame OBJ alongside PNGs.\n"
                "  --uncapped-growth          Use uncapped growth solve (default).\n"
                "  --capped-growth            Use capped growth solve.\n"
                "  --help                     Print this help message.");
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
      if (!ParseF32(value, options.repot_switch_gdd) ||
          options.repot_switch_gdd < 0.0f) {
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

    if (arg == "--export-mesh") {
      options.export_mesh_per_frame = true;
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

  return true;
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

  const auto target_scene = std::dynamic_pointer_cast<Scene>(ProjectManager::GetOrCreateAsset(options.scene_path));
  if (!target_scene) {
    EVOENGINE_ERROR("Failed to load scene asset: " + options.scene_path.string());
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

  const auto pine_entity = scene->CreateEntity("Scots Pine Growth");
  const auto pine = scene->GetOrSetPrivateComponent<ScotsPine>(pine_entity).lock();
  if (!pine) {
    EVOENGINE_ERROR("Unable to create ScotsPine component.");
    Application::Terminate();
    return 7;
  }
  pine->descriptor_ref = descriptor;
  pine->post_repot_descriptor_ref = post_repot_descriptor;
  pine->enable_repot_profile_switch = static_cast<bool>(post_repot_descriptor);
  pine->repot_switch_gdd = std::max(0.0f, options.repot_switch_gdd);

  auto pine_transform = scene->GetDataComponent<GlobalTransform>(pine_entity);
  pine_transform.SetScale(glm::vec3(options.visual_scale));
  scene->SetDataComponent(pine_entity, pine_transform);

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

  auto camera_transform = scene->GetDataComponent<GlobalTransform>(camera_entity);
  camera_transform.SetPosition(glm::vec3(0.0f, 0.8f, 7.5f));
  camera_transform.SetEulerRotation(glm::radians(glm::vec3(15.0f, 0.0f, 0.0f)));
  scene->SetDataComponent(camera_entity, camera_transform);

  camera->camera_settings.use_clear_color = true;
  camera->camera_settings.clear_color = glm::vec4(0.97f, 0.97f, 0.97f, 1.0f);
  camera->camera_render_mode =
      options.use_gpu ? Camera::CameraRenderMode::RayTracing
                      : Camera::CameraRenderMode::Rasterization;

  EVOENGINE_LOG("ScotsPineGrowthApp started: output_root=" + options.output_root.string() +
                ", frame_count=" + std::to_string(options.frame_count) +
                ", seed=" + std::to_string(options.seed));

  for (uint32_t frame_index = 0; frame_index < options.frame_count; frame_index++) {
    const float t = options.frame_count <= 1u
                        ? 1.0f
                        : static_cast<float>(frame_index) / static_cast<float>(options.frame_count - 1u);

    pine->seed = options.seed;
    pine->target_gdd = std::max(0.0f, options.max_target_gdd * t);
    pine->max_growth_steps_per_frame = 0u;
    pine->GenerateGeometryEntities(options.uncapped_growth);

    camera->Resize(options.render_resolution);
    Application::Loop();
    Application::Loop();

    std::ostringstream frame_name;
    frame_name << options.output_name << "_" << std::setw(4) << std::setfill('0') << frame_index << ".png";
    const auto frame_path = options.output_root / frame_name.str();
    camera->GetRenderTexture()->StoreToPng(frame_path);

    if (options.export_mesh_per_frame) {
      std::ostringstream mesh_name;
      mesh_name << options.output_name << "_" << std::setw(4) << std::setfill('0') << frame_index << ".obj";
      pine->ExportObj(options.output_root / mesh_name.str());
    }

    EVOENGINE_LOG("frame " + std::to_string(frame_index + 1u) + "/" + std::to_string(options.frame_count) +
                  " target_gdd=" + std::to_string(pine->target_gdd));
  }

  if (created_camera && scene->IsEntityValid(camera_entity)) {
    scene->DeleteEntity(camera_entity);
  }
  if (scene->IsEntityValid(pine_entity)) {
    scene->DeleteEntity(pine_entity);
  }

  Application::Terminate();
  EVOENGINE_LOG("ScotsPineGrowthApp finished.");
  return 0;
}
