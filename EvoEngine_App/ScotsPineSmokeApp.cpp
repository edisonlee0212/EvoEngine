#include "Application.hpp"
#include "ClassRegistry.hpp"
#include "Material.hpp"
#include "Mesh.hpp"
#include "MeshRenderer.hpp"
#include "Particles.hpp"
#include "ProjectManager.hpp"
#include "RenderLayer.hpp"
#include "Scene.hpp"
#include "Transform.hpp"

#include <AssetManager.hpp>

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#ifdef LSYSTEM_PLUGIN
#  include "LSystemLayer.hpp"
#  include "ScotsPine.hpp"
#  include "ScotsPineDescriptor.hpp"
using namespace l_system_plugin;
#endif

using namespace evo_engine;

namespace {

struct Options {
  std::filesystem::path output_json{};
  std::filesystem::path project_path{};
  std::filesystem::path scene_path = "DigitalAgriculture.evescene";
  std::filesystem::path smoke_scene_path = "DigitalAgriculture_ScotsPineSmoke.evescene";
  std::filesystem::path descriptor_path = "New ScotsPineDescriptor.spine";
  uint32_t tree_count = 3u;
  uint32_t seed_start = 42u;
  uint32_t seed_step = 1u;
  float target_gdd = -1.0f;
  float spacing = 3.0f;
  uint32_t min_needles_per_tree = 1u;
  float max_color_distance = 0.30f;
  bool uncapped_growth = true;
  // Multi-tree-grid mode: when rows > 0 AND cols > 0, override tree_count
  // and place pines in an R x C grid (mirrors the descriptor's Grid
  // Instantiate UI button used for the missing-needle bug repro).
  uint32_t grid_rows = 0u;
  uint32_t grid_cols = 0u;
  // Iteration B/D probe: after the initial GenerateGeometryEntities, drive
  // RebuildGeometry on every pine N times, with one Application::Loop frame
  // between cycles. The headless oracle PASSES on the initial generate; the
  // user-visible bug only appears after rebuilds + frames, so the harness has
  // to exercise that path to catch it.
  uint32_t rebuild_cycles = 0u;
};

struct TreeReport {
  uint32_t tree_index = 0u;
  uint32_t entity_index = 0u;
  uint32_t seed = 0u;
  float target_gdd = 0.0f;
  uint32_t internode_count = 0u;
  uint32_t needle_count = 0u;
  uint32_t invalid_instance_count = 0u;
  bool has_needle_entity = false;
  bool has_needle_geometry_entity = false;
  bool needle_uses_mesh_renderer = false;
  size_t needle_vertex_count = 0u;
  size_t needle_triangle_count = 0u;
  size_t needle_color_sample_count = 0u;
  glm::vec3 needle_mean_color = glm::vec3(0.0f);
  uintptr_t needle_mesh_ptr = 0u;
  uintptr_t needle_material_ptr = 0u;
  uintptr_t needle_particle_info_list_ptr = 0u;
  uint64_t needle_mesh_handle = 0u;
  uint64_t needle_material_handle = 0u;
  uint64_t needle_particle_info_list_handle = 0u;
  uint32_t needle_mesh_version = 0u;
  uint32_t needle_material_version = 0u;
  uint32_t needle_particle_info_list_version = 0u;
  uint32_t needle_geom_entity_index = 0u;
};

std::filesystem::path ResolveResourceFolderPath() {
  std::filesystem::path resource_folder_path("./Resources");
  if (!std::filesystem::exists(resource_folder_path)) {
    resource_folder_path = "../../../../../Resources";
  }
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
  EVOENGINE_LOG("ScotsPineSmokeApp options:\n"
                "  --output-json <path>         Optional JSON report output path.\n"
                "  --project-path <path>        .eveproj path (default: DigitalAgricultureProject/test.eveproj).\n"
                "  --scene-path <path>          Baseline scene asset to clone (default: DigitalAgriculture.evescene).\n"
                "  --smoke-scene-path <path>    Isolated scene asset to attach (default: DigitalAgriculture_ScotsPineSmoke.evescene).\n"
                "  --descriptor-path <path>     Scots pine descriptor path (default: New ScotsPineDescriptor.spine).\n"
                "  --tree-count <N>             Number of pines to spawn in one scene (default: 3).\n"
                "  --seed-start <N>             Seed of first tree (default: 42).\n"
                "  --seed-step <N>              Delta between tree seeds (default: 1).\n"
                "  --target-gdd <float>         Fixed target GDD. Negative uses descriptor-sampled target per seed (default: -1).\n"
                "  --spacing <float>            Tree spacing in world units (default: 3.0).\n"
                "  --min-needles-per-tree <N>   Fail if any tree has fewer needles than N (default: 1).\n"
                "  --max-color-distance <float> Fail if max pairwise mean needle-color distance exceeds threshold (default: 0.30).\n"
                "  --multi-tree-grid <R> <C>    Place pines in an R x C grid (overrides --tree-count). Mirrors descriptor's Grid Instantiate.\n"
                "  --rebuild-cycles <N>         After initial generate, RebuildGeometry every pine N times with a frame between (probes the per-frame rebuild path that triggers the bug).\n"
                "  --uncapped-growth            Use uncapped growth solve (default).\n"
                "  --capped-growth              Use capped growth solve.\n"
                "  --help                       Print this help message.");
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

bool ParseOptions(const int argc, char** argv, Options& options, bool& show_help,
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

    if (arg == "--output-json") {
      const auto* value = require_value("--output-json");
      if (!value) return false;
      options.output_json = std::filesystem::path(value);
      continue;
    }

    if (arg == "--project-path") {
      const auto* value = require_value("--project-path");
      if (!value) return false;
      options.project_path = std::filesystem::path(value);
      continue;
    }

    if (arg == "--scene-path") {
      const auto* value = require_value("--scene-path");
      if (!value) return false;
      options.scene_path = std::filesystem::path(value);
      continue;
    }

    if (arg == "--smoke-scene-path") {
      const auto* value = require_value("--smoke-scene-path");
      if (!value) return false;
      options.smoke_scene_path = std::filesystem::path(value);
      continue;
    }

    if (arg == "--descriptor-path") {
      const auto* value = require_value("--descriptor-path");
      if (!value) return false;
      options.descriptor_path = std::filesystem::path(value);
      continue;
    }

    if (arg == "--tree-count") {
      const auto* value = require_value("--tree-count");
      if (!value) return false;
      if (!ParseU32(value, options.tree_count) || options.tree_count == 0u) {
        error_message = "Invalid value for --tree-count: " + std::string(value);
        return false;
      }
      continue;
    }

    if (arg == "--seed-start") {
      const auto* value = require_value("--seed-start");
      if (!value) return false;
      if (!ParseU32(value, options.seed_start)) {
        error_message = "Invalid value for --seed-start: " + std::string(value);
        return false;
      }
      continue;
    }

    if (arg == "--seed-step") {
      const auto* value = require_value("--seed-step");
      if (!value) return false;
      if (!ParseU32(value, options.seed_step)) {
        error_message = "Invalid value for --seed-step: " + std::string(value);
        return false;
      }
      continue;
    }

    if (arg == "--target-gdd") {
      const auto* value = require_value("--target-gdd");
      if (!value) return false;
      if (!ParseF32(value, options.target_gdd)) {
        error_message = "Invalid value for --target-gdd: " + std::string(value);
        return false;
      }
      continue;
    }

    if (arg == "--spacing") {
      const auto* value = require_value("--spacing");
      if (!value) return false;
      if (!ParseF32(value, options.spacing) || options.spacing < 0.0f) {
        error_message = "Invalid value for --spacing: " + std::string(value);
        return false;
      }
      continue;
    }

    if (arg == "--min-needles-per-tree") {
      const auto* value = require_value("--min-needles-per-tree");
      if (!value) return false;
      if (!ParseU32(value, options.min_needles_per_tree)) {
        error_message = "Invalid value for --min-needles-per-tree: " + std::string(value);
        return false;
      }
      continue;
    }

    if (arg == "--max-color-distance") {
      const auto* value = require_value("--max-color-distance");
      if (!value) return false;
      if (!ParseF32(value, options.max_color_distance) || options.max_color_distance < 0.0f) {
        error_message = "Invalid value for --max-color-distance: " + std::string(value);
        return false;
      }
      continue;
    }

    if (arg == "--multi-tree-grid") {
      const auto* rows_value = require_value("--multi-tree-grid R");
      if (!rows_value) return false;
      if (!ParseU32(rows_value, options.grid_rows) || options.grid_rows == 0u) {
        error_message = "Invalid R for --multi-tree-grid: " + std::string(rows_value);
        return false;
      }
      const auto* cols_value = require_value("--multi-tree-grid C");
      if (!cols_value) return false;
      if (!ParseU32(cols_value, options.grid_cols) || options.grid_cols == 0u) {
        error_message = "Invalid C for --multi-tree-grid: " + std::string(cols_value);
        return false;
      }
      options.tree_count = options.grid_rows * options.grid_cols;
      continue;
    }

    if (arg == "--rebuild-cycles") {
      const auto* value = require_value("--rebuild-cycles");
      if (!value) return false;
      if (!ParseU32(value, options.rebuild_cycles)) {
        error_message = "Invalid value for --rebuild-cycles: " + std::string(value);
        return false;
      }
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

  return true;
}

bool StartsWith(const std::string& value, const std::string& prefix) {
  return value.rfind(prefix, 0) == 0;
}

bool IsFiniteVec4(const glm::vec4& v) {
  return std::isfinite(v.x) && std::isfinite(v.y) &&
         std::isfinite(v.z) && std::isfinite(v.w);
}

std::string JsonEscape(const std::string& value) {
  std::ostringstream escaped;
  for (const char ch : value) {
    switch (ch) {
      case '\\': escaped << "\\\\"; break;
      case '"': escaped << "\\\""; break;
      case '\n': escaped << "\\n"; break;
      case '\r': escaped << "\\r"; break;
      case '\t': escaped << "\\t"; break;
      default:
        if (static_cast<unsigned char>(ch) < 0x20) {
          escaped << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                  << static_cast<int>(static_cast<unsigned char>(ch))
                  << std::dec << std::setfill(' ');
        } else {
          escaped << ch;
        }
        break;
    }
  }
  return escaped.str();
}

#ifdef LSYSTEM_PLUGIN
std::shared_ptr<ScotsPineDescriptor> ResolveDescriptor(const std::filesystem::path& descriptor_path) {
  if (descriptor_path.empty()) {
    return AssetManager::CreateTemporaryAsset<ScotsPineDescriptor>();
  }

  if (descriptor_path.is_relative()) {
    const auto assets_candidate = ProjectManager::GetAssetsFolderPath() / descriptor_path;
    if (std::filesystem::exists(assets_candidate)) {
      auto descriptor =
          std::dynamic_pointer_cast<ScotsPineDescriptor>(ProjectManager::GetOrCreateAsset(descriptor_path));
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
      auto descriptor =
          std::dynamic_pointer_cast<ScotsPineDescriptor>(ProjectManager::GetOrCreateAsset(relative_path));
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

float ResolveTargetGddForSeed(const ScotsPineDescriptor& descriptor,
                              const uint32_t seed,
                              const float override_target_gdd) {
  if (override_target_gdd >= 0.0f) {
    return std::max(0.0f, override_target_gdd);
  }
  std::mt19937 rng(seed);
  return std::max(0.0f, SampleDistribution(descriptor.target_gdd, rng));
}

void GatherNeedleEntities(const std::shared_ptr<Scene>& scene,
                         const Entity pine_entity,
                         Entity& needle_entity,
                         Entity& needle_geometry_entity) {
  const auto inspect_children = [&](const Entity parent) {
    for (const auto& child : scene->GetChildren(parent)) {
      const auto name = scene->GetEntityName(child);
      if (StartsWith(name, "Pine Needles Geometry")) {
        needle_geometry_entity = child;
      } else if (StartsWith(name, "Pine Needles")) {
        needle_entity = child;
      }
    }
  };

  inspect_children(pine_entity);
  for (const auto& child : scene->GetChildren(pine_entity)) {
    const auto name = scene->GetEntityName(child);
    if (StartsWith(name, "Pine Needles Container")) {
      inspect_children(child);
    }
  }
}

void CaptureNeedleColorStats(const std::shared_ptr<Scene>& scene,
                             const Entity candidate,
                             const bool prefer_vertex_colors,
                             TreeReport& report) {
  if (!scene->IsEntityValid(candidate)) {
    return;
  }

  const bool has_mesh_renderer = scene->HasPrivateComponent<MeshRenderer>(candidate);
  const bool has_particles = scene->HasPrivateComponent<Particles>(candidate);
  if (!has_mesh_renderer && !has_particles) {
    return;
  }

  std::size_t sample_count = 0u;
  glm::vec3 color_sum(0.0f);
  glm::vec3 material_albedo(1.0f);
  bool use_material_albedo = false;

  std::shared_ptr<Mesh> mesh;
  std::shared_ptr<Material> material;
  std::shared_ptr<ParticleInfoList> particle_info_list;

  if (has_mesh_renderer) {
    if (const auto mesh_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(candidate).lock()) {
      report.needle_uses_mesh_renderer = true;
      mesh = mesh_renderer->mesh.Get<Mesh>();
      material = mesh_renderer->material.Get<Material>();
    }
  }

  if (!mesh && has_particles) {
    if (const auto particles = scene->GetOrSetPrivateComponent<Particles>(candidate).lock()) {
      mesh = particles->mesh.Get<Mesh>();
      if (!material) {
        material = particles->material.Get<Material>();
      }
      if (const auto list = particles->particle_info_list.Get<ParticleInfoList>()) {
        particle_info_list = list;
        report.needle_particle_info_list_ptr = reinterpret_cast<uintptr_t>(particle_info_list.get());
        report.needle_particle_info_list_handle = particle_info_list->GetHandle().GetValue();
        report.needle_particle_info_list_version = particle_info_list->GetVersion();
      }
    }
  }

  if (mesh) {
    report.needle_mesh_ptr = reinterpret_cast<uintptr_t>(mesh.get());
    report.needle_mesh_handle = mesh->GetHandle().GetValue();
    report.needle_mesh_version = mesh->GetVersion();
    const auto& vertices = mesh->UnsafeGetVertices();
    const auto& triangles = mesh->UnsafeGetTriangles();
    report.needle_vertex_count = vertices.size();
    report.needle_triangle_count = triangles.size();
    if (prefer_vertex_colors && !vertices.empty()) {
      for (const auto& vertex : vertices) {
        if (!IsFiniteVec4(vertex.color)) continue;
        color_sum += glm::vec3(vertex.color);
        sample_count++;
      }
    }
  }

  if (particle_info_list && (!prefer_vertex_colors || sample_count == 0u)) {
    const auto& particle_infos = particle_info_list->PeekParticleInfoList();
    for (const auto& info : particle_infos) {
      if (!IsFiniteVec4(info.instance_color)) continue;
      color_sum += glm::vec3(info.instance_color);
      sample_count++;
    }
  }

  if (material) {
    report.needle_material_ptr = reinterpret_cast<uintptr_t>(material.get());
    report.needle_material_handle = material->GetHandle().GetValue();
    report.needle_material_version = material->GetVersion();
    material_albedo = material->material_properties.albedo_color;
    use_material_albedo = material->vertex_color_only;
  }

  if (sample_count > 0u) {
    report.needle_color_sample_count = sample_count;
    glm::vec3 mean_color = color_sum / static_cast<float>(sample_count);
    if (use_material_albedo) {
      mean_color *= glm::clamp(material_albedo, glm::vec3(0.0f), glm::vec3(1.0f));
    }
    report.needle_mean_color = mean_color;
  }
}

void WriteJsonReport(const Options& options,
                     const std::vector<TreeReport>& tree_reports,
                     const std::vector<std::string>& failures,
                     const float max_pairwise_color_distance) {
  if (options.output_json.empty()) {
    return;
  }

  std::filesystem::create_directories(options.output_json.parent_path());
  std::ofstream out(options.output_json.string(), std::ios::trunc);
  if (!out.is_open()) {
    EVOENGINE_WARNING("Failed to open output JSON: " + options.output_json.string());
    return;
  }

  out << std::fixed << std::setprecision(6);
  out << "{\n";
  out << "  \"tree_count\": " << options.tree_count << ",\n";
  out << "  \"seed_start\": " << options.seed_start << ",\n";
  out << "  \"seed_step\": " << options.seed_step << ",\n";
  out << "  \"target_gdd_override\": " << options.target_gdd << ",\n";
  out << "  \"min_needles_per_tree\": " << options.min_needles_per_tree << ",\n";
  out << "  \"max_color_distance_threshold\": " << options.max_color_distance << ",\n";
  out << "  \"max_pairwise_color_distance\": " << max_pairwise_color_distance << ",\n";
  out << "  \"status\": \"" << (failures.empty() ? "pass" : "fail") << "\",\n";

  out << "  \"failures\": [\n";
  for (size_t i = 0; i < failures.size(); i++) {
    out << "    \"" << JsonEscape(failures[i]) << "\"";
    if (i + 1 < failures.size()) out << ",";
    out << "\n";
  }
  out << "  ],\n";

  out << "  \"trees\": [\n";
  for (size_t i = 0; i < tree_reports.size(); i++) {
    const auto& report = tree_reports[i];
    out << "    {\n";
    out << "      \"tree_index\": " << report.tree_index << ",\n";
    out << "      \"entity_index\": " << report.entity_index << ",\n";
    out << "      \"seed\": " << report.seed << ",\n";
    out << "      \"target_gdd\": " << report.target_gdd << ",\n";
    out << "      \"internode_count\": " << report.internode_count << ",\n";
    out << "      \"needle_count\": " << report.needle_count << ",\n";
    out << "      \"invalid_instance_count\": " << report.invalid_instance_count << ",\n";
    out << "      \"has_needle_entity\": " << (report.has_needle_entity ? "true" : "false") << ",\n";
    out << "      \"has_needle_geometry_entity\": "
        << (report.has_needle_geometry_entity ? "true" : "false") << ",\n";
    out << "      \"needle_uses_mesh_renderer\": "
      << (report.needle_uses_mesh_renderer ? "true" : "false") << ",\n";
    out << "      \"needle_vertex_count\": " << report.needle_vertex_count << ",\n";
    out << "      \"needle_triangle_count\": " << report.needle_triangle_count << ",\n";
    out << "      \"needle_color_sample_count\": " << report.needle_color_sample_count << ",\n";
    out << "      \"needle_mean_color\": ["
        << report.needle_mean_color.x << ", "
        << report.needle_mean_color.y << ", "
        << report.needle_mean_color.z << "],\n";
    out << "      \"needle_geom_entity_index\": " << report.needle_geom_entity_index << ",\n";
    out << "      \"needle_mesh_ptr\": \"0x"
        << std::hex << report.needle_mesh_ptr << std::dec << "\",\n";
    out << "      \"needle_material_ptr\": \"0x"
        << std::hex << report.needle_material_ptr << std::dec << "\",\n";
    out << "      \"needle_particle_info_list_ptr\": \"0x"
        << std::hex << report.needle_particle_info_list_ptr << std::dec << "\",\n";
    out << "      \"needle_mesh_handle\": " << report.needle_mesh_handle << ",\n";
    out << "      \"needle_material_handle\": " << report.needle_material_handle << ",\n";
    out << "      \"needle_particle_info_list_handle\": " << report.needle_particle_info_list_handle << ",\n";
    out << "      \"needle_mesh_version\": " << report.needle_mesh_version << ",\n";
    out << "      \"needle_material_version\": " << report.needle_material_version << ",\n";
    out << "      \"needle_particle_info_list_version\": " << report.needle_particle_info_list_version << "\n";
    out << "    }";
    if (i + 1 < tree_reports.size()) out << ",";
    out << "\n";
  }
  out << "  ]\n";
  out << "}\n";
}

std::shared_ptr<Scene> ResolveSmokeSceneAsset(const std::filesystem::path& baseline_scene_path,
                                              const std::filesystem::path& smoke_scene_path) {
  // Mirror MaizeDataGeneratorApp::ResolveGeneratorSceneAsset so the smoke harness never
  // mutates the user's editor scene. Clone the baseline once into a dedicated asset and
  // attach the clone instead.
  const std::filesystem::path smoke_scene_absolute_path =
      ProjectManager::GetAssetsFolderPath() / smoke_scene_path;

  if (!std::filesystem::exists(smoke_scene_absolute_path)) {
    const auto baseline_scene =
        std::dynamic_pointer_cast<Scene>(ProjectManager::GetOrCreateAsset(baseline_scene_path));
    if (!baseline_scene) {
      EVOENGINE_ERROR("Unable to resolve baseline scene asset: " + baseline_scene_path.string());
      return nullptr;
    }

    const auto cloned_scene = AssetManager::CreateTemporaryAsset<Scene>();
    Scene::Clone(baseline_scene, cloned_scene);

    if (!cloned_scene->SetPathAndSave(smoke_scene_path)) {
      EVOENGINE_ERROR("Failed to create isolated smoke scene; falling back to baseline scene.");
      return baseline_scene;
    }

    EVOENGINE_LOG("Created isolated Scots pine smoke scene asset: " + smoke_scene_path.string());
  }

  const auto smoke_scene =
      std::dynamic_pointer_cast<Scene>(ProjectManager::GetOrCreateAsset(smoke_scene_path));
  if (!smoke_scene) {
    EVOENGINE_ERROR("Unable to resolve isolated smoke scene asset: " + smoke_scene_path.string());
  }
  return smoke_scene;
}
#endif

}  // namespace

int main(int argc, char** argv) {
#ifndef LSYSTEM_PLUGIN
  EVOENGINE_ERROR("ScotsPineSmokeApp requires LSYSTEM_PLUGIN.");
  return 1;
#else
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

  Application::PushLayer<RenderLayer>("Render Layer");
  Application::PushLayer<LSystemLayer>("LSystem Layer");

  ApplicationInitializationSettings application_info{};
  application_info.application_name = "ScotsPineSmokeApp";
  application_info.project_path = options.project_path;
  Application::Initialize(application_info);

  const auto target_scene = ResolveSmokeSceneAsset(options.scene_path, options.smoke_scene_path);
  if (!target_scene) {
    EVOENGINE_ERROR("Failed to resolve isolated smoke scene asset (baseline: " +
                    options.scene_path.string() + ", smoke: " + options.smoke_scene_path.string() + ")");
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

  const auto descriptor = ResolveDescriptor(options.descriptor_path);
  if (!descriptor) {
    EVOENGINE_ERROR("Unable to resolve Scots pine descriptor.");
    Application::Terminate();
    return 6;
  }

  ScotsPine::SetGlobalColorMode(ScotsPine::ColorMode::Shaded);

  const auto batch_entity = scene->CreateEntity("Scots Pine Smoke Batch");
  // Layout: --multi-tree-grid R C uses an R x C grid centered at the origin
  // (matches the descriptor's Grid Instantiate path). Otherwise fall back to a
  // 1 x N row centered on X.
  const bool use_grid_layout = options.grid_rows > 0u && options.grid_cols > 0u;
  const float row_offset = (static_cast<float>(use_grid_layout ? options.grid_rows : 1u) - 1.0f)
                           * options.spacing * 0.5f;
  const float col_offset = (static_cast<float>(use_grid_layout ? options.grid_cols : options.tree_count) - 1.0f)
                           * options.spacing * 0.5f;
  const float offset = (static_cast<float>(options.tree_count) - 1.0f) * options.spacing * 0.5f;

  std::vector<TreeReport> tree_reports;
  tree_reports.reserve(options.tree_count);
  // Iteration B/D probe: keep per-pine handles around so we can drive
  // RebuildGeometry across multiple frames after the initial generate.
  struct PineSlot {
    Entity tree_entity;
    std::shared_ptr<ScotsPine> pine;
  };
  std::vector<PineSlot> pine_slots;
  pine_slots.reserve(options.tree_count);

  // Capture per-tree mesh/material/PIL state into a TreeReport. Used both for
  // the initial generate snapshot and for each rebuild-cycle snapshot.
  const auto capture_report =
      [&](const uint32_t tree_index, const uint32_t seed, const float target_gdd,
          const Entity tree_entity, const std::shared_ptr<ScotsPine>& pine) -> TreeReport {
    TreeReport report;
    report.tree_index = tree_index;
    report.entity_index = tree_entity.GetIndex();
    report.seed = seed;
    report.target_gdd = target_gdd;
    report.internode_count = pine->last_internode_count;
    report.needle_count = pine->last_needle_count;
    report.invalid_instance_count = pine->last_invalid_instance_count;

    Entity needle_entity;
    Entity needle_geometry_entity;
    GatherNeedleEntities(scene, tree_entity, needle_entity, needle_geometry_entity);
    report.has_needle_entity = scene->IsEntityValid(needle_entity);
    report.has_needle_geometry_entity = scene->IsEntityValid(needle_geometry_entity);
    if (report.has_needle_geometry_entity) {
      report.needle_geom_entity_index = needle_geometry_entity.GetIndex();
    } else if (report.has_needle_entity) {
      report.needle_geom_entity_index = needle_entity.GetIndex();
    }
    if (report.has_needle_geometry_entity) {
      CaptureNeedleColorStats(scene, needle_geometry_entity, true, report);
    }
    if (report.needle_color_sample_count == 0u && report.has_needle_entity) {
      CaptureNeedleColorStats(scene, needle_entity, false, report);
    }
    return report;
  };

  for (uint32_t i = 0u; i < options.tree_count; i++) {
    const uint32_t seed = options.seed_start + i * options.seed_step;
    const float target_gdd = ResolveTargetGddForSeed(*descriptor, seed, options.target_gdd);

    const auto tree_entity = scene->CreateEntity("Scots Pine Smoke " + std::to_string(i));
    scene->SetParent(tree_entity, batch_entity, false);

    Transform transform;
    if (use_grid_layout) {
      const uint32_t row = i / options.grid_cols;
      const uint32_t col = i % options.grid_cols;
      transform.SetPosition(glm::vec3(
          0.0f,
          static_cast<float>(row) * options.spacing - row_offset,
          static_cast<float>(col) * options.spacing - col_offset));
    } else {
      transform.SetPosition(glm::vec3(static_cast<float>(i) * options.spacing - offset, 0.0f, 0.0f));
    }
    scene->SetDataComponent(tree_entity, transform);

    const auto pine = scene->GetOrSetPrivateComponent<ScotsPine>(tree_entity).lock();
    if (!pine) {
      EVOENGINE_WARNING("Failed to create ScotsPine component for tree index " + std::to_string(i));
      continue;
    }

    pine->descriptor_ref = descriptor;
    pine->seed = seed;
    pine->target_gdd = target_gdd;
    pine->GenerateGeometryEntities(options.uncapped_growth);

    pine_slots.push_back({tree_entity, pine});

    const auto report = capture_report(i, seed, target_gdd, tree_entity, pine);
    EVOENGINE_LOG("[tree " + std::to_string(i) + " gen0] seed=" + std::to_string(seed) +
                  " target_gdd=" + std::to_string(target_gdd) +
                  " internodes=" + std::to_string(report.internode_count) +
                  " needles=" + std::to_string(report.needle_count) +
                  " mesh_verts=" + std::to_string(report.needle_vertex_count) +
                  " mesh_tris=" + std::to_string(report.needle_triangle_count) +
                  " color_samples=" + std::to_string(report.needle_color_sample_count) +
                  " geom_entity_idx=" + std::to_string(report.needle_geom_entity_index) +
                  " pil_handle=" + std::to_string(report.needle_particle_info_list_handle));

    tree_reports.emplace_back(report);
  }

  // Iteration B/D probe: drive a rebuild stress loop. Each cycle:
  //   1) Tick one Application::Loop (so the renderer consumes the previous
  //      frame's geometry — this is where async upload races would surface).
  //   2) Call RebuildGeometry on every pine in turn (mirrors LSystemLayer's
  //      RebuildAllPlantGeometry that fires on Ctrl+W and on global ColorMode
  //      toggles).
  //   3) Re-capture per-tree mesh/material/PIL state and OVERWRITE the latest
  //      snapshot in tree_reports so failures reflect the most recent state.
  //      Any cycle that drops needle_vertex_count to 0 for a tree that
  //      previously had needles is captured directly via per-cycle assertions.
  std::vector<std::string> rebuild_failures;
  for (uint32_t cycle = 1u; cycle <= options.rebuild_cycles; cycle++) {
    Application::Loop();
    for (size_t s = 0; s < pine_slots.size(); s++) {
      auto& slot = pine_slots[s];
      if (!scene->IsEntityValid(slot.tree_entity) || !slot.pine) continue;
      slot.pine->RebuildGeometry();
    }
    Application::Loop();
    for (size_t s = 0; s < pine_slots.size(); s++) {
      auto& slot = pine_slots[s];
      if (!scene->IsEntityValid(slot.tree_entity) || !slot.pine) continue;
      const uint32_t seed = options.seed_start + static_cast<uint32_t>(s) * options.seed_step;
      const float target_gdd = ResolveTargetGddForSeed(*descriptor, seed, options.target_gdd);
      const auto previous = tree_reports[s];
      const auto cycle_report =
          capture_report(static_cast<uint32_t>(s), seed, target_gdd, slot.tree_entity, slot.pine);
      EVOENGINE_LOG("[tree " + std::to_string(s) + " gen" + std::to_string(cycle) + "] " +
                    "needles=" + std::to_string(cycle_report.needle_count) +
                    " mesh_verts=" + std::to_string(cycle_report.needle_vertex_count) +
                    " mesh_tris=" + std::to_string(cycle_report.needle_triangle_count) +
                    " geom_entity_idx=" + std::to_string(cycle_report.needle_geom_entity_index) +
                    " pil_handle=" + std::to_string(cycle_report.needle_particle_info_list_handle));
      // Per-cycle regression: a tree that had needle_count > 0 before but
      // ends a cycle with 0 vertices is the missing-needle bug in flight.
      if (previous.needle_count > 0u && cycle_report.needle_vertex_count == 0u) {
        rebuild_failures.emplace_back("tree[" + std::to_string(s) + "] cycle " +
                                      std::to_string(cycle) +
                                      " lost needle mesh (was vert=" +
                                      std::to_string(previous.needle_vertex_count) +
                                      ", now 0)");
      }
      tree_reports[s] = cycle_report;
    }
  }

  std::vector<std::string> failures;

  // Carry forward any per-cycle regressions caught during the rebuild stress
  // loop. These are the strongest evidence that the bug repro'd headlessly.
  for (auto& f : rebuild_failures) failures.emplace_back(std::move(f));

  for (const auto& report : tree_reports) {
    if (report.needle_count < options.min_needles_per_tree) {
      failures.emplace_back("tree[" + std::to_string(report.tree_index) + "] needle_count=" +
                            std::to_string(report.needle_count) + " below minimum=" +
                            std::to_string(options.min_needles_per_tree));
    }

    // Iteration 1a: even when needle_count > 0, the rendered mesh must carry
    // vertices and triangles; missing-needle bug shows needle_count > 0 with
    // empty rendered mesh on at least one tree.
    if (report.needle_count > 0u && report.needle_vertex_count == 0u) {
      failures.emplace_back("tree[" + std::to_string(report.tree_index) +
                            "] needle_count=" + std::to_string(report.needle_count) +
                            " but rendered mesh has 0 vertices");
    }
    if (report.needle_count > 0u && report.needle_triangle_count == 0u) {
      failures.emplace_back("tree[" + std::to_string(report.tree_index) +
                            "] needle_count=" + std::to_string(report.needle_count) +
                            " but rendered mesh has 0 triangles");
    }

    if (report.needle_count > 0u && report.needle_color_sample_count == 0u) {
      failures.emplace_back("tree[" + std::to_string(report.tree_index) +
                            "] has needles but no color samples");
    }

    if (report.needle_color_sample_count > 0u) {
      const float luminance =
          0.2126f * report.needle_mean_color.x +
          0.7152f * report.needle_mean_color.y +
          0.0722f * report.needle_mean_color.z;
      if (luminance < 0.02f) {
        failures.emplace_back("tree[" + std::to_string(report.tree_index) +
                              "] needle mean color is near black (luma=" +
                              std::to_string(luminance) + ")");
      }
    }
  }

  std::unordered_map<uintptr_t, uint32_t> seen_mesh_ptrs;
  for (const auto& report : tree_reports) {
    if (report.needle_mesh_ptr == 0u) continue;
    const auto [it, inserted] = seen_mesh_ptrs.emplace(report.needle_mesh_ptr, report.tree_index);
    if (!inserted) {
      failures.emplace_back("shared needle mesh pointer between tree[" + std::to_string(it->second) +
                            "] and tree[" + std::to_string(report.tree_index) + "]");
    }
  }

  std::unordered_map<uintptr_t, uint32_t> seen_material_ptrs;
  for (const auto& report : tree_reports) {
    if (report.needle_material_ptr == 0u) continue;
    const auto [it, inserted] = seen_material_ptrs.emplace(report.needle_material_ptr, report.tree_index);
    if (!inserted) {
      failures.emplace_back("shared needle material pointer between tree[" + std::to_string(it->second) +
                            "] and tree[" + std::to_string(report.tree_index) + "]");
    }
  }

  // Iteration 1c: ParticleInfoList pointer uniqueness across trees. A shared
  // ParticleInfoList means two pines share the GPU range_descriptor slot in
  // GeometryStorage::particle_info_list_data_list_, which is a strong
  // candidate for the missing-needle / wrong-color symptom.
  std::unordered_map<uintptr_t, uint32_t> seen_pil_ptrs;
  for (const auto& report : tree_reports) {
    if (report.needle_particle_info_list_ptr == 0u) continue;
    const auto [it, inserted] =
        seen_pil_ptrs.emplace(report.needle_particle_info_list_ptr, report.tree_index);
    if (!inserted) {
      failures.emplace_back("shared needle ParticleInfoList pointer between tree[" +
                            std::to_string(it->second) + "] and tree[" +
                            std::to_string(report.tree_index) + "]");
    }
  }

  // Iteration C diagnostic: also assert IAsset handle uniqueness (cheap, and
  // catches asset-handle recycling that pointer comparison can miss when an
  // asset is destroyed and its address reused).
  std::unordered_map<uint64_t, uint32_t> seen_mesh_handles;
  for (const auto& report : tree_reports) {
    if (report.needle_mesh_handle == 0u) continue;
    const auto [it, inserted] = seen_mesh_handles.emplace(report.needle_mesh_handle, report.tree_index);
    if (!inserted) {
      failures.emplace_back("shared needle mesh IAsset handle between tree[" +
                            std::to_string(it->second) + "] and tree[" +
                            std::to_string(report.tree_index) + "]");
    }
  }
  std::unordered_map<uint64_t, uint32_t> seen_pil_handles;
  for (const auto& report : tree_reports) {
    if (report.needle_particle_info_list_handle == 0u) continue;
    const auto [it, inserted] =
        seen_pil_handles.emplace(report.needle_particle_info_list_handle, report.tree_index);
    if (!inserted) {
      failures.emplace_back("shared needle ParticleInfoList IAsset handle between tree[" +
                            std::to_string(it->second) + "] and tree[" +
                            std::to_string(report.tree_index) + "]");
    }
  }

  // Iteration A diagnostic: needle geometry-entity index uniqueness across
  // pines. If two ScotsPine entities resolve to the same Pine Needles
  // Geometry child (stale handle / sibling-explosion miss), one tree's
  // Particles overwrites the other.
  std::unordered_map<uint32_t, uint32_t> seen_geom_entities;
  for (const auto& report : tree_reports) {
    if (report.needle_geom_entity_index == 0u) continue;
    const auto [it, inserted] =
        seen_geom_entities.emplace(report.needle_geom_entity_index, report.tree_index);
    if (!inserted) {
      failures.emplace_back("shared needle geometry-entity index between tree[" +
                            std::to_string(it->second) + "] and tree[" +
                            std::to_string(report.tree_index) + "]");
    }
  }

  float max_pairwise_color_distance = 0.0f;
  for (size_t i = 0; i < tree_reports.size(); i++) {
    const auto& a = tree_reports[i];
    if (a.needle_color_sample_count == 0u) continue;
    for (size_t j = i + 1; j < tree_reports.size(); j++) {
      const auto& b = tree_reports[j];
      if (b.needle_color_sample_count == 0u) continue;
      const float distance = glm::length(a.needle_mean_color - b.needle_mean_color);
      max_pairwise_color_distance = std::max(max_pairwise_color_distance, distance);
    }
  }

  if (tree_reports.size() > 1u && max_pairwise_color_distance > options.max_color_distance) {
    failures.emplace_back("max pairwise mean needle-color distance=" +
                          std::to_string(max_pairwise_color_distance) +
                          " exceeds threshold=" + std::to_string(options.max_color_distance));
  }

  WriteJsonReport(options, tree_reports, failures, max_pairwise_color_distance);

  if (!options.output_json.empty()) {
    EVOENGINE_LOG("[report-json] " + options.output_json.string());
  }

  if (failures.empty()) {
    EVOENGINE_LOG("[scotspine-smoke] PASS");
  } else {
    EVOENGINE_WARNING("[scotspine-smoke] FAIL count=" + std::to_string(failures.size()));
    for (const auto& failure : failures) {
      EVOENGINE_WARNING("  - " + failure);
    }
  }

  if (scene->IsEntityValid(batch_entity)) {
    scene->DeleteEntity(batch_entity);
  }

  Application::Terminate();
  return failures.empty() ? 0 : 10;
#endif
}
