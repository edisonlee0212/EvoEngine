// PlantFactory.cpp : This file contains the 'main' function. Program execution
// begins and ends there.
//
#include "RenderLayer.hpp"
#include "WindowLayer.hpp"
#include <Application.hpp>

#include "AssetManager.hpp"
#include "Camera.hpp"
#include "EditorLayer.hpp"
#ifdef CUDA_MODULE_SERVICE
#  include <CUDAModule.hpp>
#  include <RayTracerLayer.hpp>
#endif
#include "ClassRegistry.hpp"
#include "ImGuiLayer.hpp"
#include "LSystemRuleHelpers.hpp"
#include "Lights.hpp"
#include "Material.hpp"
#include "Mesh.hpp"
#include "MeshRenderer.hpp"
#include "Particles.hpp"
#include "PathUtils.hpp"
#include "Prefab.hpp"
#include "Scene.hpp"
#include "SorghumLS.hpp"
#include "SorghumLSDescriptor.hpp"
#include "Times.hpp"
#include "Texture2D.hpp"
#include "TransformGraph.hpp"

#include "ProjectManager.hpp"

#include <array>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <random>
#include <sstream>
#include <string>
#include <system_error>
#include <vector>

#ifdef EVOENGINE_WINDOWS
#  include <Windows.h>
#endif

using namespace evo_engine;
using namespace l_system_package;

void EngineSetup();

namespace {
struct DigitalAgricultureCommandLine {
  bool export_lsystem_blender_scene = false;
  bool blender_preserve_lsystem_state = false;
  std::filesystem::path project_path;
  std::filesystem::path rt_scene_path;
  std::filesystem::path blender_output =
      std::filesystem::path("out") / "exports" / "lsystem_sorghum_blender" / "lsystem_sorghum_adult.fbx";
};

DigitalAgricultureCommandLine ParseDigitalAgricultureCommandLine(const int argc, char** argv) {
  DigitalAgricultureCommandLine command_line;
  for (int arg_index = 1; arg_index < argc; ++arg_index) {
    const std::string argument = argv[arg_index] ? argv[arg_index] : "";
    if (argument == "--export-lsystem-blender-scene") {
      command_line.export_lsystem_blender_scene = true;
    } else if (argument == "--blender-preserve-lsystem-state") {
      command_line.blender_preserve_lsystem_state = true;
    } else if (argument == "--project") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument("--project requires a path.");
      }
      command_line.project_path = argv[++arg_index];
    } else if (argument == "--rt-scene") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument("--rt-scene requires an assets-relative scene path.");
      }
      command_line.rt_scene_path = argv[++arg_index];
    } else if (argument == "--blender-output") {
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument("--blender-output requires a path.");
      }
      command_line.blender_output = argv[++arg_index];
    } else {
      throw std::invalid_argument("Unknown DigitalAgricultureApp argument: " + argument);
    }
  }
  command_line.blender_output = std::filesystem::absolute(command_line.blender_output);
  if (!command_line.project_path.empty()) {
    command_line.project_path = std::filesystem::absolute(command_line.project_path);
  }
  return command_line;
}

void ConfigureRuntimePackageDllSearchPath(const char* executable_path) {
#ifdef EVOENGINE_WINDOWS
  if (!executable_path) {
    return;
  }
  const auto packages_path = std::filesystem::absolute(executable_path).parent_path() / "Packages";
  if (std::filesystem::exists(packages_path)) {
    SetDllDirectoryW(packages_path.wstring().c_str());
  }
#else
  (void)executable_path;
#endif
}

std::shared_ptr<Scene> LoadRtSceneOverride(Application& application, const std::filesystem::path& assets_relative_path) {
  if (assets_relative_path.empty()) {
    return application.GetActiveScene();
  }

  const auto asset = ProjectManager::GetOrCreateAsset(assets_relative_path);
  auto scene = std::dynamic_pointer_cast<Scene>(asset);
  if (!scene) {
    return {};
  }
  ProjectManager::SetStartScene(scene);
  application.Attach(scene);
  return application.GetActiveScene();
}

struct SorghumLsRtMeshStats {
  uint32_t leaf_meshes = 0;
  uint32_t leaf_vertices = 0;
  uint32_t leaf_triangles = 0;
  uint32_t internode_particle_renderers = 0;
  uint32_t internode_instances = 0;
  uint32_t internode_mesh_vertices = 0;
  uint32_t internode_mesh_triangles = 0;
};

struct SorghumLsGrowthStats {
  size_t plants = 0;
  uint32_t leaves = 0;
  uint32_t live_leaves = 0;
  uint32_t internodes = 0;
  uint32_t nodes = 0;
  uint32_t autogrow_frames = 0;
};

float SampleSorghumLsDescriptorTargetGdd(const std::shared_ptr<SorghumLS>& sorghum) {
  if (!sorghum) {
    return -1.0f;
  }
  const auto descriptor = sorghum->descriptor_ref.Get<SorghumLSDescriptor>();
  if (!descriptor) {
    return -1.0f;
  }
  std::mt19937 rng(sorghum->seed);
  return std::max(0.0f, SampleDistribution(descriptor->target_gdd, rng));
}

SorghumLsGrowthStats CollectSorghumLsGrowthStats(const std::shared_ptr<Scene>& scene) {
  SorghumLsGrowthStats stats;
  if (!scene) {
    return stats;
  }

  if (const auto* sorghum_entities_ptr = scene->UnsafeGetPrivateComponentOwnersList<SorghumLS>()) {
    const std::vector<Entity> sorghum_entities = *sorghum_entities_ptr;
    for (const auto& entity : sorghum_entities) {
      if (!scene->IsEntityValid(entity)) {
        continue;
      }
      const auto sorghum = scene->GetOrSetPrivateComponent<SorghumLS>(entity).lock();
      if (!sorghum) {
        continue;
      }
      stats.plants++;
      stats.leaves += sorghum->last_leaf_count;
      stats.live_leaves += sorghum->last_live_leaf_count;
      stats.internodes += sorghum->last_internode_count;
      stats.nodes += sorghum->last_node_count;
    }
  }
  return stats;
}

SorghumLsGrowthStats GrowSorghumLsPlantsToAdulthood(const std::shared_ptr<Scene>& scene) {
  if (!scene) {
    return {};
  }

  if (const auto* sorghum_entities_ptr = scene->UnsafeGetPrivateComponentOwnersList<SorghumLS>()) {
    const std::vector<Entity> sorghum_entities = *sorghum_entities_ptr;
    for (const auto& entity : sorghum_entities) {
      if (!scene->IsEntityValid(entity)) {
        continue;
      }
      const auto sorghum = scene->GetOrSetPrivateComponent<SorghumLS>(entity).lock();
      if (!sorghum) {
        continue;
      }
      const float sampled_target = SampleSorghumLsDescriptorTargetGdd(sorghum);
      if (sampled_target >= 0.0f) {
        sorghum->target_gdd = sampled_target;
      }
      sorghum->GenerateGeometryEntities(true);
    }
  }
  const auto stats = CollectSorghumLsGrowthStats(scene);
  if (stats.plants > 0) {
    TransformGraph::CalculateTransformGraphs(scene);
  }
  return stats;
}

SorghumLsGrowthStats GenerateSorghumLsPlantsForCurrentTargetGdd(const std::shared_ptr<Scene>& scene) {
  if (!scene) {
    return {};
  }

  if (const auto* sorghum_entities_ptr = scene->UnsafeGetPrivateComponentOwnersList<SorghumLS>()) {
    const std::vector<Entity> sorghum_entities = *sorghum_entities_ptr;
    for (const auto& entity : sorghum_entities) {
      if (!scene->IsEntityValid(entity)) {
        continue;
      }
      const auto sorghum = scene->GetOrSetPrivateComponent<SorghumLS>(entity).lock();
      if (sorghum) {
        sorghum->GenerateGeometryEntities(true);
      }
    }
  }
  const auto stats = CollectSorghumLsGrowthStats(scene);
  if (stats.plants > 0) {
    TransformGraph::CalculateTransformGraphs(scene);
  }
  return stats;
}

struct BlenderExportStats {
  uint32_t mesh_renderers = 0;
  uint32_t vertices = 0;
  uint32_t triangles = 0;
  uint32_t materials = 0;
  uint32_t textured_materials = 0;
};

struct ParticleRendererBakeStats {
  uint32_t particle_renderers = 0;
  uint32_t baked_mesh_renderers = 0;
  uint32_t instances = 0;
  uint32_t vertices = 0;
  uint32_t triangles = 0;
};

struct ParbarRestoreStats {
  uint32_t roots = 0;
  uint32_t mesh_renderers = 0;
  uint32_t sections = 0;
  uint32_t couplers = 0;
  uint32_t models = 0;
};

bool BlenderExportIsFiniteVec3(const glm::vec3& value) {
  return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

bool BlenderExportIsFiniteMat4(const glm::mat4& value) {
  for (int column = 0; column < 4; ++column) {
    for (int row = 0; row < 4; ++row) {
      if (!std::isfinite(value[column][row])) {
        return false;
      }
    }
  }
  return true;
}

uint32_t AppendParticleRendererToWorldMesh(const std::shared_ptr<Scene>& scene,
                                           const Entity& entity,
                                           std::vector<Vertex>& out_vertices,
                                           std::vector<glm::uvec3>& out_triangles) {
  if (!scene || !scene->IsEntityValid(entity) || !scene->HasPrivateComponent<Particles>(entity)) {
    return 0;
  }
  const auto particles = scene->GetOrSetPrivateComponent<Particles>(entity).lock();
  if (!particles) {
    return 0;
  }
  const auto source_mesh = particles->mesh.Get<Mesh>();
  const auto particle_info_list = particles->particle_info_list.Get<ParticleInfoList>();
  if (!source_mesh || !particle_info_list) {
    return 0;
  }
  const auto& source_vertices = source_mesh->PeekVertices();
  const auto& source_triangles = source_mesh->PeekTriangles();
  const auto& instances = particle_info_list->PeekParticleInfoList();
  if (source_vertices.empty() || source_triangles.empty() || instances.empty()) {
    return 0;
  }

  uint32_t baked_instances = 0;
  const auto entity_global_transform = scene->GetDataComponent<GlobalTransform>(entity);
  for (const auto& instance : instances) {
    const glm::mat4 world_transform = entity_global_transform.value * instance.instance_matrix.value;
    if (!BlenderExportIsFiniteMat4(world_transform)) {
      continue;
    }
    const glm::mat3 world_3x3(world_transform);
    glm::mat3 normal_transform(1.0f);
    const float det = glm::determinant(world_3x3);
    if (std::isfinite(det) && std::abs(det) > 1e-8f) {
      normal_transform = glm::transpose(glm::inverse(world_3x3));
    }

    const auto vertex_offset = static_cast<uint32_t>(out_vertices.size());
    out_vertices.reserve(out_vertices.size() + source_vertices.size());
    out_triangles.reserve(out_triangles.size() + source_triangles.size());
    for (const auto& source_vertex : source_vertices) {
      Vertex vertex = source_vertex;
      vertex.position = glm::vec3(world_transform * glm::vec4(source_vertex.position, 1.0f));
      const glm::vec3 normal = normal_transform * source_vertex.normal;
      if (BlenderExportIsFiniteVec3(normal) && glm::dot(normal, normal) > 1e-12f) {
        vertex.normal = glm::normalize(normal);
      }
      const glm::vec3 tangent = normal_transform * source_vertex.tangent;
      if (BlenderExportIsFiniteVec3(tangent) && glm::dot(tangent, tangent) > 1e-12f) {
        vertex.tangent = glm::normalize(tangent);
      }
      vertex.color = instance.instance_color;
      out_vertices.emplace_back(vertex);
    }
    for (const auto& source_triangle : source_triangles) {
      out_triangles.emplace_back(vertex_offset + source_triangle.x,
                                 vertex_offset + source_triangle.y,
                                 vertex_offset + source_triangle.z);
    }
    baked_instances++;
  }
  return baked_instances;
}

ParticleRendererBakeStats BakeParticleRenderersForBlenderExport(const std::shared_ptr<Scene>& scene) {
  ParticleRendererBakeStats stats;
  if (!scene) {
    return stats;
  }

  const auto* particle_entities_ptr = scene->UnsafeGetPrivateComponentOwnersList<Particles>();
  if (!particle_entities_ptr) {
    return stats;
  }
  TransformGraph::CalculateTransformGraphs(scene);

  const std::vector<Entity> particle_entities = *particle_entities_ptr;
  for (const auto& entity : particle_entities) {
    if (!scene->IsEntityValid(entity)) {
      continue;
    }
    const auto particles = scene->GetOrSetPrivateComponent<Particles>(entity).lock();
    if (!particles) {
      continue;
    }

    std::vector<Vertex> vertices;
    std::vector<glm::uvec3> triangles;
    const uint32_t baked_instances = AppendParticleRendererToWorldMesh(scene, entity, vertices, triangles);
    if (vertices.empty() || triangles.empty()) {
      continue;
    }

    const auto export_entity = scene->CreateEntity(scene->GetEntityName(entity) + " Export Mesh");
    scene->SetEntitySerializable(export_entity, false);
    auto export_mesh = AssetManager::CreateTemporaryAsset<Mesh>();
    VertexAttributes attributes{};
    attributes.normal = true;
    attributes.tangent = true;
    attributes.tex_coord = true;
    attributes.color = true;
    export_mesh->SetVertices(attributes, vertices, triangles);

    const auto renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(export_entity).lock();
    renderer->mesh = export_mesh;
    renderer->material = particles->material;
    renderer->cast_shadow = particles->cast_shadow;

    stats.particle_renderers++;
    stats.baked_mesh_renderers++;
    stats.instances += baked_instances;
    stats.vertices += static_cast<uint32_t>(vertices.size());
    stats.triangles += static_cast<uint32_t>(triangles.size());
  }

  if (stats.baked_mesh_renderers > 0) {
    TransformGraph::CalculateTransformGraphs(scene);
  }
  return stats;
}

std::filesystem::path BlenderExportManifestPath(const std::filesystem::path& output_path) {
  return output_path.parent_path() / (output_path.stem().string() + "_manifest.json");
}

std::string EscapeJsonString(const std::string& value) {
  std::ostringstream stream;
  for (const char c : value) {
    switch (c) {
      case '\\':
        stream << "\\\\";
        break;
      case '"':
        stream << "\\\"";
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
        stream << c;
        break;
    }
  }
  return stream.str();
}

bool CopyLeafVariantHeightTextureForBlenderExport(const std::filesystem::path& output_path,
                                                  std::filesystem::path& copied_path) {
  const std::array<std::filesystem::path, 3> candidates = {
      ProjectManager::GetAssetsFolderPath() / "SorghumLeafMaterials" / "ImageTestLeafVariants" / "atlas" /
          "sorghum_lsystem_leaf_variants_height.png",
      std::filesystem::current_path() / "Resources" / "DigitalAgricultureProject" / "Assets" /
          "SorghumLeafMaterials" / "ImageTestLeafVariants" / "atlas" / "sorghum_lsystem_leaf_variants_height.png",
      std::filesystem::current_path() / "Resources" / "LSystemProject" / "Assets" / "SorghumLeafMaterials" /
          "ImageTestLeafVariants" / "atlas" / "sorghum_lsystem_leaf_variants_height.png"};

  for (const auto& candidate : candidates) {
    if (!std::filesystem::exists(candidate)) {
      continue;
    }
    const auto texture_directory = output_path.parent_path() / "textures";
    std::error_code error;
    std::filesystem::create_directories(texture_directory, error);
    copied_path = texture_directory / candidate.filename();
    std::filesystem::copy_file(candidate, copied_path, std::filesystem::copy_options::overwrite_existing, error);
    return !error && std::filesystem::exists(copied_path);
  }
  copied_path.clear();
  return false;
}

void WriteBlenderExportManifest(const std::filesystem::path& output_path,
                                const std::shared_ptr<Scene>& scene,
                                const std::filesystem::path& source_scene_path,
                                const std::string& growth_mode,
                                const size_t root_entity_count,
                                const SorghumLsGrowthStats& growth_stats,
                                const ParbarRestoreStats& parbar_stats,
                                const ParticleRendererBakeStats& particle_bake_stats,
                                const BlenderExportStats& export_stats,
                                const std::filesystem::path& leaf_height_texture_path) {
  const auto manifest_path = BlenderExportManifestPath(output_path);
  std::ofstream manifest(manifest_path);
  manifest << "{\n";
  manifest << "  \"schema\": \"evoengine_lsystem_blender_export_manifest_v1\",\n";
  manifest << "  \"output\": \"" << EscapeJsonString(output_path.string()) << "\",\n";
  manifest << "  \"project\": \"" << EscapeJsonString(ProjectManager::GetProjectPath().string()) << "\",\n";
  manifest << "  \"source_scene\": \"" << EscapeJsonString(source_scene_path.generic_string()) << "\",\n";
  manifest << "  \"growth_mode\": \"" << EscapeJsonString(growth_mode) << "\",\n";
  manifest << "  \"scene_entities\": " << (scene ? scene->UnsafeGetAllEntities().size() : 0) << ",\n";
  manifest << "  \"root_entities\": " << root_entity_count << ",\n";
  manifest << "  \"sorghum_ls\": " << growth_stats.plants << ",\n";
  manifest << "  \"leaves\": " << growth_stats.leaves << ",\n";
  manifest << "  \"live_leaves\": " << growth_stats.live_leaves << ",\n";
  manifest << "  \"internodes\": " << growth_stats.internodes << ",\n";
  manifest << "  \"mesh_renderers\": " << export_stats.mesh_renderers << ",\n";
  manifest << "  \"vertices\": " << export_stats.vertices << ",\n";
  manifest << "  \"triangles\": " << export_stats.triangles << ",\n";
  manifest << "  \"materials\": " << export_stats.materials << ",\n";
  manifest << "  \"textured_materials\": " << export_stats.textured_materials << ",\n";
  manifest << "  \"particle_renderers_baked\": " << particle_bake_stats.particle_renderers << ",\n";
  manifest << "  \"baked_particle_mesh_renderers\": " << particle_bake_stats.baked_mesh_renderers << ",\n";
  manifest << "  \"baked_particle_instances\": " << particle_bake_stats.instances << ",\n";
  manifest << "  \"baked_particle_vertices\": " << particle_bake_stats.vertices << ",\n";
  manifest << "  \"baked_particle_triangles\": " << particle_bake_stats.triangles << ",\n";
  manifest << "  \"parbar_roots\": " << parbar_stats.roots << ",\n";
  manifest << "  \"parbar_mesh_renderers\": " << parbar_stats.mesh_renderers << ",\n";
  manifest << "  \"parbar_sections\": " << parbar_stats.sections << ",\n";
  manifest << "  \"parbar_couplers\": " << parbar_stats.couplers << ",\n";
  manifest << "  \"parbar_models\": " << parbar_stats.models << ",\n";
  manifest << "  \"leaf_height_texture\": \"" << EscapeJsonString(leaf_height_texture_path.string()) << "\"\n";
  manifest << "}\n";
}

enum class ParbarTemplateKind { Section, Coupler, Model, Panel };

struct ParbarRendererTemplate {
  AssetRef mesh;
  AssetRef material;
  bool cast_shadow = true;
  bool valid = false;

  [[nodiscard]] bool Valid() const {
    return valid;
  }
};

struct ParbarPartSpec {
  const char* wrapper_name = "";
  const char* mesh_name = "";
  ParbarTemplateKind template_kind = ParbarTemplateKind::Section;
  std::array<float, 16> wrapper_matrix{};
  std::array<float, 16> mesh_matrix{};
};

glm::mat4 MatrixFromColumnMajor(const std::array<float, 16>& values) {
  return {values[0],  values[1],  values[2],  values[3],  values[4],  values[5],  values[6],  values[7],
          values[8],  values[9],  values[10], values[11], values[12], values[13], values[14], values[15]};
}

const std::array<float, 16>& IdentityMatrixValues() {
  static constexpr std::array<float, 16> kIdentity = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
                                                     0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f};
  return kIdentity;
}

std::array<float, 16> SectionMatrix(const float x, const float y, const float z) {
  return {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, -0.0f, 1.0f, 0.0f,
          0.0f, -1.0f, -0.0f, 0.0f, x,    y,     z,    1.0f};
}

std::array<float, 16> CouplerMatrix(const float x, const float y, const float z) {
  return {1.165588f,  -0.017727f, 0.033585f,  0.0f, -0.040246f, -0.055090f, 1.367671f, 0.0f,
          -0.006561f, -0.467469f, -0.019023f, 0.0f, x,          y,          z,         1.0f};
}

std::array<float, 16> ModelMatrix(const float x, const float y, const float z) {
  return {1.655761f, 0.0f,       0.897237f,  0.0f, -1.432000f, -0.082945f, 2.642612f, 0.0f,
          0.034065f, -2.590958f, -0.062864f, 0.0f, x,          y,          z,         1.0f};
}

const std::array<float, 16>& BtxTiltedModelMatrix() {
  static constexpr std::array<float, 16> kMatrix = {1.882630f, 0.0f,       0.047753f,  0.0f,
                                                    -0.076214f, -0.082945f, 3.004698f, 0.0f,
                                                    0.001813f,  -2.590958f, -0.071478f, 0.0f,
                                                    -0.044758f, 3.550811f,  0.038350f, 1.0f};
  return kMatrix;
}

const std::array<float, 16>& PawagaTiltedModelMatrix() {
  static constexpr std::array<float, 16> kMatrix = {1.870955f, 0.0f,       -0.214723f, 0.0f,
                                                    0.342700f, -0.082945f, 2.986064f,  0.0f,
                                                    -0.008152f, -2.590958f, -0.071034f, 0.0f,
                                                    -0.044758f, 3.550811f,  0.038350f,  1.0f};
  return kMatrix;
}

const std::array<float, 16>& BtxPanelWrapperMatrix() {
  static constexpr std::array<float, 16> kMatrix = {0.970296f, 0.0f, -0.241922f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
                                                    0.241922f, 0.0f, 0.970296f,  0.0f, 0.0f, 0.0f, 0.0f, 1.0f};
  return kMatrix;
}

const std::array<float, 16>& PawagaPanelWrapperMatrix() {
  static constexpr std::array<float, 16> kMatrix = {0.945519f, 0.0f, -0.325568f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
                                                    0.325568f, 0.0f, 0.945519f,  0.0f, 0.0f, 0.0f, 0.0f, 1.0f};
  return kMatrix;
}

const std::array<float, 16>& PanelMeshMatrix() {
  static constexpr std::array<float, 16> kMatrix = {0.254339f, 0.0f,      -0.769120f, 0.0f,
                                                    0.0f,      0.810083f, 0.0f,       0.0f,
                                                    0.769120f, 0.0f,      0.254339f,  0.0f,
                                                    0.171909f, 4.140316f, -0.801582f, 1.0f};
  return kMatrix;
}

std::vector<ParbarPartSpec> BuildParbarPartSpecs(const bool pawaga) {
  const auto& identity = IdentityMatrixValues();
  std::vector<ParbarPartSpec> specs;
  specs.reserve(17);
  specs.push_back({"modular_metal_gutter_section", "modular_metal_gutter_section", ParbarTemplateKind::Section,
                   identity, SectionMatrix(0.326437f, 0.008259f, 0.169506f)});
  specs.push_back({"modular_metal_gutter_section.001", "modular_metal_gutter_section.001",
                   ParbarTemplateKind::Section, identity, SectionMatrix(0.326437f, 1.008806f, 0.169506f)});
  specs.push_back({"modular_metal_gutter_section.002", "modular_metal_gutter_section.002",
                   ParbarTemplateKind::Section, identity, SectionMatrix(0.326437f, 1.989414f, 0.169506f)});
  specs.push_back({"modular_metal_gutter_section.003", "modular_metal_gutter_section.003",
                   ParbarTemplateKind::Section, identity, SectionMatrix(0.326437f, 2.965054f, 0.169506f)});
  specs.push_back({"modular_metal_gutter_section.004", "modular_metal_gutter_section.004",
                   ParbarTemplateKind::Section, identity, SectionMatrix(0.326437f, 3.953114f, 0.169506f)});
  specs.push_back({"modular_metal_gutter_section.005", "modular_metal_gutter_section.005",
                   ParbarTemplateKind::Section, identity, SectionMatrix(0.350609f, -0.070498f, 2.795782f)});
  specs.push_back({"modular_metal_gutter_section.006", "modular_metal_gutter_section.006",
                   ParbarTemplateKind::Section, identity, SectionMatrix(0.350609f, 0.851299f, 2.795782f)});
  specs.push_back({"modular_metal_gutter_section.007", "modular_metal_gutter_section.007",
                   ParbarTemplateKind::Section, identity, SectionMatrix(0.350609f, 1.274331f, 2.795782f)});
  specs.push_back({"modular_metal_gutter_section.008", "modular_metal_gutter_section.008",
                   ParbarTemplateKind::Section, identity, SectionMatrix(0.319989f, 0.044320f, 6.039947f)});
  specs.push_back({"modular_metal_gutter_coupler", "modular_metal_gutter_coupler", ParbarTemplateKind::Coupler,
                   identity, CouplerMatrix(-0.030037f, 3.568088f, -0.022308f)});
  specs.push_back({"modular_metal_gutter_coupler.001", "modular_metal_gutter_coupler.001",
                   ParbarTemplateKind::Coupler, identity, CouplerMatrix(-0.030733f, 4.189209f, -0.050174f)});
  specs.push_back({"modular_metal_gutter_coupler.002", "modular_metal_gutter_coupler.002",
                   ParbarTemplateKind::Coupler, identity, CouplerMatrix(-0.005865f, 1.747144f, 2.603969f)});
  specs.push_back({"modular_metal_gutter_coupler.003", "modular_metal_gutter_coupler.003",
                   ParbarTemplateKind::Coupler, identity, CouplerMatrix(-0.036485f, 0.479896f, 5.848134f)});
  specs.push_back({"model", "model", ParbarTemplateKind::Model, identity,
                   ModelMatrix(-0.051207f, 0.464388f, 5.908792f)});
  specs.push_back({"model.001", "model.001", ParbarTemplateKind::Model, identity,
                   ModelMatrix(-0.020587f, 1.730051f, 2.664627f)});
  specs.push_back({"model.002", "model.002", ParbarTemplateKind::Model, identity,
                   pawaga ? PawagaTiltedModelMatrix() : BtxTiltedModelMatrix()});
  specs.push_back({"model.003", "model.003", ParbarTemplateKind::Panel,
                   pawaga ? PawagaPanelWrapperMatrix() : BtxPanelWrapperMatrix(), PanelMeshMatrix()});
  return specs;
}

std::vector<Entity> CollectDescendants(const std::shared_ptr<Scene>& scene, const Entity& root) {
  std::vector<Entity> descendants;
  if (!scene || !scene->IsEntityValid(root)) {
    return descendants;
  }
  std::vector<Entity> stack = scene->GetChildren(root);
  while (!stack.empty()) {
    const Entity entity = stack.back();
    stack.pop_back();
    if (!scene->IsEntityValid(entity)) {
      continue;
    }
    descendants.emplace_back(entity);
    const auto children = scene->GetChildren(entity);
    stack.insert(stack.end(), children.begin(), children.end());
  }
  return descendants;
}

bool StartsWith(const std::string& text, const std::string& prefix) {
  return text.rfind(prefix, 0) == 0;
}

ParbarRestoreStats CountExistingParbarRenderers(const std::shared_ptr<Scene>& scene,
                                                const std::vector<Entity>& descendants) {
  ParbarRestoreStats stats;
  for (const auto& descendant : descendants) {
    if (!scene->HasPrivateComponent<MeshRenderer>(descendant)) {
      continue;
    }
    const auto name = scene->GetEntityName(descendant);
    stats.mesh_renderers++;
    if (StartsWith(name, "modular_metal_gutter_section")) {
      stats.sections++;
    } else if (StartsWith(name, "modular_metal_gutter_coupler")) {
      stats.couplers++;
    } else {
      stats.models++;
    }
  }
  return stats;
}

void MaybeCaptureParbarTemplate(const std::shared_ptr<Scene>& scene, const Entity& entity,
                                ParbarRendererTemplate& section, ParbarRendererTemplate& coupler,
                                ParbarRendererTemplate& model, ParbarRendererTemplate& panel) {
  if (!scene->HasPrivateComponent<MeshRenderer>(entity)) {
    return;
  }
  const auto renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(entity).lock();
  if (!renderer) {
    return;
  }
  const ParbarRendererTemplate candidate{renderer->mesh, renderer->material, renderer->cast_shadow, true};
  const auto name = scene->GetEntityName(entity);
  if (!section.Valid() && StartsWith(name, "modular_metal_gutter_section")) {
    section = candidate;
  } else if (!coupler.Valid() && StartsWith(name, "modular_metal_gutter_coupler")) {
    coupler = candidate;
  } else if (!model.Valid() && StartsWith(name, "model.002")) {
    model = candidate;
  } else if (!panel.Valid() && StartsWith(name, "model.003")) {
    panel = candidate;
  }
}

const ParbarRendererTemplate& SelectParbarTemplate(const ParbarPartSpec& spec,
                                                   const ParbarRendererTemplate& section,
                                                   const ParbarRendererTemplate& coupler,
                                                   const ParbarRendererTemplate& model,
                                                   const ParbarRendererTemplate& panel) {
  switch (spec.template_kind) {
    case ParbarTemplateKind::Section:
      return section;
    case ParbarTemplateKind::Coupler:
      return coupler;
    case ParbarTemplateKind::Model:
      return model;
    case ParbarTemplateKind::Panel:
      return panel;
  }
  return section;
}

void SetLocalMatrix(const std::shared_ptr<Scene>& scene, const Entity& entity, const std::array<float, 16>& values) {
  Transform transform;
  transform.value = MatrixFromColumnMajor(values);
  scene->SetDataComponent(entity, transform);
}

bool CreateParbarPart(const std::shared_ptr<Scene>& scene, const Entity& root, const ParbarPartSpec& spec,
                      const ParbarRendererTemplate& renderer_template) {
  if (!renderer_template.Valid()) {
    return false;
  }

  const auto wrapper = scene->CreateEntity(spec.wrapper_name);
  scene->SetParent(wrapper, root, false);
  SetLocalMatrix(scene, wrapper, spec.wrapper_matrix);

  const auto mesh_entity = scene->CreateEntity(spec.mesh_name);
  scene->SetParent(mesh_entity, wrapper, false);
  SetLocalMatrix(scene, mesh_entity, spec.mesh_matrix);
  const auto renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(mesh_entity).lock();
  renderer->mesh = renderer_template.mesh;
  renderer->material = renderer_template.material;
  renderer->cast_shadow = renderer_template.cast_shadow;
  return true;
}

ParbarRestoreStats RestoreFullParbarDevices(const std::shared_ptr<Scene>& scene) {
  ParbarRestoreStats stats;
  if (!scene) {
    return stats;
  }

  std::vector<Entity> roots;
  for (const auto& entity : scene->UnsafeGetAllEntities()) {
    if (!scene->IsEntityValid(entity)) {
      continue;
    }
    const auto name = scene->GetEntityName(entity);
    if (scene->GetParent(entity).GetIndex() == 0 &&
        (StartsWith(name, "PARBAR_EvoEngineReady") || StartsWith(name, "PARBAR_BTX") ||
         StartsWith(name, "PARBAR_Pawaga"))) {
      roots.emplace_back(entity);
    }
  }
  if (roots.size() < 2) {
    return stats;
  }

  std::sort(roots.begin(), roots.end(), [&](const Entity& lhs, const Entity& rhs) {
    return scene->GetDataComponent<Transform>(lhs).GetPosition().z <
           scene->GetDataComponent<Transform>(rhs).GetPosition().z;
  });
  roots.resize(2);

  for (size_t root_index = 0; root_index < roots.size(); ++root_index) {
    const auto root = roots[root_index];
    const bool pawaga = root_index == 0;
    const auto descendants = CollectDescendants(scene, root);
    const auto existing_stats = CountExistingParbarRenderers(scene, descendants);
    if (existing_stats.mesh_renderers >= 17) {
      scene->SetEntityName(root, pawaga ? "PARBAR_Pawaga" : "PARBAR_BTX");
      stats.roots++;
      stats.mesh_renderers += existing_stats.mesh_renderers;
      stats.sections += existing_stats.sections;
      stats.couplers += existing_stats.couplers;
      stats.models += existing_stats.models;
      continue;
    }

    ParbarRendererTemplate section;
    ParbarRendererTemplate coupler;
    ParbarRendererTemplate model;
    ParbarRendererTemplate panel;
    for (const auto& descendant : descendants) {
      MaybeCaptureParbarTemplate(scene, descendant, section, coupler, model, panel);
    }
    if (!section.Valid() || !coupler.Valid() || !model.Valid() || !panel.Valid()) {
      continue;
    }

    const auto children = scene->GetChildren(root);
    for (const auto& child : children) {
      scene->DeleteEntity(child);
    }
    scene->SetEntityName(root, pawaga ? "PARBAR_Pawaga" : "PARBAR_BTX");

    const auto specs = BuildParbarPartSpecs(pawaga);
    for (const auto& spec : specs) {
      const auto& renderer_template = SelectParbarTemplate(spec, section, coupler, model, panel);
      if (CreateParbarPart(scene, root, spec, renderer_template)) {
        stats.mesh_renderers++;
        if (spec.template_kind == ParbarTemplateKind::Section) {
          stats.sections++;
        } else if (spec.template_kind == ParbarTemplateKind::Coupler) {
          stats.couplers++;
        } else {
          stats.models++;
        }
      }
    }
    stats.roots++;
  }
  if (stats.roots > 0) {
    TransformGraph::CalculateTransformGraphs(scene);
  }
  return stats;
}

BlenderExportStats CountBlenderExportStats(const std::shared_ptr<Scene>& scene) {
  BlenderExportStats stats;
  if (!scene) {
    return stats;
  }

  std::vector<std::shared_ptr<Material>> materials;
  for (const auto& entity : scene->UnsafeGetAllEntities()) {
    if (!scene->IsEntityValid(entity) || !scene->HasPrivateComponent<MeshRenderer>(entity)) {
      continue;
    }
    const auto renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(entity).lock();
    if (!renderer) {
      continue;
    }
    const auto mesh = renderer->mesh.Get<Mesh>();
    if (!mesh || mesh->GetTriangleAmount() == 0) {
      continue;
    }
    stats.mesh_renderers++;
    stats.vertices += mesh->GetVerticesAmount();
    stats.triangles += mesh->GetTriangleAmount();

    const auto material = renderer->material.Get<Material>();
    if (!material || std::find(materials.begin(), materials.end(), material) != materials.end()) {
      continue;
    }
    materials.emplace_back(material);
    if (material->GetAlbedoTexture() || material->GetNormalTexture() || material->GetMetallicTexture() ||
        material->GetRoughnessTexture() || material->GetAoTexture()) {
      stats.textured_materials++;
    }
  }
  stats.materials = static_cast<uint32_t>(materials.size());
  return stats;
}

bool LoopOrFailBlenderExport(Application& application, const std::string& reason) {
  if (application.Loop()) {
    return true;
  }
  std::cerr << "EVOENGINE_LSYSTEM_BLENDER_EXPORT_RESULT failed reason=\"" << reason << "\"" << std::endl;
  return false;
}

int FailLSystemBlenderExport(Application& application, const std::string& reason) {
  std::cerr << "EVOENGINE_LSYSTEM_BLENDER_EXPORT_RESULT failed reason=\"" << reason << "\"" << std::endl;
  application.End();
  return 1;
}

int FailLSystemBlenderExport(Application& application, const std::filesystem::path& output_path,
                             const std::string& reason) {
  std::filesystem::create_directories(output_path.parent_path());
  const auto error_path = output_path.parent_path() / (output_path.stem().string() + "_error.txt");
  std::ofstream error_report(error_path);
  error_report << "failed reason=\"" << reason << "\"\n";
  error_report.flush();
  return FailLSystemBlenderExport(application, reason);
}

std::vector<Entity> CollectSceneRootEntities(const std::shared_ptr<Scene>& scene) {
  std::vector<Entity> roots;
  if (!scene) {
    return roots;
  }
  for (const auto& entity : scene->UnsafeGetAllEntities()) {
    if (scene->IsEntityValid(entity) && scene->GetParent(entity).GetIndex() == 0) {
      roots.emplace_back(entity);
    }
  }
  return roots;
}

std::shared_ptr<Prefab> CreateBlenderExportPrefab(const std::shared_ptr<Scene>& scene,
                                                  const std::vector<Entity>& root_entities) {
  const auto export_root = scene->CreateEntity("Adult LSystem Sorghum Blender Export");
  scene->SetEntitySerializable(export_root, false);
  for (const auto& root : root_entities) {
    if (scene->IsEntityValid(root)) {
      scene->SetParent(root, export_root, true);
    }
  }
  TransformGraph::CalculateTransformGraphs(scene);

  auto prefab = AssetManager::CreateTemporaryAsset<Prefab>();
  prefab->FromEntity(export_root);
  return prefab;
}

int RunLSystemBlenderExportBatch(Application& application, const DigitalAgricultureCommandLine& command_line) {
  if (command_line.blender_output.extension() != ".gltf" && command_line.blender_output.extension() != ".fbx") {
    return FailLSystemBlenderExport(application, command_line.blender_output,
                                    "Blender export output must use the .gltf or .fbx extension");
  }

  application.Start(false);
  for (size_t frame = 0; !ProjectManager::IsProjectIdle(); ++frame) {
    if (!LoopOrFailBlenderExport(application, "application ended before project load completed")) {
      return FailLSystemBlenderExport(application, command_line.blender_output,
                                      "application ended before project load completed");
    }
    if (frame >= 30000) {
      return FailLSystemBlenderExport(application, command_line.blender_output, "project load timed out");
    }
  }

  const auto scene = LoadRtSceneOverride(application, command_line.rt_scene_path);
  if (!scene) {
    return FailLSystemBlenderExport(application, command_line.blender_output, "active scene is missing");
  }

  const std::string growth_mode =
      command_line.blender_preserve_lsystem_state ? "preserve_scene_lsystems" : "adult_descriptor_target";
  const auto growth_stats = command_line.blender_preserve_lsystem_state
                                ? GenerateSorghumLsPlantsForCurrentTargetGdd(scene)
                                : GrowSorghumLsPlantsToAdulthood(scene);
  if (growth_stats.plants == 0) {
    return FailLSystemBlenderExport(application, command_line.blender_output, "no SorghumLS plants were loaded");
  }
  const auto parbar_stats = RestoreFullParbarDevices(scene);
  const auto particle_bake_stats = BakeParticleRenderersForBlenderExport(scene);
  const auto root_entities = CollectSceneRootEntities(scene);

  const auto export_stats = CountBlenderExportStats(scene);
  if (export_stats.mesh_renderers == 0) {
    return FailLSystemBlenderExport(application, command_line.blender_output,
                                    "no mesh renderers were available for export");
  }

  std::filesystem::create_directories(command_line.blender_output.parent_path());
  const auto prefab = CreateBlenderExportPrefab(scene, root_entities);
  const bool exported = prefab && prefab->Export(command_line.blender_output);
  if (!exported || !std::filesystem::exists(command_line.blender_output)) {
    const auto error_path =
        command_line.blender_output.parent_path() / (command_line.blender_output.stem().string() + "_error.txt");
    std::ofstream error_report(error_path);
    error_report << "failed to export glTF scene\n";
    error_report << "prefab=" << static_cast<bool>(prefab) << "\n";
    error_report << "exported=" << exported << "\n";
    error_report << "exists=" << std::filesystem::exists(command_line.blender_output) << "\n";
    error_report << "sorghum_ls=" << growth_stats.plants << "\n";
    error_report << "leaves=" << growth_stats.leaves << "\n";
    error_report << "mesh_renderers=" << export_stats.mesh_renderers << "\n";
    error_report << "vertices=" << export_stats.vertices << "\n";
    error_report << "triangles=" << export_stats.triangles << "\n";
    error_report << "materials=" << export_stats.materials << "\n";
    error_report << "particle_renderers_baked=" << particle_bake_stats.particle_renderers << "\n";
    error_report << "baked_particle_mesh_renderers=" << particle_bake_stats.baked_mesh_renderers << "\n";
    error_report << "baked_particle_instances=" << particle_bake_stats.instances << "\n";
    error_report << "baked_particle_vertices=" << particle_bake_stats.vertices << "\n";
    error_report << "baked_particle_triangles=" << particle_bake_stats.triangles << "\n";
    error_report << "parbar_roots=" << parbar_stats.roots << "\n";
    error_report << "parbar_mesh_renderers=" << parbar_stats.mesh_renderers << "\n";
    error_report.flush();
    return FailLSystemBlenderExport(application, "failed to export glTF scene");
  }

  std::filesystem::path leaf_height_texture_path;
  const bool leaf_height_texture_copied =
      CopyLeafVariantHeightTextureForBlenderExport(command_line.blender_output, leaf_height_texture_path);
  WriteBlenderExportManifest(command_line.blender_output,
                             scene,
                             command_line.rt_scene_path,
                             growth_mode,
                             root_entities.size(),
                             growth_stats,
                             parbar_stats,
                             particle_bake_stats,
                             export_stats,
                             leaf_height_texture_path);

  const auto report_path =
      command_line.blender_output.parent_path() / (command_line.blender_output.stem().string() + ".txt");
  std::ofstream report(report_path);
  report << "output=" << command_line.blender_output.string() << "\n";
  report << "manifest=" << BlenderExportManifestPath(command_line.blender_output).string() << "\n";
  report << "source_scene=" << command_line.rt_scene_path.generic_string() << "\n";
  report << "growth_mode=" << growth_mode << "\n";
  report << "scene_entities=" << scene->UnsafeGetAllEntities().size() << "\n";
  report << "root_entities=" << root_entities.size() << "\n";
  report << "sorghum_ls=" << growth_stats.plants << "\n";
  report << "leaves=" << growth_stats.leaves << "\n";
  report << "live_leaves=" << growth_stats.live_leaves << "\n";
  report << "internodes=" << growth_stats.internodes << "\n";
  report << "mesh_renderers=" << export_stats.mesh_renderers << "\n";
  report << "vertices=" << export_stats.vertices << "\n";
  report << "triangles=" << export_stats.triangles << "\n";
  report << "materials=" << export_stats.materials << "\n";
  report << "textured_materials=" << export_stats.textured_materials << "\n";
  report << "particle_renderers_baked=" << particle_bake_stats.particle_renderers << "\n";
  report << "baked_particle_mesh_renderers=" << particle_bake_stats.baked_mesh_renderers << "\n";
  report << "baked_particle_instances=" << particle_bake_stats.instances << "\n";
  report << "baked_particle_vertices=" << particle_bake_stats.vertices << "\n";
  report << "baked_particle_triangles=" << particle_bake_stats.triangles << "\n";
  report << "parbar_roots=" << parbar_stats.roots << "\n";
  report << "parbar_mesh_renderers=" << parbar_stats.mesh_renderers << "\n";
  report << "parbar_sections=" << parbar_stats.sections << "\n";
  report << "parbar_couplers=" << parbar_stats.couplers << "\n";
  report << "parbar_models=" << parbar_stats.models << "\n";
  report << "leaf_height_texture_copied=" << leaf_height_texture_copied << "\n";
  report << "leaf_height_texture=" << leaf_height_texture_path.string() << "\n";

  application.End();
  std::cout << "EVOENGINE_LSYSTEM_BLENDER_EXPORT_RESULT passed output=\"" << command_line.blender_output.string()
            << "\" growth_mode=" << growth_mode
            << " sorghum_ls=" << growth_stats.plants << " leaves=" << growth_stats.leaves
            << " mesh_renderers=" << export_stats.mesh_renderers << " vertices=" << export_stats.vertices
            << " triangles=" << export_stats.triangles << " materials=" << export_stats.materials
            << " textured_materials=" << export_stats.textured_materials
            << " baked_particle_mesh_renderers=" << particle_bake_stats.baked_mesh_renderers
            << " baked_particle_instances=" << particle_bake_stats.instances
            << " parbar_roots=" << parbar_stats.roots
            << " parbar_mesh_renderers=" << parbar_stats.mesh_renderers << std::endl;
  return 0;
}

}  // namespace

int main(const int argc, char** argv) {
  Application application;
  bool initialized = false;
  bool automated_run = false;
  try {
    const auto command_line = ParseDigitalAgricultureCommandLine(argc, argv);
    auto resource_folder_path = path_utils::FindAncestorChildPath("Resources", std::filesystem::current_path(), 8);
    if (resource_folder_path.empty()) {
      resource_folder_path = path_utils::NormalizeAbsolutePath("Resources");
    }
    if (std::filesystem::exists(resource_folder_path)) {
      for (auto i : std::filesystem::recursive_directory_iterator(resource_folder_path)) {
        if (i.is_directory())
          continue;
        const auto& old_path = i.path();
        auto new_path = i.path();
        bool remove = false;
        if (i.path().extension().string() == ".uescene") {
          new_path.replace_extension(".evescene");
          remove = true;
        }
        if (i.path().extension().string() == ".umeta") {
          new_path.replace_extension(".evefilemeta");
          remove = true;
        }
        if (i.path().extension().string() == ".ueproj") {
          new_path.replace_extension(".eveproj");
          remove = true;
        }
        if (i.path().extension().string() == ".ufmeta") {
          new_path.replace_extension(".evefoldermeta");
          remove = true;
        }
        if (remove) {
          std::filesystem::copy(old_path, new_path);
          std::filesystem::remove(old_path);
        }
      }
    }
    const auto project_path_text = command_line.project_path.string();
    const bool explicit_lsystem_project =
        project_path_text.find("LSystemProject") != std::string::npos ||
        project_path_text.find("LSystemProjectAssets") != std::string::npos;
    const bool uses_lsystem_project = command_line.export_lsystem_blender_scene || explicit_lsystem_project;
    const auto selected_project_path =
        !command_line.project_path.empty()
            ? command_line.project_path
            : std::filesystem::absolute(resource_folder_path / "DigitalAgricultureProject" /
                                        (uses_lsystem_project ? "test_lsystem_sorghum.eveproj" : "test.eveproj"));

    ConfigureRuntimePackageDllSearchPath(argc > 0 ? argv[0] : nullptr);
    EngineSetup();
    const bool batch_scene_load = command_line.export_lsystem_blender_scene;
    ProjectManager::SetActionAfterSceneLoad([batch_scene_load](const std::shared_ptr<Scene>& scene) {
      const auto parbar_stats = RestoreFullParbarDevices(scene);
      if (parbar_stats.roots > 0) {
        std::cout << "EVOENGINE_PARBAR_SCENE_LOAD_RESTORE roots=" << parbar_stats.roots
                  << " mesh_renderers=" << parbar_stats.mesh_renderers << " sections=" << parbar_stats.sections
                  << " couplers=" << parbar_stats.couplers << " models=" << parbar_stats.models << std::endl;
      }
      if (batch_scene_load) {
        if (const auto editor_layer = ApplicationContext::Get().GetLayer<EditorLayer>()) {
          editor_layer->main_camera_allow_auto_resize = false;
        }
      }
    });

    ApplicationContext::Get().PushLayer<RenderLayer>("Render Layer");
#ifdef CUDA_MODULE_SERVICE
    if (!command_line.export_lsystem_blender_scene) {
      ApplicationContext::Get().PushLayer<RayTracerLayer>("Ray Tracer Layer");
    }
#endif
    ApplicationContext::Get().PushLayer<WindowLayer>("Window Layer");
    ApplicationContext::Get().PushLayer<ImGuiLayer>("ImGui Layer");
    ApplicationContext::Get().PushLayer<EditorLayer>("Editor Layer");

    ApplicationInitializationSettings application_configs;
    application_configs.application_name = uses_lsystem_project ? "LSystem" : "DigitalAgriculture";
    application_configs.full_screen = !command_line.export_lsystem_blender_scene;
    application_configs.project_path = selected_project_path;
    application_configs.enable_runtime_packages = true;
    application_configs.use_custom_title_bar = true;
    application_configs.startup_runtime_packages =
        uses_lsystem_project ? std::vector<std::string>{"LSystem", "DigitalAgriculture"}
                             : std::vector<std::string>{"DigitalAgriculture", "LSystem"};
    if (batch_scene_load) {
      application_configs.hide_console_window = false;
      application_configs.redirect_standard_streams_to_console = false;
      application_configs.load_project_assets =
          !command_line.export_lsystem_blender_scene || !command_line.rt_scene_path.empty();
      application_configs.load_project_start_scene = true;
    }
    ApplicationContext::Get().Initialize(application_configs);
    initialized = true;

#ifdef CUDA_MODULE_SERVICE

    auto ray_tracer_layer = ApplicationContext::Get().GetLayer<RayTracerLayer>();
    if (ray_tracer_layer) {
      ray_tracer_layer->enable_inspection = true;
      ray_tracer_layer->show_scene_window = true;
      ray_tracer_layer->environment_properties.environmental_lighting_type = EnvironmentalLightingType::Skydome;
    }
#endif

    automated_run = command_line.export_lsystem_blender_scene;
    int exit_code = 0;
    if (command_line.export_lsystem_blender_scene) {
      exit_code = RunLSystemBlenderExportBatch(application, command_line);
    } else {
      // adjust default camera speed
      const auto editor_layer = ApplicationContext::Get().GetLayer<EditorLayer>();
      if (editor_layer) {
        editor_layer->show_scene_window = true;
        editor_layer->velocity = 2.f;
        editor_layer->default_scene_camera_position = glm::vec3(1.124, 0.218, 14.089);
      }
#pragma region Engine Loop
      ApplicationContext::Get().Start();
      ApplicationContext::Get().Run();
#pragma endregion
    }
    ApplicationContext::Get().Terminate();
    if (automated_run) {
      std::cout.flush();
      std::cerr.flush();
      std::_Exit(exit_code);
    }
    return exit_code;
  } catch (const std::exception& e) {
    std::cerr << "EVOENGINE_DIGITAL_AGRICULTURE_APP_RESULT failed reason=\"" << e.what() << "\"" << std::endl;
    if (initialized) {
      ApplicationContext::Get().Terminate();
    }
    if (automated_run) {
      std::cout.flush();
      std::cerr.flush();
      std::_Exit(1);
    }
    return 1;
  }
}

void EngineSetup() {
  ProjectManager::SetActionAfterNewScene([=](const std::shared_ptr<Scene>& scene) {
#pragma region Engine Setup
    Transform transform;
    transform.SetEulerRotation(glm::radians(glm::vec3(150, 30, 0)));
#pragma region Preparations
    ApplicationContext::Get().GetTimes().SetTimeStep(0.016f);
    transform = Transform();
    transform.SetPosition(glm::vec3(0, 2, 35));
    transform.SetEulerRotation(glm::radians(glm::vec3(15, 0, 0)));
    if (const auto main_camera = ApplicationContext::Get().GetActiveScene()->main_camera.Get<Camera>()) {
      scene->SetDataComponent(main_camera->GetOwner(), transform);
      main_camera->camera_settings.use_clear_color = true;
      main_camera->camera_settings.clear_color = glm::vec4(0.5f, 0.5f, 0.5f, 1.f);
    }
#pragma endregion
#pragma endregion
  });
}
