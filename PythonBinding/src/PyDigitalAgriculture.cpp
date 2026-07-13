#include "PyDigitalAgriculture.hpp"
#include "Serialization.hpp"

#ifdef DATASET_GENERATION_PACKAGE
#  include "DatasetGenerationSerializationAdapters.hpp"
#endif

#include "AssetManager.hpp"
#include "BtfMeshRenderer.hpp"
#include "GeometryStorage.hpp"
#include "LSystemRuleHelpers.hpp"
#include "LSystemSerializationAdapters.hpp"
#include "Lights.hpp"
#include "Material.hpp"
#include "Mesh.hpp"
#include "MeshRenderer.hpp"
#include "PARSensorGroup.hpp"
#include "Particles.hpp"
#include "Prefab.hpp"
#include "ProjectManager.hpp"
#include "RayTracerCamera.hpp"
#include "SorghumCoordinates.hpp"
#include "SorghumLS.hpp"
#include "SorghumLSDescriptor.hpp"
#include "SorghumLeafMesh.hpp"
#include "TextureStorage.hpp"
#include "TransformGraph.hpp"
#include "TriangleIlluminationEstimator.hpp"
#ifdef CUDA_MODULE_SERVICE
#  include "OptiXRayTracer.hpp"
#  include "RayTracerLayer.hpp"
#endif

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <random>
#include <set>

#ifdef DIGITAL_AGRICULTURE_PACKAGE

namespace evo_engine {
class TriangleIlluminationEstimator;
}

namespace py = pybind11;
using namespace py_digital_agriculture_package;
using namespace l_system_package;
namespace {
template <typename T>
void RegisterSerializationHandler(const std::string& type_name) {
  Serialization::RegisterSerializationHandler<T>(
      [](YAML::Emitter& out, const T& target) {
        target.Serialize(out);
      },
      [](const YAML::Node& in, T& target) {
        target.Deserialize(in);
      },
      {}, type_name);
}

struct ParbarPanel {
  const char* cultivar;
  const char* root;
  const char* mesh_entity_name;
  const char* sensor_bar_level;
};

constexpr ParbarPanel kParbarPanels[] = {{"Pawaga", "PARBAR_Pawaga", "PARBAR_Pawaga_TopSensorBarMesh", "top"},
                                         {"BTX", "PARBAR_BTX", "PARBAR_BTX_TopSensorBarMesh", "top"},
                                         {"Pawaga", "PARBAR_Pawaga", "PARBAR_Pawaga_MiddleSensorBarMesh", "middle"},
                                         {"BTX", "PARBAR_BTX", "PARBAR_BTX_MiddleSensorBarMesh", "middle"},
                                         {"Pawaga", "PARBAR_Pawaga", "PARBAR_Pawaga_BottomSensorBarMesh", "bottom"},
                                         {"BTX", "PARBAR_BTX", "PARBAR_BTX_BottomSensorBarMesh", "bottom"}};
constexpr size_t kParbarPanelCount = sizeof(kParbarPanels) / sizeof(kParbarPanels[0]);
const std::filesystem::path kIlluminationSoilDescriptorPath = "ManualAssets/Soil/bigSoil.soil";
const std::filesystem::path kIlluminationGroundMeshPath = "ManualAssets/Context/Mesh_7895692069869720144.evemesh";
const std::filesystem::path kIlluminationSoilMaterialPath = "ManualAssets/Soil/PBR/soil_PBRV2.evematerial";
const std::filesystem::path kBtxReferenceDescriptorPath = "ManualAssets/Descriptors/BTX.sorghumls";
const std::filesystem::path kPawagaReferenceDescriptorPath = "ManualAssets/Descriptors/Pawaga.sorghumls";

struct TopFaceTriangle {
  glm::vec3 p0 = glm::vec3(0.0f);
  glm::vec3 p1 = glm::vec3(0.0f);
  glm::vec3 p2 = glm::vec3(0.0f);
  glm::vec3 normal = glm::vec3(0.0f, 1.0f, 0.0f);
  float area = 0.0f;
};

struct LSystemTriangleTarget {
  Entity plant;
  Entity leaf_entity;
  Entity internode_entity;
  std::string name;
  std::string cultivar;
  uint32_t row = 0;
  uint32_t column = 0;
  glm::vec3 position = glm::vec3(0.0f);
};

struct GreenTissueStats {
  uint32_t leaf_triangle_count = 0;
  uint32_t stem_triangle_count = 0;
  float leaf_area = 0.0f;
  float stem_area = 0.0f;
  float min_y = std::numeric_limits<float>::max();
  float max_y = std::numeric_limits<float>::lowest();
  glm::vec3 min_position = glm::vec3(std::numeric_limits<float>::max());
  glm::vec3 max_position = glm::vec3(std::numeric_limits<float>::lowest());
  bool has_geometry = false;

  [[nodiscard]] uint32_t TriangleCount() const {
    return leaf_triangle_count + stem_triangle_count;
  }

  [[nodiscard]] float Area() const {
    return leaf_area + stem_area;
  }
};

struct ParbarPanelHeightMetadata {
  uint32_t represented_plant_count = 0;
  float average_plant_height = 0.0f;
  float average_root_y = 0.0f;
  float top_elevation = 0.0f;
  float height_fraction = 0.0f;
};

Entity FindEntityByName(const std::shared_ptr<Scene>& scene, const std::string& name) {
  if (!scene) {
    return {};
  }
  for (const auto& entity : scene->UnsafeGetAllEntities()) {
    if (scene->IsEntityValid(entity) && scene->GetEntityName(entity) == name) {
      return entity;
    }
  }
  return {};
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

Entity FindParbarMeshEntity(const std::shared_ptr<Scene>& scene, const ParbarPanel& panel) {
  Entity root = FindEntityByName(scene, panel.root);
  if (scene && !scene->IsEntityValid(root)) {
    root = FindEntityByName(scene, std::string(panel.root) + "_SensorRig");
  }
  if (!scene || !scene->IsEntityValid(root)) {
    EVOENGINE_ERROR("Missing PARBAR root: " + std::string(panel.root))
    return {};
  }
  auto candidates = CollectDescendants(scene, root);
  candidates.insert(candidates.begin(), root);
  for (const auto& entity : candidates) {
    if (scene->GetEntityName(entity) != panel.mesh_entity_name || !scene->HasPrivateComponent<MeshRenderer>(entity)) {
      continue;
    }
    const auto renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(entity).lock();
    if (renderer && renderer->mesh.Get<Mesh>()) {
      return entity;
    }
  }
  EVOENGINE_ERROR("Missing PARBAR mesh: " + std::string(panel.root) + "/" + std::string(panel.mesh_entity_name))
  return {};
}

bool IsMiddleParbarPanel(const ParbarPanel& panel) {
  return std::string(panel.sensor_bar_level) == "middle";
}

bool NameContains(const std::string& name, const std::string& token) {
  auto haystack = name;
  auto needle = token;
  std::transform(haystack.begin(), haystack.end(), haystack.begin(), [](const unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  std::transform(needle.begin(), needle.end(), needle.begin(), [](const unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return haystack.find(needle) != std::string::npos;
}

std::string CultivarFromPlantName(const std::string& name) {
  if (NameContains(name, "pawaga")) {
    return "Pawaga";
  }
  if (NameContains(name, "btx")) {
    return "BTX";
  }
  return {};
}

bool MatchesCultivarFilter(const std::string& cultivar, const std::string& filter) {
  return filter.empty() || NameContains(cultivar, filter);
}

std::shared_ptr<SorghumLSDescriptor> ReferenceDescriptorForCultivar(const std::string& cultivar) {
  const auto path = cultivar == "BTX"      ? kBtxReferenceDescriptorPath
                    : cultivar == "Pawaga" ? kPawagaReferenceDescriptorPath
                                           : std::filesystem::path{};
  return path.empty() ? nullptr
                      : std::dynamic_pointer_cast<SorghumLSDescriptor>(ProjectManager::GetOrCreateAsset(path));
}

std::string BasePlantNameFromClusterName(const std::string& name) {
  const std::string marker = "_cluster_";
  const size_t marker_position = name.find(marker);
  return marker_position == std::string::npos ? name : name.substr(0, marker_position);
}

bool IsSorghumPlantingMarkerName(const std::string& name) {
  return !CultivarFromPlantName(name).empty() && name.find("_LSystem_") != std::string::npos &&
         name.find("_cluster_") == std::string::npos;
}

glm::vec3 SafeNormalize(const glm::vec3& value, const glm::vec3& fallback) {
  const float length = glm::length(value);
  return length > 1e-6f ? value / length : fallback;
}

glm::vec3 ClosestPointOnTriangle(const glm::vec3& point, const glm::vec3& a, const glm::vec3& b, const glm::vec3& c) {
  const glm::vec3 ab = b - a;
  const glm::vec3 ac = c - a;
  const glm::vec3 ap = point - a;
  const float d1 = glm::dot(ab, ap);
  const float d2 = glm::dot(ac, ap);
  if (d1 <= 0.0f && d2 <= 0.0f)
    return a;

  const glm::vec3 bp = point - b;
  const float d3 = glm::dot(ab, bp);
  const float d4 = glm::dot(ac, bp);
  if (d3 >= 0.0f && d4 <= d3)
    return b;

  const float vc = d1 * d4 - d3 * d2;
  if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
    return a + ab * (d1 / (d1 - d3));
  }

  const glm::vec3 cp = point - c;
  const float d5 = glm::dot(ab, cp);
  const float d6 = glm::dot(ac, cp);
  if (d6 >= 0.0f && d5 <= d6)
    return c;

  const float vb = d5 * d2 - d1 * d6;
  if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
    return a + ac * (d2 / (d2 - d6));
  }

  const float va = d3 * d6 - d5 * d4;
  if (va <= 0.0f && d4 - d3 >= 0.0f && d5 - d6 >= 0.0f) {
    return b + (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)));
  }

  const float denominator = 1.0f / (va + vb + vc);
  const float v = vb * denominator;
  const float w = vc * denominator;
  return a + ab * v + ac * w;
}

std::vector<TopFaceTriangle> CollectTopFaceTriangles(const std::shared_ptr<Scene>& scene, const Entity& entity) {
  std::vector<TopFaceTriangle> top_triangles;
  if (!scene || !scene->IsEntityValid(entity)) {
    return top_triangles;
  }
  const auto renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(entity).lock();
  if (!renderer) {
    return top_triangles;
  }
  const auto mesh = renderer->mesh.Get<Mesh>();
  if (!mesh) {
    return top_triangles;
  }

  const auto global_transform = scene->GetDataComponent<GlobalTransform>(entity);
  const auto& vertices = mesh->PeekVertices();
  const auto& triangles = mesh->PeekTriangles();
  for (const auto& triangle : triangles) {
    if (triangle.x >= vertices.size() || triangle.y >= vertices.size() || triangle.z >= vertices.size()) {
      continue;
    }
    const glm::vec3 p0 = global_transform.TransformPoint(vertices[triangle.x].position);
    const glm::vec3 p1 = global_transform.TransformPoint(vertices[triangle.y].position);
    const glm::vec3 p2 = global_transform.TransformPoint(vertices[triangle.z].position);
    const glm::vec3 cross = glm::cross(p1 - p0, p2 - p0);
    const float double_area = glm::length(cross);
    if (double_area <= 1e-8f) {
      continue;
    }
    const glm::vec3 normal = cross / double_area;
    if (normal.y <= 0.45f) {
      continue;
    }
    top_triangles.push_back({p0, p1, p2, normal, double_area * 0.5f});
  }
  return top_triangles;
}

float TopFaceElevation(const std::vector<TopFaceTriangle>& top_triangles) {
  float weighted_y = 0.0f;
  float total_area = 0.0f;
  for (const auto& triangle : top_triangles) {
    weighted_y += (triangle.p0.y + triangle.p1.y + triangle.p2.y) * (triangle.area / 3.0f);
    total_area += triangle.area;
  }
  return total_area > 1e-8f ? weighted_y / total_area : 0.0f;
}

bool AppendParbarPanelSamplers(const std::shared_ptr<Scene>& scene, const ParbarPanel& panel,
                               const uint32_t samples_per_panel,
                               std::vector<IlluminationSampler<glm::vec3>>& samplers) {
  const Entity entity = FindParbarMeshEntity(scene, panel);
  const auto top_triangles = CollectTopFaceTriangles(scene, entity);
  if (top_triangles.empty()) {
    EVOENGINE_ERROR("No upward top-face triangles found for " + std::string(panel.root) + "/" + panel.mesh_entity_name)
    return false;
  }

  std::vector<glm::vec3> points;
  points.reserve(top_triangles.size() * 3);
  glm::vec3 centroid(0.0f);
  glm::vec3 weighted_normal(0.0f);
  float total_area = 0.0f;
  for (const auto& triangle : top_triangles) {
    points.emplace_back(triangle.p0);
    points.emplace_back(triangle.p1);
    points.emplace_back(triangle.p2);
    centroid += (triangle.p0 + triangle.p1 + triangle.p2) * (triangle.area / 3.0f);
    weighted_normal += triangle.normal * triangle.area;
    total_area += triangle.area;
  }
  if (total_area <= 1e-8f || points.empty()) {
    return false;
  }
  centroid /= total_area;
  const glm::vec3 normal = SafeNormalize(weighted_normal, glm::vec3(0.0f, 1.0f, 0.0f));

  glm::vec3 axis_u(1.0f, 0.0f, 0.0f);
  float best_distance2 = 0.0f;
  for (size_t i = 0; i < points.size(); ++i) {
    for (size_t j = i + 1; j < points.size(); ++j) {
      const glm::vec3 delta = points[j] - points[i];
      const glm::vec3 projected = delta - normal * glm::dot(delta, normal);
      const float distance2 = glm::dot(projected, projected);
      if (distance2 > best_distance2) {
        best_distance2 = distance2;
        axis_u = projected;
      }
    }
  }
  axis_u = SafeNormalize(axis_u, glm::vec3(1.0f, 0.0f, 0.0f));
  glm::vec3 axis_v = SafeNormalize(glm::cross(normal, axis_u), glm::vec3(0.0f, 0.0f, 1.0f));

  float min_u = std::numeric_limits<float>::max();
  float max_u = std::numeric_limits<float>::lowest();
  float min_v = std::numeric_limits<float>::max();
  float max_v = std::numeric_limits<float>::lowest();
  for (const auto& point : points) {
    const glm::vec3 offset = point - centroid;
    const float u = glm::dot(offset, axis_u);
    const float v = glm::dot(offset, axis_v);
    min_u = std::min(min_u, u);
    max_u = std::max(max_u, u);
    min_v = std::min(min_v, v);
    max_v = std::max(max_v, v);
  }
  const float center_v = (min_v + max_v) * 0.5f;
  const uint32_t count = std::max(1u, samples_per_panel);
  samplers.reserve(samplers.size() + count);
  for (uint32_t column = 0; column < count; ++column) {
    const float fraction = (static_cast<float>(column) + 0.5f) / static_cast<float>(count);
    const float u = glm::mix(min_u, max_u, fraction);
    const glm::vec3 target = centroid + axis_u * u + axis_v * center_v;

    float best_distance = std::numeric_limits<float>::max();
    glm::vec3 best_position = target;
    glm::vec3 best_normal = normal;
    for (const auto& triangle : top_triangles) {
      const glm::vec3 closest = ClosestPointOnTriangle(target, triangle.p0, triangle.p1, triangle.p2);
      const float distance = glm::dot(closest - target, closest - target);
      if (distance < best_distance) {
        best_distance = distance;
        best_position = closest;
        best_normal = triangle.normal;
      }
    }

    IlluminationSampler<glm::vec3> sampler;
    sampler.v_0.position = sampler.v_1.position = sampler.v_2.position = best_position;
    sampler.v_0.normal = sampler.v_1.normal = sampler.v_2.normal = SafeNormalize(best_normal, normal);
    sampler.front_face = true;
    sampler.back_face = false;
    samplers.emplace_back(sampler);
  }
  return true;
}

float IlluminationScalar(const glm::vec3& energy) {
  return glm::dot(energy, glm::vec3(0.2126f, 0.7152f, 0.0722f));
}

bool ParseUnsignedAfter(const std::string& text, size_t& offset, uint32_t& value) {
  if (offset >= text.size() || !std::isdigit(static_cast<unsigned char>(text[offset]))) {
    return false;
  }
  uint32_t parsed = 0;
  while (offset < text.size() && std::isdigit(static_cast<unsigned char>(text[offset]))) {
    parsed = parsed * 10u + static_cast<uint32_t>(text[offset] - '0');
    offset++;
  }
  value = parsed;
  return true;
}

bool TryParseGridCoordinate(const std::string& name, uint32_t& row, uint32_t& column) {
  const size_t row_marker = name.find("_R");
  if (row_marker == std::string::npos) {
    return false;
  }
  size_t row_offset = row_marker + 2;
  uint32_t parsed_row = 0;
  if (!ParseUnsignedAfter(name, row_offset, parsed_row)) {
    return false;
  }
  const size_t column_marker = name.find("_C", row_offset);
  if (column_marker == std::string::npos) {
    return false;
  }
  size_t column_offset = column_marker + 2;
  uint32_t parsed_column = 0;
  if (!ParseUnsignedAfter(name, column_offset, parsed_column)) {
    return false;
  }
  row = parsed_row;
  column = parsed_column;
  return true;
}

void PrepareSorghumLsMeshesForRayTracing(const std::shared_ptr<Scene>& scene) {
  if (!scene) {
    return;
  }
  const auto* sorghum_entities_ptr = scene->UnsafeGetPrivateComponentOwnersList<SorghumLS>();
  if (!sorghum_entities_ptr) {
    return;
  }
  for (const auto& entity : *sorghum_entities_ptr) {
    if (!scene->IsEntityValid(entity)) {
      continue;
    }
    for (const auto& child : scene->GetChildren(entity)) {
      const auto child_name = scene->GetEntityName(child);
      if (child_name == "Sorghum Internodes") {
        const auto particles = scene->GetOrSetPrivateComponent<Particles>(child).lock();
        if (!particles) {
          continue;
        }
        const auto mesh = particles->mesh.Get<Mesh>();
        const auto particle_info_list = particles->particle_info_list.Get<ParticleInfoList>();
        if (mesh && particle_info_list && !mesh->PeekVertices().empty() &&
            !particle_info_list->PeekParticleInfoList().empty()) {
          mesh->ray_tracing_acceleration_enabled = true;
        }
      } else if (child_name == "Sorghum Leaves") {
        const auto renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(child).lock();
        if (!renderer) {
          continue;
        }
        const auto mesh = renderer->mesh.Get<Mesh>();
        if (!mesh) {
          continue;
        }
        const auto vertices = mesh->PeekVertices();
        const auto triangles = mesh->PeekTriangles();
        if (vertices.empty() || triangles.empty()) {
          continue;
        }
        if (const auto material = renderer->material.Get<Material>()) {
          material->SetAlbedoTexture(nullptr);
          material->SetNormalTexture(nullptr);
          material->SetRoughnessTexture(nullptr);
          material->SetMetallicTexture(nullptr);
          material->SetAoTexture(nullptr);
          material->vertex_color_only = true;
          material->draw_settings.cull_mode = VK_CULL_MODE_NONE;
          material->material_properties.albedo_color = glm::vec3(0.34f, 0.68f, 0.24f);
          material->material_properties.metallic = 0.0f;
          material->material_properties.roughness = 0.72f;
          material->material_properties.emission = 0.05f;
        }
        mesh->ray_tracing_acceleration_enabled = true;
        mesh->compact_storage_on_update = false;
        mesh->optimize_meshlet_layout = false;
        VertexAttributes attributes{};
        attributes.normal = true;
        attributes.tangent = true;
        attributes.color = true;
        attributes.tex_coord = true;
        mesh->SetVertices(attributes, vertices, triangles);
      }
    }
  }
}

Entity FindSorghumLsChildEntity(const std::shared_ptr<Scene>& scene, const Entity& plant, const std::string& name) {
  if (!scene || !scene->IsEntityValid(plant)) {
    return {};
  }
  for (const auto& child : scene->GetChildren(plant)) {
    if (scene->GetEntityName(child) == name) {
      return child;
    }
  }
  return {};
}

Entity FindSorghumLsLeafMeshEntity(const std::shared_ptr<Scene>& scene, const Entity& plant) {
  const Entity entity = FindSorghumLsChildEntity(scene, plant, "Sorghum Leaves");
  return scene && scene->IsEntityValid(entity) && scene->HasPrivateComponent<MeshRenderer>(entity) ? entity : Entity{};
}

Entity FindSorghumLsInternodeEntity(const std::shared_ptr<Scene>& scene, const Entity& plant) {
  const Entity entity = FindSorghumLsChildEntity(scene, plant, "Sorghum Internodes");
  return scene && scene->IsEntityValid(entity) && scene->HasPrivateComponent<Particles>(entity) ? entity : Entity{};
}

void IncludeHeightPoint(GreenTissueStats& stats, const glm::vec3& point) {
  stats.min_y = std::min(stats.min_y, point.y);
  stats.max_y = std::max(stats.max_y, point.y);
  stats.min_position = glm::min(stats.min_position, point);
  stats.max_position = glm::max(stats.max_position, point);
  stats.has_geometry = true;
}

void TransformSampler(IlluminationSampler<glm::vec3>& sampler, const glm::mat4& transform) {
  sampler.v_0.position = glm::vec3(transform * glm::vec4(sampler.v_0.position, 1.0f));
  sampler.v_1.position = glm::vec3(transform * glm::vec4(sampler.v_1.position, 1.0f));
  sampler.v_2.position = glm::vec3(transform * glm::vec4(sampler.v_2.position, 1.0f));
  sampler.v_0.normal =
      SafeNormalize(glm::vec3(transform * glm::vec4(sampler.v_0.normal, 0.0f)), glm::vec3(0.0f, 1.0f, 0.0f));
  sampler.v_1.normal = SafeNormalize(glm::vec3(transform * glm::vec4(sampler.v_1.normal, 0.0f)), sampler.v_0.normal);
  sampler.v_2.normal = SafeNormalize(glm::vec3(transform * glm::vec4(sampler.v_2.normal, 0.0f)), sampler.v_0.normal);
  sampler.direction = glm::vec3(0.0f);
  sampler.energy = glm::vec3(0.0f);
}

float SamplerSurfaceArea(const IlluminationSampler<glm::vec3>& sampler) {
  float area = 0.0f;
  if (sampler.front_face) {
    area += sampler.GetArea();
  }
  if (sampler.back_face) {
    area += sampler.GetArea();
  }
  return area;
}

bool AppendLeafTriangleSamplers(const std::shared_ptr<Scene>& scene, const Entity& leaf_entity,
                                const int max_triangle_count, std::vector<IlluminationSampler<glm::vec3>>& samplers,
                                GreenTissueStats& stats) {
  if (!scene || !scene->IsEntityValid(leaf_entity)) {
    return false;
  }
  const auto mesh_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(leaf_entity).lock();
  if (!mesh_renderer) {
    return false;
  }
  const auto mesh = mesh_renderer->mesh.Get<Mesh>();
  if (!mesh || mesh->UnsafeGetTriangles().empty()) {
    return false;
  }

  const auto& triangles = mesh->UnsafeGetTriangles();
  auto& vertices = mesh->UnsafeGetVertices();
  const size_t triangle_count = triangles.size();
  const size_t sampler_count = max_triangle_count <= 0
                                   ? triangle_count
                                   : std::min(static_cast<size_t>(std::max(1, max_triangle_count)), triangle_count);
  const auto global_transform = scene->GetDataComponent<GlobalTransform>(leaf_entity);
  for (const auto& vertex : vertices) {
    IncludeHeightPoint(stats, global_transform.TransformPoint(vertex.position));
  }

  samplers.reserve(samplers.size() + sampler_count);
  for (size_t sampler_index = 0; sampler_index < sampler_count; sampler_index++) {
    const size_t triangle_index =
        sampler_count == triangle_count
            ? sampler_index
            : std::min(triangle_count - 1,
                       (sampler_index * triangle_count + triangle_count / (2 * sampler_count)) / sampler_count);
    const auto& triangle = triangles[triangle_index];
    if (triangle.x >= vertices.size() || triangle.y >= vertices.size() || triangle.z >= vertices.size()) {
      continue;
    }

    IlluminationSampler<glm::vec3> sampler;
    sampler.v_0 = vertices[triangle.x];
    sampler.v_1 = vertices[triangle.y];
    sampler.v_2 = vertices[triangle.z];
    TransformSampler(sampler, global_transform.value);
    sampler.front_face = sampler.back_face = true;
    stats.leaf_area += SamplerSurfaceArea(sampler);
    stats.leaf_triangle_count++;
    samplers.emplace_back(sampler);
  }
  return stats.leaf_area > 0.0f;
}

bool AppendInternodeTriangleSamplers(const std::shared_ptr<Scene>& scene, const Entity& internode_entity,
                                     const int max_triangle_count,
                                     std::vector<IlluminationSampler<glm::vec3>>& samplers, GreenTissueStats& stats) {
  if (!scene || !scene->IsEntityValid(internode_entity)) {
    return false;
  }
  const auto particles = scene->GetOrSetPrivateComponent<Particles>(internode_entity).lock();
  if (!particles) {
    return false;
  }
  const auto mesh = particles->mesh.Get<Mesh>();
  const auto particle_info_list = particles->particle_info_list.Get<ParticleInfoList>();
  if (!mesh || !particle_info_list || mesh->UnsafeGetTriangles().empty()) {
    return false;
  }

  const auto& instances = particle_info_list->PeekParticleInfoList();
  if (instances.empty()) {
    return false;
  }
  const auto& triangles = mesh->UnsafeGetTriangles();
  auto& vertices = mesh->UnsafeGetVertices();
  const auto entity_transform = scene->GetDataComponent<GlobalTransform>(internode_entity).value;
  const size_t triangle_count = triangles.size();
  const size_t candidate_count = instances.size() * triangle_count;
  const size_t sampler_count = max_triangle_count <= 0
                                   ? candidate_count
                                   : std::min(static_cast<size_t>(std::max(1, max_triangle_count)), candidate_count);

  for (const auto& instance : instances) {
    const glm::mat4 transform = entity_transform * instance.instance_matrix.value;
    for (const auto& vertex : vertices) {
      IncludeHeightPoint(stats, glm::vec3(transform * glm::vec4(vertex.position, 1.0f)));
    }
  }

  samplers.reserve(samplers.size() + sampler_count);
  for (size_t sampler_index = 0; sampler_index < sampler_count; sampler_index++) {
    const size_t candidate_index =
        sampler_count == candidate_count
            ? sampler_index
            : std::min(candidate_count - 1,
                       (sampler_index * candidate_count + candidate_count / (2 * sampler_count)) / sampler_count);
    const size_t instance_index = candidate_index / triangle_count;
    const size_t triangle_index = candidate_index % triangle_count;
    const auto& triangle = triangles[triangle_index];
    if (triangle.x >= vertices.size() || triangle.y >= vertices.size() || triangle.z >= vertices.size()) {
      continue;
    }

    IlluminationSampler<glm::vec3> sampler;
    sampler.v_0 = vertices[triangle.x];
    sampler.v_1 = vertices[triangle.y];
    sampler.v_2 = vertices[triangle.z];
    TransformSampler(sampler, entity_transform * instances[instance_index].instance_matrix.value);
    sampler.front_face = true;
    sampler.back_face = false;
    stats.stem_area += SamplerSurfaceArea(sampler);
    stats.stem_triangle_count++;
    samplers.emplace_back(sampler);
  }
  return stats.stem_area > 0.0f;
}

GreenTissueStats AppendGreenPlantSamplers(const std::shared_ptr<Scene>& scene, const LSystemTriangleTarget& target,
                                          const int max_triangle_count,
                                          std::vector<IlluminationSampler<glm::vec3>>& samplers) {
  GreenTissueStats stats;
  AppendLeafTriangleSamplers(scene, target.leaf_entity, max_triangle_count, samplers, stats);
  AppendInternodeTriangleSamplers(scene, target.internode_entity, max_triangle_count, samplers, stats);
  return stats;
}

GreenTissueStats MeasureGreenPlantGeometry(const std::shared_ptr<Scene>& scene, const Entity& plant) {
  LSystemTriangleTarget target;
  target.leaf_entity = FindSorghumLsLeafMeshEntity(scene, plant);
  target.internode_entity = FindSorghumLsInternodeEntity(scene, plant);
  std::vector<IlluminationSampler<glm::vec3>> unused;
  return AppendGreenPlantSamplers(scene, target, 0, unused);
}

ParbarPanelHeightMetadata GetParbarPanelHeightMetadata(const std::shared_ptr<Scene>& scene, const ParbarPanel& panel,
                                                       const float height_fraction) {
  ParbarPanelHeightMetadata metadata;
  metadata.height_fraction = IsMiddleParbarPanel(panel) ? height_fraction : 0.0f;
  const Entity panel_entity = FindParbarMeshEntity(scene, panel);
  metadata.top_elevation = TopFaceElevation(CollectTopFaceTriangles(scene, panel_entity));
  if (!IsMiddleParbarPanel(panel) || !scene) {
    return metadata;
  }

  const auto* sorghum_entities_ptr = scene->UnsafeGetPrivateComponentOwnersList<SorghumLS>();
  if (!sorghum_entities_ptr) {
    return metadata;
  }
  struct ClumpHeight {
    float height_sum = 0.0f;
    float root_y_sum = 0.0f;
    uint32_t member_count = 0;
  };
  std::map<std::string, ClumpHeight> clumps;
  for (const auto& plant : *sorghum_entities_ptr) {
    if (!scene->IsEntityValid(plant) || CultivarFromPlantName(scene->GetEntityName(plant)) != panel.cultivar) {
      continue;
    }
    const std::string base_plant_name = BasePlantNameFromClusterName(scene->GetEntityName(plant));
    const auto plant_transform = scene->GetDataComponent<GlobalTransform>(plant);
    const auto stats = MeasureGreenPlantGeometry(scene, plant);
    if (!stats.has_geometry) {
      continue;
    }
    auto& clump = clumps[base_plant_name];
    clump.root_y_sum += plant_transform.GetPosition().y;
    clump.height_sum += std::max(0.0f, stats.max_y - plant_transform.GetPosition().y);
    clump.member_count++;
  }
  for (const auto& [_, clump] : clumps) {
    if (clump.member_count == 0) {
      continue;
    }
    const float member_count = static_cast<float>(clump.member_count);
    metadata.average_plant_height += clump.height_sum / member_count;
    metadata.average_root_y += clump.root_y_sum / member_count;
    metadata.represented_plant_count++;
  }
  if (metadata.represented_plant_count > 0) {
    const float clump_count = static_cast<float>(metadata.represented_plant_count);
    metadata.average_plant_height /= clump_count;
    metadata.average_root_y /= clump_count;
  }
  return metadata;
}

size_t MoveMiddleParbarPanelsToPlantHeightFraction(const std::shared_ptr<Scene>& scene, const float height_fraction) {
  if (!scene) {
    return 0;
  }
  size_t moved_count = 0;
  for (const auto& panel : kParbarPanels) {
    if (!IsMiddleParbarPanel(panel)) {
      continue;
    }
    const auto metadata = GetParbarPanelHeightMetadata(scene, panel, height_fraction);
    if (metadata.represented_plant_count == 0) {
      continue;
    }
    const Entity panel_entity = FindParbarMeshEntity(scene, panel);
    const float target_top_y = metadata.average_root_y + height_fraction * metadata.average_plant_height;
    const float delta_y = target_top_y - metadata.top_elevation;
    auto global_transform = scene->GetDataComponent<GlobalTransform>(panel_entity);
    global_transform.SetPosition(global_transform.GetPosition() + glm::vec3(0.0f, delta_y, 0.0f));
    scene->SetDataComponent(panel_entity, global_transform);
    moved_count++;
  }
  if (moved_count > 0) {
    TransformGraph::CalculateTransformGraphs(scene, false);
  }
  return moved_count;
}

std::filesystem::path WithExtension(std::filesystem::path path, const std::string& extension) {
  if (path.extension() != extension) {
    path.replace_extension(extension);
  }
  return path;
}

float SampleTargetGddForDescriptorSeed(const SorghumLSDescriptor& descriptor, const uint32_t seed) {
  std::mt19937 rng(seed);
  return std::max(0.0f, SampleDistribution(descriptor.target_gdd, rng));
}

void ScalePlot(evo_engine::Plot2D<float>& plot, const float scale) {
  plot.min_value *= scale;
  plot.max_value *= scale;
}

bool IsProjectAssetPath(const std::filesystem::path& path) {
  return !path.empty() && path.is_relative() && path.string().find("..") == std::string::npos;
}

void ScaleLengthDistribution(evo_engine::PlottedDistribution<float>& distribution, const float mean_scale,
                             const float deviation_scale) {
  ScalePlot(distribution.mean, mean_scale);
  ScalePlot(distribution.deviation, std::abs(deviation_scale));
}

void ConfigureCalibratedDescriptor(SorghumLSDescriptor& descriptor, const float leaf_modules_mean,
                                   const float leaf_modules_deviation, const float length_mean_scale,
                                   const float length_deviation_scale, const float leaf_width_scale,
                                   const float main_culm_diameter_m, const float tiller_leaf_count_ratio,
                                   const float tiller_height_ratio) {
  descriptor.total_phytomer_count.mean = std::max(1.0f, leaf_modules_mean);
  descriptor.total_phytomer_count.deviation = std::max(0.0f, leaf_modules_deviation);
  ScaleLengthDistribution(descriptor.internode_length, std::max(0.0f, length_mean_scale), length_deviation_scale);
  ScaleLengthDistribution(descriptor.leaf_blade_length, std::max(0.0f, length_mean_scale), length_deviation_scale);
  ScaleLengthDistribution(descriptor.leaf_sheath_length, std::max(0.0f, length_mean_scale), length_deviation_scale);
  ScaleLengthDistribution(descriptor.leaf_neck_length, std::max(0.0f, length_mean_scale), length_deviation_scale);
  descriptor.leaf_width_scale = std::clamp(leaf_width_scale, 0.05f, 3.0f);
  if (main_culm_diameter_m > 0.0f) {
    descriptor.internode_thickness.mean.min_value = main_culm_diameter_m;
    descriptor.internode_thickness.mean.max_value = main_culm_diameter_m;
    descriptor.internode_thickness.deviation.min_value = 0.0f;
    descriptor.internode_thickness.deviation.max_value = 0.0f;
  }
  descriptor.tiller_model_version = 3u;
  descriptor.tiller_leaf_count_ratio.mean = std::clamp(tiller_leaf_count_ratio, 0.5f, 1.1f);
  descriptor.tiller_height_ratio.mean = std::clamp(tiller_height_ratio, 0.5f, 1.1f);
}

std::shared_ptr<SorghumLSDescriptor> CloneDescriptorTemporary(const SorghumLSDescriptor& source) {
  YAML::Emitter out;
  out << YAML::BeginMap;
  SerializeSorghumLSDescriptor(out, source);
  out << YAML::EndMap;

  auto clone = AssetManager::CreateTemporaryAsset<SorghumLSDescriptor>();
  if (!clone) {
    return {};
  }
  DeserializeSorghumLSDescriptor(YAML::Load(out.c_str()), *clone);
  return clone;
}

std::shared_ptr<SorghumLSDescriptor> LoadSorghumLsDescriptorAsset(const std::filesystem::path& path) {
  const auto descriptor_path = WithExtension(path, ".sorghumls");
  if (!IsProjectAssetPath(descriptor_path)) {
    return {};
  }
  const auto absolute_path = ProjectManager::GetAssetsFolderPath() / descriptor_path;
  if (!std::filesystem::exists(absolute_path)) {
    return {};
  }
  return std::dynamic_pointer_cast<SorghumLSDescriptor>(ProjectManager::GetOrCreateAsset(descriptor_path));
}

std::shared_ptr<SorghumLSDescriptor> CreateCalibratedDescriptorTemporary(
    const std::filesystem::path& base_path, const float leaf_modules_mean, const float leaf_modules_deviation,
    const float length_mean_scale, const float length_deviation_scale, const float leaf_width_scale,
    const float main_culm_diameter_m, const float tiller_leaf_count_ratio, const float tiller_height_ratio,
    const bool disable_tillers) {
  const auto source = LoadSorghumLsDescriptorAsset(base_path);
  if (!source) {
    return {};
  }
  auto clone = CloneDescriptorTemporary(*source);
  if (!clone) {
    return {};
  }
  ConfigureCalibratedDescriptor(*clone, leaf_modules_mean, leaf_modules_deviation, length_mean_scale,
                                length_deviation_scale, leaf_width_scale, main_culm_diameter_m, tiller_leaf_count_ratio,
                                tiller_height_ratio);
  if (disable_tillers) {
    clone->tiller_count = {0.0f, 0.0f};
    clone->tiller_count_min = 0;
    clone->tiller_count_max = 0;
  }
  return clone;
}

bool SaveDescriptorAsset(const std::shared_ptr<SorghumLSDescriptor>& descriptor, const std::filesystem::path& path) {
  if (!descriptor || path.empty() || path.is_absolute()) {
    return false;
  }
  return descriptor->SetPathAndSave(WithExtension(path, ".sorghumls"));
}

struct SorghumArchitectureStats {
  uint32_t main_culm_leaf_count = 0;
  uint32_t tiller_leaf_count = 0;
  uint32_t primary_tiller_count = 0;
  std::vector<LSystemAxisPhenotypeRecord> axes;
};

SorghumArchitectureStats MeasureSorghumArchitecture(const SorghumLS& sorghum) {
  SorghumArchitectureStats stats;
  struct AxisAccumulator {
    int origin_rank = 0;
    uint32_t leaf_count = 0;
    uint32_t internode_count = 0;
    float tip_height_m = 0.0f;
  };
  std::map<int, AxisAccumulator> axes;
  float root_height = 0.0f;
  for (const auto handle : sorghum.growth_model.graph.PeekSortedNodeList()) {
    const auto& node = sorghum.growth_model.graph.PeekNode(handle);
    if (node.data.Is<SorghumRoot>()) {
      root_height = node.info.global_position.y;
    } else if (node.data.Is<SorghumInternode>()) {
      const auto& internode = node.data.Get<SorghumInternode>();
      auto& axis = axes[internode.axis_id];
      axis.origin_rank = internode.origin_rank;
      axis.internode_count++;
      axis.tip_height_m = std::max(axis.tip_height_m, node.info.GetGlobalEndPosition().y - root_height);
    } else if (node.data.Is<SorghumLeaf>()) {
      const auto& leaf = node.data.Get<SorghumLeaf>();
      auto& axis = axes[leaf.axis_id];
      axis.origin_rank = leaf.origin_rank;
      axis.leaf_count++;
    }
  }
  const auto main = axes.find(0);
  const float main_height = main == axes.end() ? 0.0f : main->second.tip_height_m;
  const uint32_t main_leaves = main == axes.end() ? 0u : main->second.leaf_count;
  for (const auto& [axis_id, axis] : axes) {
    LSystemAxisPhenotypeRecord record;
    record.axis_id = axis_id;
    record.origin_rank = axis.origin_rank;
    record.leaf_count = axis.leaf_count;
    record.internode_count = axis.internode_count;
    record.culm_tip_height_m = axis.tip_height_m;
    record.leaf_ratio_to_main =
        axis_id == 0 || main_leaves == 0 ? 1.0f : static_cast<float>(axis.leaf_count) / static_cast<float>(main_leaves);
    record.height_ratio_to_main = axis_id == 0 || main_height <= 1.0e-6f ? 1.0f : axis.tip_height_m / main_height;
    stats.axes.emplace_back(record);
    if (axis_id == 0) {
      stats.main_culm_leaf_count = axis.leaf_count;
    } else {
      stats.tiller_leaf_count += axis.leaf_count;
      stats.primary_tiller_count++;
    }
  }
  return stats;
}

bool BuildCalibrationStemContext(const SorghumGraph& graph, const LNodeHandle leaf_handle, StemContext& stem_context) {
  stem_context.segments.clear();
  if (leaf_handle < 0)
    return false;
  const auto& leaf_node = graph.PeekNode(leaf_handle);
  if (!leaf_node.data.Is<SorghumLeaf>())
    return false;
  const int leaf_order = leaf_node.data.Get<SorghumLeaf>().order;
  std::vector<LNodeHandle> chain;
  auto cursor = leaf_node.GetParentHandle();
  while (cursor >= 0) {
    const auto& node = graph.PeekNode(cursor);
    if (node.data.Is<SorghumInternode>() && node.data.Get<SorghumInternode>().order == leaf_order) {
      chain.push_back(cursor);
    }
    cursor = node.GetParentHandle();
  }
  std::reverse(chain.begin(), chain.end());
  for (const auto handle : chain) {
    const auto& node = graph.PeekNode(handle);
    const auto& internode = node.data.Get<SorghumInternode>();
    StemContext::Segment segment;
    segment.position = node.info.global_position;
    segment.front = glm::normalize(node.info.global_rotation * glm::vec3(0, 0, -1));
    segment.up = glm::normalize(node.info.global_rotation * glm::vec3(0, 1, 0));
    segment.radius = std::max(0.0002f, internode.thickness * 0.5f);
    segment.theta = 180.0f;
    stem_context.segments.push_back(segment);
  }
  if (chain.empty())
    return false;
  const auto& last = graph.PeekNode(chain.back());
  auto tip = stem_context.segments.back();
  tip.position = last.info.GetGlobalEndPosition();
  stem_context.segments.push_back(tip);
  return true;
}

float MeasureCalibrationPlantHeight(const SorghumLS& sorghum) {
  float maximum_y = 0.0f;
  StemContext stem_context;
  l_system_package::SorghumSpline spline;
  for (const auto handle : sorghum.growth_model.graph.PeekSortedNodeList()) {
    const auto& node = sorghum.growth_model.graph.PeekNode(handle);
    if (node.data.Is<SorghumInternode>()) {
      maximum_y = std::max({maximum_y, node.info.global_position.y, node.info.GetGlobalEndPosition().y});
    } else if (node.data.Is<SorghumLeaf>() && node.data.Get<SorghumLeaf>().alive &&
               BuildCalibrationStemContext(sorghum.growth_model.graph, handle, stem_context)) {
      BuildLeafSplineFromState(node.data.Get<SorghumLeaf>(), stem_context, sorghum.growth_model.sampled,
                               sorghum.leaf_mesh_settings, spline);
      const int horizontal_steps = std::max(2, sorghum.leaf_mesh_settings.horizontal_subdivision_step) * 2;
      for (const auto& segment : spline.segments) {
        for (int i = 0; i <= horizontal_steps; ++i) {
          const float angle =
              glm::mix(-segment.theta, segment.theta, static_cast<float>(i) / static_cast<float>(horizontal_steps));
          maximum_y = std::max(maximum_y, segment.GetLeafPoint(angle).y);
        }
      }
    }
  }
  return maximum_y;
}

LSystemDescriptorPhenotypeRecord MeasureDescriptorSample(const std::shared_ptr<Scene>& scene, const Entity& plant,
                                                         const std::shared_ptr<SorghumLS>& sorghum,
                                                         const std::shared_ptr<SorghumLSDescriptor>& descriptor,
                                                         const uint32_t seed) {
  LSystemDescriptorPhenotypeRecord record;
  record.seed = seed;
  if (!scene || !scene->IsEntityValid(plant) || !sorghum || !descriptor) {
    return record;
  }
  sorghum->descriptor_ref = descriptor;
  sorghum->seed = seed;
  sorghum->target_gdd = SampleTargetGddForDescriptorSeed(*descriptor, seed);
  sorghum->growth_model.Initialize(*descriptor, seed);
  sorghum->growth_model.GrowToGDD(sorghum->target_gdd, 0u);
  sorghum->growth_model.FinalizeSnapshotMorphology();

  for (const auto handle : sorghum->growth_model.graph.PeekSortedNodeList()) {
    const auto& node = sorghum->growth_model.graph.PeekNode(handle);
    if (node.data.Is<SorghumLeaf>()) {
      ++record.leaf_count;
      if (node.data.Get<SorghumLeaf>().alive)
        ++record.live_leaf_count;
    }
  }
  const auto architecture = MeasureSorghumArchitecture(*sorghum);
  record.main_culm_leaf_count = architecture.main_culm_leaf_count;
  record.tiller_leaf_count = architecture.tiller_leaf_count;
  record.primary_tiller_count = architecture.primary_tiller_count;
  record.axes = architecture.axes;
  record.height_m = MeasureCalibrationPlantHeight(*sorghum);
  record.has_geometry = true;
  return record;
}

}  // namespace
void PyDigitalAgriculture::PushSorghumLayer() {
  ApplicationContext::Get().PushLayer<SorghumLayer>("Sorghum Layer");
}
void PyDigitalAgriculture::RegisterClasses() {
  auto& application = PyEvoEngine::GetRuntime().GetApplication();
  application.RegisterPrivateComponent<ObjectRotator>("ObjectRotator");
#  ifdef DATASET_GENERATION_PACKAGE
  application.RegisterPrivateComponent<SorghumPointCloudScanner>("SorghumPointCloudScanner");
  Serialization::RegisterSerializationHandler<SorghumPointCloudScanner>(
      SerializeSorghumPointCloudScanner, DeserializeSorghumPointCloudScanner, {}, "SorghumPointCloudScanner");
#  endif
}

Entity PyDigitalAgriculture::CreateEntityFromSorghumState(const Handle& sorghum_handle) {
  const auto sorghum_asset = PyEvoEngine::GetAsset(sorghum_handle);
  if (sorghum_asset->GetTypeName() != "SorghumState") {
    EVOENGINE_ERROR("CreateEntityFromSorghumState failed: invalid asset type!")
    return {};
  }
  return DatasetGenerator::CreateSorghumEntity(sorghum_asset);
}

Entity PyDigitalAgriculture::CreateEntityFromSorghumDescriptor(const Handle& sorghum_handle) {
  const auto sorghum_asset = PyEvoEngine::GetAsset(sorghum_handle);
  if (sorghum_asset->GetTypeName() != "SorghumDescriptor") {
    EVOENGINE_ERROR("CreateEntityFromSorghumDescriptor failed: invalid asset type!")
    return {};
  }
  return DatasetGenerator::CreateSorghumEntity(sorghum_asset);
}

Entity PyDigitalAgriculture::CreateEntityFromSorghumGenerator(const Handle& sorghum_generator_handle, const int seed) {
  const auto sorghum_asset = PyEvoEngine::GetAsset(sorghum_generator_handle);
  if (sorghum_asset->GetTypeName() != "SorghumGenerator") {
    EVOENGINE_ERROR("CreateEntityFromSorghumGenerator failed: invalid asset type!")
    return {};
  }
  return DatasetGenerator::CreateSorghumEntity(sorghum_asset, seed);
}
Entity PyDigitalAgriculture::CreateEntityFromSorghumField(const Handle& sorghum_generator_handle, int seed) {
  const auto sorghum_asset = PyEvoEngine::GetAsset(sorghum_generator_handle);
  if (sorghum_asset->GetTypeName() != "SorghumField") {
    EVOENGINE_ERROR("CreateEntityFromSorghumField failed: invalid asset type!")
    return {};
  }
  return DatasetGenerator::CreateSorghumEntity(sorghum_asset, seed);
}

void PyDigitalAgriculture::ApplySorghumGrid(const Handle& sorghum_field_handle, const Handle& sorghum_generator_handle,
                                            const SorghumGrid& sorghum_grid) {
  const auto sorghum_field = PyEvoEngine::GetAsset(sorghum_field_handle);
  const auto sorghum_generator = PyEvoEngine::GetAsset(sorghum_generator_handle);
  DatasetGenerator::ApplySorghumGrid(sorghum_field, sorghum_generator, sorghum_grid);
}

void PyDigitalAgriculture::Initialize(pybind11::module& m) {
  PyEvoEngine::Initialize(m);
  m.def("RegisterClasses", &RegisterClasses);
  m.def("PushSorghumLayer", &PushSorghumLayer);
  m.def("CreateEntityFromSorghumState", &CreateEntityFromSorghumState);
  m.def("CreateEntityFromSorghumDescriptor", &CreateEntityFromSorghumDescriptor);
  m.def("CreateEntityFromSorghumGenerator", &CreateEntityFromSorghumGenerator);
  m.def("CreateEntityFromSorghumField", &CreateEntityFromSorghumField);
  m.def("ApplySorghumGrid", &ApplySorghumGrid);
  m.def("EnableBTF", &EnableBTF);
  m.def("CheckBTFComponentsExist", &CheckBTFComponentsExist);
  m.def("SetCBTFGroup", &SetCBTFGroup);
  m.def("SetSkyDome", &SetSkyDome);
  m.def("PushRayTracerLayer", &PushRayTracerLayer);
  m.def("SetSunDirection", &SetSunDirection);
  m.def("SetPARSensors", &SetPARSensors);
  m.def("IlluminationEstimationOnSensors", &IlluminationEstimationOnSensors);
  m.def("GetAllIlluminationEstimationResultsFromSensors", &GetAllIlluminationEstimationResultsFromSensors);
  m.def("IlluminationEstimationOnSorghum", &IlluminationEstimationOnSorghum);
  m.def("CheckTriangleEstimator", &CheckTriangleEstimator);
  m.def("GetAllIlluminationEstimationResultsOnSorghum", &GetAllIlluminationEstimationResultsOnSorghum);
  m.def("InstantiateSorghumField", &InstantiateSorghumField, py::arg("sorghum_field_handle"),
        py::arg("sorghum_coordinates"), py::arg("seed"), py::arg("index") = 200, py::arg("radius") = 2000.0f);
  m.def("SetIlluminationSamples", &SetIlluminationSamples);
  m.def("RunLSystemSorghumProject", &RunLSystemSorghumProject, py::arg("project_path"),
        py::arg("runtime_package_path") = std::filesystem::path{},
        py::arg("start_scene_path") = std::filesystem::path{}, py::arg("load_project_assets") = false);
  m.def("WaitForProjectIdle", &WaitForProjectIdle, py::arg("max_frames") = 30000);
  m.def("LoopFrames", &LoopFrames, py::arg("frames"));
  m.def("EnsureIlluminationSoilContext", &EnsureIlluminationSoilContext);
  m.def("ValidateIlluminationContext", &ValidateIlluminationContext);
  m.def("ConfigureSceneReviewLighting", &ConfigureSceneReviewLighting, py::arg("ambient_light_intensity") = 1.25f,
        py::arg("directional_light_brightness") = 1.0f, py::arg("cast_shadows") = false);
  m.def("ConfigureRayTracerSkydome", &ConfigureRayTracerSkydome,
        py::arg("sun_angles_degrees") = glm::vec3(55.0f, 30.0f, 0.0f),
        py::arg("sun_angular_diameter_radians") = EnvironmentProperties::kPhysicalSunAngularDiameterRadians,
        py::arg("sun_intensity") = 1.0f, py::arg("sun_color") = glm::vec3(1.0f), py::arg("skylight_intensity") = 1.0f,
        py::arg("ambient_light_intensity") = 0.1f, py::arg("gamma") = 2.2f);
  m.def("CaptureCurrentSceneRayTraced", &CaptureCurrentSceneRayTraced, py::arg("resolution_x"), py::arg("resolution_y"),
        py::arg("output_path"), py::arg("samples") = 64, py::arg("bounces") = 4, py::arg("gamma") = 2.2f);
  m.def("GrowSorghumLsPlantsToAdulthood", &GrowSorghumLsPlantsToAdulthood, py::arg("seed_base") = -1,
        py::arg("cultivar_filter") = "");
  m.def("SetSorghumLsLeafThickness", &SetSorghumLsLeafThickness, py::arg("leaf_thickness_m"),
        py::arg("regenerate_geometry") = true);
  m.def("SetSorghumLsLeafWidthScale", &SetSorghumLsLeafWidthScale, py::arg("leaf_width_scale"),
        py::arg("regenerate_geometry") = true);
  m.def("SetSorghumLsCultivarDescriptors", &SetSorghumLsCultivarDescriptors, py::arg("btx_descriptor_path"),
        py::arg("pawaga_descriptor_path"), py::arg("regenerate_geometry") = true, py::arg("seed_base") = -1);
  m.def("SetSorghumLsGridSpacing", &SetSorghumLsGridSpacing, py::arg("spacing_x"), py::arg("spacing_z"),
        py::arg("cultivar_filter") = "");
  m.def("RemoveSorghumLsPseudoTillerPlants", &RemoveSorghumLsPseudoTillerPlants);
  m.def("ConvertSorghumLsPlantsToPlantingMarkers", &ConvertSorghumLsPlantsToPlantingMarkers);
  m.def("InstantiateSorghumLsPlantsFromPlantingMarkers", &InstantiateSorghumLsPlantsFromPlantingMarkers);
  m.def("GetSorghumLsPlantSceneMetadata", &GetSorghumLsPlantSceneMetadata, py::arg("measure_geometry") = true);
  m.def("MoveParbarMiddlePanelsToPlantHeightFraction", &MoveParbarMiddlePanelsToPlantHeightFraction,
        py::arg("height_fraction") = 2.0f / 3.0f);
  m.def("CreateParbarTopFaceSensorGroup", &CreateParbarTopFaceSensorGroup, py::arg("samples_per_panel") = 100);
  m.def("EstimatePARSensors", &EstimatePARSensors, py::arg("sensor_group_handle"), py::arg("samples") = 64,
        py::arg("bounces") = 4, py::arg("push_normal_distance") = 0.001f, py::arg("seed") = 0);
  m.def("GetParbarTopFaceSensorResults", &GetParbarTopFaceSensorResults, py::arg("sensor_group_handle"),
        py::arg("samples_per_panel") = 100);
  m.def("EstimateSorghumLsGridIllumination", &EstimateSorghumLsGridIllumination, py::arg("samples") = 64,
        py::arg("bounces") = 4, py::arg("max_triangles_per_plant") = 0, py::arg("push_normal_distance") = 0.001f,
        py::arg("seed") = 0, py::arg("cultivar_filter") = "");
  m.def("SampleSorghumLsDescriptorPhenotypes", &SampleSorghumLsDescriptorPhenotypes, py::arg("base_descriptor_path"),
        py::arg("leaf_modules_mean"), py::arg("leaf_modules_deviation"), py::arg("length_mean_scale"),
        py::arg("length_deviation_scale"), py::arg("sample_count") = 1000, py::arg("seed_base") = 0,
        py::arg("leaf_width_scale") = 1.0f, py::arg("main_culm_diameter_m") = 0.0f,
        py::arg("tiller_leaf_count_ratio") = 0.90f, py::arg("tiller_height_ratio") = 0.90f);
  m.def("SaveCalibratedSorghumLsDescriptor", &SaveCalibratedSorghumLsDescriptor, py::arg("base_descriptor_path"),
        py::arg("output_descriptor_path"), py::arg("leaf_modules_mean"), py::arg("leaf_modules_deviation"),
        py::arg("length_mean_scale"), py::arg("length_deviation_scale"), py::arg("leaf_width_scale") = 1.0f,
        py::arg("main_culm_diameter_m") = 0.0f, py::arg("tiller_leaf_count_ratio") = 0.90f,
        py::arg("tiller_height_ratio") = 0.90f);
  m.def("SaveActiveSceneAsProjectAsset", &SaveActiveSceneAsProjectAsset, py::arg("scene_asset_path"));
  m.def("CreateEntityFromPrefab", &CreateEntityFromPrefab);

  py::class_<ParbarProbeRecord>(m, "ParbarProbeRecord")
      .def_readonly("cultivar", &ParbarProbeRecord::cultivar)
      .def_readonly("model", &ParbarProbeRecord::model)
      .def_readonly("sensor_bar_level", &ParbarProbeRecord::sensor_bar_level)
      .def_readonly("height_rule", &ParbarProbeRecord::height_rule)
      .def_readonly("row", &ParbarProbeRecord::row)
      .def_readonly("column", &ParbarProbeRecord::column)
      .def_readonly("represented_plant_count", &ParbarProbeRecord::represented_plant_count)
      .def_readonly("position", &ParbarProbeRecord::position)
      .def_readonly("normal", &ParbarProbeRecord::normal)
      .def_readonly("energy", &ParbarProbeRecord::energy)
      .def_readonly("direction", &ParbarProbeRecord::direction)
      .def_readonly("average_represented_root_elevation_m", &ParbarProbeRecord::average_represented_root_elevation_m)
      .def_readonly("average_represented_plant_height_m", &ParbarProbeRecord::average_represented_plant_height_m)
      .def_readonly("sensor_top_elevation_m", &ParbarProbeRecord::sensor_top_elevation_m)
      .def_readonly("height_fraction_of_average_height", &ParbarProbeRecord::height_fraction_of_average_height)
      .def_readonly("scalar", &ParbarProbeRecord::scalar)
      .def_readonly("normalized", &ParbarProbeRecord::normalized);

  py::class_<LSystemGridIlluminationRecord>(m, "LSystemGridIlluminationRecord")
      .def_readonly("name", &LSystemGridIlluminationRecord::name)
      .def_readonly("cultivar", &LSystemGridIlluminationRecord::cultivar)
      .def_readonly("row", &LSystemGridIlluminationRecord::row)
      .def_readonly("column", &LSystemGridIlluminationRecord::column)
      .def_readonly("position", &LSystemGridIlluminationRecord::position)
      .def_readonly("triangle_count", &LSystemGridIlluminationRecord::triangle_count)
      .def_readonly("leaf_triangle_count", &LSystemGridIlluminationRecord::leaf_triangle_count)
      .def_readonly("stem_triangle_count", &LSystemGridIlluminationRecord::stem_triangle_count)
      .def_readonly("area", &LSystemGridIlluminationRecord::area)
      .def_readonly("leaf_area", &LSystemGridIlluminationRecord::leaf_area)
      .def_readonly("stem_area", &LSystemGridIlluminationRecord::stem_area)
      .def_readonly("plant_height_m", &LSystemGridIlluminationRecord::plant_height_m)
      .def_readonly("total_flux", &LSystemGridIlluminationRecord::total_flux)
      .def_readonly("average_flux", &LSystemGridIlluminationRecord::average_flux)
      .def_readonly("scalar", &LSystemGridIlluminationRecord::scalar)
      .def_readonly("isolated_total_flux", &LSystemGridIlluminationRecord::isolated_total_flux)
      .def_readonly("isolated_average_flux", &LSystemGridIlluminationRecord::isolated_average_flux)
      .def_readonly("isolated_scalar", &LSystemGridIlluminationRecord::isolated_scalar)
      .def_readonly("retention_ratio", &LSystemGridIlluminationRecord::retention_ratio)
      .def_readonly("shadow_loss", &LSystemGridIlluminationRecord::shadow_loss)
      .def_readonly("normalized", &LSystemGridIlluminationRecord::normalized);

  py::class_<LSystemAxisPhenotypeRecord>(m, "LSystemAxisPhenotypeRecord")
      .def_readonly("axis_id", &LSystemAxisPhenotypeRecord::axis_id)
      .def_readonly("origin_rank", &LSystemAxisPhenotypeRecord::origin_rank)
      .def_readonly("leaf_count", &LSystemAxisPhenotypeRecord::leaf_count)
      .def_readonly("internode_count", &LSystemAxisPhenotypeRecord::internode_count)
      .def_readonly("culm_tip_height_m", &LSystemAxisPhenotypeRecord::culm_tip_height_m)
      .def_readonly("leaf_ratio_to_main", &LSystemAxisPhenotypeRecord::leaf_ratio_to_main)
      .def_readonly("height_ratio_to_main", &LSystemAxisPhenotypeRecord::height_ratio_to_main);

  py::class_<LSystemDescriptorPhenotypeRecord>(m, "LSystemDescriptorPhenotypeRecord")
      .def_readonly("seed", &LSystemDescriptorPhenotypeRecord::seed)
      .def_readonly("leaf_count", &LSystemDescriptorPhenotypeRecord::leaf_count)
      .def_readonly("live_leaf_count", &LSystemDescriptorPhenotypeRecord::live_leaf_count)
      .def_readonly("main_culm_leaf_count", &LSystemDescriptorPhenotypeRecord::main_culm_leaf_count)
      .def_readonly("tiller_leaf_count", &LSystemDescriptorPhenotypeRecord::tiller_leaf_count)
      .def_readonly("primary_tiller_count", &LSystemDescriptorPhenotypeRecord::primary_tiller_count)
      .def_readonly("triangle_count", &LSystemDescriptorPhenotypeRecord::triangle_count)
      .def_readonly("leaf_triangle_count", &LSystemDescriptorPhenotypeRecord::leaf_triangle_count)
      .def_readonly("stem_triangle_count", &LSystemDescriptorPhenotypeRecord::stem_triangle_count)
      .def_readonly("height_m", &LSystemDescriptorPhenotypeRecord::height_m)
      .def_readonly("area", &LSystemDescriptorPhenotypeRecord::area)
      .def_readonly("leaf_area", &LSystemDescriptorPhenotypeRecord::leaf_area)
      .def_readonly("stem_area", &LSystemDescriptorPhenotypeRecord::stem_area)
      .def_readonly("has_geometry", &LSystemDescriptorPhenotypeRecord::has_geometry)
      .def_readonly("axes", &LSystemDescriptorPhenotypeRecord::axes);

  py::class_<LSystemPlantSceneMetadataRecord>(m, "LSystemPlantSceneMetadataRecord")
      .def_readonly("name", &LSystemPlantSceneMetadataRecord::name)
      .def_readonly("cultivar", &LSystemPlantSceneMetadataRecord::cultivar)
      .def_readonly("local_position", &LSystemPlantSceneMetadataRecord::local_position)
      .def_readonly("global_position", &LSystemPlantSceneMetadataRecord::global_position)
      .def_readonly("geometry_min_position", &LSystemPlantSceneMetadataRecord::geometry_min_position)
      .def_readonly("geometry_max_position", &LSystemPlantSceneMetadataRecord::geometry_max_position)
      .def_readonly("leaf_count", &LSystemPlantSceneMetadataRecord::leaf_count)
      .def_readonly("main_culm_leaf_count", &LSystemPlantSceneMetadataRecord::main_culm_leaf_count)
      .def_readonly("tiller_leaf_count", &LSystemPlantSceneMetadataRecord::tiller_leaf_count)
      .def_readonly("primary_tiller_count", &LSystemPlantSceneMetadataRecord::primary_tiller_count)
      .def_readonly("leaf_width_scale", &LSystemPlantSceneMetadataRecord::leaf_width_scale)
      .def_readonly("leaf_thickness_m", &LSystemPlantSceneMetadataRecord::leaf_thickness_m)
      .def_readonly("plant_height_m", &LSystemPlantSceneMetadataRecord::plant_height_m)
      .def_readonly("leaf_area_m2", &LSystemPlantSceneMetadataRecord::leaf_area_m2)
      .def_readonly("middle_parbar_top_elevation_m", &LSystemPlantSceneMetadataRecord::middle_parbar_top_elevation_m)
      .def_readonly("has_geometry", &LSystemPlantSceneMetadataRecord::has_geometry)
      .def_readonly("axes", &LSystemPlantSceneMetadataRecord::axes);

  py::class_<SorghumMeshGeneratorSettings>(m, "SorghumMeshGeneratorSettings")
      .def(py::init<>())
      .def_readwrite("enable_panicle", &SorghumMeshGeneratorSettings::enable_panicle)
      .def_readwrite("enable_stem", &SorghumMeshGeneratorSettings::enable_stem)
      .def_readwrite("enable_leaves", &SorghumMeshGeneratorSettings::enable_leaves)
      .def_readwrite("enable_leaf_sheath", &SorghumMeshGeneratorSettings::enable_leaf_sheath)
      .def_readwrite("single_leaf_index", &SorghumMeshGeneratorSettings::single_leaf_index)
      .def_readwrite("bottom_face", &SorghumMeshGeneratorSettings::bottom_face)
      .def_readwrite("leaf_separated", &SorghumMeshGeneratorSettings::leaf_separated)
      .def_readwrite("leaf_thickness", &SorghumMeshGeneratorSettings::leaf_thickness);

  py::class_<SorghumPointCloudPointSettings>(m, "SorghumPointCloudPointSettings")
      .def(py::init<>())
      .def_readwrite("variance", &SorghumPointCloudPointSettings::variance)
      .def_readwrite("ball_rand_radius", &SorghumPointCloudPointSettings::ball_rand_radius)
      .def_readwrite("type_index", &SorghumPointCloudPointSettings::type_index)
      .def_readwrite("instance_index", &SorghumPointCloudPointSettings::instance_index)
      .def_readwrite("leaf_index", &SorghumPointCloudPointSettings::leaf_index)
      .def_readwrite("bounding_box_limit", &SorghumPointCloudPointSettings::bounding_box_limit);

  py::class_<DatasetGenerator::SorghumDataGenerationParameters>(m, "SorghumDataGenerationParameters")
      .def(py::init<>())
      .def_readwrite("export_point_cloud", &DatasetGenerator::SorghumDataGenerationParameters::export_point_cloud)
      .def_readwrite("export_mesh", &DatasetGenerator::SorghumDataGenerationParameters::export_mesh)

      .def_readwrite("generate_ground_mesh", &DatasetGenerator::SorghumDataGenerationParameters::generate_ground_mesh)
      .def_readwrite("avoid_occlusion", &DatasetGenerator::SorghumDataGenerationParameters::avoid_occlusion)

      .def_readwrite("sorghum_point_cloud_point_settings",
                     &DatasetGenerator::SorghumDataGenerationParameters::sorghum_point_cloud_point_settings)
      .def_readwrite("sorghum_mesh_generator_settings",
                     &DatasetGenerator::SorghumDataGenerationParameters::sorghum_mesh_generator_settings)
      .def_readwrite("output_folder", &DatasetGenerator::SorghumDataGenerationParameters::output_folder)
      .def_readwrite("output_file_name", &DatasetGenerator::SorghumDataGenerationParameters::output_file_name);

  py::class_<SorghumGantryCaptureSettings>(m, "SorghumGantryCaptureSettings")
      .def(py::init<>())
      .def_readwrite("bounding_box_size", &SorghumGantryCaptureSettings::bounding_box_size)
      .def_readwrite("grid_size", &SorghumGantryCaptureSettings::grid_size)
      .def_readwrite("grid_distance", &SorghumGantryCaptureSettings::grid_distance)
      .def_readwrite("step", &SorghumGantryCaptureSettings::step)
      .def_readwrite("output_spline_info", &SorghumGantryCaptureSettings::output_spline_info)
      .def_readwrite("sample_height", &SorghumGantryCaptureSettings::sample_height);

  py::class_<SorghumGrid>(m, "SorghumGrid")
      .def(py::init<>())
      .def_readwrite("grid_distance", &SorghumGrid::grid_distance)
      .def_readwrite("position_offset_mean", &SorghumGrid::position_offset_mean)
      .def_readwrite("position_offset_variance", &SorghumGrid::position_offset_variance)
      .def_readwrite("rotation_variance_xz", &SorghumGrid::rotation_variance_xz)
      .def_readwrite("rotation_variance_y", &SorghumGrid::rotation_variance_y)
      .def_readwrite("grid_size", &SorghumGrid::grid_size);
}

void PyDigitalAgriculture::EnableBTF() {
  auto sorghum_layer = ApplicationContext::Get().GetLayer<SorghumLayer>();
  sorghum_layer->enable_compressed_btf = true;

  EVOENGINE_LOG("Sorghum layer enabled: " << sorghum_layer->enable_compressed_btf)
}

void PyDigitalAgriculture::SetCBTFGroup(const Handle& cbtf_group_handle) {
  const auto cbtf_group_asset = PyEvoEngine::GetAsset(cbtf_group_handle);
  if (cbtf_group_asset->GetTypeName() != "CBTFGroup") {
    EVOENGINE_ERROR("SetCBTFGroup failed: invalid asset type!")
  }
  auto sorghum_layer = ApplicationContext::Get().GetLayer<SorghumLayer>();
  sorghum_layer->leaf_cbtf_group = cbtf_group_asset;

  EVOENGINE_LOG("Sorghum layer set leaf cbtf group: " << sorghum_layer->leaf_cbtf_group.GetAssetHandle())
}

bool PyDigitalAgriculture::CheckBTFComponentsExist() {
  const auto scene = ApplicationContext::Get().GetActiveScene();
  EVOENGINE_LOG("!scene " << !scene)
  const auto owners = scene->GetPrivateComponentOwnersList<BtfMeshRenderer>();

  EVOENGINE_LOG("btf material count: " << owners.size())

  return !owners.empty();
}

void PyDigitalAgriculture::SetSkyDome() {
  auto ray_tracer_layer = ApplicationContext::Get().GetLayer<RayTracerLayer>();
  ray_tracer_layer->environment_properties.environmental_lighting_type = EnvironmentalLightingType::Skydome;

  EVOENGINE_LOG("sky info: " << ray_tracer_layer->environment_properties.sun_direction.x << ","
                             << ray_tracer_layer->environment_properties.sun_direction.y << ","
                             << ray_tracer_layer->environment_properties.sun_direction.z)
}

void PyDigitalAgriculture::PushRayTracerLayer() {
  ApplicationContext::Get().PushLayer<RayTracerLayer>("Ray Tracer Layer");
}

void PyDigitalAgriculture::SetSunDirection(glm::vec3 angles) {
  // adopted from RayTracerLayer::SetSunDirection, note the order of angles is (x, y, z) = (pitch, yaw, roll)
  auto ray_tracer_layer = ApplicationContext::Get().GetLayer<RayTracerLayer>();
  glm::vec3 sun_direction = glm::quat(glm::radians(angles)) * glm::vec3(0, 0, -1);
  ray_tracer_layer->environment_properties.sun_direction = sun_direction;
}

void PyDigitalAgriculture::IlluminationEstimationOnSorghum() {
  auto scene = ApplicationContext::Get().GetActiveScene();
  auto sorghum_layer = ApplicationContext::Get().GetLayer<SorghumLayer>();

  sorghum_layer->CalculateIllumination();
}

void PyDigitalAgriculture::CheckTriangleEstimator(const Entity& sorghum_entity) {
  auto scene = ApplicationContext::Get().GetActiveScene();
  auto triangle_illumination_estimator =
      scene->GetOrSetPrivateComponent<TriangleIlluminationEstimator>(sorghum_entity).lock();

  triangle_illumination_estimator->PrepareLightProbeGroup();
  EVOENGINE_LOG("triangle illumination estimator total light probes: "
                << triangle_illumination_estimator->GetLightProbeGroup().light_probes.size())
}

Entity PyDigitalAgriculture::InstantiateSorghumField(const Handle& sorghum_field_handle,
                                                     const Handle& sorghum_coordinates, const int seed, const int index,
                                                     const float radius) {
  const auto field_asset = PyEvoEngine::GetAsset(sorghum_field_handle);
  if (field_asset->GetTypeName() != "SorghumField") {
    EVOENGINE_ERROR("Instantiate SorghumField failed: invalid asset type!")
    return {};
  }
  const auto coordinates_data = PyEvoEngine::GetAsset(sorghum_coordinates);

  const auto sorghum_field = std::dynamic_pointer_cast<SorghumField>(field_asset);

  const auto coordinates = std::dynamic_pointer_cast<SorghumCoordinates>(coordinates_data);
  glm::dvec2 offset;

  coordinates->Apply(sorghum_field, offset, index, radius);

  EVOENGINE_LOG("Instantiate SorghumField:" << sorghum_field->matrices.size())

  auto field_entity = DatasetGenerator::CreateSorghumEntity(field_asset, seed);

  auto scene = ApplicationContext::Get().GetActiveScene();
  const std::vector<Entity>* leaf_mesh_list = scene->UnsafeGetPrivateComponentOwnersList<BtfMeshRenderer>();

  EVOENGINE_LOG("btfMeshRenderer count : " << leaf_mesh_list->size())

  return field_entity;
}

std::vector<std::vector<glm::vec3>> PyDigitalAgriculture::GetAllIlluminationEstimationResultsOnSorghum() {
  auto scene = ApplicationContext::Get().GetActiveScene();
  auto sorghum_layer = ApplicationContext::Get().GetLayer<SorghumLayer>();
  const std::vector<Entity>* sorghum_entities =
      scene->UnsafeGetPrivateComponentOwnersList<TriangleIlluminationEstimator>();
  std::vector<std::vector<glm::vec3>> results;

  // todo: maybe need to bind the result to a specific sorghum

  for (const auto& sorghum : *sorghum_entities) {
    auto triangle_illumination_estimator =
        scene->GetOrSetPrivateComponent<TriangleIlluminationEstimator>(sorghum).lock();
    auto transform = scene->GetDataComponent<Transform>(sorghum);

    std::vector<glm::vec3> result;
    // sorghum position
    result.emplace_back(transform.GetPosition());
    // sorghum rotation
    result.emplace_back(transform.GetEulerRotation());
    // estimator area
    result.emplace_back(glm::vec3(triangle_illumination_estimator->total_area));
    // total flux
    result.emplace_back(triangle_illumination_estimator->total_flux);
    // average flux
    result.emplace_back(triangle_illumination_estimator->average_flux);

    results.emplace_back(result);
  }
  return results;
}

// todo: the height is fixed now
// todo: need to rotate?
// given the sorghum field prepare the PARSensor group according to the sorghums / bounding boxes
Handle PyDigitalAgriculture::SetPARSensors(const Entity& sorghum_field) {
  auto sensor_group_handle = PyEvoEngine::CreateRuntimeAsset("PARSensorGroup");

  auto sensor_group_asset = PyEvoEngine::GetAsset(sensor_group_handle);
  const auto sensors = std::dynamic_pointer_cast<PARSensorGroup>(sensor_group_asset);

  auto& samplers = sensors->samplers;

  auto scene = ApplicationContext::Get().GetActiveScene();
  const std::vector<Entity>* leaf_mesh_list = scene->UnsafeGetPrivateComponentOwnersList<BtfMeshRenderer>();

  EVOENGINE_LOG("btfMeshRenderer count : " << leaf_mesh_list->size())

  Bound overall_bound;
  for (const auto& btf_mesh_renderer_entity : *leaf_mesh_list) {
    const auto transform = scene->GetDataComponent<GlobalTransform>(btf_mesh_renderer_entity).value;
    auto btf_mesh_renderer = scene->GetOrSetPrivateComponent<BtfMeshRenderer>(btf_mesh_renderer_entity).lock();
    auto bound = btf_mesh_renderer->mesh.Get<Mesh>()->GetBound();

    glm::vec3 T = glm::vec3(transform[3]);  // glm::column(transform, 3)

    glm::vec3 wmin = bound.min + T;
    glm::vec3 wmax = bound.max + T;

    overall_bound.min = glm::min(overall_bound.min, wmin);
    overall_bound.max = glm::max(overall_bound.max, wmax);
  }

  // directly clean offset caused by float numbers
  overall_bound.min.y = 0;

  EVOENGINE_LOG("overall_bound: " << overall_bound.min.x << "," << overall_bound.min.y << "," << overall_bound.min.z
                                  << ";" << overall_bound.max.x << "," << overall_bound.max.y << ","
                                  << overall_bound.max.z)

  // not to include boundaries of the bounding box
  float step = 0.8f;
  glm::vec3 max_range(overall_bound.max - glm::vec3(step, 0, step));
  glm::vec3 min_range(overall_bound.min + glm::vec3(step, 0, step));

  constexpr float above_canopy_height = 2.4f;
  constexpr float at_canopy_height = 0.8f;
  constexpr float below_canopy_height = 0.4f;

  // step y is set to 0.3 for now
  const int sx = static_cast<int>((max_range.x - min_range.x + step) / step);
  const int sy = static_cast<int>((2.4f - min_range.y + 0.4) / 0.4);
  const int sz = static_cast<int>((max_range.z - min_range.z + step) / step);
  const auto voxel_size = sx * sy * sz;
  samplers.resize(voxel_size);
  Jobs::RunParallelFor(voxel_size, [&](unsigned i) {
    float z = (i % sz) * step + min_range.z;
    float y = ((i / sz) % sy) * 0.4f + min_range.y;
    float x = ((i / sz / sy) % sx) * step + min_range.x;
    glm::vec3 start = {x, y, z};
    samplers[i].v_0.position = samplers[i].v_1.position = samplers[i].v_2.position = start;
    samplers[i].front_face = true;
    samplers[i].back_face = false;
    samplers[i].v_0.normal = samplers[i].v_1.normal = samplers[i].v_2.normal = glm::vec3(0, 1, 0);
  });
  EVOENGINE_LOG("sensor counts: " << sensors->samplers.size())
  return sensor_group_handle;
}

void PyDigitalAgriculture::IlluminationEstimationOnSensors(const Handle& sensor_group_handle) {
  const auto sensor_group_asset = PyEvoEngine::GetAsset(sensor_group_handle);

  const auto sensors = std::dynamic_pointer_cast<PARSensorGroup>(sensor_group_asset);

  const auto sorghum_layer = ApplicationContext::Get().GetLayer<SorghumLayer>();
  sensors->CalculateIllumination(sorghum_layer->ray_properties, sorghum_layer->m_seed, sorghum_layer->push_distance);
}

std::vector<std::vector<glm::vec3>> PyDigitalAgriculture::GetAllIlluminationEstimationResultsFromSensors(
    const Handle& sensor_group_handle) {
  const auto sensor_group_asset = PyEvoEngine::GetAsset(sensor_group_handle);

  const auto sensors = std::dynamic_pointer_cast<PARSensorGroup>(sensor_group_asset);

  auto& samplers = sensors->samplers;

  std::vector<std::vector<glm::vec3>> results;

  // todo: maybe need to bind the result to a specific sorghum

  for (const auto& sampler : samplers) {
    std::vector<glm::vec3> result;
    // sampler position (3 vertices are at the same position for now)
    result.emplace_back(sampler.v_0.position);

    // energy
    result.emplace_back(sampler.energy);

    // energy dominant direction
    result.emplace_back(sampler.direction);

    results.emplace_back(result);
  }
  return results;
}

void PyDigitalAgriculture::SetIlluminationSamples(int samples, int bounces) {
  const auto sorghum_layer = ApplicationContext::Get().GetLayer<SorghumLayer>();
  sorghum_layer->ray_properties.samples = samples;
  sorghum_layer->ray_properties.bounces = bounces;
}

bool PyDigitalAgriculture::RunLSystemSorghumProject(const std::filesystem::path& project_path,
                                                    const std::filesystem::path& runtime_package_path,
                                                    const std::filesystem::path& start_scene_path,
                                                    const bool load_project_assets) {
  if (std::filesystem::path(project_path).extension().string() != ".eveproj") {
    EVOENGINE_ERROR("Project path doesn't point to a EvoEngine project!")
    return false;
  }

  auto& application = PyEvoEngine::GetRuntime().GetApplication();
  if (!ApplicationContext::Get().GetLayer<RenderLayer>()) {
    ApplicationContext::Get().PushLayer<RenderLayer>("Render Layer");
  }
#  ifdef CUDA_MODULE_SERVICE
  if (!ApplicationContext::Get().GetLayer<RayTracerLayer>()) {
    ApplicationContext::Get().PushLayer<RayTracerLayer>("Ray Tracer Layer");
  }
#  endif

  const bool has_start_scene_override = !start_scene_path.empty() && start_scene_path != ".";
  ApplicationInitializationSettings application_info{};
  application_info.project_path = project_path;
  application_info.application_name = "DigitalAgriculture";
  application_info.full_screen = false;
  application_info.load_project_assets = load_project_assets;
  application_info.enable_runtime_packages = true;
  application_info.startup_runtime_packages = {"DigitalAgriculture", "LSystem"};
  if (!runtime_package_path.empty()) {
    application_info.package_search_paths.emplace_back(runtime_package_path);
  }
  ApplicationContext::Get().Initialize(application_info);
  application.Start(false);
  if (has_start_scene_override) {
    if (!WaitForProjectIdle(30000)) {
      EVOENGINE_ERROR("RunLSystemSorghumProject failed: project did not become idle before scene override")
      return false;
    }
    const auto scene = std::dynamic_pointer_cast<Scene>(ProjectManager::GetOrCreateAsset(start_scene_path));
    if (!scene) {
      EVOENGINE_ERROR("RunLSystemSorghumProject failed: start_scene_path does not point to a Scene")
      return false;
    }
    ProjectManager::SetStartScene(scene);
    application.Attach(scene);
    TransformGraph::CalculateTransformGraphs(scene);
  }
#  ifdef CUDA_MODULE_SERVICE
  if (const auto ray_tracer_layer = ApplicationContext::Get().GetLayer<RayTracerLayer>()) {
    ray_tracer_layer->environment_properties.environmental_lighting_type = EnvironmentalLightingType::Skydome;
  }
#  endif
  return true;
}

bool PyDigitalAgriculture::WaitForProjectIdle(const int max_frames) {
  const auto scene_ready = []() {
    return ProjectManager::IsProjectIdle() && !GeometryStorage::HasPendingUploads() &&
           !TextureStorage::HasPendingUploads();
  };
  auto& application = ApplicationContext::Get();
  const int frame_limit = std::max(1, max_frames);
  for (int frame = 0; frame < frame_limit; ++frame) {
    if (scene_ready()) {
      return true;
    }
    if (!application.Loop()) {
      return false;
    }
  }
  return scene_ready();
}

void PyDigitalAgriculture::LoopFrames(const int frames) {
  auto& application = ApplicationContext::Get();
  for (int frame = 0; frame < frames; ++frame) {
    if (!application.Loop()) {
      return;
    }
  }
}

bool PyDigitalAgriculture::EnsureIlluminationSoilContext() {
  const auto scene = ApplicationContext::Get().GetActiveScene();
  if (!scene) {
    EVOENGINE_ERROR("EnsureIlluminationSoilContext failed: no active scene")
    return false;
  }

  const auto soil_descriptor = ProjectManager::GetOrCreateAsset(kIlluminationSoilDescriptorPath);
  const auto ground_mesh =
      std::dynamic_pointer_cast<Mesh>(ProjectManager::GetOrCreateAsset(kIlluminationGroundMeshPath));
  const auto soil_material =
      std::dynamic_pointer_cast<Material>(ProjectManager::GetOrCreateAsset(kIlluminationSoilMaterialPath));
  if (!soil_descriptor || !ground_mesh || !soil_material) {
    EVOENGINE_ERROR("EnsureIlluminationSoilContext failed: could not resolve hardcoded soil assets")
    return false;
  }

  Entity soil_entity = FindEntityByName(scene, "bigSoil *");
  if (!scene->IsEntityValid(soil_entity)) {
    soil_entity = scene->CreateEntity("bigSoil *");
  }
  scene->SetEnable(soil_entity, true);
  scene->SetEntitySerializable(soil_entity, true);
  const auto soil = scene->GetOrSetPrivateComponent<Soil>(soil_entity).lock();
  if (!soil) {
    EVOENGINE_ERROR("EnsureIlluminationSoilContext failed: could not create Soil component")
    return false;
  }
  soil->soil_descriptor_ref = soil_descriptor;

  Entity ground_entity = {};
  for (const auto& child : scene->GetChildren(soil_entity)) {
    if (scene->IsEntityValid(child) && scene->GetEntityName(child) == "Ground Mesh" &&
        scene->HasPrivateComponent<MeshRenderer>(child)) {
      ground_entity = child;
      break;
    }
  }
  if (!scene->IsEntityValid(ground_entity)) {
    ground_entity = FindEntityByName(scene, "Ground Mesh");
  }
  if (!scene->IsEntityValid(ground_entity)) {
    ground_entity = scene->CreateEntity("Ground Mesh");
  }
  scene->SetParent(ground_entity, soil_entity, true);
  scene->SetEnable(ground_entity, true);
  scene->SetEntitySerializable(ground_entity, true);
  const auto renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(ground_entity).lock();
  if (!renderer) {
    EVOENGINE_ERROR("EnsureIlluminationSoilContext failed: could not create Ground Mesh renderer")
    return false;
  }
  renderer->cast_shadow = true;
  renderer->mesh = ground_mesh;
  renderer->material = soil_material;
  ground_mesh->ray_tracing_acceleration_enabled = true;
  TransformGraph::CalculateTransformGraphs(scene);
  return true;
}

bool PyDigitalAgriculture::ValidateIlluminationContext() {
  const auto scene = ApplicationContext::Get().GetActiveScene();
  if (!scene) {
    EVOENGINE_ERROR("ValidateIlluminationContext failed: no active scene")
    return false;
  }
  const Entity ground_entity = FindEntityByName(scene, "Ground Mesh");
  if (!scene->IsEntityValid(ground_entity) || !scene->IsEntityEnabled(ground_entity) ||
      !scene->HasPrivateComponent<MeshRenderer>(ground_entity)) {
    EVOENGINE_ERROR("ValidateIlluminationContext failed: missing enabled Ground Mesh MeshRenderer")
    return false;
  }
  const auto renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(ground_entity).lock();
  const auto mesh = renderer ? renderer->mesh.Get<Mesh>() : nullptr;
  const auto material = renderer ? renderer->material.Get<Material>() : nullptr;
  if (!mesh || mesh->UnsafeGetVertices().empty() || !material) {
    EVOENGINE_ERROR("ValidateIlluminationContext failed: Ground Mesh has unresolved mesh or material")
    return false;
  }
  if (!material->GetAlbedoTexture() || !material->GetNormalTexture() || !material->GetRoughnessTexture()) {
    EVOENGINE_ERROR("ValidateIlluminationContext failed: PBR soil material is missing CUDA-supported textures")
    return false;
  }
  return true;
}

size_t PyDigitalAgriculture::ConfigureSceneReviewLighting(const float ambient_light_intensity,
                                                          const float directional_light_brightness,
                                                          const bool cast_shadows) {
  const auto scene = ApplicationContext::Get().GetActiveScene();
  if (!scene) {
    return 0;
  }
  scene->environment.ambient_light_intensity = std::max(0.0f, ambient_light_intensity);

  size_t light_count = 0;
  if (const auto* light_entities = scene->UnsafeGetPrivateComponentOwnersList<DirectionalLight>()) {
    for (const auto& entity : *light_entities) {
      if (const auto light = scene->GetOrSetPrivateComponent<DirectionalLight>(entity).lock()) {
        light->diffuse_brightness = std::max(0.0f, directional_light_brightness);
        light->cast_shadow = cast_shadows;
        light_count++;
      }
    }
  }
  return light_count;
}

bool PyDigitalAgriculture::ConfigureRayTracerSkydome(const glm::vec3 sun_angles_degrees,
                                                     const float sun_angular_diameter_radians,
                                                     const float sun_intensity, const glm::vec3 sun_color,
                                                     const float skylight_intensity,
                                                     const float ambient_light_intensity, const float gamma) {
#  ifdef CUDA_MODULE_SERVICE
  const auto layer = ApplicationContext::Get().GetLayer<RayTracerLayer>();
  if (!layer || !std::isfinite(sun_angular_diameter_radians) || !std::isfinite(sun_intensity) ||
      !std::isfinite(skylight_intensity) || !std::isfinite(ambient_light_intensity) || !std::isfinite(gamma)) {
    return false;
  }
  auto& environment = layer->environment_properties;
  environment.environmental_lighting_type = EnvironmentalLightingType::Skydome;
  environment.sun_direction = glm::quat(glm::radians(sun_angles_degrees)) * glm::vec3(0.0f, 0.0f, -1.0f);
  environment.sun_angular_diameter_radians = glm::clamp(sun_angular_diameter_radians, 0.0f, glm::pi<float>());
  environment.sun_intensity = std::max(0.0f, sun_intensity);
  environment.sun_color = glm::max(sun_color, glm::vec3(0.0f));
  environment.skylight_intensity = std::max(0.0f, skylight_intensity);
  environment.ambient_light_intensity = std::max(0.0f, ambient_light_intensity);
  environment.gamma = std::max(0.01f, gamma);
  return true;
#  else
  return false;
#  endif
}

bool PyDigitalAgriculture::CaptureCurrentSceneRayTraced(const int resolution_x, const int resolution_y,
                                                        const std::filesystem::path& output_path, const int samples,
                                                        const int bounces, const float gamma) {
#  ifdef CUDA_MODULE_SERVICE
  if (resolution_x <= 0 || resolution_y <= 0 || samples <= 0 || bounces < 0 || gamma <= 0.0f) {
    return false;
  }
  auto& application = ApplicationContext::Get();
  const auto scene = application.GetActiveScene();
  const auto layer = application.GetLayer<RayTracerLayer>();
  const auto main_camera = scene ? scene->main_camera.Get<Camera>() : nullptr;
  if (!scene || !layer || !main_camera || CudaModule::GetRayTracer()->instances.empty()) {
    return false;
  }

  Entity camera_entity = FindEntityByName(scene, "__PythonRayTracerCaptureCamera");
  if (!scene->IsEntityValid(camera_entity)) {
    camera_entity = scene->CreateEntity("__PythonRayTracerCaptureCamera");
  }
  scene->SetDataComponent(camera_entity, scene->GetDataComponent<GlobalTransform>(main_camera->GetOwner()));
  const auto camera = scene->GetOrSetPrivateComponent<RayTracerCamera>(camera_entity).lock();
  camera->allow_auto_resize = false;
  camera->frame_size = glm::uvec2(resolution_x, resolution_y);
  camera->ApplyCameraSettings(main_camera->camera_settings);
  camera->ray_properties.samples = samples;
  camera->ray_properties.bounces = bounces;
  camera->SetGamma(gamma);
  camera->Render(camera->ray_properties, layer->environment_properties);

  if (const auto parent = output_path.parent_path(); !parent.empty()) {
    std::filesystem::create_directories(parent);
  }
  camera->render_texture->StoreToPng(output_path);
  return std::filesystem::exists(output_path) && std::filesystem::file_size(output_path) > 0;
#  else
  return false;
#  endif
}

size_t PyDigitalAgriculture::GrowSorghumLsPlantsToAdulthood(const int seed_base, const std::string& cultivar_filter) {
  const auto scene = ApplicationContext::Get().GetActiveScene();
  if (!scene) {
    return 0;
  }

  size_t plant_count = 0;
  uint32_t reseed_index = 0;
  if (const auto* sorghum_entities_ptr = scene->UnsafeGetPrivateComponentOwnersList<SorghumLS>()) {
    const std::vector<Entity> sorghum_entities = *sorghum_entities_ptr;
    for (const auto& entity : sorghum_entities) {
      if (!scene->IsEntityValid(entity)) {
        continue;
      }
      if (!MatchesCultivarFilter(CultivarFromPlantName(scene->GetEntityName(entity)), cultivar_filter)) {
        continue;
      }
      const auto sorghum = scene->GetOrSetPrivateComponent<SorghumLS>(entity).lock();
      if (!sorghum) {
        continue;
      }
      if (seed_base >= 0) {
        sorghum->seed = static_cast<uint32_t>(seed_base) + reseed_index;
      }
      auto descriptor = sorghum->descriptor_ref.Get<SorghumLSDescriptor>();
      if (!descriptor) {
        descriptor = ReferenceDescriptorForCultivar(CultivarFromPlantName(scene->GetEntityName(entity)));
        if (descriptor) {
          sorghum->descriptor_ref = descriptor;
        }
      }
      if (descriptor) {
        std::mt19937 rng(sorghum->seed);
        sorghum->target_gdd = std::max(0.0f, SampleDistribution(descriptor->target_gdd, rng));
      }
      sorghum->GenerateGeometryEntities(true);
      plant_count++;
      reseed_index++;
    }
  }
  if (plant_count > 0) {
    TransformGraph::CalculateTransformGraphs(scene);
  }
  return plant_count;
}

size_t PyDigitalAgriculture::SetSorghumLsLeafThickness(const float leaf_thickness_m, const bool regenerate_geometry) {
  const auto scene = ApplicationContext::Get().GetActiveScene();
  if (!scene) {
    return 0;
  }

  size_t plant_count = 0;
  const float thickness = std::max(0.0f, leaf_thickness_m);
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
      sorghum->leaf_mesh_settings.leaf_thickness = thickness;
      if (regenerate_geometry) {
        sorghum->GenerateGeometryEntities(true);
      }
      plant_count++;
    }
  }
  if (plant_count > 0 && regenerate_geometry) {
    TransformGraph::CalculateTransformGraphs(scene);
  }
  return plant_count;
}

size_t PyDigitalAgriculture::SetSorghumLsLeafWidthScale(const float leaf_width_scale, const bool regenerate_geometry) {
  const auto scene = ApplicationContext::Get().GetActiveScene();
  if (!scene) {
    return 0;
  }

  size_t plant_count = 0;
  const float scale = std::max(0.0f, leaf_width_scale);
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
      sorghum->leaf_mesh_settings.leaf_width_scale = scale;
      if (regenerate_geometry) {
        sorghum->GenerateGeometryEntities(true);
      }
      plant_count++;
    }
  }
  if (plant_count > 0 && regenerate_geometry) {
    TransformGraph::CalculateTransformGraphs(scene);
  }
  return plant_count;
}

size_t PyDigitalAgriculture::SetSorghumLsCultivarDescriptors(const std::filesystem::path& btx_descriptor_path,
                                                             const std::filesystem::path& pawaga_descriptor_path,
                                                             const bool regenerate_geometry, const int seed_base) {
  const auto scene = ApplicationContext::Get().GetActiveScene();
  if (!scene) {
    return 0;
  }
  const auto btx_descriptor =
      std::dynamic_pointer_cast<SorghumLSDescriptor>(ProjectManager::GetOrCreateAsset(btx_descriptor_path));
  const auto pawaga_descriptor =
      std::dynamic_pointer_cast<SorghumLSDescriptor>(ProjectManager::GetOrCreateAsset(pawaga_descriptor_path));
  if (!btx_descriptor || !pawaga_descriptor) {
    EVOENGINE_ERROR("SetSorghumLsCultivarDescriptors failed: missing calibrated descriptor asset")
    return 0;
  }

  size_t plant_count = 0;
  uint32_t reseed_index = 0;
  if (const auto* sorghum_entities_ptr = scene->UnsafeGetPrivateComponentOwnersList<SorghumLS>()) {
    const std::vector<Entity> sorghum_entities = *sorghum_entities_ptr;
    for (const auto& entity : sorghum_entities) {
      if (!scene->IsEntityValid(entity)) {
        continue;
      }
      const std::string cultivar = CultivarFromPlantName(scene->GetEntityName(entity));
      const auto descriptor = cultivar == "BTX" ? btx_descriptor : cultivar == "Pawaga" ? pawaga_descriptor : nullptr;
      if (!descriptor) {
        continue;
      }
      const auto sorghum = scene->GetOrSetPrivateComponent<SorghumLS>(entity).lock();
      if (!sorghum) {
        continue;
      }
      sorghum->descriptor_ref = descriptor;
      if (seed_base >= 0) {
        sorghum->seed = static_cast<uint32_t>(seed_base) + reseed_index;
      }
      if (regenerate_geometry) {
        std::mt19937 rng(sorghum->seed);
        sorghum->target_gdd = std::max(0.0f, SampleDistribution(descriptor->target_gdd, rng));
        sorghum->GenerateGeometryEntities(true);
      }
      plant_count++;
      reseed_index++;
    }
  }
  if (plant_count > 0 && regenerate_geometry) {
    TransformGraph::CalculateTransformGraphs(scene);
  }
  return plant_count;
}

size_t PyDigitalAgriculture::SetSorghumLsGridSpacing(const float spacing_x, const float spacing_z,
                                                     const std::string& cultivar_filter) {
  const auto scene = ApplicationContext::Get().GetActiveScene();
  if (!scene) {
    return 0;
  }
  const auto* sorghum_entities_ptr = scene->UnsafeGetPrivateComponentOwnersList<SorghumLS>();
  if (!sorghum_entities_ptr || sorghum_entities_ptr->empty()) {
    return 0;
  }

  std::vector<LSystemTriangleTarget> targets;
  targets.reserve(sorghum_entities_ptr->size());
  glm::vec3 position_sum(0.0f);
  float row_sum = 0.0f;
  float column_sum = 0.0f;
  for (const auto& plant : *sorghum_entities_ptr) {
    if (!scene->IsEntityValid(plant)) {
      continue;
    }
    LSystemTriangleTarget target;
    target.plant = plant;
    target.name = scene->GetEntityName(plant);
    target.cultivar = CultivarFromPlantName(target.name);
    if (!MatchesCultivarFilter(target.cultivar, cultivar_filter)) {
      continue;
    }
    if (!TryParseGridCoordinate(target.name, target.row, target.column)) {
      continue;
    }
    target.position = scene->GetDataComponent<GlobalTransform>(plant).GetPosition();
    position_sum += target.position;
    row_sum += static_cast<float>(target.row);
    column_sum += static_cast<float>(target.column);
    targets.emplace_back(target);
  }
  if (targets.empty()) {
    return 0;
  }

  const float count = static_cast<float>(targets.size());
  const glm::vec3 center = position_sum / count;
  const float center_row = row_sum / count;
  const float center_column = column_sum / count;
  for (const auto& target : targets) {
    const glm::vec3 desired_position(center.x + (static_cast<float>(target.column) - center_column) * spacing_x,
                                     target.position.y,
                                     center.z + (static_cast<float>(target.row) - center_row) * spacing_z);
    auto transform = scene->GetDataComponent<Transform>(target.plant);
    transform.SetPosition(transform.GetPosition() + desired_position - target.position);
    scene->SetDataComponent(target.plant, transform);
  }
  TransformGraph::CalculateTransformGraphs(scene);
  return targets.size();
}

size_t PyDigitalAgriculture::RemoveSorghumLsPseudoTillerPlants() {
  const auto scene = ApplicationContext::Get().GetActiveScene();
  if (!scene) {
    return 0;
  }
  const auto* sorghum_entities_ptr = scene->UnsafeGetPrivateComponentOwnersList<SorghumLS>();
  if (!sorghum_entities_ptr) {
    return 0;
  }
  std::vector<Entity> pseudo_tillers;
  for (const auto& plant : *sorghum_entities_ptr) {
    if (scene->IsEntityValid(plant) && scene->GetEntityName(plant).find("_cluster_") != std::string::npos) {
      pseudo_tillers.emplace_back(plant);
    }
  }
  for (const auto& plant : pseudo_tillers) {
    scene->DeleteEntity(plant);
  }
  if (!pseudo_tillers.empty()) {
    TransformGraph::CalculateTransformGraphs(scene, false);
  }
  return pseudo_tillers.size();
}

size_t PyDigitalAgriculture::ConvertSorghumLsPlantsToPlantingMarkers() {
  const auto scene = ApplicationContext::Get().GetActiveScene();
  if (!scene) {
    return 0;
  }
  if (const auto* owners = scene->UnsafeGetPrivateComponentOwnersList<SorghumLS>()) {
    const std::vector<Entity> plants = *owners;
    for (const auto& plant : plants) {
      if (!scene->IsEntityValid(plant)) {
        continue;
      }
      if (scene->GetEntityName(plant).find("_cluster_") != std::string::npos) {
        scene->DeleteEntity(plant);
        continue;
      }
      if (const auto sorghum = scene->GetOrSetPrivateComponent<SorghumLS>(plant).lock()) {
        sorghum->ClearGeometryEntities();
      }
      scene->RemovePrivateComponent<SorghumLS>(plant);
    }
  }
  size_t marker_count = 0;
  for (const auto& entity : scene->UnsafeGetAllEntities()) {
    marker_count += scene->IsEntityValid(entity) && IsSorghumPlantingMarkerName(scene->GetEntityName(entity));
  }
  TransformGraph::CalculateTransformGraphs(scene, false);
  return marker_count;
}

size_t PyDigitalAgriculture::InstantiateSorghumLsPlantsFromPlantingMarkers() {
  const auto scene = ApplicationContext::Get().GetActiveScene();
  if (!scene) {
    return 0;
  }
  size_t plant_count = 0;
  for (const auto& entity : scene->UnsafeGetAllEntities()) {
    if (!scene->IsEntityValid(entity) || !IsSorghumPlantingMarkerName(scene->GetEntityName(entity))) {
      continue;
    }
    scene->GetOrSetPrivateComponent<SorghumLS>(entity);
    ++plant_count;
  }
  return plant_count;
}

std::vector<LSystemPlantSceneMetadataRecord> PyDigitalAgriculture::GetSorghumLsPlantSceneMetadata(
    const bool measure_geometry) {
  std::vector<LSystemPlantSceneMetadataRecord> records;
  const auto scene = ApplicationContext::Get().GetActiveScene();
  if (!scene) {
    return records;
  }
  const auto* sorghum_entities_ptr = scene->UnsafeGetPrivateComponentOwnersList<SorghumLS>();
  if (!sorghum_entities_ptr) {
    return records;
  }

  std::map<std::string, float> middle_panel_elevations;
  if (measure_geometry) {
    for (const auto& panel : kParbarPanels) {
      if (IsMiddleParbarPanel(panel)) {
        middle_panel_elevations[panel.cultivar] =
            TopFaceElevation(CollectTopFaceTriangles(scene, FindParbarMeshEntity(scene, panel)));
      }
    }
  }

  records.reserve(sorghum_entities_ptr->size());
  for (const auto& plant : *sorghum_entities_ptr) {
    if (!scene->IsEntityValid(plant)) {
      continue;
    }
    LSystemPlantSceneMetadataRecord record;
    record.name = scene->GetEntityName(plant);
    record.cultivar = CultivarFromPlantName(record.name);
    record.local_position = scene->GetDataComponent<Transform>(plant).GetPosition();
    record.global_position = scene->GetDataComponent<GlobalTransform>(plant).GetPosition();
    if (const auto sorghum = scene->GetOrSetPrivateComponent<SorghumLS>(plant).lock()) {
      record.leaf_count = sorghum->last_leaf_count;
      const auto architecture = MeasureSorghumArchitecture(*sorghum);
      record.main_culm_leaf_count = architecture.main_culm_leaf_count;
      record.tiller_leaf_count = architecture.tiller_leaf_count;
      record.primary_tiller_count = architecture.primary_tiller_count;
      record.axes = architecture.axes;
      record.leaf_width_scale = sorghum->leaf_mesh_settings.leaf_width_scale;
      float blade_thickness_sum = 0.0f;
      int blade_thickness_count = 0;
      for (const auto handle : sorghum->growth_model.graph.PeekSortedNodeList()) {
        const auto& node = sorghum->growth_model.graph.PeekNode(handle);
        if (node.data.Is<SorghumLeaf>() && node.data.Get<SorghumLeaf>().order == 0) {
          blade_thickness_sum += node.data.Get<SorghumLeaf>().target_blade_thickness;
          ++blade_thickness_count;
        }
      }
      record.leaf_thickness_m =
          blade_thickness_count > 0 ? blade_thickness_sum / static_cast<float>(blade_thickness_count) : 0.0f;
    }
    if (measure_geometry) {
      const auto stats = MeasureGreenPlantGeometry(scene, plant);
      const auto root_position = scene->GetDataComponent<GlobalTransform>(plant).GetPosition();
      record.has_geometry = stats.has_geometry;
      record.plant_height_m = stats.has_geometry ? std::max(0.0f, stats.max_y - root_position.y) : 0.0f;
      record.leaf_area_m2 = stats.leaf_area;
      record.geometry_min_position = stats.has_geometry ? stats.min_position : root_position;
      record.geometry_max_position = stats.has_geometry ? stats.max_position : root_position;
      if (const auto panel = middle_panel_elevations.find(record.cultivar); panel != middle_panel_elevations.end()) {
        record.middle_parbar_top_elevation_m = panel->second;
      }
    }
    records.emplace_back(record);
  }
  return records;
}

size_t PyDigitalAgriculture::MoveParbarMiddlePanelsToPlantHeightFraction(const float height_fraction) {
  const auto scene = ApplicationContext::Get().GetActiveScene();
  return MoveMiddleParbarPanelsToPlantHeightFraction(scene, std::max(0.0f, height_fraction));
}

Handle PyDigitalAgriculture::CreateParbarTopFaceSensorGroup(const uint32_t samples_per_panel) {
  auto sensor_group_handle = PyEvoEngine::CreateRuntimeAsset("PARSensorGroup");
  const auto sensor_group_asset = PyEvoEngine::GetAsset(sensor_group_handle);
  const auto sensors = std::dynamic_pointer_cast<PARSensorGroup>(sensor_group_asset);
  if (!sensors) {
    EVOENGINE_ERROR("CreateParbarTopFaceSensorGroup failed: PARSensorGroup asset unavailable")
    return {};
  }

  sensors->samplers.clear();
  const auto scene = ApplicationContext::Get().GetActiveScene();
  for (const auto& panel : kParbarPanels) {
    AppendParbarPanelSamplers(scene, panel, samples_per_panel, sensors->samplers);
  }
  EVOENGINE_LOG("PARBAR top-face sensor counts: " << sensors->samplers.size())
  return sensor_group_handle;
}

void PyDigitalAgriculture::EstimatePARSensors(const Handle& sensor_group_handle, const int samples, const int bounces,
                                              const float push_normal_distance, const int seed) {
  const auto sensor_group_asset = PyEvoEngine::GetAsset(sensor_group_handle);
  const auto sensors = std::dynamic_pointer_cast<PARSensorGroup>(sensor_group_asset);
  if (!sensors) {
    EVOENGINE_ERROR("EstimatePARSensors failed: invalid PARSensorGroup")
    return;
  }
  RayProperties ray_properties{};
  ray_properties.samples = samples;
  ray_properties.bounces = bounces;
  sensors->CalculateIllumination(ray_properties, seed, push_normal_distance);
}

std::vector<ParbarProbeRecord> PyDigitalAgriculture::GetParbarTopFaceSensorResults(const Handle& sensor_group_handle,
                                                                                   const uint32_t samples_per_panel) {
  const auto sensor_group_asset = PyEvoEngine::GetAsset(sensor_group_handle);
  const auto sensors = std::dynamic_pointer_cast<PARSensorGroup>(sensor_group_asset);
  std::vector<ParbarProbeRecord> records;
  if (!sensors || sensors->samplers.empty()) {
    return records;
  }

  float min_scalar = std::numeric_limits<float>::max();
  float max_scalar = std::numeric_limits<float>::lowest();
  for (const auto& sampler : sensors->samplers) {
    const float scalar = IlluminationScalar(sampler.energy);
    min_scalar = std::min(min_scalar, scalar);
    max_scalar = std::max(max_scalar, scalar);
  }
  const float range = std::max(1e-12f, max_scalar - min_scalar);
  const uint32_t panel_size = std::max(1u, samples_per_panel);
  const auto scene = ApplicationContext::Get().GetActiveScene();
  std::vector<ParbarPanelHeightMetadata> panel_metadata;
  panel_metadata.reserve(kParbarPanelCount);
  for (const auto& panel : kParbarPanels) {
    panel_metadata.emplace_back(GetParbarPanelHeightMetadata(scene, panel, 2.0f / 3.0f));
  }
  records.reserve(sensors->samplers.size());
  for (size_t index = 0; index < sensors->samplers.size(); ++index) {
    const size_t panel_index = std::min(index / panel_size, kParbarPanelCount - 1);
    const auto& panel = kParbarPanels[panel_index];
    const auto& metadata = panel_metadata[panel_index];
    const auto& sampler = sensors->samplers[index];
    const float scalar = IlluminationScalar(sampler.energy);
    ParbarProbeRecord record;
    record.cultivar = panel.cultivar;
    record.model = panel.mesh_entity_name;
    record.sensor_bar_level = panel.sensor_bar_level;
    record.height_rule =
        IsMiddleParbarPanel(panel) ? "two_thirds_average_represented_plant_height" : "fixed_scene_position";
    record.row = static_cast<uint32_t>(panel_index);
    record.column = static_cast<uint32_t>(index % panel_size);
    record.represented_plant_count = metadata.represented_plant_count;
    record.position = sampler.v_0.position;
    record.normal = sampler.v_0.normal;
    record.energy = sampler.energy;
    record.direction = sampler.direction;
    record.average_represented_root_elevation_m = metadata.average_root_y;
    record.average_represented_plant_height_m = metadata.average_plant_height;
    record.sensor_top_elevation_m = metadata.top_elevation;
    record.height_fraction_of_average_height = metadata.height_fraction;
    record.scalar = scalar;
    record.normalized = (scalar - min_scalar) / range;
    records.emplace_back(record);
  }
  return records;
}

std::vector<LSystemGridIlluminationRecord> PyDigitalAgriculture::EstimateSorghumLsGridIllumination(
    const int samples, const int bounces, const int max_triangles_per_plant, const float push_normal_distance,
    const int seed, const std::string& cultivar_filter) {
  std::vector<LSystemGridIlluminationRecord> records;
  const auto scene = ApplicationContext::Get().GetActiveScene();
  if (!scene) {
    EVOENGINE_ERROR("EstimateSorghumLsGridIllumination failed: no active scene")
    return records;
  }
  const auto ray_tracer_layer = ApplicationContext::Get().GetLayer<RayTracerLayer>();
  if (!ray_tracer_layer) {
    EVOENGINE_ERROR("EstimateSorghumLsGridIllumination failed: missing RayTracerLayer")
    return records;
  }

  const auto* sorghum_entities_ptr = scene->UnsafeGetPrivateComponentOwnersList<SorghumLS>();
  if (!sorghum_entities_ptr || sorghum_entities_ptr->empty()) {
    EVOENGINE_ERROR("EstimateSorghumLsGridIllumination failed: no SorghumLS plants")
    return records;
  }

  bool generated_geometry = false;
  std::vector<Entity> all_sorghum_plants;
  std::vector<LSystemTriangleTarget> targets;
  for (const auto& plant : *sorghum_entities_ptr) {
    if (!scene->IsEntityValid(plant)) {
      continue;
    }
    all_sorghum_plants.emplace_back(plant);
    const std::string plant_name = scene->GetEntityName(plant);
    const std::string cultivar = CultivarFromPlantName(plant_name);
    if (!MatchesCultivarFilter(cultivar, cultivar_filter)) {
      continue;
    }
    Entity leaf_entity = FindSorghumLsLeafMeshEntity(scene, plant);
    if (!scene->IsEntityValid(leaf_entity)) {
      if (const auto sorghum = scene->GetOrSetPrivateComponent<SorghumLS>(plant).lock()) {
        sorghum->GenerateGeometryEntities(true);
        generated_geometry = true;
      }
      leaf_entity = FindSorghumLsLeafMeshEntity(scene, plant);
    }
    if (!scene->IsEntityValid(leaf_entity)) {
      continue;
    }

    LSystemTriangleTarget target;
    target.plant = plant;
    target.leaf_entity = leaf_entity;
    target.internode_entity = FindSorghumLsInternodeEntity(scene, plant);
    target.name = plant_name;
    target.cultivar = cultivar;
    TryParseGridCoordinate(target.name, target.row, target.column);
    targets.emplace_back(target);
  }

  if (targets.empty()) {
    EVOENGINE_ERROR("EstimateSorghumLsGridIllumination failed: no leaf meshes were found")
    return records;
  }

  if (generated_geometry) {
    TransformGraph::CalculateTransformGraphs(scene);
  }
  PrepareSorghumLsMeshesForRayTracing(scene);
  RayProperties ray_properties{};
  ray_properties.samples = std::max(1, samples);
  ray_properties.bounces = std::max(1, bounces);

  std::vector<bool> original_enabled_states;
  original_enabled_states.reserve(all_sorghum_plants.size());
  for (const auto& plant : all_sorghum_plants) {
    original_enabled_states.emplace_back(scene->IsEntityEnabled(plant));
    const std::string cultivar = CultivarFromPlantName(scene->GetEntityName(plant));
    scene->SetEnable(plant, MatchesCultivarFilter(cultivar, cultivar_filter));
  }

  ray_tracer_layer->UpdateScene(scene);

  std::vector<LSystemTriangleTarget> sampled_targets;
  sampled_targets.reserve(targets.size());
  records.reserve(targets.size());
  for (auto& target : targets) {
    std::vector<IlluminationSampler<glm::vec3>> samplers;
    const auto stats = AppendGreenPlantSamplers(scene, target, max_triangles_per_plant, samplers);
    if (samplers.empty() || stats.Area() <= 0.0f) {
      continue;
    }

    CudaModule::EstimateIlluminationRayTracing(ray_tracer_layer->environment_properties, ray_properties, samplers, seed,
                                               push_normal_distance);

    LSystemGridIlluminationRecord record;
    record.name = target.name;
    record.cultivar = target.cultivar;
    record.row = target.row;
    record.column = target.column;
    record.position = scene->GetDataComponent<GlobalTransform>(target.plant).GetPosition();
    record.triangle_count = stats.TriangleCount();
    record.leaf_triangle_count = stats.leaf_triangle_count;
    record.stem_triangle_count = stats.stem_triangle_count;
    record.area = stats.Area();
    record.leaf_area = stats.leaf_area;
    record.stem_area = stats.stem_area;
    record.plant_height_m = stats.has_geometry ? std::max(0.0f, stats.max_y - record.position.y) : 0.0f;
    for (const auto& sampler : samplers) {
      record.total_flux += sampler.energy * SamplerSurfaceArea(sampler);
    }
    record.average_flux = record.area > 0.0f ? record.total_flux / record.area : glm::vec3(0.0f);
    record.scalar = IlluminationScalar(record.total_flux);
    records.emplace_back(record);
    sampled_targets.emplace_back(target);
  }

  if (records.empty()) {
    EVOENGINE_ERROR("EstimateSorghumLsGridIllumination failed: no leaf triangles were sampled")
    for (size_t i = 0; i < all_sorghum_plants.size(); i++) {
      scene->SetEnable(all_sorghum_plants[i], original_enabled_states[i]);
    }
    ray_tracer_layer->UpdateScene(scene);
    return records;
  }

  for (size_t target_index = 0; target_index < sampled_targets.size(); target_index++) {
    const auto& target = sampled_targets[target_index];
    for (const auto& other : all_sorghum_plants) {
      scene->SetEnable(other, other == target.plant);
    }
    ray_tracer_layer->UpdateScene(scene);

    std::vector<IlluminationSampler<glm::vec3>> samplers;
    const auto stats = AppendGreenPlantSamplers(scene, target, max_triangles_per_plant, samplers);
    if (samplers.empty() || stats.Area() <= 0.0f) {
      continue;
    }
    CudaModule::EstimateIlluminationRayTracing(ray_tracer_layer->environment_properties, ray_properties, samplers, seed,
                                               push_normal_distance);

    auto& record = records[target_index];
    record.isolated_total_flux = glm::vec3(0.0f);
    for (const auto& sampler : samplers) {
      record.isolated_total_flux += sampler.energy * SamplerSurfaceArea(sampler);
    }
    record.isolated_average_flux = record.area > 0.0f ? record.isolated_total_flux / record.area : glm::vec3(0.0f);
    record.isolated_scalar = IlluminationScalar(record.isolated_total_flux);
    record.retention_ratio = record.isolated_scalar > 1e-12f ? record.scalar / record.isolated_scalar : 1.0f;
    record.normalized = glm::clamp(record.retention_ratio, 0.0f, 1.0f);
    record.shadow_loss = 1.0f - record.normalized;
  }

  for (size_t i = 0; i < all_sorghum_plants.size(); i++) {
    scene->SetEnable(all_sorghum_plants[i], original_enabled_states[i]);
  }
  ray_tracer_layer->UpdateScene(scene);

  std::sort(records.begin(), records.end(), [](const auto& lhs, const auto& rhs) {
    if (lhs.row != rhs.row) {
      return lhs.row < rhs.row;
    }
    if (lhs.column != rhs.column) {
      return lhs.column < rhs.column;
    }
    return lhs.name < rhs.name;
  });
  return records;
}

bool PyDigitalAgriculture::SaveActiveSceneAsProjectAsset(const std::filesystem::path& scene_asset_path) {
  if (scene_asset_path.empty() || scene_asset_path.is_absolute()) {
    EVOENGINE_ERROR("SaveActiveSceneAsProjectAsset failed: path must be relative to project assets")
    return false;
  }
  const auto scene = ApplicationContext::Get().GetActiveScene();
  if (!scene) {
    EVOENGINE_ERROR("SaveActiveSceneAsProjectAsset failed: no active scene")
    return false;
  }
  const auto scene_clone = AssetManager::CreateTemporaryAsset<Scene>();
  if (!scene_clone) {
    EVOENGINE_ERROR("SaveActiveSceneAsProjectAsset failed: could not create scene clone")
    return false;
  }
  Scene::Clone(scene, scene_clone);
  return scene_clone->SetPathAndSave(WithExtension(scene_asset_path, ".evescene"));
}

std::vector<LSystemDescriptorPhenotypeRecord> PyDigitalAgriculture::SampleSorghumLsDescriptorPhenotypes(
    const std::filesystem::path& base_descriptor_path, const float leaf_modules_mean,
    const float leaf_modules_deviation, const float length_mean_scale, const float length_deviation_scale,
    const int sample_count, const int seed_base, const float leaf_width_scale, const float main_culm_diameter_m,
    const float tiller_leaf_count_ratio, const float tiller_height_ratio) {
  std::vector<LSystemDescriptorPhenotypeRecord> records;
  const auto scene = ApplicationContext::Get().GetActiveScene();
  if (!scene) {
    EVOENGINE_ERROR("SampleSorghumLsDescriptorPhenotypes failed: no active scene")
    return records;
  }
  const auto descriptor = CreateCalibratedDescriptorTemporary(
      base_descriptor_path, leaf_modules_mean, leaf_modules_deviation, length_mean_scale, length_deviation_scale,
      leaf_width_scale, main_culm_diameter_m, tiller_leaf_count_ratio, tiller_height_ratio, false);
  if (!descriptor) {
    EVOENGINE_ERROR("SampleSorghumLsDescriptorPhenotypes failed: could not load descriptor " +
                    WithExtension(base_descriptor_path, ".sorghumls").string())
    return records;
  }

  const int clamped_sample_count = std::max(0, sample_count);
  records.reserve(static_cast<size_t>(clamped_sample_count));
  const Entity sample_entity = scene->CreateEntity("__DescriptorCalibrationSample");
  const auto sorghum = scene->GetOrSetPrivateComponent<SorghumLS>(sample_entity).lock();
  if (!sorghum) {
    scene->DeleteEntity(sample_entity);
    EVOENGINE_ERROR("SampleSorghumLsDescriptorPhenotypes failed: could not create sample plant")
    return records;
  }
  for (int i = 0; i < clamped_sample_count; ++i) {
    const uint32_t seed = static_cast<uint32_t>(std::max(0, seed_base)) + static_cast<uint32_t>(i);
    records.emplace_back(MeasureDescriptorSample(scene, sample_entity, sorghum, descriptor, seed));
  }
  scene->DeleteEntity(sample_entity);
  TransformGraph::CalculateTransformGraphs(scene);
  return records;
}

bool PyDigitalAgriculture::SaveCalibratedSorghumLsDescriptor(
    const std::filesystem::path& base_descriptor_path, const std::filesystem::path& output_descriptor_path,
    const float leaf_modules_mean, const float leaf_modules_deviation, const float length_mean_scale,
    const float length_deviation_scale, const float leaf_width_scale, const float main_culm_diameter_m,
    const float tiller_leaf_count_ratio, const float tiller_height_ratio) {
  if (!IsProjectAssetPath(output_descriptor_path)) {
    EVOENGINE_ERROR("SaveCalibratedSorghumLsDescriptor failed: output path must be relative to project assets")
    return false;
  }
  const auto descriptor = CreateCalibratedDescriptorTemporary(
      base_descriptor_path, leaf_modules_mean, leaf_modules_deviation, length_mean_scale, length_deviation_scale,
      leaf_width_scale, main_culm_diameter_m, tiller_leaf_count_ratio, tiller_height_ratio, false);
  if (!descriptor) {
    EVOENGINE_ERROR("SaveCalibratedSorghumLsDescriptor failed: could not load descriptor " +
                    WithExtension(base_descriptor_path, ".sorghumls").string())
    return false;
  }
  if (!SaveDescriptorAsset(descriptor, output_descriptor_path)) {
    EVOENGINE_ERROR("SaveCalibratedSorghumLsDescriptor failed: could not save " +
                    WithExtension(output_descriptor_path, ".sorghumls").string())
    return false;
  }
  return true;
}

Entity PyDigitalAgriculture::CreateEntityFromPrefab(const Handle& prefab_handle, const glm::vec3& position,
                                                    const glm::vec3& euler_rotation, const glm::vec3& scale) {
  const auto asset = PyEvoEngine::GetAsset(prefab_handle);
  if (asset->GetTypeName() != "Prefab") {
    EVOENGINE_ERROR("CreateEntityFromPrefab failed: invalid asset type!")
    return {};
  }
  auto scene = ApplicationContext::Get().GetActiveScene();
  auto entity = std::dynamic_pointer_cast<Prefab>(asset)->ToEntity(scene);
  auto transform = scene->GetDataComponent<Transform>(entity);
  transform.SetValue(position, euler_rotation, scale);
  scene->SetDataComponent(entity, transform);

  return entity;
}

#endif
