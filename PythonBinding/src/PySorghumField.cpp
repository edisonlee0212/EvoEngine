// Field layout and PARbar placement adapted from laporpe/EvoEngine sorghum-growth-render-apis.
// Measurement uses PARSensorGroup probes and this checkout's CUDA service.
#include <pybind11/stl.h>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <map>
#include <random>
#include <stdexcept>
#include "GeometryStorage.hpp"
#include "Jobs.hpp"
#include "LSystemRuleHelpers.hpp"
#include "Material.hpp"
#include "Mesh.hpp"
#include "PARSensorGroup.hpp"
#include "PyDigitalAgriculture.hpp"
#include "RayTracerLayer.hpp"
#include "SorghumLS.hpp"
#include "SorghumLSDescriptor.hpp"
#include "TextureStorage.hpp"
#include "TransformGraph.hpp"

using namespace py_digital_agriculture_package;
using namespace l_system_package;
namespace py = pybind11;
namespace {
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
  constexpr const char* marker = "_LSystem_";
  const size_t marker_position = name.find(marker);
  return marker_position == std::string::npos || marker_position == 0 ? std::string{} : name.substr(0, marker_position);
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

bool RunLSystemSorghumProject(const std::filesystem::path& project_path,
                              const std::filesystem::path& runtime_package_path,
                              const std::filesystem::path& start_scene_path, bool load_project_assets) {
  if (project_path.extension() != ".eveproj")
    return false;
  auto& app = PyEvoEngine::GetRuntime().GetApplication();
  if (!app.GetLayer<RenderLayer>())
    app.PushLayer<RenderLayer>("Render Layer");
  if (!app.GetLayer<RayTracerLayer>())
    app.PushLayer<RayTracerLayer>("Ray Tracer Layer");
  ApplicationInitializationSettings settings{};
  settings.project_path = project_path;
  settings.startup_scene_path = start_scene_path;
  settings.application_name = "DigitalAgriculture";
  settings.load_project_assets = load_project_assets;
  settings.enable_runtime_packages = true;
  settings.startup_runtime_packages = {"DigitalAgriculture", "LSystem"};
  if (!runtime_package_path.empty())
    settings.package_search_paths.emplace_back(runtime_package_path);
  app.Initialize(settings);
  app.Start(false);
  app.GetLayer<RayTracerLayer>()->environment_properties.environmental_lighting_type =
      EnvironmentalLightingType::Skydome;
  return PyEvoEngine::WaitForCurrentSceneReady(30000);
}

Bound GetFieldBoundsInGroundSpace(const std::shared_ptr<Scene>& scene, const glm::mat4& inverse_ground) {
  Bound bounds;
  for (const auto entity : scene->GetPrivateComponentOwnersList<SorghumLS>()) {
    const auto root =
        glm::vec3(inverse_ground * glm::vec4(scene->GetDataComponent<GlobalTransform>(entity).GetPosition(), 1.0f));
    bounds.min = glm::min(bounds.min, root);
    bounds.max = glm::max(bounds.max, root);
    for (const auto child : CollectDescendants(scene, entity)) {
      if (!scene->HasPrivateComponent<MeshRenderer>(child))
        continue;
      const auto renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(child).lock();
      const auto mesh = renderer->mesh.Get<Mesh>();
      if (!renderer->IsEnabled() || !mesh || mesh->PeekVertices().empty())
        continue;
      std::vector<glm::vec3> corners;
      mesh->GetBound().PopulateCorners(corners);
      const auto transform = inverse_ground * scene->GetDataComponent<GlobalTransform>(child).value;
      for (const auto corner : corners) {
        const auto point = glm::vec3(transform * glm::vec4(corner, 1.0f));
        bounds.min = glm::min(bounds.min, point);
        bounds.max = glm::max(bounds.max, point);
      }
    }
  }
  return bounds;
}

bool EnsureIlluminationSoilContext() {
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
  const auto renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(ground_entity).lock();
  if (!renderer) {
    EVOENGINE_ERROR("EnsureIlluminationSoilContext failed: could not create Ground Mesh renderer")
    return false;
  }
  renderer->cast_shadow = true;
  renderer->mesh = ground_mesh;
  renderer->material = soil_material;
  TransformGraph::CalculateTransformGraphs(scene);
  auto transform = scene->GetDataComponent<GlobalTransform>(ground_entity);
  const auto field_bounds = GetFieldBoundsInGroundSpace(scene, glm::inverse(transform.value));
  if (field_bounds.min.x <= field_bounds.max.x) {
    const auto ground_bounds = ground_mesh->GetBound();
    const glm::vec3 margin = 3.0f / transform.GetScale();
    const auto field_extent = field_bounds.max - field_bounds.min;
    const auto ground_extent = ground_bounds.max - ground_bounds.min;
    if (field_extent.x + 2 * margin.x > ground_extent.x || field_extent.z + 2 * margin.z > ground_extent.z) {
      EVOENGINE_ERROR("Ground mesh is too small for the field plus a 3 m soil margin")
      return false;
    }
    // Recenter the existing terrain in its own XZ plane; keep terrain relief, scale and texture density.
    auto offset = field_bounds.Center() - ground_bounds.Center();
    offset.y = 0;
    if (glm::length(offset) > 1e-5f) {
      transform.SetPosition(transform.TransformPoint(offset));
      scene->SetDataComponent(ground_entity, transform);
      TransformGraph::CalculateTransformGraphs(scene, false);
      EVOENGINE_LOG("Recentered ground mesh on field: local offset " << offset.x << ", " << offset.z)
    }
  }
  return true;
}

bool ValidateIlluminationContext() {
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
  if (!material->GetTexture(&GltfShadeMaterial::pbr_base_color_texture) ||
      !material->GetTexture(&GltfShadeMaterial::normal_texture)) {
    EVOENGINE_ERROR("ValidateIlluminationContext failed: soil base color or normal texture is missing")
    return false;
  }
  const auto field_bounds =
      GetFieldBoundsInGroundSpace(scene, glm::inverse(scene->GetDataComponent<GlobalTransform>(ground_entity).value));
  const auto ground_bounds = mesh->GetBound();
  if (field_bounds.min.x < ground_bounds.min.x - 1e-4f || field_bounds.max.x > ground_bounds.max.x + 1e-4f ||
      field_bounds.min.z < ground_bounds.min.z - 1e-4f || field_bounds.max.z > ground_bounds.max.z + 1e-4f) {
    EVOENGINE_ERROR("ValidateIlluminationContext failed: field roots or canopy extend outside the ground footprint")
    return false;
  }
  return true;
}

py::dict GetIlluminationGroundState() {
  const auto scene = ApplicationContext::Get().GetActiveScene();
  const auto ground = FindEntityByName(scene, "Ground Mesh");
  if (!scene || !scene->IsEntityValid(ground) || !scene->HasPrivateComponent<MeshRenderer>(ground))
    throw std::runtime_error("Missing ground mesh");
  const auto mesh = scene->GetOrSetPrivateComponent<MeshRenderer>(ground).lock()->mesh.Get<Mesh>();
  if (!mesh)
    throw std::runtime_error("Unresolved ground mesh");
  const auto transform = scene->GetDataComponent<GlobalTransform>(ground);
  const auto inverse = glm::inverse(transform.value);
  const auto bounds = mesh->GetBound();
  const auto field = GetFieldBoundsInGroundSpace(scene, inverse);
  size_t outside = 0;
  const auto plants = scene->GetPrivateComponentOwnersList<SorghumLS>();
  for (const auto entity : plants) {
    const auto point =
        glm::vec3(inverse * glm::vec4(scene->GetDataComponent<GlobalTransform>(entity).GetPosition(), 1));
    if (point.x < bounds.min.x - 1e-4f || point.x > bounds.max.x + 1e-4f || point.z < bounds.min.z - 1e-4f ||
        point.z > bounds.max.z + 1e-4f)
      ++outside;
  }
  py::list corners;
  for (const float x : {bounds.min.x, bounds.max.x})
    for (const float z : {bounds.min.z, bounds.max.z}) {
      const auto point = transform.TransformPoint(glm::vec3(x, 0, z));
      corners.append(py::make_tuple(point.x, point.y, point.z));
    }
  py::dict result;
  result["world_corners"] = corners;
  result["plant_count"] = plants.size();
  result["roots_outside"] = outside;
  if (!plants.empty()) {
    const auto scale = transform.GetScale();
    result["minimum_field_margin_m"] =
        std::min({(field.min.x - bounds.min.x) * scale.x, (bounds.max.x - field.max.x) * scale.x,
                  (field.min.z - bounds.min.z) * scale.z, (bounds.max.z - field.max.z) * scale.z});
  }
  return result;
}

size_t ConfigureSorghumLsPlantingGrid(const std::vector<std::string>& row_genotypes, const uint32_t columns,
                                      const float column_spacing_m, const float row_spacing_m, const float center_x_m,
                                      const float center_z_m) {
  const auto scene = ApplicationContext::Get().GetActiveScene();
  if (!scene || row_genotypes.empty() || columns == 0 || column_spacing_m <= 0.0f || row_spacing_m <= 0.0f) {
    return 0;
  }
  if (std::any_of(row_genotypes.begin(), row_genotypes.end(), [](const std::string& genotype) {
        return genotype.empty() || genotype.find("_LSystem_") != std::string::npos;
      })) {
    EVOENGINE_ERROR("ConfigureSorghumLsPlantingGrid: genotype names must be non-empty labels")
    return 0;
  }

  std::vector<Entity> old_markers;
  std::vector<Entity> old_parents;
  for (const auto& entity : scene->UnsafeGetAllEntities()) {
    if (!scene->IsEntityValid(entity) || !IsSorghumPlantingMarkerName(scene->GetEntityName(entity))) {
      continue;
    }
    old_markers.emplace_back(entity);
    const auto parent = scene->GetParent(entity);
    if (scene->IsEntityValid(parent) &&
        std::find(old_parents.begin(), old_parents.end(), parent) == old_parents.end()) {
      old_parents.emplace_back(parent);
    }
  }
  for (const auto& marker : old_markers) {
    if (scene->IsEntityValid(marker)) {
      scene->DeleteEntity(marker);
    }
  }
  for (const auto& parent : old_parents) {
    const std::string parent_name = scene->IsEntityValid(parent) ? scene->GetEntityName(parent) : std::string{};
    if (scene->IsEntityValid(parent) && scene->GetChildren(parent).empty() && parent_name.size() >= 6 &&
        parent_name.compare(parent_name.size() - 6, 6, "_Field") == 0) {
      scene->DeleteEntity(parent);
    }
  }

  Entity field_root = FindEntityByName(scene, "Sorghum2026_6x10_Field");
  if (!scene->IsEntityValid(field_root)) {
    field_root = scene->CreateEntity("Sorghum2026_6x10_Field");
  }
  const float center_row = 0.5f * static_cast<float>(row_genotypes.size() - 1);
  const float center_column = 0.5f * static_cast<float>(columns - 1);
  size_t marker_count = 0;
  for (size_t row = 0; row < row_genotypes.size(); ++row) {
    for (uint32_t column = 0; column < columns; ++column) {
      const std::string name = row_genotypes[row] + "_LSystem_R" + std::to_string(row) + "_C" + std::to_string(column);
      const Entity marker = scene->CreateEntity(name);
      auto transform = scene->GetDataComponent<Transform>(marker);
      transform.SetPosition(glm::vec3(center_x_m + (static_cast<float>(column) - center_column) * column_spacing_m,
                                      0.0f, center_z_m + (static_cast<float>(row) - center_row) * row_spacing_m));
      scene->SetDataComponent(marker, transform);
      scene->SetParent(marker, field_root);
      ++marker_count;
    }
  }
  TransformGraph::CalculateTransformGraphs(scene, false);
  return marker_count;
}

size_t InstantiateSorghumLsPlantsFromPlantingMarkers() {
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

size_t ConformSorghumLsPlantsToGroundMesh() {
  if (!ValidateIlluminationContext())
    return 0;
  const auto scene = ApplicationContext::Get().GetActiveScene();
  if (!scene) {
    EVOENGINE_WARNING("ConformSorghumLsPlantsToGroundMesh: no active scene")
    return 0;
  }
  const Entity ground = FindEntityByName(scene, "Ground Mesh");
  if (!scene->IsEntityValid(ground)) {
    EVOENGINE_WARNING("ConformSorghumLsPlantsToGroundMesh: Ground Mesh entity not found")
    return 0;
  }
  const auto renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(ground).lock();
  const auto mesh = renderer ? renderer->mesh.Get<Mesh>() : nullptr;
  if (!mesh) {
    EVOENGINE_WARNING("ConformSorghumLsPlantsToGroundMesh: ground mesh asset unavailable")
    return 0;
  }
  const auto& vertices = mesh->PeekVertices();
  if (vertices.empty()) {
    EVOENGINE_WARNING("ConformSorghumLsPlantsToGroundMesh: ground mesh has no vertices")
    return 0;
  }
  glm::vec2 minimum(std::numeric_limits<float>::max());
  glm::vec2 maximum(std::numeric_limits<float>::lowest());
  for (const auto& vertex : vertices) {
    const glm::vec2 position(vertex.position.x, vertex.position.z);
    minimum = glm::min(minimum, position);
    maximum = glm::max(maximum, position);
  }
  const glm::vec2 extent = maximum - minimum;
  const float cell_size =
      std::max(std::max(extent.x, extent.y) / std::sqrt(static_cast<float>(vertices.size())) * 1.5f, 0.005f);
  const size_t columns = static_cast<size_t>(std::ceil(extent.x / cell_size)) + 1;
  const size_t rows = static_cast<size_t>(std::ceil(extent.y / cell_size)) + 1;
  if (!columns || !rows || columns > std::numeric_limits<size_t>::max() / rows) {
    EVOENGINE_WARNING("ConformSorghumLsPlantsToGroundMesh: invalid ground sampling extent")
    return 0;
  }
  std::vector<glm::vec4> samples(columns * rows, glm::vec4(0.0f, 0.0f, 0.0f, std::numeric_limits<float>::max()));
  for (const auto& vertex : vertices) {
    const glm::vec2 position(vertex.position.x, vertex.position.z);
    const size_t column = std::min(static_cast<size_t>((position.x - minimum.x) / cell_size), columns - 1);
    const size_t row = std::min(static_cast<size_t>((position.y - minimum.y) / cell_size), rows - 1);
    const glm::vec2 center = minimum + glm::vec2(column + 0.5f, row + 0.5f) * cell_size;
    const glm::vec2 offset = position - center;
    const float distance = glm::dot(offset, offset);
    auto& sample = samples[row * columns + column];
    if (distance < sample.w)
      sample = glm::vec4(vertex.position, distance);
  }

  const auto ground_transform = scene->GetDataComponent<GlobalTransform>(ground);
  const glm::mat4 inverse_ground = glm::inverse(ground_transform.value);
  const auto sample_height = [&](const glm::vec3& world_position) {
    const glm::vec3 local = glm::vec3(inverse_ground * glm::vec4(world_position, 1.0f));
    const int column =
        std::clamp(static_cast<int>((local.x - minimum.x) / cell_size), 0, static_cast<int>(columns - 1));
    const int row = std::clamp(static_cast<int>((local.z - minimum.y) / cell_size), 0, static_cast<int>(rows - 1));
    glm::vec3 nearest(local.x, 0.0f, local.z);
    float nearest_distance = std::numeric_limits<float>::max();
    for (int dz = -2; dz <= 2; ++dz) {
      for (int dx = -2; dx <= 2; ++dx) {
        const int candidate_column = column + dx;
        const int candidate_row = row + dz;
        if (candidate_column < 0 || candidate_row < 0 || candidate_column >= static_cast<int>(columns) ||
            candidate_row >= static_cast<int>(rows))
          continue;
        const glm::vec4& candidate = samples[candidate_row * columns + candidate_column];
        if (candidate.w == std::numeric_limits<float>::max())
          continue;
        const glm::vec2 offset(candidate.x - local.x, candidate.z - local.z);
        const float distance = glm::dot(offset, offset);
        if (distance < nearest_distance) {
          nearest = glm::vec3(candidate);
          nearest_distance = distance;
        }
      }
    }
    return ground_transform.TransformPoint(nearest).y;
  };

  const auto* plants = scene->UnsafeGetPrivateComponentOwnersList<SorghumLS>();
  if (!plants) {
    EVOENGINE_WARNING("ConformSorghumLsPlantsToGroundMesh: no SorghumLS component owners")
    return 0;
  }
  size_t count = 0;
  for (const auto plant : *plants) {
    if (!scene->IsEntityValid(plant) || scene->GetEntityName(plant).find("_cluster_") != std::string::npos)
      continue;
    const float root_y = scene->GetDataComponent<GlobalTransform>(plant).GetPosition().y;
    auto transform = scene->GetDataComponent<Transform>(plant);
    transform.SetPosition(
        transform.GetPosition() +
        glm::vec3(0.0f, sample_height(scene->GetDataComponent<GlobalTransform>(plant).GetPosition()) - root_y, 0.0f));
    scene->SetDataComponent(plant, transform);
    ++count;
  }
  if (count)
    TransformGraph::CalculateTransformGraphs(scene);
  return count;
}

size_t SetSorghumLsGenotypeDescriptors(const std::map<std::string, std::filesystem::path>& descriptor_paths,
                                       const bool regenerate_geometry, const int seed_base) {
  const auto scene = ApplicationContext::Get().GetActiveScene();
  if (!scene || descriptor_paths.empty()) {
    return 0;
  }
  std::map<std::string, std::shared_ptr<SorghumLSDescriptor>> descriptors;
  for (const auto& [genotype, path] : descriptor_paths) {
    if (genotype.empty()) {
      EVOENGINE_ERROR("SetSorghumLsGenotypeDescriptors failed: empty genotype name")
      return 0;
    }
    const auto descriptor = std::dynamic_pointer_cast<SorghumLSDescriptor>(ProjectManager::GetOrCreateAsset(path));
    if (!descriptor) {
      EVOENGINE_ERROR("SetSorghumLsGenotypeDescriptors failed: missing descriptor asset for " + genotype)
      return 0;
    }
    descriptors.emplace(genotype, descriptor);
  }

  size_t plant_count = 0;
  uint32_t reseed_index = 0;
  if (const auto* sorghum_entities_ptr = scene->UnsafeGetPrivateComponentOwnersList<SorghumLS>()) {
    const std::vector<Entity> sorghum_entities = *sorghum_entities_ptr;
    for (const auto& entity : sorghum_entities) {
      if (!scene->IsEntityValid(entity)) {
        continue;
      }
      const std::string genotype = CultivarFromPlantName(scene->GetEntityName(entity));
      const auto descriptor_it = descriptors.find(genotype);
      if (descriptor_it == descriptors.end()) {
        continue;
      }
      const auto sorghum = scene->GetOrSetPrivateComponent<SorghumLS>(entity).lock();
      if (!sorghum) {
        continue;
      }
      sorghum->descriptor_ref = descriptor_it->second;
      if (seed_base >= 0) {
        sorghum->seed = static_cast<uint32_t>(seed_base) + reseed_index;
      }
      if (regenerate_geometry) {
        std::mt19937 rng(sorghum->seed);
        sorghum->target_gdd = std::max(0.0f, SampleDistribution(descriptor_it->second->target_gdd, rng));
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

size_t ConfigureSorghumLsLeafMeshQuality(const float vertical_subdivision_length, const int horizontal_subdivision_step,
                                         const bool bottom_face, const bool enable_leaf_sheath,
                                         const bool regenerate_geometry) {
  const auto scene = ApplicationContext::Get().GetActiveScene();
  if (!scene) {
    return 0;
  }
  const float vertical = std::max(0.001f, vertical_subdivision_length);
  const int horizontal = std::max(2, horizontal_subdivision_step);
  size_t plant_count = 0;
  if (const auto* owners = scene->UnsafeGetPrivateComponentOwnersList<SorghumLS>()) {
    for (const auto& entity : *owners) {
      if (!scene->IsEntityValid(entity)) {
        continue;
      }
      const auto sorghum = scene->GetOrSetPrivateComponent<SorghumLS>(entity).lock();
      if (!sorghum) {
        continue;
      }
      sorghum->leaf_mesh_settings.vertical_subdivision_length = vertical;
      sorghum->leaf_mesh_settings.horizontal_subdivision_step = horizontal;
      sorghum->leaf_mesh_settings.enable_leaf_sheath = enable_leaf_sheath;
      sorghum->leaf_bottom_face = bottom_face;
      if (regenerate_geometry) {
        sorghum->GenerateGeometryEntities(true);
      }
      ++plant_count;
    }
  }
  if (plant_count > 0 && regenerate_geometry) {
    TransformGraph::CalculateTransformGraphs(scene);
  }
  return plant_count;
}

size_t GrowSorghumLsPlantsToAdulthood(const int seed_base, const std::string& cultivar_filter,
                                      const bool reuse_geometry_entities, const bool update_render_geometry) {
  const auto scene = ApplicationContext::Get().GetActiveScene();
  if (!scene) {
    return 0;
  }

  std::vector<std::shared_ptr<SorghumLS>> plants;
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
      plants.emplace_back(sorghum);
      reseed_index++;
    }
  }
  if (reuse_geometry_entities) {
    std::vector<std::shared_ptr<const SorghumGeometrySnapshot>> snapshots(plants.size());
    Jobs::RunParallelFor(plants.size(), [&](const size_t index) {
      snapshots[index] = plants[index]->GenerateGeometrySnapshot(true);
    });
    for (size_t index = 0; index < plants.size(); ++index) {
      plants[index]->PublishGeometrySnapshot(snapshots[index], update_render_geometry);
    }
  } else {
    for (const auto& plant : plants) {
      plant->GenerateGeometryEntities(true);
    }
  }
  if (!plants.empty()) {
    TransformGraph::CalculateTransformGraphs(scene);
  }
  return plants.size();
}

void SetSunDirection(glm::vec3 angles) {
  // adopted from RayTracerLayer::SetSunDirection, note the order of angles is (x, y, z) = (pitch, yaw, roll)
  auto ray_tracer_layer = ApplicationContext::Get().GetLayer<RayTracerLayer>();
  glm::vec3 sun_direction = glm::quat(glm::radians(angles)) * glm::vec3(0, 0, -1);
  ray_tracer_layer->environment_properties.sun_direction = sun_direction;
}

Handle CreateParbarTopFaceSensorGroup(const uint32_t samples_per_panel) {
  if (samples_per_panel == 0)
    throw std::invalid_argument("samples_per_panel must be positive");
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
    if (!AppendParbarPanelSamplers(scene, panel, samples_per_panel, sensors->samplers))
      throw std::runtime_error("Cannot construct all PARbar panels");
  }
  EVOENGINE_LOG("PARBAR top-face sensor counts: " << sensors->samplers.size())
  return sensor_group_handle;
}

void EstimatePARSensors(const Handle& sensor_group_handle, const int samples, const int bounces,
                        const float push_normal_distance, const int seed) {
  const auto sensor_group_asset = PyEvoEngine::GetAsset(sensor_group_handle);
  const auto sensors = std::dynamic_pointer_cast<PARSensorGroup>(sensor_group_asset);
  if (!sensors) {
    EVOENGINE_ERROR("EstimatePARSensors failed: invalid PARSensorGroup")
    return;
  }
  if (samples <= 0 || bounces < 0 || !std::isfinite(push_normal_distance) || push_normal_distance < 0)
    throw std::invalid_argument("Invalid sensor sampling settings");
  const auto scene = ApplicationContext::Get().GetActiveScene();
  const auto layer = ApplicationContext::Get().GetLayer<RayTracerLayer>();
  if (!scene || !layer)
    throw std::runtime_error("Missing measurement scene or RayTracerLayer");
  TransformGraph::CalculateTransformGraphs(scene, false);
  layer->UpdateScene(scene);
  RayProperties ray_properties{};
  ray_properties.samples = samples;
  ray_properties.bounces = bounces;
  // CUDA is linked statically into both the package and binding; use the instance that owns this layer's scene.
  CudaModule::EstimateIlluminationRayTracing(layer->environment_properties, ray_properties, sensors->samplers, seed,
                                             push_normal_distance, layer->GetIlluminationClouds());
}

struct ParbarProbeRecord {
  std::string cultivar, sensor_bar_level;
  uint32_t column;
  glm::vec3 position, normal, energy;
  float scalar;
};

std::vector<ParbarProbeRecord> GetParbarTopFaceSensorResults(const Handle& handle, uint32_t samples_per_panel) {
  const auto sensors = std::dynamic_pointer_cast<PARSensorGroup>(PyEvoEngine::GetAsset(handle));
  if (!sensors || samples_per_panel == 0 || sensors->samplers.size() != kParbarPanelCount * samples_per_panel)
    throw std::invalid_argument("Sensor handle or panel size does not match the PARbar group");
  std::vector<ParbarProbeRecord> result;
  for (size_t i = 0; i < sensors->samplers.size(); ++i) {
    const auto& panel = kParbarPanels[i / samples_per_panel];
    const auto& sample = sensors->samplers[i];
    result.push_back({panel.cultivar, panel.sensor_bar_level, static_cast<uint32_t>(i % samples_per_panel),
                      sample.v_0.position, sample.v_0.normal, sample.energy, IlluminationScalar(sample.energy)});
  }
  return result;
}

bool WaitForProjectIdle(const int max_frames) {
  if (max_frames <= 0)
    throw std::invalid_argument("max_frames must be positive");
  auto& application = ApplicationContext::Get();
  for (int frame = 0; frame < max_frames; ++frame) {
    const auto scene = application.GetActiveScene();
    if (!scene)
      return false;
    size_t pending = 0;
    for (const auto& entity : scene->GetPrivateComponentOwnersList<SorghumLS>()) {
      const auto plant = scene->GetOrSetPrivateComponent<SorghumLS>(entity).lock();
      plant->FlushGeometryUpdates();
      pending += plant->CountPendingGeometryUpdates();
    }
    if (!pending && ProjectManager::IsProjectIdle() && !GeometryStorage::HasPendingUploads() &&
        !TextureStorage::HasPendingUploads())
      return true;
    if (frame % 25 == 0)
      EVOENGINE_LOG("Field readiness: " << pending << " pending geometry publications")
    if (!application.Loop())
      return false;
  }
  EVOENGINE_ERROR("Field geometry did not become ready within the frame limit")
  return false;
}

py::list GetSorghumLsFieldState() {
  py::list result;
  const auto scene = ApplicationContext::Get().GetActiveScene();
  for (const auto& entity : scene->GetPrivateComponentOwnersList<SorghumLS>()) {
    const auto plant = scene->GetOrSetPrivateComponent<SorghumLS>(entity).lock();
    const auto position = scene->GetDataComponent<GlobalTransform>(entity).GetPosition();
    py::dict record;
    record["name"] = scene->GetEntityName(entity);
    record["genotype"] = CultivarFromPlantName(scene->GetEntityName(entity));
    record["seed"] = plant->seed;
    record["position"] = py::make_tuple(position.x, position.y, position.z);
    record["leaf_count"] = plant->last_leaf_count;
    record["has_geometry"] = static_cast<bool>(plant->GetGeometrySnapshot());
    record["pending_geometry_updates"] = plant->CountPendingGeometryUpdates();
    size_t published_triangles = 0;
    for (const auto& child : CollectDescendants(scene, entity)) {
      if (!scene->HasPrivateComponent<MeshRenderer>(child))
        continue;
      const auto renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(child).lock();
      if (const auto mesh = renderer->mesh.Get<Mesh>(); mesh && renderer->IsEnabled())
        published_triangles += mesh->PeekTriangles().size();
    }
    const auto snapshot = plant->GetGeometrySnapshot();
    record["snapshot_triangles"] = snapshot ? snapshot->culm_triangles.size() + snapshot->leaf_triangles.size() +
                                                  snapshot->panicle_triangles.size()
                                            : 0;
    record["published_triangles"] = published_triangles;
    const auto descriptor = plant->descriptor_ref.Get<SorghumLSDescriptor>();
    record["textures_loaded"] =
        descriptor && descriptor->leaf_atlas_albedo_texture.Get<Texture2D>() &&
        descriptor->leaf_atlas_normal_texture.Get<Texture2D>() &&
        descriptor->leaf_atlas_roughness_texture.Get<Texture2D>() && descriptor->stem_albedo_texture.Get<Texture2D>() &&
        descriptor->stem_normal_texture.Get<Texture2D>() && descriptor->stem_roughness_texture.Get<Texture2D>();
    result.append(record);
  }
  return result;
}
}  // namespace

void PyDigitalAgriculture::RegisterSorghumFieldBindings(py::module& m) {
  m.def("RunLSystemSorghumProject", &RunLSystemSorghumProject, py::arg("project_path"),
        py::arg("runtime_package_path") = std::filesystem::path{},
        py::arg("start_scene_path") = std::filesystem::path{}, py::arg("load_project_assets") = false);
  m.def("WaitForProjectIdle", &WaitForProjectIdle, py::arg("max_frames") = 30000);
  m.def("EnsureIlluminationSoilContext", &EnsureIlluminationSoilContext);
  m.def("ValidateIlluminationContext", &ValidateIlluminationContext);
  m.def("GetIlluminationGroundState", &GetIlluminationGroundState);
  m.def("ConfigureSorghumLsPlantingGrid", &ConfigureSorghumLsPlantingGrid, py::arg("row_genotypes"), py::arg("columns"),
        py::arg("column_spacing_m"), py::arg("row_spacing_m"), py::arg("center_x_m") = 0.0f,
        py::arg("center_z_m") = 0.0f);
  m.def("InstantiateSorghumLsPlantsFromPlantingMarkers", &InstantiateSorghumLsPlantsFromPlantingMarkers);
  m.def("ConformSorghumLsPlantsToGroundMesh", &ConformSorghumLsPlantsToGroundMesh);
  m.def("SetSorghumLsGenotypeDescriptors", &SetSorghumLsGenotypeDescriptors, py::arg("descriptor_paths"),
        py::arg("regenerate_geometry") = false, py::arg("seed_base") = -1);
  m.def("ConfigureSorghumLsLeafMeshQuality", &ConfigureSorghumLsLeafMeshQuality, py::arg("vertical_subdivision_length"),
        py::arg("horizontal_subdivision_step"), py::arg("bottom_face") = false, py::arg("enable_leaf_sheath") = true,
        py::arg("regenerate_geometry") = false);
  m.def("GrowSorghumLsPlantsToAdulthood", &GrowSorghumLsPlantsToAdulthood, py::arg("seed_base") = -1,
        py::arg("cultivar_filter") = "", py::arg("reuse_geometry_entities") = false,
        py::arg("update_render_geometry") = true);
  m.def("SetSunDirection", &SetSunDirection);
  m.def("ConfigureRayTracerSkydome", &PyEvoEngine::ConfigureCurrentSceneCudaOutdoorLightingForCapture,
        py::arg("sun_angles_degrees") = glm::vec3(55, 30, 0),
        py::arg("sun_angular_diameter_radians") = EnvironmentProperties::kPhysicalSunAngularDiameterRadians,
        py::arg("sun_intensity") = 1.0f, py::arg("sun_color") = glm::vec3(1.0f), py::arg("skylight_intensity") = 1.0f,
        py::arg("ambient_light_intensity") = 0.1f, py::arg("gamma") = 2.2f);
  m.def("CreateParbarTopFaceSensorGroup", &CreateParbarTopFaceSensorGroup, py::arg("samples_per_panel") = 50);
  m.def("EstimatePARSensors", &EstimatePARSensors, py::arg("sensor_group_handle"), py::arg("samples") = 128,
        py::arg("bounces") = 4, py::arg("push_normal_distance") = 0.001f, py::arg("seed") = 1);
  m.def("GetParbarTopFaceSensorResults", &GetParbarTopFaceSensorResults, py::arg("sensor_group_handle"),
        py::arg("samples_per_panel") = 50);
  m.def("GetSorghumLsFieldState", &GetSorghumLsFieldState);
  m.def("GetSorghumLsInitialLeafAzimuths", [] {
    py::dict result;
    const auto scene = ApplicationContext::Get().GetActiveScene();
    if (!scene)
      return result;
    for (const auto& entity : scene->GetPrivateComponentOwnersList<SorghumLS>()) {
      const auto plant = scene->GetOrSetPrivateComponent<SorghumLS>(entity).lock();
      result[py::str(scene->GetEntityName(entity))] = plant->growth_model.initial_leaf_azimuth_degrees;
    }
    return result;
  });
  py::class_<ParbarProbeRecord>(m, "ParbarProbeRecord")
      .def_readonly("cultivar", &ParbarProbeRecord::cultivar)
      .def_readonly("sensor_bar_level", &ParbarProbeRecord::sensor_bar_level)
      .def_readonly("column", &ParbarProbeRecord::column)
      .def_readonly("position", &ParbarProbeRecord::position)
      .def_readonly("normal", &ParbarProbeRecord::normal)
      .def_readonly("energy", &ParbarProbeRecord::energy)
      .def_readonly("scalar", &ParbarProbeRecord::scalar);
}
