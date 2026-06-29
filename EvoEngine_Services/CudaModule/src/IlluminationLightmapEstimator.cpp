
#include <IlluminationLightmapEstimator.hpp>

#include "BtfMeshRenderer.hpp"
#include "EditorLayer.hpp"
#include "Mesh.hpp"
#include "MeshRenderer.hpp"
#include "Platform.hpp"
#include "ProjectManager.hpp"
#include "RayTracerLayer.hpp"
#include "Scene.hpp"
#include "Utilities.hpp"
using namespace evo_engine;

namespace {
glm::vec3 ToVertexColor(const glm::vec3& energy, const float exposure, const bool tone_mapping) {
  const auto scaled_energy = glm::max(energy * exposure, glm::vec3(0.0f));
  if (!tone_mapping)
    return glm::clamp(scaled_energy, glm::vec3(0.0f), glm::vec3(1.0f));
  return glm::vec3(1.0f) - glm::exp(-scaled_energy);
}

IlluminationSampler<glm::vec3> BuildVertexLightProbe(const Vertex& source_vertex, const glm::mat4& global_transform) {
  IlluminationSampler<glm::vec3> light_probe;
  light_probe.v_0 = source_vertex;
  light_probe.v_1 = source_vertex;
  light_probe.v_2 = source_vertex;
  light_probe.v_0.position = global_transform * glm::vec4(light_probe.v_0.position, 1.0f);
  light_probe.v_1.position = light_probe.v_0.position;
  light_probe.v_2.position = light_probe.v_0.position;
  light_probe.v_0.normal = global_transform * glm::vec4(light_probe.v_0.normal, 0.0f);
  light_probe.v_1.normal = light_probe.v_0.normal;
  light_probe.v_2.normal = light_probe.v_0.normal;
  light_probe.direction = glm::vec3(0.0f);
  light_probe.energy = glm::vec3(0.0f);
  return light_probe;
}

void ApplyLightmapToDescendentsVertices(const std::shared_ptr<Scene>& scene, const Entity& owner,
                                        const std::vector<glm::vec3>& vertex_lightmap, const float exposure,
                                        const bool tone_mapping) {
  auto entities = scene->GetDescendants(owner);
  entities.push_back(owner);
  size_t lightmap_index = 0;
  for (const auto& entity : entities) {
    std::shared_ptr<Mesh> mesh;
    if (scene->HasPrivateComponent<MeshRenderer>(entity)) {
      const auto mesh_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(entity).lock();
      mesh = mesh_renderer->mesh.Get<Mesh>();
    } else if (scene->HasPrivateComponent<BtfMeshRenderer>(entity)) {
      const auto mesh_renderer = scene->GetOrSetPrivateComponent<BtfMeshRenderer>(entity).lock();
      mesh = mesh_renderer->mesh.Get<Mesh>();
    }
    if (!mesh)
      continue;
    for (auto& vertex : mesh->UnsafeGetVertices()) {
      if (lightmap_index >= vertex_lightmap.size())
        return;
      vertex.color = glm::vec4(ToVertexColor(vertex_lightmap[lightmap_index], exposure, tone_mapping), 1.0f);
      lightmap_index++;
    }
  }
}

void ExportDescendentsObj(const std::shared_ptr<Scene>& scene, const Entity& owner, const std::filesystem::path& path) {
  std::ofstream of;
  of.open(path.string(), std::ofstream::out | std::ofstream::trunc);
  if (!of.is_open()) {
    EVOENGINE_ERROR("Can't open file!");
    return;
  }

  std::string start = "#Illumination lightmap OBJ exporter";
  start += "\n";
  of.write(start.c_str(), start.size());
  of.flush();

  auto entities = scene->GetDescendants(owner);
  entities.push_back(owner);
  unsigned start_index = 1;
  for (const auto& entity : entities) {
    std::shared_ptr<Mesh> mesh;
    if (scene->HasPrivateComponent<MeshRenderer>(entity)) {
      mesh = scene->GetOrSetPrivateComponent<MeshRenderer>(entity).lock()->mesh.Get<Mesh>();
    } else if (scene->HasPrivateComponent<BtfMeshRenderer>(entity)) {
      mesh = scene->GetOrSetPrivateComponent<BtfMeshRenderer>(entity).lock()->mesh.Get<Mesh>();
    }
    if (!mesh || mesh->UnsafeGetTriangles().empty())
      continue;

    const auto global_transform = scene->GetDataComponent<GlobalTransform>(entity).value;
    std::string data;
    data += "#Vertices: " + std::to_string(mesh->GetVerticesAmount()) +
            ", tris: " + std::to_string(mesh->GetTriangleAmount()) + "\n";
    data += "o " + std::to_string(entity.GetIndex()) + "\n";
    for (const auto& vertex : mesh->UnsafeGetVertices()) {
      const auto position = glm::vec3(global_transform * glm::vec4(vertex.position, 1.0f));
      data += "v " + std::to_string(position.x) + " " + std::to_string(position.y) + " " +
              std::to_string(position.z) + " " + std::to_string(vertex.color.x) + " " +
              std::to_string(vertex.color.y) + " " + std::to_string(vertex.color.z) + "\n";
    }
    for (const auto& vertex : mesh->UnsafeGetVertices()) {
      const auto normal = glm::normalize(glm::vec3(global_transform * glm::vec4(vertex.normal, 0.0f)));
      data += "vn " + std::to_string(normal.x) + " " + std::to_string(normal.y) + " " + std::to_string(normal.z) +
              "\n";
    }
    for (const auto& vertex : mesh->UnsafeGetVertices()) {
      data += "vt " + std::to_string(vertex.tex_coord.x) + " " + std::to_string(vertex.tex_coord.y) + "\n";
    }
    data += "# List of indices for faces vertices, with (x, y, z).\n";
    for (const auto& triangle : mesh->UnsafeGetTriangles()) {
      const auto f1 = triangle.x + start_index;
      const auto f2 = triangle.y + start_index;
      const auto f3 = triangle.z + start_index;
      data += "f " + std::to_string(f1) + "/" + std::to_string(f1) + "/" + std::to_string(f1) + " " +
              std::to_string(f2) + "/" + std::to_string(f2) + "/" + std::to_string(f2) + " " +
              std::to_string(f3) + "/" + std::to_string(f3) + "/" + std::to_string(f3) + "\n";
    }
    start_index += mesh->GetVerticesAmount();
    of.write(data.c_str(), data.size());
    of.flush();
  }
  of.close();
  EVOENGINE_LOG("Lightmapped OBJ saved as " + path.string());
}
}  // namespace

bool IlluminationLightmapEstimator::DrawGui(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  const auto scene = GetScene();
  const auto owner = GetOwner();
  light_probe_group_.DrawGui();
  static int seed = 0;
  static float push_normal_distance = 0.001f;
  static float color_exposure = 1.0f;
  static bool tone_mapping = true;
  static RayProperties ray_properties;
  if (ImGui::DragInt("Seed", &seed))
    changed = true;
  if (ImGui::DragFloat("Normal Distance", &push_normal_distance, 0.0001f, -1.0f, 1.0f))
    changed = true;
  if (ImGui::DragInt("Samples", &ray_properties.samples))
    changed = true;
  if (ImGui::DragInt("Bounces", &ray_properties.bounces))
    changed = true;
  if (ImGui::DragFloat("Color exposure", &color_exposure, 0.01f, 0.0f, 100.0f))
    changed = true;
  if (ImGui::Checkbox("Tone mapping", &tone_mapping))
    changed = true;
  if (ImGui::Button("Estimate")) {
    PrepareLightProbeGroup();
    SampleLightProbeGroup(ray_properties, seed, push_normal_distance);
    ApplyLightmapToVertices(color_exposure, tone_mapping);
    changed = true;
  }
  if (ImGui::TreeNode("Details")) {
    if (ImGui::Button("Prepare light probe group")) {
      PrepareLightProbeGroup();
    }
    if (ImGui::Button("Sample light probe group")) {
      SampleLightProbeGroup(ray_properties, seed, push_normal_distance);
    }
    if (ImGui::Button("Color vertices")) {
      ApplyLightmapToVertices(color_exposure, tone_mapping);
    }
    ImGui::TreePop();
  }
  const auto export_color_exposure = color_exposure;
  const auto export_tone_mapping = tone_mapping;
  FileUtils::SaveFile(
      "Export lightmapped OBJ", "OBJ", {".obj"},
      [this, export_color_exposure, export_tone_mapping](const std::filesystem::path& path) {
        ExportLightmappedObj(path, export_color_exposure, export_tone_mapping);
      },
      false);
  FileUtils::SaveFile(
      "Export raw lightmap CSV", "CSV", {".csv"},
      [this](const std::filesystem::path& path) {
        ExportRawLightmapCsv(path);
      },
      false);

  static bool show_probes = true;

  ImGui::Checkbox("Show probes", &show_probes);
  if (show_probes) {
    static float node_render_size = .005f;
    static float energy_scale_factor = 1.f;
    if (ImGui::TreeNode("Debug settings")) {
      ImGui::DragFloat("Probe size", &node_render_size, 0.001f, 0.0f, 1.f);
      ImGui::DragFloat("Energy scale factor", &energy_scale_factor, 0.01f, 0.0f, 100.f);
      ImGui::TreePop();
    }
    static Entity previous_referenced_entity;
    static std::shared_ptr<ParticleInfoList> probe_debug_info_list;
    if (!probe_debug_info_list)
      probe_debug_info_list = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
    if (ImGui::Button("Refresh debug info") || previous_referenced_entity != owner) {
      previous_referenced_entity = owner;
      std::vector<ParticleInfo> particle_infos;
      particle_infos.resize(light_probe_group_.light_probes.size());
      for (int i = 0; i < light_probe_group_.light_probes.size(); i++) {
        const auto& probe = light_probe_group_.light_probes.at(i);
        auto& matrix = particle_infos[i].instance_matrix;
        matrix.value = glm::translate(probe.GetCenter()) * glm::scale(glm::vec3(node_render_size));
        particle_infos[i].instance_color = glm::vec4(glm::vec3(glm::length(probe.energy) * energy_scale_factor), 1.0f);
      }
      probe_debug_info_list->SetParticleInfos(particle_infos);
    }
    GizmoSettings gizmo_settings{};
    gizmo_settings.depth_test = false;
    editor_layer->DrawGizmoCubes(probe_debug_info_list, glm::mat4(1), 1, gizmo_settings);
  }

  ImGui::Text("%s", ("Lightmap vertices: " + std::to_string(vertex_lightmap_.size())).c_str());

  return changed;
}

const LightProbeGroup& IlluminationLightmapEstimator::PeekProbes() const {
  return light_probe_group_;
}

const std::vector<glm::vec3>& IlluminationLightmapEstimator::PeekLightmap() const {
  return vertex_lightmap_;
}

void IlluminationLightmapEstimator::SampleLightProbeGroup(const RayProperties& ray_properties, int seed,
                                                          float push_normal_distance) {
  light_probe_group_.CalculateIlluminationSpectral(ray_properties, seed, push_normal_distance);
  vertex_lightmap_.clear();
  vertex_lightmap_.reserve(light_probe_group_.light_probes.size());
  for (const auto& probe : light_probe_group_.light_probes) {
    vertex_lightmap_.emplace_back(probe.energy);
  }
}

void IlluminationLightmapEstimator::ApplyLightmapToVertices(const float exposure, const bool tone_mapping) const {
  ApplyLightmapToDescendentsVertices(GetScene(), GetOwner(), vertex_lightmap_, exposure, tone_mapping);
}

void IlluminationLightmapEstimator::ExportLightmappedObj(const std::filesystem::path& path, const float exposure,
                                                         const bool tone_mapping) const {
  ApplyLightmapToVertices(exposure, tone_mapping);
  ExportDescendentsObj(GetScene(), GetOwner(), path);
}

void IlluminationLightmapEstimator::ExportRawLightmapCsv(const std::filesystem::path& path) const {
  std::ofstream of(path.string(), std::ofstream::out | std::ofstream::trunc);
  if (!of.is_open()) {
    EVOENGINE_ERROR("Can't open file!");
    return;
  }

  of << "entity,vertex_index,world_x,world_y,world_z,energy_r,energy_g,energy_b,energy_length\n";
  auto entities = GetScene()->GetDescendants(GetOwner());
  entities.push_back(GetOwner());
  size_t lightmap_index = 0;
  for (const auto& entity : entities) {
    std::shared_ptr<Mesh> mesh;
    if (GetScene()->HasPrivateComponent<MeshRenderer>(entity)) {
      mesh = GetScene()->GetOrSetPrivateComponent<MeshRenderer>(entity).lock()->mesh.Get<Mesh>();
    } else if (GetScene()->HasPrivateComponent<BtfMeshRenderer>(entity)) {
      mesh = GetScene()->GetOrSetPrivateComponent<BtfMeshRenderer>(entity).lock()->mesh.Get<Mesh>();
    }
    if (!mesh)
      continue;

    const auto global_transform = GetScene()->GetDataComponent<GlobalTransform>(entity).value;
    const auto& vertices = mesh->UnsafeGetVertices();
    for (size_t vertex_index = 0; vertex_index < vertices.size(); vertex_index++) {
      if (lightmap_index >= vertex_lightmap_.size()) {
        EVOENGINE_WARNING("Raw lightmap export stopped early because the lightmap has fewer entries than vertices.");
        EVOENGINE_LOG("Raw lightmap CSV saved as " + path.string());
        return;
      }
      const auto position = glm::vec3(global_transform * glm::vec4(vertices[vertex_index].position, 1.0f));
      const auto energy = vertex_lightmap_[lightmap_index++];
      of << entity.GetIndex() << "," << vertex_index << "," << position.x << "," << position.y << "," << position.z
         << "," << energy.r << "," << energy.g << "," << energy.b << "," << glm::length(energy) << "\n";
    }
  }
  EVOENGINE_LOG("Raw lightmap CSV saved as " + path.string());
}

void IlluminationLightmapEstimator::PrepareLightProbeGroup() {
  vertex_lightmap_.clear();
  light_probe_group_.light_probes.clear();
  const auto scene = GetScene();
  auto entities = scene->GetDescendants(GetOwner());
  entities.push_back(GetOwner());
  for (const auto& entity : entities) {
    if (scene->HasPrivateComponent<MeshRenderer>(entity)) {
      const auto global_transform = scene->GetDataComponent<GlobalTransform>(entity);
      const auto mesh_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(entity).lock();
      const auto mesh = mesh_renderer->mesh.Get<Mesh>();
      const auto material = mesh_renderer->material.Get<Material>();
      if (!mesh || !material)
        continue;
      for (const auto& vertex : mesh->UnsafeGetVertices()) {
        auto light_probe = BuildVertexLightProbe(vertex, global_transform.value);
        switch (material->draw_settings.cull_mode) {
          case VK_CULL_MODE_NONE: {
            light_probe.front_face = light_probe.back_face = true;
          } break;
          case VK_CULL_MODE_FRONT_BIT: {
            light_probe.back_face = true;
            light_probe.front_face = false;
          } break;
          case VK_CULL_MODE_BACK_BIT: {
            light_probe.front_face = true;
            light_probe.back_face = false;
          } break;
          case VK_CULL_MODE_FRONT_AND_BACK: {
            light_probe.front_face = light_probe.back_face = false;
          } break;
        }
        light_probe_group_.light_probes.push_back(light_probe);
      }
    } else if (scene->HasPrivateComponent<BtfMeshRenderer>(entity)) {
      const auto global_transform = scene->GetDataComponent<GlobalTransform>(entity);
      const auto mesh_renderer = scene->GetOrSetPrivateComponent<BtfMeshRenderer>(entity).lock();
      const auto mesh = mesh_renderer->mesh.Get<Mesh>();
      const auto material = mesh_renderer->btf.Get<BtfMaterial>();
      if (!mesh || !material)
        continue;
      for (const auto& vertex : mesh->UnsafeGetVertices()) {
        auto light_probe = BuildVertexLightProbe(vertex, global_transform.value);
        light_probe.front_face = light_probe.back_face = true;
        light_probe_group_.light_probes.push_back(light_probe);
      }
    }
  }
}

void IlluminationLightmapEstimator::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "vertex_lightmap_size" << YAML::Value << vertex_lightmap_.size();
}

void IlluminationLightmapEstimator::Deserialize(const YAML::Node& in) {
}
