
#include "CudaSerializationAdapters.hpp"

#include "BtfMeshRenderer.hpp"
#include "EditorLayer.hpp"
#include "Mesh.hpp"
#include "MeshRenderer.hpp"
#include "Platform.hpp"
#include "ProjectManager.hpp"
#include "RayTracerLayer.hpp"
#include "Scene.hpp"
using namespace evo_engine;

void ColorDescendentsVertices(const std::shared_ptr<Scene>& scene, const Entity& owner,
                              const LightProbeGroup& light_probe_group) {
  std::vector<glm::vec4> probe_colors;
  for (const auto& probe : light_probe_group.light_probes) {
    probe_colors.emplace_back(glm::vec4(probe.energy, 1.0f));
  }
  auto entities = scene->GetDescendants(owner);
  entities.push_back(owner);
  size_t i = 0;
  for (const auto& entity : entities) {
    if (scene->HasPrivateComponent<MeshRenderer>(entity)) {
      const auto mesh_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(entity).lock();
      auto mesh = mesh_renderer->mesh.Get<Mesh>();
      auto material = mesh_renderer->material.Get<Material>();
      if (!mesh || !material)
        continue;
      std::vector<std::pair<size_t, glm::vec4>> colors;
      colors.resize(mesh->GetVerticesAmount());
      for (auto& color : colors) {
        color.first = 0;
        color.second = glm::vec4(0.0f);
      }
      size_t ti = 0;
      for (const auto& triangle : mesh->UnsafeGetTriangles()) {
        const auto color = probe_colors[i];
        colors[triangle.x].first++;
        colors[triangle.y].first++;
        colors[triangle.z].first++;
        colors[triangle.x].second += color;
        colors[triangle.y].second += color;
        colors[triangle.z].second += color;
        ti++;
        i++;
      }
      ti = 0;
      for (auto& vertices : mesh->UnsafeGetVertices()) {
        vertices.color = colors[ti].second / static_cast<float>(colors[ti].first);
        ti++;
      }
    }
  }
}

bool TriangleIlluminationEstimator::DrawGui(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  const auto scene = GetScene();
  const auto owner = GetOwner();
  light_probe_group_.DrawGui();
  static int seed = 0;
  static float push_normal_distance = 0.001f;
  static RayProperties ray_properties;
  if (ImGui::DragInt("Seed", &seed))
    changed = true;
  if (ImGui::DragFloat("Normal Distance", &push_normal_distance, 0.0001f, -1.0f, 1.0f))
    changed = true;
  if (ImGui::DragInt("Samples", &ray_properties.samples))
    changed = true;
  if (ImGui::DragInt("Bounces", &ray_properties.bounces))
    changed = true;
  if (ImGui::Button("Estimate")) {
    PrepareLightProbeGroup();
    SampleLightProbeGroup(ray_properties, seed, push_normal_distance);
    ColorDescendentsVertices(scene, owner, light_probe_group_);
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
      ColorDescendentsVertices(scene, owner, light_probe_group_);
    }
    ImGui::TreePop();
  }

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

  ImGui::Text("%s", ("Surface area: " + std::to_string(total_area)).c_str());
  ImGui::Text("%s", ("Total energy: " + std::to_string(glm::length(total_flux))).c_str());
  ImGui::Text("%s", ("Radiant flux: " + std::to_string(glm::length(average_flux))).c_str());

  return changed;
}

const LightProbeGroup& TriangleIlluminationEstimator::PeekProbes() const {
  return light_probe_group_;
}

void TriangleIlluminationEstimator::SampleLightProbeGroup(const RayProperties& ray_properties, int seed,
                                                          float push_normal_distance) {
  light_probe_group_.CalculateIllumination(ray_properties, seed, push_normal_distance);
  total_flux = glm::vec3(0.0f);
  for (const auto& probe : light_probe_group_.light_probes) {
    total_flux += probe.energy * probe.GetArea();
  }
  average_flux = total_flux / total_area;
}

void TriangleIlluminationEstimator::PrepareLightProbeGroup() {
  total_area = 0.0f;
  light_probe_group_.light_probes.clear();
  const auto scene = GetScene();
  auto entities = scene->GetDescendants(GetOwner());
  entities.push_back(GetOwner());
  for (const auto& entity : entities) {
    if (scene->HasPrivateComponent<MeshRenderer>(entity)) {
      auto global_transform = scene->GetDataComponent<GlobalTransform>(entity);
      const auto mesh_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(entity).lock();
      const auto mesh = mesh_renderer->mesh.Get<Mesh>();
      const auto material = mesh_renderer->material.Get<Material>();
      if (!mesh || !material)
        continue;
      for (const auto& triangle : mesh->UnsafeGetTriangles()) {
        auto& vertices = mesh->UnsafeGetVertices();
        IlluminationSampler<glm::vec3> light_probe;
        light_probe.v_0 = vertices[triangle.x];
        light_probe.v_1 = vertices[triangle.y];
        light_probe.v_2 = vertices[triangle.z];
        light_probe.v_0.position = global_transform.value * glm::vec4(light_probe.v_0.position, 1.0f);
        light_probe.v_1.position = global_transform.value * glm::vec4(light_probe.v_1.position, 1.0f);
        light_probe.v_2.position = global_transform.value * glm::vec4(light_probe.v_2.position, 1.0f);
        light_probe.v_0.normal = global_transform.value * glm::vec4(light_probe.v_0.normal, 0.0f);
        light_probe.v_1.normal = global_transform.value * glm::vec4(light_probe.v_1.normal, 0.0f);
        light_probe.v_2.normal = global_transform.value * glm::vec4(light_probe.v_2.normal, 0.0f);
        auto area = light_probe.GetArea();
        light_probe.direction = glm::vec3(0.0f);
        light_probe.energy = glm::vec3(0.0f);
        switch (material->draw_settings.cull_mode) {
          case VK_CULL_MODE_NONE: {
            light_probe.front_face = light_probe.back_face = true;
            total_area += 2.0f * area;
          } break;
          case VK_CULL_MODE_FRONT_BIT: {
            light_probe.back_face = true;
            light_probe.front_face = false;
            total_area += area;
          } break;
          case VK_CULL_MODE_BACK_BIT: {
            light_probe.front_face = true;
            light_probe.back_face = false;
            total_area += area;
          } break;
          case VK_CULL_MODE_FRONT_AND_BACK: {
            light_probe.front_face = light_probe.back_face = false;
          } break;
        }
        light_probe_group_.light_probes.push_back(light_probe);
      }
    } else if (scene->HasPrivateComponent<BtfMeshRenderer>(entity)) {
      auto global_transform = scene->GetDataComponent<GlobalTransform>(entity);
      const auto mesh_renderer = scene->GetOrSetPrivateComponent<BtfMeshRenderer>(entity).lock();
      const auto mesh = mesh_renderer->mesh.Get<Mesh>();
      const auto material = mesh_renderer->btf.Get<BtfMaterial>();
      if (!mesh || !material)
        continue;
      for (const auto& triangle : mesh->UnsafeGetTriangles()) {
        auto& vertices = mesh->UnsafeGetVertices();
        IlluminationSampler<glm::vec3> light_probe;
        light_probe.v_0 = vertices[triangle.x];
        light_probe.v_1 = vertices[triangle.y];
        light_probe.v_2 = vertices[triangle.z];
        light_probe.v_0.position = global_transform.value * glm::vec4(light_probe.v_0.position, 1.0f);
        light_probe.v_1.position = global_transform.value * glm::vec4(light_probe.v_1.position, 1.0f);
        light_probe.v_2.position = global_transform.value * glm::vec4(light_probe.v_2.position, 1.0f);
        light_probe.v_0.normal = global_transform.value * glm::vec4(light_probe.v_0.normal, 0.0f);
        light_probe.v_1.normal = global_transform.value * glm::vec4(light_probe.v_1.normal, 0.0f);
        light_probe.v_2.normal = global_transform.value * glm::vec4(light_probe.v_2.normal, 0.0f);
        const auto area = light_probe.GetArea();
        light_probe.direction = glm::vec3(0.0f);
        light_probe.energy = glm::vec3(0.0f);
        light_probe.front_face = light_probe.back_face = true;
        total_area += 2.0f * area;

        light_probe_group_.light_probes.push_back(light_probe);
      }
    }
  }
}

void evo_engine::SerializeTriangleIlluminationEstimator(YAML::Emitter& out,
                                                        const TriangleIlluminationEstimator& target) {
  out << YAML::Key << "total_area" << YAML::Value << target.total_area;
  out << YAML::Key << "total_flux" << YAML::Value << target.total_flux;
  out << YAML::Key << "average_flux" << YAML::Value << target.average_flux;
}

void evo_engine::DeserializeTriangleIlluminationEstimator(const YAML::Node& in, TriangleIlluminationEstimator& target) {
  target.total_area = in["total_area"].as<float>();
  target.total_flux = in["total_flux"].as<glm::vec3>();
  target.average_flux = in["average_flux"].as<glm::vec3>();
}
