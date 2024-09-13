
#include <TriangleIlluminationEstimator.hpp>
#include "Platform.hpp"
#include "Mesh.hpp"
#include "MeshRenderer.hpp"
#include "RayTracerLayer.hpp"
#include "Scene.hpp"
using namespace evo_engine;

void ColorDescendentsVertices(const std::shared_ptr<Scene>& scene, const Entity& owner,
                              const LightProbeGroup& light_probe_group) {
  std::vector<glm::vec4> probe_colors;
  for (const auto& probe : light_probe_group.light_probes) {
    probe_colors.emplace_back(glm::vec4(probe.m_energy, 1.0f));
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

bool TriangleIlluminationEstimator::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  auto scene = GetScene();
  auto owner = GetOwner();
  light_probe_group_.OnInspect();
  static int seed = 0;
  static float push_normal_distance = 0.001f;
  static RayProperties ray_properties;
  if (ImGui::DragInt("Seed", &seed))
    changed = true;
  if (ImGui::DragFloat("Normal Distance", &push_normal_distance, 0.0001f, -1.0f, 1.0f))
    changed = true;
  if (ImGui::DragInt("Samples", &ray_properties.m_samples))
    changed = true;
  if (ImGui::DragInt("Bounces", &ray_properties.m_bounces))
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

  ImGui::Text("%s", ("Surface area: " + std::to_string(total_area)).c_str());
  ImGui::Text("%s", ("Total energy: " + std::to_string(glm::length(total_flux))).c_str());
  ImGui::Text("%s", ("Radiant flux: " + std::to_string(glm::length(average_flux))).c_str());

  return changed;
}

void TriangleIlluminationEstimator::SampleLightProbeGroup(const RayProperties& ray_properties, int seed,
                                                          float push_normal_distance) {
  light_probe_group_.CalculateIllumination(ray_properties, seed, push_normal_distance);
  total_flux = glm::vec3(0.0f);
  for (const auto& probe : light_probe_group_.light_probes) {
    total_flux += probe.m_energy * probe.GetArea();
  }
  average_flux = total_flux / total_area;
}

void TriangleIlluminationEstimator::PrepareLightProbeGroup() {
  total_area = 0.0f;
  light_probe_group_.light_probes.clear();
  auto scene = GetScene();
  auto entities = scene->GetDescendants(GetOwner());
  entities.push_back(GetOwner());
  for (const auto& entity : entities) {
    if (scene->HasPrivateComponent<MeshRenderer>(entity)) {
      auto global_transform = scene->GetDataComponent<GlobalTransform>(entity);
      const auto mesh_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(entity).lock();
      auto mesh = mesh_renderer->mesh.Get<Mesh>();
      auto material = mesh_renderer->material.Get<Material>();
      if (!mesh || !material)
        continue;
      for (const auto& triangle : mesh->UnsafeGetTriangles()) {
        auto& vertices = mesh->UnsafeGetVertices();
        IlluminationSampler<glm::vec3> light_probe;
        light_probe.m_a = vertices[triangle.x];
        light_probe.m_b = vertices[triangle.y];
        light_probe.m_c = vertices[triangle.z];
        light_probe.m_a.position = global_transform.value * glm::vec4(light_probe.m_a.position, 1.0f);
        light_probe.m_b.position = global_transform.value * glm::vec4(light_probe.m_b.position, 1.0f);
        light_probe.m_c.position = global_transform.value * glm::vec4(light_probe.m_c.position, 1.0f);
        light_probe.m_a.normal = global_transform.value * glm::vec4(light_probe.m_a.normal, 0.0f);
        light_probe.m_b.normal = global_transform.value * glm::vec4(light_probe.m_b.normal, 0.0f);
        light_probe.m_c.normal = global_transform.value * glm::vec4(light_probe.m_c.normal, 0.0f);
        auto area = light_probe.GetArea();
        light_probe.m_direction = glm::vec3(0.0f);
        light_probe.m_energy = glm::vec3(0.0f);
        switch (material->draw_settings.cull_mode) {
          case VK_CULL_MODE_NONE: {
            light_probe.m_frontFace = light_probe.m_backFace = true;
            total_area += 2.0f * area;
          } break;
          case VK_CULL_MODE_FRONT_BIT: {
            light_probe.m_backFace = true;
            light_probe.m_frontFace = false;
            total_area += area;
          } break;
          case VK_CULL_MODE_BACK_BIT: {
            light_probe.m_frontFace = true;
            light_probe.m_backFace = false;
            total_area += area;
          } break;
          case VK_CULL_MODE_FRONT_AND_BACK: {
            light_probe.m_frontFace = light_probe.m_backFace = false;
          } break;
        }
        light_probe_group_.light_probes.push_back(light_probe);
      }
    }
  }
}

void TriangleIlluminationEstimator::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "total_area" << YAML::Value << total_area;
  out << YAML::Key << "total_flux" << YAML::Value << total_flux;
  out << YAML::Key << "average_flux" << YAML::Value << average_flux;
}

void TriangleIlluminationEstimator::Deserialize(const YAML::Node& in) {
  total_area = in["total_area"].as<float>();
  total_flux = in["total_flux"].as<glm::vec3>();
  average_flux = in["average_flux"].as<glm::vec3>();
}
