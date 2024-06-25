//
// Created by lllll on 2/23/2022.
//
#include <Jobs.hpp>
#ifdef BUILD_WITH_RAYTRACER
#  include "Graphics.hpp"
#  include "PARSensorGroup.hpp"
#  include "RayTracerLayer.hpp"

using namespace eco_sys_lab;
void eco_sys_lab::PARSensorGroup::CalculateIllumination(const RayProperties& ray_properties, int seed,
                                                        float push_normal_distance) {
  if (m_samplers.empty())
    return;
  CudaModule::EstimateIlluminationRayTracing(Application::GetLayer<RayTracerLayer>()->environment_properties,
                                             ray_properties, m_samplers, seed, push_normal_distance);
}
bool PARSensorGroup::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  ImGui::Text("Sampler size: %llu", m_samplers.size());
  if (ImGui::TreeNode("Grid settings")) {
    static auto min_range = glm::vec3(-25, 0, -25);
    static auto max_range = glm::vec3(25, 3, 25);
    static float step = 3.0f;
    if (ImGui::DragFloat3("Min", &min_range.x, 0.1f)) {
      min_range = (glm::min)(min_range, max_range);
    }
    if (ImGui::DragFloat3("Max", &max_range.x, 0.1f)) {
      max_range = (glm::max)(min_range, max_range);
    }
    if (ImGui::DragFloat("Step", &step, 0.01f)) {
      step = glm::clamp(step, 0.1f, 10.0f);
    }
    if (ImGui::Button("Instantiate")) {
      const int sx = static_cast<int>((max_range.x - min_range.x + step) / step);
      const int sy = static_cast<int>((max_range.y - min_range.y + step) / step);
      const int sz = static_cast<int>((max_range.z - min_range.z + step) / step);
      const auto voxel_size = sx * sy * sz;
      m_samplers.resize(voxel_size);
      Jobs::RunParallelFor(voxel_size, [&](unsigned i) {
        float z = (i % sz) * step + min_range.z;
        float y = ((i / sz) % sy) * step + min_range.y;
        float x = ((i / sz / sy) % sx) * step + min_range.x;
        glm::vec3 start = {x, y, z};
        m_samplers[i].m_a.position = m_samplers[i].m_b.position = m_samplers[i].m_c.position = start;
        m_samplers[i].m_frontFace = true;
        m_samplers[i].m_backFace = false;
        m_samplers[i].m_a.normal = m_samplers[i].m_b.normal = m_samplers[i].m_c.normal = glm::vec3(0, 1, 0);
      });
    }
    ImGui::TreePop();
  }
  if (ImGui::TreeNode("Estimation")) {
    static RayProperties ray_properties = {8, 1000};
    ray_properties.OnInspect();
    if (ImGui::Button("Run!"))
      CalculateIllumination(ray_properties, 0, 0.0f);
    ImGui::TreePop();
  }
  static bool draw = true;
  ImGui::Checkbox("Render field", &draw);
  if (draw && !m_samplers.empty()) {
    static float line_width = 0.05f;
    static float line_length_factor = 3.0f;
    static float point_size = 0.1f;
    static std::vector<glm::vec3> starts;
    static std::vector<glm::vec3> ends;
    static std::shared_ptr<ParticleInfoList> ray_particle_info_list;
    static std::shared_ptr<ParticleInfoList> point_particle_info_list;
    if (!ray_particle_info_list) {
      ray_particle_info_list = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
      point_particle_info_list = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
    }

    static glm::vec4 color = {0.0f, 1.0f, 0.0f, 0.5f};
    static glm::vec4 point_color = {1.0f, 0.0f, 0.0f, 0.75f};
    starts.resize(m_samplers.size());
    ends.resize(m_samplers.size());
    std::vector<ParticleInfo> point_particle_infos;
    point_particle_infos.resize(m_samplers.size());
    ImGui::DragFloat("Vector width", &line_width, 0.01f);
    ImGui::DragFloat("Vector length factor", &line_length_factor, 0.01f);
    ImGui::ColorEdit4("Vector Color", &color.x);
    ImGui::DragFloat("Point Size", &point_size, 0.01f);
    ImGui::ColorEdit4("Point Color", &point_color.x);
    Jobs::RunParallelFor(m_samplers.size(), [&](unsigned i) {
      const auto start = m_samplers[i].m_a.position;
      starts[i] = start;
      ends[i] = start + m_samplers[i].m_direction * line_length_factor * m_samplers[i].m_energy;
      point_particle_infos[i].instance_matrix.value = glm::translate(start) * glm::scale(glm::vec3(point_size));
      point_particle_infos[i].instance_color = point_color;
    });
    ray_particle_info_list->ApplyConnections(starts, ends, color, line_width);
    point_particle_info_list->SetParticleInfos(point_particle_infos);
    editor_layer->DrawGizmoMeshInstancedColored(Resources::GetResource<Mesh>("PRIMITIVE_CYLINDER"),
                                                ray_particle_info_list);
    editor_layer->DrawGizmoMeshInstancedColored(Resources::GetResource<Mesh>("PRIMITIVE_CUBE"),
                                                point_particle_info_list);
  }
  return changed;
}
void PARSensorGroup::Serialize(YAML::Emitter& out) const {
  if (!m_samplers.empty()) {
    out << YAML::Key << "m_samplers" << YAML::Value
        << YAML::Binary((const unsigned char*)m_samplers.data(),
                        m_samplers.size() * sizeof(IlluminationSampler<glm::vec3>));
  }
}
void PARSensorGroup::Deserialize(const YAML::Node& in) {
  if (in["m_samplers"]) {
    const auto binary_list = in["m_samplers"].as<YAML::Binary>();
    m_samplers.resize(binary_list.size() / sizeof(IlluminationSampler<glm::vec3>));
    std::memcpy(m_samplers.data(), binary_list.data(), binary_list.size());
  }
}
#endif