//
// Created by lllll on 2/23/2022.
//
#include <Jobs.hpp>
#include "DigitalAgricultureInspectionAdapters.hpp"
#include "DigitalAgricultureSerializationAdapters.hpp"
#include "Platform.hpp"

using namespace digital_agriculture_package;
bool digital_agriculture_package::InspectPARSensorGroup(InspectorContext& context, PARSensorGroup& group) {
  const auto& editor_layer = context.editor_layer;
  auto& samplers = group.samplers;
  bool changed = false;
  ImGui::Text("Sampler size: %llu", samplers.size());
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
      samplers.resize(voxel_size);
      Jobs::RunParallelFor(voxel_size, [&](size_t i) {
        float z = (i % sz) * step + min_range.z;
        float y = ((i / sz) % sy) * step + min_range.y;
        float x = ((i / sz / sy) % sx) * step + min_range.x;
        glm::vec3 start = {x, y, z};
        samplers[i].v_0.position = samplers[i].v_1.position = samplers[i].v_2.position = start;
        samplers[i].front_face = true;
        samplers[i].back_face = false;
        samplers[i].v_0.normal = samplers[i].v_1.normal = samplers[i].v_2.normal = glm::vec3(0, 1, 0);
      });
    }
    ImGui::TreePop();
  }
  static bool draw = true;
  ImGui::Checkbox("Render field", &draw);
  if (draw && !samplers.empty()) {
    static float line_width = 0.05f;
    static float line_length_factor = 3.0f;
    static float point_size = 0.1f;
    static std::vector<glm::vec3> starts;
    static std::vector<glm::vec3> ends;
    static std::shared_ptr<ParticleInfoList> ray_particle_info_list;
    static std::shared_ptr<ParticleInfoList> point_particle_info_list;
    if (!ray_particle_info_list) {
      ray_particle_info_list = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
      point_particle_info_list = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
    }

    static glm::vec4 color = {0.0f, 1.0f, 0.0f, 0.5f};
    static glm::vec4 point_color = {1.0f, 0.0f, 0.0f, 0.75f};
    starts.resize(samplers.size());
    ends.resize(samplers.size());
    std::vector<ParticleInfo> point_particle_infos;
    point_particle_infos.resize(samplers.size());
    ImGui::DragFloat("Vector width", &line_width, 0.01f);
    ImGui::DragFloat("Vector length factor", &line_length_factor, 0.01f);
    ImGui::ColorEdit4("Vector Color", &color.x);
    ImGui::DragFloat("Point Size", &point_size, 0.01f);
    ImGui::ColorEdit4("Point Color", &point_color.x);
    Jobs::RunParallelFor(samplers.size(), [&](size_t i) {
      const auto start = samplers[i].v_0.position;
      starts[i] = start;
      ends[i] = start + samplers[i].direction * line_length_factor * samplers[i].energy;
      point_particle_infos[i].instance_matrix.value = glm::translate(start) * glm::scale(glm::vec3(point_size));
      point_particle_infos[i].instance_color = point_color;
    });
    ray_particle_info_list->ApplyConnections(starts, ends, color, line_width);
    point_particle_info_list->SetParticleInfos(point_particle_infos);
    editor_layer->DrawGizmoMeshInstancedColored(Resources::GetInstance().GetPrimitives().cylinder,
                                                ray_particle_info_list);
    editor_layer->DrawGizmoMeshInstancedColored(Resources::GetInstance().GetPrimitives().cube,
                                                point_particle_info_list);
  }
  return changed;
}
void digital_agriculture_package::SerializePARSensorGroup(YAML::Emitter& out, const PARSensorGroup& target) {
  if (!target.samplers.empty()) {
    out << YAML::Key << "samplers" << YAML::Value
        << YAML::Binary((const unsigned char*)target.samplers.data(),
                        target.samplers.size() * sizeof(IlluminationSampler<glm::vec3>));
  }
}
void digital_agriculture_package::DeserializePARSensorGroup(const YAML::Node& in, PARSensorGroup& target) {
  if (in["samplers"]) {
    const auto binary_list = in["samplers"].as<YAML::Binary>();
    target.samplers.resize(binary_list.size() / sizeof(IlluminationSampler<glm::vec3>));
    std::memcpy(target.samplers.data(), binary_list.data(), binary_list.size());
  }
}
