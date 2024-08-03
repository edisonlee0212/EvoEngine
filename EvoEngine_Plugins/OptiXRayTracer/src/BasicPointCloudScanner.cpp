//
// Created by lllll on 12/15/2021.
//

#include "BasicPointCloudScanner.hpp"

#include "Resources.hpp"
#include "EditorLayer.hpp"
#include "Jobs.hpp"
#include "RayTracerLayer.hpp"
using namespace evo_engine;

bool BasicPointCloudScanner::OnInspect(const std::shared_ptr<EditorLayer> &editor_layer) {
  bool changed = false;

  if (ImGui::DragFloat("Angle", &rotate_angle, 0.1f, -90.0f, 90.0f))
    changed = true;
  if (ImGui::DragFloat2("Size", &size.x, 0.1f))
    changed = true;
  if (ImGui::DragFloat2("Distance", &distance.x, 0.001f, 1.0f, 0.001f))
    changed = true;
  auto scene = GetScene();
  static glm::vec4 color = glm::vec4(0, 1, 0, 0.5);
  if (ImGui::ColorEdit4("Color", &color.x))
    changed = true;
  static bool render_plane = true;
  ImGui::Checkbox("Render plane", &render_plane);
  auto gt = scene->GetDataComponent<GlobalTransform>(GetOwner());
  glm::vec3 front = glm::normalize(gt.GetRotation() * glm::vec3(0, 0, -1));
  glm::vec3 up = glm::normalize(gt.GetRotation() * glm::vec3(0, 1, 0));
  glm::vec3 left = glm::normalize(gt.GetRotation() * glm::vec3(1, 0, 0));
  const glm::vec3 actual_vector = glm::rotate(front, glm::radians(rotate_angle), up);
  if (render_plane) {
    editor_layer->DrawGizmoMesh(Resources::GetResource<Mesh>("PRIMITIVE_QUAD"), glm::vec4(1, 0, 0, 0.5),
                                glm::translate(gt.GetPosition() + front * 0.5f) *
                                    glm::mat4_cast(glm::quatLookAt(up, glm::normalize(actual_vector))) *
                                    glm::scale(glm::vec3(0.1, 0.5, 0.1f)),
                                1.0f);
    editor_layer->DrawGizmoMesh(Resources::GetResource<Mesh>("PRIMITIVE_QUAD"), color,
                                glm::translate(gt.GetPosition()) * glm::mat4_cast(glm::quatLookAt(up, front)) *
                                    glm::scale(glm::vec3(size.x / 2.0f, 1.0, size.y / 2.0f)),
                                1.0f);
  }
  if (ImGui::Button("Scan")) {
    Scan();
    changed = true;
  }

  ImGui::Text("Sample amount: %d", points.size());
  if (!points.empty()) {
    if (ImGui::Button("Clear")) {
      points.clear();
      point_colors.clear();
      point_colors.clear();
    }
    static AssetRef point_cloud;
    ImGui::Text("Construct PointCloud");
    ImGui::SameLine();
    if (editor_layer->DragAndDropButton<PointCloud>(point_cloud, "Here", false)) {
      if (const auto ptr = point_cloud.Get<PointCloud>()) {
        ConstructPointCloud(ptr);
      }
      point_cloud.Clear();
    }
  }
  return changed;
}

void BasicPointCloudScanner::Serialize(YAML::Emitter &out) const {
}

void BasicPointCloudScanner::Deserialize(const YAML::Node &in) {
}

void BasicPointCloudScanner::Scan() {
  const auto column = static_cast<unsigned>(size.x / distance.x);
  const int column_start = -static_cast<int>(column / 2);
  const auto row = static_cast<unsigned>(size.y / distance.y);
  const int row_start = -(row / 2);
  const auto size = column * row;
  auto gt = GetScene()->GetDataComponent<GlobalTransform>(GetOwner());
  glm::vec3 center = gt.GetPosition();
  glm::vec3 front = gt.GetRotation() * glm::vec3(0, 0, -1);
  glm::vec3 up = gt.GetRotation() * glm::vec3(0, 1, 0);
  glm::vec3 left = gt.GetRotation() * glm::vec3(1, 0, 0);
  const glm::vec3 actual_vector = glm::rotate(front, glm::radians(rotate_angle), up);
  std::vector<PointCloudSample> pc_samples;
  pc_samples.resize(size);

  std::vector<std::shared_future<void>> results;
  Jobs::RunParallelFor(size, [&](unsigned i) {
    const int column_index = (int)i / row;
    const int row_index = (int)i % row;
    const auto position = center + left * (float)(column_start + column_index) * distance.x +
                          up * (float)(row_start + row_index) * distance.y;
    pc_samples[i].m_start = position;
    pc_samples[i].m_direction = glm::normalize(actual_vector);
  });

  CudaModule::SamplePointCloud(Application::GetLayer<RayTracerLayer>()->environment_properties, pc_samples);
  for (const auto &sample : pc_samples) {
    if (sample.m_hit) {
      points.push_back(sample.m_hitInfo.position - gt.GetPosition());
      point_colors.push_back(sample.m_hitInfo.color);
      handles.push_back(sample.m_handle);
    }
  }
}

void BasicPointCloudScanner::ConstructPointCloud(const std::shared_ptr<PointCloud> &point_cloud) const {
  for (int i = 0; i < points.size(); i++) {
    point_cloud->positions.push_back(points[i]);
  }
}
