//
// Created by lllll on 12/15/2021.
//

#include "PointCloudScanner.hpp"

#include "Jobs.hpp"
#include "Resources.hpp"
#include "Scene.hpp"
using namespace evo_engine;

void PointCloudScanner::Scan() {
  const auto column = static_cast<unsigned>(size.x / distance.x);
  const int column_start = -static_cast<int>(column / 2);
  const auto row = static_cast<unsigned>(size.y / distance.y);
  const int row_start = -static_cast<int>(row / 2);
  const auto sample_size = column * row;
  const auto gt = GetScene()->GetDataComponent<GlobalTransform>(GetOwner());
  const glm::vec3 center = gt.GetPosition();
  const glm::vec3 front = gt.GetRotation() * glm::vec3(0, 0, -1);
  const glm::vec3 up = gt.GetRotation() * glm::vec3(0, 1, 0);
  const glm::vec3 left = gt.GetRotation() * glm::vec3(1, 0, 0);
  const glm::vec3 actual_vector = glm::rotate(front, glm::radians(rotate_angle), up);
  std::vector<PointCloudSample> pc_samples;
  pc_samples.resize(sample_size);

  std::vector<std::shared_future<void>> results;
  Jobs::RunParallelFor(sample_size, [&](size_t i) {
    const int column_index = static_cast<int>(i) / row;
    const int row_index = static_cast<int>(i) % row;
    const auto position = center + left * static_cast<float>(column_start + column_index) * distance.x +
                          up * static_cast<float>(row_start + row_index) * distance.y;
    pc_samples[i].start = position;
    pc_samples[i].direction = glm::normalize(actual_vector);
  });

  PointCloud::SampleCurrentScene(pc_samples);
  for (const auto &sample : pc_samples) {
    if (sample.hit_count != 0) {
      points.push_back(sample.hit_info.position);
      point_colors.emplace_back(sample.hit_info.color);
      handles.push_back(sample.handle);
    }
  }
}

void PointCloudScanner::ConstructPointCloud(const std::shared_ptr<PointCloud> &point_cloud) const {
  point_cloud->positions.reserve(points.size());
  point_cloud->colors.reserve(point_colors.size());
  for (int i = 0; i < points.size(); i++) {
    point_cloud->positions.emplace_back(points[i]);
    point_cloud->colors.emplace_back(point_colors[i], 1.f);
  }
}
