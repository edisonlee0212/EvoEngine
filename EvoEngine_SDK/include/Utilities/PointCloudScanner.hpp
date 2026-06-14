#pragma once
#include "AssetRef.hpp"
#include "PointCloud.hpp"
namespace evo_engine {
class PointCloudScanner : public IPrivateComponent {
 public:
  float rotate_angle = 0.0f;
  glm::vec2 size = glm::vec2(8, 4);
  glm::vec2 distance = glm::vec2(0.02f, 0.02f);

  std::vector<uint64_t> handles;
  std::vector<glm::vec3> points;
  std::vector<glm::vec3> point_colors;
  AssetRef point_cloud_drop_ref;
  void ConstructPointCloud(const std::shared_ptr<PointCloud> &point_cloud) const;

  void Scan();
};
}  // namespace evo_engine
