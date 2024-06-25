#pragma once
#include "EvoEngine_SDK_PCH.hpp"

#include "Application.hpp"
#include "CUDAModule.hpp"
#include "IPrivateComponent.hpp"
#include "PointCloud.hpp"
namespace evo_engine {
class BasicPointCloudScanner : public IPrivateComponent {
 public:
  float rotate_angle = 0.0f;
  glm::vec2 size = glm::vec2(8, 4);
  glm::vec2 distance = glm::vec2(0.02f, 0.02f);

  std::vector<uint64_t> handles;
  std::vector<glm::vec3> points;
  std::vector<glm::vec3> point_colors;
  void ConstructPointCloud(const std::shared_ptr<PointCloud> &point_cloud) const;

  void Scan();

  bool OnInspect(const std::shared_ptr<EditorLayer> &editor_layer) override;

  void Serialize(YAML::Emitter &out) const override;

  void Deserialize(const YAML::Node &in) override;
};
}  // namespace evo_engine