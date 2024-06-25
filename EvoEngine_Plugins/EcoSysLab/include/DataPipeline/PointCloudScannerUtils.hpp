#pragma once

namespace evo_engine {
struct PointCloudSample;
struct HitInfo;

#ifndef BUILD_WITH_RAYTRACER
struct HitInfo {
  glm::vec3 position = glm::vec3(0.0f);
  glm::vec3 normal = glm::vec3(0.0f);
  glm::vec3 tangent = glm::vec3(0.0f);
  glm::vec4 color = glm::vec4(1.0f);
  glm::vec2 texCoord = glm::vec2(0.0f);
  glm::vec3 data = glm::vec4(0.0f);
  glm::vec2 data2 = glm::vec4(0.0f);
};
struct PointCloudSample {
  // Input
  glm::vec3 direction = glm::vec3(0.0f);
  glm::vec3 start = glm::vec3(0.0f);

  // Output
  uint64_t handle_ = 0;
  bool m_hit = false;

  HitInfo m_hitInfo;
};
#endif
class PointCloudCaptureSettings {
 public:
  virtual bool OnInspect() = 0;
  virtual void Save(const std::string& name, YAML::Emitter& out) const {
  }
  virtual void Load(const std::string& name, const YAML::Node& in) {
  }
  virtual void GenerateSamples(std::vector<PointCloudSample>& pointCloudSamples) = 0;
  virtual bool SampleFilter(const PointCloudSample& sample) {
    return true;
  }
};
}  // namespace evo_engine