#pragma once
#include <Curve.hpp>
#include "SorghumGrowthStages.hpp"
using namespace evo_engine;
namespace digital_agriculture_plugin {
class SorghumSplineSegment {
 public:
  glm::vec3 position;
  glm::vec3 front;
  glm::vec3 up;
  float radius;
  float theta;
  float left_height_offset = 1.0f;
  float right_height_offset = 1.0f;
  SorghumSplineSegment() = default;
  SorghumSplineSegment(const glm::vec3& position, const glm::vec3& up, const glm::vec3& front, float radius,
                       float theta, float left_height_offset = 0.0f, float right_height_offset = 0.0f);

  [[nodiscard]] glm::vec3 GetLeafPoint(float angle) const;
  [[nodiscard]] glm::vec3 GetStemPoint(float angle) const;
  [[nodiscard]] glm::vec3 GetNormal(float angle) const;

};

class SorghumSpline {
 public:
  std::vector<SorghumSplineSegment> segments;
  SorghumSplineSegment Interpolate(int left_index, float a) const;

  void Subdivide(float subdivision_distance, std::vector<SorghumSplineSegment>& subdivided_segments) const;

  void Serialize(const std::string &name, YAML::Emitter& out) const;
  void Deserialize(const std::string& name, const YAML::Node& in);
};
}  // namespace digital_agriculture_plugin