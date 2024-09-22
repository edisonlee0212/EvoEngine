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
  //SorghumSplineSegment Interpolate(int left_index, float a) const;

  void SubdivideByDistance(float subdivision_distance, std::vector<SorghumSplineSegment>& subdivided_segments) const;

  void Serialize(const std::string& name, YAML::Emitter& out) const;
  void Deserialize(const std::string& name, const YAML::Node& in);
  void GetPositionControlPoints(uint32_t segment_index, glm::vec3& p0, glm::vec3& p1, glm::vec3& p2,
                                              glm::vec3& p3) const;
  [[nodiscard]] float GetSegmentArcLength(uint32_t segment_index, float t_start = 0.f, float t_end = 1.f,
                            float tolerance = 0.0001f) const;

  [[nodiscard]] float GetArcLength(float tolerance = 0.0001f) const;
  [[nodiscard]] SorghumSplineSegment InterpolateSegment(uint32_t segment_index, float t) const;
  [[nodiscard]] std::vector<SorghumSplineSegment> RebuildFixedSizeSegments(size_t segment_count,
                                                                           float tolerance = 0.0001f) const;
  [[nodiscard]] std::vector<SorghumSplineSegment> RebuildFixedLengthSegments(float segment_length,
                                                                           float tolerance = 0.0001f) const;

  [[nodiscard]] std::vector<SorghumSplineSegment> GetStemPart() const;
  [[nodiscard]] std::vector<SorghumSplineSegment> GetLeafPart() const;
};
}  // namespace digital_agriculture_plugin