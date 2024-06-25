#pragma once
#include "IDataComponent.hpp"

namespace evo_engine {
struct Vertex;

struct Bound {
  glm::vec3 min = glm::vec3(FLT_MAX);
  glm::vec3 max = glm::vec3(-FLT_MAX);
  [[nodiscard]] glm::vec3 Size() const;
  [[nodiscard]] glm::vec3 Center() const;
  [[nodiscard]] bool InBound(const glm::vec3& position) const;
  void ApplyTransform(const glm::mat4& transform);
  void PopulateCorners(std::vector<glm::vec3>& corners) const;
};
struct Ray : IDataComponent {
  glm::vec3 start;
  glm::vec3 direction;
  float length;
  Ray() = default;
  Ray(const glm::vec3& start, const glm::vec3& end);
  Ray(const glm::vec3& start, const glm::vec3& direction, float length);
  [[nodiscard]] bool Intersect(const glm::vec3& position, float radius) const;
  [[nodiscard]] bool Intersect(const glm::mat4& transform, const Bound& bound) const;
  [[nodiscard]] glm::vec3 GetEnd() const;
  [[nodiscard]] static glm::vec3 ClosestPointOnLine(const glm::vec3& point, const glm::vec3& a, const glm::vec3& b);
};
struct Plane {
  explicit Plane(const glm::vec4& param);
  Plane(const glm::vec3& normal, float distance);
  float a, b, c, d;
  Plane();
  void Normalize();
  [[nodiscard]] float CalculateTriangleDistance(const std::vector<Vertex>& vertices, const glm::uvec3& triangle) const;
  [[nodiscard]] float CalculateTriangleMaxDistance(const std::vector<Vertex>& vertices,
                                                   const glm::uvec3& triangle) const;
  [[nodiscard]] float CalculateTriangleMinDistance(const std::vector<Vertex>& vertices,
                                                   const glm::uvec3& triangle) const;
  [[nodiscard]] glm::vec3 GetNormal() const;
  [[nodiscard]] float GetDistance() const;
  [[nodiscard]] float CalculatePointDistance(const glm::vec3& point) const;
};
}  // namespace evo_engine
