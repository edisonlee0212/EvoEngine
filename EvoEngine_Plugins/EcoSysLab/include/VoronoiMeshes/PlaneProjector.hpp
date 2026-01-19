#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

namespace kinDS {
class PlaneProjector {
 public:
  // Construct from plane-local -> world transforms
  // Local coordinates are (u, 0, v)
  PlaneProjector(const glm::mat4& planeAToWorld, const glm::mat4& planeBToWorld);

  // Project local v on plane A to local return value on plane B
  glm::vec2 project(const glm::vec2& v) const;

 private:
  // Extract origin + spanning vectors from transform
  void extractPlaneFromTransform(const glm::mat4& M, glm::vec3& origin, glm::vec3& u, glm::vec3& v);

  // Plane data
  glm::vec3 m_oA, m_uA, m_vA;
  glm::vec3 m_oB, m_uB, m_vB;

  // Normals
  glm::vec3 m_nA, m_nB;

  // Parallel or not
  bool m_parallel;

  // --- Non-parallel case ---
  glm::vec3 m_axis;
  float m_angle;
  glm::mat3 m_rot;
  glm::vec3 m_p0;  // point on intersection line

  // --- Parallel case ---
  float m_dB;  // plane B offset

  // Helpers
  glm::vec3 localAToWorld(float a, float b) const;
  glm::vec3 applyTransform(const glm::vec3& x) const;
  glm::vec2 worldToLocalB(const glm::vec3& x) const;
};
}  // namespace kinDS
