#include "PlaneProjector.hpp"

#include <cassert>
#include <cmath>

using namespace kinDS;

static const float EPS = 1e-8f;

PlaneProjector::PlaneProjector(const glm::mat4& planeAToWorld, const glm::mat4& planeBToWorld) {
  // Extract plane A
  extractPlaneFromTransform(planeAToWorld, m_oA, m_uA, m_vA);

  // Extract plane B
  extractPlaneFromTransform(planeBToWorld, m_oB, m_uB, m_vB);

  // Compute normals
  m_nA = glm::normalize(glm::cross(m_uA, m_vA));
  m_nB = glm::normalize(glm::cross(m_uB, m_vB));

  // Check parallelism
  glm::vec3 axis = glm::cross(m_nA, m_nB);
  float axisLen = glm::length(axis);

  if (axisLen < EPS) {
    // Parallel planes
    m_parallel = true;
    m_dB = -glm::dot(m_nB, m_oB);
  } else {
    // Non-parallel planes
    m_parallel = false;

    m_axis = axis / axisLen;

    float cosTheta = glm::clamp(glm::dot(m_nA, m_nB), -1.0f, 1.0f);
    m_angle = std::acos(cosTheta);

    // Rotation matrix
    m_rot = glm::mat3(glm::rotate(glm::mat4(1.0f), m_angle, m_axis));

    // Plane offsets
    float dA = -glm::dot(m_nA, m_oA);
    float dB = -glm::dot(m_nB, m_oB);

    // Point on intersection line
    float denom = glm::dot(m_nA, glm::cross(m_nB, m_axis));

    assert(std::abs(denom) > EPS);

    m_p0 = (dB * glm::cross(m_axis, m_nA) + dA * glm::cross(m_nB, m_axis)) / denom;
  }
}

void PlaneProjector::extractPlaneFromTransform(const glm::mat4& M, glm::vec3& origin, glm::vec3& u, glm::vec3& v) {
  // Origin
  origin = glm::vec3(M * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f));

  // Spanning vectors (directions)
  u = glm::vec3(M * glm::vec4(1.0f, 0.0f, 0.0f, 0.0f));

  v = glm::vec3(M * glm::vec4(0.0f, 0.0f, 1.0f, 0.0f));
}

glm::vec3 PlaneProjector::localAToWorld(float a, float b) const {
  return m_oA + a * m_uA + b * m_vA;
}

glm::vec3 PlaneProjector::applyTransform(const glm::vec3& x) const {
  if (!m_parallel) {
    // Rotate around intersection line
    return m_p0 + m_rot * (x - m_p0);
  } else {
    // Project along plane normal
    float t = (glm::dot(m_nB, x) + m_dB) / glm::dot(m_nB, m_nA);
    return x - t * m_nA;
  }
}

glm::vec2 PlaneProjector::worldToLocalB(const glm::vec3& x) const {
  glm::vec3 w = x - m_oB;

  float uu = glm::dot(m_uB, m_uB);
  float uv = glm::dot(m_uB, m_vB);
  float vv = glm::dot(m_vB, m_vB);

  float wu = glm::dot(w, m_uB);
  float wv = glm::dot(w, m_vB);

  float det = uu * vv - uv * uv;
  assert(std::abs(det) > EPS);

  float c = (wu * vv - wv * uv) / det;
  float d = (wv * uu - wu * uv) / det;

  return glm::vec2(c, d);
}

glm::vec2 PlaneProjector::project(const glm::vec2& v) const {
  glm::vec3 xA = localAToWorld(v.x, v.y);
  glm::vec3 xW = applyTransform(xA);
  return worldToLocalB(xW);
}
