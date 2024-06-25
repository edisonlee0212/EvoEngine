#include "SorghumSpline.hpp"
#include "SorghumLayer.hpp"

using namespace eco_sys_lab;

SorghumSplineSegment::SorghumSplineSegment(const glm::vec3& position, const glm::vec3& up, const glm::vec3& front,
                                           const float radius, const float theta, const float left_height_offset,
                                           const float right_height_offset) {
  this->m_position = position;
  this->m_up = up;
  this->m_front = front;
  this->m_radius = radius;
  this->m_theta = theta;
  this->m_leftHeightOffset = left_height_offset;
  this->m_rightHeightOffset = right_height_offset;
}

glm::vec3 SorghumSplineSegment::GetLeafPoint(const float angle) const {
  if (glm::abs(m_theta) < 90.0f) {
    const auto arc_radius = m_radius / glm::sin(glm::radians(glm::max(89.f, m_theta)));
    const auto center = m_position + arc_radius * m_up;
    const auto direction = glm::normalize(glm::rotate(m_up, glm::radians(angle), m_front));
    const auto point = center - arc_radius * direction;
    const auto distance_to_center = glm::sin(glm::radians(angle)) * arc_radius / m_radius;
    return point - (angle < 0 ? m_leftHeightOffset : m_rightHeightOffset) * glm::pow(distance_to_center, 2.f) * m_up;
  }
  const auto radius = m_radius;
  const auto center = m_position + radius * m_up;
  const auto direction = glm::rotate(m_up, glm::radians(angle), m_front);
  return center - radius * direction;
}

glm::vec3 SorghumSplineSegment::GetStemPoint(const float angle) const {
  const auto direction = glm::rotate(m_up, glm::radians(angle), m_front);
  return m_position - m_radius * direction;
}

glm::vec3 SorghumSplineSegment::GetNormal(const float angle) const {
  return glm::normalize(glm::rotate(m_up, glm::radians(angle), m_front));
}

SorghumSplineSegment SorghumSpline::Interpolate(int left_index, float a) const {
  if (a < glm::epsilon<float>()) {
    return m_segments.at(left_index);
  }
  if (1.f - a < glm::epsilon<float>()) {
    return m_segments.at(left_index + 1);
  }
  SorghumSplineSegment ret_val{};
  const auto& s1 = m_segments.at(left_index);
  const auto& s2 = m_segments.at(left_index + 1);
  SorghumSplineSegment s0, s3;
  if (left_index == 0) {
    s0.m_position = 2.f * s1.m_position - s2.m_position;
    s0.m_front = 2.f * s1.m_front - s2.m_front;
    s0.m_up = 2.f * s1.m_up - s2.m_up;
    s0.m_radius = 2.f * s1.m_radius - s2.m_radius;
    s0.m_theta = 2.f * s1.m_theta - s2.m_theta;
    s0.m_leftHeightOffset = 2.f * s1.m_leftHeightOffset - s2.m_leftHeightOffset;
    s0.m_rightHeightOffset = 2.f * s1.m_rightHeightOffset - s2.m_rightHeightOffset;
  } else {
    s0 = m_segments.at(left_index - 1);
  }
  if (left_index < m_segments.size() - 1) {
    s3.m_position = 2.f * s2.m_position - s1.m_position;
    s3.m_front = 2.f * s2.m_front - s1.m_front;
    s3.m_up = 2.f * s2.m_up - s1.m_up;
    s3.m_radius = 2.f * s2.m_radius - s1.m_radius;
    s3.m_theta = 2.f * s2.m_theta - s1.m_theta;
    s3.m_leftHeightOffset = 2.f * s2.m_leftHeightOffset - s1.m_leftHeightOffset;
    s3.m_rightHeightOffset = 2.f * s2.m_rightHeightOffset - s1.m_rightHeightOffset;
  } else {
    s3 = m_segments.at(left_index + 2);
  }
  // Strands::CubicInterpolation(s0.m_position, s1.m_position, s2.m_position, s3.m_position, retVal.m_position,
  // retVal.m_front, a); retVal.m_front = glm::normalize(retVal.m_front); retVal.m_up =
  // Strands::CubicInterpolation(s0.m_up, s1.m_up, s2.m_up, s3.m_up, a);

  ret_val.m_radius =
      glm::mix(s1.m_radius, s2.m_radius,
               a);  // Strands::CubicInterpolation(s0.m_radius, s1.m_radius, s2.m_radius, s3.m_radius, a);
  ret_val.m_theta = glm::mix(s1.m_theta, s2.m_theta,
                             a);  // Strands::CubicInterpolation(s0.m_theta, s1.m_theta, s2.m_theta, s3.m_theta, a);
  ret_val.m_leftHeightOffset = glm::mix(s1.m_leftHeightOffset, s2.m_leftHeightOffset,
                                        a);  // Strands::CubicInterpolation(s0.m_leftHeightOffset,
                                             // s1.m_leftHeightOffset, s2.m_leftHeightOffset, s3.m_leftHeightOffset, a);
  ret_val.m_rightHeightOffset =
      glm::mix(s1.m_rightHeightOffset, s2.m_rightHeightOffset,
               a);  // Strands::CubicInterpolation(s0.m_rightHeightOffset, s1.m_rightHeightOffset,
                    // s2.m_rightHeightOffset, s3.m_rightHeightOffset, a);

  ret_val.m_position = glm::mix(s1.m_position, s2.m_position, a);
  ret_val.m_front = glm::normalize(glm::mix(s1.m_front, s2.m_front, a));
  ret_val.m_up = glm::normalize(glm::mix(s1.m_up, s2.m_up, a));
  ret_val.m_up = glm::normalize(glm::cross(glm::cross(ret_val.m_front, ret_val.m_up), ret_val.m_front));
  return ret_val;
}

void SorghumSpline::Subdivide(const float subdivision_distance,
                              std::vector<SorghumSplineSegment>& subdivided_segments) const {
  std::vector<float> lengths;
  lengths.resize(m_segments.size() - 1);
  for (int i = 0; i < m_segments.size() - 1; i++) {
    lengths[i] = glm::distance(m_segments.at(i).m_position, m_segments.at(i + 1).m_position);
  }
  int current_index = 0;
  float accumulated_distance = 0.f;
  subdivided_segments.emplace_back(m_segments.front());
  while (true) {
    accumulated_distance += subdivision_distance;
    const auto current_segment_length = lengths.at(current_index);
    if (accumulated_distance > current_segment_length) {
      current_index++;
      accumulated_distance -= current_segment_length;
    }
    if (current_index < lengths.size() - 1)
      subdivided_segments.emplace_back(Interpolate(current_index, accumulated_distance / current_segment_length));
    else
      break;
  }
}
