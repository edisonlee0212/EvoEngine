#include "SorghumSpline.hpp"
#include "SorghumLayer.hpp"

using namespace digital_agriculture_plugin;

SorghumSplineSegment::SorghumSplineSegment(const glm::vec3& position, const glm::vec3& up, const glm::vec3& front,
                                           const float radius, const float theta, const float left_height_offset,
                                           const float right_height_offset) {
  this->position = position;
  this->up = up;
  this->front = front;
  this->radius = radius;
  this->theta = theta;
  this->left_height_offset = left_height_offset;
  this->right_height_offset = right_height_offset;
}

glm::vec3 SorghumSplineSegment::GetLeafPoint(const float angle) const {
  if (glm::abs(theta) < 90.0f) {
    const auto arc_radius = radius / glm::sin(glm::radians(glm::max(89.f, theta)));
    const auto center = position + arc_radius * up;
    const auto direction = glm::normalize(glm::rotate(up, glm::radians(angle), front));
    const auto point = center - arc_radius * direction;
    const auto distance_to_center = glm::sin(glm::radians(angle)) * arc_radius / radius;
    return point - (angle < 0 ? left_height_offset : right_height_offset) * glm::pow(distance_to_center, 2.f) * up;
  }
  const auto center = position + radius * up;
  const auto direction = glm::rotate(up, glm::radians(angle), front);
  return center - radius * direction;
}

glm::vec3 SorghumSplineSegment::GetStemPoint(const float angle) const {
  const auto direction = glm::rotate(up, glm::radians(angle), front);
  return position - radius * direction;
}

glm::vec3 SorghumSplineSegment::GetNormal(const float angle) const {
  return glm::normalize(glm::rotate(up, glm::radians(angle), front));
}
/*
SorghumSplineSegment SorghumSpline::Interpolate(int left_index, float a) const {
  if (a < glm::epsilon<float>()) {
    return segments.at(left_index);
  }
  if (1.f - a < glm::epsilon<float>()) {
    return segments.at(left_index + 1);
  }
  SorghumSplineSegment ret_val{};
  const auto& s1 = segments.at(left_index);
  const auto& s2 = segments.at(left_index + 1);
  SorghumSplineSegment s0, s3;
  if (left_index == 0) {
    s0.position = 2.f * s1.position - s2.position;
    s0.front = 2.f * s1.front - s2.front;
    s0.up = 2.f * s1.up - s2.up;
    s0.radius = 2.f * s1.radius - s2.radius;
    s0.theta = 2.f * s1.theta - s2.theta;
    s0.left_height_offset = 2.f * s1.left_height_offset - s2.left_height_offset;
    s0.right_height_offset = 2.f * s1.right_height_offset - s2.right_height_offset;
  } else {
    s0 = segments.at(left_index - 1);
  }
  if (left_index < segments.size() - 1) {
    s3.position = 2.f * s2.position - s1.position;
    s3.front = 2.f * s2.front - s1.front;
    s3.up = 2.f * s2.up - s1.up;
    s3.radius = 2.f * s2.radius - s1.radius;
    s3.theta = 2.f * s2.theta - s1.theta;
    s3.left_height_offset = 2.f * s2.left_height_offset - s1.left_height_offset;
    s3.right_height_offset = 2.f * s2.right_height_offset - s1.right_height_offset;
  } else {
    s3 = segments.at(left_index + 2);
  }
  // Strands::CubicInterpolation(s0.position, s1.position, s2.position, s3.position, retVal.position,
  // retVal.front, a); retVal.front = glm::normalize(retVal.front); retVal.up =
  // Strands::CubicInterpolation(s0.up, s1.up, s2.up, s3.up, a);

  ret_val.radius = glm::mix(s1.radius, s2.radius,
                            a);  // Strands::CubicInterpolation(s0.radius, s1.radius, s2.radius, s3.radius, a);
  ret_val.theta = glm::mix(s1.theta, s2.theta,
                           a);  // Strands::CubicInterpolation(s0.theta, s1.theta, s2.theta, s3.theta, a);
  ret_val.left_height_offset = glm::mix(s1.left_height_offset, s2.left_height_offset,
                                        a);  // Strands::CubicInterpolation(s0.left_height_offset,
                                             // s1.left_height_offset, s2.left_height_offset, s3.left_height_offset, a);
  ret_val.right_height_offset =
      glm::mix(s1.right_height_offset, s2.right_height_offset,
               a);  // Strands::CubicInterpolation(s0.right_height_offset, s1.right_height_offset,
                    // s2.right_height_offset, s3.right_height_offset, a);

  ret_val.position = glm::mix(s1.position, s2.position, a);
  ret_val.front = glm::normalize(glm::mix(s1.front, s2.front, a));
  ret_val.up = glm::normalize(glm::mix(s1.up, s2.up, a));
  ret_val.up = glm::normalize(glm::cross(glm::cross(ret_val.front, ret_val.up), ret_val.front));
  return ret_val;
}
*/
void SorghumSpline::SubdivideByDistance(const float subdivision_distance,
                                        std::vector<SorghumSplineSegment>& subdivided_segments) const {
  std::vector<float> lengths;
  lengths.resize(segments.size() - 1);
  for (int i = 0; i < segments.size() - 1; i++) {
    lengths[i] = glm::distance(segments.at(i).position, segments.at(i + 1).position);
  }
  int current_index = 0;
  float accumulated_distance = 0.f;
  subdivided_segments.emplace_back(segments.front());
  while (true) {
    accumulated_distance += subdivision_distance;
    const auto current_segment_length = lengths.at(current_index);
    if (accumulated_distance > current_segment_length) {
      current_index++;
      accumulated_distance -= current_segment_length;
    }
    if (current_index < lengths.size() - 1)
      subdivided_segments.emplace_back(
          InterpolateSegment(current_index, accumulated_distance / current_segment_length));
    else
      break;
  }
}

void SorghumSpline::Serialize(const std::string& name, YAML::Emitter& out) const {
  out << YAML::Key << name << YAML::BeginMap;
  Serialization::SerializeVector("segments", segments, out);
  out << YAML::EndMap;
}

void SorghumSpline::Deserialize(const std::string& name, const YAML::Node& in) {
  if (in[name])
    Serialization::DeserializeVector("segments", segments, in[name]);
}

void SorghumSpline::GetPositionControlPoints(const uint32_t segment_index, glm::vec3& p0, glm::vec3& p1, glm::vec3& p2,
                                             glm::vec3& p3) const {
  p1 = segments[segment_index].position;
  p2 = segments[segment_index + 1].position;
  if (segment_index == 0) {
    p0 = p1 * 2.0f - p2;
  } else {
    p0 = segments[segment_index - 1].position;
  }
  if (segment_index + 1 == segments.size() - 1) {
    p3 = p2 * 2.0f - p1;
  } else {
    p3 = segments[segment_index + 2].position;
  }
}

float SorghumSpline::GetSegmentArcLength(const uint32_t segment_index, const float t_start, const float t_end,
                                         const float tolerance) const {
  if (segment_index == segments.size() - 1)
    return 0.f;
  glm::vec3 p[4];
  GetPositionControlPoints(segment_index, p[0], p[1], p[2], p[3]);
  return Strands::CalculateLengthAdaptive(p[0], p[1], p[2], p[3], t_start, t_end, tolerance);
}

float SorghumSpline::GetArcLength(const float tolerance) const {
  float sum = 0.f;
  for (int i = 0; i < segments.size() - 1; i++) {
    sum += GetSegmentArcLength(i, 0, 1, tolerance);
  }
  return sum;
}

SorghumSplineSegment SorghumSpline::InterpolateSegment(const uint32_t segment_index, const float t) const {
  SorghumSplineSegment ret_val;

  glm::vec3 p[4];
  GetPositionControlPoints(segment_index, p[0], p[1], p[2], p[3]);
  Strands::CubicInterpolation(p[0], p[1], p[2], p[3], ret_val.position, ret_val.front, t);

  glm::vec3 u[4];
  u[1] = segments[segment_index].up;
  u[2] = segments[segment_index + 1].up;
  if (segment_index == 0) {
    u[0] = u[1] * 2.0f - u[2];
  } else {
    u[0] = segments[segment_index - 1].up;
  }
  if (segment_index + 1 == segments.size() - 1) {
    u[3] = u[2] * 2.0f - u[1];
  } else {
    u[3] = segments[segment_index + 2].up;
  }

  ret_val.up = Strands::CubicInterpolation(u[0], u[1], u[2], u[3], t);

  float radius[4];
  radius[1] = segments[segment_index].radius;
  radius[2] = segments[segment_index + 1].radius;
  if (segment_index == 0) {
    radius[0] = radius[1] * 2.0f - radius[2];
  } else {
    radius[0] = segments[segment_index - 1].radius;
  }
  if (segment_index + 1 == segments.size() - 1) {
    radius[3] = radius[2] * 2.0f - radius[1];
  } else {
    radius[3] = segments[segment_index + 2].radius;
  }

  ret_val.radius = Strands::CubicInterpolation(radius[0], radius[1], radius[2], radius[3], t);

  float theta[4];
  theta[1] = segments[segment_index].theta;
  theta[2] = segments[segment_index + 1].theta;
  if (segment_index == 0) {
    theta[0] = theta[1] * 2.0f - theta[2];
  } else {
    theta[0] = segments[segment_index - 1].theta;
  }
  if (segment_index + 1 == segments.size() - 1) {
    theta[3] = theta[2] * 2.0f - theta[1];
  } else {
    theta[3] = segments[segment_index + 2].theta;
  }

  ret_val.theta = Strands::CubicInterpolation(theta[0], theta[1], theta[2], theta[3], t);

  float l[4];
  l[1] = segments[segment_index].left_height_offset;
  l[2] = segments[segment_index + 1].left_height_offset;
  if (segment_index == 0) {
    l[0] = l[1] * 2.0f - l[2];
  } else {
    l[0] = segments[segment_index - 1].left_height_offset;
  }
  if (segment_index + 1 == segments.size() - 1) {
    l[3] = l[2] * 2.0f - l[1];
  } else {
    l[3] = segments[segment_index + 2].left_height_offset;
  }

  ret_val.left_height_offset = Strands::CubicInterpolation(l[0], l[1], l[2], l[3], t);

  float r[4];
  r[1] = segments[segment_index].right_height_offset;
  r[2] = segments[segment_index + 1].right_height_offset;
  if (segment_index == 0) {
    r[0] = r[1] * 2.0f - r[2];
  } else {
    r[0] = segments[segment_index - 1].right_height_offset;
  }
  if (segment_index + 1 == segments.size() - 1) {
    r[3] = r[2] * 2.0f - r[1];
  } else {
    r[3] = segments[segment_index + 2].right_height_offset;
  }

  ret_val.right_height_offset = Strands::CubicInterpolation(radius[0], radius[1], radius[2], radius[3], t);
  return ret_val;
}

std::vector<SorghumSplineSegment> SorghumSpline::RebuildFixedSizeSegments(const size_t segment_count,
                                                                          const float tolerance) const {
  const float total_arc_length = GetArcLength(tolerance);
  const float target_segment_length = total_arc_length / segment_count;
  std::vector<SorghumSplineSegment> new_segments;
  float t = 0.f;
  float remaining_length = target_segment_length;
  for (uint32_t segment_index = 0; segment_index < segments.size() - 1; segment_index++) {
    glm::vec3 p[4];
    GetPositionControlPoints(segment_index, p[0], p[1], p[2], p[3]);
    while (true) {
      if (const float t_next = Strands::FindTAdaptive(p[0], p[1], p[2], p[3], t, remaining_length, tolerance);
          t_next != 1.f) {
        t = t_next;
        remaining_length = target_segment_length;
        new_segments.emplace_back(InterpolateSegment(segment_index, t));
      } else {
        remaining_length -= Strands::CalculateLengthAdaptive(p[0], p[1], p[2], p[3], t, 1.f, tolerance);
        t = 0.f;
        break;
      }
    }
  }
  new_segments.resize(segment_count);
  new_segments.back() = segments.back();
  return new_segments;
}

std::vector<SorghumSplineSegment> SorghumSpline::RebuildFixedLengthSegments(const float segment_length,
                                                                            const float tolerance) const {
  std::vector<SorghumSplineSegment> new_segments;
  float t = 0.f;
  float remaining_length = segment_length;
  for (uint32_t segment_index = 0; segment_index < segments.size() - 1; segment_index++) {
    glm::vec3 p[4];
    GetPositionControlPoints(segment_index, p[0], p[1], p[2], p[3]);
    while (true) {
      if (const float t_next = Strands::FindTAdaptive(p[0], p[1], p[2], p[3], t, remaining_length, tolerance);
          t_next != 1.f) {
        t = t_next;
        remaining_length = segment_length;
        new_segments.emplace_back(InterpolateSegment(segment_index, t));
      } else {
        remaining_length -= Strands::CalculateLengthAdaptive(p[0], p[1], p[2], p[3], t, 1.f, tolerance);
        t = 0.f;
        break;
      }
    }
  }
  if (GetSegmentArcLength(segments.size() - 1, t, 1.f) > segment_length * .5f) {
    new_segments.emplace_back(segments.back());
  }
  return new_segments;
}

std::vector<SorghumSplineSegment> SorghumSpline::GetStemPart() const {
  std::vector<SorghumSplineSegment> ret_val;
  for (const auto& i : segments) {
    if (i.theta != 180.f)
      break;
    ret_val.emplace_back(i);
  }
  return ret_val;
}

std::vector<SorghumSplineSegment> SorghumSpline::GetLeafPart() const {
  std::vector<SorghumSplineSegment> ret_val;
  for (auto i = segments.begin(); i != segments.end(); ++i) {
    if (i->theta != 180.f) {
      ret_val.insert(ret_val.end(), i, segments.end());
      break;
    }
  }
  return ret_val;
}
