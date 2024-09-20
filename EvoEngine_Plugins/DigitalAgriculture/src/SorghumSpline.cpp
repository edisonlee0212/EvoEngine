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

void SorghumSpline::Subdivide(const float subdivision_distance,
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
      subdivided_segments.emplace_back(Interpolate(current_index, accumulated_distance / current_segment_length));
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
