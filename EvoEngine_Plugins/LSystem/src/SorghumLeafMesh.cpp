#include "SorghumLeafMesh.hpp"

#include <Strands.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

#include <glm/common.hpp>
#include <glm/gtc/noise.hpp>
#include <glm/gtx/rotate_vector.hpp>

using namespace l_system_plugin;

namespace {

constexpr uint32_t kLeafMeshBendingSalt = 0xB67A2C11u;
constexpr uint32_t kLeafMeshCurlingProfileSalt = 0xC1D49E27u;
constexpr uint32_t kLeafMeshWavinessProfileSalt = 0xD8F2B431u;
constexpr uint32_t kLeafMeshWidthProfileSalt = 0xE2AB7D53u;

glm::vec3 SafeNormalize(const glm::vec3& v, const glm::vec3& fallback) {
  const float sq = glm::dot(v, v);
  if (!std::isfinite(sq) || sq <= 1.0e-12f) {
    return fallback;
  }
  return glm::normalize(v);
}

float ResolveVerticalSubdivision(const SorghumLeafMeshSettings& settings) {
  return std::max(0.0001f, settings.vertical_subdivision_length);
}

float ComputePolylineLength(const std::vector<StemContext::Segment>& segments) {
  if (segments.size() < 2) return 0.0f;
  float length = 0.0f;
  for (size_t i = 1; i < segments.size(); ++i) {
    length += glm::distance(segments[i - 1].position, segments[i].position);
  }
  return std::max(0.0f, length);
}

SorghumSplineSegment ToSplineSegment(const StemContext::Segment& s) {
  return SorghumSplineSegment(
      s.position,
      s.up,
      s.front,
      s.radius,
      s.theta,
      s.left_height_offset,
      s.right_height_offset);
}

SorghumSplineSegment SampleStemAtNormalizedArc(const StemContext& stem_ctx,
                                               const float s_norm) {
  if (stem_ctx.segments.empty()) {
    return SorghumSplineSegment();
  }
  if (stem_ctx.segments.size() == 1) {
    return ToSplineSegment(stem_ctx.segments.front());
  }

  const float target = std::clamp(s_norm, 0.0f, 1.0f) * ComputePolylineLength(stem_ctx.segments);
  float accum = 0.0f;
  for (size_t i = 1; i < stem_ctx.segments.size(); ++i) {
    const auto& a = stem_ctx.segments[i - 1];
    const auto& b = stem_ctx.segments[i];
    const float seg_len = glm::distance(a.position, b.position);
    if (seg_len <= 1.0e-6f) {
      continue;
    }

    if (target <= accum + seg_len || i + 1 == stem_ctx.segments.size()) {
      const float u = std::clamp((target - accum) / seg_len, 0.0f, 1.0f);
      SorghumSplineSegment out;
      out.position = glm::mix(a.position, b.position, u);
      out.front = SafeNormalize(glm::mix(a.front, b.front, u), glm::vec3(0.0f, 1.0f, 0.0f));
      out.up = SafeNormalize(glm::mix(a.up, b.up, u), glm::vec3(0.0f, 0.0f, 1.0f));
      out.up = SafeNormalize(glm::cross(glm::cross(out.front, out.up), out.front), glm::vec3(0.0f, 0.0f, 1.0f));
      out.radius = glm::mix(a.radius, b.radius, u);
      out.theta = glm::mix(a.theta, b.theta, u);
      out.left_height_offset = glm::mix(a.left_height_offset, b.left_height_offset, u);
      out.right_height_offset = glm::mix(a.right_height_offset, b.right_height_offset, u);
      return out;
    }

    accum += seg_len;
  }

  return ToSplineSegment(stem_ctx.segments.back());
}

}  // namespace

SorghumSplineSegment::SorghumSplineSegment(const glm::vec3& position,
                                           const glm::vec3& up,
                                           const glm::vec3& front,
                                           const float radius,
                                           const float theta,
                                           const float left_height_offset,
                                           const float right_height_offset) {
  this->position = position;
  this->up = up;
  this->front = front;
  this->radius = radius;
  this->theta = theta;
  this->left_height_offset = left_height_offset;
  this->right_height_offset = right_height_offset;
}

glm::vec3 SorghumSplineSegment::GetLeafPoint(const float angle_deg) const {
  if (glm::abs(theta) < 90.0f) {
    const float arc_radius = radius / glm::sin(glm::radians(glm::max(89.0f, theta)));
    const glm::vec3 center = position + arc_radius * up;
    const glm::vec3 direction = glm::normalize(glm::rotate(up, glm::radians(angle_deg), front));
    const glm::vec3 point = center - arc_radius * direction;
    const float distance_to_center = glm::sin(glm::radians(angle_deg)) * arc_radius / std::max(1.0e-6f, radius);
    const float offset = angle_deg < 0.0f ? left_height_offset : right_height_offset;
    return point - offset * glm::pow(distance_to_center, 2.0f) * up;
  }
  const glm::vec3 center = position + radius * up;
  const glm::vec3 direction = glm::rotate(up, glm::radians(angle_deg), front);
  return center - radius * direction;
}

glm::vec3 SorghumSplineSegment::GetStemPoint(const float angle_deg) const {
  const glm::vec3 direction = glm::rotate(up, glm::radians(angle_deg), front);
  return position - radius * direction;
}

glm::vec3 SorghumSplineSegment::GetNormal(const float angle_deg) const {
  return glm::normalize(glm::rotate(up, glm::radians(angle_deg), front));
}

void SorghumSpline::SubdivideByDistance(const float subdivision_distance,
                                        std::vector<SorghumSplineSegment>& subdivided_segments) const {
  subdivided_segments.clear();
  if (segments.empty()) return;
  if (segments.size() == 1) {
    subdivided_segments.emplace_back(segments.front());
    return;
  }

  std::vector<float> lengths(segments.size() - 1, 0.0f);
  for (size_t i = 0; i + 1 < segments.size(); i++) {
    lengths[i] = glm::distance(segments[i].position, segments[i + 1].position);
  }

  const float step = std::max(0.0001f, subdivision_distance);
  size_t current_index = 0;
  float accumulated_distance = 0.0f;
  subdivided_segments.emplace_back(segments.front());
  while (current_index < lengths.size()) {
    accumulated_distance += step;
    const float current_segment_length = std::max(1.0e-6f, lengths[current_index]);
    if (accumulated_distance > current_segment_length) {
      accumulated_distance -= current_segment_length;
      current_index++;
      continue;
    }

    if (current_index < lengths.size() - 1) {
      subdivided_segments.emplace_back(
          InterpolateSegment(static_cast<uint32_t>(current_index), accumulated_distance / current_segment_length));
    } else {
      break;
    }
  }

  if (glm::distance(subdivided_segments.back().position, segments.back().position) > 1.0e-6f) {
    subdivided_segments.emplace_back(segments.back());
  }
}

void SorghumSpline::GetPositionControlPoints(const uint32_t segment_index,
                                             glm::vec3& p0,
                                             glm::vec3& p1,
                                             glm::vec3& p2,
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

float SorghumSpline::GetSegmentArcLength(const uint32_t segment_index,
                                         const float t_start,
                                         const float t_end,
                                         const float tolerance) const {
  if (segment_index >= segments.size() - 1) return 0.0f;
  glm::vec3 p[4];
  GetPositionControlPoints(segment_index, p[0], p[1], p[2], p[3]);
  return evo_engine::Strands::CalculateLengthAdaptive(p[0], p[1], p[2], p[3], t_start, t_end, tolerance);
}

float SorghumSpline::GetArcLength(const float tolerance) const {
  if (segments.size() < 2) return 0.0f;
  float sum = 0.0f;
  for (uint32_t i = 0; i + 1 < segments.size(); i++) {
    sum += GetSegmentArcLength(i, 0.0f, 1.0f, tolerance);
  }
  return sum;
}

SorghumSplineSegment SorghumSpline::InterpolateSegment(const uint32_t segment_index, const float t) const {
  SorghumSplineSegment ret;

  glm::vec3 p[4];
  GetPositionControlPoints(segment_index, p[0], p[1], p[2], p[3]);
  evo_engine::Strands::CubicInterpolation(p[0], p[1], p[2], p[3], ret.position, ret.front, t);

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
  ret.up = evo_engine::Strands::CubicInterpolation(u[0], u[1], u[2], u[3], t);

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
  ret.radius = evo_engine::Strands::CubicInterpolation(radius[0], radius[1], radius[2], radius[3], t);

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
  ret.theta = evo_engine::Strands::CubicInterpolation(theta[0], theta[1], theta[2], theta[3], t);

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
  ret.left_height_offset = evo_engine::Strands::CubicInterpolation(l[0], l[1], l[2], l[3], t);

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
  ret.right_height_offset = evo_engine::Strands::CubicInterpolation(r[0], r[1], r[2], r[3], t);

  return ret;
}

std::vector<SorghumSplineSegment> SorghumSpline::RebuildFixedSizeSegments(const size_t segment_count,
                                                                           const float tolerance) const {
  std::vector<SorghumSplineSegment> out;
  if (segments.empty() || segment_count == 0) return out;

  const float total_arc_length = GetArcLength(tolerance);
  const float target_segment_length = total_arc_length / static_cast<float>(segment_count);
  float t = 0.0f;
  float remaining_length = target_segment_length;

  for (uint32_t segment_index = 0; segment_index + 1 < segments.size(); segment_index++) {
    glm::vec3 p[4];
    GetPositionControlPoints(segment_index, p[0], p[1], p[2], p[3]);
    while (true) {
      const float t_next = evo_engine::Strands::FindTAdaptive(
          p[0], p[1], p[2], p[3], t, remaining_length, tolerance);
      if (t_next != 1.0f) {
        t = t_next;
        remaining_length = target_segment_length;
        out.emplace_back(InterpolateSegment(segment_index, t));
      } else {
        remaining_length -= evo_engine::Strands::CalculateLengthAdaptive(
            p[0], p[1], p[2], p[3], t, 1.0f, tolerance);
        t = 0.0f;
        break;
      }
    }
  }

  out.resize(segment_count);
  out.back() = segments.back();
  return out;
}

std::vector<SorghumSplineSegment> SorghumSpline::RebuildFixedLengthSegments(const float segment_length,
                                                                             const float tolerance) const {
  std::vector<SorghumSplineSegment> out;
  if (segments.size() < 2) return out;

  float t = 0.0f;
  float remaining_length = std::max(0.0001f, segment_length);

  for (uint32_t segment_index = 0; segment_index + 1 < segments.size(); segment_index++) {
    glm::vec3 p[4];
    GetPositionControlPoints(segment_index, p[0], p[1], p[2], p[3]);
    while (true) {
      const float t_next = evo_engine::Strands::FindTAdaptive(
          p[0], p[1], p[2], p[3], t, remaining_length, tolerance);
      if (t_next != 1.0f) {
        t = t_next;
        remaining_length = segment_length;
        out.emplace_back(InterpolateSegment(segment_index, t));
      } else {
        remaining_length -= evo_engine::Strands::CalculateLengthAdaptive(
            p[0], p[1], p[2], p[3], t, 1.0f, tolerance);
        t = 0.0f;
        break;
      }
    }
  }

  if (GetSegmentArcLength(static_cast<uint32_t>(segments.size() - 1), t, 1.0f, tolerance) > segment_length * 0.5f) {
    out.emplace_back(segments.back());
  }
  return out;
}

std::vector<SorghumSplineSegment> SorghumSpline::GetStemPart() const {
  std::vector<SorghumSplineSegment> out;
  for (const auto& seg : segments) {
    if (seg.theta >= 90.0f) break;
    out.emplace_back(seg);
  }
  return out;
}

std::vector<SorghumSplineSegment> SorghumSpline::GetLeafPart() const {
  std::vector<SorghumSplineSegment> out;
  for (auto it = segments.begin(); it != segments.end(); ++it) {
    if (it->theta <= 90.0f) {
      out.insert(out.end(), it, segments.end());
      break;
    }
  }
  return out;
}

void l_system_plugin::BuildLeafSplineFromState(const SorghumLeaf& leaf,
                                               const StemContext& stem_ctx,
                                               const SampledSorghumParams& params,
                                               const SorghumLeafMeshSettings& settings,
                                               SorghumSpline& out_spline) {
  out_spline.segments.clear();
  if (!leaf.alive) return;
  if (stem_ctx.segments.empty()) return;

  const float vertical_step = ResolveVerticalSubdivision(settings);
  const SorghumSplineSegment anchor = SampleStemAtNormalizedArc(stem_ctx, leaf.s_along_parent_norm);
  const glm::vec3 stem_base = stem_ctx.segments.front().position;

  float stem_length = ComputePolylineLength(stem_ctx.segments);
  if (stem_length <= 1.0e-6f) {
    stem_length = std::max(vertical_step, glm::distance(stem_ctx.segments.front().position,
                                                        stem_ctx.segments.back().position));
  }
  stem_length = std::max(stem_length, vertical_step);

  glm::vec3 stem_direction = SafeNormalize(anchor.front,
      SafeNormalize(stem_ctx.segments.back().position - stem_base, glm::vec3(0.0f, 1.0f, 0.0f)));
  glm::vec3 stem_up = SafeNormalize(anchor.up, glm::vec3(0.0f, 0.0f, 1.0f));
  glm::vec3 base_left = SafeNormalize(glm::cross(stem_up, stem_direction), glm::vec3(1.0f, 0.0f, 0.0f));

  glm::vec3 leaf_left = SafeNormalize(
      glm::rotate(base_left, glm::radians(leaf.roll_angle_deg), stem_direction), base_left);
  glm::vec3 leaf_up = SafeNormalize(glm::cross(stem_direction, leaf_left), stem_up);
  glm::vec3 stem_offset = std::max(0.0005f, anchor.radius) * -leaf_up;

  glm::vec3 direction = SafeNormalize(
      glm::rotate(stem_direction, glm::radians(leaf.insertion_angle_deg), leaf_left), stem_direction);

  const float starting_point = std::clamp(leaf.s_along_parent_norm, 0.0f, 1.0f);
  float back_track_ratio = 0.05f;
  if (starting_point < back_track_ratio) back_track_ratio = starting_point;
  const float sheath_ratio = starting_point - back_track_ratio;
  const float stem_width = std::max(0.0005f, anchor.radius);

  if (sheath_ratio > 0.0f) {
    const int root_to_sheath_node_count = static_cast<int>(glm::min(
        2.0f, stem_length * sheath_ratio / vertical_step));
    for (int i = 0; i < root_to_sheath_node_count; i++) {
      const float factor = static_cast<float>(i) /
                           static_cast<float>(std::max(1, root_to_sheath_node_count));
      const float current_root_to_sheath_point = glm::mix(0.0f, sheath_ratio, factor);
      const glm::vec3 up = SafeNormalize(glm::cross(stem_direction, leaf_left), leaf_up);
      out_spline.segments.emplace_back(
          stem_base + stem_direction * current_root_to_sheath_point * stem_length + stem_offset,
          up,
          stem_direction,
          stem_width,
          180.0f,
          0.0f,
          0.0f);
    }
  }

  const float blade_length = std::max(vertical_step, leaf.blade_length);
  const int node_amount = static_cast<int>(glm::max(4.0f, blade_length / vertical_step));
  const float unit_length = blade_length / static_cast<float>(std::max(1, node_amount));
  const int node_to_full_expand = std::max(
      1, static_cast<int>(0.1f * blade_length / vertical_step));

  glm::vec2 current_period(0.0f, 0.0f);
  glm::vec3 node_position = anchor.position + stem_offset;
  for (int i = 1; i <= node_amount; i++) {
    const float factor = static_cast<float>(i) /
                         static_cast<float>(std::max(1, node_amount));

    const float bend_profile = EvaluatePlottedDeterministic(
      params.leaf_bending_development_curve,
      factor,
      leaf.node_random,
      kLeafMeshBendingSalt,
      0.0f,
      1.0f);
    const float senescence_bend_bias_deg = 18.0f * std::clamp(leaf.senescence_phase, 0.0f, 1.0f) * factor;
    const float rotate_angle = leaf.bending * bend_profile + senescence_bend_bias_deg;
    const glm::vec3 current_direction = SafeNormalize(
        glm::rotate(direction, glm::radians(rotate_angle), leaf_left), direction);
    node_position += current_direction * unit_length;

    const float expand_profile = EvaluatePlottedDeterministic(
      params.curling_along_leaf,
      factor,
      leaf.node_random,
      kLeafMeshCurlingProfileSalt,
      0.0f,
      1.0f);
    const float expand_angle = leaf.curling * expand_profile;

    const float collar_factor = glm::min(
        1.0f, static_cast<float>(i) / static_cast<float>(node_to_full_expand));

    const float waviness_profile = EvaluatePlottedDeterministic(
      params.waviness_along_leaf,
      factor,
      leaf.node_random,
      kLeafMeshWavinessProfileSalt,
      0.0f,
      std::numeric_limits<float>::infinity());
    const float waviness = leaf.waviness * waviness_profile;
    const float waviness_frequency = std::max(0.0f, leaf.waviness_frequency);
    current_period += glm::vec2(waviness_frequency, waviness_frequency);

    const float width_profile = EvaluatePlottedDeterministic(
      params.width_along_leaf,
      factor,
      leaf.node_random,
      kLeafMeshWidthProfileSalt,
      0.0f,
      std::numeric_limits<float>::infinity());
    const float width = glm::mix(stem_width + 0.002f, width_profile, collar_factor);
    const float angle = 90.0f - (90.0f - expand_angle) * glm::pow(collar_factor, 2.0f);

    const glm::vec3 up = SafeNormalize(glm::cross(current_direction, leaf_left), leaf_up);
    out_spline.segments.emplace_back(
        node_position,
        up,
        current_direction,
        width,
        angle,
        waviness * glm::simplex(glm::vec2(current_period.x, 0.0f)),
        waviness * glm::simplex(glm::vec2(0.0f, current_period.y)));
  }
}

void l_system_plugin::GenerateBladeGeometry(const SorghumSpline& spline,
                                            const SorghumLeaf& leaf,
                                            const SampledSorghumParams& /*params*/,
                                            const SorghumLeafMeshSettings& settings,
                                            std::vector<evo_engine::Vertex>& vertices,
                                            std::vector<unsigned int>& indices,
                                            const bool current_bottom_face,
                                            const uint32_t leaf_index) {
  if (!leaf.alive) return;
  if (spline.segments.empty()) return;

  SorghumSpline temp_spline;
  spline.SubdivideByDistance(ResolveVerticalSubdivision(settings), temp_spline.segments);

  std::vector<SorghumSplineSegment> segments;
  if (settings.enable_leaf_sheath) {
    segments = temp_spline.segments;
  } else {
    segments = temp_spline.GetLeafPart();
  }
  if (segments.empty()) return;

  const unsigned int vertex_index = static_cast<unsigned int>(vertices.size());
  evo_engine::Vertex archetype{};

  const float senescence = std::clamp(leaf.senescence_phase, 0.0f, 1.0f);
  const glm::vec4 healthy_green = glm::vec4(0.20f, 0.56f, 0.18f, 1.0f);
  const glm::vec4 wilt_brown = glm::vec4(0.56f, 0.40f, 0.17f, 1.0f);
  archetype.color = glm::mix(healthy_green, wilt_brown, senescence);
  archetype.vertex_info1 = glm::uintBitsToFloat(leaf_index + 1u);

  const int horizontal_step = std::max(2, settings.horizontal_subdivision_step);
  const float x_step = 1.0f / static_cast<float>(horizontal_step) / 2.0f;
  const int segment_size = static_cast<int>(segments.size());
  const float y_leaf_step = 0.5f / static_cast<float>(std::max(1, segment_size));

  for (int i = 0; i < segment_size; i++) {
    auto& segment = segments[i];
    const float angle_step = segment.theta / static_cast<float>(horizontal_step);
    const int verts_count = horizontal_step * 2 + 1;
    for (int j = 0; j < verts_count; j++) {
      const float angle =
          (static_cast<float>(j) - static_cast<float>(horizontal_step)) * angle_step;
      glm::vec3 position = segment.GetLeafPoint(angle);
      glm::vec3 normal = segment.GetNormal(angle);
      if (i != 0 && j != 0 && j != verts_count - 1) {
        position -= normal * settings.leaf_thickness;
      }

      archetype.position = position;
      archetype.normal = current_bottom_face ? -normal : normal;
      const float y_pos = 0.5f + y_leaf_step * static_cast<float>(i);
      archetype.tex_coord = glm::vec2(j * x_step, y_pos);
      vertices.push_back(archetype);
    }

    if (i != 0) {
      for (int j = 0; j < verts_count - 1; j++) {
        if (current_bottom_face) {
          // Down triangle
          indices.emplace_back(vertex_index + i * verts_count + j);
          indices.emplace_back(vertex_index + (i - 1) * verts_count + j + 1);
          indices.emplace_back(vertex_index + (i - 1) * verts_count + j);
          // Up triangle
          indices.emplace_back(vertex_index + (i - 1) * verts_count + j + 1);
          indices.emplace_back(vertex_index + i * verts_count + j);
          indices.emplace_back(vertex_index + i * verts_count + j + 1);
        } else {
          // Down triangle
          indices.emplace_back(vertex_index + (i - 1) * verts_count + j);
          indices.emplace_back(vertex_index + (i - 1) * verts_count + j + 1);
          indices.emplace_back(vertex_index + i * verts_count + j);
          // Up triangle
          indices.emplace_back(vertex_index + i * verts_count + j + 1);
          indices.emplace_back(vertex_index + i * verts_count + j);
          indices.emplace_back(vertex_index + (i - 1) * verts_count + j + 1);
        }
      }
    }
  }
}
