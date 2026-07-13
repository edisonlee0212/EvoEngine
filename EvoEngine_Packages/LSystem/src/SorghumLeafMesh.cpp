#include "SorghumLeafMesh.hpp"

#include <Strands.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

#include <glm/common.hpp>
#include <glm/gtc/noise.hpp>
#include <glm/gtx/rotate_vector.hpp>

using namespace l_system_package;

namespace {

constexpr uint32_t kLeafMeshBendingSalt = 0xB67A2C11u;
constexpr uint32_t kLeafMeshCurlingProfileSalt = 0xC1D49E27u;
constexpr uint32_t kLeafMeshWavinessProfileSalt = 0xD8F2B431u;
constexpr uint32_t kLeafMeshSheathWidthProfileSalt = 0x6D17A4B3u;
constexpr uint32_t kLeafMeshNeckWidthProfileSalt = 0x7E31BC65u;
constexpr uint32_t kLeafMeshWidthProfileSalt = 0xE2AB7D53u;
constexpr uint32_t kLeafMeshTropismOrderSalt = 0xF9137ABDu;
constexpr float kLeafSheathShade = 0.62f;
constexpr float kLeafNeckShade = 0.82f;
constexpr float kLeafBladeShade = 1.00f;
constexpr float kLegacyNormalizedCurlingThreshold = 1.01f;
constexpr float kLegacyNormalizedCurlingToDegrees = 90.0f;
constexpr float kMinBladeOpeningAngleDeg = 4.0f;
constexpr float kMaxBladeOpeningAngleDeg = 89.0f;

glm::vec3 SafeNormalize(const glm::vec3& v, const glm::vec3& fallback) {
  const float sq = glm::dot(v, v);
  if (!std::isfinite(sq) || sq <= 1.0e-12f) {
    return fallback;
  }
  return glm::normalize(v);
}

float ResolveBladeOpeningAngleDeg(const float raw_curling_value) {
  float opening_angle_deg = raw_curling_value;
  if (std::isfinite(opening_angle_deg) && opening_angle_deg >= 0.0f &&
      opening_angle_deg <= kLegacyNormalizedCurlingThreshold) {
    opening_angle_deg *= kLegacyNormalizedCurlingToDegrees;
  }

  // Near-zero theta collapses projected blade width into a stick-like strip.
  return std::clamp(opening_angle_deg, kMinBladeOpeningAngleDeg, kMaxBladeOpeningAngleDeg);
}

[[maybe_unused]] void NormalizeBladeStageRatios(float& stage1, float& stage2, float& stage3) {
  stage1 = std::max(0.0f, stage1);
  stage2 = std::max(0.0f, stage2);
  stage3 = std::max(0.0f, stage3);

  const float total = stage1 + stage2 + stage3;
  if (total <= 1.0e-6f) {
    stage1 = stage2 = stage3 = 1.0f / 3.0f;
    return;
  }

  stage1 /= total;
  stage2 /= total;
  stage3 /= total;
}

[[maybe_unused]] float EvaluateBladeStageWidthScale(const float x, const SampledSorghumParams& params) {
  float stage1_span = params.leaf_blade_stage1_length_ratio;
  float stage2_span = params.leaf_blade_stage2_length_ratio;
  float stage3_span = params.leaf_blade_stage3_length_ratio;
  NormalizeBladeStageRatios(stage1_span, stage2_span, stage3_span);

  const float base_mid_boundary = stage1_span;
  const float mid_tip_boundary = stage1_span + stage2_span;
  const float blend12 = std::max(0.02f, 0.2f * std::min(stage1_span, stage2_span));
  const float blend23 = std::max(0.02f, 0.2f * std::min(stage2_span, stage3_span));

  const float stage1_width = std::max(0.05f, params.leaf_blade_stage1_width_scale);
  const float stage2_width = std::max(0.05f, params.leaf_blade_stage2_width_scale);
  const float stage3_width = std::max(0.05f, params.leaf_blade_stage3_width_scale);

  const float clamped_x = std::clamp(x, 0.0f, 1.0f);
  const float t12 = glm::smoothstep(base_mid_boundary - blend12, base_mid_boundary + blend12, clamped_x);
  const float t23 = glm::smoothstep(mid_tip_boundary - blend23, mid_tip_boundary + blend23, clamped_x);

  const float stage12 = glm::mix(stage1_width, stage2_width, t12);
  return glm::mix(stage12, stage3_width, t23);
}

float EvaluateStageWidthRatio(const evo_engine::PlottedDistribution<float>& profile, const float u,
                              const float end_ratio, const float node_random, const uint32_t salt) {
  const float clamped_u = std::clamp(u, 0.0f, 1.0f);
  if (clamped_u <= 0.0f) {
    return 1.0f;
  }
  if (clamped_u >= 1.0f) {
    return std::max(0.01f, end_ratio);
  }

  const float p0 = EvaluatePlottedDeterministic(profile, 0.0f, node_random, salt, -std::numeric_limits<float>::max(),
                                                std::numeric_limits<float>::max());
  const float p1 = EvaluatePlottedDeterministic(profile, 1.0f, node_random, salt, -std::numeric_limits<float>::max(),
                                                std::numeric_limits<float>::max());
  const float pu = EvaluatePlottedDeterministic(profile, clamped_u, node_random, salt,
                                                -std::numeric_limits<float>::max(), std::numeric_limits<float>::max());

  const float denominator = p1 - p0;
  if (std::abs(denominator) <= 1.0e-6f) {
    return std::max(0.01f, glm::mix(1.0f, end_ratio, clamped_u));
  }

  const float a = (end_ratio - 1.0f) / denominator;
  const float b = 1.0f - a * p0;
  return std::max(0.01f, a * pu + b);
}

float EvaluateNormalizedWidthProfile(const evo_engine::PlottedDistribution<float>& profile, const float u,
                                     const float node_random, const uint32_t salt) {
  float maximum = 0.0f;
  for (int i = 0; i <= 32; ++i) {
    maximum = std::max(maximum, EvaluatePlottedDeterministic(profile, static_cast<float>(i) / 32.0f, node_random, salt,
                                                             0.0f, std::numeric_limits<float>::infinity()));
  }
  if (maximum <= 1.0e-6f)
    return 0.0f;
  return std::clamp(EvaluatePlottedDeterministic(profile, std::clamp(u, 0.0f, 1.0f), node_random, salt, 0.0f,
                                                 std::numeric_limits<float>::infinity()) /
                        maximum,
                    0.0f, 1.0f);
}

float ResolveVerticalSubdivision(const SorghumLeafMeshSettings& settings) {
  return std::max(0.0001f, settings.vertical_subdivision_length);
}

float ComputePolylineLength(const std::vector<StemContext::Segment>& segments) {
  if (segments.size() < 2)
    return 0.0f;
  float length = 0.0f;
  for (size_t i = 1; i < segments.size(); ++i) {
    length += glm::distance(segments[i - 1].position, segments[i].position);
  }
  return std::max(0.0f, length);
}

SorghumSplineSegment ToSplineSegment(const StemContext::Segment& s) {
  return SorghumSplineSegment(s.position, s.up, s.front, s.radius, s.theta, s.left_height_offset,
                              s.right_height_offset);
}

SorghumSplineSegment SampleStemAtNormalizedArc(const StemContext& stem_ctx, const float s_norm) {
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

float ComputeLeafTropismBendDeg(const SorghumLeaf& leaf, const SampledSorghumParams& params,
                                const glm::vec3& current_direction, const glm::vec3& bend_axis) {
  if (params.tropisms.empty()) {
    return 0.0f;
  }

  const float order_t = std::clamp(static_cast<float>(leaf.order), 0.0f, 1.0f);
  float accumulated_bend_deg = 0.0f;

  for (size_t i = 0; i < params.tropisms.size(); ++i) {
    const auto& tropism = params.tropisms[i];
    if (!std::isfinite(tropism.strength) || std::abs(tropism.strength) <= 1.0e-6f) {
      continue;
    }

    const glm::vec3 target_direction = SafeNormalize(tropism.direction, glm::vec3(0.0f, -1.0f, 0.0f));

    const float order_response =
        EvaluatePlottedDeterministic(tropism.order_response, order_t, leaf.node_random,
                                     kLeafMeshTropismOrderSalt + static_cast<uint32_t>(i) * 0x9E3779B9u, -2.0f, 2.0f);
    if (std::abs(order_response) <= 1.0e-6f) {
      continue;
    }

    const float sin_term = glm::dot(glm::cross(current_direction, target_direction), bend_axis);
    const float cos_term = std::clamp(glm::dot(current_direction, target_direction), -1.0f, 1.0f);
    const float signed_turn_deg = glm::degrees(std::atan2(sin_term, cos_term));

    const float strength_alpha = std::clamp(std::abs(tropism.strength) / 5.0f, 0.0f, 1.0f);
    const float strength_sign = (tropism.strength >= 0.0f) ? 1.0f : -1.0f;
    accumulated_bend_deg += signed_turn_deg * strength_alpha * strength_sign * order_response * 0.35f;
  }

  return std::clamp(accumulated_bend_deg, -45.0f, 45.0f);
}

glm::vec3 HashLeafBaseColor(const uint32_t id) {
  const float hue = static_cast<float>((id * 2654435761u) & 1023u) / 1024.0f;
  const float s = 0.72f;
  const float v = 0.92f;
  const float h6 = hue * 6.0f;
  const int sector = static_cast<int>(h6);
  const float f = h6 - static_cast<float>(sector);
  const float p = v * (1.0f - s);
  const float q = v * (1.0f - s * f);
  const float t = v * (1.0f - s * (1.0f - f));

  switch (sector % 6) {
    case 0:
      return glm::vec3(v, t, p);
    case 1:
      return glm::vec3(q, v, p);
    case 2:
      return glm::vec3(p, v, t);
    case 3:
      return glm::vec3(p, q, v);
    case 4:
      return glm::vec3(t, p, v);
    default:
      return glm::vec3(v, p, q);
  }
}

int ComputeLeafStageId(const float arc_length, const float sheath_length, const float neck_length) {
  if (arc_length <= sheath_length) {
    return 0;
  }
  if (arc_length <= sheath_length + neck_length) {
    return 1;
  }
  return 2;
}

float ComputeLeafStageShade(const int stage_id) {
  if (stage_id == 0) {
    return kLeafSheathShade;
  }
  if (stage_id == 1) {
    return kLeafNeckShade;
  }
  return kLeafBladeShade;
}

}  // namespace

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
  if (segments.empty())
    return;
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
  if (segment_index >= segments.size() - 1)
    return 0.0f;
  glm::vec3 p[4];
  GetPositionControlPoints(segment_index, p[0], p[1], p[2], p[3]);
  return evo_engine::Strands::CalculateLengthAdaptive(p[0], p[1], p[2], p[3], t_start, t_end, tolerance);
}

float SorghumSpline::GetArcLength(const float tolerance) const {
  if (segments.size() < 2)
    return 0.0f;
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
  if (segments.empty() || segment_count == 0)
    return out;

  if (segments.size() == 1) {
    out.assign(segment_count, segments.front());
    return out;
  }
  if (segment_count == 1) {
    out.emplace_back(segments.back());
    return out;
  }

  const float safe_tolerance = std::max(1.0e-6f, tolerance);
  const uint32_t span_count = static_cast<uint32_t>(segments.size() - 1);

  std::vector<float> span_lengths(span_count, 0.0f);
  float total_arc_length = 0.0f;
  for (uint32_t span_index = 0; span_index < span_count; ++span_index) {
    float span_length = GetSegmentArcLength(span_index, 0.0f, 1.0f, safe_tolerance);
    if (!std::isfinite(span_length)) {
      span_length = 0.0f;
    }
    span_length = std::max(0.0f, span_length);
    span_lengths[span_index] = span_length;
    total_arc_length += span_length;
  }

  if (total_arc_length <= safe_tolerance) {
    out.resize(segment_count, segments.front());
    out.back() = segments.back();
    return out;
  }

  out.reserve(segment_count);
  out.emplace_back(segments.front());

  uint32_t current_span = 0;
  float consumed_arc_length = 0.0f;
  float current_span_length = span_lengths[current_span];

  for (size_t sample_index = 1; sample_index + 1 < segment_count; ++sample_index) {
    const float alpha = static_cast<float>(sample_index) / static_cast<float>(segment_count - 1);
    const float target_arc_length = total_arc_length * alpha;

    while (current_span + 1 < span_count &&
           consumed_arc_length + current_span_length < target_arc_length - safe_tolerance) {
      consumed_arc_length += current_span_length;
      ++current_span;
      current_span_length = span_lengths[current_span];
    }

    if (current_span_length <= safe_tolerance) {
      out.emplace_back(segments[current_span + 1]);
      continue;
    }

    const float local_target_arc = std::clamp(target_arc_length - consumed_arc_length, 0.0f, current_span_length);

    glm::vec3 p[4];
    GetPositionControlPoints(current_span, p[0], p[1], p[2], p[3]);
    float t = evo_engine::Strands::FindTAdaptive(p[0], p[1], p[2], p[3], 0.0f, local_target_arc, safe_tolerance);
    t = std::clamp(t, 0.0f, 1.0f);
    out.emplace_back(InterpolateSegment(current_span, t));
  }

  out.emplace_back(segments.back());

  if (out.size() != segment_count) {
    out.resize(segment_count, segments.back());
    out.front() = segments.front();
    out.back() = segments.back();
  }
  return out;
}

std::vector<SorghumSplineSegment> SorghumSpline::RebuildFixedLengthSegments(const float segment_length,
                                                                            const float tolerance) const {
  std::vector<SorghumSplineSegment> out;
  if (segments.size() < 2)
    return out;

  float t = 0.0f;
  float remaining_length = std::max(0.0001f, segment_length);

  for (uint32_t segment_index = 0; segment_index + 1 < segments.size(); segment_index++) {
    glm::vec3 p[4];
    GetPositionControlPoints(segment_index, p[0], p[1], p[2], p[3]);
    while (true) {
      const float t_next = evo_engine::Strands::FindTAdaptive(p[0], p[1], p[2], p[3], t, remaining_length, tolerance);
      if (t_next != 1.0f) {
        t = t_next;
        remaining_length = segment_length;
        out.emplace_back(InterpolateSegment(segment_index, t));
      } else {
        remaining_length -= evo_engine::Strands::CalculateLengthAdaptive(p[0], p[1], p[2], p[3], t, 1.0f, tolerance);
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
    if (seg.theta >= 90.0f)
      break;
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

void l_system_package::BuildLeafSplineFromState(const SorghumLeaf& leaf, const StemContext& stem_ctx,
                                                const SampledSorghumParams& params,
                                                const SorghumLeafMeshSettings& settings, SorghumSpline& out_spline) {
  out_spline.segments.clear();
  if (!leaf.alive)
    return;
  if (stem_ctx.segments.empty())
    return;

  const float vertical_step = ResolveVerticalSubdivision(settings);
  const SorghumSplineSegment anchor = SampleStemAtNormalizedArc(stem_ctx, leaf.s_along_parent_norm);
  const glm::vec3 stem_base = stem_ctx.segments.front().position;

  float stem_length = ComputePolylineLength(stem_ctx.segments);
  if (stem_length <= 1.0e-6f) {
    stem_length =
        std::max(vertical_step, glm::distance(stem_ctx.segments.front().position, stem_ctx.segments.back().position));
  }
  stem_length = std::max(stem_length, vertical_step);

  glm::vec3 stem_direction = SafeNormalize(
      anchor.front, SafeNormalize(stem_ctx.segments.back().position - stem_base, glm::vec3(0.0f, 1.0f, 0.0f)));
  glm::vec3 stem_up = SafeNormalize(anchor.up, glm::vec3(0.0f, 0.0f, 1.0f));
  glm::vec3 base_left = SafeNormalize(glm::cross(stem_up, stem_direction), glm::vec3(1.0f, 0.0f, 0.0f));

  glm::vec3 leaf_left =
      SafeNormalize(glm::rotate(base_left, glm::radians(leaf.roll_angle_deg), stem_direction), base_left);
  glm::vec3 leaf_up = SafeNormalize(glm::cross(stem_direction, leaf_left), stem_up);

  glm::vec3 direction =
      SafeNormalize(glm::rotate(stem_direction, glm::radians(leaf.insertion_angle_deg), leaf_left), stem_direction);

  const float starting_point = std::clamp(leaf.s_along_parent_norm, 0.0f, 1.0f);
  const float stem_width = std::max(0.0005f, anchor.radius);
  const float stem_anchor_distance = starting_point * stem_length;
  const float clamped_sheath_length = std::clamp(std::max(0.0f, leaf.sheath_length), 0.0f, stem_anchor_distance);
  const float sheath_start_ratio =
      (stem_anchor_distance > 1.0e-6f)
          ? std::clamp((stem_anchor_distance - clamped_sheath_length) / stem_length, 0.0f, starting_point)
          : starting_point;
  const float sheath_span_ratio = std::max(0.0f, starting_point - sheath_start_ratio);

  const float neck_length = std::max(0.0f, leaf.neck_length);
  const float blade_length = std::max(vertical_step, leaf.blade_length);
  const float target_neck_length = std::max(neck_length, leaf.target_neck_length);
  const float target_blade_length = std::max(blade_length, leaf.target_blade_length);
  const float total_distal_length = std::max(vertical_step, neck_length + blade_length);
  const float target_distal_length = std::max(vertical_step, target_neck_length + target_blade_length);

  const float sheath_radius_ratio = std::max(1.0f, params.leaf_sheath_radius_ratio);
  const float sheath_start_width = stem_width * sheath_radius_ratio;
  const float blade_max_half_width = std::max(0.00025f, leaf.blade_max_width * 0.5f);
  const float blade_start_width = std::max(
      0.00025f, blade_max_half_width * EvaluateNormalizedWidthProfile(params.width_along_leaf, 0.0f, leaf.node_random,
                                                                      kLeafMeshWidthProfileSalt));
  const float neck_start_width = sheath_start_width;
  const float neck_end_width = blade_start_width;
  const glm::vec3 stem_offset = sheath_start_width * -leaf_up;

  if (sheath_span_ratio > 0.0f) {
    const int sheath_node_count =
        std::max(2, static_cast<int>(std::ceil(stem_length * sheath_span_ratio / vertical_step)) + 1);
    for (int i = 0; i < sheath_node_count; i++) {
      const float stage_u = static_cast<float>(i) / static_cast<float>(std::max(1, sheath_node_count - 1));
      const float current_root_to_sheath_point = glm::mix(sheath_start_ratio, starting_point, stage_u);
      const auto local_stem = SampleStemAtNormalizedArc(stem_ctx, current_root_to_sheath_point);
      const glm::vec3 local_front = SafeNormalize(local_stem.front, stem_direction);
      const glm::vec3 local_base_left = SafeNormalize(glm::cross(local_stem.up, local_front), base_left);
      const glm::vec3 local_left =
          SafeNormalize(glm::rotate(local_base_left, glm::radians(leaf.roll_angle_deg), local_front), leaf_left);
      const glm::vec3 local_up = SafeNormalize(glm::cross(local_front, local_left), leaf_up);
      const float sheath_width = std::max(local_stem.radius, local_stem.radius * sheath_radius_ratio);
      out_spline.segments.emplace_back(local_stem.position - sheath_width * local_up, local_up, local_front,
                                       sheath_width, std::clamp(params.leaf_sheath_wrap_angle * 0.5f, 90.0f, 270.0f),
                                       0.0f, 0.0f);
    }
  }

  const int distal_node_count = std::max(2, static_cast<int>(std::ceil(target_distal_length / vertical_step)));

  glm::vec3 node_position = anchor.position + stem_offset;
  float previous_travel_distance = 0.0f;
  const int distal_start_index = out_spline.segments.empty() ? 0 : 1;
  const float safe_total_distal_length = std::max(1.0e-6f, total_distal_length);
  const auto emit_distal_sample = [&](const float factor, const float travel_distance, const float step_length) {
    const float clamped_factor = std::clamp(factor, 0.0f, 1.0f);

    const float bend_profile = EvaluatePlottedDeterministic(params.bending_along_leaf, clamped_factor, leaf.node_random,
                                                            kLeafMeshBendingSalt, 0.0f, 1.0f);
    const float senescence_bend_bias_deg = 18.0f * std::clamp(leaf.senescence_phase, 0.0f, 1.0f) * clamped_factor;
    float rotate_angle = leaf.bending * bend_profile + senescence_bend_bias_deg;
    // Tropism bend is evaluated from the provisional direction so global droop entries can influence leaves.
    const glm::vec3 provisional_direction =
        SafeNormalize(glm::rotate(direction, glm::radians(rotate_angle), leaf_left), direction);
    const float tropism_bend_deg = ComputeLeafTropismBendDeg(leaf, params, provisional_direction, leaf_left);
    rotate_angle += tropism_bend_deg * clamped_factor;
    const glm::vec3 current_direction =
        SafeNormalize(glm::rotate(direction, glm::radians(rotate_angle), leaf_left), direction);
    node_position += current_direction * std::max(0.0f, step_length);

    const float expand_profile = EvaluatePlottedDeterministic(
        params.curling_along_leaf, clamped_factor, leaf.node_random, kLeafMeshCurlingProfileSalt, 0.0f, 1.0f);
    const float opening_angle_deg = ResolveBladeOpeningAngleDeg(leaf.curling);
    const float profiled_opening_angle_deg =
        std::clamp(opening_angle_deg * expand_profile, kMinBladeOpeningAngleDeg, kMaxBladeOpeningAngleDeg);

    const float collar_factor = (neck_length > 1.0e-6f) ? std::clamp(travel_distance / neck_length, 0.0f, 1.0f) : 1.0f;

    const bool within_neck_stage = neck_length > 1.0e-6f && travel_distance <= neck_length + 1.0e-6f;
    float stage_u = 1.0f;

    if (within_neck_stage) {
      stage_u = std::clamp(travel_distance / std::max(1.0e-6f, neck_length), 0.0f, 1.0f);
    } else {
      const float blade_distance =
          (neck_length > 1.0e-6f) ? std::max(0.0f, travel_distance - neck_length) : travel_distance;
      stage_u = std::clamp(blade_distance / std::max(vertical_step, blade_length), 0.0f, 1.0f);
    }

    const float waviness_profile =
        within_neck_stage
            ? 0.0f
            : EvaluatePlottedDeterministic(params.waviness_along_leaf, stage_u, leaf.node_random,
                                           kLeafMeshWavinessProfileSalt, 0.0f, std::numeric_limits<float>::infinity());
    const float waviness = leaf.waviness * waviness_profile;
    const float waviness_frequency = std::max(0.0f, leaf.waviness_frequency);
    const float phase = leaf.node_random * glm::two_pi<float>();
    const float wave_phase = glm::two_pi<float>() * waviness_frequency * stage_u + phase;

    const float width =
        within_neck_stage
            ? std::max(0.00025f,
                       neck_start_width * EvaluateStageWidthRatio(params.width_along_neck, stage_u,
                                                                  neck_end_width / std::max(0.00025f, neck_start_width),
                                                                  leaf.node_random, kLeafMeshNeckWidthProfileSalt))
            : std::max(0.00025f, blade_max_half_width * EvaluateNormalizedWidthProfile(params.width_along_leaf, stage_u,
                                                                                       leaf.node_random,
                                                                                       kLeafMeshWidthProfileSalt));
    const float angle = 90.0f - (90.0f - profiled_opening_angle_deg) * glm::pow(collar_factor, 2.0f);

    const glm::vec3 up = SafeNormalize(glm::cross(current_direction, leaf_left), leaf_up);
    out_spline.segments.emplace_back(node_position, up, current_direction, width, angle,
                                     waviness * std::sin(wave_phase), waviness * std::sin(wave_phase + 0.65f));
  };

  for (int i = distal_start_index; i <= distal_node_count; i++) {
    const float factor = static_cast<float>(i) / static_cast<float>(std::max(1, distal_node_count));
    const float travel_distance = factor * total_distal_length;
    const float previous_distance = previous_travel_distance;

    // Emit an explicit neck->blade boundary sample so junction width matches
    // neck_end_width_ratio even when discretization skips exactly at neck_length.
    const bool crosses_neck_boundary =
        neck_length > 1.0e-6f && previous_distance + 1.0e-6f < neck_length && travel_distance > neck_length + 1.0e-6f;
    if (crosses_neck_boundary) {
      const float neck_boundary_factor = std::clamp(neck_length / safe_total_distal_length, 0.0f, 1.0f);
      const float neck_boundary_step = std::max(0.0f, neck_length - previous_travel_distance);
      emit_distal_sample(neck_boundary_factor, neck_length, neck_boundary_step);
      previous_travel_distance = neck_length;
    }

    const float step_length = std::max(0.0f, travel_distance - previous_travel_distance);
    emit_distal_sample(factor, travel_distance, step_length);
    previous_travel_distance = travel_distance;
  }
}

void l_system_package::GenerateBladeGeometry(const SorghumSpline& spline, const SorghumLeaf& leaf,
                                             const SampledSorghumParams& /*params*/,
                                             const SorghumLeafMeshSettings& settings,
                                             std::vector<evo_engine::Vertex>& vertices,
                                             std::vector<glm::uvec3>& triangles, const bool current_bottom_face,
                                             const uint32_t leaf_index,
                                             const SorghumLeafAtlasLayout& raw_atlas_layout) {
  if (!leaf.alive)
    return;
  if (spline.segments.empty())
    return;
  const auto atlas_layout = NormalizeSorghumLeafAtlasLayout(raw_atlas_layout);

  const float vertical_step = ResolveVerticalSubdivision(settings);
  const float target_total_length = std::max(
      vertical_step, leaf.target_blade_length + (settings.enable_leaf_sheath ? leaf.target_sheath_length : 0.0f));
  const size_t stable_segment_count =
      static_cast<size_t>(std::max(4, static_cast<int>(std::ceil(target_total_length / vertical_step))));

  SorghumSpline temp_spline;
  temp_spline.segments = spline.RebuildFixedSizeSegments(stable_segment_count);

  const auto& full_segments = temp_spline.segments;
  if (full_segments.empty())
    return;

  size_t visible_start_index = 0;
  if (!settings.enable_leaf_sheath) {
    while (visible_start_index < full_segments.size() && full_segments[visible_start_index].theta > 90.0f) {
      visible_start_index++;
    }
  }
  if (visible_start_index >= full_segments.size())
    return;

  std::vector<SorghumSplineSegment> segments(full_segments.begin() + static_cast<std::ptrdiff_t>(visible_start_index),
                                             full_segments.end());
  if (segments.empty())
    return;

  evo_engine::Vertex archetype{};

  const float senescence = std::clamp(leaf.senescence_phase, 0.0f, 1.0f);
  const glm::vec3 wilt_brown = glm::vec3(0.56f, 0.40f, 0.17f);
  const glm::vec3 node_color = HashLeafBaseColor(leaf_index + 1u);
  const glm::vec3 senescence_tinted_node_color = glm::mix(node_color, wilt_brown, senescence * 0.35f);
  archetype.vertex_info1 = glm::uintBitsToFloat(leaf_index + 1u);

  const int horizontal_step = std::max(2, settings.horizontal_subdivision_step);
  const int verts_count = horizontal_step * 2 + 1;
  const float x_step = 1.0f / static_cast<float>(horizontal_step) / 2.0f;
  const int segment_size = static_cast<int>(segments.size());

  std::vector<float> full_row_arcs(full_segments.size(), 0.0f);
  for (size_t i = 1; i < full_segments.size(); i++) {
    full_row_arcs[i] = full_row_arcs[i - 1] + glm::distance(full_segments[i - 1].position, full_segments[i].position);
  }

  const float full_total_arc = full_row_arcs.back();
  size_t first_distal_index = 0;
  while (first_distal_index < full_segments.size() && full_segments[first_distal_index].theta > 90.0f) {
    first_distal_index++;
  }

  const float effective_sheath_length = first_distal_index > 0 ? full_row_arcs[first_distal_index - 1] : 0.0f;
  const float effective_distal_length = std::max(0.0f, full_total_arc - effective_sheath_length);

  const float neck_length = std::max(0.0f, leaf.neck_length);
  const float blade_length = std::max(0.0f, leaf.blade_length);
  const float distal_stage_total = std::max(1.0e-6f, neck_length + blade_length);
  const float neck_ratio = std::clamp(neck_length / distal_stage_total, 0.0f, 1.0f);
  const float effective_neck_length = effective_distal_length * neck_ratio;
  const float effective_blade_length = std::max(1.0e-6f, effective_distal_length - effective_neck_length);

  std::vector<float> local_row_arcs(segment_size, 0.0f);
  for (int i = 1; i < segment_size; i++) {
    local_row_arcs[i] = local_row_arcs[i - 1] + glm::distance(segments[i - 1].position, segments[i].position);
  }
  const float visible_arc_offset = full_row_arcs[visible_start_index];

  const auto emit_segment_strip = [&](const unsigned int row_a_start, const unsigned int row_b_start) {
    const auto emit_triangle = [&](const unsigned int i0, const unsigned int i1, const unsigned int i2) {
      const glm::vec3 face_normal =
          glm::cross(vertices[i1].position - vertices[i0].position, vertices[i2].position - vertices[i0].position);
      const glm::vec3 vertex_normal = vertices[i0].normal + vertices[i1].normal + vertices[i2].normal;
      if (glm::dot(face_normal, vertex_normal) < 0.0f) {
        triangles.emplace_back(i0, i2, i1);
      } else {
        triangles.emplace_back(i0, i1, i2);
      }
    };

    for (int j = 0; j < verts_count - 1; j++) {
      if (current_bottom_face) {
        emit_triangle(row_a_start + j, row_a_start + j + 1, row_b_start + j);
        emit_triangle(row_b_start + j + 1, row_b_start + j, row_a_start + j + 1);
      } else {
        emit_triangle(row_b_start + j, row_a_start + j + 1, row_a_start + j);
        emit_triangle(row_a_start + j + 1, row_b_start + j, row_b_start + j + 1);
      }
    }
  };

  const auto emit_segment_row = [&](const SorghumSplineSegment& segment, const int row_index,
                                    const glm::vec4& row_color, const float row_v) -> unsigned int {
    const unsigned int row_start = static_cast<unsigned int>(vertices.size());
    const float angle_step = segment.theta / static_cast<float>(horizontal_step);
    const float absolute_arc = visible_arc_offset + local_row_arcs[static_cast<size_t>(row_index)];
    float region_thickness = leaf.blade_thickness;
    if (absolute_arc <= effective_sheath_length) {
      region_thickness = leaf.sheath_thickness;
    } else if (absolute_arc < effective_sheath_length + effective_neck_length && effective_neck_length > 1.0e-6f) {
      const float neck_u = std::clamp((absolute_arc - effective_sheath_length) / effective_neck_length, 0.0f, 1.0f);
      region_thickness =
          glm::mix(leaf.sheath_thickness, leaf.blade_thickness, neck_u * neck_u * (3.0f - 2.0f * neck_u));
    }
    if (region_thickness <= 0.0f)
      region_thickness = settings.leaf_thickness;
    archetype.color = row_color;
    for (int j = 0; j < verts_count; j++) {
      const float angle = (static_cast<float>(j) - static_cast<float>(horizontal_step)) * angle_step;
      glm::vec3 position = segment.GetLeafPoint(angle);
      glm::vec3 normal = segment.GetNormal(angle);
      if (row_index != 0 && j != 0 && j != verts_count - 1) {
        position += normal * region_thickness * (current_bottom_face ? -0.5f : 0.5f);
      }

      archetype.position = position;
      archetype.normal = current_bottom_face ? -normal : normal;
      archetype.tangent = SafeNormalize(glm::cross(normal, segment.front), glm::vec3(1.0f, 0.0f, 0.0f));
      float local_u = j * x_step;
      if (atlas_layout.semantic_quadrants) {
        const bool right_quadrant = current_bottom_face;
        local_u = (right_quadrant ? 0.5f : 0.0f) + local_u * 0.5f;
      }
      archetype.tex_coord = RemapSorghumLeafAtlasUv(glm::vec2(local_u, row_v), atlas_layout);
      vertices.push_back(archetype);
    }
    return row_start;
  };

  const auto compute_atlas_v = [&](const float global_segment_arc, const int stage_id) {
    if (atlas_layout.semantic_quadrants) {
      if (stage_id < 2) {
        const float proximal_length = std::max(1.0e-6f, effective_sheath_length + effective_neck_length);
        const float t = std::clamp(global_segment_arc / proximal_length, 0.0f, 1.0f);
        return 0.01f + 0.48f * t;
      }
      const float t = std::clamp(
          (global_segment_arc - effective_sheath_length - effective_neck_length) / effective_blade_length, 0.0f, 1.0f);
      return 0.51f + 0.48f * t;
    }
    if (atlas_layout.distal_region_uses_top_half) {
      if (stage_id == 0) {
        const float t = effective_sheath_length > 1.0e-6f
                            ? std::clamp(global_segment_arc / effective_sheath_length, 0.0f, 1.0f)
                            : 0.0f;
        return 0.02f + 0.46f * t;
      }
      const float t =
          effective_distal_length > 1.0e-6f
              ? std::clamp((global_segment_arc - effective_sheath_length) / effective_distal_length, 0.0f, 1.0f)
              : 0.0f;
      return 0.5f + 0.49f * t;
    }
    if (stage_id == 0) {
      const float t = effective_sheath_length > 1.0e-6f
                          ? std::clamp(global_segment_arc / effective_sheath_length, 0.0f, 1.0f)
                          : 0.0f;
      return 0.02f + 0.21f * t;
    }
    if (stage_id == 1) {
      const float t =
          effective_neck_length > 1.0e-6f
              ? std::clamp((global_segment_arc - effective_sheath_length) / effective_neck_length, 0.0f, 1.0f)
              : 0.0f;
      return 0.27f + 0.21f * t;
    }
    const float t = std::clamp(
        (global_segment_arc - effective_sheath_length - effective_neck_length) / effective_blade_length, 0.0f, 1.0f);
    return 0.5f + 0.49f * t;
  };

  unsigned int previous_row_start = 0u;
  int previous_stage_id = 0;

  for (int i = 0; i < segment_size; i++) {
    auto& segment = segments[i];

    const float global_segment_arc = visible_arc_offset + local_row_arcs[i];
    const int stage_id = ComputeLeafStageId(global_segment_arc, effective_sheath_length, effective_neck_length);
    const glm::vec4 current_stage_color =
        glm::vec4(senescence_tinted_node_color * ComputeLeafStageShade(stage_id), 1.0f);
    const float current_atlas_v = compute_atlas_v(global_segment_arc, stage_id);

    if (i == 0) {
      previous_row_start = emit_segment_row(segment, i, current_stage_color, current_atlas_v);
      previous_stage_id = stage_id;
      continue;
    }

    if (stage_id != previous_stage_id) {
      const glm::vec4 previous_stage_color =
          glm::vec4(senescence_tinted_node_color * ComputeLeafStageShade(previous_stage_id), 1.0f);
      const float previous_atlas_v = compute_atlas_v(global_segment_arc, previous_stage_id);
      const unsigned int boundary_previous_color_row =
          emit_segment_row(segment, i, previous_stage_color, previous_atlas_v);
      emit_segment_strip(previous_row_start, boundary_previous_color_row);
      previous_row_start = emit_segment_row(segment, i, current_stage_color, current_atlas_v);
      previous_stage_id = stage_id;
      continue;
    }

    const unsigned int current_row_start = emit_segment_row(segment, i, current_stage_color, current_atlas_v);
    emit_segment_strip(previous_row_start, current_row_start);
    previous_row_start = current_row_start;
  }
}
