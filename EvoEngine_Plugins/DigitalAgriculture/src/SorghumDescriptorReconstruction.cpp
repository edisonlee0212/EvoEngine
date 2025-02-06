
// @edisonlee0212: here are the implementations.

#include "SorghumDescriptorReconstruction.hpp"

#include "Sorghum.hpp"
#include "SorghumLayer.hpp"

using namespace digital_agriculture_plugin;

glm::vec3 CubicBezierSpline::Interpolation(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2,
                                           const glm::vec3& v3, const float t) {
  const glm::vec3 b = (v1 - v0) * 3.0f;
  const glm::vec3 c = (v2 - v1) * 3.0f - b;
  const glm::vec3 d = (v3 - v0) - b - c;
  return v0 + b * t + c * t * t + d * t * t * t;
}

glm::vec3 CubicBezierSpline::GetTangent(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2,
                                        const glm::vec3& v3, const float t) {
  const glm::vec3 b = (v1 - v0) * 3.0f;
  const glm::vec3 c = (v2 - v1) * 3.0f - b;
  const glm::vec3 d = (v3 - v0) - b - c;

  const glm::vec3 tangent = b + 2.0f * c * t + 3.0f * d * t * t;

  return normalize(tangent);
}

float CubicBezierSpline::CalculateLengthAdaptive(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2,
                                                 const glm::vec3& v3, const float t_start, const float t_end,
                                                 const float tolerance) {
  const glm::vec3 mid_point = Interpolation(v0, v1, v2, v3, (t_start + t_end) * 0.5f);
  const glm::vec3 start_point = Interpolation(v0, v1, v2, v3, t_start);
  const glm::vec3 end_point = Interpolation(v0, v1, v2, v3, t_end);

  const float linear_distance = glm::distance(start_point, end_point);
  if (const float curve_distance = glm::distance(start_point, mid_point) + glm::distance(mid_point, end_point);
      fabs(linear_distance - curve_distance) < tolerance) {
    return curve_distance;  // Close enough, return this estimate
  }
  // Subdivide further
  return CalculateLengthAdaptive(v0, v1, v2, v3, t_start, (t_start + t_end) * 0.5f, tolerance) +
         CalculateLengthAdaptive(v0, v1, v2, v3, (t_start + t_end) * 0.5f, t_end, tolerance);
}

float CubicBezierSpline::FindTAdaptive(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2,
                                       const glm::vec3& v3, const float t_start, const float target_length,
                                       const float tolerance) {
  if (CalculateLengthAdaptive(v0, v1, v2, v3, t_start, 1.f, tolerance) <= target_length)
    return 1.f;
  float t_low = t_start, t_high = 1.0f;
  while (t_high - t_low > tolerance) {
    if (const float t_mid = (t_low + t_high) * 0.5f;
        CalculateLengthAdaptive(v0, v1, v2, v3, t_start, t_mid) < target_length) {
      t_low = t_mid;
    } else {
      t_high = t_mid;
    }
  }
  return (t_low + t_high) * 0.5f;
}

float CubicBezierSpline::GetLength() {
  float result = 0;
  segment_lengths_.clear();
  for (int i = 0; i < joints.size() - 1; i++) {
    float segment_length = CalculateLengthAdaptive(joints[i].position, joints[i].right_handle,
                                                   joints[i + 1].left_handle, joints[i + 1].position);
    result += segment_length;
    segment_lengths_.emplace_back(segment_length);
  }
  return result;
}

std::vector<CubicSplineSample> CubicBezierSpline::GetSamplesByLength(const float distance) {
  const float length = GetLength();
  const int sample_num = length / distance;
  return GetUniformSamples(sample_num);
}

std::vector<CubicSplineSample> CubicBezierSpline::GetUniformSamples(int num) {
  std::vector<CubicSplineSample> samples;
  float length = GetLength();
  float length_per_sample = length / (num + 1);
  float sample_length = length_per_sample;
  float current_length = 0;
  std::cout << "segment count in getUniform: " << segment_lengths_.size() << "\n";
  for (int i = 0; i < segment_lengths_.size(); i++) {
    while (current_length < sample_length) {
      current_length += segment_lengths_[i++];
    }
    i--;

    float t = FindTAdaptive(joints[i].position, joints[i].right_handle, joints[i + 1].left_handle,
                            joints[i + 1].position, 0, sample_length - current_length + segment_lengths_[i], 0.00001f);
    glm::vec3 pos = (Interpolation(joints[i].position, joints[i].right_handle, joints[i + 1].left_handle,
                                   joints[i + 1].position, t));

    CubicSplineSample s;
    s.position = pos;
    s.segment_index = i;
    s.t = t;
    samples.emplace_back(s);

    if (samples.size() == num) {
      break;
    }
    sample_length += length_per_sample;
  }
  if (samples.size() != num) {
    EVOENGINE_ERROR("invalid samples count in getUniformSamples")
  }
  return samples;
}

std::vector<glm::vec3> CubicBezierSpline::GetSurfaceIntersection(const glm::vec3& plane_point,
                                                                 const glm::vec3& normal) const {
  std::vector<glm::vec3> results;
  for (int i = 0; i < joints.size() - 1; i++) {
    glm::vec3 p0 = joints[i].position;
    glm::vec3 p1 = joints[i].right_handle;
    glm::vec3 p2 = joints[i + 1].left_handle;
    glm::vec3 p3 = joints[i + 1].position;

    // assume: that each of the bezier curves should only intersect with the plane once

    // check whether the start point and the end point are on different sides of the plane
    if (glm::dot(p0 - plane_point, normal) * glm::dot(p3 - plane_point, normal) > 0) {
      // std::cout << "skip, i = " << i << "\n";
      continue;
    }

    // if there is intersection, two-step search for t
    float t = BisectionMethod(0.0f, 1.0f, p0, p1, p2, p3, plane_point, normal);
    // std::cout << "after bisection, t = :" << t << "i = " <<i<< "\n";
    t = NewtonMethod(t, p0, p1, p2, p3, plane_point, normal);
    // std::cout << "after newton, t = :" << t << "i = " << i << "\n";
    if (t > 1 || t < 0) {
      continue;
    }
    glm::vec3 intersection_point = Interpolation(p0, p1, p2, p3, t);

    results.emplace_back(intersection_point);
  }
  // std::cout << "get intersection size:" << results.size() << "\n";
  return results;
}

float CubicBezierSpline::BisectionMethod(float t_min, float t_max, const glm::vec3& v0, const glm::vec3& v1,
                                         const glm::vec3& v2, const glm::vec3& v3, const glm::vec3& plane_point,
                                         const glm::vec3& normal, const float epsilon) {
  while (t_max - t_min > epsilon) {
    const float t_mid = (t_min + t_max) / 2.0f;
    glm::vec3 bezier_point = Interpolation(v0, v1, v2, v3, t_mid);
    if (const float value = PlaneEquation(bezier_point, normal, plane_point); value > 0) {
      t_max = t_mid;
    } else {
      t_min = t_mid;
    }
  }
  return (t_min + t_max) / 2.0f;
}

float CubicBezierSpline::NewtonMethod(float t, const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2,
                                      const glm::vec3& v3, const glm::vec3& plane_point, const glm::vec3& normal,
                                      const float epsilon, const int max_iteration) {
  for (int i = 0; i < max_iteration; ++i) {
    glm::vec3 bezier_point = Interpolation(v0, v1, v2, v3, t);
    const float value = PlaneEquation(bezier_point, normal, plane_point);
    if (std::abs(value) < epsilon) {
      return t;  // find intersection
    }

    // calculate the tangent
    glm::vec3 bezier_derivative = GetTangent(v0, v1, v2, v3, t);
    const float derivative = glm::dot(normal, bezier_derivative);  // calculate derivative
    if (std::abs(derivative) < epsilon) {
      break;
    }
    t -= value / derivative;  // update t
  }
  return t;
}

glm::vec3 CubicBezierSpline::GetTangent(const CubicSplineSample& sample) const {
  const auto& p0 = joints[sample.segment_index].position;
  const auto& p1 = joints[sample.segment_index].right_handle;
  const auto& p2 = joints[sample.segment_index + 1].left_handle;
  const auto& p3 = joints[sample.segment_index + 1].position;
  float t = sample.t;
  return GetTangent(p0, p1, p2, p3, t);
}

glm::vec3 CubicBezierSpline::SegmentInterpolation(const int segment_index, const float t) const {
  const auto& p0 = joints[segment_index].position;
  const auto& p1 = joints[segment_index].right_handle;
  const auto& p2 = joints[segment_index + 1].left_handle;
  const auto& p3 = joints[segment_index + 1].position;
  return Interpolation(p0, p1, p2, p3, t);
}

std::vector<glm::vec3> CubicBezierSpline::GetLineSamples(const int num_per_curve) const {
  std::vector<glm::vec3> results;

  for (int i = 0; i < joints.size() - 1; i++) {
    const auto& p0 = joints[i].position;
    const auto& p1 = joints[i].right_handle;
    const auto& p2 = joints[i + 1].left_handle;
    const auto& p3 = joints[i + 1].position;

    int num = 0;
    const float val = 1 / static_cast<float>(num_per_curve);
    while (num < num_per_curve) {
      const float t = num * val;
      results.push_back(Interpolation(p0, p1, p2, p3, t));
      num++;
    }
  }
  return results;
}

std::vector<std::unordered_map<std::string, CubicBezierSpline>>
SorghumDescriptorReconstruction::ReconstructBezierSplineFromYaml(
    std::vector<std::unordered_map<std::string, std::vector<glm::vec3>>>& yaml_content) {
  if (yaml_content.empty()) {
    EVOENGINE_ERROR("Empty yaml loaded")
  }
  std::vector<std::unordered_map<std::string, CubicBezierSpline>> bezierSplines;

  std::vector<std::string> keys = {"leftPoints", "rightPoints", "centerPoints"};

  // iteration for leaves
  for (int i = 0; i < yaml_content.size(); i++) {
    std::unordered_map<std::string, CubicBezierSpline> leafRepresentation;

    auto& leaf = yaml_content[i];

    // iteration for lines
    for (int j = 0; j < keys.size(); j++) {
      CubicBezierSpline bezier_spline;
      std::vector<glm::vec3> points = leaf[keys[j]];

      // iteration for points
      for (int k = 0; k < points.size(); k++) {
        // generate CubicBezierSpline from the points
        glm::vec3 front;
        if (k == points.size() - 1) {
          front = (points[k] - points[k - 1]);
        } else {
          front = (points[k + 1] - points[k]);
        }

        CubicBezierPoint p;
        p.position = points[k];
        // C1 continuity
        p.right_handle = p.position + front * 0.25f;
        p.left_handle = p.position - front * 0.25f;

        bezier_spline.joints.emplace_back(p);
      }

      leafRepresentation.insert(std::make_pair(keys[j], bezier_spline));
    }

    bezierSplines.emplace_back(leafRepresentation);
  }

  return bezierSplines;
}

std::vector<std::unordered_map<std::string, std::vector<glm::vec3>>>
SorghumDescriptorReconstruction::GetLineSamplesFromBezierSplines(
    const std::vector<std::unordered_map<std::string, CubicBezierSpline>>& bezier_splines, const int num_per_curve) {
  std::vector<std::unordered_map<std::string, std::vector<glm::vec3>>> results;

  const std::vector<std::string> keys = {"leftPoints", "rightPoints", "centerPoints"};

  for (auto leaf_splines : bezier_splines) {
    std::unordered_map<std::string, std::vector<glm::vec3>> point_set;

    for (const auto& key : keys) {
      CubicBezierSpline bezier_spline = leaf_splines[key];
      auto points = bezier_spline.GetLineSamples(num_per_curve);

      point_set.insert(std::make_pair(key, points));
    }

    results.emplace_back(point_set);
  }

  return results;
}

void SorghumDescriptorReconstruction::ExtendLeafToStem(SorghumLeafDescriptor& leaf) const {
  auto first_segment = leaf.spline.segments[0];

  auto right = normalize(glm::cross(first_segment.front, glm::vec3(0, 1, 0)));

  // construct the local frame
  SorghumSplineSegment segment;

  segment.front = normalize(glm::vec3(-0.04, 1, 0.01));

  segment.up = normalize(glm::cross(right, segment.front));

  segment.theta = 180;
  segment.radius = this->stem_radius_;
  segment.left_height_offset = segment.right_height_offset = 0;

  // find the position on the circle;
  auto projection(segment.up);
  projection.y = 0;
  projection = segment.radius * normalize(-projection);

  segment.position = projection + this->center_;

  // the segment for smooth Interpolation
  SorghumSplineSegment segment2(segment);
  float coefficient = 1.5f;
  segment2.position.y = first_segment.position.y / coefficient;
  segment2.theta = this->theta_ / 2;

  leaf.spline.segments.insert(leaf.spline.segments.begin(), segment);
  leaf.spline.segments.insert(leaf.spline.segments.begin() + 1, segment2);
}

std::vector<glm::vec3> SorghumDescriptorReconstruction::ReconstructSorghumStem(
    SorghumDescriptor& sorghum_descriptor) const {
  std::vector<glm::vec3> results;
  SorghumStemDescriptor stem;
  glm::vec3 start_point(0, 0, 0);
  start_point += this->center_;
  glm::vec3 up(0, 0.01, -1);
  glm::vec3 front(-0.04, 1, 0.01);
  for (int i = 0; i < this->stem_segments_count_; i++) {
    SorghumSplineSegment segment;
    segment.position = start_point + static_cast<float>(i) * glm::vec3(0, 0.01, 0);
    segment.up = up;
    segment.front = front;
    segment.radius = this->stem_radius_;
    segment.theta = 180;
    segment.right_height_offset = segment.left_height_offset = 0;
    stem.spline.segments.emplace_back(segment);

    results.emplace_back(segment.position);
    results.emplace_back(up);
    results.emplace_back(front);
  }

  sorghum_descriptor.stem = stem;
  return results;
}

std::vector<std::vector<glm::vec3>> SorghumDescriptorReconstruction::ReconstructSorghumFromBezierSplines(
    SorghumDescriptor& sorghum_descriptor,
    const std::vector<std::unordered_map<std::string, CubicBezierSpline>>& bezier_splines) const {
  // clear previous data
  sorghum_descriptor.leaves.clear();

  std::vector<std::string> keys = {"leftPoints", "rightPoints", "centerPoints"};

  std::vector<std::vector<glm::vec3>> results;
  for (int i = 0; i < bezier_splines.size(); i++) {
    auto leaf_splines = bezier_splines[i];

    SorghumSpline sorghum_spline;
    auto left_line = leaf_splines[keys[0]];
    auto right_line = leaf_splines[keys[1]];
    auto center_line = leaf_splines[keys[2]];

    std::vector<glm::vec3> leaf_profile;

    // update: getting samples based on distance
    // auto samples = centerLine.getUniformSamples(32);
    auto samples = center_line.GetSamplesByLength(0.015f);

    // reconstruct the local coordinate at each of the samples
    for (int j = 0; j < samples.size(); j++) {
      CubicSplineSample sample = samples[j];
      SorghumSplineSegment segment;

      // get the intersections of the left and right lines on the profile
      glm::vec3 position = sample.position * this->scale_;
      glm::vec3 normal = center_line.GetTangent(sample);
      // assume the closest intersection point should be the correct one
      // todo: how to deal with no intersection?
      std::vector<glm::vec3> left_candidates = left_line.GetSurfaceIntersection(sample.position, normal);
      auto left_intersection = std::min_element(left_candidates.begin(), left_candidates.end(),
                                                [&position](const glm::vec3& a, const glm::vec3& b) {
                                                  return glm::distance(a, position) < glm::distance(b, position);
                                                });

      std::vector<glm::vec3> right_candidates = right_line.GetSurfaceIntersection(sample.position, normal);
      auto right_intersection = std::min_element(right_candidates.begin(), right_candidates.end(),
                                                 [&position](const glm::vec3& a, const glm::vec3& b) {
                                                   return glm::distance(a, position) < glm::distance(b, position);
                                                 });

      glm::vec3 left_point;
      glm::vec3 right_point;

      if (left_intersection != left_candidates.end()) {
        left_point = *left_intersection * this->scale_;
      }
      if (right_intersection != right_candidates.end()) {
        right_point = *right_intersection * this->scale_;
      }

      if (left_intersection == left_candidates.end() || right_intersection == right_candidates.end()) {
        std::cout << "error in leaf: " << i << "\n";
      }

      //-----------------------
      // debug info
      //-----------------------
      leaf_profile.emplace_back(position);
      leaf_profile.emplace_back(left_point);
      leaf_profile.emplace_back(right_point);

      //----------------------------
      // from the profile, reconstruct the up vector
      //----------------------------
      glm::vec3 right = normalize(glm::cross(normal, glm::vec3(0, 1, 0)));
      right = glm::dot(right, left_point - position) < 0 ? -right : right;

      // position
      segment.position = position;

      // front
      segment.front = normal;

      segment.up = normalize(glm::cross(right, segment.front));

      // world -> local
      glm::mat4 world_to_local = inverse(glm::mat4(right.x, right.y, right.z, 0,                 // 1st column
                                                   segment.up.x, segment.up.y, segment.up.z, 0,  // 2nd column
                                                   -normal.x, -normal.y, -normal.z, 0,           // 3rd column
                                                   position.x, position.y, position.z, 1.0f      // 4th column
                                                   ));
      glm::vec3 right_local = world_to_local * glm::vec4(right_point, 1.0f);
      glm::vec3 left_local = world_to_local * glm::vec4(left_point, 1.0f);

      // reconstruct theta, radius, height_offsets
      segment.theta = this->theta_;
      // segment.radius = std::max(std::max(glm::length(leftLocal), glm::length(rightLocal)), 0.01f);
      segment.radius = std::max(std::abs(left_local.x), std::abs(right_local.x));
      // segment.radius = radius;

      segment.left_height_offset = -left_local.y * 1.5;
      segment.right_height_offset = -right_local.y * 1.5;
      // segment.left_height_offset = left_height_offset;
      // segment.right_height_offset = right_height_offset;

      sorghum_spline.segments.emplace_back(segment);
    }

    SorghumLeafDescriptor leaf_descriptor;
    leaf_descriptor.spline = sorghum_spline;
    leaf_descriptor.index = i;

    ExtendLeafToStem(leaf_descriptor);

    results.emplace_back(leaf_profile);
    sorghum_descriptor.leaves.emplace_back(leaf_descriptor);
  }
  // std::cout << "total samples count: " << results.size() << "\n";
  return results;
}

void SorghumDescriptorReconstruction::FillYamlPointsParticle(
    int leaf_index, const float scale,
    std::vector<std::unordered_map<std::string, std::vector<glm::vec3>>>& yaml_content,
    std::vector<ParticleInfo>& particle_infos, const int leaf_count, const int line_count, int points_count) {
  particle_infos.assign(leaf_count * points_count * line_count, ParticleInfo{});
  Jobs::RunParallelFor(leaf_count * points_count, [&](const auto i) {
    const int yamlContentIndex = leaf_index > -1 ? leaf_index : i / points_count;
    auto& center_info = particle_infos[i];
    const int index = i % points_count;
    center_info.instance_color = glm::vec4(256, 0, 0, 256) / 256.f;
    center_info.instance_matrix.SetPosition(glm::vec3(yaml_content[yamlContentIndex]["centerPoints"][index]) * scale);
    center_info.instance_matrix.SetScale(glm::vec3(0.005f));

    auto& left_info = particle_infos[i + leaf_count * points_count];
    left_info.instance_color = glm::vec4(0, 256, 0, 256) / 256.f;
    left_info.instance_matrix.SetPosition(glm::vec3(yaml_content[yamlContentIndex]["leftPoints"][index]) * scale);
    left_info.instance_matrix.SetScale(glm::vec3(0.005f));

    auto& right_info = particle_infos[i + 2 * leaf_count * points_count];
    right_info.instance_color = glm::vec4(0, 0, 256, 256) / 256.f;
    right_info.instance_matrix.SetPosition(glm::vec3(yaml_content[yamlContentIndex]["rightPoints"][index]) * scale);
    right_info.instance_matrix.SetScale(glm::vec3(0.005f));
  });
}

void SorghumDescriptorReconstruction::FillBezierSplinePointsParticle(
    const int leaf_index, const float scale, const std::vector<std::vector<glm::vec3>>& bezier_spline_points,
    std::vector<ParticleInfo>& particle_infos, const int leaf_count, int line_count, int points_count,
    bool uniform_segment_count) {
  const std::vector<glm::vec4> colors = {glm::vec4(0, 256, 256, 128), glm::vec4(256, 0, 256, 128),
                                         glm::vec4(256, 256, 0, 128)};

  particle_infos.clear();
  const int leafStartIndex = leaf_index > -1 ? leaf_index : 0;
  for (int i = leafStartIndex; i < bezier_spline_points.size() && i < leafStartIndex + leaf_count; i++) {
    for (int j = 0; j < bezier_spline_points[i].size(); j += 3) {
      ParticleInfo center_info;
      center_info.instance_matrix.SetPosition(bezier_spline_points[i][j] * scale);
      center_info.instance_matrix.SetScale(glm::vec3(0.005f));
      center_info.instance_color = colors[0] / 256.f;

      ParticleInfo left_info;
      left_info.instance_matrix.SetPosition(bezier_spline_points[i][j + 1] * scale);
      left_info.instance_matrix.SetScale(glm::vec3(0.005f));
      left_info.instance_color = colors[1] / 256.f;

      ParticleInfo right_info;
      right_info.instance_matrix.SetPosition(bezier_spline_points[i][j + 2] * scale);
      right_info.instance_matrix.SetScale(glm::vec3(0.005f));
      right_info.instance_color = colors[2] / 256.f;

      particle_infos.emplace_back(center_info);
      particle_infos.emplace_back(left_info);
      particle_infos.emplace_back(right_info);
    }
  }
}

void SorghumDescriptorReconstruction::FillLeafSegmentFrameParticle(int leaf_index, float scale,
                                                                   SorghumDescriptor& sorghum_descriptor,
                                                                   std::vector<ParticleInfo>& particle_infos,
                                                                   int leaf_count, bool uniform_segment_count) {
  std::vector<glm::vec4> colors = {glm::vec4(256, 256, 256, 256), glm::vec4(0, 256, 256, 256),
                                   glm::vec4(256, 0, 256, 256), glm::vec4(256, 256, 0, 256)};
  int sample_points = 10;

  int segment_count = sorghum_descriptor.leaves[0].spline.segments.size();
  float dis = glm::distance(sorghum_descriptor.leaves[0].spline.segments[0].position,
                            sorghum_descriptor.leaves[0].spline.segments[1].position);

  dis = 0.003f;
  if (uniform_segment_count) {
    particle_infos.assign(leaf_count * segment_count * sample_points, ParticleInfo{});
    Jobs::RunParallelFor(leaf_count * segment_count, [&](const auto i) {
      int leaf_start_index = leaf_index > -1 ? leaf_index : i / segment_count;
      int segment_index = i % segment_count;
      auto segment = sorghum_descriptor.leaves[leaf_start_index].spline.segments[segment_index];
      auto position = segment.position;
      auto front = segment.front;
      auto up = segment.up;
      auto right = normalize(glm::cross(front, up));

      for (int j = 0; j < sample_points; j++) {
        auto& info = particle_infos[i + j * leaf_count * segment_count];
        if (j == 0) {
          info.instance_color = colors[0] / 256.f;
          info.instance_matrix.SetPosition(position * scale);
        } else if (j < 3) {
          info.instance_color = colors[1] / 256.f;
          info.instance_matrix.SetPosition(((3 - j) * dis * front + position) * scale);
        } else if (j < 6) {
          info.instance_color = colors[2] / 256.f;
          info.instance_matrix.SetPosition(((6 - j) * dis * right + position) * scale);
        } else {
          info.instance_color = colors[3] / 256.f;
          info.instance_matrix.SetPosition(((9 - j) * dis * up + position) * scale);
        }

        info.instance_matrix.SetScale(glm::vec3(0.002f));
      }
    });
  } else {
    particle_infos.clear();
    int leaf_start_index = leaf_index > -1 ? leaf_index : 0;
    for (int i = leaf_start_index; i < sorghum_descriptor.leaves.size() && i < leaf_start_index + leaf_count; i++) {
      auto leaf = sorghum_descriptor.leaves[i];
      for (auto segment : leaf.spline.segments) {
        auto position = segment.position;
        auto front = normalize(segment.front);
        auto up = normalize(segment.up);
        auto right = normalize(glm::cross(front, up));

        for (int j = 0; j < sample_points; j++) {
          ParticleInfo info;
          if (j == 0) {
            info.instance_color = colors[0] / 256.f;
            info.instance_matrix.SetPosition(position * scale);
          } else if (j < 3) {
            info.instance_color = colors[1] / 256.f;
            info.instance_matrix.SetPosition(((3 - j) * dis * front + position) * scale);
          } else if (j < 6) {
            info.instance_color = colors[2] / 256.f;
            info.instance_matrix.SetPosition(((6 - j) * dis * right + position) * scale);
          } else {
            info.instance_color = colors[3] / 256.f;
            info.instance_matrix.SetPosition(((9 - j) * dis * up + position) * scale);
          }
          info.instance_matrix.SetScale(glm::vec3(0.002f));
          particle_infos.emplace_back(info);
        }
      }
    }
  }
}
