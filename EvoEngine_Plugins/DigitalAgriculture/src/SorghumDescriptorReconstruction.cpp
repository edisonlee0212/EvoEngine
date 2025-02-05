/*****************************************************************//**
 * \file   SorghumDescriptorReconstruction.cpp
 * \brief  Provides implementation of interfaces defined in SorghumDescriptorReconstruction
 * 
 * \author Demoy
 * \date   February 2025
 *********************************************************************/

// @edisonlee0212: here are the implementations.

#include "SorghumDescriptorReconstruction.hpp"

#include "Sorghum.hpp"
#include "SorghumLayer.hpp"

using namespace digital_agriculture_plugin;

glm::vec3 CubicBezierSpline::interpolation(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2, const glm::vec3& v3, const float t) {
  glm::vec3 b = (v1 - v0) * 3.0f;
  glm::vec3 c = (v2 - v1) * 3.0f - b;
  glm::vec3 d = (v3 - v0) - b - c;
  return v0 + b * t + c * t * t + d * t * t * t;
}

glm::vec3 CubicBezierSpline::getTangent(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2, const glm::vec3& v3, const float t) {
  glm::vec3 b = (v1 - v0) * 3.0f;
  glm::vec3 c = (v2 - v1) * 3.0f - b;
  glm::vec3 d = (v3 - v0) - b - c;

  glm::vec3 tangent = b + 2.0f * c * t + 3.0f * d * t * t;

  return normalize(tangent);
}

float CubicBezierSpline::calculateLengthAdaptive(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2, const glm::vec3& v3, float t_start, float t_end, const float tolerance) {
  const glm::vec3 mid_point = interpolation(v0, v1, v2, v3, (t_start + t_end) * 0.5f);
  const glm::vec3 start_point = interpolation(v0, v1, v2, v3, t_start);
  const glm::vec3 end_point = interpolation(v0, v1, v2, v3, t_end);

  const float linear_distance = glm::distance(start_point, end_point);
  const float curve_distance = glm::distance(start_point, mid_point) + glm::distance(mid_point, end_point);
  if (fabs(linear_distance - curve_distance) < tolerance) {
    return curve_distance;  // Close enough, return this estimate
  }
  // Subdivide further
  return calculateLengthAdaptive(v0, v1, v2, v3, t_start, (t_start + t_end) * 0.5f, tolerance) +
         calculateLengthAdaptive(v0, v1, v2, v3, (t_start + t_end) * 0.5f, t_end, tolerance);
}

float CubicBezierSpline::findTAdaptive(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2, const glm::vec3& v3, const float t_start, const float target_length, const float tolerance) {
  
  if (calculateLengthAdaptive(v0, v1, v2, v3, t_start, 1.f, tolerance) <= target_length)
    return 1.f;
  float t_low = t_start, t_high = 1.0f;
  while (t_high - t_low > tolerance) {
    if (float t_mid = (t_low + t_high) * 0.5f;
        calculateLengthAdaptive(v0, v1, v2, v3, t_start, t_mid) < target_length) {
      t_low = t_mid;
    } else {
      t_high = t_mid;
    }
  }
  return (t_low + t_high) * 0.5f;
  
}

float CubicBezierSpline::getLength() {
  float result = 0;
  segmentLengths.clear();
  for (int i = 0; i < joints.size() - 1; i++) {
    float segmentLength = calculateLengthAdaptive(joints[i].position, joints[i].right_handle, joints[i + 1].left_handle,
                                                  joints[i + 1].position);
    result += segmentLength;
    segmentLengths.push_back(segmentLength);
  }
  return result;
}

std::vector<CubicSplineSample> CubicBezierSpline::getSamplesByLength(float distance) {
  float length = getLength();
  int sampleNum = length / distance;
  return getUniformSamples(sampleNum);
}


std::vector<CubicSplineSample> CubicBezierSpline::getUniformSamples(int num) {
  std::vector<CubicSplineSample> samples;
  float length = getLength();
  float lengthPerSample = length / (num + 1);
  float sampleLength = lengthPerSample;
  float currentLength = 0;
  std::cout << "segment count in getUniform: " << segmentLengths.size() << "\n";
  for (int i = 0; i < segmentLengths.size(); i++) {
    while (currentLength < sampleLength) {
      currentLength += segmentLengths[i++];
    }
    i--;
    glm::vec3 p0 = joints[i].position;
    glm::vec3 p1 = joints[i].right_handle;
    glm::vec3 p2 = joints[i + 1].left_handle;
    glm::vec3 p3 = joints[i + 1].position;
    glm::vec3 pp0 = p0 + (p0 - p1);
    glm::vec3 pp3 = p3 + (p3 - p2);

    float t = findTAdaptive(joints[i].position, joints[i].right_handle, joints[i + 1].left_handle,
                            joints[i + 1].position, 0, sampleLength - currentLength + segmentLengths[i], 0.00001f);
    glm::vec3 pos = (interpolation(joints[i].position, joints[i].right_handle, joints[i + 1].left_handle,
                                   joints[i + 1].position, t));

    CubicSplineSample s;
    s.position = pos;
    s.segmentIndex = i;
    s.t = t;
    samples.emplace_back(s);

    if (samples.size() == num) {
      break;
    }
    sampleLength += lengthPerSample;
  }
  if (samples.size() != num) {
    EVOENGINE_ERROR("invalid samples count in getUniformSamples")
  }
  return samples;
}

std::vector<glm::vec3> CubicBezierSpline::getSurfaceIntersection(const glm::vec3& planePoint, const glm::vec3& normal) const {
  std::vector<glm::vec3> results;
  for (int i = 0; i < joints.size() - 1; i++) {
    glm::vec3 p0 = joints[i].position;
    glm::vec3 p1 = joints[i].right_handle;
    glm::vec3 p2 = joints[i + 1].left_handle;
    glm::vec3 p3 = joints[i + 1].position;

    // assume: that each of the bezier curves should only intersect with the plane once

    // check whether the start point and the end point are on different sides of the plane
    if (glm::dot(p0 - planePoint, normal) * glm::dot(p3 - planePoint, normal) > 0) {
      // std::cout << "skip, i = " << i << "\n";
      continue;
    }

    // if there is intersection, two-step search for t
    float t = bisectionMethod(0.0f, 1.0f, p0, p1, p2, p3, planePoint, normal);
    // std::cout << "after bisection, t = :" << t << "i = " <<i<< "\n";
    t = newtonMethod(t, p0, p1, p2, p3, planePoint, normal);
    // std::cout << "after newton, t = :" << t << "i = " << i << "\n";
    if (t > 1 || t < 0) {
      continue;
    }
    glm::vec3 intersectionPoint = interpolation(p0, p1, p2, p3, t);

    results.push_back(intersectionPoint);
  }
  // std::cout << "get intersection size:" << results.size() << "\n";
  return results;
}

float CubicBezierSpline::bisectionMethod(float t_min, float t_max, const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2, const glm::vec3& v3, const glm::vec3& planePoint, const glm::vec3& normal, float epsilon) {
  float t_mid;
  while (t_max - t_min > epsilon) {
    t_mid = (t_min + t_max) / 2.0f;
    glm::vec3 bezierPoint = interpolation(v0, v1, v2, v3, t_mid);
    float value = planeEquation(bezierPoint, normal, planePoint);
    if (value > 0) {
      t_max = t_mid;
    } else {
      t_min = t_mid;
    }
  }
  return (t_min + t_max) / 2.0f;
}

float CubicBezierSpline::newtonMethod(float t, const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2, const glm::vec3& v3, const glm::vec3& planePoint, const glm::vec3& normal, float epsilon, int maxIter) {
  for (int i = 0; i < maxIter; ++i) {
    glm::vec3 bezierPoint = interpolation(v0, v1, v2, v3, t);
    float value = planeEquation(bezierPoint, normal, planePoint);
    if (std::abs(value) < epsilon) {
      return t;  // find intersection
    }

    // calculate the tangent
    glm::vec3 bezierDerivative = getTangent(v0, v1, v2, v3, t);
    float derivative = glm::dot(normal, bezierDerivative);  // calculate derivative
    if (std::abs(derivative) < epsilon) {
      break;
    }
    t -= value / derivative;  // update t
  }
  return t;
}

glm::vec3 CubicBezierSpline::getTangent(const CubicSplineSample& sample) const {
  glm::vec3 p0 = joints[sample.segmentIndex].position;
  glm::vec3 p1 = joints[sample.segmentIndex].right_handle;
  glm::vec3 p2 = joints[sample.segmentIndex + 1].left_handle;
  glm::vec3 p3 = joints[sample.segmentIndex + 1].position;
  float t = sample.t;
  return getTangent(p0, p1, p2, p3, t);
}

glm::vec3 CubicBezierSpline::segmentInterpolation(int segmentIndex, float t) const {
  glm::vec3 p0 = joints[segmentIndex].position;
  glm::vec3 p1 = joints[segmentIndex].right_handle;
  glm::vec3 p2 = joints[segmentIndex + 1].left_handle;
  glm::vec3 p3 = joints[segmentIndex + 1].position;
  return interpolation(p0, p1, p2, p3, t);
}

std::vector<glm::vec3> CubicBezierSpline::getLineSamples(int numPerCurve) const {
  std::vector<glm::vec3> results;

  for (int i = 0; i < joints.size() - 1; i++) {
    glm::vec3 p0 = joints[i].position;
    glm::vec3 p1 = joints[i].right_handle;
    glm::vec3 p2 = joints[i + 1].left_handle;
    glm::vec3 p3 = joints[i + 1].position;

    int num = 0;
    float val = 1 / (float)numPerCurve;
    while (num < numPerCurve) {
      float t = num * val;
      results.push_back(interpolation(p0, p1, p2, p3, t));
      num++;
    }
  }
  return results;
}

std::vector<std::unordered_map<std::string, CubicBezierSpline>> SorghumDescriptorReconstruction::ReconstructBezierSplineFromYAML(std::vector<std::unordered_map<std::string, std::vector<glm::vec3>>>& yaml_content) {
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
      CubicBezierSpline bezierSpline;
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

        bezierSpline.joints.emplace_back(p);
      }

      leafRepresentation.insert(std::make_pair(keys[j], bezierSpline));
    }

    bezierSplines.emplace_back(leafRepresentation);
  }

  return bezierSplines;
}

std::vector<std::unordered_map<std::string, std::vector<glm::vec3>>> SorghumDescriptorReconstruction::GetLineSamplesFromBezierSplines(std::vector<std::unordered_map<std::string, CubicBezierSpline>> bezierSplines, int numPerCurve) {
  std::vector<std::unordered_map<std::string, std::vector<glm::vec3>>> results;

  std::vector<std::string> keys = {"leftPoints", "rightPoints", "centerPoints"};

  for (int i = 0; i < bezierSplines.size(); i++) {
    std::unordered_map<std::string, std::vector<glm::vec3>> pointSet;

    auto leafSplines = bezierSplines[i];

    for (auto key : keys) {
      CubicBezierSpline bezierSpline = leafSplines[key];
      auto points = bezierSpline.getLineSamples(numPerCurve);

      pointSet.insert(std::make_pair(key, points));
    }

    results.emplace_back(pointSet);
  }

  return results;
}

void SorghumDescriptorReconstruction::ExtendLeafToStem(SorghumLeafDescriptor& leaf) const{
  auto firstSegment = leaf.spline.segments[0];

  auto right = normalize(glm::cross(firstSegment.front, glm::vec3(0, 1, 0)));

  // construct the local frame
  SorghumSplineSegment segment;

  segment.front = normalize(glm::vec3(-0.04, 1, 0.01));

  segment.up = normalize(glm::cross(right, segment.front));

  segment.theta = 180;
  segment.radius = stemRaius;
  segment.left_height_offset = segment.right_height_offset = 0;

  // find the position on the circle;
  auto projection(segment.up);
  projection.y = 0;
  projection = segment.radius * normalize(-projection);

  segment.position = projection + center;

  // the segment for smooth interpolation
  SorghumSplineSegment segment2(segment);
  float coefficient = 1.5f;
  segment2.position.y = firstSegment.position.y / coefficient;
  segment2.theta = theta / 2;

  leaf.spline.segments.insert(leaf.spline.segments.begin(), segment);
  leaf.spline.segments.insert(leaf.spline.segments.begin() + 1, segment2);
}

std::vector<glm::vec3> SorghumDescriptorReconstruction::ReconstructSorghumStem(SorghumDescriptor& sorghum_descriptor) const{
  std::vector<glm::vec3> results;
  SorghumStemDescriptor stem;
  glm::vec3 startPoint(0, 0, 0);
  startPoint += center;
  glm::vec3 up(0, 0.01, -1);
  glm::vec3 front(-0.04, 1, 0.01);
  for (int i = 0; i < stemSegmentsCount; i++) {
    SorghumSplineSegment segment;
    segment.position = startPoint + (float)i * glm::vec3(0, 0.01, 0);
    segment.up = up;
    segment.front = front;
    segment.radius = stemRaius;
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

std::vector<std::vector<glm::vec3>> SorghumDescriptorReconstruction::ReconstructSorghumFromBezierSplines(SorghumDescriptor& sorghum_descriptor, const std::vector<std::unordered_map<std::string, CubicBezierSpline>>& bezierSplines) const{
  // clear previous data
  sorghum_descriptor.leaves.clear();

  std::vector<std::string> keys = {"leftPoints", "rightPoints", "centerPoints"};
  int leafCount = bezierSplines.size();

  std::vector<std::vector<glm::vec3>> results;
  for (int i = 0; i < bezierSplines.size(); i++) {
    auto leafSplines = bezierSplines[i];

    SorghumSpline sorghumSpline;
    auto leftLine = leafSplines[keys[0]];
    auto rightLine = leafSplines[keys[1]];
    auto centerLine = leafSplines[keys[2]];

    std::vector<glm::vec3> leafProfile;

    // update: getting samples based on distance
    // auto samples = centerLine.getUniformSamples(32);
    auto samples = centerLine.getSamplesByLength(0.015f);

    // reconstruct the local coordinate at each of the samples
    for (int j = 0; j < samples.size(); j++) {
      CubicSplineSample sample = samples[j];
      SorghumSplineSegment segment;

      // get the intersections of the left and right lines on the profile
      glm::vec3 position = sample.position * scale;
      glm::vec3 normal = centerLine.getTangent(sample);
      // assume the closest intersection point should be the correct one
      // todo: how to deal with no intersection?
      std::vector<glm::vec3> leftCandidates = leftLine.getSurfaceIntersection(sample.position, normal);
      auto leftIntersection = std::min_element(leftCandidates.begin(), leftCandidates.end(),
                                               [&position](const glm::vec3& a, const glm::vec3& b) {
                                                 return glm::distance(a, position) < glm::distance(b, position);
                                               });

      std::vector<glm::vec3> rightCandidates = rightLine.getSurfaceIntersection(sample.position, normal);
      auto rightIntersection = std::min_element(rightCandidates.begin(), rightCandidates.end(),
                                                [&position](const glm::vec3& a, const glm::vec3& b) {
                                                  return glm::distance(a, position) < glm::distance(b, position);
                                                });

      glm::vec3 leftPoint;
      glm::vec3 rightPoint;

      if (leftIntersection != leftCandidates.end()) {
        leftPoint = *leftIntersection * scale;
      }
      if (rightIntersection != rightCandidates.end()) {
        rightPoint = *rightIntersection * scale;
      }

      if (leftIntersection == leftCandidates.end() || rightIntersection == rightCandidates.end()) {
        std::cout << "error in leaf: " << i << "\n";
      }

      //-----------------------
      // debug info
      //-----------------------
      leafProfile.emplace_back(position);
      leafProfile.emplace_back(leftPoint);
      leafProfile.emplace_back(rightPoint);

      //----------------------------
      // from the profile, reconstruct the up vector
      //----------------------------
      glm::vec3 right = normalize(glm::cross(normal, glm::vec3(0, 1, 0)));
      right = glm::dot(right, leftPoint - position) < 0 ? -right : right;

      // position
      segment.position = position;

      // front
      segment.front = normal;

      segment.up = normalize(glm::cross(right, segment.front));

      // world -> local
      glm::mat4 worldToLocal = inverse(glm::mat4(right.x, right.y, right.z, 0,                 // 1st column
                                                 segment.up.x, segment.up.y, segment.up.z, 0,  // 2nd column
                                                 -normal.x, -normal.y, -normal.z, 0,           // 3rd column
                                                 position.x, position.y, position.z, 1.0f      // 4th column
                                                 ));
      glm::vec3 rightLocal = worldToLocal * glm::vec4(rightPoint, 1.0f);
      glm::vec3 leftLocal = worldToLocal * glm::vec4(leftPoint, 1.0f);

      // reconstruct theta, radius, height_offsets
      segment.theta = theta;
      // segment.radius = std::max(std::max(glm::length(leftLocal), glm::length(rightLocal)), 0.01f);
      segment.radius = std::max(std::abs(leftLocal.x), std::abs(rightLocal.x));
      // segment.radius = radius;

      segment.left_height_offset = -leftLocal.y * 1.5;
      segment.right_height_offset = -rightLocal.y * 1.5;
      // segment.left_height_offset = left_height_offset;
      // segment.right_height_offset = right_height_offset;

      sorghumSpline.segments.emplace_back(segment);
    }

    SorghumLeafDescriptor leaf_descriptor;
    leaf_descriptor.spline = sorghumSpline;
    leaf_descriptor.index = i;

    ExtendLeafToStem(leaf_descriptor);

    results.emplace_back(leafProfile);
    sorghum_descriptor.leaves.emplace_back(leaf_descriptor);
  }
  //std::cout << "total samples count: " << results.size() << "\n";
  return results;
}

void SorghumDescriptorReconstruction::FillYAMLPointsParticle(int leafIndex, float scale, std::vector<std::unordered_map<std::string, std::vector<glm::vec3>>>& yamlContent, std::vector<ParticleInfo>& particleInfos, int leafCount, int lineCount, int PointsCount) {
   particleInfos.assign(leafCount* PointsCount* lineCount, ParticleInfo{});
   Jobs::RunParallelFor(leafCount* PointsCount, [&](const auto i) {
     int yamlContentIndex = leafIndex > -1 ? leafIndex : i / PointsCount;
     auto& center_info = particleInfos[i];
     int index = i % PointsCount;
     center_info.instance_color = glm::vec4(256, 0, 0, 256) / 256.f;
     center_info.instance_matrix.SetPosition(glm::vec3(yamlContent[yamlContentIndex]["centerPoints"][index]) * scale);
     center_info.instance_matrix.SetScale(glm::vec3(0.005f));
  
     auto& left_info = particleInfos[i + leafCount * PointsCount];
     left_info.instance_color = glm::vec4(0, 256, 0, 256) / 256.f;
     left_info.instance_matrix.SetPosition(glm::vec3(yamlContent[yamlContentIndex]["leftPoints"][index]) * scale);
     left_info.instance_matrix.SetScale(glm::vec3(0.005f));
  
     auto& right_info = particleInfos[i + 2 * leafCount * PointsCount];
     right_info.instance_color = glm::vec4(0, 0, 256, 256) / 256.f;
     right_info.instance_matrix.SetPosition(glm::vec3(yamlContent[yamlContentIndex]["rightPoints"][index]) * scale);
     right_info.instance_matrix.SetScale(glm::vec3(0.005f));
   });
}

void SorghumDescriptorReconstruction::FillBezierSplinePointsParticle(int leafIndex, float scale, std::vector<std::vector<glm::vec3>>& bezierSplinePoints, std::vector<ParticleInfo>& particleInfos, int leafCount, int lineCount, int PointsCount, bool uniformSegmentCount) {
  
  std::vector<glm::vec4> colors = {glm::vec4(0, 256, 256, 128), glm::vec4(256, 0, 256, 128),
                                   glm::vec4(256, 256, 0, 128)};

  particleInfos.clear();
  int leafStartIndex = leafIndex > -1 ? leafIndex : 0;
  for (int i = leafStartIndex; i < bezierSplinePoints.size() && i < leafStartIndex + leafCount; i++) {
    for (int j = 0; j < bezierSplinePoints[i].size(); j += 3) {
      ParticleInfo centerInfo;
      centerInfo.instance_matrix.SetPosition(bezierSplinePoints[i][j] * scale);
      centerInfo.instance_matrix.SetScale(glm::vec3(0.005f));
      centerInfo.instance_color = colors[0] / 256.f;

      ParticleInfo leftInfo;
      leftInfo.instance_matrix.SetPosition(bezierSplinePoints[i][j + 1] * scale);
      leftInfo.instance_matrix.SetScale(glm::vec3(0.005f));
      leftInfo.instance_color = colors[1] / 256.f;

      ParticleInfo rightInfo;
      rightInfo.instance_matrix.SetPosition(bezierSplinePoints[i][j + 2] * scale);
      rightInfo.instance_matrix.SetScale(glm::vec3(0.005f));
      rightInfo.instance_color = colors[2] / 256.f;

      particleInfos.emplace_back(centerInfo);
      particleInfos.emplace_back(leftInfo);
      particleInfos.emplace_back(rightInfo);
    }
  }
  
}

void SorghumDescriptorReconstruction::FillLeafSegmentFrameParticle(int leafIndex, float scale, SorghumDescriptor& sorghum_descriptor, std::vector<ParticleInfo>& particleInfos, int leafCount, bool uniformSegmentCount) {
  std::vector<glm::vec4> colors = {glm::vec4(256, 256, 256, 256), glm::vec4(0, 256, 256, 256),
                                   glm::vec4(256, 0, 256, 256), glm::vec4(256, 256, 0, 256)};
  int samplePoints = 10;

  int segmentCount = sorghum_descriptor.leaves[0].spline.segments.size();
  float dis = glm::distance(sorghum_descriptor.leaves[0].spline.segments[0].position,
                            sorghum_descriptor.leaves[0].spline.segments[1].position);

  dis = 0.003f;
  if (uniformSegmentCount) {
    particleInfos.assign(leafCount * segmentCount * samplePoints, ParticleInfo{});
    Jobs::RunParallelFor(leafCount * segmentCount, [&](const auto i) {
      int leafStartIndex = leafIndex > -1 ? leafIndex : i / segmentCount;
      int segmentIndex = i % segmentCount;
      auto segment = sorghum_descriptor.leaves[leafStartIndex].spline.segments[segmentIndex];
      auto position = segment.position;
      auto front = segment.front;
      auto up = segment.up;
      auto right = normalize(glm::cross(front, up));

      for (int j = 0; j < samplePoints; j++) {
        auto& info = particleInfos[i + j * leafCount * segmentCount];
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
    particleInfos.clear();
    int leafStartIndex = leafIndex > -1 ? leafIndex : 0;
    for (int i = leafStartIndex; i < sorghum_descriptor.leaves.size() && i < leafStartIndex + leafCount; i++) {
      auto leaf = sorghum_descriptor.leaves[i];
      for (auto segment : leaf.spline.segments) {
        auto position = segment.position;
        auto front = normalize(segment.front);
        auto up = normalize(segment.up);
        auto right = normalize(glm::cross(front, up));

        for (int j = 0; j < samplePoints; j++) {
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
          particleInfos.emplace_back(info);
        }
      }
    }
  }
}









