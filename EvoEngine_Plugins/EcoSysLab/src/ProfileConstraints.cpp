#include "ProfileConstraints.hpp"

#include "TreeVisualizer.hpp"
using namespace eco_sys_lab;

bool OnSegment(const glm::vec2& p, const glm::vec2& q, const glm::vec2& r) {
  if (q.x <= glm::max(p.x, r.x) && q.x >= glm::min(p.x, r.x) && q.y <= glm::max(p.y, r.y) && q.y >= glm::min(p.y, r.y))
    return true;

  return false;
}
int Orientation(const glm::vec2& p, const glm::vec2& q, const glm::vec2& r) {
  // See https://www.geeksforgeeks.org/orientation-3-ordered-points/
  // for details of below formula.
  const float val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);

  if (val == 0.0f)
    return 0;  // collinear

  return (val > 0) ? 1 : 2;  // clock or counterclock wise
}

void ProfileBoundary::CalculateCenter() {
  center = glm::vec2(0.0f);
  float sum = 0.0f;
  for (int line_index = 0; line_index < points.size(); line_index++) {
    const auto& p1 = points[line_index];
    const auto& p2 = points[(line_index + 1) % points.size()];
    const auto line_length = glm::distance(p1, p2);
    sum += line_length;
    center += (p1 + p2) * 0.5f * line_length;
  }
  center /= sum;
}

void ProfileBoundary::RenderBoundary(const ImVec2 origin, const float zoom_factor, ImDrawList* draw_list, ImU32 color,
                                     float thickness) const {
  for (int point_index = 0; point_index < points.size() - 1; point_index++) {
    const auto& p1 = points[point_index];
    const auto& p2 = points[point_index + 1];
    draw_list->AddLine(ImVec2(origin.x + p1.x * zoom_factor, origin.y + p1.y * zoom_factor),
                       ImVec2(origin.x + p2.x * zoom_factor, origin.y + p2.y * zoom_factor), color, thickness);
  }

  const auto& p1 = points.back();
  const auto& p2 = points[0];
  draw_list->AddLine(ImVec2(origin.x + p1.x * zoom_factor, origin.y + p1.y * zoom_factor),
                     ImVec2(origin.x + p2.x * zoom_factor, origin.y + p2.y * zoom_factor), color, thickness);
}

void ProfileAttractor::RenderAttractor(ImVec2 origin, float zoom_factor, ImDrawList* draw_list, ImU32 color,
                                       float thickness) const {
  if (attractor_points.empty())
    return;
  for (const auto& attractor_point : attractor_points) {
    const auto& p1 = attractor_point.first;
    const auto& p2 = attractor_point.second;
    draw_list->AddLine(ImVec2(origin.x + p1.x * zoom_factor, origin.y + p1.y * zoom_factor),
                       ImVec2(origin.x + p2.x * zoom_factor, origin.y + p2.y * zoom_factor), color, thickness);
  }
}

glm::vec2 ProfileAttractor::FindClosestPoint(const glm::vec2& position) const {
  auto distance = FLT_MAX;
  auto ret_val = glm::vec2(0.0f);
  for (const auto& attractor_point : attractor_points) {
    const auto& p1 = attractor_point.first;
    const auto& p2 = attractor_point.second;
    const auto test_point = glm::closestPointOnLine(position, p1, p2);
    if (const auto new_distance = glm::distance(test_point, position); distance > new_distance) {
      distance = new_distance;
      ret_val = test_point;
    }
  }
  return ret_val;
}

bool ProfileBoundary::BoundaryValid() const {
  if (points.size() <= 3)
    return false;
  for (int line_index = 0; line_index < points.size(); line_index++) {
    const auto& p1 = points[line_index];
    const auto& p2 = points[(line_index + 1) % points.size()];
    for (int line_index2 = 0; line_index2 < points.size(); line_index2++) {
      if (line_index == line_index2)
        continue;
      if ((line_index + 1) % points.size() == line_index2 || (line_index2 + 1) % points.size() == line_index)
        continue;
      const auto& p3 = points[line_index2];
      const auto& p4 = points[(line_index2 + 1) % points.size()];
      if (Intersect(p1, p2, p3, p4)) {
        return false;
      }
    }
  }
  return true;
}

bool ProfileBoundary::InBoundary(const glm::vec2& position) const {
  auto distance = FLT_MAX;
  int intersect_count1 = 0;
  int intersect_count2 = 0;
  int intersect_count3 = 0;
  int intersect_count4 = 0;
  for (int line_index = 0; line_index < points.size(); line_index++) {
    const auto& p1 = points[line_index];
    const auto& p2 = points[(line_index + 1) % points.size()];
    const auto p3 = position;
    const auto p41 = position + glm::vec2(1000.0f, 0.0f);
    const auto p42 = position + glm::vec2(-1000.0f, 0.0f);
    const auto p43 = position + glm::vec2(0.0f, 1000.0f);
    const auto p44 = position + glm::vec2(0.0f, -1000.0f);
    if (Intersect(p1, p2, p3, p41)) {
      intersect_count1++;
    }
    if (Intersect(p1, p2, p3, p42)) {
      intersect_count2++;
    }
    if (Intersect(p1, p2, p3, p43)) {
      intersect_count3++;
    }
    if (Intersect(p1, p2, p3, p44)) {
      intersect_count4++;
    }
    const auto test_point = glm::closestPointOnLine(position, p1, p2);
    if (const auto new_distance = glm::distance(test_point, position); distance > new_distance) {
      distance = new_distance;
    }
  }
  return intersect_count1 % 2 != 0 && intersect_count2 % 2 != 0 && intersect_count3 % 2 != 0 &&
         intersect_count4 % 2 != 0;
}

bool ProfileBoundary::InBoundary(const glm::vec2& position, glm::vec2& closest_point) const {
  closest_point = glm::vec2(0.0f);
  auto distance = FLT_MAX;
  int intersect_count1 = 0;
  int intersect_count2 = 0;
  int intersect_count3 = 0;
  int intersect_count4 = 0;
  for (int line_index = 0; line_index < points.size(); line_index++) {
    const auto& p1 = points[line_index];
    const auto& p2 = points[(line_index + 1) % points.size()];
    const auto p3 = position;
    const auto p41 = position + glm::vec2(1000.0f, 0.0f);
    const auto p42 = position + glm::vec2(-1000.0f, 0.0f);
    const auto p43 = position + glm::vec2(0.0f, 1000.0f);
    const auto p44 = position + glm::vec2(0.0f, -1000.0f);
    if (Intersect(p1, p2, p3, p41)) {
      intersect_count1++;
    }
    if (Intersect(p1, p2, p3, p42)) {
      intersect_count2++;
    }
    if (Intersect(p1, p2, p3, p43)) {
      intersect_count3++;
    }
    if (Intersect(p1, p2, p3, p44)) {
      intersect_count4++;
    }
    const auto test_point = glm::closestPointOnLine(position, p1, p2);
    if (const auto new_distance = glm::distance(test_point, position); distance > new_distance) {
      closest_point = test_point;
      distance = new_distance;
    }
  }
  const bool ret_val =
      intersect_count1 % 2 != 0 && intersect_count2 % 2 != 0 && intersect_count3 % 2 != 0 && intersect_count4 % 2 != 0;
  return ret_val;
}

bool ProfileBoundary::Intersect(const glm::vec2& p1, const glm::vec2& q1, const glm::vec2& p2, const glm::vec2& q2) {
  // Find the four orientations needed for general and
  // special cases
  const int o1 = Orientation(p1, q1, p2);
  const int o2 = Orientation(p1, q1, q2);
  const int o3 = Orientation(p2, q2, p1);
  const int o4 = Orientation(p2, q2, q1);
  // General case
  if (o1 != o2 && o3 != o4)
    return true;

  // Special Cases
  // p1, q1 and p2 are collinear and p2 lies on segment p1q1
  if (o1 == 0 && OnSegment(p1, p2, q1))
    return true;

  // p1, q1 and q2 are collinear and q2 lies on segment p1q1
  if (o2 == 0 && OnSegment(p1, q2, q1))
    return true;

  // p2, q2 and p1 are collinear and p1 lies on segment p2q2
  if (o3 == 0 && OnSegment(p2, p1, q2))
    return true;

  // p2, q2 and q1 are collinear and q1 lies on segment p2q2
  if (o4 == 0 && OnSegment(p2, q1, q2))
    return true;

  return false;  // Doesn't fall in any of the above cases
}

int ProfileConstraints::FindBoundary(const glm::vec2& position) const {
  int ret_val = -1;
  auto distance_to_closest_boundary_center = FLT_MAX;
  for (int boundary_index = 0; boundary_index < boundaries.size(); boundary_index++) {
    const auto& boundary = boundaries.at(boundary_index);
    glm::vec2 test;
    if (boundary.InBoundary(position, test)) {
      if (const auto current_distance_to_boundary_center = glm::distance(position, boundary.center);
          current_distance_to_boundary_center < distance_to_closest_boundary_center) {
        distance_to_closest_boundary_center = current_distance_to_boundary_center;
        ret_val = boundary_index;
      }
    }
  }
  return ret_val;
}

bool ProfileConstraints::Valid(const size_t boundary_index) const {
  const auto& target_boundary = boundaries[boundary_index];
  if (!target_boundary.BoundaryValid())
    return false;

  for (int test_boundary_index = 0; test_boundary_index < boundaries.size() - 1; test_boundary_index++) {
    const auto& test_boundary = boundaries.at(test_boundary_index);
    for (int new_point_index = 0; new_point_index < target_boundary.points.size(); new_point_index++) {
      const auto& p1 = target_boundary.points[new_point_index];
      const auto& p2 = target_boundary.points[(new_point_index + 1) % target_boundary.points.size()];
      for (int test_point_index = 0; test_point_index < test_boundary.points.size(); test_point_index++) {
        const auto& p3 = test_boundary.points[test_point_index];
        const auto& p4 = test_boundary.points[(test_point_index + 1) % test_boundary.points.size()];
        if (ProfileBoundary::Intersect(p1, p2, p3, p4))
          return false;
      }
    }
  }

  for (int test_boundary_index = 0; test_boundary_index < boundaries.size() - 1; test_boundary_index++) {
    const auto& test_boundary = boundaries.at(test_boundary_index);
    for (const auto& new_point : target_boundary.points) {
      if (test_boundary.InBoundary(new_point))
        return false;
    }
  }
  return true;
}

glm::vec2 ProfileConstraints::GetTarget(const glm::vec2& position) const {
  glm::vec2 closest_point;
  // 1. If in any of the boundary
  int boundary_index = -1;
  auto distance_to_closest_point_on_boundary = FLT_MAX;
  auto distance_to_closest_boundary_center = FLT_MAX;
  for (int current_boundary_index = 0; current_boundary_index < boundaries.size(); current_boundary_index++) {
    const auto& boundary = boundaries.at(current_boundary_index);
    glm::vec2 current_closest_point;
    const auto current_in_boundary = boundary.InBoundary(position, current_closest_point);
    const auto current_distance = glm::distance(current_closest_point, position);
    if (current_in_boundary) {
      if (const auto current_distance_to_boundary_center = glm::distance(position, boundary.center);
          current_distance_to_boundary_center < distance_to_closest_boundary_center) {
        distance_to_closest_boundary_center = current_distance_to_boundary_center;
        boundary_index = current_boundary_index;
      }
    } else if (current_distance < distance_to_closest_point_on_boundary) {
      distance_to_closest_point_on_boundary = current_distance;
      closest_point = current_closest_point;
    }
  }
  if (boundary_index != -1) {
    if (attractors.empty()) {
      closest_point = boundaries.at(boundary_index).center;
    } else {
      auto distance_to_attractor = FLT_MAX;
      for (const auto& attractor : attractors) {
        const auto current_closest_point = attractor.FindClosestPoint(position);
        if (const float current_distance_to_attractor = glm::distance(current_closest_point, position);
            current_distance_to_attractor < distance_to_attractor) {
          distance_to_attractor = current_distance_to_attractor;
          closest_point = current_closest_point;
        }
      }
    }
  } else if (boundaries.empty() && !attractors.empty()) {
    auto distance_to_attractor = FLT_MAX;
    for (const auto& attractor : attractors) {
      const auto current_closest_point = attractor.FindClosestPoint(position);
      if (const float current_distance_to_attractor = glm::distance(current_closest_point, position);
          current_distance_to_attractor < distance_to_attractor) {
        distance_to_attractor = current_distance_to_attractor;
        closest_point = current_closest_point;
      }
    }
  }
  return closest_point - position;
}