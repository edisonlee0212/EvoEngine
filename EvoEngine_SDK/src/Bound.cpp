#include "Bound.hpp"
#include "Mesh.hpp"
#include "Transform.hpp"
using namespace evo_engine;

glm::vec3 Bound::Size() const {
  return (max - min) * .5f;
}

glm::vec3 Bound::Center() const {
  return (max + min) * .5f;
}

bool Bound::InBound(const glm::vec3& position) const {
  const glm::vec3 center = (min + max) * .5f;
  const glm::vec3 size = (max - min) * .5f;
  if (glm::abs(position.x - center.x) > size.x)
    return false;
  if (glm::abs(position.y - center.y) > size.y)
    return false;
  if (glm::abs(position.z - center.z) > size.z)
    return false;
  return true;
}

void Bound::ApplyTransform(const glm::mat4& transform) {
  std::vector<glm::vec3> corners;
  PopulateCorners(corners);
  min = glm::vec3(FLT_MAX);
  max = glm::vec3(-FLT_MAX);

  // Transform all the corners, and keep track of the greatest and least
  // values we see on each coordinate axis.
  for (int i = 0; i < 8; i++) {
    glm::vec3 transformed = transform * glm::vec4(corners[i], 1.0f);
    min = glm::min(min, transformed);
    max = glm::max(max, transformed);
  }
}

void Bound::PopulateCorners(std::vector<glm::vec3>& corners) const {
  corners.resize(8);
  corners[0] = min;
  corners[1] = glm::vec3(min.x, min.y, max.z);
  corners[2] = glm::vec3(min.x, max.y, min.z);
  corners[3] = glm::vec3(max.x, min.y, min.z);
  corners[4] = glm::vec3(min.x, max.y, max.z);
  corners[5] = glm::vec3(max.x, min.y, max.z);
  corners[6] = glm::vec3(max.x, max.y, min.z);
  corners[7] = max;
}

Ray::Ray(const glm::vec3& start, const glm::vec3& end) {
  this->start = start;
  this->direction = glm::normalize(end - start);
  this->length = glm::distance(start, end);
}

Ray::Ray(const glm::vec3& start, const glm::vec3& direction, const float length) {
  this->start = start;
  this->direction = direction;
  this->length = length;
}

bool Ray::Intersect(const glm::vec3& position, const float radius) const {
  const glm::vec3 ray_end = start + direction * length;
  const auto cp = glm::closestPointOnLine(position, start, ray_end);
  if (cp == start || cp == ray_end)
    return false;
  return glm::distance(cp, position) <= radius;
}

bool Ray::Intersect(const glm::mat4& transform, const Bound& bound) const {
  float t_min = 0.0f;
  float t_max = 100000.0f;
  GlobalTransform temp;
  temp.value = transform;
  glm::vec3 scale = temp.GetScale();
  temp.SetScale(glm::vec3(1.0f));
  glm::mat4 model = temp.value;

  glm::vec3 obb_world_space(model[3].x, model[3].y, model[3].z);

  glm::vec3 delta = obb_world_space - start;
  glm::vec3 aabb_min = scale * (bound.min);
  glm::vec3 aabb_max = scale * (bound.max);
  // Test intersection with the 2 planes perpendicular to the OBB's X axis
  {
    glm::vec3 x_axis(model[0].x, model[0].y, model[0].z);

    float e = glm::dot(x_axis, delta);

    if (float f = glm::dot(direction, x_axis); fabs(f) > 0.001f) {  // Standard case

      float t1 = (e + aabb_min.x) / f;  // Intersection with the "left" plane
      float t2 = (e + aabb_max.x) / f;  // Intersection with the "right" plane
      // t1 and t2 now contain distances between ray origin and ray-plane intersections

      // We want t1 to represent the nearest intersection,
      // so if it's not the case, invert t1 and t2
      if (t1 > t2) {
        float w = t1;
        t1 = t2;
        t2 = w;  // swap t1 and t2
      }

      // tMax is the nearest "far" intersection (amongst the X,Y and Z planes pairs)
      if (t2 < t_max)
        t_max = t2;
      // tMin is the farthest "near" intersection (amongst the X,Y and Z planes pairs)
      if (t1 > t_min)
        t_min = t1;

      // And here's the trick :
      // If "far" is closer than "near", then there is NO intersection.
      // See the images in the tutorials for the visual explanation.
      if (t_max < t_min)
        return false;
    } else {  // Rare case : the ray is almost parallel to the planes, so they don't have any "intersection"
      if (-e + aabb_min.x > 0.0f || -e + aabb_max.x < 0.0f)
        return false;
    }
  }

  // Test intersection with the 2 planes perpendicular to the obb's Y axis
  // Exactly the same thing than above.
  {
    glm::vec3 y_axis(model[1].x, model[1].y, model[1].z);
    float e = glm::dot(y_axis, delta);

    if (float f = glm::dot(direction, y_axis); fabs(f) > 0.001f) {
      float t1 = (e + aabb_min.y) / f;
      float t2 = (e + aabb_max.y) / f;

      if (t1 > t2) {
        float w = t1;
        t1 = t2;
        t2 = w;
      }

      if (t2 < t_max)
        t_max = t2;
      if (t1 > t_min)
        t_min = t1;
      if (t_min > t_max)
        return false;
    } else {
      if (-e + aabb_min.y > 0.0f || -e + aabb_max.y < 0.0f)
        return false;
    }
  }

  // Test intersection with the 2 planes perpendicular to the OBB's Z axis
  // Exactly the same thing than above.
  {
    glm::vec3 z_axis(model[2].x, model[2].y, model[2].z);
    float e = glm::dot(z_axis, delta);

    if (float f = glm::dot(direction, z_axis); fabs(f) > 0.001f) {
      float t1 = (e + aabb_min.z) / f;
      float t2 = (e + aabb_max.z) / f;

      if (t1 > t2) {
        float w = t1;
        t1 = t2;
        t2 = w;
      }

      if (t2 < t_max)
        t_max = t2;
      if (t1 > t_min)
        t_min = t1;
      if (t_min > t_max)
        return false;
    } else {
      if (-e + aabb_min.z > 0.0f || -e + aabb_max.z < 0.0f)
        return false;
    }
  }
  return true;
}

glm::vec3 Ray::GetEnd() const {
  return start + direction * length;
}

glm::vec3 Ray::ClosestPointOnLine(const glm::vec3& point, const glm::vec3& a, const glm::vec3& b) {
  const float line_length = distance(a, b);
  const glm::vec3 vector = point - a;
  const glm::vec3 line_direction = (b - a) / line_length;

  // Project Vector to LineDirection to get the distance of point from a
  const float distance = dot(vector, line_direction);
  return a + line_direction * distance;
}

Plane::Plane(const glm::vec4& param) {
  a = param.x;
  b = param.y;
  c = param.z;
  d = param.w;
  Normalize();
}

Plane::Plane(const glm::vec3& normal, const float distance) {
  const auto n = glm::normalize(normal);
  a = n.x;
  b = n.y;
  c = n.z;
  d = -(n.x * (n * distance).x + n.y * (n * distance).y + n.z * (n * distance).z);
}

Plane::Plane() : a(0), b(0), c(0), d(0) {
}

void Plane::Normalize() {
  const float mag = glm::sqrt(a * a + b * b + c * c);
  a /= mag;
  b /= mag;
  c /= mag;
  d /= mag;
}

float Plane::CalculateTriangleDistance(const std::vector<Vertex>& vertices, const glm::uvec3& triangle) const {
  const auto& p0 = vertices[triangle.x].position;
  const auto& p1 = vertices[triangle.y].position;
  const auto& p2 = vertices[triangle.z].position;

  const auto centroid = glm::vec3((p0.x + p1.x + p2.x) / 3, (p0.y + p1.y + p2.y) / 3, (p0.z + p1.z + p2.z) / 3);
  return CalculatePointDistance(centroid);
}

float Plane::CalculateTriangleMaxDistance(const std::vector<Vertex>& vertices, const glm::uvec3& triangle) const {
  const auto& p0 = vertices[triangle.x].position;
  const auto& p1 = vertices[triangle.y].position;
  const auto& p2 = vertices[triangle.z].position;

  const auto d0 = CalculatePointDistance(p0);
  const auto d1 = CalculatePointDistance(p1);
  const auto d2 = CalculatePointDistance(p2);

  return glm::max(d0, glm::max(d1, d2));
}

float Plane::CalculatePointDistance(const glm::vec3& point) const {
  return glm::abs(a * point.x + b * point.y + c * point.z + d) / glm::sqrt(a * a + b * b + c * c);
}

float Plane::CalculateTriangleMinDistance(const std::vector<Vertex>& vertices, const glm::uvec3& triangle) const {
  const auto& p0 = vertices[triangle.x].position;
  const auto& p1 = vertices[triangle.y].position;
  const auto& p2 = vertices[triangle.z].position;

  const auto d0 = CalculatePointDistance(p0);
  const auto d1 = CalculatePointDistance(p1);
  const auto d2 = CalculatePointDistance(p2);

  return glm::min(d0, glm::min(d1, d2));
}

glm::vec3 Plane::GetNormal() const {
  return glm::normalize(glm::vec3(a, b, c));
}

float Plane::GetDistance() const {
  return d / glm::length(glm::vec3(a, b, c));
}