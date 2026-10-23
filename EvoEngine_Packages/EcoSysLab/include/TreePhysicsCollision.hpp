#pragma once

#include <cmath>
#include <glm/gtx/quaternion.hpp>

namespace eco_sys_lab_package {
inline glm::vec3 ProjectOutsideBox(glm::vec3 point, const glm::vec3& center, const glm::quat& rotation,
                                   const glm::vec3& half_extent, const float radius) {
  glm::vec3 local = glm::inverse(rotation) * (point - center);
  const glm::vec3 delta = local - glm::clamp(local, -half_extent, half_extent);
  const float distance_squared = glm::dot(delta, delta);
  if (distance_squared >= radius * radius)
    return point;
  if (distance_squared > 1e-12f) {
    local += delta * (radius / std::sqrt(distance_squared) - 1.0f);
  } else {
    const glm::vec3 depth = half_extent - glm::abs(local);
    const int axis = depth.x < depth.y ? (depth.x < depth.z ? 0 : 2) : (depth.y < depth.z ? 1 : 2);
    local[axis] = (local[axis] < 0 ? -1.0f : 1.0f) * (half_extent[axis] + radius);
  }
  return center + rotation * local;
}

inline glm::vec3 ProjectOutsideSphere(const glm::vec3& point, const glm::vec3& center, const float sphere_radius,
                                      const float rod_radius) {
  const glm::vec3 delta = point - center;
  const float distance_squared = glm::dot(delta, delta);
  const float limit = sphere_radius + rod_radius;
  if (distance_squared >= limit * limit)
    return point;
  if (distance_squared < 1e-12f)
    return center + glm::vec3(0, limit, 0);
  return center + delta * (limit / std::sqrt(distance_squared));
}

inline glm::vec3 ProjectOutsideCylinder(const glm::vec3& point, const glm::vec3& center, const glm::quat& rotation,
                                        const float cylinder_radius, const float half_height, const float rod_radius) {
  glm::vec3 local = glm::inverse(rotation) * (point - center);
  const float radial = std::sqrt(local.x * local.x + local.z * local.z);
  const glm::vec2 direction = radial > 1e-6f ? glm::vec2(local.x, local.z) / radial : glm::vec2(0, 1);
  if (radial <= cylinder_radius && std::abs(local.y) <= half_height) {
    if (cylinder_radius - radial <= half_height - std::abs(local.y)) {
      local.x = direction.x * (cylinder_radius + rod_radius);
      local.z = direction.y * (cylinder_radius + rod_radius);
    } else {
      local.y = (local.y < 0 ? -1.0f : 1.0f) * (half_height + rod_radius);
    }
  } else {
    const glm::vec3 closest(direction.x * glm::min(radial, cylinder_radius),
                            glm::clamp(local.y, -half_height, half_height),
                            direction.y * glm::min(radial, cylinder_radius));
    const glm::vec3 delta = local - closest;
    const float distance_squared = glm::dot(delta, delta);
    if (distance_squared >= rod_radius * rod_radius)
      return point;
    local = closest + delta * (rod_radius / std::sqrt(distance_squared));
  }
  return center + rotation * local;
}
}  // namespace eco_sys_lab_package
