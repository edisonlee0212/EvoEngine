#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>

#ifdef __CUDACC__
#  define EVOENGINE_SUN_HOST_DEVICE __host__ __device__ __forceinline__
#else
#  define EVOENGINE_SUN_HOST_DEVICE inline
#endif

namespace evo_engine {
EVOENGINE_SUN_HOST_DEVICE glm::mat3 SunTangentSpace(const glm::vec3 &direction) {
  const glm::vec3 normal = glm::normalize(direction);
  const glm::vec3 helper = glm::abs(normal.x) > 0.99f ? glm::vec3(0.0f, 0.0f, 1.0f) : glm::vec3(1.0f, 0.0f, 0.0f);
  const glm::vec3 tangent = glm::normalize(glm::cross(normal, helper));
  return glm::mat3(tangent, glm::normalize(glm::cross(normal, tangent)), normal);
}

EVOENGINE_SUN_HOST_DEVICE glm::vec3 SampleSunDirection(const glm::vec3 &sun_direction,
                                                       const float angular_diameter_radians, const float radial_sample,
                                                       const float azimuth_sample) {
  const glm::vec3 axis = glm::normalize(sun_direction);
  if (angular_diameter_radians <= 0.0f)
    return axis;
  const float cos_theta_max = glm::cos(glm::min(angular_diameter_radians, glm::pi<float>()) * 0.5f);
  const float cos_theta = 1.0f - glm::clamp(radial_sample, 0.0f, 1.0f) * (1.0f - cos_theta_max);
  const float sin_theta = glm::sqrt(glm::max(0.0f, 1.0f - cos_theta * cos_theta));
  const float phi = 2.0f * glm::pi<float>() * glm::clamp(azimuth_sample, 0.0f, 1.0f);
  return SunTangentSpace(axis) * glm::vec3(glm::cos(phi) * sin_theta, glm::sin(phi) * sin_theta, cos_theta);
}
}  // namespace evo_engine

#undef EVOENGINE_SUN_HOST_DEVICE
