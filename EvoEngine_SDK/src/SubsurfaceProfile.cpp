#include "SubsurfaceProfile.hpp"

#include <algorithm>
#include <cmath>

using namespace evo_engine;

float evo_engine::SubsurfaceRadialPdf(const float radius, const float distance) {
  if (radius < 0.0f || distance <= 0.0f)
    return 0.0f;
  return (std::exp(-radius / distance) + std::exp(-radius / (3.0f * distance))) / (4.0f * distance);
}

float evo_engine::SampleSubsurfaceRadius(float u, const float distance) {
  u = std::clamp(u, 0.0f, 1.0f - 1e-7f);
  const float a = std::asinh(2.0f * (1.0f - u)) / 3.0f;
  return -3.0f * distance * std::log(std::min(2.0f * std::sinh(a), 1.0f));
}

glm::vec3 evo_engine::SubsurfaceThinTransmission(const float thickness, const glm::vec3 distance) {
  return glm::exp(-std::max(thickness, 0.0f) / glm::max(distance, glm::vec3(1e-6f)));
}
