
#include "IVolume.hpp"

#include <Jobs.hpp>

using namespace eco_sys_lab;

bool IVolume::InVolume(const GlobalTransform& global_transform, const glm::vec3& position) {
  return false;
}

bool IVolume::InVolume(const glm::vec3& position) {
  return false;
}

void IVolume::InVolume(const GlobalTransform& global_transform, const std::vector<glm::vec3>& positions,
                       std::vector<bool>& results) {
  results.resize(positions.size());
  Jobs::RunParallelFor(positions.size(), [&](unsigned i) {
    results[i] = InVolume(global_transform, positions[i]);
  });
}

void IVolume::InVolume(const std::vector<glm::vec3>& positions, std::vector<bool>& results) {
  results.resize(positions.size());
  Jobs::RunParallelFor(positions.size(), [&](unsigned i) {
    results[i] = InVolume(positions[i]);
  });
}

glm::vec3 SphericalVolume::GetRandomPoint() {
  return glm::ballRand(1.0f) * m_radius;
}
bool SphericalVolume::InVolume(const GlobalTransform& global_transform, const glm::vec3& position) {
  return false;
}
bool SphericalVolume::InVolume(const glm::vec3& position) {
  const auto relative_position = glm::vec3(position.x / m_radius.x, position.y / m_radius.y, position.z / m_radius.z);
  return glm::length(relative_position) <= 1.0f;
}
