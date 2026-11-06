#pragma once

#include <cmath>
#include <cstdint>
#include <random>
#include <vector>

#include <glm/glm.hpp>

namespace evo_engine {
struct IlluminationVoxelPoint {
  glm::ivec3 coordinate = glm::ivec3(0);
  glm::vec3 position = glm::vec3(0.0f);
};

struct IlluminationVoxel {
  glm::ivec3 coordinate = glm::ivec3(0);
  glm::vec3 center = glm::vec3(0.0f);
  glm::vec3 energy = glm::vec3(0.0f);
  uint32_t sample_count = 0;

  [[nodiscard]] float GetIntensity() const {
    return glm::length(energy);
  }
};

inline bool IsValidIlluminationVoxelGrid(const glm::vec3& volume_size, const glm::ivec3& resolution,
                                         const uint32_t samples_per_voxel) {
  return glm::all(glm::greaterThan(volume_size, glm::vec3(0.0f))) && std::isfinite(volume_size.x) &&
         std::isfinite(volume_size.y) && std::isfinite(volume_size.z) &&
         glm::all(glm::greaterThan(resolution, glm::ivec3(0))) && samples_per_voxel > 0;
}

inline std::vector<IlluminationVoxelPoint> GenerateIlluminationVoxelPoints(const glm::vec3& volume_size,
                                                                           const glm::ivec3& resolution,
                                                                           const uint32_t samples_per_voxel,
                                                                           const uint32_t seed) {
  if (!IsValidIlluminationVoxelGrid(volume_size, resolution, samples_per_voxel))
    return {};

  const auto voxel_count = static_cast<size_t>(resolution.x) * resolution.y * resolution.z;
  std::vector<IlluminationVoxelPoint> points;
  points.reserve(voxel_count * samples_per_voxel);
  const glm::vec3 voxel_size = volume_size / glm::vec3(resolution);
  const glm::vec3 minimum = -volume_size * 0.5f;
  std::mt19937 generator(seed);
  std::uniform_real_distribution<float> distribution(0.0f, 1.0f);
  for (int x = 0; x < resolution.x; x++) {
    for (int y = 0; y < resolution.y; y++) {
      for (int z = 0; z < resolution.z; z++) {
        const glm::ivec3 coordinate(x, y, z);
        const glm::vec3 voxel_minimum = minimum + glm::vec3(coordinate) * voxel_size;
        for (uint32_t sample = 0; sample < samples_per_voxel; sample++) {
          const glm::vec3 offset(distribution(generator), distribution(generator), distribution(generator));
          points.push_back({coordinate, voxel_minimum + offset * voxel_size});
        }
      }
    }
  }
  return points;
}

inline std::vector<IlluminationVoxel> AggregateIlluminationVoxels(const std::vector<IlluminationVoxelPoint>& points,
                                                                  const std::vector<glm::vec3>& energies,
                                                                  const glm::vec3& volume_size,
                                                                  const glm::ivec3& resolution) {
  if (points.size() != energies.size() || !IsValidIlluminationVoxelGrid(volume_size, resolution, 1))
    return {};

  const auto voxel_count = static_cast<size_t>(resolution.x) * resolution.y * resolution.z;
  std::vector<IlluminationVoxel> voxels(voxel_count);
  const glm::vec3 voxel_size = volume_size / glm::vec3(resolution);
  const glm::vec3 minimum = -volume_size * 0.5f;
  for (int x = 0; x < resolution.x; x++) {
    for (int y = 0; y < resolution.y; y++) {
      for (int z = 0; z < resolution.z; z++) {
        const glm::ivec3 coordinate(x, y, z);
        const size_t index = (static_cast<size_t>(x) * resolution.y + y) * resolution.z + z;
        voxels[index].coordinate = coordinate;
        voxels[index].center = minimum + (glm::vec3(coordinate) + 0.5f) * voxel_size;
      }
    }
  }
  for (size_t i = 0; i < points.size(); i++) {
    const auto coordinate = points[i].coordinate;
    if (glm::any(glm::lessThan(coordinate, glm::ivec3(0))) || glm::any(glm::greaterThanEqual(coordinate, resolution)))
      continue;
    const size_t index =
        (static_cast<size_t>(coordinate.x) * resolution.y + coordinate.y) * resolution.z + coordinate.z;
    voxels[index].energy += energies[i];
    voxels[index].sample_count++;
  }
  for (auto& voxel : voxels) {
    if (voxel.sample_count > 0)
      voxel.energy /= static_cast<float>(voxel.sample_count);
  }
  return voxels;
}
}  // namespace evo_engine
