#include "DsAlphaShapeUtils.hpp"
#include <algorithm>

using namespace eco_sys_lab_plugin;

float DsAlphaShapeUtils::PointPlaneDistance(const glm::vec3& target_point, const glm::vec3& target_a,
                                            const glm::vec3& target_b, const glm::vec3& target_c) {
  // Compute the normal of the triangle
  const glm::vec3 ab = target_b - target_a;
  const glm::vec3 ac = target_c - target_a;
  const glm::vec3 normal = glm::normalize(glm::cross(ab, ac));

  // Compute signed distance from point to triangle's plane
  const float distance = glm::dot(normal, target_point - target_a);

  return distance;
}

/// @brief Given two arrays of size 4, each containing one element that is not in
/// the other, compute the respective indices of these elements in the arrays
/// @param a index array of size 4
/// @param b index array of size 4
/// @return pair of indices:
/// first indicates the position in a that does not occur in b,
/// second indicates the position in b that does not occur in a.
std::pair<int, int> DsAlphaShapeUtils::CompareIndices(const int a[4], const int b[4]) {
  std::vector a_in_both(4, false);
  std::vector b_in_both(4, false);

  for (size_t i = 0; i < 4; i++) {
    for (size_t j = 0; j < 4; j++) {
      if (a[i] == b[j]) {
        a_in_both[i] = true;
        b_in_both[j] = true;
      }
    }
  }

  int a_not_in_b = -1, b_not_in_a = -1;

  for (int i = 0; i < 4; i++) {
    if (!a_in_both[i]) {
      a_not_in_b = i;
    }
    if (!b_in_both[i]) {
      b_not_in_a = i;
    }
  }

  if (a_not_in_b >= 4 || b_not_in_a >= 4) {
    return std::make_pair(a_not_in_b, b_not_in_a);
    // throw std::exception("Did not find mismatched indices!");
  }

  return std::make_pair(a_not_in_b, b_not_in_a);
}

bool DsAlphaShapeUtils::IsBetweenPlanes(const int target_indices[4],
                                        std::vector<DsAlphaShapeMeshing::GpuUniformParticle>& particles) {
  int max_difference = -1;

  for (size_t i = 0; i < 4; i++) {
    for (size_t j = i + 1; j < 4; j++) {
      int diff = glm::abs(particles[target_indices[i]].segment_index - particles[target_indices[j]].segment_index);
      if (diff > max_difference) {
        max_difference = diff;
      }
    }
  }
  // return max_difference == 1;
  return max_difference <= 1;  // for now also permit same distance
}

bool DsAlphaShapeUtils::IsValid(const int target_indices[4], int size) {
  for (size_t i = 0; i < 4; i++) {
    if (static_cast<unsigned>(target_indices[i]) >= size) {
      // EVOENGINE_ERROR("tetrahedron vertex index out of range, will be discarded: " << target_indices[i]);
      return false;
    }
  }

  // check if all indices are distinct
  for (size_t i = 0; i < 4; i++) {
    for (size_t j = i + 1; j < 4; j++) {
      if (target_indices[i] == target_indices[j]) {
        return false;
      }
    }
  }

  return true;
}

glm::vec3 DsAlphaShapeUtils::CubicHermiteSpline(const glm::vec3& P0, const glm::vec3& P1, const glm::vec3& M0,
                                                const glm::vec3& M1, float t) {
  float t2 = t * t;
  float t3 = t2 * t;

  float h00 = 2.0 * t3 - 3.0 * t2 + 1.0;
  float h10 = t3 - 2.0 * t2 + t;
  float h01 = -2.0 * t3 + 3.0 * t2;
  float h11 = t3 - t2;

  return h00 * P0 + h10 * M0 + h01 * P1 + h11 * M1;
}

glm::vec3 DsAlphaShapeUtils::CubicHermiteSplineTangent(const glm::vec3& P0, const glm::vec3& P1, const glm::vec3& M0,
                                                       const glm::vec3& M1, float t) {
  float t2 = t * t;

  float h00 = 6.0 * t2 - 6.0 * t;
  float h10 = 3.0 * t2 - 4.0 * t + 1.0;
  float h01 = -6.0 * t2 + 6.0 * t;
  float h11 = 3.0 * t2 - 2.0 * t;

  return h00 * P0 + h10 * M0 + h01 * P1 + h11 * M1;
}

std::vector<std::map<int, std::vector<size_t>>> DsAlphaShapeUtils::ComputeBundleMaps(
    std::vector<DsAlphaShapeMeshing::GpuUniformParticle>& uniform_particles) {
  int max_dist_from_root = 0;

  for (int i = 0; i < uniform_particles.size(); i++) {
    max_dist_from_root = std::max(max_dist_from_root, uniform_particles[i].segment_index);
  }

  std::vector<std::map<int, std::vector<size_t>>> bundle_maps(max_dist_from_root + 1);
  std::vector<size_t> offsets(max_dist_from_root + 1, 0);
  std::vector<std::vector<size_t>> particle_adjacent_tets(uniform_particles.size(), std::vector<size_t>{});

  for (int i = 0; i < uniform_particles.size(); i++) {
    auto& particle = uniform_particles[i];
    auto& node_handle = particle.node_index;

    if (bundle_maps[particle.segment_index].find(node_handle) == bundle_maps[particle.segment_index].end()) {
      bundle_maps[particle.segment_index][node_handle] = std::vector<size_t>();
    }

    bundle_maps[particle.segment_index][node_handle].push_back(i);
  }

  return bundle_maps;
}
