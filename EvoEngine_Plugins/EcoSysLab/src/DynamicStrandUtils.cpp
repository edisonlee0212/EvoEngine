#include "DynamicStrandUtils.hpp"

using namespace eco_sys_lab_plugin;

float DynamicStrandUtils::PointPlaneDistance(const glm::vec3& target_point, const glm::vec3& target_a,
                                             const glm::vec3& target_b, const glm::vec3& target_c) {
  // Compute the normal of the triangle
  const glm::vec3 ab = target_b - target_a;
  const glm::vec3 ac = target_c - target_a;
  const glm::vec3 normal = glm::normalize(glm::cross(ab, ac));

  // Compute signed distance from point to triangle's plane
  const float distance = glm::dot(normal, target_point - target_a);

  return distance;
};

/// @brief Given two arrays of size 4, each containing one element that is not in
/// the other, compute the respective indices of these elements in the arrays
/// @param a index array of size 4
/// @param b index array of size 4
/// @return pair of indices:
/// first indicates the position in a that does not occur in b,
/// second indicates the position in b that does not occur in a.
std::pair<int, int> DynamicStrandUtils::CompareIndices(const int a[4], const int b[4]) {
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
};

bool DynamicStrandUtils::IsBetweenPlanes(const int target_indices[4],
                                         std::vector<DynamicStrands::GpuUniformParticle>& particles) {
  int max_difference = -1;

  for (size_t i = 0; i < 4; i++) {
    for (size_t j = i + 1; j < 4; j++) {
      int diff = glm::abs(particles[target_indices[i]].segment_index - particles[target_indices[j]].segment_index);
      if (diff > max_difference) {
        max_difference = diff;
      }
    }
  }
  return max_difference == 1;
  // return max_difference <= 1;  // for now also permit same distance
}

bool DynamicStrandUtils::IsValid(const int target_indices[4], int size) {
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
};