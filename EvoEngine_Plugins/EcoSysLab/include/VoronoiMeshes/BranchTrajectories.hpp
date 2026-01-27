#pragma once
#include <vector>

#include "PlaneProjector.hpp"
#include "Polynomial.hpp"

namespace kinDS {

template <size_t dim>
using Trajectory = std::array<Polynomial, dim>;

static glm::dvec3 ProfileToModelCoordinatesBranch(
    const std::vector<std::vector<glm::mat4>>& profile_to_model_transforms, glm::dvec3 point, float t,
    const std::vector<size_t>& branch_indices, float w = 1.0f) {
  size_t lower_section_index = static_cast<size_t>(std::max(0.0f, glm::floor(t)));

  size_t upper_section_index = std::min(profile_to_model_transforms.size() - 1, static_cast<size_t>(glm::ceil(t)));

  // check range
  auto coord_str = std::to_string(t);
  if (lower_section_index >= profile_to_model_transforms.size()) {
    std::cout << ("ProfileToModelCoordinates: lower bound of point z-coordinate out of range: " + coord_str).c_str()
              << std::endl;
  }
  if (upper_section_index >= profile_to_model_transforms.size()) {
    std::cout << ("ProfileToModelCoordinates: upper bound of point z-coordinate out of range: " + coord_str).c_str()
              << std::endl;
  }

  // only set second coordinate to 0 for points, not for normal vectors
  // TODO: I actually wanted to get rid of this coordinate swap at some point
  glm::vec4 local_pos(point[0], (1.0f - w) * point[2], point[1], w);
  if (lower_section_index >= branch_indices.size()) {
    std::cout << ("ProfileToModelCoordinates: lower bound of point z-coordinate out of range: " + coord_str).c_str()
              << ", index: " << lower_section_index << ", size: " << branch_indices.size() << std::endl;
  }
  size_t lower_branch_index = branch_indices[lower_section_index];
  glm::vec4 global_pos = profile_to_model_transforms[lower_section_index][lower_branch_index] * local_pos;

  if (upper_section_index != lower_section_index) {
    size_t upper_branch_index = branch_indices[upper_section_index];
    glm::vec4 upper_global_pos = profile_to_model_transforms[upper_section_index][upper_branch_index] * local_pos;
    double frac = t - static_cast<double>(lower_section_index);
    global_pos = glm::mix(global_pos, upper_global_pos, frac);
  }

  if (w == 0.0f) {
    global_pos = glm::normalize(global_pos);
  }

  return {global_pos.x, global_pos.y, global_pos.z};
}

/**
 * This class handles the trajectories of strands according to branches, allowing to easily get points in a different
 * frame of reference as needed
 */
class BranchTrajectories {
 private:
  std::vector<std::vector<glm::dvec2>> support_points;
  std::vector<std::vector<glm::mat4>> transforms_by_height_and_branch;

  // Create a branch index lookup using [strand_id][h]
  std::vector<std::vector<size_t>> branch_indices;
  std::vector<std::vector<std::vector<size_t>>> strands_by_branch_id;
  size_t height = 0;

 public:
  BranchTrajectories(const std::vector<std::vector<glm::dvec2>>& support_points,
                     const std::vector<std::vector<glm::mat4>>& transforms_by_height_and_branch,
                     const std::vector<std::vector<size_t>>& branch_indices,
                     const std::vector<std::vector<std::vector<size_t>>>& strands_by_branch_id)
      : support_points(support_points),
        transforms_by_height_and_branch(transforms_by_height_and_branch),
        branch_indices(branch_indices),
        strands_by_branch_id(strands_by_branch_id) {
    // compute height
    for (const auto& pts : support_points) {
      // always one less than size because the parameter is in range from smallest to largest index
      height = std::max(height, pts.size() - 1);
    }
  }

  const std::vector<std::vector<glm::dvec2>>& getPoints() const {
    return support_points;
  }

  size_t getHeight() const {
    return height;
  }

  size_t addTrajectory(const std::vector<glm::dvec2>& traj) {
    size_t index = support_points.size();
    support_points.push_back(traj);
    return index;
  }

  glm::dvec2 evaluate(size_t strand_id, double t) const {
    if (t < 0) {
      throw std::runtime_error("Parameter t out of bounds");
    }

    size_t lower_index = std::floor(t);
    size_t upper_index = lower_index + 1;
    double frac = t - lower_index;

    if (upper_index >= support_points[strand_id].size() && lower_index < support_points[strand_id].size() &&
        frac < std::numeric_limits<double>::epsilon()) {
      return support_points[strand_id].back();
    }

    if (upper_index >= support_points[strand_id].size()) {
      throw std::runtime_error("Parameter t out of bounds");
    }

    const glm::dvec2& lower = support_points[strand_id][lower_index];
    const glm::dvec2& upper = support_points[strand_id][upper_index];

    return lower * (1.0 - frac) + upper * frac;
  }

  glm::dvec2 getPointTransformed(size_t strand_id, size_t index, size_t reference_branch) const {
    size_t actual_branch;
    // dummy strands might not be mapped to a branch:
    if (strand_id >= branch_indices.size()) {
      actual_branch = reference_branch;
    } else {
      actual_branch = branch_indices[strand_id][index];
    }

    glm::dvec2 point = support_points[strand_id][index];
    if (actual_branch == reference_branch) {
      return point;
    }

    PlaneProjector plane_projector(transforms_by_height_and_branch[index][actual_branch],
                                   transforms_by_height_and_branch[index][reference_branch]);

    auto result = plane_projector.project(glm::vec2(point[0], point[1]));
    return glm::dvec2{result.x, result.y};
  }

  glm::dvec2 evaluateTransformed(size_t strand_id, double t, size_t reference_branch) const {
    if (t < 0) {
      throw std::runtime_error("Parameter t out of bounds");
    }

    size_t lower_index = std::floor(t);
    size_t upper_index = lower_index + 1;
    double frac = t - lower_index;

    if (upper_index >= support_points[strand_id].size() && lower_index < support_points[strand_id].size() &&
        frac < std::numeric_limits<double>::epsilon()) {
      return getPointTransformed(strand_id, support_points[strand_id].size() - 1, reference_branch);
    }

    if (upper_index >= support_points[strand_id].size()) {
      throw std::runtime_error("Parameter t out of bounds");
    }

    const glm::dvec2& lower = getPointTransformed(strand_id, lower_index, reference_branch);
    const glm::dvec2& upper = getPointTransformed(strand_id, upper_index, reference_branch);

    return lower * (1.0 - frac) + upper * frac;
  }

  glm::dvec3 getPointInObjectSpace(size_t strand_id, double t) const {
    glm::dvec2 v = evaluate(strand_id, t);

    glm::dvec3 v_3d{v[0], 0.0, v[1]};
    return ProfileToModelCoordinatesBranch(transforms_by_height_and_branch, v_3d, t, branch_indices[strand_id]);
  }

  glm::dvec3 transformToObjectSpace(glm::dvec3& v_3d, size_t strand_id, double t) const {
    return ProfileToModelCoordinatesBranch(transforms_by_height_and_branch, v_3d, t, branch_indices[strand_id]);
  }

  // TODO: also adjust to different reference frame
  std::array<Polynomial, 2> getPiecePolynomial(size_t strand_id, size_t index) const {
    if (strand_id >= support_points.size()) {
      throw std::out_of_range("Strand id " + std::to_string(strand_id) + " out of range.");
    }

    if (index >= support_points[strand_id].size() - 1) {
      throw std::out_of_range("Index " + std::to_string(index) + " out of range for piece polynomial.");
    }
    const auto& P0 = support_points[strand_id][index];
    const auto& P1 = support_points[strand_id][index + 1];

    std::array<Polynomial, 2> result;
    // Create linear polynomials for each dimension
    for (int i = 0; i < 2; ++i) {
      result[i] = POLYNOMIAL(P0[i] + (P1[i] - P0[i]) * x);
    }

    return result;
  }
};
};  // namespace kinDS