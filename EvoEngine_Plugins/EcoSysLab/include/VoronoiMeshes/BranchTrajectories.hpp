#pragma once
#include <vector>

#include "PlaneProjector.hpp"
#include "Polynomial.hpp"
#include "VoronoiPoint.hpp"

namespace kinDS {

/**
 * This class handles the trajectories of strands according to branches, allowing to easily get points in a different
 * frame of reference as needed
 */
class BranchTrajectories {
 private:
  std::vector<std::vector<VoronoiPoint<2>>> support_points;
  std::vector<std::vector<glm::mat4>> transforms_by_height_and_branch;

  // Create a branch index lookup using [strand_id][h]
  std::vector<std::vector<size_t>> branch_indices;
  std::vector<std::vector<std::vector<size_t>>> strands_by_branch_id;
  size_t height = 0;

 public:
  BranchTrajectories(const std::vector<std::vector<VoronoiPoint<2>>>& support_points,
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

  const std::vector<std::vector<VoronoiPoint<2>>>& getPoints() const {
    return support_points;
  }

  size_t getHeight() const {
    return height;
  }

  size_t addTrajectory(const std::vector<kinDS::VoronoiPoint<2>>& traj) {
    size_t index = support_points.size();
    support_points.push_back(traj);
    return index;
  }

  VoronoiPoint<2> evaluate(size_t strand_id, double t) const {
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

    const VoronoiPoint<2>& lower = support_points[strand_id][lower_index];
    const VoronoiPoint<2>& upper = support_points[strand_id][upper_index];

    return lower * (1.0 - frac) + upper * frac;
  }

  VoronoiPoint<2> getPointTransformed(size_t strand_id, size_t index, size_t reference_branch) const {
    size_t actual_branch;
    // dummy strands might not be mapped to a branch:
    if (strand_id >= branch_indices.size()) {
      actual_branch = reference_branch;
    } else {
      actual_branch = branch_indices[strand_id][index];
    }

    VoronoiPoint<2> point = support_points[strand_id][index];
    if (actual_branch == reference_branch) {
      return point;
    }

    PlaneProjector plane_projector(transforms_by_height_and_branch[index][actual_branch],
                                   transforms_by_height_and_branch[index][reference_branch]);

    auto result = plane_projector.project(glm::vec2(point[0], point[1]));
    return VoronoiPoint<2>{result.x, result.y};
  }

  VoronoiPoint<2> evaluateTransformed(size_t strand_id, double t, size_t reference_branch) const {
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

    const VoronoiPoint<2>& lower = getPointTransformed(strand_id, lower_index, reference_branch);
    const VoronoiPoint<2>& upper = getPointTransformed(strand_id, upper_index, reference_branch);

    return lower * (1.0 - frac) + upper * frac;
  }

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