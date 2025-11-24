#pragma once

#include "DsAlphaShapeMeshing.hpp"

namespace eco_sys_lab_plugin {
/**
 * @class DsAlphaShapeUtils
 * @brief A utility class for performing various computations related to dynamic strands.
 */
class DsAlphaShapeUtils {
 public:
  /**
   * @brief Computes the distance from a point to a plane defined by three points.
   * @param target_point The point from which the distance is measured.
   * @param target_a The first point defining the plane.
   * @param target_b The second point defining the plane.
   * @param target_c The third point defining the plane.
   * @return The shortest distance from the target_point to the plane.
   */
  static float PointPlaneDistance(const glm::vec3& target_point, const glm::vec3& target_a, const glm::vec3& target_b,
                                  const glm::vec3& target_c);

  /**
   * @brief Compares two sets of four indices and determines the number of matching indices.
   * @param a The first set of four indices.
   * @param b The second set of four indices.
   * @return A pair of integers representing the number of matching indices and their positions.
   */
  static std::pair<int, int> CompareIndices(const int a[4], const int b[4]);

  /**
   * @brief Checks if a set of indices falls between two planes in a GPU-based strand simulation.
   * @param target_indices The indices of the target strand.
   * @param particles The list of GPU uniform particles used in the simulation.
   * @return True if the strand is between the planes, otherwise false.
   */
  static bool IsBetweenPlanes(const int target_indices[4],
                              std::vector<DsAlphaShapeMeshing::GpuUniformParticle>& particles);

  /**
   * @brief Validates whether a given set of indices is within a valid range.
   * @param target_indices The indices to be checked.
   * @param size The maximum valid size that indices can take.
   * @return True if the indices are valid, otherwise false.
   */
  static bool IsValid(const int target_indices[4], int size);

  static glm::vec3 CubicHermiteSpline(const glm::vec3& P0, const glm::vec3& P1, const glm::vec3& M0,
                                      const glm::vec3& M1, float t);

  static glm::vec3 CubicHermiteSplineTangent(const glm::vec3& P0, const glm::vec3& P1, const glm::vec3& M0,
                                             const glm::vec3& M1, float t);

  /**
   * @brief Computes bundles, i.e. uniform particles which belong to the same branch at the same distance from root.
   * @param uniform_particles The list of uniform particles used for rendering.
   * @return bundle maps, the outer vector is indexed by hop distance from root, each map uses the node handle as key.
   */
  static std::vector<std::map<int, std::vector<size_t>>> ComputeBundleMaps(
      std::vector<DsAlphaShapeMeshing::GpuUniformParticle>& uniform_particles);
};
}  // namespace eco_sys_lab_plugin