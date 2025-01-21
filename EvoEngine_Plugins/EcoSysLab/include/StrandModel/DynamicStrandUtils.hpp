#pragma once

#include "DynamicStrands.hpp"

namespace eco_sys_lab_plugin {
class DynamicStrandUtils {
 public:
  static float PointPlaneDistance(const glm::vec3& target_point, const glm::vec3& target_a, const glm::vec3& target_b,
                                  const glm::vec3& target_c);
  static std::pair<int, int> CompareIndices(const int a[4], const int b[4]);
  static bool IsBetweenPlanes(const int target_indices[4], std::vector<DynamicStrands::GpuUniformParticle>& particles);
  static bool IsValid(const int target_indices[4], int size);
  static void AlphaComplex(std::vector<DynamicStrands::GpuDelaunayTetrahedron>& delaunay_triangulation,
                                      std::function<bool(DynamicStrands::GpuDelaunayTetrahedron&)> is_inside);
  static void FillAlphaComplex(std::vector<DynamicStrands::GpuDelaunayTetrahedron>& alpha_complex);
  static void FlagBark(std::vector<DynamicStrands::GpuDelaunayTetrahedron>& alpha_complex);
};
}  // namespace eco_sys_lab_plugin
