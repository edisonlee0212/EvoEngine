#pragma once

#include "DsAlphaShapeMeshing.hpp"

#include <vector>

namespace eco_sys_lab_plugin {

/**
 * \brief CPU volume helpers for Alpha Shape Delaunay tetrahedra after a GPU download.
 * Alive tets are those with `inside == 1`; all others contribute volume 0.
 */
class DsAlphaShapeVolumeUtils {
 public:
  struct VolumeResult {
    /// Parallel to input tetrahedra; 0 for dead / invalid tets.
    std::vector<double> per_tet_volume;
    double cumulative_volume = 0.0;
    size_t alive_count = 0;
    size_t dead_count = 0;
  };

  static bool IsTetrahedronAlive(const DsAlphaShapeMeshing::GpuDelaunayTetrahedron& tet) {
    return tet.inside == 1;
  }

  /// Absolute tet volume |scalar triple| / 6.
  static double TetrahedronVolume(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, const glm::vec3& d);

  /**
   * \brief Per-tet volumes and cumulative solid volume.
   * \param use_current_position true → particle `position`, false → `initial_position`.
   */
  static VolumeResult ComputeTetrahedronVolumes(
      const std::vector<DsAlphaShapeMeshing::GpuUniformParticle>& particles,
      const std::vector<DsAlphaShapeMeshing::GpuDelaunayTetrahedron>& tetrahedrons,
      bool use_current_position = true);
};

}  // namespace eco_sys_lab_plugin
