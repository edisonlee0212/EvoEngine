#pragma once

#include "DsAlphaShapeMeshing.hpp"

#include <cstdint>
#include <vector>

namespace eco_sys_lab_plugin {

/**
 * \brief CPU volume helpers for Alpha Shape Delaunay tetrahedra after a GPU download.
 * Alive tets are those with `inside == 1`; all others contribute volume 0.
 *
 * Near-degeneracy is classified separately at initialization (rest pose) and stored by the
 * meshing owner; simulation-time collapse of a previously valid tet is still measured.
 */
class DsAlphaShapeVolumeUtils {
 public:
  /// Relative volume threshold: volume / max_edge^3. Below this → nearly degenerate.
  static double nearly_degenerate_relative_volume;

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

  /// Absolute tet volume |scalar triple| / 6 (zeros exact degenerates below 1e-18).
  static double TetrahedronVolume(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, const glm::vec3& d);

  /// True when the tet is flat/sliver-like: volume / max_edge^3 <= @ref nearly_degenerate_relative_volume.
  static bool IsNearlyDegenerate(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, const glm::vec3& d);

  /**
   * \brief Mark nearly-degenerate tets at the chosen particle positions (parallel to @p tetrahedrons).
   * Dead / invalid tets are left 0 (not classified as near-degenerate).
   */
  static std::vector<uint8_t> ClassifyNearlyDegenerate(
      const std::vector<DsAlphaShapeMeshing::GpuUniformParticle>& particles,
      const std::vector<DsAlphaShapeMeshing::GpuDelaunayTetrahedron>& tetrahedrons,
      bool use_current_position = false);

  /**
   * \brief Per-tet volumes and cumulative solid volume for all currently alive tets.
   * Does not filter near-degeneracy — callers exclude init-time near-degenerate tets via a stored mask.
   * \param use_current_position true → particle `position`, false → `initial_position`.
   */
  static VolumeResult ComputeTetrahedronVolumes(
      const std::vector<DsAlphaShapeMeshing::GpuUniformParticle>& particles,
      const std::vector<DsAlphaShapeMeshing::GpuDelaunayTetrahedron>& tetrahedrons,
      bool use_current_position = true);
};

}  // namespace eco_sys_lab_plugin
