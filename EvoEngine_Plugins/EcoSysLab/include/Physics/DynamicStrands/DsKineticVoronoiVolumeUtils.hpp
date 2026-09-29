#pragma once

#include "DsKineticVoronoiMeshing.hpp"

#include <unordered_map>
#include <vector>

namespace eco_sys_lab_plugin {

/**
 * \brief CPU volume helpers for Kinetic Voronoi segment meshlets after a GPU download.
 * Does not touch GPU state; callers should Download() (or EnsureCpuMeshletsFromGpu for rest-pose) first.
 */
class DsKineticVoronoiVolumeUtils {
 public:
  struct MeshClosedness {
    bool is_closed = false;
    /// Undirected boundary edges (multiplicity != 2). Zero iff closed manifold skin.
    size_t boundary_edge_count = 0;
    /// Edges with multiplicity > 2 (non-manifold).
    size_t non_manifold_edge_count = 0;
  };

  struct MeshletVolume {
    unsigned int segment_index = 0;
    double volume = 0.0;
    bool is_closed = false;
    /// True when the robust centroid-fan estimate was used instead of the divergence formula.
    bool used_fallback = false;
  };

  struct VolumeResult {
    std::vector<MeshletVolume> meshlets;
    /// Sum of per-meshlet volumes (non-negative).
    double cumulative_volume = 0.0;
    size_t closed_meshlet_count = 0;
    size_t open_meshlet_count = 0;
  };

  /// Group triangle indices by owning physics segment (`vertices[tri.vertex_index0].segment_index`).
  static std::unordered_map<unsigned int, std::vector<size_t>> TriangleIndicesBySegment(
      const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletVertex>& vertices,
      const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletTriangle>& triangles);

  /// Closed manifold skin: every undirected edge appears exactly twice.
  static MeshClosedness DiagnoseClosedness(
      const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletVertex>& vertices,
      const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletTriangle>& triangles,
      const std::vector<size_t>& triangle_indices);

  /**
   * \brief Volume of one meshlet (one physics segment's triangles).
   * If closed: divergence theorem (1/6) Σ p0·(p1×p2), absolute value.
   * Otherwise: robust centroid-fan estimate (sum of |tet(centroid, tri)| / 6).
   * \param use_current_x true → deformed `x`, false → rest `x0`.
   */
  static double ComputeMeshletVolume(
      const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletVertex>& vertices,
      const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletTriangle>& triangles,
      const std::vector<size_t>& triangle_indices, bool use_current_x = true, bool* used_fallback = nullptr,
      MeshClosedness* closedness_out = nullptr);

  /// Per-meshlet volumes for every segment present in the buffers, plus cumulative total.
  static VolumeResult ComputeAllMeshletVolumes(
      const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletVertex>& vertices,
      const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletTriangle>& triangles, bool use_current_x = true);
};

}  // namespace eco_sys_lab_plugin
