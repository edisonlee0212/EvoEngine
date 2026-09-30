// Kinetic Voronoi meshlet volume contributions (per-triangle signed divergence).
// Requires: KineticVoronoiMeshing.glsl, VolumeMeasure.glsl
// Bindings (DYNAMIC_STRANDS_SET):
//   14 = per-segment signed volume scratch (float[], sized to segment count)
//   15 = VolumeMeasureResult (kinetic)
//
// Closed meshlets: V_seg = |sum_tri signed|; cumulative = sum_seg V_seg (finalize pass).

#ifndef KINETIC_VORONOI_VOLUME_MEASURE_GLSL
#define KINETIC_VORONOI_VOLUME_MEASURE_GLSL

layout(std430, set = DYNAMIC_STRANDS_SET, binding = 14) buffer KINETIC_SEGMENT_SIGNED_VOLUME_BLOCK {
  float kinetic_segment_signed_volumes[];
};

layout(std430, set = DYNAMIC_STRANDS_SET, binding = 15) buffer KINETIC_VOLUME_RESULT_BLOCK {
  VolumeMeasureResult kinetic_volume_result;
};

// Signed divergence contribution of triangle `tri_id` using current vertex positions `x`.
// Also returns owning segment index via out parameter.
float MeasureKineticTriangleSignedVolume(uint tri_id, out int segment_index) {
  SegmentMeshletTriangle tri = segment_meshlet_triangles[tri_id];
  uint i0 = tri.vertex_indices[0];
  uint i1 = tri.vertex_indices[1];
  uint i2 = tri.vertex_indices[2];

  SegmentMeshletVertex v0 = segment_meshlet_vertices[i0];
  SegmentMeshletVertex v1 = segment_meshlet_vertices[i1];
  SegmentMeshletVertex v2 = segment_meshlet_vertices[i2];
  segment_index = v0.segment_index;

  vec3 p0 = v0.x;
  vec3 p1 = v1.x;
  vec3 p2 = v2.x;
  if (TriangleAreaNearlyZero(p0, p1, p2)) {
    return 0.0;
  }
  return TriangleSignedVolumeContribution(p0, p1, p2);
}

#endif  // KINETIC_VORONOI_VOLUME_MEASURE_GLSL
