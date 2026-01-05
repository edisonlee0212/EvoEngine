// Kinetic Voronoi meshlet volume contributions (per-triangle signed divergence).
// Requires: KineticVoronoiMeshing.glsl, VolumeMeasure.glsl
// Bindings (DYNAMIC_STRANDS_SET):
//   14 = per-segment volumes (signed during measure, abs current after finalize)
//   15 = VolumeMeasureResult (kinetic)
//   18 = initial per-segment volumes (float[])
//
// Closed meshlets: V_seg = |sum_tri signed|; cumulative = sum_seg V_seg (finalize pass).

#ifndef KINETIC_VORONOI_VOLUME_MEASURE_GLSL
#define KINETIC_VORONOI_VOLUME_MEASURE_GLSL

layout(std430, set = DYNAMIC_STRANDS_SET, binding = 14) buffer KINETIC_SEGMENT_VOLUME_BLOCK {
  float kinetic_segment_volumes[];
};

layout(std430, set = DYNAMIC_STRANDS_SET, binding = 15) buffer KINETIC_VOLUME_RESULT_BLOCK {
  VolumeMeasureResult kinetic_volume_result;
};

layout(std430, set = DYNAMIC_STRANDS_SET, binding = 18) readonly buffer KINETIC_SEGMENT_INITIAL_VOLUME_BLOCK {
  float kinetic_segment_initial_volumes[];
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

vec3 KineticSegmentVolumeChangeHeatmapColor(uint segment_index) {
  float initial_volume = kinetic_segment_initial_volumes[segment_index];
  float current_volume = kinetic_segment_volumes[segment_index];
  return VolumeChangeHeatmapColor(initial_volume, current_volume);
}

#endif  // KINETIC_VORONOI_VOLUME_MEASURE_GLSL
