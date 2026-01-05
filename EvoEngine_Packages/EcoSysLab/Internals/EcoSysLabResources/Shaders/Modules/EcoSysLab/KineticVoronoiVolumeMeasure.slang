import EcoSysLab.KineticVoronoiMeshing;
import EcoSysLab.KineticVoronoiMeshingSet0;
import EcoSysLab.VolumeMeasure;

// Bindings (strands set 0):
//   14 = per-segment volumes (signed during measure, abs current after finalize)
//   15 = VolumeMeasureResult (kinetic)
//   18 = initial per-segment volumes (float[])

[[vk::binding(14, 0)]] public RWStructuredBuffer<float, Std430DataLayout> kinetic_segment_volumes;
[[vk::binding(15, 0)]] public RWStructuredBuffer<VolumeMeasureResult, Std430DataLayout> kinetic_volume_result;
[[vk::binding(18, 0)]] public StructuredBuffer<float, Std430DataLayout> kinetic_segment_initial_volumes;

// Signed divergence contribution of triangle `tri_id` using current vertex positions `x`.
public float MeasureKineticTriangleSignedVolume(uint tri_id, out int segment_index) {
  SegmentMeshletTriangle tri = segment_meshlet_triangles[tri_id];
  uint i0 = tri.vertex_indices[0];
  uint i1 = tri.vertex_indices[1];
  uint i2 = tri.vertex_indices[2];

  SegmentMeshletVertex v0 = segment_meshlet_vertices[i0];
  SegmentMeshletVertex v1 = segment_meshlet_vertices[i1];
  SegmentMeshletVertex v2 = segment_meshlet_vertices[i2];
  segment_index = v0.segment_index;

  float3 p0 = v0.x;
  float3 p1 = v1.x;
  float3 p2 = v2.x;
  if (TriangleAreaNearlyZero(p0, p1, p2)) {
    return 0.0;
  }
  return TriangleSignedVolumeContribution(p0, p1, p2);
}

public float3 KineticSegmentVolumeChangeHeatmapColor(uint segment_index) {
  float initial_volume = kinetic_segment_initial_volumes[segment_index];
  float current_volume = kinetic_segment_volumes[segment_index];
  return VolumeChangeHeatmapColor(initial_volume, current_volume);
}
