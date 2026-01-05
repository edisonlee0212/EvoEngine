import EcoSysLab.AlphaShapeMeshing;
import EcoSysLab.AlphaShapeMeshingSet0;
import EcoSysLab.VolumeMeasure;

// Bindings (strands set 0) — kept clear of DynamicStrandsSet0 particle/biology slots 10–13:
//   16 = current per-tet volumes (float[])
//   17 = initial per-tet volumes (float[])
//   19 = near-degenerate-at-init flags (uint[], 1 = exclude)
//   20 = VolumeMeasureResult (alpha)

[[vk::binding(16, 0)]] public RWStructuredBuffer<float, Std430DataLayout> alpha_tet_current_volumes;
[[vk::binding(17, 0)]] public StructuredBuffer<float, Std430DataLayout> alpha_tet_initial_volumes;
[[vk::binding(19, 0)]] public StructuredBuffer<uint, Std430DataLayout> alpha_near_degenerate;
[[vk::binding(20, 0)]] public RWStructuredBuffer<VolumeMeasureResult, Std430DataLayout> alpha_volume_result;

// Returns absolute volume of tet `tet_id`, or 0 if dead / invalid / near-degenerate at init.
public float MeasureAlphaTetrahedronVolume(uint tet_id) {
  DelaunayTetrahedron tet = delaunay_tetrahedrons[tet_id];
  if (tet.inside != 1) {
    return 0.0;
  }
  if (alpha_near_degenerate[tet_id] != 0) {
    return 0.0;
  }

  for (int c = 0; c < 4; ++c) {
    if (tet.indices[c] < 0) {
      return 0.0;
    }
  }

  float3 a = uniform_particles[tet.indices[0]].position_t.xyz;
  float3 b = uniform_particles[tet.indices[1]].position_t.xyz;
  float3 c = uniform_particles[tet.indices[2]].position_t.xyz;
  float3 d = uniform_particles[tet.indices[3]].position_t.xyz;
  return TetAbsoluteVolume(a, b, c, d);
}

public float3 AlphaTetVolumeChangeHeatmapColor(uint tet_id) {
  float initial_volume = alpha_tet_initial_volumes[tet_id];
  float current_volume = alpha_tet_current_volumes[tet_id];
  return VolumeChangeHeatmapColor(initial_volume, current_volume);
}
