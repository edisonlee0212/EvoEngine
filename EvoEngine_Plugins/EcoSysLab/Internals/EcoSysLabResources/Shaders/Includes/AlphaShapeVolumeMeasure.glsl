// Alpha Shape tet volume contribution.
// Requires: AlphaShapeMeshing.glsl, VolumeMeasure.glsl
// Bindings (DYNAMIC_STRANDS_SET):
//   12 = near-degenerate-at-init flags (uint[], 1 = exclude)
//   13 = VolumeMeasureResult (alpha)
//   16 = current per-tet volumes (float[])
//   17 = initial per-tet volumes (float[])

#ifndef ALPHA_SHAPE_VOLUME_MEASURE_GLSL
#define ALPHA_SHAPE_VOLUME_MEASURE_GLSL

layout(std430, set = DYNAMIC_STRANDS_SET, binding = 12) readonly buffer ALPHA_NEAR_DEGENERATE_BLOCK {
  uint alpha_near_degenerate[];
};

layout(std430, set = DYNAMIC_STRANDS_SET, binding = 13) buffer ALPHA_VOLUME_RESULT_BLOCK {
  VolumeMeasureResult alpha_volume_result;
};

layout(std430, set = DYNAMIC_STRANDS_SET, binding = 16) buffer ALPHA_TET_CURRENT_VOLUMES_BLOCK {
  float alpha_tet_current_volumes[];
};

layout(std430, set = DYNAMIC_STRANDS_SET, binding = 17) readonly buffer ALPHA_TET_INITIAL_VOLUMES_BLOCK {
  float alpha_tet_initial_volumes[];
};

// Returns absolute volume of tet `tet_id`, or 0 if dead / invalid / near-degenerate at init.
float MeasureAlphaTetrahedronVolume(uint tet_id) {
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

  // Current deformed particle positions (prediction updates position_t).
  vec3 a = uniform_particles[tet.indices[0]].position_t.xyz;
  vec3 b = uniform_particles[tet.indices[1]].position_t.xyz;
  vec3 c = uniform_particles[tet.indices[2]].position_t.xyz;
  vec3 d = uniform_particles[tet.indices[3]].position_t.xyz;
  return TetAbsoluteVolume(a, b, c, d);
}

vec3 AlphaTetVolumeChangeHeatmapColor(uint tet_id) {
  float initial_volume = alpha_tet_initial_volumes[tet_id];
  float current_volume = alpha_tet_current_volumes[tet_id];
  return VolumeChangeHeatmapColor(initial_volume, current_volume);
}

#endif  // ALPHA_SHAPE_VOLUME_MEASURE_GLSL
