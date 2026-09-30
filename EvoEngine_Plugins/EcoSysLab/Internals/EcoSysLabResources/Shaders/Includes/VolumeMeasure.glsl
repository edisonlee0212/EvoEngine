// Shared volume primitives for DynamicStrands GPU volume measurement.
// Include after Math / meshing buffer headers as needed.

#ifndef VOLUME_MEASURE_GLSL
#define VOLUME_MEASURE_GLSL

const float VOLUME_MEASURE_EPSILON = 1e-18;

// Absolute tetrahedron volume |scalar triple| / 6.
float TetAbsoluteVolume(in vec3 a, in vec3 b, in vec3 c, in vec3 d) {
  vec3 u = b - a;
  vec3 v = c - a;
  vec3 w = d - a;
  float triple = dot(u, cross(v, w));
  float volume = abs(triple) / 6.0;
  return volume < VOLUME_MEASURE_EPSILON ? 0.0 : volume;
}

// Divergence contribution of one oriented triangle: (1/6) p0 · (p1 × p2).
float TriangleSignedVolumeContribution(in vec3 p0, in vec3 p1, in vec3 p2) {
  return dot(p0, cross(p1, p2)) / 6.0;
}

bool TriangleAreaNearlyZero(in vec3 p0, in vec3 p1, in vec3 p2) {
  vec3 c = cross(p1 - p0, p2 - p0);
  return dot(c, c) <= VOLUME_MEASURE_EPSILON;
}

// GPU readback / CSSO layout for cumulative volume (percent resolved on CPU).
struct VolumeMeasureResult {
  float cumulative_volume;
  float initial_cumulative_volume;
  uint frame_index;
  uint padding0;
};

#endif  // VOLUME_MEASURE_GLSL
