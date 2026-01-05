// Shared volume primitives for DynamicStrands GPU volume measurement.
// Include after Math / meshing buffer headers as needed.

#ifndef VOLUME_MEASURE_GLSL
#define VOLUME_MEASURE_GLSL

const float VOLUME_MEASURE_EPSILON = 1e-18;
const float VOLUME_CHANGE_HEATMAP_MAX_ABS_PERCENT = 30.0;

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

float VolumePercentChange(float initial_volume, float current_volume) {
  if (initial_volume > VOLUME_MEASURE_EPSILON) {
    return 100.0 * (current_volume - initial_volume) / initial_volume;
  }
  if (current_volume > VOLUME_MEASURE_EPSILON) {
    return VOLUME_CHANGE_HEATMAP_MAX_ABS_PERCENT;
  }
  return 0.0;
}

// White at 0%, blue for gains, red for losses.
// Display mapping saturates at +-max_abs_percent (default 30): larger changes still use the endpoint color.
vec3 VolumeChangeHeatmapColor(float initial_volume, float current_volume, float max_abs_percent) {
  float percent = VolumePercentChange(initial_volume, current_volume);
  float clamp_abs = max(1e-6, max_abs_percent);
  // t in [-1, 1] where |percent| >= clamp_abs maps to full red/blue.
  float t = clamp(percent / clamp_abs, -1.0, 1.0);
  vec3 white = vec3(1.0);
  if (t >= 0.0) {
    return mix(white, vec3(0.15, 0.35, 1.0), t);
  }
  return mix(white, vec3(1.0, 0.12, 0.12), -t);
}

vec3 VolumeChangeHeatmapColor(float initial_volume, float current_volume) {
  return VolumeChangeHeatmapColor(initial_volume, current_volume, VOLUME_CHANGE_HEATMAP_MAX_ABS_PERCENT);
}

// GPU readback layout for cumulative volume (percent resolved on CPU).
struct VolumeMeasureResult {
  float cumulative_volume;
  float initial_cumulative_volume;
  uint frame_index;
  uint padding0;
};

#endif  // VOLUME_MEASURE_GLSL
