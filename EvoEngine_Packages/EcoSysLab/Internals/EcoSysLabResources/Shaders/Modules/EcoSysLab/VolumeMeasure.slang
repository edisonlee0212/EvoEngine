// Shared volume primitives for DynamicStrands GPU volume measurement.

public static const float VOLUME_MEASURE_EPSILON = 1e-18;
public static const float VOLUME_CHANGE_HEATMAP_MAX_ABS_PERCENT = 30.0;

public struct VolumeMeasureResult {
  public float cumulative_volume;
  public float initial_cumulative_volume;
  public uint frame_index;
  public uint padding0;
};

// Absolute tetrahedron volume |scalar triple| / 6.
public float TetAbsoluteVolume(float3 a, float3 b, float3 c, float3 d) {
  float3 u = b - a;
  float3 v = c - a;
  float3 w = d - a;
  float triple = dot(u, cross(v, w));
  float volume = abs(triple) / 6.0;
  return volume < VOLUME_MEASURE_EPSILON ? 0.0 : volume;
}

// Divergence contribution of one oriented triangle: (1/6) p0 · (p1 × p2).
public float TriangleSignedVolumeContribution(float3 p0, float3 p1, float3 p2) {
  return dot(p0, cross(p1, p2)) / 6.0;
}

public bool TriangleAreaNearlyZero(float3 p0, float3 p1, float3 p2) {
  float3 c = cross(p1 - p0, p2 - p0);
  return dot(c, c) <= VOLUME_MEASURE_EPSILON;
}

public float VolumePercentChange(float initial_volume, float current_volume) {
  if (initial_volume > VOLUME_MEASURE_EPSILON) {
    return 100.0 * (current_volume - initial_volume) / initial_volume;
  }
  if (current_volume > VOLUME_MEASURE_EPSILON) {
    return VOLUME_CHANGE_HEATMAP_MAX_ABS_PERCENT;
  }
  return 0.0;
}

// White at 0%, blue for gains, red for losses.
public float3 VolumeChangeHeatmapColor(float initial_volume, float current_volume, float max_abs_percent) {
  float percent = VolumePercentChange(initial_volume, current_volume);
  float clamp_abs = max(1e-6, max_abs_percent);
  float t = clamp(percent / clamp_abs, -1.0, 1.0);
  float3 white = float3(1.0, 1.0, 1.0);
  if (t >= 0.0) {
    return lerp(white, float3(0.15, 0.35, 1.0), t);
  }
  return lerp(white, float3(1.0, 0.12, 0.12), -t);
}

public float3 VolumeChangeHeatmapColor(float initial_volume, float current_volume) {
  return VolumeChangeHeatmapColor(initial_volume, current_volume, VOLUME_CHANGE_HEATMAP_MAX_ABS_PERCENT);
}
