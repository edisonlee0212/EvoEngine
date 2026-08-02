// @evoengine-dialect native
public float3 EE_CAMERA_SCALE_INDEPENDENT_NORMALIZE(const float3 value, const float3 fallback) {
  if (any(isnan(value)) || any(isinf(value))) {
    return fallback;
  }
  const float scale = max(max(abs(value.x), abs(value.y)), abs(value.z));
  if (scale == 0.0f) {
    return fallback;
  }
  const float3 scaled_value = value / scale;
  return scaled_value * rsqrt(dot(scaled_value, scaled_value));
}

public float3 EE_CAMERA_GEOMETRIC_NORMAL(const float3 edge_1, const float3 edge_2, const float3 fallback) {
  const float3 normalized_edge_1 = EE_CAMERA_SCALE_INDEPENDENT_NORMALIZE(edge_1, float3(0.0f));
  const float3 normalized_edge_2 = EE_CAMERA_SCALE_INDEPENDENT_NORMALIZE(edge_2, float3(0.0f));
  return EE_CAMERA_SCALE_INDEPENDENT_NORMALIZE(cross(normalized_edge_1, normalized_edge_2), fallback);
}
