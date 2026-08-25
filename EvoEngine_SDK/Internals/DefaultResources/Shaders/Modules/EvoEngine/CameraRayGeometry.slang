import EvoEngine.RayTracingBasic;

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

#if EE_RAY_TRACING_LINEAR_SWEPT_SPHERES_SUPPORTED
public void EE_CAMERA_LSS_LOAD_POINTS(const Instance instance, const uint primitive_id,
                                      out RayTracingStrandPoint point_0, out RayTracingStrandPoint point_1) {
  const uint primitive_index = uint(instance.ray_tracing_geometry.y) + primitive_id;
  const uint point_index = EE_RAY_TRACING_STRAND_INDICES[primitive_index];
  point_0 = EE_RAY_TRACING_STRAND_POINTS[point_index];
  point_1 = EE_RAY_TRACING_STRAND_POINTS[point_index + 1u];
}

public float3 EE_CAMERA_LSS_OBJECT_GEOMETRIC_NORMAL(const float3 position_0, const float3 position_1,
                                                     const float parameter, const float3 object_hit,
                                                     const float3 object_ray_direction) {
  const float3 segment = position_1 - position_0;
  const float3 fallback = EE_CAMERA_SCALE_INDEPENDENT_NORMALIZE(
      cross(segment, object_ray_direction), float3(0.0f, 1.0f, 0.0f));
  return EE_CAMERA_SCALE_INDEPENDENT_NORMALIZE(object_hit - lerp(position_0, position_1, parameter), fallback);
}
#endif

public float3 EE_CAMERA_GEOMETRIC_NORMAL(const float3 edge_1, const float3 edge_2, const float3 fallback) {
  const float3 normalized_edge_1 = EE_CAMERA_SCALE_INDEPENDENT_NORMALIZE(edge_1, float3(0.0f));
  const float3 normalized_edge_2 = EE_CAMERA_SCALE_INDEPENDENT_NORMALIZE(edge_2, float3(0.0f));
  return EE_CAMERA_SCALE_INDEPENDENT_NORMALIZE(cross(normalized_edge_1, normalized_edge_2), fallback);
}
