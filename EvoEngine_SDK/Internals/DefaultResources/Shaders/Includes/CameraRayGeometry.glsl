#ifndef EE_CAMERA_RAY_GEOMETRY_GLSL
#define EE_CAMERA_RAY_GEOMETRY_GLSL

vec3 EE_CAMERA_SCALE_INDEPENDENT_NORMALIZE(const vec3 value, const vec3 fallback) {
  if (any(isnan(value)) || any(isinf(value))) {
    return fallback;
  }
  const float scale = max(max(abs(value.x), abs(value.y)), abs(value.z));
  if (scale == 0.0f) {
    return fallback;
  }
  const vec3 scaled_value = value / scale;
  return scaled_value * inversesqrt(dot(scaled_value, scaled_value));
}

vec3 EE_CAMERA_GEOMETRIC_NORMAL(const vec3 edge_1, const vec3 edge_2, const vec3 fallback) {
  const vec3 normalized_edge_1 = EE_CAMERA_SCALE_INDEPENDENT_NORMALIZE(edge_1, vec3(0.0f));
  const vec3 normalized_edge_2 = EE_CAMERA_SCALE_INDEPENDENT_NORMALIZE(edge_2, vec3(0.0f));
  return EE_CAMERA_SCALE_INDEPENDENT_NORMALIZE(cross(normalized_edge_1, normalized_edge_2), fallback);
}

#endif
