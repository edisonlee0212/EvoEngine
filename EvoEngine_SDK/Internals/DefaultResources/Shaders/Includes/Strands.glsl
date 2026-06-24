
int EE_STRANDS_SEGMENT_SUBDIVISION(in vec3 worldPosA, in vec3 worldPosB) {
  vec4 coordA = EE_CAMERAS[EE_CAMERA_INDEX].projection_view * vec4(worldPosA, 1.0);
  vec4 coordB = EE_CAMERAS[EE_CAMERA_INDEX].projection_view * vec4(worldPosB, 1.0);
  vec2 screenSize = vec2(EE_CAMERA_RESOLUTION_X(EE_CAMERA_INDEX), EE_CAMERA_RESOLUTION_Y(EE_CAMERA_INDEX));
  if (abs(coordA.w) <= 1.0e-6 || abs(coordB.w) <= 1.0e-6) {
    return 1;
  }
  coordA = coordA / coordA.w;
  coordB = coordB / coordB.w;
  if (any(isnan(coordA)) || any(isnan(coordB)) || any(isinf(coordA)) || any(isinf(coordB))) {
    return 1;
  }
  if (coordA.z < -1.0 && coordB.z < -1.0)
    return 0;
  float pixelDistance = distance(coordA.xy * screenSize / 2.0, coordB.xy * screenSize / 2.0);
  if (isnan(pixelDistance) || isinf(pixelDistance)) {
    return 1;
  }
  return max(1, min(EE_RENDER_INFO.strand_subdivision_max_x, int(pixelDistance / EE_RENDER_INFO.strand_subdivision_x)));
}

vec3 EE_SAFE_NORMALIZE(in vec3 value, in vec3 fallback) {
  if (any(isnan(value)) || any(isinf(value))) {
    return fallback;
  }
  float len = length(value);
  if (len <= 1.0e-8 || isnan(len) || isinf(len)) {
    return fallback;
  }
  return value / len;
}

vec3 EE_STRANDS_SIDE_VECTOR(in vec3 normal, in vec3 tangent) {
  vec3 n = EE_SAFE_NORMALIZE(normal, vec3(0.0, 1.0, 0.0));
  vec3 t = EE_SAFE_NORMALIZE(tangent, vec3(0.0, 0.0, 1.0));
  vec3 side = cross(n, t);
  if (any(isnan(side)) || any(isinf(side)) || length(side) <= 1.0e-8) {
    vec3 reference = abs(n.y) < 0.9 ? vec3(0.0, 1.0, 0.0) : vec3(1.0, 0.0, 0.0);
    side = cross(n, reference);
  }
  return EE_SAFE_NORMALIZE(side, vec3(1.0, 0.0, 0.0));
}

bool EE_STRAND_PROFILE_IS_SEMIELLIPSE(in vec4 profile) {
  return profile.z > 0.5 && profile.x > 0.0 && profile.y > 0.0 && !any(isnan(profile)) && !any(isinf(profile));
}

float EE_STRAND_PROFILE_EFFECTIVE_RADIUS(in vec4 profile, in float fallbackThickness) {
  if (EE_STRAND_PROFILE_IS_SEMIELLIPSE(profile)) {
    return max(max(profile.x, profile.y), fallbackThickness);
  }
  return fallbackThickness;
}

vec2 EE_STRAND_PROFILE_OFFSET(in vec4 profile, in float fallbackThickness, in float u) {
  float safeU = clamp(u, 0.0, 1.0);
  if (EE_STRAND_PROFILE_IS_SEMIELLIPSE(profile)) {
    float widthRadius = max(profile.x, 1.0e-8);
    float thicknessRadius = max(profile.y, 1.0e-8);
    if (safeU <= 0.5) {
      float flatU = safeU * 2.0;
      return vec2(mix(widthRadius, -widthRadius, flatU), 0.0);
    }
    float arcU = (safeU - 0.5) * 2.0;
    float theta = 3.14159265358979323846 * (1.0 - arcU);
    return vec2(widthRadius * cos(theta), thicknessRadius * sin(theta));
  }

  float angle = 6.28318530717958647692 * safeU;
  return vec2(cos(-angle), sin(-angle)) * fallbackThickness;
}

vec2 EE_STRAND_PROFILE_NORMAL(in vec4 profile, in float u) {
  float safeU = clamp(u, 0.0, 1.0);
  if (EE_STRAND_PROFILE_IS_SEMIELLIPSE(profile)) {
    if (safeU <= 0.5) {
      return vec2(0.0, -1.0);
    }
    float arcU = (safeU - 0.5) * 2.0;
    float theta = 3.14159265358979323846 * (1.0 - arcU);
    return normalize(vec2(cos(theta) / max(profile.x, 1.0e-8), sin(theta) / max(profile.y, 1.0e-8)));
  }

  float angle = 6.28318530717958647692 * safeU;
  return normalize(vec2(cos(-angle), sin(-angle)));
}

void EE_SPLINE_INTERPOLATION(in vec3 v0, in vec3 v1, in vec3 v2, in vec3 v3, out vec3 result, out vec3 tangent,
                             float u) {
  vec3 p0 = (v2 + v0) / 6.0 + v1 * (4.0 / 6.0);
  vec3 p1 = v2 - v0;
  vec3 p2 = v2 - v1;
  vec3 p3 = v3 - v1;
  float uu = u * u;
  float u3 = (1.0f / 6.0) * uu * u;
  vec3 q = vec3(u3 + 0.5 * (u - uu), uu - 4.0 * u3, u3);
  result = p0 + q.x * p1 + q.y * p2 + q.z * p3;
  if (u == 0.0)
    u = 0.000001;
  if (u == 1.0)
    u = 0.999999;
  float v = 1.0 - u;
  tangent = 0.5 * v * v * p1 + 2.0 * v * u * p2 + 0.5 * u * u * p3;
}

void EE_SPLINE_INTERPOLATION(in float v0, in float v1, in float v2, in float v3, out float result, out float tangent,
                             float u) {
  float p0 = (v2 + v0) / 6.0 + v1 * (4.0 / 6.0);
  float p1 = v2 - v0;
  float p2 = v2 - v1;
  float p3 = v3 - v1;
  float uu = u * u;
  float u3 = (1.0f / 6.0) * uu * u;
  vec3 q = vec3(u3 + 0.5 * (u - uu), uu - 4.0 * u3, u3);
  result = p0 + q.x * p1 + q.y * p2 + q.z * p3;
  if (u == 0.0)
    u = 0.000001;
  if (u == 1.0)
    u = 0.999999;
  float v = 1.0 - u;
  tangent = 0.5 * v * v * p1 + 2.0 * v * u * p2 + 0.5 * u * u * p3;
}

void EE_SPLINE_INTERPOLATION(in vec4 v0, in vec4 v1, in vec4 v2, in vec4 v3, out vec4 result, out vec4 tangent,
                             float u) {
  vec4 p0 = (v2 + v0) / 6.0 + v1 * (4.0 / 6.0);
  vec4 p1 = v2 - v0;
  vec4 p2 = v2 - v1;
  vec4 p3 = v3 - v1;
  float uu = u * u;
  float u3 = (1.0f / 6.0) * uu * u;
  vec3 q = vec3(u3 + 0.5 * (u - uu), uu - 4.0 * u3, u3);
  result = p0 + q.x * p1 + q.y * p2 + q.z * p3;
  if (u == 0.0)
    u = 0.000001;
  if (u == 1.0)
    u = 0.999999;
  float v = 1.0 - u;
  tangent = 0.5 * v * v * p1 + 2.0 * v * u * p2 + 0.5 * u * u * p3;
}

int EE_STRANDS_RING_SUBDIVISION(in mat4 model, in vec3 worldPos, in vec3 modelPos, in float thickness) {
  if (any(isnan(worldPos)) || any(isnan(modelPos)) || isnan(thickness) || any(isinf(worldPos)) || any(isinf(modelPos)) ||
      isinf(thickness)) {
    return 3;
  }
  vec3 modelPosB = thickness * vec3(0, 1, 0) + modelPos;
  vec3 endPointWorldPos = (model * vec4(modelPosB, 1.0)).xyz;
  vec3 redirectedWorldPosB = worldPos + EE_CAMERA_UP(EE_CAMERA_INDEX) * distance(worldPos, endPointWorldPos);
  float subdivision = EE_PIXEL_DISTANCE(EE_CAMERA_INDEX, worldPos, redirectedWorldPosB);
  if (isnan(subdivision) || isinf(subdivision)) {
    return 3;
  }
  return max(3, min(int(subdivision), EE_RENDER_INFO.strand_subdivision_max_y));
}
