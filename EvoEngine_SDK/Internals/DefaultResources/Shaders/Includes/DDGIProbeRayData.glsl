#ifndef EE_DDGI_PROBE_RAY_DATA_GLSL
#define EE_DDGI_PROBE_RAY_DATA_GLSL

const float EE_DDGI_PROBE_RAY_MISS_DISTANCE = 1e27f;
const float EE_DDGI_PROBE_RAY_INACTIVE_DISTANCE = -1e27f;
const float EE_DDGI_PROBE_RAY_BACKFACE_DISTANCE_SCALE = -0.2f;
const float EE_DDGI_PROBE_RAY_SENTINEL_THRESHOLD = 5e26f;

struct DdgiProbeRayData {
  vec4 radiance_and_signed_distance;
};

vec3 EE_DDGI_PROBE_RAY_RADIANCE(const DdgiProbeRayData ray_data) {
  return max(ray_data.radiance_and_signed_distance.rgb, vec3(0.0f));
}

float EE_DDGI_PROBE_RAY_SIGNED_DISTANCE(const DdgiProbeRayData ray_data) {
  return ray_data.radiance_and_signed_distance.w;
}

bool EE_DDGI_PROBE_RAY_INACTIVE(const DdgiProbeRayData ray_data) {
  return EE_DDGI_PROBE_RAY_SIGNED_DISTANCE(ray_data) <= -EE_DDGI_PROBE_RAY_SENTINEL_THRESHOLD;
}

bool EE_DDGI_PROBE_RAY_MISS(const DdgiProbeRayData ray_data) {
  return EE_DDGI_PROBE_RAY_SIGNED_DISTANCE(ray_data) >= EE_DDGI_PROBE_RAY_SENTINEL_THRESHOLD;
}

bool EE_DDGI_PROBE_RAY_BACKFACE_HIT(const DdgiProbeRayData ray_data) {
  const float signed_distance = EE_DDGI_PROBE_RAY_SIGNED_DISTANCE(ray_data);
  return !EE_DDGI_PROBE_RAY_INACTIVE(ray_data) && (floatBitsToUint(signed_distance) & 0x80000000u) != 0u;
}

bool EE_DDGI_PROBE_RAY_FRONTFACE_HIT(const DdgiProbeRayData ray_data) {
  return !EE_DDGI_PROBE_RAY_MISS(ray_data) && !EE_DDGI_PROBE_RAY_INACTIVE(ray_data) &&
         !EE_DDGI_PROBE_RAY_BACKFACE_HIT(ray_data);
}

bool EE_DDGI_PROBE_RAY_HIT(const DdgiProbeRayData ray_data) {
  return EE_DDGI_PROBE_RAY_FRONTFACE_HIT(ray_data) || EE_DDGI_PROBE_RAY_BACKFACE_HIT(ray_data);
}

#endif
