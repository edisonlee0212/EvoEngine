// @evoengine-dialect native
public static const float EE_DDGI_PROBE_RAY_MISS_DISTANCE = 1e27f;
public static const float EE_DDGI_PROBE_RAY_INACTIVE_DISTANCE = -1e27f;
public static const float EE_DDGI_PROBE_RAY_BACKFACE_DISTANCE_SCALE = -0.2f;
public static const float EE_DDGI_PROBE_RAY_SENTINEL_THRESHOLD = 5e26f;

public struct DdgiProbeRayData {
  public float4 radiance_and_signed_distance;
};

public float3 EE_DDGI_PROBE_RAY_RADIANCE(DdgiProbeRayData ray_data) {
  return max(ray_data.radiance_and_signed_distance.rgb, float3(0.0f));
}

public float EE_DDGI_PROBE_RAY_SIGNED_DISTANCE(DdgiProbeRayData ray_data) {
  return ray_data.radiance_and_signed_distance.w;
}

public bool EE_DDGI_PROBE_RAY_INACTIVE(DdgiProbeRayData ray_data) {
  return EE_DDGI_PROBE_RAY_SIGNED_DISTANCE(ray_data) <= -EE_DDGI_PROBE_RAY_SENTINEL_THRESHOLD;
}

public bool EE_DDGI_PROBE_RAY_MISS(DdgiProbeRayData ray_data) {
  return EE_DDGI_PROBE_RAY_SIGNED_DISTANCE(ray_data) >= EE_DDGI_PROBE_RAY_SENTINEL_THRESHOLD;
}

public bool EE_DDGI_PROBE_RAY_BACKFACE_HIT(DdgiProbeRayData ray_data) {
  float signed_distance = EE_DDGI_PROBE_RAY_SIGNED_DISTANCE(ray_data);
  return !EE_DDGI_PROBE_RAY_INACTIVE(ray_data) && (asuint(signed_distance) & 0x80000000u) != 0u;
}

public bool EE_DDGI_PROBE_RAY_FRONTFACE_HIT(DdgiProbeRayData ray_data) {
  return !EE_DDGI_PROBE_RAY_MISS(ray_data) && !EE_DDGI_PROBE_RAY_INACTIVE(ray_data) &&
         !EE_DDGI_PROBE_RAY_BACKFACE_HIT(ray_data);
}

public bool EE_DDGI_PROBE_RAY_HIT(DdgiProbeRayData ray_data) {
  return EE_DDGI_PROBE_RAY_FRONTFACE_HIT(ray_data) || EE_DDGI_PROBE_RAY_BACKFACE_HIT(ray_data);
}
