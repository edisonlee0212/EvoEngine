// @evoengine-dialect native
public
static const float EE_DDGI_PROBE_RAY_MISS_DISTANCE = 1e27f;
public
static const float EE_DDGI_PROBE_RAY_INACTIVE_DISTANCE = -1e27f;
public
static const float EE_DDGI_PROBE_RAY_BACKFACE_DISTANCE_SCALE = -0.2f;
public
static const float EE_DDGI_PROBE_RAY_SENTINEL_THRESHOLD = 5e26f;

public
struct DdgiProbeRayData {
 public
  float4 radiance_and_signed_distance;
};

public
struct DdgiProbeRaySampleInfo {
 public
  uint2 packed_direction_and_inverse_pdf;
};

public
float2 EE_DDGI_OCTAHEDRAL_SIGN_NOT_ZERO(float2 value) {
  return float2(value.x >= 0.0f ? 1.0f : -1.0f, value.y >= 0.0f ? 1.0f : -1.0f);
}

public
uint EE_DDGI_PACK_OCTAHEDRAL_DIRECTION(float3 direction) {
  float denominator = abs(direction.x) + abs(direction.y) + abs(direction.z);
  float3 octahedral = denominator > 0.0f && isfinite(denominator) ? direction / denominator : float3(0.0f, 0.0f, 1.0f);
  float2 encoded = octahedral.xy;
  if (octahedral.z < 0.0f) {
    encoded = (1.0f - abs(encoded.yx)) * EE_DDGI_OCTAHEDRAL_SIGN_NOT_ZERO(encoded);
  }
  return packSnorm2x16(clamp(encoded, float2(-1.0f), float2(1.0f)));
}

public
float3 EE_DDGI_UNPACK_OCTAHEDRAL_DIRECTION(uint packed_direction) {
  float2 encoded = unpackSnorm2x16ToFloat(packed_direction);
  float3 direction = float3(encoded, 1.0f - abs(encoded.x) - abs(encoded.y));
  if (direction.z < 0.0f) {
    direction.xy = (1.0f - abs(direction.yx)) * EE_DDGI_OCTAHEDRAL_SIGN_NOT_ZERO(direction.xy);
  }
  return normalize(direction);
}

public
DdgiProbeRaySampleInfo EE_DDGI_PACK_PROBE_RAY_SAMPLE_INFO(float3 direction, float inverse_mixture_pdf) {
  DdgiProbeRaySampleInfo result;
  result.packed_direction_and_inverse_pdf =
      uint2(EE_DDGI_PACK_OCTAHEDRAL_DIRECTION(direction), asuint(inverse_mixture_pdf));
  return result;
}

public
float3 EE_DDGI_PROBE_RAY_DIRECTION(DdgiProbeRaySampleInfo sample_info) {
  return EE_DDGI_UNPACK_OCTAHEDRAL_DIRECTION(sample_info.packed_direction_and_inverse_pdf.x);
}

public
float EE_DDGI_PROBE_RAY_INVERSE_PDF(DdgiProbeRaySampleInfo sample_info) {
  return asfloat(sample_info.packed_direction_and_inverse_pdf.y);
}

public
struct DdgiEmissiveGuideInfo {
 public
  float4 center_and_radius;
 public
  float4 power_and_reserved;
};

public
float3 EE_DDGI_PROBE_RAY_RADIANCE(DdgiProbeRayData ray_data) {
  return max(ray_data.radiance_and_signed_distance.rgb, float3(0.0f));
}

public
float EE_DDGI_PROBE_RAY_SIGNED_DISTANCE(DdgiProbeRayData ray_data) {
  return ray_data.radiance_and_signed_distance.w;
}

public
bool EE_DDGI_PROBE_RAY_INACTIVE(DdgiProbeRayData ray_data) {
  return EE_DDGI_PROBE_RAY_SIGNED_DISTANCE(ray_data) <= -EE_DDGI_PROBE_RAY_SENTINEL_THRESHOLD;
}

public
bool EE_DDGI_PROBE_RAY_MISS(DdgiProbeRayData ray_data) {
  return EE_DDGI_PROBE_RAY_SIGNED_DISTANCE(ray_data) >= EE_DDGI_PROBE_RAY_SENTINEL_THRESHOLD;
}

public
bool EE_DDGI_PROBE_RAY_BACKFACE_HIT(DdgiProbeRayData ray_data) {
  float signed_distance = EE_DDGI_PROBE_RAY_SIGNED_DISTANCE(ray_data);
  return !EE_DDGI_PROBE_RAY_INACTIVE(ray_data) && (asuint(signed_distance) & 0x80000000u) != 0u;
}

public
bool EE_DDGI_PROBE_RAY_FRONTFACE_HIT(DdgiProbeRayData ray_data) {
  return !EE_DDGI_PROBE_RAY_MISS(ray_data) && !EE_DDGI_PROBE_RAY_INACTIVE(ray_data) &&
         !EE_DDGI_PROBE_RAY_BACKFACE_HIT(ray_data);
}

public
bool EE_DDGI_PROBE_RAY_HIT(DdgiProbeRayData ray_data) {
  return EE_DDGI_PROBE_RAY_FRONTFACE_HIT(ray_data) || EE_DDGI_PROBE_RAY_BACKFACE_HIT(ray_data);
}
