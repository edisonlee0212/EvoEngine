[[vk::binding(7, 1)]] RWStructuredBuffer<uint2> EE_DDGI_IRRADIANCE_RING;
[[vk::binding(8, 1)]] RWStructuredBuffer<uint4> EE_DDGI_IRRADIANCE_SUM;
[[vk::binding(9, 1)]] RWStructuredBuffer<float4> EE_DDGI_HISTORY_ORIGINS;
[[vk::binding(10, 1)]] RWStructuredBuffer<uint> EE_DDGI_VISIBILITY_RING;
[[vk::binding(11, 1)]] RWStructuredBuffer<uint2> EE_DDGI_VISIBILITY_SUM;

uint2 EE_DDGI_PACK_HISTORY(uint4 value) {
  return uint2(value.x | (value.y << 16u), value.z | (value.w << 16u));
}

uint4 EE_DDGI_UNPACK_HISTORY(uint2 value) {
  return uint4(value.x & 65535u, value.x >> 16u, value.y & 65535u, value.y >> 16u);
}

void EE_DDGI_CLEAR_IRRADIANCE_HISTORY(uint texel, uint count) {
  for (uint slot = 0u; slot < count; ++slot)
    EE_DDGI_IRRADIANCE_RING[texel * count + slot] = uint2(0u);
  EE_DDGI_IRRADIANCE_SUM[texel] = uint4(0u);
}

// Linear bounded samples are averaged before atlas gamma encoding.
float4 EE_DDGI_ROLL_IRRADIANCE(uint texel, uint count, uint phase, float4 sample, bool reset) {
  if (reset)
    EE_DDGI_CLEAR_IRRADIANCE_HISTORY(texel, count);
  uint4 sum = EE_DDGI_IRRADIANCE_SUM[texel];
  if (sample.w > 0.0f && all(isfinite(sample))) {
    uint slot = texel * count + phase;
    uint4 value = uint4(uint3(floor(clamp(sample.xyz / 64.0f, 0.0f, 1.0f) * 65535.0f + 0.5f)), 1u);
    sum = sum - EE_DDGI_UNPACK_HISTORY(EE_DDGI_IRRADIANCE_RING[slot]) + value;
    EE_DDGI_IRRADIANCE_RING[slot] = EE_DDGI_PACK_HISTORY(value);
    EE_DDGI_IRRADIANCE_SUM[texel] = sum;
  }
  return float4(float3(sum.xyz) * (64.0f / (65535.0f * float(count))), sum.w > 0u ? 1.0f : 0.0f);
}

void EE_DDGI_CLEAR_VISIBILITY_HISTORY(uint texel, uint count) {
  for (uint slot = 0u; slot < count; ++slot)
    EE_DDGI_VISIBILITY_RING[texel * count + slot] = 0u;
  EE_DDGI_VISIBILITY_SUM[texel] = uint2(0u);
}

float2 EE_DDGI_ROLL_VISIBILITY(uint texel, uint count, uint phase, float2 sample, float distance_bound, bool reset) {
  if (reset) EE_DDGI_CLEAR_VISIBILITY_HISTORY(texel, count);
  float2 bounds = float2(distance_bound, distance_bound * distance_bound) * 0.5f;
  uint2 sum = EE_DDGI_VISIBILITY_SUM[texel];
  if (all(isfinite(sample)) && all(sample >= 0.0f)) {
    uint slot = texel * count + phase;
    uint previous = EE_DDGI_VISIBILITY_RING[slot];
    uint2 value = uint2(floor(saturate(sample / bounds) * 65535.0f + 0.5f));
    sum = sum - uint2(previous & 65535u, previous >> 16u) + value;
    EE_DDGI_VISIBILITY_RING[slot] = value.x | (value.y << 16u);
    EE_DDGI_VISIBILITY_SUM[texel] = sum;
  }
  return float2(sum) * (bounds / (65535.0f * float(count)));
}
