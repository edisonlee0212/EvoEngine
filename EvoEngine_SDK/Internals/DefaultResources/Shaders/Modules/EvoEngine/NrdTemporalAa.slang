import EvoEngine.Nrd;

public bool EE_NRD_TAA_ACCEPT_HISTORY(const float4 current_packed, const float expected_depth,
                                      const float4 old_geometry, const float4 old_color) {
  if (expected_depth <= 0.0f || expected_depth >= 1e8f || !all(isfinite(old_geometry)) ||
      !all(isfinite(old_color))) return false;
  const float4 current = NRD_FrontEnd_UnpackNormalAndRoughness(current_packed);
  const float4 previous = NRD_FrontEnd_UnpackNormalAndRoughness(float4(old_geometry.xyz, old_color.w));
  return abs(old_color.w - current_packed.w) <= 0.1f &&
         abs(old_geometry.w - expected_depth) <= max(0.01f, expected_depth * 0.01f) &&
         dot(previous.xyz, current.xyz) >= 0.95f && abs(previous.w - current.w) <= 0.1f;
}

public uint2 EE_NRD_TAA_PACK_GEOMETRY(const float3 packed, const float depth) {
  const uint3 normal = uint3(round(saturate(packed) * 1023.0f));
  return uint2(asuint(depth), normal.x | (normal.y << 10u) | (normal.z << 20u));
}

public float4 EE_NRD_TAA_UNPACK_GEOMETRY(const uint2 geometry) {
  const uint3 normal = (geometry.y >> uint3(0u, 10u, 20u)) & 1023u;
  return float4(float3(normal) / 1023.0f, asfloat(geometry.x));
}
