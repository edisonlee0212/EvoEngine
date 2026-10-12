// @evoengine-dialect native
/*
Copyright (c) 2022, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
// NRD 4.18 (7033ffbc): radiance and normal packing for R10G10B10A2 normals and linear roughness.
static const float NRD_FP16_MAX = 65504.0;
bool _NRD_IsInvalid(float value) {
  return !isfinite(value);
}
bool _NRD_IsInvalid(float3 value) {
  return !all(isfinite(value));
}

float3 _NRD_SafeNormalize(float3 v) {
  return v * rsqrt(dot(v, v) + 1e-9);
}

float3 _NRD_LinearToYCoCg(float3 color) {
  float Y = dot(color, float3(0.25, 0.5, 0.25));
  float Co = dot(color, float3(0.5, 0.0, -0.5));
  float Cg = dot(color, float3(-0.25, 0.5, -0.25));

  return float3(Y, Co, Cg);
}

float4 _NRD_DecodeNormalRoughness101010(float3 p) {
  float t = p.z * 2.0 - 1.0;

  float4 r;
  r.x = p.x - p.y;
  r.y = p.x + p.y - 1.0;
  r.z = t < 0 ? -1 : 1;
  r.z *= 1.0 - abs(r.x) - abs(r.y);

  r.w = abs(t);

  return r;
}

float _NRD_GetSpecMagicCurve(float roughness, float power) {
  float f = 1.0 - exp2(-200.0 * roughness * roughness);
  f *= pow(saturate(roughness), power);

  return f;
}

float _REBLUR_GetHitDistanceNormalization(float viewZ, float3 hitDistParams, float roughness) {
  float smc = _NRD_GetSpecMagicCurve(roughness, 0.5);

  return (hitDistParams.x + abs(viewZ) * hitDistParams.y) * lerp(hitDistParams.z, 1.0, smc);
}

public float REBLUR_FrontEnd_GetNormHitDist(float hitDist, float viewZ, float3 hitDistParams, float roughness) {
  float f = _REBLUR_GetHitDistanceNormalization(viewZ, hitDistParams, roughness);

  return saturate(hitDist / f);
}

public float4 REBLUR_FrontEnd_PackRadianceAndNormHitDist(float3 radiance, float normHitDist, bool sanitize) {
  if (sanitize) {
    radiance = _NRD_IsInvalid(radiance) ? float3(0, 0, 0) : clamp(radiance, 0, NRD_FP16_MAX);
    normHitDist = _NRD_IsInvalid(normHitDist) ? 0 : saturate(normHitDist);
  }

  radiance = _NRD_LinearToYCoCg(radiance);

  return float4(radiance, normHitDist);
}

public float4 NRD_FrontEnd_UnpackNormalAndRoughness(float4 packed) {
  float4 decoded = _NRD_DecodeNormalRoughness101010(packed.xyz);
  decoded.xyz = _NRD_SafeNormalize(decoded.xyz);
  return decoded;
}
