import EvoEngine.Nrd;
import EvoEngine.CameraRayOutputTypes;
import EvoEngine.NrdTemporalAa;

[[vk::binding(0, 0)]] RWStructuredBuffer<float4> output;

[shader("compute")][numthreads(1, 1, 1)] void main(uint3 id : SV_DispatchThreadID) {
  const float4 packed = REBLUR_FrontEnd_PackRadianceAndNormHitDist(float3(2.0f, 4.0f, 6.0f), 0.5f, true);
  output[0] = packed;
  output[1] = NRD_FrontEnd_UnpackNormalAndRoughness(float4(0.5f, 0.5f, 0.25f, 0.0f));
  output[2] = float4(REBLUR_FrontEnd_GetNormHitDist(1.0f, 0.0f, float3(3.0f, 0.1f, 20.0f), 1.0f), 0, 0, 0);
  const float4 current = float4(0.5f, 0.5f, 0.75f, 0.0f);
  float4 geometry = float4(current.xyz, 10.0f);
  float4 color = float4(2.0f, 4.0f, 6.0f, 0.0f);
  output[3] = float4(EE_NRD_TAA_ACCEPT_HISTORY(current, 10.0f, geometry, color) ? 1.0f : 0.0f, 0, 0, 0);
  output[4] = float4(EE_NRD_TAA_ACCEPT_HISTORY(current, 12.0f, geometry, color) ? 1.0f : 0.0f, 0, 0, 0);
  geometry.z = 0.25f;
  output[5] = float4(EE_NRD_TAA_ACCEPT_HISTORY(current, 10.0f, geometry, color) ? 1.0f : 0.0f, 0, 0, 0);
  geometry.z = current.z;
  color.w = 1.0f / 3.0f;
  output[6] = float4(EE_NRD_TAA_ACCEPT_HISTORY(current, 10.0f, geometry, color) ? 1.0f : 0.0f, 0, 0, 0);
  color.w = 0.0f;
  output[7] = float4(EE_NRD_TAA_ACCEPT_HISTORY(current, 1e8f, geometry, color) ? 1.0f : 0.0f, 0, 0, 0);
  color.x = asfloat(0x7fc00000u);
  output[8] = float4(EE_NRD_TAA_ACCEPT_HISTORY(current, 10.0f, geometry, color) ? 1.0f : 0.0f, 0, 0, 0);
  color.x = 2.0f;
  geometry.z = 0.9f;
  output[9] = float4(EE_NRD_TAA_ACCEPT_HISTORY(current, 10.0f, geometry, color) ? 1.0f : 0.0f, 0, 0, 0);
  geometry.z = current.z;
  output[10] = float4(EE_NRD_TAA_ACCEPT_HISTORY(current, -10.0f, geometry, color) ? 1.0f : 0.0f, 0, 0, 0);
  output[11] = EE_NRD_TAA_UNPACK_GEOMETRY(EE_NRD_TAA_PACK_GEOMETRY(current.xyz, 123456.75f));
  output[12] = float4(EE_CAMERA_NRD_ROUGHNESS(float2(0.25f)), EE_CAMERA_NRD_ROUGHNESS(float2(0.04f, 0.64f)),
                      EE_CAMERA_NRD_ROUGHNESS(float2(0.0f)), EE_CAMERA_NRD_ROUGHNESS(float2(1.0f)));
}
